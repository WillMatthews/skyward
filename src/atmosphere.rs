//! Physically based sky (Hillaire 2020 style LUTs) and the sun's position.
//!
//! GPU: transmittance + multiple-scattering LUTs (rebuilt when the haze
//! changes), a per-frame sky-view LUT, aerial-perspective froxel volumes for
//! the main view and the mirrored water-reflection view, and an L1-SH sky
//! irradiance + exposure metering buffer. All scene, cloud and post shaders
//! read these, so sky colour, haze, ambient light, sun colour and exposure
//! always agree.
//!
//! CPU: solar position for a date/time/place and a matching transmittance
//! integrator (sun colour for the cloud renderer).

use glam::{vec3, Mat4, Vec3};

/// Top-of-atmosphere sun irradiance in scene units (exposure normalises it).
pub const SUN_E: f32 = 2.5;
/// Typical metered illuminance at noon in scene units (exposure reference).
const REF_E: f32 = 2.1;
const EXPOSURE_KEY: f32 = 1.15;

pub const TRANS_W: u32 = 256;
pub const TRANS_H: u32 = 64;
pub const MS_N: u32 = 32;
pub const SKY_W: u32 = 256;
pub const SKY_H: u32 = 108;
pub const AP_N: u32 = 32;
pub const LUT_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
/// Size of the `AtmosOut` storage buffer (bytes).
pub const OUT_SIZE: u64 = 4 * 16 + 16 + 16;
/// Skyglow table: azimuth bins x elevation bins (see `atmos_core.wgsl`).
pub const GLOW_AZ: usize = 16;
pub const GLOW_EL: usize = 8;

// ---------------------------------------------------------------- sun position

/// Direction toward the sun (world: +X east, -Z north, +Y up) for a local
/// clock time (`hours`, civil time with European-style summer time) on
/// `day_of_year` at `lat`/`lon` (degrees).
pub fn sun_direction(lat: f64, lon: f64, day_of_year: u32, hours: f32) -> Vec3 {
    sun_direction_zoned(lat, lon, civil_offset_h(lat, lon, day_of_year), day_of_year, hours)
}

/// As [`sun_direction`], with the clock in a given civil offset (hours from
/// UTC) — e.g. the start's zone while the aircraft flies elsewhere, so the
/// sun moves smoothly with the aircraft's position.
pub fn sun_direction_zoned(lat: f64, lon: f64, offset_h: f32, day_of_year: u32, hours: f32) -> Vec3 {
    let doy = day_of_year.clamp(1, 366) as f64;
    let utc = hours as f64 - offset_h as f64;
    let g = std::f64::consts::TAU / 365.0 * (doy - 1.0 + (utc - 12.0) / 24.0);
    let eqt = 229.18 * (0.000075 + 0.001868 * g.cos() - 0.032077 * g.sin() - 0.014615 * (2.0 * g).cos() - 0.040849 * (2.0 * g).sin());
    let decl = 0.006918 - 0.399912 * g.cos() + 0.070257 * g.sin() - 0.006758 * (2.0 * g).cos() + 0.000907 * (2.0 * g).sin()
        - 0.002697 * (3.0 * g).cos()
        + 0.00148 * (3.0 * g).sin();
    let tst = utc * 60.0 + eqt + 4.0 * lon;
    let ha = (tst / 4.0 - 180.0).to_radians();
    let la = lat.to_radians();
    let cz = (la.sin() * decl.sin() + la.cos() * decl.cos() * ha.cos()).clamp(-1.0, 1.0);
    let el = std::f64::consts::FRAC_PI_2 - cz.acos();
    // Azimuth clockwise from north.
    let az = ha.sin().atan2(ha.cos() * la.sin() - decl.tan() * la.cos()) + std::f64::consts::PI;
    let (e, n) = (az.sin() * el.cos(), az.cos() * el.cos());
    vec3(e as f32, el.sin() as f32, -n as f32).normalize()
}

/// The clock convention `sun_direction` uses: civil time zone ≈ nearest 15°
/// of longitude, +1 h summer time (late March .. late October, north).
pub fn civil_offset_h(lat: f64, lon: f64, day_of_year: u32) -> f32 {
    let doy = day_of_year.clamp(1, 366) as f64;
    let dst = if lat >= 0.0 { (86.0..=300.0).contains(&doy) } else { !(86.0..=300.0).contains(&doy) };
    ((lon / 15.0).round() + if dst { 1.0 } else { 0.0 }) as f32
}

/// Day of year for a month/day (non-leap year).
pub fn day_of_year(month: u32, day: u32) -> u32 {
    const CUM: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    CUM[(month.clamp(1, 12) - 1) as usize] + day.clamp(1, 31)
}

// ---------------------------------------------------------------- night sky (moon, stars)

/// Scene irradiance per lux (sun_e = SUN_E is ~128 klx at the top of the
/// atmosphere). The same factor converts luminance (cd/m²) to scene radiance.
pub const LUX: f32 = SUN_E / 128_000.0;

/// Moment and place for the night sky (moon, stars).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkyClock {
    pub lat: f64,
    pub lon: f64,
    pub year: i32,
    pub day_of_year: u32,
    /// Universal time (hours, may run past 0..24).
    pub utc_hours: f64,
    /// Local clock time (hours) — drives city life (windows, traffic).
    pub local_hours: f32,
}

impl Default for SkyClock {
    fn default() -> Self {
        SkyClock { lat: 51.5, lon: 0.0, year: 2026, day_of_year: 227, utc_hours: 14.5, local_hours: 15.5 }
    }
}

impl SkyClock {
    /// The clock `sun_direction_zoned` uses: local `hours` in the civil zone
    /// `offset_h`, at `lat`/`lon`, this year.
    pub fn new(lat: f64, lon: f64, offset_h: f32, day_of_year: u32, hours: f32) -> SkyClock {
        SkyClock { lat, lon, year: current_year(), day_of_year, utc_hours: hours as f64 - offset_h as f64, local_hours: hours }
    }

    /// Days since 2000 Jan 0.0 UT (Schlyter's `d`).
    pub fn days(&self) -> f64 {
        let y = self.year;
        let leaps = if y > 2000 { (y - 2001).div_euclid(4) + 1 } else { (y - 2000).div_euclid(4) };
        (365 * (y - 2000) + leaps) as f64 + self.day_of_year as f64 + self.utc_hours / 24.0
    }
}

/// The current calendar year (UTC) from the system clock.
pub fn current_year() -> i32 {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(1.8e9);
    // Exact enough away from New Year's Eve: 365.2425-day years.
    1970 + (secs / (365.2425 * 86400.0)).floor() as i32
}

fn rev(x: f64) -> f64 {
    x.rem_euclid(360.0)
}

/// Local sidereal time (degrees) for the clock.
pub fn local_sidereal_deg(c: &SkyClock) -> f64 {
    let d = c.days();
    let ls = rev(282.9404 + 4.70935e-5 * d + 356.0470 + 0.9856002585 * d);
    rev(ls + 180.0 + c.utc_hours * 15.0 + c.lon)
}

/// Rotation from equatorial coordinates (x → RA 0h, z → north celestial
/// pole; J2000, precession ignored) to the world frame (+X east, +Y up,
/// -Z north) at the clock's place and moment.
pub fn star_rotation(c: &SkyClock) -> glam::Mat3 {
    let lst = local_sidereal_deg(c).to_radians();
    let (sl, cl) = lst.sin_cos();
    let (sp, cp) = c.lat.to_radians().sin_cos();
    // Hour-angle frame: x' = cos(lst) x + sin(lst) y, y' = -sin(lst) x + cos(lst) y, z.
    // world.x (east) = y', world.y (up) = cos(lat) x' + sin(lat) z,
    // world.z (south) = sin(lat) x' - cos(lat) z.
    let m = |r: [f64; 3]| Vec3::new(r[0] as f32, r[1] as f32, r[2] as f32);
    let rx = m([-sl, cl, 0.0]);
    let ry = m([cp * cl, cp * sl, sp]);
    let rz = m([sp * cl, sp * sl, -cp]);
    // Columns of the matrix = images of the equatorial axes.
    glam::Mat3::from_cols(Vec3::new(rx.x, ry.x, rz.x), Vec3::new(rx.y, ry.y, rz.y), Vec3::new(rx.z, ry.z, rz.z))
}

/// Topocentric direction toward the moon (world frame) and its distance
/// (km). Low-precision lunar theory (P. Schlyter), good to a few arcminutes.
pub fn moon_direction(c: &SkyClock) -> (Vec3, f64) {
    let d = c.days();
    let rad = std::f64::consts::PI / 180.0;
    let n = rev(125.1228 - 0.0529538083 * d);
    let i = 5.1454;
    let w = rev(318.0634 + 0.1643573223 * d);
    let a = 60.2666;
    let e = 0.054900;
    let m = rev(115.3654 + 13.0649929509 * d);
    // Kepler's equation.
    let mut ea = m + e / rad * (m * rad).sin() * (1.0 + e * (m * rad).cos());
    for _ in 0..5 {
        ea -= (ea - e / rad * (ea * rad).sin() - m) / (1.0 - e * (ea * rad).cos());
    }
    let xv = a * ((ea * rad).cos() - e);
    let yv = a * ((1.0 - e * e).sqrt() * (ea * rad).sin());
    let v = yv.atan2(xv) / rad;
    let mut r = (xv * xv + yv * yv).sqrt();
    let (nr, ir, vw) = (n * rad, i * rad, (v + w) * rad);
    let xh = r * (nr.cos() * vw.cos() - nr.sin() * vw.sin() * ir.cos());
    let yh = r * (nr.sin() * vw.cos() + nr.cos() * vw.sin() * ir.cos());
    let zh = r * (vw.sin() * ir.sin());
    let mut lon = yh.atan2(xh) / rad;
    let mut lat = zh.atan2((xh * xh + yh * yh).sqrt()) / rad;
    // Main perturbations.
    let ms = rev(356.0470 + 0.9856002585 * d);
    let ws = 282.9404 + 4.70935e-5 * d;
    let ls = ms + ws;
    let lm = m + w + n;
    let dd = lm - ls;
    let f = lm - n;
    let s = |x: f64| (x * rad).sin();
    let co = |x: f64| (x * rad).cos();
    lon += -1.274 * s(m - 2.0 * dd) + 0.658 * s(2.0 * dd) - 0.186 * s(ms) - 0.059 * s(2.0 * m - 2.0 * dd) - 0.057 * s(m - 2.0 * dd + ms)
        + 0.053 * s(m + 2.0 * dd)
        + 0.046 * s(2.0 * dd - ms)
        + 0.041 * s(m - ms)
        - 0.035 * s(dd)
        - 0.031 * s(m + ms)
        - 0.015 * s(2.0 * f - 2.0 * dd)
        + 0.011 * s(m - 4.0 * dd);
    lat += -0.173 * s(f - 2.0 * dd) - 0.055 * s(m - f - 2.0 * dd) - 0.046 * s(m + f - 2.0 * dd) + 0.033 * s(f + 2.0 * dd) + 0.017 * s(2.0 * m + f);
    r += -0.58 * co(m - 2.0 * dd) - 0.46 * co(2.0 * dd);
    // Ecliptic → equatorial.
    let ecl = (23.4393 - 3.563e-7 * d) * rad;
    let (lo, la) = (lon * rad, lat * rad);
    let xe = lo.cos() * la.cos();
    let ye = lo.sin() * la.cos() * ecl.cos() - la.sin() * ecl.sin();
    let ze = lo.sin() * la.cos() * ecl.sin() + la.sin() * ecl.cos();
    let geo = star_rotation(c) * Vec3::new(xe as f32, ye as f32, ze as f32);
    // Topocentric parallax (~1°): the observer sits one Earth radius off the centre.
    let up = Vec3::Y;
    let topo = (geo.as_dvec3() * r - up.as_dvec3()).normalize();
    (topo.as_vec3(), r * 6378.14)
}

/// Moon phase quantities from the moon and sun directions: (illuminated
/// fraction 0..1, phase angle (rad, 0 = full)).
pub fn moon_phase(moon: Vec3, sun: Vec3) -> (f32, f32) {
    let elong = moon.dot(sun).clamp(-1.0, 1.0).acos();
    let alpha = std::f32::consts::PI - elong;
    ((1.0 + alpha.cos()) * 0.5, alpha)
}

/// Moonlight illuminance (lux) at the top of the atmosphere for a phase angle
/// (rad) and distance (km): 0.27 lux at a mean full moon (Allen's phase law).
pub fn moon_lux(alpha: f32, dist_km: f64) -> f32 {
    let a = alpha.abs().to_degrees();
    let mag = 0.026 * a + 4.0e-9 * a.powi(4);
    0.27 * 10f32.powf(-0.4 * mag) * ((384_400.0 / dist_km.max(3.0e5)) as f32).powi(2)
}

// ---------------------------------------------------------------- CPU medium (== atmos_core.wgsl)

const RG: f32 = 6360.0;
const RT: f32 = 6460.0;
const RAY_S: Vec3 = Vec3::new(5.802e-3, 13.558e-3, 33.1e-3);
const MIE_EXT: f32 = 3.996e-3 + 0.444e-3;
const OZONE_A: Vec3 = Vec3::new(0.650e-3, 1.881e-3, 0.085e-3);

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn extinction(h: f32, haze: f32, fog: (f32, f32)) -> Vec3 {
    let h = h.max(0.0);
    let top = fog.1.max(1e-3);
    let f = fog.0 * (1.0 - smoothstep(top * 0.55, top, h));
    let dr = (-h / 8.0).exp();
    let e = (-h / 1.2).exp();
    let bl = haze * e * (1.0 + 2.0 * (1.0 - smoothstep(0.6, 2.2, h)));
    let dm = e + (bl - e).max(0.0);
    let doz = (1.0 - (h - 25.0).abs() / 15.0).max(0.0);
    RAY_S * dr + Vec3::splat(MIE_EXT * dm + f) + OZONE_A * doz
}

/// Extinction of the air at the ground (per km, green) for a haze level.
pub fn transmittance_ext0(haze: f32) -> f32 {
    extinction(0.0, haze, (0.0, 0.0)).y
}

/// Scale of the stored sky-view / aerial-perspective LUTs for a sun
/// elevation (sin): 1 by day, up to 2^16 at night so the faint night sky
/// (radiance per unit sun irradiance ~1e-8) stays a normal fp16 number.
/// Readers multiply by `SUN_E / lut_scale`.
pub fn lut_scale(sun_y: f32) -> f32 {
    2f32.powf(16.0 * smoothstep(0.02, -0.12, sun_y))
}

/// Pre-exposure of the HDR targets for a sun elevation (sin): 1 by day,
/// 2^13 at night (see [`AtmosFrame::pre`]).
pub fn pre_exposure(sun_y: f32) -> f32 {
    (lut_scale(sun_y) / 8.0).max(1.0)
}

/// Transmittance from altitude `h_m` (metres) toward direction cosine `mu` to
/// space. `fog` = (ground fog extinction /km, fog top km).
pub fn transmittance(h_m: f32, mu: f32, haze: f32, fog: (f32, f32)) -> Vec3 {
    let r = RG + h_m.max(0.0) * 0.001;
    let disc = r * r * (mu * mu - 1.0) + RG * RG;
    if mu < 0.0 && disc >= 0.0 {
        return Vec3::ZERO;
    }
    let len = -r * mu + (r * r * (mu * mu - 1.0) + RT * RT).max(0.0).sqrt();
    // Fine steps through a shallow ground-fog layer (== cs_transmittance).
    let tf = if fog.0 > 0.0 && r - RG < fog.1 { len.min(if mu > 1e-3 { (fog.1 - (r - RG)) / mu.max(1e-3) } else { 5.0 }) } else { 0.0 };
    let mut od = Vec3::ZERO;
    for (t0, t1, n) in [(0.0, tf, 12), (tf, len, 64)] {
        let dt = (t1 - t0) / n as f32;
        for i in 0..n {
            let t = t0 + (i as f32 + 0.5) * dt;
            let ri = (r * r + t * t + 2.0 * r * mu * t).sqrt();
            od += extinction(ri - RG, haze, fog) * dt;
        }
    }
    (-od).exp()
}

// ---------------------------------------------------------------- GPU

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AtmosU {
    inv_view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    sun: [f32; 4],
    sun_e: [f32; 4],
    params: [f32; 4],
    fog: [f32; 4],
    /// Moon direction, w = moon irradiance / SUN_E (top of the atmosphere).
    moon: [f32; 4],
    /// x natural night-sky (airglow) radiance / SUN_E, y city upward radiance
    /// / SUN_E, z artificial illuminance for metering (scene units), w unused.
    night: [f32; 4],
    /// Skyglow colour (rgb), w unused.
    glow_tint: [f32; 4],
    /// Skyglow radiance / SUN_E, GLOW_EL x GLOW_AZ.
    glow: [[f32; 4]; GLOW_AZ * GLOW_EL / 4],
}

/// Per-frame atmosphere inputs.
#[derive(Clone, Copy, Debug)]
pub struct AtmosFrame {
    pub sun: Vec3,
    pub haze: f32,
    pub cover: f32,
    pub eye: Vec3,
    pub inv_view_proj: Mat4,
    pub refl_eye: Vec3,
    pub refl_inv_view_proj: Mat4,
    pub cloud_base: f32,
    /// Ground fog / mist: extinction (/km) and layer top (m).
    pub fog_ext: f32,
    pub fog_top: f32,
    /// Light let through an overcast deck (1 = typical).
    pub deck_trans: f32,
    /// Toward the moon and its irradiance at the top of the atmosphere (scene units).
    pub moon: Vec3,
    pub moon_e: f32,
    /// Natural night-sky radiance (airglow + unresolved stars, scene units).
    pub airglow: f32,
    /// Upward radiance of the lit city around the eye (scene units).
    pub city_up: f32,
    /// Artificial light the eye adapts to (scene illuminance).
    pub art_meter: f32,
    /// Light-pollution skyglow radiance (scene units), `GLOW_EL` x `GLOW_AZ`, and its colour.
    pub glow: [f32; GLOW_AZ * GLOW_EL],
    pub glow_tint: Vec3,
    /// The sky-view and aerial-perspective LUTs hold radiance per unit sun
    /// irradiance times this (see [`lut_scale`]).
    pub lut_scale: f32,
    /// Pre-exposure: every radiance written to the HDR targets (and the SH
    /// ambient, sun_eye) is multiplied by this, so night scenes (~1e-7 scene
    /// units) stay out of fp16 denormals; `expo` is divided by it.
    pub pre: f32,
}

pub struct Atmosphere {
    pub trans: wgpu::TextureView,
    pub ms: wgpu::TextureView,
    pub sky: wgpu::TextureView,
    pub ap_main: wgpu::TextureView,
    pub ap_refl: wgpu::TextureView,
    pub out: wgpu::Buffer,
    ubuf_main: wgpu::Buffer,
    ubuf_refl: wgpu::Buffer,
    p_trans: wgpu::ComputePipeline,
    p_ms: wgpu::ComputePipeline,
    p_sky: wgpu::ComputePipeline,
    p_ap: wgpu::ComputePipeline,
    p_amb: wgpu::ComputePipeline,
    bg_trans: wgpu::BindGroup,
    bg_ms: wgpu::BindGroup,
    bg_sky: wgpu::BindGroup,
    bg_ap_main: wgpu::BindGroup,
    bg_ap_refl: wgpu::BindGroup,
    bg_amb: wgpu::BindGroup,
    lut_key: Option<(f32, f32, f32)>,
}

fn lut(device: &wgpu::Device, label: &str, w: u32, h: u32, d: u32) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: d },
            mip_level_count: 1,
            sample_count: 1,
            dimension: if d > 1 { wgpu::TextureDimension::D3 } else { wgpu::TextureDimension::D2 },
            format: LUT_FMT,
            usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

impl Atmosphere {
    pub fn new(device: &wgpu::Device) -> Self {
        let src = format!("{}\n{}", include_str!("shaders/atmos_core.wgsl"), include_str!("shaders/atmosphere.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("atmosphere"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let pipe = |entry: &str| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: None,
                module: &module,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let (p_trans, p_ms, p_sky, p_ap, p_amb) = (pipe("cs_transmittance"), pipe("cs_multiscatter"), pipe("cs_skyview"), pipe("cs_aerial"), pipe("cs_ambient"));
        let trans = lut(device, "atmos transmittance", TRANS_W, TRANS_H, 1);
        let ms = lut(device, "atmos multiscatter", MS_N, MS_N, 1);
        let sky = lut(device, "atmos skyview", SKY_W, SKY_H, 1);
        let ap_main = lut(device, "atmos ap main", AP_N, AP_N, AP_N);
        let ap_refl = lut(device, "atmos ap refl", AP_N, AP_N, AP_N);
        let ubuf = |l: &str| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(l),
                size: std::mem::size_of::<AtmosU>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let ubuf_main = ubuf("atmos u main");
        let ubuf_refl = ubuf("atmos u refl");
        let out = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("atmos out"),
            size: OUT_SIZE,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let samp = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("atmos lin"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        use wgpu::BindingResource as R;
        let bg = |p: &wgpu::ComputePipeline, entries: &[(u32, R)]| {
            let e: Vec<wgpu::BindGroupEntry> = entries.iter().map(|(b, r)| wgpu::BindGroupEntry { binding: *b, resource: r.clone() }).collect();
            device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("atmos"), layout: &p.get_bind_group_layout(0), entries: &e })
        };
        let bg_trans = bg(&p_trans, &[(0, ubuf_main.as_entire_binding()), (4, R::TextureView(&trans))]);
        let bg_ms = bg(&p_ms, &[(0, ubuf_main.as_entire_binding()), (1, R::TextureView(&trans)), (3, R::Sampler(&samp)), (5, R::TextureView(&ms))]);
        let bg_sky = bg(
            &p_sky,
            &[(0, ubuf_main.as_entire_binding()), (1, R::TextureView(&trans)), (2, R::TextureView(&ms)), (3, R::Sampler(&samp)), (6, R::TextureView(&sky))],
        );
        let ap_bg = |u: &wgpu::Buffer, v: &wgpu::TextureView| {
            bg(&p_ap, &[(0, u.as_entire_binding()), (1, R::TextureView(&trans)), (2, R::TextureView(&ms)), (3, R::Sampler(&samp)), (7, R::TextureView(v))])
        };
        let bg_ap_main = ap_bg(&ubuf_main, &ap_main);
        let bg_ap_refl = ap_bg(&ubuf_refl, &ap_refl);
        let bg_amb = bg(
            &p_amb,
            &[(0, ubuf_main.as_entire_binding()), (1, R::TextureView(&trans)), (3, R::Sampler(&samp)), (8, out.as_entire_binding()), (9, R::TextureView(&sky))],
        );
        Atmosphere {
            trans,
            ms,
            sky,
            ap_main,
            ap_refl,
            out,
            ubuf_main,
            ubuf_refl,
            p_trans,
            p_ms,
            p_sky,
            p_ap,
            p_amb,
            bg_trans,
            bg_ms,
            bg_sky,
            bg_ap_main,
            bg_ap_refl,
            bg_amb,
            lut_key: None,
        }
    }

    /// Views the cloud renderer samples.
    pub fn cloud_views(&self) -> crate::clouds::CloudAtmos {
        crate::clouds::CloudAtmos { trans: self.trans.clone(), sky: self.sky.clone(), ap_main: self.ap_main.clone(), ap_refl: self.ap_refl.clone(), out: self.out.clone() }
    }

    /// Record this frame's atmosphere passes.
    pub fn update(&mut self, queue: &wgpu::Queue, encoder: &mut wgpu::CommandEncoder, f: &AtmosFrame) {
        let sun = f.sun.normalize();
        let u = |ivp: Mat4, eye: Vec3, mirrored: bool| AtmosU {
            inv_view_proj: ivp.to_cols_array_2d(),
            eye: [eye.x, eye.y, eye.z, f.haze],
            sun: [sun.x, sun.y, sun.z, f.cover],
            sun_e: [SUN_E * f.pre, SUN_E * f.pre, SUN_E * f.pre, if mirrored { 1.0 } else { 0.0 }],
            params: [REF_E, f.cloud_base, f.lut_scale, EXPOSURE_KEY],
            fog: [f.fog_ext, f.fog_top * 0.001, f.deck_trans, 0.0],
            moon: [f.moon.x, f.moon.y, f.moon.z, f.moon_e / SUN_E],
            night: [f.airglow / SUN_E, f.city_up / SUN_E, f.art_meter, f.pre],
            glow_tint: [f.glow_tint.x, f.glow_tint.y, f.glow_tint.z, 0.0],
            glow: bytemuck::cast(f.glow.map(|g| g / SUN_E)),
        };
        queue.write_buffer(&self.ubuf_main, 0, bytemuck::bytes_of(&u(f.inv_view_proj, f.eye, false)));
        queue.write_buffer(&self.ubuf_refl, 0, bytemuck::bytes_of(&u(f.refl_inv_view_proj, f.refl_eye, true)));
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("atmosphere"), timestamp_writes: None });
        let key = (f.haze, f.fog_ext, f.fog_top);
        if self.lut_key != Some(key) {
            self.lut_key = Some(key);
            pass.set_pipeline(&self.p_trans);
            pass.set_bind_group(0, &self.bg_trans, &[]);
            pass.dispatch_workgroups(TRANS_W / 8, TRANS_H / 8, 1);
            pass.set_pipeline(&self.p_ms);
            pass.set_bind_group(0, &self.bg_ms, &[]);
            pass.dispatch_workgroups(MS_N, MS_N, 1);
        }
        pass.set_pipeline(&self.p_sky);
        pass.set_bind_group(0, &self.bg_sky, &[]);
        pass.dispatch_workgroups(SKY_W.div_ceil(8), SKY_H.div_ceil(8), 1);
        pass.set_pipeline(&self.p_ap);
        pass.set_bind_group(0, &self.bg_ap_main, &[]);
        pass.dispatch_workgroups(AP_N / 4, AP_N / 4, AP_N / 4);
        pass.set_bind_group(0, &self.bg_ap_refl, &[]);
        pass.dispatch_workgroups(AP_N / 4, AP_N / 4, AP_N / 4);
        pass.set_pipeline(&self.p_amb);
        pass.set_bind_group(0, &self.bg_amb, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sun_in_london() {
        // 15 Aug, 13:00 BST: sun roughly south, ~50° up.
        let d = sun_direction(51.5, 0.0, day_of_year(8, 15), 13.0);
        let el = d.y.asin().to_degrees();
        assert!((el - 50.0).abs() < 3.0, "elevation {el}");
        assert!(d.z > 0.9 * d.length() * el.to_radians().cos() - 0.2, "should point south: {d:?}");
        // Sunset around 20:20 BST.
        let s = sun_direction(51.5, 0.0, day_of_year(8, 15), 20.3);
        assert!(s.y.abs() < 0.03, "{s:?}");
        assert!(s.x < -0.5, "sets in the west: {s:?}");
    }

    #[test]
    fn transmittance_reddens_low_sun() {
        let hi = transmittance(0.0, 0.9, 1.0, (0.0, 0.0));
        let lo = transmittance(0.0, 0.05, 1.0, (0.0, 0.0));
        assert!(hi.x > 0.8 && hi.z > 0.6);
        assert!(lo.x > lo.z * 2.0, "{lo:?}");
        assert_eq!(transmittance(0.0, -0.2, 1.0, (0.0, 0.0)), Vec3::ZERO);
        // Fog dims the sun near the ground but not above the fog top.
        let fog = (20.0, 0.15);
        assert!(transmittance(0.0, 0.5, 1.0, fog).y < 0.03, "{}", transmittance(0.0, 0.5, 1.0, fog).y);
        assert!((transmittance(300.0, 0.5, 1.0, fog).y - transmittance(300.0, 0.5, 1.0, (0.0, 0.0)).y).abs() < 1e-3);
    }
}
