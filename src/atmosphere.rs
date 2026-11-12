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
pub const SKY_W: u32 = 192;
pub const SKY_H: u32 = 108;
pub const AP_N: u32 = 32;
pub const LUT_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
/// Size of the `AtmosOut` storage buffer (bytes).
pub const OUT_SIZE: u64 = 4 * 16 + 16 + 16;

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
            sun_e: [SUN_E, SUN_E, SUN_E, if mirrored { 1.0 } else { 0.0 }],
            params: [REF_E, f.cloud_base, 0.0, EXPOSURE_KEY],
            fog: [f.fog_ext, f.fog_top * 0.001, f.deck_trans, 0.0],
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
