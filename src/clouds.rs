//! Volumetric cumulus clouds and the thermals that feed them (Track C).
//!
//! The cloud layout is a deterministic jittered world-space cell grid. The same
//! integer hash runs in Rust (`CloudField`: updraft / thermals) and in WGSL
//! (`clouds.wgsl`: weather map → ray-marched clouds), so every cumulus the
//! pilot sees has a thermal under it.
//!
//! Rendering: a weather map (coverage / height / stratus) baked around the eye,
//! a half-resolution ray march (Beer–Lambert, powder, dual-lobe HG, multiple
//! scattering octaves, aerial perspective, cirrus), temporal reprojection with
//! variance clipping, and a depth-aware upsample composited in place over the
//! HDR target. A top-down cloud-shadow transmittance map is produced for the
//! scene shaders.
//!
//! Per-frame use (see `CloudRenderer::render` for target requirements):
//! 1. `prepare(main frame)` — optional but recommended, before the scene
//!    passes: refreshes the weather map + cloud-shadow map so scene shaders
//!    sample a zero-lag `shadow_view()` with `shadow_params()`.
//! 2. after the reflection scene pass: `render(.., refl_hdr, refl_dist, 1)`
//!    with the mirrored view_proj / eye exactly as the scene used them.
//! 3. after the main pass resolve: `render(.., hdr, dist, 0)`.
//! 4. bloom + tonemap as before.
//! Call `reset_history()` after camera cuts / teleports.

use glam::{Mat4, Vec2, Vec3, vec2};

// ------------------------------------------------------------------ field

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Weather {
    Clear,
    Few,
    Scattered,
    Broken,
    Overcast,
}

impl Weather {
    pub const ALL: [Weather; 5] = [Weather::Clear, Weather::Few, Weather::Scattered, Weather::Broken, Weather::Overcast];
    pub fn name(self) -> &'static str {
        match self {
            Weather::Clear => "Clear",
            Weather::Few => "Few",
            Weather::Scattered => "Scattered",
            Weather::Broken => "Broken",
            Weather::Overcast => "Overcast",
        }
    }
}

/// Per-weather tuning shared by the CPU field and the shader.
#[derive(Clone, Copy, Debug)]
struct Params {
    /// Grid cell size (m); at most one cumulus / thermal per cell.
    cell: f32,
    /// Probability that a cell holds a thermal (and a cumulus if `visible`).
    prob: f32,
    /// Cumulus footprint radius range (m).
    r_min: f32,
    r_max: f32,
    /// Cumulus vertical extent range, base → top (m).
    h_min: f32,
    h_max: f32,
    /// Cloud base (m).
    base: f32,
    /// Stratiform (stratocumulus deck) coverage 0..1 and thickness (m).
    strato: f32,
    strato_thick: f32,
    /// Cirrus amount 0..1 (high thin layer, render only).
    cirrus: f32,
    /// Thermal strength multiplier (overcast starves thermals).
    thermal_gain: f32,
    /// Background sink between thermals (m/s, positive number).
    sink: f32,
    /// Draw cumulus (false = "blue" thermals with no cloud markers).
    visible: bool,
    /// Extinction scale (per metre at density 1).
    sigma: f32,
    /// Stratiform deck base (m) and the sky fraction it covers.
    strato_base: f32,
    strato_frac: f32,
    /// Mid-level layer (altocumulus / altostratus): coverage parameter, sky
    /// fraction, base and thickness (m).
    mid: f32,
    mid_frac: f32,
    mid_base: f32,
    mid_thick: f32,
}

/// Continuous description of the cloud layers (from a METAR / live data, see
/// `weather.rs`). Coverages are sky fractions 0..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CloudSpec {
    /// Cumulus amount 0..1 (0.25 ≈ FEW, 0.5 ≈ SCT, 0.75 ≈ BKN cumulus), base
    /// and maximum vertical extent (m).
    pub cumulus: f32,
    pub cu_base: f32,
    pub cu_depth: f32,
    /// Low stratiform deck (stratocumulus / stratus): sky fraction, base, thickness.
    pub strato: f32,
    pub st_base: f32,
    pub st_thick: f32,
    /// Mid-level layer: sky fraction, base, thickness.
    pub mid: f32,
    pub mid_base: f32,
    pub mid_thick: f32,
    /// High cirrus amount 0..1.
    pub cirrus: f32,
    /// Thermal strength multiplier (sunshine / instability).
    pub thermal: f32,
}

impl CloudSpec {
    pub fn clear() -> CloudSpec {
        CloudSpec {
            cumulus: 0.0,
            cu_base: 1250.0,
            cu_depth: 1000.0,
            strato: 0.0,
            st_base: 0.0,
            st_thick: 450.0,
            mid: 0.0,
            mid_base: 3500.0,
            mid_thick: 600.0,
            cirrus: 0.0,
            thermal: 0.6,
        }
    }
}

/// Weather-map threshold parameter for a stratiform sky fraction (the map
/// thresholds fbm noise: 0.42 covers about half the sky, >= 0.85 is a full
/// deck that continues to the horizon).
fn deck_param(frac: f32) -> f32 {
    if frac <= 0.01 {
        0.0
    } else if frac >= 0.97 {
        1.0
    } else {
        (0.42 + (frac - 0.5) * 0.42).clamp(0.2, 0.8)
    }
}

impl Params {
    fn of(w: Weather) -> Params {
        match w {
            Weather::Clear => Params {
                cell: 2600.0, prob: 0.45, r_min: 300.0, r_max: 600.0, h_min: 300.0, h_max: 600.0, base: 1250.0,
                strato: 0.0, strato_thick: 400.0, cirrus: 0.35, thermal_gain: 0.55, sink: 0.35, visible: false, sigma: 0.045,
                strato_base: 1250.0, strato_frac: 0.0, mid: 0.0, mid_frac: 0.0, mid_base: 3500.0, mid_thick: 600.0,
            },
            Weather::Few => Params {
                cell: 2800.0, prob: 0.36, r_min: 320.0, r_max: 700.0, h_min: 380.0, h_max: 950.0, base: 1150.0,
                strato: 0.0, strato_thick: 400.0, cirrus: 0.25, thermal_gain: 1.0, sink: 0.4, visible: true, sigma: 0.06,
                strato_base: 1150.0, strato_frac: 0.0, mid: 0.0, mid_frac: 0.0, mid_base: 3500.0, mid_thick: 600.0,
            },
            Weather::Scattered => Params {
                cell: 2100.0, prob: 0.62, r_min: 380.0, r_max: 950.0, h_min: 550.0, h_max: 1500.0, base: 1100.0,
                strato: 0.0, strato_thick: 450.0, cirrus: 0.45, thermal_gain: 1.0, sink: 0.5, visible: true, sigma: 0.07,
                strato_base: 1100.0, strato_frac: 0.0, mid: 0.0, mid_frac: 0.0, mid_base: 3500.0, mid_thick: 600.0,
            },
            Weather::Broken => Params {
                cell: 1800.0, prob: 0.82, r_min: 500.0, r_max: 1150.0, h_min: 700.0, h_max: 1900.0, base: 1050.0,
                strato: 0.42, strato_thick: 420.0, cirrus: 0.3, thermal_gain: 0.9, sink: 0.45, visible: true, sigma: 0.07,
                strato_base: 1050.0, strato_frac: 0.5, mid: 0.0, mid_frac: 0.0, mid_base: 3500.0, mid_thick: 600.0,
            },
            Weather::Overcast => Params {
                cell: 1700.0, prob: 0.8, r_min: 500.0, r_max: 1100.0, h_min: 600.0, h_max: 1400.0, base: 950.0,
                strato: 1.0, strato_thick: 650.0, cirrus: 0.0, thermal_gain: 0.35, sink: 0.25, visible: true, sigma: 0.06,
                strato_base: 950.0, strato_frac: 1.0, mid: 0.0, mid_frac: 0.0, mid_base: 3500.0, mid_thick: 600.0,
            },
        }
    }

    /// Continuous parameters: cumulus numbers interpolate between the presets'
    /// (amount 0 = Clear's blue thermals, 0.25 Few, 0.5 Scattered, 0.75 Broken).
    fn from_spec(s: &CloudSpec) -> Params {
        const DENSE: (f32, f32, f32, f32, f32, f32) = (1600.0, 0.92, 550.0, 1250.0, 0.075, 0.4);
        let anchors: [(f32, f32, f32, f32, f32, f32); 5] = [
            (2600.0, 0.45, 300.0, 600.0, 0.045, 0.35),
            (2800.0, 0.36, 320.0, 700.0, 0.06, 0.4),
            (2100.0, 0.62, 380.0, 950.0, 0.07, 0.5),
            (1800.0, 0.82, 500.0, 1150.0, 0.07, 0.45),
            DENSE,
        ];
        let a = s.cumulus.clamp(0.0, 1.0) * 4.0;
        let i = (a.floor() as usize).min(3);
        let t = a - i as f32;
        let (p, q) = (anchors[i], anchors[i + 1]);
        let h_max = s.cu_depth.clamp(250.0, 9000.0);
        Params {
            cell: lerp(p.0, q.0, t),
            prob: lerp(p.1, q.1, t),
            r_min: lerp(p.2, q.2, t),
            r_max: lerp(p.3, q.3, t),
            h_min: h_max * 0.4,
            h_max,
            base: s.cu_base.max(50.0),
            strato: deck_param(s.strato),
            strato_thick: s.st_thick.max(80.0),
            cirrus: s.cirrus.clamp(0.0, 1.0),
            thermal_gain: s.thermal.clamp(0.0, 1.6),
            sink: lerp(p.5, q.5, t),
            visible: s.cumulus > 0.04,
            sigma: lerp(p.4, q.4, t).max(0.05),
            strato_base: if s.st_base > 0.0 { s.st_base } else { s.cu_base },
            strato_frac: s.strato.clamp(0.0, 1.0),
            mid: deck_param(s.mid),
            mid_frac: s.mid.clamp(0.0, 1.0),
            mid_base: s.mid_base.max(300.0),
            mid_thick: s.mid_thick.max(100.0),
        }
    }
}

/// One cumulus cell: the cloud and the thermal feeding it share a centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cumulus {
    pub center: Vec2,
    /// Footprint radius of the cloud (m).
    pub radius: f32,
    /// Vertical extent base → top (m).
    pub height: f32,
    /// 0..1 relative size (drives thermal strength).
    pub size: f32,
    /// Thermal core radius (m) and peak strength (m/s).
    pub thermal_radius: f32,
    pub strength: f32,
    /// Footprint blobs (main body + turrets): centre, radius, height. The
    /// shader's weather map is built from exactly these.
    pub blobs: [(Vec2, f32, f32); BLOBS],
}

/// Blobs per cumulus: the main body over the thermal + 4 turrets.
pub const BLOBS: usize = 5;

// Integer hash identical to `lowbias32` in clouds.wgsl.
#[inline]
fn lowbias32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

#[inline]
fn cell_hash(seed: u32, ix: i32, iz: i32, k: u32) -> u32 {
    lowbias32(lowbias32(lowbias32(seed ^ (ix as u32)).wrapping_add(iz as u32)).wrapping_add(k.wrapping_mul(0x9e37_79b9)))
}

#[inline]
fn u01(h: u32) -> f32 {
    (h >> 8) as f32 * (1.0 / 16_777_216.0)
}

#[inline]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Deterministic world-space cloud / thermal field.
#[derive(Clone, Debug)]
pub struct CloudField {
    seed: u32,
    weather: Weather,
    p: Params,
    /// How far the whole field (clouds and the thermals under them) has
    /// drifted with the wind (m, world xz). World point xz lies at field
    /// point xz - drift.
    drift: Vec2,
}

impl CloudField {
    pub fn new(seed: u32, weather: Weather) -> Self {
        CloudField { seed, weather, p: Params::of(weather), drift: Vec2::ZERO }
    }

    /// A field from continuous layer parameters (live / METAR weather).
    pub fn from_spec(seed: u32, spec: &CloudSpec) -> Self {
        let mut f = CloudField { seed, weather: Weather::Clear, p: Params::from_spec(spec), drift: Vec2::ZERO };
        // Nearest preset, for the menu.
        let c = f.sky_cover();
        f.weather = if c < 0.06 {
            Weather::Clear
        } else if c < 0.25 {
            Weather::Few
        } else if c < 0.5 {
            Weather::Scattered
        } else if c < 0.88 {
            Weather::Broken
        } else {
            Weather::Overcast
        };
        f
    }

    /// Wind drift of the field (m): set every frame from the wind aloft x time.
    pub fn set_drift(&mut self, d: Vec2) {
        if d.is_finite() {
            self.drift = d;
        }
    }
    pub fn drift(&self) -> Vec2 {
        self.drift
    }

    /// Lowest base of the stratiform layers / mid layer.
    pub fn strato_base(&self) -> f32 {
        self.p.strato_base
    }
    /// Cumulus (thermal) cloud base.
    pub fn cu_base(&self) -> f32 {
        self.p.base
    }

    pub fn weather(&self) -> Weather {
        self.weather
    }
    pub fn seed(&self) -> u32 {
        self.seed
    }

    /// Lowest cloud base of any drawn layer (m).
    /// Lift every layer by `dy` (m): reported bases are above the ground,
    /// which is not at sea level in the hills (terrain.rs, World::apply_wx).
    pub fn raise(&mut self, dy: f32) {
        self.p.base += dy;
        self.p.strato_base += dy;
        self.p.mid_base += dy;
    }

    pub fn base(&self) -> f32 {
        let p = &self.p;
        let mut b = f32::MAX;
        if p.visible {
            b = b.min(p.base);
        }
        if p.strato > 0.0 {
            b = b.min(p.strato_base);
        }
        if p.mid > 0.0 {
            b = b.min(p.mid_base);
        }
        if b == f32::MAX { p.base } else { b }
    }

    /// Highest possible cloud top of all layers (m).
    pub fn top(&self) -> f32 {
        let p = &self.p;
        let mut t = p.base + p.h_max;
        if p.strato > 0.0 {
            t = t.max(p.strato_base + p.strato_thick);
        }
        if p.mid > 0.0 {
            t = t.max(p.mid_base + p.mid_thick);
        }
        t
    }

    /// Rough fraction of the sky covered (0 clear .. 1 overcast), for dimming
    /// sky ambient / fog under heavy cloud.
    pub fn sky_cover(&self) -> f32 {
        let p = &self.p;
        let cu = if p.visible { (p.prob * (p.r_min + p.r_max) * 0.5 / p.cell * 2.2).clamp(0.0, 0.75) } else { 0.0 };
        (1.0 - (1.0 - cu) * (1.0 - p.strato_frac) * (1.0 - 0.85 * p.mid_frac)).clamp(0.0, 1.0)
    }

    /// Optical thickness of the overcast (0 thin .. 1 thick nimbostratus) for
    /// how much light gets through the deck.
    pub fn deck_density(&self) -> f32 {
        let p = &self.p;
        let st = p.strato_frac * (p.strato_thick / 1500.0).clamp(0.2, 1.0);
        let mid = p.mid_frac * (p.mid_thick / 2000.0).clamp(0.1, 0.6);
        (st + mid).clamp(0.0, 1.0)
    }

    /// Whether cumulus are drawn (false for Clear: blue thermals only).
    pub fn visible(&self) -> bool {
        self.p.visible
    }

    /// The cumulus (and thermal) of grid cell (ix, iz), if any.
    pub fn cell(&self, ix: i32, iz: i32) -> Option<Cumulus> {
        let p = &self.p;
        if u01(cell_hash(self.seed, ix, iz, 0)) >= p.prob {
            return None;
        }
        let jx = 0.2 + 0.6 * u01(cell_hash(self.seed, ix, iz, 1));
        let jz = 0.2 + 0.6 * u01(cell_hash(self.seed, ix, iz, 2));
        let size = u01(cell_hash(self.seed, ix, iz, 3));
        let hv = u01(cell_hash(self.seed, ix, iz, 4));
        let center = vec2((ix as f32 + jx) * p.cell, (iz as f32 + jz) * p.cell);
        let radius = lerp(p.r_min, p.r_max, size);
        let height = lerp(p.h_min, p.h_max, size * 0.7 + hv * 0.3);
        let thermal_radius = (radius * 0.32).clamp(110.0, 330.0);
        let strength = (1.2 + 2.8 * size) * p.thermal_gain;
        let mut blobs = [(center, radius * 0.8, height); BLOBS];
        for (j, b) in blobs.iter_mut().enumerate().skip(1) {
            let k = j as u32 - 1;
            let a = u01(cell_hash(self.seed, ix, iz, 5 + k)) * std::f32::consts::TAU;
            let d = radius * (0.25 + 0.45 * u01(cell_hash(self.seed, ix, iz, 9 + k)));
            let r = radius * (0.35 + 0.3 * u01(cell_hash(self.seed, ix, iz, 13 + k)));
            let h = height * (0.35 + 0.6 * u01(cell_hash(self.seed, ix, iz, 17 + k)));
            *b = (center + vec2(a.cos(), a.sin()) * d, r, h);
        }
        Some(Cumulus { center, radius, height, size, thermal_radius, strength, blobs })
    }

    fn cell_of(&self, xz: Vec2) -> (i32, i32) {
        ((xz.x / self.p.cell).floor() as i32, (xz.y / self.p.cell).floor() as i32)
    }

    /// All cumulus cells whose cloud or thermal could touch a disc around
    /// world `xz` (positions in world space, i.e. including the drift).
    pub fn cells_near(&self, xz: Vec2, radius: f32) -> Vec<Cumulus> {
        let d = self.drift;
        let mut v = self.cells_near_field(xz - d, radius);
        for c in &mut v {
            c.center += d;
            for b in &mut c.blobs {
                b.0 += d;
            }
        }
        v
    }

    fn cells_near_field(&self, xz: Vec2, radius: f32) -> Vec<Cumulus> {
        let r = radius + self.p.r_max * 1.4;
        let (x0, z0) = self.cell_of(xz - Vec2::splat(r));
        let (x1, z1) = self.cell_of(xz + Vec2::splat(r));
        let mut v = Vec::new();
        for iz in z0..=z1 {
            for ix in x0..=x1 {
                if let Some(c) = self.cell(ix, iz) {
                    v.push(c);
                }
            }
        }
        v
    }

    /// Cumulus footprint coverage 0..1 at `xz` (the weather map's R channel,
    /// before render-distance fading). 0 for weather without visible cumulus.
    pub fn coverage(&self, xz: Vec2) -> f32 {
        self.coverage_height(xz).0
    }

    /// (coverage, local cloud height in m) at world `xz` — the weather map's
    /// R and G channels.
    pub fn coverage_height(&self, xz: Vec2) -> (f32, f32) {
        self.coverage_height_field(xz - self.drift)
    }

    fn coverage_height_field(&self, xz: Vec2) -> (f32, f32) {
        let p = &self.p;
        let (cx, cz) = self.cell_of(xz);
        let mut cov: f32 = 0.0;
        let (mut hs, mut ws) = (0.0f32, 0.0f32);
        for iz in cz - 1..=cz + 1 {
            for ix in cx - 1..=cx + 1 {
                if let Some(c) = self.cell(ix, iz) {
                    for (bc, br, bh) in c.blobs {
                        let b = bump(xz.distance(bc) / br);
                        cov = cov.max(b);
                        let w = b * b * b;
                        hs += w * bh;
                        ws += w;
                    }
                }
            }
        }
        let h = if ws > 1e-6 { hs / ws } else { 0.5 * (p.h_min + p.h_max) };
        (if p.visible { cov } else { 0.0 }, h)
    }

    /// Thermals within `radius` of `xz`: (centre, core radius, peak strength m/s).
    pub fn thermals_near(&self, xz: Vec2, radius: f32) -> Vec<(Vec2, f32, f32)> {
        self.cells_near(xz, radius)
            .into_iter()
            .filter(|c| c.center.distance(xz) <= radius + c.thermal_radius)
            .map(|c| (c.center, c.thermal_radius, c.strength))
            .collect()
    }

    /// Vertical air velocity (m/s, + = rising) at a point.
    pub fn updraft(&self, p: Vec3) -> f32 {
        if !p.is_finite() || p.y <= 0.0 {
            return 0.0;
        }
        let base = self.p.base;
        let xz = vec2(p.x, p.z) - self.drift;
        // Convective boundary layer: thermals ramp up off the ground.
        let ground = (p.y / 220.0).min(1.0);
        let mut w = 0.0;
        let mut sink_scale: f32 = if p.y < base { ground } else { 0.0 };
        let (cx, cz) = self.cell_of(xz);
        for iz in cz - 1..=cz + 1 {
            for ix in cx - 1..=cx + 1 {
                let Some(c) = self.cell(ix, iz) else { continue };
                let d = xz.distance(c.center);
                let rt = c.thermal_radius;
                let q = d / rt;
                if q > 6.0 {
                    continue;
                }
                // Vertical profile: full strength in the mixed layer, slightly
                // stronger just under base ("cloud suck"), continuing up inside
                // the cumulus and dying out at its top.
                let vert = if p.y < base {
                    ground * (1.0 + 0.15 * ((p.y - 0.6 * base) / (0.4 * base)).clamp(0.0, 1.0))
                } else {
                    let hf = (p.y - base) / c.height;
                    if hf >= 1.0 { 0.0 } else { 1.15 * (1.0 - hf * hf) }
                };
                let core = (-q * q).exp();
                // Compensating sink ring just outside the core.
                let ring = 0.22 * (-(q - 2.1) * (q - 2.1) * 1.6).exp();
                w += c.strength * vert * (core - ring);
                // No background sink inside the cloud body itself.
                if p.y >= base && d < c.radius && p.y < base + c.height {
                    sink_scale = 0.0;
                }
            }
        }
        w - self.p.sink * sink_scale
    }
}

/// Footprint bump: 1 at the centre → 0 at r = 1 (smooth). Mirrors WGSL `bump`.
#[inline]
fn bump(r: f32) -> f32 {
    let x = (1.0 - r * r).max(0.0);
    x * x
}

// ------------------------------------------------------------------ renderer

/// Everything the cloud renderer needs to know about the frame.
#[derive(Clone, Copy, Debug)]
pub struct CloudFrame {
    pub view_proj: Mat4,
    pub inv_view_proj: Mat4,
    /// Eye-relative view-projection (the view's rotation only, no
    /// translation): rays and reprojection are done relative to the eye so
    /// large world coordinates never cancel in f32 (that made the clouds judder).
    pub rot_view_proj: Mat4,
    pub eye: Vec3,
    pub sun_dir: Vec3,
    pub sun_col: Vec3,
    pub sky_intensity: f32,
    pub time: f32,
    pub width: u32,
    pub height: u32,
    /// [top-of-atmosphere sun irradiance, haze] (see atmosphere.rs).
    pub fog: [f32; 2],
    /// Ground fog [extinction /km, top km] (see atmosphere.rs `medium`).
    pub ground_fog: [f32; 2],
}

const WMAP_RES: u32 = 1024;
const WMAP_TEXEL: f32 = 64.0; // m per texel → 65.5 km map
const WMAP_SNAP: f32 = 32.0 * WMAP_TEXEL; // regenerate when the eye moves 2 km
const SHADOW_RES: u32 = 1024;
const SHADOW_TEXEL: f32 = 24.0; // m → 24.6 km
const SHAPE_RES: u32 = 128;
const DETAIL_RES: u32 = 32;
const DEP_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Rg32Float;
const COL_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
const SHADOW_FMT: wgpu::TextureFormat = wgpu::TextureFormat::R8Unorm;
/// Storm cells the weather map takes (== MAX_CELLS in clouds.wgsl).
const MAX_STORM: usize = crate::wx::storm::MAX_GPU_CELLS;

/// Thunderstorms for the clouds (wx/storm.rs): the cells near the eye (field
/// space, like the cumulus), where the rain shafts end and the shear that
/// slants them.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StormInput {
    pub cells: Vec<crate::wx::storm::Cell>,
    /// Bumped when the cells change (the weather map is re-baked).
    pub version: u64,
    /// Lowest rain-shaft bottom (world y, the ground) and the slant of the
    /// shafts (m of horizontal drift per m of fall, world xz).
    pub shaft_bottom: f32,
    pub shear: Vec2,
    /// 0..1: how dry the air under the base is (virga), for the shaft colour.
    pub dry: f32,
}

/// Standing lenticular clouds over mountain waves (wx/mountain.rs): cover on a
/// ground-fixed grid, its placement in the current frame, the lens level.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LentInput {
    pub grid: Vec<f32>,
    pub res: u32,
    /// World xz (current frame) of the grid's corner, and its size (m).
    pub origin: Vec2,
    pub size: f32,
    /// Lens level (world y) and maximum half thickness (m).
    pub alt: f32,
    pub thick: f32,
    pub version: u64,
}

/// A lightning flash lighting the clouds from inside: world position and
/// radiant intensity (scene radiance units x m², pre-exposed).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlashLight {
    pub pos: Vec3,
    pub intensity: f32,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    prev_view_proj: [[f32; 4]; 4],
    eye: [f32; 4],      // xyz, w = time
    sun_dir: [f32; 4],  // xyz, w = sky intensity
    sun_col: [f32; 4],  // rgb, w = temporal blend (1 = no history)
    fog: [f32; 4],      // density, falloff, frame jitter 0..1, slot
    screen: [f32; 4],   // full w, h, trace w, h
    wmap: [f32; 4],     // origin x, z, m/texel, texels
    shadow: [f32; 4],   // origin x, z, inv size (1/m), m/texel
    shape: [f32; 4],    // base, layer top, sigma, strato thickness
    cells: [f32; 4],    // cell size, prob, r_min, r_max
    cells2: [f32; 4],   // h_min, h_max, strato coverage, cirrus
    seed: [u32; 4],     // seed, frame, visible, 0
    layers: [f32; 4],   // strato base, mid coverage param, mid base, mid thickness
    drift: [f32; 4],    // field drift x, z (m), lowest base, 0
    gfog: [f32; 4],     // ground fog extinction (/km), top (km), 0, 0
    storm: [f32; 4],    // storm cells, rain-shaft bottom (world y), shaft slant x, z
    storm2: [f32; 4],   // dryness, storm top (world y), 0, 0
    lent: [f32; 4],     // lenticular grid origin x, z (world), size (m), lens level (world y)
    lent2: [f32; 4],    // on, half thickness (m), 0, 0
    flash: [[f32; 4]; 2], // eye-relative position, intensity
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StormCellsU {
    cells: [[f32; 4]; MAX_STORM * 4],
}

struct Slot {
    w: u32,
    h: u32,
    tw: u32,
    th: u32,
    trace_col: wgpu::TextureView,
    trace_dep: wgpu::TextureView,
    hist: [(wgpu::TextureView, wgpu::TextureView); 2],
    cur: usize,
    /// Last frame's eye-relative view-projection and eye.
    prev_vp: (Mat4, Vec3),
    valid: bool,
    frame: u32,
    ubuf: wgpu::Buffer,
}

/// The atmosphere the clouds are lit and hazed by (see `atmosphere.rs`):
/// transmittance + sky-view LUTs, aerial-perspective volumes for slot 0
/// (main view) and slot 1 (water reflection), ambient SH buffer.
#[derive(Clone)]
pub struct CloudAtmos {
    pub trans: wgpu::TextureView,
    pub sky: wgpu::TextureView,
    pub ap_main: wgpu::TextureView,
    pub ap_refl: wgpu::TextureView,
    pub out: wgpu::Buffer,
}

pub struct CloudRenderer {
    /// Ray-march quality 0 (low) .. 3 (ultra), gfx::quality (default 2).
    quality: u32,
    atmos: CloudAtmos,
    /// Thunderstorms (wx/): cells uniform, storm maps (tower / anvil / rain;
    /// anvil levels / shaft bottom / base), lenticular grid.
    storm: StormInput,
    storm_buf: wgpu::Buffer,
    storm_a: wgpu::TextureView,
    storm_b: wgpu::TextureView,
    lent: LentInput,
    lent_tex: wgpu::Texture,
    lent_view: wgpu::TextureView,
    lent_uploaded: u64,
    /// Any lens on the lenticular grid (a wave field without lenses keeps
    /// the calm trace).
    lent_on: bool,
    flashes: [FlashLight; 2],
    bgl: wgpu::BindGroupLayout,
    shape_view: wgpu::TextureView,
    detail_view: wgpu::TextureView,
    weather_tex: wgpu::Texture,
    weather_view: wgpu::TextureView,
    shadow_view: wgpu::TextureView,
    dummy_col: wgpu::TextureView,
    dummy_dep: wgpu::TextureView,
    dummy_dist: wgpu::TextureView,
    dummy_shadow: wgpu::TextureView,
    samp_rep: wgpu::Sampler,
    samp_clamp: wgpu::Sampler,
    weather_pipe: wgpu::RenderPipeline,
    shadow_pipe: wgpu::RenderPipeline,
    /// Without storms: [with lenticulars, without].
    shadow_pipe_calm: [wgpu::RenderPipeline; 2],
    trace_pipe: wgpu::RenderPipeline,
    trace_pipe_calm: [wgpu::RenderPipeline; 2],
    temporal_pipe: wgpu::RenderPipeline,
    composite_pipe: wgpu::RenderPipeline,
    /// Uniforms for the weather / shadow passes.
    aux_ubuf: wgpu::Buffer,
    slots: Vec<Option<Slot>>,
    field: CloudField,
    wmap_origin: Option<Vec2>,
    shadow_origin: Vec2,
    cirrus_drift: Vec2,
    shadow_frame: Option<u64>,
    frame_counter: u64,
    prepared_frame: Option<u64>,
}

fn tex2d(device: &wgpu::Device, label: &str, w: u32, h: u32, format: wgpu::TextureFormat, usage: wgpu::TextureUsages) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

fn tex3d(device: &wgpu::Device, label: &str, n: u32) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d { width: n, height: n, depth_or_array_layers: n },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D3,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    })
}

impl CloudRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, atmos: CloudAtmos) -> Self {
        let src = format!("{}\n{}", include_str!("shaders/atmos_core.wgsl"), include_str!("shaders/clouds.wgsl"));
        // Without storms (and lenticulars) the trace / shadow passes run
        // copies with that code compiled out (its registers slow the
        // fair-weather march down; a pipeline-overridable constant is not
        // folded away).
        let calm_src = src.replace("const WX: bool = true;", "const WX: bool = false;");
        let module_calm = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("clouds (calm)"), source: wgpu::ShaderSource::Wgsl(calm_src.as_str().into()) });
        let module_clear = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("clouds (clear)"),
            source: wgpu::ShaderSource::Wgsl(calm_src.replace("const WXL: bool = true;", "const WXL: bool = false;").into()),
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("clouds"),
            source: wgpu::ShaderSource::Wgsl(src.into()),
        });
        let fs = wgpu::ShaderStages::FRAGMENT;
        let float_tex = |binding: u32, dim: wgpu::TextureViewDimension, filterable: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: fs,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable },
                view_dimension: dim,
                multisampled: false,
            },
            count: None,
        };
        use wgpu::TextureViewDimension as D;
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("clouds"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | fs,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                float_tex(1, D::D3, true),  // shape noise
                float_tex(2, D::D3, true),  // detail noise
                float_tex(3, D::D2, true),  // weather map
                float_tex(4, D::D2, false), // scene distance (full res)
                float_tex(5, D::D2, true),  // trace colour
                float_tex(6, D::D2, false), // trace depth
                float_tex(7, D::D2, true),  // history colour
                float_tex(8, D::D2, false), // history depth
                wgpu::BindGroupLayoutEntry { binding: 9, visibility: fs, ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering), count: None },
                wgpu::BindGroupLayoutEntry { binding: 10, visibility: fs, ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering), count: None },
                float_tex(12, D::D2, true), // cloud shadow map (god rays)
                float_tex(13, D::D2, true), // atmosphere transmittance LUT
                float_tex(14, D::D2, true), // sky-view LUT
                float_tex(15, D::D3, true), // aerial perspective volume
                wgpu::BindGroupLayoutEntry {
                    binding: 16,
                    visibility: fs,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 17,
                    visibility: fs,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                float_tex(18, D::D2, true), // storm map A: tower cover, top (km), anvil cover, rain
                float_tex(19, D::D2, true), // storm map B: anvil base, anvil top, shaft bottom, base (km)
                float_tex(20, D::D2, true), // lenticular cover
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("clouds"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });
        let pipe_with = |label: &str, module: &wgpu::ShaderModule, fs_entry: &str, targets: &[Option<wgpu::ColorTargetState>]| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module, entry_point: Some("vs_full"), compilation_options: Default::default(), buffers: &[] },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState { module, entry_point: Some(fs_entry), compilation_options: Default::default(), targets }),
                multiview_mask: None,
                cache: None,
            })
        };
        let pipe = |label: &str, fs_entry: &str, targets: &[Option<wgpu::ColorTargetState>]| pipe_with(label, &module, fs_entry, targets);
        let t = |format| Some(wgpu::ColorTargetState { format, blend: None, write_mask: wgpu::ColorWrites::ALL });
        let weather_pipe = pipe("cloud weather", "fs_weather", &[t(COL_FMT), t(COL_FMT), t(COL_FMT)]);
        let shadow_pipe = pipe("cloud shadow", "fs_shadow", &[t(SHADOW_FMT)]);
        let shadow_pipe_calm = [&module_calm, &module_clear].map(|m| pipe_with("cloud shadow (calm)", m, "fs_shadow", &[t(SHADOW_FMT)]));
        let trace_pipe = pipe("cloud trace", "fs_trace", &[t(COL_FMT), t(DEP_FMT)]);
        let trace_pipe_calm = [&module_calm, &module_clear].map(|m| pipe_with("cloud trace (calm)", m, "fs_trace", &[t(COL_FMT), t(DEP_FMT)]));
        let temporal_pipe = pipe("cloud temporal", "fs_temporal", &[t(COL_FMT), t(DEP_FMT)]);
        // Premultiplied "over": dst = cloud.rgb + dst * transmittance.
        let composite_pipe = pipe(
            "cloud composite",
            "fs_composite",
            &[Some(wgpu::ColorTargetState {
                format: wgpu::TextureFormat::Rgba16Float,
                blend: Some(wgpu::BlendState {
                    color: wgpu::BlendComponent {
                        src_factor: wgpu::BlendFactor::One,
                        dst_factor: wgpu::BlendFactor::SrcAlpha,
                        operation: wgpu::BlendOperation::Add,
                    },
                    alpha: wgpu::BlendComponent {
                        src_factor: wgpu::BlendFactor::Zero,
                        dst_factor: wgpu::BlendFactor::One,
                        operation: wgpu::BlendOperation::Add,
                    },
                }),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        );

        // --- 3D noise, generated once on the GPU.
        let shape = tex3d(device, "cloud shape noise", SHAPE_RES);
        let detail = tex3d(device, "cloud detail noise", DETAIL_RES);
        {
            let noise_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("cloud noise"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 11,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format: wgpu::TextureFormat::Rgba8Unorm,
                        view_dimension: D::D3,
                    },
                    count: None,
                }],
            });
            let nl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("cloud noise"),
                bind_group_layouts: &[Some(&noise_bgl)],
                immediate_size: 0,
            });
            let cp = |entry: &str| {
                device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(entry),
                    layout: Some(&nl),
                    module: &module,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    cache: None,
                })
            };
            let shape_pipe = cp("cs_shape_noise");
            let detail_pipe = cp("cs_detail_noise");
            let mut enc = device.create_command_encoder(&Default::default());
            for (tex, p, n) in [(&shape, &shape_pipe, SHAPE_RES), (&detail, &detail_pipe, DETAIL_RES)] {
                let view = tex.create_view(&Default::default());
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("cloud noise"),
                    layout: &noise_bgl,
                    entries: &[wgpu::BindGroupEntry { binding: 11, resource: wgpu::BindingResource::TextureView(&view) }],
                });
                let mut pass = enc.begin_compute_pass(&Default::default());
                pass.set_pipeline(p);
                pass.set_bind_group(0, &bg, &[]);
                pass.dispatch_workgroups(n / 4, n / 4, n / 4);
            }
            queue.submit([enc.finish()]);
        }

        let rt = wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        let weather_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("cloud weather"),
            size: wgpu::Extent3d { width: WMAP_RES, height: WMAP_RES, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COL_FMT,
            usage: rt | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let weather_view = weather_tex.create_view(&Default::default());
        let shadow_view = tex2d(device, "cloud shadow", SHADOW_RES, SHADOW_RES, SHADOW_FMT, rt);
        let tb = wgpu::TextureUsages::TEXTURE_BINDING;
        let dummy_col = tex2d(device, "cloud dummy", 1, 1, COL_FMT, tb);
        let dummy_dep = tex2d(device, "cloud dummy dep", 1, 1, DEP_FMT, tb);
        let dummy_dist = tex2d(device, "cloud dummy dist", 1, 1, wgpu::TextureFormat::R16Float, tb);
        let dummy_shadow = tex2d(device, "cloud dummy shadow", 1, 1, SHADOW_FMT, tb);
        let samp = |mode| {
            device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("cloud"),
                address_mode_u: mode,
                address_mode_v: mode,
                address_mode_w: mode,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                ..Default::default()
            })
        };
        let samp_rep = samp(wgpu::AddressMode::Repeat);
        let samp_clamp = samp(wgpu::AddressMode::ClampToEdge);
        let storm_a = tex2d(device, "storm map a", WMAP_RES, WMAP_RES, COL_FMT, rt);
        let storm_b = tex2d(device, "storm map b", WMAP_RES, WMAP_RES, COL_FMT, rt);
        let storm_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("storm cells"),
            size: std::mem::size_of::<StormCellsU>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let lent_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("lenticulars"),
            size: wgpu::Extent3d { width: crate::wx::mountain::RES as u32, height: crate::wx::mountain::RES as u32, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let lent_view = lent_tex.create_view(&Default::default());
        let aux_ubuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cloud aux u"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        CloudRenderer {
            atmos,
            storm: StormInput::default(),
            storm_buf,
            storm_a,
            storm_b,
            lent: LentInput::default(),
            lent_tex,
            lent_view,
            lent_uploaded: 0,
            lent_on: false,
            flashes: [FlashLight::default(); 2],
            bgl,
            shape_view: shape.create_view(&Default::default()),
            detail_view: detail.create_view(&Default::default()),
            weather_tex,
            weather_view,
            shadow_view,
            dummy_col,
            dummy_dep,
            dummy_dist,
            dummy_shadow,
            samp_rep,
            samp_clamp,
            weather_pipe,
            shadow_pipe,
            shadow_pipe_calm,
            trace_pipe,
            trace_pipe_calm,
            temporal_pipe,
            composite_pipe,
            aux_ubuf,
            slots: Vec::new(),
            field: CloudField::new(1, Weather::Scattered),
            wmap_origin: None,
            shadow_origin: Vec2::ZERO,
            cirrus_drift: Vec2::ZERO,
            shadow_frame: None,
            frame_counter: 0,
            quality: 2,
            prepared_frame: None,
        }
    }

    pub fn set_field(&mut self, field: &CloudField) {
        self.field = field.clone();
        self.wmap_origin = None;
        for s in self.slots.iter_mut().flatten() {
            s.valid = false;
        }
    }

    pub fn field(&self) -> &CloudField {
        &self.field
    }

    /// Move the clouds with the wind (see `CloudField::set_drift`); keeps history.
    pub fn set_drift(&mut self, d: Vec2) {
        self.field.set_drift(d);
    }

    /// Extra drift of the cirrus layer (wind only: the field drift already
    /// carries the floating-origin offset).
    pub fn set_cirrus_drift(&mut self, d: Vec2) {
        if d.is_finite() {
            self.cirrus_drift = d;
        }
    }

    /// Top-down cloud-shadow transmittance (R8Unorm, 0 = full shadow .. 1 = clear)
    /// for a ray from the ground point (x, 0, z) toward the sun.
    pub fn shadow_view(&self) -> &wgpu::TextureView {
        &self.shadow_view
    }

    /// The latest half-resolution cloud result of a slot (after `render`):
    /// colour (a = transmittance) and depth (x = march limit, y = mean cloud
    /// distance) — the effects composite against it (vfx/gpu.rs).
    pub fn history(&self, slot: usize) -> Option<(&wgpu::TextureView, &wgpu::TextureView)> {
        let s = self.slots.get(slot)?.as_ref()?;
        let h = &s.hist[s.cur];
        Some((&h.0, &h.1))
    }

    /// origin_x, origin_z, inv_size, cloud base: uv = (xz - origin) * inv_size + 0.5.
    pub fn shadow_params(&self) -> [f32; 4] {
        [self.shadow_origin.x, self.shadow_origin.y, 1.0 / (SHADOW_RES as f32 * SHADOW_TEXEL), self.field.base()]
    }

    #[allow(clippy::too_many_arguments)]
    fn uniforms(&self, frame: &CloudFrame, prev: (Mat4, Vec3), blend: f32, jitter: f32, slot: usize, tw: u32, th: u32, fidx: u32) -> Uniforms {
        let p = &self.field.p;
        let wo = self.wmap_origin.unwrap_or(Vec2::ZERO);
        let sun = frame.sun_dir.normalize();
        // Eye-relative matrices: a point at `eye + r` is at `r + (eye - prev_eye)`
        // relative to last frame's eye.
        let prev_rel = prev.0 * Mat4::from_translation(frame.eye - prev.1);
        Uniforms {
            view_proj: frame.rot_view_proj.to_cols_array_2d(),
            inv_view_proj: frame.rot_view_proj.inverse().to_cols_array_2d(),
            prev_view_proj: prev_rel.to_cols_array_2d(),
            eye: [frame.eye.x, frame.eye.y, frame.eye.z, frame.time],
            sun_dir: [sun.x, sun.y, sun.z, frame.sky_intensity],
            sun_col: [frame.sun_col.x, frame.sun_col.y, frame.sun_col.z, blend],
            fog: [frame.fog[0], frame.fog[1], jitter, slot as f32],
            screen: [frame.width as f32, frame.height as f32, tw as f32, th as f32],
            wmap: [wo.x, wo.y, WMAP_TEXEL, WMAP_RES as f32],
            shadow: [self.shadow_origin.x, self.shadow_origin.y, 1.0 / (SHADOW_RES as f32 * SHADOW_TEXEL), SHADOW_TEXEL],
            shape: [p.base, self.field.top().max(self.storm_top()).max(if !self.lent_on { 0.0 } else { self.lent.alt + self.lent.thick * 2.0 + 1500.0 }), p.sigma, p.strato_thick],
            cells: [p.cell, p.prob, p.r_min, p.r_max],
            cells2: [p.h_min, p.h_max, p.strato, p.cirrus],
            seed: [self.field.seed, fidx, p.visible as u32, self.quality],
            layers: [p.strato_base, p.mid, p.mid_base, p.mid_thick],
            drift: [self.field.drift.x, self.field.drift.y, self.storm.cells.iter().take(MAX_STORM).map(|c| c.base).fold(self.field.base(), f32::min), 0.0],
            gfog: [frame.ground_fog[0], frame.ground_fog[1], self.cirrus_drift.x, self.cirrus_drift.y],
            storm: [self.storm.cells.len().min(MAX_STORM) as f32, self.storm.shaft_bottom, self.storm.shear.x, self.storm.shear.y],
            storm2: [self.storm.dry, self.storm_top(), 0.0, 0.0],
            lent: [self.lent.origin.x, self.lent.origin.y, self.lent.size.max(1.0), self.lent.alt],
            lent2: [if self.lent_on { 1.0 } else { 0.0 }, self.lent.thick, 0.0, 0.0],
            flash: std::array::from_fn(|i| {
                let f = self.flashes[i];
                let r = f.pos - frame.eye;
                [r.x, r.y, r.z, f.intensity]
            }),
        }
    }

    /// The trace / shadow pipeline for the weather: with the storm (and
    /// lenticular) code only when there are any.
    fn pick<'a>(&self, full: &'a wgpu::RenderPipeline, calm: &'a [wgpu::RenderPipeline; 2]) -> &'a wgpu::RenderPipeline {
        if !self.storm.cells.is_empty() {
            full
        } else if self.lent_on {
            &calm[0]
        } else {
            &calm[1]
        }
    }

    /// Highest storm top (world y), 0 without storms.
    fn storm_top(&self) -> f32 {
        self.storm.cells.iter().take(MAX_STORM).map(|c| c.top + c.overshoot + 200.0).fold(0.0, f32::max)
    }

    /// Thunderstorm cells (wx/storm.rs) — re-bakes the weather map when they change.
    pub fn set_storms(&mut self, queue: &wgpu::Queue, s: &StormInput) {
        if s.version == self.storm.version && s.cells.len() == self.storm.cells.len() {
            self.storm.shaft_bottom = s.shaft_bottom;
            self.storm.shear = s.shear;
            return;
        }
        self.storm = s.clone();
        let mut u = StormCellsU { cells: [[0.0; 4]; MAX_STORM * 4] };
        for (i, c) in s.cells.iter().take(MAX_STORM).enumerate() {
            let rn = (c.rain_mmh / 120.0).clamp(0.0, 1.0);
            u.cells[i * 4] = [c.center.x, c.center.y, c.radius, c.base];
            u.cells[i * 4 + 1] = [c.top, c.overshoot, c.intensity, c.anvil_len];
            u.cells[i * 4 + 2] = [c.anvil_dir.x, c.anvil_dir.y, c.rain_off.x, c.rain_off.y];
            u.cells[i * 4 + 3] = [c.rain_r, rn, c.virga_bottom, c.anvil_base()];
        }
        queue.write_buffer(&self.storm_buf, 0, bytemuck::bytes_of(&u));
        self.wmap_origin = None;
        for sl in self.slots.iter_mut().flatten() {
            sl.valid = false;
        }
    }

    /// Lenticular clouds over mountain waves (wx/mountain.rs).
    pub fn set_lenticular(&mut self, queue: &wgpu::Queue, l: &LentInput) {
        if l.version != self.lent_uploaded && !l.grid.is_empty() && l.res as usize == crate::wx::mountain::RES {
            let n = l.res;
            let bytes: Vec<u8> = l.grid.iter().map(|v| (v.clamp(0.0, 1.0) * 255.0).round() as u8).collect();
            queue.write_texture(
                wgpu::TexelCopyTextureInfo { texture: &self.lent_tex, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                &bytes,
                wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(n), rows_per_image: Some(n) },
                wgpu::Extent3d { width: n, height: n, depth_or_array_layers: 1 },
            );
            self.lent_uploaded = l.version;
        }
        let was_on = self.lent_on;
        self.lent = l.clone();
        self.lent_on = l.grid.iter().any(|v| *v > 0.01);
        if was_on != self.lent_on {
            for sl in self.slots.iter_mut().flatten() {
                sl.valid = false;
            }
        }
    }

    /// Lightning lighting the clouds this frame (applied after the temporal
    /// filter, so a flash neither smears into the history nor ghosts).
    pub fn set_flashes(&mut self, f: [FlashLight; 2]) {
        self.flashes = f;
    }

    #[allow(clippy::too_many_arguments)]
    fn bind_group(
        &self,
        device: &wgpu::Device,
        ubuf: &wgpu::Buffer,
        weather: Option<&wgpu::TextureView>,
        dist: Option<&wgpu::TextureView>,
        trace: Option<(&wgpu::TextureView, &wgpu::TextureView)>,
        hist: Option<(&wgpu::TextureView, &wgpu::TextureView)>,
        with_shadow: bool,
        slot: usize,
    ) -> wgpu::BindGroup {
        use wgpu::BindingResource as R;
        let (tc, td) = trace.unwrap_or((&self.dummy_col, &self.dummy_dep));
        let (hc, hd) = hist.unwrap_or((&self.dummy_col, &self.dummy_dep));
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("clouds"),
            layout: &self.bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(&self.shape_view) },
                wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(&self.detail_view) },
                wgpu::BindGroupEntry { binding: 3, resource: R::TextureView(weather.unwrap_or(&self.dummy_col)) },
                wgpu::BindGroupEntry { binding: 4, resource: R::TextureView(dist.unwrap_or(&self.dummy_dist)) },
                wgpu::BindGroupEntry { binding: 5, resource: R::TextureView(tc) },
                wgpu::BindGroupEntry { binding: 6, resource: R::TextureView(td) },
                wgpu::BindGroupEntry { binding: 7, resource: R::TextureView(hc) },
                wgpu::BindGroupEntry { binding: 8, resource: R::TextureView(hd) },
                wgpu::BindGroupEntry { binding: 9, resource: R::Sampler(&self.samp_rep) },
                wgpu::BindGroupEntry { binding: 10, resource: R::Sampler(&self.samp_clamp) },
                wgpu::BindGroupEntry { binding: 12, resource: R::TextureView(if with_shadow { &self.shadow_view } else { &self.dummy_shadow }) },
                wgpu::BindGroupEntry { binding: 13, resource: R::TextureView(&self.atmos.trans) },
                wgpu::BindGroupEntry { binding: 14, resource: R::TextureView(&self.atmos.sky) },
                wgpu::BindGroupEntry { binding: 15, resource: R::TextureView(if slot == 1 { &self.atmos.ap_refl } else { &self.atmos.ap_main }) },
                wgpu::BindGroupEntry { binding: 16, resource: self.atmos.out.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 17, resource: self.storm_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 18, resource: R::TextureView(if weather.is_some() { &self.storm_a } else { &self.dummy_col }) },
                wgpu::BindGroupEntry { binding: 19, resource: R::TextureView(if weather.is_some() { &self.storm_b } else { &self.dummy_col }) },
                wgpu::BindGroupEntry { binding: 20, resource: R::TextureView(&self.lent_view) },
            ],
        })
    }

    fn ensure_slot(&mut self, device: &wgpu::Device, slot: usize, w: u32, h: u32) {
        if self.slots.len() <= slot {
            self.slots.resize_with(slot + 1, || None);
        }
        if let Some(s) = &self.slots[slot]
            && s.w == w
            && s.h == h
        {
            return;
        }
        let (tw, th) = (w.div_ceil(2).max(1), h.div_ceil(2).max(1));
        let rt = wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        let mk = |l: &str| (tex2d(device, l, tw, th, COL_FMT, rt), tex2d(device, l, tw, th, DEP_FMT, rt));
        let (trace_col, trace_dep) = mk("cloud trace");
        self.slots[slot] = Some(Slot {
            w,
            h,
            tw,
            th,
            trace_col,
            trace_dep,
            hist: [mk("cloud hist a"), mk("cloud hist b")],
            cur: 0,
            prev_vp: (Mat4::IDENTITY, Vec3::ZERO),
            valid: false,
            frame: 0,
            ubuf: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("cloud u"),
                size: std::mem::size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
        });
    }

    /// Optional: update the weather map and the cloud-shadow map for this frame
    /// *before* the scene pass, so scene shaders sample a zero-lag shadow.
    /// `render(.., slot 0)` calls this itself if it was not called this frame.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, encoder: &mut wgpu::CommandEncoder, frame: &CloudFrame) {
        self.frame_counter += 1;
        self.prepared_frame = Some(self.frame_counter);
        self.update_maps(device, queue, encoder, frame, true);
    }

    fn update_maps(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, encoder: &mut wgpu::CommandEncoder, frame: &CloudFrame, shadow: bool) {
        let eye_xz = vec2(frame.eye.x, frame.eye.z);
        // The weather map lives in field space (it drifts with the wind).
        let want = ((eye_xz - self.field.drift) / WMAP_SNAP).round() * WMAP_SNAP;
        let regen_weather = self.wmap_origin != Some(want);
        if regen_weather {
            self.wmap_origin = Some(want);
        }
        // The cloud-shadow map is refreshed every other frame (clouds move
        // well under a texel per frame), always after a weather-map rebuild.
        let do_shadow = shadow && self.shadow_frame != Some(self.frame_counter) && (regen_weather || self.shadow_frame.is_none_or(|f| self.frame_counter >= f + 2));
        if !regen_weather && !do_shadow {
            return;
        }
        if do_shadow {
            self.shadow_frame = Some(self.frame_counter);
            self.shadow_origin = (eye_xz / SHADOW_TEXEL).round() * SHADOW_TEXEL;
        }
        let u = self.uniforms(frame, (frame.rot_view_proj, frame.eye), 1.0, 0.0, 0, 1, 1, 0);
        queue.write_buffer(&self.aux_ubuf, 0, bytemuck::bytes_of(&u));
        let fullscreen = |encoder: &mut wgpu::CommandEncoder, view: &wgpu::TextureView, pipe: &wgpu::RenderPipeline, bg: &wgpu::BindGroup| {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("clouds aux"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(pipe);
            pass.set_bind_group(0, bg, &[]);
            pass.draw(0..3, 0..1);
        };
        if regen_weather {
            let bg = self.bind_group(device, &self.aux_ubuf, None, None, None, None, false, 0);
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("clouds weather"),
                color_attachments: &[&self.weather_view, &self.storm_a, &self.storm_b]
                    .map(|view| {
                        Some(wgpu::RenderPassColorAttachment {
                            view,
                            depth_slice: None,
                            resolve_target: None,
                            ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                        })
                    }),
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.weather_pipe);
            pass.set_bind_group(0, &bg, &[]);
            pass.draw(0..3, 0..1);
        }
        if do_shadow {
            let bg = self.bind_group(device, &self.aux_ubuf, Some(&self.weather_view), None, None, None, false, 0);
            fullscreen(encoder, &self.shadow_view, self.pick(&self.shadow_pipe, &self.shadow_pipe_calm), &bg);
        }
    }

    /// Composite clouds in place over `hdr` (Rgba16Float, RENDER_ATTACHMENT,
    /// resolved) using `dist` (R16Float, same size: eye → opaque surface in
    /// metres; sky = 65000). `slot`: 0 = main view, 1 = water reflection, ...
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &CloudFrame,
        hdr: &wgpu::TextureView,
        dist: &wgpu::TextureView,
        slot: usize,
    ) {
        if slot == 0 && self.prepared_frame != Some(self.frame_counter) {
            self.frame_counter += 1;
            self.prepared_frame = Some(self.frame_counter);
        }
        self.update_maps(device, queue, encoder, frame, slot == 0);
        self.ensure_slot(device, slot, frame.width, frame.height);

        let (tw, th, prev_vp, valid, fidx) = {
            let s = self.slots[slot].as_ref().unwrap();
            (s.tw, s.th, s.prev_vp, s.valid, s.frame)
        };
        // Golden-ratio sequence for the per-pixel ray offset.
        let jitter = (fidx as f32 * 0.618_034) % 1.0;
        let blend = if valid { 0.08 } else { 1.0 };
        let u = self.uniforms(frame, if valid { prev_vp } else { (frame.rot_view_proj, frame.eye) }, blend, jitter, slot, tw, th, fidx);
        let s = self.slots[slot].as_ref().unwrap();
        queue.write_buffer(&s.ubuf, 0, bytemuck::bytes_of(&u));

        let (prev, next) = (&s.hist[s.cur], &s.hist[1 - s.cur]);
        let bg_trace = self.bind_group(device, &s.ubuf, Some(&self.weather_view), Some(dist), None, None, true, slot);
        let bg_temporal = self.bind_group(device, &s.ubuf, None, None, Some((&s.trace_col, &s.trace_dep)), Some((&prev.0, &prev.1)), false, slot);
        let bg_comp = self.bind_group(device, &s.ubuf, None, Some(dist), None, Some((&next.0, &next.1)), false, slot);

        let pass = |encoder: &mut wgpu::CommandEncoder,
                    label: &str,
                    views: &[&wgpu::TextureView],
                    load: bool,
                    pipe: &wgpu::RenderPipeline,
                    bg: &wgpu::BindGroup| {
            let atts: Vec<Option<wgpu::RenderPassColorAttachment>> = views
                .iter()
                .map(|v| {
                    Some(wgpu::RenderPassColorAttachment {
                        view: v,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: if load { wgpu::LoadOp::Load } else { wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT) },
                            store: wgpu::StoreOp::Store,
                        },
                    })
                })
                .collect();
            let mut p = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some(label),
                color_attachments: &atts,
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            p.set_pipeline(pipe);
            p.set_bind_group(0, bg, &[]);
            p.draw(0..3, 0..1);
        };
        pass(encoder, "cloud trace", &[&s.trace_col, &s.trace_dep], false, self.pick(&self.trace_pipe, &self.trace_pipe_calm), &bg_trace);
        pass(encoder, "cloud temporal", &[&next.0, &next.1], false, &self.temporal_pipe, &bg_temporal);
        pass(encoder, "cloud composite", &[hdr], true, &self.composite_pipe, &bg_comp);

        let s = self.slots[slot].as_mut().unwrap();
        s.cur = 1 - s.cur;
        s.prev_vp = (frame.rot_view_proj, frame.eye);
        s.valid = true;
        s.frame = s.frame.wrapping_add(1);
    }

    /// The world origin moved: points p → p + delta. The field itself is
    /// unchanged (the caller shifts the drift), so the history stays valid
    /// once the stored eyes / shadow-map origin move with the frame.
    pub fn rebase(&mut self, delta: glam::Vec3) {
        for s in self.slots.iter_mut().flatten() {
            s.prev_vp.1 += delta;
        }
        self.shadow_origin += vec2(delta.x, delta.z);
    }

    /// Ray-march quality: 0 low (coarse steps) .. 2 high (the reference) .. 3 ultra (finer).
    pub fn set_quality(&mut self, q: u8) {
        self.quality = q.min(3) as u32;
    }

    /// Forget temporal history (e.g. after a camera cut / teleport).
    pub fn reset_history(&mut self) {
        for s in self.slots.iter_mut().flatten() {
            s.valid = false;
        }
    }
}

// ------------------------------------------------------------------ CPU/GPU consistency

fn f16_to_f32(h: u16) -> f32 {
    let s = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let e = ((h >> 10) & 0x1f) as i32;
    let m = (h & 0x3ff) as f32;
    s * match e {
        0 => m * 2f32.powi(-24),
        31 => f32::INFINITY,
        _ => (1.0 + m / 1024.0) * 2f32.powi(e - 15),
    }
}

/// Bake the GPU weather map around `eye` for `field`, read it back and compare
/// its cumulus coverage with `CloudField::coverage` (the same footprint the
/// thermals use). Returns (texels compared, max abs error, cells checked).
/// Panics if the two disagree.
pub fn consistency_check(device: &wgpu::Device, queue: &wgpu::Queue, r: &mut CloudRenderer, field: &CloudField, eye: Vec3) -> (usize, f32, usize) {
    r.set_field(field);
    let frame = CloudFrame {
        view_proj: Mat4::IDENTITY,
        inv_view_proj: Mat4::IDENTITY,
        rot_view_proj: Mat4::IDENTITY,
        eye,
        sun_dir: Vec3::new(0.3, 0.6, 0.4),
        sun_col: Vec3::ONE,
        sky_intensity: 1.0,
        time: 0.0,
        width: 4,
        height: 4,
        fog: [0.0, 1.0],
        ground_fog: [0.0, 0.0],
    };
    let mut enc = device.create_command_encoder(&Default::default());
    r.prepare(device, queue, &mut enc, &frame);
    let bpr = WMAP_RES * 8;
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("weather readback"),
        size: (bpr * WMAP_RES) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    enc.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo { texture: &r.weather_tex, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
        wgpu::TexelCopyBufferInfo { buffer: &buf, layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(bpr), rows_per_image: Some(WMAP_RES) } },
        wgpu::Extent3d { width: WMAP_RES, height: WMAP_RES, depth_or_array_layers: 1 },
    );
    queue.submit([enc.finish()]);
    buf.slice(..).map_async(wgpu::MapMode::Read, |_| {});
    let _ = device.poll(wgpu::PollType::wait_indefinitely());
    let data = buf.slice(..).get_mapped_range().expect("map weather");
    let px: &[u16] = bytemuck::cast_slice(&data);
    let origin = r.wmap_origin.unwrap();
    let texel_xz = |ix: u32, iz: u32| origin + (vec2(ix as f32, iz as f32) + 0.5 - WMAP_RES as f32 * 0.5) * WMAP_TEXEL;
    let mut n = 0;
    let mut max_err: f32 = 0.0;
    // Every 7th texel inside the unfaded disc.
    for iz in (0..WMAP_RES).step_by(7) {
        for ix in (0..WMAP_RES).step_by(7) {
            let xz = texel_xz(ix, iz);
            if xz.distance(origin) > 0.78 * 0.5 * WMAP_RES as f32 * WMAP_TEXEL {
                continue;
            }
            let gpu = f16_to_f32(px[((iz * WMAP_RES + ix) * 4) as usize]);
            let cpu = field.coverage_height_field(xz).0;
            let err = (gpu - cpu).abs();
            assert!(err < 0.01, "coverage mismatch at {xz}: gpu {gpu} cpu {cpu}");
            max_err = max_err.max(err);
            n += 1;
        }
    }
    // Every thermal centre near the eye must sit under a full cumulus footprint
    // (when cumulus are visible), and the cloud height channel must match.
    let mut cells = 0;
    for c in field.cells_near_field(vec2(eye.x, eye.z) - field.drift, 15000.0) {
        let t = (c.center - origin) / WMAP_TEXEL + WMAP_RES as f32 * 0.5 - 0.5;
        let (ix, iz) = (t.x.round() as u32, t.y.round() as u32);
        let i = ((iz * WMAP_RES + ix) * 4) as usize;
        let (cov, h) = (f16_to_f32(px[i]), f16_to_f32(px[i + 1]) * 1000.0);
        if field.visible() {
            let (texel_cov, texel_h) = field.coverage_height_field(texel_xz(ix, iz));
            assert!(texel_cov > 0.9 && (cov - texel_cov).abs() < 0.01, "no cloud drawn over thermal at {}: {cov}", c.center);
            assert!((h - texel_h).abs() < 0.01 * texel_h + 2.0, "height {h} vs {texel_h}");
        } else {
            assert!(cov == 0.0);
        }
        cells += 1;
    }
    drop(data);
    (n, max_err, cells)
}

// ------------------------------------------------------------------ tests

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec3;

    #[test]
    fn deterministic() {
        let a = CloudField::new(42, Weather::Scattered);
        let b = CloudField::new(42, Weather::Scattered);
        for i in 0..200 {
            let p = vec3(i as f32 * 377.0 - 30000.0, 600.0, i as f32 * -211.0 + 9000.0);
            assert_eq!(a.updraft(p), b.updraft(p));
        }
        let c = CloudField::new(43, Weather::Scattered);
        let differs = (0..50).any(|i| {
            let p = vec3(i as f32 * 900.0, 600.0, 0.0);
            a.updraft(p) != c.updraft(p)
        });
        assert!(differs, "seed must change the field");
    }

    #[test]
    fn thermal_under_every_cumulus() {
        for w in [Weather::Few, Weather::Scattered, Weather::Broken] {
            let f = CloudField::new(7, w);
            let cells = f.cells_near(Vec2::ZERO, 30000.0);
            assert!(cells.len() > 20, "{w:?}: {} cells", cells.len());
            for c in &cells {
                let up = f.updraft(vec3(c.center.x, 0.7 * f.base(), c.center.y));
                assert!((0.5..=4.8).contains(&up), "{w:?}: core updraft {up} for {c:?}");
                // Bigger clouds → stronger thermals.
                assert!((c.strength - (1.2 + 2.8 * c.size) * f.p.thermal_gain).abs() < 1e-4);
                // The cumulus footprint really is there.
                assert!(f.coverage(c.center) > 0.99);
            }
            // Between clouds the air gently sinks.
            let mut sinks = 0;
            let mut n = 0;
            for i in 0..400 {
                let p = vec2(i as f32 * 173.3 - 20000.0, (i * 7919 % 400) as f32 * 101.0 - 20000.0);
                if f.coverage(p) == 0.0 && f.thermals_near(p, 800.0).is_empty() {
                    n += 1;
                    let w = f.updraft(vec3(p.x, 600.0, p.y));
                    assert!((-1.0..0.0).contains(&w), "{w}");
                    sinks += 1;
                }
            }
            assert!(n > 10 && sinks == n);
        }
    }

    #[test]
    fn thermals_near_is_consistent_with_updraft() {
        let f = CloudField::new(99, Weather::Scattered);
        let ts = f.thermals_near(vec2(1234.0, -5678.0), 15000.0);
        assert!(!ts.is_empty());
        for (c, r, s) in ts {
            assert!(r >= 110.0 && r <= 330.0);
            assert!((1.0..=4.0).contains(&s));
            let at = f.updraft(vec3(c.x, 500.0, c.y));
            let off = f.updraft(vec3(c.x + 3.0 * r, 500.0, c.y));
            assert!(at > off + 0.5, "core {at} vs off {off}");
        }
    }

    #[test]
    fn no_lift_above_tops_and_on_ground() {
        let f = CloudField::new(3, Weather::Scattered);
        for c in f.cells_near(Vec2::ZERO, 10000.0) {
            assert_eq!(f.updraft(vec3(c.center.x, f.top() + 200.0, c.center.y)), 0.0);
            assert_eq!(f.updraft(vec3(c.center.x, 0.0, c.center.y)), 0.0);
            // Still rising inside the cloud body.
            assert!(f.updraft(vec3(c.center.x, f.base() + c.height * 0.3, c.center.y)) > 0.5);
        }
    }

    /// Cross-language check: the WGSL weather map (where cumulus are drawn)
    /// against the Rust field (where thermals are). Needs a GPU; skipped without.
    #[test]
    fn gpu_weather_map_matches_cpu_field() {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
        let Ok(adapter) = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            apply_limit_buckets: false,
        })) else {
            eprintln!("no GPU adapter: skipping");
            return;
        };
        let (device, queue) = pollster::block_on(adapter.request_device(&Default::default())).expect("device");
        let atmos = crate::atmosphere::Atmosphere::new(&device);
        let mut r = CloudRenderer::new(&device, &queue, atmos.cloud_views());
        for (seed, w, eye) in [
            (1234, Weather::Scattered, vec3(500.0, 800.0, -300.0)),
            (7, Weather::Few, vec3(-52_000.0, 1500.0, 81_000.0)),
            (99, Weather::Broken, vec3(3.0e5, 2000.0, -2.2e5)),
            (5, Weather::Clear, vec3(0.0, 500.0, 0.0)),
        ] {
            let f = CloudField::new(seed, w);
            let (n, err, cells) = consistency_check(&device, &queue, &mut r, &f, eye);
            assert!(n > 5000 && cells > 10, "{w:?}: {n} texels, {cells} cells");
            assert!(err < 0.01, "{w:?}: {err}");
        }
    }

    #[test]
    fn clear_has_blue_thermals_overcast_weak() {
        let clear = CloudField::new(5, Weather::Clear);
        assert!(!clear.thermals_near(Vec2::ZERO, 20000.0).is_empty());
        assert_eq!(clear.coverage(clear.thermals_near(Vec2::ZERO, 20000.0)[0].0), 0.0);
        let oc = CloudField::new(5, Weather::Overcast);
        for (_, _, s) in oc.thermals_near(Vec2::ZERO, 20000.0) {
            assert!(s <= 1.5);
        }
    }
}
