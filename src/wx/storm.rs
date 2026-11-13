//! Thunderstorms: cumulonimbus cells with anvils, overshooting tops, rain
//! shafts / virga, updrafts, downdrafts and the gust front, lightning.
//!
//! Like the cumulus (clouds.rs) the cells live in *field space* on a jittered
//! grid that drifts with the wind (world xz = field xz + drift), so the CPU
//! (winds, turbulence, radar, lightning, rain at the aircraft) and the GPU
//! (`storm` weather map in clouds.wgsl, built from the same [`Cell`] list)
//! agree on where every storm is. Everything is a pure function of the spec,
//! the seed and the time: lightning is a deterministic Poisson process per
//! cell (no state to save, same flashes for every observer).

use glam::{Vec2, Vec3, vec2, vec3};

/// The convective weather to build storms from (weather.rs → `StormSpec::from_wx`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StormSpec {
    /// Share of grid cells holding a storm (0 = none).
    pub amount: f32,
    /// 0..1: towering cumulus (≈0.35) .. thunderstorm (0.75) .. heavy (1).
    pub strength: f32,
    /// Cloud base above the ground (m) and tops (m above the ground).
    pub base: f32,
    pub top: f32,
    /// Steering wind (cells move with it; m/s world xz) and the upper wind
    /// that blows the anvils out (direction of motion, m/s).
    pub upper: Vec2,
    /// Dry air below the base: rain evaporates before it reaches the ground (0..1).
    pub dry: f32,
    /// Hail in the cores.
    pub hail: bool,
    /// Lightning: flashes per minute per mature cell (0 = no thunder: TCU).
    pub flash_rate: f32,
    /// A cell placed near the start (the METAR's thunderstorm "at the
    /// station"): distance from the origin (m), 0 = none.
    pub home_km: f32,
}

impl StormSpec {
    pub const NONE: StormSpec = StormSpec { amount: 0.0, strength: 0.0, base: 1200.0, top: 9000.0, upper: Vec2::ZERO, dry: 0.0, hail: false, flash_rate: 0.0, home_km: 0.0 };

    pub fn active(&self) -> bool {
        self.amount > 0.0 && self.strength > 0.05
    }

    /// Storms from the decoded weather: METAR TS / CB / TCU / +TSRA / VCTS / GR,
    /// Open-Meteo thunderstorm codes and CAPE.
    pub fn from_wx(wx: &crate::weather::Wx) -> StormSpec {
        use crate::weather::Cover;
        let mut s = StormSpec::NONE;
        let metar = wx.metar.as_ref();
        let has = |c: &str| metar.is_some_and(|m| m.wx.iter().any(|g| g.contains(c)));
        let heavy_ts = metar.is_some_and(|m| m.wx.iter().any(|g| g.starts_with("+") && g.contains("TS")));
        let light_ts = metar.is_some_and(|m| m.wx.iter().any(|g| g.starts_with("-") && g.contains("TS")));
        let vcts = metar.is_some_and(|m| m.wx.iter().any(|g| g.starts_with("VCTS")));
        let cb = wx.layers.iter().find(|l| l.convective);
        let cb_tcu = |tcu: bool| metar.is_some_and(|m| m.raw.split_whitespace().any(|t| (t.ends_with("TCU") == tcu) && (t.ends_with("CB") || t.ends_with("TCU"))));
        let om_ts = wx.om.as_ref().is_some_and(|o| o.code >= 95);
        let thunder = wx.thunder || vcts || om_ts;
        if thunder {
            s.strength = if heavy_ts {
                1.0
            } else if light_ts {
                0.62
            } else {
                0.8
            };
            s.flash_rate = if heavy_ts {
                6.0
            } else if light_ts {
                1.5
            } else {
                3.0
            };
        } else if cb.is_some() {
            // CB without thunder reported (yet): mature storm cells about.
            s.strength = if cb_tcu(false) { 0.65 } else { 0.38 };
            s.flash_rate = if cb_tcu(false) { 1.5 } else { 0.0 };
        } else if wx.showers && wx.cape > 1200.0 {
            s.strength = 0.45;
            s.flash_rate = 0.0;
        } else {
            return s;
        }
        // Instability makes taller, stronger storms.
        let cape = (wx.cape / 2500.0).clamp(0.0, 1.0);
        s.strength = (s.strength + 0.15 * cape * (s.strength > 0.5) as u8 as f32).min(1.0);
        s.amount = match cb.map(|l| l.cover) {
            Some(Cover::Few) => 0.22,
            Some(Cover::Sct) => 0.36,
            Some(Cover::Bkn) => 0.5,
            Some(Cover::Ovc | Cover::Vv) => 0.6,
            None => 0.3,
        };
        if thunder && s.amount < 0.3 {
            s.amount = 0.3;
        }
        s.base = cb.or(wx.layers.first()).map(|l| (l.base_ft * 0.3048).max(250.0)).unwrap_or(1200.0);
        s.top = (s.base + 4500.0 + 7500.0 * s.strength + 2000.0 * cape).min(16_000.0);
        s.upper = wx.wind_aloft() * 2.2 + crate::weather::wind_vec(wx.aloft_from + 20.0, 25.0);
        s.dry = ((wx.temp_c - wx.dew_c - 6.0) / 14.0).clamp(0.0, 1.0);
        s.hail = has("GR") || has("GS") || (heavy_ts && cape > 0.6);
        s.home_km = if !thunder && cb.is_none() {
            0.0
        } else if vcts && !wx.thunder {
            13.0
        } else if wx.thunder {
            7.0
        } else {
            10.0
        };
        s
    }
}

/// One storm cell (field space; heights absolute world y).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cell {
    pub id: u32,
    pub center: Vec2,
    /// Updraft tower radius (m).
    pub radius: f32,
    /// Cloud base and anvil top (world y, m); overshooting dome height above it.
    pub base: f32,
    pub top: f32,
    pub overshoot: f32,
    /// 0..1 maturity / strength of this cell.
    pub intensity: f32,
    /// Unit direction the anvil streams toward and its length (m).
    pub anvil_dir: Vec2,
    pub anvil_len: f32,
    /// Rain core: centre offset from the tower (m, field xz), radius, rain
    /// rate (mm/h) and the height the shaft ends (world y; ground = virga-free).
    pub rain_off: Vec2,
    pub rain_r: f32,
    pub rain_mmh: f32,
    pub virga_bottom: f32,
}

impl Cell {
    pub fn rain_center(&self) -> Vec2 {
        self.center + self.rain_off
    }
    /// Anvil base (world y).
    pub fn anvil_base(&self) -> f32 {
        self.top - (1800.0 + 1800.0 * self.intensity).min((self.top - self.base) * 0.45)
    }
    /// Centre of the tower at height y (field xz): it leans downshear, along
    /// the anvil (== STORM_TILT in clouds.wgsl).
    pub fn tower_axis(&self, y: f32) -> Vec2 {
        self.center + self.anvil_dir * TILT * (y - self.base).clamp(0.0, (self.top - self.base).max(0.0))
    }
    /// Tower footprint radius at height y (the anvil flares out at the top).
    pub fn tower_radius(&self, y: f32) -> f32 {
        let h = ((y - self.base) / (self.top - self.base)).clamp(0.0, 1.2);
        self.radius * (0.82 + 0.25 * h)
    }
    /// Anvil coverage 0..1 at field xz: a plume from the (leaning) tower top
    /// spreading downshear (the outline clouds.wgsl draws, without its ragged edge).
    pub fn anvil(&self, xz: Vec2) -> f32 {
        if self.anvil_len <= 0.0 {
            return 0.0;
        }
        let d = xz - self.tower_axis(self.top);
        let s = d.dot(self.anvil_dir);
        let n = d.perp_dot(self.anvil_dir).abs();
        let back = self.radius * 1.15;
        if s < -back || s > self.anvil_len {
            return 0.0;
        }
        let width = self.radius * 1.15 + s.max(0.0) * 0.5;
        let along = if s < 0.0 { -s / back } else { (s - self.anvil_len * 0.45).max(0.0) / (self.anvil_len * 0.55) };
        1.0 - smooth(0.7, 1.0, vec2(n / width, along).length())
    }
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn lowbias32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

fn h3(seed: u32, a: i32, b: i32, k: u32) -> f32 {
    let x = lowbias32(lowbias32(lowbias32(seed ^ (a as u32)).wrapping_add(b as u32)).wrapping_add(k.wrapping_mul(0x9e37_79b9)));
    (x >> 8) as f32 * (1.0 / 16_777_216.0)
}

/// How far the towers lean downshear (m per m of height).
pub const TILT: f32 = 0.09;

/// Grid cell size (m): at most one storm per cell.
pub const GRID: f32 = 16_000.0;
/// The GPU weather map takes at most this many cells (nearest the eye).
pub const MAX_GPU_CELLS: usize = 24;

/// Lightning flash type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashKind {
    /// Cloud-to-ground: a visible channel from the base to the ground, 1-5 return strokes.
    Ground,
    /// Intra-cloud: lights the cloud from inside, channel mostly hidden.
    Cloud,
}

/// One lightning flash (world space at the time asked for).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Flash {
    pub key: u64,
    pub kind: FlashKind,
    /// Start time (sim s).
    pub t0: f32,
    /// Charge centre the flash starts from (world), and where a CG flash strikes.
    pub origin: Vec3,
    pub strike: Vec3,
    /// Return strokes (CG) / bursts (IC) and their spacing (s).
    pub strokes: u8,
    pub gap: f32,
    /// Relative energy (0.5..2) and the deterministic seed of its channel shape.
    pub energy: f32,
    pub seed: u64,
}

impl Flash {
    /// Total duration (s).
    pub fn duration(&self) -> f32 {
        self.gap * self.strokes as f32 + 0.25
    }
    /// Brightness 0..~1 at `t` (sim s): return strokes (sharp, decaying in
    /// ~15 ms, continuing current between) or the flicker of a cloud flash.
    pub fn brightness(&self, t: f32) -> f32 {
        let dt = t - self.t0;
        if dt < 0.0 || dt > self.duration() {
            return 0.0;
        }
        let mut b: f32 = 0.0;
        for i in 0..self.strokes {
            let ts = i as f32 * self.gap * (0.7 + 0.6 * frac(self.seed as f32 * 0.37 + i as f32 * 0.61));
            let x = dt - ts;
            if x >= 0.0 {
                let amp = if i == 0 { 1.0 } else { 0.55 + 0.4 * frac(self.seed as f32 * 0.13 + i as f32 * 0.29) };
                b = b.max(amp * ((-x / 0.018).exp() + 0.12 * (-x / 0.09).exp()));
            }
        }
        // Frames are ~16 ms apart: never let a stroke fall between two frames entirely.
        (b * self.energy).min(2.0)
    }
}

fn frac(x: f32) -> f32 {
    x - x.floor()
}

/// The storm field.
#[derive(Clone, Debug, PartialEq)]
pub struct StormField {
    pub spec: StormSpec,
    seed: u32,
    /// Ground level the heights refer to (World::terrain_base).
    ground: f32,
    /// Field → world offset (the cloud drift).
    drift: Vec2,
    /// Home cell centre (field space) when the spec places one.
    home: Option<Vec2>,
}

impl StormField {
    pub fn new(seed: u32, spec: StormSpec) -> StormField {
        let home = (spec.active() && spec.home_km > 0.0).then(|| {
            let a = h3(seed, 7, 7, 1) * std::f32::consts::TAU;
            vec2(a.cos(), a.sin()) * spec.home_km * 1000.0
        });
        StormField { spec, seed, ground: 0.0, drift: Vec2::ZERO, home }
    }
    pub fn none() -> StormField {
        StormField::new(1, StormSpec::NONE)
    }
    pub fn active(&self) -> bool {
        self.spec.active()
    }
    pub fn set_drift(&mut self, d: Vec2) {
        if d.is_finite() {
            self.drift = d;
        }
    }
    pub fn drift(&self) -> Vec2 {
        self.drift
    }
    /// Heights are above this ground (m).
    pub fn set_ground(&mut self, g: f32) {
        self.ground = g;
    }
    /// Highest point any storm reaches (world y).
    pub fn top(&self) -> f32 {
        if self.active() { self.ground + self.spec.top + 1500.0 } else { 0.0 }
    }

    fn make(&self, id: u32, center: Vec2, size: f32, mature: f32) -> Cell {
        let s = &self.spec;
        let st = s.strength * (0.55 + 0.45 * mature);
        let radius = 1600.0 + 2600.0 * st * (0.6 + 0.4 * size);
        let base = self.ground + s.base;
        let top = self.ground + s.base + (s.top - s.base) * (0.72 + 0.28 * size) * (0.75 + 0.25 * mature);
        let up = if s.upper.length() > 0.5 { s.upper.normalize() } else { Vec2::X };
        let upper_spd = s.upper.length();
        let anvil = st > 0.45;
        let rain = 25.0 + 95.0 * st * st * (0.5 + 0.5 * mature);
        let dry = s.dry * (1.0 - 0.6 * st);
        Cell {
            id,
            center,
            radius,
            base,
            top,
            overshoot: if st > 0.6 { 500.0 + 1300.0 * (st - 0.6) / 0.4 * mature } else { 0.0 },
            intensity: st,
            anvil_dir: up,
            anvil_len: if anvil { (6000.0 + upper_spd * 400.0) * (0.5 + 0.5 * mature) * (0.6 + 0.6 * st) } else { 0.0 },
            rain_off: up * radius * 0.45,
            rain_r: radius * 0.85,
            rain_mmh: rain * if s.hail { 1.15 } else { 1.0 },
            virga_bottom: self.ground + s.base * 0.85 * dry * dry,
        }
    }

    /// The cell of grid square (ix, iz), if it holds one.
    pub fn cell(&self, ix: i32, iz: i32) -> Option<Cell> {
        if !self.active() {
            return None;
        }
        if u01(self.seed, ix, iz, 0) >= self.spec.amount {
            return None;
        }
        let jx = 0.15 + 0.7 * u01(self.seed, ix, iz, 1);
        let jz = 0.15 + 0.7 * u01(self.seed, ix, iz, 2);
        let center = vec2((ix as f32 + jx) * GRID, (iz as f32 + jz) * GRID);
        // Keep grid cells clear of the home cell (no two storms merged).
        if let Some(h) = self.home
            && center.distance(h) < GRID * 0.7
        {
            return None;
        }
        let size = u01(self.seed, ix, iz, 3);
        let mature = 0.45 + 0.55 * u01(self.seed, ix, iz, 4);
        Some(self.make((((ix + 0x8000) as u32) & 0xffff) << 16 | (((iz + 0x8000) as u32) & 0xffff), center, size, mature))
    }

    /// Cells whose influence can reach a disc around world `xz` (field space positions).
    pub fn cells_near_field(&self, xz_field: Vec2, radius: f32) -> Vec<Cell> {
        let mut v = Vec::new();
        if !self.active() {
            return v;
        }
        if let Some(h) = self.home {
            let c = self.make(0xffff_ffff, h, 0.85, 1.0);
            if c.center.distance(xz_field) < radius + c.anvil_len + c.radius * 3.0 {
                v.push(c);
            }
        }
        let r = radius + 40_000.0;
        let (x0, z0) = (((xz_field.x - r) / GRID).floor() as i32, ((xz_field.y - r) / GRID).floor() as i32);
        let (x1, z1) = (((xz_field.x + r) / GRID).floor() as i32, ((xz_field.y + r) / GRID).floor() as i32);
        for iz in z0..=z1 {
            for ix in x0..=x1 {
                if let Some(c) = self.cell(ix, iz)
                    && c.center.distance(xz_field) < radius + c.anvil_len + c.radius * 3.0
                {
                    v.push(c);
                }
            }
        }
        v
    }

    /// Cells near world `xz` (positions still in field space).
    pub fn cells_near(&self, xz: Vec2, radius: f32) -> Vec<Cell> {
        self.cells_near_field(xz - self.drift, radius)
    }

    /// The (up to [`MAX_GPU_CELLS`]) cells nearest the eye, for the weather map.
    pub fn gpu_cells(&self, eye_xz: Vec2) -> Vec<Cell> {
        let f = eye_xz - self.drift;
        let mut v = self.cells_near_field(f, 85_000.0);
        v.sort_by(|a, b| a.center.distance(f).total_cmp(&b.center.distance(f)));
        v.truncate(MAX_GPU_CELLS);
        v
    }

    // -------------------------------------------------------------- the air

    /// What the storms do at world point `p`: vertical air velocity (m/s),
    /// horizontal outflow (gust front, m/s world xz), turbulence severity
    /// (0..4), cloud (0..1: inside a CB), rain rate (mm/h) and radar
    /// reflectivity (dBZ, -inf = nothing).
    pub fn air(&self, p: Vec3) -> StormAir {
        let mut out = StormAir::default();
        if !self.active() || !p.is_finite() {
            return out;
        }
        let f = vec2(p.x, p.z) - self.drift;
        for c in self.cells_near_field(f, 20_000.0) {
            self.cell_air_pub(&c, f, p.y, &mut out);
        }
        out
    }

    pub(crate) fn cell_air_pub(&self, c: &Cell, f: Vec2, y: f32, out: &mut StormAir) {
        let st = c.intensity;
        let d_tower = f.distance(c.tower_axis(y));
        let rc = c.rain_center();
        let d_rain = f.distance(rc);
        let h_rel = (y - c.base) / (c.top - c.base);
        let gy = y - self.ground;
        // ---- cloud volume (tower + anvil + dome)
        let tr = c.tower_radius(y);
        let in_tower =
            if y >= c.base - 50.0 && y <= c.top + c.overshoot * (1.0 - (d_tower / (c.radius * 0.6)).powi(2)).max(0.0) { 1.0 - smooth(tr * 0.75, tr * 1.05, d_tower) } else { 0.0 };
        let ab = c.anvil_base();
        let in_anvil = if y >= ab && y <= c.top { c.anvil(f) * smooth(ab, ab + 300.0, y) } else { 0.0 };
        let cloud = in_tower.max(in_anvil);
        out.cloud = out.cloud.max(cloud);
        // ---- updraft: tower core (strongest mid-levels), inflow below the base on the forward flank
        let core = (-(d_tower / (c.radius * 0.55)).powi(2)).exp();
        let w_up = (8.0 + 22.0 * st) * core;
        let up = if h_rel >= 0.0 && h_rel <= 1.0 {
            w_up * (std::f32::consts::PI * (0.15 + 0.85 * h_rel)).sin().max(0.15)
        } else if h_rel < 0.0 {
            // Inflow: weaker, dying out near the ground.
            w_up * 0.35 * smooth(0.0, 400.0, gy)
        } else {
            0.0
        };
        // ---- downdraft in the rain core: strongest a few hundred metres up, spreading at the ground
        let rcore = (-(d_rain / (c.rain_r * 0.6)).powi(2)).exp();
        let dd_mag = (4.0 + 14.0 * st) * rcore;
        let down = if y < c.base + (c.top - c.base) * 0.55 && y > c.virga_bottom.min(self.ground + 1.0) - 1.0 {
            -dd_mag * smooth(0.0, 350.0, gy) * (1.0 - smooth(c.base, c.base + (c.top - c.base) * 0.55, y))
        } else {
            0.0
        };
        out.updraft += up + down;
        // ---- outflow / gust front: radial wind spreading from under the rain core
        if gy < 1500.0 {
            let r = d_rain / c.rain_r;
            let prof = r * (-(r / 1.6).powi(2)).exp() * 1.4; // peaks ~1.1 r_rain, gone by ~4 r_rain
            let lift = (-gy / 450.0).exp() * smooth(0.0, 25.0, gy + 25.0);
            let dir = if d_rain > 1.0 { (f - rc) / d_rain } else { Vec2::ZERO };
            let spd = (6.0 + 16.0 * st) * prof * lift;
            out.outflow += dir * spd;
            // The gust front itself (leading edge, ~2-3 r): turbulent, lifting.
            let front = (-((r - 2.2) / 0.5).powi(2)).exp() * lift;
            out.severity = out.severity.max(front * (1.5 + 1.5 * st));
            out.updraft += front * (1.5 + 3.0 * st) * smooth(0.0, 300.0, gy) * (1.0 - smooth(600.0, 1400.0, gy));
        }
        // ---- turbulence: severe in the cloud, strongest in the updraft core; light-moderate near it / under the anvil
        let near = 1.0 - smooth(tr, tr + 3500.0, d_tower);
        let sev_cloud = cloud * (2.4 + 1.2 * st + 0.6 * core) + near * (1.0 + st) * (h_rel > -0.2 && h_rel < 1.1) as u8 as f32;
        out.severity = out.severity.max(sev_cloud.min(4.0));
        // ---- rain (+ radar reflectivity)
        let rain_prof = (-(d_rain / (c.rain_r * 0.75)).powi(2)).exp();
        if y < c.base && y > c.virga_bottom {
            out.rain_mmh += c.rain_mmh * rain_prof;
        }
        // Reflectivity: rain shaft below the base, the core aloft (strongest
        // low-mid levels, falling off above the freezing level), weak in the anvil.
        let mut z = 0.0f32;
        if y < c.base && y > c.virga_bottom {
            z += 200.0 * (c.rain_mmh * rain_prof).powf(1.6);
        }
        if y >= c.base && y <= c.top + c.overshoot {
            let core_z = 200.0 * (c.rain_mmh * 1.1).powf(1.6) * (-(d_tower.min(d_rain) / (c.radius * 0.8)).powi(2)).exp();
            z += core_z * (1.0 - smooth(0.35, 1.0, h_rel)) * if self.spec.hail { 2.5 } else { 1.0 };
            z += 60.0 * in_anvil + 200.0 * in_tower * (st * 20.0);
        }
        out.z_lin += z;
    }

    // -------------------------------------------------------------- lightning

    /// Flashes (world space) that started in (t - window, t], from cells within
    /// `range` m of world `xz`. `rate_scale` scales every cell's flash rate.
    pub fn flashes(&self, t: f32, xz: Vec2, range: f32, window: f32) -> Vec<Flash> {
        let mut out = Vec::new();
        if !self.active() || self.spec.flash_rate <= 0.0 || !t.is_finite() {
            return out;
        }
        const SLOT: f32 = 0.25;
        let s1 = (t / SLOT).floor() as i64;
        let s0 = ((t - window) / SLOT).floor() as i64;
        for c in self.cells_near(xz, range).into_iter().filter(|c| (c.center + self.drift).distance(xz) < range) {
            let rate = self.spec.flash_rate / 60.0 * (0.3 + 0.7 * c.intensity * c.intensity);
            let p_slot = (rate * SLOT).min(0.8);
            for s in s0..=s1 {
                let key = (c.id as u64) << 32 ^ (s as u64 & 0xffff_ffff);
                let r = |k: u32| u01(self.seed ^ 0x51ab, (key >> 16) as i32, key as i32, k);
                if r(0) >= p_slot {
                    continue;
                }
                let t0 = (s as f32 + r(1)) * SLOT;
                if t0 > t || t0 <= t - window {
                    continue;
                }
                out.push(self.flash_of(&c, key, t0, &r));
            }
        }
        out
    }

    fn flash_of(&self, c: &Cell, key: u64, t0: f32, r: &dyn Fn(u32) -> f32) -> Flash {
        let ground_frac = 0.25 + 0.15 * c.intensity;
        let kind = if r(2) < ground_frac { FlashKind::Ground } else { FlashKind::Cloud };
        let a = r(3) * std::f32::consts::TAU;
        let rr = c.radius * (0.2 + 0.7 * r(4));
        let oxz = c.center + vec2(a.cos(), a.sin()) * rr + self.drift;
        // Negative charge centre ~ the -10..-25 °C level: lower-middle of the cloud.
        let oy = c.base + (c.top - c.base) * (0.25 + 0.3 * r(5));
        let origin = vec3(oxz.x, oy, oxz.y);
        // CG strikes come down under the cloud (often near the rain shaft); a
        // few "bolts from the blue" land out under the anvil.
        let sa = r(6) * std::f32::consts::TAU;
        let sr = if r(7) < 0.1 { c.radius * 2.5 + c.anvil_len * 0.3 * r(8) } else { c.rain_r * 0.9 * r(8) };
        let sxz = c.rain_center() + vec2(sa.cos(), sa.sin()) * sr + self.drift;
        let strike = vec3(sxz.x, self.ground, sxz.y);
        let strokes = match kind {
            FlashKind::Ground => 1 + (r(9) * 4.5) as u8,
            FlashKind::Cloud => 2 + (r(9) * 4.0) as u8,
        };
        Flash {
            key,
            kind,
            t0,
            origin,
            strike,
            strokes,
            gap: if kind == FlashKind::Ground { 0.045 + 0.04 * r(10) } else { 0.07 + 0.06 * r(10) },
            energy: 0.6 + 1.0 * r(11) * (0.5 + 0.5 * c.intensity),
            seed: key ^ ((self.seed as u64) << 40),
        }
    }

    /// A flash forced at `t` from the cell nearest world `xz` (screenshots /
    /// tests: `SKYWARD_FLASH=cg|ic`).
    pub fn forced_flash(&self, t: f32, xz: Vec2, kind: FlashKind) -> Option<Flash> {
        let f = xz - self.drift;
        let c = self.cells_near_field(f, 60_000.0).into_iter().min_by(|a, b| a.center.distance(f).total_cmp(&b.center.distance(f)))?;
        let key = 0xf1a5_u64 << 40 | c.id as u64;
        let r = |k: u32| u01(self.seed ^ 0x77, c.id as i32, 99, k);
        let mut fl = self.flash_of(&c, key, t - 0.004, &r);
        fl.kind = kind;
        fl.energy = 1.4;
        fl.strokes = fl.strokes.max(3);
        Some(fl)
    }
}

fn u01(seed: u32, a: i32, b: i32, k: u32) -> f32 {
    h3(seed, a, b, k)
}

/// The storms' effect on the air at a point (see [`StormField::air`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StormAir {
    pub updraft: f32,
    pub outflow: Vec2,
    pub severity: f32,
    pub cloud: f32,
    pub rain_mmh: f32,
    /// Linear reflectivity factor Z (mm⁶/m³).
    pub z_lin: f32,
}

impl Default for StormAir {
    fn default() -> Self {
        StormAir { updraft: 0.0, outflow: Vec2::ZERO, severity: 0.0, cloud: 0.0, rain_mmh: 0.0, z_lin: 0.0 }
    }
}

impl StormAir {
    pub fn dbz(&self) -> f32 {
        if self.z_lin <= 1e-3 { -99.0 } else { 10.0 * self.z_lin.log10() }
    }
}

// ------------------------------------------------------------------ lightning channel

/// One segment of a lightning channel: ends (world) and brightness 0..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seg {
    pub a: Vec3,
    pub b: Vec3,
    pub bright: f32,
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 40) as f32) / (1u64 << 24) as f32
    }
    fn sym(&mut self) -> f32 {
        self.next() * 2.0 - 1.0
    }
}

/// The tortuous, branching channel of a flash (deterministic from its seed).
/// CG: from the charge centre down to the strike point (stepped-leader
/// tortuosity, ~35 m steps, side branches that fade and die out); IC: a
/// spidery, mostly horizontal discharge spreading through the cloud.
pub fn channel(f: &Flash) -> Vec<Seg> {
    let mut rng = Rng(f.seed ^ 0x9e37_79b9_7f4a_7c15);
    let mut segs = Vec::new();
    match f.kind {
        FlashKind::Ground => {
            walk(&mut rng, f.origin, f.strike, 1.0, 2, &mut segs, true);
        }
        FlashKind::Cloud => {
            for _ in 0..3 {
                let a = rng.next() * std::f32::consts::TAU;
                let len = 3000.0 + 5000.0 * rng.next();
                let end = f.origin + vec3(a.cos() * len, (rng.sym() * 0.15) * len, a.sin() * len);
                walk(&mut rng, f.origin, end, 0.7, 2, &mut segs, false);
            }
        }
    }
    segs
}

fn walk(rng: &mut Rng, from: Vec3, to: Vec3, bright: f32, depth: u32, out: &mut Vec<Seg>, main: bool) {
    let total = from.distance(to).max(1.0);
    let step = (total / 90.0).clamp(18.0, 60.0);
    let mut p = from;
    let mut dir = (to - from).normalize_or_zero();
    let mut travelled = 0.0;
    let n_max = (total / step * 1.6) as usize + 4;
    for i in 0..n_max {
        let to_go = to - p;
        let dist = to_go.length();
        if dist < step * 1.2 {
            if main {
                out.push(Seg { a: p, b: to, bright });
            }
            break;
        }
        // Tortuosity: steer back toward the target, kink by ~±35°.
        let want = to_go / dist;
        let jitter = vec3(rng.sym(), rng.sym() * 0.6, rng.sym());
        dir = (dir * 0.45 + want * 0.4 + jitter * 0.5).normalize_or_zero();
        let len = step * (0.5 + rng.next());
        let q = p + dir * len;
        // Branches thin out and fade toward the end of the main channel.
        let b = if main { bright } else { bright * (1.0 - travelled / total).max(0.15) };
        out.push(Seg { a: p, b: q, bright: b });
        if depth > 0 && rng.next() < if main { 0.06 } else { 0.03 } && i > 2 {
            let side = vec3(rng.sym(), -0.4 - rng.next() * 0.6, rng.sym()).normalize_or_zero();
            let blen = (total - travelled) * (0.15 + 0.35 * rng.next());
            walk(rng, q, q + side * blen, bright * 0.45, depth - 1, out, false);
        }
        travelled += len;
        p = q;
        if out.len() > 2600 {
            break;
        }
        if !main && travelled > total {
            break;
        }
    }
}

/// Speed of sound for thunder (m/s, ~15 °C).
pub const SOUND_SPEED: f32 = 340.0;

/// When the first thunder from a flash reaches `ear` (sim s), and when the
/// rumble ends (the farthest part of the channel).
pub fn thunder_times(f: &Flash, segs: &[Seg], ear: Vec3) -> (f32, f32) {
    let (mut near, mut far) = (f32::MAX, 0.0f32);
    for s in segs {
        for p in [s.a, s.b] {
            let d = p.distance(ear);
            near = near.min(d);
            far = far.max(d);
        }
    }
    if segs.is_empty() {
        near = f.origin.distance(ear);
        far = near;
    }
    (f.t0 + near / SOUND_SPEED, f.t0 + far / SOUND_SPEED)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts_field() -> StormField {
        let wx = crate::weather::Wx::from_metar_str("EGLL 151350Z 24015G30KT 4000 +TSRA BKN012CB 22/18 Q1006").unwrap();
        let spec = StormSpec::from_wx(&wx);
        assert!(spec.active() && spec.strength > 0.9 && spec.flash_rate >= 5.0, "{spec:?}");
        StormField::new(11, spec)
    }

    #[test]
    fn metar_drives_storms() {
        let calm = crate::weather::Wx::from_metar_str("EGLL 151350Z 24008KT 9999 SCT040 18/09 Q1018").unwrap();
        assert!(!StormSpec::from_wx(&calm).active());
        let tcu = crate::weather::Wx::from_metar_str("EGLL 151350Z 24008KT 9999 SCT030TCU 18/09 Q1018").unwrap();
        let s = StormSpec::from_wx(&tcu);
        assert!(s.active() && s.flash_rate == 0.0 && s.strength < 0.5, "{s:?}");
        let cb = crate::weather::Wx::from_metar_str("EGLL 151350Z 24008KT 9999 FEW030CB 18/09 Q1018").unwrap();
        assert!(StormSpec::from_wx(&cb).flash_rate > 0.0);
        let ts = ts_field();
        // The METAR's storm is near the station, tall, with an anvil.
        let cells = ts.cells_near(Vec2::ZERO, 20_000.0);
        let home = cells.iter().find(|c| c.id == 0xffff_ffff).expect("home cell");
        assert!(home.center.length() > 4000.0 && home.center.length() < 12_000.0);
        assert!(home.top - home.base > 8000.0 && home.anvil_len > 8000.0 && home.overshoot > 0.0);
    }

    #[test]
    fn tower_leans_into_its_anvil() {
        let f = ts_field();
        let c = f.cells_near(Vec2::ZERO, 20_000.0).into_iter().find(|c| c.id == 0xffff_ffff).unwrap();
        // The tower leans downshear with height; the anvil sits on its top
        // and streams downwind, thinning out at its tip, nothing far upwind.
        let top = c.tower_axis(c.top);
        assert!((c.tower_axis(c.base) - c.center).length() < 1.0);
        assert!((top - c.center).dot(c.anvil_dir) > 500.0);
        assert!(c.anvil(top) > 0.99 && c.anvil(top + c.anvil_dir * c.anvil_len * 0.4) > 0.99);
        assert!(c.anvil(top + c.anvil_dir * c.anvil_len * 0.99) < 0.2);
        assert_eq!(c.anvil(top - c.anvil_dir * c.radius * 2.0), 0.0);
        // The updraft core follows the lean.
        let mid = c.base + (c.top - c.base) * 0.6;
        assert!(f.air(vec3(c.tower_axis(mid).x, mid, c.tower_axis(mid).y)).updraft > 15.0);
    }

    #[test]
    fn storm_air_is_sane() {
        let f = ts_field();
        let c = f.cells_near(Vec2::ZERO, 20_000.0).into_iter().find(|c| c.id == 0xffff_ffff).unwrap();
        let at = |xz: Vec2, y: f32| f.air(vec3(xz.x, y, xz.y));
        // Updraft in the tower, downdraft + heavy rain + >50 dBZ in the core below the base.
        let mid = at(c.center, c.base + (c.top - c.base) * 0.5);
        assert!(mid.updraft > 15.0 && mid.cloud > 0.9 && mid.severity >= 3.0, "{mid:?}");
        let under = at(c.rain_center(), c.base * 0.5);
        assert!(under.updraft < -3.0 && under.rain_mmh > 40.0 && under.dbz() > 50.0, "{under:?}");
        // Gust front: strong outflow at the surface away from the core.
        let dir = vec2(0.6, 0.8);
        let gf = at(c.rain_center() + dir * c.rain_r * 1.2, 30.0);
        assert!(gf.outflow.length() > 8.0 && gf.outflow.dot(dir) > 0.0, "{gf:?}");
        // Far away: nothing.
        let far = at(c.center + vec2(80_000.0, 80_000.0), 1000.0);
        assert!(far.updraft.abs() < 0.5 && far.rain_mmh == 0.0);
        // Everything finite everywhere.
        for i in 0..2000 {
            let p = vec3((i * 97 % 40_000) as f32 - 20_000.0, (i * 31 % 15_000) as f32, (i * 53 % 40_000) as f32 - 20_000.0);
            let a = f.air(p);
            assert!(a.updraft.is_finite() && a.outflow.is_finite() && a.severity.is_finite() && a.z_lin.is_finite());
        }
    }

    #[test]
    fn lightning_and_thunder_delay() {
        let f = ts_field();
        // Over 10 minutes the cells near the start flash several times a minute.
        let n = f.flashes(600.0, Vec2::ZERO, 30_000.0, 600.0).len();
        assert!(n > 20 && n < 1500, "{n} flashes in 10 min");
        // Deterministic.
        assert_eq!(f.flashes(300.0, Vec2::ZERO, 30_000.0, 5.0), f.flashes(300.0, Vec2::ZERO, 30_000.0, 5.0));
        // A forced ground flash: a channel from the cloud to the ground; thunder
        // arrives after distance / c (3 s per km) and rumbles on.
        let fl = f.forced_flash(100.0, Vec2::ZERO, FlashKind::Ground).unwrap();
        assert!(fl.brightness(fl.t0 + 0.002) > 0.5 && fl.brightness(fl.t0 + 5.0) == 0.0);
        let segs = channel(&fl);
        assert!(segs.len() > 40, "{} segments", segs.len());
        let low = segs.iter().map(|s| s.a.y.min(s.b.y)).fold(f32::MAX, f32::min);
        assert!((low - fl.strike.y).abs() < 1.0, "channel reaches the ground");
        let ear = fl.strike + vec3(5000.0, 0.0, 0.0);
        let (t_first, t_last) = thunder_times(&fl, &segs, ear);
        let d_near = segs.iter().map(|s| s.a.distance(ear).min(s.b.distance(ear))).fold(f32::MAX, f32::min);
        assert!((t_first - fl.t0 - d_near / SOUND_SPEED).abs() < 1e-3);
        assert!(t_first - fl.t0 > 13.0 && t_first - fl.t0 < 15.0, "5 km ≈ 14.7 s: {}", t_first - fl.t0);
        assert!(t_last - t_first > 2.0, "the rumble lasts: {}", t_last - t_first);
    }
}
