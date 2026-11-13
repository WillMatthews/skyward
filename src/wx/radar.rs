//! Airborne weather radar (X band) for the airliners' ND.
//!
//! A stabilised pencil beam (≈3.5°) is swept ±80° ahead of the aircraft at a
//! tilt above / below the horizon; each range bin returns the strongest
//! reflectivity the beam volume sees (rain shafts and storm cores, showers in
//! cumulus, steady rain under a deck), less the two-way path attenuation
//! through heavy rain on the way (the "radar shadow" behind a cell), plus
//! gain. Bins map to the standard colours: green (20–30 dBZ, light), yellow
//! (30–40, moderate), red (40–50, heavy), magenta (≥ 50: extreme / turbulence).
//! With the beam tilted down the ground returns too (shown dim).

use glam::{Vec2, Vec3, vec2, vec3};

pub const AZ_BINS: usize = 81;
pub const RANGE_BINS: usize = 96;
/// Sweep half-angle (rad).
pub const HALF_SWEEP: f32 = 80.0 * std::f32::consts::PI / 180.0;
/// Beam width (rad).
pub const BEAM: f32 = 3.5 * std::f32::consts::PI / 180.0;

/// Display level of a return.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    None = 0,
    Green = 1,
    Yellow = 2,
    Red = 3,
    Magenta = 4,
    /// Ground return (tilted down).
    Ground = 5,
}

pub fn level_for_dbz(dbz: f32) -> Level {
    if dbz >= 50.0 {
        Level::Magenta
    } else if dbz >= 40.0 {
        Level::Red
    } else if dbz >= 30.0 {
        Level::Yellow
    } else if dbz >= 20.0 {
        Level::Green
    } else {
        Level::None
    }
}

impl Level {
    /// ND colour (sRGB).
    pub fn rgb(self) -> [u8; 3] {
        match self {
            Level::None => [0, 0, 0],
            Level::Green => [0, 190, 40],
            Level::Yellow => [235, 220, 0],
            Level::Red => [235, 30, 20],
            Level::Magenta => [235, 0, 235],
            Level::Ground => [140, 90, 30],
        }
    }
}

/// Radar controls (the ND's WX panel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadarCtl {
    pub on: bool,
    /// Beam tilt above the horizon (deg); None = automatic (by altitude).
    pub tilt: Option<f32>,
    /// Gain offset (dB): 0 = calibrated.
    pub gain: f32,
}

impl Default for RadarCtl {
    fn default() -> Self {
        RadarCtl { on: true, tilt: None, gain: 0.0 }
    }
}

impl RadarCtl {
    /// The tilt in use (deg) at `agl` m: automatic tilt keeps the beam's lower
    /// edge just clear of the ground at ~40 NM.
    pub fn tilt_deg(&self, agl: f32) -> f32 {
        self.tilt.unwrap_or_else(|| {
            let target = 74_000.0;
            let a = (-agl / target).atan().to_degrees() + BEAM.to_degrees() * 0.5;
            a.clamp(-6.0, 4.0)
        })
    }
}

/// One sweep: `bins[az * RANGE_BINS + r]`, azimuth from -HALF_SWEEP (left) to
/// +HALF_SWEEP relative to the heading, range 0..`range` m.
#[derive(Clone, Debug, PartialEq)]
pub struct RadarImage {
    pub bins: Vec<Level>,
    pub range: f32,
    pub tilt: f32,
    pub gain: f32,
}

impl RadarImage {
    pub fn empty(range: f32) -> RadarImage {
        RadarImage { bins: vec![Level::None; AZ_BINS * RANGE_BINS], range, tilt: 0.0, gain: 0.0 }
    }
    pub fn at(&self, az: usize, r: usize) -> Level {
        self.bins[az * RANGE_BINS + r]
    }
    /// Count of bins at each level (tests / HUD).
    pub fn histogram(&self) -> [usize; 6] {
        let mut h = [0; 6];
        for b in &self.bins {
            h[*b as usize] += 1;
        }
        h
    }
}

/// Sweep the beam from `pos` along `heading` (rad, 0 = north) out to `range` m.
/// `dbz(p)` gives the reflectivity at a world point, `ground(xz)` the terrain.
pub fn scan(pos: Vec3, heading: f32, ctl: &RadarCtl, range: f32, dbz: &dyn Fn(Vec3) -> f32, ground: &dyn Fn(Vec2) -> f32) -> RadarImage {
    let mut img = RadarImage::empty(range);
    scan_columns(pos, heading, ctl, range, dbz, ground, 0, AZ_BINS, &mut img);
    img
}

/// Sweep `cols` azimuth columns from `start` into `img` (the radar sweeps
/// across the frames); returns the next column.
#[allow(clippy::too_many_arguments)]
pub fn scan_columns(
    pos: Vec3,
    heading: f32,
    ctl: &RadarCtl,
    range: f32,
    dbz: &dyn Fn(Vec3) -> f32,
    ground: &dyn Fn(Vec2) -> f32,
    start: usize,
    cols: usize,
    img: &mut RadarImage,
) -> usize {
    let agl = pos.y - ground(vec2(pos.x, pos.z));
    let tilt = ctl.tilt_deg(agl).to_radians();
    img.tilt = tilt.to_degrees();
    img.gain = ctl.gain;
    img.range = range;
    let dr = range / RANGE_BINS as f32;
    let end = (start + cols).min(AZ_BINS);
    for a in start..end {
        if !ctl.on {
            for r in 0..RANGE_BINS {
                img.bins[a * RANGE_BINS + r] = Level::None;
            }
            continue;
        }
        let az = heading + (a as f32 / (AZ_BINS - 1) as f32 * 2.0 - 1.0) * HALF_SWEEP;
        let dir = vec2(az.sin(), -az.cos());
        let mut path_db = 0.0f32;
        for r in 0..RANGE_BINS {
            let d = (r as f32 + 0.5) * dr;
            let xz = vec2(pos.x, pos.z) + dir * d;
            // Beam centre and edges (earth curvature: the beam rises d²/2R' over a 4/3 earth).
            let curve = d * d / (2.0 * 8.5e6);
            let yc = pos.y + d * tilt.tan() + curve;
            let half = d * (BEAM * 0.5).tan();
            let mut best = -99.0f32;
            for k in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
                let y = yc + k * half;
                best = best.max(dbz(vec3(xz.x, y, xz.y)));
            }
            let g = ground(xz);
            // Two-way specific attenuation in rain (X band): ~2 x 1e-4 Z^0.78 dB/km.
            if best > 10.0 {
                let z = 10f32.powf(best / 10.0);
                path_db += 2.0 * 1.0e-4 * z.powf(0.78) * dr * 0.001;
            }
            let shown = best - path_db.min(60.0) + ctl.gain;
            let mut lvl = level_for_dbz(shown);
            if lvl == Level::None && yc - half < g && d > 2000.0 {
                lvl = Level::Ground;
            }
            img.bins[a * RANGE_BINS + r] = lvl;
        }
    }
    end
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returns_map_to_colours() {
        assert_eq!(level_for_dbz(15.0), Level::None);
        assert_eq!(level_for_dbz(25.0), Level::Green);
        assert_eq!(level_for_dbz(35.0), Level::Yellow);
        assert_eq!(level_for_dbz(45.0), Level::Red);
        assert_eq!(level_for_dbz(55.0), Level::Magenta);
        // A storm 30 km dead ahead (north): a core of 55 dBZ ringed by lighter rain, from the ground to 9 km.
        let storm = vec2(0.0, -30_000.0);
        let dbz = |p: Vec3| {
            let d = vec2(p.x, p.z).distance(storm);
            if p.y > 9000.0 { -99.0 } else { 58.0 - d / 250.0 }
        };
        let ground = |_: Vec2| 0.0;
        let ctl = RadarCtl { tilt: Some(0.0), ..Default::default() };
        let img = scan(vec3(0.0, 3000.0, 0.0), 0.0, &ctl, 80_000.0, &dbz, &ground);
        let mid = AZ_BINS / 2;
        let r_core = (30_000.0 / (80_000.0 / RANGE_BINS as f32)) as usize;
        assert!(img.at(mid, r_core) >= Level::Red, "core {:?}", img.at(mid, r_core));
        assert_eq!(img.at(mid, 5), Level::None);
        // Off to the side: nothing.
        assert_eq!(img.at(0, r_core), Level::None);
        // Colours ordered with distance from the core (green ring outside red).
        let ring = (0..RANGE_BINS).map(|r| img.at(mid, r)).filter(|l| *l == Level::Green).count();
        assert!(ring > 0);
        // Behind the core the rain attenuates the beam: the far side reads weaker than the near side.
        let near = img.at(mid, r_core - 12);
        let far = img.at(mid, r_core + 12);
        assert!(far <= near, "near {near:?} far {far:?}");
        // Gain up shows more; radar off shows nothing.
        let hot = scan(vec3(0.0, 3000.0, 0.0), 0.0, &RadarCtl { gain: 8.0, ..ctl }, 80_000.0, &dbz, &ground);
        assert!(hot.histogram()[1..5].iter().sum::<usize>() > img.histogram()[1..5].iter().sum::<usize>());
        let off = scan(vec3(0.0, 3000.0, 0.0), 0.0, &RadarCtl { on: false, ..ctl }, 80_000.0, &dbz, &ground);
        assert_eq!(off.histogram()[0], AZ_BINS * RANGE_BINS);
        // Beam tilted at the ground: ground returns.
        let down = scan(vec3(0.0, 3000.0, 0.0), 0.0, &RadarCtl { tilt: Some(-8.0), ..ctl }, 80_000.0, &|_| -99.0, &ground);
        assert!(down.histogram()[Level::Ground as usize] > 100);
        // Auto tilt: a little down at altitude, up near the ground.
        assert!(RadarCtl::default().tilt_deg(10_000.0) < 0.0 && RadarCtl::default().tilt_deg(100.0) > 0.0);
    }
}
