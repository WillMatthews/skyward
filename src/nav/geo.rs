//! Navigation geometry: great-circle bearings / distances, localizer,
//! glideslope and VOR deviations, DME slant range, radio horizon and
//! terrain line of sight.
//!
//! Conventions (as everywhere in Skyward): world metres, +X east, −Z north,
//! ground positions as `Vec2(x, z)`; headings / bearings in radians, 0 =
//! north, π/2 = east, **true** (the sim has no magnetic variation model).
//! Deviations are signed so that **+ means the course / path lies to the
//! right / above** — the way the needle points ("fly right", "fly up").

use glam::{Vec2, Vec3, vec2};
use std::f32::consts::{PI, TAU};

pub const NM: f32 = 1852.0;
pub const FT: f32 = 0.3048;
const EARTH_R: f64 = 6_371_000.0;

/// (latitude, longitude) degrees.
pub type LL = (f64, f64);

pub fn wrap_pi(a: f32) -> f32 {
    let mut d = a.rem_euclid(TAU);
    if d > PI {
        d -= TAU;
    }
    d
}

pub fn wrap_2pi(a: f32) -> f32 {
    a.rem_euclid(TAU)
}

/// Heading (0 = north, π/2 = east) of a world XZ direction.
pub fn heading_of(d: Vec2) -> f32 {
    wrap_2pi(d.x.atan2(-d.y))
}

/// World XZ unit direction of a heading.
pub fn dir_of(h: f32) -> Vec2 {
    vec2(h.sin(), -h.cos())
}

/// Unit vector 90° to the right of `d` (looking along it).
pub fn right_of(d: Vec2) -> Vec2 {
    vec2(-d.y, d.x)
}

/// Initial great-circle bearing from `a` to `b` (true, radians 0..2π).
pub fn gc_bearing(a: LL, b: LL) -> f32 {
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let dlo = (b.1 - a.1).to_radians();
    let y = dlo.sin() * la2.cos();
    let x = la1.cos() * la2.sin() - la1.sin() * la2.cos() * dlo.cos();
    wrap_2pi(y.atan2(x) as f32)
}

/// Great-circle distance (m).
pub fn gc_dist(a: LL, b: LL) -> f64 {
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let (dla, dlo) = (la2 - la1, (b.1 - a.1).to_radians());
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    2.0 * EARTH_R * h.sqrt().min(1.0).asin()
}

/// The point `dist` metres from `a` on initial bearing `brg` (radians true).
pub fn gc_dest(a: LL, brg: f32, dist: f64) -> LL {
    let (la1, lo1) = (a.0.to_radians(), a.1.to_radians());
    let d = dist / EARTH_R;
    let b = brg as f64;
    let la2 = (la1.sin() * d.cos() + la1.cos() * d.sin() * b.cos()).asin();
    let lo2 = lo1 + (b.sin() * d.sin() * la1.cos()).atan2(d.cos() - la1.sin() * la2.sin());
    (la2.to_degrees(), (lo2.to_degrees() + 540.0).rem_euclid(360.0) - 180.0)
}

/// Localizer geometry: the aircraft at `p` relative to a localizer antenna at
/// `ant` whose front course points along `dir` (the landing direction, unit).
/// Returns (angular deviation, rad, + = the course is to the right of the
/// aircraft; distance from the antenna, m; in the front course sector).
pub fn loc_deviation(ant: Vec2, dir: Vec2, p: Vec2) -> (f32, f32, bool) {
    let v = p - ant;
    let behind = -v.dot(dir); // > 0 on the approach side
    let side = v.dot(right_of(dir)); // > 0: aircraft right of the course
    let ang = side.atan2(behind); // aircraft's bearing off the course, seen from the antenna
    (-ang, v.length(), behind > 0.0)
}

/// Glideslope elevation angle (rad) of `p` above the antenna site `ant`
/// (ground position with its ground height in `y`).
pub fn elevation_angle(ant: Vec3, p: Vec3) -> f32 {
    let h = vec2(p.x - ant.x, p.z - ant.z).length().max(1.0);
    (p.y - ant.y).atan2(h)
}

/// VOR course deviation for selected course `obs` with the aircraft on
/// `radial` (both radians true): (deviation rad, + = fly right; TO flag).
pub fn vor_deviation(obs: f32, radial: f32) -> (f32, bool) {
    let d = wrap_pi(radial - obs);
    if d.abs() <= PI / 2.0 {
        // FROM: on the selected radial's side of the station.
        (wrap_pi(obs - radial), false)
    } else {
        (wrap_pi(radial + PI - obs), true)
    }
}

/// Smooth-earth radio horizon (m) for antenna heights `h1`, `h2` (m above
/// the terrain between them), with the standard 4/3-earth refraction.
pub fn radio_horizon(h1: f32, h2: f32) -> f32 {
    // 1.23 √h(ft) NM per antenna.
    let k = 1.23 * NM / FT.sqrt();
    k * (h1.max(0.0).sqrt() + h2.max(0.0).sqrt())
}

/// Line of sight between `a` and `b` (world, y up) over `ground` (sampled at
/// `n` points), allowing for the earth's curvature (4/3-earth radius).
/// `ground` returns `None` where the terrain is unknown (treated as clear).
pub fn line_of_sight(ground: &dyn Fn(Vec2) -> Option<f32>, a: Vec3, b: Vec3, n: usize) -> bool {
    let (pa, pb) = (vec2(a.x, a.z), vec2(b.x, b.z));
    let d = pa.distance(pb);
    if d < 200.0 {
        return true;
    }
    let re = 4.0 / 3.0 * EARTH_R as f32;
    for i in 1..n {
        let t = i as f32 / n as f32;
        let q = pa.lerp(pb, t);
        let Some(g) = ground(q) else { continue };
        // Earth bulge at this point of the path.
        let bulge = t * (1.0 - t) * d * d / (2.0 * re);
        let line = a.y + (b.y - a.y) * t;
        if g + bulge > line - 2.0 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn great_circle() {
        // EGLL 27R → EGKK (≈ 41 km SSE-ish).
        let (a, b) = ((51.4775, -0.4614), (51.1481, -0.1903));
        let d = gc_dist(a, b);
        assert!((d - 40_600.0).abs() < 600.0, "{d}");
        let brg = gc_bearing(a, b).to_degrees();
        assert!((brg - 151.0).abs() < 2.0, "{brg}");
        let c = gc_dest(a, gc_bearing(a, b), d);
        assert!((c.0 - b.0).abs() < 1e-5 && (c.1 - b.1).abs() < 1e-5, "{c:?}");
        // Due east along the equator: 1° = 111.2 km.
        assert!((gc_dist((0.0, 0.0), (0.0, 1.0)) - 111_195.0).abs() < 10.0);
        assert!((gc_bearing((0.0, 0.0), (0.0, 1.0)).to_degrees() - 90.0).abs() < 1e-3);
    }

    #[test]
    fn localizer_sign_and_sectors() {
        // Runway pointing north (landing northbound), antenna at the far (north) end.
        let ant = vec2(0.0, -3000.0);
        let dir = vec2(0.0, -1.0);
        // 10 km out on the centreline: no deviation, front course.
        let (d, r, front) = loc_deviation(ant, dir, vec2(0.0, 7000.0));
        assert!(d.abs() < 1e-6 && front && (r - 10_000.0).abs() < 1e-3);
        // Aircraft east (right) of the course: the course is to its left → negative.
        let (d, _, _) = loc_deviation(ant, dir, vec2(100.0, 7000.0));
        assert!(d < 0.0 && (d.to_degrees() + 0.573).abs() < 0.01, "{}", d.to_degrees());
        let (d, _, _) = loc_deviation(ant, dir, vec2(-100.0, 7000.0));
        assert!(d > 0.0);
        // Beyond the antenna: back course.
        let (_, _, front) = loc_deviation(ant, dir, vec2(0.0, -5000.0));
        assert!(!front);
    }

    #[test]
    fn vor_to_from() {
        let r = |d: f32| d.to_radians();
        // OBS 090 on radial 090: FROM, centred.
        let (d, to) = vor_deviation(r(90.0), r(90.0));
        assert!(!to && d.abs() < 1e-6);
        // FROM 090 on radial 080 (north of the line, flying east): course is right.
        let (d, to) = vor_deviation(r(90.0), r(80.0));
        assert!(!to && (d.to_degrees() - 10.0).abs() < 1e-4);
        // TO 090 on radial 280 (west of station, north of the inbound line): fly right.
        let (d, to) = vor_deviation(r(90.0), r(280.0));
        assert!(to && (d.to_degrees() - 10.0).abs() < 1e-4, "{}", d.to_degrees());
        let (d, to) = vor_deviation(r(90.0), r(260.0));
        assert!(to && (d.to_degrees() + 10.0).abs() < 1e-4);
        // Across north.
        let (d, to) = vor_deviation(r(5.0), r(355.0));
        assert!(!to && (d.to_degrees() - 10.0).abs() < 1e-4);
    }

    #[test]
    fn horizon_and_los() {
        // 10,000 ft and a ground station: ≈ 123 NM.
        let h = radio_horizon(10_000.0 * FT, 0.0) / NM;
        assert!((h - 123.0).abs() < 1.0, "{h}");
        let flat = |_: Vec2| Some(0.0f32);
        assert!(line_of_sight(&flat, Vec3::new(0.0, 10.0, 0.0), Vec3::new(40_000.0, 600.0, 0.0), 32));
        // A 500 m ridge halfway blocks a low aircraft, not a high one.
        let ridge = |p: Vec2| Some(if (p.x - 20_000.0).abs() < 1000.0 { 500.0 } else { 0.0 });
        assert!(!line_of_sight(&ridge, Vec3::new(0.0, 10.0, 0.0), Vec3::new(40_000.0, 600.0, 0.0), 64));
        assert!(line_of_sight(&ridge, Vec3::new(0.0, 10.0, 0.0), Vec3::new(40_000.0, 1500.0, 0.0), 64));
        // Curvature alone: 200 km at 100 m over flat ground is below the horizon.
        assert!(!line_of_sight(&flat, Vec3::new(0.0, 10.0, 0.0), Vec3::new(200_000.0, 100.0, 0.0), 64));
        let unknown = |_: Vec2| None;
        assert!(line_of_sight(&unknown, Vec3::new(0.0, 10.0, 0.0), Vec3::new(200_000.0, 100.0, 0.0), 64));
    }
}
