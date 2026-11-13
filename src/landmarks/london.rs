//! London landmarks, modelled from published dimensions. Positions and
//! headings from OpenStreetMap (footprint centroids / principal axes, 3D
//! building parts where mapped); heights and proportions from the
//! structures' published figures (see each builder).

use super::geom::*;
use super::{Landmark, Suppress};
use crate::scene::{mat, MeshData};
use glam::{vec2, vec3, Mat4, Vec2, Vec3};
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::OnceLock;

pub static LANDMARKS: &[Landmark] = &[
    Landmark { name: "The Shard", lat: 51.504408, lon: -0.086438, heading: 41.0, suppress: Suppress::Rect(-36.0, 36.0, -36.0, 36.0), build: shard, deck: 0.0, animate: None },
    Landmark { name: "Tower Bridge", lat: 51.5055314, lon: -0.0753864, heading: 22.7, suppress: Suppress::Rect(-19.0, 19.0, -168.0, 168.0), build: tower_bridge, deck: 9.0, animate: None },
    Landmark { name: "London Eye", lat: 51.503342, lon: -0.119677, heading: 8.0, suppress: Suppress::Rect(-10.0, 50.0, -70.0, 70.0), build: london_eye, deck: 0.0, animate: Some(london_eye_wheel) },
    Landmark { name: "St Paul's Cathedral", lat: 51.513774, lon: -0.098479, heading: -6.3, suppress: Suppress::Rect(-84.0, 86.0, -46.0, 46.0), build: st_pauls, deck: 0.0, animate: None },
    Landmark { name: "Elizabeth Tower (Big Ben)", lat: 51.5007121, lon: -0.1245519, heading: 6.0, suppress: Suppress::Rect(-11.0, 11.0, -11.0, 11.0), build: big_ben, deck: 0.0, animate: None },
    Landmark { name: "Palace of Westminster", lat: 51.499317, lon: -0.124576, heading: 10.3, suppress: Suppress::Rect(-58.0, 56.0, -136.0, 134.0), build: westminster, deck: 0.0, animate: None },
    Landmark { name: "BT Tower", lat: 51.5214383, lon: -0.1388047, heading: 0.0, suppress: Suppress::Circle(12.0), build: bt_tower, deck: 0.0, animate: None },
    Landmark { name: "30 St Mary Axe (Gherkin)", lat: 51.514475, lon: -0.080333, heading: 0.0, suppress: Suppress::Circle(32.0), build: gherkin, deck: 0.0, animate: None },
    Landmark { name: "20 Fenchurch Street (Walkie-Talkie)", lat: 51.511253, lon: -0.083544, heading: 13.0, suppress: Suppress::Rect(-34.0, 34.0, -40.0, 42.0), build: walkie_talkie, deck: 0.0, animate: None },
    Landmark { name: "One Canada Square", lat: 51.504935, lon: -0.019524, heading: 9.0, suppress: Suppress::Rect(-28.0, 28.0, -28.0, 28.0), build: canada_square, deck: 0.0, animate: None },
    Landmark { name: "The O2", lat: 51.502935, lon: 0.003205, heading: 0.0, suppress: Suppress::Circle(176.0), build: the_o2, deck: 0.0, animate: None },
    Landmark { name: "Wembley Stadium", lat: 51.556038, lon: -0.279600, heading: 7.0, suppress: Suppress::Rect(-152.0, 152.0, -146.0, 146.0), build: wembley, deck: 0.0, animate: None },
    Landmark { name: "Battersea Power Station", lat: 51.481787, lon: -0.144541, heading: 12.6, suppress: Suppress::Rect(-84.0, 82.0, -86.0, 86.0), build: battersea, deck: 0.0, animate: None },
];

// ---------------------------------------------------------------- palette

const PORTLAND: [f32; 3] = rgb(0.86, 0.84, 0.78);
const ANSTON: [f32; 3] = rgb(0.80, 0.72, 0.56);
const SLATE: [f32; 3] = rgb(0.30, 0.33, 0.37);
const LEAD: [f32; 3] = rgb(0.50, 0.52, 0.54);
const GOLD: [f32; 3] = rgb(0.88, 0.72, 0.34);
const DARK: [f32; 3] = rgb(0.07, 0.07, 0.08);
const WHITE: [f32; 3] = rgb(0.92, 0.92, 0.90);
const WARM_WIN: [f32; 3] = rgb(1.0, 0.80, 0.52);

const STONE_LUM: f32 = 6.0;
fn lit() -> Surf {
    Surf::Flood(STONE_LUM)
}
const DIFF: Surf = Surf::Plain(mat::DIFFUSE);
const METAL: Surf = Surf::Plain(mat::METAL);
const ROOF: Surf = Surf::Plain(mat::ROOF);

/// Vertical ribs (buttresses / panelling) on the four faces of an
/// axis-aligned block: `n` per face, evenly spaced inside the corners.
#[allow(clippy::too_many_arguments)]
fn ribs(k: &mut Kit, c: Vec2, hx: f32, hz: f32, y0: f32, y1: f32, nx: usize, nz: usize, depth: f32, w: f32, s: Surf, col: [f32; 3]) {
    let ym = (y0 + y1) * 0.5;
    let hy = (y1 - y0) * 0.5;
    for i in 0..nx {
        let x = c.x - hx + 2.0 * hx * (i as f32 + 0.5) / nx as f32;
        for sz in [-1.0, 1.0] {
            k.boxr(vec3(x, ym, c.y + sz * (hz + depth * 0.5)), vec3(w * 0.5, hy, depth * 0.5), 0.0, s, col);
        }
    }
    for i in 0..nz {
        let z = c.y - hz + 2.0 * hz * (i as f32 + 0.5) / nz as f32;
        for sx in [-1.0, 1.0] {
            k.boxr(vec3(c.x + sx * (hx + depth * 0.5), ym, z), vec3(depth * 0.5, hy, w * 0.5), 0.0, s, col);
        }
    }
}

/// Windows on the faces of an axis-aligned block: rows at `levels`
/// (centre heights), `nx` / `nz` per face.
#[allow(clippy::too_many_arguments)]
fn windows(k: &mut Kit, c: Vec2, hx: f32, hz: f32, levels: &[f32], w: f32, h: f32, nx: usize, nz: usize, s: Surf, col: [f32; 3]) {
    let o = 0.08;
    for &y in levels {
        for i in 0..nx {
            let x = c.x - hx + 2.0 * hx * (i as f32 + 0.5) / nx as f32;
            for sz in [-1.0f32, 1.0] {
                k.panel(vec3(x, y, c.y + sz * (hz + o)), vec3(0.0, 0.0, sz), w, h, s, col);
            }
        }
        for i in 0..nz {
            let z = c.y - hz + 2.0 * hz * (i as f32 + 0.5) / nz as f32;
            for sx in [-1.0f32, 1.0] {
                k.panel(vec3(c.x + sx * (hx + o), y, z), vec3(sx, 0.0, 0.0), w, h, s, col);
            }
        }
    }
}

/// Pinnacles around the parapet of an axis-aligned block, about every `step` m.
fn parapet_pinnacles(k: &mut Kit, c: Vec2, hx: f32, hz: f32, y: f32, step: f32, h: f32, s: Surf, col: [f32; 3]) {
    let corners = [c + vec2(-hx, -hz), c + vec2(hx, -hz), c + vec2(hx, hz), c + vec2(-hx, hz)];
    for i in 0..4 {
        let (a, b) = (corners[i], corners[(i + 1) % 4]);
        let n = ((a.distance(b) / step).round() as usize).max(1);
        for j in 0..n {
            k.pinnacle(a.lerp(b, j as f32 / n as f32), y, 0.45, h, s, col);
        }
    }
}

// ---------------------------------------------------------------- The Shard

/// The Shard (Renzo Piano, 2012): 309.6 m, eight sloping glass facets on a
/// ~56 m base that taper toward a virtual apex and stop short of each other
/// at the corners ("fractures") and at the top, where the facets stand free
/// as an open spire from ~250 m (floodlit at night).
fn shard(k: &mut Kit) {
    const H0: f32 = 324.0;
    let taper = |y: f32| (1.0 - y / H0).max(0.0);
    let glass = rgb(0.70, 0.78, 0.84);
    let core = rgb(0.20, 0.24, 0.28);
    let sq = |h: f32| rect(Vec2::ZERO, h, h);
    k.loft(&sq(25.0), 0.0, &sq(25.0 * taper(296.0)), 296.0, DIFF, core);
    let tops = [309.6, 298.0, 304.0, 291.0, 306.5, 295.0, 301.0, 288.0];
    for side in 0..4 {
        let a = side as f32 * FRAC_PI_2;
        let nrm = vec2(a.cos(), a.sin());
        let tan = vec2(-a.sin(), a.cos());
        for f in 0..2 {
            let front = (side + f) % 2 == 0;
            let off = if front { 0.0 } else { -1.7 };
            let (t0, t1) = if f == 0 { (-1.0, 0.14) } else { (-0.14, 1.0) };
            let top = tops[side * 2 + f];
            let pt = |t: f32, y: f32| {
                let tp = taper(y);
                let q = nrm * ((28.0 + off) * tp + 0.2) + tan * (t * 23.5 * tp + 0.6 * t);
                vec3(q.x, y, q.y)
            };
            let ys = [0.0, 50.0, 100.0, 150.0, 200.0, 250.0];
            let rows: Vec<Vec<Vec3>> = ys.iter().map(|&y| vec![pt(t0, y), pt(t1, y)]).collect();
            k.grid(&rows, false, Shade::Flat, Surf::Facade(mat::GLASS, 40 + 2 * (side * 2 + f) as u32), &|_, _| glass);
            // Free-standing shard tips, slanted tops.
            let slant = if f == 0 { (top - 7.0, top) } else { (top, top - 7.0) };
            let rows = vec![vec![pt(t0, 250.0), pt(t1, 250.0)], vec![pt(t0, slant.0), pt(t1, slant.1)]];
            k.grid(&rows, false, Shade::Flat, Surf::Lit(48.0), &|_, _| rgb(0.64, 0.72, 0.80));
        }
    }
    // Lights: red obstacle lights at the tip and on the corners at two levels.
    k.obstacle(vec3(0.0, 310.5, 0.0));
    for y in [150.0f32, 245.0] {
        let d = 28.0 * taper(y) + 0.5;
        for c in [vec2(d, d), vec2(-d, d), vec2(d, -d), vec2(-d, -d)] {
            k.obstacle(vec3(c.x, y, c.y));
        }
    }
    k.solid(sq(28.0), 100.0);
    k.solid(sq(19.5), 200.0);
    k.solid(sq(11.0), 310.0);
}

// ---------------------------------------------------------------- Tower Bridge

/// Tower Bridge (1894): two 65 m Gothic towers on river piers, 82.6 m apart
/// (OSM 3D parts), high-level walkways at 40–45 m, closed bascules, side
/// suspension spans with curved chains down to the shore towers. Local z
/// runs along the bridge (-z = north bank).
fn tower_bridge(k: &mut Kit) {
    let stone = rgb(0.80, 0.77, 0.70);
    let granite = rgb(0.60, 0.59, 0.56);
    let blue = rgb(0.50, 0.68, 0.86);
    let white = rgb(0.90, 0.91, 0.92);
    let road = rgb(0.26, 0.26, 0.27);
    let steel = Surf::Lit(7.0);
    let (hx, hz) = (10.0, 8.0);
    for &tz in &[-41.4f32, 41.4] {
        let c = vec2(0.0, tz);
        // Pier with cutwaters pointing up- and downstream (±x).
        let pier = vec![vec2(-18.0, tz), vec2(-13.0, tz - 10.0), vec2(13.0, tz - 10.0), vec2(18.0, tz), vec2(13.0, tz + 10.0), vec2(-13.0, tz + 10.0)];
        k.prism(&pier, -3.0, 7.0, Surf::Lit(4.0), granite, None);
        // Tower body, deck to eaves.
        k.prism(&rect(c, hx, hz), 7.0, 47.0, lit(), stone, None);
        ribs(k, c, hx, hz, 9.0, 46.0, 2, 2, 0.5, 1.0, lit(), stone);
        // String courses.
        for y in [9.0f32, 24.0, 38.5] {
            k.boxr(vec3(0.0, y, tz), vec3(hx + 0.35, 0.4, hz + 0.35), 0.0, lit(), stone);
        }
        // Roadway arch through the tower (the ±z faces), Gothic windows.
        for sz in [-1.0f32, 1.0] {
            let z = tz + sz * (hz + 0.06);
            k.panel(vec3(0.0, 13.0, z), vec3(0.0, 0.0, sz), 8.0, 8.0, DIFF, DARK);
            k.tri(vec3(-4.0, 17.0, z), vec3(4.0, 17.0, z), vec3(0.0, 20.0, z), DIFF, DARK);
        }
        windows(k, c, hx, hz, &[21.0, 30.0], 1.3, 4.0, 3, 2, Surf::Glow(5.0), WARM_WIN);
        windows(k, c, hx, hz, &[42.0], 1.0, 3.0, 2, 1, Surf::Glow(5.0), WARM_WIN);
        // Corner turrets with spires and gilded finials.
        for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let p = vec2(sx * (hx + 0.3), tz + sz * (hz + 0.3));
            let ring = ngon(p, 1.8, 8, PI / 8.0);
            k.walls(&ring, 7.0, 52.0, lit(), stone);
            k.pyramid(&ring, 52.0, vec3(p.x, 59.0, p.y), ROOF, SLATE);
            k.tube(vec3(p.x, 58.5, p.y), vec3(p.x, 61.0, p.y), 0.25, 0.05, 4, METAL, GOLD);
        }
        // Steep hipped roof, central spire and finial.
        k.roof(c, hx, hz, FRAC_PI_2, 47.0, 10.5, 3.5, ROOF, SLATE, None);
        let sp = rect(c, 1.6, 1.6);
        k.pyramid(&sp, 57.0, vec3(0.0, 65.0, tz), ROOF, SLATE);
        k.tube(vec3(0.0, 64.5, tz), vec3(0.0, 67.0, tz), 0.3, 0.05, 4, METAL, GOLD);
        // Dormer gables facing the river.
        for sx in [-1.0f32, 1.0] {
            k.roof(vec2(sx * (hx - 2.0), tz), 2.6, 2.2, 0.0, 47.0, 4.5, 0.0, ROOF, SLATE, Some((lit(), stone)));
            k.pinnacle(vec2(sx * (hx + 0.6), tz), 47.0, 0.4, 5.5, lit(), stone);
        }
    }
    // High-level walkways (two, east and west) between the towers.
    let (z0, z1) = (-41.4 + hz, 41.4 - hz);
    for sx in [-1.0f32, 1.0] {
        let x = sx * 7.0;
        k.boxr(vec3(x, 42.5, 0.0), vec3(2.2, 2.5, z1), 0.0, steel, blue);
        // Lattice girder on the outer face + chords.
        let xo = x + sx * 2.3;
        let bays = 14;
        for b in 0..bays {
            let za = z0 + (z1 - z0) * b as f32 / bays as f32;
            let zb = z0 + (z1 - z0) * (b + 1) as f32 / bays as f32;
            k.beam(vec3(xo, 40.2, za), vec3(xo, 44.8, zb), 0.3, 0.3, steel, white);
            k.beam(vec3(xo, 44.8, za), vec3(xo, 40.2, zb), 0.3, 0.3, steel, white);
        }
        for y in [40.1f32, 44.9] {
            k.beam(vec3(xo, y, z0), vec3(xo, y, z1), 0.4, 0.4, steel, white);
        }
        // Curved cantilever brackets under the walkway ends.
        for (zt, d) in [(z0, 1.0f32), (z1, -1.0)] {
            let pts: Vec<Vec3> = (0..=6).map(|i| {
                let t = i as f32 / 6.0;
                vec3(x, 40.0 - 10.0 * (1.0 - (t * FRAC_PI_2).sin()), zt + d * 9.0 * t)
            }).collect();
            for w in pts.windows(2) {
                k.beam(w[0], w[1], 1.0, 0.8, steel, blue);
            }
        }
    }
    // Decks: bascules (closed), side spans, approaches.
    let deck = |k: &mut Kit, za: f32, zb: f32| {
        let (za, zb) = (za.min(zb), za.max(zb));
        k.boxr(vec3(0.0, 8.0, (za + zb) * 0.5), vec3(9.5, 1.0, (zb - za) * 0.5), 0.0, Surf::Plain(mat::ASPHALT), road);
        for sx in [-1.0f32, 1.0] {
            k.boxr(vec3(sx * 9.3, 9.8, (za + zb) * 0.5), vec3(0.25, 0.8, (zb - za) * 0.5), 0.0, steel, blue);
        }
    };
    deck(k, z0, z1);
    for s in [-1.0f32, 1.0] {
        deck(k, s * (41.4 + 10.0), s * 129.0);
        deck(k, s * 137.0, s * 166.0);
        // Shore (abutment) towers.
        let c = vec2(0.0, s * 133.0);
        k.prism(&rect(c, 10.0, 4.0), -2.0, 18.0, lit(), stone, None);
        k.roof(c, 10.0, 4.0, 0.0, 18.0, 4.0, 2.0, ROOF, SLATE, None);
        for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let p = c + vec2(sx * 10.2, sz * 4.2);
            let ring = ngon(p, 1.2, 8, PI / 8.0);
            k.walls(&ring, -2.0, 21.0, lit(), stone);
            k.pyramid(&ring, 21.0, vec3(p.x, 25.5, p.y), ROOF, SLATE);
        }
        // Suspension chains (both sides) dipping to the deck, with hangers.
        for sx in [-1.0f32, 1.0] {
            let x = sx * 9.7;
            let n = 18;
            let pts: Vec<Vec3> = (0..=n).map(|i| {
                let t = i as f32 / n as f32;
                let y = 37.0 - 17.0 * t - 4.0 * 16.0 * t * (1.0 - t);
                vec3(x, y.max(11.0), s * (41.4 + hz + (129.0 - 41.4 - hz) * t))
            }).collect();
            for w in pts.windows(2) {
                k.beam(w[0], w[1], 0.8, 2.4, steel, blue);
            }
            for p in &pts[1..n] {
                if p.y > 12.5 {
                    k.beam(vec3(p.x, 10.4, p.z), vec3(p.x, p.y - 1.0, p.z), 0.25, 0.25, steel, white);
                }
            }
        }
    }
    for &tz in &[-41.4f32, 41.4] {
        k.solid(rect(vec2(0.0, tz), hx + 1.0, hz + 1.0), 65.0);
    }
    k.solid(rect_xz(-10.0, 10.0, -166.0, 166.0), 10.5);
    k.solid(rect(vec2(0.0, -133.0), 11.0, 5.0), 25.0);
    k.solid(rect(vec2(0.0, 133.0), 11.0, 5.0), 25.0);
}

// ---------------------------------------------------------------- London Eye

/// Hub height, rim and capsule radius (135 m overall, 120 m wheel; OSM
/// footprint gives the wheel plane at 8°, A-frame legs on the land side).
const EYE_HUB: f32 = 69.0;
const EYE_RIM: f32 = 60.5;
const EYE_CAPSULE_R: f32 = 63.8;
/// One revolution per 30 minutes.
const EYE_PERIOD: f32 = 1800.0;

fn london_eye(k: &mut Kit) {
    let white = rgb(0.90, 0.91, 0.92);
    let grey = rgb(0.55, 0.57, 0.60);
    let apex = vec3(15.0, EYE_HUB, 0.0);
    // A-frame legs (land side, +x), splayed along the wheel plane.
    for sz in [-1.0f32, 1.0] {
        k.tube(vec3(46.0, -1.0, sz * 24.0), apex + vec3(0.0, 0.0, sz * 1.5), 1.5, 1.1, 12, Surf::Lit(5.0), white);
    }
    // Cross tie between the legs.
    k.tube(vec3(38.0, 18.0, -18.5), vec3(38.0, 18.0, 18.5), 0.5, 0.5, 6, Surf::Lit(5.0), white);
    // Backstay cables.
    for z in [-14.0f32, -8.0, -3.0, 3.0, 8.0, 14.0] {
        k.tube(apex, vec3(96.0, 0.0, z), 0.14, 0.14, 4, DIFF, grey);
    }
    // Spindle and hub (rotationally symmetric, so static).
    k.tube(vec3(-8.0, EYE_HUB, 0.0), apex + vec3(1.0, 0.0, 0.0), 1.3, 1.3, 12, METAL, rgb(0.75, 0.76, 0.78));
    let hub = |x: f32| vec3(x, EYE_HUB, 0.0);
    k.tube(hub(-5.5), hub(5.5), 3.4, 3.4, 16, Surf::Lit(5.0), white);
    k.disc(hub(-5.5), Vec3::X, 3.4, 16, Surf::Lit(5.0), white);
    k.disc(hub(5.5), Vec3::X, 3.4, 16, Surf::Lit(5.0), white);
    // Boarding platform over the river edge.
    k.boxr(vec3(-2.0, 1.0, 0.0), vec3(6.0, 1.0, 26.0), 0.0, DIFF, rgb(0.45, 0.46, 0.48));
    k.obstacle(hub(-6.0) + vec3(0.0, 3.6, 0.0));
    k.obstacle(apex + vec3(0.0, 2.0, 0.0));
    k.solid(rect_xz(-3.0, 3.0, -EYE_CAPSULE_R - 2.5, EYE_CAPSULE_R + 2.5), EYE_HUB + EYE_CAPSULE_R + 2.5);
    k.solid(rect_xz(-3.0, 18.0, -6.0, 6.0), EYE_HUB + 4.0);
}

/// The turning parts, built once in the wheel frame (axis = x, hub at the origin).
struct EyeParts {
    wheel: MeshData,
    capsule: MeshData,
}

fn eye_parts() -> &'static EyeParts {
    static P: OnceLock<EyeParts> = OnceLock::new();
    P.get_or_init(|| {
        let rim_col = rgb(0.80, 0.87, 1.0);
        let rim = Surf::Lit(55.0);
        let mut k = Kit::default();
        let n = 96;
        let ring = |r: f32, x: f32| -> Vec<Vec3> {
            (0..n).map(|i| {
                let a = TAU * i as f32 / n as f32;
                vec3(x, r * a.cos(), r * a.sin())
            }).collect()
        };
        let (ro, ri, xi) = (EYE_RIM, EYE_RIM - 2.6, 2.1);
        k.path(&ring(ro, 0.0), &|_| 0.9, 6, true, rim, rim_col);
        k.path(&ring(ri, -xi), &|_| 0.6, 5, true, rim, rim_col);
        k.path(&ring(ri, xi), &|_| 0.6, 5, true, rim, rim_col);
        // Truss web: 64 bays.
        let bays = 64;
        let at = |r: f32, x: f32, i: usize| {
            let a = TAU * i as f32 / bays as f32;
            vec3(x, r * a.cos(), r * a.sin())
        };
        for i in 0..bays {
            let (o, o1) = (at(ro, 0.0, i), at(ro, 0.0, i + 1));
            let (l, r) = (at(ri, -xi, i), at(ri, xi, i));
            k.beam(l, o, 0.25, 0.25, rim, rim_col);
            k.beam(r, o, 0.25, 0.25, rim, rim_col);
            k.beam(l, r, 0.25, 0.25, rim, rim_col);
            k.beam(if i % 2 == 0 { l } else { r }, o1, 0.2, 0.2, rim, rim_col);
        }
        // Cable spokes from the hub flanges to the inner rims (alternating sides).
        for i in 0..64 {
            let a = TAU * (i as f32 + 0.5) / 64.0;
            let side = if i % 2 == 0 { 1.0 } else { -1.0 };
            let h = vec3(side * 5.2, 3.3 * a.cos(), 3.3 * a.sin());
            let r = vec3(side * xi, ri * a.cos(), ri * a.sin());
            k.tube(h, r, 0.12, 0.12, 3, Surf::Lit(18.0), rgb(0.85, 0.88, 0.92));
        }
        let wheel = std::mem::take(&mut k.m);
        // Capsule: 8 m glass ovoid along the axis, white end caps, floor.
        let mut c = Kit::default();
        c.ellipsoid(Vec3::ZERO, vec3(4.0, 2.25, 2.25), 14, 8, Surf::Lit(9.0), rgb(0.72, 0.80, 0.86));
        c.disc(vec3(4.05, 0.0, 0.0), Vec3::X, 1.0, 8, Surf::Lit(9.0), WHITE);
        c.disc(vec3(-4.05, 0.0, 0.0), Vec3::X, 1.0, 8, Surf::Lit(9.0), WHITE);
        // Mounting ring around the capsule's middle.
        let mring: Vec<Vec3> = (0..16).map(|i| {
            let a = TAU * i as f32 / 16.0;
            vec3(0.0, 2.35 * a.cos(), 2.35 * a.sin())
        }).collect();
        c.path(&mring, &|_| 0.18, 4, true, Surf::Lit(9.0), WHITE);
        EyeParts { wheel, capsule: c.m }
    })
}

/// Wheel, spokes and the 32 capsules (kept level) at time `t`.
fn london_eye_wheel(out: &mut MeshData, xf: Mat4, t: f32) {
    let p = eye_parts();
    let ang = -(t / EYE_PERIOD).fract() * TAU;
    let hub = Mat4::from_translation(vec3(0.0, EYE_HUB, 0.0));
    out.append(&p.wheel, xf * hub * Mat4::from_rotation_x(ang));
    for i in 0..32 {
        let a = ang + TAU * (i as f32 + 0.25) / 32.0;
        let pos = vec3(0.0, EYE_HUB + EYE_CAPSULE_R * a.cos(), EYE_CAPSULE_R * a.sin());
        out.append(&p.capsule, xf * Mat4::from_translation(pos));
    }
}

// ---------------------------------------------------------------- St Paul's

/// St Paul's Cathedral (Wren, 1710): 158 m long, 111 m to the cross, dome
/// ~34 m across on a colonnaded drum, two 67 m west towers. Local +x = east
/// (the axis); crossing at x = 12 (from the OSM footprint's transepts).
fn st_pauls(k: &mut Kit) {
    let st = lit();
    let stone = PORTLAND;
    let lead = LEAD;
    let win = Surf::Glow(3.5);
    // Two-storey blocks with cornices, a lead roof behind the parapet.
    let block = |k: &mut Kit, x0: f32, x1: f32, hz: f32, ridge_x: bool| {
        let c = vec2((x0 + x1) * 0.5, 0.0);
        let hx = (x1 - x0) * 0.5;
        k.walls(&rect(c, hx, hz), 0.0, 15.0, st, stone);
        k.walls(&rect(c, hx - 0.4, hz - 0.4), 15.0, 26.5, st, stone);
        k.boxr(vec3(c.x, 15.0, 0.0), vec3(hx + 0.4, 0.5, hz + 0.4), 0.0, st, stone);
        k.boxr(vec3(c.x, 26.5, 0.0), vec3(hx + 0.2, 0.9, hz + 0.2), 0.0, st, stone);
        let (rx, rz, yaw) = if ridge_x { (hx - 1.0, hz - 1.5, 0.0) } else { (hz - 1.5, hx - 1.0, FRAC_PI_2) };
        k.roof(c, rx, rz, yaw, 27.4, 5.5, 0.0, ROOF, lead, Some((st, stone)));
        let bays_x = ((2.0 * hx) / 6.0).round().max(1.0) as usize;
        let bays_z = ((2.0 * hz) / 6.0).round().max(1.0) as usize;
        windows(k, c, hx, hz, &[7.0], 1.8, 5.0, bays_x, bays_z, win, WARM_WIN);
        windows(k, c, hx - 0.4, hz - 0.4, &[20.5], 1.6, 4.5, bays_x, bays_z, win, WARM_WIN);
        ribs(k, c, hx, hz, 0.0, 26.0, bays_x + 1, bays_z + 1, 0.35, 0.9, st, stone);
    };
    block(k, -74.0, -44.0, 27.0, true); // west end (chapels + towers' base)
    block(k, -44.0, -8.0, 19.0, true); // nave
    block(k, -8.0, 32.0, 38.0, false); // transepts + crossing
    block(k, 32.0, 76.0, 19.0, true); // choir
    // East apse and the transept porticos (half-round).
    k.lathe_arc(vec3(76.0, 0.0, 0.0), &[(12.0, 0.0), (12.0, 22.0)], 12, -FRAC_PI_2, FRAC_PI_2, Shade::Smooth, st, &|_, _| stone);
    let apse_dome: Vec<(f32, f32)> = (0..=5).map(|i| {
        let a = FRAC_PI_2 * i as f32 / 5.0;
        (12.0 * a.cos() + 0.01, 22.0 + 6.0 * a.sin())
    }).collect();
    k.lathe_arc(vec3(76.0, 0.0, 0.0), &apse_dome, 12, -FRAC_PI_2, FRAC_PI_2, Shade::Smooth, ROOF, &|_, _| lead);
    for sz in [-1.0f32, 1.0] {
        let (a0, a1) = if sz > 0.0 { (0.0, PI) } else { (PI, TAU) };
        k.lathe_arc(vec3(12.0, 0.0, sz * 38.0), &[(6.5, 0.0), (6.5, 15.0)], 10, a0, a1, Shade::Smooth, st, &|_, _| stone);
        let cap: Vec<(f32, f32)> = (0..=4).map(|i| {
            let a = FRAC_PI_2 * i as f32 / 4.0;
            (6.5 * a.cos() + 0.01, 15.0 + 4.0 * a.sin())
        }).collect();
        k.lathe_arc(vec3(12.0, 0.0, sz * 38.0), &cap, 10, a0, a1, Shade::Smooth, ROOF, &|_, _| lead);
    }
    // West portico: two orders of paired columns and a pediment.
    let (px0, px1) = (-80.0, -74.0);
    k.prism(&rect_xz(px1 - 0.5, px1, -14.0, 14.0), 0.0, 24.0, st, stone, None);
    k.boxr(vec3(px0 + 2.5, 12.6, 0.0), vec3(3.2, 0.6, 13.5), 0.0, st, stone);
    k.boxr(vec3(px0 + 2.5, 23.6, 0.0), vec3(3.2, 0.7, 13.5), 0.0, st, stone);
    for i in 0..8 {
        let z = -11.9 + 23.8 * i as f32 / 7.0;
        k.tube(vec3(px0 + 0.8, 0.0, z), vec3(px0 + 0.8, 12.0, z), 0.75, 0.68, 8, st, stone);
        if (2..6).contains(&i) || i == 0 || i == 7 {
            k.tube(vec3(px0 + 0.8, 13.2, z), vec3(px0 + 0.8, 23.0, z), 0.65, 0.58, 8, st, stone);
        }
    }
    let (pa, pb) = (vec3(px0 - 0.2, 24.3, -13.5), vec3(px0 - 0.2, 24.3, 13.5));
    let apex = vec3(px0 - 0.2, 31.0, 0.0);
    k.tri(pa, pb, apex, st, stone);
    k.quad(pa, apex, apex + vec3(6.5, 0.0, 0.0), pa + vec3(6.5, 0.0, 0.0), ROOF, lead);
    k.quad(pb, apex, apex + vec3(6.5, 0.0, 0.0), pb + vec3(6.5, 0.0, 0.0), ROOF, lead);
    // West towers: square base, colonnaded drum, ogee cap, gilded pineapple.
    for sz in [-1.0f32, 1.0] {
        let c = vec2(-68.0, sz * 21.0);
        k.prism(&rect(c, 6.0, 6.0), 26.0, 41.0, st, stone, None);
        k.boxr(vec3(c.x, 41.0, c.y), vec3(6.4, 0.6, 6.4), 0.0, st, stone);
        k.lathe(vec3(c.x, 0.0, c.y), &[(3.8, 41.0), (3.8, 53.0)], 12, Shade::Smooth, st, &|_, _| stone);
        for i in 0..12 {
            let a = TAU * i as f32 / 12.0;
            let p = vec3(c.x + 4.9 * a.cos(), 41.5, c.y + 4.9 * a.sin());
            k.tube(p, p + vec3(0.0, 10.5, 0.0), 0.38, 0.34, 6, st, stone);
        }
        let ogee = [(5.6, 52.0), (5.6, 53.6), (4.4, 54.4), (3.0, 56.5), (2.4, 58.5), (1.6, 60.5), (0.9, 62.0), (0.3, 63.0)];
        k.lathe(vec3(c.x, 0.0, c.y), &ogee, 12, Shade::Smooth, st, &|_, _| stone);
        k.ellipsoid(vec3(c.x, 64.0, c.y), vec3(0.8, 1.2, 0.8), 8, 5, METAL, GOLD);
        k.tube(vec3(c.x, 65.0, c.y), vec3(c.x, 67.0, c.y), 0.15, 0.03, 4, METAL, GOLD);
    }
    // --- the dome over the crossing
    let dc = vec3(12.0, 0.0, 0.0);
    k.lathe(dc, &[(22.0, 26.0), (22.0, 31.0), (21.0, 32.0)], 32, Shade::Smooth, st, &|_, _| stone);
    k.cap(&ngon(vec2(dc.x, dc.z), 21.0, 32, 0.0), 32.0, st, stone);
    // Drum wall behind the peristyle, with windows.
    k.lathe(dc, &[(17.6, 32.0), (17.6, 48.0)], 32, Shade::Smooth, st, &|_, _| rgb(0.80, 0.78, 0.72));
    for i in 0..16 {
        let a = TAU * (i as f32 + 0.5) / 16.0;
        let n = vec3(a.cos(), 0.0, a.sin());
        k.panel(dc + n * 17.7 + vec3(0.0, 40.0, 0.0), n, 2.2, 7.0, win, WARM_WIN);
    }
    // Peristyle: 32 columns, entablature and balustrade ring.
    for i in 0..32 {
        let a = TAU * i as f32 / 32.0;
        let p = dc + vec3(21.0 * a.cos(), 32.0, 21.0 * a.sin());
        k.tube(p, p + vec3(0.0, 15.0, 0.0), 0.85, 0.78, 8, st, stone);
    }
    k.lathe(dc, &[(17.6, 47.0), (22.0, 47.0), (22.0, 49.5), (21.5, 50.0), (18.6, 50.0)], 32, Shade::Flat, st, &|_, _| stone);
    // Attic (stone gallery) and cornice.
    k.lathe(dc, &[(18.2, 50.0), (18.2, 58.0), (18.9, 58.3), (18.9, 59.2), (18.0, 59.4)], 32, Shade::Flat, st, &|_, _| stone);
    windows_ring(k, dc, 18.25, 54.0, 24, 1.4, 3.6, win, WARM_WIN);
    // Dome: lead, slightly taller than a hemisphere, with 32 ribs.
    let dome: Vec<(f32, f32)> = (0..=10).map(|i| {
        let ph = (75.5f32).to_radians() * i as f32 / 10.0;
        (18.0 * ph.cos(), 59.4 + 26.5 * ph.sin())
    }).collect();
    k.lathe(dc, &dome, 64, Shade::Split, Surf::Lit(4.0), &|i, _| if i % 2 == 0 { lead } else { rgb(0.44, 0.46, 0.48) });
    // Golden gallery, lantern, cupola, ball and cross.
    k.lathe(dc, &[(4.5, 85.0), (5.4, 85.2), (5.4, 86.2), (3.6, 86.4)], 16, Shade::Flat, st, &|_, _| stone);
    k.lathe(dc, &[(3.6, 86.4), (3.6, 95.0), (4.0, 95.4), (4.0, 96.4), (3.0, 96.6)], 16, Shade::Smooth, st, &|_, _| stone);
    for i in 0..8 {
        let a = TAU * (i as f32 + 0.5) / 8.0;
        let p = dc + vec3(4.1 * a.cos(), 86.4, 4.1 * a.sin());
        k.tube(p, p + vec3(0.0, 8.6, 0.0), 0.3, 0.3, 6, st, stone);
    }
    k.lathe(dc, &[(3.0, 96.6), (2.6, 98.2), (1.8, 99.6), (0.8, 100.6), (0.4, 101.2)], 16, Shade::Smooth, ROOF, &|_, _| lead);
    k.ellipsoid(dc + vec3(0.0, 103.0, 0.0), vec3(1.6, 1.6, 1.6), 12, 8, METAL, GOLD);
    k.boxr(dc + vec3(0.0, 107.8, 0.0), vec3(0.22, 3.2, 0.22), 0.0, METAL, GOLD);
    k.boxr(dc + vec3(0.0, 109.0, 0.0), vec3(0.22, 0.22, 1.4), 0.0, METAL, GOLD);
    k.obstacle(dc + vec3(0.0, 111.5, 0.0));
    // Collision.
    k.solid(rect_xz(-80.0, 88.0, -19.0, 19.0), 33.0);
    k.solid(rect_xz(-74.0, -44.0, -27.0, 27.0), 33.0);
    k.solid(rect_xz(-8.0, 32.0, -44.0, 44.0), 33.0);
    k.solid(ngon(vec2(dc.x, dc.z), 22.0, 16, 0.0), 60.0);
    k.solid(ngon(vec2(dc.x, dc.z), 15.0, 12, 0.0), 86.0);
    k.solid(ngon(vec2(dc.x, dc.z), 4.0, 8, 0.0), 111.0);
    k.solid(rect(vec2(-68.0, 21.0), 6.0, 6.0), 67.0);
    k.solid(rect(vec2(-68.0, -21.0), 6.0, 6.0), 67.0);
}

/// Windows around a cylinder.
#[allow(clippy::too_many_arguments)]
fn windows_ring(k: &mut Kit, c: Vec3, r: f32, y: f32, n: usize, w: f32, h: f32, s: Surf, col: [f32; 3]) {
    for i in 0..n {
        let a = TAU * (i as f32 + 0.5) / n as f32;
        let nrm = vec3(a.cos(), 0.0, a.sin());
        k.panel(c + nrm * r + vec3(0.0, y, 0.0), nrm, w, h, s, col);
    }
}

// ---------------------------------------------------------------- Westminster

/// Elizabeth Tower ("Big Ben", 1859): 96 m, ~12 m square shaft, clock stage
/// at 51–61 m with four 7 m dials (centre 56 m, lit at night), belfry and a
/// cast-iron spire (heights from the OSM 3D parts).
fn big_ben(k: &mut Kit) {
    let stone = ANSTON;
    let st = Surf::Flood(9.0);
    let spire_col = rgb(0.26, 0.28, 0.32);
    let h = 6.0;
    k.walls(&rect(Vec2::ZERO, h, h), 0.0, 51.0, st, stone);
    ribs(k, Vec2::ZERO, h, h, 2.0, 50.0, 3, 3, 0.3, 0.6, st, stone);
    for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
        k.boxr(vec3(sx * h, 25.5, sz * h), vec3(0.7, 25.5, 0.7), 0.0, st, stone);
    }
    windows(k, Vec2::ZERO, h, h, &[20.0, 31.0, 42.0], 0.8, 4.2, 2, 2, Surf::Glow(3.0), WARM_WIN);
    // Clock stage.
    let hc = 7.0;
    k.prism(&rect(Vec2::ZERO, hc, hc), 50.5, 61.0, st, stone, None);
    k.boxr(vec3(0.0, 61.0, 0.0), vec3(hc + 0.4, 0.5, hc + 0.4), 0.0, st, stone);
    for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
        let p = vec2(sx * hc, sz * hc);
        let ring = ngon(p, 0.9, 8, PI / 8.0);
        k.walls(&ring, 50.5, 62.5, st, stone);
        k.pyramid(&ring, 62.5, vec3(p.x, 66.0, p.y), st, stone);
    }
    for i in 0..4 {
        let a = i as f32 * FRAC_PI_2;
        let n = vec3(a.cos(), 0.0, a.sin());
        let c = n * (hc + 0.06) + vec3(0.0, 56.0, 0.0);
        k.panel(c, n, 9.0, 9.0, Surf::Lit(14.0), GOLD);
        k.disc(c + n * 0.08, n, 3.5, 24, Surf::Lit(110.0), rgb(0.96, 0.94, 0.86));
        // Hands at ten past ten.
        let t = vec3(-n.z, 0.0, n.x);
        let hand = |k: &mut Kit, ang: f32, len: f32, w: f32| {
            let d = t * ang.sin() + Vec3::Y * ang.cos();
            let side = d.cross(n).normalize() * (w * 0.5);
            let o = c + n * 0.16;
            k.quad(o - side, o + side, o + side + d * len, o - side + d * len, DIFF, DARK);
        };
        hand(k, -60f32.to_radians(), 2.1, 0.32);
        hand(k, 60f32.to_radians(), 3.1, 0.22);
    }
    // Belfry with a gable on each face.
    k.walls(&rect(Vec2::ZERO, h, h), 61.5, 72.5, st, stone);
    windows(k, Vec2::ZERO, h, h, &[66.5], 2.2, 6.0, 2, 2, DIFF, DARK);
    for i in 0..4 {
        let a = i as f32 * FRAC_PI_2;
        let n = vec3(a.cos(), 0.0, a.sin());
        let t = vec3(-n.z, 0.0, n.x);
        let b = n * (h + 0.1);
        k.tri(b + t * 4.5 + Vec3::Y * 70.0, b - t * 4.5 + Vec3::Y * 70.0, b + Vec3::Y * 74.5, st, stone);
    }
    // Spire: steep cast-iron pyramid, lantern, spirelet, finial.
    k.loft(&rect(Vec2::ZERO, 6.0, 6.0), 72.5, &rect(Vec2::ZERO, 1.8, 1.8), 87.5, ROOF, spire_col);
    for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
        k.pinnacle(vec2(sx * 6.0, sz * 6.0), 72.0, 0.55, 7.0, st, stone);
    }
    k.prism(&rect(Vec2::ZERO, 1.8, 1.8), 87.5, 90.5, Surf::Lit(14.0), GOLD, None);
    k.pyramid(&rect(Vec2::ZERO, 1.5, 1.5), 90.5, vec3(0.0, 95.0, 0.0), ROOF, spire_col);
    k.tube(vec3(0.0, 94.5, 0.0), vec3(0.0, 96.3, 0.0), 0.15, 0.03, 4, METAL, GOLD);
    k.obstacle(vec3(0.0, 96.6, 0.0));
    k.solid(rect(Vec2::ZERO, 7.5, 7.5), 96.0);
}

/// Palace of Westminster (Barry & Pugin, 1840–70): 266 m Gothic river front
/// (OSM footprint, bearing 10.3°), Victoria Tower (98.5 m incl. corner
/// turrets; flagstaff above), octagonal Central Tower (91 m), Westminster
/// Hall. Local +x = east (river), -z = north.
fn westminster(k: &mut Kit) {
    let stone = ANSTON;
    let st = Surf::Flood(5.0);
    let roof_col = rgb(0.32, 0.35, 0.39);
    let win = Surf::Glow(4.0);
    // A range of the palace: walls, steep roof, window rows, buttresses.
    let range = |k: &mut Kit, x0: f32, x1: f32, z0: f32, z1: f32, h: f32, pin: bool| {
        let c = vec2((x0 + x1) * 0.5, (z0 + z1) * 0.5);
        let (hx, hz) = ((x1 - x0) * 0.5, (z1 - z0) * 0.5);
        k.walls(&rect(c, hx, hz), 0.0, h, st, stone);
        let along_z = hz > hx;
        let (rx, rz, yaw) = if along_z { (hz, hx, FRAC_PI_2) } else { (hx, hz, 0.0) };
        k.roof(c, rx + 0.3, rz + 0.3, yaw, h, (rz * 0.9).min(7.0), 0.0, ROOF, roof_col, Some((st, stone)));
        let nx = ((2.0 * hx) / 4.6).round().max(1.0) as usize;
        let nz = ((2.0 * hz) / 4.6).round().max(1.0) as usize;
        let levels: Vec<f32> = [4.5f32, 10.5, 16.5].iter().copied().filter(|&y| y < h - 3.0).collect();
        windows(k, c, hx, hz, &levels, 1.5, 3.6, nx, nz, win, WARM_WIN);
        ribs(k, c, hx, hz, 0.0, h + 0.8, nx + 1, nz + 1, 0.35, 0.7, st, stone);
        if pin {
            parapet_pinnacles(k, c, hx, hz, h, 9.2, 4.5, st, stone);
        }
        k.solid(rect(c, hx, hz), h + (rz * 0.9).min(7.0));
    };
    // River front with three taller pavilions.
    range(k, 36.0, 50.0, -127.0, 117.0, 22.0, true);
    for (z0, z1) in [(-127.0f32, -111.0), (-12.0, 4.0), (101.0, 117.0)] {
        let c = vec2(43.5, (z0 + z1) * 0.5);
        k.walls(&rect(c, 8.0, 8.0), 0.0, 28.0, st, stone);
        k.pyramid(&rect(c, 8.0, 8.0), 28.0, vec3(c.x, 37.0, c.y), ROOF, roof_col);
        parapet_pinnacles(k, c, 8.0, 8.0, 28.0, 4.0, 5.0, st, stone);
        k.solid(rect(c, 8.0, 8.0), 37.0);
    }
    // Inner ranges (N-S) and cross ranges (E-W) around the courts.
    range(k, -6.0, 6.0, -112.0, 102.0, 21.0, false);
    range(k, 16.0, 26.0, -112.0, 102.0, 19.0, false);
    range(k, -50.0, -38.0, -45.0, 108.0, 21.0, true);
    for (z, w) in [(-112.0f32, 9.0), (-58.0, 9.0), (-12.0, 12.0), (42.0, 9.0), (95.0, 9.0)] {
        range(k, -38.0, 36.0, z - w * 0.5, z + w * 0.5, 23.0, false);
    }
    // Westminster Hall (1097, hammerbeam roof): 73 x 21 m, ridge ~ 32 m.
    let hall = vec2(-41.5, -92.0);
    k.walls(&rect(hall, 10.5, 36.5), 0.0, 18.0, st, rgb(0.74, 0.66, 0.50));
    k.roof(hall, 36.8, 10.8, FRAC_PI_2, 18.0, 14.0, 0.0, ROOF, rgb(0.44, 0.45, 0.46), Some((st, rgb(0.74, 0.66, 0.50))));
    k.prism(&ngon(vec2(hall.x, hall.y), 2.2, 6, 0.0), 30.0, 35.0, st, rgb(0.74, 0.66, 0.50), None);
    k.pyramid(&ngon(vec2(hall.x, hall.y), 2.4, 6, 0.0), 35.0, vec3(hall.x, 39.0, hall.y), ROOF, roof_col);
    for sz in [-1.0f32, 1.0] {
        k.pinnacle(hall + vec2(-10.5, sz * 36.5), 18.0, 1.0, 16.0, st, stone);
        k.pinnacle(hall + vec2(10.5, sz * 36.5), 18.0, 1.0, 16.0, st, stone);
    }
    k.solid(rect(hall, 10.5, 36.5), 32.0);
    // Central Tower: octagonal lantern over the Central Lobby and a stone spire.
    let ct = vec2(-11.7, -6.1);
    let oct = ngon(ct, 9.5, 8, PI / 8.0);
    k.walls(&oct, 0.0, 55.0, st, stone);
    for i in 0..8 {
        let a = PI / 8.0 + TAU * (i as f32 + 0.5) / 8.0;
        let n = vec3(a.cos(), 0.0, a.sin());
        let r = 9.5 * (PI / 8.0).cos() + 0.07;
        k.panel(vec3(ct.x, 44.0, ct.y) + n * r, n, 3.0, 12.0, win, WARM_WIN);
        k.pinnacle(ct + ngon(Vec2::ZERO, 9.6, 8, PI / 8.0)[i], 55.0, 0.6, 8.0, st, stone);
    }
    k.pyramid(&ngon(ct, 7.8, 8, PI / 8.0), 55.0, vec3(ct.x, 91.0, ct.y), st, stone);
    k.solid(oct.clone(), 91.0);
    k.obstacle(vec3(ct.x, 91.5, ct.y));
    // Victoria Tower: 98.5 m to the turret tops; Sovereign's Entrance arch.
    let vt = vec2(-34.0, 118.6);
    let hv = 11.0;
    k.prism(&rect(vt, hv, hv), 0.0, 85.0, st, stone, Some((ROOF, roof_col)));
    ribs(k, vt, hv, hv, 0.0, 85.0, 4, 4, 0.45, 0.9, st, stone);
    for y in [22.0f32, 45.0, 66.0, 84.5] {
        k.boxr(vec3(vt.x, y, vt.y), vec3(hv + 0.5, 0.5, hv + 0.5), 0.0, st, stone);
    }
    windows(k, vt, hv, hv, &[30.0, 55.0, 75.0], 1.4, 7.0, 3, 3, win, WARM_WIN);
    k.panel(vec3(vt.x, 9.0, vt.y + hv + 0.6), Vec3::Z, 8.0, 15.0, DIFF, DARK);
    for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
        let p = vt + vec2(sx * (hv + 0.6), sz * (hv + 0.6));
        let ring = ngon(p, 2.0, 8, PI / 8.0);
        k.walls(&ring, 0.0, 89.0, st, stone);
        k.pyramid(&ring, 89.0, vec3(p.x, 98.5, p.y), st, stone);
        k.tube(vec3(p.x, 98.0, p.y), vec3(p.x, 99.5, p.y), 0.2, 0.04, 4, METAL, GOLD);
    }
    for i in 0..4 {
        let a = i as f32 * FRAC_PI_2;
        let (n, t) = (vec2(a.cos(), a.sin()), vec2(-a.sin(), a.cos()));
        for j in [-1.0f32, 0.0, 1.0] {
            k.pinnacle(vt + n * hv + t * (j * 5.0), 85.0, 0.5, 5.0, st, stone);
        }
    }
    k.tube(vec3(vt.x, 85.0, vt.y), vec3(vt.x, 112.0, vt.y), 0.3, 0.15, 6, DIFF, WHITE);
    k.solid(rect(vt, hv + 2.0, hv + 2.0), 99.0);
    k.obstacle(vec3(vt.x, 112.5, vt.y));
}

// ---------------------------------------------------------------- towers

/// BT Tower (1964): 177 m structure + mast to 189 m; narrow glazed shaft,
/// open antenna galleries, enclosed former restaurant levels with the LED
/// ribbon screen near the top.
fn bt_tower(k: &mut Kit) {
    let c = Vec3::ZERO;
    let glass = rgb(0.36, 0.44, 0.52);
    let conc = rgb(0.78, 0.78, 0.76);
    let gl = Surf::Facade(mat::GLASS, 72);
    k.lathe(c, &[(7.8, 0.0), (7.6, 107.0)], 24, Shade::Smooth, gl, &|_, _| glass);
    let gallery = |k: &mut Kit, y: f32, r: f32| {
        k.lathe(c, &[(5.9, y - 0.5), (r, y - 0.5), (r, y + 0.5), (5.9, y + 0.5)], 24, Shade::Flat, Surf::Lit(6.0), &|_, _| conc);
    };
    k.lathe(c, &[(6.0, 107.0), (6.0, 140.0)], 24, Shade::Smooth, Surf::Lit(5.0), &|_, _| conc);
    for y in [108.0f32, 115.0, 122.0, 129.0, 136.0] {
        gallery(k, y, 10.6);
    }
    gallery(k, 140.0, 9.6);
    k.lathe(c, &[(9.0, 140.0), (9.0, 164.0)], 24, Shade::Smooth, Surf::Facade(mat::GLASS, 74), &|_, _| rgb(0.30, 0.36, 0.42));
    for y in [146.0f32, 152.0] {
        gallery(k, y, 9.5);
    }
    // LED ribbon screen.
    k.lathe(c, &[(9.25, 155.0), (9.25, 160.5)], 32, Shade::Smooth, Surf::Glow(45.0), &|i, _| if (i / 4) % 2 == 0 { rgb(0.35, 0.65, 1.0) } else { rgb(0.85, 0.35, 1.0) });
    gallery(k, 164.0, 9.6);
    k.lathe(c, &[(5.6, 164.0), (5.6, 177.0)], 24, Shade::Smooth, Surf::Lit(5.0), &|_, _| conc);
    for y in [168.0f32, 172.5] {
        gallery(k, y, 9.0);
    }
    k.cap(&ngon(Vec2::ZERO, 5.6, 24, 0.0), 177.0, Surf::Lit(5.0), conc);
    // Mast with red / white bands.
    for i in 0..6 {
        let (y0, y1) = (177.0 + i as f32 * 2.0, 179.0 + i as f32 * 2.0);
        let r0 = 1.4 - 0.15 * i as f32;
        k.tube(vec3(0.0, y0, 0.0), vec3(0.0, y1, 0.0), r0, r0 - 0.15, 8, DIFF, if i % 2 == 0 { rgb(0.85, 0.2, 0.15) } else { WHITE });
    }
    k.obstacle(vec3(0.0, 189.5, 0.0));
    for a in 0..4 {
        let a = a as f32 * FRAC_PI_2;
        k.obstacle(vec3(9.8 * a.cos(), 177.5, 9.8 * a.sin()));
        k.obstacle(vec3(8.0 * a.cos(), 95.0, 8.0 * a.sin()));
    }
    k.solid(ngon(Vec2::ZERO, 10.6, 12, 0.0), 189.0);
}

/// 30 St Mary Axe (Foster, 2004): 180 m ogive, 49 m at the base, widest
/// (56.15 m) at level 16 (~66 m); six dark spiralling light wells in the
/// diagrid; glass lens at the top.
fn gherkin(k: &mut Kit) {
    let light = rgb(0.56, 0.70, 0.70);
    let dark = rgb(0.17, 0.24, 0.28);
    let radius = |y: f32| -> f32 {
        if y < 66.0 {
            24.5 + 3.575 * (y / 66.0 * FRAC_PI_2).sin()
        } else {
            let t = (y - 66.0) / (180.5 - 66.0);
            28.075 * (1.0 - t * t).max(0.0).sqrt()
        }
    };
    let segs = 48;
    let ys: Vec<f32> = (0..=40).map(|j| 172.0 * j as f32 / 40.0).collect();
    let prof: Vec<(f32, f32)> = ys.iter().map(|&y| (radius(y), y)).collect();
    let twist = 200f32.to_radians() / 180.0;
    let col = |i: usize, j: usize| {
        let th = TAU * (i as f32 + 0.5) / segs as f32;
        let y = (ys[j] + ys[(j + 1).min(ys.len() - 1)]) * 0.5;
        let u = ((th - y * twist) * 6.0 / TAU).rem_euclid(1.0);
        if u < 0.26 { dark } else { light }
    };
    k.lathe(Vec3::ZERO, &prof, segs, Shade::Split, Surf::Facade(mat::GLASS, 76), &col);
    // Lens: the top dome.
    let top: Vec<(f32, f32)> = (0..=4).map(|i| {
        let y = 172.0 + 8.5 * i as f32 / 4.0;
        (radius(y.min(180.0)).max(0.05) * (1.0 - 0.15 * i as f32 / 4.0), y.min(180.5))
    }).collect();
    k.lathe(Vec3::ZERO, &top, segs / 2, Shade::Smooth, Surf::Lit(14.0), &|_, _| rgb(0.40, 0.48, 0.52));
    k.obstacle(vec3(0.0, 181.0, 0.0));
    for a in 0..4 {
        let a = a as f32 * FRAC_PI_2 + 0.4;
        k.obstacle(vec3(radius(120.0) * a.cos(), 120.0, radius(120.0) * a.sin()));
    }
    k.solid(ngon(Vec2::ZERO, 28.0, 16, 0.0), 120.0);
    k.solid(ngon(Vec2::ZERO, 19.0, 12, 0.0), 160.0);
    k.solid(ngon(Vec2::ZERO, 9.0, 8, 0.0), 180.0);
}

/// 20 Fenchurch Street (Viñoly, 2014): 160 m, top-heavy — the north and
/// south facades curve outward in both plan and elevation (OSM top outline
/// ~62 x 74 m); glazed Sky Garden vault on top.
fn walkie_talkie(k: &mut Kit) {
    let glass = rgb(0.62, 0.70, 0.77);
    let hxy = |y: f32| 26.0 + 5.0 * (y / 152.0).powf(1.7);
    let hx = hxy(152.0);
    let ys: Vec<f32> = (0..=16).map(|j| 152.0 * j as f32 / 16.0).collect();
    let zs = |y: f32| 21.0 + 14.5 * (y / 152.0).powf(1.7);
    let zn = |y: f32| 21.0 + 12.5 * (y / 152.0).powf(1.7);
    let bow = |x: f32, y: f32| (1.5 + 2.5 * y / 152.0) * (1.0 - (x / hxy(y)).powi(2)).max(0.0);
    let nx = 10;
    let xs: Vec<f32> = (0..=nx).map(|i| -hx + 2.0 * hx * i as f32 / nx as f32).collect();
    let xs_at = |y: f32| -> Vec<f32> { (0..=nx).map(|i| -hxy(y) + 2.0 * hxy(y) * i as f32 / nx as f32).collect() };
    let gl = |seed| Surf::Facade(mat::GLASS, seed);
    // South (+z) and north (-z) faces: curved grids.
    for (sign, f, seed) in [(1.0f32, &zs as &dyn Fn(f32) -> f32, 80u32), (-1.0, &zn, 82)] {
        let rows: Vec<Vec<Vec3>> = ys.iter().map(|&y| xs_at(y).iter().map(|&x| vec3(x, y, sign * (f(y) + bow(x, y)))).collect()).collect();
        k.grid(&rows, false, Shade::Smooth, gl(seed), &|_, _| glass);
    }
    // East / west faces: flat, outline follows the curved faces.
    for (sx, seed) in [(1.0f32, 84u32), (-1.0, 86)] {
        let rows: Vec<Vec<Vec3>> = ys.iter().map(|&y| vec![vec3(sx * hxy(y), y, -zn(y)), vec3(sx * hxy(y), y, zs(y))]).collect();
        k.grid(&rows, false, Shade::Flat, gl(seed), &|_, _| rgb(0.55, 0.62, 0.68));
    }
    // Sky Garden vault: rises toward the north.
    let yt = 152.0;
    let n = 8;
    let vault = |x: f32, t: f32| {
        let z = -(zn(yt) + bow(x, yt)) + t * (zn(yt) + zs(yt) + 2.0 * bow(x, yt));
        let y = yt + 8.0 * (PI * t).sin() * (1.0 - 0.35 * t);
        vec3(x, y, z)
    };
    let rows: Vec<Vec<Vec3>> = xs.iter().map(|&x| (0..=n).map(|i| vault(x, i as f32 / n as f32)).collect()).collect();
    k.grid(&rows, false, Shade::Smooth, Surf::Lit(10.0), &|_, _| rgb(0.70, 0.78, 0.82));
    for sx in [-hx, hx] {
        let rows = vec![(0..=n).map(|i| {
            let p = vault(sx, i as f32 / n as f32);
            vec3(p.x, yt, p.z)
        }).collect(), (0..=n).map(|i| vault(sx, i as f32 / n as f32)).collect::<Vec<_>>()];
        k.grid(&rows, false, Shade::Flat, Surf::Lit(10.0), &|_, _| rgb(0.66, 0.74, 0.78));
    }
    for (x, z) in [(-hx, -zn(yt)), (hx, -zn(yt)), (-hx, zs(yt)), (hx, zs(yt))] {
        k.obstacle(vec3(x, yt + 1.0, z));
    }
    k.solid(rect_xz(-hx, hx, -36.0, 39.0), 160.0);
}

/// One Canada Square (Pelli, 1991): 235 m, 50 storeys, ~50 m square with
/// notched corners, stainless-steel cladding and a pyramid roof whose apex
/// carries the famous flashing aircraft-warning light.
fn canada_square(k: &mut Kit) {
    let steel = rgb(0.80, 0.82, 0.85);
    let (a, b) = (25.0, 21.5);
    let plan = vec![
        vec2(a, -b), vec2(a, b), vec2(b, b), vec2(b, a), vec2(-b, a), vec2(-b, b),
        vec2(-a, b), vec2(-a, -b), vec2(-b, -b), vec2(-b, -a), vec2(b, -a), vec2(b, -b),
    ];
    k.walls(&plan, 0.0, 202.0, Surf::Facade(mat::WALL, 90), steel);
    k.walls(&plan, 202.0, 205.0, METAL, steel);
    k.cap(&plan, 205.0, ROOF, rgb(0.6, 0.6, 0.62));
    k.pyramid(&rect(Vec2::ZERO, 22.5, 22.5), 205.0, vec3(0.0, 235.0, 0.0), METAL, rgb(0.82, 0.84, 0.86));
    k.strobe(vec3(0.0, 236.0, 0.0), 0.37);
    for c in [vec2(a, a), vec2(-a, a), vec2(a, -a), vec2(-a, -a)] {
        k.obstacle(vec3(c.x * 0.9, 205.5, c.y * 0.9));
        k.obstacle(vec3(c.x * 0.9, 110.0, c.y * 0.9));
    }
    k.solid(rect(Vec2::ZERO, a, a), 205.0);
    k.solid(rect(Vec2::ZERO, 12.0, 12.0), 235.0);
}

// ---------------------------------------------------------------- venues

/// The O2 (Millennium Dome, 1999): fabric roof ~ 346 m across (OSM), 52 m
/// high at the centre, hung from twelve 100 m yellow masts on a 107 m ring
/// (OSM mast nodes, first at bearing 23.7°).
fn the_o2(k: &mut Kit) {
    let r_edge = 172.0;
    let (h_edge, h_top) = (11.0f32, 52.0f32);
    let rs = (r_edge * r_edge + (h_top - h_edge).powi(2)) / (2.0 * (h_top - h_edge));
    let yr = |r: f32| h_edge + (rs * rs - r * r).max(0.0).sqrt() - (rs - (h_top - h_edge));
    let fabric = rgb(0.93, 0.92, 0.88);
    let seam = rgb(0.80, 0.79, 0.76);
    let segs = 96;
    let prof: Vec<(f32, f32)> = (0..=14).rev().map(|j| {
        let r = r_edge * j as f32 / 14.0;
        (r.max(0.01), yr(r))
    }).collect();
    k.lathe(Vec3::ZERO, &prof, segs, Shade::Split, Surf::Flood(3.0), &|i, j| {
        if i % 4 == 0 || j % 4 == 3 { seam } else { fabric }
    });
    // Perimeter wall.
    k.lathe(Vec3::ZERO, &[(r_edge, 0.0), (r_edge, h_edge)], segs, Shade::Smooth, Surf::Lit(2.0), &|_, _| rgb(0.62, 0.63, 0.66));
    // Masts: lattice-looking yellow tubes with outrigger struts and cables.
    let yellow = rgb(0.96, 0.78, 0.10);
    let cable = rgb(0.25, 0.25, 0.27);
    for m in 0..12 {
        let brg = (23.7 + 30.0 * m as f32).to_radians();
        let dir = vec2(brg.sin(), -brg.cos());
        let base = dir * 107.0;
        let top = vec3(base.x, 100.0, base.y) + vec3(dir.x, 0.0, dir.y) * 6.0;
        k.tube(vec3(base.x, yr(107.0) - 2.0, base.y), top, 1.5, 0.9, 8, Surf::Lit(9.0), yellow);
        // Cross-arm struts near the top.
        let side = vec3(-dir.y, 0.0, dir.x);
        for s in [-1.0f32, 1.0] {
            k.tube(top - vec3(0.0, 22.0, 0.0) + vec3(dir.x, 0.0, dir.y) * 2.0, top + side * s * 7.0 - vec3(0.0, 8.0, 0.0), 0.35, 0.35, 5, Surf::Lit(9.0), yellow);
        }
        // Hanging cables down to the roof (inward / outward / sideways).
        for (dr, da) in [(-60.0f32, 0.0f32), (-30.0, 0.0), (35.0, 0.0), (55.0, 0.0), (0.0, 0.16), (0.0, -0.16), (-30.0, 0.12), (-30.0, -0.12)] {
            let r = (107.0 + dr).clamp(5.0, r_edge - 4.0);
            let a = brg + da;
            let p = vec2(a.sin(), -a.cos()) * r;
            k.tube(top, vec3(p.x, yr(r) + 0.2, p.y), 0.12, 0.12, 3, DIFF, cable);
        }
        k.obstacle(top + vec3(0.0, 0.8, 0.0));
    }
    k.solid(ngon(Vec2::ZERO, r_edge, 24, 0.0), h_edge + 8.0);
    k.solid(ngon(Vec2::ZERO, 100.0, 16, 0.0), h_top);
    for m in 0..12 {
        let brg = (23.7 + 30.0 * m as f32).to_radians();
        k.solid(ngon(vec2(brg.sin(), -brg.cos()) * 110.0, 3.0, 6, 0.0), 100.0);
    }
}

/// Wembley Stadium (Foster / Populous, 2007): bowl ~ 292 x 285 m (OSM),
/// roof ~ 52 m, and the 133 m arch: 315 m span, 7.4 m lattice tube,
/// leaning 22° from vertical over the north roof. Local +x along the pitch
/// (bearing 97°).
fn wembley(k: &mut Kit) {
    let clad = rgb(0.72, 0.74, 0.77);
    let roof = rgb(0.80, 0.81, 0.82);
    let seats = rgb(0.62, 0.10, 0.10);
    let (a, b) = (146.0, 140.0);
    let segs = 72;
    let ring = |a: f32, b: f32, n: f32| superellipse(Vec2::ZERO, a, b, n, segs);
    let to3 = |r: &[Vec2], y: f32| r.iter().map(|p| vec3(p.x, y, p.y)).collect::<Vec<_>>();
    let outer = ring(a, b, 3.2);
    // Facade: lower glazed band, cladding above.
    k.grid(&[to3(&outer, 0.0), to3(&outer, 12.0)], true, Shade::Flat, Surf::Facade(mat::GLASS, 92), &|_, _| rgb(0.40, 0.46, 0.52));
    let upper = ring(a + 2.0, b + 2.0, 3.2);
    k.grid(&[to3(&outer, 12.0), to3(&upper, 40.0)], true, Shade::Flat, Surf::Flood(4.0), &|i, _| if i % 3 == 0 { rgb(0.62, 0.64, 0.67) } else { clad });
    // Roof: from the outer edge (high) down to the opening over the pitch.
    let r_out = ring(a + 3.0, b + 3.0, 3.2);
    let r_mid = ring(118.0, 108.0, 3.0);
    let r_in = ring(82.0, 66.0, 2.6);
    k.grid(&[to3(&upper, 40.0), to3(&r_out, 49.0), to3(&r_mid, 50.0), to3(&r_in, 44.0)], true, Shade::Smooth, Surf::Flood(2.0), &|_, _| roof);
    // Bowl: seats from under the roof edge down to the pitch.
    let s0 = ring(84.0, 68.0, 2.6);
    let s1 = ring(70.0, 52.0, 2.6);
    let s2 = ring(60.0, 41.0, 3.2);
    k.grid(&[to3(&s2, 1.0), to3(&s1, 16.0), to3(&s0, 40.0)], true, Shade::Smooth, DIFF, &|_, _| seats);
    k.cap(&ring(60.5, 41.5, 3.2), 0.6, Surf::Plain(mat::GRASS), rgb(0.30, 0.52, 0.22));
    // The arch: lattice tube in a plane through the feet, leaning north.
    let tilt = 22f32.to_radians();
    let half = 157.5;
    let l = 133.0 / tilt.cos();
    let n = 40;
    let pts: Vec<Vec3> = (0..=n).map(|i| {
        let s = -half + 2.0 * half * i as f32 / n as f32;
        let f = (1.0 - (s / half).powi(2)).max(0.0).powf(0.8);
        vec3(s, l * f * tilt.cos(), -l * f * tilt.sin())
    }).collect();
    let white = rgb(0.94, 0.94, 0.93);
    k.path(&pts, &|i| {
        let t = (i as f32 / n as f32 - 0.5).abs() * 2.0;
        3.7 - 1.6 * t * t
    }, 12, false, Surf::Lit(30.0), white);
    // Ring stiffeners along the arch (lattice read).
    for i in (2..n - 1).step_by(2) {
        let t = (pts[i + 1] - pts[i - 1]).normalize();
        let r = 3.75 - 1.6 * ((i as f32 / n as f32 - 0.5).abs() * 2.0).powi(2);
        let u = t.cross(Vec3::Y).normalize_or(Vec3::Z);
        let v = t.cross(u);
        let ring: Vec<Vec3> = (0..10).map(|q| {
            let a = TAU * q as f32 / 10.0;
            pts[i] + (u * a.cos() + v * a.sin()) * (r + 0.15)
        }).collect();
        k.path(&ring, &|_| 0.18, 3, true, Surf::Lit(30.0), white);
    }
    // Cables from the arch to the roof (north and south).
    for i in (6..n - 5).step_by(3) {
        let p = pts[i];
        let xs = p.x * 0.92;
        let zn = -b * (1.0 - (xs / a).abs().powf(3.2)).max(0.0).powf(1.0 / 3.2) * 0.85;
        let zs = -zn * 0.75;
        k.tube(p, vec3(xs, 49.0, zn), 0.12, 0.12, 3, DIFF, rgb(0.6, 0.6, 0.6));
        k.tube(p, vec3(xs, 48.0, zs), 0.12, 0.12, 3, DIFF, rgb(0.6, 0.6, 0.6));
    }
    k.obstacle(pts[n / 2] + vec3(0.0, 4.0, 0.0));
    k.obstacle(pts[n / 4] + vec3(0.0, 4.0, 0.0));
    k.obstacle(pts[3 * n / 4] + vec3(0.0, 4.0, 0.0));
    k.solid(outer.clone(), 50.0);
    for i in (0..=n).step_by(4) {
        let p = pts[i];
        k.solid(rect(vec2(p.x, p.z), 6.0, 6.0), p.y + 4.0);
    }
}

/// Battersea Power Station (Scott, 1933/55): brick body ~ 160 x 170 m, the
/// central boiler house carrying four 103 m white chimneys on 50 m plinths
/// (chimney positions and heights from OSM).
fn battersea(k: &mut Kit) {
    let brick = rgb(0.60, 0.38, 0.29);
    let brick_dark = rgb(0.52, 0.32, 0.25);
    let wall = |seed| Surf::Facade(mat::WALL, seed);
    let roof_col = rgb(0.36, 0.34, 0.33);
    let bx = -2.1; // boiler house centre line
    // Boiler house (centre spine) and the two turbine halls.
    k.walls(&rect_xz(bx - 27.0, bx + 27.0, -84.0, 84.0), 0.0, 50.0, wall(94), brick);
    k.cap(&rect_xz(bx - 27.0, bx + 27.0, -84.0, 84.0), 50.0, ROOF, roof_col);
    for (x0, x1, seed) in [(-80.0f32, bx - 27.0, 96u32), (bx + 27.0, 78.0, 98)] {
        k.walls(&rect_xz(x0, x1, -76.0, 76.0), 0.0, 38.0, wall(seed), brick);
        k.cap(&rect_xz(x0, x1, -76.0, 76.0), 38.0, ROOF, roof_col);
        // Art-deco fluted corner piers.
        for x in [x0, x1] {
            for z in [-76.0f32, 76.0] {
                k.boxr(vec3(x, 21.0, z), vec3(1.6, 21.0, 1.6), 0.0, DIFF, brick_dark);
            }
        }
    }
    // Vertical brick fins on the long facades of the boiler house.
    for i in 0..9 {
        let z = -72.0 + 18.0 * i as f32;
        for x in [bx - 27.4, bx + 27.4] {
            k.boxr(vec3(x, 30.0, z), vec3(0.5, 20.0, 1.0), 0.0, DIFF, brick_dark);
        }
    }
    // Chimneys: square stepped plinths, fluted white columns, dark rim.
    let chims = [vec2(-26.8, -77.2), vec2(22.9, -78.0), vec2(-26.4, 78.6), vec2(21.9, 79.1)];
    let white = rgb(0.92, 0.91, 0.88);
    for c in chims {
        k.prism(&rect(c, 7.5, 7.5), 46.0, 53.0, wall(100), brick, Some((ROOF, roof_col)));
        k.prism(&rect(c, 6.0, 6.0), 53.0, 57.0, Surf::Lit(14.0), white, None);
        let prof = [(4.6, 57.0), (4.3, 75.0), (4.0, 98.0), (4.2, 99.0), (4.2, 102.0), (3.6, 103.0)];
        k.lathe(vec3(c.x, 0.0, c.y), &prof, 20, Shade::Split, Surf::Lit(14.0), &|i, j| {
            if j >= 3 { rgb(0.55, 0.55, 0.55) } else if i % 2 == 0 { white } else { rgb(0.84, 0.83, 0.80) }
        });
        k.disc(vec3(c.x, 101.5, c.y), Vec3::Y, 3.7, 12, DIFF, DARK);
        k.obstacle(vec3(c.x, 103.5, c.y));
        k.solid(ngon(c, 5.0, 8, 0.0), 103.0);
    }
    k.solid(rect_xz(-80.0, 78.0, -84.0, 84.0), 50.0);
}
