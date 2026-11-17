//! Procedural ships: spawning at the density around the eye, picking a
//! class by the per-layer density, placing them only on water deep enough
//! for their draught, and steering them along the lanes the density draws
//! (greedy look-ahead over candidate headings: stay where the traffic is,
//! turn smoothly, never into land or shoals).
//!
//! Everything works in the world frame (x east, z south) through [`Sea`],
//! so the tests run it on synthetic seas.

use super::density::{self, LAYERS, Layer, N, Sample};
use super::types::{Class, Model, model_for};
use glam::{Vec2, vec2};

/// What the ships need to know about the world.
pub trait Sea {
    /// Water at `xz`? None = unknown (no scenery there yet).
    fn water(&self, xz: Vec2) -> Option<bool>;
    /// Sea depth (m, + down), if known; negative: known not to be sea
    /// (land, or inland water without a depth).
    fn depth(&self, xz: Vec2) -> Option<f32>;
    /// The vessel density at `xz`.
    fn density(&self, xz: Vec2) -> Sample;
    /// Month 1..12 (leisure peaks in summer).
    fn month(&self) -> u32 {
        7
    }
}

/// Per-layer ships per km² at `xz`: the data, or the generic mix on water
/// without data (None: land / unknown).
pub fn ships_per_km2(sea: &dyn Sea, xz: Vec2) -> Option<[f32; N]> {
    match sea.density(xz) {
        Sample::Ships(v) => Some(v),
        Sample::Land => None,
        Sample::Uncovered => (sea.water(xz) != Some(false)).then(|| density::generic(sea.depth(xz).filter(|d| *d >= 0.0), sea.month())),
        Sample::Unknown => None,
    }
}

/// Small deterministic RNG (splitmix64).
#[derive(Clone, Debug)]
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    /// Uniform 0..1.
    pub fn f(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    pub fn range(&mut self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.f()
    }
}

/// Compass heading (rad, 0 = north, clockwise) → world xz direction.
pub fn dir(h: f32) -> Vec2 {
    vec2(h.sin(), -h.cos())
}

/// World xz direction → compass heading (rad, 0..2π).
pub fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y).rem_euclid(std::f32::consts::TAU)
}

/// Signed smallest angle a → b (rad).
pub fn ang_diff(a: f32, b: f32) -> f32 {
    (b - a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

/// One ship.
#[derive(Clone, Debug)]
pub struct Ship {
    /// Stable id (MMSI for live ships).
    pub id: u64,
    /// Live: the AIS MMSI.
    pub mmsi: Option<u32>,
    pub name: String,
    pub class: Class,
    pub model: Model,
    /// The density layer it was spawned from (procedural).
    pub layer: Option<Layer>,
    pub length: f32,
    pub beam: f32,
    pub draught: f32,
    /// World xz (current frame).
    pub pos: Vec2,
    /// Water surface height there (world y).
    pub level: f32,
    /// Compass heading, rad.
    pub heading: f32,
    /// Speed through the water, m/s.
    pub speed: f32,
    /// Cruising speed, m/s (0 = anchored / moored).
    pub cruise: f32,
    /// Heading it steers for.
    pub want: f32,
    /// Next steering decision (session time, s).
    pub think_at: f64,
    /// Turn-rate limit, rad/s.
    pub rot_max: f32,
    pub anchored: bool,
    /// Blocked ahead with nowhere to go: removed once out of sight.
    pub stuck: bool,
    /// Recent positions (world xz, oldest first) for the wake, with times.
    pub trail: Vec<(Vec2, f64)>,
    pub seed: u32,
}

impl Ship {
    pub fn new(id: u64, class: Class, length: f32, beam: f32, draught: f32, pos: Vec2, heading: f32) -> Ship {
        Ship {
            id,
            mmsi: None,
            name: String::new(),
            class,
            model: model_for(class, length),
            layer: None,
            length,
            beam,
            draught,
            pos,
            level: 0.0,
            heading,
            speed: 0.0,
            cruise: 0.0,
            want: heading,
            think_at: 0.0,
            rot_max: (2.0f32 / length.max(5.0)).clamp(0.004, 0.12),
            anchored: false,
            stuck: false,
            trail: Vec::new(),
            seed: (id ^ (id >> 32)) as u32,
        }
    }

    pub fn fwd(&self) -> Vec2 {
        dir(self.heading)
    }

    /// Velocity, world xz (m/s).
    pub fn vel(&self) -> Vec2 {
        self.fwd() * self.speed
    }
}

/// Is a ship of `draught` fine at `xz`: water (if known), not land in the
/// density (if no scenery), deep enough (if the depth is known)?
pub fn navigable(sea: &dyn Sea, xz: Vec2, draught: f32) -> bool {
    let need = draught * 1.2 + 1.0;
    // The coarse DEMs often call harbours, docks and dredged estuaries land
    // (bathy/ marks those "not sea", -1, or "unsure", UNSURE_DEPTH): there the
    // map or the vessel density decide. Real depths are a hard limit.
    let unsure = |d: f32| d < 0.0 || (d - crate::bathy::UNSURE_DEPTH).abs() < 0.01;
    let depth_ok = || sea.depth(xz).is_none_or(|d| unsure(d) || d >= need);
    match sea.water(xz) {
        Some(false) => false,
        Some(true) => depth_ok(),
        // No scenery here yet: the density data must know water there (ships
        // go there); without density data a real depth must be known.
        None => match sea.density(xz) {
            Sample::Ships(_) => depth_ok(),
            Sample::Uncovered => sea.depth(xz).is_some_and(|d| !unsure(d) && d >= need),
            Sample::Land | Sample::Unknown => false,
        },
    }
}

/// The whole hull fits: bow, stern, both sides.
pub fn fits(sea: &dyn Sea, xz: Vec2, heading: f32, length: f32, beam: f32, draught: f32) -> bool {
    let f = dir(heading);
    let r = vec2(-f.y, f.x);
    [Vec2::ZERO, f * length * 0.5, -f * length * 0.5, r * beam * 0.6, -r * beam * 0.6].iter().all(|o| navigable(sea, xz + *o, draught))
}

/// Density of one layer (ships / km²; 0 where none / unknown).
fn layer_at(sea: &dyn Sea, xz: Vec2, l: usize) -> f32 {
    ships_per_km2(sea, xz).map(|v| v[l]).unwrap_or(0.0)
}

/// The lane direction at `xz` for layer `l`: the principal axis across
/// which the density changes least (structure tensor of its gradient over
/// a few km), as a heading 0..π, with its strength 0..1 (0 = no lane).
pub fn lane_axis(sea: &dyn Sea, xz: Vec2, l: usize, step: f32) -> (f32, f32) {
    let (mut sxx, mut sxy, mut syy) = (0.0f32, 0.0f32, 0.0f32);
    for j in -2..=2 {
        for i in -2..=2 {
            let p = xz + vec2(i as f32, j as f32) * step;
            let g =
                vec2(layer_at(sea, p + vec2(step, 0.0), l) - layer_at(sea, p - vec2(step, 0.0), l), layer_at(sea, p + vec2(0.0, step), l) - layer_at(sea, p - vec2(0.0, step), l));
            // Log-ish weighting: lanes of very different density count alike.
            let g = g / (layer_at(sea, p, l) + 1e-3);
            sxx += g.x * g.x;
            sxy += g.x * g.y;
            syy += g.y * g.y;
        }
    }
    let tr = sxx + syy;
    if tr < 1e-9 {
        return (0.0, 0.0);
    }
    // Eigenvector of the larger eigenvalue = across the lane.
    let disc = ((sxx - syy) * (sxx - syy) * 0.25 + sxy * sxy).sqrt();
    let l1 = 0.5 * tr + disc;
    let across = if sxy.abs() > 1e-12 {
        vec2(l1 - syy, sxy).normalize_or_zero()
    } else if sxx >= syy {
        vec2(1.0, 0.0)
    } else {
        vec2(0.0, 1.0)
    };
    let along = vec2(-across.y, across.x);
    let strength = (2.0 * disc / tr).clamp(0.0, 1.0);
    (heading_of(along).rem_euclid(std::f32::consts::PI), strength)
}

/// How far ahead (m) a ship checks its way.
pub fn lookahead(s: &Ship) -> f32 {
    (s.cruise.max(s.speed) * 150.0).clamp(4.0 * s.length, 3000.0).max(300.0)
}

/// Score of steering `h` from `s`: traffic along the way (lanes), little
/// turning, and a heavy penalty for land or shoals ahead (the nearer the
/// worse). None if blocked within the first stretch.
pub fn score_heading(sea: &dyn Sea, s: &Ship, h: f32, l: Option<usize>) -> Option<f32> {
    let la = lookahead(s);
    let d = dir(h);
    let mut score = 0.0;
    for k in 1..=6 {
        let dist = la * k as f32 / 6.0;
        let p = s.pos + d * dist;
        if !navigable(sea, p, s.draught) {
            if k <= 2 {
                return None;
            }
            score -= 40.0 / k as f32;
            break;
        }
        if let Some(l) = l {
            score += (1.0 + layer_at(sea, p, l) * 2000.0).ln() * 0.35;
        }
    }
    // Margin at the sides of the first leg (narrow channels).
    let r = vec2(-d.y, d.x);
    for side in [-1.0f32, 1.0] {
        if !navigable(sea, s.pos + d * la * 0.25 + r * side * (s.beam * 2.0 + 30.0), s.draught) {
            score -= 1.5;
        }
    }
    score -= ang_diff(s.heading, h).abs() * 2.2;
    Some(score)
}

/// Pick the heading to steer for (`rng` for the loiterers' wandering).
pub fn steer(sea: &dyn Sea, s: &Ship, rng: &mut Rng) -> Option<f32> {
    let l = s.layer.map(|l| l.index());
    // Lane direction (the side nearer the current heading).
    let mut pref = s.heading;
    if let Some(li) = l.filter(|_| !s.class.loiters()) {
        let (axis, strength) = lane_axis(sea, s.pos, li, 900.0);
        if strength > 0.25 {
            let a = if ang_diff(s.heading, axis).abs() < std::f32::consts::FRAC_PI_2 { axis } else { axis + std::f32::consts::PI };
            pref = s.heading + ang_diff(s.heading, a) * 0.6;
        }
    } else if s.class.loiters() {
        pref = s.heading + rng.range(-0.6, 0.6);
    }
    let mut best: Option<(f32, f32)> = None;
    for off in [0.0f32, 10.0, -10.0, 22.0, -22.0, 40.0, -40.0, 65.0, -65.0, 95.0, -95.0, 135.0, -135.0, 180.0] {
        let h = (pref + off.to_radians()).rem_euclid(std::f32::consts::TAU);
        if let Some(sc) = score_heading(sea, s, h, l) {
            let sc = sc - ang_diff(pref, h).abs() * 0.8;
            if best.is_none_or(|b| sc > b.1) {
                best = Some((h, sc));
            }
        }
    }
    best.map(|b| b.0)
}

/// Advance a procedural ship by `dt` (session time `t`).
pub fn step(sea: &dyn Sea, s: &mut Ship, t: f64, dt: f32, rng: &mut Rng) {
    if s.anchored {
        // Swinging slowly at the anchor.
        s.speed = 0.0;
        s.heading = (s.heading + (t as f32 * 0.002 + s.seed as f32).sin() * 0.0004 * dt).rem_euclid(std::f32::consts::TAU);
        return;
    }
    if t >= s.think_at {
        s.think_at = t + 4.0 + (s.seed % 7) as f64 * 0.5;
        match steer(sea, s, rng) {
            Some(h) => {
                s.want = h;
                s.stuck = false;
            }
            None => {
                // Hemmed in: stop and turn about.
                s.want = (s.heading + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU);
                s.stuck = true;
            }
        }
    }
    let turn = ang_diff(s.heading, s.want);
    let rate = s.rot_max * (0.3 + 0.7 * (s.speed / s.cruise.max(0.5)).clamp(0.0, 1.0));
    s.heading = (s.heading + turn.clamp(-rate * dt, rate * dt)).rem_euclid(std::f32::consts::TAU);
    // Slower in hard turns and when blocked.
    let target = if s.stuck { s.cruise * 0.15 } else { s.cruise * (1.0 - 0.5 * (turn.abs() / std::f32::consts::PI)) };
    let acc = (s.cruise * 0.01).max(0.02);
    s.speed += (target - s.speed).clamp(-acc * 2.0 * dt, acc * dt);
    let next = s.pos + s.fwd() * s.speed * dt;
    // Never onto land: hold position (the next decision turns away).
    if navigable(sea, next + s.fwd() * s.length * 0.5, s.draught) {
        s.pos = next;
    } else {
        s.speed = 0.0;
        s.think_at = t;
    }
}

/// Expected ships per layer within `radius` of `c` (sampled on a `cell` m grid),
/// and the cells (centre, per-layer weight) to draw spawn points from.
pub struct Census {
    pub centre: Vec2,
    pub expected: [f32; N],
    pub cells: Vec<(Vec2, [f32; N])>,
    pub cell: f32,
}

pub fn census(sea: &dyn Sea, c: Vec2, radius: f32, cell: f32) -> Census {
    let n = (radius / cell).ceil() as i32;
    let area_km2 = cell * cell * 1e-6;
    let mut expected = [0.0f32; N];
    let mut cells = Vec::new();
    for j in -n..=n {
        for i in -n..=n {
            let p = c + vec2(i as f32, j as f32) * cell;
            if p.distance(c) > radius {
                continue;
            }
            if let Some(v) = ships_per_km2(sea, p) {
                let w = v.map(|x| x * area_km2);
                for l in 0..N {
                    expected[l] += w[l];
                }
                if w.iter().any(|x| *x > 0.0) {
                    cells.push((p, w));
                }
            }
        }
    }
    Census { centre: c, expected, cells, cell }
}

/// Draw a spawn point for layer `l` from the census (weighted by the
/// layer's density), at least `min_dist` from `eye`.
pub fn pick_cell(cen: &Census, l: usize, eye: Vec2, min_dist: f32, rng: &mut Rng) -> Option<Vec2> {
    let total: f32 = cen.cells.iter().filter(|c| c.0.distance(eye) >= min_dist).map(|c| c.1[l]).sum();
    if total <= 0.0 {
        return None;
    }
    let mut u = rng.f() * total;
    for (p, w) in cen.cells.iter().filter(|c| c.0.distance(eye) >= min_dist) {
        u -= w[l];
        if u <= 0.0 {
            return Some(*p + vec2(rng.range(-0.5, 0.5), rng.range(-0.5, 0.5)) * cen.cell);
        }
    }
    None
}

/// Try to create a ship of layer `l` near `p`: class by the layer, size by
/// the class, anchored at hot spots, heading along the lane; None if it
/// does not fit there (land, shoal, another ship too close).
pub fn spawn(sea: &dyn Sea, l: Layer, p: Vec2, others: &[Ship], id: u64, rng: &mut Rng) -> Option<Ship> {
    let li = l.index();
    let class = density::spawn_class(l, rng.f());
    let (lo, hi) = class.length_range();
    // Smaller ships are commoner.
    let u = rng.f();
    let length = lo + (hi - lo) * u * u;
    let beam = length * class.beam_ratio();
    let draught = class.draught(length);
    let here = ships_per_km2(sea, p).map(|v| v[li]).unwrap_or(0.0);
    // Hot spots (ports, anchorages): many ships stopped.
    let hot = smooth(0.08, 0.6, here);
    let anchored = rng.f() < class.anchor_share() * hot;
    let (axis, strength) = lane_axis(sea, p, li, 900.0);
    let mut heading = if strength > 0.25 && !class.loiters() { axis + if rng.f() < 0.5 { 0.0 } else { std::f32::consts::PI } } else { rng.range(0.0, std::f32::consts::TAU) };
    heading = heading.rem_euclid(std::f32::consts::TAU);
    // Turn until it fits (a ship along the channel, not across it).
    let mut ok = None;
    for off in [0.0f32, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0] {
        let h = (heading + off.to_radians()).rem_euclid(std::f32::consts::TAU);
        if fits(sea, p, h, length, beam, draught) {
            ok = Some(h);
            break;
        }
    }
    let heading = ok?;
    if others.iter().any(|o| o.pos.distance(p) < (o.length + length) * 0.8 + 40.0) {
        return None;
    }
    let mut s = Ship::new(id, class, length, beam, draught, p, heading);
    s.layer = Some(l);
    s.anchored = anchored;
    let (a, b) = class.speed_kn();
    s.cruise = if anchored { 0.0 } else { rng.range(a, b) * 0.514_444 };
    s.speed = s.cruise;
    s.name = format!("{} {}", class.name(), id % 1000);
    s.think_at = 0.0;
    Some(s)
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Layer names for diagnostics.
pub fn layer_name(l: usize) -> &'static str {
    match LAYERS[l].0 {
        Layer::Cargo => "cargo",
        Layer::Tanker => "tanker",
        Layer::Passenger => "passenger",
        Layer::HighSpeed => "hsc",
        Layer::Fishing => "fishing",
        Layer::Tug => "tug",
        Layer::Pleasure => "pleasure",
        Layer::Sailing => "sailing",
    }
}
