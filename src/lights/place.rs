//! Where the lights are: city lights per tile (tile workers), airport lighting
//! from the runways, and the aircraft's own lights.

use super::{colour, col_param, kind, LightInst};
use crate::osm::{Proj, TileId};
use glam::{vec2, vec3, Quat, Vec2, Vec3};

// ---------------------------------------------------------------- city

/// Moving traffic on one stretch of major road (tile frame), drawn as head /
/// tail-light pairs animated in the vertex shader. 32 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TrafficSeg {
    pub a: [f32; 2],
    pub b: [f32; 2],
    /// Road width (m).
    pub width: f32,
    /// Per-segment random 0..1.
    pub seed: f32,
    /// Cars per direction on this stretch (at the busiest hour).
    pub cars: f32,
    /// Speed (m/s).
    pub speed: f32,
}

/// Light sources of one map tile, in the tile's frame.
#[derive(Clone, Debug, Default)]
pub struct TileLights {
    pub lights: Vec<LightInst>,
    pub traffic: Vec<TrafficSeg>,
    /// Far (low-zoom) tiles: upward light flux (lumen) of the built-up area on
    /// a `GLOW_N` x `GLOW_N` grid over the tile (row 0 = north), for skyglow.
    pub glow: Vec<f32>,
}

/// Skyglow source grid per far tile.
pub const GLOW_N: usize = 8;

/// Upward light flux of built-up land (lm per m²) by urban class: residential,
/// commercial, industrial, institutional. (~6 Mlm/km² emitted by lamps,
/// windows and signs in a city centre, ~15–20 % of it going up.)
const URBAN_FLUX: [f32; 4] = [0.55, 1.6, 0.8, 0.7];

fn ring_area_abs(pts: &[Vec2]) -> f32 {
    let n = pts.len();
    (0..n).map(|i| pts[i].perp_dot(pts[(i + 1) % n])).sum::<f32>().abs() * 0.5
}

fn hash(x: u64) -> u64 {
    crate::scene::hash(x)
}

fn h01(x: u64) -> f32 {
    (hash(x) >> 40) as f32 / (1u64 << 24) as f32
}

/// Street-lighting style of a neighbourhood (~600 m cells, global): councils
/// re-lamp area by area, so cool LED, warm LED and old sodium come in patches.
fn regional_colour(proj: &Proj, p: Vec2, motorway: bool) -> [f32; 3] {
    let (lat, lon) = proj.to_latlon(p);
    let cell = ((lat * 180.0).floor() as i64 as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ ((lon * 110.0).floor() as i64 as u64);
    let r = h01(cell ^ 0x51ee7);
    if motorway {
        if r < 0.6 { colour::LED_COOL } else { colour::SODIUM }
    } else if r < 0.42 {
        colour::LED_COOL
    } else if r < 0.75 {
        colour::LED_WARM
    } else {
        colour::SODIUM
    }
}

/// Builds the lights of one tile (called from the tile worker, `scene.rs`).
pub struct CityGen<'a> {
    proj: &'a Proj,
    lo: Vec2,
    hi: Vec2,
    out: TileLights,
    /// Built-up cells (n x n over the tile, row 0 = north): lit roads.
    built: Vec<bool>,
    built_n: usize,
}

impl<'a> CityGen<'a> {
    pub fn new(id: TileId, proj: &'a Proj, rect: (Vec2, Vec2)) -> Self {
        let mut out = TileLights::default();
        if id.is_far() {
            out.glow = vec![0.0; GLOW_N * GLOW_N];
        }
        CityGen { proj, lo: rect.0, hi: rect.1, out, built: Vec::new(), built_n: 0 }
    }

    pub fn finish(self) -> TileLights {
        self.out
    }

    /// Mark which parts of the tile are built up (street lights only there).
    pub fn set_built(&mut self, built: Vec<bool>, n: usize) {
        self.built = built;
        self.built_n = n;
    }

    fn is_built(&self, p: Vec2) -> bool {
        if self.built_n == 0 {
            return true;
        }
        let f = (p - self.lo) / (self.hi - self.lo).max(Vec2::ONE) * self.built_n as f32;
        let (x, y) = (f.x as i32, f.y as i32);
        let n = self.built_n as i32;
        (0..n).contains(&x) && (0..n).contains(&y) && self.built[(y * n + x) as usize]
    }

    fn inside(&self, p: Vec2) -> bool {
        p.x >= self.lo.x && p.x < self.hi.x && p.y >= self.lo.y && p.y < self.hi.y
    }

    /// Street lights (and, on major roads, traffic) along a road centreline.
    pub fn road(&mut self, pts: &[Vec2], width: f32, major: bool) {
        // Lamp spacing, mounting height, intensity seen from the side (cd),
        // illuminance under the lamp (lux), both sides?
        let (spacing, h, cd, lux, both) = if major && width > 15.0 {
            (40.0, 12.0, 900.0, 18.0, true)
        } else if major {
            (36.0, 10.0, 800.0, 15.0, true)
        } else if width > 7.0 {
            (33.0, 8.0, 650.0, 11.0, false)
        } else if width > 5.0 {
            (36.0, 6.0, 380.0, 8.0, false)
        } else {
            (45.0, 5.0, 220.0, 6.0, false)
        };
        let Some(first) = pts.first() else { return };
        let col = regional_colour(self.proj, *first, major && width > 15.0);
        let ground_cd = lux * h * h;
        // Phase from the position so overlapping pieces don't stack lamps.
        let mut next = spacing * (0.35 + 0.5 * h01((first.x * 7.1 + first.y * 3.3) as i64 as u64));
        let mut along = 0.0;
        let mut k = 0u32;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let len = a.distance(b);
            if len < 1e-3 {
                continue;
            }
            let d = (b - a) / len;
            let n = vec2(-d.y, d.x);
            while next <= along + len {
                let p = a + d * (next - along);
                if self.inside(p) && self.is_built(p) {
                    let side: &[f32] = if both { &[-1.0, 1.0] } else if k % 2 == 0 { &[1.0] } else { &[-1.0] };
                    for &s in side {
                        let q = p + n * s * (width * 0.5 + 0.6);
                        let jitter = 0.85 + 0.3 * h01(k as u64 ^ (q.x.to_bits() as u64) << 7);
                        self.out.lights.push(LightInst::new(vec3(q.x, h, q.y), cd * jitter, col, kind::STREET).with_ab(ground_cd * jitter, h));
                    }
                }
                next += spacing;
                k += 1;
            }
            along += len;
        }
        // Traffic on through roads.
        if width >= 8.0 {
            let busy = if major { 1.0 } else { 0.55 };
            for w in pts.windows(2) {
                let (a, b) = (w[0], w[1]);
                let len = a.distance(b);
                if len < 8.0 {
                    continue;
                }
                // Stretches of at most ~220 m (the shader animates <= 6 cars per direction).
                let pieces = (len / 220.0).ceil().max(1.0);
                for i in 0..pieces as usize {
                    let p0 = a.lerp(b, i as f32 / pieces);
                    let p1 = a.lerp(b, (i + 1) as f32 / pieces);
                    if !self.inside((p0 + p1) * 0.5) || !self.is_built((p0 + p1) * 0.5) {
                        continue;
                    }
                    let l = p0.distance(p1);
                    let seed = h01((p0.x.to_bits() as u64) << 21 ^ p1.y.to_bits() as u64);
                    let cars = (l / 38.0 * busy * (0.6 + 0.8 * seed)).clamp(0.0, 6.0);
                    let speed = if width > 15.0 { 24.0 } else if major { 13.0 } else { 10.0 } * (0.8 + 0.4 * seed);
                    self.out.traffic.push(TrafficSeg { a: p0.into(), b: p1.into(), width, seed, cars, speed });
                }
            }
        }
    }

    /// Taxiway centreline (green) and edge (blue) lights.
    pub fn taxiway(&mut self, pts: &[Vec2], width: f32) {
        let mut next = 15.0;
        let mut along = 0.0;
        let mut k = 0u32;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let len = a.distance(b);
            if len < 1e-3 {
                continue;
            }
            let d = (b - a) / len;
            let n = vec2(-d.y, d.x);
            while next <= along + len {
                let p = a + d * (next - along);
                if self.inside(p) {
                    self.out.lights.push(LightInst::new(vec3(p.x, 0.15, p.y), 70.0, colour::TAXI_GREEN, kind::TAXI));
                    if k % 2 == 0 {
                        for s in [-1.0, 1.0] {
                            let q = p + n * s * (width * 0.5 + 1.0);
                            self.out.lights.push(LightInst::new(vec3(q.x, 0.3, q.y), 40.0, colour::TAXI_BLUE, kind::TAXI));
                        }
                    }
                }
                next += 30.0;
                k += 1;
            }
            along += len;
        }
    }

    /// Floodlight masts around an apron.
    pub fn apron(&mut self, outer: &[Vec2]) {
        let n = outer.len();
        if n < 3 {
            return;
        }
        let c = outer.iter().copied().sum::<Vec2>() / n as f32;
        let mut carry = 60.0;
        for i in 0..n {
            let (a, b) = (outer[i], outer[(i + 1) % n]);
            let len = a.distance(b);
            let mut t = carry;
            while t < len {
                let p = a.lerp(b, t / len);
                let q = p + (c - p).normalize_or_zero() * 12.0;
                if self.inside(q) {
                    self.out.lights.push(LightInst::new(vec3(q.x, 25.0, q.y), 9000.0, colour::FLOOD, kind::FLOOD).with_ab(25.0 * 25.0 * 28.0, 25.0));
                }
                t += 140.0;
            }
            carry = t - len;
        }
    }

    /// Aircraft-warning lights on a tall building (and a strobe on the tallest).
    pub fn tall_building(&mut self, outer: &[Vec2], height: f32) {
        if height < 90.0 || outer.len() < 3 {
            return;
        }
        // Extreme corners of the footprint.
        let pick = |f: &dyn Fn(Vec2) -> f32| *outer.iter().max_by(|a, b| f(**a).total_cmp(&f(**b))).unwrap();
        let mut corners = vec![pick(&|p| p.x + p.y), pick(&|p| p.x - p.y), pick(&|p| -p.x + p.y), pick(&|p| -p.x - p.y)];
        corners.dedup_by(|a, b| a.distance(*b) < 4.0);
        for c in &corners {
            self.out.lights.push(LightInst::new(vec3(c.x, height + 0.8, c.y), 2000.0, colour::OBST_RED, kind::OBSTACLE));
            if height > 150.0 {
                self.out.lights.push(LightInst::new(vec3(c.x, height * 0.5, c.y), 2000.0, colour::OBST_RED, kind::OBSTACLE));
            }
        }
        if height > 200.0 {
            let m = outer.iter().copied().sum::<Vec2>() / outer.len() as f32;
            let ph = h01(m.x.to_bits() as u64);
            self.out.lights.push(LightInst::new(vec3(m.x, height + 3.0, m.y), 20000.0, [1.0, 1.0, 1.0], kind::STROBE).with_ab(ph, 0.0));
        }
    }

    // --- far (low-zoom) tiles

    fn cell_code(&self, p: Vec2) -> f32 {
        let t = TileId::at_world_z(self.proj, p, crate::osm::ZOOM);
        ((t.x & 127) | ((t.y & 127) << 7)) as f32
    }

    fn glow_add(&mut self, p: Vec2, lm: f32) {
        if self.out.glow.is_empty() {
            return;
        }
        let f = (p - self.lo) / (self.hi - self.lo).max(Vec2::ONE);
        let (x, y) = ((f.x * GLOW_N as f32) as i32, (f.y * GLOW_N as f32) as i32);
        if (0..GLOW_N as i32).contains(&x) && (0..GLOW_N as i32).contains(&y) {
            self.out.glow[y as usize * GLOW_N + x as usize] += lm;
        }
    }

    /// Built-up area of a far tile: aggregated city lights + skyglow flux.
    pub fn far_urban(&mut self, outer: &[Vec2], holes: &[Vec<Vec2>], class: u8, seed: u64) {
        if outer.len() < 3 {
            return;
        }
        let area = ring_area_abs(outer) - holes.iter().map(|h| ring_area_abs(h)).sum::<f32>();
        if area <= 0.0 {
            return;
        }
        let (mut blo, mut bhi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
        for p in outer {
            blo = blo.min(*p);
            bhi = bhi.max(*p);
        }
        // One aggregated sprite per ~5000 m² (≈ 8 street lamps), flux preserved.
        const PER: f32 = 5000.0;
        let want = ((area / PER) as usize).min(60_000);
        let fill = (area / ((bhi.x - blo.x) * (bhi.y - blo.y)).max(1.0)).clamp(0.02, 1.0);
        let tries = (want as f32 / fill * 1.2) as usize + 4;
        let mut s = hash(seed ^ 0xfa11);
        let mut rnd = || {
            s = hash(s);
            (s >> 40) as f32 / (1u64 << 24) as f32
        };
        let k = class.min(3) as usize;
        let lm = URBAN_FLUX[k] * PER;
        let mut placed = 0;
        for _ in 0..tries {
            if placed >= want {
                break;
            }
            let p = vec2(blo.x + rnd() * (bhi.x - blo.x), blo.y + rnd() * (bhi.y - blo.y));
            if !self.inside(p) || !crate::osm::point_in_ring(p, outer) || holes.iter().any(|h| crate::osm::point_in_ring(p, h)) {
                continue;
            }
            placed += 1;
            let r = rnd();
            let col = if r < 0.4 { colour::LED_COOL } else if r < 0.72 { colour::LED_WARM } else { colour::SODIUM };
            let cd = 650.0 * 8.0 * (0.5 + rnd());
            let code = self.cell_code(p);
            self.out.lights.push(LightInst::new(vec3(p.x, 8.0, p.y), cd, col, kind::FAR).with_ab(0.0, code));
            self.glow_add(p, lm);
        }
    }

    /// Lit major roads in far tiles (motorway lamp lines beyond the detailed area).
    pub fn far_road(&mut self, pts: &[Vec2], width: f32) {
        let spacing = 80.0;
        let mut next = spacing * 0.5;
        let mut along = 0.0;
        let col = regional_colour(self.proj, pts.first().copied().unwrap_or_default(), width > 15.0);
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let len = a.distance(b);
            if len < 1e-3 {
                continue;
            }
            let d = (b - a) / len;
            while next <= along + len {
                let p = a + d * (next - along);
                if self.inside(p) && self.is_built(p) {
                    let code = self.cell_code(p);
                    self.out.lights.push(LightInst::new(vec3(p.x, 11.0, p.y), 2.0 * 1400.0 * 2.0, col, kind::FAR).with_ab(0.0, code));
                    self.glow_add(p, 600.0 * 4.0);
                }
                next += spacing;
            }
            along += len;
        }
    }
}

// ---------------------------------------------------------------- airports

/// One runway end as the lighting sees it.
struct End {
    /// Physical end of the pavement (end lights) and landing threshold.
    end: Vec2,
    thr: Vec2,
    /// Landing direction (along the runway).
    dir: Vec2,
    /// Glidepath (rad) for the PAPI.
    glide: f32,
}

/// Airport lighting for all runways (world frame): edge, centreline,
/// threshold / end, touchdown zone, approach lights with sequenced flashers,
/// PAPI and the aerodrome beacons. `glide(rw, reverse)` gives the approach
/// angle (rad) for the PAPI of the end `reverse = false` (rw.a → rw.b).
pub fn airport_lights(runways: &[crate::scene::Runway], beacons: &[Vec2], glide: impl Fn(&crate::scene::Runway, bool) -> f32) -> Vec<LightInst> {
    let mut out = Vec::new();
    let y = 0.3;
    for rw in runways {
        let len = rw.length();
        // Lit runways only: as reported (OurAirports), or paved ones of a
        // useful length when the runway comes from the map tiles.
        let lit = if rw.source == crate::scene::RunwaySource::Tiles { rw.paved || rw.surface.is_empty() } else { rw.lighted };
        if len < 400.0 || !lit {
            continue;
        }
        let d = (rw.b - rw.a) / len;
        let n = vec2(-d.y, d.x); // right, looking from a to b
        let w = rw.width;
        let p3 = |p: Vec2, h: f32| vec3(p.x, h, p.y);
        let axis = vec3(d.x, 0.0, d.y);
        let big = len >= 1800.0;
        let edge_cd = if big { 10000.0 } else { 3000.0 };
        // Edge lights: white, yellow over the last 600 m (as seen landing).
        let spacing = if len > 1200.0 { 60.0 } else { 50.0 };
        let count = (len / spacing).round().max(2.0) as i32;
        for i in 0..=count {
            let t = len * i as f32 / count as f32;
            // Seen landing from a (viewer on the -d side): caution zone near b.
            let from_a = if len - t < 600.0 && len > 1200.0 { colour::RWY_YELLOW } else { colour::RWY_WHITE };
            let from_b = if t < 600.0 && len > 1200.0 { colour::RWY_YELLOW } else { colour::RWY_WHITE };
            for s in [-1.0, 1.0] {
                let p = rw.a + d * t + n * s * (w * 0.5 + 1.5);
                out.push(LightInst::new(p3(p, y), edge_cd, from_a, kind::BIDIR).with_dir(axis, 0).with_ab(col_param(from_b), 0.0));
            }
        }
        // Centreline: white, alternating red / white 900..300 m from the end, red last 300 m.
        if big {
            let cl = 15.0;
            let k = (len / cl) as i32;
            for i in 0..k {
                let t = 7.5 + i as f32 * cl;
                if t > len - 7.5 {
                    break;
                }
                let zone = |r: f32| {
                    if r < 300.0 {
                        colour::RWY_RED
                    } else if r < 900.0 && i % 2 == 1 {
                        colour::RWY_RED
                    } else {
                        colour::RWY_WHITE
                    }
                };
                let p = rw.a + d * t;
                out.push(LightInst::new(p3(p, 0.05), 5000.0, zone(len - t), kind::BIDIR).with_dir(axis, 0).with_ab(col_param(zone(t)), 0.0));
            }
        }
        let disp = |x: f32| x.clamp(0.0, len * 0.4);
        let ends = [
            End { end: rw.a, thr: rw.a + d * disp(rw.displaced_a), dir: d, glide: glide(rw, false) },
            End { end: rw.b, thr: rw.b - d * disp(rw.displaced_b), dir: -d, glide: glide(rw, true) },
        ];
        for e in &ends {
            let dir = e.dir;
            let right = vec2(-dir.y, dir.x);
            let toward = vec3(-dir.x, 0.0, -dir.y); // toward approaching aircraft
            // Threshold (green toward the approach) / end lights (red toward the runway):
            // a bidirectional light whose -axis side is the approach side.
            let ax = vec3(dir.x, 0.0, dir.y);
            let nthr = ((w / 3.0) as i32).max(4);
            let displaced = e.thr.distance(e.end) > 1.0;
            for j in 0..=nthr {
                let l = -w * 0.5 + w * j as f32 / nthr as f32;
                if displaced {
                    // Displaced threshold: green bar at the threshold, red end
                    // lights at the end of the pavement.
                    let p = e.thr + right * l;
                    out.push(LightInst::new(p3(p, y), 8000.0, colour::RWY_GREEN, kind::UNIDIR).with_dir(toward, 0));
                    let q = e.end + right * l - dir * 1.0;
                    out.push(LightInst::new(p3(q, y), 8000.0, colour::RWY_RED, kind::UNIDIR).with_dir(ax, 0));
                } else {
                    let p = e.thr + right * l - dir * 1.0;
                    out.push(LightInst::new(p3(p, y), 8000.0, colour::RWY_GREEN, kind::BIDIR).with_dir(ax, 0).with_ab(col_param(colour::RWY_RED), 0.0));
                }
            }
            // Wing bars beyond the edges.
            if big {
                for s in [-1.0f32, 1.0] {
                    for j in 1..=5 {
                        let p = e.thr + right * s * (w * 0.5 + 1.5 * j as f32 + 1.0) - dir * 1.0;
                        out.push(LightInst::new(p3(p, y), 8000.0, colour::RWY_GREEN, kind::UNIDIR).with_dir(toward, 0));
                    }
                }
            }
            // Touchdown zone barrettes (long runways).
            if len >= 2400.0 {
                let mut t = 60.0;
                while t <= 900.0 {
                    for s in [-1.0f32, 1.0] {
                        for j in 0..3 {
                            let p = e.thr + dir * t + right * s * (w * 0.2 + 1.5 * j as f32);
                            out.push(LightInst::new(p3(p, 0.05), 5000.0, colour::RWY_WHITE, kind::UNIDIR).with_dir(toward, 0));
                        }
                    }
                    t += 60.0;
                }
            }
            // Approach lighting: CAT I (900 m, crossbar at 300 m, sequenced
            // flashers) on long runways, a simple 420 m system otherwise.
            let als = if big { 900.0 } else if len >= 1000.0 { 420.0 } else { 0.0 };
            let mut s = 30.0;
            while s <= als {
                let c = e.thr - dir * s;
                let h = 1.0 + s * 0.004;
                for j in -2..=2 {
                    let p = c + right * (j as f32 * 1.05);
                    out.push(LightInst::new(p3(p, h), 15000.0, colour::RWY_WHITE, kind::UNIDIR).with_dir(toward, 0));
                }
                if (s - 300.0).abs() < 1.0 || (!big && (s - 150.0).abs() < 1.0) {
                    let half = if big { 15.0 } else { 9.0 };
                    let mut l = 4.0;
                    while l <= half {
                        for sg in [-1.0f32, 1.0] {
                            let p = c + right * sg * l;
                            out.push(LightInst::new(p3(p, h), 15000.0, colour::RWY_WHITE, kind::UNIDIR).with_dir(toward, 0));
                        }
                        l += 1.5;
                    }
                }
                if big && s >= 300.0 {
                    // Rabbit: flashes run in from the outer end twice a second.
                    let phase = ((s - 300.0) / (als - 300.0)).clamp(0.0, 1.0) * 0.85;
                    out.push(LightInst::new(p3(c, h + 0.6), 30000.0, [1.0, 1.0, 1.0], kind::FLASHER).with_dir(toward, 0).with_ab(phase, 0.0));
                }
                s += 30.0;
            }
            // PAPI on the left, abeam the aiming point: from the runway outward
            // the units are set at G+30', G+10', G-10', G-30' (on slope: two
            // white, two red).
            if len >= 800.0 {
                let at = if len > 1500.0 { 300.0 } else { (len * 0.17).max(150.0) };
                let g = e.glide;
                let arcmin = (1.0f32 / 60.0).to_radians();
                for (j, off) in [30.0, 10.0, -10.0, -30.0].iter().enumerate() {
                    let p = e.thr + dir * at - right * (w * 0.5 + 15.0 + 9.0 * j as f32);
                    out.push(LightInst::new(p3(p, 0.6), 15000.0, [1.0, 1.0, 1.0], kind::PAPI).with_dir(toward, 0).with_ab(g + off * arcmin, 0.0));
                }
            }
        }
    }
    for b in beacons {
        out.push(LightInst::new(vec3(b.x, 28.0, b.y), 40000.0, [1.0, 0.95, 0.85], kind::BEACON).with_ab(col_param(colour::RWY_GREEN), 0.0));
    }
    out
}

/// Where the aerodrome beacons stand: one per airport that has a runway
/// nearby, off the side of that runway's middle.
pub fn beacon_sites(runways: &[crate::scene::Runway], airports: &[Vec2]) -> Vec<Vec2> {
    let mut out: Vec<Vec2> = Vec::new();
    for ap in airports {
        let best = runways
            .iter()
            .filter(|r| r.length() > 600.0)
            .map(|r| ((r.a + r.b) * 0.5, r))
            .min_by(|a, b| a.0.distance(*ap).total_cmp(&b.0.distance(*ap)));
        let Some((mid, rw)) = best else { continue };
        if mid.distance(*ap) > 4000.0 {
            continue;
        }
        let d = (rw.b - rw.a).normalize_or_zero();
        let mut side = vec2(-d.y, d.x);
        if side.dot(*ap - mid) < 0.0 {
            side = -side;
        }
        let p = mid + side * (rw.width * 0.5 + 220.0);
        if !out.iter().any(|q| q.distance(p) < 1500.0) {
            out.push(p);
        }
    }
    out
}

// ---------------------------------------------------------------- aircraft

/// Where an aircraft model's lights are (body frame: +Z nose, +Y up, +X left).
/// Derived from the model (nav-light lenses, Strobe / Beacon parts, gear);
/// a model may override any of it.
#[derive(Clone, Debug, Default)]
pub struct AircraftLightRig {
    /// Steady navigation lights: position, colour.
    pub nav: Vec<(Vec3, [f32; 3])>,
    pub strobes: Vec<Vec3>,
    pub beacons: Vec<Vec3>,
    /// Landing lights: position, beam direction.
    pub landing: Vec<(Vec3, Vec3)>,
    /// Taxi light: position, beam direction.
    pub taxi: Option<(Vec3, Vec3)>,
}

impl AircraftLightRig {
    /// Find the lights on a model: EMISSIVE body lenses (red = left tip,
    /// green = right tip, white = tail), Strobe / Beacon parts; landing lights
    /// in the wing roots (or the nose of a light aircraft), the taxi light on
    /// the nose gear.
    pub fn from_model(m: &crate::models::AircraftModel) -> Self {
        use crate::models::PartKind;
        let emissive = |v: &crate::scene::Vert| (v.mat - crate::scene::mat::EMISSIVE).abs() < 0.5 && v.uv[1] < 0.5;
        let mut acc: [(Vec3, f32); 3] = [(Vec3::ZERO, 0.0); 3];
        let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
        for v in &m.body.verts {
            let p = Vec3::from(v.pos);
            lo = lo.min(p);
            hi = hi.max(p);
            if !emissive(v) {
                continue;
            }
            let [r, g, b] = v.col;
            let k = if r > 2.0 * g && r > 2.0 * b {
                0
            } else if g > 1.5 * r && g >= b * 0.8 {
                1
            } else if r > 0.5 && g > 0.5 && b > 0.5 {
                2
            } else {
                continue;
            };
            acc[k].0 += p;
            acc[k].1 += 1.0;
        }
        let mut rig = AircraftLightRig::default();
        let cols = [[1.0, 0.04, 0.02], [0.05, 1.0, 0.3], [1.0, 1.0, 1.0]];
        for (k, (s, n)) in acc.iter().enumerate() {
            if *n > 0.0 {
                rig.nav.push((*s / *n, cols[k]));
            }
        }
        if rig.nav.len() < 2 {
            // No lenses found: put them on the bounding box.
            rig.nav = vec![(vec3(hi.x, 0.0, 0.0), cols[0]), (vec3(lo.x, 0.0, 0.0), cols[1]), (vec3(0.0, 0.3, lo.z), cols[2])];
        }
        let centre = |p: &crate::models::Part| {
            let n = p.mesh.verts.len().max(1) as f32;
            p.pivot + p.mesh.verts.iter().map(|v| Vec3::from(v.pos)).sum::<Vec3>() / n
        };
        for p in &m.parts {
            match p.kind {
                PartKind::Strobe => rig.strobes.push(centre(p)),
                PartKind::Beacon => rig.beacons.push(centre(p)),
                _ => {}
            }
        }
        let span = hi.x - lo.x;
        let tip = rig.nav.iter().find(|(_, c)| c[0] > 0.9 && c[1] < 0.5).map(|(p, _)| *p).unwrap_or(vec3(hi.x, 0.0, 0.0));
        // Beams point ahead and a little down (body frame).
        let fwd_down = vec3(0.0, -0.07, 1.0).normalize();
        let nose_gear = m.parts.iter().find(|p| p.kind == PartKind::GearNose).map(|p| p.pivot);
        if let Some(ng) = nose_gear.filter(|_| span > 12.0) {
            // Airliner / fighter: lights in the wing roots, taxi light on the nose gear.
            let x = (span * 0.14).max(1.2);
            let z = tip.z + (hi.z - tip.z) * 0.18;
            rig.landing = vec![(vec3(x, tip.y.min(0.0) - 0.2, z), fwd_down), (vec3(-x, tip.y.min(0.0) - 0.2, z), fwd_down)];
            rig.taxi = Some((ng + vec3(0.0, -0.3, 0.4), vec3(0.0, -0.13, 1.0).normalize()));
        } else {
            // Light aircraft: one landing light in the nose / left wing leading edge.
            let z = hi.z - 0.4;
            rig.landing = vec![(vec3(0.0, 0.0, z), fwd_down)];
            rig.taxi = nose_gear.map(|ng| (ng + vec3(0.0, 0.2, 0.3), vec3(0.0, -0.13, 1.0).normalize()));
        }
        rig
    }
}

/// Max sprites / analytic lights of the aircraft per frame.
pub const AC_SPRITES: usize = 16;
pub const AC_SPOTS: usize = 4;

/// One analytic light in the scene shader (eye-relative): spot (cone) or
/// point (`cos_outer` = -1).
#[derive(Clone, Copy, Debug, Default)]
pub struct SpotLight {
    pub pos: Vec3,
    pub dir: Vec3,
    /// Colour * intensity (cd).
    pub cd: Vec3,
    pub cos_inner: f32,
    pub cos_outer: f32,
    /// Distance beyond which it is ignored (m).
    pub range: f32,
}

/// The aircraft's lights for one frame (eye-relative). `Copy`, so it can
/// travel in `gfx::FrameView`.
#[derive(Clone, Copy, Debug)]
pub struct AircraftLights {
    pub sprites: [LightInst; AC_SPRITES],
    pub n_sprites: usize,
    pub spots: [SpotLight; AC_SPOTS],
    pub n_spots: usize,
}

impl Default for AircraftLights {
    fn default() -> Self {
        AircraftLights { sprites: [LightInst::default(); AC_SPRITES], n_sprites: 0, spots: [SpotLight::default(); AC_SPOTS], n_spots: 0 }
    }
}

/// Flash timing shared with the aircraft mesh (sim.rs): beacon ~1 Hz, strobes
/// double-flash every 1.2 s.
pub fn beacon_on(time: f32) -> bool {
    time.fract() < 0.12
}
pub fn strobe_on(time: f32) -> bool {
    let st = (time / 1.2).fract();
    st < 0.05 || (0.12..0.17).contains(&st)
}

impl AircraftLights {
    fn sprite(&mut self, l: LightInst) {
        if self.n_sprites < AC_SPRITES {
            self.sprites[self.n_sprites] = l;
            self.n_sprites += 1;
        }
    }
    fn spot(&mut self, s: SpotLight) {
        if self.n_spots < AC_SPOTS {
            self.spots[self.n_spots] = s;
            self.n_spots += 1;
        }
    }

    /// The lights of an aircraft at `pos` (eye-relative) / `rot` at sim time
    /// `time`. `landing` / `taxi`: switches.
    pub fn build(rig: &AircraftLightRig, pos: Vec3, rot: Quat, time: f32, landing: bool, taxi: bool) -> Self {
        let mut a = AircraftLights::default();
        let at = |p: Vec3| pos + rot * p;
        for (p, c) in &rig.nav {
            a.sprite(LightInst::new(at(*p), 60.0, *c, kind::OMNI));
        }
        if strobe_on(time) {
            for p in &rig.strobes {
                a.sprite(LightInst::new(at(*p), 2500.0, [1.0, 1.0, 1.0], kind::OMNI));
            }
            if let Some(p) = rig.strobes.first() {
                a.spot(SpotLight { pos: at(*p), dir: Vec3::Y, cd: Vec3::splat(2500.0 * rig.strobes.len() as f32), cos_inner: -1.0, cos_outer: -1.0, range: 120.0 });
            }
        }
        if beacon_on(time) {
            for p in &rig.beacons {
                a.sprite(LightInst::new(at(*p), 900.0, [1.0, 0.05, 0.02], kind::OMNI));
            }
            if let Some(p) = rig.beacons.first() {
                a.spot(SpotLight { pos: at(*p), dir: Vec3::Y, cd: vec3(900.0, 45.0, 18.0), cos_inner: -1.0, cos_outer: -1.0, range: 60.0 });
            }
        }
        if landing && !rig.landing.is_empty() {
            let n = rig.landing.len() as f32;
            let c = rig.landing.iter().map(|l| l.0).sum::<Vec3>() / n;
            let d = (rot * rig.landing[0].1).normalize();
            let cd = 160_000.0;
            for (p, dir) in &rig.landing {
                a.sprite(LightInst::new(at(*p), cd, [1.0, 0.94, 0.85], kind::SPOT).with_dir(rot * *dir, 0).with_ab(10f32.to_radians().cos(), 0.0));
            }
            a.spot(SpotLight { pos: at(c), dir: d, cd: vec3(1.0, 0.94, 0.85) * cd * n, cos_inner: 5f32.to_radians().cos(), cos_outer: 11f32.to_radians().cos(), range: 1500.0 });
        }
        if taxi {
            if let Some((p, dir)) = rig.taxi {
                let cd = 40_000.0;
                a.sprite(LightInst::new(at(p), cd, [1.0, 0.95, 0.88], kind::SPOT).with_dir(rot * dir, 0).with_ab(18f32.to_radians().cos(), 0.0));
                a.spot(SpotLight { pos: at(p), dir: (rot * dir).normalize(), cd: vec3(1.0, 0.95, 0.88) * cd, cos_inner: 10f32.to_radians().cos(), cos_outer: 24f32.to_radians().cos(), range: 500.0 });
            }
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::Runway;

    #[test]
    fn runway_lights_layout() {
        let rw = Runway { a: vec2(0.0, 0.0), b: vec2(3000.0, 0.0), width: 45.0, ..Default::default() };
        let l = airport_lights(&[rw], &[], |_, _| 3f32.to_radians());
        let count = |k: u32| l.iter().filter(|x| x.kind() == k).count();
        assert!(count(kind::BIDIR) > 300, "edge + centreline");
        assert_eq!(count(kind::PAPI), 8, "4 units per end");
        assert!(count(kind::FLASHER) >= 2 * 20, "sequenced flashers");
        // PAPI on the left of the landing direction (a → b = +x: left = -z... right = (0, 1)).
        let papi_a: Vec<_> = l.iter().filter(|x| x.kind() == kind::PAPI && x.pos[0] < 1500.0).collect();
        assert!(papi_a.iter().all(|x| x.pos[2] < -20.0), "{:?}", papi_a.iter().map(|x| x.pos).collect::<Vec<_>>());
        // Settings straddle the glidepath.
        let mut ang: Vec<f32> = papi_a.iter().map(|x| x.a.to_degrees()).collect();
        ang.sort_by(f32::total_cmp);
        assert!((ang[0] - 2.5).abs() < 0.01 && (ang[3] - 3.5).abs() < 0.01, "{ang:?}");
    }

    #[test]
    fn packing() {
        let c = pack_col_test([1.0, 0.5, 0.0], kind::PAPI);
        assert_eq!(c >> 24, kind::PAPI);
        assert_eq!(c & 0xff, 255);
        let p = col_param(colour::RWY_RED);
        assert!(p.is_finite());
    }

    fn pack_col_test(c: [f32; 3], k: u32) -> u32 {
        super::super::pack_col(c, k)
    }
}
