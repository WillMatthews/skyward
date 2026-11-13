//! Lane graph of one map tile, built in the tile workers from the
//! OpenMapTiles `transportation` layer.
//!
//! * Roads are split where they meet (shared vertices, and road ends that
//!   touch another road between its vertices), giving *pieces* between
//!   *nodes*.
//! * Each piece gets lanes per direction (one-way or two-way, lane count by
//!   class and width, laid out to match the painted lane markings of the
//!   road shader), offset to the driving side of the region, trimmed back
//!   from junctions and draped on the terrain.
//! * Junctions are crossed on *connectors*: short Bézier lanes from an
//!   incoming lane to an outgoing one, with a turn weight, a speed for the
//!   curve, a priority (major roads and straight-on first) and the list of
//!   connectors whose paths cross or merge (the simulation lets only one of
//!   a conflicting set through at a time).
//! * Ends on the tile boundary are *portals*: the simulation links them to
//!   the neighbouring tile's lanes at run time, so traffic flows across.
//! * Tunnels are kept (vehicles drive through them hidden).
//!
//! Everything is in the tile's own frame (`Proj::tile_frame`).

use crate::mvt::Feature;
use crate::osm::{Proj, TileId};
use glam::{Vec2, vec2};
use std::collections::HashMap;

/// Road classes (lane `class`); lower is more important.
pub mod class {
    pub const MOTORWAY: u8 = 0;
    pub const TRUNK: u8 = 1;
    pub const PRIMARY: u8 = 2;
    pub const SECONDARY: u8 = 3;
    pub const TERTIARY: u8 = 4;
    pub const MINOR: u8 = 5;
    pub const SERVICE: u8 = 6;
    pub const BUSWAY: u8 = 7;
    /// Junction connector (its own class for statistics).
    pub const LINK: u8 = 8;
}

/// [`RoadLine::flags`] and [`Lane`] flags.
pub mod flag {
    pub const ONEWAY: u8 = 1;
    pub const RAMP: u8 = 2;
    pub const TUNNEL: u8 = 4;
    pub const BRIDGE: u8 = 8;
    /// Driveways, private roads: little traffic.
    pub const LOWUSE: u8 = 16;
    /// Lane flags only: a junction connector, starts / ends on the tile edge.
    pub const CONNECTOR: u8 = 32;
    pub const EDGE_IN: u8 = 64;
    pub const EDGE_OUT: u8 = 128;
}

/// A drivable road as extracted from the tile (tile frame, clipped).
#[derive(Clone, Debug)]
pub struct RoadLine {
    pub class: u8,
    pub flags: u8,
    pub layer: i8,
    pub pts: Vec<Vec2>,
    /// The start / end lies on the tile boundary (clip edge).
    pub cut: [bool; 2],
}

/// Road width (m) the scenery draws for a class (osm.rs `line_kind`): lanes
/// are laid out inside it, on its painted lane lines.
pub fn width_of(class: u8) -> f32 {
    match class {
        class::MOTORWAY => 18.0,
        class::TRUNK => 14.0,
        class::PRIMARY => 11.0,
        class::SECONDARY => 9.0,
        class::TERTIARY => 8.0,
        class::MINOR => 6.0,
        class::BUSWAY => 7.0,
        _ => 4.0,
    }
}

/// Collect a `transportation` feature that cars drive on (osm.rs calls this
/// for every line feature of the layer). `clip` clips a part to the tile
/// and maps it to the tile frame.
pub fn extract_road(out: &mut Vec<RoadLine>, f: &Feature, class: &str, clip: &dyn Fn(&[[f32; 2]]) -> Vec<(Vec<Vec2>, [bool; 2])>) {
    let c = match class {
        "motorway" => class::MOTORWAY,
        "trunk" => class::TRUNK,
        "primary" => class::PRIMARY,
        "secondary" => class::SECONDARY,
        "tertiary" => class::TERTIARY,
        "minor" => class::MINOR,
        "service" => class::SERVICE,
        "busway" => class::BUSWAY,
        _ => return,
    };
    let service = f.str("service").unwrap_or("");
    if matches!(service, "parking_aisle" | "drive-through" | "emergency_access") {
        return;
    }
    if matches!(f.str("access"), Some("no")) && c >= class::MINOR {
        return;
    }
    let mut flags = 0u8;
    let oneway = f.num("oneway").unwrap_or(0.0) as i32;
    if oneway != 0 {
        flags |= flag::ONEWAY;
    }
    if f.num("ramp").unwrap_or(0.0) > 0.5 {
        flags |= flag::RAMP;
    }
    match f.str("brunnel") {
        Some("tunnel") => flags |= flag::TUNNEL,
        Some("bridge") => flags |= flag::BRIDGE,
        _ => {}
    }
    if matches!(service, "driveway" | "alley") || matches!(f.str("access"), Some("private" | "destination" | "customers")) {
        flags |= flag::LOWUSE;
    }
    let layer = f.num("layer").unwrap_or(0.0).clamp(-8.0, 8.0) as i8;
    for part in &f.parts {
        if part.len() < 2 {
            continue;
        }
        let mut p = part.clone();
        // One-way against the drawing direction: store it in travel order.
        if oneway < 0 {
            p.reverse();
        }
        for (pts, cut) in clip(&p) {
            if pts.len() >= 2 {
                out.push(RoadLine { class: c, flags, layer, pts, cut });
            }
        }
    }
}

/// One lane point (tile frame) and the distance along the lane to it.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LanePt {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub s: f32,
}

/// A lane or a junction connector.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Lane {
    /// Points `[p0, p0 + np)` of `RoadNet::pts`.
    pub p0: u32,
    pub np: u32,
    pub len: f32,
    /// Speed limit (m/s); on connectors, the speed for the curve.
    pub speed: f32,
    /// Successors `[succ0, succ0 + nsucc)` of `RoadNet::succ`.
    pub succ0: u32,
    pub nsucc: u32,
    /// Predecessors `[pred0, pred0 + npred)` of `RoadNet::pred`.
    pub pred0: u32,
    pub npred: u32,
    /// Connectors: conflicting connectors `[conf0, conf0 + nconf)` of `RoadNet::conf`.
    pub conf0: u32,
    pub nconf: u32,
    /// Bounding box (x, z).
    pub lo: [f32; 2],
    pub hi: [f32; 2],
    /// class | flags << 8 | lane index from the kerb << 16 | lanes this way << 24.
    pub meta: u32,
    /// Vehicles per metre at the busiest hour (before the density setting).
    pub density: f32,
    /// Connectors: priority at the junction (lower goes first).
    pub prio: f32,
    /// Connectors: index of the junction node (conflict bookkeeping).
    pub node: u32,
}

impl Lane {
    pub fn class(&self) -> u8 {
        self.meta as u8
    }
    pub fn flags(&self) -> u8 {
        (self.meta >> 8) as u8
    }
    pub fn has(&self, f: u8) -> bool {
        self.flags() & f != 0
    }
    pub fn kerb_index(&self) -> u8 {
        (self.meta >> 16) as u8
    }
    pub fn lanes(&self) -> u8 {
        (self.meta >> 24) as u8
    }
}

/// A successor of a lane, with its share of the traffic (relative weight).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Succ {
    pub lane: u32,
    pub weight: f32,
}

/// Region-dependent rules of the road (from the tile's position).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Region {
    /// Traffic keeps left.
    pub left: bool,
    /// 0 UK / Ireland, 1 continental Europe & rest, 2 North America.
    pub rules: u8,
    /// Greater London: red double-deck buses, black cabs.
    pub london: bool,
}

impl Region {
    pub fn at(lat: f64, lon: f64) -> Region {
        let inb = |la0: f64, la1: f64, lo0: f64, lo1: f64| lat >= la0 && lat <= la1 && lon >= lo0 && lon <= lo1;
        let uk = inb(49.8, 61.0, -11.0, 1.9) && !(inb(49.8, 51.2, 1.4, 1.9));
        // Left-hand traffic: UK & Ireland, Japan, Australia, NZ, South Asia,
        // southern Africa, parts of South-East Asia (coarse boxes).
        let left = uk
            || inb(30.0, 46.0, 129.0, 146.0)
            || inb(-45.0, -10.0, 112.0, 154.0)
            || inb(-48.0, -34.0, 166.0, 179.0)
            || inb(5.0, 36.0, 68.0, 92.0)
            || inb(-35.0, -15.0, 15.0, 36.0)
            || inb(-11.0, 7.0, 95.0, 141.0) && !inb(-11.0, 7.0, 116.0, 127.0)
            || inb(1.0, 7.0, 99.0, 104.5)
            || inb(22.1, 22.6, 113.8, 114.5);
        let rules = if uk {
            0
        } else if inb(15.0, 72.0, -170.0, -50.0) {
            2
        } else {
            1
        };
        Region { left, rules, london: inb(51.28, 51.70, -0.52, 0.34) }
    }
}

/// Speed limit (m/s) by class, region rules and whether it is built up.
pub fn speed_limit(c: u8, rules: u8, urban: bool, ramp: bool) -> f32 {
    const MPH: f32 = 0.44704;
    const KMH: f32 = 1.0 / 3.6;
    let v = match rules {
        0 => match (c, urban) {
            (class::MOTORWAY, _) => 70.0 * MPH,
            (class::TRUNK, false) => 60.0 * MPH,
            (class::TRUNK, true) => 40.0 * MPH,
            (class::PRIMARY | class::SECONDARY, false) => 50.0 * MPH,
            (class::PRIMARY | class::SECONDARY, true) => 30.0 * MPH,
            (class::TERTIARY, false) => 40.0 * MPH,
            (class::TERTIARY, true) => 25.0 * MPH,
            (class::MINOR, false) => 30.0 * MPH,
            (class::MINOR, true) => 20.0 * MPH,
            _ => 12.0 * MPH,
        },
        2 => match (c, urban) {
            (class::MOTORWAY, _) => 65.0 * MPH,
            (class::TRUNK, false) => 55.0 * MPH,
            (class::TRUNK, true) => 45.0 * MPH,
            (class::PRIMARY | class::SECONDARY, false) => 50.0 * MPH,
            (class::PRIMARY | class::SECONDARY, true) => 35.0 * MPH,
            (class::TERTIARY, _) => 30.0 * MPH,
            (class::MINOR, _) => 25.0 * MPH,
            _ => 12.0 * MPH,
        },
        _ => match (c, urban) {
            (class::MOTORWAY, _) => 120.0 * KMH,
            (class::TRUNK, false) => 100.0 * KMH,
            (class::TRUNK, true) => 70.0 * KMH,
            (class::PRIMARY | class::SECONDARY, false) => 80.0 * KMH,
            (class::PRIMARY | class::SECONDARY | class::TERTIARY, true) => 50.0 * KMH,
            (class::TERTIARY, false) => 70.0 * KMH,
            (class::MINOR, false) => 50.0 * KMH,
            (class::MINOR, true) => 30.0 * KMH,
            _ => 20.0 * KMH,
        },
    };
    // Slip roads: a little slower than the road they serve.
    if ramp && c <= class::PRIMARY { v * 0.8 } else { v }
}

/// Vehicles per metre per lane at the busiest hour, by class and built-up area.
pub fn base_density(c: u8, urban: bool, lowuse: bool) -> f32 {
    let k = match c {
        class::MOTORWAY => 0.030,
        class::TRUNK => 0.028,
        class::PRIMARY => 0.028,
        class::SECONDARY => 0.020,
        class::TERTIARY => 0.013,
        class::MINOR => 0.0045,
        class::SERVICE => 0.0018,
        class::BUSWAY => 0.002,
        _ => 0.0,
    };
    let rural = match c {
        class::MOTORWAY | class::TRUNK => 0.8,
        class::PRIMARY | class::SECONDARY => 0.45,
        _ => 0.3,
    };
    k * if urban { 1.0 } else { rural } * if lowuse { 0.25 } else { 1.0 }
}

/// Lane layout of one direction of a road: lateral offsets (to the driving
/// side, from the centreline) of its lanes, kerb lane first. Matches the
/// lane markings of the road shader (scene.wgsl, 3.4 m lanes on roads 12 m
/// and wider, a centre line from 6.5 m).
pub fn lane_offsets(c: u8, oneway: bool, ramp: bool) -> Vec<f32> {
    let w = width_of(c);
    let hw = w * 0.5;
    const LW: f32 = 3.4;
    if oneway {
        if ramp {
            // Slip roads: one lane (just off the centre line).
            return vec![LW * 0.5];
        }
        if w >= 12.0 {
            let n = 2 * (hw / LW).floor() as usize;
            // Centred, kerb (driving side) first.
            return (0..n).map(|k| (n as f32 * 0.5 - k as f32 - 0.5) * LW).collect();
        }
        if w >= 6.5 {
            return vec![hw * 0.5, -hw * 0.5];
        }
        return vec![0.0];
    }
    if w >= 12.0 {
        let n = (hw / LW).floor() as usize;
        return (0..n).map(|k| (n as f32 - k as f32 - 0.5) * LW).collect();
    }
    vec![(hw * 0.5).max(1.0)]
}

/// The lane graph of one tile.
#[derive(Clone, Debug)]
pub struct RoadNet {
    pub id: TileId,
    pub lanes: Vec<Lane>,
    pub pts: Vec<LanePt>,
    pub succ: Vec<Succ>,
    pub pred: Vec<u32>,
    pub conf: Vec<u32>,
    /// Lanes that start / end on the tile edge (portals to the neighbours).
    pub edge_in: Vec<u32>,
    pub edge_out: Vec<u32>,
    pub region: Region,
    /// Built-up share of the tile 0..1.
    pub urban: f32,
}

impl RoadNet {
    pub fn lane_pts(&self, l: u32) -> &[LanePt] {
        let ln = &self.lanes[l as usize];
        &self.pts[ln.p0 as usize..(ln.p0 + ln.np) as usize]
    }
    pub fn succs(&self, l: u32) -> &[Succ] {
        let ln = &self.lanes[l as usize];
        &self.succ[ln.succ0 as usize..(ln.succ0 + ln.nsucc) as usize]
    }
    pub fn preds(&self, l: u32) -> &[u32] {
        let ln = &self.lanes[l as usize];
        &self.pred[ln.pred0 as usize..(ln.pred0 + ln.npred) as usize]
    }
    pub fn conflicts(&self, l: u32) -> &[u32] {
        let ln = &self.lanes[l as usize];
        &self.conf[ln.conf0 as usize..(ln.conf0 + ln.nconf) as usize]
    }
    /// Position (x, y, z) and unit direction (x, z) at distance `s` along a
    /// lane; `hint` is a point index to start the search from (updated).
    pub fn eval(&self, l: u32, s: f32, hint: &mut u32) -> ([f32; 3], Vec2, f32) {
        let p = self.lane_pts(l);
        let n = p.len();
        if n < 2 {
            let q = p.first().copied().unwrap_or_default();
            return ([q.x, q.y, q.z], Vec2::X, 0.0);
        }
        let h = (*hint as usize).min(n - 2);
        let i = if p[h].s <= s && (s < p[h + 1].s || h == n - 2) { h } else { p.partition_point(|q| q.s <= s).clamp(1, n - 1) - 1 };
        *hint = i as u32;
        let (a, b) = (p[i], p[i + 1]);
        let seg = (b.s - a.s).max(1e-4);
        let t = ((s - a.s) / seg).clamp(0.0, 1.0);
        let d = vec2(b.x - a.x, b.z - a.z) / seg;
        let grade = (b.y - a.y) / seg;
        ([a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t], d.normalize_or(Vec2::X), grade)
    }
    /// Start / end position and direction of a lane (x, z).
    pub fn start(&self, l: u32) -> (Vec2, Vec2) {
        let p = self.lane_pts(l);
        let (a, b) = (p[0], p[1.min(p.len() - 1)]);
        (vec2(a.x, a.z), (vec2(b.x, b.z) - vec2(a.x, a.z)).normalize_or(Vec2::X))
    }
    pub fn end(&self, l: u32) -> (Vec2, Vec2) {
        let p = self.lane_pts(l);
        let n = p.len();
        let (a, b) = (p[n.saturating_sub(2)], p[n - 1]);
        (vec2(b.x, b.z), (vec2(b.x, b.z) - vec2(a.x, a.z)).normalize_or(Vec2::X))
    }
    pub fn bytes(&self) -> usize {
        self.lanes.len() * std::mem::size_of::<Lane>() + self.pts.len() * 16 + self.succ.len() * 8 + (self.pred.len() + self.conf.len()) * 4
    }
}

// ---------------------------------------------------------------- build

/// Inputs of a tile's lane graph besides the roads.
pub struct Ctx<'a> {
    pub id: TileId,
    pub proj: &'a Proj,
    /// Ground height (tile frame).
    pub height: &'a dyn Fn(Vec2) -> f32,
    /// Built-up land at a point (street lights, urban speed limits).
    pub built: &'a dyn Fn(Vec2) -> bool,
}

/// Split road between two nodes.
struct Piece {
    class: u8,
    flags: u8,
    pts: Vec<Vec2>,
    /// Node at the start / end.
    n: [usize; 2],
}

struct Node {
    p: Vec2,
    edge: bool,
    /// (piece, end 0 = start / 1 = end).
    ends: Vec<(usize, usize)>,
}

/// A directed way along a piece: its lanes (kerb first) in travel order.
#[derive(Clone)]
struct Way {
    piece: usize,
    fwd: bool,
    lanes: Vec<u32>,
}

fn key(p: Vec2) -> (i32, i32) {
    ((p.x * 4.0).round() as i32, (p.y * 4.0).round() as i32)
}

fn polyline_len(p: &[Vec2]) -> f32 {
    p.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// Cut a polyline to the arc length range [a, b].
fn sub_polyline(p: &[Vec2], a: f32, b: f32) -> Vec<Vec2> {
    let mut out = Vec::new();
    let mut s = 0.0;
    for w in p.windows(2) {
        let l = w[0].distance(w[1]);
        let (s0, s1) = (s, s + l);
        if s1 >= a && s0 <= b && l > 1e-6 {
            if out.is_empty() {
                out.push(w[0].lerp(w[1], ((a - s0) / l).clamp(0.0, 1.0)));
            }
            out.push(w[0].lerp(w[1], ((b - s0) / l).clamp(0.0, 1.0)));
        }
        s = s1;
    }
    out.dedup_by(|x, y| x.distance(*y) < 1e-3);
    if out.len() == 1 {
        out.push(out[0]);
    }
    out
}

/// Offset a polyline sideways by `o` (positive = left of travel when
/// `left_hand`, else right) with mitred joins.
fn offset_polyline(p: &[Vec2], o: f32, left_hand: bool) -> Vec<Vec2> {
    let n = p.len();
    let side = |d: Vec2| if left_hand { vec2(d.y, -d.x) } else { vec2(-d.y, d.x) };
    (0..n)
        .map(|i| {
            let d0 = if i > 0 { (p[i] - p[i - 1]).normalize_or_zero() } else { Vec2::ZERO };
            let d1 = if i + 1 < n { (p[i + 1] - p[i]).normalize_or_zero() } else { Vec2::ZERO };
            let d = (d0 + d1).normalize_or(if d1 != Vec2::ZERO { d1 } else { d0 });
            let nrm = side(d);
            // Mitre: keep the lane width through bends (limited at sharp ones).
            let c = if d0 != Vec2::ZERO && d1 != Vec2::ZERO { side(d0).dot(nrm).max(0.5) } else { 1.0 };
            p[i] + nrm * (o / c)
        })
        .collect()
}

/// Insert points so no segment is longer than `step` (heights follow the terrain).
fn densify(p: &[Vec2], step: f32) -> Vec<Vec2> {
    let mut out = Vec::with_capacity(p.len() * 2);
    for w in p.windows(2) {
        let l = w[0].distance(w[1]);
        let k = (l / step).ceil().max(1.0) as usize;
        for j in 0..k {
            out.push(w[0].lerp(w[1], j as f32 / k as f32));
        }
    }
    if let Some(l) = p.last() {
        out.push(*l);
    }
    out
}

fn cubic(p0: Vec2, c1: Vec2, c2: Vec2, p1: Vec2, t: f32) -> Vec2 {
    let u = 1.0 - t;
    p0 * (u * u * u) + c1 * (3.0 * u * u * t) + c2 * (3.0 * u * t * t) + p1 * (t * t * t)
}

fn seg_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let o = |p: Vec2, q: Vec2, r: Vec2| (q - p).perp_dot(r - p);
    let (d1, d2, d3, d4) = (o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

fn paths_cross(a: &[Vec2], b: &[Vec2]) -> bool {
    a.windows(2).any(|s| b.windows(2).any(|t| seg_cross(s[0], s[1], t[0], t[1])))
}

/// Builder state: lanes as (points, attributes) before packing.
struct Pending {
    pts: Vec<Vec2>,
    class: u8,
    flags: u8,
    kerb: u8,
    nlanes: u8,
    speed: f32,
    density: f32,
    succ: Vec<Succ>,
    prio: f32,
    node: u32,
    conf: Vec<u32>,
    /// Connectors: the approach (way) they come from.
    way: u32,
}

/// Build the lane graph of a tile (None when it has no roads).
pub fn build(ctx: &Ctx, roads: &[RoadLine]) -> Option<RoadNet> {
    if roads.is_empty() {
        return None;
    }
    let (lo, hi) = ctx.id.rect(ctx.proj);
    let centre = ctx.proj.to_latlon((lo + hi) * 0.5);
    let region = Region::at(centre.0, centre.1);
    let left = region.left;

    // --- 1. Split the roads where they meet.
    // Vertex incidences: (line, vertex index, is an end, layer).
    let mut inc: HashMap<(i32, i32), Vec<(usize, usize, bool, i8)>> = HashMap::new();
    for (li, r) in roads.iter().enumerate() {
        let n = r.pts.len();
        for (vi, p) in r.pts.iter().enumerate() {
            let end = (vi == 0 && !r.cut[0]) || (vi + 1 == n && !r.cut[1]);
            inc.entry(key(*p)).or_default().push((li, vi, end, r.layer));
        }
    }
    // Split points per line: (vertex index, inserted point after it if any).
    let mut splits: Vec<Vec<(f32, Vec2)>> = vec![Vec::new(); roads.len()];
    for v in inc.values() {
        if v.len() < 2 {
            continue;
        }
        let any_end = v.iter().any(|x| x.2);
        for &(li, vi, end, layer) in v {
            if end || vi == 0 || vi + 1 == roads[li].pts.len() {
                continue;
            }
            let other_same_layer = v.iter().any(|x| x.0 != li && x.3 == layer);
            if any_end || other_same_layer {
                splits[li].push((vi as f32, roads[li].pts[vi]));
            }
        }
    }
    // Road ends touching another road between its vertices (simplified
    // geometry): split that road at the closest point.
    {
        const CELL: f32 = 40.0;
        let mut grid: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
        for (li, r) in roads.iter().enumerate() {
            for (si, w) in r.pts.windows(2).enumerate() {
                let (a, b) = (w[0].min(w[1]) - Vec2::splat(2.0), w[0].max(w[1]) + Vec2::splat(2.0));
                for gy in (a.y / CELL).floor() as i32..=(b.y / CELL).floor() as i32 {
                    for gx in (a.x / CELL).floor() as i32..=(b.x / CELL).floor() as i32 {
                        grid.entry((gx, gy)).or_default().push((li, si));
                    }
                }
            }
        }
        for (li, r) in roads.iter().enumerate() {
            for (ei, &p) in [r.pts[0], *r.pts.last().unwrap()].iter().enumerate() {
                if r.cut[ei] || inc.get(&key(p)).is_some_and(|v| v.len() > 1) {
                    continue;
                }
                let g = ((p.x / CELL).floor() as i32, (p.y / CELL).floor() as i32);
                let mut best: Option<(f32, usize, usize, f32, Vec2)> = None;
                for &(lj, sj) in grid.get(&g).map(|v| v.as_slice()).unwrap_or(&[]) {
                    if lj == li || (roads[lj].layer != r.layer && roads[lj].flags & (flag::BRIDGE | flag::TUNNEL) != 0) {
                        continue;
                    }
                    let (a, b) = (roads[lj].pts[sj], roads[lj].pts[sj + 1]);
                    let ab = b - a;
                    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
                    let q = a + ab * t;
                    let d = q.distance(p);
                    if d < 1.6 && t > 0.0 && t < 1.0 && best.is_none_or(|x| d < x.0) {
                        best = Some((d, lj, sj, t, q));
                    }
                }
                if let Some((_, lj, sj, t, q)) = best {
                    splits[lj].push((sj as f32 + t, q));
                }
            }
        }
    }
    // Pieces.
    let mut pieces: Vec<Piece> = Vec::new();
    let mut piece_cut: Vec<[bool; 2]> = Vec::new();
    for (li, r) in roads.iter().enumerate() {
        let mut sp = std::mem::take(&mut splits[li]);
        sp.sort_by(|a, b| a.0.total_cmp(&b.0));
        sp.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-4);
        let mut cur: Vec<Vec2> = vec![r.pts[0]];
        let mut first = true;
        let mut k = 0;
        for vi in 1..r.pts.len() {
            // Inserted split points on segment (vi-1, vi).
            while k < sp.len() && sp[k].0 < vi as f32 {
                let (t, q) = sp[k];
                k += 1;
                if t.fract() == 0.0 {
                    continue; // a vertex split, handled below
                }
                cur.push(q);
                pieces.push(Piece { class: r.class, flags: r.flags, pts: std::mem::replace(&mut cur, vec![q]), n: [0, 0] });
                piece_cut.push([first && r.cut[0], false]);
                first = false;
            }
            cur.push(r.pts[vi]);
            let at_vertex = sp.iter().any(|s| s.0 == vi as f32);
            if at_vertex && vi + 1 < r.pts.len() {
                pieces.push(Piece { class: r.class, flags: r.flags, pts: std::mem::replace(&mut cur, vec![r.pts[vi]]), n: [0, 0] });
                piece_cut.push([first && r.cut[0], false]);
                first = false;
            }
        }
        pieces.push(Piece { class: r.class, flags: r.flags, pts: cur, n: [0, 0] });
        piece_cut.push([first && r.cut[0], r.cut[1]]);
    }
    // Drop degenerate pieces.
    let keep: Vec<bool> = pieces.iter().map(|p| polyline_len(&p.pts) > 0.5).collect();
    let mut pi = 0;
    pieces.retain(|_| {
        pi += 1;
        keep[pi - 1]
    });
    let mut pi = 0;
    piece_cut.retain(|_| {
        pi += 1;
        keep[pi - 1]
    });

    // --- 2. Nodes: piece ends within ~1.6 m are one junction (tile-edge ends stay apart).
    let mut nodes: Vec<Node> = Vec::new();
    let mut nkey: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, pc) in pieces.iter_mut().enumerate() {
        for e in 0..2 {
            let p = if e == 0 { pc.pts[0] } else { *pc.pts.last().unwrap() };
            if piece_cut[i][e] {
                pc.n[e] = nodes.len();
                nodes.push(Node { p, edge: true, ends: vec![(i, e)] });
                continue;
            }
            let g = ((p.x / 2.0).floor() as i32, (p.y / 2.0).floor() as i32);
            let mut found = None;
            'search: for dy in -1..=1 {
                for dx in -1..=1 {
                    for &ni in nkey.get(&(g.0 + dx, g.1 + dy)).map(|v| v.as_slice()).unwrap_or(&[]) {
                        if nodes[ni].p.distance(p) < 1.6 {
                            found = Some(ni);
                            break 'search;
                        }
                    }
                }
            }
            let ni = found.unwrap_or_else(|| {
                nodes.push(Node { p, edge: false, ends: Vec::new() });
                nkey.entry(g).or_default().push(nodes.len() - 1);
                nodes.len() - 1
            });
            nodes[ni].ends.push((i, e));
            pc.n[e] = ni;
        }
    }

    // --- 2b. Join pieces of the same road through two-way nodes (the tiles
    // split roads wherever an attribute changes): fewer, longer lanes.
    let mut alive = vec![true; pieces.len()];
    fn reverse(pieces: &mut [Piece], nodes: &mut [Node], i: usize) {
        pieces[i].pts.reverse();
        pieces[i].n.swap(0, 1);
        for e in 0..2 {
            for x in nodes[pieces[i].n[e]].ends.iter_mut() {
                if x.0 == i {
                    x.1 = 1 - x.1;
                }
            }
        }
    }
    for ni in 0..nodes.len() {
        if nodes[ni].edge || nodes[ni].ends.len() != 2 {
            continue;
        }
        let ((p, ep), (q, eq)) = (nodes[ni].ends[0], nodes[ni].ends[1]);
        if p == q || !alive[p] || !alive[q] || pieces[p].n[0] == pieces[p].n[1] || pieces[q].n[0] == pieces[q].n[1] {
            continue;
        }
        const KEEP: u8 = flag::ONEWAY | flag::RAMP | flag::TUNNEL | flag::LOWUSE;
        if pieces[p].class != pieces[q].class || (pieces[p].flags ^ pieces[q].flags) & KEEP != 0 {
            continue;
        }
        let oneway = pieces[p].flags & flag::ONEWAY != 0;
        // `a` must end at the node, `b` start there.
        let (a, ea, b, eb) = if ep == 1 { (p, ep, q, eq) } else { (q, eq, p, ep) };
        if oneway {
            if ea != 1 || eb != 0 {
                continue;
            }
        } else {
            if ea != 1 {
                reverse(&mut pieces, &mut nodes, a);
            }
            if eb != 0 {
                reverse(&mut pieces, &mut nodes, b);
            }
        }
        let tail: Vec<Vec2> = pieces[b].pts[1..].to_vec();
        pieces[a].pts.extend(tail);
        let far = pieces[b].n[1];
        pieces[a].n[1] = far;
        pieces[a].flags |= pieces[b].flags & flag::BRIDGE;
        for x in nodes[far].ends.iter_mut() {
            if x.0 == b {
                *x = (a, 1);
            }
        }
        nodes[ni].ends.clear();
        alive[b] = false;
    }

    // --- 3. Lanes per piece and direction, trimmed back from junctions.
    let trim_at = |ni: usize| -> f32 {
        let nd = &nodes[ni];
        if nd.edge || nd.ends.len() < 2 {
            return 0.0;
        }
        let wmax = nd.ends.iter().map(|&(p, _)| width_of(pieces[p].class)).fold(0.0, f32::max);
        if nd.ends.len() == 2 {
            // A bend or a change of road: a short easement.
            let dir = |&(p, e): &(usize, usize)| {
                let q = &pieces[p].pts;
                if e == 0 { (q[1] - q[0]).normalize_or_zero() } else { (q[q.len() - 2] - q[q.len() - 1]).normalize_or_zero() }
            };
            let (a, b) = (dir(&nd.ends[0]), dir(&nd.ends[1]));
            let straight = -a.dot(b);
            return if straight > 0.97 { 1.5 } else { (wmax * 0.5).clamp(2.0, 9.0) };
        }
        (wmax * 0.5 + 2.0).clamp(4.0, 14.0)
    };
    let mut pend: Vec<Pending> = Vec::new();
    let mut ways: Vec<Way> = Vec::new();
    // Ways ending / starting at each node.
    let mut ins: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    let mut outs: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    let mut edge_in = Vec::new();
    let mut edge_out = Vec::new();
    for (i, pc) in pieces.iter().enumerate() {
        if !alive[i] {
            continue;
        }
        let len = polyline_len(&pc.pts);
        let (t0, t1) = (trim_at(pc.n[0]), trim_at(pc.n[1]));
        // Short pieces between two junctions: keep at least a third.
        let scale = if t0 + t1 > len * 0.66 { len * 0.66 / (t0 + t1).max(1e-3) } else { 1.0 };
        let centre = densify(&sub_polyline(&pc.pts, t0 * scale, len - t1 * scale), 14.0);
        if centre.len() < 2 {
            continue;
        }
        let oneway = pc.flags & flag::ONEWAY != 0;
        let ramp = pc.flags & flag::RAMP != 0;
        let offs = lane_offsets(pc.class, oneway, ramp);
        let mid = pc.pts[pc.pts.len() / 2];
        let urban = (ctx.built)(mid);
        let speed = speed_limit(pc.class, region.rules, urban, ramp);
        let density = base_density(pc.class, urban, pc.flags & flag::LOWUSE != 0);
        for fwd in [true, false] {
            if !fwd && oneway {
                continue;
            }
            let path: Vec<Vec2> = if fwd { centre.clone() } else { centre.iter().rev().copied().collect() };
            let (n_start, n_end) = if fwd { (pc.n[0], pc.n[1]) } else { (pc.n[1], pc.n[0]) };
            let mut lanes = Vec::new();
            for (k, &o) in offs.iter().enumerate() {
                let li = pend.len() as u32;
                let mut fl = pc.flags & !flag::ONEWAY;
                if nodes[n_start].edge {
                    fl |= flag::EDGE_IN;
                    edge_in.push(li);
                }
                if nodes[n_end].edge {
                    fl |= flag::EDGE_OUT;
                    edge_out.push(li);
                }
                pend.push(Pending {
                    pts: offset_polyline(&path, o, left),
                    class: pc.class,
                    flags: fl,
                    kerb: k as u8,
                    nlanes: offs.len() as u8,
                    speed,
                    density,
                    succ: Vec::new(),
                    prio: 0.0,
                    node: n_end as u32,
                    conf: Vec::new(),
                    way: u32::MAX,
                });
                lanes.push(li);
            }
            let wi = ways.len();
            ways.push(Way { piece: i, fwd, lanes });
            outs[n_start].push(wi);
            ins[n_end].push(wi);
        }
    }

    // --- 4. Connectors through the junctions.
    let tangent_end = |l: &Pending| {
        let n = l.pts.len();
        (l.pts[n - 1] - l.pts[n - 2]).normalize_or(Vec2::X)
    };
    let tangent_start = |l: &Pending| (l.pts[1] - l.pts[0]).normalize_or(Vec2::X);
    for ni in 0..nodes.len() {
        if nodes[ni].edge {
            continue;
        }
        let node_conn_start = pend.len();
        let degree = nodes[ni].ends.len();
        for &wi in &ins[ni] {
            let win = ways[wi].clone();
            let in_l = &pend[win.lanes[0] as usize];
            let (in_class, in_flags) = (in_l.class, in_l.flags);
            let d_in = tangent_end(&pend[*win.lanes.last().unwrap() as usize]);
            // Candidate exits: (way, signed turn, kerb-side turn).
            let mut cands: Vec<(usize, f32, bool)> = Vec::new();
            for &wo in &outs[ni] {
                let wout = &ways[wo];
                let u_turn = wout.piece == win.piece;
                if u_turn && degree > 1 {
                    continue;
                }
                let d_out = tangent_start(&pend[wout.lanes[0] as usize]);
                let cross = d_in.perp_dot(d_out);
                let ang = d_in.dot(d_out).clamp(-1.0, 1.0).acos();
                if !u_turn && ang > 2.6 {
                    continue;
                }
                // Left turn when cross < 0 (x east, z south).
                let kerb = if left { cross < 0.0 } else { cross > 0.0 };
                cands.push((wo, if u_turn { 3.0 } else { ang }, kerb));
            }
            if cands.is_empty() {
                continue;
            }
            let has_straight = cands.iter().any(|c| c.1 < 0.6);
            for &(wo, ang, kerb) in &cands {
                let wout = ways[wo].clone();
                let (n_in, n_out) = (win.lanes.len(), wout.lanes.len());
                let out_l = &pend[wout.lanes[0] as usize];
                let (out_class, out_flags) = (out_l.class, out_l.flags);
                let turn = ang >= 0.6;
                // Which in-lane feeds which out-lane (kerb index).
                let mut pairs: Vec<(usize, usize)> = Vec::new();
                if ang > 2.9 {
                    pairs.push((n_in - 1, n_out - 1));
                } else if !turn {
                    for k in 0..n_in {
                        let j = if n_in == 1 { 0 } else { ((k as f32) * (n_out - 1) as f32 / (n_in - 1) as f32).round() as usize };
                        pairs.push((k, j.min(n_out - 1)));
                    }
                    // More lanes out than in: feed the extra ones too.
                    for j in 0..n_out {
                        if !pairs.iter().any(|p| p.1 == j) {
                            let k = if n_out == 1 { 0 } else { ((j as f32) * (n_in - 1) as f32 / (n_out - 1) as f32).round() as usize };
                            pairs.push((k.min(n_in - 1), j));
                        }
                    }
                } else if !has_straight {
                    // End of the road: every lane may turn, keeping its side.
                    for k in 0..n_in {
                        let j = if kerb { 0 } else { n_out - 1 };
                        let ok = n_in == 1 || (kerb && k < n_in.div_ceil(2)) || (!kerb && k >= n_in / 2);
                        if ok {
                            pairs.push((k, j));
                        }
                    }
                } else if kerb {
                    pairs.push((0, 0));
                } else {
                    pairs.push((n_in - 1, n_out - 1));
                }
                // Share of the traffic taking this exit.
                let rank_diff = out_class as i32 - in_class as i32;
                let mut w = if rank_diff <= 0 { 1.0 } else { 0.55f32.powi(rank_diff) };
                if out_class == class::SERVICE {
                    w *= 0.15;
                }
                if out_flags & flag::LOWUSE != 0 {
                    w *= 0.3;
                }
                w *= if ang > 2.9 {
                    0.02
                } else if !turn {
                    1.0
                } else if kerb {
                    0.4
                } else {
                    0.28
                };
                // Leaving a motorway / trunk by a slip road: a minority.
                if in_class <= class::TRUNK && in_flags & flag::RAMP == 0 && out_flags & flag::RAMP != 0 {
                    w *= 0.15;
                }
                // Priority: major roads first, straight on before turning, slip roads give way.
                let prio = in_class as f32 * 2.0 + if turn { 1.0 } else { 0.0 } + if in_flags & flag::RAMP != 0 { 1.5 } else { 0.0 };
                for (k, j) in pairs {
                    let (la, lb) = (win.lanes[k], wout.lanes[j]);
                    let (a, b) = (&pend[la as usize], &pend[lb as usize]);
                    let (p0, d0) = (*a.pts.last().unwrap(), tangent_end(a));
                    let (p1, d1) = (b.pts[0], tangent_start(b));
                    let l = p0.distance(p1);
                    let pts: Vec<Vec2> = if ang > 2.9 {
                        // U-turn at a dead end: a loop.
                        let r = l.max(4.0);
                        let (c1, c2) = (p0 + d0 * r, p1 - d1 * r);
                        (0..=8).map(|t| cubic(p0, c1, c2, p1, t as f32 / 8.0)).collect()
                    } else if l < 0.05 {
                        vec![p0, p1 + d1 * 0.05]
                    } else {
                        let c = l * 0.42;
                        let (c1, c2) = (p0 + d0 * c, p1 - d1 * c);
                        let k = ((l / 3.5).ceil() as usize).clamp(2, 8);
                        (0..=k).map(|t| cubic(p0, c1, c2, p1, t as f32 / k as f32)).collect()
                    };
                    // Speed for the curve (2.5 m/s² sideways), slower through busy junctions.
                    let v_lim = a.speed.min(b.speed);
                    let radius = if ang > 0.05 { (l * 0.5 / (ang * 0.5).sin()).max(3.0) } else { 1e4 };
                    let mut v = v_lim.min((2.5 * radius).sqrt());
                    if degree >= 3 {
                        v = v.min((v_lim * 0.75).max(7.0));
                    }
                    if ang > 2.9 {
                        v = 3.0;
                    }
                    let ci = pend.len() as u32;
                    pend[la as usize].succ.push(Succ { lane: ci, weight: w });
                    pend.push(Pending {
                        pts,
                        class: class::LINK,
                        flags: flag::CONNECTOR | (in_flags & flag::TUNNEL) | (out_flags & flag::TUNNEL),
                        kerb: 0,
                        nlanes: 1,
                        speed: v,
                        density: 0.0,
                        succ: vec![Succ { lane: lb, weight: 1.0 }],
                        prio,
                        node: ni as u32,
                        conf: Vec::new(),
                        way: wi as u32,
                    });
                }
            }
        }
        // Conflicts between connectors from different approaches.
        let conns: Vec<usize> = (node_conn_start..pend.len()).collect();
        if conns.len() > 1 && conns.len() <= 96 {
            for x in 0..conns.len() {
                for y in x + 1..conns.len() {
                    let (a, b) = (conns[x], conns[y]);
                    if pend[a].way == pend[b].way {
                        continue;
                    }
                    let merge = pend[a].succ[0].lane == pend[b].succ[0].lane;
                    if merge || paths_cross(&pend[a].pts, &pend[b].pts) {
                        pend[a].conf.push(b as u32);
                        pend[b].conf.push(a as u32);
                    }
                }
            }
        }
    }

    // --- 5. Pack: drape on the terrain, lengths, predecessors.
    let n = pend.len();
    let mut preds: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (i, l) in pend.iter().enumerate() {
        for s in &l.succ {
            preds[s.lane as usize].push(i as u32);
        }
    }
    let mut net = RoadNet { id: ctx.id, lanes: Vec::new(), pts: Vec::new(), succ: Vec::new(), pred: Vec::new(), conf: Vec::new(), edge_in, edge_out, region, urban: 0.0 };
    let mut built_n = 0usize;
    for (i, l) in pend.iter().enumerate() {
        let p0 = net.pts.len() as u32;
        let mut s = 0.0;
        let mut lo = Vec2::splat(f32::MAX);
        let mut hi = Vec2::splat(f32::MIN);
        for (j, p) in l.pts.iter().enumerate() {
            if j > 0 {
                s += p.distance(l.pts[j - 1]);
            }
            lo = lo.min(*p);
            hi = hi.max(*p);
            net.pts.push(LanePt { x: p.x, y: (ctx.height)(*p), z: p.y, s });
        }
        if l.class != class::LINK && (ctx.built)(l.pts[l.pts.len() / 2]) {
            built_n += 1;
        }
        let succ0 = net.succ.len() as u32;
        net.succ.extend_from_slice(&l.succ);
        let pred0 = net.pred.len() as u32;
        net.pred.extend_from_slice(&preds[i]);
        let conf0 = net.conf.len() as u32;
        net.conf.extend_from_slice(&l.conf);
        net.lanes.push(Lane {
            p0,
            np: l.pts.len() as u32,
            len: s.max(0.05),
            speed: l.speed,
            succ0,
            nsucc: l.succ.len() as u32,
            pred0,
            npred: preds[i].len() as u32,
            conf0,
            nconf: l.conf.len() as u32,
            lo: lo.into(),
            hi: hi.into(),
            meta: l.class as u32 | (l.flags as u32) << 8 | (l.kerb as u32) << 16 | (l.nlanes as u32) << 24,
            density: l.density,
            prio: l.prio,
            node: l.node,
        });
    }
    let roads_n = pend.iter().filter(|l| l.class != class::LINK).count().max(1);
    net.urban = built_n as f32 / roads_n as f32;
    (!net.lanes.is_empty()).then_some(net)
}

mod wire_impls {
    use super::*;
    use crate::platform::wire::{R, Res, W, Wire, get_pod, put_pod};

    impl Wire for Region {
        fn put(&self, w: &mut W) {
            (self.left as u8 | self.rules << 1 | (self.london as u8) << 4).put(w);
        }
        fn get(r: &mut R) -> Res<Self> {
            let b = u8::get(r)?;
            Ok(Region { left: b & 1 != 0, rules: (b >> 1) & 7, london: b & 16 != 0 })
        }
    }

    impl Wire for RoadNet {
        fn put(&self, w: &mut W) {
            self.id.put(w);
            put_pod(w, &self.lanes);
            put_pod(w, &self.pts);
            put_pod(w, &self.succ);
            put_pod(w, &self.pred);
            put_pod(w, &self.conf);
            put_pod(w, &self.edge_in);
            put_pod(w, &self.edge_out);
            self.region.put(w);
            self.urban.put(w);
        }
        fn get(r: &mut R) -> Res<Self> {
            Ok(RoadNet {
                id: Wire::get(r)?,
                lanes: get_pod(r)?,
                pts: get_pod(r)?,
                succ: get_pod(r)?,
                pred: get_pod(r)?,
                conf: get_pod(r)?,
                edge_in: get_pod(r)?,
                edge_out: get_pod(r)?,
                region: Wire::get(r)?,
                urban: Wire::get(r)?,
            })
        }
    }
}
