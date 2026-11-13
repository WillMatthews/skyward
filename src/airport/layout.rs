//! Airport layout: everything placed at one airport, built from the
//! airport's map tiles (aprons, taxiways with refs, gates, service roads,
//! airside buildings — [`AeroTile`]), its OurAirports runways and, when
//! available, Overpass data (parking positions, windsocks, towers), with a
//! procedural fallback wherever the data is sparse:
//!
//! * ground markings: taxiway centrelines (black-edged on concrete), edge
//!   lines, runway-holding positions (pattern A) with enhanced centrelines,
//!   stand lead-in lines, stop bars, stand numbers, equipment restraint
//!   lines, apron road edges;
//! * stands from gates / parking positions / terminal facades / apron rows,
//!   with parked aircraft sized to the stand, jet bridges at contact stands
//!   and ground vehicles serving them; vehicles driving the service roads;
//! * signs (runway holding position, location, direction), windsocks,
//!   floodlight masts, a control tower where the map has none, and the
//!   taxiway / stop-bar / apron lights.
//!
//! Built on a background thread (airport/mod.rs) in the airport's own frame
//! (`Proj::new(lat, lon)` of its reference point), flat: [`bake`] places it
//! on each tile's terrain.

use super::mesh::{self, Panel, Vehicle};
use super::overpass::OsmAirport;
use super::tile::{AeroTile, BClass};
use super::types::{self, Role};
use crate::airports::{Airport, Kind as AKind};
use crate::lights::{LightInst, colour, kind};
use crate::osm::{Proj, TileId, point_in_ring};
use crate::scene::{MeshData, Runway, mat};
use glam::{Vec2, Vec3, vec2, vec3};
use std::collections::HashMap;
use std::sync::Arc;

/// A static object: a local mesh (y up from its base) placed at `at`.
pub struct Object {
    pub at: Vec2,
    pub mesh: MeshData,
    /// Base = lowest ground under this footprint (else the ground at `at`).
    pub foot: Option<Vec<Vec2>>,
}

/// An instance on the ground: aircraft type / vehicle kind, position, heading.
#[derive(Clone, Copy, Debug)]
pub struct Placed {
    pub model: u16,
    pub at: Vec2,
    pub hdg: f32,
    pub livery: u16,
}

/// Vehicles shuttling along a service road.
pub struct Route {
    pub pts: Vec<Vec2>,
    pub len: f32,
    /// (kind, phase offset m, speed m/s).
    pub vehicles: Vec<(u16, f32, f32)>,
}

#[derive(Clone, Debug)]
pub struct Stand {
    pub name: String,
    /// Nose-wheel stop point and aircraft heading (rad).
    pub stop: Vec2,
    pub hdg: f32,
    /// Largest wingspan it takes (m).
    pub span: f32,
    /// Facade point and outward normal for a jet bridge.
    pub contact: Option<(Vec2, Vec2)>,
}

#[derive(Clone, Debug)]
pub struct Hold {
    pub at: Vec2,
    /// Taxiway direction toward the runway.
    pub dir: Vec2,
    pub width: f32,
    pub taxiway: String,
    pub runway: String,
}

pub struct Layout {
    pub ident: String,
    pub frame: Proj,
    pub kind: AKind,
    pub marks: MeshData,
    pub objects: Vec<Object>,
    /// Lights: pos = (x, height above ground, z) in the airport frame.
    pub lights: Vec<LightInst>,
    pub parked: Vec<Placed>,
    pub vehicles: Vec<Placed>,
    pub socks: Vec<Vec2>,
    pub routes: Vec<Route>,
    pub stands: Vec<Stand>,
    pub holds: Vec<Hold>,
    /// Taxiway centrelines and their refs (airport frame): ATC taxi routing (atc/taxi.rs).
    pub taxiways: Vec<(Vec<Vec2>, String)>,
    /// Taxiway width (m) and apron outlines (airport frame): the pavement the
    /// tyres roll on (src/ground/surface.rs).
    pub taxi_w: f32,
    pub aprons: Vec<Vec<Vec2>>,
    /// Bounds of everything (airport frame).
    pub lo: Vec2,
    pub hi: Vec2,
    /// Tiles it was built from.
    pub tiles: Vec<TileId>,
}

// ---------------------------------------------------------------- inputs

struct Bld {
    ring: Vec<Vec2>,
    class: BClass,
    height: f32,
    lo: Vec2,
    hi: Vec2,
    /// Edges lying on a tile seam (not real facades).
    seam: Vec<bool>,
}

struct Apron {
    outer: Vec<Vec2>,
    holes: Vec<Vec<Vec2>>,
    lo: Vec2,
    hi: Vec2,
}

impl Apron {
    fn contains(&self, p: Vec2) -> bool {
        p.x >= self.lo.x && p.x <= self.hi.x && p.y >= self.lo.y && p.y <= self.hi.y && point_in_ring(p, &self.outer) && !self.holes.iter().any(|h| point_in_ring(p, h))
    }
}

struct Taxi {
    pts: Vec<Vec2>,
    name: String,
}

/// Uniform grid of segment ids for nearest / intersection queries.
struct SegGrid {
    cell: f32,
    map: HashMap<(i32, i32), Vec<(u32, u32)>>,
}

impl SegGrid {
    fn new(cell: f32) -> Self {
        SegGrid { cell, map: HashMap::new() }
    }
    fn key(&self, p: Vec2) -> (i32, i32) {
        ((p.x / self.cell).floor() as i32, (p.y / self.cell).floor() as i32)
    }
    fn insert(&mut self, a: Vec2, b: Vec2, id: (u32, u32)) {
        let (k0, k1) = (self.key(a.min(b)), self.key(a.max(b)));
        for x in k0.0..=k1.0 {
            for y in k0.1..=k1.1 {
                self.map.entry((x, y)).or_default().push(id);
            }
        }
    }
    fn near(&self, p: Vec2, r: f32) -> Vec<(u32, u32)> {
        let (k0, k1) = (self.key(p - Vec2::splat(r)), self.key(p + Vec2::splat(r)));
        let mut out = Vec::new();
        for x in k0.0..=k1.0 {
            for y in k0.1..=k1.1 {
                if let Some(v) = self.map.get(&(x, y)) {
                    out.extend_from_slice(v);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

struct Ctx {
    kind: AKind,
    scheduled: bool,
    /// An air force / naval air base (by name): transports on the big stands.
    military: bool,
    runways: Vec<Runway>,
    aprons: Vec<Apron>,
    blds: Vec<Bld>,
    taxis: Vec<Taxi>,
    tgrid: SegGrid,
    gates: Vec<(Vec2, String)>,
    service: Vec<Vec<Vec2>>,
    /// Aerodrome outline pieces owned by this airport.
    pieces: Vec<(Vec<Vec2>, Vec2, Vec2)>,
    taxi_w: f32,
    seed: u64,
}

impl Ctx {
    fn in_apron(&self, p: Vec2) -> bool {
        self.aprons.iter().any(|a| a.contains(p))
    }
    fn in_building(&self, p: Vec2, margin: f32) -> bool {
        self.blds.iter().any(|b| {
            p.x >= b.lo.x - margin
                && p.x <= b.hi.x + margin
                && p.y >= b.lo.y - margin
                && p.y <= b.hi.y + margin
                && (point_in_ring(p, &b.ring) || (margin > 0.0 && super::tile::ring_dist(&b.ring, p) < margin))
        })
    }
    fn in_aerodrome(&self, p: Vec2) -> bool {
        self.pieces.is_empty() || self.pieces.iter().any(|(r, lo, hi)| p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && point_in_ring(p, r))
    }
    /// Distance to the nearest taxiway centreline, its point and direction.
    fn nearest_taxi(&self, p: Vec2, r: f32) -> Option<(f32, Vec2, Vec2, usize)> {
        let mut best: Option<(f32, Vec2, Vec2, usize)> = None;
        for (ti, si) in self.tgrid.near(p, r) {
            let t = &self.taxis[ti as usize];
            let (a, b) = (t.pts[si as usize], t.pts[si as usize + 1]);
            let ab = b - a;
            let k = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            let q = a + ab * k;
            let d = p.distance(q);
            if d <= r && best.is_none_or(|bb| d < bb.0) {
                best = Some((d, q, ab.normalize_or_zero(), ti as usize));
            }
        }
        best
    }
    /// On (or near) a runway: (runway, signed lateral offset, along).
    fn runway_at(&self, p: Vec2, margin: f32) -> Option<(usize, f32, f32)> {
        self.runways.iter().enumerate().find_map(|(i, rw)| {
            let len = rw.length();
            let d = (rw.b - rw.a) / len;
            let s = (p - rw.a).dot(d);
            let l = (p - rw.a).perp_dot(d);
            (s > -margin && s < len + margin && l.abs() < rw.width * 0.5 + margin).then_some((i, l, s))
        })
    }
    fn hash(&self, k: u64) -> u64 {
        crate::scene::hash(self.seed ^ k.wrapping_mul(0x9e37_79b9_7f4a_7c15))
    }
    fn rand(&self, k: u64) -> f32 {
        (self.hash(k) >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// Hold-position distance from the runway centreline (m), ICAO Annex 14 table 3-2-ish.
fn hold_dist(rw: &Runway) -> f32 {
    let len = rw.length();
    if len >= 1800.0 {
        90.0
    } else if len >= 1200.0 {
        75.0
    } else if len >= 800.0 {
        45.0
    } else {
        35.0
    }
}

// ---------------------------------------------------------------- build

/// Build the layout of `ap` from its tiles (and Overpass data if any).
pub fn build(ap: &Airport, tiles: &[(TileId, Arc<AeroTile>)], osm: Option<&OsmAirport>) -> Layout {
    let frame = Proj::new(ap.lat, ap.lon);
    let seed = crate::scene::hash(ap.ident.bytes().fold(0u64, |h, b| h.wrapping_mul(131).wrapping_add(b as u64)));
    let runways: Vec<Runway> = ap.runways.iter().filter_map(|r| crate::airports::to_world(ap, r, &frame)).collect();
    let taxi_w = match ap.kind {
        AKind::Large => 23.0,
        AKind::Medium => 18.0,
        _ => 10.5,
    };
    let mut c = Ctx {
        kind: ap.kind,
        scheduled: ap.scheduled || ap.kind == AKind::Large,
        military: is_military(&ap.name),
        runways,
        aprons: Vec::new(),
        blds: Vec::new(),
        taxis: Vec::new(),
        tgrid: SegGrid::new(60.0),
        gates: Vec::new(),
        service: Vec::new(),
        pieces: Vec::new(),
        taxi_w,
        seed,
    };
    // Tile frames → the airport frame; only what lies in this airport's aerodrome.
    for (id, t) in tiles {
        let xf = Proj::tile_frame(*id).map_to(&frame);
        let (rlo, rhi) = id.rect(&Proj::tile_frame(*id));
        let tr = |p: Vec2| xf.apply(p);
        let own: Vec<&super::tile::AeroPiece> = t.pieces.iter().filter(|p| p.owner == ap.ident).collect();
        if own.is_empty() {
            continue;
        }
        let mine = |p: Vec2, d: f32| own.iter().any(|pc| point_in_ring(p, &pc.poly.outer) || super::tile::ring_dist(&pc.poly.outer, p) < d);
        for pc in &own {
            let r: Vec<Vec2> = pc.poly.outer.iter().map(|p| tr(*p)).collect();
            let (lo, hi) = mesh::bbox(&r);
            c.pieces.push((r, lo, hi));
        }
        for a in &t.aprons {
            if !mine(a.outer[0], 150.0) && !a.outer.iter().any(|p| mine(*p, 50.0)) {
                continue;
            }
            let outer: Vec<Vec2> = a.outer.iter().map(|p| tr(*p)).collect();
            let (lo, hi) = mesh::bbox(&outer);
            c.aprons.push(Apron { outer, holes: a.holes.iter().map(|h| h.iter().map(|p| tr(*p)).collect()).collect(), lo, hi });
        }
        for tx in &t.taxiways {
            if tx.pts.len() < 2 || !tx.pts.iter().any(|p| mine(*p, 150.0)) {
                continue;
            }
            c.taxis.push(Taxi { pts: tx.pts.iter().map(|p| tr(*p)).collect(), name: tx.name.clone() });
        }
        for (p, n) in &t.gates {
            if mine(*p, 150.0) {
                c.gates.push((tr(*p), n.clone()));
            }
        }
        for s in &t.service {
            if s.pts.len() >= 2 {
                c.service.push(s.pts.iter().map(|p| tr(*p)).collect());
            }
        }
        for b in &t.buildings {
            let on_seam = |p: Vec2| (p.x - rlo.x).abs() < 0.05 || (p.x - rhi.x).abs() < 0.05 || (p.y - rlo.y).abs() < 0.05 || (p.y - rhi.y).abs() < 0.05;
            let n = b.outer.len();
            let seam = (0..n).map(|i| on_seam(b.outer[i]) && on_seam(b.outer[(i + 1) % n])).collect();
            let ring: Vec<Vec2> = b.outer.iter().map(|p| tr(*p)).collect();
            let (lo, hi) = mesh::bbox(&ring);
            c.blds.push(Bld { ring, class: BClass::from_code(b.class), height: b.height, lo, hi, seam });
        }
    }
    if std::env::var("SKYWARD_AIRPORT_DEBUG").is_ok_and(|v| v == "2") {
        let n = |k: BClass| c.blds.iter().filter(|b| b.class == k).count();
        eprintln!(
            "  {}: {} pieces, {} aprons, {} taxiways, {} gates, {} service roads; buildings: {} terminal {} hangar {} tower {} airside",
            ap.ident,
            c.pieces.len(),
            c.aprons.len(),
            c.taxis.len(),
            c.gates.len(),
            c.service.len(),
            n(BClass::Terminal),
            n(BClass::Hangar),
            n(BClass::Tower),
            n(BClass::Airside)
        );
    }
    if std::env::var("SKYWARD_AIRPORT_DEBUG").is_ok_and(|v| v == "3") {
        for a in &c.aprons {
            let ctr = a.outer.iter().copied().sum::<Vec2>() / a.outer.len() as f32;
            eprintln!("  apron at {:?} area {:.0}", frame.to_latlon(ctr), mesh::ring_area(&a.outer).abs());
        }
        for b in &c.blds {
            eprintln!("  building {:?} at {:?} area {:.0} h {:.0}", b.class, frame.to_latlon((b.lo + b.hi) * 0.5), mesh::ring_area(&b.ring).abs(), b.height);
        }
    }
    for (ti, t) in c.taxis.iter().enumerate() {
        for si in 0..t.pts.len() - 1 {
            c.tgrid.insert(t.pts[si], t.pts[si + 1], (ti as u32, si as u32));
        }
    }
    let mut l = Layout {
        ident: ap.ident.clone(),
        frame,
        kind: ap.kind,
        marks: MeshData::default(),
        objects: Vec::new(),
        lights: Vec::new(),
        parked: Vec::new(),
        vehicles: Vec::new(),
        socks: Vec::new(),
        routes: Vec::new(),
        stands: Vec::new(),
        holds: Vec::new(),
        taxiways: c.taxis.iter().map(|t| (t.pts.clone(), t.name.clone())).collect(),
        taxi_w: c.taxi_w,
        aprons: c.aprons.iter().map(|a| a.outer.clone()).collect(),
        lo: Vec2::splat(f32::MAX),
        hi: Vec2::splat(f32::MIN),
        tiles: tiles.iter().map(|t| t.0).collect(),
    };
    let osm_xy = |ll: (f64, f64)| frame.to_world(ll.0, ll.1);
    holds(&c, &mut l, osm);
    let stands = stands(&c, osm, &osm_xy);
    occupy(&c, &mut l, stands);
    marks(&c, &mut l);
    signs(&c, &mut l);
    lights_and_masts(&c, &mut l);
    windsocks(&c, &mut l, osm, &osm_xy);
    tower(&c, &mut l, osm, &osm_xy);
    services(&c, &mut l);
    routes(&c, &mut l);
    // Bounds.
    let mut lo = Vec2::splat(f32::MAX);
    let mut hi = Vec2::splat(f32::MIN);
    for (r, a, b) in &c.pieces {
        let _ = r;
        lo = lo.min(*a);
        hi = hi.max(*b);
    }
    for rw in &c.runways {
        lo = lo.min(rw.a.min(rw.b));
        hi = hi.max(rw.a.max(rw.b));
    }
    l.lo = lo - Vec2::splat(200.0);
    l.hi = hi + Vec2::splat(200.0);
    l
}

// ---------------------------------------------------------------- holds

fn holds(c: &Ctx, l: &mut Layout, _osm: Option<&OsmAirport>) {
    for t in &c.taxis {
        for w in t.pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            for rw in &c.runways {
                let len = rw.length();
                let d = (rw.b - rw.a) / len;
                let hd = hold_dist(rw).min(rw.width * 0.5 + 80.0);
                let side = |p: Vec2| (p - rw.a).perp_dot(d);
                let along = |p: Vec2| (p - rw.a).dot(d);
                let (la, lb) = (side(a), side(b));
                for s in [-1.0f32, 1.0] {
                    let target = s * hd;
                    if (la - target) * (lb - target) >= 0.0 || (la - lb).abs() < 1e-3 {
                        continue;
                    }
                    let k = (target - la) / (lb - la);
                    let p = a + (b - a) * k;
                    let al = along(p);
                    if al < -40.0 || al > len + 40.0 {
                        continue;
                    }
                    // Direction toward the runway centreline.
                    let mut dir = (b - a).normalize_or_zero();
                    if (side(p + dir) - side(p)).abs() < 1e-4 {
                        continue;
                    }
                    if side(p + dir).abs() > side(p).abs() {
                        dir = -dir;
                    }
                    // Mostly crossing (not a parallel taxiway grazing the line).
                    if dir.dot(vec2(-d.y, d.x)).abs() < 0.35 {
                        continue;
                    }
                    if l.holds.iter().any(|h| h.at.distance(p) < 20.0) {
                        continue;
                    }
                    l.holds.push(Hold { at: p, dir, width: c.taxi_w, taxiway: t.name.clone(), runway: rw.name.clone() });
                }
            }
        }
    }
}

// ---------------------------------------------------------------- stands

/// Nearest facade of an airside building to `p` within `r`: (point, outward normal, class, building index).
fn facade(c: &Ctx, p: Vec2, r: f32, want_terminal: bool) -> Option<(Vec2, Vec2, BClass, usize)> {
    let mut best: Option<(f32, Vec2, Vec2, BClass, usize)> = None;
    for (bi, b) in c.blds.iter().enumerate() {
        if want_terminal && b.class != BClass::Terminal {
            continue;
        }
        if p.x < b.lo.x - r || p.x > b.hi.x + r || p.y < b.lo.y - r || p.y > b.hi.y + r {
            continue;
        }
        let n = b.ring.len();
        let s = if mesh::ring_area(&b.ring) > 0.0 { 1.0 } else { -1.0 };
        for i in 0..n {
            if b.seam[i] {
                continue;
            }
            let (a, q) = (b.ring[i], b.ring[(i + 1) % n]);
            let e = q - a;
            if e.length() < 4.0 {
                continue;
            }
            let k = ((p - a).dot(e) / e.length_squared()).clamp(0.0, 1.0);
            let x = a + e * k;
            let d = p.distance(x);
            if d < r && best.is_none_or(|bb| d < bb.0) {
                // Outward normal of a CCW ring (+X east, +Z south): (e.y, -e.x) * s.
                let nrm = vec2(e.y, -e.x).normalize() * s;
                best = Some((d, x, nrm, b.class, bi));
            }
        }
    }
    best.map(|b| (b.1, b.2, b.3, b.4))
}

/// Aircraft type for a stand of span `span` at this airport.
/// Air force / naval air base, by its OurAirports name.
fn is_military(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    n.starts_with("raf ")
        || n.starts_with("rnas ")
        || [
            "air force base",
            "air base",
            "afb",
            "naval air station",
            "air station",
            "army airfield",
            "air reserve base",
            "air national guard",
            "fliegerhorst",
            "base aérea",
            "base aerea",
            "base aérienne",
            "militar",
            "military",
        ]
        .iter()
        .any(|k| n.contains(k))
}

/// A flying-club / GA mix for a light stand: mostly C172s, Super Cubs, the odd
/// Extra 500 and aerobatic Zlin (`r` uniform 0..1).
fn light_type(r: f32) -> u16 {
    if r < 0.55 {
        types::C172
    } else if r < 0.75 {
        types::CUB
    } else if r < 0.88 {
        types::EXTRA500
    } else {
        types::ZLIN
    }
}

fn pick_type(c: &Ctx, span: f32, k: u64) -> Option<u16> {
    let r = c.rand(k);
    let light = |c: &Ctx| light_type(c.rand(k ^ 0x9E37_79B9_7F4A_7C15));
    // Air bases: Hercules on the big stands, the odd Vulcan; light aircraft
    // (flying club) elsewhere.
    if c.military && span >= 30.0 && c.kind != AKind::Small {
        return Some(if r < 0.07 {
            types::VULCAN
        } else if span >= 40.0 && r < 0.75 {
            types::C130
        } else {
            types::C172
        });
    }
    match c.kind {
        // Business / general aviation fields (no scheduled service): light
        // aircraft, the odd warbird, a few regional jets on the big stands.
        AKind::Medium if !c.scheduled => {
            if span >= 30.0 && r < 0.25 {
                Some(types::E190)
            } else if span >= 20.0 && r < 0.6 {
                // Business jets.
                Some(types::CITATIONX)
            } else if span >= 30.0 && r < 0.32 {
                // A vintage DC-3 on the odd big GA stand.
                Some(types::DC3)
            } else if span >= 12.0 {
                Some(if r > 0.93 { types::SPITFIRE } else { light(c) })
            } else {
                None
            }
        }
        AKind::Large | AKind::Medium => {
            if span >= 62.0 && c.kind == AKind::Large {
                // Widebody stands at the hubs.
                Some(if r < 0.8 { types::TRIPLE7 } else { types::A320 })
            } else if span >= 35.0 {
                Some(if r < 0.72 || c.kind == AKind::Large && r < 0.82 {
                    types::A320
                } else if r < 0.88 {
                    types::E190
                } else if r < 0.94 {
                    types::CRJ700
                } else {
                    types::ATR
                })
            } else if span >= 29.0 {
                Some(if r < 0.45 {
                    types::E190
                } else if r < 0.7 {
                    types::CRJ700
                } else {
                    types::ATR
                })
            } else if span >= 23.5 && r < 0.5 {
                Some(types::CRJ700)
            } else if span >= 27.5 {
                Some(types::ATR)
            } else if span >= 12.0 {
                Some(types::C172)
            } else {
                None
            }
        }
        _ => {
            if span >= 12.0 {
                Some(if r < 0.08 { types::SPITFIRE } else { light(c) })
            } else {
                None
            }
        }
    }
}

fn stands(c: &Ctx, osm: Option<&OsmAirport>, osm_xy: &dyn Fn((f64, f64)) -> Vec2) -> Vec<Stand> {
    let mut out: Vec<Stand> = Vec::new();
    let bridge_ok = matches!(c.kind, AKind::Large | AKind::Medium);
    let add = |out: &mut Vec<Stand>, s: Stand| {
        if !out.iter().any(|o| o.stop.distance(s.stop) < 18.0) {
            out.push(s);
        }
    };
    // 1. Mapped parking positions (Overpass).
    if let Some(o) = osm {
        for (pts, name) in &o.stands {
            let ps: Vec<Vec2> = pts.iter().map(|p| osm_xy(*p)).collect();
            if !c.in_aerodrome(ps[0]) {
                continue;
            }
            if ps.len() >= 2 {
                // A stand line: the stop end is the one nearer a building / farther from the taxilane.
                let (a, b) = (ps[0], *ps.last().unwrap());
                let score = |p: Vec2| facade(c, p, 80.0, false).map_or(200.0, |f| f.0.distance(p)) - c.nearest_taxi(p, 150.0).map_or(150.0, |t| t.0);
                let (stop, from) = if score(a) < score(b) { (a, b) } else { (b, a) };
                let dir = (stop - from).normalize_or_zero();
                if dir == Vec2::ZERO {
                    continue;
                }
                let contact = facade(c, stop, 35.0, true).filter(|f| bridge_ok && f.1.dot(-dir) > 0.5).map(|f| (f.0, f.1));
                // Hub stands may be widebody stands: the spacing to the
                // neighbours (below) decides.
                let span = if c.kind == AKind::Large { 80.0 } else { 36.0 };
                add(&mut out, Stand { name: name.clone(), stop, hdg: mesh::dir_heading(dir), span, contact });
            } else {
                gate_stand(c, ps[0], name, bridge_ok).into_iter().for_each(|s| add(&mut out, s));
            }
        }
    }
    // 2. Gates from the map tiles.
    if out.len() < 3 {
        for (p, name) in &c.gates {
            if let Some(s) = gate_stand(c, *p, name, bridge_ok) {
                add(&mut out, s);
            }
        }
    }
    // 3. Terminal facades facing an apron without stands.
    if matches!(c.kind, AKind::Large | AKind::Medium) {
        let pitch = if c.kind == AKind::Large { 52.0 } else { 42.0 };
        for (bi, b) in c.blds.iter().enumerate() {
            if b.class != BClass::Terminal {
                continue;
            }
            let n = b.ring.len();
            let s = if mesh::ring_area(&b.ring) > 0.0 { 1.0 } else { -1.0 };
            for i in 0..n {
                if b.seam[i] {
                    continue;
                }
                let (a, q) = (b.ring[i], b.ring[(i + 1) % n]);
                let e = q - a;
                let len = e.length();
                if len < pitch + 20.0 {
                    continue;
                }
                let nrm = vec2(e.y, -e.x).normalize() * s;
                let dir = e / len;
                let k = ((len - 30.0) / pitch).floor() as usize;
                let start = (len - (k.saturating_sub(1)) as f32 * pitch) * 0.5;
                for j in 0..k {
                    let x = a + dir * (start + j as f32 * pitch);
                    let stop = x + nrm * 5.0;
                    let mid = stop + nrm * 20.0;
                    let tip = stop + nrm * 40.0;
                    if !c.in_apron(mid) || !c.in_apron(tip) || c.in_building(mid, 4.0) || c.nearest_taxi(mid, 18.0).is_some() {
                        continue;
                    }
                    if out.iter().any(|o| o.stop.distance(stop) < pitch * 0.8) {
                        continue;
                    }
                    let _ = bi;
                    out.push(Stand { name: String::new(), stop, hdg: mesh::dir_heading(-nrm), span: pitch * 0.9, contact: bridge_ok.then_some((x, nrm)) });
                }
            }
        }
    }
    // 4. Remote rows on big aprons with no stands, GA rows on small ones.
    for (ai, a) in c.aprons.iter().enumerate() {
        let area = mesh::ring_area(&a.outer).abs();
        let ga = matches!(c.kind, AKind::Small) || area < 9000.0;
        if area < 1200.0 || out.iter().any(|s| a.contains(s.stop) || a.contains(s.stop + mesh::heading_dir(s.hdg) * -15.0)) {
            continue;
        }
        let (pitch, depth) = if ga { (14.0, 9.0) } else { (48.0, 40.0) };
        // Along the apron's longest edges, nose toward the edge.
        let n = a.outer.len();
        let s = if mesh::ring_area(&a.outer) > 0.0 { 1.0 } else { -1.0 };
        let mut edges: Vec<usize> = (0..n).collect();
        edges.sort_by(|&i, &j| a.outer[(j + 1) % n].distance(a.outer[j]).total_cmp(&a.outer[(i + 1) % n].distance(a.outer[i])));
        let mut placed = 0;
        for &i in edges.iter().take(if ga { 8 } else { 3 }) {
            let (p, q) = (a.outer[i], a.outer[(i + 1) % n]);
            let e = q - p;
            let len = e.length();
            if len < pitch * 2.0 {
                continue;
            }
            let dir = e / len;
            let inward = -vec2(e.y, -e.x).normalize() * s;
            let k = ((len - pitch) / pitch).floor() as usize;
            for j in 0..k {
                let x = p + dir * (pitch * (j as f32 + 1.0));
                let stop = x + inward * 6.0;
                let ctr = stop + inward * (depth * 0.5);
                let corners = [stop + dir * (pitch * 0.4), stop - dir * (pitch * 0.4), stop + inward * depth + dir * (pitch * 0.4), stop + inward * depth - dir * (pitch * 0.4)];
                if !corners.iter().all(|q| a.contains(*q)) || c.in_building(ctr, if ga { 2.0 } else { 6.0 }) || c.nearest_taxi(ctr, if ga { 8.0 } else { 25.0 }).is_some() {
                    continue;
                }
                if out.iter().any(|o| o.stop.distance(stop) < pitch * 0.85) {
                    continue;
                }
                out.push(Stand { name: String::new(), stop, hdg: mesh::dir_heading(-inward), span: pitch * 0.9, contact: None });
                placed += 1;
                if placed > if ga { 14 } else { 40 } {
                    break;
                }
            }
        }
        let _ = ai;
    }
    // 5. Grass fields: aircraft out in front of the hangars.
    if c.kind == AKind::Small {
        for (bi, b) in c.blds.iter().enumerate() {
            if b.class != BClass::Hangar || out.len() > 60 {
                continue;
            }
            let n = b.ring.len();
            let s = if mesh::ring_area(&b.ring) > 0.0 { 1.0 } else { -1.0 };
            // Longest real edge.
            let Some(i) = (0..n).filter(|&i| !b.seam[i]).max_by(|&i, &j| b.ring[(i + 1) % n].distance(b.ring[i]).total_cmp(&b.ring[(j + 1) % n].distance(b.ring[j]))) else {
                continue;
            };
            let (p, q) = (b.ring[i], b.ring[(i + 1) % n]);
            let nrm = vec2((q - p).y, -(q - p).x).normalize() * s;
            let mid = (p + q) * 0.5;
            for (k, off) in [-0.25f32, 0.25].iter().enumerate() {
                let stop = mid + (q - p) * *off + nrm * 14.0;
                if c.in_building(stop, 3.0) || c.runway_at(stop, 25.0).is_some() || out.iter().any(|o| o.stop.distance(stop) < 12.0) {
                    continue;
                }
                if c.rand(bi as u64 * 7 + k as u64) < 0.6 {
                    out.push(Stand { name: String::new(), stop, hdg: mesh::dir_heading(nrm * -1.0) + std::f32::consts::PI, span: 13.0, contact: None });
                }
            }
        }
    }
    // Spacing: the largest span that fits between neighbours.
    let stops: Vec<Vec2> = out.iter().map(|s| s.stop).collect();
    for (i, s) in out.iter_mut().enumerate() {
        let mut nearest = f32::MAX;
        for (j, q) in stops.iter().enumerate() {
            if i != j {
                nearest = nearest.min(q.distance(s.stop));
            }
        }
        s.span = s.span.min(nearest * 0.98 + 2.0);
    }
    out
}

/// A stand at a gate point: nose in to the nearest facade (contact stand,
/// jet bridge) or, out on the apron, nose away from the taxilane.
fn gate_stand(c: &Ctx, g: Vec2, name: &str, bridge_ok: bool) -> Option<Stand> {
    if let Some((x, n, class, _)) = facade(c, g, 70.0, false) {
        let stop = x + n * 5.0;
        // The aircraft must stand on pavement.
        if !c.in_apron(stop + n * 18.0) && c.nearest_taxi(stop + n * 18.0, 30.0).is_none() {
            return None;
        }
        let contact = (bridge_ok && class == BClass::Terminal).then_some((x, n));
        let span = if c.kind == AKind::Large { 80.0 } else { 40.0 };
        return Some(Stand { name: name.to_string(), stop, hdg: mesh::dir_heading(-n), span, contact });
    }
    if !c.in_apron(g) {
        return None;
    }
    let (_, q, _, _) = c.nearest_taxi(g, 120.0)?;
    let dir = (g - q).normalize_or_zero();
    let span = if c.kind == AKind::Large { 80.0 } else { 40.0 };
    (dir != Vec2::ZERO).then(|| Stand { name: name.to_string(), stop: g, hdg: mesh::dir_heading(dir), span, contact: None })
}

// ---------------------------------------------------------------- parked aircraft, bridges, vehicles

fn occupy(c: &Ctx, l: &mut Layout, stands: Vec<Stand>) {
    for (i, s) in stands.iter().enumerate() {
        let k = i as u64 * 31 + 7;
        let occupied = c.rand(k) < if c.kind == AKind::Small { 0.8 } else { 0.72 };
        let ty = pick_type(c, s.span, k + 1);
        let fwd = mesh::heading_dir(s.hdg);
        let left = vec2(fwd.y, -fwd.x);
        let mut bridge_door: Option<Vec3> = None;
        if let (true, Some(t)) = (occupied, ty) {
            let tt = types::get(t);
            let cg = s.stop - fwd * tt.nose.max(2.0);
            l.parked.push(Placed { model: t, at: cg, hdg: s.hdg, livery: (c.hash(k + 2) & 0xff) as u16 });
            if let Some((dz, dx, sill)) = tt.door {
                let d = cg + fwd * dz + left * dx;
                bridge_door = Some(vec3(d.x, sill, d.y));
            }
            // Ground handling around it.
            if tt.role == Role::Jet || tt.role == Role::Turboprop {
                service_vehicles(c, l, cg, fwd, tt, k, s.contact.is_some());
            }
        }
        // Jet bridge: to the door, or parked short when the stand is empty.
        if let Some((x, n)) = s.contact {
            let jet = ty.is_some_and(|t| types::get(t).role == Role::Jet);
            if !jet && occupied {
                continue;
            }
            let root_h = if c.kind == AKind::Large { 5.2 } else { 4.4 };
            let along = vec2(-n.y, n.x);
            let door = match (occupied, bridge_door) {
                (true, Some(d)) => d,
                _ => {
                    let p = x + n * 16.0 + along * 6.0;
                    vec3(p.x, 3.2, p.y)
                }
            };
            // Bridges longer than ~45 m don't reach.
            if vec2(door.x, door.z).distance(x) > 48.0 {
                continue;
            }
            let fus = if occupied { fwd } else { n };
            let mut m = MeshData::default();
            let at = x;
            mesh::jet_bridge(&mut m, Vec2::ZERO, n, root_h, door - vec3(at.x, 0.0, at.y), fus, c.hash(k + 5));
            l.objects.push(Object { at, mesh: m, foot: None });
        }
        l.stands.push(s.clone());
    }
}

fn service_vehicles(c: &Ctx, l: &mut Layout, cg: Vec2, fwd: Vec2, t: &types::Type, k: u64, contact: bool) {
    let left = vec2(fwd.y, -fwd.x);
    let at = |x: f32, z: f32| cg + left * x + fwd * z;
    let hdg = mesh::dir_heading(fwd);
    let r = |j: u64| c.rand(k * 13 + j);
    let put = |l: &mut Layout, v: Vehicle, p: Vec2, h: f32, j: u64| {
        if !c.in_building(p, 1.0) && c.in_aerodrome(p) {
            l.vehicles.push(Placed { model: v.index() as u16, at: p, hdg: h, livery: (c.hash(k * 3 + j) & 0xff) as u16 });
        }
    };
    let s = t.length() / 37.6;
    if r(1) < 0.55 {
        put(l, Vehicle::Tug, at(0.0, t.nose + 4.5), hdg + std::f32::consts::PI, 1);
    }
    if r(2) < 0.45 {
        put(l, Vehicle::Gpu, at(-2.6, t.nose - 3.0 * s), hdg, 2);
    }
    if r(3) < 0.3 {
        put(l, Vehicle::FuelTruck, at(-t.span * 0.28, -1.5 * s), hdg, 3);
    }
    if r(4) < 0.5 {
        // Belt loader at the forward hold, a baggage train behind it.
        let p = at(-3.2, t.nose - 9.0 * s);
        put(l, Vehicle::BeltLoader, p, mesh::dir_heading((left * 0.9 + fwd * 0.4).normalize()), 4);
        let h2 = hdg + std::f32::consts::PI;
        put(l, Vehicle::Tractor, at(-9.5, t.nose - 9.0 * s), h2, 5);
        for j in 0..(2 + (r(6) * 3.0) as u64) {
            put(l, Vehicle::Cart, at(-9.5, t.nose - 9.0 * s + 3.6 * (j + 1) as f32), h2, 6 + j);
        }
    }
    if r(7) < 0.3 {
        put(l, Vehicle::Catering, at(-4.6, t.tail + 8.0 * s), mesh::dir_heading(left), 7);
    }
    if !contact {
        if t.role == Role::Turboprop || r(8) < 0.6 {
            put(l, Vehicle::Stairs, at(4.6, t.nose - 4.5 * s), mesh::dir_heading(-left), 8);
        }
        if r(9) < 0.5 {
            put(l, Vehicle::Bus, at(t.span * 0.45, t.nose - 6.0), hdg, 9);
        }
    }
}

// ---------------------------------------------------------------- markings

const PAINT_YELLOW: [f32; 3] = [0.86, 0.66, 0.10];
const PAINT_BLACK: [f32; 3] = [0.06, 0.06, 0.06];
const PAINT_RED: [f32; 3] = [0.72, 0.10, 0.08];
const PAINT_WHITE: [f32; 3] = [0.84, 0.84, 0.82];

/// Painted ribbon along a polyline (flat, y = 0).
fn ribbon(m: &mut MeshData, pts: &[Vec2], w: f32, col: [f32; 3]) {
    for s in pts.windows(2) {
        let d = (s[1] - s[0]).normalize_or_zero();
        if d == Vec2::ZERO {
            continue;
        }
        let ext = d * (w * 0.5);
        m.ground_rect(s[0] - ext, s[1] + ext, w, 0.0, col);
    }
}

/// Dashed ribbon (dash / gap metres) along a polyline.
fn dashed(m: &mut MeshData, pts: &[Vec2], w: f32, col: [f32; 3], dash: f32, gap: f32) {
    let period = dash + gap;
    let mut acc = 0.0f32;
    for s in pts.windows(2) {
        let len = s[0].distance(s[1]);
        if len < 1e-3 {
            continue;
        }
        let d = (s[1] - s[0]) / len;
        // Dashes start at multiples of the period along the whole line.
        let mut k = (acc / period).floor();
        loop {
            let a = (k * period - acc).max(0.0);
            let b = (k * period + dash - acc).min(len);
            if k * period - acc >= len {
                break;
            }
            if b > a {
                m.ground_rect(s[0] + d * a, s[0] + d * b, w, 0.0, col);
            }
            k += 1.0;
        }
        acc += len;
    }
}

/// Painted text on the ground: baseline centred at `at`, reading along
/// `right`, letters' up = `up` (unit vectors), cap height `h`.
fn paint_text(m: &mut MeshData, text: &str, at: Vec2, right: Vec2, up: Vec2, h: f32, col: [f32; 3]) -> f32 {
    let (qs, w) = super::font::quads(text, h, 1.2);
    let o = at - right * (w * 0.5);
    for q in qs {
        let p = |v: Vec2| o + right * v.x + up * v.y;
        m.flat_quad([p(q[0]), p(q[1]), p(q[2]), p(q[3])], 0.0, col);
    }
    w
}

/// Split a polyline into the runs where `keep(point)` holds (sampled every `step` m).
fn runs(pts: &[Vec2], step: f32, keep: &dyn Fn(Vec2) -> bool) -> Vec<Vec<Vec2>> {
    let mut out = Vec::new();
    let mut cur: Vec<Vec2> = Vec::new();
    for s in pts.windows(2) {
        let len = s[0].distance(s[1]);
        let k = ((len / step).ceil() as usize).max(1);
        for i in 0..k {
            let p = s[0].lerp(s[1], i as f32 / k as f32);
            if keep(p) {
                cur.push(p);
            } else if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            } else {
                cur.clear();
            }
        }
    }
    if let Some(p) = pts.last() {
        if keep(*p) {
            cur.push(*p);
        }
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

/// Offset a polyline sideways by `off` (left positive).
fn offset(pts: &[Vec2], off: f32) -> Vec<Vec2> {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let d0 = if i > 0 { (pts[i] - pts[i - 1]).normalize_or_zero() } else { Vec2::ZERO };
            let d1 = if i + 1 < n { (pts[i + 1] - pts[i]).normalize_or_zero() } else { Vec2::ZERO };
            let d = (d0 + d1).normalize_or(d0.max(d1));
            pts[i] + vec2(d.y, -d.x) * off
        })
        .collect()
}

fn marks(c: &Ctx, l: &mut Layout) {
    let mut m = MeshData::default();
    m.mat = mat::DIFFUSE;
    let big = matches!(c.kind, AKind::Large | AKind::Medium);
    let paved = c.runways.iter().any(|r| r.paved) || !c.aprons.is_empty();
    if !paved {
        l.marks = m;
        return;
    }
    let off_runway = |p: Vec2| c.runway_at(p, 2.0).is_none();
    // Taxiway centrelines (black-edged at the big airports, for contrast on concrete).
    let centre: Vec<Vec<Vec2>> = c.taxis.iter().flat_map(|t| runs(&t.pts, 4.0, &off_runway)).collect();
    if big {
        for r in &centre {
            ribbon(&mut m, r, 0.95, PAINT_BLACK);
        }
    }
    for r in &centre {
        ribbon(&mut m, r, if big { 0.4 } else { 0.3 }, PAINT_YELLOW);
    }
    // Taxiway edge lines (double yellow) where the pavement ends: not on aprons,
    // not at junctions, not on runways.
    if big {
        let hw = c.taxi_w * 0.5 - 0.6;
        for (ti, t) in c.taxis.iter().enumerate() {
            for s in [-1.0f32, 1.0] {
                let e0 = offset(&t.pts, s * hw);
                let e1 = offset(&t.pts, s * (hw - 0.45));
                let keep = |p: Vec2| {
                    !c.in_apron(p)
                        && c.runway_at(p, 15.0).is_none()
                        && !c.tgrid.near(p, c.taxi_w).iter().any(|&(oi, si)| {
                            let o = &c.taxis[oi as usize];
                            oi as usize != ti && mesh::seg_dist(p, o.pts[si as usize], o.pts[si as usize + 1]) < c.taxi_w * 0.75
                        })
                };
                for e in [e0, e1] {
                    for r in runs(&e, 3.0, &keep) {
                        ribbon(&mut m, &r, 0.18, PAINT_YELLOW);
                    }
                }
            }
        }
    }
    // Runway-holding positions: pattern A (two solid lines on the taxiway
    // side, two dashed toward the runway), enhanced centrelines before them.
    for h in &l.holds {
        let across = vec2(-h.dir.y, h.dir.x);
        let hw = h.width * 0.5;
        let lw = 0.3;
        if big {
            m.ground_rect(h.at - h.dir * 2.4 - across * hw, h.at + h.dir * 0.6 - across * hw, 0.01, 0.0, PAINT_BLACK);
            m.flat_quad([h.at - h.dir * 2.3 - across * hw, h.at - h.dir * 2.3 + across * hw, h.at + h.dir * 0.5 + across * hw, h.at + h.dir * 0.5 - across * hw], 0.0, PAINT_BLACK);
        }
        for (k, off) in [-1.95f32, -1.35, -0.75, -0.15].iter().enumerate() {
            let a = h.at + h.dir * *off - across * hw;
            let b = h.at + h.dir * *off + across * hw;
            if k < 2 {
                m.ground_rect(a, b, lw, 0.0, PAINT_YELLOW);
            } else {
                dashed(&mut m, &[a, b], lw, PAINT_YELLOW, 0.9, 0.9);
            }
        }
        if c.kind == AKind::Large {
            for s in [-1.0f32, 1.0] {
                let a = h.at - h.dir * 2.5 + across * (s * 0.55);
                let b = h.at - h.dir * 47.5 + across * (s * 0.55);
                dashed(&mut m, &[a, b], 0.15, PAINT_YELLOW, 3.0, 1.0);
            }
        }
    }
    // Stands: lead-in line from the taxilane, stop bar, number, restraint box.
    for (i, s) in l.stands.iter().enumerate() {
        let fwd = mesh::heading_dir(s.hdg);
        let left = vec2(fwd.y, -fwd.x);
        if c.kind == AKind::Small {
            continue;
        }
        // Back along the stand centreline to the taxilane.
        let mut tail = s.stop - fwd * 60.0;
        for k in 1..=60 {
            let p = s.stop - fwd * (k as f32 * 2.5);
            if c.nearest_taxi(p, 1.5).is_some() {
                tail = p;
                break;
            }
            if k as f32 * 2.5 > 140.0 {
                break;
            }
        }
        let line = [tail, s.stop];
        ribbon(&mut m, &line, 0.9, PAINT_BLACK);
        ribbon(&mut m, &line, 0.35, PAINT_YELLOW);
        // Stop bar (with a short T for the nose wheel).
        m.ground_rect(s.stop - left * 2.2, s.stop + left * 2.2, 0.9, 0.0, PAINT_BLACK);
        m.ground_rect(s.stop - left * 2.0, s.stop + left * 2.0, 0.4, 0.0, PAINT_YELLOW);
        // Stand number on a black box near the start of the lead-in, readable when taxiing in.
        let name = if s.name.is_empty() { format!("{}", i + 1) } else { s.name.chars().take(4).collect() };
        let np = s.stop - fwd * ((s.stop.distance(tail) * 0.55).clamp(10.0, 40.0)) + left * 0.0;
        let up = fwd;
        let right = vec2(-up.y, up.x) * -1.0;
        let right = vec2(right.x, right.y);
        let w = super::font::width(&name) * 1.6 / 6.0;
        let pad = 0.6;
        let ctr = np + up * 0.8;
        let hx = w * 0.5 + pad;
        let hy = 0.8 + pad;
        m.flat_quad([ctr - right * hx - up * hy, ctr + right * hx - up * hy, ctr + right * hx + up * hy, ctr - right * hx + up * hy], 0.0, PAINT_BLACK);
        paint_text(&mut m, &name, np, right, up, 1.6, PAINT_YELLOW);
        // Equipment restraint line: red box clear of the wingtips / tail.
        if s.span > 25.0 {
            let half = s.span * 0.5 + 4.0;
            let a = s.stop + fwd * 4.0;
            let b = s.stop - fwd * (s.span * 1.15 + 6.0).min(55.0);
            let corners = [a + left * half, a - left * half, b - left * half, b + left * half];
            for k in 0..4 {
                let (p, q) = (corners[k], corners[(k + 1) % 4]);
                if k == 2 {
                    continue; // open toward the taxilane
                }
                dashed(&mut m, &[p, q], 0.2, PAINT_RED, 1.5, 0.6);
            }
        }
    }
    // Apron roads: white edge lines where service roads cross the aprons.
    if big {
        for s in &c.service {
            for side in [-1.0f32, 1.0] {
                let e = offset(s, side * 3.6);
                for r in runs(&e, 3.0, &|p| c.in_apron(p) && c.nearest_taxi(p, 12.0).is_none()) {
                    ribbon(&mut m, &r, 0.15, PAINT_WHITE);
                }
            }
            for r in runs(s, 3.0, &|p| c.in_apron(p) && c.nearest_taxi(p, 12.0).is_none()) {
                dashed(&mut m, &r, 0.12, PAINT_WHITE, 3.0, 3.0);
            }
        }
    }
    l.marks = m;
}

// ---------------------------------------------------------------- signs

fn arrow_for(turn: f32) -> char {
    let d = turn.to_degrees();
    if d > 120.0 {
        '\u{2199}'
    } else if d > 60.0 {
        '\u{2190}'
    } else if d > 20.0 {
        '\u{2196}'
    } else if d > -20.0 {
        '\u{2191}'
    } else if d > -60.0 {
        '\u{2197}'
    } else if d > -120.0 {
        '\u{2192}'
    } else {
        '\u{2198}'
    }
}

fn signs(c: &Ctx, l: &mut Layout) {
    if !matches!(c.kind, AKind::Large | AKind::Medium) && !c.runways.iter().any(|r| r.paved && r.lighted) {
        return;
    }
    let size = if c.kind == AKind::Large { 0.9 } else { 0.7 };
    let mut placed: Vec<Vec2> = Vec::new();
    let mut put = |l: &mut Layout, at: Vec2, facing: Vec2, panels: Vec<(Panel, String)>, back: bool| {
        if placed.iter().any(|p| p.distance(at) < 12.0) || c.in_building(at, 3.0) || c.runway_at(at, 30.0).is_some() || c.nearest_taxi(at, c.taxi_w * 0.5 + 3.0).is_some() {
            return;
        }
        placed.push(at);
        let m = mesh::sign(&panels, size, back);
        // Local +Z (the readable face) toward the approaching aircraft.
        let mut g = MeshData::default();
        g.append(&m, glam::Mat4::from_quat(mesh::heading_quat(mesh::dir_heading(facing))));
        l.objects.push(Object { at, mesh: g, foot: None });
    };
    // Runway holding position signs: mandatory (left end first) + location.
    for h in l.holds.clone() {
        let left = vec2(h.dir.y, -h.dir.x);
        let Some(rw) = c.runways.iter().find(|r| r.name == h.runway) else { continue };
        let (ia, ib) = (rw.ident_a.clone(), rw.ident_b.clone());
        let a_left = (rw.a - h.at).dot(left) > (rw.b - h.at).dot(left);
        let text = if ia.is_empty() {
            h.runway.clone()
        } else if a_left {
            format!("{ia}-{ib}")
        } else {
            format!("{ib}-{ia}")
        };
        let mut panels = vec![(Panel::Mandatory, text)];
        if !h.taxiway.is_empty() && h.taxiway.len() <= 4 {
            panels.push((Panel::Location, h.taxiway.clone()));
        }
        let at = h.at - h.dir * 1.0 + left * (h.width * 0.5 + 9.0);
        put(l, at, -h.dir, panels.clone(), true);
        if c.kind == AKind::Large {
            put(l, h.at - h.dir * 1.0 - left * (h.width * 0.5 + 9.0), -h.dir, panels, true);
        }
    }
    if !matches!(c.kind, AKind::Large | AKind::Medium) {
        return;
    }
    // Junctions: where taxiways with different refs share a node.
    let mut nodes: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (ti, t) in c.taxis.iter().enumerate() {
        if t.name.is_empty() || t.name.len() > 4 {
            continue;
        }
        for (pi, p) in t.pts.iter().enumerate() {
            nodes.entry(((p.x / 2.0).round() as i32, (p.y / 2.0).round() as i32)).or_default().push((ti, pi));
        }
    }
    let mut count = 0;
    let mut keys: Vec<&(i32, i32)> = nodes.keys().collect();
    keys.sort();
    for key in keys {
        let v = &nodes[key];
        let mut names: Vec<&str> = v.iter().map(|(ti, _)| c.taxis[*ti].name.as_str()).collect();
        names.sort();
        names.dedup();
        if names.len() < 2 || count > 400 {
            continue;
        }
        let j = c.taxis[v[0].0].pts[v[0].1];
        if c.in_apron(j) || c.runway_at(j, 60.0).is_some() {
            continue;
        }
        // Legs out of the junction: (taxiway, direction).
        let mut legs: Vec<(String, Vec2)> = Vec::new();
        for (ti, pi) in v {
            let t = &c.taxis[*ti];
            for step in [-1i32, 1] {
                let k = *pi as i32 + step;
                if k < 0 || k as usize >= t.pts.len() {
                    continue;
                }
                let d = (t.pts[k as usize] - j).normalize_or_zero();
                if d != Vec2::ZERO && !legs.iter().any(|(n, dd)| *n == t.name && dd.dot(d) > 0.95) {
                    legs.push((t.name.clone(), d));
                }
            }
        }
        // A sign on the left of every approach, ~30 m before the junction.
        for (name, d_out) in &legs {
            let approach = -*d_out; // travelling toward the junction
            let left = vec2(approach.y, -approach.x);
            let at = j - approach * 30.0 + left * (c.taxi_w * 0.5 + 8.0);
            let mut dirs: Vec<(f32, String)> = legs
                .iter()
                .filter(|(n, _)| n != name)
                .map(|(n, d)| {
                    let turn = approach.perp_dot(*d) * -1.0;
                    let ang = turn.atan2(approach.dot(*d));
                    (ang, format!("{n}{}", arrow_for(ang)))
                })
                .collect();
            if dirs.is_empty() {
                continue;
            }
            dirs.sort_by(|a, b| b.0.total_cmp(&a.0));
            dirs.dedup_by(|a, b| a.1 == b.1);
            let mut panels: Vec<(Panel, String)> = dirs.iter().filter(|d| d.0 > 0.0).map(|d| (Panel::Direction, d.1.clone())).collect();
            panels.push((Panel::Location, name.clone()));
            panels.extend(dirs.iter().filter(|d| d.0 <= 0.0).map(|d| (Panel::Direction, d.1.clone())));
            panels.truncate(5);
            put(l, at, approach * -1.0, panels, false);
            count += 1;
        }
    }
}

// ---------------------------------------------------------------- lights, masts

fn lights_and_masts(c: &Ctx, l: &mut Layout) {
    let lit = c.runways.iter().any(|r| r.lighted);
    if !lit {
        return;
    }
    let (centre, edge) = match c.kind {
        AKind::Large => (Some(15.0), None),
        AKind::Medium => (Some(30.0), Some(50.0)),
        _ => (None, Some(60.0)),
    };
    for t in &c.taxis {
        let keep = |p: Vec2| !c.in_apron(p) && c.runway_at(p, 3.0).is_none();
        if let Some(sp) = centre {
            for r in runs(&t.pts, sp, &keep) {
                for p in &r {
                    l.lights.push(LightInst::new(vec3(p.x, 0.12, p.y), 60.0, colour::TAXI_GREEN, kind::TAXI));
                }
            }
        }
        if let Some(sp) = edge {
            for s in [-1.0f32, 1.0] {
                let e = offset(&t.pts, s * (c.taxi_w * 0.5 + 1.0));
                for r in runs(&e, sp, &|p| keep(p) && c.nearest_taxi(p, c.taxi_w * 0.5 - 0.5).is_none()) {
                    for p in &r {
                        l.lights.push(LightInst::new(vec3(p.x, 0.35, p.y), 40.0, colour::TAXI_BLUE, kind::TAXI));
                    }
                }
            }
        }
    }
    // Stop bars (red, toward the aircraft) and runway guard lights at the holds.
    if matches!(c.kind, AKind::Large | AKind::Medium) {
        for h in &l.holds {
            let across = vec2(-h.dir.y, h.dir.x);
            let n = (h.width / 3.0) as i32;
            for k in 0..=n {
                let p = h.at - h.dir * 0.3 + across * (-h.width * 0.5 + 3.0 * k as f32);
                l.lights.push(LightInst::new(vec3(p.x, 0.12, p.y), 120.0, colour::RWY_RED, kind::UNIDIR).with_dir(vec3(-h.dir.x, 0.05, -h.dir.y), 0));
            }
            for s in [-1.0f32, 1.0] {
                let p = h.at - h.dir * 0.5 + across * (s * (h.width * 0.5 + 3.0));
                l.lights.push(
                    LightInst::new(vec3(p.x, 0.6, p.y), 300.0, colour::RWY_YELLOW, kind::FLASHER)
                        .with_dir(vec3(-h.dir.x, 0.1, -h.dir.y), 0)
                        .with_ab(if s > 0.0 { 0.0 } else { 0.5 }, 0.0),
                );
            }
        }
    }
    // Apron floodlight masts around the aprons' airside edges.
    if c.kind == AKind::Small {
        return;
    }
    let (pitch, h) = if c.kind == AKind::Large { (120.0, 30.0) } else { (140.0, 24.0) };
    let mut masts: Vec<Vec2> = Vec::new();
    for a in &c.aprons {
        let area = mesh::ring_area(&a.outer).abs();
        if area < 6000.0 {
            continue;
        }
        let ctr = a.outer.iter().copied().sum::<Vec2>() / a.outer.len() as f32;
        let n = a.outer.len();
        let s = if mesh::ring_area(&a.outer) > 0.0 { 1.0 } else { -1.0 };
        let mut carry = pitch * 0.5;
        for i in 0..n {
            let (p, q) = (a.outer[i], a.outer[(i + 1) % n]);
            let e = q - p;
            let len = e.length();
            let out = vec2(e.y, -e.x).normalize_or_zero() * s;
            let mut t = carry;
            while t < len {
                let x = p + e * (t / len) + out * 7.0;
                let ok = !c.in_building(x, 4.0)
                    && !c.in_apron(x)
                    && c.nearest_taxi(x, c.taxi_w * 0.5 + 12.0).is_none()
                    && c.runway_at(x, 120.0).is_none()
                    && !masts.iter().any(|m| m.distance(x) < pitch * 0.7);
                if ok {
                    masts.push(x);
                    let face = (ctr - x).normalize_or(-out);
                    let (mm, lamp) = mesh::flood_mast(h, face);
                    l.objects.push(Object { at: x, mesh: mm, foot: None });
                    l.lights.push(LightInst::new(vec3(x.x + lamp.x, lamp.y, x.y + lamp.z), 9000.0, colour::FLOOD, kind::FLOOD).with_ab(h * h * 28.0, h));
                }
                t += pitch;
            }
            carry = t - len;
        }
    }
}

// ---------------------------------------------------------------- windsocks, tower

fn windsocks(c: &Ctx, l: &mut Layout, osm: Option<&OsmAirport>, osm_xy: &dyn Fn((f64, f64)) -> Vec2) {
    let mut socks: Vec<Vec2> = osm.map(|o| o.windsocks.iter().map(|p| osm_xy(*p)).filter(|p| c.in_aerodrome(*p) || c.runway_at(*p, 400.0).is_some()).collect()).unwrap_or_default();
    if socks.is_empty() {
        for (i, rw) in c.runways.iter().enumerate() {
            let len = rw.length();
            let d = (rw.b - rw.a) / len;
            let left = vec2(d.y, -d.x);
            let ends: &[(Vec2, Vec2)] = &[(rw.a, d), (rw.b, -d)];
            let per = if c.kind == AKind::Small { 1 } else { 2 };
            for (k, (thr, dir)) in ends.iter().take(per).enumerate() {
                let along = (len * 0.18).clamp(120.0, 320.0);
                for side in [1.0f32, -1.0] {
                    let lat = rw.width * 0.5 + if c.kind == AKind::Small { 40.0 } else { 70.0 };
                    let lv = vec2(dir.y, -dir.x) * side;
                    let p = *thr + *dir * along + lv * lat;
                    let free = !c.in_building(p, 15.0) && !c.in_apron(p) && c.nearest_taxi(p, 25.0).is_none() && c.runway_at(p, 20.0).is_none();
                    if free && !socks.iter().any(|s| s.distance(p) < 300.0) {
                        socks.push(p);
                        break;
                    }
                }
                let _ = (i, k, left);
            }
        }
    }
    for s in &socks {
        l.objects.push(Object { at: *s, mesh: mesh::windsock_mast(), foot: None });
        // Lit at night (a small flood on the sock).
        l.lights.push(LightInst::new(vec3(s.x, mesh::WINDSOCK_H + 0.8, s.y), 300.0, colour::LED_WARM, kind::OMNI));
    }
    l.socks = socks;
}

fn tower(c: &Ctx, l: &mut Layout, osm: Option<&OsmAirport>, osm_xy: &dyn Fn((f64, f64)) -> Vec2) {
    if c.blds.iter().any(|b| b.class == BClass::Tower) || !matches!(c.kind, AKind::Large | AKind::Medium) {
        return;
    }
    let h = if c.kind == AKind::Large { 58.0 } else { 30.0 };
    let free = |p: Vec2| !c.in_building(p, 6.0) && !c.in_apron(p) && c.nearest_taxi(p, 30.0).is_none() && c.runway_at(p, 150.0).is_none() && c.in_aerodrome(p);
    let mut at = osm.and_then(|o| o.towers.iter().map(|t| (osm_xy(t.0), t.1)).find(|(p, _)| free(*p)));
    if at.is_none() {
        // Beside the aprons, with a view of the runways' middle.
        let mid = c.runways.iter().map(|r| (r.a + r.b) * 0.5).sum::<Vec2>() / c.runways.len().max(1) as f32;
        let mut best: Option<(f32, Vec2)> = None;
        for a in &c.aprons {
            let n = a.outer.len();
            let s = if mesh::ring_area(&a.outer) > 0.0 { 1.0 } else { -1.0 };
            for i in 0..n {
                let (p, q) = (a.outer[i], a.outer[(i + 1) % n]);
                let out = vec2((q - p).y, -(q - p).x).normalize_or_zero() * s;
                let x = (p + q) * 0.5 + out * 28.0;
                let dr = c.runways.iter().map(|r| mesh::seg_dist(x, r.a, r.b)).fold(f32::MAX, f32::min);
                if !(220.0..1100.0).contains(&dr) || !free(x) {
                    continue;
                }
                let score = x.distance(mid);
                if best.is_none_or(|b| score < b.0) {
                    best = Some((score, x));
                }
            }
        }
        at = best.map(|b| (b.1, None));
    }
    if let Some((p, th)) = at {
        let height = th.unwrap_or(h).clamp(15.0, 90.0);
        let (m, top) = mesh::control_tower(height, (height * 0.07).clamp(3.0, 5.5), c.seed);
        l.lights.push(LightInst::new(vec3(p.x + top.x, top.y, p.y + top.z), 2000.0, colour::OBST_RED, kind::OBSTACLE));
        l.objects.push(Object { at: p, mesh: m, foot: None });
    }
}

// ---------------------------------------------------------------- fire station, fuel farm

/// A free spot for a facility of radius `r`: off the pavement, buildings
/// and runway strips, inside the aerodrome; candidates at lateral offsets
/// `lat` (m) from a runway, along its middle half.
fn free_spot(c: &Ctx, l: &Layout, r: f32, lat: (f32, f32), k: u64) -> Option<(Vec2, Vec2)> {
    for (ri, rw) in c.runways.iter().enumerate() {
        let len = rw.length();
        let d = (rw.b - rw.a) / len;
        let n = vec2(-d.y, d.x);
        for step in 0..48u64 {
            let along = len * (0.25 + 0.5 * c.rand(k + step * 7 + ri as u64));
            let side = if c.rand(k + step * 11) < 0.5 { -1.0 } else { 1.0 };
            let off = lat.0 + (lat.1 - lat.0) * c.rand(k + step * 13);
            let p = rw.a + d * along + n * (side * off);
            let ok = c.in_aerodrome(p)
                && !c.in_building(p, r)
                && !c.aprons.iter().any(|a| a.contains(p) || super::tile::ring_dist(&a.outer, p) < r)
                && c.nearest_taxi(p, r + c.taxi_w).is_none()
                && c.runway_at(p, r + 120.0).is_none()
                && !l.objects.iter().any(|o| o.at.distance(p) < r + 30.0)
                && !l.socks.iter().any(|s| s.distance(p) < r + 20.0);
            if ok {
                // Facing the runway.
                return Some((p, -n * side));
            }
        }
    }
    None
}

fn services(c: &Ctx, l: &mut Layout) {
    if !matches!(c.kind, AKind::Large | AKind::Medium) || c.runways.is_empty() {
        return;
    }
    if let Some((p, face)) = free_spot(c, l, 40.0, (220.0, 520.0), 0xf12e) {
        let mut m = MeshData::default();
        m.append(&mesh::fire_station(c.seed), glam::Mat4::from_quat(mesh::heading_quat(mesh::dir_heading(face))));
        l.objects.push(Object { at: p, mesh: m, foot: None });
        let left = vec2(face.y, -face.x);
        for i in 0..2 {
            let v = p + face * 22.0 + left * (-8.0 + 16.0 * i as f32);
            l.vehicles.push(Placed { model: Vehicle::FireTruck.index() as u16, at: v, hdg: mesh::dir_heading(face), livery: 0 });
        }
    }
    if c.kind == AKind::Large {
        if let Some((p, face)) = free_spot(c, l, 60.0, (500.0, 1100.0), 0xf0e1) {
            let mut m = MeshData::default();
            m.append(&mesh::fuel_farm(c.seed), glam::Mat4::from_quat(mesh::heading_quat(mesh::dir_heading(face))));
            l.objects.push(Object { at: p, mesh: m, foot: None });
        }
    }
}

// ---------------------------------------------------------------- moving vehicles

fn routes(c: &Ctx, l: &mut Layout) {
    if c.kind == AKind::Small {
        return;
    }
    let kinds = [Vehicle::Tractor, Vehicle::Car, Vehicle::Bus, Vehicle::FuelTruck, Vehicle::Catering, Vehicle::Tractor, Vehicle::Car];
    let mut total = 0;
    for (i, s) in c.service.iter().enumerate() {
        let len: f32 = s.windows(2).map(|w| w[0].distance(w[1])).sum();
        if len < 120.0 || !s.iter().any(|p| c.in_aerodrome(*p)) {
            continue;
        }
        let n = ((len / 380.0) as usize).clamp(1, 3);
        let mut v = Vec::new();
        for k in 0..n {
            let r = c.rand(i as u64 * 17 + k as u64);
            let kind = kinds[(c.hash(i as u64 * 5 + k as u64) % kinds.len() as u64) as usize];
            v.push((kind.index() as u16, r * len * 2.0, 6.0 + 5.0 * c.rand(i as u64 * 3 + k as u64 + 99)));
        }
        total += n;
        l.routes.push(Route { pts: s.clone(), len, vehicles: v });
        if total > 160 {
            break;
        }
    }
}

impl Route {
    /// Position and heading of a vehicle `phase` m along a back-and-forth trip at time `t`.
    pub fn at(&self, phase: f32, speed: f32, t: f32) -> (Vec2, f32) {
        let s = (phase + speed * t).rem_euclid(self.len * 2.0);
        let (s, back) = if s > self.len { (self.len * 2.0 - s, true) } else { (s, false) };
        let mut acc = 0.0;
        for w in self.pts.windows(2) {
            let d = w[0].distance(w[1]);
            if acc + d >= s || d == 0.0 {
                let k = if d > 0.0 { (s - acc) / d } else { 0.0 };
                let p = w[0].lerp(w[1], k.clamp(0.0, 1.0));
                let mut dir = (w[1] - w[0]).normalize_or(Vec2::X);
                if back {
                    dir = -dir;
                }
                // Keep right of the road centre.
                let right = vec2(-dir.y, dir.x);
                return (p + right * 2.2, mesh::dir_heading(dir));
            }
            acc += d;
        }
        (*self.pts.last().unwrap(), 0.0)
    }
}

// ---------------------------------------------------------------- bake

/// A layout placed on one tile's terrain (tile frame).
pub struct Bake {
    pub id: TileId,
    /// Ground markings draped on the terrain (drawn like the tile's layers).
    pub marks: MeshData,
    /// Static objects (jet bridges, signs, masts, windsock masts, tower).
    pub solids: MeshData,
    pub lights: Vec<LightInst>,
    /// Parked aircraft / vehicles: model, position (tile frame, on the ground), heading.
    pub parked: Vec<(u16, Vec3, f32, u16)>,
    pub vehicles: Vec<(u16, Vec3, f32, u16)>,
    /// Windsock swivel points.
    pub socks: Vec<Vec3>,
    pub max_y: f32,
    pub min_y: f32,
}

/// Clip a convex polygon to a rectangle.
fn clip_rect(poly: &[Vec2], lo: Vec2, hi: Vec2) -> Vec<Vec2> {
    let mut cur = poly.to_vec();
    for (axis, bound, keep_ge) in [(0, lo.x, true), (0, hi.x, false), (1, lo.y, true), (1, hi.y, false)] {
        if cur.is_empty() {
            break;
        }
        let inside = |p: Vec2| {
            let v = if axis == 0 { p.x } else { p.y };
            if keep_ge { v >= bound } else { v <= bound }
        };
        let input = std::mem::take(&mut cur);
        let n = input.len();
        for i in 0..n {
            let (p, q) = (input[i], input[(i + 1) % n]);
            let (ip, iq) = (inside(p), inside(q));
            if ip {
                cur.push(p);
            }
            if ip != iq {
                let (vp, vq) = if axis == 0 { (p.x, q.x) } else { (p.y, q.y) };
                let t = (bound - vp) / (vq - vp);
                cur.push(p + (q - p) * t);
            }
        }
    }
    cur
}

/// Place a layout on the terrain of tile `id` (its grid).
pub fn bake(l: &Layout, id: TileId, grid: &crate::terrain::Grid) -> Bake {
    let tf = Proj::tile_frame(id);
    let (lo, hi) = id.rect(&tf);
    let to_tile = l.frame.map_to(&tf);
    let tp = |p: Vec2| to_tile.apply(p);
    let inside = |p: Vec2| p.x >= lo.x && p.x < hi.x && p.y >= lo.y && p.y < hi.y;
    // Marks: clip every triangle to the tile, drape onto the terrain.
    let mut src = MeshData::default();
    for t in l.marks.idx.chunks_exact(3) {
        let vs = [l.marks.verts[t[0] as usize], l.marks.verts[t[1] as usize], l.marks.verts[t[2] as usize]];
        let p: Vec<Vec2> = vs.iter().map(|v| tp(vec2(v.pos[0], v.pos[2]))).collect();
        let (blo, bhi) = (p[0].min(p[1]).min(p[2]), p[0].max(p[1]).max(p[2]));
        if bhi.x < lo.x || blo.x > hi.x || bhi.y < lo.y || blo.y > hi.y {
            continue;
        }
        let poly = if blo.x >= lo.x && bhi.x <= hi.x && blo.y >= lo.y && bhi.y <= hi.y { p } else { clip_rect(&p, lo, hi) };
        if poly.len() < 3 {
            continue;
        }
        src.mat = vs[0].mat;
        let base = src.verts.len() as u32;
        for q in &poly {
            src.v_uv(vec3(q.x, 0.0, q.y), Vec3::Y, vs[0].col, vs[0].uv);
        }
        for k in 1..poly.len() as u32 - 1 {
            src.idx.extend([base, base + k, base + k + 1]);
        }
    }
    let mut marks = MeshData::default();
    if !src.idx.is_empty() {
        // (drape hands out the indices of each piece; collect them in order.)
        let mut idx = Vec::new();
        crate::terrain::drape(&src, grid, &mut marks, |_, _, i| idx.extend_from_slice(i));
        marks.idx = idx;
    }
    // Objects anchored in this tile.
    let mut solids = MeshData::default();
    let (mut min_y, mut max_y) = (grid.min, grid.max);
    for o in &l.objects {
        let at = tp(o.at);
        if !inside(at) {
            continue;
        }
        let base = match &o.foot {
            Some(f) => grid.min_over(&f.iter().map(|p| tp(*p)).collect::<Vec<_>>()),
            None => grid.height(at),
        };
        let v0 = solids.verts.len();
        solids.append(&o.mesh, glam::Mat4::from_translation(vec3(at.x, base, at.y)));
        for v in &mut solids.verts[v0..] {
            // (x of the airport frame scaled into the tile frame: ≤ 1e-3 relative.)
            v.pos[0] = at.x + (v.pos[0] - at.x) * to_tile.sx as f32;
            max_y = max_y.max(v.pos[1]);
            min_y = min_y.min(v.pos[1]);
        }
    }
    let lights = l
        .lights
        .iter()
        .filter_map(|li| {
            let p = tp(vec2(li.pos[0], li.pos[2]));
            inside(p).then(|| {
                let mut li = *li;
                li.pos = [p.x, li.pos[1] + grid.height(p), p.y];
                li
            })
        })
        .collect();
    let place = |v: &[Placed]| -> Vec<(u16, Vec3, f32, u16)> {
        v.iter()
            .filter_map(|p| {
                let at = tp(p.at);
                inside(at).then(|| (p.model, vec3(at.x, grid.height(at), at.y), p.hdg, p.livery))
            })
            .collect()
    };
    let socks = l.socks.iter().map(|s| tp(*s)).filter(|s| inside(*s)).map(|s| vec3(s.x, grid.height(s) + mesh::WINDSOCK_H, s.y)).collect();
    Bake { id, marks, solids, lights, parked: place(&l.parked), vehicles: place(&l.vehicles), socks, max_y: max_y + 1.0, min_y: min_y - 1.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airport::tile::{AeroBuilding, AeroPiece, Path, Poly};
    use crate::airports::{End, Rwy, Surface};

    #[test]
    fn military_fields_by_name() {
        for n in ["RAF Brize Norton", "Ramstein Air Base", "Dover Air Force Base", "NAS Whidbey Island Naval Air Station", "RNAS Yeovilton"] {
            assert!(is_military(n), "{n}");
        }
        for n in ["London Heathrow Airport", "Rafina Heliport", "Baseline Airfield", "London City Airport"] {
            assert!(!is_military(n), "{n}");
        }
    }

    /// A synthetic airport inside one z14 tile: an E-W runway, a parallel
    /// taxiway with a link crossing the holding position, an apron with a
    /// terminal and gates north of it.
    fn synthetic() -> (Airport, Vec<(TileId, Arc<AeroTile>)>, TileId) {
        let id = TileId::new(8193, 5448);
        let tf = Proj::tile_frame(id);
        let (lo, hi) = id.rect(&tf);
        let c = (lo + hi) * 0.5;
        let ll = |p: Vec2| tf.to_latlon(p);
        let (ra, rb) = (c + vec2(-600.0, 200.0), c + vec2(600.0, 200.0));
        let end = |p: Vec2, ident: &str| {
            let (lat, lon) = ll(p);
            End { ident: ident.into(), lat: Some(lat), lon: Some(lon), hdg_true: None, displaced_m: 0.0 }
        };
        let (lat, lon) = ll(c);
        let ap = Airport {
            ident: "TEST".into(),
            kind: AKind::Medium,
            name: "Test".into(),
            lat,
            lon,
            elev_ft: 0.0,
            country: String::new(),
            region: String::new(),
            city: String::new(),
            scheduled: true,
            icao: "TEST".into(),
            iata: String::new(),
            runways: vec![Rwy { length_m: 1200.0, width_m: 45.0, surface: Surface::Asphalt, lighted: true, ends: [end(ra, "09"), end(rb, "27")] }],
        };
        let sq = |c: Vec2, h: Vec2| vec![c + vec2(-h.x, -h.y), c + vec2(h.x, -h.y), c + vec2(h.x, h.y), c + vec2(-h.x, h.y)];
        let t = AeroTile {
            id,
            pieces: vec![AeroPiece { poly: Poly { outer: sq(c, vec2(700.0, 500.0)), holes: vec![] }, owner: "TEST".into() }],
            aprons: vec![Poly { outer: sq(c + vec2(0.0, -150.0), vec2(250.0, 60.0)), holes: vec![] }],
            taxiways: vec![
                Path { pts: vec![c + vec2(-500.0, 50.0), c + vec2(500.0, 50.0)], cut: [false; 2], name: "A".into() },
                Path { pts: vec![c + vec2(0.0, 50.0), c + vec2(0.0, 200.0)], cut: [false; 2], name: "B".into() },
                Path { pts: vec![c + vec2(-300.0, 50.0), c + vec2(-300.0, -90.0)], cut: [false; 2], name: "C".into() },
            ],
            gates: vec![(c + vec2(-60.0, -212.0), "1".into()), (c + vec2(0.0, -212.0), "2".into()), (c + vec2(60.0, -212.0), "3".into())],
            service: vec![],
            buildings: vec![AeroBuilding { outer: sq(c + vec2(0.0, -240.0), vec2(200.0, 30.0)), class: BClass::Terminal.code(), height: 15.0, base: 0.0 }],
        };
        (ap, vec![(id, Arc::new(t))], id)
    }

    #[test]
    fn layout_places_holds_stands_signs_and_bakes() {
        let (ap, tiles, id) = synthetic();
        let l = build(&ap, &tiles, None);
        assert!(!l.holds.is_empty(), "no holding position");
        let h = &l.holds[0];
        assert_eq!(h.taxiway, "B");
        assert!(h.dir.y > 0.9, "hold faces the runway: {:?}", h.dir);
        assert!(l.stands.len() >= 3, "{} stands", l.stands.len());
        // Gate stands nose in to the terminal (north of the apron: heading ~0°).
        let s = l.stands.iter().find(|s| s.name == "2").expect("stand 2");
        assert!(s.hdg.to_degrees().abs() < 5.0, "{}", s.hdg.to_degrees());
        assert!(s.contact.is_some(), "contact stand (jet bridge)");
        assert!(!l.marks.idx.is_empty());
        assert!(l.objects.len() >= 3, "signs, bridges, masts: {}", l.objects.len());
        assert!(!l.socks.is_empty());
        assert!(l.lights.iter().any(|li| li.kind() == kind::TAXI));
        // Baked on a flat grid: draped markings and objects stay in the tile.
        let tf = Proj::tile_frame(id);
        let (lo, hi) = id.rect(&tf);
        let grid = crate::terrain::Grid::flat(lo, hi - lo, 64);
        let b = bake(&l, id, &grid);
        assert!(!b.marks.idx.is_empty() && !b.solids.idx.is_empty());
        assert!(b.marks.verts.iter().all(|v| v.pos[0] >= lo.x - 0.01 && v.pos[0] <= hi.x + 0.01 && v.pos[1].abs() < 1e-3));
        assert_eq!(b.parked.len(), l.parked.len());
    }

    #[test]
    fn air_bases_park_transports() {
        let (mut ap, tiles, _) = synthetic();
        let civil: Vec<u16> = build(&ap, &tiles, None).parked.iter().map(|p| p.model).collect();
        assert!(!civil.iter().any(|&m| m == types::C130 || m == types::VULCAN), "{civil:?}");
        ap.name = "RAF Test".into();
        let mil: Vec<u16> = build(&ap, &tiles, None).parked.iter().map(|p| p.model).collect();
        assert!(!mil.is_empty());
        assert!(mil.iter().all(|m| [types::C130, types::VULCAN, types::C172].contains(m)), "{mil:?}");
    }

    #[test]
    fn dashes_follow_the_line() {
        let mut m = MeshData::default();
        dashed(&mut m, &[vec2(0.0, 0.0), vec2(10.0, 0.0), vec2(10.0, 10.0)], 0.2, PAINT_YELLOW, 1.0, 1.0);
        // 20 m of line, 1 m dashes every 2 m: ~10 dashes (+ split ones).
        let quads = m.idx.len() / 6;
        assert!((10..=12).contains(&quads), "{quads}");
    }
}
