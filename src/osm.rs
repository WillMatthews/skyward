//! OpenStreetMap data from OpenMapTiles-schema vector tiles: projection,
//! slippy-tile math and MVT → world-space feature extraction (clipped to the
//! exact tile extent so neighbouring tiles never double up).
//!
//! World frame: metres, +X = east, -Z = north, +Y = up, equirectangular
//! around a floating origin (see [`Proj`]; `World::rebase` moves it).

use crate::mvt::{self, GeomType, Layer};
use glam::{vec2, Vec2};
use std::f64::consts::PI;

/// Max zoom of the OpenMapTiles planet tiles: detailed scenery.
pub const ZOOM: u32 = 14;
/// Far-field zoom: generalised landuse / water / woods out to the horizon.
pub const FAR_ZOOM: u32 = 10;

/// Local equirectangular frame around (lat0, lon0): exact metres along the
/// meridians, metres at the origin's latitude along the parallels. Only
/// accurate near the origin — the app keeps the aircraft near it by
/// re-centring ("rebasing") the frame as it flies, and every map tile is
/// built in its own frame anchored at its north-west corner (small, precise
/// vertex coordinates) and placed with an exact [`Xform2`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Proj {
    pub lat0: f64,
    pub lon0: f64,
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

/// Metres per degree of latitude (all frames).
pub const M_PER_DEG_LAT: f64 = 111_320.0;

/// Map between two equirectangular frames: x' = sx * x + tx, z' = z + tz.
/// Exact (both frames are linear in lon / lat), so tiles built in different
/// frames line up perfectly in the current one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Xform2 {
    pub sx: f64,
    pub tx: f64,
    pub tz: f64,
}

impl Xform2 {
    pub fn apply(&self, p: Vec2) -> Vec2 {
        vec2((self.sx * p.x as f64 + self.tx) as f32, (p.y as f64 + self.tz) as f32)
    }
    pub fn inverse(&self, p: Vec2) -> Vec2 {
        vec2(((p.x as f64 - self.tx) / self.sx) as f32, (p.y as f64 - self.tz) as f32)
    }
    /// GPU form: (sx - 1, tx, tz, 0) — all zeros is the identity.
    pub fn gpu(&self) -> [f32; 4] {
        [(self.sx - 1.0) as f32, self.tx as f32, self.tz as f32, 0.0]
    }
}

#[allow(clippy::wrong_self_convention)]
impl Proj {
    pub fn new(lat0: f64, lon0: f64) -> Self {
        let m_per_deg_lat = M_PER_DEG_LAT;
        Proj { lat0, lon0, m_per_deg_lat, m_per_deg_lon: m_per_deg_lat * lat0.to_radians().cos() }
    }
    /// The frame a tile's geometry is built in: anchored at its north-west corner.
    pub fn tile_frame(id: TileId) -> Proj {
        Proj::new(tile_lat(id.y as f64, id.z), tile_lon(id.x as f64, id.z))
    }
    /// lat/lon → world (x, z)
    pub fn to_world(&self, lat: f64, lon: f64) -> Vec2 {
        let (x, z) = self.to_world64(lat, lon);
        vec2(x as f32, z as f32)
    }
    pub fn to_world64(&self, lat: f64, lon: f64) -> (f64, f64) {
        ((lon - self.lon0) * self.m_per_deg_lon, -(lat - self.lat0) * self.m_per_deg_lat)
    }
    /// world (x, z) → (lat, lon)
    pub fn to_latlon(&self, p: Vec2) -> (f64, f64) {
        (self.lat0 - p.y as f64 / self.m_per_deg_lat, self.lon0 + p.x as f64 / self.m_per_deg_lon)
    }
    /// The exact map from coordinates in this frame to coordinates in `to`.
    pub fn map_to(&self, to: &Proj) -> Xform2 {
        Xform2 {
            sx: to.m_per_deg_lon / self.m_per_deg_lon,
            tx: (self.lon0 - to.lon0) * to.m_per_deg_lon,
            tz: (to.lat0 - self.lat0) * self.m_per_deg_lat,
        }
    }
}

// ---------------------------------------------------------------- tiles

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TileId {
    pub z: u32,
    pub x: u32,
    pub y: u32,
}

fn tile_lon(x: f64, z: u32) -> f64 {
    x / (1u64 << z) as f64 * 360.0 - 180.0
}
fn tile_lat(y: f64, z: u32) -> f64 {
    (PI * (1.0 - 2.0 * y / (1u64 << z) as f64)).sinh().atan().to_degrees()
}

impl TileId {
    pub fn new(x: u32, y: u32) -> Self {
        TileId { z: ZOOM, x, y }
    }
    /// Fractional tile coordinates of a lat/lon at zoom `z`.
    pub fn frac(lat: f64, lon: f64, z: u32) -> (f64, f64) {
        let n = (1u64 << z) as f64;
        let lat = lat.clamp(-85.05, 85.05).to_radians();
        ((lon + 180.0) / 360.0 * n, (1.0 - lat.tan().asinh() / PI) / 2.0 * n)
    }
    pub fn at_world(proj: &Proj, p: Vec2) -> TileId {
        Self::at_world_z(proj, p, ZOOM)
    }
    pub fn at_world_z(proj: &Proj, p: Vec2, z: u32) -> TileId {
        let (lat, lon) = proj.to_latlon(p);
        let (x, y) = Self::frac(lat, lon, z);
        TileId { z, x: x.floor() as u32, y: y.floor() as u32 }
    }
    pub fn is_far(&self) -> bool {
        self.z < ZOOM
    }
    /// World-space rectangle (min, max) covered by the tile. Mercator tile
    /// edges are meridians/parallels, i.e. axis-aligned in our projection.
    pub fn rect(&self, proj: &Proj) -> (Vec2, Vec2) {
        let nw = proj.to_world(tile_lat(self.y as f64, self.z), tile_lon(self.x as f64, self.z));
        let se = proj.to_world(tile_lat(self.y as f64 + 1.0, self.z), tile_lon(self.x as f64 + 1.0, self.z));
        (nw.min(se), nw.max(se))
    }
    /// Stable 64-bit seed.
    pub fn seed(&self) -> u64 {
        ((self.z as u64) << 56) ^ ((self.x as u64) << 28) ^ self.y as u64
    }
}

/// Maps tile-local units (0..extent, y down) to world space.
struct Xf {
    proj: Proj,
    id: TileId,
    extent: f64,
}

impl Xf {
    fn world(&self, p: [f32; 2]) -> Vec2 {
        let lon = tile_lon(self.id.x as f64 + p[0] as f64 / self.extent, self.id.z);
        let lat = tile_lat(self.id.y as f64 + p[1] as f64 / self.extent, self.id.z);
        self.proj.to_world(lat, lon)
    }
    fn ring(&self, r: &[[f32; 2]]) -> Vec<Vec2> {
        r.iter().map(|p| self.world(*p)).collect()
    }
}

// ---------------------------------------------------------------- features

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Building { height: f32, min_height: f32, colour: Option<[f32; 3]> },
    /// Built-up ground: 0 residential, 1 commercial/retail, 2 industrial/railway, 3 institutional.
    Urban(u8),
    Water,
    Grass,
    /// Parks, gardens, pitches: grass with occasional trees.
    Park,
    Wood,
    Scrub,
    Wetland,
    Sand,
    Rock,
    Farmland,
    Aerodrome,
    Apron,
    RunwayArea,
    Pier,
    Road { width: f32, major: bool },
    Track { width: f32 },
    Rail,
    Waterway { width: f32 },
    Taxiway { width: f32 },
}

/// Polygon feature, clipped to the tile. `border` flags the outer-ring
/// edges (i → i+1) that lie on the tile boundary (artificial clip edges).
#[derive(Clone, Debug)]
pub struct Area {
    pub kind: Kind,
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
    pub border: Vec<bool>,
    pub hole_border: Vec<Vec<bool>>,
    /// Stable per-feature key (same in every tile that holds a piece of it).
    pub key: u64,
}

/// Polyline, clipped to the tile. `cut[0]/cut[1]`: start/end lie on a clip edge.
#[derive(Clone, Debug)]
pub struct Line {
    pub kind: Kind,
    pub pts: Vec<Vec2>,
    pub cut: [bool; 2],
}

/// A runway centreline piece (unclipped, may extend into the tile buffer).
#[derive(Clone, Debug)]
pub struct RunwayPiece {
    pub a: Vec2,
    pub b: Vec2,
    pub name: String,
}

#[derive(Clone, Debug)]
pub struct Airport {
    pub pos: Vec2,
    pub icao: String,
    pub iata: String,
    pub name: String,
}

#[derive(Default)]
pub struct TileFeatures {
    pub areas: Vec<Area>,
    pub lines: Vec<Line>,
    pub runways: Vec<RunwayPiece>,
    pub airports: Vec<Airport>,
}

fn hash(mut x: u64) -> u64 {
    x ^= x >> 33;
    x = x.wrapping_mul(0xff51afd7ed558ccd);
    x ^= x >> 33;
    x = x.wrapping_mul(0xc4ceb9fe1a85ec53);
    x ^ (x >> 33)
}

fn parse_colour(s: &str) -> Option<[f32; 3]> {
    let s = s.strip_prefix('#')?;
    if s.len() != 6 {
        return None;
    }
    let v = u32::from_str_radix(s, 16).ok()?;
    Some([((v >> 16) & 255) as f32 / 255.0, ((v >> 8) & 255) as f32 / 255.0, (v & 255) as f32 / 255.0])
}

// ---------------------------------------------------------------- clipping

/// Sutherland–Hodgman clip of a ring against [0,e]².
fn clip_ring(ring: &[[f32; 2]], e: f32) -> Vec<[f32; 2]> {
    let mut out: Vec<[f32; 2]> = ring.to_vec();
    // (axis, keep >= bound if lower)
    for (axis, bound, lower) in [(0usize, 0.0f32, true), (0, e, false), (1, 0.0, true), (1, e, false)] {
        if out.is_empty() {
            break;
        }
        let inside = |p: &[f32; 2]| if lower { p[axis] >= bound } else { p[axis] <= bound };
        let input = std::mem::take(&mut out);
        let n = input.len();
        for i in 0..n {
            let cur = input[i];
            let prev = input[(i + n - 1) % n];
            let (ci, pi) = (inside(&cur), inside(&prev));
            if ci != pi {
                let t = (bound - prev[axis]) / (cur[axis] - prev[axis]);
                let mut q = [prev[0] + (cur[0] - prev[0]) * t, prev[1] + (cur[1] - prev[1]) * t];
                q[axis] = bound;
                out.push(q);
            }
            if ci {
                out.push(cur);
            }
        }
    }
    // Drop consecutive duplicates.
    out.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-3 && (a[1] - b[1]).abs() < 1e-3);
    while out.len() > 1 && {
        let (f, l) = (out[0], out[out.len() - 1]);
        (f[0] - l[0]).abs() < 1e-3 && (f[1] - l[1]).abs() < 1e-3
    } {
        out.pop();
    }
    if out.len() < 3 || mvt::ring_area(&out).abs() < 0.5 {
        return Vec::new();
    }
    out
}

fn on_border(a: [f32; 2], b: [f32; 2], e: f32) -> bool {
    (a[0] == b[0] && (a[0] <= 0.0 || a[0] >= e)) || (a[1] == b[1] && (a[1] <= 0.0 || a[1] >= e))
}

fn border_flags(r: &[[f32; 2]], e: f32) -> Vec<bool> {
    (0..r.len()).map(|i| on_border(r[i], r[(i + 1) % r.len()], e)).collect()
}

/// Clip a polyline to [0,e]², returning pieces with "cut" flags at each end.
fn clip_line(pts: &[[f32; 2]], e: f32) -> Vec<(Vec<[f32; 2]>, [bool; 2])> {
    let mut out: Vec<(Vec<[f32; 2]>, [bool; 2])> = Vec::new();
    let mut cur: Option<(Vec<[f32; 2]>, bool)> = None;
    for w in pts.windows(2) {
        let (p, q) = (w[0], w[1]);
        let d = [q[0] - p[0], q[1] - p[1]];
        let (mut t0, mut t1) = (0.0f32, 1.0f32);
        let mut ok = true;
        for (pp, qq) in [(-d[0], p[0]), (d[0], e - p[0]), (-d[1], p[1]), (d[1], e - p[1])] {
            if pp == 0.0 {
                if qq < 0.0 {
                    ok = false;
                    break;
                }
            } else {
                let r = qq / pp;
                if pp < 0.0 {
                    t0 = t0.max(r);
                } else {
                    t1 = t1.min(r);
                }
            }
        }
        if !ok || t0 > t1 {
            if let Some((v, s)) = cur.take() {
                out.push((v, [s, true]));
            }
            continue;
        }
        let a = [p[0] + d[0] * t0, p[1] + d[1] * t0];
        let b = [p[0] + d[0] * t1, p[1] + d[1] * t1];
        match cur.as_mut() {
            Some((v, _)) if t0 == 0.0 => v.push(b),
            _ => {
                if let Some((v, s)) = cur.take() {
                    out.push((v, [s, true]));
                }
                cur = Some((vec![a, b], t0 > 0.0));
            }
        }
        if t1 < 1.0 {
            if let Some((v, s)) = cur.take() {
                out.push((v, [s, true]));
            }
        }
    }
    if let Some((v, s)) = cur.take() {
        out.push((v, [s, false]));
    }
    out.retain(|(v, _)| v.len() >= 2);
    out
}

// ---------------------------------------------------------------- extraction

fn synth_height(area_m2: f32, key: u64) -> f32 {
    let r = (hash(key ^ 0x5bd1e995) >> 40) as f32 / (1u64 << 24) as f32;
    let (lo, hi) = if area_m2 < 40.0 {
        (2.6, 3.6) // sheds, garages
    } else if area_m2 < 220.0 {
        (6.0, 10.0) // houses
    } else if area_m2 < 900.0 {
        (8.0, 14.0) // terraces, small blocks
    } else if area_m2 < 4000.0 {
        (9.0, 18.0) // apartment blocks, warehouses
    } else {
        (10.0, 22.0) // big sheds, malls
    };
    lo + (hi - lo) * r
}

fn area_kind(layer: &str, class: &str, subclass: &str) -> Option<Kind> {
    Some(match layer {
        "landuse" => match class {
            "residential" | "suburb" | "quarter" | "neighbourhood" | "garages" => Kind::Urban(0),
            "commercial" | "retail" | "bus_station" => Kind::Urban(1),
            "industrial" | "railway" | "dam" => Kind::Urban(2),
            "school" | "university" | "college" | "kindergarten" | "hospital" | "library" | "stadium"
            | "military" | "theme_park" | "zoo" => Kind::Urban(3),
            "cemetery" | "pitch" | "playground" | "track" => Kind::Park,
            "quarry" => Kind::Sand,
            _ => return None,
        },
        "landcover" => match class {
            "wood" | "forest" => Kind::Wood,
            "grass" => match subclass {
                "scrub" | "shrubbery" | "heath" => Kind::Scrub,
                "park" | "garden" | "recreation_ground" | "village_green" | "allotments" => Kind::Park,
                _ => Kind::Grass,
            },
            "wetland" => Kind::Wetland,
            "sand" | "beach" => Kind::Sand,
            "rock" | "ice" => Kind::Rock,
            "farmland" => Kind::Farmland,
            _ => return None,
        },
        "water" => Kind::Water,
        "aeroway" => match class {
            "aerodrome" => Kind::Aerodrome,
            "apron" => Kind::Apron,
            "runway" => Kind::RunwayArea,
            _ => return None,
        },
        "transportation" => match class {
            "pier" => Kind::Pier,
            _ => return None,
        },
        _ => return None,
    })
}

fn line_kind(layer: &str, class: &str) -> Option<Kind> {
    Some(match layer {
        "transportation" => match class {
            "motorway" => Kind::Road { width: 18.0, major: true },
            "trunk" => Kind::Road { width: 14.0, major: true },
            "primary" => Kind::Road { width: 11.0, major: true },
            "secondary" => Kind::Road { width: 9.0, major: false },
            "tertiary" => Kind::Road { width: 8.0, major: false },
            "minor" => Kind::Road { width: 6.0, major: false },
            "service" => Kind::Road { width: 4.0, major: false },
            "raceway" | "busway" => Kind::Road { width: 7.0, major: false },
            "track" => Kind::Track { width: 3.0 },
            "rail" | "transit" => Kind::Rail,
            _ => return None,
        },
        "waterway" => match class {
            "river" => Kind::Waterway { width: 14.0 },
            "canal" => Kind::Waterway { width: 10.0 },
            "stream" => Kind::Waterway { width: 3.0 },
            "drain" | "ditch" => Kind::Waterway { width: 1.6 },
            _ => return None,
        },
        "aeroway" => match class {
            "taxiway" => Kind::Taxiway { width: 18.0 },
            _ => return None,
        },
        _ => return None,
    })
}

/// Decode one MVT tile into clipped world-space features.
pub fn extract(layers: &[Layer], id: TileId, proj: &Proj) -> TileFeatures {
    let mut out = TileFeatures::default();
    for layer in layers {
        let e = layer.extent as f32;
        let xf = Xf { proj: *proj, id, extent: layer.extent as f64 };
        let name = layer.name.as_str();
        // Metres per tile unit (for area estimates).
        let (lo, hi) = id.rect(proj);
        let mpu = ((hi.x - lo.x) / e, (hi.y - lo.y) / e);
        for f in &layer.features {
            let class = f.str("class").unwrap_or("");
            match (name, f.geom_type) {
                ("building", GeomType::Polygon) => {
                    if f.tags.get("hide_3d").and_then(|v| v.as_f64()).unwrap_or(0.0) > 0.5
                        || matches!(f.tags.get("hide_3d"), Some(mvt::Value::Bool(true)))
                    {
                        continue;
                    }
                    let rh = f.num("render_height").unwrap_or(0.0) as f32;
                    let rmin = f.num("render_min_height").unwrap_or(0.0) as f32;
                    let colour = f.str("colour").and_then(parse_colour);
                    for (outer, holes) in mvt::polygons(&f.parts) {
                        // Stable key: bbox centre in global tile units.
                        let (mut a, mut b) = ([f32::MAX; 2], [f32::MIN; 2]);
                        for p in &outer {
                            a = [a[0].min(p[0]), a[1].min(p[1])];
                            b = [b[0].max(p[0]), b[1].max(p[1])];
                        }
                        let gx = id.x as i64 * layer.extent as i64 + ((a[0] + b[0]) * 0.5 / 4.0) as i64 * 4;
                        let gy = id.y as i64 * layer.extent as i64 + ((a[1] + b[1]) * 0.5 / 4.0) as i64 * 4;
                        let key = hash((gx as u64) << 32 ^ gy as u64);
                        let full_area = mvt::ring_area(&outer).abs() * mpu.0 * mpu.1;
                        let height = if rh <= 5.0 { synth_height(full_area, key) } else { rh.min(500.0) };
                        let min_height = rmin.clamp(0.0, (height - 1.0).max(0.0));
                        push_area(&mut out, &xf, e, Kind::Building { height, min_height, colour }, &outer, &holes, key);
                    }
                }
                (_, GeomType::Polygon) => {
                    if f.str("brunnel") == Some("tunnel") {
                        continue;
                    }
                    let Some(kind) = area_kind(name, class, f.str("subclass").unwrap_or("")) else { continue };
                    for (outer, holes) in mvt::polygons(&f.parts) {
                        push_area(&mut out, &xf, e, kind, &outer, &holes, hash(f.id));
                    }
                }
                ("aeroway", GeomType::Line) if class == "runway" => {
                    for part in &f.parts {
                        if part.len() >= 2 {
                            out.runways.push(RunwayPiece {
                                a: xf.world(part[0]),
                                b: xf.world(*part.last().unwrap()),
                                name: f.str("ref").unwrap_or("").to_string(),
                            });
                        }
                    }
                }
                (_, GeomType::Line) => {
                    if matches!(f.str("brunnel"), Some("tunnel")) {
                        continue;
                    }
                    let Some(kind) = line_kind(name, class) else { continue };
                    for part in &f.parts {
                        for (pts, cut) in clip_line(part, e) {
                            out.lines.push(Line { kind, pts: xf.ring(&pts), cut });
                        }
                    }
                }
                ("aerodrome_label", GeomType::Point) => {
                    for part in &f.parts {
                        for p in part {
                            if p[0] >= 0.0 && p[0] < e && p[1] >= 0.0 && p[1] < e {
                                out.airports.push(Airport {
                                    pos: xf.world(*p),
                                    icao: f.str("icao").unwrap_or("").to_string(),
                                    iata: f.str("iata").unwrap_or("").to_string(),
                                    name: f.str("name").unwrap_or("").to_string(),
                                });
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }
    out
}

fn push_area(out: &mut TileFeatures, xf: &Xf, e: f32, kind: Kind, outer: &[[f32; 2]], holes: &[Vec<[f32; 2]>], key: u64) {
    let o = clip_ring(outer, e);
    if o.is_empty() {
        return;
    }
    let mut hs = Vec::new();
    let mut hb = Vec::new();
    for h in holes {
        let c = clip_ring(h, e);
        if !c.is_empty() {
            hb.push(border_flags(&c, e));
            hs.push(xf.ring(&c));
        }
    }
    out.areas.push(Area {
        kind,
        border: border_flags(&o, e),
        outer: xf.ring(&o),
        holes: hs,
        hole_border: hb,
        key,
    });
}

pub fn point_in_ring(pt: Vec2, ring: &[Vec2]) -> bool {
    let mut inside = false;
    let n = ring.len();
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (ring[i], ring[j]);
        if (a.y > pt.y) != (b.y > pt.y) && pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// (Browser tile workers: platform/wire.rs.)
crate::wire_struct!(TileId { z, x, y });
crate::wire_struct!(RunwayPiece { a, b, name });
crate::wire_struct!(Airport { pos, icao, iata, name });

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_roundtrip() {
        let p = Proj::new(51.5045, 0.03);
        let t = TileId::at_world(&p, Vec2::ZERO);
        assert_eq!((t.x, t.y), (8193, 5448));
        let (lo, hi) = t.rect(&p);
        assert!(lo.x < 0.0 && hi.x > 0.0 && lo.y < 0.0 && hi.y > 0.0);
        assert!((hi.x - lo.x - 1523.0).abs() < 10.0, "{}", hi.x - lo.x);
        let c = (lo + hi) * 0.5;
        assert_eq!(TileId::at_world(&p, c), t);
    }

    #[test]
    fn clip_square() {
        let r = vec![[-10.0, -10.0], [50.0, -10.0], [50.0, 50.0], [-10.0, 50.0]];
        let c = clip_ring(&r, 4096.0);
        assert_eq!(c.len(), 4);
        let f = border_flags(&c, 4096.0);
        assert_eq!(f.iter().filter(|b| **b).count(), 2);
        let l = clip_line(&[[-10.0, 5.0], [10.0, 5.0], [10.0, 5000.0]], 4096.0);
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].1, [true, true]);
        assert_eq!(l[0].0.len(), 3);
    }
}
