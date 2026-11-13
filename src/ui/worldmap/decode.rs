//! Vector tile → world-map tile (runs on a worker thread natively, in a tile
//! worker in the browser): land use / water / park / aeroway / building
//! fills clipped to the tile square and triangulated; roads, rail, rivers,
//! taxiways, runways and borders by class as width-independent stroke
//! meshes (each vertex carries its offset direction, so one mesh draws at
//! any width — no re-tessellation while zooming); place, water, road,
//! taxiway, gate and helipad labels. Everything in tile units (0..1).

use crate::mvt::{self, GeomType};
use crate::osm::TileId;

// Fill classes, painter's order.
pub const F_RESIDENTIAL: u8 = 0;
pub const F_COMMERCIAL: u8 = 1;
pub const F_INDUSTRIAL: u8 = 2;
pub const F_FARM: u8 = 3;
pub const F_GRASS: u8 = 4;
pub const F_SCRUB: u8 = 5;
pub const F_WOOD: u8 = 6;
pub const F_WETLAND: u8 = 7;
pub const F_SAND: u8 = 8;
pub const F_ICE: u8 = 9;
pub const F_PARK: u8 = 10;
pub const F_AERODROME: u8 = 11;
pub const F_WATER: u8 = 12;
pub const F_APRON: u8 = 13;
pub const F_HELIPAD: u8 = 14;
pub const F_BUILDING: u8 = 15;
pub const N_FILLS: usize = 16;

// Line classes, painter's order.
pub const L_STREAM: u8 = 0;
pub const L_RIVER: u8 = 1;
pub const L_PATH: u8 = 2;
pub const L_SERVICE: u8 = 3;
pub const L_MINOR: u8 = 4;
pub const L_TERTIARY: u8 = 5;
pub const L_SECONDARY: u8 = 6;
pub const L_PRIMARY: u8 = 7;
pub const L_TRUNK: u8 = 8;
pub const L_MOTORWAY: u8 = 9;
pub const L_RAIL: u8 = 10;
pub const L_TAXIWAY: u8 = 11;
pub const L_RUNWAY: u8 = 12;
pub const L_STATE: u8 = 13;
pub const L_COUNTRY: u8 = 14;
pub const N_LINES: usize = 15;

// Label kinds.
pub const K_COUNTRY: u8 = 0;
pub const K_STATE: u8 = 1;
pub const K_CITY: u8 = 2;
pub const K_TOWN: u8 = 3;
pub const K_VILLAGE: u8 = 4;
pub const K_SUBURB: u8 = 5;
pub const K_SEA: u8 = 6;
pub const K_LAKE: u8 = 7;
pub const K_ROAD: u8 = 8;
pub const K_SHIELD: u8 = 9;
pub const K_TAXI: u8 = 10;
pub const K_GATE: u8 = 11;
pub const K_PAD: u8 = 12;
pub const K_RIVER: u8 = 13;

/// A stroke mesh: vertex = `base + off · half-width` (tile units + points).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Strokes {
    pub base: Vec<[f32; 2]>,
    pub off: Vec<[f32; 2]>,
    pub idx: Vec<u32>,
}

impl Strokes {
    /// Add a polyline: a quad per segment, bevel joins between them.
    pub fn add(&mut self, pts: &[[f32; 2]]) {
        let mut prev_end: Option<u32> = None;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let d = [b[0] - a[0], b[1] - a[1]];
            let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
            if len < 1e-9 {
                continue;
            }
            let n = [-d[1] / len, d[0] / len];
            let m = [-n[0], -n[1]];
            let i = self.base.len() as u32;
            self.base.extend([a, a, b, b]);
            self.off.extend([n, m, n, m]);
            self.idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
            if let Some(pe) = prev_end {
                let c = self.base.len() as u32;
                self.base.push(a);
                self.off.push([0.0, 0.0]);
                self.idx.extend([c, pe, i, c, pe + 1, i + 1]);
            }
            prev_end = Some(i + 2);
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Label {
    pub pos: [f32; 2],
    pub text: String,
    pub kind: u8,
    /// Lower = more important (OpenMapTiles rank).
    pub rank: u8,
    /// Text angle (rad, screen orientation), 0 for points.
    pub angle: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapTile {
    /// (class, vertices, triangle indices), painter's order.
    pub fills: Vec<(u8, Vec<[f32; 2]>, Vec<u32>)>,
    pub lines: Vec<(u8, Strokes)>,
    pub labels: Vec<Label>,
}

impl MapTile {
    /// Approximate memory (bytes) for the cache budget.
    pub fn bytes(&self) -> usize {
        let f: usize = self.fills.iter().map(|f| f.1.len() * 8 + f.2.len() * 4).sum();
        let l: usize = self.lines.iter().map(|l| l.1.base.len() * 16 + l.1.idx.len() * 4).sum();
        let t: usize = self.labels.iter().map(|l| 32 + l.text.len()).sum();
        f + l + t + 256
    }
    pub fn fill(&self, class: u8) -> Option<(&[[f32; 2]], &[u32])> {
        self.fills.iter().find(|f| f.0 == class).map(|f| (&f.1[..], &f.2[..]))
    }
}

/// Simplification tolerance (tile units): ½ px with the tile 512 px wide;
/// the deepest tiles are overzoomed, so they keep their full precision.
fn tolerance(z: u32) -> f32 {
    if z >= super::pyramid::MAX_TILE_Z { 0.4 / 4096.0 } else { 1.0 / 1024.0 }
}

fn simplify(p: &[[f32; 2]], tol: f32) -> Vec<[f32; 2]> {
    let v: Vec<glam::Vec2> = p.iter().map(|q| glam::vec2(q[0], q[1])).collect();
    super::super::mapdata::simplify(&v, tol).iter().map(|q| [q.x, q.y]).collect()
}

/// Sutherland–Hodgman clip of a ring to the unit square.
pub fn clip_ring(ring: &[[f32; 2]]) -> Vec<[f32; 2]> {
    let mut out: Vec<[f32; 2]> = ring.to_vec();
    for (axis, bound, lower) in [(0usize, 0.0f32, true), (0, 1.0, false), (1, 0.0, true), (1, 1.0, false)] {
        if out.is_empty() {
            break;
        }
        let inside = |p: &[f32; 2]| if lower { p[axis] >= bound } else { p[axis] <= bound };
        let input = std::mem::take(&mut out);
        let n = input.len();
        for i in 0..n {
            let (cur, prev) = (input[i], input[(i + n - 1) % n]);
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
    out.dedup();
    out
}

/// Liang–Barsky clip of a polyline to the unit square: the pieces inside.
pub fn clip_line(pts: &[[f32; 2]]) -> Vec<Vec<[f32; 2]>> {
    let mut out: Vec<Vec<[f32; 2]>> = Vec::new();
    let mut cur: Vec<[f32; 2]> = Vec::new();
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let (mut t0, mut t1) = (0.0f32, 1.0f32);
        let mut ok = true;
        for (p, q) in [(-d[0], a[0]), (d[0], 1.0 - a[0]), (-d[1], a[1]), (d[1], 1.0 - a[1])] {
            if p == 0.0 {
                if q < 0.0 {
                    ok = false;
                    break;
                }
            } else {
                let r = q / p;
                if p < 0.0 {
                    t0 = t0.max(r);
                } else {
                    t1 = t1.min(r);
                }
            }
        }
        if !ok || t0 > t1 {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            }
            cur.clear();
            continue;
        }
        let pa = [a[0] + d[0] * t0, a[1] + d[1] * t0];
        let pb = [a[0] + d[0] * t1, a[1] + d[1] * t1];
        if cur.last() != Some(&pa) {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            }
            cur.clear();
            cur.push(pa);
        }
        cur.push(pb);
        if t1 < 1.0 {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            }
            cur.clear();
        }
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

fn fill_class(layer: &str, class: &str, subclass: &str) -> Option<u8> {
    Some(match (layer, class) {
        ("water", _) => F_WATER,
        ("landcover", "wood" | "forest") => F_WOOD,
        ("landcover", "grass") if matches!(subclass, "scrub" | "heath") => F_SCRUB,
        ("landcover", "grass") => F_GRASS,
        ("landcover", "farmland") => F_FARM,
        ("landcover", "wetland") => F_WETLAND,
        ("landcover", "sand" | "rock" | "beach") => F_SAND,
        ("landcover", "ice" | "glacier") => F_ICE,
        ("landuse", "residential" | "suburb" | "neighbourhood" | "quarter") => F_RESIDENTIAL,
        ("landuse", "commercial" | "retail" | "hospital" | "school" | "university" | "college" | "kindergarten") => F_COMMERCIAL,
        ("landuse", "industrial" | "railway" | "garages" | "quarry" | "military" | "dam") => F_INDUSTRIAL,
        ("landuse", "cemetery" | "pitch" | "playground" | "stadium" | "park" | "zoo" | "theme_park") => F_PARK,
        ("park", _) => F_PARK,
        ("aeroway", "aerodrome") => F_AERODROME,
        ("aeroway", "apron") => F_APRON,
        ("aeroway", "helipad" | "heliport") => F_HELIPAD,
        ("building", _) => F_BUILDING,
        _ => return None,
    })
}

fn line_class(layer: &str, class: &str) -> Option<u8> {
    Some(match (layer, class) {
        ("waterway", "river" | "canal") => L_RIVER,
        ("waterway", _) => L_STREAM,
        ("transportation", "motorway") => L_MOTORWAY,
        ("transportation", "trunk") => L_TRUNK,
        ("transportation", "primary") => L_PRIMARY,
        ("transportation", "secondary") => L_SECONDARY,
        ("transportation", "tertiary") => L_TERTIARY,
        ("transportation", "minor" | "busway") => L_MINOR,
        ("transportation", "service") => L_SERVICE,
        ("transportation", "track" | "path") => L_PATH,
        ("transportation", "rail" | "transit") => L_RAIL,
        ("aeroway", "taxiway") => L_TAXIWAY,
        ("aeroway", "runway") => L_RUNWAY,
        _ => return None,
    })
}

fn name_of(f: &mvt::Feature) -> &str {
    f.str("name_en").or(f.str("name:latin")).or(f.str("name")).unwrap_or("")
}

/// "Taxiway F" → "F", "Link 58" → "58", "A10E" stays.
pub fn taxi_ref(s: &str) -> String {
    let s = s.trim();
    for p in ["Taxiway ", "taxiway ", "TWY ", "Twy ", "Link ", "link "] {
        if let Some(r) = s.strip_prefix(p) {
            return r.trim().to_string();
        }
    }
    s.to_string()
}

/// Upright text angle along a direction (screen y down).
fn upright(dx: f32, dy: f32) -> f32 {
    let mut a = dy.atan2(dx);
    if a > std::f32::consts::FRAC_PI_2 {
        a -= std::f32::consts::PI;
    } else if a < -std::f32::consts::FRAC_PI_2 {
        a += std::f32::consts::PI;
    }
    a
}

/// Label anchor along a line: the middle of its longest piece in the tile.
fn along(parts: &[Vec<[f32; 2]>], min_len: f32) -> Option<([f32; 2], f32)> {
    let mut best: Option<(f32, [f32; 2], f32)> = None;
    for p in parts {
        let len: f32 = p.windows(2).map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt()).sum();
        if len < min_len || best.is_some_and(|b| b.0 >= len) {
            continue;
        }
        let mut acc = 0.0;
        for w in p.windows(2) {
            let l = ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt();
            if acc + l >= len * 0.5 && l > 0.0 {
                let t = (len * 0.5 - acc) / l;
                let q = [w[0][0] + (w[1][0] - w[0][0]) * t, w[0][1] + (w[1][1] - w[0][1]) * t];
                best = Some((len, q, upright(w[1][0] - w[0][0], w[1][1] - w[0][1])));
                break;
            }
            acc += l;
        }
    }
    best.and_then(|b| (b.1[0] >= 0.0 && b.1[0] < 1.0 && b.1[1] >= 0.0 && b.1[1] < 1.0).then_some((b.1, b.2)))
}

fn inside(p: [f32; 2]) -> bool {
    p[0] >= 0.0 && p[0] < 1.0 && p[1] >= 0.0 && p[1] < 1.0
}

/// Decode one vector tile.
pub fn decode(id: TileId, bytes: &[u8]) -> MapTile {
    let mut t = MapTile::default();
    let Ok(layers) = mvt::decode(bytes) else { return t };
    let tol = tolerance(id.z);
    let min_area = (tol * 2.0) * (tol * 2.0);
    let mut fills: Vec<(Vec<[f32; 2]>, Vec<u32>)> = vec![Default::default(); N_FILLS];
    let mut lines: Vec<Strokes> = vec![Default::default(); N_LINES];
    for layer in &layers {
        let e = layer.extent as f32;
        let norm = |p: &Vec<[f32; 2]>| -> Vec<[f32; 2]> { p.iter().map(|q| [q[0] / e, q[1] / e]).collect() };
        let lname = layer.name.as_str();
        for f in &layer.features {
            let class = f.str("class").unwrap_or("");
            match f.geom_type {
                GeomType::Polygon => {
                    let Some(fc) = fill_class(lname, class, f.str("subclass").unwrap_or("")) else { continue };
                    if lname == "building" && f.tags.get("hide_3d").is_some_and(|v| matches!(v, mvt::Value::Bool(true)) || v.as_f64().unwrap_or(0.0) > 0.5) {
                        continue;
                    }
                    let slot = &mut fills[fc as usize];
                    for (outer, holes) in mvt::polygons(&f.parts) {
                        let outer = clip_ring(&simplify(&norm(&outer), tol));
                        if outer.len() < 3 || mvt::ring_area(&outer).abs() < min_area {
                            continue;
                        }
                        let mut pts = outer;
                        let mut hi = Vec::new();
                        for h in &holes {
                            let h = clip_ring(&simplify(&norm(h), tol));
                            if h.len() >= 3 && mvt::ring_area(&h).abs() >= min_area {
                                hi.push(pts.len());
                                pts.extend(h);
                            }
                        }
                        let data: Vec<f64> = pts.iter().flat_map(|p| [p[0] as f64, p[1] as f64]).collect();
                        if let Ok(tr) = earcutr::earcut(&data, &hi, 2) {
                            let base = slot.0.len() as u32;
                            slot.0.extend(pts);
                            slot.1.extend(tr.into_iter().map(|i| base + i as u32));
                        }
                    }
                    // Helipad areas also get a label anchor.
                    if fc == F_HELIPAD {
                        if let Some(r) = f.parts.first() {
                            let n = r.len().max(1) as f32;
                            let c = [r.iter().map(|p| p[0]).sum::<f32>() / n / e, r.iter().map(|p| p[1]).sum::<f32>() / n / e];
                            if inside(c) {
                                t.labels.push(Label { pos: c, text: f.str("ref").unwrap_or("H").to_string(), kind: K_PAD, rank: 0, angle: 0.0 });
                            }
                        }
                    }
                }
                GeomType::Line => {
                    if lname == "boundary" {
                        if f.num("maritime").unwrap_or(0.0) > 0.5 || matches!(f.tags.get("maritime"), Some(mvt::Value::Bool(true))) {
                            continue;
                        }
                        let lc = match f.num("admin_level").unwrap_or(10.0) as i32 {
                            2 => L_COUNTRY,
                            3 | 4 => L_STATE,
                            _ => continue,
                        };
                        for p in &f.parts {
                            for piece in clip_line(&simplify(&norm(p), tol)) {
                                lines[lc as usize].add(&piece);
                            }
                        }
                        continue;
                    }
                    if lname == "transportation_name" || lname == "water_name" {
                        let name = name_of(f);
                        let parts: Vec<Vec<[f32; 2]>> = f.parts.iter().map(&norm).collect();
                        if lname == "water_name" {
                            if !name.is_empty() {
                                if let Some((p, a)) = along(&parts, 0.05) {
                                    t.labels.push(Label { pos: p, text: name.to_string(), kind: K_RIVER, rank: 10, angle: a });
                                }
                            }
                            continue;
                        }
                        let r = f.str("ref").unwrap_or("");
                        if matches!(class, "motorway" | "trunk") && !r.is_empty() {
                            if let Some((p, _)) = along(&parts, 0.15) {
                                t.labels.push(Label {
                                    pos: p,
                                    text: r.split(';').next().unwrap_or(r).to_string(),
                                    kind: K_SHIELD,
                                    rank: if class == "motorway" { 1 } else { 2 },
                                    angle: 0.0,
                                });
                            }
                        }
                        if !name.is_empty() && !matches!(class, "motorway" | "path" | "track" | "service" | "rail" | "transit") {
                            if let Some((p, a)) = along(&parts, 0.08) {
                                let rank = match class {
                                    "trunk" | "primary" => 3,
                                    "secondary" => 4,
                                    "tertiary" => 5,
                                    _ => 7,
                                };
                                t.labels.push(Label { pos: p, text: name.to_string(), kind: K_ROAD, rank, angle: a });
                            }
                        }
                        continue;
                    }
                    if f.str("brunnel") == Some("tunnel") {
                        continue;
                    }
                    let Some(lc) = line_class(lname, class) else { continue };
                    if lc == L_RAIL && f.str("subclass").is_some_and(|s| matches!(s, "subway" | "tram" | "funicular")) {
                        continue;
                    }
                    let parts: Vec<Vec<[f32; 2]>> = f.parts.iter().map(|p| simplify(&norm(p), tol)).collect();
                    for p in &parts {
                        for piece in clip_line(p) {
                            lines[lc as usize].add(&piece);
                        }
                    }
                    if lc == L_TAXIWAY {
                        let r = taxi_ref(f.str("ref").unwrap_or(""));
                        if !r.is_empty() {
                            if let Some((p, a)) = along(&parts, 0.02) {
                                t.labels.push(Label { pos: p, text: r, kind: K_TAXI, rank: 0, angle: a });
                            }
                        }
                    }
                }
                GeomType::Point => {
                    let kind = match (lname, class) {
                        ("place", "country") => K_COUNTRY,
                        ("place", "state" | "province") => K_STATE,
                        ("place", "city") => K_CITY,
                        ("place", "town") => K_TOWN,
                        ("place", "village") => K_VILLAGE,
                        ("place", "suburb" | "quarter" | "neighbourhood") => K_SUBURB,
                        ("water_name", "ocean" | "sea" | "bay" | "strait") => K_SEA,
                        ("water_name", _) => K_LAKE,
                        ("aeroway", "gate") => K_GATE,
                        ("aeroway", "helipad" | "heliport") => K_PAD,
                        _ => continue,
                    };
                    let text = match kind {
                        K_GATE => f.str("ref").unwrap_or("").to_string(),
                        K_PAD => f.str("ref").unwrap_or("H").to_string(),
                        _ => name_of(f).to_string(),
                    };
                    if text.is_empty() {
                        continue;
                    }
                    let rank = f.num("rank").unwrap_or(match kind {
                        K_COUNTRY | K_STATE => 3.0,
                        _ => 10.0,
                    }) as u8;
                    for part in &f.parts {
                        for p in part {
                            let q = [p[0] / e, p[1] / e];
                            if inside(q) {
                                t.labels.push(Label { pos: q, text: text.clone(), kind, rank, angle: 0.0 });
                            }
                        }
                    }
                }
                GeomType::Unknown => {}
            }
        }
    }
    t.fills = fills.into_iter().enumerate().filter(|(_, f)| !f.1.is_empty()).map(|(i, f)| (i as u8, f.0, f.1)).collect();
    t.lines = lines.into_iter().enumerate().filter(|(_, l)| !l.idx.is_empty()).map(|(i, l)| (i as u8, l)).collect();
    t
}

/// Browser: the tile crosses from the worker to the page as bytes.
mod wire_impls {
    use super::*;
    use crate::platform::wire::{R, Res, W, Wire, get_pod, put_pod};

    crate::wire_struct!(Label { pos, text, kind, rank, angle });

    impl Wire for Strokes {
        fn put(&self, w: &mut W) {
            put_pod(w, &self.base);
            put_pod(w, &self.off);
            put_pod(w, &self.idx);
        }
        fn get(r: &mut R) -> Res<Self> {
            Ok(Strokes { base: get_pod(r)?, off: get_pod(r)?, idx: get_pod(r)? })
        }
    }

    impl Wire for MapTile {
        fn put(&self, w: &mut W) {
            self.fills.len().put(w);
            for (c, v, i) in &self.fills {
                c.put(w);
                put_pod(w, v);
                put_pod(w, i);
            }
            self.lines.len().put(w);
            for (c, s) in &self.lines {
                c.put(w);
                s.put(w);
            }
            self.labels.put(w);
        }
        fn get(r: &mut R) -> Res<Self> {
            let mut t = MapTile::default();
            for _ in 0..usize::get(r)? {
                t.fills.push((u8::get(r)?, get_pod(r)?, get_pod(r)?));
            }
            for _ in 0..usize::get(r)? {
                t.lines.push((u8::get(r)?, Strokes::get(r)?));
            }
            t.labels = Vec::get(r)?;
            Ok(t)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clip_line_pieces() {
        // crosses the square left to right, leaves and re-enters at the top
        let p = clip_line(&[[-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [0.8, -0.5], [0.8, 0.2]]);
        assert_eq!(p.len(), 2, "{p:?}");
        let near = |a: &[[f32; 2]], b: &[[f32; 2]]| a.len() == b.len() && a.iter().zip(b).all(|(p, q)| (p[0] - q[0]).abs() < 1e-5 && (p[1] - q[1]).abs() < 1e-5);
        assert!(near(&p[0], &[[0.0, 0.5], [0.5, 0.5], [0.5, 0.0]]), "{p:?}");
        assert!(near(&p[1], &[[0.8, 0.0], [0.8, 0.2]]), "{p:?}");
        assert!(clip_line(&[[2.0, 2.0], [3.0, 3.0]]).is_empty());
    }

    #[test]
    fn clip_ring_to_square() {
        let r = clip_ring(&[[-1.0, -1.0], [2.0, -1.0], [2.0, 2.0], [-1.0, 2.0]]);
        assert!((mvt::ring_area(&r).abs() - 1.0).abs() < 1e-6, "{r:?}");
    }

    #[test]
    fn strokes_have_joins_and_offsets() {
        let mut s = Strokes::default();
        s.add(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]);
        // two quads + one join vertex
        assert_eq!(s.base.len(), 9);
        assert_eq!(s.idx.len(), 6 * 2 + 6);
        assert_eq!(s.off[0], [-0.0, 1.0]);
        assert!(s.off.iter().all(|o| (o[0] * o[0] + o[1] * o[1] - 1.0).abs() < 1e-6 || *o == [0.0, 0.0]));
    }

    #[test]
    fn taxiway_refs() {
        assert_eq!(taxi_ref("Taxiway F"), "F");
        assert_eq!(taxi_ref("Link 58"), "58");
        assert_eq!(taxi_ref("A10E"), "A10E");
    }

    #[test]
    fn upright_text() {
        assert!(upright(-1.0, 0.0).abs() < 1e-6);
        assert!((upright(0.0, -1.0) + std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    /// A cached Heathrow tile (when the tile cache is present): aprons,
    /// taxiways with refs, gates, the runway.
    #[test]
    fn decodes_cached_airport_tile() {
        let Ok(b) = std::fs::read("cache/tiles/14/8170/5450.pbf") else { return };
        let t = decode(TileId { z: 14, x: 8170, y: 5450 }, &b);
        assert!(t.fill(F_APRON).is_some());
        assert!(t.lines.iter().any(|l| l.0 == L_TAXIWAY));
        assert!(t.labels.iter().any(|l| l.kind == K_TAXI && l.text == "F"));
        assert!(t.labels.iter().any(|l| l.kind == K_GATE));
        let w = crate::platform::wire::encode(&t);
        assert_eq!(crate::platform::wire::decode::<MapTile>(&w).unwrap(), t);
    }
}
