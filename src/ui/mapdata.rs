//! Vector moving-map data, built on the tile workers next to the 3D meshes:
//! simplified, pre-triangulated land-use / water / building fills, road /
//! rail / river polylines by class and place labels — in the tile's own
//! frame (like every tile mesh), placed with the tile's exact transform.

use crate::mvt::{GeomType, Layer};
use crate::osm::{Area, Kind, Proj, TileFeatures, TileId};
use glam::{vec2, Vec2};

/// Fill classes, in painter's order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Fill {
    Farm,
    Residential,
    Commercial,
    Industrial,
    Park,
    Scrub,
    Wood,
    Wetland,
    Sand,
    Aerodrome,
    Apron,
    Water,
    Building,
}

pub const FILLS: [Fill; 13] = [
    Fill::Farm,
    Fill::Residential,
    Fill::Commercial,
    Fill::Industrial,
    Fill::Park,
    Fill::Scrub,
    Fill::Wood,
    Fill::Wetland,
    Fill::Sand,
    Fill::Aerodrome,
    Fill::Apron,
    Fill::Water,
    Fill::Building,
];

/// Line classes, in painter's order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Line {
    Stream,
    River,
    Track,
    Minor,
    Secondary,
    Rail,
    Taxiway,
    Primary,
    Motorway,
}

pub const LINES: [Line; 9] = [Line::Stream, Line::River, Line::Track, Line::Minor, Line::Secondary, Line::Rail, Line::Taxiway, Line::Primary, Line::Motorway];

/// Triangulated fill (tile frame, metres).
#[derive(Clone, Debug, Default)]
pub struct FillMesh {
    pub verts: Vec<[f32; 2]>,
    pub idx: Vec<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum LabelKind {
    City,
    Town,
    Village,
    Suburb,
    Water,
}

#[derive(Clone, Debug)]
pub struct Label {
    pub pos: [f32; 2],
    pub text: String,
    pub kind: LabelKind,
    /// Lower = more important.
    pub rank: u16,
}

#[derive(Clone, Debug, Default)]
pub struct MapVectors {
    pub far: bool,
    pub fills: Vec<(Fill, FillMesh)>,
    /// Polylines (tile frame) per class; widths are in metres where useful.
    pub lines: Vec<(Line, Vec<Vec<[f32; 2]>>)>,
    pub labels: Vec<Label>,
    /// Coarse versions (≈8× the tolerance, small areas dropped) for zoomed-out views.
    pub fills_lo: Vec<(Fill, FillMesh)>,
    pub lines_lo: Vec<(Line, Vec<Vec<[f32; 2]>>)>,
    /// Rough count of vertices (budgeting).
    pub verts: usize,
}

impl MapVectors {
    pub fn fill(&self, f: Fill, lo: bool) -> Option<&FillMesh> {
        let v = if lo { &self.fills_lo } else { &self.fills };
        v.iter().find(|x| x.0 == f).map(|x| &x.1)
    }
    pub fn line(&self, l: Line, lo: bool) -> Option<&Vec<Vec<[f32; 2]>>> {
        let v = if lo { &self.lines_lo } else { &self.lines };
        v.iter().find(|x| x.0 == l).map(|x| &x.1)
    }
}

fn fill_of(k: &Kind) -> Option<Fill> {
    Some(match k {
        Kind::Building { .. } => Fill::Building,
        Kind::Urban(0) => Fill::Residential,
        Kind::Urban(2) => Fill::Industrial,
        Kind::Urban(_) => Fill::Commercial,
        Kind::Water => Fill::Water,
        Kind::Grass | Kind::Park => Fill::Park,
        Kind::Wood => Fill::Wood,
        Kind::Scrub => Fill::Scrub,
        Kind::Wetland => Fill::Wetland,
        Kind::Sand | Kind::Rock => Fill::Sand,
        Kind::Farmland => Fill::Farm,
        Kind::Aerodrome => Fill::Aerodrome,
        Kind::Apron | Kind::RunwayArea | Kind::Pier => Fill::Apron,
        _ => return None,
    })
}

fn line_of(k: &Kind) -> Option<Line> {
    Some(match *k {
        Kind::Road { width, .. } if width >= 15.0 => Line::Motorway,
        Kind::Road { major: true, .. } => Line::Primary,
        Kind::Road { width, .. } if width >= 8.0 => Line::Secondary,
        Kind::Road { .. } => Line::Minor,
        Kind::Track { .. } => Line::Track,
        Kind::Rail => Line::Rail,
        Kind::Waterway { width } if width >= 8.0 => Line::River,
        Kind::Waterway { .. } => Line::Stream,
        Kind::Taxiway { .. } => Line::Taxiway,
        _ => return None,
    })
}

/// Douglas–Peucker simplification (keeps the end points).
pub fn simplify(pts: &[Vec2], tol: f32) -> Vec<Vec2> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (pa, pb) = (pts[a], pts[b]);
        let d = pb - pa;
        let len = d.length();
        let mut best = (0.0f32, a);
        for (i, p) in pts.iter().enumerate().take(b).skip(a + 1) {
            let dist = if len < 1e-6 { p.distance(pa) } else { (*p - pa).perp_dot(d).abs() / len };
            if dist > best.0 {
                best = (dist, i);
            }
        }
        if best.0 > tol {
            keep[best.1] = true;
            stack.push((a, best.1));
            stack.push((best.1, b));
        }
    }
    pts.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

fn simplify_ring(r: &[Vec2], tol: f32) -> Vec<Vec2> {
    if r.len() < 8 {
        return r.to_vec();
    }
    // Split at the farthest point from the first so a closed ring simplifies well.
    let far = (1..r.len()).max_by(|&a, &b| r[a].distance_squared(r[0]).total_cmp(&r[b].distance_squared(r[0]))).unwrap_or(r.len() / 2);
    let mut a = simplify(&r[..=far], tol);
    let mut tail: Vec<Vec2> = r[far..].to_vec();
    tail.push(r[0]);
    let b = simplify(&tail, tol);
    a.pop();
    a.extend(b);
    a.pop();
    a
}

fn ring_area(r: &[Vec2]) -> f32 {
    let mut s = 0.0;
    for i in 0..r.len() {
        let (a, b) = (r[i], r[(i + 1) % r.len()]);
        s += a.x * b.y - b.x * a.y;
    }
    s * 0.5
}

fn add_area(m: &mut FillMesh, a: &Area, tol: f32, min_area: f32) {
    let outer = simplify_ring(&a.outer, tol);
    if outer.len() < 3 || ring_area(&outer).abs() < min_area {
        return;
    }
    let mut data: Vec<f64> = Vec::with_capacity(outer.len() * 2);
    let mut holes = Vec::new();
    let mut all: Vec<Vec2> = outer.clone();
    for p in &outer {
        data.extend([p.x as f64, p.y as f64]);
    }
    for h in &a.holes {
        let h = simplify_ring(h, tol);
        if h.len() < 3 || ring_area(&h).abs() < min_area {
            continue;
        }
        holes.push(all.len());
        for p in &h {
            data.extend([p.x as f64, p.y as f64]);
        }
        all.extend(h);
    }
    let Ok(tris) = earcutr::earcut(&data, &holes, 2) else { return };
    let base = m.verts.len() as u32;
    m.verts.extend(all.iter().map(|p| [p.x, p.y]));
    m.idx.extend(tris.into_iter().map(|t| base + t as u32));
}

/// Build the map vectors for one tile (called on a tile worker).
pub fn build(id: TileId, proj: &Proj, f: &TileFeatures, layers: &[Layer]) -> MapVectors {
    let far = id.is_far();
    let (lo, hi) = id.rect(proj);
    let size = (hi - lo).x.max(1.0);
    // Simplification tolerance: ~1/700 of the tile (≈2 m at z14, ≈35 m at z10).
    let tol = size / 700.0;
    let min_area = tol * tol * 6.0;
    let mut out = MapVectors { far, ..Default::default() };
    for lod in 0..2 {
        let k = if far { 3.0 } else { 8.0 };
        let (tol, min_area) = if lod == 0 { (tol, min_area) } else { (tol * k, min_area * k * k) };
        let mut fills: Vec<(Fill, FillMesh)> = FILLS.iter().map(|f| (*f, FillMesh::default())).collect();
        for a in &f.areas {
            let Some(fc) = fill_of(&a.kind) else { continue };
            if lod == 1 && fc == Fill::Building {
                continue;
            }
            let (t, ma) = if fc == Fill::Building { (tol * 0.6, 12.0) } else { (tol, min_area) };
            let slot = &mut fills.iter_mut().find(|x| x.0 == fc).unwrap().1;
            add_area(slot, a, t, ma);
        }
        let fills: Vec<(Fill, FillMesh)> = fills.into_iter().filter(|x| !x.1.idx.is_empty()).collect();
        let mut lines: Vec<(Line, Vec<Vec<[f32; 2]>>)> = LINES.iter().map(|l| (*l, Vec::new())).collect();
        for l in &f.lines {
            let Some(lc) = line_of(&l.kind) else { continue };
            if (far || lod == 1) && matches!(lc, Line::Minor | Line::Track | Line::Stream) {
                continue;
            }
            if far && lc == Line::Secondary {
                continue;
            }
            let s = simplify(&l.pts, tol);
            if s.len() >= 2 {
                lines.iter_mut().find(|x| x.0 == lc).unwrap().1.push(s.iter().map(|p| [p.x, p.y]).collect());
            }
        }
        let lines: Vec<(Line, Vec<Vec<[f32; 2]>>)> = lines.into_iter().filter(|x| !x.1.is_empty()).collect();
        if lod == 0 {
            out.fills = fills;
            out.lines = lines;
        } else {
            out.fills_lo = fills;
            out.lines_lo = lines;
        }
    }
    // Labels: places and named water.
    for layer in layers {
        let e = layer.extent as f32;
        let to = |p: [f32; 2]| lo + (hi - lo) * vec2(p[0] / e, p[1] / e);
        match layer.name.as_str() {
            "place" => {
                for ft in &layer.features {
                    if ft.geom_type != GeomType::Point {
                        continue;
                    }
                    let kind = match ft.str("class").unwrap_or("") {
                        "city" => LabelKind::City,
                        "town" => LabelKind::Town,
                        "village" => LabelKind::Village,
                        "suburb" | "quarter" | "borough" => LabelKind::Suburb,
                        _ => continue,
                    };
                    if far && kind > LabelKind::Town {
                        continue;
                    }
                    let name = ft.str("name:latin").or(ft.str("name_en")).or(ft.str("name")).unwrap_or("");
                    if name.is_empty() {
                        continue;
                    }
                    let rank = ft.num("rank").unwrap_or(10.0) as u16;
                    for part in &ft.parts {
                        for p in part {
                            if p[0] >= 0.0 && p[0] < e && p[1] >= 0.0 && p[1] < e {
                                let w = to(*p);
                                out.labels.push(Label { pos: [w.x, w.y], text: name.to_string(), kind, rank });
                            }
                        }
                    }
                }
            }
            "water_name" => {
                for ft in &layer.features {
                    let name = ft.str("name:latin").or(ft.str("name")).unwrap_or("");
                    if name.is_empty() {
                        continue;
                    }
                    // Points (lakes, seas) or the middle of a line (rivers).
                    let p = match ft.geom_type {
                        GeomType::Point => ft.parts.first().and_then(|v| v.first()).copied(),
                        GeomType::Line => ft.parts.first().and_then(|v| v.get(v.len() / 2)).copied(),
                        _ => None,
                    };
                    if let Some(p) = p.filter(|p| p[0] >= 0.0 && p[0] < e && p[1] >= 0.0 && p[1] < e) {
                        let w = to(p);
                        out.labels.push(Label { pos: [w.x, w.y], text: name.to_string(), kind: LabelKind::Water, rank: 12 });
                    }
                }
            }
            _ => {}
        }
    }
    out.verts = out.fills.iter().map(|f| f.1.verts.len()).sum::<usize>() + out.lines.iter().map(|l| l.1.iter().map(|v| v.len()).sum::<usize>()).sum::<usize>();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simplify_keeps_shape() {
        let pts: Vec<Vec2> = (0..=100).map(|i| vec2(i as f32, ((i as f32) * 0.3).sin() * 0.05)).collect();
        let s = simplify(&pts, 0.1);
        assert_eq!(s.len(), 2);
        let sq = vec![vec2(0.0, 0.0), vec2(5.0, 0.01), vec2(10.0, 0.0), vec2(10.0, 10.0), vec2(0.0, 10.0)];
        let s = simplify(&sq, 0.5);
        assert_eq!(s.len(), 4);
        let ring: Vec<Vec2> = (0..64).map(|i| {
            let a = i as f32 / 64.0 * std::f32::consts::TAU;
            vec2(a.cos(), a.sin()) * 100.0
        }).collect();
        let r = simplify_ring(&ring, 2.0);
        assert!(r.len() >= 8 && r.len() < 64, "{}", r.len());
        assert!((ring_area(&r).abs() - ring_area(&ring).abs()).abs() / ring_area(&ring).abs() < 0.05);
    }
}
