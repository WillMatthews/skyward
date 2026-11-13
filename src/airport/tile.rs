//! Tile-side airport detail (runs in the tile workers, from `scene.rs`):
//! which aerodrome an area belongs to (OurAirports owner and class), how
//! wide its taxiways are, airside building styles (glass terminals,
//! corrugated hangars with doors, control towers), and the per-tile
//! summary ([`AeroTile`]) the main thread assembles airport layouts from.

use super::mesh;
use crate::airports::Kind as AKind;
use crate::osm::{Area, Kind, Proj, TileFeatures, TileId, point_in_ring};
use crate::scene::{MeshData, mat};
use glam::{Vec2, vec2, vec3};
use std::sync::Arc;

/// Airside building classes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BClass {
    Terminal,
    Hangar,
    Tower,
    Airside,
}

impl BClass {
    pub fn code(self) -> u8 {
        self as u8
    }
    pub fn from_code(c: u8) -> BClass {
        match c {
            0 => BClass::Terminal,
            1 => BClass::Hangar,
            2 => BClass::Tower,
            _ => BClass::Airside,
        }
    }
}

/// An aerodrome polygon piece (clipped to the tile) and its airport.
pub struct Piece {
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
    lo: Vec2,
    hi: Vec2,
    /// OurAirports ident, class, has a paved runway (None: no airport near).
    pub owner: Option<(String, AKind, bool)>,
}

impl Piece {
    fn contains(&self, p: Vec2) -> bool {
        p.x >= self.lo.x && p.x <= self.hi.x && p.y >= self.lo.y && p.y <= self.hi.y && point_in_ring(p, &self.outer) && !self.holes.iter().any(|h| point_in_ring(p, h))
    }
    fn near(&self, p: Vec2, d: f32) -> bool {
        if p.x < self.lo.x - d || p.x > self.hi.x + d || p.y < self.lo.y - d || p.y > self.hi.y + d {
            return false;
        }
        self.contains(p) || ring_dist(&self.outer, p) < d
    }
}

/// The aerodromes of one tile.
pub struct Ctx {
    pub pieces: Vec<Piece>,
    /// Gate points (tile frame).
    gates: Vec<Vec2>,
    /// Apron rings with bounds.
    aprons: Vec<(Vec<Vec2>, Vec2, Vec2)>,
}

/// Owner of an aerodrome piece: the nearest OurAirports airport (with
/// runways, not a seaplane base) within the piece's size + 2.5 km.
pub fn owner_of(db: &crate::airports::Db, lat: f64, lon: f64, radius_km: f64) -> Option<(String, AKind, bool)> {
    db.near(lat, lon, radius_km + 2.5)
        .into_iter()
        .map(|(i, _)| &db.airports[i])
        .find(|a| a.kind != AKind::Seaplane && !a.runways.is_empty())
        .map(|a| (a.ident.clone(), a.kind, a.runways.iter().any(|r| r.surface.paved())))
}

impl Ctx {
    pub fn new(id: TileId, proj: &Proj, f: &TileFeatures) -> Ctx {
        let _ = id;
        let db = crate::terrain::airport_db();
        let pieces = f
            .areas
            .iter()
            .filter(|a| a.kind == Kind::Aerodrome && a.outer.len() >= 3)
            .map(|a| {
                let (lo, hi) = mesh::bbox(&a.outer);
                let c = (lo + hi) * 0.5;
                let (lat, lon) = proj.to_latlon(c);
                let owner = db.as_ref().and_then(|db| owner_of(db, lat, lon, ((hi - lo).length() * 0.5 / 1000.0) as f64));
                Piece { outer: a.outer.clone(), holes: a.holes.clone(), lo, hi, owner }
            })
            .collect();
        let aprons = f
            .areas
            .iter()
            .filter(|a| a.kind == Kind::Apron)
            .map(|a| {
                let (lo, hi) = mesh::bbox(&a.outer);
                (a.outer.clone(), lo, hi)
            })
            .collect();
        Ctx { pieces, gates: f.aero.gates.iter().map(|g| g.0).collect(), aprons }
    }

    pub fn is_empty(&self) -> bool {
        self.pieces.is_empty()
    }

    /// The aerodrome piece containing `p`.
    pub fn at(&self, p: Vec2) -> Option<&Piece> {
        self.pieces.iter().find(|pc| pc.contains(p))
    }

    /// Inside an aerodrome whose airport detail layer draws the markings and
    /// lights (see airport/layout.rs): the generic ones are left out there.
    pub fn owned(&self, p: Vec2) -> bool {
        self.pieces.iter().any(|pc| pc.owner.is_some() && pc.near(p, 60.0))
    }

    pub fn class_at(&self, p: Vec2) -> Option<AKind> {
        self.pieces.iter().find(|pc| pc.near(p, 60.0)).and_then(|pc| pc.owner.as_ref().map(|o| o.1))
    }

    /// Taxiway pavement width (m) by airport class, None = no pavement (grass fields).
    pub fn taxiway_width(&self, pts: &[Vec2]) -> Option<f32> {
        let mid = pts[pts.len() / 2];
        match self.pieces.iter().find(|pc| pc.near(mid, 80.0)).and_then(|pc| pc.owner.as_ref()) {
            Some((_, AKind::Large, _)) => Some(23.0),
            Some((_, AKind::Medium, _)) => Some(18.0),
            Some((_, _, true)) => Some(10.5),
            Some((_, _, false)) => None,
            None => Some(18.0),
        }
    }

    fn near_apron(&self, ring: &[Vec2], d: f32) -> bool {
        let (lo, hi) = mesh::bbox(ring);
        self.aprons.iter().any(|(r, alo, ahi)| {
            alo.x <= hi.x + d && ahi.x >= lo.x - d && alo.y <= hi.y + d && ahi.y >= lo.y - d && ring.iter().any(|p| point_in_ring(*p, r) || ring_dist(r, *p) < d)
        })
    }

    /// Airside class of a building inside an aerodrome (None: landside / no aerodrome).
    pub fn classify(&self, a: &Area, height: f32) -> Option<BClass> {
        let c = a.outer.iter().copied().sum::<Vec2>() / a.outer.len().max(1) as f32;
        let pc = self.pieces.iter().find(|pc| pc.contains(c))?;
        let kind = pc.owner.as_ref().map_or(AKind::Small, |o| o.1);
        let area = mesh::ring_area(&a.outer).abs();
        if height >= 14.0 && area < 450.0 && height > 1.8 * area.sqrt() {
            return Some(BClass::Tower);
        }
        let gate = self.gates.iter().any(|g| ring_dist(&a.outer, *g) < 45.0 || point_in_ring(*g, &a.outer));
        let apron = self.near_apron(&a.outer, 30.0);
        if (gate && area > 1200.0) || (apron && kind != AKind::Small && area > 9000.0 && height >= 8.0) {
            return Some(BClass::Terminal);
        }
        if apron && (200.0..30000.0).contains(&area) && height <= 32.0 {
            return Some(BClass::Hangar);
        }
        Some(BClass::Airside)
    }

    /// Build an airside building (heights relative to its base, like
    /// `scene::building`). Returns the light (obstruction lamp) offsets.
    pub fn building(&self, m: &mut MeshData, a: &Area, class: BClass, height: f32, min_h: f32) -> Vec<glam::Vec3> {
        let seed = crate::scene::hash(a.key);
        match class {
            BClass::Tower => {
                let (lo, hi) = mesh::bbox(&a.outer);
                let c = (lo + hi) * 0.5;
                let r = (mesh::ring_area(&a.outer).abs() / std::f32::consts::PI).sqrt();
                let (t, top) = mesh::control_tower(height, r * 0.8, seed);
                m.append(&t, glam::Mat4::from_translation(vec3(c.x, 0.0, c.y)));
                vec![top + vec3(c.x, 0.0, c.y)]
            }
            BClass::Terminal => {
                // One glazing style per airport (parts of a building may overlap).
                let c = a.outer.iter().copied().sum::<Vec2>() / a.outer.len().max(1) as f32;
                let owner = self.at(c).and_then(|p| p.owner.as_ref()).map_or(0, |o| o.0.bytes().fold(7u64, |h, b| h.wrapping_mul(131).wrapping_add(b as u64)));
                mesh::terminal(m, &a.outer, &a.holes, &a.border, height.max(12.0), min_h, crate::scene::hash(owner));
                Vec::new()
            }
            BClass::Hangar => {
                let tones: [[f32; 3]; 5] = [[0.72, 0.74, 0.76], [0.62, 0.66, 0.70], [0.58, 0.62, 0.56], [0.80, 0.80, 0.78], [0.45, 0.52, 0.62]];
                let col = tones[(seed % 5) as usize];
                match rect_plan(a) {
                    Some(q) if min_h <= 0.5 => {
                        let short = (q[3] - q[0]).length();
                        let rise = (short * 0.11).clamp(1.0, 7.0);
                        let eaves = (height - rise * 0.6).clamp(5.0, 26.0);
                        // Doors on the side facing the apron.
                        let door = (0..4).filter(|&i| {
                            let (p, q2) = (q[i], q[(i + 1) % 4]);
                            let mid = (p + q2) * 0.5;
                            let c = (q[0] + q[1] + q[2] + q[3]) * 0.25;
                            let out = (mid - c).normalize_or_zero();
                            self.aprons.iter().any(|(r, _, _)| point_in_ring(mid + out * 25.0, r) || ring_dist(r, mid + out * 10.0) < 15.0)
                        });
                        let door = door.max_by(|&i, &j| q[i].distance(q[(i + 1) % 4]).total_cmp(&q[j].distance(q[(j + 1) % 4])));
                        mesh::hangar(m, q, eaves, rise, door, col, seed);
                    }
                    _ => {
                        m.mat = mat::CLADDING;
                        walls(m, a, min_h, height, col);
                        m.mat = mat::ROOF;
                        m.flat_poly(&a.outer, &a.holes, height, glam::Vec3::Y, [col[0] * 0.8, col[1] * 0.8, col[2] * 0.82]);
                    }
                }
                Vec::new()
            }
            BClass::Airside => {
                let area = mesh::ring_area(&a.outer).abs();
                if area > 1500.0 && height < 16.0 {
                    // Cargo sheds, maintenance blocks: clad walls, flat roof.
                    let col = [[0.76, 0.77, 0.78], [0.66, 0.70, 0.74], [0.82, 0.80, 0.74]][(seed % 3) as usize];
                    m.mat = mat::CLADDING;
                    walls(m, a, min_h, height, col);
                    m.mat = mat::ROOF;
                    m.flat_poly(&a.outer, &a.holes, height, glam::Vec3::Y, [0.55, 0.56, 0.57]);
                } else {
                    let col = [[0.84, 0.84, 0.82], [0.74, 0.76, 0.78], [0.80, 0.78, 0.72], [0.70, 0.70, 0.70]][(seed % 4) as usize];
                    crate::scene::building(m, a, height, min_h, Some(col));
                }
                Vec::new()
            }
        }
    }
}

/// Plain walls over every non-seam edge (uv = metres along, height).
fn walls(m: &mut MeshData, a: &Area, y0: f32, y1: f32, col: [f32; 3]) {
    let rings = std::iter::once((&a.outer, &a.border, false)).chain(a.holes.iter().zip(&a.hole_border).map(|(h, b)| (h, b, true)));
    let mut u = 0.0;
    for (pts, border, hole) in rings {
        let n = pts.len();
        let s = if (mesh::ring_area(pts) > 0.0) != hole { 1.0 } else { -1.0 };
        for i in 0..n {
            let (p, q) = (pts[i], pts[(i + 1) % n]);
            let e = q - p;
            let len = e.length();
            if border.get(i).copied().unwrap_or(false) {
                u += len;
                continue;
            }
            let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * s;
            let i0 = m.v_uv(vec3(p.x, y0, p.y), nrm, col, [u, y0]);
            m.v_uv(vec3(q.x, y0, q.y), nrm, col, [u + len, y0]);
            m.v_uv(vec3(q.x, y1, q.y), nrm, col, [u + len, y1]);
            m.v_uv(vec3(p.x, y1, p.y), nrm, col, [u, y1]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
            u += len;
        }
    }
}

/// Oriented rectangle for a near-rectangular footprint (≥ 88 % fill, no
/// holes, no seam edges): corners with edge 0→1 along the long side.
fn rect_plan(a: &Area) -> Option<[Vec2; 4]> {
    if !a.holes.is_empty() || a.border.iter().any(|b| *b) || a.outer.len() < 4 {
        return None;
    }
    let pts = &a.outer;
    let n = pts.len();
    let (mut best, mut dir) = (0.0, Vec2::X);
    for i in 0..n {
        let e = pts[(i + 1) % n] - pts[i];
        if e.length() > best {
            best = e.length();
            dir = e / best;
        }
    }
    let side = vec2(-dir.y, dir.x);
    let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
    for p in pts {
        let q = vec2(p.dot(dir), p.dot(side));
        lo = lo.min(q);
        hi = hi.max(q);
    }
    let ext = hi - lo;
    let area = mesh::ring_area(pts).abs();
    if area < 0.88 * ext.x * ext.y || ext.min_element() < 6.0 {
        return None;
    }
    let w = |x: f32, y: f32| dir * x + side * y;
    let mut q = [w(lo.x, lo.y), w(hi.x, lo.y), w(hi.x, hi.y), w(lo.x, hi.y)];
    if ext.y > ext.x {
        q = [q[1], q[2], q[3], q[0]];
    }
    Some(q)
}

pub fn ring_dist(r: &[Vec2], p: Vec2) -> f32 {
    let n = r.len();
    (0..n).map(|i| mesh::seg_dist(p, r[i], r[(i + 1) % n])).fold(f32::MAX, f32::min)
}

// ---------------------------------------------------------------- summary

/// A polygon (tile frame).
#[derive(Clone, Debug, Default)]
pub struct Poly {
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
}

/// A centreline piece (tile frame), cut flags as in `osm::Line`.
#[derive(Clone, Debug, Default)]
pub struct Path {
    pub pts: Vec<Vec2>,
    pub cut: [bool; 2],
    pub name: String,
}

/// An aerodrome piece and its airport ident ("" = none).
#[derive(Clone, Debug, Default)]
pub struct AeroPiece {
    pub poly: Poly,
    pub owner: String,
}

/// An airside building (tile frame): footprint, class, height and base.
#[derive(Clone, Debug, Default)]
pub struct AeroBuilding {
    pub outer: Vec<Vec2>,
    pub class: u8,
    pub height: f32,
    pub base: f32,
}

/// What a tile contributes to its airports' layouts (airport/layout.rs).
#[derive(Clone, Debug)]
pub struct AeroTile {
    pub id: TileId,
    pub pieces: Vec<AeroPiece>,
    pub aprons: Vec<Poly>,
    pub taxiways: Vec<Path>,
    pub gates: Vec<(Vec2, String)>,
    pub service: Vec<Path>,
    pub buildings: Vec<AeroBuilding>,
}

impl Ctx {
    /// The tile's summary, if it holds any owned aerodrome.
    pub fn summary(&self, id: TileId, f: &TileFeatures, buildings: Vec<AeroBuilding>) -> Option<Arc<AeroTile>> {
        if !self.pieces.iter().any(|p| p.owner.is_some()) {
            return None;
        }
        let near_any = |p: Vec2| self.pieces.iter().any(|pc| pc.owner.is_some() && pc.near(p, 150.0));
        let t = AeroTile {
            id,
            pieces: self
                .pieces
                .iter()
                .map(|p| AeroPiece { poly: Poly { outer: p.outer.clone(), holes: p.holes.clone() }, owner: p.owner.as_ref().map(|o| o.0.clone()).unwrap_or_default() })
                .collect(),
            aprons: f.areas.iter().filter(|a| a.kind == Kind::Apron && near_any(a.outer[0])).map(|a| Poly { outer: a.outer.clone(), holes: a.holes.clone() }).collect(),
            taxiways: f.aero.taxiways.iter().filter(|t| t.0.iter().any(|p| near_any(*p))).map(|t| Path { pts: t.0.clone(), cut: t.1, name: t.2.clone() }).collect(),
            gates: f.aero.gates.iter().filter(|g| near_any(g.0)).cloned().collect(),
            service: f
                .aero
                .service
                .iter()
                .filter(|s| s.0.iter().any(|p| self.pieces.iter().any(|pc| pc.owner.is_some() && pc.contains(*p))))
                .map(|s| Path { pts: s.0.clone(), cut: s.1, name: String::new() })
                .collect(),
            buildings,
        };
        Some(Arc::new(t))
    }
}

crate::wire_struct!(Poly { outer, holes });
crate::wire_struct!(Path { pts, cut, name });
crate::wire_struct!(AeroPiece { poly, owner });
crate::wire_struct!(AeroBuilding { outer, class, height, base });
crate::wire_struct!(AeroTile { id, pieces, aprons, taxiways, gates, service, buildings });

impl crate::platform::wire::Wire for [bool; 2] {
    fn put(&self, w: &mut crate::platform::wire::W) {
        self[0].put(w);
        self[1].put(w);
    }
    fn get(r: &mut crate::platform::wire::R) -> crate::platform::wire::Res<Self> {
        Ok([bool::get(r)?, bool::get(r)?])
    }
}
