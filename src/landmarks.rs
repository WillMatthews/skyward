//! Landmark registry: hand-modelled (procedural) structures that replace the
//! OSM extrusions of famous buildings — London's skyline first.
//!
//! * A landmark is a row in a static table ([`all`]): position (lat / lon),
//!   heading, the local-frame area whose OSM buildings it replaces, a builder
//!   that writes its geometry / collision / lights in a local frame
//!   (`geom::Kit`), and optionally a builder for moving parts (the London
//!   Eye's wheel). Adding a city = adding a table (see `london.rs`).
//! * [`for_tile`] places the landmarks anchored in a z14 tile into that tile's
//!   frame (tile workers; deterministic, no I/O).
//! * [`animated`] builds the moving parts near the eye every frame,
//!   eye-relative (main thread).
//!
//! Local frame: metres, +y up, y = 0 at the ground reference (the caller adds
//! the terrain height sampled at the anchor), -z along `heading` (compass
//! degrees), +x to its right.

mod geom;
mod london;

use crate::lights::LightInst;
use crate::osm::{Proj, TileId};
use crate::scene::MeshData;
use glam::{vec2, vec3, Mat4, Vec2, Vec3};

pub use geom::Kit;

/// A landmark placed in one z14 map tile.
pub struct Placed {
    /// Geometry in the tile's frame (`Proj::tile_frame(id)`: x east, z SOUTH,
    /// metres from the tile's NW corner), y up with y = 0 at the landmark's
    /// ground reference — the caller adds the terrain height there.
    pub mesh: MeshData,
    /// Tile-frame polygons: OSM buildings whose footprint centroid lies inside
    /// one are NOT extruded (the landmark replaces them).
    pub suppress: Vec<Vec<Vec2>>,
    /// Collision: (footprint ring in tile frame, top height above y = 0).
    pub solids: Vec<(Vec<Vec2>, f32)>,
    /// Night lights (tile frame, y above y = 0): aircraft-warning reds etc.
    pub lights: Vec<LightInst>,
    /// Tile-frame point where the caller samples the ground height.
    pub anchor: Vec2,
    pub max_h: f32,
}

/// Which OSM buildings a landmark replaces (local frame).
#[derive(Clone, Copy, Debug)]
pub enum Suppress {
    /// x0, x1, z0, z1.
    Rect(f32, f32, f32, f32),
    /// Radius around the anchor.
    Circle(f32),
}

impl Suppress {
    fn ring(self) -> Vec<Vec2> {
        match self {
            Suppress::Rect(x0, x1, z0, z1) => geom::rect_xz(x0, x1, z0, z1),
            Suppress::Circle(r) => geom::ngon(Vec2::ZERO, r, 24, 0.0),
        }
    }
}

/// One row of the registry.
pub struct Landmark {
    #[allow(dead_code)]
    pub name: &'static str,
    pub lat: f64,
    pub lon: f64,
    /// Compass bearing (degrees) of the builder's local -z axis.
    pub heading: f32,
    pub suppress: Suppress,
    /// Static geometry, collision and lights in the local frame.
    pub build: fn(&mut Kit),
    /// Moving parts at time t (s), appended to `out` through `xf` (local →
    /// output frame).
    pub animate: Option<fn(out: &mut MeshData, xf: Mat4, t: f32)>,
}

/// Every city's table.
const CITIES: &[&[Landmark]] = &[london::LANDMARKS];

/// All registered landmarks.
pub fn all() -> impl Iterator<Item = &'static Landmark> {
    CITIES.iter().flat_map(|c| c.iter())
}

/// Moving parts are built within this distance (m) of the eye.
pub const ANIM_RANGE: f32 = 25_000.0;

impl Landmark {
    /// The z14 tile its anchor lies in.
    pub fn tile(&self) -> TileId {
        let (x, y) = TileId::frac(self.lat, self.lon, crate::osm::ZOOM);
        TileId::new(x.floor() as u32, y.floor() as u32)
    }
    /// Local → frame rotation (about +y).
    fn rot(&self) -> Mat4 {
        Mat4::from_rotation_y(-self.heading.to_radians())
    }
    /// Build and place in the frame of tile `id` (normally `self.tile()`).
    pub fn place(&self, id: TileId) -> Placed {
        let anchor = Proj::tile_frame(id).to_world(self.lat, self.lon);
        let xf = Mat4::from_translation(vec3(anchor.x, 0.0, anchor.y)) * self.rot();
        let p2 = |p: Vec2| {
            let q = xf.transform_point3(vec3(p.x, 0.0, p.y));
            vec2(q.x, q.z)
        };
        let mut k = Kit::default();
        (self.build)(&mut k);
        let mut mesh = MeshData::default();
        mesh.append(&k.m, xf);
        let max_h = mesh.verts.iter().map(|v| v.pos[1]).fold(0.0f32, f32::max);
        let lights = k
            .lights
            .into_iter()
            .map(|mut l| {
                l.pos = xf.transform_point3(Vec3::from(l.pos)).into();
                l
            })
            .collect();
        Placed {
            mesh,
            suppress: vec![self.suppress.ring().into_iter().map(p2).collect()],
            solids: k.solids.into_iter().map(|(r, top)| (r.into_iter().map(p2).collect(), top)).collect(),
            lights,
            anchor,
            max_h,
        }
    }
}

/// Landmarks whose anchor lies in this z14 tile (tile worker threads; cheap
/// and deterministic, no I/O).
pub fn for_tile(id: TileId) -> Vec<Placed> {
    if id.z != crate::osm::ZOOM {
        return Vec::new();
    }
    all().filter(|l| l.tile() == id).map(|l| l.place(id)).collect()
}

/// Moving parts (the London Eye's wheel and capsules, one turn per 30 min)
/// of landmarks within [`ANIM_RANGE`] of the eye, as one mesh in eye-relative
/// coordinates of the current frame `proj`. `ground(xz)` = ground height at
/// a current-frame point. Empty if none are near.
pub fn animated(proj: &Proj, eye: Vec3, time: f32, ground: &dyn Fn(Vec2) -> f32) -> MeshData {
    let mut out = MeshData::default();
    for l in all() {
        let Some(anim) = l.animate else { continue };
        let p = proj.to_world(l.lat, l.lon);
        if p.distance(vec2(eye.x, eye.z)) > ANIM_RANGE {
            continue;
        }
        // Same east-west scale as the static parts (built in the tile's
        // frame, placed with its exact map into `proj`).
        let sx = Proj::tile_frame(l.tile()).map_to(proj).sx as f32;
        let xf = Mat4::from_translation(vec3(p.x, ground(p), p.y) - eye) * Mat4::from_scale(vec3(sx, 1.0, 1.0)) * l.rot();
        anim(&mut out, xf, time);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_landmark_lands_in_its_tile() {
        for l in all() {
            let id = l.tile();
            let placed = for_tile(id);
            assert!(!placed.is_empty(), "{} not returned for its tile {:?}", l.name, id);
            // The anchor is inside the tile rectangle.
            let (lo, hi) = id.rect(&Proj::tile_frame(id));
            let a = Proj::tile_frame(id).to_world(l.lat, l.lon);
            assert!(a.x >= lo.x && a.x <= hi.x && a.y >= lo.y && a.y <= hi.y, "{} anchor {:?} outside {:?}", l.name, a, (lo, hi));
        }
        // A tile far away (Paris) has none; far (low-zoom) tiles never do.
        let (x, y) = TileId::frac(48.8566, 2.3522, 14);
        assert!(for_tile(TileId::new(x as u32, y as u32)).is_empty());
        let t = all().next().unwrap().tile();
        assert!(for_tile(TileId { z: 10, x: t.x >> 4, y: t.y >> 4 }).is_empty());
    }

    #[test]
    fn meshes_are_sane() {
        let mut total = 0;
        for l in all() {
            let p = l.place(l.tile());
            let tris = p.mesh.idx.len() / 3;
            total += tris;
            eprintln!("{:<40} {:>7} triangles  {:>3} solids  {:>3} lights  max_h {:.0} m", l.name, tris, p.solids.len(), p.lights.len(), p.max_h);
            assert!(tris > 50, "{}: {} triangles", l.name, tris);
            assert!(p.mesh.idx.iter().all(|&i| (i as usize) < p.mesh.verts.len()), "{}: bad index", l.name);
            for v in &p.mesh.verts {
                assert!(v.pos.iter().chain(&v.nrm).chain(&v.col).chain(&v.uv).all(|f| f.is_finite()), "{}: non-finite vertex", l.name);
                assert!(Vec3::from(v.pos).distance(vec3(p.anchor.x, 0.0, p.anchor.y)) < 400.0, "{}: vertex far from anchor", l.name);
            }
            assert!(p.max_h > 40.0 && p.max_h < 340.0, "{}: max_h {}", l.name, p.max_h);
            assert!(!p.solids.is_empty() && !p.suppress.is_empty());
            assert!(p.solids.iter().all(|(r, top)| r.len() >= 3 && top.is_finite()));
            assert!(p.lights.iter().all(|l| l.pos.iter().all(|f| f.is_finite())));
            // The suppression area covers the anchor.
            assert!(p.suppress.iter().any(|r| crate::osm::point_in_ring(p.anchor, r)), "{}", l.name);
        }
        eprintln!("total {} triangles", total);
        assert!(total < 400_000, "{} triangles in total", total);
    }

    #[test]
    fn eye_turns_near_and_vanishes_far() {
        let eye_lm = all().find(|l| l.animate.is_some()).unwrap();
        let proj = Proj::new(eye_lm.lat, eye_lm.lon);
        let eye = vec3(300.0, 100.0, 400.0);
        let a = animated(&proj, eye, 0.0, &|_| 0.0);
        let b = animated(&proj, eye, 60.0, &|_| 0.0);
        assert!(a.idx.len() > 1000);
        eprintln!("animated (London Eye wheel): {} triangles", a.idx.len() / 3);
        assert_eq!(a.verts.len(), b.verts.len());
        assert!(a.verts.iter().zip(&b.verts).any(|(p, q)| Vec3::from(p.pos).distance(Vec3::from(q.pos)) > 1.0), "wheel did not turn");
        assert!(a.verts.iter().all(|v| v.pos.iter().all(|f| f.is_finite())));
        // Every vertex within the 135 m wheel around the hub.
        let hub = proj.to_world(eye_lm.lat, eye_lm.lon);
        assert!(a.verts.iter().all(|v| (Vec3::from(v.pos) + eye).distance(vec3(hub.x, 69.0, hub.y)) < 70.0));
        let far = animated(&proj, vec3(40_000.0, 500.0, 0.0), 0.0, &|_| 0.0);
        assert!(far.idx.is_empty());
        // Cost: well under the per-frame budget (generous bound for debug / CI).
        let t0 = web_time::Instant::now();
        for i in 0..20 {
            std::hint::black_box(animated(&proj, eye, i as f32, &|_| 0.0));
        }
        let per = t0.elapsed().as_secs_f64() / 20.0;
        eprintln!("animated(): {:.3} ms per call", per * 1e3);
        assert!(per < if cfg!(debug_assertions) { 0.05 } else { 0.002 }, "animated() took {:.3} ms", per * 1e3);
    }
}
