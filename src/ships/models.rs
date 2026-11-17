//! Ship hulls: the imported FlightGear AI ship models (`assets/ships/<id>/`,
//! `tools/fgimport/ships/`) and a procedural workboat / patrol hull, all as
//! [`AircraftAsset`]s so they draw through the textured-model renderer
//! (gfx_aircraft.rs: the scene's lighting, shadows, reflections, TAA).
//!
//! Body frame as for the aircraft: +Z bow, +Y up, +X port; the importer's
//! origin is FlightGear's, which for these AI ships is on the waterline
//! (y = 0). Models load in the background on first use (natively a thread;
//! in the browser after the directory has downloaded); until then a ship
//! is drawn with the procedural hull.

use super::types::Model;
use crate::aircraft_asset::{AircraftAsset, AnimState, Chunk, Material, Node, Pose, Vertex};
use crate::gfx_aircraft::{AircraftFrame, ChunkDraw};
use glam::{Mat4, Vec3, Vec4, vec3};
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex};

/// A loaded hull, posed once (the ships have no moving parts worth animating).
pub struct ShipModel {
    pub asset: Arc<AircraftAsset>,
    world: Vec<Mat4>,
    visible: Vec<bool>,
    /// Exterior bounds, body frame.
    pub lo: Vec3,
    pub hi: Vec3,
}

impl ShipModel {
    /// Pose the asset and square it up: turned by `yaw_deg` about the
    /// vertical (models built along another axis) and centred midships on
    /// the waterline origin.
    pub fn new(asset: AircraftAsset, yaw_deg: f32) -> ShipModel {
        let ch = vec![0.0; asset.channels.len()];
        let mut st = AnimState::default();
        let mut pose = Pose::default();
        asset.pose(&ch, &mut st, 0.0, &mut pose);
        let bounds = |fix: Mat4| {
            let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
            for c in asset.chunks.iter().filter(|c| pose.visible[c.node]) {
                let m = fix * pose.world[c.node];
                for k in c.first..c.first + c.count {
                    let p = m.transform_point3(Vec3::from(asset.vertices[(asset.indices[k as usize] + c.base_vertex) as usize].pos));
                    lo = lo.min(p);
                    hi = hi.max(p);
                }
            }
            if lo.x > hi.x { (Vec3::splat(-1.0), Vec3::splat(1.0)) } else { (lo, hi) }
        };
        let turn = Mat4::from_rotation_y(yaw_deg.to_radians());
        let (lo, hi) = bounds(turn);
        let fix = Mat4::from_translation(vec3(-0.5 * (lo.x + hi.x), 0.0, -0.5 * (lo.z + hi.z))) * turn;
        let (lo, hi) = bounds(fix);
        let world = pose.world.iter().map(|w| fix * *w).collect();
        ShipModel { asset: Arc::new(asset), world, visible: pose.visible, lo, hi }
    }

    /// Length overall (bow to stern), m.
    pub fn length(&self) -> f32 {
        (self.hi.z - self.lo.z).max(1.0)
    }

    /// Midships (body z; 0 once squared up).
    pub fn centre_z(&self) -> f32 {
        0.5 * (self.hi.z + self.lo.z)
    }

    /// Draw list: `body` = body → eye-relative world (scale included).
    pub fn frame(&self, body: Mat4) -> AircraftFrame {
        let draws = self
            .asset
            .chunks
            .iter()
            .enumerate()
            .filter(|(_, c)| self.visible[c.node])
            .map(|(i, c)| ChunkDraw { chunk: i as u32, model: body * self.world[c.node], emit: 0.0, uv: crate::aircraft_asset::UV_IDENTITY })
            .collect();
        AircraftFrame { asset: self.asset.clone(), draws, parts: None, dynamics: None, displays: None, cockpit: false }
    }
}

enum Slot {
    Loading,
    Ready(Arc<ShipModel>),
    Failed,
}

static SLOTS: LazyLock<Mutex<HashMap<Model, Slot>>> = LazyLock::new(Default::default);

/// Load an imported hull (blocking).
pub fn load(m: Model) -> Result<ShipModel, String> {
    let Some(dir) = m.dir() else { return Ok(procedural(m)) };
    let raw = crate::assets::read(&format!("ships/{dir}/model.skac"))?;
    // (Textures resolve to ships/<dir>/ through the aircraft loader.)
    let a = AircraftAsset::parse_unfixed(&raw, &format!("../ships/{dir}"))?;
    if a.vertices.is_empty() {
        return Err("empty model".into());
    }
    Ok(ShipModel::new(a, m.yaw_fix()))
}

/// The hull of a model if loaded; starts loading it otherwise (None
/// meanwhile, and for good if it failed: the caller draws the fallback).
pub fn get(m: Model) -> Option<Arc<ShipModel>> {
    if m.dir().is_none() {
        return Some(procedural_cached(m));
    }
    {
        let mut s = SLOTS.lock().unwrap();
        match s.get(&m) {
            Some(Slot::Ready(x)) => return Some(x.clone()),
            Some(_) => return None,
            None => {
                #[cfg(target_arch = "wasm32")]
                {
                    let dir = format!("ships/{}/", m.dir().unwrap());
                    match crate::platform::web::dir_state(&dir) {
                        crate::platform::web::DirState::Ready => {}
                        crate::platform::web::DirState::Absent => {
                            crate::platform::web::preload_dir(&dir);
                            return None;
                        }
                        crate::platform::web::DirState::Loading(..) => return None,
                        crate::platform::web::DirState::Failed(_) => {
                            s.insert(m, Slot::Failed);
                            return None;
                        }
                    }
                }
                s.insert(m, Slot::Loading);
            }
        }
    }
    // Headless captures and tests: at once (the shot must show it).
    if cfg!(test) || super::inline_loads() {
        finish(m, load(m));
        return get(m);
    }
    if !crate::platform::spawn("ship-model", move || finish(m, load(m))) {
        SLOTS.lock().unwrap().insert(m, Slot::Failed);
    }
    None
}

fn finish(m: Model, r: Result<ShipModel, String>) {
    let slot = match r {
        Ok(x) => Slot::Ready(Arc::new(x)),
        Err(e) => {
            eprintln!("ships: model {:?}: {e}", m);
            Slot::Failed
        }
    };
    SLOTS.lock().unwrap().insert(m, slot);
}

/// True when no model is loading (shots wait).
pub fn idle() -> bool {
    !SLOTS.lock().unwrap().values().any(|s| matches!(s, Slot::Loading))
}

fn procedural_cached(m: Model) -> Arc<ShipModel> {
    static P: LazyLock<Mutex<HashMap<Model, Arc<ShipModel>>>> = LazyLock::new(Default::default);
    P.lock().unwrap().entry(m).or_insert_with(|| Arc::new(procedural(m))).clone()
}

// ---------------------------------------------------------------- procedural hull

#[derive(Default)]
struct Builder {
    /// Per material: vertices, indices.
    parts: Vec<(Vec<Vertex>, Vec<u32>)>,
}

impl Builder {
    fn quad(&mut self, mat: usize, a: Vec3, b: Vec3, c: Vec3, d: Vec3) {
        while self.parts.len() <= mat {
            self.parts.push(Default::default());
        }
        let n = (c - a).cross(d - b).normalize_or(Vec3::Y);
        let (v, i) = &mut self.parts[mat];
        let base = v.len() as u32;
        for p in [a, b, c, d] {
            v.push(Vertex { pos: p.into(), nrm: n.into(), uv: [0.0, 0.0] });
        }
        i.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// Axis-aligned box from `lo` to `hi`, all faces.
    fn cuboid(&mut self, mat: usize, lo: Vec3, hi: Vec3) {
        let c = |x: f32, y: f32, z: f32| vec3(x, y, z);
        let (x0, y0, z0, x1, y1, z1) = (lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
        self.quad(mat, c(x0, y1, z0), c(x0, y1, z1), c(x1, y1, z1), c(x1, y1, z0)); // top
        self.quad(mat, c(x0, y0, z1), c(x0, y1, z1), c(x0, y1, z0), c(x0, y0, z0)); // +x (port, x0 < x1: -x side)
        self.quad(mat, c(x1, y0, z0), c(x1, y1, z0), c(x1, y1, z1), c(x1, y0, z1));
        self.quad(mat, c(x0, y0, z1), c(x1, y0, z1), c(x1, y1, z1), c(x0, y1, z1)); // bow face
        self.quad(mat, c(x1, y0, z0), c(x0, y0, z0), c(x0, y1, z0), c(x1, y1, z0)); // aft face
    }
}

/// A small steel hull (tug / workboat / patrol boat), 30 m long; scaled to
/// the ship's length when drawn.
pub fn procedural(m: Model) -> ShipModel {
    let patrol = m == Model::Patrol;
    let mat = |c: [f32; 3], spec: f32| Material {
        col: Vec4::new(c[0], c[1], c[2], 1.0),
        emis: Vec3::ZERO,
        spec: Vec3::splat(spec),
        shininess: 30.0,
        tex: None,
        translucent: false,
        alpha_tex: false,
        two_sided: true,
    };
    let (top, house, deck) = if patrol { ([0.36, 0.38, 0.40], [0.42, 0.44, 0.46], [0.22, 0.23, 0.24]) } else { ([0.55, 0.08, 0.06], [0.92, 0.92, 0.88], [0.20, 0.26, 0.22]) };
    let materials = vec![mat([0.30, 0.07, 0.05], 0.1), mat(top, 0.25), mat(deck, 0.1), mat(house, 0.3), mat([0.06, 0.06, 0.07], 0.4)];
    let (antifoul, side, deck_m, house_m, dark) = (0, 1, 2, 3, 4);
    let len = 30.0f32;
    let half_b = len * if patrol { 0.07 } else { 0.17 };
    let (keel, wl, deck_y) = (-len * 0.1, 0.0, len * if patrol { 0.07 } else { 0.08 });
    // Stations bow (+z) → stern: half beam at the deck and the waterline.
    let n = 9;
    let st: Vec<(f32, f32, f32)> = (0..=n)
        .map(|i| {
            let t = i as f32 / n as f32; // 0 bow, 1 stern
            let z = len * (0.5 - t);
            let fine = if t < 0.35 { (t / 0.35).powf(0.55) } else { 1.0 };
            let aft = if t > 0.85 { 1.0 - (t - 0.85) * 0.8 } else { 1.0 };
            (z, half_b * fine.max(0.02) * aft, half_b * (fine * 0.92).max(0.0) * aft * 0.9)
        })
        .collect();
    let mut b = Builder::default();
    for w in st.windows(2) {
        let ((z0, d0, w0), (z1, d1, w1)) = (w[0], w[1]);
        for s in [1.0f32, -1.0] {
            let p = |x: f32, y: f32, z: f32| vec3(x * s, y, z);
            // Topsides (waterline → deck) and the bottom (keel → waterline).
            let (a, bb, c, d) = (p(w0, wl, z0), p(w1, wl, z1), p(d1, deck_y, z1), p(d0, deck_y, z0));
            let (k0, k1) = (p(w0 * 0.25, keel, z0), p(w1 * 0.25, keel, z1));
            if s > 0.0 {
                b.quad(side, a, bb, c, d);
                b.quad(antifoul, k0, k1, bb, a);
            } else {
                b.quad(side, d, c, bb, a);
                b.quad(antifoul, a, bb, k1, k0);
            }
        }
        // Deck.
        b.quad(deck_m, vec3(-d0, deck_y, z0), vec3(d0, deck_y, z0), vec3(d1, deck_y, z1), vec3(-d1, deck_y, z1));
    }
    // Transom.
    let (zs, ds, ws) = st[n];
    b.quad(side, vec3(ds, deck_y, zs), vec3(-ds, deck_y, zs), vec3(-ws, wl, zs), vec3(ws, wl, zs));
    b.quad(antifoul, vec3(ws, wl, zs), vec3(-ws, wl, zs), vec3(-ws * 0.25, keel, zs), vec3(ws * 0.25, keel, zs));
    // Deckhouse, wheelhouse, mast / funnel.
    let hb = half_b * 0.62;
    if patrol {
        b.cuboid(house_m, vec3(-hb, deck_y, -len * 0.1), vec3(hb, deck_y + len * 0.09, len * 0.12));
        b.cuboid(house_m, vec3(-hb * 0.7, deck_y + len * 0.09, -len * 0.02), vec3(hb * 0.7, deck_y + len * 0.15, len * 0.1));
        b.cuboid(dark, vec3(-0.25, deck_y + len * 0.15, 0.0), vec3(0.25, deck_y + len * 0.3, 0.5));
    } else {
        b.cuboid(house_m, vec3(-hb, deck_y, -len * 0.05), vec3(hb, deck_y + len * 0.1, len * 0.22));
        b.cuboid(house_m, vec3(-hb * 0.8, deck_y + len * 0.1, len * 0.06), vec3(hb * 0.8, deck_y + len * 0.17, len * 0.2));
        b.cuboid(dark, vec3(-hb * 0.85, deck_y + len * 0.13, len * 0.199), vec3(hb * 0.85, deck_y + len * 0.16, len * 0.205));
        b.cuboid(dark, vec3(-0.6, deck_y + len * 0.1, -len * 0.04), vec3(0.6, deck_y + len * 0.22, len * 0.02));
        b.cuboid(dark, vec3(-0.2, deck_y + len * 0.17, len * 0.12), vec3(0.2, deck_y + len * 0.3, len * 0.14));
    }
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut chunks = Vec::new();
    for (material, (v, i)) in b.parts.into_iter().enumerate() {
        if i.is_empty() {
            continue;
        }
        let base = vertices.len() as u32;
        let first = indices.len() as u32;
        vertices.extend(v);
        indices.extend(i);
        chunks.push(Chunk {
            node: 0,
            material,
            base_vertex: base,
            first,
            count: indices.len() as u32 - first,
            interior: false,
            emit: None,
            uv: Vec::new(),
            display: None,
            backlight: false,
        });
    }
    let radius = vertices.iter().map(|v| Vec3::from_array(v.pos).length()).fold(0.0, f32::max);
    let a = AircraftAsset {
        id: if patrol { "ship-patrol".into() } else { "ship-workboat".into() },
        name: if patrol { "Patrol boat".into() } else { "Workboat".into() },
        nodes: vec![Node { name: "hull".into(), parent: -1, pre: Mat4::IDENTITY, anim: None }],
        materials,
        textures: Vec::new(),
        chunks,
        vertices,
        indices,
        eye: None,
        radius,
        lights: Vec::new(),
        channels: Vec::new(),
    };
    ShipModel::new(a, 0.0)
}
