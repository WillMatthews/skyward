//! Aircraft models: real FlightGear aircraft imported by `tools/fgimport`
//! into `assets/aircraft/<id>/` (see `aircraft_asset.rs` for the format and
//! `assets/aircraft/<id>/CREDITS.md` for sources and licences).
//!
//! | id | aircraft | source |
//! |---|---|---|
//! | `kestrel` | Cessna 172P Skyhawk | c172p-team/c172p |
//! | `swift` | Schleicher ASK 21 | FGAddon ASK21 |
//! | `peregrine` | Supermarine Spitfire Mk IX | FGAddon spitfireIX |
//! | `lancer` | General Dynamics F-16 | FGAddon f16 |
//! | `aerolink` | ATR 72-500 | FGAddon ATR-72-500 |
//! | `meridian190` | Embraer E190 | E-jet-family-YV |
//! | `meridian320` | Airbus A320-200 | FGAddon A320-family |
//!
//! Body frame: metres, +Z nose, +Y up, +X left wing, origin = CG (as
//! `flight.rs`). The model is a node tree with FlightGear animations mapped
//! onto Skyward state through named channels ([`channel_value`]). The app
//! renders it with `gfx_aircraft.rs`; nav / beacon / strobe glows and the
//! afterburner flame are generated here as small emissive meshes.

use crate::aircraft_asset::{AircraftAsset, AnimState, Pose};
use crate::flight::{Aircraft, AircraftDef, Controls, EngineKind};
use crate::gfx_aircraft::{AircraftFrame, ChunkDraw};
use crate::scene::{mat, MeshData};
use glam::{vec3, Mat4, Vec3};
use std::sync::Arc;

pub struct AircraftModel {
    pub asset: Option<Arc<AircraftAsset>>,
    /// Cockpit eye point (body frame; x > 0 = left seat).
    pub eye: Vec3,
    /// Bounding radius of the exterior, m.
    pub radius: f32,
    /// Exterior bounding box (body frame, gear down).
    pub bounds: (Vec3, Vec3),
    /// Nose (or steerable) wheel contact, set by the app from the flight model.
    pub nose_gear: Option<Vec3>,
    state: AnimState,
    pose: Pose,
    chan: Vec<f32>,
    last_time: Option<f32>,
}

/// Load the model for an aircraft id (see the table above).
pub fn build(id: &str, def_eye: Vec3) -> AircraftModel {
    // Loaded once per process (decoding the textures takes a moment).
    static CACHE: std::sync::Mutex<Vec<(String, Arc<AircraftAsset>)>> = std::sync::Mutex::new(Vec::new());
    let cached = CACHE.lock().unwrap().iter().find(|(k, _)| k == id).map(|(_, a)| a.clone());
    let loaded = match cached {
        Some(a) => Ok(a),
        None => AircraftAsset::load(id).map(|a| {
            let a = Arc::new(a);
            CACHE.lock().unwrap().push((id.to_string(), a.clone()));
            a
        }),
    };
    match loaded {
        Ok(a) => {
            let eye = a.eye.unwrap_or(def_eye);
            let radius = a.radius.max(1.0);
            let n = a.channels.len();
            let bounds = exterior_bounds(&a);
            AircraftModel { asset: Some(a), eye, radius, bounds, nose_gear: None, state: AnimState::default(), pose: Pose::default(), chan: vec![0.0; n], last_time: None }
        }
        Err(e) => {
            eprintln!("aircraft model {id}: {e}");
            AircraftModel { asset: None, eye: def_eye, radius: 8.0, bounds: (Vec3::splat(-4.0), Vec3::splat(4.0)), nose_gear: None, state: AnimState::default(), pose: Pose::default(), chan: Vec::new(), last_time: None }
        }
    }
}

/// Everything the animation channels are computed from.
pub struct ModelInputs<'a> {
    pub def: &'a AircraftDef,
    pub ac: &'a Aircraft,
    pub ctl: &'a Controls,
    pub time: f32,
    /// Lights on: navigation, beacon, strobes, landing / taxi.
    pub nav: bool,
    pub beacon: bool,
    pub strobe: bool,
    pub landing: bool,
}

/// Value of one animation channel (FlightGear semantics, see `aircraft_asset.rs`).
pub fn channel_value(name: &str, i: &ModelInputs) -> f32 {
    let s = i.ac.surfaces();
    let ac = i.ac;
    let idx = |pre: &str, suf: &str| -> Option<usize> { name.strip_prefix(pre)?.strip_suffix(suf)?.parse().ok() };
    let b = |x: bool| if x { 1.0 } else { 0.0 };
    match name {
        "elevator" => return -s.elevator,
        "aileron_l" => return s.aileron,
        "aileron_r" => return -s.aileron,
        "rudder" => return s.rudder,
        "flaps" => return s.flaps,
        "spoilers" => return s.spoilers,
        "gear_pos" => return 1.0 - s.gear,
        "wow" => return b(ac.on_ground),
        "steer_norm" => return if ac.on_ground { s.rudder } else { 0.0 },
        "steer_deg" => return if ac.on_ground { s.rudder * i.def.nose_steer_deg } else { 0.0 },
        "afterburner" => return s.afterburner,
        "ctl_elevator" => return -i.ctl.elevator,
        "ctl_aileron" => return i.ctl.aileron,
        "ctl_rudder" => return i.ctl.rudder,
        "ctl_flaps" => return i.ctl.flaps,
        "ctl_spoilers" => return i.ctl.spoilers,
        "ctl_trim" => return -i.ctl.trim,
        "ctl_gear" => return b(i.ctl.gear_down),
        "throttle" => return i.ctl.throttle,
        "brakes" => return i.ctl.brakes,
        "nav" => return b(i.nav),
        "beacon_sw" => return b(i.beacon),
        "strobe_sw" => return b(i.strobe),
        "beacon" => return b(i.beacon && (i.time * 1.0).fract() < 0.12),
        "strobe" => {
            let st = (i.time / 1.2).fract();
            return b(i.strobe && (st < 0.05 || (0.12..0.17).contains(&st)));
        }
        "landing_light" | "taxi_light" => return b(i.landing),
        "time" => return i.time,
        "ias_kt" => return ac.airspeed_kt(),
        "alt_ft" => return ac.pos.y / 0.3048,
        "pitch_deg" => return ac.pitch().to_degrees(),
        "roll_deg" => return ac.roll().to_degrees(),
        "heading_deg" => return ac.heading().to_degrees().rem_euclid(360.0),
        "vs_fpm" => return ac.vertical_speed() / 0.00508,
        "turn_rate" => return (ac.turn_rate().to_degrees() / 3.0).clamp(-2.0, 2.0),
        "slip" => return (ac.slip() * 4.0).clamp(-1.0, 1.0),
        "mach" => return ac.mach(),
        "alpha_deg" => return ac.alpha().to_degrees(),
        _ => {}
    }
    let n_eng = ac.engine_count(i.def).max(1);
    let eng = |k: usize| k.min(n_eng - 1);
    if let Some(k) = idx("eng", "_rpm") {
        return ac.engine_rpm_i(eng(k));
    }
    if let Some(k) = idx("prop", "_rpm") {
        let r = ac.engine_rpm_i(eng(k));
        return if i.def.engine_kind == EngineKind::Piston { r * i.def.prop_gear } else { r };
    }
    if let Some(k) = idx("eng", "_n1") {
        return ac.n1(eng(k)) * 100.0;
    }
    if let Some(k) = idx("eng", "_n2") {
        return (ac.n1(eng(k)) * 100.0 * 0.9 + 10.0).min(105.0);
    }
    if let Some(k) = idx("eng", "_running") {
        return b(ac.engine_running(eng(k)));
    }
    if idx("eng", "_reverser").is_some() {
        return s.reverser;
    }
    if let Some(k) = idx("gear", "_roll_ms") {
        let _ = k;
        return if ac.on_ground { ac.vel.length() } else { 0.0 };
    }
    if let Some(k) = idx("gear", "_comp_m") {
        return ac.gear_compression(k).0;
    }
    if let Some(k) = idx("gear", "_comp_norm") {
        return ac.gear_compression(k).1;
    }
    0.0
}

impl AircraftModel {
    /// Evaluate the animation for this frame.
    pub fn update(&mut self, inp: &ModelInputs) {
        let Some(a) = &self.asset else { return };
        let dt = match self.last_time {
            Some(t) => (inp.time - t).clamp(0.0, 0.25),
            None => 0.0,
        };
        self.last_time = Some(inp.time);
        for (k, name) in a.channels.iter().enumerate() {
            self.chan[k] = channel_value(name, inp);
        }
        a.pose(&self.chan, &mut self.state, dt, &mut self.pose);
    }

    /// Draw list for the renderer. `body` = body → eye-relative world.
    /// `interior`: include cockpit / cabin geometry.
    pub fn frame(&self, body: Mat4, interior: bool) -> Option<AircraftFrame> {
        let a = self.asset.as_ref()?;
        if self.pose.world.len() != a.nodes.len() {
            return None;
        }
        let draws = a
            .chunks
            .iter()
            .enumerate()
            .filter(|(_, c)| (interior || !c.interior) && self.pose.visible[c.node])
            .map(|(i, c)| ChunkDraw { chunk: i as u32, model: body * self.pose.world[c.node], emit: self.pose.emit[i] })
            .collect();
        Some(AircraftFrame { asset: a.clone(), draws })
    }

    /// Light positions (body frame, current pose) by kind.
    pub fn lights(&self) -> Vec<(&str, Vec3)> {
        let Some(a) = &self.asset else { return Vec::new() };
        a.lights
            .iter()
            .map(|l| {
                let p = match l.node {
                    // The importer stores positions in the static pose; follow animated parents.
                    Some(_) => l.pos,
                    None => l.pos,
                };
                (l.kind.as_str(), p)
            })
            .collect()
    }
}

/// Emissive glows for the navigation / beacon / strobe lights (vertex-coloured
/// scene mesh, body frame). `on`: (nav, beacon flash, strobe flash).
pub fn light_glows(model: &AircraftModel, out: &mut MeshData, nav: bool, beacon: bool, strobe: bool, size: f32) {
    out.mat = mat::EMISSIVE;
    for (kind, p) in model.lights() {
        let (col, k, on) = match kind {
            "nav_l" => ([1.0, 0.05, 0.03], 4.0, nav),
            "nav_r" => ([0.05, 1.0, 0.15], 4.0, nav),
            "nav_tail" => ([1.0, 1.0, 1.0], 3.0, nav),
            "beacon" => ([1.0, 0.08, 0.02], 6.0, beacon),
            "strobe" | "strobe_l" | "strobe_r" | "strobe_tail" => ([1.0, 1.0, 1.0], 12.0, strobe),
            _ => continue,
        };
        if !on {
            continue;
        }
        octahedron(out, p, size, col, k);
    }
}

fn octahedron(out: &mut MeshData, c: Vec3, r: f32, col: [f32; 3], k: f32) {
    let d = [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z];
    let faces = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)];
    for (a, b, e) in faces {
        let (pa, pb, pe) = (c + d[a] * r, c + d[b] * r, c + d[e] * r);
        let n = (d[a] + d[b] + d[e]).normalize();
        let base = out.verts.len() as u32;
        for p in [pa, pb, pe] {
            out.v_uv(p, n, col, [k, 0.0]);
        }
        out.idx.extend([base, base + 1, base + 2]);
    }
}

/// Afterburner flame cone at `nozzle` (body frame), pointing aft (−Z), 1 m long
/// at scale 1; EMISSIVE flame verts (uv.y = 1), intensity in uv.x.
pub fn flame_mesh(nozzle: Vec3, radius: f32) -> MeshData {
    let mut m = MeshData { mat: mat::EMISSIVE, ..Default::default() };
    let seg = 16;
    let rings = [(0.0, 1.0, [1.0, 0.75, 0.45], 3.0), (0.25, 0.85, [1.0, 0.55, 0.25], 2.4), (0.7, 0.45, [0.9, 0.35, 0.6], 1.4), (1.0, 0.05, [0.5, 0.25, 0.9], 0.6)];
    let len = radius * 9.0;
    let mut ring_idx = Vec::new();
    for (z, r, col, k) in rings {
        let base = m.verts.len() as u32;
        for s in 0..seg {
            let a = s as f32 / seg as f32 * std::f32::consts::TAU;
            let p = nozzle + vec3(a.cos() * r * radius, a.sin() * r * radius, -z * len);
            m.v_uv(p, vec3(a.cos(), a.sin(), 0.0), col, [k, 1.0]);
        }
        ring_idx.push(base);
    }
    for w in ring_idx.windows(2) {
        for s in 0..seg as u32 {
            let (a0, a1) = (w[0] + s, w[0] + (s + 1) % seg as u32);
            let (b0, b1) = (w[1] + s, w[1] + (s + 1) % seg as u32);
            m.idx.extend([a0, b0, a1, a1, b0, b1]);
        }
    }
    m
}

/// Exterior bounding box in the static pose with the gear down.
fn exterior_bounds(a: &AircraftAsset) -> (Vec3, Vec3) {
    let ch: Vec<f32> = a.channels.iter().map(|c| if c == "gear_pos" { 1.0 } else { 0.0 }).collect();
    let mut st = AnimState::default();
    let mut pose = Pose::default();
    a.pose(&ch, &mut st, 0.0, &mut pose);
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for c in a.chunks.iter().filter(|c| !c.interior && pose.visible[c.node]) {
        let m = pose.world[c.node];
        for k in c.first..c.first + c.count {
            let p = m.transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos));
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    if lo.x > hi.x { (Vec3::splat(-4.0), Vec3::splat(4.0)) } else { (lo, hi) }
}
