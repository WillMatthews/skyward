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
use glam::Vec2;
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
    /// Per channel: the cockpit-instrument source, if it is one.
    chan_src: Vec<Option<crate::cockpit::channels::Chan>>,
    /// Live instrument state (lags, fuel, clocks).
    pub instruments: crate::cockpit::channels::Instruments,
    last_time: Option<f32>,
    /// Structural parts of every vertex (wing flex, debris), see [`classify`].
    pub parts: Option<Arc<PartMap>>,
}

/// The model cut into structural parts ([`crate::damage::Part`]).
pub struct PartMap {
    /// Per asset vertex: part index, signed span station (+ = left wing,
    /// 0..1 root → tip; 0 = rigid), chord offset from the wing's elastic
    /// axis (m, + = forward), and 1 for exterior / 0 for interior geometry.
    pub vert: Vec<[f32; 4]>,
    /// Body-frame boxes of each part (gear down).
    pub geom: crate::debris::PartGeom,
    /// The asset's vertices / indices with seams split between parts and
    /// the wings tessellated spanwise (new vertices are appended).
    pub vertices: Vec<crate::aircraft_asset::Vertex>,
    pub indices: Vec<u32>,
    /// Per asset chunk: its (first index, index count) in `indices`.
    pub ranges: Vec<(u32, u32)>,
    /// Wing planform by span station: (|x|, leading-edge z, trailing-edge z, mean y).
    pub planform: Vec<(f32, f32, f32, f32)>,
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
            let chan_src = a.channels.iter().map(|c| crate::cockpit::channels::resolve(c)).collect();
            AircraftModel {
                asset: Some(a),
                eye,
                radius,
                bounds,
                nose_gear: None,
                state: AnimState::default(),
                pose: Pose::default(),
                chan: vec![0.0; n],
                chan_src,
                instruments: Default::default(),
                last_time: None,
                parts: None,
            }
        }
        Err(e) => {
            eprintln!("aircraft model {id}: {e}");
            AircraftModel {
                asset: None,
                eye: def_eye,
                radius: 8.0,
                bounds: (Vec3::splat(-4.0), Vec3::splat(4.0)),
                nose_gear: None,
                state: AnimState::default(),
                pose: Pose::default(),
                chan: Vec::new(),
                chan_src: Vec::new(),
                instruments: Default::default(),
                last_time: None,
                parts: None,
            }
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
    /// Autopilot targets (bugs, flight director), if the aircraft has one.
    pub ap: Option<&'a crate::autopilot::Autopilot>,
    /// Altimeter setting (the airfield QNH), hPa.
    pub qnh_hpa: f32,
    /// Local clock time, hours.
    pub hours: f32,
    /// Ambient daylight 0 (night) .. 1 (day): panel lighting / display brightness.
    pub daylight: f32,
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
        self.instruments.step(inp, dt);
        for (k, name) in a.channels.iter().enumerate() {
            self.chan[k] = match self.chan_src.get(k).copied().flatten() {
                Some(c) => self.instruments.value(c, inp),
                None => channel_value(name, inp),
            };
        }
        if std::env::var_os("SKYWARD_CHAN_DUMP").is_some() {
            let s: Vec<String> = a.channels.iter().zip(&self.chan).map(|(n, v)| format!("{n}={v:.2}")).collect();
            eprintln!("chan t={:.2}: {}", inp.time, s.join(" "));
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
            .map(|(i, c)| ChunkDraw { chunk: i as u32, model: body * self.pose.world[c.node], emit: self.pose.emit[i], uv: self.pose.uv.get(i).copied().unwrap_or(crate::aircraft_asset::UV_IDENTITY) })
            .collect();
        Some(AircraftFrame { asset: a.clone(), draws, parts: self.parts.clone(), dynamics: None, displays: None })
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

/// Cut the model into structural parts: gear by node name, then by position
/// (nacelles around the engines, the empennage aft of the tail station, wings
/// outboard of the fuselage, fuselage sections fore / aft), and give each
/// wing vertex its span station and chord offset for the bending shader.
/// Cached per aircraft id.
pub fn classify(a: &AircraftAsset, def: &AircraftDef) -> Arc<PartMap> {
    static CACHE: std::sync::Mutex<Vec<(String, Arc<PartMap>)>> = std::sync::Mutex::new(Vec::new());
    if let Some(m) = CACHE.lock().unwrap().iter().find(|(k, _)| k == def.id).map(|(_, m)| m.clone()) {
        return m;
    }
    let m = Arc::new(classify_uncached(a, def));
    CACHE.lock().unwrap().push((def.id.to_string(), m.clone()));
    m
}

fn classify_uncached(a: &AircraftAsset, def: &AircraftDef) -> PartMap {
    use crate::damage::{gear_part, struct_def, Part};
    let sd = struct_def(def.id);
    let ch: Vec<f32> = a.channels.iter().map(|c| if c == "gear_pos" { 1.0 } else { 0.0 }).collect();
    let mut st = AnimState::default();
    let mut pose = Pose::default();
    a.pose(&ch, &mut st, 0.0, &mut pose);
    let semi = (def.wingspan * 0.5).max(sd.fus_r + 0.5);
    // Gear / engine nodes by name (the node or any ancestor).
    let chain_has = |n: usize, keys: &[&str], not: &[&str]| -> bool {
        let mut i = n as i32;
        while i >= 0 {
            let name = a.nodes[i as usize].name.to_ascii_lowercase();
            if not.iter().any(|k| name.contains(k)) {
                return false;
            }
            if keys.iter().any(|k| name.contains(k)) {
                return true;
            }
            i = a.nodes[i as usize].parent;
        }
        false
    };
    let gear_keys = ["wheel", "tire", "tyre", "strut", "gear", "mlg", "nlg", "axle", "link", "sidestay", "compression"];
    let gear_not = ["door", "placard", "panel", "rod"];
    let eng_keys = ["prop", "spinner", "fan", "reverser", "nacelle", "engine", "cowl"];
    let has_gear = |p: Part| def.gear.iter().any(|g| gear_part(def, g) == Some(p));
    let n_eng = def.engine_count.min(def.engine_pos.len()).min(4);
    let wing_engine = |i: usize| def.engine_pos[i].x.abs() > sd.fus_r;
    // Node centres (gear legs are assigned as a whole).
    let mut node_c: Vec<(Vec3, f32)> = vec![(Vec3::ZERO, 0.0); a.nodes.len()];
    for c in &a.chunks {
        let m = pose.world[c.node];
        for k in (c.first..c.first + c.count).step_by(3) {
            let v = a.vertices[(a.indices[k as usize] + c.base_vertex) as usize];
            node_c[c.node].0 += m.transform_point3(Vec3::from(v.pos));
            node_c[c.node].1 += 1.0;
        }
    }
    let classify_pt = |p: Vec3, node: usize, interior: bool| -> Part {
        let fus = |z: f32| if z > sd.nose_z { Part::Nose } else if z < sd.aft_z { Part::Aft } else { Part::Fuselage };
        if interior {
            return fus(p.z);
        }
        if chain_has(node, &gear_keys, &gear_not) {
            let c = if node_c[node].1 > 0.0 { node_c[node].0 / node_c[node].1 } else { p };
            let g = if c.x > 0.25 { Part::GearL } else if c.x < -0.25 { Part::GearR } else if c.z > 0.0 { Part::GearNose } else { Part::Aft };
            return if g.is_gear() && !has_gear(g) { fus(p.z) } else { g };
        }
        // Engines: nacelles (and pylons) around wing engines; the nose of single-engine types.
        if n_eng > 0 {
            let named = chain_has(node, &eng_keys, &["door"]);
            for i in 0..n_eng {
                let e = def.engine_pos[i];
                if wing_engine(i) {
                    let (r, f, b) = sd.nacelle;
                    let d = Vec2::new(p.x - e.x, p.y - e.y);
                    let in_z = p.z < e.z + f && p.z > e.z - b;
                    let pylon = d.x.abs() < r * 0.35 && p.y > e.y && p.y < def.wing_height + 0.2 && in_z;
                    if in_z && (d.length() < r * 1.08 || pylon || (named && d.length() < r * 1.6)) {
                        return Part::engine(i);
                    }
                } else if let Some(fw) = sd.firewall_z {
                    if p.z > fw && p.x.abs() < sd.fus_r * 1.6 + if named { 2.0 } else { 0.0 } {
                        return Part::engine(i);
                    }
                }
            }
        }
        if p.z < sd.tail_z {
            return if p.x.abs() > sd.fin_r {
                if p.x > 0.0 { Part::HTailL } else { Part::HTailR }
            } else if p.y > sd.fin_y {
                Part::Fin
            } else {
                Part::Aft
            };
        }
        if p.x.abs() > sd.fus_r && p.z <= sd.wing_z.1 {
            return if p.x > 0.0 { Part::WingL } else { Part::WingR };
        }
        fus(p.z)
    };
    // Triangles are classified by their centroid, so every triangle belongs
    // to one part; vertices shared across a part boundary are duplicated
    // (the seam is where the airframe tears).
    let n0 = a.vertices.len();
    let mut out_v = a.vertices.clone();
    let mut out_i = a.indices.clone();
    let mut vpart: Vec<i16> = vec![-1; n0];
    let mut vext: Vec<bool> = vec![true; n0];
    let mut pos = vec![Vec3::ZERO; n0];
    let mut dup: std::collections::HashMap<(usize, u8), u32> = Default::default();
    let mut geom = crate::debris::PartGeom::empty();
    // Wing chord bins (leading / trailing edge per span station) for the elastic axis.
    const NB: usize = 16;
    let mut le = [f32::MIN; NB];
    let mut te = [f32::MAX; NB];
    let mut ysum = [0.0f64; NB];
    let mut yn = [0.0f64; NB];
    let bin = |x: f32| (((x.abs() - sd.fus_r) / (semi - sd.fus_r)).clamp(0.0, 0.999) * NB as f32) as usize;
    for c in &a.chunks {
        let m = pose.world[c.node];
        let mut k = c.first;
        while k + 3 <= c.first + c.count {
            let vis = [0, 1, 2].map(|j| (a.indices[(k + j) as usize] + c.base_vertex) as usize);
            let ps = vis.map(|vi| m.transform_point3(Vec3::from(a.vertices[vi].pos)));
            let cen = (ps[0] + ps[1] + ps[2]) / 3.0;
            let part = classify_pt(cen, c.node, c.interior);
            for j in 0..3 {
                let vi = vis[j];
                let p = ps[j];
                if !c.interior {
                    geom.add(part, p);
                }
                if matches!(part, Part::WingL | Part::WingR) {
                    let b = bin(p.x);
                    le[b] = le[b].max(p.z);
                    te[b] = te[b].min(p.z);
                    ysum[b] += p.y as f64;
                    yn[b] += 1.0;
                }
                if vpart[vi] < 0 {
                    vpart[vi] = part as i16;
                    pos[vi] = p;
                    vext[vi] = !c.interior;
                } else if vpart[vi] != part as i16 {
                    let ni = *dup.entry((vi, part as u8)).or_insert_with(|| {
                        out_v.push(a.vertices[vi]);
                        pos.push(p);
                        vpart.push(part as i16);
                        vext.push(!c.interior);
                        (out_v.len() - 1) as u32
                    });
                    out_i[k as usize + j] = ni - c.base_vertex;
                }
            }
            k += 3;
        }
    }
    // Coarse wings bend piecewise (a kink at every spanwise vertex): cut
    // their triangles to short spanwise segments.
    let ranges = tessellate_wings(a, &mut out_v, &mut out_i, &mut pos, &mut vpart, &mut vext, WING_SEG);
    let mut vert = vec![[0.0f32; 4]; out_v.len()];
    for vi in 0..out_v.len() {
        vert[vi] = [vpart[vi].max(0) as f32, 0.0, 0.0, if vext[vi] { 1.0 } else { 0.0 }];
    }
    let done = vec![true; out_v.len()];
    // Elastic axis ~40 % chord; empty bins borrow their neighbours.
    let mut ea = [0.0f32; NB];
    let mut last = 0.5 * (sd.wing_z.0 + sd.wing_z.1);
    for b in 0..NB {
        if le[b] > te[b] {
            last = le[b] - 0.4 * (le[b] - te[b]);
        }
        ea[b] = last;
    }
    let station = |x: f32| ((x.abs() - sd.fus_r) / (semi - sd.fus_r)).clamp(0.0, 1.0) * x.signum();
    for vi in 0..vert.len() {
        if !done[vi] {
            continue;
        }
        let part = Part::from_idx(vert[vi][0] as usize);
        let p = pos[vi];
        match part {
            Part::WingL | Part::WingR => {
                vert[vi][1] = station(p.x);
                vert[vi][2] = p.z - ea[bin(p.x)];
            }
            Part::Eng0 | Part::Eng1 | Part::Eng2 | Part::Eng3 => {
                // Wing engines ride rigidly on the wing at their station.
                let e = def.engine_pos[part.engine_index().unwrap_or(0).min(def.engine_pos.len().saturating_sub(1))];
                if e.x.abs() > sd.fus_r {
                    vert[vi][1] = station(e.x);
                    vert[vi][2] = e.z - ea[bin(e.x)];
                }
            }
            _ => {}
        }
    }
    let planform = (0..NB)
        .filter(|&b| le[b] > te[b] && yn[b] > 0.0)
        .map(|b| (sd.fus_r + (b as f32 + 0.5) / NB as f32 * (semi - sd.fus_r), le[b], te[b], (ysum[b] / yn[b]) as f32))
        .collect();
    PartMap { vert, geom, vertices: out_v, indices: out_i, ranges, planform }
}

/// Longest spanwise extent (m) of a wing triangle edge after tessellation.
const WING_SEG: f32 = 0.5;

/// Subdivide the wing triangles (parts `WingL` / `WingR`) until no edge
/// spans more than `seg` metres sideways (body x), so the bending shader's
/// smooth curve shows in the silhouette. Edges are split at their midpoints
/// (position, normal and UV interpolated), keyed by their vertex pair, so
/// neighbouring triangles split alike (no cracks); a triangle with one, two
/// or three split edges becomes two, three or four, keeping its winding.
/// Rebuilds `idx` chunk by chunk and returns each chunk's (first, count).
fn tessellate_wings(a: &AircraftAsset, verts: &mut Vec<crate::aircraft_asset::Vertex>, idx: &mut Vec<u32>, pos: &mut Vec<Vec3>, vpart: &mut Vec<i16>, vext: &mut Vec<bool>, seg: f32) -> Vec<(u32, u32)> {
    use crate::damage::Part;
    let wing = |p: i16| p == Part::WingL as i16 || p == Part::WingR as i16;
    let mut mid: std::collections::HashMap<(u32, u32), u32> = Default::default();
    let mut out = Vec::with_capacity(idx.len());
    let mut ranges = Vec::with_capacity(a.chunks.len());
    for c in &a.chunks {
        let first = out.len() as u32;
        let base = c.base_vertex;
        let mut tris: Vec<[u32; 3]> = (c.first..c.first + c.count).step_by(3).filter(|&k| k + 3 <= c.first + c.count).map(|k| [0, 1, 2].map(|j| idx[(k + j) as usize] + base)).collect();
        if !c.interior {
            for _ in 0..8 {
                let mut next = Vec::with_capacity(tris.len());
                let mut changed = false;
                for t in tris {
                    if !t.iter().all(|&v| wing(vpart[v as usize])) {
                        next.push(t);
                        continue;
                    }
                    let mut m = [None; 3];
                    for e in 0..3 {
                        let (p, q) = (t[e], t[(e + 1) % 3]);
                        if (pos[p as usize].x - pos[q as usize].x).abs() > seg {
                            let key = (p.min(q), p.max(q));
                            m[e] = Some(*mid.entry(key).or_insert_with(|| {
                                let (va, vb) = (verts[p as usize], verts[q as usize]);
                                let l = |x: [f32; 3], y: [f32; 3]| [0.5 * (x[0] + y[0]), 0.5 * (x[1] + y[1]), 0.5 * (x[2] + y[2])];
                                let n = Vec3::from(l(va.nrm, vb.nrm)).normalize_or(Vec3::from(va.nrm));
                                verts.push(crate::aircraft_asset::Vertex { pos: l(va.pos, vb.pos), nrm: n.to_array(), uv: [0.5 * (va.uv[0] + vb.uv[0]), 0.5 * (va.uv[1] + vb.uv[1])] });
                                pos.push(0.5 * (pos[p as usize] + pos[q as usize]));
                                vpart.push(vpart[p as usize]);
                                vext.push(vext[p as usize]);
                                (verts.len() - 1) as u32
                            }));
                        }
                    }
                    let n_split = m.iter().filter(|x| x.is_some()).count();
                    if n_split == 0 {
                        next.push(t);
                        continue;
                    }
                    changed = true;
                    // Rotate so the first split edge is edge 0 (a → b).
                    let r = (0..3).find(|&e| m[e].is_some() && (n_split == 3 || m[(e + 2) % 3].is_none())).unwrap_or(0);
                    let [a0, b0, c0] = [t[r], t[(r + 1) % 3], t[(r + 2) % 3]];
                    let [m0, m1, m2] = [m[r], m[(r + 1) % 3], m[(r + 2) % 3]];
                    match (m0, m1, m2) {
                        (Some(x), None, None) => next.extend([[a0, x, c0], [x, b0, c0]]),
                        (Some(x), Some(y), None) => next.extend([[x, b0, y], [a0, x, y], [a0, y, c0]]),
                        (Some(x), Some(y), Some(z)) => next.extend([[a0, x, z], [x, b0, y], [z, y, c0], [x, y, z]]),
                        _ => next.push(t),
                    }
                }
                tris = next;
                if !changed {
                    break;
                }
            }
        }
        for t in tris {
            out.extend(t.iter().map(|&v| v - base));
        }
        ranges.push((first, out.len() as u32 - first));
    }
    *idx = out;
    ranges
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wings_are_tessellated_spanwise() {
        use crate::damage::Part;
        for id in ["kestrel", "swift", "meridian320"] {
            let def = crate::flight::find(id).unwrap();
            let Ok(a) = AircraftAsset::load(id) else { continue };
            let pm = classify_uncached(&a, &def);
            let sd = crate::damage::struct_def(id);
            let span = (def.wingspan * 0.5).max(sd.fus_r + 0.5) - sd.fus_r;
            assert_eq!(pm.ranges.len(), a.chunks.len());
            assert_eq!(pm.vert.len(), pm.vertices.len());
            let mut n = 0;
            for (c, &(first, count)) in a.chunks.iter().zip(&pm.ranges) {
                assert_eq!(count % 3, 0);
                assert!((first + count) as usize <= pm.indices.len());
                for t in pm.indices[first as usize..(first + count) as usize].chunks(3) {
                    let v = [t[0], t[1], t[2]].map(|i| (i + c.base_vertex) as usize);
                    assert!(v.iter().all(|&i| i < pm.vertices.len()));
                    let wing = v.iter().all(|&i| matches!(Part::from_idx(pm.vert[i][0] as usize), Part::WingL | Part::WingR));
                    if wing && !c.interior {
                        n += 1;
                        // Spanwise extent of every edge ≤ the segment (stations are clamped at the root).
                        for e in 0..3 {
                            let ds = (pm.vert[v[e]][1] - pm.vert[v[(e + 1) % 3]][1]).abs() * span;
                            assert!(ds <= WING_SEG + 0.01, "{id}: wing edge spans {ds} m");
                        }
                    }
                }
            }
            assert!(n > 100, "{id}: {n} wing triangles");
        }
    }
}
