//! Load-time corrections to the imported aircraft animations
//! (`assets/aircraft/<id>/model.skac`, see `aircraft_asset.rs`).
//!
//! `tools/fgimport` maps FlightGear properties onto Skyward's control
//! channels with one convention for every aircraft, but the FlightGear models
//! do not all agree with it:
//!
//! * **Sign conventions differ between FDMs.** Some JSBSim / YASim models
//!   define `right-aileron-pos-norm` with the left aileron's sign, some
//!   `rudder-pos-norm` or `elevator-pos-norm` the other way round; the
//!   model XML then rotates the surface the "wrong" way for Skyward's channel.
//!   [`normalize_senses`] measures, per aircraft and channel, which way the
//!   trailing edges of the surfaces driven by `elevator`, `aileron_l`,
//!   `aileron_r`, `flaps` and `rudder` move, and negates the channel where
//!   they go the wrong way (+1 = trailing edge down; rudder: trailing edge
//!   right) — the same correction for every aircraft, by geometry.
//! * **Some surfaces are driven by properties the importer cannot map** (the
//!   F-16's right stabilator and flaperons read `sim/multiplay/generic/float[4..6]`,
//!   aliases of FCS outputs declared in the `-set.xml`). Their animation
//!   folded to a constant and the importer merged their geometry into the
//!   static airframe, so one stabilator moved and the other did not.
//!   [`bind_static_parts`] cuts such parts back out of the static geometry
//!   (by their AC3D object boxes) and gives them the animation the
//!   FlightGear model intended; [`rescale`] fixes surfaces whose property
//!   is not normalised (radians / degrees read as −1..1).
//! * **Forward nose-gear doors left open with the gear down** (Citation X,
//!   CRJ700) hung beside the nose wheel like a stray control surface;
//!   [`CLOSE_WHEN_DOWN`] closes them once the gear is down.
//!
//! The fixes are applied by [`apply`] when a model is parsed, so the shipped
//! assets need no re-import (re-importing the F-16 from its pinned FGAddon
//! revision does not reproduce the shipped cockpit). The per-aircraft data
//! is in [`STATIC_PARTS`] and [`RESCALE`]; `cargo test aircraft_fixups`
//! checks every aircraft's surfaces.

use crate::aircraft_asset::{AircraftAsset, Anim, AnimState, Chunk, Node, Op, Pose, RExpr};
use glam::{Mat4, Vec3};

/// A part whose animation the importer dropped: the static triangles of the
/// root node inside `boxes` (FlightGear model frame: x aft, y right, z up,
/// metres) become a new node rotating about `center` / `axis` (model frame)
/// by `deg(channels)` degrees.
pub struct StaticPart {
    pub aircraft: &'static str,
    pub name: &'static str,
    pub boxes: &'static [([f32; 3], [f32; 3])],
    pub center: [f32; 3],
    pub axis: [f32; 3],
    pub deg: fn(&mut dyn FnMut(&str) -> RExpr) -> RExpr,
    /// Triangles the AC3D objects have (checked by the tests).
    pub tris: usize,
}

fn mul(a: RExpr, k: f32) -> RExpr {
    RExpr::Op(Op::Mul, vec![a, RExpr::C(k)])
}
fn add(a: RExpr, b: RExpr) -> RExpr {
    RExpr::Op(Op::Add, vec![a, b])
}
fn clamp(a: RExpr, lo: f32, hi: f32) -> RExpr {
    RExpr::Op(Op::Clamp, vec![a, RExpr::C(lo), RExpr::C(hi)])
}

/// F-16 (`lancer`, FGAddon f16 r21799 `Models/F-16.xml`). FlightGear drives
/// the right stabilator with `float[4]` = `fcs/dht-right-pos-rad` (× 57.3),
/// the flaperons with `float[5]` / `float[6]` = ∓0.05 × flaperon degrees
/// (× 20; the outboard "Aileron" objects only through a shader effect on the
/// same hinge line). Stabilators: ±25° pitch plus ±5° differential (roll);
/// flaperons: ±21.5° roll plus 20° droop with the flaps, −23 .. +20°.
/// Positive deflection = trailing edge down; about the left-hand axes the
/// model rotates by −deflection, about the right-hand ones by +deflection.
const F16_TAIL_R: &[([f32; 3], [f32; 3])] = &[([3.182, 1.083, -0.265], [5.648, 2.849, 0.102])];
const F16_FLAPERON_L: &[([f32; 3], [f32; 3])] = &[([1.327, -3.583, 0.049], [2.179, -1.065, 0.203]), ([1.72, -4.619, 0.013], [2.37, -3.57, 0.09])];
const F16_FLAPERON_R: &[([f32; 3], [f32; 3])] = &[([1.328, 1.073, 0.048], [2.181, 3.608, 0.201]), ([1.721, 3.563, 0.012], [2.37, 4.63, 0.089])];

fn f16_stab(ch: &mut dyn FnMut(&str) -> RExpr, ail: &str) -> RExpr {
    clamp(add(mul(ch("elevator"), 25.0), mul(ch(ail), 5.0)), -25.0, 25.0)
}
fn f16_flaperon(ch: &mut dyn FnMut(&str) -> RExpr, ail: &str) -> RExpr {
    clamp(add(mul(ch(ail), 21.5), mul(ch("flaps"), 20.0)), -23.0, 20.0)
}

pub static STATIC_PARTS: &[StaticPart] = &[
    StaticPart {
        aircraft: "lancer",
        name: "RightHorizonTail",
        boxes: F16_TAIL_R,
        center: [4.36, 1.965, -0.1],
        axis: [0.0, 0.981645, -0.19072],
        deg: |ch| f16_stab(ch, "aileron_r"),
        tris: 22,
    },
    StaticPart {
        aircraft: "lancer",
        name: "LeftFlaperon",
        boxes: F16_FLAPERON_L,
        center: [1.54, -2.35, 0.12],
        axis: [0.150157, -0.987872, -0.039515],
        deg: |ch| RExpr::Op(Op::Neg, vec![f16_flaperon(ch, "aileron_l")]),
        tris: 1011,
    },
    StaticPart {
        aircraft: "lancer",
        name: "RightFlaperon",
        boxes: F16_FLAPERON_R,
        center: [1.54, 2.35, 0.12],
        axis: [0.150157, 0.987872, -0.039515],
        deg: |ch| f16_flaperon(ch, "aileron_r"),
        tris: 1011,
    },
];

/// Animations whose property is not normalised: (aircraft, node name, new
/// expression). F-16: the left stabilator reads `elevator-pos-norm`, which
/// the F-16 FCS writes as −(left tail radians) (× 57.3 in the model): ±57°
/// for Skyward's ±1, and against the right stabilator's sense — it gets the
/// stabilator law above (the model rotates the left side by −deflection).
/// The leading-edge flaps read `flap-pos-norm`, which the F-16 FCS writes
/// in degrees (0 .. 25): Skyward's 0..1 flap channel drooped them ≤ 1°.
pub struct Rescale {
    pub aircraft: &'static str,
    pub node: &'static str,
    pub deg: fn(&mut dyn FnMut(&str) -> RExpr) -> RExpr,
}

pub static RESCALE: &[Rescale] = &[
    Rescale { aircraft: "lancer", node: "rotate:LeftUpperHorizonTail", deg: |ch| RExpr::Op(Op::Neg, vec![f16_stab(ch, "aileron_l")]) },
    Rescale { aircraft: "lancer", node: "rotate:LeftLowerHorizonTail", deg: |ch| RExpr::Op(Op::Neg, vec![f16_stab(ch, "aileron_l")]) },
    Rescale { aircraft: "lancer", node: "rotate:LeftUpperLef", deg: |ch| mul(ch("flaps"), 25.0) },
    Rescale { aircraft: "lancer", node: "rotate:LeftLowerLef", deg: |ch| mul(ch("flaps"), 25.0) },
    Rescale { aircraft: "lancer", node: "rotate:RightUpperLef", deg: |ch| mul(ch("flaps"), -25.0) },
    Rescale { aircraft: "lancer", node: "rotate:RightLowerLef", deg: |ch| mul(ch("flaps"), -25.0) },
];

/// Forward nose-gear doors that the FlightGear model leaves standing open
/// (90° / 85°) with the gear down: flat panels hanging beside and ahead of
/// the nose wheel that read as a stray control surface. On these types the
/// forward doors close again once the gear is down and locked (the
/// Bombardier sequence; the doors on the strut stay open): open in transit,
/// shut over the last 15 % of travel. (aircraft, node name.)
pub static CLOSE_WHEN_DOWN: &[(&str, &str)] =
    &[("citationx", "rotate:LHfrt.geardoor"), ("citationx", "rotate:RHfrt.geardoor"), ("crj700", "rotate:Nose gear left door"), ("crj700", "rotate:Nose gear right door")];

fn close_nose_doors(a: &mut AircraftAsset, log: &mut Vec<String>) {
    let Some(g) = a.channel("gear_pos") else { return };
    for (ac, name) in CLOSE_WHEN_DOWN {
        if *ac != a.id {
            continue;
        }
        for n in a.nodes.iter_mut().filter(|n| n.name == *name) {
            if let Some(Anim::Rotate { deg, .. }) = &mut n.anim {
                let shut = RExpr::Interp(Box::new(RExpr::Ch(g as u16)), vec![(0.0, 1.0), (0.85, 1.0), (1.0, 0.0)]);
                *deg = RExpr::Op(Op::Mul, vec![deg.clone(), shut]);
                log.push(format!("{name}: closes with the gear down"));
            }
        }
    }
}

/// Apply every correction; returns what was changed (for logs and tests).
pub fn apply(a: &mut AircraftAsset) -> Vec<String> {
    let mut log = Vec::new();
    rescale(a, &mut log);
    bind_static_parts(a, &mut log);
    close_nose_doors(a, &mut log);
    normalize_senses(a, &mut log);
    if !log.is_empty() && std::env::var_os("SKYWARD_FIXUP_LOG").is_some() {
        for l in &log {
            eprintln!("aircraft fixup {}: {l}", a.id);
        }
    }
    log
}

/// Channel lookup that adds the channel to the model if it is new.
fn channel_expr(a: &mut AircraftAsset, name: &str) -> RExpr {
    let i = match a.channel(name) {
        Some(i) => i,
        None => {
            a.channels.push(name.to_string());
            a.channels.len() - 1
        }
    };
    RExpr::Ch(i as u16)
}

fn rescale(a: &mut AircraftAsset, log: &mut Vec<String>) {
    let id = a.id.clone();
    for r in RESCALE.iter().filter(|r| r.aircraft == id) {
        let Some(k) = a.nodes.iter().position(|n| n.name == r.node) else { continue };
        let e = (r.deg)(&mut |c| channel_expr(a, c));
        if let Some(Anim::Rotate { deg, .. }) = &mut a.nodes[k].anim {
            *deg = e;
            log.push(format!("{}: rescaled", r.node));
        }
    }
}

fn bind_static_parts(a: &mut AircraftAsset, log: &mut Vec<String>) {
    let Some(root) = a.nodes.iter().position(|n| n.parent < 0) else { return };
    let id = a.id.clone();
    for p in STATIC_PARTS.iter().filter(|p| p.aircraft == id) {
        let inside = |v: Vec3| {
            p.boxes.iter().any(|(lo, hi)| {
                let (lo, hi) = (Vec3::from(*lo) - 0.003, Vec3::from(*hi) + 0.003);
                v.cmpge(lo).all() && v.cmple(hi).all()
            })
        };
        let deg = (p.deg)(&mut |c| channel_expr(a, c));
        let node = a.nodes.len();
        let mut moved = 0;
        let mut new_chunks: Vec<Chunk> = Vec::new();
        for c in a.chunks.iter_mut().filter(|c| c.node == root && !c.interior) {
            let (first, n) = (c.first as usize, c.count as usize);
            let tris: Vec<[u32; 3]> = a.indices[first..first + n].chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect();
            let hit = |t: &[u32; 3]| t.iter().all(|&i| inside(Vec3::from(a.vertices[(i + c.base_vertex) as usize].pos)));
            let (sel, keep): (Vec<[u32; 3]>, Vec<[u32; 3]>) = tris.into_iter().partition(hit);
            if sel.is_empty() {
                continue;
            }
            for (k, i) in keep.iter().chain(&sel).flatten().enumerate() {
                a.indices[first + k] = *i;
            }
            c.count = (keep.len() * 3) as u32;
            let mut part = c.clone();
            part.node = node;
            part.first = (first + keep.len() * 3) as u32;
            part.count = (sel.len() * 3) as u32;
            moved += sel.len();
            new_chunks.push(part);
        }
        if moved == 0 {
            continue;
        }
        a.chunks.extend(new_chunks);
        a.chunks.retain(|c| c.count > 0);
        a.nodes.push(Node {
            name: format!("rotate:{}", p.name),
            parent: root as i32,
            pre: Mat4::IDENTITY,
            anim: Some(Anim::Rotate { center: Vec3::from(p.center), axis: Vec3::from(p.axis).normalize(), deg }),
        });
        log.push(format!("{}: {moved} static triangles animated", p.name));
    }
}

/// The surface channels and the motion +1 should give.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    /// Trailing edge down (elevator: stick forward; flaps: extended).
    Down,
    /// Roll: trailing edges down on the left (`true`: roll right, the
    /// `aileron_l` channel) or on the right (`false`: roll left, `aileron_r`),
    /// up on the other side.
    Roll(bool),
    /// Trailing edge to the right (rudder: yaw right).
    Right,
}

pub const SURFACE_CHANNELS: [(&str, Sense); 5] =
    [("elevator", Sense::Down), ("aileron_l", Sense::Roll(true)), ("aileron_r", Sense::Roll(false)), ("flaps", Sense::Down), ("rudder", Sense::Right)];

/// Rest pose for the measurements: everything neutral, gear down.
pub fn rest_channels(a: &AircraftAsset) -> Vec<f32> {
    a.channels.iter().map(|c| if c == "gear_pos" { 1.0 } else { 0.0 }).collect()
}

fn reads(e: &RExpr, ch: u16) -> bool {
    let mut ids = Vec::new();
    e.channel_ids(&mut ids);
    ids.contains(&ch)
}

/// Does the node's own animation read channel `ch`?
pub fn node_reads(n: &Node, ch: u16) -> bool {
    match &n.anim {
        Some(Anim::Rotate { deg: e, .. } | Anim::Translate { m: e, .. }) => reads(e, ch),
        _ => false,
    }
}

/// How a surface node moves when channel `ch` goes 0 → +1: the mean motion
/// of its subtree's trailing edge (the aft fifth of its exterior geometry),
/// along the expected [`Sense`] (positive = the right way), and the vertex
/// count. `None` unless the node rotates exterior geometry hinged ahead of it.
pub fn trailing_edge_motion(a: &AircraftAsset, node: usize, ch: usize, sense: Sense) -> Option<(f32, usize)> {
    let mut in_sub = vec![false; a.nodes.len()];
    for (i, n) in a.nodes.iter().enumerate() {
        in_sub[i] = i == node || (n.parent >= 0 && in_sub[n.parent as usize]);
    }
    let rest = rest_channels(a);
    let mut moved = rest.clone();
    moved[ch] = 1.0;
    let (mut p0, mut p1) = (Pose::default(), Pose::default());
    a.pose(&rest, &mut AnimState::default(), 0.0, &mut p0);
    a.pose(&moved, &mut AnimState::default(), 0.0, &mut p1);
    let mut pts: Vec<(Vec3, Vec3)> = Vec::new();
    for c in a.chunks.iter().filter(|c| !c.interior && in_sub[c.node] && p0.visible[c.node]) {
        for k in c.first..c.first + c.count {
            let v = Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos);
            pts.push((p0.world[c.node].transform_point3(v), p1.world[c.node].transform_point3(v)));
        }
    }
    if pts.is_empty() {
        return None;
    }
    // Trailing-edge surfaces only: hinged ahead of their geometry (not
    // slats, leading-edge flaps, nose visors, gear legs, rods).
    let Some(Anim::Rotate { center, axis, .. }) = &a.nodes[node].anim else { return None };
    let hinge = p0.world[node].transform_point3(*center);
    let axis = p0.world[node].transform_vector3(*axis).normalize_or_zero();
    let centroid = pts.iter().map(|(p, _)| *p).sum::<Vec3>() / pts.len() as f32;
    // (Off the hinge line, not off its centre point: swept hinges.)
    let arm = centroid - hinge;
    if (arm - axis * arm.dot(axis)).z > -0.015 {
        return None;
    }
    let (zmin, zmax) = pts.iter().fold((f32::MAX, f32::MIN), |(l, h), (p, _)| (l.min(p.z), h.max(p.z)));
    let cut = zmin + 0.2 * (zmax - zmin);
    let te: Vec<Vec3> = pts.iter().filter(|(p, _)| p.z <= cut).map(|(p, q)| *q - *p).collect();
    let d = te.iter().copied().sum::<Vec3>() / te.len() as f32;
    // Body frame: +Y up, +X left.
    let along = match sense {
        Sense::Down => -d.y,
        Sense::Right => -d.x,
        // Ailerons, elevons, flaperons, roll spoilers: by side (body +X = left).
        Sense::Roll(_) if centroid.x.abs() < 0.3 => return None,
        Sense::Roll(left_down) => {
            if (centroid.x > 0.0) == left_down {
                -d.y
            } else {
                d.y
            }
        }
    };
    Some((along, pts.len()))
}

/// Per surface channel: the exterior nodes driven by it and how their
/// trailing edges move (see [`trailing_edge_motion`]).
pub fn surface_motions(a: &AircraftAsset) -> Vec<(&'static str, usize, f32, usize)> {
    let mut out = Vec::new();
    for (name, sense) in SURFACE_CHANNELS {
        let Some(ch) = a.channel(name) else { continue };
        for (i, n) in a.nodes.iter().enumerate() {
            if node_reads(n, ch as u16) {
                if let Some((m, nv)) = trailing_edge_motion(a, i, ch, sense) {
                    out.push((name, i, m, nv));
                }
            }
        }
    }
    out
}

/// Negate a channel wherever the model reads it.
fn negate_channel(a: &mut AircraftAsset, ch: u16) {
    fn neg(e: &mut RExpr, ch: u16) {
        match e {
            RExpr::Ch(i) if *i == ch => *e = RExpr::Op(Op::Neg, vec![RExpr::Ch(ch)]),
            RExpr::Op(_, args) => args.iter_mut().for_each(|x| neg(x, ch)),
            RExpr::Interp(x, _) => neg(x, ch),
            _ => {}
        }
    }
    for n in &mut a.nodes {
        match &mut n.anim {
            Some(Anim::Rotate { deg: e, .. } | Anim::Translate { m: e, .. } | Anim::Spin { rpm: e, .. } | Anim::Select(e)) => neg(e, ch),
            Some(Anim::Scale { s, .. }) => s.iter_mut().for_each(|e| neg(e, ch)),
            None => {}
        }
    }
    for c in &mut a.chunks {
        if let Some((_, e)) = &mut c.emit {
            neg(e, ch);
        }
        for u in &mut c.uv {
            match u {
                crate::aircraft_asset::UvAnim::Translate { v, .. } | crate::aircraft_asset::UvAnim::Rotate { v, .. } => neg(v, ch),
            }
        }
    }
}

/// Negate the surface channels whose surfaces move the wrong way: a vote of
/// the exterior nodes driven by the channel, weighted by vertex count and
/// motion; a channel flips only on a clear (3:1) majority, so a model with
/// one odd part (a cable, a tab, a mis-hinged panel) keeps its convention.
fn normalize_senses(a: &mut AircraftAsset, log: &mut Vec<String>) {
    let motions = surface_motions(a);
    for (name, _) in SURFACE_CHANNELS {
        let (mut good, mut bad) = (0.0f32, 0.0f32);
        for &(_, _, m, nv) in motions.iter().filter(|m| m.0 == name) {
            let w = m.abs() * (nv as f32).sqrt();
            if m > 0.0 {
                good += w;
            } else {
                bad += w;
            }
        }
        if bad > 3.0 * good && bad > 0.0 {
            let ch = a.channel(name).unwrap() as u16;
            negate_channel(a, ch);
            log.push(format!("{name}: surfaces moved the wrong way, channel negated"));
        }
    }
}

#[cfg(test)]
mod tests;
