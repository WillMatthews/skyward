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
//!   `aileron_r`, `flaps`, `rudder` and `spoilers` move, and negates the
//!   channel where they go the wrong way (+1 = trailing edge down; rudder:
//!   trailing edge right; spoilers: up) — the same correction for every
//!   aircraft, by geometry. [`normalize_steering`] does the same for the
//!   nose wheels (right rudder: nose wheel right).
//! * **Some surfaces are driven by properties the importer cannot map** (the
//!   F-16's right stabilator and flaperons read `sim/multiplay/generic/float[4..6]`,
//!   aliases of FCS outputs declared in the `-set.xml`; the A320's every
//!   surface; the E190's elevators; the Extra 500's rudder and propeller).
//!   Their animation folded to a constant and the importer merged their
//!   geometry into the static airframe, so one stabilator moved and the
//!   other did not. [`bind_static_parts`] cuts such parts back out of the
//!   static geometry (by their AC3D object boxes) and gives them the
//!   animation the FlightGear model intended; [`rescale`] fixes surfaces
//!   whose property is not normalised (radians / degrees read as −1..1) or
//!   whose travel or sense is off.
//! * **Some surfaces follow the cockpit controls** (`controls/flight/*`:
//!   Citation X, CRJ700 and Vulcan): [`ctl_to_surface`] moves them onto the
//!   surface channels.
//! * **Some hinges are wrong** ([`REHINGE`], [`REANIMATE`]: the C-130's
//!   elevator, the ATR's tailplane and elevators).
//! * **Forward nose-gear doors left open with the gear down** (Citation X,
//!   CRJ700) hung beside the nose wheel like a stray control surface;
//!   [`CLOSE_WHEN_DOWN`] closes them once the gear is down.
//!
//! The fixes are applied by [`apply`] when a model is parsed, so the shipped
//! assets need no re-import (re-importing the F-16 from its pinned FGAddon
//! revision does not reproduce the shipped cockpit). The per-aircraft data
//! is in [`STATIC_PARTS`], [`RESCALE`], [`REHINGE`] and [`REANIMATE`];
//! `cargo test aircraft_fixups` checks every aircraft's surfaces, steering
//! and propellers. Not fixable here (the geometry is not in the shipped
//! model): the Zlin's rudder and the ATR's propellers; the A320's nose-wheel
//! steering would need a node inserted into its gear chain.

use crate::aircraft_asset::{AircraftAsset, Anim, AnimState, Chunk, Node, Op, Pose, RExpr};
use glam::{Mat4, Vec3};

/// A part whose animation the importer dropped: the static triangles of the
/// root node (or of `from`) inside `boxes` (FlightGear model frame: x aft, y
/// right, z up, metres) become a new node rotating about `center` / `axis`
/// (model frame) by `deg(channels)` degrees (or spinning at that rpm).
pub struct StaticPart {
    pub aircraft: &'static str,
    pub name: &'static str,
    pub boxes: &'static [([f32; 3], [f32; 3])],
    /// The node whose static triangles are cut ("" = the root, the model
    /// frame); the part becomes its child, `boxes` and hinge in its frame.
    pub from: &'static str,
    /// Frame of `boxes` and the hinge: "" = the model frame, or a node whose
    /// static transform (`pre`) places the part's submodel (the Extra 500's
    /// airframe is pitched 1.39° in its model).
    pub frame: &'static str,
    pub center: [f32; 3],
    pub axis: [f32; 3],
    /// Triangles reaching further than this ahead of the hinge line (m) stay
    /// static: the box of a control surface also holds the trailing edge of
    /// the fixed surface ahead of it. `f32::INFINITY`: the box alone.
    pub fwd: f32,
    /// Only triangles of these materials (empty: any; the A320's spoiler
    /// boxes also hold the flap mechanism below them).
    pub mats: &'static [usize],
    /// A propeller: spins about the axis at `deg(channels)` rpm.
    pub spin: bool,
    pub deg: fn(&mut dyn FnMut(&str) -> RExpr) -> RExpr,
    /// Triangles the AC3D objects have (or, for the ATR's elevators, the
    /// cut region; checked by the tests).
    pub tris: usize,
}

/// A trailing-edge surface cut out of the model frame: hinge `center` /
/// `axis` oriented so that positive degrees put the trailing edge down
/// (elevators, ailerons, spoilers: axis to the right) or to the right
/// (rudders: axis up); triangles at most `fwd` ahead of the hinge line.
#[allow(clippy::too_many_arguments)]
const fn surface(
    aircraft: &'static str,
    name: &'static str,
    boxes: &'static [([f32; 3], [f32; 3])],
    center: [f32; 3],
    axis: [f32; 3],
    fwd: f32,
    deg: fn(&mut dyn FnMut(&str) -> RExpr) -> RExpr,
    tris: usize,
) -> StaticPart {
    StaticPart { aircraft, name, boxes, from: "", frame: "", center, axis, fwd, mats: &[], spin: false, deg, tris }
}

/// An A320 spoiler (material 0, see [`StaticPart::mat`]).
const fn a320_spoiler(name: &'static str, boxes: &'static [([f32; 3], [f32; 3])], center: [f32; 3], axis: [f32; 3], tris: usize) -> StaticPart {
    StaticPart { aircraft: "meridian320", name, boxes, from: "", frame: "", center, axis, fwd: 0.1, mats: &[0], spin: false, deg: |ch| mul(ch("spoilers"), -40.0), tris }
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
        from: "",
        frame: "",
        fwd: f32::INFINITY,
        mats: &[],
        spin: false,
        center: [4.36, 1.965, -0.1],
        axis: [0.0, 0.981645, -0.19072],
        deg: |ch| f16_stab(ch, "aileron_r"),
        tris: 22,
    },
    StaticPart {
        aircraft: "lancer",
        name: "LeftFlaperon",
        boxes: F16_FLAPERON_L,
        from: "",
        frame: "",
        fwd: f32::INFINITY,
        mats: &[],
        spin: false,
        center: [1.54, -2.35, 0.12],
        axis: [0.150157, -0.987872, -0.039515],
        deg: |ch| RExpr::Op(Op::Neg, vec![f16_flaperon(ch, "aileron_l")]),
        tris: 1011,
    },
    StaticPart {
        aircraft: "lancer",
        name: "RightFlaperon",
        boxes: F16_FLAPERON_R,
        from: "",
        frame: "",
        fwd: f32::INFINITY,
        mats: &[],
        spin: false,
        center: [1.54, 2.35, 0.12],
        axis: [0.150157, 0.987872, -0.039515],
        deg: |ch| f16_flaperon(ch, "aileron_r"),
        tris: 1011,
    },
    // E190 (`meridian190`): the elevators read surface-positions/elevator-lh-
    // / -rh-pos-norm (±15°), which the importer does not map.
    surface(
        "meridian190",
        "Left elevator",
        &[([16.296, -5.239, 1.36], [18.356, -0.847, 1.976])],
        [17.012, -3.105, 1.7511],
        [-0.31991, 0.94236, -0.09809],
        0.08,
        |ch| mul(ch("elevator"), 15.0),
        26,
    ),
    surface(
        "meridian190",
        "Right elevator",
        &[([16.296, 0.847, 1.361], [18.356, 5.239, 1.976])],
        [17.012, 3.105, 1.7511],
        [0.31991, 0.94236, 0.09809],
        0.08,
        |ch| mul(ch("elevator"), 15.0),
        26,
    ),
    // ATR 72 (`aerolink`): the elevators are the aft part of the "Pitch
    // trim" object (see [`REANIMATE`]); ±20° about the hinge line of the
    // model's elevator animation (its x1 / x2 points).
    StaticPart {
        aircraft: "aerolink",
        name: "Left elevator panel",
        boxes: &[([12.26, -3.5, 2.75], [13.1, 0.0, 3.1])],
        from: "rotate:Pitch trim",
        frame: "",
        center: [12.305, -1.75, 2.931],
        axis: [0.02228, 0.99966, -0.01337],
        fwd: 0.03,
        mats: &[],
        spin: false,
        deg: |ch| mul(ch("elevator"), 20.0),
        tris: 12,
    },
    StaticPart {
        aircraft: "aerolink",
        name: "Right elevator panel",
        boxes: &[([12.26, 0.0, 2.75], [13.1, 3.5, 3.1])],
        from: "rotate:Pitch trim",
        frame: "",
        center: [12.305, 1.75, 2.931],
        axis: [-0.02228, 0.99966, 0.01337],
        fwd: 0.03,
        mats: &[],
        spin: false,
        deg: |ch| mul(ch("elevator"), 20.0),
        tris: 12,
    },
    // Extra 500: the propeller and spinner spin on
    // sim/multiplay/generic/int[12] (unmapped) and stood still.
    StaticPart {
        aircraft: "extra500",
        name: "Propellix",
        boxes: &[([-3.35, -1.1, -1.3], [-2.8, 1.1, 0.7])],
        from: "",
        frame: "",
        center: [-3.08, 0.0, -0.405],
        axis: [-0.998, 0.0, -0.056],
        fwd: f32::INFINITY,
        mats: &[56, 57],
        spin: true,
        deg: |ch| ch("prop0_rpm"),
        tris: 3180 + 382,
    },
    // A320 (`meridian320`, FGAddon r21799 `Models/Fuselage/fuselage.xml`,
    // 0.831 m up in the model): every flight-control surface reads
    // `sim/multiplay/generic/float[4..9]` (the FBW's surface degrees) or
    // `systems/fcs/spoiler-*/final-deg`. Elevators +17 / −30°, ailerons ±25°,
    // rudder ±25°, spoilers (speed brakes) 40°.
    surface(
        "meridian320",
        "Rudder",
        &[([14.791, -0.228, 2.452], [18.238, 0.233, 8.686])],
        [16.044, 0.0004, 5.6301],
        [0.3961, -0.001, 0.91821],
        0.1,
        |ch| mul(ch("rudder"), 25.0),
        541,
    ),
    surface("meridian320", "ElevatorL", &[([15.525, -6.324, 1.412], [18.089, -0.923, 2.342])], [16.4855, -3.5089, 1.9006], [-0.32775, 0.93317, -0.14755], 0.1, a320_elevator, 698),
    surface("meridian320", "ElevatorR", &[([15.525, 0.927, 1.412], [18.089, 6.324, 2.342])], [16.4855, 3.5089, 1.9006], [0.32775, 0.93317, 0.14755], 0.1, a320_elevator, 698),
    surface(
        "meridian320",
        "AileronL",
        &[([1.403, -16.295, 0.741], [3.223, -13.282, 1.192])],
        [1.9908, -14.7875, 0.9749],
        [-0.33808, 0.93649, -0.0932],
        0.1,
        |ch| mul(ch("aileron_l"), 25.0),
        968,
    ),
    surface(
        "meridian320",
        "AileronR",
        &[([1.403, 13.279, 0.741], [3.223, 16.293, 1.192])],
        [1.9908, 14.7875, 0.9749],
        [0.33808, 0.93649, 0.0932],
        0.1,
        |ch| mul(ch("aileron_r"), 25.0),
        968,
    ),
    a320_spoiler("SpoilerL1", &[([-1.246, -6.188, -0.135], [-0.495, -4.477, 0.249])], [-1.2065, -5.3328, 0.0946], [0.00236, 0.98855, -0.15085], 410),
    a320_spoiler("SpoilerL2", &[([-1.03, -8.567, 0.197], [0.204, -6.674, 0.482])], [-0.7108, -7.7399, 0.3473], [-0.33663, 0.93747, -0.08846], 408),
    a320_spoiler("SpoilerL3", &[([-0.427, -10.13, 0.355], [0.758, -8.341, 0.63])], [-0.1282, -9.3617, 0.5005], [-0.33692, 0.93737, -0.08841], 408),
    a320_spoiler("SpoilerL4", &[([0.134, -11.627, 0.504], [1.293, -9.903, 0.771])], [0.4222, -10.8928, 0.6451], [-0.33652, 0.93752, -0.08843], 406),
    a320_spoiler("SpoilerL5", &[([0.671, -13.061, 0.647], [1.804, -11.412, 0.906])], [0.9491, -12.3567, 0.7834], [-0.33677, 0.93742, -0.0885], 406),
    a320_spoiler("SpoilerR1", &[([-1.246, 4.474, -0.135], [-0.495, 6.186, 0.249])], [-1.2072, 5.3305, 0.0945], [-0.00258, 0.98842, 0.15171], 410),
    a320_spoiler("SpoilerR2", &[([-1.03, 6.672, 0.197], [0.204, 8.565, 0.482])], [-0.7119, 7.7388, 0.3465], [0.33444, 0.93827, 0.08835], 408),
    a320_spoiler("SpoilerR3", &[([-0.427, 8.339, 0.355], [0.758, 10.128, 0.63])], [-0.1312, 9.3613, 0.4995], [0.33678, 0.93742, 0.0885], 408),
    a320_spoiler("SpoilerR4", &[([0.134, 9.9, 0.504], [1.293, 11.625, 0.771])], [0.4189, 10.8925, 0.644], [0.33677, 0.93742, 0.08849], 406),
    a320_spoiler("SpoilerR5", &[([0.671, 11.409, 0.647], [1.804, 13.058, 0.906])], [0.9454, 12.3579, 0.7824], [0.33677, 0.93742, 0.0885], 406),
    // Extra 500: the rudder ("SR") reads fdm/jsbsim/fcs/rudder-pos-norm1
    // (±25°), unmapped; the airframe is pitched 1.39° (the elevators' frame).
    StaticPart {
        aircraft: "extra500",
        name: "SR",
        boxes: &[([5.366, -0.052, -0.252], [6.481, 0.052, 1.141])],
        from: "",
        frame: "rotate:HRlinks",
        center: [5.761, 0.0, 0.4837],
        axis: [0.51505, 0.0, 0.85716],
        fwd: 0.05,
        mats: &[],
        spin: false,
        deg: |ch| mul(ch("rudder"), 25.0),
        tris: 30,
    },
];

/// A320 elevators: trailing edge down 17°, up 30°.
fn a320_elevator(ch: &mut dyn FnMut(&str) -> RExpr) -> RExpr {
    RExpr::Interp(Box::new(ch("elevator")), vec![(-1.0, -30.0), (0.0, 0.0), (1.0, 17.0)])
}

/// Animations whose deflection is replaced: (aircraft, node name, new
/// expression), applied before the sense check (so written in the model's
/// own sign convention).
/// * F-16: the left stabilator reads `elevator-pos-norm`, which the F-16
///   FCS writes as −(left tail radians) (× 57.3 in the model): ±57° for
///   Skyward's ±1, and against the right stabilator's sense — it gets the
///   stabilator law above (the model rotates the left side by −deflection).
///   The leading-edge flaps read `flap-pos-norm`, which the F-16 FCS writes
///   in degrees (0 .. 25): Skyward's 0..1 flap channel drooped them ≤ 1°.
/// * ASK 21 (`swift`): elevator and rudder travel ±6° in the model, too
///   little to see; the real glider's are about 18° / 15° and ±25°.
/// * C-5 (`galaxy`): Spoiler1 and both Spoiler11 panels rotate the other way
///   round from the other ten (they went down as the rest went up), and
///   Spoiler11 only 30°.
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
    Rescale { aircraft: "swift", node: "rotate:elevator", deg: |ch| RExpr::Interp(Box::new(ch("elevator")), vec![(-1.0, -18.0), (0.0, 0.0), (1.0, 15.0)]) },
    Rescale { aircraft: "swift", node: "rotate:rudder", deg: |ch| mul(ch("rudder"), 25.0) },
    Rescale { aircraft: "galaxy", node: "rotate:Spoiler1", deg: |ch| mul(ch("spoilers"), -45.0) },
    Rescale { aircraft: "galaxy", node: "rotate:Spoiler11.Inside", deg: |ch| mul(ch("spoilers"), -45.0) },
    Rescale { aircraft: "galaxy", node: "rotate:Spoiler11.Outside", deg: |ch| mul(ch("spoilers"), -45.0) },
];

/// Surfaces hinged in the wrong place: (aircraft, node name, hinge centre in
/// the node's frame), on the same axis direction. The C-130's elevator
/// turned about a line 12 cm behind its leading edge (x 11.76; the leading
/// edge rose and fell 7 cm against the tailplane), the Cub's rudder 2 cm
/// behind its post: both now turn about the leading edge (behind the horn
/// balances). The Otter's elevator, which the old audit took for one hinged
/// 15 cm too far aft, turns about the centre of its rounded nose already
/// (its horn balances pull the centroid forward).
pub static REHINGE: &[(&str, &str, [f32; 3])] = &[("hercules", "rotate:profondeur", [11.79, 0.0, -0.966]), ("cub", "rotate:direction", [2.91, 0.0, 0.6])];

/// Rotations replaced outright (centre and axis in the node's frame,
/// expression in Skyward's convention). ATR 72 (`aerolink`): the model's
/// "ElevatorL" / "ElevatorR" objects are the fixed tailplane (x 11.1 ..
/// 12.3) and rotated with the control column about their own trailing edge;
/// the real elevators, the aft part of the "Pitch trim" object, moved only
/// with the trim wheel. The tailplane halves stay put, the elevators are cut
/// out of the trim object ([`STATIC_PARTS`]).
pub struct Reanimate {
    pub aircraft: &'static str,
    pub node: &'static str,
    pub center: [f32; 3],
    pub axis: [f32; 3],
    pub deg: fn(&mut dyn FnMut(&str) -> RExpr) -> RExpr,
}

pub static REANIMATE: &[Reanimate] = &[
    Reanimate { aircraft: "aerolink", node: "rotate:Left elevator", center: [12.305, -1.75, 2.931], axis: [0.0, 1.0, 0.0], deg: |_| RExpr::C(0.0) },
    Reanimate { aircraft: "aerolink", node: "rotate:Right elevator", center: [12.305, 1.75, 2.931], axis: [0.0, 1.0, 0.0], deg: |_| RExpr::C(0.0) },
];

fn reanimate(a: &mut AircraftAsset, log: &mut Vec<String>) {
    let id = a.id.clone();
    for r in REANIMATE.iter().filter(|r| r.aircraft == id) {
        let Some(k) = a.nodes.iter().position(|n| n.name == r.node) else { continue };
        let deg = (r.deg)(&mut |c| channel_expr(a, c));
        if let Some(Anim::Rotate { .. }) = &a.nodes[k].anim {
            a.nodes[k].anim = Some(Anim::Rotate { center: Vec3::from(r.center), axis: Vec3::from(r.axis).normalize(), deg });
            log.push(format!("{}: reanimated", r.node));
        }
    }
}

fn rehinge(a: &mut AircraftAsset, log: &mut Vec<String>) {
    for (_, name, c) in REHINGE.iter().filter(|r| r.0 == a.id) {
        for n in a.nodes.iter_mut().filter(|n| n.name == *name) {
            if let Some(Anim::Rotate { center, .. }) = &mut n.anim {
                log.push(format!("{name}: hinge moved {:.2} m", (Vec3::from(*c) - *center).length()));
                *center = Vec3::from(*c);
            }
        }
    }
}

/// Exterior control surfaces animated by the cockpit controls
/// (`controls/flight/*` in the model: `ctl_elevator`, `ctl_aileron`,
/// `ctl_rudder`) read the surface channels instead, so that they follow the
/// flight model's surfaces (and `SKYWARD_SURFACES`) like the rest; their
/// sense is checked separately ([`normalize_senses`]). Only nodes that rotate
/// exterior geometry hinged ahead of it (not yokes or pedals). Returns the
/// nodes changed.
const CTL_TO_SURFACE: [(&str, &str); 3] = [("ctl_elevator", "elevator"), ("ctl_aileron", "aileron_l"), ("ctl_rudder", "rudder")];

fn ctl_to_surface(a: &mut AircraftAsset, log: &mut Vec<String>) -> Vec<usize> {
    let mut changed = Vec::new();
    // (Rotorcraft: the cyclic tilts the swashplate, not a surface.)
    if a.channel("rotor_rpm").is_some() {
        return changed;
    }
    for (ctl, surf) in CTL_TO_SURFACE {
        let Some(c) = a.channel(ctl) else { continue };
        let nodes: Vec<usize> = (0..a.nodes.len()).filter(|&i| node_reads(&a.nodes[i], c as u16) && trailing_edge_motion(a, i, c, Sense::Down).is_some()).collect();
        if nodes.is_empty() {
            continue;
        }
        let RExpr::Ch(s) = channel_expr(a, surf) else { unreachable!() };
        for &i in &nodes {
            if let Some(Anim::Rotate { deg, .. }) = &mut a.nodes[i].anim {
                replace_channel(deg, c as u16, &RExpr::Ch(s));
            }
            log.push(format!("{}: {ctl} → {surf}", a.nodes[i].name));
        }
        changed.extend(nodes);
    }
    changed
}

/// Replace every read of channel `ch` in `e` by `with`.
fn replace_channel(e: &mut RExpr, ch: u16, with: &RExpr) {
    match e {
        RExpr::Ch(i) if *i == ch => *e = with.clone(),
        RExpr::Op(_, args) => args.iter_mut().for_each(|x| replace_channel(x, ch, with)),
        RExpr::Interp(x, _) => replace_channel(x, ch, with),
        _ => {}
    }
}

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

/// Parts that are not part of the aircraft: (aircraft, node, why). F-16:
/// "mfd-flir" (`Cockpit/Avionics/MFD/mfd-ext.xml`) is a FlightGear helper
/// screen that follows the pilot's view (turned by
/// `sim/current-view/heading- / pitch-offset-deg`, scaled with the field of
/// view); folded to its rest place it hung as a dark slab under the intake
/// in every view. (`tools/fgimport/aircraft/f16.json` now excludes it.)
pub static HIDE: &[(&str, &str, &str)] = &[("lancer", "scale:mfd-flir", "view-following MFD helper screen")];

fn hide(a: &mut AircraftAsset, log: &mut Vec<String>) {
    for (_, name, why) in HIDE.iter().filter(|h| h.0 == a.id) {
        for n in a.nodes.iter_mut().filter(|n| n.name == *name) {
            n.anim = Some(Anim::Select(RExpr::C(0.0)));
            log.push(format!("{name}: hidden ({why})"));
        }
    }
}

/// Apply every correction; returns what was changed (for logs and tests).
pub fn apply(a: &mut AircraftAsset) -> Vec<String> {
    let mut log = Vec::new();
    hide(a, &mut log);
    rescale(a, &mut log);
    rehinge(a, &mut log);
    reanimate(a, &mut log);
    let remapped = ctl_to_surface(a, &mut log);
    close_nose_doors(a, &mut log);
    normalize_senses(a, &remapped, &mut log);
    // (Written in Skyward's convention: after the sense check.)
    bind_static_parts(a, &mut log);
    normalize_steering(a, &mut log);
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
        let src = if p.from.is_empty() {
            root
        } else {
            match a.nodes.iter().position(|n| n.name == p.from) {
                Some(k) => k,
                None => continue,
            }
        };
        let pre = if p.frame.is_empty() {
            Mat4::IDENTITY
        } else {
            match a.nodes.iter().find(|n| n.name == p.frame) {
                Some(n) => n.pre,
                None => continue,
            }
        };
        let to_frame = pre.inverse();
        let (center, axis) = (Vec3::from(p.center), Vec3::from(p.axis).normalize());
        let aft = (Vec3::X - axis * axis.x).normalize_or_zero();
        let inside = |v: Vec3| {
            let v = to_frame.transform_point3(v);
            let d = v - center;
            p.boxes.iter().any(|(lo, hi)| {
                let (lo, hi) = (Vec3::from(*lo) - 0.003, Vec3::from(*hi) + 0.003);
                v.cmpge(lo).all() && v.cmple(hi).all()
            }) && -(d - axis * d.dot(axis)).dot(aft) <= p.fwd
        };
        let deg = (p.deg)(&mut |c| channel_expr(a, c));
        let node = a.nodes.len();
        let mut moved = 0;
        let mut new_chunks: Vec<Chunk> = Vec::new();
        for c in a.chunks.iter_mut().filter(|c| c.node == src && !c.interior && (p.mats.is_empty() || p.mats.contains(&c.material))) {
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
            name: format!("{}:{}", if p.spin { "spin" } else { "rotate" }, p.name),
            parent: src as i32,
            // (The triangles stay in the source node's frame: so does the hinge.)
            pre: Mat4::IDENTITY,
            anim: Some(if p.spin {
                Anim::Spin { center: pre.transform_point3(center), axis: pre.transform_vector3(axis).normalize(), rpm: deg }
            } else {
                Anim::Rotate { center: pre.transform_point3(center), axis: pre.transform_vector3(axis).normalize(), deg }
            }),
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
    /// Trailing edge up (spoilers, speed brakes: deployed).
    Up,
}

pub const SURFACE_CHANNELS: [(&str, Sense); 6] =
    [("elevator", Sense::Down), ("aileron_l", Sense::Roll(true)), ("aileron_r", Sense::Roll(false)), ("flaps", Sense::Down), ("rudder", Sense::Right), ("spoilers", Sense::Up)];

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
    // (What moves: not children that turn back, like the ATR's tailplane.)
    pts.retain(|(p, q)| p.distance_squared(*q) > 1e-6);
    if pts.is_empty() {
        return None;
    }
    let centroid = pts.iter().map(|(p, _)| *p).sum::<Vec3>() / pts.len() as f32;
    // Not wheels on the centreline below the CG (steered with the rudder),
    // nor fuselage speed brakes (split: one panel up, one down).
    let half_span = pts.iter().fold(0.0f32, |m, (p, _)| m.max(p.x.abs()));
    if (sense == Sense::Right && centroid.y < -0.3 && half_span < 1.0) || (sense == Sense::Up && half_span < 1.5) {
        return None;
    }
    // Trailing-edge surfaces only: hinged ahead of their geometry (not
    // slats, leading-edge flaps, nose visors, gear legs, rods).
    let Some(Anim::Rotate { center, axis, .. }) = &a.nodes[node].anim else { return None };
    let hinge = p0.world[node].transform_point3(*center);
    let axis = p0.world[node].transform_vector3(*axis).normalize_or_zero();
    // (Off the hinge line, not off its centre point: swept hinges.)
    let arm = |p: Vec3| {
        let d = p - hinge;
        (d - axis * d.dot(axis)).z
    };
    let (zmin, zmax) = pts.iter().fold((f32::MAX, f32::MIN), |(l, h), (p, _)| (l.min(p.z), h.max(p.z)));
    let cut = zmin + 0.2 * (zmax - zmin);
    if arm(centroid) > -0.015 {
        // Horn-balanced surfaces (the Otter's elevator) carry much of their
        // geometry ahead of the hinge: then the trailing fifth must reach
        // further behind it than the leading fifth reaches ahead.
        let mean_arm = |f: &dyn Fn(f32) -> bool| {
            let v: Vec<f32> = pts.iter().filter(|(p, _)| f(p.z)).map(|(p, _)| arm(*p)).collect();
            v.iter().sum::<f32>() / v.len().max(1) as f32
        };
        let (aft, fore) = (mean_arm(&|z| z <= cut), mean_arm(&|z| z >= zmax - 0.2 * (zmax - zmin)));
        if aft > -0.05 || -aft < fore {
            return None;
        }
    }
    let te: Vec<Vec3> = pts.iter().filter(|(p, _)| p.z <= cut).map(|(p, q)| *q - *p).collect();
    let d = te.iter().copied().sum::<Vec3>() / te.len() as f32;
    // Body frame: +Y up, +X left.
    let along = match sense {
        Sense::Down => -d.y,
        Sense::Up => d.y,
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
/// The surfaces moved over from the cockpit-control channels (`remapped`,
/// see [`ctl_to_surface`]) vote apart: FlightGear's control and surface
/// properties need not share a sign convention; where they go the wrong way
/// their own reads are negated.
fn normalize_senses(a: &mut AircraftAsset, remapped: &[usize], log: &mut Vec<String>) {
    let motions = surface_motions(a);
    for (name, _) in SURFACE_CHANNELS {
        let Some(ch) = a.channel(name) else { continue };
        let wrong = |own: bool| {
            let (mut good, mut bad) = (0.0f32, 0.0f32);
            for &(_, _, m, nv) in motions.iter().filter(|m| m.0 == name && remapped.contains(&m.1) != own) {
                let w = m.abs() * (nv as f32).sqrt();
                if m > 0.0 {
                    good += w;
                } else {
                    bad += w;
                }
            }
            bad > 3.0 * good && bad > 0.0
        };
        let (imported, moved) = (wrong(true), wrong(false));
        if imported {
            negate_channel(a, ch as u16);
            log.push(format!("{name}: surfaces moved the wrong way, channel negated"));
        }
        // (Negating the channel negated the moved-over surfaces too.)
        if moved != imported {
            for &i in remapped {
                if !node_reads(&a.nodes[i], ch as u16) {
                    continue;
                }
                if let Some(Anim::Rotate { deg, .. }) = &mut a.nodes[i].anim {
                    replace_channel(deg, ch as u16, &RExpr::Op(Op::Neg, vec![RExpr::Ch(ch as u16)]));
                    log.push(format!("{}: {name} negated", a.nodes[i].name));
                }
            }
        }
    }
}

/// Nose-wheel steering: +1 on the steering channels (`steer_norm`,
/// `steer_deg`, or `rudder` where the model steers with it) must turn the
/// nose wheel right, as the rudder pedals do. Per steering node of a nose
/// gear (exterior geometry ahead of the CG, below it, about a near-vertical
/// axis) the turn is measured and the node's reads negated where it goes
/// left. Tail wheels turn with the rudder and are left alone.
fn normalize_steering(a: &mut AircraftAsset, log: &mut Vec<String>) {
    for (i, ch, right) in steering_turns(a) {
        if right < 0.0 {
            if let Some(Anim::Rotate { deg, .. }) = &mut a.nodes[i].anim {
                replace_channel(deg, ch as u16, &RExpr::Op(Op::Neg, vec![RExpr::Ch(ch as u16)]));
                log.push(format!("{}: steered left, {} negated", a.nodes[i].name, a.channels[ch]));
            }
        }
    }
}

/// The steering channels.
pub const STEER_CHANNELS: [&str; 3] = ["steer_norm", "steer_deg", "rudder"];

/// Nose-wheel steering nodes: (node, channel, turn to the right in degrees
/// for +1 on the channel, 20 for `steer_deg`). See [`normalize_steering`].
pub fn steering_turns(a: &AircraftAsset) -> Vec<(usize, usize, f32)> {
    let mut out = Vec::new();
    let rest = rest_channels(a);
    let mut p0 = Pose::default();
    a.pose(&rest, &mut AnimState::default(), 0.0, &mut p0);
    for name in STEER_CHANNELS {
        let Some(ch) = a.channel(name) else { continue };
        let mut moved = rest.clone();
        moved[ch] = if name == "steer_deg" { 20.0 } else { 1.0 };
        let mut p1 = Pose::default();
        a.pose(&moved, &mut AnimState::default(), 0.0, &mut p1);
        for (i, n) in a.nodes.iter().enumerate() {
            let Some(Anim::Rotate { axis, .. }) = &n.anim else { continue };
            if !node_reads(n, ch as u16) || p0.world[i].transform_vector3(*axis).normalize_or_zero().y.abs() < 0.7 {
                continue;
            }
            // The subtree's exterior geometry: a nose gear, ahead of the CG and below it.
            let mut in_sub = vec![false; a.nodes.len()];
            for (k, m) in a.nodes.iter().enumerate() {
                in_sub[k] = k == i || (m.parent >= 0 && in_sub[m.parent as usize]);
            }
            let (mut s, mut cnt) = (Vec3::ZERO, 0);
            for c in a.chunks.iter().filter(|c| !c.interior && in_sub[c.node] && p0.visible[c.node]) {
                for k in c.first..c.first + c.count {
                    s += p0.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos));
                    cnt += 1;
                }
            }
            let cen = s / cnt.max(1) as f32;
            if cnt == 0 || cen.z < 0.5 || cen.y > 0.0 {
                continue;
            }
            // Heading change of the node in the body frame (+Z nose, +X left:
            // right = the nose of the wheel towards −X).
            let f = (p1.world[i] * p0.world[i].inverse()).transform_vector3(Vec3::Z);
            let right = (-f.x).atan2(f.z).to_degrees();
            if right.abs() > 0.5 {
                out.push((i, ch, right));
            }
        }
    }
    out
}

#[cfg(test)]
mod checks;
#[cfg(test)]
mod tests;
