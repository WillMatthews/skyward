//! JSBSim-compatible aerodynamics evaluator.
//!
//! Reads the `<metrics>` and `<aerodynamics>` sections of a JSBSim FDM
//! (FlightGear aircraft, imported by `tools/fgimport` into
//! `assets/aircraft/<id>/fdm.xml`) and evaluates its coefficient functions
//! exactly as JSBSim does: `<function>` trees (products, sums, quotients,
//! tables in 1–3 dimensions, comparisons, trigonometry, …) over a property
//! table, the axis sums (DRAG/SIDE/LIFT in wind axes, ROLL/PITCH/YAW about
//! the aerodynamic reference point, or X/Y/Z body forces) and the stall
//! hysteresis. Results are in SI and in Skyward's body frame (+Z nose, +Y
//! up, +X left), moments about the CG.
//!
//! The flight model (`flight.rs`) feeds the inputs (air data, rates, surface
//! deflections, gear, ground-effect height, propeller wash …) and integrates
//! the forces; flight controls, engines and landing gear stay Skyward's own.

use glam::{DVec3, Vec3};
use std::collections::HashMap;

const FT: f64 = 0.3048;
const LBF: f64 = 4.448_221_6;

#[derive(Clone, Debug)]
enum F {
    V(f64),
    /// Property slot, negated.
    P(usize, bool),
    Op(Op, Vec<F>),
    T(Box<Table>),
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Op {
    Product,
    Sum,
    Difference,
    Quotient,
    Pow,
    Sqrt,
    Abs,
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Atan2,
    Exp,
    Ln,
    Log2,
    Log10,
    Min,
    Max,
    Avg,
    Fraction,
    Integer,
    Mod,
    ToRadians,
    ToDegrees,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Nq,
    And,
    Or,
    Not,
    IfThen,
    Sign,
    Floor,
    Ceil,
    Pi,
    Zero,
}

#[derive(Clone, Debug)]
struct Table {
    /// Lookup slots: row, column, table.
    row: Option<usize>,
    col: Option<usize>,
    tab: Option<usize>,
    rows: Vec<f64>,
    cols: Vec<f64>,
    /// rows × cols (cols = 1 for a 1-D table).
    data: Vec<f64>,
    /// 3-D: (breakpoint, 2-D table).
    pages: Vec<(f64, Table)>,
}

fn bracket(v: &[f64], x: f64) -> (usize, f64) {
    let n = v.len();
    if n < 2 || x <= v[0] {
        return (0, 0.0);
    }
    if x >= v[n - 1] {
        return (n - 2, 1.0);
    }
    let mut i = 0;
    while i + 2 < n && x > v[i + 1] {
        i += 1;
    }
    let d = v[i + 1] - v[i];
    (i, if d.abs() < 1e-30 { 0.0 } else { (x - v[i]) / d })
}

impl Table {
    fn eval(&self, p: &[f64]) -> f64 {
        let get = |s: Option<usize>| s.map(|i| p[i]).unwrap_or(0.0);
        if !self.pages.is_empty() {
            let x = get(self.tab);
            let keys: Vec<f64> = self.pages.iter().map(|(k, _)| *k).collect();
            if keys.len() == 1 {
                return self.pages[0].1.eval(p);
            }
            let (i, f) = bracket(&keys, x);
            let a = self.pages[i].1.eval(p);
            let b = self.pages[i + 1].1.eval(p);
            return a + f * (b - a);
        }
        let nc = self.cols.len().max(1);
        if self.cols.is_empty() {
            if self.rows.len() == 1 {
                return self.data[0];
            }
            let (i, f) = bracket(&self.rows, get(self.row));
            return self.data[i] + f * (self.data[i + 1] - self.data[i]);
        }
        let (ri, rf) = if self.rows.len() > 1 { bracket(&self.rows, get(self.row)) } else { (0, 0.0) };
        let (ci, cf) = if self.cols.len() > 1 { bracket(&self.cols, get(self.col)) } else { (0, 0.0) };
        let at = |r: usize, c: usize| self.data[r.min(self.rows.len() - 1) * nc + c.min(nc - 1)];
        let r1 = if self.rows.len() > 1 { ri + 1 } else { ri };
        let c1 = if self.cols.len() > 1 { ci + 1 } else { ci };
        let a = at(ri, ci) + cf * (at(ri, c1) - at(ri, ci));
        let b = at(r1, ci) + cf * (at(r1, c1) - at(r1, ci));
        a + rf * (b - a)
    }
}

fn eval(f: &F, p: &[f64]) -> f64 {
    match f {
        F::V(v) => *v,
        F::P(i, neg) => {
            if *neg {
                -p[*i]
            } else {
                p[*i]
            }
        }
        F::T(t) => t.eval(p),
        F::Op(o, a) => {
            let g = |i: usize| a.get(i).map(|e| eval(e, p)).unwrap_or(0.0);
            let b = |x: bool| if x { 1.0 } else { 0.0 };
            match o {
                Op::Product => a.iter().map(|e| eval(e, p)).product(),
                Op::Sum => a.iter().map(|e| eval(e, p)).sum(),
                Op::Difference => g(0) - a.iter().skip(1).map(|e| eval(e, p)).sum::<f64>(),
                Op::Quotient => {
                    let d = g(1);
                    if d == 0.0 { f64::INFINITY.copysign(g(0)).min(1e30).max(-1e30) } else { g(0) / d }
                }
                Op::Pow => g(0).powf(g(1)),
                Op::Sqrt => g(0).max(0.0).sqrt(),
                Op::Abs => g(0).abs(),
                Op::Sin => g(0).sin(),
                Op::Cos => g(0).cos(),
                Op::Tan => g(0).tan(),
                Op::Asin => g(0).clamp(-1.0, 1.0).asin(),
                Op::Acos => g(0).clamp(-1.0, 1.0).acos(),
                Op::Atan => g(0).atan(),
                Op::Atan2 => g(0).atan2(g(1)),
                Op::Exp => g(0).exp(),
                Op::Ln => g(0).max(1e-300).ln(),
                Op::Log2 => g(0).max(1e-300).log2(),
                Op::Log10 => g(0).max(1e-300).log10(),
                Op::Min => a.iter().map(|e| eval(e, p)).fold(f64::INFINITY, f64::min),
                Op::Max => a.iter().map(|e| eval(e, p)).fold(f64::NEG_INFINITY, f64::max),
                Op::Avg => {
                    if a.is_empty() {
                        0.0
                    } else {
                        a.iter().map(|e| eval(e, p)).sum::<f64>() / a.len() as f64
                    }
                }
                Op::Fraction => g(0).fract(),
                Op::Integer => g(0).trunc(),
                Op::Mod => {
                    let d = g(1);
                    if d == 0.0 { 0.0 } else { g(0) % d }
                }
                Op::ToRadians => g(0).to_radians(),
                Op::ToDegrees => g(0).to_degrees(),
                Op::Lt => b(g(0) < g(1)),
                Op::Le => b(g(0) <= g(1)),
                Op::Gt => b(g(0) > g(1)),
                Op::Ge => b(g(0) >= g(1)),
                Op::Eq => b(g(0) == g(1)),
                Op::Nq => b(g(0) != g(1)),
                Op::And => b(a.iter().all(|e| eval(e, p) != 0.0)),
                Op::Or => b(a.iter().any(|e| eval(e, p) != 0.0)),
                Op::Not => b(g(0) == 0.0),
                Op::IfThen => {
                    if g(0) != 0.0 {
                        g(1)
                    } else {
                        g(2)
                    }
                }
                Op::Sign => {
                    if g(0) < 0.0 {
                        -1.0
                    } else {
                        1.0
                    }
                }
                Op::Floor => g(0).floor(),
                Op::Ceil => g(0).ceil(),
                Op::Pi => std::f64::consts::PI,
                Op::Zero => 0.0,
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Axis {
    Drag,
    Side,
    Lift,
    Roll,
    Pitch,
    Yaw,
    X,
    Y,
    Z,
    Axial,
    Normal,
}

/// Property slots with fixed meaning (inputs set by the flight model).
#[derive(Clone, Copy, Debug)]
pub enum In {
    Qbar,
    Alpha,
    AlphaDeg,
    Beta,
    BetaDeg,
    AlphaDot,
    BetaDot,
    Bi2Vel,
    Ci2Vel,
    P,
    Q,
    R,
    Mach,
    Vt,
    Elevator,
    ElevatorDeg,
    ElevatorNorm,
    AileronL,
    AileronR,
    AileronLDeg,
    AileronRDeg,
    AileronNorm,
    Rudder,
    RudderDeg,
    RudderNorm,
    FlapDeg,
    FlapNorm,
    SlatNorm,
    SlatDeg,
    Speedbrake,
    Spoiler,
    SpoilerDeg,
    Gear,
    HbMac,
    HbCg,
    Rho,
    PropWash,
    Re,
    StabDeg,
    StabNorm,
    LefNorm,
    LefRNorm,
}

pub const N_IN: usize = 42;

/// Default property names for each input (aircraft may add aliases).
fn default_names(i: In) -> &'static [&'static str] {
    use In::*;
    match i {
        Qbar => &["aero/qbar-psf", "aero/qbarUW-psf", "aero/qbarUV-psf"],
        Alpha => &["aero/alpha-rad", "aero/alpha-wing-rad"],
        AlphaDeg => &["aero/alpha-deg"],
        Beta => &["aero/beta-rad"],
        BetaDeg => &["aero/beta-deg"],
        AlphaDot => &["aero/alphadot-rad_sec"],
        BetaDot => &["aero/betadot-rad_sec"],
        Bi2Vel => &["aero/bi2vel"],
        Ci2Vel => &["aero/ci2vel"],
        P => &["velocities/p-aero-rad_sec", "velocities/p-rad_sec"],
        Q => &["velocities/q-aero-rad_sec", "velocities/q-rad_sec"],
        R => &["velocities/r-aero-rad_sec", "velocities/r-rad_sec"],
        Mach => &["velocities/mach"],
        Vt => &["velocities/vt-fps", "velocities/u-aero-fps"],
        Elevator => &["fcs/elevator-pos-rad", "fcs/elevator-lh-pos-rad", "fcs/elevator-rh-pos-rad"],
        ElevatorDeg => &["fcs/elevator-pos-deg", "fcs/fly-by-wire/pitch/horz-tail-deflection-deg", "/systems/fcs/elevator-l/final-deg", "/systems/fcs/elevator-r/final-deg"],
        ElevatorNorm => &["fcs/elevator-pos-norm", "fcs/elevator-lh-pos-norm", "fcs/elevator-rh-pos-norm"],
        AileronL => &["fcs/left-aileron-pos-rad", "fcs/aileron-pos-rad"],
        AileronR => &["fcs/right-aileron-pos-rad"],
        AileronLDeg => &["fcs/left-aileron-pos-deg", "/systems/fcs/aileron-l/final-deg"],
        AileronRDeg => &["fcs/right-aileron-pos-deg", "/systems/fcs/aileron-r/final-deg"],
        AileronNorm => &["fcs/left-aileron-pos-norm", "fcs/aileron-pos-norm"],
        Rudder => &["fcs/rudder-pos-rad"],
        RudderDeg => &["fcs/rudder-pos-deg", "/systems/fcs/rudder/final-deg"],
        RudderNorm => &["fcs/rudder-pos-norm"],
        FlapDeg => &["fcs/flap-pos-deg"],
        FlapNorm => &["fcs/flap-pos-norm", "fcs/fly-by-wire/tef/tef-pos-norm"],
        SlatNorm => &["fcs/slat-pos-norm"],
        SlatDeg => &["fcs/slat-pos-deg"],
        Speedbrake => &["fcs/speedbrake-pos-norm"],
        Spoiler => &["fcs/spoiler-pos-norm", "fcs/mfs1-pos-norm", "fcs/mfs2-pos-norm", "fcs/mfs3-pos-norm", "fcs/mfs4-pos-norm", "fcs/mfs5-pos-norm", "fcs/mfs6-pos-norm", "fcs/mfs7-pos-norm", "fcs/mfs8-pos-norm", "fcs/mfs9-pos-norm", "fcs/mfs10-pos-norm"],
        SpoilerDeg => &["fcs/spoiler-symmetrical-deg", "fcs/spoiler-left-deg", "fcs/spoiler-right-deg"],
        Gear => &["gear/gear-pos-norm", "/systems/gear/all-norm"],
        HbMac => &["aero/h_b-mac-ft"],
        HbCg => &["aero/h_b-cg-ft"],
        Rho => &["atmosphere/rho-slugs_ft3"],
        PropWash => &["propulsion/engine/prop-induced-velocity_fps", "propulsion/engine[0]/prop-induced-velocity_fps"],
        Re => &["aero/Re"],
        StabDeg => &["/systems/fcs/stabilizer/final-deg"],
        StabNorm => &["fcs/hstab-pos-norm"],
        LefNorm => &["fcs/fly-by-wire/lef/lef-pos-norm"],
        LefRNorm => &["fcs/fly-by-wire/lef/lef-pos-r-norm"],
    }
}

const ALL_IN: [In; N_IN] = {
    use In::*;
    [
        Qbar, Alpha, AlphaDeg, Beta, BetaDeg, AlphaDot, BetaDot, Bi2Vel, Ci2Vel, P, Q, R, Mach, Vt, Elevator, ElevatorDeg, ElevatorNorm, AileronL, AileronR,
        AileronLDeg, AileronRDeg, AileronNorm, Rudder, RudderDeg, RudderNorm, FlapDeg, FlapNorm, SlatNorm, SlatDeg, Speedbrake, Spoiler, SpoilerDeg, Gear, HbMac,
        HbCg, Rho, PropWash, Re, StabDeg, StabNorm, LefNorm, LefRNorm,
    ]
};

/// Aerodynamic inputs in SI / radians (Skyward state).
#[derive(Clone, Copy, Debug, Default)]
pub struct AeroInput {
    /// Air-relative velocity in the body frame (m/s, +Z nose, +Y up, +X left).
    pub vb: Vec3,
    /// Body rates p (roll right), q (pitch up), r (yaw right), rad/s.
    pub p: f32,
    pub q: f32,
    pub r: f32,
    pub alpha_dot: f32,
    pub rho: f32,
    pub mach: f32,
    /// Surface deflections (JSBSim sign conventions), rad.
    pub elevator: f32,
    pub aileron_l: f32,
    pub aileron_r: f32,
    pub rudder: f32,
    /// Normalised positions -1..1 (for FDMs that use `*-pos-norm`).
    pub elevator_norm: f32,
    pub aileron_norm: f32,
    pub rudder_norm: f32,
    pub flap_deg: f32,
    pub flap_norm: f32,
    pub slat_norm: f32,
    pub slat_deg: f32,
    pub speedbrake: f32,
    pub spoiler: f32,
    pub spoiler_deg: f32,
    /// Gear position, 1 = down.
    pub gear: f32,
    /// Height of the wing / CG above ground, m.
    pub h_mac: f32,
    pub h_cg: f32,
    /// Propeller-induced velocity, m/s.
    pub prop_wash: f32,
    pub stab_deg: f32,
    pub stab_norm: f32,
    /// Leading-edge flap 0..1 (F-16 schedule).
    pub lef_norm: f32,
    /// CG travel aft of the loaded CG, m (Concorde fuel transfer): moves
    /// the moment reference forward relative to the CG.
    pub cg_aft_m: f32,
}

/// Forces / moments in Skyward body axes, about the CG (N, N·m), plus coefficients.
#[derive(Clone, Copy, Debug, Default)]
pub struct AeroOutput {
    pub force: Vec3,
    pub moment: Vec3,
    pub alpha: f32,
    pub beta: f32,
    /// Lift / drag / side coefficients (wind axes), for gauges and fitting.
    pub cl: f32,
    pub cd: f32,
    pub cy: f32,
}

pub struct JsbAero {
    names: HashMap<String, usize>,
    pub n_slots: usize,
    /// Pre-axis functions (aero/…): (slot, function), evaluated in order.
    pre: Vec<(usize, F)>,
    axes: Vec<(Axis, Vec<(Option<usize>, F)>)>,
    inputs: [Vec<usize>; N_IN],
    /// Constants set once (metrics, aircraft-specific defaults).
    consts: Vec<(usize, f64)>,
    pub sw_m2: f64,
    pub bw_m: f64,
    pub cbar_m: f64,
    /// Aerodynamic reference point relative to the CG, body frame (m).
    pub arp: Vec3,
    /// Wing incidence (rad), added for aero/alpha-wing-rad.
    pub incidence: f64,
    alpha_hyst: Option<(f64, f64)>,
    /// `<aero_ref_pt_shift_x>`: the reference point moves aft by this many
    /// chords (Concorde's centre-of-pressure shift with Mach).
    arp_shift: Option<F>,
    stall_slot: Option<usize>,
    cl2_slot: Option<usize>,
    pub unresolved: Vec<String>,
}

/// Per-aircraft evaluator state (stall hysteresis, last lift coefficient).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JsbState {
    pub stalled: bool,
    pub last_cl: f32,
}

thread_local! {
    static SCRATCH: std::cell::RefCell<Vec<f64>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn text(n: &roxmltree::Node) -> String {
    n.text().unwrap_or("").trim().to_string()
}

fn unit_k(n: &roxmltree::Node, si: &[(&str, f64)], default: f64) -> f64 {
    let u = n.attribute("unit").unwrap_or("");
    si.iter().find(|(k, _)| *k == u).map(|(_, v)| *v).unwrap_or(default)
}

impl JsbAero {
    fn slot(&mut self, name: &str) -> usize {
        let name = name.trim().trim_start_matches('/').replace("[0]", "");
        if let Some(&i) = self.names.get(&name) {
            return i;
        }
        let i = self.n_slots;
        self.names.insert(name, i);
        self.n_slots += 1;
        i
    }

    pub fn slot_of(&self, name: &str) -> Option<usize> {
        self.names.get(name.trim().trim_start_matches('/')).copied()
    }

    /// Parse from FDM XML text. `cg_struct_m`: CG in JSBSim structural
    /// metres (x aft, y right, z up) for the moment reference transfer.
    pub fn parse(xml: &str, aliases: &[(&str, &str)], consts: &[(&str, f64)]) -> Result<JsbAero, String> {
        let doc = roxmltree::Document::parse_with_options(xml, roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() }).map_err(|e| e.to_string())?;
        let root = doc.root_element();
        let mut a = JsbAero {
            names: HashMap::new(),
            n_slots: 0,
            pre: Vec::new(),
            axes: Vec::new(),
            inputs: std::array::from_fn(|_| Vec::new()),
            consts: Vec::new(),
            sw_m2: 0.0,
            bw_m: 0.0,
            cbar_m: 0.0,
            arp: Vec3::ZERO,
            incidence: 0.0,
            alpha_hyst: None,
            arp_shift: None,
            stall_slot: None,
            cl2_slot: None,
            unresolved: Vec::new(),
        };
        // Inputs.
        for (k, inp) in ALL_IN.iter().enumerate() {
            let mut v: Vec<usize> = default_names(*inp).iter().map(|n| a.slot(n)).collect();
            for (alias, target) in aliases {
                if default_names(*inp).contains(target) {
                    v.push(a.slot(alias));
                }
            }
            a.inputs[k] = v;
        }

        // Metrics.
        let mut cg = DVec3::ZERO;
        let mut arp = DVec3::ZERO;
        if let Some(m) = child(root, "metrics") {
            for c in m.children().filter(|c| c.is_element()) {
                let v: f64 = text(&c).parse().unwrap_or(0.0);
                match c.tag_name().name() {
                    "wingarea" => a.sw_m2 = v * unit_k(&c, &[("M2", 1.0), ("FT2", 0.092_903)], 0.092_903),
                    "wingspan" => a.bw_m = v * unit_k(&c, &[("M", 1.0), ("FT", FT)], FT),
                    "chord" => a.cbar_m = v * unit_k(&c, &[("M", 1.0), ("FT", FT)], FT),
                    "wing_incidence" => a.incidence = v * unit_k(&c, &[("DEG", std::f64::consts::PI / 180.0), ("RAD", 1.0)], std::f64::consts::PI / 180.0),
                    "location" if c.attribute("name") == Some("AERORP") => arp = loc(&c),
                    _ => {}
                }
                // Expose metrics as properties too.
                let pname = match c.tag_name().name() {
                    "wingarea" => Some(("metrics/Sw-sqft", a.sw_m2 / 0.092_903)),
                    "wingspan" => Some(("metrics/bw-ft", a.bw_m / FT)),
                    "chord" => Some(("metrics/cbarw-ft", a.cbar_m / FT)),
                    "htailarea" => Some(("metrics/Sh-sqft", v * unit_k(&c, &[("M2", 1.0 / 0.092_903)], 1.0))),
                    "htailarm" => Some(("metrics/lh-ft", v * unit_k(&c, &[("M", 1.0 / FT)], 1.0))),
                    "vtailarea" => Some(("metrics/Sv-sqft", v * unit_k(&c, &[("M2", 1.0 / 0.092_903)], 1.0))),
                    "vtailarm" => Some(("metrics/lv-ft", v * unit_k(&c, &[("M", 1.0 / FT)], 1.0))),
                    _ => None,
                };
                if let Some((n, v)) = pname {
                    let s = a.slot(n);
                    a.consts.push((s, v));
                }
            }
        }
        if let Some(mb) = child(root, "mass_balance") {
            if let Some(l) = mb.children().find(|c| c.is_element() && c.tag_name().name() == "location" && c.attribute("name") == Some("CG")) {
                cg = loc(&l);
            }
        }
        // The importer's loaded CG (empty + payload + fuel), when present.
        if let Some(l) = child(root, "skyward_cg") {
            cg = loc(&l);
        }
        let d = arp - cg;
        a.arp = Vec3::new(-d.y as f32, d.z as f32, -d.x as f32);
        // Geometry properties some FDMs read (CG-dependent moment corrections).
        for (n, v) in [
            ("metrics/aero-rp-x-in", arp.x / 0.0254),
            ("metrics/aero-rp-y-in", arp.y / 0.0254),
            ("metrics/aero-rp-z-in", arp.z / 0.0254),
            ("inertia/cg-x-in", cg.x / 0.0254),
            ("inertia/cg-y-in", cg.y / 0.0254),
            ("inertia/cg-z-in", cg.z / 0.0254),
        ] {
            let s = a.slot(n);
            a.consts.push((s, v));
        }
        for (n, v) in consts {
            let s = a.slot(n);
            a.consts.push((s, *v));
        }
        let Some(aero) = child(root, "aerodynamics") else { return Err("no <aerodynamics>".into()) };
        for c in aero.children().filter(|c| c.is_element()) {
            match c.tag_name().name() {
                "hysteresis_limits" => {
                    let k = unit_k(&c, &[("DEG", std::f64::consts::PI / 180.0), ("RAD", 1.0)], std::f64::consts::PI / 180.0);
                    let lo = child(c, "min").map(|x| text(&x).parse::<f64>().unwrap_or(0.0) * k).unwrap_or(0.0);
                    let hi = child(c, "max").map(|x| text(&x).parse::<f64>().unwrap_or(0.0) * k).unwrap_or(1.0);
                    a.alpha_hyst = Some((lo, hi));
                }
                "function" => {
                    let name = c.attribute("name").unwrap_or("").to_string();
                    if let Some(f) = a.func_body(c) {
                        let s = a.slot(&name);
                        a.pre.push((s, f));
                    }
                }
                "aero_ref_pt_shift_x" => {
                    if let Some(f) = c.children().find(|f| f.is_element() && f.tag_name().name() == "function") {
                        if let Some(body) = a.func_body(f) {
                            if let Some(n) = f.attribute("name") {
                                let s = a.slot(n);
                                a.pre.push((s, body.clone()));
                            }
                            a.arp_shift = Some(body);
                        }
                    }
                }
                "axis" => {
                    let ax = match c.attribute("name").unwrap_or("").to_ascii_uppercase().as_str() {
                        "DRAG" => Axis::Drag,
                        "SIDE" => Axis::Side,
                        "LIFT" => Axis::Lift,
                        "ROLL" => Axis::Roll,
                        "PITCH" => Axis::Pitch,
                        "YAW" => Axis::Yaw,
                        "X" => Axis::X,
                        "Y" => Axis::Y,
                        "Z" => Axis::Z,
                        "AXIAL" => Axis::Axial,
                        "NORMAL" => Axis::Normal,
                        _ => continue,
                    };
                    let mut fs = Vec::new();
                    for f in c.children().filter(|f| f.is_element() && f.tag_name().name() == "function") {
                        let name = f.attribute("name").map(|n| n.to_string());
                        if let Some(body) = a.func_body(f) {
                            let s = name.map(|n| a.slot(&n));
                            fs.push((s, body));
                        }
                    }
                    a.axes.push((ax, fs));
                }
                _ => {}
            }
        }
        a.stall_slot = Some(a.slot("aero/stall-hyst-norm"));
        a.cl2_slot = a.slot_of("aero/cl-squared");
        // Report properties nothing writes.
        let written: std::collections::HashSet<usize> = a
            .inputs
            .iter()
            .flatten()
            .copied()
            .chain(a.consts.iter().map(|c| c.0))
            .chain(a.pre.iter().map(|p| p.0))
            .chain(a.axes.iter().flat_map(|(_, fs)| fs.iter().filter_map(|f| f.0)))
            .chain(a.stall_slot.into_iter().chain(a.cl2_slot))
            .collect();
        let mut unresolved: Vec<String> = a.names.iter().filter(|(_, s)| !written.contains(s)).map(|(n, _)| n.clone()).collect();
        unresolved.sort();
        a.unresolved = unresolved;
        Ok(a)
    }

    fn func_body(&mut self, f: roxmltree::Node) -> Option<F> {
        let first = f.children().find(|c| c.is_element() && c.tag_name().name() != "description")?;
        Some(self.node(first))
    }

    fn node(&mut self, n: roxmltree::Node) -> F {
        let tag = n.tag_name().name();
        let kids: Vec<roxmltree::Node> = n.children().filter(|c| c.is_element() && c.tag_name().name() != "description").collect();
        let op = |o: Op, me: &mut Self| F::Op(o, kids.iter().map(|k| me.node(*k)).collect());
        match tag {
            "value" | "v" => F::V(text(&n).parse().unwrap_or(0.0)),
            "property" | "p" => {
                let t = text(&n);
                let (neg, name) = match t.strip_prefix('-') {
                    Some(r) => (true, r.to_string()),
                    None => (false, t),
                };
                F::P(self.slot(&name), neg)
            }
            "table" => F::T(Box::new(self.table(n))),
            "product" => op(Op::Product, self),
            "sum" => op(Op::Sum, self),
            "difference" => op(Op::Difference, self),
            "quotient" => op(Op::Quotient, self),
            "pow" => op(Op::Pow, self),
            "sqrt" => op(Op::Sqrt, self),
            "abs" => op(Op::Abs, self),
            "sin" => op(Op::Sin, self),
            "cos" => op(Op::Cos, self),
            "tan" => op(Op::Tan, self),
            "asin" => op(Op::Asin, self),
            "acos" => op(Op::Acos, self),
            "atan" => op(Op::Atan, self),
            "atan2" => op(Op::Atan2, self),
            "exp" => op(Op::Exp, self),
            "ln" => op(Op::Ln, self),
            "log2" => op(Op::Log2, self),
            "log10" => op(Op::Log10, self),
            "min" => op(Op::Min, self),
            "max" => op(Op::Max, self),
            "avg" => op(Op::Avg, self),
            "fraction" => op(Op::Fraction, self),
            "integer" => op(Op::Integer, self),
            "mod" => op(Op::Mod, self),
            "toradians" => op(Op::ToRadians, self),
            "todegrees" => op(Op::ToDegrees, self),
            "lt" => op(Op::Lt, self),
            "le" => op(Op::Le, self),
            "gt" => op(Op::Gt, self),
            "ge" => op(Op::Ge, self),
            "eq" => op(Op::Eq, self),
            "nq" => op(Op::Nq, self),
            "and" => op(Op::And, self),
            "or" => op(Op::Or, self),
            "not" => op(Op::Not, self),
            "ifthen" => op(Op::IfThen, self),
            "sign" => op(Op::Sign, self),
            "floor" => op(Op::Floor, self),
            "ceil" => op(Op::Ceil, self),
            "pi" => F::Op(Op::Pi, vec![]),
            // random / urandom: deterministic 0.
            _ => F::Op(Op::Zero, vec![]),
        }
    }

    fn table(&mut self, n: roxmltree::Node) -> Table {
        let mut t = Table { row: None, col: None, tab: None, rows: Vec::new(), cols: Vec::new(), data: Vec::new(), pages: Vec::new() };
        let mut ivars = Vec::new();
        for c in n.children().filter(|c| c.is_element()) {
            if c.tag_name().name() == "independentVar" {
                let lookup = c.attribute("lookup").unwrap_or("row").to_string();
                let s = self.slot(&text(&c));
                ivars.push((lookup, s));
            }
        }
        for (l, s) in &ivars {
            match l.as_str() {
                "column" => t.col = Some(*s),
                "table" => t.tab = Some(*s),
                _ => t.row = Some(*s),
            }
        }
        let datas: Vec<roxmltree::Node> = n.children().filter(|c| c.is_element() && c.tag_name().name() == "tableData").collect();
        if datas.len() > 1 || datas.first().map(|d| d.attribute("breakPoint").is_some()).unwrap_or(false) {
            for d in datas {
                let k: f64 = d.attribute("breakPoint").and_then(|b| b.trim().parse().ok()).unwrap_or(0.0);
                let mut sub = Table { row: t.row, col: t.col, tab: None, rows: Vec::new(), cols: Vec::new(), data: Vec::new(), pages: Vec::new() };
                fill(&mut sub, d.text().unwrap_or(""), t.col.is_some());
                t.pages.push((k, sub));
            }
        } else if let Some(d) = datas.first() {
            let two_d = t.col.is_some();
            fill(&mut t, d.text().unwrap_or(""), two_d);
        }
        t
    }

    /// Set the input properties.
    fn set_inputs(&self, p: &mut [f64], i: &AeroInput, st: &JsbState) {
        let v = i.vb.as_dvec3();
        let vt = v.length();
        let vs2 = v.y * v.y + v.z * v.z;
        let alpha = if vs2 > 1e-8 { (-v.y).atan2(v.z) } else { 0.0 };
        let beta = if vt > 1e-3 { (-v.x / vt).clamp(-1.0, 1.0).asin() } else { 0.0 };
        let rho = i.rho as f64;
        let qbar = 0.5 * rho * vt * vt / 47.880_26; // Pa → psf
        let vt_ft = vt / FT;
        let half = |l: f64| if vt_ft > 1.0 { (l / FT) / (2.0 * vt_ft) } else { 0.0 };
        let bw_ft = self.bw_m / FT;
        let rho_sl = rho / 515.379; // kg/m³ → slug/ft³
        let re = rho * vt * self.cbar_m / 1.789e-5;
        let vals: [f64; N_IN] = {
            use In::*;
            let mut x = [0.0; N_IN];
            let mut set = |k: In, v: f64| x[k as usize] = v;
            set(Qbar, qbar);
            set(Alpha, alpha);
            set(AlphaDeg, alpha.to_degrees());
            set(Beta, beta);
            set(BetaDeg, beta.to_degrees());
            set(AlphaDot, i.alpha_dot as f64);
            set(BetaDot, 0.0);
            set(Bi2Vel, half(self.bw_m));
            set(Ci2Vel, half(self.cbar_m));
            set(P, i.p as f64);
            set(Q, i.q as f64);
            set(R, i.r as f64);
            set(Mach, i.mach as f64);
            set(Vt, vt_ft);
            set(Elevator, i.elevator as f64);
            set(ElevatorDeg, (i.elevator as f64).to_degrees());
            set(ElevatorNorm, i.elevator_norm as f64);
            set(AileronL, i.aileron_l as f64);
            set(AileronR, i.aileron_r as f64);
            set(AileronLDeg, (i.aileron_l as f64).to_degrees());
            set(AileronRDeg, (i.aileron_r as f64).to_degrees());
            set(AileronNorm, i.aileron_norm as f64);
            set(Rudder, i.rudder as f64);
            set(RudderDeg, (i.rudder as f64).to_degrees());
            set(RudderNorm, i.rudder_norm as f64);
            set(FlapDeg, i.flap_deg as f64);
            set(FlapNorm, i.flap_norm as f64);
            set(SlatNorm, i.slat_norm as f64);
            set(SlatDeg, i.slat_deg as f64);
            set(Speedbrake, i.speedbrake as f64);
            set(Spoiler, i.spoiler as f64);
            set(SpoilerDeg, i.spoiler_deg as f64);
            set(Gear, i.gear as f64);
            set(HbMac, if bw_ft > 0.0 { (i.h_mac as f64 / FT) / bw_ft } else { 10.0 });
            set(HbCg, if bw_ft > 0.0 { (i.h_cg as f64 / FT) / bw_ft } else { 10.0 });
            set(Rho, rho_sl);
            set(PropWash, i.prop_wash as f64 / FT);
            set(Re, re);
            set(StabDeg, i.stab_deg as f64);
            set(StabNorm, i.stab_norm as f64);
            set(LefNorm, i.lef_norm as f64);
            set(LefRNorm, 1.0 - i.lef_norm as f64);
            x
        };
        for (k, slots) in self.inputs.iter().enumerate() {
            for &s in slots {
                p[s] = vals[k];
            }
        }
        // alpha-wing includes the wing incidence.
        if let Some(s) = self.slot_of("aero/alpha-wing-rad") {
            p[s] = alpha + self.incidence;
        }
        if let Some(s) = self.slot_of("aero/mag-beta-rad") {
            p[s] = beta.abs();
        }
        if let Some(s) = self.slot_of("aero/mag-beta-deg") {
            p[s] = beta.abs().to_degrees();
        }
        if let Some(s) = self.slot_of("fcs/mag-elevator-pos-rad") {
            p[s] = (i.elevator as f64).abs();
        }
        if let Some(s) = self.slot_of("aero/qbar-area") {
            p[s] = qbar * self.sw_m2 / 0.092_903;
        }
        if let Some(s) = self.slot_of("velocities/u-aero-fps") {
            p[s] = v.z / FT;
        }
        for &(s, v) in &self.consts {
            p[s] = v;
        }
        if let Some(s) = self.cl2_slot {
            p[s] = (st.last_cl as f64) * (st.last_cl as f64);
        }
    }

    /// Evaluate forces and moments; returns the updated state as well
    /// (callers keep it only for the real step, not for probes).
    pub fn eval(&self, i: &AeroInput, st: &JsbState) -> (AeroOutput, JsbState) {
        SCRATCH.with(|sc| {
            let mut p = sc.borrow_mut();
            if p.len() < self.n_slots {
                p.resize(self.n_slots, 0.0);
            }
            // The scratch is shared by every model on this thread: properties
            // nothing writes must read 0, not another aircraft's values.
            p[..self.n_slots].fill(0.0);
            self.eval_with(i, st, &mut p)
        })
    }

    /// Steady-state evaluation (no history): iterates the properties that
    /// depend on the previous step (CL² for induced drag). For trim / fits.
    pub fn eval_steady(&self, i: &AeroInput) -> AeroOutput {
        let (o, s1) = self.eval(i, &JsbState::default());
        if self.cl2_slot.is_none() {
            return o;
        }
        let (o, s2) = self.eval(i, &JsbState { stalled: false, last_cl: s1.last_cl });
        self.eval(i, &JsbState { stalled: false, last_cl: 0.5 * (s1.last_cl + s2.last_cl) }).0.with_cl(o.cl)
    }

    fn eval_with(&self, i: &AeroInput, st: &JsbState, p: &mut [f64]) -> (AeroOutput, JsbState) {
        self.set_inputs(p, i, st);
        let v = i.vb.as_dvec3();
        let vt = v.length();
        let vs2 = v.y * v.y + v.z * v.z;
        let alpha = if vs2 > 1e-8 { (-v.y).atan2(v.z) } else { 0.0 };
        let beta = if vt > 1e-3 { (-v.x / vt).clamp(-1.0, 1.0).asin() } else { 0.0 };
        // Stall hysteresis (JSBSim FGAerodynamics).
        let mut hyst = st.stalled;
        if let Some((lo, hi)) = self.alpha_hyst {
            if alpha > hi {
                hyst = true;
            } else if alpha < lo {
                hyst = false;
            }
        }
        if let Some(s) = self.stall_slot {
            p[s] = if hyst { 1.0 } else { 0.0 };
        }
        for (s, f) in &self.pre {
            p[*s] = eval(f, p);
        }
        let mut sums = [0.0f64; 11];
        for (ax, fs) in &self.axes {
            let mut tot = 0.0;
            for (s, f) in fs {
                let x = eval(f, p);
                if let Some(s) = s {
                    p[*s] = x;
                }
                tot += x;
            }
            sums[*ax as usize] += tot;
        }
        let g = |a: Axis| sums[a as usize];
        // JSBSim body axes: x fwd, y right, z down. Wind → body.
        let (sa, ca, sb, cb) = (alpha.sin(), alpha.cos(), beta.sin(), beta.cos());
        let (d, s, l) = (g(Axis::Drag), g(Axis::Side), g(Axis::Lift));
        let fw = DVec3::new(-d, s, -l);
        let mut fb = DVec3::new(
            ca * cb * fw.x - ca * sb * fw.y - sa * fw.z,
            sb * fw.x + cb * fw.y,
            sa * cb * fw.x - sa * sb * fw.y + ca * fw.z,
        );
        fb += DVec3::new(g(Axis::X) - g(Axis::Axial), g(Axis::Y), g(Axis::Z) - g(Axis::Normal));
        let mb = DVec3::new(g(Axis::Roll), g(Axis::Pitch), g(Axis::Yaw));
        // lbf / lbf·ft → SI, JSBSim body → Skyward body (x left = -y, y up = -z, z fwd = x).
        let force = Vec3::new((-fb.y * LBF) as f32, (-fb.z * LBF) as f32, (fb.x * LBF) as f32);
        let k = LBF * FT;
        let mut moment = Vec3::new((-mb.y * k) as f32, (-mb.z * k) as f32, (mb.x * k) as f32);
        // JSBSim FGAerodynamics: the reference point shifts aft by
        // shift × chord; a CG moved aft (fuel transfer, `i.cg_aft_m`) brings
        // it closer.
        let shift = self.arp_shift.as_ref().map(|f| eval(f, p) * self.cbar_m).unwrap_or(0.0) as f32 - i.cg_aft_m;
        moment += (self.arp - Vec3::Z * shift).cross(force);
        // Coefficients from the total force in wind axes (works for any axis system).
        let qs_n = p[self.inputs[In::Qbar as usize][0]] * self.sw_m2 / 0.092_903 * LBF;
        let vbf = i.vb.as_dvec3();
        let (cl, cd, cy) = if qs_n > 1e-6 && vt > 1e-3 {
            let f = force.as_dvec3();
            let vs = (vbf.y * vbf.y + vbf.z * vbf.z).sqrt().max(1e-9);
            let lift_dir = DVec3::new(0.0, vbf.z, -vbf.y) / vs;
            let drag = -f.dot(vbf / vt);
            (f.dot(lift_dir) / qs_n, drag / qs_n, -f.x / qs_n)
        } else {
            (0.0, 0.0, 0.0)
        };
        let st2 = JsbState { stalled: hyst, last_cl: if qs_n > 1e-3 { cl as f32 } else { 0.0 } };
        (AeroOutput { force, moment, alpha: alpha as f32, beta: beta as f32, cl: cl as f32, cd: cd as f32, cy: cy as f32 }, st2)
    }

    /// Value of a named property after the last `eval` (diagnostics).
    #[allow(dead_code)]
    pub fn prop(&self, p: &[f64], name: &str) -> Option<f64> {
        self.slot_of(name).and_then(|s| p.get(s).copied())
    }
}

impl AeroOutput {
    fn with_cl(self, _cl: f32) -> AeroOutput {
        self
    }
}

fn child<'a, 'i>(n: roxmltree::Node<'a, 'i>, name: &str) -> Option<roxmltree::Node<'a, 'i>> {
    n.children().find(|c| c.is_element() && c.tag_name().name() == name)
}

fn loc(n: &roxmltree::Node) -> DVec3 {
    let k = unit_k(n, &[("M", 1.0), ("FT", FT), ("IN", 0.0254)], 0.0254);
    let g = |c: &str| n.children().find(|x| x.is_element() && x.tag_name().name() == c).map(|x| text(&x).parse::<f64>().unwrap_or(0.0)).unwrap_or(0.0);
    DVec3::new(g("x"), g("y"), g("z")) * k
}

fn fill(t: &mut Table, s: &str, two_d: bool) {
    let lines: Vec<Vec<f64>> = s
        .lines()
        .map(|l| l.split_whitespace().filter_map(|x| x.parse::<f64>().ok()).collect::<Vec<_>>())
        .filter(|v| !v.is_empty())
        .collect();
    if lines.is_empty() {
        t.rows = vec![0.0];
        t.data = vec![0.0];
        return;
    }
    if two_d {
        t.cols = lines[0].clone();
        for l in &lines[1..] {
            t.rows.push(l[0]);
            for c in 0..t.cols.len() {
                t.data.push(*l.get(c + 1).unwrap_or(&0.0));
            }
        }
    } else {
        for l in &lines {
            t.rows.push(l[0]);
            t.data.push(*l.get(1).unwrap_or(&0.0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cargo test jsb_polar -- --ignored --nocapture`: coefficient sweep per aircraft.
    #[test]
    #[ignore]
    fn jsb_polar() {
        for id in ["kestrel", "swift", "lancer", "meridian190", "meridian320"] {
            let xml = std::fs::read_to_string(format!("{}/assets/aircraft/{id}/fdm.xml", env!("CARGO_MANIFEST_DIR"))).unwrap();
            let a = JsbAero::parse(&xml, &[], &[]).unwrap();
            let mut st = JsbState::default();
            println!("== {id}: S {:.1} m² b {:.2} m c {:.2} m arp {:?}; {} slots", a.sw_m2, a.bw_m, a.cbar_m, a.arp, a.n_slots);
            println!("   unresolved: {}", a.unresolved.join(" "));
            for adeg in [-4.0f32, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0] {
                let al = adeg.to_radians();
                let v = 60.0;
                let inp = AeroInput { vb: Vec3::new(0.0, -v * al.sin(), v * al.cos()), rho: 1.225, mach: v / 340.0, gear: 1.0, h_mac: 100.0, h_cg: 100.0, ..Default::default() };
                let (o, s2) = a.eval(&inp, &st);
                st = s2;
                let qsc = 0.5 * 1.225 * v * v * a.sw_m2 as f32 * a.cbar_m as f32;
                println!("   a {adeg:5.1}: CL {:6.3} CD {:6.4} Cm {:7.4}", o.cl, o.cd, -o.moment.x / qsc);
            }
        }
    }
}
