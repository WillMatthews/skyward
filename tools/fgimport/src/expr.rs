//! Expressions over Skyward animation channels (the runtime evaluates the
//! same JSON form, see `src/aircraft_asset.rs`), FlightGear property →
//! channel mapping, and conversion of FG `<condition>` / `<expression>` /
//! factor-offset-interpolation animation values.

use crate::xml::Elem;
use regex::Regex;
use serde_json::{json, Value};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};

#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    C(f32),
    Ch(String),
    Op(&'static str, Vec<Expr>),
    Interp(Box<Expr>, Vec<(f32, f32)>),
}

impl Expr {
    pub fn is_const(&self) -> Option<f32> {
        match self {
            Expr::C(v) => Some(*v),
            _ => None,
        }
    }
    pub fn to_json(&self) -> Value {
        match self {
            Expr::C(v) => json!(if v.is_finite() { *v } else if *v > 0.0 { 1e30 } else { -1e30 }),
            Expr::Ch(c) => json!({ "ch": c }),
            Expr::Op(o, a) => json!({ "op": o, "a": a.iter().map(|e| e.to_json()).collect::<Vec<_>>() }),
            Expr::Interp(x, t) => json!({ "op": "interp", "a": [x.to_json()], "t": t.iter().map(|(a, b)| json!([a, b])).collect::<Vec<_>>() }),
        }
    }
    pub fn channels(&self, out: &mut Vec<String>) {
        match self {
            Expr::Ch(c) => {
                if !out.contains(c) {
                    out.push(c.clone())
                }
            }
            Expr::Op(_, a) => a.iter().for_each(|e| e.channels(out)),
            Expr::Interp(x, _) => x.channels(out),
            _ => {}
        }
    }
}

/// A string property / condition value as a number (FNV-1a hash, never 0):
/// static string properties (config `props`) compare equal to the same
/// string in a `<condition><equals>`.
pub fn str_value(s: &str) -> f32 {
    let mut h: u32 = 0x811C_9DC5;
    for b in s.bytes() {
        h = (h ^ b as u32).wrapping_mul(0x0100_0193);
    }
    (h % 1_000_003 + 1) as f32
}

pub fn interp(t: &[(f32, f32)], x: f32) -> f32 {
    if t.is_empty() {
        return 0.0;
    }
    if x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        if x <= w[1].0 {
            let d = w[1].0 - w[0].0;
            let f = if d.abs() < 1e-12 { 1.0 } else { (x - w[0].0) / d };
            return w[0].1 + f * (w[1].1 - w[0].1);
        }
    }
    t[t.len() - 1].1
}

pub fn apply(op: &str, a: &[f32]) -> f32 {
    let g = |i: usize| a.get(i).copied().unwrap_or(0.0);
    let b = |x: bool| if x { 1.0 } else { 0.0 };
    match op {
        "add" => a.iter().sum(),
        "mul" => a.iter().product(),
        "sub" => g(0) - a.iter().skip(1).sum::<f32>(),
        "div" => {
            let d = g(1);
            if d.abs() < 1e-12 { 0.0 } else { g(0) / d }
        }
        "min" => a.iter().copied().fold(f32::INFINITY, f32::min),
        "max" => a.iter().copied().fold(f32::NEG_INFINITY, f32::max),
        "clamp" => g(0).max(g(1)).min(g(2)),
        "abs" => g(0).abs(),
        "neg" => -g(0),
        "not" => b(g(0) == 0.0),
        "and" => b(a.iter().all(|x| *x != 0.0)),
        "or" => b(a.iter().any(|x| *x != 0.0)),
        "lt" => b(g(0) < g(1)),
        "le" => b(g(0) <= g(1)),
        "gt" => b(g(0) > g(1)),
        "ge" => b(g(0) >= g(1)),
        "eq" => b((g(0) - g(1)).abs() < 1e-6),
        "ne" => b((g(0) - g(1)).abs() >= 1e-6),
        "sin" => g(0).sin(),
        "cos" => g(0).cos(),
        "floor" => g(0).floor(),
        "mod" => {
            let d = g(1);
            if d.abs() < 1e-12 { 0.0 } else { g(0).rem_euclid(d) }
        }
        "pow" => g(0).powf(g(1)),
        "sqrt" => g(0).max(0.0).sqrt(),
        "step" => step_scroll(g(0), g(1), g(2)),
        _ => 0.0,
    }
}

/// FlightGear `SGStepExpression`: quantise to `step`, and with `scroll` > 0
/// roll smoothly into the next step over its last `scroll` units (drum counters).
pub fn step_scroll(v: f32, step: f32, scroll: f32) -> f32 {
    if step <= 0.0 {
        return v;
    }
    let mut m = (v / step).floor() * step;
    if scroll > 0.0 {
        let rem = step - v.abs() % step;
        if rem < scroll {
            m += (scroll - rem) / scroll * step;
        }
    }
    m
}

/// Build an op node, folding constants.
pub fn op(o: &'static str, a: Vec<Expr>) -> Expr {
    if a.iter().all(|e| e.is_const().is_some()) {
        let v: Vec<f32> = a.iter().map(|e| e.is_const().unwrap()).collect();
        return Expr::C(apply(o, &v));
    }
    // Light algebraic simplification.
    match o {
        "mul" => {
            let k: f32 = a.iter().filter_map(|e| e.is_const()).product();
            if k == 0.0 {
                return Expr::C(0.0);
            }
            let mut rest: Vec<Expr> = a.into_iter().filter(|e| e.is_const().is_none()).collect();
            if (k - 1.0).abs() > 1e-9 {
                rest.push(Expr::C(k));
            }
            if rest.len() == 1 {
                return rest.pop().unwrap();
            }
            Expr::Op("mul", rest)
        }
        "add" => {
            let k: f32 = a.iter().filter_map(|e| e.is_const()).sum();
            let mut rest: Vec<Expr> = a.into_iter().filter(|e| e.is_const().is_none()).collect();
            if k != 0.0 {
                rest.push(Expr::C(k));
            }
            if rest.len() == 1 {
                return rest.pop().unwrap();
            }
            Expr::Op("add", rest)
        }
        _ => Expr::Op(o, a),
    }
}

/// FlightGear property → channel mapping.
pub struct PropMap {
    pub statics: HashMap<String, f32>,
    rules: Vec<(Regex, String, f32, f32)>,
    pub unmapped: RefCell<BTreeMap<String, usize>>,
    pub used: RefCell<BTreeMap<String, usize>>,
}

/// Built-in mapping: (regex, channel template with {1}=index, scale, offset).
/// Channel semantics follow FlightGear's (see `src/aircraft_asset.rs`).
const DEFAULT_RULES: &[(&str, &str, f32, f32)] = &[
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/elevator-pos-norm$", "elevator", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/left-aileron-pos-norm$", "aileron_l", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/right-aileron-pos-norm$", "aileron_r", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/aileron-pos-norm$", "aileron_l", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/rudder-pos-norm$", "rudder", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/flap-pos-norm$", "flaps", 1.0, 0.0),
    (r"^(?:fdm/jsbsim/)?(?:surface-positions|fcs)/(?:left-|right-)?(?:speedbrake|spoiler)s?-pos-norm$", "spoilers", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/position-norm$", "gear_pos", 1.0, 0.0),
    (r"^fdm/jsbsim/gear/gear-pos-norm$", "gear_pos", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/compression-norm$", "gear{1}_comp_norm", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/compression-m$", "gear{1}_comp_m", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/compression-ft$", "gear{1}_comp_m", 3.28084, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/rollspeed-ms$", "gear{1}_roll_ms", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/wow$", "wow", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/steering-norm$", "steer_norm", 1.0, 0.0),
    (r"^gear/gear(?:\[(\d)\])?/caster-angle-deg$", "steer_deg", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/rpm$", "eng{1}_rpm", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/n1$", "eng{1}_n1", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/n2$", "eng{1}_n2", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/running$", "eng{1}_running", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/reverser-pos-norm$", "eng{1}_reverser", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/augmentation$", "afterburner", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/thruster/rpm$", "prop{1}_rpm", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/prop-rpm$", "prop{1}_rpm", 1.0, 0.0),
    (r"^controls/flight/elevator$", "ctl_elevator", 1.0, 0.0),
    (r"^controls/flight/aileron$", "ctl_aileron", 1.0, 0.0),
    (r"^controls/flight/rudder$", "ctl_rudder", 1.0, 0.0),
    (r"^controls/flight/flaps$", "ctl_flaps", 1.0, 0.0),
    (r"^controls/flight/(?:speedbrake|spoilers)$", "ctl_spoilers", 1.0, 0.0),
    (r"^controls/flight/elevator-trim$", "ctl_trim", 1.0, 0.0),
    (r"^controls/engines/engine(?:\[(\d)\])?/throttle$", "throttle", 1.0, 0.0),
    (r"^controls/gear/gear-down$", "ctl_gear", 1.0, 0.0),
    (r"^controls/gear/brake-(?:left|right|parking)$", "brakes", 1.0, 0.0),
    (r"^controls/lighting/nav-lights?(?:-switch)?$", "nav", 1.0, 0.0),
    (r"^controls/lighting/beacon(?:-switch)?$", "beacon_sw", 1.0, 0.0),
    (r"^controls/lighting/strobe(?:-switch)?s?$", "strobe_sw", 1.0, 0.0),
    (r"^controls/lighting/landing-lights?(?:\[\d\])?$", "landing_light", 1.0, 0.0),
    (r"^controls/lighting/taxi-lights?$", "taxi_light", 1.0, 0.0),
    (r"^sim/time/elapsed-sec$", "time", 1.0, 0.0),
    (r"^instrumentation/airspeed-indicator/indicated-speed-kt$", "ias_kt", 1.0, 0.0),
    (r"^velocities/airspeed-kt$", "ias_kt", 1.0, 0.0),
    (r"^instrumentation/altimeter/indicated-altitude-ft$", "alt_ft", 1.0, 0.0),
    (r"^position/altitude-ft$", "alt_ft", 1.0, 0.0),
    (r"^instrumentation/attitude-indicator/indicated-pitch-deg$", "pitch_deg", 1.0, 0.0),
    (r"^instrumentation/attitude-indicator/indicated-roll-deg$", "roll_deg", 1.0, 0.0),
    (r"^orientation/pitch-deg$", "pitch_deg", 1.0, 0.0),
    (r"^orientation/roll-deg$", "roll_deg", 1.0, 0.0),
    (r"^orientation/heading-(?:magnetic-)?deg$", "heading_deg", 1.0, 0.0),
    (r"^instrumentation/heading-indicator/indicated-heading-deg$", "heading_deg", 1.0, 0.0),
    (r"^instrumentation/magnetic-compass/indicated-heading-deg$", "heading_deg", 1.0, 0.0),
    (r"^instrumentation/vertical-speed-indicator/indicated-speed-fpm$", "vs_fpm", 1.0, 0.0),
    (r"^instrumentation/turn-indicator/indicated-turn-rate$", "turn_rate", 1.0, 0.0),
    (r"^instrumentation/slip-skid-ball/indicated-slip-skid$", "slip", 1.0, 0.0),
    (r"^velocities/mach$", "mach", 1.0, 0.0),
    (r"^orientation/alpha-deg$", "alpha_deg", 1.0, 0.0),
    // ---- cockpit instruments (src/cockpit/channels.rs) ----
    // Things that are always so in Skyward: powered buses, serviceable
    // instruments, selected tanks, display-unit brightness knobs.
    (r"/serviceable$|/operable$|^instrumentation/[^/]+/power-btn$", "const", 0.0, 1.0),
    (r"^consumables/fuel/tank(?:\[\d+\])?/selected$", "const", 0.0, 1.0),
    (r"^controls/lighting/(?:DU/.*|.*brightness.*|data-entry-display|.*-brt|stby-compass)$", "const", 0.0, 1.0),
    (r"^f16/avionics/(?:mfd-[lr]-(?:brt|con|gain)|hud-(?:brt|cont|sym)|power-.*|hmd-sym-int-knob)$", "const", 0.0, 1.0),
    (r"^systems/electrical/(?:light|relay)/", "const", 0.0, 0.0),
    (r"^fdm/jsbsim/elec/bus/light/", "const", 0.0, 0.0),
    (r"^systems/electrical/sources/.*(?:powered|gen)", "const", 0.0, 1.0),
    (r"^systems/electrical/(?:outputs/(?:panel-lights|cockpit-lights|instrument-lights?|main-panel-light|overhead-panel-light|pedestal-light|pushbutton-lights|dome-light|flood-lights?)(?:\[\d+\])?)$", "panel_light", 28.0, 0.0),
    (r"^systems/electrical/bus/ac", "const", 0.0, 115.0),
    (r"^systems/electrical/(?:outputs|bus)/", "const", 0.0, 28.0),
    (r"^systems/electrical/(?:volts|.*-volts?)$", "const", 0.0, 28.0),
    (r"^fdm/jsbsim/elec/(?:bus|sources/batt-bus)", "const", 0.0, 28.0),
    (r"^systems/electrical/amps$", "amps", 1.0, 0.0),
    (r"^fdm/jsbsim/systems/hydraulics/sys[ab]-psi$", "const", 0.0, 3000.0),
    (r"^systems/vacuum(?:\[\d\])?/suction-inhg$", "suction_inhg", 1.0, 0.0),
    // Panel / instrument lighting (night factor 0..1).
    (r"^controls/lighting/(?:panel-norm|instruments?-norm|instrument-lights?|panel-lights?|main-panel(?:-norm)?|overhead-panel(?:-norm)?|fcu-panel-norm|pedestal|console.*|flood.*|dome.*|lighting-panel/.*|instruments-light)$", "panel_light", 1.0, 0.0),
    (r"^sim/model/(?:material/instruments/.*factor.*|lightmap/(?:panel|flood|dome)/factor|instruments-light)$", "panel_light", 1.0, 0.0),
    // Air data.
    (r"^fdm/jsbsim/velocities/vias-kts$", "ias_kt", 1.0, 0.0),
    (r"^instrumentation/airspeed-indicator(?:\[\d\])?/indicated-(?:corr-)?speed-kt$", "ias_kt", 1.0, 0.0),
    (r"^instrumentation/airspeed-indicator(?:\[\d\])?/true-speed-kt$", "tas_kt", 1.0, 0.0),
    (r"^instrumentation/airspeed-indicator(?:\[\d\])?/indicated-mach$", "mach", 1.0, 0.0),
    (r"^velocities/(?:groundspeed-kt|ground-speed-kt)$", "gs_kt", 1.0, 0.0),
    (r"^instrumentation/(?:altimeter|gps)(?:\[\d\])?/indicated-altitude-ft$", "alt_ft", 1.0, 0.0),
    (r"^instrumentation/altimeter(?:\[\d\])?/indicated-altitude-m$", "alt_ft", 0.3048, 0.0),
    (r"^instrumentation/altimeter(?:\[\d\])?/pressure-alt-ft$", "palt_ft", 1.0, 0.0),
    (r"^instrumentation/altimeter(?:\[\d\])?/setting-inhg$", "alt_set_inhg", 1.0, 0.0),
    (r"^instrumentation/altimeter(?:\[\d\])?/setting-hpa$", "alt_set_hpa", 1.0, 0.0),
    (r"^instrumentation/vertical-speed-indicator(?:\[\d\])?/indicated-speed-fpm(?:-final)?$", "vs_fpm", 1.0, 0.0),
    (r"^instrumentation/vertical-speed-indicator(?:\[\d\])?/indicated-speed-mps$", "vs_fpm", 0.00508, 0.0),
    (r"^instrumentation/gps/indicated-vertical-speed$", "vs_fpm", 1.0, 0.0),
    (r"^velocities/vertical-speed-fps$", "vs_fpm", 1.0 / 60.0, 0.0),
    (r"^velocities/speed-down-fps$", "vs_fpm", -1.0 / 60.0, 0.0),
    (r"^f16/avionics/vvi-indicated-speed-fps$", "vs_fpm", 1.0 / 60.0, 0.0),
    (r"^autopilot/internal/vert-speed-fpm$", "vs_fpm", 1.0, 0.0),
    (r"^instrumentation/(?:variometer|ilec-sc7|vario[^/]*|sc7|iv-ac|winter[^/]*)(?:\[\d\])?/(?:te-reading-mps|indicated-speed-mps|reading-mps)$", "vario_mps", 1.0, 0.0),
    (r"^instrumentation/(?:variometer|vario[^/]*)(?:\[\d\])?/(?:te-reading-kts|indicated-speed-kts?)$", "vario_mps", 1.943_844, 0.0),
    (r"^instrumentation/attitude-indicator(?:\[\d\])?/indicated-pitch-deg$", "pitch_deg", 1.0, 0.0),
    (r"^instrumentation/attitude-indicator(?:\[\d\])?/indicated-roll-deg$", "roll_deg", 1.0, 0.0),
    (r"^instrumentation/attitude-indicator(?:\[\d\])?/horizon-offset-deg$", "const", 0.0, 0.0),
    (r"^instrumentation/heading-indicator(?:-fg|-dg)?(?:\[\d\])?/indicated-heading-deg$", "heading_deg", 1.0, 0.0),
    (r"^instrumentation/(?:gps/indicated-track-magnetic-deg|ddrmi/heading|heading-indicator/heading-deg)$", "heading_deg", 1.0, 0.0),
    (r"^instrumentation/magnetic-compass(?:\[\d\])?/indicated-heading-deg(?:-final)?$", "compass_deg", 1.0, 0.0),
    (r"^instrumentation/magnetic-compass(?:\[\d\])?/pitch-deg$", "pitch_deg", 1.0, 0.0),
    (r"^instrumentation/magnetic-compass(?:\[\d\])?/roll-deg$", "roll_deg", 1.0, 0.0),
    (r"^(?:autopilot/settings|instrumentation/heading-indicator(?:-fg)?)/heading-bug-deg$", "hdg_bug_deg", 1.0, 0.0),
    (r"^instrumentation/nav(?:\[\d\])?/radials/selected-deg$", "hdg_bug_deg", 1.0, 0.0),
    (r"^instrumentation/turn-indicator(?:\[\d\])?/indicated-turn-rate$", "turn_rate", 1.0, 0.0),
    (r"^instrumentation/slip-skid-ball(?:\[\d\])?/indicated-slip-skid$", "slip", 1.0, 0.0),
    (r"^accelerations/(?:pilot-g(?:damped)?|n-z-cg-fps_sec|pilot/z-accel-g)$", "g_load", 1.0, 0.0),
    (r"^instrumentation/g-meter(?:\[\d\])?/indicated-g$|^instrumentation/accelerometer/indicated-g$", "g_load", 1.0, 0.0),
    (r"^(?:accelerations/pilot-gmax|instrumentation/g-meter(?:\[\d\])?/max-g)$", "g_max", 1.0, 0.0),
    (r"^(?:accelerations/pilot-gmin|instrumentation/g-meter(?:\[\d\])?/min-g)$", "g_min", 1.0, 0.0),
    (r"^fdm/jsbsim/(?:aero/alpha-deg(?:-fixed)?|fcs/fly-by-wire/pitch/alpha-indicated)$", "alpha_deg", 1.0, 0.0),
    (r"^environment/temperature-degc$", "oat_c", 1.0, 0.0),
    (r"^environment/temperature-degf$", "oat_f", 1.0, 0.0),
    (r"^sim/alarms/stall-warning$|^instrumentation/stall-warning/stall$", "stall_warn", 1.0, 0.0),
    // Clocks.
    (r"^instrumentation/clock/indicated-sec$|^sim/time/local-day-seconds$|^sim/model/[^/]+/local-time-seconds$", "clock_s", 1.0, 0.0),
    (r"^instrumentation/clock/indicated-min$", "clock_min", 1.0, 0.0),
    (r"^instrumentation/clock/indicated-hour$", "clock_hour", 1.0, 0.0),
    (r"^instrumentation/clock/(?:elapsed-sec|et/elapsed-sec)$", "time", 1.0, 0.0),
    // Engines (index = engine).
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/(?:egt-degf|indicated-egt-degf)$", "eng{1}_egt_f", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/(?:egt-degc|egt_degc|itt-degc|itt_degc|tit-degc|ftit-degc)$", "eng{1}_egt_c", 1.0, 0.0),
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/egt-norm$", "eng{1}_egt_norm", 1.0, 0.0),
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/egt-bug-norm$", "const", 0.0, 0.72),
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/(?:indicated-)?oil-pressure-psi$", "eng{1}_oilp_psi", 1.0, 0.0),
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/(?:indicated-)?oil-(?:final-)?temperature-degf$|^engines/(?:engine(?:\[(\d)\])?|active-engine)/oil-temp-degf$", "eng{1}_oilt_f", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/oil-temperature-degc$", "eng{1}_oilt_c", 1.0, 0.0),
    (r"^engines/(?:engine(?:\[(\d)\])?|active-engine)/cht-degf$", "eng{1}_cht_f", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/(?:mp-osi|mp-inhg|map-inhg)$", "eng{1}_map_inhg", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/boost-gauge-inhg$", "eng{1}_map_inhg", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/fuel-flow-gph$", "eng{1}_ff_gph", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/fuel-flow[-_]pph$", "eng{1}_ff_pph", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/torque-ftlb$", "eng{1}_tq_ftlb", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/(?:torque-pct|torque-norm)$", "eng{1}_tq_pct", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/nozzle-pos-norm(?:-lag)?$", "eng{1}_nozzle", 1.0, 0.0),
    (r"^engines/engine(?:\[(\d)\])?/(?:propstrike|cranking|starter)$", "const", 0.0, 0.0),
    // Fuel.
    (r"^consumables/fuel/tank(?:\[(\d+)\])?/(?:indicated-)?level-gal_us$", "tank{1}_gal", 1.0, 0.0),
    (r"^consumables/fuel/tank(?:\[(\d+)\])?/(?:indicated-)?level-lbs$", "tank{1}_lbs", 1.0, 0.0),
    (r"^consumables/fuel/tank(?:\[(\d+)\])?/(?:indicated-)?level-kg$", "tank{1}_lbs", 0.453_592, 0.0),
    (r"^consumables/fuel/tank(?:\[(\d+)\])?/level-norm$", "tank{1}_norm", 1.0, 0.0),
    (r"^consumables/fuel/total-fuel-lbs-(\d+)$", "digit:fuel_lbs:{1}", 1.0, 0.0),
    (r"^consumables/fuel/total-fuel-lbs$", "fuel_lbs", 1.0, 0.0),
    (r"^consumables/fuel/total-fuel-kg$", "fuel_lbs", 0.453_592, 0.0),
    (r"^consumables/fuel/total-fuel-gals?$", "fuel_gal", 1.0, 0.0),
    // Autoflight.
    (r"^autopilot/settings/target-altitude-ft$|^it-autoflight/input/alt$", "ap_alt_ft", 1.0, 0.0),
    (r"^autopilot/settings/vertical-speed-fpm$|^it-autoflight/input/vs$", "ap_vs_fpm", 1.0, 0.0),
    (r"^autopilot/settings/target-speed-kt$|^it-autoflight/input/(?:spd-)?kts$", "ap_ias_kt", 1.0, 0.0),
    (r"^it-autoflight/input/hdg$", "hdg_bug_deg", 1.0, 0.0),
    (r"^autopilot/settings/(?:engaged|flight-director)$|^it-autoflight/output/(?:ap[12]|fd[12])$", "ap_on", 1.0, 0.0),
    (r"^it-autoflight/output/athr$", "ap_athr", 1.0, 0.0),
    (r"^autopilot/internal/flight-director-pitch-deg$", "fd_pitch_deg", 1.0, 0.0),
    (r"^autopilot/internal/flight-director-roll-deg$", "fd_roll_deg", 1.0, 0.0),
    (r"^autopilot/internal/fdm-heading-bug-error-deg$", "hdg_bug_err_deg", 1.0, 0.0),
    (r"^instrumentation/airspeed-indicator(?:\[\d\])?/(?:airspeed-limit-kt|vmo-kt)$", "vmo_kt", 1.0, 0.0),
    (r"^instrumentation/clock/(?:local-hour|lc2/indicated-hour)$", "clock_hour", 1.0, 0.0),
    (r"^instrumentation/clock/lc2/indicated-min$", "clock_min", 1.0, 0.0),
    (r"^instrumentation/hobbs-meter/digits(\d)$", "digit10:hobbs_tenths:{1}", 1.0, 0.0),
    (r"^systems/hydraulic/.*psi.*$", "const", 0.0, 3000.0),
    // Rotorcraft (Skyward src/rotor): rotor speeds, blade pitch / flapping, turboshaft gauges.
    (r"^rotors/main/rpm$", "rotor_rpm", 1.0, 0.0),
    (r"^rotors/tail/rpm$", "tail_rpm", 1.0, 0.0),
    (r"^rotors/main/blade(?:\[\d\])?/flap-deg$", "rotor_coning_deg", 1.0, 0.0),
    (r"^rotors/main/blade(?:\[\d\])?/incidence-deg$", "rotor_collective_deg", 1.0, 0.0),
    (r"^rotors/tail/blade(?:\[\d\])?/incidence-deg$", "tail_pitch_deg", 1.0, 0.0),
    (r"^rotors/main/(?:roll|yaw)-deg$|^rotors/main/blade(?:\[\d\])?/position-deg$|^rotors/tail/blade(?:\[\d\])?/position-deg$", "const", 0.0, 0.0),
    (r"^rotors/main/torque$", "heli_tq_pct", 60.0, 0.0),
    (r"^engines/engine(?:\[\d\])?/n2-rpm$", "heli_n2_rpm", 1.0, 0.0),
    (r"^position/gear-agl-m$", "agl_m", 1.0, 0.0),
    // Warnings.
    (r"^f16/avionics/caution/master$|^instrumentation/eicas/master/caution$|^instrumentation/annunciators/master-caution(?:/state)?$", "master_caution", 1.0, 0.0),
    (r"^instrumentation/eicas/master/warning$|^instrumentation/annunciators/master-warning(?:/state)?$", "master_warning", 1.0, 0.0),
];

impl PropMap {
    pub fn new(statics: HashMap<String, f32>, extra: &[(String, String, f32, f32)]) -> Self {
        let mut rules = Vec::new();
        // Aircraft-specific rules first (they win).
        for (re, ch, k, b) in extra {
            rules.push((Regex::new(re).expect("bad channel regex"), ch.clone(), *k, *b));
        }
        for (re, ch, k, b) in DEFAULT_RULES {
            rules.push((Regex::new(re).unwrap(), ch.to_string(), *k, *b));
        }
        PropMap { statics, rules, unmapped: RefCell::new(BTreeMap::new()), used: RefCell::new(BTreeMap::new()) }
    }

    pub fn prop(&self, raw: &str) -> Expr {
        let name = raw.trim().trim_start_matches('/');
        let name = name.replace("[0]", "");
        if let Some(v) = self.statics.get(&name) {
            return Expr::C(*v);
        }
        for (re, ch, k, b) in &self.rules {
            if let Some(c) = re.captures(&name) {
                let idx = c.iter().skip(1).flatten().next().map(|m| m.as_str()).unwrap_or("0");
                if ch == "const" {
                    return Expr::C(*b);
                }
                let digit = ch.strip_prefix("digit:").map(|r| (r, false)).or_else(|| ch.strip_prefix("digit10:").map(|r| (r, true)));
                if let Some((rest, pow10)) = digit {
                    // digit:<channel>:{1} → the decimal digit of <channel> worth {1}
                    // (digit10: worth 10^{1}).
                    let mut it = rest.split(':');
                    let chn = it.next().unwrap_or("").to_string();
                    let n: f32 = it.next().unwrap_or("1").replace("{1}", idx).parse().unwrap_or(1.0);
                    let place = if pow10 { 10f32.powf(n) } else { n };
                    *self.used.borrow_mut().entry(chn.clone()).or_default() += 1;
                    let v = op("floor", vec![op("div", vec![Expr::Ch(chn), Expr::C(place.max(1.0))])]);
                    return op("mod", vec![v, Expr::C(10.0)]);
                }
                let chn = ch.replace("{1}", idx);
                *self.used.borrow_mut().entry(chn.clone()).or_default() += 1;
                return op("add", vec![op("mul", vec![Expr::Ch(chn), Expr::C(*k)]), Expr::C(*b)]);
            }
        }
        *self.unmapped.borrow_mut().entry(name.to_string()).or_default() += 1;
        Expr::C(0.0)
    }

    fn value_of(&self, e: &Elem) -> Expr {
        // <property>x</property> | <value>v</value> | nested expression
        match e.name.as_str() {
            "property" | "prop" | "p" => self.prop(&e.text),
            "value" | "v" => Expr::C(e.text.trim().parse().unwrap_or_else(|_| str_value(e.text.trim()))),
            _ => self.sgexpr(e),
        }
    }

    /// FG `<condition>` → 0/1 expression (children are and-ed).
    pub fn condition(&self, e: &Elem) -> Expr {
        let parts: Vec<Expr> = e.kids.iter().map(|k| self.cond_node(k)).collect();
        if parts.is_empty() {
            return Expr::C(1.0);
        }
        op("and", parts)
    }

    fn cond_node(&self, e: &Elem) -> Expr {
        let operands = |e: &Elem| -> Vec<Expr> { e.kids.iter().map(|k| self.value_of(k)).collect() };
        match e.name.as_str() {
            "and" => op("and", e.kids.iter().map(|k| self.cond_node(k)).collect()),
            "or" => op("or", e.kids.iter().map(|k| self.cond_node(k)).collect()),
            "not" => op("not", vec![op("and", e.kids.iter().map(|k| self.cond_node(k)).collect())]),
            "property" => op("ne", vec![self.prop(&e.text), Expr::C(0.0)]),
            "equals" => op("eq", operands(e)),
            "not-equals" => op("ne", operands(e)),
            "less-than" => op("lt", operands(e)),
            "less-than-equals" => op("le", operands(e)),
            "greater-than" => op("gt", operands(e)),
            "greater-than-equals" => op("ge", operands(e)),
            // (String values — e.g. a model variant "WHEELS" — compare by hash, see `str_value`.)
            "value" => Expr::C(e.text.trim().parse().unwrap_or_else(|_| str_value(e.text.trim()))),
            _ => Expr::C(1.0),
        }
    }

    /// SGExpression subset.
    pub fn sgexpr(&self, e: &Elem) -> Expr {
        let args = |e: &Elem| -> Vec<Expr> { e.kids.iter().map(|k| self.sgexpr(k)).collect() };
        match e.name.as_str() {
            "expression" => e.kids.first().map(|k| self.sgexpr(k)).unwrap_or(Expr::C(0.0)),
            "property" | "prop" | "p" => self.prop(&e.text),
            "value" | "v" => Expr::C(e.text.trim().parse().unwrap_or(0.0)),
            "sum" => op("add", args(e)),
            "product" => op("mul", args(e)),
            "difference" | "dif" => op("sub", args(e)),
            "div" | "quotient" => op("div", args(e)),
            "abs" | "fabs" => op("abs", args(e)),
            "neg" => op("neg", args(e)),
            "max" => op("max", args(e)),
            "min" => op("min", args(e)),
            "mod" => op("mod", args(e)),
            "pow" => op("pow", args(e)),
            "sqr" => {
                let a = args(e);
                op("mul", vec![a[0].clone(), a[0].clone()])
            }
            "sqrt" => op("sqrt", args(e)),
            "floor" => op("floor", args(e)),
            "sin" => op("sin", args(e)),
            "cos" => op("cos", args(e)),
            "deg2rad" => op("mul", vec![op("add", args(e)), Expr::C(std::f32::consts::PI / 180.0)]),
            "rad2deg" => op("mul", vec![op("add", args(e)), Expr::C(180.0 / std::f32::consts::PI)]),
            "clip" => {
                let lo = e.f("clipMin").unwrap_or(f32::NEG_INFINITY);
                let hi = e.f("clipMax").unwrap_or(f32::INFINITY);
                let x = e.kids.iter().find(|k| k.name != "clipMin" && k.name != "clipMax").map(|k| self.sgexpr(k)).unwrap_or(Expr::C(0.0));
                op("clamp", vec![x, Expr::C(lo), Expr::C(hi)])
            }
            "table" => {
                let x = e.kids.iter().find(|k| k.name != "entry").map(|k| self.sgexpr(k)).unwrap_or(Expr::C(0.0));
                interp_expr(x, table_of(e))
            }
            "condition" => self.condition(e),
            "ifthenelse" | "cond" => {
                let a = args(e);
                if a.len() >= 3 {
                    let c = a[0].clone();
                    op("add", vec![op("mul", vec![c.clone(), a[1].clone()]), op("mul", vec![op("not", vec![c]), a[2].clone()])])
                } else {
                    Expr::C(0.0)
                }
            }
            _ => {
                *self.unmapped.borrow_mut().entry(format!("<expr {}>", e.name)).or_default() += 1;
                Expr::C(0.0)
            }
        }
    }

    /// The input of an animation: `<expression>` or `<property>` (or a constant 0).
    pub fn input(&self, a: &Elem) -> Option<Expr> {
        if let Some(x) = a.child("expression") {
            return Some(self.sgexpr(x));
        }
        a.text_of("property").map(|p| self.prop(p))
    }
}

pub fn table_of(e: &Elem) -> Vec<(f32, f32)> {
    let mut t: Vec<(f32, f32)> = e
        .children("entry")
        .filter_map(|en| Some((en.f("ind")?, en.f("dep")?)))
        .collect();
    t.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    t
}

pub fn interp_expr(x: Expr, t: Vec<(f32, f32)>) -> Expr {
    if let Some(v) = x.is_const() {
        return Expr::C(interp(&t, v));
    }
    Expr::Interp(Box::new(x), t)
}

/// FG animation value: interpolation table, or input · factor + offset, clamped.
/// `unit` is "deg" or "m". With no input the value is the offset alone.
pub fn anim_value(pm: &PropMap, a: &Elem, unit: &str) -> Expr {
    let input = pm.input(a);
    let mut v = if let Some(interp) = a.child("interpolation") {
        interp_expr(input.unwrap_or(Expr::C(0.0)), table_of(interp))
    } else {
        let factor = a.f("factor").unwrap_or(1.0);
        let offset = a.f(&format!("offset-{unit}")).or_else(|| a.f("offset")).unwrap_or(0.0);
        match input {
            Some(i) => op("add", vec![op("mul", vec![i, Expr::C(factor)]), Expr::C(offset)]),
            None => Expr::C(offset),
        }
    };
    let lo = a.f(&format!("min-{unit}")).or_else(|| a.f("min"));
    let hi = a.f(&format!("max-{unit}")).or_else(|| a.f("max"));
    if lo.is_some() || hi.is_some() {
        v = op("clamp", vec![v, Expr::C(lo.unwrap_or(f32::NEG_INFINITY)), Expr::C(hi.unwrap_or(f32::INFINITY))]);
    }
    v
}
