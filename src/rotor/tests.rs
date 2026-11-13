//! Rotorcraft flight-model tests against published EC130 B4 data and FPV
//! quad behaviour. `cargo test --release rotor -- --nocapture` prints the
//! published-vs-measured table.

use super::heli::{self, HeliDef};
use super::*;
use crate::flight::{Aircraft, Controls, Environment, FPM, GRAVITY, KT, find, isa};
use glam::{Vec2, Vec3};

const DT: f32 = 1.0 / 120.0;
const MTOW: f32 = 2427.0;

/// Flat ground at y = 0 (optionally a wind).
#[derive(Default)]
struct Flat {
    wind: Vec3,
}

impl Environment for Flat {
    fn ground_height(&self, _xz: Vec2) -> f32 {
        0.0
    }
    fn obstacle_top(&self, _xz: Vec2) -> Option<f32> {
        None
    }
    fn is_water(&self, _xz: Vec2) -> bool {
        false
    }
    fn updraft(&self, _p: Vec3) -> f32 {
        0.0
    }
    fn wind(&self, _p: Vec3) -> Vec3 {
        self.wind
    }
}

fn ec130() -> (crate::flight::AircraftDef, HeliDef) {
    let def = find("ec130").expect("ec130");
    let h = def.rotor.as_ref().unwrap().heli().unwrap().clone();
    (def, h)
}

/// Hover power required (shaft, W) out of / in ground effect.
fn hover_power(h: &HeliDef, mass: f32, alt: f32, hub_agl: f32) -> f32 {
    let atm = isa(alt);
    heli::trim(h, mass, 0.0, 0.0, atm.density, atm.speed_of_sound, hub_agl).power
}

fn power_avail(h: &HeliDef, alt: f32) -> f32 {
    let atm = isa(alt);
    h.power_available(atm.density / 1.225, atm.temperature_c(), atm.temperature_c(), 1.0, h.p_takeoff)
}

/// Highest altitude (m) where hover power ≤ power available.
fn hover_ceiling(h: &HeliDef, mass: f32, hub_agl: f32) -> f32 {
    let (mut lo, mut hi) = (0.0f32, 8000.0f32);
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if hover_power(h, mass, mid, hub_agl) <= power_avail(h, mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Max level speed (kt TAS) with power ≤ `p` at altitude `alt`.
fn max_speed(h: &HeliDef, mass: f32, alt: f32, p: f32) -> f32 {
    let atm = isa(alt);
    let mut best = 0.0;
    let mut v = 20.0;
    while v < 100.0 {
        let t = heli::trim(h, mass, v, 0.0, atm.density, atm.speed_of_sound, 1e4);
        if t.converged && t.power <= p {
            best = v;
        }
        v += 0.5;
    }
    best / KT
}

/// Hub height for an IGE hover at ~1 m skid height.
const IGE_HUB: f32 = 1.0 + 1.64 + 1.455;

#[test]
#[ignore]
fn rotor_diag() {
    let (def, h) = ec130();
    println!("sigma {:.4} area {:.1} tip {:.0} m/s", h.solidity(), h.disc_area(), h.omega * h.radius);
    for alt in [0.0, 1000.0, 2000.0, 2537.0, 3098.0] {
        let t = heli::trim(&h, MTOW, 0.0, 0.0, isa(alt).density, isa(alt).speed_of_sound, 1e4);
        let ti = heli::trim(&h, MTOW, 0.0, 0.0, isa(alt).density, isa(alt).speed_of_sound, IGE_HUB);
        println!(
            "alt {alt:5.0}: OGE {:.0} kW (main {:.0} tail {:.0}) IGE {:.0} kW avail {:.0} kW coll {:.2} lon {:.2} lat {:.2} ped {:.2} pitch {:.1} roll {:.1} conv {} res {:.1e} vi {:.1}",
            t.power / 1e3,
            t.p_main / 1e3,
            t.p_tail / 1e3,
            ti.power / 1e3,
            power_avail(&h, alt) / 1e3,
            t.inp.coll,
            t.inp.lon,
            t.inp.lat,
            t.inp.ped,
            t.pitch.to_degrees(),
            t.roll.to_degrees(),
            t.converged,
            t.residual,
            t.state.vi
        );
    }
    for kt in [0.0, 10.0, 20.0, 30.0, 40.0, 60.0, 80.0, 100.0, 120.0, 130.0, 140.0, 155.0] {
        let t = heli::trim(&h, MTOW, kt * KT, 0.0, 1.225, 340.0, 1e4);
        println!(
            "{kt:5.0} kt: P {:.0} kW coll {:.2} lon {:.2} lat {:.2} ped {:.2} pitch {:.1} roll {:.1} conv {}",
            t.power / 1e3,
            t.inp.coll,
            t.inp.lon,
            t.inp.lat,
            t.inp.ped,
            t.pitch.to_degrees(),
            t.roll.to_degrees(),
            t.converged
        );
    }
    println!("HOGE {:.0} m HIGE {:.0} m", hover_ceiling(&h, MTOW, 1e4), hover_ceiling(&h, MTOW, IGE_HUB));
    println!("Vmax MCP {:.0} kt", max_speed(&h, MTOW, 0.0, h.p_mcp.min(h.p_mgb)));
    let _ = def;
}

/// Airborne helicopter with the given assist, trimmed at `kt`.
fn heli_air(def: &crate::flight::AircraftDef, alt: f32, kt: f32, assist: Assist) -> (Aircraft, Controls) {
    let mut ac = Aircraft::airborne(def, Vec3::new(0.0, alt, 0.0), 0.0, kt * KT);
    ac.rotor.assist = assist;
    let mut ctl = crate::flight::trimmed_controls_at(def, kt * KT, alt);
    if assist != Assist::Att {
        let rd = def.rotor.as_ref().unwrap();
        let mut c2 = ctl;
        crate::rotor::settle(rd, def, &mut ac, &mut c2, true);
        ctl = c2;
    }
    (ac, ctl)
}

fn mtow_def() -> crate::flight::AircraftDef {
    let mut d = find("ec130").unwrap();
    d.mass = MTOW;
    d
}

/// Collective that holds a climb rate (simple test-pilot PI).
struct VsHold {
    i: f32,
}

impl VsHold {
    fn coll(&mut self, base: f32, vs: f32, target: f32, dt: f32) -> f32 {
        let e = target - vs;
        self.i = (self.i + e * dt * 0.05).clamp(-0.4, 0.4);
        (base + self.i + 0.04 * e).clamp(0.0, 1.0)
    }
}

#[test]
fn heli_hover_holds_with_attitude_assist() {
    let def = find("ec130").unwrap();
    let env = Flat::default();
    let (mut ac, mut ctl) = heli_air(&def, 300.0, 0.0, Assist::Att);
    let base = ctl.throttle;
    let mut vh = VsHold { i: 0.0 };
    let start = ac.pos;
    let mut max_drift = 0.0f32;
    for k in 0..(40 * 120) {
        ctl.throttle = vh.coll(base, ac.vel.y, 0.0, DT);
        ac.step(&def, &ctl, &env, DT);
        assert!(ac.crashed.is_none(), "{:?}", ac.crashed);
        if k > 10 * 120 {
            max_drift = max_drift.max(Vec2::new(ac.pos.x - start.x, ac.pos.z - start.z).length());
        }
    }
    let o = ac.rotor.out;
    println!(
        "hover: drift {max_drift:.1} m, alt {:.1}, Nr {:.1} %, Tq {:.0} %, pitch {:.1} roll {:.1} hdg {:.1}",
        ac.pos.y,
        o.nr_pct,
        o.torque_pct,
        ac.pitch().to_degrees(),
        ac.roll().to_degrees(),
        ac.heading().to_degrees()
    );
    assert!(max_drift < 25.0, "drifted {max_drift} m");
    assert!((ac.pos.y - 300.0).abs() < 8.0);
    assert!((o.nr_pct - 100.0).abs() < 2.0, "Nr {}", o.nr_pct);
    assert!(ac.heading().to_degrees().abs() < 5.0, "heading {}", ac.heading().to_degrees());
}

#[test]
fn heli_hover_ceilings_match_published() {
    let (_, h) = ec130();
    let oge = hover_ceiling(&h, MTOW, 1e4);
    let ige = hover_ceiling(&h, MTOW, IGE_HUB);
    println!("| Hover ceiling OGE (ISA, MTOW) | 2,537 m | {oge:.0} m |");
    println!("| Hover ceiling IGE (ISA, MTOW) | 3,098 m | {ige:.0} m |");
    assert!((oge / 2537.0 - 1.0).abs() < 0.06, "HOGE {oge}");
    assert!((ige / 3098.0 - 1.0).abs() < 0.06, "HIGE {ige}");
}

#[test]
fn heli_max_speed_matches_published() {
    let (_, h) = ec130();
    let v = max_speed(&h, MTOW, 0.0, h.p_mcp.min(h.p_mgb));
    println!("| Max level speed (MCP, SL, MTOW) | 130 kt | {v:.0} kt |");
    assert!((v / 130.0 - 1.0).abs() < 0.06, "Vmax {v}");
    // Vne is reachable in a dive but not level with take-off power at MTOW.
    let vt = max_speed(&h, MTOW, 0.0, h.p_mgb);
    assert!(vt < 155.0, "{vt}");
}

#[test]
fn heli_translational_lift_cuts_power() {
    let (_, h) = ec130();
    let p0 = heli::trim(&h, MTOW, 0.0, 0.0, 1.225, 340.0, 1e4).power;
    let p25 = heli::trim(&h, MTOW, 25.0 * KT, 0.0, 1.225, 340.0, 1e4).power;
    let p65 = heli::trim(&h, MTOW, 65.0 * KT, 0.0, 1.225, 340.0, 1e4).power;
    println!("| ETL: power at 25 kt / hover (OGE) | ≈ 0.75–0.85 | {:.2} |", p25 / p0);
    println!("| Minimum-power speed power / hover | ≈ 0.5–0.6 | {:.2} |", p65 / p0);
    assert!(p25 / p0 < 0.86 && p25 / p0 > 0.65, "{}", p25 / p0);
    assert!(p65 / p0 < 0.62 && p65 / p0 > 0.45, "{}", p65 / p0);
}

#[test]
fn heli_ground_effect_and_vortex_ring() {
    let (_, h) = ec130();
    let oge = heli::trim(&h, MTOW, 0.0, 0.0, 1.225, 340.0, 1e4).power;
    let ige = heli::trim(&h, MTOW, 0.0, 0.0, 1.225, 340.0, IGE_HUB).power;
    assert!(ige < 0.95 * oge, "IGE {ige} OGE {oge}");
    // Vertical descent at ~0.8 v_h: the vortex ring state (thrust loss, roughness).
    let mut st = heli::HeliState { nr: h.omega, vi: 10.0, tr_vi: 20.0, ..Default::default() };
    let inp = heli::Inp { coll: 0.6, lon: -0.24, lat: 0.0, ped: 0.2 };
    let hover = heli::steady(&h, &mut st, inp, Vec3::ZERO, 1.225, 340.0, 1e4);
    let vrs = heli::steady(&h, &mut st, inp, Vec3::new(0.0, -8.5, 0.0), 1.225, 340.0, 1e4);
    let fast = heli::steady(&h, &mut st, inp, Vec3::new(0.0, -8.5, 25.0), 1.225, 340.0, 1e4);
    println!("VRS: hover T {:.0} N, 1,700 fpm vertical descent T {:.0} N (severity {:.2}), same descent at 50 kt T {:.0} N", hover.thrust, vrs.thrust, vrs.vrs, fast.thrust);
    assert!(vrs.vrs > 0.5, "{}", vrs.vrs);
    assert!(fast.vrs < 0.05);
}

/// Test pilot: stick for a pitch attitude (through the attitude-hold assist's rate command).
fn att(ac: &Aircraft, target: f32) -> f32 {
    (3.0 * (target - ac.pitch()) - 0.4 * ac.rates().y).clamp(-1.0, 1.0)
}

/// Autorotation from 65 kt / 2,000 ft: Nr kept in the green with the
/// collective, a steady descent, then flare and cushion to a touchdown.
fn autorotation(def: &crate::flight::AircraftDef, flare: bool) -> (f32, f32, Option<String>, f32, f32) {
    let env = Flat::default();
    let (mut ac, mut ctl) = heli_air(def, 610.0, 65.0, Assist::Att);
    let base = ctl.throttle;
    let mut vh = VsHold { i: 0.0 };
    let mut t = 0.0;
    let mut descent = Vec::new();
    let mut nr_min = 200.0f32;
    let mut nr_i = 0.0;
    let mut phase = 0;
    let mut touchdown_sink = 0.0;
    let mut ias_i = 0.0;
    while t < 120.0 {
        let agl = ac.pos.y - def.gear_height;
        let nr = ac.rotor.out.nr_pct;
        if t < 5.0 {
            ctl.throttle = vh.coll(base, ac.vel.y, 0.0, DT);
        } else {
            ctl.failed_engines = 1;
        }
        // Pitch: hold 65 kt (stick back when fast), then flare.
        let ias = ac.airspeed_kt();
        let mut stick = 0.0;
        if t >= 5.0 {
            match phase {
                0 => {
                    // Pilot reaction ~1 s, then lower the collective (~1 s full travel)
                    // and fly the rotor RPM with it; hold 65 kt with the attitude.
                    if t > 6.0 {
                        let e = (nr - 96.0) / 100.0;
                        nr_i = (nr_i + e * DT * 0.8).clamp(-0.3, 0.3);
                        let want = (0.25 + 3.0 * e + nr_i).clamp(0.0, 1.0);
                        ctl.throttle += (want - ctl.throttle).clamp(-1.5 * DT, 0.8 * DT);
                    }
                    if t > 6.0 {
                        nr_min = nr_min.min(nr);
                    }
                    let e = ias - 65.0;
                    ias_i = (ias_i + e * DT * 0.002).clamp(-0.1, 0.1);
                    let target = (0.02 + 0.012 * e + ias_i).clamp(-0.2, 0.25);
                    stick = att(&ac, target);
                    if t > 12.0 && t < 30.0 {
                        descent.push(-ac.vel.y);
                    }
                    if flare && agl < 30.0 {
                        phase = 1;
                    }
                }
                1 => {
                    // Flare: ~14° nose up from 30 m (less if it balloons): speed and
                    // descent wash off, the rotor speed builds.
                    let target = if ac.vel.y > -1.0 { 0.12 } else { 0.25 };
                    stick = att(&ac, target);
                    let e = (nr - 104.0) / 100.0;
                    let want = (0.25 + 3.0 * e).clamp(0.0, 0.6);
                    ctl.throttle += (want - ctl.throttle).clamp(-0.8 * DT, 0.8 * DT);
                    if agl < 7.0 {
                        phase = 2;
                    }
                }
                _ => {
                    // Level the skids (tail clearance) and cushion with the collective.
                    stick = att(&ac, 0.04);
                    let sink = -ac.vel.y;
                    let sink_t = 0.3 + 0.7 * agl;
                    ctl.throttle = (ctl.throttle + DT * (1.5 * (sink - sink_t)).clamp(-1.2, 1.2)).clamp(0.0, 0.95);
                }
            }
        }
        ctl.elevator = stick;
        ctl.aileron = 0.0;
        if std::env::var_os("AUTO_TRACE").is_some() && ((t * 120.0) as i32) % 30 == 0 {
            eprintln!(
                "t {t:5.1} ph {phase} Nr {nr:5.1} coll {:.2} stick {stick:5.2} vs {:6.1} ias {:5.1} pitch {:5.1} roll {:5.1} agl {agl:6.1} T {:.0} vi {:.1} q {:.2}",
                ctl.throttle,
                ac.vel.y,
                ac.airspeed_kt(),
                ac.pitch().to_degrees(),
                ac.roll().to_degrees(),
                ac.rotor.out.thrust,
                ac.rotor.out.v_induced,
                ac.rates().y
            );
        }
        let was = ac.on_ground;
        ac.step(def, &ctl, &env, DT);
        if ac.on_ground && !was && touchdown_sink == 0.0 {
            touchdown_sink = -ac.vel.y.min(0.0) + 1e-3;
        }
        if ac.crashed.is_some() || (ac.on_ground && ac.vel.length() < 0.5) {
            break;
        }
        t += DT;
        if !flare && t > 32.0 {
            break;
        }
    }
    let rod = descent.iter().sum::<f32>() / descent.len().max(1) as f32;
    (rod / FPM, nr_min, ac.crashed.clone(), touchdown_sink, ac.rotor.out.nr_pct)
}

#[test]
fn heli_autorotation_descent_and_flare_landing() {
    let def = find("ec130").unwrap();
    let (rod, nr_min, crashed, sink, nr_end) = autorotation(&def, true);
    println!("| Autorotation rate of descent (65 kt) | ≈ 1,500–1,900 fpm | {rod:.0} fpm |");
    println!("| Autorotation: Nr (min after entry) | ≥ 85 % (power-off green 83–111 %) | {nr_min:.0} % |");
    println!("| Autorotation: touchdown after flare | < 1.5 m/s | {sink:.2} m/s (crash: {crashed:?}, Nr at rest {nr_end:.0} %) |");
    assert!((1350.0..2100.0).contains(&rod), "rod {rod}");
    assert!(nr_min > 85.0, "Nr min {nr_min}");
    assert!(crashed.is_none(), "{crashed:?}");
    assert!(sink < 1.5, "touchdown sink {sink}");
}

#[test]
fn heli_engine_failure_without_action_decays_rotor() {
    // Collective left up: Nr decays fast (the low-Nr horn) — the pilot has ~1–2 s.
    let def = find("ec130").unwrap();
    let env = Flat::default();
    let (mut ac, mut ctl) = heli_air(&def, 600.0, 0.0, Assist::Att);
    ctl.failed_engines = 1;
    let mut warned = None;
    for k in 0..(3 * 120) {
        ac.step(&def, &ctl, &env, DT);
        if warned.is_none() && ac.rotor.out.low_nr {
            warned = Some(k as f32 * DT);
        }
    }
    let nr = ac.rotor.out.nr_pct;
    println!("engine failure in the hover, collective held: low-Nr warning after {warned:?} s, Nr {nr:.0} % after 3 s");
    assert!(warned.is_some_and(|w| w < 1.5));
    assert!(nr < 80.0);
}

#[test]
fn heli_dynamic_rollover_on_a_skid() {
    let def = find("ec130").unwrap();
    let env = Flat::default();
    let mut ac = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    ac.rotor.assist = Assist::Off;
    let mut ctl = Controls::default();
    let mut crashed_at = None;
    // Light on the skids, then full right cyclic held: rolls over the right skid.
    for k in 0..(12 * 120) {
        let t = k as f32 * DT;
        ctl.throttle = (t * 0.25).min(0.52);
        ctl.aileron = if t > 2.5 { 1.0 } else { 0.0 };
        ctl.elevator = -0.24;
        ctl.rudder = 0.2;
        ac.step(&def, &ctl, &env, DT);
        if ac.crashed.is_some() {
            crashed_at = Some(t);
            break;
        }
    }
    println!("dynamic rollover: {:?} at {crashed_at:?} s", ac.crashed);
    assert!(ac.crashed.as_deref().is_some_and(|r| r.contains("rollover")), "{:?}", ac.crashed);
    // The same with the collective lowered as soon as the bank passes 5°: it settles back.
    let mut ac = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    ac.rotor.assist = Assist::Off;
    let mut down = false;
    for k in 0..(12 * 120) {
        let t = k as f32 * DT;
        down |= ac.roll().abs() > 5f32.to_radians();
        ctl.throttle = if down { 0.0 } else { (t * 0.25).min(0.52) };
        ctl.aileron = if t > 2.5 && !down { 1.0 } else { 0.0 };
        ac.step(&def, &ctl, &env, DT);
    }
    assert!(ac.crashed.is_none(), "{:?}", ac.crashed);
    assert!(ac.on_ground && ac.roll().abs() < 0.08);
}

#[test]
fn heli_tail_rotor_drive_failure_spins() {
    let def = find("ec130").unwrap();
    let env = Flat::default();
    let (mut ac, mut ctl) = heli_air(&def, 600.0, 0.0, Assist::Att);
    let base = ctl.throttle;
    let mut vh = VsHold { i: 0.0 };
    crate::systems::set_failure(&mut ac, &def, crate::systems::failures::Failure::new(crate::systems::failures::Kind::TailRotor, 0), true);
    let mut max_r = 0.0f32;
    for _ in 0..(3 * 120) {
        ctl.throttle = vh.coll(base, ac.vel.y, 0.0, DT);
        ac.step(&def, &ctl, &env, DT);
        max_r = max_r.max(ac.rates().z.abs());
    }
    println!("tail-rotor drive failure in the hover: yaw rate {:.0}°/s after 3 s", max_r.to_degrees());
    // Clockwise rotor: the fuselage yaws left.
    assert!(ac.rates().z < 0.0 && max_r.to_degrees() > 45.0);
}

#[test]
fn heli_cruise_is_stable_hands_off_with_sas() {
    let def = find("ec130").unwrap();
    let env = Flat::default();
    let (mut ac, ctl) = heli_air(&def, 600.0, 110.0, Assist::Att);
    for _ in 0..(30 * 120) {
        ac.step(&def, &ctl, &env, DT);
        assert!(ac.crashed.is_none());
    }
    println!("cruise 110 kt, 30 s hands off: {:.0} kt, alt {:.0} m, pitch {:.1} roll {:.1}", ac.airspeed_kt(), ac.pos.y, ac.pitch().to_degrees(), ac.roll().to_degrees());
    assert!((ac.airspeed_kt() - 110.0).abs() < 15.0);
    assert!((ac.pos.y - 600.0).abs() < 120.0);
    assert!(ac.roll().abs() < 0.2);
}

#[test]
fn heli_retreating_blade_stall_beyond_vne() {
    let (_, h) = ec130();
    let mut st = heli::HeliState { nr: h.omega, vi: 2.0, tr_vi: 5.0, ..Default::default() };
    // 175 kt pulling 1.8 g: stalled, the nose pitches up and it rolls to the retreating (right) side.
    let inp = heli::Inp { coll: 0.95, lon: -1.0, lat: 0.0, ped: -0.2 };
    let ev = heli::steady(&h, &mut st, inp, Vec3::new(0.0, 0.0, 175.0 * KT), 1.225, 340.0, 1e4);
    assert!(ev.stall > 0.2, "{}", ev.stall);
    let mut st = heli::HeliState { nr: h.omega, vi: 2.0, tr_vi: 5.0, ..Default::default() };
    let ok = heli::steady(&h, &mut st, heli::Inp { coll: 0.55, ..inp }, Vec3::new(0.0, 0.0, 120.0 * KT), 1.225, 340.0, 1e4);
    assert!(ok.stall == 0.0);
}

// ---------------------------------------------------------------------------
// Quadcopter
// ---------------------------------------------------------------------------

#[test]
fn quad_rate_mode_step_response() {
    let def = find("fpv").unwrap();
    let q = def.rotor.as_ref().unwrap().quad().unwrap().clone();
    let env = Flat::default();
    let mut ac = Aircraft::airborne(&def, Vec3::new(0.0, 100.0, 0.0), 0.0, 0.0);
    ac.rotor.assist = Assist::Off;
    let mut ctl = crate::flight::trimmed_controls_at(&def, 0.0, 100.0);
    for _ in 0..60 {
        ac.step(&def, &ctl, &env, DT);
    }
    let cmd = q.rate(0.5, false);
    ctl.aileron = 0.5;
    let (mut t90, mut peak) = (None, 0.0f32);
    for k in 0..48 {
        ac.step(&def, &ctl, &env, DT);
        let p = ac.rates().x;
        peak = peak.max(p);
        if t90.is_none() && p > 0.9 * cmd {
            t90 = Some((k + 1) as f32 * DT);
        }
    }
    let over = peak / cmd - 1.0;
    println!("| Quad acro roll step (50 % stick = {:.0}°/s): 90 % rise | < 0.12 s | {:.3} s, overshoot {:.0} % |", cmd.to_degrees(), t90.unwrap_or(9.0), over * 100.0);
    assert!(t90.is_some_and(|t| t < 0.12), "{t90:?}");
    assert!(over < 0.15, "overshoot {over}");
    assert!(ac.crashed.is_none());
}

#[test]
fn quad_angle_mode_levels_and_holds_tilt() {
    let def = find("fpv").unwrap();
    let env = Flat::default();
    let mut ac = Aircraft::airborne(&def, Vec3::new(0.0, 100.0, 0.0), 0.0, 0.0);
    ac.rotor.assist = Assist::Att;
    let mut ctl = crate::flight::trimmed_controls_at(&def, 0.0, 100.0);
    ctl.elevator = -0.5; // forward: nose down 27.5°
    for _ in 0..120 {
        ac.step(&def, &ctl, &env, DT);
    }
    let tilt = ac.pitch().to_degrees();
    ctl.elevator = 0.0;
    for _ in 0..120 {
        ac.step(&def, &ctl, &env, DT);
    }
    println!("angle mode: 50 % forward stick → {tilt:.1}° nose down; released → {:.1}°", ac.pitch().to_degrees());
    assert!((tilt + 27.5).abs() < 3.0, "{tilt}");
    assert!(ac.pitch().to_degrees().abs() < 3.0);
}

#[test]
fn quad_hover_and_battery() {
    let def = find("fpv").unwrap();
    let env = Flat::default();
    let mut ac = Aircraft::airborne(&def, Vec3::new(0.0, 50.0, 0.0), 0.0, 0.0);
    let mut ctl = crate::flight::trimmed_controls_at(&def, 0.0, 50.0);
    let base = ctl.throttle;
    let mut i = 0.0;
    for _ in 0..(60 * 120) {
        let e = -ac.vel.y;
        i = (i + e * DT * 0.02).clamp(-0.2, 0.2);
        ctl.throttle = (base + i + 0.02 * e).clamp(0.0, 1.0);
        ac.step(&def, &ctl, &env, DT);
        assert!(ac.crashed.is_none());
    }
    let o = ac.rotor.out;
    println!("| Quad hover current / 1 min used | ≈ 5–8 A / 80–130 mAh | {:.1} A / {:.0} mAh ({:.2} V) |", o.batt_a, o.batt_mah, o.batt_v);
    assert!((3.5..10.0).contains(&o.batt_a), "{}", o.batt_a);
    // Full throttle: thrust to weight ~8, the pack sags.
    ctl.throttle = 1.0;
    for _ in 0..30 {
        ac.step(&def, &ctl, &env, DT);
    }
    let q = def.rotor.as_ref().unwrap().quad().unwrap();
    let tw = 4.0 * q.max_thrust(ac.rotor.out.batt_v) / (def.mass * GRAVITY);
    println!("| Quad full-throttle static thrust / weight (sagged pack), current | ≈ 6–9, 80–130 A | {tw:.1}, {:.0} A, {:.1} V |", ac.rotor.out.batt_a, ac.rotor.out.batt_v);
    assert!((6.0..10.0).contains(&tw), "{tw}");
}

#[test]
#[ignore]
fn rotor_diag_auto() {
    let (_, h) = ec130();
    for vs in [0.0f32, -4.0, -6.0, -8.0, -9.0, -10.0, -11.0, -12.0, -14.0] {
        let t = heli::trim(&h, 2100.0, 65.0 * KT, vs, 1.225, 340.0, 1e4);
        println!(
            "vs {vs:5.1} ({:5.0} fpm): Pmain {:6.0} kW Ptail {:5.1} coll {:.2} lon {:.2} pitch {:.1} conv {} vi {:.1}",
            vs / FPM,
            t.p_main / 1e3,
            t.p_tail / 1e3,
            t.inp.coll,
            t.inp.lon,
            t.pitch.to_degrees(),
            t.converged,
            t.state.vi
        );
    }
}
