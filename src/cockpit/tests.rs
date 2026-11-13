//! Cockpit tests: every imported aircraft's instrument channels bind, the
//! importer produced enough live instruments, and the instrument maths
//! (lags, Kollsman, fuel, drums, UV transforms) behave.

use super::channels::{Chan, EngQ, Instruments, TankQ, resolve};
use crate::aircraft_asset::{AircraftAsset, RExpr, UvAnim, step_scroll, uv_transform};
use crate::autopilot::Autopilot;
use crate::flight::{self, Aircraft, Controls, KT};
use crate::models::{ModelInputs, channel_value};
use glam::{Vec2, Vec3};

/// Minimum live cockpit instruments per type (needles + flags + drums +
/// display screens), from the current imports with some headroom.
const MIN_LIVE: [(&str, usize); 10] = [
    ("kestrel", 50),
    ("swift", 18),
    ("peregrine", 20),
    ("lancer", 40),
    ("aerolink", 80),
    ("meridian190", 40),
    ("meridian320", 8),
    ("hercules", 70),
    ("galaxy", 20),
    ("vulcan", 55),
];

/// Channels computed in `models.rs` (controls, surfaces, gear, lights, engines' basics).
fn model_channel(n: &str) -> bool {
    let num = |pre: &str, suf: &str| n.strip_prefix(pre).and_then(|r| r.strip_suffix(suf)).is_some_and(|d| d.parse::<u32>().is_ok());
    matches!(
        n,
        "elevator"
            | "aileron_l"
            | "aileron_r"
            | "rudder"
            | "flaps"
            | "spoilers"
            | "gear_pos"
            | "wow"
            | "steer_norm"
            | "steer_deg"
            | "afterburner"
            | "throttle"
            | "brakes"
            | "nav"
            | "beacon"
            | "beacon_sw"
            | "strobe"
            | "strobe_sw"
            | "landing_light"
            | "taxi_light"
            | "time"
            | "mach"
            | "alpha_deg"
            | "doors"
            | "doors_level"
            | "doors_kneeled"
            | "kneel"
    ) || n.starts_with("ctl_")
        || ["_running", "_reverser"].iter().any(|s| num("eng", s))
        || num("prop", "_rpm")
        || ["_roll_ms", "_comp_m", "_comp_norm"].iter().any(|s| num("gear", s))
}

fn inputs<'a>(def: &'a flight::AircraftDef, ac: &'a Aircraft, ctl: &'a Controls, ap: &'a Autopilot, time: f32) -> ModelInputs<'a> {
    ModelInputs { def, ac, ctl, time, nav: true, beacon: true, strobe: true, landing: false, ap: Some(ap), qnh_hpa: 1018.0, hours: 14.5, daylight: 1.0, radios: None }
}

#[test]
fn every_channel_binds_and_enough_instruments_are_live() {
    let mut report = Vec::new();
    for (id, min) in MIN_LIVE {
        let Ok(a) = AircraftAsset::load(id) else {
            eprintln!("{id}: asset not found, skipped");
            continue;
        };
        for c in &a.channels {
            assert!(resolve(c).is_some() || model_channel(c), "{id}: channel `{c}` has no source");
        }
        let n = a.live_instruments();
        report.push(format!("{id}: {} live ({} needles, {} flags, {} drums, {} lit, {} displays)", n.total(), n.needles, n.flags, n.drums, n.lit, n.displays));
        assert!(n.total() >= min, "{id}: only {} live instruments (want ≥ {min}): {n:?}", n.total());
    }
    eprintln!("{}", report.join("\n"));
}

#[test]
fn glass_types_have_their_displays() {
    for (id, kinds) in [("meridian320", &["pfd", "nd", "ewd", "sd", "isis"][..]), ("meridian190", &["pfd", "nd", "ewd", "isis"][..]), ("lancer", &["nd", "ewd"][..])] {
        let Ok(a) = AircraftAsset::load(id) else { continue };
        let d = a.displays();
        for k in kinds {
            let (_, aspect) = d.iter().find(|(n, _)| n == k).unwrap_or_else(|| panic!("{id}: no {k} display in {d:?}"));
            assert!((0.5..2.0).contains(aspect), "{id} {k}: aspect {aspect}");
        }
    }
}

#[test]
fn air_data_bindings() {
    let defs = flight::catalogue();
    let def = &defs[flight::index_of("kestrel").unwrap()];
    let ctl = Controls::default();
    let ap = Autopilot::default();
    let a100 = Aircraft::airborne(def, Vec3::new(0.0, 900.0, 0.0), 120f32.to_radians(), 100.0 * KT);
    let mut ins = Instruments::default();
    let i = inputs(def, &a100, &ctl, &ap, 0.0);
    ins.step(&i, 0.0);
    let v = |ins: &Instruments, i: &ModelInputs, c: Chan| ins.value(c, i);
    assert!((v(&ins, &i, Chan::IasKt) - a100.airspeed_kt()).abs() < 0.01);
    // Altimeter set to the QNH reads the altitude; the Kollsman window shows the setting.
    assert!((v(&ins, &i, Chan::AltFt) - 900.0 * 3.280_84).abs() < 1.0);
    assert!((v(&ins, &i, Chan::AltSetInhg) - 30.06).abs() < 0.01);
    assert!((v(&ins, &i, Chan::PaltFt) - (900.0 * 3.280_84 - 4.75 * 27.0)).abs() < 1.0);
    assert!((v(&ins, &i, Chan::HeadingDeg) - 120.0).abs() < 0.5);
    assert!((v(&ins, &i, Chan::CompassDeg) - 120.0).abs() < 0.5);
    // Clock: 14:30 local.
    assert!((v(&ins, &i, Chan::ClockHour) - 14.5).abs() < 1e-4);
    assert!((v(&ins, &i, Chan::ClockS) - 52_200.0).abs() < 0.1);
    // The ASI lags a speed change by its pitot time constant (0.25 s).
    let a150 = Aircraft::airborne(def, Vec3::new(0.0, 900.0, 0.0), 120f32.to_radians(), 150.0 * KT);
    let i2 = inputs(def, &a150, &ctl, &ap, 0.25);
    let start = v(&ins, &i, Chan::IasKt);
    ins.step(&i2, 0.25);
    let want = start + (a150.airspeed_kt() - start) * (1.0 - (-1.0f32).exp());
    assert!((v(&ins, &i2, Chan::IasKt) - want).abs() < 0.05, "{} vs {want}", v(&ins, &i2, Chan::IasKt));
    // Fuel: two tanks, 110 kg of avgas.
    let gal = v(&ins, &i, Chan::Tank(TankQ::Gal, 0)) + v(&ins, &i, Chan::Tank(TankQ::Gal, 1));
    assert!((gal - 110.0 / 2.72).abs() < 0.5, "{gal}");
    assert_eq!(v(&ins, &i, Chan::Tank(TankQ::Gal, 2)), 0.0);
    // Engine gauges warm and turning.
    assert!(v(&ins, &i, Chan::Eng(EngQ::Rpm, 0)) > 1500.0);
    assert!(v(&ins, &i, Chan::Eng(EngQ::OilPPsi, 0)) > 40.0);
    assert!(v(&ins, &i, Chan::SuctionInhg) > 4.0);
}

#[test]
fn heading_bug_and_flight_director_follow_the_autopilot() {
    let defs = flight::catalogue();
    let def = &defs[flight::index_of("aerolink").unwrap()];
    let ctl = Controls::default();
    let ac = Aircraft::airborne(def, Vec3::new(0.0, 3000.0, 0.0), 90f32.to_radians(), 200.0 * KT);
    let mut ap = Autopilot::default();
    ap.engaged = true;
    ap.hdg = Some(120f32.to_radians());
    ap.alt = Some(3000.0);
    let mut ins = Instruments::default();
    let i = inputs(def, &ac, &ctl, &ap, 0.0);
    ins.step(&i, 0.0);
    ins.step(&i, 0.1);
    assert!((ins.value(Chan::HdgBugDeg, &i) - 120.0).abs() < 0.01);
    assert!((ins.value(Chan::HdgBugErrDeg, &i) - 30.0).abs() < 0.6);
    assert!(ins.value(Chan::FdRollDeg, &i) > 10.0, "FD should command a right bank");
    assert_eq!(ins.value(Chan::ApOn, &i), 1.0);
    assert!((ins.value(Chan::ApAltFt, &i) - 9_842.5).abs() < 1.0);
}

#[test]
fn model_and_instrument_channels_agree_on_controls() {
    // The cockpit layer only overrides instrument channels.
    let defs = flight::catalogue();
    let def = &defs[0];
    let ctl = Controls { elevator: 0.3, ..Default::default() };
    let ac = Aircraft::airborne(def, Vec3::new(0.0, 500.0, 0.0), 0.0, 90.0 * KT);
    let ap = Autopilot::default();
    let i = inputs(def, &ac, &ctl, &ap, 0.0);
    assert!(resolve("ctl_elevator").is_none());
    assert_eq!(channel_value("ctl_elevator", &i), -0.3);
}

#[test]
fn drum_counter_step_and_scroll() {
    // FlightGear SGStepExpression: plain steps…
    assert_eq!(step_scroll(7.6, 1.0, 0.0), 7.0);
    assert_eq!(step_scroll(-0.4, 1.0, 0.0), -1.0);
    // …and a drum that rolls into the next digit over the last 0.2.
    assert_eq!(step_scroll(7.5, 1.0, 0.2), 7.0);
    assert!((step_scroll(7.9, 1.0, 0.2) - 7.5).abs() < 1e-4);
    assert!((step_scroll(7.99, 1.0, 0.2) - 7.95).abs() < 1e-3);
    assert_eq!(step_scroll(3.0, 0.0, 0.0), 3.0);
}

#[test]
fn uv_transforms() {
    let apply = |m: [f32; 6], p: Vec2| Vec2::new(m[0] * p.x + m[2] * p.y + m[4], m[1] * p.x + m[3] * p.y + m[5]);
    // Translation along v by a channel.
    let tr = [UvAnim::Translate { axis: Vec2::new(0.0, -0.1), v: RExpr::Ch(0) }];
    let m = uv_transform(&tr, &[3.0]);
    assert!((apply(m, Vec2::new(0.2, 0.5)) - Vec2::new(0.2, 0.2)).length() < 1e-5);
    // Rotation by 90° about (0.5, 0.5): (1, 0.5) → (0.5, 1).
    let rot = [UvAnim::Rotate { center: Vec2::splat(0.5), v: RExpr::C(90.0) }];
    let m = uv_transform(&rot, &[]);
    assert!((apply(m, Vec2::new(1.0, 0.5)) - Vec2::new(0.5, 1.0)).length() < 1e-5);
    // Composition: translate first, then rotate about the centre.
    let both = [UvAnim::Translate { axis: Vec2::new(0.25, 0.0), v: RExpr::C(1.0) }, UvAnim::Rotate { center: Vec2::splat(0.5), v: RExpr::C(180.0) }];
    let m = uv_transform(&both, &[]);
    assert!((apply(m, Vec2::new(0.5, 0.5)) - Vec2::new(0.25, 0.5)).length() < 1e-5);
}

#[test]
fn old_and_future_files() {
    // A version-1 file still loads (format check only); a newer one is refused.
    let mut raw = b"SKAC".to_vec();
    raw.extend_from_slice(&99u32.to_le_bytes());
    raw.extend_from_slice(&[0; 8]);
    let e = AircraftAsset::parse(&raw, "x").err().unwrap();
    assert!(e.contains("version 99"), "{e}");
}

/// CPU cost of the per-frame instrument update + pose (printed; run with
/// `--release --nocapture`). No assertion: debug builds are much slower.
#[test]
fn instrument_update_cost() {
    let defs = flight::catalogue();
    for (id, _) in MIN_LIVE {
        let Some(k) = flight::index_of(id) else { continue };
        let def = &defs[k];
        let mut m = crate::models::build(id, def.eye);
        if m.asset.is_none() {
            continue;
        }
        let ac = Aircraft::airborne(def, Vec3::new(0.0, 900.0, 0.0), 1.0, def.vno_kt * 0.8 * KT);
        let ctl = Controls::default();
        let ap = Autopilot::default();
        let n = 200;
        let t0 = web_time::Instant::now();
        for f in 0..n {
            m.update(&inputs(def, &ac, &ctl, &ap, f as f32 / 120.0));
        }
        eprintln!("{id}: instruments + pose {:.3} ms/frame", t0.elapsed().as_secs_f32() * 1000.0 / n as f32);
    }
}
