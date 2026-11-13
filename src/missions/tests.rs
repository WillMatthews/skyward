//! Mission tests: definitions, the trigger engine, ditching, differential
//! thrust, and missions flown start to finish by the scripted pilot through
//! the real flight model and systems (headless, synthetic flat world with the
//! mission's target runway at the origin).

use super::defs::{self, Mission};
use super::pilot::{MissionPilot, PilotReq};
use super::*;
use crate::flight::{Aircraft, Controls, Environment, find};
use glam::{Vec2, Vec3, vec2, vec3};

const DT: f32 = 1.0 / 120.0;

/// Flat ground at `elev`, with water where `water` says.
struct TestEnv {
    elev: f32,
    water: Box<dyn Fn(Vec2) -> bool>,
    wind: Vec3,
}

impl Environment for TestEnv {
    fn ground_height(&self, _: Vec2) -> f32 {
        self.elev
    }
    fn obstacle_top(&self, _: Vec2) -> Option<f32> {
        None
    }
    fn is_water(&self, xz: Vec2) -> bool {
        (self.water)(xz)
    }
    fn updraft(&self, _: Vec3) -> f32 {
        0.0
    }
    fn wind(&self, _: Vec3) -> Vec3 {
        self.wind
    }
}

fn land_env(m: &Mission) -> TestEnv {
    TestEnv { elev: m.target.elev_ft * 0.3048, water: Box::new(|_| false), wind: Vec3::ZERO }
}

struct Flown {
    run: MissionRun,
    ac: Aircraft,
    log: Vec<String>,
}

/// Fly mission `id` with the scripted pilot for at most `secs`.
fn fly(id: &str, env: &TestEnv, secs: f32) -> Flown {
    let m: &'static Mission = defs::by_id(id).unwrap_or_else(|| panic!("mission {id}"));
    let def = find(&m.aircraft).unwrap();
    let strip = MissionRun::synthetic_strip(m, Vec2::ZERO);
    let strip = Strip { elev: env.elev, ..strip };
    let strips = vec![strip];
    let mut run = MissionRun::new(m);
    let mut ac = Aircraft::airborne(&def, vec3(0.0, 1000.0, 0.0), 0.0, 100.0);
    let mut ctl = Controls::default();
    let mut pilot = MissionPilot::default();
    let mut mayday = false;
    let mut severity = None;
    let amb = crate::systems::Ambient::default();
    let mut t = 0.0;
    let mut every = 0;
    while t < secs && !(run.finished() && run.end_t.is_some_and(|e| run.t - e > 0.5)) {
        if run.started && !run.finished() {
            let mut req = Vec::new();
            let air = AirEnv { inner: env, run: Some(&run) };
            pilot.fly(&def, &mut ac, &mut ctl, &run, &strips, &air, DT, &mut req);
            mayday |= req.contains(&PilotReq::Mayday);
        }
        if run.started && ac.crashed.is_none() {
            crate::systems::step(&mut ac, &def, &ctl, &amb, DT);
            let air = AirEnv { inner: env, run: Some(&run) };
            ac.step(&def, &ctl, &air, DT);
        }
        if let (Some(r), Some(imp), None) = (&ac.crashed, &ac.impact, severity) {
            severity = Some(ditching::classify(&def, r, imp, ac.dmg.failure.is_some()));
        }
        // (A crashed aircraft has stopped: the wreck model is the app's.)
        let mut ctx = Ctx { def: &def, ac: &mut ac, ctl: &mut ctl, env, strips: &strips, severity, mayday, dt: DT };
        run.step(&mut ctx);
        run.requests.clear();
        t += DT;
        every += 1;
        if every % 1200 == 0 && std::env::var_os("MISSION_TRACE").is_some() {
            let (al, si) = Strip::local(vec2(ac.pos.x, ac.pos.z), strips[0].a, (strips[0].b - strips[0].a).normalize());
            eprintln!(
                "{id} hyd {:?} rate {:?} pitch {:.1} elev {:.2} trim {:.2} t {:5.0} alt {:6.0} ft agl {:6.0} ias {:4.0} vs {:6.0} hdg {:4.0} along {:6.0} side {:5.0} thr {:.2} {:?} plan {:?}",
                ac.sys.hyd,
                ac.sys.fx.surface_rate,
                ac.pitch().to_degrees(),
                ctl.elevator,
                ctl.trim,
                run.t,
                ac.pos.y / 0.3048,
                ac.agl(env) / 0.3048,
                ac.airspeed_kt(),
                ac.vertical_speed() / crate::flight::FPM,
                ac.heading().to_degrees(),
                al,
                si,
                ctl.throttle,
                ctl.engine_throttle.map(|t| [t[0], t[1]]),
                pilot.plan
            );
        }
    }
    let log = pilot.log.clone();
    Flown { run, ac, log }
}

fn report(f: &Flown) -> String {
    let d = f.run.end.as_ref();
    format!(
        "end {:?} reason {:?} t {:.0} crashed {:?} td {:?} ditch {:?} objectives {:?}\nlog:\n{}",
        d.map(|d| (&d.headline, d.success, d.score.total, d.score.grade)),
        d.map(|d| &d.reason),
        f.run.t,
        f.ac.crashed,
        f.run.touchdown.as_ref().map(|t| (t.landing.sink_fpm, t.landing.along_m, t.landing.side_m, t.ias_kt, t.on_runway)),
        f.run.ditch.map(|d| (d.verdict, d.pitch_deg, d.sink_fpm, d.speed_kt)),
        d.map(|d| d.objectives.clone()),
        f.log.join("\n")
    )
}

fn assert_won(f: &Flown) {
    let d = f.run.end.as_ref().unwrap_or_else(|| panic!("mission did not end: {}", report(f)));
    assert!(d.success, "lost: {}", report(f));
    eprintln!("WON {}: {} — score {} ({}, {:?})", f.run.m.id, d.headline, d.score.total, d.score.grade, d.score.medal);
}

// ---------------------------------------------------------------- definitions

#[test]
fn every_mission_loads_and_validates() {
    let all = defs::all();
    assert_eq!(all.len(), defs::SOURCES.len(), "a mission failed to parse (see stderr)");
    assert!(all.len() >= 10);
    let mut ids: Vec<&str> = all.iter().map(|m| m.id.as_str()).collect();
    ids.sort();
    ids.dedup();
    assert_eq!(ids.len(), all.len(), "duplicate ids");
    for m in all {
        assert!(m.aircraft_index().is_some(), "{}", m.id);
        assert!(!m.history.is_empty() && m.history.starts_with("Inspired by"), "{}: neutral 'Inspired by' history note", m.id);
        assert!(!m.failures.is_empty() && !m.tips.is_empty(), "{}", m.id);
        // The target's geometry is consistent (threshold + course).
        assert!(m.target.length_m > 900.0 && (0.0..360.0).contains(&m.target.course_deg), "{}", m.id);
        // Fictional: no airline names / flight numbers in the text.
        for t in [&m.situation, &m.history, &m.tagline] {
            for bad in [
                "Flight 1",
                "Airways",
                "Airlines",
                "US Air",
                "Air France",
                "Swissair",
                "Qantas",
                "United",
                "Delta",
                "Aloha",
                "Birgenair",
                "Air Canada",
                "Air Transat",
                "British Airways",
            ] {
                assert!(!t.contains(bad), "{}: real operator / flight in \"{t}\"", m.id);
            }
        }
    }
}

#[test]
fn invalid_missions_are_rejected() {
    let ok = defs::SOURCES.iter().find(|s| s.0 == "river").unwrap().1;
    assert!(Mission::parse(ok).is_ok());
    for (from, to, why) in [
        ("\"meridian190\"", "\"nosuchjet\"", "aircraft"),
        ("\"fail\": \"bird\"", "\"fail\": \"wings\"", "failure"),
        ("\"pa\": \"brace\"", "\"pa\": \"karaoke\"", "pa clip"),
        ("\"after\": \"brace\"", "\"after\": \"nothing\"", "event ref"),
        ("\"required\": true", "\"required\": false", "required"),
    ] {
        let bad = ok.replacen(from, to, 1);
        assert_ne!(bad, ok, "{why}: pattern not found");
        assert!(Mission::parse(&bad).is_err(), "{why} should be rejected");
    }
}

// ---------------------------------------------------------------- trigger engine

const TRIGGERS: &str = r#"{
  "id": "t", "title": "T", "situation": "S", "aircraft": "meridian320",
  "target": { "airport": "TEST", "runway": "36", "course_deg": 0, "length_m": 3000, "width_m": 45, "elev_ft": 0, "lat": 0, "lon": 0 },
  "start": { "x_nm": -10, "alt_ft": 6000, "kias": 220 },
  "events": [
    { "id": "a", "when": { "t": 2 }, "do": [ { "note": "ATC: first" } ] },
    { "id": "b", "when": { "after": "a", "delay": 3 }, "do": [ { "pa": "brace" } ] },
    { "id": "c", "when": { "alt_below_ft": 5800 }, "do": [ { "note": "low" } ] },
    { "id": "d", "when": { "after": "b", "player": "speedbrakes" }, "do": [ { "checkpoint": "sb" } ] },
    { "id": "e", "when": { "dist_below_nm": 8 }, "do": [ { "ap_off": true } ] }
  ],
  "objectives": [
    { "id": "o1", "text": "Speed brakes within 4 s of b", "kind": "player", "act": "speedbrakes", "after": "b", "within_s": 4, "points": 10 },
    { "id": "o2", "text": "Below 5,000", "kind": "alt_below", "ft": 5000, "required": true }
  ]
}"#;

#[test]
fn triggers_fire_in_order_and_objectives_time_out() {
    let m: &'static Mission = Box::leak(Box::new(Mission::parse(TRIGGERS).unwrap()));
    let def = find("meridian320").unwrap();
    let env = TestEnv { elev: 0.0, water: Box::new(|_| false), wind: Vec3::ZERO };
    let strips = vec![MissionRun::synthetic_strip(m, Vec2::ZERO)];
    let mut run = MissionRun::new(m);
    let mut ac = Aircraft::airborne(&def, vec3(0.0, 1000.0, 0.0), 0.0, 100.0);
    let mut ctl = Controls::default();
    let step = |run: &mut MissionRun, ac: &mut Aircraft, ctl: &mut Controls, secs: f32| {
        let mut t = 0.0;
        while t < secs {
            let mut ctx = Ctx { def: &def, ac, ctl, env: &env, strips: &strips, severity: None, mayday: false, dt: 0.1 };
            run.step(&mut ctx);
            t += 0.1;
        }
    };
    step(&mut run, &mut ac, &mut ctl, 0.05);
    // Placed 10 NM south of the threshold, heading north, at 6,000 ft.
    assert!((ac.pos.z - 10.0 * 1852.0).abs() < 1.0 && ac.pos.x.abs() < 1.0, "{:?}", ac.pos);
    assert!((ac.pos.y - 6000.0 * 0.3048).abs() < 1.0 && ac.heading().abs() < 0.01);
    step(&mut run, &mut ac, &mut ctl, 2.1);
    assert!(run.fired[0].is_some() && run.fired[1].is_none());
    assert_eq!(run.lines.last().map(|l| (l.who, l.text.as_str())), Some(("ATC", "first")));
    step(&mut run, &mut ac, &mut ctl, 3.1);
    assert!(run.fired[1].is_some(), "after + delay");
    assert_eq!((run.pa_seq, run.pa.as_str()), (1, "brace"));
    assert!(run.requests.contains(&Request::Pa("brace".into())));
    // Objective o1 is visible now and times out after 4 s without speed brakes.
    assert!(run.obj[0].visible && run.obj[0].done.is_none());
    step(&mut run, &mut ac, &mut ctl, 4.5);
    assert!(run.obj[0].failed);
    // Altitude and distance triggers.
    assert!(run.fired[2].is_none());
    ac.pos.y = 5700.0 * 0.3048;
    step(&mut run, &mut ac, &mut ctl, 0.2);
    assert!(run.fired[2].is_some());
    assert!(run.fired[4].is_none());
    ac.pos.z = 7.5 * 1852.0;
    step(&mut run, &mut ac, &mut ctl, 0.2);
    assert!(run.fired[4].is_some() && run.requests.contains(&Request::ApOff));
    // Player-action trigger.
    ctl.spoilers = 1.0;
    step(&mut run, &mut ac, &mut ctl, 0.2);
    assert!(run.fired[3].is_some() && run.checkpoints == 1);
    // The required objective ends the mission.
    assert!(!run.finished());
    ac.pos.y = 4900.0 * 0.3048;
    step(&mut run, &mut ac, &mut ctl, 0.2);
    let d = run.end.as_ref().expect("ended");
    assert!(d.success, "{d:?}");
    assert_eq!(d.objectives.iter().filter(|o| o.1).count(), 1);
    // o1 failed: no procedure points.
    assert_eq!(d.score.procedures, 0.0);
}

#[test]
fn start_latlon_follows_the_runway_frame() {
    let m = defs::by_id("decompression").unwrap();
    // 40 NM before the threshold on a 257.6° course: east-north-east of it.
    let (lat, lon) = start_latlon(m, (m.target.lat, m.target.lon));
    let dn = (lat - m.target.lat) * 111_320.0;
    let de = (lon - m.target.lon) * 111_320.0 * m.target.lat.to_radians().cos();
    let d = (dn * dn + de * de).sqrt() / 1852.0;
    assert!((d - 40.0).abs() < 0.3, "{d}");
    let brg = de.atan2(dn).to_degrees().rem_euclid(360.0);
    assert!((brg - 77.6).abs() < 1.0, "{brg}");
}

// ---------------------------------------------------------------- physics hooks

/// A water landing at the textbook attitude is survivable (the wreck then
/// floats on the debris model, debris.rs); nose-first / hard / banked entries are not.
#[test]
fn ditching_attitude_decides_survival() {
    let def = find("meridian320").unwrap();
    let env = TestEnv { elev: 0.0, water: Box::new(|_| true), wind: Vec3::ZERO };
    let fly_in = |pitch_deg: f32, bank_deg: f32, sink: f32, kt: f32| {
        let mut ac = Aircraft::airborne(&def, vec3(0.0, 6.0, 0.0), 0.0, kt * crate::flight::KT);
        ac.rot = crate::flight::attitude(0.0, pitch_deg.to_radians(), bank_deg.to_radians());
        let v = ac.vel.length();
        ac.vel = vec3(0.0, -sink, -(v * v - sink * sink).max(1.0).sqrt());
        ac.retract_gear_now();
        let ctl = Controls { flaps: 1.0, gear_down: false, ..Default::default() };
        for _ in 0..600 {
            ac.step(&def, &ctl, &env, DT);
            if ac.crashed.is_some() {
                break;
            }
        }
        let r = ac.crashed.clone().expect("touched the water");
        assert!(r.contains("Ditched"), "{r}");
        let imp = ac.impact.unwrap();
        (ditching::assess(&def, &imp), ditching::classify(&def, &r, &imp, false))
    };
    let (good, sev) = fly_in(10.5, 0.0, 1.2, 128.0);
    assert!(sev.survivable() && good.quality > 0.6, "{good:?}");
    let (nose, sev) = fly_in(-3.0, 0.0, 2.0, 150.0);
    assert!(!sev.survivable(), "nose-first: {nose:?}");
    let (bank, sev) = fly_in(9.0, 18.0, 1.0, 130.0);
    assert!(!sev.survivable(), "banked: {bank:?}");
    let (hard, sev) = fly_in(9.0, 0.0, 9.0, 135.0);
    assert!(!sev.survivable(), "hard: {hard:?}");
}

/// With every hydraulic system lost the A320's surfaces freeze; split
/// throttles still yaw and bank it, either way.
#[test]
fn differential_thrust_yaws_without_hydraulics() {
    let def = find("meridian320").unwrap();
    let env = TestEnv { elev: 0.0, water: Box::new(|_| false), wind: Vec3::ZERO };
    let amb = crate::systems::Ambient::default();
    let turn = |left: f32, right: f32| -> (f32, f32) {
        let mut ac = Aircraft::airborne(&def, vec3(0.0, 1500.0, 0.0), 0.0, 210.0 * crate::flight::KT);
        let mut ctl = crate::flight::trimmed_controls_at(&def, 210.0 * crate::flight::KT, 1500.0);
        ac.sys = crate::systems::Systems::new(&def, false);
        for i in 0..3 {
            crate::systems::set_failure(&mut ac, &def, crate::systems::failures::Failure::new(crate::systems::failures::Kind::Hydraulic, i), true);
        }
        let base = ctl.throttle;
        for k in 0..(15.0 / DT) as usize {
            if k as f32 * DT > 1.0 {
                throttles::set_sides(&def, &mut ctl, base + left, base + right);
            }
            // Once the pressure has gone the stick does nothing.
            if ac.sys.hyd.iter().all(|&h| h < 0.02) {
                ctl.aileron = 1.0;
            }
            crate::systems::step(&mut ac, &def, &ctl, &amb, DT);
            ac.step(&def, &ctl, &env, DT);
        }
        assert!(ac.crashed.is_none());
        (crate::bot::wrap_pi(ac.heading()).to_degrees(), ac.roll().to_degrees())
    };
    let (h0, r0) = turn(0.0, 0.0);
    let (hr, rr) = turn(0.35, -0.35);
    let (hl, rl) = turn(-0.35, 0.35);
    eprintln!("symmetric: hdg {h0:.1} roll {r0:.1}; left>right: hdg {hr:.1} roll {rr:.1}; right>left: hdg {hl:.1} roll {rl:.1}");
    assert!(h0.abs() < 4.0, "the frozen controls fly straight on symmetric thrust: {h0}");
    assert!(hr > h0 + 8.0 && rr > 5.0, "more left thrust turns right: hdg {hr} roll {rr}");
    assert!(hl < h0 - 8.0 && rl < -5.0, "more right thrust turns left: hdg {hl} roll {rl}");
}

#[test]
fn split_lever_keys_move_one_side() {
    let def = find("galaxy").unwrap();
    let mut ctl = Controls { throttle: 0.5, ..Default::default() };
    assert!(throttles::can_split(&def) && !throttles::can_split(&find("kestrel").unwrap()));
    throttles::keys(&def, &mut ctl, 0.0, (1.0, 0.0), 0.5);
    let t = ctl.engine_throttle.unwrap();
    let left: Vec<usize> = (0..4).filter(|&i| throttles::side(&def, i) > 0).collect();
    assert_eq!(left.len(), 2);
    for i in 0..4 {
        let want = if left.contains(&i) { 0.8 } else { 0.5 };
        assert!((t[i] - want).abs() < 1e-5, "engine {i}: {}", t[i]);
        assert!((ctl.lever(i) - want).abs() < 1e-5);
    }
    // The common keys keep the split.
    throttles::keys(&def, &mut ctl, -0.1, (0.0, 0.0), 0.1);
    assert!((ctl.lever(left[0]) - 0.7).abs() < 1e-5);
    assert!((ctl.throttle - 0.55).abs() < 1e-4, "mean {}", ctl.throttle);
}

// ---------------------------------------------------------------- missions flown

#[test]
fn won_v1_rejected_takeoff() {
    let m = defs::by_id("v1").unwrap();
    let f = fly("v1", &land_env(m), 240.0);
    assert_won(&f);
    let d = f.run.end.as_ref().unwrap();
    // The correct decision: idle within 3 s, fire handle.
    assert!(d.objectives.iter().all(|o| o.1), "{:?}", d.objectives);
}

#[test]
fn won_decompression_emergency_descent() {
    let m = defs::by_id("decompression").unwrap();
    let f = fly("decompression", &land_env(m), 600.0);
    assert_won(&f);
    assert!(f.ac.sys.cabin.pax_masks && f.ac.sys.cabin.peak > 30_000.0 * 0.3048);
}

#[test]
fn won_bellyland() {
    let m = defs::by_id("bellyland").unwrap();
    let f = fly("bellyland", &land_env(m), 900.0);
    assert_won(&f);
    assert!(f.ac.crashed.as_deref().is_some_and(|r| r.to_lowercase().contains("gear")), "{:?}", f.ac.crashed);
}

#[test]
fn won_river_ditching() {
    let m = defs::by_id("river").unwrap();
    // A wide river ahead: water beyond 4 NM east of the threshold.
    let c = m.target.course_deg.to_radians();
    let dir = vec2(c.sin(), -c.cos());
    let env = TestEnv { elev: 0.0, water: Box::new(move |p| p.dot(dir) > 3.2 * 1852.0), wind: Vec3::ZERO };
    let f = fly("river", &env, 600.0);
    assert_won(&f);
    assert!(f.run.ditch.is_some_and(|d| d.severity.survivable()), "{}", report(&f));
}

#[test]
fn won_microburst_escape() {
    let m = defs::by_id("microburst").unwrap();
    let f = fly("microburst", &land_env(m), 400.0);
    assert_won(&f);
    assert!(f.run.shear_seq > 0, "the windshear warning sounded");
}

#[test]
fn won_pitot_pitch_and_power() {
    let m = defs::by_id("pitot").unwrap();
    let f = fly("pitot", &land_env(m), 600.0);
    assert_won(&f);
}

#[test]
fn won_smoke_diversion() {
    let m = defs::by_id("smoke").unwrap();
    let f = fly("smoke", &land_env(m), 1100.0);
    assert_won(&f);
}

#[test]
fn won_uncontained_heavy_landing() {
    let m = defs::by_id("uncontained").unwrap();
    let f = fly("uncontained", &land_env(m), 1500.0);
    assert_won(&f);
}

#[test]
fn won_deadstick_glide() {
    let m = defs::by_id("deadstick").unwrap();
    let f = fly("deadstick", &land_env(m), 1200.0);
    assert_won(&f);
}

#[test]
fn won_hydraulics_thrust_only() {
    let m = defs::by_id("hydraulics").unwrap();
    let f = fly("hydraulics", &land_env(m), 1200.0);
    assert_won(&f);
}

#[test]
fn a_crash_is_a_loss() {
    // Do nothing in the river mission: the glide ends in the ground short of everything.
    let m = defs::by_id("river").unwrap();
    let def = find(&m.aircraft).unwrap();
    let env = land_env(m);
    let strips = vec![MissionRun::synthetic_strip(m, Vec2::ZERO)];
    let mut run = MissionRun::new(m);
    let mut ac = Aircraft::airborne(&def, vec3(0.0, 1000.0, 0.0), 0.0, 100.0);
    let mut ctl = Controls::default();
    let amb = crate::systems::Ambient::default();
    let mut sev = None;
    for _ in 0..(400.0 / DT) as usize {
        if run.started && ac.crashed.is_none() {
            ctl.elevator = 0.4;
            crate::systems::step(&mut ac, &def, &ctl, &amb, DT);
            ac.step(&def, &ctl, &env, DT);
        }
        if let (Some(r), Some(imp), None) = (&ac.crashed, &ac.impact, sev) {
            sev = Some(ditching::classify(&def, r, imp, false));
        }
        let mut ctx = Ctx { def: &def, ac: &mut ac, ctl: &mut ctl, env: &env, strips: &strips, severity: sev, mayday: false, dt: DT };
        run.step(&mut ctx);
        if run.finished() {
            break;
        }
    }
    let d = run.end.as_ref().expect("ended");
    assert!(!d.success && d.score.medal == score::Medal::None, "{d:?}");
}

#[test]
fn won_flapjam_high_speed_landing() {
    let m = defs::by_id("flapjam").unwrap();
    let f = fly("flapjam", &land_env(m), 900.0);
    assert_won(&f);
}
