//! End-to-end combat tests on flat ground (no scenery): AI against AI
//! through the real flight model, the authoritative hit path, events.

use super::*;
use glam::{Vec2, Vec3};

struct Flat;
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
}

/// Run an AI-vs-AI fight; returns (seconds to the first weapons kill, feed).
fn fight(a: &str, b: &str, da: ai::Difficulty, db: ai::Difficulty, secs: f32) -> (Option<f32>, Combat) {
    let mut c = Combat::new(Rules::default());
    let cfg = setup::Config { format: setup::Format::OneVOne, player: a.into(), opponent: b.into(), difficulty: db, autopilot: false };
    let mut df = dogfight_stub(cfg.clone());
    let ia = c.def_index(a).unwrap();
    let ib = c.def_index(b).unwrap();
    let jet = c.defs[ia].engine_kind == crate::flight::EngineKind::Jet;
    let (alt, sep) = if jet { (5500.0, 9000.0) } else { (2600.0, 3800.0) };
    df.cfg.difficulty = da;
    let ua = df.spawn(&mut c, ia, 0, "Blue", Vec3::new(0.0, alt, sep * 0.5), 0.0);
    df.cfg.difficulty = db;
    let ub = df.spawn(&mut c, ib, 1, "Red", Vec3::new(400.0, alt, -sep * 0.5), std::f32::consts::PI);
    let _ = (ua, ub);
    // Fixed structural-strength seeds: the outcome must not depend on how many
    // aircraft were built before (flight::next_seed is a running counter).
    for (k, u) in c.units.iter_mut().enumerate() {
        u.ai.as_mut().unwrap().ac.dmg.seed = 0x5EED_u32.wrapping_add(0x9E37_79B9_u32.wrapping_mul(k as u32 + 1));
    }
    let dt = 1.0 / 120.0;
    let mut t = 0.0;
    let mut first = None;
    while t < secs {
        c.step(None, &Flat, dt);
        t += dt;
        // No aircraft may fly into the ground while it is still under control.
        for u in &c.units {
            let a = u.ai.as_ref().unwrap();
            if let Some(why) = &a.ac.crashed {
                assert!(!matches!(u.life, Life::Alive), "{} crashed while alive: {why} at {t:.1} s", u.callsign);
                assert!(u.last_hit.is_some() || u.bd.is_doomed() || a.ac.dmg.failure.is_some(), "{} flew into the ground: {why}", u.callsign);
            }
        }
        if first.is_none() && c.units.iter().any(|u| u.kills > 0) {
            first = Some(t);
        }
        if first.is_some_and(|f| t > f + 5.0) {
            break;
        }
        c.audio.clear();
        c.fx.clear();
        c.debris_fx.clear();
    }
    (first, c)
}

fn dogfight_stub(cfg: setup::Config) -> setup::Dogfight {
    let mut c = Combat::new(Rules::default());
    setup::Dogfight::start(&mut c, cfg, None, Vec2::ZERO, 0.0, 0.0);
    c.dogfight.take().unwrap()
}

#[test]
fn ai_spitfires_fight_to_a_gun_kill_without_hitting_the_ground() {
    let (t, c) = fight("peregrine", "peregrine", ai::Difficulty::Ace, ai::Difficulty::Rookie, 300.0);
    for f in &c.feed {
        eprintln!("[{:6.1}] {}", f.t, f.text);
    }
    for u in &c.units {
        eprintln!("{}: {:?}, kills {}, damage taken {:.0}, rounds left {}", u.callsign, u.life, u.kills, u.bd.total, u.ammo(None));
    }
    let t = t.expect("a kill within 5 minutes");
    eprintln!("first kill after {t:.1} s");
    let shot: Vec<&Unit> = c.units.iter().filter(|u| !u.alive()).collect();
    assert!(!shot.is_empty());
    assert!(shot.iter().any(|u| u.last_hit.is_some()), "killed by weapons");
}

#[test]
fn ai_f16s_fight_with_missiles_and_guns() {
    let (t, c) = fight("lancer", "lancer", ai::Difficulty::Ace, ai::Difficulty::Average, 300.0);
    for f in &c.feed {
        eprintln!("[{:6.1}] {}", f.t, f.text);
    }
    let launches = c.outbox.iter().filter(|e| matches!(e, CombatEvent::MissileLaunch { .. })).count();
    for u in &c.units {
        eprintln!(
            "{}: {:?}, kills {}, damage {:.0}, gun {}, AIM-9 {}, AIM-120 {}, flares {}",
            u.callsign,
            u.life,
            u.kills,
            u.bd.total,
            u.ammo(None),
            u.missiles_left(MissileKind::Aim9),
            u.missiles_left(MissileKind::Aim120),
            u.flares
        );
    }
    eprintln!("missile launches {launches}, first kill {t:?}");
    assert!(t.is_some(), "a kill within 5 minutes");
}

#[test]
fn hits_resolve_on_the_authority_and_replay_on_clients() {
    // A Spitfire fires a 1 s burst into a target held 200 m ahead.
    let mut c = Combat::new(Rules::default());
    let mut df = dogfight_stub(setup::Config::default());
    let p = c.def_index("peregrine").unwrap();
    let a = df.spawn(&mut c, p, 0, "A", Vec3::new(0.0, 2000.0, 0.0), 0.0);
    let b = df.spawn(&mut c, p, 1, "B", Vec3::new(0.0, 2000.0, -200.0), 0.0);
    let dt = 1.0 / 120.0;
    for _ in 0..120 {
        // Hold both on the same course (no AI) and keep the trigger pressed.
        for u in &mut c.units {
            let ai = u.ai.as_mut().unwrap();
            ai.passive = true;
            ai.ac.vel = Vec3::new(0.0, 0.0, -140.0);
            ai.ac.rot = glam::Quat::IDENTITY * glam::Quat::from_rotation_y(std::f32::consts::PI);
        }
        c.units[0].trig.guns = true;
        c.units[0].master = Master::Gun;
        c.step(None, &Flat, dt);
        c.units[0].trig.guns = true;
    }
    let hits = c.outbox.iter().filter(|e| matches!(e, CombatEvent::Hit { target, .. } if *target == b)).count();
    let fired: u32 = c.outbox.iter().map(|e| if let CombatEvent::Fire { shooter, rounds, .. } = e { if *shooter == a { *rounds as u32 } else { 0 } } else { 0 }).sum();
    eprintln!("burst: {fired} rounds batched in Fire events, {hits} hits");
    assert!(fired >= 50, "{fired}");
    assert!(hits > 5, "{hits}");
    let tb = c.unit(b).unwrap();
    assert!(tb.bd.total > 0.0);
    // A client applies the server's Damage events and gets the same state.
    let mut cl = Combat::new(Rules { authority: Authority::Client, ..Rules::default() });
    let mut df2 = dogfight_stub(setup::Config::default());
    df2.spawn(&mut cl, p, 0, "A", Vec3::new(0.0, 2000.0, 0.0), 0.0);
    df2.spawn(&mut cl, p, 1, "B", Vec3::new(0.0, 2000.0, -200.0), 0.0);
    let batch = CombatEvent::encode_batch(&c.outbox);
    for e in CombatEvent::decode_batch(&batch).unwrap() {
        if e.authoritative() {
            cl.apply(&e, None);
        }
    }
    let sb = &c.unit(b).unwrap().bd;
    let cb = &cl.unit(b).unwrap().bd;
    for k in 0..sb.hp.len() {
        assert!((sb.hp[k] - cb.hp[k]).abs() <= sb.max[k] / 255.0 + 1e-3, "part {k}: {} vs {}", sb.hp[k], cb.hp[k]);
    }
    assert_eq!(c.unit(b).unwrap().ai.as_ref().unwrap().ac.dmg.lost, cl.unit(b).unwrap().ai.as_ref().unwrap().ac.dmg.lost);
    // Combat off: triggers do nothing.
    let mut off = Combat::new(Rules { enabled: false, ..Rules::default() });
    let mut df3 = dogfight_stub(setup::Config::default());
    df3.spawn(&mut off, p, 0, "A", Vec3::new(0.0, 2000.0, 0.0), 0.0);
    off.units[0].trig.guns = true;
    off.step(None, &Flat, dt);
    assert!(off.rounds.is_empty());
}

#[test]
#[ignore]
fn debug_ai_trace() {
    let id = std::env::var("AC").unwrap_or("peregrine".into());
    let (_, c) = {
        let mut c = Combat::new(Rules::default());
        let mut df = dogfight_stub(setup::Config::default());
        let i = c.def_index(&id).unwrap();
        df.cfg.difficulty = ai::Difficulty::Rookie;
        df.spawn(&mut c, i, 0, "A", Vec3::new(0.0, 2600.0, 1900.0), 0.0);
        df.spawn(&mut c, i, 1, "B", Vec3::new(400.0, 2600.0, -1900.0), std::f32::consts::PI);
        let dt = 1.0 / 120.0;
        for k in 0..(120 * 60) {
            c.step(None, &Flat, dt);
            if k % 60 == 0 {
                for u in &c.units {
                    let a = u.ai.as_ref().unwrap();
                    let ac = &a.ac;
                    eprintln!(
                        "t {:5.1} {} {:?} mode {:?} alt {:5.0} ias {:4.0} kt pitch {:5.1} roll {:6.1} g {:4.1} a {:4.1} | elev {:5.2} ail {:5.2} thr {:.2}",
                        k as f32 * dt,
                        u.callsign,
                        u.life,
                        a.brain.mode,
                        ac.pos.y,
                        ac.airspeed_kt(),
                        ac.pitch().to_degrees(),
                        ac.roll().to_degrees(),
                        ac.g_load(),
                        ac.alpha().to_degrees(),
                        a.ctl.elevator,
                        a.ctl.aileron,
                        a.ctl.throttle
                    );
                }
            }
            if c.units.iter().any(|u| u.ai.as_ref().unwrap().ac.crashed.is_some()) {
                break;
            }
        }
        (0, c)
    };
    for f in &c.feed {
        eprintln!("{}", f.text);
    }
}

#[test]
#[ignore]
fn debug_signs() {
    for id in ["peregrine", "lancer"] {
        let defs = crate::flight::catalogue();
        let def = defs.iter().find(|d| d.id == id).unwrap();
        for (e, a, r) in [(0.3, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 0.3), (0.0, 1.0, 0.0)] {
            let mut ac = Aircraft::airborne(def, Vec3::new(0.0, 2600.0, 0.0), 0.0, 140.0);
            ac.sys = crate::systems::Systems::new(def, false);
            let ctl = Controls { elevator: e, aileron: a, rudder: r, throttle: 0.9, gear_down: false, ..Default::default() };
            for _ in 0..60 {
                ac.step(def, &ctl, &Flat, 1.0 / 120.0);
            }
            let up = ac.rot * Vec3::Y;
            eprintln!(
                "{id} elev {e} ail {a} rud {r}: beta {:.1} roll {:.1} pitch {:.1} omega {:?} g {:.2} alpha {:.1} up {:?} right {:?}",
                ac.beta().to_degrees(),
                ac.roll().to_degrees(),
                ac.pitch().to_degrees(),
                ac.omega,
                ac.g_load(),
                ac.alpha().to_degrees(),
                up,
                ac.right()
            );
        }
    }
}

/// Long AI fights with respawns (F-16 and Spitfire): nobody flies into the
/// ground while still in control. `cargo test --release long_fights -- --ignored`.
#[test]
#[ignore]
fn long_fights_never_hit_the_ground_in_control() {
    let only = std::env::var("LONG_ONLY").ok();
    for (id, d) in [("lancer", ai::Difficulty::Ace), ("lancer", ai::Difficulty::Veteran), ("peregrine", ai::Difficulty::Ace)] {
        if only.as_deref().is_some_and(|o| o != id) {
            continue;
        }
        let mut c = Combat::new(Rules::default());
        let mut df = dogfight_stub(setup::Config { player: id.into(), opponent: id.into(), difficulty: d, ..Default::default() });
        let i = c.def_index(id).unwrap();
        let alt = if id == "lancer" { 5500.0 } else { 2600.0 };
        df.spawn(&mut c, i, 0, "Blue", Vec3::new(0.0, alt, 4000.0), 0.0);
        df.spawn(&mut c, i, 1, "Red", Vec3::new(400.0, alt, -4000.0), std::f32::consts::PI);
        c.dogfight = Some(df);
        let dt = 1.0 / 120.0;
        let mut bad = 0;
        let trace = std::env::var("LONG_TRACE").ok().and_then(|v| v.parse::<f32>().ok());
        for k in 0..(120 * 900) {
            c.step(None, &Flat, dt);
            if trace.is_some_and(|t0| k as f32 * dt > t0 && k % 30 == 0) {
                for u in &c.units {
                    let a = u.ai.as_ref().unwrap();
                    let ac = &a.ac;
                    eprintln!(
                        "fuel {:6.0} run {} t {:6.1} {} {:?} alt {:5.0} vy {:5.0} ias {:4.0} pitch {:5.1} roll {:6.1} g {:4.1} a {:4.1} elev {:5.2} ail {:5.2}",
                        ac.sys.fuel_total(),
                        ac.engine_running(0),
                        k as f32 * dt,
                        u.callsign,
                        a.brain.mode,
                        ac.pos.y,
                        ac.vel.y,
                        ac.airspeed_kt(),
                        ac.pitch().to_degrees(),
                        ac.roll().to_degrees(),
                        ac.g_load(),
                        ac.alpha().to_degrees(),
                        a.ctl.elevator,
                        a.ctl.aileron
                    );
                }
            }
            for u in &c.units {
                let a = u.ai.as_ref().unwrap();
                if a.ac.crashed.is_some()
                    && a.ac.sys.fuel_total() > 1.0
                    && u.last_hit.is_none()
                    && !u.bd.is_doomed()
                    && a.ac.dmg.failure.is_none()
                    && matches!(u.life, Life::Dead(s) if s < dt * 1.5)
                {
                    bad += 1;
                    eprintln!("{id} {d:?}: {} crashed in control at {:.0} s: {:?} (mode {:?})", u.callsign, k as f32 * dt, a.ac.crashed, a.brain.mode);
                }
            }
            c.audio.clear();
            c.fx.clear();
            c.debris_fx.clear();
        }
        eprintln!("{id} {d:?}: feed {:?}", c.feed.iter().map(|f| f.text.clone()).collect::<Vec<_>>());
        assert_eq!(bad, 0);
    }
}
