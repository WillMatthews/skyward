//! Ground-handling tests: tyre curves, braking distances against published
//! figures, hydroplaning onset, tiller turn radius, taildragger ground loops,
//! pushback, autobrake / RTO / brake heat, and the bot's taxi at EGLL.

use super::*;
use crate::flight::{self, Aircraft, Controls, Environment, KT};
use glam::{Vec2, Vec3};

pub const DT: f32 = 1.0 / 120.0;

/// Flat ground at y = 0 with one surface everywhere (or paved only near `paved` segments).
#[derive(Default)]
pub struct Env {
    pub surface: Surface,
    pub wind: Vec3,
    /// Paved strips (a, b, half width); elsewhere grass. Empty = `surface` everywhere.
    pub paved: Vec<(Vec2, Vec2, f32)>,
}

impl Environment for Env {
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
    fn surface(&self, xz: Vec2) -> Surface {
        if self.paved.is_empty() || self.paved.iter().any(|(a, b, hw)| surface::seg_dist(xz, *a, *b) <= *hw) {
            self.surface
        } else {
            Surface::new(Ground::Grass, self.surface.contam)
        }
    }
}

pub fn def(id: &str) -> crate::flight::AircraftDef {
    flight::find(id).unwrap_or_else(|| panic!("no aircraft {id}"))
}

pub fn gs(a: &Aircraft) -> f32 {
    Vec2::new(a.vel.x, a.vel.z).length()
}

pub fn run(a: &mut Aircraft, d: &crate::flight::AircraftDef, env: &dyn Environment, secs: f32, mut f: impl FnMut(&Aircraft, f32) -> Controls) {
    let n = (secs / DT).round() as usize;
    for k in 0..n {
        let c = f(a, k as f32 * DT);
        a.step(d, &c, env, DT);
        if a.crashed.is_some() {
            break;
        }
    }
}

#[test]
fn tyre_force_curves() {
    // Normalised curves: zero at no slip, peak 1 at the peak slip, falling to the sliding value.
    for slide in [0.45f32, 0.75] {
        assert_eq!(tyre::curve(0.0, slide), 0.0);
        assert!((tyre::curve(1.0, slide) - 1.0).abs() < 1e-6);
        let mut prev = 0.0;
        for k in 1..=10 {
            let v = tyre::curve(k as f32 * 0.1, slide);
            assert!(v > prev, "rising branch");
            prev = v;
        }
        let locked = tyre::fx_coef(-1.0, slide);
        assert!((locked + slide).abs() < 0.02, "locked wheel {locked} vs slide {slide}");
        assert!(tyre::fx_coef(-0.3, slide) < 0.0 && tyre::fx_coef(-0.3, slide) > -1.0);
    }
    // Surface friction: dry > wet > snow > ice; grass rolls harder than asphalt.
    let mu = |s: Surface| s.friction(30.0, 200.0, true).0;
    let dry = mu(Surface::DRY);
    let wet = mu(Surface::new(Ground::Asphalt, Contam::Wet(1.0)));
    let snow = mu(Surface::new(Ground::Asphalt, Contam::Snow(20.0)));
    let ice = mu(Surface::new(Ground::Asphalt, Contam::Ice { wet: true }));
    eprintln!("peak μ at 58 kt: dry {dry:.2}, wet {wet:.2}, snow {snow:.2}, wet ice {ice:.2}");
    assert!(dry > 0.7 && wet < dry * 0.8 && snow < wet && ice < 0.08);
    assert!(Surface::new(Ground::Grass, Contam::Dry).rolling().1 > 0.02);
    // Wet friction falls with speed.
    let w = |kt: f32| Surface::new(Ground::Asphalt, Contam::Wet(1.0)).friction(kt * KT, 200.0, true).0;
    assert!(w(120.0) < w(40.0) * 0.8, "wet μ 40 kt {:.2} 120 kt {:.2}", w(40.0), w(120.0));
}

#[test]
fn hydroplaning_onset_follows_horne() {
    // Standing water (6 mm): μ collapses at ≈ 9·√p kt for a spinning tyre, 7.7·√p for a stopped one.
    for psi in [30.0f32, 120.0, 200.0] {
        let s = Surface::new(Ground::Asphalt, Contam::Wet(6.0));
        let onset = |spinning: bool| {
            let mut kt = 10.0;
            while kt < 400.0 {
                let (_, _, hp) = s.friction(kt * KT, psi, spinning);
                if hp > 0.5 {
                    return kt;
                }
                kt += 0.5;
            }
            kt
        };
        let (spin, stop) = (onset(true), onset(false));
        eprintln!("{psi:.0} psi: hydroplaning at {spin:.0} kt spinning (9√p = {:.0}), {stop:.0} kt stopped (7.7√p = {:.0})", 9.0 * psi.sqrt(), 7.7 * psi.sqrt());
        assert!((spin / (9.0 * psi.sqrt()) - 1.0).abs() < 0.06);
        assert!((stop / (7.7 * psi.sqrt()) - 1.0).abs() < 0.06);
        // A thin wet film does not plane.
        let (_, _, hp) = Surface::new(Ground::Asphalt, Contam::Wet(0.8)).friction(2.0 * 9.0 * psi.sqrt() * KT, psi, true);
        assert!(hp < 0.05);
    }
}

/// Just above the runway at `kt`, sinking at 1.5 m/s, flaps full, gear down, idle.
pub fn touchdown(d: &crate::flight::AircraftDef, kt: f32) -> (Aircraft, Controls) {
    let mut a = Aircraft::airborne(d, Vec3::new(0.0, d.gear_height + 0.25, 0.0), 0.0, kt * KT);
    let tp = d.trim_point_at(kt * KT, 1.0, 0.0, true);
    a.rot = flight::attitude(0.0, tp.pitch.max(if d.taildragger { 0.0 } else { 2f32.to_radians() }), 0.0);
    a.vel = Vec3::new(0.0, -1.5, -kt * KT);
    a.set_flaps_now(1.0);
    a.extend_gear_now();
    let c = Controls { flaps: 1.0, gear_down: true, trim: tp.trim, ..Default::default() };
    (a, c)
}

/// `cargo test --release diag_rollout -- --ignored --nocapture` (ID=meridian320).
#[test]
#[ignore]
fn diag_rollout() {
    let id = std::env::var("ID").unwrap_or("meridian320".into());
    let d = def(&id);
    let env = Env::default();
    let (mut a, base) = touchdown(&d, d.vref_kt);
    let mut t_g = None;
    run(&mut a, &d, &env, 60.0, |a, t| {
        if a.on_ground && t_g.is_none() {
            t_g = Some(t);
        }
        let rolling = t_g.is_some_and(|t0| t - t0 > 0.3);
        if (t * 4.0).fract() < DT * 4.0 {
            let w: Vec<String> = a
                .ground
                .wheels
                .iter()
                .take(d.gear.len())
                .map(|w| {
                    format!(
                        "[om {:5.1} k {:+.2} a {:+.3} st {:+.3} b {:.2} rel {:.2} lk {} fz {:6.0} T {:4.0}]",
                        w.omega, w.kappa, w.alpha, w.steer, w.brake_cmd, w.release, w.locked as u8, w.load, w.brake_c
                    )
                })
                .collect();
            eprintln!("t {t:5.2} gs {:5.1} kt pitch {:4.1} decel {:4.1} {}", gs(a) / KT, a.pitch().to_degrees(), a.ground.decel, w.join(" "));
        }
        let rev = std::env::var("REV").is_ok() && rolling && gs(a) > 30.0 * KT;
        Controls {
            brakes: if rolling { 1.0 } else { 0.0 },
            spoilers: 1.0,
            reverse: rev,
            throttle: if rev { 1.0 } else { 0.0 },
            rudder: (-2.0 * a.heading()).clamp(-1.0, 1.0),
            ..base
        }
    });
    eprintln!("crashed {:?} stopped at z {:.0}", a.crashed, -a.pos.z);
}

/// `ID=peregrine cargo test --release diag_takeoff -- --ignored --nocapture`
#[test]
#[ignore]
fn diag_takeoff() {
    let id = std::env::var("ID").unwrap_or("peregrine".into());
    let d = def(&id);
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    run(&mut a, &d, &env, 12.0, |a, t| {
        let ias = a.airspeed_kt();
        let el = if d.taildragger && ias > 0.45 * d.vr_kt { (3.0 * (1f32.to_radians() - a.pitch()) - 0.8 * a.rates().y).clamp(-1.0, 0.6) } else { 0.0 };
        let (k, kd) = if d.taildragger { (5.0, 2.0) } else { (2.0, 1.2) };
        let rud = (-k * a.heading() - kd * a.rates().z - 0.05 * a.pos.x).clamp(-1.0, 1.0);
        if (t * 8.0).fract() < DT * 8.0 {
            let w: Vec<String> = a
                .ground
                .wheels
                .iter()
                .take(d.gear.len())
                .map(|w| format!("[om {:5.1} k {:+.3} a {:+.3} st {:+.2} c{} fz {:6.0}]", w.omega, w.kappa, w.alpha, w.steer, w.castering as u8, w.load))
                .collect();
            eprintln!(
                "t {t:5.2} ias {ias:4.0} hdg {:+5.1} r {:+5.1} roll {:+5.1} p {:+5.1} {}",
                a.heading().to_degrees(),
                a.rates().z.to_degrees(),
                a.roll().to_degrees(),
                a.rates().x.to_degrees(),
                w.join(" ")
            );
        }
        Controls { throttle: 1.0, elevator: el, rudder: rud, ..Default::default() }
    });
    eprintln!("crashed {:?}", a.crashed);
}

// ---------------------------------------------------------------- braking

/// Ground roll (m) from touchdown at `kt` to a stop; `ctl` decides the controls.
fn rollout(d: &crate::flight::AircraftDef, env: &Env, kt: f32, mut ctl: impl FnMut(&Aircraft, Controls, f32) -> Controls) -> (Aircraft, f32, f32) {
    let (mut a, base) = touchdown(d, kt);
    let mut touch: Option<(Vec3, f32)> = None;
    let mut t_end = 0.0;
    run(&mut a, d, env, 120.0, |a, t| {
        if a.on_ground && touch.is_none() {
            touch = Some((a.pos, t));
        }
        t_end = t;
        let since = touch.map(|x| t - x.1).unwrap_or(-1.0);
        let c = ctl(a, base, since);
        Controls { rudder: (-2.0 * a.heading() - 0.02 * a.pos.x).clamp(-1.0, 1.0), ..c }
    });
    assert!(a.crashed.is_none(), "{} rollout: {:?}", d.id, a.crashed);
    let (p0, t0) = touch.expect("never touched down");
    (a.clone(), Vec2::new(a.pos.x - p0.x, a.pos.z - p0.z).length(), t_end - t0)
}

#[test]
fn autobrake_landing_distances() {
    // A320 (64 t), dry, ground spoilers, no reverse: FCOM actual landing distance
    // MED ≈ 1,350 m from 50 ft (≈ 1,050 m of it on the ground), LO ≈ 1,800 m.
    let d = def("meridian320");
    let env = Env::default();
    let mut rolls = vec![];
    for sel in [Autobrake::Lo, Autobrake::Med, Autobrake::Max] {
        let mut decels = vec![];
        let (a, roll, secs) = rollout(&d, &env, d.vref_kt, |a, base, since| {
            if since > 0.0 && (40.0..100.0).contains(&(gs(a) / KT)) {
                decels.push(a.ground.decel);
            }
            Controls { spoilers: 1.0, autobrake: sel, ..base }
        });
        let mean = decels.iter().sum::<f32>() / decels.len().max(1) as f32;
        eprintln!("A320 autobrake {}: ground roll {roll:.0} m in {secs:.0} s, mean decel 100→40 kt {mean:.2} m/s², hottest brake {:.0} °C", sel.label(), a.ground.max_brake_c);
        rolls.push((roll, mean));
    }
    let (lo, med, max) = (rolls[0], rolls[1], rolls[2]);
    assert!((850.0..1300.0).contains(&med.0), "MED roll {}", med.0);
    assert!((med.1 - 3.0).abs() < 0.45, "MED decel {}", med.1);
    assert!((lo.1 - 1.7).abs() < 0.4, "LO decel {}", lo.1);
    assert!(lo.0 > med.0 * 1.3 && max.0 < med.0 * 0.85);
}

#[test]
fn rejected_takeoff_stops() {
    // A320 RTO from V1 = 140 kt, MAX (RTO) armed, thrust levers to idle, no
    // reverse (dry-runway certification): stop from V1 ≈ 750–1,000 m; the
    // brakes absorb most of the energy and get hot.
    let d = def("meridian320");
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let mut cut: Option<Vec3> = None;
    run(&mut a, &d, &env, 120.0, |a, _| {
        if cut.is_none() && a.airspeed_kt() >= 140.0 {
            cut = Some(a.pos);
        }
        let thr = if cut.is_some() { 0.0 } else { 1.0 };
        Controls {
            throttle: thr,
            flaps: 0.25,
            autobrake: Autobrake::Max,
            spoilers: if cut.is_some() { 1.0 } else { 0.0 },
            rudder: (-2.0 * a.heading()).clamp(-1.0, 1.0),
            ..Default::default()
        }
    });
    let p = cut.expect("never reached V1");
    let dist = Vec2::new(a.pos.x - p.x, a.pos.z - p.z).length();
    eprintln!("A320 RTO from 140 kt: {dist:.0} m, RTO {} , brakes {:.0} °C (hot: {})", a.ground.ab.rto, a.ground.max_brake_c, a.ground.brakes_hot);
    assert!(a.crashed.is_none() && gs(&a) < 0.2);
    assert!(a.ground.ab.rto, "RTO autobrake did not activate");
    // (Measured from the thrust-lever cut, without the certification's 2 s recognition time.)
    assert!((500.0..950.0).contains(&dist), "RTO stop {dist}");
    assert!(a.ground.brakes_hot && a.ground.max_brake_c > 350.0);
    // Pilot pedal input on an active RTO at low speed disarms it.
    // (A second, back-to-back RTO overheats the brakes until the fusible plugs let the tyres down.)
    let mut b = a.clone();
    let mut cut2 = None;
    for _ in 0..3 {
        let mut cut_at = None;
        run(&mut b, &d, &env, 120.0, |a, _| {
            if cut_at.is_none() && a.airspeed_kt() >= 140.0 {
                cut_at = Some(());
            }
            let thr = if cut_at.is_some() { 0.0 } else { 1.0 };
            Controls {
                throttle: thr,
                flaps: 0.25,
                autobrake: Autobrake::Max,
                spoilers: if cut_at.is_some() { 1.0 } else { 0.0 },
                rudder: (-2.0 * a.heading()).clamp(-1.0, 1.0),
                ..Default::default()
            }
        });
        cut2 = cut_at;
        if b.ground.wheels.iter().any(|w| w.deflated) {
            break;
        }
        // Taxi back slowly (cooling a little), selector cycled.
        run(&mut b, &d, &env, 120.0, |_, _| Controls { brakes: 1.0, ..Default::default() });
    }
    assert!(cut2.is_some());
    // Sitting after the stops, the heat soaks into the wheels: fusible plugs.
    run(&mut b, &d, &env, 600.0, |_, _| Controls { parking_brake: true, ..Default::default() });
    eprintln!("after repeated RTOs: brakes {:.0} °C, rims {:?}", b.ground.max_brake_c, b.ground.wheels.iter().take(3).map(|w| w.rim_c.round()).collect::<Vec<_>>());
    assert!(b.ground.wheels.iter().any(|w| w.deflated), "fusible plugs should have melted");
}

#[test]
fn c172_short_field_landing() {
    // POH: 575 ft (175 m) ground roll at 2,400 lb with maximum braking; the
    // Kestrel flies at 932 kg (2,055 lb): roll ∝ W², ≈ 130 m.
    let d = def("kestrel");
    let env = Env::default();
    let (_, roll, _) = rollout(&d, &env, 50.0, |a, base, since| {
        // Maximum braking without skidding (no anti-skid): ease off a locking wheel.
        let skid = a.ground.wheels.iter().any(|w| w.locked);
        let b = if since > 0.4 { if skid { 0.4 } else { 1.0 } } else { 0.0 };
        Controls { brakes: b, elevator: if since > 0.4 { 1.0 } else { 0.3 }, ..base }
    });
    eprintln!("C172 short-field ground roll from 50 kt: {roll:.0} m (POH 175 m at 2,400 lb → ≈ 128 m at 2,055 lb)");
    assert!((90.0..200.0).contains(&roll), "C172 roll {roll}");
}

#[test]
fn hydroplaning_on_a_flooded_runway() {
    // A320 landing at Vref on 8 mm of standing water: the wheels plane above
    // ≈ 9·√200 = 127 kt (spinning) / 109 kt (not yet spun up), braking is poor
    // until well below it; on a dry runway the same braking stops it far sooner.
    let d = def("meridian320");
    let dry = Env::default();
    let wet = Env { surface: Surface::new(Ground::Asphalt, Contam::Wet(8.0)), ..Default::default() };
    let mut planed = 0.0f32;
    let mut late_spin = false;
    let max_braking = |_: &Aircraft, base: Controls, since: f32| Controls { spoilers: 1.0, brakes: if since > 0.3 { 1.0 } else { 0.0 }, ..base };
    let (_, roll_dry, _) = rollout(&d, &dry, d.vref_kt + 5.0, max_braking);
    let (_, roll_wet, _) = rollout(&d, &wet, d.vref_kt + 5.0, |a, base, since| {
        for w in a.ground.wheels.iter().filter(|w| w.contact) {
            planed = planed.max(w.planing);
            if since > 0.5 && since < 2.0 && w.omega * 0.58 < 0.5 * gs(a) && w.load > 1000.0 {
                late_spin = true;
            }
        }
        max_braking(a, base, since)
    });
    eprintln!("A320 max braking from Vref+5: dry {roll_dry:.0} m, 8 mm water {roll_wet:.0} m (planing {planed:.2}, wheels slow to spin up {late_spin})");
    assert!(roll_wet > roll_dry * 1.4, "wet {roll_wet} vs dry {roll_dry}");
    assert!(planed > 0.5);
}

// ---------------------------------------------------------------- steering

#[test]
fn tiller_turn_radius() {
    // Full tiller at walking pace: A320 published (ACAP, 75° steering) outer
    // wingtip radius ≈ 22 m (21.9).
    for (id, published) in [("meridian320", Some(22.0f32)), ("meridian190", None), ("aerolink", None)] {
        let d = def(id);
        let env = Env::default();
        let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
        let tip = d.hard_points.iter().filter(|h| h.reason == "Wingtip strike").map(|h| h.pos).fold(Vec3::ZERO, |m, p| if p.x.abs() > m.x.abs() { p } else { m });
        // Left tip is the outer one in a right turn.
        let tip = Vec3::new(tip.x.abs(), tip.y, tip.z);
        let mut pts = vec![];
        let jet = d.engine_kind == crate::flight::EngineKind::Jet;
        run(&mut a, &d, &env, 120.0, |a, t| {
            let v = gs(a);
            if t > 40.0 {
                let p = a.pos + a.rot * tip;
                pts.push(Vec2::new(p.x, p.z));
            }
            let thr = if v < 1.3 { if jet { 0.3 } else { 0.25 } } else { 0.0 };
            Controls { throttle: thr, brakes: if v > 2.0 { 0.2 } else { 0.0 }, tiller: if t > 3.0 { 1.0 } else { 0.0 }, ..Default::default() }
        });
        assert!(a.crashed.is_none());
        // Radius: mean distance from the centroid of the traced circle.
        let c = pts.iter().copied().sum::<Vec2>() / pts.len() as f32;
        let r = pts.iter().map(|p| p.distance(c)).sum::<f32>() / pts.len() as f32;
        let span = pts.iter().map(|p| p.distance(c)).fold(0.0f32, f32::max) - pts.iter().map(|p| p.distance(c)).fold(1e9f32, f32::min);
        eprintln!("{id}: full-tiller outer wingtip radius {r:.1} m (published {published:?}), circle spread {span:.1} m");
        if let Some(p) = published {
            assert!((r - p).abs() < 3.0, "{id} turn radius {r}");
        }
    }
}

#[test]
fn rudder_pedals_only_steer_a_few_degrees_on_airliners() {
    let d = def("meridian320");
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    run(&mut a, &d, &env, 30.0, |a, _| Controls { throttle: if gs(a) < 4.0 { 0.3 } else { 0.0 }, rudder: 1.0, ..Default::default() });
    let nose = a.ground.wheels[0].steer.to_degrees();
    eprintln!("A320 full pedal at {:.0} kt: nose wheel {nose:.1}°, turn {:.1}°/s", gs(&a) / KT, a.turn_rate().to_degrees());
    assert!((5.0..7.0).contains(&nose));
    // No hydraulics (green system / NWS lost): the nose wheel casters, the tiller does nothing.
    let mut b = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    b.sys.fx.steering = 0.0;
    run(&mut b, &d, &env, 25.0, |a, _| Controls { throttle: if gs(a) < 4.0 { 0.3 } else { 0.0 }, tiller: 1.0, ..Default::default() });
    eprintln!("  NWS lost: tiller full, heading {:.1}°", b.heading().to_degrees());
    assert!(b.ground.wheels[0].castering && b.heading().to_degrees().abs() < 5.0);
}

/// Rolling at `ms` m/s, the velocity is turned `beta_deg` to the right of the
/// nose (a sideslip: drift at touchdown, a swerve), feet off: the yaw rate
/// (°/s, + = right, toward the velocity) the tyres produce 0.4 s later.
fn sideslip_yaw(id: &str, ms: f32, beta_deg: f32) -> f32 {
    let d = def(id);
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let tail = d.taildragger;
    let mut steps = 0;
    while gs(&a) < ms && steps < 20_000 {
        let rud = (-5.0 * a.heading() - 2.0 * a.rates().z).clamp(-1.0, 1.0);
        a.step(&d, &Controls { throttle: 0.6, rudder: rud, elevator: if tail { 1.0 } else { 0.0 }, ..Default::default() }, &env, DT);
        steps += 1;
    }
    let v = a.vel;
    a.vel = glam::Quat::from_rotation_y(-beta_deg.to_radians()) * v;
    // (Tail wheels unlocked, as when taxiing.)
    run(&mut a, &d, &env, 0.4, |_, _| Controls { elevator: if tail { 1.0 } else { 0.0 }, tail_lock: false, ..Default::default() });
    a.rates().z.to_degrees()
}

#[test]
fn taildraggers_ground_loop() {
    // Directional stability on the wheels at taxi speed (no aero to speak of):
    // with the velocity 8° right of the nose, a tricycle's mains (behind the CG)
    // yaw it toward the velocity or leave it neutral (its steered nose wheel
    // works like a car's front axle); a taildragger's mains (ahead of the CG)
    // yaw it away and the swerve feeds itself: the ground loop.
    let mut out = vec![];
    for id in ["cub", "peregrine", "zlin", "dc3", "kestrel", "extra500", "meridian320", "meridian190"] {
        let r = sideslip_yaw(id, 6.0, 8.0);
        eprintln!("{id}: 8° sideslip at 6 m/s, feet off → yaw rate {r:+.1}°/s ({})", if r < 0.0 { "away: ground loop" } else { "into it: straightens" });
        out.push((id, def(id).taildragger, r));
    }
    let worst_tricycle = out.iter().filter(|o| !o.1).map(|o| o.2).fold(f32::INFINITY, f32::min);
    for (id, tail, r) in out {
        if tail {
            assert!(r < -0.8 && r < worst_tricycle - 0.8, "{id}: taildragger should yaw away from the sideslip ({r})");
        } else {
            assert!(r > -0.3, "{id}: tricycle should not diverge ({r})");
        }
    }
}

#[test]
fn crosswind_weathervanes_into_wind() {
    // Taxiing a DC-3 with the tail wheel unlocked (free castering), 25 kt of wind
    // from the right, feet off: the big fin turns the nose into the wind. With
    // the tail wheel locked it tracks straight.
    let d = def("dc3");
    let env = Env { wind: Vec3::new(-25.0 * KT, 0.0, 0.0), ..Default::default() };
    let mut hdg = vec![];
    for lock in [false, true] {
        let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
        run(&mut a, &d, &env, 25.0, |a, _| Controls { throttle: if gs(a) < 8.0 * KT { 0.35 } else { 0.0 }, elevator: 1.0, aileron: 0.5, tail_lock: lock, ..Default::default() });
        eprintln!("DC-3 taxi, 25 kt from the right, feet off, tail wheel {}: heading {:.0}°", if lock { "locked" } else { "free" }, a.heading().to_degrees());
        assert!(a.crashed.is_none());
        hdg.push(a.heading().to_degrees());
    }
    assert!(hdg[0] > 20.0, "should weathervane right, heading {}", hdg[0]);
    assert!(hdg[1].abs() < hdg[0] * 0.5);
}

// ---------------------------------------------------------------- holding

#[test]
fn parking_brake_and_chocks_hold() {
    let d = def("meridian320");
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let p0 = a.pos;
    run(&mut a, &d, &env, 30.0, |_, _| Controls { throttle: 0.3, parking_brake: true, ..Default::default() });
    let moved = (a.pos - p0).length();
    eprintln!("A320 parking brake against 30 % thrust: moved {moved:.2} m");
    assert!(moved < 0.3);
    // Chocks, brakes off, idle: stays put; without them, idle thrust creeps.
    let mut b = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    b.ground.chocks = true;
    run(&mut b, &d, &env, 30.0, |_, _| Controls { throttle: 0.05, ..Default::default() });
    let mut c = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    run(&mut c, &d, &env, 30.0, |_, _| Controls { throttle: 0.05, ..Default::default() });
    let (mb, mc) = ((b.pos - p0).length(), (c.pos - p0).length());
    eprintln!("A320 idle: chocks in {mb:.2} m, no chocks / brakes {mc:.1} m");
    assert!(mb < 0.3 && mc > 5.0);
}

#[test]
fn autobrake_disarms_on_pedals() {
    let d = def("meridian320");
    let env = Env::default();
    let mut disarmed_at = None;
    let (a, _, _) = rollout(&d, &env, d.vref_kt, |a, base, since| {
        let pedal = if since > 8.0 { 0.8 } else { 0.0 };
        if a.ground.ab.disarmed && disarmed_at.is_none() {
            disarmed_at = Some(since);
        }
        Controls { spoilers: 1.0, autobrake: Autobrake::Med, brakes: pedal, ..base }
    });
    eprintln!("MED autobrake disarmed by the pedals at {disarmed_at:?} s");
    assert!(disarmed_at.is_some_and(|t| t > 7.9 && t < 8.5));
    assert!(!a.ground.ab.active);
}

// ---------------------------------------------------------------- pushback

#[test]
fn pushback_paths() {
    use super::pushback::{Phase, PushDir, Pushback};
    let d = def("meridian320");
    let env = Env::default();
    for dir in [PushDir::Straight, PushDir::TailLeft, PushDir::TailRight] {
        let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
        let mut pb = Pushback::new(&d, &a, dir).unwrap();
        let mut ctl = Controls::default();
        let mut max_v = 0.0f32;
        let mut t = 0.0;
        while t < 400.0 && pb.phase != Phase::Done {
            if matches!(pb.phase, Phase::WaitBrake) {
                ctl.parking_brake = true;
            }
            a.ground.tow = pb.update(&d, &a, &ctl, DT);
            a.step(&d, &ctl, &env, DT);
            assert!(a.crashed.is_none(), "{dir:?}: {:?}", a.crashed);
            max_v = max_v.max(gs(&a));
            t += DT;
        }
        let turned = pb.turned(&a).to_degrees();
        let pushed = pb.pushed(&d, &a);
        eprintln!("A320 pushback {}: {pushed:.0} m, heading {turned:+.0}°, max {:.1} kt, {t:.0} s, tug at {:?}", dir.label(), max_v / KT, pb.tug.pos);
        assert_eq!(pb.phase, Phase::Done);
        assert!(max_v < 2.0, "pushed too fast {max_v}");
        match dir {
            PushDir::Straight => assert!(turned.abs() < 3.0 && pushed > 25.0),
            PushDir::TailLeft => assert!((turned - 90.0).abs() < 15.0, "tail left {turned}"),
            PushDir::TailRight => assert!((turned + 90.0).abs() < 15.0, "tail right {turned}"),
        }
        // Moving backwards while pushed: the aircraft went back (south of its start for a north-facing stand).
        assert!(a.pos.z > 5.0);
    }
    // Parking brake set: the tug cannot move it, and says so.
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let mut pb = Pushback::new(&d, &a, PushDir::Straight).unwrap();
    let ctl = Controls { parking_brake: true, ..Default::default() };
    for _ in 0..(30.0 / DT) as usize {
        a.ground.tow = pb.update(&d, &a, &ctl, DT);
        a.step(&d, &ctl, &env, DT);
    }
    eprintln!("push against the parking brake: moved {:.2} m, '{}'", a.pos.z, pb.msg);
    assert!(a.pos.z.abs() < 0.5 && pb.msg.contains("parking brake"));
}

#[test]
#[ignore]
fn diag_kick() {
    let id = std::env::var("ID").unwrap_or("extra500".into());
    let d = def(&id);
    let env = Env::default();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let tail = d.taildragger;
    run(&mut a, &d, &env, 60.0, |a, _| {
        let rud = (-5.0 * a.heading() - 2.0 * a.rates().z).clamp(-1.0, 1.0);
        Controls { throttle: if a.airspeed_kt() < 30.0 { 0.8 } else { 0.0 }, rudder: rud, elevator: if tail { 1.0 } else { 0.0 }, ..Default::default() }
    });
    a.omega.y -= 20f32.to_radians();
    run(&mut a, &d, &env, 4.0, |a, t| {
        if (t * 8.0).fract() < DT * 8.0 {
            let w: Vec<String> =
                a.ground.wheels.iter().take(d.gear.len()).map(|w| format!("[a {:+.3} st {:+.3} c{} fz {:5.0}]", w.alpha, w.steer, w.castering as u8, w.load)).collect();
            eprintln!(
                "t {t:4.2} gs {:4.1} hdg {:+5.1} r {:+5.1} beta {:+5.1} {}",
                gs(a) / KT,
                a.heading().to_degrees(),
                a.rates().z.to_degrees(),
                a.beta().to_degrees(),
                w.join(" ")
            );
        }
        Controls { elevator: if tail { 1.0 } else { 0.0 }, ..Default::default() }
    });
}

// ---------------------------------------------------------------- bot taxi

struct Field {
    graph: crate::atc::taxi::Graph,
    taxiways: Vec<(Vec<Vec2>, String)>,
    runways: Vec<(String, String, Vec2, Vec2, f32)>,
    holds: Vec<(Vec2, String, String)>,
}

fn field(text: &str) -> Field {
    let p = |s: &str| {
        let (a, b) = s.split_once(',').unwrap();
        Vec2::new(a.parse().unwrap(), b.parse().unwrap())
    };
    let (mut taxiways, mut runways, mut holds) = (Vec::new(), Vec::new(), Vec::new());
    for l in text.lines() {
        let f: Vec<&str> = l.split('\t').collect();
        match f[0] {
            "R" => runways.push((f[1].to_string(), f[2].to_string(), p(f[3]), p(f[4]), f[5].parse().unwrap())),
            "T" => taxiways.push((f[2].split(';').map(p).collect::<Vec<_>>(), f[1].to_string())),
            "H" => holds.push((p(f[3]), f[1].to_string(), f[2].to_string())),
            _ => {}
        }
    }
    Field { graph: crate::atc::taxi::Graph::build(&taxiways), taxiways, runways, holds }
}

/// The bot taxis an A320 from the Terminal 5 apron to the 27R holding point at
/// Heathrow (the real taxiway network): main wheels on the pavement all the
/// way, ≤ ~20 kt on the straights, slow in the turns, stopped at the hold.
#[test]
fn bot_taxi_egll_stays_on_the_pavement() {
    use crate::atc::taxi::{self, RwyBox};
    let f = field(include_str!("../atc/fixtures/egll.taxi"));
    let boxes: Vec<RwyBox> = f.runways.iter().map(|r| RwyBox { a: r.2, b: r.3, width: r.4, name: format!("{}/{}", r.0, r.1) }).collect();
    let rw = f.runways.iter().find(|r| r.1 == "27R").unwrap();
    // Departure from 27R: threshold at the b end, heading west.
    let (thr, dir) = (rw.3, (rw.2 - rw.3).normalize());
    let start = Vec2::new(-1500.0, 0.0);
    let hold = taxi::departure_holds(&f.holds, &format!("{}/{}", rw.0, rw.1), thr, dir)
        .into_iter()
        .filter_map(|(h, _)| f.graph.route(start, h.at, 400.0, &boxes).map(|r| (h, r)))
        .min_by(|a, b| a.1.length.total_cmp(&b.1.length))
        .expect("no route to a 27R hold");
    let (h, route) = hold;
    eprintln!("EGLL T5 → 27R hold {}: {:.0} m via {:?}", h.taxiway, route.length, route.via);
    // Pavement: the taxiways (23 m wide, Code C/D) and the runways.
    let mut paved: Vec<(Vec2, Vec2, f32)> = f.taxiways.iter().flat_map(|(pl, _)| pl.windows(2).map(|w| (w[0], w[1], 11.5)).collect::<Vec<_>>()).collect();
    paved.extend(f.runways.iter().map(|r| (r.2, r.3, r.4 * 0.5)));
    let env = Env { paved: paved.clone(), ..Default::default() };
    let d = def("meridian320");
    let sp = spec::spec(&d);
    let p0 = route.pts[0];
    let h0 = crate::bot::heading_of(route.pts[3.min(route.pts.len() - 1)] - p0);
    // Put the main gear on the first point.
    let (_, mid) = super::pushback::geometry(&d).unwrap();
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, h0, &env);
    let off = a.rot * mid;
    a = Aircraft::on_runway(&d, p0 - Vec2::new(off.x, off.z), h0, &env);
    let mut tx = bot_taxi::Taxi::new(&d, &route.pts, 6.0);
    let mut ctl = Controls::default();
    let (mut t, mut vmax, mut worst, mut max_x) = (0.0f32, 0.0f32, f32::NEG_INFINITY, 0.0f32);
    let mains: Vec<Vec3> = d.gear.iter().filter(|g| g.pos.z < 0.0).map(|g| g.pos).collect();
    let mut corner_v = 0.0f32;
    let mut lat_g = 0.0f32;
    while t < 1500.0 && !tx.done {
        tx.update(&d, &sp, &a, &mut ctl, DT);
        a.step(&d, &ctl, &env, DT);
        assert!(a.crashed.is_none(), "{:?}", a.crashed);
        t += DT;
        vmax = vmax.max(gs(&a));
        max_x = max_x.max(tx.xtrack.abs());
        if a.turn_rate().abs() > 6f32.to_radians() {
            corner_v = corner_v.max(gs(&a));
        }
        if t > 5.0 {
            lat_g = lat_g.max((gs(&a) * a.turn_rate()).abs() / 9.81);
        }
        for m in &mains {
            let p = a.pos + a.rot * *m;
            let q = Vec2::new(p.x, p.z);
            let dist = paved.iter().map(|(a, b, hw)| surface::seg_dist(q, *a, *b) - hw).fold(f32::INFINITY, f32::min);
            worst = worst.max(dist);
        }
    }
    let end = route.pts.last().unwrap();
    let nose = a.pos + a.rot * super::pushback::geometry(&d).unwrap().0;
    let left = Vec2::new(nose.x, nose.z).distance(*end);
    eprintln!(
        "bot taxi: {:.0} s, max {:.1} kt, fastest turning (>6°/s) {:.1} kt, peak lateral {lat_g:.2} g, main gear worst {:+.1} m from the pavement edge, centreline error ≤ {max_x:.1} m, nose {left:.0} m from the hold",
        t,
        vmax / KT,
        corner_v / KT,
        worst
    );
    assert!(tx.done, "did not reach the hold in {t} s ({:.0} m left)", tx.remaining());
    assert!(worst < 0.0, "a main wheel left the pavement by {worst:.1} m");
    assert!(vmax < 22.0 * KT && lat_g < 0.15, "too fast: {vmax} m/s, {lat_g} g");
    assert!(left < 15.0 && gs(&a) < 0.3);
}

#[test]
#[ignore]
fn diag_rto_unsteered() {
    let d = def("meridian320");
    let env = Env { wind: Vec3::new(3.0, 0.0, 0.0), paved: vec![(Vec2::new(0.0, 100.0), Vec2::new(0.0, -4000.0), 25.0)], ..Default::default() };
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &env);
    let mut cut = false;
    run(&mut a, &d, &env, 90.0, |a, t| {
        if !cut && a.airspeed_kt() >= 140.0 {
            cut = true;
        }
        if (t * 2.0).fract() < DT * 2.0 {
            let w: Vec<String> = a
                .ground
                .wheels
                .iter()
                .take(3)
                .map(|w| format!("[a {:+.3} k {:+.2} b {:.2} lk {} fz {:6.0} {}]", w.alpha, w.kappa, w.brake_cmd, w.locked as u8, w.load, w.surface.ground.name()))
                .collect();
            eprintln!(
                "t {t:5.1} ias {:5.1} x {:+6.1} hdg {:+5.1} sideg {:+.2} roll {:+.1} {}",
                a.airspeed_kt(),
                a.pos.x,
                a.heading().to_degrees(),
                a.g_load(),
                a.roll().to_degrees(),
                w.join(" ")
            );
        }
        Controls { throttle: if cut { 0.0 } else { 1.0 }, flaps: 0.0, autobrake: Autobrake::Max, ..Default::default() }
    });
    eprintln!("{:?}", a.crashed);
}
