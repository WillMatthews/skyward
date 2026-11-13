//! Track H2b: Extra 500, Super Cub, Zlin Z-50 LX, DHC-3 Otter (floats) and
//! DC-3 against published figures (sources in each `assets/aircraft/<id>/CREDITS.md`).

use super::*;

/// `cargo test --release ga_probe -- --ignored --nocapture` (GA_ID=<id> for one).
#[test]
#[ignore]
fn ga_probe() {
    let only = std::env::var("GA_ID").ok();
    for id in ["extra500", "cub", "zlin", "dc3", "otter"] {
        if only.as_deref().is_some_and(|o| o != id) {
            continue;
        }
        let def = ac(id);
        let last = def.flap_detents.len() - 1;
        let full = if last > 0 { 1.0 } else { 0.0 };
        eprintln!("== {id}: mass {} kg, stall analytic clean {:.1} / full {:.1} kt", def.mass, def.stall_speed(0.0) / KT, def.stall_speed(full) / KT);
        eprintln!("   sim stall clean {:.1} kt, full {:.1} kt", sim_stall_speed(&def, 0.0), sim_stall_speed(&def, full));
        for kt in [def.vref_kt, def.vref_kt + 20.0, def.vref_kt + 50.0] {
            eprintln!("   climb at {kt} kt: {:.0} fpm", climb_rate(&def, 300.0, kt));
        }
        for alt in [300.0, 2000.0, 4000.0, 7000.0] {
            if alt > def.ceiling_m {
                continue;
            }
            let (v, m) = max_level_tas(&def, alt, def.vno_kt * 0.8, 150.0);
            eprintln!("   max level at {alt} m: {v:.0} KTAS M{m:.2}");
        }
        if def.on_water() {
            let t = water_takeoff(&def, 2, true);
            eprintln!(
                "   water take-off: run {:.0} m in {:.1} s, lift-off {:.0} kt, hump drag peak {:.2} W at {:.0} kt, on the step at {:.0} kt",
                t.run_m, t.time_s, t.liftoff_kt, t.hump_drag, t.hump_kt, t.step_kt
            );
            let l = water_landing(&def, 300.0, 0.0, true);
            eprintln!("   water landing: run {:.0} m to 10 kt, peak slam {:.2} g, {:?}", l.0, l.1, l.2);
            continue;
        }
        let to_flap = if last >= 1 { def.flap_detents[1] / def.flap_detents[last] } else { 0.0 };
        let t = takeoff(&def, to_flap, deg(8.0), def.taildragger, true);
        eprintln!(
            "   take-off flap {to_flap:.2}: roll {:.0} m, lift-off {:.0} kt in {:.1} s, climb15 {:.0} m, max dev {:.1}°",
            t.roll_m,
            t.liftoff_kt,
            t.time_s,
            t.climb_15s,
            t.max_heading_dev.to_degrees()
        );
        eprintln!("   landing roll {:.0} m", ga_landing_roll(&def, 0.6));
    }
}

/// Landing roll (m) from a 2 m/s arrival at Vref − 5 in landing config:
/// brakes at `brakes` once rolling, stick held back on taildraggers (tail
/// down, as their pilots do), rudder on the centreline.
pub(super) fn ga_landing_roll(def: &AircraftDef, brakes: f32) -> f32 {
    let env = TestEnv::default();
    // Light taildraggers arrive three-point, just above the stall; the DC-3
    // is wheel-landed (mains first, tail lowered as it slows).
    let three = def.taildragger && def.mass < 3_000.0;
    let wheel_landing = def.taildragger && !three;
    let kt = if three { def.vs0_kt * 1.08 } else { def.vref_kt - 5.0 };
    let last = def.flap_detents.len() - 1;
    let full = if last > 0 { 1.0 } else { 0.0 };
    let (mut a, base) = flying(def, Vec3::new(0.0, def.gear_height + 0.5, 0.0), 0.0, kt * KT, full);
    let tp = def.trim_point_at(kt * KT, full, 0.0, true);
    a.rot = attitude(0.0, if three { def.rest_pitch() - deg(0.5) } else { tp.pitch + deg(1.0) }, 0.0);
    a.vel = Vec3::new(0.0, -1.5, -tas_from_cas(kt * KT, 0.0));
    a.pos.y = def.gear_height + 0.3;
    let mut touch: Option<Vec3> = None;
    let mut t_ground = 0.0;
    run(&mut a, def, &env, 120.0, |a| {
        if a.on_ground {
            touch.get_or_insert(a.pos);
            t_ground += DT;
        }
        let rolling = t_ground > 0.5;
        if std::env::var("LR_TRACE").is_ok() && ((t_ground + a.pos.y) * 10.0).fract() < 0.12 {
            eprintln!(
                "  lr y {:.2} pitch {:.1} roll {:.1} gs {:.1} vs {:.2} gnd {} alpha {:.1}",
                a.pos.y,
                a.pitch().to_degrees(),
                a.roll().to_degrees(),
                a.vel.length(),
                a.vel.y,
                a.on_ground,
                a.alpha().to_degrees()
            );
        }
        Controls {
            throttle: 0.0,
            // (Brakes with the tail up would stand a taildragger on its nose.)
            brakes: if rolling && (!wheel_landing || a.pitch() > def.rest_pitch() - deg(2.0)) { brakes } else { 0.0 },
            // Taildraggers: hold the three-point attitude (tail firmly down, no balloon).
            elevator: if three || (wheel_landing && a.airspeed_kt() < 45.0) {
                (0.3 + 4.0 * (def.rest_pitch() - a.pitch()) - 0.5 * a.rates().y).clamp(-0.3, 1.0)
            } else if wheel_landing && touch.is_some() {
                // Pin the mains on: tail up, about level, with forward stick.
                (3.0 * (deg(1.0) - a.pitch()) - 0.8 * a.rates().y - 0.1).clamp(-0.8, 0.5)
            } else if rolling {
                0.0
            } else {
                0.2
            },
            rudder: (-3.0 * a.heading() - 1.0 * a.rates().z).clamp(-1.0, 1.0),
            ..base
        }
    });
    assert!(a.crashed.is_none(), "{} landing: {:?}", def.id, a.crashed);
    assert!(a.vel.length() < 0.3, "{} still rolling {}", def.id, a.vel.length());
    (a.pos - touch.unwrap()).length()
}

/// Open water everywhere (the surface at y = 0), with an optional wind.
pub(super) fn lake(wind: Vec3) -> TestEnv {
    TestEnv { water: vec![(Vec2::splat(-50_000.0), Vec2::splat(50_000.0))], wind, ..Default::default() }
}

pub(super) struct WaterTakeoff {
    pub run_m: f32,
    pub time_s: f32,
    pub liftoff_kt: f32,
    /// Peak water drag as a fraction of the weight, and the speed it peaked at.
    pub hump_drag: f32,
    pub hump_kt: f32,
    /// Speed at which the floats were running on the step (dynamic support > 50 %).
    pub step_kt: f32,
}

/// Float take-off as taught: full power, stick fully back to lift the bows
/// over the hump, then ease forward to hold the planing attitude on the step,
/// rotate at Vr. `flap_idx` is the lever detent.
pub(super) fn water_takeoff(def: &AircraftDef, flap_idx: usize, trace: bool) -> WaterTakeoff {
    let env = lake(Vec3::ZERO);
    let mut a = Aircraft::on_runway(def, Vec2::ZERO, 0.0, &env);
    let flap = def.flap_detents[flap_idx] / def.flap_detents.last().unwrap();
    a.surf.flaps = flap;
    let start = a.pos;
    let mut t = 0.0f32;
    let (mut hump, mut hump_kt, mut step_kt) = (0.0f32, 0.0f32, 0.0f32);
    let mut airborne: Option<(f32, f32, f32)> = None;
    let mut res = None;
    let mut pitch_i = 0.0f32;
    while t < 120.0 {
        let ias = a.airspeed_kt();
        let w = a.water.last;
        let on_step = w.wet && w.planing > 0.5;
        if on_step && step_kt == 0.0 {
            step_kt = ias;
        }
        if w.wet && w.drag / (def.mass * GRAVITY) > hump {
            hump = w.drag / (def.mass * GRAVITY);
            hump_kt = ias;
        }
        // Planing attitude ~6° until Vr, then rotate to ~9°.
        let target = if ias < 18.0 {
            deg(14.0)
        } else if ias < def.vr_kt {
            deg(6.0)
        } else {
            deg(9.0)
        };
        let e = target - a.pitch();
        pitch_i = (pitch_i + 0.6 * e * DT).clamp(-0.4, 0.4);
        let elevator = if ias < 18.0 { 1.0 } else { (3.0 * e + pitch_i - 0.8 * a.rates().y).clamp(-1.0, 1.0) };
        let rudder = (-4.0 * a.heading() - 2.0 * a.rates().z - 0.05 * a.pos.x).clamp(-1.0, 1.0);
        let aileron = (-2.0 * a.roll() - 0.5 * a.rates().x).clamp(-1.0, 1.0);
        let c = Controls { throttle: 1.0, flaps: flap, elevator, rudder, aileron, ..Default::default() };
        a.step(def, &c, &env, DT);
        assert_sane(&a);
        if trace && (t * 2.0).fract() < DT * 2.0 {
            eprintln!(
                "  wt t {t:4.1} ias {ias:5.1} y {:.2} pitch {:5.1} roll {:4.1} wet {} plan {:.2} drag {:.2}W disp {:.2} slam {:.2} hdg {:.1}",
                a.pos.y,
                a.pitch().to_degrees(),
                a.roll().to_degrees(),
                w.wet,
                w.planing,
                w.drag / (def.mass * GRAVITY),
                w.displaced,
                w.slam_g,
                a.heading().to_degrees()
            );
        }
        assert!(a.crashed.is_none(), "{} water take-off: {:?} at {ias:.0} kt", def.id, a.crashed);
        t += DT;
        if !a.water.last.wet && !a.on_ground {
            if airborne.is_none() {
                airborne = Some(((a.pos - start).length(), t, ias));
            }
            if t - airborne.unwrap().1 > 2.0 {
                res = airborne;
                break;
            }
        } else {
            airborne = None;
        }
    }
    let (d, tl, v) = res.unwrap_or_else(|| panic!("{} never got off the water", def.id));
    WaterTakeoff { run_m: d, time_s: tl, liftoff_kt: v, hump_drag: hump, hump_kt, step_kt }
}

/// Water landing: arrive at 1.15 Vs0 (full flap) sinking `sink_fpm`, nose up
/// in the landing attitude, `bank` rad, power off; run out with the stick
/// back. Returns (run to 10 kt in m, peak slam g, crash).
pub(super) fn water_landing(def: &AircraftDef, sink_fpm: f32, bank: f32, trace: bool) -> (f32, f32, Option<String>) {
    let env = lake(Vec3::ZERO);
    let kt = def.vs0_kt * 1.15;
    let (mut a, base) = flying(def, Vec3::new(0.0, 6.0, 0.0), 0.0, kt * KT, 1.0);
    let tp = def.trim_point_at(kt * KT, 1.0, 0.0, true);
    a.rot = attitude(0.0, tp.pitch.max(deg(5.0)), bank);
    a.vel = Vec3::new(0.0, -sink_fpm * FPM, -tas_from_cas(kt * KT, 0.0));
    a.pos.y = def.gear_height + 1.2;
    let mut touch: Option<Vec3> = None;
    let mut slam = 0.0f32;
    let mut stop = None;
    let mut t = 0.0;
    run(&mut a, def, &env, 90.0, |a| {
        t += DT;
        let w = a.water.last;
        if w.wet {
            touch.get_or_insert(a.pos);
            slam = slam.max(w.slam_g);
        }
        if touch.is_some() && stop.is_none() && a.vel.length() < 10.0 * KT {
            stop = Some(a.pos);
        }
        if trace && (t * 10.0f32).fract() < DT * 10.0 {
            eprintln!(
                "  wl t {t:4.1} ias {:5.1} y {:.2} pitch {:5.1} vs {:5.2} wet {} plan {:.2} slam {:.2}",
                a.airspeed_kt(),
                a.pos.y,
                a.pitch().to_degrees(),
                a.vel.y,
                w.wet,
                w.planing,
                w.slam_g
            );
        }
        // Touch down in the planing attitude and hold it; full back stick once
        // off the step (the bows stay up as it settles into the water).
        let e = deg(6.0) - a.pitch();
        Controls {
            throttle: 0.0,
            elevator: if touch.is_some() && a.airspeed_kt() < 25.0 { 1.0 } else { (3.0 * e - 0.8 * a.rates().y).clamp(-1.0, 1.0) },
            aileron: (-2.0 * a.roll() - 0.5 * a.rates().x).clamp(-1.0, 1.0),
            rudder: (-3.0 * a.heading() - 1.0 * a.rates().z).clamp(-1.0, 1.0),
            ..base
        }
    });
    let run_m = match (touch, stop) {
        (Some(t0), Some(s)) => (s - t0).length(),
        _ => f32::NAN,
    };
    (run_m, slam, a.crashed.clone())
}

#[test]
#[ignore]
fn float_settle_probe() {
    let def = ac("otter");
    let env = lake(Vec3::ZERO);
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    a.crashed = None;
    a.pos = Vec3::new(0.0, def.gear_height, 0.0);
    a.rot = attitude(0.0, 0.0, 0.0);
    a.vel = Vec3::ZERO;
    a.omega = Vec3::ZERO;
    for k in 0..(20 * 120) {
        a.step(&def, &Controls::default(), &env, DT);
        if k % 30 == 0 || a.crashed.is_some() {
            let w = a.water.last;
            eprintln!(
                "t {:.2} y {:.3} pitch {:.2} roll {:.2} vy {:.3} wet {} disp {:.2} buoy {:.0} dyn {:.0} fz {:.0} m {:?} {:?}",
                k as f32 * DT,
                a.pos.y,
                a.pitch().to_degrees(),
                a.roll().to_degrees(),
                a.vel.y,
                w.wet,
                w.displaced,
                w.buoyancy,
                w.dynamic,
                w.force.y,
                w.moment,
                a.crashed
            );
        }
        if a.crashed.is_some() {
            break;
        }
    }
}

/// `GA_SPIN=1 cargo test --release zlin_spin_probe -- --ignored --nocapture`
#[test]
#[ignore]
fn zlin_spin_probe() {
    let def = ac(&std::env::var("GA_ID").unwrap_or_else(|_| "zlin".into()));
    let env = TestEnv::default();
    let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 2500.0, 0.0), 0.0, def.stall_speed(0.0) * 1.2);
    let mut t = 0.0f32;
    run(&mut a, &def, &env, 30.0, |a| {
        t += DT;
        if (t * 2.0).fract() < DT * 2.0 {
            eprintln!(
                "t {t:4.1} ias {:5.1} alt {:6.0} alpha {:6.1} pitch {:6.1} roll {:6.1} p {:6.0} q {:6.0} r {:6.0} turn {:6.0} g {:5.2}",
                a.airspeed_kt(),
                a.pos.y,
                a.alpha().to_degrees(),
                a.pitch().to_degrees(),
                a.roll().to_degrees(),
                a.rates().x.to_degrees(),
                a.rates().y.to_degrees(),
                a.rates().z.to_degrees(),
                a.turn_rate().to_degrees(),
                a.g_load()
            );
        }
        if t < 15.0 {
            Controls { elevator: 1.0, rudder: -1.0, ..Default::default() }
        } else if t < 18.0 {
            Controls { elevator: -0.5, rudder: 1.0, ..Default::default() }
        } else {
            Controls { elevator: 0.0, ..Default::default() }
        }
    });
}

// ---------------------------------------------------------------------------
// Published numbers
// ---------------------------------------------------------------------------

/// Published figures (each type's `CREDITS.md`) at the simulated weights:
/// Extra 500 1,986 kg, Super Cub 794 kg, Zlin 720 kg, DC-3 11 t; the Otter
/// (floats, 3.5 t) in `otter_on_the_water`.
#[test]
fn ga_published_numbers() {
    struct Row {
        id: &'static str,
        /// 1-g stall, kt IAS: clean, landing flap.
        stall: (f32, f32),
        /// Take-off flap lever, rotation pitch°, ground roll m, lift-off kt.
        to: (f32, f32, std::ops::Range<f32>, std::ops::Range<f32>),
        /// (KIAS, fpm, tolerance): full-power climb from 300 m.
        climb: (f32, f32, f32),
        /// (alt m, KTAS, tolerance kt): maximum level speed.
        vmax: (f32, f32, f32),
        /// Landing roll with brakes at `.1`, m.
        roll: (std::ops::Range<f32>, f32),
    }
    let rows = [
        // Extra EA-500: stall 61 kt flaps down (published), ~72 clean; ~1,000 ft ground
        // roll; 1,600 fpm; 210 KTAS at FL250 (we hold 7,000 m); ~1,300 ft roll.
        Row {
            id: "extra500",
            stall: (72.0, 61.0),
            to: (0.5, 8.0, 230.0..450.0, 68.0..85.0),
            climb: (105.0, 1_600.0, 0.25),
            vmax: (7_000.0, 212.0, 15.0),
            roll: (300.0..700.0, 1.0),
        },
        // PA-18-150 (POH, 1,750 lb): 41 / 37 kt; take-off run 200 ft (short-field,
        // we are a little longer); 960 fpm; 113 kt max at sea level; landing roll 350 ft.
        Row { id: "cub", stall: (41.0, 37.0), to: (0.5, 8.0, 60.0..130.0, 36.0..50.0), climb: (70.0, 960.0, 0.25), vmax: (300.0, 113.0, 6.0), roll: (60.0..150.0, 0.6) },
        // Zlin Z-50 L/LX (720 kg): stall ≈ 58 kt; climb 16 m/s (3,150 fpm); 158 kt max.
        Row { id: "zlin", stall: (58.0, 58.0), to: (0.0, 8.0, 90.0..220.0, 58.0..80.0), climb: (80.0, 3_150.0, 0.2), vmax: (300.0, 158.0, 6.0), roll: (180.0..450.0, 0.6) },
        // DC-3 (11 t): Vs0 64 KIAS at gross (Buckaroo's notes) → 63 at 11 t, ~73 clean;
        // climb 1,130 fpm is at gross with METO power (1,050 hp): with take-off power
        // (1,200 hp) at 11 t the same aircraft makes ≈ 1,450; 200 KTAS at 8,500 ft;
        // wheel landing, brakes once the tail is down.
        Row {
            id: "dc3",
            stall: (73.0, 63.0),
            to: (1.0 / 3.0, 7.0, 330.0..700.0, 76.0..95.0),
            climb: (105.0, 1_450.0, 0.2),
            vmax: (2_600.0, 200.0, 10.0),
            roll: (450.0..1_000.0, 0.6),
        },
    ];
    for r in rows {
        let def = ac(r.id);
        let full = if def.flap_detents.len() > 1 { 1.0 } else { 0.0 };
        let s_clean = sim_stall_speed(&def, 0.0);
        let s_full = sim_stall_speed(&def, full);
        let t = takeoff(&def, r.to.0, deg(r.to.1), def.taildragger, true);
        let roc = climb_rate(&def, 300.0, r.climb.0);
        let (vmax, _) = max_level_tas(&def, r.vmax.0, def.vno_kt * 0.8, 150.0);
        let roll = ga_landing_roll(&def, r.roll.1);
        eprintln!(
            "GA {:9} stall {:5.1}/{:5.1} kt (target {}/{}) | T/O roll {:4.0} m lift-off {:3.0} kt | climb {:5.0} fpm (target {}) | max {:4.0} KTAS (target {}) | landing roll {:4.0} m",
            r.id, s_clean, s_full, r.stall.0, r.stall.1, t.roll_m, t.liftoff_kt, roc, r.climb.1, vmax, r.vmax.1, roll
        );
        assert!((s_clean / r.stall.0 - 1.0).abs() < 0.07, "{} clean stall {s_clean}", r.id);
        assert!((s_full / r.stall.1 - 1.0).abs() < 0.08, "{} landing stall {s_full}", r.id);
        assert!(r.to.2.contains(&t.roll_m), "{} ground roll {}", r.id, t.roll_m);
        assert!(r.to.3.contains(&t.liftoff_kt), "{} lift-off {}", r.id, t.liftoff_kt);
        assert!(t.climb_15s > 25.0, "{} climb-out {}", r.id, t.climb_15s);
        assert!((roc / r.climb.1 - 1.0).abs() < r.climb.2, "{} climb {roc}", r.id);
        assert!((vmax - r.vmax.1).abs() < r.vmax.2, "{} max speed {vmax}", r.id);
        assert!(r.roll.0.contains(&roll), "{} landing roll {roll}", r.id);
    }
}

/// Taildraggers swing left on the take-off roll without right rudder (torque,
/// P-factor, slipstream on the fin) — the Cub gently, the DC-3's two
/// clockwise radials more — and track straight with it.
#[test]
fn taildragger_swing() {
    // (The DC-3's tail wheel is locked for take-off: it swings less.)
    for (id, min_swing) in [("cub", 1.0), ("dc3", 0.3), ("zlin", 5.0)] {
        let def = ac(id);
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        let mut min_hdg = 0.0f32;
        let mut t = 0.0;
        run(&mut a, &def, &env, 10.0, |a| {
            t += DT;
            min_hdg = min_hdg.min(wrap(a.heading()));
            Controls { throttle: (t * 0.4).min(1.0), ..Default::default() }
        });
        eprintln!("{id} feet-off take-off roll: heading went to {:.1}° ({:?})", min_hdg.to_degrees(), a.crashed);
        assert!(min_hdg < deg(-min_swing), "{id} swing {}", min_hdg.to_degrees());
        let r = takeoff(&def, 0.0, deg(8.0), true, true);
        assert!(r.max_heading_dev < deg(8.0), "{id} with rudder {}", r.max_heading_dev.to_degrees());
    }
}

/// Low and slow: the same 8 kt sharp-edged headwind gust is a far bigger
/// share of the Super Cub's airspeed than of the DC-3's, so it bounces it about
/// twice as hard.
#[test]
fn cub_is_gust_sensitive() {
    let response = |id: &str| {
        let def = ac(id);
        let kt = if id == "cub" { 70.0 } else { 140.0 };
        let (mut a, base) = flying(&def, Vec3::new(0.0, 800.0, 0.0), 0.0, kt * KT, 0.0);
        // (Heading north; air moving south = a headwind.)
        let env = TestEnv { wind: Vec3::new(0.0, 0.0, 8.0 * KT), ..Default::default() };
        let vs0 = a.vertical_speed();
        let mut gmax = 1.0f32;
        run(&mut a, &def, &env, 1.0, |a| {
            gmax = gmax.max(a.g_load());
            base
        });
        (gmax - 1.0, a.vertical_speed() - vs0)
    };
    let (cub_g, _) = response("cub");
    let (dc3_g, _) = response("dc3");
    eprintln!("3 m/s vertical gust: cub +{cub_g:.2} g, dc3 +{dc3_g:.2} g");
    assert!(cub_g > 1.4 * dc3_g, "cub {cub_g} vs dc3 {dc3_g}");
}

// ---------------------------------------------------------------------------
// Aerobatics (Zlin Z-50 LX)
// ---------------------------------------------------------------------------

fn zlin_at(kt: f32, alt: f32) -> (AircraftDef, Aircraft, Controls) {
    let def = ac("zlin");
    let (a, c) = flying(&def, Vec3::new(0.0, alt, 0.0), 0.0, kt * KT, 0.0);
    (def, a, c)
}

#[test]
fn zlin_roll_rate_and_g() {
    let (def, mut a, c) = zlin_at(130.0, 1500.0);
    let env = TestEnv::default();
    let mut pmax = 0.0f32;
    let mut rolled = 0.0f32;
    run(&mut a, &def, &env, 2.0, |a| {
        pmax = pmax.max(a.rates().x);
        rolled += a.rates().x * DT;
        Controls { aileron: 1.0, ..c }
    });
    eprintln!("zlin full aileron at 130 kt: {:.0}°/s peak, {:.0}° in 2 s", pmax.to_degrees(), rolled.to_degrees());
    assert!(pmax > deg(200.0) && pmax < deg(340.0), "roll rate {}", pmax.to_degrees());
    // A 6 g pull at 150 kt is well inside its +9 g limit; the structure holds.
    let (def, mut a, c) = zlin_at(150.0, 1500.0);
    let mut gmax = 0.0f32;
    run(&mut a, &def, &env, 1.5, |a| {
        gmax = gmax.max(a.g_load());
        Controls { elevator: 0.8, ..c }
    });
    eprintln!("zlin pull at 150 kt: {gmax:.1} g");
    assert!(a.crashed.is_none() && gmax > 5.0);
}

/// Inverted flight: a symmetric wing holds altitude upside down with forward
/// stick; the inverted fuel and oil systems keep the engine running. (The
/// Super Cub's float carburettor starves within a second.)
#[test]
fn inverted_flight_and_fuel() {
    use crate::systems::{self, Ambient};
    let (def, mut a, c) = zlin_at(130.0, 1500.0);
    a.sys = systems::Systems::new(&def, false);
    a.rot = attitude(0.0, deg(2.0), PI);
    let env = TestEnv::default();
    let mut i_pitch = 0.0f32;
    let amb = Ambient::default();
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for k in 0..(30 * 120) {
        let _ = k;
        // Hold the altitude inverted: push (negative elevator) to keep the nose up.
        let e = 0.03 * (1500.0 - a.pos.y) - 0.1 * a.vertical_speed();
        i_pitch = (i_pitch + 0.2 * e * DT).clamp(-0.6, 0.6);
        let elevator = (-(0.25 + i_pitch + e)).clamp(-1.0, 1.0);
        let aileron = (-2.0 * wrap(a.roll() - PI) - 0.3 * a.rates().x).clamp(-1.0, 1.0);
        let ctl = Controls { elevator, aileron, throttle: 0.85, ..c };
        systems::step(&mut a, &def, &ctl, &amb, DT);
        a.step(&def, &ctl, &env, DT);
        lo = lo.min(a.pos.y);
        hi = hi.max(a.pos.y);
        assert!(a.crashed.is_none(), "{:?}", a.crashed);
        assert!(a.up().y < -0.85, "stays inverted");
    }
    eprintln!(
        "zlin inverted 30 s: alt {lo:.0}..{hi:.0} m, roll {:.0}°, alpha {:.1}°, engine running {}, power {:.2}",
        a.roll().to_degrees(),
        a.alpha().to_degrees(),
        a.engine_running(0),
        a.sys.fx.power[0]
    );
    assert!(hi - lo < 120.0);
    assert!(a.engine_running(0) && a.sys.fx.power[0] > 0.9);
    assert!(a.sys.eng[0].oil_p_mul > 0.9, "inverted oil holds pressure");

    // The Cub's float carburettor: power dies away under negative g.
    let def = ac("cub");
    let (mut a, c) = flying(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, 90.0 * KT, 0.0);
    a.sys = systems::Systems::new(&def, false);
    let mut p_min = 1.0f32;
    for k in 0..(4 * 120) {
        let ctl = Controls { elevator: if k > 60 { -1.0 } else { 0.0 }, throttle: 1.0, ..c };
        systems::step(&mut a, &def, &ctl, &amb, DT);
        a.step(&def, &ctl, &env, DT);
        p_min = p_min.min(a.sys.fx.power[0]);
    }
    eprintln!("cub pushed to {:.1} g: power fell to {p_min:.2}", a.g_load());
    assert!(p_min < 0.2, "float carburettor starves under negative g ({p_min})");
}

/// Snap roll: at 90 kt, full aft stick and full rudder stall one wing first —
/// a full autorotating roll in ~1–2 s; neutral controls and it flies out.
#[test]
fn zlin_snap_roll() {
    let (def, mut a, c) = zlin_at(90.0, 1500.0);
    let env = TestEnv::default();
    let mut rolled = 0.0f32;
    let mut t = 0.0;
    let mut t360 = None;
    run(&mut a, &def, &env, 4.0, |a| {
        t += DT;
        rolled += a.rates().x * DT;
        if t360.is_none() && rolled.abs() > TAU {
            t360 = Some(t);
        }
        if t360.is_some() { Controls { elevator: 0.0, rudder: 0.0, ..c } } else { Controls { elevator: 1.0, rudder: 1.0, ..c } }
    });
    eprintln!("zlin snap roll at 90 kt: 360° in {:?} s, total {:.0}°, final alpha {:.1}°, rates {:?}", t360, rolled.to_degrees(), a.alpha().to_degrees(), a.rates());
    let t360 = t360.expect("snap roll completes a full turn");
    assert!(t360 < 2.5, "snap {t360}");
    assert!(a.crashed.is_none());
    assert!(a.rates().length() < 1.5, "flies out of it");
}

/// Spin entry and the standard (PARE) recovery: power idle, ailerons neutral,
/// full opposite rudder, stick forward — the rotation stops within about
/// 1.5 turns; then a pull-out.
#[test]
fn zlin_spin_and_recovery() {
    let def = ac("zlin");
    let env = TestEnv::default();
    let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 2500.0, 0.0), 0.0, def.stall_speed(0.0) * 1.15);
    let pro = Controls { elevator: 1.0, rudder: -1.0, ..Default::default() };
    let mut yaw_turned = 0.0f32;
    run(&mut a, &def, &env, 8.0, |a| {
        yaw_turned += a.turn_rate() * DT;
        pro
    });
    let spin_rate = a.turn_rate();
    let alt_spin = a.pos.y;
    eprintln!("zlin spin: {:.0}°/s, {:.1} turns in 8 s, alpha {:.0}°, ias {:.0} kt", spin_rate.to_degrees(), yaw_turned.abs() / TAU, a.alpha().to_degrees(), a.airspeed_kt());
    assert!(spin_rate < deg(-120.0), "spins left {}", spin_rate.to_degrees());
    assert!(yaw_turned.abs() > TAU * 2.0);
    // PARE.
    let mut stop_turns = 0.0f32;
    let mut stopped = false;
    let mut t = 0.0;
    run(&mut a, &def, &env, 6.0, |a| {
        t += DT;
        if !stopped {
            stop_turns += a.turn_rate().abs() * DT / TAU;
            // (The yaw rate about the vertical; heading is ill-defined straight down.)
            if a.rates().z.abs() < deg(25.0) && a.rates().x.abs() < deg(40.0) && t > 0.5 {
                stopped = true;
            }
        }
        if stopped {
            Controls { elevator: 0.0, aileron: (-1.5 * a.roll().sin()).clamp(-1.0, 1.0), ..Default::default() }
        } else {
            Controls { elevator: -0.6, rudder: 1.0, ..Default::default() }
        }
    });
    // Pull out of the dive (≤ 5 g).
    let mut gmax = 0.0f32;
    run(&mut a, &def, &env, 8.0, |a| {
        gmax = gmax.max(a.g_load());
        let wings = (-2.0 * a.roll() - 0.4 * a.rates().x).clamp(-1.0, 1.0);
        let pull = if a.pitch() < deg(5.0) { (0.6 * (4.5 - a.g_load())).clamp(-0.2, 0.7) } else { 0.0 };
        Controls { elevator: pull, aileron: wings, throttle: 0.5, ..Default::default() }
    });
    eprintln!("zlin recovery: rotation stopped after {stop_turns:.2} turns; pull-out peak {gmax:.1} g; height lost {:.0} m", alt_spin - a.pos.y);
    assert!(stopped && stop_turns < 1.6, "recovery {stop_turns} turns");
    assert!(a.crashed.is_none());
    assert!(a.vertical_speed() > -5.0 && a.pitch() > deg(-10.0), "pulled out: vs {} pitch {}", a.vertical_speed(), a.pitch().to_degrees());
}

// ---------------------------------------------------------------------------
// Water operations (DHC-3 Otter on floats)
// ---------------------------------------------------------------------------

#[test]
fn otter_on_the_water() {
    let def = ac("otter");
    // Afloat at rest: draft, trim, stays put (no brakes on water: it drifts at most).
    let env = lake(Vec3::ZERO);
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.4, &env);
    run(&mut a, &def, &env, 20.0, |_| Controls::default());
    let draft = a.pos.y - (-def.hulls[0].keel_y);
    eprintln!(
        "otter afloat: CG {:.2} m above the water (draft {:.2} m), pitch {:.1}°, roll {:.1}°, idle drift {:.1} kt",
        a.pos.y,
        -draft,
        a.pitch().to_degrees(),
        a.roll().to_degrees(),
        a.vel.length() / KT
    );
    assert!(a.crashed.is_none() && a.on_ground && a.water.afloat());
    assert!(-draft > 0.3 && -draft < 0.6, "draft {}", -draft);
    assert!(a.pitch().abs() < deg(4.0) && a.roll().abs() < deg(1.5));
    // Idle-power "step taxi": a few knots.
    assert!(a.vel.length() < 9.0 * KT);

    // Published: stall ~62 clean / ~52 flaps; ~750 fpm on floats; ~125 kt max.
    let s_clean = sim_stall_speed(&def, 0.0);
    let s_full = sim_stall_speed(&def, 1.0);
    let roc = climb_rate(&def, 300.0, 75.0);
    let (vmax, _) = max_level_tas(&def, 300.0, 100.0, 150.0);
    eprintln!("otter: stall {s_clean:.1}/{s_full:.1} kt, climb {roc:.0} fpm, max {vmax:.0} KTAS");
    assert!((s_clean / 62.0 - 1.0).abs() < 0.07 && (s_full / 52.0 - 1.0).abs() < 0.08);
    assert!((roc / 750.0 - 1.0).abs() < 0.3, "climb {roc}");
    assert!((vmax - 125.0).abs() < 10.0, "max {vmax}");

    // Take-off from the water: over the hump onto the step, lift off at ~60 kt.
    let t = water_takeoff(&def, 2, false);
    eprintln!(
        "otter water take-off: {:.0} m in {:.1} s, lift-off {:.0} kt; hump {:.2} W at {:.0} kt; on the step at {:.0} kt",
        t.run_m, t.time_s, t.liftoff_kt, t.hump_drag, t.hump_kt, t.step_kt
    );
    assert!((250.0..650.0).contains(&t.run_m), "run {}", t.run_m);
    assert!((14.0..35.0).contains(&t.time_s), "time {}", t.time_s);
    assert!((55.0..70.0).contains(&t.liftoff_kt), "lift-off {}", t.liftoff_kt);
    assert!(t.hump_drag > 0.12 && t.hump_drag < 0.3 && t.hump_kt < 25.0, "hump {} at {}", t.hump_drag, t.hump_kt);
    assert!(t.step_kt > 15.0 && t.step_kt < 45.0, "step {}", t.step_kt);

    // Water landing: normal, hard, banked and nose-low.
    let (run_m, slam, crash) = water_landing(&def, 300.0, 0.0, false);
    eprintln!("otter water landing: {run_m:.0} m to 10 kt, peak slam {slam:.2} g, {crash:?}");
    assert!(crash.is_none(), "{crash:?}");
    assert!((150.0..700.0).contains(&run_m), "landing run {run_m}");
    let (_, _, crash) = water_landing(&def, 1_100.0, 0.0, false);
    eprintln!("otter 1,100 fpm onto the water: {crash:?}");
    assert!(crash.as_deref().unwrap_or("").starts_with("Hard landing"), "{crash:?}");
    let (_, _, crash) = water_landing(&def, 200.0, deg(14.0), false);
    eprintln!("otter 14° bank onto the water: {crash:?}");
    assert!(crash.as_deref().is_some_and(|c| c.contains("float") || c.contains("wingtip") || c.contains("Wingtip")), "{crash:?}");
}

/// Wind on the fin weathervanes a floatplane into wind at low speed; the water
/// rudders steer at taxi speed and retract above ~20 kt.
#[test]
fn otter_weathervane_and_water_rudders() {
    let def = ac("otter");
    // 15 kt wind from the east (air moving west) over a 3 km lake, pointing north.
    let env = TestEnv { water: vec![(Vec2::splat(-1_500.0), Vec2::splat(1_500.0))], wind: Vec3::new(-15.0 * KT, 0.0, 0.0), ..Default::default() };
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    run(&mut a, &def, &env, 40.0, |_| Controls::default());
    let hdg = a.heading().to_degrees();
    eprintln!("otter in a 15 kt easterly, idle, hands off: heading {hdg:.0}° after 40 s ({:?})", a.crashed);
    assert!(a.crashed.is_none());
    assert!(hdg > 45.0 && hdg < 135.0, "weathervaned to {hdg}");
    // Water rudders: idle-taxi turn with full right rudder in still water.
    let env = lake(Vec3::ZERO);
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    run(&mut a, &def, &env, 30.0, |_| Controls { throttle: 0.25, rudder: 1.0, ..Default::default() });
    let turn = a.turn_rate().to_degrees();
    eprintln!("otter water-rudder taxi turn: {turn:.1}°/s at {:.1} kt, rudders down {}", a.vel.length() / KT, a.water.last.rudders_down);
    assert!(turn > 4.0 && a.water.last.rudders_down);
}

/// Choppy water: in a 20 kt wind on a lake the take-off still works, with
/// slamming and a longer run.
#[test]
fn otter_choppy_water() {
    let def = ac("otter");
    // A 4 km lake, 20 kt of wind: short, steep chop.
    let env = TestEnv { water: vec![(Vec2::splat(-2_000.0), Vec2::splat(2_000.0))], wind: Vec3::new(0.0, 0.0, 20.0 * KT), ..Default::default() };
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    // (Heading north into a 20 kt northerly: the air moves south, +Z.)
    let mut slam = 0.0f32;
    let mut t = 0.0;
    let mut off = false;
    let mut pitch_i = 0.0f32;
    run(&mut a, &def, &env, 60.0, |a| {
        t += DT;
        slam = slam.max(a.water.last.slam_g);
        if !a.water.last.wet && t > 5.0 {
            off = true;
        }
        let ias = a.airspeed_kt();
        let target = if ias < 30.0 {
            deg(12.0)
        } else if ias < def.vr_kt {
            deg(6.0)
        } else {
            deg(9.0)
        };
        let e = target - a.pitch();
        pitch_i = (pitch_i + 0.6 * e * DT).clamp(-0.4, 0.4);
        Controls {
            throttle: 1.0,
            flaps: 20.0 / 35.0,
            elevator: (3.0 * e + pitch_i - 0.8 * a.rates().y).clamp(-1.0, 1.0),
            aileron: (-2.0 * a.roll() - 0.5 * a.rates().x).clamp(-1.0, 1.0),
            rudder: (-4.0 * a.heading() - 2.0 * a.rates().z).clamp(-1.0, 1.0),
            ..Default::default()
        }
    });
    let sea = a.water.sea;
    eprintln!("otter in chop: waves {:.2} m, λ {:.0} m, off the water {off} after {t:.0} s, peak slam {slam:.2} g", sea.wave_h, sea.wavelength);
    assert!(a.crashed.is_none(), "{:?}", a.crashed);
    assert!(off);
}

#[test]
#[ignore]
fn weathervane_probe() {
    let def = ac("otter");
    let env = lake(Vec3::new(-15.0 * KT, 0.0, 0.0));
    let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
    let mut t = 0.0;
    run(&mut a, &def, &env, 20.0, |a| {
        t += DT;
        if (t * 2.0f32).fract() < DT * 2.0 {
            eprintln!(
                "t {t:.1} hdg {:.2} r {:.2} vel {:?} beta {:.1} ias {:.1} wmom {:?} wet {}",
                a.heading().to_degrees(),
                a.rates().z.to_degrees(),
                a.vel,
                a.beta().to_degrees(),
                a.airspeed_kt(),
                a.water.last.moment,
                a.water.last.wet
            );
        }
        Controls::default()
    });
}
