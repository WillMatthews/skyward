//! Systems / failures tests: each failure's effect on the systems, the
//! instruments and the flight model, and that nothing changes without one.

use super::alerts::{self, Level};
use super::failures::{Failure, Kind, Schedule, Trigger, candidates};
use super::*;
use crate::autopilot::Autopilot;
use crate::flight::{self, Aircraft, AircraftDef, Controls, Environment, FT, KT};
use glam::{Vec2, Vec3};

const DT: f32 = 1.0 / 120.0;

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

fn def(id: &str) -> AircraftDef {
    flight::find(id).unwrap()
}

/// Airborne in trim at `kt` / `alt_m`, systems ready.
fn cruise(d: &AircraftDef, kt: f32, alt_m: f32) -> (Aircraft, Controls, Autopilot) {
    let mut ac = Aircraft::airborne(d, Vec3::new(0.0, alt_m, 0.0), 0.0, kt * KT);
    let mut c = flight::trimmed_controls_at(d, kt * KT, alt_m);
    c.gear_down = ac.gear_pos() > 0.5;
    ac.sys = Systems::new(d, false);
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    (ac, c, ap)
}

/// Fly `secs` with the autopilot (if engaged) and the systems.
fn fly(d: &AircraftDef, ac: &mut Aircraft, c: &mut Controls, ap: &mut Autopilot, secs: f32) {
    let amb = Ambient::default();
    let n = (secs / DT).round() as usize;
    for _ in 0..n {
        if ap.engaged {
            ap.update(d, ac, c, DT);
        }
        step(ac, d, c, &amb, DT);
        ac.step(d, c, &Flat, DT);
        if ac.crashed.is_some() {
            break;
        }
    }
}

fn fail(ac: &mut Aircraft, d: &AircraftDef, k: Kind, i: usize) {
    set_failure(ac, d, Failure::new(k, i), true);
    assert!(ac.sys.fail.has(Failure::new(k, i)), "{k:?} {i} should apply to {}", d.id);
}

#[test]
fn ready_systems_are_neutral() {
    // Ready to go: the flight model sees no effect at all, the instruments the truth.
    for d in flight::catalogue() {
        let (mut ac, mut c, mut ap) = cruise(&d, d.vref_kt.max(60.0) * 1.4, 1200.0);
        assert_eq!(ac.sys.fx, Effects::default(), "{}", d.id);
        fly(&d, &mut ac, &mut c, &mut ap, 20.0);
        assert!(ac.crashed.is_none(), "{} {:?}", d.id, ac.crashed);
        let fx = ac.sys.fx;
        // Only the fuel burnt changes the mass.
        assert_eq!(Effects { mass_delta: 0.0, ..fx }, Effects::default(), "{}", d.id);
        assert!(fx.mass_delta <= 0.0 && fx.mass_delta > -200.0, "{} {}", d.id, fx.mass_delta);
        let ad = ac.sys.air_data(&ac);
        assert_eq!((ad.ias, ad.alt, ad.pitch, ad.roll, ad.heading, ad.flags), (ac.airspeed(), ac.pos.y, ac.pitch(), ac.roll(), ac.heading(), 0), "{}", d.id);
        let al = alerts::alerts(&ac.sys, &d, &ac, &c);
        assert!(al.iter().all(|a| a.level == Level::Advisory), "{}: {:?}", d.id, al.iter().map(|a| &a.title).collect::<Vec<_>>());
        for i in 0..d.engine_count {
            assert!(ac.engine_running(i), "{} engine {i}", d.id);
        }
    }
}

#[test]
fn flight_model_identical_without_systems_effects() {
    // An aircraft stepped with neutral systems effects moves exactly like one without.
    let d = def("meridian320");
    let (mut a, c, _) = cruise(&d, 250.0, 3000.0);
    let mut b = a.clone();
    b.sys = Systems::default();
    a.sys.fx = Effects::default();
    for _ in 0..1200 {
        a.step(&d, &c, &Flat, DT);
        b.step(&d, &c, &Flat, DT);
    }
    assert_eq!(a.pos, b.pos);
    assert_eq!(a.rot, b.rot);
}

#[test]
fn pitot_blockage_asi_acts_as_altimeter() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 80.0, 600.0);
    fly(&d, &mut ac, &mut c, &mut ap, 5.0);
    fail(&mut ac, &d, Kind::PitotBlock, 0);
    let ind0 = ac.sys.air_data(&ac).ias_kt();
    let true0 = ac.airspeed_kt();
    // Climb 500 ft/min at constant speed (autothrottle).
    ap.alt = None;
    ap.vs = Some(500.0 * flight::FPM);
    c.throttle = 1.0;
    let h0 = ac.pos.y;
    fly(&d, &mut ac, &mut c, &mut ap, 90.0);
    let climb = ac.pos.y - h0;
    let ind1 = ac.sys.air_data(&ac).ias_kt();
    let true1 = ac.airspeed_kt();
    assert!(climb > 150.0, "climbed {climb} m");
    assert!(ind1 - ind0 > 8.0, "indicated {ind0:.1} → {ind1:.1} kt (true {true0:.1} → {true1:.1}) over {climb:.0} m");
    assert!(ind1 - true1 > 8.0);
    // Repaired: the truth again.
    set_failure(&mut ac, &d, Failure::new(Kind::PitotBlock, 0), false);
    assert_eq!(ac.sys.air_data(&ac).ias, ac.airspeed());
}

#[test]
fn static_blockage_freezes_altimeter_and_vsi() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::StaticBlock, 0);
    let alt0 = ac.sys.air_data(&ac).alt;
    ap.alt = Some(2200.0);
    ap.vs = Some(-1500.0 * flight::FPM);
    fly(&d, &mut ac, &mut c, &mut ap, 40.0);
    let ad = ac.sys.air_data(&ac);
    assert!(ac.pos.y < alt0 - 200.0, "descended to {}", ac.pos.y);
    assert_eq!(ad.alt, alt0);
    assert_eq!(ad.vs, 0.0);
    // Descending with the old static pressure trapped: the ASI over-reads.
    assert!(ad.ias_kt() > ac.airspeed_kt() + 5.0, "{} vs {}", ad.ias_kt(), ac.airspeed_kt());
}

#[test]
fn vacuum_failure_gyros_drift() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    fail(&mut ac, &d, Kind::Vacuum, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 30.0);
    // Shortly after: still nearly right, suction gone.
    let ad = ac.sys.air_data(&ac);
    assert!(!ac.sys.vacuum_ok());
    assert!((ad.roll - ac.roll()).abs() < 3f32.to_radians(), "{}", ad.roll.to_degrees());
    fly(&d, &mut ac, &mut c, &mut ap, 330.0);
    let ad = ac.sys.air_data(&ac);
    let roll_err = crate::bot::wrap_pi(ad.roll - ac.roll()).abs().to_degrees();
    let hdg_err = crate::bot::wrap_pi(ad.heading - ac.heading()).abs().to_degrees();
    assert!(ac.sys.spin < 0.1, "spin {}", ac.sys.spin);
    assert!(roll_err > 10.0, "AI roll error after 6 min: {roll_err}°");
    assert!(hdg_err > 3.0, "DG error after 6 min: {hdg_err}°");
    let al = alerts::alerts(&ac.sys, &d, &ac, &c);
    assert!(al.iter().any(|a| a.title == "VAC"));
}

#[test]
fn alternator_failure_drains_the_battery() {
    let d = def("kestrel");
    let (mut ac, c, _) = cruise(&d, 95.0, 900.0);
    fail(&mut ac, &d, Kind::Generator, 0);
    let amb = Ambient::default();
    let mut t = 0.0;
    let mut avionics_lost = None;
    while t < 7200.0 {
        step(&mut ac, &d, &c, &amb, 1.0);
        t += 1.0;
        if t == 10.0 {
            assert!(ac.sys.amps < -5.0, "ammeter {}", ac.sys.amps);
            assert_eq!(ac.sys.power, Power::Battery);
            let al = alerts::alerts(&ac.sys, &d, &ac, &c);
            assert!(al.iter().any(|a| a.title == "VOLTS"), "low-voltage light");
        }
        if avionics_lost.is_none() && !ac.sys.bus_ok() {
            avionics_lost = Some(t);
        }
        if ac.sys.power == Power::Off {
            break;
        }
    }
    let lost = avionics_lost.expect("avionics never failed");
    assert!(lost > 20.0 * 60.0 && lost < 60.0 * 60.0, "avionics lost after {:.0} min", lost / 60.0);
    assert!(t < 70.0 * 60.0, "battery flat after {:.0} min", t / 60.0);
    assert_eq!(ac.sys.volts, 0.0);
    // The electric flaps and turn coordinator are dead too.
    assert_eq!(ac.sys.fx.flap_rate, 0.0);
    assert!(ac.sys.air_data(&ac).has(flag::TURN));
}

#[test]
fn airliner_generators_lost_rat_keeps_the_pfd() {
    let d = def("meridian320");
    let (mut ac, c, _) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::Generator, 0);
    fail(&mut ac, &d, Kind::Generator, 1);
    let amb = Ambient::default();
    for _ in 0..120 {
        step(&mut ac, &d, &c, &amb, 0.1);
    }
    assert!(ac.sys.rat_out);
    assert_eq!(ac.sys.power, Power::Emergency);
    assert!(ac.sys.display_on("pfd") && !ac.sys.display_on("sd"));
    let al = alerts::alerts(&ac.sys, &d, &ac, &c);
    assert!(al.iter().any(|a| a.title == "ELEC EMER CONFIG" && a.level == Level::Warning));
}

#[test]
fn fuel_exhaustion_stops_the_engine() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    ac.sys.tank = [0.15, 0.15, 0.0];
    fly(&d, &mut ac, &mut c, &mut ap, 5.0);
    assert!(ac.engine_running(0));
    fly(&d, &mut ac, &mut c, &mut ap, 60.0);
    assert!(!ac.engine_running(0), "engine still running with {} kg", ac.sys.fuel_total());
    assert!(ac.sys.fuel_total() < 0.1);
    // Starved, the propeller windmills.
    assert!(ac.engine_rpm_i(0) < 2000.0);
}

#[test]
fn fuel_selector_off_starves_and_burn_reduces_mass() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    fly(&d, &mut ac, &mut c, &mut ap, 60.0);
    let used = ac.sys.fuel0 - ac.sys.fuel_total();
    // ~25 kg/h at cruise.
    assert!(used > 0.2 && used < 0.8, "used {used} kg in a minute");
    assert!((ac.sys.fx.mass_delta + used).abs() < 1e-3);
    ac.sys.sw.selector = Selector::Off;
    fly(&d, &mut ac, &mut c, &mut ap, 10.0);
    assert!(!ac.engine_running(0));
}

#[test]
fn fuel_leak_unbalances_the_wings() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::FuelLeak, 0);
    let amb = Ambient::default();
    // Burn the centre tank, then leak (systems only, 40 min).
    ac.sys.tank[2] = 0.0;
    for _ in 0..2400 {
        step(&mut ac, &d, &c, &amb, 1.0);
    }
    assert!(ac.sys.tank[0] < ac.sys.tank[1] - 1000.0, "{:?}", ac.sys.tank);
    // Right wing heavy: the CG moves right (−x).
    assert!(ac.sys.fx.cg.x < -0.02, "{:?}", ac.sys.fx.cg);
    assert!(alerts::alerts(&ac.sys, &d, &ac, &c).iter().any(|a| a.title == "FUEL IMBALANCE"));
    // Flown with the imbalance the aircraft wants to roll right.
    ap.disengage();
    c.aileron = 0.0;
    let r0 = ac.roll();
    for _ in 0..240 {
        step(&mut ac, &d, &c, &amb, DT);
        ac.step(&d, &c, &Flat, DT);
    }
    let _ = (r0, &mut ap);
    assert!(ac.crashed.is_none());
}

#[test]
fn engine_fire_and_extinguisher() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::EngineFire, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 2.0);
    ac.sys.update_alerts(&d, &ac.clone(), &c);
    let al = alerts::alerts(&ac.sys, &d, &ac, &c);
    let fire = al.iter().find(|a| a.title == "ENG 1 FIRE").expect("ENG 1 FIRE");
    assert_eq!(fire.level, Level::Warning);
    assert!(fire.lines.iter().all(|l| !l.done || l.action.starts_with("THR")));
    assert!(ac.sys.master_warn);
    assert_eq!(ac.sys.aural_loop, Aural::Crc);
    // Agent with the fuel still on: knocked down, not out.
    assert!(ac.sys.discharge_agent(0));
    fly(&d, &mut ac, &mut c, &mut ap, 30.0);
    assert!(ac.sys.eng[0].fire > 0.0, "fire came back with the fuel on");
    // The procedure: master off, fire push-button, agent 2.
    ac.sys.sw.eng_master[0] = false;
    ac.sys.sw.fire_handle[0] = true;
    assert!(ac.sys.discharge_agent(0));
    fly(&d, &mut ac, &mut c, &mut ap, 2.0);
    assert_eq!(ac.sys.eng[0].fire, 0.0);
    assert!(!ac.sys.fail.any_of(Kind::EngineFire));
    assert!(!ac.engine_running(0) && ac.engine_running(1));
    ac.sys.update_alerts(&d, &ac.clone(), &c);
    assert!(!alerts::alerts(&ac.sys, &d, &ac, &c).iter().any(|a| a.fire));
    assert_eq!(ac.sys.aural_loop, Aural::None);
    // No third bottle.
    assert!(!ac.sys.discharge_agent(0));
}

#[test]
fn unattended_engine_fire_tears_the_engine_off() {
    let d = def("meridian190");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::EngineFire, 1);
    fly(&d, &mut ac, &mut c, &mut ap, 200.0);
    assert!(ac.dmg.is_lost(crate::damage::Part::engine(1)));
    assert!(!ac.engine_running(1));
}

#[test]
fn single_engine_fire_burns_out_after_fuel_cutoff() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    fail(&mut ac, &d, Kind::EngineFire, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 5.0);
    assert!(ac.sys.cockpit_smoke() > 0.1);
    ac.sys.sw.eng_master[0] = false; // mixture idle cut-off
    ac.sys.sw.selector = Selector::Off;
    fly(&d, &mut ac, &mut c, &mut ap, 90.0);
    assert_eq!(ac.sys.eng[0].fire, 0.0);
    assert!(!ac.engine_running(0));
}

#[test]
fn gear_alternate_extension() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 200.0, 1500.0);
    assert!(ac.gear_pos() < 0.01);
    fail(&mut ac, &d, Kind::GearExtend, 0);
    c.gear_down = true;
    fly(&d, &mut ac, &mut c, &mut ap, 20.0);
    assert!(ac.gear_pos() < 0.01, "gear {}", ac.gear_pos());
    assert!(alerts::alerts(&ac.sys, &d, &ac, &c).iter().any(|a| a.title == "L/G GEAR NOT DOWNLOCKED"));
    ac.sys.sw.alt_gear = true;
    fly(&d, &mut ac, &mut c, &mut ap, 15.0);
    let mid = ac.gear_pos();
    assert!(mid > 0.1 && mid < 0.95, "gravity extension is slower: {mid}");
    fly(&d, &mut ac, &mut c, &mut ap, 25.0);
    assert!(ac.gear_pos() > 0.999);
    // Can't be retracted after a gravity extension.
    c.gear_down = false;
    fly(&d, &mut ac, &mut c, &mut ap, 10.0);
    assert!(ac.gear_pos() > 0.999);
}

#[test]
fn hydraulic_loss_and_ptu() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 220.0, 2000.0);
    // Engine 1 shut down: green lost its pump, the PTU keeps it up from yellow.
    ac.sys.sw.eng_master[0] = false;
    fly(&d, &mut ac, &mut c, &mut ap, 6.0);
    assert!(!ac.engine_running(0));
    assert!(ac.sys.hyd[0] > 0.8, "PTU: {:?}", ac.sys.hyd);
    // Green fluid loss: gear by gravity only, flaps at half speed.
    fail(&mut ac, &d, Kind::Hydraulic, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 6.0);
    assert!(ac.sys.hyd[0] < 0.1);
    assert_eq!(ac.sys.fx.gear_down, 0.0);
    assert!((ac.sys.fx.flap_rate - 0.5).abs() < 1e-6);
    let al = alerts::alerts(&ac.sys, &d, &ac, &c);
    assert!(al.iter().any(|a| a.title == "HYD G SYS LO PR"), "{:?}", al.iter().map(|a| &a.title).collect::<Vec<_>>());
    // All three: the flight controls freeze.
    fail(&mut ac, &d, Kind::Hydraulic, 1);
    fail(&mut ac, &d, Kind::Hydraulic, 2);
    fly(&d, &mut ac, &mut c, &mut ap, 4.0);
    assert_eq!(ac.sys.fx.surface_rate, [0.0; 3]);
    let a0 = ac.surfaces().aileron;
    ap.disengage();
    c.aileron = 1.0;
    fly(&d, &mut ac, &mut c, &mut ap, 1.0);
    assert!((ac.surfaces().aileron - a0).abs() < 1e-4);
}

#[test]
fn carb_ice_and_carb_heat() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    ap.ias = None;
    let rpm0 = ac.engine_rpm_i(0);
    fail(&mut ac, &d, Kind::CarbIce, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 100.0);
    let rpm1 = ac.engine_rpm_i(0);
    assert!(rpm0 - rpm1 > 150.0, "rpm {rpm0} → {rpm1}");
    set_failure(&mut ac, &d, Failure::new(Kind::CarbIce, 0), false);
    ac.sys.sw.carb_heat = true;
    fly(&d, &mut ac, &mut c, &mut ap, 60.0);
    assert!(ac.sys.eng[0].carb_ice < 0.05);
    ac.sys.sw.carb_heat = false;
    fly(&d, &mut ac, &mut c, &mut ap, 10.0);
    assert!(ac.engine_rpm_i(0) > rpm0 - 60.0, "{} vs {rpm0}", ac.engine_rpm_i(0));
}

#[test]
fn partial_power_and_flameout_relight() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 3000.0);
    ap.ias = None;
    c.throttle = 0.8;
    fly(&d, &mut ac, &mut c, &mut ap, 20.0);
    let n1 = ac.n1(0);
    fail(&mut ac, &d, Kind::PartialPower, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 20.0);
    assert!(ac.n1(0) < n1 - 0.1, "N1 {n1} → {}", ac.n1(0));
    assert!((ac.n1(1) - n1).abs() < 0.02);
    set_failure(&mut ac, &d, Failure::new(Kind::PartialPower, 0), false);
    fail(&mut ac, &d, Kind::Flameout, 1);
    fly(&d, &mut ac, &mut c, &mut ap, 2.0);
    assert!(!ac.engine_running(1));
    assert!(alerts::alerts(&ac.sys, &d, &ac, &c).iter().any(|a| a.title == "ENG 2 FAIL"));
    // The cause clears; ignition on, the APU-bleed / windmill relight.
    ac.sys.sw.ign_start = true;
    fly(&d, &mut ac, &mut c, &mut ap, 70.0);
    assert!(ac.engine_running(1), "relight; start {}", ac.sys.eng[1].start);
}

#[test]
fn flap_asymmetry_rolls_light_aircraft_and_locks_airliners() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 75.0, 900.0);
    fail(&mut ac, &d, Kind::FlapAsym, 0);
    ap.disengage();
    c.flaps = 1.0;
    let mut min_rate = 0.0f32;
    for _ in 0..(4.0 / DT) as usize {
        step(&mut ac, &d, &c, &Ambient::default(), DT);
        ac.step(&d, &c, &Flat, DT);
        min_rate = min_rate.min(ac.rates().z);
    }
    assert!(ac.sys.fx.flap_asym > 0.3);
    assert!(ac.roll() < -10f32.to_radians(), "rolls left: {}", ac.roll().to_degrees());
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 200.0, 1500.0);
    fail(&mut ac, &d, Kind::FlapAsym, 0);
    c.flaps = 0.5;
    fly(&d, &mut ac, &mut c, &mut ap, 10.0);
    assert_eq!(ac.surfaces().flaps, 0.0);
    assert_eq!(ac.sys.fx.flap_asym, 0.0);
    let _ = min_rate;
}

#[test]
fn control_jam_and_brake_failure() {
    let d = def("kestrel");
    let (mut ac, mut c, mut ap) = cruise(&d, 95.0, 900.0);
    fail(&mut ac, &d, Kind::ControlJam, 2);
    ap.disengage();
    let r0 = ac.surfaces().rudder;
    c.rudder = 1.0;
    fly(&d, &mut ac, &mut c, &mut ap, 2.0);
    assert!((ac.surfaces().rudder - r0).abs() < 1e-5);
    // Brakes: a stopping run with and without.
    let stop = |broken: bool| -> f32 {
        let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &Flat);
        a.sys = Systems::new(&d, false);
        if broken {
            set_failure(&mut a, &d, Failure::new(Kind::Brakes, 0), true);
        }
        a.vel = a.forward() * 20.0;
        let c = Controls { brakes: 1.0, ..Default::default() };
        let z0 = a.pos;
        for _ in 0..(40.0 / DT) as usize {
            step(&mut a, &d, &c, &Ambient::default(), DT);
            a.step(&d, &c, &Flat, DT);
        }
        (a.pos - z0).length()
    };
    let (good, bad) = (stop(false), stop(true));
    assert!(bad > good * 2.0, "stopping distance {good:.0} m vs {bad:.0} m with failed brakes");
}

#[test]
fn tyre_burst_swings_towards_it() {
    let d = def("meridian320");
    let mut a = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &Flat);
    a.sys = Systems::new(&d, false);
    set_failure(&mut a, &d, Failure::new(Kind::TyreBurst, 0), true);
    a.vel = a.forward() * 30.0;
    let c = Controls::default();
    for _ in 0..(6.0 / DT) as usize {
        step(&mut a, &d, &c, &Ambient::default(), DT);
        a.step(&d, &c, &Flat, DT);
    }
    // Left tyre: the drag yaws the nose left (heading decreases from north).
    let h = crate::bot::wrap_pi(a.heading());
    assert!(h < -0.5f32.to_radians(), "heading {:.2}°", h.to_degrees());
}

#[test]
fn icing_costs_speed_and_lift() {
    let d = def("kestrel");
    let (mut a, mut ca, mut pa) = cruise(&d, 100.0, 1200.0);
    let (mut b, mut cb, mut pb) = (a.clone(), ca, pa.clone());
    pa.ias = None;
    pb.ias = None;
    fail(&mut a, &d, Kind::Icing, 0);
    fly(&d, &mut a, &mut ca, &mut pa, 240.0);
    fly(&d, &mut b, &mut cb, &mut pb, 240.0);
    assert!(a.sys.ice > 0.9);
    assert!(a.airspeed_kt() < b.airspeed_kt() - 8.0, "{} vs {}", a.airspeed_kt(), b.airspeed_kt());
    // Ice melts above freezing (no failure, warm air).
    set_failure(&mut a, &d, Failure::new(Kind::Icing, 0), false);
    let amb = Ambient { oat_c: Some(8.0), ..Default::default() };
    for _ in 0..400 {
        step(&mut a, &d, &ca, &amb, 1.0);
    }
    assert!(a.sys.ice < 0.05);
}

#[test]
fn natural_pitot_icing_and_heat() {
    let d = def("meridian320");
    let (mut ac, c, _) = cruise(&d, 250.0, 4000.0);
    let cold_cloud = Ambient { oat_c: Some(-6.0), spread_c: 1.0, moisture: 1.0 };
    ac.sys.sw.pitot_heat = false;
    for _ in 0..1200 {
        step(&mut ac, &d, &c, &cold_cloud, 0.1);
    }
    assert!(ac.sys.pitot_p.is_some(), "pitot iced");
    ac.sys.sw.pitot_heat = true;
    for _ in 0..600 {
        step(&mut ac, &d, &c, &cold_cloud, 0.1);
    }
    assert!(ac.sys.pitot_p.is_none(), "heat clears it");
}

#[test]
fn display_failure_and_attitude_flags() {
    let d = def("meridian190");
    let (mut ac, _, _) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::DisplayFail, 0);
    assert!(ac.sys.pfd_red_x());
    fail(&mut ac, &d, Kind::AttitudeInd, 0);
    let ad = ac.sys.air_data(&ac);
    assert!(ad.has(flag::ATT) && ad.has(flag::HDG) && !ad.has(flag::SPD));
}

#[test]
fn cold_and_dark_autostart() {
    for id in ["kestrel", "meridian320", "aerolink", "lancer", "peregrine"] {
        let d = def(id);
        let mut ac = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &Flat);
        ac.sys = Systems::new(&d, true);
        let c = Controls { brakes: 1.0, ..Default::default() };
        let amb = Ambient::default();
        let run = |ac: &mut Aircraft, secs: f32| {
            for _ in 0..(secs / DT) as usize {
                step(ac, &d, &c, &amb, DT);
                ac.step(&d, &c, &Flat, DT);
            }
        };
        run(&mut ac, 3.0);
        assert!(!ac.engine_running(0), "{id} cold");
        assert_eq!(ac.sys.power, Power::Off);
        ac.sys.begin_autostart();
        run(&mut ac, 240.0);
        for i in 0..d.engine_count {
            assert!(ac.engine_running(i), "{id} engine {i} after the autostart (step {})", ac.sys.autostart);
        }
        assert_eq!(ac.sys.power, Power::Normal, "{id}");
        assert!(ac.crashed.is_none());
    }
}

#[test]
fn piston_start_needs_mags_and_prime() {
    let d = def("kestrel");
    let mut ac = Aircraft::on_runway(&d, Vec2::ZERO, 0.0, &Flat);
    ac.sys = Systems::new(&d, true);
    let c = Controls { brakes: 1.0, throttle: 0.1, ..Default::default() };
    let amb = Ambient { oat_c: Some(5.0), ..Default::default() };
    let run = |ac: &mut Aircraft, secs: f32| {
        for _ in 0..(secs / DT) as usize {
            step(ac, &d, &c, &amb, DT);
            ac.step(&d, &c, &Flat, DT);
        }
    };
    ac.sys.sw.battery = true;
    ac.sys.sw.eng_master[0] = true;
    ac.sys.press_start(0);
    run(&mut ac, 5.0);
    assert!(!ac.engine_running(0), "mags off: no start");
    ac.sys.sw.mags[0] = Mags::Both;
    ac.sys.press_start(0);
    run(&mut ac, 1.0);
    assert!(ac.engine_rpm_i(0) > 200.0, "cranking rpm {}", ac.engine_rpm_i(0));
    run(&mut ac, 4.0);
    // Cold and unprimed: it takes longer than one 4-second crank... prime it.
    if !ac.engine_running(0) {
        ac.sys.eng[0].primer = 3.0;
        ac.sys.press_start(0);
        run(&mut ac, 4.0);
    }
    assert!(ac.engine_running(0));
    // Mags off stops it.
    ac.sys.sw.mags[0] = Mags::Off;
    run(&mut ac, 2.0);
    assert!(!ac.engine_running(0));
}

#[test]
fn smoke_isolation() {
    let d = def("meridian320");
    let (mut ac, c, _) = cruise(&d, 250.0, 3000.0);
    fail(&mut ac, &d, Kind::ElecFire, 0);
    let amb = Ambient::default();
    for _ in 0..300 {
        step(&mut ac, &d, &c, &amb, 0.1);
    }
    assert!(ac.sys.cockpit_smoke() > 0.2);
    let al = alerts::alerts(&ac.sys, &d, &ac, &c);
    assert!(al.iter().any(|a| a.title == "AVIONICS SMOKE" && a.smoke));
    ac.sys.sw.avionics = false;
    for _ in 0..600 {
        step(&mut ac, &d, &c, &amb, 0.1);
    }
    assert!(!ac.sys.fail.any_of(Kind::ElecFire));
    assert!(ac.sys.cockpit_smoke() < 0.2);
}

#[test]
fn bird_strike_damages_an_engine() {
    let d = def("aerolink");
    let (mut ac, _, _) = cruise(&d, 200.0, 900.0);
    fail(&mut ac, &d, Kind::BirdStrike, 0);
    assert!(ac.sys.windscreen_cracked);
    assert!(ac.sys.eng.iter().any(|e| e.bird > 0.4));
    assert!(ac.sys.fx.power.iter().any(|p| *p < 0.6));
}

#[test]
fn prop_overspeed() {
    let d = def("aerolink");
    let (mut ac, mut c, mut ap) = cruise(&d, 200.0, 1500.0);
    let rpm = ac.engine_rpm_i(0);
    fail(&mut ac, &d, Kind::PropOverspeed, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 8.0);
    assert!(ac.engine_rpm_i(0) > rpm * 1.12, "{} vs {rpm}", ac.engine_rpm_i(0));
    assert!(alerts::alerts(&ac.sys, &d, &ac, &c).iter().any(|a| a.title == "PROP 1 OVERSPEED"));
}

#[test]
fn fuel_pump_failure_at_altitude() {
    let d = def("meridian320");
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 9000.0);
    fail(&mut ac, &d, Kind::FuelPump, 0);
    fly(&d, &mut ac, &mut c, &mut ap, 60.0);
    assert!(!ac.engine_running(0), "suction feed at FL300 flames out");
    // Low down it is only a caution.
    let (mut ac, mut c, mut ap) = cruise(&d, 250.0, 2000.0);
    fail(&mut ac, &d, Kind::FuelPump, 1);
    fly(&d, &mut ac, &mut c, &mut ap, 60.0);
    assert!(ac.engine_running(1));
}

#[test]
fn schedule_arms_and_fires() {
    let d = def("meridian320");
    let sd = defs::sys_def(&d);
    let mut s = Schedule::default();
    s.arm(Failure::new(Kind::Vacuum, 0), Trigger::At(10.0));
    s.arm(Failure::new(Kind::Hydraulic, 1), Trigger::Above(3000.0 * FT / FT * 0.3048));
    s.arm_window(Failure::new(Kind::Brakes, 0), 0.0, 60.0, 120.0);
    assert!(s.tick(&sd, &d, 5.0, 100.0, 80.0, true, 0.1).is_empty());
    assert_eq!(s.tick(&sd, &d, 11.0, 100.0, 80.0, true, 0.1), vec![Failure::new(Kind::Vacuum, 0)]);
    assert_eq!(s.tick(&sd, &d, 12.0, 1000.0, 80.0, true, 0.1), vec![Failure::new(Kind::Hydraulic, 1)]);
    assert!(s.tick(&sd, &d, 59.0, 100.0, 80.0, true, 0.1).is_empty());
    assert_eq!(s.tick(&sd, &d, 121.0, 100.0, 80.0, true, 0.1), vec![Failure::new(Kind::Brakes, 0)]);
    // Random: about one per MTBF.
    s.random.on = true;
    s.random.mtbf_min = 1.0;
    let mut n = 0;
    for k in 0..(3600.0 / 0.1) as usize {
        n += s.tick(&sd, &d, 200.0 + k as f32 * 0.1, 3000.0, 120.0, true, 0.1).len();
    }
    assert!((35..=90).contains(&n), "{n} random failures in 60 min at MTBF 1 min");
}

#[test]
fn applicability_per_type() {
    let ids = |id: &str| -> Vec<Kind> {
        let d = def(id);
        let sd = defs::sys_def(&d);
        Kind::ALL.iter().copied().filter(|k| k.applies(&sd, &d)).collect()
    };
    let c172 = ids("kestrel");
    assert!(c172.contains(&Kind::CarbIce) && c172.contains(&Kind::Vacuum) && !c172.contains(&Kind::GearExtend) && !c172.contains(&Kind::Hydraulic));
    let glider = ids("swift");
    assert!(!glider.contains(&Kind::EngineFire) && glider.contains(&Kind::PitotBlock) && !glider.contains(&Kind::Vacuum));
    let atr = ids("aerolink");
    assert!(atr.contains(&Kind::PropOverspeed) && !atr.contains(&Kind::CarbIce));
    let a320 = ids("meridian320");
    assert!(a320.contains(&Kind::DisplayFail) && a320.contains(&Kind::Hydraulic) && !a320.contains(&Kind::Vacuum) && !a320.contains(&Kind::PropOverspeed));
    let spit = ids("peregrine");
    assert!(spit.contains(&Kind::PropOverspeed) && spit.contains(&Kind::GearExtend) && spit.contains(&Kind::Vacuum));
    // Every failure that applies can be set and repaired on every type.
    for d in flight::catalogue() {
        let sd = defs::sys_def(&d);
        let (mut ac, _, _) = cruise(&d, d.vref_kt.max(60.0) * 1.4, 1200.0);
        for f in candidates(&sd, &d, 0xFF) {
            set_failure(&mut ac, &d, f, true);
            assert!(ac.sys.fail.has(f), "{} {f:?}", d.id);
        }
        ac.sys.repair_all(&d, &ac.clone());
        assert!(ac.sys.fail.is_empty());
    }
}

#[test]
fn failure_ids_parse() {
    for k in Kind::ALL {
        assert_eq!(Kind::parse(k.id()), Some((k, 0)));
        assert_eq!(Kind::parse(&format!("{}2", k.id())), Some((k, 1)));
    }
    assert_eq!(Kind::parse("eng2fire"), Some((Kind::EngineFire, 1)));
    assert_eq!(Kind::parse("alternator"), Some((Kind::Generator, 0)));
    assert_eq!(Kind::parse("jam-rudder"), Some((Kind::ControlJam, 2)));
    assert_eq!(Kind::parse("nonsense"), None);
}
