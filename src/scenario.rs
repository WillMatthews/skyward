//! Scripted test flights for structures, crashes and effects (Track L):
//! `SKYWARD_SCENARIO=<name>` with the usual headless `--shot` / `--audio-render`
//! flags. Each scenario positions the aircraft once (relative to the start
//! runway where it matters) and then flies a simple control law.
//!
//! | name | what |
//! |---|---|
//! | `pullup` | hard pull for 4 s from t = 2 s (wing flex) |
//! | `overg` | re-launch at 150 kt, full back stick (C172: wing separation) |
//! | `bellyland` | gear-up landing on the start runway |
//! | `hardland` | ~1.4x design sink rate: the gear collapses |
//! | `crash` | 25° dive into the ground beside the runway |
//! | `transonic` | fast low pass at Mach 0.98 with reheat (vapour cone) |
//! | `cruise` | hold heading and altitude (contrails) |
//! | `inverted` | aerobatics: roll inverted at 130 kt / 300 m over the field and hold it (smoke on with the aerobat's smoke system) |

use crate::flight::{Aircraft, KT};
use crate::sim::Sim;
use crate::world::World;
use glam::{vec2, Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: String,
    pub t: f32,
    started: bool,
}

impl Scenario {
    pub fn from_env() -> Option<Scenario> {
        let name = std::env::var("SKYWARD_SCENARIO").ok()?.trim().to_ascii_lowercase();
        if name.is_empty() {
            return None;
        }
        Some(Scenario { name, t: 0.0, started: false })
    }
    pub fn restart(&mut self) {
        self.t = 0.0;
        self.started = false;
    }
}

/// Runway frame: threshold, direction, length.
fn runway(sim: &Sim) -> (Vec2, Vec2, f32) {
    match &sim.runway {
        Some(r) => {
            let d = (r.b - r.a).normalize_or(vec2(0.0, -1.0));
            (r.a, d, r.a.distance(r.b))
        }
        None => (vec2(sim.ac.pos.x, sim.ac.pos.z), vec2(0.0, -1.0), 2000.0),
    }
}

/// Height above the ground under the aircraft.
fn sc_alt(world: &World, ac: &Aircraft) -> f32 {
    ac.pos.y - crate::flight::Environment::ground_height(world, vec2(ac.pos.x, ac.pos.z))
}

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// Air start on the extended centreline `back` m before the threshold at `agl` m.
fn approach(sim: &mut Sim, world: &World, back: f32, agl: f32, kt: f32) {
    let (a, d, _) = runway(sim);
    let p = a - d * back;
    let gh = crate::flight::Environment::ground_height(world, p);
    let def = sim.def().clone();
    sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + agl, p.y), heading_of(d), kt * KT);
    sim.ctl = crate::flight::trimmed_controls_at(&def, kt * KT, gh + agl);
    sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
}

pub fn control(sim: &mut Sim, world: &World, dt: f32) {
    let Some(mut sc) = sim.scenario.take() else { return };
    if sim.ac.crashed.is_none() {
        sc.t += dt * sim.warp as f32;
    }
    let def = sim.def().clone();
    let first = !sc.started;
    sc.started = true;
    sim.bot = None;
    sim.ap.engaged = false;
    let ac = &sim.ac;
    let vs = ac.vertical_speed();
    let agl = ac.agl(world);
    match sc.name.as_str() {
        "pullup" => {
            if first {
                sim.level_hold = None;
            }
            sim.ctl.elevator = if (2.0..6.0).contains(&sc.t) { 1.0 } else if sc.t >= 6.0 { -0.2 * (sim.ac.pitch() - 0.05) * 5.0 } else { 0.0 };
            sim.ctl.aileron = (-2.0 * sim.ac.roll()).clamp(-1.0, 1.0);
        }
        "overg" => {
            if first {
                let p = sim.ac.pos;
                let h = sim.ac.heading();
                sim.ac = Aircraft::airborne(&def, p, h, 150.0 * KT);
                sim.ctl = crate::flight::trimmed_controls_at(&def, 150.0 * KT, p.y);
                sim.level_hold = None;
            }
            sim.ctl.elevator = if sc.t > 1.5 { 1.0 } else { 0.0 };
            sim.ctl.aileron = if sc.t < 1.5 { (-2.0 * sim.ac.roll()).clamp(-1.0, 1.0) } else { 0.0 };
        }
        "bellyland" | "hardland" => {
            let belly = sc.name == "bellyland";
            let vref = def.vref_kt;
            if first {
                sim.level_hold = None;
                approach(sim, world, 1200.0, 45.0, vref * 1.05);
                sim.set_flap_detent(def.flap_detents.len().saturating_sub(1));
                sim.ac.set_flaps_now(sim.ctl.flaps);
                if belly && def.retractable_gear {
                    sim.ctl.gear_down = false;
                    sim.ac.retract_gear_now();
                } else {
                    sim.ctl.gear_down = true;
                    sim.ac.extend_gear_now();
                }
            }
            let ac = &sim.ac;
            // Descent: a glide path, then a firm (or not) arrival.
            let target = if belly {
                (-0.06 * agl).clamp(-3.5, -0.9)
            } else {
                -def.max_touchdown_sink * 1.4
            };
            let ias = ac.airspeed() / KT;
            let fbw = def.control_law != crate::flight::ControlLaw::Direct;
            let k = if fbw { 0.35 } else { 0.12 };
            sim.ctl.elevator = (k * (target - vs) - if fbw { 0.0 } else { 0.6 * ac.rates().y }).clamp(-1.0, 1.0);
            sim.ctl.throttle = if ac.on_ground || agl < 3.0 { 0.0 } else { (0.35 + 0.05 * (vref - ias)).clamp(0.0, 1.0) };
            let (_, d, _) = runway(sim);
            let he = crate::bot::wrap_pi(heading_of(d) - ac.heading());
            sim.ctl.aileron = (-2.5 * ac.roll() - 1.0 * he).clamp(-1.0, 1.0);
            sim.ctl.rudder = 0.0;
        }
        "crash" => {
            if first {
                let (a, d, _) = runway(sim);
                // Off to the side of the runway, diving in.
                let side = Vec2::new(-d.y, d.x) * 220.0;
                let p = a + d * 300.0 + side;
                let gh = crate::flight::Environment::ground_height(world, p);
                let v = (def.vref_kt * 1.5).max(def.vs1_kt * 2.2);
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + 260.0, p.y), heading_of(d), v * KT);
                sim.ac.rot = crate::flight::attitude(heading_of(d), (-24f32).to_radians(), 0.15);
                sim.ac.vel = sim.ac.forward() * sim.ac.vel.length();
                sim.ctl = crate::flight::trimmed_controls_at(&def, v * KT, gh + 260.0);
                sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
                sim.level_hold = None;
            }
            sim.ctl.elevator = -0.05;
            sim.ctl.aileron = 0.0;
        }
        "transonic" => {
            if first {
                let p = sim.ac.pos;
                let h = sim.ac.heading();
                let gh = crate::flight::Environment::ground_height(world, vec2(p.x, p.z));
                let alt = gh + std::env::var("SKYWARD_SCENARIO_ALT").ok().and_then(|s| s.parse().ok()).unwrap_or(250.0);
                let a = crate::flight::isa(alt);
                let m = std::env::var("SKYWARD_SCENARIO_MACH").ok().and_then(|s| s.parse().ok()).unwrap_or(0.98f32);
                let ias = crate::flight::cas_from_tas(m * a.speed_of_sound, alt);
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, alt, p.z), h, ias);
                sim.ctl = crate::flight::trimmed_controls_at(&def, ias, alt);
                sim.level_hold = Some((h, alt));
            }
            sim.ctl.throttle = 1.0;
        }
        "inverted" => {
            if first {
                let (a, d, _) = runway(sim);
                let p = a + d * 400.0;
                let gh = crate::flight::Environment::ground_height(world, p);
                let kt = 130.0;
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + 300.0, p.y), heading_of(d), kt * KT);
                sim.ctl = crate::flight::trimmed_controls_at(&def, kt * KT, gh + 300.0);
                sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
                sim.level_hold = None;
                sim.vfx.smoke = def.smoke_system;
            }
            // Half roll to inverted, then hold the height with forward stick.
            let ac = &sim.ac;
            let target_roll = if sc.t > 1.0 { std::f32::consts::PI } else { 0.0 };
            let err = crate::bot::wrap_pi(ac.roll() - target_roll);
            sim.ctl.aileron = (-2.0 * err - 0.3 * ac.rates().x).clamp(-1.0, 1.0);
            let h_err = sc_alt(world, ac) - 300.0;
            let inv = ac.up().y < 0.0;
            let pitch = (-0.02 * h_err - 0.08 * vs).clamp(-0.6, 0.6);
            sim.ctl.elevator = if inv { -(0.3 + pitch) } else { 0.1 + pitch }.clamp(-1.0, 1.0);
            sim.ctl.throttle = 0.85;
        }
        "cruise" if first => {
            sim.level_hold = Some((sim.ac.heading(), sim.ac.pos.y));
        }
        _ => {}
    }
    sim.scenario = Some(sc);
}
