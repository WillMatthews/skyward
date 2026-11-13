//! The player's aircraft → the network state ([`AcState`]): geodetic
//! position (never the floating-origin frame), attitude and velocities in the
//! local tangent frame, surfaces, engines, lights, damage.

use crate::sim::Sim;
use crate::world::World;
use glam::{Vec3, vec2};
use skyward_net::proto::{AcState, EngineState, MAX_ENGINES, RotorState, flags, lights};

pub fn own_state(sim: &Sim, world: &World, paused: bool, spectating: bool) -> AcState {
    let def = sim.def();
    let ac = &sim.ac;
    let (lat, lon) = world.proj.to_latlon(vec2(ac.pos.x, ac.pos.z));
    let s = ac.surfaces();
    let omega = ac.rot * ac.omega;
    let mut st = AcState {
        t: 0.0,
        lat,
        lon,
        alt: ac.pos.y,
        rot: ac.rot.normalize().to_array(),
        vel: ac.vel.to_array(),
        omega: omega.to_array(),
        aileron: s.aileron,
        elevator: s.elevator,
        rudder: s.rudder,
        flaps: s.flaps,
        spoilers: s.spoilers,
        gear: 1.0 - s.gear,
        reverser: s.reverser,
        afterburner: if def.has_afterburner { s.afterburner } else { 0.0 },
        throttle: sim.ctl.throttle,
        brakes: sim.ctl.brakes,
        tas: ac.tas(),
        ..Default::default()
    };
    let n = if def.has_engine { ac.engine_count(def).min(MAX_ENGINES) } else { 0 };
    st.n_engines = n as u8;
    let thrust_max = crate::audio::capture::profile_for(def, sim.size).thrust_max.max(1.0);
    for i in 0..n {
        st.engines[i] = EngineState {
            rpm: ac.engine_rpm_i(i),
            spool: ac.n1(i),
            thrust: (ac.engine_thrust(i) / thrust_max).clamp(-1.0, 1.0),
            running: ac.engine_running(i) && ac.crashed.is_none(),
        };
    }
    if def.rotor.is_some() {
        let o = &ac.rotor.out;
        st.flags |= flags::ROTORCRAFT;
        st.rotor = RotorState {
            nr_rpm: o.nr_rpm,
            nr_pct: o.nr_pct,
            tail_rpm: o.tail_rpm,
            coning_deg: o.coning_deg,
            collective_deg: o.collective_deg,
            tilt_aft_deg: o.tilt_deg[0],
            tilt_right_deg: o.tilt_deg[1],
            tail_pitch_deg: o.tail_pitch_deg,
        };
    }
    // Lights as the model shows them (sim.rs `aircraft_mesh`).
    let running = (0..ac.engine_count(def)).any(|i| ac.engine_running(i)) && def.has_engine;
    let drone = def.category == crate::flight::Category::Drone;
    if !drone {
        st.lights |= lights::NAV;
        if running {
            st.lights |= lights::BEACON;
        }
        if !ac.on_ground || sim.ctl.throttle > 0.5 {
            st.lights |= lights::STROBE;
        }
        if sim.landing_lights_on() {
            st.lights |= lights::LANDING;
            if ac.on_ground {
                st.lights |= lights::TAXI;
            }
        }
    }
    if ac.on_ground {
        st.flags |= flags::ON_GROUND;
    }
    if ac.crashed.is_some() {
        st.flags |= flags::CRASHED;
    }
    if paused {
        st.flags |= flags::PAUSED;
    }
    if spectating {
        st.flags |= flags::SPECTATING;
    }
    if sim.ctl.gear_down {
        st.flags |= flags::GEAR_LEVER;
    }
    // Smoke: a burning engine or a broken airframe trailing.
    if ac.dmg.lost != 0 && ac.crashed.is_none() {
        st.flags |= flags::SMOKE;
    }
    st.lost_parts = ac.dmg.lost;
    st
}

/// The aircraft jumped (restart, new start): more than a physics step could move it.
pub fn teleported(before: Vec3, after: Vec3, dt: f32) -> bool {
    before.distance(after) > 600.0_f32.max(400.0 * dt * 60.0)
}
