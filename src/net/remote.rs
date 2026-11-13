//! Another player's aircraft: interpolated pose, the real model animated
//! from the network state, its lights, and what the map / ND / audio need.

use crate::flight::{AircraftDef, EngineKind};
use crate::models::AircraftModel;
use glam::{Mat4, Quat, Vec3, vec3};
use skyward_net::interp::{Interpolator, Sampled};
use skyward_net::proto::{AcState, flags, lights};

/// Remote aircraft farther than this are not drawn (they still show on the map).
pub const DRAW_DIST: f32 = 40_000.0;

pub struct Remote {
    pub id: u16,
    pub name: String,
    /// Aircraft id as sent (may be one this build does not have).
    pub aircraft: String,
    /// Catalogue index of the model drawn (the sent type, or a stand-in).
    pub def: usize,
    pub spectating: bool,
    pub interp: Interpolator,
    /// This frame's interpolated state.
    pub cur: Option<Sampled>,
    /// This frame, in the current world frame.
    pub pos: Vec3,
    pub rot: Quat,
    pub vel: Vec3,
    pub model: Option<AircraftModel>,
    rig: Option<crate::lights::AircraftLightRig>,
    /// Bounding radius, m.
    pub size: f32,
    #[cfg(not(target_arch = "wasm32"))]
    loading: Option<std::sync::mpsc::Receiver<AircraftModel>>,
    pub crashed: bool,
    /// Round trip of the last ping, from the player list (ms); 0 = unknown.
    pub ping_ms: u16,
}

impl Remote {
    pub fn new(id: u16, name: String, aircraft: String, defs: &[AircraftDef]) -> Remote {
        let def = defs.iter().position(|d| d.id == aircraft).unwrap_or(0);
        let mut r = Remote {
            id,
            name,
            aircraft,
            def,
            spectating: false,
            interp: Interpolator::default(),
            cur: None,
            pos: Vec3::ZERO,
            rot: Quat::IDENTITY,
            vel: Vec3::ZERO,
            model: None,
            rig: None,
            size: defs.get(def).map(|d| d.wingspan * 0.5).unwrap_or(6.0).max(1.0),
            #[cfg(not(target_arch = "wasm32"))]
            loading: None,
            crashed: false,
            ping_ms: 0,
        };
        r.load(defs);
        r
    }

    /// (Re)load the model for the current type — off the main thread (the
    /// textures take a moment to decode the first time).
    pub fn load(&mut self, defs: &[AircraftDef]) {
        self.def = defs.iter().position(|d| d.id == self.aircraft).unwrap_or(0);
        self.model = None;
        self.rig = None;
        let Some(d) = defs.get(self.def).cloned() else { return };
        let build = move || {
            let mut m = crate::models::build(d.id, d.eye);
            m.nose_gear = d.gear.iter().find(|g| matches!(g.kind, crate::flight::ContactKind::Wheel { steerable: true, .. }) && g.pos.z > 0.0).map(|g| g.pos);
            // Structural parts: pieces another player has lost are hidden.
            if let Some(a) = m.asset.clone() {
                m.parts = Some(crate::models::classify(&a, &d));
            }
            m
        };
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (tx, rx) = std::sync::mpsc::channel();
            let spawned = std::thread::Builder::new()
                .name("mp-model".into())
                .spawn(move || {
                    let _ = tx.send(build());
                })
                .is_ok();
            if spawned {
                self.loading = Some(rx);
            }
        }
        #[cfg(target_arch = "wasm32")]
        self.set_model(build(), defs);
    }

    fn set_model(&mut self, m: AircraftModel, defs: &[AircraftDef]) {
        self.rig = Some(crate::lights::AircraftLightRig::from_model(&m));
        let span = defs.get(self.def).map(|d| d.wingspan * 0.5).unwrap_or(0.0);
        self.size = m.radius.max(span).max(0.3);
        self.model = Some(m);
    }

    /// Pick up a model that finished loading.
    pub fn poll_model(&mut self, defs: &[AircraftDef]) {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(rx) = &self.loading {
            if let Ok(m) = rx.try_recv() {
                self.loading = None;
                self.set_model(m, defs);
            }
        }
        let _ = defs;
    }

    /// Sample the interpolator at server time `t` and place the aircraft in
    /// the current world frame.
    pub fn sample(&mut self, t: f64, dt: f64, proj: &crate::osm::Proj) {
        self.cur = self.interp.sample(t, dt);
        if let Some(s) = &self.cur {
            let st = &s.state;
            let (x, z) = proj.to_world64(st.lat, st.lon);
            self.pos = vec3(x as f32, st.alt, z as f32);
            self.rot = Quat::from_xyzw(st.rot[0], st.rot[1], st.rot[2], st.rot[3]).normalize();
            self.vel = Vec3::from(st.vel);
            self.crashed = st.has(flags::CRASHED);
        }
    }

    /// Drawn this frame? (Has a state, not spectating, not stale.)
    pub fn visible(&self) -> bool {
        self.cur.as_ref().is_some_and(|s| !s.stale && !s.state.has(flags::SPECTATING))
    }

    pub fn state(&self) -> Option<&AcState> {
        self.cur.as_ref().map(|s| &s.state)
    }

    pub fn heading(&self) -> f32 {
        let f = self.rot * Vec3::Z;
        f.x.atan2(-f.z)
    }

    /// Animate the model and return its draw list (eye-relative); light
    /// sprites go to `lights`, the afterburner flame to `flames`.
    pub fn frame(
        &mut self,
        defs: &[AircraftDef],
        eye: Vec3,
        time: f32,
        lights_out: &mut Vec<crate::lights::LightInst>,
        flames: &mut crate::scene::MeshData,
    ) -> Option<crate::gfx_aircraft::AircraftFrame> {
        if !self.visible() || self.pos.distance(eye) > DRAW_DIST {
            return None;
        }
        let st = *self.state()?;
        let def = defs.get(self.def)?;
        let m = self.model.as_mut()?;
        m.update_channels(time, &|name| channel(name, &st, def, time));
        let body = Mat4::from_rotation_translation(self.rot, self.pos - eye);
        let mut f = m.frame(body, false)?;
        if f.parts.is_some() {
            // Separated parts (wings, tail, gear...) are not drawn.
            let mut d = crate::gfx_aircraft::AcDynamics::default();
            let up = self.rot * Vec3::Y;
            d.up = [up.x, up.y, up.z, time];
            for p in crate::damage::Part::ALL {
                if st.lost_parts & p.bit() != 0 {
                    d.parts[p.idx()] = Mat4::from_scale(Vec3::ZERO).to_cols_array_2d();
                    d.part_flex[p.idx() / 4][p.idx() % 4] = 0.0;
                }
            }
            f.dynamics = Some(Box::new(d));
        }
        // Lights: the switches from the network, flashing on the shared clock.
        if let Some(rig) = &self.rig {
            let mut r = rig.clone();
            if st.lights & lights::NAV == 0 {
                r.nav.clear();
            }
            if st.lights & lights::BEACON == 0 {
                r.beacons.clear();
            }
            if st.lights & lights::STROBE == 0 {
                r.strobes.clear();
            }
            let l = crate::lights::AircraftLights::build(&r, self.pos - eye, self.rot, time, st.lights & lights::LANDING != 0, st.lights & lights::TAXI != 0);
            lights_out.extend_from_slice(&l.sprites[..l.n_sprites]);
        }
        // Afterburner flame (as the player's, sim.rs).
        if def.has_afterburner && st.afterburner > 0.02 {
            let ab = st.afterburner;
            let flick = 1.0 + 0.06 * (time * 37.0 + self.id as f32).sin() * (time * 23.0).cos();
            let mut local = crate::scene::MeshData::default();
            for &p in def.engine_pos {
                let mut flame = crate::models::flame_mesh(Vec3::ZERO, 0.42);
                for v in &mut flame.verts {
                    v.uv[0] *= ab * flick;
                }
                let len = 0.55 + 0.45 * ab;
                local.append(&flame, Mat4::from_translation(p) * Mat4::from_scale(vec3(1.0, 1.0, len * flick)));
            }
            for v in &mut local.verts {
                for c in &mut v.col {
                    *c = c.max(0.0).powf(1.0 / 2.2);
                }
            }
            flames.append(&local, body);
        }
        Some(f)
    }
}

/// The model's animation channels from a network state (FlightGear semantics,
/// as `models::channel_value` for the player's aircraft).
pub fn channel(name: &str, s: &AcState, def: &AircraftDef, time: f32) -> f32 {
    let b = |x: bool| if x { 1.0 } else { 0.0 };
    let on_ground = s.has(flags::ON_GROUND);
    let idx = |pre: &str, suf: &str| -> Option<usize> { name.strip_prefix(pre)?.strip_suffix(suf)?.parse().ok() };
    let r = &s.rotor;
    match name {
        "rotor_rpm" => return r.nr_rpm,
        "rotor_nr_pct" => return r.nr_pct,
        "tail_rpm" => return r.tail_rpm,
        "rotor_coning_deg" => return r.coning_deg,
        "rotor_collective_deg" => return r.collective_deg,
        "rotor_tilt_aft_deg" => return r.tilt_aft_deg,
        "rotor_tilt_right_deg" => return r.tilt_right_deg,
        "tail_pitch_deg" => return r.tail_pitch_deg,
        "elevator" | "ctl_elevator" => return -s.elevator,
        "aileron_l" | "ctl_aileron" => return s.aileron,
        "aileron_r" => return -s.aileron,
        "rudder" | "ctl_rudder" => return s.rudder,
        "flaps" | "ctl_flaps" => return s.flaps,
        "spoilers" | "ctl_spoilers" => return s.spoilers,
        "gear_pos" => return s.gear,
        "ctl_gear" => return b(s.has(flags::GEAR_LEVER)),
        "wow" => return b(on_ground),
        "steer_norm" => return if on_ground { s.rudder } else { 0.0 },
        "steer_deg" => return if on_ground { s.rudder * def.nose_steer_deg } else { 0.0 },
        "afterburner" => return s.afterburner,
        "throttle" => return s.throttle,
        "brakes" => return s.brakes,
        "nav" => return b(s.lights & lights::NAV != 0),
        "beacon_sw" => return b(s.lights & lights::BEACON != 0),
        "strobe_sw" => return b(s.lights & lights::STROBE != 0),
        "beacon" => return b(s.lights & lights::BEACON != 0 && crate::lights::place::beacon_on(time)),
        "strobe" => return b(s.lights & lights::STROBE != 0 && crate::lights::place::strobe_on(time)),
        "landing_light" => return b(s.lights & lights::LANDING != 0),
        "taxi_light" => return b(s.lights & lights::TAXI != 0),
        "time" => return time,
        "ias_kt" => return s.tas * 1.943_844,
        "alt_ft" => return s.alt / 0.3048,
        _ => {}
    }
    let n = (s.n_engines as usize).max(1);
    let eng = |k: usize| s.engines[k.min(n - 1)];
    if let Some(k) = idx("eng", "_rpm") {
        return eng(k).rpm;
    }
    if let Some(k) = idx("prop", "_rpm") {
        let r = eng(k).rpm;
        return if def.engine_kind == EngineKind::Piston { r * def.prop_gear } else { r };
    }
    if let Some(k) = idx("eng", "_n1") {
        return eng(k).spool * 100.0;
    }
    if let Some(k) = idx("eng", "_n2") {
        return (eng(k).spool * 100.0 * 0.9 + 10.0).min(105.0);
    }
    if let Some(k) = idx("eng", "_running") {
        return b(eng(k).running);
    }
    if idx("eng", "_reverser").is_some() {
        return s.reverser;
    }
    if idx("gear", "_roll_ms").is_some() {
        return if on_ground { s.tas } else { 0.0 };
    }
    0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channels_from_network_state() {
        let def = crate::flight::find("kestrel").unwrap();
        let mut s = AcState {
            gear: 1.0,
            flaps: 0.5,
            aileron: 0.3,
            elevator: -0.2,
            n_engines: 1,
            lights: lights::NAV | lights::BEACON,
            flags: flags::ON_GROUND,
            rudder: 0.4,
            ..Default::default()
        };
        s.engines[0].rpm = 2300.0;
        s.engines[0].running = true;
        assert_eq!(channel("gear_pos", &s, &def, 0.0), 1.0);
        assert_eq!(channel("flaps", &s, &def, 0.0), 0.5);
        assert_eq!(channel("aileron_l", &s, &def, 0.0), 0.3);
        assert_eq!(channel("aileron_r", &s, &def, 0.0), -0.3);
        assert_eq!(channel("elevator", &s, &def, 0.0), 0.2);
        assert_eq!(channel("wow", &s, &def, 0.0), 1.0);
        assert!(channel("steer_deg", &s, &def, 0.0).abs() > 0.0);
        assert_eq!(channel("eng0_running", &s, &def, 0.0), 1.0);
        assert!((channel("prop0_rpm", &s, &def, 0.0) - 2300.0 * def.prop_gear).abs() < 1e-3);
        // Beacon flashes on the shared clock; strobes are off.
        assert_eq!(channel("beacon", &s, &def, 0.05), 1.0);
        assert_eq!(channel("beacon", &s, &def, 0.5), 0.0);
        assert_eq!(channel("strobe", &s, &def, 0.0), 0.0);
        assert_eq!(channel("nav", &s, &def, 0.0), 1.0);
        // Engines past the count map to the last one.
        assert_eq!(channel("eng3_rpm", &s, &def, 0.0), 2300.0);
        s.flags = 0;
        assert_eq!(channel("steer_deg", &s, &def, 0.0), 0.0);
    }
}
