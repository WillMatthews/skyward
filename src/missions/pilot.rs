//! A scripted pilot for the missions: flies them through the controls only
//! (stick, rudder, thrust levers, flaps, gear, speed brakes, brakes, cockpit
//! switches) — no physics shortcuts — to prove they are winnable headlessly
//! and to fast-forward to moments for stills (`--mission-bot`).
//!
//! Building blocks: flight-path / pitch and bank inner loops (as the
//! autopilot's, both control laws), speed on thrust or on pitch (glides), an
//! approach planner to any runway end (centreline intercept, 3° path with
//! speed brakes when high, configuration by distance, flare, roll-out with
//! brakes / reversers / centreline steering), a ditching profile, an
//! emergency descent, a rejected take-off, a windshear escape, steering on
//! differential thrust alone, and the memory items (thrust idle + fire
//! handle + agent, alternate gear, MAYDAY).

use super::{MissionRun, Strip, throttles};
use crate::flight::{Aircraft, AircraftDef, ControlLaw, Controls, Environment, GRAVITY, KT};
use glam::{Vec2, vec2};

const FT: f32 = 0.3048;
const NM: f32 = 1852.0;

fn wrap(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}
fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// Requests the pilot makes outside the aircraft.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PilotReq {
    Mayday,
}

/// What the pilot is doing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Plan {
    /// Not decided yet (first step).
    #[default]
    Start,
    /// Take-off roll, continue.
    TakeOff,
    /// Rejected take-off: stop on the runway.
    Reject,
    /// Hold wings level, attitude and thrust (unreliable airspeed: pitch + power).
    PitchPower,
    /// Emergency descent to 10,000 ft.
    Descent,
    /// Approach and land on the target runway (powered, or gliding).
    Land,
    /// Glide straight ahead and ditch.
    Ditch,
    /// Windshear escape, then climb out.
    Escape,
    /// Steer with differential thrust only (no hydraulics), approach and land.
    ThrustOnly,
}

#[derive(Clone, Debug, Default)]
pub struct MissionPilot {
    pub plan: Plan,
    t: f32,
    pitch_i: f32,
    bank_cmd: f32,
    vs_cmd: f32,
    spd_i: f32,
    flare_vs0: f32,
    flaring: bool,
    mayday_t: Option<f32>,
    descent_done: bool,
    /// Differential-thrust loops.
    dt_i: f32,
    pitch_hold: f32,
    last_ias: f32,
    trend: f32,
    /// The runway end chosen for the landing.
    end: Option<String>,
    positioning: bool,
    ditch_hdg: Option<f32>,
    ditch_to: Option<Vec2>,
    ditch_plan_t: f32,
    pub log: Vec<String>,
}

/// Inner-loop gains (as the autopilot's).
fn gains(def: &AircraftDef) -> (f32, f32, f32) {
    if def.mass > 150_000.0 { (3.0, 4.0, 0.25) } else { (2.2, 2.0, 0.35) }
}

impl MissionPilot {
    fn note(&mut self, s: impl Into<String>) {
        let s = s.into();
        if self.log.last() != Some(&s) {
            self.log.push(format!("{:6.1}  {s}", self.t));
        }
    }

    /// Pitch / flight-path and bank loops → elevator, trim, aileron, rudder.
    fn inner(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, gamma_cmd: Option<f32>, pitch_cmd: Option<f32>, bank_t: f32, dt: f32) {
        let (kp, kd, ki) = gains(def);
        let fbw = def.control_law != ControlLaw::Direct;
        let tas = ac.tas().max(30.0);
        let gamma = (ac.vertical_speed() / ac.vel.length().max(1.0)).clamp(-1.0, 1.0).asin();
        let theta = ac.pitch();
        let q = ac.rates().y;
        let phi = ac.roll();
        let mut e = match (gamma_cmd, pitch_cmd) {
            (Some(gc), _) => 1.2 * (gc - gamma),
            (None, Some(pc)) => pc - theta,
            _ => 0.0,
        };
        e = e.min(20f32.to_radians() - theta).max(-12f32.to_radians() - theta);
        if fbw {
            let comp = 1.0 / phi.cos().max(0.5);
            let nz_neutral = gamma.cos() * comp;
            let k = if self.flaring { 2.2 } else { 0.6 };
            let dnz = k * e * tas / GRAVITY - 0.3 * q * tas / GRAVITY;
            let range = if dnz >= 0.0 { (def.nz_max - nz_neutral).max(0.3) } else { (nz_neutral - def.nz_min).max(0.3) };
            ctl.elevator = (dnz / range).clamp(-1.0, 1.0);
        } else {
            let turn = (1.0 / phi.cos().max(0.6) - 1.0) * 0.6;
            let ei = e + 0.15 * turn;
            self.pitch_i = (self.pitch_i + ki * kp * ei * dt).clamp(-def.trim_authority, def.trim_authority);
            ctl.trim = (self.pitch_i / def.trim_authority.max(1e-3)).clamp(-1.0, 1.0);
            // (The flare wants more, and more authority.)
            let (kf, lim) = if self.flaring { (2.0, 0.9) } else { (1.0, 0.6) };
            ctl.elevator = (kf * kp * ei - kd * q).clamp(-lim, lim);
        }
        self.bank_cmd += (bank_t - self.bank_cmd).clamp(-8f32.to_radians() * dt, 8f32.to_radians() * dt);
        let p = ac.rates().x;
        if fbw {
            ctl.aileron = (1.5 * (self.bank_cmd - phi) / def.roll_rate_max.max(0.05)).clamp(-1.0, 1.0);
            ctl.rudder = 0.0;
        } else {
            ctl.aileron = (2.5 * (self.bank_cmd - phi) - 1.5 * p).clamp(-1.0, 1.0);
            ctl.rudder = (3.0 * ac.beta()).clamp(-1.0, 1.0);
        }
    }

    /// Pitch attitude that flies `target` kt on the elevator (glides): PI on
    /// the speed error with speed-trend damping, around the pitch held.
    fn pitch_for_speed(&mut self, ac: &Aircraft, target: f32, dt: f32) -> f32 {
        let ias = ac.airspeed_kt();
        let raw = if self.last_ias > 0.0 { (ias - self.last_ias) / dt.max(1e-3) } else { 0.0 };
        self.last_ias = ias;
        self.trend += (raw - self.trend) * (dt / 1.5).min(1.0);
        let trend = self.trend;
        let e = ias - target;
        let lo = -10f32.to_radians();
        let hi = 12f32.to_radians();
        self.pitch_hold = (self.pitch_hold + 0.03f32.to_radians() * e * dt).clamp(lo, hi);
        (self.pitch_hold + 0.25f32.to_radians() * e + 0.8f32.to_radians() * trend.clamp(-4.0, 4.0)).clamp(lo, hi)
    }

    /// Bank for a heading.
    fn bank_for(ac: &Aircraft, hdg: f32, max_bank: f32) -> f32 {
        let e = wrap(hdg - ac.heading());
        let rate = (0.3 * e).clamp(-3f32.to_radians(), 3f32.to_radians());
        (ac.tas() * rate / GRAVITY).atan().clamp(-max_bank, max_bank)
    }

    /// Thrust for an indicated airspeed (PI), common lever.
    fn speed_on_thrust(&mut self, ac: &Aircraft, ctl: &mut Controls, ias_kt: f32, dt: f32) {
        let e = ias_kt - ac.airspeed_kt();
        self.spd_i = (self.spd_i + 0.004 * e * dt).clamp(-0.3, 0.6);
        ctl.engine_throttle = None;
        ctl.throttle = (0.45 + 0.04 * e + self.spd_i).clamp(0.0, 1.0);
    }

    /// The memory items for what has gone wrong.
    fn memory_items(&mut self, def: &AircraftDef, ac: &mut Aircraft, ctl: &mut Controls, req: &mut Vec<PilotReq>) {
        let n = crate::missions::n_eng(def);
        for i in 0..n {
            let e = ac.sys.eng[i];
            if e.fire > 0.0 && ac.sys.sd.fire_detect {
                // Thrust lever idle, (master off), fire handle, agent after 10 s.
                if !ac.on_ground {
                    ctl.nudge_lever(i, -1.0);
                }
                if !ac.sys.sw.fire_handle[i] {
                    ac.sys.sw.fire_handle[i] = true;
                    ac.sys.sw.eng_master[i] = false;
                    self.note(format!("ENG {} FIRE: fire handle", i + 1));
                }
                if e.fire_t > 10.0 && e.agent == 0 {
                    ac.sys.discharge_agent(i);
                } else if e.fire_t > 40.0 && e.agent == 1 {
                    ac.sys.discharge_agent(i);
                }
            }
        }
        if ac.sys.master_warn || ac.sys.master_caut {
            ac.sys.acknowledge();
        }
        if self.mayday_t.is_some_and(|t| self.t > t) {
            self.mayday_t = Some(f32::INFINITY);
            req.push(PilotReq::Mayday);
            self.note("MAYDAY");
        }
        // Gear lever down but not down: alternate extension.
        if ctl.gear_down && ac.gear_pos() < 0.05 && !ac.on_ground && def.retractable_gear && self.t > 4.0 && !ac.sys.sw.alt_gear {
            ac.sys.sw.alt_gear = true;
            self.note("gear unsafe: alternate extension");
        }
    }

    /// Fly one physics step.
    #[allow(clippy::too_many_arguments)]
    pub fn fly(&mut self, def: &AircraftDef, ac: &mut Aircraft, ctl: &mut Controls, run: &MissionRun, strips: &[Strip], env: &dyn Environment, dt: f32, req: &mut Vec<PilotReq>) {
        self.t += dt;
        if ac.crashed.is_some() {
            return;
        }
        let fired = |id: &str| run.m.events.iter().position(|e| e.id == id).and_then(|k| run.fired[k]).is_some();
        if self.plan == Plan::Start {
            self.plan = match run.m.id.as_str() {
                "v1" => Plan::TakeOff,
                "decompression" => Plan::Descent,
                "pitot" => Plan::PitchPower,
                "river" => Plan::Ditch,
                "microburst" => Plan::Land,
                "hydraulics" | "hydraulics4" => Plan::ThrustOnly,
                _ => Plan::Land,
            };
            self.pitch_hold = ac.pitch();
            self.note(format!("plan {:?}", self.plan));
        }
        // MAYDAY a few seconds after the emergency starts.
        if self.mayday_t.is_none() && run.fired.iter().any(|f| f.is_some()) {
            self.mayday_t = Some(self.t + 8.0);
        }
        self.memory_items(def, ac, ctl, req);
        // Windshear: escape (all plans in the air).
        if run.shear && self.plan != Plan::Escape && !ac.on_ground {
            self.plan = Plan::Escape;
            self.note("WINDSHEAR: escape");
        }
        let target = run.target_strip(strips).cloned();
        match self.plan {
            Plan::Start => {}
            Plan::TakeOff => {
                if fired("fire") && ac.on_ground {
                    self.plan = Plan::Reject;
                    self.note("REJECT");
                }
                ctl.throttle = 1.0;
                ctl.engine_throttle = None;
                if let Some(s) = &target {
                    self.steer_centreline(ac, ctl, s);
                }
            }
            Plan::Reject => {
                ctl.engine_throttle = None;
                ctl.throttle = 0.0;
                if def.has_reverser {
                    ctl.reverse = ac.airspeed_kt() > 40.0;
                }
                ctl.spoilers = 1.0;
                ctl.brakes = 1.0;
                ctl.elevator = -0.1;
                if let Some(s) = &target {
                    self.steer_centreline(ac, ctl, s);
                }
            }
            Plan::PitchPower => {
                // Unreliable speed: 2.5° nose up, climb thrust, wings level; after the probes clear, hold.
                let pitch = 2.5f32.to_radians();
                ctl.engine_throttle = None;
                ctl.throttle = 0.82;
                let hdg = ac.heading();
                self.inner(def, ac, ctl, None, Some(pitch), Self::bank_for(ac, hdg, 0.2), dt);
            }
            Plan::Descent => self.descent(def, ac, ctl, dt),
            Plan::Ditch => self.ditch(def, ac, ctl, env, dt),
            Plan::Land => {
                if let Some(s) = target {
                    self.land(def, ac, ctl, &s, env, dt, false);
                }
            }
            Plan::Escape => {
                // TOGA, pitch towards 15° (less at the stall warning), wings level, no configuration change.
                ctl.engine_throttle = None;
                ctl.throttle = 1.0;
                ctl.spoilers = 0.0;
                let mut pitch = 15f32.to_radians();
                if ac.stall_warning() {
                    pitch = ac.pitch() - 2f32.to_radians();
                }
                let hdg = ac.heading();
                self.inner(def, ac, ctl, None, Some(pitch), Self::bank_for(ac, hdg, 0.05), dt);
                if !run.shear && ac.agl(env) > 1500.0 * FT {
                    // Clear of it: climb on at a safe speed.
                    self.inner(def, ac, ctl, Some(4f32.to_radians()), None, 0.0, dt);
                }
            }
            Plan::ThrustOnly => {
                if let Some(s) = target {
                    self.thrust_only(def, ac, ctl, &s, env, dt);
                }
            }
        }
    }

    fn steer_centreline(&self, ac: &Aircraft, ctl: &mut Controls, s: &Strip) {
        let fwd = vec2(ac.forward().x, ac.forward().z).normalize_or(vec2(0.0, -1.0));
        let (thr, d, _) = s.frame_for(fwd);
        let (_, side) = Strip::local(vec2(ac.pos.x, ac.pos.z), thr, d);
        let he = wrap(heading_of(d) - ac.heading());
        // Gentle at speed (nose-wheel steering and differential braking bite hard).
        let k = (25.0 / ac.airspeed_kt().max(25.0)).min(1.0);
        let track = he - (0.004 * side).clamp(-0.08, 0.08);
        ctl.rudder = ((3.0 * track - 0.9 * ac.rates().z) * (0.4 + 0.6 * k)).clamp(-0.6, 0.6);
        ctl.aileron = 0.0;
    }

    fn descent(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, dt: f32) {
        let level = 10_000.0 * FT - 300.0 * FT;
        if !self.descent_done && ac.pos.y > level + 900.0 * FT {
            // Thrust idle, speed brakes, nose down for ~VMO − 20 kt, a 30° turn off the airway first.
            ctl.engine_throttle = None;
            ctl.throttle = 0.0;
            ctl.spoilers = 1.0;
            // VMO − 25 kt, or MMO − 0.04 up high.
            let a = crate::flight::isa(ac.pos.y).speed_of_sound;
            let m_cas = crate::flight::cas_from_tas((def.mmo - 0.04).max(0.5) * a, ac.pos.y) / KT;
            let target = (def.vne_kt - 25.0).min(m_cas);
            let e = ac.airspeed_kt() - target;
            self.vs_cmd = (self.vs_cmd - 0.4 * dt).max(-35.0);
            let vs = (self.vs_cmd + 0.6 * e).clamp(-35.0, 0.0);
            let g = (vs / ac.tas().max(50.0)).clamp(-0.5, 0.2).asin();
            let hdg = ac.heading();
            self.inner(def, ac, ctl, Some(g), None, Self::bank_for(ac, hdg, 0.3), dt);
        } else {
            if !self.descent_done {
                self.note("levelling at 10,000 ft");
            }
            self.descent_done = true;
            ctl.spoilers = 0.0;
            let err = level - ac.pos.y;
            let vs = (0.08 * err).clamp(-12.0, 8.0);
            let g = (vs / ac.tas().max(50.0)).clamp(-0.4, 0.3).asin();
            self.speed_on_thrust(ac, ctl, 250.0, dt);
            let hdg = ac.heading();
            self.inner(def, ac, ctl, Some(g), None, Self::bank_for(ac, hdg, 0.3), dt);
        }
    }

    fn ditch(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, env: &dyn Environment, dt: f32) {
        let agl = ac.agl(env);
        let full = def.flap_detents.len().saturating_sub(1);
        let fmax = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
        ctl.engine_throttle = None;
        ctl.throttle = 0.0;
        ctl.gear_down = false;
        // Configuration for the water: full flap below 1,500 ft.
        let flaps = if agl < 1500.0 * FT { def.flap_detents[full] / fmax } else { def.flap_detents[1.min(full)] / fmax };
        ctl.flaps = flaps;
        // Pick a touchdown point on open water within the glide (unbroken
        // water from 300 m before it to 900 m after it, the least turning,
        // comfortably short of the maximum reach); re-planned every few
        // seconds until 200 ft, then flown to with the speed brakes.
        let here = vec2(ac.pos.x, ac.pos.z);
        if (self.t - self.ditch_plan_t > 3.0 || self.ditch_to.is_none()) && agl > 200.0 * FT {
            self.ditch_plan_t = self.t;
            let reach = agl * if agl < 1500.0 * FT { 7.0 } else { 10.0 } + 300.0;
            let mut best: Option<(f32, Vec2)> = None;
            for k in -14..=14 {
                let h = ac.heading() + (k as f32 * 5.0).to_radians();
                let d = vec2(h.sin(), -h.cos());
                let mut dist = 900.0;
                while dist < reach {
                    let wet = (-3..=9).all(|i| env.is_water(here + d * (dist + i as f32 * 100.0)));
                    if wet {
                        let score = (dist / reach - 0.7).abs() * 3.0 + (k as f32).abs() * 0.08;
                        if best.is_none_or(|b| score < b.0) {
                            best = Some((score, here + d * dist));
                        }
                    }
                    dist += 250.0;
                }
            }
            if let Some((_, p)) = best {
                self.ditch_to = Some(p);
            }
        }
        if let Some(t) = self.ditch_to {
            let d = t - here;
            if d.length() > 400.0 {
                self.ditch_hdg = Some(heading_of(d));
            }
            // Too high for the point: speed brakes (a ~6° line to it).
            let line = d.length() * 6f32.to_radians().tan() + 20.0;
            ctl.spoilers = if self.flaring { 0.0 } else { ((agl - line) / 80.0).clamp(0.0, 1.0) };
        }
        let hdg = self.ditch_hdg.unwrap_or(ac.heading());
        if agl > 12.0 && !self.flaring {
            // Glide: pitch for speed (green dot clean, VLS + 10 with flap).
            let target = if agl < 1500.0 * FT { def.vref_kt + 8.0 } else { def.vref_kt * 1.32 };
            let pitch = self.pitch_for_speed(ac, target, dt);
            self.inner(def, ac, ctl, None, Some(pitch), Self::bank_for(ac, hdg, 0.35), dt);
        } else {
            // Flare to ~11° nose up, wings level, sink ~ 2–3 ft/s.
            if !self.flaring {
                self.note("ditching flare");
            }
            self.flaring = true;
            let pitch = 11f32.to_radians();
            let vs_err = -0.9 - ac.vertical_speed();
            let p = (pitch + 0.02 * vs_err).min(12.5f32.to_radians());
            self.inner(def, ac, ctl, None, Some(p), 0.0, dt);
        }
    }

    /// Approach and landing on `s` (the end facing the aircraft's side of it).
    #[allow(clippy::too_many_arguments)]
    fn land(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, s: &Strip, env: &dyn Environment, dt: f32, _thrust_only: bool) {
        let p = vec2(ac.pos.x, ac.pos.z);
        // The end to land on, chosen once: the one whose approach side we are on
        // (the target's own end when abeam the runway).
        if self.end.is_none() {
            let d = (s.b - s.a).normalize_or(vec2(0.0, -1.0));
            let (along, _) = Strip::local(p, s.a, d);
            let id = if along > s.length() + 2.0 * NM { s.ident_b.clone() } else { s.ident_a.clone() };
            self.note(format!("landing runway {id}"));
            self.end = Some(id);
        }
        let (thr, dir) = s.end(self.end.as_deref().unwrap_or("")).unwrap_or((s.a, (s.b - s.a).normalize_or(vec2(0.0, -1.0))));
        let (along, side) = Strip::local(p, thr, dir);
        let dist = (-along).max(0.0);
        let agl = ac.agl(env);
        let engines = (0..crate::missions::n_eng(def)).any(|i| ac.engine_running(i)) && def.has_engine;
        let full = def.flap_detents.len().saturating_sub(1);
        let fmax = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
        let flap = |k: usize| def.flap_detents[k.min(full)] / fmax;
        let belly = run_belly(ac);
        if ac.on_ground {
            // Roll-out.
            self.flaring = false;
            ctl.engine_throttle = None;
            ctl.throttle = 0.0;
            ctl.spoilers = 1.0;
            // (Reverse only with every engine turning: asymmetric reverse swings it off the runway.)
            let all = (0..crate::missions::n_eng(def)).all(|i| ac.engine_running(i));
            ctl.reverse = def.has_reverser && ac.airspeed_kt() > 50.0 && engines && all;
            ctl.brakes = if ac.airspeed_kt() < def.vref_kt - 15.0 || ac.pitch() < 1f32.to_radians() { 1.0 } else { 0.4 };
            ctl.elevator = -0.15;
            ctl.aileron = 0.0;
            self.steer_centreline(ac, ctl, s);
            return;
        }
        // ---- configuration by distance
        let vref = def.vref_kt + if ac.sys.flap_lock.is_some() { 25.0 } else { 0.0 };
        let (spd, f) = if dist > 14.0 * NM {
            (if engines { 230.0 } else { def.vref_kt * 1.32 }, 0)
        } else if dist > 8.0 * NM {
            (vref + 45.0, 1)
        } else if dist > 5.0 * NM {
            (vref + 25.0, full.saturating_sub(1).max(1))
        } else {
            // (Gliding: less flap, more speed for the flare.)
            (vref + if engines { 5.0 } else { 18.0 }, full)
        };
        if engines {
            ctl.flaps = flap(f);
            ctl.gear_down = !belly && dist < 9.0 * NM;
        } else {
            // Dead stick: stay clean and fast until the field is made, gravity gear early (it is slow).
            ctl.flaps = if agl < 1000.0 * FT {
                flap(full.saturating_sub(2).max(1))
            } else if dist < 7.0 * NM {
                flap(1)
            } else {
                0.0
            };
            ctl.gear_down = !belly && agl < 3500.0 * FT;
        }
        // ---- lateral: a carrot on the extended centreline
        let fwd_dist = (dist - (2.2 * NM).min(dist * 0.6)).max(0.0);
        let carrot = thr - dir * fwd_dist;
        // Positioning: too close and pointing the wrong way — fly to an 8 NM
        // final first (a downwind / base), then turn in.
        let course_err = wrap(heading_of(dir) - ac.heading()).abs();
        let gate = thr - dir * 8.0 * NM;
        if !self.positioning && dist < 7.0 * NM && course_err > 70f32.to_radians() && agl > 600.0 * FT {
            self.positioning = true;
            self.note("positioning for an 8 NM final");
        }
        if self.positioning && (p.distance(gate) < 1.2 * NM || (dist > 7.5 * NM && course_err < 60f32.to_radians())) {
            self.positioning = false;
        }
        let hdg = if self.positioning {
            heading_of(gate - p)
        } else if dist < 1.5 * NM {
            wrap(heading_of(dir) - (side * 0.004).clamp(-0.15, 0.15))
        } else {
            heading_of(carrot - p)
        };
        // Crab-free: steer on track.
        let gv = vec2(ac.vel.x, ac.vel.z);
        let drift = if gv.length() > 20.0 { wrap(heading_of(gv) - ac.heading()) } else { 0.0 };
        let max_bank = if dist < 3.0 * NM { 0.25 } else { 0.42 };
        let bank = Self::bank_for(ac, wrap(hdg - drift), max_bank);
        // ---- vertical: 3° path to 50 ft over the threshold
        // (Gliding: a steeper 4.5° reference, the speed brakes take off the rest.)
        let slope = if engines { 3f32 } else { 6f32 }.to_radians().tan();
        let path = s.elev + 15.0 + dist * slope;
        let gs = gv.length().max(40.0);
        let above = ac.pos.y - path;
        // Flare higher from a steep, fast descent.
        let flare_h = (crate::autopilot::flare_height(def) + 2.0).max(-ac.vertical_speed() * 3.5).min(60.0);
        // (Heights of the wheels, not the CG.)
        let wheels = agl - def.gear_height.max(0.0);
        if wheels < flare_h || self.flaring {
            if !self.flaring {
                self.flare_vs0 = ac.vertical_speed();
                self.note(format!("flare at {:.0} ft, {:.0} kt", agl / FT, ac.airspeed_kt()));
            }
            self.flaring = true;
            let h = wheels.max(0.0);
            let td = -0.6;
            let vs_t = (td + (self.flare_vs0 - td) * (h / flare_h).clamp(0.0, 1.0)).min(0.0);
            let g = (vs_t / ac.tas().max(40.0)).asin();
            ctl.engine_throttle = None;
            ctl.throttle = if h < 4.0 { 0.0 } else { ctl.throttle * (1.0 - 0.6 * dt) };
            ctl.spoilers = 0.0;
            self.inner(def, ac, ctl, Some(g), None, 0.0, dt);
            return;
        }
        if engines {
            // Powered: the path on pitch, speed on thrust; speed brakes when well above.
            let vs_t = (-gs * 3f32.to_radians().tan() - 0.12 * above).clamp(-20.0, 4.0);
            let vs_t = if dist > 14.0 * NM { vs_t.max(-12.0) } else { vs_t };
            let g = (vs_t / ac.tas().max(50.0)).clamp(-0.35, 0.2).asin();
            self.speed_on_thrust(ac, ctl, spd, dt);
            ctl.spoilers = if above > 250.0 && ac.airspeed_kt() > spd - 5.0 && dist > 3.0 * NM { 1.0 } else { 0.0 };
            self.inner(def, ac, ctl, Some(g), None, bank, dt);
        } else {
            // Gliding: speed on pitch; the speed brakes take off the extra height.
            ctl.throttle = 0.0;
            let pitch = self.pitch_for_speed(ac, spd, dt);
            ctl.spoilers = ((above - 30.0) / 120.0).clamp(0.0, 1.0);
            self.inner(def, ac, ctl, None, Some(pitch), bank, dt);
        }
    }

    /// No hydraulics: the surfaces are frozen. Heading with differential
    /// thrust (more thrust on the left yaws and rolls right), the flight path
    /// with total thrust (under-wing engines pitch the nose up with power).
    fn thrust_only(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, s: &Strip, env: &dyn Environment, dt: f32) {
        let p = vec2(ac.pos.x, ac.pos.z);
        let (thr, dir) = s.end(&s.ident_a).unwrap_or((s.a, (s.b - s.a).normalize_or(vec2(0.0, -1.0))));
        let (along, side) = Strip::local(p, thr, dir);
        let dist = (-along).max(0.0);
        let agl = ac.agl(env);
        if ac.on_ground {
            ctl.engine_throttle = None;
            ctl.throttle = 0.0;
            ctl.reverse = def.has_reverser && ac.airspeed_kt() > 50.0;
            ctl.brakes = 1.0;
            ctl.spoilers = 1.0;
            return;
        }
        // Gear by gravity, flaps where they are (no hydraulics).
        ctl.gear_down = dist < 10.0 * NM;
        // Lateral: track to the centreline (gentle), bank from the track error.
        let gv = vec2(ac.vel.x, ac.vel.z);
        let track = if gv.length() > 20.0 { heading_of(gv) } else { ac.heading() };
        let want = wrap(heading_of(dir) - (side * 0.0012).clamp(-0.35, 0.35));
        let te = wrap(want - track);
        let bank_t = (0.9 * te).clamp(-0.25, 0.25);
        let phi = ac.roll();
        let pr = ac.rates().x;
        let r = ac.rates().z;
        // Differential thrust: left − right (positive rolls / yaws right).
        let diff = (1.2 * (bank_t - phi) - 1.0 * pr - 0.6 * r).clamp(-0.6, 0.6);
        // Vertical: the 3° path (to the threshold), on total thrust; damp the phugoid with pitch rate.
        let path = s.elev + 15.0 + dist * 3f32.to_radians().tan();
        let vs_t = (-vec2(ac.vel.x, ac.vel.z).length() * 3f32.to_radians().tan() - 0.08 * (ac.pos.y - path)).clamp(-9.0, 3.0);
        let e_vs = vs_t - ac.vertical_speed();
        self.dt_i = (self.dt_i + 0.004 * e_vs * dt).clamp(-0.25, 0.35);
        let q = ac.rates().y;
        let mut base = (0.42 + 0.05 * e_vs + self.dt_i - 2.5 * q).clamp(0.05, 1.0);
        if agl < 12.0 {
            base = (base * 0.5).max(0.05);
        }
        throttles::set_sides(def, ctl, base + diff * 0.5, base - diff * 0.5);
    }
}

/// The gear is staying up (normal and alternate failed): land on the belly.
fn run_belly(ac: &Aircraft) -> bool {
    ac.sys.alt_gear_fail
}
