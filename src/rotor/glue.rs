//! Rotorcraft step inside `flight` (a child module of `flight.rs`, so it
//! reaches the airframe's private state): air data → the rotor model
//! (`crate::rotor`) for forces, moments and the drive train → the same
//! ground contact, touchdown / crash checks and rigid-body integration as the
//! fixed-wing aircraft, with rotorcraft wording (skids, dynamic rollover,
//! blade strikes) and limits (rotor overspeed).

use super::*;
use crate::rotor::RotorDef;

impl Aircraft {
    /// Engine gauges for rotorcraft (rotor / motor rpm, Ng or motor fraction,
    /// TOT, fuel flow, shaft power, running, thrust, shaft angle).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn set_engine_gauges(&mut self, i: usize, rpm: f32, spool: f32, egt: f32, fuel_flow: f32, power: f32, running: bool, thrust: f32, angle: f32) {
        if let Some(e) = self.eng.get_mut(i) {
            e.rpm = rpm;
            e.spool = spool;
            e.egt = egt;
            e.fuel_flow = fuel_flow;
            e.shaft_power = power;
            e.running = running;
            e.thrust = thrust;
            e.prop_angle = angle;
        }
        if i < 4 {
            self.surf.prop_angles[i] = angle;
        }
        if i == 0 {
            self.surf.prop_angle = angle;
        }
    }

    /// In the air at `speed_ms` (indicated), trimmed by the rotor model.
    pub(super) fn airborne_rotor(rd: &RotorDef, def: &AircraftDef, pos: Vec3, heading_rad: f32, speed_ms: f32) -> Self {
        let mut a = Self::blank(def);
        let alt = finite_or(pos.y, 0.0);
        let tas = tas_from_cas(finite_or(speed_ms, 0.0).max(0.0), alt);
        let atm = isa(alt);
        a.pos = pos;
        a.rot = attitude(heading_rad, 0.0, 0.0);
        a.vel = attitude(heading_rad, 0.0, 0.0) * Vec3::Z * tas;
        a.air_vel = a.vel;
        a.ias = speed_ms;
        a.mach = tas / atm.speed_of_sound;
        a.oat = atm.temperature_c();
        a.gear_pos = 1.0;
        let mut ctl = Controls::default();
        crate::rotor::settle(rd, def, &mut a, &mut ctl, true);
        a
    }

    /// One fixed step of a rotorcraft (see the module docs).
    pub(super) fn step_rotor(&mut self, rd: &RotorDef, def: &AircraftDef, ctl: &Controls, env: &dyn Environment, dt: f32) {
        let dt = dt.min(1.0 / 30.0);
        let mut c = ctl.sanitized();
        c.failed_engines |= self.dmg.failed_engine_bits();
        let fx = self.sys.fx;
        c.failed_engines |= fx.engine_cut;
        let saved = self.clone();
        let quad = rd.quad().is_some();
        // Rotorcraft failures from the systems (src/systems): tail rotor, servo hydraulics.
        if !quad {
            use crate::systems::failures::Kind;
            self.rotor.fail.tail_drive = self.sys.has(Kind::TailRotor, 0);
            self.rotor.fail.tail_pitch = self.sys.has(Kind::TailRotor, 1);
            self.rotor.fail.hydraulics = fx.surface_rate[0] == 0.0;
        }

        // ---- air data (as the fixed-wing step) ---------------------------------
        let updraft = finite_or(env.updraft(self.pos), 0.0);
        let wind = env.wind(self.pos);
        let wind = if wind.is_finite() { wind.clamp_length_max(150.0) } else { Vec3::ZERO };
        let tau = (def.wingspan * 0.8 / self.vel.length().max(8.0)).clamp(0.02, 0.6);
        let wind = match self.wind_f {
            Some(w) => w + (wind - w) * (dt / tau).min(1.0),
            None => wind,
        };
        self.wind_f = Some(wind);
        let air_world = self.vel - wind - Vec3::Y * updraft;
        let inv = self.rot.conjugate();
        let vb = inv * air_world;
        let ground_h = finite_or(env.ground_height(Vec2::new(self.pos.x, self.pos.z)), 0.0);
        let agl = self.pos.y - ground_h;
        let atm = isa(self.pos.y);
        let tas = vb.length();
        self.mach = tas / atm.speed_of_sound;
        // A helicopter's pitot reads nothing useful below ~20 kt (rotor wash).
        let ias = cas_from_qc(impact_pressure(atm.pressure, self.mach));
        self.ias = if quad { ias } else { ias * ((ias - 6.0) / 6.0).clamp(0.0, 1.0) };
        self.oat = atm.temperature_c();
        let vs2 = vb.y * vb.y + vb.z * vb.z;
        self.alpha = if vs2 > 1e-4 { (-vb.y).atan2(vb.z) } else { 0.0 };
        self.beta = if tas > 1e-2 { (-vb.x / tas).clamp(-1.0, 1.0).asin() } else { 0.0 };
        self.air_vel = air_world;
        self.gear_pos = 1.0;
        self.surf.gear = 0.0;

        // ---- rotor model ---------------------------------------------------------
        let ro = crate::rotor::step(rd, def, self, &c, vb, agl, &atm, dt);
        let eff = self.rotor.out.eff;
        // Cockpit controls follow the effective inputs (collective on `flaps`-free slots).
        self.surf.elevator = eff[1];
        self.surf.aileron = eff[2];
        self.surf.rudder = eff[3];
        let f_body = ro.force;
        let mut m_body = ro.moment;
        if fx.cg != Vec3::ZERO {
            let w_body = inv * Vec3::new(0.0, -(self.dmg.mass(def) + fx.mass_delta) * GRAVITY, 0.0);
            m_body += fx.cg.cross(w_body);
        }

        // ---- ground ---------------------------------------------------------------
        let gnd = self.ground(def, env, &c, dt);
        if let Some(reason) = gnd.crash {
            let reason = self.rotor_crash_reason(def, quad, &reason);
            self.crash(reason);
            return;
        }
        m_body += gnd.moment_body;
        self.gear_comp = gnd.comp;
        let f_ground = gnd.force_world;
        if gnd.contact && !self.on_ground {
            let sink = -self.vel.y;
            let bank = self.roll();
            if sink > 3.0 * def.max_touchdown_sink {
                self.crash(format!("Flew into the ground at {:.0} fpm", sink / FPM));
                return;
            }
            if sink > def.max_touchdown_sink {
                let sd = crate::damage::struct_def(def.id);
                if quad {
                    self.crash(format!("Crashed: hit the ground at {:.1} m/s — arms and props broken", sink));
                } else {
                    for p in [crate::damage::Part::GearL, crate::damage::Part::GearR] {
                        if def.gear.iter().any(|g| crate::damage::gear_part(def, g) == Some(p)) {
                            self.dmg.separate(sd, def, p, "hard landing");
                        }
                    }
                    self.crash(format!("Hard landing: touched down at {:.0} fpm — skids collapsed", sink / FPM));
                }
                return;
            }
            if bank.abs() > def.max_touchdown_bank && !quad {
                self.crash(format!("Dynamic rollover: touched down with {:.0}° of bank — the main rotor struck the ground", bank.abs().to_degrees()));
                return;
            }
        }
        // Rolling over about a skid: past ~30° the blades hit the ground.
        if gnd.contact && !quad && self.roll().abs() > 0.52 {
            self.crash("Dynamic rollover: rolled over on a skid — the main rotor struck the ground".to_string());
            return;
        }
        if gnd.contact && quad && self.up().y < -0.2 && Vec2::new(self.vel.x, self.vel.z).length() < 1.0 {
            self.crash("Turtled: the quad flipped over on the ground".to_string());
            return;
        }
        self.on_ground = gnd.contact;

        // ---- load factor ------------------------------------------------------------
        let mass = self.dmg.mass(def) + fx.mass_delta;
        let weight = mass * GRAVITY;
        let f_world = self.rot * f_body + f_ground;
        let up = self.up();
        self.g_load = f_world.dot(up) / weight;
        self.side_g = -f_world.dot(self.right()) / weight;
        self.g_filtered += (f_body.y / weight - self.g_filtered) * (1.0 - (-dt / 0.1f32).exp());

        // ---- integrate translation and rotation ---------------------------------------
        let acc = f_world / mass - Vec3::Y * GRAVITY;
        self.vel += acc * dt;
        self.pos += self.vel * dt;
        let inertia = self.dmg.inertia(def);
        let w = self.omega;
        let gyro = w.cross(inertia * w + ro.h);
        let damping = ro.damping.max(Vec3::ZERO);
        let mut w_new = (w + dt * (m_body - gyro) / inertia) / (Vec3::ONE + dt * damping / inertia);
        let max_rate = if quad { 30.0 } else { 6.0 };
        let wl = w_new.length();
        if wl > max_rate {
            w_new *= max_rate / wl;
        }
        self.omega = w_new;
        self.rot = (self.rot * Quat::from_scaled_axis(self.omega * dt)).normalize();

        // ---- structure / rotor limits ----------------------------------------------------
        if !quad && self.rotor.out.nr_pct > 125.0 {
            self.crash("Rotor overspeed: the main rotor came apart".to_string());
            return;
        }
        // Structural limit on the airframe's own load (not ground contact).
        let n = f_body.y / weight;
        if !self.on_ground && (n > def.g_limit_pos * 1.5 || n < def.g_limit_neg * 1.5) {
            self.dmg.failure = Some(format!("Structural failure at {n:.1} g"));
            self.crash(format!("Structural failure: {n:.1} g"));
            return;
        }

        // ---- obstacles and terrain ------------------------------------------------------------
        if let Some(reason) = self.obstacle_check(def, env) {
            let reason = if quad {
                "Crashed into a building".to_string()
            } else if reason.contains("Wingtip") || reason == "Flew into a building" && self.rotor_tip_hits(def, env) {
                "Main rotor struck a building".to_string()
            } else {
                reason
            };
            self.crash(reason);
            return;
        }
        let ground_now = finite_or(env.ground_height(Vec2::new(self.pos.x, self.pos.z)), 0.0);
        if self.pos.y < ground_now + (0.15f32).min(def.gear_height * 0.5) {
            let r = if env.is_water(Vec2::new(self.pos.x, self.pos.z)) { "Ditched in the water" } else { "Flew into the ground" };
            self.crash(r.to_string());
            return;
        }
        let finite =
            self.pos.is_finite() && self.vel.is_finite() && self.omega.is_finite() && self.rot.is_finite() && self.rotor.heli.nr.is_finite() && self.rotor.heli.vi.is_finite();
        if !finite {
            *self = saved;
            self.crash("Simulation fault (state became non-finite)".to_string());
        }
    }

    fn rotor_tip_hits(&self, def: &AircraftDef, env: &dyn Environment) -> bool {
        def.hard_points.iter().filter(|h| h.reason.contains("rotor")).any(|h| {
            let p = self.pos + self.rot * h.pos;
            env.obstacle_top(Vec2::new(p.x, p.z)).is_some_and(|t| p.y < t)
        })
    }

    fn rotor_crash_reason(&self, def: &AircraftDef, quad: bool, reason: &str) -> String {
        let _ = def;
        if quad || !reason.contains("rotor") {
            return reason.to_string();
        }
        if self.on_ground && self.roll().abs() > 0.25 { "Dynamic rollover: the main rotor struck the ground".to_string() } else { reason.to_string() }
    }
}
