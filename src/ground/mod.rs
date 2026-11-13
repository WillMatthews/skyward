//! Ground handling (Track GH): what happens between the tyres and the
//! pavement, and the ground operations around it.
//!
//! * [`tyre`]: per-wheel slip model (longitudinal slip ratio with the wheel's
//!   spin integrated — touchdown spin-up, locked wheels, anti-skid — slip
//!   angle, friction ellipse), rolling resistance; [`surface`]: dry / wet /
//!   contaminated asphalt and concrete, grass, gravel, dirt, snow and ice from
//!   the weather, hydroplaning above 9·√p kt; [`spec`]: per-type tyres,
//!   brakes, steering.
//! * This module: the undercarriage state the flight model keeps
//!   ([`GroundState`] in `Aircraft::ground`): wheel spin and steering angles,
//!   brake commands (pedals, toe brakes, rudder-split single brake lever,
//!   parking brake, autobrake LO / MED / MAX / RTO), carbon / steel brake
//!   heat with fade and fusible plugs, tyre bursts, tiller / pedal / castering
//!   / lockable tail-wheel steering (nose-wheel steering needs hydraulics), the
//!   tug's pushback force and chocks.
//! * [`pushback`]: the tug (straight back / tail left / tail right), stopping on
//!   "set parking brake"; [`guide`]: taxi routes, follow-the-greens, speed
//!   guidance; [`bot_taxi`]: the auto-flight bot taxiing on the taxiway graph;
//!   [`ops`]: the simulation glue (keys, chocks, ground power, the tug model).

pub mod bot_taxi;
pub mod guide;
pub mod ops;
pub mod pushback;
pub mod spec;
pub mod surface;
#[cfg(test)]
mod tests;
pub mod tyre;

pub use spec::{GroundSpec, Role, Steer};
pub use surface::{Contam, Ground, Pavement, Surface};

use crate::flight::{AircraftDef, ContactKind, Controls, GRAVITY, GearPoint, KT};
use glam::{Quat, Vec2, Vec3};

/// Gear contact points tracked per aircraft (as `Aircraft::gear_comp`).
pub const MAX_WHEELS: usize = 8;

/// Autobrake selector.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Autobrake {
    #[default]
    Off,
    Lo,
    Med,
    Max,
    Rto,
}

impl Autobrake {
    pub fn label(self) -> &'static str {
        match self {
            Autobrake::Off => "OFF",
            Autobrake::Lo => "LO",
            Autobrake::Med => "MED",
            Autobrake::Max => "MAX",
            Autobrake::Rto => "RTO",
        }
    }
}

/// One gear contact point's tyre / wheel / brake state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wheel {
    /// Spin (rad/s, + rolling forward).
    pub omega: f32,
    /// Wheel angle (rad, + = right) — steered, or where it has castered to.
    pub steer: f32,
    /// Free castering now.
    pub castering: bool,
    /// Brake heat sink and wheel rim temperature (°C).
    pub brake_c: f32,
    pub rim_c: f32,
    /// Slip ratio and slip angle (rad) last step.
    pub kappa: f32,
    pub alpha: f32,
    /// Normal load (N).
    pub load: f32,
    /// Peak friction available here, hydroplaning 0..1.
    pub mu: f32,
    pub planing: f32,
    /// Brake command 0..1 and anti-skid release 0..1.
    pub brake_cmd: f32,
    pub release: f32,
    pub locked: bool,
    /// Distance slid on a locked wheel at speed (m): a flat spot, then a burst.
    pub skid_m: f32,
    /// Tyre let down (fusible plugs) or burst.
    pub deflated: bool,
    /// Power scrubbed in the contact patch (W): spin-up smoke, skids.
    pub scrub: f32,
    pub contact: bool,
    pub surface: Surface,
    /// Heat flowing into the brake (W).
    pub brake_power_w: f32,
}

/// Autobrake state.
#[derive(Clone, Copy, Debug, Default)]
pub struct AutobrakeState {
    /// Selector position the state belongs to.
    pub sel: Autobrake,
    /// Braking now (DECEL), and the brake command 0..1.
    pub active: bool,
    pub cmd: f32,
    /// Ramped target deceleration (m/s²).
    pub target: f32,
    /// Seconds the activation conditions have held.
    t: f32,
    /// Take-off thrust seen (arms the RTO) / airborne since (landing).
    rto_ready: bool,
    flown: bool,
    /// The pilot disarmed it (the selector goes back to OFF).
    pub disarmed: bool,
    /// An RTO is braking.
    pub rto: bool,
}

/// A tug on the nose gear (set by the pushback each step).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tow {
    /// Nose-wheel speed along its heading (m/s, − = pushing back).
    pub speed: f32,
    /// Nose-wheel angle the tug holds (rad, + = right).
    pub steer: f32,
    /// Drawbar force limit (N).
    pub max_force: f32,
}

/// The undercarriage state (in `Aircraft::ground`).
#[derive(Clone, Debug, Default)]
pub struct GroundState {
    pub init: bool,
    pub wheels: [Wheel; MAX_WHEELS],
    pub ab: AutobrakeState,
    /// Chocks in (ground crew): the aircraft cannot roll.
    pub chocks: bool,
    pub tow: Option<Tow>,
    /// Longitudinal deceleration (m/s², + = slowing), filtered.
    pub decel: f32,
    prev_v: Option<Vec2>,
    /// Hottest brake (°C) and whether it is over the type's "hot" limit.
    pub max_brake_c: f32,
    pub brakes_hot: bool,
    /// Surface under the aircraft (last contact).
    pub surface: Surface,
    /// Free tail-wheel shimmy 0..1 (vibration).
    pub shimmy: f32,
    /// Advisory raised this step (sim shows it): sequence, text.
    pub note_seq: u32,
    pub note: &'static str,
    /// Burst events this step (bit 0 left main, bit 1 right main).
    pub burst_now: u8,
}

/// Per-step steering / brake commands derived from the controls.
#[derive(Clone, Copy, Debug, Default)]
pub struct Cmd {
    /// Brake command per side: left (+x), right (−x), centre.
    pub brake: [f32; 3],
    /// Static brake torque capacity of one braked leg at full pressure (N·m / m radius): × r.
    pub brake_force_cap: f32,
    pub hold: f32,
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Inputs the per-step updates need from the aircraft.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub vel: Vec3,
    pub rot: Quat,
    /// Body-frame angular velocity.
    pub omega: Vec3,
    pub on_ground: bool,
    /// Ground spoilers deployed 0..1.
    pub spoilers: f32,
    /// Highest thrust lever 0..1 and reverse selected.
    pub lever: f32,
    pub reverse: bool,
    pub oat: f32,
    pub gear_locked: bool,
    /// Nose-wheel steering hydraulics 0..1, wheel-brake effectiveness 0..1 (systems).
    pub nws: f32,
    pub brakes_fx: f32,
}

impl GroundState {
    fn init_once(&mut self, oat: f32) {
        if !self.init {
            self.init = true;
            for w in &mut self.wheels {
                w.brake_c = oat;
                w.rim_c = oat;
                w.mu = 0.8;
            }
        }
    }

    /// Before the contact pass: autobrake, measured deceleration, steering
    /// angles; returns the brake commands for this step.
    pub fn pre_step(&mut self, def: &AircraftDef, c: &Controls, f: &Frame, dt: f32) -> Cmd {
        let sp = spec::spec(def);
        self.init_once(f.oat);
        self.burst_now = 0;
        let gv = Vec2::new(f.vel.x, f.vel.z);
        let gs = gv.length();
        let fwd3 = f.rot * Vec3::Z;
        let fwd = Vec2::new(fwd3.x, fwd3.z).normalize_or(Vec2::Y);
        if let Some(p) = self.prev_v {
            let a = -(gv - p).dot(fwd) / dt.max(1e-4);
            if a.is_finite() {
                self.decel += (a - self.decel) * (dt / 0.25).min(1.0);
            }
        }
        self.prev_v = Some(gv);
        self.autobrake(&sp, c, f, gs, dt);

        // Brake commands: the pedals / single lever (split by the rudder on
        // lever-and-rudder-bar types), toe brakes, the parking brake and the autobrake.
        let pilot = c.brakes;
        let (pl, pr) = if sp.rudder_diff_brake { (pilot * (1.0 - c.rudder).clamp(0.0, 1.0), pilot * (1.0 + c.rudder).clamp(0.0, 1.0)) } else { (pilot, pilot) };
        let fxb = f.brakes_fx;
        let park = if c.parking_brake { 1.0 } else { 0.0 };
        let extra = |toe: f32| (toe.clamp(0.0, 1.0).max(park).max(self.ab.cmd)) * fxb;
        let left = pl.max(extra(c.brake_left));
        let right = pr.max(extra(c.brake_right));
        let centre = pilot.max(extra(c.brake_left.max(c.brake_right)));
        // Brake torque capacity of a leg: the type's braking coefficient on its share of the weight.
        let legs = def.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { braked: true, .. })).count().max(1) as f32;
        let cap = def.brake_mu * def.mass * GRAVITY * 0.95 / legs * sp.brake_scale();

        // Steering.
        let kt = gs / KT;
        let inv = f.rot.conjugate();
        let mut shimmy: f32 = 0.0;
        for (i, gp) in def.gear.iter().enumerate().take(MAX_WHEELS) {
            let role = spec::role(gp);
            let w = &mut self.wheels[i];
            if !matches!(role, Role::Nose | Role::Tail) {
                w.steer = 0.0;
                w.castering = false;
                continue;
            }
            // Where a free wheel would trail to (its contact velocity, body frame).
            let vb = inv * f.vel + f.omega.cross(gp.pos);
            let along = vb.z;
            let free_angle = if Vec2::new(vb.x, vb.z).length() > 0.2 { (-vb.x * along.signum()).atan2(along.abs()) } else { w.steer };
            let trail_rate = (Vec2::new(vb.x, vb.z).length() / 0.35).min(40.0);
            let follow = |cur: f32, to: f32| cur + (to - cur) * (trail_rate * dt).min(1.0);
            let towing = self.tow.filter(|_| role == Role::Nose);
            if let Some(t) = towing {
                w.steer = t.steer;
                w.castering = false;
                continue;
            }
            match sp.steer {
                Steer::Fixed => {
                    w.steer = 0.0;
                    w.castering = false;
                }
                Steer::Tiller { tiller_deg, pedal_deg, tiller_full_kt, tiller_zero_kt, pedal_full_kt, pedal_zero_kt, rate_dps } => {
                    let ft = 1.0 - smooth(tiller_full_kt, tiller_zero_kt, kt);
                    let fp = 1.0 - smooth(pedal_full_kt, pedal_zero_kt, kt);
                    let target = (c.tiller * tiller_deg * ft + c.rudder * pedal_deg * fp).clamp(-tiller_deg, tiller_deg).to_radians();
                    if f.nws < 0.5 {
                        // No hydraulics: the nose wheel casters (steer with differential braking).
                        w.castering = true;
                        w.steer = follow(w.steer, free_angle.clamp(-tiller_deg.to_radians(), tiller_deg.to_radians()));
                    } else {
                        w.castering = false;
                        let r = rate_dps.to_radians() * dt;
                        w.steer += (target - w.steer).clamp(-r, r);
                    }
                }
                Steer::Pedal { deg, fade_ms } => {
                    let target = c.rudder * deg.to_radians() / (1.0 + (gs / fade_ms).powi(2));
                    if f.nws < 0.5 {
                        w.castering = true;
                        w.steer = follow(w.steer, free_angle.clamp(-deg.to_radians(), deg.to_radians()));
                    } else {
                        w.castering = false;
                        w.steer = target;
                    }
                }
                Steer::Caster { link_deg, free_deg, fade_ms } => {
                    let target = c.rudder * link_deg.to_radians() / (1.0 + (gs / fade_ms).powi(2));
                    let link = link_deg.to_radians();
                    // Forced past the link by a differential-braking turn: free castering to the stop.
                    if free_angle.abs() > link && (free_angle - target).abs() > 0.03 && gs > 0.3 {
                        w.castering = true;
                        w.steer = follow(w.steer, free_angle.clamp(-free_deg.to_radians(), free_deg.to_radians()));
                    } else {
                        w.castering = false;
                        w.steer = target;
                    }
                }
                Steer::TailSteer { deg, breakout_deg, fade_ms } => {
                    // A tail wheel steers the opposite way to a nose wheel.
                    let target = -c.rudder * deg.to_radians() / (1.0 + (gs / fade_ms).powi(2));
                    if (free_angle - target).abs() > breakout_deg.to_radians() && gs > 0.3 {
                        w.castering = true;
                        w.steer = follow(w.steer, free_angle);
                    } else {
                        w.castering = false;
                        w.steer = target;
                    }
                }
                Steer::TailCaster { lockable } => {
                    if lockable && c.tail_lock {
                        w.castering = false;
                        w.steer = 0.0;
                    } else {
                        w.castering = true;
                        w.steer = follow(w.steer, free_angle);
                        shimmy = shimmy.max(smooth(8.0, 22.0, gs));
                    }
                }
            }
            if w.castering && shimmy > 0.0 {
                // A free tail wheel shimmies at speed (its damper only just copes).
                w.steer += shimmy * 0.02 * (gs * 7.3).sin();
            }
        }
        self.shimmy = shimmy;
        let hold = left.max(right).max(centre);
        Cmd { brake: [left, right, centre], brake_force_cap: cap, hold }
    }

    fn autobrake(&mut self, sp: &GroundSpec, c: &Controls, f: &Frame, gs: f32, dt: f32) {
        let Some(a) = sp.autobrake else {
            self.ab = AutobrakeState::default();
            return;
        };
        let sel = c.autobrake;
        if sel != self.ab.sel {
            let flown = self.ab.flown;
            self.ab = AutobrakeState { sel, flown, ..Default::default() };
        }
        if !f.on_ground {
            self.ab.flown = true;
            self.ab.rto_ready = false;
            self.ab.active = false;
            self.ab.cmd = 0.0;
            self.ab.t = 0.0;
            return;
        }
        let pilot = c.brakes.max(c.brake_left).max(c.brake_right);
        let ab = &mut self.ab;
        let rto_sel = sel == Autobrake::Rto || (sel == Autobrake::Max && !a.rto_position && !ab.flown);
        if rto_sel {
            if f.lever > 0.6 && !f.reverse {
                ab.rto_ready = true;
            }
            if !ab.active && ab.rto_ready && gs > a.rto_kt * KT && f.lever < 0.1 {
                ab.active = true;
                ab.rto = true;
                self.note_seq += 1;
                self.note = "RTO — maximum braking";
            }
            if ab.active {
                ab.cmd = 1.0;
                ab.target = a.max;
                if pilot > 0.6 && gs < 30.0 * KT || (f.lever > 0.5 && !f.reverse) {
                    *ab = AutobrakeState { sel, disarmed: true, ..Default::default() };
                    self.note_seq += 1;
                    self.note = "Autobrake disarmed";
                }
            }
            return;
        }
        let (level, delay) = match sel {
            Autobrake::Lo => (a.lo, a.delay_lo),
            Autobrake::Med => (a.med, a.delay_med),
            Autobrake::Max => (a.max, 0.0),
            _ => {
                ab.active = false;
                ab.cmd = 0.0;
                return;
            }
        };
        if !ab.active {
            // Landing: armed after flight, activates with the ground spoilers out
            // (or the reversers) and the thrust levers at idle, after a delay.
            let cond = ab.flown && (f.spoilers > 0.6 || f.reverse) && f.lever < 0.12 && gs > 20.0;
            ab.t = if cond { ab.t + dt } else { 0.0 };
            if ab.t >= delay {
                ab.active = true;
                ab.target = 0.0;
                ab.cmd = 0.0;
            }
            return;
        }
        // Disarm: firm pedal braking, or thrust for a go-around / taxi.
        if pilot > 0.55 || (f.lever > 0.35 && !f.reverse) {
            *ab = AutobrakeState { sel, disarmed: true, ..Default::default() };
            self.note_seq += 1;
            self.note = "Autobrake disarmed";
            return;
        }
        // Deceleration control: jerk-limited target, feed-forward + integral.
        ab.target = (ab.target + 2.5 * dt).min(level);
        if sel == Autobrake::Max {
            ab.cmd = 1.0;
            return;
        }
        let e = ab.target - self.decel;
        let ff = ab.target / 5.0;
        ab.cmd = (ab.cmd + 0.30 * e * dt).clamp(0.0, 1.0);
        if ab.cmd < ff * 0.5 {
            ab.cmd = ff * 0.5;
        }
        if gs < 0.5 {
            // Stopped: hold it.
            ab.cmd = ab.cmd.max(0.3);
        }
    }

    /// After the contact pass: brake / rim temperatures (heating, cooling,
    /// fade), fusible plugs and burst tyres.
    pub fn post_step(&mut self, def: &AircraftDef, f: &Frame, new: &[Wheel; MAX_WHEELS], dt: f32) {
        let sp = spec::spec(def);
        let gs = Vec2::new(f.vel.x, f.vel.z).length();
        let cap = sp.leg_heat_capacity().max(1.0);
        let (tau_cool, tau_rim) = match sp.brake {
            spec::BrakeKind::Carbon => (2700.0, 420.0),
            spec::BrakeKind::Steel => (1800.0, 300.0),
            spec::BrakeKind::Light => (420.0, 120.0),
        };
        let airflow = 1.0 + 0.03 * gs + if f.gear_locked { 0.0 } else { -0.4 };
        let mut hottest = f.oat;
        let mut surface = None;
        for (i, gp) in def.gear.iter().enumerate().take(MAX_WHEELS) {
            let old = self.wheels[i];
            let mut w = new[i];
            // Keep what the contact pass does not own.
            w.brake_c = old.brake_c;
            w.rim_c = old.rim_c;
            w.skid_m = old.skid_m;
            w.deflated = old.deflated;
            if !w.contact {
                // Airborne: the wheels spin down (bearings; the mains are braked
                // in the wells on retraction).
                let k = if f.gear_locked { 1.0 / 25.0 } else { 1.0 / 1.5 };
                w.omega = old.omega * (-dt * k).exp();
                w.steer = old.steer;
                w.castering = old.castering;
                w.scrub = 0.0;
                w.locked = false;
                w.load = 0.0;
            } else if surface.is_none() {
                surface = Some(w.surface);
            }
            if matches!(gp.kind, ContactKind::Wheel { braked: true, .. }) {
                // Heat in, Newtonian cooling out (faster with airflow).
                let heat = w.brake_power_w * dt / cap;
                w.brake_c += heat;
                let tau = tau_cool / airflow.max(0.3);
                w.brake_c += (f.oat - w.brake_c) * (dt / tau).min(1.0);
                // The rim soaks a fraction of the brake heat.
                let rim_t = f.oat + 0.38 * (w.brake_c - f.oat);
                w.rim_c += (rim_t - w.rim_c) * (dt / tau_rim).min(1.0);
                if !w.deflated && w.rim_c > sp.fuse_c {
                    w.deflated = true;
                    self.burst_now |= side_bit(gp);
                    self.note_seq += 1;
                    self.note = "Fusible plugs melted — tyre deflated";
                }
                // A locked wheel sliding at speed grinds a flat spot through the tyre.
                if w.locked && gs > 12.0 {
                    w.skid_m += gs * dt;
                } else if !w.locked {
                    w.skid_m = (w.skid_m - dt * 2.0).max(0.0);
                }
                if !w.deflated && w.skid_m > 70.0 {
                    w.deflated = true;
                    self.burst_now |= side_bit(gp);
                    self.note_seq += 1;
                    self.note = "Tyre burst — locked wheel";
                }
                hottest = hottest.max(w.brake_c);
            }
            self.wheels[i] = w;
        }
        if let Some(s) = surface {
            self.surface = s;
        }
        self.max_brake_c = hottest;
        self.brakes_hot = hottest > sp.hot_c();
    }

    /// Hottest brake per main leg, left to right (for the WHEEL page).
    pub fn brake_temps(&self, def: &AircraftDef) -> Vec<(f32, f32)> {
        let mut v: Vec<(f32, f32)> = def
            .gear
            .iter()
            .enumerate()
            .take(MAX_WHEELS)
            .filter(|(_, g)| matches!(g.kind, ContactKind::Wheel { braked: true, .. }))
            .map(|(i, g)| (g.pos.x, self.wheels[i].brake_c))
            .collect();
        v.sort_by(|a, b| b.0.total_cmp(&a.0));
        v
    }
}

fn side_bit(gp: &GearPoint) -> u8 {
    if gp.pos.x > 0.1 {
        1
    } else if gp.pos.x < -0.1 {
        2
    } else {
        0
    }
}

/// Everything about one wheel's contact this step.
#[allow(clippy::too_many_arguments)]
pub fn contact(def: &AircraftDef, gs: &GroundState, cmd: &Cmd, i: usize, gp: &GearPoint, rot: Quat, v_cp: Vec3, fz: f32, surface: Surface, dt: f32) -> (Vec3, Wheel) {
    let sp = spec::spec(def);
    let role = spec::role(gp);
    let mut w = gs.wheels.get(i).copied().unwrap_or_default();
    let (steerable, braked) = match gp.kind {
        ContactKind::Wheel { steerable, braked } => (steerable, braked),
        ContactKind::Skid { .. } => (false, false),
    };
    let steer = if steerable { w.steer } else { 0.0 };
    let fwd_b = Vec3::new(-steer.sin(), 0.0, steer.cos());
    let mut fwd = rot * fwd_b;
    fwd.y = 0.0;
    let fwd = fwd.try_normalize().unwrap_or(Vec3::Z);
    let side = Vec3::Y.cross(fwd);
    let main = role == Role::Main;
    let (psi, r, inertia) = if main { (sp.psi_main, sp.r_main, sp.i_main * sp.wheels_per_leg as f32) } else { (sp.psi_aux, sp.r_aux, sp.i_aux) };
    let side_cmd = if gp.pos.x > 0.1 {
        cmd.brake[0]
    } else if gp.pos.x < -0.1 {
        cmd.brake[1]
    } else {
        cmd.brake[2]
    };
    let b = if braked { side_cmd.clamp(0.0, 1.0) } else { 0.0 };
    let fade = sp.fade(w.brake_c);
    let brake_torque = b * cmd.brake_force_cap * r * fade;
    // Slush / standing water: displacement drag on each tyre (falls once planing).
    let depth = surface.contam.fluid_mm() * 0.001;
    let vx = v_cp.dot(fwd);
    let fluid_drag = if depth > 0.0 {
        let n = if main { sp.wheels_per_leg as f32 } else { 1.0 };
        let (_, _, hp) = surface.friction(vx.abs(), psi, true);
        0.5 * surface.contam.fluid_density() * vx * vx * 0.75 * (0.75 * r) * depth * n * (1.0 - 0.8 * hp)
    } else {
        0.0
    };
    let o = tyre::step(&tyre::In {
        fz,
        v: v_cp,
        fwd,
        side,
        surface,
        psi,
        r,
        inertia,
        omega: w.omega,
        roll_mu: def.roll_mu,
        brake_torque,
        antiskid: sp.antiskid,
        lat_scale: (def.tire_mu / 0.8).clamp(0.3, 1.5) * if role == Role::Tail { sp.tail_grip() } else { 1.0 },
        castering: w.castering && steerable,
        fluid_drag,
        dt,
    });
    let mut force = o.force;
    // The tug on the nose gear: drives the nose wheel's speed along its heading.
    if let (Some(t), Role::Nose) = (gs.tow, role) {
        let k = 0.25 * def.mass / dt;
        let f = (-(vx - t.speed) * k).clamp(-t.max_force, t.max_force);
        force += fwd * f;
    }
    w.omega = o.omega;
    w.kappa = o.kappa;
    w.alpha = o.alpha;
    w.load = fz;
    w.mu = o.mu;
    w.planing = o.planing;
    w.brake_cmd = b;
    w.release = o.release;
    w.locked = o.locked;
    w.scrub = o.scrub_power;
    w.brake_power_w = o.brake_power;
    w.contact = true;
    w.surface = surface;
    (force, w)
}
