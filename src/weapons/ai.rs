//! Dogfight AI: basic fighter manoeuvres flown through the real flight
//! model (the same `Aircraft::step` as the player, no special physics).
//!
//! Two layers:
//! * **Tactics** (re-planned every reaction time): ground avoidance first,
//!   then missile defence (flares / chaff, beam and break), defensive
//!   breaks and scissors against a gun threat, offensive pursuit (lag →
//!   pure → lead with a gun solution), neutral lead turns, energy
//!   management (unload when slow, yo-yo when fast and closing) and weapon
//!   employment (gun bursts in parameters, AIM-9 / AIM-120 shots).
//! * **Lift-vector control** (every step): roll the lift vector onto the
//!   plane of the desired turn, then pull the load factor the turn needs —
//!   a PI loop on the measured g with an angle-of-attack limiter (works on
//!   the Spitfire's direct controls and on the F-16's g-command FBW alike);
//!   rudder keeps the ball centred.

use super::events::Id;
use super::sensors::View;
use crate::flight::{Aircraft, AircraftDef, ControlLaw, Controls, GRAVITY, KT};
use glam::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Difficulty {
    Rookie,
    Average,
    Veteran,
    Ace,
}

impl Difficulty {
    pub const ALL: [Difficulty; 4] = [Difficulty::Rookie, Difficulty::Average, Difficulty::Veteran, Difficulty::Ace];
    pub fn name(self) -> &'static str {
        match self {
            Difficulty::Rookie => "Rookie",
            Difficulty::Average => "Average",
            Difficulty::Veteran => "Veteran",
            Difficulty::Ace => "Ace",
        }
    }
    pub fn parse(s: &str) -> Option<Difficulty> {
        Difficulty::ALL.into_iter().find(|d| d.name().eq_ignore_ascii_case(s))
    }
    pub fn skill(self) -> Skill {
        match self {
            Difficulty::Rookie => Skill { react: 1.0, g_frac: 0.6, aim: 0.02, burst: 2.0, missiles: false, flares: false, floor: 700.0, energy: false, gun_range: 0.7 },
            Difficulty::Average => Skill { react: 0.6, g_frac: 0.78, aim: 0.01, burst: 1.5, missiles: true, flares: false, floor: 450.0, energy: false, gun_range: 0.85 },
            Difficulty::Veteran => Skill { react: 0.35, g_frac: 0.9, aim: 0.005, burst: 1.0, missiles: true, flares: true, floor: 320.0, energy: true, gun_range: 1.0 },
            Difficulty::Ace => Skill { react: 0.2, g_frac: 1.0, aim: 0.0025, burst: 0.8, missiles: true, flares: true, floor: 220.0, energy: true, gun_range: 1.0 },
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Skill {
    /// Seconds between tactical decisions.
    pub react: f32,
    /// Share of the airframe's g limit used.
    pub g_frac: f32,
    /// Aim error (rad, 1 σ, slowly wandering).
    pub aim: f32,
    /// Longest burst (s).
    pub burst: f32,
    pub missiles: bool,
    pub flares: bool,
    /// Hard deck above the ground (m).
    pub floor: f32,
    /// Vertical / energy tactics.
    pub energy: bool,
    /// Share of the effective gun range it opens fire at.
    pub gun_range: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Recover,
    MissileDefence,
    Defensive,
    Offensive,
    Neutral,
    Extend,
    /// Doomed / pilot out: hands off.
    Dead,
}

impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Mode::Recover => "recover",
            Mode::MissileDefence => "missile defence",
            Mode::Defensive => "defensive",
            Mode::Offensive => "offensive",
            Mode::Neutral => "neutral",
            Mode::Extend => "extend",
            Mode::Dead => "out",
        }
    }
}

/// An incoming missile the pilot knows about.
#[derive(Clone, Copy, Debug)]
pub struct Threat {
    pub pos: Vec3,
    pub vel: Vec3,
    pub radar: bool,
}

/// What the brain needs to know this step.
pub struct Situation<'a> {
    pub me: View,
    pub def: &'a AircraftDef,
    pub ac: &'a Aircraft,
    pub enemies: &'a [View],
    pub threats: &'a [Threat],
    /// Ground height under / 3 s and 8 s ahead.
    pub ground: [f32; 3],
    /// Gun: muzzle speed, effective range (m), time-of-flight / drop function.
    pub muzzle_v: f32,
    pub gun_range: f32,
    pub has_gun: bool,
    pub aim9: bool,
    pub aim120: bool,
    /// The AIM-9 seeker sees / has locked the target; the radar has it in STT.
    pub seeker_lock: Option<Id>,
    pub radar_lock: Option<Id>,
    pub missiles_in_flight: usize,
    pub time: f32,
}

/// What the tactics want this step.
#[derive(Clone, Copy, Debug)]
pub struct Desire {
    /// Direction to point the velocity vector.
    pub dir: Vec3,
    /// Load factor cap (g).
    pub g_max: f32,
    pub throttle: f32,
    pub speedbrake: f32,
}

/// Commands to the weapons.
#[derive(Clone, Copy, Debug, Default)]
pub struct Orders {
    pub guns: bool,
    pub aim9: bool,
    pub aim120: bool,
    pub lock: Option<Id>,
    pub countermeasures: bool,
    pub eject: bool,
}

#[derive(Clone, Debug)]
pub struct Brain {
    pub skill: Skill,
    pub mode: Mode,
    pub target: Option<Id>,
    next_think: f32,
    desire: Desire,
    pub orders: Orders,
    burst_t: f32,
    pause_t: f32,
    g_int: f32,
    /// Load factor per unit of stick (normalised to q = 9.4 kPa on direct controls).
    k_est: f32,
    aim_off: Vec3,
    aim_t: f32,
    last_missile: f32,
    cm_t: f32,
    doomed_t: f32,
    scissor: f32,
    /// Below flying speed: unloading.
    slow: bool,
    /// Height above the ground (m), this step.
    agl: f32,
    rng: u32,
}

impl Brain {
    pub fn new(d: Difficulty, seed: u32) -> Brain {
        Brain {
            skill: d.skill(),
            mode: Mode::Neutral,
            target: None,
            next_think: 0.0,
            desire: Desire { dir: Vec3::Z, g_max: 3.0, throttle: 1.0, speedbrake: 0.0 },
            orders: Orders::default(),
            burst_t: 0.0,
            pause_t: 0.0,
            g_int: 0.0,
            k_est: 0.0,
            aim_off: Vec3::ZERO,
            aim_t: 0.0,
            last_missile: -99.0,
            cm_t: 0.0,
            doomed_t: 0.0,
            scissor: 1.0,
            slow: false,
            agl: 1000.0,
            rng: seed.max(1),
        }
    }
    fn rnd(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng >> 8) as f32 / 16_777_216.0
    }

    /// The airframe's usable load factor and corner speed (TAS, m/s).
    fn limits(&self, def: &AircraftDef, ac: &Aircraft) -> (f32, f32) {
        let sd = crate::damage::struct_def(def.id);
        let n = sd.n_lim.min(if def.control_law == ControlLaw::Direct { 99.0 } else { def.nz_max.max(1.0) }) * self.skill.g_frac;
        let vs = def.vs1_kt * KT;
        let rho = crate::flight::isa(ac.pos.y).density;
        let corner = vs * n.max(1.0).sqrt() * (1.225 / rho.max(0.2)).sqrt();
        (n.max(2.0), corner)
    }

    /// Decide (every reaction time) and fly (every step). `dead`: doomed / no pilot.
    pub fn update(&mut self, s: &Situation, ctl: &mut Controls, doomed: bool, pilot_dead: bool, dt: f32) {
        let ac = s.ac;
        self.orders.guns = false;
        self.orders.aim9 = false;
        self.orders.aim120 = false;
        self.orders.countermeasures = false;
        if pilot_dead {
            self.mode = Mode::Dead;
            ctl.elevator *= 0.98;
            ctl.aileron *= 0.98;
            return;
        }
        if doomed {
            self.mode = Mode::Dead;
            self.doomed_t += dt;
            // A beat to realise, then out (aces quicker).
            if self.doomed_t > 0.8 + 2.5 * self.skill.react {
                self.orders.eject = true;
            }
            ctl.throttle = 0.0;
            return;
        }
        self.aim_t -= dt;
        if self.aim_t <= 0.0 {
            self.aim_t = 0.6 + self.rnd();
            let r = Vec3::new(self.rnd() - 0.5, self.rnd() - 0.5, self.rnd() - 0.5) * 2.0;
            self.aim_off = r * self.skill.aim;
        }
        self.agl = s.ac.pos.y - s.ground[0];
        self.next_think -= dt;
        if self.next_think <= 0.0 {
            self.next_think = self.skill.react * (0.7 + 0.6 * self.rnd());
            self.think(s);
        }
        // Continuous: guns are aimed every step (the pilot tracks), the rest holds.
        self.track(s, dt);
        self.fly(s.def, ac, ctl, dt);
    }

    fn think(&mut self, s: &Situation) {
        let ac = s.ac;
        let me = &s.me;
        let v = ac.vel;
        let speed = v.length().max(1.0);
        let vhat = v / speed;
        let (n_max, corner) = self.limits(s.def, ac);
        let agl = ac.pos.y - s.ground[0];
        // ---- target: the nearest enemy that's alive (sticky while close).
        let pick = s.enemies.iter().filter(|e| e.alive && e.team != me.team).min_by(|a, b| a.pos.distance(me.pos).total_cmp(&b.pos.distance(me.pos)));
        self.target = pick.map(|e| e.id);
        let mut d = Desire { dir: horizontal(vhat), g_max: 2.0, throttle: 0.9, speedbrake: 0.0 };
        // ---- 1. the ground.
        let sink = (-v.y).max(0.0);
        let floor = self.skill.floor + speed * 0.6;
        let t_impact = (agl - s.skill_floor_pad()) / sink.max(1.0);
        let terrain_ahead = s.ground[1].max(s.ground[2]) - ac.pos.y;
        // (Once recovering, keep at it until climbing clear of the floor.)
        let still = self.mode == Mode::Recover && (agl < floor * 1.6 + 150.0 || v.y < 5.0) && agl < floor * 4.0;
        if still || (agl < floor && v.y < speed * 0.1) || t_impact < 7.0 + speed / 60.0 || terrain_ahead > -150.0 {
            self.mode = Mode::Recover;
            let h = horizontal(vhat);
            let up = (h * 0.6 + Vec3::Y * 0.8).normalize();
            d = Desire { dir: up, g_max: n_max, throttle: 1.0, speedbrake: 0.0 };
            self.desire = d;
            return;
        }
        // ---- 2. missiles coming.
        if let Some(m) = s.threats.iter().min_by(|a, b| a.pos.distance(me.pos).total_cmp(&b.pos.distance(me.pos))) {
            let r = m.pos.distance(me.pos);
            if r < 9000.0 {
                self.mode = Mode::MissileDefence;
                let los = (me.pos - m.pos).normalize();
                if self.skill.flares && r < 4500.0 && self.cm_t <= 0.0 {
                    self.orders.countermeasures = true;
                    self.cm_t = 0.6;
                }
                // Beam it (radar: also into the notch, nose low); break hard into it when close.
                let side = los.cross(Vec3::Y).normalize_or(Vec3::X);
                let side = if side.dot(vhat) > 0.0 { side } else { -side };
                let dir = if r < 1800.0 { (-los * 0.3 + side).normalize() } else { (side + Vec3::new(0.0, if m.radar { -0.25 } else { -0.1 }, 0.0)).normalize() };
                d = Desire { dir, g_max: n_max, throttle: if m.radar { 1.0 } else { 0.6 }, speedbrake: 0.0 };
                self.desire = d;
                return;
            }
        }
        let Some(t) = pick else {
            self.mode = Mode::Neutral;
            self.desire = d;
            return;
        };
        let to = t.pos - me.pos;
        let r = to.length().max(1.0);
        let tfwd = t.vel.normalize_or(t.rot * Vec3::Z);
        let ata = vhat.angle_between(to);
        let their_ata = tfwd.angle_between(-to);
        let aa = (-tfwd).angle_between(-to); // 0 = I'm at their six
        let closure = -(t.vel - v).dot(to / r);
        // ---- 3. defensive: their nose on me, close.
        if their_ata < 0.6 && r < 2200.0 && aa > 1.6 {
            self.mode = Mode::Defensive;
            // Break into the attacker (lift vector on him); scissor when he overshoots.
            let into = (t.pos - me.pos).normalize();
            let lift = (into - vhat * into.dot(vhat)).normalize_or(Vec3::Y);
            let mut dir = (vhat + lift * 1.4).normalize();
            if r < 350.0 && closure > 30.0 {
                self.scissor = -self.scissor;
                dir = (vhat + lift.cross(vhat) * self.scissor * 1.2 + Vec3::Y * 0.2).normalize();
            }
            let slow = speed < corner * 0.65;
            d = Desire { dir, g_max: if slow { n_max * 0.6 } else { n_max }, throttle: 1.0, speedbrake: 0.0 };
            if self.skill.flares && r < 1500.0 && s.time > self.cm_t {
                // (guns: no flares; just the break)
            }
            self.desire = d;
            return;
        }
        // ---- 4. offensive / neutral pursuit.
        self.mode = if ata < 1.6 { Mode::Offensive } else { Mode::Neutral };
        let lag = t.pos - t.vel * 1.2;
        let pursuit = if r > 2500.0 || ata > 0.9 {
            // Lag pursuit while closing from far / at high angles off: don't overshoot.
            (lag - me.pos).normalize()
        } else {
            to / r
        };
        let mut throttle = 1.0;
        let mut brake = 0.0;
        // Energy: too slow → unload; fast with height to trade → a high yo-yo.
        if speed < corner * 0.6 && ata > 0.5 {
            self.mode = Mode::Extend;
            d = Desire { dir: (horizontal(vhat) * 0.9 + Vec3::new(0.0, -0.35, 0.0)).normalize(), g_max: 1.5, throttle: 1.0, speedbrake: 0.0 };
            self.desire = d;
            return;
        }
        let mut dir = pursuit;
        // Don't let the fight sink onto the hard deck: climb while low.
        let deck = self.skill.floor * 3.0 + 400.0;
        if agl < deck {
            dir = (dir + Vec3::Y * (0.6 * (1.0 - agl / deck))).normalize();
        }
        if self.skill.energy && closure > 90.0 && r < 1500.0 && ata > 0.35 {
            // High yo-yo: go up out of plane to kill the closure.
            dir = (dir + Vec3::Y * 0.6).normalize();
        }
        if r < 900.0 && closure > 40.0 {
            throttle = 0.45;
            brake = ((closure - 40.0) / 60.0).clamp(0.0, 1.0);
        } else if r < 500.0 && closure > 0.0 {
            throttle = 0.7;
        }
        d = Desire { dir, g_max: n_max, throttle, speedbrake: brake };
        // ---- missiles
        if self.skill.missiles && s.time - self.last_missile > 8.0 && s.missiles_in_flight == 0 {
            if s.aim120 && s.radar_lock == Some(t.id) && (6000.0..32_000.0).contains(&r) && ata < 0.5 {
                self.orders.aim120 = true;
                self.last_missile = s.time;
            } else if s.aim9 && s.seeker_lock == Some(t.id) && (900.0..9000.0).contains(&r) && ata < 0.6 && (aa < 1.9 || r < 5000.0) {
                self.orders.aim9 = true;
                self.last_missile = s.time;
            }
            if s.aim120 && s.radar_lock != Some(t.id) && r < 40_000.0 {
                self.orders.lock = Some(t.id);
            }
        }
        self.desire = d;
    }

    /// Gun tracking: with a target near the nose and in range, steer the
    /// nose onto the lead point and fire bursts in parameters.
    fn track(&mut self, s: &Situation, dt: f32) {
        if self.mode != Mode::Offensive || !s.has_gun {
            self.burst_t = 0.0;
            return;
        }
        let Some(t) = self.target.and_then(|id| s.enemies.iter().find(|e| e.id == id)) else { return };
        let ac = s.ac;
        let to = t.pos - ac.pos;
        let r = to.length();
        if r > s.gun_range * 2.0 {
            return;
        }
        // Lead: where the target will be after the round's time of flight
        // (mean round speed ≈ muzzle + own speed, minus decay), plus drop.
        let vrel = s.muzzle_v * 0.8 + ac.vel.length();
        let tof = r / vrel.max(100.0);
        let lead = t.pos + t.vel * tof - ac.vel * tof + Vec3::Y * (0.5 * GRAVITY * tof * tof);
        let aim = ((lead - ac.pos).normalize() + self.aim_off).normalize();
        self.desire.dir = aim;
        self.desire.g_max = self.limits(s.def, ac).0;
        // In parameters: the nose (gun line) within the target's angular size (plus a little).
        let nose = ac.rot * Vec3::Z;
        let err = nose.angle_between(aim);
        let size = (5.0 / r.max(50.0)).atan();
        let in_range = r < s.gun_range * self.skill.gun_range;
        if self.pause_t > 0.0 {
            self.pause_t -= dt;
        } else if in_range && err < size * 1.6 + 0.004 {
            self.burst_t += dt;
            self.orders.guns = true;
            if self.burst_t > self.skill.burst {
                self.burst_t = 0.0;
                self.pause_t = 0.4 + self.skill.react;
            }
        } else {
            self.burst_t = 0.0;
        }
    }

    /// Lift-vector control of the real airframe.
    fn fly(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, dt: f32) {
        let d = self.desire;
        let v = ac.vel;
        let speed = v.length().max(1.0);
        let vhat = v / speed;
        let fwd = ac.rot * Vec3::Z;
        let up = ac.rot * Vec3::Y;
        let mut dir = d.dir.normalize_or(vhat);
        // Never point steeply at the ground when low (whatever the tactics want).
        let floor = self.skill.floor;
        let min_y = (-(self.agl - floor * 1.5) / 1500.0).clamp(-0.8, 0.15);
        if dir.y < min_y && self.mode != Mode::Dead {
            dir = Vec3::new(dir.x, 0.0, dir.z).normalize_or(Vec3::Z) * (1.0 - min_y * min_y).max(0.0).sqrt() + Vec3::Y * min_y;
        }
        let ang = vhat.angle_between(dir);
        // Turn rate wanted (rad/s) → acceleration toward the desired direction.
        let w = (ang * 1.6).min(1.2);
        let perp = (dir - vhat * dir.dot(vhat)).normalize_or_zero();
        let a = perp * (w * speed) + Vec3::Y * GRAVITY;
        let a_perp = a - vhat * a.dot(vhat);
        let mut n_cmd = a_perp.length() / GRAVITY;
        let lift = a_perp.normalize_or(up);
        // Roll the lift vector onto `lift` (signed angle about the nose).
        let mut phi = up.cross(lift).dot(fwd).atan2(up.dot(lift));
        // Small corrections "down": push rather than roll inverted.
        if phi.abs() > 2.0 && n_cmd < 1.6 && ang < 0.25 {
            phi -= std::f32::consts::PI * phi.signum();
            n_cmd = -n_cmd * 0.5;
        }
        let n_cmd = n_cmd.clamp(-2.0, d.g_max);
        // Pull once roughly rolled in.
        let c = phi.cos().max(0.0);
        let mut n_target = 1.0 + (n_cmd - 1.0) * c * c;
        // Too slow to manoeuvre: stop rolling, unload and let the nose fall
        // until there is flying speed again (no departures at the top of a zoom).
        let vs = def.vs1_kt * KT;
        if ac.airspeed() < vs * 1.3 || (self.slow && ac.airspeed() < vs * 1.6) {
            self.slow = true;
            phi = 0.0;
            n_target = 0.3;
        } else {
            self.slow = false;
        }
        let fbw = def.control_law != ControlLaw::Direct;
        if self.k_est <= 0.0 {
            self.k_est = if fbw { 8.0 } else { 28.0 };
        }
        // Roll: a roll-rate command toward the wanted bank, closed on the measured rate.
        let p = ac.omega.z;
        let p_cmd = (phi * 2.2).clamp(-2.5, 2.5);
        ctl.aileron = if fbw { (p_cmd / 3.7 + (p_cmd - p) * 0.05).clamp(-1.0, 1.0) } else { (p_cmd / 2.2 + (p_cmd - p) * 0.35).clamp(-1.0, 1.0) };
        // Pitch: load factor per unit stick, gain-scheduled on dynamic pressure
        // (direct controls) and learnt in flight; PI on the measured g on top.
        let rho = crate::flight::isa(ac.pos.y).density;
        let q = (0.5 * rho * speed * speed).max(500.0);
        let k = if fbw { self.k_est.max(2.0) } else { (self.k_est * q / 9400.0).max(1.5) };
        let g = ac.g_load();
        if ctl.elevator.abs() > 0.05 && (g - 1.0).abs() > 0.4 && ac.alpha() < def.alpha_peak(0.0) * 0.8 {
            let obs = (g - 1.0) / ctl.elevator;
            if obs > 0.5 {
                let norm = if fbw { obs } else { obs * 9400.0 / q };
                self.k_est += (norm.clamp(2.0, 80.0) - self.k_est) * (dt * 0.4).min(1.0);
            }
        }
        let e = n_target - g;
        self.g_int = (self.g_int + e * dt * 1.2).clamp(-3.0, 3.0);
        let mut elev = (n_target - 1.0 + 0.5 * e + self.g_int) / k;
        let a_lim = def.alpha_peak(ac.surfaces().flaps) * 0.82;
        if ac.alpha() > a_lim {
            elev -= (ac.alpha() - a_lim) * 4.0 / k * 10.0;
            self.g_int = (self.g_int - dt * 3.0).max(-3.0);
        } else if ac.alpha() < -0.5 * a_lim {
            // (Negative stall too.)
            elev += (-0.5 * a_lim - ac.alpha()) * 4.0 / k * 10.0;
            self.g_int = (self.g_int + dt * 3.0).min(3.0);
        }
        ctl.elevator = elev.clamp(-1.0, 1.0);
        ctl.rudder = if fbw { 0.0 } else { (ac.beta() * 2.0 + 0.08 * ctl.aileron).clamp(-1.0, 1.0) };
        // Reheat only when it pays (slow, or defending / recovering): it
        // empties the tanks in ~10 minutes. Military power otherwise.
        let corner = self.limits(def, ac).1;
        let need_ab = speed < corner * 0.95 || matches!(self.mode, Mode::MissileDefence | Mode::Defensive | Mode::Recover | Mode::Extend);
        let thr = if def.has_afterburner { if d.throttle >= 0.99 && need_ab { 1.0 } else { d.throttle.min(1.0) * 0.88 } } else { d.throttle };
        ctl.throttle += (thr - ctl.throttle).clamp(-dt * 1.5, dt * 1.5);
        ctl.spoilers = if def.has_spoilers { d.speedbrake } else { 0.0 };
        ctl.gear_down = false;
        ctl.flaps = 0.0;
        ctl.brakes = 0.0;
    }
}

fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z).normalize_or(Vec3::Z)
}

impl Situation<'_> {
    fn skill_floor_pad(&self) -> f32 {
        60.0
    }
}
