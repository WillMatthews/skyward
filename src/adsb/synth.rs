//! What the feed does not say: the flight phase, the attitude, gear, flaps,
//! spoilers, reversers, engines and lights — synthesised from the smoothed
//! motion, so a stand-in model looks like the real aircraft doing what it
//! is doing.
//!
//! * **Phase** from on-ground, speed, acceleration, vertical speed and
//!   height above the ground ([`phase`]).
//! * **Attitude**: heading = reported true heading, else the air vector
//!   (ground track minus the wind: real crab angles); pitch = flight-path
//!   angle + an angle of attack that grows as the speed falls towards Vref;
//!   bank = reported roll, else the coordinated-turn bank of the track rate
//!   (`tan φ = V ω / g`); on the ground the gear geometry's rest pitch, a
//!   rotation before lift-off (known in advance: the playout runs in the
//!   past) and the nose lowering after touch-down. Helicopters tilt nose
//!   down with speed.
//! * **Configuration** by phase: gear (down below ~2,000 ft on approach, up
//!   seconds after lift-off), flaps from the speed on approach, take-off
//!   flap, spoilers and reversers on the landing roll.
//! * **Lights** as airlines fly them: navigation always, beacon with the
//!   engines running, strobes on the runway and airborne, landing lights
//!   below 10,000 ft and on the runway, taxi light while taxiing.

use crate::flight::{AircraftDef, ContactKind, EngineKind};
use glam::{Quat, Vec3};
use skyward_net::proto::{AcState, EngineState, MAX_ENGINES, RotorState, flags, lights};

const KT: f32 = 0.514_444;
const FT: f32 = 0.3048;
const G: f32 = 9.81;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Stopped for a while (engines off at the stand).
    Parked,
    Taxi,
    TakeoffRoll,
    /// After lift-off, below ~3,000 ft.
    Departure,
    Climb,
    Cruise,
    Descent,
    Approach,
    LandingRoll,
    /// Rotorcraft, airborne below ~15 kt.
    Hover,
}

impl Phase {
    pub fn name(self) -> &'static str {
        match self {
            Phase::Parked => "parked",
            Phase::Taxi => "taxi",
            Phase::TakeoffRoll => "take-off",
            Phase::Departure => "departure",
            Phase::Climb => "climb",
            Phase::Cruise => "cruise",
            Phase::Descent => "descent",
            Phase::Approach => "approach",
            Phase::LandingRoll => "landing",
            Phase::Hover => "hover",
        }
    }
    pub fn airborne(self) -> bool {
        !matches!(self, Phase::Parked | Phase::Taxi | Phase::TakeoffRoll | Phase::LandingRoll)
    }
}

/// What phase inference sees.
#[derive(Clone, Copy, Debug, Default)]
pub struct Motion {
    pub on_ground: bool,
    pub gs_kt: f32,
    /// Along-track acceleration, kt/s.
    pub accel_kt_s: f32,
    pub vs_fpm: f32,
    /// Height of the wheels above the ground, ft.
    pub agl_ft: f32,
    /// Stopped for this long (on the ground), s.
    pub still_s: f32,
    /// Since the last time on the ground (None: not seen on the ground), s.
    pub since_ground_s: Option<f32>,
    pub rotorcraft: bool,
}

/// The flight phase.
pub fn phase(m: &Motion) -> Phase {
    if m.on_ground {
        return if m.gs_kt < 1.0 {
            if m.still_s > 90.0 { Phase::Parked } else { Phase::Taxi }
        } else if m.gs_kt < 40.0 || m.rotorcraft {
            Phase::Taxi
        } else if m.accel_kt_s > -0.3 {
            Phase::TakeoffRoll
        } else {
            Phase::LandingRoll
        };
    }
    if m.rotorcraft && m.gs_kt < 15.0 {
        return Phase::Hover;
    }
    if m.agl_ft < 3000.0 {
        if m.vs_fpm > 300.0 {
            return Phase::Departure;
        }
        if m.vs_fpm < -300.0 {
            return Phase::Approach;
        }
        // Level and low: just departed (a level-off) or in the pattern / on approach.
        return if m.since_ground_s.is_some_and(|s| s < 150.0) { Phase::Departure } else { Phase::Approach };
    }
    if m.vs_fpm > 500.0 {
        Phase::Climb
    } else if m.vs_fpm < -500.0 {
        Phase::Descent
    } else {
        Phase::Cruise
    }
}

/// Body-frame contact points of a type (scaled) — the wheels / skids.
pub fn contacts(def: &AircraftDef, scale: f32) -> Vec<Vec3> {
    def.gear.iter().map(|g| g.pos * scale).collect()
}

/// Height of the body origin above flat ground at attitude `rot` (the lowest contact touching).
pub fn rest_height(contacts: &[Vec3], rot: Quat) -> f32 {
    contacts.iter().map(|p| -(rot * *p).y).fold(f32::MIN, f32::max).max(0.0)
}

/// Pitch (rad) at which a type sits on its wheels: main gear and the
/// nose / tail wheel both touching.
pub fn rest_pitch(def: &AircraftDef) -> f32 {
    let wheels: Vec<Vec3> = def.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { .. })).map(|g| g.pos).collect();
    if wheels.len() < 2 {
        return 0.0;
    }
    // Mains: the pair with the largest |x|; the other wheel on the centreline.
    let main = wheels.iter().copied().max_by(|a, b| a.x.abs().total_cmp(&b.x.abs())).unwrap();
    let other = wheels.iter().copied().filter(|p| p.x.abs() < 0.3).max_by(|a, b| (a.z - main.z).abs().total_cmp(&(b.z - main.z).abs()));
    match other {
        Some(o) if (main.z - o.z).abs() > 0.5 => ((o.y - main.y) / (main.z - o.z)).atan().clamp(-0.05, 0.25),
        _ => 0.0,
    }
}

/// Smoothed per-aircraft animation state.
#[derive(Clone, Debug, Default)]
pub struct Anim {
    pub phase: Option<Phase>,
    pub heading: f32,
    pub pitch: f32,
    pub bank: f32,
    /// Filtered track rate, rad/s.
    pub turn: f32,
    last_track: Option<f32>,
    last_gs: Option<f32>,
    pub accel: f32,
    pub gear: f32,
    pub flaps: f32,
    pub spoilers: f32,
    pub reverser: f32,
    pub spool: f32,
    pub still_s: f32,
    /// Seconds since lift-off / touch-down (while < 1e6).
    pub since_liftoff: f32,
    pub since_touchdown: f32,
    pub since_ground: Option<f32>,
    was_ground: Option<bool>,
    pub init: bool,
}

/// One frame's input to [`Anim::step`].
pub struct Frame<'a> {
    pub def: &'a AircraftDef,
    pub contacts: &'a [Vec3],
    pub rest_pitch: f32,
    pub dt: f32,
    /// Velocity (world: east, up, south), m/s.
    pub vel: Vec3,
    /// Wind at the aircraft (world, m/s).
    pub wind: Vec3,
    pub on_ground: bool,
    /// Wheels above the ground, m.
    pub agl: f32,
    pub alt_m: f32,
    pub heading: Option<f32>,
    pub roll: Option<f32>,
    /// Seconds until lift-off (when on the ground and about to rotate).
    pub liftoff_in: Option<f32>,
    /// Snapped taxiway / stand heading (deg) to follow on the ground.
    pub ground_heading: Option<f32>,
    /// Stopped on a stand (parked: engines off, beacon off).
    pub at_stand: bool,
}

fn approach(x: f32, target: f32, rate: f32, dt: f32) -> f32 {
    x + (target - x).clamp(-rate * dt, rate * dt)
}

fn lowpass(x: f32, target: f32, tau: f32, dt: f32) -> f32 {
    x + (target - x) * (1.0 - (-dt / tau.max(1e-3)).exp())
}

fn wrap_deg(d: f32) -> f32 {
    (d + 540.0).rem_euclid(360.0) - 180.0
}

impl Anim {
    /// Advance by one frame; returns the attitude (body → world).
    pub fn step(&mut self, f: &Frame) -> Quat {
        let def = f.def;
        let dt = f.dt.clamp(0.0, 0.5);
        let gh = glam::vec2(f.vel.x, f.vel.z);
        let gs = gh.length();
        let rotor = def.rotor.is_some();
        // ---- track rate and acceleration (filtered)
        let track = gh.x.atan2(-gh.y).to_degrees();
        if let (Some(lt), true) = (self.last_track, gs > 8.0 && dt > 0.0) {
            let w = wrap_deg(track - lt).to_radians() / dt;
            self.turn = lowpass(self.turn, w.clamp(-0.3, 0.3), 1.2, dt);
        } else {
            self.turn = lowpass(self.turn, 0.0, 1.2, dt);
        }
        self.last_track = (gs > 8.0).then_some(track);
        if let (Some(lg), true) = (self.last_gs, dt > 0.0) {
            self.accel = lowpass(self.accel, ((gs - lg) / dt / KT).clamp(-15.0, 15.0), 2.0, dt);
        }
        self.last_gs = Some(gs);
        // ---- ground / air transitions
        let ground = f.on_ground || f.agl < 0.3;
        match self.was_ground {
            Some(true) if !ground => self.since_liftoff = 0.0,
            Some(false) if ground => self.since_touchdown = 0.0,
            None => {
                self.since_liftoff = 1e6;
                self.since_touchdown = 1e6;
            }
            _ => {}
        }
        self.was_ground = Some(ground);
        self.since_liftoff = (self.since_liftoff + dt).min(1e6);
        self.since_touchdown = (self.since_touchdown + dt).min(1e6);
        if ground {
            self.since_ground = Some(0.0);
        } else if let Some(s) = self.since_ground.as_mut() {
            *s += dt;
        }
        self.still_s = if ground && gs < 0.6 { self.still_s + dt } else { 0.0 };
        if f.at_stand {
            self.still_s = self.still_s.max(1000.0);
        }
        let m = Motion {
            on_ground: ground,
            gs_kt: gs / KT,
            accel_kt_s: self.accel,
            vs_fpm: f.vel.y / FT * 60.0,
            agl_ft: f.agl / FT,
            still_s: self.still_s,
            since_ground_s: self.since_ground,
            rotorcraft: rotor,
        };
        let ph = phase(&m);
        self.phase = Some(ph);

        // ---- attitude
        let air = f.vel - f.wind;
        let tas = air.length();
        let air_h = glam::vec2(air.x, air.z);
        let heading_target = if let Some(h) = f.ground_heading.filter(|_| ground) {
            h
        } else if let Some(h) = f.heading {
            h
        } else if !ground && air_h.length() > 15.0 && !rotor {
            air_h.x.atan2(-air_h.y).to_degrees()
        } else if gs > 1.0 {
            track
        } else {
            self.heading
        };
        let vref = (def.vref_kt * KT).max(20.0);
        let (pitch_t, bank_t) = if rotor {
            // Nose down with speed, into the turns.
            let p = -(tas / 70.0).clamp(0.0, 1.0) * 7f32.to_radians() - self.accel.clamp(-3.0, 3.0) * 0.6f32.to_radians();
            let b = (gs * self.turn / G).atan().clamp(-0.5, 0.5);
            (if ground { f.rest_pitch } else { p }, if ground { 0.0 } else { f.roll.map(|r| r.to_radians()).unwrap_or(b) })
        } else if ground {
            // Rotation just before lift-off (the playout knows when).
            let rot_target = match def.engine_kind {
                EngineKind::Jet => 9f32.to_radians(),
                _ => 7f32.to_radians(),
            };
            let p = match f.liftoff_in {
                Some(s) if s < 3.5 && gs > 25.0 => f.rest_pitch + (rot_target - f.rest_pitch) * (1.0 - s / 3.5).clamp(0.0, 1.0),
                _ => f.rest_pitch,
            };
            (p, 0.0)
        } else {
            let fpa = f.vel.y.atan2(gs.max(1.0));
            let aoa = (5.8 * (vref / tas.max(vref * 0.8)).powi(2)).clamp(2.2, 10.0).to_radians();
            let bank = f.roll.map(|r| r.to_radians()).unwrap_or_else(|| (gs * self.turn / G).atan());
            ((fpa + aoa).clamp(-0.25, 0.35), bank.clamp(-0.6, 0.6))
        };
        let first = !self.init;
        if first {
            self.init = true;
            self.heading = heading_target.rem_euclid(360.0);
            self.pitch = pitch_t;
            self.bank = bank_t;
        }
        // Heading follows quickly in the air (it is mostly the track), slower on the ground.
        let tau_h = if ground { 0.5 } else { 0.8 };
        self.heading = (self.heading + wrap_deg(heading_target - self.heading) * (1.0 - (-dt / tau_h).exp())).rem_euclid(360.0);
        self.pitch = lowpass(self.pitch, pitch_t, if ground && self.since_touchdown < 6.0 { 1.4 } else { 0.7 }, dt);
        self.bank = lowpass(self.bank, bank_t, 0.6, dt);

        // ---- configuration
        let ias = tas;
        let gear_t = if !def.retractable_gear || ground {
            1.0
        } else {
            match ph {
                Phase::Approach if f.agl < 2200.0 * FT => 1.0,
                Phase::Hover => 1.0,
                _ if self.since_liftoff < 7.0 => 1.0,
                _ => 0.0,
            }
        };
        let gear_rate = 1.0 / def.gear_time.max(4.0);
        self.gear = if first { gear_t } else { approach(self.gear, gear_t, gear_rate, dt) };
        let has_flaps = !def.flap_detents.is_empty() && def.flap_label != "Airbrakes";
        let flaps_t = if !has_flaps || rotor {
            0.0
        } else {
            match ph {
                Phase::TakeoffRoll | Phase::Departure if ias < vref * 1.6 => 0.3,
                Phase::Approach => ((1.6 * vref - ias) / (0.55 * vref)).clamp(0.0, 1.0),
                Phase::LandingRoll => 1.0,
                _ => 0.0,
            }
        };
        self.flaps = if first { flaps_t } else { approach(self.flaps, flaps_t, 0.12, dt) };
        let spoil_t = if def.has_spoilers && ph == Phase::LandingRoll && gs > 25.0 * KT { 1.0 } else { 0.0 };
        self.spoilers = approach(self.spoilers, spoil_t, 2.0, dt);
        let rev_t = if def.has_reverser && ph == Phase::LandingRoll && gs > 70.0 * KT && self.since_touchdown < 25.0 { 1.0 } else { 0.0 };
        self.reverser = approach(self.reverser, rev_t, 0.8, dt);
        let spool_t = match ph {
            Phase::Parked => 0.0,
            Phase::Taxi => 0.24,
            Phase::TakeoffRoll => 0.95,
            Phase::Departure | Phase::Climb => 0.88,
            Phase::Cruise => 0.8,
            Phase::Descent => 0.32,
            Phase::Approach => 0.55,
            Phase::LandingRoll => {
                if self.reverser > 0.5 {
                    0.75
                } else {
                    0.25
                }
            }
            Phase::Hover => 0.9,
        };
        self.spool = if first { spool_t } else { approach(self.spool, spool_t, 0.15, dt) };
        attitude_quat(self.heading, self.pitch, self.bank)
    }
}

/// Body → world for heading (deg), pitch and bank (rad).
pub fn attitude_quat(heading_deg: f32, pitch: f32, bank: f32) -> Quat {
    crate::flight::attitude(heading_deg.to_radians(), pitch, bank)
}

/// The light switches for a phase (`lights::*`).
pub fn light_bits(ph: Phase, alt_m: f32, gs: f32, running: bool, drone: bool) -> u8 {
    if drone {
        return 0;
    }
    let mut l = lights::NAV;
    if running {
        l |= lights::BEACON;
    }
    match ph {
        Phase::TakeoffRoll | Phase::LandingRoll => l |= lights::STROBE | lights::LANDING,
        Phase::Taxi if gs > 1.5 => l |= lights::TAXI,
        p if p.airborne() => {
            l |= lights::STROBE;
            if alt_m < 10_000.0 * FT {
                l |= lights::LANDING;
            }
        }
        _ => {}
    }
    l
}

/// The network-style state the remote-aircraft renderer animates from.
pub fn state(a: &Anim, def: &AircraftDef, rot: Quat, vel: Vec3, wind: Vec3, alt: f32, on_ground: bool, t: f64) -> AcState {
    let ph = a.phase.unwrap_or(Phase::Cruise);
    let running = ph != Phase::Parked;
    let tas = (vel - wind).length();
    let mut s = AcState {
        t,
        alt,
        rot: rot.to_array(),
        vel: vel.to_array(),
        flaps: a.flaps,
        spoilers: a.spoilers,
        gear: a.gear,
        reverser: a.reverser,
        throttle: a.spool,
        brakes: if ph == Phase::Parked { 1.0 } else { 0.0 },
        tas,
        aileron: (-a.turn * 3.0).clamp(-0.3, 0.3),
        ..Default::default()
    };
    let n = if def.has_engine { def.engine_count.min(MAX_ENGINES) } else { 0 };
    s.n_engines = n as u8;
    let spool = a.spool;
    let rpm = match def.engine_kind {
        EngineKind::Jet => def.n1_rpm * spool,
        EngineKind::None => 0.0,
        _ => def.rpm_idle + (def.rpm_static - def.rpm_idle) * ((spool - 0.2) / 0.8).clamp(0.0, 1.0),
    };
    let thrust = if a.reverser > 0.5 { -0.4 } else { ((spool - 0.2) / 0.8).clamp(0.0, 1.0) };
    for e in s.engines[..n].iter_mut() {
        *e = EngineState { rpm: if running { rpm } else { 0.0 }, spool: if running { spool.max(0.2) } else { 0.0 }, thrust: if running { thrust } else { 0.0 }, running };
    }
    if def.rotor.is_some() {
        s.flags |= flags::ROTORCRAFT;
        let nr = if running { def.rpm_static } else { 0.0 };
        s.rotor = RotorState {
            nr_rpm: nr,
            nr_pct: if running { 100.0 } else { 0.0 },
            tail_rpm: nr * 9.24,
            coning_deg: if on_ground { 1.0 } else { 3.5 },
            collective_deg: if on_ground { 0.0 } else { 6.0 },
            ..Default::default()
        };
    }
    if on_ground {
        s.flags |= flags::ON_GROUND;
    }
    if a.gear > 0.5 {
        s.flags |= flags::GEAR_LEVER;
    }
    s.lights = light_bits(ph, alt, vel.length(), running, def.category == crate::flight::Category::Drone);
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(on_ground: bool, gs_kt: f32, vs_fpm: f32, agl_ft: f32) -> Motion {
        Motion { on_ground, gs_kt, vs_fpm, agl_ft, since_ground_s: None, ..Default::default() }
    }

    #[test]
    fn phases() {
        assert_eq!(phase(&Motion { still_s: 300.0, ..m(true, 0.0, 0.0, 0.0) }), Phase::Parked);
        assert_eq!(phase(&m(true, 0.0, 0.0, 0.0)), Phase::Taxi); // holding
        assert_eq!(phase(&m(true, 15.0, 0.0, 0.0)), Phase::Taxi);
        assert_eq!(phase(&Motion { accel_kt_s: 4.0, ..m(true, 120.0, 0.0, 0.0) }), Phase::TakeoffRoll);
        assert_eq!(phase(&Motion { accel_kt_s: -5.0, ..m(true, 100.0, 0.0, 0.0) }), Phase::LandingRoll);
        assert_eq!(phase(&m(false, 160.0, 2500.0, 800.0)), Phase::Departure);
        assert_eq!(phase(&m(false, 140.0, -750.0, 1500.0)), Phase::Approach);
        assert_eq!(phase(&m(false, 300.0, 2000.0, 15000.0)), Phase::Climb);
        assert_eq!(phase(&m(false, 450.0, 0.0, 36000.0)), Phase::Cruise);
        assert_eq!(phase(&m(false, 300.0, -1800.0, 20000.0)), Phase::Descent);
        // Level at 2,000 ft: shortly after take-off (level-off) vs inbound.
        assert_eq!(phase(&Motion { since_ground_s: Some(60.0), ..m(false, 180.0, 0.0, 2000.0) }), Phase::Departure);
        assert_eq!(phase(&m(false, 180.0, 0.0, 2000.0)), Phase::Approach);
        assert_eq!(phase(&Motion { rotorcraft: true, ..m(false, 5.0, 0.0, 300.0) }), Phase::Hover);
    }

    #[test]
    fn rest_pitch_of_types() {
        let defs = crate::flight::catalogue();
        let get = |id: &str| defs.iter().find(|d| d.id == id).unwrap().clone();
        // Taildraggers sit nose-up, tricycles about level.
        assert!(rest_pitch(&get("cub")) > 0.12, "{}", rest_pitch(&get("cub")));
        assert!(rest_pitch(&get("meridian320")).abs() < 0.05);
        // Standing on its wheels: the lowest contact touches.
        let d = get("meridian320");
        let c = contacts(&d, 1.0);
        let h = rest_height(&c, attitude_quat(90.0, rest_pitch(&d), 0.0));
        assert!(h > 1.5 && h < 6.0, "{h}");
    }

    /// An A320 on approach: gear down, flaps out, nose a few degrees up,
    /// landing lights; then on the runway: spoilers and reversers.
    #[test]
    fn approach_and_landing_configuration() {
        let defs = crate::flight::catalogue();
        let d = defs.iter().find(|d| d.id == "meridian320").unwrap();
        let c = contacts(d, 1.0);
        let rp = rest_pitch(d);
        let mut a = Anim::default();
        let vel = Vec3::new(-70.0, -3.6, 0.0); // westbound at 136 kt, 3° glide
        let mut q = Quat::IDENTITY;
        for _ in 0..1200 {
            q = a.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel,
                wind: Vec3::ZERO,
                on_ground: false,
                agl: 300.0,
                alt_m: 330.0,
                heading: None,
                roll: None,
                liftoff_in: None,
                ground_heading: None,
                at_stand: false,
            });
        }
        assert_eq!(a.phase, Some(Phase::Approach));
        assert!(a.gear > 0.99);
        assert!(a.flaps > 0.6, "flaps {}", a.flaps);
        let pitch = a.pitch.to_degrees();
        assert!(pitch > 0.5 && pitch < 5.0, "approach pitch {pitch:.1}°");
        assert!((a.heading - 270.0).abs() < 1.0, "{}", a.heading);
        let f = q * Vec3::Z;
        assert!(f.x < -0.99, "nose points west: {f:?}");
        let st = state(&a, d, q, vel, Vec3::ZERO, 330.0, false, 0.0);
        assert!(st.lights & lights::LANDING != 0 && st.lights & lights::STROBE != 0 && st.lights & lights::BEACON != 0);
        // Crosswind from the north (blowing south, +z) at 15 m/s: the nose crabs right (into it).
        let mut b = Anim::default();
        for _ in 0..240 {
            b.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel,
                wind: Vec3::new(0.0, 0.0, 15.0),
                on_ground: false,
                agl: 300.0,
                alt_m: 330.0,
                heading: None,
                roll: None,
                liftoff_in: None,
                ground_heading: None,
                at_stand: false,
            });
        }
        assert!(b.heading > 275.0 && b.heading < 290.0, "crab heading {}", b.heading);
        // Touch-down and roll-out.
        let roll = Vec3::new(-65.0, 0.0, 0.0);
        for _ in 0..120 {
            a.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel: roll,
                wind: Vec3::ZERO,
                on_ground: true,
                agl: 0.0,
                alt_m: 25.0,
                heading: None,
                roll: None,
                liftoff_in: None,
                ground_heading: None,
                at_stand: false,
            });
        }
        let mut v = roll;
        for _ in 0..240 {
            v.x += 2.0 / 60.0; // decelerating
            a.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel: v,
                wind: Vec3::ZERO,
                on_ground: true,
                agl: 0.0,
                alt_m: 25.0,
                heading: None,
                roll: None,
                liftoff_in: None,
                ground_heading: None,
                at_stand: false,
            });
        }
        assert_eq!(a.phase, Some(Phase::LandingRoll));
        assert!(a.spoilers > 0.9 && a.reverser > 0.5, "spoilers {} reverser {}", a.spoilers, a.reverser);
        assert!((a.pitch - rp).abs() < 0.03, "nose down after touch-down");
    }

    /// Take-off: rotation before lift-off, gear up after, the bank of a turn.
    #[test]
    fn takeoff_rotation_and_turns() {
        let defs = crate::flight::catalogue();
        let d = defs.iter().find(|d| d.id == "triple7").unwrap();
        let c = contacts(d, 1.0);
        let rp = rest_pitch(d);
        let mut a = Anim::default();
        for k in 0..240 {
            let gs = 60.0 + k as f32 * 0.05;
            a.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel: Vec3::new(0.0, 0.0, -gs),
                wind: Vec3::ZERO,
                on_ground: true,
                agl: 0.0,
                alt_m: 25.0,
                heading: None,
                roll: None,
                liftoff_in: Some(4.0 - k as f32 / 60.0),
                ground_heading: None,
                at_stand: false,
            });
        }
        assert!(a.pitch.to_degrees() > 4.0, "rotating: {:.1}°", a.pitch.to_degrees());
        // Climbing out, turning right at 2°/s.
        let mut h: f32 = 0.0;
        for _ in 0..1200 {
            h += 2f32.to_radians() / 60.0;
            let v = Vec3::new(h.sin(), 0.0, -h.cos()) * 85.0 + Vec3::Y * 10.0;
            a.step(&Frame {
                def: d,
                contacts: &c,
                rest_pitch: rp,
                dt: 1.0 / 60.0,
                vel: v,
                wind: Vec3::ZERO,
                on_ground: false,
                agl: 600.0,
                alt_m: 630.0,
                heading: None,
                roll: None,
                liftoff_in: None,
                ground_heading: None,
                at_stand: false,
            });
        }
        assert!(a.gear < 0.01, "gear up after lift-off");
        let bank = a.bank.to_degrees();
        // tan φ = V ω / g = 85 × 0.0349 / 9.81 → 16.8°.
        assert!((bank - 16.8).abs() < 2.5, "bank {bank:.1}°");
        assert!(a.pitch.to_degrees() > 8.0, "climb pitch {:.1}", a.pitch.to_degrees());
    }

    #[test]
    fn lights_by_phase() {
        assert_eq!(light_bits(Phase::Parked, 25.0, 0.0, false, false), lights::NAV);
        assert_eq!(light_bits(Phase::Taxi, 25.0, 6.0, true, false), lights::NAV | lights::BEACON | lights::TAXI);
        assert_eq!(light_bits(Phase::Cruise, 11_000.0, 230.0, true, false), lights::NAV | lights::BEACON | lights::STROBE);
        assert_eq!(light_bits(Phase::Approach, 900.0, 75.0, true, false), lights::NAV | lights::BEACON | lights::STROBE | lights::LANDING);
        assert_eq!(light_bits(Phase::Hover, 100.0, 0.0, true, true), 0);
    }
}
