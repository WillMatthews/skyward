//! Rotorcraft: a real helicopter (Airbus Helicopters EC130 B4) and an FPV
//! quadcopter, each with its own flight model plugged into the common 6-DoF
//! integrator of [`crate::flight::Aircraft`] (ground contact, touchdown and
//! crash checks, structure, integration are shared; this module supplies the
//! forces and moments, the drive train and the pilot assistance).
//!
//! * [`heli`] — blade-element / momentum main rotor (collective, cyclic,
//!   twist, Glauert inflow with dynamic-inflow lag, ground effect, vortex
//!   ring state from the empirical induced-velocity curve, translational
//!   lift emerging from the inflow, flapping with blowback / coning / rate
//!   lag, hub stiffness, blade stall with advance ratio = retreating-blade
//!   stall), rotor RPM as a state with the free-turbine engine, FADEC
//!   governor and freewheel (autorotation), shrouded tail rotor with pedals
//!   and drive / pitch failures, fuselage / fin / stabiliser aerodynamics,
//!   and a stability-assist system (SAS / attitude hold).
//! * [`quad`] — four motors and props (thrust / torque from rotor speed,
//!   inflow loss, motor lag), X-frame mixer with airmode, Betaflight-style
//!   rates and PID loops in rate (acro) and angle modes, LiPo battery with
//!   sag and capacity, prop wash (the props descending into their own wake).
//!
//! Conventions as `flight.rs`: body +Z nose, +Y up, +X left; p = ω.z (roll
//! right), q = −ω.x (pitch up), r = −ω.y (yaw right).

pub mod bot;
pub mod defs;
pub mod heli;
pub mod instruments;
pub mod quad;
pub mod quad_model;
#[cfg(test)]
mod tests;

use crate::flight::{Aircraft, AircraftDef, Atmosphere, Controls};
use glam::Vec3;

pub use defs::{ec130_def, quad_def};

/// What kind of rotorcraft (and its static data).
#[derive(Clone, Debug)]
pub enum RotorDef {
    Heli(heli::HeliDef),
    Quad(quad::QuadDef),
}

impl RotorDef {
    pub fn heli(&self) -> Option<&heli::HeliDef> {
        match self {
            RotorDef::Heli(h) => Some(h),
            _ => None,
        }
    }
    pub fn quad(&self) -> Option<&quad::QuadDef> {
        match self {
            RotorDef::Quad(q) => Some(q),
            _ => None,
        }
    }
}

/// Pilot assistance. Helicopter: `Off` (raw controls), `Sas` (rate damping),
/// `Att` (attitude command / hold, heading hold, hover velocity hold — the
/// keyboard default). Quadcopter: `Att` = angle (self-levelling) mode,
/// `Off` = rate (acro) mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Assist {
    Off,
    Sas,
    #[default]
    Att,
}

impl Assist {
    pub fn label(self, quad: bool) -> &'static str {
        match (self, quad) {
            (Assist::Att, true) => "ANGLE",
            (_, true) => "ACRO",
            (Assist::Off, false) => "SAS OFF",
            (Assist::Sas, false) => "SAS",
            (Assist::Att, false) => "ATT HOLD",
        }
    }
    /// Next mode (the assist key).
    pub fn next(self, quad: bool) -> Assist {
        match (self, quad) {
            (Assist::Att, true) => Assist::Off,
            (_, true) => Assist::Att,
            (Assist::Att, false) => Assist::Off,
            (Assist::Off, false) => Assist::Sas,
            (Assist::Sas, false) => Assist::Att,
        }
    }
}

/// Rotorcraft failures (set by the systems / failures panel or tests).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RotorFailures {
    /// Tail-rotor drive failure: no anti-torque thrust at all.
    pub tail_drive: bool,
    /// Tail-rotor pitch-control failure: blade pitch frozen where it was.
    pub tail_pitch: bool,
    /// Hydraulic / servo loss: heavy, slow controls and no SAS.
    pub hydraulics: bool,
}

/// Live rotorcraft state (kept inside [`Aircraft`]).
#[derive(Clone, Debug, Default)]
pub struct RotorState {
    pub heli: heli::HeliState,
    pub quad: quad::QuadState,
    pub assist: Assist,
    pub fail: RotorFailures,
    /// Outputs of the last step, for instruments / audio / effects.
    pub out: RotorOut,
}

/// What the rest of the app reads about the rotor(s) after a step.
#[derive(Clone, Copy, Debug, Default)]
pub struct RotorOut {
    /// Main rotor speed, % of nominal (quad: mean motor speed, % of max).
    pub nr_pct: f32,
    /// Main rotor speed, rpm (quad: mean motor rpm).
    pub nr_rpm: f32,
    /// Tail rotor speed, rpm.
    pub tail_rpm: f32,
    /// Gas generator speed Ng, %.
    pub ng_pct: f32,
    /// Free (power) turbine speed N2, %.
    pub n2_pct: f32,
    /// Engine output torque, % (100 % = transmission limit at 100 % Nr).
    pub torque_pct: f32,
    /// Turbine outlet temperature, °C.
    pub tot_c: f32,
    /// First limitation indicator (the VEMD needle), 0..~11 (10 = limit).
    pub fli: f32,
    /// Main rotor thrust, N; induced velocity, m/s; hub height above the ground, m.
    pub thrust: f32,
    pub v_induced: f32,
    pub hub_agl: f32,
    /// Advance ratio.
    pub mu: f32,
    /// Vortex-ring-state / prop-wash severity 0..1, retreating-blade stall 0..1.
    pub vrs: f32,
    pub stall: f32,
    /// Blade pitch: collective (deg), coning (deg), disc tilt (deg, aft / right).
    pub collective_deg: f32,
    pub coning_deg: f32,
    pub tilt_deg: [f32; 2],
    /// Tail-rotor blade pitch, deg.
    pub tail_pitch_deg: f32,
    /// Engine shaft power, W; power available, W.
    pub power_w: f32,
    pub power_avail_w: f32,
    /// Main rotor azimuth (rad, for blade-slap phase) and tail azimuth.
    pub azimuth: f32,
    /// Low / high rotor RPM warnings.
    pub low_nr: bool,
    pub high_nr: bool,
    /// Quadcopter: battery voltage (pack), current (A), used (mAh), charge 0..1;
    /// per-motor speed fraction 0..1.
    pub batt_v: f32,
    pub batt_a: f32,
    pub batt_mah: f32,
    pub batt_soc: f32,
    pub motors: [f32; 4],
    /// Effective pilot inputs after assistance (for the cockpit controls).
    pub eff: [f32; 4],
}

/// Forces and moments the rotor model hands to the integrator (body frame).
pub struct Out {
    pub force: Vec3,
    pub moment: Vec3,
    /// Extra linear rate damping per axis (N·m per rad/s, ≥ 0), integrated
    /// implicitly by the caller.
    pub damping: Vec3,
    /// Spinning-rotor angular momentum (body frame), for gyroscopic moments.
    pub h: Vec3,
    /// Main-rotor thrust (structural "lift") and its direction.
    pub lift: f32,
}

/// One rotorcraft step: called by [`Aircraft::step`] instead of the
/// fixed-wing engines / control law / aerodynamics.
#[allow(clippy::too_many_arguments)]
pub fn step(rd: &RotorDef, def: &AircraftDef, ac: &mut Aircraft, c: &Controls, vb: Vec3, agl: f32, atm: &Atmosphere, dt: f32) -> Out {
    match rd {
        RotorDef::Heli(h) => heli::step(h, def, ac, c, vb, agl, atm, dt),
        RotorDef::Quad(q) => quad::step(q, def, ac, c, vb, agl, atm, dt),
    }
}

/// Put a freshly spawned rotorcraft in its steady state: rotors at flight
/// RPM, and (in the air) trimmed for `speed` m/s at `pos.y`.
pub fn settle(rd: &RotorDef, def: &AircraftDef, ac: &mut Aircraft, ctl: &mut Controls, airborne: bool) {
    match rd {
        RotorDef::Heli(h) => heli::settle(h, def, ac, ctl, airborne),
        RotorDef::Quad(q) => quad::settle(q, def, ac, ctl, airborne),
    }
}

/// Animation / instrument channels of the rotorcraft (`rotor_*`, `tail_*`,
/// `heli_*`, `quad_*`); None for any other name.
pub fn channel(name: &str, ac: &Aircraft) -> Option<f32> {
    let o = &ac.rotor.out;
    Some(match name {
        "rotor_rpm" => o.nr_rpm,
        "rotor_nr_pct" => o.nr_pct,
        "tail_rpm" => o.tail_rpm,
        "rotor_coning_deg" => o.coning_deg,
        "rotor_collective_deg" => o.collective_deg,
        "rotor_tilt_aft_deg" => o.tilt_deg[0],
        "rotor_tilt_right_deg" => o.tilt_deg[1],
        "tail_pitch_deg" => o.tail_pitch_deg,
        "heli_n2_rpm" => o.n2_pct * 0.01 * heli::N2_RPM_100,
        "heli_ng_pct" => o.ng_pct,
        "heli_tq_pct" => o.torque_pct,
        "heli_tot_c" => o.tot_c,
        "heli_fli" => o.fli,
        "heli_low_nr" => o.low_nr as u8 as f32,
        "ctl_collective" => o.eff[0],
        _ => return None,
    })
}
