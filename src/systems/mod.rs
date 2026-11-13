//! Aircraft systems and failures: electrics (battery, alternators /
//! generators, APU, RAT / EPU, buses, ammeter), fuel (tanks, feed and
//! selectors, crossfeed, burn per engine, mass and lateral CG fed back into
//! the flight model, leaks, starvation), hydraulics (engine / electric /
//! RAT pumps, PTU, and what depends on them: gear, flaps, spoilers, brakes,
//! nose-wheel steering, powered flight controls), the vacuum system and the
//! pitot-static system behind the instruments, engine start procedures
//! (magnetos / primer / starter; APU bleed / starter-generator / JFS; cold
//! and dark or ready to go), oil, carburettor icing, engine fire detection
//! and extinguishers, anti-ice and airframe icing — and the failures of all
//! of these ([`failures`]), with the master warning / caution, ECAM / EICAS
//! messages and their procedures ([`alerts`]).
//!
//! Per-type definitions are data ([`defs`]), with defaults by category /
//! engine kind. The systems live in [`Aircraft::sys`]; the session calls
//! [`step`] before each physics step and the flight model reads only the
//! [`Effects`] (neutral unless something failed or was switched off, so the
//! flight model is unchanged without failures). The instruments read the
//! *indicated* air data and attitude through [`Systems::air_data`].

pub mod alerts;
pub mod defs;
pub mod failures;
mod session;
#[cfg(test)]
mod tests;

use crate::flight::{Aircraft, AircraftDef, Controls, EngineKind, KT, MAX_ENGINES, Surfaces};
use defs::{Alerting, Feed, Starter, SysDef};
use failures::{Active, Failure, Kind};
use glam::Vec3;

const ME: usize = MAX_ENGINES;
const P0: f32 = 101_325.0;
const A0: f32 = 340.294;

/// What the systems do to the flight model this step. `Default` is neutral.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Effects {
    /// Engines not running (bit per engine): shut down, starved, failed.
    pub engine_cut: u8,
    /// Power available per engine (1 = normal).
    pub power: [f32; ME],
    /// Propeller overspeed: extra RPM fraction (governor failure).
    pub rpm_over: [f32; ME],
    /// Starter cranking a stopped engine: fraction of idle speed.
    pub crank: [f32; ME],
    /// Combustion during a start (EGT rises before the engine is running).
    pub lit: u8,
    /// Extra EGT (°C): fire, bird damage.
    pub egt_add: [f32; ME],
    /// Mass change from the start (fuel burnt, ice), kg.
    pub mass_delta: f32,
    /// Centre-of-gravity offset from fuel imbalance (body frame, m).
    pub cg: Vec3,
    /// Gear extension / retraction speed factors (0 = won't move).
    pub gear_down: f32,
    pub gear_up: f32,
    /// Flap motion speed factor, and flaps held (jammed / locked).
    pub flap_rate: f32,
    pub flap_hold: Option<f32>,
    /// Split flaps: right minus left extension (0..1 lever units).
    pub flap_asym: f32,
    /// Spoiler / speedbrake authority 0..1.
    pub spoilers: f32,
    /// Powered flight-control rate factor: elevator, aileron, rudder.
    pub surface_rate: [f32; 3],
    /// Jammed surfaces: elevator, aileron, rudder.
    pub jam: [Option<f32>; 3],
    /// Wheel-brake effectiveness 0..1.
    pub brakes: f32,
    /// Nose-wheel steering 0..1.
    pub steering: f32,
    /// Burst tyres: left, right main.
    pub tyre: [bool; 2],
    /// Airframe ice 0..1.
    pub ice: f32,
}

impl Default for Effects {
    fn default() -> Self {
        Effects {
            engine_cut: 0,
            power: [1.0; ME],
            rpm_over: [0.0; ME],
            crank: [0.0; ME],
            lit: 0,
            egt_add: [0.0; ME],
            mass_delta: 0.0,
            cg: Vec3::ZERO,
            gear_down: 1.0,
            gear_up: 1.0,
            flap_rate: 1.0,
            flap_hold: None,
            flap_asym: 0.0,
            spoilers: 1.0,
            surface_rate: [1.0; 3],
            jam: [None; 3],
            brakes: 1.0,
            steering: 1.0,
            tyre: [false; 2],
            ice: 0.0,
        }
    }
}

impl Effects {
    /// Is the tyre of a main wheel at body x (+ = left) burst?
    pub fn tyre_burst(&self, x: f32) -> bool {
        (x > 0.1 && self.tyre[0]) || (x < -0.1 && self.tyre[1])
    }
}

/// Hydraulic rate limits / jams on the actuators (after the control law).
pub fn apply_actuators(fx: &Effects, before: &Surfaces, s: &mut Surfaces) {
    let axis = |k: usize, b: f32, v: &mut f32| {
        if fx.surface_rate[k] < 1.0 {
            *v = b + (*v - b) * fx.surface_rate[k].max(0.0);
        }
        if let Some(j) = fx.jam[k] {
            *v = j;
        }
    };
    axis(0, before.elevator, &mut s.elevator);
    axis(1, before.aileron, &mut s.aileron);
    axis(2, before.rudder, &mut s.rudder);
    if fx.flap_rate < 1.0 {
        s.flaps = before.flaps + (s.flaps - before.flaps) * fx.flap_rate.max(0.0);
    }
    if let Some(h) = fx.flap_hold {
        s.flaps = h;
    }
}

/// Aerodynamic penalties of airframe ice and split flaps: (force, moment), body frame.
pub fn airframe_forces(fx: &Effects, def: &AircraftDef, vb: Vec3, force: Vec3, alpha: f32, _flaps: f32, q_s: f32) -> (Vec3, Vec3) {
    let mut f = Vec3::ZERO;
    let mut m = Vec3::ZERO;
    let d = vb.normalize_or_zero();
    if fx.ice > 0.0 && d != Vec3::ZERO {
        // Ice destroys the lift near the stall first (CLmax −30 % with a full
        // load), and adds drag everywhere.
        let along = force.dot(d);
        let lift = force - d * along;
        let a_rel = (alpha / def.alpha_stall.max(0.05)).clamp(0.0, 1.0);
        let loss = fx.ice * (0.05 + 0.27 * a_rel * a_rel);
        f -= lift * loss;
        f -= d * (q_s * 0.035 * fx.ice);
    }
    if fx.flap_asym != 0.0 {
        // More flap on the right lifts the right wing: roll left.
        let span = def.wingspan.max(1.0);
        m.z -= fx.flap_asym * def.flap_cl.abs().max(0.3) * q_s * span * 0.16;
    }
    (f, m)
}

/// Outside conditions the systems need (filled from the weather by the session).
#[derive(Clone, Copy, Debug)]
pub struct Ambient {
    /// Outside air temperature, °C (None: ISA at the aircraft's height).
    pub oat_c: Option<f32>,
    /// Temperature − dew point at the surface, °C.
    pub spread_c: f32,
    /// Visible moisture 0..1 (in cloud, precipitation).
    pub moisture: f32,
}

impl Default for Ambient {
    fn default() -> Self {
        Ambient { oat_c: None, spread_c: 15.0, moisture: 0.0 }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mags {
    Off,
    Left,
    Right,
    #[default]
    Both,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Selector {
    Off,
    Left,
    Right,
    #[default]
    Both,
}

/// Cockpit switches and levers the systems model needs (beyond `Controls`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Switches {
    /// Battery / master switch.
    pub battery: bool,
    /// Alternator / generator switch per engine.
    pub gens: [bool; ME],
    /// Avionics master (also isolates an avionics smoke source).
    pub avionics: bool,
    pub apu_master: bool,
    pub apu_bleed: bool,
    pub selector: Selector,
    pub crossfeed: bool,
    /// Boost pumps (all tanks).
    pub pumps: bool,
    pub mags: [Mags; ME],
    /// Mixture rich (pistons) / engine master or condition lever (turbines).
    pub eng_master: [bool; ME],
    /// Turbines: ignition / start mode.
    pub ign_start: bool,
    pub carb_heat: bool,
    pub pitot_heat: bool,
    pub wing_anti_ice: bool,
    pub eng_anti_ice: bool,
    /// Gear alternate extension pulled.
    pub alt_gear: bool,
    /// Engine fire handle / push-button (cuts fuel, bleed, hydraulics, generator).
    pub fire_handle: [bool; ME],
    /// Electric hydraulic pumps (auto).
    pub hyd_elec: bool,
}

impl Switches {
    fn ready() -> Switches {
        Switches {
            battery: true,
            gens: [true; ME],
            avionics: true,
            apu_master: false,
            apu_bleed: false,
            selector: Selector::Both,
            crossfeed: false,
            pumps: true,
            mags: [Mags::Both; ME],
            eng_master: [true; ME],
            ign_start: false,
            carb_heat: false,
            pitot_heat: true,
            wing_anti_ice: false,
            eng_anti_ice: false,
            alt_gear: false,
            fire_handle: [false; ME],
            hyd_elec: true,
        }
    }
    fn cold() -> Switches {
        Switches { battery: false, gens: [true; ME], avionics: false, pumps: false, mags: [Mags::Off; ME], eng_master: [false; ME], pitot_heat: false, ..Switches::ready() }
    }
}

/// Electrical power available.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Power {
    #[default]
    Off,
    /// Battery only.
    Battery,
    /// Emergency generator (RAT / EPU): essential buses.
    Emergency,
    Normal,
}

/// One engine's accessories and state, as the systems see it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Eng {
    /// Combustion (the flight model's engine is cut when false).
    pub run: bool,
    /// Start progress 0..1 (turbines: towards idle; pistons: cranking time).
    pub start: f32,
    /// Starter engaged (pistons: seconds of cranking left).
    pub crank: f32,
    /// Turbine start requested (the start sequence is running).
    pub starting: bool,
    /// Primer strokes (pistons).
    pub primer: f32,
    /// Fire intensity 0..1 and how long it has burnt (s).
    pub fire: f32,
    pub fire_t: f32,
    /// Extinguisher bottles discharged, and seconds since the last one.
    pub agent: u8,
    pub agent_t: f32,
    /// Damaged beyond restarting (fire, seizure).
    pub dead: bool,
    /// Seconds without fuel at the engine.
    pub starve: f32,
    /// Carburettor ice 0..1.
    pub carb_ice: f32,
    /// Bird-strike damage 0..1.
    pub bird: f32,
    /// Seconds a transient flameout cause remains.
    pub flameout_t: f32,
    /// Overspeed build-up 0..1.
    pub overspeed: f32,
    /// Oil temperature rise (°C) and pressure factor for the gauges.
    pub oil_t_add: f32,
    pub oil_p_mul: f32,
    /// Fuel reaching the engine.
    pub fed: bool,
}

/// Aural alerts (the audio director plays them).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Aural {
    #[default]
    None,
    /// Single caution chime.
    Chime,
    /// Continuous repetitive chime (Airbus / ATR warnings) — repeats while the master warning is lit.
    Crc,
    /// Fire bell (Embraer) — repeats while lit.
    Bell,
    /// Voice messages.
    EngineFire,
    Warning,
    Caution,
    Smoke,
}

/// Indicated air data and attitude (what the instruments show).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirData {
    /// Indicated airspeed m/s, altitude m (world y), vertical speed m/s.
    pub ias: f32,
    pub alt: f32,
    pub vs: f32,
    /// Attitude and heading (rad), turn rate (rad/s).
    pub pitch: f32,
    pub roll: f32,
    pub heading: f32,
    pub turn: f32,
    /// Invalid-data flags ([`flag`]).
    pub flags: u8,
}

/// [`AirData::flags`] bits.
pub mod flag {
    pub const SPD: u8 = 1;
    pub const ALT: u8 = 2;
    pub const VS: u8 = 4;
    pub const ATT: u8 = 8;
    pub const HDG: u8 = 16;
    /// Electric turn coordinator off flag.
    pub const TURN: u8 = 32;
}

impl AirData {
    pub fn ias_kt(&self) -> f32 {
        self.ias / KT
    }
    pub fn has(&self, f: u8) -> bool {
        self.flags & f != 0
    }
}

/// Live state of all systems of one aircraft.
#[derive(Clone, Debug)]
pub struct Systems {
    pub init: bool,
    pub sd: SysDef,
    pub n_eng: usize,
    pub sw: Switches,
    /// Active failures.
    pub fail: Active,
    pub eng: [Eng; ME],
    // ---- fuel
    pub tank: [f32; 3],
    pub tank_cap: [f32; 3],
    pub fuel0: f32,
    /// Never let the fuel fall below this fraction (auto-flight bot keeps flying).
    pub fuel_floor: f32,
    // ---- electrics
    pub power: Power,
    /// Battery state of charge 0..1, bus volts, battery current (+ = charging).
    pub soc: f32,
    pub volts: f32,
    pub amps: f32,
    pub gen_on: [bool; ME],
    /// APU speed 0..1 and start requested.
    pub apu: f32,
    pub apu_start: bool,
    pub rat_out: bool,
    /// Emergency power unit seconds left.
    pub epu_s: f32,
    // ---- hydraulics (pressure 0..1)
    pub hyd: [f32; 3],
    pub brake_accu: f32,
    brake_held: bool,
    // ---- instruments
    /// Vacuum gyros' rotor speed 0..1; electric turn gyro.
    pub spin: f32,
    pub tc_spin: f32,
    /// Trapped pitot total pressure (Pa); frozen static pressure and altitude.
    pub pitot_p: Option<f32>,
    pub static_p: Option<(f32, f32)>,
    pub pitot_ice: f32,
    ai: Option<(f32, f32)>,
    dg: Option<f32>,
    drift_dir: (f32, f32),
    // ---- airframe
    pub ice: f32,
    pub smoke: f32,
    smoke_iso_t: f32,
    pub flap_lock: Option<f32>,
    pub jam: [Option<f32>; 3],
    pub windscreen_cracked: bool,
    /// Icing happens with visible moisture below freezing (natural); the bot keeps it off.
    pub natural_icing: bool,
    pub protect: bool,
    // ---- alerting
    pub master_warn: bool,
    pub master_caut: bool,
    known: [u32; 25],
    /// Aural alert sequence (bumped for one-shots) and the continuous one.
    pub aural_seq: u32,
    pub aural: Aural,
    pub aural_loop: Aural,
    /// Advisory for a toast (sequence, text).
    pub note_seq: u32,
    pub note_seen: u32,
    pub note: &'static str,
    // ---- automation
    /// Auto-start sequence step (0 = idle).
    pub autostart: u8,
    auto_t: f32,
    pub time: f32,
    rng: u64,
    /// What the flight model gets.
    pub fx: Effects,
}

impl Default for Systems {
    fn default() -> Self {
        Systems {
            init: false,
            sd: SysDef::default(),
            n_eng: 0,
            sw: Switches::ready(),
            fail: Active::default(),
            eng: [Eng { oil_p_mul: 1.0, ..Default::default() }; ME],
            tank: [0.0; 3],
            tank_cap: [0.0; 3],
            fuel0: 0.0,
            fuel_floor: 0.0,
            power: Power::Normal,
            soc: 1.0,
            volts: 0.0,
            amps: 0.0,
            gen_on: [false; ME],
            apu: 0.0,
            apu_start: false,
            rat_out: false,
            epu_s: 0.0,
            hyd: [1.0; 3],
            brake_accu: 0.0,
            brake_held: false,
            spin: 1.0,
            tc_spin: 1.0,
            pitot_p: None,
            static_p: None,
            pitot_ice: 0.0,
            ai: None,
            dg: None,
            drift_dir: (1.0, 1.0),
            ice: 0.0,
            smoke: 0.0,
            smoke_iso_t: 0.0,
            flap_lock: None,
            jam: [None; 3],
            windscreen_cracked: false,
            natural_icing: true,
            protect: false,
            master_warn: false,
            master_caut: false,
            known: [0; 25],
            aural_seq: 0,
            aural: Aural::None,
            aural_loop: Aural::None,
            note_seq: 0,
            note_seen: 0,
            note: "",
            autostart: 0,
            auto_t: 0.0,
            time: 0.0,
            rng: 0x9E37_79B9_7F4A_7C15,
            fx: Effects::default(),
        }
    }
}

/// Fuel on board at the start (kg) for a type — the instruments' and the
/// systems' common figure (`cockpit::channels::fuel_load`).
pub fn fuel_load(def: &AircraftDef) -> f32 {
    crate::cockpit::channels::fuel_load(def.id, def.category).0
}

/// Advance the systems of `ac` by `dt` (call before each physics step).
pub fn step(ac: &mut Aircraft, def: &AircraftDef, ctl: &Controls, amb: &Ambient, dt: f32) {
    let mut sys = std::mem::take(&mut ac.sys);
    if !sys.init {
        sys = Systems::new(def, false);
    }
    if ac.crashed.is_none() && dt > 0.0 {
        sys.update(def, ac, ctl, amb, dt);
    }
    ac.sys = sys;
}

fn qc_from_cas(v: f32) -> f32 {
    P0 * ((1.0 + 0.2 * (v / A0).powi(2)).powf(3.5) - 1.0)
}
fn cas_from_qc(qc: f32) -> f32 {
    A0 * (5.0 * ((qc.max(0.0) / P0 + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt()
}

impl Systems {
    /// Systems for `def`, ready to go (engines running) or cold and dark.
    pub fn new(def: &AircraftDef, cold: bool) -> Systems {
        let sd = defs::sys_def(def);
        let n_eng = if def.has_engine { def.engine_count.min(def.engine_pos.len()).min(ME) } else { 0 };
        let mut s = Systems { init: true, sd, n_eng, ..Default::default() };
        let fuel = fuel_load(def);
        for (k, t) in sd.tanks.iter().enumerate().take(3) {
            s.tank[k] = fuel * t.1;
            s.tank_cap[k] = fuel * t.1 / 0.75;
        }
        s.fuel0 = s.fuel_total();
        s.brake_accu = sd.brake_accu as f32;
        s.epu_s = sd.epu_min * 60.0;
        s.rng ^= (def.id.len() as u64) << 7 | def.mass.to_bits() as u64;
        let r = s.rand();
        s.drift_dir = (if r > 0.5 { 1.0 } else { -1.0 }, if s.rand() > 0.5 { 0.7 } else { -0.5 });
        if cold {
            s.sw = Switches::cold();
            s.power = Power::Off;
            s.spin = 0.0;
            s.tc_spin = 0.0;
            s.hyd = [0.0; 3];
            s.ai = Some((0.0, 0.0));
            s.dg = None;
            for e in s.eng.iter_mut() {
                e.run = false;
            }
        } else {
            for e in s.eng.iter_mut().take(n_eng) {
                e.run = true;
                e.fed = true;
            }
            s.gen_on = [true; ME];
            s.volts = sd.volts;
        }
        s.fx = s.effects(def, None);
        s
    }

    fn rand(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        (x >> 40) as f32 / (1u64 << 24) as f32
    }

    pub fn fuel_total(&self) -> f32 {
        self.tank.iter().sum()
    }
    pub fn fuel_capacity(&self) -> f32 {
        self.tank_cap.iter().sum()
    }
    pub fn has(&self, k: Kind, i: usize) -> bool {
        self.fail.has(Failure::new(k, i))
    }
    /// Bus powered enough for the avionics.
    pub fn bus_ok(&self) -> bool {
        self.power >= Power::Emergency || (self.power == Power::Battery && self.volts > 0.855 * self.sd.batt_v)
    }
    fn hyd_any(&self, mask: u8) -> bool {
        (0..self.sd.hyd.len()).any(|k| mask & (1 << k) != 0 && self.hyd[k] > 0.5)
    }
    fn hyd_frac(&self, mask: u8) -> f32 {
        let n = (0..self.sd.hyd.len()).filter(|k| mask & (1 << k) != 0).count();
        if n == 0 {
            return 1.0;
        }
        (0..self.sd.hyd.len()).filter(|k| mask & (1 << k) != 0 && self.hyd[*k] > 0.5).count() as f32 / n as f32
    }
    /// Hydraulic pressure of system `k` in psi (3000 psi systems).
    pub fn hyd_psi(&self, k: usize) -> f32 {
        self.hyd.get(k).copied().unwrap_or(0.0) * 3000.0
    }
    fn note(&mut self, s: &'static str) {
        self.note = s;
        self.note_seq = self.note_seq.wrapping_add(1);
    }

    /// Is a glass display of `kind` ("pfd", "nd", "ewd", "sd", "isis", "mcdu") powered?
    pub fn display_on(&self, kind: &str) -> bool {
        if !self.init {
            return true;
        }
        match self.power {
            Power::Normal => true,
            Power::Emergency => matches!(kind, "pfd" | "nd" | "ewd" | "isis"),
            Power::Battery => matches!(kind, "pfd" | "isis") && self.bus_ok() || kind == "isis",
            Power::Off => false,
        }
    }
    /// The captain's PFD shows the red X of a display-unit failure.
    pub fn pfd_red_x(&self) -> bool {
        self.fail.any_of(Kind::DisplayFail)
    }
    /// Vacuum (suction gauge) available.
    pub fn vacuum_ok(&self) -> bool {
        self.sd.vacuum && !self.fail.any_of(Kind::Vacuum) && (0..self.n_eng).any(|i| self.eng[i].run)
    }
    /// Cockpit smoke 0..1 (electrical fire, engine fire on single-engine types).
    pub fn cockpit_smoke(&self) -> f32 {
        let fire_in = if self.n_eng == 1 { (self.eng[0].fire * 0.6).min(0.6) } else { 0.0 };
        self.smoke.max(fire_in)
    }

    /// Make failure `f` happen now (or repair it).
    pub fn set_failure(&mut self, def: &AircraftDef, ac: &Aircraft, f: Failure, on: bool) {
        if !self.init {
            *self = Systems::new(def, false);
        }
        if !f.kind.applies(&self.sd, def) || (f.idx as usize) >= f.kind.count(&self.sd, def).max(1) {
            return;
        }
        let was = self.fail.has(f);
        self.fail.set(f, on);
        let i = f.idx as usize;
        if on && !was {
            match f.kind {
                Kind::EngineFire => {
                    let e = &mut self.eng[i];
                    e.fire = e.fire.max(0.25);
                    e.fire_t = 0.0;
                    if !self.sd.fire_detect {
                        self.note(if self.n_eng == 1 { "Smoke and flames from the engine compartment!" } else { "Flames from an engine!" });
                    }
                }
                Kind::Flameout => {
                    let e = &mut self.eng[i];
                    e.run = false;
                    e.start = 0.0;
                    e.flameout_t = if def.engine_kind == EngineKind::Piston { f32::INFINITY } else { 15.0 };
                }
                Kind::PitotBlock => self.pitot_p = Some(crate::flight::isa(ac.pos.y).pressure + qc_from_cas(ac.airspeed())),
                Kind::StaticBlock => self.static_p = Some((crate::flight::isa(ac.pos.y).pressure, ac.pos.y)),
                Kind::FlapAsym | Kind::FlapJam => {
                    self.flap_lock.get_or_insert(ac.surfaces().flaps);
                }
                Kind::ControlJam => {
                    let s = ac.surfaces();
                    self.jam[i.min(2)] = Some([s.elevator, s.aileron, s.rudder][i.min(2)]);
                }
                Kind::BirdStrike => {
                    self.windscreen_cracked = true;
                    if self.n_eng > 0 {
                        let k = ((self.rand() * self.n_eng as f32) as usize).min(self.n_eng - 1);
                        let dmg = 0.45 + 0.3 * self.rand();
                        self.eng[k].bird = dmg;
                        if def.engine_kind == EngineKind::Jet && self.rand() < 0.3 {
                            self.eng[k].run = false;
                            self.eng[k].flameout_t = 10.0;
                        }
                    }
                    self.note("Bird strike!");
                }
                _ => {}
            }
        } else if !on && was {
            match f.kind {
                Kind::EngineFire => {
                    self.eng[i].fire = 0.0;
                }
                Kind::Flameout => self.eng[i].flameout_t = 0.0,
                Kind::PitotBlock => {
                    if self.pitot_ice < 1.0 {
                        self.pitot_p = None;
                    }
                }
                Kind::StaticBlock => self.static_p = None,
                Kind::FlapAsym | Kind::FlapJam => {
                    if !self.fail.any_of(Kind::FlapAsym) && !self.fail.any_of(Kind::FlapJam) {
                        self.flap_lock = None;
                    }
                }
                Kind::ControlJam => self.jam[i.min(2)] = None,
                Kind::BirdStrike => {
                    self.windscreen_cracked = false;
                    for e in self.eng.iter_mut() {
                        e.bird = 0.0;
                    }
                }
                _ => {}
            }
        }
        self.fx = self.effects(def, Some(ac));
    }

    /// Repair everything: failures, damage from them, refuel nothing (fuel stays).
    pub fn repair_all(&mut self, def: &AircraftDef, ac: &Aircraft) {
        for f in self.fail.list() {
            self.set_failure(def, ac, f, false);
        }
        self.fail.clear();
        for e in self.eng.iter_mut() {
            e.fire = 0.0;
            e.fire_t = 0.0;
            e.dead = false;
            e.bird = 0.0;
            e.carb_ice = 0.0;
            e.oil_t_add = 0.0;
            e.oil_p_mul = 1.0;
            e.overspeed = 0.0;
            e.flameout_t = 0.0;
        }
        self.ice = 0.0;
        self.smoke = 0.0;
        self.pitot_ice = 0.0;
        self.pitot_p = None;
        self.static_p = None;
        self.spin = if self.vacuum_ok() || !self.sd.vacuum { 1.0 } else { self.spin };
        self.ai = None;
        self.dg = None;
        self.windscreen_cracked = false;
        self.fx = self.effects(def, Some(ac));
    }

    /// Fill the tanks back to the start load.
    pub fn refuel(&mut self) {
        for k in 0..3 {
            self.tank[k] = self.tank[k].max(self.tank_cap[k] * 0.75);
        }
    }

    /// Instantly ready to fly: switches set, engines running.
    pub fn make_ready(&mut self) {
        let keep = (self.fail, self.tank, self.tank_cap, self.fuel0);
        let ice = self.ice;
        let alt_gear = self.sw.alt_gear;
        self.sw = Switches::ready();
        self.sw.alt_gear = alt_gear;
        (self.fail, self.tank, self.tank_cap, self.fuel0) = keep;
        self.ice = ice;
        for i in 0..self.n_eng {
            let e = &mut self.eng[i];
            if !e.dead && e.fire <= 0.0 {
                e.run = true;
                e.fed = true;
                e.starve = 0.0;
                e.start = 0.0;
                e.starting = false;
            }
        }
        self.soc = self.soc.max(0.8);
        self.autostart = 0;
    }

    /// Begin the automatic start sequence (switches in order, then each engine).
    pub fn begin_autostart(&mut self) {
        self.autostart = 1;
        self.auto_t = 0.0;
    }

    /// Discharge the next fire bottle of engine `i`.
    pub fn discharge_agent(&mut self, i: usize) -> bool {
        if i >= self.n_eng || self.eng[i].agent >= self.sd.bottles {
            return false;
        }
        let cut = !self.sw.eng_master[i] || self.sw.fire_handle[i];
        let e = &mut self.eng[i];
        e.agent += 1;
        e.agent_t = 0.0;
        if e.fire > 0.0 {
            // With the fuel shut off the agent puts the fire out; with fuel
            // still flowing it only knocks it down and it comes back.
            e.fire = if cut { 0.0 } else { e.fire.min(0.25) };
        }
        true
    }

    /// Start (pistons: crank for a few seconds) engine `i`, or request a turbine start.
    pub fn press_start(&mut self, i: usize) {
        if i >= self.n_eng {
            return;
        }
        match self.sd.starter {
            Starter::Piston => self.eng[i].crank = 4.0,
            Starter::None => {}
            _ => {
                self.eng[i].starting = true;
                self.sw.ign_start = true;
            }
        }
    }

    /// One step.
    pub fn update(&mut self, def: &AircraftDef, ac: &mut Aircraft, ctl: &Controls, amb: &Ambient, dt: f32) {
        self.time += dt;
        let n = self.n_eng;
        let sd = self.sd;
        let ias = ac.airspeed();
        let alt = ac.pos.y;
        let airborne = !ac.on_ground;
        let oat = amb.oat_c.unwrap_or(ac.oat());
        self.run_autostart(dt);

        // ---------------- electrics
        for i in 0..ME {
            let gen_fail = self.has(Kind::Generator, i);
            let turning = i < n && self.eng[i].run;
            self.gen_on[i] =
                i < n && sd.gens_per_engine > 0 && self.sw.gens[i] && !self.sw.fire_handle[i] && turning && (!gen_fail || sd.gens_per_engine > 1) && self.sw.battery_field(sd);
        }
        // APU
        let apu_fuel = self.fuel_total() > 1.0;
        if sd.apu && self.sw.apu_master && apu_fuel && (self.apu > 0.0 || (self.apu_start && self.power >= Power::Battery)) {
            self.apu = (self.apu + dt / 40.0).min(1.0);
        } else {
            self.apu = (self.apu - dt / 20.0).max(0.0);
            if !self.sw.apu_master {
                self.apu_start = false;
            }
        }
        let apu_gen = sd.apu && self.apu > 0.95;
        let gens = self.gen_on.iter().any(|g| *g) || apu_gen;
        // RAT: deploys automatically when all AC generation is lost in flight.
        if sd.rat && !gens && airborne && ias > 70.0 && n > 0 {
            if !self.rat_out {
                self.rat_out = true;
            }
        } else if !airborne && ias < 30.0 {
            self.rat_out = false;
        }
        let rat_power = self.rat_out && ias > 60.0;
        let epu = sd.epu_min > 0.0 && !gens && airborne && self.epu_s > 0.0;
        if epu {
            self.epu_s -= dt;
        }
        let batt_ok = self.sw.battery && self.soc > 0.0;
        self.power = if gens {
            Power::Normal
        } else if rat_power || epu {
            Power::Emergency
        } else if batt_ok {
            Power::Battery
        } else {
            Power::Off
        };
        let mut load = sd.load_a * if self.sw.avionics { 1.0 } else { 0.55 };
        if self.sw.pitot_heat && sd.pitot_heat {
            load += sd.pitot_heat_a;
        }
        if self.power == Power::Battery && sd.glass {
            // Automatic load shedding to the essential / hot battery buses.
            load *= 0.4;
        }
        match self.power {
            Power::Normal => {
                self.volts = sd.volts;
                let charge = if self.sw.battery { ((1.0 - self.soc) * sd.batt_ah * 2.0).clamp(0.0, 0.3 * sd.batt_ah) + 1.5 } else { 0.0 };
                self.amps = charge;
                if self.sw.battery {
                    self.soc = (self.soc + (charge - 1.5).max(0.0) * dt / 3600.0 / sd.batt_ah.max(0.1)).min(1.0);
                }
            }
            Power::Emergency => {
                self.volts = sd.volts * 0.96;
                self.amps = 0.0;
            }
            Power::Battery => {
                self.soc = (self.soc - load * dt / 3600.0 / sd.batt_ah.max(0.1)).max(0.0);
                self.volts = sd.batt_v * (0.84 + 0.22 * self.soc);
                self.amps = -load;
                if self.soc <= 0.0 {
                    self.power = Power::Off;
                    self.volts = 0.0;
                    self.amps = 0.0;
                }
            }
            Power::Off => {
                self.volts = 0.0;
                self.amps = 0.0;
            }
        }
        let bus = self.bus_ok();

        // ---------------- fuel feed and engines
        let fuel_out = self.engine_fuel(def, ac, alt, dt);
        for i in 0..n {
            self.engine_step(def, ac, ctl, i, fuel_out[i], bus, ias, airborne, oat, amb, dt);
        }
        // Engine fire destroys an unattended engine.
        for i in 0..n {
            if self.eng[i].fire > 0.6 && self.eng[i].fire_t > 150.0 {
                if n > 1 {
                    let sdd = crate::damage::struct_def(def.id);
                    let part = crate::damage::Part::engine(i);
                    if !ac.dmg.is_lost(part) {
                        ac.dmg.separate(sdd, def, part, "engine fire");
                    }
                }
                let e = &mut self.eng[i];
                e.dead = true;
                e.run = false;
                e.fire = 0.0;
                self.fail.set(Failure::new(Kind::EngineFire, i), false);
                self.note(if n > 1 { "The burning engine has torn away" } else { "The engine has seized — the fire is out" });
            }
        }

        // ---------------- hydraulics
        for k in 0..sd.hyd.len().min(3) {
            let h = sd.hyd[k];
            let lost = self.has(Kind::Hydraulic, k);
            let eng_pump = h.pumps.iter().any(|p| {
                let e = SysDef::pump_engine(*p, n);
                e < n && self.eng[e].run && !self.sw.fire_handle[e]
            });
            let elec = h.elec && self.sw.hyd_elec && self.power >= Power::Normal;
            // Helicopters: the pump is on the main gearbox (works in autorotation).
            let eng_pump = eng_pump || (def.category == crate::flight::Category::Helicopter && ac.rotor.out.nr_pct > 50.0);
            let rat = h.rat && self.rat_out && ias > 60.0;
            let mut target = if lost {
                0.0
            } else if eng_pump || elec || rat {
                1.0
            } else {
                0.0
            };
            if sd.ptu && target == 0.0 && !lost && sd.hyd.len() >= 2 && (k == 0 || k == sd.hyd.len() - 1) {
                let other = if k == 0 { sd.hyd.len() - 1 } else { 0 };
                if self.hyd[other] > 0.8 && !self.has(Kind::Hydraulic, other) {
                    target = 0.9;
                }
            }
            self.hyd[k] += (target - self.hyd[k]) * (dt / 1.5).min(1.0);
        }
        // Brake accumulator: applications without hydraulic pressure.
        let held = ctl.brakes > 0.4;
        if held && !self.brake_held && sd.brake_hyd != 0 && !self.hyd_any(sd.brake_hyd | sd.brake_alt_hyd) {
            self.brake_accu = (self.brake_accu - 1.0).max(0.0);
        }
        self.brake_held = held;

        // ---------------- instruments
        let vac = self.vacuum_ok();
        if sd.vacuum {
            self.spin = if vac { (self.spin + dt / 25.0).min(1.0) } else { self.spin * (-dt / 110.0).exp() };
        }
        if sd.elec_turn {
            self.tc_spin = if bus { (self.tc_spin + dt / 8.0).min(1.0) } else { self.tc_spin * (-dt / 25.0).exp() };
        }
        self.gyros(ac, dt);
        // Pitot icing with the heat off (or unpowered) in visible moisture below freezing.
        let icy = amb.moisture > 0.15 && oat < 2.0 && oat > -30.0 && self.natural_icing && !self.protect;
        let heat = sd.pitot_heat && self.sw.pitot_heat && bus;
        if icy && !heat {
            self.pitot_ice = (self.pitot_ice + dt / 70.0 * amb.moisture.min(1.0)).min(1.0);
        } else if heat || oat > 3.0 {
            self.pitot_ice = (self.pitot_ice - dt / 25.0).max(0.0);
        }
        let blocked = self.has(Kind::PitotBlock, 0) || self.pitot_ice >= 1.0;
        if blocked && self.pitot_p.is_none() {
            self.pitot_p = Some(crate::flight::isa(alt).pressure + qc_from_cas(ias));
        } else if !blocked && self.pitot_ice < 0.5 {
            self.pitot_p = None;
        }

        // ---------------- airframe ice, smoke
        let forced = self.has(Kind::Icing, 0);
        let wing_prot = sd.wing_anti_ice && self.sw.wing_anti_ice && self.power >= Power::Emergency;
        let accrete = if forced {
            1.0 / 200.0
        } else if icy && oat > -20.0 {
            amb.moisture.min(1.0) / 700.0
        } else {
            0.0
        };
        if wing_prot {
            self.ice = (self.ice - dt / 45.0).max(0.0);
        } else if accrete > 0.0 {
            self.ice = (self.ice + accrete * dt).min(1.0);
        } else if oat > 3.0 {
            self.ice = (self.ice - dt / 150.0).max(0.0);
        } else {
            self.ice = (self.ice - dt / 1500.0).max(0.0);
        }
        if self.has(Kind::ElecFire, 0) {
            if self.sw.avionics && self.power != Power::Off {
                self.smoke = (self.smoke + dt / 80.0).min(1.0);
                self.smoke_iso_t = 0.0;
            } else {
                self.smoke_iso_t += dt;
                if self.smoke_iso_t > 25.0 {
                    self.fail.set(Failure::new(Kind::ElecFire, 0), false);
                    self.note("Smoke source isolated — the fire is out");
                }
            }
        } else {
            self.smoke = (self.smoke - dt / 45.0).max(0.0);
        }
        // Carburettor heat / prop overspeed / control jams are in the effects.
        self.fx = self.effects(def, Some(ac));
    }

    /// Which tanks feed engine `i` (bit mask).
    fn feed_tanks(&self, i: usize) -> u8 {
        let nt = self.sd.tanks.len().min(3);
        let has = |k: usize| k < nt && self.tank[k] > 0.05;
        match self.sd.feed {
            Feed::Selector => match self.sw.selector {
                Selector::Off => 0,
                Selector::Left => u8::from(has(0)),
                Selector::Right => u8::from(has(1)) << 1,
                Selector::Both => (u8::from(has(0))) | (u8::from(has(1)) << 1),
            },
            Feed::Common => (0..nt).filter(|k| has(*k)).fold(0, |m, k| m | (1 << k)),
            Feed::Wing => {
                if nt >= 3 && has(2) && self.sw.pumps {
                    return 1 << 2;
                }
                let own = if self.n_eng <= 1 || i < self.n_eng / 2 { 0 } else { 1 };
                let other = 1 - own;
                let mut m = 0;
                if has(own) {
                    m |= 1 << own;
                }
                if self.sw.crossfeed && has(other) {
                    m |= 1 << other;
                }
                m
            }
        }
    }

    /// Burn fuel; returns whether each engine is getting fuel.
    fn engine_fuel(&mut self, def: &AircraftDef, ac: &Aircraft, alt: f32, dt: f32) -> [bool; ME] {
        let mut ok = [false; ME];
        if def.engine_kind == EngineKind::Electric {
            // Battery-electric (drones): the pack is modelled with the motors (src/rotor/quad.rs).
            return [true; ME];
        }
        let total0 = self.fuel_total();
        for i in 0..self.n_eng {
            let mask = self.feed_tanks(i);
            let pump_fail = self.has(Kind::FuelPump, i);
            let vital = self.sd.engine_pump_vital && pump_fail;
            ok[i] = mask != 0 && !vital && !self.sw.fire_handle[i];
            let mut flow = ac.fuel_flow(i);
            if self.eng[i].starting && !self.eng[i].run && self.eng[i].start > 0.3 {
                // Light-off: start fuel flow before the engine is self-sustaining.
                flow = 30.0 * (def.mass / 20_000.0).clamp(0.2, 4.0);
            }
            let kg = flow * dt / 3600.0;
            let n = (0..3).filter(|k| mask & (1 << k) != 0).count();
            if n > 0 && kg > 0.0 {
                // Crossfeed: draw from the fuller side.
                let ks: Vec<usize> = (0..3).filter(|k| mask & (1 << k) != 0).collect();
                if self.sd.feed == Feed::Wing && ks.len() == 2 {
                    let k = if self.tank[ks[0]] >= self.tank[ks[1]] { ks[0] } else { ks[1] };
                    self.tank[k] = (self.tank[k] - kg).max(0.0);
                } else {
                    let tot: f32 = ks.iter().map(|k| self.tank[*k]).sum::<f32>().max(1e-6);
                    if self.sd.feed == Feed::Selector {
                        for k in &ks {
                            self.tank[*k] = (self.tank[*k] - kg / ks.len() as f32).max(0.0);
                        }
                    } else {
                        for k in &ks {
                            self.tank[*k] = (self.tank[*k] - kg * self.tank[*k] / tot).max(0.0);
                        }
                    }
                }
            }
            // Boost pump failed: suction feed. Fine low down; at altitude
            // the fuel boils / cavitates: surges, then flameout.
            if pump_fail && !self.sd.engine_pump_vital && alt <= 4_500.0 && mask != 0 {
                self.eng[i].starve = 0.0;
            } else if pump_fail && !self.sd.engine_pump_vital && alt > 4_500.0 {
                let lim = if alt > 7_500.0 { 20.0 } else { 90.0 };
                if self.eng[i].run {
                    self.eng[i].starve += dt * 0.5;
                    if self.eng[i].starve > lim {
                        ok[i] = false;
                    }
                }
            }
        }
        // APU burn.
        if self.apu > 0.3 && self.tank[0] > 0.0 {
            let k = if self.sd.tanks.len() >= 3 && self.tank[2] > 0.0 { 2 } else { 0 };
            self.tank[k] = (self.tank[k] - 120.0 * dt / 3600.0).max(0.0);
        }
        // Leaks: ~40 % of the tank's capacity an hour.
        for k in 0..self.sd.tanks.len().min(3) {
            if self.has(Kind::FuelLeak, k) {
                self.tank[k] = (self.tank[k] - self.tank_cap[k] * 0.4 * dt / 3600.0).max(0.0);
            }
        }
        // Auto flight: the bot never runs dry (it would land and refuel).
        if self.fuel_floor > 0.0 && self.fuel0 > 0.0 {
            let floor = self.fuel0 * self.fuel_floor;
            let total = self.fuel_total();
            if total < floor && total < total0 {
                let add = (total0 - total).min(floor - total);
                for k in 0..self.sd.tanks.len().min(3) {
                    self.tank[k] += add * self.sd.tanks[k].1;
                }
            }
        }
        ok
    }

    #[allow(clippy::too_many_arguments)]
    fn engine_step(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &Controls, i: usize, fuel: bool, bus: bool, ias: f32, airborne: bool, oat: f32, amb: &Ambient, dt: f32) {
        let sd = self.sd;
        let piston = def.engine_kind == EngineKind::Piston;
        let lever = self.sw.eng_master[i] && !self.sw.fire_handle[i];
        let mags = !piston || self.sw.mags[i] != Mags::Off;
        // Fuel at the engine: a few seconds in the lines.
        let suction = self.has_pump_fail_alt(i);
        let e = &mut self.eng[i];
        if fuel {
            if e.starve > 0.0 && !suction {
                e.starve = 0.0;
            }
        } else {
            e.starve += dt;
        }
        let e = &mut self.eng[i];
        e.fed = fuel || e.starve < 3.0;
        e.flameout_t = (e.flameout_t - dt).max(0.0);
        let cause = e.flameout_t > 0.0;
        if e.run && (!e.fed || !lever || !mags || e.dead || cause) {
            e.run = false;
            e.start = 0.0;
            e.starting = false;
        }
        // Carburettor ice.
        if sd.carburettor && piston {
            let forced = self.fail.has(Failure::new(Kind::CarbIce, 0));
            let humid = (1.0 - amb.spread_c / 12.0).clamp(0.0, 1.0).max(amb.moisture.min(1.0));
            let in_range = oat > -8.0 && oat < 28.0;
            let low_power = if ctl.throttle < 0.45 { 2.0 } else { 1.0 };
            let natural = if in_range && humid > 0.55 && self.natural_icing && !self.protect { (humid - 0.55) * low_power / 600.0 } else { 0.0 };
            let rate = if forced { 1.0 / 150.0 } else { natural };
            let e = &mut self.eng[i];
            if self.sw.carb_heat && e.run {
                e.carb_ice = (e.carb_ice - dt / 25.0).max(0.0);
            } else if e.run || forced {
                e.carb_ice = (e.carb_ice + rate * dt).min(1.0);
            }
            if e.carb_ice > 0.97 && e.run {
                e.run = false;
                e.start = 0.0;
            }
        }
        let carb_block = self.eng[i].carb_ice > 0.85;
        let e = &mut self.eng[i];
        if !e.run {
            match sd.starter {
                Starter::Piston => {
                    // Electric starter (needs the battery), magnetos, primed or warm.
                    let cranking = e.crank > 0.0 && self.power >= Power::Battery && self.volts > 0.8 * sd.batt_v;
                    if e.crank > 0.0 {
                        e.crank -= dt;
                    }
                    let windmill = airborne && ias > 28.0;
                    if (cranking || windmill) && mags && lever && e.fed && !e.dead && !cause && !carb_block {
                        e.start += dt;
                        // A cold engine needs priming; overpriming floods it.
                        let need = if e.primer >= 1.0 || oat > 25.0 { 1.2 } else { 3.5 };
                        let flooded = e.primer > 6.0;
                        if e.start > need && !flooded {
                            e.run = true;
                            e.start = 0.0;
                            e.crank = 0.0;
                            e.primer = 0.0;
                        } else if flooded && e.start > 2.0 {
                            e.primer -= dt * 2.0;
                        }
                    } else if !cranking {
                        e.start = 0.0;
                    }
                }
                Starter::None => {}
                st => {
                    let air = match st {
                        Starter::JetApu => (self.sw.apu_bleed && self.apu > 0.95) || (0..self.n_eng).any(|k| k != i && self.eng[k].run),
                        Starter::TurbopropBattery => self.power >= Power::Battery && self.volts > 0.85 * sd.batt_v,
                        Starter::Jfs => self.power >= Power::Battery,
                        _ => false,
                    };
                    let e = &mut self.eng[i];
                    let windmill = airborne && ias > 125.0;
                    let want = lever && (e.starting || (airborne && self.sw.ign_start) || (airborne && windmill));
                    if want && (air || windmill) && e.fed && !e.dead && !cause {
                        let t = if windmill && !air { 35.0 } else { 40.0 };
                        e.start = (e.start + dt / t).min(1.0);
                        if e.start >= 1.0 {
                            e.run = true;
                            e.start = 0.0;
                            e.starting = false;
                        }
                    } else if !want || !(air || windmill) {
                        e.start = (e.start - dt / 8.0).max(0.0);
                        if !lever && !airborne {
                            e.starting = false;
                        }
                    }
                }
            }
        } else {
            e.start = 0.0;
            e.crank = 0.0;
        }
        // Fire: fed by fuel until it is cut off; agents; it burns out without fuel.
        let fire_fail = self.fail.has(Failure::new(Kind::EngineFire, i));
        let e = &mut self.eng[i];
        if e.fire > 0.0 || fire_fail {
            if e.fire <= 0.0 && fire_fail {
                // Knocked down by an agent with fuel still flowing: it comes back.
                e.agent_t += dt;
                if lever && e.agent_t > 20.0 {
                    e.fire = 0.2;
                }
            }
            if e.fire > 0.0 {
                e.fire_t += dt;
                if lever && e.fed {
                    e.fire = (e.fire + dt / 25.0).min(1.0);
                } else {
                    e.fire = (e.fire - dt / 70.0).max(0.0);
                }
                if e.fire <= 0.0 {
                    e.fire_t = 0.0;
                }
            }
            if e.fire <= 0.0 && (!lever || e.agent > 0) {
                // Out for good once the fuel is off.
                if !lever {
                    self.fail.set(Failure::new(Kind::EngineFire, i), false);
                    self.eng[i].dead = true;
                    self.note("Fire out");
                }
            }
        }
        let e = &mut self.eng[i];
        e.agent_t += dt;
        // Oil: a fire cooks it, a dead or starved engine loses pressure.
        let t_add = 70.0 * e.fire + 25.0 * e.bird + 20.0 * e.overspeed;
        e.oil_t_add += (t_add - e.oil_t_add) * (dt / 20.0).min(1.0);
        let p_mul = if e.dead { 0.0 } else { 1.0 - 0.3 * e.bird };
        e.oil_p_mul += (p_mul - e.oil_p_mul) * (dt / 3.0).min(1.0);
        // Governor failure: overspeed builds in a couple of seconds.
        let os = if self.fail.has(Failure::new(Kind::PropOverspeed, i)) && e.run { 1.0 } else { 0.0 };
        e.overspeed += (os - e.overspeed) * (dt / 2.0).min(1.0);
        let _ = (ac, bus);
    }

    fn has_pump_fail_alt(&self, i: usize) -> bool {
        self.has(Kind::FuelPump, i) && !self.sd.engine_pump_vital
    }

    /// Vacuum / electric gyros: rotor speed decides how well they follow.
    fn gyros(&mut self, ac: &Aircraft, dt: f32) {
        let (tp, tr, th) = (ac.pitch(), ac.roll(), ac.heading());
        let tumble = self.fail.any_of(Kind::AttitudeInd) && !self.sd.glass;
        let dg_fail = self.fail.any_of(Kind::HeadingInd) && !self.sd.glass;
        let spin = if self.sd.vacuum { self.spin } else { 1.0 };
        let (dp, dr) = self.drift_dir;
        if spin >= 0.999 && !tumble {
            self.ai = None;
        } else {
            let (mut p, mut r) = self.ai.unwrap_or((tp, tr));
            let k = if tumble { 0.0 } else { 8.0 * spin.powi(6) };
            let drift = 0.05 * (1.0 - spin).powi(2) + if tumble { 0.35 } else { 0.0 };
            p += (tp - p) * (k * dt).min(1.0) + dp * 0.35 * drift * dt;
            r += crate::bot::wrap_pi(tr - r) * (k * dt).min(1.0) + dr * drift * dt;
            p = p.clamp(-1.2, 1.2);
            r = crate::bot::wrap_pi(r);
            self.ai = Some((p, r));
        }
        if spin >= 0.999 && !dg_fail {
            self.dg = None;
        } else {
            let mut h = self.dg.unwrap_or(th);
            let k = if dg_fail { 0.0 } else { 8.0 * spin.powi(6) };
            let drift = 0.004 * (1.0 - spin).powi(2) + if dg_fail { 0.0045 } else { 0.0 };
            h += crate::bot::wrap_pi(th - h) * (k * dt).min(1.0) + dp * drift * dt;
            // A failed DG still turns roughly with the aircraft (precession error builds).
            if dg_fail {
                h += crate::bot::wrap_pi(th - h) * (0.9 * dt).min(1.0);
            }
            self.dg = Some(h.rem_euclid(std::f32::consts::TAU));
        }
    }

    /// The indicated air data and attitude (truth when everything works).
    pub fn air_data(&self, ac: &Aircraft) -> AirData {
        let mut d =
            AirData { ias: ac.airspeed(), alt: ac.pos.y, vs: ac.vertical_speed(), pitch: ac.pitch(), roll: ac.roll(), heading: ac.heading(), turn: ac.turn_rate(), flags: 0 };
        if !self.init {
            return d;
        }
        if self.pitot_p.is_some() || self.static_p.is_some() {
            let p_s_true = crate::flight::isa(ac.pos.y).pressure;
            let p_t_true = p_s_true + qc_from_cas(d.ias.max(0.0));
            let p_t = self.pitot_p.unwrap_or(p_t_true);
            let (p_s, alt_s) = self.static_p.unwrap_or((p_s_true, ac.pos.y));
            d.ias = cas_from_qc(p_t - p_s);
            if self.static_p.is_some() {
                d.alt = alt_s;
                d.vs = 0.0;
            }
        }
        if let Some((p, r)) = self.ai {
            d.pitch = p;
            d.roll = r;
        }
        if let Some(h) = self.dg {
            d.heading = h;
        }
        if self.sd.elec_turn {
            d.turn *= self.tc_spin;
            if self.tc_spin < 0.6 {
                d.flags |= flag::TURN;
            }
        }
        if self.sd.glass {
            if self.power == Power::Off {
                d.flags |= flag::SPD | flag::ALT | flag::VS | flag::ATT | flag::HDG;
            }
            if self.fail.any_of(Kind::AttitudeInd) {
                d.flags |= flag::ATT | flag::HDG;
            }
            if self.fail.any_of(Kind::HeadingInd) {
                d.flags |= flag::HDG;
            }
        }
        d
    }

    /// Effects on the flight model.
    fn effects(&self, def: &AircraftDef, ac: Option<&Aircraft>) -> Effects {
        let sd = self.sd;
        let mut fx = Effects::default();
        if !self.init {
            return fx;
        }
        for i in 0..self.n_eng {
            let e = &self.eng[i];
            if !e.run {
                fx.engine_cut |= 1 << i;
                // Starter / windmill-assisted start: the core turns.
                let c: f32 = if e.crank > 0.0 && self.power >= Power::Battery { 0.55 } else { 0.0 };
                fx.crank[i] = c.max(e.start * 0.95);
                if e.start > 0.3 && def.engine_kind != EngineKind::Piston {
                    fx.lit |= 1 << i;
                }
            }
            let mut p = 1.0f32;
            if self.fail.has(Failure::new(Kind::PartialPower, i)) {
                p *= 0.5;
            }
            if e.bird > 0.0 {
                p *= 1.0 - e.bird;
            }
            if e.carb_ice > 0.0 {
                p *= 1.0 - 0.75 * e.carb_ice.powf(1.5);
            }
            if sd.carburettor && self.sw.carb_heat {
                p *= 0.9;
            }
            if e.fire > 0.5 && e.fire_t > 30.0 {
                p *= 0.75;
            }
            if e.overspeed > 0.0 {
                p *= 1.0 - 0.15 * e.overspeed;
                fx.rpm_over[i] = 0.2 * e.overspeed;
            }
            // Suction feed at altitude: the engine surges.
            if self.has(Kind::FuelPump, i) && !sd.engine_pump_vital && e.starve > 0.0 {
                p *= 0.85 + 0.1 * (self.time * 2.3 + i as f32).sin();
            }
            fx.power[i] = p.clamp(0.0, 1.0);
            fx.egt_add[i] = 120.0 * e.fire + 70.0 * e.bird;
        }
        // Mass and lateral CG.
        let fuel = self.fuel_total();
        fx.mass_delta = (fuel - self.fuel0) + self.ice * def.mass * 0.006;
        let total_mass = (def.mass + fx.mass_delta).max(1.0);
        let mut m_y = 0.0;
        for (k, t) in sd.tanks.iter().enumerate().take(3) {
            m_y += self.tank[k] * t.2;
        }
        let cg_x = m_y / total_mass;
        if cg_x.abs() > 0.002 {
            fx.cg = Vec3::new(cg_x, 0.0, 0.0);
        }
        // Gear: hydraulics (or the uplocks) and the alternate extension.
        if def.retractable_gear {
            let normal = !self.fail.any_of(Kind::GearExtend) && (sd.gear_hyd == 0 || self.hyd_any(sd.gear_hyd));
            if self.sw.alt_gear {
                fx.gear_down = def.gear_time.max(0.5) / sd.alt_gear_s.max(def.gear_time).max(1.0);
                fx.gear_up = 0.0;
            } else if !normal {
                fx.gear_down = 0.0;
                fx.gear_up = if sd.gear_hyd == 0 || self.hyd_any(sd.gear_hyd) { 1.0 } else { 0.0 };
            }
        }
        // Flaps.
        if sd.flaps_electric {
            if !self.bus_ok() {
                fx.flap_rate = 0.0;
            }
        } else if sd.flap_hyd != 0 {
            fx.flap_rate = self.hyd_frac(sd.flap_hyd);
        }
        if let Some(lock) = self.flap_lock {
            let asym = self.fail.any_of(Kind::FlapAsym);
            if self.fail.any_of(Kind::FlapJam) || (asym && (sd.glass || def.category == crate::flight::Category::Airliner)) {
                // Airliners: the wing-tip brakes lock both sides on asymmetry.
                fx.flap_hold = Some(lock);
            } else if asym {
                let now = ac.map_or(lock, |a| a.surfaces().flaps);
                fx.flap_asym = now - lock;
            }
        }
        if sd.spoiler_hyd != 0 {
            fx.spoilers = self.hyd_frac(sd.spoiler_hyd);
        }
        // Powered flight controls: lose every system (or the FBW computers' power) → frozen.
        let fcs_dead = (sd.fcs_hyd != 0 && !self.hyd_any(sd.fcs_hyd)) || (def.control_law != crate::flight::ControlLaw::Direct && self.power == Power::Off && self.n_eng > 0);
        if fcs_dead {
            fx.surface_rate = [0.0; 3];
        }
        fx.jam = self.jam;
        // Brakes.
        fx.brakes = if self.fail.any_of(Kind::Brakes) {
            0.0
        } else if sd.brake_hyd == 0 || self.hyd_any(sd.brake_hyd) {
            1.0
        } else if sd.brake_alt_hyd != 0 && self.hyd_any(sd.brake_alt_hyd) {
            0.75
        } else if self.brake_accu > 0.0 {
            0.8
        } else {
            0.0
        };
        if sd.nws_hyd != 0 && !self.hyd_any(sd.nws_hyd) {
            fx.steering = 0.0;
        }
        fx.tyre = [self.has(Kind::TyreBurst, 0), self.has(Kind::TyreBurst, 1)];
        fx.ice = self.ice;
        fx
    }

    /// The automatic start: one switch / action at a time, like a checklist.
    fn run_autostart(&mut self, dt: f32) {
        if self.autostart == 0 {
            return;
        }
        self.auto_t += dt;
        let wait = |t: &mut f32, s: f32| -> bool {
            if *t >= s {
                *t = 0.0;
                true
            } else {
                false
            }
        };
        let n = self.n_eng;
        let sd = self.sd;
        match self.autostart {
            1 => {
                if wait(&mut self.auto_t, 0.8) {
                    self.sw.battery = true;
                    self.sw.gens = [true; ME];
                    self.sw.fire_handle = [false; ME];
                    self.autostart = 2;
                }
            }
            2 => {
                if wait(&mut self.auto_t, 0.8) {
                    self.sw.pumps = true;
                    self.sw.selector = Selector::Both;
                    self.sw.hyd_elec = true;
                    self.sw.mags = [Mags::Both; ME];
                    for i in 0..n {
                        self.sw.eng_master[i] = sd.starter == Starter::Piston;
                        if sd.starter == Starter::Piston {
                            self.eng[i].primer = 3.0;
                        }
                    }
                    if sd.apu {
                        self.sw.apu_master = true;
                        self.apu_start = true;
                    }
                    self.autostart = 3;
                }
            }
            3 => {
                // Jets: wait for the APU, then bleed on.
                if !sd.apu || self.apu > 0.95 {
                    self.sw.apu_bleed = sd.apu;
                    self.sw.ign_start = sd.starter != Starter::Piston;
                    self.autostart = 4;
                    self.auto_t = 0.0;
                }
            }
            s if (4..4 + ME as u8).contains(&s) => {
                let i = (s - 4) as usize;
                if i >= n {
                    self.autostart = 20;
                    return;
                }
                let e = self.eng[i];
                if e.run {
                    self.autostart += 1;
                    self.auto_t = 0.0;
                } else if e.dead {
                    self.autostart += 1;
                } else if sd.starter == Starter::Piston {
                    if e.crank <= 0.0 && self.auto_t > 0.5 {
                        self.press_start(i);
                        self.auto_t = 0.0;
                    }
                } else if !e.starting {
                    self.sw.eng_master[i] = true;
                    self.eng[i].starting = true;
                }
                if self.auto_t > 90.0 {
                    self.autostart += 1;
                }
            }
            _ => {
                self.sw.ign_start = false;
                self.sw.avionics = true;
                self.sw.pitot_heat = true;
                if sd.apu && n > 0 && (0..n).all(|i| self.eng[i].run) {
                    self.sw.apu_bleed = false;
                    self.sw.apu_master = false;
                }
                self.autostart = 0;
            }
        }
    }

    /// Acknowledge the master warning / caution (silences the aural).
    pub fn acknowledge(&mut self) {
        self.master_warn = false;
        self.master_caut = false;
        self.aural_loop = Aural::None;
    }

    /// Update the master lights / aural from the current alerts (once per frame).
    pub fn update_alerts(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &Controls) {
        if !self.init || ac.crashed.is_some() {
            self.aural_loop = Aural::None;
            return;
        }
        let list = alerts::alerts(self, def, ac, ctl);
        let mut ids = [0u32; 25];
        for (k, a) in list.iter().take(24).enumerate() {
            ids[k] = a.id;
        }
        let style = self.sd.alerting;
        for a in &list {
            if a.level == alerts::Level::Advisory || self.known.contains(&a.id) {
                continue;
            }
            // A new alert.
            match a.level {
                alerts::Level::Warning => {
                    if style != Alerting::Annunciators {
                        self.master_warn = true;
                    }
                    let (one, looped) = match (style, a.fire) {
                        (Alerting::Ecam, _) | (Alerting::Atr, _) => (Aural::None, Aural::Crc),
                        (Alerting::Eicas, true) => (Aural::EngineFire, Aural::Bell),
                        (Alerting::Eicas, false) => (Aural::Warning, Aural::None),
                        (Alerting::Vms, _) => (Aural::Warning, Aural::None),
                        (Alerting::Annunciators, _) => (Aural::None, Aural::None),
                    };
                    let one = if a.smoke && style == Alerting::Eicas { Aural::Smoke } else { one };
                    if one != Aural::None {
                        self.aural = one;
                        self.aural_seq = self.aural_seq.wrapping_add(1);
                    }
                    if looped != Aural::None {
                        self.aural_loop = looped;
                    }
                }
                alerts::Level::Caution => {
                    if style != Alerting::Annunciators {
                        self.master_caut = true;
                        self.aural = if style == Alerting::Vms { Aural::Caution } else { Aural::Chime };
                        self.aural_seq = self.aural_seq.wrapping_add(1);
                    }
                }
                alerts::Level::Advisory => {}
            }
        }
        // The continuous aural stops when no warning is left.
        if !list.iter().any(|a| a.level == alerts::Level::Warning) {
            self.aural_loop = Aural::None;
            self.master_warn = false;
        }
        if !list.iter().any(|a| a.level == alerts::Level::Caution) {
            self.master_caut = false;
        }
        self.known = ids;
    }
}

impl Switches {
    /// The C172's alternator needs the master switch (field); airliners' don't.
    fn battery_field(&self, sd: SysDef) -> bool {
        sd.alerting != Alerting::Annunciators || self.battery
    }
}

/// Run `f` on the systems of `ac` with read access to the rest of the aircraft.
pub fn with_sys<R>(ac: &mut Aircraft, f: impl FnOnce(&mut Systems, &Aircraft) -> R) -> R {
    let mut sys = std::mem::take(&mut ac.sys);
    let r = f(&mut sys, ac);
    ac.sys = sys;
    r
}

/// Make failure `f` happen (or repair it) on `ac`.
pub fn set_failure(ac: &mut Aircraft, def: &AircraftDef, f: Failure, on: bool) {
    with_sys(ac, |s, a| s.set_failure(def, a, f, on));
}

/// The outside conditions at the aircraft from the weather.
pub fn ambient(world: &crate::world::World, ac: &Aircraft, ground_y: f32) -> Ambient {
    let wx = &world.wx;
    let agl = (ac.pos.y - ground_y).max(0.0);
    let xz = glam::Vec2::new(ac.pos.x, ac.pos.z);
    let c = &world.clouds;
    let in_cloud = if ac.pos.y > c.cu_base() - 50.0 && ac.pos.y < c.top() { c.coverage(xz) } else { 0.0 };
    let precip = if wx.precip != crate::weather::Precip::None { wx.intensity.clamp(0.2, 1.0) } else { 0.0 };
    Ambient { oat_c: Some(wx.temp_c - 0.0065 * agl), spread_c: (wx.temp_c - wx.dew_c).max(0.0), moisture: in_cloud.max(precip) }
}
