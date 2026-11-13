//! Instrument channels: the values the imported cockpit animations read
//! (`tools/fgimport` maps FlightGear property paths onto these names, see
//! `tools/fgimport/src/expr.rs`), with the dynamics of the real instruments —
//! pitot-static and gyro lag, the VSI's calibrated leak, the slip ball and the
//! magnetic compass card as damped oscillators, thermal lags on the engine
//! gauges, a total-energy variometer, fuel burn and the Hobbs meter.
//!
//! [`resolve`] turns a channel name into a [`Chan`] once at load time;
//! [`Instruments::step`] advances the instrument state each frame and
//! [`Instruments::value`] reads a channel.

use crate::flight::{Category, EngineKind, KT};
use crate::models::ModelInputs;
use std::f32::consts::PI;

const FT: f32 = 3.280_84;
const G: f32 = 9.806_65;
const MAX_ENG: usize = 4;

/// Engine quantities (per engine index).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EngQ {
    Rpm,
    N1,
    N2,
    EgtC,
    EgtF,
    EgtNorm,
    OilPPsi,
    OilTF,
    OilTC,
    ChtF,
    MapInhg,
    FfGph,
    FfPph,
    TqFtlb,
    TqPct,
    Nozzle,
}

/// Fuel tank quantities (per tank index).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TankQ {
    Gal,
    Lbs,
    Norm,
}

/// An instrument channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chan {
    IasKt,
    TasKt,
    GsKt,
    Mach,
    AltFt,
    PaltFt,
    AltSetInhg,
    AltSetHpa,
    VsFpm,
    VarioMps,
    PitchDeg,
    RollDeg,
    HeadingDeg,
    CompassDeg,
    HdgBugDeg,
    HdgBugErrDeg,
    TurnRate,
    Slip,
    GLoad,
    GMax,
    GMin,
    AlphaDeg,
    OatC,
    OatF,
    StallWarn,
    MasterCaution,
    MasterWarning,
    VmoKt,
    ClockS,
    ClockMin,
    ClockHour,
    Amps,
    SuctionInhg,
    PanelLight,
    FuelLbs,
    FuelGal,
    HobbsTenths,
    ApOn,
    ApAthr,
    ApAltFt,
    ApVsFpm,
    ApIasKt,
    FdPitchDeg,
    FdRollDeg,
    Eng(EngQ, u8),
    Tank(TankQ, u8),
}

/// The instrument channel called `name`, if it is one (other channels —
/// control surfaces, gear, lights — are computed in `models.rs`).
pub fn resolve(name: &str) -> Option<Chan> {
    use Chan::*;
    let c = match name {
        "ias_kt" => IasKt,
        "tas_kt" => TasKt,
        "gs_kt" => GsKt,
        "alt_ft" => AltFt,
        "palt_ft" => PaltFt,
        "alt_set_inhg" => AltSetInhg,
        "alt_set_hpa" => AltSetHpa,
        "vs_fpm" => VsFpm,
        "vario_mps" => VarioMps,
        "pitch_deg" => PitchDeg,
        "roll_deg" => RollDeg,
        "heading_deg" => HeadingDeg,
        "compass_deg" => CompassDeg,
        "hdg_bug_deg" => HdgBugDeg,
        "hdg_bug_err_deg" => HdgBugErrDeg,
        "turn_rate" => TurnRate,
        "slip" => Slip,
        "g_load" => GLoad,
        "g_max" => GMax,
        "g_min" => GMin,
        "oat_c" => OatC,
        "oat_f" => OatF,
        "stall_warn" => StallWarn,
        "master_caution" => MasterCaution,
        "master_warning" => MasterWarning,
        "vmo_kt" => VmoKt,
        "clock_s" => ClockS,
        "clock_min" => ClockMin,
        "clock_hour" => ClockHour,
        "amps" => Amps,
        "suction_inhg" => SuctionInhg,
        "panel_light" => PanelLight,
        "fuel_lbs" => FuelLbs,
        "fuel_gal" => FuelGal,
        "hobbs_tenths" => HobbsTenths,
        "ap_on" => ApOn,
        "ap_athr" => ApAthr,
        "ap_alt_ft" => ApAltFt,
        "ap_vs_fpm" => ApVsFpm,
        "ap_ias_kt" => ApIasKt,
        "fd_pitch_deg" => FdPitchDeg,
        "fd_roll_deg" => FdRollDeg,
        _ => {
            let idx = |pre: &str, suf: &str| -> Option<u8> { name.strip_prefix(pre)?.strip_suffix(suf)?.parse().ok() };
            for (suf, q) in [
                ("_egt_c", EngQ::EgtC),
                ("_egt_f", EngQ::EgtF),
                ("_egt_norm", EngQ::EgtNorm),
                ("_oilp_psi", EngQ::OilPPsi),
                ("_oilt_f", EngQ::OilTF),
                ("_oilt_c", EngQ::OilTC),
                ("_cht_f", EngQ::ChtF),
                ("_map_inhg", EngQ::MapInhg),
                ("_ff_gph", EngQ::FfGph),
                ("_ff_pph", EngQ::FfPph),
                ("_tq_ftlb", EngQ::TqFtlb),
                ("_tq_pct", EngQ::TqPct),
                ("_nozzle", EngQ::Nozzle),
                ("_rpm", EngQ::Rpm),
                ("_n1", EngQ::N1),
                ("_n2", EngQ::N2),
            ] {
                if let Some(i) = idx("eng", suf) {
                    return Some(Eng(q, i));
                }
            }
            for (suf, q) in [("_gal", TankQ::Gal), ("_lbs", TankQ::Lbs), ("_norm", TankQ::Norm)] {
                if let Some(i) = idx("tank", suf) {
                    return Some(Tank(q, i));
                }
            }
            return None;
        }
    };
    Some(c)
}

/// First-order lag towards `target` with time constant `tau` (s).
pub fn lag(x: f32, target: f32, tau: f32, dt: f32) -> f32 {
    if tau <= 0.0 || dt <= 0.0 {
        return if dt > 0.0 { target } else { x };
    }
    x + (target - x) * (1.0 - (-dt / tau).exp())
}

/// Damped second-order follower (position, velocity) → `target`
/// (natural frequency `w` rad/s, damping ratio `z`), sub-stepped for stability.
pub fn spring(s: (f32, f32), target: f32, w: f32, z: f32, dt: f32) -> (f32, f32) {
    let (mut x, mut v) = s;
    let n = ((dt * w * 4.0).ceil() as usize).clamp(1, 32);
    let h = dt / n as f32;
    for _ in 0..n {
        let a = w * w * (target - x) - 2.0 * z * w * v;
        v += a * h;
        x += v * h;
    }
    (x, v)
}

/// Wrap an angle in degrees to −180..180.
pub fn wrap180(d: f32) -> f32 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

/// Fuel on board at the start (kg), and the tank count the gauges split it over.
pub fn fuel_load(id: &str, cat: Category) -> (f32, usize) {
    match id {
        "kestrel" => (110.0, 2),
        "swift" => (0.0, 1),
        "peregrine" => (230.0, 2),
        "lancer" => (3_000.0, 2),
        "aerolink" => (3_500.0, 2),
        "meridian190" => (9_000.0, 3),
        "meridian320" => (12_000.0, 3),
        _ => match cat {
            Category::Glider => (0.0, 1),
            Category::Trainer => (110.0, 2),
            Category::Warbird => (230.0, 2),
            Category::FastJet => (3_000.0, 2),
            Category::Airliner => (8_000.0, 3),
        },
    }
}

/// Live instrument state (one per aircraft model).
#[derive(Clone, Debug, Default)]
pub struct Instruments {
    init: bool,
    ias: f32,
    alt_ft: f32,
    vs_fpm: f32,
    vario: f32,
    energy_h: Option<f32>,
    pitch: f32,
    roll: f32,
    /// Unwrapped heading-indicator reading, degrees.
    dg: f32,
    /// Compass card: (unwrapped angle, rate), degrees.
    compass: (f32, f32),
    turn: f32,
    ball: (f32, f32),
    g: f32,
    g_max: f32,
    g_min: f32,
    hdg_bug: f32,
    suction: f32,
    rpm: [f32; MAX_ENG],
    egt_c: [f32; MAX_ENG],
    oil_t_c: [f32; MAX_ENG],
    oil_p: [f32; MAX_ENG],
    cht_c: [f32; MAX_ENG],
    nozzle: [f32; MAX_ENG],
    fuel_kg: f32,
    fuel_cap: f32,
    tanks: usize,
    hobbs_h: f32,
}

/// Engine running steady-state targets: (oil °C, oil psi, CHT °C).
fn engine_targets(i: &ModelInputs, k: usize, rpm: f32) -> (f32, f32, f32) {
    let ac = i.ac;
    let oat = ac.oat();
    if !ac.engine_running(k) {
        return (oat, 0.0, oat);
    }
    let p = ac.n1(k).clamp(0.0, 1.1);
    match i.def.engine_kind {
        EngineKind::Piston => {
            let r = (rpm / 2000.0).clamp(0.0, 1.2);
            (oat + 60.0 + 25.0 * p, 22.0 + 50.0 * r.min(1.0), oat + 110.0 + 90.0 * p)
        }
        _ => (oat + 55.0 + 40.0 * p, 30.0 + 25.0 * p, oat),
    }
}

impl Instruments {
    /// Advance the instruments by `dt` seconds of sim time.
    pub fn step(&mut self, i: &ModelInputs, dt: f32) {
        let ac = i.ac;
        let def = i.def;
        let n_eng = ac.engine_count(def).clamp(1, MAX_ENG);
        let heading = ac.heading().to_degrees().rem_euclid(360.0);
        let alt_ft = ac.pos.y * FT;
        let ias = ac.airspeed_kt().max(0.0);
        let vs = ac.vertical_speed() / 0.00508;
        let energy_h = ac.pos.y + ac.tas() * ac.tas() / (2.0 * G);
        let ball = (ac.slip() * 4.0).clamp(-1.0, 1.0);
        let turn = (ac.turn_rate().to_degrees() / 3.0).clamp(-2.0, 2.0);
        let piston = def.engine_kind == EngineKind::Piston;
        if !self.init {
            self.init = true;
            self.ias = ias;
            self.alt_ft = alt_ft;
            self.vs_fpm = vs;
            self.pitch = ac.pitch().to_degrees();
            self.roll = ac.roll().to_degrees();
            self.dg = heading;
            self.compass = (heading, 0.0);
            self.turn = turn;
            self.ball = (ball, 0.0);
            self.g = ac.g_load();
            self.g_max = self.g.max(1.0);
            self.g_min = self.g.min(1.0);
            self.hdg_bug = (heading / 5.0).round() * 5.0;
            let (fuel, tanks) = fuel_load(def.id, def.category);
            self.fuel_kg = fuel;
            self.fuel_cap = fuel / 0.75;
            self.tanks = tanks;
            self.hobbs_h = 1_234.5;
            for k in 0..n_eng {
                self.rpm[k] = ac.engine_rpm_i(k);
                self.egt_c[k] = ac.egt(k);
                let (ot, op, ct) = engine_targets(i, k, self.rpm[k]);
                self.oil_t_c[k] = ot;
                self.oil_p[k] = op;
                self.cht_c[k] = ct;
                self.nozzle[k] = 0.8;
            }
            self.suction = if piston && ac.engine_running(0) { 5.0 } else { 0.0 };
        }
        if dt <= 0.0 {
            return;
        }
        // Air data: pitot-static and gyro lags.
        let steam = matches!(def.category, Category::Trainer | Category::Warbird | Category::Glider);
        self.ias = lag(self.ias, ias, 0.25, dt);
        self.alt_ft = lag(self.alt_ft, alt_ft, 0.35, dt);
        // Conventional VSI (calibrated leak) is slow; airliner IVSIs lead.
        self.vs_fpm = lag(self.vs_fpm, vs, if steam { 2.2 } else { 0.8 }, dt);
        // Total-energy variometer: rate of change of energy height.
        let te = match self.energy_h {
            Some(h) => (energy_h - h) / dt,
            None => ac.vertical_speed(),
        };
        self.energy_h = Some(energy_h);
        self.vario = lag(self.vario, te.clamp(-15.0, 15.0), 1.3, dt);
        self.pitch = lag(self.pitch, ac.pitch().to_degrees(), 0.08, dt);
        self.roll = wrap180(self.roll + lag(0.0, wrap180(ac.roll().to_degrees() - self.roll), 0.08, dt));
        self.dg += lag(0.0, wrap180(heading - self.dg), 0.12, dt);
        // Compass card (unwrapped angle, rate): lightly damped, swings and
        // overshoots after heading changes.
        let target = self.compass.0 + wrap180(heading - self.compass.0);
        self.compass = spring(self.compass, target, 2.4, 0.3, dt);
        self.turn = lag(self.turn, turn, 0.35, dt);
        self.ball = spring(self.ball, ball, 7.0, 0.55, dt);
        self.ball.0 = self.ball.0.clamp(-1.1, 1.1);
        self.g = lag(self.g, ac.g_load(), 0.06, dt);
        self.g_max = self.g_max.max(self.g);
        self.g_min = self.g_min.min(self.g);
        if let Some(h) = i.ap.and_then(|a| a.hdg) {
            self.hdg_bug = h.to_degrees().rem_euclid(360.0);
        }
        // Engine-driven vacuum pump.
        let vac = if piston && ac.engine_running(0) { (4.2 + ac.engine_rpm_i(0) / 2700.0).min(5.3) } else { 0.0 };
        self.suction = lag(self.suction, vac, 1.5, dt);
        let mut burn = 0.0;
        for k in 0..n_eng {
            self.rpm[k] = lag(self.rpm[k], ac.engine_rpm_i(k), 0.15, dt);
            self.egt_c[k] = lag(self.egt_c[k], ac.egt(k), 4.0, dt);
            let (ot, op, ct) = engine_targets(i, k, self.rpm[k]);
            self.oil_t_c[k] = lag(self.oil_t_c[k], ot, 90.0, dt);
            self.oil_p[k] = lag(self.oil_p[k], op, 0.8, dt);
            self.cht_c[k] = lag(self.cht_c[k], ct, 60.0, dt);
            let ab = ac.afterburner();
            let noz = if ab > 0.02 {
                0.35 + 0.65 * ab
            } else if ac.n1(k) < 0.75 {
                0.85
            } else {
                0.15
            };
            self.nozzle[k] = lag(self.nozzle[k], noz, 1.0, dt);
            burn += ac.fuel_flow(k);
        }
        self.fuel_kg = (self.fuel_kg - burn * dt / 3600.0).max(0.0);
        if def.has_engine && ac.engine_running(0) {
            self.hobbs_h += dt / 3600.0;
        }
    }

    /// The current value of channel `c`.
    pub fn value(&self, c: Chan, i: &ModelInputs) -> f32 {
        use Chan::*;
        let ac = i.ac;
        let def = i.def;
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        let n_eng = ac.engine_count(def).clamp(1, MAX_ENG);
        let heading = self.dg.rem_euclid(360.0);
        let ap = i.ap.filter(|a| a.engaged);
        match c {
            IasKt => self.ias,
            TasKt => ac.tas() / KT,
            GsKt => glam::Vec2::new(ac.vel.x, ac.vel.z).length() / KT,
            Mach => ac.mach(),
            AltFt => self.alt_ft,
            PaltFt => self.alt_ft + (1013.25 - i.qnh_hpa) * 27.0,
            AltSetInhg => i.qnh_hpa * 0.029_53,
            AltSetHpa => i.qnh_hpa,
            VsFpm => self.vs_fpm,
            VarioMps => self.vario,
            PitchDeg => self.pitch,
            RollDeg => self.roll,
            HeadingDeg => heading,
            CompassDeg => self.compass.0.rem_euclid(360.0),
            HdgBugDeg => self.hdg_bug,
            HdgBugErrDeg => wrap180(self.hdg_bug - heading),
            TurnRate => self.turn,
            Slip => self.ball.0,
            GLoad => self.g,
            GMax => self.g_max,
            GMin => self.g_min,
            AlphaDeg => ac.alpha().to_degrees(),
            OatC => ac.oat(),
            OatF => ac.oat() * 1.8 + 32.0,
            StallWarn => b(ac.stall_warning()),
            MasterWarning => b(ac.stall_warning() || ac.airspeed_kt() > def.vne_kt * 1.02),
            MasterCaution => b(def.has_engine && (0..n_eng).any(|k| !ac.engine_running(k))),
            VmoKt => def.vne_kt,
            ClockS => i.hours.rem_euclid(24.0) * 3600.0,
            ClockMin => i.hours.rem_euclid(24.0) * 60.0,
            ClockHour => i.hours.rem_euclid(24.0),
            Amps => {
                if def.has_engine && ac.engine_running(0) {
                    2.0 + 3.0 * (-i.time / 120.0).exp()
                } else {
                    -6.0
                }
            }
            SuctionInhg => self.suction,
            PanelLight => (1.0 - i.daylight).clamp(0.0, 1.0),
            FuelLbs => self.fuel_kg * 2.204_62,
            FuelGal => self.fuel_kg / fuel_density(def),
            HobbsTenths => self.hobbs_h * 10.0,
            ApOn => b(ap.is_some()),
            ApAthr => b(ap.is_some_and(|a| a.ias.is_some())),
            ApAltFt => i.ap.and_then(|a| a.alt).map(|m| m * FT).unwrap_or((self.alt_ft / 100.0).round() * 100.0),
            ApVsFpm => i.ap.and_then(|a| a.vs).map(|v| v / 0.00508).unwrap_or(0.0),
            ApIasKt => i.ap.and_then(|a| a.ias).map(|v| v / KT).unwrap_or(def.vno_kt),
            FdRollDeg => match ap.and_then(|a| a.hdg) {
                Some(h) => (wrap180((h.to_degrees() - ac.heading().to_degrees()) * 2.2).clamp(-25.0, 25.0) - ac.roll().to_degrees()).clamp(-30.0, 30.0),
                None => 0.0,
            },
            FdPitchDeg => match ap {
                Some(a) => {
                    let vs_t = match (a.alt, a.vs) {
                        (Some(t), _) => ((t - ac.pos.y) * 0.08).clamp(-10.0, 10.0),
                        (None, Some(v)) => v,
                        _ => ac.vertical_speed(),
                    };
                    ((vs_t - ac.vertical_speed()) / ac.tas().max(30.0)).atan().to_degrees().clamp(-8.0, 8.0)
                }
                None => 0.0,
            },
            Eng(q, k) => {
                let k = (k as usize).min(n_eng - 1);
                self.engine(q, k, i)
            }
            Tank(q, k) => {
                let k = k as usize;
                if k >= self.tanks.max(1) {
                    return 0.0;
                }
                let kg = self.fuel_kg / self.tanks.max(1) as f32;
                match q {
                    TankQ::Gal => kg / fuel_density(def),
                    TankQ::Lbs => kg * 2.204_62,
                    TankQ::Norm => {
                        if self.fuel_cap > 0.0 {
                            self.fuel_kg / self.fuel_cap
                        } else {
                            0.0
                        }
                    }
                }
            }
        }
    }

    fn engine(&self, q: EngQ, k: usize, i: &ModelInputs) -> f32 {
        let ac = i.ac;
        let def = i.def;
        let running = ac.engine_running(k);
        match q {
            EngQ::Rpm => self.rpm[k],
            EngQ::N1 => ac.n1(k) * 100.0,
            EngQ::N2 => (ac.n1(k) * 100.0 * 0.9 + 10.0).min(105.0),
            EngQ::EgtC => self.egt_c[k],
            EngQ::EgtF => self.egt_c[k] * 1.8 + 32.0,
            EngQ::EgtNorm => ((self.egt_c[k] * 1.8 + 32.0 - 1_100.0) / 600.0).clamp(0.0, 1.0),
            EngQ::OilPPsi => self.oil_p[k],
            EngQ::OilTF => self.oil_t_c[k] * 1.8 + 32.0,
            EngQ::OilTC => self.oil_t_c[k],
            EngQ::ChtF => self.cht_c[k] * 1.8 + 32.0,
            EngQ::MapInhg => {
                // Static ambient pressure, plus the throttle's share of it when running.
                let amb = 29.92 * (1.0 - 6.875_6e-6 * ac.pos.y * FT).max(0.1).powf(5.2559);
                if running && def.engine_kind == EngineKind::Piston { amb * (0.32 + 0.72 * i.ctl.throttle.clamp(0.0, 1.0)) } else { amb }
            }
            EngQ::FfGph => ac.fuel_flow(k) / fuel_density(def),
            EngQ::FfPph => ac.fuel_flow(k) * 2.204_62,
            EngQ::TqFtlb => {
                let w = self.rpm[k].max(1.0) * 2.0 * PI / 60.0;
                if running { ac.shaft_power(k) / w * 0.737_56 } else { 0.0 }
            }
            EngQ::TqPct => {
                let np = ac.engine_rpm_i(k).max(1.0);
                if running { (ac.shaft_power(k) / def.engine_power_w.max(1.0) * (def.rpm_static.max(1.0) / np) * 100.0).clamp(0.0, 150.0) } else { 0.0 }
            }
            EngQ::Nozzle => self.nozzle[k],
        }
    }
}

/// kg per US gallon of the aircraft's fuel (avgas / Jet A).
fn fuel_density(def: &crate::flight::AircraftDef) -> f32 {
    if def.engine_kind == EngineKind::Piston { 2.72 } else { 3.04 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lag_settles_and_is_dt_independent() {
        // One second in one step or in 120 steps gives the same answer.
        let a = lag(0.0, 1.0, 0.5, 1.0);
        let mut b = 0.0;
        for _ in 0..120 {
            b = lag(b, 1.0, 0.5, 1.0 / 120.0);
        }
        assert!((a - b).abs() < 1e-4, "{a} vs {b}");
        assert!((a - (1.0 - (-2.0f32).exp())).abs() < 1e-5);
        assert_eq!(lag(3.0, 5.0, 0.5, 0.0), 3.0);
    }

    #[test]
    fn spring_is_stable_and_converges() {
        let mut s = (0.0, 0.0);
        let mut peak = 0.0f32;
        for _ in 0..600 {
            s = spring(s, 1.0, 7.0, 0.55, 1.0 / 60.0);
            peak = peak.max(s.0);
        }
        assert!((s.0 - 1.0).abs() < 1e-3, "{s:?}");
        // Under-damped: a little overshoot (ζ 0.55 → ~12 %), not a wild one.
        assert!(peak > 1.02 && peak < 1.2, "peak {peak}");
        // A huge step stays stable.
        let s = spring((0.0, 0.0), 1.0, 7.0, 0.55, 0.25);
        assert!(s.0.is_finite() && s.0.abs() < 3.0);
    }

    #[test]
    fn wrap() {
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-190.0), 170.0);
        assert_eq!(wrap180(0.0), 0.0);
    }

    #[test]
    fn resolve_names() {
        assert_eq!(resolve("ias_kt"), Some(Chan::IasKt));
        assert_eq!(resolve("eng1_egt_f"), Some(Chan::Eng(EngQ::EgtF, 1)));
        assert_eq!(resolve("eng0_rpm"), Some(Chan::Eng(EngQ::Rpm, 0)));
        assert_eq!(resolve("eng12_n1"), Some(Chan::Eng(EngQ::N1, 12)));
        assert_eq!(resolve("tank2_gal"), Some(Chan::Tank(TankQ::Gal, 2)));
        assert_eq!(resolve("elevator"), None);
        assert_eq!(resolve("eng0_reverser"), None);
    }
}
