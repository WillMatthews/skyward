//! Per-type system definitions: what each aircraft has (electrics, fuel
//! tanks, hydraulic systems and who uses them, vacuum, carburettor, starter,
//! fire protection, anti-ice, the warning philosophy). Data, not code: a
//! type without its own row gets sensible defaults from its [`Category`] /
//! [`EngineKind`] / engine count, so new aircraft get working systems.

use crate::flight::{AircraftDef, Category, EngineKind};

/// "The last engine" in [`Hyd::pumps`] (so 2- and 4-engine types share a row).
pub const LAST: u8 = 255;

/// One hydraulic (or, on the Spitfire, pneumatic) system.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hyd {
    /// Display name ("G", "B", "Y", "1", "A"...).
    pub name: &'static str,
    /// Engine-driven pumps (engine index, or [`LAST`]).
    pub pumps: &'static [u8],
    /// Electric pump(s) (need the AC / main buses).
    pub elec: bool,
    /// Ram-air turbine pump (emergency).
    pub rat: bool,
}

/// How the engines get their fuel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Feed {
    /// Single-engine selector Off / Left / Right / Both (gravity feed).
    Selector,
    /// All tanks feed all engines (fuselage tanks).
    Common,
    /// Each side's wing tank feeds its engines, a centre tank feeds first;
    /// crossfeed valve.
    Wing,
}

/// Engine start system.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Starter {
    None,
    /// Magnetos, primer, electric starter.
    Piston,
    /// Starter-generator from the battery (no APU).
    TurbopropBattery,
    /// Air starter from the APU bleed.
    JetApu,
    /// Jet fuel starter (F-16).
    Jfs,
}

/// Alerting philosophy (master warning / caution, aural, message style).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alerting {
    /// Light aircraft: annunciator lights only, no aural (besides the stall horn).
    Annunciators,
    /// Airbus ECAM: CRC for warnings, single chime for cautions, procedures.
    Ecam,
    /// Embraer EICAS: fire bell, voice, caution chime.
    Eicas,
    /// ATR: CRC / single chime, ECAM-like messages on the EWD.
    Atr,
    /// F-16 voice message system ("Warning, warning" / "Caution").
    Vms,
}

#[derive(Clone, Copy, Debug)]
pub struct SysDef {
    // ---- electrics
    /// Bus voltage with the generators on line (nominal).
    pub volts: f32,
    /// Battery nominal voltage and capacity (A·h).
    pub batt_v: f32,
    pub batt_ah: f32,
    /// Normal bus load (A) with everything on, and the extra for pitot heat.
    pub load_a: f32,
    pub pitot_heat_a: f32,
    /// Engine-driven generators per engine (alternators, IDGs, starter-gens).
    pub gens_per_engine: u8,
    pub apu: bool,
    /// Ram-air turbine emergency generator (+ hydraulic pump if a system has `rat`).
    pub rat: bool,
    /// Emergency power unit (hydrazine, F-16): minutes of running.
    pub epu_min: f32,
    // ---- hydraulics (bit k = `hyd[k]`)
    pub hyd: &'static [Hyd],
    pub gear_hyd: u8,
    pub flap_hyd: u8,
    pub spoiler_hyd: u8,
    pub brake_hyd: u8,
    /// Alternate braking (no anti-skid) and accumulator applications.
    pub brake_alt_hyd: u8,
    pub brake_accu: u8,
    pub nws_hyd: u8,
    /// Flight controls powered by hydraulics (FBW / boosted): lose all → frozen.
    pub fcs_hyd: u8,
    /// Power-transfer unit between the first and last system.
    pub ptu: bool,
    /// Electric flaps (C172, E-Jets).
    pub flaps_electric: bool,
    /// Gear alternate extension time (s): free fall / blow-down / hand crank.
    pub alt_gear_s: f32,
    // ---- fuel
    /// Tanks: name, share of the fuel, lateral arm (m, + = left wing).
    pub tanks: &'static [(&'static str, f32, f32)],
    pub feed: Feed,
    /// Boost pumps in the tanks (else gravity / engine-driven).
    pub boost_pumps: bool,
    /// The fuel pump that can fail is engine-driven (the engine stops).
    pub engine_pump_vital: bool,
    // ---- engines
    pub starter: Starter,
    pub carburettor: bool,
    /// Engine-driven vacuum pump for the attitude / heading gyros.
    pub vacuum: bool,
    /// Electric turn coordinator / turn & slip.
    pub elec_turn: bool,
    /// Glass displays (PFD / ND / EWD) rather than steam gauges.
    pub glass: bool,
    pub const_speed_prop: bool,
    /// Fire detection and extinguisher bottles per engine (0 = none).
    pub fire_detect: bool,
    pub bottles: u8,
    // ---- ice protection
    pub pitot_heat: bool,
    pub wing_anti_ice: bool,
    pub eng_anti_ice: bool,
    pub alerting: Alerting,
}

const NO_HYD: &[Hyd] = &[];

impl Default for SysDef {
    fn default() -> Self {
        trainer()
    }
}

const A320_HYD: &[Hyd] =
    &[Hyd { name: "G", pumps: &[0], elec: false, rat: false }, Hyd { name: "B", pumps: &[], elec: true, rat: true }, Hyd { name: "Y", pumps: &[LAST], elec: true, rat: false }];
const E190_HYD: &[Hyd] =
    &[Hyd { name: "1", pumps: &[0], elec: true, rat: false }, Hyd { name: "2", pumps: &[LAST], elec: true, rat: false }, Hyd { name: "3", pumps: &[], elec: true, rat: true }];
const ATR_HYD: &[Hyd] = &[
    // Both pumps are AC-wild electric pumps fed by the engine-driven ACW generators.
    Hyd { name: "BLUE", pumps: &[0], elec: false, rat: false },
    Hyd { name: "GREEN", pumps: &[LAST], elec: false, rat: false },
];
const F16_HYD: &[Hyd] = &[Hyd { name: "A", pumps: &[0], elec: false, rat: false }, Hyd { name: "B", pumps: &[0], elec: false, rat: false }];
const SPIT_HYD: &[Hyd] = &[Hyd { name: "HYD", pumps: &[0], elec: false, rat: false }];
/// Helicopter flight-control servos: one system, pump on the main gearbox.
const HELI_HYD: &[Hyd] = &[Hyd { name: "HYD", pumps: &[], elec: false, rat: false }];
const HELI_TANK: &[(&str, f32, f32)] = &[("MAIN", 1.0, 0.0)];

/// Boeing 777: left / centre / right, engine-driven pumps on L and R, the
/// centre system on electric and air-driven pumps plus the RAT.
const B777_HYD: &[Hyd] =
    &[Hyd { name: "L", pumps: &[0], elec: true, rat: false }, Hyd { name: "C", pumps: &[], elec: true, rat: true }, Hyd { name: "R", pumps: &[LAST], elec: true, rat: false }];
/// CRJ700: systems 1 / 2 on the engine-driven pumps (+ AC backups), system 3 electric + the ADG.
const CRJ_HYD: &[Hyd] =
    &[Hyd { name: "1", pumps: &[0], elec: true, rat: false }, Hyd { name: "2", pumps: &[LAST], elec: true, rat: false }, Hyd { name: "3", pumps: &[], elec: true, rat: true }];
/// Citation X: systems A / B, one engine-driven pump each.
const C750_HYD: &[Hyd] = &[Hyd { name: "A", pumps: &[0], elec: false, rat: false }, Hyd { name: "B", pumps: &[LAST], elec: false, rat: false }];
/// Concorde: Green (engines 1 + 2), Blue (3 + 4), Yellow standby (2 + 4, RAT).
const CONCORDE_HYD: &[Hyd] = &[
    Hyd { name: "GREEN", pumps: &[0, 1], elec: false, rat: false },
    Hyd { name: "BLUE", pumps: &[2, LAST], elec: false, rat: false },
    Hyd { name: "YELLOW", pumps: &[1, LAST], elec: false, rat: true },
];
const T7_TANKS: &[(&str, f32, f32)] = &[("L", 0.36, 9.0), ("R", 0.36, -9.0), ("CTR", 0.28, 0.0)];
/// Concorde: wing / collector tanks, and the trim tanks fore and aft (the
/// trim-fuel transfer itself is the flight model's `cg_schedule`).
const CONCORDE_TANKS: &[(&str, f32, f32)] = &[("L", 0.38, 4.5), ("R", 0.38, -4.5), ("TRIM FWD", 0.08, 0.0), ("TRIM AFT", 0.16, 0.0)];

const WING3: &[(&str, f32, f32)] = &[("L", 0.4, 4.0), ("R", 0.4, -4.0), ("CTR", 0.2, 0.0)];
const WING2: &[(&str, f32, f32)] = &[("L", 0.5, 4.0), ("R", 0.5, -4.0)];
const C172_TANKS: &[(&str, f32, f32)] = &[("L", 0.5, 2.6), ("R", 0.5, -2.6)];
const FUS2: &[(&str, f32, f32)] = &[("UPPER", 0.56, 0.0), ("LOWER", 0.44, 0.0)];
const F16_TANKS: &[(&str, f32, f32)] = &[("FWD", 0.5, 0.0), ("AFT", 0.5, 0.0)];
const NO_TANKS: &[(&str, f32, f32)] = &[];

/// Light single (C172-class) defaults.
fn trainer() -> SysDef {
    SysDef {
        volts: 28.0,
        batt_v: 24.0,
        batt_ah: 13.0,
        load_a: 13.0,
        pitot_heat_a: 9.0,
        gens_per_engine: 1,
        apu: false,
        rat: false,
        epu_min: 0.0,
        hyd: NO_HYD,
        gear_hyd: 0,
        flap_hyd: 0,
        spoiler_hyd: 0,
        brake_hyd: 0,
        brake_alt_hyd: 0,
        brake_accu: 0,
        nws_hyd: 0,
        fcs_hyd: 0,
        ptu: false,
        flaps_electric: true,
        alt_gear_s: 0.0,
        tanks: C172_TANKS,
        feed: Feed::Selector,
        boost_pumps: false,
        engine_pump_vital: false,
        starter: Starter::Piston,
        carburettor: true,
        vacuum: true,
        elec_turn: true,
        glass: false,
        const_speed_prop: false,
        fire_detect: false,
        bottles: 0,
        pitot_heat: true,
        wing_anti_ice: false,
        eng_anti_ice: false,
        alerting: Alerting::Annunciators,
    }
}

fn glider() -> SysDef {
    SysDef {
        volts: 12.0,
        batt_v: 12.0,
        batt_ah: 7.0,
        load_a: 0.9,
        pitot_heat_a: 0.0,
        gens_per_engine: 0,
        flaps_electric: false,
        tanks: NO_TANKS,
        feed: Feed::Common,
        starter: Starter::None,
        carburettor: false,
        vacuum: false,
        elec_turn: false,
        pitot_heat: false,
        ..trainer()
    }
}

fn warbird() -> SysDef {
    SysDef {
        volts: 14.0,
        batt_v: 12.0,
        batt_ah: 25.0,
        load_a: 18.0,
        pitot_heat_a: 6.0,
        hyd: SPIT_HYD,
        gear_hyd: 1,
        // Pneumatic flaps and brakes from the engine-driven compressor (shown as the one system).
        flap_hyd: 1,
        brake_hyd: 1,
        brake_accu: 6,
        flaps_electric: false,
        alt_gear_s: 6.0,
        tanks: FUS2,
        feed: Feed::Common,
        engine_pump_vital: true,
        carburettor: false,
        elec_turn: false,
        const_speed_prop: true,
        ..trainer()
    }
}

fn fast_jet() -> SysDef {
    SysDef {
        volts: 28.0,
        batt_v: 24.0,
        batt_ah: 15.0,
        load_a: 40.0,
        pitot_heat_a: 5.0,
        gens_per_engine: 2,
        epu_min: 10.0,
        hyd: F16_HYD,
        gear_hyd: 0b10,
        flap_hyd: 0b11,
        spoiler_hyd: 0b01,
        brake_hyd: 0b10,
        brake_accu: 8,
        nws_hyd: 0b10,
        fcs_hyd: 0b11,
        flaps_electric: false,
        alt_gear_s: 10.0,
        tanks: F16_TANKS,
        feed: Feed::Common,
        boost_pumps: true,
        starter: Starter::Jfs,
        carburettor: false,
        vacuum: false,
        elec_turn: false,
        glass: true,
        fire_detect: true,
        bottles: 0,
        eng_anti_ice: true,
        alerting: Alerting::Vms,
        ..trainer()
    }
}

fn jet_airliner() -> SysDef {
    SysDef {
        volts: 28.0,
        batt_v: 24.0,
        batt_ah: 46.0,
        load_a: 60.0,
        pitot_heat_a: 4.0,
        gens_per_engine: 1,
        apu: true,
        rat: true,
        epu_min: 0.0,
        hyd: A320_HYD,
        gear_hyd: 0b001,
        flap_hyd: 0b101,
        spoiler_hyd: 0b111,
        brake_hyd: 0b001,
        brake_alt_hyd: 0b100,
        brake_accu: 7,
        nws_hyd: 0b001,
        fcs_hyd: 0b111,
        ptu: true,
        flaps_electric: false,
        alt_gear_s: 30.0,
        tanks: WING3,
        feed: Feed::Wing,
        boost_pumps: true,
        engine_pump_vital: false,
        starter: Starter::JetApu,
        carburettor: false,
        vacuum: false,
        elec_turn: false,
        glass: true,
        const_speed_prop: false,
        fire_detect: true,
        bottles: 2,
        pitot_heat: true,
        wing_anti_ice: true,
        eng_anti_ice: true,
        alerting: Alerting::Ecam,
    }
}

fn turboprop_airliner() -> SysDef {
    SysDef {
        batt_ah: 43.0,
        load_a: 50.0,
        apu: false,
        rat: false,
        hyd: ATR_HYD,
        gear_hyd: 0b10,
        flap_hyd: 0b01,
        spoiler_hyd: 0b01,
        brake_hyd: 0b10,
        brake_alt_hyd: 0b01,
        nws_hyd: 0b01,
        // Manual flight controls (cables and tabs).
        fcs_hyd: 0,
        ptu: false,
        alt_gear_s: 25.0,
        tanks: WING2,
        starter: Starter::TurbopropBattery,
        const_speed_prop: true,
        alerting: Alerting::Atr,
        ..jet_airliner()
    }
}

/// Single-turboshaft helicopter (EC130 class): 24 V / 17 A·h battery,
/// starter-generator, one fuel tank, MGB-driven hydraulic servos (boosted
/// cyclic / collective / pedals), engine fire detection, annunciator panel.
fn helicopter() -> SysDef {
    SysDef {
        volts: 28.0,
        batt_v: 24.0,
        batt_ah: 17.0,
        load_a: 30.0,
        pitot_heat_a: 7.0,
        hyd: HELI_HYD,
        fcs_hyd: 0b1,
        tanks: HELI_TANK,
        feed: Feed::Common,
        boost_pumps: true,
        starter: Starter::TurbopropBattery,
        carburettor: false,
        vacuum: false,
        elec_turn: true,
        glass: false,
        fire_detect: true,
        bottles: 0,
        eng_anti_ice: false,
        alerting: Alerting::Annunciators,
        ..trainer()
    }
}

/// Battery-electric drone: flight controller and video on the flight pack.
fn drone() -> SysDef {
    SysDef { volts: 25.0, batt_v: 25.0, batt_ah: 1.3, load_a: 0.2, gens_per_engine: 0, ..glider() }
}

/// The systems of aircraft `def`.
pub fn sys_def(def: &AircraftDef) -> SysDef {
    let mut d = match def.id {
        "kestrel" => trainer(),
        "swift" => glider(),
        "peregrine" => warbird(),
        "lancer" => fast_jet(),
        "aerolink" => turboprop_airliner(),
        "meridian190" => SysDef {
            hyd: E190_HYD,
            gear_hyd: 0b010,
            flap_hyd: 0,
            flaps_electric: true,
            spoiler_hyd: 0b111,
            brake_hyd: 0b011,
            brake_alt_hyd: 0,
            nws_hyd: 0b010,
            ptu: true,
            batt_ah: 54.0,
            alerting: Alerting::Eicas,
            ..jet_airliner()
        },
        "meridian320" => jet_airliner(),
        "triple7" => SysDef {
            hyd: B777_HYD,
            gear_hyd: 0b010,
            flap_hyd: 0b010,
            spoiler_hyd: 0b111,
            brake_hyd: 0b100,
            brake_alt_hyd: 0b010,
            nws_hyd: 0b010,
            ptu: false,
            batt_ah: 47.0,
            load_a: 90.0,
            gens_per_engine: 2,
            tanks: T7_TANKS,
            alerting: Alerting::Eicas,
            ..jet_airliner()
        },
        "crj700" => SysDef {
            hyd: CRJ_HYD,
            gear_hyd: 0b100,
            flap_hyd: 0,
            flaps_electric: true,
            spoiler_hyd: 0b011,
            brake_hyd: 0b110,
            brake_alt_hyd: 0,
            nws_hyd: 0b100,
            ptu: false,
            batt_ah: 43.0,
            load_a: 50.0,
            alerting: Alerting::Eicas,
            ..jet_airliner()
        },
        "citationx" => SysDef {
            hyd: C750_HYD,
            gear_hyd: 0b11,
            flap_hyd: 0b11,
            spoiler_hyd: 0b11,
            brake_hyd: 0b01,
            brake_alt_hyd: 0,
            brake_accu: 10,
            nws_hyd: 0b10,
            fcs_hyd: 0b11,
            ptu: false,
            rat: false,
            batt_ah: 44.0,
            load_a: 35.0,
            tanks: WING3,
            bottles: 1,
            alerting: Alerting::Eicas,
            ..jet_airliner()
        },
        "concorde" => SysDef {
            hyd: CONCORDE_HYD,
            gear_hyd: 0b001,
            flap_hyd: 0b001,
            spoiler_hyd: 0,
            brake_hyd: 0b001,
            brake_alt_hyd: 0b100,
            nws_hyd: 0b001,
            fcs_hyd: 0b111,
            ptu: false,
            // No APU: ground air starts the first engine (modelled as the APU bleed).
            apu: true,
            batt_ah: 40.0,
            load_a: 80.0,
            tanks: CONCORDE_TANKS,
            glass: false,
            alerting: Alerting::Eicas,
            ..jet_airliner()
        },
        _ => match (def.category, def.engine_kind) {
            (Category::Helicopter, _) => helicopter(),
            (Category::Drone, _) | (_, EngineKind::Electric) => drone(),
            (Category::Glider, _) | (_, EngineKind::None) => glider(),
            (Category::FastJet, _) => fast_jet(),
            (Category::Trainer, _) => trainer(),
            (Category::Warbird, EngineKind::Piston) => warbird(),
            (_, EngineKind::Turboprop) => turboprop_airliner(),
            (_, EngineKind::Piston) => SysDef { tanks: WING2, feed: Feed::Wing, carburettor: false, const_speed_prop: true, ..warbird() },
            _ => jet_airliner(),
        },
    };
    // Things the flight model decides.
    if !def.has_engine || def.engine_count == 0 {
        d.starter = Starter::None;
        d.gens_per_engine = 0;
        d.tanks = NO_TANKS;
    }
    if !def.retractable_gear {
        d.gear_hyd = 0;
        d.alt_gear_s = 0.0;
    }
    if !def.has_spoilers {
        d.spoiler_hyd = 0;
    }
    if def.engine_count <= 1 && d.feed == Feed::Wing && def.category != Category::Airliner {
        d.feed = Feed::Selector;
    }
    d.const_speed_prop = d.const_speed_prop && def.const_speed_prop;
    d
}

impl SysDef {
    /// Engine index for a pump entry (resolving [`LAST`]).
    pub fn pump_engine(p: u8, n_eng: usize) -> usize {
        if p == LAST { n_eng.saturating_sub(1) } else { p as usize }
    }
    /// The systems that ever hold pressure (bit mask).
    pub fn hyd_mask(&self) -> u8 {
        ((1u16 << self.hyd.len().min(8)) - 1) as u8
    }
}
