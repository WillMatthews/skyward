//! The failure catalogue (what can break, on which types) and the failure
//! schedule: failures armed for a time / altitude / speed / random window,
//! and MTBF-style random failures.

use super::defs::{Alerting, Feed, Starter, SysDef};
use crate::flight::{AircraftDef, KT};

/// A kind of failure. Indexed kinds (engines, tanks, hydraulic systems,
/// control axes, tyres) take an index in [`Failure`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    EngineFire,
    Flameout,
    PartialPower,
    PropOverspeed,
    CarbIce,
    FuelLeak,
    FuelPump,
    Generator,
    Vacuum,
    PitotBlock,
    StaticBlock,
    AttitudeInd,
    HeadingInd,
    DisplayFail,
    GearExtend,
    FlapAsym,
    FlapJam,
    Hydraulic,
    ControlJam,
    Brakes,
    TyreBurst,
    ElecFire,
    BirdStrike,
    Icing,
    NavRadio,
    GlideSlope,
    Adf,
}

/// Number of failure kinds.
pub const N_KINDS: usize = 27;

/// Menu groups.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Group {
    Engines,
    Fuel,
    Electrical,
    Hydraulics,
    Instruments,
    Controls,
    Gear,
    Airframe,
}

impl Group {
    pub const ALL: [Group; 8] = [Group::Engines, Group::Fuel, Group::Electrical, Group::Hydraulics, Group::Instruments, Group::Controls, Group::Gear, Group::Airframe];
    pub fn name(self) -> &'static str {
        match self {
            Group::Engines => "Engines",
            Group::Fuel => "Fuel",
            Group::Electrical => "Electrical",
            Group::Hydraulics => "Hydraulics",
            Group::Instruments => "Instruments",
            Group::Controls => "Flight controls",
            Group::Gear => "Landing gear & brakes",
            Group::Airframe => "Airframe & environment",
        }
    }
}

/// What a kind's index counts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Index {
    None,
    Engine,
    Tank,
    Hyd,
    Axis,
    Side,
    /// NAV 1 / NAV 2.
    Radio,
}

impl Kind {
    pub const ALL: [Kind; N_KINDS] = [
        Kind::EngineFire,
        Kind::Flameout,
        Kind::PartialPower,
        Kind::PropOverspeed,
        Kind::CarbIce,
        Kind::FuelLeak,
        Kind::FuelPump,
        Kind::Generator,
        Kind::Vacuum,
        Kind::PitotBlock,
        Kind::StaticBlock,
        Kind::AttitudeInd,
        Kind::HeadingInd,
        Kind::DisplayFail,
        Kind::GearExtend,
        Kind::FlapAsym,
        Kind::FlapJam,
        Kind::Hydraulic,
        Kind::ControlJam,
        Kind::Brakes,
        Kind::TyreBurst,
        Kind::ElecFire,
        Kind::BirdStrike,
        Kind::Icing,
        Kind::NavRadio,
        Kind::GlideSlope,
        Kind::Adf,
    ];
    pub fn bit(self) -> usize {
        Kind::ALL.iter().position(|k| *k == self).unwrap_or(0)
    }
    /// Stable id (command line `--fail`, settings).
    pub fn id(self) -> &'static str {
        match self {
            Kind::EngineFire => "fire",
            Kind::Flameout => "flameout",
            Kind::PartialPower => "partial",
            Kind::PropOverspeed => "overspeed",
            Kind::CarbIce => "carbice",
            Kind::FuelLeak => "leak",
            Kind::FuelPump => "pump",
            Kind::Generator => "gen",
            Kind::Vacuum => "vacuum",
            Kind::PitotBlock => "pitot",
            Kind::StaticBlock => "static",
            Kind::AttitudeInd => "ai",
            Kind::HeadingInd => "dg",
            Kind::DisplayFail => "pfd",
            Kind::GearExtend => "gear",
            Kind::FlapAsym => "flapasym",
            Kind::FlapJam => "flapjam",
            Kind::Hydraulic => "hyd",
            Kind::ControlJam => "jam",
            Kind::Brakes => "brakes",
            Kind::TyreBurst => "tyre",
            Kind::ElecFire => "smoke",
            Kind::BirdStrike => "bird",
            Kind::Icing => "icing",
            Kind::NavRadio => "nav",
            Kind::GlideSlope => "gs",
            Kind::Adf => "adf",
        }
    }
    pub fn group(self) -> Group {
        match self {
            Kind::EngineFire | Kind::Flameout | Kind::PartialPower | Kind::PropOverspeed | Kind::CarbIce => Group::Engines,
            Kind::FuelLeak | Kind::FuelPump => Group::Fuel,
            Kind::Generator | Kind::ElecFire => Group::Electrical,
            Kind::Hydraulic => Group::Hydraulics,
            Kind::Vacuum | Kind::PitotBlock | Kind::StaticBlock | Kind::AttitudeInd | Kind::HeadingInd | Kind::DisplayFail | Kind::NavRadio | Kind::GlideSlope | Kind::Adf => {
                Group::Instruments
            }
            Kind::FlapAsym | Kind::FlapJam | Kind::ControlJam => Group::Controls,
            Kind::GearExtend | Kind::Brakes | Kind::TyreBurst => Group::Gear,
            Kind::BirdStrike | Kind::Icing => Group::Airframe,
        }
    }
    pub fn index(self) -> Index {
        match self {
            Kind::EngineFire | Kind::Flameout | Kind::PartialPower | Kind::PropOverspeed | Kind::FuelPump | Kind::Generator => Index::Engine,
            Kind::FuelLeak => Index::Tank,
            Kind::Hydraulic => Index::Hyd,
            Kind::ControlJam => Index::Axis,
            Kind::TyreBurst => Index::Side,
            Kind::NavRadio | Kind::GlideSlope => Index::Radio,
            _ => Index::None,
        }
    }
    /// Number of indices this kind has on aircraft `def`.
    pub fn count(self, sd: &SysDef, def: &AircraftDef) -> usize {
        match self.index() {
            Index::None => 1,
            Index::Engine => def.engine_count.min(crate::flight::MAX_ENGINES),
            Index::Tank => sd.tanks.len(),
            Index::Hyd => sd.hyd.len(),
            Index::Axis => 3,
            Index::Side => 2,
            Index::Radio => 2,
        }
    }
    /// Can this happen on aircraft `def`?
    pub fn applies(self, sd: &SysDef, def: &AircraftDef) -> bool {
        let eng = def.has_engine && def.engine_count > 0;
        let powered_glass = sd.glass;
        match self {
            Kind::EngineFire | Kind::Flameout | Kind::PartialPower => eng,
            Kind::PropOverspeed => eng && sd.const_speed_prop,
            Kind::CarbIce => eng && sd.carburettor,
            Kind::FuelLeak => eng && !sd.tanks.is_empty(),
            Kind::FuelPump => eng && (sd.boost_pumps || sd.engine_pump_vital),
            Kind::Generator => eng && sd.gens_per_engine > 0,
            Kind::Vacuum => sd.vacuum,
            Kind::PitotBlock | Kind::StaticBlock => true,
            Kind::AttitudeInd | Kind::HeadingInd => sd.vacuum || powered_glass,
            Kind::DisplayFail => powered_glass && def.category != crate::flight::Category::FastJet,
            Kind::GearExtend => def.retractable_gear,
            Kind::FlapAsym | Kind::FlapJam => eng && !def.flap_detents.is_empty(),
            Kind::Hydraulic => !sd.hyd.is_empty(),
            Kind::ControlJam => true,
            Kind::Brakes => true,
            Kind::TyreBurst => def.category != crate::flight::Category::Glider,
            Kind::ElecFire => sd.gens_per_engine > 0 || sd.glass,
            Kind::BirdStrike => true,
            Kind::Icing => true,
            Kind::NavRadio | Kind::GlideSlope | Kind::Adf => crate::nav::state::has_radios(def),
        }
    }
    /// Menu label for index `i` on `def`.
    pub fn label(self, sd: &SysDef, def: &AircraftDef, i: usize) -> String {
        let n_eng = def.engine_count;
        let eng = |what: &str| if n_eng > 1 { format!("{what} — engine {}", i + 1) } else { what.to_string() };
        match self {
            Kind::EngineFire => eng("Engine fire"),
            Kind::Flameout => eng(if def.engine_kind == crate::flight::EngineKind::Piston { "Engine failure" } else { "Flameout" }),
            Kind::PartialPower => eng("Partial power loss"),
            Kind::PropOverspeed => eng("Propeller overspeed (governor)"),
            Kind::CarbIce => "Carburettor icing".into(),
            Kind::FuelLeak => format!("Fuel leak — {} tank", sd.tanks.get(i).map(|t| t.0).unwrap_or("?")),
            Kind::FuelPump => eng(if sd.engine_pump_vital { "Engine-driven fuel pump" } else { "Fuel boost pump" }),
            Kind::Generator => {
                if sd.alerting == Alerting::Annunciators {
                    eng(if def.engine_kind == crate::flight::EngineKind::Piston && sd.volts < 20.0 { "Generator" } else { "Alternator" })
                } else {
                    eng("Generator")
                }
            }
            Kind::Vacuum => "Vacuum pump".into(),
            Kind::PitotBlock => "Pitot blocked (ram + drain)".into(),
            Kind::StaticBlock => "Static port blocked".into(),
            Kind::AttitudeInd => (if sd.glass { "Attitude reference (AHRS / IRS)" } else { "Attitude indicator (gyro)" }).into(),
            Kind::HeadingInd => (if sd.glass { "Heading reference" } else { "Directional gyro" }).into(),
            Kind::DisplayFail => "Captain's PFD display unit".into(),
            Kind::GearExtend => "Gear won't extend (normal system)".into(),
            Kind::FlapAsym => "Flap asymmetry".into(),
            Kind::FlapJam => "Flaps jammed".into(),
            Kind::Hydraulic => format!("Hydraulic system {} (fluid loss)", sd.hyd.get(i).map(|h| h.name).unwrap_or("?")),
            Kind::ControlJam => format!("{} jammed", ["Elevator", "Ailerons", "Rudder"][i.min(2)]),
            Kind::Brakes => "Brake failure".into(),
            Kind::TyreBurst => format!("Tyre burst — {} main", ["left", "right"][i.min(1)]),
            Kind::ElecFire => "Electrical fire / smoke".into(),
            Kind::BirdStrike => "Bird strike".into(),
            Kind::Icing => "Severe airframe icing".into(),
            Kind::NavRadio => format!("NAV {} receiver (VOR / LOC)", i + 1),
            Kind::GlideSlope => format!("NAV {} glideslope receiver", i + 1),
            Kind::Adf => "ADF receiver".into(),
        }
    }
    /// One-line hint on the effect / what to do, for the menu.
    pub fn hint(self, sd: &SysDef) -> &'static str {
        match self {
            Kind::EngineFire => {
                if sd.bottles > 0 {
                    "Fire warning: thrust idle, fuel off, fire handle, agents"
                } else {
                    "Fuel off, mixture / throttle closed, land as soon as possible"
                }
            }
            Kind::Flameout => "Engine stops; relight / restart in the Systems tab",
            Kind::PartialPower => "Power rolls back to about half",
            Kind::PropOverspeed => "RPM above the red line; reduce power and speed",
            Kind::CarbIce => "Rough running, RPM drops; carburettor heat on",
            Kind::FuelLeak => "Tank empties; imbalance, then starvation",
            Kind::FuelPump => {
                if sd.engine_pump_vital {
                    "The engine starves"
                } else {
                    "Suction feed only: surges / flameout at altitude"
                }
            }
            Kind::Generator => "No charging: the battery runs down, then the avionics go",
            Kind::Vacuum => "Attitude and heading gyros slowly spin down and topple",
            Kind::PitotBlock => "ASI acts as an altimeter",
            Kind::StaticBlock => "Altimeter and VSI freeze; ASI wrong in climbs / descents",
            Kind::AttitudeInd => {
                if sd.glass {
                    "ATT flag on the PFD"
                } else {
                    "Gyro tumbles"
                }
            }
            Kind::HeadingInd => {
                if sd.glass {
                    "HDG flag on the PFD"
                } else {
                    "Heading precesses quickly"
                }
            }
            Kind::DisplayFail => "Red X on the PFD",
            Kind::GearExtend => "Use the alternate extension",
            Kind::FlapAsym => "Split flaps roll the aircraft (airliners: flaps lock)",
            Kind::FlapJam => "Flaps stay where they are",
            Kind::Hydraulic => "Pressure lost; users of that system degrade",
            Kind::ControlJam => "The surface stays where it is",
            Kind::Brakes => "No wheel brakes",
            Kind::TyreBurst => "Drag and swing towards the burst tyre on the ground",
            Kind::ElecFire => "Smoke in the cockpit; isolate the avionics",
            Kind::BirdStrike => "Engine damage (vibration, power loss) and a cracked windscreen",
            Kind::Icing => "Ice builds up: more drag, earlier stall",
            Kind::NavRadio => "NAV flag: CDI, HSI and the coupled autopilot lose the beam",
            Kind::GlideSlope => "GS flag: localizer only (non-precision); the autopilot loses G/S",
            Kind::Adf => "ADF needle parks at 90°",
        }
    }
    /// Parse "fire", "fire2", "eng2fire", "jam-rudder", "tyre-right"... (1-based indices).
    pub fn parse(s: &str) -> Option<(Kind, usize)> {
        let s = s.trim().to_ascii_lowercase();
        let s = s.as_str();
        let alias = |a: &str| -> Option<(Kind, usize)> {
            Some(match a {
                "alternator" | "generator" => (Kind::Generator, 0),
                "attitude" | "ahrs" => (Kind::AttitudeInd, 0),
                "heading" | "hdg" => (Kind::HeadingInd, 0),
                "redx" | "display" => (Kind::DisplayFail, 0),
                "elecfire" => (Kind::ElecFire, 0),
                "birdstrike" => (Kind::BirdStrike, 0),
                "jam-elevator" | "jam-elev" => (Kind::ControlJam, 0),
                "jam-aileron" | "jam-ail" => (Kind::ControlJam, 1),
                "jam-rudder" | "jam-rud" => (Kind::ControlJam, 2),
                "tyre-left" | "tire-left" => (Kind::TyreBurst, 0),
                "tyre-right" | "tire-right" => (Kind::TyreBurst, 1),
                "leak-left" | "leakl" => (Kind::FuelLeak, 0),
                "leak-right" | "leakr" => (Kind::FuelLeak, 1),
                "leak-centre" | "leak-center" | "leakc" => (Kind::FuelLeak, 2),
                "engine" | "engfail" => (Kind::Flameout, 0),
                _ => return None,
            })
        };
        if let Some(a) = alias(s) {
            return Some(a);
        }
        // "eng2fire" → "fire2"
        let s = match s.strip_prefix("eng") {
            Some(rest) if rest.chars().next().is_some_and(|c| c.is_ascii_digit()) => {
                let d: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
                format!("{}{}", &rest[d.len()..], d)
            }
            _ => s.to_string(),
        };
        let digits: String = s.chars().rev().take_while(|c| c.is_ascii_digit()).collect::<Vec<_>>().into_iter().rev().collect();
        let name = &s[..s.len() - digits.len()];
        let idx = digits.parse::<usize>().ok().map(|i| i.max(1) - 1).unwrap_or(0);
        Kind::ALL.iter().find(|k| k.id() == name).map(|k| (*k, idx))
    }
}

/// One failure: a kind and its index (engine, tank, hydraulic system, axis, side).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Failure {
    pub kind: Kind,
    pub idx: u8,
}

impl Failure {
    pub fn new(kind: Kind, idx: usize) -> Failure {
        Failure { kind, idx: idx.min(7) as u8 }
    }
}

/// The set of active failures: one bit per (kind, index).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Active([u8; N_KINDS]);

impl Active {
    pub fn has(&self, f: Failure) -> bool {
        self.0[f.kind.bit()] & (1 << f.idx) != 0
    }
    pub fn any_of(&self, k: Kind) -> bool {
        self.0[k.bit()] != 0
    }
    pub fn bits(&self, k: Kind) -> u8 {
        self.0[k.bit()]
    }
    pub fn set(&mut self, f: Failure, on: bool) {
        if on {
            self.0[f.kind.bit()] |= 1 << f.idx;
        } else {
            self.0[f.kind.bit()] &= !(1 << f.idx);
        }
    }
    pub fn is_empty(&self) -> bool {
        self.0.iter().all(|b| *b == 0)
    }
    pub fn clear(&mut self) {
        self.0 = [0; N_KINDS];
    }
    pub fn list(&self) -> Vec<Failure> {
        let mut v = Vec::new();
        for k in Kind::ALL {
            for i in 0..8 {
                let f = Failure::new(k, i);
                if self.has(f) {
                    v.push(f);
                }
            }
        }
        v
    }
}

/// When an armed failure fires.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Trigger {
    /// At sim time (s).
    At(f32),
    /// Climbing through / above an altitude (m), descending below one.
    Above(f32),
    Below(f32),
    /// Indicated airspeed above / below (m/s).
    FasterThan(f32),
    SlowerThan(f32),
}

impl Trigger {
    pub fn describe(&self, now: f32) -> String {
        match *self {
            Trigger::At(t) => {
                let s = (t - now).max(0.0);
                if s >= 60.0 { format!("in {}:{:02}", (s / 60.0) as u32, (s % 60.0) as u32) } else { format!("in {s:.0} s") }
            }
            Trigger::Above(m) => format!("above {:.0} ft", m * 3.280_84),
            Trigger::Below(m) => format!("below {:.0} ft", m * 3.280_84),
            Trigger::FasterThan(v) => format!("above {:.0} kt", v / KT),
            Trigger::SlowerThan(v) => format!("below {:.0} kt", v / KT),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Armed {
    pub f: Failure,
    pub when: Trigger,
    /// Armed for a random time in a window: the menu doesn't tell when.
    pub secret: bool,
}

/// Random (MTBF) failures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Random {
    pub on: bool,
    /// Mean time between failures, minutes of flight.
    pub mtbf_min: f32,
    /// Groups that may fail (bit per [`Group::ALL`] index).
    pub groups: u8,
}

impl Default for Random {
    fn default() -> Self {
        Random { on: false, mtbf_min: 30.0, groups: 0xFF }
    }
}

/// Armed and random failures (lives in the session, not the aircraft).
#[derive(Clone, Debug, Default)]
pub struct Schedule {
    pub armed: Vec<Armed>,
    pub random: Random,
    seed: u64,
}

impl Schedule {
    fn rand(&mut self) -> f32 {
        if self.seed == 0 {
            self.seed = 0x2545_F491_4F6C_DD1D;
        }
        let mut x = self.seed;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.seed = x;
        (x >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Arm `f` at a random time between `t0` and `t1` seconds from `now`.
    pub fn arm_window(&mut self, f: Failure, now: f32, t0: f32, t1: f32) {
        let t = now + t0 + (t1 - t0).max(0.0) * self.rand();
        self.armed.push(Armed { f, when: Trigger::At(t), secret: true });
    }

    pub fn arm(&mut self, f: Failure, when: Trigger) {
        self.armed.push(Armed { f, when, secret: false });
    }

    pub fn clear(&mut self) {
        self.armed.clear();
    }

    /// Advance: returns the failures that fire now. `alt` m, `ias` m/s,
    /// `airborne` (random failures only happen in flight).
    pub fn tick(&mut self, sd: &SysDef, def: &AircraftDef, now: f32, alt: f32, ias: f32, airborne: bool, dt: f32) -> Vec<Failure> {
        let mut out = Vec::new();
        self.armed.retain(|a| {
            let fire = match a.when {
                Trigger::At(t) => now >= t,
                Trigger::Above(m) => alt >= m,
                Trigger::Below(m) => alt <= m,
                Trigger::FasterThan(v) => ias >= v,
                Trigger::SlowerThan(v) => ias <= v && airborne,
            };
            if fire {
                out.push(a.f);
            }
            !fire
        });
        if self.random.on && airborne && dt > 0.0 {
            let rate = 1.0 / (self.random.mtbf_min.max(0.5) * 60.0);
            if self.rand() < rate * dt {
                let cands = candidates(sd, def, self.random.groups);
                if !cands.is_empty() {
                    let k = ((self.rand() * cands.len() as f32) as usize).min(cands.len() - 1);
                    out.push(cands[k]);
                }
            }
        }
        out
    }
}

/// Every failure that applies to `def` (in the given groups).
pub fn candidates(sd: &SysDef, def: &AircraftDef, groups: u8) -> Vec<Failure> {
    let mut v = Vec::new();
    for k in Kind::ALL {
        let g = Group::ALL.iter().position(|g| *g == k.group()).unwrap_or(0);
        if groups & (1 << g) == 0 || !k.applies(sd, def) {
            continue;
        }
        for i in 0..k.count(sd, def) {
            v.push(Failure::new(k, i));
        }
    }
    v
}

/// Ready-to-go helpers for the UI.
pub fn has_starter(sd: &SysDef) -> bool {
    sd.starter != Starter::None
}
pub fn has_crossfeed(sd: &SysDef) -> bool {
    sd.feed == Feed::Wing
}
