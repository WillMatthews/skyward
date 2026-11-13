//! Structural loads, wing flex, damage and failures (Track L).
//!
//! * [`StructDef`] — per-type structural data: limit / ultimate load factors
//!   (FAR 23 / 25 / military), design dive speed and Mach, flap / gear
//!   speeds, the first wing-bending mode (tip deflection per g, frequency,
//!   damping), part geometry (used to cut the imported model into separable
//!   parts, see `models.rs`) and part masses.
//! * [`Damage`] — the live structural state carried by `flight::Aircraft`:
//!   the wing-bending modal state (one mode per wing half, driven by the
//!   load on that half, so a rolling pull loads one wing more than the
//!   other), plastic strain beyond limit load (permanent set, reduced aileron
//!   effectiveness, jammed spoilers), flutter beyond the dive speed, flap /
//!   gear overspeed, and separations beyond ultimate load.
//! * [`aero_correction`] — what a lost or damaged surface does to the
//!   aerodynamics, applied on top of either aero model (JSBSim or
//!   parametric): the lost half-wing's share of lift and drag (and of the
//!   aileron moment and roll damping) is removed at its spanwise centre of
//!   lift, the tailplane / fin forces are estimated from their area and the
//!   local flow and removed at the tail arm, together with most of the
//!   pitch / yaw damping. Lost mass moves the centre of gravity.
//!
//! Body frame as `flight.rs`: +X left wing, +Y up, +Z nose, origin = CG.

use crate::flight::{AircraftDef, Surfaces, ContactKind, GearPoint, KT};
use glam::Vec3;

/// Separable parts of an airframe (debris bodies, render groups).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum Part {
    /// Centre fuselage with the wing box (always the main body).
    Fuselage = 0,
    /// Rear fuselage / tail cone (the empennage hangs on it).
    Aft = 1,
    WingL = 2,
    WingR = 3,
    HTailL = 4,
    HTailR = 5,
    Fin = 6,
    Eng0 = 7,
    Eng1 = 8,
    Eng2 = 9,
    Eng3 = 10,
    GearNose = 11,
    GearL = 12,
    GearR = 13,
    /// Forward fuselage / cockpit section.
    Nose = 14,
    /// Unused slot (keeps the GPU table a power of two).
    Spare = 15,
}

pub const N_PARTS: usize = 16;

impl Part {
    pub const ALL: [Part; N_PARTS] = [
        Part::Fuselage,
        Part::Aft,
        Part::WingL,
        Part::WingR,
        Part::HTailL,
        Part::HTailR,
        Part::Fin,
        Part::Eng0,
        Part::Eng1,
        Part::Eng2,
        Part::Eng3,
        Part::GearNose,
        Part::GearL,
        Part::GearR,
        Part::Nose,
        Part::Spare,
    ];
    pub fn bit(self) -> u16 {
        1 << self as u16
    }
    pub fn idx(self) -> usize {
        self as usize
    }
    pub fn from_idx(i: usize) -> Part {
        Part::ALL[i.min(N_PARTS - 1)]
    }
    pub fn engine(i: usize) -> Part {
        Part::from_idx(Part::Eng0 as usize + i.min(3))
    }
    pub fn engine_index(self) -> Option<usize> {
        let i = self as usize;
        (Part::Eng0 as usize..=Part::Eng3 as usize).contains(&i).then(|| i - Part::Eng0 as usize)
    }
    pub fn is_gear(self) -> bool {
        matches!(self, Part::GearNose | Part::GearL | Part::GearR)
    }
    pub fn name(self) -> &'static str {
        match self {
            Part::Fuselage => "fuselage",
            Part::Aft => "rear fuselage",
            Part::WingL => "left wing",
            Part::WingR => "right wing",
            Part::HTailL => "left tailplane",
            Part::HTailR => "right tailplane",
            Part::Fin => "fin",
            Part::Eng0 => "engine 1",
            Part::Eng1 => "engine 2",
            Part::Eng2 => "engine 3",
            Part::Eng3 => "engine 4",
            Part::GearNose => "nose gear",
            Part::GearL => "left main gear",
            Part::GearR => "right main gear",
            Part::Nose => "nose section",
            Part::Spare => "part",
        }
    }
    /// The part this one is attached to (it goes with its parent when the
    /// parent separates). Wing engines hang on their wing, the tail surfaces
    /// on the rear fuselage.
    pub fn parent(self, sd: &StructDef, def: &AircraftDef) -> Option<Part> {
        Some(match self {
            Part::Fuselage => return None,
            Part::Aft | Part::Nose | Part::WingL | Part::WingR => Part::Fuselage,
            Part::HTailL | Part::HTailR | Part::Fin => Part::Aft,
            Part::GearNose => Part::Fuselage,
            Part::GearL | Part::GearR => {
                let side = if self == Part::GearL { 1.0 } else { -1.0 };
                // Wing-mounted main gear (outboard of the fuselage).
                let x = def.gear.iter().filter(|g| g.pos.x * side > 0.1).map(|g| g.pos.x.abs()).fold(0.0, f32::max);
                // (High-wing types carry their gear on the fuselage.)
                if x > sd.fus_r + 0.2 && def.wing_height < 0.3 { if side > 0.0 { Part::WingL } else { Part::WingR } } else { Part::Fuselage }
            }
            Part::Eng0 | Part::Eng1 | Part::Eng2 | Part::Eng3 => {
                let i = self.engine_index().unwrap_or(0);
                match def.engine_pos.get(i) {
                    Some(p) if p.x > sd.fus_r => Part::WingL,
                    Some(p) if p.x < -sd.fus_r => Part::WingR,
                    _ => Part::Fuselage,
                }
            }
            Part::Spare => Part::Fuselage,
        })
    }
}

/// Bit masks.
pub const GEAR_BITS: u16 = (1 << Part::GearNose as u16) | (1 << Part::GearL as u16) | (1 << Part::GearR as u16);

/// Per-type structural data.
#[derive(Clone, Copy, Debug)]
pub struct StructDef {
    // ---- loads -------------------------------------------------------------
    /// Limit load factors (+ / −): no permanent deformation below these.
    pub n_lim: f32,
    pub n_lim_neg: f32,
    /// Ultimate / limit (1.5 by regulation): the structure fails beyond.
    pub ult: f32,
    /// Design dive speed (kt IAS) and Mach (0 = none). Flutter begins a
    /// margin above them.
    pub vd_kt: f32,
    pub md: f32,
    /// Maximum flap-extended speed per flap detent (kt; 0 = no limit).
    pub vfe_kt: &'static [f32],
    /// Gear operating speed (kt): extending above it damages the doors.
    pub vlo_kt: f32,
    /// Lateral load (g) the main gear survives on the ground.
    pub gear_side_g: f32,
    /// The surface that flutters first beyond the dive speed.
    pub flutter_wing: bool,
    // ---- first wing-bending mode -------------------------------------------
    /// Tip deflection (m) per g of net wing load (the model is drawn in the
    /// unloaded, parked shape: 1 g flight raises the tips by this much).
    pub tip_per_g: f32,
    /// Inertia relief: wing + fuel + engine mass fraction (load that the wing
    /// carries itself, opposite to the lift).
    pub relief: f32,
    /// First bending frequency (Hz) and damping ratio.
    pub f_bend: f32,
    pub zeta: f32,
    /// Bend-twist coupling: tip twist (rad, + = leading edge up) per metre of
    /// tip deflection (swept wings wash out as they bend).
    pub twist_per_m: f32,
    // ---- geometry (body frame, m) -------------------------------------------
    /// Fuselage half-width: inboard of it is fuselage, outboard is wing.
    pub fus_r: f32,
    /// Empennage begins aft of this station.
    pub tail_z: f32,
    /// Fin: above this height (in the empennage) and within `fin_r` of the centreline.
    pub fin_y: f32,
    pub fin_r: f32,
    /// Wing root chord extent (z): wing vertices lie within it (plus sweep).
    pub wing_z: (f32, f32),
    /// Fuselage break stations: forward of `nose_z` is the nose section,
    /// aft of `aft_z` the rear fuselage.
    pub nose_z: f32,
    pub aft_z: f32,
    /// Single nose-mounted engine: the engine (cowling, propeller) is forward of this.
    pub firewall_z: Option<f32>,
    /// Wing-mounted nacelles: radius, forward and aft extent from the engine position.
    pub nacelle: (f32, f32, f32),
    // ---- aerodynamics of the parts ------------------------------------------
    /// Spanwise centre of lift of a half-wing, from the centreline (m).
    pub wing_yc: f32,
    /// Horizontal tail: area / wing area, centroid of the left half.
    pub ht_area: f32,
    pub ht_pos: Vec3,
    /// Fin: area / wing area, centroid.
    pub vt_area: f32,
    pub vt_pos: Vec3,
    // ---- masses ---------------------------------------------------------------
    /// Fractions of the aircraft mass: one half-wing (structure + its fuel),
    /// one tailplane half, the fin, one main gear leg, the nose section, the
    /// rear fuselage. Engine mass in kg (each).
    pub m_wing: f32,
    pub m_htail: f32,
    pub m_fin: f32,
    pub m_gear: f32,
    pub m_nose: f32,
    pub m_aft: f32,
    pub m_engine: f32,
    /// Fuel fraction of the aircraft mass (fires).
    pub fuel_frac: f32,
}

const KESTREL: StructDef = StructDef {
    // Cessna 172P: normal category +3.8 / −1.52 g; VD ≈ 182 kt.
    n_lim: 3.8,
    n_lim_neg: -1.52,
    ult: 1.5,
    vd_kt: 182.0,
    md: 0.0,
    vfe_kt: &[0.0, 110.0, 85.0, 85.0],
    vlo_kt: 0.0,
    gear_side_g: 0.9,
    flutter_wing: false,
    tip_per_g: 0.03,
    relief: 0.18,
    f_bend: 5.5,
    zeta: 0.05,
    twist_per_m: 0.0,
    fus_r: 0.6,
    tail_z: -4.15,
    fin_y: 0.3,
    fin_r: 0.22,
    wing_z: (-1.5, 0.65),
    nose_z: 99.0,
    aft_z: -2.4,
    firewall_z: Some(1.0),
    nacelle: (0.6, 1.2, 1.0),
    wing_yc: 2.4,
    ht_area: 0.22,
    ht_pos: Vec3::new(1.0, 0.0, -4.95),
    vt_area: 0.11,
    vt_pos: Vec3::new(0.0, 0.85, -5.2),
    m_wing: 0.11,
    m_htail: 0.012,
    m_fin: 0.01,
    m_gear: 0.015,
    m_nose: 0.0,
    m_aft: 0.08,
    m_engine: 150.0,
    fuel_frac: 0.15,
};

const SWIFT: StructDef = StructDef {
    // ASK 21: utility (CS-22) +5.3 / −2.65 g; VD ≈ 168 kt. Long glider wings
    // bend a lot (≈ 1 m at limit load).
    n_lim: 5.3,
    n_lim_neg: -2.65,
    ult: 1.5,
    vd_kt: 168.0,
    md: 0.0,
    vfe_kt: &[],
    vlo_kt: 0.0,
    gear_side_g: 1.2,
    flutter_wing: true,
    tip_per_g: 0.27,
    relief: 0.35,
    f_bend: 2.2,
    zeta: 0.03,
    twist_per_m: 0.0,
    fus_r: 0.55,
    tail_z: -4.55,
    fin_y: 0.15,
    fin_r: 0.18,
    wing_z: (-1.1, 0.4),
    nose_z: 99.0,
    aft_z: -2.2,
    firewall_z: None,
    nacelle: (0.0, 0.0, 0.0),
    wing_yc: 3.7,
    ht_area: 0.15,
    ht_pos: Vec3::new(0.75, 1.3, -5.3),
    vt_area: 0.08,
    vt_pos: Vec3::new(0.0, 0.75, -5.3),
    m_wing: 0.2,
    m_htail: 0.01,
    m_fin: 0.02,
    m_gear: 0.0,
    m_nose: 0.0,
    m_aft: 0.06,
    m_engine: 0.0,
    fuel_frac: 0.0,
};

const PEREGRINE: StructDef = StructDef {
    // Spitfire IX: aerobatic fighter, ≈ +6.3 / −3 g limit (ult +9.5).
    n_lim: 6.33,
    n_lim_neg: -3.0,
    ult: 1.5,
    vd_kt: 420.0,
    md: 0.0,
    vfe_kt: &[0.0, 139.0],
    vlo_kt: 139.0,
    gear_side_g: 0.8,
    flutter_wing: false,
    tip_per_g: 0.05,
    relief: 0.22,
    f_bend: 6.5,
    zeta: 0.05,
    twist_per_m: 0.0,
    fus_r: 0.55,
    tail_z: -4.9,
    fin_y: 0.5,
    fin_r: 0.14,
    wing_z: (-1.6, 2.1),
    nose_z: 99.0,
    aft_z: -2.6,
    firewall_z: Some(1.1),
    nacelle: (0.7, 2.0, 1.6),
    wing_yc: 2.3,
    ht_area: 0.13,
    ht_pos: Vec3::new(0.9, 0.4, -5.6),
    vt_area: 0.07,
    vt_pos: Vec3::new(0.0, 0.9, -5.7),
    m_wing: 0.09,
    m_htail: 0.01,
    m_fin: 0.008,
    m_gear: 0.015,
    m_nose: 0.0,
    m_aft: 0.06,
    m_engine: 750.0,
    fuel_frac: 0.08,
};

const LANCER: StructDef = StructDef {
    // F-16C: +9 / −3 g; 800 KIAS, Mach 2.05.
    n_lim: 9.0,
    n_lim_neg: -3.0,
    ult: 1.5,
    vd_kt: 850.0,
    md: 2.15,
    vfe_kt: &[],
    vlo_kt: 300.0,
    gear_side_g: 0.9,
    flutter_wing: false,
    tip_per_g: 0.012,
    relief: 0.2,
    f_bend: 9.0,
    zeta: 0.05,
    twist_per_m: 0.0,
    fus_r: 1.3,
    tail_z: -3.0,
    fin_y: 1.25,
    fin_r: 0.3,
    wing_z: (-2.6, 1.6),
    nose_z: 3.8,
    aft_z: -2.6,
    firewall_z: None,
    nacelle: (0.0, 0.0, 0.0),
    wing_yc: 2.3,
    ht_area: 0.21,
    ht_pos: Vec3::new(1.9, 0.2, -4.4),
    vt_area: 0.15,
    vt_pos: Vec3::new(0.0, 2.2, -4.9),
    m_wing: 0.07,
    m_htail: 0.015,
    m_fin: 0.01,
    m_gear: 0.012,
    m_nose: 0.08,
    m_aft: 0.25,
    m_engine: 0.0,
    fuel_frac: 0.22,
};

const AEROLINK: StructDef = StructDef {
    // ATR 72-500 (CS-25): +2.5 / −1 g; VD ≈ 285 kt, MD 0.6.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 285.0,
    md: 0.6,
    vfe_kt: &[0.0, 185.0, 150.0],
    vlo_kt: 170.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.5,
    relief: 0.25,
    f_bend: 2.0,
    zeta: 0.03,
    twist_per_m: 0.0,
    fus_r: 1.45,
    tail_z: -11.4,
    fin_y: 1.6,
    fin_r: 0.35,
    wing_z: (-2.1, 0.8),
    nose_z: 6.0,
    aft_z: -6.5,
    firewall_z: None,
    nacelle: (0.75, 1.3, 3.6),
    wing_yc: 5.6,
    ht_area: 0.19,
    ht_pos: Vec3::new(1.8, 4.3, -13.6),
    vt_area: 0.2,
    vt_pos: Vec3::new(0.0, 2.8, -13.8),
    m_wing: 0.14,
    m_htail: 0.008,
    m_fin: 0.01,
    m_gear: 0.015,
    m_nose: 0.12,
    m_aft: 0.12,
    m_engine: 900.0,
    fuel_frac: 0.2,
};

const MERIDIAN190: StructDef = StructDef {
    // E190: +2.5 / −1 g; VMO 320 / MMO 0.82 → VD ≈ 360 kt, MD 0.89.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 360.0,
    md: 0.89,
    vfe_kt: &[0.0, 230.0, 215.0, 200.0, 165.0],
    vlo_kt: 250.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.85,
    relief: 0.28,
    f_bend: 1.6,
    zeta: 0.03,
    twist_per_m: -0.02,
    fus_r: 1.55,
    tail_z: -14.5,
    fin_y: 2.6,
    fin_r: 0.32,
    wing_z: (-7.0, 1.9),
    nose_z: 9.5,
    aft_z: -9.0,
    firewall_z: None,
    nacelle: (0.95, 0.6, 3.6),
    wing_yc: 5.9,
    ht_area: 0.25,
    ht_pos: Vec3::new(3.0, 1.9, -19.5),
    vt_area: 0.18,
    vt_pos: Vec3::new(0.0, 5.2, -19.0),
    m_wing: 0.14,
    m_htail: 0.008,
    m_fin: 0.006,
    m_gear: 0.015,
    m_nose: 0.1,
    m_aft: 0.15,
    m_engine: 2300.0,
    fuel_frac: 0.2,
};

const MERIDIAN320: StructDef = StructDef {
    // A320-200: +2.5 / −1 g; VMO 350 / MMO 0.82 → VD 381 kt, MD 0.89.
    // Tip rises ≈ 1.2 m in 1 g flight, ≈ 2.5 m at 2.5 g.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 381.0,
    md: 0.89,
    vfe_kt: &[0.0, 230.0, 215.0, 200.0, 177.0],
    vlo_kt: 250.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 1.2,
    relief: 0.3,
    f_bend: 1.3,
    zeta: 0.03,
    twist_per_m: -0.015,
    fus_r: 2.0,
    tail_z: -16.5,
    fin_y: 2.9,
    fin_r: 0.35,
    wing_z: (-7.0, 3.6),
    nose_z: 11.0,
    aft_z: -10.0,
    firewall_z: None,
    nacelle: (1.15, 1.1, 4.6),
    wing_yc: 7.2,
    ht_area: 0.25,
    ht_pos: Vec3::new(3.3, 2.0, -19.2),
    vt_area: 0.177,
    vt_pos: Vec3::new(0.0, 5.5, -19.0),
    m_wing: 0.13,
    m_htail: 0.006,
    m_fin: 0.005,
    m_gear: 0.017,
    m_nose: 0.1,
    m_aft: 0.16,
    m_engine: 3200.0,
    fuel_frac: 0.15,
};

const HERCULES: StructDef = StructDef {
    // C-130H: +3.0 / −1.0 g (2.5 g at max weight); VMO 250 → VD ≈ 300 kt, MD 0.6.
    n_lim: 3.0,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 300.0,
    md: 0.6,
    vfe_kt: &[0.0, 175.0, 145.0],
    vlo_kt: 168.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.45,
    relief: 0.3,
    f_bend: 1.8,
    zeta: 0.03,
    twist_per_m: 0.0,
    fus_r: 2.15,
    tail_z: -11.0,
    fin_y: 2.0,
    fin_r: 0.4,
    wing_z: (-4.5, 0.75),
    nose_z: 7.0,
    aft_z: -6.5,
    firewall_z: None,
    nacelle: (0.65, 1.0, 4.8),
    wing_yc: 8.5,
    ht_area: 0.28,
    ht_pos: Vec3::new(3.9, 1.65, -14.3),
    vt_area: 0.19,
    vt_pos: Vec3::new(0.0, 4.8, -14.5),
    m_wing: 0.13,
    m_htail: 0.007,
    m_fin: 0.006,
    m_gear: 0.02,
    m_nose: 0.1,
    m_aft: 0.12,
    m_engine: 1900.0,
    fuel_frac: 0.2,
};

const GALAXY: StructDef = StructDef {
    // C-5M: +2.5 / −1.0 g; VMO 350 / MMO 0.825 → VD ≈ 400 kt, MD 0.88.
    // Long, flexible wing: the tips rise ≈ 2 m in 1 g flight.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 400.0,
    md: 0.88,
    vfe_kt: &[0.0, 250.0, 230.0, 205.0, 185.0, 165.0],
    vlo_kt: 250.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 2.0,
    relief: 0.35,
    f_bend: 0.9,
    zeta: 0.03,
    twist_per_m: -0.012,
    fus_r: 3.7,
    tail_z: -24.5,
    fin_y: 7.5,
    fin_r: 0.6,
    wing_z: (-6.3, 7.4),
    nose_z: 19.5,
    aft_z: -16.5,
    firewall_z: None,
    nacelle: (1.3, 1.5, 6.0),
    wing_yc: 14.0,
    ht_area: 0.156,
    ht_pos: Vec3::new(5.2, 15.4, -39.5),
    vt_area: 0.155,
    vt_pos: Vec3::new(0.0, 11.0, -34.5),
    m_wing: 0.15,
    m_htail: 0.004,
    m_fin: 0.004,
    m_gear: 0.02,
    m_nose: 0.08,
    m_aft: 0.12,
    m_engine: 5000.0,
    fuel_frac: 0.3,
};

const VULCAN: StructDef = StructDef {
    // Vulcan B.2: +3.0 / −1.0 g; VD ≈ 450 kt, MD 0.98. A stiff, deep delta
    // with buried engines and no tailplane (elevons).
    n_lim: 3.0,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 450.0,
    md: 0.98,
    vfe_kt: &[],
    vlo_kt: 230.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.15,
    relief: 0.3,
    f_bend: 3.0,
    zeta: 0.04,
    twist_per_m: 0.0,
    fus_r: 1.4,
    tail_z: -11.0,
    fin_y: 2.3,
    fin_r: 0.3,
    wing_z: (-10.3, 8.4),
    nose_z: 8.0,
    aft_z: -9.0,
    firewall_z: None,
    nacelle: (0.6, 6.0, 0.5),
    wing_yc: 6.0,
    ht_area: 0.0,
    ht_pos: Vec3::new(2.0, 0.0, -10.0),
    vt_area: 0.1,
    vt_pos: Vec3::new(0.0, 4.0, -11.5),
    m_wing: 0.2,
    m_htail: 0.001,
    m_fin: 0.008,
    m_gear: 0.02,
    m_nose: 0.08,
    m_aft: 0.08,
    m_engine: 1800.0,
    fuel_frac: 0.07,
};

const EC130: StructDef = StructDef {
    // EC130 B4 (CS-27): +3.5 / −1.0 g structural; Vne 155 kt. "Wings" are the
    // main rotor blades (no bending modelled), the tail is the boom / Fenestron.
    n_lim: 3.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 175.0,
    md: 0.0,
    vfe_kt: &[],
    vlo_kt: 0.0,
    gear_side_g: 0.8,
    flutter_wing: false,
    tip_per_g: 0.0,
    relief: 0.0,
    f_bend: 4.0,
    zeta: 0.1,
    twist_per_m: 0.0,
    fus_r: 1.1,
    tail_z: -3.0,
    fin_y: 0.9,
    fin_r: 0.5,
    wing_z: (-0.1, 0.7),
    nose_z: 99.0,
    aft_z: -2.6,
    firewall_z: None,
    nacelle: (0.5, 0.9, 1.6),
    wing_yc: 1.45,
    ht_area: 0.0,
    ht_pos: Vec3::new(0.0, -0.07, -4.55),
    vt_area: 0.0,
    vt_pos: Vec3::new(0.0, 0.6, -5.9),
    m_wing: 0.028,
    m_htail: 0.004,
    m_fin: 0.01,
    m_gear: 0.012,
    m_nose: 0.0,
    m_aft: 0.06,
    m_engine: 135.0,
    fuel_frac: 0.15,
};

const FPV: StructDef = StructDef {
    // 5-inch carbon frame: survives most things short of a wall.
    n_lim: 15.0,
    n_lim_neg: -15.0,
    ult: 1.5,
    vd_kt: 130.0,
    md: 0.0,
    vfe_kt: &[],
    vlo_kt: 0.0,
    gear_side_g: 5.0,
    flutter_wing: false,
    tip_per_g: 0.0,
    relief: 0.0,
    f_bend: 20.0,
    zeta: 0.1,
    twist_per_m: 0.0,
    fus_r: 0.03,
    tail_z: -0.08,
    fin_y: 0.02,
    fin_r: 0.02,
    wing_z: (-0.1, 0.1),
    nose_z: 99.0,
    aft_z: -0.06,
    firewall_z: None,
    nacelle: (0.02, 0.02, 0.02),
    wing_yc: 0.0,
    ht_area: 0.0,
    ht_pos: Vec3::new(0.0, 0.0, -0.08),
    vt_area: 0.0,
    vt_pos: Vec3::new(0.0, 0.0, -0.08),
    m_wing: 0.0,
    m_htail: 0.0,
    m_fin: 0.0,
    m_gear: 0.0,
    m_nose: 0.0,
    m_aft: 0.0,
    m_engine: 0.033,
    fuel_frac: 0.0,
};

const TRIPLE7: StructDef = StructDef {
    // 777-300ER: +2.5 / −1 g; VMO 330 / MMO 0.89 → VD ≈ 365 kt, MD 0.95. The
    // long, flexible wing: tips rise ≈ 2 m in 1 g flight.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 365.0,
    md: 0.95,
    vfe_kt: &[0.0, 255.0, 235.0, 215.0, 195.0, 185.0, 170.0],
    vlo_kt: 270.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 2.0,
    relief: 0.3,
    f_bend: 0.9,
    zeta: 0.03,
    twist_per_m: -0.012,
    fus_r: 3.1,
    tail_z: -27.5,
    fin_y: 4.0,
    fin_r: 0.5,
    wing_z: (-8.0, 5.0),
    nose_z: 25.0,
    aft_z: -20.0,
    firewall_z: None,
    nacelle: (2.0, 1.0, 7.0),
    wing_yc: 13.0,
    ht_area: 0.23,
    ht_pos: Vec3::new(5.4, 1.5, -34.5),
    vt_area: 0.12,
    vt_pos: Vec3::new(0.0, 7.5, -33.5),
    m_wing: 0.14,
    m_htail: 0.005,
    m_fin: 0.004,
    m_gear: 0.02,
    m_nose: 0.08,
    m_aft: 0.14,
    m_engine: 8_760.0,
    fuel_frac: 0.25,
};

const CRJ700: StructDef = StructDef {
    // CRJ700: +2.5 / −1 g; VMO 335 / MMO 0.825 → VD ≈ 370 kt, MD 0.89.
    // Rear-mounted engines, T-tail.
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 370.0,
    md: 0.89,
    vfe_kt: &[0.0, 230.0, 230.0, 220.0, 185.0, 170.0],
    vlo_kt: 220.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.5,
    relief: 0.3,
    f_bend: 2.0,
    zeta: 0.03,
    twist_per_m: -0.01,
    fus_r: 1.35,
    tail_z: -11.0,
    fin_y: 1.8,
    fin_r: 0.3,
    wing_z: (-4.9, 2.5),
    nose_z: 10.5,
    aft_z: -8.5,
    firewall_z: None,
    nacelle: (0.75, 0.4, 3.0),
    wing_yc: 4.8,
    ht_area: 0.25,
    ht_pos: Vec3::new(2.2, 3.8, -15.0),
    vt_area: 0.15,
    vt_pos: Vec3::new(0.0, 2.6, -13.0),
    m_wing: 0.11,
    m_htail: 0.006,
    m_fin: 0.006,
    m_gear: 0.02,
    m_nose: 0.08,
    m_aft: 0.18,
    m_engine: 1_100.0,
    fuel_frac: 0.13,
};

const CITATIONX: StructDef = StructDef {
    // Citation X: +2.9 / −1 g; VMO 350 / MMO 0.92 → VD ≈ 385 kt, MD 0.98.
    n_lim: 2.9,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 385.0,
    md: 0.98,
    vfe_kt: &[0.0, 250.0, 210.0, 180.0],
    vlo_kt: 210.0,
    gear_side_g: 0.8,
    flutter_wing: false,
    tip_per_g: 0.35,
    relief: 0.3,
    f_bend: 2.5,
    zeta: 0.03,
    twist_per_m: -0.015,
    fus_r: 0.95,
    tail_z: -6.5,
    fin_y: 1.6,
    fin_r: 0.25,
    wing_z: (-3.5, 2.7),
    nose_z: 6.0,
    aft_z: -3.5,
    firewall_z: None,
    nacelle: (0.55, 0.3, 3.5),
    wing_yc: 4.0,
    ht_area: 0.2,
    ht_pos: Vec3::new(2.2, 3.4, -9.6),
    vt_area: 0.15,
    vt_pos: Vec3::new(0.0, 2.5, -8.0),
    m_wing: 0.12,
    m_htail: 0.006,
    m_fin: 0.006,
    m_gear: 0.018,
    m_nose: 0.08,
    m_aft: 0.18,
    m_engine: 750.0,
    fuel_frac: 0.25,
};

const CONCORDE: StructDef = StructDef {
    // Concorde: +2.5 / −1 g; VMO 400-530 kt / MMO 2.04 → VD ≈ 580 kt, MD
    // 2.2. A stiff slender delta, no tailplane; droop-nose limits 325 /
    // 270 kt (5° / 12.5°).
    n_lim: 2.5,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 580.0,
    md: 2.2,
    vfe_kt: &[0.0, 325.0, 325.0, 270.0],
    vlo_kt: 270.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.15,
    relief: 0.35,
    f_bend: 2.5,
    zeta: 0.04,
    twist_per_m: 0.0,
    fus_r: 1.45,
    tail_z: -19.0,
    fin_y: 3.4,
    fin_r: 0.3,
    wing_z: (-24.0, 10.0),
    nose_z: 22.0,
    aft_z: -16.0,
    firewall_z: None,
    nacelle: (1.0, 4.5, 6.5),
    wing_yc: 6.0,
    ht_area: 0.0,
    ht_pos: Vec3::new(2.0, 0.0, -20.0),
    vt_area: 0.095,
    vt_pos: Vec3::new(0.0, 5.5, -22.5),
    m_wing: 0.18,
    m_htail: 0.001,
    m_fin: 0.006,
    m_gear: 0.02,
    m_nose: 0.06,
    m_aft: 0.10,
    m_engine: 3_175.0,
    fuel_frac: 0.22,
};

// ---- Track H2b ----------------------------------------------------------------

const EXTRA500: StructDef = StructDef {
    // Extra EA-500: CS-23 normal category +3.8 / −1.52 g; Vne 210 → VD ≈ 235 kt.
    n_lim: 3.8,
    n_lim_neg: -1.52,
    ult: 1.5,
    vd_kt: 235.0,
    md: 0.0,
    vfe_kt: &[0.0, 140.0, 110.0],
    vlo_kt: 150.0,
    gear_side_g: 0.8,
    flutter_wing: false,
    tip_per_g: 0.04,
    relief: 0.2,
    f_bend: 5.0,
    zeta: 0.05,
    twist_per_m: 0.0,
    fus_r: 0.65,
    tail_z: -4.0,
    fin_y: 0.5,
    fin_r: 0.2,
    wing_z: (-1.3, 0.6),
    nose_z: 99.0,
    aft_z: -2.6,
    firewall_z: Some(1.9),
    nacelle: (0.6, 1.4, 1.2),
    wing_yc: 2.4,
    ht_area: 0.2,
    ht_pos: Vec3::new(1.2, 1.3, -5.2),
    vt_area: 0.12,
    vt_pos: Vec3::new(0.0, 1.0, -5.3),
    m_wing: 0.1,
    m_htail: 0.01,
    m_fin: 0.01,
    m_gear: 0.015,
    m_nose: 0.0,
    m_aft: 0.08,
    m_engine: 110.0,
    fuel_frac: 0.19,
};

const CUB: StructDef = StructDef {
    // PA-18-150: normal category +4.4 / −1.76 g; Vne 133 → VD ≈ 150 kt.
    n_lim: 4.4,
    n_lim_neg: -1.76,
    ult: 1.5,
    vd_kt: 150.0,
    md: 0.0,
    vfe_kt: &[0.0, 80.0, 70.0],
    vlo_kt: 0.0,
    gear_side_g: 0.9,
    flutter_wing: false,
    tip_per_g: 0.03,
    relief: 0.15,
    f_bend: 5.5,
    zeta: 0.06,
    twist_per_m: 0.0,
    fus_r: 0.45,
    tail_z: -3.9,
    fin_y: 0.2,
    fin_r: 0.2,
    wing_z: (-1.3, 0.4),
    nose_z: 99.0,
    aft_z: -2.2,
    firewall_z: Some(1.0),
    nacelle: (0.5, 1.0, 0.9),
    wing_yc: 2.4,
    ht_area: 0.18,
    ht_pos: Vec3::new(0.8, 0.1, -4.6),
    vt_area: 0.07,
    vt_pos: Vec3::new(0.0, 0.7, -4.8),
    m_wing: 0.1,
    m_htail: 0.012,
    m_fin: 0.008,
    m_gear: 0.015,
    m_nose: 0.0,
    m_aft: 0.08,
    m_engine: 125.0,
    fuel_frac: 0.08,
};

const ZLIN: StructDef = StructDef {
    // Zlin Z-50 LX: aerobatic, +9 / −6 g limit at 720 kg; Vne 184 → VD ≈ 205 kt.
    n_lim: 9.0,
    n_lim_neg: -6.0,
    ult: 1.5,
    vd_kt: 205.0,
    md: 0.0,
    vfe_kt: &[],
    vlo_kt: 0.0,
    gear_side_g: 1.0,
    flutter_wing: false,
    tip_per_g: 0.015,
    relief: 0.15,
    f_bend: 9.0,
    zeta: 0.06,
    twist_per_m: 0.0,
    fus_r: 0.5,
    tail_z: -3.0,
    fin_y: 0.3,
    fin_r: 0.2,
    wing_z: (-1.6, 0.3),
    nose_z: 99.0,
    aft_z: -2.4,
    firewall_z: Some(1.0),
    nacelle: (0.5, 1.0, 0.9),
    wing_yc: 1.9,
    ht_area: 0.22,
    ht_pos: Vec3::new(0.85, 0.6, -3.9),
    vt_area: 0.12,
    vt_pos: Vec3::new(0.0, 0.9, -4.0),
    m_wing: 0.08,
    m_htail: 0.012,
    m_fin: 0.01,
    m_gear: 0.012,
    m_nose: 0.0,
    m_aft: 0.08,
    m_engine: 200.0,
    fuel_frac: 0.08,
};

const OTTER: StructDef = StructDef {
    // DHC-3 Otter: normal category ≈ +3.5 / −1.4 g; Vne 160 → VD ≈ 180 kt.
    n_lim: 3.5,
    n_lim_neg: -1.4,
    ult: 1.5,
    vd_kt: 180.0,
    md: 0.0,
    vfe_kt: &[0.0, 100.0, 90.0, 85.0],
    vlo_kt: 0.0,
    // (On floats: the struts take sideways loads on the water.)
    gear_side_g: 0.8,
    flutter_wing: false,
    tip_per_g: 0.06,
    relief: 0.2,
    f_bend: 4.0,
    zeta: 0.05,
    twist_per_m: 0.0,
    fus_r: 0.9,
    tail_z: -5.5,
    fin_y: 0.8,
    fin_r: 0.25,
    wing_z: (-1.7, 0.5),
    nose_z: 99.0,
    aft_z: -3.5,
    firewall_z: Some(2.4),
    nacelle: (0.8, 1.5, 1.2),
    wing_yc: 3.9,
    ht_area: 0.2,
    ht_pos: Vec3::new(1.5, 1.4, -7.8),
    vt_area: 0.1,
    vt_pos: Vec3::new(0.0, 1.8, -8.2),
    m_wing: 0.09,
    m_htail: 0.01,
    m_fin: 0.01,
    m_gear: 0.04,
    m_nose: 0.0,
    m_aft: 0.08,
    m_engine: 420.0,
    fuel_frac: 0.12,
};

const DC3: StructDef = StructDef {
    // DC-3: +2.83 / −0.83 g flaps up (Buckaroo's FDM notes); Vne 190 → VD ≈ 215 kt.
    n_lim: 2.83,
    n_lim_neg: -1.0,
    ult: 1.5,
    vd_kt: 215.0,
    md: 0.0,
    vfe_kt: &[0.0, 135.0, 99.0, 97.0],
    vlo_kt: 126.0,
    gear_side_g: 0.6,
    flutter_wing: false,
    tip_per_g: 0.2,
    relief: 0.25,
    f_bend: 2.8,
    zeta: 0.04,
    twist_per_m: 0.0,
    fus_r: 1.35,
    tail_z: -8.0,
    fin_y: 1.3,
    fin_r: 0.3,
    wing_z: (-3.0, 1.5),
    nose_z: 4.5,
    aft_z: -4.5,
    firewall_z: None,
    nacelle: (0.75, 1.0, 4.0),
    wing_yc: 6.5,
    ht_area: 0.2,
    ht_pos: Vec3::new(2.5, 1.0, -12.0),
    vt_area: 0.08,
    vt_pos: Vec3::new(0.0, 3.0, -12.4),
    m_wing: 0.13,
    m_htail: 0.01,
    m_fin: 0.008,
    m_gear: 0.02,
    m_nose: 0.04,
    m_aft: 0.08,
    m_engine: 670.0,
    fuel_frac: 0.15,
};

/// Structural data for an aircraft id (a light-aircraft default for unknown ids).
pub fn struct_def(id: &str) -> &'static StructDef {
    match id {
        "kestrel" => &KESTREL,
        "swift" => &SWIFT,
        "peregrine" => &PEREGRINE,
        "lancer" => &LANCER,
        "aerolink" => &AEROLINK,
        "meridian190" => &MERIDIAN190,
        "meridian320" => &MERIDIAN320,
        "hercules" => &HERCULES,
        "galaxy" => &GALAXY,
        "vulcan" => &VULCAN,
        "ec130" => &EC130,
        "fpv" => &FPV,
        "triple7" => &TRIPLE7,
        "crj700" => &CRJ700,
        "citationx" => &CITATIONX,
        "concorde" => &CONCORDE,
        "extra500" => &EXTRA500,
        "cub" => &CUB,
        "zlin" => &ZLIN,
        "otter" => &OTTER,
        "dc3" => &DC3,
        _ => &KESTREL,
    }
}

impl StructDef {
    /// Ultimate load factors (+, −).
    pub fn n_ult(&self) -> (f32, f32) {
        (self.n_lim * self.ult, self.n_lim_neg * self.ult)
    }
    /// Flutter onset (kt IAS): 15 % above the dive speed.
    pub fn v_flutter_kt(&self) -> f32 {
        self.vd_kt * 1.15
    }
    /// Maximum speed for a flap position (0..1 of full travel); None = no limit.
    pub fn vfe_for(&self, def: &AircraftDef, flaps: f32) -> Option<f32> {
        if self.vfe_kt.is_empty() || flaps < 0.02 {
            return None;
        }
        let full = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
        // The detent at or above the current position sets the limit.
        let mut lim = None;
        for (i, d) in def.flap_detents.iter().enumerate() {
            let v = self.vfe_kt.get(i).copied().unwrap_or(0.0);
            if v > 0.0 {
                lim = Some(v);
            }
            if *d / full >= flaps - 0.02 && i > 0 {
                return lim;
            }
        }
        lim
    }
    /// Mass of a part (kg) for an aircraft of total mass `m`; the fuselage
    /// gets whatever the others leave.
    pub fn part_mass(&self, def: &AircraftDef, p: Part) -> f32 {
        let m = def.mass;
        match p {
            Part::WingL | Part::WingR => self.m_wing * m,
            Part::HTailL | Part::HTailR => self.m_htail * m,
            Part::Fin => self.m_fin * m,
            Part::GearL | Part::GearR => self.m_gear * m,
            Part::GearNose => self.m_gear * 0.4 * m,
            Part::Nose => self.m_nose * m,
            Part::Aft => self.m_aft * m,
            Part::Eng0 | Part::Eng1 | Part::Eng2 | Part::Eng3 => {
                let i = p.engine_index().unwrap_or(0);
                if i < def.engine_count.min(def.engine_pos.len()) { self.m_engine } else { 0.0 }
            }
            Part::Spare => 0.0,
            Part::Fuselage => {
                let others: f32 = Part::ALL.iter().filter(|q| **q != Part::Fuselage).map(|q| self.part_mass(def, *q)).sum();
                (m - others).max(m * 0.2)
            }
        }
    }
    /// Approximate centre of mass of a part (body frame).
    pub fn part_centre(&self, def: &AircraftDef, p: Part) -> Vec3 {
        let semi = def.wingspan * 0.5;
        let wz = 0.5 * (self.wing_z.0 + self.wing_z.1);
        match p {
            Part::WingL => Vec3::new(self.fus_r + 0.38 * (semi - self.fus_r), def.wing_height, wz),
            Part::WingR => Vec3::new(-(self.fus_r + 0.38 * (semi - self.fus_r)), def.wing_height, wz),
            Part::HTailL => self.ht_pos,
            Part::HTailR => Vec3::new(-self.ht_pos.x, self.ht_pos.y, self.ht_pos.z),
            Part::Fin => self.vt_pos,
            Part::Eng0 | Part::Eng1 | Part::Eng2 | Part::Eng3 => def.engine_pos.get(p.engine_index().unwrap_or(0)).copied().unwrap_or(Vec3::ZERO),
            Part::GearNose => gear_points(def, p).map(|g| g.pos).next().unwrap_or(Vec3::new(0.0, -def.gear_height, 2.0)) * Vec3::new(1.0, 0.7, 1.0),
            Part::GearL | Part::GearR => gear_points(def, p).map(|g| g.pos).next().unwrap_or(Vec3::ZERO) * Vec3::new(1.0, 0.7, 1.0),
            Part::Nose => Vec3::new(0.0, 0.0, self.nose_z + 2.0),
            Part::Aft => Vec3::new(0.0, 0.3, 0.5 * (self.aft_z + self.tail_z) - 1.0),
            Part::Fuselage | Part::Spare => Vec3::ZERO,
        }
    }
}

/// Which separable gear part a gear contact belongs to (None = never fails:
/// skids, tail wheels, glider wheels).
pub fn gear_part(def: &AircraftDef, g: &GearPoint) -> Option<Part> {
    if def.category == crate::flight::Category::Glider || !matches!(g.kind, ContactKind::Wheel { .. }) {
        return None;
    }
    if g.pos.x > 0.1 {
        Some(Part::GearL)
    } else if g.pos.x < -0.1 {
        Some(Part::GearR)
    } else if g.pos.z > 0.0 {
        Some(Part::GearNose)
    } else {
        None
    }
}

fn gear_points<'a>(def: &'a AircraftDef, p: Part) -> impl Iterator<Item = &'a GearPoint> + 'a {
    def.gear.iter().filter(move |g| gear_part(def, g) == Some(p))
}

/// How bad an accident is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Damage (gear-up landing, gear collapse, nose-over): everyone walks away.
    #[default]
    Minor,
    /// Heavy damage, still survivable (wingtip strike, hard landing, ditching).
    Substantial,
    /// The airframe is destroyed.
    Destroyed,
}

impl Severity {
    pub fn survivable(self) -> bool {
        self != Severity::Destroyed
    }
}

/// One structural event this step (read and cleared by the app).
#[derive(Clone, Debug)]
pub struct DamageEvent {
    /// A part tore off (None: a message only).
    pub part: Option<Part>,
    pub msg: String,
    /// Caution (true) or warning level.
    pub caution: bool,
}

/// Live structural state (in `flight::Aircraft`).
#[derive(Clone, Debug, Default)]
pub struct Damage {
    /// Separated parts (bit per [`Part`]).
    pub lost: u16,
    /// Plastic strain per part, 0 (none) .. 1 (at ultimate).
    pub strain: [f32; N_PARTS],
    /// Permanent set of each wing tip (m, + = up): [left, right].
    pub set: [f32; 2],
    /// Wing-bending modal state: tip deflection (m, + = up) and rate, [left, right].
    pub flex: [f32; 2],
    pub flex_rate: [f32; 2],
    /// Filtered load factor carried by each half-wing.
    pub n_wing: [f32; 2],
    /// Flutter amplitude (1 = failure) and phase.
    pub flutter: f32,
    pub flutter_phase: f32,
    /// Jammed flap / spoiler positions.
    pub flaps_jam: Option<f32>,
    pub spoilers_jam: Option<f32>,
    /// Gear stuck (doors damaged).
    pub gear_jam: bool,
    /// The first catastrophic failure (becomes the crash reason).
    pub failure: Option<String>,
    /// Events since the app last looked.
    pub events: Vec<DamageEvent>,
    /// Live caution / warning states (for annunciators).
    pub overstress: bool,
    pub overspeed: bool,
    pub flap_overspeed: bool,
    pub gear_overspeed: bool,
    /// Lost mass (kg) and its moment Σ m r (body frame).
    pub mass_lost: f32,
    pub moment_lost: Vec3,
    /// Lost moments of inertia (pitch, yaw, roll).
    pub inertia_lost: Vec3,
    /// Per-flight scatter of the ultimate strength (seeded).
    pub seed: u32,
    /// Side load on the gear (g, filtered).
    pub side_f: f32,
    /// Seconds since a wing failed.
    pub since_fail: f32,
}

fn hash01(n: u32) -> f32 {
    let mut x = n.wrapping_mul(0x9E37_79B9) ^ 0x85EB_CA6B;
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    (x >> 8) as f32 / 16_777_216.0
}

/// Inputs for one structural update (from `flight::Aircraft::step`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Loads {
    /// Aerodynamic lift (N, + = up in the body frame) and rolling moment
    /// (N·m, + = right wing down, without the rate damping).
    pub lift: f32,
    pub roll_moment: f32,
    /// Total load factor along body up (with ground forces).
    pub n_total: f32,
    /// Lateral load factor.
    pub side_g: f32,
    pub ias: f32,
    pub mach: f32,
    pub flaps: f32,
    pub spoilers: f32,
    pub gear_pos: f32,
    pub gear_moving: bool,
    pub on_ground: bool,
    pub ground_speed: f32,
    /// Dynamic pressure × wing area (N) and the maximum lift coefficient:
    /// a half-wing can't carry more than its stalled lift.
    pub q_s: f32,
    pub cl_max: f32,
    pub dt: f32,
}

impl Damage {
    pub fn new(seed: u32) -> Damage {
        Damage { seed, n_wing: [1.0; 2], since_fail: 99.0, ..Default::default() }
    }
    pub fn is_lost(&self, p: Part) -> bool {
        self.lost & p.bit() != 0
    }
    /// Engines that can no longer run (separated).
    pub fn failed_engine_bits(&self) -> u8 {
        (0..4).filter(|&i| self.is_lost(Part::engine(i))).fold(0u8, |b, i| b | (1 << i))
    }
    pub fn gear_lost(&self, def: &AircraftDef, g: &GearPoint) -> bool {
        gear_part(def, g).is_some_and(|p| self.is_lost(p))
    }
    /// Anything wrong with the airframe?
    pub fn any(&self) -> bool {
        self.lost != 0 || self.strain.iter().any(|s| *s > 0.0) || self.flaps_jam.is_some() || self.spoilers_jam.is_some() || self.gear_jam
    }

    fn note(&mut self, part: Option<Part>, msg: String, caution: bool) {
        self.events.push(DamageEvent { part, msg, caution });
    }

    /// Tear a part (and whatever hangs on it) off the airframe.
    pub fn separate(&mut self, sd: &StructDef, def: &AircraftDef, p: Part, why: &str) {
        if self.is_lost(p) || p == Part::Fuselage {
            return;
        }
        let mut set = p.bit();
        // Children follow their parent.
        loop {
            let before = set;
            for q in Part::ALL {
                if let Some(par) = q.parent(sd, def) {
                    if set & par.bit() != 0 && self.lost & q.bit() == 0 {
                        set |= q.bit();
                    }
                }
            }
            if set == before {
                break;
            }
        }
        set &= !self.lost;
        for q in Part::ALL {
            if set & q.bit() == 0 {
                continue;
            }
            let m = sd.part_mass(def, q);
            if m <= 0.0 && q != p {
                set &= !q.bit();
                continue;
            }
            let r = sd.part_centre(def, q);
            self.mass_lost += m;
            self.moment_lost += r * m;
            self.inertia_lost += m * Vec3::new(r.y * r.y + r.z * r.z, r.x * r.x + r.z * r.z, r.x * r.x + r.y * r.y);
        }
        self.lost |= set;
        let msg = format!("{} separated{}", cap(p.name()), if why.is_empty() { String::new() } else { format!(" — {why}") });
        self.note(Some(p), msg, false);
    }

    fn fail(&mut self, sd: &StructDef, def: &AircraftDef, p: Part, why: String) {
        if self.failure.is_none() {
            self.failure = Some(format!("Structural failure: {why}"));
        }
        self.separate(sd, def, p, &why);
    }

    /// Effective mass (kg) and inertia (pitch, yaw, roll) after lost parts.
    pub fn mass(&self, def: &AircraftDef) -> f32 {
        (def.mass - self.mass_lost).max(def.mass * 0.15)
    }
    pub fn inertia(&self, def: &AircraftDef) -> Vec3 {
        let i0 = Vec3::new(def.inertia_pitch, def.inertia_yaw, def.inertia_roll);
        (i0 - self.inertia_lost).max(i0 * 0.2)
    }
    /// Centre-of-gravity shift (body frame) from lost mass.
    pub fn cg_shift(&self, def: &AircraftDef) -> Vec3 {
        if self.mass_lost <= 0.0 {
            return Vec3::ZERO;
        }
        -self.moment_lost / self.mass(def)
    }

    /// Jammed flaps / spoilers override the actuators.
    pub fn apply_jams(&self, s: &mut Surfaces) {
        if let Some(f) = self.flaps_jam {
            s.flaps = f;
        }
        if let Some(f) = self.spoilers_jam {
            s.spoilers = f;
        }
    }

    /// One structural step: wing bending, strain, flutter, overspeeds, failures.
    pub fn update(&mut self, sd: &StructDef, def: &AircraftDef, l: &Loads) {
        let dt = l.dt.clamp(0.0, 0.1);
        if dt <= 0.0 {
            return;
        }
        let weight = def.mass * crate::flight::GRAVITY;
        let (n_ult, n_ult_neg) = sd.n_ult();
        // ---- per-wing load: the rolling moment shifts lift between the halves.
        let yc = sd.wing_yc.max(0.5);
        // (Half of the aero rolling moment is taken as span-load asymmetry; the
        // rest comes from the tail, fuselage and the aileron near the tip.)
        let d_lift = (0.5 * l.roll_moment / (2.0 * yc)).clamp(-2.0 * weight, 2.0 * weight);
        let mut half = [0.5 * l.lift + d_lift, 0.5 * l.lift - d_lift];
        // With one half-wing gone, the other carries what lift is left.
        if self.is_lost(Part::WingL) {
            half = [0.0, l.lift];
        } else if self.is_lost(Part::WingR) {
            half = [l.lift, 0.0];
        }
        if l.q_s > 0.0 && l.cl_max > 0.0 {
            let cap = 0.5 * l.q_s * l.cl_max * 1.15;
            for h in &mut half {
                *h = h.clamp(-0.8 * cap, cap);
            }
        }
        let k = 1.0 - (-dt / 0.08f32).exp();
        // Sustained side load on the ground (gear legs).
        if l.on_ground {
            self.side_f += (l.side_g - self.side_f) * (1.0 - (-dt / 0.4f32).exp());
        } else {
            self.side_f = 0.0;
        }
        for s in 0..2 {
            let n = half[s] / (0.5 * weight);
            let n = if n.is_finite() { n.clamp(-40.0, 40.0) } else { 0.0 };
            self.n_wing[s] += (n - self.n_wing[s]) * k;
        }
        // ---- first bending mode per half-wing.
        let w = std::f32::consts::TAU * sd.f_bend;
        let wing = [Part::WingL, Part::WingR];
        for s in 0..2 {
            if self.is_lost(wing[s]) {
                self.flex[s] = 0.0;
                self.flex_rate[s] = 0.0;
                continue;
            }
            // Net bending load: lift minus the wing's own inertia (relief); the
            // model is drawn in the parked shape (net = −relief on the ground).
            let net = self.n_wing[s] - sd.relief * l.n_total;
            let target = sd.tip_per_g * (net + sd.relief) + self.set[s];
            let a = w * w * (target - self.flex[s]) - 2.0 * sd.zeta * w * self.flex_rate[s];
            self.flex_rate[s] += a * dt;
            self.flex[s] += self.flex_rate[s] * dt;
            if !self.flex[s].is_finite() || !self.flex_rate[s].is_finite() {
                self.flex[s] = target;
                self.flex_rate[s] = 0.0;
            }
            let lim = sd.tip_per_g * 30.0 + 0.5;
            self.flex[s] = self.flex[s].clamp(-lim, lim);
        }
        // ---- limit / ultimate load per half-wing (one failure at a time: the
        // first wing to go unloads the other).
        self.overstress = false;
        self.since_fail += dt;
        for s in 0..2 {
            let p = wing[s];
            if self.is_lost(p) || l.on_ground {
                continue;
            }
            let n = self.n_wing[s];
            // Real structures meet ultimate with a margin: 0..6 % stronger.
            let scatter = 1.0 + 0.06 * hash01(self.seed.wrapping_add(s as u32 * 7919));
            let (lim, ult) = if n >= 0.0 { (sd.n_lim, n_ult * scatter) } else { (sd.n_lim_neg, n_ult_neg * scatter) };
            let r = n / lim;
            if r > 1.0 {
                self.overstress = true;
                let e = ((n - lim) / (ult - lim)).clamp(0.0, 1.0);
                if e > self.strain[p.idx()] + 0.02 {
                    if self.strain[p.idx()] == 0.0 {
                        self.note(None, format!("Overstress {:+.1} g — {} damaged", n, p.name()), true);
                    }
                    self.strain[p.idx()] = e;
                    // Permanent set: a fraction of the elastic deflection at limit.
                    self.set[s] = 0.25 * e * sd.tip_per_g * lim.abs() * n.signum() + 0.02 * e * n.signum();
                    if e > 0.55 && self.spoilers_jam.is_none() && def.has_spoilers {
                        self.spoilers_jam = Some(0.0);
                        self.note(None, "Spoilers jammed (wing distortion)".into(), true);
                    }
                }
                let ult = if self.since_fail < 1.0 { ult * 1.3 } else { ult };
                if (n >= 0.0 && n >= ult) || (n < 0.0 && n <= ult) {
                    let side = if s == 0 { "left" } else { "right" };
                    self.fail(sd, def, p, format!("wings overstressed at {:+.1} g — {side} wing separated", n));
                    self.since_fail = 0.0;
                    break;
                }
            }
        }
        // Engines hang on pylons designed for ~1.5 x limit inertia loads.
        if !l.on_ground {
            for i in 0..def.engine_count.min(4) {
                let p = Part::engine(i);
                let wing_mounted = def.engine_pos.get(i).is_some_and(|e| e.x.abs() > sd.fus_r);
                if !wing_mounted || self.is_lost(p) {
                    continue;
                }
                if l.n_total.abs() > n_ult * 1.2 || l.side_g.abs() > 4.5 {
                    self.separate(sd, def, p, "pylon overload");
                }
            }
        }
        // ---- overspeed: flutter beyond the dive speed / Mach.
        let ias_kt = l.ias / KT;
        let vf = sd.v_flutter_kt();
        let mut excess = (ias_kt / vf - 1.0).max(0.0);
        if sd.md > 0.0 {
            // Flutter clearance extends to MD + 0.05.
            excess = excess.max(l.mach / (sd.md + 0.05) - 1.0);
        }
        self.overspeed = ias_kt > sd.vd_kt || (sd.md > 0.0 && l.mach > sd.md);
        if excess > 0.0 && !l.on_ground {
            // Divergent oscillation: grows faster the further past onset.
            let sigma = 3.0 + 40.0 * excess;
            self.flutter += (sigma * self.flutter + 0.05) * dt;
        } else {
            self.flutter = (self.flutter - 0.8 * dt).max(0.0) * (1.0 - (2.0 * dt).min(1.0));
        }
        self.flutter_phase = (self.flutter_phase + dt * sd.f_bend * 3.1 * std::f32::consts::TAU).rem_euclid(std::f32::consts::TAU);
        if self.flutter >= 1.0 {
            self.flutter = 0.0;
            let what = if l.mach > 0.6 { format!("overspeed at Mach {:.2}", l.mach) } else { format!("overspeed at {:.0} kt", ias_kt) };
            if sd.flutter_wing {
                let p = if hash01(self.seed ^ 0xABC) < 0.5 { Part::WingL } else { Part::WingR };
                if !self.is_lost(p) {
                    self.fail(sd, def, p, format!("wing flutter, {what}"));
                }
            } else if !self.is_lost(Part::HTailL) || !self.is_lost(Part::HTailR) {
                let why = format!("tail flutter, {what}");
                if self.failure.is_none() {
                    self.failure = Some(format!("Structural failure: {why}"));
                }
                self.separate(sd, def, Part::HTailL, &why);
                self.separate(sd, def, Part::HTailR, &why);
            }
        }
        // ---- flaps beyond their limit speed.
        self.flap_overspeed = false;
        if let Some(vfe) = sd.vfe_for(def, l.flaps) {
            if ias_kt > vfe && !l.on_ground {
                self.flap_overspeed = true;
                if self.flaps_jam.is_none() && ias_kt > vfe * 1.25 {
                    self.flaps_jam = Some(l.flaps);
                    self.note(None, format!("Flaps jammed — flap overspeed at {ias_kt:.0} kt (limit {vfe:.0} kt)"), true);
                }
                if ias_kt > vfe * 1.55 && self.flaps_jam.is_some_and(|f| f > 0.0) {
                    self.flaps_jam = Some(0.0);
                    self.note(None, "Flaps torn off".into(), false);
                }
            }
        }
        // ---- landing gear beyond its limit speeds.
        self.gear_overspeed = false;
        if def.retractable_gear && l.gear_pos > 0.05 && self.lost & GEAR_BITS != GEAR_BITS {
            if ias_kt > def.vle_kt && !l.on_ground {
                self.gear_overspeed = true;
            }
            if l.gear_moving && sd.vlo_kt > 0.0 && ias_kt > sd.vlo_kt * 1.15 && !self.gear_jam {
                self.gear_jam = true;
                self.note(None, format!("Gear doors damaged — gear operated at {ias_kt:.0} kt"), true);
            }
            if ias_kt > 1.35 * def.vle_kt && !l.on_ground {
                let why = format!("landing gear extended at {ias_kt:.0} kt (limit {:.0} kt)", def.vle_kt);
                for p in [Part::GearNose, Part::GearL, Part::GearR] {
                    if gear_points(def, p).next().is_some() {
                        self.separate(sd, def, p, &why);
                    }
                }
                self.note(None, format!("Landing gear torn off — {why}"), false);
            }
        }
    }

    /// Deflection of the wing tips for rendering (m, + up): the modal state
    /// plus visible flutter.
    pub fn tip_deflection(&self, sd: &StructDef) -> [f32; 2] {
        let fl = self.flutter.min(1.0) * (0.25 * sd.tip_per_g + 0.05) * self.flutter_phase.sin();
        [self.flex[0] + fl, self.flex[1] - fl]
    }

    /// Annunciator items: (text, warning (red) / caution (amber)).
    pub fn annunciations(&self) -> Vec<(String, bool)> {
        let mut v = Vec::new();
        if self.overstress {
            v.push(("OVERSTRESS".to_string(), true));
        }
        if self.overspeed {
            v.push(("OVERSPEED".to_string(), true));
        }
        if self.flap_overspeed {
            v.push(("FLAP OVSPD".to_string(), false));
        }
        if self.gear_overspeed {
            v.push(("GEAR OVSPD".to_string(), false));
        }
        let wings = self.strain[Part::WingL.idx()].max(self.strain[Part::WingR.idx()]);
        if self.lost & (Part::WingL.bit() | Part::WingR.bit()) != 0 {
            v.push(("WING LOST".to_string(), true));
        } else if wings > 0.0 {
            v.push(("WING DAMAGE".to_string(), false));
        }
        if self.lost & (Part::HTailL.bit() | Part::HTailR.bit() | Part::Fin.bit()) != 0 {
            v.push(("TAIL LOST".to_string(), true));
        }
        if self.lost & (Part::Eng0.bit() | Part::Eng1.bit() | Part::Eng2.bit() | Part::Eng3.bit()) != 0 {
            v.push(("ENG SEPARATED".to_string(), true));
        }
        if self.lost & GEAR_BITS != 0 {
            v.push(("GEAR LOST".to_string(), true));
        } else if self.gear_jam {
            v.push(("GEAR DOORS".to_string(), false));
        }
        if self.flaps_jam.is_some() {
            v.push(("FLAPS JAM".to_string(), false));
        }
        if self.spoilers_jam.is_some() {
            v.push(("SPLR JAM".to_string(), false));
        }
        v
    }
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Aerodynamic change from lost / damaged surfaces (body frame).
#[derive(Clone, Copy, Debug)]
pub struct AeroCorr {
    pub force: Vec3,
    pub moment: Vec3,
    /// Multiplier on the rate damping (pitch, yaw, roll).
    pub damp_scale: Vec3,
}

impl Default for AeroCorr {
    fn default() -> Self {
        AeroCorr { force: Vec3::ZERO, moment: Vec3::ZERO, damp_scale: Vec3::ONE }
    }
}

/// The aerodynamic effect of the damage, on top of the intact aircraft's
/// aero force / moment (either model). `vb` = air-relative velocity (body).
#[allow(clippy::too_many_arguments)]
pub fn aero_correction(sd: &StructDef, def: &AircraftDef, d: &Damage, vb: Vec3, rho: f32, force: Vec3, surf: &Surfaces) -> AeroCorr {
    let mut out = AeroCorr::default();
    let bits = Part::WingL.bit() | Part::WingR.bit() | Part::HTailL.bit() | Part::HTailR.bit() | Part::Fin.bit();
    let strained = d.strain[Part::WingL.idx()] > 0.0 || d.strain[Part::WingR.idx()] > 0.0;
    if d.lost & bits == 0 && !strained && d.mass_lost <= 0.0 {
        return out;
    }
    let v2 = vb.length_squared();
    let v = v2.sqrt();
    if v < 1.0 || !rho.is_finite() {
        return out;
    }
    let q = 0.5 * rho * v2;
    let s = def.wing_area;
    let vs2 = vb.y * vb.y + vb.z * vb.z;
    let lift_dir = if vs2 > 1e-4 { Vec3::new(0.0, vb.z, -vb.y) / vs2.sqrt() } else { Vec3::Y };
    let drag_dir = -vb / v;
    let lift = force.dot(lift_dir);
    let drag = force.dot(drag_dir).max(0.0);
    let alpha = if vs2 > 1e-4 { (-vb.y).atan2(vb.z) } else { 0.0 };
    // ---- wings
    let semi = def.wingspan * 0.5;
    let yc = sd.wing_yc.min(semi);
    let wing = [(Part::WingL, 1.0f32), (Part::WingR, -1.0f32)];
    let mut roll_damp: f32 = 1.0;
    for (p, side) in wing {
        let (f_lift, f_ail) = if d.is_lost(p) {
            (1.0, 1.0)
        } else {
            let e = d.strain[p.idx()];
            (0.06 * e, 0.5 * e)
        };
        if f_lift <= 0.0 && f_ail <= 0.0 {
            continue;
        }
        let r = Vec3::new(side * yc, def.wing_height, 0.0);
        // ~90 % of the lift is the wings', and half the drag in normal
        // flight, most of it once the wing is stalled flat to the flow.
        let drag_share = 0.5 + 0.42 * ((alpha.abs() - 0.25) / 0.55).clamp(0.0, 1.0);
        let df = -f_lift * (0.5 * 0.9 * lift * lift_dir + 0.5 * drag_share * drag * drag_dir);
        out.force += df;
        out.moment += r.cross(df);
        // This half's share of the aileron rolling moment.
        let ail = q * s * def.wingspan * def.cl_aileron * surf.aileron;
        out.moment.z -= f_ail * 0.5 * ail;
        if d.is_lost(p) {
            roll_damp -= 0.5;
        }
    }
    out.damp_scale.z = roll_damp.max(0.05);
    // ---- horizontal tail (estimated from its area and the local flow)
    let a_t = 3.5;
    let alpha_t = 0.55 * alpha - 0.45 * 0.44 * surf.elevator - 0.04;
    let f_tail = q * s * sd.ht_area * a_t * alpha_t.clamp(-0.5, 0.5);
    let mut pitch_damp: f32 = 1.0;
    for (p, side) in [(Part::HTailL, 1.0f32), (Part::HTailR, -1.0f32)] {
        if !d.is_lost(p) {
            continue;
        }
        let r = Vec3::new(side * sd.ht_pos.x, sd.ht_pos.y, sd.ht_pos.z);
        let df = -0.5 * f_tail * lift_dir;
        out.force += df;
        out.moment += r.cross(df);
        pitch_damp -= 0.42;
    }
    out.damp_scale.x = pitch_damp.max(0.1);
    // ---- fin
    if d.is_lost(Part::Fin) {
        let a_v = 3.0;
        let f_fin = q * s * sd.vt_area * a_v * (-0.8 * vb.x / v + 0.2 * surf.rudder).clamp(-0.6, 0.6);
        let df = Vec3::new(-f_fin, 0.0, 0.0);
        out.force += df;
        out.moment += sd.vt_pos.cross(df);
        out.damp_scale.y = 0.25;
    }
    // ---- the centre of gravity moved: every aero force now acts about it.
    let shift = d.cg_shift(def);
    if shift.length_squared() > 1e-8 {
        out.moment += (-shift).cross(force + out.force);
    }
    out
}

/// Severity of an accident from the impact state.
pub fn classify(def: &AircraftDef, reason: &str, vel: Vec3, structural_failure: bool) -> Severity {
    let r = reason.to_ascii_lowercase();
    let sink = (-vel.y).max(0.0);
    let speed = vel.length();
    let vref = def.vref_kt.max(def.vs1_kt * 1.3).max(30.0) * KT;
    let max_sink = def.max_touchdown_sink.max(1.0);
    if structural_failure || r.contains("structural") || r.contains("inverted") || r.contains("broke up") {
        return Severity::Destroyed;
    }
    if r.contains("building") {
        return if speed > 12.0 { Severity::Destroyed } else { Severity::Substantial };
    }
    if sink > 2.4 * max_sink || speed > 1.9 * vref {
        return Severity::Destroyed;
    }
    if r.contains("flew into") {
        // Flying into the ground at a shallow angle and low speed is a crash landing.
        return if sink > 1.5 * max_sink || speed > 1.4 * vref { Severity::Destroyed } else { Severity::Substantial };
    }
    if r.contains("water") || r.contains("ditch") {
        return if sink > 1.6 * max_sink || speed > 1.5 * vref { Severity::Destroyed } else { Severity::Substantial };
    }
    if r.contains("wingtip") || r.contains("pod strike") || r.contains("hard landing") || r.contains("ground loop") {
        return if speed > 1.5 * vref { Severity::Destroyed } else { Severity::Substantial };
    }
    Severity::Minor
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::{catalogue, find};

    #[test]
    fn limit_table() {
        for def in catalogue() {
            let sd = struct_def(def.id);
            let (u, un) = sd.n_ult();
            assert!(sd.n_lim > 2.0 && sd.n_lim_neg < -0.9, "{}", def.id);
            assert!((u / sd.n_lim - 1.5).abs() < 1e-3 && (un / sd.n_lim_neg - 1.5).abs() < 1e-3);
            assert!(sd.vd_kt > def.vne_kt, "{}: VD {} must exceed Vne {}", def.id, sd.vd_kt, def.vne_kt);
            // Masses add up and stay physical.
            let total: f32 = Part::ALL.iter().map(|p| sd.part_mass(&def, *p)).sum();
            assert!((total - def.mass).abs() < def.mass * 0.05, "{} {total} vs {}", def.id, def.mass);
            assert!(sd.part_mass(&def, Part::Fuselage) > 0.2 * def.mass);
        }
        let a320 = struct_def("meridian320");
        assert_eq!((a320.n_lim, a320.n_lim_neg), (2.5, -1.0));
        let c172 = struct_def("kestrel");
        assert_eq!((c172.n_lim, c172.n_lim_neg), (3.8, -1.52));
        let f16 = struct_def("lancer");
        assert_eq!((f16.n_lim, f16.n_lim_neg), (9.0, -3.0));
        let ask = struct_def("swift");
        assert_eq!((ask.n_lim, ask.n_lim_neg), (5.3, -2.65));
    }

    #[test]
    fn vfe_lookup() {
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        assert_eq!(sd.vfe_for(&def, 0.0), None);
        // Full flap (40°): 177 kt.
        assert_eq!(sd.vfe_for(&def, 1.0), Some(177.0));
        // Flap 1 (10° of 40): 230 kt.
        assert_eq!(sd.vfe_for(&def, 0.25), Some(230.0));
    }

    fn steady(d: &mut Damage, sd: &StructDef, def: &AircraftDef, n: f32, roll: f32, secs: f32) {
        let w = def.mass * crate::flight::GRAVITY;
        let l = Loads { lift: n * w, roll_moment: roll, n_total: n, ias: def.vno_kt * 0.8 * KT, dt: 1.0 / 120.0, ..Default::default() };
        for _ in 0..(secs * 120.0) as usize {
            d.update(sd, def, &l);
        }
    }

    #[test]
    fn wing_flex_magnitudes() {
        // Tip deflection at 1 g and 2.5 g (relative to the parked shape).
        for (id, n, lo, hi) in [("meridian320", 2.5, 2.2, 3.0), ("swift", 5.0, 0.9, 1.6), ("lancer", 7.0, 0.0, 0.12)] {
            let def = find(id).unwrap();
            let sd = struct_def(id);
            let mut d = Damage::new(1);
            steady(&mut d, sd, &def, n, 0.0, 6.0);
            eprintln!("{id}: tip {:.2} m at {n} g", d.flex[0]);
            assert!(d.flex[0] > lo && d.flex[0] < hi, "{id} {}", d.flex[0]);
            assert!((d.flex[0] - d.flex[1]).abs() < 1e-4);
            assert_eq!(d.lost, 0);
        }
        // Parked: no deflection.
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        let mut d = Damage::new(1);
        let l = Loads { lift: 0.0, n_total: 1.0, on_ground: true, dt: 1.0 / 120.0, ..Default::default() };
        for _ in 0..4000 {
            d.update(sd, &def, &l);
        }
        assert!(d.flex[0].abs() < 0.01, "{}", d.flex[0]);
    }

    #[test]
    fn beyond_limit_is_permanent_beyond_ultimate_separates() {
        let def = find("kestrel").unwrap();
        let sd = struct_def(def.id);
        // 4.5 g: past limit (3.8), short of ultimate (5.7): permanent set, no failure.
        let mut d = Damage::new(3);
        steady(&mut d, sd, &def, 4.5, 0.0, 1.0);
        assert!(d.overstress && d.strain[Part::WingL.idx()] > 0.2 && d.lost == 0);
        assert!(d.set[0] > 0.0 && d.failure.is_none());
        steady(&mut d, sd, &def, 1.0, 0.0, 3.0);
        assert!(!d.overstress && d.strain[Part::WingL.idx()] > 0.2, "plastic strain stays");
        // 6.5 g: ultimate exceeded, a wing goes.
        steady(&mut d, sd, &def, 6.5, 0.0, 1.0);
        assert!(d.is_lost(Part::WingL) || d.is_lost(Part::WingR));
        assert!(d.failure.as_deref().unwrap().contains("overstressed"));
        assert!(d.mass_lost > 0.05 * def.mass);
    }

    #[test]
    fn rolling_pull_fails_the_loaded_wing() {
        // Rolling pull: lift shifted to the left wing (positive roll moment).
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        let mut d = Damage::new(5);
        let w = def.mass * crate::flight::GRAVITY;
        // 3.3 g with 25 % of a half-wing's lift moved left: left ≈ 4.1 g.
        // After the first failure the aircraft's lift drops to what the other wing makes.
        for _ in 0..120 {
            let lost = d.is_lost(Part::WingL);
            let lift = if lost { 0.5 * 3.3 * w } else { 3.3 * w };
            let roll = if lost { 0.0 } else { 2.0 * 0.25 * 0.5 * 3.3 * w * 2.0 * sd.wing_yc };
            let l = Loads { lift, roll_moment: roll, n_total: 3.3, ias: def.vno_kt * 0.8 * KT, dt: 1.0 / 120.0, ..Default::default() };
            d.update(sd, &def, &l);
        }
        assert!(d.is_lost(Part::WingL), "{:?}", d.n_wing);
        assert!(!d.is_lost(Part::WingR));
        // The left engine hangs on the left wing: gone with it.
        assert!(d.is_lost(Part::Eng0) && !d.is_lost(Part::Eng1));
        assert_eq!(d.failed_engine_bits(), 1);
    }

    #[test]
    fn lost_wing_rolls_toward_it_and_lost_tail_destabilises() {
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        let vb = Vec3::new(0.0, -0.05 * 120.0, 120.0);
        let lift = def.mass * crate::flight::GRAVITY;
        let force = Vec3::new(0.0, lift, -lift / 15.0);
        let mut d = Damage::new(1);
        d.separate(sd, &def, Part::WingL, "test");
        let c = aero_correction(sd, &def, &d, vb, 1.0, force, &Surfaces::default());
        // Lift lost on the left: the aircraft rolls left (negative = left wing down).
        assert!(c.moment.z < -0.2 * lift * sd.wing_yc, "{:?}", c.moment);
        assert!(c.force.y < -0.3 * lift);
        assert!(c.damp_scale.z < 0.6);
        // Tail gone: less pitch damping and a pitching moment.
        let mut d = Damage::new(1);
        d.separate(sd, &def, Part::HTailL, "t");
        d.separate(sd, &def, Part::HTailR, "t");
        let c = aero_correction(sd, &def, &d, vb, 1.0, force, &Surfaces::default());
        assert!(c.damp_scale.x < 0.3);
        assert!(c.moment.x.abs() > 1000.0 && c.moment.x.is_finite());
        // Intact: no change.
        let c = aero_correction(sd, &def, &Damage::new(1), vb, 1.0, force, &Surfaces::default());
        assert_eq!(c.force, Vec3::ZERO);
    }

    #[test]
    fn flutter_beyond_dive_speed() {
        let def = find("kestrel").unwrap();
        let sd = struct_def(def.id);
        let mut d = Damage::new(9);
        let w = def.mass * crate::flight::GRAVITY;
        let mut l = Loads { lift: w, n_total: 1.0, ias: sd.vd_kt * 1.05 * KT, dt: 1.0 / 120.0, ..Default::default() };
        for _ in 0..1200 {
            d.update(sd, &def, &l);
        }
        assert!(d.lost == 0 && d.overspeed, "at VD: overspeed warning, no flutter");
        l.ias = sd.v_flutter_kt() * 1.1 * KT;
        for _ in 0..1200 {
            d.update(sd, &def, &l);
        }
        assert!(d.is_lost(Part::HTailL) && d.is_lost(Part::HTailR));
        assert!(d.failure.as_deref().unwrap().contains("overspeed"));
    }

    #[test]
    fn gear_and_flap_overspeed() {
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        let mut d = Damage::new(1);
        let w = def.mass * crate::flight::GRAVITY;
        let l = Loads { lift: w, n_total: 1.0, ias: 240.0 * KT, flaps: 1.0, gear_pos: 1.0, dt: 1.0 / 120.0, ..Default::default() };
        d.update(sd, &def, &l);
        assert_eq!(d.flaps_jam, Some(1.0));
        assert!(d.flap_overspeed && !d.gear_overspeed);
        let l = Loads { ias: 300.0 * KT, ..l };
        d.update(sd, &def, &l);
        assert!(d.gear_overspeed && d.lost == 0);
        let l = Loads { ias: 400.0 * KT, ..l };
        d.update(sd, &def, &l);
        assert_eq!(d.lost & GEAR_BITS, GEAR_BITS);
        assert_eq!(d.flaps_jam, Some(0.0));
        assert!(d.events.iter().any(|e| e.msg.contains("landing gear")));
    }

    #[test]
    fn severity_classes() {
        let def = find("meridian320").unwrap();
        let v = def.vref_kt * KT;
        assert_eq!(classify(&def, "Gear-up landing", Vec3::new(0.0, -1.0, v), false), Severity::Minor);
        assert!(classify(&def, "Hard landing: touched down at 1100 fpm — landing gear collapsed", Vec3::new(0.0, -5.0, v), false).survivable());
        assert_eq!(classify(&def, "Flew into the ground", Vec3::new(0.0, -40.0, 120.0), false), Severity::Destroyed);
        assert_eq!(classify(&def, "Gear-up landing", Vec3::new(0.0, -1.0, v), true), Severity::Destroyed);
    }
}

#[cfg(test)]
mod probe {
    use crate::flight::*;
    use glam::{Vec2, Vec3};
    struct Flat;
    impl Environment for Flat {
        fn ground_height(&self, _: Vec2) -> f32 { 0.0 }
        fn obstacle_top(&self, _: Vec2) -> Option<f32> { None }
        fn is_water(&self, _: Vec2) -> bool { false }
        fn updraft(&self, _: Vec3) -> f32 { 0.0 }
    }
    #[test]
    #[ignore]
    fn overg_probe() {
        let def = find(&std::env::var("PROBE_AC").unwrap_or("kestrel".into())).unwrap();
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 400.0, 0.0), 0.0, 150.0 * KT);
        let base = trimmed_controls_at(&def, 150.0 * KT, 400.0);
        for k in 0..(8.0 * 120.0) as usize {
            let t = k as f32 / 120.0;
            let c = Controls { elevator: if t > 1.5 { 1.0 } else { 0.0 }, throttle: 1.0, ..base };
            a.step(&def, &c, &Flat, 1.0 / 120.0);
            if k % 30 == 0 {
                println!("t {t:.2} alt {:.0} ias {:.0} pitch {:.0} roll {:.0} g {:.1} nw {:?} lost {:#06x} crashed {:?}", a.pos.y, a.airspeed_kt(), a.pitch().to_degrees(), a.roll().to_degrees(), a.g_load(), a.dmg.n_wing, a.dmg.lost, a.crashed);
            }
        }
    }
}
