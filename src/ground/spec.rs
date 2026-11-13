//! Per-type undercarriage data the tyre / brake / steering model needs, kept
//! here (keyed by aircraft id, with category defaults) so the flight model's
//! `AircraftDef` stays as it is: tyre pressures and sizes, wheels per leg,
//! brake heat sinks (steel / carbon), anti-skid, autobrake levels, and how
//! the nose / tail wheel steers (tiller + pedals, pedal-linked, castering,
//! steerable / lockable tail wheel).
//!
//! Sources: manufacturers' Aircraft Characteristics for Airport Planning
//! (tyre sizes / pressures, steering angles), FCOMs (autobrake rates).

use crate::flight::{AircraftDef, Category, ContactKind, GearPoint};

/// How the steerable wheel steers.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(clippy::enum_variant_names)]
pub enum Steer {
    /// No steering: the wheel keeps straight (glider nose / tail wheels).
    Fixed,
    /// Airliner nose-wheel steering: a tiller (±`tiller_deg`, full authority
    /// below `tiller_full_kt`, gone by `tiller_zero_kt`) and the rudder pedals
    /// (±`pedal_deg`, faded out between `pedal_full_kt` and `pedal_zero_kt`).
    /// Hydraulic; rate-limited.
    Tiller { tiller_deg: f32, pedal_deg: f32, tiller_full_kt: f32, tiller_zero_kt: f32, pedal_full_kt: f32, pedal_zero_kt: f32, rate_dps: f32 },
    /// Pedal steering (light types / fighters): ±`deg` at low speed, fading
    /// as 1 / (1 + (v / `fade_ms`)²).
    Pedal { deg: f32, fade_ms: f32 },
    /// Pedal-linked nose wheel that casters beyond the link (C172: ±10° from
    /// the pedals, castering to ±30° with differential braking).
    Caster { link_deg: f32, free_deg: f32, fade_ms: f32 },
    /// Steerable tail wheel through springs (±`deg`), breaking out to full
    /// castering beyond `breakout_deg` of slip (Cub, Zlin; the Spitfire's
    /// rudder-linked tail wheel).
    TailSteer { deg: f32, breakout_deg: f32, fade_ms: f32 },
    /// Free-castering tail wheel with a lock (DC-3): locked straight for take-off and
    /// landing, free for taxiing (steer with differential brakes and throttles).
    TailCaster { lockable: bool },
}

/// Brake heat-sink material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrakeKind {
    /// Steel / iron discs: fade above ~300 °C.
    Steel,
    /// Carbon-carbon: light, keep working hot, fade only past ~1000 °C.
    Carbon,
    /// Light-aircraft single-disc / drum brakes.
    Light,
}

/// Autobrake decelerations (m/s²) per selector position, and RTO.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutobrakeSpec {
    pub lo: f32,
    pub med: f32,
    pub max: f32,
    /// Delay after the ground spoilers deploy (s): LO, MED / MAX.
    pub delay_lo: f32,
    pub delay_med: f32,
    /// Separate RTO position (Boeing / Embraer); else MAX arms the RTO (Airbus).
    pub rto_position: bool,
    /// RTO activation speed (kt).
    pub rto_kt: f32,
}

/// The type's undercarriage data.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundSpec {
    /// Inflation pressure (psi): main, nose / tail.
    pub psi_main: f32,
    pub psi_aux: f32,
    /// Tyre rolling radius (m): main, nose / tail.
    pub r_main: f32,
    pub r_aux: f32,
    /// Wheels on each main gear contact point (bogies: 2, 4, 6).
    pub wheels_per_leg: u8,
    /// Spin inertia of one wheel + tyre + brake rotors (kg·m²): main, aux.
    pub i_main: f32,
    pub i_aux: f32,
    pub brake: BrakeKind,
    /// Heat-sink mass of one brake (kg).
    pub sink_kg: f32,
    pub antiskid: bool,
    pub autobrake: Option<AutobrakeSpec>,
    pub steer: Steer,
    /// Keyboard / single-lever brakes: the rudder splits the brake between the
    /// sides (warbird brake lever + rudder bar; light types' toe brakes).
    pub rudder_diff_brake: bool,
    /// Wheel (rim) temperature at which the fusible plugs melt and let the tyre down (°C).
    pub fuse_c: f32,
}

impl GroundSpec {
    /// Brake heat-sink heat capacity of one main leg (J/K).
    pub fn leg_heat_capacity(&self) -> f32 {
        // Carbon's specific heat rises steeply with temperature (≈ 0.7 → 1.8 kJ/kg·K);
        // a mean over the working range. Steel ≈ 0.5 kJ/kg·K.
        let cp = match self.brake {
            BrakeKind::Carbon => 1350.0,
            BrakeKind::Steel => 520.0,
            BrakeKind::Light => 480.0,
        };
        cp * self.sink_kg * self.wheels_per_leg as f32
    }

    /// Brake torque fade at heat-sink temperature `c` (1 = full).
    pub fn fade(&self, c: f32) -> f32 {
        let s = |a: f32, b: f32| ((c - a) / (b - a)).clamp(0.0, 1.0);
        match self.brake {
            BrakeKind::Carbon => 1.0 - 0.45 * s(1100.0, 1700.0),
            BrakeKind::Steel => 1.0 - 0.55 * s(320.0, 650.0),
            BrakeKind::Light => 1.0 - 0.6 * s(250.0, 520.0),
        }
    }

    /// Brake torque at full pedal relative to the type's braking coefficient on
    /// its static weight: light single-disc brakes only lock a lightly loaded
    /// wheel (just after touchdown, flaps out), as on the real aircraft.
    pub fn brake_scale(&self) -> f32 {
        match self.brake {
            BrakeKind::Light => 0.6,
            _ => 1.0,
        }
    }

    /// Side grip of a tail wheel relative to a main tyre's (per unit load): a
    /// small solid tyre on steering springs that let it swivel under side
    /// load — why the CG behind the mains wins and a taildragger ground-loops.
    pub fn tail_grip(&self) -> f32 {
        match self.steer {
            Steer::TailSteer { .. } => 0.3,
            Steer::TailCaster { .. } => 0.6,
            _ => 1.0,
        }
    }

    /// "BRAKES HOT" threshold (°C).
    pub fn hot_c(&self) -> f32 {
        match self.brake {
            BrakeKind::Carbon => 300.0,
            BrakeKind::Steel => 250.0,
            BrakeKind::Light => 200.0,
        }
    }

    /// The selector positions in cycling order.
    pub fn autobrake_positions(&self) -> &'static [super::Autobrake] {
        use super::Autobrake::*;
        match self.autobrake {
            None => &[Off],
            Some(a) if a.rto_position => &[Off, Rto, Lo, Med, Max],
            Some(_) => &[Off, Lo, Med, Max],
        }
    }
}

/// Role of one gear contact point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Main,
    Nose,
    Tail,
    /// Unbraked, unsteered wheel (glider nose / tail wheels).
    Free,
}

pub fn role(gp: &GearPoint) -> Role {
    match gp.kind {
        ContactKind::Wheel { braked: true, .. } => Role::Main,
        ContactKind::Wheel { steerable: true, .. } if gp.pos.z >= 0.0 => Role::Nose,
        ContactKind::Wheel { steerable: true, .. } => Role::Tail,
        _ => Role::Free,
    }
}

const fn tiller(tiller_deg: f32, pedal_deg: f32) -> Steer {
    Steer::Tiller { tiller_deg, pedal_deg, tiller_full_kt: 20.0, tiller_zero_kt: 70.0, pedal_full_kt: 40.0, pedal_zero_kt: 130.0, rate_dps: 25.0 }
}

const AIRBUS_AB: AutobrakeSpec = AutobrakeSpec { lo: 1.7, med: 3.0, max: 6.0, delay_lo: 4.0, delay_med: 2.0, rto_position: false, rto_kt: 72.0 };
const EMBRAER_AB: AutobrakeSpec = AutobrakeSpec { lo: 1.8, med: 3.1, max: 4.6, delay_lo: 2.0, delay_med: 1.0, rto_position: true, rto_kt: 60.0 };

/// Undercarriage data of `def`.
pub fn spec(def: &AircraftDef) -> GroundSpec {
    // Light-aircraft default (C172-like), adjusted per type below.
    let light = GroundSpec {
        psi_main: 38.0,
        psi_aux: 31.0,
        r_main: 0.22,
        r_aux: 0.20,
        wheels_per_leg: 1,
        i_main: 0.18,
        i_aux: 0.10,
        brake: BrakeKind::Light,
        sink_kg: 2.5,
        antiskid: false,
        autobrake: None,
        steer: Steer::Pedal { deg: def.nose_steer_deg, fade_ms: 10.0 },
        rudder_diff_brake: true,
        fuse_c: 1e9,
    };
    let airliner = GroundSpec {
        psi_main: 180.0,
        psi_aux: 150.0,
        r_main: 0.56,
        r_aux: 0.38,
        wheels_per_leg: 2,
        i_main: 28.0,
        i_aux: 4.0,
        brake: BrakeKind::Carbon,
        sink_kg: 40.0,
        antiskid: true,
        autobrake: None,
        steer: tiller(def.nose_steer_deg, 6.0),
        rudder_diff_brake: false,
        fuse_c: 177.0,
    };
    match def.id {
        "kestrel" => GroundSpec { steer: Steer::Caster { link_deg: 10.0, free_deg: 30.0, fade_ms: 10.0 }, ..light },
        "swift" => GroundSpec { psi_main: 36.0, psi_aux: 30.0, r_main: 0.19, r_aux: 0.10, i_main: 0.10, i_aux: 0.02, sink_kg: 1.0, steer: Steer::Fixed, ..light },
        // Spitfire IX: 5.5 m/s-ish pneumatic brakes on a lever with the rudder bar
        // splitting them; rudder-linked tail wheel.
        "peregrine" => GroundSpec {
            psi_main: 60.0,
            psi_aux: 55.0,
            r_main: 0.33,
            r_aux: 0.11,
            i_main: 0.9,
            i_aux: 0.03,
            brake: BrakeKind::Steel,
            sink_kg: 4.0,
            steer: Steer::TailSteer { deg: def.nose_steer_deg, breakout_deg: 30.0, fade_ms: 10.0 },
            ..light
        },
        "lancer" => GroundSpec {
            psi_main: 315.0,
            psi_aux: 300.0,
            r_main: 0.39,
            r_aux: 0.27,
            wheels_per_leg: 1,
            i_main: 6.0,
            i_aux: 1.0,
            brake: BrakeKind::Carbon,
            sink_kg: 30.0,
            // (Low-gain NWS: the FDM's ±22° at taxi speed.)
            steer: Steer::Pedal { deg: def.nose_steer_deg, fade_ms: 10.0 },
            ..airliner
        },
        "aerolink" => GroundSpec {
            psi_main: 121.0,
            psi_aux: 61.0,
            r_main: 0.43,
            r_aux: 0.31,
            i_main: 9.0,
            i_aux: 1.5,
            brake: BrakeKind::Carbon,
            sink_kg: 20.0,
            steer: tiller(def.nose_steer_deg, 5.0),
            ..airliner
        },
        "meridian190" => GroundSpec {
            psi_main: 192.0,
            psi_aux: 140.0,
            r_main: 0.51,
            r_aux: 0.30,
            i_main: 18.0,
            i_aux: 2.5,
            sink_kg: 32.0,
            autobrake: Some(EMBRAER_AB),
            steer: tiller(76.0, 7.0),
            ..airliner
        },
        "meridian320" => GroundSpec {
            psi_main: 200.0,
            psi_aux: 165.0,
            r_main: 0.58,
            r_aux: 0.38,
            i_main: 30.0,
            i_aux: 4.5,
            sink_kg: 40.0,
            autobrake: Some(AIRBUS_AB),
            steer: tiller(75.0, 6.0),
            ..airliner
        },
        "hercules" => GroundSpec {
            psi_main: 95.0,
            psi_aux: 60.0,
            r_main: 0.71,
            r_aux: 0.50,
            wheels_per_leg: 1,
            i_main: 45.0,
            i_aux: 8.0,
            brake: BrakeKind::Steel,
            sink_kg: 55.0,
            steer: tiller(60.0, 5.0),
            fuse_c: 180.0,
            ..airliner
        },
        "galaxy" => GroundSpec {
            psi_main: 115.0,
            psi_aux: 115.0,
            r_main: 0.62,
            r_aux: 0.62,
            wheels_per_leg: 6,
            i_main: 35.0,
            i_aux: 30.0,
            sink_kg: 45.0,
            steer: tiller(60.0, 5.0),
            ..airliner
        },
        "vulcan" => GroundSpec {
            psi_main: 110.0,
            psi_aux: 90.0,
            r_main: 0.47,
            r_aux: 0.45,
            wheels_per_leg: 8,
            i_main: 10.0,
            i_aux: 6.0,
            brake: BrakeKind::Steel,
            sink_kg: 25.0,
            steer: Steer::Pedal { deg: def.nose_steer_deg, fade_ms: 10.0 },
            ..airliner
        },
        "extra500" => GroundSpec { psi_main: 55.0, psi_aux: 45.0, r_main: 0.25, r_aux: 0.20, i_main: 0.3, sink_kg: 3.0, ..light },
        "cub" => GroundSpec {
            psi_main: 18.0,
            psi_aux: 40.0,
            r_main: 0.27,
            r_aux: 0.08,
            i_main: 0.25,
            i_aux: 0.01,
            sink_kg: 1.5,
            steer: Steer::TailSteer { deg: def.nose_steer_deg, breakout_deg: 25.0, fade_ms: 10.0 },
            ..light
        },
        "zlin" => GroundSpec {
            psi_main: 35.0,
            psi_aux: 40.0,
            r_main: 0.21,
            r_aux: 0.09,
            i_main: 0.15,
            i_aux: 0.01,
            sink_kg: 1.5,
            steer: Steer::TailSteer { deg: def.nose_steer_deg, breakout_deg: 25.0, fade_ms: 10.0 },
            ..light
        },
        "dc3" => GroundSpec {
            psi_main: 45.0,
            psi_aux: 40.0,
            r_main: 0.66,
            r_aux: 0.25,
            i_main: 14.0,
            i_aux: 0.5,
            brake: BrakeKind::Steel,
            sink_kg: 22.0,
            steer: Steer::TailCaster { lockable: true },
            ..light
        },
        _ => match def.category {
            Category::Airliner => airliner,
            Category::FastJet => GroundSpec { wheels_per_leg: 1, steer: Steer::Pedal { deg: def.nose_steer_deg, fade_ms: 10.0 }, ..airliner },
            Category::Warbird => GroundSpec { brake: BrakeKind::Steel, ..light },
            _ => light,
        },
    }
}
