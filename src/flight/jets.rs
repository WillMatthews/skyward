//! Track H2a jets: Boeing 777-300ER, Bombardier CRJ700, Cessna Citation X
//! and Concorde (FlightGear GPL models, see `assets/aircraft/<id>/CREDITS.md`).
//!
//! The 777 and the Citation X fly on Skyward's parametric model calibrated
//! to their YASim data and published figures; the CRJ700 and Concorde on
//! their JSBSim aerodynamics with Skyward calibration patches
//! (`tools/fgimport/patches/aero.py`).

use super::*;

// ---------------------------------------------------------------------------
// Boeing 777-300ER
// ---------------------------------------------------------------------------

const TRIPLE7_GEAR: &[GearPoint] = &[
    // FGAddon 777 model, gear down. The six-wheel bogies hang tilted with the
    // aft axle lowest (the real bogie tilts so the aircraft rotates about the
    // aft axle): that axle is the main contact.
    wheel(Vec3::new(0.0, -5.07, 29.22), 1_500_000.0, 250_000.0, true, false, true),
    wheel(Vec3::new(6.20, -5.74, -2.92), 6_000_000.0, 1_600_000.0, false, true, true),
    wheel(Vec3::new(-6.20, -5.74, -2.92), 6_000_000.0, 1_600_000.0, false, true, true),
    // Tail skid / aft belly (tail strike ~9.5° on the aft axles).
    skid(Vec3::new(0.0, -2.88, -20.5), 6_000_000.0, 900_000.0, 0.5),
    skid(Vec3::new(0.0, -1.60, -26.0), 6_000_000.0, 900_000.0, 0.5),
];

const TRIPLE7_HARD: &[HardPoint] = &[
    hard(0.0, -1.2, 35.40, "Flew into the ground"),
    hard(32.30, 2.60, -14.40, "Wingtip strike"),
    hard(-32.30, 2.60, -14.40, "Wingtip strike"),
    hard(9.89, -4.35, 8.20, "Engine pod strike"),
    hard(-9.89, -4.35, 8.20, "Engine pod strike"),
    hard(9.89, -4.10, 4.50, "Engine pod strike"),
    hard(-9.89, -4.10, 4.50, "Engine pod strike"),
    hard(0.0, 3.60, 28.0, "Crashed inverted"),
    hard(0.0, 12.80, -36.5, "Crashed inverted"),
    hard(0.0, -2.95, 18.0, GEAR_UP),
    hard(0.0, -2.95, 1.0, GEAR_UP),
    hard(0.0, -2.90, -14.0, GEAR_UP),
];

/// GE90-115B fan faces, engine 0 = left.
const TRIPLE7_ENGINES: &[Vec3] = &[Vec3::new(9.89, -2.50, 8.20), Vec3::new(-9.89, -2.50, 8.20)];

pub(super) fn triple7_def() -> AircraftDef {
    AircraftDef {
        id: "triple7",
        name: "Boeing 777-300ER",
        description: "Long-haul widebody twin (FlightGear 777 model; flight model calibrated to its \
                      YASim data and published figures). 2 × GE90-115B, 513 kN each — the most powerful \
                      jet engines in service. FBW with envelope protection, 250 t as flown.",
        role: "Widebody airliner",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 1.0, 5.0, 15.0, 20.0, 25.0, 30.0],
        flap_max_deg: 30.0,
        vs0_kt: 123.0,
        vs1_kt: 148.0,
        vno_kt: 330.0,
        vne_kt: 330.0,
        gear_height: 5.56,
        wingspan: 64.8,
        mass: 250_000.0,
        inertia_roll: 18_000_000.0,
        inertia_pitch: 52_000_000.0,
        inertia_yaw: 68_000_000.0,
        wing_area: 436.8,
        chord: 7.07,
        wing_height: -1.2,
        cl0: 0.32,
        cl_alpha: 5.3,
        alpha_stall: deg(14.0),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.0175,
        oswald: 0.85,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.80,
        flap_curve: 0.8,
        flap_alpha_stall: deg(-1.0),
        flap_cd: 0.075,
        flap_cd_exp: 1.5,
        flap_cm: -0.06,
        cm0: 0.03,
        cm_alpha: -1.2,
        cm_q: -32.0,
        cm_elevator: 0.75,
        cm_stall: -0.12,
        trim_authority: 0.5,
        cy_beta: -0.8,
        cl_beta: -0.09,
        cl_p: -0.45,
        stall_autorotation: 0.1,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.028,
        cn_beta: 0.11,
        cn_r: -0.16,
        cn_p: -0.02,
        cn_rudder: 0.065,
        cn_aileron: -0.002,
        surface_rate: 2.2,
        flap_rate: 0.05,
        static_thrust: 1_026_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 16.6,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 7.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: TRIPLE7_GEAR,
        hard_points: TRIPLE7_HARD,
        roll_mu: 0.02,
        brake_mu: 0.42,
        tire_mu: 0.8,
        nose_steer_deg: 70.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(8.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 2.5,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 2,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.57, 1.30, 32.98),
        vr_kt: 150.0,
        vref_kt: 151.0,
        vle_kt: 270.0,
        ceiling_m: 13_140.0,
        mmo: 0.89,
        control_law: ControlLaw::Airliner,
        gear_time: 10.0,
        cd_gear: 0.016,
        spoiler_cl: 0.45,
        spoiler_cd: 0.08,
        spoiler_flight_max: 0.5,
        mach_dd: 0.88,
        cd_wave: 0.06,
        engine_pos: TRIPLE7_ENGINES,
        n1_idle: 0.20,
        idle_thrust_frac: 0.035,
        thrust_mach: [-0.85, 0.35],
        n1_rpm: 2_355.0,
        sfc: 0.030,
        egt_idle: 420.0,
        egt_max: 950.0,
        reverse_eff: 0.35,
        alpha_max: deg(13.5),
        nz_max: 2.5,
        nz_min: -1.0,
        roll_rate_max: deg(12.0),
        yaw_damper: 2.5,
        min_runway_m: 2_500.0,
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// Bombardier CRJ700
// ---------------------------------------------------------------------------

const CRJ700_GEAR: &[GearPoint] = &[
    // FGMEMBERS CRJ700 model, gear down.
    wheel(Vec3::new(0.0, -2.40, 13.57), 300_000.0, 30_000.0, true, false, true),
    wheel(Vec3::new(2.05, -2.75, -1.65), 1_000_000.0, 110_000.0, false, true, true),
    wheel(Vec3::new(-2.05, -2.75, -1.65), 1_000_000.0, 110_000.0, false, true, true),
    // Aft belly (tail strike ~12°).
    skid(Vec3::new(0.0, -0.98, -10.0), 1_000_000.0, 100_000.0, 0.5),
    skid(Vec3::new(0.0, -0.33, -13.0), 1_000_000.0, 100_000.0, 0.5),
];

const CRJ700_HARD: &[HardPoint] = &[
    hard(0.0, -0.4, 15.55, "Flew into the ground"),
    hard(11.60, -0.80, -4.40, "Wingtip strike"),
    hard(-11.60, -0.80, -4.40, "Wingtip strike"),
    hard(0.0, 1.70, 12.0, "Crashed inverted"),
    hard(0.0, 4.30, -15.4, "Crashed inverted"),
    hard(0.0, -1.70, 9.0, GEAR_UP),
    hard(0.0, -1.80, -1.0, GEAR_UP),
    hard(0.0, -1.30, -7.0, GEAR_UP),
];

/// CF34-8C5B1 fan faces (rear-fuselage pods), engine 0 = left.
const CRJ700_ENGINES: &[Vec3] = &[Vec3::new(2.28, 0.77, -6.80), Vec3::new(-2.28, 0.77, -6.80)];

pub(super) fn crj700_def() -> AircraftDef {
    with_jsb(crj700_base(), include_str!("../../assets/aircraft/crj700/fdm.xml"), &[("fcs/spoilers-pos-norm", "fcs/spoiler-pos-norm")], &[], 200.0, |j| {
        j.elev_up = deg(25.0);
        j.elev_down = deg(20.0);
        j.ail_up = deg(25.0);
        j.ail_down = deg(20.0);
        j.rudder_max = deg(33.0);
        j.spoiler_max_deg = 45.0;
    })
}

fn crj700_base() -> AircraftDef {
    AircraftDef {
        id: "crj700",
        name: "Bombardier CRJ700",
        description: "70-seat regional jet (FlightGear CRJ700-family model, JSBSim aerodynamics with \
                      Skyward calibration). 2 × GE CF34-8C5B1, 61 kN each, mounted on the rear \
                      fuselage under a T-tail. 30 t as flown.",
        role: "Regional jet",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 1.0, 8.0, 20.0, 30.0, 45.0],
        flap_max_deg: 45.0,
        vs0_kt: 106.0,
        vs1_kt: 139.0,
        vno_kt: 335.0,
        vne_kt: 335.0,
        gear_height: 2.62,
        wingspan: 23.16,
        mass: 29_730.0,
        inertia_roll: 165_000.0,
        inertia_pitch: 743_000.0,
        inertia_yaw: 802_000.0,
        wing_area: 70.6,
        chord: 2.49,
        wing_height: -1.3,
        cl0: 0.25,
        cl_alpha: 5.3,
        alpha_stall: deg(13.5),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.021,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.9,
        flap_curve: 0.4,
        flap_alpha_stall: 0.0,
        flap_cd: 0.07,
        flap_cd_exp: 1.4,
        flap_cm: -0.05,
        cm0: 0.02,
        cm_alpha: -0.9,
        cm_q: -24.0,
        cm_elevator: 0.8,
        cm_stall: -0.10,
        trim_authority: 0.6,
        cy_beta: -0.6,
        cl_beta: -0.10,
        cl_p: -0.45,
        stall_autorotation: 0.12,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.04,
        cn_beta: 0.12,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.06,
        cn_aileron: -0.002,
        surface_rate: 2.5,
        flap_rate: 0.08,
        static_thrust: 122_600.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 2.4,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 6.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: CRJ700_GEAR,
        hard_points: CRJ700_HARD,
        roll_mu: 0.02,
        brake_mu: 0.45,
        tire_mu: 0.8,
        nose_steer_deg: 70.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(7.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 2.5,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 2,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.53, 0.59, 12.44),
        vr_kt: 135.0,
        vref_kt: 136.0,
        vle_kt: 220.0,
        ceiling_m: 12_497.0,
        mmo: 0.825,
        gear_time: 8.0,
        cd_gear: 0.015,
        spoiler_cl: 0.4,
        spoiler_cd: 0.07,
        spoiler_flight_max: 0.5,
        mach_dd: 0.80,
        cd_wave: 0.05,
        engine_pos: CRJ700_ENGINES,
        n1_idle: 0.24,
        idle_thrust_frac: 0.04,
        thrust_mach: [-0.80, 0.30],
        n1_rpm: 7_400.0,
        sfc: 0.039,
        egt_idle: 430.0,
        egt_max: 880.0,
        reverse_eff: 0.35,
        yaw_damper: 2.5,
        min_runway_m: 1_600.0,
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// Cessna Citation X (Model 750)
// ---------------------------------------------------------------------------

const CITATIONX_GEAR: &[GearPoint] = &[
    // FGMEMBERS CitationX model, gear down.
    wheel(Vec3::new(0.0, -2.22, 7.65), 160_000.0, 16_000.0, true, false, true),
    wheel(Vec3::new(1.585, -2.54, -0.57), 520_000.0, 55_000.0, false, true, true),
    wheel(Vec3::new(-1.585, -2.54, -0.57), 520_000.0, 55_000.0, false, true, true),
    // Aft belly under the engines.
    skid(Vec3::new(0.0, -0.80, -6.0), 600_000.0, 60_000.0, 0.5),
];

const CITATIONX_HARD: &[HardPoint] = &[
    hard(0.0, -0.8, 9.70, "Flew into the ground"),
    hard(9.80, -0.70, -5.00, "Wingtip strike"),
    hard(-9.80, -0.70, -5.00, "Wingtip strike"),
    hard(0.0, 1.20, 6.0, "Crashed inverted"),
    hard(0.0, 3.75, -10.6, "Crashed inverted"),
    hard(0.0, -1.50, 4.0, GEAR_UP),
    hard(0.0, -1.60, -1.5, GEAR_UP),
];

/// AE3007C1 fan faces, engine 0 = left.
const CITATIONX_ENGINES: &[Vec3] = &[Vec3::new(1.70, 0.73, -1.70), Vec3::new(-1.70, 0.73, -1.70)];

pub(super) fn citationx_def() -> AircraftDef {
    AircraftDef {
        id: "citationx",
        name: "Cessna Citation X",
        description: "Mach 0.92 business jet (FlightGear CitationX model; flight model calibrated to its \
                      YASim data and published figures). 2 × Rolls-Royce AE3007C1, 30 kN each; 37° \
                      wing sweep, all-moving tail. 14 t as flown.",
        role: "Business jet",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 5.0, 15.0, 35.0],
        flap_max_deg: 35.0,
        vs0_kt: 93.0,
        vs1_kt: 113.0,
        vno_kt: 350.0,
        vne_kt: 350.0,
        gear_height: 2.43,
        wingspan: 19.38,
        mass: 14_000.0,
        inertia_roll: 70_000.0,
        inertia_pitch: 160_000.0,
        inertia_yaw: 220_000.0,
        wing_area: 48.96,
        chord: 2.86,
        wing_height: -1.3,
        cl0: 0.20,
        cl_alpha: 4.9,
        alpha_stall: deg(15.0),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 30.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.0195,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.62,
        flap_curve: 0.5,
        flap_alpha_stall: deg(-1.0),
        flap_cd: 0.07,
        flap_cd_exp: 1.5,
        flap_cm: -0.05,
        cm0: 0.02,
        cm_alpha: -1.3,
        cm_q: -22.0,
        cm_elevator: 0.85,
        cm_stall: -0.10,
        trim_authority: 0.6,
        cy_beta: -0.6,
        cl_beta: -0.10,
        cl_p: -0.45,
        stall_autorotation: 0.15,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.018,
        cn_beta: 0.12,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.06,
        cn_aileron: -0.002,
        surface_rate: 2.5,
        flap_rate: 0.10,
        static_thrust: 60_200.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 1.2,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 5.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: CITATIONX_GEAR,
        hard_points: CITATIONX_HARD,
        roll_mu: 0.02,
        brake_mu: 0.42,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(8.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 3.0,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 2,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.375, 0.41, 6.55),
        vr_kt: 120.0,
        vref_kt: 121.0,
        vle_kt: 210.0,
        ceiling_m: 15_545.0,
        mmo: 0.92,
        gear_time: 7.0,
        cd_gear: 0.018,
        spoiler_cl: 0.35,
        spoiler_cd: 0.06,
        spoiler_flight_max: 0.6,
        mach_dd: 0.90,
        cd_wave: 0.06,
        engine_pos: CITATIONX_ENGINES,
        n1_idle: 0.24,
        idle_thrust_frac: 0.045,
        thrust_mach: [-0.65, 0.32],
        n1_rpm: 15_000.0,
        sfc: 0.040,
        egt_idle: 450.0,
        egt_max: 900.0,
        reverse_eff: 0.35,
        yaw_damper: 2.0,
        min_runway_m: 1_200.0,
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// Aerospatiale / BAC Concorde
// ---------------------------------------------------------------------------

const CONCORDE_GEAR: &[GearPoint] = &[
    // FGAddon Concorde model / FDM contacts, gear down. The long nose leg
    // leaves her slightly nose-up at rest.
    wheel(Vec3::new(0.0, -3.94, 14.10), 900_000.0, 90_000.0, true, false, true),
    wheel(Vec3::new(3.73, -3.37, -2.52), 2_400_000.0, 400_000.0, false, true, true),
    wheel(Vec3::new(-3.73, -3.37, -2.52), 2_400_000.0, 400_000.0, false, true, true),
    wheel(Vec3::new(3.73, -3.37, -4.27), 2_400_000.0, 400_000.0, false, true, true),
    wheel(Vec3::new(-3.73, -3.37, -4.27), 2_400_000.0, 400_000.0, false, true, true),
    // Retractable tail bumper wheel and the nozzle buckets, set for the
    // main legs' extension as they unload at rotation (the bumper touches at
    // ~12.5°, the buckets at ~13.5°: Concorde rotates to 13.5° only as she
    // unsticks) and the aft fuselage: scrapes, not crashes.
    skid(Vec3::new(0.0, 0.70, -21.6), 3_000_000.0, 300_000.0, 0.15),
    skid(Vec3::new(5.5, -0.60, -15.0), 3_000_000.0, 300_000.0, 0.5),
    skid(Vec3::new(-5.5, -0.60, -15.0), 3_000_000.0, 300_000.0, 0.5),
    skid(Vec3::new(0.0, 1.20, -26.0), 3_000_000.0, 300_000.0, 0.5),
];

const CONCORDE_HARD: &[HardPoint] = &[
    hard(0.0, 0.20, 33.6, "Flew into the ground"),
    hard(12.30, 0.70, -13.10, "Wingtip strike"),
    hard(-12.30, 0.70, -13.10, "Wingtip strike"),
    hard(6.50, -1.15, -6.0, "Engine pod strike"),
    hard(-6.50, -1.15, -6.0, "Engine pod strike"),
    hard(6.50, -1.30, -10.0, "Engine pod strike"),
    hard(-6.50, -1.30, -10.0, "Engine pod strike"),
    hard(0.0, 2.20, 20.0, "Crashed inverted"),
    hard(0.0, 8.20, -22.0, "Crashed inverted"),
    hard(0.0, -0.30, 10.0, GEAR_UP),
    hard(0.0, -0.33, -2.0, GEAR_UP),
];

/// Olympus 593 Mk 610s in their paired nacelles, engine 0 = left outboard.
const CONCORDE_ENGINES: &[Vec3] = &[Vec3::new(6.33, -0.61, -10.77), Vec3::new(4.69, -0.61, -10.77), Vec3::new(-4.69, -0.61, -10.77), Vec3::new(-6.33, -0.61, -10.77)];

pub(super) fn concorde_def() -> AircraftDef {
    with_jsb(concorde_base(), include_str!("../../assets/aircraft/concorde/fdm.xml"), &[], &[], 250.0, |j| {
        // Elevons: pitch ±17°, roll ±17° (mechanical limits 23.5° outboard);
        // rudder ±30°. The flap lever droops the nose / visor (no flaps).
        j.elev_up = deg(17.0);
        j.elev_down = deg(17.0);
        j.ail_up = deg(17.0);
        j.ail_down = deg(17.0);
        j.rudder_max = deg(30.0);
        j.flap_max_deg = 0.0;
    })
}

fn concorde_base() -> AircraftDef {
    AircraftDef {
        id: "concorde",
        name: "Aerospatiale/BAC Concorde",
        description: "Supersonic airliner (FlightGear Concorde model; JSBSim with Skyward ogival-delta \
                      aerodynamics: vortex lift to 30° alpha, no stall break). 4 × Olympus 593 Mk 610, \
                      reheat for take-off and the transonic push, Mach 2 cruise at FL550-600. Flap lever = \
                      droop nose / visor.",
        role: "Supersonic airliner",
        has_engine: true,
        flap_label: "Nose",
        // Lever positions of the FlightGear model's nose / visor animation:
        // UP, visor down (nose 0°), nose 5°, nose 12.5° (labels: `sim.rs`).
        flap_detents: &[0.0, 4.25, 8.5, 12.5],
        flap_max_deg: 0.0,
        vs0_kt: 128.0,
        vs1_kt: 128.0,
        // VMO 530 KIAS at altitude; Mmo 2.04 is the limit in the cruise.
        vno_kt: 530.0,
        vne_kt: 560.0,
        gear_height: 3.42,
        wingspan: 25.55,
        mass: 118_000.0,
        inertia_roll: 2_175_000.0,
        inertia_pitch: 27_330_000.0,
        inertia_yaw: 15_440_000.0,
        wing_area: 358.25,
        chord: 27.66,
        wing_height: 0.2,
        cl0: 0.0,
        cl_alpha: 2.9,
        alpha_stall: deg(28.0),
        alpha_stall_neg: deg(20.0),
        stall_sharpness: 15.0,
        cl_flat: 1.0,
        stall_drop: 0.95,
        stall_decay: 0.3,
        cd0: 0.0085,
        oswald: 0.6,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.0,
        flap_curve: 0.0,
        flap_alpha_stall: 0.0,
        flap_cd: 0.004,
        flap_cd_exp: 1.0,
        flap_cm: 0.0,
        cm0: 0.0,
        cm_alpha: -0.7,
        cm_q: -21.0,
        cm_elevator: 0.6,
        cm_stall: 0.0,
        trim_authority: 0.6,
        cy_beta: -1.0,
        cl_beta: -0.10,
        cl_p: -0.4,
        stall_autorotation: 0.05,
        cl_r_per_cl: 0.15,
        cl_aileron: 0.05,
        cn_beta: 0.12,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.05,
        cn_aileron: 0.0,
        surface_rate: 2.0,
        flap_rate: 0.08,
        static_thrust: 560_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 4.4,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 8.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: CONCORDE_GEAR,
        hard_points: CONCORDE_HARD,
        roll_mu: 0.02,
        brake_mu: 0.40,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(6.0),
        min_touchdown_pitch: deg(-0.5),
        g_limit_pos: 2.5,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 4,
        retractable_gear: true,
        has_reverser: true,
        has_afterburner: true,
        eye: Vec3::new(0.452, 2.24, 25.20),
        vr_kt: 165.0,
        vref_kt: 168.0,
        vle_kt: 270.0,
        ceiling_m: 18_300.0,
        mmo: 2.04,
        gear_time: 10.0,
        cd_gear: 0.011,
        mach_dd: 0.95,
        cd_wave: 0.014,
        engine_pos: CONCORDE_ENGINES,
        n1_idle: 0.30,
        idle_thrust_frac: 0.05,
        // Variable intakes: ram recovery makes the dry thrust grow with
        // Mach (×1.6 at M2: ~200 kN dry at FL550), the nose-temperature limit
        // (127 °C) caps it.
        thrust_mach: [-0.35, 0.325],
        ab_static_thrust: 116_000.0,
        ab_mach: [-0.2, 0.25],
        t_total_max: 440.0,
        n1_rpm: 6_500.0,
        sfc: 0.12,
        sfc_ab: 0.20,
        egt_idle: 400.0,
        egt_max: 750.0,
        reverse_eff: 0.30,
        // The first airliner flown by wire (analogue, with autostabilisation
        // and auto-trim): flight-path stability, alpha limit 17°.
        control_law: ControlLaw::Airliner,
        alpha_max: deg(17.0),
        nz_max: 2.5,
        nz_min: -1.0,
        roll_rate_max: deg(15.0),
        yaw_damper: 2.0,
        min_runway_m: 2_800.0,
        // Trim-fuel transfer: CG aft from 53 % to ~59 % of the root chord
        // through the transonic acceleration (follows the aft move of the
        // centre of pressure), forward again on the way down.
        cg_schedule: &[(0.0, 0.0), (0.85, 0.0), (1.0, 0.6), (1.5, 1.3), (2.0, 1.65)],
        cg_rate: 0.004,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `JETS_PROBE=<id> JETS_MACH=0.74 cargo test --release jets_aero_probe -- --ignored --nocapture`:
    /// lift, drag and pitching moment (about the CG) against alpha from the
    /// type's JSBSim aerodynamics, neutral surfaces, clean, out of ground effect.
    #[test]
    #[ignore]
    fn jets_aero_probe() {
        let id = std::env::var("JETS_PROBE").unwrap_or_else(|_| "crj700".into());
        let mach: f32 = std::env::var("JETS_MACH").ok().and_then(|v| v.parse().ok()).unwrap_or(0.3);
        let alt: f32 = std::env::var("JETS_ALT").ok().and_then(|v| v.parse().ok()).unwrap_or(3000.0);
        let flap: f32 = std::env::var("JETS_FLAP").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
        let elev: f32 = std::env::var("JETS_ELEV").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
        let def = find(&id).unwrap();
        let j = def.jsb.as_ref().expect("JSBSim type");
        let atm = isa(alt);
        let v = mach * atm.speed_of_sound;
        let q = 0.5 * atm.density * v * v;
        let s = JsbSurf { elevator: elev, aileron: 0.0, rudder: 0.0, flaps: flap, spoilers: 0.0, gear: 0.0 };
        for k in -4..=30 {
            let al = deg(k as f32);
            let vb = Vec3::new(0.0, -v * al.sin(), v * al.cos());
            let o = j.aero.eval_steady(&j.input(vb, Vec3::ZERO, atm.density, mach, &s, 1000.0, 1000.0, 0.0, atm.pressure));
            let cm = o.moment.x * -1.0 / (q * def.wing_area * def.chord);
            println!("{id} M{mach:.2} alpha {k:3}°: CL {:6.3} CD {:6.4} L/D {:5.1} Cm(cg) {:7.4}", o.cl, o.cd, o.cl / o.cd.max(1e-4), cm);
        }
    }

    struct Still;
    impl Environment for Still {
        fn ground_height(&self, _: Vec2) -> f32 {
            0.0
        }
        fn obstacle_top(&self, _: Vec2) -> Option<f32> {
            None
        }
        fn is_water(&self, _: Vec2) -> bool {
            false
        }
        fn updraft(&self, _: Vec3) -> f32 {
            0.0
        }
    }

    /// `JETS_PROBE=<id> JETS_KT=420 JETS_ALT=10670 ... jets_free_response -- --ignored --nocapture`:
    /// trimmed level flight with the controls fixed (no pilot), 60 s.
    #[test]
    #[ignore]
    fn jets_free_response() {
        let id = std::env::var("JETS_PROBE").unwrap_or_else(|_| "crj700".into());
        let kt: f32 = std::env::var("JETS_KT").ok().and_then(|v| v.parse().ok()).unwrap_or(250.0);
        let alt: f32 = std::env::var("JETS_ALT").ok().and_then(|v| v.parse().ok()).unwrap_or(3000.0);
        let def = find(&id).unwrap();
        let ias = cas_from_tas(kt * KT, alt);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt, 0.0), 0.0, ias);
        let mut c = trimmed_controls_at(&def, ias, alt);
        if let Some(t) = std::env::var("JETS_THR").ok().and_then(|v| v.parse().ok()) {
            c.throttle = t;
        }
        for k in 0..(60 * 120) {
            if k % 60 == 0 {
                let thrust: f32 = (0..def.engine_count).map(|i| a.engine_thrust(i)).sum();
                println!(
                    "t {:5.1} ktas {:5.1} M{:.3} alt {:7.1} pitch {:6.2} alpha {:6.2} q {:6.2}°/s el {:.3} trim {:.3} thrust {:.0} kN cg {:.2}",
                    k as f32 / 120.0,
                    a.tas() / KT,
                    a.mach(),
                    a.pos.y,
                    a.pitch().to_degrees(),
                    a.alpha().to_degrees(),
                    a.rates().y.to_degrees(),
                    a.surfaces().elevator,
                    c.trim,
                    thrust / 1e3,
                    a.cg_aft()
                );
            }
            a.step(&def, &c, &Still, 1.0 / 120.0);
        }
    }
}
