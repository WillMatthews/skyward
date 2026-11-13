//! Rotorcraft definitions: the Airbus Helicopters EC130 B4 (FlightGear
//! FGAddon `ec130`, GPL v2+) and an original 5-inch FPV quadcopter.
//!
//! EC130 B4 published data (Airbus Helicopters / EASA type data): MTOW
//! 2,427 kg (internal load), empty 1,379 kg; one Safran Arriel 2B1, 632 kW
//! take-off (the main gearbox limits the drive to ≈ 557 kW at 100 % Nr);
//! main rotor Ø 10.69 m, 3 blades, 386 rpm, clockwise seen from above;
//! Fenestron Ø 1.0 m, 10 blades, 3,568 rpm; Vne 155 kt; fast cruise 130 kt;
//! hover ceiling IGE 3,098 m / OGE 2,537 m (ISA, MTOW); service ceiling
//! 4,772 m; max rate of climb 9.0 m/s. Geometry (hub, Fenestron, skids, CG)
//! from the FlightGear model / YASim FDM by Heiko Schulz and Michael Habarta.

use super::heli::HeliDef;
use super::quad::QuadDef;
use super::{Assist, RotorDef};
use crate::flight::{AircraftDef, Category, ContactKind, Controls, EngineKind, GRAVITY, GearPoint, HardPoint};
use glam::Vec3;
use std::sync::Arc;

const S2: f32 = std::f32::consts::FRAC_1_SQRT_2;
const EC130_HUB: Vec3 = Vec3::new(0.0, 1.455, 0.302);
const EC130_R: f32 = 5.345;

const fn skid(x: f32, z: f32) -> GearPoint {
    GearPoint { pos: Vec3::new(x, -1.64, z), spring: 115_000.0, damping: 9_000.0, kind: ContactKind::Skid { friction: 0.55 }, retracts: false }
}

const EC130_GEAR: &[GearPoint] = &[skid(1.22, 1.727), skid(-1.22, 1.727), skid(1.22, -0.57), skid(-1.22, -0.57)];

const fn tip(c: f32, s: f32) -> HardPoint {
    HardPoint { pos: Vec3::new(EC130_HUB.x + EC130_R * c, EC130_HUB.y - 0.05, EC130_HUB.z + EC130_R * s), reason: "Main rotor blade strike", tolerate_below: 0.0 }
}

const EC130_HARD: &[HardPoint] = &[
    tip(1.0, 0.0),
    tip(-1.0, 0.0),
    tip(0.0, 1.0),
    tip(0.0, -1.0),
    tip(S2, S2),
    tip(-S2, S2),
    tip(S2, -S2),
    tip(-S2, -S2),
    HardPoint { pos: Vec3::new(0.0, -0.904, -5.85), reason: "Tail struck the ground", tolerate_below: 6.0 },
    HardPoint { pos: Vec3::new(0.0, -1.1, 2.9), reason: "Nose struck the ground", tolerate_below: 1.0 },
];

const EC130_ENGINE: &[Vec3] = &[Vec3::new(0.0, 0.95, -1.4)];

/// The helicopter's rotor / engine / airframe data.
pub fn ec130_heli() -> HeliDef {
    HeliDef {
        radius: EC130_R,
        blades: 3.0,
        chord: 0.35,
        omega: 386.0 * std::f32::consts::TAU / 60.0,
        lift_slope: 5.8,
        twist: -10.7f32.to_radians(),
        cd0: 0.0084,
        cd2: 0.006,
        kappa: 1.15,
        coll_min: -1.0f32.to_radians(),
        coll_max: 15.5f32.to_radians(),
        cyc_lon: 0.19,
        cyc_lat: 0.11,
        hub: EC130_HUB,
        mast_tilt: 0.035,
        hub_k: 100_000.0,
        lock: 5.0,
        inertia: 1300.0,
        rot_dir: -1.0,
        ct_sigma_max: 0.125,
        tr_pos: Vec3::new(0.125, 0.239, -5.998),
        tr_radius: 0.5,
        tr_solidity: 0.42,
        tr_gear: 3568.0 / 386.0,
        tr_pitch_bias: 0.21,
        tr_pitch_range: 0.40,
        tr_duct: 1.3,
        p_takeoff: 632_000.0,
        p_mcp: 543_000.0,
        p_mgb: 557_000.0,
        eta: 0.96,
        p_access: 8_000.0,
        ng_idle: 67.0,
        tau_up: 1.0,
        tau_down: 0.6,
        sfc: 0.36,
        tot_idle: 450.0,
        tot_max: 865.0,
        f_front: 1.5,
        f_side: 7.0,
        f_top: 9.0,
        download: 0.025,
        fin_area: 1.1,
        fin_pos: Vec3::new(0.0, 0.6, -5.9),
        fin_cl0: 0.25,
        hstab_area: 1.3,
        hstab_pos: Vec3::new(0.0, -0.07, -4.55),
        hstab_incidence: -0.03,
        nr_low: 0.93,
        nr_high: 1.065,
    }
}

/// Airbus Helicopters EC130 B4.
pub fn ec130_def() -> AircraftDef {
    AircraftDef {
        id: "ec130",
        name: "Airbus Helicopters EC130 B4",
        description: "Single-engine light utility helicopter: Arriel 2B1 turboshaft, 3-blade Starflex rotor, Fenestron. Collective on the throttle, cyclic on the stick, pedals on the rudder.",
        has_engine: true,
        flap_label: "—",
        flap_detents: &[0.0],
        flap_max_deg: 0.0,
        vs0_kt: 0.0,
        vs1_kt: 0.0,
        vno_kt: 130.0,
        vne_kt: 155.0,
        gear_height: 1.62,
        wingspan: 2.0 * EC130_R,
        mass: 2100.0,
        inertia_roll: 1700.0,
        inertia_pitch: 5500.0,
        inertia_yaw: 4800.0,
        wing_area: 1.0,
        chord: 1.0,
        wing_height: EC130_HUB.y,
        cd0: 0.03,
        oswald: 0.8,
        cl_alpha: 0.1,
        alpha_stall: 0.3,
        alpha_stall_neg: 0.3,
        stall_sharpness: 10.0,
        surface_rate: 3.0,
        flap_rate: 1.0,
        gear: EC130_GEAR,
        hard_points: EC130_HARD,
        roll_mu: 0.5,
        brake_mu: 0.0,
        tire_mu: 0.55,
        max_touchdown_sink: 2.6,
        max_touchdown_bank: 12f32.to_radians(),
        min_touchdown_pitch: -0.4,
        g_limit_pos: 3.5,
        g_limit_neg: -0.5,
        category: Category::Helicopter,
        engine_kind: EngineKind::Turboshaft,
        engine_count: 1,
        engine_pos: EC130_ENGINE,
        engine_power_w: 632_000.0,
        eye: Vec3::new(0.67, 0.16, 2.152),
        vr_kt: 0.0,
        vref_kt: 65.0,
        ceiling_m: 4772.0,
        rpm_idle: 386.0,
        rpm_static: 386.0,
        idle_power: 0.06,
        spool_time: 1.0,
        sfc: 0.36,
        egt_idle: 450.0,
        egt_max: 865.0,
        role: "Rotorcraft · light utility",
        min_runway_m: 25.0,
        rough_field: true,
        rotor: Some(Arc::new(RotorDef::Heli(ec130_heli()))),
        ..Default::default()
    }
}

const QUAD_ARM: f32 = 0.078;
const QUAD_MOTORS: &[Vec3] =
    &[Vec3::new(QUAD_ARM, 0.012, QUAD_ARM), Vec3::new(-QUAD_ARM, 0.012, QUAD_ARM), Vec3::new(QUAD_ARM, 0.012, -QUAD_ARM), Vec3::new(-QUAD_ARM, 0.012, -QUAD_ARM)];

const fn leg(x: f32, z: f32) -> GearPoint {
    GearPoint { pos: Vec3::new(x, -0.033, z), spring: 1_800.0, damping: 18.0, kind: ContactKind::Skid { friction: 0.6 }, retracts: false }
}

const QUAD_GEAR: &[GearPoint] = &[leg(0.07, 0.07), leg(-0.07, 0.07), leg(0.07, -0.07), leg(-0.07, -0.07)];

/// The quadcopter's motors, battery and flight controller.
pub fn quad_quad() -> QuadDef {
    QuadDef {
        motors: [QUAD_MOTORS[0], QUAD_MOTORS[1], QUAD_MOTORS[2], QUAD_MOTORS[3]],
        // Diagonal pairs turn the same way (front-left / rear-right clockwise).
        spin: [-1.0, 1.0, 1.0, -1.0],
        prop_radius: 0.0635,
        k_t: 1.36e-6,
        k_q: 1.6e-8,
        omega_max: 3300.0,
        tau_motor: 0.025,
        pitch_k: 0.109 / std::f32::consts::TAU,
        idle: 0.04,
        cells: 6.0,
        capacity_mah: 1300.0,
        r_int: 0.035,
        v_full: 25.2,
        eta: 0.78,
        p_avionics: 4.0,
        cda: Vec3::new(0.014, 0.03, 0.011),
        h_drag: 0.012,
        rate_center: 200.0,
        rate_max: 670.0,
        yaw_center: 180.0,
        yaw_max: 480.0,
        angle_max: 55f32.to_radians(),
        angle_k: 7.0,
        kp: Vec3::new(36.0, 36.0, 18.0),
        ki: Vec3::new(30.0, 30.0, 20.0),
        kd: Vec3::new(0.012, 0.012, 0.0),
        cam_tilt: 25f32.to_radians(),
        cam_fov: 120f32.to_radians(),
    }
}

/// An original 5-inch FPV quadcopter.
pub fn quad_def() -> AircraftDef {
    AircraftDef {
        id: "fpv",
        name: "Skyward 5\" FPV quad",
        description: "Original 5-inch freestyle quadcopter: 4 × 2207 motors on 6S, 650 g, ~9:1 thrust to weight. Rate (acro) and angle modes, FPV camera with OSD.",
        has_engine: true,
        flap_label: "—",
        flap_detents: &[0.0],
        vno_kt: 80.0,
        vne_kt: 110.0,
        gear_height: 0.034,
        wingspan: 0.36,
        mass: 0.68,
        inertia_roll: 0.0024,
        inertia_pitch: 0.0028,
        inertia_yaw: 0.0046,
        wing_area: 0.02,
        chord: 0.15,
        cd0: 0.03,
        oswald: 0.8,
        cl_alpha: 0.1,
        alpha_stall: 0.3,
        alpha_stall_neg: 0.3,
        stall_sharpness: 10.0,
        surface_rate: 20.0,
        flap_rate: 1.0,
        gear: QUAD_GEAR,
        hard_points: &[],
        roll_mu: 0.6,
        tire_mu: 0.6,
        max_touchdown_sink: 7.0,
        max_touchdown_bank: 3.2,
        min_touchdown_pitch: -3.2,
        g_limit_pos: 25.0,
        g_limit_neg: -25.0,
        category: Category::Drone,
        engine_kind: EngineKind::Electric,
        engine_count: 4,
        engine_pos: QUAD_MOTORS,
        eye: Vec3::new(0.0, 0.022, 0.065),
        vref_kt: 20.0,
        ceiling_m: 3000.0,
        rpm_static: 29_600.0,
        spool_time: 0.03,
        role: "Drone · 5\" FPV",
        min_runway_m: 2.0,
        rough_field: true,
        rotor: Some(Arc::new(RotorDef::Quad(quad_quad()))),
        ..Default::default()
    }
}

/// Controls that hold the trimmed state of [`crate::flight::Aircraft::airborne`]
/// (the default attitude-hold / angle assist flies the cyclic; the collective
/// / throttle is the pilot's).
pub fn trimmed_controls(def: &AircraftDef, rd: &RotorDef, speed_ms: f32, alt: f32) -> Controls {
    match rd {
        RotorDef::Heli(h) => {
            let atm = crate::flight::isa(alt);
            let tas = crate::flight::tas_from_cas(speed_ms.max(0.0), alt);
            let t = super::heli::trim(h, def.mass, tas, 0.0, atm.density, atm.speed_of_sound, 1e4);
            let assisted = Assist::default() == Assist::Att;
            Controls {
                throttle: t.inp.coll,
                elevator: if assisted { 0.0 } else { t.inp.lon },
                aileron: if assisted { 0.0 } else { t.inp.lat },
                rudder: if assisted { 0.0 } else { t.inp.ped },
                ..Default::default()
            }
        }
        RotorDef::Quad(q) => {
            let _ = GRAVITY;
            Controls { throttle: super::quad::hover_throttle(q, def), ..Default::default() }
        }
    }
}

/// Where a rotorcraft starts on the ground: a clear, dry, level spot beside
/// the runway (apron / grass, well away from the runway and any building:
/// rotor radius + 8 m), facing into wind. Without a runway: near the home point.
pub fn ground_spot(def: &AircraftDef, world: &crate::world::World, rw: Option<&crate::scene::Runway>) -> (glam::Vec2, f32) {
    use crate::flight::Environment;
    use glam::{Vec2, vec2};
    let clear_r = def.wingspan * 0.5 + 8.0;
    let wind = world.wind.mean(10.0, 0.3);
    let level_clear = |q: Vec2| {
        let h0 = world.ground_height(q);
        !world.is_water(q)
            && world.obstacle_top(q).is_none()
            && (0..8).all(|k| {
                let a = k as f32 * std::f32::consts::FRAC_PI_4;
                let o = q + vec2(a.cos(), a.sin()) * clear_r;
                world.obstacle_top(o).is_none() && !world.is_water(o) && (world.ground_height(o) - h0).abs() < 0.8
            })
    };
    let (centre, axis, half) = match rw {
        Some(r) => ((r.a + r.b) * 0.5, (r.b - r.a).normalize_or(vec2(0.0, -1.0)), r.width * 0.5),
        None => (world.home_pos(), vec2(0.0, -1.0), 0.0),
    };
    let side = vec2(-axis.y, axis.x);
    let mut best = None;
    'search: for lat in [70.0f32, 95.0, 120.0, 150.0, 190.0, 240.0] {
        for along in [0.0f32, 150.0, -150.0, 300.0, -300.0, 500.0, -500.0] {
            for s in [1.0f32, -1.0] {
                let q = centre + axis * along + side * s * (half + lat);
                if level_clear(q) && rw.is_none_or(|r| crate::bot::RunwayEnd::of(r, false).cross(q).abs() > half + 40.0) {
                    best = Some(q);
                    break 'search;
                }
            }
        }
    }
    let spot = best.unwrap_or(centre + side * (half + 90.0));
    let hdg = if wind.length() > 1.0 { crate::bot::heading_of(-wind) } else { crate::bot::heading_of(-side) };
    (spot, hdg)
}

/// Gamepad "mode 2" for rotorcraft (two self-centring sticks): left stick
/// vertical = throttle (drone: centre = hover throttle, up = more) or
/// collective (helicopter: moves the lever at up to 0.5 / s, centre = hold),
/// left stick horizontal = yaw / pedals, right stick = pitch / roll (cyclic).
/// Axis values as the bindings deliver them: `left_y` +1 = pulled back,
/// `right_y` +1 = pushed forward.
pub fn gamepad_mode2(def: &AircraftDef, ctl: &mut Controls, left_y: Option<f32>, left_x: Option<f32>, right_y: Option<f32>, right_x: Option<f32>, dt: f32) {
    let up = -left_y.unwrap_or(0.0);
    match def.rotor.as_deref() {
        Some(RotorDef::Quad(q)) => {
            let hover = super::quad::hover_throttle(q, def);
            ctl.throttle = if up >= 0.0 { hover + up * (1.0 - hover) } else { hover * (1.0 + up) };
        }
        _ => {
            let rate = if up.abs() < 0.08 { 0.0 } else { up * 0.5 };
            ctl.throttle = (ctl.throttle + rate * dt).clamp(0.0, 1.0);
        }
    }
    if let Some(x) = left_x {
        ctl.rudder = x;
    }
    if let Some(y) = right_y {
        ctl.elevator = -y;
    }
    if let Some(x) = right_x {
        ctl.aileron = x;
    }
}
