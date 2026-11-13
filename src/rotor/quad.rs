//! FPV quadcopter: four brushless motors and props in an X frame, a flight
//! controller (Betaflight-style "actual" rates, PID rate loops with D on the
//! gyro, angle mode on top, airmode mixer with thrust linearisation) and a
//! LiPo pack whose voltage sags under load and falls with the charge used —
//! the props' top speed follows the voltage.
//!
//! Props: thrust `k_t ω²` reduced by the axial inflow (climbing into the
//! air / pitch speed), torque `k_q ω²` (yaw), first-order motor lag. Prop
//! wash: descending into the props' own wake at low forward speed (the
//! vortex-ring range of the axial velocity) makes each prop's thrust
//! fluctuate — the quad shakes and wallows unless it moves sideways.

use super::{Assist, Out};
use crate::flight::{Aircraft, AircraftDef, Atmosphere, Controls, GRAVITY};
use glam::Vec3;
use std::f32::consts::PI;

/// Static quadcopter data.
#[derive(Clone, Debug)]
pub struct QuadDef {
    /// Motor positions (body frame) and spin directions (+1 = anticlockwise seen from above).
    pub motors: [Vec3; 4],
    pub spin: [f32; 4],
    pub prop_radius: f32,
    /// Thrust and torque coefficients, N/(rad/s)², N·m/(rad/s)².
    pub k_t: f32,
    pub k_q: f32,
    /// Motor speed at full throttle on a full pack, rad/s.
    pub omega_max: f32,
    pub tau_motor: f32,
    /// Prop pitch / 2π, m per radian (pitch speed = ω × this).
    pub pitch_k: f32,
    /// Idle motor command (airmode), fraction of max thrust.
    pub idle: f32,
    /// Battery: cells, capacity (mAh), pack internal resistance (Ω), full-pack voltage.
    pub cells: f32,
    pub capacity_mah: f32,
    pub r_int: f32,
    pub v_full: f32,
    /// Drive efficiency (motor + ESC), avionics / VTX power (W).
    pub eta: f32,
    pub p_avionics: f32,
    /// Body drag areas (m²): side, vertical, frontal; rotor H-drag per N of thrust per m/s.
    pub cda: Vec3,
    pub h_drag: f32,
    /// Rates: centre sensitivity and max rate (deg/s) roll/pitch and yaw.
    pub rate_center: f32,
    pub rate_max: f32,
    pub yaw_center: f32,
    pub yaw_max: f32,
    /// Angle mode: max tilt (rad), self-levelling gain (1/s).
    pub angle_max: f32,
    pub angle_k: f32,
    /// Rate-loop gains: P (1/s), I (1/s²), D (dimensionless, on the gyro).
    pub kp: Vec3,
    pub ki: Vec3,
    pub kd: Vec3,
    /// FPV camera uptilt, rad, and horizontal field of view, rad.
    pub cam_tilt: f32,
    pub cam_fov: f32,
}

/// Live quadcopter state.
#[derive(Clone, Copy, Debug, Default)]
pub struct QuadState {
    /// Motor speeds, rad/s.
    pub w: [f32; 4],
    /// Rate-loop integrators and last gyro (body p, q, r).
    pub i_err: Vec3,
    pub last_gyro: Vec3,
    pub used_mah: f32,
    pub v_pack: f32,
    pub current: f32,
    pub time: f32,
    pub flight_time: f32,
    pub init: bool,
    /// Take-off point (OSD home arrow / altitude).
    pub home: Vec3,
}

impl QuadDef {
    pub fn disc_area(&self) -> f32 {
        PI * self.prop_radius * self.prop_radius
    }
    /// Commanded body rate (rad/s) for a stick position (Betaflight "actual"-like).
    pub fn rate(&self, x: f32, yaw: bool) -> f32 {
        let (c, m) = if yaw { (self.yaw_center, self.yaw_max) } else { (self.rate_center, self.rate_max) };
        let a = x.clamp(-1.0, 1.0).abs();
        x.signum() * (c * a + (m - c) * a * a * a).to_radians()
    }
    /// Open-circuit cell voltage at state of charge `soc`.
    pub fn cell_ocv(soc: f32) -> f32 {
        const T: [(f32, f32); 8] = [(0.0, 3.2), (0.05, 3.45), (0.1, 3.6), (0.2, 3.7), (0.5, 3.82), (0.8, 3.98), (0.9, 4.08), (1.0, 4.2)];
        let s = soc.clamp(0.0, 1.0);
        for w in T.windows(2) {
            if s <= w[1].0 {
                let f = (s - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + f * (w[1].1 - w[0].1);
            }
        }
        4.2
    }
    /// Max motor thrust (N) at pack voltage `v`.
    pub fn max_thrust(&self, v: f32) -> f32 {
        let w = self.omega_max * (v / self.v_full).clamp(0.2, 1.05);
        self.k_t * w * w
    }
}

/// Mixer: per-motor thrusts for a total thrust and body torques (x, y, z).
fn mix(q: &QuadDef, total: f32, tau: Vec3) -> [f32; 4] {
    // Least-norm inverse of the 4×4 allocation for a symmetric X frame:
    // torque.x = Σ −z_i T_i, torque.z = Σ x_i T_i, torque.y = Σ −s_i (k_q/k_t) T_i.
    let kr = q.k_q / q.k_t;
    let mut t = [0.0f32; 4];
    let (sx, sz): (f32, f32) = (q.motors.iter().map(|m| m.x * m.x).sum(), q.motors.iter().map(|m| m.z * m.z).sum());
    for i in 0..4 {
        let m = q.motors[i];
        t[i] = total / 4.0 + tau.z * m.x / sx - tau.x * m.z / sz - tau.y * q.spin[i] / (4.0 * kr);
    }
    t
}

/// Spawn: armed, motors at hover (air) or idle (ground), a full pack.
pub fn settle(q: &QuadDef, def: &AircraftDef, ac: &mut Aircraft, ctl: &mut Controls, airborne: bool) {
    let st = &mut ac.rotor.quad;
    *st = QuadState { init: true, v_pack: q.v_full, home: ac.pos - Vec3::Y * def.gear_height, ..Default::default() };
    let hover_t = def.mass * GRAVITY / 4.0;
    let tmax = q.max_thrust(q.v_full);
    let w = if airborne { (hover_t / q.k_t).sqrt() } else { (q.idle * tmax / q.k_t).sqrt() };
    st.w = [w; 4];
    ctl.throttle = if airborne { hover_throttle(q, def) } else { 0.0 };
    // The OSD reads these before the first step.
    let o = &mut ac.rotor.out;
    o.batt_v = q.v_full;
    o.batt_soc = 1.0;
    o.motors = [w / q.omega_max; 4];
    o.nr_rpm = w * 60.0 / (2.0 * PI);
}

/// Throttle that hovers on a full pack.
pub fn hover_throttle(q: &QuadDef, def: &AircraftDef) -> f32 {
    let tmax = 4.0 * q.max_thrust(q.v_full);
    thr_inverse(q, def.mass * GRAVITY / tmax)
}

/// Throttle curve: stick → collective thrust fraction (idle at the bottom).
fn thr_curve(q: &QuadDef, x: f32) -> f32 {
    q.idle + (1.0 - q.idle) * x.clamp(0.0, 1.0)
}

fn thr_inverse(q: &QuadDef, f: f32) -> f32 {
    ((f - q.idle) / (1.0 - q.idle)).clamp(0.0, 1.0)
}

/// One step of the quadcopter: flight controller → motors → forces, battery.
#[allow(clippy::too_many_arguments)]
pub fn step(q: &QuadDef, def: &AircraftDef, ac: &mut Aircraft, c: &Controls, vb: Vec3, agl: f32, atm: &Atmosphere, dt: f32) -> Out {
    if !ac.rotor.quad.init {
        let mut cc = *c;
        settle(q, def, ac, &mut cc, !ac.on_ground);
    }
    let _ = agl;
    let angle = ac.rotor.assist == Assist::Att;
    let gyro = Vec3::new(ac.omega.z, -ac.omega.x, -ac.omega.y); // p, q, r
    let (pitch, roll) = (ac.pitch(), ac.roll());
    let inertia = Vec3::new(def.inertia_roll, def.inertia_pitch, def.inertia_yaw); // about p, q, r
    let st = &mut ac.rotor.quad;
    st.time += dt;
    if !ac.on_ground || c.throttle > 0.05 {
        st.flight_time += dt;
    }
    let v = if st.v_pack > 1.0 { st.v_pack } else { q.v_full };
    let tmax = q.max_thrust(v);

    // ---- flight controller -------------------------------------------------
    let mut cmd = Vec3::new(q.rate(c.aileron, false), q.rate(c.elevator, false), q.rate(c.rudder, true));
    if angle {
        // Self-levelling: stick = tilt angle.
        let roll_t = c.aileron.clamp(-1.0, 1.0) * q.angle_max;
        let pitch_t = c.elevator.clamp(-1.0, 1.0) * q.angle_max;
        let lim = q.rate_max.to_radians() * 0.6;
        cmd.x = (q.angle_k * (roll_t - roll)).clamp(-lim, lim);
        cmd.y = (q.angle_k * (pitch_t - pitch)).clamp(-lim, lim);
        // Pitch rate in the body is about the rolled pitch axis.
        let (sr, cr) = roll.sin_cos();
        let (py, ry) = (cmd.y, cmd.z);
        cmd.y = py * cr + ry * sr;
        cmd.z = ry * cr - py * sr;
    }
    let e = cmd - gyro;
    let d_gyro = (gyro - st.last_gyro) / dt.max(1e-4);
    st.last_gyro = gyro;
    let grounded = ac.on_ground && c.throttle < 0.08;
    if grounded {
        st.i_err = Vec3::ZERO;
    } else {
        st.i_err = (st.i_err + e * dt).clamp(Vec3::splat(-0.6), Vec3::splat(0.6));
    }
    let acc = q.kp * e + q.ki * st.i_err - q.kd * d_gyro * 0.01;
    // Torque command (about p, q, r) → body torque vector (x = −q, y = −r, z = p).
    let tau_pqr = inertia * acc;
    let tau = if grounded { Vec3::ZERO } else { Vec3::new(-tau_pqr.y, -tau_pqr.z, tau_pqr.x) };
    let total = 4.0 * tmax * thr_curve(q, c.throttle);
    let mut t = mix(q, total, tau);
    // Airmode: keep the attitude authority by shifting the whole set, then clamp.
    let lo = q.idle * tmax;
    let (tmin_i, tmax_i) = t.iter().fold((f32::MAX, f32::MIN), |(a, b), &x| (a.min(x), b.max(x)));
    let span = tmax_i - tmin_i;
    let shift = if span > tmax - lo {
        // Not enough room: centre the spread in the range (lose some collective).
        (lo + tmax) * 0.5 - (tmin_i + tmax_i) * 0.5
    } else if tmax_i > tmax {
        tmax - tmax_i
    } else if tmin_i < lo {
        lo - tmin_i
    } else {
        0.0
    };
    for x in &mut t {
        *x = (*x + shift).clamp(lo, tmax);
    }

    // ---- motors and props --------------------------------------------------
    let failed = c.failed_engines;
    let area = q.disc_area();
    let w_max = q.omega_max * (v / q.v_full).clamp(0.2, 1.05);
    let mut force = Vec3::ZERO;
    let mut moment = Vec3::ZERO;
    let mut p_mech = 0.0;
    let mut wash: f32 = 0.0;
    let mut tsum = 0.0;
    for i in 0..4 {
        let off = failed & (1 << i) != 0;
        let target = if off { 0.0 } else { (t[i] / q.k_t).sqrt().min(w_max) };
        st.w[i] += (target - st.w[i]) * (dt / q.tau_motor).min(1.0);
        let wi = st.w[i].max(0.0);
        let r = q.motors[i];
        let vp = vb + ac.omega.cross(r);
        // Axial inflow (climbing into the air) unloads the prop.
        let pitch_speed = (wi * q.pitch_k).max(1.0);
        let unload = (1.0 - vp.y / pitch_speed).clamp(0.0, 1.4);
        let mut thrust = q.k_t * wi * wi * unload;
        // Prop wash: low-speed descent into the wake.
        let vh = (thrust / (2.0 * atm.density * area)).sqrt().max(0.5);
        let x = vp.y / vh;
        let horiz = (vp.x * vp.x + vp.z * vp.z).sqrt();
        if x < -0.2 && x > -2.2 {
            let bump = (1.0 - ((x + 1.0) / 0.8).powi(2)).max(0.0);
            let sev = bump * (1.0 - horiz / (1.5 * vh)).clamp(0.0, 1.0);
            let tt = st.time;
            let k = i as f32 * 1.7;
            let noise = (tt * (23.0 + k * 3.1) + k).sin() * 0.5 + (tt * (37.0 - k * 2.3) + 2.0 * k).sin() * 0.3 + (tt * 9.0 + k).sin() * 0.2;
            thrust *= 1.0 - 0.25 * sev + 0.45 * sev * noise;
            wash = wash.max(sev);
        }
        let thrust = thrust.max(0.0);
        tsum += thrust;
        let f = Vec3::Y * thrust;
        force += f;
        moment += r.cross(f);
        // Prop drag torque (reaction on the frame) about +Y.
        let qd = q.k_q * wi * wi;
        moment += Vec3::Y * (-q.spin[i] * qd);
        p_mech += qd * wi;
    }
    // Body and rotor drag.
    let qv = |x: f32| 0.5 * atm.density * x * x.abs();
    force -= Vec3::new(qv(vb.x) * q.cda.x, qv(vb.y) * q.cda.y, qv(vb.z) * q.cda.z);
    force -= Vec3::new(vb.x, 0.0, vb.z) * (q.h_drag * tsum);

    // ---- battery -----------------------------------------------------------
    let soc = (1.0 - st.used_mah / q.capacity_mah).clamp(0.0, 1.0);
    let voc = q.cells * QuadDef::cell_ocv(soc);
    let p_elec = p_mech / q.eta + q.p_avionics;
    let disc = (voc * voc - 4.0 * q.r_int * p_elec).max(0.0);
    let current = (voc - disc.sqrt()) / (2.0 * q.r_int);
    st.current = current;
    st.v_pack = voc - current * q.r_int;
    st.used_mah += current * dt / 3.6;

    // ---- outputs ------------------------------------------------------------
    let st = ac.rotor.quad;
    let o = &mut ac.rotor.out;
    let wm = st.w.iter().sum::<f32>() / 4.0;
    o.nr_rpm = wm * 60.0 / (2.0 * PI);
    o.nr_pct = wm / q.omega_max * 100.0;
    o.thrust = tsum;
    o.vrs = wash;
    o.batt_v = st.v_pack;
    o.batt_a = st.current;
    o.batt_mah = st.used_mah;
    o.batt_soc = (1.0 - st.used_mah / q.capacity_mah).clamp(0.0, 1.0);
    o.power_w = p_mech;
    for i in 0..4 {
        o.motors[i] = st.w[i] / q.omega_max;
    }
    o.eff = [c.throttle, c.elevator, c.aileron, c.rudder];
    for i in 0..4 {
        let rpm = st.w[i] * 60.0 / (2.0 * PI);
        ac.set_engine_gauges(
            i,
            rpm,
            st.w[i] / q.omega_max,
            25.0 + 40.0 * (st.w[i] / q.omega_max).powi(2),
            0.0,
            p_mech / 4.0,
            failed & (1 << i) == 0,
            q.k_t * st.w[i] * st.w[i],
            0.0,
        );
    }
    // Spinning props' net angular momentum is nearly zero (two each way).
    let h: f32 = (0..4).map(|i| q.spin[i] * st.w[i] * 2.0e-6).sum();
    Out { force, moment, damping: Vec3::new(def.inertia_pitch, def.inertia_yaw, def.inertia_roll) * 0.05, h: Vec3::Y * h, lift: tsum }
}
