//! Single-main-rotor helicopter flight model.
//!
//! Main rotor (blade element integrated over the disc, uniform inflow):
//!
//! * thrust `C_T = (aσ/2)[θ₀(1/3 + μ²/2) + θ_tw(1 + μ²)/4 − λ/2]` with the
//!   inflow `λ = (w + v_i)/ΩR` taken through the tip-path plane (TPP), so
//!   climb, descent, disc tilt and forward speed all change the thrust;
//! * induced velocity from Glauert's momentum theory (hover, climb, forward
//!   flight: translational lift emerges as `v_i` falls with speed), blended
//!   into the empirical induced-velocity curve in the vortex-ring / turbulent
//!   wake states at low speed (Leishman's fit), relaxed with a dynamic-inflow
//!   lag; Cheeseman–Bennett ground effect from the hub height;
//! * power `P = T(w + κ v_i) + σC_d/8 ρA(ΩR)³(1 + 4.65μ²)`: the same
//!   expression gives autorotation when the air flows up through the disc;
//! * blade stall: `C_T/σ` is limited by a boundary that falls with the
//!   advance ratio (retreating-blade stall: thrust saturates, the nose
//!   pitches up and the aircraft rolls towards the retreating side,
//!   vibration), profile drag rises with blade loading and tip Mach;
//! * flapping as a lagged quasi-steady TPP tilt: cyclic, blowback (speed
//!   stability), coning-induced lateral tilt, pitch / roll rate lag and
//!   gyroscopic cross-coupling; hub moment from the hinge offset (stiffness
//!   ∝ Ω²) plus the thrust line;
//! * rotor speed is a state: `I_R dΩ/dt = Q_drive − Q_aero − Q_tail`.
//!
//! Engine: free-turbine turboshaft (power available with density /
//! temperature, transmission torque limit, gas-generator lag, Ng / TOT /
//! torque / FLI) under a FADEC governor holding Nr with collective
//! anticipation; the freewheel lets the rotor run on when the engine stops.
//! Tail rotor: shrouded (Fenestron) fan with pedal pitch, its own inflow
//! (yaw damping, weathercock), geared to the main rotor; drive and pitch
//! failures. Fuselage drag / download, vertical fin, horizontal stabiliser
//! in the rotor wake (pitch-up in the transition).

use super::{Assist, Out, RotorFailures};
use crate::flight::{Aircraft, AircraftDef, Atmosphere, Controls, GRAVITY};
use glam::{Quat, Vec3};
use std::f32::consts::PI;

/// Free-turbine speed at 100 % N2, rpm (Arriel 2B1 class), for gauges.
pub const N2_RPM_100: f32 = 39_000.0;

/// Static helicopter data.
#[derive(Clone, Debug)]
pub struct HeliDef {
    // ---- main rotor
    pub radius: f32,
    pub blades: f32,
    pub chord: f32,
    /// Nominal rotor speed, rad/s (100 % Nr).
    pub omega: f32,
    pub lift_slope: f32,
    /// Linear blade twist root → tip, rad (negative = washout).
    pub twist: f32,
    /// Profile drag: Cd = cd0 + cd2·Cl_mean².
    pub cd0: f32,
    pub cd2: f32,
    /// Induced power factor.
    pub kappa: f32,
    /// Collective pitch at 0.75 R, rad, at lever 0 and 1.
    pub coll_min: f32,
    pub coll_max: f32,
    /// TPP tilt (rad) at full longitudinal / lateral cyclic.
    pub cyc_lon: f32,
    pub cyc_lat: f32,
    /// Hub (body frame) and forward mast tilt, rad.
    pub hub: Vec3,
    pub mast_tilt: f32,
    /// Hub moment per radian of TPP tilt at 100 % Nr (hinge offset), N·m/rad.
    pub hub_k: f32,
    /// Lock number.
    pub lock: f32,
    /// Rotor (+ drive train) polar inertia, kg·m².
    pub inertia: f32,
    /// +1 = anticlockwise seen from above (US / Russian practice is mixed;
    /// Airbus Helicopters: clockwise = −1).
    pub rot_dir: f32,
    /// Blade-loading stall limit C_T/σ in hover.
    pub ct_sigma_max: f32,
    // ---- tail rotor (Fenestron)
    pub tr_pos: Vec3,
    pub tr_radius: f32,
    pub tr_solidity: f32,
    /// Tail rotor speed / main rotor speed.
    pub tr_gear: f32,
    /// Blade pitch at neutral pedal (magnitude; the sign follows `rot_dir`) and per full pedal, rad.
    pub tr_pitch_bias: f32,
    pub tr_pitch_range: f32,
    /// Shroud thrust augmentation (1 = open rotor).
    pub tr_duct: f32,
    // ---- engine (turboshaft) and transmission
    /// Thermodynamic take-off power at ISA sea level, W; max continuous, W.
    pub p_takeoff: f32,
    pub p_mcp: f32,
    /// Main gearbox limit at 100 % Nr, W (= 100 % torque).
    pub p_mgb: f32,
    pub eta: f32,
    /// Accessory / gearbox losses, W.
    pub p_access: f32,
    pub ng_idle: f32,
    /// Gas-generator time constants (spool up / down), s.
    pub tau_up: f32,
    pub tau_down: f32,
    /// kg/(kW·h).
    pub sfc: f32,
    pub tot_idle: f32,
    pub tot_max: f32,
    // ---- fuselage
    /// Drag areas (flat plate, m²): forward, side, vertical.
    pub f_front: f32,
    pub f_side: f32,
    pub f_top: f32,
    /// Hover download as a fraction of thrust.
    pub download: f32,
    pub fin_area: f32,
    pub fin_pos: Vec3,
    /// Fin side-force coefficient at zero sideslip (cambered fin, sign: +X).
    pub fin_cl0: f32,
    pub hstab_area: f32,
    pub hstab_pos: Vec3,
    pub hstab_incidence: f32,
    /// Rotor speed warning thresholds (fraction of nominal).
    pub nr_low: f32,
    pub nr_high: f32,
}

impl HeliDef {
    pub fn disc_area(&self) -> f32 {
        PI * self.radius * self.radius
    }
    pub fn solidity(&self) -> f32 {
        self.blades * self.chord / (PI * self.radius)
    }
    fn shaft(&self) -> (Vec3, Vec3) {
        let (s, c) = self.mast_tilt.sin_cos();
        (Vec3::new(0.0, c, s), Vec3::new(0.0, -s, c))
    }
    /// Power available (W) at density ratio `sigma` and OAT, at rotor speed fraction `nr`.
    pub fn power_available(&self, sigma: f32, oat_c: f32, isa_c: f32, nr: f32, rating: f32) -> f32 {
        // Turboshaft at its temperature limit: P ∝ δ/√θ = σ√θ (plus a hot-day loss).
        let theta = (oat_c + 273.15) / 288.15;
        let thermo = rating * sigma.max(0.05) * theta.sqrt() * (1.0 - 0.004 * (oat_c - isa_c)).clamp(0.6, 1.1);
        thermo.min(self.p_mgb * nr.clamp(0.0, 1.1))
    }
}

/// Stability-assist controller state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sas {
    pub lon_i: f32,
    pub lat_i: f32,
    pub ped_i: f32,
    /// Attitudes held with the stick centred (rad), heading held with the pedals centred.
    pub pitch_ref: Option<f32>,
    pub roll_ref: Option<f32>,
    pub hdg_ref: Option<f32>,
    /// Learned hover attitudes (velocity hold).
    pub pitch_bias: f32,
    pub roll_bias: f32,
    /// Hydraulics-off control positions (slow servos).
    pub servo: [f32; 4],
    /// In the hover regime (attitude command / velocity hold).
    pub hover: bool,
}

/// Live helicopter state.
#[derive(Clone, Copy, Debug, Default)]
pub struct HeliState {
    /// Rotor speed, rad/s.
    pub nr: f32,
    /// Induced velocities (main, tail), m/s.
    pub vi: f32,
    pub tr_vi: f32,
    /// TPP tilt relative to the shaft: aft, right (rad).
    pub a1: f32,
    pub b1: f32,
    /// Engine shaft power, W; gas generator %, TOT °C; free turbine fraction of nominal.
    pub power: f32,
    pub ng: f32,
    pub tot: f32,
    pub n2: f32,
    pub gov_i: f32,
    /// Tail-rotor pitch (rad) when the pitch control failed.
    pub tail_frozen: Option<f32>,
    pub azimuth: f32,
    pub tail_azimuth: f32,
    pub time: f32,
    pub sas: Sas,
    pub init: bool,
}

/// Effective controls into the rotor: collective 0..1, cyclic aft / right and pedal right, −1..1.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Inp {
    pub coll: f32,
    pub lon: f32,
    pub lat: f32,
    pub ped: f32,
}

/// One evaluation of the rotor and airframe aerodynamics.
#[derive(Clone, Copy, Debug, Default)]
pub struct Ev {
    pub force: Vec3,
    pub moment: Vec3,
    pub thrust: f32,
    /// Aerodynamic torque on the main rotor, N·m; tail rotor power, W.
    pub q_main: f32,
    pub p_main: f32,
    pub p_tail: f32,
    pub vi_target: f32,
    pub tr_vi_target: f32,
    pub a1_ss: f32,
    pub b1_ss: f32,
    pub tau_flap: f32,
    pub mu: f32,
    pub vrs: f32,
    pub stall: f32,
    pub coning: f32,
    pub theta75: f32,
    pub tail_pitch: f32,
    pub tail_thrust: f32,
}

/// Leishman's fit of the measured induced velocity in the vortex-ring /
/// turbulent-wake states (`x` = axial velocity / v_h, −2..0), normalised to 1 in hover.
fn vrs_curve(x: f32) -> f32 {
    1.0 - 1.125 * x - 1.372 * x * x - 1.718 * x * x * x - 0.655 * x * x * x * x
}

/// Momentum-theory induced velocity for thrust `t` through a disc of area
/// `area` with in-plane speed `vp` and axial speed `w` (+ = climbing, air down
/// through the disc), Newton on Glauert's equation from `guess`. Also returns
/// the vortex-ring severity (0..1).
fn induced(t: f32, rho: f32, area: f32, vp: f32, w: f32, guess: f32) -> (f32, f32) {
    let s = t.signum();
    let vh2 = t.abs() / (2.0 * rho * area);
    let vh = vh2.sqrt();
    if vh < 1e-3 {
        return (0.0, 0.0);
    }
    // Work in the thrust direction (a negative thrust mirrors the problem).
    let w = w * s;
    let mut v = (guess * s).max(0.05 * vh);
    for _ in 0..12 {
        let r = (vp * vp + (w + v) * (w + v)).sqrt().max(1e-3);
        let g = v * r - vh2;
        let dg = r + v * (w + v) / r;
        let nv = (v - g / dg.max(1e-3)).max(0.0);
        if (nv - v).abs() < 1e-4 {
            v = nv;
            break;
        }
        v = 0.5 * (v + nv);
    }
    // Low speed descent: blend into the measured curve (the momentum
    // solution has no physical meaning in the vortex ring state).
    let x = w / vh;
    let mut sev = 0.0;
    if x < 0.0 && x > -2.0 {
        let wgt = (1.0 - vp / (1.2 * vh)).clamp(0.0, 1.0).powi(2);
        if wgt > 0.0 {
            let emp = vh * vrs_curve(x);
            v = v + (emp - v) * wgt;
            // Severity peaks around a descent rate of ~0.9 v_h.
            let bump = (1.0 - ((x + 0.9) / 0.6).powi(2)).max(0.0);
            sev = wgt * bump;
        }
    }
    (v * s, sev)
}

/// Rotor + airframe aerodynamics at a given state (pure).
#[allow(clippy::too_many_arguments)]
pub fn eval(h: &HeliDef, st: &HeliState, inp: Inp, vb: Vec3, w: Vec3, rho: f32, sound: f32, hub_agl: f32, fail: RotorFailures) -> Ev {
    let mut ev = Ev::default();
    let omega = st.nr.max(1.0);
    let r = h.radius;
    let vt = omega * r;
    let area = h.disc_area();
    let sigma = h.solidity();
    let a = h.lift_slope;
    let (n_s, fwd_h) = h.shaft();
    let left = Vec3::X;
    let n = (n_s - st.a1 * fwd_h - st.b1 * left).normalize();

    // Hub motion through the air, split along / across the TPP.
    let vh = vb + w.cross(h.hub);
    let wn = vh.dot(n);
    let v_par = vh - n * wn;
    let vp = v_par.length();
    let mu = (vp / vt).min(1.0);
    ev.mu = mu;

    // ---- thrust (blade element) ------------------------------------------
    let theta75 = h.coll_min + inp.coll.clamp(0.0, 1.0) * (h.coll_max - h.coll_min);
    let theta0 = theta75 - 0.75 * h.twist;
    ev.theta75 = theta75;
    let lambda = (wn + st.vi) / vt;
    let mut ct = 0.5 * a * sigma * (theta0 * (1.0 / 3.0 + 0.5 * mu * mu) + 0.25 * h.twist * (1.0 + mu * mu) - 0.5 * lambda);
    // Blade stall: the C_T/σ boundary falls with advance ratio (retreating blade).
    let lim = h.ct_sigma_max * sigma * (1.0 - 1.1 * mu * mu).max(0.35);
    let mut stall = 0.0;
    if ct > lim {
        stall = ((ct / lim - 1.0) / 0.25).clamp(0.0, 1.0);
        ct = lim + (ct - lim) * 0.15;
    } else if ct < -lim {
        ct = -lim + (ct + lim) * 0.15;
    }
    ev.stall = stall;
    let thrust = ct * rho * area * vt * vt;
    ev.thrust = thrust;

    // ---- inflow target: momentum / vortex ring, ground effect ---------------
    let (mut vi_t, vrs) = induced(thrust, rho, area, vp, wn, st.vi);
    if hub_agl < 4.0 * r {
        let vh0 = (thrust.abs() / (2.0 * rho * area)).sqrt().max(1.0);
        let z = hub_agl.max(0.45 * r);
        let k = (r / (4.0 * z)).powi(2) / (1.0 + (vp / vh0).powi(2));
        vi_t *= (1.0 - k).max(0.55);
    }
    ev.vi_target = vi_t;
    ev.vrs = vrs;

    // ---- power and torque -------------------------------------------------
    let cl_mean = 6.0 * ct / sigma.max(1e-3);
    let tip_mach = (vt + vp) / sound.max(200.0);
    let cd = h.cd0 + h.cd2 * cl_mean * cl_mean + 0.04 * stall + 0.4 * (tip_mach - 0.88).max(0.0).powi(2) * 10.0;
    let p0 = sigma * cd / 8.0 * (1.0 + 4.65 * mu * mu) * rho * area * vt * vt * vt;
    let p_main = thrust * (wn + h.kappa * st.vi) + p0;
    ev.p_main = p_main;
    ev.q_main = p_main / omega;

    // ---- flapping: quasi-steady TPP tilt -----------------------------------
    let g = h.lock;
    let coning = (g / 8.0 * (theta0 + 0.8 * h.twist - 4.0 / 3.0 * lambda) - 0.02).clamp(-0.03, 0.14);
    ev.coning = coning;
    let p = w.z;
    let q = -w.x;
    let mut a1 = inp.lon.clamp(-1.0, 1.0) * h.cyc_lon - 16.0 / g * q / omega + h.rot_dir * p / omega;
    let mut b1 = inp.lat.clamp(-1.0, 1.0) * h.cyc_lat - 16.0 / g * p / omega - h.rot_dir * q / omega;
    if vp > 0.1 {
        let u = v_par / vp;
        // Blowback: the disc tilts away from the relative wind (speed stability).
        let bb = 0.8 * 2.0 * mu * (4.0 / 3.0 * theta0 + h.twist - lambda) / (1.0 - 0.5 * mu * mu);
        // Coning: tilts towards the advancing side.
        let adv = -h.rot_dir * n.cross(u);
        let lat = 4.0 / 3.0 * mu * coning / (1.0 + 0.5 * mu * mu);
        let tilt = -u * bb + adv * lat;
        a1 += -tilt.dot(fwd_h);
        b1 += -tilt.dot(left);
    }
    ev.a1_ss = a1.clamp(-0.35, 0.35);
    ev.b1_ss = b1.clamp(-0.35, 0.35);
    ev.tau_flap = 16.0 / (g * omega);

    // ---- main rotor forces / moments -------------------------------------
    let nr_frac = omega / h.omega;
    let h_force = if vp > 0.1 { -(v_par / vp) * (sigma * cd * mu / 4.0) * rho * area * vt * vt } else { Vec3::ZERO };
    let f_main = n * thrust + h_force;
    let mut moment = h.hub.cross(f_main) + n_s.cross(n) * (h.hub_k * nr_frac * nr_frac);
    // Retreating-blade stall: pitch up, roll towards the retreating side.
    if stall > 0.0 && mu > 0.15 {
        let k = stall * thrust.abs() * 0.08 * r;
        moment += Vec3::new(-k, 0.0, -h.rot_dir * 0.7 * k);
    }
    // Vortex ring: unsteady thrust and rocking.
    let t = st.time;
    let mut force = f_main;
    if vrs > 0.02 {
        let n1 = (t * 7.3).sin() * 0.6 + (t * 11.9 + 1.3).sin() * 0.4;
        let n2 = (t * 5.1 + 0.4).sin() * 0.5 + (t * 13.7).sin() * 0.5;
        let n3 = (t * 6.7 + 2.1).sin() * 0.6 + (t * 9.3).cos() * 0.4;
        force += n * (thrust * 0.18 * vrs * n1);
        moment += Vec3::new(n2, 0.0, n3) * (vrs * 0.06 * thrust.abs() * r * 0.25);
    }

    // ---- tail rotor ---------------------------------------------------------
    let tr_vt = (omega * h.tr_gear * h.tr_radius).max(1.0);
    let tr_area = PI * h.tr_radius * h.tr_radius;
    let tail_pitch = match (fail.tail_pitch, st.tail_frozen) {
        (true, Some(p)) => p,
        _ => -h.rot_dir * h.tr_pitch_bias + inp.ped.clamp(-1.0, 1.0) * h.tr_pitch_range,
    };
    ev.tail_pitch = tail_pitch;
    let v_tr = vb + w.cross(h.tr_pos);
    let (mut tr_thrust, mut p_tail, mut tr_vi_t) = (0.0, 0.0, 0.0);
    if !fail.tail_drive {
        let e = Vec3::X;
        let wn_tr = v_tr.dot(e);
        let vp_tr = (v_tr - e * wn_tr).length();
        let mu_tr = vp_tr / tr_vt;
        let lam = (wn_tr + st.tr_vi) / tr_vt;
        let ct_tr = 0.5 * a * h.tr_solidity * (tail_pitch * (1.0 / 3.0 + 0.5 * mu_tr * mu_tr) - 0.5 * lam);
        // Fan blade stall: soft saturation (loss of tail-rotor effectiveness).
        let lim_tr = 0.17 * h.tr_solidity;
        let ct_tr = if ct_tr.abs() > lim_tr { ct_tr.signum() * (lim_tr + (ct_tr.abs() - lim_tr) * 0.15) } else { ct_tr };
        let fan = ct_tr * rho * tr_area * tr_vt * tr_vt;
        tr_thrust = fan * h.tr_duct;
        tr_vi_t = induced(fan, rho, tr_area * h.tr_duct, vp_tr, wn_tr, st.tr_vi).0;
        let cl_tr = 6.0 * ct_tr / h.tr_solidity;
        let p0_tr = h.tr_solidity * (0.009 + 0.01 * cl_tr * cl_tr) / 8.0 * (1.0 + 4.65 * mu_tr * mu_tr) * rho * tr_area * tr_vt.powi(3);
        p_tail = fan * (wn_tr + 1.15 * st.tr_vi) + p0_tr;
        let f = e * tr_thrust;
        force += f;
        moment += h.tr_pos.cross(f);
    }
    ev.tail_thrust = tr_thrust;
    ev.p_tail = p_tail;
    ev.tr_vi_target = tr_vi_t;

    // ---- fuselage ---------------------------------------------------------------
    let qv = |x: f32| 0.5 * rho * x * x.abs();
    let mut f_fus = -Vec3::new(qv(vb.x) * h.f_side, qv(vb.y) * h.f_top, qv(vb.z) * h.f_front);
    // Download: the rotor wake on the cabin roof, fading as the wake skews back.
    let skew = 1.0 / (1.0 + (vp / st.vi.abs().max(1.0)).powi(2));
    f_fus -= n_s * (h.download * thrust.max(0.0) * skew);
    force += f_fus;
    // Vertical fin (side force towards +X for air from the right / cambered section).
    let v_fin = vb + w.cross(h.fin_pos);
    let qf = 0.5 * rho * (v_fin.z * v_fin.z + v_fin.x * v_fin.x);
    if qf > 1.0 {
        let beta = v_fin.x.atan2(v_fin.z.abs().max(0.1));
        let cl = (h.fin_cl0 * v_fin.z.signum().max(0.0) - 3.5 * beta.sin()).clamp(-1.1, 1.1);
        let f = Vec3::X * (qf * h.fin_area * cl);
        force += f;
        moment += h.fin_pos.cross(f);
    }
    // Horizontal stabiliser, in the rotor wake at low speed.
    let mut v_hs = vb + w.cross(h.hstab_pos);
    let wake_on_tail = smooth(vp, 4.0, 11.0) * (1.0 - smooth(vp, 18.0, 32.0));
    v_hs += n_s * (2.0 * st.vi.max(0.0) * wake_on_tail);
    let qh = 0.5 * rho * (v_hs.z * v_hs.z + v_hs.y * v_hs.y);
    if qh > 1.0 {
        let alpha = (-v_hs.y).atan2(v_hs.z.abs().max(0.1)) + h.hstab_incidence;
        let cl = (3.5 * alpha.sin()).clamp(-1.0, 1.0) * v_hs.z.signum().max(0.0);
        let f = Vec3::Y * (qh * h.hstab_area * cl) - Vec3::Z * (qh * h.hstab_area * 0.02 * v_hs.z.signum());
        force += f;
        moment += h.hstab_pos.cross(f);
    }
    ev.force = force;
    ev.moment = moment;
    ev
}

fn smooth(x: f32, a: f32, b: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Iterate the fast states (inflow, flapping) to equilibrium at fixed Ω.
pub fn steady(h: &HeliDef, st: &mut HeliState, inp: Inp, vb: Vec3, rho: f32, sound: f32, hub_agl: f32) -> Ev {
    let mut ev = Ev::default();
    for _ in 0..80 {
        ev = eval(h, st, inp, vb, Vec3::ZERO, rho, sound, hub_agl, RotorFailures::default());
        st.vi += (ev.vi_target - st.vi) * 0.5;
        st.tr_vi += (ev.tr_vi_target - st.tr_vi) * 0.5;
        st.a1 += (ev.a1_ss - st.a1) * 0.7;
        st.b1 += (ev.b1_ss - st.b1) * 0.7;
    }
    ev
}

/// A trimmed flight condition.
#[derive(Clone, Copy, Debug, Default)]
pub struct Trim {
    pub inp: Inp,
    pub pitch: f32,
    pub roll: f32,
    /// Engine power required (shaft, W) and its parts.
    pub power: f32,
    pub p_main: f32,
    pub p_tail: f32,
    pub state: HeliState,
    pub residual: f32,
    pub converged: bool,
}

/// Steady trim at true airspeed `tas` (forward, horizontal) and climb rate
/// `vs` (m/s, + up), density `rho`, mass `mass`, hub height `hub_agl`.
pub fn trim(h: &HeliDef, mass: f32, tas: f32, vs: f32, rho: f32, sound: f32, hub_agl: f32) -> Trim {
    // Continuation in speed from the hover (robust at high speed).
    let steps = (tas / 12.0).ceil().max(1.0) as usize;
    let mut t = trim_from(h, None, mass, 0.0, vs, rho, sound, hub_agl);
    for k in 1..=steps {
        if tas <= 0.0 {
            break;
        }
        t = trim_from(h, Some(&t), mass, tas * k as f32 / steps as f32, vs, rho, sound, hub_agl);
    }
    t
}

/// [`trim`] starting from a nearby solution.
#[allow(clippy::too_many_arguments)]
pub fn trim_from(h: &HeliDef, guess: Option<&Trim>, mass: f32, tas: f32, vs: f32, rho: f32, sound: f32, hub_agl: f32) -> Trim {
    let weight = mass * GRAVITY;
    let mut st = guess.map(|g| g.state).unwrap_or(HeliState { nr: h.omega, vi: 8.0, tr_vi: 5.0, ..Default::default() });
    // Unknowns: collective, lon, lat, pedal, pitch, roll.
    let mut x = guess.map(|g| [g.inp.coll, g.inp.lon, g.inp.lat, g.inp.ped, g.pitch, g.roll]).unwrap_or([0.55f32, -0.2, 0.0, 0.3, 0.06, 0.06]);
    let resid = |x: &[f32; 6], st: &mut HeliState| -> ([f32; 6], Ev) {
        let rot = crate::flight::attitude(0.0, x[4], x[5]);
        let inv = rot.conjugate();
        let v_world = Vec3::new(0.0, vs, -tas);
        // Body velocity: heading north (−Z).
        let vb = inv * v_world;
        let inp = Inp { coll: x[0], lon: x[1], lat: x[2], ped: x[3] };
        let ev = steady(h, st, inp, vb, rho, sound, hub_agl);
        let g_b = inv * Vec3::new(0.0, -weight, 0.0);
        let f = ev.force + g_b;
        // Steady drive torque reacts on the fuselage (governed at 100 % Nr).
        let q_drive = ev.q_main + (ev.p_tail + h.p_access) / h.omega;
        let m = ev.moment - h.shaft().0 * (h.rot_dir * q_drive);
        ([f.x / weight, f.y / weight, f.z / weight, m.x / (weight * 2.0), m.y / (weight * 2.0), m.z / (weight * 2.0)], ev)
    };
    let mut converged = false;
    let mut res = 1.0;
    let mut ev = Ev::default();
    for _ in 0..40 {
        let (r0, e0) = resid(&x, &mut st);
        ev = e0;
        res = r0.iter().map(|v| v * v).sum::<f32>().sqrt();
        if std::env::var_os("TRIM_DEBUG").is_some() {
            eprintln!("trim res {res:.2e} x {x:?} r {r0:?}");
        }
        if res < 2e-4 {
            converged = true;
            break;
        }
        let mut jac = [[0.0f32; 6]; 6];
        for j in 0..6 {
            let mut xp = x;
            let d = 1e-3;
            xp[j] += d;
            let mut s2 = st;
            let (r1, _) = resid(&xp, &mut s2);
            for i in 0..6 {
                jac[i][j] = (r1[i] - r0[i]) / d;
            }
        }
        let Some(dx) = solve6(jac, r0) else { break };
        let lims = [0.1, 0.2, 0.2, 0.2, 0.05, 0.05];
        // Damped Newton: halve the step until the residual drops.
        let mut k = 1.0;
        let mut accepted = false;
        for _ in 0..6 {
            let mut xn = x;
            for j in 0..6 {
                xn[j] -= (dx[j] * k).clamp(-lims[j], lims[j]);
            }
            xn[0] = xn[0].clamp(-0.3, 1.3);
            for v in &mut xn[1..4] {
                *v = v.clamp(-1.5, 1.5);
            }
            let mut s2 = st;
            let (r1, _) = resid(&xn, &mut s2);
            let n1 = r1.iter().map(|v| v * v).sum::<f32>().sqrt();
            if n1 < res {
                x = xn;
                st = s2;
                accepted = true;
                break;
            }
            k *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    let p_req = (ev.p_main + ev.p_tail + h.p_access) / h.eta;
    Trim {
        inp: Inp { coll: x[0], lon: x[1], lat: x[2], ped: x[3] },
        pitch: x[4],
        roll: x[5],
        power: p_req,
        p_main: ev.p_main,
        p_tail: ev.p_tail,
        state: st,
        residual: res,
        converged,
    }
}

fn solve6(mut a: [[f32; 6]; 6], mut b: [f32; 6]) -> Option<[f32; 6]> {
    for c in 0..6 {
        let p = (c..6).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-9 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in 0..6 {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in c..6 {
                    a[r][k] -= f * a[c][k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    let mut x = [0.0; 6];
    for i in 0..6 {
        x[i] = b[i] / a[i][i];
    }
    Some(x)
}

fn wrap(a: f32) -> f32 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

/// Engine state for steady shaft power `p` (gauges).
fn engine_gauges(h: &HeliDef, st: &mut HeliState, p: f32, p_ref: f32, oat: f32) {
    let x = (p / p_ref.max(1.0)).clamp(0.0, 1.2);
    st.ng = h.ng_idle + (101.0 - h.ng_idle) * x.powf(0.6);
    st.tot = h.tot_idle + (h.tot_max - h.tot_idle) * x.powf(1.3) + (oat - 15.0) * 0.9;
}

/// Spawn: rotors at flight RPM; in the air, trimmed at the current speed.
pub fn settle(h: &HeliDef, def: &AircraftDef, ac: &mut Aircraft, ctl: &mut Controls, airborne: bool) {
    let atm = crate::flight::isa(ac.pos.y);
    let st = &mut ac.rotor.heli;
    *st = HeliState { nr: h.omega, n2: 1.0, init: true, ..Default::default() };
    let oat = atm.temperature_c();
    let p_ref = h.power_available(atm.density / 1.225, oat, oat, 1.0, h.p_takeoff).max(1.0);
    if airborne {
        let tas = ac.vel.length();
        let tr = trim(h, def.mass, tas, 0.0, atm.density, atm.speed_of_sound, 1e4);
        let hdg = ac.heading();
        ac.rot = crate::flight::attitude(hdg, tr.pitch, tr.roll);
        let st = &mut ac.rotor.heli;
        st.vi = tr.state.vi;
        st.tr_vi = tr.state.tr_vi;
        st.a1 = tr.state.a1;
        st.b1 = tr.state.b1;
        st.power = tr.power;
        st.sas.pitch_bias = tr.pitch;
        st.sas.roll_bias = tr.roll;
        st.sas.pitch_ref = Some(tr.pitch);
        st.sas.roll_ref = Some(tr.roll);
        st.sas.hdg_ref = Some(hdg);
        st.sas.lon_i = tr.inp.lon;
        st.sas.lat_i = tr.inp.lat;
        st.sas.ped_i = tr.inp.ped;
        st.sas.servo = [tr.inp.coll, tr.inp.lon, tr.inp.lat, tr.inp.ped];
        ctl.throttle = tr.inp.coll;
        if ac.rotor.assist != Assist::Att {
            ctl.elevator = tr.inp.lon;
            ctl.aileron = tr.inp.lat;
            ctl.rudder = tr.inp.ped;
        }
        engine_gauges(h, &mut ac.rotor.heli, tr.power, p_ref, oat);
    } else {
        // Flat pitch, flight idle governed at 100 % Nr.
        let p = 0.12 * h.p_mgb;
        st.power = p;
        let hover = trim(h, def.mass, 0.0, 0.0, atm.density, atm.speed_of_sound, 2.5);
        st.sas.pitch_bias = hover.pitch;
        st.sas.roll_bias = hover.roll;
        st.sas.lon_i = hover.inp.lon;
        st.sas.lat_i = hover.inp.lat;
        st.sas.ped_i = 0.0;
        engine_gauges(h, st, p, p_ref, oat);
        ctl.throttle = 0.0;
    }
    // Gauges before the first step.
    let st = ac.rotor.heli;
    let o = &mut ac.rotor.out;
    o.nr_pct = 100.0;
    o.nr_rpm = h.omega * 60.0 / (2.0 * PI);
    o.n2_pct = 100.0;
    o.ng_pct = st.ng;
    o.tot_c = st.tot;
    o.power_w = st.power;
    o.torque_pct = st.power / h.p_mgb * 100.0;
    o.tail_rpm = h.omega * h.tr_gear * 60.0 / (2.0 * PI);
}

/// Pilot controls → rotor inputs (stability assist).
fn assist(h: &HeliDef, def: &AircraftDef, ac: &mut Aircraft, c: &Controls, thrust_w: f32, dt: f32) -> Inp {
    let mode = if ac.rotor.fail.hydraulics { Assist::Off } else { ac.rotor.assist };
    let (p, q, r) = (ac.omega.z, -ac.omega.x, -ac.omega.y);
    let pitch = ac.pitch();
    let roll = ac.roll();
    let hdg = ac.heading();
    let beta = ac.beta();
    let on_ground = ac.on_ground;
    let fwd = ac.forward();
    let right = ac.right();
    let gs_f = Vec3::new(fwd.x, 0.0, fwd.z).normalize_or_zero().dot(ac.vel);
    let gs_r = Vec3::new(right.x, 0.0, right.z).normalize_or_zero().dot(ac.vel);
    let gs = (gs_f * gs_f + gs_r * gs_r).sqrt();
    let tas = ac.tas();
    let st = &mut ac.rotor.heli;
    let mut out = Inp { coll: c.throttle, lon: c.elevator + 0.35 * c.trim, lat: c.aileron, ped: c.rudder };
    // Control powers (per unit input) for gain scheduling.
    let nr = (st.nr / h.omega).clamp(0.3, 1.2);
    let m_lon = h.cyc_lon * (h.hub_k * nr * nr + thrust_w * h.hub.y);
    let m_lat = h.cyc_lat * (h.hub_k * nr * nr + thrust_w * h.hub.y);
    let tr_vt = st.nr * h.tr_gear * h.tr_radius;
    let n_ped = h.tr_pitch_range * (5.7 * h.tr_solidity / 6.0) * 1.225 * PI * h.tr_radius * h.tr_radius * tr_vt * tr_vt * h.tr_duct * (-h.tr_pos.z);
    let (iyy, ixx, izz) = (def.inertia_pitch, def.inertia_roll, def.inertia_yaw);
    // On the ground the assist fades in as the skids get light.
    let light = if on_ground { ((thrust_w / (def.mass * GRAVITY) - 0.75) / 0.2).clamp(0.0, 1.0) } else { 1.0 };
    match mode {
        Assist::Off => {}
        Assist::Sas => {
            // Rate damping with limited (±30 %) authority, yaw damper.
            let kq = 0.6 * iyy / m_lon.max(1.0) * 4.0;
            let kp = 0.6 * ixx / m_lat.max(1.0) * 4.0;
            let r_turn = if tas > 15.0 { GRAVITY * roll.sin() / tas } else { 0.0 };
            let kr = izz / n_ped.max(1.0) * 3.0;
            out.lon += (-kq * q).clamp(-0.3, 0.3) * light;
            out.lat += (-kp * p).clamp(-0.3, 0.3) * light;
            out.ped += (-kr * (r - r_turn)).clamp(-0.3, 0.3) * light;
        }
        Assist::Att => {
            let s_lon = c.elevator;
            let s_lat = c.aileron;
            let s_ped = c.rudder;
            let centred_lon = s_lon.abs() < 0.04;
            let centred_lat = s_lat.abs() < 0.04;
            // Hover regime (with hysteresis): attitude command + velocity hold.
            st.sas.hover = if st.sas.hover { gs < 10.0 && tas < 16.0 } else { gs < 7.0 && tas < 12.0 };
            let (q_cmd, p_cmd) = if st.sas.hover {
                st.sas.pitch_ref = None;
                st.sas.roll_ref = None;
                let pitch_cmd = st.sas.pitch_bias + if centred_lon && centred_lat { (0.045 * gs_f).clamp(-0.18, 0.18) } else { s_lon * 0.30 };
                let roll_cmd = st.sas.roll_bias + if centred_lon && centred_lat { (-0.045 * gs_r).clamp(-0.18, 0.18) } else { s_lat * 0.35 };
                (2.5 * wrap(pitch_cmd - pitch), 3.0 * wrap(roll_cmd - roll))
            } else {
                // Forward flight: rate command, attitude hold.
                let q_cmd = if centred_lon {
                    let r = *st.sas.pitch_ref.get_or_insert(pitch);
                    2.5 * wrap(r - pitch)
                } else {
                    st.sas.pitch_ref = None;
                    s_lon * 0.5
                };
                let p_cmd = if centred_lat {
                    let mut r = *st.sas.roll_ref.get_or_insert(roll);
                    // Shallow banks roll back to wings level.
                    if r.abs() < 0.09 {
                        r *= 1.0 - dt * 0.6;
                        st.sas.roll_ref = Some(r);
                    }
                    3.0 * wrap(r - roll)
                } else {
                    st.sas.roll_ref = None;
                    s_lat * 0.9
                };
                (q_cmd, p_cmd)
            };
            // Attitude envelope: ±60° bank, +30° / −35° pitch.
            let q_cmd = q_cmd.min(2.0 * (0.52 - pitch)).max(2.0 * (-0.6 - pitch));
            let p_cmd = p_cmd.min(2.0 * (1.05 - roll)).max(2.0 * (-1.05 - roll));
            // Yaw: heading hold / turn coordination.
            let r_turn = if tas > 20.0 { GRAVITY * roll.sin() / tas } else { 0.0 };
            let r_cmd = if s_ped.abs() > 0.04 {
                st.sas.hdg_ref = None;
                r_turn + s_ped * if tas > 25.0 { 0.25 } else { 0.9 }
            } else if tas > 25.0 {
                st.sas.hdg_ref = None;
                r_turn + 1.5 * beta
            } else {
                let href = *st.sas.hdg_ref.get_or_insert(hdg);
                2.0 * wrap(href - hdg)
            };
            // Inner loop: rate → cyclic / pedal (gain-scheduled by control power).
            let kq = 4.0 * iyy / m_lon.max(1.0);
            let kp = 5.0 * ixx / m_lat.max(1.0);
            let kr = 3.0 * izz / n_ped.max(1.0);
            let eq = q_cmd - q;
            let ep = p_cmd - p;
            let er = r_cmd - r;
            if light > 0.5 {
                st.sas.lon_i = (st.sas.lon_i + 1.5 * kq * eq * dt).clamp(-1.0, 1.0);
                st.sas.lat_i = (st.sas.lat_i + 1.5 * kp * ep * dt).clamp(-1.0, 1.0);
                st.sas.ped_i = (st.sas.ped_i + 1.0 * kr * er * dt).clamp(-1.0, 1.0);
            }
            let a_lon = st.sas.lon_i + kq * eq;
            let a_lat = st.sas.lat_i + kp * ep;
            let a_ped = st.sas.ped_i + kr * er;
            // On the ground: the pilot's hands until the skids are light.
            out.lon = a_lon * light + out.lon * (1.0 - light);
            out.lat = a_lat * light + out.lat * (1.0 - light);
            out.ped = a_ped * light + s_ped * (1.0 - light);
            if light < 0.5 {
                st.sas.pitch_ref = None;
                st.sas.roll_ref = None;
                st.sas.hdg_ref = None;
            }
        }
    }
    out.lon = out.lon.clamp(-1.0, 1.0);
    out.lat = out.lat.clamp(-1.0, 1.0);
    out.ped = out.ped.clamp(-1.0, 1.0);
    out.coll = out.coll.clamp(0.0, 1.0);
    // Hydraulics off: slow, heavy servos (the cyclic loads feed back).
    if ac.rotor.fail.hydraulics {
        let sv = &mut ac.rotor.heli.sas.servo;
        let tgt = [out.coll, out.lon, out.lat, out.ped];
        for i in 0..4 {
            let rate = if i == 3 { 2.0 } else { 0.45 };
            sv[i] += (tgt[i] - sv[i]).clamp(-rate * dt, rate * dt);
        }
        // Without boost the rotor feedback pushes the cyclic aft-right.
        out = Inp { coll: sv[0], lon: sv[1] + 0.06, lat: sv[2] + 0.04, ped: sv[3] };
    } else {
        ac.rotor.heli.sas.servo = [out.coll, out.lon, out.lat, out.ped];
    }
    out
}

/// One step of the helicopter: assist → rotor / airframe → drive train.
#[allow(clippy::too_many_arguments)]
pub fn step(h: &HeliDef, def: &AircraftDef, ac: &mut Aircraft, c: &Controls, vb: Vec3, agl: f32, atm: &Atmosphere, dt: f32) -> Out {
    if !ac.rotor.heli.init {
        let mut cc = *c;
        settle(h, def, ac, &mut cc, !ac.on_ground);
    }
    let fail = ac.rotor.fail;
    let failed = c.failed_engines & 1 != 0;
    let power_k = ac.sys.fx.power[0].clamp(0.0, 1.0);
    let thrust_prev = ac.rotor.out.thrust;
    let inp = assist(h, def, ac, c, thrust_prev, dt);
    let up_body = ac.rot.conjugate() * Vec3::Y;
    let w = ac.omega;
    let st = &mut ac.rotor.heli;
    st.time += dt;
    if fail.tail_pitch && st.tail_frozen.is_none() {
        st.tail_frozen = Some(-h.rot_dir * h.tr_pitch_bias + inp.ped * h.tr_pitch_range);
    }
    if !fail.tail_pitch {
        st.tail_frozen = None;
    }
    // Hub height above the ground (along the vertical).
    let hub_agl = agl + h.hub.dot(up_body);
    let ev = eval(h, st, inp, vb, w, atm.density, atm.speed_of_sound, hub_agl, fail);

    // ---- fast states --------------------------------------------------------
    let tau_i = (0.6 * h.radius / (st.vi.abs() + 0.3 * h.radius * st.nr.max(1.0) * 0.05 + 2.0)).clamp(0.05, 0.4);
    st.vi += (ev.vi_target - st.vi) * (dt / tau_i).min(1.0);
    st.tr_vi += (ev.tr_vi_target - st.tr_vi) * (dt / 0.05).min(1.0);
    let kf = (dt / ev.tau_flap.max(dt)).min(1.0);
    st.a1 += (ev.a1_ss - st.a1) * kf;
    st.b1 += (ev.b1_ss - st.b1) * kf;

    // ---- engine, governor, drive train ----------------------------------------
    let sigma = atm.density / 1.225;
    let oat = atm.temperature_c();
    let isa_t = crate::flight::isa(ac.pos.y).temperature_c();
    let nr_frac = st.nr / h.omega;
    let p_ref = h.power_available(sigma, oat, oat, 1.0, h.p_takeoff).max(1.0);
    let p_av = h.power_available(sigma, oat, isa_t, nr_frac.max(0.0), h.p_takeoff) * power_k;
    let q_load = ev.q_main + (ev.p_tail + h.p_access) / st.nr.max(5.0);
    let p_idle = 0.06 * h.p_mgb * power_k;
    if failed {
        st.power += (0.0 - st.power) * (dt / 1.2).min(1.0);
        st.gov_i = 0.0;
        st.ng += (0.0 - st.ng) * (dt / 6.0).min(1.0);
        st.tot += (oat - st.tot) * (dt / 25.0).min(1.0);
        st.n2 = (st.n2 - st.n2 * dt / 8.0).min(nr_frac);
    } else {
        // FADEC: hold 100 % Nr (load anticipation + PI on the speed error).
        let err = h.omega - st.nr;
        st.gov_i = (st.gov_i + err * dt * 0.8).clamp(-4.0, 4.0);
        let corr = h.inertia * h.omega * (2.0 * err + st.gov_i);
        let demand = (q_load * h.omega / h.eta + corr).clamp(p_idle, p_av.max(p_idle));
        let tau = if demand > st.power { h.tau_up } else { h.tau_down };
        // Acceleration is slow from low power (gas generator schedule).
        let x = (st.power / p_ref.max(1.0)).clamp(0.0, 1.0);
        let tau = tau * (1.6 - 0.6 * x);
        st.power += (demand - st.power) * (dt / tau).min(1.0);
        st.power = st.power.min(p_av.max(p_idle));
        let (ng0, tot0) = (st.ng, st.tot);
        engine_gauges(h, st, st.power, p_ref, oat);
        let (ng_t, tot_t) = (st.ng, st.tot);
        st.ng = ng0 + (ng_t - ng0) * (dt / 0.35).min(1.0);
        st.tot = tot0 + (tot_t - tot0) * (dt / 1.5).min(1.0);
        // Free turbine: governed at 100 %, freewheeling when the rotor runs faster.
        st.n2 = nr_frac.min(1.005);
    }
    let q_max = 1.25 * h.p_mgb / h.omega;
    let q_drive = if failed { 0.0 } else { (st.power * h.eta / st.nr.max(3.0)).min(q_max) };
    let q_fric = 60.0 + 0.002 * h.inertia * st.nr;
    let dnr = (q_drive - q_load - q_fric) / h.inertia;
    st.nr = (st.nr + dnr * dt).max(0.0);
    st.azimuth = (st.azimuth + st.nr * dt).rem_euclid(2.0 * PI);
    st.tail_azimuth = (st.tail_azimuth + st.nr * h.tr_gear * dt * if fail.tail_drive { 0.0 } else { 1.0 }).rem_euclid(2.0 * PI);

    // ---- outputs ------------------------------------------------------------
    let (n_s, _) = h.shaft();
    // Torque reaction of the transmission on the fuselage.
    let moment = ev.moment - n_s * (h.rot_dir * q_drive);
    let st = ac.rotor.heli;
    let nr_frac = st.nr / h.omega;
    let torque_pct = st.power / st.nr.max(3.0) / (h.p_mgb / h.omega) * 100.0;
    let fli = 10.0 * [torque_pct / 100.0, (st.tot - h.tot_idle) / (h.tot_max - h.tot_idle), (st.ng - h.ng_idle) / (101.0 - h.ng_idle)].into_iter().fold(0.0f32, f32::max);
    let tail_rpm = if fail.tail_drive { 0.0 } else { st.nr * h.tr_gear * 60.0 / (2.0 * PI) };
    let airborne_or_power = !ac.on_ground || c.throttle > 0.1;
    let o = &mut ac.rotor.out;
    o.nr_pct = nr_frac * 100.0;
    o.nr_rpm = st.nr * 60.0 / (2.0 * PI);
    o.tail_rpm = tail_rpm;
    o.ng_pct = st.ng;
    o.n2_pct = st.n2 * 100.0;
    o.torque_pct = torque_pct;
    o.tot_c = st.tot;
    o.fli = fli;
    o.thrust = ev.thrust;
    o.v_induced = st.vi;
    o.hub_agl = hub_agl;
    o.mu = ev.mu;
    o.vrs = ev.vrs;
    o.stall = ev.stall;
    o.collective_deg = ev.theta75.to_degrees();
    o.coning_deg = ev.coning.to_degrees();
    o.tilt_deg = [st.a1.to_degrees(), st.b1.to_degrees()];
    o.tail_pitch_deg = ev.tail_pitch.to_degrees();
    o.power_w = st.power;
    o.power_avail_w = p_av;
    o.azimuth = st.azimuth;
    o.low_nr = nr_frac < h.nr_low && (airborne_or_power || failed) && st.nr > 0.2 * h.omega;
    o.high_nr = nr_frac > h.nr_high;
    o.eff = [inp.coll, inp.lon, inp.lat, inp.ped];
    // Engine gauges in the shared engine slots (Np = rotor rpm, spool = Ng).
    let ff = if failed { 0.0 } else { h.sfc * st.power / 1000.0 + 18.0 };
    let nr_rpm = o.nr_rpm;
    ac.set_engine_gauges(0, nr_rpm, st.ng / 100.0, st.tot, ff, st.power, !failed, 0.0, st.azimuth);

    // The blades flap freely: the rotor's gyroscopic response is in the
    // flapping (rate lag / cross-coupling above), not a rigid disc. Only the
    // hub and drive train spin rigidly with the shaft (~3 % of the rotor).
    let h_vec = n_s * (h.rot_dir * 0.03 * h.inertia * st.nr);
    Out { force: ev.force, moment, damping: Vec3::new(def.inertia_pitch, def.inertia_yaw, def.inertia_roll) * 0.02, h: h_vec, lift: ev.thrust }
}

/// The disc's world orientation helper for effects: shaft axis in the body frame.
pub fn shaft_axis(h: &HeliDef) -> Vec3 {
    h.shaft().0
}

/// Rotate a body vector by the aircraft attitude (helper for effects).
pub fn to_world(rot: Quat, v: Vec3) -> Vec3 {
    rot * v
}
