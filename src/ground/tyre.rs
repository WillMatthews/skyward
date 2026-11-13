//! Aircraft tyre: a semi-empirical slip model (normalised friction curves in
//! the style of Pacejka's magic formula, peak at ~12 % slip ratio / ~8° slip
//! angle, falling to the locked-wheel / sliding friction beyond), combined
//! slip by the friction ellipse, the wheel's spin degree of freedom
//! integrated implicitly (stable at the 120 Hz step for a 0.1 kg·m² light
//! wheel as for a 30 kg·m² airliner wheel), brake torque with ideal-ish
//! anti-skid, rolling resistance, and a velocity-level (implicit) regime at
//! walking pace so a braked aircraft stands still instead of creeping.

use super::surface::Surface;
use crate::flight::GRAVITY;
use glam::Vec3;

/// Slip ratio of peak longitudinal friction.
pub const KAPPA_PEAK: f32 = 0.12;
/// Slip angle of peak cornering force (rad, ~8°).
pub const ALPHA_PEAK: f32 = 0.14;
/// Below this contact speed (m/s) the longitudinal force is velocity-level (stiction).
const V_LO: f32 = 1.0;
const V_HI: f32 = 3.0;

/// Normalised friction curve: 0 at no slip, 1 at `x = 1` (the peak slip),
/// falling to `slide` (locked / fully sliding) beyond.
pub fn curve(x: f32, slide: f32) -> f32 {
    let x = x.abs();
    if x <= 1.0 { x * (2.0 - x) } else { slide + (1.0 - slide) * (-(x - 1.0) * 0.6).exp() }
}

/// d curve / dx.
fn curve_d(x: f32, slide: f32) -> f32 {
    let x = x.abs();
    if x <= 1.0 { 2.0 - 2.0 * x } else { -0.6 * (1.0 - slide) * (-(x - 1.0) * 0.6).exp() }
}

/// Longitudinal force coefficient (fraction of μ·Fz, signed) at slip ratio κ.
pub fn fx_coef(kappa: f32, slide: f32) -> f32 {
    kappa.signum() * curve(kappa / KAPPA_PEAK, slide)
}

/// Lateral force coefficient (fraction of μ·Fz, opposing) at slip angle α (rad).
/// Aircraft tyres corner more softly than they brake: linear (cornering
/// power ≈ μ / 8° per degree) to ~6°, rounding over to the peak at ~10°,
/// then a gentle fall towards the sliding value.
pub fn fy_coef(alpha: f32, slide: f32) -> f32 {
    let x = (alpha / ALPHA_PEAK).abs();
    let (a, b) = (0.7, 1.3);
    let f = if x <= a {
        x
    } else if x <= b {
        x - (x - a) * (x - a) / (2.0 * (b - a))
    } else {
        let s = slide.max(0.7);
        s + (1.0 - s) * (-(x - b) * 0.6).exp()
    };
    alpha.signum() * f
}

/// One wheel's inputs for a step.
#[derive(Clone, Copy, Debug)]
pub struct In {
    /// Normal load (N).
    pub fz: f32,
    /// Contact-point velocity (world).
    pub v: Vec3,
    /// Rolling direction and its left (ground plane, unit, world).
    pub fwd: Vec3,
    pub side: Vec3,
    pub surface: Surface,
    pub psi: f32,
    /// Rolling radius (m) and spin inertia of the leg's wheels (kg·m²).
    pub r: f32,
    pub inertia: f32,
    /// Spin rate now (rad/s, + rolling forward).
    pub omega: f32,
    /// Rolling resistance coefficient (paved).
    pub roll_mu: f32,
    /// Requested brake torque (N·m, ≥ 0).
    pub brake_torque: f32,
    pub antiskid: bool,
    /// Lateral grip scale (the type's tyre μ relative to 0.8).
    pub lat_scale: f32,
    /// Free castering: almost no side force (pivot friction only).
    pub castering: bool,
    /// Extra longitudinal drag (N, ≥ 0) from slush / standing water.
    pub fluid_drag: f32,
    pub dt: f32,
}

/// One wheel's result.
#[derive(Clone, Copy, Debug, Default)]
pub struct Out {
    pub force: Vec3,
    pub omega: f32,
    pub kappa: f32,
    pub alpha: f32,
    /// Peak μ here and how much the tyre is planing (0..1).
    pub mu: f32,
    pub planing: f32,
    pub locked: bool,
    /// Brake torque actually applied (after anti-skid), N·m.
    pub brake_applied: f32,
    /// Heat into the brakes (W) and scrubbed off at the contact patch (W).
    pub brake_power: f32,
    pub scrub_power: f32,
    /// Anti-skid releasing (0..1).
    pub release: f32,
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// One step of one wheel.
pub fn step(w: &In) -> Out {
    let fz = w.fz.max(0.0);
    let vx = w.v.dot(w.fwd);
    let vy = w.v.dot(w.side);
    let speed = (vx * vx + vy * vy).sqrt();
    let r = w.r.max(0.05);
    let inertia = w.inertia.max(1e-3);
    let dt = w.dt;
    let spinning = w.omega * r > 0.5 * vx.abs();
    let (mu, mu_slide, planing) = w.surface.friction(speed, w.psi, spinning);
    let slide = (mu_slide / mu.max(1e-3)).clamp(0.05, 1.0);
    let mu_fz = mu * fz;
    let share = fz / GRAVITY;
    let (roll_k, roll_add) = w.surface.rolling();
    let rr = w.roll_mu * roll_k + roll_add;

    // ---- spin degree of freedom ------------------------------------------
    let vref = vx.abs().max(V_HI);
    let kappa_of = |om: f32| (om * r - vx) / vref;
    let fx_of = |om: f32| mu_fz * fx_coef(kappa_of(om), slide);
    // Brake torque: anti-skid holds the slip near the peak (≈ 92 % efficient),
    // and keeps the brakes off a wheel that has not spun up yet (touchdown /
    // locked-wheel protection) above ~10 kt.
    let mut tb = w.brake_torque.max(0.0);
    let mut release = 0.0;
    if w.antiskid && vx.abs() > 5.0 {
        let cap = 0.92 * mu_fz * r;
        if tb > cap {
            release = 1.0 - cap / tb;
            tb = cap;
        }
        if w.omega * r < 0.6 * vx.abs() && tb > 0.0 {
            release = 1.0;
            tb = 0.0;
        }
    }
    // Planing tyres are spun down by the water wedge.
    let t_plane = planing * 0.3 * fz * r;
    let om = w.omega;
    let dir = if om.abs() > 1e-3 { om.signum() } else { vx.signum() };
    // Implicit: I (ω' − ω) = dt (−r Fx(ω') − Tb·dir − Tplane·dir), Fx linearised.
    let k = kappa_of(om);
    let g = (mu_fz * curve_d(k / KAPPA_PEAK, slide) / KAPPA_PEAK * r / vref).max(0.0);
    let fx0 = fx_of(om);
    let solve = |tb: f32| (inertia * om + dt * (-r * (fx0 - g * om) - (tb + t_plane) * dir)) / (inertia + dt * r * g);
    let mut om_new = solve(tb);
    let mut locked = false;
    let hold = tb + t_plane;
    if hold > 0.0 && (om_new * dir <= 0.0 || om.abs() < 1e-3) {
        // Stopped (or stopping this step): does the brake hold it against the
        // tyre's torque on a stopped wheel?
        let tau = (r * fx_of(0.0)).abs();
        if hold >= tau {
            om_new = 0.0;
            locked = speed > 0.5;
        } else {
            // It keeps turning, at the slip where the tyre balances the brake
            // (rising branch of the curve).
            let x = 1.0 - (1.0 - (hold / (r * mu_fz).max(1e-3)).min(1.0)).sqrt();
            om_new = vx / r * (1.0 - x * KAPPA_PEAK * vref / vx.abs().max(1e-3)).max(0.0);
        }
    }
    if !om_new.is_finite() {
        om_new = vx / r;
    }

    // ---- forces ------------------------------------------------------------
    let kappa = kappa_of(om_new);
    let k_long = 0.6 * share / dt;
    let k_lat = 0.35 * share / dt;
    // Slip model (above walking pace).
    let mut fx_slip = mu_fz * fx_coef(kappa, slide);
    let alpha = vy.abs().atan2(vx.abs() + 1.0) * vy.signum();
    let lat_mu = mu_fz * w.lat_scale;
    let mut fy_cap = if w.castering { 0.04 * fz } else { lat_mu * fy_coef(alpha.abs(), slide).max(0.05) };
    if locked && speed > 0.5 {
        // A locked tyre slides along its velocity: no directional control.
        let fs = mu_slide * fz;
        fx_slip = -fs * vx / speed;
        fy_cap = fy_cap.min((fs * vy.abs() / speed).max(0.02 * fz));
    } else if mu_fz > 1.0 {
        // Friction ellipse: braking / spin-up use up the side grip.
        let nx = (fx_slip / mu_fz).clamp(-1.0, 1.0);
        fy_cap *= (1.0 - nx * nx).max(0.0).sqrt().max(0.1);
    }
    // Rolling resistance and fluid drag.
    let resist = rr * fz + w.fluid_drag;
    // Velocity-level regime at walking pace: brakes (as a force), rolling
    // resistance and stiction in one implicit friction force.
    let brake_force = (w.brake_torque.max(0.0) / r).min(mu_fz);
    let fx_lo = -(vx * k_long).clamp(-(brake_force + resist), brake_force + resist);
    let fx_hi = fx_slip - (vx * k_long).clamp(-resist, resist);
    let s = smooth(V_LO, V_HI, vx.abs());
    let fx = fx_lo + (fx_hi - fx_lo) * s;
    let fy = -(vy * k_lat).clamp(-fy_cap, fy_cap);
    if s < 1.0 {
        // At walking pace the wheel just rolls (or is held by its brake).
        let held = brake_force >= (vx * k_long).abs() * 0.999 && w.brake_torque > 0.0;
        let roll_om = if held && vx.abs() < V_LO { 0.0 } else { vx / r };
        om_new = roll_om + (om_new - roll_om) * s;
    }
    let brake_applied = if s > 0.0 { tb } else { w.brake_torque.max(0.0) };
    // Heat: the brake torque times the wheel's spin at speed; at walking
    // pace only the braking force actually used (the rest is stiction).
    let brake_power = s * tb * om_new.abs() + (1.0 - s) * brake_force.min(fx_lo.abs()) * vx.abs();
    // Power dissipated in the contact patch (spin-up scrub, skids, side slip).
    let scrub_power = (fx_slip.abs() * (om_new * r - vx).abs() * s + fy.abs() * vy.abs() * smooth(1.0, 4.0, speed)).max(0.0);
    Out { force: w.fwd * fx + w.side * fy + Vec3::Y * fz, omega: om_new, kappa, alpha, mu, planing, locked, brake_applied, brake_power: brake_power.max(0.0), scrub_power, release }
}
