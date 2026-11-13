//! Water operations (Track H2b): float / hull hydrodynamics, sea state and
//! water "lanes" for floatplanes.
//!
//! The model is a strip ("2D + t") method along each hull, the same family
//! as Zarnick's planing-hull model:
//!
//! * **Hydrostatics** — each strip displaces the V-bottom / box section below
//!   the local water surface (mean level + waves): buoyancy, and the righting
//!   moment of two floats a few metres apart.
//! * **Hydrodynamics** — the water's added mass of a strip
//!   (`m_a = ρ π/2 c²`, Wagner splash-up `c = π/2 · d / tan β`) changes as a
//!   slice of water is passed aft along the hull; the momentum it receives
//!   (`f = −V_n · D m_a / Dt`) is the **planing lift** of the forebody, the
//!   slamming load of a hard arrival and the induced drag of a hull at trim
//!   (it acts normal to the keel). Cross-flow drag damps heave.
//! * **The step** — the keel steps up behind it, so the flow separates there
//!   and the afterbody only touches when the hull is over-trimmed (dragging
//!   the heels) or still in displacement mode.
//! * **Resistance** — skin friction (ITTC-57) on the wetted surface and the
//!   wave-making "hump" of displacement running (peaking at a volumetric
//!   Froude number of ~1.9: the floatplane has to be helped *on the step*).
//! * **Sideways** — the keel resists slipping (cross-flow drag and keel
//!   lift), so wind on the fin weathervanes the aircraft at low speed; water
//!   rudders at the float sterns steer below ~20 kt (they retract above).
//! * **Sea state** — glassy below ~3 kt of wind, chop and waves growing with
//!   wind and fetch (bounded on rivers / lakes); the waves move the surface
//!   under the strips (pitching, slamming, spray).
//!
//! # API (also for airliner ditching, src/missions)
//!
//! ```ignore
//! let hulls = [water::Hull::fuselage(nose_z, tail_z, belly_y, radius)];
//! let c = water::water_contact(&hulls, &body, env, &sea, time, rudder);
//! // c.force (world, N) and c.moment (body, N·m) act on the aircraft;
//! // c.wet, c.slam_g, c.bow_buried, c.spray tell what happened.
//! ```
//! [`WaterState`] (on every `Aircraft` as `ac.water`) keeps the sea state and
//! the last contact for the effects / audio / HUD.

use crate::flight::Environment;
use glam::{Quat, Vec2, Vec3};
use std::f32::consts::PI;

/// Water density, kg/m³ (between fresh and sea water).
pub const RHO_WATER: f32 = 1_010.0;
/// Kinematic viscosity of water, m²/s.
const NU_WATER: f32 = 1.1e-6;
const G: f32 = 9.81;
/// Strips per hull.
const STRIPS: usize = 18;
/// Spray sources per contact (two per hull, up to two hulls).
pub const MAX_SPRAY: usize = 4;
/// Water rudders are down (and steer) below this water speed, m/s (~20 kt).
pub const WATER_RUDDER_UP_MS: f32 = 10.0;

/// A float, flying-boat hull or (for ditching) a fuselage: a slender body
/// along the body Z axis. Body frame as `flight.rs` (+Z nose, +Y up, +X left).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hull {
    /// Lateral position of the keel line (body x).
    pub x: f32,
    /// Bow and stern (body z).
    pub bow_z: f32,
    pub stern_z: f32,
    /// The step (body z): the forebody ahead of it planes. A fuselage has
    /// no step (`step_z = stern_z`).
    pub step_z: f32,
    /// Keel height at the step (body y).
    pub keel_y: f32,
    /// Keel rise from the step to the bow and to the stern (m).
    pub bow_rise: f32,
    pub stern_rise: f32,
    /// Step height: the afterbody keel starts this much above the forebody's.
    pub step_h: f32,
    /// Maximum beam (m) and keel-to-deck depth (m).
    pub beam: f32,
    pub depth: f32,
    /// Bottom deadrise angle (rad): V-bottom floats ~20°; a round fuselage ~40°.
    pub deadrise: f32,
    /// Water rudder area at the stern (m², 0 = none).
    pub rudder_area: f32,
}

impl Hull {
    /// A float (one of a pair): lateral position, bow / step / stern
    /// stations, keel height at the step, beam and depth.
    #[allow(clippy::too_many_arguments)]
    pub const fn float(x: f32, bow_z: f32, step_z: f32, stern_z: f32, keel_y: f32, bow_rise: f32, stern_rise: f32, beam: f32, depth: f32, rudder_area: f32) -> Hull {
        Hull { x, bow_z, stern_z, step_z, keel_y, bow_rise, stern_rise, step_h: 0.06, beam, depth, deadrise: 0.35, rudder_area }
    }

    /// A fuselage as a ditching hull (airliners, src/missions): nose / tail
    /// stations, belly height at the widest point and the fuselage radius.
    pub fn fuselage(nose_z: f32, tail_z: f32, belly_y: f32, radius: f32) -> Hull {
        Hull {
            x: 0.0,
            bow_z: nose_z,
            stern_z: tail_z,
            step_z: tail_z,
            keel_y: belly_y,
            bow_rise: radius * 0.9,
            stern_rise: radius * 1.2,
            step_h: 0.0,
            beam: 2.0 * radius,
            depth: 2.0 * radius,
            deadrise: 0.7,
            rudder_area: 0.0,
        }
    }

    pub fn length(&self) -> f32 {
        self.bow_z - self.stern_z
    }

    /// Keel height (body y) at station `z`.
    pub fn keel_at(&self, z: f32) -> f32 {
        if z >= self.step_z {
            let u = ((z - self.step_z) / (self.bow_z - self.step_z).max(0.1)).clamp(0.0, 1.0);
            // Flat run aft, then a rounded bow.
            self.keel_y + self.bow_rise * u.powf(2.2)
        } else {
            let u = ((self.step_z - z) / (self.step_z - self.stern_z).max(0.1)).clamp(0.0, 1.0);
            self.keel_y + self.step_h + self.stern_rise * u
        }
    }

    /// Half-beam (m) at station `z`: full amidships, tapering to the ends.
    pub fn half_beam_at(&self, z: f32) -> f32 {
        let u = ((z - self.stern_z) / self.length().max(0.1)).clamp(0.0, 1.0);
        // Fuller forward of the middle (bow) than the tapering afterbody.
        let s = if u > 0.55 { ((1.0 - u) / 0.45).clamp(0.0, 1.0).powf(0.45) } else { (u / 0.55).clamp(0.0, 1.0).powf(0.3) };
        0.5 * self.beam * s.max(0.12)
    }

    /// Deck height (body y) — the keel at the step plus the depth.
    fn deck_y(&self) -> f32 {
        self.keel_y + self.depth
    }

    /// Immersed section area (m²) at normal immersion `d` (m) of a V bottom
    /// with half-beam `b`, up to the deck.
    fn section_area(&self, d: f32, b: f32, depth: f32) -> f32 {
        let d = d.clamp(0.0, depth);
        let tb = self.deadrise.tan().max(0.05);
        let hc = b * tb;
        if d <= hc { d * d / tb } else { b * hc + 2.0 * b * (d - hc) }
    }

    /// Displaced volume when immersed to the deck (m³): the reserve buoyancy.
    pub fn volume(&self) -> f32 {
        let dz = self.length() / STRIPS as f32;
        (0..STRIPS)
            .map(|i| {
                let z = self.stern_z + (i as f32 + 0.5) * dz;
                let keel = self.keel_at(z);
                self.section_area(self.deck_y() - keel, self.half_beam_at(z), self.deck_y() - keel) * dz
            })
            .sum()
    }
}

/// Wind-driven surface state of a water body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SeaState {
    /// Significant wave height, m (0 = glassy).
    pub wave_h: f32,
    /// Direction the waves travel (unit, world xz).
    pub dir: Vec2,
    /// Dominant wavelength, m.
    pub wavelength: f32,
}

impl SeaState {
    /// Fetch-limited wind sea (SMB-style, simplified): glassy below ~1.5 m/s
    /// of wind; `fetch_m` (open water upwind) bounds rivers and lakes.
    pub fn from_wind(wind: Vec3, fetch_m: f32) -> SeaState {
        let w = Vec2::new(wind.x, wind.z);
        let u = w.length();
        if u < 1.5 {
            return SeaState { wave_h: 0.0, dir: Vec2::X, wavelength: 1.0 };
        }
        // Fully developed: Hs ≈ 0.0246 U² (Pierson–Moskowitz); fetch-limited
        // (JONSWAP): g Hs / U² = 0.0016 (g F / U²)^½, g Tp / U = 0.286 (g F / U²)^⅓.
        let fetch = fetch_m.clamp(50.0, 200_000.0);
        let x = G * fetch / (u * u);
        let hs = (0.0246 * u * u).min(0.0016 * x.sqrt() * u * u / G).min(4.0);
        // Peak period → deep-water wavelength.
        let tp = (0.286 * x.cbrt() * u / G).clamp(0.6, 9.0);
        SeaState { wave_h: hs, dir: w / u, wavelength: G * tp * tp / (2.0 * PI) }
    }

    pub fn glassy(&self) -> bool {
        self.wave_h < 0.02
    }

    /// Surface elevation above the mean level at `xz`, time `t` (s): three
    /// crossing wave trains around the wind direction.
    pub fn height(&self, xz: Vec2, t: f32) -> f32 {
        if self.glassy() {
            return 0.0;
        }
        let mut h = 0.0;
        for (ang, amp, lf, ph) in [(0.0f32, 0.30f32, 1.0f32, 0.0f32), (0.45, 0.13, 0.62, 1.7), (-0.6, 0.08, 0.41, 4.1)] {
            let (s, c) = ang.sin_cos();
            let d = Vec2::new(self.dir.x * c - self.dir.y * s, self.dir.x * s + self.dir.y * c);
            let lambda = (self.wavelength * lf).max(0.5);
            let kw = 2.0 * PI / lambda;
            let om = (G * kw).sqrt();
            h += amp * self.wave_h * (kw * d.dot(xz) - om * t + ph).sin();
        }
        h
    }

    /// Rate of change of the surface elevation (m/s) at `xz`.
    pub fn rate(&self, xz: Vec2, t: f32) -> f32 {
        if self.glassy() {
            return 0.0;
        }
        (self.height(xz, t + 0.02) - self.height(xz, t - 0.02)) / 0.04
    }
}

/// Kinematic state of the body the hulls belong to.
#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    /// Body-frame angular velocity.
    pub omega: Vec3,
}

/// A spray source this step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Spray {
    /// World position (at the surface).
    pub pos: Vec3,
    /// Initial sheet velocity (world, m/s).
    pub vel: Vec3,
    /// 0..1 intensity (0 = off).
    pub rate: f32,
    /// True for the planing whisker / rooster tail behind the step, false for
    /// the displacement bow wave.
    pub planing: bool,
}

/// Forces and the state of the water contact.
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact {
    /// Total force (world, N) and moment about the CG (body frame, N·m).
    pub force: Vec3,
    pub moment: Vec3,
    /// Any strip in the water.
    pub wet: bool,
    /// Displaced volume (m³) and the hydrostatic / hydrodynamic support (N).
    pub displaced: f32,
    pub buoyancy: f32,
    pub dynamic: f32,
    /// Water resistance along the motion (N).
    pub drag: f32,
    /// Share of the support from dynamic lift (0 = floating .. 1 = planing).
    pub planing: f32,
    /// Water speed (m/s) along the hull.
    pub speed: f32,
    /// Peak dynamic normal load this step as a multiple of `weight_ref`
    /// (slamming: a hard / nose-low arrival).
    pub slam_g: f32,
    /// The bow was driven under (deck awash forward) at speed: a nose-over.
    pub bow_buried: bool,
    /// Deepest immersion as a fraction of the hull depth (1 = deck awash).
    pub max_immersion: f32,
    /// Water rudders down and steering.
    pub rudders_down: bool,
    pub spray: [Spray; MAX_SPRAY],
}

/// Water forces on `hulls`. `t` is the sea-state clock, `rudder` the pedal
/// position (−1..1, + = yaw right) for the water rudders, `weight_ref` the
/// aircraft weight (N) used to normalise [`Contact::slam_g`]. Strips over
/// land (`env.is_water` false) carry nothing — the caller's gear handles land.
pub fn water_contact(hulls: &[Hull], b: &BodyState, env: &dyn Environment, sea: &SeaState, t: f32, rudder: f32, weight_ref: f32) -> Contact {
    let mut c = Contact::default();
    if hulls.is_empty() {
        return c;
    }
    let up = b.rot * Vec3::Y;
    let fwd = b.rot * Vec3::Z;
    let left = b.rot * Vec3::X;
    let inv = b.rot.conjugate();
    let up_y = up.y;
    let mut force = Vec3::ZERO;
    let mut moment = Vec3::ZERO;
    let mut n_spray = 0;
    let mut dyn_max = 0.0f32;
    // Water speed along the hull (body forward), for friction / planing.
    let u_fwd = b.vel.dot(fwd);
    c.speed = u_fwd;
    let horiz = Vec3::new(b.vel.x, 0.0, b.vel.z);
    let mut wave_drag_pts: Vec<(f32, Vec3)> = Vec::new();
    for h in hulls {
        let dz = h.length() / STRIPS as f32;
        let tb = h.deadrise.tan().max(0.05);
        let mut dn = [0.0f32; STRIPS];
        let mut ma = [0.0f32; STRIPS];
        let mut wet = [false; STRIPS];
        let mut pts = [Vec3::ZERO; STRIPS];
        let mut hw_rate = [0.0f32; STRIPS];
        let mut half = [0.0f32; STRIPS];
        let mut depth = [0.0f32; STRIPS];
        for i in 0..STRIPS {
            let z = h.stern_z + (i as f32 + 0.5) * dz;
            let keel = h.keel_at(z);
            let pb = Vec3::new(h.x, keel, z);
            let p = b.pos + b.rot * pb;
            pts[i] = pb;
            half[i] = h.half_beam_at(z);
            depth[i] = (h.deck_y() - keel).max(0.05);
            let xz = Vec2::new(p.x, p.z);
            if !env.is_water(xz) {
                continue;
            }
            let surf = env.ground_height(xz) + sea.height(xz, t);
            let d = surf - p.y;
            if d <= 0.0 || up_y <= 0.05 {
                continue;
            }
            // Immersion normal to the bottom (the section is tilted with the hull).
            dn[i] = (d * up_y).min(depth[i] * 1.6);
            wet[i] = true;
            hw_rate[i] = sea.rate(xz, t);
            // Wagner splash-up: the wetted half-width grows faster than the geometric one.
            let c_w = (0.5 * PI * dn[i] / tb).min(half[i]);
            ma[i] = RHO_WATER * 0.5 * PI * c_w * c_w;
        }
        let fore = |i: usize| h.stern_z + (i as f32 + 0.5) * dz >= h.step_z;
        let mut hull_disp = 0.0f32;
        let mut hull_disp_mom = Vec3::ZERO;
        let mut first_wet_fwd: Option<usize> = None;
        for i in 0..STRIPS {
            if !wet[i] {
                continue;
            }
            c.wet = true;
            if first_wet_fwd.is_none_or(|k| i > k) {
                first_wet_fwd = Some(i);
            }
            let rb = pts[i];
            let v_pt = b.vel + b.rot * b.omega.cross(rb);
            let d = dn[i];
            let bh = half[i];
            // ---- hydrostatics
            let area = h.section_area(d, bh, depth[i]);
            let vol = area * dz;
            hull_disp += vol;
            let f_b = Vec3::Y * (RHO_WATER * G * vol);
            let centroid = rb + Vec3::Y * (0.55 * d.min(depth[i]));
            hull_disp_mom += centroid * vol;
            let mut f = f_b;
            c.buoyancy += f_b.y;
            // ---- hydrodynamics (momentum of the added mass)
            let v_n = v_pt.dot(up) - hw_rate[i] * up_y;
            // ∂m_a/∂z within the forebody / afterbody (the step separates the flow).
            let same = |j: usize| j < STRIPS && fore(j) == fore(i);
            let (m_lo, z_lo) = if i > 0 && same(i - 1) { (ma[i - 1], -1.0) } else { (ma[i], 0.0) };
            let (m_hi, z_hi) = if same(i + 1) { (ma[i + 1], 1.0) } else { (ma[i], 0.0) };
            let dmdz = if z_hi - z_lo > 0.0 { (m_hi - m_lo) / ((z_hi - z_lo) * dz) } else { 0.0 };
            // Local immersion rate (heave / pitch / waves) and the slice passing aft.
            let ddn = (-(v_pt.y) + hw_rate[i]) * up_y;
            let c_w = (0.5 * PI * d / tb).min(bh);
            let dma_dd = if c_w < bh { RHO_WATER * 0.5 * PI * 2.0 * c_w * (0.5 * PI / tb) } else { 0.0 };
            let u_b = v_pt.dot(fwd);
            let dm_dt = dma_dd * ddn - u_b * dmdz;
            let f_mom = (-v_n).max(0.0) * dm_dt.max(0.0);
            // Cross-flow drag and wave-radiation damping (heave and pitch
            // settle in a couple of cycles, as on real floats).
            let w_wl = (d / tb).min(bh);
            // (Radiation damping acts on the heave / pitch motion of the strip,
            // not on the steady cross-flow of a hull running at trim.)
            let w_heave = v_pt.y - hw_rate[i];
            let f_cf = -0.5 * RHO_WATER * 1.0 * (2.0 * w_wl) * v_n * v_n.abs() - 0.8 * RHO_WATER * (2.0 * w_wl) * (G * 2.0 * w_wl).sqrt() * w_heave * up_y;
            let f_n = ((f_mom + f_cf) * dz).clamp(-30.0 * weight_ref.max(1.0) / STRIPS as f32, 60.0 * weight_ref.max(1.0) / STRIPS as f32);
            f += up * f_n;
            c.dynamic += f_n * up_y;
            dyn_max += f_mom.max(0.0) * dz;
            // ---- skin friction on the wetted girth (bottom + sides)
            let hc = bh * tb;
            let girth = 2.0 * (d.min(hc) / h.deadrise.sin().max(0.1) + (d - hc).max(0.0));
            let l_wet = (STRIPS as f32 * dz).max(0.5);
            let re = (u_b.abs() * l_wet / NU_WATER).max(1e5);
            let cf = 0.075 / (re.log10() - 2.0).powi(2);
            let f_fric = -0.5 * RHO_WATER * cf * girth * dz * u_b * u_b.abs();
            f += fwd * f_fric;
            // ---- sideways: cross-flow drag and keel lift resist slipping
            let v_lat = v_pt.dot(left);
            let side_d = d.min(depth[i]);
            let f_lat = -(0.5 * RHO_WATER * 1.2 * v_lat.abs() + RHO_WATER * 0.7 * u_b.abs()) * v_lat * side_d * dz;
            f += left * f_lat;
            force += f;
            moment += rb.cross(inv * f);
        }
        // ---- water rudder at the stern (down below ~20 kt)
        if h.rudder_area > 0.0 && wet[0] && u_fwd.abs() < WATER_RUDDER_UP_MS {
            c.rudders_down = true;
            let delta = rudder.clamp(-1.0, 1.0) * 0.45;
            let cl = (2.0 * PI * 0.55 * delta).clamp(-1.1, 1.1);
            let q = 0.5 * RHO_WATER * u_fwd * u_fwd.abs();
            // Right pedal: the rudder trailing edge goes right, the stern is pushed left (+X).
            let f = left * (q * h.rudder_area * cl);
            let rb = Vec3::new(h.x, h.keel_at(h.stern_z) - 0.2, h.stern_z);
            force += f;
            moment += rb.cross(inv * f);
        }
        // ---- wave-making resistance (displacement / hump) at the centre of buoyancy
        if hull_disp > 1e-4 {
            wave_drag_pts.push((hull_disp, hull_disp_mom / hull_disp));
            c.displaced += hull_disp;
        }
        // ---- spray
        if let Some(k) = first_wet_fwd {
            let u = u_fwd.max(0.0);
            let strength = ((u - 1.5) / 8.0).clamp(0.0, 1.0) * (dn[k] / 0.25).clamp(0.2, 1.0);
            if strength > 0.02 {
                let planing = fore(k) && u > 9.0 && k < STRIPS - 3;
                for side in [-1.0f32, 1.0] {
                    if n_spray >= MAX_SPRAY {
                        break;
                    }
                    let rb = pts[k] + Vec3::new(side * half[k], dn[k] * 0.6, 0.0);
                    let p = b.pos + b.rot * rb;
                    let out = left * side;
                    let vel = out * (0.35 * u + 1.0) + Vec3::Y * (0.18 * u + 0.8) - fwd * (0.25 * u);
                    c.spray[n_spray] = Spray { pos: p, vel: b.vel * 0.15 + vel, rate: strength, planing };
                    n_spray += 1;
                }
            }
        }
    }
    // Wave-making resistance (the "hump"): volumetric Froude number.
    let h_speed = horiz.length();
    if c.displaced > 1e-3 && h_speed > 0.05 {
        let fr = h_speed / (G * c.displaced.cbrt()).sqrt();
        let r_w = RHO_WATER * G * c.displaced * 0.16 * (-((fr - 1.9) / 0.8).powi(2)).exp() * (fr / 0.3).min(1.0);
        let dir = -horiz / h_speed;
        for (vol, cb) in &wave_drag_pts {
            let f = dir * (r_w * vol / c.displaced);
            force += f;
            // (Applied at the waterline height of the centre of buoyancy.)
            moment += Vec3::new(cb.x, cb.y + 0.3, cb.z).cross(inv * f);
        }
    }
    // Water resistance along the motion (friction, induced, wave-making).
    c.drag = (-force.dot(horiz.normalize_or_zero())).max(0.0);
    c.force = force;
    c.moment = moment;
    let support = (c.buoyancy + c.dynamic.max(0.0)).max(1.0);
    c.planing = (c.dynamic.max(0.0) / support).clamp(0.0, 1.0);
    c.slam_g = dyn_max / weight_ref.max(1.0);
    // Deepest immersion and a buried bow.
    for h in hulls {
        let z = h.bow_z - 0.12 * h.length();
        let pb = Vec3::new(h.x, h.deck_y(), z);
        let p = b.pos + b.rot * pb;
        let xz = Vec2::new(p.x, p.z);
        if env.is_water(xz) {
            let surf = env.ground_height(xz) + sea.height(xz, t);
            if surf > p.y && u_fwd > 6.0 {
                c.bow_buried = true;
            }
        }
        let zc = h.step_z;
        let pk = b.pos + b.rot * Vec3::new(h.x, h.keel_at(zc), zc);
        let xz = Vec2::new(pk.x, pk.z);
        if env.is_water(xz) {
            let surf = env.ground_height(xz) + sea.height(xz, t);
            c.max_immersion = c.max_immersion.max(((surf - pk.y) / h.depth).max(0.0));
        }
    }
    c
}

/// Per-aircraft water state (sea-state clock, the sea, the last contact).
#[derive(Clone, Copy, Debug, Default)]
pub struct WaterState {
    pub time: f32,
    pub sea: SeaState,
    /// Seconds to the next fetch / sea-state estimate.
    refresh: f32,
    /// Last step's contact (forces, spray, planing, rudders).
    pub last: Contact,
}

impl WaterState {
    /// Floating or running on the water (not just touching it).
    pub fn afloat(&self) -> bool {
        self.last.wet
    }

    /// Advance the clock and re-estimate the sea state about once a second
    /// (wind at 10 m, open water upwind).
    pub fn update_sea(&mut self, env: &dyn Environment, pos: Vec3, dt: f32) {
        self.time += dt;
        self.refresh -= dt;
        if self.refresh > 0.0 {
            return;
        }
        self.refresh = 1.0;
        let xz = Vec2::new(pos.x, pos.z);
        if !env.is_water(xz) {
            return;
        }
        let gh = env.ground_height(xz);
        let wind = env.wind(Vec3::new(pos.x, gh + 10.0, pos.z));
        let w2 = Vec2::new(wind.x, wind.z);
        let fetch = if w2.length() > 0.5 {
            let up = -w2.normalize();
            let mut f = 0.0;
            while f < 20_000.0 && env.is_water(xz + up * (f + 100.0)) {
                f += 100.0;
            }
            f.max(50.0)
        } else {
            50.0
        };
        self.sea = SeaState::from_wind(wind, fetch);
    }
}

// ---------------------------------------------------------------------------
// Water lanes: where a floatplane can take off / land
// ---------------------------------------------------------------------------

/// The longest straight lane of open water (at least `width` m wide) within
/// `radius` of `near`: (start, end) in world xz, or None if there is no lane
/// of `min_len` m. Prefers long lanes, then ones close to `near`.
pub fn find_lane(env: &dyn Environment, near: Vec2, radius: f32, min_len: f32, width: f32) -> Option<(Vec2, Vec2)> {
    let cell = 40.0f32;
    let n = ((2.0 * radius / cell).ceil() as i32).clamp(8, 400);
    let origin = near - Vec2::splat(n as f32 * cell * 0.5);
    let idx = |x: i32, y: i32| (y * n + x) as usize;
    let mut water = vec![false; (n * n) as usize];
    let mut any = false;
    for y in 0..n {
        for x in 0..n {
            let p = origin + Vec2::new(x as f32 + 0.5, y as f32 + 0.5) * cell;
            let w = env.is_water(p);
            water[idx(x, y)] = w;
            any |= w;
        }
    }
    if !any {
        return None;
    }
    // Clear water: every cell within half the width is water (erosion).
    let r = ((0.5 * width) / cell).ceil() as i32;
    let mut clear = vec![false; (n * n) as usize];
    for y in r..n - r {
        for x in r..n - r {
            if !water[idx(x, y)] {
                continue;
            }
            let mut ok = true;
            'k: for dy in -r..=r {
                for dx in -r..=r {
                    if dx * dx + dy * dy <= r * r && !water[idx(x + dx, y + dy)] {
                        ok = false;
                        break 'k;
                    }
                }
            }
            clear[idx(x, y)] = ok;
        }
    }
    let at = |p: Vec2| -> bool {
        let q = (p - origin) / cell;
        let (x, y) = (q.x.floor() as i32, q.y.floor() as i32);
        x >= 0 && y >= 0 && x < n && y < n && clear[idx(x, y)]
    };
    let mut best: Option<(f32, Vec2, Vec2)> = None;
    let step = cell * 0.75;
    for y in (r..n - r).step_by(2) {
        for x in (r..n - r).step_by(2) {
            if !clear[idx(x, y)] {
                continue;
            }
            let p = origin + Vec2::new(x as f32 + 0.5, y as f32 + 0.5) * cell;
            for k in 0..24 {
                let a = k as f32 * PI / 24.0;
                let d = Vec2::new(a.cos(), a.sin());
                let mut f = 0.0;
                while at(p + d * (f + step)) && f < 8_000.0 {
                    f += step;
                }
                let mut bk = 0.0;
                while at(p - d * (bk + step)) && bk < 8_000.0 {
                    bk += step;
                }
                let len = f + bk;
                if len < min_len {
                    continue;
                }
                let (a0, a1) = (p - d * bk, p + d * f);
                let mid = (a0 + a1) * 0.5;
                // Long lanes win; nearby ones break near-ties.
                let score = len.min(3_000.0) - 0.15 * mid.distance(near);
                if best.is_none_or(|b| score > b.0) {
                    best = Some((score, a0, a1));
                }
            }
        }
    }
    best.map(|(_, a, b)| {
        // Keep clear of the banks at the ends.
        let d = (b - a).normalize_or_zero();
        let m = (0.06 * a.distance(b)).min(120.0);
        let (a, b) = (a + d * m, b - d * m);
        // At most ~3 km of it (the middle) is plenty.
        let len = a.distance(b);
        if len > 3_000.0 {
            let c = (a + b) * 0.5;
            (c - d * 1_500.0, c + d * 1_500.0)
        } else {
            (a, b)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Lake {
        lo: Vec2,
        hi: Vec2,
        wind: Vec3,
    }
    impl Environment for Lake {
        fn ground_height(&self, _xz: Vec2) -> f32 {
            0.0
        }
        fn obstacle_top(&self, _xz: Vec2) -> Option<f32> {
            None
        }
        fn is_water(&self, xz: Vec2) -> bool {
            xz.x >= self.lo.x && xz.x <= self.hi.x && xz.y >= self.lo.y && xz.y <= self.hi.y
        }
        fn updraft(&self, _p: Vec3) -> f32 {
            0.0
        }
        fn wind(&self, _p: Vec3) -> Vec3 {
            self.wind
        }
    }

    fn otter_floats() -> [Hull; 2] {
        [Hull::float(1.64, 3.58, -0.43, -3.88, -2.02, 0.32, 0.47, 0.83, 0.95, 0.07), Hull::float(-1.64, 3.58, -0.43, -3.88, -2.02, 0.32, 0.47, 0.83, 0.95, 0.07)]
    }

    #[test]
    fn reserve_buoyancy_and_static_draft() {
        let hulls = otter_floats();
        let vol: f32 = hulls.iter().map(|h| h.volume()).sum();
        // Floats must carry ≥ 180 % of the weight (FAR 23 / CS-23).
        assert!(vol * RHO_WATER > 1.8 * 3_500.0, "reserve {vol}");
        // Find the floating height of a 3,500 kg body by bisection on the buoyancy.
        let env = Lake { lo: Vec2::splat(-1e4), hi: Vec2::splat(1e4), wind: Vec3::ZERO };
        let sea = SeaState::default();
        let (mut lo, mut hi) = (0.5f32, 3.0f32);
        for _ in 0..40 {
            let y = 0.5 * (lo + hi);
            let b = BodyState { pos: Vec3::new(0.0, y, 0.0), vel: Vec3::ZERO, rot: Quat::IDENTITY, omega: Vec3::ZERO };
            let c = water_contact(&hulls, &b, &env, &sea, 0.0, 0.0, 3_500.0 * G);
            if c.force.y > 3_500.0 * G { lo = y } else { hi = y }
        }
        let draft = 2.0 - 0.5 * (lo + hi);
        eprintln!("static draft {draft:.2} m, CG {:.2} m above the water", 0.5 * (lo + hi));
        assert!(draft > 0.2 && draft < 0.6);
    }

    #[test]
    fn planing_lift_and_hump() {
        // At 25 m/s and 6° trim the forebody planes: a large share of the weight
        // carried dynamically, with drag well below the hump's.
        let hulls = otter_floats();
        let env = Lake { lo: Vec2::splat(-1e4), hi: Vec2::splat(1e4), wind: Vec3::ZERO };
        let sea = SeaState::default();
        let w = 3_500.0 * G;
        let state =
            |y: f32, trim: f32, v: f32| BodyState { pos: Vec3::new(0.0, y, 0.0), vel: Vec3::new(0.0, 0.0, v), rot: Quat::from_rotation_x(-trim.to_radians()), omega: Vec3::ZERO };
        let fast = water_contact(&hulls, &state(1.85, 6.0, 25.0), &env, &sea, 0.0, 0.0, w);
        eprintln!("planing: dyn {:.0} N buoy {:.0} N drag {:.0} share {:.2}", fast.dynamic, fast.buoyancy, -fast.force.z, fast.planing);
        assert!(fast.planing > 0.4, "planing share {}", fast.planing);
        let hump = water_contact(&hulls, &state(1.62, 8.0, 7.0), &env, &sea, 0.0, 0.0, w);
        eprintln!("hump: drag {:.0} N ({:.2} W)", -hump.force.z, -hump.force.z / w);
        assert!(-hump.force.z > 0.08 * w);
    }

    #[test]
    fn sea_state_from_wind() {
        assert!(SeaState::from_wind(Vec3::new(1.0, 0.0, 0.0), 5_000.0).glassy());
        let river = SeaState::from_wind(Vec3::new(10.0, 0.0, 0.0), 400.0);
        let sea = SeaState::from_wind(Vec3::new(10.0, 0.0, 0.0), 100_000.0);
        assert!(river.wave_h > 0.05 && river.wave_h < 0.5, "river {}", river.wave_h);
        assert!(sea.wave_h > 1.0, "open sea {}", sea.wave_h);
        let h = sea.height(Vec2::new(3.0, 4.0), 1.0);
        assert!(h.abs() < sea.wave_h);
    }

    #[test]
    fn lane_finder_picks_the_long_axis() {
        // A 2.5 km × 300 m river running east–west.
        let env = Lake { lo: Vec2::new(-1_250.0, -150.0), hi: Vec2::new(1_250.0, 150.0), wind: Vec3::ZERO };
        let (a, b) = find_lane(&env, Vec2::new(0.0, 500.0), 3_000.0, 800.0, 120.0).expect("lane");
        let d = b - a;
        eprintln!("lane {a:?} → {b:?} ({:.0} m)", d.length());
        assert!(d.length() > 1_800.0);
        assert!(d.y.abs() < 0.15 * d.length(), "lane along the river");
        assert!(a.y.abs() < 100.0 && b.y.abs() < 100.0);
        assert!(find_lane(&env, Vec2::new(9_000.0, 9_000.0), 1_000.0, 800.0, 120.0).is_none());
    }
}
