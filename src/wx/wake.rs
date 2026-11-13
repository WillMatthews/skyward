//! Wake turbulence: the counter-rotating vortex pair every lifting wing
//! leaves behind.
//!
//! Each generator (another aircraft — multiplayer remote aircraft, AI / ATC
//! traffic — or the player's own aircraft, met again after a steep 360° turn)
//! lays a trail of points; each point carries a vortex pair of initial
//! circulation Γ₀ = W / (ρ V b₀), spacing b₀ = π/4 · span, Lamb–Oseen cores
//! (r_c ≈ 0.05 span), that sinks at Γ/(2π b₀), drifts with the wind, levels
//! off and spreads in ground effect, and decays (slowly at first, then fast:
//! the two-phase decay of the wake-vortex models used for separation
//! standards). The induced velocity at a point is summed over the nearest
//! trail segment of each generator.
//!
//! Hook: `World::wakes` ([`WakeField::push`] once per frame per generator,
//! [`WakeField::velocity`] from `World::wind`). Positions are field
//! coordinates (fixed to the ground, as the floating origin moves).

use glam::{Vec2, Vec3};
use std::collections::VecDeque;

const RHO: f32 = 1.2;
/// Seconds between trail points and how long a trail lives.
const DT_POINT: f32 = 0.5;
const LIFE: f32 = 150.0;

/// One trail point: where the vortex pair was laid, along which direction,
/// and its strength.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WakePoint {
    pub pos: Vec3,
    pub dir: Vec3,
    /// Initial circulation (m²/s) and vortex spacing (m).
    pub gamma0: f32,
    pub b0: f32,
    pub t: f32,
    /// Ground height under the point (m).
    pub ground: f32,
}

#[derive(Clone, Debug, Default)]
struct Trail {
    id: u64,
    pts: VecDeque<WakePoint>,
    span: f32,
}

#[derive(Clone, Debug, Default)]
pub struct WakeField {
    trails: Vec<Trail>,
    /// Mean wind the vortices drift with (m/s, world).
    pub wind: Vec3,
}

/// Generator ids: the player's own aircraft, multiplayer players (+ their
/// id) and live ADS-B traffic (+ the ICAO address).
pub const OWN_AIRCRAFT: u64 = 1;
pub const MP_BASE: u64 = 0x1_0000;
pub const ADSB_BASE: u64 = 0x1_0000_0000;

/// A typical mass (kg) for an ICAO wake category letter (L / M / H / J).
pub fn mass_for_category(c: char) -> f32 {
    match c {
        'L' => 5_000.0,
        'H' => 230_000.0,
        'J' => 500_000.0,
        _ => 65_000.0,
    }
}

impl WakeField {
    /// Record a generator's position (field coordinates), velocity (m/s),
    /// mass (kg) and wingspan (m) at time `t`. Generators on the ground or
    /// slower than 25 m/s leave no wake.
    #[allow(clippy::too_many_arguments)]
    pub fn push(&mut self, id: u64, pos: Vec3, vel: Vec3, mass: f32, span: f32, airborne: bool, ground: f32, t: f32) {
        let tr = match self.trails.iter_mut().position(|x| x.id == id) {
            Some(i) => &mut self.trails[i],
            None => {
                self.trails.push(Trail { id, pts: VecDeque::new(), span });
                self.trails.last_mut().unwrap()
            }
        };
        tr.span = span;
        let tr_id = tr.id;
        // (Generators that left — landed, out of range — leave empty trails: drop them.)
        self.trails.retain(|t| !t.pts.is_empty() || t.id == tr_id);
        let tr = self.trails.iter_mut().find(|t| t.id == tr_id).unwrap();
        while tr.pts.front().is_some_and(|p| t - p.t > LIFE || p.t > t) {
            tr.pts.pop_front();
        }
        let v = vel.length();
        if !airborne || v < 25.0 || !pos.is_finite() || span <= 0.0 {
            return;
        }
        if tr.pts.back().is_some_and(|p| t - p.t < DT_POINT) {
            return;
        }
        let b0 = std::f32::consts::FRAC_PI_4 * span;
        let gamma0 = mass * 9.81 / (RHO * v * b0);
        tr.pts.push_back(WakePoint { pos, dir: vel / v, gamma0, b0, t, ground });
    }

    pub fn clear(&mut self) {
        self.trails.clear();
    }

    /// Shift every point (floating-origin rebase of field coordinates is not
    /// needed — they are fixed — but a teleport / reset drops the trails).
    pub fn forget(&mut self, id: u64) {
        self.trails.retain(|t| t.id != id);
    }

    /// Where a trail point's vortex pair is at time `t`: centre and current circulation.
    pub fn state(p: &WakePoint, t: f32, wind: Vec3) -> (Vec3, f32, f32) {
        let age = (t - p.t).max(0.0);
        // Two-phase decay: ~nominal for t < t1, then fast.
        let t_ref = 2.0 * std::f32::consts::PI * p.b0 * p.b0 / p.gamma0.max(1.0);
        let t1 = 1.6 * t_ref;
        let g = if age < t1 { 1.0 - 0.12 * age / t1 } else { 0.88 * (-(age - t1) / (0.9 * t_ref)).exp() };
        let gamma = p.gamma0 * g.max(0.0);
        // Sink at Γ/(2π b0), slowing as it decays, stop ~0.6 b0 above the ground.
        let w = p.gamma0 / (2.0 * std::f32::consts::PI * p.b0);
        let sink = w * age.min(t1) + w * 0.88 * (0.9 * t_ref) * (1.0 - (-(age - t1).max(0.0) / (0.9 * t_ref)).exp());
        let mut c = p.pos + wind * age - Vec3::Y * sink;
        let floor = p.ground + 0.6 * p.b0;
        let mut spread = 1.0;
        if c.y < floor {
            // Ground effect: the pair levels off and the vortices move apart.
            spread += ((floor - c.y) / p.b0).min(2.0);
            c.y = floor;
        }
        (c, gamma, spread)
    }

    /// Velocity induced at field point `p` (m/s) at time `t` by all wakes
    /// older than `min_age` s (the aircraft never meets its own fresh wake).
    pub fn velocity(&self, p: Vec3, t: f32, min_age: f32) -> Vec3 {
        let mut v = Vec3::ZERO;
        for tr in &self.trails {
            // Nearest qualifying point pair.
            let mut best: Option<(f32, usize)> = None;
            for (i, wp) in tr.pts.iter().enumerate() {
                if t - wp.t < min_age {
                    continue;
                }
                let (c, _, _) = Self::state(wp, t, self.wind);
                let d2 = c.distance_squared(p);
                if d2 < 250.0 * 250.0 && best.is_none_or(|(b, _)| d2 < b) {
                    best = Some((d2, i));
                }
            }
            let Some((_, i)) = best else { continue };
            let wp = tr.pts[i];
            let (c, gamma, spread) = Self::state(&wp, t, self.wind);
            if gamma <= 1.0 {
                continue;
            }
            let d = wp.dir;
            // Right-hand lateral axis (to the right of the flight path) and the "up" of the cross plane.
            let right = d.cross(Vec3::Y).normalize_or_zero();
            if right == Vec3::ZERO {
                continue;
            }
            let up = right.cross(d);
            let r = p - c;
            if r.dot(d).abs() > 120.0 {
                continue; // beyond this trail segment
            }
            let (y, z) = (r.dot(right), r.dot(up));
            let half = 0.5 * wp.b0 * spread;
            let rc = 0.05 * tr.span.max(4.0);
            // Left vortex clockwise (seen from behind), right vortex anticlockwise: downwash between.
            let mut vy = 0.0;
            let mut vz = 0.0;
            for (yc, g) in [(-half, -gamma), (half, gamma)] {
                let (dy, dz) = (y - yc, z);
                let r2 = dy * dy + dz * dz + 1e-3;
                let k = g / (2.0 * std::f32::consts::PI * r2) * (1.0 - (-r2 / (rc * rc)).exp());
                vy += -dz * k;
                vz += dy * k;
            }
            v += right * vy + up * vz;
        }
        if v.is_finite() { v.clamp_length_max(40.0) } else { Vec3::ZERO }
    }

    /// Wake severity (0..4) at a point: how hard an encounter would be.
    pub fn severity(&self, p: Vec3, t: f32, min_age: f32) -> f32 {
        let v = self.velocity(p, t, min_age).length();
        (v / 4.0).clamp(0.0, 4.0)
    }

    /// Number of live trail points (for the HUD / tests).
    pub fn points(&self) -> usize {
        self.trails.iter().map(|t| t.pts.len()).sum()
    }

    /// Generators with a trail.
    pub fn sources(&self) -> Vec<(u64, Vec2)> {
        self.trails.iter().filter_map(|t| t.pts.back().map(|p| (t.id, Vec2::new(p.pos.x, p.pos.z)))).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec3;

    #[test]
    fn heavy_wake_rolls_and_decays() {
        let mut w = WakeField::default();
        // An A320-class generator (64 t, 34 m span) flying north at 75 m/s, 600 m up.
        for i in 0..40 {
            let t = i as f32 * 0.5;
            w.push(7, vec3(0.0, 600.0, -75.0 * t), vec3(0.0, 0.0, -75.0), 64_000.0, 34.0, true, 0.0, t);
        }
        let t = 30.0;
        // A point laid at t = 10 (z = -750): find the vortex pair centre now.
        let wp = WakePoint { pos: vec3(0.0, 600.0, -750.0), dir: vec3(0.0, 0.0, -1.0), gamma0: 64_000.0 * 9.81 / (RHO * 75.0 * 26.7), b0: 26.7, t: 10.0, ground: 0.0 };
        let (c, gamma, _) = WakeField::state(&wp, t, Vec3::ZERO);
        assert!(c.y < 600.0 - 20.0 && c.y > 600.0 - 60.0, "sank ~1.5 m/s: {}", c.y);
        assert!(gamma > 200.0, "Γ {gamma}");
        // Between the vortices: downwash; just outside: upwash; at the core: strong rolling flow.
        let mid = w.velocity(c, t, 5.0);
        assert!(mid.y < -2.0, "downwash between the vortices {mid:?}");
        let out = w.velocity(c + vec3(25.0, 0.0, 0.0), t, 5.0);
        assert!(out.y > 2.0, "upwash outboard {out:?}");
        // Heading north, right = east (+x): the right vortex sits at +x ≈ 13 m.
        let above = w.velocity(c + vec3(13.3, 3.0, 0.0), t, 5.0);
        let below = w.velocity(c + vec3(13.3, -3.0, 0.0), t, 5.0);
        assert!(above.x * below.x < 0.0 && above.length() > 5.0, "rolling flow {above:?} {below:?}");
        // Far away / long after: nothing.
        assert!(w.velocity(c + vec3(400.0, 0.0, 0.0), t, 5.0).length() < 0.1);
        let late = w.velocity(c, 400.0, 5.0);
        assert!(late.length() < 0.5, "decayed {late:?}");
        // Own fresh wake is ignored.
        assert_eq!(w.velocity(vec3(0.0, 600.0, -75.0 * 19.5), 19.6, 8.0), Vec3::ZERO);
    }
}
