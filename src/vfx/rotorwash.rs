//! Rotor wash (Track Q): the downwash of a hovering helicopter or drone
//! hitting the ground and spreading out as a radial wall jet — dust over
//! land, spray over water. Strength from the rotor's own induced velocity
//! and thrust (src/rotor), fading with height above the ground (≈ 2.5
//! rotor diameters) and as the wake is swept back at speed; the impact
//! point is carried downwind / aft with the skewed wake.

use super::{Ctx, Kind, Vfx};
use crate::flight::Environment;
use glam::{Vec2, Vec3};

impl Vfx {
    pub(super) fn rotor_wash(&mut self, c: &Ctx, dt: f32, gh: &dyn Fn(Vec3) -> f32) {
        let Some(rd) = c.def.rotor.as_ref() else { return };
        let ac = c.ac;
        if ac.crashed.is_some() {
            return;
        }
        let o = ac.rotor.out;
        let (radius, hub) = match rd.as_ref() {
            crate::rotor::RotorDef::Heli(h) => (h.radius, h.hub),
            crate::rotor::RotorDef::Quad(q) => (0.15, Vec3::Y * q.motors[0].y),
        };
        let quad = rd.quad().is_some();
        let d = 2.0 * radius;
        let hub_w = c.pos + c.rot * hub;
        let g = gh(hub_w);
        let h = hub_w.y - g;
        // Far-wake velocity ≈ 2 v_i; it skews back with the air speed.
        let vi = o.v_induced.abs().max((o.thrust.max(0.0) / (2.0 * 1.2 * std::f32::consts::PI * radius * radius)).sqrt());
        let w_down = 2.0 * vi;
        let air = ac.vel - self.wind;
        let skew = (Vec2::new(air.x, air.z).length() / w_down.max(1.0)).min(3.0);
        let k = super::smoothstep(2.6 * d, 0.4 * d, h) * (1.0 - super::smoothstep(0.8, 2.2, skew)) * (o.thrust / (c.def.mass * 9.81)).clamp(0.0, 1.3);
        if k < 0.02 || w_down < 2.0 {
            return;
        }
        // Where the wake meets the ground: carried along by the air for the fall time.
        let t_fall = h / w_down.max(1.0);
        let centre = hub_w - Vec3::new(air.x, 0.0, air.z) * t_fall * 0.7 + self.wind * t_fall;
        let water = c.world.is_water(Vec2::new(centre.x, centre.z));
        // Ground-level outwash peaks just outside the disc and decays outwards.
        let v_out = w_down * (0.9 - 0.25 * (h / d).min(2.0)).max(0.3);
        let rate = if quad { 90.0 } else { 150.0 } * k;
        let n = self.count(0x0057_A5E1, rate, dt);
        for _ in 0..n {
            let a = self.rnd() * std::f32::consts::TAU;
            let r = radius * (0.7 + 1.6 * self.rnd() * self.rnd());
            let dir = Vec3::new(a.cos(), 0.0, a.sin());
            let mut p = centre + dir * r;
            p.y = gh(p) + if quad { 0.02 } else { 0.25 + 0.4 * self.rnd() };
            let speed = v_out * (0.6 + 0.6 * self.rnd()) * (radius / r).sqrt();
            let up = if water { 1.5 + 2.5 * self.rnd() } else { 0.3 + 1.2 * self.rnd() };
            let v = dir * speed + Vec3::Y * up + self.wind * 0.5;
            if quad {
                let s = 0.04 + 0.05 * self.rnd();
                let (kind, col, alpha) = if water { (Kind::Spray, Vec3::new(0.85, 0.9, 0.95), 0.35) } else { (Kind::Dust, Vec3::new(0.52, 0.47, 0.38), 0.18) };
                self.emit(kind, p, v, 0.7, s, s * 6.0, col, alpha * k.min(1.0), 1.6, 0.05, 0.0);
            } else if water {
                // Spray ring: droplets lifted off the surface, plus fine mist.
                let s = 0.5 + 0.6 * self.rnd();
                self.emit(Kind::Spray, p, v, 1.6, s, s * 4.0, Vec3::new(0.86, 0.9, 0.94), 0.45 * k.min(1.0), 0.8, 0.25, 0.0);
                if self.rnd() < 0.5 {
                    self.emit(Kind::Steam, p + Vec3::Y * 0.5, v * 0.6 + Vec3::Y, 3.0, 1.5, 9.0, Vec3::new(0.82, 0.86, 0.9), 0.22 * k.min(1.0), 0.9, 0.0, 0.15);
                }
            } else {
                let s = 0.6 + 0.7 * self.rnd();
                self.emit(Kind::Dust, p, v, 2.0, s, s * 4.5, Vec3::new(0.55, 0.50, 0.41), 0.13 * k.min(1.0), 0.9, 0.06, 0.05);
            }
        }
    }
}
