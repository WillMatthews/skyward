//! Float spray (Track H2b): the bow wave and its sheets of spray while the
//! floats push through the water, the whisker spray from the chines and the
//! rooster tail behind the step once they are planing — fed by the hull
//! model's spray sources (`ac.water.last.spray`, src/water.rs) — plus the
//! prop-wash mist blown off the water at high power on the water.

use super::{Ctx, Kind, Vfx};
use crate::flight::Environment;
use glam::{Vec2, Vec3};

impl Vfx {
    pub(super) fn float_spray(&mut self, c: &Ctx, dt: f32, gh: &dyn Fn(Vec3) -> f32) {
        let ac = c.ac;
        if ac.crashed.is_some() || !c.def.on_water() || !ac.water.last.wet {
            return;
        }
        // The sources are in the physics frame; the render pose is interpolated.
        let shift = c.pos - ac.pos;
        for (k, s) in ac.water.last.spray.iter().enumerate() {
            if s.rate <= 0.02 {
                continue;
            }
            let base = s.pos + shift;
            let rate = if s.planing { 140.0 } else { 90.0 } * s.rate;
            let n = self.count(0x5900 + k as u32, rate, dt);
            for _ in 0..n {
                let d = self.rsph();
                let p = Vec3::new(base.x, gh(base) + 0.05, base.z) + Vec3::new(d.x, 0.0, d.z) * 0.25;
                let v = s.vel * (0.7 + 0.5 * self.rnd()) + d * 0.8;
                let size = if s.planing { 0.12 + 0.18 * self.rnd() } else { 0.18 + 0.25 * self.rnd() };
                self.emit(Kind::Spray, p, v, 1.2 + 0.8 * self.rnd(), size, size * 5.0, Vec3::new(0.86, 0.9, 0.94), (0.35 + 0.35 * s.rate).min(0.7), 0.6, 1.0, 0.0);
                if self.rnd() < 0.25 * s.rate {
                    // Fine mist drifting off the sheet.
                    self.emit(Kind::Steam, p + Vec3::Y * 0.4, v * 0.4 + self.wind * 0.5, 2.5, 0.6, 4.0, Vec3::new(0.84, 0.88, 0.92), 0.16 * s.rate, 1.2, 0.0, 0.05);
                }
            }
        }
        // Prop wash over the water at high power and low speed: a mist ring.
        let n_eng = c.def.engine_count.min(c.def.engine_pos.len());
        let thrust = (0..n_eng).map(|i| ac.engine_thrust(i)).sum::<f32>();
        let k = (thrust / (c.def.mass * 9.81 * 0.25)).clamp(0.0, 1.0) * (1.0 - (ac.airspeed() / 25.0).clamp(0.0, 1.0));
        if k > 0.05 {
            for (i, e) in c.def.engine_pos.iter().enumerate().take(n_eng) {
                let hub = c.pos + c.rot * *e;
                let behind = hub - (c.rot * Vec3::Z) * 3.0;
                let n = self.count(0x5A00 + i as u32, 60.0 * k, dt);
                for _ in 0..n {
                    let a = self.rnd() * std::f32::consts::TAU;
                    let r = c.def.prop_radius * (0.6 + 0.8 * self.rnd());
                    let mut p = behind + Vec3::new(a.cos() * r, 0.0, a.sin() * r);
                    if !c.world.is_water(Vec2::new(p.x, p.z)) {
                        continue;
                    }
                    p.y = gh(p) + 0.1;
                    let v = -(c.rot * Vec3::Z) * (6.0 + 6.0 * self.rnd()) + Vec3::Y * (0.8 + self.rnd()) + self.wind * 0.5;
                    self.emit(Kind::Steam, p, v, 1.8, 0.5, 3.0, Vec3::new(0.85, 0.89, 0.93), 0.18 * k, 1.2, 0.05, 0.0);
                }
            }
        }
    }
}
