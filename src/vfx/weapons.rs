//! Weapon effects (Track W) on the shared particle system: muzzle flashes
//! and gun smoke, spent cases, hit sparks and HE flashes, dust where rounds
//! strike the ground, missile motor plumes and smoke trails, flares (white-hot
//! with their smoke trails) and chaff, warhead bursts, the other aircraft's
//! wrecks, and damaged aircraft trailing oil smoke, fuel / glycol vapour or
//! fire.

use super::{Kind, Vfx, mk};
use crate::debris::Fx as DFx;
use crate::weapons::{Combat, Fx};
use glam::{Quat, Vec2, Vec3};

impl Vfx {
    /// Emit this frame's weapon effects (after [`Vfx::update`]). `player`:
    /// the local aircraft's pose and velocity (its own damage smoke).
    pub fn weapons(&mut self, c: &mut Combat, player: Option<(Vec3, Quat, Vec3)>, world: &crate::world::World, dt: f32) {
        let gh = |p: Vec3| crate::flight::Environment::ground_height(world, Vec2::new(p.x, p.z));
        for f in std::mem::take(&mut c.fx) {
            self.weapon_fx(&f, dt, &gh);
        }
        for f in std::mem::take(&mut c.debris_fx) {
            self.debris_fx(&f, &gh);
        }
        // Other aircraft's wreck fires.
        for u in &c.units {
            let Some(a) = &u.ai else { continue };
            if let Some(d) = &a.debris {
                for (k, f) in d.fires.iter().enumerate() {
                    let v = f.body.and_then(|b| d.bodies.get(b)).map_or(Vec3::ZERO, |b| b.vel);
                    self.fire(5000 + u.id * 16 + k as u32, f, v, dt);
                }
            }
        }
        // Missiles: motor plume and smoke trail while burning.
        for m in &c.missiles {
            let noz = m.pos - m.dir() * (m.def.length * 0.5 + 0.1);
            if m.burning() {
                self.trail(10_000 + m.id, mk::SMOKE, noz, 0.45, 0.85, 22.0, 0.55, 0.0, 0.0, 5.0);
                // Motor plume: a short, bright tongue riding with the missile.
                let n = self.count(10_000 + m.id, 360.0, dt);
                for _ in 0..n {
                    let v = m.vel - m.dir() * 35.0 + self.rsph() * 3.0;
                    if let Some(p) = self.emit(Kind::Fire, noz - m.vel * dt * self.rnd() * 0.2, v, 0.05, 0.22, 0.45, Vec3::splat(0.035), 1.0, 0.0, 0.0, 0.0) {
                        p.temp0 = 2300.0;
                        p.src_vel = m.vel;
                        p.stretch = 4.0;
                        p.drag = 0.0;
                    }
                }
            }
        }
        // Flares: white-hot, falling, each with its own smoke trail; chaff: a glittering puff.
        for x in &c.decoys {
            if x.flare && x.t < 4.6 {
                self.trail(20_000 + x.id, mk::SMOKE, x.pos, 0.7, 0.55, 14.0, 0.9, 0.0, 0.0, 3.0);
                let n = self.count(20_000 + x.id, 40.0, dt);
                for _ in 0..n {
                    if let Some(p) = self.emit(Kind::Fire, x.pos, x.vel * 0.8 + self.rsph() * 3.0, 0.15, 0.3, 0.5, Vec3::splat(0.04), 1.0, 1.5, 0.0, 0.0) {
                        p.temp0 = 2600.0;
                        p.src_vel = x.vel;
                        p.stretch = 1.6;
                    }
                }
            }
        }
        // Damaged aircraft: smoke / vapour / fire trails.
        let mut srcs: Vec<(u32, Vec3, Quat, Vec3, (f32, f32, f32), Vec3, Vec3)> = Vec::new();
        for u in &c.units {
            let (pos, rot, vel, ac) = match (&u.ai, player) {
                (Some(a), _) => (a.ac.pos, a.ac.rot, a.ac.vel, &a.ac),
                (None, Some((p, r, v))) if Some(u.id) == c.local => {
                    // The local aircraft (its systems' engine fire is in its own Systems).
                    let tr = (u.bd.oil, u.bd.leak.max(u.bd.glycol), u.bd.fire);
                    let e = c.defs[u.def].engine_pos.first().copied().unwrap_or(Vec3::ZERO);
                    srcs.push((u.id, p, r, v, tr, e, fire_at(&u.bd, &c.defs[u.def])));
                    continue;
                }
                _ => continue,
            };
            if ac.crashed.is_some() {
                continue;
            }
            let tr = u.bd.trail(ac);
            let e = c.defs[u.def].engine_pos.first().copied().unwrap_or(Vec3::ZERO);
            srcs.push((u.id, pos, rot, vel, tr, e, fire_at(&u.bd, &c.defs[u.def])));
        }
        for (id, pos, rot, vel, (oil, vap, fire), eng, fat) in srcs {
            if oil > 0.02 {
                let at = pos + rot * eng;
                let n = self.count(30_000 + id, 25.0 + 70.0 * oil, dt);
                for _ in 0..n {
                    let g = 0.05 + 0.08 * (1.0 - oil);
                    let o = self.rsph() * 0.4 - vel * dt * self.rnd();
                    self.emit(Kind::Smoke, at + o, vel * 0.15, 6.0 + 6.0 * oil, 0.7, 4.0 + 5.0 * oil, Vec3::new(g, g * 0.95, g * 0.9), 0.35 + 0.35 * oil, 0.9, 0.0, 0.4);
                }
            }
            if vap > 0.02 {
                // Fuel / glycol vapour: a white wisp (lit condensation-like).
                let at = pos + rot * fat;
                self.trail(31_000 + id, mk::TRAIL, at, 0.9, 0.25 + 0.45 * vap, 5.0, 1.2, 0.0, 0.0, 4.0);
            }
            if fire > 0.02 {
                let at = pos + rot * fat;
                let n = self.count(32_000 + id, 60.0 + 120.0 * fire, dt);
                for _ in 0..n {
                    let o = self.rsph() * 0.5 - vel * dt * self.rnd();
                    if let Some(p) = self.emit(Kind::Fire, at + o, vel * 0.85, 0.25 + 0.3 * fire, 0.5 + 0.5 * fire, 1.2 + 1.5 * fire, Vec3::splat(0.035), 1.0, 2.0, 0.0, 0.0) {
                        p.temp0 = 1450.0;
                        p.src_vel = vel;
                        p.stretch = 2.5 + 2.0 * fire;
                    }
                }
                let n = self.count(33_000 + id, 30.0 + 60.0 * fire, dt);
                for _ in 0..n {
                    let o = self.rsph() * 0.8 - vel * dt * self.rnd();
                    self.emit(Kind::Smoke, at + o - vel * 0.03, vel * 0.1, 10.0, 1.0, 7.0 + 6.0 * fire, Vec3::splat(0.035), 0.75, 0.8, 0.0, 0.8);
                }
            }
        }
    }

    fn weapon_fx(&mut self, f: &Fx, dt: f32, gh: &dyn Fn(Vec3) -> f32) {
        match *f {
            Fx::Muzzle { pos, dir, vel, cal, rounds, .. } => {
                // Flash: a short-lived hot gas ball ahead of the muzzle (one per frame per gun).
                let big = cal > 0.015;
                let r = if big { 0.45 } else { 0.22 };
                if let Some(p) = self.emit(Kind::Fire, pos + dir * r * 1.2, vel + dir * 40.0, 0.035, r, r * 1.7, Vec3::splat(0.03), 1.0, 0.0, 0.0, 0.0) {
                    p.temp0 = 2400.0;
                    p.src_vel = vel;
                    p.stretch = 1.8;
                    p.life = 0.035;
                }
                // Propellant smoke, drifting back.
                let n = (rounds as f32 * if big { 0.8 } else { 0.35 }).ceil() as usize;
                for _ in 0..n.min(6) {
                    let o = self.rsph() * 0.2 - vel * dt * self.rnd();
                    let g = 0.55 + 0.1 * self.rnd();
                    self.emit(Kind::Smoke, pos + o, vel * 0.55 + dir * 15.0, 1.6, 0.15, if big { 2.2 } else { 1.3 }, Vec3::new(g, g, g * 0.97), 0.18, 2.5, 0.0, 0.0);
                }
            }
            Fx::Case { pos, vel, big } => {
                let v = vel + self.rsph() * 2.0;
                self.emit(Kind::Dust, pos, v, 3.0, if big { 0.05 } else { 0.03 }, if big { 0.05 } else { 0.03 }, Vec3::new(0.55, 0.42, 0.18), 1.0, 0.05, 1.0, 0.0);
            }
            Fx::Hit { pos, normal, vel, he, cal } => {
                // Sparks off the skin; HE / incendiary: a bright flash and a dark puff.
                let n = if he {
                    14
                } else if cal > 0.01 {
                    8
                } else {
                    4
                };
                for _ in 0..n {
                    let d = (normal + self.rsph() * 0.9).normalize_or(normal);
                    let v = vel + d * (40.0 + 80.0 * self.rnd());
                    self.emit(Kind::Spark, pos, v, 0.25 + 0.35 * self.rnd(), 0.04, 0.02, Vec3::new(1.0, 0.7, 0.35) * 10.0, 1.0, 0.6, 1.0, 0.0);
                }
                if he {
                    if let Some(p) = self.emit(Kind::Fire, pos + normal * 0.2, vel, 0.07, 0.5, 1.1, Vec3::splat(0.03), 1.0, 0.0, 0.0, 0.0) {
                        p.temp0 = 2500.0;
                        p.src_vel = vel;
                        p.life = 0.07;
                    }
                    let g = 0.1;
                    self.emit(Kind::Smoke, pos, vel * 0.7, 1.4, 0.3, 1.6, Vec3::new(g, g, g), 0.55, 1.6, 0.0, 0.0);
                } else {
                    let g = 0.35;
                    self.emit(Kind::Smoke, pos, vel * 0.8, 0.6, 0.1, 0.5, Vec3::new(g, g, g), 0.3, 2.0, 0.0, 0.0);
                }
            }
            Fx::Ground { pos, he } => {
                let n = if he { 10 } else { 4 };
                for _ in 0..n {
                    let v = Vec3::new(self.rnd() - 0.5, 1.6 + self.rnd(), self.rnd() - 0.5) * (6.0 + 6.0 * self.rnd());
                    self.emit(Kind::Dust, pos, v, 1.6 + self.rnd(), 0.4, 1.8, Vec3::new(0.32, 0.28, 0.22), 0.7, 1.2, 0.6, 0.0);
                }
                if he {
                    if let Some(p) = self.emit(Kind::Fire, pos + Vec3::Y * 0.3, Vec3::Y * 2.0, 0.08, 0.5, 1.0, Vec3::splat(0.03), 1.0, 0.0, 0.0, 0.0) {
                        p.temp0 = 2400.0;
                        p.life = 0.08;
                    }
                }
            }
            Fx::Burst { pos, vel, energy } => {
                // Warhead / tank: a fireball, a black cloud and fragments.
                // (In the air the smoke plume starts at the burst, not at the ground.)
                let radius = 3.0 + 7.0 * energy.min(1.0);
                let ground = gh(pos);
                let base = if pos.y - ground > radius * 3.0 { pos.y - radius - 1.0 } else { ground };
                self.debris_fx(&DFx::Explosion { pos, vel, energy: energy.clamp(0.1, 1.0), radius }, &|_| base);
                for _ in 0..(20.0 + 60.0 * energy.min(1.0)) as usize {
                    let v = vel + self.rsph().normalize_or(glam::Vec3::Y) * (150.0 + 350.0 * self.rnd());
                    self.emit(Kind::Spark, pos, v, 0.4 + 0.6 * self.rnd(), 0.05, 0.03, Vec3::new(1.0, 0.65, 0.3) * 12.0, 1.0, 0.8, 1.0, 0.0);
                }
            }
            Fx::Launch { pos, vel } => {
                for _ in 0..12 {
                    let g = 0.75;
                    let o = self.rsph() * 0.8;
                    self.emit(Kind::Smoke, pos + o, vel * 0.7, 2.5, 0.6, 3.0, Vec3::new(g, g, g * 0.98), 0.45, 1.8, 0.0, 0.0);
                }
            }
            Fx::Dispense { pos, vel, flare } => {
                if flare {
                    self.emit(Kind::Spark, pos, vel, 0.15, 0.25, 0.2, Vec3::new(1.0, 0.95, 0.8) * 30.0, 1.0, 0.5, 0.2, 0.0);
                } else {
                    // Chaff: a cloud of foil glints.
                    for _ in 0..30 {
                        let v = vel * 0.4 + self.rsph() * 12.0;
                        self.emit(Kind::Spray, pos + self.rsph() * 1.5, v, 3.0, 0.03, 0.03, Vec3::new(0.85, 0.85, 0.9), 0.9, 3.0, 0.05, 0.0);
                    }
                }
            }
        }
    }
}

/// Where a damaged aircraft's fire / vapour comes from (body frame).
fn fire_at(bd: &crate::weapons::bdmg::BattleDamage, def: &crate::flight::AircraftDef) -> Vec3 {
    use crate::weapons::hitbox::HitPart;
    let semi = def.wingspan * 0.5;
    match bd.fire_at {
        HitPart::FuelWingL => Vec3::new(semi * 0.35, def.wing_height, -0.5),
        HitPart::FuelWingR => Vec3::new(-semi * 0.35, def.wing_height, -0.5),
        _ if bd.glycol > bd.leak => Vec3::new(1.6, def.wing_height - 0.4, -0.6),
        _ => {
            def.engine_pos.first().copied().unwrap_or(Vec3::ZERO) * Vec3::new(1.0, 1.0, if def.engine_kind == crate::flight::EngineKind::Jet { 1.0 } else { 0.0 })
                + Vec3::new(0.0, -0.2, -0.8)
        }
    }
}
