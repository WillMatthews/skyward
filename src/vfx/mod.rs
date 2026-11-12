//! Visual effects (Track L): a CPU-simulated particle system drawn as GPU
//! instanced, lit, soft particles (fire, smoke, dust, spray, sparks, tyre
//! smoke, condensation), camera-facing ribbons (contrails, vortex cores,
//! smoke trails), procedural meshes (vapour cone, wing-surface vapour,
//! afterburner shock diamonds) and screen-space heat haze.
//!
//! Everything is physically motivated and cheap:
//! * fire / smoke / dust / spray / sparks come from the wreckage
//!   (`debris::Fx`, burning fuel), touchdowns and failed engines;
//! * contrails form when the exhaust plume saturates (Schmidt–Appleman,
//!   `atmo.rs`) and persist / spread only in ice-supersaturated air;
//! * vortex-core, wing-surface and transonic (Prandtl–Glauert) vapour
//!   appear when the local pressure drop cools humid air below its dew point;
//! * shock diamonds follow the under-expanded afterburner jet; heat haze
//!   trails every running jet and rises over fires.
//!
//! The renderer is `gpu.rs` + `shaders/vfx.wgsl` (lighting from the scene's
//! sun / moon, sky, cloud shadows and aerial perspective; soft against the
//! scene's eye-distance buffer; sorted back to front).

pub mod atmo;
pub mod gpu;

use crate::debris::{Debris, Fx};
use crate::flight::{Aircraft, AircraftDef, EngineKind, GRAVITY};
use crate::world::World;
use glam::{Quat, Vec2, Vec3};
use std::collections::VecDeque;

/// Particle kinds (== `vfx.wgsl`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    /// Lit, dense, dark (fuel smoke) to grey (engine smoke).
    Smoke = 0,
    /// Emissive flame / fireball.
    Fire = 1,
    /// Lit dust / dirt.
    Dust = 2,
    /// Lit water spray (stretched by velocity).
    Spray = 3,
    /// Emissive spark / ember (stretched).
    Spark = 4,
    /// Lit white condensation / tyre smoke (strong forward scattering).
    Steam = 5,
}

#[derive(Clone, Copy, Debug)]
struct Particle {
    pos: Vec3,
    vel: Vec3,
    age: f32,
    life: f32,
    size0: f32,
    size1: f32,
    kind: Kind,
    /// Albedo (lit kinds) or emission (scene radiance per unit, × pre-exposure in the shader).
    col: Vec3,
    alpha: f32,
    /// Velocity relaxation toward the wind (1/s), gravity factor, buoyant acceleration (m/s²).
    drag: f32,
    grav: f32,
    buoy: f32,
    rot: f32,
    seed: f32,
}

/// One particle on the GPU (64 bytes; == `vfx.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuParticle {
    /// Position relative to the frame origin, radius (m).
    pub pos: [f32; 4],
    /// Albedo / emission, opacity.
    pub col: [f32; 4],
    /// Velocity (m/s, for stretched kinds), kind.
    pub vel: [f32; 4],
    /// Age 0..1, rotation, seed, emissive scale.
    pub extra: [f32; 4],
}

/// A ribbon / mesh vertex (32 bytes; == `vfx.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VfxVert {
    /// Position relative to the frame origin, kind.
    pub pos: [f32; 4],
    /// u (along), v (across −1..1), intensity, extra.
    pub uv: [f32; 4],
}

/// Mesh / ribbon kinds (== `vfx.wgsl`).
pub mod mk {
    /// Condensation trail (contrail, vortex core): lit white, soft tube.
    pub const TRAIL: f32 = 10.0;
    /// Smoke trail: lit grey.
    pub const SMOKE: f32 = 11.0;
    /// Vapour sheet / cone: lit white, noisy.
    pub const VAPOUR: f32 = 12.0;
    /// Afterburner core with shock diamonds: emissive.
    pub const DIAMONDS: f32 = 13.0;
    /// Heat haze (distortion pass only).
    pub const HAZE: f32 = 14.0;
}

/// Everything the renderer needs this frame (positions relative to `origin`).
#[derive(Clone, Default)]
pub struct VfxFrame {
    pub origin: Vec3,
    pub particles: Vec<GpuParticle>,
    pub mesh: Vec<VfxVert>,
    pub mesh_idx: Vec<u32>,
    pub haze: Vec<VfxVert>,
    pub haze_idx: Vec<u32>,
    /// Up to four fire lights: position (relative to `origin`), intensity (cd).
    pub fires: [[f32; 4]; 4],
}

impl VfxFrame {
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty() && self.mesh.is_empty() && self.haze.is_empty()
    }
}

#[derive(Clone, Copy, Debug)]
struct TrailPt {
    pos: Vec3,
    born: f32,
    width: f32,
    alpha: f32,
}

#[derive(Clone, Debug)]
struct Trail {
    key: u32,
    kind: f32,
    pts: VecDeque<TrailPt>,
    life: f32,
    growth: f32,
    sink: f32,
    /// Fade in over the first seconds of a point's life (contrails need to mix).
    fade_in: f32,
    /// The emitter's current position (head), if active this frame.
    head: Option<TrailPt>,
    seg: f32,
}

/// What the aircraft looks like to the effects this frame.
pub struct Ctx<'a> {
    pub def: &'a AircraftDef,
    pub ac: &'a Aircraft,
    /// Interpolated render pose.
    pub pos: Vec3,
    pub rot: Quat,
    pub world: &'a World,
    pub debris: Option<&'a mut Debris>,
    pub time: f32,
    pub dt: f32,
    pub eye: Vec3,
    /// Wing-tip deflection (m) for surface effects.
    pub tips: [f32; 2],
    /// Wing planform bins from the model: (station x, leading-edge z, trailing-edge z, y).
    pub planform: &'a [(f32, f32, f32, f32)],
}

const MAX_PARTICLES: usize = 24_000;

pub struct Vfx {
    parts: Vec<Particle>,
    trails: Vec<Trail>,
    rng: std::cell::Cell<u32>,
    time: f32,
    prev_on_ground: bool,
    prev_vs: f32,
    prev_running: [bool; 4],
    eng_smoke: [f32; 4],
    /// Effects state for the mesh effects this frame.
    cone: f32,
    wing_vapour: f32,
    air: atmo::Air,
    wind: Vec3,
    /// Optional: humidity / temperature from ISA (tests, no weather).
    pub isa_only: bool,
    /// Accumulators for rate-based emitters.
    acc: Vec<(u32, f32)>,
}

impl Default for Vfx {
    fn default() -> Self {
        Vfx {
            parts: Vec::with_capacity(4096),
            trails: Vec::new(),
            rng: std::cell::Cell::new(0x9E37_79B9),
            time: 0.0,
            prev_on_ground: true,
            prev_vs: 0.0,
            prev_running: [true; 4],
            eng_smoke: [0.0; 4],
            cone: 0.0,
            wing_vapour: 0.0,
            air: atmo::Air::default(),
            wind: Vec3::ZERO,
            isa_only: false,
            acc: Vec::new(),
        }
    }
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

impl Vfx {
    pub fn clear(&mut self) {
        self.parts.clear();
        self.trails.clear();
        self.acc.clear();
        self.eng_smoke = [0.0; 4];
        self.prev_on_ground = true;
        self.prev_running = [true; 4];
    }

    pub fn particle_count(&self) -> usize {
        self.parts.len()
    }

    pub fn rebase(&mut self, delta: Vec3) {
        for p in &mut self.parts {
            p.pos += delta;
        }
        for t in &mut self.trails {
            for q in &mut t.pts {
                q.pos += delta;
            }
            if let Some(h) = &mut t.head {
                h.pos += delta;
            }
        }
    }

    fn rnd(&self) -> f32 {
        let mut r = self.rng.get();
        r ^= r << 13;
        r ^= r >> 17;
        r ^= r << 5;
        self.rng.set(r);
        (r >> 8) as f32 / 16_777_216.0
    }
    fn rsph(&self) -> Vec3 {
        loop {
            let v = Vec3::new(self.rnd() * 2.0 - 1.0, self.rnd() * 2.0 - 1.0, self.rnd() * 2.0 - 1.0);
            let l = v.length_squared();
            if l <= 1.0 && l > 1e-4 {
                return v;
            }
        }
    }

    /// Rate-based emission: how many to emit this frame for `rate` per second.
    fn count(&mut self, key: u32, rate: f32, dt: f32) -> usize {
        let i = match self.acc.iter().position(|a| a.0 == key) {
            Some(i) => i,
            None => {
                self.acc.push((key, 0.0));
                self.acc.len() - 1
            }
        };
        self.acc[i].1 += rate.max(0.0) * dt;
        let n = self.acc[i].1.floor();
        self.acc[i].1 -= n;
        n as usize
    }

    #[allow(clippy::too_many_arguments)]
    fn emit(&mut self, kind: Kind, pos: Vec3, vel: Vec3, life: f32, size0: f32, size1: f32, col: Vec3, alpha: f32, drag: f32, grav: f32, buoy: f32) {
        if self.parts.len() >= MAX_PARTICLES {
            // Full: the oldest smoke makes room for fresh fire / sparks.
            if matches!(kind, Kind::Smoke | Kind::Dust | Kind::Steam) {
                return;
            }
            if let Some(i) = self.parts.iter().position(|p| p.kind == Kind::Smoke && p.age > 0.5 * p.life) {
                self.parts.swap_remove(i);
            } else {
                return;
            }
        }
        let rot = self.rnd() * std::f32::consts::TAU;
        let seed = self.rnd() * 97.0;
        let life = life * (0.75 + 0.5 * self.rnd());
        self.parts.push(Particle { pos, vel, age: 0.0, life, size0, size1, kind, col, alpha, drag, grav, buoy, rot, seed });
    }

    fn trail_mut(&mut self, key: u32, kind: f32, life: f32, growth: f32, sink: f32, fade_in: f32, seg: f32) -> &mut Trail {
        let i = match self.trails.iter().position(|t| t.key == key) {
            Some(i) => i,
            None => {
                self.trails.push(Trail { key, kind, pts: VecDeque::new(), life, growth, sink, fade_in, head: None, seg });
                self.trails.len() - 1
            }
        };
        let t = &mut self.trails[i];
        t.kind = kind;
        t.life = life;
        t.growth = growth;
        t.sink = sink;
        t.fade_in = fade_in;
        t.seg = seg;
        t
    }

    /// Feed a trail emitter this frame (head at `pos`).
    #[allow(clippy::too_many_arguments)]
    fn trail(&mut self, key: u32, kind: f32, pos: Vec3, width: f32, alpha: f32, life: f32, growth: f32, sink: f32, fade_in: f32, seg: f32) {
        let time = self.time;
        let t = self.trail_mut(key, kind, life, growth, sink, fade_in, seg);
        let head = TrailPt { pos, born: time, width, alpha };
        let need = match t.pts.back() {
            None => true,
            Some(b) => b.pos.distance(pos) > t.seg || time - b.born > 0.5,
        };
        if need {
            t.pts.push_back(head);
            if t.pts.len() > 6000 {
                t.pts.pop_front();
            }
        }
        t.head = Some(head);
    }

    /// Advance and emit. Call once per frame (after the simulation).
    pub fn update(&mut self, mut c: Ctx) {
        let dt = c.dt.clamp(0.0, 0.1);
        self.time = c.time;
        let ac = c.ac;
        let def = c.def;
        let sd = crate::damage::struct_def(def.id);
        // Air and wind at the aircraft (one sample per frame; particles far
        // from it use the same wind, fine at these scales).
        self.air = atmo::air_at(c.world, c.pos, self.isa_only);
        let w = crate::flight::Environment::wind(c.world, c.pos);
        self.wind = if w.is_finite() { w } else { Vec3::ZERO };
        let wind = self.wind;
        let gh = |p: Vec3| crate::flight::Environment::ground_height(c.world, Vec2::new(p.x, p.z));
        for t in &mut self.trails {
            t.head = None;
        }

        // ---- particles
        let mut i = 0;
        while i < self.parts.len() {
            let p = &mut self.parts[i];
            p.age += dt;
            if p.age >= p.life {
                self.parts.swap_remove(i);
                continue;
            }
            let k = (p.drag * dt).min(1.0);
            p.vel += (wind - p.vel) * k;
            let lift = p.buoy * (1.0 - p.age / p.life).max(0.0);
            p.vel.y += (lift - GRAVITY * p.grav) * dt;
            p.pos += p.vel * dt;
            if matches!(p.kind, Kind::Spark | Kind::Spray | Kind::Dust) && p.grav > 0.0 {
                let g = gh(p.pos);
                if p.pos.y < g {
                    p.pos.y = g;
                    if p.kind == Kind::Spark && p.vel.y < -2.0 {
                        p.vel.y *= -0.3;
                        p.vel *= 0.6;
                    } else {
                        p.vel = Vec3::ZERO;
                        p.age = p.age.max(p.life * 0.8);
                    }
                }
            } else if p.pos.y < gh(p.pos) - 0.5 {
                p.pos.y = gh(p.pos);
                p.vel.y = p.vel.y.abs() * 0.2;
            }
            i += 1;
        }
        // ---- trails drift with the wind and sink (wake vortices descend)
        for t in &mut self.trails {
            let time = self.time;
            for q in &mut t.pts {
                let age = time - q.born;
                q.pos += (wind + Vec3::new(0.0, -t.sink * (-age / 40.0).exp(), 0.0)) * dt;
            }
            while t.pts.front().is_some_and(|q| time - q.born > t.life) {
                t.pts.pop_front();
            }
        }
        self.trails.retain(|t| !t.pts.is_empty() || t.head.is_some());

        // ---- wreckage: impacts, slides, fireballs, fires, smoking pieces
        if let Some(d) = c.debris.as_deref_mut() {
            let fx: Vec<Fx> = std::mem::take(&mut d.fx);
            for f in fx {
                self.debris_fx(&f, &gh);
            }
            let fires: Vec<crate::debris::Fire> = d.fires.clone();
            for (k, f) in fires.iter().enumerate() {
                self.fire(k as u32, f, dt);
            }
            let bodies: Vec<(Vec3, Vec3, f32, bool, bool, usize)> = d.bodies.iter().enumerate().map(|(i, b)| (b.pos, b.vel, b.half.max_element(), b.smoking && !b.in_water, b.sleeping, i)).collect();
            for (pos, vel, r, smoking, sleeping, i) in bodies {
                if smoking && d.time < 90.0 {
                    let n = self.count(1000 + i as u32, if sleeping { 1.5 } else { 6.0 }, dt);
                    for _ in 0..n {
                        let o = self.rsph() * r * 0.5;
                        let v = vel * 0.2 + Vec3::new(0.0, 1.5, 0.0);
                        self.emit(Kind::Smoke, pos + o, v, 9.0, 0.6 + 0.2 * r, 4.0 + r, Vec3::splat(0.06), 0.55, 0.6, 0.0, 1.5);
                    }
                }
            }
        }

        if ac.crashed.is_none() {
            self.flight_effects(&c, sd, dt, &gh);
        } else {
            self.cone = 0.0;
            self.wing_vapour = 0.0;
        }
        self.prev_on_ground = ac.on_ground;
        self.prev_vs = ac.vertical_speed();
    }

    fn debris_fx(&mut self, f: &Fx, gh: &dyn Fn(Vec3) -> f32) {
        use crate::audio::params::Surface;
        match *f {
            Fx::Explosion { pos, vel, energy, radius } => {
                // Fireball: a burst of expanding flame that rolls up into black smoke.
                let n = (60.0 + 160.0 * energy) as usize;
                for _ in 0..n {
                    let d = self.rsph();
                    let v = vel * 0.4 + d * radius * (0.8 + 0.8 * self.rnd()) + Vec3::Y * radius * 0.3;
                    let s0 = radius * (0.12 + 0.12 * self.rnd());
                    self.emit(Kind::Fire, pos + d * radius * 0.2, v, 1.4 + 1.6 * energy, s0, s0 * 2.6, Vec3::new(1.0, 0.45, 0.12) * (3.0 + 4.0 * energy), 1.0, 1.6, 0.0, 6.0);
                }
                for _ in 0..(n / 2) {
                    let d = self.rsph();
                    let v = vel * 0.3 + d * radius * 0.5 + Vec3::Y * radius * 0.35;
                    let s0 = radius * 0.25;
                    self.emit(Kind::Smoke, pos + d * radius * 0.4, v, 14.0 + 10.0 * energy, s0, radius * 1.4, Vec3::splat(0.035), 0.85, 0.8, 0.0, 4.0);
                }
                for _ in 0..(n / 3) {
                    let d = self.rsph();
                    let v = vel * 0.5 + d * radius * 2.2 + Vec3::Y * radius;
                    self.emit(Kind::Spark, pos, v, 1.5 + 2.0 * self.rnd(), 0.08, 0.05, Vec3::new(1.0, 0.55, 0.2) * 6.0, 1.0, 0.3, 1.0, 0.0);
                }
                // Dust ring from the blast on the ground.
                if pos.y - gh(pos) < radius {
                    for k in 0..24 {
                        let a = k as f32 / 24.0 * std::f32::consts::TAU;
                        let dir = Vec3::new(a.cos(), 0.0, a.sin());
                        let p = Vec3::new(pos.x, gh(pos) + 0.5, pos.z);
                        self.emit(Kind::Dust, p, dir * radius * 1.4 + Vec3::Y, 6.0, radius * 0.15, radius * 0.6, Vec3::new(0.32, 0.27, 0.2), 0.5, 1.2, 0.0, 0.3);
                    }
                }
            }
            Fx::Impact { pos, vel, speed, mass, surface } => {
                let e = (speed / 20.0).min(2.0) * (mass / 2000.0).sqrt().clamp(0.2, 3.0);
                let n = (6.0 + 30.0 * e) as usize;
                match surface {
                    Surface::Water => {
                        for _ in 0..n * 2 {
                            let d = self.rsph();
                            let v = vel * 0.2 + Vec3::new(d.x * 6.0, 6.0 + 10.0 * self.rnd() * e.min(1.5), d.z * 6.0);
                            self.emit(Kind::Spray, pos, v, 1.2 + self.rnd(), 0.25, 0.4, Vec3::splat(0.85), 0.75, 0.2, 1.0, 0.0);
                        }
                        for _ in 0..n / 2 {
                            let d = self.rsph();
                            self.emit(Kind::Steam, pos + d * 2.0, vel * 0.1 + Vec3::Y * 2.0, 4.0, 1.5 + e, 6.0 + 4.0 * e, Vec3::splat(0.9), 0.35, 0.8, 0.0, 0.3);
                        }
                    }
                    Surface::Paved | Surface::Building | Surface::Metal => {
                        for _ in 0..n {
                            let d = self.rsph();
                            let v = vel * 0.6 + d * 8.0 + Vec3::Y * 3.0;
                            self.emit(Kind::Spark, pos, v, 0.4 + 0.6 * self.rnd(), 0.05, 0.03, Vec3::new(1.0, 0.7, 0.35) * 8.0, 1.0, 0.5, 1.0, 0.0);
                        }
                        for _ in 0..n / 3 {
                            let d = self.rsph();
                            self.emit(Kind::Dust, pos + d, vel * 0.2 + Vec3::Y * 1.5, 4.0, 0.8, 4.0 * e.max(0.5), Vec3::new(0.38, 0.36, 0.34), 0.45, 1.0, 0.0, 0.2);
                        }
                    }
                    Surface::Ground => {
                        for _ in 0..n {
                            let d = self.rsph();
                            let v = vel * 0.35 + Vec3::new(d.x * 5.0, 3.0 + 6.0 * self.rnd(), d.z * 5.0);
                            self.emit(Kind::Dust, pos + d * 0.5, v, 2.0 + 2.0 * self.rnd(), 0.15, 0.25, Vec3::new(0.22, 0.17, 0.11), 0.95, 0.3, 1.0, 0.0);
                        }
                        for _ in 0..n {
                            let d = self.rsph();
                            self.emit(Kind::Dust, pos + d, vel * 0.25 + Vec3::Y * (1.0 + 2.0 * self.rnd()), 5.0 + 4.0 * e.min(1.0), 1.0, 4.0 + 5.0 * e.min(1.5), Vec3::new(0.36, 0.3, 0.22), 0.55, 0.9, 0.0, 0.25);
                        }
                    }
                }
            }
            Fx::Slide { pos, vel, speed, mass, surface } => {
                let k = (speed / 25.0).min(2.0) * (mass / 3000.0).sqrt().clamp(0.3, 2.5);
                let dt = 1.0 / 60.0;
                match surface {
                    Surface::Water => {
                        let n = self.count(7 + (pos.x as u32 & 0xff), 120.0 * k, dt);
                        for _ in 0..n {
                            let d = self.rsph();
                            let v = vel * 0.3 + Vec3::new(d.x * 4.0, 3.0 + 5.0 * self.rnd(), d.z * 4.0);
                            self.emit(Kind::Spray, pos + d, v, 1.0, 0.2, 0.35, Vec3::splat(0.85), 0.6, 0.2, 1.0, 0.0);
                        }
                    }
                    Surface::Paved | Surface::Building | Surface::Metal => {
                        let n = self.count(8 + (pos.x as u32 & 0xff), 300.0 * k, dt);
                        for _ in 0..n {
                            let d = self.rsph();
                            let v = vel * 0.8 + d * 4.0 + Vec3::Y * 1.5;
                            self.emit(Kind::Spark, pos, v, 0.25 + 0.5 * self.rnd(), 0.04, 0.02, Vec3::new(1.0, 0.75, 0.4) * 9.0, 1.0, 0.4, 1.0, 0.0);
                        }
                        let n = self.count(9 + (pos.x as u32 & 0xff), 10.0 * k, dt);
                        for _ in 0..n {
                            self.emit(Kind::Dust, pos, vel * 0.3 + Vec3::Y, 3.0, 0.6, 3.0, Vec3::new(0.4, 0.39, 0.37), 0.35, 1.0, 0.0, 0.2);
                        }
                    }
                    Surface::Ground => {
                        let n = self.count(10 + (pos.x as u32 & 0xff), 40.0 * k, dt);
                        for _ in 0..n {
                            let d = self.rsph();
                            self.emit(Kind::Dust, pos + d * 0.8, vel * 0.3 + Vec3::Y * (0.5 + 2.0 * self.rnd()), 4.0 + 3.0 * self.rnd(), 0.8, 3.5 + 3.0 * k.min(1.0), Vec3::new(0.34, 0.28, 0.2), 0.55, 0.9, 0.0, 0.15);
                        }
                    }
                }
            }
            Fx::Separation { pos, vel, mass } => {
                let n = (8.0 + 6.0 * (mass / 500.0).sqrt().min(4.0)) as usize;
                for _ in 0..n {
                    let d = self.rsph();
                    self.emit(Kind::Smoke, pos + d, vel * 0.7 + d * 3.0, 3.0, 0.5, 2.5, Vec3::splat(0.15), 0.6, 1.5, 0.0, 0.0);
                }
                for _ in 0..n {
                    let d = self.rsph();
                    self.emit(Kind::Spark, pos, vel + d * 10.0, 0.6, 0.05, 0.03, Vec3::new(1.0, 0.7, 0.4) * 6.0, 1.0, 0.3, 1.0, 0.0);
                }
            }
        }
    }

    /// A burning fuel fire: flames, embers and a dense, wind-blown smoke column.
    fn fire(&mut self, k: u32, f: &crate::debris::Fire, dt: f32) {
        if f.size <= 0.01 {
            return;
        }
        let r = f.radius;
        let s = f.size;
        let n = self.count(2000 + k, (20.0 + 6.0 * r * r).min(160.0) * s, dt);
        for _ in 0..n {
            let a = self.rnd() * std::f32::consts::TAU;
            let rr = r * self.rnd().sqrt() * 0.8;
            let p = f.pos + Vec3::new(a.cos() * rr, 0.3 + self.rnd() * 0.5, a.sin() * rr);
            let up = 3.0 + 3.0 * self.rnd() + r * 0.3;
            let s0 = (0.5 + 0.25 * r) * (0.6 + 0.6 * self.rnd());
            self.emit(Kind::Fire, p, Vec3::new(0.0, up, 0.0), 0.8 + 0.2 * r.sqrt(), s0, s0 * 1.6, Vec3::new(1.0, 0.42, 0.1) * (2.2 + 1.5 * s), 1.0, 1.2, 0.0, 4.0);
        }
        let n = self.count(3000 + k, (4.0 + 1.2 * r) * s, dt);
        for _ in 0..n {
            let a = self.rnd() * std::f32::consts::TAU;
            let rr = r * self.rnd().sqrt() * 0.6;
            let p = f.pos + Vec3::new(a.cos() * rr, 1.0 + r * 0.6, a.sin() * rr);
            let s0 = 0.5 + 0.35 * r;
            // Buoyant plume: rises fast near the fire, then drifts downwind and spreads.
            self.emit(Kind::Smoke, p, Vec3::new(0.0, 5.0 + 0.6 * r, 0.0), 30.0 + 2.0 * r, s0, 8.0 + 2.6 * r, Vec3::splat(0.03), 0.9, 0.25, 0.0, 3.0 + 0.3 * r);
        }
        let n = self.count(4000 + k, 8.0 * s, dt);
        for _ in 0..n {
            let d = self.rsph();
            self.emit(Kind::Spark, f.pos + d * r * 0.5, Vec3::new(d.x * 2.0, 4.0 + 6.0 * self.rnd(), d.z * 2.0), 2.5, 0.05, 0.03, Vec3::new(1.0, 0.5, 0.15) * 5.0, 1.0, 0.8, 0.15, 0.0);
        }
    }

    /// Effects of the flying (or rolling) aircraft.
    fn flight_effects(&mut self, c: &Ctx, sd: &crate::damage::StructDef, dt: f32, gh: &dyn Fn(Vec3) -> f32) {
        let ac = c.ac;
        let def = c.def;
        let rot = c.rot;
        let pos = c.pos;
        let at = |p: Vec3| pos + rot * p;
        let vel = ac.vel;
        let air = self.air;
        let tas = ac.tas();
        let q = 0.5 * air.rho * tas * tas;
        let n_eng = def.engine_count.min(def.engine_pos.len()).min(4);
        let jet = def.engine_kind == EngineKind::Jet;
        let lost = ac.dmg.lost;
        // ---- tyre smoke at touchdown (wheels spun up from rest by the runway)
        if ac.on_ground && !self.prev_on_ground {
            let gs = Vec2::new(vel.x, vel.z).length();
            if gs > 15.0 {
                let k = ((gs - 15.0) / 40.0).clamp(0.0, 1.5) * (1.0 + (-self.prev_vs).max(0.0) * 0.4);
                let gear: Vec<Vec3> = def.gear.iter().filter(|g| matches!(g.kind, crate::flight::ContactKind::Wheel { .. }) && g.pos.z < 0.5 && !ac.dmg.gear_lost(def, g)).map(|g| g.pos).collect();
                for g in gear {
                    let p = at(g);
                    let p = Vec3::new(p.x, gh(p) + 0.3, p.z);
                    let n = (6.0 + 22.0 * k) as usize;
                    for _ in 0..n {
                        let d = self.rsph();
                        let v = vel * (0.6 + 0.3 * self.rnd()) + d * 1.5 + Vec3::Y * 0.8;
                        let size = (0.4 + 0.4 * self.rnd()) * (c.def.gear_height * 0.5).clamp(0.4, 1.0);
                        self.emit(Kind::Steam, p + d * 0.3, v, 2.2 + 1.5 * self.rnd(), size, size * 6.0, Vec3::new(0.72, 0.72, 0.74), 0.5 * k.min(1.0) + 0.2, 1.6, 0.0, 0.2);
                    }
                }
            }
        }
        // ---- engines: smoke from failed / damaged engines, contrails, haze
        let (forms, c_life, c_spread) = if jet && ac.pos.y > 4000.0 { atmo::contrail(&air) } else { (0.0, 0.0, 0.0) };
        for i in 0..n_eng {
            let running = ac.engine_running(i);
            let gone = lost & crate::damage::Part::engine(i).bit() != 0;
            if !running && self.prev_running[i] && !ac.on_ground && !gone {
                self.eng_smoke[i] = 25.0;
            }
            self.prev_running[i] = running;
            let e = def.engine_pos[i];
            let (_, _, aft) = sd.nacelle;
            let exhaust = if jet { e - Vec3::Z * if e.x.abs() > sd.fus_r { aft * 0.85 } else { 0.4 } } else { e - Vec3::Z * 0.8 };
            if gone {
                continue;
            }
            if self.eng_smoke[i] > 0.0 {
                self.eng_smoke[i] -= dt;
                let n = self.count(100 + i as u32, 30.0 * (self.eng_smoke[i] / 25.0).min(1.0), dt);
                for _ in 0..n {
                    let d = self.rsph();
                    let p = at(exhaust) + d * 0.3;
                    self.emit(Kind::Smoke, p, vel * 0.85 + d, 6.0, 0.4, 3.5, Vec3::splat(0.18), 0.55, 0.5, 0.0, 0.0);
                }
            }
            if !running {
                continue;
            }
            let thrust_k = (ac.engine_thrust(i) / (def.static_thrust / n_eng as f32).max(1.0)).clamp(0.0, 1.5);
            if forms > 0.0 {
                // The plume needs ~0.3 s (a couple of wingspans) to cool and saturate.
                let p = at(exhaust);
                self.trail(200 + i as u32, mk::TRAIL, p, 1.4 + 0.6 * thrust_k, 0.85 * forms, c_life, c_spread, 1.2, 0.35, 25.0);
            }
            // Heat haze behind the jets.
            let _ = thrust_k;
        }
        // ---- condensation in low-pressure regions (humid air only)
        let ld = ac.alpha().clamp(-0.2, 0.5);
        let cl = (ac.g_load() * def.mass * GRAVITY / (q * def.wing_area).max(1.0)).clamp(-1.5, 3.5);
        let _ = ld;
        let semi = def.wingspan * 0.5;
        let airborne = !ac.on_ground && tas > 25.0;
        // Wingtip vortices: core velocity from the circulation; flap tips are tighter.
        if airborne {
            let lift = (ac.g_load() * def.mass * GRAVITY).abs();
            let gamma = lift / (air.rho * tas * (std::f32::consts::FRAC_PI_4 * def.wingspan)).max(1.0);
            let tip_pts: Vec<Vec3> = def.hard_points.iter().filter(|h| h.reason == "Wingtip strike").map(|h| h.pos).collect();
            for (k, tp) in tip_pts.iter().enumerate() {
                let side = if tp.x > 0.0 { 0 } else { 1 };
                if lost & [crate::damage::Part::WingL, crate::damage::Part::WingR][side].bit() != 0 {
                    continue;
                }
                let rc = 0.04 * def.wingspan;
                let vc = gamma / (std::f32::consts::TAU * rc);
                let dp = air.rho * vc * vc;
                let amt = atmo::condensation(&air, atmo::cooling(&air, dp));
                if amt > 0.02 {
                    let p = at(*tp + Vec3::Y * c.tips[side] - Vec3::Z * 0.3);
                    self.trail(300 + k as u32, mk::TRAIL, p, 0.18 + 0.006 * def.wingspan, amt.min(1.0) * 0.8, 1.6, 0.4, 0.0, 0.05, 6.0);
                }
            }
            // Flap tips (outer flap edge, ~60 % span) with flaps out.
            let flaps = ac.surfaces().flaps;
            if flaps > 0.25 && def.has_engine && !c.planform.is_empty() {
                let rc = 0.015 * def.wingspan;
                let g2 = gamma * 0.35 * flaps;
                let vc = g2 / (std::f32::consts::TAU * rc);
                let amt = atmo::condensation(&air, atmo::cooling(&air, air.rho * vc * vc));
                if amt > 0.02 {
                    let x = sd.fus_r + 0.62 * (semi - sd.fus_r);
                    let te = planform_at(c.planform, x).2;
                    let y = planform_at(c.planform, x).3;
                    for (side, sx) in [(0usize, 1.0f32), (1, -1.0)] {
                        let w = bend(0.62) * c.tips[side];
                        let p = at(Vec3::new(sx * x, y + w, te - 0.3));
                        self.trail(310 + side as u32, mk::TRAIL, p, 0.12 + 0.004 * def.wingspan, amt.min(1.0) * 0.7, 1.2, 0.3, 0.0, 0.05, 6.0);
                    }
                }
            }
            // Over the wing: the suction peak at high lift.
            let cp_min = 0.5 + 2.4 * cl.max(0.0);
            self.wing_vapour = atmo::condensation(&air, atmo::cooling(&air, q * cp_min)) * smoothstep(0.5, 1.2, cl);
            // Vapour cone: the local supersonic pocket around Mach 1 (Prandtl–Glauert).
            let m = ac.mach();
            let bell = smoothstep(0.88, 0.97, m) * (1.0 - smoothstep(1.03, 1.12, m));
            let amt = atmo::condensation(&air, atmo::cooling(&air, q * 0.9 * bell));
            self.cone += (amt * bell - self.cone) * (dt / 0.25).min(1.0);
        } else {
            self.wing_vapour = 0.0;
            self.cone = 0.0;
        }
        let _ = gh;
    }

    /// Build the GPU data for this frame (relative to `origin` = the eye).
    pub fn frame(&self, c: &FrameCtx) -> VfxFrame {
        let origin = c.eye;
        let mut f = VfxFrame { origin, ..Default::default() };
        // ---- particles, back to front
        let mut idx: Vec<(f32, usize)> = self.parts.iter().enumerate().map(|(i, p)| ((p.pos - origin).length_squared(), i)).collect();
        idx.sort_unstable_by(|a, b| b.0.total_cmp(&a.0));
        f.particles.reserve(idx.len());
        for (_, i) in idx {
            let p = &self.parts[i];
            let t = (p.age / p.life).clamp(0.0, 1.0);
            // Size grows fast at first (turbulent mixing), then slowly.
            let size = p.size0 + (p.size1 - p.size0) * (1.0 - (1.0 - t).powi(3));
            let fade = match p.kind {
                Kind::Fire => smoothstep(0.0, 0.08, t) * (1.0 - smoothstep(0.35, 1.0, t)),
                Kind::Spark => 1.0 - t * t,
                _ => smoothstep(0.0, 0.06, t) * (1.0 - smoothstep(0.55, 1.0, t)),
            };
            let rel = p.pos - origin;
            f.particles.push(GpuParticle {
                pos: [rel.x, rel.y, rel.z, size],
                col: [p.col.x, p.col.y, p.col.z, p.alpha * fade],
                vel: [p.vel.x, p.vel.y, p.vel.z, p.kind as u8 as f32],
                extra: [t, p.rot + t * 0.6, p.seed, 1.0],
            });
        }
        // ---- ribbons
        for t in &self.trails {
            let mut pts: Vec<TrailPt> = t.pts.iter().copied().collect();
            if let Some(h) = t.head {
                pts.push(h);
            }
            if pts.len() < 2 {
                continue;
            }
            let base = f.mesh.len() as u32;
            let n = pts.len();
            for (k, q) in pts.iter().enumerate() {
                let age = (self.time - q.born).max(0.0);
                let width = q.width + t.growth * age;
                let prev = pts[k.saturating_sub(1)].pos;
                let next = pts[(k + 1).min(n - 1)].pos;
                let tan = (next - prev).normalize_or(Vec3::Z);
                let view = (q.pos - origin).normalize_or(Vec3::Z);
                let side = tan.cross(view).normalize_or(Vec3::X) * (0.5 * width);
                let life_t = (age / t.life).clamp(0.0, 1.0);
                let a = q.alpha * smoothstep(0.0, t.fade_in.max(1e-3), age) * (1.0 - smoothstep(0.6, 1.0, life_t)) * (q.width / width).sqrt().max(0.15);
                let rel = q.pos - origin;
                for s in [-1.0f32, 1.0] {
                    let p = rel + side * s;
                    f.mesh.push(VfxVert { pos: [p.x, p.y, p.z, t.kind], uv: [age * 0.15 + (q.born * 0.37).fract() * 0.0, s, a, width] });
                }
            }
            for k in 0..(n as u32 - 1) {
                let i = base + k * 2;
                f.mesh_idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
            }
        }
        // ---- surface / transonic vapour and afterburner diamonds
        self.surface_effects(c, &mut f);
        // ---- fire lights (strongest four)
        if let Some(fires) = c.fires {
            let mut v: Vec<&crate::debris::Fire> = fires.iter().filter(|f| f.size > 0.02).collect();
            v.sort_by(|a, b| (b.size * b.radius).total_cmp(&(a.size * a.radius)));
            for (k, fi) in v.iter().take(4).enumerate() {
                let rel = fi.pos - origin + Vec3::Y * (1.0 + fi.radius * 0.5);
                let flick = 0.85 + 0.15 * ((self.time * 13.0 + k as f32).sin() * (self.time * 7.3).cos());
                f.fires[k] = [rel.x, rel.y, rel.z, 6.0e3 * fi.size * fi.radius * fi.radius * flick];
            }
        }
        f
    }

    fn surface_effects(&self, c: &FrameCtx, f: &mut VfxFrame) {
        let origin = c.eye;
        let at = |p: Vec3| c.pos + c.rot * p - origin;
        let def = c.def;
        let sd = crate::damage::struct_def(def.id);
        let semi = def.wingspan * 0.5;
        // Wing-surface vapour: a sheet hugging the upper surface, from the
        // root to ~85 % span over the forward 60 % of the chord.
        if self.wing_vapour > 0.01 && !c.planform.is_empty() {
            for (side, sx) in [(0usize, 1.0f32), (1, -1.0)] {
                if c.lost & [crate::damage::Part::WingL, crate::damage::Part::WingR][side].bit() != 0 {
                    continue;
                }
                let base = f.mesh.len() as u32;
                let ns = 10;
                for i in 0..=ns {
                    let s = 0.05 + 0.8 * i as f32 / ns as f32;
                    let x = sd.fus_r + s * (semi - sd.fus_r);
                    let (_, le, te, y) = planform_at(c.planform, x);
                    let w = bend(s) * c.tips[side];
                    let chord = (le - te).max(0.3);
                    for (j, frac) in [0.02f32, 0.6].iter().enumerate() {
                        let z = le - frac * chord;
                        let h = 0.06 * chord * (1.0 - frac);
                        let p = at(Vec3::new(sx * x, y + w + 0.05 + h, z));
                        f.mesh.push(VfxVert { pos: [p.x, p.y, p.z, mk::VAPOUR], uv: [s * 3.0, j as f32 * 2.0 - 1.0, self.wing_vapour * (1.0 - s * 0.7), 0.0] });
                    }
                }
                for i in 0..ns as u32 {
                    let k = base + i * 2;
                    f.mesh_idx.extend([k, k + 1, k + 2, k + 1, k + 3, k + 2]);
                }
            }
        }
        // Vapour cone: a bell-shaped shell opening aft from the wing's leading
        // region, ending sharply at the shock near the trailing edge.
        if self.cone > 0.01 {
            let chord = sd.wing_z.1 - sd.wing_z.0;
            let (z0, z1) = (sd.wing_z.1 + 0.9 * chord, sd.wing_z.0 - 0.05 * chord);
            let r1 = 0.6 * semi.max(3.0) + 0.5;
            let rings = 10;
            let segs = 32;
            let base = f.mesh.len() as u32;
            for i in 0..=rings {
                let t = i as f32 / rings as f32;
                let z = z0 + (z1 - z0) * t;
                // A cone flaring aft (the expansion region), cut off by the shock.
                let r = r1 * (0.12 + 0.88 * t.powf(0.85));
                for j in 0..=segs {
                    let a = j as f32 / segs as f32 * std::f32::consts::TAU;
                    let p = at(Vec3::new(a.cos() * r, a.sin() * r * 0.92 + def.wing_height * 0.5, z));
                    f.mesh.push(VfxVert { pos: [p.x, p.y, p.z, mk::VAPOUR], uv: [j as f32 / segs as f32 * 6.0, (t * 1.6 - 1.0).min(0.0), self.cone * smoothstep(0.15, 0.9, t) * (1.0 - smoothstep(0.97, 1.0, t) * 0.6), 1.0] });
                }
            }
            let w = segs as u32 + 1;
            for i in 0..rings as u32 {
                for j in 0..segs as u32 {
                    let a = base + i * w + j;
                    f.mesh_idx.extend([a, a + w, a + 1, a + 1, a + w, a + w + 1]);
                }
            }
        }
        // Engines: afterburner shock diamonds, heat haze.
        let n_eng = def.engine_count.min(def.engine_pos.len()).min(4);
        let jet = def.engine_kind == EngineKind::Jet;
        for i in 0..n_eng {
            if !c.running[i] || !jet {
                continue;
            }
            let e = def.engine_pos[i];
            let (nr, _, aft) = sd.nacelle;
            let wing = e.x.abs() > sd.fus_r;
            let nozzle = if wing { e - Vec3::Z * aft * 0.85 } else { e - Vec3::Z * 0.3 };
            let r = if wing { nr * 0.45 } else { 0.42 };
            let ab = c.afterburner;
            let thrust = c.thrust[i];
            if ab > 0.05 && def.has_afterburner {
                // Diamonds: under-expanded jet, cell length ≈ 1.2 D √(Mj² − 1), Mj ≈ 1.5–1.8.
                let len = 3.5 + 2.5 * ab;
                beam(f, at(nozzle), c.rot * -Vec3::Z, len, r * 0.9, origin, c.pos, mk::DIAMONDS, ab, 0.0, false);
            }
            if thrust > 0.05 {
                let len = (6.0 + 22.0 * thrust + 18.0 * ab) * (r / 0.5).clamp(0.6, 2.0);
                beam(f, at(nozzle), c.rot * -Vec3::Z, len, r * 1.6, origin, c.pos, mk::HAZE, (0.25 + 0.75 * thrust + ab).min(1.5), 0.0, true);
            }
        }
        // Heat haze over fires.
        if let Some(fires) = c.fires {
            for fi in fires.iter().filter(|f| f.size > 0.05).take(4) {
                let base = fi.pos - origin + Vec3::Y * fi.radius * 0.5;
                beam(f, base, Vec3::Y, 6.0 + fi.radius * 3.0, fi.radius * 1.2, origin, c.pos, mk::HAZE, fi.size, 0.0, true);
            }
        }
    }
}

/// A camera-facing beam (quad strip along `dir` from `start`, eye-relative).
#[allow(clippy::too_many_arguments)]
fn beam(f: &mut VfxFrame, start: Vec3, dir: Vec3, len: f32, radius: f32, _origin: Vec3, _pos: Vec3, kind: f32, intensity: f32, extra: f32, haze: bool) {
    let dir = dir.normalize_or(Vec3::Z);
    let n = 8;
    let (verts, idx) = if haze { (&mut f.haze, &mut f.haze_idx) } else { (&mut f.mesh, &mut f.mesh_idx) };
    let base = verts.len() as u32;
    for k in 0..=n {
        let t = k as f32 / n as f32;
        let p = start + dir * (len * t);
        let view = p.normalize_or(Vec3::Z);
        let side = dir.cross(view).normalize_or(Vec3::X);
        // Widens downstream (jet spreading / rising plume).
        let r = radius * (1.0 + 1.2 * t);
        for s in [-1.0f32, 1.0] {
            let q = p + side * r * s;
            verts.push(VfxVert { pos: [q.x, q.y, q.z, kind], uv: [t * len, s, intensity, if extra != 0.0 { extra } else { len }] });
        }
    }
    for k in 0..n as u32 {
        let i = base + k * 2;
        idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
    }
}

/// Uniformly loaded cantilever shape (== the aircraft shader).
fn bend(s: f32) -> f32 {
    s * s * (6.0 - 4.0 * s + s * s) / 3.0
}

/// Wing planform at |x|: (x, leading edge z, trailing edge z, y).
fn planform_at(pf: &[(f32, f32, f32, f32)], x: f32) -> (f32, f32, f32, f32) {
    let x = x.abs();
    if pf.is_empty() {
        return (x, 1.0, -1.0, 0.0);
    }
    let mut best = pf[0];
    for w in pf.windows(2) {
        if x >= w[0].0 && x <= w[1].0 {
            let t = (x - w[0].0) / (w[1].0 - w[0].0).max(1e-3);
            return (x, w[0].1 + (w[1].1 - w[0].1) * t, w[0].2 + (w[1].2 - w[0].2) * t, w[0].3 + (w[1].3 - w[0].3) * t);
        }
        if (w[1].0 - x).abs() < (best.0 - x).abs() {
            best = w[1];
        }
    }
    best
}

/// Inputs for [`Vfx::frame`].
pub struct FrameCtx<'a> {
    pub def: &'a AircraftDef,
    pub eye: Vec3,
    pub pos: Vec3,
    pub rot: Quat,
    pub tips: [f32; 2],
    pub lost: u16,
    pub running: [bool; 4],
    /// Thrust per engine 0..1+ of max.
    pub thrust: [f32; 4],
    pub afterburner: f32,
    pub planform: &'a [(f32, f32, f32, f32)],
    pub fires: Option<&'a [crate::debris::Fire]>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn particles_age_out_and_stay_finite() {
        let mut v = Vfx::default();
        for k in 0..2000 {
            let p = Vec3::new(k as f32, 10.0, 0.0);
            v.emit(Kind::Smoke, p, Vec3::Y, 2.0, 1.0, 3.0, Vec3::splat(0.1), 0.5, 0.5, 0.0, 1.0);
            v.emit(Kind::Spark, p, Vec3::new(3.0, 5.0, 0.0), 1.0, 0.05, 0.03, Vec3::ONE, 1.0, 0.3, 1.0, 0.0);
        }
        assert_eq!(v.particle_count(), 4000);
        // Ageing without a world: drive the integrator directly.
        for _ in 0..200 {
            let dt = 1.0 / 60.0;
            let mut i = 0;
            while i < v.parts.len() {
                let p = &mut v.parts[i];
                p.age += dt;
                if p.age >= p.life {
                    v.parts.swap_remove(i);
                    continue;
                }
                p.vel.y += (p.buoy - GRAVITY * p.grav) * dt;
                p.pos += p.vel * dt;
                assert!(p.pos.is_finite() && p.vel.is_finite());
                i += 1;
            }
        }
        assert_eq!(v.particle_count(), 0);
    }

    #[test]
    fn frame_is_sorted_back_to_front() {
        let mut v = Vfx::default();
        for k in 0..50 {
            let p = Vec3::new(0.0, 0.0, (k * 37 % 50) as f32 * 10.0);
            v.emit(Kind::Smoke, p, Vec3::ZERO, 5.0, 1.0, 2.0, Vec3::splat(0.1), 0.5, 0.5, 0.0, 0.0);
        }
        let def = crate::flight::find("lancer").unwrap();
        let ctx = FrameCtx { def: &def, eye: Vec3::ZERO, pos: Vec3::ZERO, rot: Quat::IDENTITY, tips: [0.0; 2], lost: 0, running: [true; 4], thrust: [1.0; 4], afterburner: 1.0, planform: &[], fires: None };
        let f = v.frame(&ctx);
        let d: Vec<f32> = f.particles.iter().map(|p| Vec3::new(p.pos[0], p.pos[1], p.pos[2]).length()).collect();
        assert!(d.windows(2).all(|w| w[0] >= w[1]));
        // Afterburner: diamonds in the mesh, haze in the haze list.
        assert!(f.mesh.iter().any(|m| m.pos[3] == mk::DIAMONDS));
        assert!(f.haze.iter().any(|m| m.pos[3] == mk::HAZE));
        assert!(f.mesh.iter().chain(f.haze.iter()).all(|m| m.pos.iter().all(|x| x.is_finite())));
    }
}
