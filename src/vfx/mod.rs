//! Visual effects (Tracks L, V): a CPU-simulated particle system drawn as GPU
//! instanced, lit, soft particles (fire, smoke, dust, spray, sparks, tyre
//! smoke, condensation), camera-facing ribbons (contrails, vortex cores,
//! smoke trails), ray-marched condensation volumes in the aircraft's frame
//! (vapour cone, wing-surface fog), afterburner shock diamonds and
//! screen-space heat haze.
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
mod rotorwash;
mod smoke;
mod spray;

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
    /// Flames: temperature at birth (K) and cooling time constant (s; 0 =
    /// a flame tongue, cooling over its life). 0 for other kinds.
    temp0: f32,
    cool: f32,
    /// Velocity of the source at birth (a burning, moving wreck): the flame
    /// axis is the motion relative to it.
    src_vel: Vec3,
    /// Flame elongation (length / width).
    stretch: f32,
    /// Fire plume: buoyancy flux F (m⁴/s³) and source height (0 = no plume).
    flux: f32,
    y0: f32,
}

/// One particle on the GPU (80 bytes; == `vfx.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuParticle {
    /// Position relative to the frame origin, radius (m).
    pub pos: [f32; 4],
    /// Albedo / emission, opacity.
    pub col: [f32; 4],
    /// Velocity (m/s, for stretched kinds), kind.
    pub vel: [f32; 4],
    /// Age 0..1, rotation, seed, stretch (flames).
    pub extra: [f32; 4],
    /// Key-light transmittance, fire irradiance (lux), temperature (K), sky
    /// visibility (the first, second and fourth filled in by `gpu.rs`).
    pub light: [f32; 4],
}

/// A ribbon / mesh vertex (48 bytes; == `vfx.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VfxVert {
    /// Position relative to the frame origin (ribbons: the centre line), kind.
    pub pos: [f32; 4],
    /// u (along), v (across −1..1), intensity, width (m).
    pub uv: [f32; 4],
    /// Ribbons: unit side vector, age (s).
    /// Diamonds: -, -, nozzle radius, cell length.
    pub aux: [f32; 4],
}

/// Mesh / ribbon kinds (== `vfx.wgsl`).
pub mod mk {
    /// Condensation trail (contrail, vortex core): lit white, soft tube.
    pub const TRAIL: f32 = 10.0;
    /// Smoke trail: lit grey.
    pub const SMOKE: f32 = 11.0;
    /// Afterburner core with shock diamonds: emissive.
    pub const DIAMONDS: f32 = 13.0;
    /// Heat haze (distortion pass only).
    pub const HAZE: f32 = 14.0;
    /// Vortex-core condensation: a lit white tube with a helical twist.
    pub const VORTEX: f32 = 15.0;
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
    /// Volumetric condensation in aircraft space (== `vfx.wgsl` `VOL_*`):
    /// the transonic (Prandtl–Glauert) bells and the fog over the wings,
    /// ray-marched inside up to three boxes (bits of `vol_boxes`).
    pub vol: [[f32; 4]; VOL_N],
    pub vol_boxes: u32,
}

/// Size of [`VfxFrame::vol`] (vec4s).
pub const VOL_N: usize = 20;

impl VfxFrame {
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty() && self.mesh.is_empty() && self.haze.is_empty() && self.vol_boxes == 0
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

/// Heights (m above the reference ground) of the wind profile samples.
const WIND_H: [f32; 10] = [0.0, 10.0, 30.0, 70.0, 150.0, 300.0, 600.0, 1000.0, 1800.0, 3000.0];

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
    cone_mach: f32,
    cone_aoa: f32,
    wing_vapour: f32,
    air: atmo::Air,
    wind: Vec3,
    /// Wind profile over the effects (reference ground height, samples at `WIND_H`).
    wind_ref: f32,
    winds: [Vec3; 10],
    /// Optional: humidity / temperature from ISA (tests, no weather).
    pub isa_only: bool,
    /// Accumulators for rate-based emitters.
    acc: Vec<(u32, f32)>,
    /// Airshow smoke on (aerobats with a smoke system, src/vfx/smoke.rs).
    pub smoke: bool,
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
            cone_mach: 0.0,
            cone_aoa: 0.0,
            wing_vapour: 0.0,
            air: atmo::Air::default(),
            wind: Vec3::ZERO,
            wind_ref: 0.0,
            winds: [Vec3::ZERO; 10],
            isa_only: false,
            acc: Vec::new(),
            smoke: false,
        }
    }
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A pool / wreck fire's scale: diameter (m), heat release (kW) and mean
/// flame height (m, Heskestad: L = 0.235 Q^⅖ − 1.02 D).
pub fn flame(f: &crate::debris::Fire) -> (f32, f32, f32) {
    let d = 2.0 * f.radius.max(0.3);
    // Hydrocarbon fuel: ≈ 43 MJ/kg.
    let q = (f.rate * 43_000.0 * f.size).max(1.0);
    let l = (0.235 * q.powf(0.4) - 1.02 * d).max(0.5 * d).max(0.6) * f.size.sqrt().max(0.2);
    (d, q, l)
}

/// Luminous intensity (cd) of a fire: mean flame luminance × projected area.
fn fire_cd(f: &crate::debris::Fire) -> f32 {
    let (d, _, l) = flame(f);
    // (≈ 1.2 kcd/m² over the flame's silhouette: its sooty mantle hides much of the hot core.)
    1200.0 * 0.6 * d * l * f.size
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
            p.y0 += delta.y;
        }
        self.wind_ref += delta.y;
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

    /// Wind at height `y` (from the profile sampled this frame).
    fn wind_at(&self, y: f32) -> Vec3 {
        let h = y - self.wind_ref;
        if h <= WIND_H[0] {
            return self.winds[0];
        }
        for k in 1..WIND_H.len() {
            if h < WIND_H[k] {
                let t = (h - WIND_H[k - 1]) / (WIND_H[k] - WIND_H[k - 1]);
                return self.winds[k - 1].lerp(self.winds[k], t);
            }
        }
        self.winds[WIND_H.len() - 1]
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
    fn emit(&mut self, kind: Kind, pos: Vec3, vel: Vec3, life: f32, size0: f32, size1: f32, col: Vec3, alpha: f32, drag: f32, grav: f32, buoy: f32) -> Option<&mut Particle> {
        if self.parts.len() >= MAX_PARTICLES {
            // Full: the oldest smoke makes room for fresh fire / sparks.
            if matches!(kind, Kind::Smoke | Kind::Dust | Kind::Steam) {
                return None;
            }
            let i = self.parts.iter().position(|p| p.kind == Kind::Smoke && p.age > 0.5 * p.life)?;
            self.parts.swap_remove(i);
        }
        let rot = self.rnd() * std::f32::consts::TAU;
        let seed = self.rnd() * 97.0;
        let life = life * (0.75 + 0.5 * self.rnd());
        self.parts.push(Particle { pos, vel, age: 0.0, life, size0, size1, kind, col, alpha, drag, grav, buoy, rot, seed, temp0: 0.0, cool: 0.0, src_vel: Vec3::ZERO, stretch: 1.0, flux: 0.0, y0: pos.y });
        self.parts.last_mut()
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
        // Air at the aircraft; the wind as a profile over the effects (smoke
        // columns rise into the faster, veered wind aloft).
        self.air = atmo::air_at(c.world, c.pos, self.isa_only);
        let w = crate::flight::Environment::wind(c.world, c.pos);
        self.wind = if w.is_finite() { w } else { Vec3::ZERO };
        let gh = |p: Vec3| crate::flight::Environment::ground_height(c.world, Vec2::new(p.x, p.z));
        let reference = c.debris.as_deref().and_then(|d| d.crash_at).unwrap_or(c.pos);
        self.wind_ref = gh(reference);
        for (k, h) in WIND_H.iter().enumerate() {
            let w = crate::flight::Environment::wind(c.world, Vec3::new(reference.x, self.wind_ref + h.max(2.0), reference.z));
            self.winds[k] = if w.is_finite() { w } else { Vec3::ZERO };
        }
        for t in &mut self.trails {
            t.head = None;
        }

        // ---- particles
        let mut i = 0;
        while i < self.parts.len() {
            let wind = self.wind_at(self.parts[i].pos.y);
            let p = &mut self.parts[i];
            p.age += dt;
            if p.age >= p.life {
                self.parts.swap_remove(i);
                continue;
            }
            if p.age < 0.0 {
                // Not born yet (delayed smoke): carried along.
                p.vel += (wind - p.vel) * (p.drag * dt).min(1.0);
                p.pos += p.vel * dt;
                i += 1;
                continue;
            }
            let k = (p.drag * dt).min(1.0);
            if p.flux > 0.0 {
                // Buoyant fire plume (Morton–Taylor–Turner): rise speed
                // ∝ F^⅓ z^-⅓ up to the equilibrium height in a stable
                // atmosphere; spreading b ≈ 0.12 z; bent over by the wind.
                let z = (p.pos.y - p.y0).max(1.0);
                let zmax = 3.8 * p.flux.powf(0.25) * 0.012f32.powf(-0.75);
                let w_up = 1.7 * p.flux.cbrt() * (z + 8.0).powf(-1.0 / 3.0) * (1.0 - smoothstep(0.6 * zmax, zmax, z));
                // Large eddies: each puff wanders across the column (its own
                // slowly turning direction), so the outline billows.
                let ph = p.seed + p.age * 0.35;
                let eddy = Vec3::new(ph.cos(), 0.0, (ph * 1.3).sin()) * (0.35 * p.size1 / (p.age + 4.0)).min(3.0);
                p.vel.x += (wind.x + eddy.x - p.vel.x) * k;
                p.vel.z += (wind.z + eddy.z - p.vel.z) * k;
                p.vel.y += (w_up + wind.y - p.vel.y) * (1.5 * dt).min(1.0);
                // Spreads with height, and fast once it stops rising (it flattens out).
                let b = 0.13 * z + p.size0 + 0.25 * smoothstep(0.7 * zmax, zmax, z) * (z - 0.7 * zmax).max(0.0);
                p.size1 = p.size1.max(1.5 * b);
            } else {
                // Flames on a moving wreck ride in its wake (the air there
                // moves with it), then fall behind into the free stream.
                let target = if p.kind == Kind::Fire && p.cool == 0.0 { wind + (p.src_vel - wind) * (-p.age / 0.35).exp() } else { wind };
                p.vel += (target - p.vel) * k;
                let lift = p.buoy * (1.0 - p.age / p.life).max(0.0);
                p.vel.y += (lift - GRAVITY * p.grav) * dt;
            }
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
                q.pos += (self.wind + Vec3::new(0.0, -t.sink * (-age / 40.0).exp(), 0.0)) * dt;
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
            let fires: Vec<(crate::debris::Fire, Vec3)> = d.fires.iter().map(|f| (*f, f.body.and_then(|b| d.bodies.get(b)).map_or(Vec3::ZERO, |b| b.vel))).collect();
            for (k, (f, v)) in fires.iter().enumerate() {
                self.fire(k as u32, f, *v, dt);
            }
            let bodies: Vec<(Vec3, Vec3, f32, bool, bool, usize)> = d.bodies.iter().enumerate().map(|(i, b)| (b.pos, b.vel, b.half.max_element(), b.smoking && !b.in_water, b.sleeping, i)).collect();
            for (pos, vel, r, smoking, sleeping, i) in bodies {
                if smoking && d.time < 90.0 {
                    // Torn, hot pieces trail grey-brown smoke while they move, a thin wisp once still.
                    let n = self.count(1000 + i as u32, if sleeping { 1.2 } else { 6.0f32.max(vel.length() / (0.5 + 0.2 * r)) }, dt);
                    for _ in 0..n {
                        let o = self.rsph() * r * 0.5 - vel * dt * self.rnd();
                        let v = vel * 0.3 + Vec3::new(0.0, 1.5, 0.0);
                        let g = 0.09 + 0.05 * self.rnd();
                        if let Some(p) = self.emit(Kind::Smoke, pos + o, v, 10.0, 0.5 + 0.2 * r, 4.0 + 1.5 * r, Vec3::new(g, g * 0.95, g * 0.9), 0.45, 0.8, 0.0, 1.2) {
                            p.src_vel = vel;
                        }
                    }
                }
            }
        }

        if ac.crashed.is_none() {
            self.flight_effects(&c, sd, dt, &gh);
            self.rotor_wash(&c, dt, &gh);
            self.float_spray(&c, dt, &gh);
            self.airshow_smoke(&c, dt);
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
                // Fireball: a few big, very hot, expanding puffs that decelerate,
                // rise and roll up, cooling from yellow-white through orange
                // into black soot; a dark cloud follows them up.
                let n = (18.0 + 26.0 * energy) as usize;
                let tau = 0.7 + 0.9 * energy;
                for _ in 0..n {
                    let d = self.rsph();
                    let v = vel * 0.35 + d * radius * (1.6 + 1.2 * self.rnd()) + Vec3::Y * radius * (0.5 + 0.4 * self.rnd());
                    let s0 = radius * (0.22 + 0.12 * self.rnd());
                    let t0 = 1800.0 + 180.0 * self.rnd();
                    let life = 6.0 + 4.0 * energy;
                    if let Some(p) = self.emit(Kind::Fire, pos + d * radius * 0.25, v, life, s0, radius * (0.55 + 0.25 * self.rnd()), Vec3::splat(0.035), 1.0, 2.2, 0.0, 5.0 + 4.0 * energy) {
                        p.temp0 = t0;
                        p.cool = tau * (0.7 + 0.6 * p.seed / 97.0);
                        p.stretch = 1.0;
                    }
                }
                // The smoke cloud left behind (born as the fireball cools).
                let flux = 4.0e4 * energy;
                for _ in 0..(n + 10) {
                    let d = self.rsph();
                    let v = vel * 0.3 + d * radius * 1.4 + Vec3::Y * radius * 0.7;
                    let s0 = radius * 0.5;
                    let g = 0.06 + 0.04 * self.rnd();
                    if let Some(p) = self.emit(Kind::Smoke, pos + d * radius * 0.4, v, 35.0 + 20.0 * energy, s0, s0 * 1.5, Vec3::new(g, g * 0.93, g * 0.86), 0.55, 0.6, 0.0, 0.0) {
                        p.age = -(tau * 1.6 + 0.6 * p.seed / 97.0);
                        p.flux = flux;
                        p.y0 = gh(pos);
                    }
                }
                for _ in 0..(n * 2) {
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
                        // Clods thrown out ballistically, then a low, spreading dust cloud.
                        for _ in 0..n {
                            let d = self.rsph();
                            let v = vel * 0.35 + Vec3::new(d.x * 5.0, 3.0 + 6.0 * self.rnd(), d.z * 5.0);
                            self.emit(Kind::Dust, pos + d * 0.5, v, 2.0 + 2.0 * self.rnd(), 0.12, 0.2, Vec3::new(0.2, 0.15, 0.1), 0.95, 0.3, 1.0, 0.0);
                        }
                        for _ in 0..n {
                            let d = self.rsph();
                            self.emit(Kind::Dust, pos + d, vel * 0.25 + Vec3::Y * (0.6 + 1.5 * self.rnd()), 6.0 + 4.0 * e.min(1.0), 1.0, 4.0 + 5.0 * e.min(1.5), Vec3::new(0.36, 0.3, 0.22), 0.5, 0.9, 0.0, 0.15);
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
                            self.emit(Kind::Dust, pos + d * 0.8, vel * 0.3 + Vec3::Y * (0.4 + 1.5 * self.rnd()), 5.0 + 3.0 * self.rnd(), 0.8, 3.5 + 3.0 * k.min(1.0), Vec3::new(0.34, 0.28, 0.2), 0.5, 0.9, 0.0, 0.1);
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

    /// A burning fuel fire: layered flame tongues sized by the heat release
    /// (Heskestad flame height, puffing at f ≈ 1.5 / √D Hz), embers and a
    /// buoyant, wind-bent smoke column. A fire on a moving wreck streams
    /// behind it (the flames start with the wreck's velocity).
    fn fire(&mut self, k: u32, f: &crate::debris::Fire, src_vel: Vec3, dt: f32) {
        if f.size <= 0.01 {
            return;
        }
        let (dia, q, l) = flame(f);
        let r = 0.5 * dia;
        let moving = src_vel.length();
        // Puffing: large eddies shed at ≈ 1.5 / √D Hz swell the flames.
        let fp = 1.5 / dia.sqrt();
        let puff = (std::f32::consts::TAU * fp * self.time + k as f32 * 2.1).sin();
        // Flame tongues: rise to the flame height in their lifetime.
        let rise = (0.55 * (GRAVITY * l).sqrt()).max(2.0);
        // Fast through the air the flame is shortened (blown off downstream).
        let life = (l / rise).clamp(0.5, 4.0) / (1.0 + moving / 12.0);
        let alive = (6.0 + 1.2 * dia).min(48.0);
        let n = self.count(2000 + k, alive / life * (1.0 + 0.45 * puff), dt);
        for _ in 0..n {
            let a = self.rnd() * std::f32::consts::TAU;
            let rr = r * self.rnd().sqrt() * 0.75;
            let off = Vec3::new(a.cos() * rr, 0.0, a.sin() * rr);
            // Spread along this frame's path (no gaps behind a moving wreck).
            let p = f.pos + off * if moving > 2.0 { 0.5 } else { 1.0 } + Vec3::Y * (0.2 + 0.1 * l * self.rnd()) - src_vel * dt * self.rnd();
            // Necking: the flame narrows as it rises (air entrained at the base).
            let v = src_vel + Vec3::new(-off.x, 0.0, -off.z) * (0.5 / life) + Vec3::Y * rise * (0.8 + 0.4 * self.rnd());
            let s0 = (0.13 * l).min(0.42 * dia).max(0.35) * (0.8 + 0.4 * self.rnd()) * (1.0 + 0.15 * puff);
            let t0 = 1450.0 + 170.0 * self.rnd() + 50.0 * puff;
            let drag = if moving > 2.0 { 6.0 } else { 1.0 };
            if let Some(pt) = self.emit(Kind::Fire, p, v, life, s0, s0 * 1.35, Vec3::splat(0.04), 1.0, drag, 0.0, 1.5) {
                pt.temp0 = t0;
                pt.src_vel = src_vel;
                pt.stretch = 1.5 + 0.7 * pt.seed / 97.0;
            }
        }
        // Smoke column from the flame tips (pool fires: dense black soot).
        let flux = 0.7 * q * 1000.0 * GRAVITY / (1.2 * 1005.0 * 293.0);
        let s0 = (0.35 * dia).max(0.8);
        // Moving: puffs close enough to overlap along the path, each thinner
        // (the same smoke spread over more track).
        let base_rate = (1.2 + 0.16 * dia) * f.size.sqrt();
        let rate = base_rate.max(moving / s0);
        let thin = (base_rate / rate).sqrt().max(0.2);
        let n = self.count(3000 + k, rate, dt);
        for _ in 0..n {
            let a = self.rnd() * std::f32::consts::TAU;
            let rr = r * self.rnd().sqrt() * 0.35;
            let p = f.pos + Vec3::new(a.cos() * rr, 0.6 * l, a.sin() * rr) - src_vel * dt * self.rnd();
            let g = 0.07 + 0.04 * self.rnd();
            let life = (40.0 + 2.5 * dia).min(110.0);
            if let Some(pt) = self.emit(Kind::Smoke, p, src_vel * 0.5 + Vec3::Y * rise, life, s0, s0 * 1.5, Vec3::new(g, g * 0.92, g * 0.85), 0.92 * thin, 0.35, 0.0, 0.0) {
                pt.flux = flux;
                pt.y0 = f.pos.y + 0.5 * l;
                pt.src_vel = src_vel;
            }
        }
        // Embers lofted by the plume.
        let n = self.count(4000 + k, (3.0 + 0.4 * dia) * f.size, dt);
        for _ in 0..n {
            let d = self.rsph();
            let v = src_vel * 0.5 + Vec3::new(d.x * 2.0, rise * (0.8 + 0.8 * self.rnd()), d.z * 2.0);
            self.emit(Kind::Spark, f.pos + d * r * 0.5 + Vec3::Y * l * 0.3, v, 2.5, 0.05, 0.03, Vec3::new(1.0, 0.45, 0.12) * 5.0, 1.0, 0.8, 0.12, 0.0);
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
        // ---- tyres (src/ground): smoke where the contact patch scrubs — the
        // wheels spinning up at touchdown, locked-wheel skids, hard cornering —
        // and spray thrown up from a wet / flooded runway.
        let weight = (def.mass * GRAVITY).max(1.0);
        for (i, gp) in def.gear.iter().enumerate().take(crate::ground::MAX_WHEELS) {
            let w = &ac.ground.wheels[i];
            if !w.contact || ac.dmg.gear_lost(def, gp) {
                continue;
            }
            let p = at(gp.pos);
            let p = Vec3::new(p.x, gh(p) + 0.25, p.z);
            let gs = Vec2::new(vel.x, vel.z).length();
            let wet = w.surface.contam.fluid_mm() > 0.2 || matches!(w.surface.contam, crate::ground::Contam::Slush(_));
            // Smoke ∝ the power scrubbed off the rubber (normalised by the aircraft's weight).
            let k = (w.scrub / (weight * 8.0)).min(2.0) * if gs > 6.0 { 1.0 } else { 0.0 };
            // (On a wet runway the spin-up scrub flashes the water film to a thinner vapour.)
            let k = if wet { k * 0.4 } else { k };
            if k > 0.05 {
                let n = self.count(5000 + i as u32, 260.0 * k, dt);
                for _ in 0..n {
                    let d = self.rsph();
                    // Left behind on the runway: most of the wheel's speed is lost at once.
                    let v = vel * (0.3 + 0.25 * self.rnd()) + Vec3::new(d.x * 1.5, 0.6 + 0.8 * self.rnd(), d.z * 1.5);
                    let size = (0.35 + 0.35 * self.rnd()) * (def.gear_height * 0.5).clamp(0.4, 1.0);
                    self.emit(Kind::Steam, p + d * 0.3, v, 2.5 + 1.5 * self.rnd(), size, size * 7.0, Vec3::new(0.7, 0.7, 0.72), (0.3 * k).min(0.55) + 0.12, 1.4, 0.0, 0.15);
                }
            }
            if wet && gs > 12.0 {
                // Rooster tail and side sheets from the tyre's bow wave; more on deeper water.
                let depth = w.surface.contam.fluid_mm().clamp(0.3, 12.0);
                let rate = (gs - 12.0) * (8.0 + 6.0 * depth) * if gp.pos.z < 0.0 { 1.0 } else { 0.5 };
                let n = self.count(5100 + i as u32, rate, dt);
                let fwd = Vec3::new(vel.x, 0.0, vel.z) / gs;
                let side = Vec3::new(-fwd.z, 0.0, fwd.x);
                for _ in 0..n {
                    let r = self.rnd() * 2.0 - 1.0;
                    let up = 1.5 + 4.0 * self.rnd() * (gs / 60.0).min(1.2);
                    let v = vel * (0.45 + 0.3 * self.rnd()) + side * r * (2.0 + gs * 0.08) + Vec3::Y * up;
                    self.emit(Kind::Spray, p - fwd * 0.6 + side * r * 0.3, v, 1.0 + 0.8 * self.rnd(), 0.18, 0.55 + 0.02 * gs, Vec3::splat(0.86), 0.55, 0.5, 1.0, 0.0);
                }
                // A fine mist hangs behind at speed.
                if gs > 35.0 {
                    let n = self.count(5200 + i as u32, (gs - 35.0) * 1.5, dt);
                    for _ in 0..n {
                        let d = self.rsph();
                        self.emit(Kind::Steam, p + d * 0.5, vel * 0.5 + Vec3::Y * 0.8 + d, 2.0 + self.rnd(), 0.6, 4.0, Vec3::splat(0.82), 0.22, 1.0, 0.0, 0.05);
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
                    let p = at(exhaust) + d * 0.3 - vel * dt * self.rnd();
                    self.emit(Kind::Smoke, p, vel * 0.85 + d, 6.0, 0.4, 3.5, Vec3::splat(0.18), 0.55, 0.5, 0.0, 0.0);
                }
            }
            if !running {
                continue;
            }
            let thrust_k = (ac.engine_thrust(i) / (def.static_thrust / n_eng as f32).max(1.0)).clamp(0.0, 1.5);
            if forms > 0.0 {
                // A short clear gap behind the engine while the plume cools
                // and mixes (≈ 0.1 s, a few tens of metres), then a bold,
                // dense line that spreads.
                let p = at(exhaust);
                self.trail(200 + i as u32, mk::TRAIL, p, 0.9 + 0.5 * thrust_k, forms, c_life, c_spread, 1.2, 0.14, 20.0);
            }
        }
        // ---- condensation in low-pressure regions (humid air only)
        let cl = (ac.g_load() * def.mass * GRAVITY / (q * def.wing_area).max(1.0)).clamp(-1.5, 3.5);
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
                    // The visible core is the part of the vortex below the dew
                    // point: wider the more the air is cooled (≈ rc √(1 − spread/ΔT)).
                    let spread = (air.t_c - air.td_c).max(0.0);
                    let wet = (1.0 - spread / atmo::cooling(&air, dp).max(0.05)).clamp(0.15, 1.0).sqrt();
                    let p = at(*tp + Vec3::Y * c.tips[side] - Vec3::Z * 0.3);
                    self.trail(300 + k as u32, mk::VORTEX, p, (0.9 * rc * wet).clamp(0.12, 1.4), (amt * 1.6).min(1.0), 1.6 + 1.2 * amt, 0.05, 0.0, 0.02, 3.0);
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
                    let spread = (air.t_c - air.td_c).max(0.0);
                    let wet = (1.0 - spread / atmo::cooling(&air, air.rho * vc * vc).max(0.05)).clamp(0.15, 1.0).sqrt();
                    let x = sd.fus_r + 0.62 * (semi - sd.fus_r);
                    let te = planform_at(c.planform, x).2;
                    let y = planform_at(c.planform, x).3;
                    for (side, sx) in [(0usize, 1.0f32), (1, -1.0)] {
                        let w = bend(0.62) * c.tips[side];
                        let p = at(Vec3::new(sx * x, y + w, te - 0.3));
                        self.trail(310 + side as u32, mk::VORTEX, p, (1.2 * rc * wet).clamp(0.1, 0.8), (amt * 1.5).min(1.0), 1.2 + 0.8 * amt, 0.04, 0.0, 0.02, 3.0);
                    }
                }
            }
            // Over the wing: the suction over the forward upper surface at
            // high lift (its mean, not the peak), thicker the further the
            // air is cooled past its dew point.
            let cp_up = 0.35 + 1.4 * cl.max(0.0);
            let cool = atmo::cooling(&air, q * cp_up);
            let excess = smoothstep(0.0, 6.0, cool - (air.t_c - air.td_c).max(0.0));
            self.wing_vapour = atmo::condensation(&air, cool) * excess.sqrt() * smoothstep(0.5, 1.2, cl);
            // Vapour cone: the local supersonic pocket around Mach 1 (Prandtl–Glauert).
            let m = ac.mach();
            let bell = smoothstep(0.88, 0.97, m) * (1.0 - smoothstep(1.03, 1.12, m));
            let amt = atmo::condensation(&air, atmo::cooling(&air, q * 0.9 * bell));
            self.cone += (amt * bell - self.cone) * (dt / 0.25).min(1.0);
            self.cone_mach = m;
            self.cone_aoa = ac.alpha().clamp(-0.1, 0.35);
        } else {
            self.wing_vapour = 0.0;
            self.cone = 0.0;
        }
    }

    /// Build the GPU data for this frame (relative to `origin` = the eye).
    pub fn frame(&self, c: &FrameCtx) -> VfxFrame {
        let origin = c.eye;
        let mut f = VfxFrame { origin, ..Default::default() };
        // ---- particles, back to front
        let mut idx: Vec<(f32, usize)> = self.parts.iter().enumerate().filter(|(_, p)| p.age >= 0.0).map(|(i, p)| ((p.pos - origin).length_squared(), i)).collect();
        idx.sort_unstable_by(|a, b| b.0.total_cmp(&a.0));
        f.particles.reserve(idx.len());
        for (_, i) in idx {
            let p = &self.parts[i];
            let t = (p.age / p.life).clamp(0.0, 1.0);
            // Size grows fast at first (turbulent mixing), then slowly.
            let size = if p.flux > 0.0 { p.size1 } else { p.size0 + (p.size1 - p.size0) * (1.0 - (1.0 - t).powi(3)) };
            let mut alpha = p.alpha;
            let mut temp = 0.0;
            let mut vel = p.vel;
            let fade = match p.kind {
                Kind::Fire => {
                    temp = if p.cool > 0.0 {
                        // Fireball: exponential cooling toward soot.
                        900.0 + (p.temp0 - 900.0) * (-p.age / p.cool).exp()
                    } else {
                        // Flame tongue: hot most of its life, its tip cooling to soot.
                        p.temp0 + (980.0 - p.temp0) * smoothstep(0.3, 1.0, t).powf(1.3)
                    };
                    // Flame axis: the motion relative to its source, and up.
                    vel = p.vel - p.src_vel + Vec3::Y * 1.5;
                    smoothstep(0.0, 0.06, t) * (1.0 - smoothstep(if p.cool > 0.0 { 0.5 } else { 0.8 }, 1.0, t))
                }
                Kind::Spark => 1.0 - t * t,
                Kind::Smoke if p.flux > 0.0 => {
                    // A plume dilutes as it spreads (column optical depth ∝ z^-⅔).
                    alpha *= (p.size0 / size).powf(0.6).clamp(0.12, 1.0);
                    smoothstep(0.0, 0.4, p.age) * (1.0 - smoothstep(0.6, 1.0, t))
                }
                _ => smoothstep(0.0, 0.06, t) * (1.0 - smoothstep(0.55, 1.0, t)),
            };
            let rel = p.pos - origin;
            f.particles.push(GpuParticle {
                pos: [rel.x, rel.y, rel.z, size],
                col: [p.col.x, p.col.y, p.col.z, alpha * fade],
                vel: [vel.x, vel.y, vel.z, p.kind as u8 as f32],
                extra: [t, p.rot + t * 0.6, p.seed, p.stretch],
                light: [1.0, 0.0, temp, 1.0],
            });
        }
        // ---- ribbons (centre line + side vector; the shader sets the width)
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
                let side = tan.cross(view).normalize_or(Vec3::X);
                let life_t = (age / t.life).clamp(0.0, 1.0);
                // A gap while the plume mixes, then it appears quickly; dense
                // when thin, fading as it spreads and as the ice sublimates.
                let onset = smoothstep(0.5 * t.fade_in, 1.4 * t.fade_in.max(1e-3), age);
                let a = q.alpha * onset * (1.0 - smoothstep(0.6, 1.0, life_t)) * (q.width / width).powf(0.6).max(0.12);
                let rel = q.pos - origin;
                for s in [-1.0f32, 1.0] {
                    f.mesh.push(VfxVert { pos: [rel.x, rel.y, rel.z, t.kind], uv: [q.born * 4.0, s, a, width], aux: [side.x, side.y, side.z, age] });
                }
            }
            for k in 0..(n as u32 - 1) {
                let i = base + k * 2;
                f.mesh_idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
            }
        }
        // ---- surface / transonic vapour and afterburner diamonds
        self.surface_effects(c, &mut f);
        // ---- fire lights (strongest four), flickering with the puffing
        if let Some(fires) = c.fires {
            let mut v: Vec<&crate::debris::Fire> = fires.iter().filter(|f| f.size > 0.02).collect();
            v.sort_by(|a, b| fire_cd(b).total_cmp(&fire_cd(a)));
            for (k, fi) in v.iter().take(4).enumerate() {
                let (dia, _, l) = flame(fi);
                let rel = fi.pos - origin + Vec3::Y * (0.35 * l).max(1.0);
                let fp = 1.5 / dia.sqrt();
                let ph = std::f32::consts::TAU * fp * self.time + fi.id as f32 * 2.1;
                let flick = 0.84 + 0.1 * ph.sin() + 0.04 * (2.3 * ph + 1.0).sin() + 0.02 * (self.time * 17.0 + k as f32).sin();
                f.fires[k] = [rel.x, rel.y, rel.z, fire_cd(fi) * flick];
            }
        }
        f
    }

    fn surface_effects(&self, c: &FrameCtx, f: &mut VfxFrame) {
        let origin = c.eye;
        let at = |p: Vec3| c.pos + c.rot * p - origin;
        let def = c.def;
        let sd = crate::damage::struct_def(def.id);
        self.volume(c, f);
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
                // Under-expanded jet: Mj from the nozzle pressure ratio (higher
                // as the ambient pressure falls), cell length ≈ 1.22 D √(Mj² − 1).
                let npr = (3.2 * (101_325.0 / self.air.p.max(5000.0)).powf(0.8)).clamp(2.0, 9.0);
                let g = 1.3f32;
                let mj = ((2.0 / (g - 1.0)) * (npr.powf((g - 1.0) / g) - 1.0)).max(1.05).sqrt();
                let ls = 1.22 * 2.0 * r * (mj * mj - 1.0).sqrt();
                diamonds(f, at(nozzle), c.rot * -Vec3::Z, ls, r, ab);
            }
            if thrust > 0.05 {
                let len = (6.0 + 22.0 * thrust + 18.0 * ab) * (r / 0.5).clamp(0.6, 2.0);
                beam(f, at(nozzle), c.rot * -Vec3::Z, len, r * 1.6, mk::HAZE, (0.25 + 0.75 * thrust + ab).min(1.5));
            }
        }
        // Heat haze over fires.
        if let Some(fires) = c.fires {
            for fi in fires.iter().filter(|f| f.size > 0.05).take(4) {
                let (dia, _, l) = flame(fi);
                let base = fi.pos - origin + Vec3::Y * l * 0.6;
                beam(f, base, Vec3::Y, 4.0 + l * 1.5, dia * 0.6, mk::HAZE, fi.size);
            }
        }
    }
}

impl Vfx {
    /// The volumetric condensation (`vfx.wgsl` `fs_vol`): the parameters of
    /// the density fields, in the aircraft's body frame, and their boxes.
    ///
    /// * Transonic (Prandtl–Glauert) vapour: a bell around the wing and one
    ///   over the canopy. The flow accelerates and expands from the leading
    ///   edge (condensation building up) to the near-normal shock, which
    ///   moves aft toward the trailing edge as Mach → 1; the shock evaporates
    ///   it abruptly, so the aft face is crisp. Bigger and offset over the
    ///   suction side with angle of attack.
    /// * Wing-surface fog: a layer over the forward upper surface (the suction
    ///   peak) at high lift in humid air, ending where the pressure recovers.
    fn volume(&self, c: &FrameCtx, f: &mut VfxFrame) {
        let def = c.def;
        let sd = crate::damage::struct_def(def.id);
        let semi = def.wingspan * 0.5;
        let rel = c.pos - c.eye;
        let q = c.rot;
        let v = &mut f.vol;
        v[0] = [rel.x, rel.y, rel.z, self.time];
        v[1] = [q.x, q.y, q.z, q.w];
        let mut mask = 0u32;
        let wings_ok = [crate::damage::Part::WingL, crate::damage::Part::WingR].map(|p| c.lost & p.bit() == 0);
        if self.cone > 0.01 {
            let m = self.cone_mach;
            let ak = (self.cone_aoa / 0.15).clamp(-0.5, 1.5);
            let chord = sd.wing_z.1 - sd.wing_z.0;
            let shock = smoothstep(0.86, 0.99, m);
            let fus = sd.fus_r.min(0.12 * def.wingspan + 0.4);
            let z_f = sd.wing_z.1 + 0.45 * chord;
            let z_s = sd.wing_z.1 - chord * (0.62 + 0.4 * shock);
            let r_s = (0.5 + 0.12 * shock) * semi.max(3.0) * (0.85 + 0.3 * self.cone.min(1.0));
            let y_c = def.wing_height * 0.5;
            v[2] = [z_f, z_s, fus * 0.8, r_s];
            v[3] = [y_c, ak, self.cone.min(1.0), m];
            let mut lo = Vec3::new(-1.3 * r_s, y_c - 1.0 * r_s, z_s - 0.3 * (z_f - z_s));
            let mut hi = Vec3::new(1.3 * r_s, y_c + (1.2 + 0.2 * ak.max(0.0)) * r_s, z_f + 0.2);
            // Canopy bell (fighters: the canopy shock ahead of the wing's).
            if def.eye.z > z_f + 0.5 {
                let zc_f = def.eye.z + 1.0;
                let zc_s = (def.eye.z - 1.7 - 0.6 * shock).max(z_f - 0.3);
                let rc = fus * (1.25 + 0.3 * shock);
                let yc = def.eye.y * 0.55;
                v[4] = [zc_f, zc_s, fus * 0.45, rc];
                v[5] = [yc, self.cone.min(1.0) * 0.85, ak * 0.6, 0.0];
                lo = lo.min(Vec3::new(-1.3 * rc, yc - rc, zc_s - 0.3));
                hi = hi.max(Vec3::new(1.3 * rc, yc + 1.4 * rc, zc_f + 0.2));
            } else {
                v[4] = [0.0, -1.0, 0.0, 0.0];
                v[5] = [0.0; 4];
            }
            v[12] = [lo.x, lo.y, lo.z, 0.0];
            v[13] = [hi.x, hi.y, hi.z, 0.0];
            mask |= 1;
        }
        if self.wing_vapour > 0.01 && !c.planform.is_empty() {
            // Four stations from the root to 90 % span: x, leading edge, trailing edge, y.
            let mut lo = Vec3::splat(f32::MAX);
            let mut hi = Vec3::splat(f32::MIN);
            let tip = c.tips[0].abs().max(c.tips[1].abs());
            for k in 0..4 {
                let x = sd.fus_r + [0.0, 0.3, 0.6, 0.9][k] * (semi - sd.fus_r);
                let (_, le, te, y) = planform_at(c.planform, x);
                v[8 + k] = [x, le, te.min(le - 0.3), y];
                lo = lo.min(Vec3::new(x, y - 0.4, te - 0.4));
                hi = hi.max(Vec3::new(x, y, le + 0.3));
            }
            let chord = (v[8][1] - v[8][2]).max(0.5);
            // Fog depth: a few % of the chord over the upper surface, more as the suction grows.
            let depth = 0.2 * chord * (0.6 + 0.6 * self.wing_vapour.min(1.0));
            v[6] = [self.wing_vapour.min(1.0), sd.fus_r, semi, depth];
            v[7] = [c.tips[0], c.tips[1], 0.0, 0.0];
            hi.y += tip + depth + 0.12 * chord + 0.3;
            lo.y -= tip;
            for (side, sx) in [(0usize, 1.0f32), (1, -1.0)] {
                if !wings_ok[side] {
                    continue;
                }
                let (a, b) = (Vec3::new(sx * lo.x, lo.y, lo.z), Vec3::new(sx * hi.x, hi.y, hi.z));
                let (l, h) = (a.min(b), a.max(b));
                v[14 + 2 * side] = [l.x, l.y, l.z, 0.0];
                v[15 + 2 * side] = [h.x, h.y, h.z, 0.0];
                mask |= 2 << side;
            }
        }
        f.vol_boxes = mask;
    }
}

/// Afterburner plume with shock cells: a camera-facing strip from the nozzle
/// (eye-relative `start`) along `dir`, `cells` long; `ls` = cell length.
fn diamonds(f: &mut VfxFrame, start: Vec3, dir: Vec3, ls: f32, rn: f32, ab: f32) {
    let dir = dir.normalize_or(Vec3::Z);
    let len = ls * (4.0 + 3.0 * ab) + 0.8;
    let hw = rn * 1.3;
    let n = 24;
    let base = f.mesh.len() as u32;
    for k in 0..=n {
        let t = k as f32 / n as f32;
        let p = start + dir * (len * t);
        let side = dir.cross(p.normalize_or(Vec3::Z)).normalize_or(Vec3::X);
        for s in [-1.0f32, 1.0] {
            let q = p + side * hw * s;
            f.mesh.push(VfxVert { pos: [q.x, q.y, q.z, mk::DIAMONDS], uv: [t * len, s, ab, hw], aux: [0.0, 0.0, rn, ls] });
        }
    }
    for k in 0..n as u32 {
        let i = base + k * 2;
        f.mesh_idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
    }
}

/// A camera-facing beam (quad strip along `dir` from eye-relative `start`)
/// for the heat haze.
fn beam(f: &mut VfxFrame, start: Vec3, dir: Vec3, len: f32, radius: f32, kind: f32, intensity: f32) {
    let dir = dir.normalize_or(Vec3::Z);
    let n = 8;
    let base = f.haze.len() as u32;
    for k in 0..=n {
        let t = k as f32 / n as f32;
        let p = start + dir * (len * t);
        let view = p.normalize_or(Vec3::Z);
        let side = dir.cross(view).normalize_or(Vec3::X);
        // Widens downstream (jet spreading / rising plume).
        let r = radius * (1.0 + 1.2 * t);
        for s in [-1.0f32, 1.0] {
            let q = p + side * r * s;
            f.haze.push(VfxVert { pos: [q.x, q.y, q.z, kind], uv: [t * len, s, intensity, len], aux: [0.0; 4] });
        }
    }
    for k in 0..n as u32 {
        let i = base + k * 2;
        f.haze_idx.extend([i, i + 1, i + 2, i + 1, i + 3, i + 2]);
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

    #[test]
    fn flame_height_follows_the_heat_release() {
        let fire = |radius: f32| crate::debris::Fire { pos: Vec3::ZERO, body: None, local: Vec3::ZERO, fuel: 1000.0, rate: (0.05 * std::f32::consts::PI * radius * radius).max(0.5), size: 1.0, age: 10.0, id: 0, radius };
        // Airliner pool (D ≈ 26 m): flames ≈ 1–1.5 D; a light aircraft's (D ≈ 3 m) a few metres.
        let (d, q, l) = flame(&fire(13.0));
        eprintln!("pool D {d} Q {q:.0} kW L {l:.1}");
        assert!((25.0..45.0).contains(&l), "{l}");
        let (_, _, l) = flame(&fire(1.5));
        assert!((3.0..12.0).contains(&l), "{l}");
        // Intensity grows with the fire; bounded (no glow dome from a light the size of the sun).
        assert!(fire_cd(&fire(13.0)) > 20.0 * fire_cd(&fire(1.5)));
        assert!(fire_cd(&fire(13.0)) < 5.0e6);
    }

    #[test]
    fn condensation_volumes_enclose_their_fields() {
        let def = crate::flight::find("lancer").unwrap();
        let mut v = Vfx { cone: 0.8, cone_mach: 0.98, cone_aoa: 0.05, wing_vapour: 0.7, ..Default::default() };
        v.time = 3.0;
        let pf = [(1.0, 2.0, -1.5, 0.0), (3.0, 0.5, -1.4, 0.05), (4.8, -0.6, -1.3, 0.1)];
        let ctx = FrameCtx { def: &def, eye: Vec3::new(0.0, 0.0, -30.0), pos: Vec3::ZERO, rot: Quat::IDENTITY, tips: [0.2, 0.2], lost: 0, running: [false; 4], thrust: [0.0; 4], afterburner: 0.0, planform: &pf, fires: None };
        let f = v.frame(&ctx);
        assert_eq!(f.vol_boxes, 0b111);
        assert!(!f.is_empty());
        let (lo, hi) = (Vec3::from_slice(&f.vol[12][..3]), Vec3::from_slice(&f.vol[13][..3]));
        // The bell: from its front (z_f) to its shock (z_s), radius r_s.
        let [z_f, z_s, _, r_s] = f.vol[2];
        assert!(z_f > z_s && r_s > 1.0);
        assert!(lo.z < z_s && hi.z > z_f && lo.x <= -r_s && hi.x >= r_s);
        // Each wing box lies on its own side and covers the stations.
        assert!(f.vol[14][0] >= 0.0 && f.vol[17][0] <= 0.0);
        let semi = def.wingspan * 0.5;
        assert!(f.vol[15][0] >= 0.85 * semi && f.vol[16][0] <= -0.85 * semi, "{:?}", &f.vol[14..18]);
        assert!(f.vol.iter().all(|q| q.iter().all(|x| x.is_finite())));
        // A lost wing has no fog box.
        let ctx = FrameCtx { lost: crate::damage::Part::WingR.bit(), ..ctx };
        assert_eq!(v.frame(&ctx).vol_boxes, 0b011);
    }
}
