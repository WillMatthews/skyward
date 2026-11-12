//! Crashes, impacts, wreckage: layered, positional, scaled by energy.
//!
//! A crash is a short story, not one burst: the first heavy impact
//! (recorded metal impact + a low body thud), the structure crunching and
//! tearing (recorded crunch + metal shrieks), the wreck grinding along while
//! it slides (velocity-driven scrape loop, surface-dependent), debris
//! clattering down for a few seconds, then for fuel-laden aircraft a fuel
//! explosion and a burning fire loop, and a long low rumble tail. Everything
//! is placed in the world, so outside cameras get distance attenuation, air
//! absorption and the propagation delay. In the cockpit a non-survivable
//! impact is a muffled crunch cut to ringing ears; a survivable one is
//! muffled but continues.
//!
//! The same pieces are exposed as events for debris physics
//! (`Event::Impact`, `Scrape`, `Separation`, `Fire`, `Explosion`): each
//! debris piece can drive its own impacts and scrapes, and fires persist
//! until put out.

use super::dsp::*;
use super::fx::{Kind, Pool, Route, Shot};
use super::params::Surface;
use super::samples::{self, Pcm, Voice};
use glam::Vec3;

/// Reference distance of world-placed sounds (their `amp` is the level at this range).
pub const WORLD_REF: f32 = 30.0;
const C: f32 = 343.0;

/// Energy → 0..1 size: a C172 at 30 m/s ≈ 0.3, an F-16 at 150 m/s ≈ 0.9,
/// an A320 at 75 m/s ≈ 0.95.
pub fn size_of(mass: f32, speed: f32) -> f32 {
    let e = 0.5 * mass.max(1.0) * speed * speed;
    ((e.max(1.0).log10() - 4.5) / 4.0).clamp(0.03, 1.0)
}

/// Heavier structures ring lower.
pub fn pitch_of(mass: f32) -> f32 {
    (1050.0 / mass.max(50.0)).powf(0.1).clamp(0.7, 1.15)
}

/// Delay (s) for a sound at `pos` to reach the ear.
fn travel(pos: Vec3, ear: Vec3) -> f32 {
    (pos.distance(ear) / C).min(8.0)
}

pub struct Ctx<'a> {
    pub pool: &'a mut Pool,
    pub rng: &'a mut Rng,
    pub sr: f32,
    pub ear: Vec3,
}

impl Ctx<'_> {
    fn add(&mut self, s: Shot, delay: f32) {
        let d = delay
            + if s.route == Route::World {
                travel(s.wpos, self.ear)
            } else {
                0.0
            };
        self.pool.add(s.delayed(d, self.sr).init(self.sr, self.rng));
    }
    fn sample(&mut self, name: &str, pitch: f32, amp: f32, pos: Vec3, delay: f32) {
        if let Some(s) = Shot::sample(name, pitch, Route::World, amp, 0.0, self.sr) {
            self.add(s.at(pos), delay);
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn thud(&mut self, f0: f32, f1: f32, decay: f32, noise: f32, amp: f32, pos: Vec3, delay: f32) {
        self.add(
            Shot::new(
                Kind::Thud {
                    f0,
                    f1,
                    ph: 0.0,
                    decay,
                    noise,
                },
                Route::World,
                amp,
                decay * 5.0 + 0.1,
            )
            .at(pos),
            delay,
        );
    }
    fn jitter(&mut self, pos: Vec3, r: f32) -> Vec3 {
        pos + Vec3::new(self.rng.bi(), 0.2 * self.rng.uni(), self.rng.bi()) * r
    }
}

/// The aircraft hit something hard enough to be destroyed.
#[allow(clippy::too_many_arguments)]
pub fn crash(
    cx: &mut Ctx,
    wreck: &mut Wreck,
    pos: Vec3,
    vel: Vec3,
    mass: f32,
    surface: Surface,
    in_air: bool,
    cockpit: bool,
) -> bool {
    let speed = vel.length();
    let sz = size_of(mass, speed);
    let pm = pitch_of(mass);
    // Level at WORLD_REF: a light single's crash already out-shouts its
    // engine at full power; an airliner's is far louder (the limiter rides it).
    let a = 0.8 + 1.3 * sz;
    let heavy = mass > 8000.0;
    let hs = Vec2Len(vel).len();
    let fwd = Vec3::new(vel.x, 0.0, vel.z).normalize_or_zero();
    // ---- 1. first impact
    if in_air {
        // Structural failure: the airframe tears apart.
        cx.sample("crash_heavy", pm, a * 0.9, pos, 0.0);
        cx.sample("crash_f16", pm * 1.1, a * 0.5, pos, 0.05);
    } else if surface == Surface::Water {
        cx.sample("crash_water", pm.powf(0.5), a * 1.1, pos, 0.0);
        cx.thud(90.0, 38.0, 0.22, 3.0, a * 0.55, pos, 0.0);
    } else {
        if mass < 5000.0 {
            cx.sample("crash_c172", 1.0, a * 0.9, pos, 0.0);
        } else {
            cx.sample("crash_f16", pm, a * 0.95, pos, 0.0);
            cx.sample("crash_c172", pm * 0.85, a * 0.55, pos, 0.025);
        }
        cx.thud(
            110.0 * pm,
            32.0,
            0.16 + 0.25 * sz,
            4.0 + 3.0 * sz,
            a * 0.85,
            pos,
            0.0,
        );
        if surface == Surface::Building {
            cx.sample("crash_heavy", pm * 1.15, a * 0.6, pos, 0.04);
            for i in 0..4 {
                cx.add(
                    Shot::new(Kind::Clank { kicked: false }, Route::World, 0.25 * a, 0.8).at(pos),
                    0.03 + 0.08 * i as f32,
                );
            }
        }
    }
    // ---- 2. crunch and tearing metal
    if surface != Surface::Water || in_air {
        let (u1, u2) = (cx.rng.uni(), cx.rng.uni());
        cx.sample(
            "crash_heavy",
            pm * (0.95 + 0.1 * u1),
            a * 0.7,
            pos,
            0.07 + 0.05 * u2,
        );
    }
    let tears = 2 + (3.0 * sz) as usize;
    for _ in 0..tears {
        let f0 = (600.0 + 1300.0 * cx.rng.uni()) * pm;
        let d = 0.05 + 0.6 * cx.rng.uni();
        let p = cx.jitter(pos, 3.0);
        let (u1, u2) = (cx.rng.uni(), cx.rng.uni());
        cx.add(
            Shot::new(
                Kind::Squeal {
                    f0,
                    f1: f0 * (0.4 + 0.2 * u1),
                    ph: 0.0,
                },
                Route::World,
                0.12 * a,
                0.3 + 0.7 * u2,
            )
            .at(p),
            d,
        );
    }
    // ---- 3. the wreck slides
    if !in_air && surface != Surface::Water && hs > 12.0 {
        let decel = if surface == Surface::Paved { 3.0 } else { 4.5 };
        wreck.slide(pos, fwd, hs * 0.75, decel, mass, surface, cx.ear);
    }
    if surface == Surface::Water && hs > 10.0 {
        // Spray as it skips across the water.
        cx.add(
            Shot::new(
                Kind::Rumble {
                    attack: 0.02,
                    crackle: 0.0,
                },
                Route::World,
                0.25 * a,
                1.5 + hs / 30.0,
            )
            .at(pos + fwd * 10.0),
            0.15,
        );
    }
    // ---- 4. debris clattering down
    let n = 3 + (6.0 * sz) as usize;
    for _ in 0..n {
        let d = 0.25 + 2.4 * cx.rng.uni() * cx.rng.uni().sqrt();
        let (u1, u2, u3) = (cx.rng.uni(), cx.rng.uni(), cx.rng.uni());
        let p = cx.jitter(pos + fwd * hs * 0.4 * u1, 5.0 + 20.0 * sz);
        let pitch = (0.8 + 0.6 * u2) * pm.sqrt();
        cx.sample("crash_light", pitch, a * (0.12 + 0.25 * u3), p, d);
        if cx.rng.uni() < 0.5 {
            cx.add(
                Shot::new(Kind::Clank { kicked: false }, Route::World, a * 0.12, 0.6).at(p),
                d + 0.05,
            );
        }
    }
    // ---- 5. fuel: explosion and fire (big / fuel-laden aircraft)
    let e = 0.5 * mass * speed * speed;
    if heavy || e > 2.0e7 {
        let d = 0.15 + 0.3 * cx.rng.uni();
        cx.sample("crash_explode", 0.75 + 0.25 * pm, a * 1.1, pos, d);
        cx.thud(60.0, 25.0, 0.5, 2.0, a * 0.7, pos, d);
        wreck.fire(0, pos + fwd * hs * 0.3, 0.45 + 0.55 * sz, true, cx.ear);
    } else if e > 3.0e5 && surface != Surface::Water {
        wreck.fire(0, pos + fwd * hs * 0.2, 0.25, true, cx.ear);
    }
    // ---- 6. rumble tail
    cx.add(
        Shot::new(
            Kind::Rumble {
                attack: 0.04,
                crackle: 0.4,
            },
            Route::World,
            a * 0.35,
            3.0 + 3.0 * sz,
        )
        .at(pos),
        0.05,
    );
    // ---- cockpit view
    let fatal = sz > 0.3 || speed > 30.0;
    if cockpit {
        let sr = cx.sr;
        let mut s = Shot::new(
            Kind::Thud {
                f0: 85.0,
                f1: 30.0,
                ph: 0.0,
                decay: 0.3,
                noise: 6.0,
            },
            Route::Cockpit,
            0.9,
            1.4,
        );
        s = s.muffled(if fatal { 380.0 } else { 650.0 }, sr);
        cx.add(s, 0.0);
        if let Some(s) = Shot::sample("crash_heavy", pm, Route::Cockpit, 0.75, 0.0, sr) {
            cx.add(s.muffled(if fatal { 1100.0 } else { 2200.0 }, sr), 0.02);
        }
        if fatal {
            // Ringing ears after the cut to silence.
            let f = 3900.0 + 900.0 * cx.rng.uni();
            cx.add(
                Shot::new(Kind::Ring { f, ph: 0.0 }, Route::Ui, 0.035, 7.0),
                0.14,
            );
        }
    }
    fatal && cockpit
}

/// Impact of an airframe part or debris at `pos` (debris physics hook).
pub fn impact(cx: &mut Ctx, pos: Vec3, speed: f32, mass: f32, surface: Surface) {
    let sz = size_of(mass, speed);
    let pm = pitch_of(mass);
    let a = 0.15 + 0.85 * sz;
    match surface {
        Surface::Water => {
            cx.sample("crash_water", (pm * 1.3).min(1.4), a * 0.7, pos, 0.0);
        }
        _ => {
            let name = if sz > 0.45 {
                "crash_heavy"
            } else {
                "crash_light"
            };
            let u = cx.rng.uni();
            cx.sample(name, pm * (0.9 + 0.2 * u), a * 0.8, pos, 0.0);
            cx.thud(140.0 * pm, 45.0, 0.08 + 0.15 * sz, 3.0, a * 0.5, pos, 0.0);
            if surface == Surface::Metal || surface == Surface::Building || cx.rng.uni() < 0.4 {
                cx.add(
                    Shot::new(Kind::Clank { kicked: false }, Route::World, a * 0.2, 0.7).at(pos),
                    0.01,
                );
            }
        }
    }
}

/// A part tearing off.
pub fn separation(cx: &mut Ctx, pos: Vec3, mass: f32) {
    let pm = pitch_of(mass);
    let a = 0.3 + 0.5 * ((mass.max(10.0).log10() - 1.0) / 3.0).clamp(0.0, 1.0);
    for i in 0..2 {
        let f0 = (700.0 + 900.0 * cx.rng.uni()) * pm;
        let u = cx.rng.uni();
        cx.add(
            Shot::new(
                Kind::Squeal {
                    f0,
                    f1: f0 * 0.45,
                    ph: 0.0,
                },
                Route::World,
                0.16 * a,
                0.5 + 0.4 * u,
            )
            .at(pos),
            0.04 * i as f32,
        );
    }
    cx.sample("crash_heavy", pm * 1.1, a * 0.6, pos, 0.02);
    cx.thud(120.0 * pm, 50.0, 0.12, 2.5, a * 0.4, pos, 0.0);
}

/// A fuel / ordnance explosion.
pub fn explosion(cx: &mut Ctx, pos: Vec3, energy: f32) {
    let k = energy.clamp(0.05, 1.0);
    cx.sample(
        "crash_explode",
        0.85 + 0.25 * (1.0 - k),
        0.5 + 0.7 * k,
        pos,
        0.0,
    );
    cx.thud(55.0, 24.0, 0.4 + 0.4 * k, 2.0, 0.6 * k, pos, 0.0);
    cx.add(
        Shot::new(
            Kind::Rumble {
                attack: 0.03,
                crackle: 0.6,
            },
            Route::World,
            0.4 * k,
            2.5 + 3.0 * k,
        )
        .at(pos),
        0.02,
    );
    for _ in 0..(2.0 + 5.0 * k) as usize {
        let d = 0.4 + 2.0 * cx.rng.uni();
        let p = cx.jitter(pos, 15.0 + 30.0 * k);
        let u = cx.rng.uni();
        cx.sample("crash_light", 0.9 + 0.5 * u, 0.15 * k, p, d);
    }
}

struct Vec2Len(Vec3);
impl Vec2Len {
    fn len(&self) -> f32 {
        (self.0.x * self.0.x + self.0.z * self.0.z).sqrt()
    }
}

// ------------------------------------------------------------------ loops

/// Positional gain / pan / air absorption, updated per block.
#[derive(Clone, Copy, Debug, Default)]
struct Place {
    gl: Ramp,
    gr: Ramp,
    air: OnePole,
}

impl Place {
    fn update(&mut self, pos: Vec3, ear: Vec3, right: Vec3, sr: f32, n: u32) {
        let (g, (l, r), fc) = place(pos, ear, right);
        self.gl.to(g * l, n);
        self.gr.to(g * r, n);
        self.air.set(fc, sr);
    }
}

/// (distance gain, (pan l, pan r), air-absorption cutoff) for a world position.
pub fn place(pos: Vec3, ear: Vec3, right: Vec3) -> (f32, (f32, f32), f32) {
    let dv = pos - ear;
    let d = dv.length().max(0.5);
    let g = (WORLD_REF / d.max(4.0)).min(2.5);
    let x = dv.dot(right) / d;
    let fc = (19_000.0 * (160.0 / (d + 160.0)).powf(1.05)).max(250.0);
    (g, pan(x * 0.9), fc)
}

#[derive(Clone, Copy)]
struct Fire {
    on: bool,
    /// Propagation delay still to wait before it is heard (s).
    wait: f32,
    pos: Vec3,
    size: f32,
    target: f32,
    /// Slowly dies down (crash fires) vs held at `target` (driven fires).
    decays: bool,
    amp: Ramp,
    pink: Pink,
    roar: Svf,
    body: Svf,
    flick: OnePole,
    crack_env: f32,
    crack_amp: f32,
    crack_hp: OnePole,
    /// Band-limits the crackle (pops, not digital clicks).
    crack_bp: Svf,
    crack_att: f32,
    place: Place,
}

#[derive(Clone, Copy)]
struct Scrape {
    on: bool,
    wait: f32,
    pos: Vec3,
    dir: Vec3,
    speed: f32,
    decel: f32,
    mass: f32,
    surface: Surface,
    /// Seconds since the last update (driven scrapes) — fades out after.
    age: f32,
    auto: bool,
    v: Option<(Voice, &'static Pcm)>,
    amp: Ramp,
    grind: Svf,
    grit: Svf,
    place: Place,
}

/// Long-lived wreckage sounds: fires and sliding / grinding bodies.
pub struct Wreck {
    fires: [Fire; 8],
    scrapes: [Scrape; 16],
    sr: f32,
}

impl Wreck {
    pub fn new(sr: f32) -> Wreck {
        let f = Fire {
            on: false,
            wait: 0.0,
            pos: Vec3::ZERO,
            size: 0.0,
            target: 0.0,
            decays: false,
            amp: Ramp::new(0.0),
            pink: Pink::default(),
            roar: Svf::new(260.0, 0.6, sr),
            body: Svf::new(900.0, 0.5, sr),
            flick: OnePole::new(2.5, sr),
            crack_env: 0.0,
            crack_amp: 0.0,
            crack_hp: OnePole::new(1800.0, sr),
            crack_bp: Svf::new(2600.0, 0.6, sr),
            crack_att: 0.0,
            place: Place::default(),
        };
        let s = Scrape {
            on: false,
            wait: 0.0,
            pos: Vec3::ZERO,
            dir: Vec3::ZERO,
            speed: 0.0,
            decel: 0.0,
            mass: 1000.0,
            surface: Surface::Ground,
            age: 0.0,
            auto: false,
            v: None,
            amp: Ramp::new(0.0),
            grind: Svf::new(180.0, 0.8, sr),
            grit: Svf::new(2500.0, 0.7, sr),
            place: Place::default(),
        };
        Wreck {
            fires: [f; 8],
            scrapes: [s; 16],
            sr,
        }
    }

    /// Everything off (respawn / new aircraft).
    pub fn clear(&mut self) {
        for f in &mut self.fires {
            f.target = 0.0;
            f.decays = false;
        }
        for s in &mut self.scrapes {
            s.speed = 0.0;
            s.auto = false;
            s.age = 1.0;
        }
    }

    pub fn fire(&mut self, id: u8, pos: Vec3, size: f32, decays: bool, ear: Vec3) {
        let f = &mut self.fires[(id as usize).min(7)];
        if !f.on {
            f.on = true;
            f.size = 0.0;
            f.wait = travel(pos, ear);
        }
        f.pos = pos;
        f.target = size.clamp(0.0, 1.0);
        f.decays = decays;
    }

    /// Driven scrape update (debris physics).
    pub fn scrape(
        &mut self,
        id: u8,
        pos: Vec3,
        speed: f32,
        mass: f32,
        surface: Surface,
        ear: Vec3,
    ) {
        let s = &mut self.scrapes[(id as usize).min(14)];
        if !s.on {
            s.wait = travel(pos, ear);
        }
        s.on = true;
        s.auto = false;
        s.pos = pos;
        s.speed = speed.max(0.0);
        s.mass = mass;
        s.surface = surface;
        s.age = 0.0;
    }

    /// The crashed airframe sliding to a stop (self-driven, slot 15).
    #[allow(clippy::too_many_arguments)]
    pub fn slide(
        &mut self,
        pos: Vec3,
        dir: Vec3,
        v0: f32,
        decel: f32,
        mass: f32,
        surface: Surface,
        ear: Vec3,
    ) {
        let s = &mut self.scrapes[15];
        s.wait = travel(pos, ear);
        s.on = true;
        s.auto = true;
        s.pos = pos;
        s.dir = dir;
        s.speed = v0;
        s.decel = decel;
        s.mass = mass;
        s.surface = surface;
        s.age = 0.0;
    }

    pub fn control(&mut self, ear: Vec3, right: Vec3, n: u32, dt: f32) {
        let sr = self.sr;
        for f in &mut self.fires {
            if !f.on {
                continue;
            }
            if f.wait > 0.0 {
                f.wait -= dt;
                f.place.update(f.pos, ear, right, sr, n);
                continue;
            }
            if f.decays {
                // A crash fire flares up over a few seconds, then slowly burns down.
                f.target *= (-dt / 50.0).exp();
            }
            let rate = if f.target > f.size { 0.6 } else { 0.25 };
            f.size += (f.target - f.size) * (dt * rate).min(1.0);
            if f.size < 0.003 && f.target < 0.003 {
                f.on = false;
                f.amp.to(0.0, n);
                continue;
            }
            f.amp.to(0.13 * f.size.sqrt(), n);
            f.roar.set(180.0 + 200.0 * (1.0 - f.size), 0.6, sr);
            f.place.update(f.pos, ear, right, sr, n);
        }
        let mut sample: Option<&'static Pcm> = None;
        for s in &mut self.scrapes {
            if !s.on {
                continue;
            }
            if s.wait > 0.0 {
                s.wait -= dt;
                continue;
            }
            if s.auto {
                s.speed = (s.speed - s.decel * dt).max(0.0);
                s.pos += s.dir * s.speed * dt;
            } else {
                s.age += dt;
            }
            let fade = if s.auto {
                1.0
            } else {
                1.0 - smoothstep(0.15, 0.4, s.age)
            };
            if s.speed < 0.3 || fade <= 0.0 {
                s.amp.to(0.0, n);
                if s.amp.v < 1e-4 {
                    s.on = false;
                    s.v = None;
                }
                continue;
            }
            if s.v.is_none() {
                if sample.is_none() {
                    sample = samples::pcm(samples::id("crash_scrape"));
                }
                if let Some(p) = sample {
                    let mut v = Voice::new(samples::id("crash_scrape"), 0.0);
                    v.gain = Ramp::new(1.0);
                    s.v = Some((v, p));
                }
            }
            let k = (s.speed / 25.0).clamp(0.0, 1.5);
            let mk = ((s.mass.max(10.0).log10() - 1.5) / 3.5).clamp(0.1, 1.0);
            let (bright, gritty) = match s.surface {
                Surface::Paved | Surface::Metal => (1.25, 0.6),
                Surface::Water => (0.5, 0.0),
                _ => (0.85, 1.0),
            };
            s.amp.to(0.6 * k.sqrt() * mk * fade, n);
            if let Some((v, p)) = &mut s.v {
                let pitch = (0.65 + 0.45 * k.min(1.0)) * bright * pitch_of(s.mass).sqrt();
                v.inc.to(p.rate / sr * pitch, n);
            }
            s.grind.set(90.0 + 160.0 * k * bright, 0.9, sr);
            s.grit.set(1800.0 * bright, 0.6, sr);
            let _ = gritty;
            s.place.update(s.pos, ear, right, sr, n);
        }
    }

    /// Stereo output (exterior, positional).
    #[inline]
    pub fn tick(&mut self, rng: &mut Rng) -> (f32, f32) {
        let (mut l, mut r) = (0.0f32, 0.0f32);
        for f in &mut self.fires {
            if !f.on {
                continue;
            }
            let w = rng.bi();
            let pk = f.pink.tick(w);
            let fl = 0.6 + 0.8 * f.flick.lp(rng.bi() * 6.0).abs().min(1.0);
            // Crackle: sparse sharp pops, denser for a bigger fire.
            if rng.uni() < (40.0 + 260.0 * f.size) / self.sr {
                f.crack_env = 1.0;
                f.crack_amp = rng.uni().powi(3) * 1.6;
            }
            // ~0.4 ms attack, ~2 ms decay, band-passed: a pop.
            f.crack_att += (f.crack_env - f.crack_att) * (2500.0 / self.sr).min(1.0);
            f.crack_env *= 1.0 - 450.0 / self.sr;
            let crack = f.crack_bp.bpn(f.crack_hp.hp(f.crack_att * f.crack_amp * w));
            let y = (f.roar.lp(pk) * 2.2 * fl + f.body.bpn(pk) * 0.35 * fl + crack) * f.amp.tick();
            let y = f.place.air.lp(y);
            l += y * f.place.gl.tick();
            r += y * f.place.gr.tick();
        }
        let k = samples::kern();
        for s in &mut self.scrapes {
            if !s.on {
                continue;
            }
            let w = rng.bi();
            let a = s.amp.tick();
            let mut y = s.grind.bpn(w) * 0.9 + s.grit.bpn(w) * 0.15;
            if let Some((v, p)) = &mut s.v {
                y += v.tick(p, k) * 1.6;
            }
            let y = s.place.air.lp(y * a);
            l += y * s.place.gl.tick();
            r += y * s.place.gr.tick();
        }
        (l, r)
    }
}
