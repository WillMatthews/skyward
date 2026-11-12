//! Wreckage: rigid-body debris after separations and crashes (Track L).
//!
//! The airframe is cut into [`Part`]s (damage.rs; the imported model's
//! vertices are classified by `models.rs`). A part that tears off in flight,
//! and every piece of a crashed aircraft, becomes a [`Body`]: a rigid body
//! with mass and a box inertia from its parts, simulated against the
//! terrain (`Environment::ground_height`, with its slope), building
//! footprints (`obstacle_top`) and water (buoyancy, drag, slow flooding),
//! with Coulomb friction, rolling wheels, aerodynamic drag, tumbling and
//! sleeping once settled.
//!
//! A crash hands the aircraft's last state ([`crate::flight::Impact`]) to
//! [`Debris::crash`]: survivable accidents keep the airframe in one piece
//! (it slides on its belly / remaining gear to a stop), destroyed ones break
//! it along the part hierarchy, with more pieces for more energy. Bodies
//! break again when they hit hard. Fuel-carrying pieces explode and burn.
//!
//! Each step queues sound events (`audio::params::Event`: impacts,
//! scrapes, separations, fires, explosions) and effect events ([`Fx`]) for
//! the particle system.

use crate::audio::params::{Event, Surface};
use crate::damage::{Part, Severity, StructDef, N_PARTS};
use crate::flight::{AircraftDef, Environment, Impact, GRAVITY};
use glam::{Mat3, Quat, Vec2, Vec3};

/// Body-frame boxes of each part's geometry (from the model, or estimated).
#[derive(Clone, Debug)]
pub struct PartGeom {
    pub lo: [Vec3; N_PARTS],
    pub hi: [Vec3; N_PARTS],
    /// Parts that have geometry.
    pub has: u16,
}

impl PartGeom {
    pub fn empty() -> PartGeom {
        PartGeom { lo: [Vec3::splat(f32::MAX); N_PARTS], hi: [Vec3::splat(f32::MIN); N_PARTS], has: 0 }
    }
    pub fn add(&mut self, p: Part, v: Vec3) {
        let i = p.idx();
        self.lo[i] = self.lo[i].min(v);
        self.hi[i] = self.hi[i].max(v);
        self.has |= p.bit();
    }
    /// Boxes from the flight-model geometry (no visual model loaded).
    pub fn estimate(def: &AircraftDef, sd: &StructDef) -> PartGeom {
        let mut g = PartGeom::empty();
        let semi = def.wingspan * 0.5;
        let zs: Vec<f32> = def.hard_points.iter().map(|h| h.pos.z).chain(def.gear.iter().map(|g| g.pos.z)).collect();
        let nose = zs.iter().copied().fold(2.0, f32::max);
        let tail = zs.iter().copied().fold(-2.0, f32::min);
        let r = sd.fus_r;
        let wy = def.wing_height;
        let mut b = |p: Part, lo: Vec3, hi: Vec3| {
            g.add(p, lo);
            g.add(p, hi);
        };
        let fz0 = sd.aft_z.max(tail);
        let fz1 = sd.nose_z.min(nose);
        b(Part::Fuselage, Vec3::new(-r, -r, fz0), Vec3::new(r, r * 1.1, fz1));
        if sd.nose_z < nose {
            b(Part::Nose, Vec3::new(-r, -r, sd.nose_z), Vec3::new(r, r, nose));
        }
        b(Part::Aft, Vec3::new(-r * 0.7, -r * 0.5, tail), Vec3::new(r * 0.7, r, sd.aft_z));
        b(Part::WingL, Vec3::new(r, wy - 0.15, sd.wing_z.0), Vec3::new(semi, wy + 0.25, sd.wing_z.1));
        b(Part::WingR, Vec3::new(-semi, wy - 0.15, sd.wing_z.0), Vec3::new(-r, wy + 0.25, sd.wing_z.1));
        let h = sd.ht_pos;
        b(Part::HTailL, Vec3::new(sd.fin_r, h.y - 0.1, h.z - 1.0), Vec3::new(h.x * 2.0, h.y + 0.1, h.z + 0.8));
        b(Part::HTailR, Vec3::new(-h.x * 2.0, h.y - 0.1, h.z - 1.0), Vec3::new(-sd.fin_r, h.y + 0.1, h.z + 0.8));
        let v = sd.vt_pos;
        b(Part::Fin, Vec3::new(-0.1, sd.fin_y, v.z - 1.2), Vec3::new(0.1, v.y * 2.0 - sd.fin_y, v.z + 1.0));
        for (i, e) in def.engine_pos.iter().enumerate().take(def.engine_count.min(4)) {
            let (nr, nf, na) = sd.nacelle;
            if e.x.abs() > r && nr > 0.0 {
                b(Part::engine(i), *e - Vec3::new(nr, nr, na), *e + Vec3::new(nr, nr, nf));
            } else if let Some(fw) = sd.firewall_z {
                b(Part::engine(i), Vec3::new(-r * 0.8, -r * 0.8, fw), Vec3::new(r * 0.8, r * 0.8, nose));
            }
        }
        for gp in def.gear {
            if let Some(p) = crate::damage::gear_part(def, gp) {
                b(p, gp.pos - Vec3::new(0.25, 0.0, 0.3), gp.pos + Vec3::new(0.25, def.gear_height * 0.8, 0.3));
            }
        }
        g
    }
    pub fn centre(&self, p: Part) -> Vec3 {
        let i = p.idx();
        if self.has & p.bit() == 0 { Vec3::ZERO } else { 0.5 * (self.lo[i] + self.hi[i]) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PtKind {
    /// Airframe skin / structure: slides with metal-on-ground friction.
    Hard,
    /// An attached wheel: rolls along the body's nose axis.
    Wheel,
}

/// One rigid piece of wreckage.
#[derive(Clone, Debug)]
pub struct Body {
    /// Parts in this body (bits of [`Part`]).
    pub parts: u16,
    /// Centre of mass in aircraft-body coordinates.
    pub com: Vec3,
    /// World position of the centre of mass, attitude (aircraft-body axes
    /// → world), velocity, angular velocity (world frame).
    pub pos: Vec3,
    pub rot: Quat,
    pub vel: Vec3,
    pub omega: Vec3,
    pub mass: f32,
    /// Principal moments of inertia (aircraft-body axes x, y, z).
    pub inertia: Vec3,
    /// Contact points relative to the centre of mass (aircraft-body axes).
    pub pts: Vec<(Vec3, PtKind)>,
    pub half: Vec3,
    /// Fuel on board (kg).
    pub fuel: f32,
    pub flooded: f32,
    pub in_water: bool,
    pub contact: bool,
    /// Speed of the sliding contact this step (m/s) and where.
    pub slide: f32,
    pub slide_at: Vec3,
    pub surface: Surface,
    pub sleeping: bool,
    still: f32,
    last_hit: f32,
    last_wall: f32,
    /// Audio slot (scrape id).
    pub id: u8,
    /// Damaged enough to stream smoke.
    pub smoking: bool,
}

impl Body {
    /// Aircraft-body point → world.
    pub fn world(&self, p_body: Vec3) -> Vec3 {
        self.pos + self.rot * (p_body - self.com)
    }
    /// Pose of the aircraft-body origin in the world (for rendering).
    pub fn origin(&self) -> (Vec3, Quat) {
        (self.pos - self.rot * self.com, self.rot)
    }
    fn inertia_world_inv(&self) -> Mat3 {
        let r = Mat3::from_quat(self.rot);
        let d = Mat3::from_diagonal(Vec3::ONE / self.inertia.max(Vec3::splat(1e-3)));
        r * d * r.transpose()
    }
    fn inertia_world(&self) -> Mat3 {
        let r = Mat3::from_quat(self.rot);
        r * Mat3::from_diagonal(self.inertia) * r.transpose()
    }
}

/// A fire (burning fuel) on a body or on the ground.
#[derive(Clone, Copy, Debug)]
pub struct Fire {
    pub pos: Vec3,
    /// Attached to a body (aircraft-body point) or fixed on the ground.
    pub body: Option<usize>,
    pub local: Vec3,
    /// Fuel left (kg) and the burn rate (kg/s).
    pub fuel: f32,
    pub rate: f32,
    /// 0..1 intensity (follows the fuel).
    pub size: f32,
    pub age: f32,
    pub id: u8,
    /// Footprint radius (m).
    pub radius: f32,
}

/// Effect events for the particle system.
#[derive(Clone, Copy, Debug)]
pub enum Fx {
    /// Fuel explosion / fireball: energy 0..1 (scaled by fuel), radius (m).
    Explosion { pos: Vec3, vel: Vec3, energy: f32, radius: f32 },
    /// A body hit the ground: dirt / sparks / spray by surface.
    Impact { pos: Vec3, vel: Vec3, speed: f32, mass: f32, surface: Surface },
    /// Sliding contact this step (dust / sparks / spray), per second of sliding.
    Slide { pos: Vec3, vel: Vec3, speed: f32, mass: f32, surface: Surface },
    /// A part tore off.
    Separation { pos: Vec3, vel: Vec3, mass: f32 },
}

pub struct Debris {
    pub bodies: Vec<Body>,
    /// The main wreck (fuselage) once the aircraft has crashed.
    pub main: Option<usize>,
    pub fires: Vec<Fire>,
    pub severity: Option<Severity>,
    pub time: f32,
    /// Queued for the sound director / particle system (cleared by the app).
    pub audio: Vec<Event>,
    pub fx: Vec<Fx>,
    sd: StructDef,
    def: AircraftDef,
    geom: PartGeom,
    /// Where the crash happened (camera framing).
    pub crash_at: Option<Vec3>,
    seed: u32,
    /// The gear was down at the crash (wheels to roll on).
    gear_down: bool,
}

fn hash01(n: u32) -> f32 {
    let mut x = n.wrapping_mul(0x2C1B_3C6D) ^ 0x297A_2D39;
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 13;
    (x >> 8) as f32 / 16_777_216.0
}

const WATER_RHO: f32 = 1000.0;

impl Debris {
    pub fn new(def: &AircraftDef, geom: PartGeom) -> Debris {
        let sd = *crate::damage::struct_def(def.id);
        Debris {
            bodies: Vec::new(),
            main: None,
            fires: Vec::new(),
            severity: None,
            time: 0.0,
            audio: Vec::new(),
            fx: Vec::new(),
            sd,
            def: def.clone(),
            geom,
            crash_at: None,
            seed: 0x1234_5678,
            gear_down: true,
        }
    }

    fn rnd(&mut self) -> f32 {
        self.seed = self.seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        hash01(self.seed)
    }

    pub fn rebase(&mut self, delta: Vec3) {
        for b in &mut self.bodies {
            b.pos += delta;
            b.slide_at += delta;
        }
        for f in &mut self.fires {
            f.pos += delta;
        }
        if let Some(p) = &mut self.crash_at {
            *p += delta;
        }
    }

    /// Parts that have geometry.
    pub fn geom_has(&self) -> u16 {
        self.geom.has
    }

    /// The body (index) carrying a part, if it is loose.
    pub fn body_of(&self, p: Part) -> Option<usize> {
        self.bodies.iter().position(|b| b.parts & p.bit() != 0)
    }

    /// Build a body for a set of parts from the aircraft's pose.
    fn make_body(&mut self, parts: u16, origin: Vec3, rot: Quat, vel: Vec3, omega_world: Vec3) -> Body {
        let def = &self.def;
        let sd = &self.sd;
        let mut mass = 0.0;
        let mut mom = Vec3::ZERO;
        let mut lo = Vec3::splat(f32::MAX);
        let mut hi = Vec3::splat(f32::MIN);
        let mut fuel = 0.0;
        for p in Part::ALL {
            if parts & p.bit() == 0 {
                continue;
            }
            let m = sd.part_mass(def, p).max(if self.geom.has & p.bit() != 0 { 5.0 } else { 0.0 });
            let c = if self.geom.has & p.bit() != 0 { self.geom.centre(p) } else { sd.part_centre(def, p) };
            mass += m;
            mom += c * m;
            if self.geom.has & p.bit() != 0 {
                lo = lo.min(self.geom.lo[p.idx()]);
                hi = hi.max(self.geom.hi[p.idx()]);
            }
            fuel += match p {
                Part::WingL | Part::WingR => 0.4 * sd.fuel_frac * def.mass,
                Part::Fuselage => 0.2 * sd.fuel_frac * def.mass,
                _ => 0.0,
            };
        }
        if mass <= 0.0 {
            mass = 10.0;
        }
        let com = if lo.x <= hi.x { (mom / mass).clamp(lo, hi) } else { mom / mass };
        if lo.x > hi.x {
            lo = com - Vec3::splat(0.5);
            hi = com + Vec3::splat(0.5);
        }
        let ext = (hi - lo).max(Vec3::splat(0.2));
        // Box inertia, with the mass spread like a shell (×1.4).
        let inertia = 1.4 * mass / 12.0 * Vec3::new(ext.y * ext.y + ext.z * ext.z, ext.x * ext.x + ext.z * ext.z, ext.x * ext.x + ext.y * ext.y);
        // Contact proxies: box corners and edge midpoints of every part, the
        // attached wheels and the flight model's hard points.
        let mut pts: Vec<(Vec3, PtKind)> = Vec::new();
        for p in Part::ALL {
            if parts & p.bit() == 0 || self.geom.has & p.bit() == 0 || p.is_gear() {
                continue;
            }
            let (a, b) = (self.geom.lo[p.idx()], self.geom.hi[p.idx()]);
            for k in 0..8 {
                let c = Vec3::new(if k & 1 == 0 { a.x } else { b.x }, if k & 2 == 0 { a.y } else { b.y }, if k & 4 == 0 { a.z } else { b.z });
                pts.push((c - com, PtKind::Hard));
            }
            // Long parts get extra points along their length.
            let mid = 0.5 * (a + b);
            pts.push((Vec3::new(mid.x, a.y, a.z) - com, PtKind::Hard));
            pts.push((Vec3::new(mid.x, a.y, b.z) - com, PtKind::Hard));
            pts.push((Vec3::new(mid.x, a.y, mid.z) - com, PtKind::Hard));
            pts.push((Vec3::new(mid.x, b.y, mid.z) - com, PtKind::Hard));
        }
        for gp in def.gear {
            if gp.retracts && !self.gear_down {
                continue;
            }
            if let Some(p) = crate::damage::gear_part(def, gp) {
                if parts & p.bit() != 0 {
                    // A wheel attached to this body rolls; a loose gear leg is just a lump.
                    let wheels = parts & Part::Fuselage.bit() != 0 || parts & (Part::WingL.bit() | Part::WingR.bit()) != 0;
                    pts.push((gp.pos - com, if wheels { PtKind::Wheel } else { PtKind::Hard }));
                    pts.push((gp.pos + Vec3::new(0.0, def.gear_height * 0.6, 0.0) - com, PtKind::Hard));
                }
            } else if parts & Part::Fuselage.bit() != 0 {
                pts.push((gp.pos - com, PtKind::Wheel));
            }
        }
        if parts & Part::Fuselage.bit() != 0 {
            for hp in def.hard_points {
                let inside = Part::ALL.iter().any(|p| {
                    parts & p.bit() != 0 && {
                        let (a, b) = (self.geom.lo[p.idx()], self.geom.hi[p.idx()]);
                        (hp.pos.cmpge(a - 0.6) & hp.pos.cmple(b + 0.6)).all()
                    }
                });
                if inside {
                    pts.push((hp.pos - com, PtKind::Hard));
                }
            }
        }
        if pts.is_empty() {
            for k in 0..8 {
                let c = Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
                pts.push((c - com, PtKind::Hard));
            }
        }
        let pos = origin + rot * com;
        let id = (self.bodies.len() % 15) as u8;
        Body {
            parts,
            com,
            pos,
            rot,
            vel: vel + omega_world.cross(rot * com),
            omega: omega_world,
            mass,
            inertia,
            pts,
            half: 0.5 * ext,
            fuel,
            flooded: 0.0,
            in_water: false,
            contact: false,
            slide: 0.0,
            slide_at: pos,
            surface: Surface::Ground,
            sleeping: false,
            still: 0.0,
            last_hit: -10.0,
            last_wall: -10.0,
            id,
            smoking: false,
        }
    }

    /// A part (with what hangs on it, `parts`) tore off the flying aircraft.
    pub fn separate_in_flight(&mut self, parts: u16, origin: Vec3, rot: Quat, vel: Vec3, omega_body: Vec3) {
        if parts == 0 {
            return;
        }
        let omega_w = rot * omega_body;
        let mut b = self.make_body(parts, origin, rot, vel, omega_w);
        // Torn off: it tumbles away.
        let kick = Vec3::new(self.rnd() - 0.5, self.rnd() - 0.5, self.rnd() - 0.5) * 2.0;
        b.omega += rot * kick;
        b.vel += rot * (b.com.normalize_or_zero() * 2.0);
        b.smoking = parts & (Part::WingL.bit() | Part::WingR.bit() | Part::Eng0.bit() | Part::Eng1.bit()) != 0;
        let (pos, m) = (b.pos, b.mass);
        self.audio.push(Event::Separation { pos, mass: m });
        self.fx.push(Fx::Separation { pos, vel: b.vel, mass: m });
        // A wing torn off in the air sprays its fuel: a fireball.
        if b.fuel > 50.0 {
            let e = (b.fuel / 4000.0).sqrt().clamp(0.15, 1.0);
            self.fx.push(Fx::Explosion { pos, vel: b.vel, energy: e * 0.6, radius: 3.0 + 8.0 * e });
            self.audio.push(Event::Explosion { pos, energy: e * 0.6 });
            b.fuel *= 0.5;
        }
        self.bodies.push(b);
    }

    /// The aircraft crashed: the wreck takes over from its last state.
    /// `attached` = parts still on the airframe.
    pub fn crash(&mut self, imp: &Impact, reason: &str, severity: Severity, attached: u16, gear_pos: f32, env: &dyn Environment) {
        if self.main.is_some() {
            return;
        }
        self.gear_down = gear_pos > 0.9;
        self.severity = Some(severity);
        let r = reason.to_ascii_lowercase();
        let in_water = env.is_water(Vec2::new(imp.pos.x, imp.pos.z));
        let omega_w = imp.rot * imp.omega;
        let speed = imp.vel.length();
        let sd = self.sd;
        // Pieces that come off now, by severity.
        let mut off: Vec<u16> = Vec::new();
        let has = |p: Part| attached & p.bit() != 0;
        match severity {
            Severity::Minor => {}
            Severity::Substantial => {
                // The gear goes; an engine that struck the ground may follow.
                for p in [Part::GearNose, Part::GearL, Part::GearR] {
                    if has(p) && (r.contains("hard landing") || r.contains("ground loop") || in_water || self.rnd() < 0.35) {
                        off.push(p.bit());
                    }
                }
                if r.contains("pod strike") {
                    for i in 0..self.def.engine_count.min(4) {
                        let e = Part::engine(i);
                        if has(e) && self.rnd() < 0.5 {
                            off.push(e.bit());
                        }
                    }
                }
            }
            Severity::Destroyed => {
                let big = speed > 45.0 || (-imp.vel.y) > 15.0;
                let def = self.def.clone();
                let wing_children = |w: Part| {
                    let mut m = w.bit();
                    for q in Part::ALL {
                        if q.parent(&sd, &def) == Some(w) && attached & q.bit() != 0 && !(big && q.engine_index().is_some()) {
                            m |= q.bit();
                        }
                    }
                    m
                };
                for w in [Part::WingL, Part::WingR] {
                    if has(w) && (big || self.rnd() < 0.6) {
                        off.push(wing_children(w) & attached);
                    }
                }
                for i in 0..self.def.engine_count.min(4) {
                    let e = Part::engine(i);
                    let on_wing = off.iter().any(|m| m & e.bit() != 0);
                    if has(e) && !on_wing {
                        off.push(e.bit());
                    }
                }
                for p in [Part::GearNose, Part::GearL, Part::GearR] {
                    if has(p) && !off.iter().any(|m| m & p.bit() != 0) {
                        off.push(p.bit());
                    }
                }
                if has(Part::Aft) {
                    if big {
                        off.push(Part::Aft.bit());
                        for p in [Part::Fin, Part::HTailL, Part::HTailR] {
                            if has(p) {
                                off.push(p.bit());
                            }
                        }
                    } else {
                        off.push((Part::Aft.bit() | Part::Fin.bit() | Part::HTailL.bit() | Part::HTailR.bit()) & attached);
                    }
                }
                if has(Part::Nose) && (big || self.rnd() < 0.5) {
                    off.push(Part::Nose.bit());
                }
            }
        }
        let off_all: u16 = off.iter().fold(0, |a, b| a | b);
        let origin = imp.pos;
        let main_parts = attached & !off_all;
        let main = self.make_body(main_parts | Part::Fuselage.bit(), origin, imp.rot, imp.vel, omega_w);
        let main_pos = main.pos;
        self.bodies.push(main);
        let mi = self.bodies.len() - 1;
        self.main = Some(mi);
        self.crash_at = Some(main_pos);
        let violent = severity == Severity::Destroyed;
        for m in off {
            if m == 0 {
                continue;
            }
            let mut b = self.make_body(m, origin, imp.rot, imp.vel, omega_w);
            if violent {
                // Thrown clear: outward from the centre, up, tumbling; slowed a
                // little by the break.
                let out = (imp.rot * b.com).normalize_or_zero();
                let s = (speed * 0.12).clamp(2.0, 14.0);
                b.vel = b.vel * (0.75 + 0.2 * self.rnd()) + out * s * (0.5 + self.rnd()) + Vec3::Y * s * 0.6 * self.rnd();
                b.omega += Vec3::new(self.rnd() - 0.5, self.rnd() - 0.5, self.rnd() - 0.5) * (2.0 + speed * 0.05);
            } else {
                b.vel *= 0.9;
            }
            let (pos, mass) = (b.pos, b.mass);
            self.audio.push(Event::Separation { pos, mass });
            self.fx.push(Fx::Separation { pos, vel: b.vel, mass });
            self.bodies.push(b);
        }
        // Don't start buried (the crash is detected after the fact): lift every
        // body so its lowest point is at most a little below the surface.
        for b in &mut self.bodies {
            let low = b
                .pts
                .iter()
                .map(|(p, _)| {
                    let w = b.pos + b.rot * *p;
                    w.y - env.ground_height(Vec2::new(w.x, w.z))
                })
                .fold(f32::MAX, f32::min);
            if low.is_finite() && low < -0.25 {
                b.pos.y += -0.25 - low;
            }
        }
        // Fuel: fireball and fires (not on water, not in a survivable accident).
        let fuel_total: f32 = self.bodies.iter().map(|b| b.fuel).sum();
        if violent && fuel_total > 1.0 {
            let e = (fuel_total / 6000.0).sqrt().clamp(0.2, 1.0);
            let radius = 4.0 + 22.0 * e;
            if !in_water {
                self.fx.push(Fx::Explosion { pos: main_pos, vel: imp.vel * 0.3, energy: e, radius });
                self.audio.push(Event::Explosion { pos: main_pos, energy: e });
                // Burning pool where the fuel landed, plus the burning pieces.
                let ground = env.ground_height(Vec2::new(main_pos.x, main_pos.z));
                let at = Vec3::new(main_pos.x, ground, main_pos.z) + imp.vel.with_y(0.0).normalize_or_zero() * radius * 0.6;
                self.add_fire(None, at, Vec3::ZERO, fuel_total * 0.35, 3.0 + 10.0 * e);
                let n = self.bodies.len();
                for i in 0..n {
                    let f = self.bodies[i].fuel;
                    if f > 30.0 {
                        let local = self.bodies[i].com;
                        let p = self.bodies[i].pos;
                        self.add_fire(Some(i), p, local, f * 0.6, 1.5 + 5.0 * (f / 3000.0).sqrt().min(1.0));
                        self.bodies[i].fuel *= 0.4;
                    }
                }
            } else {
                self.audio.push(Event::Explosion { pos: main_pos, energy: e * 0.3 });
            }
        }
        for b in &mut self.bodies {
            b.smoking = violent;
        }
    }

    fn add_fire(&mut self, body: Option<usize>, pos: Vec3, local: Vec3, fuel: f32, radius: f32) {
        if self.fires.len() >= 8 {
            return;
        }
        let id = self.fires.len() as u8;
        // Jet fuel pool fires burn ~0.05 kg/s per m²: minutes for an airliner.
        let rate = (0.05 * std::f32::consts::PI * radius * radius).max(0.5);
        self.fires.push(Fire { pos, body, local, fuel, rate, size: 1.0, age: 0.0, id, radius });
    }

    /// Advance the wreckage by `dt`. `paved(xz)`: on a runway / taxiway.
    pub fn update(&mut self, dt: f32, env: &dyn Environment, paved: &dyn Fn(Vec2) -> bool) {
        if dt.is_nan() || dt <= 0.0 || !dt.is_finite() {
            return;
        }
        let dt = dt.min(0.1);
        self.time += dt;
        let n_sub = ((dt / (1.0 / 240.0)).ceil() as usize).clamp(1, 24);
        let h = dt / n_sub as f32;
        for bi in 0..self.bodies.len() {
            if self.bodies[bi].sleeping {
                continue;
            }
            let mut max_hit = 0.0f32;
            let mut hit_pos = Vec3::ZERO;
            let mut hit_surface = Surface::Ground;
            let mut slide = 0.0f32;
            let mut slide_at = self.bodies[bi].pos;
            let mut surface = Surface::Ground;
            let mut contact = false;
            let mut in_water = false;
            for _ in 0..n_sub {
                let b = &mut self.bodies[bi];
                let mut force = Vec3::new(0.0, -GRAVITY * b.mass, 0.0);
                let mut torque = Vec3::ZERO;
                let n_pts = b.pts.len().max(1) as f32;
                let k = b.mass * 900.0 / n_pts.sqrt();
                let c = b.mass * 50.0 / n_pts.sqrt();
                let share = b.mass / n_pts.min(6.0);
                let fwd = b.rot * Vec3::Z;
                let mut wet_pts = 0.0;
                for &(p, kind) in &b.pts {
                    let r = b.rot * p;
                    let pw = b.pos + r;
                    let xz = Vec2::new(pw.x, pw.z);
                    let gh = env.ground_height(xz);
                    let gh = if gh.is_finite() { gh } else { 0.0 };
                    let vp = b.vel + b.omega.cross(r);
                    // Buildings: the roof is a floor; the walls stop what flies in.
                    if let Some(top) = env.obstacle_top(xz).filter(|t| t.is_finite() && *t > gh + 1.0) {
                        if pw.y < top {
                            if top - pw.y < 1.2 && vp.y <= 0.5 && top - gh > 1.5 {
                                let pen = top - pw.y;
                                let fz = (k * pen - c * vp.y).clamp(0.0, 60.0 * b.mass * GRAVITY / n_pts);
                                let f = Vec3::new(0.0, fz, 0.0) - vp.with_y(0.0) * (0.5 * share / h).min(0.6 * fz / vp.with_y(0.0).length().max(1e-3));
                                force += f;
                                torque += r.cross(f);
                                contact = true;
                                surface = Surface::Building;
                            } else if b.last_wall < self.time - 0.25 {
                                // Into a wall: most of the horizontal momentum goes.
                                let hv = b.vel.with_y(0.0);
                                let sp = hv.length();
                                if sp > 2.0 {
                                    b.vel -= hv * 0.85;
                                    b.vel.y += 0.1 * sp;
                                    b.omega += Vec3::new(0.0, (sp * 0.02).min(2.0), 0.0) + r.cross(-hv).normalize_or_zero() * (sp * 0.02).min(2.0);
                                    if sp > max_hit {
                                        max_hit = sp;
                                        hit_pos = pw;
                                        hit_surface = Surface::Building;
                                    }
                                    b.last_wall = self.time;
                                }
                            }
                            if top - pw.y < 1.2 {
                                continue;
                            }
                        }
                    }
                    let pen = gh - pw.y;
                    if pen <= 0.0 {
                        continue;
                    }
                    if env.is_water(xz) {
                        // Buoyancy of the displaced share (a hollow airframe
                        // floats until it floods) and water drag.
                        in_water = true;
                        wet_pts += 1.0;
                        let vol = b.mass / 350.0 / n_pts;
                        let depth = (pen / 1.5).min(1.0);
                        let fb = WATER_RHO * GRAVITY * vol * depth * (1.0 - b.flooded).max(0.0) * 1.3;
                        let area = 4.0 * (b.half.x * b.half.z + b.half.y * b.half.z + b.half.x * b.half.y) / n_pts;
                        let drag = -vp * vp.length() * 0.5 * WATER_RHO * 0.6 * area * depth.max(0.2);
                        let drag = drag.clamp_length_max(share * vp.length() / h * 0.8);
                        let f = Vec3::Y * fb + drag;
                        force += f;
                        torque += r.cross(f);
                        let vn = -vp.y;
                        if vn > max_hit {
                            max_hit = vn;
                            hit_pos = pw;
                            hit_surface = Surface::Water;
                        }
                        let sl = vp.with_y(0.0).length();
                        if sl > slide {
                            slide = sl;
                            slide_at = pw;
                            surface = Surface::Water;
                        }
                        continue;
                    }
                    contact = true;
                    // Terrain normal from the height field.
                    let d = 1.5;
                    let hx = env.ground_height(xz + Vec2::new(d, 0.0));
                    let hz = env.ground_height(xz + Vec2::new(0.0, d));
                    let n = if hx.is_finite() && hz.is_finite() { Vec3::new(-(hx - gh) / d, 1.0, -(hz - gh) / d).normalize() } else { Vec3::Y };
                    let pen_n = pen * n.y;
                    let vn = vp.dot(n);
                    let fz = (k * pen_n * (1.0 + (pen_n / 0.5).powi(2)) - c * vn).clamp(0.0, 80.0 * b.mass * GRAVITY / n_pts.sqrt());
                    if -vn > max_hit {
                        max_hit = -vn;
                        hit_pos = pw;
                        hit_surface = if paved(xz) { Surface::Paved } else { Surface::Ground };
                    }
                    let vt = vp - n * vn;
                    let mut f = n * fz;
                    match kind {
                        PtKind::Wheel => {
                            // Rolls along the nose axis, grips sideways.
                            let along = (fwd - n * fwd.dot(n)).normalize_or_zero();
                            let side = n.cross(along);
                            let vl = vt.dot(along);
                            let vs = vt.dot(side);
                            let fl = -(vl * share * 0.1 / h).clamp(-0.03 * fz, 0.03 * fz);
                            let fs = -(vs * share * 0.4 / h).clamp(-0.7 * fz, 0.7 * fz);
                            f += along * fl + side * fs;
                        }
                        PtKind::Hard => {
                            let on_paved = paved(xz);
                            // Metal on concrete slides; on soil it digs in.
                            let mu = if on_paved { 0.38 } else { 0.55 };
                            let vtl = vt.length();
                            if vtl > 1e-4 {
                                let cap = mu * fz;
                                let ft = (vtl * share * 0.5 / h).min(cap);
                                f -= vt / vtl * ft;
                            }
                            if vtl > slide {
                                slide = vtl;
                                slide_at = pw;
                                surface = if on_paved { Surface::Paved } else { Surface::Ground };
                            }
                        }
                    }
                    force += f;
                    torque += r.cross(f);
                }
                // Air drag per body axis (a box is mostly empty: ~10 % fill),
                // so a loose wing falls flat and slow; tumbling resistance.
                let vb = b.rot.conjugate() * b.vel;
                let hb = b.half;
                let face = Vec3::new(hb.y * hb.z, hb.x * hb.z, hb.x * hb.y) * 4.0 * 0.1;
                let fd = -vb * vb.abs() * face * (0.5 * 1.2 * 1.1);
                force += b.rot * fd;
                let area = face.x + face.y + face.z;
                let lmax = b.half.max_element() * 2.0;
                torque += -b.omega * b.omega.length() * 0.05 * 1.2 * area * lmax * lmax * lmax / 8.0;
                torque += -b.omega * 0.02 * b.inertia.max_element();
                // Water floods the wreck: it settles, then sinks.
                if wet_pts > 0.0 {
                    let rate = if b.parts == Part::Fuselage.bit() || b.parts & Part::Fuselage.bit() == 0 { 1.0 / 45.0 } else { 1.0 / 120.0 };
                    b.flooded = (b.flooded + rate * h * (wet_pts / n_pts).sqrt()).min(1.0);
                }
                // Integrate.
                b.vel += force / b.mass * h;
                let iw = b.inertia_world();
                let gyro = b.omega.cross(iw * b.omega);
                b.omega += b.inertia_world_inv() * (torque - gyro) * h;
                let wl = b.omega.length();
                if wl > 25.0 {
                    b.omega *= 25.0 / wl;
                }
                // Resting contacts rock to a stop (rolling resistance, crushing).
                if contact {
                    b.omega /= 1.0 + 2.5 * h;
                }
                b.pos += b.vel * h;
                b.rot = (Quat::from_scaled_axis(b.omega * h) * b.rot).normalize();
                if !(b.pos.is_finite() && b.vel.is_finite() && b.omega.is_finite() && b.rot.is_finite()) {
                    // Never let a numerical fault escape: freeze where it was.
                    b.vel = Vec3::ZERO;
                    b.omega = Vec3::ZERO;
                    if !b.pos.is_finite() {
                        b.pos = slide_at;
                    }
                    if !b.rot.is_finite() {
                        b.rot = Quat::IDENTITY;
                    }
                    b.sleeping = true;
                    break;
                }
            }
            // ---- per-step bookkeeping, sounds and effects
            let time = self.time;
            let b = &mut self.bodies[bi];
            b.contact = contact;
            b.in_water = in_water;
            b.slide = slide;
            b.slide_at = slide_at;
            b.surface = surface;
            let (mass, vel, id) = (b.mass, b.vel, b.id);
            if max_hit > 2.5 && time - b.last_hit > 0.2 {
                b.last_hit = time;
                self.audio.push(Event::Impact { pos: hit_pos, speed: max_hit, mass, surface: hit_surface });
                self.fx.push(Fx::Impact { pos: hit_pos, vel, speed: max_hit, mass, surface: hit_surface });
            }
            let b = &mut self.bodies[bi];
            if slide > 1.0 && (contact || in_water) {
                self.audio.push(Event::Scrape { id, pos: slide_at, speed: slide, mass, surface });
                self.fx.push(Fx::Slide { pos: slide_at, vel: b.vel, speed: slide, mass, surface });
            }
            // Asleep once it has stopped (not while sinking).
            if (contact || (in_water && b.flooded >= 1.0)) && b.vel.length() < 0.12 && b.omega.length() < 0.1 {
                b.still += dt;
                if b.still > 1.5 && !(in_water && b.flooded < 1.0) {
                    b.sleeping = true;
                    b.vel = Vec3::ZERO;
                    b.omega = Vec3::ZERO;
                }
            } else {
                b.still = 0.0;
            }
            // Sunk out of sight: stop simulating.
            if in_water && b.flooded >= 1.0 {
                let gh = env.ground_height(Vec2::new(b.pos.x, b.pos.z));
                if b.pos.y < gh - 12.0 {
                    b.sleeping = true;
                    b.vel = Vec3::ZERO;
                    b.omega = Vec3::ZERO;
                }
            }
            // Hard hits break a multi-part body further.
            if max_hit > 14.0 && b.parts.count_ones() > 1 && self.main != Some(bi) && hit_surface != Surface::Water {
                self.break_body(bi, max_hit);
            }
        }
        // Main wreck: a violent first impact may still break it (structural
        // failure in the air, then the ground).
        if let Some(mi) = self.main {
            let b = &self.bodies[mi];
            if b.parts.count_ones() > 2 && self.severity == Some(Severity::Destroyed) && b.contact && b.last_hit == self.time {
                // (handled at crash time; nothing more for now)
            }
        }
        // Fires: follow their bodies, burn their fuel down.
        for f in &mut self.fires {
            f.age += dt;
            if let Some(bi) = f.body {
                if let Some(b) = self.bodies.get(bi) {
                    f.pos = b.world(f.local);
                    if b.in_water {
                        f.fuel = 0.0;
                    }
                }
            }
            f.fuel = (f.fuel - f.rate * dt).max(0.0);
            // Grows over the first seconds, dies down as the fuel runs out.
            let grow = (f.age / 4.0).min(1.0);
            let left = (f.fuel / (f.rate * 30.0)).min(1.0);
            f.size = (grow * left).clamp(0.0, 1.0);
        }
        for f in &self.fires {
            self.audio.push(Event::Fire { id: f.id, pos: f.pos, size: f.size });
        }
    }

    /// Split a body into its parts (keeping the motion).
    fn break_body(&mut self, bi: usize, speed: f32) {
        let b = self.bodies[bi].clone();
        let (origin, rot) = b.origin();
        let mut groups: Vec<u16> = Vec::new();
        for p in Part::ALL {
            if b.parts & p.bit() != 0 {
                groups.push(p.bit());
            }
        }
        if groups.len() < 2 {
            return;
        }
        // The heaviest piece keeps this slot.
        let first = groups.remove(0);
        let mut keep = self.make_body(first, origin, rot, b.vel, b.omega);
        keep.vel = b.vel + b.omega.cross(rot * keep.com - (rot * b.com));
        keep.id = b.id;
        self.bodies[bi] = keep;
        for g in groups {
            let mut nb = self.make_body(g, origin, rot, b.vel, b.omega);
            nb.vel = b.vel + b.omega.cross(rot * nb.com - rot * b.com) + Vec3::new(self.rnd() - 0.5, self.rnd(), self.rnd() - 0.5) * speed * 0.15;
            nb.smoking = true;
            let (pos, mass) = (nb.pos, nb.mass);
            self.audio.push(Event::Separation { pos, mass });
            self.fx.push(Fx::Separation { pos, vel: nb.vel, mass });
            if self.bodies.len() < 24 {
                self.bodies.push(nb);
            }
        }
    }

    /// Everything has come to rest.
    pub fn settled(&self) -> bool {
        self.bodies.iter().all(|b| b.sleeping)
    }

    /// Centre and radius of the wreckage (for the crash camera).
    pub fn extent(&self) -> Option<(Vec3, f32)> {
        if self.bodies.is_empty() {
            return None;
        }
        let main = self.main.and_then(|m| self.bodies.get(m)).map(|b| b.pos);
        let c = main.unwrap_or_else(|| self.bodies.iter().map(|b| b.pos).sum::<Vec3>() / self.bodies.len() as f32);
        let r = self.bodies.iter().filter(|b| b.pos.distance(c) < 600.0).map(|b| b.pos.distance(c) + b.half.max_element()).fold(5.0, f32::max);
        Some((c, r))
    }

    /// The world pose of a loose part's aircraft-body frame.
    pub fn part_pose(&self, p: Part) -> Option<(Vec3, Quat)> {
        let i = self.body_of(p)?;
        Some(self.bodies[i].origin())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damage::{struct_def, Severity};
    use crate::flight::find;

    #[derive(Default)]
    struct Env {
        water: bool,
        slope: f32,
        building: Option<(Vec2, Vec2, f32)>,
    }
    impl Environment for Env {
        fn ground_height(&self, xz: Vec2) -> f32 {
            xz.x * self.slope
        }
        fn obstacle_top(&self, xz: Vec2) -> Option<f32> {
            let (lo, hi, top) = self.building?;
            (xz.x >= lo.x && xz.x <= hi.x && xz.y >= lo.y && xz.y <= hi.y).then_some(top)
        }
        fn is_water(&self, _xz: Vec2) -> bool {
            self.water
        }
        fn updraft(&self, _p: Vec3) -> f32 {
            0.0
        }
    }

    fn sane(d: &Debris) {
        for b in &d.bodies {
            assert!(b.pos.is_finite() && b.vel.is_finite() && b.omega.is_finite() && b.rot.is_finite(), "{b:?}");
            assert!(b.pos.y > -50.0 && b.pos.y < 5000.0);
        }
    }

    fn crash(id: &str, vel: Vec3, pitch: f32, sev: Severity, env: &Env, secs: f32) -> Debris {
        let def = find(id).unwrap();
        let sd = struct_def(def.id);
        let geom = PartGeom::estimate(&def, sd);
        let mut d = Debris::new(&def, geom);
        let rot = crate::flight::attitude(0.0, pitch, 0.0);
        let imp = Impact { pos: Vec3::new(0.0, def.gear_height * 0.5, 0.0), vel, rot, omega: Vec3::ZERO, on_ground: true };
        let attached: u16 = if sev == Severity::Minor { !(1 << 15) & !crate::damage::GEAR_BITS } else { !(1 << 15) };
        d.crash(&imp, "test", sev, attached, 0.0, env);
        let paved = |_: Vec2| true;
        for _ in 0..(secs * 60.0) as usize {
            d.update(1.0 / 60.0, env, &paved);
            sane(&d);
        }
        d
    }

    #[test]
    fn belly_slide_stops_in_one_piece() {
        // A320 gear-up at 130 kt: slides on its belly, in one piece, to a stop.
        let env = Env::default();
        let d = crash("meridian320", Vec3::new(0.0, -1.0, 67.0), 0.04, Severity::Minor, &env, 40.0);
        assert_eq!(d.bodies.len(), 1);
        let b = &d.bodies[0];
        eprintln!("belly slide: stopped {:.0} m on, y {:.2}, speed {:.2}, asleep {}", b.pos.z, b.pos.y, b.vel.length(), b.sleeping);
        assert!(b.sleeping, "settled");
        assert!(b.pos.z > 150.0 && b.pos.z < 900.0, "slide distance {}", b.pos.z);
        // Upright and resting on the surface.
        assert!((b.rot * Vec3::Y).y > 0.9);
        assert!(b.pos.y > 0.0 && b.pos.y < 4.0);
        assert!(d.fires.is_empty());
    }

    #[test]
    fn destroyed_breaks_up_burns_and_settles() {
        let env = Env::default();
        let d = crash("meridian320", Vec3::new(0.0, -25.0, 80.0), -0.3, Severity::Destroyed, &env, 60.0);
        eprintln!("breakup: {} bodies, {} fires, settled {}", d.bodies.len(), d.fires.len(), d.settled());
        assert!(d.bodies.len() >= 6);
        assert!(!d.fires.is_empty() && d.fires.iter().any(|f| f.size > 0.5));
        assert!(d.audio.iter().any(|e| matches!(e, Event::Explosion { .. })));
        let resting = d.bodies.iter().filter(|b| b.sleeping).count();
        assert!(resting * 10 >= d.bodies.len() * 8, "{resting}/{}", d.bodies.len());
        for b in &d.bodies {
            // Nothing sank into or flew off the ground.
            let low = b.pts.iter().map(|(p, _)| (b.pos + b.rot * *p).y).fold(f32::MAX, f32::min);
            assert!(low > -1.0, "body below ground {low}");
            assert!(b.pos.length() < 2000.0);
        }
    }

    #[test]
    fn debris_settles_on_a_slope_and_floats_then_sinks() {
        // 10 % slope: everything still comes to rest.
        let env = Env { slope: 0.1, ..Default::default() };
        let d = crash("kestrel", Vec3::new(0.0, -15.0, 40.0), -0.5, Severity::Destroyed, &env, 40.0);
        for b in &d.bodies {
            eprintln!("slope: parts {:#06x} pos {:?} v {:.2} w {:.2} sleeping {} contact {}", b.parts, b.pos, b.vel.length(), b.omega.length(), b.sleeping, b.contact);
        }
        assert!(d.bodies.iter().filter(|b| b.sleeping).count() * 10 >= d.bodies.len() * 7);
        // Water: the fuselage floats at first, then floods and sinks.
        let env = Env { water: true, ..Default::default() };
        let d = crash("meridian320", Vec3::new(0.0, -2.0, 60.0), 0.05, Severity::Substantial, &env, 20.0);
        let m = &d.bodies[d.main.unwrap()];
        eprintln!("ditched after 20 s: y {:.1}, flooded {:.2}", m.pos.y, m.flooded);
        assert!(m.pos.y > -4.0, "still afloat");
        assert!(d.fires.is_empty());
        let d = crash("meridian320", Vec3::new(0.0, -2.0, 60.0), 0.05, Severity::Substantial, &env, 200.0);
        let m = &d.bodies[d.main.unwrap()];
        eprintln!("after 200 s: y {:.1}, flooded {:.2}", m.pos.y, m.flooded);
        assert!(m.pos.y < -4.0, "sunk");
    }

    #[test]
    fn building_stops_the_wreck() {
        let env = Env { building: Some((Vec2::new(-30.0, 40.0), Vec2::new(30.0, 80.0), 25.0)), ..Default::default() };
        let d = crash("kestrel", Vec3::new(0.0, -1.0, 35.0), 0.0, Severity::Substantial, &env, 10.0);
        let m = &d.bodies[d.main.unwrap()];
        assert!(m.pos.z < 60.0, "went through the building: {}", m.pos.z);
        assert!(d.audio.iter().any(|e| matches!(e, Event::Impact { surface: Surface::Building, .. })));
    }

    #[test]
    fn in_flight_separation_falls_and_lands() {
        let def = find("meridian320").unwrap();
        let sd = struct_def(def.id);
        let mut d = Debris::new(&def, PartGeom::estimate(&def, sd));
        d.separate_in_flight(Part::WingL.bit() | Part::Eng0.bit(), Vec3::new(0.0, 800.0, 0.0), Quat::IDENTITY, Vec3::new(0.0, 0.0, 150.0), Vec3::ZERO);
        let env = Env::default();
        for _ in 0..(90.0 * 60.0) as usize {
            d.update(1.0 / 60.0, &env, &|_| false);
            sane(&d);
        }
        let b = &d.bodies[0];
        eprintln!("wing: landed at {:?}, asleep {}", b.pos, b.sleeping);
        assert!(b.pos.y < 10.0 && b.contact || b.sleeping);
        assert!(b.pos.z > 100.0 && b.pos.z < 3000.0);
    }
}
