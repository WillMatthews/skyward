//! Air-to-air missiles: motor, drag (zero-lift + induced), aerodynamic and
//! structural g limits, proportional navigation, IR / active-radar seekers
//! with countermeasures, proximity fuse and an annular blast-fragmentation
//! warhead.
//!
//! * AIM-9M: 86 kg, Mk 36 boost-sustain motor (≈ 62 kN·s), ±40° gimbal
//!   all-aspect InSb seeker with IRCCM (rejects part of the flares), 35 g,
//!   active optical target detector ≈ 9 m, WDU-17/B 9.4 kg annular frag.
//! * AIM-120C: 161 kg, WPU-6 boost-sustain motor (≈ 118 kN·s), inertial
//!   midcourse with datalink updates from the launcher's radar, active
//!   radar terminal ("pitbull") inside ≈ 15 km, 40 g, Doppler notch, chaff.
//!
//! Guidance is true proportional navigation `a = N Vc Ω × r̂` (N = 4) plus
//! gravity compensation, with half the target's lateral acceleration added
//! (augmented PN) when the target state comes from a track; the AIM-120
//! lofts on long shots.

use crate::flight::GRAVITY;
use glam::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MissileKind {
    Aim9,
    Aim120,
}

#[derive(Clone, Copy, Debug)]
pub enum SeekerKind {
    /// Gimbal limit (rad), instantaneous FOV half-angle (rad), reference
    /// lock range (m) for a 1,500 W/sr source, flare rejection probability.
    Ir { gimbal: f32, fov: f32, r_ref: f32, irccm: f32 },
    /// Gimbal (rad), active range for a 5 m² target (m), beam half-width (rad).
    Radar { gimbal: f32, active: f32, beam: f32 },
}

#[derive(Clone, Copy, Debug)]
pub struct MissileDef {
    pub kind: MissileKind,
    pub name: &'static str,
    pub mass: f32,
    pub propellant: f32,
    pub diameter: f32,
    pub length: f32,
    /// Motor: (end time s, thrust N) segments.
    pub motor: [(f32, f32); 2],
    pub max_g: f32,
    /// Normal-force coefficient limit and slope (per rad) on the body cross-section.
    pub cn_max: f32,
    pub cn_alpha: f32,
    pub nav: f32,
    pub seeker: SeekerKind,
    pub fuze: f32,
    pub warhead: f32,
    pub fragments: f32,
    /// Fragment mass (kg) and initial speed (m/s, plus the missile's).
    pub frag_mass: f32,
    pub frag_v: f32,
    pub arm: f32,
    pub life: f32,
}

pub const AIM9M: MissileDef = MissileDef {
    kind: MissileKind::Aim9,
    name: "AIM-9M",
    mass: 86.0,
    propellant: 27.0,
    diameter: 0.127,
    length: 2.87,
    motor: [(2.5, 18_000.0), (5.3, 6_000.0)],
    max_g: 35.0,
    cn_max: 15.0,
    cn_alpha: 20.0,
    nav: 4.0,
    seeker: SeekerKind::Ir { gimbal: 0.70, fov: 0.045, r_ref: 10_000.0, irccm: 0.55 },
    fuze: 9.0,
    warhead: 9.4,
    fragments: 950.0,
    frag_mass: 0.003,
    frag_v: 1900.0,
    arm: 0.45,
    life: 60.0,
};

pub const AIM120C: MissileDef = MissileDef {
    kind: MissileKind::Aim120,
    name: "AIM-120C",
    mass: 161.0,
    propellant: 50.0,
    diameter: 0.178,
    length: 3.66,
    motor: [(3.0, 26_000.0), (8.0, 8_000.0)],
    max_g: 40.0,
    cn_max: 15.0,
    cn_alpha: 20.0,
    nav: 4.0,
    seeker: SeekerKind::Radar { gimbal: 1.05, active: 15_000.0, beam: 0.07 },
    fuze: 12.0,
    warhead: 22.7,
    fragments: 1800.0,
    frag_mass: 0.004,
    frag_v: 1800.0,
    arm: 0.8,
    life: 90.0,
};

pub fn def_of(k: MissileKind) -> &'static MissileDef {
    match k {
        MissileKind::Aim9 => &AIM9M,
        MissileKind::Aim120 => &AIM120C,
    }
}

impl MissileKind {
    pub fn weapon(self) -> super::events::WeaponKind {
        match self {
            MissileKind::Aim9 => super::events::WeaponKind::Aim9,
            MissileKind::Aim120 => super::events::WeaponKind::Aim120,
        }
    }
}

/// Zero-lift drag coefficient (body cross-section) vs Mach, motor off / on (base drag).
fn cd0(m: f32, burning: bool) -> f32 {
    let base = if burning { 0.0 } else { 0.12 };
    let c = if m < 0.8 {
        0.30
    } else if m < 1.1 {
        0.30 + (m - 0.8) / 0.3 * 0.45
    } else {
        (0.75 - (m - 1.1) * 0.12).max(0.42)
    };
    c + base
}

/// What a seeker can see.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Source {
    Unit(u32),
    Flare(u32),
    Chaff(u32),
}

/// An IR emitter / radar reflector this step.
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub src: Source,
    pub pos: Vec3,
    pub vel: Vec3,
    /// IR: radiant intensity toward the seeker (W/sr); radar: RCS (m²).
    pub strength: f32,
    /// Seconds since it appeared (flares: since ejection).
    pub age: f32,
}

/// IR radiant intensity of an aircraft toward `from` (W/sr, 3–5 µm): hot
/// parts and plume seen from behind, airframe and plume from the front;
/// afterburner multiplies the plume.
pub fn ir_intensity(pos: Vec3, fwd: Vec3, from: Vec3, jet: bool, power: f32, ab: f32) -> f32 {
    let to = (from - pos).normalize_or_zero();
    let rear = ((1.0 - to.dot(fwd)) * 0.5).powi(2);
    let (front, back) = if jet { (300.0, 1500.0) } else { (90.0, 380.0) };
    let p = 0.35 + 0.65 * power.clamp(0.0, 1.0);
    (front + (back - front) * rear) * p * (1.0 + 9.0 * ab.clamp(0.0, 1.0) * (0.3 + 0.7 * rear))
}

/// Flare radiant intensity (MJU-7 class): fast rise, ~4.5 s burn.
pub fn flare_intensity(age: f32) -> f32 {
    if age < 0.0 || age > 4.5 {
        return 0.0;
    }
    18_000.0 * (age / 0.12).min(1.0) * (1.0 - (age / 4.5).powi(3))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Off the rail, not yet guiding.
    Launch,
    /// Midcourse (radar missile on datalink / inertial).
    Midcourse,
    /// Seeker tracking.
    Terminal,
    /// Seeker lost everything: flies on.
    Ballistic,
}

#[derive(Clone, Debug)]
pub struct Missile {
    pub id: u32,
    pub def: &'static MissileDef,
    pub owner: u32,
    pub target: Option<u32>,
    pub pos: Vec3,
    pub vel: Vec3,
    pub t: f32,
    pub phase: Phase,
    /// Seeker line of sight (world unit vector) and what it is tracking.
    pub seek: Vec3,
    pub tracking: Option<Source>,
    /// Target estimate (datalink / inertial / seeker) and its age.
    pub est_pos: Vec3,
    pub est_vel: Vec3,
    pub est_acc: Vec3,
    pub lost_t: f32,
    /// Sources this seeker has decided to ignore (IRCCM), and those judged.
    pub rejected: Vec<Source>,
    pub judged: Vec<Source>,
    /// Active seeker on (radar), and whether the launcher still updates it.
    pub active: bool,
    pub datalink: bool,
    /// Closest range to the target so far (fuze / miss logic).
    pub min_range: f32,
    pub prev_pos: Vec3,
    /// Lateral acceleration commanded last step (m/s²).
    pub a_lat: Vec3,
    pub loft: f32,
}

impl Missile {
    pub fn new(id: u32, def: &'static MissileDef, owner: u32, target: Option<u32>, pos: Vec3, vel: Vec3, seek: Vec3, est: Option<(Vec3, Vec3)>) -> Missile {
        let (ep, ev) = est.unwrap_or((pos + seek * 5000.0, Vec3::ZERO));
        let range = ep.distance(pos);
        // Long AIM-120 shots climb first to fly in thinner air.
        let loft = if matches!(def.seeker, SeekerKind::Radar { .. }) { ((range - 18_000.0) / 30_000.0).clamp(0.0, 1.0) * 0.35 } else { 0.0 };
        Missile {
            id,
            def,
            owner,
            target,
            pos,
            vel,
            t: 0.0,
            phase: Phase::Launch,
            seek,
            tracking: target.map(Source::Unit),
            est_pos: ep,
            est_vel: ev,
            est_acc: Vec3::ZERO,
            lost_t: 0.0,
            rejected: Vec::new(),
            judged: Vec::new(),
            active: false,
            datalink: matches!(def.seeker, SeekerKind::Radar { .. }),
            min_range: f32::MAX,
            prev_pos: pos,
            a_lat: Vec3::ZERO,
            loft,
        }
    }
    pub fn mass(&self) -> f32 {
        let burnt = (self.t / self.def.motor[1].0).clamp(0.0, 1.0);
        self.def.mass - self.def.propellant * burnt
    }
    pub fn thrust(&self) -> f32 {
        let m = &self.def.motor;
        if self.t < m[0].0 {
            m[0].1
        } else if self.t < m[1].0 {
            m[1].1
        } else {
            0.0
        }
    }
    pub fn burning(&self) -> bool {
        self.t < self.def.motor[1].0
    }
    pub fn speed(&self) -> f32 {
        self.vel.length()
    }
    pub fn dir(&self) -> Vec3 {
        self.vel.normalize_or(Vec3::Z)
    }

    /// IR seeker: track the intensity-weighted centroid of what lies in its
    /// field of view; IRCCM rejects a share of new flares; the sun blinds it.
    fn seeker_ir(&mut self, contacts: &[Contact], sun: Vec3, rng: &mut dyn FnMut() -> f32, dt: f32) {
        let SeekerKind::Ir { gimbal, fov, r_ref, irccm } = self.def.seeker else { return };
        let body = self.dir();
        let mut wsum = 0.0;
        let mut pc = Vec3::ZERO;
        let mut vc = Vec3::ZERO;
        let mut best: Option<(Source, f32)> = None;
        // The sun inside the field of view saturates the detector.
        if sun.y > -0.05 && self.seek.angle_between(sun) < fov * 1.5 {
            self.tracking = None;
            self.seek = (self.seek + (sun - self.seek) * (dt * 2.0)).normalize();
            self.lost_t += dt;
            return;
        }
        for c in contacts {
            if matches!(c.src, Source::Chaff(_)) {
                continue;
            }
            let d = c.pos - self.pos;
            let r = d.length().max(1.0);
            let dir = d / r;
            if dir.angle_between(self.seek) > fov || dir.angle_between(body) > gimbal {
                continue;
            }
            // Detectable: above the noise floor at this range.
            if c.strength < 1500.0 * (r / r_ref).powi(2) {
                continue;
            }
            if let Source::Flare(_) = c.src {
                if !self.judged.contains(&c.src) {
                    self.judged.push(c.src);
                    // Judged once, when it first appears in the field of view.
                    if rng() < irccm {
                        self.rejected.push(c.src);
                    }
                }
                if self.rejected.contains(&c.src) {
                    continue;
                }
            }
            let w = c.strength / (r * r);
            wsum += w;
            pc += c.pos * w;
            vc += c.vel * w;
            if best.is_none_or(|b| w > b.1) {
                best = Some((c.src, w));
            }
        }
        if wsum <= 0.0 {
            self.lost_t += dt;
            self.tracking = None;
            // Coast: keep the inertial estimate.
            self.est_pos += self.est_vel * dt;
            return;
        }
        self.lost_t = 0.0;
        let p = pc / wsum;
        let v = vc / wsum;
        let new_vel = v;
        self.est_acc = self.est_acc.lerp((new_vel - self.est_vel) / dt.max(1e-3), (dt * 4.0).min(1.0));
        self.est_pos = p;
        self.est_vel = v;
        self.seek = (p - self.pos).normalize_or(self.seek);
        self.tracking = best.map(|b| b.0);
    }

    /// Active radar seeker: strongest return in the range gate and beam
    /// around the estimate; ground-relative Doppler notch filters both
    /// clutter-speed targets (beaming) and chaff.
    fn seeker_radar(&mut self, contacts: &[Contact], dt: f32) -> bool {
        let SeekerKind::Radar { gimbal, active, beam } = self.def.seeker else { return false };
        let body = self.dir();
        let est_dir = (self.est_pos - self.pos).normalize_or(body);
        let mut best: Option<(&Contact, f32)> = None;
        for c in contacts {
            if matches!(c.src, Source::Flare(_)) {
                continue;
            }
            let d = c.pos - self.pos;
            let r = d.length().max(1.0);
            let dir = d / r;
            if dir.angle_between(body) > gimbal || dir.angle_between(est_dir) > beam * 3.0 {
                continue;
            }
            if (r - self.est_pos.distance(self.pos)).abs() > 600.0 + r * 0.05 {
                continue;
            }
            // Detection range ∝ RCS^¼.
            if r > active * (c.strength / 5.0).powf(0.25) {
                continue;
            }
            let vr = c.vel.dot(dir).abs();
            let looking_down = dir.y < -0.02;
            let notch = if looking_down { smooth(10.0, 35.0, vr) } else { 1.0 };
            let floor = if matches!(c.src, Source::Chaff(_)) { 0.03 } else { 0.0 };
            let w = c.strength * notch.max(floor) / r.powi(4) * 1e16;
            if w > 0.0 && best.is_none_or(|b| w > b.1) {
                best = Some((c, w));
            }
        }
        match best {
            Some((c, _)) => {
                self.est_acc = self.est_acc.lerp((c.vel - self.est_vel) / dt.max(1e-3), (dt * 3.0).min(1.0));
                self.est_pos = c.pos;
                self.est_vel = c.vel;
                self.tracking = Some(c.src);
                self.lost_t = 0.0;
                true
            }
            None => {
                self.tracking = None;
                self.est_pos += self.est_vel * dt;
                self.lost_t += dt;
                false
            }
        }
    }

    /// Datalink midcourse update from the launcher's radar track.
    pub fn uplink(&mut self, pos: Vec3, vel: Vec3) {
        if self.datalink && !self.active {
            self.est_pos = pos;
            self.est_vel = vel;
        }
    }

    /// One step: seeker, guidance, motor, aerodynamics, integration.
    pub fn step(&mut self, contacts: &[Contact], sun: Vec3, rng: &mut dyn FnMut() -> f32, dt: f32) {
        self.prev_pos = self.pos;
        self.t += dt;
        let atm = crate::flight::isa(self.pos.y);
        let v = self.speed().max(1.0);
        let mach = v / atm.speed_of_sound;
        let q = 0.5 * atm.density * v * v;
        let s = std::f32::consts::PI * 0.25 * self.def.diameter * self.def.diameter;
        let m = self.mass();
        // ---- seeker
        if self.t > 0.25 {
            match self.def.seeker {
                SeekerKind::Ir { .. } => {
                    self.seeker_ir(contacts, sun, rng, dt);
                    self.phase = if self.tracking.is_some() {
                        Phase::Terminal
                    } else if self.lost_t > 2.0 {
                        Phase::Ballistic
                    } else {
                        self.phase
                    };
                }
                SeekerKind::Radar { active, .. } => {
                    if !self.active {
                        self.est_pos += self.est_vel * dt;
                        if self.est_pos.distance(self.pos) < active {
                            self.active = true;
                        }
                        self.phase = Phase::Midcourse;
                    }
                    if self.active {
                        let got = self.seeker_radar(contacts, dt);
                        self.phase = if got {
                            Phase::Terminal
                        } else if self.lost_t > 3.0 {
                            Phase::Ballistic
                        } else {
                            Phase::Midcourse
                        };
                    }
                }
            }
        } else {
            self.est_pos += self.est_vel * dt;
        }
        // ---- guidance (proportional navigation on the estimate)
        let mut a_cmd = Vec3::ZERO;
        if self.t > 0.35 && self.phase != Phase::Ballistic {
            let r = self.est_pos - self.pos;
            let rl = r.length().max(1.0);
            let vr = self.est_vel - self.vel;
            let omega = r.cross(vr) / (rl * rl);
            let vc = (-vr.dot(r) / rl).max(50.0);
            let rhat = r / rl;
            a_cmd = omega.cross(rhat) * (self.def.nav * vc);
            // Augmented PN: lead the target's own manoeuvre (lateral part).
            let at = self.est_acc - rhat * self.est_acc.dot(rhat);
            a_cmd += at * (self.def.nav * 0.5);
            // Loft: climb while far away.
            if self.loft > 0.0 && rl > 12_000.0 && self.t < 12.0 {
                let up = Vec3::Y - self.dir() * self.dir().y;
                a_cmd += up.normalize_or_zero() * (self.loft * 9.0 * GRAVITY);
            }
        }
        // Gravity compensation (lateral part of +g).
        let dirn = self.dir();
        let gcomp = Vec3::Y * GRAVITY;
        a_cmd += gcomp - dirn * gcomp.dot(dirn);
        // Lateral only, limited by structure and by available normal force.
        a_cmd -= dirn * a_cmd.dot(dirn);
        let a_aero = q * s * self.def.cn_max / m;
        let a_max = (self.def.max_g * GRAVITY).min(a_aero).max(0.5 * GRAVITY);
        if a_cmd.length() > a_max {
            a_cmd = a_cmd.normalize() * a_max;
        }
        self.a_lat = a_cmd;
        // ---- forces
        let n = m * a_cmd.length();
        let d0 = q * s * cd0(mach, self.burning());
        // Induced: the normal force tilted by the angle of attack it needs.
        let di = if q > 1.0 { n * n / (q * s * self.def.cn_alpha) } else { 0.0 };
        let along = (self.thrust() - d0 - di) / m;
        let acc = dirn * along + a_cmd + Vec3::new(0.0, -GRAVITY, 0.0);
        self.vel += acc * dt;
        self.pos += self.vel * dt;
    }

    /// Speed below which it can no longer manoeuvre usefully.
    pub fn spent(&self) -> bool {
        (!self.burning() && self.speed() < 230.0) || self.t > self.def.life
    }
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Closest approach of two straight segments over one step (positions at
/// the start and end): (distance, fraction of the step).
pub fn closest_approach(m0: Vec3, m1: Vec3, t0: Vec3, t1: Vec3) -> (f32, f32) {
    let p = m0 - t0;
    let d = (m1 - m0) - (t1 - t0);
    let dd = d.length_squared();
    let u = if dd > 1e-9 { (-p.dot(d) / dd).clamp(0.0, 1.0) } else { 0.0 };
    ((p + d * u).length(), u)
}

/// Expected fragment hits on each hit volume of an aircraft from a warhead
/// bursting at `burst` (world) while flying along `axis`: an annular spray
/// ±`band` about the plane normal to the axis, fragments slowing in the air.
/// Returns (part, number of fragments, damage per fragment, body-frame point).
pub fn fragment_hits(
    def: &MissileDef,
    burst: Vec3,
    axis: Vec3,
    missile_vel: Vec3,
    hm: &super::hitbox::HitModel,
    ac_pos: Vec3,
    ac_rot: glam::Quat,
    ac_vel: Vec3,
) -> Vec<(super::hitbox::HitPart, f32, f32, Vec3)> {
    let band = 14f32.to_radians();
    let omega_band = 4.0 * std::f32::consts::PI * band.sin();
    let inv = ac_rot.inverse();
    let local_burst = inv * (burst - ac_pos);
    let axis_b = (inv * axis).normalize_or(Vec3::Z);
    let mut out = Vec::new();
    for v in &hm.vols {
        let c = 0.5 * (v.lo + v.hi);
        if v.wing && !hm.planform.contains(c) {
            continue;
        }
        let to = c - local_burst;
        let r = to.length().max(0.3);
        if r > 60.0 {
            continue;
        }
        let dir = to / r;
        let size = v.hi - v.lo;
        let area = size.y * size.z * dir.x.abs() + size.x * size.z * dir.y.abs() + size.x * size.y * dir.z.abs();
        let ang_r = (area.sqrt() * 0.5 / r).atan();
        // Inside the spray band (allowing for the volume's angular size)?
        let off = dir.dot(axis_b).asin().abs();
        let inband = smooth(band + ang_r, band, off).max(if off < band { 1.0 } else { 0.0 });
        if inband <= 0.0 {
            continue;
        }
        let solid = (area / (r * r)).min(2.0 * std::f32::consts::PI);
        let n = def.fragments * solid / omega_band * inband;
        // Fragment speed decays (≈ 1/e over 120 m for a 3 g steel cube).
        let world_dir = ac_rot * dir;
        let v_f = (def.frag_v * (-r / 120.0).exp()) * world_dir + missile_vel - ac_vel;
        let e = 0.5 * def.frag_mass * v_f.length_squared();
        let dmg = e / 1000.0 * 0.5;
        // Blast: overpressure close in (≈ 3 m for 10 kg).
        let blast = def.warhead * 60.0 / (r * r).max(0.6);
        if n > 0.01 || blast > 1.0 {
            out.push((v.part, n, dmg + blast / n.max(1.0), local_burst + to.clamp_length_max(r - 0.2)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Point-mass target: flies straight, then breaks at `g` (horizontal, into
    /// the missile's side) from `t_break`.
    struct Tgt {
        pos: Vec3,
        vel: Vec3,
    }

    fn run(def: &'static MissileDef, launch: Vec3, launch_vel: Vec3, mut tgt: Tgt, g: f32, t_break: f32) -> (f32, bool) {
        let dir = (tgt.pos - launch).normalize();
        let mut m = Missile::new(1, def, 0, Some(1), launch, launch_vel, dir, Some((tgt.pos, tgt.vel)));
        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        let mut min = f32::MAX;
        let mut seed = 1u32;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed >> 8) as f32 / 16_777_216.0
        };
        let sun = Vec3::new(0.3, -0.5, 0.2).normalize();
        while t < def.life {
            let t0 = tgt.pos;
            if t > t_break {
                let side = Vec3::Y.cross(tgt.vel).normalize();
                let side = if side.dot(m.pos - tgt.pos) > 0.0 { side } else { -side };
                let a = side * g * GRAVITY;
                tgt.vel += a * dt;
                tgt.vel = tgt.vel.normalize() * 250.0;
            }
            tgt.pos += tgt.vel * dt;
            let fwd = tgt.vel.normalize();
            let c = [Contact {
                src: Source::Unit(1),
                pos: tgt.pos,
                vel: tgt.vel,
                strength: if matches!(def.seeker, SeekerKind::Ir { .. }) { ir_intensity(tgt.pos, fwd, m.pos, true, 1.0, 0.0) } else { 5.0 },
                age: 9.0,
            }];
            if m.datalink {
                m.uplink(tgt.pos, tgt.vel);
            }
            let p0 = m.pos;
            m.step(&c, sun, &mut rng, dt);
            let (d, _) = closest_approach(p0, m.pos, t0, tgt.pos);
            min = min.min(d);
            if d < def.fuze {
                return (d, true);
            }
            if m.spent() || (min < 200.0 && d > min + 300.0) {
                break;
            }
            t += dt;
        }
        (min, false)
    }

    #[test]
    fn aim9_hits_a_6g_breaking_target_inside_the_envelope_and_misses_outside() {
        // Rear quarter, 2.5 km, both at 250 m/s, 5 km altitude: the target breaks at 6 g on launch.
        let alt = 5000.0;
        let tgt = Tgt { pos: Vec3::new(300.0, alt, -2500.0), vel: Vec3::new(0.0, 0.0, -250.0) };
        let (miss, hit) = run(&AIM9M, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -250.0), tgt, 6.0, 0.0);
        eprintln!("AIM-9M 2.5 km rear quarter vs 6 g break: miss {miss:.1} m, hit {hit}");
        assert!(hit, "miss {miss}");
        // Head-on at 6 km, 6 g break: still inside the envelope.
        let tgt = Tgt { pos: Vec3::new(0.0, alt, -6000.0), vel: Vec3::new(0.0, 0.0, 250.0) };
        let (miss, hit) = run(&AIM9M, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -250.0), tgt, 6.0, 2.0);
        eprintln!("AIM-9M 6 km head-on vs 6 g break: miss {miss:.1} m, hit {hit}");
        assert!(hit, "miss {miss}");
        // Outside: a tail chase at 14 km against a target running away at the same speed.
        let tgt = Tgt { pos: Vec3::new(0.0, alt, -14_000.0), vel: Vec3::new(0.0, 0.0, -250.0) };
        let (miss, hit) = run(&AIM9M, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -250.0), tgt, 6.0, 0.0);
        eprintln!("AIM-9M 14 km tail chase: closest {miss:.0} m, hit {hit}");
        assert!(!hit);
        // Too close: inside minimum range the missile can't turn (2 km abeam the launcher, 6 g).
        let tgt = Tgt { pos: Vec3::new(1200.0, alt, -300.0), vel: Vec3::new(250.0, 0.0, 0.0) };
        let (_, hit) = run(&AIM9M, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -250.0), tgt, 6.0, 0.0);
        assert!(!hit);
    }

    #[test]
    fn aim120_long_shot_hits_with_datalink() {
        let alt = 8000.0;
        let tgt = Tgt { pos: Vec3::new(2000.0, alt, -30_000.0), vel: Vec3::new(0.0, 0.0, 250.0) };
        let (miss, hit) = run(&AIM120C, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -280.0), tgt, 6.0, 30.0);
        eprintln!("AIM-120C 30 km head-on, 6 g break in the terminal phase: miss {miss:.1} m, hit {hit}");
        assert!(hit, "miss {miss}");
        let tgt = Tgt { pos: Vec3::new(0.0, alt, -40_000.0), vel: Vec3::new(0.0, 0.0, -280.0) };
        let (_, hit) = run(&AIM120C, Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -280.0), tgt, 0.0, 99.0);
        assert!(!hit, "40 km tail chase is outside the envelope");
    }

    #[test]
    fn aim9_motor_burnout_speed() {
        // Fired at Mach 0.8 at 5 km: ≈ Mach 2.5–3 at burnout (published "Mach 2.5+").
        let mut m = Missile::new(1, &AIM9M, 0, None, Vec3::new(0.0, 5000.0, 0.0), Vec3::new(0.0, 0.0, -256.0), Vec3::NEG_Z, None);
        let mut r = || 0.5;
        while m.burning() {
            m.step(&[], Vec3::Y, &mut r, 1.0 / 120.0);
        }
        let mach = m.speed() / crate::flight::isa(5000.0).speed_of_sound;
        eprintln!("AIM-9M burnout: Mach {mach:.2}, {:.0} m/s, mass {:.1} kg", m.speed(), m.mass());
        assert!((2.2..3.4).contains(&mach), "{mach}");
        assert!((m.mass() - 59.0).abs() < 0.5);
    }

    /// Flares against the AIM-9M from the rear quarter: a single flare
    /// decoys a minority of shots, a salvo of four most of them; the
    /// non-IRCCM seeker is decoyed far more often.
    #[test]
    fn flare_decoy_probability() {
        fn trial(seed: u32, flares: usize, irccm: f32) -> bool {
            let def: &'static MissileDef = Box::leak(Box::new(MissileDef { seeker: SeekerKind::Ir { gimbal: 0.7, fov: 0.045, r_ref: 10_000.0, irccm }, ..AIM9M }));
            let alt = 5000.0;
            let mut tp = Vec3::new(150.0, alt, -2200.0);
            let tv = Vec3::new(0.0, 0.0, -230.0);
            let mut m = Missile::new(1, def, 0, Some(1), Vec3::new(0.0, alt, 0.0), Vec3::new(0.0, 0.0, -250.0), (tp - Vec3::new(0.0, alt, 0.0)).normalize(), Some((tp, tv)));
            let mut s = seed.wrapping_mul(2_654_435_761).max(1);
            let mut rng = move || {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                (s >> 8) as f32 / 16_777_216.0
            };
            let dt = 1.0 / 120.0;
            let mut fl: Vec<(Vec3, Vec3, f32)> = Vec::new();
            let mut t = 0.0;
            let mut next = 1.0;
            let sun = Vec3::new(0.0, -1.0, 0.0);
            while t < 20.0 {
                if fl.len() < flares && t >= next {
                    fl.push((tp, tv * 0.9 + Vec3::new(0.0, -15.0, 0.0), 0.0));
                    next += 0.35;
                }
                let t0 = tp;
                tp += tv * dt;
                let mut c = vec![Contact { src: Source::Unit(1), pos: tp, vel: tv, strength: ir_intensity(tp, Vec3::NEG_Z, m.pos, true, 1.0, 0.0), age: 9.0 }];
                for (k, f) in fl.iter_mut().enumerate() {
                    f.1 += (Vec3::new(0.0, -9.81, 0.0) - f.1 * 0.9) * dt;
                    f.0 += f.1 * dt;
                    f.2 += dt;
                    c.push(Contact { src: Source::Flare(k as u32), pos: f.0, vel: f.1, strength: flare_intensity(f.2), age: f.2 });
                }
                let p0 = m.pos;
                m.step(&c, sun, &mut rng, dt);
                let (d, _) = closest_approach(p0, m.pos, t0, tp);
                if d < m.def.fuze {
                    return false;
                }
                if m.spent() || (m.pos.z < tp.z - 500.0) {
                    break;
                }
                t += dt;
            }
            true
        }
        let n = 120;
        let p = |flares: usize, irccm: f32| (0..n).filter(|&s| trial(s as u32 + 1, flares, irccm)).count() as f32 / n as f32;
        let (p0, p1, p4, p1_old) = (p(0, 0.55), p(1, 0.55), p(4, 0.55), p(1, 0.0));
        eprintln!("AIM-9M decoy probability: no flares {p0:.2}, 1 flare {p1:.2}, 4 flares {p4:.2}; 1 flare vs a non-IRCCM seeker {p1_old:.2}");
        assert!(p0 < 0.05, "{p0}");
        assert!((0.2..0.65).contains(&p1), "{p1}");
        assert!(p4 > p1 + 0.15 && p4 > 0.6, "{p4}");
        assert!(p1_old > p1 + 0.2, "{p1_old}");
    }
}
