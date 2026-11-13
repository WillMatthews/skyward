//! The airport fire service: after an emergency landing (or a crash landing
//! on the airfield) three crash tenders roll from beside the runway to the
//! aircraft, blue lights flashing, and take up positions around it (nose,
//! both sides). Simple vehicle kinematics (accelerate, turn towards the slot,
//! brake to a stop) over the terrain; a procedural low-poly tender drawn
//! into the dynamic mesh.

use crate::flight::Environment;
use crate::scene::{MeshData, Runway, mat};
use glam::{Vec2, Vec3, vec2, vec3};

#[derive(Clone, Debug)]
pub struct Tender {
    pub pos: Vec2,
    pub y: f32,
    pub hdg: f32,
    pub speed: f32,
    pub slot: Vec2,
    /// Seconds before it moves (crews boarding).
    pub delay: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Tenders {
    pub v: Vec<Tender>,
    pub t: f32,
}

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

fn wrap(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

impl Tenders {
    /// Roll out to an aircraft at `ac` from the side of `runway` (or nearby).
    pub fn dispatch(env: &dyn Environment, ac: Vec3, runway: Option<&Runway>) -> Tenders {
        let p = vec2(ac.x, ac.z);
        let (start, axis) = match runway {
            Some(r) => {
                let d = (r.b - r.a).normalize_or(vec2(0.0, -1.0));
                let t = ((p - r.a).dot(d)).clamp(0.0, r.a.distance(r.b));
                let on = r.a + d * t;
                // The fire station is somewhere off the side: start 220 m out, 450 m back.
                let side = vec2(-d.y, d.x);
                let s = if (p - on).dot(side) >= 0.0 { side } else { -side };
                (on + s * 220.0 - d * 450.0, d)
            }
            None => (p + vec2(500.0, 300.0), vec2(0.0, -1.0)),
        };
        let side = vec2(-axis.y, axis.x);
        let slots = [p + axis * 38.0 + side * 12.0, p + side * 30.0, p - side * 30.0];
        let v = slots
            .iter()
            .enumerate()
            .map(|(i, &slot)| {
                let pos = start - axis * (i as f32 * 14.0);
                Tender { pos, y: env.ground_height(pos), hdg: heading_of(slot - pos), speed: 0.0, slot, delay: 2.0 + i as f32 * 2.5 }
            })
            .collect();
        Tenders { v, t: 0.0 }
    }

    pub fn update(&mut self, env: &dyn Environment, _ac: Vec3, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        self.t += dt;
        for k in &mut self.v {
            if k.delay > 0.0 {
                k.delay -= dt;
                continue;
            }
            let to = k.slot - k.pos;
            let dist = to.length();
            let want = if dist < 3.0 { 0.0 } else { (dist * 0.35).clamp(3.0, 22.0) };
            let a = if want > k.speed { 2.5 } else { -4.0 };
            k.speed = (k.speed + a * dt).clamp(0.0, want.max(k.speed + a * dt).min(22.0));
            if dist < 3.0 {
                k.speed = (k.speed - 5.0 * dt).max(0.0);
            }
            if dist > 1.0 {
                let err = wrap(heading_of(to) - k.hdg);
                let rate = (k.speed / 9.0).min(0.6);
                k.hdg = wrap(k.hdg + err.clamp(-rate * dt, rate * dt));
            }
            k.pos += vec2(k.hdg.sin(), -k.hdg.cos()) * k.speed * dt;
            k.y = env.ground_height(k.pos);
        }
    }

    /// Low-poly tenders (eye-relative) into `out`; blue beacons flash.
    pub fn mesh(&self, out: &mut MeshData, eye: Vec3, time: f32) {
        for (i, k) in self.v.iter().enumerate() {
            let o = vec3(k.pos.x, k.y, k.pos.y) - eye;
            let f = vec3(k.hdg.sin(), 0.0, -k.hdg.cos());
            let r = vec3(-f.z, 0.0, f.x);
            let at = |x: f32, y: f32, z: f32| o + r * x + Vec3::Y * y + f * z;
            let red = [0.62, 0.04, 0.03];
            let white = [0.8, 0.8, 0.78];
            let dark = [0.03, 0.03, 0.035];
            let glass = [0.05, 0.07, 0.09];
            out.mat = mat::PAINT;
            boxy(out, &at, (-1.3, 1.3), (0.55, 3.3), (-5.2, 3.0), red);
            boxy(out, &at, (-1.25, 1.25), (0.55, 3.0), (3.0, 4.6), red);
            boxy(out, &at, (-1.32, 1.32), (1.4, 1.65), (-5.25, 4.65), white);
            out.mat = mat::CANOPY;
            boxy(out, &at, (-1.2, 1.2), (2.0, 2.85), (4.55, 4.62), glass);
            out.mat = mat::METAL;
            // Roof monitor (the foam cannon).
            boxy(out, &at, (-0.15, 0.15), (3.3, 3.6), (1.0, 2.6), [0.5, 0.5, 0.5]);
            out.mat = mat::DIFFUSE;
            for z in [-3.6, -1.6, 3.4] {
                for x in [-1.25, 1.25] {
                    boxy(out, &at, (x - 0.25, x + 0.25), (0.0, 1.1), (z - 0.55, z + 0.55), dark);
                }
            }
            // Beacons: alternating blue flashes.
            let ph = time * 5.0 + i as f32 * 1.7;
            for (x, s) in [(-0.9, 0.0), (0.9, 0.5)] {
                let on = ((ph + s).fract()) < 0.35;
                out.mat = mat::EMISSIVE;
                let c = if on { [0.1, 0.25, 1.0] } else { [0.02, 0.03, 0.12] };
                let p0 = at(x - 0.22, 3.0, 4.0);
                for (a, b, c2, d, n) in box_faces(p0, r * 0.44, Vec3::Y * 0.22, f * 0.35) {
                    let i0 = out.v_uv(a, n, c, [if on { 40.0 } else { 1.0 }, 0.0]);
                    out.v_uv(b, n, c, [if on { 40.0 } else { 1.0 }, 0.0]);
                    out.v_uv(c2, n, c, [if on { 40.0 } else { 1.0 }, 0.0]);
                    out.v_uv(d, n, c, [if on { 40.0 } else { 1.0 }, 0.0]);
                    out.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
                }
            }
            out.mat = mat::DIFFUSE;
        }
    }
}

/// The six faces of the box with corner `p` and edge vectors `a`, `b`, `c`: (4 corners, normal).
fn box_faces(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> [(Vec3, Vec3, Vec3, Vec3, Vec3); 6] {
    // (Counter-clockwise outside faces need a right-handed edge basis.)
    let (a, b) = if a.cross(b).dot(c) < 0.0 { (b, a) } else { (a, b) };
    let (na, nb, nc) = (a.normalize_or_zero(), b.normalize_or_zero(), c.normalize_or_zero());
    [
        (p, p + c, p + b + c, p + b, -na),
        (p + a, p + a + b, p + a + b + c, p + a + c, na),
        (p, p + a, p + a + c, p + c, -nb),
        (p + b, p + b + c, p + a + b + c, p + a + b, nb),
        (p, p + b, p + a + b, p + a, -nc),
        (p + c, p + a + c, p + a + b + c, p + b + c, nc),
    ]
}

fn boxy(out: &mut MeshData, at: &dyn Fn(f32, f32, f32) -> Vec3, x: (f32, f32), y: (f32, f32), z: (f32, f32), col: [f32; 3]) {
    let p = at(x.0, y.0, z.0);
    let a = at(x.1, y.0, z.0) - p;
    let b = at(x.0, y.1, z.0) - p;
    let c = at(x.0, y.0, z.1) - p;
    for (q0, q1, q2, q3, n) in box_faces(p, a, b, c) {
        out.quad_n(q0, q1, q2, q3, n, col);
    }
}
