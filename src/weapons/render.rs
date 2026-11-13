//! What the fight looks like: the other aircraft (textured models with
//! their animations, loose parts and wrecks), tracers (emissive streaks
//! sized to stay visible at range), missiles, parachutes, ejection seats,
//! jettisoned canopies and the shot holes in every airframe. Effects that
//! are particles (muzzle flashes, smoke, sparks, fireballs, trails) are
//! emitted by `vfx/weapons.rs` from [`super::Combat::fx`].

use super::{Combat, Life};
use crate::scene::{MeshData, mat};
use crate::vfx::GpuParticle;
use glam::{Mat4, Quat, Vec3};

/// sRGB-encode a linear colour (the scene shader's dynamic-mesh convention).
fn srgb(c: [f32; 3]) -> [f32; 3] {
    [c[0].max(0.0).powf(1.0 / 2.2), c[1].max(0.0).powf(1.0 / 2.2), c[2].max(0.0).powf(1.0 / 2.2)]
}

/// A capped cylinder along +Z from z0 to z1 (local), radius r.
fn cylinder(m: &mut MeshData, xf: Mat4, z0: f32, z1: f32, r: f32, col: [f32; 3], n: u32) {
    let base = m.verts.len() as u32;
    for i in 0..n {
        let a = std::f32::consts::TAU * i as f32 / n as f32;
        let (s, c) = a.sin_cos();
        let nrm = xf.transform_vector3(Vec3::new(c, s, 0.0)).normalize_or_zero();
        m.v(xf.transform_point3(Vec3::new(c * r, s * r, z0)), nrm, col);
        m.v(xf.transform_point3(Vec3::new(c * r, s * r, z1)), nrm, col);
    }
    for i in 0..n {
        let a = base + i * 2;
        let b = base + ((i + 1) % n) * 2;
        m.idx.extend([a, b, a + 1, b, b + 1, a + 1]);
    }
}

/// A cone from a ring at z0 (radius r) to a point at z1.
fn cone(m: &mut MeshData, xf: Mat4, z0: f32, z1: f32, r: f32, col: [f32; 3], n: u32) {
    let base = m.verts.len() as u32;
    let tip = xf.transform_point3(Vec3::new(0.0, 0.0, z1));
    for i in 0..n {
        let a = std::f32::consts::TAU * i as f32 / n as f32;
        let (s, c) = a.sin_cos();
        let nrm = xf.transform_vector3(Vec3::new(c, s, r / (z1 - z0).abs().max(0.01))).normalize_or_zero();
        m.v(xf.transform_point3(Vec3::new(c * r, s * r, z0)), nrm, col);
        m.v(tip, nrm, col);
    }
    for i in 0..n {
        let a = base + i * 2;
        let b = base + ((i + 1) % n) * 2;
        m.idx.extend([a, b, a + 1]);
    }
}

/// A thin flat fin (both faces) in the local plane given by `u` (span) × Z.
fn fin(m: &mut MeshData, xf: Mat4, u: Vec3, z0: f32, z1: f32, span: f32, col: [f32; 3]) {
    let p = |q: Vec3| xf.transform_point3(q);
    let a = p(Vec3::new(0.0, 0.0, z0));
    let b = p(Vec3::new(0.0, 0.0, z1));
    let c = p(u * span + Vec3::new(0.0, 0.0, z1 - (z1 - z0) * 0.25));
    let d = p(u * span + Vec3::new(0.0, 0.0, z0 + (z1 - z0) * 0.2));
    let n = (b - a).cross(c - a).normalize_or_zero();
    m.quad_n(a, b, c, d, n, col);
    m.quad_n(a, d, c, b, -n, col);
}

/// Missile body (eye-relative transform; +Z along the velocity).
fn missile_mesh(m: &mut MeshData, xf: Mat4, aim120: bool, burning: bool, t: f32) {
    let (len, r) = if aim120 { (3.66, 0.089) } else { (2.87, 0.0635) };
    let white = srgb([0.62, 0.63, 0.62]);
    let grey = srgb([0.30, 0.31, 0.31]);
    m.mat = mat::PAINT;
    let z0 = -len * 0.5;
    let z1 = len * 0.5 - if aim120 { 0.45 } else { 0.25 };
    cylinder(m, xf, z0, z1, r, white, 12);
    if aim120 {
        cone(m, xf, z1, len * 0.5, r, white, 12);
    } else {
        // AIM-9: glass seeker dome.
        m.mat = mat::CANOPY;
        cone(m, xf, z1, len * 0.5, r, srgb([0.10, 0.12, 0.14]), 12);
        m.mat = mat::PAINT;
    }
    // Control fins (canards on the AIM-9, mid-body wings + tail fins on the AIM-120).
    for k in 0..4 {
        let a = std::f32::consts::FRAC_PI_4 + k as f32 * std::f32::consts::FRAC_PI_2;
        let u = Vec3::new(a.cos(), a.sin(), 0.0);
        let x = Mat4::from_translation(u * r);
        fin(m, xf * x, u, z0, z0 + 0.3, if aim120 { 0.17 } else { 0.2 }, grey);
        if aim120 {
            fin(m, xf * x, u, -0.2, 0.25, 0.13, grey);
        } else {
            fin(m, xf * x, u, z1 - 0.35, z1 - 0.12, 0.09, grey);
        }
    }
    if burning {
        // Motor flame at the nozzle (emissive, flickering).
        m.mat = mat::EMISSIVE;
        let fl = 1.0 + 0.15 * (t * 53.0).sin();
        let base = m.verts.len();
        cone(m, xf, z0, z0 - 0.9 * fl, r * 0.9, [1.0, 0.72, 0.38], 10);
        for v in &mut m.verts[base..] {
            v.uv = [40.0, 1.0];
        }
        m.mat = mat::PAINT;
    }
}

/// Parachute canopy: a shallow dome of alternating gores plus rigging lines.
fn chute_mesh(m: &mut MeshData, origin: Vec3, rot: Quat, open: f32, t: f32) {
    let r = 0.4 + 3.3 * open;
    let h = 4.6 * open.max(0.15) + 1.0;
    let breathe = 1.0 + 0.03 * (t * 2.1).sin();
    let up = rot * Vec3::Y;
    let c = origin + up * h;
    let gores = 16;
    let rings = 4;
    let orange = srgb([0.85, 0.32, 0.06]);
    let white = srgb([0.86, 0.86, 0.82]);
    m.mat = mat::DIFFUSE;
    let x = rot * Vec3::X;
    let z = rot * Vec3::Z;
    for g in 0..gores {
        let col = if g % 2 == 0 { orange } else { white };
        for j in 0..rings {
            let p = |gi: u32, ji: u32| {
                let a = std::f32::consts::TAU * gi as f32 / gores as f32;
                let th = (ji as f32 / rings as f32) * 1.25;
                let rr = r * breathe * th.sin() / 1.25f32.sin();
                c + (x * a.cos() + z * a.sin()) * rr + up * (r * 0.55 * th.cos() - r * 0.55 * 1.25f32.cos())
            };
            let (a, b, cc, d) = (p(g, j), p(g + 1, j), p(g + 1, j + 1), p(g, j + 1));
            let n = (b - a).cross(d - a).normalize_or(up);
            let n = if n.dot(up) < 0.0 { -n } else { n };
            m.quad_n(a, b, cc, d, n, col);
            m.quad_n(a, d, cc, b, -n, col);
        }
    }
    // Rigging lines (thin dark quads from the skirt to the harness).
    let line = srgb([0.15, 0.15, 0.14]);
    for g in (0..gores).step_by(2) {
        let a = std::f32::consts::TAU * g as f32 / gores as f32;
        let rim = c + (x * a.cos() + z * a.sin()) * r * breathe - up * (r * 0.55 * (1.0 - 1.25f32.cos()));
        let s = (rim - origin).cross(up).normalize_or(x) * 0.012;
        m.quad_n(origin - s, origin + s, rim + s, rim - s, (rim - origin).cross(s).normalize_or(up), line);
    }
}

/// Pilot (flight suit, helmet) in a seated / hanging pose, body frame like the aircraft.
fn pilot_mesh(m: &mut MeshData, xf: Mat4, seated: bool) {
    let suit = srgb([0.22, 0.24, 0.17]);
    let helmet = srgb([0.55, 0.55, 0.52]);
    m.mat = mat::DIFFUSE;
    let mut b = MeshData::default();
    b.blob_lod(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.2, 0.33, 0.14), suit, 8, 5);
    b.blob_lod(Vec3::new(0.0, 0.48, 0.0), Vec3::new(0.12, 0.13, 0.13), helmet, 8, 5);
    if seated {
        b.blob_lod(Vec3::new(0.1, -0.3, 0.25), Vec3::new(0.08, 0.08, 0.28), suit, 6, 4);
        b.blob_lod(Vec3::new(-0.1, -0.3, 0.25), Vec3::new(0.08, 0.08, 0.28), suit, 6, 4);
    } else {
        b.blob_lod(Vec3::new(0.1, -0.62, 0.0), Vec3::new(0.08, 0.32, 0.08), suit, 6, 4);
        b.blob_lod(Vec3::new(-0.1, -0.62, 0.0), Vec3::new(0.08, 0.32, 0.08), suit, 6, 4);
    }
    for v in &mut b.verts {
        v.mat = mat::DIFFUSE;
    }
    m.append(&b, xf);
}

fn seat_mesh(m: &mut MeshData, xf: Mat4, rocket: f32) {
    let grey = srgb([0.12, 0.12, 0.12]);
    let mut b = MeshData::default();
    b.boxm(Vec3::new(0.0, -0.05, -0.22), Vec3::new(0.27, 0.55, 0.06), grey);
    b.boxm(Vec3::new(0.0, -0.5, 0.05), Vec3::new(0.27, 0.06, 0.28), grey);
    for v in &mut b.verts {
        v.mat = mat::METAL;
    }
    m.append(&b, xf);
    if rocket > 0.0 {
        let base = m.verts.len();
        m.mat = mat::EMISSIVE;
        cone(m, xf * Mat4::from_rotation_x(std::f32::consts::FRAC_PI_2), 0.55, 2.2, 0.16, [1.0, 0.78, 0.45], 10);
        for v in &mut m.verts[base..] {
            v.uv = [60.0, 1.0];
        }
    }
}

/// Shot holes on one airframe (body → eye-relative `xf`): dark punctures
/// with a scorched / bare-metal ring.
fn holes_mesh(m: &mut MeshData, xf: Mat4, holes: &[super::bdmg::Hole]) {
    m.mat = mat::DIFFUSE;
    for h in holes {
        let n = h.normal.normalize_or(Vec3::Y);
        let t = n.any_orthonormal_vector();
        let b = n.cross(t);
        let c = h.pos + n * 0.015;
        for (r, col) in [(h.size * 1.9, srgb([0.10, 0.09, 0.08])), (h.size, srgb([0.01, 0.01, 0.01]))] {
            let k = 8u32;
            let base = m.verts.len() as u32;
            let nw = xf.transform_vector3(n).normalize_or_zero();
            m.v(xf.transform_point3(c + n * 0.001 * (r < h.size * 1.5) as u32 as f32), nw, col);
            for i in 0..k {
                let a = std::f32::consts::TAU * i as f32 / k as f32;
                let jag = 0.75 + 0.5 * ((i as usize * 7 + (h.pos.x.abs() * 97.0) as usize) % 5) as f32 / 4.0;
                m.v(xf.transform_point3(c + (t * a.cos() + b * a.sin()) * r * jag), nw, col);
            }
            for i in 0..k {
                m.idx.extend([base, base + 1 + i, base + 1 + (i + 1) % k]);
            }
        }
    }
}

/// Eye-relative dynamic geometry for the fight. `player`: the local aircraft's pose (holes).
pub fn meshes(c: &Combat, eye: Vec3, time: f32, player: Option<(Vec3, Quat)>, out: &mut MeshData) {
    let mut m = MeshData::default();
    for ms in &c.missiles {
        let dir = ms.dir();
        let rot = Quat::from_rotation_arc(Vec3::Z, dir);
        let xf = Mat4::from_rotation_translation(rot, ms.pos - eye);
        missile_mesh(&mut m, xf, ms.def.kind == super::missiles::MissileKind::Aim120, ms.burning(), time);
    }
    // Missiles still on the rails.
    for u in &c.units {
        let pose = match (&u.ai, Some(u.id) == c.local) {
            (Some(a), _) => Some((a.ac.pos, a.ac.rot)),
            (None, true) => player,
            _ => None,
        };
        let Some((pos, rot)) = pose else { continue };
        if !matches!(u.life, Life::Dead(_)) {
            let span = c.defs[u.def].wingspan * 0.5;
            for (kind, n) in &u.missiles {
                for k in 0..*n {
                    let side = if (k + 1) % 2 == 0 { 1.0 } else { -1.0 };
                    let st = match kind {
                        super::missiles::MissileKind::Aim9 => Vec3::new(side * span, -0.1, -0.5),
                        super::missiles::MissileKind::Aim120 => Vec3::new(side * (span - 1.2), -0.45, -0.2),
                    };
                    let xf = Mat4::from_rotation_translation(rot, pos - eye) * Mat4::from_translation(st);
                    missile_mesh(&mut m, xf, *kind == super::missiles::MissileKind::Aim120, false, time);
                }
            }
        }
        if !u.bd.holes.is_empty() && pos.distance(eye) < 900.0 {
            holes_mesh(&mut m, Mat4::from_rotation_translation(rot, pos - eye), &u.bd.holes);
        }
    }
    for ch in &c.chutes {
        let xf = Mat4::from_rotation_translation(ch.rot, ch.pos - eye);
        if ch.seat_attached {
            seat_mesh(&mut m, xf, ch.rocket);
            pilot_mesh(&mut m, xf * Mat4::from_translation(Vec3::new(0.0, 0.05, 0.0)), true);
        } else {
            pilot_mesh(&mut m, xf, ch.stage == super::eject::Stage::Fall);
            if ch.seat {
                seat_mesh(&mut m, Mat4::from_rotation_translation(ch.seat_rot, ch.seat_pos - eye), 0.0);
            }
        }
        if ch.stage == super::eject::Stage::Canopy || ch.stage == super::eject::Stage::Landed {
            let open = if ch.stage == super::eject::Stage::Landed { (1.0 - ch.landed_t / 3.0).clamp(0.05, 1.0) } else { ch.open };
            chute_mesh(&mut m, ch.pos - eye + Vec3::Y * 0.4, Quat::IDENTITY, open, time);
        }
    }
    for cn in &c.canopies {
        let xf = Mat4::from_rotation_translation(cn.rot, cn.pos - eye);
        let mut b = MeshData::default();
        b.blob_lod(Vec3::ZERO, Vec3::new(0.42, 0.32, 0.9), srgb([0.18, 0.2, 0.22]), 10, 5);
        for v in &mut b.verts {
            v.mat = mat::CANOPY;
        }
        m.append(&b, xf);
    }
    out.append(&m, Mat4::IDENTITY);
}

/// Tracers and burning flares as emissive streak particles (relative to `origin`).
pub fn particles(c: &Combat, origin: Vec3, eye: Vec3, out: &mut Vec<GpuParticle>) {
    // Pixel angle at 1080 lines / 60°: keep a streak at least ~1.5 px wide.
    let px = 1.0f32.to_radians();
    for r in c.rounds.iter().filter(|r| r.tracer) {
        let d = r.pos.distance(eye).max(1.0);
        // Burn-out after ~3.5 s / beyond ~2.5 km.
        if r.t > 3.5 {
            continue;
        }
        let size = (0.05f32).max(d * px * 0.03);
        // Energy kept as it grows: dimmer per area, so distant tracers stay points.
        let k = (0.05 / size).powi(2).max(0.02) * (1.0 - (r.t / 3.5).powi(4));
        let hot = if r.weapon == super::events::WeaponKind::Browning303 { [1.0, 0.36, 0.12] } else { [1.0, 0.52, 0.2] };
        let i = 160.0 * k;
        // Streak along the travel (the shader stretches by velocity): ≈ 1/80 s of motion.
        let v = r.vel * 0.42;
        out.push(GpuParticle {
            pos: [r.pos.x - origin.x, r.pos.y - origin.y, r.pos.z - origin.z, size],
            col: [hot[0] * i, hot[1] * i, hot[2] * i, 1.0],
            vel: [v.x, v.y, v.z, 4.0],
            extra: [0.3, 0.0, (r.t * 13.0).fract() * 97.0, 1.0],
            light: [1.0, 0.0, 0.0, 1.0],
        });
    }
    for x in c.decoys.iter().filter(|x| x.flare) {
        let a = super::missiles::flare_intensity(x.t) / 18_000.0;
        if a <= 0.0 {
            continue;
        }
        let d = x.pos.distance(eye).max(1.0);
        let size = (0.35f32).max(d * px * 0.02);
        let i = 400.0 * a * (0.35 / size).powi(2).max(0.05) * (1.0 + 0.2 * ((x.t * 41.0 + x.id as f32).sin()));
        out.push(GpuParticle {
            pos: [x.pos.x - origin.x, x.pos.y - origin.y, x.pos.z - origin.z, size],
            col: [i, i * 0.93, i * 0.8, 1.0],
            vel: [0.0, 0.0, 0.0, 4.0],
            extra: [0.3, 0.0, x.id as f32, 1.0],
            light: [1.0, 0.0, 0.0, 1.0],
        });
    }
    for ms in c.missiles.iter().filter(|m| m.burning()) {
        // The motor's exit flare (bright at range, where the mesh flame is sub-pixel).
        let p = ms.pos - ms.dir() * (ms.def.length * 0.5 + 0.3);
        let d = p.distance(eye).max(1.0);
        let size = (0.3f32).max(d * px * 0.025);
        let i = 120.0 * (0.3 / size).powi(2).max(0.05);
        out.push(GpuParticle {
            pos: [p.x - origin.x, p.y - origin.y, p.z - origin.z, size],
            col: [i, i * 0.7, i * 0.4, 1.0],
            vel: [0.0, 0.0, 0.0, 4.0],
            extra: [0.3, 0.0, ms.id as f32, 1.0],
            light: [1.0, 0.0, 0.0, 1.0],
        });
    }
}

/// The other aircraft's model frames (textured, animated, with their loose
/// parts), building their models on first use.
pub fn aircraft_frames(c: &mut Combat, eye: Vec3, time: f32, hours: f32, daylight: f32) -> Vec<crate::gfx_aircraft::AircraftFrame> {
    let mut out = Vec::new();
    let defs = c.defs.clone();
    for u in &mut c.units {
        let Some(a) = u.ai.as_deref() else { continue };
        if a.external || matches!(u.life, Life::Dead(s) if s > 30.0) {
            continue;
        }
        let def = &defs[u.def];
        if a.ac.pos.distance(eye) > 25_000.0 {
            continue;
        }
        if u.model.is_none() {
            let mut model = crate::models::build(def.id, def.eye);
            if let Some(asset) = model.asset.clone() {
                let pm = crate::models::classify(&asset, def);
                // Hit volumes from the real model's parts and wing planform.
                let sd = crate::damage::struct_def(def.id);
                u.hm = std::sync::Arc::new(super::hitbox::HitModel::build(def, sd, &pm.geom, super::hitbox::planform_for(def, sd, &pm.planform)));
                model.parts = Some(pm);
            }
            u.model = Some(model);
        }
        let a = u.ai.as_deref().unwrap();
        let model = u.model.as_mut().unwrap();
        let running = a.ac.engine_running(0);
        model.update(&crate::models::ModelInputs {
            def,
            ac: &a.ac,
            ctl: &a.ctl,
            time,
            nav: true,
            beacon: running,
            strobe: true,
            landing: false,
            ap: None,
            qnh_hpa: 1013.25,
            hours,
            daylight,
            radios: None,
        });
        let body = Mat4::from_rotation_translation(a.ac.rot, a.ac.pos - eye);
        let Some(mut f) = model.frame(body, false) else { continue };
        f.dynamics = Some(Box::new(structure(def, &a.ac, a.debris.as_ref(), body, eye, time)));
        out.push(f);
    }
    out
}

/// Wing bending and loose parts for the aircraft shader (as `Sim::structure_frame`).
fn structure(
    def: &crate::flight::AircraftDef,
    ac: &crate::flight::Aircraft,
    debris: Option<&crate::debris::Debris>,
    body: Mat4,
    eye: Vec3,
    time: f32,
) -> crate::gfx_aircraft::AcDynamics {
    use crate::damage::Part;
    let sd = crate::damage::struct_def(def.id);
    let mut d = crate::gfx_aircraft::AcDynamics::default();
    let up = ac.rot * Vec3::Y;
    d.up = [up.x, up.y, up.z, time];
    let tip = ac.dmg.tip_deflection(sd);
    d.flex = [tip[0], tip[1], sd.twist_per_m, 1.0 / (def.wingspan * 0.5 - sd.fus_r).max(0.5)];
    let inv = body.inverse();
    let lost = ac.dmg.lost;
    for p in Part::ALL {
        let k = p.idx();
        let attached = lost & p.bit() == 0;
        let in_main = debris.and_then(|db| db.main.map(|m| db.bodies[m].parts & p.bit() != 0)).unwrap_or(false);
        d.part_flex[k / 4][k % 4] = if attached && ac.crashed.is_none() {
            1.0
        } else if in_main {
            1.0
        } else {
            0.0
        };
        if let Some((o, r)) = debris.filter(|_| !in_main).and_then(|db| db.part_pose(p)) {
            let w = Mat4::from_rotation_translation(r, o - eye);
            d.parts[k] = (w * inv).to_cols_array_2d();
            d.part_flex[k / 4][k % 4] = 0.0;
        } else if !attached && !in_main {
            d.parts[k] = Mat4::from_scale(Vec3::ZERO).to_cols_array_2d();
        }
    }
    d
}
