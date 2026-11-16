//! Cockpit eye points measured against the models: rays from each
//! aircraft's eye through its posed geometry (gear down).
//!
//! `cargo test --release eye_audit -- --ignored --nocapture` prints, per
//! aircraft, the clearances up / down / back, the nearest geometry in a cone
//! above the eye (head room), the over-the-nose angle (first opaque surface
//! below the horizon straight ahead within 4 m: the glareshield) and the
//! windscreen-top angle (first opaque surface above it). `SKAC_AUDIT=<id>`
//! limits it to one aircraft and lists what each ray hits.

use crate::aircraft_asset::{AircraftAsset, AnimState, Pose};
use crate::flight;
use glam::Vec3;

/// Möller–Trumbore: distance along `d` (unit) from `o` to the triangle, if hit.
fn ray_tri(o: Vec3, d: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    let (e1, e2) = (b - a, c - a);
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-9 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > 1e-4).then_some(t)
}

/// One triangle of the posed model (body frame): corners, glass, node.
type Tri = ([Vec3; 3], bool, usize);

fn posed_tris(a: &AircraftAsset) -> Vec<Tri> {
    let mut ch = vec![0.0f32; a.channels.len()];
    for (i, c) in a.channels.iter().enumerate() {
        if c == "gear_pos" {
            ch[i] = 1.0;
        }
    }
    let mut pose = Pose::default();
    a.pose(&ch, &mut AnimState::default(), 0.0, &mut pose);
    let mut out = Vec::new();
    for c in a.chunks.iter().filter(|c| pose.visible[c.node]) {
        let glass = a.materials[c.material].translucent;
        let m = pose.world[c.node];
        for t in (c.first..c.first + c.count).step_by(3) {
            let p = |k: u32| m.transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos));
            out.push(([p(t), p(t + 1), p(t + 2)], glass, c.node));
        }
    }
    out
}

/// Every hit along a ray: (distance, glass, node), nearest first.
fn ray_hits(tris: &[Tri], o: Vec3, d: Vec3) -> Vec<(f32, bool, usize)> {
    let mut h: Vec<(f32, bool, usize)> = tris.iter().filter_map(|(t, g, n)| ray_tri(o, d, t[0], t[1], t[2]).map(|x| (x, *g, *n))).collect();
    h.sort_by(|a, b| a.0.total_cmp(&b.0));
    h
}

/// Head room above the eye (nearest surface in a 40° cone about the
/// vertical, m): the top of the head is ~0.11 m above the eyes.
const HEAD_ROOM: f32 = 0.15;
/// Clear behind the eye (m): the back of the head is ~0.10 m behind them.
const HEAD_BACK: f32 = 0.12;
/// Nothing nearer than this in any direction (m).
const CLEAR: f32 = 0.08;

/// The triangles that can matter within `r` of `p` (conservative).
fn near(tris: &[Tri], p: Vec3, r: f32) -> Vec<Tri> {
    tris.iter()
        .filter(|(t, _, _)| {
            let lo = t[0].min(t[1]).min(t[2]);
            let hi = t[0].max(t[1]).max(t[2]);
            p.clamp(lo, hi).distance(p) < r
        })
        .copied()
        .collect()
}

/// Measurements of one eye point.
#[derive(Debug)]
struct EyeFit {
    /// Nearest surface straight up / in a 40° cone about the vertical, m.
    up: f32,
    cone: f32,
    /// Nearest surface straight down / behind, m.
    down: f32,
    back: f32,
    /// Nearest surface in any direction, m.
    all: f32,
}

impl EyeFit {
    fn ok(&self) -> bool {
        self.cone >= HEAD_ROOM && self.back >= HEAD_BACK && self.all >= CLEAR
    }
}

fn measure(tris: &[Tri], eye: Vec3) -> EyeFit {
    let first = |d: Vec3| ray_hits(tris, eye, d).first().map(|h| h.0).unwrap_or(f32::INFINITY);
    let mut cone = f32::INFINITY;
    for i in 0..12 {
        for k in 1..=4 {
            let el = (90.0 - 10.0 * k as f32).to_radians();
            let az = (30.0 * i as f32).to_radians();
            cone = cone.min(first(Vec3::new(el.cos() * az.sin(), el.sin(), el.cos() * az.cos())));
        }
    }
    let up = first(Vec3::Y);
    let mut all = cone.min(up);
    for i in 0..16 {
        for k in -3..=3 {
            let el = (25.0 * k as f32).to_radians();
            let az = (22.5 * i as f32).to_radians();
            all = all.min(first(Vec3::new(el.cos() * az.sin(), el.sin(), el.cos() * az.cos())));
        }
    }
    let down = first(-Vec3::Y);
    EyeFit { up, cone, down, back: first(-Vec3::Z), all: all.min(down) }
}

/// Over-the-nose angle (deg below the horizon) and windscreen top (deg
/// above it): straight ahead, the first opaque surface within 4 m.
fn view_angles(tris: &[Tri], eye: Vec3) -> (f32, f32) {
    let opaque_at = |deg: f32| {
        let r = deg.to_radians();
        ray_hits(tris, eye, Vec3::new(0.0, r.sin(), r.cos())).iter().find(|h| !h.1).is_some_and(|h| h.0 < 4.0)
    };
    ((0..=60).map(|k| -(k as f32)).find(|&d| opaque_at(d)).unwrap_or(f32::NAN), (0..=80).map(|k| k as f32).find(|&d| opaque_at(d)).unwrap_or(f32::NAN))
}

/// The smallest move of the eye (down, then forward; 1 cm steps, up to
/// 15 cm each) that clears the head room / head rest, if any.
fn fit(tris: &[Tri], eye: Vec3) -> Option<Vec3> {
    let mut best: Option<(f32, Vec3)> = None;
    for dy in 0..=15 {
        for dz in 0..=15 {
            let d = Vec3::new(0.0, -0.01 * dy as f32, 0.01 * dz as f32);
            let cost = (dy + dz) as f32;
            if best.is_some_and(|b| b.0 <= cost) {
                continue;
            }
            if measure(tris, eye + d).ok() {
                best = Some((cost, d));
            }
        }
    }
    best.map(|b| b.1)
}

/// `cargo test --release eye_audit -- --ignored --nocapture`.
#[test]
#[ignore]
fn eye_audit() {
    let only = std::env::var("SKAC_AUDIT").ok();
    println!("{:<12} {:>22} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}  fix", "id", "eye", "up", "cone", "down", "back", "all", "nose°", "top°");
    for def in flight::catalogue() {
        if only.as_deref().is_some_and(|o| o != def.id) {
            continue;
        }
        let Ok(a) = AircraftAsset::load(def.id) else { continue };
        let mut eye = crate::models::eye_point(&a, def.eye);
        // SKAC_EYE_D="dx,dy,dz": try a moved eye (as SKYWARD_EYE in the app).
        if let Some(v) = std::env::var("SKAC_EYE_D").ok().map(|s| s.split(',').filter_map(|x| x.trim().parse::<f32>().ok()).collect::<Vec<_>>()).filter(|v| v.len() == 3) {
            eye += Vec3::new(v[0], v[1], v[2]);
        }
        let tris = near(&posed_tris(&a), eye, 4.5);
        let close = near(&tris, eye, 0.6);
        let f = measure(&tris, eye);
        let (nose, top) = view_angles(&tris, eye);
        let fix = if f.ok() { "ok".to_string() } else { fit(&close, eye).map(|d| format!("{d:.2?}")).unwrap_or_else(|| "none".into()) };
        println!(
            "{:<12} ({:6.3},{:6.3},{:6.3}) {:6.2} {:6.2} {:6.2} {:6.2} {:6.2} {:6.0} {:6.0}  {fix}",
            def.id, eye.x, eye.y, eye.z, f.up, f.cone, f.down, f.back, f.all, nose, top
        );
        if only.is_some() {
            for (n, d) in [("up", Vec3::Y), ("down", -Vec3::Y), ("fwd", Vec3::Z), ("back", -Vec3::Z)] {
                for h in ray_hits(&tris, eye, d).iter().take(6) {
                    println!("  {n:<5} {:5.2} {} {}", h.0, if h.1 { "glass" } else { "     " }, a.nodes[h.2].name);
                }
            }
        }
    }
}

/// Every aircraft's eye clears its model: head room above, the head rest
/// behind, nothing at the near plane. (A new import / eye failing this:
/// `eye_audit` above prints the smallest move that fixes it.)
#[test]
fn eye_points_clear_the_model() {
    let mut bad = Vec::new();
    for def in flight::catalogue() {
        let Ok(a) = AircraftAsset::load(def.id) else { continue };
        if def.rotor.as_ref().is_some_and(|r| r.quad().is_some()) {
            continue; // the FPV quad's camera sits in its frame
        }
        let eye = crate::models::eye_point(&a, def.eye);
        let f = measure(&near(&posed_tris(&a), eye, 0.6), eye);
        if !f.ok() {
            bad.push(format!("{} {eye:?}: {f:?}", def.id));
        }
    }
    assert!(bad.is_empty(), "eye points inside the model:\n{}", bad.join("\n"));
}

/// Blended (glass) chunks per aircraft: node path, animations, opacity.
/// `cargo test --release glass_dump -- --ignored --nocapture` (SKAC_AUDIT=<id> for one).
#[test]
#[ignore]
fn glass_dump() {
    use crate::aircraft_asset::Anim;
    let only = std::env::var("SKAC_AUDIT").ok();
    for def in flight::catalogue() {
        if only.as_deref().is_some_and(|o| o != def.id) {
            continue;
        }
        let Ok(a) = AircraftAsset::load(def.id) else { continue };
        for c in a.chunks.iter().filter(|c| a.materials[c.material].translucent) {
            let mut path = Vec::new();
            let mut i = c.node as i32;
            while i >= 0 {
                let n = &a.nodes[i as usize];
                let k = match n.anim {
                    Some(Anim::Spin { .. }) => "[spin]",
                    Some(Anim::Rotate { .. }) => "[rot]",
                    Some(Anim::Select(_)) => "[sel]",
                    Some(Anim::Translate { .. }) => "[tr]",
                    Some(Anim::Scale { .. }) => "[scale]",
                    None => "",
                };
                path.push(format!("{}{k}", n.name));
                i = n.parent;
            }
            let m = &a.materials[c.material];
            let int = if c.interior { "int" } else { "ext" };
            println!("{} {int} a={:.2} tris={} {}", def.id, m.col.w, c.count / 3, path.join(" < "));
        }
    }
}
