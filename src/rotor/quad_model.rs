//! The FPV quad's 3D model, built procedurally (original design, no
//! external assets) as an [`AircraftAsset`] so it renders, animates and
//! breaks up through the same pipeline as the imported aircraft:
//! carbon X frame (arms, plates, standoffs), flight stack, 6S pack with its
//! strap, tilted FPV camera, VTX antenna, rear LEDs, four anodised motors
//! and tri-blade 5" props that spin with the motor channels (`prop{i}_rpm`)
//! and give way to translucent blur discs above a few thousand rpm.

use crate::aircraft_asset::{AircraftAsset, Anim, Chunk, Material, Node, Op, RExpr, Vertex};
use glam::{Mat4, Quat, Vec3, Vec4};
use std::collections::BTreeMap;
use std::f32::consts::TAU;

const ARM: f32 = 0.078;
const PROP_R: f32 = 0.0635;
/// Above this motor speed the blades are drawn as a blur disc.
const BLUR_RPM: f32 = 3500.0;

#[derive(Default)]
struct Builder {
    /// (node, material) → vertices / indices.
    parts: BTreeMap<(usize, usize), (Vec<Vertex>, Vec<u32>)>,
}

impl Builder {
    fn tri(&mut self, node: usize, mat: usize, a: Vec3, b: Vec3, c: Vec3, n: Vec3) {
        let (v, i) = self.parts.entry((node, mat)).or_default();
        let base = v.len() as u32;
        for p in [a, b, c] {
            v.push(Vertex { pos: p.to_array(), nrm: n.to_array(), uv: [0.0, 0.0] });
        }
        i.extend([base, base + 1, base + 2]);
    }
    fn quad(&mut self, node: usize, mat: usize, p: [Vec3; 4], n: Vec3) {
        self.tri(node, mat, p[0], p[1], p[2], n);
        self.tri(node, mat, p[0], p[2], p[3], n);
    }
    /// Oriented box: centre, half extents, rotation.
    fn cuboid(&mut self, node: usize, mat: usize, c: Vec3, h: Vec3, r: Quat) {
        let corner = |sx: f32, sy: f32, sz: f32| c + r * Vec3::new(sx * h.x, sy * h.y, sz * h.z);
        let faces: [(Vec3, [(f32, f32, f32); 4]); 6] = [
            (Vec3::X, [(1., -1., -1.), (1., 1., -1.), (1., 1., 1.), (1., -1., 1.)]),
            (Vec3::NEG_X, [(-1., -1., 1.), (-1., 1., 1.), (-1., 1., -1.), (-1., -1., -1.)]),
            (Vec3::Y, [(-1., 1., -1.), (-1., 1., 1.), (1., 1., 1.), (1., 1., -1.)]),
            (Vec3::NEG_Y, [(-1., -1., 1.), (-1., -1., -1.), (1., -1., -1.), (1., -1., 1.)]),
            (Vec3::Z, [(1., -1., 1.), (1., 1., 1.), (-1., 1., 1.), (-1., -1., 1.)]),
            (Vec3::NEG_Z, [(-1., -1., -1.), (-1., 1., -1.), (1., 1., -1.), (1., -1., -1.)]),
        ];
        for (n, q) in faces {
            let p = q.map(|(x, y, z)| corner(x, y, z));
            self.quad(node, mat, p, r * n);
        }
    }
    /// Cylinder along `axis` (unit) from `base`, radius r, length l, n sides, with caps.
    fn cylinder(&mut self, node: usize, mat: usize, base: Vec3, axis: Vec3, r: f32, l: f32, n: usize) {
        let u = axis.any_orthonormal_vector();
        let w = axis.cross(u);
        let top = base + axis * l;
        for k in 0..n {
            let (a0, a1) = (k as f32 / n as f32 * TAU, (k + 1) as f32 / n as f32 * TAU);
            let d0 = u * a0.cos() + w * a0.sin();
            let d1 = u * a1.cos() + w * a1.sin();
            let nm = (d0 + d1).normalize();
            self.quad(node, mat, [base + d0 * r, base + d1 * r, top + d1 * r, top + d0 * r], nm);
            self.tri(node, mat, top, top + d0 * r, top + d1 * r, axis);
            self.tri(node, mat, base, base + d1 * r, base + d0 * r, -axis);
        }
    }
    /// Flat annulus (both faces) in the XZ plane at height y around `c`.
    fn disc(&mut self, node: usize, mat: usize, c: Vec3, r0: f32, r1: f32, n: usize) {
        for k in 0..n {
            let (a0, a1) = (k as f32 / n as f32 * TAU, (k + 1) as f32 / n as f32 * TAU);
            let p = |a: f32, r: f32| c + Vec3::new(a.cos() * r, 0.0, a.sin() * r);
            self.quad(node, mat, [p(a0, r0), p(a0, r1), p(a1, r1), p(a1, r0)], Vec3::Y);
            self.quad(node, mat, [p(a1, r0), p(a1, r1), p(a0, r1), p(a0, r0)], Vec3::NEG_Y);
        }
    }
    /// One prop blade from the hub along `dir` (XZ plane), twisted, both faces.
    fn blade(&mut self, node: usize, mat: usize, hub: Vec3, dir: Vec3, spin: f32) {
        let side = Vec3::Y.cross(dir) * spin;
        let st = 6;
        let mut prev: Option<[Vec3; 2]> = None;
        for k in 0..=st {
            let t = k as f32 / st as f32;
            let r = 0.008 + t * (PROP_R - 0.008);
            let chord = 0.016 * (1.0 - 0.55 * t) + 0.004;
            let pitch = (0.45 - 0.28 * t) * spin;
            let lead = side * (chord * 0.55) * pitch.cos() + Vec3::Y * (chord * 0.55 * pitch.sin());
            let trail = -side * (chord * 0.45) * pitch.cos() - Vec3::Y * (chord * 0.45 * pitch.sin());
            let c = hub + dir * r;
            let cur = [c + lead, c + trail];
            if let Some(p) = prev {
                let n = (cur[0] - p[0]).cross(p[1] - p[0]).normalize_or(Vec3::Y);
                let n = if n.y < 0.0 { -n } else { n };
                self.quad(node, mat, [p[0], cur[0], cur[1], p[1]], n);
                self.quad(node, mat, [p[1], cur[1], cur[0], p[0]], -n);
            }
            prev = Some(cur);
        }
    }
}

fn mat(col: [f32; 4], spec: f32, shin: f32, translucent: bool) -> Material {
    Material { col: Vec4::from_array(col), emis: Vec3::ZERO, spec: Vec3::splat(spec), shininess: shin, tex: None, translucent, alpha_tex: false, two_sided: translucent }
}

/// Build the quad's model (motor positions and spin senses from the flight model).
pub fn asset(motors: &[Vec3; 4], spin: &[f32; 4], eye: Vec3, cam_tilt: f32) -> AircraftAsset {
    // Materials.
    let carbon = 0;
    let stack = 1;
    let pack = 2;
    let stripe = 3;
    let bell = 4;
    let stator = 5;
    let prop = 6;
    let blur = 7;
    let cam_body = 8;
    let lens = 9;
    let antenna = 10;
    let led = 11;
    let materials = vec![
        mat([0.10, 0.10, 0.11, 1.0], 0.35, 40.0, false), // carbon fibre
        mat([0.12, 0.30, 0.16, 1.0], 0.2, 20.0, false),  // flight-controller stack (green PCB)
        mat([0.08, 0.08, 0.09, 1.0], 0.15, 10.0, false), // battery wrap
        mat([0.95, 0.72, 0.10, 1.0], 0.2, 20.0, false),  // battery stripe / strap
        mat([0.55, 0.20, 0.85, 1.0], 0.8, 90.0, false),  // anodised motor bells
        mat([0.62, 0.62, 0.64, 1.0], 0.9, 100.0, false), // stator / nut
        mat([0.25, 0.85, 0.95, 0.88], 0.5, 60.0, true),  // props (tinted polycarbonate)
        mat([0.30, 0.80, 0.92, 0.18], 0.3, 30.0, true),  // prop blur disc
        mat([0.95, 0.45, 0.10, 1.0], 0.3, 30.0, false),  // FPV camera body
        mat([0.02, 0.02, 0.03, 1.0], 1.0, 120.0, false), // lens
        mat([0.15, 0.15, 0.16, 1.0], 0.2, 20.0, false),  // antenna
        mat([1.0, 0.25, 0.35, 1.0], 0.2, 20.0, false),   // LED bar
    ];
    let mut nodes = vec![Node { name: "frame".into(), parent: -1, pre: Mat4::IDENTITY, anim: None }];
    let mut b = Builder::default();
    let y0 = -0.004;
    // Arms (X), plates, standoffs.
    for m in motors {
        let d = Vec3::new(m.x, 0.0, m.z);
        let len = d.length();
        let rot = Quat::from_rotation_arc(Vec3::Z, d / len);
        b.cuboid(0, carbon, d * 0.5 + Vec3::Y * y0, Vec3::new(0.009, 0.0025, len * 0.5 + 0.012), rot);
        // Motor mount pad.
        b.cylinder(0, carbon, *m + Vec3::Y * (y0 - 0.0025), Vec3::Y, 0.016, 0.005, 16);
    }
    b.cuboid(0, carbon, Vec3::new(0.0, y0, 0.0), Vec3::new(0.022, 0.0025, 0.055), Quat::IDENTITY);
    b.cuboid(0, carbon, Vec3::new(0.0, 0.027, -0.004), Vec3::new(0.022, 0.0015, 0.05), Quat::IDENTITY);
    for (x, z) in [(0.018, 0.04), (-0.018, 0.04), (0.018, -0.045), (-0.018, -0.045)] {
        b.cylinder(0, stator, Vec3::new(x, y0, z), Vec3::Y, 0.0025, 0.03, 8);
    }
    b.cuboid(0, stack, Vec3::new(0.0, 0.010, -0.004), Vec3::new(0.015, 0.008, 0.015), Quat::IDENTITY);
    // 6S pack on top with the strap.
    b.cuboid(0, pack, Vec3::new(0.0, 0.046, -0.01), Vec3::new(0.0175, 0.017, 0.038), Quat::IDENTITY);
    b.cuboid(0, stripe, Vec3::new(0.0, 0.0465, -0.01), Vec3::new(0.0180, 0.0175, 0.006), Quat::IDENTITY);
    b.cuboid(0, stripe, Vec3::new(0.0, 0.0636, -0.03), Vec3::new(0.012, 0.0006, 0.005), Quat::IDENTITY);
    // FPV camera between the plates, tilted up.
    let tilt = Quat::from_rotation_x(-cam_tilt);
    b.cuboid(0, cam_body, eye + Vec3::new(0.0, -0.002, -0.006), Vec3::new(0.0095, 0.0095, 0.009), tilt);
    b.cylinder(0, lens, eye + tilt * Vec3::new(0.0, 0.0, 0.002), tilt * Vec3::Z, 0.0065, 0.007, 14);
    // VTX antenna at the back, angled up.
    let ant = Vec3::new(0.0, 0.03, -0.055);
    let ant_dir = Vec3::new(0.0, 0.8, -0.6).normalize();
    b.cylinder(0, antenna, ant, ant_dir, 0.002, 0.05, 6);
    b.cylinder(0, antenna, ant + ant_dir * 0.05, ant_dir, 0.006, 0.012, 10);
    // Rear LED bar.
    b.cuboid(0, led, Vec3::new(0.0, y0 - 0.003, -0.058), Vec3::new(0.014, 0.0015, 0.002), Quat::IDENTITY);
    // Motors, then per motor: spinning prop node with blades (slow) / blur disc (fast).
    let mut channels = Vec::new();
    for (i, m) in motors.iter().enumerate() {
        b.cylinder(0, stator, *m, Vec3::Y, 0.0125, 0.008, 18);
        b.cylinder(0, bell, *m + Vec3::Y * 0.008, Vec3::Y, 0.014, 0.012, 20);
        b.cylinder(0, stator, *m + Vec3::Y * 0.020, Vec3::Y, 0.0035, 0.008, 8);
        channels.push(format!("prop{i}_rpm"));
        let hub = *m + Vec3::Y * 0.024;
        let ch = RExpr::Ch(i as u16);
        let spin_node = nodes.len();
        nodes.push(Node { name: format!("prop{i}"), parent: 0, pre: Mat4::IDENTITY, anim: Some(Anim::Spin { center: hub, axis: Vec3::Y * spin[i], rpm: ch.clone() }) });
        let slow = nodes.len();
        nodes.push(Node {
            name: format!("prop{i}_blades"),
            parent: spin_node as i32,
            pre: Mat4::IDENTITY,
            anim: Some(Anim::Select(RExpr::Op(Op::Lt, vec![ch.clone(), RExpr::C(BLUR_RPM)]))),
        });
        let fast = nodes.len();
        nodes.push(Node { name: format!("prop{i}_disc"), parent: 0, pre: Mat4::IDENTITY, anim: Some(Anim::Select(RExpr::Op(Op::Ge, vec![ch, RExpr::C(BLUR_RPM)]))) });
        b.cylinder(spin_node, prop, hub - Vec3::Y * 0.002, Vec3::Y, 0.0055, 0.006, 12);
        for k in 0..3 {
            let a = k as f32 / 3.0 * TAU + i as f32 * 0.7;
            b.blade(slow, prop, hub, Vec3::new(a.cos(), 0.0, a.sin()), spin[i]);
        }
        b.disc(fast, blur, hub + Vec3::Y * 0.001, 0.006, PROP_R, 40);
    }
    // Flatten into one vertex / index buffer with a chunk per (node, material).
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut chunks = Vec::new();
    for ((node, material), (v, i)) in b.parts {
        let base = vertices.len() as u32;
        let first = indices.len() as u32;
        vertices.extend(v);
        indices.extend(i);
        let emit = (material == led).then(|| (Vec3::new(1.0, 0.2, 0.3), RExpr::C(1.0)));
        chunks.push(Chunk {
            node,
            material,
            base_vertex: base,
            first,
            count: indices.len() as u32 - first,
            interior: false,
            emit,
            uv: Vec::new(),
            display: None,
            backlight: false,
        });
    }
    let radius = vertices.iter().map(|v| Vec3::from_array(v.pos).length()).fold(0.0, f32::max);
    AircraftAsset {
        id: "fpv".into(),
        name: "Skyward 5\" FPV quad".into(),
        nodes,
        materials,
        textures: Vec::new(),
        chunks,
        vertices,
        indices,
        eye: Some(eye),
        radius,
        lights: Vec::new(),
        channels,
    }
}

/// The model of the catalogue's `fpv` quad.
pub fn default_asset() -> AircraftAsset {
    let q = super::defs::quad_quad();
    let d = super::defs::quad_def();
    asset(&q.motors, &q.spin, d.eye, q.cam_tilt)
}
