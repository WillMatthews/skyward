//! Procedural road-vehicle meshes, local frame +Z forward, +X left, ground
//! at y = 0, centred on the footprint. Two levels of detail per kind:
//! LOD 0 (body with sloped screens and bonnet, glasshouse, wheels, bumpers,
//! lamps, grille) and LOD 1 (a few boxes).
//!
//! Special vertex colours / uvs read by traffic.wgsl:
//! * colour (-1, -1, -1): body paint (the instance colour);
//! * colour (-2, -2, -2): box body / trailer (white, grey or a livery by seed);
//! * `mat::EMISSIVE` with uv.y = 1 headlamp, 2 tail lamp (brighter when
//!   braking), 3 amber roof sign (taxis): intensity set per frame.

use super::vehicle::{KINDS, kind, spec};
use crate::airport::mesh::{obox, wheel};
use crate::scene::{MeshData, mat};
use glam::{Vec3, vec3};

const PAINT: [f32; 3] = [-1.0, -1.0, -1.0];
const BOX: [f32; 3] = [-2.0, -2.0, -2.0];
const GLASS: [f32; 3] = [0.05, 0.06, 0.07];
const TRIM: [f32; 3] = [0.07, 0.07, 0.075];
const GRILLE: [f32; 3] = [0.03, 0.03, 0.035];
const CHASSIS: [f32; 3] = [0.06, 0.06, 0.06];
const LAMP_HEAD: [f32; 3] = [1.0, 0.97, 0.9];
const LAMP_TAIL: [f32; 3] = [0.9, 0.05, 0.03];

const AX: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

/// Hexahedron from 8 corners: bottom (back-right, back-left, front-left,
/// front-right), then the top in the same order. Flat-shaded faces.
fn hexa(m: &mut MeshData, c: [Vec3; 8], col: [f32; 3]) {
    let centre = c.iter().copied().sum::<Vec3>() / 8.0;
    let faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 3, 7, 4], [1, 5, 6, 2], [3, 2, 6, 7], [0, 4, 5, 1]];
    for f in faces {
        let (a, b, cc, d) = (c[f[0]], c[f[1]], c[f[2]], c[f[3]]);
        let mut n = (cc - a).cross(d - b).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        if n.dot((a + b + cc + d) * 0.25 - centre) < 0.0 {
            n = -n;
        }
        m.quad_n(a, b, cc, d, n, col);
    }
}

/// A slab with independent bottom / top extents along z and widths:
/// bottom from z0..z1 at y0 (half width w0), top from z2..z3 at y1 (half width w1).
#[allow(clippy::too_many_arguments)]
fn slab(m: &mut MeshData, y0: f32, y1: f32, z0: f32, z1: f32, z2: f32, z3: f32, w0: f32, w1: f32, col: [f32; 3]) {
    hexa(m, [vec3(-w0, y0, z0), vec3(w0, y0, z0), vec3(w0, y0, z1), vec3(-w0, y0, z1), vec3(-w1, y1, z2), vec3(w1, y1, z2), vec3(w1, y1, z3), vec3(-w1, y1, z3)], col);
}

/// Lamp lens: a small emissive quad facing +Z (front) or -Z (back).
fn lamp(m: &mut MeshData, c: Vec3, hw: f32, hh: f32, front: bool, tag: f32) {
    let k = m.mat;
    m.mat = mat::EMISSIVE;
    let n = if front { Vec3::Z } else { -Vec3::Z };
    let col = if tag < 1.5 {
        LAMP_HEAD
    } else if tag < 2.5 {
        LAMP_TAIL
    } else {
        [1.0, 0.65, 0.1]
    };
    let p = |x: f32, y: f32| c + vec3(x, y, 0.0);
    let i = m.v_uv(p(-hw, -hh), n, col, [0.0, tag]);
    m.v_uv(p(hw, -hh), n, col, [0.0, tag]);
    m.v_uv(p(hw, hh), n, col, [0.0, tag]);
    m.v_uv(p(-hw, hh), n, col, [0.0, tag]);
    m.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    m.mat = k;
}

fn lamps(m: &mut MeshData, k: u8, z_front: f32, z_back: f32) {
    let sp = spec(k);
    let (hy, hx) = sp.head;
    let (ty, tx) = sp.tail;
    let big = sp.len > 6.0;
    let (hw, hh) = if big { (0.16, 0.09) } else { (0.14, 0.06) };
    if hx == 0.0 {
        lamp(m, vec3(0.0, hy, z_front + 0.01), 0.09, 0.08, true, 1.0);
        lamp(m, vec3(0.0, ty, z_back - 0.01), 0.07, 0.04, false, 2.0);
        return;
    }
    for s in [-1.0f32, 1.0] {
        lamp(m, vec3(s * hx, hy, z_front + 0.01), hw, hh, true, 1.0);
        lamp(m, vec3(s * tx, ty, z_back - 0.01), if big { 0.1 } else { 0.13 }, if big { 0.14 } else { 0.06 }, false, 2.0);
    }
}

fn wheels(m: &mut MeshData, zs: &[f32], half_track: f32, r: f32, w: f32) {
    let k = m.mat;
    for &z in zs {
        for s in [-1.0f32, 1.0] {
            wheel(m, vec3(s * half_track, r, z), Vec3::X, r, w);
        }
    }
    m.mat = k;
}

/// Car body lofted through cross-sections along z: a side profile (bumpers,
/// bonnet, windscreen, roof, rear screen, boot) with tumblehome above the
/// beltline and rounded corners in plan. Glass where the profile is a
/// window. `fine` 1 = detailed (more sections), 0 = LOD 1.
#[allow(clippy::too_many_arguments)]
fn loft_car(m: &mut MeshData, l: f32, hw: f32, belt: f32, roof: f32, gb: f32, gf: f32, tb: f32, tf: f32, clear: f32, fine: f32) {
    // (z, top height, top glass) of the profile; the roof is flat between tb and tf.
    let boot = belt - 0.03;
    let nose = belt - 0.2;
    let tail = (boot - 0.06).max(clear + 0.45);
    let prof: Vec<(f32, f32, bool)> = if fine > 0.5 {
        vec![
            (-l, tail, false),
            (-l + 0.12, boot - 0.01, false),
            (gb - 0.02, boot, false),
            (gb, belt, false),
            (tb, roof - 0.03, true),
            (tb + 0.1, roof, false),
            ((tb + tf) * 0.5, roof + 0.012, false),
            (tf - 0.1, roof, false),
            (tf, roof - 0.03, false),
            (gf, belt, true),
            (gf + 0.02, belt - 0.005, false),
            ((gf + l) * 0.5, (belt + nose) * 0.5 + 0.03, false),
            (l - 0.14, nose + 0.01, false),
            (l, nose - 0.08, false),
        ]
    } else {
        vec![(-l, tail, false), (gb, belt, false), (tb, roof, true), (tf, roof, false), (gf, belt, true), (l, nose - 0.04, false)]
    };
    let hw_at = |z: f32| {
        // Rounded corners in plan.
        let e = (l - z.abs()).max(0.0);
        let r = 0.32;
        if e >= r { hw } else { hw - (r - (r * r - (r - e) * (r - e)).max(0.0).sqrt()) * 0.9 }
    };
    let low = |z: f32| if (l - z.abs()) < 0.3 { clear + 0.08 } else { clear };
    // Cross-section corners at a station (+X = left): lower, belt, top (left, right).
    let sec = |z: f32, top: f32| {
        let w = hw_at(z);
        let wt = if top > belt + 0.1 { w - 0.17 - (top - belt) * 0.06 } else { w - 0.04 };
        [
            vec3(w - 0.03, low(z), z),
            vec3(w, belt - 0.12, z),
            vec3(w - 0.015, belt, z),
            vec3(wt, top, z),
            vec3(-wt, top, z),
            vec3(-w + 0.015, belt, z),
            vec3(-w, belt - 0.12, z),
            vec3(-w + 0.03, low(z), z),
        ]
    };
    let quad = |m: &mut MeshData, a: Vec3, b: Vec3, c: Vec3, d: Vec3, centre: Vec3, col: [f32; 3]| {
        let mut n = (c - a).cross(d - b).normalize_or_zero();
        if n == Vec3::ZERO {
            return;
        }
        if n.dot((a + b + c + d) * 0.25 - centre) < 0.0 {
            n = -n;
        }
        m.quad_n(a, b, c, d, n, col);
    };
    let centre = vec3(0.0, (clear + roof) * 0.45, 0.0);
    for w in prof.windows(2) {
        let ((z0, t0, _), (z1, t1, glass_top)) = (w[0], w[1]);
        if z1 - z0 < 1e-3 {
            continue;
        }
        let (a, b) = (sec(z0, t0), sec(z1, t1));
        let cabin = z0 >= gb - 0.01 && z1 <= gf + 0.01;
        for i in 0..7 {
            // Upper sides (belt → top): side windows in the cabin.
            let side_glass = cabin && (i == 2 || i == 4) && t0 > belt + 0.1 && t1 > belt + 0.1;
            let top_glass = i == 3 && glass_top;
            let glass = side_glass || top_glass;
            m.mat = if glass { mat::CANOPY } else { mat::PAINT };
            quad(m, a[i], a[i + 1], b[i + 1], b[i], centre, if glass { GLASS } else { PAINT });
        }
    }
    // Nose and tail faces.
    m.mat = mat::PAINT;
    for (z, top, front) in [(prof[0].0, prof[0].1, false), (prof[prof.len() - 1].0, prof[prof.len() - 1].1, true)] {
        let c = sec(z, top);
        let n = if front { Vec3::Z } else { -Vec3::Z };
        let ci = m.v((c[0] + c[7]) * 0.5 + vec3(0.0, (top - clear) * 0.5, 0.0), n, PAINT);
        let j0 = m.verts.len() as u32;
        for p in c {
            m.v(p, n, PAINT);
        }
        for k in 0..8u32 {
            m.idx.extend([ci, j0 + k, j0 + (k + 1) % 8]);
        }
    }
}

/// Passenger car family: (length, beltline, roof, glasshouse bottom z back /
/// front, top z back / front, ground clearance).
#[allow(clippy::too_many_arguments)]
fn car(m: &mut MeshData, k: u8, belt: f32, roof: f32, gb: f32, gf: f32, tb: f32, tf: f32, clear: f32, detail: bool) {
    let sp = spec(k);
    let (l, hw) = (sp.len * 0.5, sp.width * 0.5);
    loft_car(m, l, hw, belt, roof, gb, gf, tb, tf, clear, if detail { 1.0 } else { 0.0 });
    if !detail {
        return;
    }
    // B pillars between the side windows.
    let tw = hw - 0.2;
    let bz = (gb + gf) * 0.5 - 0.1;
    let tz = (tb + tf) * 0.5 - 0.1;
    for s in [-1.0f32, 1.0] {
        hexa(
            m,
            [
                vec3(s * (hw - 0.03), belt, bz - 0.06),
                vec3(s * (hw - 0.03) + s * 0.02, belt, bz - 0.06),
                vec3(s * (hw - 0.03) + s * 0.02, belt, bz + 0.06),
                vec3(s * (hw - 0.03), belt, bz + 0.06),
                vec3(s * (tw + 0.01), roof - 0.05, tz - 0.06),
                vec3(s * (tw + 0.01) + s * 0.02, roof - 0.05, tz - 0.06),
                vec3(s * (tw + 0.01) + s * 0.02, roof - 0.05, tz + 0.06),
                vec3(s * (tw + 0.01), roof - 0.05, tz + 0.06),
            ],
            PAINT,
        );
    }
    m.mat = mat::DIFFUSE;
    // Bumpers, grille, sills.
    obox(m, vec3(0.0, clear + 0.2, l - 0.05), AX, vec3(hw - 0.02, 0.13, 0.06), TRIM);
    obox(m, vec3(0.0, clear + 0.22, -l + 0.04), AX, vec3(hw - 0.02, 0.14, 0.06), TRIM);
    obox(m, vec3(0.0, belt - 0.32, l - 0.12), AX, vec3(hw * 0.45, 0.08, 0.03), GRILLE);
    for s in [-1.0f32, 1.0] {
        obox(m, vec3(s * (hw - 0.02), clear + 0.06, 0.0), AX, vec3(0.03, 0.05, l * 0.62), TRIM);
        // Mirrors.
        obox(m, vec3(s * (hw + 0.08), belt + 0.08, gf - 0.1), AX, vec3(0.08, 0.06, 0.04), TRIM);
    }
    lamps(m, k, l - 0.1, -l + 0.08);
    let r = 0.31 + (clear - 0.14).max(0.0) * 0.6;
    wheels(m, &[l - 0.85 - (l - 2.0).max(0.0) * 0.15, -l + 0.8 + (l - 2.0).max(0.0) * 0.12], hw - 0.12, r, 0.22);
}

/// Box-bodied vehicles: cab + body.
fn van(m: &mut MeshData, detail: bool) {
    let sp = spec(kind::VAN);
    let (l, hw) = (sp.len * 0.5, sp.width * 0.5);
    m.mat = mat::PAINT;
    // Load box (tall) and the cab's lower front.
    slab(m, 0.38, sp.height, -l + 0.02, l - 1.25, -l + 0.06, l - 1.5, hw, hw - 0.04, PAINT);
    slab(m, 0.38, 1.05, l - 1.3, l - 0.05, l - 1.3, l - 0.25, hw, hw - 0.04, PAINT);
    m.mat = mat::CANOPY;
    slab(m, 1.04, sp.height - 0.12, l - 1.3, l - 0.28, l - 1.5, l - 1.0, hw - 0.06, hw - 0.12, GLASS);
    if !detail {
        return;
    }
    m.mat = mat::DIFFUSE;
    obox(m, vec3(0.0, 0.52, l - 0.04), AX, vec3(hw - 0.03, 0.14, 0.06), TRIM);
    obox(m, vec3(0.0, 0.5, -l + 0.02), AX, vec3(hw - 0.03, 0.13, 0.05), TRIM);
    obox(m, vec3(0.0, 0.82, l - 0.1), AX, vec3(hw * 0.5, 0.1, 0.03), GRILLE);
    for s in [-1.0f32, 1.0] {
        obox(m, vec3(s * (hw + 0.1), 1.35, l - 1.2), AX, vec3(0.08, 0.12, 0.05), TRIM);
    }
    lamps(m, kind::VAN, l - 0.08, -l + 0.02);
    wheels(m, &[l - 0.95, -l + 1.0], hw - 0.14, 0.36, 0.24);
}

fn bus(m: &mut MeshData, decker: bool, detail: bool) {
    let k = if decker { kind::DECKER } else { kind::BUS };
    let sp = spec(k);
    let (l, hw) = (sp.len * 0.5, sp.width * 0.5);
    m.mat = mat::PAINT;
    slab(m, 0.32, sp.height, -l, l, -l + 0.05, l - 0.15, hw, hw - 0.05, PAINT);
    m.mat = mat::CANOPY;
    // Window bands (just proud of the body), windscreen.
    let bands: &[(f32, f32)] = if decker { &[(1.15, 2.05), (2.55, 3.9)] } else { &[(1.2, 2.6)] };
    for &(y0, y1) in bands {
        for s in [-1.0f32, 1.0] {
            obox(m, vec3(s * (hw + 0.005), (y0 + y1) * 0.5, -0.2), AX, vec3(0.01, (y1 - y0) * 0.5, l - 0.6), GLASS);
        }
        obox(m, vec3(0.0, (y0 + y1) * 0.5, l - 0.08 + 0.01), AX, vec3(hw - 0.12, (y1 - y0) * 0.5, 0.02), GLASS);
    }
    obox(m, vec3(0.0, 1.0 + 0.6, l - 0.07), AX, vec3(hw - 0.15, 0.55, 0.02), GLASS);
    if !detail {
        return;
    }
    m.mat = mat::DIFFUSE;
    obox(m, vec3(0.0, 0.42, l - 0.02), AX, vec3(hw - 0.02, 0.12, 0.05), TRIM);
    obox(m, vec3(0.0, 0.42, -l + 0.02), AX, vec3(hw - 0.02, 0.12, 0.05), TRIM);
    // Destination blind.
    let k2 = m.mat;
    m.mat = mat::EMISSIVE;
    let dy = if decker { 2.3 } else { 2.75 };
    let i = m.v_uv(vec3(-0.7, dy - 0.12, l + 0.005), Vec3::Z, [1.0, 0.7, 0.2], [0.0, 3.0]);
    m.v_uv(vec3(0.7, dy - 0.12, l + 0.005), Vec3::Z, [1.0, 0.7, 0.2], [0.0, 3.0]);
    m.v_uv(vec3(0.7, dy + 0.12, l + 0.005), Vec3::Z, [1.0, 0.7, 0.2], [0.0, 3.0]);
    m.v_uv(vec3(-0.7, dy + 0.12, l + 0.005), Vec3::Z, [1.0, 0.7, 0.2], [0.0, 3.0]);
    m.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    m.mat = k2;
    lamps(m, k, l, -l);
    wheels(m, &[l - 2.4, -l + 2.9], hw - 0.25, 0.5, 0.32);
}

fn lorry(m: &mut MeshData, artic: bool, detail: bool) {
    let k = if artic { kind::ARTIC } else { kind::LORRY };
    let sp = spec(k);
    let (l, hw) = (sp.len * 0.5, sp.width * 0.5);
    let cab_l = 2.1;
    let cab_h = 3.05;
    // Cab (paint) with windscreen.
    m.mat = mat::PAINT;
    slab(m, 0.55, cab_h, l - cab_l, l, l - cab_l, l - 0.12, hw, hw - 0.06, PAINT);
    m.mat = mat::CANOPY;
    obox(m, vec3(0.0, 2.15, l - 0.1), AX, vec3(hw - 0.15, 0.48, 0.03), GLASS);
    // Body: box (rigid) or trailer.
    m.mat = mat::DIFFUSE;
    let (b0, b1) = if artic { (-l, l - cab_l - 0.5) } else { (-l, l - cab_l - 0.15) };
    obox(m, vec3(0.0, (1.15 + sp.height) * 0.5, (b0 + b1) * 0.5), AX, vec3(hw, (sp.height - 1.15) * 0.5, (b1 - b0) * 0.5), BOX);
    obox(m, vec3(0.0, 0.85, (b0 + l) * 0.5), AX, vec3(hw - 0.35, 0.3, (l - b0) * 0.5 - 0.05), CHASSIS);
    if !detail {
        return;
    }
    obox(m, vec3(0.0, 0.62, l + 0.01), AX, vec3(hw - 0.02, 0.12, 0.04), TRIM);
    obox(m, vec3(0.0, 1.1, l + 0.005), AX, vec3(hw * 0.6, 0.25, 0.02), GRILLE);
    for s in [-1.0f32, 1.0] {
        obox(m, vec3(s * (hw + 0.12), 2.1, l - 0.3), AX, vec3(0.05, 0.25, 0.05), TRIM);
    }
    lamps(m, k, l + 0.01, -l);
    let zs: &[f32] = if artic { &[l - 1.2, l - 4.6, -l + 2.8, -l + 1.5, -l + 4.1] } else { &[l - 1.3, -l + 2.2] };
    wheels(m, zs, hw - 0.3, 0.52, 0.5);
}

fn bike(m: &mut MeshData, detail: bool) {
    let sp = spec(kind::BIKE);
    let l = sp.len * 0.5;
    m.mat = mat::PAINT;
    // Tank / fairing and seat.
    obox(m, vec3(0.0, 0.85, 0.15), AX, vec3(0.16, 0.18, 0.45), PAINT);
    m.mat = mat::DIFFUSE;
    obox(m, vec3(0.0, 0.88, -0.45), AX, vec3(0.14, 0.06, 0.3), GRILLE);
    // Rider: torso, head, legs (dark gear).
    obox(m, vec3(0.0, 1.22, -0.2), [Vec3::X, vec3(0.0, 0.94, 0.34).normalize(), vec3(0.0, -0.34, 0.94).normalize()], vec3(0.2, 0.3, 0.12), [0.05, 0.05, 0.06]);
    obox(m, vec3(0.0, 1.62, -0.05), AX, vec3(0.13, 0.14, 0.15), [0.12, 0.12, 0.14]);
    for s in [-1.0f32, 1.0] {
        obox(m, vec3(s * 0.17, 0.78, -0.2), AX, vec3(0.06, 0.2, 0.22), [0.05, 0.05, 0.06]);
    }
    if !detail {
        obox(m, vec3(0.0, 0.33, 0.0), AX, vec3(0.06, 0.32, l * 0.85), TRIM);
        return;
    }
    lamps(m, kind::BIKE, 0.62, -0.75);
    let k = m.mat;
    wheel(m, vec3(0.0, 0.32, l - 0.35), Vec3::X, 0.32, 0.12);
    wheel(m, vec3(0.0, 0.32, -l + 0.35), Vec3::X, 0.32, 0.14);
    m.mat = k;
}

/// Distant LOD (and the shadow caster): a body box and, for cars, a glass
/// cabin box with a painted roof.
fn boxes(m: &mut MeshData, k: u8) {
    let sp = spec(k);
    let (l, hw) = (sp.len * 0.5, sp.width * 0.5);
    m.mat = mat::PAINT;
    match k {
        kind::HATCH | kind::SALOON | kind::ESTATE | kind::SUV | kind::CAB => {
            let belt = sp.height * 0.62;
            obox(m, vec3(0.0, (0.2 + belt) * 0.5, 0.0), AX, vec3(hw, (belt - 0.2) * 0.5, l), PAINT);
            let (z0, z1) = match k {
                kind::SALOON => (-1.2, 0.75),
                kind::HATCH => (-1.85, 0.5),
                _ => (-l + 0.2, 0.6),
            };
            m.mat = mat::CANOPY;
            slab(m, belt, sp.height - 0.02, z0, z1, z0 + 0.3, z1 - 0.6, hw - 0.06, hw - 0.2, GLASS);
            m.mat = mat::PAINT;
            let y = sp.height;
            m.quad_n(vec3(-hw + 0.2, y, z0 + 0.32), vec3(hw - 0.2, y, z0 + 0.32), vec3(hw - 0.2, y, z1 - 0.62), vec3(-hw + 0.2, y, z1 - 0.62), Vec3::Y, PAINT);
        }
        kind::LORRY | kind::ARTIC => {
            obox(m, vec3(0.0, 1.8, l - 1.05), AX, vec3(hw, 1.25, 1.05), PAINT);
            m.mat = mat::DIFFUSE;
            obox(m, vec3(0.0, (1.15 + sp.height) * 0.5, -1.1), AX, vec3(hw, (sp.height - 1.15) * 0.5, l - 1.1), BOX);
        }
        kind::BIKE => {
            obox(m, vec3(0.0, 0.8, 0.0), AX, vec3(0.2, 0.75, l * 0.8), PAINT);
        }
        _ => {
            obox(m, vec3(0.0, (0.35 + sp.height) * 0.5, 0.0), AX, vec3(hw, (sp.height - 0.35) * 0.5, l), PAINT);
        }
    }
}

/// Mesh of a kind at a level of detail (0 detailed, 1 simple, 2 boxes).
pub fn vehicle(k: u8, lod: u8) -> MeshData {
    let mut m = MeshData::default();
    if lod >= 2 {
        boxes(&mut m, k);
        return m;
    }
    let d = lod == 0;
    match k {
        kind::HATCH => car(&mut m, k, 0.95, 1.48, -1.82, 0.55, -1.72, -0.25, 0.15, d),
        kind::SALOON => car(&mut m, k, 0.92, 1.45, -1.3, 0.78, -0.98, 0.0, 0.14, d),
        kind::ESTATE => car(&mut m, k, 0.94, 1.5, -2.25, 0.78, -2.2, 0.0, 0.15, d),
        kind::SUV => car(&mut m, k, 1.1, 1.72, -2.12, 0.62, -2.05, -0.12, 0.24, d),
        kind::CAB => {
            car(&mut m, k, 1.0, 1.83, -1.75, 0.55, -1.55, -0.05, 0.18, d);
            if d {
                // The roof sign (lit when for hire).
                lamp(&mut m, vec3(0.0, 1.92, 0.0), 0.25, 0.06, true, 3.0);
                lamp(&mut m, vec3(0.0, 1.92, -0.002), 0.25, 0.06, false, 3.0);
            }
        }
        kind::VAN => van(&mut m, d),
        kind::BUS => bus(&mut m, false, d),
        kind::DECKER => bus(&mut m, true, d),
        kind::LORRY => lorry(&mut m, false, d),
        kind::ARTIC => lorry(&mut m, true, d),
        _ => bike(&mut m, d),
    }
    m
}

/// Mesh levels of detail per kind.
pub const LODS: usize = 3;

/// All meshes, [kind][lod].
pub fn all() -> Vec<[MeshData; LODS]> {
    (0..KINDS as u8).map(|k| [vehicle(k, 0), vehicle(k, 1), vehicle(k, 2)]).collect()
}
