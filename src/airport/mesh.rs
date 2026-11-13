//! Procedural airport geometry: control towers, hangars, terminals, jet
//! bridges, ground vehicles, signs, floodlight masts, fuel tanks and the
//! windsock. Everything is built in a local frame (metres, +Y up, ground at
//! y = 0) as [`MeshData`] with the scene's materials, so it is lit, shadowed
//! and fogged exactly like the rest of the world.
//!
//! Vertex colours are sRGB-ish (the shader linearises them), like the
//! tile builders' colours.

use super::font;
use crate::scene::{MeshData, mat};
use glam::{Mat4, Quat, Vec2, Vec3, vec2, vec3};
use std::f32::consts::{PI, TAU};

pub type Col = [f32; 3];

pub const WHITE: Col = [0.86, 0.86, 0.84];
pub const OFF_WHITE: Col = [0.80, 0.80, 0.77];
pub const CONCRETE: Col = [0.70, 0.69, 0.66];
pub const DARK: Col = [0.10, 0.10, 0.11];
pub const TYRE: Col = [0.05, 0.05, 0.05];
pub const STEEL: Col = [0.55, 0.56, 0.58];
pub const YELLOW: Col = [0.95, 0.72, 0.08];
pub const SIGN_YELLOW: Col = [0.98, 0.80, 0.10];
pub const SIGN_RED: Col = [0.78, 0.06, 0.05];
pub const SIGN_BLACK: Col = [0.04, 0.04, 0.04];
pub const ORANGE: Col = [0.96, 0.42, 0.06];
pub const GLASS_TINT: Col = [0.30, 0.40, 0.45];

// ---------------------------------------------------------------- primitives

/// Oriented box: centre, unit axes (x, y, z) and half extents.
pub fn obox(m: &mut MeshData, c: Vec3, ax: [Vec3; 3], h: Vec3, col: Col) {
    let p = |x: f32, y: f32, z: f32| c + ax[0] * (x * h.x) + ax[1] * (y * h.y) + ax[2] * (z * h.z);
    m.quad_n(p(-1., 1., -1.), p(1., 1., -1.), p(1., 1., 1.), p(-1., 1., 1.), ax[1], col);
    m.quad_n(p(-1., -1., -1.), p(-1., -1., 1.), p(1., -1., 1.), p(1., -1., -1.), -ax[1], col);
    m.quad_n(p(1., -1., -1.), p(1., -1., 1.), p(1., 1., 1.), p(1., 1., -1.), ax[0], col);
    m.quad_n(p(-1., -1., -1.), p(-1., 1., -1.), p(-1., 1., 1.), p(-1., -1., 1.), -ax[0], col);
    m.quad_n(p(-1., -1., 1.), p(-1., 1., 1.), p(1., 1., 1.), p(1., -1., 1.), ax[2], col);
    m.quad_n(p(-1., -1., -1.), p(1., -1., -1.), p(1., 1., -1.), p(-1., 1., -1.), -ax[2], col);
}

/// Axis-aligned box from min / max corners.
pub fn aabox(m: &mut MeshData, lo: Vec3, hi: Vec3, col: Col) {
    obox(m, (lo + hi) * 0.5, [Vec3::X, Vec3::Y, Vec3::Z], (hi - lo) * 0.5, col);
}

/// Box rotated about +Y by `yaw` (rad, from +Z toward +X), centre `c`.
pub fn ybox(m: &mut MeshData, c: Vec3, yaw: f32, h: Vec3, col: Col) {
    let (s, co) = yaw.sin_cos();
    let z = vec3(s, 0.0, co);
    let x = vec3(co, 0.0, -s);
    obox(m, c, [x, Vec3::Y, z], h, col);
}

/// Frustum along `axis` from `base` (radius r0) to base + axis * len (r1);
/// smooth sides, optional caps. `u` coordinates run around the rim (metres)
/// and `v` along the axis (for wall shaders).
pub fn frustum(m: &mut MeshData, base: Vec3, axis: Vec3, len: f32, r0: f32, r1: f32, sides: u32, col: Col, caps: bool) {
    let a = axis.normalize();
    let t = a.any_orthonormal_vector();
    let b = a.cross(t);
    let slope = (r0 - r1) / len.max(1e-3);
    let i0 = m.verts.len() as u32;
    let circ = TAU * (r0 + r1) * 0.5;
    for k in 0..=sides {
        let ang = TAU * k as f32 / sides as f32;
        let d = t * ang.cos() + b * ang.sin();
        let n = (d + a * slope).normalize();
        let u = circ * k as f32 / sides as f32;
        m.v_uv(base + d * r0, n, col, [u, 0.0]);
        m.v_uv(base + a * len + d * r1, n, col, [u, len]);
    }
    for k in 0..sides {
        let q = i0 + 2 * k;
        m.idx.extend([q, q + 2, q + 3, q, q + 3, q + 1]);
    }
    if caps {
        for (c, r, n) in [(base, r0, -a), (base + a * len, r1, a)] {
            if r <= 1e-3 {
                continue;
            }
            let ci = m.v(c, n, col);
            let j0 = m.verts.len() as u32;
            for k in 0..sides {
                let ang = TAU * k as f32 / sides as f32;
                m.v(c + (t * ang.cos() + b * ang.sin()) * r, n, col);
            }
            for k in 0..sides {
                m.idx.extend([ci, j0 + k, j0 + (k + 1) % sides]);
            }
        }
    }
}

/// Vertical cylinder standing on `base`.
pub fn cyl(m: &mut MeshData, base: Vec3, r: f32, h: f32, sides: u32, col: Col) {
    frustum(m, base, Vec3::Y, h, r, r, sides, col, true);
}

/// Wheel: a short cylinder along `axle` centred at `c`.
pub fn wheel(m: &mut MeshData, c: Vec3, axle: Vec3, r: f32, w: f32) {
    let a = axle.normalize();
    frustum(m, c - a * (w * 0.5), a, w, r, r, 10, TYRE, true);
    m.mat = mat::METAL;
    frustum(m, c - a * (w * 0.52), a, w * 1.04, r * 0.5, r * 0.5, 8, STEEL, true);
    m.mat = mat::DIFFUSE;
}

/// Vertical prism over a polygon ring (walls + top cap), walls carrying
/// (perimeter metres, height) uvs.
pub fn prism(m: &mut MeshData, ring: &[Vec2], y0: f32, y1: f32, col: Col, top: Option<(f32, Col)>) {
    let n = ring.len();
    if n < 3 {
        return;
    }
    let s = if ring_area(ring) > 0.0 { 1.0 } else { -1.0 };
    let mut u = 0.0;
    for i in 0..n {
        let (p, q) = (ring[i], ring[(i + 1) % n]);
        let e = q - p;
        let len = e.length();
        let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * s;
        let i0 = m.v_uv(vec3(p.x, y0, p.y), nrm, col, [u, y0]);
        m.v_uv(vec3(q.x, y0, q.y), nrm, col, [u + len, y0]);
        m.v_uv(vec3(q.x, y1, q.y), nrm, col, [u + len, y1]);
        m.v_uv(vec3(p.x, y1, p.y), nrm, col, [u, y1]);
        m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        u += len;
    }
    if let Some((material, c)) = top {
        let keep = m.mat;
        m.mat = material;
        m.flat_poly(ring, &[], y1, Vec3::Y, c);
        m.mat = keep;
    }
}

pub fn ring_area(pts: &[Vec2]) -> f32 {
    let n = pts.len();
    (0..n).map(|i| pts[i].perp_dot(pts[(i + 1) % n])).sum::<f32>() * 0.5
}

/// Regular polygon ring.
pub fn ngon(c: Vec2, r: f32, n: u32, phase: f32) -> Vec<Vec2> {
    (0..n).map(|k| c + Vec2::from_angle(phase + TAU * k as f32 / n as f32) * r).collect()
}

// ---------------------------------------------------------------- tower

/// Air-traffic control tower, base centre at the origin, `height` to the
/// cab roof. Large towers: tapered concrete shaft, equipment drum, a wide
/// outward-leaning glass cab, catwalk, roof and antenna mast. Small ones
/// (< 18 m): a two / three storey block with a glass cab on top.
/// Returns the mesh and the top of the antenna (obstruction light).
pub fn control_tower(height: f32, shaft_r: f32, seed: u64) -> (MeshData, Vec3) {
    let mut m = MeshData::default();
    let h = height.max(8.0);
    if h < 18.0 {
        let w = (shaft_r * 2.2).clamp(5.0, 9.0);
        let cab = 3.0;
        let body = h - cab;
        m.mat = mat::WALL;
        let ring = vec![vec2(-w * 0.5, -w * 0.5), vec2(w * 0.5, -w * 0.5), vec2(w * 0.5, w * 0.5), vec2(-w * 0.5, w * 0.5)];
        prism(&mut m, &ring, 0.0, body, OFF_WHITE, Some((mat::ROOF, [0.40, 0.40, 0.42])));
        let c = (w * 0.5 + 0.6, cab);
        m.mat = mat::GLASS;
        let cab_ring = ngon(Vec2::ZERO, c.0 * 1.25, 8, PI / 8.0);
        prism(&mut m, &cab_ring, body, body + c.1, GLASS_TINT, None);
        m.mat = mat::DIFFUSE;
        let roof = ngon(Vec2::ZERO, c.0 * 1.35, 8, PI / 8.0);
        prism(&mut m, &roof, body + c.1, body + c.1 + 0.35, [0.30, 0.30, 0.32], Some((mat::ROOF, [0.30, 0.30, 0.32])));
        m.mat = mat::METAL;
        cyl(&mut m, vec3(0.0, body + c.1 + 0.35, 0.0), 0.06, 3.0, 6, STEEL);
        return (m, vec3(0.0, h + 3.3, 0.0));
    }
    let r = shaft_r.clamp(2.5, 6.5);
    let cab_h = (h * 0.07).clamp(4.0, 6.5);
    let drum_h = (h * 0.09).clamp(4.5, 9.0);
    let shaft_top = h - cab_h - drum_h - 1.0;
    let style = seed % 3;
    // Shaft: tapered, concrete.
    m.mat = mat::DIFFUSE;
    let shaft_col = [[0.80, 0.79, 0.76], [0.74, 0.74, 0.74], [0.84, 0.83, 0.80]][style as usize];
    frustum(&mut m, Vec3::ZERO, Vec3::Y, shaft_top, r, r * 0.82, 16, shaft_col, false);
    // A fin / lift core on one side (most modern towers).
    if style != 1 {
        ybox(&mut m, vec3(0.0, shaft_top * 0.5, r * 0.85), 0.0, vec3(r * 0.45, shaft_top * 0.5, r * 0.35), shaft_col);
    }
    // Base building around the foot.
    m.mat = mat::WALL;
    let base_r = r * 2.6;
    prism(&mut m, &ngon(Vec2::ZERO, base_r, 10, 0.0), 0.0, 7.5, OFF_WHITE, Some((mat::ROOF, [0.42, 0.42, 0.44])));
    // Equipment drum with a window band.
    let y0 = shaft_top;
    m.mat = mat::DIFFUSE;
    frustum(&mut m, vec3(0.0, y0, 0.0), Vec3::Y, 1.0, r * 0.82, r * 1.55, 16, shaft_col, false);
    m.mat = mat::WALL;
    frustum(&mut m, vec3(0.0, y0 + 1.0, 0.0), Vec3::Y, drum_h, r * 1.55, r * 1.6, 16, OFF_WHITE, false);
    m.mat = mat::GLASS;
    frustum(&mut m, vec3(0.0, y0 + 1.0 + drum_h * 0.35, 0.0), Vec3::Y, drum_h * 0.4, r * 1.62, r * 1.66, 16, [0.22, 0.28, 0.32], false);
    // Floor slab + catwalk.
    let yc = y0 + 1.0 + drum_h;
    m.mat = mat::DIFFUSE;
    frustum(&mut m, vec3(0.0, yc - 0.4, 0.0), Vec3::Y, 0.5, r * 2.15, r * 2.15, 20, [0.62, 0.62, 0.63], true);
    // Cab: 12 facets leaning out (glare-free glazing), thin mullions.
    m.mat = mat::GLASS;
    frustum(&mut m, vec3(0.0, yc + 0.1, 0.0), Vec3::Y, cab_h, r * 1.85, r * 2.15, 12, [0.20, 0.30, 0.33], false);
    m.mat = mat::METAL;
    for k in 0..12 {
        let a = TAU * (k as f32 + 0.5) / 12.0;
        let d0 = vec3(a.cos(), 0.0, a.sin());
        let p0 = vec3(0.0, yc + 0.1, 0.0) + d0 * (r * 1.87);
        let p1 = vec3(0.0, yc + 0.1 + cab_h, 0.0) + d0 * (r * 2.17);
        frustum(&mut m, p0, p1 - p0, (p1 - p0).length(), 0.09, 0.09, 4, [0.3, 0.31, 0.33], false);
    }
    // Roof: overhanging cap, plant room, antenna mast.
    let yr = yc + 0.1 + cab_h;
    m.mat = mat::DIFFUSE;
    frustum(&mut m, vec3(0.0, yr, 0.0), Vec3::Y, 0.8, r * 2.35, r * 2.25, 20, [0.86, 0.86, 0.85], true);
    m.mat = mat::WALL;
    prism(&mut m, &ngon(Vec2::ZERO, r * 0.9, 8, 0.0), yr + 0.8, yr + 3.2, OFF_WHITE, Some((mat::ROOF, [0.45, 0.45, 0.46])));
    m.mat = mat::METAL;
    let mast_h = (h * 0.12).clamp(4.0, 11.0);
    frustum(&mut m, vec3(0.0, yr + 3.2, 0.0), Vec3::Y, mast_h, 0.22, 0.08, 6, [0.80, 0.80, 0.82], false);
    // Radar dome / dishes.
    m.mat = mat::DIFFUSE;
    m.blob_lod(vec3(r * 1.2, yr + 1.6, 0.0), Vec3::splat(1.1), [0.90, 0.90, 0.88], 10, 6);
    (m, vec3(0.0, yr + 3.2 + mast_h, 0.0))
}

// ---------------------------------------------------------------- hangar

/// Hangar on an oriented rectangle `q` (corners in order; edge q0→q1 runs
/// along the ridge): corrugated walls, a barrel roof (`rise` m) and big
/// sliding doors on edge `door` (0..4, None = no door). Heights relative to
/// the ground (y = 0); the caller lifts it onto the terrain.
pub fn hangar(m: &mut MeshData, q: [Vec2; 4], eaves: f32, rise: f32, door: Option<usize>, col: Col, seed: u64) {
    let c = (q[0] + q[1] + q[2] + q[3]) * 0.25;
    let along = (q[1] - q[0]).normalize_or_zero();
    let across = q[3] - q[0];
    let width = across.length();
    let across_d = across / width.max(1e-3);
    let p3 = |v: Vec2, y: f32| vec3(v.x, y, v.y);
    // Walls (corrugated cladding; uv = metres along, height).
    m.mat = mat::CLADDING;
    let mut u = 0.0;
    for i in 0..4 {
        let (a, b) = (q[i], q[(i + 1) % 4]);
        let e = b - a;
        let len = e.length();
        let mut nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero();
        if nrm.dot(p3(a - c, 0.0)) < 0.0 {
            nrm = -nrm;
        }
        let i0 = m.v_uv(p3(a, -0.6), nrm, col, [u, 0.0]);
        m.v_uv(p3(b, -0.6), nrm, col, [u + len, 0.0]);
        m.v_uv(p3(b, eaves), nrm, col, [u + len, eaves]);
        m.v_uv(p3(a, eaves), nrm, col, [u, eaves]);
        m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        u += len;
        // Gable ends (edges across the ridge) close the barrel.
        if i % 2 == 1 && rise > 0.1 {
            let k = 12;
            let base = m.verts.len() as u32;
            let mid = (a + b) * 0.5;
            m.v_uv(p3(mid, eaves), nrm, col, [u, eaves]);
            for s in 0..=k {
                let t = s as f32 / k as f32;
                let p = a.lerp(b, t);
                let y = eaves + rise * (1.0 - (2.0 * t - 1.0).powi(2));
                m.v_uv(p3(p, y), nrm, col, [u + len * t, y]);
            }
            for s in 0..k {
                m.idx.extend([base, base + 1 + s, base + 2 + s]);
            }
        }
    }
    // Barrel roof (ribs run over the arch: uv.x = metres along the ridge).
    m.mat = mat::CLADDING;
    let roof_col = [col[0] * 0.92, col[1] * 0.92, col[2] * 0.94];
    let k = 14;
    let len = (q[1] - q[0]).length();
    let o = 0.5; // eaves overhang
    let base = m.verts.len() as u32;
    for s in 0..=k {
        let t = s as f32 / k as f32;
        let x = (2.0 * t - 1.0) * (width * 0.5 + o);
        let y = eaves + rise * (1.0 - (x / (width * 0.5)).powi(2)).max(-0.1);
        let slope = -2.0 * rise * x / (width * 0.5).powi(2);
        let n2 = vec2(-slope, 1.0).normalize();
        let n = vec3(across_d.x * n2.x, n2.y, across_d.y * n2.x);
        let mid0 = (q[0] + q[3]) * 0.5 - along * o;
        let mid1 = (q[1] + q[2]) * 0.5 + along * o;
        let off = across_d * x;
        m.v_uv(p3(mid0 + off, y), n, roof_col, [0.0, x]);
        m.v_uv(p3(mid1 + off, y), n, roof_col, [len + 2.0 * o, x]);
    }
    for s in 0..k {
        let i = base + 2 * s;
        m.idx.extend([i, i + 2, i + 3, i, i + 3, i + 1]);
    }
    // Doors: tall panels standing proud of the wall, a header beam, seams.
    if let Some(d) = door {
        let (a, b) = (q[d], q[(d + 1) % 4]);
        let e = b - a;
        let len = e.length();
        let dir = e / len.max(1e-3);
        let mut out = vec2(dir.y, -dir.x);
        if out.dot(a - c) < 0.0 {
            out = -out;
        }
        let dh = (eaves * 0.86).min(eaves - 1.2).max(3.0);
        let dw = len * 0.86;
        let panels = ((dw / 7.0).round() as usize).clamp(2, 14);
        let pw = dw / panels as f32;
        let start = a + dir * (len - dw) * 0.5 + out * 0.25;
        let tones = [[0.70, 0.71, 0.72], [0.60, 0.62, 0.64], [0.78, 0.78, 0.76]];
        let door_col = tones[(seed % 3) as usize];
        m.mat = mat::CLADDING;
        for i in 0..panels {
            let p0 = start + dir * (pw * i as f32 + 0.06);
            let p1 = start + dir * (pw * (i + 1) as f32 - 0.06);
            let nn = p3(out, 0.0);
            let o2 = out * 0.12 * (i % 2) as f32;
            let i0 = m.v_uv(p3(p0 + o2, 0.0), nn, door_col, [0.0, 0.0]);
            m.v_uv(p3(p1 + o2, 0.0), nn, door_col, [pw, 0.0]);
            m.v_uv(p3(p1 + o2, dh), nn, door_col, [pw, dh]);
            m.v_uv(p3(p0 + o2, dh), nn, door_col, [0.0, dh]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        }
        // Header beam and a dark slot under it (the door track).
        m.mat = mat::DIFFUSE;
        let hc = (a + b) * 0.5 + out * 0.45;
        let yaw = dir.x.atan2(dir.y);
        ybox(m, vec3(hc.x, dh + 0.6, hc.y), yaw, vec3(0.45, 0.6, len * 0.47), [0.30, 0.31, 0.33]);
    }
}

// ---------------------------------------------------------------- terminal

/// Terminal facade over a footprint piece: full-height glazing (curtain-wall
/// material) above a solid plinth, a deep roof canopy overhanging the
/// glass, a parapet and roof plant. `border[i]`: edge i is a tile-seam edge
/// (no wall, no canopy). Heights relative to the ground.
pub fn terminal(m: &mut MeshData, outer: &[Vec2], holes: &[Vec<Vec2>], border: &[bool], height: f32, min_h: f32, seed: u64) {
    let tints: [Col; 4] = [[0.58, 0.68, 0.74], [0.64, 0.72, 0.76], [0.52, 0.64, 0.70], [0.68, 0.74, 0.78]];
    let tint = tints[(seed % 4) as usize];
    let plinth = if min_h > 0.5 { min_h } else { 0.0 };
    let glass_lo = plinth + if min_h > 0.5 { 0.0 } else { 0.9 };
    let s = if ring_area(outer) > 0.0 { 1.0 } else { -1.0 };
    let n = outer.len();
    // Per-building seed in the uv (glass shader: mullion pitch, floor height).
    // (Negative seeds: the glass shader lights terminals evenly at night.)
    let mut u = -((1 + seed % 30) as f32) * 4096.0;
    for i in 0..n {
        let (p, q) = (outer[i], outer[(i + 1) % n]);
        let e = q - p;
        let len = e.length();
        if border.get(i).copied().unwrap_or(false) || len < 0.05 {
            u += len;
            continue;
        }
        let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * s;
        // Plinth (concrete), then glass to the roof.
        if glass_lo > plinth + 0.01 {
            m.mat = mat::DIFFUSE;
            let i0 = m.v(vec3(p.x, plinth, p.y), nrm, [0.55, 0.55, 0.54]);
            m.v(vec3(q.x, plinth, q.y), nrm, [0.55, 0.55, 0.54]);
            m.v(vec3(q.x, glass_lo, q.y), nrm, [0.55, 0.55, 0.54]);
            m.v(vec3(p.x, glass_lo, p.y), nrm, [0.55, 0.55, 0.54]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        }
        // Glass, uv along the wall from a world-fixed origin: walls of
        // overlapping building parts on the same line shade identically (no
        // z-fighting pattern).
        m.mat = mat::GLASS;
        let d = e / len;
        let dc = if d.x > 1e-4 || (d.x.abs() <= 1e-4 && d.y > 0.0) { d } else { -d };
        let (a0, a1) = (p.dot(dc), q.dot(dc));
        let base = -((1 + seed % 30) as f32) * 4096.0 + a0.min(a1).rem_euclid(1024.0) - a0.min(a1);
        let i0 = m.v_uv(vec3(p.x, glass_lo, p.y), nrm, tint, [base + a0, glass_lo]);
        m.v_uv(vec3(q.x, glass_lo, q.y), nrm, tint, [base + a1, glass_lo]);
        m.v_uv(vec3(q.x, height - 1.2, q.y), nrm, tint, [base + a1, height - 1.2]);
        m.v_uv(vec3(p.x, height - 1.2, p.y), nrm, tint, [base + a0, height - 1.2]);
        m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        // Fascia band under the roof edge.
        m.mat = mat::METAL;
        let fc = [0.62, 0.64, 0.66];
        let j0 = m.v(vec3(p.x, height - 1.2, p.y), nrm, fc);
        m.v(vec3(q.x, height - 1.2, q.y), nrm, fc);
        m.v(vec3(q.x, height + 0.6, q.y), nrm, fc);
        m.v(vec3(p.x, height + 0.6, p.y), nrm, fc);
        m.idx.extend([j0, j0 + 1, j0 + 2, j0, j0 + 2, j0 + 3]);
        // Roof canopy overhanging the glass (shades the facade).
        if len > 6.0 && min_h <= 0.5 && height >= 8.0 {
            m.mat = mat::DIFFUSE;
            let o = vec2(nrm.x, nrm.z) * 2.5;
            let cc = [0.78, 0.79, 0.80];
            m.quad_n(vec3(p.x, height + 0.6, p.y), vec3(q.x, height + 0.6, q.y), vec3(q.x + o.x, height + 0.3, q.y + o.y), vec3(p.x + o.x, height + 0.3, p.y + o.y), Vec3::Y, cc);
            m.quad_n(
                vec3(p.x, height - 0.2, p.y),
                vec3(q.x, height - 0.2, q.y),
                vec3(q.x + o.x, height - 0.1, q.y + o.y),
                vec3(p.x + o.x, height - 0.1, p.y + o.y),
                -Vec3::Y,
                [0.55, 0.56, 0.57],
            );
            m.quad_n(
                vec3(p.x + o.x, height - 0.1, p.y + o.y),
                vec3(q.x + o.x, height - 0.1, q.y + o.y),
                vec3(q.x + o.x, height + 0.3, q.y + o.y),
                vec3(p.x + o.x, height + 0.3, p.y + o.y),
                nrm,
                fc,
            );
        }
        u += len;
    }
    for h in holes {
        let hn = h.len();
        let hs = if ring_area(h) > 0.0 { -1.0 } else { 1.0 };
        m.mat = mat::GLASS;
        for i in 0..hn {
            let (p, q) = (h[i], h[(i + 1) % hn]);
            let e = q - p;
            let len = e.length();
            let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * hs;
            let i0 = m.v_uv(vec3(p.x, glass_lo, p.y), nrm, tint, [u, glass_lo]);
            m.v_uv(vec3(q.x, glass_lo, q.y), nrm, tint, [u + len, glass_lo]);
            m.v_uv(vec3(q.x, height, q.y), nrm, tint, [u + len, height]);
            m.v_uv(vec3(p.x, height, p.y), nrm, tint, [u, height]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
            u += len;
        }
    }
    // Roof: light membrane; plinth underside when raised.
    m.mat = mat::ROOF;
    m.flat_poly(outer, holes, height + 0.6, Vec3::Y, [0.66, 0.66, 0.65]);
    if min_h > 0.5 {
        m.mat = mat::DIFFUSE;
        m.flat_poly(outer, holes, min_h, -Vec3::Y, [0.5, 0.5, 0.5]);
    }
    // Roof plant: a few boxes on bigger roofs (inside the footprint).
    let area = ring_area(outer).abs();
    if area > 2500.0 {
        let (lo, hi) = bbox(outer);
        let mut r = seed;
        let k = ((area / 4000.0) as usize).clamp(1, 8);
        m.mat = mat::METAL;
        for _ in 0..k * 3 {
            r = crate::scene::hash(r.wrapping_add(0x9e37));
            let t = vec2((r & 0xffff) as f32 / 65535.0, ((r >> 16) & 0xffff) as f32 / 65535.0);
            let p = lo + (hi - lo) * t;
            if !crate::osm::point_in_ring(p, outer) || holes.iter().any(|h| crate::osm::point_in_ring(p, h)) || border_near(outer, border, p, 12.0) {
                continue;
            }
            let sz = vec3(2.0 + (r >> 40) as f32 % 4.0, 1.2 + (r >> 48) as f32 % 2.0, 2.0 + (r >> 44) as f32 % 5.0);
            aabox(m, vec3(p.x, height + 0.6, p.y) - vec3(sz.x, 0.0, sz.z) * 0.5, vec3(p.x, height + 0.6, p.y) + vec3(sz.x * 0.5, sz.y, sz.z * 0.5), [0.62, 0.63, 0.64]);
        }
    }
}

fn border_near(ring: &[Vec2], border: &[bool], p: Vec2, d: f32) -> bool {
    let n = ring.len();
    (0..n).any(|i| border.get(i).copied().unwrap_or(false) && seg_dist(p, ring[i], ring[(i + 1) % n]) < d)
}

pub fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

pub fn bbox(pts: &[Vec2]) -> (Vec2, Vec2) {
    pts.iter().fold((Vec2::splat(f32::MAX), Vec2::splat(f32::MIN)), |(a, b), p| (a.min(*p), b.max(*p)))
}

// ---------------------------------------------------------------- jet bridge

/// An apron-drive passenger boarding bridge from a terminal facade to an
/// aircraft door. `root`: facade point at the ground (the rotunda stands
/// `out` metres in front of it), `root_h`: terminal floor height, `door`:
/// door sill point (ground-relative height in y), `fus`: the fuselage axis
/// at the door (unit, horizontal), so the cab meets the door squarely.
pub fn jet_bridge(m: &mut MeshData, root: Vec2, out: Vec2, root_h: f32, door: Vec3, fus: Vec2, seed: u64) {
    let rot = root + out * 4.0;
    let r3 = vec3(rot.x, 0.0, rot.y);
    // Fixed link from the facade to the rotunda.
    let skin = [[0.80, 0.81, 0.82], [0.72, 0.73, 0.75], [0.84, 0.82, 0.78]][(seed % 3) as usize];
    let glass_band = [0.20, 0.26, 0.30];
    let link_c = (root + rot) * 0.5;
    m.mat = mat::CLADDING;
    let yaw_o = out.x.atan2(out.y);
    ybox(m, vec3(link_c.x, root_h + 1.45, link_c.y), yaw_o, vec3(1.4, 1.45, 2.1), skin);
    // Rotunda column + cabin.
    m.mat = mat::METAL;
    cyl(m, r3, 0.55, root_h - 0.2, 10, [0.50, 0.52, 0.55]);
    m.mat = mat::CLADDING;
    frustum(m, r3 + vec3(0.0, root_h - 0.2, 0.0), Vec3::Y, 3.3, 2.3, 2.3, 16, skin, true);
    // Tunnel: rotunda → cab, three telescoping sections.
    let cab_back = {
        let side = vec2(-fus.y, fus.x);
        let s = if side.dot(vec2(door.x, door.z) - rot) > 0.0 { -side } else { side };
        // The cab stands off the fuselage by its own depth.
        vec2(door.x, door.z) + s * 3.6
    };
    let start = r3 + vec3(0.0, root_h + 1.45, 0.0);
    let end = vec3(cab_back.x, door.y + 1.45, cab_back.y);
    let d = end - start;
    let len = d.length().max(1.0);
    let dir = d / len;
    let hdir = vec3(dir.x, 0.0, dir.z).normalize_or(Vec3::Z);
    let side = Vec3::Y.cross(hdir).normalize();
    let up = dir.cross(side).normalize() * -1.0;
    let up = if up.y < 0.0 { -up } else { up };
    let secs = 3;
    let r0 = 2.3;
    for i in 0..secs {
        let t0 = r0 / len + (i as f32 / secs as f32) * (1.0 - r0 / len);
        let t1 = r0 / len + ((i + 1) as f32 / secs as f32) * (1.0 - r0 / len) + 0.03;
        let c = start + d * ((t0 + t1) * 0.5);
        let half = vec3(1.45 - 0.12 * i as f32, 1.45 - 0.1 * i as f32, len * (t1 - t0) * 0.5);
        m.mat = mat::CLADDING;
        obox(m, c, [side, up, dir], half, skin);
        // Window band along both sides.
        m.mat = mat::GLASS;
        for s in [-1.0f32, 1.0] {
            let cc = c + side * (s * (half.x + 0.02)) + up * 0.25;
            obox(m, cc, [side, up, dir], vec3(0.02, 0.45, half.z * 0.9), glass_band);
        }
    }
    // Drive column with its bogie at ~2/3 of the length.
    let dc = start + d * 0.68;
    let foot = vec3(dc.x, 0.0, dc.z);
    m.mat = mat::METAL;
    for s in [-1.0f32, 1.0] {
        let b = foot + side * (s * 1.0);
        frustum(m, b + Vec3::Y * 0.9, Vec3::Y, (dc.y - 1.4 - 0.9).max(0.3), 0.18, 0.18, 6, [0.45, 0.46, 0.48], false);
    }
    m.mat = mat::DIFFUSE;
    obox(m, foot + Vec3::Y * 0.65, [side, Vec3::Y, hdir], vec3(1.5, 0.35, 0.7), [0.25, 0.26, 0.28]);
    for s in [-1.0f32, 1.0] {
        wheel(m, foot + side * (s * 1.25) + Vec3::Y * 0.5, side, 0.5, 0.45);
    }
    // Cab: squared to the fuselage, with a dark bellows canopy at the door.
    let fz = vec3(fus.x, 0.0, fus.y);
    let fx = Vec3::Y.cross(fz).normalize();
    let to_door = vec3(door.x - cab_back.x, 0.0, door.z - cab_back.y).normalize_or(hdir);
    let cab_c = vec3(cab_back.x, door.y + 1.45, cab_back.y) + to_door * 1.5;
    m.mat = mat::CLADDING;
    obox(m, cab_c, [fx, Vec3::Y, fz], vec3(1.7, 1.55, 1.9), skin);
    m.mat = mat::DIFFUSE;
    obox(m, cab_c + to_door * 1.75, [fx, Vec3::Y, fz], vec3(1.2, 1.35, 1.3), [0.12, 0.12, 0.13]);
    m.mat = mat::GLASS;
    obox(m, cab_c - to_door * 0.4 + Vec3::Y * 0.4, [fx, Vec3::Y, fz], vec3(1.75, 0.5, 1.6), glass_band);
}

// ---------------------------------------------------------------- vehicles

/// Ground vehicle kinds (instanced meshes).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Vehicle {
    Tug,
    Tractor,
    Cart,
    FuelTruck,
    Catering,
    Bus,
    BeltLoader,
    Gpu,
    Car,
    Stairs,
    Bowser,
    FireTruck,
}

pub const VEHICLES: [Vehicle; 12] = [
    Vehicle::Tug,
    Vehicle::Tractor,
    Vehicle::Cart,
    Vehicle::FuelTruck,
    Vehicle::Catering,
    Vehicle::Bus,
    Vehicle::BeltLoader,
    Vehicle::Gpu,
    Vehicle::Car,
    Vehicle::Stairs,
    Vehicle::Bowser,
    Vehicle::FireTruck,
];

impl Vehicle {
    pub fn index(self) -> usize {
        VEHICLES.iter().position(|v| *v == self).unwrap_or(0)
    }
    /// Length (m) along +Z.
    pub fn length(self) -> f32 {
        match self {
            Vehicle::Tug => 5.6,
            Vehicle::Tractor => 2.9,
            Vehicle::Cart => 3.1,
            Vehicle::FuelTruck => 10.5,
            Vehicle::Catering => 8.5,
            Vehicle::Bus => 13.0,
            Vehicle::BeltLoader => 8.0,
            Vehicle::Gpu => 2.6,
            Vehicle::Car => 4.6,
            Vehicle::Stairs => 6.5,
            Vehicle::Bowser => 6.8,
            Vehicle::FireTruck => 12.0,
        }
    }
}

/// Vehicle mesh: local frame +Z forward, +X left, ground at y = 0.
pub fn vehicle(v: Vehicle, livery: u64) -> MeshData {
    let mut m = MeshData::default();
    m.mat = mat::PAINT;
    let ax = [Vec3::X, Vec3::Y, Vec3::Z];
    let ops: [Col; 4] = [[0.95, 0.70, 0.08], [0.90, 0.90, 0.88], [0.20, 0.42, 0.75], [0.80, 0.25, 0.10]];
    let body = ops[(livery % 4) as usize];
    let glass = |m: &mut MeshData, c: Vec3, h: Vec3| {
        let k = m.mat;
        m.mat = mat::CANOPY;
        obox(m, c, ax, h, [0.05, 0.06, 0.07]);
        m.mat = k;
    };
    let wheels = |m: &mut MeshData, zs: &[f32], half_track: f32, r: f32| {
        for &z in zs {
            for s in [-1.0f32, 1.0] {
                wheel(m, vec3(s * half_track, r, z), Vec3::X, r, 0.32);
            }
        }
        m.mat = mat::PAINT;
    };
    match v {
        Vehicle::Tug => {
            // Low, heavy pushback tractor: wedge nose, recessed cab, tow bar.
            obox(&mut m, vec3(0.0, 0.75, 0.0), ax, vec3(1.3, 0.45, 2.8), [0.92, 0.92, 0.90]);
            obox(&mut m, vec3(0.0, 1.22, 0.6), ax, vec3(1.25, 0.04, 1.9), body);
            obox(&mut m, vec3(0.0, 1.45, -1.7), ax, vec3(0.9, 0.25, 0.75), body);
            glass(&mut m, vec3(0.0, 1.95, -1.7), vec3(0.85, 0.25, 0.7));
            obox(&mut m, vec3(0.0, 2.24, -1.7), ax, vec3(0.92, 0.05, 0.78), body);
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 0.5, 2.9), ax, vec3(1.0, 0.18, 0.15), DARK);
            obox(&mut m, vec3(0.0, 0.45, 4.2), ax, vec3(0.08, 0.06, 1.3), [0.15, 0.15, 0.16]);
            wheels(&mut m, &[1.8, -1.8], 1.5, 0.5);
        }
        Vehicle::Tractor => {
            obox(&mut m, vec3(0.0, 0.75, 0.3), ax, vec3(0.75, 0.4, 1.1), body);
            m.mat = mat::DIFFUSE;
            for (x, z) in [(-0.65, -0.6), (0.65, -0.6), (-0.65, 0.4), (0.65, 0.4)] {
                obox(&mut m, vec3(x, 1.55, z), ax, vec3(0.04, 0.45, 0.04), DARK);
            }
            m.mat = mat::PAINT;
            obox(&mut m, vec3(0.0, 2.0, -0.1), ax, vec3(0.75, 0.05, 0.6), body);
            obox(&mut m, vec3(0.0, 1.0, -0.6), ax, vec3(0.45, 0.2, 0.35), DARK);
            wheels(&mut m, &[0.9, -0.8], 0.82, 0.38);
        }
        Vehicle::Cart => {
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 0.55, 0.0), ax, vec3(0.8, 0.08, 1.4), [0.25, 0.26, 0.28]);
            obox(&mut m, vec3(0.0, 1.25, 0.0), ax, vec3(0.78, 0.62, 1.38), [[0.85, 0.83, 0.78], [0.72, 0.75, 0.80], [0.30, 0.38, 0.55], [0.82, 0.82, 0.82]][(livery % 4) as usize]);
            obox(&mut m, vec3(0.0, 0.5, 1.75), ax, vec3(0.05, 0.05, 0.4), DARK);
            wheels(&mut m, &[0.9, -0.9], 0.82, 0.25);
        }
        Vehicle::FuelTruck | Vehicle::Bowser => {
            let l = v.length();
            let cab_z = l * 0.5 - 1.2;
            obox(&mut m, vec3(0.0, 1.55, cab_z), ax, vec3(1.25, 1.0, 1.1), [0.90, 0.90, 0.88]);
            glass(&mut m, vec3(0.0, 2.0, cab_z + 1.12), vec3(1.15, 0.4, 0.02));
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 0.75, -0.6), ax, vec3(1.1, 0.25, l * 0.45), DARK);
            m.mat = mat::PAINT;
            let tank_len = l - 3.0;
            frustum(&mut m, vec3(0.0, 1.95, cab_z - 1.25), -Vec3::Z, tank_len, 1.15, 1.15, 14, [0.86, 0.87, 0.86], true);
            obox(&mut m, vec3(0.0, 1.95, cab_z - 1.25 - tank_len * 0.5), ax, vec3(1.17, 0.2, tank_len * 0.5), [0.10, 0.45, 0.22]);
            let axles: &[f32] = if v == Vehicle::Bowser { &[2.0, -2.2] } else { &[3.6, -2.0, -3.4] };
            wheels(&mut m, axles, 1.2, 0.52);
        }
        Vehicle::Catering => {
            obox(&mut m, vec3(0.0, 1.5, 3.2), ax, vec3(1.2, 1.0, 1.0), [0.90, 0.90, 0.88]);
            glass(&mut m, vec3(0.0, 1.95, 4.22), vec3(1.1, 0.38, 0.02));
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 0.8, -0.5), ax, vec3(1.1, 0.3, 3.6), DARK);
            // Scissor lift (raised) and the box body.
            m.mat = mat::METAL;
            for s in [-1.0f32, 1.0] {
                let a = vec3(s * 0.9, 1.1, -3.0);
                let b = vec3(s * 0.9, 3.4, 1.5);
                frustum(&mut m, a, b - a, (b - a).length(), 0.08, 0.08, 4, STEEL, false);
                let a = vec3(s * 0.9, 1.1, 1.5);
                let b = vec3(s * 0.9, 3.4, -3.0);
                frustum(&mut m, a, b - a, (b - a).length(), 0.08, 0.08, 4, STEEL, false);
            }
            m.mat = mat::PAINT;
            obox(&mut m, vec3(0.0, 4.9, -0.7), ax, vec3(1.25, 1.3, 2.9), [0.94, 0.94, 0.94]);
            obox(&mut m, vec3(0.0, 4.9, -0.7), ax, vec3(1.27, 0.15, 2.92), body);
            wheels(&mut m, &[2.9, -2.6], 1.2, 0.5);
        }
        Vehicle::Bus => {
            let c = [[0.92, 0.92, 0.90], [0.20, 0.40, 0.70], [0.88, 0.60, 0.10]][(livery % 3) as usize];
            obox(&mut m, vec3(0.0, 1.55, 0.0), ax, vec3(1.5, 1.25, 6.4), c);
            m.mat = mat::CANOPY;
            for s in [-1.0f32, 1.0] {
                obox(&mut m, vec3(s * 1.51, 1.85, 0.0), ax, vec3(0.02, 0.62, 6.0), [0.05, 0.06, 0.07]);
            }
            obox(&mut m, vec3(0.0, 1.8, 6.41), ax, vec3(1.35, 0.8, 0.02), [0.05, 0.06, 0.07]);
            m.mat = mat::PAINT;
            obox(&mut m, vec3(0.0, 2.85, 0.0), ax, vec3(1.45, 0.05, 6.2), [0.75, 0.76, 0.77]);
            wheels(&mut m, &[4.6, -4.4], 1.4, 0.45);
        }
        Vehicle::BeltLoader => {
            obox(&mut m, vec3(0.0, 0.75, 0.0), ax, vec3(0.9, 0.35, 2.6), body);
            obox(&mut m, vec3(0.6, 1.5, -1.6), ax, vec3(0.3, 0.4, 0.4), [0.2, 0.2, 0.22]);
            // Inclined belt: from low at the back to the cargo door ahead.
            let a = vec3(0.0, 1.0, -3.2);
            let b = vec3(0.0, 3.0, 3.8);
            let d = (b - a).normalize();
            let side = Vec3::X;
            let up = d.cross(side).normalize() * -1.0;
            let up = if up.y < 0.0 { -up } else { up };
            m.mat = mat::DIFFUSE;
            obox(&mut m, (a + b) * 0.5, [side, up, d], vec3(0.55, 0.12, (b - a).length() * 0.5), [0.12, 0.12, 0.12]);
            m.mat = mat::PAINT;
            for s in [-1.0f32, 1.0] {
                obox(&mut m, (a + b) * 0.5 + side * (s * 0.6) + up * 0.3, [side, up, d], vec3(0.04, 0.25, (b - a).length() * 0.5), [0.85, 0.85, 0.85]);
            }
            wheels(&mut m, &[1.8, -1.8], 1.0, 0.35);
        }
        Vehicle::Gpu => {
            obox(&mut m, vec3(0.0, 0.85, 0.0), ax, vec3(0.75, 0.5, 1.2), [0.90, 0.85, 0.20]);
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 1.0, 1.22), ax, vec3(0.5, 0.3, 0.02), DARK);
            wheels(&mut m, &[0.8, -0.8], 0.85, 0.3);
        }
        Vehicle::Car => {
            let c = [[0.95, 0.75, 0.10], [0.92, 0.92, 0.92], [0.12, 0.13, 0.15], [0.55, 0.06, 0.06]][(livery % 4) as usize];
            obox(&mut m, vec3(0.0, 0.7, 0.0), ax, vec3(0.88, 0.35, 2.25), c);
            obox(&mut m, vec3(0.0, 1.3, -0.2), ax, vec3(0.8, 0.28, 1.2), c);
            glass(&mut m, vec3(0.0, 1.3, -0.2), vec3(0.82, 0.24, 1.15));
            // Roof beacon bar (follow-me / ops cars).
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 1.63, -0.2), ax, vec3(0.5, 0.06, 0.12), [0.95, 0.55, 0.05]);
            wheels(&mut m, &[1.4, -1.4], 0.84, 0.33);
        }
        Vehicle::FireTruck => {
            // Airport crash tender: big, low, 8×8, roof monitor.
            let red = [0.78, 0.08, 0.06];
            obox(&mut m, vec3(0.0, 2.0, 3.6), ax, vec3(1.45, 1.25, 1.6), red);
            glass(&mut m, vec3(0.0, 2.55, 5.22), vec3(1.3, 0.55, 0.02));
            obox(&mut m, vec3(0.0, 2.1, -1.6), ax, vec3(1.45, 1.35, 3.6), red);
            obox(&mut m, vec3(0.0, 1.0, -1.6), ax, vec3(1.47, 0.12, 3.62), [0.9, 0.9, 0.88]);
            m.mat = mat::METAL;
            frustum(&mut m, vec3(0.0, 3.45, 3.0), vec3(0.0, 0.3, 1.0).normalize(), 2.2, 0.12, 0.08, 6, STEEL, false);
            m.mat = mat::DIFFUSE;
            obox(&mut m, vec3(0.0, 3.5, 4.6), ax, vec3(0.9, 0.08, 0.15), [0.2, 0.3, 0.95]);
            wheels(&mut m, &[4.3, 2.2, -2.6, -4.6], 1.55, 0.6);
        }
        Vehicle::Stairs => {
            obox(&mut m, vec3(0.0, 0.75, 1.5), ax, vec3(1.0, 0.35, 1.8), body);
            obox(&mut m, vec3(0.0, 1.6, 2.6), ax, vec3(0.9, 0.5, 0.7), [0.9, 0.9, 0.88]);
            m.mat = mat::METAL;
            let a = vec3(0.0, 1.0, -3.0);
            let b = vec3(0.0, 3.3, 0.8);
            let d = (b - a).normalize();
            let up = d.cross(Vec3::X).normalize() * -1.0;
            let up = if up.y < 0.0 { -up } else { up };
            obox(&mut m, (a + b) * 0.5, [Vec3::X, up, d], vec3(0.6, 0.15, (b - a).length() * 0.5), [0.75, 0.76, 0.78]);
            for s in [-1.0f32, 1.0] {
                obox(&mut m, (a + b) * 0.5 + Vec3::X * (s * 0.6) + up * 0.55, [Vec3::X, up, d], vec3(0.03, 0.03, (b - a).length() * 0.5), STEEL);
            }
            obox(&mut m, b + vec3(0.0, 0.0, 0.6), ax, vec3(0.65, 0.08, 0.6), [0.75, 0.76, 0.78]);
            wheels(&mut m, &[2.5, -0.5], 1.05, 0.38);
        }
    }
    m
}

// ---------------------------------------------------------------- signs

/// Airport sign panel styles (ICAO Annex 14).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Panel {
    /// Mandatory instruction: white on red (runway holding positions).
    Mandatory,
    /// Location: yellow on black (the taxiway you are on).
    Location,
    /// Direction / destination: black on yellow.
    Direction,
}

/// A sign: panels side by side on a frangible frame, facing +Z (readable
/// from +Z), centred on x = 0, standing on y = 0. `size`: legend height (m).
/// Internally lit at night (NIGHT_EMISSIVE faces). `back`: the back carries
/// the same legend (mandatory signs are double-faced).
pub fn sign(panels: &[(Panel, String)], size: f32, back: bool) -> MeshData {
    let mut m = MeshData::default();
    let h = size;
    let pad = h * 0.35;
    let gap = h * 0.12;
    let widths: Vec<f32> = panels.iter().map(|(_, t)| font::width(t) * h / 6.0 + 2.0 * pad).collect();
    let total: f32 = widths.iter().sum::<f32>() + gap * (panels.len().saturating_sub(1)) as f32;
    let ph = h + 2.0 * pad;
    let y0 = h * 0.55;
    let depth = 0.18;
    // Frame / box.
    m.mat = mat::DIFFUSE;
    aabox(&mut m, vec3(-total * 0.5 - 0.05, y0 - 0.05, -depth), vec3(total * 0.5 + 0.05, y0 + ph + 0.05, depth * 0.2), [0.18, 0.18, 0.19]);
    for s in [-1.0f32, 1.0] {
        aabox(&mut m, vec3(s * total * 0.38 - 0.05, 0.0, -depth * 0.6), vec3(s * total * 0.38 + 0.05, y0, -depth * 0.3), [0.5, 0.5, 0.52]);
    }
    let mut x = -total * 0.5;
    for ((kind, text), w) in panels.iter().zip(&widths) {
        let (bg, fg, bg_lum, fg_lum) = match kind {
            Panel::Mandatory => (SIGN_RED, [0.95, 0.95, 0.95], 60.0, 160.0),
            Panel::Location => (SIGN_BLACK, SIGN_YELLOW, 0.0, 120.0),
            Panel::Direction => (SIGN_YELLOW, SIGN_BLACK, 110.0, 0.0),
        };
        for face in if back { &[1.0f32, -1.0][..] } else { &[1.0f32][..] } {
            let z = if *face > 0.0 { depth * 0.2 + 0.005 } else { -depth - 0.005 };
            let n = Vec3::Z * *face;
            // Background (lit at night: NIGHT_EMISSIVE, uv.y = 1 → also lit by day).
            m.mat = mat::NIGHT_EMISSIVE;
            let (a, b) = (x, x + w);
            let p = |xx: f32, yy: f32| vec3(if *face > 0.0 { xx } else { -xx }, yy, z);
            let i0 = m.v_uv(p(a, y0), n, bg, [bg_lum, 1.0]);
            m.v_uv(p(b, y0), n, bg, [bg_lum, 1.0]);
            m.v_uv(p(b, y0 + ph), n, bg, [bg_lum, 1.0]);
            m.v_uv(p(a, y0 + ph), n, bg, [bg_lum, 1.0]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
            // Legend, a hair in front.
            let (quads, tw) = font::quads(text, h, font::STROKE);
            let ox = a + (w - tw) * 0.5;
            let z2 = z + 0.006 * *face;
            for q in quads {
                let pp = |v: Vec2| vec3(if *face > 0.0 { ox + v.x } else { -(ox + v.x) }, y0 + pad + v.y, z2);
                let j0 = m.v_uv(pp(q[0]), n, fg, [fg_lum, 1.0]);
                m.v_uv(pp(q[1]), n, fg, [fg_lum, 1.0]);
                m.v_uv(pp(q[2]), n, fg, [fg_lum, 1.0]);
                m.v_uv(pp(q[3]), n, fg, [fg_lum, 1.0]);
                m.idx.extend([j0, j0 + 1, j0 + 2, j0, j0 + 2, j0 + 3]);
            }
            // Location panels have a yellow border.
            if *kind == Panel::Location {
                let bw = h * 0.07;
                for (xa, ya, xb, yb) in [
                    (a + bw, y0 + bw, b - bw, y0 + 2.0 * bw),
                    (a + bw, y0 + ph - 2.0 * bw, b - bw, y0 + ph - bw),
                    (a + bw, y0 + bw, a + 2.0 * bw, y0 + ph - bw),
                    (b - 2.0 * bw, y0 + bw, b - bw, y0 + ph - bw),
                ] {
                    let j0 = m.v_uv(p(xa, ya) + vec3(0.0, 0.0, 0.006 * *face), n, fg, [fg_lum, 1.0]);
                    m.v_uv(p(xb, ya) + vec3(0.0, 0.0, 0.006 * *face), n, fg, [fg_lum, 1.0]);
                    m.v_uv(p(xb, yb) + vec3(0.0, 0.0, 0.006 * *face), n, fg, [fg_lum, 1.0]);
                    m.v_uv(p(xa, yb) + vec3(0.0, 0.0, 0.006 * *face), n, fg, [fg_lum, 1.0]);
                    m.idx.extend([j0, j0 + 1, j0 + 2, j0, j0 + 2, j0 + 3]);
                }
            }
        }
        x += w + gap;
    }
    m
}

// ---------------------------------------------------------------- masts, tanks

/// Apron floodlight mast (`h` m) with a lamp head facing `facing` (unit,
/// horizontal). Returns the mesh and the lamp position.
pub fn flood_mast(h: f32, facing: Vec2) -> (MeshData, Vec3) {
    let mut m = MeshData::default();
    m.mat = mat::METAL;
    frustum(&mut m, Vec3::ZERO, Vec3::Y, h, 0.38, 0.16, 8, [0.62, 0.63, 0.65], false);
    m.mat = mat::DIFFUSE;
    cyl(&mut m, vec3(0.0, -0.5, 0.0), 0.75, 0.8, 8, CONCRETE);
    let f = vec3(facing.x, 0.0, facing.y).normalize_or(Vec3::Z);
    let s = Vec3::Y.cross(f).normalize();
    let head = vec3(0.0, h, 0.0) + f * 0.6;
    m.mat = mat::METAL;
    obox(&mut m, head, [s, Vec3::Y, f], vec3(1.9, 0.12, 0.12), [0.4, 0.4, 0.42]);
    obox(&mut m, head + Vec3::Y * 0.9, [s, Vec3::Y, f], vec3(1.9, 0.12, 0.12), [0.4, 0.4, 0.42]);
    // Lamp housings tilted down toward the apron, lenses lit at night.
    for k in 0..6 {
        let x = (k % 3) as f32 - 1.0;
        let y = (k / 3) as f32 * 0.9;
        let c = head + s * (x * 1.2) + Vec3::Y * (y + 0.2) + f * 0.25;
        let fd = (f + Vec3::Y * -0.5).normalize();
        let up = fd.cross(s).normalize();
        m.mat = mat::DIFFUSE;
        obox(&mut m, c, [s, up, fd], vec3(0.45, 0.32, 0.18), [0.25, 0.25, 0.27]);
        m.mat = mat::NIGHT_EMISSIVE;
        let lc = c + fd * 0.19;
        let i0 = m.v_uv(lc + s * -0.4 + up * -0.27, fd, [1.0, 0.92, 0.78], [9000.0, 0.0]);
        m.v_uv(lc + s * 0.4 + up * -0.27, fd, [1.0, 0.92, 0.78], [9000.0, 0.0]);
        m.v_uv(lc + s * 0.4 + up * 0.27, fd, [1.0, 0.92, 0.78], [9000.0, 0.0]);
        m.v_uv(lc + s * -0.4 + up * 0.27, fd, [1.0, 0.92, 0.78], [9000.0, 0.0]);
        m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
    }
    (m, head + Vec3::Y * 0.45 + f * 0.5)
}

/// Vertical storage tank (fuel farm).
pub fn tank(m: &mut MeshData, c: Vec2, r: f32, h: f32) {
    m.mat = mat::METAL;
    frustum(m, vec3(c.x, -0.3, c.y), Vec3::Y, h + 0.3, r, r, 24, [0.86, 0.87, 0.87], false);
    m.mat = mat::DIFFUSE;
    frustum(m, vec3(c.x, h, c.y), Vec3::Y, r * 0.18, r * 1.01, r * 0.2, 24, [0.78, 0.79, 0.79], true);
    // Spiral stair (a band) and a walkway rail on top.
    m.mat = mat::METAL;
    frustum(m, vec3(c.x, h - 0.05, c.y), Vec3::Y, 1.0, r * 1.02, r * 1.02, 24, [0.5, 0.5, 0.52], false);
}

/// Airport fire station: a clad appliance hall with big red doors facing
/// `front` (+Z local), a hose-drying tower and a crew block.
pub fn fire_station(seed: u64) -> MeshData {
    let mut m = MeshData::default();
    let (w, d, h) = (46.0, 22.0, 9.0);
    let ring = vec![vec2(-w * 0.5, -d * 0.5), vec2(w * 0.5, -d * 0.5), vec2(w * 0.5, d * 0.5), vec2(-w * 0.5, d * 0.5)];
    m.mat = mat::CLADDING;
    prism(&mut m, &ring, -0.5, h, [0.86, 0.86, 0.84], Some((mat::ROOF, [0.52, 0.53, 0.55])));
    // Doors along the front.
    let n = 5;
    for i in 0..n {
        let x = -w * 0.5 + w * (i as f32 + 0.5) / n as f32;
        m.mat = mat::CLADDING;
        aabox(&mut m, vec3(x - 3.6, 0.0, d * 0.5), vec3(x + 3.6, 6.2, d * 0.5 + 0.15), [0.80, 0.10, 0.08]);
        m.mat = mat::GLASS;
        aabox(&mut m, vec3(x - 3.4, 4.6, d * 0.5 + 0.16), vec3(x + 3.4, 5.6, d * 0.5 + 0.2), GLASS_TINT);
    }
    // Crew block with windows (masonry), hose tower.
    m.mat = mat::WALL;
    let crew = vec![vec2(w * 0.5, -d * 0.5), vec2(w * 0.5 + 14.0, -d * 0.5), vec2(w * 0.5 + 14.0, d * 0.5 - 4.0), vec2(w * 0.5, d * 0.5 - 4.0)];
    prism(&mut m, &crew, -0.5, 7.5, [0.80, 0.78, 0.74], Some((mat::ROOF, [0.45, 0.45, 0.47])));
    m.mat = mat::WALL;
    let tower = vec![vec2(-w * 0.5 - 6.0, -d * 0.5), vec2(-w * 0.5, -d * 0.5), vec2(-w * 0.5, -d * 0.5 + 6.0), vec2(-w * 0.5 - 6.0, -d * 0.5 + 6.0)];
    prism(&mut m, &tower, -0.5, 18.0 + (seed % 4) as f32, [0.84, 0.83, 0.80], Some((mat::ROOF, [0.40, 0.40, 0.42])));
    // "FIRE" on the fascia, red.
    let (qs, tw) = font::quads("FIRE", 1.6, 1.0);
    m.mat = mat::DIFFUSE;
    for q in qs {
        let p = |v: Vec2| vec3(-tw * 0.5 + v.x, 6.8 + v.y, d * 0.5 + 0.05);
        m.quad_n(p(q[0]), p(q[1]), p(q[2]), p(q[3]), Vec3::Z, [0.80, 0.08, 0.06]);
    }
    m
}

/// Fuel farm: four tanks in a bunded compound (local frame, centre at the origin).
pub fn fuel_farm(seed: u64) -> MeshData {
    let mut m = MeshData::default();
    let r = 9.0 + (seed % 3) as f32;
    let s = r * 2.6;
    m.mat = mat::DIFFUSE;
    // Bund wall.
    let hw = s + r + 6.0;
    for (a, b) in [(vec2(-hw, -hw), vec2(hw, -hw)), (vec2(hw, -hw), vec2(hw, hw)), (vec2(hw, hw), vec2(-hw, hw)), (vec2(-hw, hw), vec2(-hw, -hw))] {
        let c = (a + b) * 0.5;
        let e = b - a;
        ybox(&mut m, vec3(c.x, 0.4, c.y), e.x.atan2(e.y), vec3(0.3, 0.9, e.length() * 0.5 + 0.3), CONCRETE);
    }
    for (i, (x, z)) in [(-1.0f32, -1.0f32), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)].iter().enumerate() {
        let h = 12.0 + ((seed >> (i * 3)) % 3) as f32;
        tank(&mut m, vec2(x * s * 0.5, z * s * 0.5), r, h);
    }
    m
}

/// Windsock mast (6 m, with its swivel frame) — the sock itself is dynamic.
pub fn windsock_mast() -> MeshData {
    let mut m = MeshData::default();
    m.mat = mat::METAL;
    frustum(&mut m, Vec3::ZERO, Vec3::Y, WINDSOCK_H, 0.09, 0.06, 8, [0.85, 0.85, 0.86], false);
    m.mat = mat::DIFFUSE;
    cyl(&mut m, vec3(0.0, -0.2, 0.0), 0.5, 0.35, 8, CONCRETE);
    // A small ring of white edge markers around the base.
    m
}

pub const WINDSOCK_H: f32 = 6.2;
const SOCK_LEN: f32 = 3.6;

/// The sock (eye-relative or world, `top` = swivel point): five alternating
/// orange / white bands, pointing downwind (`toward`, unit horizontal),
/// drooping as the wind drops (`fill` 0 limp .. 1 fully extended), with a
/// little flutter.
pub fn windsock_sock(m: &mut MeshData, top: Vec3, toward: Vec2, fill: f32, time: f32, seed: f32) {
    let f = fill.clamp(0.0, 1.0);
    let droop = (1.0 - f) * 1.25 + 0.06;
    let az = vec3(toward.x, 0.0, toward.y).normalize_or(Vec3::Z);
    let side = Vec3::Y.cross(az).normalize();
    let mut p = top;
    let segs = 5;
    let mut prev_r = 0.45;
    let mut prev = p;
    let flutter = |k: f32| (time * (6.0 + 3.0 * seed) + k * 1.7).sin() * 0.06 * (0.3 + f);
    m.mat = mat::DIFFUSE;
    // Ring frame.
    frustum(m, top - az * 0.05, az, 0.12, 0.47, 0.47, 12, [0.3, 0.3, 0.32], false);
    for i in 0..segs {
        let t1 = (i + 1) as f32 / segs as f32;
        let ang = droop * (0.35 + 0.65 * t1) + flutter(i as f32) * 0.5;
        let dir = (az * ang.cos() - Vec3::Y * ang.sin() + side * flutter(i as f32 + 0.5)).normalize();
        let seg_len = SOCK_LEN / segs as f32 * (0.9 + 0.1 * f);
        let next = p + dir * seg_len;
        // Taper (more when limp: the cloth collapses).
        let r = 0.45 - (0.45 - 0.17) * t1 - (1.0 - f) * 0.1 * t1;
        let col = if i % 2 == 0 { ORANGE } else { [0.94, 0.94, 0.92] };
        frustum(m, prev, next - prev, (next - prev).length(), prev_r, r.max(0.08), 12, col, false);
        prev = next;
        prev_r = r.max(0.08);
        p = next;
    }
}

/// Unit rotation: body frame (+Z nose) to a heading (rad, 0 = north = -Z,
/// clockwise) on the ground.
pub fn heading_quat(hdg: f32) -> Quat {
    // Body +Z → world direction (sin h, 0, -cos h): rotation about +Y by (π - h).
    Quat::from_rotation_y(PI - hdg)
}

/// World direction of a heading (rad): (sin h, -cos h) in (x, z).
pub fn heading_dir(hdg: f32) -> Vec2 {
    vec2(hdg.sin(), -hdg.cos())
}

/// Heading (rad) of a world direction.
pub fn dir_heading(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// Placement of a local mesh (+Z forward) at `p` facing heading `hdg`.
pub fn place(p: Vec3, hdg: f32) -> Mat4 {
    Mat4::from_rotation_translation(heading_quat(hdg), p)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn headings_round_trip() {
        for h in [0.0f32, 0.7, 1.5, 3.0, -2.0] {
            let d = heading_dir(h);
            let q = heading_quat(h) * Vec3::Z;
            assert!((vec2(q.x, q.z) - d).length() < 1e-5, "{h}");
            assert!((dir_heading(d) - h).rem_euclid(TAU).min((h - dir_heading(d)).rem_euclid(TAU)) < 1e-4);
        }
    }

    #[test]
    fn meshes_are_well_formed() {
        let mut all = vec![
            control_tower(60.0, 4.0, 1).0,
            control_tower(12.0, 3.0, 2).0,
            sign(&[(Panel::Location, "A".into()), (Panel::Direction, "B\u{2192}".into())], 0.8, false),
            flood_mast(25.0, Vec2::X).0,
            windsock_mast(),
        ];
        for v in VEHICLES {
            all.push(vehicle(v, 3));
        }
        let mut jb = MeshData::default();
        jet_bridge(&mut jb, Vec2::ZERO, vec2(0.0, 1.0), 4.5, vec3(3.0, 3.4, 25.0), vec2(1.0, 0.0), 7);
        all.push(jb);
        let mut h = MeshData::default();
        hangar(&mut h, [vec2(0.0, 0.0), vec2(60.0, 0.0), vec2(60.0, 40.0), vec2(0.0, 40.0)], 12.0, 4.0, Some(2), [0.7, 0.72, 0.74], 3);
        all.push(h);
        for m in &all {
            assert!(!m.idx.is_empty());
            assert!(m.idx.iter().all(|&i| (i as usize) < m.verts.len()));
            assert!(m.verts.iter().all(|v| v.pos.iter().all(|x| x.is_finite()) && v.nrm.iter().all(|x| x.is_finite())));
        }
    }
}
