//! Shared geometry kit: triangle soups, paint liveries (exact triangle
//! splitting along signed-distance fields), lofted fuselages, NACA wings with
//! control-surface cut-outs, wheels, propellers, nacelles and part helpers.

#![allow(clippy::too_many_arguments)]

use super::{AircraftModel, Part, PartKind};
pub use crate::scene::{mat, MeshData};
use glam::{vec2, vec3, Mat4, Quat, Vec2, Vec3};
use std::collections::HashMap;
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::rc::Rc;

// =================================================================== colours

pub const WHITE: [f32; 3] = [0.80, 0.80, 0.78];
pub const GELCOAT: [f32; 3] = [0.84, 0.84, 0.82];
pub const BLUE: [f32; 3] = [0.010, 0.030, 0.150];
pub const GOLD: [f32; 3] = [0.62, 0.36, 0.045];
pub const GLASS: [f32; 3] = [0.018, 0.026, 0.036];
pub const ANTIGLARE: [f32; 3] = [0.018, 0.018, 0.020];
pub const SEAM: [f32; 3] = [0.42, 0.42, 0.41];
pub const STRUT: [f32; 3] = [0.52, 0.53, 0.54];
pub const METAL: [f32; 3] = [0.35, 0.36, 0.37];
pub const DARK: [f32; 3] = [0.03, 0.03, 0.03];
pub const TYRE: [f32; 3] = [0.018, 0.018, 0.018];
pub const RED: [f32; 3] = [0.62, 0.02, 0.015];
pub const NAV_RED: [f32; 3] = [0.9, 0.03, 0.02];
pub const NAV_GREEN: [f32; 3] = [0.03, 0.75, 0.12];
pub const LENS: [f32; 3] = [0.30, 0.32, 0.34];
pub const PROP_DARK: [f32; 3] = [0.025, 0.025, 0.027];
pub const PROP_TIP: [f32; 3] = [0.75, 0.50, 0.02];

#[derive(Clone, Copy)]
pub struct Surf {
    pub col: [f32; 3],
    pub mat: f32,
    /// Vertex uv (EMISSIVE: x = intensity, y > 0.5 = flame).
    pub uv: [f32; 2],
}

pub const fn paint(col: [f32; 3]) -> Surf {
    Surf { col, mat: mat::PAINT, uv: [0.0, 0.0] }
}
pub const fn matte(col: [f32; 3]) -> Surf {
    Surf { col, mat: mat::DIFFUSE, uv: [0.0, 0.0] }
}
pub const fn glass(col: [f32; 3]) -> Surf {
    Surf { col, mat: mat::CANOPY, uv: [0.0, 0.0] }
}

// =================================================================== livery

pub type Fun = Rc<dyn Fn(Vec3) -> f32>;

/// A scalar paint field: `cut` is used to split triangles (may be disabled
/// away from where it matters), `test` classifies (negative = inside).
#[derive(Clone)]
pub struct SF {
    pub cut: Fun,
    pub test: Fun,
}

/// One paint region. `cuts` are scalar fields whose zero sets bound the
/// region; triangles are split exactly along them so edges stay crisp.
#[derive(Clone)]
pub struct Layer {
    pub cuts: Vec<SF>,
    pub surf: Surf,
}

#[derive(Clone)]
pub struct Livery {
    pub base: Surf,
    pub layers: Vec<Layer>,
}

impl Livery {
    pub fn new(base: Surf) -> Self {
        Livery { base, layers: Vec::new() }
    }
    /// Region where all fields are negative (intersection of half-spaces / SDFs).
    pub fn all_neg(mut self, fields: Vec<SF>, surf: Surf) -> Self {
        self.layers.push(Layer { cuts: fields, surf });
        self
    }
    pub fn sdf(self, f: impl Fn(Vec3) -> f32 + 'static, surf: Surf) -> Self {
        self.all_neg(vec![sf(f)], surf)
    }
    /// Region lo(p) < 0 < hi(p)... expressed as: `a(p) > 0` and `b(p) < 0`, plus extra negative masks.
    pub fn band(self, a: impl Fn(Vec3) -> f32 + 'static, b: impl Fn(Vec3) -> f32 + 'static, masks: Vec<SF>, surf: Surf) -> Self {
        let mut f: Vec<SF> = vec![sf(move |p| -a(p)), sf(b)];
        f.extend(masks);
        self.all_neg(f, surf)
    }
    /// Surface of a (fully split) triangle. Fields are judged by their mean
    /// vertex value, i.e. consistently with the linear splitting.
    pub fn at(&self, p: &[Vec3; 3]) -> Surf {
        for l in &self.layers {
            let inside = |f: &SF| {
                let m = (f.test)(p[0]) + (f.test)(p[1]) + (f.test)(p[2]);
                if m.abs() < 1e-4 {
                    // all vertices on the zero set: judge by the centroid
                    (f.test)((p[0] + p[1] + p[2]) / 3.0) < 0.0
                } else {
                    m < 0.0
                }
            };
            if l.cuts.iter().all(inside) {
                return l.surf;
            }
        }
        self.base
    }
}

pub fn sf(f: impl Fn(Vec3) -> f32 + 'static) -> SF {
    let f: Fun = Rc::new(f);
    SF { cut: f.clone(), test: f }
}

// =================================================================== triangles

#[derive(Clone, Copy)]
pub struct Tri {
    pub p: [Vec3; 3],
    pub n: [Vec3; 3],
}

pub fn mirror_x(t: &[Tri]) -> Vec<Tri> {
    let m = |v: Vec3| vec3(-v.x, v.y, v.z);
    t.iter().map(|t| Tri { p: t.p.map(m), n: t.n.map(m) }).collect()
}

pub fn transform_tris(t: &mut [Tri], m: Mat4) {
    for t in t {
        t.p = t.p.map(|p| m.transform_point3(p));
        t.n = t.n.map(|n| m.transform_vector3(n));
    }
}

/// Split a triangle along the zero set of `f` (linear interpolation per edge).
pub fn split(t: &Tri, f: &dyn Fn(Vec3) -> f32, out: &mut Vec<Tri>) {
    let d = t.p.map(|p| {
        let v = f(p);
        if v.abs() < 1e-6 { 0.0 } else { v }
    });
    if d.iter().all(|&v| v >= 0.0) || d.iter().all(|&v| v <= 0.0) {
        out.push(*t);
        return;
    }
    let mut pos: Vec<(Vec3, Vec3)> = Vec::with_capacity(4);
    let mut neg: Vec<(Vec3, Vec3)> = Vec::with_capacity(4);
    for i in 0..3 {
        let j = (i + 1) % 3;
        let (pi, ni, di) = (t.p[i], t.n[i], d[i]);
        if di >= 0.0 {
            pos.push((pi, ni));
        }
        if di <= 0.0 {
            neg.push((pi, ni));
        }
        let dj = d[j];
        if (di > 0.0 && dj < 0.0) || (di < 0.0 && dj > 0.0) {
            // root of f along the edge (regula falsi; exact for linear fields)
            let (mut lo, mut hi, mut flo, mut fhi) = (0.0f32, 1.0f32, di, dj);
            let mut s = di / (di - dj);
            for _ in 0..12 {
                let fs = f(pi.lerp(t.p[j], s));
                if fs.abs() < 1e-7 {
                    break;
                }
                if (fs > 0.0) == (flo > 0.0) {
                    lo = s;
                    flo = fs;
                } else {
                    hi = s;
                    fhi = fs;
                }
                s = (lo + (hi - lo) * flo / (flo - fhi)).clamp(lo + (hi - lo) * 0.02, hi - (hi - lo) * 0.02);
            }
            let q = (pi.lerp(t.p[j], s), ni.lerp(t.n[j], s));
            pos.push(q);
            neg.push(q);
        }
    }
    for poly in [pos, neg] {
        for k in 1..poly.len().saturating_sub(1) {
            out.push(Tri { p: [poly[0].0, poly[k].0, poly[k + 1].0], n: [poly[0].1, poly[k].1, poly[k + 1].1] });
        }
    }
}

/// Push one triangle with winding made consistent with its vertex normals
/// (counter-clockwise seen from the side the normals point to).
pub fn push_tri(m: &mut MeshData, t: &Tri, s: Surf) {
    let g = (t.p[1] - t.p[0]).cross(t.p[2] - t.p[0]);
    if g.length_squared() < 1e-14 {
        return;
    }
    let gn = g.normalize();
    let n = t.n.map(|n| {
        let n = n.normalize_or_zero();
        if n == Vec3::ZERO { gn } else { n }
    });
    let order = if gn.dot(n[0] + n[1] + n[2]) < 0.0 { [0, 2, 1] } else { [0, 1, 2] };
    m.mat = s.mat;
    let base = m.verts.len() as u32;
    for k in order {
        m.v_uv(t.p[k], n[k], s.col, s.uv);
    }
    m.idx.extend([base, base + 1, base + 2]);
}

pub fn emit(m: &mut MeshData, tris: &[Tri], liv: &Livery) {
    let cuts: Vec<Fun> = liv.layers.iter().flat_map(|l| l.cuts.iter().map(|f| f.cut.clone())).collect();
    let mut a = Vec::new();
    let mut b = Vec::new();
    for t in tris {
        a.clear();
        a.push(*t);
        for f in &cuts {
            b.clear();
            for t in &a {
                split(t, f.as_ref(), &mut b);
            }
            std::mem::swap(&mut a, &mut b);
        }
        for t in &a {
            push_tri(m, t, liv.at(&t.p));
        }
    }
}

pub fn emit_plain(m: &mut MeshData, tris: &[Tri], s: Surf) {
    emit(m, tris, &Livery::new(s));
}

/// Merge bit-identical vertices.
pub fn weld(m: &mut MeshData) {
    let mut map: HashMap<[u32; 12], u32> = HashMap::new();
    let mut verts = Vec::with_capacity(m.verts.len() / 3);
    let mut remap = Vec::with_capacity(m.verts.len());
    for v in &m.verts {
        let mut key = [0u32; 12];
        let f = [v.pos[0], v.pos[1], v.pos[2], v.nrm[0], v.nrm[1], v.nrm[2], v.col[0], v.col[1], v.col[2], v.uv[0], v.uv[1], v.mat];
        for (k, x) in key.iter_mut().zip(f) {
            *k = x.to_bits();
        }
        let i = *map.entry(key).or_insert_with(|| {
            verts.push(*v);
            (verts.len() - 1) as u32
        });
        remap.push(i);
    }
    for i in &mut m.idx {
        *i = remap[*i as usize];
    }
    m.verts = verts;
}

// =================================================================== surfaces

pub fn spow(v: f32, e: f32) -> f32 {
    v.signum() * v.abs().powf(e)
}

pub fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn sign3(v: f32) -> f32 {
    if v > 1e-9 {
        1.0
    } else if v < -1e-9 {
        -1.0
    } else {
        0.0
    }
}

/// Triangulate a grid of points `rows[i][j]` with smooth (area-weighted)
/// normals, oriented away from each row's centroid.
pub fn grid(rows: &[Vec<Vec3>], wrap_j: bool, wrap_i: bool) -> Vec<Tri> {
    let ni = rows.len();
    let nj = rows[0].len();
    let qi = if wrap_i { ni } else { ni - 1 };
    let qj = if wrap_j { nj } else { nj - 1 };
    let mut nrm = vec![vec![Vec3::ZERO; nj]; ni];
    for i in 0..qi {
        let i1 = (i + 1) % ni;
        for j in 0..qj {
            let j1 = (j + 1) % nj;
            let (a, b, c, d) = (rows[i][j], rows[i1][j], rows[i1][j1], rows[i][j1]);
            let n = (c - a).cross(d - b);
            nrm[i][j] += n;
            nrm[i1][j] += n;
            nrm[i1][j1] += n;
            nrm[i][j1] += n;
        }
    }
    for i in 0..ni {
        if rows[i].iter().all(|p| p.distance(rows[i][0]) < 1e-6) {
            let s: Vec3 = nrm[i].iter().copied().sum();
            nrm[i].iter_mut().for_each(|n| *n = s);
        }
    }
    let mut vote = 0.0;
    for i in 0..ni {
        let c = rows[i].iter().copied().sum::<Vec3>() / nj as f32;
        for j in 0..nj {
            vote += sign3(nrm[i][j].dot(rows[i][j] - c));
        }
    }
    let flip = if vote < 0.0 { -1.0 } else { 1.0 };
    let mut out = Vec::with_capacity(qi * qj * 2);
    for i in 0..qi {
        let i1 = (i + 1) % ni;
        for j in 0..qj {
            let j1 = (j + 1) % nj;
            let p = [rows[i][j], rows[i1][j], rows[i1][j1], rows[i][j1]];
            let n = [nrm[i][j], nrm[i1][j], nrm[i1][j1], nrm[i][j1]].map(|n| n.normalize_or_zero() * flip);
            out.push(Tri { p: [p[0], p[1], p[2]], n: [n[0], n[1], n[2]] });
            out.push(Tri { p: [p[0], p[2], p[3]], n: [n[0], n[2], n[3]] });
        }
    }
    out
}

/// Flat polygon fan from the centroid; normal oriented along `hint`.
pub fn fan(poly: &[Vec3], hint: Vec3) -> Vec<Tri> {
    let n = poly.len();
    let c = poly.iter().copied().sum::<Vec3>() / n as f32;
    let mut nn = Vec3::ZERO;
    for i in 0..n {
        nn += (poly[i] - c).cross(poly[(i + 1) % n] - c);
    }
    let mut nn = nn.normalize_or_zero();
    if nn == Vec3::ZERO {
        nn = hint.normalize();
    }
    if nn.dot(hint) < 0.0 {
        nn = -nn;
    }
    (0..n)
        .map(|i| Tri { p: [c, poly[i], poly[(i + 1) % n]], n: [nn; 3] })
        .collect()
}

pub fn quad(a: Vec3, b: Vec3, c: Vec3, d: Vec3, hint: Vec3) -> Vec<Tri> {
    let mut n = (c - a).cross(d - b).normalize_or_zero();
    if n.dot(hint) < 0.0 {
        n = -n;
    }
    vec![Tri { p: [a, b, c], n: [n; 3] }, Tri { p: [a, c, d], n: [n; 3] }]
}

/// Tube along a→b with superellipse cross-sections; `prof` = (t, ra, rb)
/// where ra is the half-axis along `side` (projected ⊥ to the tube).
pub fn tube_rows(a: Vec3, b: Vec3, side: Vec3, prof: &[(f32, f32, f32)], n: f32, around: usize) -> Vec<Vec<Vec3>> {
    let d = b - a;
    let dn = d.normalize();
    let u = (side - dn * side.dot(dn)).normalize();
    let v = dn.cross(u);
    let e = 2.0 / n;
    prof.iter()
        .map(|&(t, ra, rb)| {
            (0..around)
                .map(|j| {
                    let ph = TAU * j as f32 / around as f32;
                    a + d * t + u * (ra * spow(ph.cos(), e)) + v * (rb * spow(ph.sin(), e))
                })
                .collect()
        })
        .collect()
}

/// Closed tube: grid plus flat end caps where the ends are open.
pub fn tube(a: Vec3, b: Vec3, side: Vec3, prof: &[(f32, f32, f32)], n: f32, around: usize) -> Vec<Tri> {
    let rows = tube_rows(a, b, side, prof, n, around);
    let mut t = grid(&rows, true, false);
    let d = (b - a).normalize();
    if prof[0].1 > 1e-5 {
        t.extend(fan(&rows[0], -d));
    }
    if prof.last().unwrap().1 > 1e-5 {
        t.extend(fan(rows.last().unwrap(), d));
    }
    t
}

pub fn cyl(a: Vec3, b: Vec3, r: f32, around: usize) -> Vec<Tri> {
    let side = if (b - a).normalize().dot(Vec3::Y).abs() > 0.9 { Vec3::X } else { Vec3::Y };
    tube(a, b, side, &[(0.0, r, r), (1.0, r, r)], 2.0, around)
}

/// Ellipsoid spanning a→b with half-axes ra (along `side`) and rb.
pub fn ellipsoid(a: Vec3, b: Vec3, side: Vec3, ra: f32, rb: f32, k: usize, around: usize) -> Vec<Tri> {
    let prof: Vec<_> = (0..=k)
        .map(|i| {
            let th = PI * i as f32 / k as f32;
            ((1.0 - th.cos()) * 0.5, ra * th.sin(), rb * th.sin())
        })
        .collect();
    tube(a, b, side, &prof, 2.0, around)
}

/// Teardrop fairing profile: rounded front, pointed tail; `tp` = fraction of max section.
pub fn teardrop(k: usize, tp: f32, ra: f32, rb: f32) -> Vec<(f32, f32, f32)> {
    (0..=k)
        .map(|i| {
            let t = i as f32 / k as f32;
            let t = if t < 0.5 { tp * (1.0 - (t * PI).cos()) } else { tp + (1.0 - tp) * ((t - 0.5) * 2.0).powf(1.15) };
            let f = if t < tp {
                let u = (tp - t) / tp;
                (1.0 - u * u).max(0.0).sqrt()
            } else {
                let u = (t - tp) / (1.0 - tp);
                (1.0 - u * u).max(0.0).powf(0.9)
            };
            (t, ra * f, rb * f)
        })
        .collect()
}

/// Box from 8 corners given by centre/half extents in an orthonormal frame.
pub fn obox(c: Vec3, ax: [Vec3; 3], h: Vec3) -> Vec<Tri> {
    let p = |x: f32, y: f32, z: f32| c + ax[0] * (x * h.x) + ax[1] * (y * h.y) + ax[2] * (z * h.z);
    let mut t = Vec::new();
    for (k, s) in [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0), (2, 1.0), (2, -1.0)] {
        let (u, v) = ((k + 1) % 3, (k + 2) % 3);
        let corner = |a: f32, b: f32| {
            let mut q = [0.0f32; 3];
            q[k] = s;
            q[u] = a;
            q[v] = b;
            p(q[0], q[1], q[2])
        };
        t.extend(quad(corner(-1., -1.), corner(1., -1.), corner(1., 1.), corner(-1., 1.), ax[k] * s));
    }
    t
}

/// Tyre (rounded torus) plus hub discs; axle along X.
pub fn wheel(c: Vec3, r: f32, width: f32, tyre: &mut Vec<Tri>, hub: &mut Vec<Tri>) {
    let (nu, nv) = (24, 8);
    let rt = width * 0.5;
    let rr = r * 0.30;
    let rows: Vec<Vec<Vec3>> = (0..nu)
        .map(|i| {
            let a = TAU * i as f32 / nu as f32;
            let radial = vec3(0.0, a.cos(), a.sin());
            (0..nv)
                .map(|j| {
                    let b = TAU * j as f32 / nv as f32;
                    let rad = r - rr + rr * spow(b.cos(), 0.7);
                    c + radial * rad + Vec3::X * (rt * spow(b.sin(), 0.7))
                })
                .collect()
        })
        .collect();
    tyre.extend(grid(&rows, true, true));
    let rh = r - rr * 1.6;
    hub.extend(cyl(c - Vec3::X * rt * 0.85, c + Vec3::X * rt * 0.85, rh, 24));
}

// =================================================================== fuselage loft

/// Monotone cubic (PCHIP) interpolant.
pub struct Pchip {
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    pub d: Vec<f32>,
}

impl Pchip {
    pub fn new(x: Vec<f32>, y: Vec<f32>) -> Self {
        let n = x.len();
        let h: Vec<f32> = (0..n - 1).map(|i| x[i + 1] - x[i]).collect();
        let del: Vec<f32> = (0..n - 1).map(|i| (y[i + 1] - y[i]) / h[i]).collect();
        let mut d = vec![0.0; n];
        d[0] = del[0];
        d[n - 1] = del[n - 2];
        for k in 1..n - 1 {
            if del[k - 1] * del[k] > 0.0 {
                let w1 = 2.0 * h[k] + h[k - 1];
                let w2 = h[k] + 2.0 * h[k - 1];
                d[k] = (w1 + w2) / (w1 / del[k - 1] + w2 / del[k]);
            }
        }
        Pchip { x, y, d }
    }
    pub fn at(&self, t: f32) -> f32 {
        let n = self.x.len();
        let t = t.clamp(self.x[0], self.x[n - 1]);
        let mut i = 0;
        while i < n - 2 && t > self.x[i + 1] {
            i += 1;
        }
        let h = self.x[i + 1] - self.x[i];
        let s = (t - self.x[i]) / h;
        let (s2, s3) = (s * s, s * s * s);
        (2.0 * s3 - 3.0 * s2 + 1.0) * self.y[i]
            + (s3 - 2.0 * s2 + s) * h * self.d[i]
            + (-2.0 * s3 + 3.0 * s2) * self.y[i + 1]
            + (s3 - s2) * h * self.d[i + 1]
    }
}

/// Fuselage keys: [s (m aft of datum), half-width, top y, bottom y, waist y, exponent top, exponent bottom].
pub type FusKey = [f32; 7];

pub struct Fuselage {
    pub c: Vec<Pchip>,
    pub s0: f32,
    pub s1: f32,
}

impl Fuselage {
    pub fn new(keys: &[FusKey]) -> Self {
        let xs: Vec<f32> = keys.iter().map(|k| k[0]).collect();
        let c = (1..7).map(|j| Pchip::new(xs.clone(), keys.iter().map(|k| k[j]).collect())).collect();
        Fuselage { c, s0: xs[0], s1: *xs.last().unwrap() }
    }
    pub fn param(&self, s: f32) -> [f32; 6] {
        [0, 1, 2, 3, 4, 5].map(|j| self.c[j].at(s))
    }
    pub fn top(&self, s: f32) -> f32 {
        self.c[1].at(s)
    }
    pub fn ring(&self, s: f32, z0: f32, around: usize) -> Vec<Vec3> {
        let [w, top, bot, waist, nt, nb] = self.param(s);
        (0..around)
            .map(|j| {
                let t = TAU * j as f32 / around as f32;
                let (st, ct) = t.sin_cos();
                let n = if ct >= 0.0 { nt } else { nb };
                let e = 2.0 / n;
                let x = w.max(0.0) * spow(st, e);
                let y = waist + if ct >= 0.0 { top - waist } else { waist - bot } * spow(ct, e);
                vec3(x, y, z0 - s)
            })
            .collect()
    }
    /// Like [`Fuselage::rows`], plus extra stations (e.g. window edges) merged in.
    pub fn rows_with(&self, z0: f32, step: f32, around: usize, dense: (f32, f32, f32), extra: &[f32]) -> Vec<Vec<Vec3>> {
        let base = self.rows(z0, step, around, dense);
        let mut st: Vec<f32> = base.iter().map(|r| z0 - r[0].z).collect();
        let mut add: Vec<f32> = extra.iter().copied().filter(|&s| s > self.s0 + 0.05 && s < self.s1 - 0.05).collect();
        st.append(&mut add);
        st.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut out: Vec<f32> = Vec::new();
        for s in st {
            if let Some(&l) = out.last() {
                if s - l < 0.03 {
                    // keep explicit extras over generated stations
                    if extra.iter().any(|&e| (e - s).abs() < 1e-5) {
                        *out.last_mut().unwrap() = s;
                    }
                    continue;
                }
            }
            out.push(s);
        }
        out.iter().map(|&s| self.ring(s, z0, around)).collect()
    }
    /// Stations placed by silhouette arc length (denser where the shape changes).
    /// `dense` = (s0, s1, factor): station density multiplier over a range.
    pub fn rows(&self, z0: f32, step: f32, around: usize, dense: (f32, f32, f32)) -> Vec<Vec<Vec3>> {
        let fine = 0.002;
        let n = ((self.s1 - self.s0) / fine).ceil() as usize;
        let mut acc = vec![0.0f32];
        let mut prev = self.param(self.s0);
        for i in 1..=n {
            let s = self.s0 + (self.s1 - self.s0) * i as f32 / n as f32;
            let p = self.param(s);
            let dd = (p[0] - prev[0]).powi(2) + (p[1] - prev[1]).powi(2) + (p[2] - prev[2]).powi(2);
            let w = if s > dense.0 && s < dense.1 { dense.2 } else { 1.0 };
            acc.push(acc[i - 1] + w * (fine * fine + 6.0 * dd).sqrt());
            prev = p;
        }
        let total = acc[n];
        let m = (total / step).ceil() as usize;
        let mut out = Vec::new();
        let mut k = 0;
        for i in 0..=m {
            let target = total * i as f32 / m as f32;
            while k < n && acc[k + 1] < target {
                k += 1;
            }
            let s = if k >= n {
                self.s1
            } else {
                let f = (target - acc[k]) / (acc[k + 1] - acc[k]).max(1e-9);
                self.s0 + (self.s1 - self.s0) * (k as f32 + f) / n as f32
            };
            out.push(self.ring(s, z0, around));
        }
        out
    }
}

/// Signed distance to a polygon (negative inside).
pub fn poly_sdf(p: Vec2, v: &[Vec2]) -> f32 {
    let mut d = (p - v[0]).length_squared();
    let mut s = 1.0;
    let n = v.len();
    let mut j = n - 1;
    for i in 0..n {
        let e = v[j] - v[i];
        let w = p - v[i];
        let b = w - e * (w.dot(e) / e.dot(e)).clamp(0.0, 1.0);
        d = d.min(b.length_squared());
        let c = [p.y >= v[i].y, p.y < v[j].y, e.x * w.y > e.y * w.x];
        if c.iter().all(|&x| x) || c.iter().all(|&x| !x) {
            s = -s;
        }
        j = i;
    }
    s * d.sqrt()
}

pub type Proj = Rc<dyn Fn(Vec3) -> Vec2>;

/// Only split triangles along `f` inside a 2D box of a projection, so its
/// zero set does not slice unrelated geometry (classification still uses
/// the full field).
pub fn masked(f: SF, proj: Proj, lo: Vec2, hi: Vec2) -> SF {
    let c = f.cut;
    SF {
        cut: Rc::new(move |p| {
            let q = proj(p);
            if q.cmplt(lo).any() || q.cmpgt(hi).any() { 1.0 } else { c(p) }
        }),
        test: f.test,
    }
}

/// Rounded polygon (inner polygon offset by `r`) in a 2D projection as a
/// smooth signed-distance field, inactive (+1) away from its bounding box.
pub fn rpoly(poly: &[Vec2], r: f32, proj: Proj) -> SF {
    let poly = poly.to_vec();
    let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
    for p in &poly {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let m = Vec2::splat(r + 0.15);
    let pr = proj.clone();
    masked(sf(move |p| poly_sdf(pr(p), &poly) - r), proj, lo - m, hi + m)
}

// =================================================================== wings

#[derive(Clone, Copy)]
pub struct Foil {
    pub m: f32,
    pub p: f32,
    pub t: f32,
}

impl Foil {
    pub const fn sym(t: f32) -> Foil {
        Foil { m: 0.0, p: 0.4, t }
    }
    pub fn yt(&self, x: f32) -> f32 {
        let x = x.clamp(0.0, 1.0);
        5.0 * self.t * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x)
    }
    pub fn yc(&self, x: f32) -> f32 {
        let (m, p) = (self.m, self.p);
        if m == 0.0 {
            0.0
        } else if x < p {
            m / (p * p) * (2.0 * p * x - x * x)
        } else {
            m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x)
        }
    }
    /// Section point: `tsc` scales thickness/camber, `cs` shrinks chord from the LE.
    pub fn pt(&self, x: f32, upper: bool, tsc: f32, cs: f32) -> Vec2 {
        let y = (self.yc(x) + if upper { self.yt(x) } else { -self.yt(x) }) * tsc;
        vec2(1.0 - (1.0 - x) * cs, y)
    }
}

#[derive(Clone, Copy)]
pub struct Frame {
    pub le: Vec3,
    pub aft: Vec3,
    pub up: Vec3,
    pub span: Vec3,
    pub chord: f32,
}

impl Frame {
    pub fn at(&self, p: Vec2, d: f32) -> Vec3 {
        self.le + self.aft * (p.x * self.chord) + self.up * (p.y * self.chord) + self.span * d
    }
}

/// Planform key: span parameter, leading-edge point, chord, twist (deg, nose
/// up), roll (deg about the chord axis; dihedral / fin = 90).
#[derive(Clone, Copy)]
pub struct Key {
    pub s: f32,
    pub le: Vec3,
    pub chord: f32,
    pub twist: f32,
    pub roll: f32,
}

pub fn key(s: f32, le: Vec3, chord: f32, twist: f32, roll: f32) -> Key {
    Key { s, le, chord, twist, roll }
}

pub fn key_frame(keys: &[Key], s: f32) -> Frame {
    let mut i = 0;
    while i < keys.len() - 2 && s > keys[i + 1].s {
        i += 1;
    }
    let (a, b) = (keys[i], keys[i + 1]);
    let t = ((s - a.s) / (b.s - a.s)).clamp(0.0, 1.0);
    let l = |x: f32, y: f32| x + (y - x) * t;
    let q = Quat::from_rotation_z(l(a.roll, b.roll).to_radians()) * Quat::from_rotation_x(-l(a.twist, b.twist).to_radians());
    Frame { le: a.le.lerp(b.le, t), aft: q * -Vec3::Z, up: q * Vec3::Y, span: q * Vec3::X, chord: l(a.chord, b.chord) }
}

#[derive(Clone, Copy)]
pub enum Cap {
    Open,
    Flat,
    Round(f32),
}

pub struct Seg {
    pub s0: f32,
    pub s1: f32,
    pub n: usize,
    /// Control-surface cut-out: hinge chord fraction at s0 and s1.
    pub cut: Option<(f32, f32)>,
    /// s0 is a mirror seam (no gap / no end walls there).
    pub seam0: bool,
}

pub fn seg(s0: f32, s1: f32, n: usize) -> Seg {
    Seg { s0, s1, n, cut: None, seam0: false }
}
pub fn cut(s0: f32, s1: f32, n: usize, h0: f32, h1: f32) -> Seg {
    Seg { s0, s1, n, cut: Some((h0, h1)), seam0: false }
}

pub struct Wing {
    pub frame: Box<dyn Fn(f32) -> Frame>,
    pub breaks: Vec<f32>,
    pub foil: Foil,
    pub segs: Vec<Seg>,
    pub cap0: Cap,
    pub cap1: Cap,
    pub k1: usize,
    pub k2: usize,
}

pub struct Ctrl {
    pub tris: Vec<Tri>,
    /// Hinge (nose-arc centre) at both span ends.
    pub a: Vec3,
    pub b: Vec3,
}

pub const HS_DEFAULT: f32 = 0.75;
pub const CHORD_GAP: f32 = 0.006;
pub const SPAN_GAP: f32 = 0.008;
pub const KARC: usize = 8;

pub fn le_samples(h: f32, k: usize) -> Vec<f32> {
    (0..=k).map(|i| h * (1.0 - (FRAC_PI_2 * i as f32 / k as f32).cos())).collect()
}
pub fn lin_samples(a: f32, b: f32, k: usize) -> Vec<f32> {
    (0..=k).map(|i| a + (b - a) * i as f32 / k as f32).collect()
}

impl Wing {
    pub fn fr(&self, s: f32) -> Frame {
        (self.frame)(s)
    }
    /// Full closed section TE(upper) → LE → TE(lower), sampled with a break at `hs`.
    pub fn ring_full(&self, f: &Frame, hs: f32, tsc: f32, cs: f32, d: f32) -> Vec<Vec3> {
        let a = le_samples(hs, self.k1);
        let b = lin_samples(hs, 1.0, self.k2);
        let fo = &self.foil;
        let mut r = Vec::new();
        r.extend(b.iter().rev().map(|&x| f.at(fo.pt(x, true, tsc, cs), d)));
        r.extend(a.iter().rev().skip(1).map(|&x| f.at(fo.pt(x, true, tsc, cs), d)));
        r.extend(a.iter().skip(1).map(|&x| f.at(fo.pt(x, false, tsc, cs), d)));
        r.extend(b.iter().skip(1).map(|&x| f.at(fo.pt(x, false, tsc, cs), d)));
        r
    }
    /// Section forward of the hinge: upper(h) → LE → lower(h).
    pub fn ring_main(&self, f: &Frame, h: f32) -> Vec<Vec3> {
        let a = le_samples(h, self.k1);
        let fo = &self.foil;
        let mut r: Vec<Vec3> = a.iter().rev().map(|&x| f.at(fo.pt(x, true, 1.0, 1.0), 0.0)).collect();
        r.extend(a.iter().skip(1).map(|&x| f.at(fo.pt(x, false, 1.0, 1.0), 0.0)));
        r
    }
    /// Hinge-arc centre chord fraction for a cut at `h`.
    pub fn xs(&self, f: &Frame, h: f32) -> f32 {
        let g = CHORD_GAP / f.chord;
        let mut xs = h + g;
        for _ in 0..4 {
            xs = h + g + self.foil.yt(xs);
        }
        xs
    }
    /// Control surface section: upper TE → hinge arc → lower TE.
    pub fn ring_ctrl(&self, f: &Frame, xs: f32) -> Vec<Vec3> {
        let b = lin_samples(xs, 1.0, self.k2);
        let fo = &self.foil;
        let (yc, r) = (fo.yc(xs), fo.yt(xs));
        let mut out: Vec<Vec3> = b.iter().rev().map(|&x| f.at(fo.pt(x, true, 1.0, 1.0), 0.0)).collect();
        for i in 1..KARC {
            let ph = PI * i as f32 / KARC as f32;
            out.push(f.at(vec2(xs - r * ph.sin(), yc + r * ph.cos()), 0.0));
        }
        out.extend(b.iter().map(|&x| f.at(fo.pt(x, false, 1.0, 1.0), 0.0)));
        out
    }
    pub fn stations(&self, s0: f32, s1: f32, n: usize) -> Vec<f32> {
        let mut v: Vec<f32> = (0..=n).map(|i| s0 + (s1 - s0) * i as f32 / n as f32).collect();
        v.extend(self.breaks.iter().copied().filter(|&b| b > s0 + 1e-4 && b < s1 - 1e-4));
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.dedup_by(|a, b| (*a - *b).abs() < 1e-4);
        v
    }
    pub fn cap_rows(&self, s: f32, r: f32, dir: f32) -> Vec<Vec<Vec3>> {
        let f = self.fr(s);
        let nk = 5;
        (0..=nk)
            .map(|k| {
                let th = FRAC_PI_2 * k as f32 / nk as f32;
                self.ring_full(&f, HS_DEFAULT, th.cos().max(0.0), 1.0 - 0.12 * (1.0 - th.cos()), dir * r * th.sin())
            })
            .collect()
    }

    pub fn build(&self) -> (Vec<Tri>, Vec<Ctrl>) {
        let mut out = Vec::new();
        let mut ctrls = Vec::new();
        let hinge: Vec<Option<(Vec3, Vec3)>> = self
            .segs
            .iter()
            .map(|sg| {
                sg.cut.map(|(h0, h1)| {
                    let (f0, f1) = (self.fr(sg.s0), self.fr(sg.s1));
                    (f0.at(vec2(h0, self.foil.yc(h0)), 0.0), f1.at(vec2(h1, self.foil.yc(h1)), 0.0))
                })
            })
            .collect();
        let h_at = |i: usize, s: f32| -> f32 {
            let (a, b) = hinge[i].unwrap();
            let sg = &self.segs[i];
            let t = (s - sg.s0) / (sg.s1 - sg.s0);
            let f = self.fr(s);
            (a.lerp(b, t) - f.le).dot(f.aft) / f.chord
        };
        let nseg = self.segs.len();
        let mut first_ring = None;
        let mut last_ring = None;
        for (i, sg) in self.segs.iter().enumerate() {
            let st = self.stations(sg.s0, sg.s1, sg.n);
            match sg.cut {
                None => {
                    let hs0 = if i > 0 && self.segs[i - 1].cut.is_some() { h_at(i - 1, sg.s0) } else { HS_DEFAULT };
                    let hs1 = if i + 1 < nseg && self.segs[i + 1].cut.is_some() { h_at(i + 1, sg.s1) } else { HS_DEFAULT };
                    let rows: Vec<Vec<Vec3>> = st
                        .iter()
                        .map(|&s| {
                            let t = (s - sg.s0) / (sg.s1 - sg.s0);
                            self.ring_full(&self.fr(s), hs0 + (hs1 - hs0) * t, 1.0, 1.0, 0.0)
                        })
                        .collect();
                    out.extend(grid(&rows, false, false));
                    if i == 0 {
                        first_ring = Some(rows[0].clone());
                    }
                    if i + 1 == nseg {
                        last_ring = Some(rows.last().unwrap().clone());
                    }
                }
                Some(_) => {
                    let rows: Vec<Vec<Vec3>> = st.iter().map(|&s| self.ring_main(&self.fr(s), h_at(i, s))).collect();
                    out.extend(grid(&rows, false, false));
                    if i == 0 {
                        first_ring = Some(rows[0].clone());
                    }
                    // spar wall (cove)
                    for w in rows.windows(2) {
                        let f = self.fr(st[0]);
                        let (l0, l1) = (w[0].len() - 1, w[1].len() - 1);
                        out.extend(quad(w[0][0], w[0][l0], w[1][l1], w[1][0], f.aft));
                    }
                    // cove end walls
                    let wall = |s: f32, hint: Vec3| -> Vec<Tri> {
                        let r = self.ring_full(&self.fr(s), h_at(i, s), 1.0, 1.0, 0.0);
                        let k2 = self.k2;
                        let n = r.len();
                        let mut poly: Vec<Vec3> = r[0..=k2].iter().rev().copied().collect();
                        poly.extend(r[n - 1 - k2..n - 1].iter().rev().copied());
                        fan(&poly, hint)
                    };
                    if !sg.seam0 {
                        out.extend(wall(sg.s0, self.fr(sg.s0).span));
                    }
                    out.extend(wall(sg.s1, -self.fr(sg.s1).span));
                    // control surface
                    let sa = if sg.seam0 { sg.s0 } else { sg.s0 + SPAN_GAP };
                    let sb = sg.s1 - SPAN_GAP;
                    let cst = self.stations(sa, sb, sg.n);
                    let crow: Vec<Vec<Vec3>> = cst
                        .iter()
                        .map(|&s| {
                            let f = self.fr(s);
                            self.ring_ctrl(&f, self.xs(&f, h_at(i, s)))
                        })
                        .collect();
                    let mut ct = grid(&crow, false, false);
                    if !sg.seam0 {
                        ct.extend(fan(&crow[0], -self.fr(sa).span));
                    }
                    ct.extend(fan(crow.last().unwrap(), self.fr(sb).span));
                    let hp = |s: f32| {
                        let f = self.fr(s);
                        let xs = self.xs(&f, h_at(i, s));
                        f.at(vec2(xs, self.foil.yc(xs)), 0.0)
                    };
                    ctrls.push(Ctrl { tris: ct, a: hp(sa), b: hp(sb) });
                }
            }
        }
        let (s_first, s_last) = (self.segs[0].s0, self.segs[nseg - 1].s1);
        match self.cap0 {
            Cap::Open => {}
            Cap::Flat => out.extend(fan(&first_ring.unwrap(), -self.fr(s_first).span)),
            Cap::Round(r) => out.extend(grid(&self.cap_rows(s_first, r, -1.0), false, false)),
        }
        match self.cap1 {
            Cap::Open => {}
            Cap::Flat => out.extend(fan(&last_ring.unwrap(), self.fr(s_last).span)),
            Cap::Round(r) => out.extend(grid(&self.cap_rows(s_last, r, 1.0), false, false)),
        }
        (out, ctrls)
    }
}

pub fn keyed_wing(keys: Vec<Key>, foil: Foil, segs: Vec<Seg>, cap0: Cap, cap1: Cap) -> Wing {
    let breaks = keys.iter().map(|k| k.s).collect();
    Wing { frame: Box::new(move |s| key_frame(&keys, s)), breaks, foil, segs, cap0, cap1, k1: 14, k2: 5 }
}

// =================================================================== parts

/// Pick the axis sign so a small positive rotation moves `probe` along `want`.
pub fn orient_axis(axis: Vec3, pivot: Vec3, probe: Vec3, want: Vec3) -> Vec3 {
    let r = probe - pivot;
    let d = Quat::from_axis_angle(axis, 0.05) * r - r;
    if d.dot(want) >= 0.0 { axis } else { -axis }
}

/// Point of the part farthest from its hinge axis (the trailing edge).
pub fn te_probe(tris: &[Tri], pivot: Vec3, axis: Vec3) -> Vec3 {
    let mut best = (0.0, pivot);
    for t in tris {
        for p in t.p {
            let r = p - pivot;
            let d = (r - axis * r.dot(axis)).length();
            if d > best.0 {
                best = (d, p);
            }
        }
    }
    best.1
}

pub fn make_part(kind: PartKind, mut mesh: MeshData, pivot: Vec3, axis: Vec3, max_angle: f32) -> Part {
    for v in &mut mesh.verts {
        v.pos = (Vec3::from(v.pos) - pivot).into();
    }
    weld(&mut mesh);
    Part { kind, mesh, pivot, axis: axis.normalize(), max_angle }
}

/// Hinged surface part from control-surface triangles in body coordinates.
pub fn hinged(kind: PartKind, tris: &[Tri], liv: &Livery, pivot: Vec3, axis: Vec3, want: Vec3, max_angle: f32) -> Part {
    let probe = te_probe(tris, pivot, axis.normalize());
    let axis = orient_axis(axis.normalize(), pivot, probe, want);
    let mut m = MeshData::default();
    emit(&mut m, tris, liv);
    make_part(kind, m, pivot, axis, max_angle)
}

pub fn finish(mut body: MeshData, parts: Vec<Part>, eye: Vec3) -> AircraftModel {
    weld(&mut body);
    AircraftModel { body, parts, eye }
}

// =================================================================== v2 kit

pub const fn metal(col: [f32; 3]) -> Surf {
    Surf { col, mat: mat::METAL, uv: [0.0, 0.0] }
}
/// Self-lit surface; `k` = intensity (scaled further by the app for lights / flame).
pub const fn emissive(col: [f32; 3], k: f32) -> Surf {
    Surf { col, mat: mat::EMISSIVE, uv: [k, 0.0] }
}
/// Additive-looking flame (alpha = colour luminance).
pub const fn flame(col: [f32; 3], k: f32) -> Surf {
    Surf { col, mat: mat::EMISSIVE, uv: [k, 1.0] }
}

pub const WHITE_LIGHT: [f32; 3] = [1.0, 0.97, 0.92];
pub const STROBE_COL: [f32; 3] = [1.0, 1.0, 1.0];
pub const BEACON_COL: [f32; 3] = [1.0, 0.06, 0.02];
pub const NAV_K: f32 = 6.0;
pub const STROBE_K: f32 = 24.0;
pub const BEACON_K: f32 = 12.0;

/// Body-space triangles to a part mesh with one surface.
pub fn plain_part(kind: PartKind, tris: &[Tri], s: Surf, pivot: Vec3, axis: Vec3, max_angle: f32) -> Part {
    let mut m = MeshData::default();
    emit_plain(&mut m, tris, s);
    make_part(kind, m, pivot, axis, max_angle)
}

/// Multi-surface part: `groups` of (tris, surf) in body coordinates.
pub fn multi_part(kind: PartKind, groups: &[(Vec<Tri>, Surf)], pivot: Vec3, axis: Vec3, max_angle: f32) -> Part {
    let mut m = MeshData::default();
    for (t, s) in groups {
        emit_plain(&mut m, t, *s);
    }
    make_part(kind, m, pivot, axis, max_angle)
}

/// Rotate body-space triangles about a hinge.
pub fn rotated(t: &[Tri], pivot: Vec3, axis: Vec3, ang: f32) -> Vec<Tri> {
    let m = Mat4::from_translation(pivot) * Mat4::from_axis_angle(axis.normalize(), ang) * Mat4::from_translation(-pivot);
    let mut t = t.to_vec();
    transform_tris(&mut t, m);
    t
}

/// A part authored in its *deflection = 1* pose (e.g. a closed gear door,
/// flush with the skin): the stored mesh is the deflection-0 pose, rotated
/// back by `-max_angle`, so that `transform(1)` reproduces the authored pose.
pub fn part_at_one(kind: PartKind, groups: &[(Vec<Tri>, Surf)], pivot: Vec3, axis: Vec3, max_angle: f32) -> Part {
    let g: Vec<(Vec<Tri>, Surf)> = groups.iter().map(|(t, s)| (rotated(t, pivot, axis, -max_angle), *s)).collect();
    multi_part(kind, &g, pivot, axis, max_angle)
}

/// Light lens: small ellipsoid along `dir` (length `l`) at `c`.
pub fn lens(c: Vec3, dir: Vec3, l: f32, r: f32) -> Vec<Tri> {
    let d = dir.normalize();
    let side = if d.dot(Vec3::Y).abs() > 0.9 { Vec3::X } else { Vec3::Y };
    ellipsoid(c - d * l * 0.5, c + d * l * 0.5, side, r, r, 6, 10)
}

/// Surface of revolution about `axis` through `c`: profile = (axial offset, radius).
/// `vscale` squashes the section along `side` x up (1 = round). Closed ends are capped
/// when `cap` is set and the end radius is non-zero.
pub fn lathe(c: Vec3, axis: Vec3, up: Vec3, prof: &[(f32, f32)], around: usize, squash: f32, cap: bool) -> Vec<Tri> {
    let ax = axis.normalize();
    let u = (up - ax * up.dot(ax)).normalize();
    let v = ax.cross(u);
    let rows: Vec<Vec<Vec3>> = prof
        .iter()
        .map(|&(z, r)| {
            (0..around)
                .map(|j| {
                    let a = TAU * j as f32 / around as f32;
                    c + ax * z + u * (r * squash * a.cos()) + v * (r * a.sin())
                })
                .collect()
        })
        .collect();
    let mut t = grid(&rows, true, false);
    if cap {
        if prof[0].1 > 1e-5 {
            t.extend(fan(&rows[0], -ax));
        }
        if prof.last().unwrap().1 > 1e-5 {
            t.extend(fan(rows.last().unwrap(), ax));
        }
    }
    t
}

/// Thin flat annulus/disc (one-sided, normal along `n`) in the plane through `c`.
pub fn annulus(c: Vec3, n: Vec3, r0: f32, r1: f32, around: usize) -> Vec<Tri> {
    let n = n.normalize();
    let side = if n.dot(Vec3::Y).abs() > 0.9 { Vec3::X } else { Vec3::Y };
    let u = (side - n * side.dot(n)).normalize();
    let v = n.cross(u);
    let ring = |r: f32| -> Vec<Vec3> { (0..around).map(|j| { let a = TAU * j as f32 / around as f32; c + (u * a.cos() + v * a.sin()) * r }).collect() };
    let mut out = Vec::new();
    if r0 < 1e-5 {
        out.extend(fan(&ring(r1), n));
    } else {
        let (a, b) = (ring(r0), ring(r1));
        for j in 0..around {
            let k = (j + 1) % around;
            out.extend(quad(a[j], b[j], b[k], a[k], n));
        }
    }
    out
}

/// Mirror a part-pair helper: mirror a point across the symmetry plane.
pub fn mx(v: Vec3) -> Vec3 {
    vec3(-v.x, v.y, v.z)
}

pub struct PropSpec {
    pub engine: u8,
    /// Hub centre in the disc plane (body coords).
    pub pivot: Vec3,
    pub radius: f32,
    pub blades: usize,
    /// (r/R, chord m) keys.
    pub chord: Vec<(f32, f32)>,
    /// Geometric pitch (m per revolution) of the blade at 0.75 R.
    pub pitch: f32,
    /// Blade root r/R and thickness ratio.
    pub root: f32,
    pub thick: f32,
    /// Spinner: (offset of base behind disc, length, radius).
    pub spinner: (f32, f32, f32),
    pub spinner_col: Surf,
    pub blade_col: [f32; 3],
    pub tip_col: [f32; 3],
    /// Tip band starts at this r/R (>= 1 = no band).
    pub tip_from: f32,
    /// Blade sweep at the tip (scimitar), as a fraction of tip chord (0 = straight).
    pub scimitar: f32,
    /// +1 = rotates with +angle about the thrust axis (+Z).
    pub disc_col: [f32; 3],
}

/// Propeller part (axis +Z = thrust axis, max_angle = 1): spinner, blades
/// (PROP verts uv.x = 0) and the blur disc (uv.x = 1); uv.y = r/R.
pub fn propeller(p: &PropSpec) -> Part {
    let pivot = p.pivot;
    let rad = p.radius;
    let mut m = MeshData::default();
    // spinner
    let (sb, sl, sr) = p.spinner;
    let n = 14;
    let rows: Vec<Vec<Vec3>> = (0..=n)
        .map(|i| {
            let t = i as f32 / n as f32;
            let r = sr * (1.0 - t.powf(1.9)).max(0.0).powf(0.55);
            (0..28).map(|j| {
                let a = TAU * j as f32 / 28.0;
                pivot + vec3(r * a.cos(), r * a.sin(), -sb + sl * t)
            }).collect()
        })
        .collect();
    let mut sp = grid(&rows, true, false);
    sp.extend(fan(&rows[0], -Vec3::Z));
    emit_plain(&mut m, &sp, p.spinner_col);

    // blade along +Y, LE towards -X (leads for +angle rotation about +Z)
    let chord = Pchip::new(p.chord.iter().map(|c| c.0 * rad).collect(), p.chord.iter().map(|c| c.1).collect());
    let pitch = p.pitch;
    let (sc, tipc) = (p.scimitar, p.chord.last().unwrap().1);
    let r0 = p.root * rad;
    let blade = Wing {
        frame: Box::new(move |r: f32| {
            let beta = (pitch / (TAU * r.max(0.05))).atan();
            let c = chord.at(r);
            let aft = vec3(beta.cos(), 0.0, -beta.sin());
            let up = vec3(beta.sin(), 0.0, beta.cos());
            let sweep = sc * tipc * ((r - 0.5 * rad) / (0.5 * rad)).max(0.0).powi(2);
            Frame { le: vec3(0.0, r, 0.0) - aft * (0.35 * c) + aft * sweep, aft, up, span: Vec3::Y, chord: c }
        }),
        breaks: vec![],
        foil: Foil { m: 0.04, p: 0.4, t: p.thick },
        segs: vec![seg(r0, rad * 0.985, 10)],
        cap0: Cap::Flat,
        cap1: Cap::Round(0.015 * rad),
        k1: 8,
        k2: 3,
    };
    let (b1, _) = blade.build();
    let mut bt = Vec::new();
    for k in 0..p.blades {
        let mut b = b1.clone();
        transform_tris(&mut b, Mat4::from_translation(pivot) * Mat4::from_rotation_z(TAU * k as f32 / p.blades as f32));
        bt.extend(b);
    }
    // blade root cuffs (round shanks into the spinner)
    let mut shanks = Vec::new();
    for k in 0..p.blades {
        let q = Mat4::from_translation(pivot) * Mat4::from_rotation_z(TAU * k as f32 / p.blades as f32);
        let mut s = cyl(vec3(0.0, sr * 0.6, 0.0), vec3(0.0, r0 + 0.02, 0.0), (p.chord[0].1 * 0.32).min(sr * 0.3), 10);
        transform_tris(&mut s, q);
        shanks.extend(s);
    }
    let surf = |c: [f32; 3]| Surf { col: c, mat: mat::PROP, uv: [0.0, 0.0] };
    let tip_from = p.tip_from * rad;
    let bliv = Livery::new(surf(p.blade_col)).sdf(move |q| tip_from - vec2(q.x - pivot.x, q.y - pivot.y).length(), surf(p.tip_col));
    let nb = m.verts.len();
    emit(&mut m, &bt, &bliv);
    emit_plain(&mut m, &shanks, surf(p.blade_col));
    for v in &mut m.verts[nb..] {
        let r = vec2(v.pos[0] - pivot.x, v.pos[1] - pivot.y).length();
        v.uv = [0.0, (r / rad).min(1.0)];
    }
    // disc (two-sided annulus), uv.x = 1
    let nd = m.verts.len();
    let mut disc = Vec::new();
    for (dz, nz) in [(0.004, 1.0f32), (-0.004, -1.0)] {
        let rows: Vec<Vec<Vec3>> = [sr * 0.95, rad * 0.55, rad]
            .iter()
            .map(|&r| (0..48).map(|j| {
                let a = TAU * j as f32 / 48.0;
                pivot + vec3(r * a.cos(), r * a.sin(), dz)
            }).collect())
            .collect();
        let mut g = grid(&rows, true, false);
        for t in &mut g {
            t.n = [Vec3::Z * nz; 3];
        }
        disc.extend(g);
    }
    emit_plain(&mut m, &disc, surf(p.disc_col));
    for v in &mut m.verts[nd..] {
        let r = vec2(v.pos[0] - pivot.x, v.pos[1] - pivot.y).length();
        v.uv = [1.0, (r / rad).min(1.0)];
    }
    make_part(PartKind::Propeller(p.engine), m, pivot, Vec3::Z, 1.0)
}

/// Rounded rectangle SDF (half extents `h`, corner radius `r`).
pub fn rrect(p: Vec2, h: Vec2, r: f32) -> f32 {
    let q = p.abs() - h + Vec2::splat(r);
    q.max(Vec2::ZERO).length() + q.x.max(q.y).min(0.0) - r
}

/// Periodic row of rounded windows along s: returns SDF (negative inside a window).
pub fn window_row(s: f32, y: f32, s0: f32, pitch: f32, count: usize, w: f32, h: f32, r: f32, yc: f32) -> f32 {
    let k = ((s - s0) / pitch).round().clamp(0.0, count as f32 - 1.0);
    let ds = s - (s0 + k * pitch);
    rrect(vec2(ds, y - yc), vec2(w * 0.5, h * 0.5), r)
}

/// Rigid fold: the rotation (about a hinge through the origin of `v0`) that
/// maps direction `v0` onto `v1` and, among those, brings `x0` closest to `xt`
/// (sign-agnostic: `xt` and `-xt` are both acceptable). Returns (axis, angle).
pub fn fold(v0: Vec3, v1: Vec3, x0: Vec3, xt: Vec3) -> (Vec3, f32) {
    let (v0, v1) = (v0.normalize(), v1.normalize());
    let qa = Quat::from_rotation_arc(v0, v1);
    let x1 = qa * x0;
    let perp = |v: Vec3| (v - v1 * v.dot(v1)).normalize_or_zero();
    let (a, mut b) = (perp(x1), perp(xt));
    if a.dot(b) < 0.0 {
        b = -b;
    }
    let phi = a.cross(b).dot(v1).atan2(a.dot(b));
    let q = Quat::from_axis_angle(v1, phi) * qa;
    let (axis, ang) = q.to_axis_angle();
    (axis, ang)
}

/// Thin slab panel: `outer[i][j]` sampled on a surface with outward normals
/// `nrm[i][j]`; the panel is `t` thick (inner face = outer - n t).
pub fn slab(outer: &[Vec<Vec3>], nrm: &[Vec<Vec3>], t: f32) -> Vec<Tri> {
    let (ni, nj) = (outer.len(), outer[0].len());
    let inner: Vec<Vec<Vec3>> = (0..ni).map(|i| (0..nj).map(|j| outer[i][j] - nrm[i][j] * t).collect()).collect();
    let mut out = Vec::new();
    // smooth-shaded faces (per-vertex normals), flat edges
    for i in 0..ni - 1 {
        for j in 0..nj - 1 {
            let (a, b, c, d) = ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1));
            for (f, sg) in [(outer, 1.0f32), (&inner[..], -1.0)] {
                let p = |k: (usize, usize)| f[k.0][k.1];
                let n = |k: (usize, usize)| nrm[k.0][k.1] * sg;
                out.push(Tri { p: [p(a), p(b), p(c)], n: [n(a), n(b), n(c)] });
                out.push(Tri { p: [p(a), p(c), p(d)], n: [n(a), n(c), n(d)] });
            }
        }
    }
    let c = outer.iter().flatten().copied().sum::<Vec3>() / (ni * nj) as f32;
    let mut edge = |a: Vec3, b: Vec3, na: Vec3, nb: Vec3| {
        let (ai, bi) = (a - na * t, b - nb * t);
        let m = (a + b) * 0.5;
        out.extend(quad(a, b, bi, ai, m - c));
    };
    for i in 0..ni - 1 {
        edge(outer[i][0], outer[i + 1][0], nrm[i][0], nrm[i + 1][0]);
        edge(outer[i][nj - 1], outer[i + 1][nj - 1], nrm[i][nj - 1], nrm[i + 1][nj - 1]);
    }
    for j in 0..nj - 1 {
        edge(outer[0][j], outer[0][j + 1], nrm[0][j], nrm[0][j + 1]);
        edge(outer[ni - 1][j], outer[ni - 1][j + 1], nrm[ni - 1][j], nrm[ni - 1][j + 1]);
    }
    out
}

impl Wing {
    /// Surface point at span parameter `s`, chord fraction `x` (upper or lower skin).
    pub fn skin(&self, s: f32, x: f32, upper: bool) -> Vec3 {
        let f = self.fr(s);
        f.at(self.foil.pt(x, upper, 1.0, 1.0), 0.0)
    }
    /// Approximate outward skin normal.
    pub fn skin_n(&self, s: f32, x: f32, upper: bool) -> Vec3 {
        let e = 1e-3;
        let a = self.skin(s, (x - e).max(0.0), upper);
        let b = self.skin(s, (x + e).min(1.0), upper);
        let c = self.skin(s + e, x, upper);
        let n = (b - a).cross(c - self.skin(s, x, upper)).normalize();
        let up = self.fr(s).up * if upper { 1.0 } else { -1.0 };
        if n.dot(up) < 0.0 { -n } else { n }
    }
    /// Chord fraction of body z at span parameter s.
    pub fn frac(&self, s: f32, z: f32) -> f32 {
        let f = self.fr(s);
        (z - f.le.z) / (f.aft.z * f.chord)
    }
    /// Panel lying on the skin between span params s0..s1, chord fractions
    /// `xr(s)` = (x0, x1), lifted `lift` off the skin, `t` thick.
    pub fn panel(&self, s0: f32, s1: f32, ns: usize, xr: &dyn Fn(f32) -> (f32, f32), nx: usize, upper: bool, lift: f32, t: f32) -> Vec<Tri> {
        let mut rows = Vec::new();
        let mut nrm = Vec::new();
        for i in 0..=ns {
            let s = s0 + (s1 - s0) * i as f32 / ns as f32;
            let (a, b) = xr(s);
            let mut r = Vec::new();
            let mut n = Vec::new();
            for j in 0..=nx {
                let x = a + (b - a) * j as f32 / nx as f32;
                let nn = self.skin_n(s, x, upper);
                r.push(self.skin(s, x, upper) + nn * lift);
                n.push(nn);
            }
            rows.push(r);
            nrm.push(n);
        }
        slab(&rows, &nrm, t)
    }
}

impl Fuselage {
    /// Lower skin y at station s and lateral offset x (None outside the section).
    pub fn bottom_at(&self, s: f32, x: f32) -> Option<f32> {
        let [w, _top, bot, waist, _nt, nb] = self.param(s);
        let u = x.abs() / w.max(1e-6);
        if u >= 1.0 {
            return None;
        }
        Some(waist - (waist - bot) * (1.0 - u.powf(nb)).max(0.0).powf(1.0 / nb))
    }
}

/// Grid of points on a height field y = f(x, z) over a rectangle, as a slab
/// (outer face down when `down`), `t` thick, lifted `lift` along the normal.
pub fn hf_slab(f: &dyn Fn(f32, f32) -> f32, x0: f32, x1: f32, z0: f32, z1: f32, nx: usize, nz: usize, down: bool, lift: f32, t: f32) -> Vec<Tri> {
    let sgn = if down { -1.0 } else { 1.0 };
    let mut rows = Vec::new();
    let mut nrm = Vec::new();
    let e = 1e-3;
    for i in 0..=nz {
        let z = z0 + (z1 - z0) * i as f32 / nz as f32;
        let mut r = Vec::new();
        let mut n = Vec::new();
        for j in 0..=nx {
            let x = x0 + (x1 - x0) * j as f32 / nx as f32;
            let y = f(x, z);
            let dx = (f(x + e, z) - f(x - e, z)) / (2.0 * e);
            let dz = (f(x, z + e) - f(x, z - e)) / (2.0 * e);
            let nn = vec3(-dx, 1.0, -dz).normalize() * sgn;
            r.push(vec3(x, y, z) + nn * lift);
            n.push(nn);
        }
        rows.push(r);
        nrm.push(n);
    }
    slab(&rows, &nrm, t)
}

/// Double-sided copy of triangles (adds reversed copies with flipped normals).
pub fn two_sided(t: &[Tri]) -> Vec<Tri> {
    let mut out = t.to_vec();
    // back copy nudged 1 mm behind so it never z-fights without culling
    out.extend(t.iter().map(|t| {
        let g = (t.p[1] - t.p[0]).cross(t.p[2] - t.p[0]).normalize_or_zero() * 0.001;
        Tri { p: [t.p[0] - g, t.p[2] - g, t.p[1] - g], n: [-t.n[0], -t.n[2], -t.n[1]] }
    }));
    out
}

/// Orthonormal frame with first axis along `a`, second as close to `b` as possible.
pub fn frame3(a: Vec3, b: Vec3) -> [Vec3; 3] {
    let a = a.normalize();
    let c = a.cross(b).normalize();
    let b = c.cross(a);
    [a, b, c]
}

pub fn map_tris(t: &[Tri], f: impl Fn(Vec3) -> Vec3) -> Vec<Tri> {
    let mirror = f(Vec3::X).x < 0.0;
    t.iter()
        .map(|t| {
            let p = t.p.map(&f);
            let n = t.n.map(|n| if mirror { vec3(-n.x, n.y, n.z) } else { n });
            Tri { p, n }
        })
        .collect()
}

/// Hinge axis for the mirror image (x -> -x) of a rotation about `a`.
pub fn mirror_axis(a: Vec3) -> Vec3 {
    vec3(a.x, -a.y, -a.z)
}

/// Mirror triangles to the other side when `sx` < 0.
pub fn side_tris(t: &[Tri], sx: f32) -> Vec<Tri> {
    if sx > 0.0 { t.to_vec() } else { mirror_x(t) }
}
