//! Geometry kit for the procedurally modelled landmarks. Every builder works
//! in a local frame: metres, +x = right of the registry heading, -z = along
//! the heading ("forward"), +y up, y = 0 at the landmark's ground reference.
//!
//! The scene shader shades both sides of every triangle (it flips the normal
//! toward the viewer), so winding and normal sign do not matter here.

use crate::lights::{kind, LightInst};
use crate::scene::{mat, MeshData};
use glam::{vec2, vec3, Vec2, Vec3};
use std::f32::consts::{PI, TAU};

/// How a surface is shaded.
#[derive(Clone, Copy, Debug)]
pub enum Surf {
    /// A plain material (uv unused): DIFFUSE, METAL, ROOF, PAINT, ...
    Plain(f32),
    /// Facade with punched windows / curtain wall (`mat::WALL` / `mat::GLASS`):
    /// uv = (seed * 4096 + metres along, height). Even seeds (no shopfronts).
    Facade(f32, u32),
    /// Floodlit: shaded like DIFFUSE by day, self-lit at night with this
    /// luminance (cd/m², times the vertex colour). `NIGHT_EMISSIVE` with
    /// uv.y = 1 — needs the scene.wgsl branch added with this module.
    Lit(f32),
    /// Floodlit from below: like `Lit`, the luminance fading to ~55 % with
    /// height (y in the local frame).
    Flood(f32),
    /// Night-only glow (`NIGHT_EMISSIVE`, uv.y = 0): black by day — windows,
    /// LED screens.
    Glow(f32),
}

impl Surf {
    fn mat(self) -> f32 {
        match self {
            Surf::Plain(m) | Surf::Facade(m, _) => m,
            Surf::Lit(_) | Surf::Flood(_) | Surf::Glow(_) => mat::NIGHT_EMISSIVE,
        }
    }
    fn uv(self, along: f32, y: f32) -> [f32; 2] {
        match self {
            Surf::Plain(_) => [0.0, 0.0],
            Surf::Facade(_, seed) => [seed as f32 * 4096.0 + along.rem_euclid(4000.0), y],
            Surf::Lit(l) => [l, 1.0],
            Surf::Flood(l) => [l * (0.55 + 0.45 * (-y.max(0.0) / 30.0).exp()), 1.0],
            Surf::Glow(l) => [l, 0.0],
        }
    }
}

/// Normals of a [`Kit::grid`] surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shade {
    /// Shared vertices, averaged normals (colour per vertex).
    Smooth,
    /// Separate quads with their own flat normal (colour per quad).
    Flat,
    /// Separate quads with the smooth normals (colour per quad: stripes and
    /// panels on curved surfaces).
    Split,
}

pub const fn rgb(r: f32, g: f32, b: f32) -> [f32; 3] {
    [r, g, b]
}

/// Builder output in the local frame.
#[derive(Default)]
pub struct Kit {
    pub m: MeshData,
    /// (footprint ring, top height)
    pub solids: Vec<(Vec<Vec2>, f32)>,
    pub lights: Vec<LightInst>,
}

impl Kit {
    fn vert(&mut self, p: Vec3, n: Vec3, col: [f32; 3], s: Surf, along: f32) -> u32 {
        self.m.mat = s.mat();
        let mut uv = s.uv(along, p.y);
        if let Surf::Flood(_) = s {
            // Floodlights aim at the walls from a few directions: faces
            // facing different ways get different levels (reads the form at
            // night); roofs catch little.
            let h = vec2(n.x, n.z);
            let k = if n.y.abs() > 0.7 || h.length() < 1e-3 { 0.3 } else { 0.5 + 0.5 * (h.normalize().dot(vec2(0.8, 0.6))).abs() };
            uv[0] *= k;
        }
        self.m.v_uv(p, n, col, uv)
    }

    /// Surface through `rows` of points (`rows[j][i]`: i runs around / across,
    /// j up). `closed`: i wraps around. Colour per (i, j) vertex (Smooth) or
    /// quad (Flat / Split). uv "along" = arc length along each row.
    pub fn grid(&mut self, rows: &[Vec<Vec3>], closed: bool, shade: Shade, s: Surf, col: &dyn Fn(usize, usize) -> [f32; 3]) {
        let nj = rows.len();
        if nj < 2 {
            return;
        }
        let ni = rows[0].len();
        debug_assert!(rows.iter().all(|r| r.len() == ni));
        let nq = if closed { ni } else { ni - 1 };
        let at = |i: usize, j: usize| rows[j][i % ni];
        // Area-weighted vertex normals from the quads around each vertex.
        let mut nrm = vec![Vec3::ZERO; ni * nj];
        for j in 0..nj - 1 {
            for i in 0..nq {
                let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                let n = (c - a).cross(d - b);
                for (ii, jj) in [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)] {
                    nrm[jj * ni + ii % ni] += n;
                }
            }
        }
        for n in &mut nrm {
            *n = n.normalize_or(Vec3::Y);
        }
        // Arc length along each row.
        let along: Vec<Vec<f32>> = rows
            .iter()
            .map(|r| {
                let mut acc = 0.0;
                let mut out = vec![0.0];
                for i in 0..nq {
                    acc += r[i].distance(r[(i + 1) % ni]);
                    out.push(acc);
                }
                out
            })
            .collect();
        match shade {
            Shade::Smooth => {
                let cols = nq + 1;
                let base = self.m.verts.len() as u32;
                for j in 0..nj {
                    for i in 0..cols {
                        self.vert(at(i, j), nrm[j * ni + i % ni], col(i % ni, j), s, along[j][i]);
                    }
                }
                for j in 0..nj as u32 - 1 {
                    for i in 0..nq as u32 {
                        let a = base + j * cols as u32 + i;
                        let d = a + cols as u32;
                        self.m.idx.extend([a, a + 1, d + 1, a, d + 1, d]);
                    }
                }
            }
            Shade::Flat | Shade::Split => {
                for j in 0..nj - 1 {
                    for i in 0..nq {
                        let c = col(i, j);
                        let p = [at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)];
                        let fnrm = (p[2] - p[0]).cross(p[3] - p[1]).normalize_or(Vec3::Y);
                        let ix = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
                        let base = self.m.verts.len() as u32;
                        for (k, &(ii, jj)) in ix.iter().enumerate() {
                            let n = if shade == Shade::Flat { fnrm } else { nrm[jj * ni + ii % ni] };
                            self.vert(p[k], n, c, s, along[jj][ii]);
                        }
                        self.m.idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
                    }
                }
            }
        }
    }

    /// Vertical walls of a closed ring from y0 to y1 (flat per edge).
    pub fn walls(&mut self, ring: &[Vec2], y0: f32, y1: f32, s: Surf, col: [f32; 3]) {
        let rows = vec![ring.iter().map(|p| vec3(p.x, y0, p.y)).collect(), ring.iter().map(|p| vec3(p.x, y1, p.y)).collect()];
        self.grid(&rows, true, Shade::Flat, s, &|_, _| col);
    }

    /// Horizontal polygon at height y.
    pub fn cap(&mut self, ring: &[Vec2], y: f32, s: Surf, col: [f32; 3]) {
        let base = self.m.verts.len();
        self.m.mat = s.mat();
        self.m.flat_poly(ring, &[], y, Vec3::Y, col);
        for v in &mut self.m.verts[base..] {
            v.uv = s.uv(0.0, y);
        }
    }

    /// Prism: walls + roof.
    pub fn prism(&mut self, ring: &[Vec2], y0: f32, y1: f32, s: Surf, col: [f32; 3], roof: Option<(Surf, [f32; 3])>) {
        self.walls(ring, y0, y1, s, col);
        let (rs, rc) = roof.unwrap_or((s, col));
        self.cap(ring, y1, rs, rc);
    }

    /// Frustum between two rings with the same vertex count (flat faces).
    pub fn loft(&mut self, r0: &[Vec2], y0: f32, r1: &[Vec2], y1: f32, s: Surf, col: [f32; 3]) {
        let rows = vec![r0.iter().map(|p| vec3(p.x, y0, p.y)).collect(), r1.iter().map(|p| vec3(p.x, y1, p.y)).collect()];
        self.grid(&rows, true, Shade::Flat, s, &|_, _| col);
    }

    /// Pyramid / spire from a ring at y0 to an apex (flat faces).
    pub fn pyramid(&mut self, ring: &[Vec2], y0: f32, apex: Vec3, s: Surf, col: [f32; 3]) {
        let n = ring.len();
        let mut u = 0.0;
        for i in 0..n {
            let (a, b) = (vec3(ring[i].x, y0, ring[i].y), vec3(ring[(i + 1) % n].x, y0, ring[(i + 1) % n].y));
            let nr = (b - a).cross(apex - a).normalize_or(Vec3::Y);
            let len = a.distance(b);
            let i0 = self.vert(a, nr, col, s, u);
            self.vert(b, nr, col, s, u + len);
            self.vert(apex, nr, col, s, u + len * 0.5);
            self.m.idx.extend([i0, i0 + 1, i0 + 2]);
            u += len;
        }
    }

    /// Surface of revolution around the vertical axis through `c`:
    /// profile (radius, height) bottom to top. Colour per (segment, profile point).
    pub fn lathe(&mut self, c: Vec3, profile: &[(f32, f32)], segs: usize, shade: Shade, s: Surf, col: &dyn Fn(usize, usize) -> [f32; 3]) {
        self.lathe_arc(c, profile, segs, 0.0, TAU, shade, s, col);
    }

    /// Part of a surface of revolution: angles a0..a1 (0 = +x, toward +z).
    #[allow(clippy::too_many_arguments)]
    pub fn lathe_arc(&mut self, c: Vec3, profile: &[(f32, f32)], segs: usize, a0: f32, a1: f32, shade: Shade, s: Surf, col: &dyn Fn(usize, usize) -> [f32; 3]) {
        let full = (a1 - a0 - TAU).abs() < 1e-4;
        let n = if full { segs } else { segs + 1 };
        let rows: Vec<Vec<Vec3>> = profile
            .iter()
            .map(|&(r, y)| {
                (0..n)
                    .map(|i| {
                        let a = a0 + (a1 - a0) * i as f32 / segs as f32;
                        c + vec3(r * a.cos(), y, r * a.sin())
                    })
                    .collect()
            })
            .collect();
        self.grid(&rows, full, shade, s, col);
    }

    /// Tube along a polyline (parallel-transported frames), radius per point.
    pub fn path(&mut self, pts: &[Vec3], radius: &dyn Fn(usize) -> f32, sides: usize, closed_loop: bool, s: Surf, col: [f32; 3]) {
        let n = pts.len();
        if n < 2 {
            return;
        }
        let tangent = |i: usize| -> Vec3 {
            if closed_loop {
                (pts[(i + 1) % n] - pts[(i + n - 1) % n]).normalize_or(Vec3::X)
            } else {
                (pts[(i + 1).min(n - 1)] - pts[i.saturating_sub(1)]).normalize_or(Vec3::X)
            }
        };
        let t0 = tangent(0);
        let mut u = if t0.y.abs() < 0.9 { t0.cross(Vec3::Y).normalize() } else { t0.cross(Vec3::X).normalize() };
        let mut rows = Vec::with_capacity(n + 1);
        let count = if closed_loop { n + 1 } else { n };
        for k in 0..count {
            let i = k % n;
            let t = tangent(i);
            u = (u - t * u.dot(t)).normalize_or(u);
            let v = t.cross(u);
            let r = radius(i);
            rows.push((0..sides).map(|q| {
                let a = TAU * q as f32 / sides as f32;
                pts[i] + (u * a.cos() + v * a.sin()) * r
            }).collect::<Vec<_>>());
        }
        self.grid(&rows, true, Shade::Smooth, s, &|_, _| col);
    }

    /// Straight tube (cylinder / cone) from a to b.
    #[allow(clippy::too_many_arguments)]
    pub fn tube(&mut self, a: Vec3, b: Vec3, r0: f32, r1: f32, sides: usize, s: Surf, col: [f32; 3]) {
        self.path(&[a, b], &|i| if i == 0 { r0 } else { r1 }, sides, false, s, col);
    }

    /// Oriented box: centre, half extents, rotation about +y (radians,
    /// counter-clockwise seen from above in x/-z). No bottom face.
    pub fn boxr(&mut self, c: Vec3, h: Vec3, yaw: f32, s: Surf, col: [f32; 3]) {
        let ring = rect_rot(vec2(c.x, c.z), h.x, h.z, yaw);
        self.walls(&ring, c.y - h.y, c.y + h.y, s, col);
        self.cap(&ring, c.y + h.y, s, col);
    }

    /// Box between two points (a beam / girder of square-ish section):
    /// width across (horizontal), height (vertical).
    pub fn beam(&mut self, a: Vec3, b: Vec3, w: f32, h: f32, s: Surf, col: [f32; 3]) {
        let d = b - a;
        let side = vec3(-d.z, 0.0, d.x).normalize_or(Vec3::X) * (w * 0.5);
        let up = d.cross(side).normalize_or(Vec3::Y) * (h * 0.5);
        let up = if up.y < 0.0 { -up } else { up };
        let ring = |p: Vec3| vec![p - side - up, p + side - up, p + side + up, p - side + up];
        let rows = vec![ring(a), ring(b)];
        self.grid(&rows, true, Shade::Flat, s, &|_, _| col);
    }

    /// Single quad (a, b, c, d in order around).
    pub fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, s: Surf, col: [f32; 3]) {
        let n = (c - a).cross(d - b).normalize_or(Vec3::Y);
        let w = a.distance(b);
        let i = self.vert(a, n, col, s, 0.0);
        self.vert(b, n, col, s, w);
        self.vert(c, n, col, s, w);
        self.vert(d, n, col, s, 0.0);
        self.m.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    }

    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, s: Surf, col: [f32; 3]) {
        let n = (b - a).cross(c - a).normalize_or(Vec3::Y);
        let i = self.vert(a, n, col, s, 0.0);
        self.vert(b, n, col, s, a.distance(b));
        self.vert(c, n, col, s, a.distance(b) * 0.5);
        self.m.idx.extend([i, i + 1, i + 2]);
    }

    /// A vertical rectangle on a wall: centre `c`, outward normal `n`
    /// (horizontal), width w, height h — windows, dials, doors.
    pub fn panel(&mut self, c: Vec3, n: Vec3, w: f32, h: f32, s: Surf, col: [f32; 3]) {
        let t = vec3(-n.z, 0.0, n.x).normalize_or(Vec3::X) * (w * 0.5);
        let u = Vec3::Y * (h * 0.5);
        self.quad(c - t - u, c + t - u, c + t + u, c - t + u, s, col);
    }

    /// Flat disc facing `n`.
    pub fn disc(&mut self, c: Vec3, n: Vec3, r: f32, segs: usize, s: Surf, col: [f32; 3]) {
        let n = n.normalize();
        let u = if n.y.abs() < 0.9 { n.cross(Vec3::Y).normalize() } else { n.cross(Vec3::X).normalize() };
        let v = n.cross(u);
        let i0 = self.vert(c, n, col, s, 0.0);
        for k in 0..segs {
            let a = TAU * k as f32 / segs as f32;
            self.vert(c + (u * a.cos() + v * a.sin()) * r, n, col, s, 0.0);
        }
        for k in 0..segs as u32 {
            self.m.idx.extend([i0, i0 + 1 + k, i0 + 1 + (k + 1) % segs as u32]);
        }
    }

    /// Ellipsoid (smooth).
    pub fn ellipsoid(&mut self, c: Vec3, r: Vec3, nu: usize, nv: usize, s: Surf, col: [f32; 3]) {
        let profile: Vec<(f32, f32)> = (0..=nv)
            .map(|j| {
                let t = -PI / 2.0 + PI * j as f32 / nv as f32;
                (t.cos().max(0.0), t.sin())
            })
            .collect();
        let rows: Vec<Vec<Vec3>> = profile
            .iter()
            .map(|&(cr, sy)| (0..nu).map(|i| {
                let a = TAU * i as f32 / nu as f32;
                c + vec3(cr * a.cos() * r.x, sy * r.y, cr * a.sin() * r.z)
            }).collect())
            .collect();
        self.grid(&rows, true, Shade::Smooth, s, &|_, _| col);
    }

    /// Gabled (or hipped) roof over an oriented rectangle: centre, half
    /// extents (hx along the ridge, hz across), yaw, eaves height, rise.
    #[allow(clippy::too_many_arguments)]
    pub fn roof(&mut self, c: Vec2, hx: f32, hz: f32, yaw: f32, y0: f32, rise: f32, hip: f32, s: Surf, col: [f32; 3], gable: Option<(Surf, [f32; 3])>) {
        let (sn, cs) = yaw.sin_cos();
        let ax = vec2(cs, -sn);
        let az = vec2(sn, cs);
        let p = |x: f32, z: f32, y: f32| {
            let q = c + ax * x + az * z;
            vec3(q.x, y, q.y)
        };
        let r = (hx - hip).max(0.0);
        let top = y0 + rise;
        self.quad(p(-hx, -hz, y0), p(hx, -hz, y0), p(r, 0.0, top), p(-r, 0.0, top), s, col);
        self.quad(p(-hx, hz, y0), p(hx, hz, y0), p(r, 0.0, top), p(-r, 0.0, top), s, col);
        if hip > 0.0 {
            self.tri(p(hx, -hz, y0), p(hx, hz, y0), p(r, 0.0, top), s, col);
            self.tri(p(-hx, -hz, y0), p(-hx, hz, y0), p(-r, 0.0, top), s, col);
        } else if let Some((gs, gc)) = gable {
            self.tri(p(hx, -hz, y0), p(hx, hz, y0), p(hx, 0.0, top), gs, gc);
            self.tri(p(-hx, -hz, y0), p(-hx, hz, y0), p(-hx, 0.0, top), gs, gc);
        }
    }

    /// Small square spike (Gothic pinnacle / finial).
    pub fn pinnacle(&mut self, c: Vec2, y0: f32, half: f32, h: f32, s: Surf, col: [f32; 3]) {
        let ring = rect(c, half, half);
        let shaft = h * 0.35;
        self.walls(&ring, y0, y0 + shaft, s, col);
        self.pyramid(&ring, y0 + shaft, vec3(c.x, y0 + h, c.y), s, col);
    }

    // --- non-geometry

    pub fn solid(&mut self, ring: Vec<Vec2>, top: f32) {
        self.solids.push((ring, top));
    }
    /// Steady red aircraft-warning light.
    pub fn obstacle(&mut self, p: Vec3) {
        self.lights.push(LightInst::new(p, 2000.0, crate::lights::colour::OBST_RED, kind::OBSTACLE));
    }
    /// Flashing white high-intensity obstacle light.
    pub fn strobe(&mut self, p: Vec3, phase: f32) {
        self.lights.push(LightInst::new(p, 20000.0, [1.0, 1.0, 1.0], kind::STROBE).with_ab(phase, 0.0));
    }
}

// ---------------------------------------------------------------- rings

/// Axis-aligned rectangle ring.
pub fn rect(c: Vec2, hx: f32, hz: f32) -> Vec<Vec2> {
    vec![c + vec2(-hx, -hz), c + vec2(hx, -hz), c + vec2(hx, hz), c + vec2(-hx, hz)]
}

/// Rectangle rotated by `yaw` (radians, x toward -z).
pub fn rect_rot(c: Vec2, hx: f32, hz: f32, yaw: f32) -> Vec<Vec2> {
    let (s, co) = yaw.sin_cos();
    let ax = vec2(co, -s);
    let az = vec2(s, co);
    [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|&(x, z)| c + ax * (x * hx) + az * (z * hz)).collect()
}

/// Rectangle between x0..x1, z0..z1.
pub fn rect_xz(x0: f32, x1: f32, z0: f32, z1: f32) -> Vec<Vec2> {
    vec![vec2(x0, z0), vec2(x1, z0), vec2(x1, z1), vec2(x0, z1)]
}

/// Regular polygon.
pub fn ngon(c: Vec2, r: f32, n: usize, phase: f32) -> Vec<Vec2> {
    (0..n).map(|i| {
        let a = phase + TAU * i as f32 / n as f32;
        c + vec2(a.cos(), a.sin()) * r
    }).collect()
}

/// Superellipse |x/a|^n + |z/b|^n = 1.
pub fn superellipse(c: Vec2, a: f32, b: f32, n: f32, segs: usize) -> Vec<Vec2> {
    (0..segs).map(|i| {
        let t = TAU * i as f32 / segs as f32;
        let (s, co) = t.sin_cos();
        let f = |v: f32| v.signum() * v.abs().powf(2.0 / n);
        c + vec2(a * f(co), b * f(s))
    }).collect()
}
