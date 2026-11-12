//! Shared airliner kit: fuselage templates, turbofan nacelles with
//! reversers, gear units, cockpit / cabin window and door paint fields.

use super::geom::*;
use super::{Part, PartKind};
use glam::{vec2, vec3, Mat4, Quat, Vec3};
use std::f32::consts::{PI, TAU};
use std::rc::Rc;

pub const TYRE_C: [f32; 3] = [0.02, 0.02, 0.021];
pub const HUB_C: [f32; 3] = [0.42, 0.43, 0.44];
pub const GEAR_C: [f32; 3] = [0.50, 0.51, 0.52];
pub const CHROME: [f32; 3] = [0.60, 0.61, 0.63];
pub const WELL: [f32; 3] = [0.035, 0.037, 0.04];
pub const WIN: [f32; 3] = [0.012, 0.016, 0.022];
pub const SEAM_C: [f32; 3] = [0.30, 0.30, 0.30];
pub const WING_GREY: [f32; 3] = [0.46, 0.48, 0.50];
pub const WING_GREY_LO: [f32; 3] = [0.36, 0.38, 0.40];
pub const LIP: [f32; 3] = [0.52, 0.53, 0.55];
pub const FAN: [f32; 3] = [0.10, 0.105, 0.11];
pub const CORE: [f32; 3] = [0.20, 0.205, 0.21];
pub const BURNT_C: [f32; 3] = [0.06, 0.055, 0.05];

/// Normalised nose template: (u, w, top, bot, waist) with u = s / nose_len,
/// w / R, (top - yc) / R_top, (yc - bot) / R_bot, (waist - yc) / R.
const NOSE: [(f32, f32, f32, f32, f32); 9] = [
    (0.0, 0.0, -0.228, 0.228, -0.228),
    (0.027, 0.228, 0.025, 0.456, -0.228),
    (0.091, 0.430, 0.228, 0.623, -0.218),
    (0.18, 0.597, 0.380, 0.744, -0.187),
    (0.327, 0.770, 0.608, 0.861, -0.137),
    (0.47, 0.886, 0.785, 0.927, -0.086),
    (0.655, 0.962, 0.927, 0.972, -0.035),
    (0.91, 0.995, 0.987, 0.995, -0.005),
    (1.0, 1.0, 1.0, 1.0, 0.0),
];
/// Tail template: u = (s - (len - tail_len)) / tail_len.
const TAIL: [(f32, f32, f32, f32, f32); 8] = [
    (0.0, 1.0, 1.0, 1.0, 0.0),
    (0.149, 0.977, 0.997, 0.911, 0.025),
    (0.347, 0.886, 0.967, 0.658, 0.101),
    (0.545, 0.734, 0.896, 0.354, 0.203),
    (0.743, 0.506, 0.760, 0.025, 0.329),
    (0.901, 0.278, 0.608, -0.203, 0.405),
    (0.98, 0.152, 0.532, -0.304, 0.420),
    (1.0, 0.127, 0.512, -0.329, 0.420),
];

pub struct Body {
    pub len: f32,
    pub r: f32,
    pub r_top: f32,
    pub r_bot: f32,
    pub yc: f32,
    pub nose_len: f32,
    pub tail_len: f32,
}

impl Body {
    pub fn keys(&self) -> Vec<FusKey> {
        let mut k = Vec::new();
        let r = self.r;
        let mk = |s: f32, w: f32, t: f32, b: f32, wa: f32| -> FusKey { [s, w * r, self.yc + t * self.r_top, self.yc - b * self.r_bot, self.yc + wa * r, 2.0, 2.0] };
        for &(u, w, t, b, wa) in &NOSE {
            k.push(mk(u * self.nose_len, w.max(0.004), t, b, wa));
        }
        let s_tail = self.len - self.tail_len;
        let mid = (self.nose_len + s_tail) * 0.5;
        k.push(mk(mid, 1.0, 1.0, 1.0, 0.0));
        for &(u, w, t, b, wa) in &TAIL {
            k.push(mk(s_tail + u * self.tail_len, w, t, b, wa));
        }
        k
    }
}

// =================================================================== turbofan nacelle

pub struct Fan {
    /// Fan-face centre (left engine, +X), thrust along +Z.
    pub c: Vec3,
    pub r_fan: f32,
    pub r_hi: f32,
    pub r_max: f32,
    pub r_end: f32,
    /// Inlet length ahead of the fan face and fan-cowl length behind it.
    pub l_in: f32,
    pub l_cowl: f32,
    pub r_core0: f32,
    pub r_core1: f32,
    pub l_core: f32,
    pub l_plug: f32,
    /// Lower-half radius factor (< 1 flattens the bottom, "hamster pouch").
    pub flat: f32,
    pub blades: usize,
    /// true: four pivoting blocker doors; false: translating cascade sleeve.
    pub pivot_doors: bool,
}

pub struct FanGeom {
    pub cowl: Vec<Tri>,
    pub lip: Vec<Tri>,
    pub dark: Vec<Tri>,
    pub fan: Vec<Tri>,
    pub spinner: Vec<Tri>,
    pub core: Vec<Tri>,
    pub burnt: Vec<Tri>,
    /// Reverser parts' triangles + hinge (pivot, axis, max angle) for the left engine.
    pub reversers: Vec<(Vec<Tri>, Vec3, Vec3, f32)>,
}

impl Fan {
    fn flat_at(&self, dz: f32) -> f32 {
        // fully flattened from the lip aft; round at the fan face
        if dz >= 0.0 {
            1.0 + (self.flat - 1.0) * (dz / self.l_in).clamp(0.0, 1.0).powf(0.5)
        } else {
            self.flat
        }
    }
    fn ring(&self, dz: f32, r: f32, f: f32, n: usize) -> Vec<Vec3> {
        (0..n)
            .map(|j| {
                let a = TAU * j as f32 / n as f32;
                let (s, c) = a.sin_cos();
                let ry = if s < 0.0 { r * (1.0 + (f - 1.0) * s * s) } else { r };
                self.c + vec3(r * c, ry * s, dz)
            })
            .collect()
    }
    /// Outer cowl radius at dz (for pylons / doors).
    pub fn outer_r(&self, dz: f32) -> f32 {
        let pts = self.outer_prof();
        for w in pts.windows(2) {
            if dz <= w[0].0 && dz >= w[1].0 {
                let t = (dz - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + (w[1].1 - w[0].1) * t;
            }
        }
        self.r_end
    }
    fn outer_prof(&self) -> Vec<(f32, f32)> {
        let (li, lc) = (self.l_in, self.l_cowl);
        let (rh, rm, re) = (self.r_hi, self.r_max, self.r_end);
        let mut v = vec![(li, rh), (li - 0.015, rh + 0.035 * rm), (li - 0.06, rh + 0.065 * rm)];
        for k in 1..=6 {
            let t = k as f32 / 6.0;
            let dz = li - 0.06 - (li * 0.9 + 0.15 * lc) * t;
            let r = rh + 0.065 * rm + (rm - rh - 0.065 * rm) * (1.0 - (1.0 - t).powi(2));
            v.push((dz, r));
        }
        let z0 = v.last().unwrap().0;
        for k in 1..=6 {
            let t = k as f32 / 6.0;
            let dz = z0 + (-lc - z0) * t;
            let r = rm + (re - rm) * t.powf(1.8);
            v.push((dz, r));
        }
        v
    }

    pub fn build(&self) -> FanGeom {
        let n = 48;
        let (li, lc) = (self.l_in, self.l_cowl);
        // inner inlet: fan face -> highlight
        let inner: Vec<(f32, f32)> = vec![
            (-0.14, self.r_fan),
            (0.0, self.r_fan),
            (li * 0.35, self.r_fan + 0.01),
            (li * 0.75, self.r_fan + 0.025),
            (li - 0.05, self.r_hi - 0.035),
            (li - 0.012, self.r_hi - 0.012),
        ];
        let outer = self.outer_prof();
        let mut lip_rows = Vec::new();
        let mut cowl_rows = Vec::new();
        for &(dz, r) in &inner {
            lip_rows.push(self.ring(dz, r, self.flat_at(dz), n));
        }
        for (k, &(dz, r)) in outer.iter().enumerate() {
            let row = self.ring(dz, r, self.flat_at(dz), n);
            if k < 3 {
                lip_rows.push(row.clone());
            }
            if k >= 2 {
                cowl_rows.push(row);
            }
        }
        let mut lip = grid(&lip_rows, true, false);
        // the lip wraps from the inner duct to the outside: orient by the outer rows
        let rad = |t: &Tri| {
            let d = (t.p[0] + t.p[1] + t.p[2]) / 3.0 - self.c;
            vec2(d.x, d.y).length()
        };
        let probe = *lip.iter().max_by(|a, b| rad(a).partial_cmp(&rad(b)).unwrap()).unwrap();
        let radial = { let d = probe.p[0] - self.c; vec3(d.x, d.y, 0.0) };
        if probe.n[0].dot(radial) < 0.0 {
            for t in &mut lip {
                t.n = t.n.map(|n| -n);
            }
        }
        let cowl = grid(&cowl_rows, true, false);
        // bypass exit: annulus from the cowl trailing edge to the core cowl
        let mut dark = Vec::new();
        let te = self.ring(-lc, self.r_end, self.flat, n);
        let te_in = self.ring(-lc + 0.05, self.r_end - 0.035, self.flat, n);
        let core_front = self.ring(-lc + 0.25, self.r_core0, 1.0, n);
        let mut lipte = Vec::new();
        for j in 0..n {
            let k = (j + 1) % n;
            lipte.extend(quad(te[j], te[k], te_in[k], te_in[j], -Vec3::Z));
            dark.extend(quad(te_in[j], te_in[k], core_front[k], core_front[j], -Vec3::Z));
        }
        // core cowl + primary nozzle + plug
        let core_prof: Vec<(f32, f32)> = (0..=6).map(|k| {
            let t = k as f32 / 6.0;
            (-lc + 0.25 - (self.l_core + 0.25) * t, self.r_core0 + (self.r_core1 - self.r_core0) * t.powf(1.4))
        }).collect();
        let core = lathe(self.c, Vec3::Z, Vec3::Y, &core_prof, n, 1.0, false);
        let zc = -lc - self.l_core;
        let plug_r = self.r_core1 * 0.62;
        let mut burnt = annulus(self.c + vec3(0.0, 0.0, zc), -Vec3::Z, plug_r, self.r_core1, n);
        burnt.extend(lathe(self.c, Vec3::Z, Vec3::Y, &[(zc + 0.05, plug_r), (zc - self.l_plug * 0.3, plug_r * 0.85), (zc - self.l_plug * 0.75, plug_r * 0.4), (zc - self.l_plug, 0.0)], 24, 1.0, false));
        // fan: dark disc, spinner, blades
        let mut fan = annulus(self.c + vec3(0.0, 0.0, -0.06), Vec3::Z, 0.0, self.r_fan + 0.005, n);
        let rs = self.r_fan * 0.30;
        let spinner = lathe(self.c, Vec3::Z, Vec3::Y, &[(-0.04, rs), (rs * 0.6, rs * 0.85), (rs * 1.3, rs * 0.45), (rs * 1.65, 0.0)], 32, 1.0, false);
        for b in 0..self.blades {
            let a0 = TAU * b as f32 / self.blades as f32;
            let mut q = Vec::new();
            let k = 4;
            let rows: Vec<Vec<Vec3>> = (0..=k)
                .map(|i| {
                    let t = i as f32 / k as f32;
                    let r = rs * 0.95 + (self.r_fan * 0.985 - rs * 0.95) * t;
                    let pitch = (35.0 + 30.0 * t).to_radians();
                    let ch = self.r_fan * (0.20 + 0.10 * t);
                    let dir = vec3(-a0.sin(), a0.cos(), 0.0);
                    let radial = vec3(a0.cos(), a0.sin(), 0.0);
                    let chord = dir * pitch.cos() + Vec3::Z * pitch.sin();
                    let cen = self.c + radial * r + Vec3::Z * 0.05;
                    vec![cen - chord * ch * 0.5, cen + chord * ch * 0.5]
                })
                .collect();
            for i in 0..k {
                let nrm = (rows[i][1] - rows[i][0]).cross(rows[i + 1][0] - rows[i][0]);
                q.extend(quad(rows[i][0], rows[i][1], rows[i + 1][1], rows[i + 1][0], nrm));
            }
            fan.extend(two_sided(&q));
        }
        // reversers
        let mut reversers = Vec::new();
        if self.pivot_doors {
            for k in 0..4 {
                let am = (45.0 + 90.0 * k as f32).to_radians();
                let half = 28f32.to_radians();
                let (dz0, dz1) = (-lc * 0.42, -lc * 0.86);
                let rows: Vec<Vec<Vec3>> = (0..=4)
                    .map(|i| {
                        let dz = dz0 + (dz1 - dz0) * i as f32 / 4.0;
                        let r = self.outer_r(dz) + 0.006;
                        (0..=6)
                            .map(|j| {
                                let a = am - half + 2.0 * half * j as f32 / 6.0;
                                let (s, c) = a.sin_cos();
                                let ry = if s < 0.0 { r * (1.0 + (self.flat - 1.0) * s * s) } else { r };
                                self.c + vec3(r * c, ry * s, dz)
                            })
                            .collect()
                    })
                    .collect();
                let nrm: Vec<Vec<Vec3>> = rows.iter().map(|r| r.iter().map(|p| { let d = *p - self.c; vec3(d.x, d.y, 0.0).normalize() }).collect()).collect();
                let door = slab(&rows, &nrm, 0.035);
                let dm = (dz0 + dz1) * 0.5;
                let rm = self.outer_r(dm);
                let pivot = self.c + vec3(rm * 0.93 * am.cos(), rm * 0.93 * am.sin(), dm);
                let tang = vec3(-am.sin(), am.cos(), 0.0);
                let front = self.c + vec3(rm * am.cos(), rm * am.sin(), dz0);
                let axis = orient_axis(tang, pivot, front, vec3(am.cos(), am.sin(), 0.0));
                reversers.push((door, pivot, axis, 32f32.to_radians()));
            }
        } else {
            // translating sleeve (aft cowl) - modelled as a rotation about a far-away hinge
            let (dz0, dz1) = (-lc * 0.48, -lc);
            let mut rows = Vec::new();
            let mut nrm = Vec::new();
            for i in 0..=5 {
                let dz = dz0 + (dz1 - dz0) * i as f32 / 5.0;
                let r = self.outer_r(dz) + 0.006;
                let row = self.ring(dz, r, self.flat, n);
                nrm.push(row.iter().map(|p| { let d = *p - self.c; vec3(d.x, d.y, 0.0).normalize() }).collect::<Vec<_>>());
                rows.push(row);
            }
            // closed ring: wrap by repeating the first column
            for (r, nn) in rows.iter_mut().zip(nrm.iter_mut()) {
                r.push(r[0]);
                nn.push(nn[0]);
            }
            let sleeve = slab(&rows, &nrm, 0.03);
            let lever = 25.0;
            let travel = 0.32 * lc / 2.2;
            let pivot = self.c + vec3(0.0, lever, (dz0 + dz1) * 0.5);
            let axis = orient_axis(Vec3::X, pivot, self.c + vec3(0.0, 0.0, (dz0 + dz1) * 0.5), -Vec3::Z);
            reversers.push((sleeve, pivot, axis, travel / lever));
        }
        let mut lipall = lip;
        lipall.extend(lipte);
        let _ = (PI, Mat4::IDENTITY, Quat::IDENTITY, vec2(0.0, 0.0));
        FanGeom { cowl, lip: lipall, dark, fan, spinner, core, burnt, reversers }
    }

    /// Paint field for the reverser cavities under the doors / cascades (negative inside).
    pub fn cavity(&self) -> impl Fn(Vec3) -> f32 + 'static {
        let (c, lc, doors) = (self.c, self.l_cowl, self.pivot_doors);
        move |p: Vec3| {
            let q = vec3(p.x.abs(), p.y, p.z) - c;
            let dz = q.z;
            if doors {
                let a = q.y.atan2(q.x);
                let mut best = f32::MAX;
                for k in 0..4 {
                    let am = (45.0 + 90.0 * k as f32).to_radians();
                    let mut d = (a - am).abs();
                    if d > PI {
                        d = TAU - d;
                    }
                    best = best.min(d);
                }
                (best - 26f32.to_radians()).max((dz - (-lc * 0.44)).max(-lc * 0.84 - dz))
            } else {
                (dz - (-lc * 0.50)).max(-lc * 0.50 - 0.34 * lc / 2.2 - dz)
            }
        }
    }
}

/// Mirror a list of reverser doors to the right engine and make parts.
pub fn reverser_parts(g: &FanGeom, surf: Surf, both: bool) -> Vec<Part> {
    let mut out = Vec::new();
    for (t, pivot, axis, ang) in &g.reversers {
        out.push(plain_part(PartKind::Reverser, t, surf, *pivot, *axis, *ang));
        if both {
            out.push(plain_part(PartKind::Reverser, &mirror_x(t), surf, mx(*pivot), mirror_axis(*axis), *ang));
        }
    }
    out
}

// =================================================================== gear units

pub struct GearGroups {
    pub tyre: Vec<Tri>,
    pub hub: Vec<Tri>,
    pub leg: Vec<Tri>,
    pub chrome: Vec<Tri>,
}

/// Oleo leg from `top` down to the axle centre `axle`, with `nw` wheels of
/// radius `r`, width `w`, spaced `sp` along X (centre-to-centre).
pub fn gear_unit(top: Vec3, axle: Vec3, r: f32, w: f32, sp: f32, nw: usize, leg_r: f32) -> GearGroups {
    let (mut tyre, mut hub, mut leg, mut chrome) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for k in 0..nw {
        let off = if nw == 1 { 0.0 } else { (k as f32 - (nw as f32 - 1.0) * 0.5) * sp };
        wheel(axle + Vec3::X * off, r, w, &mut tyre, &mut hub);
    }
    let lv = (axle - top).normalize();
    let len = (axle - top).length();
    let split = top + lv * (len * 0.55);
    leg.extend(cyl(top - lv * 0.05, split + lv * 0.08, leg_r, 16));
    chrome.extend(cyl(split, axle + Vec3::Y * (leg_r * 0.8), leg_r * 0.72, 16));
    if nw > 1 {
        leg.extend(cyl(axle - Vec3::X * (sp * 0.5 - w * 0.4), axle + Vec3::X * (sp * 0.5 - w * 0.4), leg_r * 0.55, 10));
    } else {
        leg.extend(cyl(axle - Vec3::X * w * 0.6, axle + Vec3::X * w * 0.6, leg_r * 0.5, 10));
    }
    leg.extend(obox(axle + Vec3::Y * (leg_r * 0.9), [Vec3::X, Vec3::Y, Vec3::Z], vec3(leg_r * 1.2, leg_r * 0.7, leg_r * 1.2)));
    // torque links on the front of the oleo
    let t0 = split + Vec3::Z * (leg_r * 1.1);
    let t1 = axle + Vec3::Y * (leg_r * 1.6) + Vec3::Z * (leg_r * 1.1);
    let tm = (t0 + t1) * 0.5 + Vec3::Z * (leg_r * 2.2);
    for (a, b) in [(t0, tm), (tm, t1)] {
        leg.extend(obox((a + b) * 0.5, frame3(b - a, Vec3::X), vec3((b - a).length() * 0.5, leg_r * 0.25, leg_r * 0.5)));
    }
    GearGroups { tyre, hub, leg, chrome }
}

impl GearGroups {
    pub fn groups(self, sx: f32, extra: Vec<(Vec<Tri>, Surf)>) -> Vec<(Vec<Tri>, Surf)> {
        let mut g = vec![
            (side_tris(&self.tyre, sx), matte(TYRE_C)),
            (side_tris(&self.hub, sx), paint(HUB_C)),
            (side_tris(&self.leg, sx), paint(GEAR_C)),
            (side_tris(&self.chrome, sx), metal(CHROME)),
        ];
        for (t, s) in extra {
            g.push((side_tris(&t, sx), s));
        }
        g
    }
}

// =================================================================== paint fields

/// Side projection (station s along -z from `z0`, y).
pub fn side_proj(z0: f32) -> Proj {
    Rc::new(move |p: Vec3| vec2(z0 - p.z, p.y))
}

/// Rounded door outline seam (band of half-width `w` around a rounded rect).
pub fn door_seam(z0: f32, s: (f32, f32), y: (f32, f32), r: f32, w: f32) -> (SF, SF) {
    let c = vec2((s.0 + s.1) * 0.5, (y.0 + y.1) * 0.5);
    let h = vec2((s.1 - s.0) * 0.5, (y.1 - y.0) * 0.5);
    let lo = vec2(s.0 - 0.2, y.0 - 0.2);
    let hi = vec2(s.1 + 0.2, y.1 + 0.2);
    let pr = side_proj(z0);
    let p1 = pr.clone();
    let p2 = pr.clone();
    let outer = masked(sf(move |p| rrect(p1(p) - c, h, r) - w), pr.clone(), lo, hi);
    let inner = masked(sf(move |p| -(rrect(p2(p) - c, h, r) + w)), pr, lo, hi);
    (outer, inner)
}

/// Cockpit glazing in (station, polar angle from the crown) coordinates.
/// Each pane: (u0, u1, theta_lo at u0, theta_lo at u1, theta_hi at u0, theta_hi at u1)
/// with u = s / nose_len and theta in degrees (0 = crown, 90 = waist).
pub fn cockpit_panes(mut liv: Livery, z0: f32, yc: f32, nl: f32, panes: &[(f32, f32, f32, f32, f32, f32)], surf: Surf) -> Livery {
    let side = side_proj(z0);
    for &(u0, u1, a0, a1, b0, b1) in panes {
        let (s0, s1) = (u0 * nl, u1 * nl);
        let th = move |p: Vec3| p.x.abs().atan2(p.y - yc).to_degrees();
        let lin = move |a: f32, b: f32| move |s: f32| a + (b - a) * ((s - s0) / (s1 - s0)).clamp(0.0, 1.0);
        let (lo, hi) = (lin(a0, a1), lin(b0, b1));
        let bx = |f: SF| masked(f, side.clone(), vec2(s0 - 0.3, yc - 3.0), vec2(s1 + 0.3, yc + 5.0));
        let sof = move |p: Vec3| z0 - p.z;
        liv = liv.all_neg(
            vec![
                bx(sf(move |p| s0 - sof(p))),
                bx(sf(move |p| sof(p) - s1)),
                bx(sf(move |p| (lo(sof(p)) - th(p)) * 0.02)),
                bx(sf(move |p| (th(p) - hi(sof(p))) * 0.02)),
                bx(sf(move |p| yc - 0.2 - p.y)),
            ],
            surf,
        );
    }
    liv
}
