//! Lancer F2: single-engine afterburning multirole fighter (blended body,
//! chin intake, bubble canopy, all-moving stabilators, retractable tricycle
//! gear with doors). Two-tone grey air-superiority scheme, invented low-vis
//! markings.

use super::geom::*;
use super::{AircraftModel, PartKind};
use glam::{vec2, vec3, Quat, Vec2, Vec3};
use std::f32::consts::TAU;
use std::rc::Rc;

/// z of the radome tip (fuselage station s = 0); z = L_Z0 - s.
const L_Z0: f32 = 7.35;
const NOZZLE_Z: f32 = -7.20;

const LIGHT: [f32; 3] = [0.300, 0.320, 0.340];
const MED: [f32; 3] = [0.165, 0.178, 0.195];
const DARKG: [f32; 3] = [0.092, 0.100, 0.112];
const RADOME: [f32; 3] = [0.120, 0.128, 0.138];
const MARK: [f32; 3] = [0.060, 0.064, 0.072];
const GOLD_GLASS: [f32; 3] = [0.040, 0.034, 0.014];
const NOZ: [f32; 3] = [0.10, 0.085, 0.07];
const BURNT: [f32; 3] = [0.05, 0.045, 0.04];

const FUS: [FusKey; 15] = [
    // s     w      top    bot     waist  nt   nb
    [0.00, 0.020, 0.100, 0.060, 0.080, 2.0, 2.0],
    [0.30, 0.170, 0.240, -0.080, 0.080, 2.0, 2.0],
    [0.80, 0.300, 0.360, -0.220, 0.070, 2.0, 2.0],
    [1.50, 0.420, 0.460, -0.360, 0.050, 2.0, 2.0],
    [2.20, 0.500, 0.520, -0.460, 0.030, 2.0, 2.1],
    [2.80, 0.560, 0.560, -0.520, 0.000, 2.0, 2.3],
    [3.40, 0.630, 0.580, -0.560, -0.010, 2.1, 2.5],
    [4.40, 0.700, 0.600, -0.620, -0.020, 2.2, 2.8],
    [6.00, 0.880, 0.660, -0.760, -0.020, 2.3, 3.0],
    [8.00, 0.980, 0.680, -0.800, -0.020, 2.3, 3.0],
    [10.0, 0.900, 0.660, -0.720, -0.020, 2.2, 2.8],
    [11.5, 0.740, 0.620, -0.620, -0.020, 2.1, 2.4],
    [12.8, 0.620, 0.580, -0.570, 0.000, 2.0, 2.1],
    [13.6, 0.565, 0.555, -0.555, 0.000, 2.0, 2.0],
    [13.80, 0.560, 0.550, -0.550, 0.000, 2.0, 2.0],
];

const DUCT: [FusKey; 7] = [
    [3.45, 0.500, -0.250, -1.000, -0.660, 2.2, 2.6],
    [4.20, 0.540, -0.220, -0.980, -0.640, 2.2, 2.6],
    [5.50, 0.600, -0.200, -0.930, -0.620, 2.2, 2.8],
    [7.50, 0.680, -0.200, -0.870, -0.600, 2.2, 3.0],
    [9.50, 0.700, -0.200, -0.800, -0.580, 2.2, 3.0],
    [11.0, 0.600, -0.200, -0.660, -0.500, 2.2, 2.6],
    [11.6, 0.500, -0.200, -0.560, -0.450, 2.2, 2.4],
];

fn ls(p: Vec3) -> f32 {
    L_Z0 - p.z
}

fn camo(p: Vec3) -> f32 {
    let (x, z) = (p.x.abs(), p.z);
    (1.30 * x + 0.62 * z + 1.0).sin() + 0.8 * (-0.85 * x + 1.05 * z + 2.2 + 0.9 * p.y).sin() + 0.45 * (2.2 * x - 0.55 * z).sin() - 0.25
}

/// Chevron (pointing +u) as two straight arms, each an intersection of
/// half-planes in the projection `proj` (so the paint splits stay exact).
fn chevron(liv: Livery, proj: Proj, c: Vec2, size: f32, masks: Vec<SF>, surf: Surf) -> Livery {
    let w = size * 0.13 * std::f32::consts::SQRT_2;
    let h = size * 0.75;
    let mut out = liv;
    for sgn in [1.0f32, -1.0] {
        let q = |pr: &Proj| {
            let pr = pr.clone();
            move |p: Vec3| pr(p) - c
        };
        let (q1, q2, q3, q4) = (q(&proj), q(&proj), q(&proj), q(&proj));
        let mut f = vec![
            sf(move |p| -sgn * q1(p).y),
            sf(move |p| sgn * q2(p).y - h),
            sf(move |p| q3(p).x + sgn * q3(p).y - w),
            sf(move |p| -(q4(p).x + sgn * q4(p).y) - w),
        ];
        f.extend(masks.clone());
        out = out.all_neg(f, surf);
    }
    out
}

pub fn lancer() -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts = Vec::new();

    let fus = Rc::new(Fuselage::new(&FUS));
    let duct = Rc::new(Fuselage::new(&DUCT));
    let side: Proj = Rc::new(|p: Vec3| vec2(ls(p), p.y));

    // ---------------------------------------------------------- landing gear geometry
    // nose: folds aft into the bay ahead of the intake
    let (nr, nw) = (0.25f32, 0.17f32);
    let npiv = vec3(0.0, -0.46, 4.98);
    let nhub = vec3(0.0, -1.90 + nr, 4.80);
    let nt = vec3(0.0, 20f32.to_radians().sin(), -20f32.to_radians().cos());
    let (naxis, nang) = fold(nhub - npiv, nt, Vec3::X, Vec3::X);
    // mains: fold forward / inward, wheels lie flat under the duct
    let (mr, mw) = (0.35f32, 0.22f32);
    let mpiv = vec3(0.80, -0.56, -0.32);
    let mhub = vec3(1.30, -1.90 + mr, -0.60);
    let mleg = mhub - vec3(0.14, 0.0, 0.0);
    let reach = (mhub - mpiv).length();
    let mtarget = mpiv + vec3(-0.36, -0.075, 0.93).normalize() * reach;
    let (maxis, mang) = fold(mhub - mpiv, mtarget - mpiv, Vec3::X, Vec3::Y);
    let mstow = |p: Vec3| mpiv + Quat::from_axis_angle(maxis, mang) * (p - mpiv);
    let mhs = mstow(mhub);

    // lower envelope (fuselage + duct) for doors / wells
    let (f1, d1) = (fus.clone(), duct.clone());
    let belly = move |x: f32, z: f32| -> f32 {
        let s = L_Z0 - z;
        let a = f1.bottom_at(s, x).unwrap_or(0.0);
        let b = if s > 3.45 && s < 11.6 { d1.bottom_at(s, x).unwrap_or(0.0) } else { 0.0 };
        a.min(b)
    };
    // door outlines (x0, x1, z0, z1)
    let nbay = (-0.20f32, 0.20f32, 3.98f32, 5.10f32);
    let mbay = (0.06f32, 0.86f32, -0.52f32, 1.18f32);

    // ---------------------------------------------------------- fuselage + duct
    let rows = fus.rows(L_Z0, 0.12, 48, (0.0, 3.0, 1.6));
    let mut t = grid(&rows, true, false);
    t.extend(fan(&rows[0], Vec3::Z));
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    let demarc = |p: Vec3| -0.08 + 0.05 * (ls(p) * 1.3).sin() - p.y;
    let bay = |b: (f32, f32, f32, f32), mirror: bool| {
        move |p: Vec3| {
            let x = if mirror { p.x.abs() } else { p.x };
            let q = vec2(x - (b.0 + b.1) * 0.5, p.z - (b.2 + b.3) * 0.5);
            rrect(q, vec2((b.1 - b.0) * 0.5, (b.3 - b.2) * 0.5), 0.05)
        }
    };
    let low = |p: Vec3| p.y + 0.3;
    // invented low-vis chevron on the forward fuselage sides
    let fl = chevron(Livery::new(paint(LIGHT)), side.clone(), vec2(5.35, 0.30), 0.30, vec![sf(|p| 0.4 - p.x.abs())], paint(MARK));
    let fl = fl
        .all_neg(vec![sf(bay(nbay, false)), sf(low)], matte(DARK))
        .all_neg(vec![sf(bay(mbay, true)), sf(low)], matte(DARK))
        // radome
        .all_neg(vec![sf(|p| ls(p) - 1.75 - 0.05 * p.y)], paint(RADOME))
        .all_neg(vec![sf(demarc), sf(camo)], paint(DARKG))
        .all_neg(vec![sf(demarc)], paint(MED));
    emit(&mut body, &t, &fl);

    let drows = duct.rows(L_Z0, 0.10, 40, (3.4, 4.0, 2.0));
    let mut dt = grid(&drows, true, false);
    dt.extend(fan(drows.last().unwrap(), -Vec3::Z));
    let dl = Livery::new(paint(LIGHT))
        .all_neg(vec![sf(bay(mbay, true)), sf(low)], matte(DARK))
        .all_neg(vec![sf(|p| ls(p) - 3.52)], paint(MED));
    emit(&mut body, &dt, &dl);
    // intake mouth (dark, recessed look) + splitter plate
    let mut mouth = fan(&drows[0], Vec3::Z);
    transform_tris(&mut mouth, glam::Mat4::from_translation(vec3(0.0, 0.0, -0.02)));
    emit_plain(&mut body, &mouth, matte(DARK));
    let lip = tube(vec3(0.0, -0.20, L_Z0 - 3.47), vec3(0.0, -0.20, L_Z0 - 4.6), Vec3::Y, &[(0.0, 0.010, 0.40), (1.0, 0.012, 0.40)], 2.0, 8);
    emit_plain(&mut body, &lip, paint(MED));

    // canopy (one-piece bubble), dorsal spine
    let can_keys: [FusKey; 11] = [
        [2.25, 0.200, 0.540, 0.20, 0.42, 2.2, 2.0],
        [2.50, 0.350, 0.790, 0.20, 0.42, 2.4, 2.0],
        [2.90, 0.430, 1.060, 0.20, 0.42, 2.3, 2.0],
        [3.40, 0.465, 1.200, 0.20, 0.42, 2.2, 2.0],
        [3.95, 0.470, 1.230, 0.20, 0.42, 2.2, 2.0],
        [4.50, 0.445, 1.170, 0.20, 0.42, 2.2, 2.0],
        [5.10, 0.380, 1.020, 0.20, 0.42, 2.2, 2.0],
        [5.80, 0.300, 0.860, 0.20, 0.42, 2.2, 2.0],
        [7.20, 0.240, 0.760, 0.20, 0.42, 2.2, 2.0],
        [10.8, 0.200, 0.720, 0.20, 0.42, 2.2, 2.0],
        [12.2, 0.050, 0.640, 0.20, 0.42, 2.0, 2.0],
    ];
    let can = Fuselage::new(&can_keys);
    let crow = can.rows(L_Z0, 0.07, 40, (2.2, 5.6, 1.5));
    let mut ct = grid(&crow, true, false);
    ct.extend(fan(&crow[0], Vec3::Z));
    ct.extend(fan(crow.last().unwrap(), -Vec3::Z));
    let sill = |s: f32| 0.585 + 0.03 * (s - 2.5).max(0.0);
    let cliv = Livery::new(paint(MED))
        .all_neg(vec![sf(|p| (ls(p) - 5.32).abs() - 0.035)], paint(DARKG))
        .all_neg(vec![sf(move |p| p.y - sill(ls(p)) - 0.03)], paint(DARKG))
        .all_neg(vec![sf(|p| ls(p) - 5.32)], glass(GOLD_GLASS))
        .all_neg(vec![sf(camo)], paint(DARKG));
    emit(&mut body, &ct, &cliv);

    // ---------------------------------------------------------- wing
    let wy = -0.05f32;
    let wk: [(f32, f32, f32); 5] = [(0.0, 5.0, -3.45), (0.62, 4.60, -3.44), (0.98, 2.80, -3.43), (1.32, 0.98, -3.42), (4.60, -1.77, -2.95)];
    let wkeys: Vec<Key> = wk.iter().map(|&(x, le, te)| key(x, vec3(x, wy, le), le - te, 0.0, 0.0)).collect();
    let wk2 = wkeys.clone();
    let hfrac = move |s: f32, hz: f32| {
        let f = key_frame(&wk2, s);
        (f.le.z - hz) / f.chord
    };
    let (fhz, ahz) = (-2.66f32, -2.40f32);
    let mut wing = keyed_wing(
        wkeys,
        Foil::sym(0.045),
        vec![
            seg(0.0, 1.32, 6),
            cut(1.32, 2.70, 6, hfrac(1.20, fhz), hfrac(2.70, fhz - 0.06)),
            seg(2.70, 2.76, 1),
            cut(2.76, 4.25, 6, hfrac(2.76, ahz - 0.07), hfrac(4.25, ahz)),
            seg(4.25, 4.60, 2),
        ],
        Cap::Open,
        Cap::Flat,
    );
    wing.k1 = 16;
    wing.k2 = 6;
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let wliv = Livery::new(paint(LIGHT))
        .all_neg(vec![sf(move |p| wy + 0.002 - p.y), sf(camo)], paint(DARKG))
        .all_neg(vec![sf(move |p| wy + 0.002 - p.y)], paint(MED));
    emit(&mut body, &wt, &wliv);
    for (c, kinds, maxa, want) in [
        (&ctrls[0], [PartKind::FlapLeft, PartKind::FlapRight], 20f32.to_radians(), [-Vec3::Y, -Vec3::Y]),
        (&ctrls[1], [PartKind::AileronLeft, PartKind::AileronRight], 21f32.to_radians(), [-Vec3::Y, Vec3::Y]),
    ] {
        let lp = (c.a + c.b) * 0.5;
        let la = (c.b - c.a).normalize();
        parts.push(hinged(kinds[0], &c.tris, &wliv, lp, la, want[0], maxa));
        parts.push(hinged(kinds[1], &mirror_x(&c.tris), &wliv, mx(lp), mx(la), want[1], maxa));
    }
    // wingtip launcher rails
    let mut rails = Vec::new();
    for sx in [1.0f32, -1.0] {
        let a = vec3(4.67 * sx, wy, -0.55);
        let b = vec3(4.67 * sx, wy, -3.25);
        let prof = [(0.0, 0.0, 0.0), (0.02, 0.035, 0.035), (0.08, 0.06, 0.06), (0.95, 0.065, 0.065), (1.0, 0.04, 0.04)];
        rails.extend(tube(a, b, Vec3::Y, &prof, 2.0, 14));
        rails.extend(obox(vec3(4.62 * sx, wy, -1.9), [Vec3::X, Vec3::Y, Vec3::Z], vec3(0.05, 0.025, 1.2)));
    }
    emit_plain(&mut body, &rails, paint(MED));

    // ---------------------------------------------------------- tail booms + speedbrake petals
    let by = -0.10f32;
    let (bz0, bz1, bz2) = (-2.40f32, -5.95f32, -6.62f32);
    let bprof = |t: f32| {
        let f = (t / 0.12).min(1.0);
        (1.0 - (1.0 - f).powi(2)).sqrt()
    };
    let mut boom = Vec::new();
    for sx in [1.0f32, -1.0] {
        let a = vec3(0.98 * sx, by, bz0);
        let b = vec3(0.98 * sx, by, bz1);
        let prof: Vec<(f32, f32, f32)> = (0..=8).map(|i| {
            let t = i as f32 / 8.0;
            (t, 0.20 * bprof(t).max(0.05), 0.12 * bprof(t).max(0.05))
        }).collect();
        boom.extend(tube(a, b, Vec3::X, &prof, 3.0, 20));
    }
    emit(&mut body, &boom, &Livery::new(paint(LIGHT)).all_neg(vec![sf(move |p| by - p.y), sf(camo)], paint(DARKG)).all_neg(vec![sf(move |p| by - p.y)], paint(MED)));
    for upper in [true, false] {
        let mut pet = Vec::new();
        for sx in [1.0f32, -1.0] {
            let n = 10;
            let rows: Vec<Vec<Vec3>> = (0..=6)
                .map(|i| {
                    let t = i as f32 / 6.0;
                    let z = bz1 - 0.003 - (bz2 - bz1 + 0.003).abs() * t;
                    let k = 1.0 - 0.30 * t * t;
                    (0..=n)
                        .map(|j| {
                            let a = std::f32::consts::PI * j as f32 / n as f32;
                            let (c, s) = (a.cos(), a.sin());
                            let e = 2.0 / 3.0;
                            vec3(0.98 * sx + 0.20 * k * spow(c, e), by + if upper { 1.0 } else { -1.0 } * 0.12 * k * spow(s, e), z)
                        })
                        .collect()
                })
                .collect();
            let out = if upper { Vec3::Y } else { -Vec3::Y };
            for i in 0..rows.len() - 1 {
                for j in 0..n {
                    let mid = (rows[i][j] + rows[i + 1][j + 1]) * 0.5;
                    let c = vec3(0.98 * sx, by, mid.z);
                    pet.extend(quad(rows[i][j], rows[i + 1][j], rows[i + 1][j + 1], rows[i][j + 1], mid - c));
                }
                pet.extend(quad(rows[i][0], rows[i + 1][0], rows[i + 1][n], rows[i][n], -out));
            }
            pet.extend(fan(&rows[0], Vec3::Z));
            pet.extend(fan(rows.last().unwrap(), -Vec3::Z));
        }
        let hinge = vec3(0.0, by + if upper { 0.11 } else { -0.11 }, bz1);
        let probe = vec3(0.98, by, bz2);
        let want = if upper { Vec3::Y } else { -Vec3::Y };
        let axis = orient_axis(Vec3::X, hinge, probe, want);
        let surf = if upper { paint(MED) } else { paint(LIGHT) };
        parts.push(multi_part(PartKind::Spoilers, &[(pet, surf)], hinge, axis, 55f32.to_radians()));
    }

    // ---------------------------------------------------------- stabilators (one all-moving part)
    let anh = -10f32;
    let ca = anh.to_radians();
    let sk: Vec<Key> = [(0.0f32, 0.0f32, -3.85f32, -6.50f32), (0.80, 0.80, -3.95, -6.50), (2.85, 2.85, -5.68, -6.45)]
        .iter()
        .map(|&(s, x, le, te)| {
            let r = (s - 0.8).max(0.0);
            key(s, vec3(x.min(0.8) + r * ca.cos(), -0.10 + r * ca.sin(), le), le - te, 0.0, if s > 0.5 { anh } else { 0.0 })
        })
        .collect();
    let stab = keyed_wing(sk, Foil::sym(0.05), vec![seg(0.55, 0.80, 1), seg(0.80, 2.85, 8)], Cap::Flat, Cap::Flat);
    let (st, _) = stab.build();
    let mut stt = st.clone();
    stt.extend(mirror_x(&st));
    let sliv = Livery::new(paint(LIGHT)).all_neg(vec![sf(|p| -0.10 - p.y + 0.0 * p.x - 0.2 * (p.x.abs() - 0.8).max(0.0) * 0.176), sf(camo)], paint(DARKG)).all_neg(
        vec![sf(|p| -0.10 - 0.176 * (p.x.abs() - 0.8).max(0.0) - p.y)],
        paint(MED),
    );
    let spiv = vec3(0.0, -0.12, -4.75);
    parts.push(hinged(PartKind::Elevator, &stt, &sliv, spiv, Vec3::X, Vec3::Y, 25f32.to_radians()));

    // ---------------------------------------------------------- fin + rudder
    let fk: Vec<Key> = [(0.50f32, -2.55f32, -6.30f32), (0.70, -2.62, -6.30), (3.20, -5.44, -6.49)]
        .iter()
        .map(|&(y, le, te)| key(y, vec3(0.0, y, le), le - te, 0.0, 90.0))
        .collect();
    let fk2 = fk.clone();
    let rh = |y: f32| {
        let f = key_frame(&fk2, y);
        (f.le.z - (-5.72 - 0.05 * (y - 0.95))) / f.chord
    };
    let mut fin = keyed_wing(fk, Foil::sym(0.055), vec![seg(0.50, 0.95, 2), cut(0.95, 2.80, 8, rh(0.95), rh(2.80)), seg(2.80, 3.20, 2)], Cap::Flat, Cap::Flat);
    fin.k1 = 14;
    let (ft, fc) = fin.build();
    let fin_proj: Proj = Rc::new(|p: Vec3| vec2(-p.z, p.y));
    let finliv = chevron(Livery::new(paint(MED)), fin_proj, vec2(5.55, 2.45), 0.55, vec![], paint(MARK))
        .all_neg(vec![sf(|p| -camo(p) + 0.3)], paint(DARKG));
    emit(&mut body, &ft, &finliv);
    let r = &fc[0];
    parts.push(hinged(PartKind::Rudder, &r.tris, &finliv, (r.a + r.b) * 0.5, r.b - r.a, -Vec3::X, 30f32.to_radians()));
    // drag-chute / ECM fairing at the fin root trailing edge
    let fair = tube(vec3(0.0, 0.66, -5.2), vec3(0.0, 0.66, -6.75), Vec3::X, &teardrop(10, 0.3, 0.10, 0.12), 2.2, 14);
    emit(&mut body, &fair, &Livery::new(paint(MED)));

    // ventral fins
    let vk: Vec<Key> = [(0.0f32, -3.55f32, -4.35f32), (0.42, -4.05, -4.62)]
        .iter()
        .map(|&(s, le, te)| key(s, vec3(0.50 + s * 0.34, -0.62 - s * 0.94, le), le - te, 0.0, -70.0))
        .collect();
    let vf = keyed_wing(vk, Foil::sym(0.06), vec![seg(0.0, 0.42, 2)], Cap::Flat, Cap::Flat);
    let (vt, _) = vf.build();
    let mut vtt = vt.clone();
    vtt.extend(mirror_x(&vt));
    emit_plain(&mut body, &vtt, paint(LIGHT));

    // ---------------------------------------------------------- engine nozzle
    let mut noz = Vec::new();
    let mut noz_in = Vec::new();
    let around = 48;
    let petals = 15.0;
    let ring = |z: f32, r: f32, flute: f32| -> Vec<Vec3> {
        (0..around)
            .map(|j| {
                let a = TAU * j as f32 / around as f32;
                let rr = r * (1.0 + flute * (a * petals).cos());
                vec3(rr * a.cos(), rr * a.sin(), z)
            })
            .collect()
    };
    let zb = L_Z0 - 13.80;
    let nrows = vec![
        ring(zb + 0.01, 0.552, 0.0),
        ring(zb - 0.15, 0.545, 0.004),
        ring(NOZZLE_Z + 0.25, 0.50, 0.012),
        ring(NOZZLE_Z, 0.470, 0.016),
        ring(NOZZLE_Z - 0.002, 0.440, 0.010),
        ring(NOZZLE_Z + 0.35, 0.420, 0.0),
    ];
    let g = grid(&nrows, true, false);
    let half = g.len() * 3 / 5;
    noz.extend_from_slice(&g[..half]);
    noz_in.extend_from_slice(&g[half..]);
    let tail_cone = lathe(vec3(0.0, 0.0, NOZZLE_Z + 0.35), -Vec3::Z, Vec3::Y, &[(0.0, 0.42), (0.05, 0.40), (0.25, 0.22), (0.40, 0.0)], 24, 1.0, false);
    emit_plain(&mut body, &noz, metal(NOZ));
    emit_plain(&mut body, &noz_in, matte(BURNT));
    emit_plain(&mut body, &tail_cone, metal(BURNT));

    // ---------------------------------------------------------- afterburner flame (EMISSIVE, nested)
    let zf = NOZZLE_Z + 0.10;
    let mut layers: Vec<(Vec<Tri>, Surf)> = Vec::new();
    let cone = |r0: f32, len: f32, e: f32| -> Vec<Tri> {
        let prof: Vec<(f32, f32)> = (0..=10).map(|i| {
            let t = i as f32 / 10.0;
            (-len * t, r0 * (1.0 - t).max(0.0).powf(e) * (1.0 + 0.15 * (t * 6.0).sin().abs() * (1.0 - t)))
        }).collect();
        two_sided(&lathe(vec3(0.0, 0.0, zf), Vec3::Z, Vec3::Y, &prof, 24, 1.0, false))
    };
    layers.push((cone(0.43, 3.4, 0.7), flame([0.30, 0.13, 0.20], 1.6)));
    layers.push((cone(0.36, 2.6, 0.8), flame([0.85, 0.40, 0.12], 2.2)));
    layers.push((cone(0.26, 1.7, 0.9), flame([1.0, 0.78, 0.45], 3.2)));
    let mut diamonds = Vec::new();
    for k in 0..4 {
        let z = NOZZLE_Z - 0.45 - 0.55 * k as f32;
        let r = 0.15 - 0.025 * k as f32;
        diamonds.extend(ellipsoid(vec3(0.0, 0.0, z + 0.12), vec3(0.0, 0.0, z - 0.12), Vec3::X, r, r, 6, 12));
    }
    layers.push((diamonds, flame([1.0, 0.92, 0.75], 4.0)));
    parts.push(multi_part(PartKind::Afterburner, &layers, vec3(0.0, 0.0, NOZZLE_Z), Vec3::Z, 0.0));

    // ---------------------------------------------------------- nose gear
    {
        let (mut tyre, mut hb, mut leg, mut chrome) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        wheel(nhub, nr, nw, &mut tyre, &mut hb);
        let top = npiv;
        let fork_top = nhub + vec3(0.0, 0.42, 0.10);
        leg.extend(cyl(top + vec3(0.0, 0.04, 0.0), fork_top + vec3(0.0, 0.25, 0.0), 0.065, 14));
        chrome.extend(cyl(fork_top + vec3(0.0, 0.28, 0.0), fork_top, 0.048, 14));
        for sx in [1.0f32, -1.0] {
            leg.extend(tube(fork_top + vec3(0.11 * sx, 0.0, 0.0), nhub + vec3(0.11 * sx, 0.0, 0.0), Vec3::Z, &[(0.0, 0.03, 0.05), (1.0, 0.025, 0.04)], 2.5, 10));
        }
        leg.extend(obox(fork_top, [Vec3::X, Vec3::Y, Vec3::Z], vec3(0.13, 0.03, 0.06)));
        leg.extend(cyl(nhub - vec3(0.12, 0.0, 0.0), nhub + vec3(0.12, 0.0, 0.0), 0.03, 8));
        // drag brace
        leg.extend(cyl(top + vec3(0.0, -0.05, -0.55), fork_top + vec3(0.0, 0.35, -0.02), 0.03, 8));
        // taxi light
        let tl = lens(fork_top + vec3(0.0, 0.42, 0.08), Vec3::Z, 0.05, 0.05);
        let groups = [(tyre, matte(TYRE)), (hb, paint(LIGHT)), (leg, paint(LIGHT)), (chrome, metal([0.55, 0.56, 0.58])), (tl, emissive(WHITE_LIGHT, 2.0))];
        parts.push(multi_part(PartKind::GearNose, &groups, npiv, naxis, nang));
        // two doors hinged at the bay sides
        for sx in [1.0f32, -1.0] {
            let b2 = belly.clone();
            let (x0, x1) = if sx > 0.0 { (0.006, nbay.1) } else { (nbay.0, -0.006) };
            let d = hf_slab(&move |x, z| b2(x, z), x0, x1, nbay.2, nbay.3, 3, 6, true, 0.004, 0.012);
            let hx = if sx > 0.0 { nbay.1 } else { nbay.0 };
            let hinge = vec3(hx, belly(hx, (nbay.2 + nbay.3) * 0.5) + 0.01, (nbay.2 + nbay.3) * 0.5);
            // closed at deflection 1; open = inner edge swung down ~95 deg
            let axis = orient_axis(Vec3::Z, hinge, vec3(0.0, hinge.y, hinge.z), Vec3::Y);
            parts.push(part_at_one(PartKind::GearDoors, &[(d, paint(LIGHT))], hinge, axis, 95f32.to_radians()));
        }
    }

    // ---------------------------------------------------------- main gear
    for sx in [1.0f32, -1.0] {
        let (mut tyre, mut hb, mut leg, mut chrome) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        wheel(mhub, mr, mw, &mut tyre, &mut hb);
        let lv = (mleg - mpiv).normalize();
        leg.extend(cyl(mpiv - lv * 0.04, mpiv + lv * 0.62, 0.075, 14));
        chrome.extend(cyl(mpiv + lv * 0.55, mleg + vec3(0.0, 0.08, 0.0), 0.055, 14));
        leg.extend(cyl(mleg - vec3(0.03, 0.0, 0.0), mhub, 0.045, 10));
        let groups = [
            (side_tris(&tyre, sx), matte(TYRE)),
            (side_tris(&hb, sx), paint(LIGHT)),
            (side_tris(&leg, sx), paint(LIGHT)),
            (side_tris(&chrome, sx), metal([0.55, 0.56, 0.58])),
        ];
        let (ax, pv) = if sx > 0.0 { (maxis, mpiv) } else { (mirror_axis(maxis), mx(mpiv)) };
        parts.push(multi_part(PartKind::GearMain, &groups, pv, ax, mang));
        // main door, hinged along its outer edge, hanging down when open
        let b2 = belly.clone();
        let d = hf_slab(&move |x, z| b2(x, z), mbay.0, mbay.1, mbay.2, mbay.3, 16, 12, true, 0.006, 0.014);
        let hinge = vec3(mbay.1, belly(mbay.1, (mbay.2 + mbay.3) * 0.5) + 0.01, (mbay.2 + mbay.3) * 0.5);
        let axis = orient_axis(Vec3::Z, hinge, vec3(mbay.0, hinge.y, hinge.z), Vec3::Y);
        let (d, hinge, axis) = if sx > 0.0 { (d, hinge, axis) } else { (mirror_x(&d), mx(hinge), mirror_axis(axis)) };
        parts.push(part_at_one(PartKind::GearDoors, &[(d, paint(LIGHT))], hinge, axis, 92f32.to_radians()));
    }
    let _ = mhs;

    // ---------------------------------------------------------- details
    let mut dk = Vec::new();
    // pitot boom
    dk.extend(tube(vec3(0.0, 0.08, L_Z0 - 0.05), vec3(0.0, 0.08, L_Z0 + 0.45), Vec3::Y, &[(0.0, 0.035, 0.035), (0.3, 0.022, 0.022), (1.0, 0.012, 0.012)], 2.0, 10));
    // blade antennas
    for (s, y, h) in [(6.2f32, 0.70f32, 0.20f32), (9.0, -0.80, -0.16)] {
        let c = vec3(0.0, y + h * 0.5, L_Z0 - s);
        dk.extend(obox(c, [Vec3::X, Vec3::Y, Vec3::Z], vec3(0.006, h.abs() * 0.5, 0.06)));
    }
    emit_plain(&mut body, &dk, paint(DARKG));

    // ---------------------------------------------------------- lights
    let nav = lens(vec3(4.67, wy + 0.07, -0.75), Vec3::Z, 0.08, 0.022);
    emit_plain(&mut body, &nav, emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&nav), emissive(NAV_GREEN, NAV_K));
    let tail = lens(vec3(0.0, 3.05, -6.50), Vec3::Z, 0.05, 0.022);
    emit_plain(&mut body, &tail, emissive(WHITE_LIGHT, NAV_K));
    let strobe = lens(vec3(0.0, 3.21, -5.95), Vec3::Z, 0.10, 0.025);
    parts.push(plain_part(PartKind::Strobe, &strobe, emissive(STROBE_COL, STROBE_K), Vec3::ZERO, Vec3::Y, 0.0));
    let beacon = lens(vec3(0.0, -0.80, -1.0), Vec3::Z, 0.10, 0.04);
    parts.push(plain_part(PartKind::Beacon, &beacon, emissive(BEACON_COL, BEACON_K), Vec3::ZERO, Vec3::Y, 0.0));

    finish(body, parts, vec3(0.0, 0.95, 4.20))
}
