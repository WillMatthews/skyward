//! Peregrine Mk IX: elliptical-wing V12 warbird (taildragger, retractable
//! outward-folding mains, fixed tailwheel). Original markings.

use super::geom::*;
use super::{AircraftModel, PartKind};
use glam::{vec2, vec3, Mat4, Quat, Vec2, Vec3};
use std::f32::consts::PI;
use std::rc::Rc;

/// z of the fuselage station s = 0 (spinner base); z = P_Z0 - s.
const P_Z0: f32 = 3.05;
const THRUST_Y: f32 = 0.10;
const PROP_Z: f32 = 3.25;

const DK_GREEN: [f32; 3] = [0.040, 0.062, 0.024];
const OCEAN_GREY: [f32; 3] = [0.100, 0.110, 0.122];
const SEA_GREY: [f32; 3] = [0.300, 0.325, 0.340];
const SKY_BAND: [f32; 3] = [0.42, 0.48, 0.40];
const ID_YELLOW: [f32; 3] = [0.78, 0.52, 0.04];
const INS_CREAM: [f32; 3] = [0.74, 0.70, 0.56];
const INS_BLUE: [f32; 3] = [0.012, 0.040, 0.130];
const EXH: [f32; 3] = [0.045, 0.035, 0.030];
const STAIN: [f32; 3] = [0.075, 0.077, 0.072];

const FUS: [FusKey; 20] = [
    // s     w      top    bot     waist  nt   nb
    [0.00, 0.330, 0.430, -0.230, 0.10, 2.0, 2.0],
    [0.10, 0.360, 0.470, -0.285, 0.10, 2.1, 2.0],
    [0.40, 0.395, 0.515, -0.370, 0.09, 2.2, 2.1],
    [0.90, 0.415, 0.540, -0.440, 0.07, 2.3, 2.2],
    [1.50, 0.428, 0.552, -0.505, 0.05, 2.4, 2.3],
    [2.10, 0.438, 0.560, -0.580, 0.03, 2.4, 2.4],
    [2.55, 0.443, 0.565, -0.640, 0.02, 2.4, 2.5],
    [3.00, 0.445, 0.560, -0.690, 0.02, 2.4, 2.5],
    [3.60, 0.440, 0.560, -0.700, 0.02, 2.4, 2.5],
    [3.95, 0.430, 0.600, -0.695, 0.02, 2.3, 2.4],
    [4.25, 0.415, 0.770, -0.680, 0.03, 2.2, 2.4],
    [4.60, 0.395, 0.775, -0.660, 0.03, 2.1, 2.3],
    [5.00, 0.365, 0.735, -0.620, 0.04, 2.1, 2.3],
    [6.00, 0.290, 0.625, -0.500, 0.06, 2.0, 2.2],
    [7.00, 0.210, 0.540, -0.380, 0.08, 2.0, 2.1],
    [7.80, 0.150, 0.490, -0.280, 0.10, 2.0, 2.0],
    [8.40, 0.090, 0.460, -0.190, 0.12, 2.0, 2.0],
    [8.62, 0.055, 0.448, -0.140, 0.13, 2.0, 2.0],
    [8.74, 0.025, 0.440, -0.100, 0.14, 2.0, 2.0],
    [8.80, 0.006, 0.436, -0.080, 0.14, 2.0, 2.0],
];

fn ps(p: Vec3) -> f32 {
    P_Z0 - p.z
}

/// Disruptive two-colour pattern: > 0 = dark green, < 0 = ocean grey.
fn camo(p: Vec3) -> f32 {
    let (x, y, z) = (p.x.abs() * 0.85 + p.x * 0.15, p.y, p.z);
    (1.35 * x + 0.95 * z + 0.4).sin() + 0.85 * (-0.75 * x + 1.55 * z + 2.1 + 1.6 * y).sin() + 0.55 * (2.3 * x - 0.5 * z + 1.0 - 1.2 * y).sin() - 0.15
}

/// Invented insignia: cream lozenge, dark-blue lozenge, yellow centre disc.
/// `q` = coordinates in the marking plane, `a` = outer half-diagonal.
fn insignia(liv: Livery, proj: Proj, c: Vec2, a: f32, masks: Vec<SF>) -> Livery {
    let p1 = proj.clone();
    let p2 = proj.clone();
    let p3 = proj;
    let lz = move |q: Vec2, r: f32| ((q.x.abs() + q.y.abs()) - r) * std::f32::consts::FRAC_1_SQRT_2;
    let mut m1 = vec![sf(move |p| (p1(p) - c).length() - a * 0.30)];
    m1.extend(masks.clone());
    let mut m2 = vec![sf(move |p| lz(p2(p) - c, a * 0.74))];
    m2.extend(masks.clone());
    let mut m3 = vec![sf(move |p| lz(p3(p) - c, a))];
    m3.extend(masks);
    liv.all_neg(m1, paint(ID_YELLOW)).all_neg(m2, paint(INS_BLUE)).all_neg(m3, paint(INS_CREAM))
}

pub fn peregrine() -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts = Vec::new();

    // ---------------------------------------------------------- wing (needed by several liveries)
    let dih = 6.0f32;
    let ty = dih.to_radians().tan();
    let (semi, c0, zq, y0) = (5.60f32, 2.54f32, 0.12f32, -0.47f32);
    let chord_at = move |x: f32| c0 * (1.0 - (x / semi).powi(2)).max(0.0).sqrt();
    let mut wkeys = Vec::new();
    let xs: Vec<f32> = (0..=16).map(|i| semi * 0.997 * (i as f32 / 16.0 * PI * 0.5).sin()).collect();
    for &x in &xs {
        let c = chord_at(x).max(0.16);
        let tw = 2.0 - 2.5 * (x / semi);
        wkeys.push(key(x, vec3(x, y0 + x * ty, zq + 0.25 * c), c, tw, dih));
    }
    let s_tip = *xs.last().unwrap();
    let mut wing = keyed_wing(
        wkeys,
        Foil { m: 0.02, p: 0.4, t: 0.125 },
        vec![seg(0.0, 3.25, 16), cut(3.25, 5.0, 10, 0.74, 0.70), seg(5.0, s_tip, 6)],
        Cap::Open,
        Cap::Round(0.025),
    );
    wing.k1 = 20;
    wing.k2 = 8;
    let wing = Rc::new(wing);

    // ---------------------------------------------------------- landing gear geometry (folds)
    let (mr, mw) = (0.345f32, 0.165f32);
    let piv = vec3(0.60, -0.43, 0.42);
    let hub = vec3(0.90, -1.75 + mr, 0.90);
    let leg_end = hub - vec3(0.115, 0.0, 0.0);
    let reach = (hub - piv).length();
    let w2 = wing.clone();
    let mid_y = move |x: f32, z: f32| {
        let f = w2.frac(x, z);
        (w2.skin(x, f, true).y + w2.skin(x, f, false).y) * 0.5
    };
    let target = vec3(1.66, 0.0, 0.10);
    let target = vec3(target.x, mid_y(target.x, target.z), target.z);
    let dir = (target - piv).normalize();
    let hs = piv + dir * reach;
    let wing_up = Quat::from_rotation_z(dih.to_radians()) * Vec3::Y;
    let (gaxis, gang) = fold(hub - piv, hs - piv, Vec3::X, wing_up);
    let stow = |p: Vec3| piv + Quat::from_axis_angle(gaxis, gang) * (p - piv);
    let hs = stow(hub);
    let ls = stow(leg_end);

    // ---------------------------------------------------------- fuselage
    let fus = Fuselage::new(&FUS);
    let rows = fus.rows(P_Z0, 0.10, 40, (2.0, 5.0, 1.6));
    let mut t = grid(&rows, true, false);
    t.extend(fan(&rows[0], Vec3::Z));
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    let side: Proj = Rc::new(|p: Vec3| vec2(ps(p), p.y));
    let demarc = |s: f32| -0.16 + 0.30 * smooth(5.0, 8.6, s) - 0.06 * smooth(0.0, 1.5, s);
    let on_side = |s0: f32, s1: f32| masked(sf(|p| 0.12 - p.x.abs()), side.clone(), vec2(s0, -1.0), vec2(s1, 1.0));
    let mut fl = Livery::new(paint(SEA_GREY));
    // invented insignia on both sides
    fl = insignia(fl, side.clone(), vec2(5.75, 0.10), 0.36, vec![on_side(5.2, 6.3)]);
    let fl = fl
        // rear fuselage band
        .band(|p| ps(p) - 7.05, |p| ps(p) - 7.38, vec![], paint(SKY_BAND))
        // exhaust staining along the cowl
        .all_neg(
            vec![
                masked(sf(|p| (ps(p) - 1.6).abs() - 1.05 - 0.02 * (p.y * 30.0).sin()), side.clone(), vec2(0.3, -0.2), vec2(2.9, 0.6)),
                masked(sf(|p| (p.y - 0.17 + 0.05 * smooth(0.5, 2.6, ps(p))).abs() - 0.05 - 0.04 * smooth(0.5, 2.6, ps(p))), side.clone(), vec2(0.3, -0.2), vec2(2.9, 0.6)),
                sf(|p| 0.15 - p.x.abs()),
            ],
            matte(STAIN),
        )
        .all_neg(vec![sf(move |p| demarc(ps(p)) - p.y), sf(camo)], paint(DK_GREEN))
        .all_neg(vec![sf(move |p| demarc(ps(p)) - p.y)], paint(OCEAN_GREY));
    emit(&mut body, &t, &fl);

    // wing-root fillets (upper trailing edge)
    let mut fil = Vec::new();
    for sx in [1.0f32, -1.0] {
        let prof: Vec<(f32, f32, f32)> = (0..=10)
            .map(|i| {
                let t = i as f32 / 10.0;
                let f = (1.0 - (1.0 - (t / 0.35).min(1.0)).powi(2)).sqrt() * (1.0 - smooth(0.35, 1.0, t)).powf(0.7);
                let k = (1.0 - t.powi(6)).max(0.0).sqrt();
                (t, (0.06 + 0.26 * f) * k, (0.03 + 0.12 * f) * k)
            })
            .collect();
        fil.extend(tube(vec3(0.40 * sx, -0.33, 0.30), vec3(0.26 * sx, -0.28, -2.10), Vec3::Y, &prof, 2.0, 16));
    }
    emit(&mut body, &fil, &Livery::new(paint(OCEAN_GREY)).all_neg(vec![sf(camo)], paint(DK_GREEN)));

    // canopy: windscreen + sliding bubble hood, framed, fairing into the spine
    let can_keys: [FusKey; 10] = [
        [2.50, 0.240, 0.585, 0.30, 0.40, 2.3, 2.0],
        [2.60, 0.290, 0.700, 0.30, 0.40, 2.4, 2.0],
        [2.78, 0.325, 0.880, 0.30, 0.40, 2.5, 2.0],
        [2.98, 0.345, 0.975, 0.30, 0.40, 2.4, 2.0],
        [3.30, 0.352, 1.010, 0.30, 0.40, 2.3, 2.0],
        [3.70, 0.345, 0.995, 0.30, 0.40, 2.3, 2.0],
        [4.05, 0.325, 0.950, 0.30, 0.40, 2.3, 2.0],
        [4.40, 0.300, 0.865, 0.30, 0.40, 2.3, 2.0],
        [4.80, 0.250, 0.800, 0.30, 0.40, 2.2, 2.0],
        [5.10, 0.180, 0.700, 0.30, 0.40, 2.1, 2.0],
    ];
    let can = Fuselage::new(&can_keys);
    let crow = can.rows(P_Z0, 0.05, 40, (0.0, 0.0, 1.0));
    let mut ct = grid(&crow, true, false);
    ct.extend(fan(&crow[0], Vec3::Z));
    ct.extend(fan(crow.last().unwrap(), -Vec3::Z));
    let sill = |s: f32| 0.555 + 0.02 * (s - 2.6);
    let frame = |s0: f32, w: f32| move |p: Vec3| (ps(p) - s0).abs() - w;
    let cliv = Livery::new(paint(DK_GREEN))
        // frames (painted) win over glass
        .all_neg(vec![sf(frame(2.62, 0.022))], paint(DK_GREEN))
        .all_neg(vec![sf(frame(2.84, 0.024))], paint(DK_GREEN))
        .all_neg(vec![sf(frame(4.08, 0.03))], paint(DK_GREEN))
        .all_neg(vec![sf(move |p| p.y - sill(ps(p)) - 0.035)], paint(DK_GREEN))
        .all_neg(vec![sf(|p| ps(p) - 4.10)], glass(GLASS))
        .all_neg(vec![sf(camo)], paint(DK_GREEN));
    let cliv = Livery { base: paint(OCEAN_GREY), layers: cliv.layers };
    emit(&mut body, &ct, &cliv);

    // ---------------------------------------------------------- wing skin
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let w3 = wing.clone();
    let upper = move |p: Vec3| {
        let x = p.x.abs().min(s_tip);
        let f = w3.frac(x, p.z).clamp(0.0, 1.0);
        let m = (w3.skin(x, f, true).y + w3.skin(x, f, false).y) * 0.5;
        m - p.y
    };
    let up2 = upper.clone();
    let lead = move |p: Vec3| {
        let x = p.x.abs();
        let c = chord_at(x).max(0.16);
        let le = zq + 0.25 * c;
        (le - p.z) / c - 0.075
    };
    let top: Proj = Rc::new(|p: Vec3| vec2(p.x.abs(), p.z));
    let mut wliv = Livery::new(paint(SEA_GREY))
        // wheel wells (dark), lower surface
        .all_neg(vec![sf(move |p| vec2(p.x.abs() - hs.x, p.z - hs.z).length() - mr - 0.03), sf(move |p| -up2(p))], matte(DARK))
        // identification stripe on the outer leading edge
        .all_neg(vec![sf(lead), sf(|p| p.x.abs() - 4.30), sf(|p| 3.55 - p.x.abs())], paint(ID_YELLOW));
    // insignia on the upper and lower wing
    let up3 = upper.clone();
    let up4 = upper.clone();
    wliv = insignia(wliv, top.clone(), vec2(3.70, zq - 0.15), 0.42, vec![sf(move |p| up3(p))]);
    wliv = insignia(wliv, top.clone(), vec2(3.70, zq - 0.15), 0.42, vec![sf(move |p| -up4(p))]);
    let up5 = upper.clone();
    let up6 = upper.clone();
    let wliv = wliv
        .all_neg(vec![sf(move |p| up5(p)), sf(camo)], paint(DK_GREEN))
        .all_neg(vec![sf(move |p| up6(p))], paint(OCEAN_GREY));
    emit(&mut body, &wt, &wliv);
    let c = &ctrls[0];
    let (lp, la) = ((c.a + c.b) * 0.5, (c.b - c.a).normalize());
    parts.push(hinged(PartKind::AileronLeft, &c.tris, &wliv, lp, la, -Vec3::Y, 20f32.to_radians()));
    parts.push(hinged(PartKind::AileronRight, &mirror_x(&c.tris), &wliv, mx(lp), mx(la), Vec3::Y, 20f32.to_radians()));

    // split flaps under the inner wing (hinge line straight, TE at 97 %)
    let (fs0, fs1) = (0.48f32, 3.15f32);
    let hz = |s: f32| {
        let (za, zb) = (zq + 0.25 * c0 - 0.73 * chord_at(fs0), zq + 0.25 * chord_at(fs1) - 0.73 * chord_at(fs1));
        za + (zb - za) * (s - fs0) / (fs1 - fs0)
    };
    let w4 = wing.clone();
    let xr = move |s: f32| (w4.frac(s, hz(s)), 0.975);
    let flap = wing.panel(fs0, fs1, 10, &xr, 4, false, 0.003, 0.012);
    let fa = wing.skin(fs0, wing.frac(fs0, hz(fs0)), false);
    let fb = wing.skin(fs1, wing.frac(fs1, hz(fs1)), false);
    let flap_liv = Livery::new(paint(SEA_GREY));
    parts.push(hinged(PartKind::FlapLeft, &flap, &flap_liv, (fa + fb) * 0.5, fb - fa, -Vec3::Y, 80f32.to_radians()));
    parts.push(hinged(PartKind::FlapRight, &mirror_x(&flap), &flap_liv, mx((fa + fb) * 0.5), mx(fb - fa), -Vec3::Y, 80f32.to_radians()));

    // ---------------------------------------------------------- radiators under the wings
    let mut rad = Vec::new();
    let mut rad_dark = Vec::new();
    for sx in [1.0f32, -1.0] {
        let (x, zf, zr) = (1.62f32, -0.30f32, -1.50f32);
        let ylo = wing.skin(x, wing.frac(x, -0.7), false).y;
        let yc = ylo - 0.07;
        let prof: Vec<(f32, f32, f32)> = (0..=12)
            .map(|i| {
                let t = i as f32 / 12.0;
                let f = if t < 0.15 { 0.80 + 0.20 * (1.0 - ((0.15 - t) / 0.15).powi(2)).sqrt() } else { 1.0 - 0.45 * smooth(0.55, 1.0, t) };
                (t, 0.30 * f, 0.17 * f)
            })
            .collect();
        let (a, b) = (vec3(x * sx, yc, zf), vec3(x * sx, yc + 0.05, zr));
        let rows = tube_rows(a, b, Vec3::X, &prof, 4.0, 24);
        rad.extend(grid(&rows, true, false));
        // inlet face and recessed matrix
        rad_dark.extend(fan(&rows[0], Vec3::Z));
        rad_dark.extend(fan(rows.last().unwrap(), -Vec3::Z));
    }
    let rliv = Livery::new(paint(SEA_GREY));
    emit(&mut body, &rad, &rliv);
    emit_plain(&mut body, &rad_dark, matte(DARK));

    // ---------------------------------------------------------- empennage
    let fin_pts: [(f32, f32, f32); 8] = [
        // y, LE z, TE z
        (0.30, -4.30, -5.78),
        (0.55, -4.45, -5.80),
        (0.85, -4.62, -5.82),
        (1.10, -4.80, -5.81),
        (1.30, -4.98, -5.78),
        (1.46, -5.15, -5.72),
        (1.58, -5.33, -5.62),
        (1.64, -5.47, -5.53),
    ];
    let fin_keys: Vec<Key> = fin_pts.iter().map(|&(y, le, te)| key(y, vec3(0.0, y, le), le - te, 0.0, 90.0)).collect();
    let fk = fin_keys.clone();
    let hinge_z = -5.30f32;
    let fin_h = move |y: f32| {
        let f = key_frame(&fk, y);
        (f.le.z - hinge_z) / f.chord
    };
    let fin = keyed_wing(
        fin_keys,
        Foil::sym(0.10),
        vec![seg(0.30, 0.56, 2), cut(0.56, 1.52, 6, fin_h(0.56), fin_h(1.52)), seg(1.52, 1.64, 2)],
        Cap::Flat,
        Cap::Round(0.02),
    );
    let (ft, fc) = fin.build();
    let finliv = Livery::new(paint(OCEAN_GREY)).all_neg(vec![sf(|p| -camo(p) + 0.6)], paint(DK_GREEN));
    emit(&mut body, &ft, &finliv);
    let r = &fc[0];
    parts.push(hinged(PartKind::Rudder, &r.tris, &finliv, (r.a + r.b) * 0.5, r.b - r.a, -Vec3::X, 28f32.to_radians()));

    let sy = 0.36f32;
    let stab_pts: [(f32, f32, f32); 7] = [(0.0, -4.48, -5.70), (0.50, -4.55, -5.68), (0.90, -4.66, -5.62), (1.20, -4.80, -5.54), (1.42, -4.96, -5.44), (1.56, -5.10, -5.32), (1.62, -5.18, -5.24)];
    let stab_keys: Vec<Key> = stab_pts.iter().map(|&(x, le, te)| key(x, vec3(x, sy, le), le - te, 0.0, 0.0)).collect();
    let sk = stab_keys.clone();
    let ehz = -5.20f32;
    let stab_h = move |x: f32| {
        let f = key_frame(&sk, x);
        (f.le.z - ehz) / f.chord
    };
    let stab = keyed_wing(stab_keys, Foil::sym(0.10), vec![seg(0.0, 0.10, 1), cut(0.10, 1.50, 6, stab_h(0.10), stab_h(1.50)), seg(1.50, 1.62, 2)], Cap::Open, Cap::Round(0.02));
    let (sl, sc) = stab.build();
    let mut stt = sl.clone();
    stt.extend(mirror_x(&sl));
    let sliv = Livery::new(paint(SEA_GREY)).all_neg(vec![sf(move |p| sy - p.y), sf(camo)], paint(DK_GREEN)).all_neg(vec![sf(move |p| sy - p.y)], paint(OCEAN_GREY));
    emit(&mut body, &stt, &sliv);
    let e = &sc[0];
    let mut et = e.tris.clone();
    et.extend(mirror_x(&e.tris));
    let epiv = vec3(0.0, (e.a.y + e.b.y) * 0.5, (e.a.z + e.b.z) * 0.5);
    parts.push(hinged(PartKind::Elevator, &et, &sliv, epiv, Vec3::X, Vec3::Y, 25f32.to_radians()));

    // ---------------------------------------------------------- engine details
    let mut exh = Vec::new();
    for sx in [1.0f32, -1.0] {
        for k in 0..6 {
            let s = 0.62 + 0.20 * k as f32;
            let y = 0.19;
            let [w, top, _bot, waist, nt, _] = fus.param(s);
            let fy = ((y - waist) / (top - waist)).clamp(0.0, 1.0);
            let xw = w * (1.0 - fy.powf(nt)).max(0.0).powf(1.0 / nt);
            let base = vec3((xw - 0.03) * sx, y, P_Z0 - s);
            let tip = base + vec3(0.085 * sx, -0.025, -0.06);
            exh.extend(tube(base, tip, Vec3::Y, &[(0.0, 0.045, 0.030), (0.8, 0.048, 0.032), (1.0, 0.040, 0.026)], 3.0, 10));
        }
    }
    emit_plain(&mut body, &exh, metal(EXH));
    // carburettor intake under the nose
    let mut intake = tube(
        vec3(0.0, -0.40, P_Z0 - 0.95),
        vec3(0.0, -0.46, P_Z0 - 2.15),
        Vec3::X,
        &[(0.0, 0.10, 0.075), (0.08, 0.115, 0.09), (0.6, 0.11, 0.09), (1.0, 0.03, 0.03)],
        2.6,
        16,
    );
    intake.truncate(intake.len() - 16);
    emit(&mut body, &intake, &Livery::new(paint(SEA_GREY)));
    let mouth = fan(&tube_rows(vec3(0.0, -0.40, P_Z0 - 0.95), vec3(0.0, -0.46, P_Z0 - 2.15), Vec3::X, &[(0.0, 0.10, 0.075)], 2.6, 16)[0], Vec3::Z);
    emit_plain(&mut body, &mouth, matte(DARK));

    // ---------------------------------------------------------- details
    let mut dk = Vec::new();
    // aerial mast behind the canopy
    let ms = 4.95;
    let mb = vec3(0.0, fus.top(ms) - 0.02, P_Z0 - ms);
    let rake = Quat::from_rotation_x(0.30);
    dk.extend(tube(mb, mb + rake * vec3(0.0, 0.42, 0.0), Vec3::Z, &[(0.0, 0.035, 0.010), (1.0, 0.012, 0.005)], 2.0, 10));
    // pitot under the left wing
    dk.extend(cyl(vec3(3.9, -0.27 - 0.03, 0.40), vec3(3.9, -0.27 - 0.03, 0.62), 0.012, 8));
    emit_plain(&mut body, &dk, paint(DARK));

    // ---------------------------------------------------------- tail wheel (fixed)
    let (tr, tz) = (0.13f32, -5.40f32);
    let thub = vec3(0.0, -0.55 + tr, tz);
    let (mut tyres, mut hubs) = (Vec::new(), Vec::new());
    wheel(thub, tr, 0.075, &mut tyres, &mut hubs);
    let mut tm = Vec::new();
    for sx in [1.0f32, -1.0] {
        tm.extend(tube(thub + vec3(0.050 * sx, 0.0, 0.0), vec3(0.04 * sx, -0.10, tz + 0.12), Vec3::Z, &[(0.0, 0.016, 0.03), (1.0, 0.016, 0.03)], 2.5, 8));
    }
    tm.extend(cyl(vec3(0.0, -0.10, tz + 0.12), vec3(0.0, -0.02, tz + 0.16), 0.035, 10));
    tm.extend(cyl(thub - vec3(0.06, 0.0, 0.0), thub + vec3(0.06, 0.0, 0.0), 0.015, 8));
    emit_plain(&mut body, &tyres, matte(TYRE));
    emit_plain(&mut body, &hubs, paint(SEA_GREY));
    emit_plain(&mut body, &tm, metal(METAL));

    // ---------------------------------------------------------- main gear (one part per side)
    for sx in [1.0f32, -1.0] {
        let m = |v: Vec3| vec3(v.x * sx, v.y, v.z);
        let (mut tyre, mut hb, mut leg, mut chrome, mut door) = (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
        wheel(hub, mr, mw, &mut tyre, &mut hb);
        // oleo: upper cylinder, chromed slider, axle
        let lv = (leg_end - piv).normalize();
        leg.extend(cyl(piv - lv * 0.05, piv + lv * 0.62, 0.058, 14));
        chrome.extend(cyl(piv + lv * 0.55, leg_end + vec3(0.0, 0.06, 0.0), 0.042, 14));
        leg.extend(cyl(leg_end + vec3(-0.02, 0.0, 0.0), hub + vec3(0.02, 0.0, 0.0), 0.035, 10));
        // torque links (front)
        let tl0 = piv + lv * 0.62 + vec3(0.0, 0.0, 0.06);
        let tl1 = leg_end + vec3(0.0, 0.12, 0.07);
        let tlm = (tl0 + tl1) * 0.5 + vec3(0.0, 0.0, 0.10);
        leg.extend(obox((tl0 + tlm) * 0.5, frame3(tlm - tl0, Vec3::X), vec3((tlm - tl0).length() * 0.5, 0.012, 0.03)));
        leg.extend(obox((tl1 + tlm) * 0.5, frame3(tlm - tl1, Vec3::X), vec3((tlm - tl1).length() * 0.5, 0.012, 0.03)));
        // leg fairing door: flush with the wing lower skin when stowed
        let s0 = piv.x + 0.06;
        let s1 = ls.x - 0.05;
        let zc = move |s: f32| piv.z + (ls.z - piv.z) * (s - piv.x) / (ls.x - piv.x);
        let w5 = wing.clone();
        let dxr = move |s: f32| (w5.frac(s, zc(s) + 0.13), w5.frac(s, zc(s) - 0.13));
        let dpan = wing.panel(s0, s1, 6, &dxr, 3, false, 0.004, 0.012);
        door.extend(rotated(&dpan, piv, gaxis, -gang));
        let groups = [
            (map_tris(&tyre, m), matte(TYRE)),
            (map_tris(&hb, m), paint(SEA_GREY)),
            (map_tris(&leg, m), paint(SEA_GREY)),
            (map_tris(&chrome, m), metal([0.55, 0.56, 0.58])),
            (map_tris(&door, m), paint(SEA_GREY)),
        ];
        let (ax, pv) = if sx > 0.0 { (gaxis, piv) } else { (vec3(gaxis.x, -gaxis.y, -gaxis.z), m(piv)) };
        parts.push(multi_part(PartKind::GearMain, &groups, pv, ax, gang));
    }
    let _ = hs;

    // ---------------------------------------------------------- lights
    let tipy = y0 + s_tip * ty;
    let nav = lens(vec3(s_tip + 0.005, tipy, zq), Vec3::Z, 0.10, 0.025);
    emit_plain(&mut body, &nav, emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&nav), emissive(NAV_GREEN, NAV_K));
    let tail = lens(vec3(0.0, 0.40, -5.81), Vec3::Z, 0.05, 0.02);
    emit_plain(&mut body, &tail, emissive(WHITE_LIGHT, NAV_K));

    // ---------------------------------------------------------- propeller
    parts.push(propeller(&PropSpec {
        engine: 0,
        pivot: vec3(0.0, THRUST_Y, PROP_Z),
        radius: 1.60,
        blades: 4,
        chord: vec![(0.15, 0.20), (0.35, 0.27), (0.60, 0.29), (0.85, 0.24), (0.985, 0.13)],
        pitch: 4.6,
        root: 0.15,
        thick: 0.10,
        spinner: (PROP_Z - P_Z0, 0.66, 0.33),
        spinner_col: paint(SKY_BAND),
        blade_col: PROP_DARK,
        tip_col: ID_YELLOW,
        tip_from: 0.93,
        scimitar: 0.0,
        disc_col: [0.02, 0.02, 0.022],
    }));

    let _ = Mat4::IDENTITY;
    finish(body, parts, vec3(0.0, 0.75, -0.40))
}

