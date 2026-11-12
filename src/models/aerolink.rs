//! Aerolink 72: high-wing twin-turboprop regional airliner (T-tail, 6-blade
//! props, main gear retracting into fuselage sponsons, forward-retracting
//! nose gear). Fictional regional livery.

use super::airkit::*;
use super::geom::*;
use super::{AircraftModel, Part, PartKind};
use glam::{vec2, vec3, Mat4, Quat, Vec3};
use std::rc::Rc;

const Z0: f32 = 13.80;
const YC: f32 = 0.55;
const R: f32 = 1.30;
const LEN: f32 = 27.2;
const PROP: Vec3 = vec3(4.10, 0.90, 4.10);

const TEAL: [f32; 3] = [0.000, 0.120, 0.135];
const LIME: [f32; 3] = [0.30, 0.55, 0.035];
const WHITE_A: [f32; 3] = [0.82, 0.83, 0.83];

fn s_of(p: Vec3) -> f32 {
    Z0 - p.z
}

pub fn aerolink() -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts: Vec<Part> = Vec::new();
    let side = side_proj(Z0);
    let b = Body { len: LEN, r: R, r_top: 1.30, r_bot: 1.30, yc: YC, nose_len: 3.7, tail_len: 8.6 };
    let fus = Rc::new(Fuselage::new(&b.keys()));
    let nl = b.nose_len;

    // ---------------------------------------------------------- wing (high, on top of the fuselage)
    let wy = YC + R + 0.20;
    let dih = 2.5f32;
    let ty = dih.to_radians().tan();
    let xb = 4.95f32;
    let (x_tip, le_root, c_root, c_tip) = (13.48f32, 1.50f32, 2.57f32, 1.41f32);
    let le_tip = le_root - (x_tip - xb) * 3f32.to_radians().tan();
    let wkeys = vec![
        key(0.0, vec3(0.0, wy, le_root), c_root, 2.0, 0.0),
        key(xb, vec3(xb, wy, le_root), c_root, 2.0, 0.0),
        key(xb + 0.05, vec3(xb + 0.05, wy + 0.05 * ty, le_root), c_root - 0.005, 2.0, dih),
        key(x_tip, vec3(x_tip, wy + (x_tip - xb) * ty, le_tip), c_tip, 0.0, dih),
    ];
    let wk = wkeys.clone();
    let z_at = move |s: f32, f: f32| {
        let k = key_frame(&wk, s);
        k.le.z - f * k.chord
    };
    let wk2 = wkeys.clone();
    let lf = move |s: f32, za: (f32, f32), xa: (f32, f32)| {
        let t = ((s - xa.0) / (xa.1 - xa.0)).clamp(0.0, 1.0);
        let z = za.0 + (za.1 - za.0) * t;
        let f = key_frame(&wk2, s);
        (f.le.z - z) / f.chord
    };
    let (f0, f1, f2, f3) = (1.40f32, xb - 0.10, xb + 0.15, 8.70f32);
    let (a0, a1) = (8.85f32, 12.75f32);
    let fl1 = (z_at(f0, 0.70), z_at(f1, 0.70));
    let fl2 = (z_at(f2, 0.70), z_at(f3, 0.70));
    let al = (z_at(a0, 0.74), z_at(a1, 0.74));
    let mut wing = keyed_wing(
        wkeys,
        Foil { m: 0.025, p: 0.4, t: 0.15 },
        vec![
            seg(0.0, f0, 2),
            cut(f0, f1, 5, lf(f0, fl1, (f0, f1)), lf(f1, fl1, (f0, f1))),
            seg(f1, f2, 1),
            cut(f2, f3, 6, lf(f2, fl2, (f2, f3)), lf(f3, fl2, (f2, f3))),
            seg(f3, a0, 1),
            cut(a0, a1, 6, lf(a0, al, (a0, a1)), lf(a1, al, (a0, a1))),
            seg(a1, x_tip, 2),
        ],
        Cap::Open,
        Cap::Round(0.04),
    );
    wing.k1 = 16;
    wing.k2 = 6;
    let wing = Rc::new(wing);
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let w1 = wing.clone();
    let upper = move |p: Vec3| -> f32 {
        let x = p.x.abs().min(x_tip);
        let f = w1.frac(x, p.z).clamp(0.0, 1.0);
        (w1.skin(x, f, true).y + w1.skin(x, f, false).y) * 0.5 - p.y
    };
    let w2 = wing.clone();
    let boot = move |p: Vec3| -> f32 {
        // de-icing boots: dark leading-edge band
        let x = p.x.abs();
        if x < 1.4 {
            return 1.0;
        }
        w2.frac(x.min(x_tip), p.z) - 0.07
    };
    let wliv = Livery::new(paint(WHITE_A))
        .all_neg(vec![sf(move |p| x_tip - 0.35 - p.x.abs())], paint(TEAL))
        .all_neg(vec![sf(boot)], matte([0.035, 0.036, 0.04]))
        .all_neg(vec![sf(move |p| -upper(p))], paint(WING_GREY_LO));
    emit(&mut body, &wt, &wliv);
    for (i, c) in ctrls.iter().enumerate() {
        let lp = (c.a + c.b) * 0.5;
        let la = (c.b - c.a).normalize();
        if i < 2 {
            let pv = lp - Vec3::Y * 0.55 + Vec3::Z * 0.10;
            parts.push(hinged(PartKind::FlapLeft, &c.tris, &wliv, pv, la, -Vec3::Y, 33f32.to_radians()));
            parts.push(hinged(PartKind::FlapRight, &mirror_x(&c.tris), &wliv, mx(pv), mx(la), -Vec3::Y, 33f32.to_radians()));
        } else {
            parts.push(hinged(PartKind::AileronLeft, &c.tris, &wliv, lp, la, -Vec3::Y, 22f32.to_radians()));
            parts.push(hinged(PartKind::AileronRight, &mirror_x(&c.tris), &wliv, mx(lp), mx(la), Vec3::Y, 22f32.to_radians()));
        }
    }
    // one spoiler per wing ahead of the outer flap
    {
        let w4 = wing.clone();
        let fl = fl2;
        let hz = move |s: f32| fl.0 + (fl.1 - fl.0) * ((s - f2) / (f3 - f2)).clamp(0.0, 1.0);
        let xr = move |s: f32| {
            let h = w4.frac(s, hz(s));
            (h - 0.16, h - 0.01)
        };
        let (s0, s1) = (6.6f32, 8.5f32);
        let pan = wing.panel(s0, s1, 3, &xr, 3, true, 0.004, 0.012);
        let a = wing.skin(s0, xr(s0).0, true);
        let bb = wing.skin(s1, xr(s1).0, true);
        let probe = wing.skin((s0 + s1) * 0.5, xr((s0 + s1) * 0.5).1, true);
        let ax = orient_axis((bb - a).normalize(), a, probe, Vec3::Y);
        parts.push(plain_part(PartKind::Spoilers, &pan, paint(WHITE_A), a, ax, 50f32.to_radians()));
        parts.push(plain_part(PartKind::Spoilers, &mirror_x(&pan), paint(WHITE_A), mx(a), mirror_axis(ax), 50f32.to_radians()));
    }
    // flap track fairings
    let mut canoes = Vec::new();
    for x in [2.6f32, 7.0] {
        let f = wing.fr(x);
        let y = wing.skin(x, 0.7, false).y - 0.02;
        let (za, zb) = (f.le.z - f.chord * 0.55, f.le.z - f.chord * 1.15);
        canoes.extend(tube(vec3(x, y, za), vec3(x, y - 0.06, zb), Vec3::X, &teardrop(10, 0.42, 0.08, 0.20), 2.3, 14));
    }
    let mut cc = canoes.clone();
    cc.extend(mirror_x(&canoes));
    emit_plain(&mut body, &cc, paint(WING_GREY_LO));

    // ---------------------------------------------------------- fuselage
    let (w0, pitch, count, ww, wh, wyc) = (4.35f32, 0.762f32, 25usize, 0.25f32, 0.36f32, YC + 0.28);
    let mut extra = Vec::new();
    for k in 0..count {
        let s = w0 + pitch * k as f32;
        extra.extend([s - ww * 0.5, s, s + ww * 0.5]);
    }
    let doors = [(2.35f32, 3.60f32, YC - 0.95, YC + 0.95), (21.35, 22.10, YC - 0.95, YC + 0.95)];
    for &(s0, s1, _, _) in &doors {
        extra.extend([s0, s0 + 0.03, s1 - 0.03, s1]);
    }
    let rows = fus.rows_with(Z0, 0.30, 64, (0.0, nl, 3.0), &extra);
    let mut t = grid(&rows, true, false);
    t.extend(fan(&rows[0], Vec3::Z));
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    let mut liv = Livery::new(paint(WHITE_A));
    // cockpit glazing + dark surround
    liv = cockpit_panes(liv, Z0, YC, nl, &[(0.235, 0.415, 4.0, 4.0, 52.0, 55.0), (0.435, 0.575, 42.0, 42.0, 64.0, 64.0), (0.59, 0.69, 45.0, 52.0, 64.0, 64.0)], glass(WIN));
    liv = cockpit_panes(liv, Z0, YC, nl, &[(0.20, 0.73, 0.0, 30.0, 59.0, 69.0)], paint([0.012, 0.012, 0.014]));
    let wend = w0 + pitch * (count as f32 - 1.0);
    let wf = move |p: Vec3| window_row(s_of(p), p.y, w0, pitch, count, ww, wh, 0.08, wyc);
    let band = masked(sf(wf), side.clone(), vec2(w0 - 0.4, wyc - 0.4), vec2(wend + 0.4, wyc + 0.4));
    let sidem = {
        let side = side.clone();
        move || masked(sf(move |p| 0.5 * R - p.x.abs()), side.clone(), vec2(nl - 1.0, YC - 1.5 * R), vec2(LEN - 2.0, YC + 1.5 * R))
    };
    liv = liv.all_neg(vec![band, sidem()], glass(WIN));
    for &(s0, s1, y0, y1) in &doors {
        let (o, i) = door_seam(Z0, (s0, s1), (y0, y1), 0.12, 0.012);
        liv = liv.all_neg(vec![o, i, sidem()], paint(SEAM_C));
    }
    // livery: teal belly + lime wave rising to the tail
    let wave = move |s: f32| YC - 0.45 * R + 0.10 * (s * 0.55).sin() + 1.05 * R * smooth(nl + 9.0, LEN - 1.0, s).powf(1.5);
    liv = liv
        .all_neg(vec![sf(move |p| p.y - wave(s_of(p)) - 0.22)], paint(TEAL))
        .all_neg(vec![sf(move |p| p.y - wave(s_of(p)) - 0.22 - 0.20 * smooth(nl + 1.0, nl + 6.0, s_of(p)))], paint(LIME));
    emit(&mut body, &t, &liv);

    // wing-root / dorsal fairing under the wing
    let wf_keys: Vec<FusKey> = (0..=10)
        .map(|i| {
            let u = i as f32 / 10.0;
            let s = (Z0 - le_root - 0.9) + (c_root + 2.6) * u;
            let e = (std::f32::consts::PI * u).sin().powf(0.5).max(0.06);
            [s, 0.95 * e, wy + 0.06 * e, fus.top(s) - 0.3, wy - 0.25, 2.5, 2.0]
        })
        .collect();
    let wfair = Fuselage::new(&wf_keys);
    let wr = wfair.rows(Z0, 0.2, 32, (0.0, 0.0, 1.0));
    emit_plain(&mut body, &grid(&wr, true, false), paint(WHITE_A));

    // sponsons (main gear housings)
    let sx_c = 1.70f32;
    let sp_keys: Vec<FusKey> = (0..=12)
        .map(|i| {
            let u = i as f32 / 12.0;
            let s = 12.55 + 4.9 * u;
            let e = if u < 0.3 { (1.0 - ((0.3 - u) / 0.3).powi(2)).sqrt() } else { 1.0 - 0.75 * ((u - 0.3) / 0.7).powf(1.6) };
            let e = e.max(0.05);
            [s, 0.90 * e, 0.0, -0.45 - 0.60 * e.powf(0.5), -0.45, 2.0, 3.0]
        })
        .collect();
    let spon = Rc::new(Fuselage::new(&sp_keys));
    let spr = spon.rows(Z0, 0.12, 40, (0.0, 0.0, 1.0));
    let mut st = grid(&spr, true, false);
    transform_tris(&mut st, Mat4::from_translation(vec3(sx_c, 0.0, 0.0)));
    let mut stt = st.clone();
    stt.extend(mirror_x(&st));
    // main gear well paint on the sponson bottom
    let mbay = (1.40f32, 2.48f32, -1.95f32, 0.62f32);
    let sliv = Livery::new(paint(TEAL)).all_neg(
        vec![sf(move |p| rrect(vec2(p.x.abs() - (mbay.0 + mbay.1) * 0.5, p.z - (mbay.2 + mbay.3) * 0.5), vec2((mbay.1 - mbay.0) * 0.5, (mbay.3 - mbay.2) * 0.5), 0.08)), sf(|p| p.y - (YC - 1.5))],
        matte(WELL),
    );
    emit(&mut body, &stt, &sliv);

    // ---------------------------------------------------------- nacelles + props
    let nk: [FusKey; 10] = [
        [0.00, 0.380, 1.280, 0.520, 0.90, 2.0, 2.0],
        [0.15, 0.420, 1.340, 0.420, 0.88, 2.0, 2.2],
        [0.60, 0.460, 1.470, 0.330, 0.86, 2.2, 2.4],
        [1.50, 0.480, 1.800, 0.320, 0.88, 2.4, 2.4],
        [2.50, 0.480, 2.050, 0.360, 0.95, 2.4, 2.4],
        [4.00, 0.440, 2.050, 0.450, 1.05, 2.3, 2.3],
        [5.00, 0.360, 2.000, 0.620, 1.15, 2.2, 2.2],
        [5.80, 0.210, 1.950, 0.950, 1.35, 2.1, 2.1],
        [6.15, 0.080, 1.900, 1.250, 1.55, 2.0, 2.0],
        [6.25, 0.020, 1.880, 1.400, 1.62, 2.0, 2.0],
    ];
    let nac = Fuselage::new(&nk);
    let nz0 = PROP.z - 0.25;
    let nr = nac.rows(nz0, 0.12, 40, (0.0, 1.0, 1.5));
    let mut ntr = grid(&nr, true, false);
    ntr.extend(fan(nr.last().unwrap(), -Vec3::Z));
    transform_tris(&mut ntr, Mat4::from_translation(vec3(PROP.x, 0.0, 0.0)));
    let mut nt2 = ntr.clone();
    nt2.extend(mirror_x(&ntr));
    let nliv = Livery::new(paint(WHITE_A))
        // intake scoop under the spinner
        .all_neg(vec![sf(|p| { let q = vec2((p.x.abs() - PROP.x) / 0.26, (p.y - 0.50) / 0.13); (q.length() - 1.0) * 0.13 }), sf(|p| p.z - (PROP.z - 0.38))], matte(DARK))
        .all_neg(vec![sf(|p| (PROP.z - 0.75) - p.z)], paint(TEAL));
    emit(&mut body, &nt2, &nliv);
    // exhaust stacks on the outboard side
    let mut ex = Vec::new();
    let a = vec3(PROP.x + 0.40, 1.20, PROP.z - 3.3);
    ex.extend(tube(a, a + vec3(0.22, 0.02, -0.45), Vec3::Y, &[(0.0, 0.16, 0.20), (1.0, 0.14, 0.17)], 2.5, 14));
    let mut ex2 = ex.clone();
    ex2.extend(mirror_x(&ex));
    emit_plain(&mut body, &ex2, metal(BURNT_C));
    let exm = annulus(a + vec3(0.22, 0.02, -0.45), vec3(0.22, 0.02, -0.45), 0.0, 0.13, 14);
    let mut exm2 = exm.clone();
    exm2.extend(mirror_x(&exm));
    emit_plain(&mut body, &exm2, matte(DARK));

    for (i, sx) in [(0u8, 1.0f32), (1, -1.0)] {
        parts.push(propeller(&PropSpec {
            engine: i,
            pivot: vec3(PROP.x * sx, PROP.y, PROP.z),
            radius: 1.95,
            blades: 6,
            chord: vec![(0.17, 0.17), (0.35, 0.25), (0.62, 0.26), (0.85, 0.21), (0.985, 0.11)],
            pitch: 3.6,
            root: 0.17,
            thick: 0.08,
            spinner: (0.25, 1.0, 0.38),
            spinner_col: paint(WHITE_A),
            blade_col: [0.03, 0.03, 0.032],
            tip_col: [0.03, 0.03, 0.032],
            tip_from: 1.5,
            scimitar: 0.45,
            disc_col: [0.02, 0.02, 0.022],
        }));
    }

    // ---------------------------------------------------------- T-tail
    let fin_pts = [(YC + R - 0.2, -7.9f32, -12.05f32), (YC + R + 0.4, -8.5, -12.15), (5.30, -10.75, -12.60)];
    let fin_keys: Vec<Key> = fin_pts.iter().map(|&(y, le, te)| key(y, vec3(0.0, y, le), le - te, 0.0, 90.0)).collect();
    let fk = fin_keys.clone();
    let (fy0, fy1) = (fin_pts[1].0 + 0.1, 5.15f32);
    let rfrac = move |y: f32| {
        let f = key_frame(&fk, y);
        let t = (y - fy0) / (fy1 - fy0);
        (f.le.z - (-11.35 - 0.30 * t)) / f.chord
    };
    let mut fin = keyed_wing(fin_keys, Foil::sym(0.13), vec![seg(fin_pts[0].0, fy0, 2), cut(fy0, fy1, 8, rfrac(fy0), rfrac(fy1)), seg(fy1, 5.30, 1)], Cap::Flat, Cap::Flat);
    fin.k1 = 14;
    let (ftr, fc) = fin.build();
    let fin_proj: Rc<dyn Fn(Vec3) -> glam::Vec2> = Rc::new(|p: Vec3| vec2(-p.z, p.y));
    let p1 = fin_proj.clone();
    let finliv = Livery::new(paint(TEAL))
        // lime wave emblem
        .all_neg(vec![sf(move |p| { let q = p1(p); (q.y - (3.7 + 0.30 * ((q.x - 9.8) * 1.6).sin() - 0.20 * (q.x - 9.8))).abs() - 0.22 })], paint(LIME));
    emit(&mut body, &ftr, &finliv);
    let rr = &fc[0];
    parts.push(hinged(PartKind::Rudder, &rr.tris, &finliv, (rr.a + rr.b) * 0.5, rr.b - rr.a, -Vec3::X, 28f32.to_radians()));
    // dorsal fin fillet
    let dors = tube(vec3(0.0, YC + R - 0.15, -4.6), vec3(0.0, YC + R - 0.15, -8.6), Vec3::X, &(0..=8).map(|i| { let t = i as f32 / 8.0; (t, 0.04 + 0.08 * t, 0.02 + 0.50 * t.powf(1.8)) }).collect::<Vec<_>>(), 2.0, 12);
    emit(&mut body, &dors, &finliv);
    // stabiliser on top of the fin
    let sy = 5.28f32;
    let stab_pts = [(0.0f32, -10.55f32, -12.55f32), (3.65, -11.35, -12.55)];
    let stab_keys: Vec<Key> = stab_pts.iter().map(|&(x, le, te)| key(x, vec3(x, sy, le), le - te, 0.0, 0.0)).collect();
    let sk = stab_keys.clone();
    let efrac = move |x: f32| {
        let f = key_frame(&sk, x);
        (f.le.z - (-11.95 + 0.0 * x)) / f.chord
    };
    let mut stab = keyed_wing(stab_keys, Foil::sym(0.11), vec![seg(0.0, 0.25, 1), cut(0.25, 3.45, 6, efrac(0.25), efrac(3.45)), seg(3.45, 3.65, 1)], Cap::Open, Cap::Round(0.03));
    stab.k1 = 12;
    let (sl, sc) = stab.build();
    let mut stt2 = sl.clone();
    stt2.extend(mirror_x(&sl));
    let stliv = Livery::new(paint(TEAL));
    emit(&mut body, &stt2, &stliv);
    let e = &sc[0];
    let mut et = e.tris.clone();
    et.extend(mirror_x(&e.tris));
    let epiv = vec3(0.0, (e.a.y + e.b.y) * 0.5, (e.a.z + e.b.z) * 0.5);
    parts.push(hinged(PartKind::Elevator, &et, &stliv, epiv, Vec3::X, Vec3::Y, 25f32.to_radians()));
    // fin / stabiliser bullet
    let bullet = tube(vec3(0.0, sy, -10.2), vec3(0.0, sy, -12.9), Vec3::X, &teardrop(12, 0.3, 0.16, 0.16), 2.0, 16);
    emit_plain(&mut body, &bullet, paint(TEAL));

    // ---------------------------------------------------------- main gear (trailing arm, folds up/aft into the sponsons)
    let (mr, mw, msp, mlr) = (0.42f32, 0.22f32, 0.56f32, 0.085f32);
    let mpiv = vec3(2.00, -0.70, 0.32);
    let maxle = vec3(2.00, -2.00 + mr, -0.90);
    let target = vec3(0.0, 9f32.to_radians().sin(), -9f32.to_radians().cos());
    let (maxis, mang) = fold(maxle - mpiv, target, Vec3::X, Vec3::X);
    for sx in [1.0f32, -1.0] {
        let gu = gear_unit(mpiv, maxle, mr, mw, msp, 2, mlr);
        let groups = gu.groups(sx, vec![]);
        let (ax, pv) = if sx > 0.0 { (maxis, mpiv) } else { (mirror_axis(maxis), mx(mpiv)) };
        parts.push(multi_part(PartKind::GearMain, &groups, pv, ax, mang));
        // sponson bottom doors: inner and outer halves, hinged on their outer edges
        let sp2 = spon.clone();
        let bottom = move |x: f32, z: f32| sp2.bottom_at(Z0 - z, x - sx_c).unwrap_or(-0.45);
        let xm = (mbay.0 + mbay.1) * 0.5;
        let hz = (mbay.2 + mbay.3) * 0.5;
        for (x0, x1, hx, ox) in [(mbay.0, xm - 0.004, mbay.0, xm), (xm + 0.004, mbay.1, mbay.1, xm)] {
            let d = hf_slab(&bottom, x0, x1, mbay.2, mbay.3, 4, 10, true, 0.006, 0.015);
            let hinge = vec3(hx, bottom(hx, hz) + 0.01, hz);
            let axis = orient_axis(Vec3::Z, hinge, vec3(ox, hinge.y, hinge.z), Vec3::Y);
            let (d, hinge, axis) = if sx > 0.0 { (d, hinge, axis) } else { (mirror_x(&d), mx(hinge), mirror_axis(axis)) };
            parts.push(part_at_one(PartKind::GearDoors, &[(d, paint(TEAL))], hinge, axis, 92f32.to_radians()));
        }
    }

    // ---------------------------------------------------------- nose gear (forward) + doors
    let (nr_, nw, nsp, nlr) = (0.29f32, 0.17f32, 0.40f32, 0.065f32);
    let npiv = vec3(0.0, -0.42, 10.68);
    let naxle = vec3(0.0, -2.00 + nr_, 10.50);
    let ndir = vec3(0.0, 11f32.to_radians().sin(), 11f32.to_radians().cos());
    let (naxis, nang) = fold(naxle - npiv, ndir, Vec3::X, Vec3::X);
    {
        let gu = gear_unit(npiv, naxle, nr_, nw, nsp, 2, nlr);
        let tl = lens(naxle + vec3(0.0, 0.50, nlr * 1.3), Vec3::Z, 0.05, 0.045);
        let lv = (naxle - npiv).normalize();
        let brace = cyl(npiv + lv * 0.12 + Vec3::Z * 0.05, npiv + lv * 0.70 + Vec3::Z * 0.04, nlr * 0.35, 8);
        let mut groups = gu.groups(1.0, vec![(tl, emissive(WHITE_LIGHT, 3.0))]);
        groups.push((brace, paint(GEAR_C)));
        parts.push(multi_part(PartKind::GearNose, &groups, npiv, naxis, nang));
    }
    let nreach = (naxle - npiv).length();
    let (bz0, bz1, bw) = (npiv.z - 0.30, npiv.z + nreach + nr_ + 0.08, 0.30f32);
    let f2 = fus.clone();
    let belly_y = move |x: f32, z: f32| f2.bottom_at(Z0 - z, x).unwrap_or(YC - R);
    for sx in [1.0f32, -1.0] {
        let (x0, x1) = if sx > 0.0 { (0.008, bw) } else { (-bw, -0.008) };
        let by = belly_y.clone();
        let d = hf_slab(&move |x, z| by(x, z), x0, x1, bz0, bz1, 3, 8, true, 0.006, 0.015);
        let hx = if sx > 0.0 { bw } else { -bw };
        let hz = (bz0 + bz1) * 0.5;
        let hinge = vec3(hx, belly_y(hx, hz) + 0.01, hz);
        let axis = orient_axis(Vec3::Z, hinge, vec3(0.0, hinge.y, hinge.z), Vec3::Y);
        parts.push(part_at_one(PartKind::GearDoors, &[(d, paint(WHITE_A))], hinge, axis, 92f32.to_radians()));
    }

    // ---------------------------------------------------------- lights
    let tip_y = wy + (x_tip - xb) * ty;
    let nav_p = vec3(x_tip + 0.03, tip_y, le_tip - c_tip * 0.35);
    emit_plain(&mut body, &lens(nav_p, Vec3::Z, 0.12, 0.035), emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&lens(nav_p, Vec3::Z, 0.12, 0.035)), emissive(NAV_GREEN, NAV_K));
    let last = rows.last().unwrap();
    let ctr = last.iter().copied().sum::<Vec3>() / last.len() as f32;
    emit_plain(&mut body, &lens(ctr + vec3(0.0, 0.0, -0.02), Vec3::Z, 0.06, 0.04), emissive(WHITE_LIGHT, NAV_K));
    let mut stb = lens(nav_p - vec3(0.0, 0.0, 0.25), Vec3::Z, 0.10, 0.03);
    stb.extend(mirror_x(&stb.clone()));
    stb.extend(lens(vec3(0.0, sy + 0.17, -12.2), Vec3::Z, 0.10, 0.03));
    parts.push(plain_part(PartKind::Strobe, &stb, emissive(STROBE_COL, STROBE_K), Vec3::ZERO, Vec3::Y, 0.0));
    let sb = 14.5f32;
    let mut bc = ellipsoid(vec3(0.0, fus.top(sb) - 0.02, Z0 - sb + 0.12), vec3(0.0, fus.top(sb) - 0.02, Z0 - sb - 0.12), Vec3::X, 0.06, 0.08, 6, 12);
    let bb = fus.bottom_at(9.0, 0.0).unwrap_or(YC - R);
    bc.extend(ellipsoid(vec3(0.0, bb + 0.02, Z0 - 9.0 + 0.12), vec3(0.0, bb + 0.02, Z0 - 9.12), Vec3::X, 0.06, 0.08, 6, 12));
    parts.push(plain_part(PartKind::Beacon, &bc, emissive(BEACON_COL, BEACON_K), Vec3::ZERO, Vec3::Y, 0.0));

    let _ = Quat::IDENTITY;
    finish(body, parts, vec3(0.55, 1.05, 11.60))
}
