//! Meridian 190 / 320: low-wing twin-turbofan airliners sharing one
//! parametric builder (fuselage loft with cabin windows and doors, swept wing
//! with Fowler flaps on low hinges, spoilers, ailerons, winglets, underwing
//! nacelles with reversers, conventional tail, inward-retracting main gear,
//! forward-retracting nose gear with doors). Fictional liveries.

use super::airkit::*;
use super::geom::*;
use super::{AircraftModel, Part, PartKind};
use glam::{vec2, vec3, Quat, Vec2, Vec3};
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq)]
enum Style {
    /// Meridian 190 house colours: white, warm grey belly, red/orange sweep, sunset tail.
    Sunset,
    /// Meridian 320 house colours: white, navy belly, cyan sweep, navy tail with ring emblem.
    Ocean,
}

struct Jet {
    style: Style,
    z0: f32,
    body: Body,
    eye: Vec3,
    /// Wing: dihedral (deg), root LE y at x = 0, keys (x, LE z, TE z, twist), foil t.
    dih: f32,
    wy0: f32,
    wkeys: Vec<(f32, f32, f32, f32)>,
    foil_t: f32,
    x_side: f32,
    x_kink: f32,
    flap_end: f32,
    ail: (f32, f32),
    flap_hinge: f32,
    ail_hinge: f32,
    flap_max: f32,
    spoilers: Vec<(f32, f32)>,
    canoes: Vec<f32>,
    /// Winglet: bend radius, height, cant (deg from vertical, outward), tip chord, sweep (m aft).
    wl: (f32, f32, f32, f32, f32),
    /// Belly fairing: (s0, s1, half width, bottom y, waist y).
    fair: (f32, f32, f32, f32, f32),
    fan: Fan,
    /// Pylon: (nacelle top dz front, nacelle dz aft) relative to fan face.
    pylon: (f32, f32),
    fin: Vec<(f32, f32, f32)>,
    rudder_hinge: (f32, f32),
    stab_y: f32,
    stab_dih: f32,
    stab: Vec<(f32, f32, f32)>,
    elev_hinge: (f32, f32),
    /// Main gear: pivot, axle centre, wheel r, w, spacing, leg r.
    m_piv: Vec3,
    m_axle: Vec3,
    m_wheel: (f32, f32, f32, f32),
    /// Nose gear: pivot, axle, wheel r, w, spacing, leg r, bay half width.
    n_piv: Vec3,
    n_axle: Vec3,
    n_wheel: (f32, f32, f32, f32),
    n_bay_w: f32,
    /// Cabin windows: first s, pitch, count, width, height, centre y.
    win: (f32, f32, usize, f32, f32, f32),
    doors: Vec<(f32, f32, f32, f32)>,
    exits: Vec<(f32, f32, f32, f32)>,
}

const NAVY: [f32; 3] = [0.006, 0.018, 0.075];
const CYAN: [f32; 3] = [0.02, 0.32, 0.55];
const SUN_RED: [f32; 3] = [0.40, 0.020, 0.012];
const SUN_ORANGE: [f32; 3] = [0.85, 0.28, 0.02];
const WARM_GREY: [f32; 3] = [0.30, 0.29, 0.28];
const AIR_WHITE: [f32; 3] = [0.82, 0.83, 0.83];
const MASK: [f32; 3] = [0.012, 0.012, 0.014];

pub fn meridian320() -> AircraftModel {
    let yc = 0.75;
    let j = Jet {
        style: Style::Ocean,
        z0: 18.50,
        body: Body { len: 37.6, r: 1.975, r_top: 1.975, r_bot: 1.975, yc, nose_len: 5.6, tail_len: 10.2 },
        eye: vec3(0.55, 1.25, 15.80),
        dih: 5.1,
        wy0: -0.48,
        wkeys: vec![(0.0, 4.60, -3.55, 2.5), (1.97, 3.85, -3.30, 2.2), (6.30, 1.63, -3.00, 1.0), (17.0, -3.85, -5.35, -1.5)],
        foil_t: 0.125,
        x_side: 1.97,
        x_kink: 6.30,
        flap_end: 12.7,
        ail: (12.9, 16.3),
        flap_hinge: 0.70,
        ail_hinge: 0.76,
        flap_max: 40.0,
        spoilers: vec![(3.4, 5.4), (6.6, 8.1), (8.1, 9.6), (9.6, 11.1), (11.1, 12.5)],
        canoes: vec![6.55, 9.4, 12.15],
        wl: (0.55, 2.0, 12.0, 0.55, 1.35),
        fair: (10.6, 25.9, 2.40, -1.48, -0.62),
        fan: Fan {
            c: vec3(5.75, -1.60, 4.20),
            r_fan: 0.80,
            r_hi: 0.85,
            r_max: 0.96,
            r_end: 0.80,
            l_in: 0.78,
            l_cowl: 2.35,
            r_core0: 0.52,
            r_core1: 0.36,
            l_core: 1.05,
            l_plug: 0.75,
            flat: 0.82,
            blades: 24,
            pivot_doors: true,
        },
        pylon: (-0.35, -3.0),
        fin: vec![(2.30, -10.6, -16.9), (2.90, -11.2, -17.15), (9.10, -15.45, -17.35)],
        rudder_hinge: (-15.15, -16.45),
        stab_y: 1.05,
        stab_dih: 6.0,
        stab: vec![(0.0, -13.9, -18.0), (1.25, -14.3, -18.0), (6.20, -17.5, -18.95)],
        elev_hinge: (-16.85, -18.30),
        m_piv: vec3(3.55, -0.38, -1.40),
        m_axle: vec3(3.80, -2.60 + 0.575, -1.40),
        m_wheel: (0.575, 0.40, 0.92, 0.13),
        n_piv: vec3(0.0, -0.62, 13.55),
        n_axle: vec3(0.0, -2.60 + 0.38, 13.30),
        n_wheel: (0.38, 0.22, 0.50, 0.085),
        n_bay_w: 0.40,
        win: (6.45, 0.533, 44, 0.235, 0.335, yc + 0.22),
        doors: vec![(4.55, 5.40, yc - 0.62, yc + 1.25), (31.25, 32.05, yc - 0.55, yc + 1.25)],
        exits: vec![(15.95, 16.48, yc - 0.20, yc + 0.72), (16.49 + 0.0, 17.02, yc - 0.20, yc + 0.72)],
    };
    jetliner(&j)
}

pub fn meridian190() -> AircraftModel {
    let yc = 0.70;
    let j = Jet {
        style: Style::Sunset,
        z0: 17.80,
        body: Body { len: 36.2, r: 1.505, r_top: 1.62, r_bot: 1.55, yc, nose_len: 4.6, tail_len: 9.0 },
        eye: vec3(0.55, 1.15, 15.40),
        dih: 5.0,
        wy0: -0.36,
        wkeys: vec![(0.0, 4.35, -2.05, 2.5), (1.50, 3.75, -1.95, 2.2), (5.60, 1.75, -1.95, 1.0), (13.65, -2.20, -3.40, -1.5)],
        foil_t: 0.13,
        x_side: 1.50,
        x_kink: 5.60,
        flap_end: 9.9,
        ail: (10.1, 12.9),
        flap_hinge: 0.70,
        ail_hinge: 0.76,
        flap_max: 37.0,
        spoilers: vec![(2.6, 4.4), (5.8, 7.2), (7.2, 8.5), (8.5, 9.8)],
        canoes: vec![5.85, 7.9, 9.65],
        wl: (0.30, 1.45, 14.0, 0.50, 1.10),
        fair: (10.0, 23.6, 1.95, -1.02, -0.28),
        fan: Fan {
            c: vec3(5.00, -1.20, 3.80),
            r_fan: 0.66,
            r_hi: 0.70,
            r_max: 0.80,
            r_end: 0.66,
            l_in: 0.62,
            l_cowl: 2.05,
            r_core0: 0.43,
            r_core1: 0.29,
            l_core: 0.90,
            l_plug: 0.62,
            flat: 1.0,
            blades: 22,
            pivot_doors: false,
        },
        pylon: (-0.30, -2.6),
        fin: vec![(1.90, -10.5, -15.7), (2.50, -11.0, -15.95), (7.90, -15.0, -16.75)],
        rudder_hinge: (-14.55, -15.95),
        stab_y: 1.00,
        stab_dih: 7.0,
        stab: vec![(0.0, -13.6, -17.15), (1.0, -13.9, -17.15), (6.05, -16.95, -18.15)],
        elev_hinge: (-16.15, -17.55),
        m_piv: vec3(2.85, -0.30, -1.50),
        m_axle: vec3(2.70, -2.40 + 0.52, -1.50),
        m_wheel: (0.52, 0.34, 0.80, 0.115),
        n_piv: vec3(0.0, -0.40, 13.25),
        n_axle: vec3(0.0, -2.40 + 0.33, 13.00),
        n_wheel: (0.33, 0.19, 0.44, 0.075),
        n_bay_w: 0.36,
        win: (5.80, 0.508, 47, 0.24, 0.34, yc + 0.28),
        doors: vec![(4.00, 4.82, yc - 0.55, yc + 1.18), (30.70, 31.45, yc - 0.45, yc + 1.18)],
        exits: vec![(15.10, 15.62, yc - 0.12, yc + 0.80)],
    };
    jetliner(&j)
}

fn jetliner(j: &Jet) -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts: Vec<Part> = Vec::new();
    let z0 = j.z0;
    let yc = j.body.yc;
    let r = j.body.r;
    let s_of = move |p: Vec3| z0 - p.z;
    let side = side_proj(z0);
    let fus = Rc::new(Fuselage::new(&j.body.keys()));
    let style = j.style;

    // ---------------------------------------------------------- wing
    let dih = j.dih;
    let ty = dih.to_radians().tan();
    let wy0 = j.wy0;
    let mut wkeys: Vec<Key> = j.wkeys.iter().map(|&(x, le, te, tw)| key(x, vec3(x, wy0 + x * ty, le), le - te, tw, dih)).collect();
    let tip = *j.wkeys.last().unwrap();
    let (x_tip, tip_le, tip_te) = (tip.0, tip.1, tip.2);
    let tip_c = tip_le - tip_te;
    let (rb, wl_h, cant, wl_c, wl_sw) = j.wl;
    let tip_y = wy0 + x_tip * ty;
    // winglet: bend up about a radius, then straight, canted and swept
    let bc = vec3(x_tip, tip_y + rb, 0.0);
    let mut s_acc = x_tip;
    let bend: Vec<f32> = vec![15.0, 35.0, 55.0, 72.0, 90.0 - cant];
    let mut prev = vec3(x_tip, tip_y, 0.0);
    for (i, phi) in bend.iter().enumerate() {
        let a = phi.to_radians();
        let p = vec3(bc.x + rb * a.sin(), bc.y - rb * a.cos(), 0.0);
        s_acc += (p - prev).length();
        prev = p;
        let t = (i + 1) as f32 / bend.len() as f32;
        let c = tip_c + (wl_c * 1.6 - tip_c) * t * 0.6;
        let le = tip_le - 0.25 * wl_sw * t;
        wkeys.push(key(s_acc, vec3(p.x, p.y, le), c.min(tip_c), -1.5, dih + phi));
    }
    let s_wl0 = s_acc;
    let cr = cant.to_radians();
    let top = prev + vec3(wl_h * cr.sin(), wl_h * cr.cos(), 0.0);
    let s_end = s_wl0 + wl_h;
    wkeys.push(key(s_end, vec3(top.x, top.y, tip_le - wl_sw), wl_c, -1.0, 90.0 - cant));
    let wk = wkeys.clone();
    let _ = &wk;
    let wk2 = wkeys.clone();
    let line_frac = move |s: f32, za: (f32, f32), xa: (f32, f32)| -> f32 {
        let t = ((s - xa.0) / (xa.1 - xa.0)).clamp(0.0, 1.0);
        let z = za.0 + (za.1 - za.0) * t;
        let f = key_frame(&wk2, s);
        (f.le.z - z) / f.chord
    };
    let wk3 = wkeys.clone();
    let z_at = move |s: f32, frac: f32| -> f32 {
        let f = key_frame(&wk3, s);
        f.le.z - frac * f.chord
    };
    let (xs, xk, xf, (xa0, xa1)) = (j.x_side, j.x_kink, j.flap_end, j.ail);
    let f0 = xs + 0.12;
    // straight hinge lines per segment (defined by the chord fraction at the segment ends)
    let fl_in = (z_at(f0, j.flap_hinge), z_at(xk, j.flap_hinge));
    let fl_out = (z_at(xk + 0.06, j.flap_hinge), z_at(xf, j.flap_hinge + 0.02));
    let al = (z_at(xa0, j.ail_hinge), z_at(xa1, j.ail_hinge));
    let segs = vec![
        seg(0.0, f0, 3),
        cut(f0, xk, 6, line_frac(f0, fl_in, (f0, xk)), line_frac(xk, fl_in, (f0, xk))),
        seg(xk, xk + 0.06, 1),
        cut(xk + 0.06, xf, 10, line_frac(xk + 0.06, fl_out, (xk + 0.06, xf)), line_frac(xf, fl_out, (xk + 0.06, xf))),
        seg(xf, xa0, 1),
        cut(xa0, xa1, 6, line_frac(xa0, al, (xa0, xa1)), line_frac(xa1, al, (xa0, xa1))),
        seg(xa1, x_tip, 2),
        seg(x_tip, s_wl0, 6),
        seg(s_wl0, s_end, 4),
    ];
    let mut wing = keyed_wing(wkeys, Foil { m: 0.02, p: 0.4, t: j.foil_t }, segs, Cap::Open, Cap::Round(0.02));
    wing.k1 = 18;
    wing.k2 = 7;
    let wing = Rc::new(wing);
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let w1 = wing.clone();
    let x_tip_c = x_tip;
    let upper = move |p: Vec3| -> f32 {
        let x = p.x.abs().min(x_tip_c);
        let f = w1.frac(x, p.z).clamp(0.0, 1.0);
        (w1.skin(x, f, true).y + w1.skin(x, f, false).y) * 0.5 - p.y
    };
    let w2 = wing.clone();
    let slat = move |p: Vec3| -> f32 {
        let x = p.x.abs();
        if x < xs + 0.3 || x > x_tip_c - 0.2 {
            return 1.0;
        }
        let f = w2.frac(x, p.z);
        (f - 0.13).abs() - 0.004
    };
    let up2 = upper.clone();
    let accent = match style {
        Style::Ocean => NAVY,
        Style::Sunset => SUN_RED,
    };
    // main wheel stow position (needed for well paint)
    let (mr, mw, msp, mlr) = j.m_wheel;
    let mreach = (j.m_axle - j.m_piv).length();
    let v1 = vec3(-dih.to_radians().cos(), dih.to_radians().sin() * 0.6, 0.0);
    let (maxis, mang) = fold(j.m_axle - j.m_piv, v1, Vec3::X, Vec3::Y);
    let mstow = |p: Vec3| j.m_piv + Quat::from_axis_angle(maxis, mang) * (p - j.m_piv);
    let mhs = mstow(j.m_axle);
    let _ = mreach;
    let well = move |p: Vec3| -> f32 { vec2(p.x.abs() - mhs.x, p.z - mhs.z).length() - mr - 0.06 };
    let wliv = Livery::new(paint(WING_GREY))
        .all_neg(vec![sf(move |p| x_tip_c - 0.02 - p.x.abs())], paint(accent))
        .all_neg(vec![sf(well), sf(move |p| -up2(p) - 0.0)], matte(WELL))
        .all_neg(vec![sf(slat)], paint(SEAM_C))
        .all_neg(vec![sf(move |p| -upper(p))], paint(WING_GREY_LO));
    emit(&mut body, &wt, &wliv);
    // flaps (low hinge: aft + down), ailerons
    let drop = j.fan.r_max * 0.9;
    for (i, c) in ctrls.iter().enumerate() {
        let lp = (c.a + c.b) * 0.5;
        let la = (c.b - c.a).normalize();
        if i < 2 {
            let pv = lp - Vec3::Y * drop + Vec3::Z * 0.15;
            let fm = j.flap_max.to_radians();
            parts.push(hinged(PartKind::FlapLeft, &c.tris, &wliv, pv, la, -Vec3::Y, fm));
            parts.push(hinged(PartKind::FlapRight, &mirror_x(&c.tris), &wliv, mx(pv), mx(la), -Vec3::Y, fm));
        } else {
            parts.push(hinged(PartKind::AileronLeft, &c.tris, &wliv, lp, la, -Vec3::Y, 25f32.to_radians()));
            parts.push(hinged(PartKind::AileronRight, &mirror_x(&c.tris), &wliv, mx(lp), mx(la), Vec3::Y, 25f32.to_radians()));
        }
    }
    // spoiler panels ahead of the flap hinge line
    let fl_in2 = fl_in;
    let fl_out2 = fl_out;
    let w3 = wing.clone();
    let hinge_z = move |s: f32| -> f32 {
        if s <= xk {
            let t = ((s - f0) / (xk - f0)).clamp(0.0, 1.0);
            fl_in2.0 + (fl_in2.1 - fl_in2.0) * t
        } else {
            let t = ((s - xk - 0.06) / (xf - xk - 0.06)).clamp(0.0, 1.0);
            fl_out2.0 + (fl_out2.1 - fl_out2.0) * t
        }
    };
    for &(s0, s1) in &j.spoilers {
        let w4 = w3.clone();
        let xr = move |s: f32| {
            let h = w4.frac(s, hinge_z(s));
            (h - 0.15, h - 0.008)
        };
        let pan = wing.panel(s0 + 0.02, s1 - 0.02, 3, &xr, 3, true, 0.004, 0.012);
        let a = wing.skin(s0 + 0.02, xr(s0 + 0.02).0, true);
        let b = wing.skin(s1 - 0.02, xr(s1 - 0.02).0, true);
        let axis = (b - a).normalize();
        let probe = wing.skin((s0 + s1) * 0.5, xr((s0 + s1) * 0.5).1, true);
        let ax = orient_axis(axis, a, probe, Vec3::Y);
        let liv = Livery::new(paint(WING_GREY));
        let mut m = MeshData::default();
        emit(&mut m, &pan, &liv);
        let mut m2 = MeshData::default();
        emit(&mut m2, &mirror_x(&pan), &liv);
        parts.push(make_part(PartKind::Spoilers, m, a, ax, 50f32.to_radians()));
        parts.push(make_part(PartKind::Spoilers, m2, mx(a), mirror_axis(ax), 50f32.to_radians()));
    }
    // flap-track fairings (canoes)
    let mut canoes = Vec::new();
    for &x in &j.canoes {
        let f = wing.fr(x);
        let zf = f.le.z - f.chord * 0.55;
        let zb = f.le.z - f.chord * 1.18;
        let y = wing.skin(x, 0.7, false).y - 0.02;
        let a = vec3(x, y, zf);
        let b = vec3(x, y - 0.10, zb);
        let prof = teardrop(12, 0.42, 0.13, 0.30);
        let prof: Vec<(f32, f32, f32)> = prof.iter().map(|&(t, a, b)| (t, a, b * if t < 0.5 { 1.0 } else { 1.0 - (t - 0.5) * 0.6 })).collect();
        canoes.extend(tube(a, b, Vec3::X, &prof, 2.3, 16));
    }
    let mut cc = canoes.clone();
    cc.extend(mirror_x(&canoes));
    emit_plain(&mut body, &cc, paint(WING_GREY_LO));

    // ---------------------------------------------------------- fuselage
    let mut extra = Vec::new();
    let (w0, pitch, count, ww, _, _) = j.win;
    for k in 0..count {
        let s = w0 + pitch * k as f32;
        extra.push(s - ww * 0.5);
        extra.push(s);
        extra.push(s + ww * 0.5);
    }
    for &(s0, s1, _, _) in j.doors.iter().chain(j.exits.iter()) {
        extra.extend([s0, s0 + 0.03, s1 - 0.03, s1]);
    }
    let around = if r < 1.7 { 64 } else { 72 };
    let rows = fus.rows_with(z0, 0.30, around, (0.0, j.body.nose_len, 3.0), &extra);
    let mut t = grid(&rows, true, false);
    t.extend(fan(&rows[0], Vec3::Z));
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    let nl = j.body.nose_len;
    let flv = fus_livery(j, side.clone(), fus.clone());
    emit(&mut body, &t, &flv);
    // APU exhaust
    let last = rows.last().unwrap();
    let ctr = last.iter().copied().sum::<Vec3>() / last.len() as f32;
    let apu = ellipsoid(ctr + vec3(0.0, 0.0, 0.06), ctr - vec3(0.0, 0.0, 0.10), Vec3::X, 0.16, 0.16, 6, 14);
    emit_plain(&mut body, &apu, metal(BURNT_C));

    // belly fairing
    let (fs0, fs1, fw, fb, fwa) = j.fair;
    let fkeys: Vec<FusKey> = (0..=12)
        .map(|i| {
            let u = i as f32 / 12.0;
            let s = fs0 + (fs1 - fs0) * u;
            let e = (smooth(0.0, 0.32, u).sqrt() * (1.0 - smooth(0.72, 1.0, u)).powf(0.6)).max(0.08);
            let base = fus.bottom_at(s, 0.0).unwrap_or(yc - r) + 0.35;
            [s, fw * e, base.max(fwa + 0.05), fwa - (fwa - fb) * e.sqrt(), fwa, 2.0, 3.2]
        })
        .collect();
    let fair = Fuselage::new(&fkeys);
    let frows = fair.rows(z0, 0.25, 48, (0.0, 0.0, 1.0));
    let ft = grid(&frows, true, false);
    let up3 = upper_dummy();
    let _ = up3;
    let fair_liv = belly_livery(j, well);
    emit(&mut body, &ft, &fair_liv);

    // ---------------------------------------------------------- tail
    let fin_keys: Vec<Key> = j.fin.iter().map(|&(y, le, te)| key(y, vec3(0.0, y, le), le - te, 0.0, 90.0)).collect();
    let fk = fin_keys.clone();
    let (rh0, rh1) = j.rudder_hinge;
    let (fy0, fy1) = (j.fin[1].0 + 0.15, j.fin.last().unwrap().0 - 0.25);
    let rfrac = move |y: f32| {
        let f = key_frame(&fk, y);
        let t = (y - fy0) / (fy1 - fy0);
        (f.le.z - (rh0 + (rh1 - rh0) * t)) / f.chord
    };
    let mut fin = keyed_wing(
        fin_keys,
        Foil::sym(0.11),
        vec![seg(j.fin[0].0, fy0, 2), cut(fy0, fy1, 8, rfrac(fy0), rfrac(fy1)), seg(fy1, j.fin.last().unwrap().0, 2)],
        Cap::Flat,
        Cap::Round(0.03),
    );
    fin.k1 = 16;
    fin.k2 = 6;
    let (ftr, fc) = fin.build();
    let finliv = tail_livery(j);
    emit(&mut body, &ftr, &finliv);
    let rr = &fc[0];
    parts.push(hinged(PartKind::Rudder, &rr.tris, &finliv, (rr.a + rr.b) * 0.5, rr.b - rr.a, -Vec3::X, 30f32.to_radians()));
    // dorsal fillet
    let fl0 = j.fin[0];
    let dors = tube(
        vec3(0.0, fl0.0 + 0.05, fl0.1 + 2.2),
        vec3(0.0, fl0.0 + 0.05, fl0.1 - 1.0),
        Vec3::X,
        &(0..=8).map(|i| { let t = i as f32 / 8.0; (t, 0.06 + 0.10 * t, 0.02 + 0.45 * t.powf(1.5)) }).collect::<Vec<_>>(),
        2.0,
        12,
    );
    emit(&mut body, &dors, &finliv);

    let sdih = j.stab_dih;
    let sty = sdih.to_radians().tan();
    let stab_keys: Vec<Key> = j.stab.iter().map(|&(x, le, te)| key(x, vec3(x, j.stab_y + x * sty, le), le - te, 0.0, sdih)).collect();
    let sk = stab_keys.clone();
    let (eh0, eh1) = j.elev_hinge;
    let sx0 = j.stab[1].0 - 0.15;
    let sx1 = j.stab.last().unwrap().0 - 0.25;
    let efrac = move |x: f32| {
        let f = key_frame(&sk, x);
        let t = (x - sx0) / (sx1 - sx0);
        (f.le.z - (eh0 + (eh1 - eh0) * t)) / f.chord
    };
    let mut stab = keyed_wing(
        stab_keys,
        Foil::sym(0.10),
        vec![seg(0.0, sx0, 2), cut(sx0, sx1, 8, efrac(sx0), efrac(sx1)), seg(sx1, j.stab.last().unwrap().0, 2)],
        Cap::Open,
        Cap::Round(0.03),
    );
    stab.k1 = 14;
    let (stl, sc) = stab.build();
    let mut stt = stl.clone();
    stt.extend(mirror_x(&stl));
    let sliv = Livery::new(paint(WING_GREY));
    emit(&mut body, &stt, &sliv);
    let e = &sc[0];
    let mut et = e.tris.clone();
    et.extend(mirror_x(&e.tris));
    let epiv = vec3(0.0, (e.a.y + e.b.y) * 0.5, (e.a.z + e.b.z) * 0.5);
    parts.push(hinged(PartKind::Elevator, &et, &sliv, epiv, Vec3::X, Vec3::Y, 28f32.to_radians()));

    // ---------------------------------------------------------- engines + pylons
    let fan_spec = &j.fan;
    let g = fan_spec.build();
    let cav = fan_spec.cavity();
    let nac_col = match style {
        Style::Ocean => NAVY,
        Style::Sunset => AIR_WHITE,
    };
    let lip_band = match style {
        Style::Ocean => CYAN,
        Style::Sunset => SUN_RED,
    };
    let fc_ = fan_spec.c;
    let lb = fan_spec.l_in;
    let nliv = Livery::new(paint(nac_col))
        .all_neg(vec![sf(cav)], matte(WELL))
        .all_neg(vec![sf(move |p| (fc_.z + lb * 0.25) - p.z), sf(move |p| p.z - (fc_.z - 0.05))], paint(lip_band));
    let both = |t: &Vec<Tri>| {
        let mut v = t.clone();
        v.extend(mirror_x(t));
        v
    };
    emit(&mut body, &both(&g.cowl), &nliv);
    let duct_z = fc_.z + lb - 0.12;
    emit(&mut body, &both(&g.lip), &Livery::new(metal(LIP)).all_neg(vec![sf(move |p| p.z - duct_z)], metal([0.16, 0.165, 0.17])));
    emit_plain(&mut body, &both(&g.dark), matte(WELL));
    emit_plain(&mut body, &both(&g.fan), metal(FAN));
    emit_plain(&mut body, &both(&g.spinner), metal([0.08, 0.08, 0.085]));
    emit_plain(&mut body, &both(&g.core), metal(CORE));
    emit_plain(&mut body, &both(&g.burnt), metal(BURNT_C));
    parts.extend(reverser_parts(&g, paint(nac_col), true));
    // pylon
    let c = fan_spec.c;
    // pylon: from the core cowl (aft fairing behind the fan cowl) up into the wing
    let ny = c.y + fan_spec.r_core0 * 0.85;
    let wing_lo = wing.skin(c.x, 0.3, false).y;
    let (pz0, _pz1) = j.pylon;
    let te_lo = c.z - fan_spec.l_cowl - fan_spec.l_core * 0.75;
    let te_hi = wing.skin(c.x, 0.55, false).z;
    let pk = vec![
        key(ny, vec3(c.x, ny, c.z + pz0), c.z + pz0 - te_lo, 0.0, 90.0),
        key(c.y + fan_spec.r_max * 0.9, vec3(c.x, c.y + fan_spec.r_max * 0.9, c.z + pz0 - 0.15), c.z + pz0 - 0.15 - (te_lo + 0.5), 0.0, 90.0),
        key(wing_lo + 0.25, vec3(c.x, wing_lo + 0.25, c.z + pz0 - 0.9), c.z + pz0 - 0.9 - te_hi, 0.0, 90.0),
    ];
    let mut py = keyed_wing(pk, Foil::sym(0.07), vec![seg(ny, wing_lo + 0.25, 4)], Cap::Flat, Cap::Flat);
    py.k1 = 10;
    py.k2 = 5;
    let (pyl, _) = py.build();
    let pyt = pyl;
    emit(&mut body, &both(&pyt), &Livery::new(paint(WING_GREY)));

    // ---------------------------------------------------------- main gear (fold inward, wheels into the belly)
    for sx in [1.0f32, -1.0] {
        let gu = gear_unit(j.m_piv, j.m_axle, mr, mw, msp, 2, mlr);
        // leg door: flush with the wing lower skin when stowed
        let x_in = mhs.x + mr * 0.95;
        let x_out = j.m_piv.x + 0.25;
        let zc = j.m_piv.z;
        let w5 = wing.clone();
        let dxr = move |s: f32| (w5.frac(s, zc + mlr * 2.6), w5.frac(s, zc - mlr * 2.6));
        let dpan = wing.panel(x_in, x_out, 4, &dxr, 2, false, 0.006, 0.015);
        let door = rotated(&dpan, j.m_piv, maxis, -mang);
        let groups = gu.groups(sx, vec![(door, paint(WING_GREY_LO))]);
        let (ax, pv) = if sx > 0.0 { (maxis, j.m_piv) } else { (mirror_axis(maxis), mx(j.m_piv)) };
        parts.push(multi_part(PartKind::GearMain, &groups, pv, ax, mang));
    }

    // ---------------------------------------------------------- nose gear (fold forward) + doors
    let (nr, nw, nsp, nlr) = j.n_wheel;
    let ndir = vec3(0.0, 12f32.to_radians().sin(), 12f32.to_radians().cos());
    let (naxis, nang) = fold(j.n_axle - j.n_piv, ndir, Vec3::X, Vec3::X);
    {
        let gu = gear_unit(j.n_piv, j.n_axle, nr, nw, nsp, 2, nlr);
        let tl = lens(j.n_axle + vec3(0.0, 0.55, nlr * 1.3), Vec3::Z, 0.05, 0.05);
        let mut groups = gu.groups(1.0, vec![(tl, emissive(WHITE_LIGHT, 3.0))]);
        // drag brace
        let lv = (j.n_axle - j.n_piv).normalize();
        let brace = cyl(j.n_piv + lv * 0.15 + Vec3::Z * 0.06, j.n_piv + lv * 0.75 + Vec3::Z * 0.05, nlr * 0.35, 8);
        groups.push((brace, paint(GEAR_C)));
        parts.push(multi_part(PartKind::GearNose, &groups, j.n_piv, naxis, nang));
    }
    let nreach = (j.n_axle - j.n_piv).length();
    let (bz0, bz1) = (j.n_piv.z - 0.35, j.n_piv.z + nreach + nr + 0.10);
    let bw = j.n_bay_w;
    let f2 = fus.clone();
    let belly_y = move |x: f32, z: f32| f2.bottom_at(z0 - z, x).unwrap_or(yc - r);
    for sx in [1.0f32, -1.0] {
        let (x0, x1) = if sx > 0.0 { (0.008, bw) } else { (-bw, -0.008) };
        let by = belly_y.clone();
        let d = hf_slab(&move |x, z| by(x, z), x0, x1, bz0, bz1, 3, 10, true, 0.006, 0.015);
        let hx = if sx > 0.0 { bw } else { -bw };
        let hz = (bz0 + bz1) * 0.5;
        let hinge = vec3(hx, belly_y(hx, hz) + 0.01, hz);
        let axis = orient_axis(Vec3::Z, hinge, vec3(0.0, hinge.y, hinge.z), Vec3::Y);
        parts.push(part_at_one(PartKind::GearDoors, &[(d, paint(AIR_WHITE))], hinge, axis, 92f32.to_radians()));
    }

    // ---------------------------------------------------------- lights
    let nav_p = vec3(x_tip + 0.02, tip_y + 0.02, tip_te + 0.25);
    let nav = lens(nav_p, Vec3::Z, 0.12, 0.035);
    emit_plain(&mut body, &nav, emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&nav), emissive(NAV_GREEN, NAV_K));
    let tail_p = ctr + vec3(0.0, 0.22, 0.05);
    emit_plain(&mut body, &lens(tail_p, Vec3::Z, 0.06, 0.04), emissive(WHITE_LIGHT, NAV_K));
    let mut st = lens(nav_p - vec3(0.0, 0.0, 0.22), Vec3::Z, 0.10, 0.03);
    st.extend(mirror_x(&st.clone()));
    st.extend(lens(tail_p + vec3(0.0, 0.10, -0.02), Vec3::Z, 0.05, 0.035));
    parts.push(plain_part(PartKind::Strobe, &st, emissive(STROBE_COL, STROBE_K), Vec3::ZERO, Vec3::Y, 0.0));
    let sb = nl + 9.0;
    let top_y = fus.top(sb);
    let bot_y = fus.bottom_at(fs1 + 1.2, 0.0).unwrap_or(yc - r);
    let mut bc_ = ellipsoid(vec3(0.0, top_y - 0.02, z0 - sb + 0.15), vec3(0.0, top_y - 0.02, z0 - sb - 0.15), Vec3::X, 0.07, 0.09, 6, 12);
    bc_.extend(ellipsoid(vec3(0.0, bot_y + 0.02, z0 - fs1 - 1.05), vec3(0.0, bot_y + 0.02, z0 - fs1 - 1.35), Vec3::X, 0.07, 0.09, 6, 12));
    parts.push(plain_part(PartKind::Beacon, &bc_, emissive(BEACON_COL, BEACON_K), Vec3::ZERO, Vec3::Y, 0.0));

    // antennas
    let mut ant = Vec::new();
    for (s, up) in [(nl + 6.0, true), (nl + 14.0, true), (nl + 3.0, false)] {
        let y = if up { fus.top(s) } else { fus.bottom_at(s, 0.0).unwrap_or(yc - r) };
        let h = if up { 0.22 } else { -0.18 };
        let rake = Quat::from_rotation_x(if up { 0.35 } else { -0.35 });
        ant.extend(obox(vec3(0.0, y + h * 0.45, z0 - s), [Vec3::X, rake * Vec3::Y, rake * Vec3::Z], vec3(0.008, h.abs() * 0.5, 0.07)));
    }
    emit_plain(&mut body, &ant, paint(WING_GREY_LO));

    let _ = (s_of, Vec2::ZERO);
    finish(body, parts, j.eye)
}

fn upper_dummy() -> f32 {
    0.0
}

/// Fuselage livery: windows, doors, cockpit glazing, cheat lines.
fn fus_livery(j: &Jet, side: Proj, fus: Rc<Fuselage>) -> Livery {
    let z0 = j.z0;
    let yc = j.body.yc;
    let r = j.body.r;
    let nl = j.body.nose_len;
    let len = j.body.len;
    let style = j.style;
    let s_of = move |p: Vec3| z0 - p.z;
    let mut liv = Livery::new(paint(AIR_WHITE));
    // cockpit glazing
    liv = cockpit_panes(
        liv,
        z0,
        yc,
        nl,
        &[(0.215, 0.365, 4.0, 4.0, 54.0, 56.0), (0.385, 0.505, 44.0, 44.0, 66.0, 66.0), (0.520, 0.600, 47.0, 54.0, 66.0, 66.0)],
        glass(WIN),
    );
    if style == Style::Sunset {
        // dark window surround
        liv = cockpit_panes(liv, z0, yc, nl, &[(0.185, 0.64, 0.0, 34.0, 61.0, 70.0)], paint(MASK));
    }
    let rel = move |u: f32| yc + u * r;
    let _ = rel;
    // cabin windows (one periodic field), both sides
    let (w0, pitch, count, ww, wh, wy) = j.win;
    let wend = w0 + pitch * (count as f32 - 1.0);
    let wf = move |p: Vec3| window_row(s_of(p), p.y, w0, pitch, count, ww, wh, 0.07, wy);
    let band = masked(sf(wf), side.clone(), vec2(w0 - 0.4, wy - 0.4), vec2(wend + 0.4, wy + 0.4));
    let rr = r;
    let sidem = {
        let side = side.clone();
        move || masked(sf(move |p| 0.5 * rr - p.x.abs()), side.clone(), vec2(nl - 1.0, yc - 1.5 * rr), vec2(len - 2.0, yc + 1.5 * rr))
    };
    liv = liv.all_neg(vec![band, sidem()], glass(WIN));
    // doors + exits: seams, door windows
    for &(s0, s1, y0, y1) in j.doors.iter().chain(j.exits.iter()) {
        let (o, i) = door_seam(z0, (s0, s1), (y0, y1), 0.12, 0.012);
        liv = liv.all_neg(vec![o, i, sidem()], paint(SEAM_C));
    }
    for &(s0, s1, _y0, y1) in &j.doors {
        let c = vec2((s0 + s1) * 0.5, y1 - 0.42);
        let pr = side.clone();
        let f = masked(sf(move |p| rrect(pr(p) - c, vec2(0.11, 0.15), 0.08)), side.clone(), vec2(s0, y1 - 0.8), vec2(s1, y1));
        liv = liv.all_neg(vec![f, sidem()], glass(WIN));
    }
    // cheat lines
    let f1 = fus.clone();
    let _ = f1;
    match style {
        Style::Ocean => {
            // navy belly sweeping up to the tail, cyan accent above it
            let lo = move |s: f32| yc - 0.70 * r + 0.95 * r * smooth(nl + 6.0, len - 1.0, s).powf(1.6) + 0.10 * r * (1.0 - smooth(0.0, nl + 4.0, s));
            liv = liv
                .all_neg(vec![sf(move |p| p.y - lo(s_of(p)))], paint(NAVY))
                .all_neg(vec![sf(move |p| p.y - lo(s_of(p)) - 0.16 - 0.10 * smooth(nl, len, s_of(p))), sf(move |p| (nl + 1.0) - s_of(p))], paint(CYAN))
                .all_neg(vec![sf(move |p| p.y - lo(s_of(p)) - 0.16 - 0.10 * smooth(nl, len, s_of(p)))], paint(AIR_WHITE));
        }
        Style::Sunset => {
            let lo = move |s: f32| yc - 0.62 * r + 1.10 * r * smooth(nl + 12.0, len - 0.5, s).powf(1.3);
            liv = liv
                .all_neg(vec![sf(move |p| p.y - (yc - 0.55 * r))], paint(WARM_GREY))
                .all_neg(vec![sf(move |p| p.y - lo(s_of(p)) - 0.30), sf(move |p| nl + 12.0 - s_of(p))], paint(SUN_RED))
                .all_neg(vec![sf(move |p| p.y - lo(s_of(p)) - 0.55), sf(move |p| nl + 12.0 - s_of(p))], paint(SUN_ORANGE));
        }
    }
    liv
}

fn belly_livery(j: &Jet, well: impl Fn(Vec3) -> f32 + 'static) -> Livery {
    let base = match j.style {
        Style::Ocean => NAVY,
        Style::Sunset => WARM_GREY,
    };
    Livery::new(paint(base)).all_neg(vec![sf(well)], matte(WELL))
}

fn tail_livery(j: &Jet) -> Livery {
    let fin_proj: Proj = Rc::new(|p: Vec3| vec2(-p.z, p.y));
    let (y0, y1) = (j.fin[1].0, j.fin.last().unwrap().0);
    let le0 = j.fin[1].1;
    let le1 = j.fin.last().unwrap().1;
    let te = j.fin[1].2;
    let ym = y0 + (y1 - y0) * 0.52;
    let zm = -(le0 + (le1 - le0) * 0.52 + te) * 0.5 + 0.2;
    let c = vec2(zm, ym);
    match j.style {
        Style::Ocean => {
            let p1 = fin_proj.clone();
            let p2 = fin_proj.clone();
            let p3 = fin_proj.clone();
            let rr = (y1 - y0) * 0.26;
            Livery::new(paint(NAVY))
                .all_neg(vec![sf(move |p| (p1(p) - c).length() - rr * 0.58)], paint(CYAN))
                .all_neg(vec![sf(move |p| (p2(p) - c).length() - rr * 0.78)], paint(NAVY))
                .all_neg(vec![sf(move |p| (p3(p) - c).length() - rr)], paint(AIR_WHITE))
        }
        Style::Sunset => {
            let p1 = fin_proj.clone();
            let p2 = fin_proj.clone();
            let rr = (y1 - y0) * 0.30;
            let c2 = c - vec2(0.0, rr * 0.35);
            let stripe = move |p: Vec3| {
                let q = p2(p) - c2;
                let k = ((q.y / (rr * 0.18)).floor() as i32).rem_euclid(2);
                if k == 0 { -1.0 } else { 1.0 }
            };
            let _ = stripe;
            Livery::new(paint(SUN_RED))
                // setting sun: half disc split by two horizontal bars
                .all_neg(vec![sf(move |p| (p1(p).y - c2.y - rr * 0.22).abs() - rr * 0.055)], paint(SUN_RED))
                .all_neg(vec![sf(move |p| (p.y - c2.y - rr * 0.50).abs() - rr * 0.055)], paint(SUN_RED))
                .all_neg(vec![sf(move |p| (vec2(-p.z, p.y) - c2).length() - rr), sf(move |p| c2.y - p.y)], paint(SUN_ORANGE))
        }
    }
}
