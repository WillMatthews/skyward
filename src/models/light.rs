//! v1 light aircraft: Kestrel C1 trainer and Swift S2 sailplane.

use super::geom::*;
use super::{AircraftModel, Part, PartKind};
use glam::{vec2, vec3, Quat, Vec3};
use std::rc::Rc;


// =================================================================== Kestrel C1

/// Nose datum: z of station s = 0 (spinner base plane region); z = Z0 - s.
const K_Z0: f32 = 2.70;
const K_THRUST_Y: f32 = -0.06;

const KESTREL_FUS: [FusKey; 19] = [
    // s     w      top    bot     waist  nt   nb
    [0.27, 0.215, 0.150, -0.275, -0.06, 2.0, 2.0],
    [0.33, 0.330, 0.270, -0.390, -0.07, 2.3, 2.3],
    [0.45, 0.420, 0.360, -0.475, -0.07, 2.5, 2.4],
    [0.70, 0.490, 0.420, -0.530, -0.07, 2.7, 2.5],
    [1.00, 0.530, 0.460, -0.555, -0.06, 2.8, 2.6],
    [1.30, 0.555, 0.490, -0.565, -0.05, 2.9, 2.7],
    [1.55, 0.562, 0.540, -0.570, -0.04, 2.8, 2.8],
    [1.85, 0.566, 0.625, -0.575, -0.03, 2.6, 2.9],
    [2.15, 0.566, 0.690, -0.575, -0.02, 2.6, 3.0],
    [2.40, 0.566, 0.705, -0.575, -0.02, 2.8, 3.0],
    [3.50, 0.560, 0.700, -0.560, -0.02, 2.8, 3.0],
    [3.88, 0.540, 0.690, -0.520, 0.00, 2.6, 2.9],
    [4.40, 0.470, 0.600, -0.420, 0.04, 2.4, 2.7],
    [5.00, 0.380, 0.490, -0.300, 0.08, 2.3, 2.5],
    [5.80, 0.270, 0.390, -0.150, 0.12, 2.2, 2.3],
    [6.60, 0.180, 0.320, -0.020, 0.15, 2.1, 2.2],
    [7.30, 0.110, 0.285, 0.070, 0.175, 2.0, 2.0],
    [7.90, 0.050, 0.260, 0.130, 0.190, 2.0, 2.0],
    [8.00, 0.030, 0.245, 0.150, 0.195, 2.0, 2.0],
];

fn ks(p: Vec3) -> f32 {
    K_Z0 - p.z
}

/// Windscreen base (s) as a function of |x|.
fn k_ws_front(ax: f32) -> f32 {
    1.50 + 0.30 * (ax / 0.56).powi(2)
}

fn k_stripe_lo(s: f32) -> f32 {
    -0.22 + 0.30 * smooth(3.0, 8.0, s)
}
fn k_stripe_hi(s: f32) -> f32 {
    k_stripe_lo(s) + 0.15 - 0.05 * smooth(3.0, 8.0, s)
}

fn kestrel_fus_livery() -> Livery {
    let side: Proj = Rc::new(|p: Vec3| vec2(ks(p), p.y));
    let top: Proj = Rc::new(|p: Vec3| vec2(ks(p), p.x));
    let on_side = |s0: f32, s1: f32| masked(sf(|p| 0.25 - p.x.abs()), side.clone(), vec2(s0, -0.6), vec2(s1, 0.8));
    let door = vec![
        rpoly(&[vec2(2.34, 0.25), vec2(3.24, 0.25), vec2(3.24, 0.58), vec2(2.44, 0.58)], 0.045, side.clone()),
        on_side(2.2, 3.4),
    ];
    let rear = vec![
        rpoly(&[vec2(3.46, 0.25), vec2(4.16, 0.33), vec2(4.08, 0.52), vec2(3.46, 0.58)], 0.04, side.clone()),
        on_side(3.3, 4.4),
    ];
    let over = vec![
        rpoly(&[vec2(3.93, -0.30), vec2(4.50, -0.18), vec2(4.50, 0.18), vec2(3.93, 0.30)], 0.03, top),
        masked(sf(|p| 0.45 - p.y), side.clone(), vec2(3.7, 0.2), vec2(4.8, 0.9)),
    ];
    let windscreen = vec![
        masked(sf(|p| 0.30 - p.y), side.clone(), vec2(1.3, -0.6), vec2(2.5, 0.9)),
        masked(sf(|p| ks(p) - 2.25), side.clone(), vec2(1.3, 0.1), vec2(2.5, 0.9)),
        sf(|p| k_ws_front(p.x.abs()) - ks(p)),
    ];
    let antiglare = vec![
        sf(|p| 0.95 - ks(p)),
        sf(|p| ks(p) - k_ws_front(p.x.abs())),
        masked(sf(|p| p.x.abs() - 0.20 - 0.10 * (ks(p) - 0.95)), side.clone(), vec2(0.8, 0.0), vec2(2.0, 0.9)),
    ];
    // door seam: rounded outline, so both edges of the thin band are smooth
    let seam = [vec2(2.35, -0.31), vec2(3.25, -0.31), vec2(3.25, 0.59), vec2(2.41, 0.59)];
    let seam2 = seam;
    let (seam_r, seam_w) = (0.05, 0.007);
    Livery::new(paint(WHITE))
        .all_neg(windscreen, glass(GLASS))
        .all_neg(door, glass(GLASS))
        .all_neg(rear, glass(GLASS))
        .all_neg(over, glass(GLASS))
        .band(
            move |p| poly_sdf(vec2(ks(p), p.y), &seam) - seam_r + seam_w,
            move |p| poly_sdf(vec2(ks(p), p.y), &seam2) - seam_r - seam_w,
            vec![on_side(2.2, 3.4)],
            paint(SEAM),
        )
        // anti-glare panel ahead of the windscreen
        .all_neg(antiglare, paint(ANTIGLARE))
        // cowling / firewall seam
        .band(|p| ks(p) - 1.292, |p| ks(p) - 1.302, vec![], paint(SEAM))
        // cooling intakes on the cowl face
        .all_neg(
            vec![
                sf(|p| {
                    let q = vec2((p.x.abs() - 0.255) / 0.105, (p.y - 0.06) / 0.07);
                    (q.length() - 1.0) * 0.07
                }),
                sf(|p| ks(p) - 0.40),
            ],
            matte(DARK),
        )
        // two-tone cheat line
        .band(|p| p.y - k_stripe_lo(ks(p)), |p| p.y - k_stripe_hi(ks(p)), vec![sf(|p| 0.40 - ks(p))], paint(BLUE))
        .band(
            |p| p.y - (k_stripe_lo(ks(p)) - 0.075),
            |p| p.y - (k_stripe_lo(ks(p)) - 0.035),
            vec![sf(|p| 0.40 - ks(p))],
            paint(GOLD),
        )
}

/// Fin / rudder / dorsal livery: diagonal blue + gold bands rising aft.
fn kestrel_fin_livery() -> Livery {
    let g = |p: Vec3| p.y - 0.62 * ks(p);
    Livery::new(paint(WHITE))
        .band(move |p| g(p) + 3.70, move |p| g(p) + 3.44, vec![], paint(BLUE))
        .band(move |p| g(p) + 3.80, move |p| g(p) + 3.745, vec![], paint(GOLD))
        .band(move |p| g(p) + 3.38, move |p| g(p) + 3.335, vec![], paint(GOLD))
}

fn kestrel_wing_livery() -> Livery {
    Livery::new(paint(WHITE))
        .sdf(|p| 5.33 - p.x.abs(), paint(BLUE))
        // landing light lens in the left wing leading edge
        .sdf(|p| (1.45 - p.x).max(p.x - 1.75).max(0.39 - p.z).max(0.62 - p.y), glass(LENS))
}

pub fn kestrel() -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts = Vec::new();

    // ---------------------------------------------------------- fuselage
    let fus = Fuselage::new(&KESTREL_FUS);
    let rows = fus.rows(K_Z0, 0.12, 36, (0.9, 4.7, 1.8));
    let mut t = grid(&rows, true, false);
    t.extend(fan(&rows[0], Vec3::Z));
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    emit(&mut body, &t, &kestrel_fus_livery());

    // dorsal fin fairing
    let dorsal_keys: Vec<FusKey> = (0..=8)
        .map(|i| {
            let s = 4.9 + 2.05 * i as f32 / 8.0;
            let u = (s - 4.9) / 2.05;
            let base = fus.top(s);
            let top = base + (0.64 - fus.top(6.95)) * u.powf(1.3) + 0.002;
            [s, 0.034 * u.powf(0.4) + 0.002, top, base - 0.06, base - 0.03, 2.0, 2.0]
        })
        .collect();
    let dorsal = Fuselage::new(&dorsal_keys);
    let drows = dorsal.rows(K_Z0, 0.14, 12, (0.0, 0.0, 1.0));
    let mut t = grid(&drows, true, false);
    t.extend(fan(drows.last().unwrap(), -Vec3::Z));
    emit(&mut body, &t, &kestrel_fin_livery());

    // ---------------------------------------------------------- wing
    let dih = 1.7f32;
    let ty = dih.to_radians().tan();
    let wing_keys = vec![
        key(0.0, vec3(0.0, 0.70, 0.45), 1.63, 0.0, dih),
        key(2.5, vec3(2.5, 0.70 + 2.5 * ty, 0.45), 1.63, -0.5, dih),
        key(5.40, vec3(5.40, 0.70 + 5.4 * ty, -0.06), 1.12, -2.5, dih),
    ];
    let wing = keyed_wing(
        wing_keys,
        Foil { m: 0.02, p: 0.4, t: 0.13 },
        vec![seg(0.0, 0.62, 1), cut(0.62, 2.50, 3, 0.74, 0.74), seg(2.50, 2.72, 1), cut(2.72, 5.12, 4, 0.76, 0.76), seg(5.12, 5.40, 1)],
        Cap::Open,
        Cap::Round(0.10),
    );
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let wliv = kestrel_wing_livery();
    emit(&mut body, &wt, &wliv);
    // flaps (ctrls[0]) and ailerons (ctrls[1]); left = +X, right = mirrored.
    for (c, kinds, maxa, want) in [
        (&ctrls[0], [PartKind::FlapLeft, PartKind::FlapRight], 35f32.to_radians(), [-Vec3::Y, -Vec3::Y]),
        (&ctrls[1], [PartKind::AileronLeft, PartKind::AileronRight], 20f32.to_radians(), [-Vec3::Y, Vec3::Y]),
    ] {
        let left_pivot = (c.a + c.b) * 0.5;
        let left_axis = (c.b - c.a).normalize();
        parts.push(hinged(kinds[0], &c.tris, &wliv, left_pivot, left_axis, want[0], maxa));
        let m = |v: Vec3| vec3(-v.x, v.y, v.z);
        parts.push(hinged(kinds[1], &mirror_x(&c.tris), &wliv, m(left_pivot), m(left_axis), want[1], maxa));
    }

    // wing struts
    let mut st = Vec::new();
    for sx in [1.0f32, -1.0] {
        let a = vec3(0.40 * sx, -0.47, -0.02);
        let b = vec3(2.58 * sx, 0.735 + 2.58 * ty - 0.02, 0.02);
        let prof: Vec<_> = [(0.0, 1.0), (0.04, 1.0), (0.96, 1.0), (1.0, 1.0)].iter().map(|&(t, f)| (t, 0.052 * f, 0.017 * f)).collect();
        st.extend(tube(a, b, Vec3::Z, &prof, 2.0, 16));
        // root fairing at the wing
        st.extend(ellipsoid(b - vec3(0.0, 0.0, -0.12), b - vec3(0.0, 0.0, 0.16), Vec3::X, 0.045, 0.04, 8, 12));
    }
    emit_plain(&mut body, &st, paint(STRUT));

    // ---------------------------------------------------------- empennage
    let fin_keys = vec![
        key(0.12, vec3(0.0, 0.12, -3.79), 1.76, 0.0, 90.0),
        key(1.50, vec3(0.0, 1.50, -4.85), 0.70, 0.0, 90.0),
    ];
    let fin_h = |y: f32| {
        let lez = -3.79 + (y - 0.12) * (-4.85 + 3.79) / 1.38;
        let ch = lez + 5.55;
        (lez + 5.15) / ch
    };
    let fin = keyed_wing(
        fin_keys,
        Foil::sym(0.10),
        vec![seg(0.12, 0.30, 1), cut(0.30, 1.40, 4, fin_h(0.30), fin_h(1.40)), seg(1.40, 1.50, 1)],
        Cap::Flat,
        Cap::Round(0.06),
    );
    let (ft, fc) = fin.build();
    let fliv = kestrel_fin_livery();
    emit(&mut body, &ft, &fliv);
    let r = &fc[0];
    parts.push(hinged(PartKind::Rudder, &r.tris, &fliv, (r.a + r.b) * 0.5, r.b - r.a, -Vec3::X, 25f32.to_radians()));

    let tail_keys = vec![key(0.0, vec3(0.0, 0.17, -4.15), 1.20, 0.0, 0.0), key(1.70, vec3(1.70, 0.17, -4.46), 0.89, 0.0, 0.0)];
    let tail_h = |x: f32| {
        let lez = -4.15 + (-4.46 + 4.15) * x / 1.70;
        (lez + 4.85) / (lez + 5.35)
    };
    let tail = keyed_wing(
        tail_keys,
        Foil::sym(0.10),
        vec![seg(0.0, 0.10, 1), cut(0.10, 1.60, 3, tail_h(0.10), tail_h(1.60)), seg(1.60, 1.70, 1)],
        Cap::Open,
        Cap::Round(0.06),
    );
    let (tl, tc) = tail.build();
    let mut tt = tl.clone();
    tt.extend(mirror_x(&tl));
    let tliv = Livery::new(paint(WHITE)).sdf(|p| 1.62 - p.x.abs(), paint(BLUE));
    emit(&mut body, &tt, &tliv);
    let e = &tc[0];
    let mut et = e.tris.clone();
    et.extend(mirror_x(&e.tris));
    let epiv = vec3(0.0, (e.a.y + e.b.y) * 0.5, (e.a.z + e.b.z) * 0.5);
    parts.push(hinged(PartKind::Elevator, &et, &tliv, epiv, Vec3::X, Vec3::Y, 25f32.to_radians()));

    // ---------------------------------------------------------- landing gear
    let gear_h = 1.20;
    let (mut tyres, mut hubs, mut white, mut metal) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    // mains
    let (mr, mz, mx) = (0.20, -0.55, 1.22);
    for sx in [1.0f32, -1.0] {
        let hub = vec3(mx * sx, -gear_h + mr, mz);
        wheel(hub, mr, 0.13, &mut tyres, &mut hubs);
        // spring-steel leg
        let a = vec3(0.40 * sx, -0.47, mz + 0.02);
        let b = vec3((mx - 0.09) * sx, hub.y + 0.05, mz);
        metal.extend(tube(a, b, Vec3::Z, &[(0.0, 0.045, 0.016), (1.0, 0.032, 0.012)], 4.0, 12));
        // spat
        let front = hub + vec3(0.0, 0.05, 0.38);
        let back = hub + vec3(0.0, 0.10, -0.72);
        white.extend(tube(front, back, Vec3::X, &teardrop(12, 0.31, 0.11, 0.205), 2.5, 18));
    }
    // nose gear
    let (nr, nz) = (0.18, 1.95);
    let nhub = vec3(0.0, -gear_h + nr, nz);
    wheel(nhub, nr, 0.11, &mut tyres, &mut hubs);
    metal.extend(cyl(vec3(0.0, -0.45, nz - 0.06), vec3(0.0, nhub.y + 0.10, nz - 0.01), 0.038, 14));
    metal.extend(cyl(vec3(0.0, -0.75, nz - 0.03), vec3(0.0, nhub.y + 0.05, nz), 0.028, 12));
    white.extend(tube(nhub + vec3(0.0, 0.05, 0.27), nhub + vec3(0.0, 0.10, -0.36), Vec3::X, &teardrop(10, 0.38, 0.095, 0.19), 2.4, 16));
    emit_plain(&mut body, &tyres, matte(TYRE));
    emit_plain(&mut body, &hubs, paint(METAL));
    emit_plain(&mut body, &metal, paint(STRUT));
    let spat_liv = Livery::new(paint(WHITE)).band(
        move |p| p.y - (-gear_h + 0.20),
        move |p| p.y - (-gear_h + 0.245),
        vec![],
        paint(BLUE),
    );
    emit(&mut body, &white, &spat_liv);

    // ---------------------------------------------------------- details
    let mut dark = Vec::new();
    // exhaust stubs
    for sx in [1.0f32, -1.0] {
        let a = vec3(0.16 * sx, -0.50, K_Z0 - 1.10);
        dark.extend(cyl(a, a + vec3(0.0, -0.10, -0.06), 0.022, 10));
    }
    // VHF blade antenna on the roof behind the wing
    let ant_s = 4.45;
    let ant_c = vec3(0.0, fus.top(ant_s) + 0.08, K_Z0 - ant_s);
    let rake = Quat::from_rotation_x(0.35);
    dark.extend(obox(ant_c, [Vec3::X, rake * Vec3::Y, rake * Vec3::Z], vec3(0.006, 0.09, 0.03)));
    emit_plain(&mut body, &dark, paint(DARK));
    // nav lights (steady, body), strobes + beacon (parts)
    let tip_y = 0.70 + 5.4 * ty;
    let nav_l = lens(vec3(5.47, tip_y, -0.05), Vec3::Z, 0.13, 0.032);
    emit_plain(&mut body, &nav_l, emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&nav_l), emissive(NAV_GREEN, NAV_K));
    let tail_l = lens(vec3(0.0, 0.245 - 0.05, K_Z0 - 8.02), Vec3::Z, 0.05, 0.022);
    emit_plain(&mut body, &tail_l, emissive(WHITE_LIGHT, NAV_K));
    let strobe = lens(vec3(5.46, tip_y, -0.16), Vec3::Z, 0.06, 0.022);
    let mut st = strobe.clone();
    st.extend(mirror_x(&strobe));
    parts.push(plain_part(PartKind::Strobe, &st, emissive(STROBE_COL, STROBE_K), Vec3::ZERO, Vec3::Y, 0.0));
    let beacon = ellipsoid(vec3(0.0, 1.52, -4.86), vec3(0.0, 1.52, -5.02), Vec3::X, 0.03, 0.035, 8, 12);
    parts.push(plain_part(PartKind::Beacon, &beacon, emissive(BEACON_COL, BEACON_K), Vec3::ZERO, Vec3::Y, 0.0));

    // ---------------------------------------------------------- propeller
    parts.push(kestrel_prop());

    finish(body, parts, vec3(0.32, 0.62, 0.55))
}

fn kestrel_prop() -> Part {
    propeller(&PropSpec {
        engine: 0,
        pivot: vec3(0.0, K_THRUST_Y, 2.50),
        radius: 0.94,
        blades: 2,
        chord: vec![(0.085, 0.070), (0.27, 0.115), (0.585, 0.135), (0.85, 0.115), (0.985, 0.080)],
        pitch: 1.45,
        root: 0.085,
        thick: 0.11,
        spinner: (0.09, 0.42, 0.205),
        spinner_col: paint(BLUE),
        blade_col: PROP_DARK,
        tip_col: PROP_TIP,
        tip_from: 0.915,
        scimitar: 0.0,
        disc_col: [0.02, 0.02, 0.022],
    })
}

// =================================================================== Swift S2

const S_Z0: f32 = 2.55;

const SWIFT_FUS: [FusKey; 20] = [
    // s     w      top     bot     waist  nt   nb
    [0.00, 0.000, -0.300, -0.320, -0.310, 2.0, 2.0],
    [0.04, 0.085, -0.225, -0.395, -0.315, 2.0, 2.0],
    [0.14, 0.160, -0.140, -0.470, -0.320, 2.0, 2.0],
    [0.35, 0.235, -0.040, -0.555, -0.320, 2.0, 2.1],
    [0.60, 0.275, 0.070, -0.605, -0.310, 2.0, 2.2],
    [0.90, 0.300, 0.180, -0.640, -0.300, 2.0, 2.2],
    [1.25, 0.310, 0.255, -0.665, -0.290, 2.0, 2.3],
    [1.60, 0.308, 0.260, -0.678, -0.280, 2.0, 2.3],
    [1.95, 0.295, 0.190, -0.682, -0.270, 2.0, 2.3],
    [2.25, 0.275, 0.090, -0.678, -0.260, 2.0, 2.3],
    [2.60, 0.245, 0.060, -0.650, -0.250, 2.0, 2.3],
    [3.00, 0.200, 0.040, -0.580, -0.240, 2.0, 2.2],
    [3.40, 0.150, 0.005, -0.470, -0.225, 2.0, 2.1],
    [3.85, 0.110, -0.050, -0.390, -0.215, 2.0, 2.0],
    [4.40, 0.092, -0.105, -0.360, -0.230, 2.0, 2.0],
    [5.10, 0.080, -0.160, -0.365, -0.262, 2.0, 2.0],
    [5.80, 0.068, -0.215, -0.375, -0.295, 2.0, 2.0],
    [6.20, 0.055, -0.245, -0.370, -0.307, 2.0, 2.0],
    [6.38, 0.030, -0.270, -0.345, -0.307, 2.0, 2.0],
    [6.42, 0.010, -0.290, -0.325, -0.307, 2.0, 2.0],
];

fn ss(p: Vec3) -> f32 {
    S_Z0 - p.z
}

fn swift_canopy_sill(s: f32) -> f32 {
    -0.015 + 0.02 * s + 0.55 * 0.08 * (1.0 + ((s - 1.80) / 0.08).exp()).ln()
}

pub fn swift() -> AircraftModel {
    let mut body = MeshData::default();
    let mut parts = Vec::new();
    let gear_h = 0.90;

    // ---------------------------------------------------------- fuselage
    let fus = Fuselage::new(&SWIFT_FUS);
    let rows = fus.rows(S_Z0, 0.08, 36, (0.3, 2.4, 1.6));
    let mut t = grid(&rows, true, false);
    t.extend(fan(rows.last().unwrap(), -Vec3::Z));
    let side: Proj = Rc::new(|p: Vec3| vec2(ss(p), p.y));
    let fliv = Livery::new(paint(GELCOAT))
        .all_neg(
            vec![
                sf(|p| swift_canopy_sill(ss(p)) - p.y),
                masked(sf(|p| 0.42 - ss(p)), side.clone(), vec2(0.2, -0.2), vec2(0.7, 0.5)),
                masked(sf(|p| ss(p) - 2.3), side.clone(), vec2(2.0, -0.2), vec2(2.6, 0.5)),
            ],
            glass(GLASS),
        )
        // thin dark seal line around the canopy
        .all_neg(
            vec![
                sf(|p| swift_canopy_sill(ss(p)) - 0.012 - p.y),
                masked(sf(|p| 0.40 - ss(p)), side.clone(), vec2(0.2, -0.2), vec2(0.7, 0.5)),
                masked(sf(|p| ss(p) - 2.3), side, vec2(2.0, -0.2), vec2(2.6, 0.5)),
            ],
            paint(SEAM),
        );
    emit(&mut body, &t, &fliv);

    // ---------------------------------------------------------- wing
    let dih = 2.5f32;
    let ty = dih.to_radians().tan();
    let wy = |x: f32| -0.02 + x * ty;
    let qc = 0.075; // quarter-chord line z
    let le = |x: f32, c: f32| vec3(x, wy(x), qc + 0.25 * c);
    let mut keys = vec![
        key(0.0, le(0.0, 0.90), 0.90, 1.0, dih),
        key(3.40, le(3.40, 0.80), 0.80, 0.5, dih),
        key(7.28, le(7.28, 0.42), 0.42, -1.5, dih),
    ];
    // winglet: bend upward about a 0.15 m radius, then straight & swept
    let bc = vec3(7.28, wy(7.28) + 0.15, 0.0);
    for (i, (phi, c)) in [(20.0f32, 0.41f32), (45.0, 0.395), (65.0, 0.375), (80.0, 0.35)].iter().enumerate() {
        let a = phi.to_radians();
        let p = vec3(bc.x + 0.15 * a.sin(), bc.y - 0.15 * a.cos(), qc + 0.25 * 0.42 - 0.02 * (i as f32 + 1.0));
        keys.push(key(7.28 + 0.15 * a, p, *c, -1.5, dih + phi));
    }
    let s_w0 = keys.last().unwrap().s;
    let top_y = bc.y + 0.30;
    keys.push(key(s_w0 + 0.30, vec3(bc.x + 0.175, top_y, -0.14), 0.20, -1.0, 88.0));
    let s_end = s_w0 + 0.30;
    let wing = keyed_wing(
        keys,
        Foil { m: 0.03, p: 0.45, t: 0.145 },
        vec![seg(0.0, 3.40, 3), seg(3.40, 3.60, 1), cut(3.60, 7.00, 6, 0.78, 0.78), seg(7.00, 7.28, 1), seg(7.28, s_end, 9)],
        Cap::Open,
        Cap::Round(0.03),
    );
    let (wl, ctrls) = wing.build();
    let mut wt = wl.clone();
    wt.extend(mirror_x(&wl));
    let wliv = Livery::new(paint(GELCOAT)).sdf(|p| 7.12 - p.x.abs(), paint(RED));
    emit(&mut body, &wt, &wliv);
    let c = &ctrls[0];
    let (lp, la) = ((c.a + c.b) * 0.5, (c.b - c.a).normalize());
    let m = |v: Vec3| vec3(-v.x, v.y, v.z);
    parts.push(hinged(PartKind::AileronLeft, &c.tris, &wliv, lp, la, -Vec3::Y, 20f32.to_radians()));
    parts.push(hinged(PartKind::AileronRight, &mirror_x(&c.tris), &wliv, m(lp), m(la), Vec3::Y, 20f32.to_radians()));

    // airbrakes: one paddle per wing, upper surface at ~55 % chord
    let wing_ref = &wing;
    let skin = |x: f32| {
        let f = wing_ref.fr(x);
        f.at(wing_ref.foil.pt(0.55, true, 1.0, 1.0), 0.0)
    };
    let (xa, xb) = (2.05, 3.25);
    let (pa, pb) = (skin(xa), skin(xb));
    let mut ab = Vec::new();
    for sx in [1.0f32, -1.0] {
        let (a, b) = (vec3(pa.x * sx, pa.y, pa.z), vec3(pb.x * sx, pb.y, pb.z));
        let c = (a + b) * 0.5;
        let dir = (b - a).normalize();
        let up = (Vec3::Y - dir * Vec3::Y.dot(dir)).normalize();
        let fwd = dir.cross(up) * sx.signum();
        let half = (b - a).length() * 0.5;
        // cap plate (flush with the skin when retracted)
        ab.extend(obox(c + up * 0.003, [dir, up, fwd], vec3(half, 0.004, 0.030)));
        // blades below it
        ab.extend(obox(c - up * 0.085, [dir, up, fwd], vec3(half - 0.01, 0.085, 0.010)));
    }
    let lift = 0.17;
    let lever = 4.0;
    let ab_pivot = vec3(0.0, pa.y.max(pb.y), (pa.z + pb.z) * 0.5 + lever);
    let mut am = MeshData::default();
    emit_plain(&mut am, &ab, paint([0.70, 0.70, 0.69]));
    parts.push(make_part(PartKind::Airbrakes, am, ab_pivot, Vec3::X, (lift / lever).asin()));

    // ---------------------------------------------------------- empennage
    // fin (with ventral part carrying the tail wheel), rudder over full height
    let fin_keys = vec![
        key(-0.80, vec3(0.0, -0.80, -3.55), 0.45, 0.0, 90.0),
        key(-0.30, vec3(0.0, -0.30, -3.00), 1.05, 0.0, 90.0),
        key(0.82, vec3(0.0, 0.82, -3.50), 0.62, 0.0, 90.0),
    ];
    let fk = fin_keys.clone();
    let fin_h = move |y: f32, hz: f32| {
        let f = key_frame(&fk, y);
        (f.le.z - hz) / f.chord
    };
    let fin = keyed_wing(
        fin_keys,
        Foil::sym(0.12),
        vec![seg(-0.80, -0.74, 1), cut(-0.74, 0.70, 6, fin_h(-0.74, -3.80), fin_h(0.70, -3.88)), seg(0.70, 0.82, 1)],
        Cap::Round(0.05),
        Cap::Flat,
    );
    let (ft, fc) = fin.build();
    let tliv = Livery::new(paint(GELCOAT)).sdf(|p| 0.50 - p.y, paint(RED));
    emit(&mut body, &ft, &tliv);
    let r = &fc[0];
    parts.push(hinged(PartKind::Rudder, &r.tris, &tliv, (r.a + r.b) * 0.5, r.b - r.a, -Vec3::X, 28f32.to_radians()));

    // T-tail stabiliser + elevator (one piece across the centre)
    let sy = 0.84;
    let stab_keys = vec![key(0.0, vec3(0.0, sy, -3.42), 0.60, 0.0, 0.0), key(1.30, vec3(1.30, sy, -3.55), 0.38, 0.0, 0.0)];
    let sk = stab_keys.clone();
    let stab_h = move |x: f32| {
        let f = key_frame(&sk, x);
        (f.le.z + 3.83) / f.chord
    };
    let mut elev_seg = cut(0.0, 1.22, 4, stab_h(0.0), stab_h(1.22));
    elev_seg.seam0 = true;
    let stab = keyed_wing(stab_keys, Foil::sym(0.11), vec![elev_seg, seg(1.22, 1.30, 1)], Cap::Open, Cap::Round(0.04));
    let (sl, sc) = stab.build();
    let mut stt = sl.clone();
    stt.extend(mirror_x(&sl));
    let sliv = Livery::new(paint(GELCOAT)).sdf(|p| 1.12 - p.x.abs(), paint(RED));
    emit(&mut body, &stt, &sliv);
    let e = &sc[0];
    let mut et = e.tris.clone();
    et.extend(mirror_x(&e.tris));
    let epiv = vec3(0.0, (e.a.y + e.b.y) * 0.5, (e.a.z + e.b.z) * 0.5);
    parts.push(hinged(PartKind::Elevator, &et, &sliv, epiv, Vec3::X, Vec3::Y, 22f32.to_radians()));

    // fin/stab junction bullet
    let bullet = tube(vec3(0.0, sy - 0.01, -3.33), vec3(0.0, sy - 0.01, -4.08), Vec3::X, &teardrop(14, 0.3, 0.042, 0.042), 2.0, 16);
    emit_plain(&mut body, &bullet, paint(GELCOAT));

    // ---------------------------------------------------------- landing gear
    let (mut tyres, mut hubs) = (Vec::new(), Vec::new());
    let (mr, mz) = (0.19, 0.10);
    let mhub = vec3(0.0, -gear_h + mr, mz);
    wheel(mhub, mr, 0.12, &mut tyres, &mut hubs);
    let tr = 0.075;
    let thub = vec3(0.0, -gear_h + tr, -3.72);
    wheel(thub, tr, 0.05, &mut tyres, &mut hubs);
    emit_plain(&mut body, &tyres, matte(TYRE));
    emit_plain(&mut body, &hubs, paint(METAL));
    // main wheel doors
    let mut doors = Vec::new();
    for sx in [1.0f32, -1.0] {
        let c = vec3(0.088 * sx, -0.72, mz + 0.03);
        let prof: Vec<_> = (0..=10)
            .map(|i| {
                let t = i as f32 / 10.0;
                let f = (1.0 - (2.0 * t - 1.0).powi(8)).max(0.0).powf(0.5);
                (t, 0.075 * f, 0.005 * f.max(0.3))
            })
            .collect();
        doors.extend(tube(c + Vec3::Z * 0.24, c - Vec3::Z * 0.24, Vec3::Y, &prof, 4.0, 16));
    }
    // tail wheel fairing on the ventral fin
    doors.extend(tube(thub + vec3(0.0, 0.03, 0.16), thub + vec3(0.0, 0.05, -0.16), Vec3::X, &teardrop(12, 0.4, 0.035, 0.065), 2.2, 16));
    emit_plain(&mut body, &doors, paint(GELCOAT));
    // tow hook & pitot
    let mut dk = Vec::new();
    dk.extend(cyl(vec3(0.0, -0.33, S_Z0 - 0.02), vec3(0.0, -0.33, S_Z0 + 0.025), 0.012, 8));
    dk.extend(cyl(vec3(0.0, -0.66, 0.42), vec3(0.0, -0.70, 0.42), 0.02, 8));
    emit_plain(&mut body, &dk, paint(METAL));

    // small nav lights in the winglet roots + anti-collision strobe on the fin
    let nav_l = lens(vec3(7.30, wy(7.30) + 0.05, qc - 0.02), Vec3::Z, 0.06, 0.016);
    emit_plain(&mut body, &nav_l, emissive(NAV_RED, NAV_K));
    emit_plain(&mut body, &mirror_x(&nav_l), emissive(NAV_GREEN, NAV_K));
    let strobe = lens(vec3(0.0, 0.83, -3.335), Vec3::Z, 0.05, 0.016);
    parts.push(plain_part(PartKind::Strobe, &strobe, emissive(STROBE_COL, STROBE_K), Vec3::ZERO, Vec3::Y, 0.0));

    finish(body, parts, vec3(0.0, 0.42, 1.55))
}
