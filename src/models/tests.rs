use super::*;
use crate::scene::mat;
use glam::{vec3, Vec3};

fn tris_of(mesh: &MeshData, tf: Mat4) -> Vec<[Vec3; 3]> {
    mesh.idx.chunks(3).map(|t| [0, 1, 2].map(|k| tf.transform_point3(Vec3::from(mesh.verts[t[k] as usize].pos)))).collect()
}

fn is_gear(k: PartKind) -> bool {
    matches!(k, PartKind::GearNose | PartKind::GearMain | PartKind::GearTail | PartKind::GearDoors)
}

/// All opaque triangles with gear at `gear` and other surfaces at `defl`.
fn all_tris(m: &AircraftModel, defl: f32, gear: f32) -> Vec<[Vec3; 3]> {
    let mut out = tris_of(&m.body, Mat4::IDENTITY);
    for p in &m.parts {
        if matches!(p.kind, PartKind::Afterburner) {
            continue;
        }
        let d = if is_gear(p.kind) { gear } else { defl };
        out.extend(tris_of(&p.mesh, p.transform(d)));
    }
    out
}

fn bounds(t: &[[Vec3; 3]]) -> (Vec3, Vec3) {
    let (mut lo, mut hi) = (Vec3::splat(1e9), Vec3::splat(-1e9));
    for t in t {
        for p in t {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
    }
    (lo, hi)
}

/// Ray / triangle: distance along `dir` (> 0) and geometric normal.
fn hit(o: Vec3, dir: Vec3, t: &[Vec3; 3]) -> Option<(f32, Vec3)> {
    let e1 = t[1] - t[0];
    let e2 = t[2] - t[0];
    let pv = dir.cross(e2);
    let det = e1.dot(pv);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let tv = o - t[0];
    let u = tv.dot(pv) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = tv.cross(e1);
    let v = dir.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let d = e2.dot(q) * inv;
    (d > 0.0).then(|| (d, e1.cross(e2)))
}

fn te_move(p: &Part, defl: f32) -> Vec3 {
    let mut best = (0.0, Vec3::ZERO);
    for v in &p.mesh.verts {
        let r = Vec3::from(v.pos);
        let d = (r - p.axis * r.dot(p.axis)).length();
        if d > best.0 {
            best = (d, r);
        }
    }
    p.transform(defl).transform_point3(best.1) - p.transform(0.0).transform_point3(best.1)
}

fn part(m: &AircraftModel, k: PartKind) -> &Part {
    m.parts.iter().find(|p| p.kind == k).unwrap_or_else(|| panic!("missing {k:?}"))
}

/// (id, span, length, eye, nose/tail contact, mains contact (+x side))
const TABLE: [(&str, f32, f32, [f32; 3], [f32; 3], [f32; 3]); 7] = [
    ("kestrel", 11.0, 8.3, [0.32, 0.62, 0.55], [0.0, -1.20, 1.95], [1.22, -1.20, -0.55]),
    ("swift", 15.0, 6.7, [0.0, 0.42, 1.55], [0.0, -0.90, -3.72], [0.0, -0.90, 0.10]),
    ("peregrine", 11.2, 9.55, [0.0, 0.75, -0.40], [0.0, -0.55, -5.40], [0.90, -1.75, 0.90]),
    ("lancer", 9.5, 15.0, [0.0, 0.95, 4.20], [0.0, -1.90, 4.80], [1.30, -1.90, -0.60]),
    ("aerolink", 27.0, 27.2, [0.55, 1.05, 11.60], [0.0, -2.00, 10.50], [2.00, -2.00, -0.90]),
    ("meridian190", 28.7, 36.2, [0.55, 1.15, 15.40], [0.0, -2.40, 13.00], [2.70, -2.40, -1.50]),
    ("meridian320", 35.8, 37.6, [0.55, 1.25, 15.80], [0.0, -2.60, 13.30], [3.80, -2.60, -1.40]),
];

#[test]
fn control_sign_conventions() {
    for id in IDS {
        let m = build(id);
        let e = te_move(part(&m, PartKind::Elevator), 1.0);
        assert!(e.y > 0.05, "{id} elevator +1 must raise TE: {e}");
        let ar = part(&m, PartKind::AileronRight);
        let al = part(&m, PartKind::AileronLeft);
        assert!(ar.pivot.x < 0.0 && al.pivot.x > 0.0, "{id} aileron sides");
        assert!(te_move(ar, 1.0).y > 0.03, "{id} right aileron TE up");
        assert!(te_move(al, 1.0).y < -0.03, "{id} left aileron TE down");
        let r = te_move(part(&m, PartKind::Rudder), 1.0);
        assert!(r.x < -0.05, "{id} rudder +1 TE right (-X): {r}");
        for p in &m.parts {
            assert!((p.axis.length() - 1.0).abs() < 1e-4, "{id} {:?} axis not unit", p.kind);
            match p.kind {
                PartKind::FlapLeft | PartKind::FlapRight => {
                    let d = te_move(p, 1.0);
                    assert!(d.y < -0.1, "{id} flap TE down: {d}");
                    assert_eq!(p.pivot.x > 0.0, p.kind == PartKind::FlapLeft, "{id} flap sides");
                }
                PartKind::Propeller(_) => {
                    assert_eq!(p.axis, Vec3::Z);
                    assert_eq!(p.max_angle, 1.0);
                }
                PartKind::Spoilers => {
                    // panels open away from the skin (airliners: up; fighter petals: up / down)
                    let d = te_move(p, 1.0);
                    assert!(d.length() > 0.1, "{id} spoiler moves: {d}");
                    if id.starts_with("meridian") || id == "aerolink" {
                        assert!(d.y > 0.1, "{id} spoiler rises: {d}");
                    }
                }
                PartKind::Reverser => {
                    assert!(te_move(p, 1.0).length() > 0.1, "{id} reverser moves");
                }
                PartKind::Strobe | PartKind::Beacon | PartKind::Afterburner => {
                    assert!(p.mesh.verts.iter().all(|v| v.mat == mat::EMISSIVE), "{id} {:?} emissive", p.kind);
                }
                _ => {}
            }
        }
        if id == "meridian190" || id == "meridian320" || id == "aerolink" {
            for k in [PartKind::FlapLeft, PartKind::Spoilers, PartKind::GearDoors, PartKind::Strobe, PartKind::Beacon] {
                part(&m, k);
            }
        }
    }
    let s = build("swift");
    let ab = part(&s, PartKind::Airbrakes);
    for v in &ab.mesh.verts {
        let r = Vec3::from(v.pos);
        let d = ab.transform(1.0).transform_point3(r) - ab.transform(0.0).transform_point3(r);
        assert!(d.y > 0.12 && d.y < 0.22 && d.z.abs() < 0.02, "airbrake moves up: {d}");
    }
    let l = build("lancer");
    let a = part(&l, PartKind::Afterburner);
    assert!(a.mesh.verts.iter().all(|v| v.uv[1] > 0.5 && v.uv[0] > 0.0));
    assert!((a.pivot - vec3(0.0, 0.0, -7.2)).length() < 1e-4);
    assert!(l.parts.iter().filter(|p| p.kind == PartKind::Reverser).count() == 0);
    for id in ["meridian190", "meridian320"] {
        assert!(build(id).parts.iter().filter(|p| p.kind == PartKind::Reverser).count() >= 2, "{id} reversers");
    }
}

#[test]
fn propellers_match_table() {
    let k = build("kestrel");
    assert!(k.parts.iter().any(|p| p.kind == PartKind::Propeller(0)));
    let p = build("peregrine");
    let pp = part(&p, PartKind::Propeller(0));
    assert!((pp.pivot.x).abs() < 1e-4 && (pp.pivot.y - 0.10).abs() < 1e-4, "peregrine prop {:?}", pp.pivot);
    let r = pp.mesh.verts.iter().map(|v| (v.pos[0].powi(2) + v.pos[1].powi(2)).sqrt()).fold(0.0, f32::max);
    assert!((r - 1.60).abs() < 0.03, "peregrine prop radius {r}");
    let a = build("aerolink");
    for (i, x) in [(0u8, 4.10f32), (1, -4.10)] {
        let pp = part(&a, PartKind::Propeller(i));
        assert!((pp.pivot - vec3(x, 0.90, 4.10)).length() < 1e-3, "aerolink prop {i} {:?}", pp.pivot);
        let r = pp.mesh.verts.iter().map(|v| (v.pos[0].powi(2) + v.pos[1].powi(2)).sqrt()).fold(0.0, f32::max);
        assert!((r - 1.95).abs() < 0.03, "aerolink prop radius {r}");
    }
    assert!(build("swift").parts.iter().all(|p| !matches!(p.kind, PartKind::Propeller(_))));
}

#[test]
fn winding_matches_normals() {
    for id in IDS {
        let m = build(id);
        let mut meshes = vec![&m.body];
        meshes.extend(m.parts.iter().map(|p| &p.mesh));
        for mesh in meshes {
            for t in mesh.idx.chunks(3) {
                let v = [0, 1, 2].map(|k| mesh.verts[t[k] as usize]);
                let p = v.map(|v| Vec3::from(v.pos));
                let g = (p[1] - p[0]).cross(p[2] - p[0]);
                let n: Vec3 = v.iter().map(|v| Vec3::from(v.nrm)).sum();
                assert!(g.dot(n) > 0.0, "{id}: triangle winding disagrees with normals");
                for v in &v {
                    assert!((Vec3::from(v.nrm).length() - 1.0).abs() < 1e-3);
                }
            }
        }
    }
}

/// Rays from outside: the first surface hit must be front-facing.
#[test]
fn surfaces_face_outward() {
    for id in IDS {
        let m = build(id);
        for gear in [0.0, 1.0] {
            let tris = all_tris(&m, 0.0, gear);
            let (lo, hi) = bounds(&tris);
            let (c, ext) = ((lo + hi) * 0.5, hi - lo);
            let mut seed = 12345u32;
            let mut rnd = || {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                seed as f32 / u32::MAX as f32
            };
            let (mut hits, mut back) = (0, 0);
            for _ in 0..1200 {
                let dir = loop {
                    let d = vec3(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0);
                    if d.length() > 0.2 && d.length() < 1.0 {
                        break d.normalize();
                    }
                };
                let target = c + (vec3(rnd(), rnd(), rnd()) - 0.5) * ext * vec3(1.0, 0.8, 1.0);
                let o = target - dir * (ext.length() * 2.0);
                let mut best: Option<(f32, Vec3)> = None;
                for t in &tris {
                    if let Some((d, g)) = hit(o, dir, t) {
                        if best.is_none_or(|b| d < b.0) {
                            best = Some((d, g));
                        }
                    }
                }
                if let Some((_, g)) = best {
                    hits += 1;
                    if g.dot(dir) > 0.0 {
                        back += 1;
                    }
                }
            }
            assert!(hits > 150, "{id}: too few hits {hits}");
            assert!(back * 150 <= hits, "{id} gear {gear}: {back}/{hits} rays hit back faces first");
        }
    }
}

#[test]
fn dimensions_eye_and_budget() {
    for (id, span, len, eye, _, _) in TABLE {
        let m = build(id);
        let (lo, hi) = bounds(&all_tris(&m, 0.0, 0.0));
        let ext = hi - lo;
        assert!((ext.x - span).abs() < 0.25, "{id} span {}", ext.x);
        assert!((ext.z - len).abs() < 0.30, "{id} length {}", ext.z);
        assert!((m.eye - Vec3::from(eye)).length() < 1e-4, "{id} eye");
        let tris: usize = m.body.idx.len() / 3 + m.parts.iter().map(|p| p.mesh.idx.len() / 3).sum::<usize>();
        let budget = if id.starts_with("meridian") || id == "aerolink" { 120_000 } else { 80_000 };
        assert!(tris <= budget, "{id}: {tris} triangles");
        // nav lights: red on +X, green on -X
        let mut red = Vec3::ZERO;
        let mut green = Vec3::ZERO;
        for v in &m.body.verts {
            if v.mat == mat::EMISSIVE && v.col[0] > 0.8 && v.col[1] < 0.1 {
                red = Vec3::from(v.pos);
            }
            if v.mat == mat::EMISSIVE && v.col[1] > 0.7 && v.col[0] < 0.1 {
                green = Vec3::from(v.pos);
            }
        }
        assert!(red.x > span * 0.3 && green.x < -span * 0.3, "{id} nav lights {red} {green}");
    }
    assert!(build("nonsense").parts.iter().any(|p| p.kind == PartKind::Propeller(0)));
}

/// Lowest points of a set of triangles (within 1 cm of the minimum): centroid.
fn contact(t: &[[Vec3; 3]]) -> Vec3 {
    let ymin = t.iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
    let pts: Vec<Vec3> = t.iter().flatten().copied().filter(|p| p.y < ymin + 0.01).collect();
    let c = pts.iter().copied().sum::<Vec3>() / pts.len() as f32;
    vec3(c.x, ymin, c.z)
}

#[test]
fn gear_contact_points_match_table() {
    for (id, _, _, _, nose, main) in TABLE {
        let m = build(id);
        let (nose, main) = (Vec3::from(nose), Vec3::from(main));
        let all = all_tris(&m, 0.0, 0.0);
        let (lo, _) = bounds(&all);
        assert!((lo.y - main.y).abs() < 0.005, "{id} ground contact {}", lo.y);
        // nose / tail: lowest points with |z - table z| < 1
        let near = |c: Vec3| -> Vec<[Vec3; 3]> { all.iter().copied().filter(|t| t.iter().all(|p| (p.z - c.z).abs() < 1.0 && (p.x - c.x).abs() < 1.0)).collect() };
        for c in [nose, main, vec3(-main.x, main.y, main.z)] {
            let k = contact(&near(c));
            assert!((k.y - c.y).abs() < 0.006 && (k.x - c.x).abs() < 0.08 && (k.z - c.z).abs() < 0.08, "{id}: contact {k} vs table {c}");
        }
        // retractable mains: the contact must come from the GearMain parts
        if m.parts.iter().any(|p| p.kind == PartKind::GearMain) {
            for p in m.parts.iter().filter(|p| p.kind == PartKind::GearMain) {
                let k = contact(&tris_of(&p.mesh, p.transform(0.0)));
                let want = if p.pivot.x > 0.0 { main } else { vec3(-main.x, main.y, main.z) };
                assert!((k - want).length() < 0.08, "{id}: GearMain contact {k} vs {want}");
            }
        }
    }
}

/// At gear = 1 the legs and wheels are inside the airframe envelope: a ray
/// straight down from (a sample of) their vertices hits the airframe skin or a
/// closed door. Doors at 1 lie flush (skin right above them).
#[test]
fn gear_stows_inside_and_doors_close() {
    for id in ["peregrine", "lancer", "aerolink", "meridian190", "meridian320"] {
        let m = build(id);
        let mut skin = tris_of(&m.body, Mat4::IDENTITY);
        for p in m.parts.iter().filter(|p| p.kind == PartKind::GearDoors || !is_gear(p.kind)) {
            if !matches!(p.kind, PartKind::Afterburner | PartKind::Propeller(_)) {
                skin.extend(tris_of(&p.mesh, p.transform(if p.kind == PartKind::GearDoors { 1.0 } else { 0.0 })));
            }
        }
        let gear: Vec<&Part> = m.parts.iter().filter(|p| matches!(p.kind, PartKind::GearMain | PartKind::GearNose)).collect();
        assert!(!gear.is_empty(), "{id}: no retractable gear");
        let (mut total, mut out) = (0, 0);
        for p in &gear {
            let tf = p.transform(1.0);
            let pts: Vec<Vec3> = p.mesh.verts.iter().step_by(7).map(|v| tf.transform_point3(Vec3::from(v.pos))).collect();
            let (plo, phi) = pts.iter().fold((Vec3::splat(1e9), Vec3::splat(-1e9)), |(a, b), q| (a.min(*q), b.max(*q)));
            let local: Vec<[Vec3; 3]> = skin
                .iter()
                .copied()
                .filter(|t| t.iter().any(|q| q.x > plo.x - 1.5 && q.x < phi.x + 1.5 && q.z > plo.z - 1.5 && q.z < phi.z + 1.5))
                .collect();
            for q in pts {
                total += 1;
                // hidden from below (3 cm tolerance: leg fairings that close flush with the skin)
                let below = local.iter().any(|t| hit(q + Vec3::Y * 0.03, -Vec3::Y, t).is_some());
                if !below {
                    if out < 12 {
                        eprintln!("{id} {:?} visible stowed pt {q}", p.kind);
                    }
                    out += 1;
                }
            }
        }
        assert!(out * 100 <= total * 3, "{id}: {out}/{total} stowed gear points visible from below");
        // doors closed: skin right above each door vertex (within 8 cm)
        let body = tris_of(&m.body, Mat4::IDENTITY);
        for p in m.parts.iter().filter(|p| p.kind == PartKind::GearDoors) {
            let tf = p.transform(1.0);
            let mut bad = 0;
            let n = p.mesh.verts.len();
            for v in p.mesh.verts.iter().step_by(3) {
                // outer face only (the slab's inner face sits just inside the skin)
                if tf.transform_vector3(Vec3::from(v.nrm)).y > -0.5 {
                    continue;
                }
                let q = tf.transform_point3(Vec3::from(v.pos));
                let d = body.iter().filter_map(|t| hit(q, Vec3::Y, t).map(|h| h.0)).fold(f32::MAX, f32::min);
                if d > 0.08 {
                    if bad < 3 {
                        eprintln!("{id} door pt {q} d {d}");
                    }
                    bad += 1;
                }
            }
            assert!(bad * 10 <= n / 3 + 1, "{id}: door at {} not flush when closed ({bad})", p.pivot);
            // and open (hanging) at 0
            let open = te_move(p, 1.0).length();
            assert!(open > 0.1, "{id}: door moves");
        }
    }
}

