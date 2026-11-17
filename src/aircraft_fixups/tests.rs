use super::*;
use std::collections::{HashMap, HashSet};

/// Parts hanging free of the airframe in a pose: the root's children
/// (each with its subtree) are "parts"; a part is attached when some of its
/// visible geometry comes within `GAP` of the static airframe or of
/// an attached part (grid cells of `GAP`, neighbours included). Returns the
/// detached parts: (node, vertex count, centroid in the body frame).
pub(crate) fn detached_parts(a: &AircraftAsset, ch: &[f32]) -> Vec<(usize, usize, Vec3)> {
    const GAP: f32 = 0.3;
    let mut p = Pose::default();
    a.pose(ch, &mut AnimState::default(), 0.0, &mut p);
    let Some(root) = a.nodes.iter().position(|n| n.parent < 0) else { return Vec::new() };
    // Top-level part of every node (root: none).
    let mut part = vec![usize::MAX; a.nodes.len()];
    for (i, n) in a.nodes.iter().enumerate() {
        if n.parent >= 0 {
            part[i] = if n.parent as usize == root { i } else { part[n.parent as usize] };
        }
    }
    let cell = |v: Vec3| ((v.x / GAP).floor() as i32, (v.y / GAP).floor() as i32, (v.z / GAP).floor() as i32);
    let mut airframe: HashSet<(i32, i32, i32)> = HashSet::new();
    let mut cells: HashMap<usize, (HashSet<(i32, i32, i32)>, usize, Vec3)> = HashMap::new();
    // (Cockpit geometry too: it is drawn in every view.)
    for c in a.chunks.iter().filter(|c| p.visible[c.node]) {
        let m = p.world[c.node];
        for t in (c.first..c.first + c.count).step_by(3) {
            let v: [Vec3; 3] = std::array::from_fn(|j| m.transform_point3(Vec3::from(a.vertices[(a.indices[(t + j as u32) as usize] + c.base_vertex) as usize].pos)));
            // The triangle's surface, sampled finer than a cell (big low-poly
            // panels carry small parts far from their corners).
            let n = (v[0].distance(v[1]).max(v[1].distance(v[2])).max(v[2].distance(v[0])) / GAP).ceil().clamp(1.0, 64.0) as usize;
            let target = if c.node == root { None } else { Some(part[c.node]) };
            for i in 0..=n {
                for j in 0..=n - i {
                    let q = v[0] + (v[1] - v[0]) * (i as f32 / n as f32) + (v[2] - v[0]) * (j as f32 / n as f32);
                    match target {
                        None => {
                            airframe.insert(cell(q));
                        }
                        Some(k) => {
                            cells.entry(k).or_default().0.insert(cell(q));
                        }
                    }
                }
            }
            if let Some(k) = target {
                let e = cells.entry(k).or_default();
                e.1 += 3;
                e.2 += v[0] + v[1] + v[2];
            }
        }
    }
    // (The C-5's whole airframe kneels: the biggest part is airframe too.)
    if let Some(big) = cells.iter().max_by_key(|(_, e)| e.1).map(|(k, _)| *k) {
        if cells[&big].1 > airframe.len() {
            let e = cells.remove(&big).unwrap();
            airframe.extend(e.0);
        }
    }
    let near = |set: &HashSet<(i32, i32, i32)>, c: &(i32, i32, i32)| (-1..=1).any(|dx| (-1..=1).any(|dy| (-1..=1).any(|dz| set.contains(&(c.0 + dx, c.1 + dy, c.2 + dz)))));
    let mut attached = airframe;
    let mut left: Vec<usize> = cells.keys().copied().collect();
    loop {
        let (now, rest): (Vec<usize>, Vec<usize>) = left.iter().partition(|k| cells[k].0.iter().any(|c| near(&attached, c)));
        if now.is_empty() {
            break;
        }
        for k in now {
            attached.extend(cells[&k].0.iter().copied());
        }
        left = rest;
    }
    let mut out: Vec<(usize, usize, Vec3)> = left.into_iter().map(|k| (k, cells[&k].1, cells[&k].2 / cells[&k].1 as f32)).collect();
    out.sort_by_key(|x| x.0);
    out
}

/// Cockpit (interior) geometry outside the airframe — drawn in every view,
/// so it hangs in the air (the F-16's view-following "mfd-flir" screen sat
/// under the intake): per node, the share of its vertices with no exterior
/// skin above or below them (0.25 m columns), gear down, everything neutral.
pub(crate) fn stray_interior(a: &AircraftAsset) -> Vec<(usize, f32, usize)> {
    const CELL: f32 = 0.25;
    let mut p = Pose::default();
    a.pose(&rest_channels(a), &mut AnimState::default(), 0.0, &mut p);
    let col = |v: Vec3| ((v.x / CELL).floor() as i32, (v.z / CELL).floor() as i32);
    let mut span: HashMap<(i32, i32), (f32, f32)> = HashMap::new();
    for c in a.chunks.iter().filter(|c| !c.interior && p.visible[c.node]) {
        let m = p.world[c.node];
        for t in (c.first..c.first + c.count).step_by(3) {
            let v: [Vec3; 3] = std::array::from_fn(|j| m.transform_point3(Vec3::from(a.vertices[(a.indices[(t + j as u32) as usize] + c.base_vertex) as usize].pos)));
            let n = (v[0].distance(v[1]).max(v[1].distance(v[2])).max(v[2].distance(v[0])) / CELL).ceil().clamp(1.0, 64.0) as usize;
            for i in 0..=n {
                for j in 0..=n - i {
                    let q = v[0] + (v[1] - v[0]) * (i as f32 / n as f32) + (v[2] - v[0]) * (j as f32 / n as f32);
                    let e = span.entry(col(q)).or_insert((f32::MAX, f32::MIN));
                    *e = (e.0.min(q.y), e.1.max(q.y));
                }
            }
        }
    }
    let mut per: HashMap<usize, (usize, usize)> = HashMap::new();
    for c in a.chunks.iter().filter(|c| c.interior && p.visible[c.node]) {
        for k in c.first..c.first + c.count {
            let v = p.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos));
            // (The column or its neighbours: thin skins between samples.)
            let (cx, cz) = col(v);
            let inside = (-1..=1).any(|dx| (-1..=1).any(|dz| span.get(&(cx + dx, cz + dz)).is_some_and(|s| v.y >= s.0 - 0.1 && v.y <= s.1 + 0.1)));
            let e = per.entry(c.node).or_default();
            e.0 += 1;
            e.1 += usize::from(!inside);
        }
    }
    let mut out: Vec<(usize, f32, usize)> = per.into_iter().filter(|(_, (n, out))| *out * 4 > *n && *n >= 30).map(|(k, (n, out))| (k, out as f32 / n as f32, n)).collect();
    out.sort_by_key(|x| x.0);
    out
}

/// Cockpit geometry outside the airframe, every aircraft:
/// `cargo test --release stray_interior_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn stray_interior_dump() {
    for id in ids() {
        let Some((_, b, _)) = load(&id) else { continue };
        for (n, f, nv) in stray_interior(&b) {
            println!("{id}: {} ({nv} verts) {:.0} % outside the airframe", b.nodes[n].name, f * 100.0);
        }
    }
}

/// Parts that come off the airframe when one channel moves (gear up):
/// `SKAC_AUDIT=lancer cargo test --release detach_sweep_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn detach_sweep_dump() {
    let id = std::env::var("SKAC_AUDIT").unwrap_or_default();
    let Some((_, b, _)) = load(&id) else { return };
    for gear in [0.0, 1.0] {
        let mut base = rest_channels(&b);
        if let Some(g) = b.channel("gear_pos") {
            base[g] = gear;
        }
        let at_rest: Vec<usize> = detached_parts(&b, &base).iter().map(|d| d.0).collect();
        for (c, name) in b.channels.iter().enumerate() {
            for v in [-1.0, 1.0, 30.0, 300.0] {
                let mut ch = base.clone();
                ch[c] = v;
                for (n, nv, cen) in detached_parts(&b, &ch).into_iter().filter(|d| !at_rest.contains(&d.0)) {
                    println!("gear {gear} {name}={v}: {} ({nv} verts) at {cen:.2?}", b.nodes[n].name);
                }
            }
        }
    }
}

/// Parts hanging free of the airframe: `SKAC_AUDIT=lancer cargo test --release detach_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn detach_dump() {
    let only = std::env::var("SKAC_AUDIT").ok();
    for id in ids().into_iter().filter(|id| only.as_ref().is_none_or(|o| o.split(',').any(|x| x == id))) {
        let Some((a, b, _)) = load(&id) else { continue };
        for (label, x) in [("as imported", &a), ("fixed", &b)] {
            for gear in [1.0, 0.0] {
                let mut ch = rest_channels(x);
                if let Some(g) = x.channel("gear_pos") {
                    ch[g] = gear;
                }
                for (n, nv, c) in detached_parts(x, &ch) {
                    println!("{id} {label} gear {gear}: {} ({nv} verts) at {c:.2?} detached", x.nodes[n].name);
                }
            }
        }
    }
}

pub(crate) fn ids() -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir("assets/aircraft")
        .map(|d| {
            d.filter_map(|e| Some(e.ok()?.file_name().to_string_lossy().into_owned()))
                .filter(|id| std::path::Path::new(&format!("assets/aircraft/{id}/model.skac")).exists())
                .collect()
        })
        .unwrap_or_default();
    v.sort();
    v
}

/// (as imported, with the fixes and their log). Textures are not loaded.
pub(crate) fn load(id: &str) -> Option<(AircraftAsset, AircraftAsset, Vec<String>)> {
    let raw = std::fs::read(format!("assets/aircraft/{id}/model.skac")).ok()?;
    let a = AircraftAsset::parse_unfixed(&raw, "__no_textures__").ok()?;
    let mut b = AircraftAsset::parse_unfixed(&raw, "__no_textures__").ok()?;
    let log = apply(&mut b);
    Some((a, b, log))
}

/// Body-frame boxes of the nodes whose name contains `SKAC_PART` (lower
/// case), gear up and gear down:
/// `SKAC_AUDIT=citationx SKAC_PART=door cargo test --release part_box_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn part_box_dump() {
    let pat = std::env::var("SKAC_PART").unwrap_or_else(|_| "door".into()).to_lowercase();
    let only = std::env::var("SKAC_AUDIT").ok();
    for id in ids().into_iter().filter(|id| only.as_ref().is_none_or(|o| o.split(',').any(|x| x == id))) {
        let Some((_, a, _)) = load(&id) else { continue };
        for gear in [0.0, 1.0] {
            let mut ch = rest_channels(&a);
            if let Some(g) = a.channel("gear_pos") {
                ch[g] = gear;
            }
            let mut p = Pose::default();
            a.pose(&ch, &mut AnimState::default(), 0.0, &mut p);
            for (i, n) in a.nodes.iter().enumerate() {
                if !n.name.to_lowercase().contains(&pat) {
                    continue;
                }
                let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
                for c in a.chunks.iter().filter(|c| c.node == i && !c.interior) {
                    for k in c.first..c.first + c.count {
                        let v = p.world[i].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos));
                        lo = lo.min(v);
                        hi = hi.max(v);
                    }
                }
                if lo.x <= hi.x {
                    println!("{id} gear_pos {gear} {:<34} vis {} lo {lo:6.2?} hi {hi:6.2?}", n.name, p.visible[i]);
                }
            }
        }
    }
}

/// Per aircraft: the fixes applied and every trailing-edge surface's motion
/// before / after: `cargo test --release fixup_audit_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn fixup_audit_dump() {
    for id in ids() {
        let Some((a, b, log)) = load(&id) else { continue };
        println!("== {id}: {}", if log.is_empty() { "no fixes".to_string() } else { log.join("; ") });
        let steer0 = steering_turns(&a);
        for (node, ch, right) in steering_turns(&b) {
            let was = steer0.iter().find(|x| x.0 == node && x.1 == ch).map(|x| format!("{:+.1}°", x.2)).unwrap_or_else(|| "  -  ".into());
            println!("   steering {:<10} {:<36} {was} -> {right:+.1}° right", b.channels[ch], b.nodes[node].name);
        }
        let before = surface_motions(&a);
        for (ch, node, m, nv) in surface_motions(&b) {
            let was = before.iter().find(|x| x.0 == ch && x.1 == node).map(|x| format!("{:+.3}", x.2)).unwrap_or_else(|| "  -   ".into());
            println!("   {ch:<10} {:<36} {was} -> {m:+.3}  ({nv} verts){}", b.nodes[node].name, if m < 0.0 { "   WRONG" } else { "" });
        }
        // Surface-channel nodes left out of the check, with their hinge arm.
        let rest = rest_channels(&b);
        let mut p0 = Pose::default();
        b.pose(&rest, &mut AnimState::default(), 0.0, &mut p0);
        for (name, sense) in SURFACE_CHANNELS {
            let Some(ch) = b.channel(name) else { continue };
            for (i, n) in b.nodes.iter().enumerate() {
                if !node_reads(n, ch as u16) || trailing_edge_motion(&b, i, ch, sense).is_some() {
                    continue;
                }
                let mut in_sub = vec![false; b.nodes.len()];
                for (k, m) in b.nodes.iter().enumerate() {
                    in_sub[k] = k == i || (m.parent >= 0 && in_sub[m.parent as usize]);
                }
                let (mut s, mut cnt) = (Vec3::ZERO, 0);
                for c in b.chunks.iter().filter(|c| !c.interior && in_sub[c.node]) {
                    for k in c.first..c.first + c.count {
                        s += p0.world[c.node].transform_point3(Vec3::from(b.vertices[(b.indices[k as usize] + c.base_vertex) as usize].pos));
                        cnt += 1;
                    }
                }
                if let (Some(Anim::Rotate { center, axis, .. }), true) = (&n.anim, cnt > 0) {
                    let h = p0.world[i].transform_point3(*center);
                    let ax = p0.world[i].transform_vector3(*axis).normalize_or_zero();
                    let arm = s / cnt as f32 - h;
                    println!("   skipped {name:<10} {:<32} centroid {:.2?} hinge {:.2?} axis {:.2?} arm⊥ {:.2?}", n.name, s / cnt as f32, h, ax, arm - ax * arm.dot(ax));
                }
            }
        }
    }
}

/// Every trailing-edge surface driven by a control channel moves the right
/// way for +1 once the fixes are applied (elevator / flaps: TE down;
/// aileron_l / aileron_r: that side down, the other up; rudder: TE right).
#[test]
fn every_surface_moves_the_right_way() {
    let mut checked = 0;
    for id in ids() {
        let Some((_, b, _)) = load(&id) else { continue };
        for (ch, node, m, nv) in surface_motions(&b) {
            checked += 1;
            // (Slivers of a few vertices — fairings, bands — prove nothing.)
            assert!(m > -0.01 || nv < 10, "{id}: {} driven by {ch} moves the wrong way ({m:+.3})", b.nodes[node].name);
        }
    }
    if !ids().is_empty() {
        assert!(checked > 100, "only {checked} surfaces checked");
    }
}

/// Body x of a node's exterior geometry and its children's (+ = left).
pub(crate) fn side_x(a: &AircraftAsset, node: usize) -> f32 {
    let mut p = Pose::default();
    a.pose(&rest_channels(a), &mut AnimState::default(), 0.0, &mut p);
    let (mut s, mut n) = (0.0, 0);
    for c in a.chunks.iter().filter(|c| !c.interior && (c.node == node || a.nodes[c.node].parent == node as i32)) {
        for k in c.first..c.first + c.count {
            s += p.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos)).x;
            n += 1;
        }
    }
    if n == 0 { 0.0 } else { s / n as f32 }
}

/// Left and right surfaces deflect alike: one side moving while the other
/// stands still was the F-16's stabilator bug. Elevators and flaps by
/// channel; ailerons as aileron_l (left) against aileron_r (right), or both
/// sides on aileron_l when the model has only that channel.
#[test]
fn paired_surfaces_move_alike() {
    for id in ids() {
        let Some((_, b, _)) = load(&id) else { continue };
        let ms = surface_motions(&b);
        // The biggest surface per side (not its tabs).
        let sum = |ch: &str, left: bool| -> Option<f32> {
            ms.iter().filter(|m| m.0 == ch && (side_x(&b, m.1) > 0.3) == left && side_x(&b, m.1).abs() > 0.3).max_by_key(|m| m.3).map(|m| m.2)
        };
        let mut pairs = vec![("elevator", "elevator"), ("flaps", "flaps")];
        pairs.push(if sum("aileron_r", false).is_some() { ("aileron_l", "aileron_r") } else { ("aileron_l", "aileron_l") });
        for (lc, rc) in pairs {
            let (l, r) = (sum(lc, true), sum(rc, false));
            if l.is_none() && r.is_none() {
                continue;
            }
            let (Some(l), Some(r)) = (l, r) else { panic!("{id}: {lc}/{rc} moves only one side (left {l:?}, right {r:?})") };
            assert!((l - r).abs() <= 0.25 * l.max(r), "{id}: {lc}/{rc} left {l:.3} vs right {r:.3}");
        }
    }
}

/// F-16: the right stabilator and both flaperons are cut out of the static
/// airframe (their AC3D triangles), the stabilators deflect together in
/// pitch (±25°, was ±57° on the left only) and differentially in roll, and
/// the flaperons roll.
#[test]
fn f16_stabilators_and_flaperons() {
    let Some((a, b, log)) = load("lancer") else { return };
    for p in STATIC_PARTS.iter().filter(|p| p.aircraft == "lancer") {
        let k = b.nodes.iter().position(|n| n.name == format!("rotate:{}", p.name)).unwrap_or_else(|| panic!("{} missing: {log:?}", p.name));
        let tris: u32 = b.chunks.iter().filter(|c| c.node == k).map(|c| c.count / 3).sum();
        // The objects' triangles; a flaperon box may also take a few
        // hinge-line fairing triangles inside it.
        assert!(tris as usize >= p.tris && tris as usize <= p.tris * 13 / 10, "{}: {tris} triangles, the objects have {}", p.name, p.tris);
    }
    // Nothing lost or duplicated.
    let total = |x: &AircraftAsset| x.chunks.iter().map(|c| c.count as usize).sum::<usize>();
    assert_eq!(total(&a), total(&b));
    let node = |n: &str| b.nodes.iter().position(|x| x.name == n).unwrap();
    let (stab_l, stab_r) = (node("rotate:LeftUpperHorizonTail"), node("rotate:RightHorizonTail"));
    let (fl_l, fl_r) = (node("rotate:LeftFlaperon"), node("rotate:RightFlaperon"));
    let ch = |n: &str| b.channel(n).unwrap();
    let pitch_l = trailing_edge_motion(&b, stab_l, ch("elevator"), Sense::Down).unwrap().0;
    let pitch_r = trailing_edge_motion(&b, stab_r, ch("elevator"), Sense::Down).unwrap().0;
    assert!(pitch_l > 0.3 && (pitch_l - pitch_r).abs() < 0.1 * pitch_l, "stabilators in pitch: left {pitch_l} right {pitch_r}");
    assert!(pitch_l < 0.7, "left stabilator over-travels: {pitch_l}");
    for (n, c) in [(fl_l, "aileron_l"), (fl_r, "aileron_r"), (stab_l, "aileron_l"), (stab_r, "aileron_r")] {
        let roll = trailing_edge_motion(&b, n, ch(c), Sense::Roll(c == "aileron_l")).unwrap().0;
        assert!(roll > 0.05, "{}: roll {roll}", b.nodes[n].name);
    }
}

/// The forward nose-gear doors of the Citation X and CRJ700 stand open only
/// while the gear travels.
#[test]
fn nose_doors_close_with_the_gear_down() {
    for (id, name) in CLOSE_WHEN_DOWN {
        let Some((_, b, _)) = load(id) else { continue };
        let k = b.nodes.iter().position(|n| n.name == *name).unwrap();
        let Some(Anim::Rotate { deg, .. }) = &b.nodes[k].anim else { panic!("{name}") };
        let mut ch = rest_channels(&b);
        let g = b.channel("gear_pos").unwrap();
        for (pos, open) in [(0.0, false), (0.6, true), (1.0, false)] {
            ch[g] = pos;
            assert_eq!(deg.eval(&ch).abs() > 30.0, open, "{id} {name} at gear {pos}: {}°", deg.eval(&ch));
        }
    }
}

/// A channel negated by the sense check reads negated wherever it is used.
#[test]
fn negated_channel_reads_negated() {
    let Some((a, b, log)) = load("crj700") else { return };
    assert!(log.iter().any(|l| l.starts_with("rudder")), "{log:?}");
    let k = b.nodes.iter().position(|n| n.name == "rotate:Rudder").unwrap();
    let (Some(Anim::Rotate { deg: d0, .. }), Some(Anim::Rotate { deg: d1, .. })) = (&a.nodes[k].anim, &b.nodes[k].anim) else { panic!() };
    let mut ch = rest_channels(&b);
    ch[b.channel("rudder").unwrap()] = 0.7;
    assert!((d0.eval(&ch) + d1.eval(&ch)).abs() < 1e-4 && d0.eval(&ch).abs() > 1.0);
}

/// Parts cut out of the static geometry take their AC3D objects'
/// triangles (and at most a few hinge-line ones more); nothing is lost or
/// duplicated.
#[test]
fn static_parts_take_their_objects() {
    let mut done: Vec<&str> = Vec::new();
    for p in STATIC_PARTS {
        let Some((a, b, log)) = load(p.aircraft) else { continue };
        let k = b.nodes.iter().position(|n| n.name.split_once(':').is_some_and(|(_, x)| x == p.name)).unwrap_or_else(|| panic!("{} {} missing: {log:?}", p.aircraft, p.name));
        let tris: usize = b.chunks.iter().filter(|c| c.node == k).map(|c| c.count as usize / 3).sum();
        assert!(tris * 10 >= p.tris * 9 && tris * 10 <= p.tris * 14, "{} {}: {tris} triangles, the objects have {}", p.aircraft, p.name, p.tris);
        if !done.contains(&p.aircraft) {
            let total = |x: &AircraftAsset| x.chunks.iter().map(|c| c.count as usize).sum::<usize>();
            assert_eq!(total(&a), total(&b), "{}", p.aircraft);
            done.push(p.aircraft);
        }
    }
}

/// Mean displacement of a node's leading and trailing fifth (body z) for
/// channel `ch` = 1, gear down.
pub(crate) fn edge_motion(a: &AircraftAsset, node: usize, ch: usize) -> (f32, f32) {
    let mut in_sub = vec![false; a.nodes.len()];
    for (i, n) in a.nodes.iter().enumerate() {
        in_sub[i] = i == node || (n.parent >= 0 && in_sub[n.parent as usize]);
    }
    let rest = rest_channels(a);
    let mut moved = rest.clone();
    moved[ch] = 1.0;
    let (mut p0, mut p1) = (Pose::default(), Pose::default());
    a.pose(&rest, &mut AnimState::default(), 0.0, &mut p0);
    a.pose(&moved, &mut AnimState::default(), 0.0, &mut p1);
    let mut pts = Vec::new();
    for c in a.chunks.iter().filter(|c| !c.interior && in_sub[c.node]) {
        for k in c.first..c.first + c.count {
            let v = Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos);
            pts.push((p0.world[c.node].transform_point3(v), p1.world[c.node].transform_point3(v)));
        }
    }
    // The middle third of the span (along the hinge): clear of horn balances.
    let Some(Anim::Rotate { axis, .. }) = &a.nodes[node].anim else { return (0.0, 0.0) };
    let ax = p0.world[node].transform_vector3(*axis).normalize_or_zero();
    let (smin, smax) = pts.iter().fold((f32::MAX, f32::MIN), |(l, h), (p, _)| (l.min(p.dot(ax)), h.max(p.dot(ax))));
    pts.retain(|(p, _)| (p.dot(ax) - (smin + smax) / 2.0).abs() < (smax - smin) / 6.0);
    let (zmin, zmax) = pts.iter().fold((f32::MAX, f32::MIN), |(l, h), (p, _)| (l.min(p.z), h.max(p.z)));
    let band = |f: &dyn Fn(f32) -> bool| {
        let d: Vec<f32> = pts.iter().filter(|(p, _)| f(p.z)).map(|(p, q)| p.distance(*q)).collect();
        d.iter().sum::<f32>() / d.len().max(1) as f32
    };
    let cut = 0.05 * (zmax - zmin);
    (band(&|z| z >= zmax - cut), band(&|z| z <= zmin + cut))
}

/// The C-130 elevator and the Cub rudder turn about their leading edge: the
/// leading edge (away from the horn balances) stays where it is.
#[test]
fn rehinged_surfaces_turn_about_their_leading_edge() {
    for (id, name, _) in REHINGE {
        let Some((a, b, _)) = load(id) else { continue };
        let k = b.nodes.iter().position(|n| n.name == *name).unwrap();
        let ch = ["elevator", "rudder"].into_iter().filter_map(|c| b.channel(c)).find(|&c| node_reads(&b.nodes[k], c as u16)).unwrap();
        let (fore, aft) = edge_motion(&b, k, ch);
        let (fore0, _) = edge_motion(&a, k, ch);
        assert!(fore < 0.1 * aft && fore <= fore0, "{id} {name}: leading edge moves {fore:.3} m (was {fore0:.3}), trailing edge {aft:.3} m");
    }
}

/// Nose-wheel steering turns the wheel right for right rudder (the Citation
/// X's turned left), on every aircraft whose model steers its nose wheel.
#[test]
fn nose_wheels_steer_right() {
    // (The ASK 21's nose wheel turns with the rudder in its model.)
    const STEERS: [&str; 9] = ["aerolink", "citationx", "concorde", "crj700", "extra500", "galaxy", "hercules", "swift", "vulcan"];
    for id in ids() {
        let Some((_, b, _)) = load(&id) else { continue };
        let turns = steering_turns(&b);
        for (n, ch, right) in &turns {
            assert!(*right > 0.0, "{id}: {} turns {right:.0}° right for +1 on {}", b.nodes[*n].name, b.channels[*ch]);
        }
        assert!(turns.is_empty() != STEERS.contains(&id.as_str()), "{id}: steering nodes {turns:?}");
    }
}

/// Surfaces the models drove from the cockpit controls read the surface
/// channels, so that they follow the flight model (and `SKYWARD_SURFACES`).
#[test]
fn cockpit_driven_surfaces_follow_the_surface_channels() {
    for (id, name, surf) in [
        ("citationx", "rotate:LHelevator", "elevator"),
        ("citationx", "rotate:Rudder", "rudder"),
        ("crj700", "rotate:Left elevator", "elevator"),
        ("vulcan", "rotate:Rudder", "rudder"),
        ("vulcan", "rotate:OuterElevonLeft", "aileron_l"),
    ] {
        let Some((_, b, _)) = load(id) else { continue };
        let ch = b.channel(surf).unwrap_or_else(|| panic!("{id}: no {surf}"));
        assert!(b.nodes.iter().any(|n| n.name == name && node_reads(n, ch as u16)), "{id} {name} does not read {surf}");
        let ctl = b.channel(&format!("ctl_{}", if surf == "aileron_l" { "aileron" } else { surf }));
        assert!(ctl.is_none_or(|c| !b.nodes.iter().any(|n| n.name == name && node_reads(n, c as u16))), "{id} {name} still reads the cockpit control");
    }
}

/// Propellers, fans and rotors spin about the thrust (or mast) axis and stay
/// on their hubs: a hub, disc or whole propeller has its centre on the spin
/// axis (single blades — the Spitfire's, the EC130's — reach out from it).
/// Every propeller aircraft's propeller spins.
#[test]
fn propellers_spin_on_their_hubs() {
    /// (aircraft, why its propeller does not spin).
    const STILL: [(&str, &str); 1] = [("aerolink", "the propeller blades are not in the shipped model (their select read an unmapped livery property): needs a re-import")];
    const PROPS: [&str; 10] = ["aerolink", "cub", "dc3", "extra500", "hercules", "kestrel", "otter", "peregrine", "zlin", "ec130"];
    let wheel = |n: &str| ["wheel", "tire", "tyre", "roue", "gear"].iter().any(|w| n.to_lowercase().contains(w));
    for id in ids() {
        let Some((_, b, _)) = load(&id) else { continue };
        let mut spins = 0;
        for (i, n) in b.nodes.iter().enumerate().filter(|(_, n)| !wheel(&n.name)) {
            let Some((ax, off, r, _)) = spin_geometry(&b, i) else { continue };
            spins += 1;
            let rotor = id == "ec130";
            let along = if rotor && !n.name.to_lowercase().contains("tail") { ax.y.abs() } else { ax.z.abs().max(if rotor { ax.x.abs() } else { 0.0 }) };
            assert!(along > 0.98, "{id} {}: spin axis {ax:?}", n.name);
            let single = n.name.contains("blade") || n.name.trim_end_matches(char::is_numeric) != n.name || rotor;
            assert!(single || off < 0.1 * r.max(0.1), "{id} {}: centre {off:.3} m off the spin axis (radius {r:.2} m)", n.name);
        }
        if PROPS.contains(&id.as_str()) && !STILL.iter().any(|s| s.0 == id) {
            assert!(spins > 0, "{id}: no propeller spins");
        }
    }
}

/// Which nodes have exterior triangles inside a body-frame box (gear down):
/// `SKAC_AUDIT=citationx SKAC_BOX="-0.5,-1.3,7.7,0.5,-0.7,9.6" cargo test --release node_box_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn node_box_dump() {
    let b: Vec<f32> = std::env::var("SKAC_BOX").unwrap_or_default().split(',').filter_map(|x| x.trim().parse().ok()).collect();
    if b.len() != 6 {
        return;
    }
    let (lo, hi) = (Vec3::new(b[0], b[1], b[2]), Vec3::new(b[3], b[4], b[5]));
    let id = std::env::var("SKAC_AUDIT").unwrap_or_default();
    let Some((_, a, _)) = load(&id) else { return };
    let mut p = Pose::default();
    a.pose(&rest_channels(&a), &mut AnimState::default(), 0.0, &mut p);
    let all = std::env::var_os("SKAC_ALL").is_some();
    for (ci, c) in a.chunks.iter().enumerate().filter(|(_, c)| all || (!c.interior && p.visible[c.node])) {
        let mut n = 0;
        for t in (c.first..c.first + c.count).step_by(3) {
            let cen = (0..3).map(|j| p.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[(t + j) as usize] + c.base_vertex) as usize].pos))).sum::<Vec3>() / 3.0;
            if cen.cmpge(lo).all() && cen.cmple(hi).all() {
                n += 1;
            }
        }
        if n > 0 {
            println!(
                "chunk {ci} node {} '{}' mat {} two_sided {} interior {} visible {}: {n} triangles",
                c.node, a.nodes[c.node].name, c.material, a.materials[c.material].two_sided, c.interior, p.visible[c.node]
            );
        }
    }
}

/// Rotating nodes whose name contains `SKAC_PART`: their hinge (centre /
/// axis, in the node's own frame: FlightGear model metres for parts of the
/// main model) and the geometry of their subtree in that frame at rest: box,
/// centroid, and the leading (forward, low x) 5 %:
/// `SKAC_AUDIT=hercules SKAC_PART=profondeur cargo test --release hinge_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn hinge_dump() {
    let pat = std::env::var("SKAC_PART").unwrap_or_default().to_lowercase();
    let id = std::env::var("SKAC_AUDIT").unwrap_or_default();
    let Some((_, a, _)) = load(&id) else { return };
    let mut p = Pose::default();
    a.pose(&rest_channels(&a), &mut AnimState::default(), 0.0, &mut p);
    for (i, n) in a.nodes.iter().enumerate() {
        if !n.name.to_lowercase().contains(&pat) {
            continue;
        }
        let Some(Anim::Rotate { center, axis, deg } | Anim::Spin { center, axis, rpm: deg }) = &n.anim else { continue };
        let frame = (if n.parent >= 0 { p.world[n.parent as usize] } else { Mat4::IDENTITY }) * n.pre;
        let inv = frame.inverse();
        let mut in_sub = vec![false; a.nodes.len()];
        for (k, m) in a.nodes.iter().enumerate() {
            in_sub[k] = k == i || (m.parent >= 0 && in_sub[m.parent as usize]);
        }
        let mut pts = Vec::new();
        for c in a.chunks.iter().filter(|c| !c.interior && in_sub[c.node]) {
            for k in c.first..c.first + c.count {
                pts.push(inv.transform_point3(p.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos))));
            }
        }
        if pts.is_empty() {
            println!("{i} {}: no exterior geometry", n.name);
            continue;
        }
        let (lo, hi) = pts.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(l, h), v| (l.min(*v), h.max(*v)));
        let cen = pts.iter().copied().sum::<Vec3>() / pts.len() as f32;
        let le: Vec<Vec3> = pts.iter().copied().filter(|v| v.x < lo.x + 0.05 * (hi.x - lo.x)).collect();
        let lec = le.iter().copied().sum::<Vec3>() / le.len().max(1) as f32;
        let mut ids = Vec::new();
        deg.channel_ids(&mut ids);
        let chs: Vec<&str> = ids.iter().map(|&c| a.channels[c as usize].as_str()).collect();
        println!(
            "{i} {} reads {chs:?}\n   centre {center:.3?} axis {axis:.3?}\n   box {lo:.3?} .. {hi:.3?} centroid {cen:.3?}\n   leading 5% centroid {lec:.3?} ({} pts)",
            n.name,
            le.len()
        );
        // Leading edge (most forward point and the thickness there) at
        // stations along the longer of span (y) and height (z).
        let k = if hi.y - lo.y > hi.z - lo.z { 1 } else { 2 };
        for s in 0..13 {
            let at = lo[k] + (hi[k] - lo[k]) * (0.03 + 0.94 * s as f32 / 12.0);
            let band: Vec<Vec3> = pts.iter().copied().filter(|v| (v[k] - at).abs() < 0.04 * (hi[k] - lo[k])).collect();
            let Some(f) = band.iter().copied().min_by(|a, b| a.x.total_cmp(&b.x)) else { continue };
            let near: Vec<Vec3> = band.iter().copied().filter(|v| v.x < f.x + 0.15).collect();
            let j = if k == 1 { 2 } else { 1 };
            let (tl, th) = near.iter().fold((f32::MAX, f32::MIN), |(l, h), v| (l.min(v[j]), h.max(v[j])));
            let te = band.iter().map(|v| v.x).fold(f32::MIN, f32::max);
            println!("   station {}={at:7.3}: LE {f:.3?}, 15 cm aft of it {}∈[{tl:.3}, {th:.3}], TE x {te:.3}", ["x", "y", "z"][k], ["x", "y", "z"][j]);
        }
    }
}

/// Exterior triangles of the root node's static geometry with all three
/// corners inside a model-frame box (FlightGear metres: x aft, y right, z
/// up) as imported: count and box, per chunk:
/// `SKAC_AUDIT=meridian190 SKAC_BOX="16,-5.5,1.2,18.5,-0.8,2.1" cargo test --release static_box_dump -- --ignored --nocapture`.
#[test]
#[ignore]
fn static_box_dump() {
    let b: Vec<f32> = std::env::var("SKAC_BOX").unwrap_or_default().split(',').filter_map(|x| x.trim().parse().ok()).collect();
    if b.len() != 6 {
        return;
    }
    let (lo, hi) = (Vec3::new(b[0], b[1], b[2]), Vec3::new(b[3], b[4], b[5]));
    let id = std::env::var("SKAC_AUDIT").unwrap_or_default();
    let Some((a, _, _)) = load(&id) else { return };
    let Some(root) = a.nodes.iter().position(|n| n.parent < 0) else { return };
    let (mut tlo, mut thi, mut total) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN), 0);
    for (ci, c) in a.chunks.iter().enumerate().filter(|(_, c)| c.node == root && !c.interior) {
        let (mut n, mut clo, mut chi) = (0, Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
        for t in (c.first..c.first + c.count).step_by(3) {
            let v: Vec<Vec3> = (0..3).map(|j| Vec3::from(a.vertices[(a.indices[(t + j) as usize] + c.base_vertex) as usize].pos)).collect();
            if v.iter().all(|p| p.cmpge(lo).all() && p.cmple(hi).all()) {
                n += 1;
                for p in &v {
                    clo = clo.min(*p);
                    chi = chi.max(*p);
                }
            }
        }
        if n > 0 {
            println!("chunk {ci} mat {}: {n} triangles, box {clo:.3?} .. {chi:.3?}", c.material);
            total += n;
            tlo = tlo.min(clo);
            thi = thi.max(chi);
        }
    }
    println!("total {total} triangles, box {tlo:.3?} .. {thi:.3?}");
}

/// A spinning node's exterior geometry (with its subtree) in the body
/// frame, gear down: (spin axis, distance of the geometry's centroid from
/// the spin axis, radius about the axis, vertex count).
fn spin_geometry(a: &AircraftAsset, i: usize) -> Option<(Vec3, f32, f32, usize)> {
    let Some(Anim::Spin { center, axis, .. }) = &a.nodes[i].anim else { return None };
    let mut p = Pose::default();
    a.pose(&rest_channels(a), &mut AnimState::default(), 0.0, &mut p);
    let mut in_sub = vec![false; a.nodes.len()];
    for (k, m) in a.nodes.iter().enumerate() {
        in_sub[k] = k == i || (m.parent >= 0 && in_sub[m.parent as usize]);
    }
    let c = p.world[i].transform_point3(*center);
    let ax = p.world[i].transform_vector3(*axis).normalize_or_zero();
    let (mut s, mut n, mut r) = (Vec3::ZERO, 0, 0.0f32);
    for ch in a.chunks.iter().filter(|ch| !ch.interior && in_sub[ch.node]) {
        for k in ch.first..ch.first + ch.count {
            let v = p.world[ch.node].transform_point3(Vec3::from(a.vertices[(a.indices[k as usize] + ch.base_vertex) as usize].pos)) - c;
            s += v;
            n += 1;
            r = r.max((v - ax * v.dot(ax)).length());
        }
    }
    if n == 0 {
        return None;
    }
    let m = s / n as f32;
    Some((ax, (m - ax * m.dot(ax)).length(), r, n))
}

/// Every spinning part (propellers, rotors, wheels): axis, how far its
/// geometry's centre is off the axis, radius:
/// `SKAC_AUDIT=otter cargo test --release spin_dump -- --ignored --nocapture` (all when unset).
#[test]
#[ignore]
fn spin_dump() {
    let only = std::env::var("SKAC_AUDIT").ok();
    for id in ids().into_iter().filter(|id| only.as_ref().is_none_or(|o| o.split(',').any(|x| x == id))) {
        let Some((_, b, _)) = load(&id) else { continue };
        println!("== {id}");
        for (i, n) in b.nodes.iter().enumerate() {
            if let Some((ax, off, r, nv)) = spin_geometry(&b, i) {
                println!("   {:<36} axis {ax:5.2?} off-axis {off:.3} m radius {r:.2} m ({nv} verts){}", n.name, if off > 0.1 * r.max(0.1) { "   OFF" } else { "" });
            }
        }
    }
}
