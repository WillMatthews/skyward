use super::*;

fn ids() -> Vec<String> {
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
fn load(id: &str) -> Option<(AircraftAsset, AircraftAsset, Vec<String>)> {
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
fn side_x(a: &AircraftAsset, node: usize) -> f32 {
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
    for (ci, c) in a.chunks.iter().enumerate().filter(|(_, c)| !c.interior && p.visible[c.node]) {
        let mut n = 0;
        for t in (c.first..c.first + c.count).step_by(3) {
            let cen = (0..3).map(|j| p.world[c.node].transform_point3(Vec3::from(a.vertices[(a.indices[(t + j) as usize] + c.base_vertex) as usize].pos))).sum::<Vec3>() / 3.0;
            if cen.cmpge(lo).all() && cen.cmple(hi).all() {
                n += 1;
            }
        }
        if n > 0 {
            println!("chunk {ci} node {} '{}' mat {} two_sided {}: {n} triangles", c.node, a.nodes[c.node].name, c.material, a.materials[c.material].two_sided);
        }
    }
}
