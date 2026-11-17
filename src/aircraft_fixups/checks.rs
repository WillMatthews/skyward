//! Control-surface checks for every aircraft in `assets/aircraft/` (no list:
//! a new aircraft is checked as soon as it is added). Per aircraft and
//! surface class (elevator, aileron, rudder, flaps, spoilers, leading-edge
//! devices, gear doors):
//!
//! * **moves** — some exterior part moves visibly at full deflection;
//! * **sense** — trailing edges go the right way for the sign of the input;
//! * **pairs** — left and right deflect alike (ailerons: one up, one down);
//! * **hinge** — the hinge is on the leading edge: nothing moves at zero
//!   deflection and, away from horn balances, the leading edge moves far less
//!   than the trailing edge;
//! * **attached** — no part comes off the airframe at full deflection of any
//!   surface or with the gear up or down, and no cockpit geometry hangs
//!   outside it;
//! * **limits** — no surface turns further than a real one could.
//!
//! What an aircraft legitimately lacks is in [`EXCEPTIONS`], with the reason;
//! anything else missing fails. `cargo test --release control_surface_checks
//! -- --nocapture` prints the table.

use super::tests::{detached_parts, ids, load, side_x, stray_interior};
use super::*;

/// The surface classes: (name, channels).
const CLASSES: [(&str, &[&str]); 7] = [
    ("elevator", &["elevator"]),
    ("aileron", &["aileron_l", "aileron_r"]),
    ("rudder", &["rudder"]),
    ("flaps", &["flaps"]),
    ("spoilers", &["spoilers"]),
    ("slats", &["flaps"]),
    ("gear doors", &["gear_pos"]),
];

/// What aircraft lack, and why: (aircraft, class, reason). A class listed
/// here must not move (the entry is then stale and fails too).
pub const EXCEPTIONS: &[(&str, &str, &str)] = &[
    ("ec130", "elevator", "helicopter: the cyclic tilts the rotor"),
    ("ec130", "aileron", "helicopter: the cyclic tilts the rotor"),
    ("ec130", "rudder", "helicopter: the pedals pitch the tail rotor"),
    ("ec130", "flaps", "helicopter"),
    ("ec130", "spoilers", "helicopter"),
    ("ec130", "slats", "helicopter"),
    ("ec130", "gear doors", "helicopter on skids"),
    ("zlin", "rudder", "the rudder's triangles are not in the shipped model (lost at import): needs a re-import"),
    // The real aircraft has none.
    ("concorde", "spoilers", "Concorde has no spoilers or speed brakes"),
    ("hercules", "spoilers", "the C-130 has no spoilers"),
    ("cub", "spoilers", "light aircraft"),
    ("cub", "slats", "fixed slot-less wing"),
    ("cub", "gear doors", "fixed gear"),
    ("dc3", "spoilers", "the DC-3 has none"),
    ("dc3", "slats", "the DC-3 has none"),
    ("dc3", "gear doors", "the DC-3's main wheels retract into open nacelle wells"),
    ("extra500", "spoilers", "light aircraft"),
    ("kestrel", "spoilers", "light aircraft"),
    ("kestrel", "gear doors", "fixed gear"),
    ("otter", "spoilers", "light aircraft"),
    ("otter", "gear doors", "fixed gear"),
    ("peregrine", "spoilers", "the Spitfire has none"),
    ("peregrine", "slats", "the Spitfire has none"),
    ("peregrine", "gear doors", "the Spitfire's leg fairings ride on the legs"),
    ("swift", "flaps", "the ASK 21 has no flaps (airbrakes: spoilers)"),
    ("swift", "slats", "glider"),
    ("swift", "gear doors", "fixed main wheel"),
    ("vulcan", "flaps", "the Vulcan has none (elevons only)"),
    ("vulcan", "slats", "the Vulcan has none"),
    ("zlin", "spoilers", "light aircraft"),
    ("zlin", "slats", "light aircraft"),
    ("zlin", "gear doors", "fixed gear"),
    // Model gaps: the real aircraft has them, the shipped model does not
    // animate them (their geometry is static or merged into the airframe).
    ("aerolink", "spoilers", "MODEL GAP: the ATR's roll spoilers are not animated in the FlightGear model"),
    ("citationx", "slats", "MODEL GAP: slats read surface-positions/slat-pos-norm (unmapped), not yet cut out"),
    ("crj700", "slats", "MODEL GAP: slats not animated in the 2016 model"),
    ("extra500", "gear doors", "MODEL GAP: only the door mechanisms are animated, the panels are static"),
    ("meridian190", "spoilers", "MODEL GAP: spoilers merged into the static wing, not yet cut out"),
    ("meridian190", "slats", "MODEL GAP: slats merged into the static wing, not yet cut out"),
    ("meridian320", "flaps", "MODEL GAP: Fowler flaps (float[12], track motion) not yet cut out"),
    ("meridian320", "slats", "MODEL GAP: slats not yet cut out"),
    ("vulcan", "spoilers", "MODEL GAP: the Vulcan's airbrakes are not animated in the model"),
    ("zlin", "flaps", "MODEL GAP: the Zlin's flaps are not animated in the model"),
];

/// Surfaces that rightly turn about their middle: (aircraft, node name
/// part, why).
pub const MID_HINGED: &[(&str, &str, &str)] = &[("lancer", "HorizonTail", "all-moving stabilators pivot near mid-chord")];

/// Upper bound on a surface's own rotation at full deflection, degrees.
fn limit(class: &str) -> f32 {
    match class {
        // (The Spitfire's split flaps go down 85°.)
        "flaps" | "spoilers" => 90.0,
        "gear doors" => 180.0,
        _ => 60.0,
    }
}

/// Nodes driven by a class's channels that move exterior geometry at full
/// deflection (either sign), with their trailing-edge motion when they are
/// trailing-edge surfaces.
fn class_nodes(a: &AircraftAsset, class: &str, chs: &[&str]) -> Vec<(usize, usize, Option<(f32, usize)>)> {
    let door = |n: &str| ["door", "porte", "trappe", "hatch"].iter().any(|d| n.to_lowercase().contains(d));
    let mut out = Vec::new();
    for name in chs {
        let Some(ch) = a.channel(name) else { continue };
        let sense = SURFACE_CHANNELS.iter().find(|s| s.0 == *name).map(|s| s.1);
        for (i, n) in a.nodes.iter().enumerate() {
            if !node_reads(n, ch as u16) || !moves(a, i, ch) {
                continue;
            }
            let te = sense.and_then(|s| trailing_edge_motion(a, i, ch, s));
            let keep = match class {
                "gear doors" => door(&n.name),
                "slats" => te.is_none() && !door(&n.name),
                // (Fuselage speed brakes and glider airbrakes are no
                // trailing-edge surfaces.)
                "spoilers" => true,
                _ => te.is_some(),
            };
            if keep {
                out.push((i, ch, te));
            }
        }
    }
    out
}

/// Does the node move its exterior geometry by more than 1 cm for its
/// channel at ±1 (gear down; the gear channel from down to up)?
fn moves(a: &AircraftAsset, node: usize, ch: usize) -> bool {
    let mut in_sub = vec![false; a.nodes.len()];
    for (i, n) in a.nodes.iter().enumerate() {
        in_sub[i] = i == node || (n.parent >= 0 && in_sub[n.parent as usize]);
    }
    let rest = rest_channels(a);
    let mut p0 = Pose::default();
    a.pose(&rest, &mut AnimState::default(), 0.0, &mut p0);
    let targets: &[f32] = if a.channels[ch] == "gear_pos" { &[0.0] } else { &[1.0, -1.0] };
    targets.iter().any(|&v| {
        let mut ch1 = rest.clone();
        ch1[ch] = v;
        let mut p1 = Pose::default();
        a.pose(&ch1, &mut AnimState::default(), 0.0, &mut p1);
        a.chunks.iter().filter(|c| !c.interior && in_sub[c.node] && p0.visible[c.node] && p1.visible[c.node]).any(|c| {
            (c.first..c.first + c.count).step_by(7).any(|k| {
                let v = Vec3::from(a.vertices[(a.indices[k as usize] + c.base_vertex) as usize].pos);
                p0.world[c.node].transform_point3(v).distance(p1.world[c.node].transform_point3(v)) > 0.01
            })
        })
    })
}

/// Share of a hinged node's exterior surface area (with its subtree, at rest)
/// more than 15 % of its depth (at least 5 cm) ahead of its hinge line: a few horn balances and the
/// rounded nose are fine, half the surface means it turns about its middle.
fn ahead_of_hinge(a: &AircraftAsset, node: usize) -> f32 {
    let Some(Anim::Rotate { center, axis, .. }) = &a.nodes[node].anim else { return 0.0 };
    let mut p = Pose::default();
    a.pose(&rest_channels(a), &mut AnimState::default(), 0.0, &mut p);
    let (c, ax) = (p.world[node].transform_point3(*center), p.world[node].transform_vector3(*axis).normalize_or_zero());
    // Body −Z (aft), square to the hinge.
    let aft = (Vec3::NEG_Z - ax * -ax.z).normalize_or_zero();
    let mut in_sub = vec![false; a.nodes.len()];
    for (i, n) in a.nodes.iter().enumerate() {
        in_sub[i] = i == node || (n.parent >= 0 && in_sub[n.parent as usize]);
    }
    // (Triangle centroids weighted by area: dense, rounded horn balances
    // carry many vertices but little surface.)
    let mut d = Vec::new();
    for ch in a.chunks.iter().filter(|ch| !ch.interior && in_sub[ch.node] && p.visible[ch.node]) {
        let m = p.world[ch.node];
        for t in (ch.first..ch.first + ch.count).step_by(3) {
            let v: [Vec3; 3] = std::array::from_fn(|j| m.transform_point3(Vec3::from(a.vertices[(a.indices[(t + j as u32) as usize] + ch.base_vertex) as usize].pos)));
            let area = (v[1] - v[0]).cross(v[2] - v[0]).length() / 2.0;
            d.push((((v[0] + v[1] + v[2]) / 3.0 - c).dot(aft), area));
        }
    }
    // (Rounded noses reach ahead of the hinge by a share of the chord.)
    let depth = d.iter().map(|x| x.0).fold(0.0f32, f32::max);
    let margin = (0.15 * depth).max(0.05);
    let total: f32 = d.iter().map(|x| x.1).sum();
    d.iter().filter(|x| x.0 < -margin).map(|x| x.1).sum::<f32>() / total.max(1e-9)
}

/// The node's own rotation at full deflection, degrees (largest of ±1).
fn own_angle(a: &AircraftAsset, node: usize, ch: usize) -> f32 {
    let Some(Anim::Rotate { deg, .. }) = &a.nodes[node].anim else { return 0.0 };
    let mut c = rest_channels(a);
    [1.0f32, -1.0, 0.0]
        .iter()
        .map(|&v| {
            c[ch] = v;
            deg.eval(&c).abs()
        })
        .fold(0.0, f32::max)
}

/// One aircraft's results: per class, the failures (empty = pass).
fn check(id: &str) -> Vec<(&'static str, Vec<String>)> {
    let Some((_, b, _)) = load(id) else { return Vec::new() };
    let mut out = Vec::new();
    for (class, chs) in CLASSES {
        let mut f = Vec::new();
        let nodes = class_nodes(&b, class, chs);
        let excused = EXCEPTIONS.iter().find(|e| e.0 == id && e.1 == class);
        // a) moves
        let visible = nodes.iter().any(|(_, _, te)| te.is_none_or(|(m, nv)| m.abs() > 0.02 && nv >= 10));
        match (visible, excused) {
            (false, None) => f.push("missing: nothing moves".to_string()),
            (true, Some(_)) => f.push("moves, but listed as an exception".to_string()),
            _ => {}
        }
        for &(i, ch, te) in &nodes {
            let name = &b.nodes[i].name;
            // b) sense (slivers of a few vertices prove nothing)
            if let Some((m, nv)) = te {
                if m < -0.01 && nv >= 10 {
                    f.push(format!("{name}: {} moves the wrong way ({m:+.3} m)", b.channels[ch]));
                }
                // d) hinge on the leading edge (hinged surfaces that move
                // visibly; flaps run on tracks below the wing)
                let mid = MID_HINGED.iter().any(|h| h.0 == id && name.contains(h.1));
                if m.abs() > 0.02 && nv >= 10 && class != "flaps" && !mid {
                    let ahead = ahead_of_hinge(&b, i);
                    if ahead > 0.4 {
                        f.push(format!("{name}: {:.0} % of it lies ahead of its hinge line (hinged in its middle)", ahead * 100.0));
                    }
                }
            }
            // f) limits
            let ang = own_angle(&b, i, ch);
            if ang > limit(class) {
                f.push(format!("{name}: turns {ang:.0}° (limit {:.0}°)", limit(class)));
            }
        }
        // c) pairs: the biggest surface per side, for the TE classes
        if matches!(class, "elevator" | "aileron" | "flaps" | "spoilers") {
            let side = |left: bool| {
                nodes
                    .iter()
                    .filter_map(|&(i, ch, te)| te.map(|(m, nv)| (i, ch, m, nv)))
                    .filter(|&(i, ..)| side_x(&b, i).abs() > 0.3 && (side_x(&b, i) > 0.0) == left)
                    .filter(|&(_, ch, ..)| class != "aileron" || b.channels[ch] == if left { "aileron_l" } else { "aileron_r" } || b.channel("aileron_r").is_none())
                    .max_by_key(|x| x.3)
                    .map(|x| x.2)
            };
            match (side(true), side(false)) {
                (Some(l), Some(r)) if (l - r).abs() > 0.25 * l.abs().max(r.abs()) => f.push(format!("left {l:+.3} m vs right {r:+.3} m")),
                (Some(_), None) | (None, Some(_)) => f.push("moves on one side only".to_string()),
                _ => {}
            }
        }
        out.push((class, f));
    }
    // e) attached: every class at ±1, gear up and down
    let mut f = Vec::new();
    let rest = rest_channels(&b);
    let mut poses: Vec<(String, Vec<f32>)> = vec![("rest".into(), rest.clone())];
    for name in ["elevator", "aileron_l", "aileron_r", "rudder", "flaps", "spoilers", "steer_norm", "steer_deg"] {
        if let Some(c) = b.channel(name) {
            for v in [1.0, -1.0] {
                let mut ch = rest.clone();
                ch[c] = v;
                poses.push((format!("{name} {v:+}"), ch));
            }
        }
    }
    if let Some(g) = b.channel("gear_pos") {
        for v in [0.0, 0.5] {
            let mut ch = rest.clone();
            ch[g] = v;
            poses.push((format!("gear {v}"), ch));
        }
    }
    for (label, ch) in &poses {
        for (n, nv, c) in detached_parts(&b, ch) {
            f.push(format!("{label}: {} ({nv} verts at {c:.1?}) off the airframe", b.nodes[n].name));
        }
    }
    for (n, share, nv) in stray_interior(&b) {
        f.push(format!("cockpit part {} ({nv} verts) {:.0} % outside the airframe", b.nodes[n].name, share * 100.0));
    }
    out.push(("attached", f));
    out
}

/// Every aircraft passes every check (see the module docs); prints the table.
#[test]
fn control_surface_checks() {
    let ids = ids();
    let mut failed = Vec::new();
    let classes: Vec<&str> = CLASSES.iter().map(|c| c.0).chain(["attached"]).collect();
    println!("{:<12} {}", "aircraft", classes.iter().map(|c| format!("{c:<11}")).collect::<String>());
    for id in &ids {
        let res = check(id);
        let row: String = classes
            .iter()
            .map(|c| {
                let fails = res.iter().find(|r| r.0 == *c).map(|r| r.1.len()).unwrap_or(0);
                let exc = EXCEPTIONS.iter().any(|e| e.0 == id && e.1 == *c);
                format!(
                    "{:<11}",
                    if fails > 0 {
                        "FAIL"
                    } else if exc {
                        "n/a"
                    } else {
                        "pass"
                    }
                )
            })
            .collect();
        println!("{id:<12} {row}");
        for (c, fails) in res {
            for x in fails {
                failed.push(format!("{id} {c}: {x}"));
            }
        }
    }
    for e in EXCEPTIONS {
        assert!(ids.is_empty() || ids.iter().any(|i| i == e.0), "exception for unknown aircraft {}", e.0);
    }
    assert!(failed.is_empty(), "{} failures:\n{}", failed.len(), failed.join("\n"));
}
