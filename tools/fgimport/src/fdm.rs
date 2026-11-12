//! JSBSim FDM geometry needed to place the visual model in Skyward's body
//! frame (CG origin) and to report contact points / engines / eye.

use crate::xml::Elem;
use glam::Vec3;
use serde_json::{json, Value};

pub const IN: f32 = 0.0254;

/// A location in JSBSim structural coordinates, converted to metres
/// (x aft, y right, z up — the same axes as FlightGear model coordinates).
pub fn loc_m(e: &Elem) -> Vec3 {
    let unit = e.attr("unit").unwrap_or("IN");
    let k = match unit {
        "M" => 1.0,
        "FT" => 0.3048,
        "IN" => IN,
        _ => IN,
    };
    Vec3::new(e.f("x").unwrap_or(0.0), e.f("y").unwrap_or(0.0), e.f("z").unwrap_or(0.0)) * k
}

fn val(e: &Elem, n: &str) -> Option<(f32, String)> {
    let c = e.child(n)?;
    Some((c.text.trim().parse().ok()?, c.attr("unit").unwrap_or("").to_string()))
}

pub struct Jsb {
    /// Structural (m) of the visual reference point and the CG.
    pub vrp: Vec3,
    pub cg: Vec3,
    pub report: Value,
}

/// Structural metres → Skyward body frame (+Z nose, +Y up, +X left), CG origin.
pub fn to_body(s: Vec3, cg: Vec3) -> Vec3 {
    let d = s - cg;
    Vec3::new(-d.y, d.z, -d.x)
}

fn lbs_to_kg(v: f32, unit: &str) -> f32 {
    match unit {
        "KG" => v,
        _ => v * 0.453_592,
    }
}

fn inertia_si(v: f32, unit: &str) -> f32 {
    match unit {
        "KG*M2" => v,
        _ => v * 1.355_818, // slug·ft²
    }
}

fn area_si(v: f32, unit: &str) -> f32 {
    match unit {
        "M2" => v,
        _ => v * 0.092_903,
    }
}

fn len_si(v: f32, unit: &str) -> f32 {
    match unit {
        "M" => v,
        "IN" => v * IN,
        _ => v * 0.3048,
    }
}

pub fn read(fdm: &Elem, cfg: &Value) -> Jsb {
    let metrics = fdm.child("metrics");
    let mut vrp = Vec3::ZERO;
    let mut aerorp = Vec3::ZERO;
    let mut eye = None;
    let (mut sw, mut bw, mut cbar) = (0.0, 0.0, 0.0);
    if let Some(m) = metrics {
        for l in m.children("location") {
            match l.attr("name").unwrap_or("") {
                "VRP" => vrp = loc_m(l),
                "AERORP" => aerorp = loc_m(l),
                "EYEPOINT" => eye = Some(loc_m(l)),
                _ => {}
            }
        }
        if let Some((v, u)) = val(m, "wingarea") {
            sw = area_si(v, &u);
        }
        if let Some((v, u)) = val(m, "wingspan") {
            bw = len_si(v, &u);
        }
        if let Some((v, u)) = val(m, "chord") {
            cbar = len_si(v, &u);
        }
    }
    let mb = fdm.child("mass_balance");
    let mut cg = Vec3::ZERO;
    let (mut empty, mut ixx, mut iyy, mut izz) = (0.0, 0.0, 0.0, 0.0);
    let mut point_mass = 0.0;
    if let Some(mb) = mb {
        if let Some(l) = mb.children("location").find(|l| l.attr("name") == Some("CG")) {
            cg = loc_m(l);
        }
        if let Some((v, u)) = val(mb, "emptywt") {
            empty = lbs_to_kg(v, &u);
        }
        if let Some((v, u)) = val(mb, "ixx") {
            ixx = inertia_si(v, &u);
        }
        if let Some((v, u)) = val(mb, "iyy") {
            iyy = inertia_si(v, &u);
        }
        if let Some((v, u)) = val(mb, "izz") {
            izz = inertia_si(v, &u);
        }
        for pm in mb.children("pointmass") {
            if let Some((v, u)) = val(pm, "weight") {
                point_mass += lbs_to_kg(v, &u);
            }
        }
    }
    // Loaded condition: empty + point masses (defaults, or config `pointmass_kg`
    // by name) + fuel (config `fuel_kg`, spread over the tanks by capacity;
    // default the FDM's initial contents). Inertia: empty-weight inertia plus
    // the point masses' parallel-axis terms about the loaded CG.
    let empty_cg = cg;
    let mut masses: Vec<(f32, Vec3)> = vec![(empty, empty_cg)];
    if let Some(mb) = mb {
        for pm in mb.children("pointmass") {
            let name = pm.attr("name").unwrap_or("");
            let w = cfg.get("pointmass_kg").and_then(|m| m.get(name)).and_then(|v| v.as_f64()).map(|v| v as f32)
                .or_else(|| val(pm, "weight").map(|(v, u)| lbs_to_kg(v, &u)))
                .unwrap_or(0.0);
            if let Some(l) = pm.child("location") {
                masses.push((w, loc_m(l)));
            }
        }
    }
    let mut tanks: Vec<(f32, f32, Vec3)> = Vec::new(); // (capacity, contents, loc)
    if let Some(pr) = fdm.child("propulsion") {
        for t in pr.children("tank") {
            let cap = val(t, "capacity").map(|(v, u)| lbs_to_kg(v, &u)).unwrap_or(0.0);
            let cont = val(t, "contents").map(|(v, u)| lbs_to_kg(v, &u)).unwrap_or(0.0);
            if let Some(l) = t.child("location") {
                tanks.push((cap, cont, loc_m(l)));
            }
        }
    }
    let fuel_total = cfg.get("fuel_kg").and_then(|v| v.as_f64()).map(|v| v as f32);
    let cap_sum: f32 = tanks.iter().map(|t| t.0).sum();
    for (cap, cont, l) in &tanks {
        let w = match fuel_total {
            Some(f) if cap_sum > 0.0 => f * cap / cap_sum,
            _ => *cont,
        };
        masses.push((w, *l));
    }
    let m_tot: f32 = masses.iter().map(|m| m.0).sum();
    let mut cg_loaded = if m_tot > 0.0 { masses.iter().map(|(m, p)| *p * *m).sum::<Vec3>() / m_tot } else { empty_cg };
    // Skyward's flight model is laterally symmetric: keep the CG on the centreline.
    cg_loaded.y = 0.0;
    let (mut ixx_l, mut iyy_l, mut izz_l) = (ixx, iyy, izz);
    for (k, (m, p)) in masses.iter().enumerate() {
        let d = *p - cg_loaded;
        if k == 0 {
            // Empty inertia is about the empty CG: shift it.
            ixx_l += m * (d.y * d.y + d.z * d.z);
            iyy_l += m * (d.x * d.x + d.z * d.z);
            izz_l += m * (d.x * d.x + d.y * d.y);
        } else {
            ixx_l += m * (d.y * d.y + d.z * d.z);
            iyy_l += m * (d.x * d.x + d.z * d.z);
            izz_l += m * (d.x * d.x + d.y * d.y);
        }
    }
    let cg = cg_loaded;
    let mut contacts = Vec::new();
    if let Some(gr) = fdm.child("ground_reactions") {
        for c in gr.children("contact") {
            let Some(l) = c.child("location") else { continue };
            let p = to_body(loc_m(l), cg);
            let k = val(c, "spring_coeff").map(|(v, u)| if u.contains("N") { v } else { v * 14.5939 }).unwrap_or(0.0);
            let d = val(c, "damping_coeff").map(|(v, u)| if u.contains("N") { v } else { v * 14.5939 }).unwrap_or(0.0);
            contacts.push(json!({
                "name": c.attr("name").unwrap_or(""),
                "type": c.attr("type").unwrap_or(""),
                "pos": [p.x, p.y, p.z],
                "spring_n_m": k, "damping_n_s_m": d,
                "steer_deg": c.f("max_steer").unwrap_or(0.0),
                "brake_group": c.text_of("brake_group").unwrap_or(""),
                "retractable": c.f("retractable").unwrap_or(0.0),
                "static_friction": c.f("static_friction").unwrap_or(0.0),
            }));
        }
    }
    let mut engines = Vec::new();
    let mut fuel = 0.0;
    if let Some(pr) = fdm.child("propulsion") {
        for e in pr.children("engine") {
            let loc = e.child("location").map(loc_m).map(|s| to_body(s, cg));
            let th = e.child("thruster").and_then(|t| t.child("location")).map(loc_m).map(|s| to_body(s, cg));
            engines.push(json!({
                "file": e.attr("file").unwrap_or(""),
                "thruster": e.child("thruster").and_then(|t| t.attr("file")).unwrap_or(""),
                "pos": loc.map(|p| vec![p.x, p.y, p.z]),
                "thruster_pos": th.map(|p| vec![p.x, p.y, p.z]),
            }));
        }
        for t in pr.children("tank") {
            if let Some((v, u)) = val(t, "contents") {
                fuel += lbs_to_kg(v, &u);
            }
        }
    }
    let eye_b = eye.map(|e| to_body(e, cg));
    let ar = to_body(aerorp, cg);
    let report = json!({
        "wing_area_m2": sw, "wingspan_m": bw, "chord_m": cbar,
        "empty_kg": empty, "pointmass_kg": point_mass, "fuel_initial_kg": fuel,
        "loaded_kg": m_tot, "inertia_loaded_kg_m2": {"roll_ixx": ixx_l, "pitch_iyy": iyy_l, "yaw_izz": izz_l},
        "empty_cg_structural_m": [empty_cg.x, empty_cg.y, empty_cg.z],
        "ixx": ixx, "iyy": iyy, "izz": izz,
        "cg_structural_m": [cg.x, cg.y, cg.z],
        "aerorp_body": [ar.x, ar.y, ar.z],
        "eyepoint_body": eye_b.map(|p| vec![p.x, p.y, p.z]),
        "contacts": contacts,
        "engines": engines,
    });
    Jsb { vrp, cg, report }
}
