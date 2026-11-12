//! FlightGear model XML → node tree with animations → flattened Skyward asset.

use crate::ac3d::{self, AcMaterial, AcObject};
use crate::expr::{anim_value, op, Expr, PropMap};
use crate::fdm;
use crate::xml::{self, Ctx, Elem};
use glam::{Mat3, Mat4, Quat, Vec2, Vec3};
use regex::Regex;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::Write;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug)]
enum Anim {
    Rotate { center: Vec3, axis: Vec3, deg: Expr },
    Translate { axis: Vec3, m: Expr },
    Spin { center: Vec3, axis: Vec3, rpm: Expr },
    Select(Expr),
    Scale { center: Vec3, s: [Expr; 3] },
}

#[derive(Clone, Debug, Default)]
struct Flags {
    translucent: bool,
    hide: bool,
    interior: bool,
    /// Emission colour × expression (material animations).
    emit: Option<(Vec3, Expr)>,
    texture: Option<PathBuf>,
}

#[derive(Default)]
struct Geom {
    obj_tris: Vec<ac3d::Tri>,
    materials: std::rc::Rc<Vec<AcMaterial>>,
    texture: Option<PathBuf>,
    crease: f32,
}

#[derive(Default)]
struct Node {
    name: String,
    parent: Option<usize>,
    kids: Vec<usize>,
    local: Mat4,
    anim: Option<Anim>,
    geom: Option<Geom>,
    lines: Vec<[Vec3; 2]>,
    flags: Flags,
    /// Light marker (kind) placed at this node's origin.
    light: Option<String>,
}

struct Builder<'a> {
    nodes: Vec<Node>,
    ctx: Ctx,
    pm: PropMap,
    cfg: &'a Value,
    exclude_models: Vec<Regex>,
    exclude_objects: Vec<Regex>,
    interior_models: Vec<Regex>,
    interior_objects: Vec<Regex>,
    light_models: Vec<Regex>,
    tex_override: HashMap<String, String>,
    range_m: f32,
    log: Vec<String>,
    anim_counts: HashMap<String, usize>,
}

fn regexes(cfg: &Value, key: &str) -> Vec<Regex> {
    cfg.get(key)
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|s| s.as_str()).map(|s| Regex::new(s).expect("bad regex")).collect())
        .unwrap_or_default()
}

fn v3(e: Option<&Elem>, suffix: &str) -> Vec3 {
    match e {
        Some(e) => Vec3::new(
            e.f(&format!("x{suffix}")).unwrap_or(0.0),
            e.f(&format!("y{suffix}")).unwrap_or(0.0),
            e.f(&format!("z{suffix}")).unwrap_or(0.0),
        ),
        None => Vec3::ZERO,
    }
}

fn offsets_matrix(o: Option<&Elem>) -> Mat4 {
    let Some(o) = o else { return Mat4::IDENTITY };
    let t = Vec3::new(o.f("x-m").unwrap_or(0.0), o.f("y-m").unwrap_or(0.0), o.f("z-m").unwrap_or(0.0));
    let h = o.f("heading-deg").unwrap_or(0.0).to_radians();
    let p = o.f("pitch-deg").unwrap_or(0.0).to_radians();
    let r = o.f("roll-deg").unwrap_or(0.0).to_radians();
    let q = Quat::from_rotation_z(h) * Quat::from_rotation_y(p) * Quat::from_rotation_x(r);
    let m = Mat4::from_translation(t) * Mat4::from_quat(q);
    // Uniform scale (rare).
    let s = Vec3::new(o.f("x-scale").unwrap_or(1.0), o.f("y-scale").unwrap_or(1.0), o.f("z-scale").unwrap_or(1.0));
    m * Mat4::from_scale(s)
}

impl<'a> Builder<'a> {
    fn add(&mut self, mut n: Node, parent: Option<usize>) -> usize {
        n.parent = parent;
        if n.local == Mat4::ZERO {
            n.local = Mat4::IDENTITY;
        }
        let id = self.nodes.len();
        self.nodes.push(n);
        if let Some(p) = parent {
            self.nodes[p].kids.push(id);
        }
        id
    }

    fn group(&mut self, name: &str, local: Mat4, parent: Option<usize>) -> usize {
        self.add(Node { name: name.to_string(), local, ..Default::default() }, parent)
    }

    fn rel(&self, p: &Path) -> String {
        p.strip_prefix(&self.ctx.root).unwrap_or(p).to_string_lossy().replace('\\', "/")
    }

    fn load_ac(&mut self, path: &Path, parent: usize) -> Result<(), String> {
        let raw = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let text = String::from_utf8_lossy(&raw);
        let ac = ac3d::parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        let mats = std::rc::Rc::new(ac.materials);
        let dir = path.parent().unwrap().to_path_buf();
        let interior = self.interior_models.iter().any(|r| r.is_match(&self.rel(path)));
        self.add_ac_object(&ac.root, &mats, &dir, parent, interior);
        Ok(())
    }

    fn add_ac_object(&mut self, o: &AcObject, mats: &std::rc::Rc<Vec<AcMaterial>>, dir: &Path, parent: usize, interior: bool) {
        if self.exclude_objects.iter().any(|r| r.is_match(&o.name)) {
            return;
        }
        let texture = o.texture.as_ref().and_then(|t| self.texture_path(t, dir));
        let interior = interior || self.interior_objects.iter().any(|r| r.is_match(&o.name));
        let geom = (!o.tris.is_empty()).then(|| Geom { obj_tris: o.tris.clone(), materials: mats.clone(), texture, crease: o.crease });
        let id = self.add(
            Node {
                name: o.name.clone(),
                geom,
                lines: o.lines.clone(),
                flags: Flags { interior, ..Default::default() },
                ..Default::default()
            },
            Some(parent),
        );
        for k in &o.kids {
            self.add_ac_object(k, mats, dir, id, interior);
        }
    }

    fn texture_path(&self, t: &str, dir: &Path) -> Option<PathBuf> {
        let base = Path::new(t).file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
        if let Some(r) = self.tex_override.get(t).or_else(|| self.tex_override.get(&base)) {
            let p = self.ctx.root.join(r);
            if p.exists() {
                return Some(p);
            }
        }
        let p = dir.join(t);
        if p.exists() {
            return Some(p);
        }
        self.ctx.resolve(t, dir).or_else(|| {
            // FG also searches the model's directory for the bare name.
            let p = dir.join(&base);
            p.exists().then_some(p)
        })
    }

    /// Load a model XML (or .ac) under `parent`; returns its group node.
    fn load_model(&mut self, path: &Path, name: &str, parent: usize, depth: usize) -> Result<usize, String> {
        if depth > 16 {
            return Err("model nesting too deep".into());
        }
        if path.extension().map(|e| e.eq_ignore_ascii_case("ac")).unwrap_or(false) {
            let g = self.group(name, Mat4::IDENTITY, Some(parent));
            self.load_ac(path, g)?;
            return Ok(g);
        }
        let x = xml::load(&self.ctx, path)?;
        let dir = path.parent().unwrap().to_path_buf();
        let g = self.group(name, offsets_matrix(x.child("offsets")), Some(parent));
        if self.interior_models.iter().any(|r| r.is_match(&self.rel(path))) {
            self.nodes[g].flags.interior = true;
        }
        if let Some(p) = x.text_of("path") {
            if let Some(acp) = self.ctx.resolve(p, &dir) {
                if acp.extension().map(|e| e == "xml").unwrap_or(false) {
                    self.load_model(&acp, "", g, depth + 1)?;
                } else if let Err(e) = self.load_ac(&acp, g) {
                    self.log.push(e);
                }
            } else {
                self.log.push(format!("missing model file {p} (from {})", self.rel(path)));
            }
        }
        for m in x.children("model") {
            let mname = m.text_of("name").unwrap_or("").to_string();
            let Some(mp) = m.text_of("path") else { continue };
            let resolved = self.ctx.resolve(mp, &dir);
            let rel = resolved.as_ref().map(|p| self.rel(p)).unwrap_or_else(|| mp.to_string());
            let local = offsets_matrix(m.child("offsets"));
            if let Some(re) = self.light_models.iter().find(|r| r.is_match(&rel) || r.is_match(&mname)) {
                let _ = re;
                let kind = light_kind(&format!("{rel} {mname}"));
                let id = self.add(Node { name: mname.clone(), local, light: Some(kind), ..Default::default() }, Some(g));
                // Conditions on lights are ignored: the app decides when lights show.
                let _ = id;
                continue;
            }
            if self.exclude_models.iter().any(|r| r.is_match(&rel) || r.is_match(&mname)) {
                continue;
            }
            let Some(sub) = resolved else {
                self.log.push(format!("skip external/missing submodel {mp}"));
                continue;
            };
            // Static or dynamic condition on the submodel.
            let cond = m.child("condition").map(|c| self.pm.condition(c));
            let mut attach = g;
            match cond.as_ref().and_then(|c| c.is_const()) {
                Some(v) if v == 0.0 => continue,
                Some(_) => {}
                None => {
                    if let Some(c) = cond {
                        attach = self.add(Node { name: format!("{mname}#cond"), anim: Some(Anim::Select(c)), ..Default::default() }, Some(g));
                    }
                }
            }
            let wrap = self.group(&mname, local, Some(attach));
            if let Err(e) = self.load_model(&sub, "", wrap, depth + 1) {
                self.log.push(e);
            }
        }
        for e in x.children("effect") {
            let inh = e.text_of("inherits-from").unwrap_or("").to_lowercase();
            let names: Vec<String> = e.children("object-name").map(|o| o.text.trim().to_string()).collect();
            let hide = ["light-cone", "lightcone", "light_cone", "procedural-light", "procedural_light", "rain", "thrust", "flame", "contrail", "landing-light", "heat-blur", "heatblur"]
                .iter()
                .any(|k| inh.contains(k));
            let transl = ["transparent", "translucent"].iter().any(|k| inh.contains(k))
                || (["glass", "canopy", "windshield", "window"].iter().any(|k| inh.contains(k)) && !inh.contains("reflect"));
            if !(hide || transl) {
                continue;
            }
            for n in &names {
                for id in self.find(g, n) {
                    if hide {
                        self.nodes[id].flags.hide = true;
                    }
                    if transl {
                        self.nodes[id].flags.translucent = true;
                    }
                }
            }
        }
        for a in x.children("animation") {
            self.animation(g, a, &dir);
        }
        Ok(g)
    }

    /// All nodes named `name` in the subtree of `root`.
    fn find(&self, root: usize, name: &str) -> Vec<usize> {
        let mut out = Vec::new();
        let mut stack = vec![root];
        while let Some(i) = stack.pop() {
            if self.nodes[i].name == name && i != root {
                out.push(i);
            }
            stack.extend(self.nodes[i].kids.iter().copied());
        }
        out
    }

    /// Gather `targets` under a new group named `name` (placed where the first one was).
    fn group_under(&mut self, name: &str, targets: &[usize]) -> usize {
        let parent = self.nodes[targets[0]].parent;
        let id = self.nodes.len();
        self.nodes.push(Node { name: name.to_string(), parent, local: Mat4::IDENTITY, ..Default::default() });
        if let Some(p) = parent {
            for k in &mut self.nodes[p].kids {
                if *k == targets[0] {
                    *k = id;
                }
            }
        }
        for &t in targets {
            if let Some(op) = self.nodes[t].parent {
                if op != id {
                    self.nodes[op].kids.retain(|&k| k != t);
                }
            }
            self.nodes[t].parent = Some(id);
            if !self.nodes[id].kids.contains(&t) {
                self.nodes[id].kids.push(t);
            }
        }
        id
    }

    /// Insert an animation node directly above `target`.
    fn wrap(&mut self, target: usize, anim: Anim, label: &str) -> usize {
        let parent = self.nodes[target].parent;
        let id = self.nodes.len();
        self.nodes.push(Node { name: format!("{label}"), parent, anim: Some(anim), local: Mat4::IDENTITY, ..Default::default() });
        if let Some(p) = parent {
            for k in &mut self.nodes[p].kids {
                if *k == target {
                    *k = id;
                }
            }
        }
        self.nodes[id].kids.push(target);
        self.nodes[target].parent = Some(id);
        id
    }

    fn axis_of(&self, g: usize, a: &Elem) -> (Vec3, Vec3) {
        let ax = a.child("axis");
        let mut center = v3(a.child("center"), "-m");
        let mut axis = Vec3::X;
        if let Some(ax) = ax {
            if let Some(on) = ax.text_of("object-name") {
                // Axis given by a line object.
                if let Some(seg) = self.find(g, on).iter().find_map(|&i| self.nodes[i].lines.first().copied()) {
                    return ((seg[0] + seg[1]) * 0.5, (seg[1] - seg[0]).normalize_or_zero());
                }
                // Mark axis objects hidden either way (they are helper lines).
            }
            if ax.child("x1-m").is_some() {
                let p1 = v3(Some(ax), "1-m");
                let p2 = v3(Some(ax), "2-m");
                center = (p1 + p2) * 0.5;
                axis = (p2 - p1).normalize_or_zero();
            } else {
                axis = v3(Some(ax), "").normalize_or_zero();
            }
        }
        if axis == Vec3::ZERO {
            axis = Vec3::X;
        }
        (center, axis)
    }

    fn animation(&mut self, g: usize, a: &Elem, dir: &Path) {
        let ty = a.text_of("type").unwrap_or("").to_string();
        *self.anim_counts.entry(ty.clone()).or_default() += 1;
        let names: Vec<String> = a.children("object-name").map(|o| o.text.trim().to_string()).collect();
        let targets: Vec<usize> = if names.is_empty() {
            // Whole model.
            self.nodes[g].kids.clone()
        } else {
            names.iter().flat_map(|n| self.find(g, n)).collect()
        };
        if targets.is_empty() {
            return;
        }
        // A named animation groups its objects under that name (later
        // animations can then refer to the group).
        let targets = match a.text_of("name") {
            Some(n) if !n.is_empty() && !names.is_empty() => vec![self.group_under(n, &targets)],
            _ => targets,
        };
        let anim = match ty.as_str() {
            "rotate" | "knob" => {
                let (center, axis) = self.axis_of(g, a);
                Some(Anim::Rotate { center, axis, deg: anim_value(&self.pm, a, "deg") })
            }
            "translate" | "slider" => {
                let (_, axis) = self.axis_of(g, a);
                Some(Anim::Translate { axis, m: anim_value(&self.pm, a, "m") })
            }
            "spin" => {
                let (center, axis) = self.axis_of(g, a);
                let input = self.pm.input(a).unwrap_or(Expr::C(0.0));
                let rpm = op("mul", vec![input, Expr::C(a.f("factor").unwrap_or(1.0))]);
                Some(Anim::Spin { center, axis, rpm })
            }
            "select" => {
                let c = a.child("condition").map(|c| self.pm.condition(c)).unwrap_or(Expr::C(1.0));
                Some(Anim::Select(c))
            }
            "range" => {
                let lo = a.f("min-m").unwrap_or(0.0);
                let hi = a.f("max-m").unwrap_or(f32::INFINITY);
                // Keep the level of detail seen at a close chase distance.
                let vis = lo <= self.range_m && self.range_m < hi;
                Some(Anim::Select(Expr::C(if vis { 1.0 } else { 0.0 })))
            }
            "scale" => {
                let center = v3(a.child("center"), "-m");
                let input = self.pm.input(a);
                let one = |k: &str| {
                    let f = a.f(&format!("{k}-factor")).unwrap_or(0.0);
                    let o = a.f(&format!("{k}-offset")).unwrap_or(1.0);
                    let mut v = match &input {
                        Some(i) => op("add", vec![op("mul", vec![i.clone(), Expr::C(f)]), Expr::C(o)]),
                        None => Expr::C(o),
                    };
                    let lo = a.f(&format!("{k}-min"));
                    let hi = a.f(&format!("{k}-max"));
                    if lo.is_some() || hi.is_some() {
                        v = op("clamp", vec![v, Expr::C(lo.unwrap_or(-1e30)), Expr::C(hi.unwrap_or(1e30))]);
                    }
                    v
                };
                Some(Anim::Scale { center, s: [one("x"), one("y"), one("z")] })
            }
            "material" => {
                self.material_anim(a, &targets, dir);
                None
            }
            "light" => {
                for &t in &targets {
                    self.nodes[t].flags.hide = true;
                }
                None
            }
            "alpha-test" | "blend" => {
                for &t in &targets {
                    self.nodes[t].flags.translucent = true;
                }
                None
            }
            "billboard" | "noshadow" | "pick" | "touch" | "textranslate" | "texrotate" | "texmultiple" | "dist-scale" | "flash"
            | "locked-track" | "timed" | "trigger" | "interaction" | "shader" | "tooltip" => None,
            _ => None,
        };
        let Some(anim) = anim else { return };
        // Fold constant transforms / static selects right away.
        match &anim {
            Anim::Select(c) => match c.is_const() {
                Some(v) if v != 0.0 => return,
                Some(_) => {
                    for &t in &targets {
                        self.nodes[t].flags.hide = true;
                    }
                    return;
                }
                None => {}
            },
            Anim::Rotate { deg, .. } if deg.is_const() == Some(0.0) => return,
            Anim::Translate { m, .. } if m.is_const() == Some(0.0) => return,
            Anim::Spin { rpm, .. } if rpm.is_const() == Some(0.0) => return,
            _ => {}
        }
        for t in targets {
            self.wrap(t, anim.clone(), &format!("{}:{}", ty, self.nodes[t].name));
        }
    }

    fn material_anim(&mut self, a: &Elem, targets: &[usize], dir: &Path) {
        // Emission driven by a condition / factor property (light lenses, panel lighting).
        if let Some(em) = a.child("emission") {
            let col = Vec3::new(em.f("red").unwrap_or(0.0), em.f("green").unwrap_or(0.0), em.f("blue").unwrap_or(0.0));
            let mut k = em.text_of("factor-prop").map(|p| self.pm.prop(p)).unwrap_or(Expr::C(1.0));
            if let Some(f) = em.f("factor") {
                k = op("mul", vec![k, Expr::C(f)]);
            }
            if let Some(c) = a.child("condition") {
                k = op("mul", vec![k, self.pm.condition(c)]);
            }
            if col.length() > 0.0 && k.is_const() != Some(0.0) {
                for &t in targets {
                    self.nodes[t].flags.emit = Some((col, k.clone()));
                }
            }
        }
        // Static texture swap (e.g. a fixed livery part).
        if let Some(t) = a.text_of("texture") {
            if a.child("condition").is_none() {
                if let Some(p) = self.texture_path(t, dir) {
                    for &n in targets {
                        self.nodes[n].flags.texture = Some(p.clone());
                    }
                }
            }
        }
        if let Some(tp) = a.text_of("texture-prop") {
            // Livery: the config gives the value of the texture property.
            let key = tp.trim_start_matches('/');
            if let Some(v) = self.cfg.get("texture_props").and_then(|m| m.get(key)).and_then(|v| v.as_str()) {
                let base = a.text_of("property-base").unwrap_or("");
                let _ = base;
                if let Some(p) = self.texture_path(v, dir).or_else(|| self.ctx.resolve(v, dir)) {
                    for &n in targets {
                        self.nodes[n].flags.texture = Some(p.clone());
                    }
                }
            }
        }
    }
}

fn light_kind(s: &str) -> String {
    let s = s.to_lowercase();
    // Side: explicit words first, then a trailing -l / _r style suffix on the file / object name.
    let stem = s.rsplit(['/', ' ']).find(|t| !t.is_empty()).unwrap_or("").trim_end_matches(".xml").to_string();
    let right = s.contains("right") || s.contains("stbd") || s.contains("starboard") || s.contains("green")
        || stem.ends_with("_r") || stem.ends_with("-r") || stem.ends_with(".r");
    let left = !right && (s.contains("left") || s.contains("port") || s.contains("red")
        || stem.ends_with("_l") || stem.ends_with("-l") || stem.ends_with(".l"));
    let side = |l: &'static str, r: &'static str, c: &'static str| if right { r } else if left { l } else { c };
    let k = if s.contains("strobe") {
        if s.contains("tail") { "strobe_tail" } else { side("strobe_l", "strobe_r", "strobe") }
    } else if s.contains("beacon") {
        "beacon"
    } else if s.contains("nav") || s.contains("position") {
        if s.contains("tail") || s.contains("rear") || s.contains("white") { "nav_tail" } else { side("nav_l", "nav_r", "nav_tail") }
    } else if s.contains("landing") {
        "landing"
    } else if s.contains("taxi") {
        "taxi"
    } else if s.contains("tail") {
        "nav_tail"
    } else {
        "other"
    };
    k.to_string()
}

// ------------------------------------------------------------------ flatten / export

struct OutNode {
    parent: i32,
    pre: Mat4,
    anim: Option<Anim>,
    name: String,
}

#[derive(Clone, Copy, PartialEq)]
struct V {
    p: Vec3,
    n: Vec3,
    uv: Vec2,
}

#[derive(Default)]
struct ChunkBuild {
    verts: Vec<V>,
    idx: Vec<u32>,
}

#[derive(Clone, PartialEq)]
struct MatKey {
    tex: Option<PathBuf>,
    col: [u32; 4],
    emis: [u32; 3],
    spec: [u32; 3],
    shi: u32,
    translucent: bool,
    two_sided: bool,
}

struct Exporter {
    out_nodes: Vec<OutNode>,
    mats: Vec<MatKey>,
    /// (out node, material, interior, emit) → geometry
    chunks: Vec<((usize, usize, bool, Option<(Vec3, Expr)>), ChunkBuild)>,
    chunk_index: HashMap<String, usize>,
    lights: Vec<(String, Vec3, usize)>,
}

fn smooth_normals(tris: &[ac3d::Tri], crease_deg: f32) -> Vec<[Vec3; 3]> {
    let cos_c = crease_deg.to_radians().cos();
    let fn_: Vec<Vec3> = tris.iter().map(|t| (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]).normalize_or_zero()).collect();
    // Group corners by position.
    let key = |p: Vec3| ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64, (p.z * 1e4).round() as i64);
    let mut by_pos: HashMap<(i64, i64, i64), Vec<(usize, usize)>> = HashMap::new();
    for (ti, t) in tris.iter().enumerate() {
        for c in 0..3 {
            by_pos.entry(key(t.v[c])).or_default().push((ti, c));
        }
    }
    let mut out = vec![[Vec3::ZERO; 3]; tris.len()];
    for (ti, t) in tris.iter().enumerate() {
        for c in 0..3 {
            if !t.smooth {
                out[ti][c] = fn_[ti];
                continue;
            }
            let mut n = Vec3::ZERO;
            for &(tj, _) in &by_pos[&key(t.v[c])] {
                if tris[tj].smooth && fn_[tj].dot(fn_[ti]) >= cos_c {
                    // Area weighting.
                    let a = (tris[tj].v[1] - tris[tj].v[0]).cross(tris[tj].v[2] - tris[tj].v[0]).length();
                    n += fn_[tj] * a.max(1e-9);
                }
            }
            out[ti][c] = n.normalize_or(fn_[ti]);
        }
    }
    out
}

impl Exporter {
    fn walk(&mut self, b: &Builder, id: usize, out_parent: usize, acc: Mat4, inherited: &Flags) {
        let n = &b.nodes[id];
        let mut flags = inherited.clone();
        if n.flags.hide {
            return;
        }
        flags.translucent |= n.flags.translucent;
        flags.interior |= n.flags.interior;
        if n.flags.emit.is_some() {
            flags.emit = n.flags.emit.clone();
        }
        if n.flags.texture.is_some() {
            flags.texture = n.flags.texture.clone();
        }
        let mut acc = acc * n.local;
        let mut cur = out_parent;
        if let Some(anim) = &n.anim {
            self.out_nodes.push(OutNode { parent: out_parent as i32, pre: acc, anim: Some(anim.clone()), name: n.name.clone() });
            cur = self.out_nodes.len() - 1;
            acc = Mat4::IDENTITY;
        }
        if let Some(kind) = &n.light {
            self.lights.push((kind.clone(), acc.transform_point3(Vec3::ZERO), cur));
        }
        if let (Some(g), Ok(re)) = (&n.geom, std::env::var("FGIMPORT_DUMP")) {
            if Regex::new(&re).map(|r| r.is_match(&n.name)).unwrap_or(false) {
                // Bounding box in the model frame (static pose; animated parents at rest).
                let (mut lo, mut hi) = (Vec3::splat(1e9), Vec3::splat(-1e9));
                for t in &g.obj_tris {
                    for v in t.v {
                        let p = acc.transform_point3(v);
                        lo = lo.min(p);
                        hi = hi.max(p);
                    }
                }
                eprintln!("DUMP {:<28} model lo {:>7.3} {:>7.3} {:>7.3}  hi {:>7.3} {:>7.3} {:>7.3}", n.name, lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
            }
        }
        if let Some(g) = &n.geom {
            let normals = smooth_normals(&g.obj_tris, g.crease);
            let nm = Mat3::from_mat4(acc).inverse().transpose();
            for (t, nr) in g.obj_tris.iter().zip(normals) {
                let m = g.materials.get(t.mat).cloned().unwrap_or(AcMaterial {
                    name: String::new(),
                    rgb: Vec3::ONE,
                    amb: Vec3::ONE,
                    emis: Vec3::ZERO,
                    spec: Vec3::ZERO,
                    shi: 10.0,
                    trans: 0.0,
                });
                let tex = flags.texture.clone().or_else(|| g.texture.clone());
                let q = |v: f32| (v * 1000.0).round() as u32;
                let key = MatKey {
                    tex: tex.clone(),
                    col: [q(m.rgb.x), q(m.rgb.y), q(m.rgb.z), q(1.0 - m.trans)],
                    emis: [q(m.emis.x), q(m.emis.y), q(m.emis.z)],
                    spec: [q(m.spec.x), q(m.spec.y), q(m.spec.z)],
                    shi: q(m.shi),
                    translucent: flags.translucent || m.trans > 0.01,
                    two_sided: t.two_sided,
                };
                let mi = match self.mats.iter().position(|k| *k == key) {
                    Some(i) => i,
                    None => {
                        self.mats.push(key);
                        self.mats.len() - 1
                    }
                };
                let ck = format!("{cur}/{mi}/{}/{:?}", flags.interior, flags.emit.as_ref().map(|(c, e)| (c.to_array(), format!("{e:?}"))));
                let ci = match self.chunk_index.get(&ck) {
                    Some(&i) => i,
                    None => {
                        self.chunks.push(((cur, mi, flags.interior, flags.emit.clone()), ChunkBuild::default()));
                        self.chunk_index.insert(ck, self.chunks.len() - 1);
                        self.chunks.len() - 1
                    }
                };
                let cb = &mut self.chunks[ci].1;
                for c in 0..3 {
                    let p = acc.transform_point3(t.v[c]);
                    let nn = (nm * nr[c]).normalize_or_zero();
                    cb.idx.push(cb.verts.len() as u32);
                    cb.verts.push(V { p, n: nn, uv: Vec2::new(t.uv[c].x, 1.0 - t.uv[c].y) });
                }
            }
        }
        let kids = n.kids.clone();
        for k in kids {
            self.walk(b, k, cur, acc, &flags);
        }
    }
}

/// Weld identical vertices within each chunk.
fn weld(cb: &ChunkBuild) -> (Vec<V>, Vec<u32>) {
    let mut map: HashMap<[u32; 8], u32> = HashMap::new();
    let mut verts = Vec::new();
    let mut idx = Vec::with_capacity(cb.idx.len());
    for &i in &cb.idx {
        let v = cb.verts[i as usize];
        let k = [v.p.x.to_bits(), v.p.y.to_bits(), v.p.z.to_bits(), v.n.x.to_bits(), v.n.y.to_bits(), v.n.z.to_bits(), v.uv.x.to_bits(), v.uv.y.to_bits()];
        let j = *map.entry(k).or_insert_with(|| {
            verts.push(v);
            (verts.len() - 1) as u32
        });
        idx.push(j);
    }
    (verts, idx)
}

fn anim_json(a: &Anim) -> Value {
    let v = |x: Vec3| json!([x.x, x.y, x.z]);
    match a {
        Anim::Rotate { center, axis, deg } => json!({"t": "rot", "c": v(*center), "a": v(*axis), "v": deg.to_json()}),
        Anim::Translate { axis, m } => json!({"t": "tr", "a": v(*axis), "v": m.to_json()}),
        Anim::Spin { center, axis, rpm } => json!({"t": "spin", "c": v(*center), "a": v(*axis), "v": rpm.to_json()}),
        Anim::Select(c) => json!({"t": "sel", "v": c.to_json()}),
        Anim::Scale { center, s } => json!({"t": "scale", "c": v(*center), "v": [s[0].to_json(), s[1].to_json(), s[2].to_json()]}),
    }
}

fn oct_encode(n: Vec3) -> [i16; 2] {
    let n = n / (n.x.abs() + n.y.abs() + n.z.abs()).max(1e-12);
    let (mut x, mut y) = (n.x, n.y);
    if n.z < 0.0 {
        let (ox, oy) = (x, y);
        x = (1.0 - oy.abs()) * if ox >= 0.0 { 1.0 } else { -1.0 };
        y = (1.0 - ox.abs()) * if oy >= 0.0 { 1.0 } else { -1.0 };
    }
    [(x.clamp(-1.0, 1.0) * 32767.0).round() as i16, (y.clamp(-1.0, 1.0) * 32767.0).round() as i16]
}

pub fn run(cfg: &Value, src: &Path, out: &Path) -> Result<(), String> {
    let fg_name = cfg["fg_name"].as_str().ok_or("fg_name")?;
    // Start clean: drop textures from a previous import.
    if let Ok(rd) = std::fs::read_dir(out) {
        for e in rd.flatten() {
            let n = e.file_name().to_string_lossy().to_string();
            if n.starts_with('t') && (n.ends_with(".png") || n.ends_with(".jpg")) && n[1..n.len() - 4].chars().all(|c| c.is_ascii_digit()) {
                let _ = std::fs::remove_file(e.path());
            }
        }
    }
    let ctx = Ctx { root: src.to_path_buf(), prefix: format!("Aircraft/{fg_name}/") };
    let mut statics = HashMap::new();
    if let Some(m) = cfg.get("props").and_then(|v| v.as_object()) {
        for (k, v) in m {
            statics.insert(k.trim_start_matches('/').to_string(), v.as_f64().unwrap_or(0.0) as f32);
        }
    }
    let extra: Vec<(String, String, f32, f32)> = cfg
        .get("channels")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|r| {
                    let r = r.as_array()?;
                    Some((
                        r.first()?.as_str()?.to_string(),
                        r.get(1)?.as_str()?.to_string(),
                        r.get(2).and_then(|x| x.as_f64()).unwrap_or(1.0) as f32,
                        r.get(3).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32,
                    ))
                })
                .collect()
        })
        .unwrap_or_default();
    let tex_override: HashMap<String, String> = cfg
        .get("textures")
        .and_then(|v| v.as_object())
        .map(|m| m.iter().map(|(k, v)| (k.clone(), v.as_str().unwrap_or("").to_string())).collect())
        .unwrap_or_default();
    let mut b = Builder {
        nodes: Vec::new(),
        ctx,
        pm: PropMap::new(statics, &extra),
        cfg,
        exclude_models: regexes(cfg, "exclude_models"),
        exclude_objects: regexes(cfg, "exclude_objects"),
        interior_models: regexes(cfg, "interior_models"),
        interior_objects: regexes(cfg, "interior_objects"),
        light_models: regexes(cfg, "light_models"),
        tex_override,
        range_m: cfg.get("range_m").and_then(|v| v.as_f64()).unwrap_or(15.0) as f32,
        log: Vec::new(),
        anim_counts: HashMap::new(),
    };

    // ---- FDM: CG / VRP → body frame
    let mut fdm_report = json!({});
    let (cg_model, eye_default) = if let Some(f) = cfg["fdm"].get("jsbsim").and_then(|v| v.as_str()) {
        let p = src.join(f);
        let mut x = xml::load(&b.ctx, &p)?;
        expand_jsb_files(&b.ctx, &mut x, p.parent().unwrap());
        let j = fdm::read(&x, cfg);
        // The runtime aerodynamics use the same loaded CG.
        x.kids.push(xml::Elem { name: "skyward_cg".into(), attrs: vec![("unit".into(), "M".into())], kids: vec![
            xml::Elem { name: "x".into(), text: j.cg.x.to_string(), ..Default::default() },
            xml::Elem { name: "y".into(), text: j.cg.y.to_string(), ..Default::default() },
            xml::Elem { name: "z".into(), text: j.cg.z.to_string(), ..Default::default() },
        ], ..Default::default() });
        std::fs::write(out.join("fdm.xml"), fdm_extract(&x)).map_err(|e| e.to_string())?;
        fdm_report = j.report.clone();
        let cg_model = j.cg - j.vrp;
        let eye = fdm_report["eyepoint_body"].as_array().map(|a| Vec3::new(a[0].as_f64().unwrap() as f32, a[1].as_f64().unwrap() as f32, a[2].as_f64().unwrap() as f32));
        (cg_model, eye)
    } else {
        let a = cfg["fdm"]["cg_model"].as_array().ok_or("fdm.cg_model")?;
        (Vec3::new(a[0].as_f64().unwrap() as f32, a[1].as_f64().unwrap() as f32, a[2].as_f64().unwrap() as f32), None)
    };
    // model (x aft, y right, z up) → body (+X left, +Y up, +Z nose), CG origin.
    let to_body = Mat4::from_cols(
        glam::Vec4::new(0.0, 0.0, -1.0, 0.0),
        glam::Vec4::new(-1.0, 0.0, 0.0, 0.0),
        glam::Vec4::new(0.0, 1.0, 0.0, 0.0),
        glam::Vec4::W,
    ) * Mat4::from_translation(-cg_model);

    let root = b.group("root", Mat4::IDENTITY, None);
    let entry = src.join(cfg["model"].as_str().ok_or("model")?);
    b.load_model(&entry, "", root, 0)?;

    // ---- flatten
    let mut ex = Exporter { out_nodes: Vec::new(), mats: Vec::new(), chunks: Vec::new(), chunk_index: HashMap::new(), lights: Vec::new() };
    ex.out_nodes.push(OutNode { parent: -1, pre: to_body, anim: None, name: "root".into() });
    ex.walk(&b, root, 0, Mat4::IDENTITY, &Flags::default());

    // Prune animation nodes without geometry below them.
    let n_out = ex.out_nodes.len();
    let mut used = vec![false; n_out];
    for ((node, ..), _) in &ex.chunks {
        let mut i = *node as i32;
        while i >= 0 && !used[i as usize] {
            used[i as usize] = true;
            i = ex.out_nodes[i as usize].parent;
        }
    }
    for (_, _, node) in &ex.lights {
        let mut i = *node as i32;
        while i >= 0 && !used[i as usize] {
            used[i as usize] = true;
            i = ex.out_nodes[i as usize].parent;
        }
    }
    let mut remap = vec![-1i32; n_out];
    let mut nodes_json = Vec::new();
    let mut channels: Vec<String> = Vec::new();
    for i in 0..n_out {
        if !used[i] {
            continue;
        }
        remap[i] = nodes_json.len() as i32;
        let n = &ex.out_nodes[i];
        let parent = if n.parent >= 0 { remap[n.parent as usize] } else { -1 };
        if let Some(a) = &n.anim {
            match a {
                Anim::Rotate { deg: e, .. } | Anim::Translate { m: e, .. } | Anim::Spin { rpm: e, .. } | Anim::Select(e) => e.channels(&mut channels),
                Anim::Scale { s, .. } => s.iter().for_each(|e| e.channels(&mut channels)),
            }
        }
        nodes_json.push(json!({
            "name": n.name,
            "parent": parent,
            "pre": n.pre.to_cols_array().to_vec(),
            "anim": n.anim.as_ref().map(anim_json),
        }));
    }

    // ---- textures
    let max_tex = cfg.get("max_tex").and_then(|v| v.as_u64()).unwrap_or(2048) as u32;
    let interior_max = cfg.get("interior_max_tex").and_then(|v| v.as_u64()).unwrap_or(1024) as u32;
    let jpeg_q = cfg.get("jpeg_quality").and_then(|v| v.as_u64()).unwrap_or(85) as u8;
    // Which textures are used only by interior chunks.
    let mut tex_interior: HashMap<PathBuf, bool> = HashMap::new();
    for ((_, mi, interior, _), _) in &ex.chunks {
        if let Some(t) = &ex.mats[*mi].tex {
            let e = tex_interior.entry(t.clone()).or_insert(true);
            *e &= *interior;
        }
    }
    let mut tex_list: Vec<PathBuf> = Vec::new();
    let mut tex_json = Vec::new();
    let mut tex_alpha: Vec<bool> = Vec::new();
    let mut tex_bytes = 0usize;
    let mut tex_index: HashMap<PathBuf, usize> = HashMap::new();
    for k in &ex.mats {
        if let Some(t) = &k.tex {
            if tex_index.contains_key(t) {
                continue;
            }
            let lim = if tex_interior.get(t).copied().unwrap_or(false) { interior_max } else { max_tex };
            let i = tex_list.len();
            match convert_texture(t, out, i, lim, jpeg_q) {
                Ok((file, alpha, bytes)) => {
                    tex_bytes += bytes;
                    tex_index.insert(t.clone(), i);
                    tex_list.push(t.clone());
                    tex_alpha.push(alpha);
                    tex_json.push(json!({"file": file, "alpha": alpha, "src": b.rel(t)}));
                }
                Err(e) => b.log.push(format!("texture {}: {e}", t.display())),
            }
        }
    }
    let mats_json: Vec<Value> = ex
        .mats
        .iter()
        .map(|k| {
            let f = |v: u32| v as f32 / 1000.0;
            let tex = k.tex.as_ref().and_then(|t| tex_index.get(t)).map(|&i| i as i64).unwrap_or(-1);
            let alpha_tex = tex >= 0 && tex_alpha[tex as usize];
            json!({
                "col": [f(k.col[0]), f(k.col[1]), f(k.col[2]), f(k.col[3])],
                "emis": [f(k.emis[0]), f(k.emis[1]), f(k.emis[2])],
                "spec": [f(k.spec[0]), f(k.spec[1]), f(k.spec[2])],
                "shi": f(k.shi),
                "tex": tex,
                "translucent": k.translucent,
                "two_sided": k.two_sided,
                "alpha_tex": alpha_tex,
            })
        })
        .collect();

    // ---- geometry blob
    let mut vblob: Vec<u8> = Vec::new();
    let mut iblob: Vec<u8> = Vec::new();
    let mut chunks_json = Vec::new();
    let (mut nv, mut ni) = (0u32, 0u32);
    let mut radius = 0.0f32;
    let mut tri_ext = 0usize;
    let mut tri_int = 0usize;
    for ((node, mi, interior, emit), cb) in &ex.chunks {
        let (verts, idx) = weld(cb);
        let node = remap[*node];
        for v in &verts {
            vblob.extend_from_slice(bytemuck_f32(&[v.p.x, v.p.y, v.p.z]));
            for c in oct_encode(v.n) {
                vblob.extend_from_slice(&c.to_le_bytes());
            }
            vblob.extend_from_slice(bytemuck_f32(&[v.uv.x, v.uv.y]));
        }
        for i in &idx {
            iblob.extend_from_slice(&i.to_le_bytes());
        }
        if let Some((_, e)) = emit {
            e.channels(&mut channels);
        }
        if *interior {
            tri_int += idx.len() / 3;
        } else {
            tri_ext += idx.len() / 3;
        }
        chunks_json.push(json!({
            "node": node, "mat": mi, "base_vertex": nv, "first": ni, "count": idx.len(),
            "vertex_count": verts.len(),
            "interior": interior,
            "emit": emit.as_ref().map(|(c, e)| json!({"col": [c.x, c.y, c.z], "v": e.to_json()})),
        }));
        nv += verts.len() as u32;
        ni += idx.len() as u32;
    }
    // Bounding radius in body frame (approximate: static pose).
    {
        let mut world = vec![Mat4::IDENTITY; nodes_json.len()];
        for (i, n) in nodes_json.iter().enumerate() {
            let pre = Mat4::from_cols_slice(&n["pre"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap() as f32).collect::<Vec<_>>());
            let p = n["parent"].as_i64().unwrap();
            world[i] = if p >= 0 { world[p as usize] * pre } else { pre };
        }
        let mut off = 0usize;
        for (k, ((node, _, interior, _), _)) in ex.chunks.iter().enumerate() {
            let cnt = chunks_json[k]["vertex_count"].as_u64().unwrap() as usize;
            if !interior {
                let m = world[remap[*node] as usize];
                for j in 0..cnt {
                    let o = (off + j) * 24;
                    let x = f32::from_le_bytes(vblob[o..o + 4].try_into().unwrap());
                    let y = f32::from_le_bytes(vblob[o + 4..o + 8].try_into().unwrap());
                    let z = f32::from_le_bytes(vblob[o + 8..o + 12].try_into().unwrap());
                    radius = radius.max(m.transform_point3(Vec3::new(x, y, z)).length());
                }
            }
            off += cnt;
        }
    }

    // Lights in body frame (static pose).
    let mut lights_json = Vec::new();
    {
        let mut world = vec![Mat4::IDENTITY; n_out];
        for i in 0..n_out {
            let n = &ex.out_nodes[i];
            world[i] = if n.parent >= 0 { world[n.parent as usize] * n.pre } else { n.pre };
        }
        for (kind, p, node) in &ex.lights {
            let w = world[*node].transform_point3(*p);
            lights_json.push(json!({"kind": kind, "pos": [w.x, w.y, w.z], "node": remap[*node]}));
        }
    }
    let eye = if let Some(a) = cfg.get("eye_body").and_then(|v| v.as_array()) {
        let f = |i: usize| a[i].as_f64().unwrap() as f32;
        Some(Vec3::new(f(0), f(1), f(2)))
    } else if let Some(a) = cfg.get("eye_view").and_then(|v| v.as_array()) {
        // FlightGear view offsets: x right, y up, z aft (model frame: x aft, y right, z up).
        let f = |i: usize| a[i].as_f64().unwrap() as f32;
        Some(to_body.transform_point3(Vec3::new(f(2), f(0), f(1))))
    } else if let Some(a) = cfg.get("eye_model").and_then(|v| v.as_array()) {
        let m = Vec3::new(a[0].as_f64().unwrap() as f32, a[1].as_f64().unwrap() as f32, a[2].as_f64().unwrap() as f32);
        Some(to_body.transform_point3(m))
    } else {
        eye_default
    };
    channels.sort();
    let header = json!({
        "format": 1,
        "id": cfg["id"],
        "name": cfg["name"],
        "eye": eye.map(|e| vec![e.x, e.y, e.z]),
        "radius": radius,
        "nodes": nodes_json,
        "materials": mats_json,
        "textures": tex_json,
        "chunks": chunks_json,
        "lights": lights_json,
        "vertex_count": nv,
        "index_count": ni,
        "channels": channels,
    });
    let hs = serde_json::to_vec(&header).unwrap();
    let mut payload = Vec::new();
    payload.extend_from_slice(&(hs.len() as u32).to_le_bytes());
    payload.extend_from_slice(&hs);
    payload.extend_from_slice(&vblob);
    payload.extend_from_slice(&iblob);
    let mut enc = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::best());
    enc.write_all(&payload).unwrap();
    let z = enc.finish().unwrap();
    let mut file = b"SKAC".to_vec();
    file.extend_from_slice(&1u32.to_le_bytes());
    file.extend_from_slice(&z);
    std::fs::write(out.join("model.skac"), &file).map_err(|e| e.to_string())?;
    std::fs::write(out.join("fdm.json"), serde_json::to_string_pretty(&fdm_report).unwrap()).map_err(|e| e.to_string())?;

    // ---- report
    let mut rep = String::new();
    rep += &format!("{}: {} nodes ({} animated out of {}), {} materials, {} textures ({:.1} MB), {} chunks\n", cfg["id"], nodes_json.len(), nodes_json.len() - 1, n_out, mats_json.len(), tex_json.len(), tex_bytes as f64 / 1e6, chunks_json.len());
    rep += &format!("triangles: exterior {tri_ext}, interior {tri_int}; vertices {nv}; model.skac {:.1} MB; radius {radius:.1} m\n", file.len() as f64 / 1e6);
    rep += &format!("eye: {:?}\nlights: {}\n", eye, lights_json.len());
    rep += &format!("channels used: {}\n", channels.join(" "));
    let mut ac: Vec<_> = b.anim_counts.iter().collect();
    ac.sort();
    rep += &format!("animation types: {ac:?}\n");
    rep += "unmapped properties (count):\n";
    let mut um: Vec<_> = b.pm.unmapped.borrow().iter().map(|(k, v)| (*v, k.clone())).collect();
    um.sort_by(|a, b| b.cmp(a));
    for (c, k) in um.iter().take(60) {
        rep += &format!("  {c:4} {k}\n");
    }
    rep += "log:\n";
    for l in &b.log {
        rep += &format!("  {l}\n");
    }
    std::fs::write(out.join("import-report.txt"), &rep).map_err(|e| e.to_string())?;
    print!("{rep}");
    Ok(())
}

fn bytemuck_f32(v: &[f32]) -> &[u8] {
    unsafe { std::slice::from_raw_parts(v.as_ptr() as *const u8, v.len() * 4) }
}

/// JSBSim `<section file="x"/>` includes (relative to the FDM dir, ".xml" optional);
/// engine / thruster definitions are inlined from `Engines/`.
fn expand_jsb_files(ctx: &Ctx, x: &mut Elem, dir: &Path) {
    let find = |f: &str, extra: &[&str]| -> Option<PathBuf> {
        let mut c = vec![dir.join(f), dir.join(format!("{f}.xml")), ctx.root.join(f), ctx.root.join(format!("{f}.xml"))];
        for e in extra {
            c.push(ctx.root.join(e).join(format!("{f}.xml")));
            c.push(dir.join(e).join(format!("{f}.xml")));
        }
        c.into_iter().find(|p| p.is_file())
    };
    for k in &mut x.kids {
        if k.name == "propulsion" {
            if let Some(f) = k.attr("file").map(|s| s.to_string()) {
                if let Some(p) = find(&f, &[]) {
                    if let Ok(inc) = xml::load(ctx, &p) {
                        k.kids.extend(inc.kids);
                    }
                }
            }
            for e in &mut k.kids {
                if e.name != "engine" {
                    continue;
                }
                if let Some(f) = e.attr("file").map(|s| s.to_string()) {
                    if let Some(p) = find(&f, &["Engines", "engine"]) {
                        if let Ok(inc) = xml::load(ctx, &p) {
                            e.kids.push(inc);
                        }
                    }
                }
                for t in &mut e.kids {
                    if t.name == "thruster" {
                        if let Some(f) = t.attr("file").map(|s| s.to_string()) {
                            if let Some(p) = find(&f, &["Engines", "engine"]) {
                                if let Ok(inc) = xml::load(ctx, &p) {
                                    t.kids.push(inc);
                                }
                            }
                        }
                    }
                }
            }
            continue;
        }
        if let Some(f) = k.attr("file").map(|s| s.to_string()) {
            if let Some(p) = find(&f, &[]) {
                if let Ok(inc) = xml::load(ctx, &p) {
                    k.kids.extend(inc.kids);
                }
            }
        }
    }
}

/// The FDM sections the flight model reads (aero + geometry + engines), merged.
fn fdm_extract(x: &Elem) -> String {
    let mut out = String::from("<?xml version=\"1.0\"?>\n<!-- Extracted by tools/fgimport from a GPL FlightGear JSBSim FDM; see CREDITS.md. -->\n");
    let mut root = Elem { name: "fdm_config".into(), attrs: x.attrs.clone(), ..Default::default() };
    for k in &x.kids {
        if ["metrics", "mass_balance", "ground_reactions", "propulsion", "aerodynamics", "skyward_cg"].contains(&k.name.as_str()) {
            root.kids.push(k.clone());
        }
    }
    root.to_xml(&mut out, 0);
    out
}

/// Load, downsize and re-encode a texture. Returns (file name, has alpha, bytes).
fn convert_texture(src: &Path, out: &Path, i: usize, max: u32, q: u8) -> Result<(String, bool, usize), String> {
    let mut p = src.to_path_buf();
    let ext = p.extension().map(|e| e.to_string_lossy().to_lowercase()).unwrap_or_default();
    if ["rgb", "rgba", "sgi", "bw", "dds"].contains(&ext.as_str()) {
        p = PathBuf::from(format!("{}.png", src.display()));
    }
    let img = image::open(&p).map_err(|e| e.to_string())?;
    let mut img = img.to_rgba8();
    let (w, h) = img.dimensions();
    let s = (max as f32 / w.max(h) as f32).min(1.0);
    if s < 1.0 {
        let nw = ((w as f32 * s).round() as u32).max(1);
        let nh = ((h as f32 * s).round() as u32).max(1);
        img = image::imageops::resize(&img, nw, nh, image::imageops::FilterType::Lanczos3);
    }
    let alpha = img.pixels().any(|p| p.0[3] < 250);
    let (name, bytes) = if alpha {
        let name = format!("t{i}.png");
        img.save(out.join(&name)).map_err(|e| e.to_string())?;
        let n = std::fs::metadata(out.join(&name)).map(|m| m.len() as usize).unwrap_or(0);
        (name, n)
    } else {
        let name = format!("t{i}.jpg");
        let rgb = image::DynamicImage::ImageRgba8(img).to_rgb8();
        let f = std::fs::File::create(out.join(&name)).map_err(|e| e.to_string())?;
        let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(std::io::BufWriter::new(f), q);
        enc.encode_image(&rgb).map_err(|e| e.to_string())?;
        let n = std::fs::metadata(out.join(&name)).map(|m| m.len() as usize).unwrap_or(0);
        (name, n)
    };
    Ok((name, alpha, bytes))
}
