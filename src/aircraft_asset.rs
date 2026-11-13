//! Imported aircraft models (`assets/aircraft/<id>/model.skac`, produced by
//! `tools/fgimport` from FlightGear aircraft) and their animation runtime.
//!
//! File: `"SKAC"`, u32 version, then zlib( u32 json length, JSON header,
//! vertices (pos f32×3, octahedral normal i16×2, uv f32×2 = 24 bytes), u32 indices ).
//! Version 2 adds per-chunk texture animations (`uv`: FlightGear
//! `textranslate` / `texrotate`, drum counters, attitude balls) and glass
//! display screens (`display`); version-1 files still load.
//!
//! The header holds a node tree in the body frame (+Z nose, +Y up, +X left,
//! CG origin). Each node has a static `pre` transform (relative to its
//! parent) followed by an optional animation:
//! * `rot`  — rotation about axis `a` through centre `c` by `v` degrees
//!   (right-handed, FlightGear's convention);
//! * `tr`   — translation along unit axis `a` by `v` metres;
//! * `spin` — continuous rotation at `v` rpm (integrated per instance);
//! * `sel`  — visible when `v` ≠ 0;
//! * `scale`— per-axis scale about `c`.
//!
//! Expressions read named *channels* (FlightGear semantics): `elevator`
//! (+1 = trailing edge down, i.e. stick forward), `aileron_l` (+1 = roll
//! right: left aileron TE down), `aileron_r` (= −aileron), `rudder` (+1 =
//! yaw right), `flaps`, `spoilers` 0..1, `gear_pos` (1 = down), per-gear
//! `gear{i}_comp_m` / `gear{i}_comp_norm`, per-engine `eng{i}_rpm` / `eng{i}_n1`
//! (percent) / `prop{i}_rpm` / `eng{i}_reverser`, `afterburner`, cockpit
//! controls `ctl_*`, lights `nav` / `beacon` / `strobe` / `landing_light`,
//! instruments `ias_kt` / `alt_ft` / `pitch_deg` / `roll_deg` /
//! `heading_deg` / `vs_fpm` / `turn_rate` / `slip`, and `time`; the cockpit
//! instrument channels are listed in `src/cockpit/channels.rs`.

use glam::{Mat4, Quat, Vec3, Vec4};
use serde_json::Value;
use std::io::Read;

#[derive(Clone, Debug)]
pub enum RExpr {
    C(f32),
    Ch(u16),
    Op(Op, Vec<RExpr>),
    Interp(Box<RExpr>, Vec<(f32, f32)>),
}

#[derive(Clone, Copy, Debug)]
pub enum Op {
    Add,
    Mul,
    Sub,
    Div,
    Min,
    Max,
    Clamp,
    Abs,
    Neg,
    Not,
    And,
    Or,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    Sin,
    Cos,
    Floor,
    Mod,
    Pow,
    Sqrt,
    /// FlightGear step / scroll quantisation (drum counters): (x, step, scroll).
    Step,
}

impl RExpr {
    /// Channel indices this expression reads.
    pub fn channel_ids(&self, out: &mut Vec<u16>) {
        match self {
            RExpr::Ch(i) => out.push(*i),
            RExpr::Op(_, a) => a.iter().for_each(|e| e.channel_ids(out)),
            RExpr::Interp(x, _) => x.channel_ids(out),
            RExpr::C(_) => {}
        }
    }

    pub fn eval(&self, ch: &[f32]) -> f32 {
        match self {
            RExpr::C(v) => *v,
            RExpr::Ch(i) => ch.get(*i as usize).copied().unwrap_or(0.0),
            RExpr::Interp(x, t) => interp(t, x.eval(ch)),
            RExpr::Op(o, a) => {
                let g = |i: usize| a.get(i).map(|e| e.eval(ch)).unwrap_or(0.0);
                let b = |x: bool| if x { 1.0 } else { 0.0 };
                match o {
                    Op::Add => a.iter().map(|e| e.eval(ch)).sum(),
                    Op::Mul => a.iter().map(|e| e.eval(ch)).product(),
                    Op::Sub => g(0) - a.iter().skip(1).map(|e| e.eval(ch)).sum::<f32>(),
                    Op::Div => {
                        let d = g(1);
                        if d.abs() < 1e-12 { 0.0 } else { g(0) / d }
                    }
                    Op::Min => a.iter().map(|e| e.eval(ch)).fold(f32::INFINITY, f32::min),
                    Op::Max => a.iter().map(|e| e.eval(ch)).fold(f32::NEG_INFINITY, f32::max),
                    Op::Clamp => g(0).max(g(1)).min(g(2)),
                    Op::Abs => g(0).abs(),
                    Op::Neg => -g(0),
                    Op::Not => b(g(0) == 0.0),
                    Op::And => b(a.iter().all(|e| e.eval(ch) != 0.0)),
                    Op::Or => b(a.iter().any(|e| e.eval(ch) != 0.0)),
                    Op::Lt => b(g(0) < g(1)),
                    Op::Le => b(g(0) <= g(1)),
                    Op::Gt => b(g(0) > g(1)),
                    Op::Ge => b(g(0) >= g(1)),
                    Op::Eq => b((g(0) - g(1)).abs() < 1e-6),
                    Op::Ne => b((g(0) - g(1)).abs() >= 1e-6),
                    Op::Sin => g(0).sin(),
                    Op::Cos => g(0).cos(),
                    Op::Floor => g(0).floor(),
                    Op::Mod => {
                        let d = g(1);
                        if d.abs() < 1e-12 { 0.0 } else { g(0).rem_euclid(d) }
                    }
                    Op::Pow => g(0).powf(g(1)),
                    Op::Sqrt => g(0).max(0.0).sqrt(),
                    Op::Step => step_scroll(g(0), g(1), g(2)),
                }
            }
        }
    }
}

/// FlightGear `SGStepExpression`: quantise `v` to `step`; with `scroll` > 0
/// the value rolls into the next step over the last `scroll` units of each
/// step (a drum counter's carry).
pub fn step_scroll(v: f32, step: f32, scroll: f32) -> f32 {
    if step <= 0.0 {
        return v;
    }
    let mut m = (v / step).floor() * step;
    if scroll > 0.0 {
        let rem = step - v.abs() % step;
        if rem < scroll {
            m += (scroll - rem) / scroll * step;
        }
    }
    m
}

fn interp(t: &[(f32, f32)], x: f32) -> f32 {
    if t.is_empty() {
        return 0.0;
    }
    if x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        if x <= w[1].0 {
            let d = w[1].0 - w[0].0;
            let f = if d.abs() < 1e-12 { 1.0 } else { (x - w[0].0) / d };
            return w[0].1 + f * (w[1].1 - w[0].1);
        }
    }
    t[t.len() - 1].1
}

#[derive(Clone, Debug)]
pub enum Anim {
    Rotate { center: Vec3, axis: Vec3, deg: RExpr },
    Translate { axis: Vec3, m: RExpr },
    Spin { center: Vec3, axis: Vec3, rpm: RExpr },
    Select(RExpr),
    Scale { center: Vec3, s: [RExpr; 3] },
}

#[derive(Clone, Debug)]
pub struct Node {
    pub name: String,
    pub parent: i32,
    pub pre: Mat4,
    pub anim: Option<Anim>,
}

#[derive(Clone, Debug)]
pub struct Material {
    /// Diffuse colour (linear-ish AC3D values) and opacity.
    pub col: Vec4,
    pub emis: Vec3,
    pub spec: Vec3,
    pub shininess: f32,
    pub tex: Option<usize>,
    /// Blended / see-through (canopies, windows, prop discs).
    pub translucent: bool,
    /// The texture has an alpha channel (cut-outs).
    pub alpha_tex: bool,
    /// Drawn from both sides (otherwise back faces are culled, as in FlightGear:
    /// the cockpit view looks out through the one-sided fuselage skin).
    pub two_sided: bool,
}

pub struct Texture {
    pub width: u32,
    pub height: u32,
    /// RGBA8, sRGB-encoded colour.
    pub rgba: Vec<u8>,
}

/// Texture-coordinate animation of a chunk (UV space, v down).
#[derive(Clone, Debug)]
pub enum UvAnim {
    Translate { axis: glam::Vec2, v: RExpr },
    /// Counter-clockwise (in u, v) rotation about `center` by `v` degrees.
    Rotate { center: glam::Vec2, v: RExpr },
}

/// A glass display shown on a chunk.
#[derive(Clone, Debug)]
pub struct DisplayScreen {
    /// What it shows: `pfd`, `nd`, `ewd`, `sd`, `isis`, `mcdu`.
    pub kind: String,
    /// The screen's window in the chunk's UVs: (u0, v0, u1, v1).
    pub uv: [f32; 4],
    /// Physical width / height of that window.
    pub aspect: f32,
}

#[derive(Clone, Debug)]
pub struct Chunk {
    pub node: usize,
    pub material: usize,
    pub base_vertex: u32,
    pub first: u32,
    pub count: u32,
    /// Cockpit / cabin geometry (drawn only when the camera is close).
    pub interior: bool,
    /// Animated emission: colour × expression.
    pub emit: Option<(Vec3, RExpr)>,
    /// Texture animations, innermost first.
    pub uv: Vec<UvAnim>,
    /// Glass display screen.
    pub display: Option<DisplayScreen>,
    /// The emission is cockpit backlighting (driven by the panel lighting),
    /// even on geometry not flagged interior.
    pub backlight: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub nrm: [f32; 3],
    pub uv: [f32; 2],
}

#[derive(Clone, Debug)]
pub struct Light {
    pub kind: String,
    pub pos: Vec3,
    pub node: Option<usize>,
}

pub struct AircraftAsset {
    pub id: String,
    pub name: String,
    pub nodes: Vec<Node>,
    pub materials: Vec<Material>,
    pub textures: Vec<Texture>,
    pub chunks: Vec<Chunk>,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub eye: Option<Vec3>,
    pub radius: f32,
    pub lights: Vec<Light>,
    /// Channel names, indexed by `RExpr::Ch`.
    pub channels: Vec<String>,
}

fn vec3(v: &Value) -> Vec3 {
    let a = v.as_array();
    let f = |i: usize| a.and_then(|a| a.get(i)).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32;
    Vec3::new(f(0), f(1), f(2))
}

fn expr(v: &Value, chans: &[String]) -> RExpr {
    if let Some(x) = v.as_f64() {
        return RExpr::C(x as f32);
    }
    if let Some(c) = v.get("ch").and_then(|c| c.as_str()) {
        return match chans.iter().position(|n| n == c) {
            Some(i) => RExpr::Ch(i as u16),
            None => RExpr::C(0.0),
        };
    }
    let a: Vec<RExpr> = v.get("a").and_then(|a| a.as_array()).map(|a| a.iter().map(|e| expr(e, chans)).collect()).unwrap_or_default();
    let o = v.get("op").and_then(|o| o.as_str()).unwrap_or("");
    if o == "interp" {
        let t = v
            .get("t")
            .and_then(|t| t.as_array())
            .map(|t| {
                t.iter()
                    .filter_map(|p| {
                        let p = p.as_array()?;
                        Some((p.first()?.as_f64()? as f32, p.get(1)?.as_f64()? as f32))
                    })
                    .collect()
            })
            .unwrap_or_default();
        return RExpr::Interp(Box::new(a.into_iter().next().unwrap_or(RExpr::C(0.0))), t);
    }
    let op = match o {
        "add" => Op::Add,
        "mul" => Op::Mul,
        "sub" => Op::Sub,
        "div" => Op::Div,
        "min" => Op::Min,
        "max" => Op::Max,
        "clamp" => Op::Clamp,
        "abs" => Op::Abs,
        "neg" => Op::Neg,
        "not" => Op::Not,
        "and" => Op::And,
        "or" => Op::Or,
        "lt" => Op::Lt,
        "le" => Op::Le,
        "gt" => Op::Gt,
        "ge" => Op::Ge,
        "eq" => Op::Eq,
        "ne" => Op::Ne,
        "sin" => Op::Sin,
        "cos" => Op::Cos,
        "floor" => Op::Floor,
        "mod" => Op::Mod,
        "pow" => Op::Pow,
        "sqrt" => Op::Sqrt,
        "step" => Op::Step,
        _ => return RExpr::C(0.0),
    };
    RExpr::Op(op, a)
}

fn oct_decode(x: i16, y: i16) -> Vec3 {
    let (fx, fy) = (x as f32 / 32767.0, y as f32 / 32767.0);
    let mut n = Vec3::new(fx, fy, 1.0 - fx.abs() - fy.abs());
    if n.z < 0.0 {
        let (ox, oy) = (n.x, n.y);
        n.x = (1.0 - oy.abs()) * ox.signum();
        n.y = (1.0 - ox.abs()) * oy.signum();
    }
    n.normalize_or(Vec3::Y)
}

impl AircraftAsset {
    /// Load `aircraft/<dir>/model.skac` and its textures through [`crate::assets`].
    pub fn load(dir: &str) -> Result<AircraftAsset, String> {
        let raw = crate::assets::read(&format!("aircraft/{dir}/model.skac"))?;
        Self::parse(&raw, dir)
    }

    /// Parse a model; textures are read from `aircraft/<dir>/`.
    pub fn parse(raw: &[u8], dir: &str) -> Result<AircraftAsset, String> {
        if raw.len() < 8 || &raw[0..4] != b"SKAC" {
            return Err("not a SKAC file".into());
        }
        let version = u32::from_le_bytes(raw[4..8].try_into().unwrap());
        if !(1..=FORMAT).contains(&version) {
            return Err(format!("SKAC version {version} not supported (this build reads 1..={FORMAT}; re-import or update)"));
        }
        let mut z = flate2::read::ZlibDecoder::new(&raw[8..]);
        let mut data = Vec::new();
        z.read_to_end(&mut data).map_err(|e| e.to_string())?;
        let hl = u32::from_le_bytes(data[0..4].try_into().unwrap()) as usize;
        let h: Value = serde_json::from_slice(&data[4..4 + hl]).map_err(|e| e.to_string())?;
        let chans: Vec<String> = h["channels"].as_array().map(|a| a.iter().filter_map(|c| c.as_str().map(String::from)).collect()).unwrap_or_default();
        let nodes = h["nodes"]
            .as_array()
            .ok_or("nodes")?
            .iter()
            .map(|n| {
                let pre: Vec<f32> = n["pre"].as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(0.0) as f32).collect()).unwrap_or_default();
                let anim = n.get("anim").filter(|a| !a.is_null()).map(|a| match a["t"].as_str().unwrap_or("") {
                    "rot" => Anim::Rotate { center: vec3(&a["c"]), axis: vec3(&a["a"]).normalize_or(Vec3::X), deg: expr(&a["v"], &chans) },
                    "tr" => Anim::Translate { axis: vec3(&a["a"]).normalize_or(Vec3::X), m: expr(&a["v"], &chans) },
                    "spin" => Anim::Spin { center: vec3(&a["c"]), axis: vec3(&a["a"]).normalize_or(Vec3::X), rpm: expr(&a["v"], &chans) },
                    "sel" => Anim::Select(expr(&a["v"], &chans)),
                    "scale" => Anim::Scale { center: vec3(&a["c"]), s: [expr(&a["v"][0], &chans), expr(&a["v"][1], &chans), expr(&a["v"][2], &chans)] },
                    _ => Anim::Select(RExpr::C(1.0)),
                });
                Node {
                    name: n["name"].as_str().unwrap_or("").to_string(),
                    parent: n["parent"].as_i64().unwrap_or(-1) as i32,
                    pre: if pre.len() == 16 { Mat4::from_cols_slice(&pre) } else { Mat4::IDENTITY },
                    anim,
                }
            })
            .collect();
        let materials = h["materials"]
            .as_array()
            .ok_or("materials")?
            .iter()
            .map(|m| {
                let c = m["col"].as_array();
                let cf = |i: usize| c.and_then(|c| c.get(i)).and_then(|x| x.as_f64()).unwrap_or(1.0) as f32;
                Material {
                    col: Vec4::new(cf(0), cf(1), cf(2), cf(3)),
                    emis: vec3(&m["emis"]),
                    spec: vec3(&m["spec"]),
                    shininess: m["shi"].as_f64().unwrap_or(10.0) as f32,
                    tex: m["tex"].as_i64().filter(|&t| t >= 0).map(|t| t as usize),
                    translucent: m["translucent"].as_bool().unwrap_or(false),
                    alpha_tex: m["alpha_tex"].as_bool().unwrap_or(false),
                    two_sided: m["two_sided"].as_bool().unwrap_or(true),
                }
            })
            .collect();
        let chunks = h["chunks"]
            .as_array()
            .ok_or("chunks")?
            .iter()
            .map(|c| Chunk {
                node: c["node"].as_u64().unwrap_or(0) as usize,
                material: c["mat"].as_u64().unwrap_or(0) as usize,
                base_vertex: c["base_vertex"].as_u64().unwrap_or(0) as u32,
                first: c["first"].as_u64().unwrap_or(0) as u32,
                count: c["count"].as_u64().unwrap_or(0) as u32,
                interior: c["interior"].as_bool().unwrap_or(false),
                emit: c.get("emit").filter(|e| !e.is_null()).map(|e| (vec3(&e["col"]), expr(&e["v"], &chans))),
                uv: c["uv"]
                    .as_array()
                    .map(|a| {
                        a.iter()
                            .filter_map(|u| {
                                let p = |k: &str| {
                                    let a = u[k].as_array();
                                    let f = |i: usize| a.and_then(|a| a.get(i)).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32;
                                    glam::Vec2::new(f(0), f(1))
                                };
                                match u["t"].as_str()? {
                                    "tr" => Some(UvAnim::Translate { axis: p("a"), v: expr(&u["v"], &chans) }),
                                    "rot" => Some(UvAnim::Rotate { center: p("c"), v: expr(&u["v"], &chans) }),
                                    _ => None,
                                }
                            })
                            .collect()
                    })
                    .unwrap_or_default(),
                display: c.get("display").filter(|d| d.is_object()).map(|d| {
                    let w = d["uv"].as_array();
                    let f = |i: usize, def: f32| w.and_then(|a| a.get(i)).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(def);
                    DisplayScreen {
                        kind: d["kind"].as_str().unwrap_or("pfd").to_string(),
                        uv: [f(0, 0.0), f(1, 0.0), f(2, 1.0), f(3, 1.0)],
                        aspect: (d["aspect"].as_f64().unwrap_or(1.0) as f32).clamp(0.2, 5.0),
                    }
                }),
                backlight: false,
            })
            .collect();
        let nv = h["vertex_count"].as_u64().unwrap_or(0) as usize;
        let ni = h["index_count"].as_u64().unwrap_or(0) as usize;
        let mut off = 4 + hl;
        if data.len() < off + nv * 24 + ni * 4 {
            return Err("truncated geometry".into());
        }
        let f = |o: usize| f32::from_le_bytes(data[o..o + 4].try_into().unwrap());
        let s = |o: usize| i16::from_le_bytes(data[o..o + 2].try_into().unwrap());
        let mut vertices = Vec::with_capacity(nv);
        for i in 0..nv {
            let o = off + i * 24;
            let n = oct_decode(s(o + 12), s(o + 14));
            vertices.push(Vertex { pos: [f(o), f(o + 4), f(o + 8)], nrm: n.to_array(), uv: [f(o + 16), f(o + 20)] });
        }
        off += nv * 24;
        let indices = (0..ni).map(|i| u32::from_le_bytes(data[off + i * 4..off + i * 4 + 4].try_into().unwrap())).collect();
        let lights = h["lights"]
            .as_array()
            .map(|a| {
                a.iter()
                    .map(|l| Light {
                        kind: l["kind"].as_str().unwrap_or("").to_string(),
                        pos: vec3(&l["pos"]),
                        node: l["node"].as_i64().filter(|&n| n >= 0).map(|n| n as usize),
                    })
                    .collect()
            })
            .unwrap_or_default();
        let textures = h["textures"]
            .as_array()
            .map(|a| a.iter().map(|t| t["file"].as_str().unwrap_or("").to_string()).collect::<Vec<_>>())
            .unwrap_or_default();
        let id = h["id"].as_str().unwrap_or("").to_string();
        let mut asset = AircraftAsset {
            id: id.clone(),
            name: h["name"].as_str().unwrap_or("").to_string(),
            nodes,
            materials,
            textures: Vec::new(),
            chunks,
            vertices,
            indices,
            eye: h["eye"].as_array().map(|_| vec3(&h["eye"])),
            radius: h["radius"].as_f64().unwrap_or(10.0) as f32,
            lights,
            channels: chans,
        };
        if let Some(pl) = asset.channel("panel_light") {
            for c in &mut asset.chunks {
                if let Some((_, e)) = &c.emit {
                    let mut ids = Vec::new();
                    e.channel_ids(&mut ids);
                    c.backlight = ids.contains(&(pl as u16));
                }
            }
        }
        asset.textures = textures.iter().map(|f| load_texture(dir, f)).collect();
        Ok(asset)
    }

    /// Index of a channel, if the model uses it.
    pub fn channel(&self, name: &str) -> Option<usize> {
        // (Also used while parsing, before the textures are loaded.)
        self.channels.iter().position(|c| c == name)
    }
}

fn load_texture(dir: &str, file: &str) -> Texture {
    let fallback = || Texture { width: 1, height: 1, rgba: vec![200, 200, 200, 255] };
    let Ok(bytes) = crate::assets::read(&format!("aircraft/{dir}/{file}")) else {
        eprintln!("aircraft texture missing: {dir}/{file}");
        return fallback();
    };
    match image::load_from_memory(&bytes) {
        Ok(img) => {
            let img = img.to_rgba8();
            Texture { width: img.width(), height: img.height(), rgba: img.into_raw() }
        }
        Err(e) => {
            eprintln!("aircraft texture {dir}/{file}: {e}");
            fallback()
        }
    }
}

/// Per-instance animation state (spin angles).
#[derive(Default, Clone)]
pub struct AnimState {
    spin_deg: Vec<f32>,
}

/// Newest `.skac` format version this build reads.
pub const FORMAT: u32 = 2;

/// Identity UV transform (see [`Pose::uv`]).
pub const UV_IDENTITY: [f32; 6] = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0];

/// Evaluated pose: node transforms in the body frame, visibility, chunk emission.
#[derive(Default, Clone)]
pub struct Pose {
    pub world: Vec<Mat4>,
    pub visible: Vec<bool>,
    pub emit: Vec<f32>,
    /// Per chunk UV transform `[a, b, c, d, tx, ty]`:
    /// u' = a·u + c·v + tx, v' = b·u + d·v + ty (column-major 2×2 + offset).
    pub uv: Vec<[f32; 6]>,
}

/// Compose a chunk's texture animations into one affine UV transform.
pub fn uv_transform(anims: &[UvAnim], ch: &[f32]) -> [f32; 6] {
    // m = [[a c tx] [b d ty]] acting on (u, v, 1).
    let (mut a, mut b, mut c, mut d, mut tx, mut ty) = (1.0f32, 0.0f32, 0.0f32, 1.0f32, 0.0f32, 0.0f32);
    for an in anims {
        match an {
            UvAnim::Translate { axis, v } => {
                let k = v.eval(ch);
                tx += axis.x * k;
                ty += axis.y * k;
            }
            UvAnim::Rotate { center, v } => {
                let (s, co) = v.eval(ch).to_radians().sin_cos();
                // p' = R (p − c) + c applied after the current transform.
                let (na, nb) = (co * a - s * b, s * a + co * b);
                let (nc, nd) = (co * c - s * d, s * c + co * d);
                let (px, py) = (tx - center.x, ty - center.y);
                let (ntx, nty) = (co * px - s * py + center.x, s * px + co * py + center.y);
                (a, b, c, d, tx, ty) = (na, nb, nc, nd, ntx, nty);
            }
        }
    }
    [a, b, c, d, tx, ty]
}

impl AircraftAsset {
    pub fn pose(&self, ch: &[f32], st: &mut AnimState, dt: f32, out: &mut Pose) {
        let n = self.nodes.len();
        if st.spin_deg.len() != n {
            st.spin_deg = vec![0.0; n];
        }
        out.world.resize(n, Mat4::IDENTITY);
        out.visible.resize(n, true);
        for (i, node) in self.nodes.iter().enumerate() {
            let (pw, pv) = if node.parent >= 0 { (out.world[node.parent as usize], out.visible[node.parent as usize]) } else { (Mat4::IDENTITY, true) };
            let mut vis = pv;
            let a = match &node.anim {
                None => Mat4::IDENTITY,
                Some(Anim::Rotate { center, axis, deg }) => rot_about(*center, *axis, deg.eval(ch).to_radians()),
                Some(Anim::Translate { axis, m }) => Mat4::from_translation(*axis * m.eval(ch)),
                Some(Anim::Spin { center, axis, rpm }) => {
                    let s = &mut st.spin_deg[i];
                    *s = (*s + rpm.eval(ch) * 6.0 * dt).rem_euclid(360.0);
                    rot_about(*center, *axis, s.to_radians())
                }
                Some(Anim::Select(c)) => {
                    vis &= c.eval(ch) != 0.0;
                    Mat4::IDENTITY
                }
                Some(Anim::Scale { center, s }) => {
                    let k = Vec3::new(s[0].eval(ch), s[1].eval(ch), s[2].eval(ch));
                    Mat4::from_translation(*center) * Mat4::from_scale(k) * Mat4::from_translation(-*center)
                }
            };
            out.world[i] = pw * node.pre * a;
            out.visible[i] = vis;
        }
        out.emit.clear();
        out.emit.extend(self.chunks.iter().map(|c| c.emit.as_ref().map(|(_, e)| e.eval(ch).clamp(0.0, 4.0)).unwrap_or(0.0)));
        out.uv.clear();
        out.uv.extend(self.chunks.iter().map(|c| if c.uv.is_empty() { UV_IDENTITY } else { uv_transform(&c.uv, ch) }));
    }

    /// The distinct glass displays (kind, aspect), in first-seen order.
    pub fn displays(&self) -> Vec<(String, f32)> {
        let mut out: Vec<(String, f32)> = Vec::new();
        for d in self.chunks.iter().filter_map(|c| c.display.as_ref()) {
            if !out.iter().any(|(k, _)| *k == d.kind) {
                out.push((d.kind.clone(), d.aspect));
            }
        }
        out
    }

    /// Live cockpit instruments: animated cockpit parts (needles, cards,
    /// drums, flags, annunciators, backlit faces) driven by anything but the
    /// pilot's own controls, plus glass display screens.
    pub fn live_instruments(&self) -> InstrumentCount {
        let pilot = |c: &str| c.starts_with("ctl_") || matches!(c, "throttle" | "brakes" | "time" | "steer_norm" | "steer_deg");
        let live_expr = |e: &RExpr| {
            let mut ids = Vec::new();
            e.channel_ids(&mut ids);
            ids.iter().any(|&i| self.channels.get(i as usize).is_some_and(|n| !pilot(n)))
        };
        // Nodes with interior geometry below them.
        let mut interior = vec![false; self.nodes.len()];
        for c in self.chunks.iter().filter(|c| c.interior) {
            let mut i = c.node as i32;
            while i >= 0 && !interior[i as usize] {
                interior[i as usize] = true;
                i = self.nodes[i as usize].parent;
            }
        }
        let mut n = InstrumentCount::default();
        for (k, node) in self.nodes.iter().enumerate() {
            if !interior[k] {
                continue;
            }
            let live = match &node.anim {
                Some(Anim::Rotate { deg, .. }) => live_expr(deg),
                Some(Anim::Translate { m, .. }) => live_expr(m),
                Some(Anim::Select(c)) => live_expr(c),
                Some(Anim::Scale { s, .. }) => s.iter().any(live_expr),
                _ => false,
            };
            if live {
                match node.anim {
                    Some(Anim::Select(_)) => n.flags += 1,
                    _ => n.needles += 1,
                }
            }
        }
        for c in self.chunks.iter().filter(|c| c.interior) {
            if c.uv.iter().any(|u| match u {
                UvAnim::Translate { v, .. } | UvAnim::Rotate { v, .. } => live_expr(v),
            }) {
                n.drums += 1;
            }
            if c.emit.as_ref().is_some_and(|(_, e)| live_expr(e)) {
                n.lit += 1;
            }
            if c.display.is_some() {
                n.displays += 1;
            }
        }
        n
    }
}

/// Live cockpit instrument parts by kind (see [`AircraftAsset::live_instruments`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InstrumentCount {
    /// Needles, pointers, cards, balls (rotate / translate / scale).
    pub needles: usize,
    /// Flags and annunciator / indicator selects.
    pub flags: usize,
    /// Texture-animated drums, tapes, attitude balls.
    pub drums: usize,
    /// Backlit / annunciator emission driven by the systems.
    pub lit: usize,
    /// Glass display screens.
    pub displays: usize,
}

impl InstrumentCount {
    /// Live instruments: needles, flags, drums and displays (backlit parts
    /// are counted separately in `lit`).
    pub fn total(&self) -> usize {
        self.needles + self.flags + self.drums + self.displays
    }
}

fn rot_about(c: Vec3, axis: Vec3, rad: f32) -> Mat4 {
    Mat4::from_translation(c) * Mat4::from_quat(Quat::from_axis_angle(axis, rad)) * Mat4::from_translation(-c)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Posed geometry report (gear down, everything else neutral):
    /// `SKAC_DUMP=meridian320 cargo test geometry_dump -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn geometry_dump() {
        let id = std::env::var("SKAC_DUMP").unwrap_or_else(|_| "kestrel".into());
        let a = AircraftAsset::load(&id).expect("load");
        let mut ch = vec![0.0f32; a.channels.len()];
        for (i, c) in a.channels.iter().enumerate() {
            if c == "gear_pos" || c == "nav" {
                ch[i] = 1.0;
            }
        }
        let mut st = AnimState::default();
        let mut pose = Pose::default();
        a.pose(&ch, &mut st, 0.0, &mut pose);
        let (mut lo, mut hi) = (Vec3::splat(1e9), Vec3::splat(-1e9));
        let mut per_node: std::collections::BTreeMap<usize, (Vec3, Vec3)> = Default::default();
        for c in &a.chunks {
            if c.interior || !pose.visible[c.node] {
                continue;
            }
            let m = pose.world[c.node];
            for k in c.first..c.first + c.count {
                let v = a.vertices[(a.indices[k as usize] + c.base_vertex) as usize];
                let p = m.transform_point3(Vec3::from(v.pos));
                lo = lo.min(p);
                hi = hi.max(p);
                let e = per_node.entry(c.node).or_insert((Vec3::splat(1e9), Vec3::splat(-1e9)));
                e.0 = e.0.min(p);
                e.1 = e.1.max(p);
            }
        }
        println!("{id}: exterior bbox lo {lo:?} hi {hi:?} (span {:.2}, length {:.2}, height {:.2})", hi.x - lo.x, hi.z - lo.z, hi.y - lo.y);
        let re = std::env::var("SKAC_NODES").unwrap_or_else(|_| "(?i)wheel|tyre|tire|gear|strut|prop|spinner|nozzle|engine|eng|fan".into());
        let _ = re;
        for (n, (l, h)) in per_node {
            let name = &a.nodes[n].name;
            let lname = name.to_lowercase();
            if ["wheel", "tyre", "tire", "gear", "prop", "spinner", "nozzle", "fan", "engine", "skid", "tail"].iter().any(|k| lname.contains(k)) {
                println!("  {:<40} lo ({:6.2} {:6.2} {:6.2}) hi ({:6.2} {:6.2} {:6.2})", name, l.x, l.y, l.z, h.x, h.y, h.z);
            }
        }
    }
}

#[cfg(test)]
mod sign_tests {
    use super::*;

    /// Mean height (body Y) of the exterior vertices under nodes whose name matches.
    fn mean_y(a: &AircraftAsset, ch: &[f32], pat: &str) -> Option<f32> {
        let mut st = AnimState::default();
        let mut pose = Pose::default();
        a.pose(ch, &mut st, 0.0, &mut pose);
        let (mut sum, mut n) = (0.0, 0usize);
        for c in a.chunks.iter().filter(|c| !c.interior) {
            // A chunk belongs to the pattern if its node or an ancestor matches.
            let mut i = c.node as i32;
            let mut hit = false;
            while i >= 0 {
                if a.nodes[i as usize].name.to_lowercase().contains(pat) {
                    hit = true;
                    break;
                }
                i = a.nodes[i as usize].parent;
            }
            if !hit {
                continue;
            }
            for k in c.first..c.first + c.count {
                let v = a.vertices[(a.indices[k as usize] + c.base_vertex) as usize];
                sum += pose.world[c.node].transform_point3(Vec3::from(v.pos)).y;
                n += 1;
            }
        }
        (n > 0).then(|| sum / n as f32)
    }

    /// FlightGear rotation sense: with the channels at FlightGear semantics a
    /// positive (right-roll) aileron puts the left aileron's trailing edge
    /// down, and positive elevator (stick forward) puts the elevator TE down.
    #[test]
    fn control_surface_senses() {
        let Ok(a) = AircraftAsset::load("kestrel") else { return };
        let mut ch = vec![0.0f32; a.channels.len()];
        let set = |ch: &mut Vec<f32>, n: &str, v: f32| {
            if let Some(i) = a.channels.iter().position(|c| c == n) {
                ch[i] = v;
            }
        };
        let base = mean_y(&a, &ch, "leftaileron").unwrap();
        set(&mut ch, "aileron_l", 1.0);
        set(&mut ch, "aileron_r", -1.0);
        let rolled = mean_y(&a, &ch, "leftaileron").unwrap();
        assert!(rolled < base - 0.01, "left aileron should go down for right roll: {base} → {rolled}");
        let mut ch = vec![0.0f32; a.channels.len()];
        let base = mean_y(&a, &ch, "elevator").unwrap();
        set(&mut ch, "elevator", 1.0);
        let pushed = mean_y(&a, &ch, "elevator").unwrap();
        assert!(pushed < base - 0.005, "elevator TE should go down for push: {base} → {pushed}");
    }
}
