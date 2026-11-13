//! Parked aircraft models: the sim's real aircraft assets
//! (`aircraft_asset.rs`, read-only) posed at rest (gear down, controls
//! neutral) and baked into static meshes with three levels of detail:
//!
//! * LOD 0 — the textured model (exterior chunks, per-material draws);
//! * LOD 1 / 2 — vertex-clustered meshes coloured from the textures
//!   (≈ 1/20 and 1/200 of the triangles), for the stands seen from afar.
//!
//! Built once per type on a background thread (natively; in the browser
//! only for types whose files are already downloaded) and shared by every
//! parked instance.

use super::types;
use crate::aircraft_asset::{AircraftAsset, AnimState, Pose};
use crate::scene::{MeshData, mat};
use glam::{Mat4, Vec2, Vec3, vec2, vec3};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

/// LOD 0 vertex: position, normal, uv, material colour (sRGB unorm8 + alpha),
/// params (f0, roughness, alpha mode / 2, textured) as unorm8.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PVert {
    pub pos: [f32; 3],
    pub nrm: [f32; 3],
    pub uv: [f32; 2],
    pub col: [u8; 4],
    pub params: [u8; 4],
}

/// A per-texture draw of LOD 0.
pub struct Group {
    pub first: u32,
    pub count: u32,
    pub tex: Option<usize>,
}

/// An sRGB RGBA8 texture with its mip chain (built off the render thread).
pub struct Texture {
    pub w: u32,
    pub h: u32,
    pub rgba: Vec<u8>,
    pub mips: Vec<Vec<u8>>,
}

/// Mip chain (level 1..) of an sRGB RGBA8 image, box-filtered in linear light.
fn mip_chain(w: u32, h: u32, rgba: &[u8]) -> Vec<Vec<u8>> {
    let to_lin: Vec<f32> = (0..256).map(|i| (i as f32 / 255.0).powf(2.2)).collect();
    let mut cur: Vec<f32> = rgba.iter().enumerate().map(|(i, &b)| if i % 4 == 3 { b as f32 / 255.0 } else { to_lin[b as usize] }).collect();
    let (mut cw, mut ch) = (w, h);
    let mut out = Vec::new();
    while cw > 1 || ch > 1 {
        let (nw, nh) = ((cw / 2).max(1), (ch / 2).max(1));
        let mut next = vec![0f32; (nw * nh * 4) as usize];
        for y in 0..nh {
            for x in 0..nw {
                let o = ((y * nw + x) * 4) as usize;
                for dy in 0..2u32 {
                    for dx in 0..2u32 {
                        let (sx, sy) = ((x * 2 + dx).min(cw - 1), (y * 2 + dy).min(ch - 1));
                        let so = ((sy * cw + sx) * 4) as usize;
                        for k in 0..4 {
                            next[o + k] += cur[so + k] * 0.25;
                        }
                    }
                }
            }
        }
        out.push(next.iter().enumerate().map(|(i, &v)| if i % 4 == 3 { (v * 255.0 + 0.5) as u8 } else { (v.max(0.0).powf(1.0 / 2.2) * 255.0 + 0.5).min(255.0) as u8 }).collect());
        cur = next;
        cw = nw;
        ch = nh;
    }
    out
}

pub struct Model {
    pub ty: u16,
    /// Body frame → standing on the ground at the origin (pitch, height).
    pub rest: Mat4,
    pub verts: Vec<PVert>,
    pub idx: Vec<u32>,
    pub groups: Vec<Group>,
    pub textures: Vec<Texture>,
    pub lod1: MeshData,
    pub lod2: MeshData,
}

enum Slot {
    Loading,
    Ready(Arc<Model>),
    Failed,
}

fn slots() -> &'static Mutex<HashMap<u16, Slot>> {
    static S: OnceLock<Mutex<HashMap<u16, Slot>>> = OnceLock::new();
    S.get_or_init(|| Mutex::new(HashMap::new()))
}

/// The model of a type if built; starts building it otherwise.
pub fn get(ty: u16) -> Option<Arc<Model>> {
    {
        let mut s = slots().lock().unwrap();
        match s.get(&ty) {
            Some(Slot::Ready(m)) => return Some(m.clone()),
            Some(_) => return None,
            None => {
                #[cfg(target_arch = "wasm32")]
                {
                    // Browser: only aircraft already downloaded (no extra megabytes for scenery).
                    let dir = format!("aircraft/{}/", types::get(ty).id);
                    if !matches!(crate::platform::web::dir_state(&dir), crate::platform::web::DirState::Ready) {
                        return None;
                    }
                }
                s.insert(ty, Slot::Loading);
            }
        }
    }
    let ok = crate::platform::spawn("fleet", move || {
        let r = build(ty);
        if let Err(e) = &r {
            eprintln!("airport: parked {}: {e}", types::get(ty).id);
        }
        slots().lock().unwrap().insert(ty, r.map(|m| Slot::Ready(Arc::new(m))).unwrap_or(Slot::Failed));
    });
    if !ok {
        slots().lock().unwrap().insert(ty, Slot::Failed);
    }
    None
}

/// Every type requested so far has finished loading (or failed).
pub fn settled() -> bool {
    !slots().lock().unwrap().values().any(|s| matches!(s, Slot::Loading))
}

fn to_srgb8(v: f32) -> u8 {
    (v.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0 + 0.5) as u8
}

/// Downscale an RGBA8 image to at most `max` on its long side (box filter).
fn shrink(w: u32, h: u32, rgba: &[u8], max: u32) -> (u32, u32, Vec<u8>) {
    let mut f = 1;
    while w / f > max || h / f > max {
        f *= 2;
    }
    if f == 1 {
        return (w, h, rgba.to_vec());
    }
    let (nw, nh) = ((w / f).max(1), (h / f).max(1));
    let mut out = vec![0u8; (nw * nh * 4) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let mut acc = [0u32; 4];
            let mut n = 0;
            for dy in 0..f {
                for dx in 0..f {
                    let (sx, sy) = ((x * f + dx).min(w - 1), (y * f + dy).min(h - 1));
                    let o = ((sy * w + sx) * 4) as usize;
                    for k in 0..4 {
                        acc[k] += rgba[o + k] as u32;
                    }
                    n += 1;
                }
            }
            let o = ((y * nw + x) * 4) as usize;
            for k in 0..4 {
                out[o + k] = (acc[k] / n) as u8;
            }
        }
    }
    (nw, nh, out)
}

/// Bilinear sample of a small RGBA8 image (wrapping uvs), 0..1 floats (sRGB).
fn sample(w: u32, h: u32, rgba: &[u8], uv: [f32; 2]) -> [f32; 4] {
    let x = (uv[0].rem_euclid(1.0) * w as f32 - 0.5).max(0.0);
    let y = (uv[1].rem_euclid(1.0) * h as f32 - 0.5).max(0.0);
    let (x0, y0) = ((x as u32).min(w - 1), (y as u32).min(h - 1));
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (fx, fy) = (x - x0 as f32, y - y0 as f32);
    let px = |xx: u32, yy: u32, k: usize| rgba[((yy * w + xx) * 4) as usize + k] as f32 / 255.0;
    let mut o = [0.0; 4];
    for (k, v) in o.iter_mut().enumerate() {
        let a = px(x0, y0, k) * (1.0 - fx) + px(x1, y0, k) * fx;
        let b = px(x0, y1, k) * (1.0 - fx) + px(x1, y1, k) * fx;
        *v = a * (1.0 - fy) + b * fy;
    }
    o
}

pub fn build(ty: u16) -> Result<Model, String> {
    let t = types::get(ty);
    // (Reuse the flown aircraft's asset if it is loaded: in the browser its
    // textures were decoded once, in a worker.)
    let a = match crate::models::cached_asset(t.id) {
        Some(a) => a,
        None => Arc::new(AircraftAsset::load(t.id)?),
    };
    // Rest pose: gear down, cargo doors / ramps open (a C-5 kneels to
    // load), everything else neutral.
    let mut ch = vec![0.0f32; a.channels.len()];
    for (i, c) in a.channels.iter().enumerate() {
        if matches!(c.as_str(), "gear_pos" | "doors" | "doors_kneeled" | "kneel") {
            ch[i] = 1.0;
        }
    }
    let mut pose = Pose::default();
    a.pose(&ch, &mut AnimState::default(), 0.0, &mut pose);
    let textures: Vec<Texture> = a
        .textures
        .iter()
        .map(|tx| {
            let (w, h, rgba) = shrink(tx.width, tx.height, &tx.rgba, 1024);
            let mips = mip_chain(w, h, &rgba);
            Texture { w, h, rgba, mips }
        })
        .collect();
    let small: Vec<(u32, u32, Vec<u8>)> = textures.iter().map(|tx| shrink(tx.w, tx.h, &tx.rgba, 32)).collect();
    // Bake every visible exterior chunk, grouped by texture.
    let mut by_tex: HashMap<Option<usize>, Vec<u32>> = HashMap::new();
    let mut verts: Vec<PVert> = Vec::new();
    let mut vcol: Vec<[f32; 3]> = Vec::new();
    let mut translucent: Vec<bool> = Vec::new();
    for c in &a.chunks {
        if c.interior || c.emit.is_some() || !pose.visible[c.node] {
            continue;
        }
        let m = &a.materials[c.material];
        if m.col.w < 0.05 {
            continue;
        }
        let xf = pose.world[c.node];
        let nx = xf.inverse().transpose();
        let mut remap: HashMap<u32, u32> = HashMap::new();
        let list = by_tex.entry(m.tex.filter(|&i| i < textures.len())).or_default();
        let (f0, rough) = if m.translucent { (0.06, 0.05) } else { ((m.spec.max_element() * 0.08).clamp(0.03, 0.08), (1.0 - m.shininess / 128.0).clamp(0.25, 0.8)) };
        let mode = if m.translucent {
            2.0
        } else if m.alpha_tex {
            1.0
        } else {
            0.0
        };
        for k in c.first..c.first + c.count {
            let vi = a.indices[k as usize] + c.base_vertex;
            let ni = *remap.entry(vi).or_insert_with(|| {
                let v = a.vertices[vi as usize];
                let p = xf.transform_point3(Vec3::from(v.pos));
                let n = nx.transform_vector3(Vec3::from(v.nrm)).normalize_or(Vec3::Y);
                let col = [to_srgb8(m.col.x), to_srgb8(m.col.y), to_srgb8(m.col.z), (m.col.w.clamp(0.0, 1.0) * 255.0) as u8];
                let params = [(f0 * 255.0) as u8, (rough * 255.0) as u8, (mode * 0.5 * 255.0) as u8, if m.tex.is_some() { 255 } else { 0 }];
                verts.push(PVert { pos: p.into(), nrm: n.into(), uv: v.uv, col, params });
                // LOD colour: material × the texture's average around the uv.
                let mut cc = [m.col.x.clamp(0.0, 1.0).powf(1.0 / 2.2), m.col.y.clamp(0.0, 1.0).powf(1.0 / 2.2), m.col.z.clamp(0.0, 1.0).powf(1.0 / 2.2)];
                if let Some((w, h, rgba)) = m.tex.and_then(|i| small.get(i)) {
                    let s = sample(*w, *h, rgba, v.uv);
                    cc = [cc[0] * s[0], cc[1] * s[1], cc[2] * s[2]];
                }
                if m.translucent {
                    cc = [0.05, 0.06, 0.07];
                }
                vcol.push(cc);
                translucent.push(m.translucent);
                (verts.len() - 1) as u32
            });
            list.push(ni);
        }
    }
    if verts.is_empty() {
        return Err("no exterior geometry".into());
    }
    let mut idx = Vec::new();
    let mut groups = Vec::new();
    let mut keys: Vec<Option<usize>> = by_tex.keys().copied().collect();
    keys.sort();
    for k in keys {
        let l = &by_tex[&k];
        groups.push(Group { first: idx.len() as u32, count: l.len() as u32, tex: k });
        idx.extend_from_slice(l);
    }
    let rest = rest_pose(&verts);
    let span = t.span.max(5.0);
    let lod1 = cluster(&verts, &vcol, &translucent, &idx, span / 110.0);
    let lod2 = cluster(&verts, &vcol, &translucent, &idx, span / 28.0);
    Ok(Model { ty, rest, verts, idx, groups, textures, lod1, lod2 })
}

/// The rest attitude on the ground: the lower convex hull of the side
/// profile (z, y); its edge under the CG is the ground line (tricycle:
/// level; taildragger: nose up on the tail wheel).
fn rest_pose(verts: &[PVert]) -> Mat4 {
    let mut bins: HashMap<i32, f32> = HashMap::new();
    for v in verts {
        let k = (v.pos[2] / 0.2).floor() as i32;
        let e = bins.entry(k).or_insert(f32::MAX);
        *e = e.min(v.pos[1]);
    }
    let mut pts: Vec<Vec2> = bins.iter().map(|(k, y)| vec2((*k as f32 + 0.5) * 0.2, *y)).collect();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x));
    // Lower hull (monotone chain).
    let mut hull: Vec<Vec2> = Vec::new();
    for p in pts {
        while hull.len() >= 2 && (hull[hull.len() - 1] - hull[hull.len() - 2]).perp_dot(p - hull[hull.len() - 2]) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let (mut slope, mut y0) = (0.0, hull.iter().map(|p| p.y).fold(f32::MAX, f32::min));
    for w in hull.windows(2) {
        if w[0].x <= 0.0 && w[1].x >= 0.0 && w[1].x > w[0].x {
            slope = (w[1].y - w[0].y) / (w[1].x - w[0].x);
            y0 = w[0].y - slope * w[0].x;
        }
    }
    let slope = slope.clamp(-0.35, 0.35);
    // Rotate about X so the ground line is level, then stand it on y = 0.
    let ang = slope.atan();
    let r = Mat4::from_rotation_x(ang);
    let c = r.transform_point3(vec3(0.0, y0, 0.0)).y;
    Mat4::from_translation(vec3(0.0, -c, 0.0)) * r
}

/// Vertex clustering (cell `s` m; separate clusters per normal octant so
/// thin skins keep both sides), coloured meshes for the far LODs.
fn cluster(verts: &[PVert], cols: &[[f32; 3]], transl: &[bool], idx: &[u32], s: f32) -> MeshData {
    let mut map: HashMap<(i32, i32, i32, u8), u32> = HashMap::new();
    let mut acc: Vec<(Vec3, Vec3, [f32; 3], f32, bool)> = Vec::new();
    let mut of = vec![0u32; verts.len()];
    for (i, v) in verts.iter().enumerate() {
        let p = Vec3::from(v.pos);
        let n = Vec3::from(v.nrm);
        let oct = (n.x > 0.0) as u8 | ((n.y > 0.0) as u8) << 1 | ((n.z > 0.0) as u8) << 2;
        let key = ((p.x / s).floor() as i32, (p.y / s).floor() as i32, (p.z / s).floor() as i32, oct);
        let ci = *map.entry(key).or_insert_with(|| {
            acc.push((Vec3::ZERO, Vec3::ZERO, [0.0; 3], 0.0, false));
            (acc.len() - 1) as u32
        });
        let a = &mut acc[ci as usize];
        a.0 += p;
        a.1 += n;
        for k in 0..3 {
            a.2[k] += cols[i][k];
        }
        a.3 += 1.0;
        a.4 |= transl[i];
        of[i] = ci;
    }
    let mut m = MeshData::default();
    for a in &acc {
        let n = a.3.max(1.0);
        m.mat = if a.4 { mat::CANOPY } else { mat::PAINT };
        m.v(a.0 / n, (a.1 / n).normalize_or(Vec3::Y), [a.2[0] / n, a.2[1] / n, a.2[2] / n]);
    }
    let mut seen = std::collections::HashSet::new();
    for t in idx.chunks_exact(3) {
        let (a, b, c) = (of[t[0] as usize], of[t[1] as usize], of[t[2] as usize]);
        if a == b || b == c || a == c {
            continue;
        }
        let mut k = [a, b, c];
        k.sort_unstable();
        if seen.insert(k) {
            m.idx.extend([a, b, c]);
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every parked type bakes (the air-base transports posed with their doors open).
    #[test]
    fn parked_types_bake() {
        for ty in 0..types::TYPES.len() as u16 {
            let id = types::get(ty).id;
            if AircraftAsset::load(id).is_err() {
                eprintln!("{id}: asset not found, skipped");
                continue;
            }
            let m = build(ty).unwrap_or_else(|e| panic!("{id}: {e}"));
            assert!(!m.idx.is_empty() && m.lod1.idx.len() < m.idx.len(), "{id}");
        }
    }
}
