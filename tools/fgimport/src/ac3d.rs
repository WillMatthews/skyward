//! AC3D (.ac) reader. Produces an object tree with geometry already moved into
//! the file's root frame (each object's `loc` / `rot` applied cumulatively)
//! and converted from AC3D's y-up axes to FlightGear model axes
//! (x aft, y right, z up): model = (ac.x, -ac.z, ac.y).

use glam::{Mat3, Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct AcMaterial {
    pub name: String,
    pub rgb: Vec3,
    pub amb: Vec3,
    pub emis: Vec3,
    pub spec: Vec3,
    pub shi: f32,
    pub trans: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Tri {
    pub v: [Vec3; 3],
    pub uv: [Vec2; 3],
    pub mat: usize,
    /// Smooth-shaded surface (normals averaged within the crease angle).
    pub smooth: bool,
    pub two_sided: bool,
}

#[derive(Clone, Debug, Default)]
pub struct AcObject {
    pub name: String,
    pub texture: Option<String>,
    pub crease: f32,
    pub tris: Vec<Tri>,
    /// Line segments (kept for axis objects: `<axis><object-name>`).
    pub lines: Vec<[Vec3; 2]>,
    pub kids: Vec<AcObject>,
}

pub struct AcFile {
    pub materials: Vec<AcMaterial>,
    pub root: AcObject,
}

fn to_model(p: Vec3) -> Vec3 {
    Vec3::new(p.x, -p.z, p.y)
}

struct Lines<'a> {
    it: std::iter::Peekable<std::str::Lines<'a>>,
}

impl<'a> Lines<'a> {
    fn next(&mut self) -> Option<&'a str> {
        loop {
            let l = self.it.next()?;
            let t = l.trim();
            if !t.is_empty() {
                return Some(t);
            }
        }
    }
}

fn quoted(s: &str) -> String {
    let s = s.trim();
    if let Some(r) = s.strip_prefix('"') {
        r.rsplit_once('"').map(|(a, _)| a.to_string()).unwrap_or_else(|| r.to_string())
    } else {
        s.to_string()
    }
}

fn floats(s: &str) -> Vec<f32> {
    s.split_whitespace().filter_map(|x| x.parse::<f32>().ok()).collect()
}

fn v3(f: &[f32], i: usize) -> Vec3 {
    Vec3::new(*f.get(i).unwrap_or(&0.0), *f.get(i + 1).unwrap_or(&0.0), *f.get(i + 2).unwrap_or(&0.0))
}

pub fn parse(text: &str) -> Result<AcFile, String> {
    let mut l = Lines { it: text.lines().peekable() };
    let head = l.next().ok_or("empty file")?;
    if !head.starts_with("AC3D") {
        return Err("not an AC3D file".into());
    }
    let mut materials = Vec::new();
    // Materials then the world object.
    let mut first_obj = None;
    while let Some(line) = l.next() {
        if line.starts_with("MATERIAL") {
            // MATERIAL "name" rgb r g b  amb r g b  emis r g b  spec r g b  shi n  trans t
            let rest = &line[8..];
            let name = quoted(rest);
            let after = rest.trim().strip_prefix('"').and_then(|r| r.split_once('"')).map(|(_, b)| b).unwrap_or(rest);
            let toks: Vec<&str> = after.split_whitespace().collect();
            let mut m = AcMaterial { name, rgb: Vec3::ONE, amb: Vec3::splat(0.2), emis: Vec3::ZERO, spec: Vec3::ZERO, shi: 10.0, trans: 0.0 };
            let mut i = 0;
            while i < toks.len() {
                let f = |k: usize| toks.get(i + k).and_then(|x| x.parse::<f32>().ok()).unwrap_or(0.0);
                match toks[i] {
                    "rgb" => { m.rgb = Vec3::new(f(1), f(2), f(3)); i += 4; }
                    "amb" => { m.amb = Vec3::new(f(1), f(2), f(3)); i += 4; }
                    "emis" => { m.emis = Vec3::new(f(1), f(2), f(3)); i += 4; }
                    "spec" => { m.spec = Vec3::new(f(1), f(2), f(3)); i += 4; }
                    "shi" => { m.shi = f(1); i += 2; }
                    "trans" => { m.trans = f(1); i += 2; }
                    _ => i += 1,
                }
            }
            materials.push(m);
        } else if line.starts_with("OBJECT") {
            first_obj = Some(line);
            break;
        }
    }
    let first = first_obj.ok_or("no OBJECT")?;
    let root = parse_object(&mut l, first, Mat3::IDENTITY, Vec3::ZERO)?;
    Ok(AcFile { materials, root })
}

fn parse_object(l: &mut Lines, header: &str, prot: Mat3, ploc: Vec3) -> Result<AcObject, String> {
    let _kind = header.split_whitespace().nth(1).unwrap_or("poly").to_string();
    let mut o = AcObject { crease: 61.0, ..Default::default() };
    let mut rot = Mat3::IDENTITY;
    let mut loc = Vec3::ZERO;
    let mut texrep = Vec2::ONE;
    let mut texoff = Vec2::ZERO;
    let mut verts: Vec<Vec3> = Vec::new();
    // Surfaces are buffered until "kids" so that loc/rot given after numvert still apply
    // (the spec puts them before, but be lenient).
    let mut surfs: Vec<(u32, usize, Vec<(usize, Vec2)>)> = Vec::new();
    loop {
        let line = l.next().ok_or("unexpected EOF in object")?;
        let (tok, rest) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        match tok {
            "name" => o.name = quoted(rest),
            "data" => {
                // data <len>\n<len bytes possibly multi-line>
                let n: usize = rest.trim().parse().unwrap_or(0);
                let mut got = 0;
                while got < n {
                    match l.it.next() {
                        Some(s) => got += s.len() + 1,
                        None => break,
                    }
                }
            }
            "texture" => o.texture = Some(quoted(rest)),
            "texrep" => {
                let f = floats(rest);
                texrep = Vec2::new(*f.first().unwrap_or(&1.0), *f.get(1).unwrap_or(&1.0));
            }
            "texoff" => {
                let f = floats(rest);
                texoff = Vec2::new(*f.first().unwrap_or(&0.0), *f.get(1).unwrap_or(&0.0));
            }
            "rot" => {
                let f = floats(rest);
                if f.len() >= 9 {
                    // As OSG's AC3D loader: row-vector convention v' = v·M with M
                    // rows (f0 f1 f2) ... → column-vector matrix columns = those rows.
                    rot = Mat3::from_cols(Vec3::new(f[0], f[1], f[2]), Vec3::new(f[3], f[4], f[5]), Vec3::new(f[6], f[7], f[8]));
                }
            }
            "loc" => loc = v3(&floats(rest), 0),
            "crease" => o.crease = rest.trim().parse().unwrap_or(61.0),
            "url" | "subdiv" | "hidden" | "locked" | "folded" => {}
            "numvert" => {
                let n: usize = rest.trim().parse().unwrap_or(0);
                verts.reserve(n);
                for _ in 0..n {
                    let f = floats(l.next().ok_or("EOF in verts")?);
                    verts.push(v3(&f, 0));
                }
            }
            "numsurf" => {
                let n: usize = rest.trim().parse().unwrap_or(0);
                for _ in 0..n {
                    let mut flags = 0u32;
                    let mut mat = 0usize;
                    let mut refs = Vec::new();
                    loop {
                        let sl = l.next().ok_or("EOF in surf")?;
                        let (t, r) = sl.split_once(char::is_whitespace).unwrap_or((sl, ""));
                        match t {
                            "SURF" => {
                                let r = r.trim();
                                flags = if let Some(h) = r.strip_prefix("0x").or_else(|| r.strip_prefix("0X")) {
                                    u32::from_str_radix(h, 16).unwrap_or(0)
                                } else {
                                    r.parse().unwrap_or(0)
                                };
                            }
                            "mat" => mat = r.trim().parse().unwrap_or(0),
                            "refs" => {
                                let k: usize = r.trim().parse().unwrap_or(0);
                                for _ in 0..k {
                                    let f = floats(l.next().ok_or("EOF in refs")?);
                                    let vi = *f.first().unwrap_or(&0.0) as usize;
                                    let uv = Vec2::new(*f.get(1).unwrap_or(&0.0), *f.get(2).unwrap_or(&0.0));
                                    refs.push((vi, uv));
                                }
                                break;
                            }
                            _ => {}
                        }
                    }
                    surfs.push((flags, mat, refs));
                }
            }
            "kids" => {
                let n: usize = rest.trim().parse().unwrap_or(0);
                // World transform of this object.
                let wrot = prot * rot;
                let wloc = ploc + prot * loc;
                let xf = |p: Vec3| to_model(wrot * p + wloc);
                for (flags, mat, refs) in surfs.drain(..) {
                    let ty = flags & 0xF;
                    let pts: Vec<(Vec3, Vec2)> = refs
                        .iter()
                        .filter(|(vi, _)| *vi < verts.len())
                        .map(|(vi, uv)| (xf(verts[*vi]), *uv * texrep + texoff))
                        .collect();
                    if ty != 0 {
                        // Lines: keep segments (axis definitions).
                        for w in pts.windows(2) {
                            o.lines.push([w[0].0, w[1].0]);
                        }
                        if ty == 1 && pts.len() > 2 {
                            o.lines.push([pts[pts.len() - 1].0, pts[0].0]);
                        }
                        continue;
                    }
                    if pts.len() < 3 {
                        if pts.len() == 2 {
                            o.lines.push([pts[0].0, pts[1].0]);
                        }
                        continue;
                    }
                    for tri in triangulate(&pts) {
                        o.tris.push(Tri {
                            v: [pts[tri[0]].0, pts[tri[1]].0, pts[tri[2]].0],
                            uv: [pts[tri[0]].1, pts[tri[1]].1, pts[tri[2]].1],
                            mat,
                            smooth: flags & 0x10 != 0,
                            two_sided: flags & 0x20 != 0,
                        });
                    }
                }
                for _ in 0..n {
                    // A kid count larger than the objects that follow (some
                    // exporters, e.g. the FGAddon Concorde) is tolerated, as
                    // OSG's loader does: stop at the end of the file.
                    let Some(h) = l.next() else { break };
                    if !h.starts_with("OBJECT") {
                        return Err(format!("expected OBJECT, got {h}"));
                    }
                    o.kids.push(parse_object(l, h, wrot, wloc)?);
                }
                return Ok(o);
            }
            _ => {}
        }
    }
}

/// Ear-clipping triangulation of a (nearly planar) polygon; falls back to a fan.
fn triangulate(p: &[(Vec3, Vec2)]) -> Vec<[usize; 3]> {
    let n = p.len();
    if n == 3 {
        return vec![[0, 1, 2]];
    }
    // Newell normal.
    let mut nrm = Vec3::ZERO;
    for i in 0..n {
        let a = p[i].0;
        let b = p[(i + 1) % n].0;
        nrm += Vec3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    if nrm.length_squared() < 1e-20 || n == 4 && is_convex(p, nrm) {
        return (1..n - 1).map(|i| [0, i, i + 1]).collect();
    }
    let nrm = nrm.normalize();
    let mut idx: Vec<usize> = (0..n).collect();
    let mut out = Vec::new();
    let mut guard = 0;
    while idx.len() > 3 && guard < 4 * n {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for k in 0..m {
            let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
            let (a, b, c) = (p[ia].0, p[ib].0, p[ic].0);
            if (b - a).cross(c - b).dot(nrm) <= 0.0 {
                continue;
            }
            let inside = idx.iter().any(|&j| j != ia && j != ib && j != ic && in_tri(p[j].0, a, b, c, nrm));
            if inside {
                continue;
            }
            out.push([ia, ib, ic]);
            idx.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    } else if idx.len() > 3 {
        for i in 1..idx.len() - 1 {
            out.push([idx[0], idx[i], idx[i + 1]]);
        }
    }
    out
}

fn is_convex(p: &[(Vec3, Vec2)], nrm: Vec3) -> bool {
    let n = p.len();
    (0..n).all(|i| {
        let a = p[i].0;
        let b = p[(i + 1) % n].0;
        let c = p[(i + 2) % n].0;
        (b - a).cross(c - b).dot(nrm) >= 0.0
    })
}

fn in_tri(q: Vec3, a: Vec3, b: Vec3, c: Vec3, n: Vec3) -> bool {
    let s1 = (b - a).cross(q - a).dot(n);
    let s2 = (c - b).cross(q - b).dot(n);
    let s3 = (a - c).cross(q - c).dot(n);
    s1 > 0.0 && s2 > 0.0 && s3 > 0.0
}
