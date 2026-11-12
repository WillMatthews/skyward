//! Scene geometry: shared mesh types, per-tile mesh/collision
//! builders (run on the tile workers), runway markings and thermals.

use crate::osm::{point_in_ring, Airport, Area, Kind, Proj, RunwayPiece, TileFeatures, TileId};
use glam::{vec2, vec3, Vec2, Vec3};


/// Material ids interpreted by the shaders.
pub mod mat {
    pub const DIFFUSE: f32 = 0.0;
    pub const WATER: f32 = 1.0;
    /// Masonry building wall with punched windows (uv = wall metres).
    pub const WALL: f32 = 2.0;
    /// Curtain-wall glass tower (uv = wall metres).
    pub const GLASS: f32 = 3.0;
    pub const ROOF: f32 = 5.0;
    /// Vegetation / grass with world-space noise.
    pub const GRASS: f32 = 6.0;
    /// Asphalt / concrete with world-space noise.
    pub const ASPHALT: f32 = 7.0;
    /// Glossy aircraft paint.
    pub const PAINT: f32 = 8.0;
    /// Aircraft canopy glass.
    pub const CANOPY: f32 = 9.0;
    /// Spinning propeller disc (translucent-looking dark).
    pub const PROP: f32 = 10.0;
    pub const FOLIAGE: f32 = 11.0;
    /// Base ground: procedural fields when uv.x = 0; urban ground tinted by
    /// the vertex colour when uv.x = 1.
    pub const GROUND: f32 = 12.0;
    /// Self-lit: colour * uv.x intensity (nav/strobe lights, afterburner flame).
    /// uv.y > 0.5 = additive-looking flame (rendered with alpha-to-coverage, alpha = col luminance).
    pub const EMISSIVE: f32 = 13.0;
    /// Metallic (bare aluminium / engine cowls): tinted specular.
    pub const METAL: f32 = 14.0;
    /// Runway asphalt: uv = (metres from threshold a, metres across) — wear, rubber.
    pub const RUNWAY: f32 = 15.0;
    /// Road asphalt with lane markings: uv.x = metres along,
    /// uv.y = metres across + 1000 * round(width * 10).
    pub const ROAD: f32 = 16.0;
    /// Self-lit only at night (floodlit crowns of towers): uv.x = luminance (cd/m²).
    pub const NIGHT_EMISSIVE: f32 = 17.0;
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vert {
    pub pos: [f32; 3],
    pub nrm: [f32; 3],
    pub col: [f32; 3],
    pub uv: [f32; 2],
    pub mat: f32,
}

#[derive(Default)]
pub struct MeshData {
    pub verts: Vec<Vert>,
    pub idx: Vec<u32>,
    /// Material applied to vertices added from now on.
    pub mat: f32,
}

impl MeshData {
    pub fn v(&mut self, p: Vec3, n: Vec3, c: [f32; 3]) -> u32 {
        self.v_uv(p, n, c, [0.0, 0.0])
    }
    pub fn v_uv(&mut self, p: Vec3, n: Vec3, c: [f32; 3], uv: [f32; 2]) -> u32 {
        self.verts.push(Vert { pos: p.into(), nrm: n.into(), col: c, uv, mat: self.mat });
        (self.verts.len() - 1) as u32
    }
    /// Append another mesh (e.g. a model part) transformed by `m`.
    pub fn append(&mut self, other: &MeshData, m: glam::Mat4) {
        let base = self.verts.len() as u32;
        for v in &other.verts {
            let mut v = *v;
            v.pos = m.transform_point3(Vec3::from(v.pos)).into();
            v.nrm = m.transform_vector3(Vec3::from(v.nrm)).normalize_or_zero().into();
            self.verts.push(v);
        }
        self.idx.extend(other.idx.iter().map(|i| i + base));
    }
    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, col: [f32; 3]) {
        let n = (b - a).cross(c - a).normalize_or_zero();
        let i = self.v(a, n, col);
        self.v(b, n, col);
        self.v(c, n, col);
        self.idx.extend([i, i + 1, i + 2]);
    }
    /// Quad a-b-c-d (in order around the edge) with an explicit normal.
    pub fn quad_n(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, n: Vec3, col: [f32; 3]) {
        let i = self.v(a, n, col);
        self.v(b, n, col);
        self.v(c, n, col);
        self.v(d, n, col);
        self.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    }
    /// Flat horizontal quad from four xz corners.
    pub fn flat_quad(&mut self, c: [Vec2; 4], y: f32, col: [f32; 3]) {
        let p = |q: Vec2| vec3(q.x, y, q.y);
        self.quad_n(p(c[0]), p(c[1]), p(c[2]), p(c[3]), Vec3::Y, col);
    }
    /// Oriented rectangle on the ground: centre line from a to b, given width.
    pub fn ground_rect(&mut self, a: Vec2, b: Vec2, width: f32, y: f32, col: [f32; 3]) {
        let d = (b - a).normalize_or_zero();
        let n = vec2(-d.y, d.x) * (width * 0.5);
        self.flat_quad([a + n, b + n, b - n, a - n], y, col);
    }
    /// Triangulated polygon (with holes) at height y.
    pub fn flat_poly(&mut self, outer: &[Vec2], holes: &[Vec<Vec2>], y: f32, n: Vec3, col: [f32; 3]) {
        if outer.len() < 3 {
            return;
        }
        let mut data: Vec<f64> = Vec::with_capacity((outer.len() + 8) * 2);
        let mut hole_idx = Vec::new();
        let mut all: Vec<Vec2> = Vec::new();
        for p in outer {
            data.extend([p.x as f64, p.y as f64]);
            all.push(*p);
        }
        for h in holes {
            if h.len() < 3 {
                continue;
            }
            hole_idx.push(all.len());
            for p in h {
                data.extend([p.x as f64, p.y as f64]);
                all.push(*p);
            }
        }
        let Ok(tris) = earcutr::earcut(&data, &hole_idx, 2) else { return };
        let base = self.verts.len() as u32;
        for p in &all {
            self.v(vec3(p.x, y, p.y), n, col);
        }
        self.idx.extend(tris.into_iter().map(|t| base + t as u32));
    }
    /// Polyline ribbon on the ground.
    pub fn strip(&mut self, pts: &[Vec2], width: f32, y: f32, col: [f32; 3]) {
        for w in pts.windows(2) {
            let d = (w[1] - w[0]).normalize_or_zero();
            let ext = d * width * 0.5; // overlap joints
            self.ground_rect(w[0] - ext, w[1] + ext, width, y, col);
        }
    }
    /// Axis-aligned box.
    pub fn boxm(&mut self, c: Vec3, h: Vec3, col: [f32; 3]) {
        let p = |x: f32, y: f32, z: f32| c + vec3(x * h.x, y * h.y, z * h.z);
        self.quad_n(p(-1., 1., -1.), p(1., 1., -1.), p(1., 1., 1.), p(-1., 1., 1.), Vec3::Y, col);
        self.quad_n(p(-1., -1., -1.), p(-1., -1., 1.), p(1., -1., 1.), p(1., -1., -1.), -Vec3::Y, col);
        self.quad_n(p(1., -1., -1.), p(1., -1., 1.), p(1., 1., 1.), p(1., 1., -1.), Vec3::X, col);
        self.quad_n(p(-1., -1., -1.), p(-1., 1., -1.), p(-1., 1., 1.), p(-1., -1., 1.), -Vec3::X, col);
        self.quad_n(p(-1., -1., 1.), p(-1., 1., 1.), p(1., 1., 1.), p(1., -1., 1.), Vec3::Z, col);
        self.quad_n(p(-1., -1., -1.), p(1., -1., -1.), p(1., 1., -1.), p(-1., 1., -1.), -Vec3::Z, col);
    }
    /// Low-poly ellipsoid (smooth normals).
    pub fn blob(&mut self, c: Vec3, r: Vec3, col: [f32; 3]) {
        self.blob_lod(c, r, col, 10, 6);
    }
    /// Ellipsoid with a chosen tessellation.
    pub fn blob_lod(&mut self, c: Vec3, r: Vec3, col: [f32; 3], nu: u32, nv: u32) {
        let base = self.verts.len() as u32;
        for j in 0..=nv {
            let th = std::f32::consts::PI * j as f32 / nv as f32;
            for i in 0..nu {
                let ph = std::f32::consts::TAU * i as f32 / nu as f32;
                let n = vec3(th.sin() * ph.cos(), th.cos(), th.sin() * ph.sin());
                self.v(c + n * r, (n / r).normalize(), col);
            }
        }
        for j in 0..nv {
            for i in 0..nu {
                let a = base + j * nu + i;
                let b = base + j * nu + (i + 1) % nu;
                let c2 = a + nu;
                let d = b + nu;
                self.idx.extend([a, c2, b, b, c2, d]);
            }
        }
    }
}

impl MeshData {
    /// Polyline ribbon whose ends are only extended (to cover joints) where
    /// they were not cut by the tile boundary, so pieces meet exactly at seams.
    pub fn strip_cut(&mut self, pts: &[Vec2], width: f32, y: f32, col: [f32; 3], cut: [bool; 2]) {
        let n = pts.len();
        for (i, w) in pts.windows(2).enumerate() {
            let d = (w[1] - w[0]).normalize_or_zero();
            let ext = d * width * 0.5;
            let a = if i == 0 && cut[0] { w[0] } else { w[0] - ext };
            let b = if i + 2 == n && cut[1] { w[1] } else { w[1] + ext };
            self.ground_rect(a, b, width, y, col);
        }
    }
}

impl MeshData {
    /// Road ribbon carrying (along, across + width code) uvs for lane markings.
    pub fn strip_road(&mut self, pts: &[Vec2], width: f32, col: [f32; 3], cut: [bool; 2]) {
        let n = pts.len();
        let code = 1000.0 * (width * 10.0).round();
        // Dash phase from the world position so pieces from different tiles line up roughly.
        let mut along = pts.first().map_or(0.0, |p| (p.x * 0.7 + p.y * 1.3).rem_euclid(9.0));
        for (i, w) in pts.windows(2).enumerate() {
            let d = (w[1] - w[0]).normalize_or_zero();
            let ext = d * width * 0.5;
            let a = if i == 0 && cut[0] { w[0] } else { w[0] - ext };
            let b = if i + 2 == n && cut[1] { w[1] } else { w[1] + ext };
            let nn = vec2(-d.y, d.x) * (width * 0.5);
            let a0 = along - (w[0] - a).length();
            let b0 = along + (b - w[0]).length();
            let hw = width * 0.5;
            let p = |q: Vec2| vec3(q.x, 0.0, q.y);
            let base = self.verts.len() as u32;
            self.v_uv(p(a + nn), Vec3::Y, col, [a0, hw + code]);
            self.v_uv(p(b + nn), Vec3::Y, col, [b0, hw + code]);
            self.v_uv(p(b - nn), Vec3::Y, col, [b0, -hw + code]);
            self.v_uv(p(a - nn), Vec3::Y, col, [a0, -hw + code]);
            self.idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
            along += (w[1] - w[0]).length();
        }
    }
}

// ---------------------------------------------------------------- colours

pub const LAND: [f32; 3] = [0.56, 0.55, 0.50];
const GRASS: [f32; 3] = [0.34, 0.43, 0.25];
const PARK: [f32; 3] = [0.31, 0.41, 0.22];
const AERODROME: [f32; 3] = [0.36, 0.44, 0.26];
const SCRUB: [f32; 3] = [0.34, 0.40, 0.24];
const WETLAND: [f32; 3] = [0.33, 0.42, 0.30];
const SAND: [f32; 3] = [0.70, 0.64, 0.50];
const ROCK: [f32; 3] = [0.55, 0.54, 0.52];
const WOOD: [f32; 3] = [0.22, 0.30, 0.16];
const WATER: [f32; 3] = [0.20, 0.36, 0.50];
const APRON: [f32; 3] = [0.52, 0.52, 0.52];
const ROAD: [f32; 3] = [0.34, 0.34, 0.36];
const MAJOR_ROAD: [f32; 3] = [0.40, 0.39, 0.37];
const TRACK: [f32; 3] = [0.48, 0.43, 0.34];
const RAIL: [f32; 3] = [0.33, 0.30, 0.28];
const TAXI: [f32; 3] = [0.40, 0.40, 0.41];
const RUNWAY: [f32; 3] = [0.24, 0.24, 0.26];
const PAINT: [f32; 3] = [0.80, 0.80, 0.79];
const TAXI_LINE: [f32; 3] = [0.85, 0.70, 0.15];
/// Urban ground by landuse class (residential, commercial, industrial, institutional),
/// and for dense-building cells.
const URBAN: [[f32; 3]; 4] = [[0.50, 0.49, 0.45], [0.54, 0.52, 0.50], [0.52, 0.51, 0.50], [0.52, 0.52, 0.48]];
const URBAN_DENSE: [f32; 3] = [0.53, 0.52, 0.49];

pub fn hash(mut x: u64) -> u64 {
    x ^= x >> 33;
    x = x.wrapping_mul(0xff51afd7ed558ccd);
    x ^= x >> 33;
    x = x.wrapping_mul(0xc4ceb9fe1a85ec53);
    x ^ (x >> 33)
}

struct Rng(u64);
impl Rng {
    fn f(&mut self) -> f32 {
        self.0 = hash(self.0.wrapping_add(0x9e3779b97f4a7c15));
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
    fn range(&mut self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.f()
    }
}

fn building_colour(key: u64, height: f32) -> [f32; 3] {
    const NORMAL: [[f32; 3]; 7] = [
        [0.80, 0.78, 0.72],
        [0.72, 0.66, 0.58],
        [0.64, 0.50, 0.42],
        [0.86, 0.85, 0.82],
        [0.62, 0.63, 0.66],
        [0.76, 0.72, 0.64],
        [0.58, 0.44, 0.38],
    ];
    const GLASS: [[f32; 3]; 4] = [[0.50, 0.60, 0.70], [0.58, 0.66, 0.72], [0.70, 0.72, 0.75], [0.45, 0.52, 0.58]];
    let h = hash(key) as usize;
    if height > 45.0 {
        GLASS[h % GLASS.len()]
    } else {
        NORMAL[h % NORMAL.len()]
    }
}

// ---------------------------------------------------------------- lookup grid

struct Grid {
    origin: Vec2,
    cell: f32,
    nx: usize,
    nz: usize,
    cells: Vec<Vec<u32>>,
}

impl Grid {
    fn new(min: Vec2, max: Vec2, n: usize) -> Self {
        let cell = ((max.x - min.x).max(max.y - min.y) / n as f32).max(1.0);
        let nx = (((max.x - min.x) / cell).ceil() as usize).max(1);
        let nz = (((max.y - min.y) / cell).ceil() as usize).max(1);
        Grid { origin: min, cell, nx, nz, cells: vec![Vec::new(); nx * nz] }
    }
    fn range(&self, lo: Vec2, hi: Vec2) -> Option<(usize, usize, usize, usize)> {
        let a = ((lo - self.origin) / self.cell).floor();
        let b = ((hi - self.origin) / self.cell).floor();
        if b.x < 0.0 || b.y < 0.0 || a.x >= self.nx as f32 || a.y >= self.nz as f32 {
            return None;
        }
        Some((a.x.max(0.0) as usize, a.y.max(0.0) as usize, (b.x as usize).min(self.nx - 1), (b.y as usize).min(self.nz - 1)))
    }
    fn insert(&mut self, lo: Vec2, hi: Vec2, item: u32) {
        if let Some((x0, z0, x1, z1)) = self.range(lo, hi) {
            for z in z0..=z1 {
                for x in x0..=x1 {
                    self.cells[z * self.nx + x].push(item);
                }
            }
        }
    }
    fn at(&self, p: Vec2) -> &[u32] {
        match self.range(p, p) {
            Some((x, z, _, _)) => &self.cells[z * self.nx + x],
            None => &[],
        }
    }
}

fn bbox(pts: &[Vec2]) -> (Vec2, Vec2) {
    let mut lo = Vec2::splat(f32::MAX);
    let mut hi = Vec2::splat(f32::MIN);
    for p in pts {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    (lo, hi)
}

fn ring_area(pts: &[Vec2]) -> f32 {
    let n = pts.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (pts[i], pts[(i + 1) % n]);
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

// ---------------------------------------------------------------- collision

struct Solid {
    pts: Vec<Vec2>,
    lo: Vec2,
    hi: Vec2,
    top: f32,
}

struct WaterPoly {
    outer: Vec<Vec2>,
    holes: Vec<Vec<Vec2>>,
    lo: Vec2,
    hi: Vec2,
}

/// Per-tile collision lookups (building footprints + water), world space.
pub struct TileCollision {
    solids: Vec<Solid>,
    sgrid: Grid,
    water: Vec<WaterPoly>,
    wgrid: Grid,
}

impl TileCollision {
    /// Highest building top at this point, if inside a footprint.
    pub fn obstacle_top(&self, p: Vec2) -> Option<f32> {
        let mut top: Option<f32> = None;
        for &i in self.sgrid.at(p) {
            let s = &self.solids[i as usize];
            if p.x >= s.lo.x && p.x <= s.hi.x && p.y >= s.lo.y && p.y <= s.hi.y && point_in_ring(p, &s.pts) {
                top = Some(top.map_or(s.top, |t: f32| t.max(s.top)));
            }
        }
        top
    }
    pub fn is_water(&self, p: Vec2) -> bool {
        self.wgrid.at(p).iter().any(|&i| {
            let w = &self.water[i as usize];
            p.x >= w.lo.x
                && p.x <= w.hi.x
                && p.y >= w.lo.y
                && p.y <= w.hi.y
                && point_in_ring(p, &w.outer)
                && !w.holes.iter().any(|h| point_in_ring(p, h))
        })
    }
}

// ---------------------------------------------------------------- raster

/// Scanline polygon fill in pixel space (even-odd over all rings).
fn fill_rings(w: i32, h: i32, rings: &[Vec<Vec2>], mut set: impl FnMut(i32, i32)) {
    let (mut y0, mut y1) = (f32::MAX, f32::MIN);
    for r in rings {
        for p in r {
            y0 = y0.min(p.y);
            y1 = y1.max(p.y);
        }
    }
    let (y0, y1) = ((y0.floor() as i32).max(0), (y1.ceil() as i32).min(h - 1));
    let mut xs = Vec::new();
    for y in y0..=y1 {
        let yc = y as f32 + 0.5;
        xs.clear();
        for r in rings {
            let n = r.len();
            for i in 0..n {
                let (a, b) = (r[i], r[(i + 1) % n]);
                if (a.y > yc) != (b.y > yc) {
                    xs.push(a.x + (yc - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
        }
        xs.sort_by(|a, b| a.total_cmp(b));
        for pair in xs.chunks(2) {
            if pair.len() < 2 {
                break;
            }
            let x0 = ((pair[0] - 0.5).ceil() as i32).max(0);
            let x1 = ((pair[1] - 0.5).floor() as i32).min(w - 1);
            for x in x0..=x1 {
                set(x, y);
            }
        }
    }
}

/// Maps world space onto a square raster covering the tile rect.
#[derive(Clone, Copy)]
struct RasterXf {
    lo: Vec2,
    scale: Vec2,
    n: i32,
}

impl RasterXf {
    fn new(rect: (Vec2, Vec2), n: u32) -> Self {
        RasterXf { lo: rect.0, scale: Vec2::splat(n as f32) / (rect.1 - rect.0), n: n as i32 }
    }
    fn px(&self, p: Vec2) -> Vec2 {
        (p - self.lo) * self.scale
    }
    fn ring(&self, r: &[Vec2]) -> Vec<Vec2> {
        r.iter().map(|p| self.px(*p)).collect()
    }
    fn fill(&self, outer: &[Vec2], holes: &[Vec<Vec2>], set: impl FnMut(i32, i32)) {
        let mut rings = vec![self.ring(outer)];
        rings.extend(holes.iter().map(|h| self.ring(h)));
        fill_rings(self.n, self.n, &rings, set);
    }
    fn stroke(&self, pts: &[Vec2], width_m: f32, mut set: impl FnMut(i32, i32)) {
        let hw = (width_m * 0.5 * self.scale.x).max(0.55);
        for w in pts.windows(2) {
            let (a, b) = (self.px(w[0]), self.px(w[1]));
            let d = (b - a).normalize_or_zero();
            let nrm = vec2(-d.y, d.x) * hw;
            let (a, b) = (a - d * hw * 0.5, b + d * hw * 0.5);
            fill_rings(self.n, self.n, &[vec![a + nrm, b + nrm, b - nrm, a - nrm]], &mut set);
        }
    }
}

// ---------------------------------------------------------------- trees

/// One tree instance: position, height, foliage tint, rotation about Y.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TreeInst {
    pub pos: [f32; 3],
    pub height: f32,
    pub tint: [f32; 3],
    pub rot: f32,
}

/// Unit-height tree mesh (foliage vertices are near-white and tinted per instance).
/// `lod` 0 = detailed, 1 = distant. Foliage normals point away from the crown
/// centre (blended with the lobe normal), so a crown shades as one soft volume.
pub fn tree_mesh(conifer: bool, lod: u32) -> MeshData {
    let mut m = MeshData::default();
    let trunk = [0.30, 0.24, 0.18];
    // Lobe with normals bent toward the crown's radial direction.
    fn lobe(m: &mut MeshData, c: Vec3, r: Vec3, crown: Vec3, col: [f32; 3], nu: u32, nv: u32, seed: u64) {
        let base = m.verts.len() as u32;
        let mut rng = Rng(hash(seed));
        for j in 0..=nv {
            let th = std::f32::consts::PI * j as f32 / nv as f32;
            for i in 0..nu {
                let ph = std::f32::consts::TAU * i as f32 / nu as f32;
                let dir = vec3(th.sin() * ph.cos(), th.cos(), th.sin() * ph.sin());
                // Lumpy surface.
                let k = if j == 0 || j == nv { 1.0 } else { rng.range(0.82, 1.12) };
                let p = c + dir * r * k;
                let own = (dir / r).normalize();
                let radial = (p - crown).normalize_or_zero();
                let n = (own * 0.35 + radial * 0.65).normalize();
                m.v(p, n, col);
            }
        }
        for j in 0..nv {
            for i in 0..nu {
                let a = base + j * nu + i;
                let b = base + j * nu + (i + 1) % nu;
                m.idx.extend([a, a + nu, b, b, a + nu, b + nu]);
            }
        }
    }
    if conifer {
        let sides = if lod == 0 { 9 } else { 6 };
        if lod == 0 {
            m.mat = mat::DIFFUSE;
            m.boxm(vec3(0.0, 0.1, 0.0), vec3(0.022, 0.1, 0.022), trunk);
        }
        m.mat = mat::FOLIAGE;
        let tiers: &[(f32, f32, f32)] =
            if lod == 0 { &[(0.10, 0.45, 0.25), (0.28, 0.62, 0.21), (0.45, 0.80, 0.16), (0.62, 1.0, 0.11)] } else { &[(0.08, 0.6, 0.24), (0.42, 1.0, 0.15)] };
        for (t, &(y0, y1, r)) in tiers.iter().enumerate() {
            let base = m.verts.len() as u32;
            let apex = vec3(0.0, y1, 0.0);
            let shade = 0.85 + 0.05 * t as f32;
            for i in 0..sides {
                let a = std::f32::consts::TAU * (i as f32 + 0.5 * t as f32) / sides as f32;
                // Drooping skirt: the rim sits a little below the tier base.
                let rr = r * if i % 2 == 0 { 1.0 } else { 0.86 };
                let p = vec3(a.cos() * rr, y0 - 0.03, a.sin() * rr);
                let n = vec3(a.cos(), 0.9, a.sin()).normalize();
                m.v(p, n, [shade, shade, shade]);
                m.v(apex, Vec3::Y, [shade, shade, shade]);
            }
            for i in 0..sides {
                let j = (i + 1) % sides;
                m.idx.extend([base + 2 * i, base + 2 * i + 1, base + 2 * j]);
            }
        }
    } else {
        let crown = vec3(0.0, 0.66, 0.0);
        if lod == 0 {
            m.mat = mat::DIFFUSE;
            m.boxm(vec3(0.0, 0.2, 0.0), vec3(0.024, 0.2, 0.024), trunk);
            m.mat = mat::FOLIAGE;
            // Irregular crown of overlapping lobes.
            let lobes: [(Vec3, f32, [f32; 3]); 7] = [
                (vec3(0.0, 0.58, 0.0), 0.30, [0.92, 0.92, 0.92]),
                (vec3(0.17, 0.64, 0.06), 0.22, [1.0, 1.02, 0.95]),
                (vec3(-0.15, 0.62, -0.10), 0.23, [0.88, 0.9, 0.88]),
                (vec3(0.04, 0.80, -0.06), 0.21, [1.06, 1.08, 1.0]),
                (vec3(-0.07, 0.70, 0.17), 0.20, [0.98, 1.0, 0.94]),
                (vec3(0.12, 0.50, -0.17), 0.19, [0.85, 0.86, 0.86]),
                (vec3(-0.02, 0.92, 0.03), 0.13, [1.1, 1.1, 1.0]),
            ];
            for (k, (c, r, col)) in lobes.iter().enumerate() {
                lobe(&mut m, *c, vec3(*r, *r * 0.88, *r), crown, *col, 8, 5, k as u64 + 11);
            }
        } else {
            m.mat = mat::FOLIAGE;
            lobe(&mut m, vec3(0.0, 0.62, 0.0), vec3(0.36, 0.33, 0.36), crown, [0.95, 0.95, 0.95], 7, 4, 3);
            lobe(&mut m, vec3(0.05, 0.82, -0.03), vec3(0.22, 0.17, 0.22), crown, [1.05, 1.06, 1.0], 6, 3, 4);
        }
    }
    // Foliage uv.x = 1 marks conifers (evergreen: no seasonal leaf colour /
    // leaf fall; the tree vertex shader replaces foliage uvs anyway).
    if conifer {
        for v in m.verts.iter_mut().filter(|v| v.mat == mat::FOLIAGE) {
            v.uv = [1.0, 0.0];
        }
    }
    m
}

fn tree_tint(rng: &mut Rng, conifer: bool) -> [f32; 3] {
    let s = rng.f();
    let v = rng.f();
    if conifer {
        [0.11 + 0.04 * s, 0.17 + 0.05 * s, 0.11 + 0.03 * s]
    } else {
        // From dark summer oak to lighter, yellower lime / birch greens.
        let dark = [0.13, 0.19, 0.08];
        let light = [0.26, 0.32, 0.13];
        let k = s * s;
        let o = 0.02 * (v - 0.5);
        [dark[0] + (light[0] - dark[0]) * k + o, dark[1] + (light[1] - dark[1]) * k, dark[2] + (light[2] - dark[2]) * k - o]
    }
}

// ---------------------------------------------------------------- tile build

/// Everything a worker builds for one map tile.
pub struct TileBuild {
    pub id: TileId,
    /// Tallest building (m), for culling bounds.
    pub max_h: f32,
    /// Ground layers, drawn in order with no depth test (painter's).
    pub flat: MeshData,
    pub buildings: MeshData,
    /// Deciduous trees first, then `conifers` conifers at the end.
    pub trees: Vec<TreeInst>,
    pub conifers: usize,
    pub collision: TileCollision,
    pub runways: Vec<RunwayPiece>,
    pub airports: Vec<Airport>,
    pub building_count: usize,
    /// Street lights, traffic, warning lights, skyglow sources (lights.rs).
    pub lights: crate::lights::TileLights,
}

const DENSITY_N: usize = 24;
const MASK_N: u32 = 128;
const MAX_TREES: usize = 9000;

fn sel<'a>(f: &'a TileFeatures, pred: impl Fn(&Kind) -> bool + 'a) -> impl Iterator<Item = &'a Area> + 'a {
    f.areas.iter().filter(move |a| pred(&a.kind))
}

pub fn build_tile(id: TileId, proj: &Proj, f: TileFeatures) -> TileBuild {
    let rect = id.rect(proj);
    let (lo, hi) = rect;
    let mut flat = MeshData::default();
    let mut bld = MeshData::default();
    let mut lights = crate::lights::place::CityGen::new(id, proj, rect);
    let rx_mask = RasterXf::new(rect, MASK_N);
    let mut mask = vec![0u8; (MASK_N * MASK_N) as usize];
    let mut mask_set = |x: i32, y: i32| mask[(y as u32 * MASK_N + x as u32) as usize] = 1;


    // --- opaque field ground over the whole tile (hides the far-field layer)
    flat.mat = mat::GROUND;
    flat.flat_quad([lo, vec2(hi.x, lo.y), hi, vec2(lo.x, hi.y)], 0.0, LAND);

    // --- building density → urban ground cells
    let mut density = vec![0f32; DENSITY_N * DENSITY_N];
    let cell = (hi - lo) / DENSITY_N as f32;
    for a in sel(&f, |k| matches!(k, Kind::Building { .. })) {
        let (blo, bhi) = bbox(&a.outer);
        let c = ((blo + bhi) * 0.5 - lo) / cell;
        let (cx, cz) = (c.x.floor() as i32, c.y.floor() as i32);
        if (0..DENSITY_N as i32).contains(&cx) && (0..DENSITY_N as i32).contains(&cz) {
            density[cz as usize * DENSITY_N + cx as usize] += ring_area(&a.outer).abs() / (cell.x * cell.y);
        }
    }
    // Built-up cells (buildings or urban landuse, grown by one cell): only
    // their roads get street lights — country roads stay dark.
    {
        let mut built = vec![false; DENSITY_N * DENSITY_N];
        for z in 0..DENSITY_N as i32 {
            for x in 0..DENSITY_N as i32 {
                let mut any = false;
                for dz in -1i32..=1 {
                    for dx in -1i32..=1 {
                        let (xx, zz) = (x + dx, z + dz);
                        if (0..DENSITY_N as i32).contains(&xx) && (0..DENSITY_N as i32).contains(&zz) {
                            any |= density[zz as usize * DENSITY_N + xx as usize] > 0.02;
                        }
                    }
                }
                built[z as usize * DENSITY_N + x as usize] = any;
            }
        }
        let rx_d = RasterXf::new(rect, DENSITY_N as u32);
        for a in sel(&f, |k| matches!(k, Kind::Urban(_))) {
            rx_d.fill(&a.outer, &a.holes, |x, y| built[y as usize * DENSITY_N + x as usize] = true);
        }
        lights.set_built(built, DENSITY_N);
    }
    flat.mat = mat::GROUND;
    for z in 0..DENSITY_N {
        let mut run: Option<usize> = None;
        for x in 0..=DENSITY_N {
            let dense = x < DENSITY_N && {
                let mut s = 0.0;
                let mut n = 0.0;
                for dz in -1i32..=1 {
                    for dx in -1i32..=1 {
                        let (xx, zz) = (x as i32 + dx, z as i32 + dz);
                        if (0..DENSITY_N as i32).contains(&xx) && (0..DENSITY_N as i32).contains(&zz) {
                            let w = if dx == 0 && dz == 0 { 2.0 } else { 1.0 };
                            s += density[zz as usize * DENSITY_N + xx as usize] * w;
                            n += w;
                        }
                    }
                }
                s / n > 0.09
            };
            match (dense, run) {
                (true, None) => run = Some(x),
                (false, Some(x0)) => {
                    let a = lo + vec2(x0 as f32, z as f32) * cell;
                    let b = lo + vec2(x as f32, z as f32 + 1.0) * cell;
                    let i = flat.v_uv(vec3(a.x, 0.0, a.y), Vec3::Y, URBAN_DENSE, [1.0, 0.0]);
                    flat.v_uv(vec3(b.x, 0.0, a.y), Vec3::Y, URBAN_DENSE, [1.0, 0.0]);
                    flat.v_uv(vec3(b.x, 0.0, b.y), Vec3::Y, URBAN_DENSE, [1.0, 0.0]);
                    flat.v_uv(vec3(a.x, 0.0, b.y), Vec3::Y, URBAN_DENSE, [1.0, 0.0]);
                    flat.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
                    run = None;
                }
                _ => {}
            }
        }
    }

    // --- landuse (urban ground)
    for a in sel(&f, |k| matches!(k, Kind::Urban(_))) {
        let Kind::Urban(c) = a.kind else { continue };
        let col = URBAN[c as usize % URBAN.len()];
        let base = flat.verts.len();
        flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, col);
        for v in &mut flat.verts[base..] {
            v.uv = [1.0, 0.0];
        }
    }

    // --- natural ground cover (order: big/low first)
    flat.mat = mat::GRASS;
    let ground_layers: [(Kind, [f32; 3], f32); 7] = [
        (Kind::Aerodrome, AERODROME, mat::GRASS),
        (Kind::Grass, GRASS, mat::GRASS),
        (Kind::Wetland, WETLAND, mat::GRASS),
        (Kind::Scrub, SCRUB, mat::GRASS),
        (Kind::Park, PARK, mat::GRASS),
        (Kind::Sand, SAND, mat::DIFFUSE),
        (Kind::Rock, ROCK, mat::DIFFUSE),
    ];
    for (kind, col, m) in ground_layers {
        flat.mat = m;
        let pred = move |k: &Kind| *k == kind;
        for a in sel(&f, pred) {
            flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, col);
        }
    }
    flat.mat = mat::GRASS;
    for a in sel(&f, |k| *k == Kind::Wood) {
        flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, WOOD);
    }

    // --- water
    flat.mat = mat::WATER;
    for a in sel(&f, |k| *k == Kind::Water) {
        flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, WATER);
        rx_mask.fill(&a.outer, &a.holes, &mut mask_set);
    }
    for l in &f.lines {
        if let Kind::Waterway { width } = l.kind {
            flat.strip_cut(&l.pts, width, 0.0, WATER, l.cut);
            rx_mask.stroke(&l.pts, width + 2.0, &mut mask_set);
        }
    }

    // --- paved areas
    flat.mat = mat::ASPHALT;
    for a in sel(&f, |k| matches!(k, Kind::Pier | Kind::Apron | Kind::RunwayArea)) {
        let col = if a.kind == Kind::RunwayArea { RUNWAY } else { APRON };
        if a.kind == Kind::Apron && ring_area(&a.outer).abs() > 8000.0 {
            lights.apron(&a.outer);
        }
        flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, col);
        rx_mask.fill(&a.outer, &a.holes, &mut mask_set);
    }
    for a in sel(&f, |k| *k == Kind::Aerodrome) {
        rx_mask.fill(&a.outer, &a.holes, &mut mask_set);
    }

    // --- linear features, minor first so majors draw on top
    let line_order = |k: &Kind| -> Option<(u8, f32, [f32; 3])> {
        Some(match *k {
            Kind::Track { width } => (0, width, TRACK),
            Kind::Rail => (1, 3.2, RAIL),
            Kind::Road { width, major: false } => (2 + (width > 7.0) as u8, width, ROAD),
            Kind::Road { width, major: true } => (4 + (width > 15.0) as u8, width, MAJOR_ROAD),
            _ => return None,
        })
    };
    for pass in 0..6u8 {
        for l in &f.lines {
            let Some((order, width, col)) = line_order(&l.kind) else { continue };
            if order != pass {
                continue;
            }
            if matches!(l.kind, Kind::Rail | Kind::Track { .. }) {
                flat.mat = mat::DIFFUSE;
                flat.strip_cut(&l.pts, width, 0.0, col, l.cut);
            } else {
                flat.mat = mat::ROAD;
                flat.strip_road(&l.pts, width, col, l.cut);
                if let Kind::Road { width, major } = l.kind {
                    lights.road(&l.pts, width, major);
                }
            }
            rx_mask.stroke(&l.pts, width + 3.0, &mut mask_set);
        }
    }
    flat.mat = mat::ASPHALT;
    for l in &f.lines {
        if let Kind::Taxiway { width } = l.kind {
            lights.taxiway(&l.pts, width);
            flat.strip_cut(&l.pts, width, 0.0, TAXI, l.cut);
            rx_mask.stroke(&l.pts, width + 10.0, &mut mask_set);
        }
    }
    for l in &f.lines {
        if let Kind::Taxiway { .. } = l.kind {
            flat.strip_cut(&l.pts, 0.5, 0.0, TAXI_LINE, l.cut);
        }
    }

    // --- buildings + collision
    let mut solids = Vec::new();
    let mut sgrid = Grid::new(lo, hi, 16);
    let mut building_count = 0;
    let mut max_h: f32 = 0.0;
    for a in sel(&f, |k| matches!(k, Kind::Building { .. })) {
        let Kind::Building { height, min_height, colour } = a.kind else { continue };
        if a.outer.len() < 3 {
            continue;
        }
        building(&mut bld, a, height, min_height, colour);
        lights.tall_building(&a.outer, height);
        building_count += 1;
        max_h = max_h.max(height);
        let (blo, bhi) = bbox(&a.outer);
        sgrid.insert(blo, bhi, solids.len() as u32);
        solids.push(Solid { pts: a.outer.clone(), lo: blo, hi: bhi, top: height });
        rx_mask.fill(&a.outer, &[], &mut mask_set);
    }
    let mut water = Vec::new();
    let mut wgrid = Grid::new(lo, hi, 8);
    for a in sel(&f, |k| *k == Kind::Water) {
        let (wlo, whi) = bbox(&a.outer);
        wgrid.insert(wlo, whi, water.len() as u32);
        water.push(WaterPoly { outer: a.outer.clone(), holes: a.holes.clone(), lo: wlo, hi: whi });
    }
    // Keep trees clear of runway centrelines (pieces known to this tile).
    for r in &f.runways {
        rx_mask.stroke(&[r.a, r.b], 260.0, &mut mask_set);
    }

    // --- trees
    let mut rng = Rng(hash(id.seed()));
    let mut decid = Vec::new();
    let mut conif = Vec::new();
    let wood_area: f32 = sel(&f, |k| *k == Kind::Wood).map(|a| ring_area(&a.outer).abs()).sum();
    // Woods: ~1 tree per 200 m², thinned if the tile is mostly forest.
    let wood_density = (1.0 / 200.0f32).min(MAX_TREES as f32 * 0.85 / wood_area.max(1.0));
    let mask_at = |p: Vec2| {
        let q = rx_mask.px(p);
        let (x, y) = (q.x as i32, q.y as i32);
        x < 0 || y < 0 || x >= MASK_N as i32 || y >= MASK_N as i32 || mask[(y as u32 * MASK_N + x as u32) as usize] != 0
    };
    for a in &f.areas {
        let (density, conifer_frac, hmin, hmax) = match a.kind {
            Kind::Wood => (wood_density, 0.3, 9.0, 19.0),
            Kind::Scrub => (1.0 / 900.0, 0.1, 3.0, 7.0),
            Kind::Park => (1.0 / 2200.0, 0.1, 8.0, 16.0),
            Kind::Wetland => (1.0 / 3000.0, 0.0, 4.0, 9.0),
            Kind::Urban(0) => (1.0 / 1800.0, 0.05, 6.0, 13.0),
            _ => continue,
        };
        let area = ring_area(&a.outer).abs();
        let want = ((area * density) as usize).min(MAX_TREES.saturating_sub(decid.len() + conif.len()));
        if want == 0 {
            continue;
        }
        let (blo, bhi) = bbox(&a.outer);
        let fill = (area / ((bhi.x - blo.x) * (bhi.y - blo.y)).max(1.0)).clamp(0.05, 1.0);
        let tries = ((want as f32 / fill) * 1.3) as usize + 4;
        let mut placed = 0;
        for _ in 0..tries {
            if placed >= want {
                break;
            }
            let p = vec2(rng.range(blo.x, bhi.x), rng.range(blo.y, bhi.y));
            if mask_at(p) || !point_in_ring(p, &a.outer) || a.holes.iter().any(|h| point_in_ring(p, h)) {
                continue;
            }
            let conifer = rng.f() < conifer_frac;
            let t = TreeInst { pos: [p.x, 0.0, p.y], height: rng.range(hmin, hmax), tint: tree_tint(&mut rng, conifer), rot: rng.range(0.0, std::f32::consts::TAU) };
            if conifer { conif.push(t) } else { decid.push(t) }
            placed += 1;
        }
    }
    let conifers = conif.len();
    decid.extend(conif);

    TileBuild {
        id,
        max_h,
        flat,
        buildings: bld,
        trees: decid,
        conifers,
        collision: TileCollision { solids, sgrid, water, wgrid },
        runways: f.runways,
        airports: f.airports,
        building_count,
        lights: lights.finish(),
    }
}

/// Far-field tile (low zoom): generalised ground layers + city lights.
pub fn build_far_tile(id: TileId, proj: &Proj, f: TileFeatures) -> TileBuild {
    let rect = id.rect(proj);
    let (lo, hi) = rect;
    let mut flat = MeshData::default();
    let mut lights = crate::lights::place::CityGen::new(id, proj, rect);
    {
        // Lit roads only in and around built-up areas.
        const N: usize = 64;
        let rx_u = RasterXf::new(rect, N as u32);
        let mut built = vec![false; N * N];
        for a in sel(&f, |k| matches!(k, Kind::Urban(_))) {
            rx_u.fill(&a.outer, &a.holes, |x, y| built[y as usize * N + x as usize] = true);
        }
        lights.set_built(built, N);
    }
    for a in sel(&f, |k| matches!(k, Kind::Urban(_))) {
        let Kind::Urban(c) = a.kind else { continue };
        lights.far_urban(&a.outer, &a.holes, c, a.key);
        let base = flat.verts.len();
        flat.mat = mat::GROUND;
        flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, URBAN[c as usize % URBAN.len()]);
        for v in &mut flat.verts[base..] {
            v.uv = [1.0, 0.0];
        }
    }
    let layers: [(Kind, [f32; 3], f32); 8] = [
        (Kind::Aerodrome, AERODROME, mat::GRASS),
        (Kind::Grass, GRASS, mat::GRASS),
        (Kind::Park, PARK, mat::GRASS),
        (Kind::Scrub, SCRUB, mat::GRASS),
        (Kind::Wetland, WETLAND, mat::GRASS),
        (Kind::Sand, SAND, mat::DIFFUSE),
        (Kind::Wood, WOOD, mat::GRASS),
        (Kind::Water, WATER, mat::WATER),
    ];
    for (kind, col, m) in layers {
        flat.mat = m;
        for a in sel(&f, move |k| *k == kind) {
            flat.flat_poly(&a.outer, &a.holes, 0.0, Vec3::Y, col);
        }
    }
    for l in &f.lines {
        let (w, col, m) = match l.kind {
            Kind::Waterway { width } if width >= 10.0 => (width * 1.5, WATER, mat::WATER),
            Kind::Road { width, major: true } => (width, MAJOR_ROAD, mat::ASPHALT),
            _ => continue,
        };
        if let Kind::Road { width, .. } = l.kind {
            lights.far_road(&l.pts, width);
        }
        flat.mat = m;
        flat.strip_cut(&l.pts, w, 0.0, col, l.cut);
    }
    TileBuild {
        id,
        max_h: 0.0,
        flat,
        buildings: MeshData::default(),
        trees: Vec::new(),
        conifers: 0,
        collision: TileCollision { solids: Vec::new(), sgrid: Grid::new(lo, hi, 1), water: Vec::new(), wgrid: Grid::new(lo, hi, 1) },
        runways: Vec::new(),
        airports: Vec::new(),
        building_count: 0,
        lights: lights.finish(),
    }
}

/// Pitched-roof plan for a small near-rectangular footprint: ring indices
/// rotated so edge 0-1 runs along the ridge, roof rise (m), hipped?
fn pitched_plan(a: &Area, height: f32, min_h: f32) -> Option<([Vec2; 4], f32, bool)> {
    let mut pts: Vec<Vec2> = a.outer.clone();
    if pts.len() > 1 && pts[0].distance(pts[pts.len() - 1]) < 0.01 {
        pts.pop();
    }
    if !(4..=10).contains(&pts.len()) || !a.holes.is_empty() || min_h > 0.5 || height > 16.0 || a.border.iter().any(|b| *b) {
        return None;
    }
    let area = ring_area(&pts).abs();
    if !(20.0..=450.0).contains(&area) {
        return None;
    }
    // Oriented box along the longest edge; the footprint must nearly fill it.
    let n = pts.len();
    let (mut best, mut dir) = (0.0, Vec2::X);
    for i in 0..n {
        let e = pts[(i + 1) % n] - pts[i];
        if e.length() > best {
            best = e.length();
            dir = e / best;
        }
    }
    let side = vec2(-dir.y, dir.x);
    let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
    for p in &pts {
        let q = vec2(p.dot(dir), p.dot(side));
        lo = lo.min(q);
        hi = hi.max(q);
    }
    let ext = hi - lo;
    if area < 0.82 * ext.x * ext.y || ext.min_element() < 3.0 {
        return None;
    }
    let h = hash(a.key ^ 0x5eed);
    if h % 100 < 10 {
        return None; // some flat-roofed extensions / garages
    }
    let w = |x: f32, y: f32| dir * x + side * y;
    let mut q = [w(lo.x, lo.y), w(hi.x, lo.y), w(hi.x, hi.y), w(lo.x, hi.y)];
    if ext.y > ext.x {
        q = [q[1], q[2], q[3], q[0]];
    }
    let short = ext.min_element();
    let pitch = (30.0 + (h >> 8) as f32 % 15.0).to_radians();
    let rise = (short * 0.5 * pitch.tan()).min(height * 0.45).max(1.0);
    Some((q, rise, (h >> 16) % 3 == 0))
}

fn building(m: &mut MeshData, a: &Area, height: f32, min_h: f32, colour: Option<[f32; 3]>) {
    let col = match colour {
        // OSM colours are often garish; pull them toward a neutral facade.
        Some(c) => {
            let g = (c[0] + c[1] + c[2]) / 3.0;
            let k = 0.55;
            [(g + (c[0] - g) * k) * 0.8 + 0.12, (g + (c[1] - g) * k) * 0.8 + 0.12, (g + (c[2] - g) * k) * 0.8 + 0.12]
        }
        None => building_colour(a.key, height),
    };
    let base_col = [col[0] * 0.9, col[1] * 0.9, col[2] * 0.9];
    let roof = [col[0] * 0.78, col[1] * 0.78, col[2] * 0.80];
    let pitched = pitched_plan(a, height, min_h);
    let eaves = pitched.map_or(height, |(_, rise, _)| height - rise);
    m.mat = if height > 45.0 { mat::GLASS } else { mat::WALL };
    // uv.x: per-building seed in 4096 m blocks (facade style) + window phase.
    let hk = hash(a.key);
    // Seed parity: odd = ground-floor shopfronts (larger, taller buildings only).
    let shop = height >= 8.5 && ring_area(&a.outer).abs() > 120.0 && (hk >> 20) % 100 < 45;
    let seed = (hk % 30) * 2 + shop as u64;
    let mut u = seed as f32 * 4096.0 + ((hk >> 12) % 97) as f32;
    let rings = std::iter::once((&a.outer, &a.border, false)).chain(a.holes.iter().zip(&a.hole_border).map(|(h, b)| (h, b, true)));
    for (pts, border, hole) in rings {
        let n = pts.len();
        let s = if (ring_area(pts) > 0.0) != hole { 1.0 } else { -1.0 };
        for i in 0..n {
            let (p, q) = (pts[i], pts[(i + 1) % n]);
            let e = q - p;
            let len = e.length();
            if border.get(i).copied().unwrap_or(false) {
                u += len; // artificial tile-seam edge: no wall
                continue;
            }
            let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * s;
            let i0 = m.v_uv(vec3(p.x, min_h, p.y), nrm, base_col, [u, min_h]);
            m.v_uv(vec3(q.x, min_h, q.y), nrm, base_col, [u + len, min_h]);
            m.v_uv(vec3(q.x, eaves, q.y), nrm, col, [u + len, eaves]);
            m.v_uv(vec3(p.x, eaves, p.y), nrm, col, [u, eaves]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
            u += len;
        }
    }
    // Floodlit crown on the tallest towers: a band of light under the roof
    // line, at night only (NIGHT_EMISSIVE: uv.x = luminance, cd/m²).
    if height > 140.0 && pitched.is_none() {
        const CROWN: [[f32; 3]; 4] = [[0.85, 0.92, 1.0], [0.45, 0.62, 1.0], [1.0, 0.86, 0.62], [0.72, 0.5, 1.0]];
        let c = CROWN[(hash(a.key ^ 0xc409) % 4) as usize];
        // (Vertex colours are sRGB-encoded; the shader linearises them.)
        let c = [c[0].powf(1.0 / 2.2), c[1].powf(1.0 / 2.2), c[2].powf(1.0 / 2.2)];
        let band = (height * 0.035).clamp(4.0, 9.0);
        m.mat = mat::NIGHT_EMISSIVE;
        let n = a.outer.len();
        let s = if ring_area(&a.outer) > 0.0 { 1.0 } else { -1.0 };
        for i in 0..n {
            if a.border.get(i).copied().unwrap_or(false) {
                continue;
            }
            let (p, q) = (a.outer[i], a.outer[(i + 1) % n]);
            let e = q - p;
            let nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero() * s;
            let o = nrm * 0.15;
            let lum = 70.0;
            let i0 = m.v_uv(vec3(p.x, height - band, p.y) + o, nrm, c, [lum * 0.35, 0.0]);
            m.v_uv(vec3(q.x, height - band, q.y) + o, nrm, c, [lum * 0.35, 0.0]);
            m.v_uv(vec3(q.x, height - 0.6, q.y) + o, nrm, c, [lum, 0.0]);
            m.v_uv(vec3(p.x, height - 0.6, p.y) + o, nrm, c, [lum, 0.0]);
            m.idx.extend([i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3]);
        }
    }
    match pitched {
        Some((q, _rise, hipped)) => {
            // Ridge along the long side, gables (or hips) on the short ends.
            const ROOFS: [[f32; 3]; 4] = [[0.36, 0.37, 0.40], [0.56, 0.33, 0.25], [0.45, 0.32, 0.27], [0.30, 0.30, 0.32]];
            let rc = ROOFS[(hash(a.key ^ 0xf00f) % 4) as usize];
            let along = (q[1] - q[0]).normalize();
            let short = (q[2] - q[1]).length();
            let inset = if hipped { (short * 0.5).min((q[1] - q[0]).length() * 0.45) } else { 0.0 };
            let r0 = (q[0] + q[3]) * 0.5 + along * inset;
            let r1 = (q[1] + q[2]) * 0.5 - along * inset;
            let p3 = |v: Vec2, y: f32| vec3(v.x, y, v.y);
            let (top, low) = (height, eaves);
            // Small overhang beyond the walls.
            let c = (q[0] + q[1] + q[2] + q[3]) * 0.25;
            let o = |v: Vec2| v + (v - c).normalize_or_zero() * 0.35;
            let (q0, q1, q2, q3) = (o(q[0]), o(q[1]), o(q[2]), o(q[3]));
            let low = low - 0.25;
            m.mat = mat::ROOF;
            let face = |m: &mut MeshData, pts: &[Vec3]| {
                let mut nrm = (pts[1] - pts[0]).cross(pts[2] - pts[0]).normalize_or_zero();
                if nrm.y < 0.0 {
                    nrm = -nrm;
                }
                let base = m.verts.len() as u32;
                for p in pts {
                    m.v(*p, nrm, rc);
                }
                m.idx.extend([base, base + 1, base + 2]);
                if pts.len() == 4 {
                    m.idx.extend([base, base + 2, base + 3]);
                }
            };
            face(m, &[p3(q0, low), p3(q1, low), p3(r1, top), p3(r0, top)]);
            face(m, &[p3(q2, low), p3(q3, low), p3(r0, top), p3(r1, top)]);
            if hipped {
                face(m, &[p3(q1, low), p3(q2, low), p3(r1, top)]);
                face(m, &[p3(q3, low), p3(q0, low), p3(r0, top)]);
            } else {
                // Gable walls.
                m.mat = mat::WALL;
                for (a2, b2, r) in [(q[1], q[2], r1), (q[3], q[0], r0)] {
                    let e = b2 - a2;
                    let mut nrm = vec3(e.y, 0.0, -e.x).normalize_or_zero();
                    if nrm.dot(vec3(a2.x - c.x, 0.0, a2.y - c.y)) < 0.0 {
                        nrm = -nrm;
                    }
                    let base = m.verts.len() as u32;
                    m.v_uv(p3(a2, eaves), nrm, col, [u, eaves]);
                    m.v_uv(p3(b2, eaves), nrm, col, [u + e.length(), eaves]);
                    m.v_uv(p3(r, top), nrm, col, [u + e.length() * 0.5, top]);
                    m.idx.extend([base, base + 1, base + 2]);
                }
            }
        }
        None => {
            m.mat = mat::ROOF;
            m.flat_poly(&a.outer, &a.holes, height, Vec3::Y, roof);
        }
    }
    if min_h > 0.5 {
        m.mat = mat::DIFFUSE;
        m.flat_poly(&a.outer, &a.holes, min_h, -Vec3::Y, base_col);
    }
}

// ---------------------------------------------------------------- runways

/// A runway in the current frame. `a` / `b` are the physical ends (start of
/// the pavement) of the low / high-numbered end. Fields after `name` come from
/// OurAirports (airports.rs); tile-merged runways leave them at their defaults.
#[derive(Clone, Debug, Default)]
pub struct Runway {
    pub a: Vec2,
    pub b: Vec2,
    pub width: f32,
    /// "09/27" (OurAirports) or the tile's `ref`.
    pub name: String,
    /// End identifiers ("09L", "27R"); empty for tile runways.
    pub ident_a: String,
    pub ident_b: String,
    /// Displaced threshold distance (m) from `a` / `b` along the runway.
    pub displaced_a: f32,
    pub displaced_b: f32,
    /// Hard surface (asphalt / concrete). Tile runways count as paved.
    pub paved: bool,
    /// Surface name ("Asphalt", "Grass", ...); empty when unknown.
    pub surface: String,
    /// Runway lighting reported.
    pub lighted: bool,
    /// Owning airport ident (OurAirports), empty for tile runways.
    pub airport: String,
    pub source: RunwaySource,
}

/// Where a runway's geometry came from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RunwaySource {
    /// Merged from map-tile centreline pieces (width estimated).
    #[default]
    Tiles,
    /// Surveyed OurAirports runway ends.
    OurAirports,
    /// OurAirports runway without surveyed ends: placed through the airport
    /// reference point along its heading.
    Estimated,
}

impl Runway {
    pub fn length(&self) -> f32 {
        self.a.distance(self.b)
    }
    /// Typical width for a runway of this length (z14 lines carry no width).
    pub fn width_for(len: f32) -> f32 {
        match len {
            l if l < 900.0 => 18.0,
            l if l < 1300.0 => 23.0,
            l if l < 2000.0 => 30.0,
            l if l < 2800.0 => 45.0,
            _ => 60.0,
        }
    }
}

/// Global runway mesh (asphalt + markings) for all merged runways.
pub fn runway_mesh(runways: &[Runway]) -> MeshData {
    let mut m = MeshData::default();
    m.mat = mat::ASPHALT;
    for rw in runways {
        runway_geometry(&mut m, rw);
    }
    m
}

fn runway_geometry(m: &mut MeshData, rw: &Runway) {
    let d = (rw.b - rw.a).normalize();
    let n = vec2(-d.y, d.x);
    let len = rw.length();
    let w = rw.width;
    {
        // Asphalt with (along, across) uvs for wear / rubber deposits.
        m.mat = mat::RUNWAY;
        let h = n * (w * 0.5);
        let p = |q: Vec2| vec3(q.x, 0.0, q.y);
        // Two halves so uv.x is the distance from the nearer threshold.
        let mid = (rw.a + rw.b) * 0.5;
        for (s, e) in [(rw.a, mid), (rw.b, mid)] {
            let base = m.verts.len() as u32;
            m.v_uv(p(s + h), Vec3::Y, RUNWAY, [0.0, w * 0.5]);
            m.v_uv(p(e + h), Vec3::Y, RUNWAY, [len * 0.5, w * 0.5]);
            m.v_uv(p(e - h), Vec3::Y, RUNWAY, [len * 0.5, -w * 0.5]);
            m.v_uv(p(s - h), Vec3::Y, RUNWAY, [0.0, -w * 0.5]);
            m.idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        }
        m.mat = mat::ASPHALT;
    }
    for s in [-1.0, 1.0] {
        let off = n * s * (w * 0.5 - 1.0);
        m.ground_rect(rw.a + off, rw.b + off, 0.9, 0.0, PAINT);
    }
    let mut t = 80.0;
    while t + 30.0 < len - 80.0 {
        m.ground_rect(rw.a + d * t, rw.a + d * (t + 30.0), 0.9, 0.0, PAINT);
        t += 50.0;
    }
    for (start, dir) in [(rw.a, d), (rw.b, -d)] {
        let stripes = ((w / 3.6) as i32 / 2 * 2).clamp(4, 16);
        let usable = w - 6.0;
        let pitch = usable / stripes as f32;
        for i in 0..stripes {
            let lat = -usable * 0.5 + pitch * (i as f32 + 0.5);
            if lat.abs() < pitch * 0.6 {
                continue;
            }
            let c = start + n * lat;
            m.ground_rect(c + dir * 6.0, c + dir * 36.0, pitch * 0.55, 0.0, PAINT);
        }
        if len > 900.0 {
            for s in [-1.0, 1.0] {
                let c = start + n * s * w * 0.22;
                m.ground_rect(c + dir * 300.0, c + dir * 345.0, (w * 0.13).max(4.0), 0.0, PAINT);
            }
        }
    }
}
