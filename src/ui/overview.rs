//! Overview world map for the airport picker: OpenFreeMap vector tiles at
//! z0–z8 (fetched + cached like the scenery tiles), decoded on background
//! threads into triangulated water / land-cover fills, borders, major roads
//! and place labels, drawn in Web Mercator through an MSAA canvas.

use super::canvas;
use super::{f_semi, f_ui, pal};
use crate::mvt::{self, GeomType};
use crate::osm::TileId;
use egui::{epaint, Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2};
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};

pub const MAX_Z: u32 = 8;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Fill {
    Wood,
    Grass,
    Ice,
    Sand,
    Urban,
    Water,
}

const FILLS: [Fill; 6] = [Fill::Grass, Fill::Wood, Fill::Sand, Fill::Ice, Fill::Urban, Fill::Water];

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Line {
    River,
    Road,
    State,
    Country,
}

const LINES: [Line; 4] = [Line::River, Line::Road, Line::State, Line::Country];

struct Label {
    /// Tile units (0..1).
    pos: [f32; 2],
    text: String,
    /// 0 country, 1 state, 2 city, 3 town, 4 sea
    kind: u8,
    rank: u8,
}

#[derive(Default)]
struct OvTile {
    fills: Vec<(Fill, Vec<[f32; 2]>, Vec<u32>)>,
    lines: Vec<(Line, Vec<Vec<[f32; 2]>>)>,
    labels: Vec<Label>,
}

/// Mercator x / y in 0..1 for a lat / lon.
pub fn merc(lat: f64, lon: f64) -> (f64, f64) {
    let lat = lat.clamp(-85.05, 85.05).to_radians();
    ((lon + 180.0) / 360.0, (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0)
}

pub fn unmerc(x: f64, y: f64) -> (f64, f64) {
    let lon = x * 360.0 - 180.0;
    let n = PI - 2.0 * PI * y;
    (n.sinh().atan().to_degrees(), lon)
}

/// Map view: centre in Mercator units, zoom (world width = 256·2^zoom points).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    pub cx: f64,
    pub cy: f64,
    pub zoom: f64,
    pub rect: Rect,
}

impl View {
    pub fn scale(&self) -> f64 {
        256.0 * 2f64.powf(self.zoom)
    }
    pub fn to_screen(&self, x: f64, y: f64) -> Pos2 {
        let s = self.scale();
        let mut dx = x - self.cx;
        dx -= dx.round(); // wrap around the antimeridian
        Pos2::new(self.rect.center().x + (dx * s) as f32, self.rect.center().y + ((y - self.cy) * s) as f32)
    }
    pub fn ll_to_screen(&self, lat: f64, lon: f64) -> Pos2 {
        let (x, y) = merc(lat, lon);
        self.to_screen(x, y)
    }
    pub fn to_merc(&self, p: Pos2) -> (f64, f64) {
        let s = self.scale();
        (self.cx + (p.x - self.rect.center().x) as f64 / s, self.cy + (p.y - self.rect.center().y) as f64 / s)
    }
    pub fn to_ll(&self, p: Pos2) -> (f64, f64) {
        let (x, y) = self.to_merc(p);
        unmerc(x.rem_euclid(1.0), y.clamp(0.0, 1.0))
    }
    /// Zoom by `f` keeping the point under `at` fixed.
    pub fn zoom_at(&mut self, at: Pos2, dz: f64) {
        let before = self.to_merc(at);
        self.zoom = (self.zoom + dz).clamp(1.2, 15.0);
        let after = self.to_merc(at);
        self.cx += before.0 - after.0;
        self.cy = (self.cy + before.1 - after.1).clamp(0.0, 1.0);
    }
    /// Lat / lon bounds of the view.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let (lat0, lon0) = self.to_ll(self.rect.left_bottom());
        let (lat1, lon1) = self.to_ll(self.rect.right_top());
        let w = self.rect.width() as f64 / self.scale();
        if w >= 1.0 { (lat0, lat1, -180.0, 180.0) } else { (lat0, lat1, lon0, lon1) }
    }
}

struct Queue {
    jobs: Mutex<Vec<TileId>>,
    cv: Condvar,
}

pub struct Overview {
    tiles: HashMap<TileId, Arc<OvTile>>,
    pending: HashSet<TileId>,
    failed: HashSet<TileId>,
    q: Arc<Queue>,
    rx: Receiver<(TileId, Option<OvTile>)>,
    _tx: Sender<(TileId, Option<OvTile>)>,
    line_cache: HashMap<(TileId, u8, i32, bool), Arc<epaint::Mesh>>,
}

impl Default for Overview {
    fn default() -> Self {
        Self::new()
    }
}

/// Simplification tolerance (tile units): ≈ ½ px when the tile is shown 512 px wide.
const TOL: f32 = 1.0 / 1000.0;

fn simp_line(p: &[[f32; 2]], e: f32) -> Vec<[f32; 2]> {
    let v: Vec<glam::Vec2> = p.iter().map(|q| glam::vec2(q[0] / e, q[1] / e)).collect();
    super::mapdata::simplify(&v, TOL).iter().map(|q| [q.x, q.y]).collect()
}

/// Simplified ring in tile units, or None when it is too small to see.
fn simp_ring(r: &[[f32; 2]], e: f32) -> Option<Vec<[f32; 2]>> {
    if (mvt::ring_area(r).abs() / (e * e)) < TOL * TOL * 4.0 {
        return None;
    }
    let mut v = simp_line(r, e);
    if v.len() > 2 && v[0] == v[v.len() - 1] {
        v.pop();
    }
    (v.len() >= 3).then_some(v)
}

fn decode(id: TileId, bytes: &[u8]) -> OvTile {
    let mut t = OvTile::default();
    let Ok(layers) = mvt::decode(bytes) else { return t };
    let mut fills: HashMap<Fill, (Vec<[f32; 2]>, Vec<u32>)> = HashMap::new();
    let mut lines: HashMap<Line, Vec<Vec<[f32; 2]>>> = HashMap::new();
    for layer in &layers {
        let e = layer.extent as f32;
        let n = |p: &[f32; 2]| [p[0] / e, p[1] / e];
        for f in &layer.features {
            let class = f.str("class").unwrap_or("");
            match (layer.name.as_str(), f.geom_type) {
                (name @ ("water" | "landcover" | "landuse"), GeomType::Polygon) => {
                    let fc = match (name, class) {
                        ("water", _) => Fill::Water,
                        ("landcover", "wood" | "forest") => Fill::Wood,
                        ("landcover", "grass" | "wetland" | "farmland") => Fill::Grass,
                        ("landcover", "ice" | "glacier") => Fill::Ice,
                        ("landcover", "sand" | "rock") => Fill::Sand,
                        ("landuse", "residential" | "commercial" | "industrial" | "suburb" | "city") => Fill::Urban,
                        _ => continue,
                    };
                    let slot = fills.entry(fc).or_default();
                    for (outer, holes) in mvt::polygons(&f.parts) {
                        let mut data: Vec<f64> = Vec::new();
                        let mut hi = Vec::new();
                        let Some(mut pts) = simp_ring(&outer, e) else { continue };
                        for h in &holes {
                            if let Some(hp) = simp_ring(h, e) {
                                hi.push(pts.len());
                                pts.extend(hp);
                            }
                        }
                        for p in &pts {
                            data.extend([p[0] as f64, p[1] as f64]);
                        }
                        if let Ok(tr) = earcutr::earcut(&data, &hi, 2) {
                            let base = slot.0.len() as u32;
                            slot.0.extend(pts);
                            slot.1.extend(tr.into_iter().map(|i| base + i as u32));
                        }
                    }
                }
                ("boundary", GeomType::Line) => {
                    if f.num("maritime").unwrap_or(0.0) > 0.5 {
                        continue;
                    }
                    let lc = match f.num("admin_level").unwrap_or(10.0) as i32 {
                        2 => Line::Country,
                        3 | 4 => Line::State,
                        _ => continue,
                    };
                    lines.entry(lc).or_default().extend(f.parts.iter().map(|p| simp_line(p, e)));
                }
                ("transportation", GeomType::Line) if matches!(class, "motorway" | "trunk") => {
                    lines.entry(Line::Road).or_default().extend(f.parts.iter().map(|p| simp_line(p, e)));
                }
                ("waterway", GeomType::Line) if class == "river" => {
                    lines.entry(Line::River).or_default().extend(f.parts.iter().map(|p| simp_line(p, e)));
                }
                ("place" | "water_name", GeomType::Point) => {
                    let kind = match (layer.name.as_str(), class) {
                        ("place", "country") => 0,
                        ("place", "state") => 1,
                        ("place", "city") => 2,
                        ("place", "town") => 3,
                        ("water_name", "ocean" | "sea") => 4,
                        _ => continue,
                    };
                    let name = f.str("name:latin").or(f.str("name_en")).or(f.str("name")).unwrap_or("");
                    if name.is_empty() {
                        continue;
                    }
                    for part in &f.parts {
                        for p in part {
                            if p[0] >= 0.0 && p[0] < e && p[1] >= 0.0 && p[1] < e {
                                t.labels.push(Label { pos: n(p), text: name.to_string(), kind, rank: f.num("rank").unwrap_or(5.0) as u8 });
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }
    let _ = id;
    t.fills = FILLS.iter().filter_map(|f| fills.remove(f).map(|(v, i)| (*f, v, i))).collect();
    t.lines = LINES.iter().filter_map(|l| lines.remove(l).map(|v| (*l, v))).collect();
    t
}

impl Overview {
    pub fn new() -> Overview {
        let q = Arc::new(Queue { jobs: Mutex::new(Vec::new()), cv: Condvar::new() });
        let (tx, rx) = channel();
        // Browser: one async fetch per request instead (see `request`).
        #[cfg(not(target_arch = "wasm32"))]
        for i in 0..3 {
            let q = q.clone();
            let tx = tx.clone();
            std::thread::Builder::new()
                .name(format!("overview-{i}"))
                .spawn(move || loop {
                    let id = {
                        let mut j = q.jobs.lock().unwrap();
                        loop {
                            if let Some(id) = j.pop() {
                                break id;
                            }
                            j = q.cv.wait(j).unwrap();
                        }
                    };
                    let t = crate::tiles::fetch_any(id).ok().map(|b| decode(id, &b));
                    if tx.send((id, t)).is_err() {
                        return;
                    }
                })
                .ok();
        }
        Overview { tiles: HashMap::new(), pending: HashSet::new(), failed: HashSet::new(), q, rx, _tx: tx, line_cache: HashMap::new() }
    }

    pub fn poll(&mut self) {
        while let Ok((id, t)) = self.rx.try_recv() {
            self.pending.remove(&id);
            match t {
                Some(t) => {
                    self.tiles.insert(id, Arc::new(t));
                }
                None => {
                    self.failed.insert(id);
                }
            }
        }
    }

    /// Tiles still loading.
    pub fn busy(&self) -> usize {
        self.pending.len()
    }

    fn request(&mut self, id: TileId) {
        if self.tiles.contains_key(&id) || self.pending.contains(&id) || self.failed.contains(&id) {
            return;
        }
        self.pending.insert(id);
        #[cfg(target_arch = "wasm32")]
        {
            let tx = self._tx.clone();
            crate::platform::web::spawn_local(async move {
                let t = crate::tiles::fetch_any_async(id).await.ok().map(|b| decode(id, &b));
                let _ = tx.send((id, t));
            });
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.q.jobs.lock().unwrap().push(id);
            self.q.cv.notify_one();
        }
    }

    /// Tiles covering the view at the right zoom, requesting missing ones;
    /// returns (tile, its origin x/y, size) to draw, coarse fallbacks first.
    fn visible(&mut self, v: &View) -> Vec<(TileId, f64, f64, f64)> {
        let z = (v.zoom + 0.35).floor().clamp(0.0, MAX_Z as f64) as u32;
        let n = 1u32 << z;
        let (x0, y0) = v.to_merc(v.rect.min);
        let (x1, y1) = v.to_merc(v.rect.max);
        let (tx0, tx1) = ((x0 * n as f64).floor() as i64, (x1 * n as f64).floor() as i64);
        let (ty0, ty1) = ((y0.max(0.0) * n as f64).floor() as i64, (y1.min(0.9999) * n as f64).floor() as i64);
        // Newest requests are served first: queue the centre last.
        let mut want = Vec::new();
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1.min(tx0 + n as i64 - 1) {
                let x = tx.rem_euclid(n as i64) as u32;
                want.push((TileId { z, x, y: ty as u32 }, tx));
            }
        }
        let c = ((v.cx * n as f64), (v.cy * n as f64));
        want.sort_by(|a, b| {
            let da = (a.1 as f64 + 0.5 - c.0).powi(2) + (a.0.y as f64 + 0.5 - c.1).powi(2);
            let db = (b.1 as f64 + 0.5 - c.0).powi(2) + (b.0.y as f64 + 0.5 - c.1).powi(2);
            db.total_cmp(&da)
        });
        let mut out = Vec::new();
        let mut parents = HashSet::new();
        for (id, tx) in &want {
            self.request(*id);
            if !self.tiles.contains_key(id) {
                // fall back to the nearest loaded ancestor
                let mut pz = id.z;
                let (mut px, mut py) = (id.x, id.y);
                let mut ptx = *tx;
                while pz > 0 {
                    pz -= 1;
                    px /= 2;
                    py /= 2;
                    ptx = ptx.div_euclid(2);
                    let pid = TileId { z: pz, x: px, y: py };
                    if self.tiles.contains_key(&pid) {
                        if parents.insert((pid, ptx)) {
                            let s = 1.0 / (1u32 << pz) as f64;
                            out.insert(0, (pid, ptx as f64 * s, py as f64 * s, s));
                        }
                        break;
                    }
                }
            }
        }
        for (id, tx) in want {
            if self.tiles.contains_key(&id) {
                let s = 1.0 / n as f64;
                out.push((id, tx as f64 * s, id.y as f64 * s, s));
            }
        }
        // the whole world as a backdrop at z0
        self.request(TileId { z: 0, x: 0, y: 0 });
        out
    }

    /// Draw the map; returns label rects (for collision with the caller's symbols).
    pub fn draw(&mut self, p: &egui::Painter, cv: &mut canvas::Frame, key: &'static str, ppp: f32, v: &View, dark: bool) -> Vec<Rect> {
        self.poll();
        let vis = self.visible(v);
        let (land, cols, line_cols) = colours(dark);
        let off = v.rect.min.to_vec2();
        let mut shapes = Vec::new();
        for (fi, fc) in FILLS.iter().enumerate() {
            let col = cols[fi];
            let mut m = epaint::Mesh::default();
            for (id, ox, oy, s) in &vis {
                let Some(t) = self.tiles.get(id) else { continue };
                let o = v.to_screen(*ox, *oy) - off;
                let k = (*s * v.scale()) as f32;
                for (f, verts, idx) in &t.fills {
                    if f != fc {
                        continue;
                    }
                    let b = m.vertices.len() as u32;
                    m.vertices.extend(verts.iter().map(|q| epaint::Vertex { pos: o + Vec2::new(q[0], q[1]) * k, uv: epaint::WHITE_UV, color: col }));
                    m.indices.extend(idx.iter().map(|i| b + i));
                }
            }
            if !m.indices.is_empty() {
                shapes.push(Shape::mesh(m));
            }
        }
        for (li, lc) in LINES.iter().enumerate() {
            let (col, w) = line_cols[li];
            let mut m = epaint::Mesh::default();
            for (id, ox, oy, s) in &vis {
                let Some(t) = self.tiles.get(id) else { continue };
                let o = v.to_screen(*ox, *oy) - off;
                let k = (*s * v.scale()) as f32;
                // Tessellated once per tile, class and zoom bucket (in tile units).
                let bucket = (k.log2() * 4.0).round() as i32;
                let k0 = 2f32.powf(bucket as f32 / 4.0);
                let key = (*id, li as u8, bucket, dark);
                let tm = self.line_cache.entry(key).or_insert_with(|| {
                    let mut tess = epaint::Tessellator::new(k0, epaint::TessellationOptions { feathering: false, round_line_segments_to_pixels: false, coarse_tessellation_culling: false, ..Default::default() }, [1, 1], Vec::new());
                    let mut mesh = epaint::Mesh::default();
                    for (l, ls) in &t.lines {
                        if l != lc {
                            continue;
                        }
                        for pts in ls {
                            tess.tessellate_shape(Shape::line(pts.iter().map(|q| Pos2::new(q[0], q[1])).collect(), Stroke::new(w / k0, col)), &mut mesh);
                        }
                    }
                    Arc::new(mesh)
                });
                let b = m.vertices.len() as u32;
                m.vertices.extend(tm.vertices.iter().map(|vx| epaint::Vertex { pos: o + vx.pos.to_vec2() * k, uv: epaint::WHITE_UV, color: vx.color }));
                m.indices.extend(tm.indices.iter().map(|i| b + i));
            }
            if !m.indices.is_empty() {
                shapes.push(Shape::mesh(m));
            }
        }
        if self.line_cache.len() > 5000 {
            self.line_cache.clear();
        }
        let id = cv.draw(key, v.rect.size(), ppp, land, shapes);
        match id {
            Some(id) => canvas::image(p, id, v.rect, Color32::WHITE),
            None => {
                p.rect_filled(v.rect, 0, land);
            }
        }
        // Labels (countries big, then states, cities).
        let mut taken: Vec<Rect> = Vec::new();
        let mut cands: Vec<(f32, Pos2, String, egui::FontId, Color32)> = Vec::new();
        let mut seen = HashSet::new();
        for (id, ox, oy, s) in &vis {
            let Some(t) = self.tiles.get(id) else { continue };
            let o = v.to_screen(*ox, *oy);
            let k = (*s * v.scale()) as f32;
            for l in &t.labels {
                let show = match l.kind {
                    0 => v.zoom < 7.0,
                    1 => v.zoom > 3.5 && v.zoom < 9.0,
                    2 => v.zoom > 3.2 + l.rank as f64 * 0.35,
                    3 => v.zoom > 6.5,
                    _ => v.zoom < 5.0,
                };
                if !show || !seen.insert(l.text.clone()) {
                    continue;
                }
                let q = o + Vec2::new(l.pos[0], l.pos[1]) * k;
                if !v.rect.contains(q) {
                    continue;
                }
                let (font, col, pri) = match l.kind {
                    0 => (f_semi(13.0), if dark { Color32::from_gray(170) } else { Color32::from_rgb(90, 84, 100) }, 0.0 + l.rank as f32),
                    1 => (f_ui(11.0), if dark { Color32::from_gray(120) } else { Color32::from_rgb(130, 120, 130) }, 20.0),
                    2 => (f_ui(12.0), if dark { Color32::from_gray(205) } else { Color32::from_rgb(40, 44, 52) }, 10.0 + l.rank as f32),
                    3 => (f_ui(11.0), if dark { Color32::from_gray(175) } else { Color32::from_rgb(60, 64, 72) }, 25.0 + l.rank as f32),
                    _ => (f_ui(12.0), if dark { Color32::from_rgb(90, 140, 190) } else { Color32::from_rgb(70, 120, 170) }, 5.0),
                };
                let text = if l.kind == 0 { l.text.to_uppercase() } else { l.text.clone() };
                cands.push((pri, q, text, font, col));
            }
        }
        cands.sort_by(|a, b| a.0.total_cmp(&b.0));
        let halo = if dark { Color32::from_black_alpha(170) } else { Color32::from_white_alpha(190) };
        for (_, q, text, font, col) in cands.into_iter().take(160) {
            let g = p.layout_no_wrap(text.clone(), font.clone(), col);
            let r = Rect::from_center_size(q, g.size() + Vec2::splat(6.0));
            if taken.iter().any(|t| t.intersects(r)) {
                continue;
            }
            taken.push(r);
            for o in [Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(0.0, -1.0)] {
                p.text(q + o, Align2::CENTER_CENTER, &text, font.clone(), halo);
            }
            p.text(q, Align2::CENTER_CENTER, text, font, col);
        }
        let _ = pal::TEXT;
        taken
    }
}

fn colours(dark: bool) -> (Color32, [Color32; 6], [(Color32, f32); 4]) {
    let c = |h: u32| Color32::from_rgb((h >> 16) as u8, (h >> 8) as u8, h as u8);
    if dark {
        (
            c(0x1A2028),
            [c(0x1C2721), c(0x18261E), c(0x2A2822), c(0x2C3640), c(0x272C35), c(0x0D1B2A)],
            [(c(0x16324D), 1.0), (c(0x5A4A30), 1.2), (c(0x3A4250), 0.8), (c(0x6C7686), 1.3)],
        )
    } else {
        (
            c(0xEDEBE3),
            [c(0xD9E8C6), c(0xBFDAA8), c(0xF1E7C8), c(0xF8FBFC), c(0xE2DDD8), c(0xA9CFEC)],
            [(c(0x9CC6E8), 1.0), (c(0xE8B26A), 1.3), (c(0xB9AEC0), 0.8), (c(0x8F8299), 1.4)],
        )
    }
}
