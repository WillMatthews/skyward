//! Vector moving map: the tiles' map vectors (mapdata.rs) drawn offscreen
//! with MSAA (canvas.rs) in a day, night or navigation-display style, with
//! crisp overlays on top — runways, airport symbols, collision-free labels,
//! range rings, the aircraft with its trend vector, wind and the auto-flight
//! route.

use super::canvas;
use super::mapdata::{Fill, LabelKind, Line, MapVectors, FILLS, LINES};
use super::{alpha, f_inst, f_inst_b, f_semi, f_ui, pal, polar, text_halo};
use crate::osm::{Proj, TileId};
use crate::world::World;
use egui::{epaint, Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use glam::{vec2, Vec2};
use std::collections::HashMap;
use std::sync::Arc;

pub type Tiles = HashMap<TileId, Arc<MapVectors>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Style {
    Day,
    Night,
    /// Navigation display: dark, desaturated, low contrast under the symbology.
    Nd,
}

struct Pal {
    bg: Color32,
    fills: [Color32; 13],
    /// (colour, casing, width m, min width pt)
    lines: [(Color32, Color32, f32, f32); 9],
    label: Color32,
    label_halo: Color32,
    water_label: Color32,
    runway: Color32,
    runway_edge: Color32,
}

fn rgb(h: u32) -> Color32 {
    Color32::from_rgb((h >> 16) as u8, (h >> 8) as u8, h as u8)
}

fn palette(s: Style) -> Pal {
    match s {
        Style::Day => Pal {
            bg: rgb(0xEDEFE6),
            fills: [
                rgb(0xE9ECDD), // farm
                rgb(0xE5E1DC), // residential
                rgb(0xEADDDA), // commercial
                rgb(0xE0DCE4), // industrial
                rgb(0xCBE5BA), // park
                rgb(0xD5E3C0), // scrub
                rgb(0xADD19B), // wood
                rgb(0xCDE2D6), // wetland
                rgb(0xF0E6C6), // sand
                rgb(0xE1E2E9), // aerodrome
                rgb(0xCACCD6), // apron
                rgb(0x9FC8EA), // water
                rgb(0xD6CFC9), // building
            ],
            lines: [
                (rgb(0x9FC8EA), Color32::TRANSPARENT, 3.0, 0.7),   // stream
                (rgb(0x9FC8EA), Color32::TRANSPARENT, 14.0, 1.4),  // river
                (rgb(0xC8B795), Color32::TRANSPARENT, 3.0, 0.6),   // track
                (rgb(0xFFFFFF), rgb(0xD9D4CC), 7.0, 0.8),          // minor
                (rgb(0xFFFFFF), rgb(0xCFC8BE), 11.0, 1.3),         // secondary
                (rgb(0x8D94A0), Color32::TRANSPARENT, 3.0, 1.0),   // rail
                (rgb(0xB7B9C4), Color32::TRANSPARENT, 18.0, 1.0),  // taxiway
                (rgb(0xFBD68C), rgb(0xD9A85A), 15.0, 1.8),         // primary
                (rgb(0xF5AE63), rgb(0xC9833A), 22.0, 2.4),         // motorway
            ],
            label: rgb(0x2B3038),
            label_halo: Color32::from_rgba_premultiplied(240, 242, 236, 230),
            water_label: rgb(0x3D6E9C),
            runway: rgb(0x4E5361),
            runway_edge: rgb(0x2E323C),
        },
        Style::Night => Pal {
            bg: rgb(0x161B22),
            fills: [
                rgb(0x181E25),
                rgb(0x222731),
                rgb(0x272A35),
                rgb(0x252834),
                rgb(0x1A2A22),
                rgb(0x1C2A23),
                rgb(0x172A20),
                rgb(0x192A29),
                rgb(0x2A2922),
                rgb(0x232731),
                rgb(0x313646),
                rgb(0x0F2A43),
                rgb(0x323846),
            ],
            lines: [
                (rgb(0x173A5A), Color32::TRANSPARENT, 3.0, 0.7),
                (rgb(0x173A5A), Color32::TRANSPARENT, 14.0, 1.4),
                (rgb(0x3A3830), Color32::TRANSPARENT, 3.0, 0.6),
                (rgb(0x3E4552), Color32::TRANSPARENT, 7.0, 0.8),
                (rgb(0x4E5665), Color32::TRANSPARENT, 11.0, 1.3),
                (rgb(0x4A505C), Color32::TRANSPARENT, 3.0, 1.0),
                (rgb(0x4A4F60), Color32::TRANSPARENT, 18.0, 1.0),
                (rgb(0x8C7444), rgb(0x3B3324), 15.0, 1.8),
                (rgb(0xA87A3E), rgb(0x45331F), 22.0, 2.4),
            ],
            label: rgb(0xD8DEE6),
            label_halo: Color32::from_rgba_premultiplied(14, 17, 22, 220),
            water_label: rgb(0x6FA6D8),
            runway: rgb(0x8C95A8),
            runway_edge: rgb(0x5B6273),
        },
        Style::Nd => Pal {
            bg: rgb(0x080B10),
            fills: [
                rgb(0x0A0E13),
                rgb(0x10141B),
                rgb(0x12161D),
                rgb(0x12151C),
                rgb(0x0C1712),
                rgb(0x0D1713),
                rgb(0x0B1711),
                rgb(0x0C1616),
                rgb(0x16150F),
                rgb(0x131720),
                rgb(0x1A1F2A),
                rgb(0x0A2036),
                rgb(0x181C24),
            ],
            lines: [
                (rgb(0x0E2B45), Color32::TRANSPARENT, 3.0, 0.6),
                (rgb(0x0E2B45), Color32::TRANSPARENT, 14.0, 1.2),
                (rgb(0x1C1E20), Color32::TRANSPARENT, 3.0, 0.5),
                (rgb(0x1E232B), Color32::TRANSPARENT, 7.0, 0.6),
                (rgb(0x262C36), Color32::TRANSPARENT, 11.0, 0.9),
                (rgb(0x2A2E36), Color32::TRANSPARENT, 3.0, 0.8),
                (rgb(0x2A2E38), Color32::TRANSPARENT, 18.0, 0.8),
                (rgb(0x3A3426), Color32::TRANSPARENT, 15.0, 1.2),
                (rgb(0x4A3D27), Color32::TRANSPARENT, 22.0, 1.6),
            ],
            label: rgb(0x8E98A8),
            label_halo: Color32::from_rgba_premultiplied(5, 7, 10, 220),
            water_label: rgb(0x4F7DA8),
            runway: rgb(0xD8DCE4),
            runway_edge: rgb(0xD8DCE4),
        },
    }
}

/// World ↔ screen mapping of one map view.
#[derive(Clone, Copy, Debug)]
pub struct View {
    /// Screen rect (points).
    pub rect: Rect,
    /// Where `centre` sits on screen.
    pub anchor: Pos2,
    /// World point (current frame, m) at `anchor`.
    pub centre: Vec2,
    /// Points per metre.
    pub s: f32,
    /// Direction (radians clockwise from north) that points up on screen.
    pub up: f32,
}

impl View {
    pub fn to_screen(&self, w: Vec2) -> Pos2 {
        let d = w - self.centre;
        let (sn, cs) = self.up.sin_cos();
        let (rx, ry) = (cs, sn); // right
        let (fx, fy) = (sn, -cs); // forward
        Pos2::new(self.anchor.x + (d.x * rx + d.y * ry) * self.s, self.anchor.y - (d.x * fx + d.y * fy) * self.s)
    }
    pub fn to_world(&self, p: Pos2) -> Vec2 {
        let (sn, cs) = self.up.sin_cos();
        let a = (p.x - self.anchor.x) / self.s;
        let b = -(p.y - self.anchor.y) / self.s;
        self.centre + vec2(cs, sn) * a + vec2(sn, -cs) * b
    }
    /// Screen direction of a world bearing (radians clockwise from north).
    pub fn screen_angle(&self, bearing: f32) -> f32 {
        bearing - self.up
    }
    /// World radius that covers the whole rect from the anchor.
    pub fn reach(&self) -> f32 {
        let r = self.rect;
        [r.left_top(), r.right_top(), r.left_bottom(), r.right_bottom()].iter().map(|c| c.distance(self.anchor)).fold(0.0, f32::max) / self.s
    }
}

type LineKey = (TileId, u8, u8, u8, i32);

thread_local! {
    /// Tessellated tile lines (tile metres) per zoom bucket and style.
    static LINE_CACHE: std::cell::RefCell<HashMap<LineKey, (Arc<epaint::Mesh>, u64)>> = std::cell::RefCell::new(HashMap::new());
}
static FRAME: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// The base layers (fills, lines, runways) as shapes in canvas-local
/// points (origin = view.rect.min), for an MSAA canvas.
pub fn base_shapes(view: &View, world: &World, tiles: &Tiles, style: Style) -> Vec<Shape> {
    let pal = palette(style);
    let mut out = Vec::new();
    let off = view.rect.min.to_vec2();
    let local = View { anchor: view.anchor - off, rect: view.rect.translate(-off), ..*view };
    let reach = view.reach();
    let s = view.s;
    // Which tiles are visible, far ones first.
    let mut vis: Vec<(TileId, &Arc<MapVectors>, crate::osm::Xform2)> = Vec::new();
    for (id, mv) in tiles {
        let (lo, hi) = id.rect(&world.proj);
        let c = (lo + hi) * 0.5;
        if c.distance(view.centre) - (hi - lo).length() * 0.5 > reach {
            continue;
        }
        // Detailed tiles only when they're not too small to matter
        // (the generalised far tiles carry the overview).
        if !id.is_far() && s < if style == Style::Nd { 0.02 } else { 0.012 } {
            continue;
        }
        vis.push((*id, mv, Proj::tile_frame(*id).map_to(&world.proj)));
    }
    vis.sort_by_key(|(id, ..)| (!id.is_far(), id.z, id.y, id.x));
    let near_start = vis.iter().position(|v| !v.0.is_far()).unwrap_or(vis.len());
    // Per-tile affine: tile frame → canvas points.
    let affine = |xf: &crate::osm::Xform2| {
        let o = local.to_screen(xf.apply(Vec2::ZERO));
        let ex = local.to_screen(xf.apply(vec2(1000.0, 0.0))) - o;
        let ez = local.to_screen(xf.apply(vec2(0.0, 1000.0))) - o;
        (o, ex / 1000.0, ez / 1000.0)
    };
    let frame = FRAME.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    LINE_CACHE.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() > 6000 {
            c.retain(|_, v| frame.saturating_sub(v.1) < 600);
        }
    });
    let show_buildings = s > 0.22;
    // Coarse geometry when zoomed out (detailed tiles below 12 m/pt, far tiles below 40 m/pt).
    let lo_near = s < 0.08;
    let lo_far = s < 0.025;
    let show_minor = s > 0.06;
    let show_track = s > 0.14;
    let draw_tiles = |out: &mut Vec<Shape>, range: std::ops::Range<usize>, backing: bool| {
        if backing {
            // Opaque land under the detailed tiles hides the generalised layer.
            let mut m = epaint::Mesh::default();
            for (id, _, _) in &vis[range.clone()] {
                let (lo, hi) = id.rect(&world.proj);
                let q = [lo, vec2(hi.x, lo.y), hi, vec2(lo.x, hi.y)].map(|w| local.to_screen(w));
                let b = m.vertices.len() as u32;
                for p in q {
                    m.colored_vertex(p, pal.bg);
                }
                m.add_triangle(b, b + 1, b + 2);
                m.add_triangle(b, b + 2, b + 3);
            }
            out.push(Shape::mesh(m));
        }
        for (fi, fc) in FILLS.iter().enumerate() {
            if *fc == Fill::Building && !show_buildings {
                continue;
            }
            let col = pal.fills[fi];
            let mut m = epaint::Mesh::default();
            for (_, mv, xf) in &vis[range.clone()] {
                let lo = if mv.far { lo_far } else { lo_near };
                let Some(f) = mv.fill(*fc, lo) else { continue };
                let (o, ex, ez) = affine(xf);
                let b = m.vertices.len() as u32;
                m.vertices.extend(f.verts.iter().map(|v| epaint::Vertex { pos: o + ex * v[0] + ez * v[1], uv: epaint::WHITE_UV, color: col }));
                m.indices.extend(f.idx.iter().map(|i| b + i));
            }
            if !m.indices.is_empty() {
                out.push(Shape::mesh(m));
            }
        }
        // Lines: casings first, then fills. Each tile's lines are tessellated
        // once per zoom bucket (in tile metres) and only transformed per frame.
        let bucket = (s.log2() * 4.0).round() as i32;
        let s0 = 2f32.powf(bucket as f32 / 4.0);
        for pass in 0..2u8 {
            for (li, lc) in LINES.iter().enumerate() {
                let (col, casing, width_m, min_pt) = pal.lines[li];
                if (*lc == Line::Minor && !show_minor) || (*lc == Line::Track && !show_track) || (*lc == Line::Stream && !show_minor) {
                    continue;
                }
                if style == Style::Nd && (matches!(lc, Line::Minor | Line::Track | Line::Stream) || (*lc == Line::Secondary && s < 0.02)) {
                    continue;
                }
                let w = (width_m * s0).max(min_pt);
                let (c, ww) = if pass == 0 {
                    if casing == Color32::TRANSPARENT || w < 1.6 {
                        continue;
                    }
                    (casing, w + 1.4)
                } else {
                    (col, w)
                };
                let mut m = epaint::Mesh::default();
                for (id, mv, xf) in &vis[range.clone()] {
                    let lo = if mv.far { lo_far } else { lo_near };
                    let Some(ls) = mv.line(*lc, lo) else { continue };
                    let key = (*id, li as u8 + if lo { 100 } else { 0 }, pass, style as u8, bucket);
                    let tm = LINE_CACHE.with(|cache| {
                        let mut cache = cache.borrow_mut();
                        let e = cache.entry(key).or_insert_with(|| {
                            let mut tess = epaint::Tessellator::new(s0, epaint::TessellationOptions { feathering: false, round_line_segments_to_pixels: false, coarse_tessellation_culling: false, ..Default::default() }, [1, 1], Vec::new());
                            let mut mesh = epaint::Mesh::default();
                            for l in ls {
                                let pts: Vec<Pos2> = l.iter().map(|v| Pos2::new(v[0], v[1])).collect();
                                tess.tessellate_shape(Shape::line(pts, Stroke::new(ww / s0, c)), &mut mesh);
                            }
                            (Arc::new(mesh), 0)
                        });
                        e.1 = frame;
                        e.0.clone()
                    });
                    let (o, ex, ez) = affine(xf);
                    let b = m.vertices.len() as u32;
                    m.vertices.extend(tm.vertices.iter().map(|v| epaint::Vertex { pos: o + ex * v.pos.x + ez * v.pos.y, uv: epaint::WHITE_UV, color: v.color }));
                    m.indices.extend(tm.indices.iter().map(|i| b + i));
                }
                if !m.indices.is_empty() {
                    out.push(Shape::mesh(m));
                }
            }
        }
    };
    draw_tiles(&mut out, 0..near_start, false);
    draw_tiles(&mut out, near_start..vis.len(), near_start < vis.len());
    if std::env::var_os("SKYWARD_UI_TIMING").is_some() {
        let nv: usize = out.iter().map(|s| if let Shape::Mesh(m) = s { m.vertices.len() } else { 0 }).sum();
        eprintln!("map {:?}: {} tiles ({} far), {} shapes, {} mesh verts, s {:.4}", style, vis.len(), near_start, out.len(), nv, s);
    }
    // Runways: true width when zoomed in, a firm line when zoomed out.
    for rw in &world.runways {
        let (a, b) = (local.to_screen(rw.a), local.to_screen(rw.b));
        let w = (rw.width * s).max(if style == Style::Nd { 2.2 } else { 2.6 });
        let col = if rw.paved || style == Style::Nd { pal.runway } else { Color32::from_rgb(120, 150, 110) };
        out.push(Shape::line_segment([a, b], Stroke::new(w + 1.2, pal.runway_edge)));
        out.push(Shape::line_segment([a, b], Stroke::new(w, col)));
        if style != Style::Nd && rw.width * s > 6.0 {
            // centreline dashes + thresholds
            let d = (b - a).normalized();
            let len = (b - a).length();
            let mut t = 0.08 * len;
            while t < len * 0.92 {
                out.push(Shape::line_segment([a + d * t, a + d * (t + 0.03 * len)], Stroke::new((0.9 * s).max(0.8), Color32::WHITE)));
                t += 0.06 * len;
            }
            let n = EVec2::new(-d.y, d.x) * w * 0.42;
            for (p, dir) in [(a, d), (b, -d)] {
                out.push(Shape::line_segment([p + dir * 2.0 + n, p + dir * 2.0 - n], Stroke::new((3.0 * s).max(1.0), Color32::WHITE)));
            }
        }
    }
    out
}

/// One label candidate, placed by priority.
struct Lab {
    pos: Pos2,
    text: String,
    size: f32,
    col: Color32,
    pri: f32,
    bold: bool,
}

/// Labels from the tiles (places, water) with simple collision avoidance.
pub fn labels(p: &egui::Painter, view: &View, world: &World, tiles: &Tiles, style: Style, taken: &mut Vec<Rect>) {
    let pal = palette(style);
    let s = view.s;
    let mut cands: Vec<Lab> = Vec::new();
    let mut seen: HashMap<String, Pos2> = HashMap::new();
    let reach = view.reach();
    for (id, mv) in tiles {
        // Far tiles carry the big places; near tiles the local ones.
        let xf = Proj::tile_frame(*id).map_to(&world.proj);
        for l in &mv.labels {
            let w = xf.apply(vec2(l.pos[0], l.pos[1]));
            if w.distance(view.centre) > reach {
                continue;
            }
            let (min_s, size, pri) = match l.kind {
                LabelKind::City => (0.0, 15.0, 0.0),
                LabelKind::Town => (0.008, 13.0, 10.0),
                LabelKind::Village => (0.03, 11.5, 20.0),
                LabelKind::Suburb => (0.05, 11.0, 25.0),
                LabelKind::Water => (0.02, 11.0, 30.0),
            };
            if s < min_s {
                continue;
            }
            let q = view.to_screen(w);
            if !view.rect.shrink(4.0).contains(q) {
                continue;
            }
            if let Some(o) = seen.get(&l.text) {
                if o.distance(q) < 200.0 {
                    continue;
                }
            }
            seen.insert(l.text.clone(), q);
            let col = if l.kind == LabelKind::Water { pal.water_label } else { pal.label };
            let text = if l.kind == LabelKind::City { l.text.to_uppercase() } else { l.text.clone() };
            cands.push(Lab { pos: q, text, size, col, pri: pri + l.rank as f32 * 0.5, bold: l.kind <= LabelKind::Town });
        }
    }
    cands.sort_by(|a, b| a.pri.total_cmp(&b.pri));
    for c in cands.into_iter().take(120) {
        let font = if c.bold { f_semi(c.size) } else { f_ui(c.size) };
        let g = p.layout_no_wrap(c.text.clone(), font.clone(), c.col);
        let r = Rect::from_center_size(c.pos, g.size() + EVec2::splat(4.0));
        if taken.iter().any(|t| t.intersects(r)) || !view.rect.contains_rect(r) {
            continue;
        }
        taken.push(r);
        for o in [EVec2::new(1.0, 0.0), EVec2::new(-1.0, 0.0), EVec2::new(0.0, 1.0), EVec2::new(0.0, -1.0)] {
            p.text(c.pos + o, Align2::CENTER_CENTER, &c.text, font.clone(), pal.label_halo);
        }
        p.text(c.pos, Align2::CENTER_CENTER, &c.text, font, c.col);
    }
}

/// Airport symbol (chart style): blue for airports with scheduled service
/// or a major field, magenta otherwise; a bar along the main runway.
pub fn airport_symbol(p: &egui::Painter, q: Pos2, r: f32, major: bool, rwy_hdg_screen: Option<f32>, nd: bool) {
    let col = if nd { pal::CYAN } else if major { Color32::from_rgb(40, 104, 214) } else { Color32::from_rgb(176, 52, 160) };
    p.circle_filled(q, r + 1.5, Color32::from_white_alpha(if nd { 0 } else { 170 }));
    p.circle_stroke(q, r, Stroke::new(2.0, col));
    if let Some(a) = rwy_hdg_screen {
        let d = EVec2::new(a.sin(), -a.cos()) * (r + 3.5);
        p.line_segment([q - d, q + d], Stroke::new(3.0, col));
    }
    if major && !nd {
        for k in 0..4 {
            let a = k as f32 * std::f32::consts::FRAC_PI_2 + std::f32::consts::FRAC_PI_4;
            p.line_segment([polar(q, r, a), polar(q, r + 3.0, a)], Stroke::new(2.0, col));
        }
    }
}

/// Airports from the database (or the tiles) with their codes.
pub fn airports(p: &egui::Painter, view: &View, world: &World, style: Style, taken: &mut Vec<Rect>) {
    let nd = style == Style::Nd;
    let reach = view.reach();
    let mut list: Vec<(Pos2, String, bool, Option<f32>, f32)> = Vec::new();
    if let Some(db) = &world.airport_db {
        let (lat, lon) = world.proj.to_latlon(view.centre);
        for (i, _) in db.near(lat, lon, (reach as f64 / 1000.0).min(400.0)) {
            let a = &db.airports[i];
            if a.kind == crate::airports::Kind::Seaplane {
                continue;
            }
            let major = matches!(a.kind, crate::airports::Kind::Large | crate::airports::Kind::Medium) || a.scheduled;
            // Small strips only when zoomed in.
            if !major && view.s < 0.012 {
                continue;
            }
            let q = view.to_screen(world.proj.to_world(a.lat, a.lon));
            if !view.rect.shrink(6.0).contains(q) {
                continue;
            }
            let hdg = a.runways.first().and_then(|r| r.heading(0)).map(|h| view.screen_angle(h.to_radians()));
            let pri = match a.kind {
                crate::airports::Kind::Large => 0.0,
                crate::airports::Kind::Medium => 1.0,
                _ => 2.0,
            };
            list.push((q, a.code().to_string(), major, hdg, pri));
        }
    } else {
        for a in &world.airports {
            let q = view.to_screen(a.pos);
            if view.rect.shrink(6.0).contains(q) {
                let code = [&a.icao, &a.iata, &a.name].into_iter().find(|c| !c.is_empty()).cloned().unwrap_or_default();
                list.push((q, code, true, None, 1.0));
            }
        }
    }
    list.sort_by(|a, b| a.4.total_cmp(&b.4));
    // Hide the symbol once the runways themselves are big enough to read.
    let detailed = view.s > 0.05;
    for (q, code, major, hdg, _) in list {
        if !detailed {
            airport_symbol(p, q, if major { 6.0 } else { 4.5 }, major, hdg, nd);
        }
        let font = f_inst_b(11.0);
        let col = if nd { pal::CYAN } else if style == Style::Night { Color32::from_rgb(150, 190, 255) } else if major { Color32::from_rgb(28, 78, 170) } else { Color32::from_rgb(140, 40, 128) };
        let g = p.layout_no_wrap(code.clone(), font.clone(), col);
        let at = q + EVec2::new(0.0, if detailed { 0.0 } else { 9.0 });
        let r = Align2::CENTER_TOP.anchor_size(at, g.size()).expand(2.0);
        if taken.iter().any(|t| t.intersects(r)) {
            continue;
        }
        taken.push(r);
        let halo = if nd || style == Style::Night { Color32::from_black_alpha(200) } else { Color32::from_white_alpha(210) };
        for o in [EVec2::new(1.0, 0.0), EVec2::new(-1.0, 0.0), EVec2::new(0.0, 1.0), EVec2::new(0.0, -1.0)] {
            p.text(at + o, Align2::CENTER_TOP, &code, font.clone(), halo);
        }
        p.text(at, Align2::CENTER_TOP, code, font, col);
    }
}

/// Aircraft symbol at `q` pointing along screen angle `a`.
pub fn aircraft_symbol(p: &egui::Painter, q: Pos2, a: f32, size: f32, fill: Color32) {
    let f = EVec2::new(a.sin(), -a.cos());
    let r = EVec2::new(-f.y, f.x);
    let k = size / 12.0;
    let pts: Vec<Pos2> = [
        (0.0, 12.0),
        (1.4, 6.0),
        (1.4, 3.0),
        (10.0, -1.5),
        (10.0, -3.5),
        (1.4, -1.5),
        (1.2, -7.0),
        (4.5, -9.5),
        (4.5, -11.0),
        (0.0, -10.0),
        (-4.5, -11.0),
        (-4.5, -9.5),
        (-1.2, -7.0),
        (-1.4, -1.5),
        (-10.0, -3.5),
        (-10.0, -1.5),
        (-1.4, 3.0),
        (-1.4, 6.0),
    ]
    .iter()
    .map(|&(x, y)| q + r * x * k + f * y * k)
    .collect();
    p.add(Shape::Path(epaint::PathShape { points: pts.clone(), closed: true, fill: Color32::TRANSPARENT, stroke: Stroke::new(3.2, Color32::from_black_alpha(150)).into() }));
    // Non-convex outline: fill as triangles from the centre line.
    let mut m = epaint::Mesh::default();
    for pt in &pts {
        m.colored_vertex(*pt, fill);
    }
    let n = pts.len() as u32;
    let data: Vec<f64> = pts.iter().flat_map(|q| [q.x as f64, q.y as f64]).collect();
    if let Ok(t) = earcutr::earcut(&data, &[], 2) {
        m.indices.extend(t.into_iter().map(|i| i as u32));
    }
    let _ = n;
    p.add(Shape::mesh(m));
}

/// Wind arrow block: arrow showing where the wind blows to (screen frame), text below.
pub fn wind_block(p: &egui::Painter, at: Pos2, view: &View, world: &World, dark: bool) {
    let w = &world.wx;
    let bg = if dark { Color32::from_black_alpha(150) } else { Color32::from_rgba_premultiplied(18, 22, 30, 190) };
    let r = Rect::from_min_size(at, EVec2::new(74.0, 58.0));
    p.rect_filled(r, 8.0, bg);
    let c = Pos2::new(r.center().x, r.min.y + 22.0);
    let col = pal::CYAN;
    if w.wind_kt >= 1.0 {
        let to = view.screen_angle((w.wind_from + 180.0).to_radians());
        let d = EVec2::new(to.sin(), -to.cos());
        let (a, b) = (c - d * 14.0, c + d * 14.0);
        p.line_segment([a, b], Stroke::new(2.2, col));
        let n = EVec2::new(-d.y, d.x);
        p.add(Shape::convex_polygon(vec![b + d * 2.0, b - d * 7.0 + n * 5.0, b - d * 7.0 - n * 5.0], col, Stroke::NONE));
    } else {
        p.circle_stroke(c, 5.0, Stroke::new(1.5, col));
    }
    let txt = if w.wind_kt < 1.0 {
        "CALM".to_string()
    } else if w.gust_kt > w.wind_kt + 3.0 {
        format!("{:03.0}/{:.0}G{:.0}", w.wind_from, w.wind_kt, w.gust_kt)
    } else {
        format!("{:03.0}° {:.0} kt", w.wind_from, w.wind_kt)
    };
    p.text(Pos2::new(r.center().x, r.max.y - 6.0), Align2::CENTER_BOTTOM, txt, f_inst(10.5), pal::TEXT);
}

/// Trend vector: where the aircraft will be in 30 / 60 / 120 s at the
/// current ground speed and turn rate (screen polyline + ticks).
pub fn trend(p: &egui::Painter, view: &View, pos: Vec2, vel: Vec2, turn_rate: f32, col: Color32) {
    if vel.length() < 3.0 {
        return;
    }
    let mut pts = vec![view.to_screen(pos)];
    let mut q = pos;
    let mut v = vel;
    let dt = 2.0;
    let mut ticks = Vec::new();
    for i in 1..=60 {
        q += v * dt;
        let (s, c) = (turn_rate * dt).sin_cos();
        v = vec2(v.x * c - v.y * s, v.x * s + v.y * c);
        pts.push(view.to_screen(q));
        if i == 15 || i == 30 || i == 60 {
            ticks.push((view.to_screen(q), v));
        }
    }
    p.add(Shape::line(pts, Stroke::new(2.0, col)));
    for (t, v) in ticks {
        let a = view.screen_angle(v.x.atan2(-v.y)) + std::f32::consts::FRAC_PI_2;
        let d = EVec2::new(a.sin(), -a.cos()) * 4.0;
        p.line_segment([t - d, t + d], Stroke::new(2.0, col));
    }
}

/// Range rings around `view.anchor` with distance labels.
pub fn range_rings(p: &egui::Painter, view: &View, range_m: f32, nm: bool, col: Color32) {
    let unit = if nm { 1852.0 } else { 1000.0 };
    let step_units = nice_step(range_m / unit / 2.0);
    let step = step_units * unit;
    let mut k = 1;
    while (k as f32) * step <= range_m * 1.42 && k < 8 {
        let r = k as f32 * step * view.s;
        let n = 96;
        // dashed circle
        for i in 0..n {
            if i % 2 == 1 {
                continue;
            }
            let a0 = i as f32 / n as f32 * std::f32::consts::TAU;
            let a1 = (i + 1) as f32 / n as f32 * std::f32::consts::TAU;
            p.line_segment([polar(view.anchor, r, a0), polar(view.anchor, r, a1)], Stroke::new(1.0, col));
        }
        let lab = polar(view.anchor, r, -0.62);
        if view.rect.shrink(10.0).contains(lab) {
            let v = k as f32 * step_units;
            let t = if v.fract() == 0.0 { format!("{:.0}", v) } else { format!("{v:.1}") };
            text_halo(p, lab, Align2::CENTER_CENTER, format!("{t} {}", if nm { "NM" } else { "km" }), f_inst(10.0), col);
        }
        k += 1;
    }
}

fn nice_step(x: f32) -> f32 {
    let e = 10f32.powf(x.max(1e-3).log10().floor());
    let m = x / e;
    let n = if m < 1.5 {
        1.0
    } else if m < 3.5 {
        2.0
    } else if m < 7.5 {
        5.0
    } else {
        10.0
    };
    n * e
}

/// Auto-flight route: line through the remaining waypoints (magenta), names at the points.
pub fn route(p: &egui::Painter, view: &View, from: Vec2, pts: &[(String, Vec2)], col: Color32) {
    if pts.is_empty() {
        return;
    }
    let mut line = vec![view.to_screen(from)];
    line.extend(pts.iter().map(|(_, w)| view.to_screen(*w)));
    for w in line.windows(2) {
        // dashed after the first leg
        p.line_segment([w[0], w[1]], Stroke::new(2.4, col));
    }
    for (i, (name, w)) in pts.iter().enumerate() {
        let q = view.to_screen(*w);
        if !view.rect.expand(-4.0).contains(q) {
            continue;
        }
        let d = if i == 0 { 7.0 } else { 5.5 };
        p.add(Shape::convex_polygon(vec![q + EVec2::new(0.0, -d), q + EVec2::new(d, 0.0), q + EVec2::new(0.0, d), q + EVec2::new(-d, 0.0)], if i == 0 { col } else { Color32::TRANSPARENT }, Stroke::new(1.8, col)));
        text_halo(p, q + EVec2::new(d + 4.0, 0.0), Align2::LEFT_CENTER, name, f_semi(11.0), col);
    }
}

/// Draw a canvas (if ready) into a rounded rect.
pub fn show_canvas(p: &egui::Painter, id: Option<egui::TextureId>, rect: Rect, radius: u8, fallback: Color32) {
    match id {
        Some(id) => {
            p.add(Shape::Rect(epaint::RectShape::filled(rect, radius, Color32::WHITE).with_texture(id, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)))));
        }
        None => {
            p.rect_filled(rect, radius, fallback);
        }
    }
}

pub fn bg(style: Style) -> Color32 {
    palette(style).bg
}

/// Draw the whole map (base canvas + overlays) for the corner / large map.
#[allow(clippy::too_many_arguments)]
pub fn draw_map(
    p: &egui::Painter,
    cv: &mut canvas::Frame,
    key: &'static str,
    ppp: f32,
    view: &View,
    world: &World,
    tiles: &Tiles,
    style: Style,
    radius: u8,
    reserved: &[Rect],
) -> Vec<Rect> {
    let shapes = base_shapes(view, world, tiles, style);
    let id = cv.draw(key, view.rect.size(), ppp, palette(style).bg, shapes);
    show_canvas(p, id, view.rect, radius, palette(style).bg);
    let mp = p.with_clip_rect(view.rect);
    let mut taken = reserved.to_vec();
    airports(&mp, view, world, style, &mut taken);
    labels(&mp, view, world, tiles, style, &mut taken);
    let _ = alpha;
    taken
}
