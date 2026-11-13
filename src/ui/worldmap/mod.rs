//! The world map (departure screen airport picker, menu backdrops): a
//! continuous level-of-detail vector map from the whole globe down to one
//! stand. OpenFreeMap tiles z0–z14 (overzoomed beyond) are chosen by their
//! size on screen ([`pyramid`]), loaded centre-first in the background and
//! kept under a memory budget ([`store`]), decoded off the main thread into
//! fills and width-independent stroke meshes ([`decode`]) and drawn coarse
//! to fine through an MSAA canvas, each tile over its own background so a
//! finer tile covers its fallback as it fades in. Cartography by zoom
//! ([`style`]), collision-free labels ([`labels`]), airport diagrams
//! ([`diagram`], [`aviation`]) with start-position picking ([`pick`]) and
//! smooth fly-to ([`flyto`]).

pub mod aviation;
pub mod decode;
pub mod diagram;
pub mod flyto;
pub mod gpu;
pub mod labels;
pub mod pick;
pub mod pyramid;
pub mod store;
pub mod style;

use super::canvas;
use super::overview::View;
use super::{f_inst_b, f_semi, f_ui};
use decode::*;
use egui::{Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2, epaint};
use pyramid::{Draw, cover, plan, tile_zoom};
use std::collections::HashSet;
use std::sync::Arc;
use style::{MapStyle, Palette, fill_alpha, label_rule, line_width, m_per_pt, palette};

/// How to draw a map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Opts {
    /// Canvas key (one texture per map).
    pub key: &'static str,
    /// Deepest tile level to load (backdrops stay coarse).
    pub max_z: u32,
    pub style: MapStyle,
    pub labels: bool,
    /// Gate refs from the tiles (off once an airport diagram shows the stands).
    pub gates: bool,
}

impl Default for Opts {
    fn default() -> Self {
        Opts { key: "overview", max_z: pyramid::MAX_TILE_Z, style: MapStyle::Night, labels: true, gates: true }
    }
}

/// Frame statistics (perf overlay / logs).
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub tiles_drawn: usize,
    pub verts: usize,
    pub build_ms: f32,
    pub redrawn: bool,
    /// Tiles still fading in.
    pub fading: bool,
}

pub struct WorldMap {
    pub store: store::Store,
    pub diagrams: diagram::Diagrams,
    /// What the canvas shows (redraw only when it changes).
    last: Option<(f64, f64, f64, [f32; 4], MapStyle, u64, u32, &'static str)>,
    pub stats: Stats,
}

impl Default for WorldMap {
    fn default() -> Self {
        Self::new()
    }
}

/// The zoom's colours and line half-widths for the GPU pass.
fn look(pal: &Palette, zoom: f64, mpp: f64) -> gpu::Look {
    let mut l = gpu::Look::default();
    for c in 0..N_FILLS {
        l.col[1][c] = gpu::rgba(pal.fills[c].gamma_multiply(fill_alpha(c as u8, zoom)));
    }
    l.col[1][gpu::C_BG as usize] = gpu::rgba(pal.bg);
    l.hw[gpu::C_BG as usize] = [0.0, 0.35, 0.0, 0.0];
    for c in 0..N_LINES {
        let s = gpu::line_slot(c as u8);
        let Some((hw, cas, a)) = line_width(c as u8, zoom, mpp) else { continue };
        let (fill, case) = pal.lines[c];
        let centre = if c as u8 == L_TAXIWAY { (hw * 0.07).clamp(0.5, 1.3) } else { 0.0 };
        l.hw[s] = [if cas > 0.0 && case.a() > 0 { hw + cas } else { 0.0 }, hw, centre, 0.0];
        l.col[0][s] = gpu::rgba(case.gamma_multiply(a));
        l.col[1][s] = gpu::rgba(fill.gamma_multiply(a));
        if c as u8 == L_TAXIWAY {
            let ca = ((zoom - 14.5) / 0.5).clamp(0.0, 1.0) as f32;
            l.col[2][s] = gpu::rgba(pal.taxi_centre.gamma_multiply(ca));
        }
    }
    l
}

/// Tile origin (screen) and tile size (points) of a slot in a view (no
/// antimeridian wrap: slots carry unwrapped columns in the view's frame).
fn place(v: &View, d: &Draw) -> (Pos2, f32) {
    let (ox, oy, s) = d.slot.rect();
    let sc = v.scale();
    let c = v.rect.center();
    (Pos2::new(c.x + ((ox - v.cx) * sc) as f32, c.y + ((oy - v.cy) * sc) as f32), (s * sc) as f32)
}

impl WorldMap {
    pub fn new() -> WorldMap {
        WorldMap { store: store::Store::new(), diagrams: diagram::Diagrams::default(), last: None, stats: Stats::default() }
    }

    /// Tiles / diagrams still loading (or fading in).
    pub fn busy(&self) -> usize {
        self.store.busy() + self.diagrams.busy() + self.stats.fading as usize
    }

    /// Draw the map into `v.rect`; returns the boxes its labels took (for
    /// the caller's symbols to avoid).
    /// `blocked`: screen boxes labels keep clear of (controls over the map);
    /// `under`: drawn over the map but under its labels.
    #[allow(clippy::too_many_arguments)]
    pub fn draw(&mut self, p: &egui::Painter, cv: &mut canvas::Frame, ppp: f32, v: &View, o: &Opts, blocked: &[Rect], under: &mut dyn FnMut(&egui::Painter)) -> Vec<Rect> {
        self.store.poll();
        let now = self.store.now();
        let (x0, y0) = v.to_merc(v.rect.min);
        let (x1, y1) = v.to_merc(v.rect.max);
        let z = tile_zoom(v.zoom, o.max_z);
        let want: Vec<pyramid::Slot> = cover(x0, y0, x1, y1, v.cx, v.cy, z).into_iter().map(|s| s.0).collect();
        let (draws, mut missing) = plan(&want, |id| self.store.alpha(id, now));
        // The whole world as the last-resort backdrop.
        missing.push(crate::osm::TileId { z: 0, x: 0, y: 0 });
        self.store.request(&missing);
        for d in &draws {
            self.store.touch(d.slot.id);
        }
        let pal = palette(o.style);
        let fading = draws.iter().any(|d| d.alpha < 1.0);
        self.stats.fading = fading;
        let sig = (v.cx, v.cy, v.zoom, [v.rect.min.x, v.rect.min.y, v.rect.max.x, v.rect.max.y], o.style, self.store.generation, o.max_z, o.key);
        let size = v.rect.size();
        let have_tex = cv.ids.contains_key(o.key);
        self.stats.redrawn = false;
        if (fading || self.last != Some(sig) || !have_tex) && cv.due(o.key, size) {
            let t0 = web_time::Instant::now();
            let lat = super::overview::unmerc(0.5, v.cy.clamp(0.0, 1.0)).0;
            let look = look(&pal, v.zoom, m_per_pt(lat, v.zoom));
            let shift = v.rect.min.to_vec2();
            let mut items = Vec::with_capacity(draws.len());
            let mut verts = 0;
            for d in &draws {
                let Some(t) = self.store.get(d.slot.id) else { continue };
                let (org, k) = place(v, d);
                verts += t.fills.iter().map(|f| f.1.len()).sum::<usize>() + t.lines.iter().map(|l| l.1.base.len()).sum::<usize>();
                items.push(gpu::Item { id: d.slot.id, tile: t.clone(), origin: org - shift, k, alpha: d.alpha });
            }
            self.stats.tiles_drawn = items.len();
            self.stats.verts = verts;
            let cb = gpu::MapPaint { size, items, look, centre: v.zoom >= 14.5 };
            let shape = Shape::Callback(egui_wgpu::Callback::new_paint_callback(Rect::from_min_size(Pos2::ZERO, size), cb));
            cv.draw(o.key, size, ppp, pal.bg, vec![shape]);
            self.stats.build_ms = t0.elapsed().as_secs_f32() * 1000.0;
            self.stats.redrawn = true;
            self.last = Some(sig);
        }
        match cv.ids.get(o.key) {
            Some(id) => canvas::image(p, *id, v.rect, Color32::WHITE),
            None => {
                p.rect_filled(v.rect, 0, pal.bg);
            }
        }
        under(p);
        let taken = if o.labels { self.labels(p, v, &draws, &pal, o, blocked) } else { Vec::new() };
        self.store.end_frame();
        taken
    }

    fn labels(&self, p: &egui::Painter, v: &View, draws: &[Draw], pal: &Palette, o: &Opts, blocked: &[Rect]) -> Vec<Rect> {
        struct L {
            at: Pos2,
            text: String,
            kind: u8,
            angle: f32,
            font: egui::FontId,
            col: Color32,
        }
        let mut ls: Vec<L> = Vec::new();
        let mut cands = Vec::new();
        let mut seen: HashSet<(u8, String)> = HashSet::new();
        let mut near: Vec<(u8, String, Pos2)> = Vec::new();
        let view = v.rect.shrink(4.0);
        // Finest tiles first, so their copy of a label wins the de-duplication.
        for d in draws.iter().rev() {
            let Some(t) = self.store.get(d.slot.id) else { continue };
            if d.alpha < 0.5 {
                continue;
            }
            let (org, k) = place(v, d);
            for l in &t.labels {
                if l.kind == K_GATE && !o.gates {
                    continue;
                }
                let Some((pri, size)) = label_rule(l.kind, l.rank, v.zoom) else { continue };
                let at = Pos2::new(org.x + l.pos[0] * k, org.y + l.pos[1] * k);
                if !view.contains(at) {
                    continue;
                }
                // Repeated refs (both carriageways, every segment): one per neighbourhood.
                let cell = match l.kind {
                    K_TAXI => 260.0,
                    K_ROAD => 420.0,
                    K_SHIELD => 340.0,
                    _ => 90.0,
                };
                let dedupe = if matches!(l.kind, K_TAXI | K_GATE | K_ROAD | K_PAD | K_SHIELD) {
                    format!("{}@{},{}", l.text, (at.x / cell).floor() as i32, (at.y / cell).floor() as i32)
                } else {
                    l.text.clone()
                };
                if !seen.insert((l.kind, dedupe)) {
                    continue;
                }
                if matches!(l.kind, K_TAXI | K_ROAD | K_SHIELD) && near.iter().any(|(k, t, q): &(u8, String, Pos2)| *k == l.kind && *t == l.text && q.distance(at) < cell * 0.7) {
                    continue;
                }
                near.push((l.kind, l.text.clone(), at));
                let (font, col) = match l.kind {
                    K_COUNTRY => (f_semi(size), pal.label2),
                    K_STATE => (f_ui(size), pal.label2),
                    K_CITY => (f_semi(size), pal.label),
                    K_SEA | K_LAKE | K_RIVER => (f_ui(size), pal.water_label),
                    K_SHIELD => (f_inst_b(size), pal.shield_fg),
                    K_TAXI => (f_inst_b(size), Color32::from_rgb(250, 206, 60)),
                    K_GATE | K_PAD => (f_inst_b(size), pal.stand_text),
                    K_ROAD => (f_ui(size), pal.label2),
                    _ => (f_ui(size), pal.label),
                };
                let text = if l.kind == K_COUNTRY { l.text.to_uppercase() } else { l.text.clone() };
                let g = p.layout_no_wrap(text.clone(), font.clone(), col);
                let pad = match l.kind {
                    K_SHIELD | K_TAXI | K_GATE | K_PAD => Vec2::new(8.0, 4.0),
                    _ => Vec2::new(4.0, 2.0),
                };
                let sz = g.size() + pad;
                let rect = if l.angle != 0.0 { labels::rotated_bounds(at, sz.x, sz.y, l.angle) } else { Rect::from_center_size(at, sz) };
                cands.push(labels::Cand { pri, rect, key: labels::key(&text) ^ (l.kind as u64) << 56 });
                ls.push(L { at, text, kind: l.kind, angle: l.angle, font, col });
            }
        }
        let placed = labels::place(&cands, blocked, 260);
        let mut taken = Vec::with_capacity(placed.len());
        for i in placed {
            let l = &ls[i];
            taken.push(cands[i].rect);
            match l.kind {
                K_SHIELD | K_TAXI | K_GATE | K_PAD => {
                    let r = Rect::from_center_size(l.at, cands[i].rect.size());
                    let (bg, border) = match l.kind {
                        K_SHIELD => (pal.shield_bg, Color32::from_white_alpha(140)),
                        // ICAO location sign: yellow on black.
                        K_TAXI => (Color32::from_rgb(16, 16, 16), Color32::from_rgb(250, 206, 60)),
                        K_PAD => (Color32::from_rgb(40, 120, 70), Color32::WHITE),
                        _ => (pal.stand, Color32::WHITE),
                    };
                    p.rect_filled(r, 3, bg);
                    p.rect_stroke(r, 3, Stroke::new(1.0, border), egui::StrokeKind::Inside);
                    p.text(r.center(), Align2::CENTER_CENTER, &l.text, l.font.clone(), l.col);
                }
                _ if l.angle != 0.0 => {
                    let g = p.layout_no_wrap(l.text.clone(), l.font.clone(), l.col);
                    let (s, c) = l.angle.sin_cos();
                    let h = g.size() * 0.5;
                    let tl = l.at - Vec2::new(h.x * c - h.y * s, h.x * s + h.y * c);
                    for d in [Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(0.0, -1.0)] {
                        p.add(epaint::TextShape::new(tl + d, g.clone(), pal.halo).with_override_text_color(pal.halo).with_angle(l.angle));
                    }
                    p.add(epaint::TextShape::new(tl, g, l.col).with_angle(l.angle));
                }
                _ => {
                    for d in [Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(0.0, -1.0)] {
                        p.text(l.at + d, Align2::CENTER_CENTER, &l.text, l.font.clone(), pal.halo);
                    }
                    p.text(l.at, Align2::CENTER_CENTER, &l.text, l.font.clone(), l.col);
                }
            }
        }
        taken
    }

    /// The airport diagram of `ap` (built in the background on first ask).
    pub fn diagram(&mut self, ap: &crate::airports::Airport, db: &Arc<crate::airports::Db>) -> Option<Arc<diagram::Diagram>> {
        self.diagrams.get(ap, db)
    }
}

/// Browser: tiles are decoded (triangulated) in a tile worker, not on the
/// page (tens of ms each: hitches while the map pans).
#[cfg(target_arch = "wasm32")]
pub mod web {
    use super::decode::{MapTile, decode};
    use crate::osm::TileId;
    use crate::platform::wire::{self, R, W, Wire};

    /// Worker job "overview" (tiles_web.rs): (z, x, y) + the vector tile → the decoded tile.
    pub fn decode_job(data: &[u8]) -> Option<Vec<u8>> {
        let mut r = R::new(data);
        let id = TileId { z: u32::get(&mut r).ok()?, x: u32::get(&mut r).ok()?, y: u32::get(&mut r).ok()? };
        Some(wire::encode(&decode(id, &data[12..])))
    }

    pub async fn decode_off_page(id: TileId, bytes: Vec<u8>) -> MapTile {
        let mut w = W::default();
        id.z.put(&mut w);
        id.x.put(&mut w);
        id.y.put(&mut w);
        w.0.extend_from_slice(&bytes);
        if let Some(f) = crate::platform::web::offload_begin("overview", w.0) {
            if let Some(t) = f.await.and_then(|b| wire::decode::<MapTile>(&b).ok()) {
                return t;
            }
        }
        decode(id, &bytes)
    }
}
