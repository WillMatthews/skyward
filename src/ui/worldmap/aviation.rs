//! Aviation layers over the world map, drawn crisp on top of the canvas:
//! runways to scale (pavement, edges, centreline, threshold bars, painted
//! numbers) for every airport in view once they're big enough to see, and
//! the full diagram of the airport in focus — airside buildings by class,
//! stands with their numbers and lead-in lines, runway-holding points,
//! helipads, windsocks — with hover tooltips and click-to-pick start
//! positions ([`super::pick`]).

use super::diagram::Diagram;
use super::labels;
use super::pick::{self, Pick};
use super::style::{MapStyle, palette};
use crate::airports::Airport;
use crate::ui::overview::View;
use crate::ui::{f_inst_b, f_ui, pal, thousands};
use egui::{Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2, epaint};

/// Zoom where runways are drawn to scale.
pub const RUNWAYS_FROM: f64 = 10.5;
/// Zoom where the focused airport's diagram appears.
pub const DIAGRAM_FROM: f64 = 13.0;
/// Zoom where stands appear.
pub const STANDS_FROM: f64 = 14.3;

/// The start currently chosen at this airport (highlighted on the diagram).
#[derive(Clone, Debug, PartialEq)]
pub enum Chosen {
    None,
    End(String),
    Final(String, f32),
    Spot(f64, f64, f32),
}

/// Unit screen direction of a compass heading (deg).
fn dir(h: f32) -> Vec2 {
    let r = h.to_radians();
    Vec2::new(r.sin(), -r.cos())
}

/// Runways of `a` to scale. `detail`: markings and painted numbers.
pub fn runways(p: &egui::Painter, v: &View, a: &Airport, style: MapStyle, detail: bool) {
    let pl = palette(style);
    let mpp = v.m_per_pt() as f32;
    for rw in a.runways.iter().filter(|r| !r.ends[0].ident.starts_with('H') && r.length_m >= 100.0) {
        let Some([e0, e1]) = pick::runway_ends(a, rw) else { continue };
        let (pa, pb) = (v.ll_to_screen(e0.0, e0.1), v.ll_to_screen(e1.0, e1.1));
        let len = (pb - pa).length();
        if len < 3.0 {
            continue;
        }
        let d = (pb - pa) / len;
        let n = d.rot90();
        let w = (rw.width_m.max(15.0) / mpp).max(2.5);
        let paved = rw.surface.paved();
        let fill = if paved { pl.runway } else { Color32::from_rgb(110, 150, 96) };
        let quad = |w: f32, c: Color32| Shape::convex_polygon(vec![pa + n * w * 0.5, pb + n * w * 0.5, pb - n * w * 0.5, pa - n * w * 0.5], c, Stroke::NONE);
        p.add(quad(w + 2.0, pl.runway_edge));
        p.add(quad(w, fill));
        if !detail || w < 7.0 || !paved {
            continue;
        }
        let white = Color32::from_white_alpha(235);
        // centreline dashes (30 m dash, 20 m gap) clear of the numbers
        let dash = (30.0 / mpp).max(2.0);
        let gap = (20.0 / mpp).max(2.0);
        let clear = (180.0 / mpp).min(len * 0.25);
        let mut t = clear;
        let lw = (0.9 / mpp).clamp(0.8, 2.0);
        while t + dash < len - clear {
            p.line_segment([pa + d * t, pa + d * (t + dash)], Stroke::new(lw, white));
            t += dash + gap;
        }
        // thresholds: piano keys, numbers, displaced thresholds
        for (end, (pt, dd)) in [(pa, d), (pb, -d)].into_iter().enumerate() {
            let e = &rw.ends[end];
            let disp = e.displaced_m / mpp;
            let thr = pt + dd * disp;
            if disp > 1.0 {
                p.line_segment([thr - n * w * 0.5, thr + n * w * 0.5], Stroke::new((1.0 / mpp).max(1.2), white));
                // displaced area arrows: a dashed centre
                let mut t = 0.0;
                while t + 3.0 < disp {
                    p.line_segment([pt + dd * t, pt + dd * (t + (disp * 0.12).max(2.0))], Stroke::new(lw, white));
                    t += (disp * 0.25).max(5.0);
                }
            }
            if w > 14.0 {
                let keys = 8;
                let kl = (30.0 / mpp).max(3.0);
                let kw = w * 0.9 / (keys as f32 * 2.0);
                for k in 0..keys {
                    let off = -w * 0.45 + kw * (0.5 + 2.0 * k as f32) + kw * 0.5;
                    let a0 = thr + dd * (6.0 / mpp) + n * off;
                    p.line_segment([a0, a0 + dd * kl], Stroke::new(kw, white));
                }
            }
            if w > 18.0 && !e.ident.is_empty() {
                let h = rw.heading(end).unwrap_or(0.0);
                let size = (w * 0.55).clamp(9.0, 60.0);
                let at = thr + dd * ((60.0 / mpp).max(size * 1.4));
                let g = p.layout_no_wrap(e.ident.clone(), f_inst_b(size), white);
                let ang = h.to_radians();
                let (s, c) = ang.sin_cos();
                let half = g.size() * 0.5;
                let tl = at - Vec2::new(half.x * c - half.y * s, half.x * s + half.y * c);
                p.add(epaint::TextShape::new(tl, g, white).with_angle(ang));
            }
        }
    }
}

/// A stand's lead-in line and stop marker.
fn stand_symbol(p: &egui::Painter, q: Pos2, hdg: f32, r: f32, lead: f32, col: Color32, contact: bool) {
    let f = dir(hdg);
    p.line_segment([q - f * lead, q], Stroke::new((r * 0.35).clamp(1.0, 2.5), col.gamma_multiply(0.8)));
    if contact {
        let n = f.rot90();
        p.add(Shape::convex_polygon(
            vec![q + n * r + f * r * 0.6, q - n * r + f * r * 0.6, q - n * r - f * r, q + n * r - f * r],
            col,
            Stroke::new(1.0, Color32::from_black_alpha(140)),
        ));
    } else {
        p.circle_filled(q, r, col);
        p.circle_stroke(q, r, Stroke::new(1.0, Color32::from_black_alpha(140)));
    }
}

/// What the pointer did on the diagram.
#[derive(Default)]
pub struct Out {
    pub clicked: Option<Pick>,
    /// Hovered target: tooltip title, detail line, anchor.
    pub hover: Option<(String, String, Pos2)>,
}

/// Inputs for the focused airport's diagram.
pub struct Focus<'a> {
    pub style: MapStyle,
    pub diag: Option<&'a Diagram>,
    pub chosen: Chosen,
    /// Wind (from deg, kt) for the runway tooltips.
    pub wind: Option<(f32, f32)>,
    /// Pointer over the map (not dragging), and a click there this frame.
    pub hover: Option<Pos2>,
    pub click: Option<Pos2>,
}

/// Is `a` big enough on screen to draw its runways to scale?
pub fn to_scale(a: &Airport, v: &View) -> bool {
    v.zoom >= RUNWAYS_FROM && a.longest_m() / v.m_per_pt() as f32 > 18.0
}

/// The diagram's ground layer (under the map labels): airside buildings,
/// runway-holding points, windsocks, helipads.
pub fn diagram_base(p: &egui::Painter, v: &View, d: &Diagram, style: MapStyle, wind: Option<(f32, f32)>) {
    let pl = palette(style);
    let mpp = v.m_per_pt() as f32;
    let z = v.zoom;
    let f = Focus { style, diag: Some(d), chosen: Chosen::None, wind, hover: None, click: None };
    // Airside buildings by class (terminals stand out).
    if z >= DIAGRAM_FROM {
        let a_in = ((z - DIAGRAM_FROM) / 0.6).clamp(0.0, 1.0) as f32;
        for (class, ring, tris) in &d.buildings {
            let col = match class {
                0 => Color32::from_rgb(92, 140, 205),
                1 => Color32::from_rgb(150, 156, 170),
                2 => Color32::from_rgb(205, 120, 80),
                _ => Color32::from_rgb(170, 160, 150),
            };
            let mut m = epaint::Mesh::default();
            for (la, lo) in ring {
                m.colored_vertex(v.ll_to_screen(*la, *lo), col.gamma_multiply(0.32 * a_in));
            }
            m.indices.extend(tris);
            p.add(Shape::mesh(m));
            let pts: Vec<Pos2> = ring.iter().map(|(la, lo)| v.ll_to_screen(*la, *lo)).collect();
            p.add(Shape::closed_line(pts, Stroke::new(1.0, col.gamma_multiply(0.9 * a_in))));
        }
    }
    // Runway-holding points: the red stop bar across the taxiway.
    if z >= 15.0 {
        for h in &d.holds {
            let q = v.ll_to_screen(h.lat, h.lon);
            if !v.rect.contains(q) {
                continue;
            }
            let n = dir(h.dir).rot90();
            let hw = (h.width.max(15.0) * 0.5 / mpp).max(5.0);
            p.line_segment([q - n * hw, q + n * hw], Stroke::new(2.5, pl.hold));
            let back = -dir(h.dir) * 3.0;
            p.line_segment([q - n * hw + back, q + n * hw + back], Stroke::new(1.0, Color32::from_rgb(250, 206, 60)));
        }
    }
    // Windsocks.
    if z >= 14.0 {
        for s in &d.socks {
            let q = v.ll_to_screen(s.0, s.1);
            let w = dir(f.wind.map(|w| w.0 + 180.0).unwrap_or(90.0)) * 11.0;
            let n = w.rot90() * 0.28;
            p.line_segment([q, q + Vec2::new(0.0, -2.0)], Stroke::new(1.5, Color32::WHITE));
            p.add(Shape::convex_polygon(
                vec![q + n * 1.4, q + w + n * 0.5, q + w - n * 0.5, q - n * 1.4],
                Color32::from_rgb(255, 120, 30),
                Stroke::new(1.0, Color32::from_black_alpha(160)),
            ));
        }
    }
    // Helipads.
    if z >= 13.5 {
        for pad in &d.pads {
            let q = v.ll_to_screen(pad.lat, pad.lon);
            let r = (pad.size * 0.5 / mpp).clamp(7.0, 40.0);
            p.circle_filled(q, r, Color32::from_rgb(40, 120, 70));
            p.circle_stroke(q, r, Stroke::new(1.5, Color32::WHITE));
            p.text(q, Align2::CENTER_CENTER, "H", f_inst_b((r * 1.1).clamp(9.0, 30.0)), Color32::WHITE);
        }
    }
}

/// The diagram of the airport in focus (stands, the chosen start, picking); `taken`: label boxes so far.
pub fn diagram(p: &egui::Painter, v: &View, a: &Airport, f: &Focus, taken: &mut Vec<Rect>) -> Out {
    let pl = palette(f.style);
    let mpp = v.m_per_pt() as f32;
    let z = v.zoom;
    let mut out = Out::default();
    if let Some(d) = f.diag {
        // Stands: lead-in and stop marker, numbers where they fit.
        if z >= STANDS_FROM {
            let r = (4.0 / mpp).clamp(3.0, 9.0);
            let mut cands = Vec::new();
            let mut texts = Vec::new();
            for s in &d.stands {
                let q = v.ll_to_screen(s.lat, s.lon);
                if !v.rect.expand(20.0).contains(q) {
                    continue;
                }
                let sel = matches!(f.chosen, Chosen::Spot(la, lo, _) if (la - s.lat).abs() < 1e-7 && (lo - s.lon).abs() < 1e-7);
                // Up close, the stand's size: a ghost of the largest aircraft it takes.
                if z >= 16.5 && !sel {
                    let a = ((z - 16.5) / 0.7).clamp(0.0, 1.0) as f32 * 0.22;
                    // (the symbol's span is 20/12 of its size; its nose is `size` ahead)
                    let size = s.span / mpp * 0.6;
                    crate::ui::map::aircraft_symbol(p, q - dir(s.hdg) * size, s.hdg.to_radians(), size, pl.stand.gamma_multiply(a));
                }
                stand_symbol(p, q, s.hdg, r, (s.span * 0.55 / mpp).clamp(6.0, 120.0), if sel { pal::AMBER } else { pl.stand }, s.contact);
                if z >= 15.4 && !s.name.is_empty() {
                    let g = p.layout_no_wrap(s.name.clone(), f_inst_b(10.5), pl.stand_text);
                    let at = q - dir(s.hdg) * ((s.span * 0.55 / mpp).clamp(6.0, 120.0) + 9.0);
                    let rect = Rect::from_center_size(at, g.size() + Vec2::new(6.0, 3.0));
                    cands.push(labels::Cand { pri: if sel { -1.0 } else { 0.0 }, rect, key: labels::key(&s.name) });
                    texts.push((rect, g, sel));
                }
            }
            for i in labels::place(&cands, taken, 400) {
                let (rect, g, sel) = &texts[i];
                p.rect_filled(*rect, 3, if *sel { pal::AMBER } else { pl.stand });
                p.galley(rect.center() - g.size() * 0.5, g.clone(), pl.stand_text);
                taken.push(*rect);
            }
        }
    }
    // The chosen start: an aircraft on its spot / threshold.
    match &f.chosen {
        Chosen::Spot(la, lo, h) => crate::ui::map::aircraft_symbol(p, v.ll_to_screen(*la, *lo), h.to_radians(), 18.0, pal::AMBER),
        Chosen::End(e) | Chosen::Final(e, _) => {
            for rw in &a.runways {
                let Some(ends) = pick::runway_ends(a, rw) else { continue };
                for (i, end) in rw.ends.iter().enumerate() {
                    if end.ident != *e {
                        continue;
                    }
                    let (pt, other) = (v.ll_to_screen(ends[i].0, ends[i].1), v.ll_to_screen(ends[1 - i].0, ends[1 - i].1));
                    let dd = (other - pt).normalized();
                    if let Chosen::Final(_, nm) = f.chosen {
                        let far = pt - dd * (nm * 1852.0 / mpp);
                        for k in 0..20 {
                            let t0 = k as f32 / 20.0;
                            p.line_segment([pt + (far - pt) * t0, pt + (far - pt) * (t0 + 0.025)], Stroke::new(2.0, pal::MAGENTA));
                        }
                        crate::ui::map::aircraft_symbol(p, far, dd.x.atan2(-dd.y), 14.0, pal::AMBER);
                    } else if z >= DIAGRAM_FROM {
                        crate::ui::map::aircraft_symbol(p, pt + dd * (60.0 / mpp).max(10.0), dd.x.atan2(-dd.y), 16.0, pal::AMBER);
                    }
                }
            }
        }
        Chosen::None => {}
    }
    // Hover / click: what's under the pointer.
    let hit = |q: Pos2| -> Option<Pick> {
        let ll = v.to_ll(q);
        let d = if z >= 13.5 { f.diag } else { None };
        pick::pick(a, d, ll, mpp as f64)
    };
    if let Some(q) = f.click {
        out.clicked = hit(q);
    }
    if let Some(q) = f.hover {
        if let Some(h) = hit(q) {
            out.hover = Some(describe(a, f, &h, q));
        }
    }
    out
}

fn describe(a: &Airport, f: &Focus, h: &Pick, q: Pos2) -> (String, String, Pos2) {
    match h {
        Pick::RunwayEnd(e) => {
            let rw = a.runways.iter().find(|r| r.ends.iter().any(|x| x.ident == *e));
            let sub = rw
                .map(|r| {
                    let end = r.ends.iter().position(|x| x.ident == *e).unwrap_or(0);
                    let wind = match (f.wind, r.heading(end)) {
                        (Some((from, kt)), Some(hd)) if kt >= 1.0 => {
                            let (hw, xw) = crate::airports::wind_components(hd, from, kt);
                            format!(" · {}{:.0} X{:.0} kt", if hw >= 0.0 { "head " } else { "tail " }, hw.abs(), xw.abs())
                        }
                        _ => String::new(),
                    };
                    format!("{} × {} m · {}{}{}", thousands(r.length_m), r.width_m.round(), r.surface.name(), if r.lighted { " · lit" } else { "" }, wind)
                })
                .unwrap_or_default();
            (format!("Runway {e} — click to line up"), sub, q)
        }
        Pick::Stand { name, .. } => {
            let s = f.diag.and_then(|d| d.stands.iter().find(|s| s.name == *name));
            let sub = s.map(|s| format!("{} · up to {:.0} m span", if s.contact { "Contact stand (jet bridge)" } else { "Remote stand" }, s.span)).unwrap_or_default();
            (format!("Stand {name} — click to start here"), sub, q)
        }
        Pick::Pad { name, .. } => (format!("Helipad {name} — click to start here"), "Rotorcraft start".into(), q),
    }
}

/// A tooltip card next to `q`.
pub fn tooltip(p: &egui::Painter, title: &str, sub: &str, q: Pos2, bounds: Rect) {
    let g1 = p.layout_no_wrap(title.to_string(), crate::ui::f_semi(13.0), pal::TEXT);
    let g2 = p.layout_no_wrap(sub.to_string(), f_ui(12.0), pal::TEXT2);
    let w = g1.size().x.max(g2.size().x) + 20.0;
    let h = if sub.is_empty() { 28.0 } else { 46.0 };
    let mut r = Rect::from_min_size(q + Vec2::new(16.0, -h * 0.5), Vec2::new(w, h));
    if r.max.x > bounds.max.x - 8.0 {
        r = r.translate(Vec2::new(-(w + 32.0), 0.0));
    }
    p.rect_filled(r.expand(1.0), 8, Color32::from_black_alpha(120));
    p.rect_filled(r, 8, pal::PANEL_SOLID);
    p.galley(r.min + Vec2::new(10.0, 6.0), g1, pal::TEXT);
    if !sub.is_empty() {
        p.galley(r.min + Vec2::new(10.0, 25.0), g2, pal::TEXT2);
    }
}

/// Scale bar (bottom-left of the map): a round distance in metres / km and feet / NM.
pub fn scale_bar(p: &egui::Painter, v: &View, at: Pos2, style: MapStyle) {
    let mpp = v.m_per_pt();
    let target = 120.0 * mpp;
    let nice = |x: f64| {
        let e = 10f64.powf(x.log10().floor());
        let m = x / e;
        (if m >= 5.0 {
            5.0
        } else if m >= 2.0 {
            2.0
        } else {
            1.0
        }) * e
    };
    let m = nice(target);
    let wm = (m / mpp) as f32;
    // imperial: NM once a mile fits, feet below
    let (wn, fmt_n) = if target >= 1852.0 * 0.5 {
        let nm = nice(target / 1852.0);
        ((nm * 1852.0 / mpp) as f32, format!("{nm} NM"))
    } else {
        let ft = nice(target * 3.28084);
        ((ft / 3.28084 / mpp) as f32, format!("{ft} ft"))
    };
    let (fg, halo) = match style {
        MapStyle::Day => (Color32::from_rgb(40, 44, 52), Color32::from_white_alpha(200)),
        MapStyle::Night => (Color32::from_rgb(220, 226, 234), Color32::from_black_alpha(170)),
    };
    let fmt_m = if m >= 1000.0 { format!("{} km", m / 1000.0) } else { format!("{m} m") };
    let bg = Rect::from_min_max(at + Vec2::new(-6.0, -30.0), at + Vec2::new(wm.max(wn) + 60.0, 18.0));
    p.rect_filled(bg, 6, halo.gamma_multiply(0.75));
    for (y, w, t) in [(-8.0, wm, fmt_m), (6.0, wn, fmt_n)] {
        let a = at + Vec2::new(0.0, y);
        p.line_segment([a, a + Vec2::new(w, 0.0)], Stroke::new(2.0, fg));
        p.line_segment([a + Vec2::new(0.0, -4.0), a + Vec2::new(0.0, 4.0)], Stroke::new(1.5, fg));
        p.line_segment([a + Vec2::new(w, -4.0), a + Vec2::new(w, 4.0)], Stroke::new(1.5, fg));
        p.text(a + Vec2::new(w + 6.0, 0.0), Align2::LEFT_CENTER, t, f_ui(11.0), fg);
    }
}
