//! Analogue instruments for light and vintage types: Cessna-style six-pack
//! (knots, feet), Spitfire "basic six" with boost and RPM (mph, feet),
//! glider panel with a total-energy variometer and MacCready speed ring
//! (km/h, m/s, metres). Bezels, dial faces, needles and glass are all
//! vector paths (egui feathering), sized from one radius.

use super::{arc, bezel_ring, f_inst, f_inst_b, gradient_disc, lerp_col, pal, polar, ring_sector};
use crate::flight::{AircraftDef, Category};
use crate::sim::Sim;
use egui::{epaint, Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2};
use std::f32::consts::{PI, TAU};

const FACE: Color32 = Color32::from_rgb(18, 18, 19);
const INK: Color32 = Color32::from_rgb(242, 239, 230);
const INK_DIM: Color32 = Color32::from_rgb(170, 168, 160);
/// Vintage luminous paint (Spitfire dials).
const RADIUM: Color32 = Color32::from_rgb(226, 232, 196);
const ARC_WHITE: Color32 = Color32::from_rgb(236, 236, 236);
const ARC_GREEN: Color32 = Color32::from_rgb(46, 178, 76);
const ARC_YELLOW: Color32 = Color32::from_rgb(236, 196, 40);
const ARC_RED: Color32 = Color32::from_rgb(222, 46, 40);

/// Which analogue panel a type gets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Panel {
    Trainer,
    Warbird,
    Glider,
}

pub fn panel_for(def: &AircraftDef) -> Panel {
    match def.category {
        Category::Glider => Panel::Glider,
        Category::Warbird => Panel::Warbird,
        _ => Panel::Trainer,
    }
}

// ---------------------------------------------------------------- dial parts

/// Instrument case: metal bezel ring, black face with a soft inner shadow.
fn case(p: &egui::Painter, c: Pos2, r: f32) {
    p.add(epaint::Shadow { offset: [0, 3], blur: 10, spread: 0, color: Color32::from_black_alpha(120) }.as_shape(Rect::from_center_size(c, Vec2::splat(2.0 * r + 10.0)), (r + 5.0) as u8));
    bezel_ring(p, c, r, r + 5.5, Color32::from_rgb(92, 94, 98), Color32::from_rgb(22, 23, 25));
    p.circle_stroke(c, r + 5.5, Stroke::new(1.0, Color32::from_black_alpha(200)));
    gradient_disc(p, c, r, Color32::from_rgb(28, 28, 30), FACE);
    // inner shadow under the bezel lip
    for (k, a) in [(0.0, 80u8), (1.5, 40), (3.0, 18)] {
        p.circle_stroke(c, r - k, Stroke::new(1.6, Color32::from_black_alpha(a)));
    }
}

/// Glass: a soft highlight crescent at the top-left and a faint rim.
fn glass(p: &egui::Painter, c: Pos2, r: f32) {
    let n = 48;
    let mut m = epaint::Mesh::default();
    let (a0, a1) = (-2.25f32, 0.55f32);
    for i in 0..=n {
        let t = i as f32 / n as f32;
        let a = a0 + (a1 - a0) * t;
        let fade = (t * PI).sin();
        let outer = polar(c, r * 0.96, a);
        let inner = polar(c + Vec2::new(r * 0.10, r * 0.13), r * 0.86, a);
        m.colored_vertex(outer, Color32::from_white_alpha((20.0 * fade) as u8));
        m.colored_vertex(inner, Color32::TRANSPARENT);
        if i > 0 {
            let k = i as u32 * 2;
            m.add_triangle(k - 2, k - 1, k);
            m.add_triangle(k - 1, k + 1, k);
        }
    }
    p.add(Shape::mesh(m));
    p.circle_stroke(c, r - 0.5, Stroke::new(1.0, Color32::from_white_alpha(10)));
}

/// Tapered needle with a counterweight tail, drop shadow and hub.
fn needle(p: &egui::Painter, c: Pos2, len: f32, a: f32, w: f32, col: Color32) {
    let f = Vec2::new(a.sin(), -a.cos());
    let n = Vec2::new(-f.y, f.x);
    let shape = |o: Vec2, col: Color32| {
        let base = c + o;
        vec![
            Shape::convex_polygon(vec![base - f * len * 0.22 + n * w * 0.75, base + f * len * 0.72 + n * w * 0.5, base + f * len, base + f * len * 0.72 - n * w * 0.5, base - f * len * 0.22 - n * w * 0.75], col, Stroke::NONE),
        ]
    };
    for s in shape(Vec2::new(1.5, 2.5), Color32::from_black_alpha(110)) {
        p.add(s);
    }
    for s in shape(Vec2::ZERO, col) {
        p.add(s);
    }
    p.circle_filled(c, w * 1.25, Color32::from_rgb(30, 30, 32));
    p.circle_filled(c - Vec2::new(w * 0.3, w * 0.3), w * 0.5, Color32::from_white_alpha(30));
}

/// Short broad pointer (altimeter thousands).
fn broad_needle(p: &egui::Painter, c: Pos2, len: f32, a: f32, w: f32, col: Color32) {
    let f = Vec2::new(a.sin(), -a.cos());
    let n = Vec2::new(-f.y, f.x);
    let pts = |o: Vec2| vec![c + o - f * len * 0.12 + n * w * 0.5, c + o + f * len * 0.78 + n * w, c + o + f * len, c + o + f * len * 0.78 - n * w, c + o - f * len * 0.12 - n * w * 0.5];
    p.add(Shape::convex_polygon(pts(Vec2::new(1.5, 2.5)), Color32::from_black_alpha(110), Stroke::NONE));
    p.add(Shape::convex_polygon(pts(Vec2::ZERO), col, Stroke::NONE));
}

fn ticks(p: &egui::Painter, c: Pos2, r: f32, a: impl Fn(f32) -> f32, from: f32, to: f32, step: f32, len: f32, w: f32, col: Color32) {
    let mut v = from;
    while v <= to + step * 0.01 {
        let an = a(v);
        p.line_segment([polar(c, r, an), polar(c, r - len, an)], Stroke::new(w, col));
        v += step;
    }
}

fn label(p: &egui::Painter, at: Pos2, s: &str, size: f32, col: Color32) {
    p.text(at, Align2::CENTER_CENTER, s, f_inst(size.max(8.5)), col);
}

fn label_b(p: &egui::Painter, at: Pos2, s: &str, size: f32, col: Color32) {
    p.text(at, Align2::CENTER_CENTER, s, f_inst_b(size), col);
}

/// Screw heads in the panel around a dial.
fn screws(p: &egui::Painter, c: Pos2, r: f32) {
    for k in 0..4 {
        let a = PI / 4.0 + k as f32 * PI / 2.0;
        let q = polar(c, r + 13.0, a);
        p.circle_filled(q, 3.2, Color32::from_rgb(40, 41, 44));
        p.circle_stroke(q, 3.2, Stroke::new(0.8, Color32::from_black_alpha(160)));
        let d = Vec2::new((a * 1.7).cos(), (a * 1.7).sin()) * 2.4;
        p.line_segment([q - d, q + d], Stroke::new(1.0, Color32::from_black_alpha(200)));
    }
}

// ---------------------------------------------------------------- instruments

/// Airspeed indicator. `unit`: 0 knots, 1 mph, 2 km/h.
fn asi(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, unit: u8, ink: Color32) {
    case(p, c, r);
    let def = sim.def();
    let k = match unit {
        1 => 1.150_78,
        2 => 1.852,
        _ => 1.0,
    };
    let to_u = |kt: f32| kt * k;
    let max = match unit {
        1 => ((def.vne_kt * k * 1.12) / 50.0).ceil() * 50.0,
        2 => ((def.vne_kt * k * 1.12) / 20.0).ceil() * 20.0,
        _ => ((def.vne_kt * 1.12) / 20.0).ceil() * 20.0,
    };
    // Low speeds compressed into the first 30°, then linear.
    let lo = (def.vs0_kt * k * 0.75 / 10.0).floor() * 10.0;
    let ang = move |v: f32| {
        let v = v.clamp(0.0, max);
        if v < lo { v / lo.max(1.0) * 0.5 } else { 0.5 + (v - lo) / (max - lo) * (TAU - 0.95) }
    };
    // Colour arcs (white flap range, green normal, yellow caution, red line).
    let white_top = (def.vs1_kt * 1.75).min(def.vno_kt);
    if def.has_engine && def.category != Category::Glider {
        ring_sector(p, c, r * 0.80, r * 0.86, ang(to_u(def.vs0_kt)), ang(to_u(white_top)), ARC_WHITE);
    }
    ring_sector(p, c, r * 0.86, r * 0.93, ang(to_u(def.vs1_kt)), ang(to_u(def.vno_kt)), ARC_GREEN);
    ring_sector(p, c, r * 0.86, r * 0.93, ang(to_u(def.vno_kt)), ang(to_u(def.vne_kt)), ARC_YELLOW);
    let red = ang(to_u(def.vne_kt));
    p.line_segment([polar(c, r * 0.97, red), polar(c, r * 0.78, red)], Stroke::new(r * 0.05, ARC_RED));
    let (major, minor) = match unit {
        1 => (50.0, 10.0),
        2 => (40.0, 10.0),
        _ => (if max > 250.0 { 50.0 } else { 20.0 }, if max > 250.0 { 10.0 } else { 5.0 }),
    };
    ticks(p, c, r * 0.97, ang, lo, max, minor, r * 0.07, 1.2, ink);
    ticks(p, c, r * 0.97, ang, lo, max, major, r * 0.14, 2.2, ink);
    let mut v = (lo / major).ceil() * major;
    while v <= max - major * 0.5 {
        label_b(p, polar(c, r * 0.64, ang(v)), &format!("{v:.0}"), r * 0.17, ink);
        v += major;
    }
    let title = match unit {
        1 => "MILES PER HOUR",
        2 => "km/h",
        _ => "KNOTS",
    };
    label(p, c + Vec2::new(0.0, -r * 0.24), "AIRSPEED", r * 0.10, INK_DIM);
    label(p, c + Vec2::new(0.0, r * 0.30), title, r * 0.10, INK_DIM);
    if sim.ac.mach() > 0.45 {
        label(p, c + Vec2::new(0.0, r * 0.46), &format!("M {:.2}", sim.ac.mach()), r * 0.12, pal::AMBER);
    }
    // Indicated (pitot-static system, systems/).
    let ias = to_u(sim.ac.sys.air_data(&sim.ac).ias_kt());
    if let Some(t) = sim.ap.ias.filter(|_| sim.ap.engaged) {
        let a = ang(to_u(t * 1.943_844));
        p.add(Shape::convex_polygon(vec![polar(c, r * 1.0, a - 0.05), polar(c, r * 1.0, a + 0.05), polar(c, r * 0.88, a)], pal::CYAN, Stroke::NONE));
    }
    needle(p, c, r * 0.9, ang(ias), r * 0.07, ink);
    glass(p, c, r);
}

/// Attitude indicator.
fn attitude(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, vintage: bool) {
    case(p, c, r);
    // Vacuum gyro (spins down / tumbles: systems/).
    let ad = sim.ac.sys.air_data(&sim.ac);
    let pitch = ad.pitch.to_degrees();
    let roll = ad.roll;
    let k = r / 32.0; // points per degree
    let up = Vec2::new(-roll.sin(), -roll.cos());
    let along = Vec2::new(roll.cos(), -roll.sin());
    let h0 = c - up * (pitch.clamp(-35.0, 35.0) * k);
    let (sky_t, sky_b, gnd_t, gnd_b) = if vintage {
        (Color32::from_rgb(20, 22, 26), Color32::from_rgb(52, 56, 62), Color32::from_rgb(28, 28, 30), Color32::from_rgb(14, 14, 15))
    } else {
        (Color32::from_rgb(28, 98, 182), Color32::from_rgb(92, 160, 222), Color32::from_rgb(140, 92, 48), Color32::from_rgb(92, 58, 28))
    };
    let ri = r * 0.995;
    // Sky / ground split by the horizon, built as a fan of the inner disc.
    let n = 120;
    let mut sky = epaint::Mesh::default();
    let mut gnd = epaint::Mesh::default();
    let below = |q: Pos2| (q - h0).dot(up) < 0.0;
    let sky_col = |q: Pos2| lerp_col(sky_b, sky_t, ((q - h0).dot(up) / (r * 1.6)).clamp(0.0, 1.0));
    let gnd_col = |q: Pos2| lerp_col(gnd_t, gnd_b, ((h0 - q).dot(up) / (r * 1.6)).clamp(0.0, 1.0));
    sky.colored_vertex(c, sky_col(c));
    gnd.colored_vertex(c, gnd_col(c));
    // simple per-vertex classification with the horizon intersection points
    let mut prev: Option<Pos2> = None;
    let mut sky_ring: Vec<Pos2> = Vec::new();
    let mut gnd_ring: Vec<Pos2> = Vec::new();
    for i in 0..=n {
        let q = polar(c, ri, i as f32 / n as f32 * TAU);
        if let Some(pq) = prev {
            if below(pq) != below(q) {
                let (da, db) = ((pq - h0).dot(up), (q - h0).dot(up));
                let x = pq + (q - pq) * (da / (da - db));
                sky_ring.push(x);
                gnd_ring.push(x);
            }
        }
        if below(q) { gnd_ring.push(q) } else { sky_ring.push(q) }
        prev = Some(q);
    }
    // Each region: polygon = its ring points + the horizon chord (convex enough: disc ∩ half-plane).
    for (ring, mesh, col) in [(&sky_ring, &mut sky, &sky_col as &dyn Fn(Pos2) -> Color32), (&gnd_ring, &mut gnd, &gnd_col)] {
        if ring.len() < 3 {
            continue;
        }
        let cen = ring.iter().fold(Vec2::ZERO, |a, q| a + q.to_vec2()) / ring.len() as f32;
        mesh.vertices.clear();
        mesh.colored_vertex(cen.to_pos2(), col(cen.to_pos2()));
        for q in ring.iter() {
            mesh.colored_vertex(*q, col(*q));
        }
        let m = ring.len() as u32;
        for i in 0..m {
            mesh.add_triangle(0, 1 + i, 1 + (i + 1) % m);
        }
    }
    p.add(Shape::mesh(sky));
    p.add(Shape::mesh(gnd));
    let line_col = if vintage { RADIUM } else { Color32::WHITE };
    let inside = |q: Pos2| (q - c).length() < r * 0.8;
    // horizon
    let hl = r * 1.2;
    let (a, b) = (h0 - along * hl, h0 + along * hl);
    let clip_seg = |a: Pos2, b: Pos2, rr: f32| -> Option<[Pos2; 2]> {
        // clip a segment to the circle of radius rr
        let d = b - a;
        let f = a - c;
        let (aa, bb, cc) = (d.dot(d), 2.0 * f.dot(d), f.dot(f) - rr * rr);
        let disc = bb * bb - 4.0 * aa * cc;
        if disc <= 0.0 {
            return None;
        }
        let s = disc.sqrt();
        let (t0, t1) = (((-bb - s) / (2.0 * aa)).max(0.0), ((-bb + s) / (2.0 * aa)).min(1.0));
        (t1 > t0).then(|| [a + d * t0, a + d * t1])
    };
    if let Some(s) = clip_seg(a, b, ri) {
        p.line_segment(s, Stroke::new(2.0, line_col));
    }
    for i in [-4, -3, -2, -1, 1, 2, 3, 4] {
        let deg = i as f32 * 5.0;
        let cen = h0 + up * (deg * k * 1.0);
        let half = if i % 2 == 0 { r * 0.26 } else { r * 0.12 };
        if let Some(s) = clip_seg(cen - along * half, cen + along * half, r * 0.8) {
            p.line_segment(s, Stroke::new(1.4, line_col));
        }
        if i % 2 == 0 && inside(cen) {
            for sgn in [-1.0, 1.0] {
                let t = cen + along * sgn * (half + r * 0.09);
                if inside(t) {
                    label(p, t, &format!("{:.0}", deg.abs()), r * 0.10, line_col);
                }
            }
        }
    }
    // bank scale (fixed) with the roll pointer on the moving card
    for d in [-60.0f32, -45.0, -30.0, -20.0, -10.0, 10.0, 20.0, 30.0, 45.0, 60.0] {
        let an = d.to_radians();
        let len = if d.abs() == 30.0 || d.abs() == 60.0 { r * 0.15 } else { r * 0.08 };
        p.line_segment([polar(c, r * 0.97, an), polar(c, r * 0.97 - len, an)], Stroke::new(2.0, INK));
    }
    p.add(Shape::convex_polygon(vec![polar(c, r * 0.97, 0.0), polar(c, r * 0.84, -0.07), polar(c, r * 0.84, 0.07)], INK, Stroke::NONE));
    // moving pointer
    let pa = -roll;
    p.add(Shape::convex_polygon(vec![polar(c, r * 0.83, pa), polar(c, r * 0.72, pa - 0.08), polar(c, r * 0.72, pa + 0.08)], pal::AMBER, Stroke::NONE));
    // fixed aircraft symbol
    let o = if vintage { RADIUM } else { Color32::from_rgb(255, 170, 30) };
    let w = r * 0.52;
    let sh = Vec2::new(1.0, 2.0);
    for (col, off) in [(Color32::from_black_alpha(140), sh), (o, Vec2::ZERO)] {
        p.line_segment([c + off + Vec2::new(-w, 0.0), c + off + Vec2::new(-w * 0.32, 0.0)], Stroke::new(r * 0.055, col));
        p.line_segment([c + off + Vec2::new(w * 0.32, 0.0), c + off + Vec2::new(w, 0.0)], Stroke::new(r * 0.055, col));
        p.add(Shape::line(vec![c + off + Vec2::new(-w * 0.32, 0.0), c + off + Vec2::new(0.0, r * 0.12), c + off + Vec2::new(w * 0.32, 0.0)], Stroke::new(r * 0.05, col)));
        p.circle_filled(c + off, r * 0.04, col);
    }
    glass(p, c, r);
}

/// Altimeter (three pointers, pressure window). `metres` for gliders.
fn altimeter(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, qnh_hpa: f32, metres: bool, ink: Color32) {
    case(p, c, r);
    let alt_m = sim.ac.sys.air_data(&sim.ac).alt;
    let alt = if metres { alt_m } else { alt_m * 3.280_84 };
    let full = if metres { 1000.0 } else { 1000.0 };
    let a = |v: f32| v / 10.0 * TAU;
    ticks(p, c, r * 0.97, |v| v / 50.0 * TAU, 0.0, 49.0, 1.0, r * 0.07, 1.1, ink);
    for i in 0..10 {
        let an = a(i as f32);
        p.line_segment([polar(c, r * 0.97, an), polar(c, r * 0.80, an)], Stroke::new(2.4, ink));
        label_b(p, polar(c, r * 0.66, an), &format!("{i}"), r * 0.2, ink);
    }
    // pressure window
    let win = Rect::from_center_size(c + Vec2::new(r * 0.46, 0.0), Vec2::new(r * 0.42, r * 0.2));
    p.rect_filled(win, 2.0, Color32::from_rgb(8, 8, 9));
    p.rect_stroke(win, 2.0, Stroke::new(1.0, Color32::from_gray(70)), egui::StrokeKind::Outside);
    p.text(win.center(), Align2::CENTER_CENTER, format!("{qnh_hpa:.0}"), f_inst(r * 0.12), ink);
    label(p, c + Vec2::new(0.0, -r * 0.30), "ALT", r * 0.11, INK_DIM);
    label(p, c + Vec2::new(0.0, r * 0.34), if metres { "METRES" } else { "FEET" }, r * 0.09, INK_DIM);
    // 10 000 pointer (thin with a triangle), 1 000 (broad), 100 (long)
    let t10 = alt / (full * 100.0) * TAU;
    let tip = polar(c, r * 0.94, t10);
    p.line_segment([c, polar(c, r * 0.86, t10)], Stroke::new(1.4, ink));
    p.add(Shape::convex_polygon(vec![tip, polar(c, r * 0.80, t10 - 0.07), polar(c, r * 0.80, t10 + 0.07)], ink, Stroke::NONE));
    broad_needle(p, c, r * 0.55, (alt / (full * 10.0)).rem_euclid(1.0) * TAU, r * 0.085, ink);
    needle(p, c, r * 0.9, (alt / full).rem_euclid(1.0) * TAU, r * 0.06, ink);
    glass(p, c, r);
}

/// Vertical speed (fpm ×1000) or glider variometer (m/s ±5) with a MacCready ring.
fn vsi(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, glider: Option<&Polar>, ink: Color32) {
    case(p, c, r);
    let (max, val) = match glider {
        Some(_) => (5.0, sim.ac.sys.air_data(&sim.ac).vs),
        None => (2.0, sim.ac.sys.air_data(&sim.ac).vs * 3.280_84 * 60.0 / 1000.0),
    };
    let span = 170f32.to_radians();
    let ang = |v: f32| -PI / 2.0 + (v.clamp(-max, max) / max) * span;
    let minor = if glider.is_some() { 0.5 } else { 0.1 };
    ticks(p, c, r * 0.97, ang, -max, max, minor, r * 0.07, 1.2, ink);
    ticks(p, c, r * 0.97, ang, -max, max, if glider.is_some() { 1.0 } else { 0.5 }, r * 0.14, 2.2, ink);
    let step = if glider.is_some() { 1.0 } else { 0.5 };
    let mut v = -max;
    while v <= max + 0.01 {
        if v.abs() > 0.01 && (v.abs() - max).abs() > 0.01 && (glider.is_some() || (v * 2.0).fract().abs() < 0.01 && v.fract().abs() < 0.01) {
            label_b(p, polar(c, r * 0.66, ang(v)), &format!("{:.0}", v.abs()), r * 0.17, ink);
        }
        v += step;
    }
    label_b(p, polar(c, r * 0.66, ang(0.0)), "0", r * 0.17, ink);
    if let Some(pol) = glider {
        label(p, c + Vec2::new(r * 0.18, -r * 0.30), "UP", r * 0.09, INK_DIM);
        label(p, c + Vec2::new(r * 0.18, r * 0.30), "DOWN", r * 0.09, INK_DIM);
        label(p, c + Vec2::new(r * 0.30, -r * 0.02), "m/s", r * 0.10, INK_DIM);
        // MacCready speed-to-fly ring: speed V sits at the vario reading m − V·w′(V).
        let mc = 1.0f32;
        let ring_r = r * 1.0;
        let mut v = 70.0f32;
        while v <= 200.0 {
            let vm = v / 3.6;
            let reading = mc - vm * pol.dsink(vm);
            if reading > -max && reading < max {
                let an = ang(reading);
                p.line_segment([polar(c, ring_r + 1.0, an), polar(c, ring_r + 5.0, an)], Stroke::new(1.5, pal::AMBER));
                if (v as i32) % 20 == 0 {
                    p.text(polar(c, ring_r + 11.0, an), Align2::CENTER_CENTER, format!("{v:.0}"), f_inst(r * 0.10), pal::AMBER);
                }
            }
            v += 10.0;
        }
        p.text(c + Vec2::new(-r * 0.36, r * 0.52), Align2::CENTER_CENTER, format!("MC {mc:.1}"), f_inst(r * 0.10), pal::AMBER);
        let col = if val > 0.2 { pal::GREEN } else { INK };
        p.text(c + Vec2::new(-r * 0.34, -r * 0.02), Align2::CENTER_CENTER, format!("{val:+.1}"), f_inst_b(r * 0.14), col);
    } else {
        label(p, c + Vec2::new(r * 0.24, -r * 0.24), "UP", r * 0.09, INK_DIM);
        label(p, c + Vec2::new(r * 0.24, r * 0.24), "DOWN", r * 0.09, INK_DIM);
        label(p, c + Vec2::new(r * 0.30, 0.0), "FPM ×1000", r * 0.08, INK_DIM);
    }
    needle(p, c, r * 0.9, ang(val), r * 0.07, ink);
    glass(p, c, r);
}

/// Turn coordinator: banking aircraft for the turn rate (standard-rate marks) and a slip ball.
fn turn_coordinator(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, ink: Color32) {
    case(p, c, r);
    let ad = sim.ac.sys.air_data(&sim.ac);
    let rate = ad.turn.to_degrees(); // deg/s, + right (electric gyro: systems/)
    let std = 3.0;
    let bank = (rate / std * 20f32.to_radians()).clamp(-0.6, 0.6);
    for s in [-1.0f32, 1.0] {
        let a = PI / 2.0 * s + s * 0.0;
        let _ = a;
        let an = s * (PI / 2.0 + 20f32.to_radians());
        p.line_segment([polar(c, r * 0.95, an), polar(c, r * 0.80, an)], Stroke::new(2.6, ink));
        let an0 = s * PI / 2.0;
        p.line_segment([polar(c, r * 0.95, an0), polar(c, r * 0.82, an0)], Stroke::new(2.6, ink));
        label_b(p, polar(c, r * 0.68, s * (PI / 2.0 + 0.36)), if s < 0.0 { "L" } else { "R" }, r * 0.14, ink);
    }
    label(p, c + Vec2::new(0.0, -r * 0.52), "TURN", r * 0.10, INK_DIM);
    label(p, c + Vec2::new(0.0, r * 0.36), "2 MIN", r * 0.08, INK_DIM);
    // aircraft (rear view)
    let along = Vec2::new(bank.cos(), bank.sin());
    let upv = Vec2::new(-bank.sin(), bank.cos()) * -1.0;
    let cc = c + Vec2::new(0.0, -r * 0.05);
    p.line_segment([cc - along * r * 0.62, cc + along * r * 0.62], Stroke::new(r * 0.06, ink));
    p.circle_filled(cc, r * 0.1, ink);
    p.line_segment([cc, cc + upv * r * 0.2], Stroke::new(r * 0.05, ink));
    // inclinometer
    let tube = Rect::from_center_size(c + Vec2::new(0.0, r * 0.58), Vec2::new(r * 0.95, r * 0.2));
    p.rect_filled(tube, r * 0.1, Color32::from_rgb(196, 200, 188));
    p.line_segment([Pos2::new(tube.center().x - r * 0.11, tube.min.y), Pos2::new(tube.center().x - r * 0.11, tube.max.y)], Stroke::new(1.5, Color32::BLACK));
    p.line_segment([Pos2::new(tube.center().x + r * 0.11, tube.min.y), Pos2::new(tube.center().x + r * 0.11, tube.max.y)], Stroke::new(1.5, Color32::BLACK));
    let slip = (-sim.ac.slip() * 3.0).clamp(-1.0, 1.0);
    p.circle_filled(Pos2::new(tube.center().x + slip * r * 0.38, tube.center().y), r * 0.085, Color32::BLACK);
    if ad.has(crate::systems::flag::TURN) {
        // Power-off flag.
        let f = Rect::from_center_size(c + Vec2::new(-r * 0.42, r * 0.18), Vec2::new(r * 0.32, r * 0.16));
        p.rect_filled(f, 1.0, ARC_RED);
        label_b(p, f.center(), "OFF", r * 0.11, Color32::WHITE);
    }
    glass(p, c, r);
}

/// Heading indicator (directional gyro), with the AP heading bug.
fn heading(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, ink: Color32) {
    case(p, c, r);
    let hdg = sim.ac.sys.air_data(&sim.ac).heading;
    for i in 0..72 {
        let deg = i as f32 * 5.0;
        let a = deg.to_radians() - hdg;
        let major = i % 2 == 0;
        p.line_segment([polar(c, r * 0.97, a), polar(c, r * if major { 0.85 } else { 0.91 }, a)], Stroke::new(if major { 2.0 } else { 1.2 }, ink));
        if i % 6 == 0 {
            let txt = match i / 6 {
                0 => "N".to_string(),
                3 => "E".to_string(),
                6 => "S".to_string(),
                9 => "W".to_string(),
                k => format!("{}", k * 3),
            };
            label_b(p, polar(c, r * 0.68, a), &txt, r * if i % 18 == 0 { 0.2 } else { 0.16 }, ink);
        }
    }
    if let Some(h) = sim.ap.hdg.filter(|_| sim.ap.engaged) {
        let a = h - hdg;
        p.add(Shape::convex_polygon(vec![polar(c, r * 0.99, a - 0.08), polar(c, r * 0.99, a + 0.08), polar(c, r * 0.88, a + 0.04), polar(c, r * 0.88, a - 0.04)], pal::CYAN, Stroke::NONE));
    }
    // lubber + aircraft
    p.add(Shape::convex_polygon(vec![polar(c, r * 0.99, 0.0) + Vec2::new(-6.0, -1.0), polar(c, r * 0.99, 0.0) + Vec2::new(6.0, -1.0), polar(c, r * 0.86, 0.0)], pal::AMBER, Stroke::NONE));
    let o = pal::AMBER;
    p.line_segment([c + Vec2::new(0.0, -r * 0.34), c + Vec2::new(0.0, r * 0.30)], Stroke::new(r * 0.045, o));
    p.line_segment([c + Vec2::new(-r * 0.30, -r * 0.04), c + Vec2::new(r * 0.30, -r * 0.04)], Stroke::new(r * 0.045, o));
    p.line_segment([c + Vec2::new(-r * 0.12, r * 0.24), c + Vec2::new(r * 0.12, r * 0.24)], Stroke::new(r * 0.045, o));
    glass(p, c, r);
}

/// Tachometer: RPM ×100 with green arc; for supercharged engines `boost` draws a boost gauge instead.
fn tach(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, ink: Color32) {
    case(p, c, r);
    let def = sim.def();
    let rs = def.rpm_static.max(1000.0);
    let max = ((rs * 1.3) / 500.0).ceil() * 500.0;
    let ang = |v: f32| -0.75 * PI + (v.clamp(0.0, max) / max) * 1.5 * PI;
    ring_sector(p, c, r * 0.86, r * 0.93, ang(rs * 0.82), ang(rs * 1.10), ARC_GREEN);
    let red = ang(rs * 1.14);
    p.line_segment([polar(c, r * 0.97, red), polar(c, r * 0.78, red)], Stroke::new(r * 0.05, ARC_RED));
    ticks(p, c, r * 0.97, ang, 0.0, max, 100.0, r * 0.07, 1.1, ink);
    ticks(p, c, r * 0.97, ang, 0.0, max, 500.0, r * 0.14, 2.2, ink);
    let mut v = 0.0;
    while v <= max + 1.0 {
        label_b(p, polar(c, r * 0.66, ang(v)), &format!("{:.0}", v / 100.0), r * 0.15, ink);
        v += 500.0;
    }
    label(p, c + Vec2::new(0.0, -r * 0.30), "RPM", r * 0.11, INK_DIM);
    label(p, c + Vec2::new(0.0, r * 0.30), "×100", r * 0.09, INK_DIM);
    let rpm = sim.ac.engine_rpm();
    p.text(c + Vec2::new(0.0, r * 0.52), Align2::CENTER_CENTER, format!("{rpm:4.0}"), f_inst(r * 0.12), if sim.ac.engine_running(0) { INK_DIM } else { ARC_RED });
    needle(p, c, r * 0.9, ang(rpm), r * 0.07, ink);
    glass(p, c, r);
}

/// Spitfire boost gauge (lb/sq in).
fn boost(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    case(p, c, r);
    let ink = RADIUM;
    let (lo, hi) = (-6.0f32, 24.0f32);
    let ang = |v: f32| -0.75 * PI + ((v - lo) / (hi - lo)).clamp(0.0, 1.0) * 1.5 * PI;
    ticks(p, c, r * 0.97, ang, lo, hi, 1.0, r * 0.07, 1.1, ink);
    ticks(p, c, r * 0.97, ang, lo, hi, 2.0, r * 0.14, 2.2, ink);
    for v in [-6.0, -4.0, -2.0, 0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0] {
        label_b(p, polar(c, r * 0.66, ang(v)), &format!("{:+.0}", v).replace("+0", "0"), r * 0.13, ink);
    }
    ring_sector(p, c, r * 0.86, r * 0.93, ang(18.0), ang(hi), ARC_RED);
    label(p, c + Vec2::new(0.0, -r * 0.28), "BOOST", r * 0.11, INK_DIM);
    label(p, c + Vec2::new(0.0, r * 0.30), "LBS. PER SQ.IN.", r * 0.075, INK_DIM);
    let pw = sim.ac.n1(0).clamp(0.0, 1.0);
    let alt_loss = (sim.ac.pos.y / 9000.0).clamp(0.0, 1.0) * 6.0;
    let b = if sim.ac.engine_running(0) { -4.0 + 22.0 * pw - alt_loss * (1.0 - pw * 0.3) } else { -5.0 };
    needle(p, c, r * 0.88, ang(b), r * 0.07, ink);
    glass(p, c, r);
}

/// Simple glider polar w(v) = k1 v³ + k2 / v (sink m/s, v m/s).
pub struct Polar {
    k1: f32,
    k2: f32,
}

impl Polar {
    pub fn from_def(def: &AircraftDef) -> Polar {
        let v_ms = def.vs1_kt * 0.5144 * 1.12;
        let ld = 34.0;
        let v_ld = 3f32.powf(0.25) * v_ms;
        let k1 = v_ld / (ld * (v_ld.powi(3) + 3.0 * v_ms.powi(4) / v_ld));
        Polar { k1, k2: 3.0 * k1 * v_ms.powi(4) }
    }
    #[allow(dead_code)]
    pub fn sink(&self, v: f32) -> f32 {
        self.k1 * v.powi(3) + self.k2 / v.max(1.0)
    }
    pub fn dsink(&self, v: f32) -> f32 {
        3.0 * self.k1 * v * v - self.k2 / (v * v).max(1.0)
    }
}

/// Compass-style heading drum for the glider (a slice of the card).
fn compass(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    case(p, c, r);
    let hdg = sim.ac.heading().to_degrees();
    let win = Rect::from_center_size(c, Vec2::new(r * 1.5, r * 0.62));
    p.rect_filled(win, 4.0, Color32::from_rgb(10, 10, 11));
    let pc = p.with_clip_rect(win);
    let ppd = r * 1.5 / 60.0;
    let base = (hdg / 5.0).floor() as i32 * 5;
    for k in -8..=8 {
        let d = base + k * 5;
        let x = c.x + (d as f32 - hdg) * ppd;
        let major = d.rem_euclid(10) == 0;
        pc.line_segment([Pos2::new(x, win.max.y), Pos2::new(x, win.max.y - if major { r * 0.16 } else { r * 0.09 })], Stroke::new(1.4, INK));
        if d.rem_euclid(30) == 0 {
            let dd = d.rem_euclid(360);
            let t = match dd {
                0 => "N".to_string(),
                90 => "E".to_string(),
                180 => "S".to_string(),
                270 => "W".to_string(),
                v => format!("{}", v / 10),
            };
            pc.text(Pos2::new(x, win.min.y + r * 0.2), Align2::CENTER_CENTER, t, f_inst_b(r * 0.2), INK);
        }
    }
    p.line_segment([Pos2::new(c.x, win.min.y - 2.0), Pos2::new(c.x, win.max.y + 2.0)], Stroke::new(2.0, pal::AMBER));
    label(p, c + Vec2::new(0.0, -r * 0.55), "COMPASS", r * 0.09, INK_DIM);
    p.text(c + Vec2::new(0.0, r * 0.55), Align2::CENTER_CENTER, format!("{:03.0}°", hdg.rem_euclid(360.0)), f_inst_b(r * 0.14), INK);
    glass(p, c, r);
}

/// Glider g-meter / yaw string stand-in: g with max/min markers.
fn gmeter(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    case(p, c, r);
    let ang = |g: f32| -PI / 2.0 + ((g + 2.0) / 8.0).clamp(0.0, 1.0) * 1.6 * PI - 0.0;
    ticks(p, c, r * 0.97, ang, -2.0, 6.0, 0.5, r * 0.07, 1.1, INK);
    ticks(p, c, r * 0.97, ang, -2.0, 6.0, 1.0, r * 0.14, 2.2, INK);
    for g in -2..=6 {
        label_b(p, polar(c, r * 0.66, ang(g as f32)), &format!("{g}"), r * 0.16, INK);
    }
    ring_sector(p, c, r * 0.86, r * 0.93, ang(5.3), ang(6.0), ARC_RED);
    label(p, c + Vec2::new(0.0, r * 0.32), "G", r * 0.14, INK_DIM);
    needle(p, c, r * 0.88, ang(sim.ac.g_load()), r * 0.07, INK);
    glass(p, c, r);
}

/// The analogue instrument panel. Returns the panel rect.
pub fn draw_panel(p: &egui::Painter, screen: Rect, sim: &Sim, qnh_hpa: f32, compact: bool) -> Rect {
    let def = sim.def();
    let kind = panel_for(def);
    let r = if compact { 48.0 } else { 64.0 };
    let gap = r * 2.0 + 32.0;
    let n = match kind {
        Panel::Trainer => 7,
        Panel::Warbird => 8,
        Panel::Glider => 5,
    };
    let w = gap * n as f32 + 24.0;
    let h = r * 2.0 + 34.0;
    let rect = Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - h * 0.5 - 12.0), Vec2::new(w, h));
    // Panel: dark painted metal with a gentle top light.
    let cr = egui::CornerRadius::same(16);
    p.add(epaint::Shadow { offset: [0, 6], blur: 24, spread: 0, color: Color32::from_black_alpha(110) }.as_shape(rect, cr));
    let (top, bot) = match kind {
        Panel::Warbird => (Color32::from_rgb(34, 36, 32), Color32::from_rgb(20, 21, 19)),
        _ => (Color32::from_rgb(38, 40, 44), Color32::from_rgb(22, 23, 26)),
    };
    p.rect_filled(rect, cr, bot);
    let inner = rect.shrink(1.0);
    p.add(Shape::Rect(epaint::RectShape::filled(Rect::from_min_max(inner.min, Pos2::new(inner.max.x, inner.min.y + h * 0.5)), egui::CornerRadius { nw: 15, ne: 15, sw: 0, se: 0 }, top)));
    p.rect_stroke(rect, cr, Stroke::new(1.0, Color32::from_white_alpha(18)), egui::StrokeKind::Inside);
    let x0 = rect.min.x + 12.0 + gap * 0.5;
    let cy = rect.center().y;
    let at = |i: usize| Pos2::new(x0 + gap * i as f32, cy);
    match kind {
        Panel::Trainer => {
            asi(p, at(0), r, sim, 0, INK);
            attitude(p, at(1), r, sim, false);
            altimeter(p, at(2), r, sim, qnh_hpa, false, INK);
            turn_coordinator(p, at(3), r, sim, INK);
            heading(p, at(4), r, sim, INK);
            vsi(p, at(5), r, sim, None, INK);
            tach(p, at(6), r, sim, INK);
        }
        Panel::Warbird => {
            asi(p, at(0), r, sim, 1, RADIUM);
            attitude(p, at(1), r, sim, true);
            vsi(p, at(2), r, sim, None, RADIUM);
            altimeter(p, at(3), r, sim, qnh_hpa, false, RADIUM);
            heading(p, at(4), r, sim, RADIUM);
            turn_coordinator(p, at(5), r, sim, RADIUM);
            boost(p, at(6), r, sim);
            tach(p, at(7), r, sim, RADIUM);
        }
        Panel::Glider => {
            let pol = Polar::from_def(def);
            asi(p, at(0), r, sim, 2, INK);
            vsi(p, at(1), r, sim, Some(&pol), INK);
            altimeter(p, at(2), r, sim, qnh_hpa, true, INK);
            compass(p, at(3), r, sim);
            gmeter(p, at(4), r, sim);
        }
    }
    for i in 0..n {
        screws(p, at(i), r);
    }
    let _ = arc;
    rect
}
