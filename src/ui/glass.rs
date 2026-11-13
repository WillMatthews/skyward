//! Airliner glass cockpit: primary flight display (speed / altitude / heading
//! tapes, attitude with flight-director bars, FMA), navigation display (map
//! mode, track-up, range arcs, wind, auto-flight route, airports) and an
//! engine / warning display (N1 or torque, EGT, flaps, gear, memo).

use super::canvas;
use super::map::{self, Style, Tiles, View};
use super::{gradient_rect, pal, polar, wrap_pi};
use crate::flight::{EngineKind, KT};
use crate::sim::Sim;
use crate::world::World;
use egui::{epaint, Align2, Color32, CornerRadius, Pos2, Rect, Shape, Stroke, Vec2};
use glam::vec2;
use std::f32::consts::PI;

const SCREEN_BG: Color32 = Color32::from_rgb(6, 8, 11);
const TAPE_BG: Color32 = Color32::from_rgb(52, 58, 68);
const GREEN: Color32 = pal::GREEN;
const AMBER: Color32 = pal::AMBER;
const CYAN: Color32 = pal::CYAN;
const MAG: Color32 = pal::MAGENTA;
const WHITE: Color32 = Color32::from_rgb(240, 242, 245);
const YELLOW: Color32 = Color32::from_rgb(255, 222, 40);
const FT: f32 = 3.280_84;
/// Display text scale (sizes below are in layout units of 1 % of the screen width).
const TS: f32 = 1.28;
fn fi(s: f32) -> egui::FontId {
    super::f_inst(s * TS)
}
fn fib(s: f32) -> egui::FontId {
    super::f_inst_b(s * TS)
}
fn fd(s: f32) -> egui::FontId {
    super::f_digits(s * TS)
}
fn fdb(s: f32) -> egui::FontId {
    super::f_digits_b(s * TS)
}

/// Display unit housing: dark bezel with a thin highlight, black screen.
fn housing(p: &egui::Painter, r: Rect) -> Rect {
    p.add(epaint::Shadow { offset: [0, 6], blur: 20, spread: 0, color: Color32::from_black_alpha(120) }.as_shape(r, 12));
    p.rect_filled(r, 12, Color32::from_rgb(30, 32, 36));
    p.rect_stroke(r, 12, Stroke::new(1.0, Color32::from_white_alpha(22)), egui::StrokeKind::Inside);
    let s = r.shrink(7.0);
    p.rect_filled(s, 6, SCREEN_BG);
    s
}

fn txt(p: &egui::Painter, at: Pos2, align: Align2, s: impl ToString, size: f32, col: Color32) -> Rect {
    p.text(at, align, s.to_string(), fi(size), col)
}

fn digits(p: &egui::Painter, at: Pos2, align: Align2, s: impl ToString, size: f32, col: Color32) -> Rect {
    p.text(at, align, s.to_string(), fdb(size), col)
}

/// Barber-pole band between y0 and y1 (x0..x1).
fn barber(p: &egui::Painter, x0: f32, x1: f32, y0: f32, y1: f32, a: Color32, b: Color32) {
    let (ya, yb) = (y0.min(y1), y0.max(y1));
    if yb - ya < 0.5 {
        return;
    }
    p.rect_filled(Rect::from_min_max(Pos2::new(x0, ya), Pos2::new(x1, yb)), 0.0, a);
    let pc = p.with_clip_rect(Rect::from_min_max(Pos2::new(x0, ya), Pos2::new(x1, yb)));
    let w = x1 - x0;
    let mut y = ya - w;
    while y < yb + w {
        pc.add(Shape::convex_polygon(vec![Pos2::new(x0, y), Pos2::new(x1, y - w), Pos2::new(x1, y - w + w * 0.6), Pos2::new(x0, y + w * 0.6)], b, Stroke::NONE));
        y += w * 1.2;
    }
}

// ---------------------------------------------------------------- PFD

/// Flight-director commands (bank, pitch offsets in radians) from the autopilot targets.
fn flight_director(sim: &Sim) -> Option<(f32, f32)> {
    if !sim.ap.engaged {
        return None;
    }
    // The autopilot's own lateral / vertical commands (its NAV / LOC / G/S modes included).
    if sim.bot.is_none() {
        if let Some(fd) = sim.ap.fd {
            return Some(fd);
        }
    }
    let ac = &sim.ac;
    let bank_cmd = match sim.ap.hdg {
        Some(h) => (wrap_pi(h - ac.heading()) * 2.2).clamp(-25f32.to_radians(), 25f32.to_radians()),
        None => 0.0,
    };
    let vs_t = match (sim.ap.alt, sim.ap.vs) {
        (Some(a), vs) => {
            let err = a - ac.pos.y;
            let cap = (err * 0.08).clamp(-10.0, 10.0);
            vs.map(|v| v.abs() * err.signum()).filter(|_| err.abs() > 120.0).unwrap_or(cap)
        }
        (None, Some(v)) => v,
        _ => ac.vertical_speed(),
    };
    let tas = ac.tas().max(30.0);
    let dpitch = ((vs_t - ac.vertical_speed()) / tas).atan().clamp(-8f32.to_radians(), 8f32.to_radians());
    Some((bank_cmd - ac.roll(), dpitch))
}

/// Primary flight display in `r`.
pub fn pfd(p: &egui::Painter, r: Rect, sim: &Sim, world: &World) {
    pfd_unit(p, r, sim, world, false);
}

/// A dark (unpowered) screen.
fn dark(p: &egui::Painter, s: Rect) {
    p.rect_filled(s, 6, Color32::from_rgb(2, 3, 4));
}

/// The big red X of a display unit with no valid data.
pub fn red_x(p: &egui::Painter, s: Rect) {
    let st = Stroke::new((s.width() * 0.012).max(2.0), pal::RED);
    let q = s.shrink(s.width() * 0.04);
    p.line_segment([q.left_top(), q.right_bottom()], st);
    p.line_segment([q.right_top(), q.left_bottom()], st);
}

/// A red failure flag ("ATT", "SPD", "ALT", "V/S", "HDG") centred in `r`.
fn flag_box(p: &egui::Painter, c: Pos2, s: &str, u: f32) {
    let rr = p.text(c, Align2::CENTER_CENTER, s, fib(3.6 * u), pal::RED);
    p.rect_stroke(rr.expand2(Vec2::new(0.8 * u, 0.3 * u)), 0, Stroke::new(1.2, pal::RED), egui::StrokeKind::Outside);
}

/// PFD, or the standby instrument (`standby`: its own sensors, no red X / AHRS flags).
pub fn pfd_unit(p: &egui::Painter, r: Rect, sim: &Sim, world: &World, standby: bool) {
    use crate::systems::flag;
    let s = housing(p, r);
    let sys = &sim.ac.sys;
    if !sys.display_on(if standby { "isis" } else { "pfd" }) {
        dark(p, s);
        return;
    }
    if !standby && sys.pfd_red_x() {
        red_x(p, s);
        return;
    }
    let p = &p.with_clip_rect(s);
    let u = s.width() / 100.0; // layout unit
    let def = sim.def();
    // What the air-data / attitude sources indicate (systems/).
    let mut ad = sys.air_data(&sim.ac);
    if standby {
        ad.flags &= !(flag::ATT | flag::HDG);
        ad.pitch = sim.ac.pitch();
        ad.roll = sim.ac.roll();
        ad.heading = sim.ac.heading();
    }
    let ias = ad.ias / KT;
    let alt_ft = ad.alt * FT;
    // --- FMA (top band)
    let fma = Rect::from_min_size(s.min, Vec2::new(s.width(), 9.0 * u));
    fma_band(p, fma, sim, u);
    // --- attitude
    let att = Rect::from_center_size(Pos2::new(s.min.x + 48.0 * u, s.min.y + 47.0 * u), Vec2::new(52.0 * u, 52.0 * u));
    if ad.has(flag::ATT) {
        flag_box(p, att.center(), "ATT", u);
    } else {
        attitude(p, att, sim, u, ad.pitch, ad.roll);
        if !standby && ls_tuned(sim) {
            ls_scales(p, att, sim, u);
        }
    }
    if !standby && ls_tuned(sim) {
        ls_info(p, Pos2::new(s.min.x + 2.0 * u, s.max.y - 12.0 * u), sim, u);
    }
    // --- speed tape
    let st = Rect::from_min_max(Pos2::new(s.min.x + 3.0 * u, att.min.y - 2.0 * u), Pos2::new(s.min.x + 17.0 * u, att.max.y + 2.0 * u));
    if ad.has(flag::SPD) {
        flag_box(p, st.center(), "SPD", u);
    } else {
        speed_tape(p, st, sim, ias, u);
    }
    // --- altitude tape + VS
    let at = Rect::from_min_max(Pos2::new(s.min.x + 77.0 * u, att.min.y - 2.0 * u), Pos2::new(s.min.x + 90.0 * u, att.max.y + 2.0 * u));
    let vs = Rect::from_min_max(Pos2::new(s.min.x + 91.0 * u, att.min.y + 2.0 * u), Pos2::new(s.max.x - 1.0 * u, att.max.y - 2.0 * u));
    if ad.has(flag::VS) {
        flag_box(p, vs.center(), "V/S", u * 0.8);
    } else {
        vs_scale(p, vs, ad.vs * FT * 60.0, u);
    }
    if ad.has(flag::ALT) {
        flag_box(p, at.center(), "ALT", u);
    } else {
        alt_tape(p, at, sim, alt_ft, u, world.wx.qnh_hpa);
    }
    // --- heading tape
    let ht = Rect::from_min_max(Pos2::new(att.min.x, s.max.y - 13.0 * u), Pos2::new(att.max.x, s.max.y - 4.0 * u));
    if ad.has(flag::HDG) {
        flag_box(p, ht.center(), "HDG", u);
    } else {
        heading_tape(p, ht, sim, u, ad.heading);
    }
    let _ = def;
}

fn fma_band(p: &egui::Painter, r: Rect, sim: &Sim, u: f32) {
    let ap = &sim.ap;
    let def = sim.def();
    let cols = 5;
    let w = r.width() / cols as f32;
    for i in 1..cols {
        let x = r.min.x + w * i as f32;
        p.line_segment([Pos2::new(x, r.min.y + 1.0 * u), Pos2::new(x, r.max.y - 1.0 * u)], Stroke::new(1.0, Color32::from_gray(90)));
    }
    p.line_segment([Pos2::new(r.min.x, r.max.y), Pos2::new(r.max.x, r.max.y)], Stroke::new(1.0, Color32::from_gray(70)));
    let cell = |i: usize, line: usize, s: &str, col: Color32, boxed: bool| {
        let c = Pos2::new(r.min.x + w * (i as f32 + 0.5), r.min.y + if line == 0 { 3.0 * u } else { 6.8 * u });
        let rr = txt(p, c, Align2::CENTER_CENTER, s, 2.9 * u, col);
        if boxed {
            p.rect_stroke(rr.expand2(Vec2::new(0.8 * u, 0.2 * u)), 0, Stroke::new(1.0, WHITE), egui::StrokeKind::Outside);
        }
    };
    let bot = sim.bot.is_some();
    if ap.engaged && !bot {
        // Airbus FMA: thrust · vertical · lateral · approach capability · engagement;
        // active modes green, armed cyan, engagement white.
        let f = ap.fma(def, &sim.ac);
        if !f[0].is_empty() {
            cell(0, 0, &f[0], GREEN, f[0] == "RETARD");
        }
        cell(1, 0, &f[1], GREEN, ap.gs_star > 0.0);
        if !f[2].is_empty() {
            cell(1, 1, &f[2], CYAN, false);
        } else if let Some(a) = ap.alt.filter(|a| (a - sim.ac.pos.y).abs() >= 15.0 && ap.vert == crate::autopilot::Vert::Basic) {
            cell(1, 1, &format!("ALT {:.0}", a * FT), CYAN, false);
        }
        cell(2, 0, &f[3], GREEN, ap.capturing);
        if !f[4].is_empty() {
            cell(2, 1, &f[4], CYAN, false);
        }
        if ap.appr {
            let auto = crate::autopilot::Autopilot::autoland(def);
            cell(3, 0, if auto { "CAT 3" } else { "CAT 1" }, WHITE, false);
            cell(3, 1, if auto { "DUAL" } else { "DH 200" }, if auto { WHITE } else { CYAN }, false);
        }
        cell(4, 0, "AP1", WHITE, false);
        cell(4, 1, if def.has_engine && ap.ias.is_some() || ap.retard { "A/THR" } else { "1FD2" }, WHITE, false);
    } else if bot {
        // A/THR
        if def.has_engine {
            if ap.ias.is_some() {
                cell(0, 0, "SPEED", GREEN, false);
            } else {
                cell(0, 0, "THR", GREEN, false);
            }
        }
        cell(1, 0, "AUTO", GREEN, false);
        cell(2, 0, "NAV", GREEN, false);
        cell(3, 0, "AUTO FLT", MAG, false);
        if def.has_engine {
            cell(4, 1, "A/THR", WHITE, false);
        }
    } else if sim.ap_warn > 0.0 && (sim.time * 3.0).fract() < 0.6 {
        cell(4, 0, "AP OFF", pal::RED, true);
    }
}

/// NAV 1 tuned to a localizer frequency (the PFD shows the LS scales).
fn ls_tuned(sim: &Sim) -> bool {
    sim.nav.equipped && crate::nav::db::is_ils_freq(sim.nav.radios.nav[0].active)
}

/// Localizer / glideslope deviation scales with magenta diamonds (2 dots = full scale).
fn ls_scales(p: &egui::Painter, att: Rect, sim: &Sim, u: f32) {
    let o = &sim.nav.radios.nav[0].out;
    let dot = att.width() * 0.085;
    // LOC: below the attitude sphere.
    let ly = att.max.y + 4.4 * u;
    let lc = Pos2::new(att.center().x, ly);
    for k in [-2.0f32, -1.0, 1.0, 2.0] {
        p.circle_stroke(lc + Vec2::new(k * dot, 0.0), 0.75 * u, Stroke::new(1.3, WHITE));
    }
    p.line_segment([lc + Vec2::new(0.0, -1.6 * u), lc + Vec2::new(0.0, 1.6 * u)], Stroke::new(2.0, YELLOW));
    let diamond = |c: Pos2, filled: bool| {
        let (w, h) = (1.6 * u, 1.2 * u);
        let pts = vec![c + Vec2::new(-w, 0.0), c + Vec2::new(0.0, -h), c + Vec2::new(w, 0.0), c + Vec2::new(0.0, h)];
        if filled {
            p.add(Shape::convex_polygon(pts, MAG, Stroke::new(1.0, MAG)));
        } else {
            p.add(Shape::closed_line(pts, Stroke::new(1.4, MAG)));
        }
    };
    if o.loc && o.valid {
        let x = (o.cdi * 2.0).clamp(-2.2, 2.2);
        diamond(lc + Vec2::new(x * dot, 0.0), x.abs() < 2.0);
    }
    // G/S: right of the attitude sphere.
    let gx = att.max.x - 3.0 * u;
    let gc = Pos2::new(gx, att.center().y);
    for k in [-2.0f32, -1.0, 1.0, 2.0] {
        p.circle_stroke(gc + Vec2::new(0.0, k * dot), 0.75 * u, Stroke::new(1.3, WHITE));
    }
    p.line_segment([gc + Vec2::new(-1.6 * u, 0.0), gc + Vec2::new(1.6 * u, 0.0)], Stroke::new(2.0, YELLOW));
    if o.loc && o.gs_valid {
        let y = (o.gs * 2.0).clamp(-2.2, 2.2);
        diamond(gc - Vec2::new(0.0, y * dot), y.abs() < 2.0);
    }
}

/// ILS ident / frequency / DME, bottom left of the PFD (magenta).
fn ls_info(p: &egui::Painter, at: Pos2, sim: &Sim, u: f32) {
    let rx = &sim.nav.radios.nav[0];
    let o = &rx.out;
    let ident = if o.valid { o.ident.clone() } else { String::new() };
    txt(p, at, Align2::LEFT_TOP, ident, 2.8 * u, MAG);
    txt(p, at + Vec2::new(0.0, 3.4 * u), Align2::LEFT_TOP, crate::nav::db::freq_label(rx.active, false), 2.8 * u, MAG);
    if let Some(d) = o.dme {
        txt(p, at + Vec2::new(0.0, 6.8 * u), Align2::LEFT_TOP, format!("{:.1}NM", d / 1852.0), 2.8 * u, MAG);
    }
}

fn attitude(p: &egui::Painter, r: Rect, sim: &Sim, u: f32, pitch: f32, roll: f32) {
    let ac = &sim.ac;
    let pitch = pitch.to_degrees();
    let ppd = r.height() / 32.0; // points per degree
    let c = r.center();
    let up = Vec2::new(-roll.sin(), -roll.cos());
    let along = Vec2::new(roll.cos(), -roll.sin());
    let h0 = c - up * (pitch * ppd);
    // Sky / ground: big rotated quads clipped to the attitude rect.
    let pc = p.with_clip_rect(r.shrink(0.5));
    let big = r.width() * 3.0;
    let quad = |off0: f32, off1: f32, top: Color32, bot: Color32| {
        let a = h0 + up * off0 - along * big;
        let b = h0 + up * off0 + along * big;
        let cc = h0 + up * off1 + along * big;
        let d = h0 + up * off1 - along * big;
        let mut m = epaint::Mesh::default();
        m.colored_vertex(a, top);
        m.colored_vertex(b, top);
        m.colored_vertex(cc, bot);
        m.colored_vertex(d, bot);
        m.add_triangle(0, 1, 2);
        m.add_triangle(0, 2, 3);
        pc.add(Shape::mesh(m));
    };
    quad(big, 0.0, Color32::from_rgb(22, 92, 176), Color32::from_rgb(62, 146, 216));
    quad(0.0, -big, Color32::from_rgb(132, 88, 46), Color32::from_rgb(88, 56, 28));
    pc.line_segment([h0 - along * big, h0 + along * big], Stroke::new(1.6, WHITE));
    // pitch ladder (2.5° steps, labels every 10°), clipped to a central window
    let lad = pc.with_clip_rect(Rect::from_center_size(c, Vec2::new(r.width() * 0.8, r.height() * 0.72)));
    for i in -16..=16 {
        if i == 0 {
            continue;
        }
        let deg = i as f32 * 2.5;
        let cen = h0 + up * (deg * ppd);
        let half = if i % 4 == 0 { r.width() * 0.16 } else if i % 2 == 0 { r.width() * 0.08 } else { r.width() * 0.04 };
        lad.line_segment([cen - along * half, cen + along * half], Stroke::new(1.3, WHITE));
        if i % 4 == 0 {
            for sg in [-1.0, 1.0] {
                lad.text(cen + along * sg * (half + 3.2 * u), Align2::CENTER_CENTER, format!("{:.0}", deg.abs()), fi(2.6 * u), WHITE);
            }
        }
    }
    // bank scale (fixed, top) + moving index and slip trapezoid
    let br = r.height() * 0.46;
    for d in [-45.0f32, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0] {
        let a = d.to_radians();
        let len = if d == 0.0 { 0.0 } else if d.abs() == 30.0 { 3.0 * u } else { 1.8 * u };
        if len > 0.0 {
            p.line_segment([polar(c, br, a), polar(c, br + len, a)], Stroke::new(1.5, WHITE));
        }
    }
    super::arc(p, c, br, -45f32.to_radians(), 45f32.to_radians(), 1.5, WHITE);
    p.add(Shape::convex_polygon(vec![polar(c, br, 0.0), polar(c, br + 2.6 * u, -0.05), polar(c, br + 2.6 * u, 0.05)], YELLOW, Stroke::NONE));
    let a = -roll;
    let tip = polar(c, br - 0.4 * u, a);
    let n = Vec2::new(a.cos(), a.sin());
    let inward = (c - tip).normalized();
    p.add(Shape::convex_polygon(vec![tip, tip + inward * 2.4 * u - n * 1.6 * u, tip + inward * 2.4 * u + n * 1.6 * u], YELLOW, Stroke::NONE));
    let slip = (-ac.slip() * 4.0).clamp(-1.0, 1.0) * 2.5 * u;
    let base = tip + inward * 2.9 * u + n * slip;
    p.add(Shape::convex_polygon(vec![base - n * 2.0 * u, base + n * 2.0 * u, base + n * 2.4 * u + inward * 1.0 * u, base - n * 2.4 * u + inward * 1.0 * u], YELLOW, Stroke::NONE));
    // flight director (green bars)
    if let Some((db, dp)) = flight_director(sim) {
        let k = r.width() * 0.6;
        let x = c.x + (db / 25f32.to_radians()).clamp(-1.0, 1.0) * r.width() * 0.3;
        let y = c.y - dp.to_degrees() * ppd;
        p.line_segment([Pos2::new(x, c.y - k * 0.42), Pos2::new(x, c.y + k * 0.42)], Stroke::new(2.0, GREEN));
        p.line_segment([Pos2::new(c.x - k * 0.42, y), Pos2::new(c.x + k * 0.42, y)], Stroke::new(2.0, GREEN));
    }
    // aircraft symbol (Airbus-style yellow wings with black outline)
    let w = r.width();
    let bar = |a: Pos2, b: Pos2| {
        p.line_segment([a, b], Stroke::new(1.4 * u + 2.0, Color32::BLACK));
        p.line_segment([a, b], Stroke::new(1.4 * u, YELLOW));
    };
    bar(Pos2::new(c.x - w * 0.36, c.y), Pos2::new(c.x - w * 0.16, c.y));
    bar(Pos2::new(c.x - w * 0.16, c.y), Pos2::new(c.x - w * 0.16, c.y + 3.0 * u));
    bar(Pos2::new(c.x + w * 0.16, c.y), Pos2::new(c.x + w * 0.36, c.y));
    bar(Pos2::new(c.x + w * 0.16, c.y), Pos2::new(c.x + w * 0.16, c.y + 3.0 * u));
    p.rect_filled(Rect::from_center_size(c, Vec2::splat(1.8 * u)), 0, Color32::BLACK);
    p.rect_stroke(Rect::from_center_size(c, Vec2::splat(1.8 * u)), 0, Stroke::new(1.2, YELLOW), egui::StrokeKind::Outside);
    // radio altitude
    let agl = (ac.pos.y - sim.ground_y) * FT;
    if agl < 2500.0 && !ac.on_ground {
        let col = if agl < 400.0 { AMBER } else { GREEN };
        digits(p, Pos2::new(c.x, r.max.y - 3.0 * u), Align2::CENTER_CENTER, format!("{:.0}", (agl / 10.0).round() * 10.0), 3.6 * u, col);
    }
    p.rect_stroke(r, 0, Stroke::new(1.0, Color32::from_gray(40)), egui::StrokeKind::Outside);
}

fn speed_tape(p: &egui::Painter, r: Rect, sim: &Sim, ias: f32, u: f32) {
    let def = sim.def();
    p.rect_filled(r, 0, TAPE_BG);
    let pc = p.with_clip_rect(r);
    let ppk = r.height() / 84.0; // points per knot (±42 kt visible)
    let c = r.center();
    let y = |kt: f32| c.y - (kt - ias) * ppk;
    let lo = ((ias - 46.0) / 10.0).floor() as i32 * 10;
    let mut kt = lo.max(30);
    while (kt as f32) < ias + 46.0 {
        let yy = y(kt as f32);
        let major = kt % 20 == 0;
        pc.line_segment([Pos2::new(r.max.x - if major { 2.6 * u } else { 1.4 * u }, yy), Pos2::new(r.max.x, yy)], Stroke::new(1.2, WHITE));
        if major {
            pc.text(Pos2::new(r.max.x - 3.4 * u, yy), Align2::RIGHT_CENTER, format!("{kt}"), fd(3.0 * u), WHITE);
        }
        kt += 10;
    }
    // Stall / VLS / overspeed bands on the right edge.
    let flaps = sim.ac.surfaces().flaps;
    let vs = def.vs1_kt + (def.vs0_kt - def.vs1_kt) * flaps;
    let vls = vs * 1.23;
    let x0 = r.max.x - 1.1 * u;
    barber(&pc, x0, r.max.x, y(vs), r.max.y + 10.0, pal::RED, Color32::BLACK);
    pc.rect_filled(Rect::from_min_max(Pos2::new(x0, y(vls)), Pos2::new(r.max.x, y(vs))), 0, AMBER);
    pc.rect_filled(Rect::from_min_max(Pos2::new(x0 + 0.4 * u, y(vls)), Pos2::new(r.max.x, y(vs) - 1.0)), 0, Color32::BLACK);
    barber(&pc, x0, r.max.x, r.min.y - 10.0, y(def.vne_kt), pal::RED, Color32::BLACK);
    // V speeds
    if def.has_engine {
        if sim.ac.on_ground {
            let yy = y(def.vr_kt);
            pc.circle_filled(Pos2::new(r.max.x + 0.2 * u, yy), 0.9 * u, CYAN);
            pc.text(Pos2::new(r.max.x - 1.6 * u, yy), Align2::RIGHT_CENTER, "R", fi(2.4 * u), CYAN);
        } else {
            let yy = y(def.vref_kt);
            pc.line_segment([Pos2::new(r.max.x - 2.2 * u, yy), Pos2::new(r.max.x, yy)], Stroke::new(2.0, GREEN));
            pc.text(Pos2::new(r.max.x - 2.6 * u, yy), Align2::RIGHT_CENTER, "REF", fi(2.0 * u), GREEN);
        }
    }
    // Selected speed (cyan) or managed (magenta).
    if let Some(t) = sim.ap.ias.filter(|_| sim.ap.engaged) {
        let yy = y(t / KT).clamp(r.min.y + 1.0 * u, r.max.y - 1.0 * u);
        let col = if sim.bot.is_some() { MAG } else { CYAN };
        pc.add(Shape::convex_polygon(vec![Pos2::new(r.max.x, yy), Pos2::new(r.max.x + 2.2 * u, yy - 1.4 * u), Pos2::new(r.max.x + 2.2 * u, yy + 1.4 * u)], col, Stroke::NONE));
    }
    // Trend arrow: speed in 10 s.
    let trend = sim_speed_trend(sim);
    if trend.abs() > 2.0 {
        let x = r.max.x - 0.4 * u;
        let (y0, y1) = (c.y, y(ias + trend));
        p.line_segment([Pos2::new(x, y0), Pos2::new(x, y1)], Stroke::new(1.6, YELLOW));
        let d = if y1 < y0 { -1.0 } else { 1.0 };
        p.add(Shape::line(vec![Pos2::new(x - 1.0 * u, y1 - d * 1.4 * u), Pos2::new(x, y1), Pos2::new(x + 1.0 * u, y1 - d * 1.4 * u)], Stroke::new(1.6, YELLOW)));
    }
    // Reference line + readout.
    p.line_segment([Pos2::new(r.min.x - 0.5 * u, c.y), Pos2::new(r.max.x + 2.6 * u, c.y)], Stroke::new(2.0, YELLOW));
    let rb = Rect::from_min_max(Pos2::new(r.min.x, c.y - 2.4 * u), Pos2::new(r.max.x - 3.0 * u, c.y + 2.4 * u));
    p.rect_filled(rb, 2, Color32::BLACK);
    p.rect_stroke(rb, 2, Stroke::new(1.0, YELLOW), egui::StrokeKind::Outside);
    digits(p, Pos2::new(rb.max.x - 0.8 * u, c.y), Align2::RIGHT_CENTER, format!("{:.0}", ias.max(30.0)), 3.6 * u, if ias < vls && !sim.ac.on_ground { AMBER } else { WHITE });
    // Mach below the tape.
    let m = sim.ac.mach();
    if m > 0.45 {
        digits(p, Pos2::new(r.center().x, r.max.y + 3.4 * u), Align2::CENTER_CENTER, mach_readout(m), 3.0 * u, GREEN);
    }
    if let Some(t) = sim.ap.ias.filter(|_| sim.ap.engaged) {
        txt(p, Pos2::new(r.center().x, r.min.y - 3.0 * u), Align2::CENTER_CENTER, format!("{:.0}", t / KT), 3.0 * u, if sim.bot.is_some() { MAG } else { CYAN });
    }
}

/// Speed change expected in 10 s (kt), from the last frames.
fn sim_speed_trend(sim: &Sim) -> f32 {
    // Along-track acceleration from the flight model's forces is not exposed;
    // estimate from the velocity / attitude: thrust-drag shows up as d(IAS)/dt.
    thread_local! {
        static PREV: std::cell::Cell<(f32, f32, f32)> = const { std::cell::Cell::new((0.0, 0.0, 0.0)) };
    }
    PREV.with(|c| {
        let (t0, v0, f) = c.get();
        let t = sim.time;
        let v = sim.ac.airspeed() / KT;
        let dt = t - t0;
        let mut out = f;
        if dt > 1e-3 && dt < 0.5 {
            let a = (v - v0) / dt;
            out = f + (a * 10.0 - f) * (dt / 1.2).min(1.0);
        } else if dt.abs() > 0.5 {
            out = 0.0;
        }
        if dt > 1e-3 || dt < 0.0 {
            c.set((t, v, out));
        }
        out
    })
}

fn alt_tape(p: &egui::Painter, r: Rect, sim: &Sim, alt: f32, u: f32, qnh: f32) {
    p.rect_filled(r, 0, TAPE_BG);
    let pc = p.with_clip_rect(r);
    let ppf = r.height() / 1100.0; // ±550 ft visible
    let c = r.center();
    let y = |ft: f32| c.y - (ft - alt) * ppf;
    let lo = ((alt - 600.0) / 100.0).floor() as i32 * 100;
    let mut ft = lo;
    while (ft as f32) < alt + 600.0 {
        let yy = y(ft as f32);
        pc.line_segment([Pos2::new(r.min.x, yy), Pos2::new(r.min.x + if ft % 500 == 0 { 2.2 * u } else { 1.2 * u }, yy)], Stroke::new(1.2, WHITE));
        if ft % 500 == 0 {
            pc.text(Pos2::new(r.min.x + 3.0 * u, yy), Align2::LEFT_CENTER, format!("{}", ft / 100), fd(3.0 * u), WHITE);
        }
        ft += 100;
    }
    // ground (red-amber strip at 0 ft)
    let gy = y(0.0);
    if gy < r.max.y {
        barber(&pc, r.max.x - 1.2 * u, r.max.x, gy, r.max.y + 10.0, AMBER, Color32::BLACK);
    }
    // selected altitude
    if let Some(a) = sim.ap.alt.filter(|_| sim.ap.engaged) {
        let sel = a * FT;
        let col = if sim.bot.is_some() { MAG } else { CYAN };
        let yy = y(sel);
        if yy > r.min.y && yy < r.max.y {
            let b = Rect::from_min_max(Pos2::new(r.min.x - 0.5 * u, yy - 2.6 * u), Pos2::new(r.min.x + 1.6 * u, yy + 2.6 * u));
            pc.rect_stroke(b, 0, Stroke::new(1.6, col), egui::StrokeKind::Inside);
        }
        txt(p, Pos2::new(r.center().x, r.min.y - 3.0 * u), Align2::CENTER_CENTER, format!("{:.0}", sel), 3.0 * u, col);
    }
    // readout box with a rolling drum for the last two digits
    let rb = Rect::from_min_max(Pos2::new(r.min.x + 0.5 * u, c.y - 2.8 * u), Pos2::new(r.max.x + 3.0 * u, c.y + 2.8 * u));
    p.rect_filled(rb, 2, Color32::BLACK);
    p.rect_stroke(rb, 2, Stroke::new(1.0, YELLOW), egui::StrokeKind::Outside);
    let hundreds = (alt / 100.0).floor();
    let rem = alt - hundreds * 100.0;
    let lead = format!("{:.0}", hundreds.max(-9.0));
    let split = rb.min.x + rb.width() * 0.60;
    digits(p, Pos2::new(split, c.y), Align2::RIGHT_CENTER, lead, 3.6 * u, GREEN);
    let drum = p.with_clip_rect(Rect::from_min_max(Pos2::new(split, rb.min.y - 1.6 * u), Pos2::new(rb.max.x, rb.max.y + 1.6 * u)));
    let step = 20.0;
    let base = (rem / step).floor() * step;
    let frac = (rem - base) / step;
    let pitch = 4.4 * u;
    for k in -1..=2 {
        let v = (base + k as f32 * step).rem_euclid(100.0);
        let yy = c.y - (k as f32 - frac) * pitch;
        drum.text(Pos2::new(split + 0.3 * u, yy), Align2::LEFT_CENTER, format!("{:02.0}", v), fd(2.8 * u), GREEN);
    }
    // baro setting
    txt(p, Pos2::new(r.center().x, r.max.y + 3.2 * u), Align2::CENTER_CENTER, format!("QNH {qnh:.0}"), 2.8 * u, CYAN);
}

fn vs_scale(p: &egui::Painter, r: Rect, fpm: f32, u: f32) {
    p.rect_filled(r, 2, Color32::from_rgb(28, 32, 38));
    let c = r.left_center() + Vec2::new(r.width() * 0.25, 0.0);
    let span = r.height() * 0.45;
    let y = |v: f32| {
        let s = v.signum();
        let a = v.abs();
        let t = if a <= 1000.0 { a / 1000.0 * 0.5 } else if a <= 2000.0 { 0.5 + (a - 1000.0) / 1000.0 * 0.25 } else { 0.75 + ((a - 2000.0) / 4000.0).min(1.0) * 0.25 };
        c.y - s * t * span
    };
    for v in [-6000.0, -2000.0, -1000.0, -500.0, 0.0, 500.0, 1000.0, 2000.0, 6000.0] {
        let yy = y(v);
        let major = (v as i32) % 1000 == 0;
        p.line_segment([Pos2::new(r.min.x + 0.4 * u, yy), Pos2::new(r.min.x + if major { 1.6 * u } else { 1.0 * u }, yy)], Stroke::new(1.0, WHITE));
        if major && v != 0.0 {
            txt(p, Pos2::new(r.min.x + 2.4 * u, yy), Align2::LEFT_CENTER, format!("{:.0}", v.abs() / 1000.0), 2.3 * u, WHITE);
        }
    }
    let tip = Pos2::new(r.min.x + 0.6 * u, y(fpm.clamp(-6000.0, 6000.0)));
    let col = if fpm.abs() > 6000.0 { AMBER } else { GREEN };
    p.line_segment([Pos2::new(r.max.x + 2.0 * u, c.y), tip], Stroke::new(2.0, col));
    if fpm.abs() > 200.0 {
        let at = if fpm > 0.0 { r.center_top() + Vec2::new(0.0, -2.2 * u) } else { r.center_bottom() + Vec2::new(0.0, 2.2 * u) };
        digits(p, at, Align2::CENTER_CENTER, format!("{:.0}", (fpm.abs() / 100.0).round()), 2.6 * u, col);
    }
}

fn heading_tape(p: &egui::Painter, r: Rect, sim: &Sim, u: f32, heading: f32) {
    p.rect_filled(r, 0, TAPE_BG);
    let pc = p.with_clip_rect(r);
    let hdg = heading.to_degrees().rem_euclid(360.0);
    let ppd = r.width() / 50.0;
    let c = r.center();
    let x = |d: f32| c.x + wrap_deg(d - hdg) * ppd;
    let base = (hdg / 5.0).floor() as i32 * 5;
    for k in -6..=6 {
        let d = base + k * 5;
        let xx = x(d as f32);
        let major = d.rem_euclid(10) == 0;
        pc.line_segment([Pos2::new(xx, r.min.y), Pos2::new(xx, r.min.y + if major { 2.0 * u } else { 1.0 * u })], Stroke::new(1.2, WHITE));
        if major {
            let lab = (d.rem_euclid(360) / 10).to_string();
            pc.text(Pos2::new(xx, r.min.y + 4.6 * u), Align2::CENTER_CENTER, lab, fd(if d.rem_euclid(30) == 0 { 3.2 } else { 2.6 } * u), WHITE);
        }
    }
    // track diamond (green)
    let v = sim.ac.vel;
    if vec2(v.x, v.z).length() > 5.0 {
        let trk = v.x.atan2(-v.z).to_degrees();
        let xx = x(trk);
        let y0 = r.min.y + 1.4 * u;
        p.add(Shape::closed_line(vec![Pos2::new(xx, y0 - 1.2 * u), Pos2::new(xx + 1.0 * u, y0), Pos2::new(xx, y0 + 1.2 * u), Pos2::new(xx - 1.0 * u, y0)], Stroke::new(1.4, GREEN)));
    }
    if let Some(h) = sim.ap.hdg.filter(|_| sim.ap.engaged) {
        let xx = x(h.to_degrees()).clamp(r.min.x, r.max.x);
        let col = if sim.bot.is_some() { MAG } else { CYAN };
        p.add(Shape::convex_polygon(vec![Pos2::new(xx, r.min.y), Pos2::new(xx - 1.3 * u, r.min.y - 2.0 * u), Pos2::new(xx + 1.3 * u, r.min.y - 2.0 * u)], col, Stroke::NONE));
    }
    // ILS course (magenta), when NAV 1 has a localizer.
    let o = &sim.nav.radios.nav[0].out;
    if o.loc && o.valid {
        let xx = x(o.course.to_degrees());
        if xx > r.min.x && xx < r.max.x {
            p.line_segment([Pos2::new(xx, r.min.y), Pos2::new(xx, r.min.y + 3.4 * u)], Stroke::new(2.0, MAG));
        }
    }
    p.line_segment([Pos2::new(c.x, r.min.y - 2.4 * u), Pos2::new(c.x, r.min.y + 3.0 * u)], Stroke::new(2.0, YELLOW));
}

fn wrap_deg(d: f32) -> f32 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

// ---------------------------------------------------------------- ND

pub const ND_RANGES_NM: [f32; 6] = [2.5, 5.0, 10.0, 20.0, 40.0, 80.0];

/// Navigation display: ARC / ROSE NAV (map, flight plan), ROSE ILS / ROSE
/// VOR (course and deviation from NAV 1) and PLAN (north up around the TO
/// waypoint); `route` = the remaining waypoints (flight plan or the bot's).
#[allow(clippy::too_many_arguments)]
pub fn nd(p: &egui::Painter, cv: &mut canvas::Frame, ppp: f32, r: Rect, sim: &Sim, world: &World, tiles: &Tiles, route: &[(String, glam::Vec2)]) {
    use crate::nav::state::NdMode;
    let s = housing(p, r);
    if !sim.ac.sys.display_on("nd") {
        dark(p, s);
        return;
    }
    let mode = if sim.nav.equipped { sim.nav.nd_mode } else { NdMode::Arc };
    match mode {
        NdMode::Arc => nd_map(p, cv, ppp, s, sim, world, tiles, route, false),
        NdMode::RoseNav => nd_map(p, cv, ppp, s, sim, world, tiles, route, true),
        NdMode::RoseIls | NdMode::RoseVor => nd_rose_radio(p, s, sim, mode == NdMode::RoseIls),
        NdMode::Plan => nd_plan(p, s, sim, world, route),
    }
}

/// Heading / track pointers, compass ticks for an ND rose (`full`) or arc.
fn nd_compass(pc: &egui::Painter, anchor: Pos2, rad: f32, up: f32, full: bool, u: f32) {
    let lim = if full { PI + 0.01 } else { PI / 2.2 };
    if full {
        pc.circle_stroke(anchor, rad, Stroke::new(1.5, WHITE));
    } else {
        super::arc(pc, anchor, rad, -PI / 2.2, PI / 2.2, 1.5, WHITE);
    }
    let tdeg = up.to_degrees();
    for d in 0..72 {
        let deg = d as f32 * 5.0;
        let a = wrap_deg(deg - tdeg).to_radians();
        if a.abs() > lim {
            continue;
        }
        let major = d % 2 == 0;
        let (r0, r1) = if full { (rad, rad - if major { 2.4 * u } else { 1.2 * u }) } else { (rad, rad + if major { 2.4 * u } else { 1.2 * u }) };
        pc.line_segment([polar(anchor, r0, a), polar(anchor, r1, a)], Stroke::new(1.2, WHITE));
        if d % 6 == 0 {
            let lr = if full { rad - 5.0 * u } else { rad + 5.0 * u };
            pc.text(polar(anchor, lr, a), Align2::CENTER_CENTER, format!("{}", (deg as i32 / 10) % 36), fd(if d % 18 == 0 { 3.4 } else { 2.8 } * u), WHITE);
        }
    }
}

/// ARC / ROSE NAV: the moving map with the route and navaids.
#[allow(clippy::too_many_arguments)]
fn nd_map(p: &egui::Painter, cv: &mut canvas::Frame, ppp: f32, s: Rect, sim: &Sim, world: &World, tiles: &Tiles, route: &[(String, glam::Vec2)], rose: bool) {
    let pc = p.with_clip_rect(s);
    let u = s.width() / 100.0;
    let ac = &sim.ac;
    let range_nm = ND_RANGES_NM[sim.map_zoom.min(ND_RANGES_NM.len() - 1)];
    let range_m = range_nm * 1852.0 * if rose { 0.5 } else { 1.0 };
    let anchor = if rose { Pos2::new(s.center().x, s.min.y + 54.0 * u) } else { Pos2::new(s.center().x, s.min.y + 78.0 * u) };
    let arc_r = if rose { 36.0 * u } else { 66.0 * u };
    let v = vec2(ac.vel.x, ac.vel.z);
    let trk = if v.length() > 8.0 { v.x.atan2(-v.y) } else { ac.heading() };
    let pos = vec2(ac.pos.x, ac.pos.z);
    let view = View { rect: s, anchor, centre: pos, s: arc_r / range_m, up: trk };
    // map underneath, clipped to the compass circle
    let key = if rose { "nd_rose" } else { "nd" };
    let id = if cv.due(key, s.size()) {
        let shapes = map::base_shapes(&view, world, tiles, Style::Nd);
        cv.draw(key, s.size(), ppp, map::bg(Style::Nd), shapes)
    } else {
        cv.ids.get(key).copied()
    };
    if let Some(id) = id {
        canvas::image_circle(&pc, id, s, anchor, arc_r * 1.02, Color32::from_gray(255));
    }
    // Weather radar returns (wx/radar.rs) over the map, under the symbols.
    if sim.def().category == crate::flight::Category::Airliner && sim.wxc.radar.ctl.on {
        radar_returns(&pc, &sim.wxc.radar.img, anchor, arc_r, range_m, ac.heading() - trk);
        let r = &sim.wxc.radar;
        let tilt = if r.ctl.tilt.is_none() { format!("AUTO {:+.1}", r.img.tilt) } else { format!("{:+.1}", r.img.tilt) };
        let gain = if r.ctl.gain == 0.0 { "CAL".to_string() } else { format!("{:+.0}", r.ctl.gain) };
        txt(&pc, Pos2::new(s.min.x + 2.0 * u, s.max.y - 6.0 * u), Align2::LEFT_BOTTOM, format!("WXR  TILT {tilt}  GAIN {gain}"), 2.4 * u, GREEN);
    }
    let mut taken = Vec::new();
    let clip = pc.with_clip_rect(Rect::from_min_max(s.min + Vec2::new(0.0, 8.0 * u), s.max - Vec2::new(0.0, 8.0 * u)));
    map::airports(&clip, &view, world, Style::Nd, &mut taken);
    if sim.nav.equipped && sim.nav.show_navaids {
        let env = crate::nav::state::WorldEnv(world);
        let near = sim.nav.navaids_near(&env, pos, range_m * 1.2);
        map::navaids(&clip, &view, world, &near, CYAN, &mut taken);
    }
    // route (magenta)
    map::route(&clip, &view, pos, route, MAG);
    nd_compass(&pc, anchor, arc_r, trk, rose, u);
    // range arcs
    let half = arc_r * 0.5;
    let n = if rose { 80 } else { 40 };
    let span = if rose { 2.0 * PI } else { PI / 1.1 };
    let a_start = if rose { -PI } else { -PI / 2.2 };
    for i in 0..n {
        if i % 2 == 0 {
            let a0 = a_start + i as f32 / n as f32 * span;
            let a1 = a0 + span / n as f32;
            pc.line_segment([polar(anchor, half, a0), polar(anchor, half, a1)], Stroke::new(1.0, Color32::from_gray(150)));
        }
    }
    let shown = range_m / 1852.0;
    txt(&pc, polar(anchor, half, -1.15) + Vec2::new(-1.0 * u, 0.0), Align2::RIGHT_CENTER, format!("{}", shown / 2.0), 2.6 * u, CYAN);
    if !rose {
        txt(&pc, polar(anchor, arc_r, -1.15) + Vec2::new(-1.4 * u, 0.0), Align2::RIGHT_CENTER, format!("{}", shown), 2.6 * u, CYAN);
    }
    // heading pointer (yellow lubber at track) and heading triangle
    let (r_in, r_out) = if rose { (arc_r + 1.0 * u, arc_r + 4.0 * u) } else { (arc_r - 2.0 * u, arc_r + 3.0 * u) };
    pc.line_segment([polar(anchor, r_in, 0.0), polar(anchor, r_out, 0.0)], Stroke::new(2.0, YELLOW));
    let ha = wrap_pi(ac.heading() - trk);
    pc.add(Shape::convex_polygon(vec![polar(anchor, arc_r + 0.2 * u, ha), polar(anchor, arc_r + 2.6 * u, ha - 0.03), polar(anchor, arc_r + 2.6 * u, ha + 0.03)], YELLOW, Stroke::NONE));
    if let Some(h) = sim.ap.hdg.filter(|_| sim.ap.engaged && sim.ap.lat == crate::autopilot::Lat::Hdg) {
        let a = wrap_pi(h - trk);
        if rose || a.abs() < PI / 2.2 {
            let col = if sim.bot.is_some() { MAG } else { CYAN };
            pc.add(Shape::closed_line(vec![polar(anchor, arc_r, a), polar(anchor, arc_r + 2.4 * u, a - 0.035), polar(anchor, arc_r + 2.4 * u, a + 0.035)], Stroke::new(1.6, col)));
        }
    }
    // ILS course (magenta) when NAV 1 has a localizer.
    let o = &sim.nav.radios.nav[0].out;
    if o.loc && o.valid {
        let a = wrap_pi(o.course - trk);
        if rose || a.abs() < PI / 2.2 {
            pc.line_segment([polar(anchor, arc_r - 3.0 * u, a), polar(anchor, arc_r + 3.0 * u, a)], Stroke::new(2.4, MAG));
        }
    }
    // track line (green dashed)
    for i in 0..12 {
        let a = anchor + Vec2::new(0.0, -arc_r * i as f32 / 12.0);
        let b = anchor + Vec2::new(0.0, -arc_r * (i as f32 + 0.5) / 12.0);
        pc.line_segment([a, b], Stroke::new(1.4, GREEN));
    }
    // Multiplayer traffic (TCAS symbols, ui/mp.rs).
    super::mp::nd_traffic(&clip, &view, sim, u);
    aircraft_mark(&pc, anchor, u);
    nd_corner_info(&pc, s, sim, world, route, u);
    adf_vor_footer(&pc, s, sim, u);
    pc.text(Pos2::new(s.max.x - 2.0 * u, s.max.y - 2.0 * u), Align2::RIGHT_BOTTOM, format!("{}  {range_nm} NM", if rose { "ROSE NAV" } else { "ARC" }), fi(2.6 * u), CYAN);
}

/// Radar bins as coloured polar cells: `rel` = heading − the display's up direction.
fn radar_returns(pc: &egui::Painter, img: &crate::wx::radar::RadarImage, anchor: Pos2, arc_r: f32, range_m: f32, rel: f32) {
    use crate::wx::radar::{AZ_BINS, HALF_SWEEP, Level, RANGE_BINS};
    let mut mesh = egui::Mesh::default();
    let k = arc_r / range_m.max(1.0);
    let da = HALF_SWEEP * 2.0 / (AZ_BINS - 1) as f32;
    let dr = img.range / RANGE_BINS as f32;
    for a in 0..AZ_BINS {
        let az = rel + (a as f32 / (AZ_BINS - 1) as f32 * 2.0 - 1.0) * HALF_SWEEP;
        let (a0, a1) = (az - da * 0.5, az + da * 0.5);
        for r in 0..RANGE_BINS {
            let lvl = img.at(a, r);
            if lvl == Level::None {
                continue;
            }
            let (r0, r1) = (r as f32 * dr * k, (r + 1) as f32 * dr * k);
            if r0 > arc_r * 1.02 {
                break;
            }
            let [cr, cg, cb] = lvl.rgb();
            let col = Color32::from_rgba_unmultiplied(cr, cg, cb, if lvl == Level::Ground { 120 } else { 210 });
            let i = mesh.vertices.len() as u32;
            for p in [polar(anchor, r0, a0), polar(anchor, r1, a0), polar(anchor, r1, a1), polar(anchor, r0, a1)] {
                mesh.colored_vertex(p, col);
            }
            mesh.add_triangle(i, i + 1, i + 2);
            mesh.add_triangle(i, i + 2, i + 3);
        }
    }
    if !mesh.vertices.is_empty() {
        pc.add(Shape::mesh(mesh));
    }
}

fn aircraft_mark(pc: &egui::Painter, anchor: Pos2, u: f32) {
    let y = YELLOW;
    pc.line_segment([anchor + Vec2::new(0.0, -2.0 * u), anchor + Vec2::new(0.0, 4.6 * u)], Stroke::new(2.4, y));
    pc.line_segment([anchor + Vec2::new(-3.4 * u, 0.4 * u), anchor + Vec2::new(3.4 * u, 0.4 * u)], Stroke::new(2.4, y));
    pc.line_segment([anchor + Vec2::new(-1.4 * u, 3.8 * u), anchor + Vec2::new(1.4 * u, 3.8 * u)], Stroke::new(2.4, y));
}

/// GS / TAS / wind (top left) and the TO waypoint (top right).
fn nd_corner_info(pc: &egui::Painter, s: Rect, sim: &Sim, world: &World, route: &[(String, glam::Vec2)], u: f32) {
    let ac = &sim.ac;
    let v = vec2(ac.vel.x, ac.vel.z);
    let trk = if v.length() > 8.0 { v.x.atan2(-v.y) } else { ac.heading() };
    let pos = vec2(ac.pos.x, ac.pos.z);
    let gs = v.length() / KT;
    let tl = s.min + Vec2::new(2.0 * u, 2.4 * u);
    pc.text(tl, Align2::LEFT_TOP, "GS", fi(2.6 * u), WHITE);
    pc.text(tl + Vec2::new(4.6 * u, -0.3 * u), Align2::LEFT_TOP, format!("{gs:.0}"), fdb(3.2 * u), GREEN);
    pc.text(tl + Vec2::new(13.0 * u, 0.0), Align2::LEFT_TOP, "TAS", fi(2.6 * u), WHITE);
    pc.text(tl + Vec2::new(19.0 * u, -0.3 * u), Align2::LEFT_TOP, format!("{:.0}", ac.tas() / KT), fdb(3.2 * u), GREEN);
    let w = &world.wx;
    pc.text(tl + Vec2::new(0.0, 4.4 * u), Align2::LEFT_TOP, format!("{:03.0}°/{:.0}", w.wind_from, w.wind_kt), fdb(3.0 * u), GREEN);
    if w.wind_kt >= 2.0 {
        let to = wrap_pi((w.wind_from + 180.0).to_radians() - trk);
        let c = tl + Vec2::new(4.0 * u, 12.0 * u);
        let d = Vec2::new(to.sin(), -to.cos()) * 3.4 * u;
        pc.line_segment([c - d, c + d], Stroke::new(2.0, GREEN));
        let n = Vec2::new(-d.y, d.x) * 0.4;
        pc.add(Shape::line(vec![c + d * 0.4 + n, c + d, c + d * 0.4 - n], Stroke::new(2.0, GREEN)));
    }
    if let Some((name, wp)) = route.first() {
        let d = *wp - pos;
        let brg = d.x.atan2(-d.y).to_degrees().rem_euclid(360.0);
        let tr = Pos2::new(s.max.x - 2.0 * u, s.min.y + 2.4 * u);
        pc.text(tr, Align2::RIGHT_TOP, name.to_uppercase(), fib(2.8 * u), if sim.bot.is_some() { MAG } else { WHITE });
        pc.text(tr + Vec2::new(0.0, 4.0 * u), Align2::RIGHT_TOP, format!("{brg:03.0}°   {:.1} NM", d.length() / 1852.0), fdb(2.8 * u), GREEN);
        if sim.bot.is_none() {
            let gsm = v.length().max(30.0);
            let t = (sim.hours + d.length() / gsm / 3600.0).rem_euclid(24.0);
            pc.text(tr + Vec2::new(0.0, 8.0 * u), Align2::RIGHT_TOP, format!("{:02}:{:02}", t as i32, (t.fract() * 60.0) as i32), fdb(2.8 * u), GREEN);
        }
    }
}

/// Bottom corners: VOR 1 / ADF (ident, DME) as on the Airbus ND.
fn adf_vor_footer(pc: &egui::Painter, s: Rect, sim: &Sim, u: f32) {
    if !sim.nav.equipped {
        return;
    }
    let o = &sim.nav.radios.nav[0].out;
    let bl = Pos2::new(s.min.x + 2.0 * u, s.max.y - 10.0 * u);
    if o.valid && !o.loc {
        pc.text(bl, Align2::LEFT_TOP, format!("VOR1 {}", o.ident), fi(2.6 * u), WHITE);
        if let Some(d) = o.dme {
            pc.text(bl + Vec2::new(0.0, 3.6 * u), Align2::LEFT_TOP, format!("{:.1} NM", d / 1852.0), fdb(2.8 * u), GREEN);
        }
    }
    let a = &sim.nav.radios.adf.out;
    if a.valid {
        pc.text(Pos2::new(s.max.x - 2.0 * u, s.max.y - 10.0 * u), Align2::RIGHT_TOP, format!("ADF1 {}", a.ident), fi(2.6 * u), GREEN);
    }
}

/// ROSE ILS / ROSE VOR: NAV 1 course pointer, deviation bar and dots,
/// TO / FROM, glideslope scale, ADF needle.
fn nd_rose_radio(p: &egui::Painter, s: Rect, sim: &Sim, ils: bool) {
    let pc = p.with_clip_rect(s);
    let u = s.width() / 100.0;
    let ac = &sim.ac;
    let hdg = sim.ac.sys.air_data(ac).heading;
    let c = Pos2::new(s.center().x, s.min.y + 54.0 * u);
    let rad = 36.0 * u;
    nd_compass(&pc, c, rad, hdg, true, u);
    pc.line_segment([polar(c, rad + 1.0 * u, 0.0), polar(c, rad + 4.0 * u, 0.0)], Stroke::new(2.0, YELLOW));
    let rx = &sim.nav.radios.nav[0];
    let o = &rx.out;
    let course = if o.loc { o.course } else { rx.obs };
    let a = wrap_pi(course - hdg);
    let dir = Vec2::new(a.sin(), -a.cos());
    let side = Vec2::new(-dir.y, dir.x);
    let col = if ils { MAG } else { CYAN };
    // course pointer (head and tail)
    let head = c + dir * (rad - 2.0 * u);
    pc.line_segment([c + dir * (rad * 0.42), head], Stroke::new(2.6, col));
    pc.add(Shape::convex_polygon(vec![head, head - dir * 3.0 * u + side * 1.6 * u, head - dir * 3.0 * u - side * 1.6 * u], col, Stroke::NONE));
    pc.line_segment([c - dir * (rad * 0.42), c - dir * (rad - 2.0 * u)], Stroke::new(2.6, col));
    // deviation dots (±2) and the bar
    let dot = rad * 0.17;
    for k in [-2.0f32, -1.0, 1.0, 2.0] {
        pc.circle_stroke(c + side * k * dot, 0.8 * u, Stroke::new(1.3, WHITE));
    }
    let ok = o.valid && (o.loc == ils || !ils && !o.loc);
    if ok {
        let x = (o.cdi * 2.0).clamp(-2.4, 2.4) * dot;
        pc.line_segment([c + side * x + dir * rad * 0.38, c + side * x - dir * rad * 0.38], Stroke::new(2.6, col));
        if let Some(to) = o.to.filter(|_| !o.loc) {
            let t = c + dir * if to { rad * 0.24 } else { -rad * 0.24 };
            let d = if to { dir } else { -dir };
            pc.add(Shape::convex_polygon(vec![t + d * 2.4 * u, t + side * 1.8 * u, t - side * 1.8 * u], WHITE, Stroke::NONE));
        }
    } else {
        let flag = if ils { "LOC" } else { "VOR" };
        txt(&pc, c + Vec2::new(0.0, -rad * 0.6), Align2::CENTER_CENTER, flag, 3.4 * u, pal::RED);
    }
    // glideslope (right)
    if ils {
        let gx = s.max.x - 6.0 * u;
        for k in [-2.0f32, -1.0, 1.0, 2.0] {
            pc.circle_stroke(Pos2::new(gx, c.y + k * dot), 0.8 * u, Stroke::new(1.3, WHITE));
        }
        pc.line_segment([Pos2::new(gx - 2.0 * u, c.y), Pos2::new(gx + 2.0 * u, c.y)], Stroke::new(2.0, YELLOW));
        if o.loc && o.gs_valid {
            let y = c.y - (o.gs * 2.0).clamp(-2.3, 2.3) * dot;
            let (w, h) = (1.6 * u, 1.3 * u);
            pc.add(Shape::convex_polygon(vec![Pos2::new(gx - w, y), Pos2::new(gx, y - h), Pos2::new(gx + w, y), Pos2::new(gx, y + h)], MAG, Stroke::NONE));
        } else if o.loc && o.valid {
            txt(&pc, Pos2::new(gx, c.y - 3.0 * dot), Align2::CENTER_CENTER, "G/S", 2.6 * u, pal::RED);
        }
    }
    // ADF needle (green, single)
    let adf = &sim.nav.radios.adf.out;
    if adf.valid {
        let a = wrap_pi(adf.bearing - hdg);
        let d = Vec2::new(a.sin(), -a.cos());
        pc.line_segment([c + d * rad * 0.7, c + d * (rad - 1.0 * u)], Stroke::new(2.0, GREEN));
        pc.line_segment([c - d * rad * 0.7, c - d * (rad - 1.0 * u)], Stroke::new(2.0, GREEN));
    }
    aircraft_mark(&pc, c, u);
    // top right: receiver, course, ident, DME
    let tr = Pos2::new(s.max.x - 2.0 * u, s.min.y + 2.4 * u);
    pc.text(tr, Align2::RIGHT_TOP, format!("{}1 {}", if ils { "ILS" } else { "VOR" }, crate::nav::db::freq_label(rx.active, false)), fib(2.8 * u), WHITE);
    pc.text(tr + Vec2::new(0.0, 4.0 * u), Align2::RIGHT_TOP, format!("CRS {:03.0}°", course.to_degrees().round().rem_euclid(360.0)), fdb(2.8 * u), col);
    if o.valid {
        pc.text(tr + Vec2::new(0.0, 8.0 * u), Align2::RIGHT_TOP, &o.ident, fib(2.8 * u), col);
    }
    // top left: GS / TAS
    let tl = s.min + Vec2::new(2.0 * u, 2.4 * u);
    let v = vec2(ac.vel.x, ac.vel.z);
    pc.text(tl, Align2::LEFT_TOP, "GS", fi(2.6 * u), WHITE);
    pc.text(tl + Vec2::new(4.6 * u, -0.3 * u), Align2::LEFT_TOP, format!("{:.0}", v.length() / KT), fdb(3.2 * u), GREEN);
    pc.text(tl + Vec2::new(13.0 * u, 0.0), Align2::LEFT_TOP, "TAS", fi(2.6 * u), WHITE);
    pc.text(tl + Vec2::new(19.0 * u, -0.3 * u), Align2::LEFT_TOP, format!("{:.0}", ac.tas() / KT), fdb(3.2 * u), GREEN);
    if let Some(d) = o.dme {
        pc.text(Pos2::new(s.min.x + 2.0 * u, s.max.y - 10.0 * u), Align2::LEFT_TOP, "DME", fi(2.6 * u), WHITE);
        pc.text(Pos2::new(s.min.x + 2.0 * u, s.max.y - 6.4 * u), Align2::LEFT_TOP, format!("{:.1} NM", d / 1852.0), fdb(2.8 * u), GREEN);
    }
    pc.text(Pos2::new(s.max.x - 2.0 * u, s.max.y - 2.0 * u), Align2::RIGHT_BOTTOM, if ils { "ROSE ILS" } else { "ROSE VOR" }, fi(2.6 * u), CYAN);
}

/// PLAN: north up, centred on the TO waypoint, the route and the aircraft.
fn nd_plan(p: &egui::Painter, s: Rect, sim: &Sim, world: &World, route: &[(String, glam::Vec2)]) {
    let pc = p.with_clip_rect(s);
    let u = s.width() / 100.0;
    let ac = &sim.ac;
    let pos = vec2(ac.pos.x, ac.pos.z);
    let range_nm = ND_RANGES_NM[sim.map_zoom.min(ND_RANGES_NM.len() - 1)];
    let rad = 36.0 * u;
    let c = Pos2::new(s.center().x, s.min.y + 54.0 * u);
    let centre = route.first().map(|r| r.1).unwrap_or(pos);
    let view = View { rect: s, anchor: c, centre, s: rad / (range_nm * 0.5 * 1852.0), up: 0.0 };
    pc.circle_stroke(c, rad, Stroke::new(1.3, WHITE));
    pc.circle_stroke(c, rad * 0.5, Stroke::new(1.0, Color32::from_gray(140)));
    for (lab, a) in [("N", 0.0f32), ("E", PI / 2.0), ("S", PI), ("W", -PI / 2.0)] {
        txt(&pc, polar(c, rad + 3.0 * u, a), Align2::CENTER_CENTER, lab, 3.0 * u, WHITE);
    }
    let env = crate::nav::state::WorldEnv(world);
    if sim.bot.is_some() {
        map::route(&pc, &view, pos, route, MAG);
    } else {
        let all = sim.nav.plan.all_world(&env);
        let off = usize::from(sim.nav.plan.origin.is_some());
        map::flight_plan(&pc, &view, pos, &all, sim.nav.plan.active + off, GREEN, Color32::from_gray(110));
    }
    let me = view.to_screen(pos);
    if s.contains(me) {
        let a = ac.heading();
        let d = Vec2::new(a.sin(), -a.cos());
        let n = Vec2::new(-d.y, d.x);
        pc.line_segment([me - d * 2.0 * u, me + d * 4.6 * u], Stroke::new(2.4, YELLOW));
        pc.line_segment([me + d * 2.0 * u - n * 3.4 * u, me + d * 2.0 * u + n * 3.4 * u], Stroke::new(2.4, YELLOW));
    }
    txt(&pc, polar(c, rad * 0.5, -0.8), Align2::RIGHT_CENTER, format!("{}", range_nm / 4.0), 2.6 * u, CYAN);
    nd_corner_info(&pc, s, sim, world, route, u);
    pc.text(Pos2::new(s.max.x - 2.0 * u, s.max.y - 2.0 * u), Align2::RIGHT_BOTTOM, format!("PLAN  {range_nm} NM"), fi(2.6 * u), CYAN);
}

// ---------------------------------------------------------------- engine / warning display

/// "-THR LEVER 1 ....... IDLE" (an ECAM action line, dotted to the value).
fn ecam_line(action: &str, value: &str) -> String {
    if value.is_empty() {
        return format!("-{action}");
    }
    let n = 30usize.saturating_sub(action.chars().count() + value.chars().count() + 2).max(2);
    format!("-{action} {} {value}", ".".repeat(n))
}

pub fn ewd(p: &egui::Painter, r: Rect, sim: &Sim) {
    let s = housing(p, r);
    if !sim.ac.sys.display_on("ewd") {
        dark(p, s);
        return;
    }
    let p = &p.with_clip_rect(s);
    let u = s.width() / 100.0;
    let def = sim.def();
    let ac = &sim.ac;
    let n = def.engine_count.clamp(1, 4);
    let jet = def.engine_kind == EngineKind::Jet;
    let col_w = s.width() / n as f32;
    let dial_r = (col_w * 0.36).min(19.0 * u);
    let top = s.min.y + 7.0 * u + dial_r;
    txt(p, Pos2::new(s.min.x + 3.0 * u, s.min.y + 3.4 * u), Align2::LEFT_CENTER, if jet { "N1" } else { "TQ" }, 3.0 * u, WHITE);
    txt(p, Pos2::new(s.max.x - 3.0 * u, s.min.y + 3.4 * u), Align2::RIGHT_CENTER, "%", 3.0 * u, CYAN);
    let mut rows_y = top + dial_r + 4.0 * u;
    for i in 0..n {
        let c = Pos2::new(s.min.x + col_w * (i as f32 + 0.5), top);
        let running = ac.engine_running(i);
        let (val, max, red) = if jet {
            (ac.n1(i) * 100.0, 110.0, 104.0)
        } else {
            let np = ac.engine_rpm_i(i).max(1.0);
            let tq = ac.shaft_power(i) / def.engine_power_w.max(1.0) * (def.rpm_static.max(1.0) / np) * 100.0;
            (if running { tq.clamp(0.0, 150.0) } else { 0.0 }, 120.0, 100.0)
        };
        // Airbus-style 200° arc from 7:30 to 2:30.
        let a0 = -1.75f32;
        let span = 3.5f32;
        let ang = |v: f32| a0 + (v.clamp(0.0, max) / max) * span;
        if val > 0.5 {
            super::ring_sector(p, c, dial_r * 0.18, dial_r - 1.0, ang(0.0), ang(val), Color32::from_rgba_unmultiplied(64, 222, 134, 36));
        }
        super::arc(p, c, dial_r, ang(0.0), ang(red), 2.0, WHITE);
        super::arc(p, c, dial_r, ang(red), ang(max), 3.0, pal::RED);
        for k in [0.0, 20.0, 40.0, 60.0, 80.0, 100.0] {
            if k <= max {
                p.line_segment([polar(c, dial_r, ang(k)), polar(c, dial_r - 1.8 * u, ang(k))], Stroke::new(1.6, WHITE));
            }
            if k == 50.0 || k == 100.0 {
                txt(p, polar(c, dial_r - 4.6 * u, ang(k)), Align2::CENTER_CENTER, format!("{}", k as i32 / 10), 2.4 * u, WHITE);
            }
        }
        // commanded thrust (throttle) as a small cyan circle on the arc
        if def.has_engine {
            let t = sim.ctl.lever(i) * if jet { 100.0 } else { 100.0 };
            p.circle_stroke(polar(c, dial_r + 1.4 * u, ang(t)), 0.9 * u, Stroke::new(1.4, CYAN));
        }
        let col = if !running { AMBER } else if val > red { pal::RED } else { GREEN };
        p.line_segment([c, polar(c, dial_r + 0.6 * u, ang(val))], Stroke::new(2.6, col));
        p.circle_filled(c, 0.9 * u, col);
        let rb = Rect::from_min_size(c + Vec2::new(-1.0 * u, 1.6 * u), Vec2::new(dial_r + 1.0 * u, 5.4 * u));
        p.rect_stroke(rb, 2, Stroke::new(1.0, Color32::from_gray(110)), egui::StrokeKind::Inside);
        digits(p, rb.right_center() - Vec2::new(0.8 * u, 0.0), Align2::RIGHT_CENTER, format!("{val:.1}"), 3.2 * u, col);
        if !running {
            txt(p, c + Vec2::new(0.0, -dial_r * 0.4), Align2::CENTER_CENTER, "FAIL", 3.0 * u, AMBER);
        }
        if def.has_afterburner && ac.afterburner() > 0.02 {
            txt(p, Pos2::new(c.x, c.y - dial_r * 0.45), Align2::CENTER_CENTER, "A/B", 2.8 * u, AMBER);
        }
        // rows: EGT / NP, FF
        let egt = ac.egt(i);
        let ecol = if egt > def.egt_max + 15.0 { pal::RED } else { GREEN };
        let v1 = if jet { format!("{egt:.0}") } else { format!("{:.0}", ac.engine_rpm_i(i)) };
        digits(p, Pos2::new(c.x, rows_y), Align2::CENTER_CENTER, v1, 3.2 * u, if jet { ecol } else { GREEN });
        let ff = ac.fuel_flow(i);
        digits(p, Pos2::new(c.x, rows_y + 6.0 * u), Align2::CENTER_CENTER, format!("{:.0}", (ff / 10.0).round() * 10.0), 3.2 * u, GREEN);
    }
    // row labels in the middle (or at the left for one engine)
    let lx = if n > 1 { s.center().x } else { s.min.x + 5.0 * u };
    let la = if n > 1 { Align2::CENTER_CENTER } else { Align2::LEFT_CENTER };
    txt(p, Pos2::new(lx, rows_y), la, if jet { "EGT °C" } else { "NP RPM" }, 2.5 * u, CYAN);
    txt(p, Pos2::new(lx, rows_y + 6.0 * u), la, "FF KG/H", 2.5 * u, CYAN);
    rows_y += 12.0 * u;
    p.line_segment([Pos2::new(s.min.x + 3.0 * u, rows_y), Pos2::new(s.max.x - 3.0 * u, rows_y)], Stroke::new(1.0, Color32::from_gray(60)));
    // flaps: slat / flap track with the lever target
    let fy = rows_y + 7.0 * u;
    let flaps = ac.surfaces().flaps.clamp(0.0, 1.0);
    txt(p, Pos2::new(s.min.x + 4.0 * u, fy), Align2::LEFT_CENTER, if sim.def().flap_label == "Nose" { "NOSE" } else { "FLAP" }, 3.0 * u, WHITE);
    let track = [Pos2::new(s.min.x + 22.0 * u, fy - 1.5 * u), Pos2::new(s.max.x - 20.0 * u, fy + 2.5 * u)];
    let d = track[1] - track[0];
    for k in 0..=4 {
        let q = track[0] + d * (k as f32 / 4.0);
        p.circle_filled(q, 0.6 * u, Color32::from_gray(120));
    }
    let q = track[0] + d * flaps;
    p.add(Shape::convex_polygon(vec![q + Vec2::new(-1.6 * u, -1.0 * u), q + Vec2::new(1.6 * u, 0.0), q + Vec2::new(-1.6 * u, 1.0 * u)], GREEN, Stroke::NONE));
    let tq = track[0] + d * sim.ctl.flaps.clamp(0.0, 1.0);
    p.circle_stroke(tq, 1.4 * u, Stroke::new(1.4, CYAN));
    txt(p, Pos2::new(s.max.x - 4.0 * u, fy), Align2::RIGHT_CENTER, sim.flap_label(), 3.0 * u, GREEN);
    // gear + speed brake
    let gy = fy + 9.0 * u;
    if def.retractable_gear {
        let g = ac.gear_pos();
        let col = if g > 0.995 { GREEN } else if g > 0.005 { pal::RED } else { Color32::from_gray(70) };
        txt(p, Pos2::new(s.min.x + 4.0 * u, gy), Align2::LEFT_CENTER, "GEAR", 3.0 * u, WHITE);
        for k in 0..3 {
            let c = Pos2::new(s.min.x + 23.0 * u + k as f32 * 7.0 * u, gy);
            p.add(Shape::convex_polygon(vec![c + Vec2::new(0.0, 1.8 * u), c + Vec2::new(-2.2 * u, -1.6 * u), c + Vec2::new(2.2 * u, -1.6 * u)], col, Stroke::NONE));
        }
        txt(p, Pos2::new(s.min.x + 46.0 * u, gy), Align2::LEFT_CENTER, if g > 0.995 { "DN" } else if g > 0.005 { "TRANSIT" } else { "UP" }, 2.8 * u, col);
    }
    if def.has_spoilers {
        let sp = ac.surfaces().spoilers;
        txt(p, Pos2::new(s.max.x - 4.0 * u, gy), Align2::RIGHT_CENTER, if sp > 0.05 { format!("SPD BRK {:.0}%", sp * 100.0) } else { "SPD BRK".into() }, 2.8 * u, if sp > 0.05 { AMBER } else { Color32::from_gray(90) });
    }
    // memo
    let my = gy + 6.0 * u;
    p.line_segment([Pos2::new(s.min.x + 3.0 * u, my), Pos2::new(s.max.x - 3.0 * u, my)], Stroke::new(1.0, Color32::from_gray(60)));
    let mut memo: Vec<(String, Color32)> = Vec::new();
    if ac.stall_warning() {
        memo.push(("STALL".into(), pal::RED));
    }
    // ECAM / EICAS: warnings (red), cautions (amber) with the procedure
    // lines still to do (cyan), advisories (systems/alerts.rs).
    let alerts = crate::systems::alerts::alerts(&ac.sys, def, ac, &sim.ctl);
    for a in &alerts {
        use crate::systems::alerts::Level;
        let col = match a.level {
            Level::Warning => pal::RED,
            Level::Caution => AMBER,
            Level::Advisory => GREEN,
        };
        memo.push((a.title.clone(), col));
        if a.level != Level::Advisory {
            for l in a.lines.iter().filter(|l| !l.done).take(4) {
                memo.push((ecam_line(&l.action, &l.value), CYAN));
            }
        }
    }
    for i in 0..def.engine_count {
        let tag = format!("ENG {}", i + 1);
        let listed = alerts.iter().any(|a| a.title.starts_with(&tag) || (def.engine_count == 1 && a.title.starts_with("ENG")));
        if !ac.engine_running(i) && def.has_engine && !listed && (sim.ctl.failed_engines & (1 << i) != 0 || !ac.sys.init) {
            memo.push((format!("ENG {} FAIL", i + 1), AMBER));
        }
    }
    if sim.ctl.reverse || ac.surfaces().reverser > 0.1 {
        memo.push(("REV".into(), GREEN));
    }
    if ac.on_ground && sim.ctl.brakes > 0.5 {
        memo.push(("BRAKES".into(), GREEN));
    }
    // Ground handling (src/ground): parking brake, autobrake, ground services.
    if sim.ctl.parking_brake {
        memo.push(("PARK BRK".into(), if ac.on_ground { GREEN } else { AMBER }));
    }
    if sim.ctl.autobrake != crate::ground::Autobrake::Off {
        let decel = ac.ground.ab.active;
        memo.push((format!("AUTO BRK {}{}", sim.ctl.autobrake.label(), if decel { "  DECEL" } else { "" }), GREEN));
    }
    if sim.ground.gpu || sim.ground.chocks {
        memo.push((if sim.ground.gpu { "GND PWR · CHOCKS".into() } else { "CHOCKS".into() }, GREEN));
    }
    if sim.ground.push.is_some() {
        memo.push(("PUSHBACK · NWS DISC".into(), GREEN));
    }
    if sim.bot.is_some() {
        memo.push(("AUTO FLT".into(), MAG));
    }
    if sim.ap.engaged {
        memo.push((if sim.ap.ias.is_some() { "AP · A/THR".into() } else { "AP".into() }, GREEN));
    }
    if sim.paused {
        memo.push(("PAUSED".into(), WHITE));
    }
    if memo.is_empty() {
        memo.push(("NORMAL".into(), GREEN));
    }
    let rows = ((s.max.y - my - 3.0 * u) / (4.6 * u)).floor().max(1.0) as usize;
    for (i, (m, col)) in memo.iter().take(rows).enumerate() {
        txt(p, Pos2::new(s.min.x + 4.0 * u, my + 4.4 * u + i as f32 * 4.6 * u), Align2::LEFT_CENTER, m, 3.0 * u, *col);
    }
    let _ = (gradient_rect, CornerRadius::ZERO);
}

/// PFD Mach readout: Airbus-style ".78" below Mach 1, "1.99" from Mach 1 up
/// (Concorde, the F-16).
pub fn mach_readout(m: f32) -> String {
    let h = (m.max(0.0) * 100.0).round();
    if h >= 100.0 { format!("{:.2}", h / 100.0) } else { format!(".{h:02.0}") }
}

#[cfg(test)]
mod mach_tests {
    #[test]
    fn mach_readout_keeps_the_units_digit_above_mach_1() {
        assert_eq!(super::mach_readout(0.78), ".78");
        assert_eq!(super::mach_readout(0.05), ".05");
        assert_eq!(super::mach_readout(0.996), "1.00");
        assert_eq!(super::mach_readout(1.99), "1.99");
        assert_eq!(super::mach_readout(2.04), "2.04");
    }
}
