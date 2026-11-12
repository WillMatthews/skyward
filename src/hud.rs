//! Instrument panel, minimap and overlays (egui painter).

use crate::gfx::RenderStats;
use crate::osm::TileId;
use crate::flight::{Category, EngineKind};
use crate::sim::{Sim, Start};
use crate::world::World;
use egui::{epaint::PathStroke, Align2, Color32, FontId, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use std::collections::HashMap;
use std::f32::consts::PI;

/// Per-tile moving-map textures (placed with the tile's rect in the current frame).
pub type MapTiles = HashMap<TileId, egui::TextureHandle>;

pub const ATTRIBUTION: &str = "© OpenMapTiles © OpenStreetMap contributors";

const KT: f32 = 1.943_844;
const FT: f32 = 3.280_84;

fn pt(c: Pos2, r: f32, a: f32) -> Pos2 {
    Pos2::new(c.x + r * a.sin(), c.y - r * a.cos())
}

fn arc(p: &egui::Painter, c: Pos2, r: f32, a0: f32, a1: f32, w: f32, col: Color32) {
    let n = ((a1 - a0).abs() * 30.0).ceil().max(2.0) as usize;
    let pts: Vec<Pos2> = (0..=n).map(|i| pt(c, r, a0 + (a1 - a0) * i as f32 / n as f32)).collect();
    p.add(Shape::line(pts, PathStroke::new(w, col)));
}

fn bezel(p: &egui::Painter, c: Pos2, r: f32) {
    p.circle_filled(c, r + 5.0, Color32::from_rgb(28, 28, 30));
    p.circle_stroke(c, r + 5.0, Stroke::new(2.0, Color32::from_rgb(70, 70, 74)));
    p.circle_filled(c, r, Color32::from_rgb(14, 14, 16));
}

fn needle(p: &egui::Painter, c: Pos2, len: f32, a: f32, w: f32, col: Color32) {
    let tip = pt(c, len, a);
    let tail = pt(c, -len * 0.18, a);
    let side = EVec2::new(a.cos(), a.sin()) * w * 0.5;
    p.add(Shape::convex_polygon(vec![tail + side, tip, tail - side], col, Stroke::NONE));
    p.circle_filled(c, w * 0.9, Color32::from_rgb(40, 40, 40));
}

fn ticks(p: &egui::Painter, c: Pos2, r: f32, a0: f32, a1: f32, n: usize, len: f32, w: f32) {
    for i in 0..=n {
        let a = a0 + (a1 - a0) * i as f32 / n as f32;
        p.line_segment([pt(c, r, a), pt(c, r - len, a)], Stroke::new(w, Color32::WHITE));
    }
}

fn label(p: &egui::Painter, pos: Pos2, s: &str, size: f32, col: Color32) {
    p.text(pos, Align2::CENTER_CENTER, s, FontId::proportional(size), col);
}

fn mono(p: &egui::Painter, pos: Pos2, align: Align2, s: &str, size: f32, col: Color32) {
    p.text(pos, align, s, FontId::monospace(size), col);
}

fn airspeed(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    let def = sim.def();
    let max = (def.vne_kt * 1.15 / 20.0).ceil() * 20.0;
    let ang = |kt: f32| (kt.clamp(0.0, max) / max) * 1.8 * PI - 0.0;
    // Arcs
    arc(p, c, r - 4.0, ang(def.vs0_kt), ang(def.vs1_kt * 1.9), 4.0, Color32::WHITE);
    arc(p, c, r - 9.0, ang(def.vs1_kt), ang(def.vno_kt), 5.0, Color32::from_rgb(40, 190, 70));
    arc(p, c, r - 9.0, ang(def.vno_kt), ang(def.vne_kt), 5.0, Color32::from_rgb(230, 200, 40));
    p.line_segment([pt(c, r, ang(def.vne_kt)), pt(c, r - 16.0, ang(def.vne_kt))], Stroke::new(3.0, Color32::RED));
    let step = if max > 400.0 { 50.0 } else if max > 200.0 { 20.0 } else { 10.0 };
    let mut kt = 0.0;
    while kt <= max {
        let major = (kt / (step * 2.0)).fract().abs() < 1e-3;
        let a = ang(kt);
        p.line_segment([pt(c, r - 2.0, a), pt(c, r - if major { 14.0 } else { 8.0 }, a)], Stroke::new(1.5, Color32::WHITE));
        if major && kt > 0.0 {
            label(p, pt(c, r - 26.0, a), &format!("{kt:.0}"), r * 0.15, Color32::WHITE);
        }
        kt += step;
    }
    // V-speed bugs (powered aircraft): Vr cyan, Vref green; AP speed bug magenta.
    let bug = |kt: f32, col: Color32, txt: &str| {
        let a = ang(kt);
        p.add(Shape::convex_polygon(vec![pt(c, r + 1.0, a - 0.035), pt(c, r + 1.0, a + 0.035), pt(c, r - 7.0, a)], col, Stroke::NONE));
        let _ = txt;
    };
    if def.has_engine && def.category != Category::Trainer {
        bug(def.vr_kt, Color32::from_rgb(90, 200, 255), "R");
        bug(def.vref_kt, Color32::from_rgb(90, 240, 120), "REF");
    }
    if let Some(v) = sim.ap.ias.filter(|_| sim.ap.engaged) {
        bug(v * KT, Color32::from_rgb(255, 80, 255), "");
    }
    let ias = sim.ac.airspeed() * KT;
    label(p, c + EVec2::new(0.0, r * 0.32), "KNOTS", r * 0.11, Color32::GRAY);
    mono(p, c + EVec2::new(0.0, r * 0.52), Align2::CENTER_CENTER, &format!("{ias:3.0}"), r * 0.2, Color32::WHITE);
    if sim.ac.mach() > 0.4 {
        mono(p, c + EVec2::new(0.0, -r * 0.3), Align2::CENTER_CENTER, &format!("M{:.2}", sim.ac.mach()), r * 0.16, Color32::from_rgb(255, 220, 120));
    }
    needle(p, c, r * 0.85, ang(ias), 6.0, Color32::WHITE);
}

/// Piston RPM (tachometer) with manifold pressure.
fn tach(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    let def = sim.def();
    let rs = def.rpm_static.max(1000.0);
    let max = ((rs * 1.25) / 500.0).ceil() * 500.0;
    let ang = |v: f32| -0.75 * PI + (v.clamp(0.0, max) / max) * 1.5 * PI;
    arc(p, c, r - 8.0, ang(rs * 0.85), ang(rs * 1.12), 5.0, Color32::from_rgb(40, 190, 70));
    p.line_segment([pt(c, r, ang(rs * 1.15)), pt(c, r - 16.0, ang(rs * 1.15))], Stroke::new(3.0, Color32::RED));
    let n = (max / 100.0) as i32;
    for i in 0..=n {
        let v = i as f32 * 100.0;
        let major = i % 5 == 0;
        p.line_segment([pt(c, r - 2.0, ang(v)), pt(c, r - if major { 13.0 } else { 6.0 }, ang(v))], Stroke::new(if major { 1.5 } else { 1.0 }, Color32::WHITE));
        if major {
            label(p, pt(c, r - 25.0, ang(v)), &format!("{}", i / 5 * 5), r * 0.14, Color32::WHITE);
        }
    }
    label(p, c + EVec2::new(0.0, -r * 0.3), "RPM x100", r * 0.1, Color32::GRAY);
    let rpm = sim.ac.engine_rpm();
    mono(p, c + EVec2::new(0.0, r * 0.38), Align2::CENTER_CENTER, &format!("{rpm:4.0}"), r * 0.16, Color32::WHITE);
    // Manifold pressure (inHg): ambient → boosted for supercharged engines.
    let amb = 29.92 * crate::flight::isa(sim.ac.pos.y).pressure / 101_325.0;
    let pw = sim.ac.n1(0).clamp(0.0, 1.0);
    let mp = if def.crit_alt_m > 500.0 { 12.0 + (61.0 - 12.0) * pw } else { amb * (0.32 + 0.68 * pw) };
    let col = if sim.ac.engine_running(0) { Color32::from_gray(200) } else { Color32::from_rgb(255, 80, 60) };
    mono(p, c + EVec2::new(0.0, r * 0.62), Align2::CENTER_CENTER, &format!("MP {mp:4.1}"), r * 0.13, col);
    needle(p, c, r * 0.85, ang(rpm), 6.0, Color32::WHITE);
}

/// One turbine engine dial: jets N1 % (+ EGT), turboprops torque % (+ Np, ITT).
fn turbine(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim, i: usize) {
    bezel(p, c, r);
    let def = sim.def();
    let ac = &sim.ac;
    let jet = def.engine_kind == EngineKind::Jet;
    let running = ac.engine_running(i);
    let (val, max, redl, title) = if jet {
        (ac.n1(i) * 100.0, 110.0, 104.0, "N1 %")
    } else {
        // Torque: shaft power over prop speed, % of the rating at governed Np.
        let np = ac.engine_rpm_i(i).max(1.0);
        let tq = ac.shaft_power(i) / def.engine_power_w.max(1.0) * (def.rpm_static.max(1.0) / np) * 100.0;
        (if running { tq.clamp(0.0, 150.0) } else { 0.0 }, 120.0, 100.0, "TQ %")
    };
    let a0 = -0.75 * PI;
    let ang = |v: f32| a0 + (v.clamp(0.0, max) / max) * 1.5 * PI;
    arc(p, c, r - 6.0, ang(0.0), ang(redl), 3.0, Color32::from_gray(150));
    p.line_segment([pt(c, r, ang(redl)), pt(c, r - 12.0, ang(redl))], Stroke::new(3.0, Color32::RED));
    for k in 0..=((max / 10.0) as i32) {
        let v = k as f32 * 10.0;
        let major = k % 2 == 0;
        p.line_segment([pt(c, r - 2.0, ang(v)), pt(c, r - if major { 10.0 } else { 5.0 }, ang(v))], Stroke::new(1.2, Color32::WHITE));
        if major && r > 40.0 {
            label(p, pt(c, r - 19.0, ang(v)), &format!("{}", k), r * 0.13, Color32::from_gray(210));
        }
    }
    // Filled sector up to the value (Airbus-style).
    let n = 24;
    let mut sector = vec![c];
    for k in 0..=n {
        sector.push(pt(c, r * 0.62, ang(val * k as f32 / n as f32)));
    }
    if val > 1.0 {
        p.add(Shape::convex_polygon(sector.clone(), Color32::from_rgba_unmultiplied(60, 160, 90, 90), Stroke::NONE));
    }
    let col = if !running {
        Color32::from_rgb(255, 80, 60)
    } else if val > redl {
        Color32::from_rgb(255, 90, 60)
    } else {
        Color32::WHITE
    };
    // "N1 %" on a single engine; "ENG 2  N1 %" only when there are several.
    let name = if def.engine_count > 1 { format!("ENG {}  {title}", i + 1) } else { title.to_string() };
    label(p, c + EVec2::new(0.0, -r * 0.34), &name, r * 0.12, Color32::GRAY);
    mono(p, c + EVec2::new(0.0, r * 0.3), Align2::CENTER_CENTER, &format!("{val:5.1}"), r * 0.2, col);
    let egt = ac.egt(i);
    let egt_col = if egt > def.egt_max + 15.0 { Color32::from_rgb(255, 90, 60) } else { Color32::from_rgb(120, 220, 140) };
    let sec = if jet { format!("EGT {egt:3.0}") } else { format!("NP {:4.0}", ac.engine_rpm_i(i)) };
    mono(p, c + EVec2::new(0.0, r * 0.56), Align2::CENTER_CENTER, &sec, r * 0.14, if jet { egt_col } else { Color32::from_gray(200) });
    if !jet {
        mono(p, c + EVec2::new(0.0, r * 0.76), Align2::CENTER_CENTER, &format!("ITT {egt:3.0}"), r * 0.12, egt_col);
    }
    needle(p, c, r * 0.82, ang(val), 4.0, col);
    if !running {
        label(p, c + EVec2::new(0.0, -r * 0.08), "FAIL", r * 0.2, Color32::from_rgb(255, 80, 60));
    }
    if def.has_afterburner && ac.afterburner() > 0.02 {
        let rr = Rect::from_center_size(c + EVec2::new(0.0, -r * 0.62), EVec2::new(r * 0.62, r * 0.24));
        p.rect_filled(rr, 3.0, Color32::from_rgb(255, 140, 30));
        label(p, rr.center(), &format!("AB {:.0}%", ac.afterburner() * 100.0), r * 0.13, Color32::BLACK);
    }
}

/// Glider: g-meter with height above ground.
fn g_meter(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    label(p, c + EVec2::new(0.0, -r * 0.35), "G", r * 0.18, Color32::GRAY);
    mono(p, c, Align2::CENTER_CENTER, &format!("{:+.1}", sim.ac.g_load()), r * 0.3, Color32::WHITE);
    mono(p, c + EVec2::new(0.0, r * 0.45), Align2::CENTER_CENTER, &format!("{:.0} m", sim.ac.pos.y), r * 0.15, Color32::LIGHT_GRAY);
}

fn bar(p: &egui::Painter, rect: Rect, frac: f32, col: Color32, text: &str) {
    p.rect_filled(rect, 3.0, Color32::from_rgb(20, 20, 22));
    let h = rect.height() * frac.clamp(0.0, 1.0);
    p.rect_filled(Rect::from_min_max(Pos2::new(rect.min.x, rect.max.y - h), rect.max), 3.0, col);
    p.rect_stroke(rect, 3.0, Stroke::new(1.0, Color32::from_gray(90)), egui::StrokeKind::Inside);
    label(p, Pos2::new(rect.center().x, rect.max.y + 10.0), text, 11.0, Color32::LIGHT_GRAY);
}

/// Three gear lights (nose, left, right): green down and locked, red in transit, dark when up.
fn gear_lights(p: &egui::Painter, c: Pos2, sim: &Sim) {
    let g = sim.ac.gear_pos();
    let (col, txt) = if g > 0.995 {
        (Color32::from_rgb(60, 230, 90), Color32::BLACK)
    } else if g > 0.005 {
        (Color32::from_rgb(240, 60, 50), Color32::WHITE)
    } else {
        (Color32::from_rgb(34, 34, 36), Color32::from_gray(90))
    };
    let lever_mismatch = sim.ctl.gear_down != (g > 0.5) || (0.005..0.995).contains(&g);
    for (o, t) in [(EVec2::new(0.0, -13.0), "N"), (EVec2::new(-13.0, 9.0), "L"), (EVec2::new(13.0, 9.0), "R")] {
        p.circle_filled(c + o, 8.0, col);
        p.circle_stroke(c + o, 8.0, Stroke::new(1.0, Color32::from_gray(80)));
        label(p, c + o, t, 9.0, txt);
    }
    label(p, c + EVec2::new(0.0, 28.0), if sim.ctl.gear_down { "GEAR DN" } else { "GEAR UP" }, 10.5, if lever_mismatch { Color32::from_rgb(255, 180, 60) } else { Color32::LIGHT_GRAY });
}

fn attitude(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    let pitch = sim.ac.pitch().to_degrees();
    let roll = sim.ac.roll();
    let k = r / 30.0; // px per degree
    let up = EVec2::new(-roll.sin(), -roll.cos());
    let along = EVec2::new(roll.cos(), -roll.sin());
    let h0 = c - up * (pitch * k);
    let n = 72;
    let circle: Vec<Pos2> = (0..n).map(|i| pt(c, r, i as f32 / n as f32 * 2.0 * PI)).collect();
    p.add(Shape::convex_polygon(circle.clone(), Color32::from_rgb(40, 110, 190), Stroke::NONE));
    // ground = circle ∩ half-plane below the horizon
    let below = |q: Pos2| (q - h0).dot(up) < 0.0;
    let mut ground = Vec::new();
    for i in 0..n {
        let a = circle[i];
        let b = circle[(i + 1) % n];
        let (ia, ib) = (below(a), below(b));
        if ia {
            ground.push(a);
        }
        if ia != ib {
            let da = (a - h0).dot(up);
            let db = (b - h0).dot(up);
            let t = da / (da - db);
            ground.push(a + (b - a) * t);
        }
    }
    if ground.len() >= 3 {
        p.add(Shape::convex_polygon(ground, Color32::from_rgb(120, 78, 40), Stroke::NONE));
    }
    // Horizon line and pitch ladder, clipped to the inner circle.
    let clip = |a: Pos2, b: Pos2| -> Option<[Pos2; 2]> {
        let rr = r * 0.92;
        ((a - c).length() < rr && (b - c).length() < rr).then_some([a, b])
    };
    for i in -6..=6 {
        let deg = i as f32 * 5.0;
        let centre = h0 + up * (deg * k);
        let half = if i == 0 { r } else if i % 2 == 0 { r * 0.28 } else { r * 0.14 };
        let (a, b) = (centre - along * half, centre + along * half);
        if i == 0 {
            // horizon: clip by intersecting with circle
            let pts: Vec<Pos2> = (0..=40)
                .map(|j| centre + along * (-r + 2.0 * r * j as f32 / 40.0))
                .filter(|q| (*q - c).length() < r)
                .collect();
            if pts.len() >= 2 {
                p.line_segment([pts[0], *pts.last().unwrap()], Stroke::new(2.0, Color32::WHITE));
            }
        } else if let Some(seg) = clip(a, b) {
            p.line_segment(seg, Stroke::new(1.5, Color32::WHITE));
            if i % 2 == 0 {
                let t = centre + along * (half + 10.0);
                if (t - c).length() < r * 0.9 {
                    label(p, t, &format!("{:.0}", deg.abs()), r * 0.11, Color32::WHITE);
                }
            }
        }
    }
    // Bank scale (fixed) and pointer (rotates with roll).
    for d in [-60.0f32, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0] {
        let a = d.to_radians();
        let len = if d.abs() % 30.0 < 1e-3 { 12.0 } else { 7.0 };
        p.line_segment([pt(c, r - 1.0, a), pt(c, r - len, a)], Stroke::new(2.0, Color32::WHITE));
    }
    let pa = -roll;
    p.add(Shape::convex_polygon(
        vec![pt(c, r - 14.0, pa), pt(c, r - 24.0, pa - 0.08), pt(c, r - 24.0, pa + 0.08)],
        Color32::from_rgb(255, 170, 30),
        Stroke::NONE,
    ));
    // Fixed aircraft symbol.
    let orange = Color32::from_rgb(255, 170, 30);
    let w = r * 0.55;
    p.line_segment([c + EVec2::new(-w, 0.0), c + EVec2::new(-w * 0.35, 0.0)], Stroke::new(4.0, orange));
    p.line_segment([c + EVec2::new(w * 0.35, 0.0), c + EVec2::new(w, 0.0)], Stroke::new(4.0, orange));
    p.line_segment([c + EVec2::new(-w * 0.35, 0.0), c + EVec2::new(0.0, r * 0.12)], Stroke::new(4.0, orange));
    p.line_segment([c + EVec2::new(w * 0.35, 0.0), c + EVec2::new(0.0, r * 0.12)], Stroke::new(4.0, orange));
    p.circle_filled(c, 3.0, orange);
}

fn altimeter(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    ticks(p, c, r - 2.0, 0.0, 2.0 * PI * 49.0 / 50.0, 49, 6.0, 1.0);
    for i in 0..10 {
        let a = i as f32 / 10.0 * 2.0 * PI;
        p.line_segment([pt(c, r - 2.0, a), pt(c, r - 14.0, a)], Stroke::new(2.0, Color32::WHITE));
        label(p, pt(c, r - 26.0, a), &format!("{i}"), r * 0.2, Color32::WHITE);
    }
    let ft = sim.ac.pos.y * FT;
    label(p, c + EVec2::new(0.0, -r * 0.3), "ALT", r * 0.11, Color32::GRAY);
    mono(p, c + EVec2::new(0.0, r * 0.38), Align2::CENTER_CENTER, &format!("{:5.0}", ft), r * 0.18, Color32::WHITE);
    needle(p, c, r * 0.5, (ft / 10000.0) * 2.0 * PI, 4.0, Color32::from_rgb(200, 200, 200));
    needle(p, c, r * 0.62, (ft / 1000.0).rem_euclid(10.0) / 10.0 * 2.0 * PI, 8.0, Color32::WHITE);
    needle(p, c, r * 0.88, (ft / 100.0).rem_euclid(10.0) / 10.0 * 2.0 * PI, 4.0, Color32::WHITE);
}

fn vsi(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    let glider = !sim.def().has_engine;
    // glider: vario in m/s (±5), powered: fpm (±2000)
    let (max, val, unit) = if glider {
        (5.0, sim.ac.vertical_speed(), "m/s")
    } else {
        (2000.0, sim.ac.vertical_speed() * FT * 60.0, "FPM")
    };
    let span = 170f32.to_radians();
    let ang = |v: f32| -PI / 2.0 + (v.clamp(-max, max) / max) * span;
    let n = 10;
    for i in -n..=n {
        let v = max * i as f32 / n as f32;
        let major = i % 2 == 0;
        p.line_segment([pt(c, r - 2.0, ang(v)), pt(c, r - if major { 13.0 } else { 7.0 }, ang(v))], Stroke::new(1.5, Color32::WHITE));
        if major && i != n && i != -n {
            let txt = if glider { format!("{:.0}", v.abs()) } else { format!("{:.0}", v.abs() / 100.0) };
            label(p, pt(c, r - 24.0, ang(v)), &txt, r * 0.14, Color32::WHITE);
        }
    }
    label(p, c + EVec2::new(r * 0.3, 0.0), if glider { "VARIO" } else { "VS" }, r * 0.11, Color32::GRAY);
    label(p, c + EVec2::new(r * 0.3, r * 0.17), unit, r * 0.1, Color32::GRAY);
    if glider {
        let col = if val > 0.2 { Color32::from_rgb(80, 230, 110) } else { Color32::WHITE };
        mono(p, c + EVec2::new(0.0, r * 0.5), Align2::CENTER_CENTER, &format!("{val:+.1}"), r * 0.18, col);
    } else {
        mono(p, c + EVec2::new(0.0, r * 0.5), Align2::CENTER_CENTER, &format!("{val:+5.0}"), r * 0.16, Color32::WHITE);
    }
    needle(p, c, r * 0.85, ang(val), 6.0, Color32::WHITE);
}

fn heading(p: &egui::Painter, c: Pos2, r: f32, sim: &Sim) {
    bezel(p, c, r);
    let hdg = sim.ac.heading();
    for i in 0..72 {
        let deg = i as f32 * 5.0;
        let a = deg.to_radians() - hdg;
        let major = i % 2 == 0;
        p.line_segment([pt(c, r - 2.0, a), pt(c, r - if major { 12.0 } else { 6.0 }, a)], Stroke::new(1.5, Color32::WHITE));
        if i % 6 == 0 {
            let txt = match i / 6 {
                0 => "N".to_string(),
                3 => "E".to_string(),
                6 => "S".to_string(),
                9 => "W".to_string(),
                k => format!("{}", k * 3),
            };
            let col = if i == 0 { Color32::from_rgb(255, 170, 30) } else { Color32::WHITE };
            label(p, pt(c, r - 25.0, a), &txt, r * 0.16, col);
        }
    }
    // lubber line + aircraft silhouette
    p.add(Shape::convex_polygon(
        vec![pt(c, r + 3.0, 0.0) + EVec2::new(-6.0, -6.0), pt(c, r + 3.0, 0.0) + EVec2::new(6.0, -6.0), pt(c, r - 8.0, 0.0)],
        Color32::from_rgb(255, 170, 30),
        Stroke::NONE,
    ));
    let o = Color32::from_rgb(255, 170, 30);
    p.line_segment([c + EVec2::new(0.0, -r * 0.35), c + EVec2::new(0.0, r * 0.3)], Stroke::new(3.0, o));
    p.line_segment([c + EVec2::new(-r * 0.3, -r * 0.05), c + EVec2::new(r * 0.3, -r * 0.05)], Stroke::new(3.0, o));
    p.line_segment([c + EVec2::new(-r * 0.12, r * 0.25), c + EVec2::new(r * 0.12, r * 0.25)], Stroke::new(3.0, o));
    let deg = hdg.to_degrees().rem_euclid(360.0);
    mono(p, c + EVec2::new(0.0, r * 0.52), Align2::CENTER_CENTER, &format!("{:03.0}°", deg), r * 0.16, Color32::WHITE);
}

#[allow(clippy::too_many_arguments)]
pub fn draw(ctx: &egui::Context, sim: &Sim, world: &World, map_tiles: &MapTiles, fps: f32, gpu_name: &str, rs: &RenderStats) {
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::NULL.with("hud")));
    let def = sim.def();

    // ---- instrument panel: 5 flight instruments, the engine cluster, levers, annunciators
    let r = (screen.width() / 24.0).clamp(46.0, 80.0);
    let gap = r * 2.0 + 20.0;
    let n_eng = def.engine_count.max(1);
    let turbine_kind = matches!(def.engine_kind, EngineKind::Jet | EngineKind::Turboprop);
    let re = if turbine_kind && n_eng > 1 { r * 0.78 } else { r };
    let eng_w = if turbine_kind { (re * 2.0 + 14.0) * n_eng as f32 } else { r * 2.0 + 14.0 };
    let n_bars = 3 + def.has_spoilers as usize;
    let lever_w = n_bars as f32 * 36.0 + 12.0;
    let ann_w = 2.0 * 58.0 + if def.retractable_gear { 52.0 } else { 0.0 };
    let panel_w = 16.0 + gap * 5.0 + eng_w + lever_w + ann_w + 20.0;
    let panel = Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - r - 22.0), EVec2::new(panel_w, r * 2.0 + 36.0));
    p.rect_filled(panel, 14.0, Color32::from_rgba_unmultiplied(12, 12, 14, 205));
    let y = panel.center().y;
    let x0 = panel.min.x + 16.0 + r + 5.0;
    let cx = |i: f32| Pos2::new(x0 + gap * i, y);
    airspeed(&p, cx(0.0), r, sim);
    attitude(&p, cx(1.0), r, sim);
    altimeter(&p, cx(2.0), r, sim);
    heading(&p, cx(3.0), r, sim);
    vsi(&p, cx(4.0), r, sim);
    let mut x = x0 + gap * 4.0 + r + 14.0;
    match def.engine_kind {
        EngineKind::None => {
            g_meter(&p, Pos2::new(x + r, y), r, sim);
            x += r * 2.0 + 14.0;
        }
        EngineKind::Piston => {
            tach(&p, Pos2::new(x + r, y), r, sim);
            x += r * 2.0 + 14.0;
        }
        _ => {
            for i in 0..n_eng {
                turbine(&p, Pos2::new(x + re, y), re, sim, i);
                x += re * 2.0 + 14.0;
            }
        }
    }
    // Levers: throttle (with reheat detent / reverse), flaps, spoilers, trim
    let bx = x + 4.0;
    let bh = r * 1.5;
    let top = y - bh * 0.5 - 6.0;
    let mut lx = bx;
    if def.has_engine {
        let rr = Rect::from_min_size(Pos2::new(lx, top), EVec2::new(16.0, bh));
        let (col, txt) = if sim.ctl.reverse {
            (Color32::from_rgb(240, 160, 40), "REV")
        } else if def.has_afterburner && sim.ctl.throttle > 0.9 {
            (Color32::from_rgb(255, 120, 40), "AB")
        } else {
            (Color32::from_rgb(60, 140, 230), "THR")
        };
        bar(&p, rr, sim.ctl.throttle, col, txt);
        if def.has_afterburner {
            // Reheat detent at 90 %.
            let yd = rr.max.y - rr.height() * 0.9;
            p.line_segment([Pos2::new(rr.min.x - 4.0, yd), Pos2::new(rr.max.x + 4.0, yd)], Stroke::new(2.0, Color32::from_rgb(255, 120, 40)));
        }
        if sim.ap.engaged && sim.ap.ias.is_some() {
            label(&p, Pos2::new(rr.center().x, rr.min.y - 8.0), "A/T", 9.5, Color32::from_rgb(120, 230, 140));
        }
        lx += 36.0;
    }
    bar(&p, Rect::from_min_size(Pos2::new(lx, top), EVec2::new(16.0, bh)), sim.ctl.flaps, Color32::from_rgb(230, 230, 230), if def.has_engine { "FLAP" } else { "BRK" });
    lx += 36.0;
    if def.has_spoilers {
        let s = sim.ac.surfaces().spoilers;
        bar(&p, Rect::from_min_size(Pos2::new(lx, top), EVec2::new(16.0, bh)), s.max(sim.ctl.spoilers), Color32::from_rgb(120, 220, 230), if def.category == Category::FastJet { "SB" } else { "SPLR" });
        lx += 36.0;
    }
    // trim: centre-zero bar
    let tr = Rect::from_min_size(Pos2::new(lx, top), EVec2::new(16.0, bh));
    p.rect_filled(tr, 3.0, Color32::from_rgb(20, 20, 22));
    let ty = tr.center().y - sim.ctl.trim * bh * 0.5;
    p.line_segment([Pos2::new(tr.min.x - 3.0, ty), Pos2::new(tr.max.x + 3.0, ty)], Stroke::new(3.0, Color32::from_rgb(80, 220, 120)));
    p.line_segment([Pos2::new(tr.min.x, tr.center().y), Pos2::new(tr.max.x, tr.center().y)], Stroke::new(1.0, Color32::GRAY));
    label(&p, Pos2::new(tr.center().x, tr.max.y + 10.0), if def.control_law == crate::flight::ControlLaw::Airliner { "AUTO" } else { "TRIM" }, 11.0, Color32::LIGHT_GRAY);
    lx += 34.0;
    // gear lights
    if def.retractable_gear {
        gear_lights(&p, Pos2::new(lx + 18.0, y - 8.0), sim);
        lx += 52.0;
    }
    // annunciators (two columns)
    let blink = (sim.time * 4.0).fract() < 0.6;
    let ann = |p: &egui::Painter, col_i: usize, i: usize, on: bool, text: &str, col: Color32| {
        let rr = Rect::from_min_size(Pos2::new(lx + col_i as f32 * 58.0, top + i as f32 * 24.0), EVec2::new(52.0, 19.0));
        p.rect_filled(rr, 3.0, if on { col } else { Color32::from_rgb(30, 30, 32) });
        label(p, rr.center(), text, 10.5, if on { Color32::BLACK } else { Color32::from_gray(90) });
    };
    let amber = Color32::from_rgb(255, 180, 40);
    ann(&p, 0, 0, sim.ac.stall_warning() && blink, "STALL", Color32::from_rgb(255, 60, 50));
    ann(&p, 0, 1, sim.ctl.brakes > 0.5, "BRAKE", amber);
    ann(&p, 0, 2, sim.ac.on_ground, "GND", Color32::from_rgb(120, 200, 255));
    ann(&p, 0, 3, sim.paused, "PAUSE", Color32::from_rgb(230, 230, 230));
    let failed = (0..def.engine_count).any(|i| !sim.ac.engine_running(i)) && def.has_engine;
    let ap_on = sim.ap.engaged || sim.bot.is_some();
    ann(&p, 1, 0, ap_on || (sim.ap_warn > 0.0 && blink), if sim.bot.is_some() { "AUTO" } else { "A/P" }, if ap_on { Color32::from_rgb(90, 230, 120) } else { Color32::from_rgb(255, 60, 50) });
    if def.has_afterburner {
        ann(&p, 1, 1, sim.ac.afterburner() > 0.02, "REHEAT", Color32::from_rgb(255, 130, 40));
    } else {
        ann(&p, 1, 1, sim.ac.surfaces().reverser > 0.1 || sim.ctl.reverse, "REV", amber);
    }
    ann(&p, 1, 2, sim.ac.surfaces().spoilers > 0.05, if def.has_engine { "SPLR" } else { "BRAKES" }, Color32::from_rgb(120, 220, 230));
    ann(&p, 1, 3, failed && blink, "ENG", Color32::from_rgb(255, 60, 50));

    // ---- top-left info
    let mach = if sim.ac.mach() > 0.4 { format!("   M {:.2}", sim.ac.mach()) } else { String::new() };
    let info = format!(
        "{}   ·   {} cam   ·   {}x\nIAS {:>3.0} kt{}   ALT {:>5.0} ft   AoA {:>4.1}°   G {:+.1}\nflaps {}   {:.0} fps   {}",
        def.name,
        sim.cam.name(),
        sim.warp,
        sim.ac.airspeed() * KT,
        mach,
        sim.ac.pos.y * FT,
        sim.ac.alpha().to_degrees(),
        sim.ac.g_load(),
        sim.flap_label(),
        fps,
        gpu_name,
    );
    let galley = p.layout_no_wrap(info, FontId::monospace(13.0), Color32::WHITE);
    let ir = Rect::from_min_size(Pos2::new(12.0, 12.0), galley.size() + EVec2::new(16.0, 12.0));
    p.rect_filled(ir, 8.0, Color32::from_rgba_unmultiplied(10, 10, 12, 170));
    p.galley(Pos2::new(20.0, 18.0), galley, Color32::WHITE);
    let hint = p.layout_no_wrap("H help   C camera   Tab aircraft & start   J auto flight".into(), FontId::proportional(12.0), Color32::from_gray(225));
    let hr = Rect::from_min_size(Pos2::new(12.0, ir.max.y + 6.0), hint.size() + EVec2::new(16.0, 8.0));
    p.rect_filled(hr, 6.0, Color32::from_rgba_unmultiplied(10, 10, 12, 150));
    p.galley(Pos2::new(20.0, ir.max.y + 10.0), hint, Color32::WHITE);

    // ---- top centre: auto-flight / autopilot strip
    let mut sy = 12.0;
    if let Some(b) = &sim.bot {
        let g = p.layout_no_wrap(format!("AUTO FLIGHT   {}", b.status), FontId::proportional(17.0), Color32::WHITE);
        let br = Rect::from_center_size(Pos2::new(screen.center().x, sy + g.size().y * 0.5 + 8.0), g.size() + EVec2::new(28.0, 16.0));
        p.rect_filled(br, 8.0, Color32::from_rgba_unmultiplied(10, 70, 90, 215));
        p.rect_stroke(br, 8.0, Stroke::new(1.5, Color32::from_rgb(90, 220, 255)), egui::StrokeKind::Inside);
        p.galley(br.min + EVec2::new(14.0, 8.0), g, Color32::WHITE);
        sy = br.max.y + 4.0;
        let h2 = p.layout_no_wrap(
            if sim.watch { "watching — any control input or J takes over" } else { "any control input or J takes over" }.into(),
            FontId::proportional(11.5),
            Color32::from_gray(220),
        );
        let hr = Rect::from_min_size(Pos2::new(screen.center().x - h2.size().x * 0.5 - 8.0, sy - 2.0), h2.size() + EVec2::new(16.0, 4.0));
        p.rect_filled(hr, 4.0, Color32::from_rgba_unmultiplied(10, 10, 12, 150));
        p.galley(Pos2::new(screen.center().x - h2.size().x * 0.5, sy), h2, Color32::WHITE);
        sy += 20.0;
    }
    if sim.ap.engaged || sim.ap_warn > 0.0 {
        let ap = &sim.ap;
        let mut s = String::new();
        if ap.engaged {
            s += if sim.bot.is_some() { "AP (auto)" } else { "AP" };
            match ap.hdg {
                Some(h) => s += &format!("   HDG {:03.0}", h.to_degrees().rem_euclid(360.0)),
                None => s += "   WINGS LVL",
            }
            match (ap.alt, ap.vs) {
                (Some(a), _) => s += &format!("   ALT {:>5.0} ft", a * FT),
                (None, Some(v)) => s += &format!("   V/S {:+.0} fpm", v * FT * 60.0),
                _ => s += "   PITCH",
            }
            if let Some(v) = ap.ias {
                s += &format!("   SPD {:.0} kt{}", v * KT, if def.has_engine { "  A/THR" } else { "" });
            }
        }
        let warn = sim.ap_warn > 0.0 && !ap.engaged;
        if warn {
            s = "AP DISCONNECT".into();
        }
        if !warn || blink {
            let g = p.layout_no_wrap(s, FontId::monospace(14.0), if warn { Color32::WHITE } else { Color32::from_rgb(120, 240, 140) });
            let br = Rect::from_center_size(Pos2::new(screen.center().x, sy + g.size().y * 0.5 + 6.0), g.size() + EVec2::new(24.0, 12.0));
            p.rect_filled(br, 6.0, if warn { Color32::from_rgba_unmultiplied(170, 20, 20, 230) } else { Color32::from_rgba_unmultiplied(8, 20, 10, 200) });
            p.galley(br.min + EVec2::new(12.0, 6.0), g, Color32::WHITE);
            sy = br.max.y + 4.0;
        }
    }
    if let Some((msg, t)) = &sim.note {
        let sy = sy.max(screen.height() * 0.16);
        let a = (t.min(1.0) * 230.0) as u8;
        let g = p.layout(msg.clone(), FontId::proportional(15.0), Color32::from_rgba_unmultiplied(255, 255, 255, a.max(30)), 620.0);
        let br = Rect::from_center_size(Pos2::new(screen.center().x, sy + g.size().y * 0.5 + 10.0), g.size() + EVec2::new(24.0, 12.0));
        p.rect_filled(br, 6.0, Color32::from_rgba_unmultiplied(20, 20, 26, (a as f32 * 0.85) as u8));
        p.galley(br.min + EVec2::new(12.0, 6.0), g, Color32::WHITE);
    }

    // ---- moving map (north up, centred on the aircraft)
    if sim.show_map {
        let size = (screen.height() * 0.30).clamp(170.0, 340.0);
        let mr = Rect::from_min_size(Pos2::new(screen.max.x - size - 14.0, 14.0), EVec2::splat(size));
        moving_map(&p, mr, sim, world, map_tiles);
        let line = format!(
            "{}   ·   tiles {} ({} queued)   ·   {:.0} MB",
            ATTRIBUTION,
            rs.tiles,
            world.queued() + world.uploads.len() + world.gpu_pending.len(),
            rs.gpu_bytes as f64 / (1 << 20) as f64
        );
        let g = p.layout_no_wrap(line, FontId::proportional(11.0), Color32::from_gray(235));
        let tr = Rect::from_min_size(Pos2::new(mr.max.x - g.size().x - 8.0, mr.max.y + 6.0), g.size() + EVec2::new(8.0, 4.0));
        p.rect_filled(tr, 4.0, Color32::from_rgba_unmultiplied(10, 10, 12, 170));
        p.galley(tr.min + EVec2::new(4.0, 2.0), g, Color32::WHITE);
    } else {
        p.text(Pos2::new(screen.max.x - 10.0, 10.0), Align2::RIGHT_TOP, ATTRIBUTION, FontId::proportional(11.0), Color32::from_gray(220));
    }

    // ---- banners
    if let Some(reason) = &sim.ac.crashed {
        let br = Rect::from_center_size(screen.center() - EVec2::new(0.0, screen.height() * 0.15), EVec2::new(560.0, 92.0));
        p.rect_filled(br, 12.0, Color32::from_rgba_unmultiplied(120, 10, 10, 220));
        label(&p, br.center() - EVec2::new(0.0, 16.0), &format!("CRASH — {reason}"), 26.0, Color32::WHITE);
        label(&p, br.center() + EVec2::new(0.0, 22.0), "R: try again    Tab: change aircraft/start", 15.0, Color32::from_gray(230));
    } else if sim.ac.stall_warning() && blink {
        label(&p, screen.center() - EVec2::new(0.0, screen.height() * 0.2), "STALL", 40.0, Color32::from_rgb(255, 70, 50));
    } else if def.retractable_gear && sim.ac.gear_pos() < 0.99 && !sim.ac.on_ground && sim.ac.pos.y < 150.0 && sim.ac.vertical_speed() < -1.0 && sim.ac.airspeed() < def.vref_kt * 1.3 / KT && blink {
        label(&p, screen.center() - EVec2::new(0.0, screen.height() * 0.2), "GEAR!", 36.0, Color32::from_rgb(255, 70, 50));
    }
    if sim.paused {
        label(&p, screen.center() - EVec2::new(0.0, screen.height() * 0.3), "PAUSED", 32.0, Color32::WHITE);
    }
    if sim.show_help {
        help_window(ctx, sim);
    }
}

/// Small mouse-driven autopilot panel (shown while the autopilot is engaged and the bot is off).
pub fn ap_panel(ctx: &egui::Context, sim: &mut Sim) {
    if !sim.ap.engaged || sim.bot.is_some() {
        return;
    }
    let has_engine = sim.def().has_engine;
    let vs1 = sim.def().vs1_kt;
    let (hdg_now, alt_now, ias_now) = (sim.ac.heading(), sim.ac.pos.y, sim.ac.airspeed());
    egui::Window::new("Autopilot")
        .anchor(Align2::RIGHT_BOTTOM, [-14.0, -(ctx.content_rect().height() * 0.0 + 190.0)])
        .collapsible(true)
        .resizable(false)
        .default_width(230.0)
        .show(ctx, |ui| {
            let ap = &mut sim.ap;
            egui::Grid::new("ap").num_columns(4).spacing([6.0, 4.0]).show(ui, |ui| {
                // HDG
                let h = ap.hdg.unwrap_or(hdg_now);
                ui.label("HDG");
                if ui.small_button(" - ").clicked() {
                    ap.hdg = Some(crate::bot::wrap_pi(h - 5f32.to_radians()));
                }
                ui.monospace(format!("{:03.0}°", ap.hdg.map_or(f32::NAN, |h| h.to_degrees().rem_euclid(360.0))));
                if ui.small_button(" + ").clicked() {
                    ap.hdg = Some(crate::bot::wrap_pi(h + 5f32.to_radians()));
                }
                ui.end_row();
                // ALT
                let a = ap.alt.unwrap_or(alt_now);
                ui.label("ALT");
                if ui.small_button(" - ").clicked() {
                    ap.alt = Some(((a - 30.48) / 30.48).round() * 30.48);
                    ap.vs = None;
                }
                ui.monospace(match ap.alt {
                    Some(a) => format!("{:5.0} ft", a * FT),
                    None => "  ---  ".into(),
                });
                if ui.small_button(" + ").clicked() {
                    ap.alt = Some(((a + 30.48) / 30.48).round() * 30.48);
                    ap.vs = None;
                }
                ui.end_row();
                // SPD
                let v = ap.ias.unwrap_or(ias_now);
                ui.label("SPD");
                if ui.small_button(" - ").clicked() {
                    ap.ias = Some((v - 5.0 / KT).max(vs1 / KT));
                }
                ui.monospace(match ap.ias {
                    Some(v) => format!("{:3.0} kt", v * KT),
                    None => " ---  ".into(),
                });
                if ui.small_button(" + ").clicked() {
                    ap.ias = Some(v + 5.0 / KT);
                }
                ui.end_row();
                // V/S (only when no altitude target)
                ui.label("V/S");
                if ui.small_button(" - ").clicked() {
                    ap.vs = Some(ap.vs.unwrap_or(0.0) - 500.0 * 0.00508);
                }
                ui.monospace(match ap.vs {
                    Some(v) => format!("{:+5.0} fpm", v / 0.00508),
                    None => "  ---  ".into(),
                });
                if ui.small_button(" + ").clicked() {
                    ap.vs = Some(ap.vs.unwrap_or(0.0) + 500.0 * 0.00508);
                }
                ui.end_row();
            });
            ui.horizontal(|ui| {
                if has_engine {
                    let mut at = ap.ias.is_some();
                    if ui.checkbox(&mut at, "A/THR").changed() {
                        ap.ias = if at { Some(ias_now) } else { None };
                    }
                }
                if ui.button("ALT hold").clicked() {
                    ap.alt = Some(alt_now);
                    ap.vs = None;
                }
                if ui.button("AP off").clicked() {
                    ap.disengage();
                }
            });
            ui.label(egui::RichText::new("Z on/off · Ins/Del hdg · Home/End alt · 9/0 spd").size(10.5).color(Color32::GRAY));
        });
}

fn help_window(ctx: &egui::Context, sim: &Sim) {
    let def = sim.def();
    egui::Window::new("Controls")
        .anchor(Align2::CENTER_CENTER, [0.0, -60.0])
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            egui::Grid::new("keys").striped(true).show(ui, |ui| {
                let rows: Vec<(&str, String)> = vec![
                    ("↑ / W   ↓ / S", "pitch down / up (stick forward / back)".into()),
                    ("← / A   → / D", "roll left / right".into()),
                    ("Q / E", "rudder left / right (steers on ground)".into()),
                    (
                        "Shift / Ctrl  or  = / -",
                        if !def.has_engine {
                            "(no engine)".into()
                        } else if def.has_afterburner {
                            "throttle up / down (above 90 %: afterburner)".into()
                        } else {
                            "throttle up / down".into()
                        },
                    ),
                    ("F / V", if def.has_engine { "flaps down / up a notch".into() } else { "airbrakes out / in".into() }),
                    ("G", if def.retractable_gear { "landing gear up / down".into() } else { "(fixed gear)".into() }),
                    ("/   \\ or '", if def.has_spoilers { "spoilers / speed brake out / in a step".into() } else { "(no spoilers)".into() }),
                    ("T", if def.has_reverser { "thrust reverse on / off (ground; throttle sets reverse power)".into() } else { "(no reversers)".into() }),
                    ("B or Space (hold)", "wheel brakes".into()),
                    ("[  ]", "trim nose down / up".into()),
                    ("Z", "autopilot: engage holding heading / altitude / speed, or off".into()),
                    ("Ins / Del", "autopilot heading bug left / right 5°".into()),
                    ("Home / End", "autopilot altitude +/- 100 ft".into()),
                    ("9 / 0", "autopilot speed -/+ 5 kt (autothrottle)".into()),
                    ("K", "engine-failure practice (cycles engines, then restores)".into()),
                    ("J", "auto flight: the bot flies (any control input takes back)".into()),
                    ("C", "camera: chase, cockpit, tower, fly-by, cinematic".into()),
                    ("Right-drag / wheel", "orbit / zoom chase camera".into()),
                    ("R", "restart at the chosen start".into()),
                    ("Tab", "aircraft, start, auto flight, time & weather".into()),
                    ("P", "pause      X: time warp 1/2/4/8x".into()),
                    ("M   ,  .", "toggle map, zoom map out / in".into()),
                    ("I", "weather info (METAR, wind, visibility, local time)".into()),
                    ("H", "toggle this help      Esc: quit".into()),
                ];
                for (k, v) in rows {
                    ui.monospace(k);
                    ui.label(v);
                    ui.end_row();
                }
            });
        });
}

/// Time of day + weather picked in the menu (applied live).
pub struct EnvChoice {
    pub hours: f32,
    pub weather: crate::clouds::Weather,
    /// Real weather & time (live data, see weather.rs).
    pub live: bool,
}

fn category_name(c: Category) -> &'static str {
    match c {
        Category::Trainer => "Light aircraft",
        Category::Glider => "Gliders",
        Category::Warbird => "Warbirds",
        Category::FastJet => "Fast jets",
        Category::Airliner => "Airliners",
    }
}

/// Key numbers for the menu's detail pane.
fn stats_lines(d: &crate::flight::AircraftDef) -> Vec<(&'static str, String)> {
    let n = d.engine_count;
    let engines = match d.engine_kind {
        EngineKind::None => "none (sailplane)".to_string(),
        EngineKind::Piston => format!("{}× piston, {:.0} hp", n, d.engine_power_w / 745.7),
        EngineKind::Turboprop => format!("{}× turboprop, {:.0} shp each", n, d.engine_power_w / 745.7),
        EngineKind::Jet => {
            let dry = d.static_thrust / n.max(1) as f32 / 1000.0;
            if d.has_afterburner {
                format!("{}× turbofan, {:.0} kN dry / {:.0} kN reheat", n, dry, (d.static_thrust + d.ab_static_thrust) / 1000.0)
            } else {
                format!("{}× turbofan, {:.0} kN each", n, dry)
            }
        }
    };
    let mass = if d.mass >= 10_000.0 { format!("{:.1} t", d.mass / 1000.0) } else { format!("{:.0} kg", d.mass) };
    let mut v = vec![
        ("Engines", engines),
        ("Mass · span", format!("{mass} · {:.1} m", d.wingspan)),
        ("Stall (clean / landing)", format!("{:.0} / {:.0} kt", d.vs1_kt, d.vs0_kt)),
    ];
    if d.has_engine {
        v.push(("Vr · Vref", format!("{:.0} · {:.0} kt", d.vr_kt, d.vref_kt)));
    }
    v.push((
        "Never exceed",
        if d.mmo > 0.0 { format!("{:.0} kt / M{:.2}", d.vne_kt, d.mmo) } else { format!("{:.0} kt", d.vne_kt) },
    ));
    let mut sys = Vec::new();
    if d.retractable_gear {
        sys.push("retractable gear");
    }
    if d.has_spoilers {
        sys.push(if d.category == Category::FastJet { "speed brake" } else { "spoilers" });
    }
    if d.has_reverser {
        sys.push("reversers");
    }
    if d.has_afterburner {
        sys.push("afterburner");
    }
    if d.taildragger {
        sys.push("taildragger");
    }
    match d.control_law {
        crate::flight::ControlLaw::Airliner => sys.push("fly-by-wire"),
        crate::flight::ControlLaw::FastJet => sys.push("FBW g-command"),
        _ => {}
    }
    if !sys.is_empty() {
        v.push(("Systems", sys.join(", ")));
    }
    v.push(("Runway needed", format!("{:.0} m", crate::sim::min_runway(d))));
    v
}

/// Aircraft / start-position menu. Returns Some((aircraft, start, watch)) when the user hits Fly.
pub fn menu(ctx: &egui::Context, sim: &Sim, choice: &mut (usize, Start), watch: &mut bool, env: &mut EnvChoice) -> Option<(usize, Start, bool)> {
    let mut go = None;
    egui::Window::new("Skyward")
        .anchor(Align2::CENTER_CENTER, [0.0, -40.0])
        .collapsible(false)
        .resizable(false)
        .min_width(760.0)
        .show(ctx, |ui| {
            ui.horizontal_top(|ui| {
                // Left: aircraft grouped by category.
                ui.vertical(|ui| {
                    ui.set_width(230.0);
                    ui.heading("Aircraft");
                    let mut cats: Vec<Category> = Vec::new();
                    for d in &sim.defs {
                        if !cats.contains(&d.category) {
                            cats.push(d.category);
                        }
                    }
                    for c in cats {
                        ui.add_space(6.0);
                        ui.label(egui::RichText::new(category_name(c)).size(12.5).color(Color32::from_rgb(140, 190, 255)));
                        for (i, d) in sim.defs.iter().enumerate().filter(|(_, d)| d.category == c) {
                            ui.radio_value(&mut choice.0, i, egui::RichText::new(d.name).strong().size(15.0));
                        }
                    }
                });
                ui.separator();
                // Right: details, start options, auto flight, time & weather.
                ui.vertical(|ui| {
                    ui.set_width(500.0);
                    let d = &sim.defs[choice.0.min(sim.defs.len() - 1)];
                    ui.heading(d.name);
                    ui.label(egui::RichText::new(d.description).size(13.0).color(Color32::from_gray(200)));
                    ui.add_space(4.0);
                    egui::Grid::new("stats").num_columns(2).spacing([12.0, 2.0]).show(ui, |ui| {
                        for (k, v) in stats_lines(d) {
                            ui.label(egui::RichText::new(k).size(12.0).color(Color32::GRAY));
                            ui.label(egui::RichText::new(v).size(12.0));
                            ui.end_row();
                        }
                    });
                    ui.separator();
                    ui.heading("Start");
                    if !choice.1.valid_for(d) {
                        choice.1 = Start::default_for(d);
                    }
                    for s in Start::ALL.into_iter().filter(|s| s.valid_for(d)) {
                        ui.radio_value(&mut choice.1, s, s.label_for(d));
                    }
                    ui.add_space(4.0);
                    ui.checkbox(
                        watch,
                        egui::RichText::new(if d.has_engine {
                            "Auto flight (watch): take-off, sightseeing tour, landing — on repeat"
                        } else {
                            "Auto flight (watch): the glider soars by itself, thermal to thermal"
                        })
                        .strong(),
                    );
                    ui.separator();
                    ui.heading("Time & weather");
                    ui.checkbox(&mut env.live, "Real weather & time (live)");
                    ui.horizontal(|ui| {
                        let (h, m) = (env.hours.floor() as u32, ((env.hours.fract() * 60.0).round() as u32).min(59));
                        ui.add(egui::Slider::new(&mut env.hours, 0.0..=23.99).show_value(false).text(format!("{h:02}:{m:02}")));
                    });
                    ui.horizontal(|ui| {
                        for w in crate::clouds::Weather::ALL {
                            ui.radio_value(&mut env.weather, w, w.name());
                        }
                    });
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new(egui::RichText::new("  Fly  ").size(18.0))).clicked() {
                            go = Some((choice.0, choice.1, *watch));
                        }
                        ui.label(egui::RichText::new("Enter = Fly, Tab = back").color(Color32::GRAY));
                    });
                });
            });
            if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                go = Some((choice.0, choice.1, *watch));
            }
        });
    go
}

/// Centred status banner (e.g. scenery still streaming after a restart).
pub fn notice(ctx: &egui::Context, msg: &str) {
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::NULL.with("notice")));
    let g = p.layout(msg.to_string(), FontId::proportional(20.0), Color32::WHITE, 500.0);
    let r = Rect::from_center_size(screen.center() - EVec2::new(0.0, screen.height() * 0.22), g.size() + EVec2::new(40.0, 24.0));
    p.rect_filled(r, 10.0, Color32::from_rgba_unmultiplied(10, 14, 24, 215));
    p.galley(r.min + EVec2::new(20.0, 12.0), g, Color32::WHITE);
}

fn moving_map(p: &egui::Painter, mr: Rect, sim: &Sim, world: &World, tiles: &MapTiles) {
    p.rect_filled(mr.expand(4.0), 6.0, Color32::from_rgba_unmultiplied(10, 10, 12, 210));
    p.rect_filled(mr, 0.0, Color32::from_rgb(52, 58, 52));
    let mp = p.with_clip_rect(mr);
    let range = sim.map_range();
    let scale = mr.width() * 0.5 / range;
    let c = glam::vec2(sim.ac.pos.x, sim.ac.pos.z);
    let to = |w: glam::Vec2| {
        let q = (w - c) * scale;
        Pos2::new(mr.center().x + q.x, mr.center().y + q.y)
    };
    let view = Rect::from_center_size(mr.center(), mr.size());
    let mut order: Vec<_> = tiles.iter().collect();
    order.sort_by_key(|(id, _)| id.z);
    for (id, tex) in order {
        let (lo, hi) = id.rect(&world.proj);
        let r = Rect::from_min_max(to(lo), to(hi));
        if r.intersects(view) {
            mp.image(tex.id(), r, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
        }
    }
    for rw in &world.runways {
        let w = (rw.width * scale).max(3.0);
        mp.line_segment([to(rw.a), to(rw.b)], Stroke::new(w, Color32::from_rgb(40, 40, 46)));
    }
    for a in &world.airports {
        let q = to(a.pos);
        if view.expand(-10.0).contains(q) {
            let code = [&a.icao, &a.iata, &a.name].into_iter().find(|c| !c.is_empty()).map_or("", |c| c.as_str());
            mp.text(q + EVec2::new(0.0, -10.0), Align2::CENTER_BOTTOM, code, FontId::proportional(11.0), Color32::from_rgb(120, 40, 140));
            mp.circle_stroke(q, 4.0, Stroke::new(2.0, Color32::from_rgb(120, 40, 140)));
        }
    }
    // Auto flight: the runway it is working and its next sightseeing target.
    if let Some(b) = &sim.bot {
        if let Some(end) = b.focus_runway() {
            mp.line_segment([to(end.thr), to(end.far())], Stroke::new((end.width * scale).max(3.0) + 2.0, Color32::from_rgb(90, 220, 255)));
        }
        if let Some((name, wp)) = b.waypoint() {
            let q = to(wp);
            let d = 6.0;
            let col = Color32::from_rgb(230, 60, 230);
            mp.add(Shape::convex_polygon(vec![q + EVec2::new(0.0, -d), q + EVec2::new(d, 0.0), q + EVec2::new(0.0, d), q + EVec2::new(-d, 0.0)], col, Stroke::new(1.0, Color32::BLACK)));
            mp.text(q + EVec2::new(8.0, 0.0), Align2::LEFT_CENTER, name, FontId::proportional(11.0), col);
        }
    }
    crate::weather::map_wind(&mp, mr, world);
    // Aircraft symbol at the centre, rotated with heading.
    let pp = mr.center();
    let h = sim.ac.heading();
    let f = EVec2::new(h.sin(), -h.cos());
    let rt = EVec2::new(-f.y, f.x);
    p.add(Shape::convex_polygon(
        vec![pp + f * 10.0, pp - f * 7.0 + rt * 7.0, pp - f * 3.0, pp - f * 7.0 - rt * 7.0],
        Color32::from_rgb(255, 200, 40),
        Stroke::new(1.0, Color32::BLACK),
    ));
    let km = range / 1000.0;
    label(p, Pos2::new(mr.max.x - 12.0, mr.min.y + 12.0), "N", 13.0, Color32::WHITE);
    p.text(
        Pos2::new(mr.min.x + 6.0, mr.max.y - 4.0),
        Align2::LEFT_BOTTOM,
        format!("{:.0} km across  ( , . zoom)", km * 2.0),
        FontId::proportional(11.0),
        Color32::from_gray(20),
    );
}
