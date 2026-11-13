//! Rotorcraft displays: the EC130's VEMD (vehicle and engine multifunction
//! display, two LCD screens in the cockpit, drawn into the model's screen
//! textures by `cockpit::displays`), the FPV quad's OSD (on-screen display,
//! Betaflight / DJI style) and the rotorcraft status strip of the 2D HUD.

use crate::sim::{CamMode, Sim};
use crate::ui::{f_digits_b, f_inst, f_inst_b, pal};
use egui::{Align2, Color32, FontId, Pos2, Rect, Shape, Stroke, Vec2};
use std::f32::consts::PI;

const LCD_BG: Color32 = Color32::from_rgb(4, 8, 6);
const LCD_GREEN: Color32 = Color32::from_rgb(70, 230, 120);
const LCD_AMBER: Color32 = Color32::from_rgb(255, 184, 40);
const LCD_RED: Color32 = Color32::from_rgb(255, 70, 60);
const LCD_TEXT: Color32 = Color32::from_rgb(220, 236, 228);

fn polar(c: Pos2, r: f32, a: f32) -> Pos2 {
    c + Vec2::new(a.sin(), -a.cos()) * r
}

fn arc(p: &egui::Painter, c: Pos2, r: f32, a0: f32, a1: f32, w: f32, col: Color32) {
    let n = ((a1 - a0).abs() / 0.05).ceil().max(2.0) as usize;
    let pts: Vec<Pos2> = (0..=n).map(|i| polar(c, r, a0 + (a1 - a0) * i as f32 / n as f32)).collect();
    p.add(Shape::line(pts, Stroke::new(w, col)));
}

/// One VEMD screen. Upper: the first limitation indicator (FLI) with the
/// torque / T4 / Ng readouts and the rotor speed. Lower: engine and
/// vehicle data (fuel, fuel flow, electrics, OAT, power margin).
pub fn vemd(p: &egui::Painter, r: Rect, sim: &Sim, lower: bool) {
    p.rect_filled(r, 0.0, LCD_BG);
    let o = sim.ac.rotor.out;
    let s = r.height().min(r.width() * 1.3);
    let font = |k: f32| FontId::monospace(s * k);
    if !lower {
        let c = Pos2::new(r.center().x, r.min.y + r.height() * 0.58);
        let rad = s * 0.36;
        let ang = |v: f32| -0.72 * PI + (v / 12.0).clamp(0.0, 1.0) * 1.44 * PI;
        arc(p, c, rad, ang(0.0), ang(9.0), s * 0.025, LCD_GREEN);
        arc(p, c, rad, ang(9.0), ang(10.0), s * 0.025, LCD_AMBER);
        arc(p, c, rad, ang(10.0), ang(12.0), s * 0.025, LCD_RED);
        for k in 0..=12 {
            let a = ang(k as f32);
            let l = if k % 2 == 0 { 0.12 } else { 0.06 };
            p.line_segment([polar(c, rad * (1.0 - l), a), polar(c, rad, a)], Stroke::new(s * 0.012, LCD_TEXT));
            if k % 2 == 0 {
                p.text(polar(c, rad * 0.74, a), Align2::CENTER_CENTER, format!("{k}"), font(0.07), LCD_TEXT);
            }
        }
        let a = ang(o.fli);
        let col = if o.fli > 10.0 {
            LCD_RED
        } else if o.fli > 9.0 {
            LCD_AMBER
        } else {
            LCD_TEXT
        };
        p.line_segment([c, polar(c, rad * 0.95, a)], Stroke::new(s * 0.022, col));
        p.circle_filled(c, s * 0.03, col);
        p.text(Pos2::new(r.min.x + s * 0.05, r.min.y + s * 0.05), Align2::LEFT_TOP, "FLI", font(0.07), LCD_GREEN);
        let nr_col = if o.low_nr || o.high_nr { LCD_RED } else { LCD_TEXT };
        p.text(Pos2::new(r.max.x - s * 0.05, r.min.y + s * 0.05), Align2::RIGHT_TOP, format!("NR {:3.0}", o.nr_rpm), font(0.075), nr_col);
        let y = r.max.y - s * 0.08;
        let lim = |v: f32, l: f32| if v > l { LCD_RED } else { LCD_TEXT };
        p.text(Pos2::new(r.min.x + s * 0.05, y), Align2::LEFT_BOTTOM, format!("TQ{:4.0}", o.torque_pct), font(0.07), lim(o.torque_pct, 100.0));
        p.text(Pos2::new(r.center().x, y), Align2::CENTER_BOTTOM, format!("T4{:4.0}", o.tot_c), font(0.07), lim(o.tot_c, 865.0));
        p.text(Pos2::new(r.max.x - s * 0.05, y), Align2::RIGHT_BOTTOM, format!("NG{:5.1}", o.ng_pct), font(0.07), lim(o.ng_pct, 101.0));
    } else {
        let sys = &sim.ac.sys;
        let rows: [(&str, String, Color32); 6] = [
            ("FUEL", format!("{:4.0} KG", sys.fuel_total()), if sys.fuel_total() < 60.0 { LCD_AMBER } else { LCD_TEXT }),
            ("F.FLOW", format!("{:4.0} KG/H", sim.ac.fuel_flow(0)), LCD_TEXT),
            ("GEN", format!("{:4.1} V {:3.0} A", sys.volts, sys.amps), if sys.volts < 25.0 { LCD_AMBER } else { LCD_TEXT }),
            ("OAT", format!("{:+4.0} °C", sim.ac.oat()), LCD_TEXT),
            ("MARGIN", format!("{:4.0} %", (1.0 - o.power_w / o.power_avail_w.max(1.0)) * 100.0), if o.power_w > 0.97 * o.power_avail_w { LCD_AMBER } else { LCD_TEXT }),
            ("N2", format!("{:5.1} %", o.n2_pct), LCD_TEXT),
        ];
        let h = r.height() / (rows.len() as f32 + 0.6);
        for (i, (k, v, col)) in rows.iter().enumerate() {
            let y = r.min.y + h * (0.8 + i as f32);
            p.text(Pos2::new(r.min.x + s * 0.05, y), Align2::LEFT_CENTER, *k, font(0.065), LCD_GREEN);
            p.text(Pos2::new(r.max.x - s * 0.05, y), Align2::RIGHT_CENTER, v, font(0.07), *col);
        }
    }
}

/// Text with a 1-px black outline (OSD style: legible on any scenery).
fn osd_text(p: &egui::Painter, at: Pos2, align: Align2, s: impl Into<String>, size: f32, col: Color32) {
    let s = s.into();
    for (dx, dy) in [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0), (1.2, 1.2)] {
        p.text(at + Vec2::new(dx, dy), align, &s, f_digits_b(size), Color32::from_black_alpha(220));
    }
    p.text(at, align, s, f_digits_b(size), col);
}

/// Wide-angle lens fall-off and a faint analogue-video tint (FPV view).
fn vignette(p: &egui::Painter, r: Rect) {
    let c = r.center();
    let rx = r.width() * 0.5;
    let ry = r.height() * 0.5;
    let n = 48;
    let mut mesh = egui::epaint::Mesh::default();
    // Rings from clear (inner) to dark (outer corners).
    let rings: [(f32, u8); 3] = [(0.70, 0), (1.0, 70), (1.45, 150)];
    for (k, (s, a)) in rings.iter().enumerate() {
        for i in 0..n {
            let t = i as f32 / n as f32 * std::f32::consts::TAU;
            mesh.colored_vertex(c + Vec2::new(t.cos() * rx * s, t.sin() * ry * s), Color32::from_black_alpha(*a));
            if k > 0 {
                let (i0, i1) = ((k - 1) * n + i, (k - 1) * n + (i + 1) % n);
                let (j0, j1) = (k * n + i, k * n + (i + 1) % n);
                mesh.add_triangle(i0 as u32, j0 as u32, j1 as u32);
                mesh.add_triangle(i0 as u32, j1 as u32, i1 as u32);
            }
        }
    }
    p.add(Shape::mesh(mesh));
}

/// The FPV quad's on-screen display.
pub fn osd(p: &egui::Painter, screen: Rect, sim: &Sim) {
    let ac = &sim.ac;
    let o = ac.rotor.out;
    let q = ac.rotor.quad;
    let w = screen.width().min(screen.height() * 1.78);
    let k = (w / 1280.0).clamp(0.7, 2.0);
    let fs = 17.0 * k;
    let white = Color32::from_rgb(245, 245, 245);
    let m = Vec2::new(28.0 * k, 22.0 * k);
    let fpv = sim.cam == CamMode::Cockpit;
    if fpv {
        vignette(p, screen);
    }
    // Lower corners (the top of the screen belongs to the status card / map).
    let (bl, br) = (Pos2::new(screen.min.x + m.x, screen.max.y - m.y), screen.max - m);
    let cells = 6.0;
    let cell = o.batt_v / cells;
    let bcol = if cell < 3.3 {
        pal::RED
    } else if cell < 3.5 {
        pal::AMBER
    } else {
        white
    };
    let line = fs * 1.35;
    let gs = glam::Vec2::new(ac.vel.x, ac.vel.z).length() * 3.6;
    let alt = ac.pos.y - q.home.y;
    let t = q.flight_time;
    osd_text(p, bl - Vec2::new(0.0, line * 4.0), Align2::LEFT_BOTTOM, format!("{:4.1}V", o.batt_v), fs * 1.2, bcol);
    osd_text(p, bl - Vec2::new(0.0, line * 3.0), Align2::LEFT_BOTTOM, format!("{cell:4.2}v {:4.0}mAh", o.batt_mah), fs, bcol);
    osd_text(p, bl - Vec2::new(0.0, line * 2.0), Align2::LEFT_BOTTOM, format!("{:5.1}A", o.batt_a), fs, white);
    osd_text(p, bl - Vec2::new(0.0, line), Align2::LEFT_BOTTOM, format!("THR {:3.0}%", sim.ctl.throttle * 100.0), fs, white);
    osd_text(p, bl, Align2::LEFT_BOTTOM, format!("{gs:3.0} km/h"), fs, white);
    osd_text(p, br - Vec2::new(0.0, line * 3.0), Align2::RIGHT_BOTTOM, format!("{:02}:{:02}", (t / 60.0) as u32, (t % 60.0) as u32), fs, white);
    osd_text(p, br - Vec2::new(0.0, line * 2.0), Align2::RIGHT_BOTTOM, format!("ALT {alt:4.0}m"), fs, white);
    let to_home = glam::Vec2::new(q.home.x - ac.pos.x, q.home.z - ac.pos.z);
    let dist = to_home.length();
    osd_text(p, br - Vec2::new(0.0, line), Align2::RIGHT_BOTTOM, format!("H {dist:4.0}m"), fs, white);
    // Home arrow relative to the heading.
    let bearing = to_home.x.atan2(-to_home.y) - ac.heading();
    let ac_pos = Pos2::new(br.x - 128.0 * k, br.y - line * 1.55);
    let tip = ac_pos + Vec2::new(bearing.sin(), -bearing.cos()) * 9.0 * k;
    let side = Vec2::new(bearing.cos(), bearing.sin()) * 5.0 * k;
    let tail = ac_pos - Vec2::new(bearing.sin(), -bearing.cos()) * 6.0 * k;
    if dist > 3.0 {
        p.add(Shape::convex_polygon(vec![tip, tail + side, tail - side], white, Stroke::new(1.5, Color32::from_black_alpha(200))));
    }
    // Centre: crosshair and the artificial horizon where the real one is (FPV only).
    let c = screen.center();
    if fpv {
        let cs = 9.0 * k;
        for (a, b) in
            [(Vec2::new(-cs * 2.0, 0.0), Vec2::new(-cs * 0.6, 0.0)), (Vec2::new(cs * 0.6, 0.0), Vec2::new(cs * 2.0, 0.0)), (Vec2::new(0.0, -cs * 0.6), Vec2::new(0.0, -cs * 1.4))]
        {
            p.line_segment([c + a, c + b], Stroke::new(3.0 * k, Color32::from_black_alpha(200)));
            p.line_segment([c + a, c + b], Stroke::new(1.6 * k, white));
        }
        if let Some(qd) = sim.def().rotor.as_ref().and_then(|r| r.quad()) {
            // Camera pitch above the horizon → the horizon's offset below the centre.
            let aspect = screen.width() / screen.height().max(1.0);
            let t_half_v = (qd.cam_fov * 0.5).tan() / aspect.max(0.5);
            let cam_pitch = ac.pitch() + qd.cam_tilt * ac.roll().cos();
            let roll = ac.roll();
            let dy = screen.height() * 0.5 * cam_pitch.tan() / t_half_v;
            let dir = Vec2::new(roll.cos(), -roll.sin());
            let hc = c + Vec2::new(-roll.sin(), -roll.cos()) * -dy;
            if (hc - c).length() < screen.height() * 0.45 {
                for s in [-1.0f32, 1.0] {
                    let a = hc + dir * s * 60.0 * k;
                    let b = hc + dir * s * 150.0 * k;
                    p.line_segment([a, b], Stroke::new(3.0 * k, Color32::from_black_alpha(160)));
                    p.line_segment([a, b], Stroke::new(1.4 * k, Color32::from_white_alpha(200)));
                }
            }
        }
    }
    // Warnings.
    let warn = if cell < 3.3 && cell > 0.5 {
        Some(("LAND NOW", pal::RED))
    } else if cell < 3.5 && cell > 0.5 {
        Some(("BATTERY LOW", pal::AMBER))
    } else if o.vrs > 0.4 {
        Some(("PROP WASH", pal::AMBER))
    } else {
        None
    };
    if let Some((s, col)) = warn {
        let blink = (sim.time * 2.0).fract() < 0.6;
        if blink {
            osd_text(p, c + Vec2::new(0.0, 70.0 * k), Align2::CENTER_CENTER, s, fs * 1.2, col);
        }
    }
}

/// Rotorcraft status strip for the 2D HUD (assist mode, collective, rotor /
/// engine warnings). Returns the strip's top edge.
pub fn status_strip(p: &egui::Painter, screen: Rect, sim: &Sim, bottom: f32) -> f32 {
    let def = sim.def();
    let Some(rd) = &def.rotor else { return bottom };
    let o = sim.ac.rotor.out;
    let quad = rd.quad().is_some();
    let mut items: Vec<(String, Color32)> = vec![(sim.ac.rotor.assist.label(quad).to_string(), pal::CYAN)];
    if !quad {
        items.push((format!("COLL {:3.0}%", sim.ctl.throttle * 100.0), pal::TEXT));
        if o.low_nr {
            items.push(("LOW NR".into(), pal::RED));
        }
        if o.high_nr {
            items.push(("HIGH NR".into(), pal::AMBER));
        }
        if sim.ac.rotor.fail.tail_drive || sim.ac.rotor.fail.tail_pitch {
            items.push(("TAIL ROTOR".into(), pal::RED));
        }
        if sim.ac.rotor.fail.hydraulics {
            items.push(("HYD".into(), pal::AMBER));
        }
        if o.vrs > 0.45 {
            items.push(("VORTEX RING".into(), pal::AMBER));
        }
        if o.stall > 0.2 {
            items.push(("BLADE STALL".into(), pal::RED));
        }
        if !sim.ac.engine_running(0) {
            items.push(("ENGINE OUT".into(), pal::RED));
        }
    }
    let f = f_inst_b(12.5);
    let widths: Vec<f32> = items.iter().map(|(s, _)| s.len() as f32 * 8.2 + 18.0).collect();
    let total: f32 = widths.iter().sum::<f32>() + 6.0 * (items.len() as f32 - 1.0);
    let h = 24.0;
    let mut x = screen.center().x - total * 0.5;
    let y = bottom - h;
    for ((s, col), w) in items.iter().zip(widths) {
        let r = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, h));
        p.rect_filled(r, 5.0, pal::PANEL);
        p.rect_stroke(r, 5.0, Stroke::new(1.0, col.gamma_multiply(0.5)), egui::StrokeKind::Inside);
        p.text(r.center(), Align2::CENTER_CENTER, s, f.clone(), *col);
        x += w + 6.0;
    }
    let _ = f_inst;
    y
}

/// Help-screen rows for the rotorcraft (keys as bound by default).
pub fn control_rows(def: &crate::flight::AircraftDef) -> Vec<(&'static str, &'static str, String)> {
    let quad = def.rotor.as_ref().is_some_and(|r| r.quad().is_some());
    let mut v: Vec<(&str, &str, String)> = if quad {
        vec![
            ("Flying", "W S  /  ↑ ↓", "Pitch (angle: tilt · acro: rate)".into()),
            ("Flying", "A D  /  ← →", "Roll".into()),
            ("Flying", "Q E", "Yaw".into()),
            ("Flying", "Shift Ctrl  /  = −", "Throttle".into()),
            ("Flying", "G  ;", "Angle ↔ acro (rate) mode".into()),
            ("Flying", "Gamepad", "Mode 2: left stick throttle / yaw, right stick pitch / roll".into()),
            ("Flying", "K", "Motor failure practice".into()),
        ]
    } else {
        vec![
            ("Flying", "W S  /  ↑ ↓", "Cyclic fore / aft".into()),
            ("Flying", "A D  /  ← →", "Cyclic left / right".into()),
            ("Flying", "Q E", "Pedals (yaw, anti-torque)".into()),
            ("Flying", "Shift Ctrl  /  = −", "Collective up / down".into()),
            ("Flying", "[  ]", "Cyclic trim".into()),
            ("Flying", "G  ;", "Stability assist: attitude hold → off → SAS".into()),
            ("Engine", "K", "Engine failure (autorotation practice)".into()),
            ("Systems", "O  F2", "Failures: tail rotor, hydraulics, engine…".into()),
        ]
    };
    v.extend([
        ("Systems", "L", "Landing lights".into()),
        ("Autopilot", "Z", "Hold heading, altitude, speed".into()),
        ("Autopilot", "Ins Del · Home End · 9 0", "Heading · altitude · speed".into()),
        ("Autopilot", "J", "Auto flight".into()),
        ("View", "C", if quad { "Camera (cockpit = FPV)".into() } else { "Camera".into() }),
        ("View", "Right drag · wheel", "Orbit · zoom".into()),
        ("View", "M  U  I", "Map · instruments · weather".into()),
        ("Sim", "P  X  R", "Pause · time warp · restart".into()),
        ("Sim", "Esc  Tab", "Menu".into()),
        ("Sim", "H  F1", "This help".into()),
    ]);
    v
}

/// Aircraft-card specification rows for the rotorcraft.
pub fn spec_lines(d: &crate::flight::AircraftDef) -> Vec<(&'static str, String)> {
    use super::RotorDef;
    match d.rotor.as_deref() {
        Some(RotorDef::Heli(h)) => vec![
            ("Engine", format!("1 × turboshaft, {:.0} shp take-off", h.p_takeoff / 745.7)),
            ("Main rotor", format!("Ø {:.2} m, {} blades, {:.0} rpm", 2.0 * h.radius, h.blades, h.omega * 60.0 / std::f32::consts::TAU)),
            ("Tail rotor", format!("Fenestron Ø {:.1} m, {:.0} rpm", 2.0 * h.tr_radius, h.omega * h.tr_gear * 60.0 / std::f32::consts::TAU)),
            ("Mass", format!("{:.0} kg (max 2,427 kg)", d.mass)),
            ("Cruise · never exceed", format!("{:.0} · {:.0} kt", d.vno_kt, d.vne_kt)),
            ("Hover ceiling", "OGE 2,537 m · IGE 3,098 m".into()),
            ("Landing spot", "Helipad, apron or grass".into()),
            ("Model", "FlightGear 3D · Skyward rotor FDM".into()),
        ],
        Some(RotorDef::Quad(q)) => vec![
            ("Motors", format!("4 × 2207, {:.0} rpm max", q.omega_max * 60.0 / std::f32::consts::TAU)),
            ("Props", format!("5\" tri-blade, {:.0} mm", 2000.0 * q.prop_radius)),
            ("Battery", format!("{:.0}S LiPo, {:.0} mAh", q.cells, q.capacity_mah)),
            ("Mass", format!("{:.0} g", d.mass * 1000.0)),
            ("Thrust / weight", format!("{:.1} : 1", 4.0 * q.max_thrust(q.v_full) / (d.mass * crate::flight::GRAVITY))),
            ("Rates", format!("{:.0}°/s max, angle {:.0}°", q.rate_max, q.angle_max.to_degrees())),
            ("Camera", format!("{:.0}° up-tilt, {:.0}° FOV", q.cam_tilt.to_degrees(), q.cam_fov.to_degrees())),
            ("Model", "Original procedural · Skyward quad FDM".into()),
        ],
        None => Vec::new(),
    }
}
