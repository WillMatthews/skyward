//! Fast-jet head-up display. In the cockpit view the symbology is
//! conformal: projected through the real camera, so the flight-path marker
//! sits on the point the aircraft is flying towards and the pitch ladder
//! on the real horizon. In outside views the same symbology is drawn as a
//! compact repeater with a virtual camera along the nose.

use super::{f_digits_b, f_inst, pal};
use crate::flight::KT;
use crate::sim::Sim;
use egui::{Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use glam::{Mat4, Quat, Vec3};

const FT: f32 = 3.280_84;

/// A camera for projecting directions to the screen.
pub struct Cam {
    /// World → camera rotation (camera looks down −Z).
    pub rot: Mat4,
    pub fov_y: f32,
    pub screen: Rect,
}

impl Cam {
    pub fn from_view(view: Mat4, fov_y: f32, screen: Rect) -> Cam {
        let rot = Mat4::from_cols(view.x_axis, view.y_axis, view.z_axis, glam::Vec4::W);
        Cam { rot, fov_y, screen }
    }
    /// Virtual camera along the aircraft's nose (for the outside-view repeater).
    pub fn along_nose(rot: Quat, fov_y: f32, screen: Rect) -> Cam {
        let fwd = rot * Vec3::Z;
        let up = rot * Vec3::Y;
        let view = glam::camera::rh::view::look_at_mat4(Vec3::ZERO, fwd, up);
        Cam::from_view(view, fov_y, screen)
    }
    pub fn project(&self, dir: Vec3) -> Option<Pos2> {
        let v = self.rot.transform_vector3(dir);
        if v.z > -1e-3 {
            return None;
        }
        let t = (self.fov_y * 0.5).tan();
        let aspect = self.screen.width() / self.screen.height();
        let x = v.x / -v.z / t / aspect;
        let y = v.y / -v.z / t;
        Some(Pos2::new(self.screen.center().x + x * self.screen.width() * 0.5, self.screen.center().y - y * self.screen.height() * 0.5))
    }
    /// Points per degree near the centre.
    pub fn ppd(&self) -> f32 {
        self.screen.height() * 0.5 / (self.fov_y * 0.5).tan() * 1f32.to_radians().tan()
    }
}

fn dir(az: f32, el: f32) -> Vec3 {
    Vec3::new(az.sin() * el.cos(), el.sin(), -az.cos() * el.cos())
}

/// Glowing HUD stroke.
/// HUD stroke: a faint dark under-stroke keeps the green legible against a
/// bright sky or cloud (a real combiner can't, a monitor can).
fn line(p: &egui::Painter, a: Pos2, b: Pos2, w: f32, col: Color32) {
    p.line_segment([a, b], Stroke::new(w + 2.2, Color32::from_black_alpha(60)));
    p.line_segment([a, b], Stroke::new(w, col));
}

fn poly(p: &egui::Painter, pts: Vec<Pos2>, w: f32, col: Color32) {
    p.add(Shape::line(pts.clone(), Stroke::new(w + 2.2, Color32::from_black_alpha(60))));
    p.add(Shape::line(pts, Stroke::new(w, col)));
}

fn circle(p: &egui::Painter, c: Pos2, r: f32, w: f32, col: Color32) {
    p.circle_stroke(c, r, Stroke::new(w + 2.2, Color32::from_black_alpha(60)));
    p.circle_stroke(c, r, Stroke::new(w, col));
}

fn text(p: &egui::Painter, at: Pos2, align: Align2, s: impl ToString, size: f32, col: Color32) -> Rect {
    let s = s.to_string();
    let size = size.max(13.0);
    for o in [EVec2::new(1.0, 1.0), EVec2::new(-1.0, 1.0), EVec2::new(0.0, -1.0)] {
        p.text(at + o, align, &s, f_digits_b(size), Color32::from_black_alpha(80));
    }
    p.text(at, align, s, f_digits_b(size), col)
}

/// Draw the HUD. `field` = the combiner area symbology is clipped to;
/// `k` scales strokes / text.
pub fn draw(p: &egui::Painter, cam: &Cam, field: Rect, sim: &Sim, k: f32, col: Color32) {
    let p = &p.with_clip_rect(field);
    let ac = &sim.ac;
    let (_, rot) = sim.render_pose();
    let w = 1.6 * k;
    let fwd = rot * Vec3::Z;
    let up = rot * Vec3::Y;
    let vel = ac.vel;
    let bore = cam.project(fwd);
    // Gun cross at the boresight.
    if let Some(b) = bore {
        let s = 9.0 * k;
        line(p, b - EVec2::new(s, 0.0), b - EVec2::new(s * 0.35, 0.0), w, col);
        line(p, b + EVec2::new(s * 0.35, 0.0), b + EVec2::new(s, 0.0), w, col);
        line(p, b - EVec2::new(0.0, s), b - EVec2::new(0.0, s * 0.35), w, col);
        line(p, b + EVec2::new(0.0, s * 0.35), b + EVec2::new(0.0, s), w, col);
    }
    // Flight-path marker.
    let fpm = if vel.length() > 15.0 { cam.project(vel.normalize()) } else { bore };
    let roll = ac.roll();
    let track_az = if glam::vec2(vel.x, vel.z).length() > 5.0 { vel.x.atan2(-vel.z) } else { ac.heading() };
    if let Some(f) = fpm {
        let r = 6.0 * k;
        circle(p, f, r, w, col);
        line(p, f - EVec2::new(r * 2.6, 0.0), f - EVec2::new(r, 0.0), w, col);
        line(p, f + EVec2::new(r, 0.0), f + EVec2::new(r * 2.6, 0.0), w, col);
        line(p, f - EVec2::new(0.0, r), f - EVec2::new(0.0, r * 2.0), w, col);
    }
    // Pitch ladder, conformal (each rung at its true elevation along the track),
    // kept clear of the heading tape and the bank scale.
    let ppd = cam.ppd();
    let full = p.clone();
    let p = &p.with_clip_rect(Rect::from_min_max(Pos2::new(field.min.x, field.min.y + field.height() * 0.12), Pos2::new(field.max.x, field.max.y - field.height() * 0.2)));
    let centre_x = fpm.map(|f| f.x);
    for i in -18..=18 {
        let el_deg = i as f32 * 5.0;
        let el = el_deg.to_radians();
        let Some(c) = cam.project(dir(track_az, el)) else { continue };
        let Some(c2) = cam.project(dir(track_az + 0.02, el)) else { continue };
        let along = (c2 - c).normalized();
        // Rungs are centred on the flight-path marker's lateral position.
        let c = match centre_x {
            Some(fx) if along.x.abs() > 0.3 => c + along * ((fx - c.x) / along.x),
            _ => c,
        };
        let down = EVec2::new(-along.y, along.x); // towards the ground on screen
        if i == 0 {
            let half = 160.0 * k;
            line(p, c - along * half, c - along * 30.0 * k, w, col);
            line(p, c + along * 30.0 * k, c + along * half, w, col);
            continue;
        }
        let (gap, len) = (22.0 * k, 34.0 * k);
        let tick = if i > 0 { down * 7.0 * k } else { -down * 7.0 * k };
        for s in [-1.0, 1.0] {
            let a = c + along * s * gap;
            let b = c + along * s * (gap + len);
            if i > 0 {
                line(p, a, b, w, col);
            } else {
                // dashed below the horizon
                for d in 0..3 {
                    let t0 = d as f32 / 3.0;
                    let t1 = t0 + 0.2;
                    line(p, a + (b - a) * t0, a + (b - a) * t1, w, col);
                }
            }
            line(p, a, a + tick, w, col);
            text(p, b + along * s * 12.0 * k, Align2::CENTER_CENTER, format!("{}", el_deg.abs() as i32), 11.0 * k, col);
        }
    }
    let _ = ppd;
    let p = &full;
    // Heading tape (top of the field).
    let hdg = ac.heading().to_degrees().rem_euclid(360.0);
    let (fw, fh) = (field.width(), field.height());
    let top = field.min.y + fh * 0.075;
    let tape_w = fw * 0.30;
    let cx = field.center().x;
    let ppd_t = tape_w / 30.0;
    let base = (hdg / 5.0).floor() as i32 * 5;
    let tp = p.with_clip_rect(Rect::from_center_size(Pos2::new(cx, top), EVec2::new(tape_w * 2.0, 40.0 * k)));
    for kk in -7..=7 {
        let d = base + kk * 5;
        let x = cx + ((d as f32 - hdg + 180.0).rem_euclid(360.0) - 180.0) * ppd_t;
        let major = d.rem_euclid(10) == 0;
        line(&tp, Pos2::new(x, top), Pos2::new(x, top - if major { 8.0 * k } else { 4.0 * k }), w, col);
        if major {
            text(&tp, Pos2::new(x, top - 16.0 * k), Align2::CENTER_CENTER, format!("{:02}", d.rem_euclid(360) / 10), 11.0 * k, col);
        }
    }
    poly(p, vec![Pos2::new(cx - 5.0 * k, top + 9.0 * k), Pos2::new(cx, top + 2.0 * k), Pos2::new(cx + 5.0 * k, top + 9.0 * k)], w, col);
    // Airspeed (left) and altitude (right) boxes with short scales.
    let mid_y = field.center().y + fh * 0.02;
    let ias = ac.airspeed() / KT;
    let alt = ac.pos.y * FT;
    let lx = field.min.x + fw * 0.12;
    let rx = field.max.x - fw * 0.12;
    for (x, v, step, left) in [(lx, ias, 10.0, true), (rx, alt, 100.0, false)] {
        let spp = fh * 0.13 / (step * 5.0);
        let sp = p.with_clip_rect(Rect::from_center_size(Pos2::new(x, mid_y), EVec2::new(fw * 0.16, fh * 0.30)));
        let b0 = (v / step).floor() as i32;
        for j in -4..=5 {
            let val = (b0 + j) as f32 * step;
            let y = mid_y - (val - v) * spp;
            let tx = if left { x + 22.0 * k } else { x - 22.0 * k };
            let len = if (b0 + j) % 5 == 0 { 8.0 * k } else { 4.0 * k };
            line(&sp, Pos2::new(tx, y), Pos2::new(if left { tx + len } else { tx - len }, y), w, col);
        }
        let s = if left { format!("{:.0}", v) } else { format!("{:.0}", (v / 10.0).round() * 10.0) };
        let r = Rect::from_center_size(Pos2::new(x, mid_y), EVec2::new((fw * 0.10).max(50.0), (fh * 0.045).max(20.0)));
        p.rect_stroke(r, 0, Stroke::new(w, col), egui::StrokeKind::Outside);
        text(p, r.center(), Align2::CENTER_CENTER, s, 13.0 * k, col);
    }
    // AP / selected values above the boxes.
    if sim.ap.engaged {
        if let Some(t) = sim.ap.ias {
            text(p, Pos2::new(lx, mid_y - fh * 0.18), Align2::CENTER_CENTER, format!("{:.0}", t / KT), 11.0 * k, col);
        }
        if let Some(a) = sim.ap.alt {
            text(p, Pos2::new(rx, mid_y - fh * 0.18), Align2::CENTER_CENTER, format!("{:.0}", a * FT), 11.0 * k, col);
        }
    }
    // Vertical speed under the altitude, AoA / Mach / G at bottom-left.
    text(p, Pos2::new(rx, mid_y + fh * 0.18), Align2::CENTER_CENTER, format!("{:+.0}", ac.vertical_speed() * FT * 60.0), 11.0 * k, col);
    let bl = Pos2::new(field.min.x + fw * 0.03, field.max.y - fh * 0.17);
    text(p, bl, Align2::LEFT_CENTER, format!("α {:.1}", ac.alpha().to_degrees()), 11.0 * k, col);
    text(p, bl + EVec2::new(0.0, 16.0 * k), Align2::LEFT_CENTER, format!("M {:.2}", ac.mach()), 11.0 * k, col);
    text(p, bl + EVec2::new(0.0, 32.0 * k), Align2::LEFT_CENTER, format!("G {:.1}", ac.g_load()), 11.0 * k, col);
    // Bottom-right: thrust and reheat.
    let br = Pos2::new(field.max.x - fw * 0.03, field.max.y - fh * 0.17);
    text(p, br, Align2::RIGHT_CENTER, format!("THR {:.0}", sim.ctl.throttle * 100.0), 11.0 * k, col);
    if ac.afterburner() > 0.02 {
        text(p, br + EVec2::new(0.0, 16.0 * k), Align2::RIGHT_CENTER, "AB", 12.0 * k, col);
    }
    if sim.ac.gear_pos() > 0.5 {
        text(p, br + EVec2::new(0.0, 32.0 * k), Align2::RIGHT_CENTER, "GEAR", 11.0 * k, col);
    }
    if sim.ap.engaged || sim.bot.is_some() {
        text(p, Pos2::new(field.center().x, field.max.y - fh * 0.03), Align2::CENTER_CENTER, if sim.bot.is_some() { "AUTO" } else { "AP" }, 11.0 * k, col);
    }
    // Bank scale at the bottom.
    let bc = Pos2::new(field.center().x, field.max.y - fh * 0.17);
    let rr = fh * 0.09;
    for d in [-60.0f32, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0] {
        let a = std::f32::consts::PI + d.to_radians();
        let len = if d == 0.0 || d.abs() == 30.0 || d.abs() == 60.0 { 8.0 * k } else { 4.5 * k };
        line(p, super::polar(bc, rr, a), super::polar(bc, rr + len, a), w, col);
    }
    let pa = std::f32::consts::PI + roll;
    let tip = super::polar(bc, rr - 1.0 * k, pa);
    let inn = super::polar(bc, rr - 9.0 * k, pa);
    let n = (tip - inn).normalized().rot90() * 4.0 * k;
    poly(p, vec![inn + n, tip, inn - n], w, col);
    if ac.stall_warning() && (sim.time * 4.0).fract() < 0.6 {
        text(p, Pos2::new(field.center().x, field.center().y + 50.0 * k), Align2::CENTER_CENTER, "AOA", 16.0 * k, pal::RED);
    }
    let _ = (up, f_inst);
}
