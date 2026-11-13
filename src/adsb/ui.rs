//! Live traffic in the interface: the moving-map layer (symbol, callsign,
//! altitude / flight level, type, a speed vector) and callsign tags over
//! the aircraft in the 3-D view (distance-faded, like the multiplayer ones).

use super::{LiveTraffic, synth::Phase};
use crate::ui::map::View;
use crate::ui::{alpha, f_inst_b, f_semi, f_ui, pal};
use egui::{Align2, Color32, Pos2, Rect, Stroke, Vec2};
use glam::{Vec3, vec2};

const TAG_NEAR: f32 = 2000.0;
const TAG_FAR: f32 = 12_000.0;

/// "FL350" above the transition altitude (~6,000 ft), else "A045" (hundreds of feet).
pub fn alt_text(alt_m: f32, on_ground: bool) -> String {
    if on_ground {
        return "GND".into();
    }
    let ft = alt_m / 0.3048;
    if ft >= 6000.0 { format!("FL{:03.0}", (ft / 100.0).round()) } else { format!("A{:03.0}", (ft / 100.0).round().max(0.0)) }
}

/// Vertical trend arrow.
fn trend(vs: f32) -> &'static str {
    if vs > 2.5 {
        "↑"
    } else if vs < -2.5 {
        "↓"
    } else {
        ""
    }
}

/// Live aircraft on the moving map. Labels thin out when the map is zoomed out.
pub fn map_layer(p: &egui::Painter, view: &View, lt: &LiveTraffic, dark: bool) {
    if !lt.settings.enabled {
        return;
    }
    let col = if dark { Color32::from_rgb(120, 220, 255) } else { Color32::from_rgb(0, 95, 140) };
    let ground_col = if dark { Color32::from_rgb(150, 160, 175) } else { Color32::from_rgb(90, 96, 110) };
    let tc = if dark { pal::TEXT } else { Color32::from_rgb(20, 24, 30) };
    // Points per km: labels only when there is room for them.
    let ppkm = view.s * 1000.0;
    let mut taken: Vec<Rect> = Vec::new();
    let mut list: Vec<_> = lt.aircraft.values().filter(|a| a.shown).collect();
    // Airborne first (on top of ground clutter), nearest first.
    list.sort_by(|a, b| a.on_ground.cmp(&b.on_ground).then(a.dist.total_cmp(&b.dist)));
    for a in list {
        let q = view.to_screen(vec2(a.pos.x, a.pos.z));
        if !view.rect.shrink(4.0).contains(q) {
            continue;
        }
        if a.on_ground && ppkm < 8.0 {
            continue; // airport clutter at small scales
        }
        let c = if a.emergency.is_some() || matches!(a.squawk, Some(7500 | 7600 | 7700)) {
            pal::RED
        } else if a.on_ground {
            ground_col
        } else {
            col
        };
        let size = if a.on_ground { 7.0 } else { 9.5 };
        crate::ui::map::aircraft_symbol(p, q, view.screen_angle(a.anim.heading.to_radians()), size, c);
        // One-minute speed vector.
        if !a.on_ground {
            let ahead = view.to_screen(vec2(a.pos.x + a.vel.x * 60.0, a.pos.z + a.vel.z * 60.0));
            if q.distance(ahead) > 6.0 {
                p.line_segment([q, ahead], Stroke::new(1.0, alpha(c, 0.7)));
            }
        }
        let show_label = (!a.on_ground && ppkm > 1.2) || ppkm > 70.0;
        if !show_label {
            continue;
        }
        let l1 = a.label();
        let l2 = format!("{}{} {}", alt_text(a.pos.y, a.on_ground), trend(a.vel.y), a.type_label());
        let at = q + Vec2::new(9.0, -8.0);
        let r = Rect::from_min_size(at - Vec2::new(2.0, 14.0), Vec2::new(12.0 + 6.6 * l1.len().max(l2.len()) as f32, 28.0));
        if taken.iter().any(|t| t.intersects(r)) {
            continue;
        }
        taken.push(r);
        p.text(at, Align2::LEFT_BOTTOM, l1, f_inst_b(10.5), tc);
        p.text(at, Align2::LEFT_TOP, l2, f_inst_b(9.5), alpha(tc, 0.75));
    }
}

/// Callsign tags over live aircraft in the 3-D view.
pub fn tags(ctx: &egui::Context, lt: &LiveTraffic, me: Vec3, alt_unit: crate::ui::AltUnit) {
    if !lt.settings.enabled || !lt.settings.tags {
        return;
    }
    let Some(cam) = lt.cam else { return };
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::NULL.with("adsb_tags")));
    // Nearest first; a tag that would overlap a nearer one is left out.
    let mut list: Vec<_> = lt.aircraft.values().filter(|a| a.shown).map(|a| (a.pos.distance(cam.0), a)).collect();
    list.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut taken: Vec<Rect> = Vec::new();
    for (d, a) in list {
        if d > TAG_FAR || d < 30.0 {
            continue;
        }
        if a.anim.phase == Some(Phase::Parked) && d > 1500.0 {
            continue;
        }
        let Some((q, _)) = crate::ui::mp::project(&cam, screen, a.pos + Vec3::Y * (a.stand.span * 0.12 + 3.0)) else { continue };
        let k = 1.0 - ((d - TAG_NEAR) / (TAG_FAR - TAG_NEAR)).clamp(0.0, 1.0);
        let k = k * k;
        if k < 0.04 {
            continue;
        }
        let col = Color32::from_rgb(120, 220, 255);
        let m = q + Vec2::new(0.0, -4.0);
        p.add(egui::Shape::convex_polygon(vec![m + Vec2::new(-4.0, -6.0), m + Vec2::new(4.0, -6.0), m], alpha(col, k), Stroke::NONE));
        let dd = a.pos.distance(me);
        let dist = if dd < 1000.0 { format!("{dd:.0} m") } else { format!("{:.1} km", dd / 1000.0) };
        let sub = format!("{} · {} · {}", a.type_label(), alt_text_unit(a.pos.y, a.on_ground, alt_unit), dist);
        let g1 = p.layout_no_wrap(a.label(), f_semi(13.0), pal::WHITE);
        let g2 = p.layout_no_wrap(sub, f_ui(10.5), pal::TEXT);
        let bw = g1.size().x.max(g2.size().x) + 16.0;
        let bg = Rect::from_center_size(q + Vec2::new(0.0, -26.0), Vec2::new(bw, 33.0));
        if taken.iter().any(|t| t.intersects(bg.expand(2.0))) {
            continue;
        }
        taken.push(bg);
        p.rect_filled(bg, 7, Color32::from_black_alpha((90.0 * k) as u8));
        p.rect_filled(Rect::from_min_size(bg.min + Vec2::new(0.0, 6.0), Vec2::new(2.5, bg.height() - 12.0)), 1, alpha(col, k));
        p.galley(Pos2::new(bg.center().x - g1.size().x * 0.5, bg.min.y + 2.0), g1, alpha(pal::WHITE, k));
        p.galley(Pos2::new(bg.center().x - g2.size().x * 0.5, bg.min.y + 18.0), g2, alpha(pal::TEXT, k * 0.9));
    }
}

fn alt_text_unit(alt_m: f32, on_ground: bool, u: crate::ui::AltUnit) -> String {
    if on_ground {
        return "ground".into();
    }
    match u {
        crate::ui::AltUnit::Metres => format!("{:.0} m", alt_m),
        _ => alt_text(alt_m, false),
    }
}

/// The traffic status line for the settings / pause screens.
pub fn status_line(lt: &LiveTraffic) -> String {
    if lt.settings.enabled { format!("{} · {}", lt.status, super::ATTRIBUTION) } else { "Live traffic is off".into() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn altitude_labels() {
        assert_eq!(alt_text(10_668.0, false), "FL350");
        assert_eq!(alt_text(914.4, false), "A030");
        assert_eq!(alt_text(30.0, true), "GND");
        assert_eq!(trend(-4.0), "↓");
    }
}
