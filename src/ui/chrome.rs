//! Shared chrome for menus and in-flight panels: one header style (title,
//! subtitle, key cap, close ×), key hints, badges. Keeps the settings screen,
//! the failures / radio / comms panels, the server browser and the pause menu
//! looking like one product.

use super::menus::{Btn, button};
use super::{f_display, f_semi, f_ui, pal, sp};
use egui::{Color32, Pos2, Rect, Sense, Stroke, Ui, Vec2};

/// A close button (a drawn ×: the fonts have no ✕). Returns true when clicked.
pub fn close_x(ui: &mut Ui, r: Rect, id: impl std::hash::Hash + std::fmt::Debug) -> bool {
    let resp = button(ui, r, id, "", Btn::Ghost, true).on_hover_text("Close");
    let c = r.center();
    let k = r.height() * 0.18;
    let st = Stroke::new(1.6, if resp.hovered() { pal::TEXT } else { pal::TEXT2 });
    ui.painter().line_segment([c + Vec2::new(-k, -k), c + Vec2::new(k, k)], st);
    ui.painter().line_segment([c + Vec2::new(k, -k), c + Vec2::new(-k, k)], st);
    resp.clicked()
}

/// A key cap ("F2", "Esc") at `left_center`; returns its rect.
pub fn keycap(p: &egui::Painter, left_center: Pos2, key: &str) -> Rect {
    let g = p.layout_no_wrap(key.to_string(), f_semi(11.5), pal::TEXT2);
    let r = Rect::from_min_size(Pos2::new(left_center.x, left_center.y - 11.0), Vec2::new(g.size().x + 14.0, 22.0));
    p.rect_filled(r, 5, pal::RAISED);
    p.rect_stroke(r, 5, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    p.galley(r.center() - g.size() * 0.5, g, pal::TEXT2);
    r
}

/// Width of `hints` as drawn by [`key_hints`].
pub fn key_hints_width(p: &egui::Painter, hints: &[(&str, &str)]) -> f32 {
    hints
        .iter()
        .map(|(k, l)| p.layout_no_wrap(k.to_string(), f_semi(11.5), pal::TEXT2).size().x + 14.0 + 8.0 + p.layout_no_wrap(l.to_string(), f_ui(12.5), pal::TEXT3).size().x)
        .sum::<f32>()
        + 24.0 * (hints.len() as f32 - 1.0).max(0.0)
}

/// Key hints ("[Tab] Category") centred in `r`, if they all fit.
pub fn key_hints(p: &egui::Painter, r: Rect, hints: &[(&str, &str)]) {
    let n = hints.len();
    if n == 0 || key_hints_width(p, hints) > r.width() {
        return;
    }
    let w = key_hints_width(p, &hints[..n]);
    let mut x = r.center().x - w * 0.5;
    for (k, l) in &hints[..n] {
        let kr = keycap(p, Pos2::new(x, r.center().y), k);
        x = kr.max.x + 8.0;
        let g = p.layout_no_wrap(l.to_string(), f_ui(12.5), pal::TEXT3);
        let gw = g.size().x;
        p.galley(Pos2::new(x, r.center().y - g.size().y * 0.5), g, pal::TEXT3);
        x += gw + 24.0;
    }
}

/// A small outlined badge ("Restart", "Soon") at `left_center`; returns its rect.
pub fn badge(p: &egui::Painter, left_center: Pos2, text: &str, col: Color32) -> Rect {
    let g = p.layout_no_wrap(text.to_uppercase(), super::f_inst_b(9.5), col);
    let r = Rect::from_min_size(Pos2::new(left_center.x, left_center.y - 8.5), Vec2::new(g.size().x + 12.0, 17.0));
    p.rect_stroke(r, 8.5, Stroke::new(1.0, super::alpha(col, 0.8)), egui::StrokeKind::Inside);
    p.galley(r.center() - g.size() * 0.5, g, col);
    r
}

/// The header of an in-flight panel laid out with egui (failures, radios):
/// title, a subtitle line, the key that toggles the panel and a close ×.
/// `extra` adds buttons left of the close button (right-to-left layout).
/// Returns true when closed.
pub fn panel_header(ui: &mut Ui, title: &str, sub: &str, sub_col: Color32, key: Option<&str>, id: &str, extra: impl FnOnce(&mut Ui)) -> bool {
    let mut closed = false;
    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new(title).font(f_display(22.0)).color(pal::TEXT));
            ui.label(egui::RichText::new(sub).font(f_ui(12.5)).color(sub_col));
        });
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
            let (r, _) = ui.allocate_exact_size(Vec2::new(32.0, 32.0), Sense::hover());
            closed = close_x(ui, r, (id, "close"));
            if let Some(k) = key {
                let w = ui.painter().layout_no_wrap(k.to_string(), f_semi(11.5), pal::TEXT2).size().x + 14.0;
                let (r, _) = ui.allocate_exact_size(Vec2::new(w, 32.0), Sense::hover());
                keycap(ui.painter(), Pos2::new(r.min.x, r.center().y), k);
            }
            ui.add_space(sp::XS);
            extra(ui);
        });
    });
    closed
}

/// The label of the first key bound to an action ("F2"), for headers and hints.
pub fn key_of(st: &super::UiState, k: crate::sim::Key) -> Option<String> {
    st.settings.controls.keys_of(k).first().map(|c| crate::input::keys::label(*c))
}
