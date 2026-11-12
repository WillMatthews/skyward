//! Toast notifications: short messages that slide in under the top bar and
//! fade out (replacing ad-hoc banners).

use super::{f_semi, f_ui, pal, panel_with};
use egui::{Align2, Color32, Pos2, Rect, Stroke, Vec2};

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Info,
    Good,
    Warn,
    Bad,
}

#[derive(Clone, Debug)]
pub struct Toast {
    pub title: String,
    pub body: String,
    pub kind: Kind,
    pub age: f32,
    pub life: f32,
}

#[derive(Default)]
pub struct Toasts {
    pub list: Vec<Toast>,
}

impl Toasts {
    pub fn push(&mut self, kind: Kind, title: impl Into<String>, body: impl Into<String>, life: f32) {
        let title = title.into();
        let body = body.into();
        // Same message again: refresh instead of stacking.
        if let Some(t) = self.list.iter_mut().find(|t| t.title == title && t.body == body) {
            t.age = t.age.min(0.3);
            t.life = t.life.max(life);
            return;
        }
        self.list.push(Toast { title, body, kind, age: 0.0, life });
        if self.list.len() > 4 {
            self.list.remove(0);
        }
    }

    pub fn tick(&mut self, dt: f32) {
        for t in &mut self.list {
            t.age += dt;
        }
        self.list.retain(|t| t.age < t.life);
    }

    /// Draw stacked under `top` (centre x of `screen`).
    pub fn draw(&self, p: &egui::Painter, screen: Rect, top: f32) {
        let mut y = top;
        for t in self.list.iter().rev() {
            let fade_in = (t.age / 0.25).clamp(0.0, 1.0);
            let fade_out = ((t.life - t.age) / 0.6).clamp(0.0, 1.0);
            let a = fade_in.min(fade_out);
            let slide = (1.0 - fade_in) * -10.0;
            let col = match t.kind {
                Kind::Info => pal::ACCENT,
                Kind::Good => pal::GREEN,
                Kind::Warn => pal::AMBER,
                Kind::Bad => pal::RED,
            };
            let tg = p.layout_no_wrap(t.title.clone(), f_semi(14.0), pal::TEXT.gamma_multiply(a));
            let bg = p.layout(t.body.clone(), f_ui(13.0), pal::TEXT2.gamma_multiply(a), 520.0);
            let w = tg.size().x.max(bg.size().x) + 44.0;
            let h = tg.size().y + if t.body.is_empty() { 0.0 } else { bg.size().y + 4.0 } + 20.0;
            let r = Rect::from_min_size(Pos2::new(screen.center().x - w * 0.5, y + slide), Vec2::new(w, h));
            panel_with(p, r, 10, Color32::from_rgba_premultiplied(13, 16, 22, (225.0 * a) as u8));
            p.rect_filled(Rect::from_min_size(r.min + Vec2::new(12.0, 12.0), Vec2::new(4.0, h - 24.0)), 2, col.gamma_multiply(a));
            p.galley(r.min + Vec2::new(26.0, 10.0), tg.clone(), pal::TEXT);
            if !t.body.is_empty() {
                p.galley(r.min + Vec2::new(26.0, 14.0 + tg.size().y), bg, pal::TEXT2);
            }
            let _ = (Align2::CENTER_TOP, Stroke::NONE);
            y += h + 8.0;
        }
    }
}
