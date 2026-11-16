//! Start-up progress: what has to finish before the aircraft can fly (the
//! loading screen's steps and bar, the main menu's "preparing" card) and
//! the first-run downloads that carry on in the background (a quiet pill
//! on the menus). Filled each frame by main.rs; nothing here blocks.

use super::{alpha, f_ui, pal};
use egui::{Align2, Color32, Painter, Pos2, Rect, Stroke, Vec2};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Step {
    Waiting,
    Active,
    Done,
}

/// The departure being prepared (None: ready to fly, or nothing loading).
#[derive(Clone, Default, Debug)]
pub struct Preparing {
    /// What must finish before flying, in order.
    pub steps: Vec<(&'static str, Step)>,
    /// The current step in words ("Scenery · 12 / 27 tiles").
    pub status: String,
    /// Overall progress 0..1 (None: indeterminate).
    pub frac: Option<f32>,
}

#[derive(Clone, Default, Debug)]
pub struct Boot {
    pub preparing: Option<Preparing>,
    /// Downloads / imports that don't hold up flying ("Airport layouts (first run, 156 MB)").
    pub background: Vec<String>,
}

impl Preparing {
    /// Steps with weights → overall fraction; `step_frac` is the active step's own progress.
    pub fn new(steps: Vec<(&'static str, Step, f32)>, status: String, step_frac: Option<f32>) -> Preparing {
        let total: f32 = steps.iter().map(|s| s.2).sum::<f32>().max(1e-3);
        let mut done = 0.0;
        for &(_, st, w) in &steps {
            match st {
                Step::Done => done += w,
                Step::Active => done += w * step_frac.unwrap_or(0.0).clamp(0.0, 1.0),
                Step::Waiting => {}
            }
        }
        Preparing { steps: steps.into_iter().map(|(n, s, _)| (n, s)).collect(), status, frac: Some(done / total) }
    }
}

/// The step chips in a row from `at` (left-top); returns the row's width.
pub fn steps_row(p: &Painter, at: Pos2, steps: &[(&'static str, Step)], time: f32) -> f32 {
    let mut x = at.x;
    let cy = at.y + 8.0;
    for (i, (name, st)) in steps.iter().enumerate() {
        if i > 0 {
            p.line_segment([Pos2::new(x, cy), Pos2::new(x + 14.0, cy)], Stroke::new(1.0, pal::HAIRLINE_HI));
            x += 22.0;
        }
        let c = Pos2::new(x + 6.0, cy);
        match st {
            Step::Done => {
                p.circle_filled(c, 6.0, pal::GREEN);
                let k = Color32::from_rgb(8, 30, 18);
                p.line_segment([c + Vec2::new(-3.0, 0.2), c + Vec2::new(-0.8, 2.5)], Stroke::new(1.6, k));
                p.line_segment([c + Vec2::new(-0.8, 2.5), c + Vec2::new(3.2, -2.4)], Stroke::new(1.6, k));
            }
            Step::Active => {
                let pulse = (time * 3.0).sin() * 0.5 + 0.5;
                p.circle_stroke(c, 6.0, Stroke::new(1.5, pal::ACCENT));
                p.circle_filled(c, 2.5 + pulse * 1.5, pal::ACCENT);
            }
            Step::Waiting => {
                p.circle_stroke(c, 6.0, Stroke::new(1.2, pal::TEXT3));
            }
        }
        let col = match st {
            Step::Done => pal::TEXT2,
            Step::Active => pal::TEXT,
            Step::Waiting => pal::TEXT3,
        };
        let r = p.text(Pos2::new(x + 18.0, cy), Align2::LEFT_CENTER, *name, f_ui(12.5), col);
        x = r.max.x + 8.0;
    }
    x - at.x
}

/// An indeterminate spinner (an arc going round).
pub fn spinner(p: &Painter, c: Pos2, r: f32, time: f32, col: Color32) {
    let a0 = time * 5.0;
    let pts: Vec<Pos2> = (0..=16).map(|i| a0 + i as f32 / 16.0 * 4.2).map(|a| c + Vec2::angled(a) * r).collect();
    p.circle_stroke(c, r, Stroke::new(1.5, alpha(col, 0.2)));
    p.add(egui::Shape::line(pts, Stroke::new(1.5, col)));
}

/// "In the background: …" as one line of text.
pub fn background_line(bg: &[String]) -> String {
    format!("In the background: {}", bg.join(" · "))
}

/// The quiet pill for background downloads, bottom-right of `screen`.
pub fn background_pill(p: &Painter, screen: Rect, bg: &[String], time: f32) {
    if bg.is_empty() {
        return;
    }
    let text = bg.join(" · ");
    let font = f_ui(12.0);
    let galley = p.layout_no_wrap(text, font, pal::TEXT2);
    let w = galley.size().x + 46.0;
    // (Right-aligned with the main menu's cards, level with its credits line.)
    let right = screen.max.x - (screen.width() * 0.05).max(40.0);
    let r = Rect::from_min_size(Pos2::new(right - w, screen.max.y - 18.0 - 28.0), Vec2::new(w, 28.0));
    p.rect_filled(r, 14, pal::PANEL);
    p.rect_stroke(r, 14, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    spinner(p, Pos2::new(r.min.x + 16.0, r.center().y), 6.0, time, pal::ACCENT);
    p.galley(Pos2::new(r.min.x + 30.0, r.center().y - galley.size().y / 2.0), galley, pal::TEXT2);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overall_fraction_weights_steps() {
        let p = Preparing::new(vec![("Airports", Step::Done, 1.0), ("Aircraft", Step::Done, 1.0), ("Scenery", Step::Active, 2.0)], String::new(), Some(0.5));
        assert!((p.frac.unwrap() - 0.75).abs() < 1e-6);
        let p = Preparing::new(vec![("Airports", Step::Active, 1.0), ("Scenery", Step::Waiting, 3.0)], String::new(), None);
        assert_eq!(p.frac, Some(0.0));
    }
}
