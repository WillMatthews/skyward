//! G-effects (Settings › Flight & Realism): the pilot's vision greys out,
//! tunnels and blacks out under sustained positive g, and reddens under
//! negative g. A simple onset / recovery model on a "g stress" budget, drawn
//! as a radial vignette over the whole view (instruments included).

use super::UiState;
use crate::flight::Category;
use crate::sim::Sim;
use egui::{Color32, Pos2, Shape, epaint};

/// Stress state: (positive, negative), 0 = clear .. 1 = blacked / redded out.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GStress {
    pub pos: f32,
    pub neg: f32,
}

/// Grey-out onset (g) for a type: fast-jet pilots wear g-suits and strain.
pub fn onset(cat: Category) -> f32 {
    match cat {
        Category::FastJet => 7.0,
        Category::Warbird => 5.0,
        _ => 4.5,
    }
}

/// Advance the stress by `dt` at load factor `g`.
pub fn step(s: GStress, g: f32, onset_g: f32, dt: f32) -> GStress {
    let mut s = s;
    // Above the onset it builds (faster the harder you pull), below it recovers.
    let over = g - onset_g;
    s.pos = if over > 0.0 { s.pos + over * 0.12 * dt } else { s.pos - (0.25 + (onset_g - g).max(0.0) * 0.05) * dt };
    let under = -1.8 - g;
    s.neg = if under > 0.0 { s.neg + under * 0.25 * dt } else { s.neg - 0.35 * dt };
    s.pos = s.pos.clamp(0.0, 1.0);
    s.neg = s.neg.clamp(0.0, 1.0);
    s
}

/// Update and draw (call each flight frame).
pub fn draw(ctx: &egui::Context, st: &mut UiState, sim: &Sim) {
    let on = st.settings.realism.g_effects && sim.ac.crashed.is_none() && sim.bot.is_none() && sim.def().category != Category::Drone;
    let dt = if sim.paused { 0.0 } else { st.dt.min(0.1) * sim.warp.min(4) as f32 };
    st.g_stress = if on { step(st.g_stress, sim.ac.g_load(), onset(sim.def().category), dt) } else { GStress::default() };
    let s = st.g_stress;
    if s.pos < 0.02 && s.neg < 0.02 {
        return;
    }
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Middle, egui::Id::new("g_effects")));
    let (k, col) = if s.pos >= s.neg { (s.pos, Color32::from_rgb(4, 4, 6)) } else { (s.neg, Color32::from_rgb(120, 6, 8)) };
    // Tunnel: the clear radius shrinks, the rim darkens; past 0.85 everything goes.
    let r_full = screen.size().length() * 0.55;
    let clear = r_full * (1.0 - k * 1.1).max(0.0);
    let n = 64;
    let mut m = epaint::Mesh::default();
    let c = screen.center();
    let rim_a = (k * 1.4).min(1.0);
    let centre_a = ((k - 0.75) / 0.25).clamp(0.0, 1.0);
    let rgba = |a: f32| Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), (a * 255.0) as u8);
    m.colored_vertex(c, rgba(centre_a));
    for i in 0..=n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        let d = egui::Vec2::new(a.cos(), a.sin());
        m.colored_vertex(c + d * clear, rgba(centre_a));
        m.colored_vertex(c + d * r_full, rgba(rim_a));
    }
    for i in 0..n as u32 {
        let (a0, b0, a1, b1) = (1 + 2 * i, 2 + 2 * i, 3 + 2 * i, 4 + 2 * i);
        m.add_triangle(0, a0, a1);
        m.add_triangle(a0, b0, b1);
        m.add_triangle(a0, b1, a1);
    }
    p.add(Shape::mesh(m));
    // Grey-out also drains colour: a flat grey veil.
    if s.pos > 0.2 && s.pos >= s.neg {
        p.rect_filled(screen, 0, Color32::from_black_alpha(((s.pos - 0.2) * 140.0) as u8));
    }
    let _ = Pos2::ZERO;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn onset_and_recovery() {
        let o = onset(Category::Trainer);
        let mut s = GStress::default();
        // 1 g for a minute: nothing.
        for _ in 0..600 {
            s = step(s, 1.0, o, 0.1);
        }
        assert_eq!(s, GStress::default());
        // A sustained 7 g pull greys out within seconds and blacks out by ~5 s.
        let mut t = 0.0;
        while s.pos < 1.0 && t < 30.0 {
            s = step(s, 7.0, o, 0.1);
            t += 0.1;
        }
        assert!((2.0..8.0).contains(&t), "blackout after {t:.1} s");
        // Unloading recovers in a few seconds.
        let mut t = 0.0;
        while s.pos > 0.0 && t < 30.0 {
            s = step(s, 1.0, o, 0.1);
            t += 0.1;
        }
        assert!(t < 6.0, "recovery {t:.1} s");
        // A fast jet tolerates the same pull longer.
        let (mut a, mut b) = (GStress::default(), GStress::default());
        for _ in 0..30 {
            a = step(a, 7.5, onset(Category::Trainer), 0.1);
            b = step(b, 7.5, onset(Category::FastJet), 0.1);
        }
        assert!(b.pos < a.pos);
        // Negative g reds out.
        let mut s = GStress::default();
        for _ in 0..40 {
            s = step(s, -3.0, o, 0.1);
        }
        assert!(s.neg > 0.5 && s.pos == 0.0);
    }
}
