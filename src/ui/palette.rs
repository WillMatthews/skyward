//! Accessibility palettes: colour-vision-deficiency variants of the design
//! system's status colours and a high-contrast mode, applied to every
//! interface vertex after tessellation (main.rs, ui/canvas.rs) so that each
//! panel, HUD and map picks them up without knowing about them.
//!
//! * The six semantic colours (green = engaged / good, red = warning, amber =
//!   caution, cyan = selected, accent blue, magenta = route) are anchors:
//!   each mode moves them to a hand-tuned target that stays distinguishable
//!   under that deficiency (checked against the Machado et al. 2009
//!   simulation in the tests). Every other saturated colour follows by
//!   interpolating the anchors' hue / saturation / value change along the
//!   hue circle; greys (text, panels) are left alone, so text contrast is
//!   unchanged by the colour-blind modes.
//! * High contrast lifts mid greys (secondary text) towards white and makes
//!   translucent dark panels more opaque.
//!
//! The standard palette costs nothing: `apply` returns at once.

use super::pal;
use super::settings::Palette;
use egui::Color32;

/// What the transform does this frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Mode {
    pub palette: Palette,
    pub high_contrast: bool,
}

impl Mode {
    pub fn identity(self) -> bool {
        self.palette == Palette::Standard && !self.high_contrast
    }
}

/// (source, target) for the semantic colours of a palette.
fn anchors(p: Palette) -> [(Color32, Color32); 6] {
    let c = Color32::from_rgb;
    match p {
        // Red-green deficiencies: "good" becomes sky blue, warnings a crimson
        // pink, cautions yellow; cyan / magenta move apart in lightness.
        Palette::Deuteranopia | Palette::Protanopia => [
            (pal::GREEN, c(60, 165, 255)),
            (pal::RED, c(255, 70, 110)),
            (pal::AMBER, c(255, 200, 40)),
            (pal::CYAN, c(200, 240, 255)),
            (pal::ACCENT, pal::ACCENT),
            (pal::MAGENTA, c(255, 150, 225)),
        ],
        // Blue-yellow: cyan (selected) leaves green for periwinkle, magenta goes violet.
        Palette::Tritanopia => {
            [(pal::GREEN, pal::GREEN), (pal::RED, pal::RED), (pal::AMBER, pal::AMBER), (pal::CYAN, c(130, 160, 255)), (pal::ACCENT, pal::ACCENT), (pal::MAGENTA, c(200, 100, 255))]
        }
        Palette::Standard => {
            [(pal::GREEN, pal::GREEN), (pal::RED, pal::RED), (pal::AMBER, pal::AMBER), (pal::CYAN, pal::CYAN), (pal::ACCENT, pal::ACCENT), (pal::MAGENTA, pal::MAGENTA)]
        }
    }
}

fn to_hsv(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    let mx = r.max(g).max(b);
    let mn = r.min(g).min(b);
    let d = mx - mn;
    let h = if d <= 1e-6 {
        0.0
    } else if mx == r {
        60.0 * ((g - b) / d).rem_euclid(6.0)
    } else if mx == g {
        60.0 * ((b - r) / d + 2.0)
    } else {
        60.0 * ((r - g) / d + 4.0)
    };
    (h, if mx <= 1e-6 { 0.0 } else { d / mx }, mx)
}

fn from_hsv(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let h = h.rem_euclid(360.0) / 60.0;
    let c = v * s;
    let x = c * (1.0 - (h % 2.0 - 1.0).abs());
    let (r, g, b) = match h as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = v - c;
    (r + m, g + m, b + m)
}

fn hsv_of(c: Color32) -> (f32, f32, f32) {
    to_hsv(c.r() as f32 / 255.0, c.g() as f32 / 255.0, c.b() as f32 / 255.0)
}

/// Per-anchor change: (source hue, hue shift, saturation ratio, value ratio).
fn deltas(p: Palette) -> Vec<(f32, f32, f32, f32)> {
    let mut v: Vec<_> = anchors(p)
        .iter()
        .map(|(s, t)| {
            let (hs, ss, vs) = hsv_of(*s);
            let (ht, st, vt) = hsv_of(*t);
            let dh = (ht - hs + 540.0).rem_euclid(360.0) - 180.0;
            (hs, dh, st / ss.max(1e-3), vt / vs.max(1e-3))
        })
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The colour-vision transform of an opaque colour.
fn cvd(c: (f32, f32, f32), d: &[(f32, f32, f32, f32)]) -> (f32, f32, f32) {
    let (h, s, v) = to_hsv(c.0, c.1, c.2);
    // Greys and near-greys (text, the blue-grey panels) stay: weight by
    // chroma, full effect from moderately colourful colours up.
    let w = ((s * v - 0.11) / 0.14).clamp(0.0, 1.0);
    if w <= 0.0 || d.is_empty() {
        return c;
    }
    // Neighbouring anchors around the hue circle.
    let n = d.len();
    let i = d.iter().position(|a| a.0 > h).unwrap_or(n);
    let (a, b) = (d[(i + n - 1) % n], d[i % n]);
    let span = (b.0 - a.0).rem_euclid(360.0).max(1e-3);
    let t = ((h - a.0).rem_euclid(360.0) / span).clamp(0.0, 1.0);
    let lerp = |x: f32, y: f32| x + (y - x) * t;
    let (dh, ks, kv) = (lerp(a.1, b.1), lerp(a.2, b.2), lerp(a.3, b.3));
    let (r, g, bb) = from_hsv(h + dh * w, (s * (1.0 + (ks - 1.0) * w)).clamp(0.0, 1.0), (v * (1.0 + (kv - 1.0) * w)).clamp(0.0, 1.0));
    (r, g, bb)
}

/// The full transform of one (premultiplied) vertex colour.
pub fn map(c: Color32, m: Mode) -> Color32 {
    if m.identity() || c.a() == 0 {
        return c;
    }
    let a = c.a() as f32 / 255.0;
    // Un-premultiply.
    let un = |x: u8| (x as f32 / 255.0 / a).min(1.0);
    let mut rgb = (un(c.r()), un(c.g()), un(c.b()));
    let mut alpha = a;
    if m.palette != Palette::Standard {
        let d = DELTAS.with(|cache| {
            let mut cache = cache.borrow_mut();
            if cache.0 != Some(m.palette) {
                *cache = (Some(m.palette), deltas(m.palette));
            }
            cache.1.clone()
        });
        rgb = cvd(rgb, &d);
    }
    if m.high_contrast {
        let (h, s, v) = to_hsv(rgb.0, rgb.1, rgb.2);
        if s * v < 0.12 && (0.3..0.9).contains(&v) {
            // Secondary / tertiary text towards white.
            let v2 = v + (1.0 - v) * 0.4;
            rgb = from_hsv(h, s, v2);
        } else if v < 0.22 && alpha < 1.0 && alpha > 0.3 {
            // Translucent dark panels: nearly opaque.
            alpha = (alpha * 1.25 + 0.1).min(0.97);
        }
    }
    let p = |x: f32| (x * alpha * 255.0).round().clamp(0.0, 255.0) as u8;
    Color32::from_rgba_premultiplied(p(rgb.0), p(rgb.1), p(rgb.2), (alpha * 255.0).round() as u8)
}

thread_local! {
    static DELTAS: std::cell::RefCell<(Option<Palette>, Vec<(f32, f32, f32, f32)>)> = const { std::cell::RefCell::new((None, Vec::new())) };
}

/// Apply the mode to tessellated interface meshes (a small cache: a frame has
/// few distinct colours).
pub fn apply(prims: &mut [egui::ClippedPrimitive], m: Mode) {
    if m.identity() {
        return;
    }
    const N: usize = 4096;
    let mut cache: Vec<(u32, Color32)> = vec![(0, Color32::TRANSPARENT); N];
    for p in prims {
        if let egui::epaint::Primitive::Mesh(mesh) = &mut p.primitive {
            for v in &mut mesh.vertices {
                let key = u32::from_le_bytes(v.color.to_array());
                if key == 0 {
                    continue;
                }
                let slot = (key.wrapping_mul(0x9E37_79B1) >> 20) as usize % N;
                if cache[slot].0 == key {
                    v.color = cache[slot].1;
                } else {
                    let out = map(v.color, m);
                    cache[slot] = (key, out);
                    v.color = out;
                }
            }
        }
    }
}

/// The same transform for a colour the app hands to the GPU itself (clear colours).
pub fn map_color(c: Color32, m: Mode) -> Color32 {
    map(c, m)
}

// ---------------------------------------------------------------- colour science (tests, docs)

/// sRGB channel → linear.
pub fn lin(c: u8) -> f32 {
    let c = c as f32 / 255.0;
    if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}

/// WCAG relative luminance.
pub fn luminance(c: Color32) -> f32 {
    0.2126 * lin(c.r()) + 0.7152 * lin(c.g()) + 0.0722 * lin(c.b())
}

/// WCAG contrast ratio of two opaque colours (1 .. 21).
pub fn contrast(a: Color32, b: Color32) -> f32 {
    let (x, y) = (luminance(a), luminance(b));
    (x.max(y) + 0.05) / (x.min(y) + 0.05)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Machado, Oliveira & Fernandes (2009), severity 1.0, linear RGB.
    fn simulate(c: Color32, p: Palette) -> Color32 {
        let m: [[f32; 3]; 3] = match p {
            Palette::Protanopia => [[0.152286, 1.052583, -0.204868], [0.114503, 0.786281, 0.099216], [-0.003882, -0.048116, 1.051998]],
            Palette::Deuteranopia => [[0.367322, 0.860646, -0.227968], [0.280085, 0.672501, 0.047413], [-0.011820, 0.042940, 0.968881]],
            Palette::Tritanopia => [[1.255528, -0.076749, -0.178779], [-0.078411, 0.930809, 0.147602], [0.004733, 0.691367, 0.303900]],
            Palette::Standard => return c,
        };
        let l = [lin(c.r()), lin(c.g()), lin(c.b())];
        let enc = |x: f32| {
            let x = x.clamp(0.0, 1.0);
            ((if x <= 0.003_130_8 { 12.92 * x } else { 1.055 * x.powf(1.0 / 2.4) - 0.055 }) * 255.0).round() as u8
        };
        let o: Vec<u8> = (0..3).map(|i| enc(m[i][0] * l[0] + m[i][1] * l[1] + m[i][2] * l[2])).collect();
        Color32::from_rgb(o[0], o[1], o[2])
    }

    fn lab(c: Color32) -> [f32; 3] {
        let (r, g, b) = (lin(c.r()), lin(c.g()), lin(c.b()));
        let x = (0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.9505;
        let y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        let z = (0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.089;
        let f = |t: f32| if t > 0.008856 { t.cbrt() } else { 7.787 * t + 16.0 / 116.0 };
        [116.0 * f(y) - 16.0, 500.0 * (f(x) - f(y)), 200.0 * (f(y) - f(z))]
    }

    /// CIE76 colour difference as seen with deficiency `p`.
    fn seen_diff(a: Color32, b: Color32, p: Palette) -> f32 {
        let (x, y) = (lab(simulate(a, p)), lab(simulate(b, p)));
        ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2)).sqrt()
    }

    fn mode(p: Palette) -> Mode {
        Mode { palette: p, high_contrast: false }
    }

    #[test]
    fn anchors_land_on_their_targets_and_greys_stay() {
        for p in Palette::ALL {
            for (s, t) in anchors(p) {
                let m = map(s, mode(p));
                let d = (m.r() as i32 - t.r() as i32).abs().max((m.g() as i32 - t.g() as i32).abs()).max((m.b() as i32 - t.b() as i32).abs());
                assert!(d <= 2, "{p:?}: {s:?} → {m:?}, want {t:?}");
            }
            for g in [pal::TEXT, pal::TEXT2, pal::TEXT3, pal::PANEL_SOLID, pal::RAISED, pal::WHITE, Color32::BLACK] {
                assert_eq!(map(g, mode(p)), g, "{p:?} changed grey {g:?}");
            }
        }
        // Standard: identity, even for odd colours.
        let c = Color32::from_rgba_premultiplied(40, 90, 20, 128);
        assert_eq!(map(c, Mode::default()), c);
    }

    #[test]
    fn premultiplied_alpha_is_kept() {
        let m = mode(Palette::Deuteranopia);
        let faded = super::super::alpha(pal::GREEN, 0.5);
        let out = map(faded, m);
        assert_eq!(out.a(), faded.a());
        let full = map(pal::GREEN, m);
        // Same hue at half the strength.
        assert!((out.b() as i32 * 2 - full.b() as i32).abs() <= 3, "{out:?} vs {full:?}");
    }

    /// WCAG AA (4.5 : 1) for text: every status colour, and the text greys,
    /// on the solid panel and on raised rows, in every palette (and high contrast).
    #[test]
    fn text_contrast_wcag_aa() {
        let semantic = [pal::GREEN, pal::RED, pal::AMBER, pal::CYAN, pal::ACCENT, pal::MAGENTA, pal::TEXT, pal::TEXT2, pal::TEXT3];
        for p in Palette::ALL {
            for hc in [false, true] {
                let m = Mode { palette: p, high_contrast: hc };
                for bg in [pal::PANEL_SOLID, pal::RAISED, pal::BG] {
                    let bg = map(bg, m);
                    for c in semantic {
                        let fg = map(c, m);
                        let r = contrast(fg, bg);
                        assert!(r >= 4.5, "{p:?} hc={hc}: {fg:?} on {bg:?} is {r:.2}:1");
                    }
                }
            }
        }
        // High contrast really raises secondary text.
        let hc = Mode { palette: Palette::Standard, high_contrast: true };
        assert!(contrast(map(pal::TEXT3, hc), pal::PANEL_SOLID) > contrast(pal::TEXT3, pal::PANEL_SOLID) + 1.5);
        assert!(map(pal::PANEL, hc).a() > pal::PANEL.a());
    }

    /// Green / red / amber (good / warning / caution) stay clearly apart for
    /// the deficiency each palette is for — and better than the standard palette.
    #[test]
    fn status_colours_distinguishable() {
        let pairs = [(pal::GREEN, pal::RED), (pal::GREEN, pal::AMBER), (pal::RED, pal::AMBER)];
        for p in [Palette::Deuteranopia, Palette::Protanopia, Palette::Tritanopia] {
            let worst = |m: Mode| pairs.iter().map(|(a, b)| seen_diff(map(*a, m), map(*b, m), p)).fold(f32::MAX, f32::min);
            let before = worst(Mode::default());
            let after = worst(mode(p));
            assert!(after >= 45.0, "{p:?}: worst status pair ΔE {after:.1}");
            assert!(after >= before - 0.5, "{p:?}: {after:.1} < standard {before:.1}");
            // Selected (cyan) no longer looks like engaged (green).
            let gc = seen_diff(map(pal::GREEN, mode(p)), map(pal::CYAN, mode(p)), p);
            assert!(gc >= 25.0, "{p:?}: green vs cyan ΔE {gc:.1}");
        }
        // The red-green case the palettes exist for: the standard green and red are close.
        assert!(seen_diff(pal::GREEN, pal::RED, Palette::Deuteranopia) < 30.0);
    }

    #[test]
    fn apply_uses_the_cache_consistently() {
        let m = mode(Palette::Protanopia);
        let mut mesh = egui::epaint::Mesh::default();
        for c in [pal::GREEN, pal::RED, pal::GREEN, pal::TEXT, Color32::TRANSPARENT] {
            mesh.colored_vertex(egui::Pos2::ZERO, c);
        }
        let mut prims = vec![egui::ClippedPrimitive { clip_rect: egui::Rect::EVERYTHING, primitive: egui::epaint::Primitive::Mesh(mesh) }];
        apply(&mut prims, m);
        let egui::epaint::Primitive::Mesh(mesh) = &prims[0].primitive else { unreachable!() };
        assert_eq!(mesh.vertices[0].color, map(pal::GREEN, m));
        assert_eq!(mesh.vertices[2].color, mesh.vertices[0].color);
        assert_eq!(mesh.vertices[1].color, map(pal::RED, m));
        assert_eq!(mesh.vertices[3].color, pal::TEXT);
        assert_eq!(mesh.vertices[4].color, Color32::TRANSPARENT);
    }
}
