//! Scrolling for the menus, to the design system.
//!
//! * [`area`]: a vertical scroll area with a thin floating bar (always faintly
//!   visible when there is more to see, wider on hover, accent while dragged)
//!   and soft edge fades that say the list continues. Wheel, trackpad,
//!   dragging the content and the bar all scroll it.
//! * [`page`]: the same for a page laid out with the painter at absolute
//!   positions (the menus' style): it reports its height and scrolls when it
//!   does not fit.
//! * [`nav`]: keyboard arrows and game-controller d-pad / stick / A / B /
//!   bumpers as one-shot moves (with auto-repeat), for lists and grids.

use super::{alpha, gradient_rect, pal};
use crate::input::{DevKind, InputSystem, Std};
use egui::scroll_area::ScrollAreaOutput;
use egui::style::ScrollStyle;
use egui::{Color32, Id, Pos2, Rect, Sense, Ui, UiBuilder, Vec2};

/// Width kept free at the right of scrolling content for the floating bar.
pub const GUTTER: f32 = 14.0;
/// Height of the edge fades.
const FADE: f32 = 44.0;

/// Design-system scroll bars for `ui` (the scroll area paints its bar with the
/// style of the `Ui` it is shown in; the content gets the original style back).
fn style_bars(ui: &mut Ui) {
    let s = ui.style_mut();
    s.spacing.scroll = ScrollStyle {
        floating: true,
        bar_width: 8.0,
        floating_width: 4.0,
        floating_allocated_width: 0.0,
        handle_min_length: 40.0,
        bar_inner_margin: 4.0,
        bar_outer_margin: 3.0,
        foreground_color: false,
        dormant_background_opacity: 0.0,
        active_background_opacity: 0.6,
        interact_background_opacity: 1.0,
        dormant_handle_opacity: 0.7,
        active_handle_opacity: 0.9,
        interact_handle_opacity: 1.0,
        ..ScrollStyle::floating()
    };
    let w = &mut s.visuals.widgets;
    w.inactive.bg_fill = pal::TEXT3;
    w.hovered.bg_fill = pal::TEXT2;
    w.active.bg_fill = pal::ACCENT;
    for v in [&mut w.inactive, &mut w.hovered, &mut w.active] {
        v.corner_radius = 4.into();
    }
    s.visuals.extreme_bg_color = Color32::from_white_alpha(10);
}

/// A vertical scroll area filling `rect`. `fade` is the colour behind it,
/// used for the edge fades (transparent: none).
pub fn area<R>(ui: &mut Ui, rect: Rect, id: impl std::hash::Hash + std::fmt::Debug, fade: Color32, add: impl FnOnce(&mut Ui) -> R) -> ScrollAreaOutput<R> {
    let out = ui
        .scope_builder(UiBuilder::new().max_rect(rect), |ui| {
            let content_style = ui.style().clone();
            style_bars(ui);
            egui::ScrollArea::vertical().id_salt(id).auto_shrink([false, false]).show(ui, |ui| {
                ui.set_style(content_style);
                add(ui)
            })
        })
        .inner;
    if fade.a() > 0 {
        fades(ui.painter(), &out, fade);
    }
    out
}

/// A design-system vertical scroll area in a layout (panels built with
/// egui's layout rather than the painter), at most `max_h` tall.
pub fn vertical<R>(ui: &mut Ui, id: impl std::hash::Hash + std::fmt::Debug, max_h: f32, add: impl FnOnce(&mut Ui) -> R) -> ScrollAreaOutput<R> {
    let content_style = ui.style().clone();
    ui.scope(|ui| {
        style_bars(ui);
        egui::ScrollArea::vertical().id_salt(id).auto_shrink([false, false]).max_height(max_h).show(ui, |ui| {
            ui.set_style(content_style);
            add(ui)
        })
    })
    .inner
}

/// Fades at the edges that have more content beyond them.
fn fades<R>(p: &egui::Painter, out: &ScrollAreaOutput<R>, col: Color32) {
    let r = out.inner_rect;
    let off = out.state.offset().y;
    let rest = out.content_size.y - r.height() - off;
    let w = Rect::from_min_max(r.min, Pos2::new(r.max.x - GUTTER, r.max.y));
    let top = (off / FADE).clamp(0.0, 1.0);
    if top > 0.01 {
        gradient_rect(p, Rect::from_min_max(w.min, Pos2::new(w.max.x, w.min.y + FADE)), alpha(col, top), Color32::TRANSPARENT);
    }
    let bot = (rest / FADE).clamp(0.0, 1.0);
    if bot > 0.01 {
        gradient_rect(p, Rect::from_min_max(Pos2::new(w.min.x, w.max.y - FADE), w.max), Color32::TRANSPARENT, alpha(col, bot));
    }
}

/// A painter-laid-out page in `rect` that scrolls when it is taller.
/// `draw(ui, top_left, width)` paints with `ui`'s painter and interacts with
/// `ui` (both clipped to the viewport) and returns the content height.
pub fn page(ui: &mut Ui, rect: Rect, id: &str, fade: Color32, draw: impl FnOnce(&mut Ui, Pos2, f32) -> f32) -> ScrollAreaOutput<()> {
    let hid = Id::new(("scroll_page_h", id));
    let h = ui.data(|d| d.get_temp::<f32>(hid)).unwrap_or(rect.height());
    area(ui, rect, id, fade, |ui| {
        let w = ui.available_width();
        let (r, _) = ui.allocate_exact_size(Vec2::new(w, h.max(1.0)), Sense::hover());
        let used = draw(ui, r.min, w).ceil();
        if (used - h).abs() > 0.5 {
            ui.data_mut(|d| d.insert_temp(hid, used));
            ui.ctx().request_repaint();
        }
    })
}

// ---------------------------------------------------------------- navigation

/// One frame's navigation moves.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct Nav {
    /// −1 / +1: left / right.
    pub dx: i32,
    /// −1 / +1: up / down.
    pub dy: i32,
    /// Enter / A (Cross).
    pub ok: bool,
    /// Esc / B (Circle).
    pub back: bool,
    /// −1 / +1: previous / next tab (Shift+Tab / Tab, LB / RB).
    pub tab: i32,
    /// Home / End: −1 / +1.
    pub ends: i32,
}

const UP: u8 = 1;
const DOWN: u8 = 2;
const LEFT: u8 = 4;
const RIGHT: u8 = 8;
const OK: u8 = 16;
const BACK: u8 = 32;
const LB: u8 = 64;
const RB: u8 = 128;
/// Hold this long before a direction repeats, then repeat this often (s).
const REPEAT_DELAY: f64 = 0.42;
const REPEAT_EVERY: f64 = 0.11;

/// Controller state as button bits: d-pad (buttons or hat), left stick, A, B, bumpers.
fn pad_bits(input: &InputSystem) -> u8 {
    let mut bits = 0;
    for d in input.devices.iter().filter(|d| d.info.kind == DevKind::Gamepad) {
        let v = |s: Std| d.info.std(s).and_then(|c| d.value(c)).unwrap_or(0.0);
        let (hx, hy) = (v(Std::HatX), v(Std::HatY));
        let (sx, sy) = (v(Std::X), v(Std::Y));
        // (Up is positive on the hat and the stick: see input/defaults.rs.)
        if v(Std::DUp) > 0.5 || hy > 0.5 || sy > 0.6 {
            bits |= UP;
        }
        if v(Std::DDown) > 0.5 || hy < -0.5 || sy < -0.6 {
            bits |= DOWN;
        }
        if v(Std::DLeft) > 0.5 || hx < -0.5 || sx < -0.6 {
            bits |= LEFT;
        }
        if v(Std::DRight) > 0.5 || hx > 0.5 || sx > 0.6 {
            bits |= RIGHT;
        }
        for (s, b) in [(Std::South, OK), (Std::East, BACK), (Std::LB, LB), (Std::RB, RB)] {
            if v(s) > 0.5 {
                bits |= b;
            }
        }
    }
    bits
}

#[derive(Clone, Copy, Default)]
struct PadMem {
    bits: u8,
    frame: u64,
    held_since: f64,
    last_repeat: f64,
}

/// This frame's moves from the keyboard and game controllers. `frame` is the
/// UI frame counter: a controller already held when a screen opens does not
/// act until released.
pub fn nav(ui: &Ui, input: &InputSystem, frame: u64) -> Nav {
    let mut n = ui.input(|i| {
        let k = |key| i.key_pressed(key);
        use egui::Key;
        Nav {
            dx: k(Key::ArrowRight) as i32 - k(Key::ArrowLeft) as i32,
            dy: k(Key::ArrowDown) as i32 - k(Key::ArrowUp) as i32,
            ok: k(Key::Enter) && !i.modifiers.any(),
            back: k(Key::Escape),
            tab: if k(Key::Tab) { if i.modifiers.shift { -1 } else { 1 } } else { 0 },
            ends: k(Key::End) as i32 - k(Key::Home) as i32,
        }
    });
    let now = ui.input(|i| i.time);
    let bits = pad_bits(input);
    let id = Id::new("scroll_pad_nav");
    let mut m: PadMem = ui.data(|d| d.get_temp(id)).unwrap_or_default();
    if m.frame + 1 != frame {
        // First frame on this screen (or after a gap): only track.
        m = PadMem { bits, frame, held_since: now, last_repeat: now };
        ui.data_mut(|d| d.insert_temp(id, m));
        return n;
    }
    let pressed = bits & !m.bits;
    let dirs = bits & (UP | DOWN | LEFT | RIGHT);
    if dirs != m.bits & (UP | DOWN | LEFT | RIGHT) {
        m.held_since = now;
        m.last_repeat = now;
    }
    let mut fire = pressed;
    if dirs != 0 && now - m.held_since > REPEAT_DELAY && now - m.last_repeat > REPEAT_EVERY {
        fire |= dirs;
        m.last_repeat = now;
    }
    let has = |b: u8| fire & b != 0;
    n.dx += has(RIGHT) as i32 - has(LEFT) as i32;
    n.dy += has(DOWN) as i32 - has(UP) as i32;
    n.ok |= has(OK);
    n.back |= has(BACK);
    n.tab += has(RB) as i32 - has(LB) as i32;
    m.bits = bits;
    m.frame = frame;
    ui.data_mut(|d| d.insert_temp(id, m));
    n
}
