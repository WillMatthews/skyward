//! Settings › Controls: axis bindings with response curves and calibration,
//! button / key bindings with listen-to-bind and conflict warnings, connected
//! devices with live inputs, and the mouse (yoke, head-look).

use super::menus::{Btn, button, chip_btn, segmented};
use super::{UiState, alpha, f_mono, f_semi, f_ui, pal, sp};
use crate::input::{AxisAction, AxisCfg, BUTTONS, Captured, Config, Half, Listen, code_label};
use crate::sim::Key;
use egui::{Align2, Color32, Pos2, Rect, RichText, Sense, Stroke, Ui, UiBuilder, Vec2};

pub const PAGES: [&str; 4] = ["Axes", "Buttons & keys", "Devices", "Mouse & view"];

/// Apply a binding captured while listening (from the keyboard or a device).
pub(super) fn apply_captured(st: &mut UiState) {
    let Some(c) = st.input.captured.take() else { return };
    let cfg = &mut st.settings.controls;
    let (what, action) = match &c {
        Captured::Key(k, code) => {
            cfg.bind_key(*k, *code);
            (crate::input::keys::label(*code), crate::input::key_label(*k).to_string())
        }
        Captured::Button { action, dev, code, half } => {
            cfg.bind_button(dev, *action, *code, *half);
            (code_label(st.input.device(dev).map(|d| &d.info), *code, *half), crate::input::key_label(*action).to_string())
        }
        Captured::Axis { action, dev, code, half } => {
            cfg.bind_axis(dev, *action, *code, *half);
            st.controls_axis = Some((*action, cfg.profile(dev).map(|p| p.axes.len().saturating_sub(1)).unwrap_or(0)));
            (code_label(st.input.device(dev).map(|d| &d.info), *code, *half), action.label().to_string())
        }
    };
    st.settings_dirty = true;
    let clash = cfg.conflicts().into_iter().find(|(d, ks)| *d == what && ks.len() > 1);
    match clash {
        Some((_, ks)) => {
            let others: Vec<&str> = ks.iter().map(|k| crate::input::key_label(*k)).filter(|l| *l != action).collect();
            st.toasts.push(super::toast::Kind::Warn, format!("{what} → {action}"), format!("Also bound to {}", others.join(", ")), 4.0);
        }
        None => st.toasts.push(super::toast::Kind::Good, format!("{what} → {action}"), "", 2.0),
    }
}

/// The Controls page. `foot`: a footer row for its own "Reset all controls"
/// button (None: the settings shell's Reset category does it).
pub fn page(ui: &mut Ui, st: &mut UiState, body: Rect, foot: Option<Rect>) {
    apply_captured(st);
    let p = ui.painter().clone();
    let seg = Rect::from_min_size(body.min, Vec2::new(body.width().min(560.0), 34.0));
    if let Some(i) = segmented(ui, seg, "ctlpages", &PAGES, st.controls_page) {
        st.controls_page = i;
        st.input.listen = None;
        st.input.calib = None;
    }
    let n = st.settings.controls.conflicts().len();
    if n > 0 {
        p.text(Pos2::new(body.max.x, seg.center().y), Align2::RIGHT_CENTER, format!("⚠  {n} conflict{}", if n > 1 { "s" } else { "" }), f_semi(13.0), pal::AMBER);
    }
    let mut area = Rect::from_min_max(Pos2::new(body.min.x, seg.max.y + 14.0), body.max);
    // Listening banner.
    if let Some(l) = st.input.listen {
        let r = Rect::from_min_size(area.min, Vec2::new(area.width(), 40.0));
        let pulse = 0.55 + 0.45 * (st.time * 5.0).sin().abs();
        p.rect_filled(r, sp::R_CTRL, alpha(pal::ACCENT_DEEP, 0.55));
        p.rect_stroke(r, sp::R_CTRL, Stroke::new(1.5, alpha(pal::ACCENT, pulse)), egui::StrokeKind::Inside);
        let msg = match l {
            Listen::Key(k) => format!("Press a key for “{}”", crate::input::key_label(k)),
            Listen::Button(k) => format!("Press a controller button, or push a hat / stick, for “{}”", crate::input::key_label(k)),
            Listen::Axis(a) => format!("Move the axis or lever for “{}” through its range", a.label()),
        };
        p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, msg, f_semi(14.0), pal::WHITE);
        p.text(Pos2::new(r.max.x - 14.0, r.center().y), Align2::RIGHT_CENTER, "Esc to cancel", f_ui(12.5), pal::TEXT2);
        area.min.y = r.max.y + 10.0;
    }
    match st.controls_page {
        0 => axes_page(ui, st, area),
        1 => buttons_page(ui, st, area),
        2 => devices_page(ui, st, area),
        _ => mouse_page(ui, st, area),
    }
    // Footer: reset.
    let Some(foot) = foot else { return };
    let r = Rect::from_min_size(foot.min, Vec2::new(190.0, foot.height()));
    if button(ui, r, "ctlreset", "Reset all controls", Btn::Secondary, true).clicked() {
        st.settings.controls = Config::default();
        st.input.ensure_profiles(&mut st.settings.controls);
        st.input.listen = None;
        st.controls_axis = None;
        st.settings_dirty = true;
        st.toasts.push(super::toast::Kind::Info, "Controls reset to defaults", "Keyboard, controllers and mouse", 2.5);
    }
}

/// Esc while listening cancels instead of leaving the screen.
pub fn swallow_esc(st: &mut UiState) -> bool {
    if st.input.listen.is_some() || st.input.calib.is_some() {
        st.input.listen = None;
        st.input.calib = None;
        return true;
    }
    false
}

/// A design-system slider filling `r` exactly.
fn put_slider(ui: &mut Ui, r: Rect, v: &mut f32, range: std::ops::RangeInclusive<f32>) {
    // The unfilled track, visible on raised rows too.
    let t = Rect::from_center_size(r.center(), Vec2::new(r.width() - 8.0, 6.0));
    ui.painter().rect_filled(t, 3, Color32::from_rgb(9, 11, 15));
    // A child Ui: placing it must not move the parent's cursor (rows are pre-allocated).
    let mut child = ui.new_child(UiBuilder::new().max_rect(r));
    child.spacing_mut().slider_width = r.width();
    child.put(r, egui::Slider::new(v, range).show_value(false));
}

/// `text` shortened with an ellipsis to fit `max_w` pixels.
fn fit(p: &egui::Painter, text: &str, font: egui::FontId, max_w: f32) -> String {
    let w = |t: &str| p.layout_no_wrap(t.to_string(), font.clone(), pal::TEXT).size().x;
    if w(text) <= max_w {
        return text.to_string();
    }
    let mut chars: Vec<char> = text.chars().collect();
    while !chars.is_empty() && w(&format!("{}…", chars.iter().collect::<String>())) > max_w {
        chars.pop();
    }
    format!("{}…", chars.iter().collect::<String>().trim_end())
}

fn scroll(ui: &mut Ui, area: Rect, id: &str, f: impl FnOnce(&mut Ui)) {
    super::scroll::area(ui, area, id, pal::PANEL_SOLID, |ui| {
        ui.spacing_mut().item_spacing = Vec2::new(0.0, 6.0);
        f(ui)
    });
}

fn group_label(ui: &mut Ui, t: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(t.to_uppercase()).font(super::f_inst_b(11.0)).color(pal::ACCENT));
}

/// A small binding chip; returns (clicked, rect). `hot`: listening / active look.
fn chip(ui: &mut Ui, at: Pos2, id: impl std::hash::Hash + std::fmt::Debug, text: &str, kind: u8) -> (bool, Rect) {
    let g = ui.painter().layout_no_wrap(text.to_string(), f_semi(12.5), pal::TEXT);
    let r = Rect::from_min_size(at, Vec2::new(g.size().x + 20.0, 26.0));
    let resp = ui.interact(r, ui.id().with(id), Sense::click());
    let p = ui.painter();
    let hov = resp.hovered();
    let (fill, stroke, fg) = match kind {
        // 0 bound, 1 add (+), 2 listening, 3 conflict
        1 => (if hov { pal::RAISED_HI } else { Color32::TRANSPARENT }, pal::HAIRLINE_HI, if hov { pal::TEXT } else { pal::TEXT3 }),
        2 => (pal::ACCENT_DEEP, pal::ACCENT, pal::WHITE),
        3 => (if hov { pal::RAISED_HI } else { pal::RAISED }, pal::AMBER, pal::TEXT),
        _ => (if hov { pal::RAISED_HI } else { pal::RAISED }, pal::HAIRLINE_HI, pal::TEXT),
    };
    p.rect_filled(r, sp::R_CTRL, fill);
    p.rect_stroke(r, sp::R_CTRL, Stroke::new(1.0, stroke), egui::StrokeKind::Inside);
    let shown = if hov && kind == 0 || hov && kind == 3 { format!("{text}  ×") } else { text.to_string() };
    p.text(r.center(), Align2::CENTER_CENTER, shown, f_semi(12.5), fg);
    if hov {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    (resp.clicked(), r)
}

// ---------------------------------------------------------------- axes

/// One axis binding: (device key, device name, index in the profile's axes).
type BindRef = (String, String, usize);

fn axes_page(ui: &mut Ui, st: &mut UiState, area: Rect) {
    // (device key, device name, binding index) for every binding of each action.
    let mut rows: Vec<(AxisAction, Vec<BindRef>)> = AxisAction::ALL.iter().map(|a| (*a, Vec::new())).collect();
    for (key, p) in &st.settings.controls.devices {
        for (i, b) in p.axes.iter().enumerate() {
            rows[b.action.idx()].1.push((key.clone(), p.name.clone(), i));
        }
    }
    let analog = st.input.analog;
    let any_device = !st.input.devices.is_empty();
    scroll(ui, area, "axes", |ui| {
        if !any_device {
            ui.label(
                RichText::new("No controller connected — the keyboard flies (smoothed). Plug in a gamepad, joystick, throttle or pedals and they appear here.")
                    .font(f_ui(13.0))
                    .color(pal::TEXT3),
            );
            ui.add_space(4.0);
        }
        let groups = [("Flight", 0..3), ("Engines & levers", 3..12), ("View", 12..14)];
        for (g, range) in groups {
            group_label(ui, g);
            for (a, binds) in &rows[range] {
                axis_row(ui, st, *a, binds, analog.get(*a));
            }
        }
    });
}

fn axis_row(ui: &mut Ui, st: &mut UiState, a: AxisAction, binds: &[BindRef], value: Option<f32>) {
    let open = st.controls_axis.is_some_and(|(x, _)| x == a);
    let w = ui.available_width() - 12.0;
    let open_h = 52.0 + 150.0 + if binds.len() > 1 { 36.0 } else { 0.0 };
    let (r, resp) = ui.allocate_exact_size(Vec2::new(w, if open { open_h } else { 52.0 }), Sense::click());
    let p = ui.painter().clone();
    let head = Rect::from_min_size(r.min, Vec2::new(w, 52.0));
    p.rect_filled(
        r,
        sp::R_CTRL,
        if open {
            pal::RAISED
        } else if resp.hovered() {
            alpha(pal::RAISED, 0.8)
        } else {
            alpha(pal::RAISED, 0.45)
        },
    );
    p.text(Pos2::new(r.min.x + 14.0, head.center().y - 8.0), Align2::LEFT_CENTER, a.label(), f_semi(14.0), pal::TEXT);
    let src = if binds.is_empty() {
        if a.centred() && !matches!(a, AxisAction::LookX | AxisAction::LookY) { "Keyboard only".to_string() } else { "Not bound".to_string() }
    } else {
        binds
            .iter()
            .map(|(dev, name, i)| {
                let b = &st.settings.controls.profile(dev).unwrap().axes[*i];
                format!("{} · {}", short_name(name), code_label(st.input.device(dev).map(|d| &d.info), b.code, b.half))
            })
            .collect::<Vec<_>>()
            .join("   +   ")
    };
    let src = fit(&p, &src, f_ui(12.0), w - 420.0);
    p.text(Pos2::new(r.min.x + 14.0, head.center().y + 10.0), Align2::LEFT_CENTER, src, f_ui(12.0), pal::TEXT3);
    // Live value.
    let bar = Rect::from_min_size(Pos2::new(head.max.x - 380.0, head.center().y - 4.0), Vec2::new(170.0, 8.0));
    value_bar(&p, bar, value, a.unipolar());
    p.text(
        Pos2::new(bar.max.x + 10.0, bar.center().y),
        Align2::LEFT_CENTER,
        value.map(|v| format!("{:+.2}", v)).unwrap_or_else(|| "—".into()),
        f_mono(12.0),
        if value.is_some() { pal::TEXT2 } else { pal::TEXT3 },
    );
    // Bind / clear.
    let listening = st.input.listen == Some(Listen::Axis(a));
    let (c, _) =
        chip(ui, Pos2::new(head.max.x - 150.0, head.center().y - 13.0), ("bind", a.idx()), if listening { "Listening…" } else { "Bind axis" }, if listening { 2 } else { 1 });
    if c {
        if listening {
            st.input.listen = None;
        } else {
            st.input.start_listening(Listen::Axis(a));
        }
    }
    if !binds.is_empty() {
        let (c, _) = chip(ui, Pos2::new(head.max.x - 58.0, head.center().y - 13.0), ("clear", a.idx()), "Clear", 1);
        if c {
            for (dev, _, _) in binds {
                if let Some(p) = st.settings.controls.profile_mut(dev) {
                    p.axes.retain(|b| b.action != a);
                }
            }
            st.controls_axis = None;
            st.settings_dirty = true;
            return;
        }
    }
    if resp.clicked() && !binds.is_empty() {
        st.controls_axis = if open { None } else { Some((a, 0)) };
    }
    if open {
        let sel = st.controls_axis.map(|x| x.1).unwrap_or(0).min(binds.len().saturating_sub(1));
        if let Some((dev, _, bi)) = binds.get(sel).cloned() {
            axis_details(ui, st, a, Rect::from_min_max(Pos2::new(r.min.x + 14.0, head.max.y), r.max - Vec2::new(14.0, 10.0)), &dev, bi, binds, sel);
        }
    }
}

fn short_name(n: &str) -> String {
    let n = n.replace("Thrustmaster ", "").replace("Saitek ", "").replace("Logitech ", "").replace(" Flight Control System", "");
    if n.chars().count() > 26 { format!("{}…", n.chars().take(25).collect::<String>()) } else { n }
}

fn value_bar(p: &egui::Painter, r: Rect, v: Option<f32>, unipolar: bool) {
    p.rect_filled(r, 4, Color32::from_rgb(9, 11, 15));
    let Some(v) = v else { return };
    let fill = if unipolar {
        Rect::from_min_size(r.min, Vec2::new(r.width() * v.clamp(0.0, 1.0), r.height()))
    } else {
        let c = r.center().x;
        let x = c + r.width() * 0.5 * v.clamp(-1.0, 1.0);
        Rect::from_min_max(Pos2::new(c.min(x), r.min.y), Pos2::new(c.max(x), r.max.y))
    };
    p.rect_filled(fill, 4, pal::ACCENT);
    if !unipolar {
        p.line_segment([Pos2::new(r.center().x, r.min.y - 3.0), Pos2::new(r.center().x, r.max.y + 3.0)], Stroke::new(1.0, pal::TEXT3));
    }
}

#[allow(clippy::too_many_arguments)]
fn axis_details(ui: &mut Ui, st: &mut UiState, a: AxisAction, r: Rect, dev: &str, bi: usize, binds: &[BindRef], sel: usize) {
    let p = ui.painter().clone();
    p.line_segment([Pos2::new(r.min.x, r.min.y), Pos2::new(r.max.x, r.min.y)], Stroke::new(1.0, pal::HAIRLINE_HI));
    let mut y = r.min.y + 12.0;
    if binds.len() > 1 {
        let labels: Vec<String> = binds.iter().map(|b| short_name(&b.1)).collect();
        let refs: Vec<&str> = labels.iter().map(|s| s.as_str()).collect();
        if let Some(i) = segmented(ui, Rect::from_min_size(Pos2::new(r.min.x, y), Vec2::new(360.0, 28.0)), &format!("axsel{}", a.idx()), &refs, sel) {
            st.controls_axis = Some((a, i));
        }
        y += 36.0;
    }
    let raw = st.input.device(dev).and_then(|d| st.settings.controls.profile(dev).and_then(|p| p.axes.get(bi)).and_then(|b| d.value(b.code)));
    let Some(b) = st.settings.controls.profile(dev).and_then(|p| p.axes.get(bi)).cloned() else { return };
    let mut cfg = b.cfg;
    // Sliders (left) and the response curve (right).
    let lw = r.width() - 200.0;
    let slider = |ui: &mut Ui, y: f32, label: &str, v: &mut f32, lo: f32, hi: f32, fmt: &dyn Fn(f32) -> String| {
        ui.painter().text(Pos2::new(r.min.x, y + 12.0), Align2::LEFT_CENTER, label, f_ui(13.0), pal::TEXT2);
        let sr = Rect::from_min_size(Pos2::new(r.min.x + 110.0, y), Vec2::new(lw - 270.0, 24.0));
        put_slider(ui, sr, v, lo..=hi);
        ui.painter().text(Pos2::new(sr.max.x + 12.0, sr.center().y), Align2::LEFT_CENTER, fmt(*v), f_mono(12.0), pal::TEXT2);
    };
    slider(ui, y, "Deadzone", &mut cfg.deadzone, 0.0, 0.3, &|v| format!("{:.0} %", v * 100.0));
    slider(ui, y + 32.0, "Curve", &mut cfg.expo, 0.0, 1.0, &|v| if v < 0.01 { "linear".into() } else { format!("{:.0} %", v * 100.0) });
    slider(ui, y + 64.0, "Sensitivity", &mut cfg.sens, 0.3, 2.0, &|v| format!("{:.2}×", v));
    // Invert + calibrate.
    let cx = r.min.x + lw - 80.0;
    if chip_btn(ui, Rect::from_min_size(Pos2::new(cx, y), Vec2::new(100.0, 26.0)), ("inv", a.idx()), "Invert", cfg.invert) {
        cfg.invert = !cfg.invert;
    }
    let calibrating = st.input.calib.is_some_and(|c| st.input.device(dev).is_some_and(|d| d.info.id == c.dev) && c.code == b.code);
    let cal_label = if calibrating {
        "Done"
    } else if cfg.cal.is_some() {
        "Recalibrate"
    } else {
        "Calibrate"
    };
    if button(
        ui,
        Rect::from_min_size(Pos2::new(cx, y + 34.0), Vec2::new(100.0, 28.0)),
        ("cal", a.idx()),
        cal_label,
        if calibrating { Btn::Primary } else { Btn::Secondary },
        raw.is_some(),
    )
    .clicked()
    {
        if calibrating {
            if let Some(res) = st.input.calib.and_then(|c| c.result()) {
                cfg.cal = Some(res);
            }
            st.input.calib = None;
        } else {
            st.input.start_calibration(dev, b.code);
        }
    }
    if cfg.cal.is_some()
        && !calibrating
        && button(ui, Rect::from_min_size(Pos2::new(cx, y + 68.0), Vec2::new(100.0, 24.0)), ("uncal", a.idx()), "Clear calibration", Btn::Ghost, true).clicked()
    {
        cfg.cal = None;
    }
    let hint = if calibrating {
        let c = st.input.calib.unwrap();
        format!("Move it to both ends, leave it at rest, then Done  ·  seen {:+.2} … {:+.2}", c.lo, c.hi)
    } else {
        format!("{} · raw {}", b.half_label(), raw.map(|v| format!("{v:+.3}")).unwrap_or_else(|| "— (not moved yet)".into()))
    };
    p.text(Pos2::new(r.min.x, y + 108.0), Align2::LEFT_CENTER, hint, f_ui(12.0), if calibrating { pal::ACCENT } else { pal::TEXT3 });
    // Response curve.
    let plot = Rect::from_min_size(Pos2::new(r.max.x - 170.0, y - 2.0), Vec2::new(170.0, 120.0));
    curve_plot(&p, plot, &cfg, b.half, a.unipolar(), raw);
    if cfg != b.cfg
        && let Some(bb) = st.settings.controls.profile_mut(dev).and_then(|p| p.axes.get_mut(bi))
    {
        bb.cfg = cfg;
        st.settings_dirty = true;
    }
}

trait HalfLabel {
    fn half_label(&self) -> &'static str;
}
impl HalfLabel for crate::input::AxisBind {
    fn half_label(&self) -> &'static str {
        match self.half {
            Half::Full => "Whole axis",
            Half::Pos => "Upper half",
            Half::Neg => "Lower half",
        }
    }
}

fn curve_plot(p: &egui::Painter, r: Rect, cfg: &AxisCfg, half: Half, unipolar: bool, raw: Option<f32>) {
    p.rect_filled(r, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    let inner = r.shrink(8.0);
    let (lo, hi) = if unipolar { (0.0, 1.0) } else { (-1.0, 1.0) };
    let to = |x: f32, y: f32| Pos2::new(inner.min.x + (x + 1.0) * 0.5 * inner.width(), inner.max.y - (y - lo) / (hi - lo) * inner.height());
    // Grid: centre lines and the linear reference.
    p.line_segment([to(-1.0, if unipolar { 0.5 } else { 0.0 }), to(1.0, if unipolar { 0.5 } else { 0.0 })], Stroke::new(1.0, pal::HAIRLINE_HI));
    p.line_segment([to(0.0, lo), to(0.0, hi)], Stroke::new(1.0, pal::HAIRLINE_HI));
    let pts: Vec<Pos2> = (0..=64)
        .map(|i| {
            let x = -1.0 + 2.0 * i as f32 / 64.0;
            to(x, cfg.apply(x, half, unipolar))
        })
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(2.0, pal::ACCENT)));
    if let Some(x) = raw {
        let x = x.clamp(-1.0, 1.0);
        p.circle_filled(to(x, cfg.apply(x, half, unipolar)), 4.5, pal::WHITE);
    }
}

// ---------------------------------------------------------------- buttons

fn buttons_page(ui: &mut Ui, st: &mut UiState, area: Rect) {
    let conflicts = st.settings.controls.conflicts();
    let clash = |k: Key, what: &str| conflicts.iter().any(|(d, ks)| d == what && ks.contains(&k));
    let devices: Vec<(String, String)> = st.settings.controls.devices.iter().filter(|(_, p)| p.enabled).map(|(k, p)| (k.clone(), p.name.clone())).collect();
    let connected: Vec<String> = st.input.devices.iter().map(|d| d.info.key.clone()).collect();
    scroll(ui, area, "buttons", |ui| {
        let w = ui.available_width() - 12.0;
        let (hr, _) = ui.allocate_exact_size(Vec2::new(w, 18.0), Sense::hover());
        ui.painter().text(Pos2::new(hr.min.x + 230.0, hr.center().y), Align2::LEFT_CENTER, "KEYBOARD", super::f_inst_b(10.5), pal::TEXT3);
        ui.painter().text(Pos2::new(hr.min.x + 230.0 + (w - 230.0) * 0.5, hr.center().y), Align2::LEFT_CENTER, "CONTROLLER", super::f_inst_b(10.5), pal::TEXT3);
        let mut group = "";
        for &(k, _, label, g) in BUTTONS {
            if g != group {
                group = g;
                group_label(ui, g);
            }
            let (r, _) = ui.allocate_exact_size(Vec2::new(w, 34.0), Sense::hover());
            let p = ui.painter().clone();
            p.rect_filled(r, sp::R_CTRL, alpha(pal::RAISED, 0.45));
            p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, f_ui(13.5), pal::TEXT);
            let cy = r.center().y - 13.0;
            // Keyboard chips.
            let mut x = r.min.x + 226.0;
            let kx_max = r.min.x + 226.0 + (w - 230.0) * 0.5 - 8.0;
            let codes = st.settings.controls.keys_of(k).to_vec();
            for c in codes {
                let t = crate::input::keys::label(c);
                let (clicked, cr) = chip(ui, Pos2::new(x, cy), ("k", label, &t), &t, if clash(k, &t) { 3 } else { 0 });
                if clicked {
                    st.settings.controls.keys_of_mut(k).retain(|x| *x != c);
                    st.settings_dirty = true;
                }
                x = cr.max.x + 5.0;
            }
            let listening = st.input.listen == Some(Listen::Key(k));
            if x < kx_max {
                let (clicked, _) = chip(ui, Pos2::new(x, cy), ("k+", label), if listening { "Press a key…" } else { "+" }, if listening { 2 } else { 1 });
                if clicked {
                    if listening { st.input.listen = None } else { st.input.start_listening(Listen::Key(k)) }
                }
            }
            // Controller chips.
            let mut x = kx_max + 12.0;
            for (dev, name) in &devices {
                let Some(prof) = st.settings.controls.profile(dev).cloned() else { continue };
                let info = st.input.device(dev).map(|d| d.info.clone());
                for b in prof.buttons.iter().filter(|b| b.action == k).cloned() {
                    let what = code_label(info.as_ref(), b.code, b.half);
                    let t = if devices.len() > 1 { format!("{} · {}", short_name(name).split_whitespace().next().unwrap_or(""), what) } else { what.clone() };
                    let conflict = clash(k, &format!("{} · {}", prof.name, code_label(None, b.code, b.half)));
                    let (clicked, cr) = chip(ui, Pos2::new(x, cy), ("b", label, dev, b.code, b.half as u8), &t, if conflict { 3 } else { 0 });
                    if clicked {
                        if let Some(p) = st.settings.controls.profile_mut(dev) {
                            p.buttons.retain(|x| !(x.action == k && x.code == b.code && x.half == b.half));
                        }
                        st.settings_dirty = true;
                    }
                    x = cr.max.x + 5.0;
                }
            }
            let listening = st.input.listen == Some(Listen::Button(k));
            if !connected.is_empty() && x < r.max.x - 40.0 {
                let (clicked, _) = chip(ui, Pos2::new(x, cy), ("b+", label), if listening { "Press a button…" } else { "+" }, if listening { 2 } else { 1 });
                if clicked {
                    if listening { st.input.listen = None } else { st.input.start_listening(Listen::Button(k)) }
                }
            }
        }
        ui.add_space(8.0);
        ui.label(RichText::new("Click a binding to remove it, + to add one. Esc always opens the pause menu.").font(f_ui(12.0)).color(pal::TEXT3));
    });
}

// ---------------------------------------------------------------- devices

fn devices_page(ui: &mut Ui, st: &mut UiState, area: Rect) {
    let mut status = st.input.status.clone();
    if !st.input.ignored.is_empty() {
        status += &format!("  ·  ignored (not game controllers): {}", st.input.ignored.join(", "));
    }
    let devs = st.input.devices.clone();
    scroll(ui, area, "devices", |ui| {
        let w = ui.available_width() - 12.0;
        if devs.is_empty() {
            let (r, _) = ui.allocate_exact_size(Vec2::new(w, 120.0), Sense::hover());
            let p = ui.painter();
            p.rect_stroke(r, sp::R_PANEL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
            p.text(r.center() - Vec2::new(0.0, 14.0), Align2::CENTER_CENTER, "No controllers connected", f_semi(16.0), pal::TEXT);
            p.text(
                r.center() + Vec2::new(0.0, 14.0),
                Align2::CENTER_CENTER,
                "Gamepads, joysticks, throttles and rudder pedals are picked up when plugged in.",
                f_ui(13.0),
                pal::TEXT3,
            );
        }
        for d in &devs {
            let Some(prof) = st.settings.controls.profile(&d.info.key).cloned() else { continue };
            let axes: Vec<_> = d.values.iter().filter(|v| !v.2).collect();
            let btns: Vec<_> = d.values.iter().filter(|v| v.2).collect();
            let rows = axes.len().div_ceil(2);
            let h = 76.0 + rows as f32 * 24.0 + if btns.is_empty() { 0.0 } else { 34.0 };
            let (r, _) = ui.allocate_exact_size(Vec2::new(w, h), Sense::hover());
            let p = ui.painter().clone();
            p.rect_filled(r, sp::R_PANEL, pal::RAISED);
            p.text(r.min + Vec2::new(16.0, 14.0), Align2::LEFT_TOP, &d.info.name, f_semi(15.0), pal::TEXT);
            let src = if d.info.mapped { format!("{} (standard mapping)", d.info.backend) } else { d.info.backend.to_string() };
            let sub = format!("{}  ·  {}  ·  {} axes, {} buttons bound  ·  {src}", d.info.kind.name(), prof.preset, prof.axes.len(), prof.buttons.len());
            p.text(r.min + Vec2::new(16.0, 36.0), Align2::LEFT_TOP, sub, f_ui(12.5), pal::TEXT3);
            let mut on = prof.enabled;
            if chip_btn(
                ui,
                Rect::from_min_size(Pos2::new(r.max.x - 250.0, r.min.y + 14.0), Vec2::new(92.0, 28.0)),
                ("en", &d.info.key),
                if on { "Enabled" } else { "Disabled" },
                on,
            ) {
                on = !on;
                if let Some(p) = st.settings.controls.profile_mut(&d.info.key) {
                    p.enabled = on;
                }
                st.settings_dirty = true;
            }
            if button(ui, Rect::from_min_size(Pos2::new(r.max.x - 150.0, r.min.y + 14.0), Vec2::new(134.0, 28.0)), ("rst", &d.info.key), "Reset to defaults", Btn::Secondary, true)
                .clicked()
            {
                if let Some(p) = st.settings.controls.profile_mut(&d.info.key) {
                    *p = crate::input::defaults::profile(&d.info);
                }
                st.settings_dirty = true;
            }
            // Live inputs: axes as bars, buttons as lit pills.
            let mut y = r.min.y + 66.0;
            let colw = (w - 48.0) * 0.5;
            if axes.is_empty() && btns.is_empty() {
                p.text(Pos2::new(r.min.x + 16.0, y + 4.0), Align2::LEFT_TOP, "Move a control to see it here", f_ui(12.5), pal::TEXT3);
            }
            for (i, (code, v, _)) in axes.iter().enumerate() {
                let x = r.min.x + 16.0 + (i % 2) as f32 * (colw + 16.0);
                let yy = y + (i / 2) as f32 * 24.0;
                p.text(Pos2::new(x, yy + 8.0), Align2::LEFT_CENTER, code_label(Some(&d.info), *code, Half::Full), f_ui(12.0), pal::TEXT2);
                value_bar(&p, Rect::from_min_size(Pos2::new(x + 120.0, yy + 4.0), Vec2::new(colw - 180.0, 8.0)), Some(*v), false);
                p.text(Pos2::new(x + colw - 8.0, yy + 8.0), Align2::RIGHT_CENTER, format!("{v:+.2}"), f_mono(11.5), pal::TEXT3);
            }
            y += rows as f32 * 24.0 + 6.0;
            let mut x = r.min.x + 16.0;
            for (code, v, _) in btns {
                let t = code_label(Some(&d.info), *code, Half::Full);
                let g = p.layout_no_wrap(t.clone(), f_ui(11.5), pal::TEXT);
                let br = Rect::from_min_size(Pos2::new(x, y), Vec2::new(g.size().x + 14.0, 22.0));
                if br.max.x > r.max.x - 12.0 {
                    break;
                }
                let lit = *v > 0.5;
                p.rect_filled(br, 11, if lit { pal::ACCENT_DEEP } else { Color32::from_rgb(9, 11, 15) });
                p.text(br.center(), Align2::CENTER_CENTER, t, f_ui(11.5), if lit { pal::WHITE } else { pal::TEXT3 });
                x = br.max.x + 5.0;
            }
        }
        ui.add_space(6.0);
        ui.label(RichText::new(status).font(f_ui(12.0)).color(pal::TEXT3));
    });
}

// ---------------------------------------------------------------- mouse

fn mouse_page(ui: &mut Ui, st: &mut UiState, area: Rect) {
    let p = ui.painter().clone();
    let m = &mut st.settings.controls.mouse;
    let before = m.clone();
    let x = area.min.x;
    let lw = 250.0;
    let ctrl = |y: f32| Rect::from_min_size(Pos2::new(x + lw, y + 6.0), Vec2::new(area.max.x - x - lw, 34.0));
    let label = |y: f32, k: &str, hint: &str| {
        p.text(Pos2::new(x, y + 8.0), Align2::LEFT_TOP, k, f_semi(14.0), pal::TEXT);
        p.text(Pos2::new(x, y + 28.0), Align2::LEFT_TOP, hint, f_ui(11.5), pal::TEXT3);
    };
    let mut y = area.min.y;
    label(y, "Mouse yoke", "Y toggles it in flight");
    if let Some(i) = segmented(ui, ctrl(y), "yoke", &["Off", "Available"], m.yoke as usize) {
        m.yoke = i == 1;
    }
    y += 56.0;
    let slider = |ui: &mut Ui, y: f32, v: &mut f32| {
        let r = ctrl(y);
        let sr = Rect::from_min_size(r.min, Vec2::new(r.width() - 70.0, r.height()));
        put_slider(ui, sr, v, 0.3..=2.5);
        ui.painter().text(Pos2::new(r.max.x, r.center().y), Align2::RIGHT_CENTER, format!("{:.1}×", *v), f_mono(12.0), pal::TEXT2);
    };
    label(y, "Yoke sensitivity", "Cursor offset from the centre = stick");
    slider(ui, y, &mut m.yoke_sens);
    y += 56.0;
    label(y, "Head-look speed", "Hold the right button in the cockpit and drag");
    slider(ui, y, &mut m.look_sens);
    y += 56.0;
    label(y, "Head-look vertical", "");
    if let Some(i) = segmented(ui, ctrl(y), "invlook", &["Normal", "Inverted"], m.invert_look as usize) {
        m.invert_look = i == 1;
    }
    y += 56.0;
    label(y, "After looking around", "Num 5 / right stick click centres the view");
    if let Some(i) = segmented(ui, ctrl(y), "lookret", &["View stays", "Returns to centre"], m.look_return as usize) {
        m.look_return = i == 1;
    }
    y += 64.0;
    let tips = [
        "Glance with a joystick hat, the d-pad or Num 4 / 6 / 8 / 2 while held.",
        "The gamepad's right stick looks around in the cockpit and orbits outside.",
        "Outside views: right-drag orbits, the wheel zooms.",
    ];
    for t in tips {
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, format!("·  {t}"), f_ui(12.5), pal::TEXT3);
        y += 20.0;
    }
    if *m != before {
        st.settings_dirty = true;
    }
}
