//! The settings screen: a category rail, search across every setting, one
//! row per setting (label, one-line description, a consistent control),
//! "changed" dots, restart / coming-soon badges, Apply / Revert / Reset
//! category / Reset all with confirmations, and keyboard + game-controller
//! navigation (ui/scroll.rs `nav`). The same screen opens from the main menu
//! and from the pause menu in flight.
//!
//! Changes preview live (the app applies `st.settings` every frame) but are
//! only written to disk on Apply; Revert, or leaving without applying, goes
//! back to the settings the screen opened with. The rows come from
//! ui/settings_schema.rs; Video is drawn by ui/settings_video.rs (GFX track)
//! and Controls by ui/controls.rs.

use super::chrome;
use super::menus::{Btn, backdrop, button, segmented};
use super::settings::Settings;
use super::settings_schema::{self as schema, Act, Cat, Info, Item, Kind, Val};
use super::{Action, Screen, UiState, alpha, f_display, f_mono, f_semi, f_ui, pal, panel_with, sp};
use egui::{Align2, Color32, Pos2, Rect, Sense, Stroke, Ui, UiBuilder, Vec2};
use std::sync::{Arc, Mutex};

/// A destructive or leaving action waiting for a yes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Confirm {
    ResetCat(Cat),
    ResetAll,
    Clear(Act),
    /// Leave with unapplied changes (then do this, or go back).
    Leave(Option<Action>),
}

/// Settings-screen state (in `UiState`).
pub struct SetUi {
    pub cat: Cat,
    pub query: String,
    /// The settings when the screen opened / were last applied (None: closed).
    pub base: Option<Settings>,
    /// The settings at start-up (restart badges).
    pub launch: Option<Settings>,
    /// Keyboard / controller focus: index into last frame's focus order.
    pub focus: usize,
    order: Vec<&'static str>,
    drawn: Vec<&'static str>,
    focused_id: Option<&'static str>,
    /// Show the focus ring (the last input was keys / a controller).
    kb_mode: bool,
    scroll_to_focus: bool,
    focus_search: bool,
    /// A text field has the keyboard.
    typing: bool,
    adjust: i32,
    activate: bool,
    pub confirm: Option<Confirm>,
    confirm_sel: usize,
    devices: Arc<Mutex<Option<Vec<(String, String)>>>>,
    devices_asked: bool,
    cache_size: Arc<Mutex<Option<u64>>>,
    cache_asked: bool,
    drafts: std::collections::HashMap<&'static str, String>,
    /// Headless captures: no device enumeration, no disk walks.
    pub headless: bool,
}

impl Default for SetUi {
    fn default() -> Self {
        SetUi {
            cat: Cat::Video,
            query: String::new(),
            base: None,
            launch: None,
            focus: 0,
            order: Vec::new(),
            drawn: Vec::new(),
            focused_id: None,
            kb_mode: false,
            scroll_to_focus: false,
            focus_search: false,
            typing: false,
            adjust: 0,
            activate: false,
            confirm: None,
            confirm_sel: 0,
            devices: Default::default(),
            devices_asked: false,
            cache_size: Default::default(),
            cache_asked: false,
            drafts: Default::default(),
            headless: false,
        }
    }
}

impl SetUi {
    /// Unapplied changes (the screen is open and the settings differ from its base).
    pub fn pending(&self, s: &Settings) -> bool {
        self.base.as_ref().is_some_and(|b| b != s)
    }
    /// Open on a category (headless captures, `--settings-category`).
    pub fn open_at(&mut self, cat: Cat, query: &str) {
        self.cat = cat;
        self.query = query.to_string();
        self.focus = 0;
    }
}

const GUTTER: f32 = super::scroll::GUTTER;

/// The settings screen.
pub fn screen(ui: &mut Ui, st: &mut UiState, in_flight: bool) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, if in_flight { 0.85 } else { 1.0 });
    if st.settings_ui.base.is_none() {
        st.settings_ui.base = Some(st.settings.clone());
        st.settings_ui.focus = 0;
        st.settings_ui.confirm = None;
    }
    if st.settings_ui.launch.is_none() {
        st.settings_ui.launch = Some(st.settings.clone());
    }
    super::controls::apply_captured(st);
    let flying = in_flight && st.back_to == Screen::Pause;

    // ---- input
    handle_nav(ui, st);

    // ---- layout
    let mx = (screen.width() * 0.04).clamp(10.0, 72.0);
    let my = (screen.height() * 0.05).clamp(10.0, 56.0);
    let card = Rect::from_center_size(screen.center(), Vec2::new((screen.width() - 2.0 * mx).min(1480.0), (screen.height() - 2.0 * my).min(940.0)));
    panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
    let compact = card.width() < 900.0;
    let pad = if compact { 16.0 } else { 24.0 };
    let head_h = if compact { 60.0 } else { 72.0 };
    let foot_h = if compact { 58.0 } else { 68.0 };
    let head = Rect::from_min_size(card.min, Vec2::new(card.width(), head_h));
    let foot = Rect::from_min_max(Pos2::new(card.min.x, card.max.y - foot_h), card.max);
    let body = Rect::from_min_max(Pos2::new(card.min.x, head.max.y), Pos2::new(card.max.x, foot.min.y));
    p.line_segment([Pos2::new(card.min.x + 1.0, body.min.y), Pos2::new(card.max.x - 1.0, body.min.y)], Stroke::new(1.0, pal::HAIRLINE_HI));
    p.line_segment([Pos2::new(card.min.x + 1.0, body.max.y), Pos2::new(card.max.x - 1.0, body.max.y)], Stroke::new(1.0, pal::HAIRLINE_HI));

    // ---- header: title, context, search
    let title_size = if compact { 22.0 } else { 26.0 };
    p.text(Pos2::new(card.min.x + pad, head.center().y), Align2::LEFT_CENTER, "Settings", f_display(title_size), pal::TEXT);
    let sw = (card.width() * 0.36).clamp(180.0, 400.0);
    let sr = Rect::from_min_size(Pos2::new(card.max.x - pad - sw, head.center().y - 19.0), Vec2::new(sw, 38.0));
    if !compact || card.width() > 640.0 {
        let tw = p.layout_no_wrap("Settings".into(), f_display(title_size), pal::TEXT).size().x;
        let ctx_text = if flying { "In flight · paused" } else { "" };
        if !ctx_text.is_empty() && card.min.x + pad + tw + 24.0 + 120.0 < sr.min.x {
            p.text(Pos2::new(card.min.x + pad + tw + 16.0, head.center().y + 2.0), Align2::LEFT_CENTER, ctx_text, f_ui(13.0), pal::TEXT3);
        }
    }
    search_box(ui, st, sr);

    // ---- rail
    let rail_w = (card.width() * 0.22).clamp(150.0, 250.0);
    let rail = Rect::from_min_max(Pos2::new(card.min.x + pad * 0.5, body.min.y + 10.0), Pos2::new(card.min.x + pad * 0.5 + rail_w, body.max.y - 10.0));
    draw_rail(ui, st, rail);
    p.line_segment([Pos2::new(rail.max.x + pad * 0.5, body.min.y), Pos2::new(rail.max.x + pad * 0.5, body.max.y)], Stroke::new(1.0, pal::HAIRLINE));

    // ---- content
    let cx = rail.max.x + pad;
    let content = Rect::from_min_max(Pos2::new(cx, body.min.y + 14.0), Pos2::new(card.max.x - pad * 0.5, body.max.y - 6.0));
    st.settings_ui.drawn.clear();
    let searching = !st.settings_ui.query.trim().is_empty();
    let (title, blurb) = if searching {
        let n = schema::search(&st.settings_ui.query, flying).len();
        ("Search".to_string(), format!("{n} {} “{}”", if n == 1 { "setting matches" } else { "settings match" }, st.settings_ui.query.trim()))
    } else {
        (st.settings_ui.cat.name().to_string(), st.settings_ui.cat.blurb().to_string())
    };
    p.text(Pos2::new(cx, content.min.y), Align2::LEFT_TOP, &title, f_display(if compact { 19.0 } else { 22.0 }), pal::TEXT);
    p.text(Pos2::new(cx, content.min.y + if compact { 26.0 } else { 31.0 }), Align2::LEFT_TOP, fit(&p, &blurb, f_ui(13.0), content.width() - GUTTER), f_ui(13.0), pal::TEXT3);
    let mut top = content.min.y + if compact { 50.0 } else { 60.0 };
    // Restart banner.
    let restart: Vec<&str> = st.settings_ui.launch.as_ref().map(|l| schema::restart_needed(&st.settings, l).iter().map(|i| i.label).collect()).unwrap_or_default();
    if !restart.is_empty() {
        let r = Rect::from_min_size(Pos2::new(cx, top), Vec2::new(content.width() - GUTTER, 34.0));
        p.rect_filled(r, sp::R_CTRL, alpha(pal::AMBER, 0.08));
        p.rect_stroke(r, sp::R_CTRL, Stroke::new(1.0, alpha(pal::AMBER, 0.6)), egui::StrokeKind::Inside);
        p.text(
            Pos2::new(r.min.x + 12.0, r.center().y),
            Align2::LEFT_CENTER,
            fit(&p, &format!("Restart Skyward to finish applying: {}", restart.join(", ")), f_semi(13.0), r.width() - 24.0),
            f_semi(13.0),
            pal::AMBER,
        );
        top = r.max.y + 10.0;
    }
    let rows_r = Rect::from_min_max(Pos2::new(cx, top), content.max);
    if searching {
        let q = st.settings_ui.query.clone();
        super::scroll::area(ui, rows_r, "settings_search", pal::PANEL_SOLID, |ui| search_results(ui, st, &q, flying));
    } else {
        match st.settings_ui.cat {
            Cat::Controls => super::controls::page(ui, st, Rect::from_min_max(rows_r.min, Pos2::new(rows_r.max.x - GUTTER, rows_r.max.y)), None),
            Cat::Video => {
                super::scroll::area(ui, rows_r, "settings_video", pal::PANEL_SOLID, |ui| {
                    super::settings_video::video_tab(ui, st);
                    ui.add_space(12.0);
                });
            }
            cat => {
                let rows = schema::rows(cat, flying);
                super::scroll::area(ui, rows_r, ("settings_cat", cat.id()), pal::PANEL_SOLID, |ui| {
                    rows_ui(ui, st, &rows);
                    ui.add_space(12.0);
                });
            }
        }
    }

    // ---- footer
    footer(ui, st, foot, pad, compact, searching);

    // ---- focus order for the next frame
    let s = &mut st.settings_ui;
    s.order = std::mem::take(&mut s.drawn);
    s.order.extend(schema::FOOTER);
    s.focus = s.focus.min(s.order.len().saturating_sub(1));
    s.focused_id = s.order.get(s.focus).copied();
    s.scroll_to_focus = false;
    s.adjust = 0;
    s.activate = false;

    if let Some(c) = st.settings_ui.confirm {
        confirm_dialog(ui, st, c, screen);
    }
}

// ---------------------------------------------------------------- input

fn handle_nav(ui: &mut Ui, st: &mut UiState) {
    let n = super::scroll::nav(ui, &st.input, st.frame_no);
    let moved_mouse = ui.input(|i| i.pointer.delta().length_sq() > 0.5 || i.pointer.any_pressed());
    let s = &mut st.settings_ui;
    if moved_mouse {
        s.kb_mode = false;
    }
    // Ctrl+F / "/" focuses search.
    let find = ui.input(|i| (i.modifiers.command && i.key_pressed(egui::Key::F)) || (!s.typing && i.key_pressed(egui::Key::Slash)));
    if find && s.confirm.is_none() {
        s.focus_search = true;
        return;
    }
    if let Some(c) = s.confirm {
        let nbtn = confirm_buttons(c).len();
        if n.dx != 0 || n.tab != 0 {
            s.confirm_sel = (s.confirm_sel as i32 + n.dx + n.tab).clamp(0, nbtn as i32 - 1) as usize;
        }
        if n.back {
            s.confirm = None;
        } else if n.ok {
            let sel = s.confirm_sel;
            confirm_choose(st, c, sel);
        }
        return;
    }
    if s.typing {
        // The search box / a field has the keyboard: Down or Enter hands it back to the list.
        if n.dy > 0 || n.ok {
            ui.ctx().memory_mut(|m| m.request_focus(egui::Id::NULL));
            ui.ctx().memory_mut(|m| m.surrender_focus(egui::Id::new("settings_search_box")));
            s.focus = 0;
            s.kb_mode = true;
            s.scroll_to_focus = true;
        } else if n.back {
            if s.query.is_empty() {
                ui.ctx().memory_mut(|m| m.surrender_focus(egui::Id::new("settings_search_box")));
            } else {
                s.query.clear();
            }
        }
        return;
    }
    if n != super::scroll::Nav::default() {
        s.kb_mode = true;
    }
    if n.tab != 0 {
        let i = Cat::ALL.iter().position(|c| *c == s.cat).unwrap_or(0) as i32;
        s.cat = Cat::ALL[(i + n.tab).rem_euclid(Cat::ALL.len() as i32) as usize];
        s.query.clear();
        s.focus = 0;
        st.input.listen = None;
        st.input.calib = None;
        return;
    }
    let len = s.order.len().max(1);
    let foot0 = len.saturating_sub(schema::FOOTER.len());
    if n.dy != 0 {
        // From the footer, up goes back to the last row.
        s.focus = if s.focus >= foot0 && n.dy < 0 { foot0.saturating_sub(1) } else { (s.focus as i32 + n.dy).clamp(0, len as i32 - 1) as usize };
        if n.dy > 0 && s.focus > foot0 {
            s.focus = foot0;
        }
        s.scroll_to_focus = true;
    }
    if n.ends != 0 {
        s.focus = if n.ends < 0 { 0 } else { foot0.saturating_sub(1) };
        s.scroll_to_focus = true;
    }
    if n.dx != 0 {
        if s.focus >= foot0 {
            s.focus = (s.focus as i32 + n.dx).clamp(foot0 as i32, len as i32 - 1) as usize;
        } else {
            s.adjust = n.dx;
        }
    }
    s.activate = n.ok;
    if n.back && !super::controls::swallow_esc(st) {
        if !st.settings_ui.query.is_empty() {
            st.settings_ui.query.clear();
        } else {
            leave(st, None);
        }
    }
}

/// Leave the screen (asking first when changes are not applied).
fn leave(st: &mut UiState, then: Option<Action>) {
    if st.settings_ui.pending(&st.settings) {
        st.settings_ui.confirm = Some(Confirm::Leave(then));
        st.settings_ui.confirm_sel = 2;
        return;
    }
    finish_leave(st, then);
}

fn finish_leave(st: &mut UiState, then: Option<Action>) {
    st.settings_ui.base = None;
    st.settings_ui.confirm = None;
    st.settings_ui.query.clear();
    st.input.listen = None;
    st.input.calib = None;
    st.actions.push(then.unwrap_or(Action::Back));
}

/// Make the current settings the saved ones.
pub fn apply(st: &mut UiState) {
    let base = st.settings_ui.base.clone().unwrap_or_else(|| st.settings.clone());
    if !crate::platform::WEB && st.settings.player_name != base.player_name && !st.settings.player_name.trim().is_empty() {
        let n = skyward_net::codec::sanitize(st.settings.player_name.trim(), skyward_net::proto::MAX_NAME);
        st.settings.player_name = n.clone();
        st.mp.prefs.name = n;
        st.mp.prefs.save();
    }
    if st.settings.mp.name_tags != base.mp.name_tags {
        st.mp.tags = st.settings.mp.name_tags;
    }
    st.settings_ui.base = Some(st.settings.clone());
    st.settings_dirty = true;
    let restart = st.settings_ui.launch.as_ref().map(|l| schema::restart_needed(&st.settings, l).len()).unwrap_or(0);
    if restart > 0 {
        st.toasts.push(super::toast::Kind::Warn, "Settings applied", "Some changes take effect after a restart", 3.5);
    } else {
        st.toasts.push(super::toast::Kind::Good, "Settings applied", "", 1.8);
    }
}

/// Back to the settings the screen opened with.
pub fn revert(st: &mut UiState) {
    if let Some(b) = st.settings_ui.base.clone() {
        st.settings = b;
        st.input.ensure_profiles(&mut st.settings.controls);
        st.settings_ui.drafts.clear();
    }
}

// ---------------------------------------------------------------- header / rail / footer

fn search_box(ui: &mut Ui, st: &mut UiState, r: Rect) {
    let p = ui.painter().clone();
    p.rect_filled(r, r.height() * 0.5, Color32::from_rgb(9, 11, 15));
    let id = egui::Id::new("settings_search_box");
    let focused = ui.ctx().memory(|m| m.has_focus(id));
    p.rect_stroke(r, r.height() * 0.5, Stroke::new(1.0, if focused { pal::ACCENT } else { pal::HAIRLINE_HI }), egui::StrokeKind::Inside);
    // Magnifier.
    let c = Pos2::new(r.min.x + 20.0, r.center().y - 1.0);
    p.circle_stroke(c, 5.5, Stroke::new(1.6, pal::TEXT3));
    p.line_segment([c + Vec2::new(4.0, 4.0), c + Vec2::new(8.0, 8.0)], Stroke::new(1.8, pal::TEXT3));
    let has = !st.settings_ui.query.is_empty();
    let right = if has { 34.0 } else { 12.0 };
    let tr = Rect::from_min_max(Pos2::new(r.min.x + 36.0, r.min.y + 8.0), Pos2::new(r.max.x - right, r.max.y - 8.0));
    let te = ui.put(tr, egui::TextEdit::singleline(&mut st.settings_ui.query).id(id).hint_text("Search settings").frame(egui::Frame::NONE).font(f_ui(14.5)).text_color(pal::TEXT));
    if std::mem::take(&mut st.settings_ui.focus_search) {
        te.request_focus();
    }
    if te.changed() {
        st.settings_ui.focus = 0;
    }
    st.settings_ui.typing = te.has_focus() || st.settings_ui.drafts_focused(ui);
    if !has && !focused && r.width() > 230.0 {
        let k = if cfg!(target_os = "macos") { "⌘ F" } else { "Ctrl F" };
        let w = p.layout_no_wrap(k.into(), f_semi(11.5), pal::TEXT2).size().x + 14.0;
        chrome::keycap(&p, Pos2::new(r.max.x - 10.0 - w, r.center().y), k);
    }
    if has {
        let xr = Rect::from_center_size(Pos2::new(r.max.x - 19.0, r.center().y), Vec2::splat(26.0));
        if chrome::close_x(ui, xr, "settings_search_clear") {
            st.settings_ui.query.clear();
        }
    }
}

impl SetUi {
    fn drafts_focused(&self, ui: &Ui) -> bool {
        self.drafts.keys().any(|k| ui.ctx().memory(|m| m.has_focus(egui::Id::new(("settings_field", *k)))))
    }
}

fn draw_rail(ui: &mut Ui, st: &mut UiState, rail: Rect) {
    let searching = !st.settings_ui.query.trim().is_empty();
    let base = st.settings_ui.base.clone().unwrap_or_else(|| st.settings.clone());
    let row_h = if rail.height() < 480.0 { 32.0 } else { 38.0 };
    let need = row_h * Cat::ALL.len() as f32;
    let small = rail.width() < 180.0;
    super::scroll::area(ui, rail, "settings_rail", if need > rail.height() { pal::PANEL_SOLID } else { Color32::TRANSPARENT }, |ui| {
        let w = ui.available_width() - if need > rail.height() { GUTTER } else { 0.0 };
        for c in Cat::ALL {
            let (r, resp) = ui.allocate_exact_size(Vec2::new(w, row_h), Sense::click());
            let p = ui.painter();
            let sel = !searching && c == st.settings_ui.cat;
            if sel {
                p.rect_filled(r, sp::R_CTRL, pal::RAISED_HI);
                p.rect_filled(Rect::from_min_size(Pos2::new(r.min.x, r.min.y + 8.0), Vec2::new(3.0, r.height() - 16.0)), 1.5, pal::ACCENT);
            } else if resp.hovered() {
                p.rect_filled(r, sp::R_CTRL, pal::RAISED);
            }
            let font = f_semi(if small { 12.5 } else { 14.0 });
            let label = fit(p, c.name(), font.clone(), r.width() - 34.0);
            p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, font, if sel { pal::TEXT } else { pal::TEXT2 });
            if schema::cat_changed(c, &st.settings, &base) {
                p.circle_filled(Pos2::new(r.max.x - 12.0, r.center().y), 3.5, pal::ACCENT);
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }
            if resp.clicked() {
                let s = &mut st.settings_ui;
                s.cat = c;
                s.query.clear();
                s.focus = 0;
                st.input.listen = None;
                st.input.calib = None;
            }
        }
    });
}

fn footer(ui: &mut Ui, st: &mut UiState, foot: Rect, pad: f32, compact: bool, searching: bool) {
    let p = ui.painter().clone();
    let h = if compact { 38.0 } else { 44.0 };
    let y = foot.center().y - h * 0.5;
    let pending = st.settings_ui.pending(&st.settings);
    let (kb, act, fid) = (st.settings_ui.kb_mode, st.settings_ui.activate, st.settings_ui.focused_id);
    let focused = |id: &str| kb && fid == Some(id);
    let activated = |id: &str| act && fid == Some(id);
    let mut x = foot.min.x + pad;
    // Back.
    let bw = if compact { 84.0 } else { 104.0 };
    let br = Rect::from_min_size(Pos2::new(x, y), Vec2::new(bw, h));
    if button(ui, br, "set_back", "Back", Btn::Ghost, true).clicked() {
        leave(st, None);
    }
    x = br.max.x + 8.0;
    let cat = st.settings_ui.cat;
    let can_reset = !searching && !matches!(cat, Cat::About);
    let rw = if compact { 96.0 } else { 160.0 };
    let rr = Rect::from_min_size(Pos2::new(x, y), Vec2::new(rw, h));
    let reset_label = if compact { "Reset" } else { "Reset category" };
    if (button(ui, rr, "set_reset_cat", reset_label, Btn::Secondary, can_reset).on_hover_text(format!("Defaults for {}", cat.name())).clicked() || activated("@reset_cat"))
        && can_reset
    {
        st.settings_ui.confirm = Some(Confirm::ResetCat(cat));
        st.settings_ui.confirm_sel = 0;
    }
    ring(&p, rr, focused("@reset_cat"));
    x = rr.max.x + 8.0;
    let aw = if compact { 88.0 } else { 112.0 };
    let ar = Rect::from_min_size(Pos2::new(x, y), Vec2::new(aw, h));
    if button(ui, ar, "set_reset_all", "Reset all", Btn::Ghost, true).clicked() || activated("@reset_all") {
        st.settings_ui.confirm = Some(Confirm::ResetAll);
        st.settings_ui.confirm_sel = 0;
    }
    ring(&p, ar, focused("@reset_all"));
    // Right: Apply, Revert.
    let apw = if compact { 96.0 } else { 128.0 };
    let apr = Rect::from_min_size(Pos2::new(foot.max.x - pad - apw, y), Vec2::new(apw, h));
    if (button(ui, apr, "set_apply", "Apply", Btn::Primary, pending).clicked() || activated("@apply")) && pending {
        apply(st);
    }
    ring(&p, apr, focused("@apply"));
    let rvw = if compact { 88.0 } else { 112.0 };
    let rvr = Rect::from_min_size(Pos2::new(apr.min.x - 8.0 - rvw, y), Vec2::new(rvw, h));
    if (button(ui, rvr, "set_revert", "Revert", Btn::Secondary, pending).clicked() || activated("@revert")) && pending {
        revert(st);
        st.toasts.push(super::toast::Kind::Info, "Changes reverted", "", 1.6);
    }
    ring(&p, rvr, focused("@revert"));
    // Middle: what is pending, else key hints.
    let mid = Rect::from_min_max(Pos2::new(ar.max.x + 16.0, foot.min.y), Pos2::new(rvr.min.x - 16.0, foot.max.y));
    if pending {
        let base = st.settings_ui.base.clone().unwrap_or_default();
        let n = schema::items().iter().filter(|i| schema::changed(i, &st.settings, &base)).count() + (st.settings.controls != base.controls) as usize;
        let t = if n == 0 { "Unapplied changes".to_string() } else { format!("{n} unapplied change{}", if n == 1 { "" } else { "s" }) };
        let g = p.layout_no_wrap(t.clone(), f_ui(13.0), pal::TEXT2);
        if g.size().x + 18.0 < mid.width() {
            let at = Pos2::new(mid.max.x - g.size().x, mid.center().y);
            p.circle_filled(Pos2::new(at.x - 10.0, at.y), 3.5, pal::ACCENT);
            p.text(at, Align2::LEFT_CENTER, t, f_ui(13.0), pal::TEXT2);
        }
    } else {
        let pad_in = st.input.devices.iter().any(|d| d.info.kind == crate::input::DevKind::Gamepad);
        let hints: &[(&str, &str)] =
            if pad_in { &[("A", "Select"), ("LB  RB", "Category"), ("B", "Back")] } else { &[("↑ ↓", "Move"), ("← →", "Change"), ("Tab", "Category"), ("Esc", "Back")] };
        chrome::key_hints(&p, mid, hints);
    }
}

/// The keyboard focus ring.
fn ring(p: &egui::Painter, r: Rect, on: bool) {
    if on {
        p.rect_stroke(r.expand(3.0), sp::R_CTRL + 3, Stroke::new(2.0, pal::ACCENT), egui::StrokeKind::Outside);
    }
}

// ---------------------------------------------------------------- confirmations

fn confirm_buttons(c: Confirm) -> Vec<(&'static str, Btn)> {
    match c {
        Confirm::Leave(_) => vec![("Discard", Btn::Ghost), ("Cancel", Btn::Secondary), ("Apply", Btn::Primary)],
        Confirm::Clear(_) => vec![("Cancel", Btn::Secondary), ("Clear", Btn::Primary)],
        _ => vec![("Cancel", Btn::Secondary), ("Reset", Btn::Primary)],
    }
}

fn confirm_text(c: Confirm) -> (String, String) {
    match c {
        Confirm::ResetCat(cat) => {
            (format!("Reset {}?", cat.name()), "Every setting in this category goes back to its default. Apply to keep the change, or Revert to undo it.".into())
        }
        Confirm::ResetAll => (
            "Reset all settings?".into(),
            "Every setting, key binding and controller profile goes back to its default. Your recent airports, favourites and player name are kept.".into(),
        ),
        Confirm::Clear(Act::ClearTerrain) => ("Clear the terrain cache?".into(), "Elevation data will download again as you fly. This cannot be undone.".into()),
        Confirm::Clear(_) => ("Clear the map tile cache?".into(), "Roads, buildings and airport layouts will download again as you fly. This cannot be undone.".into()),
        Confirm::Leave(_) => ("Apply your changes?".into(), "Some settings have changed but are not applied yet.".into()),
    }
}

fn confirm_choose(st: &mut UiState, c: Confirm, sel: usize) {
    st.settings_ui.confirm = None;
    match c {
        Confirm::Leave(then) => match sel {
            0 => {
                revert(st);
                finish_leave(st, then);
            }
            2 => {
                apply(st);
                finish_leave(st, then);
            }
            _ => {}
        },
        Confirm::ResetCat(cat) if sel == 1 => {
            schema::reset_category(&mut st.settings, cat);
            st.input.ensure_profiles(&mut st.settings.controls);
            st.input.listen = None;
            st.settings_ui.drafts.clear();
            st.toasts.push(super::toast::Kind::Info, format!("{} reset to defaults", cat.name()), "Apply to keep the change", 2.5);
        }
        Confirm::ResetAll if sel == 1 => {
            schema::reset_all(&mut st.settings);
            st.input.ensure_profiles(&mut st.settings.controls);
            st.input.listen = None;
            st.settings_ui.drafts.clear();
            st.toasts.push(super::toast::Kind::Info, "All settings reset to defaults", "Apply to keep the change", 2.5);
        }
        Confirm::Clear(act) if sel == 1 => clear_cache(st, act),
        _ => {}
    }
}

fn confirm_dialog(ui: &mut Ui, st: &mut UiState, c: Confirm, screen: Rect) {
    let (title, body) = confirm_text(c);
    let btns = confirm_buttons(c);
    egui::Area::new(egui::Id::new("settings_confirm")).fixed_pos(screen.min).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
        let (_, bg) = ui.allocate_exact_size(screen.size(), Sense::click());
        let p = ui.painter().clone();
        p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(3, 4, 6, 170));
        let w = 460.0f32.min(screen.width() - 32.0);
        let g = p.layout(body, f_ui(14.0), pal::TEXT2, w - 56.0);
        let h = 68.0 + g.size().y + 84.0;
        let card = Rect::from_center_size(screen.center(), Vec2::new(w, h));
        panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
        p.text(card.min + Vec2::new(28.0, 26.0), Align2::LEFT_TOP, title, f_display(20.0), pal::TEXT);
        p.galley(card.min + Vec2::new(28.0, 62.0), g, pal::TEXT2);
        let bw = ((w - 56.0 - 10.0 * (btns.len() as f32 - 1.0)) / btns.len() as f32).min(130.0);
        let mut x = card.max.x - 28.0 - bw * btns.len() as f32 - 10.0 * (btns.len() as f32 - 1.0);
        let y = card.max.y - 28.0 - 42.0;
        let mut chosen = None;
        for (i, (label, kind)) in btns.iter().enumerate() {
            let r = Rect::from_min_size(Pos2::new(x, y), Vec2::new(bw, 42.0));
            if button(ui, r, ("confirm", i), label, *kind, true).clicked() {
                chosen = Some(i);
            }
            ring(&p, r, st.settings_ui.kb_mode && st.settings_ui.confirm_sel == i);
            x += bw + 10.0;
        }
        if let Some(i) = chosen {
            confirm_choose(st, c, i);
        } else if bg.clicked() && !card.contains(bg.interact_pointer_pos().unwrap_or(card.center())) {
            st.settings_ui.confirm = None;
        }
    });
}

// ---------------------------------------------------------------- rows

/// Draw `rows` (with their group headings) into a vertical `ui`; true when a value changed.
/// One catalogue row in the shell's style (for categories drawn by their
/// own module, ui/settings_video.rs); true when its value changed.
pub fn setting_row(ui: &mut Ui, st: &mut UiState, it: &'static Item) -> bool {
    row(ui, st, it, false)
}

pub fn rows_ui(ui: &mut Ui, st: &mut UiState, rows: &[&'static Item]) -> bool {
    let mut group = "";
    let mut changed = false;
    for it in rows {
        if it.group != group {
            group = it.group;
            ui.add_space(if group.is_empty() { 0.0 } else { 6.0 });
            ui.label(egui::RichText::new(group.to_uppercase()).font(super::f_inst_b(11.0)).color(pal::ACCENT));
            ui.add_space(2.0);
        }
        changed |= row(ui, st, it, false);
        ui.add_space(4.0);
    }
    changed
}

fn search_results(ui: &mut Ui, st: &mut UiState, q: &str, flying: bool) {
    let hits = schema::search(q, flying);
    let cats = schema::search_categories(q);
    // Whole categories (Video / Controls / About) as links.
    for c in &cats {
        let w = ui.available_width() - GUTTER;
        let (r, resp) = ui.allocate_exact_size(Vec2::new(w, 44.0), Sense::click());
        let p = ui.painter();
        p.rect_filled(r, sp::R_CTRL, if resp.hovered() { pal::RAISED_HI } else { alpha(pal::RAISED, 0.45) });
        p.text(Pos2::new(r.min.x + 22.0, r.center().y), Align2::LEFT_CENTER, format!("Open {}", c.name()), f_semi(14.0), pal::TEXT);
        p.text(Pos2::new(r.max.x - 16.0, r.center().y), Align2::RIGHT_CENTER, "›", f_semi(18.0), pal::TEXT2);
        if resp.clicked() {
            st.settings_ui.cat = *c;
            st.settings_ui.query.clear();
            st.settings_ui.focus = 0;
        }
        ui.add_space(4.0);
    }
    for it in &hits {
        row(ui, st, it, true);
        ui.add_space(4.0);
    }
    // Key bindings by name.
    let ql = q.trim().to_lowercase();
    let binds: Vec<_> = crate::input::BUTTONS.iter().filter(|(_, _, label, _)| ql.split_whitespace().all(|w| label.to_lowercase().contains(w))).take(12).collect();
    if !binds.is_empty() {
        ui.add_space(6.0);
        ui.label(egui::RichText::new("KEY BINDINGS").font(super::f_inst_b(11.0)).color(pal::ACCENT));
        ui.add_space(2.0);
        for &&(k, _, label, group) in &binds {
            binding_row(ui, st, k, label, group);
            ui.add_space(4.0);
        }
    }
    if hits.is_empty() && cats.is_empty() && binds.is_empty() {
        ui.add_space(30.0);
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new(format!("No settings match “{}”", q.trim())).font(f_semi(16.0)).color(pal::TEXT));
            ui.add_space(4.0);
            ui.label(egui::RichText::new("Try a shorter word, or browse the categories on the left.").font(f_ui(13.0)).color(pal::TEXT3));
        });
    }
}

/// A key binding found by search: the bound keys and a button to add one.
fn binding_row(ui: &mut Ui, st: &mut UiState, k: crate::sim::Key, label: &str, group: &str) {
    let w = ui.available_width() - GUTTER;
    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 52.0), Sense::hover());
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, alpha(pal::RAISED, 0.45));
    p.text(Pos2::new(r.min.x + 22.0, r.min.y + 9.0), Align2::LEFT_TOP, label, f_semi(14.0), pal::TEXT);
    p.text(Pos2::new(r.min.x + 22.0, r.min.y + 30.0), Align2::LEFT_TOP, format!("Controls · {group}"), f_ui(12.0), pal::TEXT3);
    let listening = st.input.listen == Some(crate::input::Listen::Key(k));
    let br = Rect::from_min_size(Pos2::new(r.max.x - 12.0 - 110.0, r.center().y - 16.0), Vec2::new(110.0, 32.0));
    if button(ui, br, ("bind_add", label), if listening { "Press a key…" } else { "Add key" }, if listening { Btn::Primary } else { Btn::Secondary }, true).clicked() {
        if listening {
            st.input.listen = None;
        } else {
            st.input.start_listening(crate::input::Listen::Key(k));
        }
    }
    let mut x = br.min.x - 10.0;
    for c in st.settings.controls.keys_of(k).iter().rev() {
        let t = crate::input::keys::label(*c);
        let gw = p.layout_no_wrap(t.clone(), f_semi(11.5), pal::TEXT2).size().x + 14.0;
        if x - gw < r.min.x + 220.0 {
            break;
        }
        chrome::keycap(&p, Pos2::new(x - gw, r.center().y), &t);
        x -= gw + 6.0;
    }
}

/// One settings row; true when its value changed.
fn row(ui: &mut Ui, st: &mut UiState, it: &'static Item, show_cat: bool) -> bool {
    let w = ui.available_width() - GUTTER;
    let desc = if show_cat { if it.desc.is_empty() { it.cats[0].name().to_string() } else { format!("{} · {}", it.cats[0].name(), it.desc) } } else { it.desc.to_string() };
    let h = if desc.is_empty() { 46.0 } else { 58.0 };
    let (r, resp) = ui.allocate_exact_size(Vec2::new(w, h), Sense::click());
    let info = matches!(it.kind, Kind::Info(_));
    if !info {
        st.settings_ui.drawn.push(it.id);
    }
    let focused = !info && st.settings_ui.focused_id == Some(it.id) && st.settings_ui.drawn.len() - 1 == st.settings_ui.focus;
    if focused && st.settings_ui.scroll_to_focus {
        ui.scroll_to_rect(r.expand(8.0), None);
    }
    let adjust = if focused { st.settings_ui.adjust } else { 0 };
    let activate = focused && st.settings_ui.activate;
    if resp.clicked() && !info {
        st.settings_ui.focus = st.settings_ui.drawn.len() - 1;
    }
    let p = ui.painter().clone();
    let base = st.settings_ui.base.clone().unwrap_or_else(|| st.settings.clone());
    let fill = if focused && st.settings_ui.kb_mode {
        pal::RAISED_HI
    } else if resp.hovered() && !info {
        pal::RAISED
    } else {
        alpha(pal::RAISED, 0.45)
    };
    p.rect_filled(r, sp::R_CTRL, fill);
    ring(&p, r.shrink(1.0), focused && st.settings_ui.kb_mode);
    // Control column.
    let cw = (w * 0.46).clamp(180.0, 430.0).min(w - 170.0);
    let cr = Rect::from_min_size(Pos2::new(r.max.x - 12.0 - cw, r.center().y - 17.0), Vec2::new(cw, 34.0));
    let lx = r.min.x + 22.0;
    let label_w = cr.min.x - lx - 14.0;
    // Changed (not yet applied) dot.
    if schema::changed(it, &st.settings, &base) {
        p.circle_filled(Pos2::new(r.min.x + 10.0, if desc.is_empty() { r.center().y } else { r.min.y + 19.0 }), 3.5, pal::ACCENT);
    }
    let ly = if desc.is_empty() { r.center().y } else { r.min.y + 19.0 };
    let label = fit(&p, it.label, f_semi(14.0), label_w);
    let lg = p.layout_no_wrap(label.clone(), f_semi(14.0), pal::TEXT);
    let lw = lg.size().x;
    p.text(Pos2::new(lx, ly), Align2::LEFT_CENTER, label, f_semi(14.0), pal::TEXT);
    let mut bx = lx + lw + 8.0;
    if it.restart && bx + 70.0 < cr.min.x {
        bx = chrome::badge(&p, Pos2::new(bx, ly), "Restart", pal::AMBER).max.x + 6.0;
    }
    if it.pending.is_some() && bx + 50.0 < cr.min.x {
        chrome::badge(&p, Pos2::new(bx, ly), "Soon", pal::TEXT3);
    }
    if !desc.is_empty() {
        p.text(Pos2::new(lx, r.min.y + 32.0), Align2::LEFT_TOP, fit(&p, &desc, f_ui(12.0), label_w), f_ui(12.0), pal::TEXT3);
    }
    if !desc.is_empty() && resp.hovered() && p.layout_no_wrap(desc.clone(), f_ui(12.0), pal::TEXT3).size().x > label_w {
        resp.clone().on_hover_text(&desc);
    }
    // The control.
    let cur = (it.get)(&st.settings);
    let mut new: Option<Val> = None;
    let id = ("setrow", it.id);
    match it.kind {
        Kind::Toggle => {
            let on = cur.b();
            toggle_switch(&p, cr, on);
            if resp.clicked() || activate || adjust != 0 {
                new = Some(Val::B(if adjust > 0 {
                    true
                } else if adjust < 0 {
                    false
                } else {
                    !on
                }));
            }
        }
        Kind::Choice(opts) => {
            let sel = cur.i().min(opts.len() - 1);
            // (Custom realism is shown, not picked.)
            let pickable = if it.id == "realism_preset" { 3 } else { opts.len() };
            let seg_fits = opts.len() <= 4 && opts.iter().all(|o| p.layout_no_wrap(o.to_string(), f_semi(13.0), pal::TEXT).size().x + 16.0 < cr.width() / opts.len() as f32);
            if opts.len() == 1 {
                p.text(Pos2::new(cr.max.x - 12.0, cr.center().y), Align2::RIGHT_CENTER, opts[0], f_semi(13.0), pal::TEXT2);
            } else if seg_fits {
                if let Some(i) = segmented(ui, cr, &format!("seg_{}", it.id), opts, sel) {
                    if i < pickable {
                        new = Some(Val::I(i));
                    }
                }
            } else if let Some(d) = stepper(ui, cr, id, opts[sel], sel > 0, sel + 1 < pickable) {
                new = Some(Val::I((sel as i32 + d).clamp(0, pickable as i32 - 1) as usize));
            }
            if adjust != 0 || activate {
                let d = if adjust != 0 { adjust } else { 1 };
                let n = if activate && adjust == 0 { (sel + 1) % pickable } else { (sel as i32 + d).clamp(0, pickable as i32 - 1) as usize };
                if n != sel {
                    new = Some(Val::I(n));
                }
            }
        }
        Kind::Slider { min, max, step, fmt } => {
            let mut v = cur.f();
            let vw = 64.0;
            let sr = Rect::from_min_max(cr.min, Pos2::new(cr.max.x - vw - 8.0, cr.max.y));
            if slider(ui, sr, &mut v, min..=max) {
                new = Some(Val::F((v / step).round() * step));
            }
            p.text(Pos2::new(cr.max.x, cr.center().y), Align2::RIGHT_CENTER, fmt(v), f_mono(12.5), pal::TEXT2);
            if adjust != 0 {
                new = Some(Val::F(((v + adjust as f32 * step) / step).round() * step));
            }
        }
        Kind::Text { max, digits } => {
            let fid = egui::Id::new(("settings_field", it.id));
            let has = ui.ctx().memory(|m| m.has_focus(fid));
            let draft = st.settings_ui.drafts.entry(it.id).or_insert_with(|| cur.s().to_string());
            if !has {
                *draft = cur.s().to_string();
            }
            p.rect_filled(cr, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
            p.rect_stroke(cr, sp::R_CTRL, Stroke::new(1.0, if has { pal::ACCENT } else { pal::HAIRLINE_HI }), egui::StrokeKind::Inside);
            let te = ui.put(
                cr.shrink2(Vec2::new(12.0, 7.0)),
                egui::TextEdit::singleline(draft).id(fid).char_limit(max).frame(egui::Frame::NONE).font(if digits { f_mono(14.0) } else { f_ui(14.0) }).text_color(pal::TEXT),
            );
            if te.changed() {
                if digits {
                    draft.retain(|c| c.is_ascii_digit());
                }
                new = Some(Val::S(draft.clone()));
            }
            if activate {
                te.request_focus();
            }
        }
        Kind::Device => {
            let list = devices(st);
            let cur_id = cur.s().to_string();
            let mut names: Vec<(String, String)> = vec![(String::new(), "System default".into())];
            names.extend(list);
            if !cur_id.is_empty() && !names.iter().any(|(i, _)| *i == cur_id) {
                names.push((cur_id.clone(), format!("{cur_id} (not connected)")));
            }
            let sel = names.iter().position(|(i, _)| *i == cur_id).unwrap_or(0);
            let d = stepper(ui, cr, id, &names[sel].1, sel > 0, sel + 1 < names.len()).unwrap_or(adjust);
            if d != 0 {
                let n = (sel as i32 + d).clamp(0, names.len() as i32 - 1) as usize;
                if n != sel {
                    new = Some(Val::S(names[n].0.clone()));
                }
            }
        }
        Kind::Action { act, button: label, confirm } => {
            let bw = 120.0f32.min(cr.width());
            let br = Rect::from_min_size(Pos2::new(cr.max.x - bw, cr.min.y), Vec2::new(bw, cr.height()));
            if button(ui, br, id, label, Btn::Secondary, true).clicked() || activate {
                if confirm {
                    st.settings_ui.confirm = Some(Confirm::Clear(act));
                    st.settings_ui.confirm_sel = 0;
                } else {
                    run_action(st, act);
                }
            }
        }
        Kind::Info(Info::Palette) => {
            // Status colours as chips (the palette transform recolours them on screen).
            let items = [("Engaged", pal::GREEN), ("Caution", pal::AMBER), ("Warning", pal::RED), ("Selected", pal::CYAN), ("Route", pal::MAGENTA)];
            let gap = 6.0;
            let cw = (cr.width() - gap * 4.0) / 5.0;
            for (k, (t, c)) in items.iter().enumerate() {
                let r = Rect::from_min_size(Pos2::new(cr.min.x + k as f32 * (cw + gap), cr.min.y + 3.0), Vec2::new(cw, cr.height() - 6.0));
                p.rect_filled(r, sp::R_CTRL, alpha(*c, 0.14));
                p.rect_stroke(r, sp::R_CTRL, Stroke::new(1.0, *c), egui::StrokeKind::Inside);
                p.text(r.center(), Align2::CENTER_CENTER, fit(&p, t, f_semi(12.0), cw - 6.0), f_semi(12.0), *c);
            }
        }
        Kind::Info(info) => {
            let t = info_text(st, info);
            let mono = matches!(info, Info::ConfigPath | Info::CachePath);
            let font = if mono { f_mono(12.5) } else { f_ui(13.5) };
            let shown = fit_left(&p, &t, font.clone(), cr.width());
            p.text(Pos2::new(cr.max.x, cr.center().y), Align2::RIGHT_CENTER, shown, font, pal::TEXT2);
            if resp.hovered() {
                resp.clone().on_hover_text(&t);
            }
        }
    }
    if let Some(v) = new {
        if v != cur {
            (it.set)(&mut st.settings, v);
            if it.id == "ui_scale" {
                // (The layout jumps: keep the focused row in view.)
                st.settings_ui.scroll_to_focus = true;
            }
            return true;
        }
    }
    false
}

/// Switch with an On / Off word, right-aligned in `r`.
fn toggle_switch(p: &egui::Painter, r: Rect, on: bool) {
    let sw = Rect::from_center_size(Pos2::new(r.max.x - 24.0, r.center().y), Vec2::new(44.0, 24.0));
    p.rect_filled(sw, 12.0, if on { pal::ACCENT } else { Color32::from_gray(62) });
    p.circle_filled(Pos2::new(if on { sw.max.x - 12.0 } else { sw.min.x + 12.0 }, sw.center().y), 9.0, Color32::WHITE);
    p.text(Pos2::new(sw.min.x - 10.0, r.center().y), Align2::RIGHT_CENTER, if on { "On" } else { "Off" }, f_semi(13.0), if on { pal::TEXT } else { pal::TEXT2 });
}

/// ‹ value › — returns −1 / +1 when an arrow is clicked.
fn stepper(ui: &mut Ui, r: Rect, id: impl std::hash::Hash + std::fmt::Debug + Copy, text: &str, prev: bool, next: bool) -> Option<i32> {
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    let bw = 34.0;
    let lr = Rect::from_min_size(r.min, Vec2::new(bw, r.height())).shrink(3.0);
    let rr = Rect::from_min_size(Pos2::new(r.max.x - bw, r.min.y), Vec2::new(bw, r.height())).shrink(3.0);
    let mut out = None;
    for (rect, dir, on, k) in [(lr, -1.0f32, prev, 0u8), (rr, 1.0, next, 1)] {
        let resp = ui.interact(rect, ui.id().with((id, k)), if on { Sense::click() } else { Sense::hover() });
        if resp.hovered() && on {
            p.rect_filled(rect, sp::R_CTRL - 2, pal::RAISED_HI);
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        let c = rect.center();
        let col = if on { pal::TEXT } else { pal::TEXT3 };
        let s = Stroke::new(1.8, col);
        p.line_segment([c + Vec2::new(-3.0 * dir, -5.5), c + Vec2::new(3.0 * dir, 0.0)], s);
        p.line_segment([c + Vec2::new(3.0 * dir, 0.0), c + Vec2::new(-3.0 * dir, 5.5)], s);
        if resp.clicked() && on {
            out = Some(dir as i32);
        }
    }
    let tw = r.width() - 2.0 * bw - 8.0;
    p.text(r.center(), Align2::CENTER_CENTER, fit(&p, text, f_semi(13.0), tw), f_semi(13.0), pal::TEXT);
    out
}

/// A design-system slider filling `r`; true when dragged.
fn slider(ui: &mut Ui, r: Rect, v: &mut f32, range: std::ops::RangeInclusive<f32>) -> bool {
    let t = Rect::from_center_size(r.center(), Vec2::new(r.width() - 8.0, 6.0));
    ui.painter().rect_filled(t, 3, Color32::from_rgb(9, 11, 15));
    let mut child = ui.new_child(UiBuilder::new().max_rect(r));
    child.spacing_mut().slider_width = r.width();
    child.put(r, egui::Slider::new(v, range).show_value(false)).changed()
}

// ---------------------------------------------------------------- actions & info

fn run_action(st: &mut UiState, act: Act) {
    match act {
        Act::OpenFailures => leave(st, Some(Action::Failures)),
        Act::Credits => leave(st, Some(Action::Goto(Screen::Credits))),
        Act::ClearTiles | Act::ClearTerrain => clear_cache(st, act),
    }
}

/// Where the downloaded data lives (relative to the working directory, as tiles.rs etc. use it).
fn cache_dir() -> std::path::PathBuf {
    std::env::current_dir().map(|d| d.join("cache")).unwrap_or_else(|_| "cache".into())
}

fn clear_cache(st: &mut UiState, act: Act) {
    if crate::platform::WEB || st.settings_ui.headless {
        return;
    }
    let dirs: &[&str] = match act {
        Act::ClearTerrain => &["terrain"],
        _ => &["tiles", "airports"],
    };
    let root = cache_dir();
    let dirs: Vec<std::path::PathBuf> = dirs.iter().map(|d| root.join(d)).collect();
    let size = st.settings_ui.cache_size.clone();
    crate::platform::spawn("cache-clear", move || {
        for d in dirs {
            let _ = std::fs::remove_dir_all(&d);
        }
        *size.lock().unwrap() = Some(dir_size(&root));
    });
    st.toasts.push(super::toast::Kind::Good, if act == Act::ClearTerrain { "Terrain cache cleared" } else { "Map tile cache cleared" }, "It fills again as you fly", 2.5);
}

/// Bytes under `d` (recursive).
pub fn dir_size(d: &std::path::Path) -> u64 {
    let Ok(rd) = std::fs::read_dir(d) else { return 0 };
    rd.flatten()
        .map(|e| match e.file_type() {
            Ok(t) if t.is_dir() => dir_size(&e.path()),
            Ok(_) => e.metadata().map(|m| m.len()).unwrap_or(0),
            Err(_) => 0,
        })
        .sum()
}

/// Remove the oldest cached tiles / terrain beyond `limit_mb` (start-up, native).
pub fn prune_cache(limit_mb: u32) {
    if limit_mb == 0 || crate::platform::WEB {
        return;
    }
    crate::platform::spawn("cache-prune", move || {
        let root = cache_dir();
        let mut files = Vec::new();
        fn walk(d: &std::path::Path, out: &mut Vec<(std::time::SystemTime, u64, std::path::PathBuf)>) {
            let Ok(rd) = std::fs::read_dir(d) else { return };
            for e in rd.flatten() {
                let Ok(m) = e.metadata() else { continue };
                if m.is_dir() {
                    walk(&e.path(), out);
                } else {
                    out.push((m.modified().unwrap_or(std::time::UNIX_EPOCH), m.len(), e.path()));
                }
            }
        }
        for d in ["tiles", "terrain"] {
            walk(&root.join(d), &mut files);
        }
        let mut total: u64 = files.iter().map(|f| f.1).sum();
        let limit = limit_mb as u64 * 1024 * 1024;
        if total <= limit {
            return;
        }
        files.sort();
        let mut removed = 0;
        for (_, len, path) in files {
            if total <= limit {
                break;
            }
            if std::fs::remove_file(&path).is_ok() {
                total -= len;
                removed += 1;
            }
        }
        eprintln!("cache: pruned {removed} files to stay under {limit_mb} MB");
    });
}

fn human_size(b: u64) -> String {
    let mb = b as f64 / (1024.0 * 1024.0);
    if mb >= 1024.0 { format!("{:.1} GB", mb / 1024.0) } else { format!("{mb:.0} MB") }
}

fn devices(st: &mut UiState) -> Vec<(String, String)> {
    let s = &mut st.settings_ui;
    if !s.devices_asked && !s.headless && !crate::platform::WEB {
        s.devices_asked = true;
        let out = s.devices.clone();
        crate::platform::spawn("audio-devices", move || {
            *out.lock().unwrap() = Some(crate::audio::output_devices());
        });
    }
    s.devices.lock().unwrap().clone().unwrap_or_default()
}

fn info_text(st: &mut UiState, i: Info) -> String {
    match i {
        Info::Version => format!("{} · {}", env!("CARGO_PKG_VERSION"), if crate::platform::WEB { "browser" } else { "native" }),
        Info::Gpu => {
            if st.gpu_name.is_empty() {
                "—".into()
            } else {
                st.gpu_name.clone()
            }
        }
        Info::Licence => "Free software · GNU GPL v2".into(),
        Info::ConfigPath => {
            if crate::platform::WEB {
                "Browser storage".into()
            } else {
                super::settings::path().display().to_string()
            }
        }
        Info::CachePath => cache_dir().display().to_string(),
        Info::Palette => String::new(),
        Info::Audio => {
            if st.audio_status.is_empty() {
                "—".into()
            } else {
                st.audio_status.clone()
            }
        }
        Info::CacheSize => {
            let s = &mut st.settings_ui;
            if !s.cache_asked && !s.headless {
                s.cache_asked = true;
                let out = s.cache_size.clone();
                crate::platform::spawn("cache-size", move || *out.lock().unwrap() = Some(dir_size(&cache_dir())));
            }
            match *s.cache_size.lock().unwrap() {
                Some(b) => human_size(b),
                None if s.headless => "—".into(),
                None => "Measuring…".into(),
            }
        }
    }
}

// ---------------------------------------------------------------- text helpers

/// `text` shortened with a trailing ellipsis to fit `max_w`.
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

/// `text` shortened from the left ("…/skyward/settings.json") to fit `max_w`.
fn fit_left(p: &egui::Painter, text: &str, font: egui::FontId, max_w: f32) -> String {
    let w = |t: &str| p.layout_no_wrap(t.to_string(), font.clone(), pal::TEXT).size().x;
    if w(text) <= max_w {
        return text.to_string();
    }
    let mut chars: Vec<char> = text.chars().collect();
    while !chars.is_empty() && w(&format!("…{}", chars.iter().collect::<String>())) > max_w {
        chars.remove(0);
    }
    format!("…{}", chars.iter().collect::<String>())
}
