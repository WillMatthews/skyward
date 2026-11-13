//! Skyward's interface: one design system (palette, type, spacing, panels)
//! shared by the in-flight displays, the moving map and the menus.
//!
//! * `theme` values below; fonts are bundled (assets/fonts, all OFL).
//! * Layout works in egui points; the zoom factor maps a 1080-point-high
//!   layout onto any window (1080p → 4K, ultrawide), times the user's UI scale.
//! * Map-like vector layers render offscreen with 4× MSAA (`canvas`).

#![allow(deprecated)]

pub mod browser;
pub mod canvas;
pub mod comms;
pub mod controls;
pub mod failures;
pub mod flight;
pub mod gauges;
pub mod glass;
pub mod hudsym;
pub mod map;
pub mod mapdata;
pub mod menus;
pub mod mp;
pub mod missions;
pub mod overview;
pub mod radio;
pub mod scroll;
pub mod settings;
pub mod toast;
pub mod worldmap;

use egui::{epaint, Align2, Color32, CornerRadius, FontFamily, FontId, Pos2, Rect, Shape, Stroke, Vec2};

// ---------------------------------------------------------------- palette

/// Colours. Panels are dark and slightly translucent so the scene reads
/// through; text keeps ≥ 7:1 contrast on them whether the sky behind is
/// bright or it is night. Avionics colours follow cockpit convention:
/// green = engaged, cyan = selected, magenta = managed / route, amber =
/// caution, red = warning.
#[allow(dead_code)]
pub mod pal {
    use egui::Color32;
    pub const BG: Color32 = Color32::from_rgb(10, 13, 18);
    pub const PANEL: Color32 = Color32::from_rgba_premultiplied(13, 16, 22, 214);
    pub const PANEL_SOLID: Color32 = Color32::from_rgb(17, 21, 28);
    pub const RAISED: Color32 = Color32::from_rgb(27, 33, 43);
    pub const RAISED_HI: Color32 = Color32::from_rgb(36, 44, 57);
    pub const HAIRLINE: Color32 = Color32::from_rgba_premultiplied(22, 22, 24, 26);
    pub const HAIRLINE_HI: Color32 = Color32::from_rgba_premultiplied(48, 52, 60, 60);
    pub const TEXT: Color32 = Color32::from_rgb(236, 240, 245);
    pub const TEXT2: Color32 = Color32::from_rgb(164, 174, 188);
    pub const TEXT3: Color32 = Color32::from_rgb(112, 122, 138);
    pub const ACCENT: Color32 = Color32::from_rgb(92, 178, 255);
    pub const ACCENT_DEEP: Color32 = Color32::from_rgb(38, 110, 196);
    pub const ACCENT_BG: Color32 = Color32::from_rgba_premultiplied(14, 28, 40, 46);
    pub const GREEN: Color32 = Color32::from_rgb(64, 222, 134);
    pub const CYAN: Color32 = Color32::from_rgb(70, 214, 236);
    pub const MAGENTA: Color32 = Color32::from_rgb(236, 110, 236);
    pub const AMBER: Color32 = Color32::from_rgb(255, 180, 40);
    pub const RED: Color32 = Color32::from_rgb(255, 82, 82);
    pub const WHITE: Color32 = Color32::from_rgb(250, 250, 250);
    /// HUD (head-up display) green.
    pub const HUD: Color32 = Color32::from_rgb(110, 255, 140);
}

/// Spacing grid (points).
#[allow(dead_code)]
pub mod sp {
    pub const XS: f32 = 4.0;
    pub const S: f32 = 8.0;
    pub const M: f32 = 12.0;
    pub const L: f32 = 16.0;
    pub const XL: f32 = 24.0;
    pub const XXL: f32 = 32.0;
    /// Corner radii.
    pub const R_CTRL: u8 = 6;
    pub const R_PANEL: u8 = 10;
    pub const R_CARD: u8 = 14;
}

// ---------------------------------------------------------------- type

pub const F_SEMI: &str = "semibold";
pub const F_DISPLAY: &str = "display";
pub const F_INST: &str = "b612";
pub const F_INST_B: &str = "b612b";
pub const F_DIGITS: &str = "b612m";
pub const F_DIGITS_B: &str = "b612mb";

/// Body text (Inter).
pub fn f_ui(size: f32) -> FontId {
    FontId::proportional(size)
}
/// Labels, buttons, headings (Inter SemiBold).
pub fn f_semi(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_SEMI.into()))
}
/// Large titles (Inter Display Bold).
pub fn f_display(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_DISPLAY.into()))
}
/// Data, codes, METARs (JetBrains Mono).
pub fn f_mono(size: f32) -> FontId {
    FontId::monospace(size)
}
/// Instrument legends (B612, designed for cockpit displays).
pub fn f_inst(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_INST.into()))
}
pub fn f_inst_b(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_INST_B.into()))
}
/// Instrument digits (B612 Mono).
pub fn f_digits(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_DIGITS.into()))
}
pub fn f_digits_b(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name(F_DIGITS_B.into()))
}

/// Install the bundled fonts (egui's defaults stay as fallbacks for symbols).
pub fn install_fonts(ctx: &egui::Context) {
    let mut defs = egui::FontDefinitions::default();
    let add = |defs: &mut egui::FontDefinitions, name: &str, bytes: &'static [u8]| {
        defs.font_data.insert(name.to_string(), std::sync::Arc::new(egui::FontData::from_static(bytes)));
    };
    add(&mut defs, "inter", include_bytes!("../../assets/fonts/Inter-Regular.ttf"));
    add(&mut defs, "inter-semi", include_bytes!("../../assets/fonts/Inter-SemiBold.ttf"));
    add(&mut defs, "inter-display", include_bytes!("../../assets/fonts/InterDisplay-Bold.ttf"));
    add(&mut defs, "jbmono", include_bytes!("../../assets/fonts/JetBrainsMono-Regular.ttf"));
    add(&mut defs, "b612", include_bytes!("../../assets/fonts/B612-Regular.ttf"));
    add(&mut defs, "b612b", include_bytes!("../../assets/fonts/B612-Bold.ttf"));
    add(&mut defs, "b612m", include_bytes!("../../assets/fonts/B612Mono-Regular.ttf"));
    add(&mut defs, "b612mb", include_bytes!("../../assets/fonts/B612Mono-Bold.ttf"));
    let fallbacks: Vec<String> = defs.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    let mono_fb: Vec<String> = defs.families.get(&FontFamily::Monospace).cloned().unwrap_or_default();
    let chain = |first: &str, fb: &[String]| -> Vec<String> {
        let mut v = vec![first.to_string()];
        v.extend(fb.iter().cloned());
        v
    };
    defs.families.insert(FontFamily::Proportional, chain("inter", &fallbacks));
    defs.families.insert(FontFamily::Monospace, chain("jbmono", &mono_fb));
    for (fam, font) in [(F_SEMI, "inter-semi"), (F_DISPLAY, "inter-display"), (F_INST, "b612"), (F_INST_B, "b612b"), (F_DIGITS, "b612m"), (F_DIGITS_B, "b612mb")] {
        defs.families.insert(FontFamily::Name(fam.into()), chain(font, &fallbacks));
    }
    ctx.set_fonts(defs);
}

/// egui widget styling for the menus and panels.
pub fn install_style(ctx: &egui::Context) {
    ctx.set_visuals(egui::Visuals::dark());
    ctx.all_styles_mut(|s| {
        use egui::TextStyle as T;
        s.text_styles = [
            (T::Small, f_ui(11.5)),
            (T::Body, f_ui(14.0)),
            (T::Button, f_semi(14.0)),
            (T::Heading, f_display(22.0)),
            (T::Monospace, f_mono(13.0)),
        ]
        .into();
        s.spacing.item_spacing = Vec2::new(sp::S, sp::S);
        s.spacing.button_padding = Vec2::new(sp::M, 7.0);
        s.spacing.interact_size = Vec2::new(36.0, 30.0);
        s.spacing.slider_width = 220.0;
        s.spacing.combo_width = 160.0;
        s.spacing.text_edit_width = 280.0;
        s.spacing.window_margin = egui::Margin::same(sp::L as i8);
        let v = &mut s.visuals;
        v.dark_mode = true;
        v.override_text_color = Some(pal::TEXT);
        v.panel_fill = pal::PANEL_SOLID;
        v.window_fill = pal::PANEL;
        v.window_stroke = Stroke::new(1.0, pal::HAIRLINE_HI);
        v.window_corner_radius = CornerRadius::same(sp::R_PANEL);
        v.window_shadow = epaint::Shadow { offset: [0, 8], blur: 28, spread: 0, color: Color32::from_black_alpha(110) };
        v.popup_shadow = epaint::Shadow { offset: [0, 6], blur: 18, spread: 0, color: Color32::from_black_alpha(100) };
        v.menu_corner_radius = CornerRadius::same(sp::R_CTRL);
        v.extreme_bg_color = Color32::from_rgb(9, 11, 15);
        v.faint_bg_color = pal::RAISED;
        v.selection.bg_fill = pal::ACCENT_DEEP;
        v.selection.stroke = Stroke::new(1.0, pal::ACCENT);
        v.hyperlink_color = pal::ACCENT;
        v.slider_trailing_fill = true;
        v.handle_shape = egui::style::HandleShape::Circle;
        let r = CornerRadius::same(sp::R_CTRL);
        let w = &mut v.widgets;
        w.noninteractive.bg_fill = pal::RAISED;
        w.noninteractive.weak_bg_fill = pal::RAISED;
        w.noninteractive.bg_stroke = Stroke::new(1.0, pal::HAIRLINE_HI);
        w.noninteractive.fg_stroke = Stroke::new(1.0, pal::TEXT2);
        w.noninteractive.corner_radius = r;
        w.inactive.bg_fill = pal::RAISED;
        w.inactive.weak_bg_fill = pal::RAISED;
        w.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgba_premultiplied(40, 44, 52, 50));
        w.inactive.fg_stroke = Stroke::new(1.0, pal::TEXT);
        w.inactive.corner_radius = r;
        w.hovered.bg_fill = pal::RAISED_HI;
        w.hovered.weak_bg_fill = pal::RAISED_HI;
        w.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgba_premultiplied(50, 84, 116, 120));
        w.hovered.fg_stroke = Stroke::new(1.5, pal::WHITE);
        w.hovered.corner_radius = r;
        w.hovered.expansion = 0.0;
        w.active.bg_fill = pal::ACCENT_DEEP;
        w.active.weak_bg_fill = pal::ACCENT_DEEP;
        w.active.bg_stroke = Stroke::new(1.0, pal::ACCENT);
        w.active.fg_stroke = Stroke::new(1.5, pal::WHITE);
        w.active.corner_radius = r;
        w.active.expansion = 0.0;
        w.open = w.hovered.clone();
    });
}

/// Zoom factor that maps a 1080-point-high layout onto the window (clamped
/// so small windows stay usable), times the user's UI scale.
pub fn zoom_for(height_px: f32, native_ppp: f32, user_scale: f32) -> f32 {
    // egui's points-per-pixel = zoom × native; aim for ~1080 points of height.
    let native = native_ppp.max(0.5);
    ((height_px / native / 1080.0).clamp(0.72, 3.0) * user_scale.clamp(0.6, 2.0)).max(0.5)
}

// ---------------------------------------------------------------- painting helpers

/// Polar point: angle `a` clockwise from 12 o'clock.
pub fn polar(c: Pos2, r: f32, a: f32) -> Pos2 {
    Pos2::new(c.x + r * a.sin(), c.y - r * a.cos())
}

/// Arc as a polyline (feathered by egui).
pub fn arc(p: &egui::Painter, c: Pos2, r: f32, a0: f32, a1: f32, w: f32, col: Color32) {
    let n = ((a1 - a0).abs() * r.max(10.0) / 6.0).ceil().clamp(4.0, 256.0) as usize;
    let pts: Vec<Pos2> = (0..=n).map(|i| polar(c, r, a0 + (a1 - a0) * i as f32 / n as f32)).collect();
    p.add(Shape::line(pts, Stroke::new(w, col)));
}

/// Filled annular sector (for coloured dial arcs), as a mesh.
pub fn ring_sector(p: &egui::Painter, c: Pos2, r0: f32, r1: f32, a0: f32, a1: f32, col: Color32) {
    let n = ((a1 - a0).abs() * r1.max(10.0) / 5.0).ceil().clamp(2.0, 256.0) as usize;
    let mut m = epaint::Mesh::default();
    for i in 0..=n {
        let a = a0 + (a1 - a0) * i as f32 / n as f32;
        m.colored_vertex(polar(c, r0, a), col);
        m.colored_vertex(polar(c, r1, a), col);
        if i > 0 {
            let k = (i as u32) * 2;
            m.add_triangle(k - 2, k - 1, k);
            m.add_triangle(k - 1, k + 1, k);
        }
    }
    p.add(Shape::mesh(m));
}

/// Disc filled with a vertical gradient (top → bottom), as a mesh.
pub fn gradient_disc(p: &egui::Painter, c: Pos2, r: f32, top: Color32, bottom: Color32) {
    let n = 96;
    let mut m = epaint::Mesh::default();
    m.colored_vertex(c, lerp_col(top, bottom, 0.5));
    for i in 0..=n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        let q = polar(c, r, a);
        let t = ((q.y - (c.y - r)) / (2.0 * r)).clamp(0.0, 1.0);
        m.colored_vertex(q, lerp_col(top, bottom, t));
        if i > 0 {
            m.add_triangle(0, i, i + 1);
        }
    }
    p.add(Shape::mesh(m));
}

/// Annulus with a gradient (bezel), lit from the top-left.
pub fn bezel_ring(p: &egui::Painter, c: Pos2, r0: f32, r1: f32, light: Color32, dark: Color32) {
    let n = 120;
    let mut m = epaint::Mesh::default();
    for i in 0..=n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        // light from the upper left (a = -45°)
        let t = 0.5 + 0.5 * (a + std::f32::consts::FRAC_PI_4).cos();
        let col = lerp_col(dark, light, t);
        m.colored_vertex(polar(c, r0, a), col);
        m.colored_vertex(polar(c, r1, a), col);
        if i > 0 {
            let k = (i as u32) * 2;
            m.add_triangle(k - 2, k - 1, k);
            m.add_triangle(k - 1, k + 1, k);
        }
    }
    p.add(Shape::mesh(m));
}

/// Vertical gradient rectangle (no rounding).
pub fn gradient_rect(p: &egui::Painter, r: Rect, top: Color32, bottom: Color32) {
    let mut m = epaint::Mesh::default();
    m.colored_vertex(r.left_top(), top);
    m.colored_vertex(r.right_top(), top);
    m.colored_vertex(r.right_bottom(), bottom);
    m.colored_vertex(r.left_bottom(), bottom);
    m.add_triangle(0, 1, 2);
    m.add_triangle(0, 2, 3);
    p.add(Shape::mesh(m));
}

pub fn lerp_col(a: Color32, b: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let l = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t).round() as u8;
    Color32::from_rgba_premultiplied(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()), l(a.a(), b.a()))
}

/// Same colour at a given opacity (0..1), premultiplied.
pub fn alpha(c: Color32, a: f32) -> Color32 {
    c.gamma_multiply(a.clamp(0.0, 1.0))
}

/// Translucent panel: soft shadow, fill, hairline and a faint top highlight.
pub fn panel(p: &egui::Painter, r: Rect, radius: u8) {
    panel_with(p, r, radius, pal::PANEL);
}

pub fn panel_with(p: &egui::Painter, r: Rect, radius: u8, fill: Color32) {
    let cr = CornerRadius::same(radius);
    p.add(epaint::Shadow { offset: [0, 4], blur: 18, spread: 0, color: Color32::from_black_alpha(70) }.as_shape(r, cr));
    p.rect_filled(r, cr, fill);
    p.rect_stroke(r, cr, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    let hl = Rect::from_min_max(r.min + Vec2::new(radius as f32, 1.0), Pos2::new(r.max.x - radius as f32, r.min.y + 1.0));
    p.line_segment([hl.left_top(), hl.right_top()], Stroke::new(1.0, Color32::from_white_alpha(18)));
}

/// Text with a soft dark halo — legible over a bright sky.
pub fn text_halo(p: &egui::Painter, pos: Pos2, align: Align2, s: impl ToString, font: FontId, col: Color32) -> Rect {
    let s = s.to_string();
    let halo = Color32::from_black_alpha(((col.a() as f32) * 0.55) as u8);
    for o in [Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0), Vec2::new(0.0, 1.4)] {
        p.text(pos + o, align, &s, font.clone(), halo);
    }
    p.text(pos, align, s, font, col)
}

/// Rounded "chip" with text, returns its rect.
pub fn chip(p: &egui::Painter, pos: Pos2, align: Align2, s: &str, font: FontId, fg: Color32, bg: Color32) -> Rect {
    let g = p.layout_no_wrap(s.to_string(), font, fg);
    let size = g.size() + Vec2::new(14.0, 6.0);
    let r = align.anchor_size(pos, size);
    p.rect_filled(r, CornerRadius::same(sp::R_CTRL), bg);
    p.galley(r.min + Vec2::new(7.0, 3.0), g, fg);
    r
}

// ---------------------------------------------------------------- units

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpeedUnit {
    #[default]
    Knots,
    Kmh,
    Mph,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum AltUnit {
    #[default]
    Feet,
    Metres,
}

impl SpeedUnit {
    pub fn from_ms(self, v: f32) -> f32 {
        match self {
            SpeedUnit::Knots => v * 1.943_844,
            SpeedUnit::Kmh => v * 3.6,
            SpeedUnit::Mph => v * 2.236_936,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            SpeedUnit::Knots => "kt",
            SpeedUnit::Kmh => "km/h",
            SpeedUnit::Mph => "mph",
        }
    }
}

impl AltUnit {
    pub fn from_m(self, v: f32) -> f32 {
        match self {
            AltUnit::Feet => v * 3.280_84,
            AltUnit::Metres => v,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            AltUnit::Feet => "ft",
            AltUnit::Metres => "m",
        }
    }
}

/// "1,234" style thousands separator.
pub fn thousands(v: f32) -> String {
    let n = v.round() as i64;
    let s = n.abs().to_string();
    let mut out = String::new();
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(ch);
    }
    if n < 0 { format!("-{out}") } else { out }
}

/// Hours → "14:05".
pub fn hhmm(h: f32) -> String {
    let h = h.rem_euclid(24.0);
    let mut hh = h.floor() as u32;
    let mut mm = ((h - hh as f32) * 60.0).round() as u32;
    if mm == 60 {
        mm = 0;
        hh = (hh + 1) % 24;
    }
    format!("{hh:02}:{mm:02}")
}

/// Shortest signed angle difference (radians).
pub fn wrap_pi(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

// ---------------------------------------------------------------- app shell state

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Screen {
    Main,
    Aircraft,
    Departure,
    Conditions,
    Settings,
    Credits,
    Pause,
    Loading,
    Flying,
    /// Server browser (net/, ui/browser.rs).
    Multiplayer,
    /// Airline-emergency missions (ui/missions.rs): cards, briefing, debrief.
    Missions,
    Briefing,
    Debrief,
}

impl Screen {
    pub fn parse(s: &str) -> Option<Screen> {
        Some(match s {
            "main" => Screen::Main,
            "aircraft" => Screen::Aircraft,
            "airport" | "departure" => Screen::Departure,
            "conditions" => Screen::Conditions,
            "settings" => Screen::Settings,
            "credits" => Screen::Credits,
            "pause" => Screen::Pause,
            "loading" => Screen::Loading,
            "multiplayer" | "mp" | "servers" => Screen::Multiplayer,
            "missions" => Screen::Missions,
            "briefing" => Screen::Briefing,
            "debrief" => Screen::Debrief,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Goto(Screen),
    Back,
    Fly,
    FlyLast,
    Resume,
    Restart,
    /// Systems & failures panel (over the flight).
    Failures,
    /// Leave the multiplayer server (keep flying alone).
    LeaveServer,
    Quit,
    /// Fly the mission in `missions.pending` (missions/).
    StartMission,
    /// Back to the mission's last checkpoint.
    RetryCheckpoint,
}

/// What to fly: aircraft index, departure airport ident, start, auto flight.
#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub aircraft: usize,
    pub airport: String,
    pub spec: crate::startpos::Spec,
    pub watch: bool,
}

pub struct UiState {
    pub screen: Screen,
    /// Where Back / Done returns to from settings and credits.
    pub back_to: Screen,
    pub settings: settings::Settings,
    pub settings_dirty: bool,
    pub settings_tab: usize,
    pub plan: Plan,
    pub defs: Vec<crate::flight::AircraftDef>,
    pub picker: menus::Picker,
    pub toasts: toast::Toasts,
    pub canvas: canvas::Frame,
    pub map_tiles: map::Tiles,
    pub map: flight::MapUi,
    pub map_zoom_override: bool,
    pub db: Option<std::sync::Arc<crate::airports::Db>>,
    pub airport_status: Option<String>,
    pub actions: Vec<Action>,
    pub thumbs: std::collections::HashMap<&'static str, egui::TextureId>,
    pub show_wx: bool,
    pub ap_open: bool,
    /// Systems & failures panel (ui/failures.rs).
    pub fail: failures::FailUi,
    /// Radios & flight-plan panel (ui/radio.rs).
    pub radio: radio::RadioUi,
    /// Comms menu / subtitles (ui/comms.rs).
    pub comms: comms::CommsUi,
    pub audio_status: String,
    /// Game controllers, rebinding and the mouse yoke (input/).
    pub input: crate::input::InputSystem,
    /// Settings › Controls: sub-page and the expanded axis row.
    pub controls_page: usize,
    pub controls_axis: Option<(crate::input::AxisAction, usize)>,
    /// Aircraft screen: the family filter (None: all).
    pub aircraft_group: Option<menus::Group>,
    /// Multiplayer session (net/) and the server browser's model.
    pub mp: crate::net::Mp,
    pub browser: crate::net::browse::Browser,
    /// Missions: selection, records, the active one, the last result.
    pub missions: missions::MissionUi,
    /// Seconds since the last frame / since start, frame counter.
    pub dt: f32,
    pub time: f32,
    pub frame_no: u64,
}

impl UiState {
    pub fn new(settings: settings::Settings) -> UiState {
        UiState {
            screen: Screen::Main,
            back_to: Screen::Main,
            settings,
            settings_dirty: false,
            settings_tab: 0,
            plan: Plan { aircraft: 0, airport: "EGLC".into(), spec: crate::startpos::Spec::Runway(String::new()), watch: false },
            defs: crate::flight::catalogue(),
            picker: menus::Picker::default(),
            toasts: toast::Toasts::default(),
            canvas: canvas::Frame::default(),
            map_tiles: Default::default(),
            map: flight::MapUi::default(),
            map_zoom_override: false,
            db: None,
            airport_status: None,
            actions: Vec::new(),
            thumbs: Default::default(),
            show_wx: true,
            ap_open: true,
            fail: Default::default(),
            radio: Default::default(),
            comms: Default::default(),
            audio_status: String::new(),
            input: crate::input::InputSystem::new(),
            controls_page: 0,
            controls_axis: None,
            aircraft_group: None,
            mp: crate::net::Mp::new(),
            browser: Default::default(),
            missions: Default::default(),
            dt: 1.0 / 60.0,
            time: 0.0,
            frame_no: 0,
        }
    }

    pub fn go(&mut self, s: Screen) {
        if matches!(s, Screen::Settings | Screen::Credits) && !matches!(self.screen, Screen::Settings | Screen::Credits) {
            self.back_to = self.screen;
        }
        if s == Screen::Departure && self.screen != Screen::Departure {
            self.picker.selected = None;
        }
        if s != Screen::Multiplayer {
            self.browser.close();
        }
        self.screen = s;
    }
}

/// Draw the current menu screen (not the in-flight UI). `in_flight` dims less.
pub fn draw_menus(ctx: &egui::Context, st: &mut UiState, in_flight: bool, loading: Option<(&str, &str, Option<f32>)>) {
    let screen = ctx.content_rect();
    egui::Area::new(egui::Id::new("menu_root")).fixed_pos(screen.min).order(egui::Order::Middle).show(ctx, |ui| {
        ui.set_min_size(screen.size());
        match st.screen {
            Screen::Main => menus::main_menu(ui, st),
            Screen::Aircraft => menus::aircraft(ui, st),
            Screen::Departure => menus::departure(ui, st),
            Screen::Conditions => menus::conditions(ui, st),
            Screen::Settings => menus::settings(ui, st, in_flight),
            Screen::Credits => menus::credits(ui, st),
            Screen::Multiplayer => browser::draw(ui, st),
            Screen::Pause if st.missions.active.is_some() => missions::pause(ui, st),
            Screen::Missions => missions::missions_menu(ui, st),
            Screen::Briefing => missions::briefing(ui, st),
            Screen::Debrief => missions::debrief(ui, st),
            Screen::Pause => {
                let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].name;
                let title = format!("{def} · {}", st.plan.airport);
                let sub = st.plan.spec.label();
                menus::pause(ui, st, &title, &sub)
            }
            Screen::Loading => {
                let (t, d, f) = loading.unwrap_or(("Loading", "", None));
                menus::loading(ui, st, t, d, f)
            }
            Screen::Flying => {}
        }
    });
}
