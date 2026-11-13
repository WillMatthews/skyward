//! In-flight interface: the instrument set for the aircraft class, the
//! moving map, status / auto-flight bar, annunciators, autopilot and
//! weather panels, warnings, help and toasts.

use super::glass;
use super::hudsym;
use super::map::{self, Style, View};
use super::settings::{HudMode, MapStyle};
use super::{chip, f_digits_b, f_inst, f_inst_b, f_mono, f_semi, f_ui, pal, panel, sp, text_halo, UiState};
use crate::flight::{Category, KT};
use crate::sim::{CamMode, Sim};
use crate::world::World;
use egui::{Align2, Color32, Pos2, Rect, RichText, Stroke, Vec2};
use glam::vec2;

/// Map display modes (M cycles).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MapMode {
    #[default]
    Corner,
    Large,
    Off,
}

impl MapMode {
    pub fn next(self) -> MapMode {
        match self {
            MapMode::Corner => MapMode::Large,
            MapMode::Large => MapMode::Off,
            MapMode::Off => MapMode::Corner,
        }
    }
}

/// Map half-ranges (m) per zoom step (the `,` / `.` keys step `sim.map_zoom`).
pub const MAP_RANGES: [f32; 6] = [1200.0, 2500.0, 5000.0, 10_000.0, 20_000.0, 40_000.0];

pub struct MapUi {
    pub mode: MapMode,
    /// Smoothed half-range (m).
    pub range: f32,
    /// Pan offset from the aircraft (m, world) after dragging; None = following.
    pub pan: Option<glam::Vec2>,
}

impl Default for MapUi {
    fn default() -> Self {
        MapUi { mode: MapMode::Corner, range: MAP_RANGES[2], pan: None }
    }
}

pub fn map_style(set: MapStyle, sim: &Sim) -> Style {
    match set {
        MapStyle::Day => Style::Day,
        MapStyle::Night => Style::Night,
        MapStyle::Auto => {
            if sim.sun_dir().y < -0.07 {
                Style::Night
            } else {
                Style::Day
            }
        }
    }
}

fn is_glass(sim: &Sim) -> bool {
    sim.def().category == Category::Airliner
}

fn is_jet(sim: &Sim) -> bool {
    sim.def().category == Category::FastJet
}

/// Everything drawn while flying.
#[allow(clippy::too_many_arguments)]
pub fn draw(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim, world: &World, ppp: f32, fps: f32) {
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("flight")));
    let mode = st.settings.hud;
    // The bot's route, else the flight plan's (nav/).
    let route: Vec<(String, glam::Vec2)> = match &sim.bot {
        Some(b) => b.route(),
        None => sim.nav.plan.route_world(&crate::nav::state::WorldEnv(world)),
    };
    let mut bottom = screen.max.y;
    cockpit_hazards(&p, screen, sim);
    // ---- instruments
    if mode != HudMode::Off {
        let compact = mode == HudMode::Minimal;
        let rotor = sim.def().rotor.is_some();
        if sim.def().category == Category::Drone {
            // FPV quad: the on-screen display instead of instruments (src/rotor).
            crate::rotor::instruments::osd(&p, screen, sim);
            bottom = crate::rotor::instruments::status_strip(&p, screen, sim, screen.max.y - 64.0);
        } else if rotor && !compact {
            let qnh = world.wx.qnh_hpa;
            let r = super::gauges::draw_panel(&p, screen, sim, qnh, false);
            bottom = crate::rotor::instruments::status_strip(&p, screen, sim, r.min.y - 8.0);
        } else if rotor {
            bottom = crate::rotor::instruments::status_strip(&p, screen, sim, minimal_strip(&p, screen, sim, st) - 6.0);
        } else if is_glass(sim) {
            let h = if compact { 290.0 } else { 340.0 };
            let y0 = screen.max.y - h - 12.0;
            let pfd_r = Rect::from_min_size(Pos2::new(screen.center().x - h - h * 0.5 - 12.0, y0), Vec2::splat(h));
            let nd_r = Rect::from_min_size(Pos2::new(pfd_r.max.x + 10.0, y0), Vec2::splat(h));
            if compact {
                let pfd_c = Rect::from_min_size(Pos2::new(screen.center().x - h - 5.0, y0), Vec2::splat(h));
                glass::pfd(&p, pfd_c, sim, world);
                let nd_c = Rect::from_min_size(Pos2::new(screen.center().x + 5.0, y0), Vec2::splat(h));
                glass::nd(&p, &mut st.canvas, ppp, nd_c, sim, world, &st.map_tiles, &route);
            } else {
                glass::pfd(&p, pfd_r, sim, world);
                glass::nd(&p, &mut st.canvas, ppp, nd_r, sim, world, &st.map_tiles, &route);
                let ew = Rect::from_min_size(Pos2::new(nd_r.max.x + 10.0, y0), Vec2::new(h * 0.78, h));
                glass::ewd(&p, ew, sim);
            }
            bottom = y0;
        } else if is_jet(sim) {
            if sim.cam == CamMode::Cockpit {
                if let Some(v) = &sim.last_view {
                    let cam = hudsym::Cam::from_view(v.view, v.fov_y, screen);
                    // The combiner: ~30° × 26° around the boresight.
                    let ppd = cam.ppd();
                    let (_, rot) = sim.render_pose();
                    let c = cam.project(rot * glam::Vec3::Z).unwrap_or(screen.center());
                    let field = Rect::from_center_size(c + Vec2::new(0.0, ppd * 4.0), Vec2::new(ppd * 42.0, ppd * 36.0));
                    hudsym::draw(&p, &cam, field, sim, (field.height() / 520.0).max(0.7), pal::HUD);
                }
                bottom = annunciators(&p, screen, sim, screen.max.y - 12.0);
            } else {
                let w = if compact { 400.0 } else { 480.0 };
                let r = Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - w * 0.34 - 14.0), Vec2::new(w, w * 0.68));
                super::panel_with(&p, r, sp::R_PANEL, Color32::from_rgba_premultiplied(6, 12, 9, 200));
                let (_, rot) = sim.render_pose();
                let cam = hudsym::Cam::along_nose(rot, 26f32.to_radians(), r);
                hudsym::draw(&p, &cam, r.shrink(6.0), sim, (r.height() / 520.0).max(0.7), pal::HUD);
                bottom = annunciators(&p, screen, sim, r.min.y - 8.0);
            }
        } else if !compact {
            let qnh = world.wx.qnh_hpa;
            let r = super::gauges::draw_panel(&p, screen, sim, qnh, false);
            bottom = annunciators(&p, screen, sim, r.min.y - 8.0);
        } else {
            bottom = minimal_strip(&p, screen, sim, st);
        }
        if is_glass(sim) {
            bottom = annunciators(&p, screen, sim, bottom - 8.0);
        }
    }
    // ---- top-left status
    let top = status_card(&p, screen, sim, st, fps);
    // ---- weather panel (I) and AP panel
    let mut y = top + sp::S;
    if mode != HudMode::Off {
        y = weather_panel(ctx, st, sim, world, Pos2::new(sp::L, y));
    }
    let _ = y;
    // ---- top centre: auto flight / AP strip + toasts
    let mut ty = sp::M;
    if mode != HudMode::Off {
        ty = auto_strip(&p, screen, sim, ty);
    }
    st.toasts.draw(&p, screen, ty + 4.0);
    // ---- map
    if st.map.mode != MapMode::Off && mode != HudMode::Off {
        moving_map(ctx, &p, st, sim, world, ppp, screen, bottom);
    } else {
        p.text(Pos2::new(screen.max.x - 10.0, screen.max.y - 6.0), Align2::RIGHT_BOTTOM, crate::hud::ATTRIBUTION, f_ui(10.5), Color32::from_white_alpha(170));
    }
    if mode != HudMode::Off && !is_glass(sim) {
        ap_panel(ctx, st, sim, Pos2::new(screen.max.x - sp::L, bottom - sp::S));
    } else if is_glass(sim) && mode != HudMode::Off {
        ap_panel(ctx, st, sim, Pos2::new(screen.max.x - sp::L, bottom - sp::S));
    }
    // ---- warnings / banners
    warnings(&p, screen, sim);
    if sim.show_help {
        help(ctx, sim);
    }
    // ---- systems & failures panel (O / F2)
    super::failures::panel(ctx, st, sim);
    // ---- radios & flight plan (F3)
    super::radio::panel(ctx, st, sim, world);
}

/// Smoke in the cockpit (electrical / engine fire) and a cracked windscreen (systems/).
fn cockpit_hazards(p: &egui::Painter, screen: Rect, sim: &Sim) {
    let sys = &sim.ac.sys;
    let smoke = sys.cockpit_smoke();
    if smoke > 0.01 && sim.ac.crashed.is_none() {
        let inside = if sim.cam == CamMode::Cockpit { 1.0 } else { 0.35 };
        let a = (smoke * 0.72 * inside).clamp(0.0, 0.75);
        // Layered grey haze, denser at the top (smoke rises).
        let top = Color32::from_rgba_unmultiplied(150, 150, 146, (a * 255.0) as u8);
        let bot = Color32::from_rgba_unmultiplied(110, 110, 108, (a * 150.0) as u8);
        super::gradient_rect(p, screen, top, bot);
    }
    if sys.windscreen_cracked && sim.cam == CamMode::Cockpit {
        let c = Pos2::new(screen.center().x + screen.width() * 0.12, screen.min.y + screen.height() * 0.3);
        let st = Stroke::new(1.2, Color32::from_white_alpha(150));
        let mut seed = 7u32;
        let mut rnd = || {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            (seed >> 8) as f32 / (1u32 << 24) as f32
        };
        for k in 0..9 {
            let mut a = k as f32 / 9.0 * std::f32::consts::TAU + rnd() * 0.4;
            let mut q = c;
            for _ in 0..6 {
                let len = 18.0 + rnd() * 46.0;
                a += (rnd() - 0.5) * 0.6;
                let nq = q + Vec2::new(a.cos(), a.sin()) * len;
                p.line_segment([q, nq], st);
                q = nq;
            }
        }
        p.circle_stroke(c, 7.0, Stroke::new(1.5, Color32::from_white_alpha(170)));
    }
}

fn status_card(p: &egui::Painter, screen: Rect, sim: &Sim, st: &UiState, fps: f32) -> f32 {
    let _ = screen;
    let def = sim.def();
    let at = Pos2::new(sp::L, sp::M);
    let name = p.layout_no_wrap(def.name.to_string(), f_semi(15.0), pal::TEXT);
    let mut sub = sim.cam.name().to_string();
    if sim.warp > 1 {
        sub += &format!("  ·  {}×", sim.warp);
    }
    if sim.paused {
        sub += "  ·  Paused";
    }
    let subg = p.layout_no_wrap(sub, f_ui(12.0), pal::TEXT2);
    let s = &st.settings;
    let ias = s.speed.from_ms(sim.ac.airspeed());
    let alt = s.alt.from_m(sim.ac.pos.y);
    let data = format!("{:>4.0} {}   {:>6} {}   {:+.1} g", ias, s.speed.label(), super::thousands(alt), s.alt.label(), sim.ac.g_load());
    let dg = p.layout_no_wrap(data, f_mono(12.5), pal::TEXT);
    let w = name.size().x.max(subg.size().x).max(dg.size().x) + 28.0;
    let h = 64.0;
    let r = Rect::from_min_size(at, Vec2::new(w.max(230.0), h));
    panel(p, r, sp::R_PANEL);
    p.galley(r.min + Vec2::new(14.0, 9.0), name.clone(), pal::TEXT);
    p.galley(Pos2::new(r.min.x + 14.0 + name.size().x + 10.0, r.min.y + 12.0), subg, pal::TEXT2);
    p.galley(r.min + Vec2::new(14.0, 36.0), dg, pal::TEXT);
    if fps >= 1.0 {
        p.text(Pos2::new(r.max.x - 10.0, r.min.y + 9.0), Align2::RIGHT_TOP, format!("{fps:.0}"), f_mono(10.5), pal::TEXT3);
    }
    r.max.y
}

/// Auto-flight / autopilot strip at the top centre. Returns its bottom.
fn auto_strip(p: &egui::Painter, screen: Rect, sim: &Sim, y: f32) -> f32 {
    let mut y = y;
    if let Some(b) = &sim.bot {
        let head = p.layout_no_wrap("AUTO FLIGHT".into(), f_inst_b(12.0), pal::MAGENTA);
        let st = p.layout_no_wrap(b.status.clone(), f_semi(14.0), pal::TEXT);
        let hint = p.layout_no_wrap(if sim.watch { "Watching · any control input takes over".into() } else { "Any control input takes over".into() }, f_ui(11.5), pal::TEXT3);
        let w = (head.size().x + st.size().x + 40.0).max(hint.size().x + 28.0);
        let r = Rect::from_center_size(Pos2::new(screen.center().x, y + 27.0), Vec2::new(w, 54.0));
        panel(p, r, sp::R_PANEL);
        p.rect_filled(Rect::from_min_size(r.min + Vec2::new(12.0, 12.0), Vec2::new(3.0, 30.0)), 2, pal::MAGENTA);
        p.galley(r.min + Vec2::new(22.0, 10.0), head.clone(), pal::MAGENTA);
        p.galley(r.min + Vec2::new(30.0 + head.size().x, 8.0), st, pal::TEXT);
        p.galley(r.min + Vec2::new(22.0, 32.0), hint, pal::TEXT3);
        y = r.max.y + 6.0;
    } else if sim.ap.engaged && !is_glass(sim) {
        // Compact FMA for types without a PFD.
        let ap = &sim.ap;
        let mut items: Vec<(String, Color32)> = vec![("AP".into(), pal::GREEN)];
        items.push((ap.hdg.map(|h| format!("HDG {:03.0}", h.to_degrees().rem_euclid(360.0))).unwrap_or_else(|| "ROLL".into()), pal::GREEN));
        items.push(match (ap.alt, ap.vs) {
            (Some(a), _) => (format!("ALT {}", super::thousands(a * 3.280_84)), pal::GREEN),
            (None, Some(v)) => (format!("V/S {:+.0}", v * 3.280_84 * 60.0), pal::GREEN),
            _ => ("PITCH".into(), pal::GREEN),
        });
        if let Some(v) = ap.ias {
            items.push((format!("SPD {:.0}", v / KT), pal::CYAN));
        }
        let gs: Vec<_> = items.iter().map(|(s, c)| p.layout_no_wrap(s.clone(), f_inst_b(13.0), *c)).collect();
        let w: f32 = gs.iter().map(|g| g.size().x + 22.0).sum::<f32>() + 10.0;
        let r = Rect::from_center_size(Pos2::new(screen.center().x, y + 17.0), Vec2::new(w, 34.0));
        panel(p, r, sp::R_CTRL);
        let mut x = r.min.x + 16.0;
        for (i, g) in gs.into_iter().enumerate() {
            if i > 0 {
                p.line_segment([Pos2::new(x - 11.0, r.min.y + 8.0), Pos2::new(x - 11.0, r.max.y - 8.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
            }
            let gw = g.size().x;
            p.galley(Pos2::new(x, r.center().y - g.size().y * 0.5), g, pal::TEXT);
            x += gw + 22.0;
        }
        y = r.max.y + 6.0;
    } else if sim.ap_warn > 0.0 && (sim.time * 3.0).fract() < 0.6 {
        let r = chip(p, Pos2::new(screen.center().x, y), Align2::CENTER_TOP, "AUTOPILOT DISCONNECTED", f_inst_b(14.0), Color32::WHITE, Color32::from_rgb(190, 30, 30));
        y = r.max.y + 6.0;
    }
    y
}

/// Annunciator strip centred above `y` (bottom). Returns its top.
fn annunciators(p: &egui::Painter, screen: Rect, sim: &Sim, y: f32) -> f32 {
    let def = sim.def();
    let ac = &sim.ac;
    let blink = (sim.time * 3.0).fract() < 0.6;
    let mut items: Vec<(String, Color32, bool)> = Vec::new(); // text, colour, lit
    if def.has_engine {
        let t = sim.ctl.throttle * 100.0;
        let lab = if sim.ctl.reverse { format!("REV {t:.0}%") } else { format!("THR {t:.0}%") };
        items.push((lab, if sim.ctl.reverse { pal::AMBER } else { pal::TEXT }, true));
    }
    let flap = if def.flap_label == "Airbrakes" { format!("AIRBRAKE {}", sim.flap_label()) } else { format!("FLAPS {}", sim.flap_label()) };
    items.push((flap, pal::TEXT, sim.ctl.flaps > 0.01));
    if def.has_spoilers {
        let s = ac.surfaces().spoilers;
        items.push((if def.category == Category::FastJet { "SPD BRK".into() } else { "SPOILERS".into() }, pal::CYAN, s > 0.05));
    }
    items.push(("BRAKES".into(), pal::AMBER, sim.ctl.brakes > 0.5));
    if def.has_afterburner {
        items.push(("REHEAT".into(), pal::AMBER, ac.afterburner() > 0.02));
    }
    let trim = sim.ctl.trim;
    items.push((format!("TRIM {:+.0}", trim * 100.0), pal::TEXT, trim.abs() > 0.02));
    // Engine-failure practice (K); real failures come from the systems below.
    let failed = def.has_engine && (0..def.engine_count).any(|i| !ac.engine_running(i) && (sim.ctl.failed_engines & (1 << i) != 0 || !ac.sys.init));
    if failed {
        items.push(("ENG FAIL".into(), pal::RED, blink));
    }
    // Master warning / caution, then the warning and caution messages (systems/alerts.rs).
    if ac.crashed.is_none() && ac.sys.init {
        if ac.sys.master_warn {
            items.insert(0, ("MASTER WARN".into(), if blink { pal::RED } else { Color32::from_rgb(150, 40, 40) }, true));
        }
        if ac.sys.master_caut {
            items.insert(0, ("MASTER CAUT".into(), pal::AMBER, true));
        }
        for (t, lvl) in crate::systems::alerts::annunciator_labels(&ac.sys, def, ac, &sim.ctl).into_iter().take(4) {
            let warning = lvl == crate::systems::alerts::Level::Warning;
            items.push((t, if warning { pal::RED } else { pal::AMBER }, true));
        }
    }
    if ac.stall_warning() {
        items.push(("STALL".into(), pal::RED, blink));
    }
    // Structure: overstress / overspeed warnings, damage and lost parts (damage.rs).
    if ac.crashed.is_none() {
        for (t, warning) in ac.dmg.annunciations() {
            items.push((t, if warning { pal::RED } else { pal::AMBER }, !warning || blink));
        }
    }
    let font = f_inst_b(11.5);
    let gs: Vec<_> = items.iter().map(|(s, c, lit)| p.layout_no_wrap(s.clone(), font.clone(), if *lit { *c } else { pal::TEXT3 })).collect();
    let gear_w = if def.retractable_gear { 74.0 } else { 0.0 };
    let w: f32 = gs.iter().map(|g| g.size().x + 20.0).sum::<f32>() + gear_w + 12.0;
    let r = Rect::from_center_size(Pos2::new(screen.center().x, y - 15.0), Vec2::new(w, 30.0));
    panel(p, r, sp::R_CTRL);
    let mut x = r.min.x + 6.0;
    if def.retractable_gear {
        let g = ac.gear_pos();
        let col = if g > 0.995 { pal::GREEN } else if g > 0.005 { pal::RED } else { Color32::from_gray(60) };
        // The indicator lights need power (systems/).
        let col = if ac.sys.init && ac.sys.power == crate::systems::Power::Off { Color32::from_gray(60) } else { col };
        for k in 0..3 {
            let c = Pos2::new(x + 12.0 + k as f32 * 15.0, r.center().y);
            p.circle_filled(c, 5.0, col);
            if g > 0.995 {
                p.circle_filled(c, 8.0, pal::GREEN.gamma_multiply(0.15));
            }
        }
        x += gear_w;
    }
    for ((_, c, lit), g) in items.iter().zip(gs) {
        let gw = g.size().x;
        let cell = Rect::from_min_size(Pos2::new(x, r.min.y + 4.0), Vec2::new(gw + 14.0, 22.0));
        if *lit {
            p.rect_filled(cell, 4, c.gamma_multiply(0.16));
        }
        p.galley(Pos2::new(x + 7.0, r.center().y - g.size().y * 0.5), g, pal::TEXT);
        x += gw + 20.0;
    }
    r.min.y
}

/// Minimal mode for analogue types: one data strip.
fn minimal_strip(p: &egui::Painter, screen: Rect, sim: &Sim, st: &UiState) -> f32 {
    let s = &st.settings;
    let ac = &sim.ac;
    let cells = [
        ("SPD", format!("{:.0}", s.speed.from_ms(ac.airspeed())), s.speed.label()),
        ("ALT", super::thousands(s.alt.from_m(ac.pos.y)), s.alt.label()),
        ("V/S", format!("{:+.0}", ac.vertical_speed() * 3.280_84 * 60.0), "fpm"),
        ("HDG", format!("{:03.0}", ac.heading().to_degrees().rem_euclid(360.0)), "°"),
    ];
    let cw = 120.0;
    let r = Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - 40.0), Vec2::new(cw * 4.0 + 16.0, 56.0));
    panel(p, r, sp::R_PANEL);
    for (i, (k, v, u)) in cells.iter().enumerate() {
        let x = r.min.x + 8.0 + cw * i as f32;
        if i > 0 {
            p.line_segment([Pos2::new(x, r.min.y + 10.0), Pos2::new(x, r.max.y - 10.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
        }
        p.text(Pos2::new(x + 14.0, r.min.y + 9.0), Align2::LEFT_TOP, *k, f_inst(10.5), pal::TEXT3);
        let g = p.text(Pos2::new(x + 14.0, r.max.y - 8.0), Align2::LEFT_BOTTOM, v, f_digits_b(20.0), pal::TEXT);
        p.text(Pos2::new(g.max.x + 4.0, r.max.y - 11.0), Align2::LEFT_BOTTOM, *u, f_ui(11.0), pal::TEXT3);
    }
    annunciators(p, screen, sim, r.min.y - 8.0)
}

#[allow(clippy::too_many_arguments)]
fn moving_map(ctx: &egui::Context, p: &egui::Painter, st: &mut UiState, sim: &mut Sim, world: &World, ppp: f32, screen: Rect, bottom: f32) {
    let large = st.map.mode == MapMode::Large;
    let size = if large { Vec2::new((screen.width() * 0.46).min(900.0), (bottom - 90.0).min(screen.height() * 0.62)) } else { Vec2::splat(300.0) };
    let rect = Rect::from_min_size(Pos2::new(screen.max.x - size.x - sp::L, sp::M), size);
    // interaction: wheel zooms, drag pans, double-click recentres
    let resp = egui::Area::new(egui::Id::new("map_area")).fixed_pos(rect.min).order(egui::Order::Background).show(ctx, |ui| ui.allocate_rect(Rect::from_min_size(rect.min, rect.size()), egui::Sense::click_and_drag())).inner;
    let target = MAP_RANGES[sim.map_zoom.min(MAP_RANGES.len() - 1)] * if large { 1.6 } else { 1.0 };
    if resp.hovered() {
        let scroll = ctx.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.1 {
            st.map.range = (st.map.range * (1.0 - scroll * 0.0025)).clamp(400.0, 120_000.0);
            st.map_zoom_override = true;
        }
    }
    if !st.map_zoom_override {
        st.map.range += (target - st.map.range) * (st.dt * 8.0).min(1.0);
    }
    let style = map_style(st.settings.map_style, sim);
    let pos = vec2(sim.ac.pos.x, sim.ac.pos.z);
    let v = vec2(sim.ac.vel.x, sim.ac.vel.z);
    let trk = if v.length() > 5.0 { v.x.atan2(-v.y) } else { sim.ac.heading() };
    let up = if st.settings.north_up { 0.0 } else { trk };
    let s = rect.width().min(rect.height()) * 0.5 / st.map.range;
    let anchor = if st.settings.north_up || st.map.pan.is_some() { rect.center() } else { Pos2::new(rect.center().x, rect.min.y + rect.height() * 0.68) };
    let mut view = View { rect, anchor, centre: pos + st.map.pan.unwrap_or_default(), s, up };
    let edit = large && st.radio.edit_route;
    let env = crate::nav::state::WorldEnv(world);
    // Route editing (F3 › Edit on the map): drag a waypoint, click to add, right-click to delete.
    let legs_w = sim.nav.plan.legs.iter().map(|w| world.proj.to_world(w.ll.0, w.ll.1)).collect::<Vec<_>>();
    let hit = |q: Pos2, view: &View| legs_w.iter().enumerate().map(|(i, w)| (i, view.to_screen(*w).distance(q))).filter(|x| x.1 < 12.0).min_by(|a, b| a.1.total_cmp(&b.1)).map(|x| x.0);
    let pointer = ctx.input(|i| i.pointer.interact_pos());
    if edit && resp.drag_started() {
        st.radio.drag = pointer.and_then(|q| hit(q, &view));
    }
    if !resp.dragged() && !resp.drag_started() {
        st.radio.drag = None;
    }
    if let (true, Some(i), Some(q)) = (edit, st.radio.drag, pointer) {
        let ll = world.proj.to_latlon(view.to_world(q));
        sim.nav.plan.move_to(i, ll);
    } else if resp.dragged() {
        let d = resp.drag_delta();
        let w0 = view.to_world(anchor);
        let w1 = view.to_world(anchor - d);
        st.map.pan = Some(st.map.pan.unwrap_or_default() + (w1 - w0));
        view.centre = pos + st.map.pan.unwrap_or_default();
    }
    if resp.double_clicked() {
        st.map.pan = None;
        st.map_zoom_override = false;
    }
    let nb_hit = |q: Pos2| Rect::from_center_size(Pos2::new(rect.max.x - 20.0, rect.min.y + 20.0), Vec2::splat(28.0)).contains(q);
    if edit && resp.clicked() && !resp.double_clicked() {
        if let Some(q) = pointer.filter(|q| !nb_hit(*q) && hit(*q, &view).is_none()) {
            let w = view.to_world(q);
            let ll = world.proj.to_latlon(w);
            // Into the en-route leg it lengthens least (else before the approach).
            let n = sim.nav.plan.enroute_len();
            let start = sim.nav.plan.active.min(n);
            let mut best = (n, f32::MAX);
            let mut prev = pos;
            for (i, lw) in legs_w.iter().enumerate().take(n + 1).skip(start) {
                let add = prev.distance(w) + w.distance(*lw) - prev.distance(*lw);
                if add < best.1 {
                    best = (i, add);
                }
                prev = *lw;
            }
            let k = sim.nav.plan.legs.iter().filter(|x| x.kind == crate::nav::fms::WptKind::User).count() + 1;
            sim.nav.plan.insert(best.0, crate::nav::fms::Wpt::new(format!("WP{k:02}"), ll, crate::nav::fms::WptKind::User));
        }
    }
    if edit && resp.secondary_clicked() {
        if let Some(i) = pointer.and_then(|q| hit(q, &view)) {
            sim.nav.plan.remove(i);
        }
    }
    p.add(egui::epaint::Shadow { offset: [0, 6], blur: 22, spread: 0, color: Color32::from_black_alpha(100) }.as_shape(rect, sp::R_PANEL));
    let mp = p.with_clip_rect(rect);
    let reserved = [
        Rect::from_min_size(rect.min + Vec2::new(4.0, 4.0), Vec2::new(82.0, 66.0)),
        Rect::from_center_size(Pos2::new(rect.max.x - 20.0, rect.min.y + 20.0), Vec2::splat(34.0)),
        Rect::from_min_max(Pos2::new(rect.min.x, rect.max.y - 24.0), rect.max),
    ];
    let mut taken = map::draw_map(&mp, &mut st.canvas, if large { "map_large" } else { "map" }, ppp, &view, world, &st.map_tiles, style, sp::R_PANEL, &reserved);
    let dark = style != Style::Day;
    let ring_col = if dark { Color32::from_white_alpha(110) } else { Color32::from_black_alpha(90) };
    // focus runway (bot) extended centreline
    if let Some(b) = &sim.bot {
        if let Some(end) = b.focus_runway() {
            let a = view.to_screen(end.thr - end.dir * 9000.0);
            let t = view.to_screen(end.thr);
            for k in 0..12 {
                let f0 = k as f32 / 12.0;
                let f1 = f0 + 0.5 / 12.0;
                mp.line_segment([a + (t - a) * f0, a + (t - a) * f1], Stroke::new(1.5, pal::MAGENTA));
            }
        }
    }
    // Navaids around (VOR / DME / NDB, localizers) and the route: the bot's, or the flight plan.
    if sim.nav.equipped && sim.nav.show_navaids {
        let near = sim.nav.navaids_near(&env, view.centre, view.reach().min(150_000.0));
        let col = if dark { Color32::from_rgb(120, 200, 255) } else { Color32::from_rgb(30, 90, 170) };
        map::navaids(&mp, &view, world, &near, col, &mut taken);
    }
    match &sim.bot {
        Some(b) => map::route(&mp, &view, pos, &b.route(), pal::MAGENTA),
        None => {
            let all = sim.nav.plan.all_world(&env);
            let off = usize::from(sim.nav.plan.origin.is_some());
            map::flight_plan(&mp, &view, pos, &all, sim.nav.plan.active + off, pal::MAGENTA, if dark { Color32::from_white_alpha(90) } else { Color32::from_black_alpha(90) });
        }
    }
    if edit {
        mp.text(Pos2::new(rect.min.x + 10.0, rect.max.y - 34.0), Align2::LEFT_BOTTOM, "EDIT ROUTE · click add · drag move · right-click delete", f_inst_b(10.5), pal::MAGENTA);
    }
    map::range_rings(&mp, &view, st.map.range, true, ring_col);
    let me = view.to_screen(pos);
    map::trend(&mp, &view, pos, v, sim.ac.turn_rate(), if dark { pal::GREEN } else { Color32::from_rgb(20, 140, 70) });
    map::aircraft_symbol(&mp, me, view.screen_angle(sim.ac.heading()), 13.0, if dark { pal::AMBER } else { Color32::from_rgb(255, 196, 30) });
    map::wind_block(&mp, rect.min + Vec2::new(8.0, 8.0), &view, world, dark);
    // compass / orientation button
    let nb = Pos2::new(rect.max.x - 20.0, rect.min.y + 20.0);
    p.circle_filled(nb, 14.0, Color32::from_rgba_premultiplied(14, 17, 22, 200));
    let na = -view.up;
    let d = Vec2::new(na.sin(), -na.cos());
    p.add(egui::Shape::convex_polygon(vec![nb + d * 10.0, nb + d.rot90() * 4.0, nb - d.rot90() * 4.0], pal::RED, Stroke::NONE));
    p.text(nb - d * 4.0, Align2::CENTER_CENTER, "N", f_inst_b(10.0), pal::TEXT);
    let nb_r = Rect::from_center_size(nb, Vec2::splat(28.0));
    if resp.clicked() && ctx.input(|i| i.pointer.interact_pos()).is_some_and(|q| nb_r.contains(q)) {
        st.settings.north_up = !st.settings.north_up;
        st.settings_dirty = true;
    }
    // footer: scale + attribution
    let foot = Rect::from_min_max(Pos2::new(rect.min.x, rect.max.y - 22.0), rect.max);
    mp.rect_filled(foot, 0, if dark { Color32::from_black_alpha(140) } else { Color32::from_white_alpha(170) });
    let tc = if dark { pal::TEXT2 } else { Color32::from_rgb(60, 64, 72) };
    mp.text(Pos2::new(rect.min.x + 8.0, foot.center().y), Align2::LEFT_CENTER, format!("{} {}", if st.settings.north_up { "NORTH UP" } else { "TRACK UP" }, if st.map.pan.is_some() { "· double-click to recentre" } else { "" }), f_inst(10.0), tc);
    mp.text(Pos2::new(rect.max.x - 8.0, foot.center().y), Align2::RIGHT_CENTER, "© OpenMapTiles © OSM", f_ui(9.5), tc);
    p.rect_stroke(rect, sp::R_PANEL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    taken.clear();
}

/// Weather panel (I collapses / expands). Returns its bottom.
fn weather_panel(ctx: &egui::Context, st: &mut UiState, sim: &Sim, world: &World, at: Pos2) -> f32 {
    let w = &world.wx;
    let ac = &sim.ac;
    let open = st.show_wx;
    let mut bottom = at.y;
    egui::Area::new(egui::Id::new("wx_panel")).fixed_pos(at).order(egui::Order::Middle).show(ctx, |ui| {
        let frame = egui::Frame::new().fill(pal::PANEL).corner_radius(sp::R_PANEL).stroke(Stroke::new(1.0, pal::HAIRLINE_HI)).inner_margin(egui::Margin::symmetric(12, 10));
        let r = frame
            .show(ui, |ui| {
                ui.set_width(if open { 300.0 } else { 196.0 });
                let (rect, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 40.0), egui::Sense::click());
                if resp.clicked() {
                    st.show_wx = !st.show_wx;
                    st.settings_dirty = true;
                }
                let p = ui.painter();
                // wind relative to the nose
                let c = Pos2::new(rect.min.x + 20.0, rect.center().y);
                p.circle_stroke(c, 18.0, Stroke::new(1.0, pal::HAIRLINE_HI));
                p.line_segment([c + Vec2::new(0.0, -18.0), c + Vec2::new(0.0, -13.0)], Stroke::new(1.5, pal::TEXT3));
                let rel = (w.wind_from + 180.0).to_radians() - ac.heading();
                if w.wind_kt >= 1.0 {
                    let d = Vec2::new(rel.sin(), -rel.cos());
                    p.line_segment([c - d * 12.0, c + d * 12.0], Stroke::new(2.0, pal::CYAN));
                    p.add(egui::Shape::convex_polygon(vec![c + d * 14.0, c + d * 6.0 + d.rot90() * 5.0, c + d * 6.0 - d.rot90() * 5.0], pal::CYAN, Stroke::NONE));
                }
                let wind = if w.wind_kt < 1.0 { "Calm".to_string() } else if w.gust_kt > w.wind_kt + 3.0 { format!("{:03.0}° {:.0}G{:.0} kt", w.wind_from, w.wind_kt, w.gust_kt) } else { format!("{:03.0}° {:.0} kt", w.wind_from, w.wind_kt) };
                p.text(Pos2::new(c.x + 28.0, rect.min.y + 3.0), Align2::LEFT_TOP, wind, f_semi(14.0), pal::TEXT);
                let wv = crate::weather::wind_vec(w.wind_from, w.wind_kt);
                let hd = ac.heading();
                let head = -(wv.x * hd.sin() - wv.y * hd.cos()) / KT;
                let cross = (wv.x * hd.cos() + wv.y * hd.sin()) / KT;
                p.text(Pos2::new(c.x + 28.0, rect.min.y + 23.0), Align2::LEFT_TOP, format!("{} {:.0}   {} {:.0} kt", if head >= 0.0 { "Head" } else { "Tail" }, head.abs(), if cross >= 0.0 { "X →" } else { "X ←" }, cross.abs()), f_ui(11.5), pal::TEXT2);
                if open {
                    let gs = vec2(ac.vel.x, ac.vel.z).length() / KT;
                    p.text(Pos2::new(rect.max.x, rect.min.y + 3.0), Align2::RIGHT_TOP, format!("GS {gs:.0}"), f_mono(12.5), pal::TEXT);
                    p.text(Pos2::new(rect.max.x, rect.min.y + 23.0), Align2::RIGHT_TOP, format!("TAS {:.0}", ac.tas() / KT), f_mono(11.5), pal::TEXT2);
                }
                if !open {
                    return;
                }
                ui.add_space(4.0);
                ui.separator();
                let src = if world.live_weather { "Live" } else { "Set" };
                let station = w.metar.as_ref().map(|m| format!("{} · {:.0} km", m.station, w.station_km)).unwrap_or_else(|| w.source.replace("preset ", ""));
                ui.horizontal(|ui| {
                    ui.label(RichText::new(w.category()).font(f_inst_b(12.0)).color(cat_colour(w.category())));
                    ui.label(RichText::new(format!("{src} · {station}")).font(f_ui(12.0)).color(pal::TEXT2));
                });
                if let Some(m) = &w.metar {
                    ui.label(RichText::new(&m.raw).font(f_mono(11.0)).color(pal::TEXT3));
                }
                egui::Grid::new("wxgrid").num_columns(2).spacing([14.0, 3.0]).show(ui, |ui| {
                    let row = |ui: &mut egui::Ui, k: &str, v: String| {
                        ui.label(RichText::new(k).font(f_ui(12.0)).color(pal::TEXT3));
                        ui.label(RichText::new(v).font(f_ui(12.5)).color(pal::TEXT));
                        ui.end_row();
                    };
                    row(ui, "Visibility", if w.vis_m >= 9999.0 { "10 km +".into() } else { format!("{:.1} km", w.vis_m / 1000.0) });
                    row(ui, "Cloud", w.cloud_text());
                    row(ui, "Aloft", format!("{:03.0}° {:.0} kt", w.aloft_from, w.aloft_kt));
                    row(ui, "Temp / dew", format!("{:.0}° / {:.0}°C", w.temp_c, w.dew_c));
                    row(ui, "QNH", format!("{:.0} hPa", w.qnh_hpa));
                    let time = match &world.real_time {
                        Some(rt) => rt.text.clone(),
                        None => format!("{} local · day {}", super::hhmm(sim.hours), sim.day_of_year),
                    };
                    row(ui, "Time", time);
                });
            })
            .response;
        bottom = r.rect.max.y;
    });
    bottom
}

fn cat_colour(cat: &str) -> Color32 {
    match cat {
        "VFR" => pal::GREEN,
        "MVFR" => pal::ACCENT,
        "IFR" => pal::RED,
        "LIFR" => pal::MAGENTA,
        _ => pal::TEXT2,
    }
}

/// Autopilot panel (shown while the autopilot is engaged and the bot is off), anchored bottom-right at `at`.
fn ap_panel(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim, at: Pos2) {
    if !sim.ap.engaged || sim.bot.is_some() {
        return;
    }
    let has_engine = sim.def().has_engine;
    let vs1 = sim.def().vs1_kt;
    let radios = sim.nav.equipped;
    let loc_tuned = crate::nav::db::is_ils_freq(sim.nav.radios.nav[0].active);
    let has_plan = !sim.nav.plan.legs.is_empty();
    let has_appr = sim.nav.plan.approach.is_some();
    let def = sim.def().clone();
    let ac = sim.ac.clone();
    let (hdg_now, alt_now, ias_now) = (sim.ac.heading(), sim.ac.pos.y, sim.ac.airspeed());
    let open = st.ap_open;
    egui::Area::new(egui::Id::new("ap_panel")).pivot(Align2::RIGHT_BOTTOM).fixed_pos(at).order(egui::Order::Middle).show(ctx, |ui| {
        egui::Frame::new().fill(pal::PANEL).corner_radius(sp::R_PANEL).stroke(Stroke::new(1.0, pal::HAIRLINE_HI)).inner_margin(egui::Margin::symmetric(12, 10)).show(ui, |ui| {
            ui.set_width(236.0);
            ui.horizontal(|ui| {
                ui.label(RichText::new("AUTOPILOT").font(f_inst_b(12.0)).color(pal::GREEN));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.add(egui::Button::new(RichText::new(if open { "–" } else { "+" }).font(f_semi(13.0))).frame(false)).clicked() {
                        st.ap_open = !st.ap_open;
                    }
                });
            });
            if !open {
                return;
            }
            let ap = &mut sim.ap;
            let row = |ui: &mut egui::Ui, label: &str, value: String, col: Color32| -> (bool, bool) {
                let mut out = (false, false);
                ui.horizontal(|ui| {
                    ui.add_sized([38.0, 26.0], egui::Label::new(RichText::new(label).font(f_inst(11.5)).color(pal::TEXT3)));
                    out.0 = ui.add_sized([30.0, 26.0], egui::Button::new(RichText::new("−").font(f_semi(15.0)))).clicked();
                    ui.add_sized([96.0, 26.0], egui::Label::new(RichText::new(value).font(f_digits_b(15.0)).color(col)));
                    out.1 = ui.add_sized([30.0, 26.0], egui::Button::new(RichText::new("+").font(f_semi(15.0)))).clicked();
                });
                out
            };
            let h = ap.hdg.unwrap_or(hdg_now);
            let (m, p) = row(ui, "HDG", ap.hdg.map(|h| format!("{:03.0}°", h.to_degrees().rem_euclid(360.0))).unwrap_or("---".into()), pal::CYAN);
            if m || p {
                ap.hdg = Some(crate::bot::wrap_pi(h + if p { 5f32 } else { -5f32 }.to_radians()));
            }
            let a = ap.alt.unwrap_or(alt_now);
            let (m, p) = row(ui, "ALT", ap.alt.map(|a| format!("{:.0} ft", a * 3.280_84)).unwrap_or("---".into()), pal::CYAN);
            if m || p {
                ap.alt = Some(((a + if p { 30.48 } else { -30.48 }) / 30.48).round() * 30.48);
                ap.vs = None;
            }
            let (m, p) = row(ui, "V/S", ap.vs.map(|v| format!("{:+.0}", v / 0.00508)).unwrap_or("---".into()), pal::CYAN);
            if m || p {
                ap.vs = Some(ap.vs.unwrap_or(0.0) + if p { 500.0 } else { -500.0 } * 0.00508);
            }
            if has_engine {
                let v = ap.ias.unwrap_or(ias_now);
                let (m, p) = row(ui, "SPD", ap.ias.map(|v| format!("{:.0} kt", v / KT)).unwrap_or("---".into()), pal::CYAN);
                if m || p {
                    ap.ias = Some((v + if p { 5.0 } else { -5.0 } * KT).max(vs1 * KT));
                }
            }
            // Navigation modes (nav/): LOC / VOR, APPR, NAV (LNAV), VNAV; armed in cyan, active in green.
            if radios {
                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    use crate::autopilot::{Lat, Vert};
                    let chip = |ui: &mut egui::Ui, t: &str, active: bool, armed: bool| -> bool {
                        let col = if active { pal::GREEN } else if armed { pal::CYAN } else { pal::TEXT2 };
                        ui.add_sized([50.0, 24.0], egui::Button::new(RichText::new(t).font(f_inst_b(11.0)).color(col)).stroke(Stroke::new(1.0, if active || armed { col } else { pal::HAIRLINE_HI }))).clicked()
                    };
                    let locmode = if loc_tuned { Lat::Loc } else { Lat::Vor };
                    if chip(ui, if loc_tuned { "LOC" } else { "VOR" }, ap.lat == locmode, ap.arm_lat == Some(locmode)) {
                        ap.toggle_lat(locmode);
                    }
                    if chip(ui, "APPR", ap.appr && ap.vert == Vert::Gs, ap.appr) {
                        ap.toggle_appr();
                    }
                    if chip(ui, "NAV", ap.lat == Lat::Lnav, ap.arm_lat == Some(Lat::Lnav)) && has_plan {
                        ap.toggle_lat(Lat::Lnav);
                    }
                    if chip(ui, "VNAV", ap.vert == Vert::Vnav, ap.arm_vnav) && has_appr {
                        ap.toggle_vnav();
                    }
                });
                let f = ap.fma(&def, &ac);
                let line = [f[0].as_str(), f[1].as_str(), f[3].as_str()].into_iter().filter(|x| !x.is_empty()).collect::<Vec<_>>().join("  ");
                let arm = [f[2].as_str(), f[4].as_str()].into_iter().filter(|x| !x.is_empty()).collect::<Vec<_>>().join(" ");
                ui.horizontal(|ui| {
                    ui.label(RichText::new(line).font(f_inst_b(11.5)).color(pal::GREEN));
                    if !arm.is_empty() {
                        ui.label(RichText::new(arm).font(f_inst_b(11.5)).color(pal::CYAN));
                    }
                });
            }
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                if has_engine {
                    let mut at = ap.ias.is_some();
                    if ui.checkbox(&mut at, RichText::new("A/THR").font(f_inst(11.5))).changed() {
                        ap.ias = if at { Some(ias_now) } else { None };
                    }
                }
                if ui.button(RichText::new("ALT HOLD").font(f_inst(11.0))).clicked() {
                    ap.alt = Some(alt_now);
                    ap.vs = None;
                }
                if ui.button(RichText::new("OFF").font(f_inst(11.0))).clicked() {
                    ap.disengage();
                }
            });
        });
    });
}

fn warnings(p: &egui::Painter, screen: Rect, sim: &Sim) {
    let blink = (sim.time * 3.0).fract() < 0.6;
    let def = sim.def();
    let y = screen.center().y - screen.height() * 0.2;
    if let Some(reason) = &sim.ac.crashed {
        // Survivable accidents (belly landing, gear collapse) read differently from a crash.
        let survivable = sim.severity.is_some_and(|s| s.survivable());
        let (title, sub, fill, edge) = if survivable {
            ("Crash landing", "Aircraft damaged · everyone walks away", Color32::from_rgba_premultiplied(58, 40, 6, 230), pal::AMBER)
        } else {
            ("Crashed", "", Color32::from_rgba_premultiplied(60, 8, 10, 230), pal::RED)
        };
        let h = if sub.is_empty() { 108.0 } else { 126.0 };
        let r = Rect::from_center_size(Pos2::new(screen.center().x, y), Vec2::new(560.0, h));
        super::panel_with(p, r, sp::R_CARD, fill);
        p.rect_stroke(r, sp::R_CARD, Stroke::new(1.0, edge.gamma_multiply(0.6)), egui::StrokeKind::Inside);
        let top = r.min.y + 30.0;
        p.text(Pos2::new(r.center().x, top), Align2::CENTER_CENTER, title, super::f_display(28.0), pal::TEXT);
        p.text(Pos2::new(r.center().x, top + 30.0), Align2::CENTER_CENTER, cap(reason), f_ui(14.0), pal::TEXT2);
        if !sub.is_empty() {
            p.text(Pos2::new(r.center().x, top + 50.0), Align2::CENTER_CENTER, sub, f_ui(12.5), pal::TEXT3);
        }
        p.text(Pos2::new(r.center().x, r.max.y - 18.0), Align2::CENTER_CENTER, "R  Restart      Esc  Menu", f_inst(12.0), pal::TEXT3);
    } else if sim.ac.stall_warning() && blink {
        chip(p, Pos2::new(screen.center().x, y), Align2::CENTER_CENTER, "STALL", f_inst_b(28.0), Color32::WHITE, Color32::from_rgb(200, 30, 30));
    } else if let Some(fire) = crate::systems::alerts::alerts(&sim.ac.sys, def, &sim.ac, &sim.ctl).into_iter().find(|a| a.fire || a.smoke).filter(|_| blink) {
        chip(p, Pos2::new(screen.center().x, y), Align2::CENTER_CENTER, &fire.title, f_inst_b(24.0), Color32::WHITE, Color32::from_rgb(200, 30, 30));
    } else if def.retractable_gear && sim.ac.gear_pos() < 0.99 && !sim.ac.on_ground && sim.ac.pos.y - sim.ground_y < 150.0 && sim.ac.vertical_speed() < -1.0 && sim.ac.airspeed() < def.vref_kt * 1.3 * KT && blink {
        chip(p, Pos2::new(screen.center().x, y), Align2::CENTER_CENTER, "GEAR NOT DOWN", f_inst_b(22.0), Color32::WHITE, Color32::from_rgb(200, 30, 30));
    }
    if sim.paused {
        text_halo(p, Pos2::new(screen.center().x, screen.center().y - screen.height() * 0.3), Align2::CENTER_CENTER, "PAUSED", f_inst_b(26.0), pal::TEXT);
    }
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Controls reference rows (also used by the settings screen).
pub fn control_rows(def: &crate::flight::AircraftDef) -> Vec<(&'static str, &'static str, String)> {
    if def.rotor.is_some() {
        return crate::rotor::instruments::control_rows(def);
    }
    let mut v: Vec<(&str, &str, String)> = vec![
        ("Flying", "W S  /  ↑ ↓", "Pitch".into()),
        ("Flying", "A D  /  ← →", "Roll".into()),
        ("Flying", "Q E", "Rudder · nosewheel".into()),
        ("Flying", "[  ]", "Trim".into()),
    ];
    if def.has_engine {
        v.push(("Engine", "Shift Ctrl  /  = −", if def.has_afterburner { "Throttle (above 90 %: reheat)".into() } else { "Throttle".into() }));
        if def.has_reverser {
            v.push(("Engine", "T", "Reverse thrust (on the ground)".into()));
        }
        v.push(("Engine", "K", "Engine failure practice".into()));
    }
    v.push(("Systems", "F V", def.flap_label.into()));
    if def.retractable_gear {
        v.push(("Systems", "G", "Landing gear".into()));
    }
    if def.has_spoilers {
        v.push(("Systems", "/  \\", "Spoilers · speed brake".into()));
    }
    v.push(("Systems", "B  Space", "Wheel brakes".into()));
    v.push(("Systems", "L", "Landing · taxi lights".into()));
    v.extend([
        ("Autopilot", "Z", "Engage · disengage".into()),
        ("Autopilot", "Ins Del", "Heading −/+ 5°".into()),
        ("Autopilot", "Home End", "Altitude +/− 100 ft".into()),
        ("Autopilot", "9 0", "Speed −/+ 5 kt".into()),
        ("Autopilot", "1 2 3 4", "LOC · APPR · LNAV · VNAV".into()),
        ("Autopilot", "J", "Auto flight".into()),
        ("Radios", "5 · 6 7 · 8", "NAV 1 swap · course −/+ · ident".into()),
        ("Radios", "F3 · ;", "Radios & flight plan · ND mode".into()),
        ("View", "C", "Camera".into()),
        ("View", "Right drag · wheel", "Orbit · zoom".into()),
        ("View", "M", "Map: corner · large · off".into()),
        ("View", ",  .", "Map range".into()),
        ("View", "U", "Instruments: full · minimal · off".into()),
        ("View", "I", "Weather panel".into()),
        ("Sim", "P", "Pause".into()),
        ("Sim", "X", "Time warp".into()),
        ("Sim", "R", "Restart".into()),
        ("Sim", "N", "Mute sound".into()),
        ("Sim", "Esc  Tab", "Menu".into()),
        ("Sim", "H  F1", "This help".into()),
    ]);
    v
}

fn help(ctx: &egui::Context, sim: &Sim) {
    let rows = control_rows(sim.def());
    egui::Area::new(egui::Id::new("help")).anchor(Align2::CENTER_CENTER, [0.0, -20.0]).order(egui::Order::Foreground).show(ctx, |ui| {
        egui::Frame::new().fill(pal::PANEL).corner_radius(sp::R_CARD).stroke(Stroke::new(1.0, pal::HAIRLINE_HI)).inner_margin(egui::Margin::same(22)).shadow(egui::epaint::Shadow { offset: [0, 10], blur: 30, spread: 0, color: Color32::from_black_alpha(120) }).show(ui, |ui| {
            ui.label(RichText::new("Controls").font(super::f_display(22.0)));
            ui.add_space(8.0);
            ui.columns(2, |cols| {
                let half = rows.len().div_ceil(2);
                let mut group = "";
                for (i, (g, k, d)) in rows.iter().enumerate() {
                    let ui = &mut cols[if i < half { 0 } else { 1 }];
                    if *g != group {
                        group = g;
                        ui.add_space(6.0);
                        ui.label(RichText::new(g.to_uppercase()).font(f_inst_b(11.0)).color(pal::ACCENT));
                    }
                    ui.horizontal(|ui| {
                        ui.add_sized([150.0, 20.0], egui::Label::new(RichText::new(*k).font(f_mono(12.5)).color(pal::TEXT)));
                        ui.label(RichText::new(d).font(f_ui(13.0)).color(pal::TEXT2));
                    });
                }
            });
            ui.add_space(8.0);
            ui.label(RichText::new("H to close").font(f_ui(11.5)).color(pal::TEXT3));
        });
    });
}
