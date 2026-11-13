//! Missions in the interface: the mission cards (main menu › Missions), the
//! briefing (situation, what has failed, the clock, tips), the in-flight
//! objective panel with the crew / ATC / cabin lines, the mission pause menu
//! and the debrief (outcome, score breakdown, touchdown numbers, medals).

use super::menus::{Btn, button};
use super::{Action, Screen, UiState, alpha, f_display, f_inst_b, f_mono, f_semi, f_ui, gradient_rect, pal, panel, panel_with, sp};
use crate::missions::defs::{self, Mission};
use crate::missions::records::Records;
use crate::missions::score::Medal;
use crate::missions::{Debrief, ObjShow};
use crate::sim::Sim;
use egui::{Align2, Color32, CornerRadius, Pos2, Rect, Sense, Shape, Stroke, Ui, Vec2, epaint};

/// Mission state of the interface.
#[derive(Default)]
pub struct MissionUi {
    /// Selected card.
    pub sel: usize,
    /// Best results (records.rs), loaded on first use.
    pub records: Option<Records>,
    /// The mission being flown (id), if any.
    pub active: Option<String>,
    /// Start this mission (set by the briefing's Fly; the app picks it up).
    pub pending: Option<String>,
    /// The last result, for the debrief: (mission id, debrief, new best, checkpoint name).
    pub last: Option<(String, Debrief, bool, Option<String>)>,
    /// Flown by the scripted pilot (`--mission-bot`).
    pub bot: bool,
}

impl MissionUi {
    pub fn records(&mut self) -> &mut Records {
        self.records.get_or_insert_with(Records::load)
    }
    pub fn selected(&self) -> &'static Mission {
        let all = defs::all();
        &all[self.sel.min(all.len().saturating_sub(1))]
    }
}

fn difficulty_name(d: u8) -> &'static str {
    match d {
        1 => "STRAIGHTFORWARD",
        2 => "MODERATE",
        3 => "DEMANDING",
        4 => "HARD",
        _ => "EXPERT",
    }
}

fn difficulty_col(d: u8) -> Color32 {
    match d {
        1 => pal::GREEN,
        2 => pal::CYAN,
        3 => pal::AMBER,
        4 => Color32::from_rgb(255, 128, 60),
        _ => pal::RED,
    }
}

pub fn medal_col(m: Medal) -> Color32 {
    match m {
        Medal::Gold => Color32::from_rgb(255, 204, 72),
        Medal::Silver => Color32::from_rgb(200, 210, 222),
        Medal::Bronze => Color32::from_rgb(214, 140, 84),
        Medal::None => pal::TEXT3,
    }
}

/// A medal: a filled disc with a ribbon notch.
fn medal(p: &egui::Painter, c: Pos2, r: f32, m: Medal) {
    let col = medal_col(m);
    let ribbon =
        [Pos2::new(c.x - r * 0.55, c.y - r * 0.6), Pos2::new(c.x - r * 0.15, c.y - r * 2.0), Pos2::new(c.x + r * 0.15, c.y - r * 2.0), Pos2::new(c.x + r * 0.55, c.y - r * 0.6)];
    p.add(Shape::convex_polygon(ribbon.to_vec(), alpha(pal::ACCENT_DEEP, 0.9), Stroke::NONE));
    p.circle_filled(c, r, col);
    p.circle_stroke(c, r * 0.72, Stroke::new(1.2, alpha(Color32::BLACK, 0.25)));
}

/// Difficulty pips.
fn pips(p: &egui::Painter, at: Pos2, d: u8) {
    for i in 0..5u8 {
        let c = Pos2::new(at.x + i as f32 * 11.0 + 4.0, at.y);
        if i < d {
            p.circle_filled(c, 3.6, difficulty_col(d));
        } else {
            p.circle_stroke(c, 3.2, Stroke::new(1.2, alpha(pal::TEXT3, 0.8)));
        }
    }
}

fn wrap_text(p: &egui::Painter, at: Pos2, text: &str, font: egui::FontId, col: Color32, w: f32) -> f32 {
    let g = p.layout(text.to_string(), font, col, w);
    let h = g.size().y;
    p.galley(at, g, col);
    h
}

fn aircraft_name(m: &Mission, st: &UiState) -> String {
    st.defs.iter().find(|d| crate::flight::find(&m.aircraft).is_some_and(|f| f.id == d.id)).map(|d| d.name.to_string()).unwrap_or_else(|| m.aircraft.clone())
}

fn thumb(p: &egui::Painter, st: &UiState, m: &Mission, r: Rect, radius: u8) {
    let id = crate::flight::find(&m.aircraft).map(|d| d.id).unwrap_or("");
    match st.thumbs.get(id) {
        Some(t) => {
            let a = r.width() / r.height() / 1.786;
            let uv = if a >= 1.0 {
                Rect::from_min_max(Pos2::new(0.0, 0.5 - 0.5 / a), Pos2::new(1.0, 0.5 + 0.5 / a))
            } else {
                Rect::from_min_max(Pos2::new(0.5 - 0.5 * a, 0.0), Pos2::new(0.5 + 0.5 * a, 1.0))
            };
            p.add(Shape::Rect(epaint::RectShape::filled(r, radius, Color32::WHITE).with_texture(*t, uv)));
        }
        None => gradient_rect(p, r, Color32::from_rgb(40, 62, 92), Color32::from_rgb(110, 140, 176)),
    }
}

// ---------------------------------------------------------------- mission cards

pub fn missions_menu(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    super::menus::backdrop_pub(&p, screen, 1.0);
    let margin = (screen.width() * 0.05).clamp(24.0, 96.0);
    let content = Rect::from_min_max(Pos2::new(screen.min.x + margin, screen.min.y + 36.0), Pos2::new(screen.max.x - margin, screen.max.y - 36.0));
    p.text(content.min + Vec2::new(0.0, 6.0), Align2::LEFT_TOP, "MISSIONS", f_inst_b(12.0), pal::ACCENT);
    p.text(content.min + Vec2::new(0.0, 26.0), Align2::LEFT_TOP, "Airline emergencies", f_display(30.0), pal::TEXT);
    p.text(
        content.min + Vec2::new(0.0, 66.0),
        Align2::LEFT_TOP,
        "Fictional flights inspired by real accidents and incidents. Something is about to go wrong — bring everyone home.",
        f_ui(14.0),
        pal::TEXT2,
    );
    // Progress: completed / medals.
    let all = defs::all();
    let rec = st.missions.records().clone();
    let done = all.iter().filter(|m| rec.get(&m.id).is_some_and(|b| b.completed)).count();
    let golds = all.iter().filter(|m| rec.get(&m.id).is_some_and(|b| b.medal == Medal::Gold)).count();
    let prog = format!("{done} / {} recovered  ·  {golds} gold", all.len());
    p.text(Pos2::new(content.max.x, content.min.y + 34.0), Align2::RIGHT_TOP, prog, f_semi(14.0), pal::TEXT2);
    let top = content.min.y + 104.0;
    let bottom = content.max.y - 70.0;
    let grid_r = Rect::from_min_max(Pos2::new(content.min.x, top), Pos2::new(content.max.x, bottom));
    let gap = 16.0;
    let min_w = 300.0;
    let gw = grid_r.width() - super::scroll::GUTTER;
    let cols = (((gw + gap) / (min_w + gap)).floor() as usize).max(1);
    let cw = (gw - gap * (cols as f32 - 1.0)) / cols as f32;
    let ch = cw * 0.42 + 150.0;
    let nav = super::scroll::nav(ui, &st.input, st.frame_no);
    let n = all.len() as i32;
    let mut sel = st.missions.sel.min(all.len() - 1) as i32;
    let before = sel;
    if nav.dx != 0 {
        sel = (sel + nav.dx).rem_euclid(n);
    }
    if nav.dy != 0 {
        sel = (sel + nav.dy * cols as i32).clamp(0, n - 1);
    }
    let moved = sel != before;
    st.missions.sel = sel as usize;
    let mut open = false;
    let fade = Color32::from_rgba_premultiplied(5, 7, 10, 250);
    let records = rec;
    super::scroll::area(ui, grid_r, "mission_grid", fade, |ui| {
        let rows = all.len().div_ceil(cols);
        let total = Vec2::new(gw, rows as f32 * (ch + gap) - gap + 4.0);
        let (area, _) = ui.allocate_exact_size(total, Sense::hover());
        for (k, m) in all.iter().enumerate() {
            let (row, col) = (k / cols, k % cols);
            let r = Rect::from_min_size(area.min + Vec2::new(col as f32 * (cw + gap), row as f32 * (ch + gap)), Vec2::new(cw, ch));
            let resp = ui.interact(r, ui.id().with(("mission", k)), Sense::click());
            let p = ui.painter();
            let is_sel = st.missions.sel == k;
            panel_with(p, r, sp::R_CARD, if resp.hovered() { pal::RAISED } else { pal::PANEL_SOLID });
            let img = Rect::from_min_size(r.min + Vec2::splat(6.0), Vec2::new(cw - 12.0, cw * 0.42));
            thumb(p, st, m, img, 10);
            // Darken the image towards its bottom for the chips.
            gradient_rect(p, Rect::from_min_max(Pos2::new(img.min.x, img.max.y - 40.0), img.max), Color32::TRANSPARENT, Color32::from_black_alpha(150));
            super::chip(
                p,
                img.min + Vec2::new(10.0, 10.0),
                Align2::LEFT_TOP,
                difficulty_name(m.difficulty),
                f_inst_b(10.5),
                Color32::from_rgb(8, 10, 14),
                difficulty_col(m.difficulty),
            );
            if let Some(b) = records.get(&m.id) {
                if b.medal != Medal::None {
                    medal(p, Pos2::new(img.max.x - 20.0, img.min.y + 32.0), 9.0, b.medal);
                } else if b.attempts > 0 {
                    super::chip(p, Pos2::new(img.max.x - 10.0, img.min.y + 10.0), Align2::RIGHT_TOP, "NOT YET", f_inst_b(10.0), pal::TEXT2, Color32::from_black_alpha(160));
                }
            }
            p.text(
                Pos2::new(img.min.x + 10.0, img.max.y - 10.0),
                Align2::LEFT_BOTTOM,
                format!("{} · {} {}", aircraft_name(m, st), m.target.airport, m.target.runway),
                f_semi(12.5),
                pal::WHITE,
            );
            let x = r.min.x + 18.0;
            let mut y = img.max.y + 14.0;
            p.text(Pos2::new(x, y), Align2::LEFT_TOP, &m.title, f_semi(18.0), pal::TEXT);
            y += 28.0;
            wrap_text(p, Pos2::new(x, y), &m.tagline, f_ui(13.5), pal::TEXT2, cw - 36.0);
            let fy = r.max.y - 26.0;
            pips(p, Pos2::new(x, fy), m.difficulty);
            p.text(Pos2::new(x + 62.0, fy), Align2::LEFT_CENTER, format!("≈ {:.0} min", m.minutes), f_ui(12.5), pal::TEXT3);
            if let Some(b) = records.get(&m.id).filter(|b| b.attempts > 0) {
                p.text(
                    Pos2::new(r.max.x - 18.0, fy),
                    Align2::RIGHT_CENTER,
                    format!("Best {:.0} · {}", b.score, b.grade),
                    f_mono(12.0),
                    if b.completed { pal::TEXT2 } else { pal::TEXT3 },
                );
            }
            if is_sel {
                p.rect_stroke(r, sp::R_CARD, Stroke::new(2.0, pal::ACCENT), egui::StrokeKind::Inside);
                if moved {
                    ui.scroll_to_rect(r.expand(12.0), None);
                }
            }
            if resp.clicked() {
                st.missions.sel = k;
            }
            if resp.double_clicked() {
                open = true;
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }
        }
    });
    let fy = content.max.y - 52.0;
    if button(ui, Rect::from_min_size(Pos2::new(content.min.x, fy), Vec2::new(140.0, 48.0)), "mback", "Back", Btn::Secondary, true).clicked() || nav.back {
        st.actions.push(Action::Goto(Screen::Main));
    }
    let next_r = Rect::from_min_size(Pos2::new(content.max.x - 220.0, fy), Vec2::new(220.0, 48.0));
    if button(ui, next_r, "mbrief", "Briefing  ›", Btn::Primary, true).clicked() || nav.ok || open {
        st.actions.push(Action::Goto(Screen::Briefing));
    }
    let m = st.missions.selected();
    p.text(Pos2::new(next_r.min.x - 20.0, fy + 15.0), Align2::RIGHT_CENTER, &m.title, f_semi(15.0), pal::TEXT);
    p.text(
        Pos2::new(next_r.min.x - 20.0, fy + 35.0),
        Align2::RIGHT_CENTER,
        format!("{} · {}", aircraft_name(m, st), difficulty_name(m.difficulty).to_lowercase()),
        f_ui(12.5),
        pal::TEXT2,
    );
}

// ---------------------------------------------------------------- briefing

fn bullet_list(p: &egui::Painter, x: f32, mut y: f32, w: f32, items: &[String], dot: Color32) -> f32 {
    for t in items {
        p.circle_filled(Pos2::new(x + 4.0, y + 9.0), 3.0, dot);
        y += wrap_text(p, Pos2::new(x + 18.0, y), t, f_ui(14.5), pal::TEXT, w - 18.0) + 8.0;
    }
    y
}

fn label(p: &egui::Painter, at: Pos2, s: &str, col: Color32) {
    p.text(at, Align2::LEFT_TOP, s, f_inst_b(11.5), col);
}

pub fn clock(secs: f32) -> String {
    let s = secs.max(0.0).round() as i32;
    format!("{}:{:02}", s / 60, s % 60)
}

pub fn briefing(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    super::menus::backdrop_pub(&p, screen, 1.0);
    let m = st.missions.selected();
    let margin = (screen.width() * 0.05).clamp(24.0, 96.0);
    let content = Rect::from_min_max(Pos2::new(screen.min.x + margin, screen.min.y + 36.0), Pos2::new(screen.max.x - margin, screen.max.y - 36.0));
    let wide = content.width() >= 1000.0;
    let side_w = if wide { (content.width() * 0.32).clamp(340.0, 480.0) } else { 0.0 };
    let main_r = Rect::from_min_max(content.min, Pos2::new(content.max.x - if wide { side_w + 32.0 } else { 0.0 }, content.max.y - 70.0));
    let rec = st.missions.records().get(&m.id).cloned();
    let (name, st_ro) = (aircraft_name(m, st), &*st);
    super::scroll::page(ui, main_r, "briefing", Color32::from_rgba_premultiplied(5, 7, 10, 250), |ui, o, w| {
        let p = ui.painter().clone();
        let w = w - super::scroll::GUTTER;
        let x = o.x;
        let mut y = o.y + 6.0;
        label(&p, Pos2::new(x, y), "MISSION BRIEFING", pal::ACCENT);
        super::chip(&p, Pos2::new(x + 150.0, y - 3.0), Align2::LEFT_TOP, difficulty_name(m.difficulty), f_inst_b(10.5), Color32::from_rgb(8, 10, 14), difficulty_col(m.difficulty));
        y += 22.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, &m.title, f_display(40.0), pal::TEXT);
        y += 54.0;
        y += wrap_text(&p, Pos2::new(x, y), &m.tagline, f_ui(17.0), pal::TEXT2, w) + 22.0;
        label(&p, Pos2::new(x, y), "THE SITUATION", pal::TEXT3);
        y += 22.0;
        y += wrap_text(&p, Pos2::new(x, y), &m.situation, f_ui(15.5), pal::TEXT, w.min(760.0)) + 24.0;
        // What has failed: a warning-coloured block, like an ECAM page.
        let fr_top = y;
        let fh = 34.0 + m.failures.iter().map(|f| p.layout(f.clone(), f_ui(14.5), pal::TEXT, w.min(760.0) - 58.0).size().y + 8.0).sum::<f32>() + 10.0;
        let fr = Rect::from_min_size(Pos2::new(x, fr_top), Vec2::new(w.min(760.0), fh));
        p.rect_filled(fr, sp::R_PANEL, Color32::from_rgba_premultiplied(40, 10, 10, 140));
        p.rect_stroke(fr, sp::R_PANEL, Stroke::new(1.0, alpha(pal::RED, 0.45)), egui::StrokeKind::Inside);
        p.rect_filled(Rect::from_min_size(fr.min, Vec2::new(4.0, fr.height())), CornerRadius { nw: sp::R_PANEL, sw: sp::R_PANEL, ne: 0, se: 0 }, pal::RED);
        label(&p, Pos2::new(x + 20.0, fr_top + 12.0), "WHAT'S FAILED", pal::RED);
        bullet_list(&p, x + 20.0, fr_top + 34.0, w.min(760.0) - 40.0, &m.failures, pal::AMBER);
        y = fr.max.y + 24.0;
        // The clock.
        label(&p, Pos2::new(x, y), "THE CLOCK", pal::TEXT3);
        y += 22.0;
        let clock_txt = match m.time_limit_s {
            Some(l) => format!("{} {}", if m.clock_label.is_empty() { "Time limit".to_string() } else { m.clock_label.clone() }, clock(l)),
            None => format!("No hard limit · par {}", clock(m.par_s)),
        };
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, clock_txt, f_semi(18.0), if m.time_limit_s.is_some() { pal::AMBER } else { pal::TEXT });
        y += 34.0;
        label(&p, Pos2::new(x, y), "TO WIN", pal::TEXT3);
        y += 22.0;
        let wins: Vec<String> = m.objectives.iter().filter(|o| o.required).map(|o| o.text.clone()).collect();
        y = bullet_list(&p, x, y, w.min(760.0), &wins, pal::GREEN) + 8.0;
        let extra: Vec<String> = m.objectives.iter().filter(|o| !o.required).map(|o| format!("{} (+{:.0})", o.text, o.points)).collect();
        if !extra.is_empty() {
            label(&p, Pos2::new(x, y), "FOR THE SCORE", pal::TEXT3);
            y += 22.0;
            y = bullet_list(&p, x, y, w.min(760.0), &extra, pal::CYAN) + 8.0;
        }
        label(&p, Pos2::new(x, y), "CAPTAIN'S NOTES", pal::TEXT3);
        y += 22.0;
        y = bullet_list(&p, x, y, w.min(760.0), &m.tips, pal::ACCENT) + 14.0;
        y += wrap_text(&p, Pos2::new(x, y), &m.history, f_ui(12.5), pal::TEXT3, w.min(760.0)) + 16.0;
        let _ = st_ro;
        y - o.y
    });
    // ---- side card: aircraft, airport, conditions, best
    if wide {
        // (Sized to its content: image, name, five rows, the METAR, the best result.)
        let img_h = (side_w - 16.0) * 0.56;
        let metar_h = if m.metar.is_empty() { 0.0 } else { 36.0 + p.layout(m.metar.clone(), f_mono(12.0), pal::TEXT2, side_w - 44.0).size().y };
        let h = 8.0 + img_h + 18.0 + 34.0 + 5.0 * 22.0 + metar_h + 6.0 + 22.0 + 44.0 + 16.0;
        let sr = Rect::from_min_size(Pos2::new(content.max.x - side_w, content.min.y + 30.0), Vec2::new(side_w, h.min(content.height() - 100.0)));
        panel(&p, sr, sp::R_CARD);
        let img = Rect::from_min_size(sr.min + Vec2::splat(8.0), Vec2::new(sr.width() - 16.0, (sr.width() - 16.0) * 0.56));
        thumb(&p, st, m, img, 10);
        let x = sr.min.x + 22.0;
        let mut y = img.max.y + 18.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, &name, f_semi(19.0), pal::TEXT);
        y += 34.0;
        let rows: Vec<(&str, String)> = vec![
            ("Airport", format!("{} · runway {}", m.target.airport, m.target.runway)),
            ("Local time", format!("{} · {}", super::hhmm(m.hours), day_label(m.day))),
            ("Aboard", format!("{} passengers and crew", m.pax)),
            ("Fuel", format!("{:.0} %", m.fuel_frac * 100.0)),
            ("Difficulty", format!("{} / 5", m.difficulty)),
        ];
        for (k, v) in rows {
            p.text(Pos2::new(x, y), Align2::LEFT_TOP, k, f_ui(13.0), pal::TEXT3);
            p.text(Pos2::new(x + 98.0, y), Align2::LEFT_TOP, v, f_ui(13.0), pal::TEXT);
            y += 22.0;
        }
        if !m.metar.is_empty() {
            y += 6.0;
            label(&p, Pos2::new(x, y), "WEATHER", pal::TEXT3);
            y += 20.0;
            y += wrap_text(&p, Pos2::new(x, y), &m.metar, f_mono(12.0), pal::TEXT2, sr.width() - 44.0) + 10.0;
        }
        y += 6.0;
        label(&p, Pos2::new(x, y), "YOUR BEST", pal::TEXT3);
        y += 22.0;
        match rec {
            Some(b) if b.attempts > 0 => {
                if b.medal != Medal::None {
                    medal(&p, Pos2::new(x + 10.0, y + 14.0), 10.0, b.medal);
                }
                let tx = if b.medal != Medal::None { x + 32.0 } else { x };
                p.text(Pos2::new(tx, y), Align2::LEFT_TOP, format!("{:.0} points · {}", b.score, b.grade), f_semi(16.0), pal::TEXT);
                p.text(
                    Pos2::new(tx, y + 22.0),
                    Align2::LEFT_TOP,
                    format!("{} · {} attempt{}", if b.completed { "Recovered" } else { "Not yet recovered" }, b.attempts, if b.attempts == 1 { "" } else { "s" }),
                    f_ui(12.5),
                    pal::TEXT3,
                );
            }
            _ => {
                p.text(Pos2::new(x, y), Align2::LEFT_TOP, "Not flown yet", f_ui(13.5), pal::TEXT2);
            }
        }
    }
    let nav = super::scroll::nav(ui, &st.input, st.frame_no);
    let fy = content.max.y - 52.0;
    if button(ui, Rect::from_min_size(Pos2::new(content.min.x, fy), Vec2::new(140.0, 48.0)), "bback", "Back", Btn::Secondary, true).clicked() || nav.back {
        st.actions.push(Action::Goto(Screen::Missions));
    }
    let r = Rect::from_min_size(Pos2::new(content.max.x - 240.0, fy), Vec2::new(240.0, 48.0));
    if button(ui, r, "bfly", "Fly this mission", Btn::Primary, true).clicked() || nav.ok {
        st.missions.pending = Some(m.id.clone());
        st.actions.push(Action::StartMission);
    }
    p.text(Pos2::new(r.min.x - 20.0, fy + 24.0), Align2::RIGHT_CENTER, "Tab: pause · O: systems & failures · F4: radio (MAYDAY)", f_ui(12.5), pal::TEXT3);
}

fn day_label(doy: u32) -> String {
    const M: [&str; 12] = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    const START: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let i = START.iter().rposition(|&s| doy > s).unwrap_or(0);
    format!("{} {}", doy - START[i], M[i])
}

// ---------------------------------------------------------------- in flight

/// Objective panel, mission clock and the crew / cabin / ATC lines, in the
/// left column at `at` (under the status card). Returns the bottom.
pub fn hud(ctx: &egui::Context, st: &UiState, sim: &Sim, at: Pos2) -> f32 {
    let Some(run) = sim.mission.as_ref() else { return at.y };
    if !run.started {
        return at.y;
    }
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("mission_hud")));
    let objs = run.hud_objectives();
    let w = 330.0;
    let x0 = at.x;
    let y0 = at.y;
    let row_h = |t: &str| p.layout(t.to_string(), f_ui(13.0), pal::TEXT, w - 64.0).size().y.max(16.0) + 9.0;
    let h = 74.0 + objs.iter().map(|o| row_h(&o.0)).sum::<f32>() + 6.0;
    let r = Rect::from_min_size(Pos2::new(x0, y0), Vec2::new(w, h));
    panel(&p, r, sp::R_PANEL);
    p.text(r.min + Vec2::new(16.0, 12.0), Align2::LEFT_TOP, "MISSION", f_inst_b(10.5), pal::ACCENT);
    p.text(r.min + Vec2::new(16.0, 28.0), Align2::LEFT_TOP, &run.m.title, f_semi(16.0), pal::TEXT);
    // The clock: the limit counting down (amber, red in the last minute), else elapsed.
    let (txt, col, cap) = match run.time_left() {
        Some(l) => (
            clock(l),
            if l < 60.0 { if (st.time * 2.0).fract() < 0.5 { pal::RED } else { pal::WHITE } } else { pal::AMBER },
            if run.m.clock_label.is_empty() { "TIME LEFT".to_string() } else { run.m.clock_label.to_uppercase() },
        ),
        None => (clock(run.t), pal::TEXT, "ELAPSED".to_string()),
    };
    p.text(Pos2::new(r.max.x - 16.0, r.min.y + 12.0), Align2::RIGHT_TOP, cap, f_inst_b(9.5), pal::TEXT3);
    p.text(Pos2::new(r.max.x - 16.0, r.min.y + 25.0), Align2::RIGHT_TOP, txt, f_semi(22.0), col);
    p.line_segment([Pos2::new(r.min.x + 14.0, r.min.y + 62.0), Pos2::new(r.max.x - 14.0, r.min.y + 62.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
    let mut y = r.min.y + 72.0;
    for (text, state, required, left) in &objs {
        let c = Pos2::new(r.min.x + 25.0, y + 8.0);
        match state {
            ObjShow::Done => {
                p.circle_filled(c, 7.5, pal::GREEN);
                p.add(Shape::line(vec![c + Vec2::new(-3.5, 0.0), c + Vec2::new(-1.0, 2.8), c + Vec2::new(3.8, -3.0)], Stroke::new(1.8, Color32::from_rgb(6, 30, 16))));
            }
            ObjShow::Failed => {
                p.circle_filled(c, 7.5, pal::RED);
                p.line_segment([c + Vec2::new(-3.0, -3.0), c + Vec2::new(3.0, 3.0)], Stroke::new(1.8, Color32::from_rgb(40, 6, 6)));
                p.line_segment([c + Vec2::new(-3.0, 3.0), c + Vec2::new(3.0, -3.0)], Stroke::new(1.8, Color32::from_rgb(40, 6, 6)));
            }
            ObjShow::Open => {
                p.circle_stroke(c, 7.0, Stroke::new(1.6, if *required { pal::WHITE } else { pal::TEXT3 }));
            }
        }
        let col = match state {
            ObjShow::Done => pal::TEXT2,
            ObjShow::Failed => alpha(pal::RED, 0.9),
            ObjShow::Open => {
                if *required {
                    pal::TEXT
                } else {
                    pal::TEXT2
                }
            }
        };
        let tw = w - 64.0 - if left.is_some() { 40.0 } else { 0.0 };
        let hh = wrap_text(&p, Pos2::new(r.min.x + 42.0, y), text, if *required { f_semi(13.0) } else { f_ui(13.0) }, col, tw);
        if let Some(s) = left {
            p.text(Pos2::new(r.max.x - 14.0, y), Align2::RIGHT_TOP, format!("{:.0}s", s.ceil()), f_mono(12.0), if *s < 10.0 { pal::RED } else { pal::AMBER });
        }
        y += hh.max(16.0) + 9.0;
    }
    // ---- lines: the last few, fading out (bottom-left, above the instruments).
    let recent: Vec<_> = run.lines.iter().rev().filter(|l| run.t - l.t < 9.0).take(3).collect();
    let mut ly = r.max.y + 12.0;
    for l in recent.iter().rev() {
        let a = (1.0 - ((run.t - l.t) - 7.0) / 2.0).clamp(0.0, 1.0);
        let who = if l.who.is_empty() { "" } else { l.who };
        let tw = w - 24.0;
        let g = p.layout(l.text.clone(), f_ui(13.5), alpha(pal::TEXT, a), tw - if who.is_empty() { 0.0 } else { 0.0 });
        let lh = g.size().y + if who.is_empty() { 18.0 } else { 36.0 };
        let lr = Rect::from_min_size(Pos2::new(x0, ly), Vec2::new(w, lh));
        p.rect_filled(lr, sp::R_PANEL, alpha(Color32::from_rgba_premultiplied(8, 10, 14, 220), a));
        let accent = match who {
            "PA" => pal::CYAN,
            "ATC" => pal::GREEN,
            "" => pal::RED,
            _ => pal::AMBER,
        };
        p.rect_filled(Rect::from_min_size(lr.min, Vec2::new(3.0, lh)), CornerRadius { nw: sp::R_PANEL, sw: sp::R_PANEL, ne: 0, se: 0 }, alpha(accent, a));
        let mut ty = ly + 9.0;
        if !who.is_empty() {
            let label = match who {
                "PA" => "CABIN PA".to_string(),
                w => w.to_uppercase(),
            };
            p.text(Pos2::new(x0 + 14.0, ty), Align2::LEFT_TOP, label, f_inst_b(10.0), alpha(accent, a));
            ty += 18.0;
        }
        p.galley(Pos2::new(x0 + 14.0, ty), g, alpha(pal::TEXT, a));
        ly = lr.max.y + 6.0;
    }
    // The end: a banner until the debrief opens (a crash has its own).
    if let Some(d) = run.end.as_ref().filter(|_| sim.ac.crashed.is_none()) {
        let col = if d.success { pal::GREEN } else { pal::RED };
        let br = Rect::from_center_size(Pos2::new(screen.center().x, screen.min.y + 90.0), Vec2::new(520.0, 64.0));
        panel_with(&p, br, sp::R_PANEL, Color32::from_rgba_premultiplied(8, 10, 14, 230));
        p.rect_stroke(br, sp::R_PANEL, Stroke::new(1.5, col), egui::StrokeKind::Inside);
        p.text(br.center() - Vec2::new(0.0, 10.0), Align2::CENTER_CENTER, &d.headline, f_semi(20.0), col);
        p.text(br.center() + Vec2::new(0.0, 15.0), Align2::CENTER_CENTER, "Debrief in a moment…", f_ui(12.5), pal::TEXT3);
    }
    ly
}

// ---------------------------------------------------------------- pause (in a mission)

pub fn pause(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(4, 6, 9, 150));
    let m = st.missions.active.as_deref().and_then(defs::by_id);
    let w = 400.0;
    let items: Vec<(&str, Action, Btn)> = vec![
        ("Resume", Action::Resume, Btn::Primary),
        ("Retry from checkpoint", Action::RetryCheckpoint, Btn::Secondary),
        ("Restart mission", Action::Restart, Btn::Secondary),
        ("Systems & failures", Action::Failures, Btn::Secondary),
        ("Settings", Action::Goto(Screen::Settings), Btn::Secondary),
        ("Abandon — missions", Action::Goto(Screen::Missions), Btn::Ghost),
        ("Main menu", Action::Goto(Screen::Main), Btn::Ghost),
    ];
    let step = ((screen.height() - 48.0 - 130.0) / items.len() as f32).clamp(44.0, 54.0);
    let h = 120.0 + items.len() as f32 * step + 10.0;
    let card = Rect::from_center_size(screen.center(), Vec2::new(w, h));
    panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
    p.text(card.min + Vec2::new(28.0, 24.0), Align2::LEFT_TOP, "Paused", f_display(28.0), pal::TEXT);
    p.text(card.min + Vec2::new(28.0, 64.0), Align2::LEFT_TOP, m.map(|m| m.title.as_str()).unwrap_or("Mission"), f_semi(14.0), pal::TEXT2);
    p.text(card.min + Vec2::new(28.0, 84.0), Align2::LEFT_TOP, m.map(|m| m.tagline.as_str()).unwrap_or(""), f_ui(12.5), pal::TEXT3);
    let mut y = card.min.y + 118.0;
    for (i, (label, act, kind)) in items.into_iter().enumerate() {
        let r = Rect::from_min_size(Pos2::new(card.min.x + 24.0, y), Vec2::new(w - 48.0, step - 8.0));
        if button(ui, r, ("mpause", i), label, kind, true).clicked() {
            st.actions.push(act);
        }
        y += step;
    }
}

// ---------------------------------------------------------------- debrief

fn bar(p: &egui::Painter, r: Rect, v: f32, max: f32, col: Color32) {
    p.rect_filled(r, 3, Color32::from_white_alpha(14));
    let f = (v / max.max(1.0)).clamp(0.0, 1.0);
    if f > 0.0 {
        p.rect_filled(Rect::from_min_size(r.min, Vec2::new(r.width() * f, r.height())), 3, col);
    }
}

pub fn debrief(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(4, 6, 9, 120));
    super::menus::backdrop_pub(&p, screen, 0.95);
    let Some((id, d, new_best, cp)) = st.missions.last.clone() else {
        st.actions.push(Action::Goto(Screen::Missions));
        return;
    };
    let m = defs::by_id(&id);
    let margin = (screen.width() * 0.06).clamp(24.0, 110.0);
    let content = Rect::from_min_max(Pos2::new(screen.min.x + margin, screen.min.y + 40.0), Pos2::new(screen.max.x - margin, screen.max.y - 36.0));
    let col = if d.success { pal::GREEN } else { pal::RED };
    let body = Rect::from_min_max(content.min, Pos2::new(content.max.x, content.max.y - 70.0));
    super::scroll::page(ui, body, "debrief", Color32::from_rgba_premultiplied(5, 7, 10, 240), |ui, o, w| {
        let p = ui.painter().clone();
        let w = w - super::scroll::GUTTER;
        let x = o.x;
        let mut y = o.y;
        label(&p, Pos2::new(x, y), "DEBRIEF", pal::ACCENT);
        p.text(Pos2::new(x + 82.0, y), Align2::LEFT_TOP, m.map(|m| m.title.as_str()).unwrap_or(""), f_inst_b(11.5), pal::TEXT3);
        y += 22.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, &d.headline, f_display(40.0), col);
        y += 54.0;
        y += wrap_text(&p, Pos2::new(x, y), &d.reason, f_ui(16.0), pal::TEXT2, w * 0.6) + 14.0;
        // Occupants.
        let ok = d.pax.saturating_sub(d.injured + d.fatal);
        let occ = if d.fatal > 0 {
            format!("{} aboard — no survivors", d.pax)
        } else if d.injured > 0 {
            format!("{} aboard — {ok} uninjured, {} injured", d.pax, d.injured)
        } else {
            format!("{} aboard — everyone walks away", d.pax)
        };
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, occ, f_semi(15.0), if d.fatal > 0 { pal::RED } else { pal::TEXT });
        y += 26.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, format!("Mission time {}", clock(d.time_s)), f_ui(13.5), pal::TEXT3);
        // ---- score card (right)
        let sw = 360.0f32.min(w * 0.38);
        let sr = Rect::from_min_size(Pos2::new(x + w - sw, o.y + 4.0), Vec2::new(sw, 250.0));
        panel_with(&p, sr, sp::R_CARD, pal::PANEL_SOLID);
        let mc = Pos2::new(sr.min.x + 44.0, sr.min.y + 62.0);
        if d.score.medal != Medal::None {
            medal(&p, mc, 22.0, d.score.medal);
        } else {
            p.circle_stroke(mc, 22.0, Stroke::new(2.0, pal::TEXT3));
        }
        p.text(Pos2::new(sr.min.x + 82.0, sr.min.y + 24.0), Align2::LEFT_TOP, format!("{:.0}", d.score.total), super::f_digits_b(40.0), pal::TEXT);
        p.text(
            Pos2::new(sr.min.x + 82.0, sr.min.y + 72.0),
            Align2::LEFT_TOP,
            format!("Grade {}{}", d.score.grade, if d.score.medal != Medal::None { format!(" · {}", d.score.medal.name()) } else { String::new() }),
            f_semi(14.0),
            medal_col(d.score.medal),
        );
        if new_best {
            super::chip(&p, Pos2::new(sr.max.x - 16.0, sr.min.y + 18.0), Align2::RIGHT_TOP, "NEW BEST", f_inst_b(10.5), Color32::from_rgb(6, 18, 32), pal::ACCENT);
        }
        let rows =
            [("Survival", d.score.survival, 400.0), ("Landing / handling", d.score.landing, 300.0), ("Procedures", d.score.procedures, 200.0), ("Time", d.score.time, 100.0)];
        let mut yy = sr.min.y + 112.0;
        for (k, v, mx) in rows {
            p.text(Pos2::new(sr.min.x + 20.0, yy), Align2::LEFT_TOP, k, f_ui(13.0), pal::TEXT2);
            p.text(Pos2::new(sr.max.x - 20.0, yy), Align2::RIGHT_TOP, format!("{v:.0} / {mx:.0}"), f_mono(12.5), pal::TEXT);
            bar(
                &p,
                Rect::from_min_size(Pos2::new(sr.min.x + 20.0, yy + 19.0), Vec2::new(sw - 40.0, 5.0)),
                v,
                mx,
                if v >= mx * 0.75 {
                    pal::GREEN
                } else if v > 0.0 {
                    pal::AMBER
                } else {
                    pal::RED
                },
            );
            yy += 32.0;
        }
        y = y.max(sr.max.y - o.y + o.y) + 26.0;
        // ---- columns: touchdown / landing factors, objectives
        let cw = (w - 24.0) * 0.5;
        let mut ly = y;
        label(&p, Pos2::new(x, ly), "THE NUMBERS", pal::TEXT3);
        ly += 24.0;
        for (k, v) in &d.stats {
            p.text(Pos2::new(x, ly), Align2::LEFT_TOP, k, f_ui(13.5), pal::TEXT3);
            ly += wrap_text(&p, Pos2::new(x + 150.0, ly), v, f_ui(13.5), pal::TEXT, cw - 150.0).max(18.0) + 6.0;
        }
        if !d.score.factors.is_empty() {
            ly += 10.0;
            label(&p, Pos2::new(x, ly), "LANDING QUALITY", pal::TEXT3);
            ly += 24.0;
            for (k, v) in &d.score.factors {
                p.text(Pos2::new(x, ly), Align2::LEFT_TOP, *k, f_ui(13.0), pal::TEXT2);
                bar(
                    &p,
                    Rect::from_min_size(Pos2::new(x + 150.0, ly + 6.0), Vec2::new(cw - 200.0, 6.0)),
                    *v,
                    1.0,
                    if *v > 0.75 {
                        pal::GREEN
                    } else if *v > 0.35 {
                        pal::AMBER
                    } else {
                        pal::RED
                    },
                );
                p.text(Pos2::new(x + cw - 10.0, ly), Align2::RIGHT_TOP, format!("{:.0}%", v * 100.0), f_mono(12.0), pal::TEXT);
                ly += 24.0;
            }
        }
        let ox = x + cw + 24.0;
        let mut oy = y;
        label(&p, Pos2::new(ox, oy), "OBJECTIVES", pal::TEXT3);
        oy += 24.0;
        for (t, done, req) in &d.objectives {
            let c = Pos2::new(ox + 8.0, oy + 9.0);
            if *done {
                p.circle_filled(c, 7.0, pal::GREEN);
            } else {
                p.circle_stroke(c, 7.0, Stroke::new(1.6, if *req { pal::RED } else { pal::TEXT3 }));
            }
            let lab = if *req { format!("{t}  (required)") } else { t.clone() };
            oy += wrap_text(&p, Pos2::new(ox + 26.0, oy), &lab, f_ui(13.5), if *done { pal::TEXT } else { pal::TEXT2 }, cw - 30.0).max(18.0) + 9.0;
        }
        if let Some(m) = m {
            oy += 14.0;
            oy += wrap_text(&p, Pos2::new(ox, oy), &m.history, f_ui(12.0), pal::TEXT3, cw - 10.0) + 6.0;
        }
        ly.max(oy) + 10.0 - o.y
    });
    let fy = content.max.y - 52.0;
    let mut bx = content.min.x;
    let mut btn = |ui: &mut Ui, label: &str, w: f32, kind: Btn, act: Action, id: &str| {
        let r = Rect::from_min_size(Pos2::new(bx, fy), Vec2::new(w, 48.0));
        if button(ui, r, id, label, kind, true).clicked() {
            st.actions.push(act);
        }
        bx += w + 12.0;
    };
    btn(ui, "Missions", 150.0, Btn::Secondary, Action::Goto(Screen::Missions), "dmiss");
    btn(ui, "Main menu", 150.0, Btn::Ghost, Action::Goto(Screen::Main), "dmain");
    let r = Rect::from_min_size(Pos2::new(content.max.x - 200.0, fy), Vec2::new(200.0, 48.0));
    if button(ui, r, "dretry", "Fly it again", Btn::Primary, true).clicked() {
        st.actions.push(Action::Restart);
    }
    if let Some(cp) = cp.filter(|c| c != "Start") {
        let r2 = Rect::from_min_size(Pos2::new(r.min.x - 272.0, fy), Vec2::new(260.0, 48.0));
        if button(ui, r2, "dcp", &format!("Retry from “{cp}”"), Btn::Secondary, true).clicked() {
            st.actions.push(Action::RetryCheckpoint);
        }
    }
}
