//! In-flight multiplayer interface: name tags over other aircraft
//! (distance-faded, Num * toggles), the chat (Enter), the player list with
//! spectating (F11), the quick-chat wheel (Backspace / Num 0 / hold a controller's View
//! button), the connection chip, other aircraft on the moving map and
//! TCAS-style traffic on the navigation display.

use super::map::View;
use super::{UiState, alpha, f_inst_b, f_mono, f_semi, f_ui, pal, panel_with, sp, text_halo};
use crate::net::{ChatKind, QUICK_CHAT, Status, Traffic};
use crate::sim::Sim;
use egui::{Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2};
use glam::{Mat4, Vec3, Vec4, vec2};

/// Tags are fully visible within this distance and gone beyond `TAG_FAR`, m.
const TAG_NEAR: f32 = 2500.0;
const TAG_FAR: f32 = 15_000.0;
/// Chat lines stay this long (s) unless the chat is open.
const CHAT_LIFE: f64 = 14.0;

/// World point → screen with the frame's camera; also the distance along the view.
fn project(cam: &(Vec3, Mat4, f32), screen: Rect, p: Vec3) -> Option<(Pos2, f32)> {
    let (eye, view, fov) = *cam;
    // (Rotation-only view: positions eye-relative, like the renderer.)
    let v = Mat4::from_cols(view.x_axis, view.y_axis, view.z_axis, Vec4::W).transform_vector3(p - eye);
    if v.z > -0.5 {
        return None;
    }
    let t = (fov * 0.5).tan();
    let aspect = screen.width() / screen.height().max(1.0);
    let (x, y) = (v.x / -v.z / t / aspect, v.y / -v.z / t);
    if x.abs() > 1.3 || y.abs() > 1.3 {
        return None;
    }
    Some((Pos2::new(screen.center().x + x * screen.width() * 0.5, screen.center().y - y * screen.height() * 0.5), -v.z))
}

fn thousands(v: f32) -> String {
    super::thousands(v)
}

/// Everything multiplayer over the flight. `sim`: the player's aircraft.
pub fn draw(ctx: &egui::Context, st: &mut UiState, sim: &Sim) {
    if !st.mp.connected() && !matches!(st.mp.status, Status::Closed(_)) {
        return;
    }
    let screen = ctx.content_rect();
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Background, egui::Id::new("mp_tags")));
    let now = st.mp.now();
    // ---- name tags
    if st.mp.tags {
        if let Some(cam) = st.mp.cam {
            let me = sim.ac.pos;
            for r in st.mp.remotes.values() {
                if !r.visible() {
                    continue;
                }
                let d = r.pos.distance(cam.0);
                if d > TAG_FAR {
                    continue;
                }
                let Some((q, _)) = project(&cam, screen, r.pos + Vec3::Y * (r.size * 0.3 + 1.0)) else { continue };
                let a = 1.0 - ((d - TAG_NEAR) / (TAG_FAR - TAG_NEAR)).clamp(0.0, 1.0);
                let a = a * a.max(0.0);
                if a < 0.03 {
                    continue;
                }
                if st.mp.spectate == Some(r.id) {
                    continue; // (the spectator banner names it)
                }
                let col = if r.crashed { pal::RED } else { pal::CYAN };
                // Marker: a small chevron pointing at the aircraft.
                let m = q + Vec2::new(0.0, -4.0);
                p.add(Shape::convex_polygon(vec![m + Vec2::new(-5.0, -7.0), m + Vec2::new(5.0, -7.0), m], alpha(col, a), Stroke::NONE));
                let name_at = q + Vec2::new(0.0, -14.0);
                // Distance from our aircraft (from the camera while spectating).
                let dd = if st.mp.spectate.is_some() { d } else { r.pos.distance(me) };
                let dist = if dd < 1000.0 { format!("{dd:.0} m") } else { format!("{:.1} km", dd / 1000.0) };
                let alt_u = st.settings.alt;
                let dalt = alt_u.from_m(r.pos.y - me.y);
                let ac = st.defs.iter().find(|x| x.id == r.aircraft).map(|x| x.name).unwrap_or(r.aircraft.as_str());
                let sub = format!("{ac} · {dist} · {}{} {}", if dalt >= 0.0 { "+" } else { "−" }, thousands(dalt.abs()), alt_u.label());
                // A soft dark chip keeps the tag legible over bright sky and cloud.
                let g1 = p.layout_no_wrap(r.name.clone(), f_semi(14.0), pal::WHITE);
                let g2 = p.layout_no_wrap(sub, f_ui(11.5), pal::TEXT);
                let bw = g1.size().x.max(g2.size().x) + 18.0;
                let bg = Rect::from_center_size(name_at + Vec2::new(0.0, -14.0), Vec2::new(bw, 36.0));
                p.rect_filled(bg, 8, Color32::from_black_alpha((95.0 * a) as u8));
                p.rect_filled(Rect::from_min_size(bg.min + Vec2::new(0.0, 6.0), Vec2::new(2.5, bg.height() - 12.0)), 1, alpha(col, a));
                p.galley(Pos2::new(bg.center().x - g1.size().x * 0.5, bg.min.y + 2.0), g1, alpha(pal::WHITE, a));
                p.galley(Pos2::new(bg.center().x - g2.size().x * 0.5, bg.min.y + 19.0), g2, alpha(pal::TEXT, a * 0.9));
            }
        }
    }
    // ---- connection chip (top centre, under the toasts' origin)
    {
        let (txt, col) = match &st.mp.status {
            Status::Connected => {
                let ms = st.mp.ping_ms();
                let loss = st.mp.loss();
                let col = if ms < 80.0 && loss < 0.03 {
                    pal::GREEN
                } else if ms < 200.0 && loss < 0.1 {
                    pal::AMBER
                } else {
                    pal::RED
                };
                let name = st.mp.server.as_ref().map(|s| s.name.clone()).unwrap_or_default();
                let n = st.mp.remotes.len() + 1;
                (format!("{name} · {n} online · {ms:.0} ms{}", if loss >= 0.01 { format!(" · {:.0} % loss", loss * 100.0) } else { String::new() }), col)
            }
            Status::Closed(m) => (format!("Offline — {m}"), pal::RED),
            _ => (String::new(), pal::TEXT3),
        };
        if !txt.is_empty() {
            // Under the chat (left), clear of every instrument layout; centred at the
            // bottom while spectating (no instruments then).
            let g = p.layout_no_wrap(txt, f_ui(11.5), pal::TEXT2);
            let size = g.size() + Vec2::new(30.0, 8.0);
            let r = if st.mp.spectate.is_some() {
                Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - 8.0 - size.y * 0.5), size)
            } else {
                Rect::from_min_size(Pos2::new(sp::L, screen.max.y * 0.62 + 8.0), size)
            };
            p.rect_filled(r, r.height() * 0.5, Color32::from_rgba_premultiplied(8, 10, 14, 170));
            p.circle_filled(Pos2::new(r.min.x + 11.0, r.center().y), 3.5, col);
            p.galley(Pos2::new(r.min.x + 20.0, r.min.y + 4.0), g, pal::TEXT2);
        }
    }
    chat(ctx, st, screen, now);
    if st.mp.spectate.is_some() {
        // (The flight HUD, which shows the toasts, is hidden while spectating.)
        st.toasts.draw(&p, screen, 132.0);
    }
    spectate_banner(ctx, st, screen);
    if st.mp.show_players {
        players(ctx, st, sim, screen);
    }
    if st.mp.wheel.is_some() {
        wheel(ctx, st, screen);
    }
}

fn chat(ctx: &egui::Context, st: &mut UiState, screen: Rect, now: f64) {
    let w = 460.0;
    let x = sp::L;
    let bottom = screen.max.y * 0.62;
    let open = st.mp.chat_open;
    let lines: Vec<_> = st.mp.chat.iter().rev().filter(|l| open || now - l.at < CHAT_LIFE).take(if open { 12 } else { 6 }).cloned().collect();
    if lines.is_empty() && !open {
        return;
    }
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Middle, egui::Id::new("mp_chat")));
    let line_h = 21.0;
    let h = lines.len() as f32 * line_h + if open { 52.0 } else { 12.0 };
    let r = Rect::from_min_size(Pos2::new(x, bottom - h), Vec2::new(w, h));
    if open {
        panel_with(&p, r, sp::R_PANEL, pal::PANEL);
    }
    let mut y = r.min.y + 8.0 + (lines.len() as f32 - 1.0) * line_h;
    for l in &lines {
        let fade = if open { 1.0 } else { ((CHAT_LIFE - (now - l.at)) / 2.0).clamp(0.0, 1.0) as f32 };
        let (nc, tc) = match l.kind {
            ChatKind::Player => (pal::CYAN, pal::TEXT),
            ChatKind::Me => (pal::GREEN, pal::TEXT),
            ChatKind::Server => (pal::AMBER, pal::TEXT),
            ChatKind::System => (pal::TEXT3, pal::TEXT2),
        };
        let pos = Pos2::new(r.min.x + 12.0, y);
        if l.name.is_empty() {
            text_halo(&p, pos, Align2::LEFT_TOP, &l.text, f_ui(13.0), alpha(tc, fade));
        } else {
            let nr = text_halo(&p, pos, Align2::LEFT_TOP, format!("{}:", l.name), f_semi(13.0), alpha(nc, fade));
            let mut text: String = l.text.chars().take(80).collect();
            // Settings › Multiplayer › Chat filter (other players' lines).
            if st.settings.mp.chat_filter && matches!(l.kind, ChatKind::Player) {
                text = super::settings_schema::filter_chat(&text);
            }
            text_halo(&p, Pos2::new(nr.max.x + 6.0, y), Align2::LEFT_TOP, text, f_ui(13.0), alpha(tc, fade));
        }
        y -= line_h;
    }
    if open {
        let fr = Rect::from_min_size(Pos2::new(r.min.x + 8.0, r.max.y - 42.0), Vec2::new(w - 16.0, 34.0));
        let mut send = false;
        let mut close = false;
        egui::Area::new(egui::Id::new("mp_chat_in")).order(egui::Order::Foreground).fixed_pos(fr.min).show(ctx, |ui| {
            let pp = ui.painter();
            pp.rect_filled(fr, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
            pp.rect_stroke(fr, sp::R_CTRL, Stroke::new(1.0, pal::ACCENT_DEEP), egui::StrokeKind::Inside);
            let te = ui.put(
                fr.shrink2(Vec2::new(10.0, 7.0)),
                egui::TextEdit::singleline(&mut st.mp.chat_draft)
                    .hint_text("Say something — Enter to send, Esc to close")
                    .frame(egui::Frame::NONE)
                    .font(f_ui(14.0))
                    .text_color(pal::TEXT)
                    .char_limit(skyward_net::proto::MAX_CHAT),
            );
            te.request_focus();
            let fresh = st.frame_no <= st.mp.chat_open_frame + 1;
            ui.input(|i| {
                if i.key_pressed(egui::Key::Enter) && !fresh {
                    send = true;
                }
                if i.key_pressed(egui::Key::Escape) {
                    close = true;
                }
            });
        });
        if send {
            let t = std::mem::take(&mut st.mp.chat_draft);
            st.mp.say(&t);
            st.mp.chat_open = false;
        }
        if close {
            st.mp.chat_open = false;
        }
    }
}

fn spectate_banner(ctx: &egui::Context, st: &mut UiState, screen: Rect) {
    let Some(name) = st.mp.spectating_name() else { return };
    let ac = st
        .mp
        .spectate
        .and_then(|id| st.mp.remotes.get(&id))
        .map(|r| st.defs.iter().find(|d| d.id == r.aircraft).map(|d| d.name.to_string()).unwrap_or(r.aircraft.clone()))
        .unwrap_or_default();
    let w = 520.0;
    let r = Rect::from_min_size(Pos2::new(screen.center().x - w * 0.5, 64.0), Vec2::new(w, 56.0));
    egui::Area::new(egui::Id::new("mp_spec")).order(egui::Order::Foreground).fixed_pos(r.min).show(ctx, |ui| {
        let p = ui.painter().clone();
        panel_with(&p, r, sp::R_PANEL, pal::PANEL_SOLID);
        p.text(Pos2::new(r.min.x + 18.0, r.min.y + 10.0), Align2::LEFT_TOP, "SPECTATING", f_inst_b(10.5), pal::AMBER);
        p.text(Pos2::new(r.min.x + 18.0, r.min.y + 26.0), Align2::LEFT_TOP, format!("{name} · {ac}"), f_semi(15.0), pal::TEXT);
        let b = |i: usize| Rect::from_min_size(Pos2::new(r.max.x - 12.0 - (3 - i) as f32 * 76.0, r.min.y + 11.0), Vec2::new(70.0, 34.0));
        if super::menus::button(ui, b(0), "spec_prev", "◀", super::menus::Btn::Secondary, true).clicked() {
            st.mp.cycle_spectate(-1);
        }
        if super::menus::button(ui, b(1), "spec_next", "▶", super::menus::Btn::Secondary, true).clicked() {
            st.mp.cycle_spectate(1);
        }
        if super::menus::button(ui, b(2), "spec_stop", "Fly", super::menus::Btn::Primary, true).clicked() {
            st.mp.stop_spectating();
        }
    });
}

fn players(ctx: &egui::Context, st: &mut UiState, sim: &Sim, screen: Rect) {
    let n = st.mp.remotes.len() + 1;
    let row = 40.0;
    let h = 186.0 + n as f32 * row;
    let w = 620.0;
    let r = Rect::from_center_size(screen.center(), Vec2::new(w, h.min(screen.height() - 80.0)));
    let mut close = false;
    egui::Area::new(egui::Id::new("mp_players")).order(egui::Order::Foreground).fixed_pos(r.min).show(ctx, |ui| {
        ui.set_min_size(r.size());
        let p = ui.painter().clone();
        panel_with(&p, r, sp::R_CARD, pal::PANEL_SOLID);
        let sv = st.mp.server.clone().unwrap_or_default();
        p.text(r.min + Vec2::new(24.0, 20.0), Align2::LEFT_TOP, &sv.name, super::f_display(22.0), pal::TEXT);
        let place = if sv.region.is_empty() { sv.airport.clone() } else { format!("{} · {}", sv.airport, sv.region) };
        p.text(r.min + Vec2::new(24.0, 52.0), Align2::LEFT_TOP, format!("{place} · {} · {:.0} ms", sv.addr, st.mp.ping_ms()), f_ui(12.5), pal::TEXT2);
        let mut y = r.min.y + 88.0;
        for (k, h) in [("PLAYER", 24.0), ("AIRCRAFT", 230.0), ("DISTANCE", 400.0)] {
            p.text(Pos2::new(r.min.x + h, y), Align2::LEFT_TOP, k, f_inst_b(10.5), pal::TEXT3);
        }
        y += 22.0;
        let me = sim.ac.pos;
        let mut rows: Vec<(u16, String, String, Option<f32>, bool)> = vec![(st.mp.me, format!("{} (you)", st.mp.name_of(st.mp.me)), sim.def().name.to_string(), None, false)];
        for rm in st.mp.remotes.values() {
            let ac = st.defs.iter().find(|d| d.id == rm.aircraft).map(|d| d.name.to_string()).unwrap_or(rm.aircraft.clone());
            rows.push((rm.id, rm.name.clone(), ac, rm.visible().then(|| rm.pos.distance(me)), rm.spectating));
        }
        for (id, name, ac, dist, spec) in rows {
            if y > r.max.y - 64.0 - row {
                break;
            }
            p.text(Pos2::new(r.min.x + 24.0, y + 10.0), Align2::LEFT_TOP, &name, f_semi(14.0), if id == st.mp.me { pal::GREEN } else { pal::TEXT });
            p.text(Pos2::new(r.min.x + 230.0, y + 11.0), Align2::LEFT_TOP, &ac, f_ui(13.0), pal::TEXT2);
            let d = match (dist, spec) {
                (_, true) => "spectating".to_string(),
                (Some(d), _) if d < 1000.0 => format!("{d:.0} m"),
                (Some(d), _) => format!("{:.1} km", d / 1000.0),
                (None, _) if id == st.mp.me => "—".into(),
                (None, _) => "…".into(),
            };
            p.text(Pos2::new(r.min.x + 400.0, y + 11.0), Align2::LEFT_TOP, d, f_mono(12.5), pal::TEXT2);
            if id != st.mp.me && dist.is_some() {
                let br = Rect::from_min_size(Pos2::new(r.max.x - 124.0, y + 4.0), Vec2::new(100.0, 30.0));
                if super::menus::button(ui, br, ("spec", id), "Spectate", super::menus::Btn::Secondary, true).clicked() {
                    st.mp.start_spectating(id);
                }
            }
            y += row;
        }
        let by = r.max.y - 52.0;
        if super::menus::button(ui, Rect::from_min_size(Pos2::new(r.max.x - 124.0, by), Vec2::new(100.0, 36.0)), "players_close", "Close", super::menus::Btn::Ghost, true).clicked()
        {
            close = true;
        }
        let key = st.settings.controls.keys_of(crate::sim::Key::NameTags).first().map(|c| format!(" ({})", crate::input::keys::label(*c))).unwrap_or_default();
        let tags = format!("Name tags: {}{key}", if st.mp.tags { "on" } else { "off" });
        if super::menus::button(ui, Rect::from_min_size(Pos2::new(r.min.x + 24.0, by), Vec2::new(190.0, 36.0)), "players_tags", &tags, super::menus::Btn::Secondary, true).clicked()
        {
            st.mp.tags = !st.mp.tags;
        }
    });
    if close {
        st.mp.show_players = false;
    }
}

/// Wheel geometry: sector `i` (0 = top, clockwise) centre direction.
fn sector_dir(i: usize) -> Vec2 {
    let a = i as f32 / QUICK_CHAT.len() as f32 * std::f32::consts::TAU;
    Vec2::new(a.sin(), -a.cos())
}

/// The sector a stick / pointer direction points at (None near the centre).
pub fn wheel_pick(d: Vec2) -> Option<usize> {
    if d.length() < 0.35 {
        return None;
    }
    let a = d.x.atan2(-d.y).rem_euclid(std::f32::consts::TAU);
    let n = QUICK_CHAT.len() as f32;
    Some(((a / std::f32::consts::TAU * n + 0.5).floor() as usize) % QUICK_CHAT.len())
}

fn wheel(ctx: &egui::Context, st: &mut UiState, screen: Rect) {
    let c = screen.center();
    let rad = 190.0;
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("mp_wheel")));
    p.circle_filled(c, rad + 70.0, Color32::from_rgba_premultiplied(5, 7, 10, 215));
    p.circle_stroke(c, rad + 70.0, Stroke::new(1.0, pal::HAIRLINE_HI));
    // Pointer selects (mouse), click sends.
    let ptr = ctx.input(|i| i.pointer.hover_pos());
    let mut w = st.mp.wheel.unwrap_or_default();
    if let Some(q) = ptr {
        if let Some(i) = wheel_pick((q - c) / rad) {
            if !w.held {
                w.sel = Some(i);
            }
        }
    }
    for (i, label) in QUICK_CHAT.iter().enumerate() {
        let d = sector_dir(i);
        let at = c + d * rad;
        let on = w.sel == Some(i);
        let num = p.layout_no_wrap(format!("{}", i + 1), f_inst_b(11.0), if on { pal::WHITE } else { pal::TEXT3 });
        let g = p.layout_no_wrap(label.to_string(), f_semi(14.0), pal::TEXT);
        let r = Rect::from_center_size(at, Vec2::new(g.size().x + num.size().x + 34.0, g.size().y + 14.0));
        p.rect_filled(r, r.height() * 0.5, if on { pal::ACCENT_DEEP } else { pal::RAISED });
        p.rect_stroke(r, r.height() * 0.5, Stroke::new(if on { 1.5 } else { 1.0 }, if on { pal::ACCENT } else { pal::HAIRLINE_HI }), egui::StrokeKind::Inside);
        let ny = r.center().y - num.size().y * 0.5;
        p.galley(Pos2::new(r.min.x + 12.0, ny), num, pal::TEXT3);
        p.galley(Pos2::new(r.max.x - 12.0 - g.size().x, r.min.y + 7.0), g, if on { pal::WHITE } else { pal::TEXT });
    }
    p.text(c, Align2::CENTER_CENTER, "QUICK CHAT", f_inst_b(12.0), pal::TEXT2);
    p.text(c + Vec2::new(0.0, 18.0), Align2::CENTER_CENTER, if w.held { "stick to pick · release to send" } else { "1–8 or click · Esc closes" }, f_ui(11.0), pal::TEXT3);
    if let Some(i) = w.sel {
        p.line_segment([c + sector_dir(i) * 40.0, c + sector_dir(i) * (rad - 60.0)], Stroke::new(2.0, pal::ACCENT));
    }
    let clicked = ctx.input(|i| i.pointer.primary_clicked());
    let esc = ctx.input(|i| i.key_pressed(egui::Key::Escape));
    st.mp.wheel = Some(w);
    if clicked && !w.held {
        if let Some(i) = w.sel {
            st.mp.say(QUICK_CHAT[i]);
        }
        st.mp.wheel = None;
    } else if esc {
        st.mp.wheel = None;
    }
}

// ---------------------------------------------------------------- map / ND traffic

/// Other aircraft on the moving map: symbol, name and relative altitude.
pub fn map_traffic(p: &egui::Painter, view: &View, st: &UiState, sim: &Sim, dark: bool) {
    if !st.mp.connected() {
        return;
    }
    let col = if dark { pal::CYAN } else { Color32::from_rgb(0, 120, 150) };
    for t in st.mp.traffic() {
        let q = view.to_screen(vec2(t.pos.x, t.pos.z));
        if !view.rect.shrink(4.0).contains(q) {
            continue;
        }
        super::map::aircraft_symbol(p, q, view.screen_angle(t.heading), 11.0, col);
        let dft = (t.pos.y - sim.ac.pos.y) / 0.3048 / 100.0;
        let lab = format!("{}  {}{:02.0}", t.name, if dft >= 0.0 { "+" } else { "−" }, dft.abs().min(99.0));
        let tc = if dark { pal::TEXT } else { Color32::from_rgb(20, 24, 30) };
        p.text(q + Vec2::new(10.0, -12.0), Align2::LEFT_BOTTOM, lab, f_inst_b(10.5), tc);
    }
}

/// TCAS classification of one intruder (simplified TCAS II thresholds).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Threat {
    Other,
    Proximate,
    /// Traffic advisory.
    Ta,
    /// Resolution advisory.
    Ra,
}

pub fn threat(own_pos: Vec3, own_vel: Vec3, t: &Traffic) -> Threat {
    let r = t.pos - own_pos;
    let v = t.vel - own_vel;
    let rh = vec2(r.x, r.z);
    let vh = vec2(v.x, v.z);
    let range = rh.length();
    let dz = r.y.abs() / 0.3048;
    let closing = -rh.dot(vh) / range.max(1.0);
    let tau = if closing > 0.5 { range / closing } else { f32::INFINITY };
    let nm = range / 1852.0;
    if t.on_ground {
        return if nm < 6.0 && dz < 1200.0 { Threat::Proximate } else { Threat::Other };
    }
    if (tau < 25.0 || nm < 0.35) && dz < 600.0 {
        Threat::Ra
    } else if (tau < 40.0 || nm < 0.55) && dz < 850.0 {
        Threat::Ta
    } else if nm < 6.0 && dz < 1200.0 {
        Threat::Proximate
    } else {
        Threat::Other
    }
}

/// TCAS symbols on the ND (`glass.rs`): hollow / filled diamonds, amber TA
/// circles, red RA squares, relative altitude (hundreds of feet) and a
/// climb / descent arrow. Reads the per-frame traffic picture.
pub fn nd_traffic(p: &egui::Painter, view: &View, sim: &Sim, u: f32) {
    let traffic = crate::net::traffic_picture();
    if traffic.is_empty() {
        return;
    }
    let own = sim.ac.pos;
    let mut worst = Threat::Other;
    for t in &traffic {
        let q = view.to_screen(vec2(t.pos.x, t.pos.z));
        if !view.rect.contains(q) {
            continue;
        }
        let th = threat(own, sim.ac.vel, t);
        worst = worst.max(th);
        let s = 1.9 * u;
        let white = Color32::from_rgb(240, 242, 245);
        match th {
            Threat::Ra => {
                p.rect_filled(Rect::from_center_size(q, Vec2::splat(s * 1.8)), 0, pal::RED);
            }
            Threat::Ta => {
                p.circle_filled(q, s, pal::AMBER);
            }
            Threat::Proximate => {
                p.add(Shape::convex_polygon(vec![q + Vec2::new(0.0, -s), q + Vec2::new(s, 0.0), q + Vec2::new(0.0, s), q + Vec2::new(-s, 0.0)], white, Stroke::NONE));
            }
            Threat::Other => {
                p.add(Shape::closed_line(vec![q + Vec2::new(0.0, -s), q + Vec2::new(s, 0.0), q + Vec2::new(0.0, s), q + Vec2::new(-s, 0.0)], Stroke::new(1.4, white)));
            }
        }
        let col = match th {
            Threat::Ra => pal::RED,
            Threat::Ta => pal::AMBER,
            _ => white,
        };
        let dh = ((t.pos.y - own.y) / 0.3048 / 100.0).round();
        let lab = format!("{}{:02.0}", if dh >= 0.0 { "+" } else { "−" }, dh.abs().min(99.0));
        let above = dh >= 0.0;
        p.text(q + Vec2::new(0.0, if above { -s - 1.0 } else { s + 1.0 }), if above { Align2::CENTER_BOTTOM } else { Align2::CENTER_TOP }, lab, f_mono(2.6 * u), col);
        let vs = t.vel.y / 0.3048 * 60.0;
        if vs.abs() > 500.0 {
            let x = q.x + s + 1.2 * u;
            let (y0, y1) = if vs > 0.0 { (q.y + s, q.y - s) } else { (q.y - s, q.y + s) };
            p.line_segment([Pos2::new(x, y0), Pos2::new(x, y1)], Stroke::new(1.4, col));
            let d = if vs > 0.0 { -1.0 } else { 1.0 };
            p.add(Shape::convex_polygon(vec![Pos2::new(x, y1), Pos2::new(x - 0.7 * u, y1 - d * 1.2 * u), Pos2::new(x + 0.7 * u, y1 - d * 1.2 * u)], col, Stroke::NONE));
        }
    }
    if worst >= Threat::Ta {
        let (txt, col) = if worst == Threat::Ra { ("TRAFFIC", pal::RED) } else { ("TRAFFIC", pal::AMBER) };
        p.text(Pos2::new(view.rect.min.x + 2.0 * u, view.rect.max.y - 9.0 * u), Align2::LEFT_BOTTOM, txt, f_semi(3.2 * u), col);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tr(pos: Vec3, vel: Vec3) -> Traffic {
        Traffic { id: 1, name: "x".into(), aircraft: "kestrel".into(), pos, vel, heading: 0.0, on_ground: false }
    }

    #[test]
    fn tcas_thresholds() {
        let own = Vec3::new(0.0, 1000.0, 0.0);
        let v = Vec3::new(0.0, 0.0, -100.0);
        // Head-on, 3 km, same level, closing at 200 m/s: tau 15 s → RA.
        assert_eq!(threat(own, v, &tr(Vec3::new(0.0, 1000.0, -3000.0), Vec3::new(0.0, 0.0, 100.0))), Threat::Ra);
        // Same, 6.5 km: tau 32 s → TA.
        assert_eq!(threat(own, v, &tr(Vec3::new(0.0, 1000.0, -6500.0), Vec3::new(0.0, 0.0, 100.0))), Threat::Ta);
        // 3 nm, diverging → proximate.
        assert_eq!(threat(own, v, &tr(Vec3::new(5500.0, 1100.0, 0.0), Vec3::new(100.0, 0.0, 0.0))), Threat::Proximate);
        // 3,000 ft above → other.
        assert_eq!(threat(own, v, &tr(Vec3::new(0.0, 1914.0, -3000.0), Vec3::new(0.0, 0.0, 100.0))), Threat::Other);
    }

    #[test]
    fn wheel_sectors() {
        assert_eq!(wheel_pick(Vec2::new(0.0, -1.0)), Some(0));
        assert_eq!(wheel_pick(Vec2::new(1.0, 0.0)), Some(2));
        assert_eq!(wheel_pick(Vec2::new(0.0, 1.0)), Some(4));
        assert_eq!(wheel_pick(Vec2::new(-0.7, -0.7)), Some(7));
        assert_eq!(wheel_pick(Vec2::new(0.1, 0.1)), None);
        for i in 0..QUICK_CHAT.len() {
            assert_eq!(wheel_pick(sector_dir(i)), Some(i));
        }
    }
}
