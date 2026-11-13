//! Main menu › Multiplayer: the server browser. LAN servers appear by
//! themselves (discovery broadcast + mDNS), favourites and recent servers
//! are pinged, any address can be added; a details panel shows the MOTD,
//! players and conditions; joining asks for a password when the server has
//! one and explains any refusal (unreachable, full, wrong version, wrong
//! password, banned). Model: `net/browse.rs`.

use super::menus::{Btn, button, chip_btn, segmented};
use super::{Action, Screen, UiState, f_display, f_inst_b, f_mono, f_semi, f_ui, pal, panel_with, sp};
use crate::net::Status;
use crate::net::browse::{Entry, SortKey, Tab};
use egui::{Align2, Color32, Pos2, Rect, Sense, Stroke, Ui, Vec2};

const ROW_H: f32 = 58.0;

fn clip(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { format!("{}…", s.chars().take(n.saturating_sub(1)).collect::<String>()) }
}

fn field(ui: &mut Ui, r: Rect, text: &mut String, hint: &str, mono: bool, password: bool) -> egui::Response {
    let p = ui.painter();
    p.rect_filled(r, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    p.rect_stroke(r, sp::R_CTRL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    let font = if mono { f_mono(14.0) } else { f_ui(14.5) };
    ui.put(r.shrink2(Vec2::new(12.0, 8.0)), egui::TextEdit::singleline(text).hint_text(hint).frame(egui::Frame::NONE).font(font).text_color(pal::TEXT).password(password))
}

/// Ping as a coloured "▮▮▮" meter + ms.
fn ping_label(p: &egui::Painter, at: Pos2, ms: Option<f32>) {
    let (bars, col) = match ms {
        Some(m) if m < 40.0 => (3, pal::GREEN),
        Some(m) if m < 120.0 => (2, pal::AMBER),
        Some(_) => (1, pal::RED),
        None => (0, pal::TEXT3),
    };
    for i in 0..3 {
        let h = 5.0 + 4.0 * i as f32;
        let r = Rect::from_min_size(Pos2::new(at.x + i as f32 * 6.0, at.y + 14.0 - h), Vec2::new(4.0, h));
        p.rect_filled(r, 1, if i < bars { col } else { Color32::from_white_alpha(28) });
    }
    let t = ms.map(|m| format!("{:.0} ms", m.max(1.0))).unwrap_or_else(|| "—".into());
    p.text(Pos2::new(at.x + 24.0, at.y + 7.0), Align2::LEFT_CENTER, t, f_mono(12.5), if ms.is_some() { pal::TEXT2 } else { pal::TEXT3 });
}

/// A small padlock (password-protected server).
fn lock_icon(p: &egui::Painter, c: Pos2, col: Color32) {
    let body = Rect::from_center_size(c + Vec2::new(0.0, 2.0), Vec2::new(10.0, 7.5));
    p.rect_filled(body, 1.5, col);
    let pts: Vec<Pos2> = (0..=12)
        .map(|i| {
            let a = std::f32::consts::PI * i as f32 / 12.0;
            c + Vec2::new(-3.2 * a.cos(), -1.8 - 3.4 * a.sin())
        })
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(1.6, col)));
}

/// ‹ / › drawn as strokes (the UI fonts have no arrow glyphs at this weight).
fn chevron(p: &egui::Painter, c: Pos2, dir: f32, col: Color32) {
    let (a, b) = (Vec2::new(-3.0 * dir, -5.5), Vec2::new(-3.0 * dir, 5.5));
    p.add(egui::Shape::line(vec![c + a, c + Vec2::new(3.0 * dir, 0.0), c + b], Stroke::new(2.0, col)));
}

fn status_line(e: &Entry) -> (String, Color32) {
    if let Some(err) = &e.error {
        return (err.clone(), pal::RED);
    }
    if e.incompatible() {
        return (format!("Skyward {} — incompatible version", if e.version.is_empty() { "?" } else { &e.version }), pal::AMBER);
    }
    match &e.info {
        None if e.ping_ms.is_none() => ("Contacting…".into(), pal::TEXT3),
        None => ("Answered, no details".into(), pal::TEXT3),
        Some(i) => {
            let mut s = i.airport.clone();
            if !i.region.is_empty() {
                s = format!("{s} · {}", i.region);
            }
            (s, pal::TEXT2)
        }
    }
}

pub fn draw(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    super::menus::backdrop(&p, screen, 1.0);
    let now = st.mp.now();
    let mx = (screen.width() * 0.05).max(32.0);
    let x = screen.min.x + mx;
    let right = screen.max.x - mx;
    let y0 = super::menus::heading(
        &p,
        Pos2::new(x, screen.min.y + 64.0),
        "Multiplayer",
        "Fly with others on your network. Servers on the LAN appear here by themselves; add any other by address.",
    );
    if crate::platform::WEB {
        let card = Rect::from_min_size(Pos2::new(x, y0 + 16.0), Vec2::new((right - x).min(640.0), 120.0));
        panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
        let g = p.layout(crate::net::WEB_MESSAGE.to_string(), f_ui(15.0), pal::TEXT, card.width() - 48.0);
        p.galley(card.min + Vec2::new(24.0, 24.0), g, pal::TEXT);
        if button(ui, Rect::from_min_size(Pos2::new(x, card.max.y + 24.0), Vec2::new(140.0, 44.0)), "mp_back", "Back", Btn::Secondary, true).clicked() || super::menus::esc(ui) {
            st.actions.push(Action::Goto(Screen::Main));
        }
        return;
    }
    st.browser.open(&st.mp.prefs, now);
    st.browser.tick(now);

    // ---- who you are: callsign + aircraft (top right)
    let ny = screen.min.y + 82.0;
    let aw = 300.0;
    let ax = right - aw;
    let nw = 230.0;
    let nx = ax - 16.0 - nw;
    p.text(Pos2::new(nx, ny - 18.0), Align2::LEFT_TOP, "CALLSIGN", f_inst_b(11.0), pal::TEXT3);
    p.text(Pos2::new(ax, ny - 18.0), Align2::LEFT_TOP, "AIRCRAFT", f_inst_b(11.0), pal::TEXT3);
    let nr = Rect::from_min_size(Pos2::new(nx, ny), Vec2::new(nw, 38.0));
    let mut name = st.mp.prefs.name.clone();
    if field(ui, nr, &mut name, "Your name", false, false).changed() {
        st.mp.prefs.name = skyward_net::codec::sanitize(&name, skyward_net::proto::MAX_NAME);
    }
    let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].clone();
    let ar = Rect::from_min_size(Pos2::new(ax, ny), Vec2::new(aw, 38.0));
    p.rect_filled(ar, sp::R_CTRL, pal::RAISED);
    p.rect_stroke(ar, sp::R_CTRL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    super::map::aircraft_symbol(&p, Pos2::new(ar.min.x + 18.0, ar.center().y), std::f32::consts::FRAC_PI_2, 9.0, pal::ACCENT);
    p.text(Pos2::new(ar.min.x + 34.0, ar.center().y), Align2::LEFT_CENTER, clip(def.name, 24), f_semi(13.5), pal::TEXT);
    let n = st.defs.len();
    for (k, d) in [n - 1, 1].into_iter().enumerate() {
        let b = Rect::from_min_size(Pos2::new(ar.max.x - 66.0 + 32.0 * k as f32, ar.min.y + 4.0), Vec2::new(30.0, 30.0));
        let clicked = button(ui, b, ("mp_ac", k), "", Btn::Ghost, true).clicked();
        chevron(&p, b.center(), if k == 0 { -1.0 } else { 1.0 }, pal::TEXT2);
        if clicked {
            st.plan.aircraft = (st.plan.aircraft + d) % n;
        }
    }

    // ---- toolbar: tabs, search, filters, refresh
    let ty = y0 + 14.0;
    let tabs: Vec<&str> = Tab::ALL.iter().map(|t| t.label()).collect();
    let cur = Tab::ALL.iter().position(|t| *t == st.browser.tab).unwrap_or(0);
    if let Some(i) = segmented(ui, Rect::from_min_size(Pos2::new(x, ty), Vec2::new(380.0, 36.0)), "mp_tabs", &tabs, cur) {
        st.browser.tab = Tab::ALL[i];
    }
    let sr = Rect::from_min_size(Pos2::new(x + 396.0, ty), Vec2::new(240.0, 36.0));
    let mut q = st.browser.search.clone();
    if field(ui, sr, &mut q, "Filter servers", false, false).changed() {
        st.browser.search = q;
    }
    let c1 = Rect::from_min_size(Pos2::new(sr.max.x + 12.0, ty + 3.0), Vec2::new(92.0, 30.0));
    if chip_btn(ui, c1, "mp_full", "Not full", st.browser.hide_full) {
        st.browser.hide_full = !st.browser.hide_full;
    }
    let c2 = Rect::from_min_size(Pos2::new(c1.max.x + 8.0, ty + 3.0), Vec2::new(104.0, 30.0));
    if chip_btn(ui, c2, "mp_open", "No password", st.browser.hide_locked) {
        st.browser.hide_locked = !st.browser.hide_locked;
    }
    let rb = Rect::from_min_size(Pos2::new(right - 120.0, ty), Vec2::new(120.0, 36.0));
    if button(ui, rb, "mp_refresh", "↻  Refresh", Btn::Secondary, true).clicked() {
        st.browser.refresh(now);
    }

    // ---- list (left) and details (right)
    let top = ty + 52.0;
    let bottom = screen.max.y - 92.0;
    let dw = ((right - x) * 0.36).clamp(320.0, 460.0);
    let list_r = Rect::from_min_max(Pos2::new(x, top), Pos2::new(right - dw - 16.0, bottom));
    let det_r = Rect::from_min_max(Pos2::new(right - dw, top), Pos2::new(right, bottom));
    panel_with(&p, list_r, sp::R_PANEL, pal::PANEL_SOLID);
    // header
    let cols = [
        (SortKey::Name, "SERVER", list_r.min.x + 52.0),
        (SortKey::Airport, "AIRPORT", list_r.min.x + list_r.width() * 0.52),
        (SortKey::Players, "PLAYERS", list_r.max.x - 196.0),
        (SortKey::Ping, "PING", list_r.max.x - 100.0),
    ];
    let hy = list_r.min.y + 12.0;
    for (k, label, cx) in cols {
        let g = p.layout_no_wrap(label.to_string(), f_inst_b(11.0), pal::TEXT3);
        let hr = Rect::from_min_size(Pos2::new(cx - 4.0, hy - 4.0), Vec2::new(g.size().x + 24.0, 22.0));
        let resp = ui.interact(hr, ui.id().with(("mp_sort", label)), Sense::click());
        let on = st.browser.sort == k;
        let col = if on {
            pal::ACCENT
        } else if resp.hovered() {
            pal::TEXT2
        } else {
            pal::TEXT3
        };
        p.text(Pos2::new(cx, hy), Align2::LEFT_TOP, label, f_inst_b(11.0), col);
        if on {
            p.text(Pos2::new(cx + g.size().x + 5.0, hy - 1.0), Align2::LEFT_TOP, if st.browser.ascending { "▲" } else { "▼" }, f_ui(9.5), col);
        }
        if resp.clicked() {
            st.browser.sort_by(k);
        }
    }
    p.line_segment([Pos2::new(list_r.min.x + 12.0, hy + 24.0), Pos2::new(list_r.max.x - 12.0, hy + 24.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
    let rows_r = Rect::from_min_max(Pos2::new(list_r.min.x, hy + 30.0), Pos2::new(list_r.max.x, list_r.max.y - 6.0));
    let visible: Vec<Entry> = st.browser.visible(&st.mp.prefs).into_iter().cloned().collect();
    if visible.is_empty() {
        let (t, d) = match st.browser.tab {
            Tab::Favourites => ("No favourite servers yet", "Select a server and press ☆ to keep it here."),
            Tab::Recent => ("No recent servers", "Servers you join appear here."),
            _ if st.browser.scans <= 1 => ("Scanning your network…", "Looking for Skyward servers on every local network (UDP broadcast and mDNS)."),
            _ => ("No servers found", "Start one on any computer here with  skyward-server  — or add an address below."),
        };
        p.text(Pos2::new(rows_r.center().x, rows_r.min.y + 70.0), Align2::CENTER_TOP, t, f_semi(16.0), pal::TEXT2);
        p.text(Pos2::new(rows_r.center().x, rows_r.min.y + 98.0), Align2::CENTER_TOP, d, f_ui(13.0), pal::TEXT3);
        if st.browser.tab == Tab::All || st.browser.tab == Tab::Lan {
            // A gentle radar sweep while scanning.
            let c = Pos2::new(rows_r.center().x, rows_r.min.y + 30.0);
            let a = (st.time * 2.4) % std::f32::consts::TAU;
            super::arc(&p, c, 14.0, a, a + 1.6, 2.0, pal::ACCENT);
        }
    }
    let mut join_key: Option<String> = None;
    // Keyboard / controller: ↑ ↓ (d-pad, stick) select, Enter / A joins, Esc / B goes back, bumpers switch tabs.
    let typing = ui.ctx().egui_wants_keyboard_input();
    let modal = st.browser.password.is_some() || st.mp.status != Status::Idle;
    let nav = if typing || modal { super::scroll::Nav::default() } else { super::scroll::nav(ui, &st.input, st.frame_no) };
    if nav.dy != 0 && !visible.is_empty() {
        let cur = st.browser.selected.as_ref().and_then(|k| visible.iter().position(|e| &e.key == k));
        let i = match cur {
            Some(i) => (i as i32 + nav.dy).clamp(0, visible.len() as i32 - 1) as usize,
            None => 0,
        };
        st.browser.selected = Some(visible[i].key.clone());
    }
    if nav.ok {
        join_key = st.browser.selected.clone();
    }
    if nav.tab != 0 {
        let cur = Tab::ALL.iter().position(|t| *t == st.browser.tab).unwrap_or(0) as i32;
        st.browser.tab = Tab::ALL[(cur + nav.tab).rem_euclid(Tab::ALL.len() as i32) as usize];
    }
    super::scroll::area(ui, rows_r, "mp_list", pal::PANEL_SOLID, |ui| {
        for e in &visible {
            let (row, resp) = ui.allocate_exact_size(Vec2::new(rows_r.width(), ROW_H), Sense::click());
            let p = ui.painter();
            let sel = st.browser.selected.as_deref() == Some(e.key.as_str());
            if sel {
                p.rect_filled(row.shrink2(Vec2::new(8.0, 2.0)), 8, pal::ACCENT_BG);
                p.rect_filled(Rect::from_min_size(row.min + Vec2::new(8.0, 10.0), Vec2::new(3.0, ROW_H - 20.0)), 2, pal::ACCENT);
            } else if resp.hovered() {
                p.rect_filled(row.shrink2(Vec2::new(8.0, 2.0)), 8, Color32::from_white_alpha(7));
            }
            let dim = !e.online() || e.incompatible();
            let tc = if dim { pal::TEXT3 } else { pal::TEXT };
            // Icons: favourite, password, how it was found.
            let fav = st.mp.prefs.is_favourite(&e.key);
            p.text(Pos2::new(row.min.x + 24.0, row.min.y + 12.0), Align2::LEFT_TOP, if fav { "★" } else { "" }, f_ui(13.0), pal::AMBER);
            if e.locked() {
                lock_icon(p, Pos2::new(row.min.x + 29.0, row.min.y + 40.0), pal::TEXT2);
            }
            let name_x = cols[0].2;
            p.text(Pos2::new(name_x, row.min.y + 10.0), Align2::LEFT_TOP, clip(&e.name(), 34), f_semi(15.0), tc);
            let via = match (e.lan, e.mdns) {
                (true, true) => "LAN · mDNS",
                (true, false) => "LAN",
                _ => "",
            };
            let sub = if via.is_empty() { e.key.clone() } else { format!("{}  ·  {via}", e.key) };
            p.text(Pos2::new(name_x, row.min.y + 33.0), Align2::LEFT_TOP, clip(&sub, 44), f_mono(11.5), pal::TEXT3);
            let (sl, sc) = status_line(e);
            p.text(Pos2::new(cols[1].2, row.min.y + 20.0), Align2::LEFT_TOP, clip(&sl, 30), f_ui(13.0), sc);
            if let Some(i) = &e.info {
                let full = i.players >= i.max_players;
                p.text(
                    Pos2::new(cols[2].2, row.min.y + 20.0),
                    Align2::LEFT_TOP,
                    format!("{} / {}", i.players, i.max_players),
                    f_mono(13.0),
                    if full { pal::AMBER } else { pal::TEXT2 },
                );
            }
            ping_label(p, Pos2::new(cols[3].2, row.min.y + 20.0), e.ping_ms.filter(|_| e.error.is_none()));
            if resp.clicked() {
                st.browser.selected = Some(e.key.clone());
            }
            if resp.double_clicked() {
                join_key = Some(e.key.clone());
            }
        }
    });

    // ---- details
    panel_with(&p, det_r, sp::R_PANEL, pal::PANEL_SOLID);
    let sel = st.browser.selected.clone().and_then(|k| st.browser.entry(&k).cloned());
    let dx = det_r.min.x + 22.0;
    let dwid = det_r.width() - 44.0;
    match &sel {
        None => {
            p.text(Pos2::new(det_r.center().x, det_r.min.y + 80.0), Align2::CENTER_TOP, "Select a server", f_semi(15.0), pal::TEXT2);
            p.text(Pos2::new(det_r.center().x, det_r.min.y + 106.0), Align2::CENTER_TOP, "Its message, players and conditions show here.", f_ui(12.5), pal::TEXT3);
        }
        Some(e) => {
            let mut yy = det_r.min.y + 20.0;
            let g = p.layout(e.name(), f_display(22.0), pal::TEXT, dwid);
            let gh = g.size().y;
            p.galley(Pos2::new(dx, yy), g, pal::TEXT);
            yy += gh + 6.0;
            p.text(Pos2::new(dx, yy), Align2::LEFT_TOP, &e.key, f_mono(12.5), pal::TEXT3);
            if e.locked() {
                lock_icon(&p, Pos2::new(det_r.max.x - 30.0, yy + 8.0), pal::TEXT2);
            }
            yy += 24.0;
            let (sl, sc) = status_line(e);
            p.text(Pos2::new(dx, yy), Align2::LEFT_TOP, clip(&sl, 44), f_semi(13.5), sc);
            ping_label(&p, Pos2::new(det_r.max.x - 92.0, yy - 2.0), e.ping_ms.filter(|_| e.error.is_none()));
            yy += 30.0;
            let kv = |p: &egui::Painter, y: f32, k: &str, v: &str| {
                p.text(Pos2::new(dx, y), Align2::LEFT_TOP, k, f_inst_b(11.0), pal::TEXT3);
                p.text(Pos2::new(dx + 104.0, y - 2.0), Align2::LEFT_TOP, v, f_ui(13.0), pal::TEXT2);
            };
            if let Some(i) = &e.info {
                kv(&p, yy, "PLAYERS", &format!("{} / {}{}", i.players, i.max_players, if i.password { "  ·  password" } else { "" }));
                yy += 24.0;
                kv(&p, yy, "CONDITIONS", &clip(&i.conditions, 40));
                yy += 24.0;
            }
            if !e.version.is_empty() {
                kv(&p, yy, "VERSION", &format!("Skyward {}", e.version));
                yy += 24.0;
            }
            if let Some(d) = &e.details {
                if !d.world.metar.is_empty() {
                    kv(&p, yy, "METAR", "");
                    let g = p.layout(d.world.metar.clone(), f_mono(11.5), pal::TEXT2, dwid - 104.0);
                    let gh = g.size().y;
                    p.galley(Pos2::new(dx + 104.0, yy), g, pal::TEXT2);
                    yy += gh + 10.0;
                }
                if !d.motd.is_empty() {
                    yy += 6.0;
                    let g = p.layout(d.motd.clone(), f_ui(13.5), pal::TEXT, dwid - 24.0);
                    let r = Rect::from_min_size(Pos2::new(dx, yy), Vec2::new(dwid, g.size().y + 22.0));
                    p.rect_filled(r, sp::R_CTRL, pal::RAISED);
                    p.rect_filled(Rect::from_min_size(r.min, Vec2::new(3.0, r.height())), 1, pal::ACCENT);
                    p.galley(r.min + Vec2::new(14.0, 11.0), g, pal::TEXT);
                    yy = r.max.y + 12.0;
                }
                p.text(Pos2::new(dx, yy), Align2::LEFT_TOP, format!("ONLINE  {}", d.players.len()), f_inst_b(11.0), pal::TEXT3);
                yy += 20.0;
                for pl in d.players.iter().take(((det_r.max.y - 80.0 - yy) / 22.0).max(0.0) as usize) {
                    let an = st.defs.iter().find(|x| x.id == pl.aircraft).map(|x| x.name).unwrap_or(pl.aircraft.as_str());
                    p.circle_filled(Pos2::new(dx + 4.0, yy + 8.0), 3.0, pal::GREEN);
                    p.text(Pos2::new(dx + 14.0, yy), Align2::LEFT_TOP, clip(&pl.name, 20), f_semi(13.0), pal::TEXT);
                    p.text(Pos2::new(dx + 168.0, yy + 1.0), Align2::LEFT_TOP, clip(an, 20), f_ui(12.0), pal::TEXT2);
                    p.text(Pos2::new(det_r.max.x - 22.0, yy + 1.0), Align2::RIGHT_TOP, format!("{} ms", pl.ping_ms), f_mono(11.5), pal::TEXT3);
                    yy += 22.0;
                }
            }
            // Actions.
            let by = det_r.max.y - 62.0;
            let can = e.online() && !e.incompatible() && !e.full();
            let jr = Rect::from_min_size(Pos2::new(dx, by), Vec2::new(dwid - 112.0, 44.0));
            let label = if e.full() {
                "Server full"
            } else if e.incompatible() {
                "Incompatible"
            } else {
                "Join"
            };
            if button(ui, jr, "mp_join", label, Btn::Primary, can).clicked() {
                join_key = Some(e.key.clone());
            }
            let fav = st.mp.prefs.is_favourite(&e.key);
            let fr = Rect::from_min_size(Pos2::new(jr.max.x + 8.0, by), Vec2::new(48.0, 44.0));
            if button(ui, fr, "mp_fav", if fav { "★" } else { "☆" }, Btn::Secondary, true).clicked() {
                let n = e.name();
                st.mp.prefs.toggle_favourite(&e.key, &n);
                st.mp.prefs.save();
            }
            let xr = Rect::from_min_size(Pos2::new(fr.max.x + 8.0, by), Vec2::new(48.0, 44.0));
            let forget = e.manual && button(ui, xr, "mp_forget", "", Btn::Ghost, true).on_hover_text("Forget this address").clicked();
            if e.manual {
                let c = xr.center();
                p.line_segment([c + Vec2::new(-5.0, -5.0), c + Vec2::new(5.0, 5.0)], Stroke::new(1.8, pal::TEXT2));
                p.line_segment([c + Vec2::new(-5.0, 5.0), c + Vec2::new(5.0, -5.0)], Stroke::new(1.8, pal::TEXT2));
            }
            if forget {
                st.mp.prefs.forget(&e.key);
                st.mp.prefs.save();
                st.browser.entries.retain(|x| x.key != e.key);
                st.browser.selected = None;
            }
        }
    }

    // ---- footer: back, add by address
    let fy = screen.max.y - 72.0;
    if button(ui, Rect::from_min_size(Pos2::new(x, fy), Vec2::new(120.0, 44.0)), "mp_back", "Back", Btn::Secondary, true).clicked() || (nav.back && !modal) {
        st.mp.prefs.save();
        st.actions.push(Action::Goto(Screen::Main));
    }
    p.text(Pos2::new(x + 148.0, fy + 22.0), Align2::LEFT_CENTER, "ADD BY ADDRESS", f_inst_b(11.0), pal::TEXT3);
    let ar = Rect::from_min_size(Pos2::new(x + 270.0, fy + 3.0), Vec2::new(300.0, 38.0));
    let mut t = st.browser.add_text.clone();
    let resp = field(ui, ar, &mut t, "host or IP, e.g. 192.168.1.20:47820", true, false);
    st.browser.add_text = t;
    let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
    if button(ui, Rect::from_min_size(Pos2::new(ar.max.x + 8.0, fy + 3.0), Vec2::new(80.0, 38.0)), "mp_add", "Add", Btn::Secondary, !st.browser.add_text.trim().is_empty())
        .clicked()
        || enter
    {
        st.browser.add_manual(now);
    }
    p.text(
        Pos2::new(right, fy + 22.0),
        Align2::RIGHT_CENTER,
        format!("UDP {} game · {} discovery · mDNS _skyward._udp", skyward_net::DEFAULT_PORT, skyward_net::DISCOVERY_PORT),
        f_ui(11.0),
        pal::TEXT3,
    );

    // ---- joining
    if let Some(k) = join_key {
        if let Some(e) = st.browser.entry(&k).cloned() {
            if e.incompatible() || e.full() {
                st.browser.selected = Some(k);
            } else if e.locked() {
                st.browser.password = Some((k, String::new()));
            } else {
                start_join(st, &k, "");
            }
        }
    }
    password_modal(ui, st, screen);
    status_modal(ui, st, screen);
}

fn start_join(st: &mut UiState, key: &str, password: &str) {
    let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].id;
    let name = if st.mp.prefs.name.trim().is_empty() { "Pilot".to_string() } else { st.mp.prefs.name.clone() };
    st.mp.prefs.save();
    let addr = st.browser.entry(key).and_then(|e| e.sock).map(|s| s.to_string()).unwrap_or_else(|| key.to_string());
    st.mp.join(&addr, password, &name, def);
    // Remember it under the key shown in the list (favourites / recents).
    if let Some(s) = st.mp.server.as_mut() {
        s.addr = key.to_string();
    }
}

fn dim(p: &egui::Painter, screen: Rect) {
    p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(3, 4, 6, 170));
}

fn password_modal(ui: &mut Ui, st: &mut UiState, screen: Rect) {
    let Some((key, mut pw)) = st.browser.password.clone() else { return };
    let p = ui.ctx().layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("mp_pw_layer")));
    dim(&p, screen);
    let card = Rect::from_center_size(screen.center(), Vec2::new(440.0, 214.0));
    let mut ok = false;
    let mut cancel = false;
    egui::Area::new(egui::Id::new("mp_pw")).order(egui::Order::Foreground).fixed_pos(card.min).show(ui.ctx(), |ui| {
        ui.set_min_size(card.size());
        let p = ui.painter().clone();
        panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
        let name = st.browser.entry(&key).map(|e| e.name()).unwrap_or(key.clone());
        p.text(card.min + Vec2::new(26.0, 22.0), Align2::LEFT_TOP, "Password", f_display(22.0), pal::TEXT);
        p.text(card.min + Vec2::new(26.0, 56.0), Align2::LEFT_TOP, format!("{} asks for a password.", clip(&name, 30)), f_ui(13.5), pal::TEXT2);
        let fr = Rect::from_min_size(card.min + Vec2::new(26.0, 88.0), Vec2::new(card.width() - 52.0, 40.0));
        let r = field(ui, fr, &mut pw, "Server password", false, true);
        r.request_focus();
        if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            ok = true;
        }
        let by = card.max.y - 64.0;
        if button(ui, Rect::from_min_size(Pos2::new(card.max.x - 26.0 - 120.0, by), Vec2::new(120.0, 42.0)), "mp_pw_ok", "Join", Btn::Primary, true).clicked() {
            ok = true;
        }
        if button(ui, Rect::from_min_size(Pos2::new(card.max.x - 26.0 - 248.0, by), Vec2::new(120.0, 42.0)), "mp_pw_cancel", "Cancel", Btn::Ghost, true).clicked()
            || super::menus::esc(ui)
        {
            cancel = true;
        }
    });
    if ok {
        st.browser.password = None;
        start_join(st, &key, &pw);
    } else if cancel {
        st.browser.password = None;
    } else {
        st.browser.password = Some((key, pw));
    }
}

/// Connecting… / the reason a join failed.
fn status_modal(ui: &mut Ui, st: &mut UiState, screen: Rect) {
    let server = st.mp.server.as_ref().map(|s| if s.name.is_empty() { s.addr.clone() } else { s.name.clone() }).unwrap_or_default();
    let (title, body, err) = match &st.mp.status {
        Status::Resolving => ("Connecting".to_string(), "Looking up the address…".to_string(), false),
        Status::Connecting => ("Connecting".to_string(), format!("Joining {server}…"), false),
        Status::Connected => (format!("On {server}"), "You are connected. Leave this server to join another.".to_string(), false),
        Status::Failed(m) => ("Could not join".to_string(), m.clone(), true),
        Status::Closed(m) => ("Disconnected".to_string(), m.clone(), true),
        Status::Idle => return,
    };
    let connected = st.mp.connected();
    let p = ui.ctx().layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("mp_status_layer")));
    dim(&p, screen);
    let card = Rect::from_center_size(screen.center(), Vec2::new(480.0, 200.0));
    egui::Area::new(egui::Id::new("mp_status")).order(egui::Order::Foreground).fixed_pos(card.min).show(ui.ctx(), |ui| {
        ui.set_min_size(card.size());
        let p = ui.painter().clone();
        panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
        if err {
            p.rect_filled(Rect::from_min_size(card.min + Vec2::new(0.0, 18.0), Vec2::new(4.0, 40.0)), 2, pal::RED);
        } else if !connected {
            let c = card.min + Vec2::new(card.width() - 44.0, 40.0);
            let a = (st.time * 4.0) % std::f32::consts::TAU;
            super::arc(&p, c, 12.0, a, a + 4.2, 2.5, pal::ACCENT);
        }
        p.text(card.min + Vec2::new(26.0, 22.0), Align2::LEFT_TOP, clip(&title, 34), f_display(22.0), if err { pal::RED } else { pal::TEXT });
        let g = p.layout(body.clone(), f_ui(14.0), pal::TEXT2, card.width() - 52.0);
        p.galley(card.min + Vec2::new(26.0, 62.0), g, pal::TEXT2);
        let br = Rect::from_min_size(Pos2::new(card.max.x - 26.0 - 120.0, card.max.y - 62.0), Vec2::new(120.0, 42.0));
        let esc = super::menus::esc(ui);
        if err {
            if button(ui, br, "mp_status_ok", "OK", Btn::Primary, true).clicked() || esc {
                st.mp.clear_error();
            }
        } else if connected {
            if button(ui, br, "mp_status_fly", "Fly", Btn::Primary, true).clicked() {
                st.actions.push(Action::Goto(Screen::Flying));
            }
            if button(ui, br.translate(Vec2::new(-128.0, 0.0)), "mp_status_leave", "Leave", Btn::Secondary, true).clicked() {
                st.mp.leave();
            }
            if esc {
                st.actions.push(Action::Goto(Screen::Main));
            }
        } else if button(ui, br, "mp_status_cancel", "Cancel", Btn::Secondary, true).clicked() || esc {
            st.mp.leave();
        }
    });
}
