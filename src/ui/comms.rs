//! In-flight comms (atc/): the radio subtitles (who is talking and what
//! they say, accessibility) and the compact comms menu (F4) — the requests
//! that make sense for the unit COM 1 is on, numbered for the keyboard
//! (1–9, 0 = tune), navigable with a controller (next / previous / select)
//! and clickable; a "tune" page lists the stations in range.

use super::{UiState, f_inst, f_inst_b, f_semi, f_ui, pal, sp};
use crate::atc::{Req, freqs::Fac};
use crate::sim::Sim;
use crate::world::World;
use egui::{Align2, Color32, Pos2, Rect, Sense, Stroke, Vec2};

/// Comms menu state (in [`UiState`]).
#[derive(Clone, Debug)]
pub struct CommsUi {
    pub open: bool,
    /// 0 = requests, 1 = tune.
    pub page: usize,
    pub sel: usize,
    /// Digit keys pressed while the menu is open (1..9, 0).
    pub keys: Vec<u8>,
    /// Controller navigation: −1 / +1 steps, select.
    pub nav: i32,
    pub select: bool,
    pub subtitles: bool,
}

impl Default for CommsUi {
    fn default() -> Self {
        CommsUi { open: false, page: 0, sel: 0, keys: Vec::new(), nav: 0, select: false, subtitles: true }
    }
}

enum Item {
    Req(Req, String),
    Tune(usize, bool, String, String),
    Page(usize, String),
    Toggle(u8, String, bool),
}

/// Subtitles + menu. `bottom` = top of the instrument area (subtitles sit above it).
pub fn draw(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim, world: &World, bottom: f32) {
    let screen = ctx.content_rect();
    if st.comms.subtitles {
        subtitles(ctx, sim, screen, bottom);
    }
    if st.comms.open && sim.atc.enabled {
        menu(ctx, st, sim, world, screen);
    } else {
        st.comms.keys.clear();
        st.comms.nav = 0;
        st.comms.select = false;
    }
}

fn subtitles(ctx: &egui::Context, sim: &Sim, screen: Rect, bottom: f32) {
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("comms_subs")));
    let mut lines: Vec<&crate::atc::OnAir> = sim.atc.on_air.iter().filter(|o| o.left > -0.4).collect();
    // ATC first, ATIS after (it loops).
    lines.sort_by_key(|o| o.atis);
    let w = (screen.width() * 0.62).min(860.0);
    let mut y = bottom - sp::M;
    for o in lines.iter().take(2).rev() {
        let fade = ((o.left + 0.4) / 0.4).clamp(0.0, 1.0).min(((o.total - o.left) / 0.15).clamp(0.0, 1.0));
        let text = if o.atis { atis_window(o) } else { o.text.clone() };
        let font = f_ui(17.0);
        let galley = p.layout(text, font, Color32::WHITE, w - 32.0);
        let h = galley.size().y + 34.0;
        let r = Rect::from_min_size(Pos2::new(screen.center().x - w * 0.5, y - h), Vec2::new(w, h));
        let fill = Color32::from_rgba_premultiplied(6, 9, 13, (200.0 * fade) as u8);
        p.rect_filled(r, sp::R_PANEL, fill);
        let who_col = if o.own {
            pal::CYAN
        } else if o.atis {
            pal::TEXT3
        } else {
            pal::GREEN
        };
        let ch = if o.ch == 0 { "COM 1" } else { "COM 2" };
        p.text(r.min + Vec2::new(16.0, 9.0), Align2::LEFT_TOP, o.who.to_uppercase(), f_inst_b(11.5), who_col.gamma_multiply(fade));
        p.text(Pos2::new(r.max.x - 16.0, r.min.y + 9.0), Align2::RIGHT_TOP, ch, f_inst(11.0), pal::TEXT3.gamma_multiply(fade));
        let col = if o.own { Color32::from_rgb(200, 236, 244) } else { pal::TEXT };
        p.galley_with_override_text_color(r.min + Vec2::new(16.0, 27.0), galley, col.gamma_multiply(fade));
        y = r.min.y - sp::S;
    }
}

/// ATIS subtitles: the sentence being read now (the broadcast is long).
fn atis_window(o: &crate::atc::OnAir) -> String {
    let sents: Vec<&str> = o.text.split_inclusive(". ").collect();
    if sents.is_empty() {
        return o.text.clone();
    }
    let frac = (1.0 - o.left / o.total.max(0.1)).clamp(0.0, 0.999);
    // Weight by length.
    let total: usize = sents.iter().map(|s| s.len()).sum();
    let mut acc = 0usize;
    for (i, s) in sents.iter().enumerate() {
        acc += s.len();
        if acc as f32 / total as f32 > frac {
            let next = sents.get(i + 1).copied().unwrap_or("");
            return format!("{}{}", s.trim_end(), if next.is_empty() { String::new() } else { format!(" {}", next.trim_end()) });
        }
    }
    o.text.clone()
}

fn menu(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim, world: &World, screen: Rect) {
    let khz = sim.nav.radios.com[0].active;
    let station = sim.atc.station_on(khz, world, &sim.ac, sim.ground_y);
    let mut items: Vec<Item> = Vec::new();
    if st.comms.page == 0 {
        for (r, l) in crate::atc::menu(sim, world) {
            items.push(Item::Req(r, l));
        }
        items.truncate(9);
        items.push(Item::Page(1, "Tune a station…".into()));
        items.push(Item::Toggle(0, "Read back automatically".into(), sim.atc.auto_readback));
        items.push(Item::Toggle(1, "Assist: tune hand-offs, set squawk".into(), sim.atc.assist));
        items.push(Item::Toggle(2, "Subtitles".into(), st.comms.subtitles));
        items.push(Item::Toggle(3, format!("COM 2 receive · {}", crate::atc::phrase::freq_text(sim.nav.radios.com[1].active)), sim.atc.com2_rx));
    } else {
        for (i, _) in sim.atc.tune_list(world, &sim.ac, sim.ground_y).into_iter().take(9) {
            let s = &sim.atc.stations[i];
            let com2 = s.fac == Fac::Atis;
            items.push(Item::Tune(i, com2, s.text.clone(), format!("{} · {}", crate::atc::phrase::freq_text(s.khz), if com2 { "COM 2" } else { "COM 1" })));
        }
        items.push(Item::Page(0, "Back".into()));
    }
    // Keyboard digits: 1..9 = the numbered rows, 0 = the page row.
    let mut chosen: Option<usize> = None;
    let page_row = items.iter().position(|i| matches!(i, Item::Page(..)));
    for k in std::mem::take(&mut st.comms.keys) {
        if k == 0 {
            chosen = page_row;
        } else if (k as usize) <= items.len() && !matches!(items[k as usize - 1], Item::Page(..) | Item::Toggle(..)) {
            chosen = Some(k as usize - 1);
        }
    }
    if st.comms.nav != 0 && !items.is_empty() {
        st.comms.sel = (st.comms.sel as i32 + st.comms.nav).rem_euclid(items.len() as i32) as usize;
        st.comms.nav = 0;
    }
    st.comms.sel = st.comms.sel.min(items.len().saturating_sub(1));
    if std::mem::take(&mut st.comms.select) {
        chosen = Some(st.comms.sel);
    }
    // Layout.
    let w = 380.0f32.min(screen.width() - 32.0);
    let row = 30.0;
    let log_n = sim.atc.log.len().min(3);
    let h = 74.0 + items.len() as f32 * row + if log_n > 0 { 16.0 + log_n as f32 * 18.0 } else { 0.0 } + 12.0;
    // Right-hand side, below the corner map.
    let top = if st.map.mode == super::flight::MapMode::Corner { 330.0 } else { screen.height() * 0.16 };
    let at = Pos2::new(screen.max.x - sp::L - w, (screen.min.y + top).min(screen.max.y - h - 120.0).max(screen.min.y + sp::L));
    let area = egui::Area::new(egui::Id::new("comms_menu")).fixed_pos(at).order(egui::Order::Foreground).show(ctx, |ui| {
        let (r, _) = ui.allocate_exact_size(Vec2::new(w, h), Sense::hover());
        let p = ui.painter().clone();
        super::panel_with(&p, r, sp::R_CARD, pal::PANEL_SOLID);
        // Header: COM 1, the unit, signal.
        let x0 = r.min.x + 16.0;
        p.text(Pos2::new(x0, r.min.y + 14.0), Align2::LEFT_TOP, "COMMS", f_inst_b(11.0), pal::TEXT3);
        p.text(Pos2::new(r.max.x - 16.0, r.min.y + 14.0), Align2::RIGHT_TOP, "F4 close", f_inst(11.0), pal::TEXT3);
        p.text(Pos2::new(x0, r.min.y + 32.0), Align2::LEFT_TOP, format!("COM 1  {:.3}", khz as f32 / 1000.0), super::f_digits_b(17.0), pal::GREEN);
        let (name, col) = match &station {
            Some((s, q)) => {
                // Signal bars.
                let bx = r.max.x - 16.0 - 4.0 * 6.0;
                for k in 0..4 {
                    let hh = 4.0 + 3.0 * k as f32;
                    let on = *q * 4.0 > k as f32 + 0.2;
                    let br = Rect::from_min_max(Pos2::new(bx + k as f32 * 6.0, r.min.y + 48.0 - hh), Pos2::new(bx + k as f32 * 6.0 + 4.0, r.min.y + 48.0));
                    p.rect_filled(br, 1.0, if on { pal::GREEN } else { pal::RAISED_HI });
                }
                (s.text.clone(), pal::TEXT)
            }
            None if !sim.nav.radios.powered => ("no power".into(), pal::AMBER),
            None => ("no station in range".into(), pal::AMBER),
        };
        p.text(Pos2::new(x0 + 172.0, r.min.y + 34.0), Align2::LEFT_TOP, name, f_semi(14.0), col);
        let mut y = r.min.y + 62.0;
        p.line_segment([Pos2::new(r.min.x + 12.0, y - 4.0), Pos2::new(r.max.x - 12.0, y - 4.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
        let mut n = 0;
        for (i, it) in items.iter().enumerate() {
            let rr = Rect::from_min_size(Pos2::new(r.min.x + 8.0, y), Vec2::new(w - 16.0, row - 2.0));
            let resp = ui.interact(rr, ui.id().with(("comms_row", st.comms.page, i)), Sense::click());
            let sel = i == st.comms.sel;
            if resp.hovered() || sel {
                p.rect_filled(rr, sp::R_CTRL, if sel { pal::RAISED_HI } else { pal::RAISED });
            }
            if resp.clicked() {
                chosen = Some(i);
            }
            let key = match it {
                Item::Page(..) => "0".to_string(),
                Item::Toggle(..) => String::new(),
                _ => {
                    n += 1;
                    if n <= 9 { n.to_string() } else { String::new() }
                }
            };
            p.text(Pos2::new(rr.min.x + 12.0, rr.center().y), Align2::LEFT_CENTER, &key, super::f_digits_b(14.0), pal::CYAN);
            let (label, sub, colr) = match it {
                Item::Req(r, l) => (l.clone(), String::new(), if matches!(r, Req::Mayday | Req::PanPan) { pal::AMBER } else { pal::TEXT }),
                Item::Tune(_, _, l, f) => (l.clone(), f.clone(), pal::TEXT),
                Item::Page(_, l) => (l.clone(), String::new(), pal::TEXT2),
                Item::Toggle(_, l, on) => (l.clone(), if *on { "ON".into() } else { "OFF".into() }, pal::TEXT2),
            };
            p.text(Pos2::new(rr.min.x + 34.0, rr.center().y), Align2::LEFT_CENTER, label, f_ui(14.0), colr);
            if !sub.is_empty() {
                let c = if sub == "ON" {
                    pal::GREEN
                } else if sub == "OFF" {
                    pal::TEXT3
                } else {
                    pal::CYAN
                };
                p.text(Pos2::new(rr.max.x - 10.0, rr.center().y), Align2::RIGHT_CENTER, sub, f_inst(12.0), c);
            }
            y += row;
        }
        if log_n > 0 {
            y += 8.0;
            p.line_segment([Pos2::new(r.min.x + 12.0, y - 4.0), Pos2::new(r.max.x - 12.0, y - 4.0)], Stroke::new(1.0, pal::HAIRLINE_HI));
            for l in sim.atc.log.iter().rev().take(log_n).collect::<Vec<_>>().into_iter().rev() {
                let t = format!("{}: {}", l.who, l.text);
                let t = if t.chars().count() > 58 { format!("{}…", t.chars().take(57).collect::<String>()) } else { t };
                p.text(Pos2::new(x0, y), Align2::LEFT_TOP, t, f_ui(11.5), if l.own { pal::CYAN.gamma_multiply(0.8) } else { pal::TEXT3 });
                y += 18.0;
            }
        }
    });
    let _ = area;
    if let Some(i) = chosen {
        match items.get(i) {
            Some(Item::Req(r, _)) => {
                crate::atc::request(sim, world, r.clone());
                st.comms.open = false;
                st.comms.sel = 0;
            }
            Some(Item::Tune(ix, com2, _, _)) => {
                crate::atc::request(sim, world, Req::Tune(*ix, *com2));
                st.comms.page = 0;
                st.comms.sel = 0;
            }
            Some(Item::Page(pg, _)) => {
                st.comms.page = *pg;
                st.comms.sel = 0;
            }
            Some(Item::Toggle(k, _, _)) => match k {
                0 => sim.atc.auto_readback = !sim.atc.auto_readback,
                1 => sim.atc.assist = !sim.atc.assist,
                2 => st.comms.subtitles = !st.comms.subtitles,
                _ => sim.atc.com2_rx = !sim.atc.com2_rx,
            },
            None => {}
        }
    }
}
