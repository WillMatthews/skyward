//! In-flight "Radios & flight plan" panel (F3): COM / NAV / ADF active and
//! standby frequencies with swap and tuning, NAV course (OBS), ident audio
//! and what each receiver hears; the transponder; and the FMS-lite flight
//! plan — origin, destination and approach (ILS / RNAV, auto-tuning NAV 1),
//! waypoints with direct-to / delete / insert, and distance / ETA / fuel
//! predictions (nav/).

use super::menus::{Btn, button, chip_btn, segmented};
use super::{UiState, f_digits_b, f_display, f_inst, f_inst_b, f_semi, f_ui, pal, sp};
use crate::nav::db::{self, freq_label};
use crate::nav::fms::{ApprKind, Wpt, WptKind};
use crate::nav::geo::{self, NM};
use crate::nav::radio::{self, XpdrMode};
use crate::nav::state::WorldEnv;
use crate::sim::Sim;
use crate::world::World;
use egui::{Align2, Color32, Pos2, Rect, Sense, Stroke, Ui, Vec2};

/// Panel state kept in [`UiState`].
#[derive(Clone, Debug, Default)]
pub struct RadioUi {
    pub open: bool,
    pub tab: usize,
    pub wpt_text: String,
    pub dest_text: String,
    pub rwy: String,
    pub rnav: bool,
    /// Last result line (text, error).
    pub msg: Option<(String, bool)>,
    /// Map: clicks add / drags move waypoints.
    pub edit_route: bool,
    /// Map: waypoint being dragged.
    pub drag: Option<usize>,
}

pub fn panel(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim, world: &World) {
    if !st.radio.open {
        return;
    }
    let screen = ctx.content_rect();
    let w = 520.0f32.min(screen.width() - 32.0);
    let h = (screen.height() * 0.84).min(780.0);
    let at = Pos2::new(sp::L, screen.center().y);
    egui::Area::new(egui::Id::new("radio_panel")).pivot(Align2::LEFT_CENTER).fixed_pos(at).order(egui::Order::Foreground).show(ctx, |ui| {
        egui::Frame::new()
            .fill(pal::PANEL_SOLID)
            .corner_radius(sp::R_CARD)
            .stroke(Stroke::new(1.0, pal::HAIRLINE_HI))
            .shadow(egui::epaint::Shadow { offset: [0, 10], blur: 32, spread: 0, color: Color32::from_black_alpha(120) })
            .inner_margin(egui::Margin::same(20))
            .show(ui, |ui| {
                ui.set_width(w - 40.0);
                ui.set_max_height(h - 40.0);
                header(ui, st, sim);
                ui.add_space(sp::S);
                let r = ui.allocate_exact_size(Vec2::new(w - 40.0, 36.0), Sense::hover()).0;
                if let Some(t) = segmented(ui, r, "radio_tabs", &["Radios", "Flight plan"], st.radio.tab) {
                    st.radio.tab = t;
                }
                ui.add_space(sp::M);
                egui::ScrollArea::vertical().auto_shrink([false, false]).max_height(h - 170.0).show(ui, |ui| match st.radio.tab {
                    0 => radios_tab(ui, sim),
                    _ => plan_tab(ui, st, sim, world),
                });
            });
    });
}

fn header(ui: &mut Ui, st: &mut UiState, sim: &Sim) {
    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new("Radios & flight plan").font(f_display(22.0)).color(pal::TEXT));
            let sub = if !sim.nav.equipped {
                format!("{} · no navigation radios", sim.def().name)
            } else if !sim.nav.radios.powered {
                format!("{} · avionics unpowered", sim.def().name)
            } else {
                format!("{} · {}", sim.def().name, db::status())
            };
            ui.label(egui::RichText::new(sub).font(f_ui(12.5)).color(if sim.nav.radios.powered { pal::TEXT3 } else { pal::AMBER }));
        });
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
            let (r, _) = ui.allocate_exact_size(Vec2::new(32.0, 32.0), Sense::hover());
            let resp = button(ui, r, "radio_close", "", Btn::Ghost, true);
            let c = r.center();
            let k = r.height() * 0.18;
            let s = Stroke::new(1.6, if resp.hovered() { pal::TEXT } else { pal::TEXT2 });
            ui.painter().line_segment([c + Vec2::new(-k, -k), c + Vec2::new(k, k)], s);
            ui.painter().line_segment([c + Vec2::new(k, -k), c + Vec2::new(-k, k)], s);
            if resp.clicked() {
                st.radio.open = false;
            }
        });
    });
}

fn section(ui: &mut Ui, title: &str) {
    ui.add_space(sp::S);
    ui.label(egui::RichText::new(title.to_uppercase()).font(f_inst_b(11.0)).color(pal::TEXT3));
    ui.add_space(2.0);
}

/// A small design-system button at the cursor.
fn small(ui: &mut Ui, id: impl std::hash::Hash + std::fmt::Debug, text: &str, w: f32) -> bool {
    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 26.0), Sense::hover());
    button(ui, r, id, text, Btn::Secondary, true).clicked()
}

/// Swap arrows button (drawn: the fonts have no ⇄).
fn swap_btn(ui: &mut Ui, id: impl std::hash::Hash + std::fmt::Debug) -> bool {
    let (r, _) = ui.allocate_exact_size(Vec2::new(40.0, 30.0), Sense::hover());
    let resp = button(ui, r, id, "", Btn::Secondary, true);
    let p = ui.painter();
    let c = r.center();
    let col = if resp.hovered() { pal::TEXT } else { pal::TEXT2 };
    let s = Stroke::new(1.6, col);
    for (dy, dir) in [(-4.0f32, 1.0f32), (4.0, -1.0)] {
        let a = c + Vec2::new(-8.0 * dir, dy);
        let b = c + Vec2::new(8.0 * dir, dy);
        p.line_segment([a, b], s);
        p.line_segment([b, b + Vec2::new(-4.0 * dir, -3.0)], s);
        p.line_segment([b, b + Vec2::new(-4.0 * dir, 3.0)], s);
    }
    resp.clicked()
}

/// One radio: label, active (green), swap, standby (cyan) with MHz / channel steps.
/// Returns (swap, Δ MHz, Δ channels).
fn freq_row(ui: &mut Ui, id: &str, label: &str, active: &str, standby: &str) -> (bool, i32, i32) {
    let mut out = (false, 0, 0);
    let w = ui.available_width();
    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 48.0), Sense::hover());
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, pal::RAISED);
    p.text(r.min + Vec2::new(12.0, 8.0), Align2::LEFT_TOP, label, f_inst_b(11.0), pal::TEXT3);
    p.text(r.min + Vec2::new(12.0, 22.0), Align2::LEFT_TOP, active, f_digits_b(20.0), pal::GREEN);
    let mut child = ui.new_child(
        egui::UiBuilder::new().max_rect(Rect::from_min_max(r.min + Vec2::new(116.0, 9.0), r.max - Vec2::new(8.0, 9.0))).layout(egui::Layout::left_to_right(egui::Align::Center)),
    );
    if swap_btn(&mut child, (id, "swap")) {
        out.0 = true;
    }
    child.add_space(10.0);
    let (sr, _) = child.allocate_exact_size(Vec2::new(92.0, 30.0), Sense::hover());
    p.text(sr.left_center(), Align2::LEFT_CENTER, standby, f_digits_b(18.0), pal::CYAN);
    for (k, (t, d)) in [("−", (-1, 0)), ("+", (1, 0)), ("‹", (0, -1)), ("›", (0, 1))].into_iter().enumerate() {
        if k == 2 {
            child.add_space(4.0);
        }
        if small(&mut child, (id, k), t, 30.0) {
            out.1 = d.0;
            out.2 = d.1;
        }
    }
    out
}

fn status(ui: &mut Ui, text: String, col: Color32) {
    ui.add_space(2.0);
    ui.label(egui::RichText::new(text).font(f_inst(11.5)).color(col));
    ui.add_space(4.0);
}

fn radios_tab(ui: &mut Ui, sim: &mut Sim) {
    let r = &mut sim.nav.radios;
    section(ui, "Communication");
    for k in 0..2 {
        let c = r.com[k];
        let (sw, dm, dc) = freq_row(ui, &format!("com{k}"), &format!("COM {}", k + 1), &format!("{:.3}", c.active as f32 / 1000.0), &format!("{:.3}", c.standby as f32 / 1000.0));
        let c = &mut r.com[k];
        if sw {
            std::mem::swap(&mut c.active, &mut c.standby);
        }
        if dm != 0 || dc != 0 {
            c.standby = radio::step_freq(c.standby, dm, dc, radio::COM_MIN, radio::COM_MAX, 25);
        }
        ui.add_space(4.0);
    }
    section(ui, "Navigation");
    for k in 0..2 {
        let (sw, dm, dc) = {
            let n = &r.nav[k];
            freq_row(ui, &format!("nav{k}"), &format!("NAV {}", k + 1), &freq_label(n.active, false), &freq_label(n.standby, false))
        };
        let n = &mut r.nav[k];
        if sw {
            n.swap();
        }
        if dm != 0 || dc != 0 {
            n.standby = radio::step_freq(n.standby, dm, dc, radio::NAV_MIN, radio::NAV_MAX, 50);
        }
        // What it hears, course, ident.
        let o = &n.out;
        let (txt, col) = match n.station() {
            Some(s) if o.loc => {
                let gs = if o.gs_valid { "G/S" } else { "no G/S" };
                let dme = o.dme.map(|d| format!(" · {:.1} NM", d / NM)).unwrap_or_default();
                (
                    format!(
                        "{} · {} {} {}{} · {gs}{}",
                        s.ident,
                        s.kind.label(),
                        s.airport,
                        o.runway,
                        if s.ils.as_ref().is_some_and(|i| i.synthetic) { " (synthetic)" } else { "" },
                        dme
                    ),
                    pal::GREEN,
                )
            }
            Some(s) => {
                let dme = o.dme.map(|d| format!(" · {:.1} NM", d / NM)).unwrap_or_default();
                let tf = match o.to {
                    Some(true) => "TO",
                    Some(false) => "FROM",
                    None => "—",
                };
                (format!("{} {} · {} · radial {:03.0}° {tf}{dme}", s.ident, s.name, s.kind.label(), o.radial.to_degrees().round().rem_euclid(360.0)), pal::GREEN)
            }
            None if !r.powered => ("no power".into(), pal::AMBER),
            None => ("no signal".into(), pal::AMBER),
        };
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.label(egui::RichText::new(txt).font(f_inst(11.5)).color(col));
        });
        let n = &mut r.nav[k];
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.label(egui::RichText::new("CRS").font(f_inst_b(11.0)).color(pal::TEXT3));
            if small(ui, ("crs-", k), "−", 30.0) {
                n.obs = geo::wrap_2pi(n.obs.to_degrees().round().to_radians() - 1f32.to_radians());
            }
            ui.add_sized(
                [52.0, 26.0],
                egui::Label::new(egui::RichText::new(format!("{:03.0}°", n.obs.to_degrees().round().rem_euclid(360.0))).font(f_digits_b(15.0)).color(pal::CYAN)),
            );
            if small(ui, ("crs+", k), "+", 30.0) {
                n.obs = geo::wrap_2pi(n.obs.to_degrees().round().to_radians() + 1f32.to_radians());
            }
            ui.add_space(8.0);
            let (cr, _) = ui.allocate_exact_size(Vec2::new(72.0, 26.0), Sense::hover());
            if chip_btn(ui, cr, ("ident", k), "IDENT", n.ident_audio) {
                n.ident_audio = !n.ident_audio;
            }
        });
        ui.add_space(6.0);
    }
    let auto = &mut sim.nav.auto_tune;
    ui.horizontal(|ui| {
        ui.add_space(12.0);
        let (cr, _) = ui.allocate_exact_size(Vec2::new(176.0, 26.0), Sense::hover());
        if chip_btn(ui, cr, "autotune", "Auto-tune ILS (NAV 1)", *auto) {
            *auto = !*auto;
        }
    });
    let r = &mut sim.nav.radios;
    section(ui, "ADF");
    let (sw, dm, dc) = freq_row(ui, "adf", "ADF", &r.adf.active.to_string(), &r.adf.standby.to_string());
    if sw {
        std::mem::swap(&mut r.adf.active, &mut r.adf.standby);
    }
    if dm != 0 || dc != 0 {
        // ADF: 100 kHz / 1 kHz steps.
        r.adf.standby = (r.adf.standby as i32 + dm * 100 + dc).clamp(radio::ADF_MIN as i32, radio::ADF_MAX as i32) as u32;
    }
    let a = &r.adf.out;
    if a.valid {
        status(ui, format!("    {} · bearing {:03.0}°", a.ident, a.bearing.to_degrees().round().rem_euclid(360.0)), pal::GREEN);
    } else {
        status(ui, "    no signal".into(), pal::AMBER);
    }
    ui.horizontal(|ui| {
        ui.add_space(12.0);
        let (cr, _) = ui.allocate_exact_size(Vec2::new(72.0, 26.0), Sense::hover());
        if chip_btn(ui, cr, "adf_ident", "IDENT", r.adf.ident_audio) {
            r.adf.ident_audio = !r.adf.ident_audio;
        }
    });
    section(ui, "Transponder");
    let x = &mut r.xpdr;
    ui.horizontal(|ui| {
        ui.add_space(12.0);
        for d in 0..4 {
            let (dr, _) = ui.allocate_exact_size(Vec2::new(34.0, 40.0), Sense::hover());
            let resp = ui.interact(dr, ui.id().with(("sq", d)), Sense::click());
            let p = ui.painter();
            p.rect_filled(dr, sp::R_CTRL, if resp.hovered() { pal::RAISED_HI } else { pal::RAISED });
            p.text(dr.center(), Align2::CENTER_CENTER, x.code[d].to_string(), f_digits_b(22.0), if x.mode == XpdrMode::Off { pal::TEXT3 } else { pal::GREEN });
            if resp.clicked() {
                x.code[d] = (x.code[d] + 1) % 8;
            }
            if resp.secondary_clicked() {
                x.code[d] = (x.code[d] + 7) % 8;
            }
        }
        ui.add_space(10.0);
        let (sr, _) = ui.allocate_exact_size(Vec2::new(220.0, 34.0), Sense::hover());
        let modes = [XpdrMode::Off, XpdrMode::Stby, XpdrMode::On, XpdrMode::Alt];
        let sel = modes.iter().position(|m| *m == x.mode).unwrap_or(1);
        if let Some(i) = segmented(ui, sr, "xpdr_mode", &["OFF", "STBY", "ON", "ALT"], sel) {
            x.mode = modes[i];
        }
        ui.add_space(6.0);
        let (ir, _) = ui.allocate_exact_size(Vec2::new(64.0, 34.0), Sense::hover());
        if button(ui, ir, "xpdr_ident", if x.ident > 0.0 { "IDENT" } else { "IDT" }, if x.ident > 0.0 { Btn::Primary } else { Btn::Secondary }, x.mode != XpdrMode::Off).clicked() {
            x.ident = 18.0;
        }
    });
    status(ui, "    click a digit +1, right-click −1 · 7700 emergency · 7600 radio failure".into(), pal::TEXT3);
}

/// Resolve a waypoint entry: navaid ident, airport code or "lat,lon".
fn resolve(text: &str, sim: &Sim, world: &World) -> Option<Wpt> {
    let t = text.trim();
    if t.is_empty() {
        return None;
    }
    if let Some((a, b)) = t.split_once(',') {
        if let (Ok(lat), Ok(lon)) = (a.trim().parse::<f64>(), b.trim().parse::<f64>()) {
            return Some(Wpt::new(format!("{:.0}N{:.0}", lat.abs() * 10.0, lon.abs() * 10.0), (lat, lon), WptKind::User));
        }
    }
    let here = world.proj.to_latlon(glam::vec2(sim.ac.pos.x, sim.ac.pos.z));
    if let Some(n) = db::db().and_then(|d| d.find(t, here).cloned()) {
        return Some(Wpt::new(n.ident.clone(), n.ll(), WptKind::Navaid));
    }
    let a = world.airport_db.as_ref()?.get(t)?;
    Some(Wpt::new(a.code(), (a.lat, a.lon), WptKind::Airport))
}

fn plan_tab(ui: &mut Ui, st: &mut UiState, sim: &mut Sim, world: &World) {
    let env = WorldEnv(world);
    let ru = &mut st.radio;
    section(ui, "Route");
    let w = ui.available_width();
    // origin / destination
    ui.horizontal(|ui| {
        let o = sim.nav.plan.origin.as_ref().map(|o| o.ident.clone()).unwrap_or_else(|| "----".into());
        ui.label(egui::RichText::new("FROM").font(f_inst_b(11.0)).color(pal::TEXT3));
        ui.label(egui::RichText::new(o).font(f_digits_b(16.0)).color(pal::GREEN));
        ui.add_space(14.0);
        ui.label(egui::RichText::new("TO").font(f_inst_b(11.0)).color(pal::TEXT3));
        let (tr, _) = ui.allocate_exact_size(Vec2::new(82.0, 30.0), Sense::hover());
        ui.painter().rect_filled(tr, sp::R_CTRL, pal::RAISED);
        if ru.dest_text.is_empty() {
            if let Some(d) = &sim.nav.plan.dest {
                ru.dest_text = d.ident.clone();
            }
        }
        ui.put(
            tr.shrink2(Vec2::new(8.0, 5.0)),
            egui::TextEdit::singleline(&mut ru.dest_text).hint_text("ICAO").frame(egui::Frame::NONE).font(f_digits_b(15.0)).text_color(pal::CYAN),
        );
        ui.label(egui::RichText::new("RWY").font(f_inst_b(11.0)).color(pal::TEXT3));
        let (rr, _) = ui.allocate_exact_size(Vec2::new(56.0, 30.0), Sense::hover());
        ui.painter().rect_filled(rr, sp::R_CTRL, pal::RAISED);
        ui.put(rr.shrink2(Vec2::new(8.0, 5.0)), egui::TextEdit::singleline(&mut ru.rwy).hint_text("27R").frame(egui::Frame::NONE).font(f_digits_b(15.0)).text_color(pal::CYAN));
        let (kr, _) = ui.allocate_exact_size(Vec2::new(110.0, 30.0), Sense::hover());
        if let Some(i) = segmented(ui, kr, "appr_kind", &["ILS", "RNAV"], ru.rnav as usize) {
            ru.rnav = i == 1;
        }
    });
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        let (br, _) = ui.allocate_exact_size(Vec2::new(150.0, 32.0), Sense::hover());
        if button(ui, br, "set_appr", "Set approach", Btn::Primary, !ru.dest_text.trim().is_empty()).clicked() {
            let res = match world.airport_db.as_ref() {
                Some(adb) => {
                    let ap = ru.dest_text.trim().to_ascii_uppercase();
                    let rwy = if ru.rwy.trim().is_empty() {
                        // Into wind: the end with the most headwind.
                        adb.get(&ap)
                            .and_then(|a| {
                                a.runways.iter().flat_map(|r| (0..2).filter_map(move |e| r.heading(e).map(|h| (r.ends[e].ident.clone(), h)))).max_by(|x, y| {
                                    let hw = |h: f32| crate::airports::wind_components(h, world.wx.wind_from, world.wx.wind_kt).0;
                                    hw(x.1).total_cmp(&hw(y.1))
                                })
                            })
                            .map(|x| x.0)
                            .unwrap_or_default()
                    } else {
                        ru.rwy.trim().to_ascii_uppercase()
                    };
                    sim.nav.plan_approach(adb, &ap, &rwy, if ru.rnav { ApprKind::Rnav } else { ApprKind::Ils })
                }
                None => Err("airport database not loaded".into()),
            };
            ru.msg = Some(match res {
                Ok(m) => (format!("{m} — added"), false),
                Err(e) => (e, true),
            });
        }
        let (cr, _) = ui.allocate_exact_size(Vec2::new(120.0, 32.0), Sense::hover());
        if button(ui, cr, "clear_plan", "Clear route", Btn::Ghost, !sim.nav.plan.legs.is_empty()).clicked() {
            let origin = sim.nav.plan.origin.clone();
            let cruise = sim.nav.plan.cruise;
            sim.nav.plan = Default::default();
            sim.nav.plan.origin = origin;
            sim.nav.plan.cruise = cruise;
        }
        let (er, _) = ui.allocate_exact_size(Vec2::new(150.0, 32.0), Sense::hover());
        if chip_btn(ui, er, "edit_route", "Edit on the map", ru.edit_route) {
            ru.edit_route = !ru.edit_route;
        }
    });
    if st.radio.edit_route {
        st.map.mode = super::flight::MapMode::Large;
    }
    let ru = &mut st.radio;
    if let Some((m, err)) = &ru.msg {
        status(ui, m.clone(), if *err { pal::AMBER } else { pal::GREEN });
    }
    if ru.edit_route {
        status(ui, "Map: click to add a waypoint · drag one to move it · right-click to delete".into(), pal::TEXT3);
    }
    // insert
    ui.horizontal(|ui| {
        let (tr, _) = ui.allocate_exact_size(Vec2::new(w - 240.0, 32.0), Sense::hover());
        ui.painter().rect_filled(tr, sp::R_CTRL, pal::RAISED);
        let te = ui.put(
            tr.shrink2(Vec2::new(10.0, 6.0)),
            egui::TextEdit::singleline(&mut ru.wpt_text).hint_text("Navaid, airport or lat,lon").frame(egui::Frame::NONE).font(f_ui(14.0)).text_color(pal::TEXT),
        );
        let enter = te.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        let (br, _) = ui.allocate_exact_size(Vec2::new(110.0, 32.0), Sense::hover());
        if button(ui, br, "insert_wpt", "Add waypoint", Btn::Secondary, !ru.wpt_text.trim().is_empty()).clicked() || enter {
            match resolve(&ru.wpt_text, sim, world) {
                Some(wp) => {
                    let n = sim.nav.plan.enroute_len();
                    ru.msg = Some((format!("{} added", wp.ident), false));
                    sim.nav.plan.insert(n, wp);
                    ru.wpt_text.clear();
                }
                None => ru.msg = Some((format!("{}: not found", ru.wpt_text.trim()), true)),
            }
        }
        let (cr, _) = ui.allocate_exact_size(Vec2::new(110.0, 32.0), Sense::hover());
        let fl = (sim.nav.plan.cruise / 0.3048 / 100.0).round() as i32;
        let resp = button(ui, cr, "cruise", &format!("CRZ FL{fl:03}"), Btn::Ghost, true);
        if resp.clicked() {
            sim.nav.plan.cruise += 1000.0 * 0.3048;
        }
        if resp.secondary_clicked() {
            sim.nav.plan.cruise = (sim.nav.plan.cruise - 1000.0 * 0.3048).max(1000.0 * 0.3048);
        }
    });
    // legs with predictions
    section(ui, "Flight plan");
    let p = glam::vec2(sim.ac.pos.x, sim.ac.pos.z);
    let gs = glam::vec2(sim.ac.vel.x, sim.ac.vel.z).length();
    let (fob, flow) = crate::nav::state::fuel(&sim.ac);
    let preds = sim.nav.plan.predictions(&env, p, gs, fob, flow);
    let hours = sim.hours;
    let active = sim.nav.plan.active;
    if preds.is_empty() {
        status(ui, "No waypoints. Add navaids / airports above, set an approach, or edit on the map (M → large).".into(), pal::TEXT3);
    }
    let mut dir = None;
    let mut del = None;
    for (k, pr) in preds.iter().enumerate() {
        let i = active + k;
        let (r, _) = ui.allocate_exact_size(Vec2::new(w, 34.0), Sense::hover());
        let pt = ui.painter().clone();
        if k == 0 {
            pt.rect_filled(r, sp::R_CTRL, pal::RAISED);
        }
        let col = if k == 0 { pal::MAGENTA } else { pal::GREEN };
        pt.text(r.min + Vec2::new(10.0, 17.0), Align2::LEFT_CENTER, &pr.ident, f_digits_b(14.0), col);
        pt.text(r.min + Vec2::new(98.0, 17.0), Align2::LEFT_CENTER, format!("{:03.0}°", pr.course.to_degrees().round().rem_euclid(360.0)), f_digits_b(13.0), pal::TEXT2);
        pt.text(r.min + Vec2::new(150.0, 17.0), Align2::LEFT_CENTER, format!("{:5.1}", pr.dist / NM), f_digits_b(13.0), pal::TEXT);
        let t = (hours + pr.eta / 3600.0).rem_euclid(24.0);
        pt.text(r.min + Vec2::new(205.0, 17.0), Align2::LEFT_CENTER, format!("{:02}:{:02}", t as i32, ((t.fract()) * 60.0) as i32), f_digits_b(13.0), pal::TEXT);
        if let Some(a) = pr.alt {
            pt.text(r.min + Vec2::new(258.0, 17.0), Align2::LEFT_CENTER, format!("{:.0}", (a / 0.3048 / 10.0).round() * 10.0), f_digits_b(13.0), pal::MAGENTA);
        }
        if let Some(f) = pr.efob {
            pt.text(r.min + Vec2::new(318.0, 17.0), Align2::LEFT_CENTER, format!("{:.1}t", f / 1000.0), f_digits_b(13.0), if f < 500.0 { pal::AMBER } else { pal::TEXT2 });
        }
        let b1 = Rect::from_min_size(Pos2::new(r.max.x - 92.0, r.min.y + 4.0), Vec2::new(48.0, 26.0));
        let b2 = Rect::from_min_size(Pos2::new(r.max.x - 40.0, r.min.y + 4.0), Vec2::new(36.0, 26.0));
        if button(ui, b1, ("dir", i), "DIR", Btn::Ghost, true).clicked() {
            dir = Some(i);
        }
        if button(ui, b2, ("del", i), "DEL", Btn::Ghost, true).clicked() {
            del = Some(i);
        }
    }
    if let Some(i) = dir {
        let here = world.proj.to_latlon(p);
        sim.nav.plan.direct_to(i, here);
    }
    if let Some(i) = del {
        sim.nav.plan.remove(i);
    }
    if let Some(l) = preds.last() {
        let t = l.eta;
        let fuel = l.efob.map(|f| format!(" · fuel at destination {:.1} t", f / 1000.0)).unwrap_or_default();
        status(
            ui,
            format!("{:.0} NM to go · {:.0} h {:02.0} min at {:.0} kt{fuel}", l.dist / NM, (t / 3600.0).floor(), (t / 60.0) % 60.0, gs.max(30.0) / crate::flight::KT),
            pal::TEXT2,
        );
    }
    if let Some(v) = sim.nav.plan.vnav(&env, p) {
        let tod = if v.to_tod > 0.0 { format!("T/D in {:.1} NM", v.to_tod / NM) } else { "past T/D".into() };
        status(
            ui,
            format!(
                "VNAV: {} path {:.1}° · {tod} · path altitude {:.0} ft",
                sim.nav.plan.approach.as_ref().map(|a| a.name()).unwrap_or_default(),
                v.angle.to_degrees(),
                v.alt / 0.3048
            ),
            pal::TEXT2,
        );
    }
    let _ = f_semi;
}
