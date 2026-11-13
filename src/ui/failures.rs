//! In-flight "Systems & failures" panel (O / F2, or the pause menu):
//! fail anything that applies to the type now or arm it for a time,
//! altitude, speed or random window; MTBF random failures; repair all; and
//! the systems panel — electrics, fuel, engine start (cold and dark or
//! auto-start), hydraulics, anti-ice, gear alternate extension, fire
//! handles and extinguishers (systems/).

use super::menus::{Btn, button, segmented};
use super::{UiState, alpha, f_display, f_inst_b, f_semi, f_ui, pal, sp};
use crate::flight::{AircraftDef, EngineKind, KT};
use crate::sim::Sim;
use crate::systems::defs::{Feed, Starter};
use crate::systems::failures::{Failure, Group, Kind, Trigger};
use crate::systems::{self, Mags, Power, Selector};
use egui::{Align2, Color32, Pos2, Rect, Sense, Stroke, Ui, Vec2};

/// Panel state kept in [`UiState`].
#[derive(Clone, Debug, Default)]
pub struct FailUi {
    pub open: bool,
    pub tab: usize,
    /// Failure being armed, trigger mode and its values.
    pub arm: Option<Failure>,
    pub mode: usize,
    pub a: f32,
    pub b: f32,
}

const MODES: [&str; 6] = ["In", "Above alt", "Below alt", "Above IAS", "Below IAS", "Random"];

/// Draw the panel (when open).
pub fn panel(ctx: &egui::Context, st: &mut UiState, sim: &mut Sim) {
    if !st.fail.open {
        return;
    }
    let screen = ctx.content_rect();
    let w = 560.0f32.min(screen.width() - 32.0);
    let h = (screen.height() * 0.84).min(820.0);
    let at = Pos2::new(screen.max.x - sp::L, screen.center().y);
    egui::Area::new(egui::Id::new("failures_panel")).pivot(Align2::RIGHT_CENTER).fixed_pos(at).order(egui::Order::Foreground).show(ctx, |ui| {
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
                if let Some(t) = segmented(ui, r, "fail_tabs", &["Failures", "Armed & random", "Systems"], st.fail.tab) {
                    st.fail.tab = t;
                }
                ui.add_space(sp::M);
                egui::ScrollArea::vertical().auto_shrink([false, false]).max_height(h - 170.0).show(ui, |ui| match st.fail.tab {
                    0 => failures_tab(ui, st, sim),
                    1 => schedule_tab(ui, sim),
                    _ => systems_tab(ui, sim),
                });
            });
    });
}

fn header(ui: &mut Ui, st: &mut UiState, sim: &mut Sim) {
    let def = sim.def().clone();
    let n_fail = sim.ac.sys.fail.list().len();
    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            ui.label(egui::RichText::new("Systems & failures").font(f_display(22.0)).color(pal::TEXT));
            let sub = if n_fail > 0 { format!("{} · {n_fail} active", def.name) } else { format!("{} · all systems normal", def.name) };
            ui.label(egui::RichText::new(sub).font(f_ui(12.5)).color(if n_fail > 0 { pal::AMBER } else { pal::TEXT3 }));
        });
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
            let (r, _) = ui.allocate_exact_size(Vec2::new(32.0, 32.0), Sense::hover());
            if close_x(ui, r, "fail_close") {
                st.fail.open = false;
            }
            let (r, _) = ui.allocate_exact_size(Vec2::new(104.0, 32.0), Sense::hover());
            if button(ui, r, "repair_all", "Repair all", Btn::Secondary, n_fail > 0 || sim.ac.sys.ice > 0.0).clicked() {
                systems::with_sys(&mut sim.ac, |s, a| s.repair_all(&def, a));
                sim.say("All failures repaired", 2.5);
            }
        });
    });
}

/// A small close / remove button (a drawn ×: the fonts have no ✕).
fn close_x(ui: &mut Ui, r: Rect, id: impl std::hash::Hash + std::fmt::Debug) -> bool {
    let resp = button(ui, r, id, "", Btn::Ghost, true);
    let c = r.center();
    let k = r.height() * 0.18;
    let st = Stroke::new(1.6, if resp.hovered() { pal::TEXT } else { pal::TEXT2 });
    ui.painter().line_segment([c + Vec2::new(-k, -k), c + Vec2::new(k, k)], st);
    ui.painter().line_segment([c + Vec2::new(k, -k), c + Vec2::new(-k, k)], st);
    resp.clicked()
}

fn section(ui: &mut Ui, title: &str) {
    ui.add_space(sp::S);
    ui.label(egui::RichText::new(title.to_uppercase()).font(f_inst_b(11.0)).color(pal::TEXT3));
    ui.add_space(2.0);
}

/// A failure row: label, hint, state; returns (fail / repair clicked, arm clicked).
fn fail_row(ui: &mut Ui, id: impl std::hash::Hash + std::fmt::Debug + Copy, label: &str, hint: &str, active: bool, armed: bool) -> (bool, bool) {
    let w = ui.available_width();
    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 50.0), Sense::hover());
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, if active { Color32::from_rgb(44, 22, 24) } else { pal::RAISED });
    if active {
        p.rect_filled(Rect::from_min_size(r.min + Vec2::new(0.0, 8.0), Vec2::new(3.0, r.height() - 16.0)), 2, pal::RED);
    }
    p.text(r.min + Vec2::new(14.0, 9.0), Align2::LEFT_TOP, label, f_ui(14.0), pal::TEXT);
    p.text(r.min + Vec2::new(14.0, 29.0), Align2::LEFT_TOP, hint, f_ui(11.5), pal::TEXT3);
    let b1 = Rect::from_min_size(Pos2::new(r.max.x - 86.0, r.min.y + 9.0), Vec2::new(76.0, 32.0));
    let b0 = Rect::from_min_size(Pos2::new(b1.min.x - 66.0, r.min.y + 9.0), Vec2::new(58.0, 32.0));
    let fail = button(ui, b1, (id, "f"), if active { "Repair" } else { "Fail" }, if active { Btn::Primary } else { Btn::Secondary }, true).clicked();
    let arm = !active && button(ui, b0, (id, "a"), if armed { "Armed" } else { "Arm" }, Btn::Ghost, true).clicked();
    if armed && !active {
        p.circle_filled(Pos2::new(b0.min.x - 8.0, r.center().y), 3.5, pal::AMBER);
    }
    (fail, arm)
}

fn failures_tab(ui: &mut Ui, st: &mut UiState, sim: &mut Sim) {
    let def = sim.def().clone();
    let sd = sim.ac.sys.sd;
    if !sim.ac.sys.init {
        ui.label(egui::RichText::new("Systems start with the flight.").font(f_ui(13.0)).color(pal::TEXT3));
        return;
    }
    if let Some(f) = st.fail.arm {
        arm_card(ui, st, sim, f);
        ui.add_space(sp::M);
    }
    for g in Group::ALL {
        let kinds: Vec<Kind> = Kind::ALL.iter().copied().filter(|k| k.group() == g && k.applies(&sd, &def)).collect();
        if kinds.is_empty() {
            continue;
        }
        section(ui, g.name());
        for k in kinds {
            for i in 0..k.count(&sd, &def) {
                let f = Failure::new(k, i);
                let active = sim.ac.sys.fail.has(f);
                let armed = sim.failures.armed.iter().any(|a| a.f == f);
                let (fail, arm) = fail_row(ui, (k.bit(), i), &k.label(&sd, &def, i), k.hint(&sd), active, armed);
                if fail {
                    systems::set_failure(&mut sim.ac, &def, f, !active);
                    if !active {
                        sim.say(format!("Failure: {}", k.label(&sd, &def, i)), 2.5);
                    }
                }
                if arm {
                    st.fail.arm = Some(f);
                    if st.fail.a == 0.0 {
                        st.fail.mode = 0;
                        st.fail.a = 2.0;
                        st.fail.b = 10.0;
                    }
                }
                ui.add_space(4.0);
            }
        }
    }
}

/// Trigger editor for the failure being armed.
fn arm_card(ui: &mut Ui, st: &mut UiState, sim: &mut Sim, f: Failure) {
    let def = sim.def().clone();
    let sd = sim.ac.sys.sd;
    egui::Frame::new().fill(pal::RAISED).corner_radius(sp::R_PANEL).stroke(Stroke::new(1.0, alpha(pal::AMBER, 0.6))).inner_margin(egui::Margin::same(14)).show(ui, |ui| {
        ui.label(egui::RichText::new(format!("Arm: {}", f.kind.label(&sd, &def, f.idx as usize))).font(f_semi(14.0)).color(pal::TEXT));
        ui.add_space(6.0);
        let r = ui.allocate_exact_size(Vec2::new(ui.available_width(), 32.0), Sense::hover()).0;
        if let Some(m) = segmented(ui, r, "arm_mode", &MODES, st.fail.mode) {
            st.fail.mode = m;
            let (alt_ft, kt) = (sim.ac.pos.y * 3.280_84, sim.ac.airspeed_kt());
            (st.fail.a, st.fail.b) = match m {
                0 => (2.0, 0.0),
                1 => (((alt_ft + 1000.0) / 100.0).round() * 100.0, 0.0),
                2 => ((((alt_ft - 1000.0).max(500.0)) / 100.0).round() * 100.0, 0.0),
                3 => (((kt + 20.0) / 5.0).round() * 5.0, 0.0),
                4 => ((((kt - 20.0).max(def.vs1_kt)) / 5.0).round() * 5.0, 0.0),
                _ => (2.0, 15.0),
            };
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let (lab, unit, max) = match st.fail.mode {
                0 => ("in", "min", 120.0),
                1 | 2 => ("at", "ft", 45_000.0),
                3 | 4 => ("at", "kt", 700.0),
                _ => ("between", "min", 240.0),
            };
            ui.label(egui::RichText::new(lab).font(f_ui(13.0)).color(pal::TEXT2));
            let speed = if unit == "min" { 0.1 } else { 10.0 };
            ui.add(egui::DragValue::new(&mut st.fail.a).range(0.0..=max).speed(speed).suffix(format!(" {unit}")));
            if st.fail.mode == 5 {
                ui.label(egui::RichText::new("and").font(f_ui(13.0)).color(pal::TEXT2));
                ui.add(egui::DragValue::new(&mut st.fail.b).range(st.fail.a..=max).speed(speed).suffix(" min"));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let (r, _) = ui.allocate_exact_size(Vec2::new(70.0, 30.0), Sense::hover());
                if button(ui, r, "arm_cancel", "Cancel", Btn::Ghost, true).clicked() {
                    st.fail.arm = None;
                }
                let (r, _) = ui.allocate_exact_size(Vec2::new(70.0, 30.0), Sense::hover());
                if button(ui, r, "arm_ok", "Arm", Btn::Primary, true).clicked() {
                    let now = sim.time;
                    let (a, b) = (st.fail.a, st.fail.b);
                    match st.fail.mode {
                        0 => sim.failures.arm(f, Trigger::At(now + a * 60.0)),
                        1 => sim.failures.arm(f, Trigger::Above(a * 0.3048)),
                        2 => sim.failures.arm(f, Trigger::Below(a * 0.3048)),
                        3 => sim.failures.arm(f, Trigger::FasterThan(a * KT)),
                        4 => sim.failures.arm(f, Trigger::SlowerThan(a * KT)),
                        _ => sim.failures.arm_window(f, now, a * 60.0, b.max(a) * 60.0),
                    }
                    st.fail.arm = None;
                    sim.say("Failure armed", 2.0);
                }
            });
        });
    });
}

fn schedule_tab(ui: &mut Ui, sim: &mut Sim) {
    let def = sim.def().clone();
    let sd = sim.ac.sys.sd;
    section(ui, "Armed failures");
    if sim.failures.armed.is_empty() {
        ui.label(egui::RichText::new("Nothing armed. Use Arm on the Failures tab.").font(f_ui(13.0)).color(pal::TEXT3));
    }
    let mut remove = None;
    for (k, a) in sim.failures.armed.iter().enumerate() {
        let w = ui.available_width();
        let (r, _) = ui.allocate_exact_size(Vec2::new(w, 40.0), Sense::hover());
        let p = ui.painter().clone();
        p.rect_filled(r, sp::R_CTRL, pal::RAISED);
        p.text(r.min + Vec2::new(14.0, r.height() * 0.5), Align2::LEFT_CENTER, a.f.kind.label(&sd, &def, a.f.idx as usize), f_ui(13.5), pal::TEXT);
        // Random windows are stored as a time: keep them a secret.
        let when = if a.secret { "random time".to_string() } else { a.when.describe(sim.time) };
        p.text(Pos2::new(r.max.x - 52.0, r.center().y), Align2::RIGHT_CENTER, when, f_ui(12.0), pal::AMBER);
        if close_x(ui, Rect::from_min_size(Pos2::new(r.max.x - 40.0, r.min.y + 5.0), Vec2::new(30.0, 30.0)), ("rm", k)) {
            remove = Some(k);
        }
        ui.add_space(4.0);
    }
    if let Some(k) = remove {
        sim.failures.armed.remove(k);
    }
    if !sim.failures.armed.is_empty() {
        let (r, _) = ui.allocate_exact_size(Vec2::new(140.0, 32.0), Sense::hover());
        if button(ui, r, "clear_armed", "Clear armed", Btn::Secondary, true).clicked() {
            sim.failures.clear();
        }
    }
    section(ui, "Random failures");
    let rnd = &mut sim.failures.random;
    switch(ui, "rnd_on", "Random failures (MTBF)", &mut rnd.on);
    ui.add_enabled_ui(rnd.on, |ui| {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Mean time between failures").font(f_ui(13.0)).color(pal::TEXT2));
            ui.add(egui::Slider::new(&mut rnd.mtbf_min, 2.0..=600.0).logarithmic(true).suffix(" min").max_decimals(0));
        });
        ui.add_space(4.0);
        ui.label(egui::RichText::new("Can fail").font(f_ui(12.5)).color(pal::TEXT3));
        ui.horizontal_wrapped(|ui| {
            for (gi, g) in Group::ALL.iter().enumerate() {
                if !Kind::ALL.iter().any(|k| k.group() == *g && k.applies(&sd, &def)) {
                    continue;
                }
                let mut on = rnd.groups & (1 << gi) != 0;
                if ui.toggle_value(&mut on, egui::RichText::new(g.name()).font(f_semi(12.5))).changed() {
                    rnd.groups ^= 1 << gi;
                }
            }
        });
    });
    section(ui, "Environment");
    switch(ui, "nat_ice", "Natural icing (carburettor, pitot, airframe)", &mut sim.ac.sys.natural_icing);
}

/// A switch row; returns true when toggled.
fn switch(ui: &mut Ui, id: &str, label: &str, v: &mut bool) -> bool {
    let w = ui.available_width();
    let (r, resp) = ui.allocate_exact_size(Vec2::new(w, 38.0), Sense::click());
    let p = ui.painter();
    p.rect_filled(r, sp::R_CTRL, if resp.hovered() { pal::RAISED_HI } else { pal::RAISED });
    p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, f_ui(13.5), pal::TEXT);
    let sw = Rect::from_center_size(Pos2::new(r.max.x - 32.0, r.center().y), Vec2::new(40.0, 22.0));
    p.rect_filled(sw, 11.0, if *v { pal::ACCENT } else { Color32::from_gray(70) });
    p.circle_filled(Pos2::new(if *v { sw.max.x - 11.0 } else { sw.min.x + 11.0 }, sw.center().y), 8.5, Color32::WHITE);
    let _ = id;
    ui.add_space(3.0);
    if resp.clicked() {
        *v = !*v;
        return true;
    }
    false
}

/// A label + value readout row.
fn readout(ui: &mut Ui, label: &str, value: String, col: Color32) {
    let w = ui.available_width();
    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 24.0), Sense::hover());
    let p = ui.painter();
    p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, f_ui(13.0), pal::TEXT2);
    p.text(Pos2::new(r.max.x - 14.0, r.center().y), Align2::RIGHT_CENTER, value, super::f_digits_b(14.0), col);
}

fn eng_status(sim: &Sim, i: usize) -> (String, Color32) {
    let e = &sim.ac.sys.eng[i];
    if e.fire > 0.0 {
        ("FIRE".into(), pal::RED)
    } else if e.dead {
        ("SHUT DOWN — DAMAGED".into(), pal::RED)
    } else if e.run {
        ("RUNNING".into(), pal::GREEN)
    } else if e.crank > 0.0 {
        ("CRANKING".into(), pal::AMBER)
    } else if e.start > 0.0 {
        (format!("STARTING {:.0} %", e.start * 100.0), pal::AMBER)
    } else {
        ("OFF".into(), pal::TEXT3)
    }
}

fn systems_tab(ui: &mut Ui, sim: &mut Sim) {
    let def: AircraftDef = sim.def().clone();
    if !sim.ac.sys.init {
        ui.label(egui::RichText::new("Systems start with the flight.").font(f_ui(13.0)).color(pal::TEXT3));
        return;
    }
    let sd = sim.ac.sys.sd;
    let n = sim.ac.sys.n_eng;
    // ---- quick actions
    ui.horizontal(|ui| {
        let (r, _) = ui.allocate_exact_size(Vec2::new(124.0, 34.0), Sense::hover());
        if button(ui, r, "ready", "Ready to fly", Btn::Primary, n > 0).clicked() {
            sim.ac.sys.make_ready();
        }
        let (r, _) = ui.allocate_exact_size(Vec2::new(110.0, 34.0), Sense::hover());
        let busy = sim.ac.sys.autostart != 0;
        if button(ui, r, "autostart", if busy { "Starting…" } else { "Auto-start" }, Btn::Secondary, n > 0 && !busy).clicked() {
            sim.ac.sys.begin_autostart();
        }
        let (r, _) = ui.allocate_exact_size(Vec2::new(120.0, 34.0), Sense::hover());
        if button(ui, r, "cold", "Cold & dark", Btn::Ghost, n > 0 && sim.ac.on_ground).clicked() {
            let keep = (sim.ac.sys.tank, sim.ac.sys.fail, sim.ac.sys.fuel0);
            sim.ac.sys = systems::Systems::new(&def, true);
            (sim.ac.sys.tank, sim.ac.sys.fail, sim.ac.sys.fuel0) = keep;
            sim.ctl.throttle = 0.0;
        }
    });
    switch(ui, "cold_start", "Start cold & dark (next restart)", &mut sim.cold_dark);
    let s = &mut sim.ac.sys;

    // ---- electrics
    section(ui, "Electrical");
    switch(ui, "bat", if sd.alerting == crate::systems::defs::Alerting::Annunciators { "Master (battery)" } else { "Battery" }, &mut s.sw.battery);
    for i in 0..n.min(4) {
        if sd.gens_per_engine > 0 {
            let lab = if n > 1 {
                format!("Generator {}", i + 1)
            } else if def.engine_kind == EngineKind::Piston {
                "Alternator".to_string()
            } else {
                "Generator".to_string()
            };
            switch(ui, &format!("gen{i}"), &lab, &mut s.sw.gens[i]);
        }
    }
    if n > 0 {
        switch(ui, "avionics", "Avionics", &mut s.sw.avionics);
    }
    if sd.apu {
        ui.horizontal(|ui| {
            let mut m = s.sw.apu_master;
            if ui.toggle_value(&mut m, egui::RichText::new("APU master").font(f_semi(13.0))).changed() {
                s.sw.apu_master = m;
            }
            if ui.add_enabled(s.sw.apu_master && s.apu < 0.95, egui::Button::new(egui::RichText::new("APU start").font(f_semi(13.0)))).clicked() {
                s.apu_start = true;
            }
            let mut b = s.sw.apu_bleed;
            if ui.toggle_value(&mut b, egui::RichText::new("APU bleed").font(f_semi(13.0))).changed() {
                s.sw.apu_bleed = b;
            }
            let (t, c) = if s.apu > 0.95 {
                ("AVAIL".to_string(), pal::GREEN)
            } else if s.apu > 0.0 {
                (format!("{:.0} %", s.apu * 100.0), pal::AMBER)
            } else {
                ("OFF".to_string(), pal::TEXT3)
            };
            ui.label(egui::RichText::new(t).font(f_inst_b(12.5)).color(c));
        });
    }
    let pw = match s.power {
        Power::Normal => ("NORMAL", pal::GREEN),
        Power::Emergency => (if sd.epu_min > 0.0 { "EPU" } else { "EMERGENCY (RAT)" }, pal::AMBER),
        Power::Battery => ("BATTERY ONLY", pal::AMBER),
        Power::Off => ("NO POWER", pal::RED),
    };
    readout(ui, "Bus", format!("{}  {:.1} V", pw.0, s.volts), pw.1);
    readout(ui, "Battery", format!("{:.0} %  {:+.0} A", s.soc * 100.0, s.amps), if s.amps < -0.5 { pal::AMBER } else { pal::GREEN });

    // ---- fuel
    if !sd.tanks.is_empty() {
        section(ui, "Fuel");
        if sd.feed == Feed::Selector {
            let r = ui.allocate_exact_size(Vec2::new(ui.available_width(), 32.0), Sense::hover()).0;
            let sel = [Selector::Off, Selector::Left, Selector::Right, Selector::Both];
            let cur = sel.iter().position(|x| *x == s.sw.selector).unwrap_or(3);
            if let Some(k) = segmented(ui, r, "fuel_sel", &["Off", "Left", "Right", "Both"], cur) {
                s.sw.selector = sel[k];
            }
            ui.add_space(4.0);
        }
        if sd.boost_pumps {
            switch(ui, "pumps", "Boost pumps", &mut s.sw.pumps);
        }
        if sd.feed == Feed::Wing {
            switch(ui, "xfeed", "Crossfeed", &mut s.sw.crossfeed);
        }
        for (k, t) in sd.tanks.iter().enumerate().take(3) {
            let frac = if s.tank_cap[k] > 0.0 { s.tank[k] / s.tank_cap[k] } else { 0.0 };
            let w = ui.available_width();
            let (r, _) = ui.allocate_exact_size(Vec2::new(w, 22.0), Sense::hover());
            let p = ui.painter();
            p.text(Pos2::new(r.min.x, r.center().y), Align2::LEFT_CENTER, t.0, f_inst_b(12.0), pal::TEXT2);
            let bar = Rect::from_min_max(Pos2::new(r.min.x + 70.0, r.min.y + 7.0), Pos2::new(r.max.x - 90.0, r.max.y - 7.0));
            p.rect_filled(bar, 4, Color32::from_white_alpha(14));
            p.rect_filled(Rect::from_min_size(bar.min, Vec2::new(bar.width() * frac.clamp(0.0, 1.0), bar.height())), 4, if frac < 0.1 { pal::AMBER } else { pal::ACCENT });
            p.text(Pos2::new(r.max.x, r.center().y), Align2::RIGHT_CENTER, format!("{:.0} kg", s.tank[k]), super::f_digits_b(13.0), pal::TEXT);
        }
    }

    // ---- engines
    if n > 0 {
        section(ui, "Engines");
        let piston = def.engine_kind == EngineKind::Piston;
        if !piston && sd.starter != Starter::None {
            switch(ui, "ign", "Engine mode: IGN / START", &mut s.sw.ign_start);
        }
        for i in 0..n.min(4) {
            let (txt, col) = eng_status(sim, i);
            let s = &mut sim.ac.sys;
            ui.horizontal(|ui| {
                ui.add_sized([86.0, 26.0], egui::Label::new(egui::RichText::new(if n > 1 { format!("Engine {}", i + 1) } else { "Engine".into() }).font(f_semi(13.5))));
                ui.label(egui::RichText::new(txt).font(f_inst_b(12.0)).color(col));
            });
            ui.horizontal_wrapped(|ui| {
                if piston {
                    for (m, lab) in [(Mags::Off, "OFF"), (Mags::Right, "R"), (Mags::Left, "L"), (Mags::Both, "BOTH")] {
                        if ui.selectable_label(s.sw.mags[i] == m, egui::RichText::new(lab).font(f_semi(12.5))).clicked() {
                            s.sw.mags[i] = m;
                        }
                    }
                    ui.separator();
                    let mut rich = s.sw.eng_master[i];
                    let txt = if rich { "Mixture RICH" } else { "Mixture CUT-OFF" };
                    if ui.toggle_value(&mut rich, egui::RichText::new(txt).font(f_semi(12.5))).changed() {
                        s.sw.eng_master[i] = rich;
                    }
                    if ui.button(egui::RichText::new(format!("Prime ({:.0})", s.eng[i].primer)).font(f_semi(12.5))).clicked() {
                        s.eng[i].primer += 1.0;
                    }
                    if ui.add_enabled(!s.eng[i].run, egui::Button::new(egui::RichText::new("Start").font(f_semi(12.5)))).clicked() {
                        s.press_start(i);
                    }
                } else {
                    let mut m = s.sw.eng_master[i];
                    let lab = if def.engine_kind == EngineKind::Turboprop { "Condition lever" } else { "Master" };
                    let txt = format!("{lab} {}", if m { "ON" } else { "OFF" });
                    if ui.toggle_value(&mut m, egui::RichText::new(txt).font(f_semi(12.5))).changed() {
                        s.sw.eng_master[i] = m;
                    }
                    if ui.add_enabled(!s.eng[i].run && sd.starter != Starter::None, egui::Button::new(egui::RichText::new("Start").font(f_semi(12.5)))).clicked() {
                        s.press_start(i);
                    }
                }
                if sd.fire_detect || sd.bottles > 0 {
                    ui.separator();
                    let mut h = s.sw.fire_handle[i];
                    if ui.toggle_value(&mut h, egui::RichText::new("Fire handle").font(f_semi(12.5)).color(if s.eng[i].fire > 0.0 { pal::RED } else { pal::TEXT })).changed() {
                        s.sw.fire_handle[i] = h;
                    }
                    if sd.bottles > 0
                        && ui.add_enabled(s.eng[i].agent < sd.bottles, egui::Button::new(egui::RichText::new(format!("Agent {}", s.eng[i].agent + 1)).font(f_semi(12.5)))).clicked()
                    {
                        s.discharge_agent(i);
                    }
                }
            });
            let s = &sim.ac.sys;
            let oil_t = sim.ac.oat() + 70.0 + s.eng[i].oil_t_add;
            if s.eng[i].run || s.eng[i].oil_t_add > 1.0 {
                ui.label(
                    egui::RichText::new(format!(
                        "Oil {:.0} °C · {:.0} psi{}",
                        oil_t,
                        if s.eng[i].run { 60.0 * s.eng[i].oil_p_mul } else { 0.0 },
                        if s.eng[i].carb_ice > 0.1 { "  · carb ice!" } else { "" }
                    ))
                    .font(f_ui(12.0))
                    .color(pal::TEXT3),
                );
            }
            ui.add_space(4.0);
        }
        let s = &mut sim.ac.sys;
        if sd.carburettor {
            switch(ui, "carb", "Carburettor heat", &mut s.sw.carb_heat);
        }
    }

    // ---- hydraulics
    let s = &mut sim.ac.sys;
    if !sd.hyd.is_empty() {
        section(ui, "Hydraulics");
        ui.horizontal(|ui| {
            for (k, h) in sd.hyd.iter().enumerate().take(3) {
                let ok = s.hyd[k] > 0.5;
                ui.label(egui::RichText::new(format!("{} {:.0} psi", h.name, (s.hyd_psi(k) / 50.0).round() * 50.0)).font(f_inst_b(12.5)).color(if ok {
                    pal::GREEN
                } else {
                    pal::AMBER
                }));
                ui.add_space(10.0);
            }
        });
        if sd.hyd.iter().any(|h| h.elec) {
            switch(ui, "hyd_elec", "Electric pumps", &mut s.sw.hyd_elec);
        }
    }

    // ---- ice protection
    section(ui, "Ice protection");
    if sd.pitot_heat {
        switch(ui, "pitot", "Pitot / probe heat", &mut s.sw.pitot_heat);
    }
    if sd.wing_anti_ice {
        switch(ui, "wing_ai", if sd.alerting == crate::systems::defs::Alerting::Atr { "Airframe de-icing" } else { "Wing anti-ice" }, &mut s.sw.wing_anti_ice);
    }
    if sd.eng_anti_ice {
        switch(ui, "eng_ai", "Engine anti-ice", &mut s.sw.eng_anti_ice);
    }
    readout(ui, "Airframe ice", format!("{:.0} %", s.ice * 100.0), if s.ice > 0.05 { pal::AMBER } else { pal::GREEN });

    // ---- gear
    if def.retractable_gear {
        section(ui, "Landing gear");
        let (r, _) = ui.allocate_exact_size(Vec2::new(240.0, 34.0), Sense::hover());
        let label = if s.sw.alt_gear { "Alternate extension pulled" } else { "Alternate gear extension" };
        if button(ui, r, "alt_gear", label, Btn::Secondary, !s.sw.alt_gear).clicked() {
            s.sw.alt_gear = true;
            sim.ctl.gear_down = true;
            sim.say("Gear: alternate (gravity) extension", 3.0);
        }
    }
}
