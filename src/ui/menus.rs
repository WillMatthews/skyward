//! Menus and flow: main menu → aircraft → departure (airport picker with a
//! world map and airport diagram) → conditions → fly; settings, pause and
//! the loading screen.

use super::overview::{self, View as OView};
use super::settings::{HudMode, MapStyle, DETAIL_NAMES};
use super::{alpha, f_display, f_inst, f_inst_b, f_mono, f_semi, f_ui, gradient_rect, hhmm, pal, panel, panel_with, sp, thousands, Action, AltUnit, Screen, SpeedUnit, UiState};
use crate::airports::{self, Airport, Db, Kind};
use crate::flight::{AircraftDef, Category, EngineKind};
use crate::startpos::Spec;
use egui::{epaint, Align2, Color32, CornerRadius, Id, Pos2, Rect, Response, RichText, Sense, Shape, Stroke, Ui, UiBuilder, Vec2};
use std::sync::Arc;

// ---------------------------------------------------------------- widgets

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Btn {
    Primary,
    Secondary,
    Ghost,
}

/// A button drawn to the design system. Returns the response.
pub fn button(ui: &mut Ui, rect: Rect, id: impl std::hash::Hash + std::fmt::Debug, text: &str, kind: Btn, enabled: bool) -> Response {
    let r = ui.interact(rect, ui.id().with(id), if enabled { Sense::click() } else { Sense::hover() });
    let p = ui.painter();
    let hov = r.hovered() && enabled;
    let (fill, fg, stroke) = match kind {
        Btn::Primary => (if hov { Color32::from_rgb(112, 192, 255) } else { pal::ACCENT }, Color32::from_rgb(6, 18, 32), Color32::TRANSPARENT),
        Btn::Secondary => (if hov { pal::RAISED_HI } else { pal::RAISED }, pal::TEXT, pal::HAIRLINE_HI),
        Btn::Ghost => (if hov { Color32::from_white_alpha(14) } else { Color32::TRANSPARENT }, if hov { pal::TEXT } else { pal::TEXT2 }, Color32::TRANSPARENT),
    };
    let fill = if enabled { fill } else { alpha(fill, 0.4) };
    let fg = if enabled { fg } else { alpha(fg, 0.5) };
    p.rect_filled(rect, sp::R_CTRL, fill);
    if stroke != Color32::TRANSPARENT {
        p.rect_stroke(rect, sp::R_CTRL, Stroke::new(1.0, stroke), egui::StrokeKind::Inside);
    }
    p.text(rect.center(), Align2::CENTER_CENTER, text, f_semi(if rect.height() > 40.0 { 16.0 } else { 14.0 }), fg);
    if hov {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    r
}

/// Toggle chip; returns true when clicked.
pub(super) fn chip_btn(ui: &mut Ui, rect: Rect, id: impl std::hash::Hash + std::fmt::Debug, text: &str, on: bool) -> bool {
    let r = ui.interact(rect, ui.id().with(id), Sense::click());
    let p = ui.painter();
    let fill = if on { pal::ACCENT_DEEP } else if r.hovered() { pal::RAISED_HI } else { pal::RAISED };
    p.rect_filled(rect, rect.height() * 0.5, fill);
    if on {
        p.rect_stroke(rect, rect.height() * 0.5, Stroke::new(1.0, pal::ACCENT), egui::StrokeKind::Inside);
    }
    p.text(rect.center(), Align2::CENTER_CENTER, text, f_semi(13.0), if on { pal::WHITE } else { pal::TEXT2 });
    r.clicked()
}

/// Segmented control; returns the newly clicked index.
pub(super) fn segmented(ui: &mut Ui, rect: Rect, id: &str, items: &[&str], sel: usize) -> Option<usize> {
    let p = ui.painter().clone();
    p.rect_filled(rect, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    let w = rect.width() / items.len() as f32;
    let mut out = None;
    for (i, it) in items.iter().enumerate() {
        let r = Rect::from_min_size(rect.min + Vec2::new(w * i as f32, 0.0), Vec2::new(w, rect.height())).shrink(3.0);
        let resp = ui.interact(r, ui.id().with((id, i)), Sense::click());
        if i == sel {
            p.rect_filled(r, sp::R_CTRL - 2, pal::RAISED_HI);
        } else if resp.hovered() {
            p.rect_filled(r, sp::R_CTRL - 2, Color32::from_white_alpha(8));
        }
        p.text(r.center(), Align2::CENTER_CENTER, *it, f_semi(13.0), if i == sel { pal::TEXT } else { pal::TEXT2 });
        if resp.clicked() && i != sel {
            out = Some(i);
        }
    }
    out
}

pub(super) fn heading(p: &egui::Painter, at: Pos2, title: &str, sub: &str) -> f32 {
    p.text(at, Align2::LEFT_TOP, title, f_display(30.0), pal::TEXT);
    p.text(at + Vec2::new(0.0, 40.0), Align2::LEFT_TOP, sub, f_ui(14.0), pal::TEXT2);
    at.y + 70.0
}

/// "Aircraft › Departure › Conditions" breadcrumb; returns a clicked step.
fn steps(ui: &mut Ui, at: Pos2, current: usize) -> Option<usize> {
    let names = ["Aircraft", "Departure", "Conditions"];
    let mut x = at.x;
    let mut out = None;
    for (i, n) in names.iter().enumerate() {
        let g = ui.painter().layout_no_wrap(format!("{}  {}", i + 1, n), f_semi(13.0), pal::TEXT);
        let r = Rect::from_min_size(Pos2::new(x, at.y), Vec2::new(g.size().x + 24.0, 30.0));
        let resp = ui.interact(r, ui.id().with(("step", i)), Sense::click());
        let col = if i == current { pal::TEXT } else if i < current { pal::ACCENT } else { pal::TEXT3 };
        if i == current {
            ui.painter().rect_filled(r, 15.0, pal::RAISED);
        } else if resp.hovered() {
            ui.painter().rect_filled(r, 15.0, Color32::from_white_alpha(8));
        }
        ui.painter().text(r.center(), Align2::CENTER_CENTER, format!("{}  {}", i + 1, n), f_semi(13.0), col);
        if resp.clicked() {
            out = Some(i);
        }
        x = r.max.x + 6.0;
        if i < 2 {
            ui.painter().text(Pos2::new(x + 2.0, r.center().y), Align2::LEFT_CENTER, "›", f_semi(15.0), pal::TEXT3);
            x += 16.0;
        }
    }
    out
}

/// The menus' backdrop, for other screens (ui/missions.rs).
pub fn backdrop_pub(p: &egui::Painter, screen: Rect, strength: f32) {
    backdrop(p, screen, strength);
}

/// Darken the 3D scene behind a menu: strong on the left, lighter right.
pub(super) fn backdrop(p: &egui::Painter, screen: Rect, strength: f32) {
    let mut m = epaint::Mesh::default();
    let a = |v: f32| Color32::from_rgba_premultiplied(5, 7, 10, (v * 255.0 * strength) as u8);
    m.colored_vertex(screen.left_top(), a(0.92));
    m.colored_vertex(screen.right_top(), a(0.55));
    m.colored_vertex(screen.right_bottom(), a(0.7));
    m.colored_vertex(screen.left_bottom(), a(0.95));
    m.add_triangle(0, 1, 2);
    m.add_triangle(0, 2, 3);
    p.add(Shape::mesh(m));
}

/// Display class of a type (its role, else the category name).
pub fn class_name(d: &AircraftDef) -> &'static str {
    if d.role.is_empty() { category_name(d.category) } else { d.role }
}

pub fn category_name(c: Category) -> &'static str {
    match c {
        Category::Trainer => "Light aircraft",
        Category::Glider => "Glider",
        Category::Warbird => "Warbird",
        Category::FastJet => "Fast jet",
        Category::Airliner => "Airliner",
        Category::Helicopter => "Rotorcraft",
        Category::Drone => "Drone",
    }
}

/// Runway length (m) and surface a type needs.
pub fn needs(def: &AircraftDef) -> (f32, bool) {
    (crate::sim::min_runway(def), matches!(def.category, Category::Airliner | Category::FastJet) && !def.rough_field)
}

pub fn suitable(a: &Airport, def: &AircraftDef) -> bool {
    if def.on_water() {
        // Floatplanes start on the nearest open water (src/water.rs): seaplane
        // bases and anywhere by a river or lake.
        return true;
    }
    let (len, paved) = needs(def);
    a.kind != Kind::Seaplane && a.usable_m(paved) >= len * 0.95
}

fn spec_lines(d: &AircraftDef) -> Vec<(&'static str, String)> {
    if d.rotor.is_some() {
        return crate::rotor::instruments::spec_lines(d);
    }
    let n = d.engine_count;
    let engines = match d.engine_kind {
        EngineKind::None => "None".to_string(),
        EngineKind::Piston => format!("{n} × piston, {:.0} hp", d.engine_power_w / 745.7),
        EngineKind::Turboprop => format!("{n} × turboprop, {:.0} shp", d.engine_power_w / 745.7),
        EngineKind::Turboshaft => format!("{n} × turboshaft, {:.0} shp", d.engine_power_w / 745.7),
        EngineKind::Electric => format!("{n} × brushless motor, 6S LiPo"),
        EngineKind::Jet => {
            let dry = d.static_thrust / n.max(1) as f32 / 1000.0;
            // (The Vulcan's Olympus 301s and Concorde's 593s are pure turbojets.)
            let kind = if matches!(d.id, "vulcan" | "concorde") { "turbojet" } else { "turbofan" };
            if d.has_afterburner {
                // Thrust per engine, dry and with reheat.
                format!("{n} × {kind}, {:.0} kN · {:.0} kN reheat", dry, (d.static_thrust + d.ab_static_thrust) / n.max(1) as f32 / 1000.0)
            } else {
                format!("{n} × {kind}, {:.0} kN", dry)
            }
        }
    };
    let mass = if d.mass >= 10_000.0 { format!("{:.1} t", d.mass / 1000.0) } else { format!("{:.0} kg", d.mass) };
    let mut v = vec![("Engines", engines), ("Mass", mass), ("Wingspan", format!("{:.1} m", d.wingspan)), ("Stall", format!("{:.0} kt clean · {:.0} kt landing", d.vs1_kt, d.vs0_kt))];
    if d.has_engine {
        v.push(("Rotate · approach", format!("{:.0} · {:.0} kt", d.vr_kt, d.vref_kt)));
    }
    v.push(("Never exceed", if d.mmo > 0.0 { format!("{:.0} kt · M {:.2}", d.vne_kt, d.mmo) } else { format!("{:.0} kt", d.vne_kt) }));
    let (len, paved) = needs(d);
    if d.on_water() {
        v.push(("Operates from", format!("Water — rivers, lakes, sea ({} m)", thousands(len))));
    } else {
        v.push(("Runway", format!("{} m{}", thousands(len), if paved { " paved" } else { "" })));
    }
    v.push(("Model", if d.jsb.is_some() { "FlightGear · JSBSim aero".to_string() } else { "FlightGear · calibrated".to_string() }));
    v
}

// ---------------------------------------------------------------- main menu

/// Why Missions is greyed out while connected (missions are single-player).
pub const MISSIONS_SOLO: &str = "Missions are single-player — leave the server to fly one.";

pub fn main_menu(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, 1.0);
    let x = screen.min.x + (screen.width() * 0.07).max(48.0);
    let y = screen.center().y - 230.0;
    p.text(Pos2::new(x, y), Align2::LEFT_TOP, "SKYWARD", f_display(64.0), pal::TEXT);
    p.text(Pos2::new(x + 3.0, y + 78.0), Align2::LEFT_TOP, "Fly anywhere on Earth, over real map data.", f_ui(17.0), pal::TEXT2);
    let bw = 340.0;
    let mut by = y + 140.0;
    let last = st.settings.last.clone();
    if let Some(l) = &last {
        let def = st.defs.iter().find(|d| d.id == l.aircraft).map(|d| d.name).unwrap_or("Aircraft");
        let r = Rect::from_min_size(Pos2::new(x, by), Vec2::new(bw, 64.0));
        if button(ui, r, "continue", "", Btn::Primary, true).clicked() {
            st.actions.push(Action::FlyLast);
        }
        p.text(r.min + Vec2::new(20.0, 13.0), Align2::LEFT_TOP, "Fly again", f_semi(17.0), Color32::from_rgb(6, 18, 32));
        let spec = Spec::decode(&l.start).map(|s| s.label()).unwrap_or_default();
        p.text(r.min + Vec2::new(20.0, 37.0), Align2::LEFT_TOP, format!("{def} · {} · {spec}", l.airport), f_ui(13.0), Color32::from_rgb(20, 44, 70));
        by += 76.0;
    }
    let items: [(&str, Action, &str); 6] = [
        ("New flight", Action::Goto(Screen::Aircraft), "new"),
        ("Multiplayer", Action::Goto(Screen::Multiplayer), "multiplayer"),
        ("Missions", Action::Goto(Screen::Missions), "missions"),
        ("Settings", Action::Goto(Screen::Settings), "settings"),
        ("Credits", Action::Goto(Screen::Credits), "credits"),
        ("Quit", Action::Quit, "quit"),
    ];
    // (A browser tab has nothing to quit to.)
    for (i, (label, act, id)) in items.into_iter().filter(|it| !(crate::platform::WEB && matches!(it.1, Action::Quit))).enumerate() {
        let r = Rect::from_min_size(Pos2::new(x, by), Vec2::new(bw, 48.0));
        let kind = if i == 0 && last.is_none() { Btn::Primary } else { Btn::Secondary };
        // Missions are single-player: not while connected to a server.
        let solo_only = matches!(act, Action::Goto(Screen::Missions)) && st.mp.connected();
        let mut resp = button(ui, r, id, label, kind, !solo_only);
        if solo_only {
            resp = resp.on_hover_text(MISSIONS_SOLO);
        }
        if resp.clicked() {
            st.actions.push(act);
        }
        by += 58.0;
    }
    // The flight the menu is ready to start (its scenery is loading behind the menu).
    let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].clone();
    let cw = 420.0;
    let card = Rect::from_min_size(Pos2::new(screen.max.x - cw - (screen.width() * 0.05).max(40.0), screen.max.y - 220.0), Vec2::new(cw, 150.0));
    panel(&p, card, sp::R_CARD);
    let img = Rect::from_min_size(card.min + Vec2::splat(10.0), Vec2::new(170.0, 130.0));
    if let Some(t) = st.thumbs.get(def.id) {
        p.add(Shape::Rect(epaint::RectShape::filled(img, 10, Color32::WHITE).with_texture(*t, Rect::from_min_max(Pos2::new(0.12, 0.0), Pos2::new(0.88, 1.0)))));
    } else {
        gradient_rect(&p, img, Color32::from_rgb(70, 120, 180), Color32::from_rgb(170, 200, 230));
    }
    let tx = img.max.x + 18.0;
    p.text(Pos2::new(tx, card.min.y + 18.0), Align2::LEFT_TOP, "READY TO FLY", f_inst_b(11.0), pal::ACCENT);
    p.text(Pos2::new(tx, card.min.y + 38.0), Align2::LEFT_TOP, def.name, f_semi(17.0), pal::TEXT);
    let ap = st.db.as_ref().and_then(|d| d.get(&st.plan.airport)).map(|a| format!("{} · {}", a.code(), clip_text(&a.name, 22))).unwrap_or_else(|| st.plan.airport.clone());
    p.text(Pos2::new(tx, card.min.y + 64.0), Align2::LEFT_TOP, ap, f_ui(13.0), pal::TEXT2);
    p.text(Pos2::new(tx, card.min.y + 84.0), Align2::LEFT_TOP, st.plan.spec.label(), f_ui(13.0), pal::TEXT3);
    if button(ui, Rect::from_min_size(Pos2::new(tx, card.max.y - 46.0), Vec2::new(card.max.x - tx - 16.0, 36.0)), "flynow", "Fly", Btn::Primary, true).clicked() {
        st.plan.dogfight = None;
        st.actions.push(Action::Fly);
    }
    dogfight_card(ui, st, Rect::from_min_size(Pos2::new(card.min.x, card.min.y - 286.0), Vec2::new(cw, 274.0)));
    p.text(Pos2::new(x, screen.max.y - 28.0), Align2::LEFT_BOTTOM, format!("{} · {} · {}", crate::hud::ATTRIBUTION, airports::CREDIT, WEATHER_CREDIT_SHORT), f_ui(11.0), pal::TEXT3);
}

/// Weather sources (the browser reads METARs through VATSIM's CORS-enabled
/// mirror: aviationweather.gov sends no CORS headers).
const WEATHER_CREDIT: &str = if crate::platform::WEB {
    "METARs: national weather services via VATSIM's METAR service (metar.vatsim.net) · forecasts: Open-Meteo (CC BY 4.0)"
} else {
    "METARs: aviationweather.gov (NOAA) · forecasts: Open-Meteo (CC BY 4.0)"
};
const WEATHER_CREDIT_SHORT: &str = if crate::platform::WEB { "Weather: METARs via VATSIM, Open-Meteo" } else { "Weather: NOAA aviationweather.gov, Open-Meteo" };

/// The dogfight quick start: aircraft, opponent, format, opponents' skill.
fn dogfight_card(ui: &mut Ui, st: &mut UiState, card: Rect) {
    use crate::weapons::ai::Difficulty;
    use crate::weapons::setup::Format;
    let p = ui.painter().clone();
    panel(&p, card, sp::R_CARD);
    let x = card.min.x + 18.0;
    let w = card.width() - 36.0;
    p.text(Pos2::new(x, card.min.y + 16.0), Align2::LEFT_TOP, "DOGFIGHT", f_inst_b(11.0), pal::ACCENT);
    p.text(Pos2::new(x, card.min.y + 34.0), Align2::LEFT_TOP, "Guns, missiles and AI pilots over this airfield", f_ui(13.0), pal::TEXT2);
    let types = [("peregrine", "Spitfire IX"), ("lancer", "F-16C")];
    let lw = 84.0;
    let mut y = card.min.y + 62.0;
    let row = |p: &egui::Painter, y: f32, label: &str| {
        p.text(Pos2::new(x, y + 15.0), Align2::LEFT_CENTER, label, f_ui(13.0), pal::TEXT3);
        Rect::from_min_size(Pos2::new(x + lw, y), Vec2::new(w - lw, 30.0))
    };
    let names: Vec<&str> = types.iter().map(|t| t.1).collect();
    let sel = types.iter().position(|t| t.0 == st.dogfight.player).unwrap_or(0);
    if let Some(i) = segmented(ui, row(&p, y, "You"), "df_you", &names, sel) {
        st.dogfight.player = types[i].0.to_string();
    }
    y += 38.0;
    let sel = types.iter().position(|t| t.0 == st.dogfight.opponent).unwrap_or(0);
    if let Some(i) = segmented(ui, row(&p, y, "Opponent"), "df_opp", &names, sel) {
        st.dogfight.opponent = types[i].0.to_string();
    }
    y += 38.0;
    let fmts = [Format::OneVOne, Format::TwoVTwo];
    let sel = fmts.iter().position(|f| *f == st.dogfight.format).unwrap_or(0);
    if let Some(i) = segmented(ui, row(&p, y, "Format"), "df_fmt", &["1 v 1", "2 v 2"], sel) {
        st.dogfight.format = fmts[i];
    }
    y += 38.0;
    let names: Vec<&str> = Difficulty::ALL.iter().map(|d| d.name()).collect();
    let sel = Difficulty::ALL.iter().position(|d| *d == st.dogfight.difficulty).unwrap_or(1);
    if let Some(i) = segmented(ui, row(&p, y, "Pilots"), "df_skill", &names, sel) {
        st.dogfight.difficulty = Difficulty::ALL[i];
    }
    y += 44.0;
    if button(ui, Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, 36.0)), "df_fight", "Fight", Btn::Primary, true).clicked() {
        if let Some(i) = st.defs.iter().position(|d| d.id == st.dogfight.player) {
            st.plan.aircraft = i;
        }
        st.plan.watch = false;
        st.plan.dogfight = Some(st.dogfight.clone());
        st.actions.push(Action::Fly);
    }
}

pub fn credits(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, 1.0);
    let x = screen.min.x + (screen.width() * 0.07).max(48.0);
    let mut y = heading(&p, Pos2::new(x, screen.min.y + 80.0), "Credits", "Data, fonts and services Skyward is built on.");
    let rows = [
        ("Map data", "© OpenMapTiles · © OpenStreetMap contributors (ODbL)"),
        ("Vector tiles", "OpenFreeMap · airport detail: OSM via the Overpass API"),
        ("Airports & runways", "OurAirports — public domain (ourairports.com/data)"),
        ("Navaids", "VOR / DME / NDB: OurAirports — public domain · ILS: synthetic, derived from the runway data (a few London ILS from public charts)"),
        ("ATC", "Frequencies: OurAirports — public domain · voices: Kokoro-82M (Apache-2.0), generated for Skyward"),
        ("Weather", WEATHER_CREDIT),
        ("Elevation", "Terrain Tiles on AWS (Mapzen / Tilezen: SRTM, GMTED2010, 3DEP, ETOPO1, EU-DEM and more)"),
        ("Aircraft & sounds", "FlightGear aircraft and recordings — GPL v2 / v2+ (per-aircraft credits in the source)"),
        ("Fonts", "Inter (Rasmus Andersson), JetBrains Mono, B612 (Airbus / Intactile / ENAC) — SIL Open Font License 1.1"),
        ("Licence", "Skyward is free software: GNU GPL v2 · github.com/WillMatthews/skyward"),
    ];
    // Rows wrap to the window and scroll when it is short.
    let rows_r = Rect::from_min_max(Pos2::new(x, y), Pos2::new(screen.max.x - (screen.width() * 0.07).max(48.0) + super::scroll::GUTTER, screen.max.y - 100.0));
    let out = super::scroll::page(ui, rows_r, "credits", Color32::from_rgba_premultiplied(5, 7, 10, 235), |ui, o, w| {
        let p = ui.painter().clone();
        let kw = 200.0;
        let mut yy = o.y;
        for (k, v) in rows {
            p.text(Pos2::new(o.x, yy), Align2::LEFT_TOP, k, f_semi(14.0), pal::TEXT);
            let g = p.layout(v.to_string(), f_ui(14.0), pal::TEXT2, (w - kw - super::scroll::GUTTER).max(160.0));
            let gh = g.size().y;
            p.galley(Pos2::new(o.x + kw, yy), g, pal::TEXT2);
            yy += gh.max(18.0) + 14.0;
        }
        yy - o.y
    });
    y += out.content_size.y.min(rows_r.height());
    if button(ui, Rect::from_min_size(Pos2::new(x, y + 24.0), Vec2::new(140.0, 44.0)), "back", "Back", Btn::Secondary, true).clicked() || esc(ui) {
        st.actions.push(Action::Back);
    }
    // Browser: the full attributions and the licence texts are pages of the site.
    #[cfg(target_arch = "wasm32")]
    if button(ui, Rect::from_min_size(Pos2::new(x + 156.0, y + 24.0), Vec2::new(240.0, 44.0)), "credits_page", "Full credits & licence", Btn::Secondary, true).clicked() {
        crate::platform::web::open_url("credits.html");
    }
}

pub(super) fn esc(ui: &Ui) -> bool {
    ui.input(|i| i.key_pressed(egui::Key::Escape))
}

// ---------------------------------------------------------------- aircraft

/// Families on the aircraft screen (filter chips; the grid is in this order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Group {
    Light,
    Warbird,
    Jet,
    Airliner,
    Military,
    Rotor,
}

impl Group {
    pub const ALL: [Group; 6] = [Group::Light, Group::Warbird, Group::Jet, Group::Airliner, Group::Military, Group::Rotor];

    pub fn of(d: &AircraftDef) -> Group {
        match d.category {
            Category::Trainer | Category::Glider => Group::Light,
            Category::Warbird => Group::Warbird,
            Category::FastJet => Group::Jet,
            Category::Helicopter | Category::Drone => Group::Rotor,
            Category::Airliner => {
                // Military heavies fly airliner-class models; their role says what they are.
                let r = d.role.to_ascii_lowercase();
                if ["military", "airlift", "bomber", "tanker", "maritime"].iter().any(|k| r.contains(k)) { Group::Military } else { Group::Airliner }
            }
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Group::Light => "Light & gliders",
            Group::Warbird => "Warbirds",
            Group::Jet => "Fast jets",
            Group::Airliner => "Airliners",
            Group::Military => "Military heavies",
            Group::Rotor => "Rotorcraft & drones",
        }
    }

    /// For `SKYWARD_AIRCRAFT_GROUP` (screenshots).
    pub fn parse(s: &str) -> Option<Group> {
        Some(match s {
            "light" => Group::Light,
            "warbird" => Group::Warbird,
            "jet" => Group::Jet,
            "airliner" => Group::Airliner,
            "military" => Group::Military,
            "rotor" => Group::Rotor,
            _ => return None,
        })
    }
}

/// A filter chip with a count; returns true when clicked.
fn count_chip(ui: &mut Ui, r: Rect, id: impl std::hash::Hash + std::fmt::Debug, label: &str, n: usize, on: bool) -> bool {
    let resp = ui.interact(r, ui.id().with(id), Sense::click());
    let p = ui.painter();
    let fill = if on { pal::ACCENT_DEEP } else if resp.hovered() { pal::RAISED_HI } else { pal::RAISED };
    p.rect_filled(r, r.height() * 0.5, fill);
    if on {
        p.rect_stroke(r, r.height() * 0.5, Stroke::new(1.0, pal::ACCENT), egui::StrokeKind::Inside);
    }
    p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, f_semi(13.0), if on { pal::WHITE } else { pal::TEXT2 });
    p.text(Pos2::new(r.max.x - 13.0, r.center().y), Align2::RIGHT_CENTER, n.to_string(), f_mono(12.0), if on { Color32::from_rgb(190, 222, 255) } else { pal::TEXT3 });
    if resp.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    resp.clicked()
}

/// Card grid geometry for a width: (columns, card width, card height).
fn card_grid(w: f32) -> (usize, f32, f32) {
    const MIN_W: f32 = 228.0;
    let cols = (((w + CARD_GAP) / (MIN_W + CARD_GAP)).floor() as usize).max(1);
    let cw = (w - CARD_GAP * (cols as f32 - 1.0)) / cols as f32;
    (cols, cw, cw * 0.62 + 64.0)
}

const CARD_GAP: f32 = 16.0;

pub fn aircraft(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, 1.0);
    let margin = (screen.width() * 0.05).clamp(24.0, 96.0);
    let content = Rect::from_min_max(Pos2::new(screen.min.x + margin, screen.min.y + 36.0), Pos2::new(screen.max.x - margin, screen.max.y - 36.0));
    if let Some(s) = steps(ui, content.min, 0) {
        st.actions.push(Action::Goto([Screen::Aircraft, Screen::Departure, Screen::Conditions][s]));
    }
    let y = heading(&p, content.min + Vec2::new(0.0, 56.0), "Choose an aircraft", "Each type flies its own flight model and has its own instruments.");
    let n_all = st.defs.len();
    let sel_now = st.plan.aircraft.min(n_all - 1);
    // ---- families: filter chips (only families that have types)
    let mut order: Vec<usize> = (0..n_all).collect();
    order.sort_by_key(|&i| (Group::of(&st.defs[i]), i));
    let count = |g: Group| order.iter().filter(|&&i| Group::of(&st.defs[i]) == g).count();
    let groups: Vec<(Group, usize)> = Group::ALL.iter().map(|&g| (g, count(g))).filter(|e| e.1 > 0).collect();
    if st.aircraft_group.is_some_and(|g| Group::of(&st.defs[sel_now]) != g || count(g) == 0) {
        // The selection always shows (it may have been chosen elsewhere).
        st.aircraft_group = None;
    }
    let chip_w = |label: &str, n: usize| p.layout_no_wrap(label.to_string(), f_semi(13.0), pal::TEXT).size().x + p.layout_no_wrap(n.to_string(), f_mono(12.0), pal::TEXT).size().x + 40.0;
    let (mut cx, mut cy) = (content.min.x, y + 2.0);
    let mut picked: Option<Option<Group>> = None;
    let entries: Vec<(Option<Group>, &str, usize)> = std::iter::once((None, "All aircraft", n_all)).chain(groups.iter().map(|&(g, n)| (Some(g), g.name(), n))).collect();
    for (k, (g, label, n)) in entries.iter().enumerate() {
        let w = chip_w(label, *n);
        if cx + w > content.max.x && cx > content.min.x {
            cx = content.min.x;
            cy += 40.0;
        }
        if count_chip(ui, Rect::from_min_size(Pos2::new(cx, cy), Vec2::new(w, 32.0)), ("acgroup", k), label, *n, st.aircraft_group == *g) {
            picked = Some(*g);
        }
        cx += w + 8.0;
    }
    let top = cy + 32.0 + 18.0;
    // ---- input: chips, arrows / d-pad, bumpers, Enter / A, Esc / B
    let nav = super::scroll::nav(ui, &st.input, st.frame_no);
    if nav.tab != 0 {
        let k = entries.iter().position(|e| e.0 == st.aircraft_group).unwrap_or(0) as i32;
        picked = Some(entries[(k + nav.tab).rem_euclid(entries.len() as i32) as usize].0);
    }
    // Scroll the selection into view: (alignment, animated).
    let mut reveal: Option<(Option<egui::Align>, bool)> = None;
    if let Some(g) = picked {
        if g != st.aircraft_group {
            st.aircraft_group = g;
            // A new family: select its first type, from the top.
            if let Some(&i) = order.iter().find(|&&i| g.is_none_or(|g| Group::of(&st.defs[i]) == g)) {
                if g.is_some() {
                    select_aircraft(st, i);
                }
            }
            reveal = Some((Some(egui::Align::Center), false));
        }
    }
    let vis: Vec<usize> = order.iter().copied().filter(|&i| st.aircraft_group.is_none_or(|g| Group::of(&st.defs[i]) == g)).collect();
    // ---- layout: card grid (left, scrolls) + details (right); narrow windows drop the details
    let wide = content.width() >= 980.0;
    let detail_w = if wide { (content.width() * 0.34).clamp(360.0, 560.0) } else { 0.0 };
    let bottom = content.max.y - 70.0;
    let grid_r = Rect::from_min_max(Pos2::new(content.min.x, top), Pos2::new(content.max.x - if wide { detail_w + 24.0 } else { 0.0 }, bottom));
    // The bar floats in the gutter right of the cards.
    let (cols, cw, ch) = card_grid(grid_r.width() - super::scroll::GUTTER);
    let pos = vis.iter().position(|&i| i == st.plan.aircraft).unwrap_or(0) as i32;
    let (n, c) = (vis.len() as i32, cols as i32);
    let mut to = pos;
    if nav.dx != 0 {
        to = (to + nav.dx).rem_euclid(n);
    }
    if nav.dy != 0 {
        to = (to + nav.dy * c).clamp(0, n - 1);
        // Down from the row above a short last row lands on its last card.
        if nav.dy > 0 && to == pos && pos / c != (n - 1) / c {
            to = n - 1;
        }
    }
    if nav.ends != 0 {
        to = if nav.ends < 0 { 0 } else { n - 1 };
    }
    if to != pos {
        select_aircraft(st, vis[to as usize]);
        reveal = reveal.or(Some((None, true)));
    }
    // Opening the screen: the selection in view, without animating. (For a
    // few frames: the first is egui's sizing pass for the new menu area.)
    let entry_id = Id::new("aircraft_screen_entry");
    let (mut first, last) = ui.data(|d| d.get_temp::<(u64, u64)>(entry_id)).unwrap_or((0, 0));
    if last + 1 < st.frame_no || first == 0 {
        first = st.frame_no;
    }
    if st.frame_no < first + 3 {
        reveal = Some((Some(egui::Align::Center), false));
    }
    ui.data_mut(|d| d.insert_temp(entry_id, (first, st.frame_no)));
    // ---- the grid
    let sel = st.plan.aircraft;
    let mut clicked = None;
    let mut open = false;
    let fade = Color32::from_rgba_premultiplied(5, 7, 10, 250);
    super::scroll::area(ui, grid_r, "aircraft_grid", fade, |ui| {
        let rows = vis.len().div_ceil(cols);
        let total = Vec2::new(grid_r.width(), rows as f32 * (ch + CARD_GAP) - CARD_GAP + 4.0);
        let (area, _) = ui.allocate_exact_size(total, Sense::hover());
        for (k, &i) in vis.iter().enumerate() {
            let (row, col) = (k / cols, k % cols);
            let r = Rect::from_min_size(area.min + Vec2::new(col as f32 * (cw + CARD_GAP), row as f32 * (ch + CARD_GAP)), Vec2::new(cw, ch));
            let resp = ui.interact(r, ui.id().with(("ac", i)), Sense::click());
            let d = &st.defs[i];
            let p = ui.painter();
            let is_sel = sel == i;
            panel_with(p, r, sp::R_CARD, if resp.hovered() { pal::RAISED } else { pal::PANEL_SOLID });
            let img = Rect::from_min_size(r.min + Vec2::splat(6.0), Vec2::new(cw - 12.0, cw * 0.62 - 6.0));
            match st.thumbs.get(d.id) {
                Some(t) => {
                    p.add(Shape::Rect(epaint::RectShape::filled(img, 10, Color32::WHITE).with_texture(*t, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)))));
                }
                None => {
                    gradient_rect(p, img, Color32::from_rgb(70, 120, 180), Color32::from_rgb(170, 200, 230));
                }
            }
            let tw = cw - 32.0;
            // Long names step down a size before they are shortened.
            let font = [16.0, 15.0, 14.0].into_iter().map(f_semi).find(|f| p.layout_no_wrap(d.name.into(), f.clone(), pal::TEXT).size().x <= tw).unwrap_or(f_semi(14.0));
            p.text(Pos2::new(r.min.x + 16.0, img.max.y + 12.0 + (16.0 - font.size) * 0.5), Align2::LEFT_TOP, fit_text(p, d.name, font.clone(), tw), font, pal::TEXT);
            p.text(Pos2::new(r.min.x + 16.0, img.max.y + 34.0), Align2::LEFT_TOP, fit_text(p, class_name(d), f_ui(12.5), tw), f_ui(12.5), pal::TEXT3);
            if is_sel {
                p.rect_stroke(r, sp::R_CARD, Stroke::new(2.0, pal::ACCENT), egui::StrokeKind::Inside);
                if let Some((align, animate)) = reveal {
                    // (The margin keeps the outline and a sliver of the neighbours in view.)
                    let a = if animate { ui.style().scroll_animation } else { egui::style::ScrollAnimation::none() };
                    ui.scroll_to_rect_animation(r.expand(12.0), align, a);
                }
            }
            if resp.clicked() {
                clicked = Some(i);
                ui.scroll_to_rect(r.expand(12.0), None);
            }
            if resp.double_clicked() {
                open = true;
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }
        }
    });
    if let Some(i) = clicked {
        select_aircraft(st, i);
    }
    // ---- details (scrolls when the window is short)
    let d = st.defs[st.plan.aircraft.min(n_all - 1)].clone();
    if wide {
        let dr = Rect::from_min_max(Pos2::new(content.max.x - detail_w, top), Pos2::new(content.max.x, bottom));
        panel(&p, dr, sp::R_CARD);
        let watch = &mut st.plan.watch;
        let thumb = st.thumbs.get(d.id).copied();
        super::scroll::page(ui, dr.shrink(2.0), "aircraft_detail", pal::PANEL_SOLID, |ui, o, w| {
            let p = ui.painter().clone();
            let x = o.x + 20.0;
            let tw = w - 40.0 - super::scroll::GUTTER * 0.5;
            let img = Rect::from_min_size(o + Vec2::splat(6.0), Vec2::new(w - 12.0, ((w - 12.0) * 0.56).min(dr.height() * 0.42)));
            if let Some(t) = thumb {
                // Crop to the box's aspect (the thumbnail is 16:10).
                let a = img.width() / img.height() / 1.786;
                let uv = if a >= 1.0 { Rect::from_min_max(Pos2::new(0.0, 0.5 - 0.5 / a), Pos2::new(1.0, 0.5 + 0.5 / a)) } else { Rect::from_min_max(Pos2::new(0.5 - 0.5 * a, 0.0), Pos2::new(0.5 + 0.5 * a, 1.0)) };
                p.add(Shape::Rect(epaint::RectShape::filled(img, 10, Color32::WHITE).with_texture(t, uv)));
            }
            let mut yy = img.max.y + 18.0;
            let g = p.layout(d.name.to_string(), f_display(26.0), pal::TEXT, tw);
            let gh = g.size().y;
            p.galley(Pos2::new(x, yy), g, pal::TEXT);
            yy += gh + 6.0;
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, class_name(&d).to_uppercase(), f_inst_b(11.5), pal::ACCENT);
            yy += 24.0;
            let g = p.layout(d.description.to_string(), f_ui(14.0), pal::TEXT2, tw);
            let gh = g.size().y;
            p.galley(Pos2::new(x, yy), g, pal::TEXT2);
            yy += gh + 16.0;
            let kw = (tw * 0.36).clamp(110.0, 148.0);
            for (k, v) in spec_lines(&d) {
                p.text(Pos2::new(x, yy), Align2::LEFT_TOP, k, f_ui(13.0), pal::TEXT3);
                let g = p.layout(v, f_ui(13.0), pal::TEXT, tw - kw);
                let gh = g.size().y;
                p.galley(Pos2::new(x + kw, yy), g, pal::TEXT);
                yy += gh.max(16.0) + 6.0;
            }
            yy += 12.0;
            let wr = Rect::from_min_size(Pos2::new(x, yy), Vec2::new(tw, 40.0));
            let label = if d.has_engine { "Auto flight: take-off, tour, landing" } else { "Auto flight: soar thermal to thermal" };
            toggle_row(ui, wr, "watch", label, watch);
            wr.max.y + 20.0 - o.y
        });
    }
    // ---- footer
    let fy = content.max.y - 52.0;
    if button(ui, Rect::from_min_size(Pos2::new(content.min.x, fy), Vec2::new(140.0, 48.0)), "back", "Back", Btn::Secondary, true).clicked() || nav.back {
        st.actions.push(Action::Goto(Screen::Main));
    }
    let next_r = Rect::from_min_size(Pos2::new(content.max.x - 220.0, fy), Vec2::new(220.0, 48.0));
    if button(ui, next_r, "next", "Choose departure  ›", Btn::Primary, true).clicked() || nav.ok || open {
        st.actions.push(Action::Goto(Screen::Departure));
    }
    // Between the buttons: the selection (narrow windows), else the keys.
    let mid = Rect::from_min_max(Pos2::new(content.min.x + 160.0, fy), Pos2::new(next_r.min.x - 20.0, fy + 48.0));
    if !wide {
        p.text(Pos2::new(mid.max.x, mid.center().y - 9.0), Align2::RIGHT_CENTER, fit_text(&p, d.name, f_semi(15.0), mid.width()), f_semi(15.0), pal::TEXT);
        p.text(Pos2::new(mid.max.x, mid.center().y + 11.0), Align2::RIGHT_CENTER, fit_text(&p, class_name(&d), f_ui(12.5), mid.width()), f_ui(12.5), pal::TEXT3);
    } else {
        let pad = st.input.devices.iter().any(|d| d.info.kind == crate::input::DevKind::Gamepad);
        let hints: &[(&str, &str)] = if pad { &[("D-pad", "Browse"), ("LB  RB", "Family"), ("A", "Continue")] } else { &[("← → ↑ ↓", "Browse"), ("Tab", "Family"), ("Enter", "Continue")] };
        key_hints(&p, mid, hints);
    }
}

/// Key hints ("[Tab] Family") centred in `r`, if they fit.
fn key_hints(p: &egui::Painter, r: Rect, hints: &[(&str, &str)]) {
    let items: Vec<_> = hints.iter().map(|(k, l)| (p.layout_no_wrap(k.to_string(), f_semi(11.5), pal::TEXT2), p.layout_no_wrap(l.to_string(), f_ui(12.5), pal::TEXT3))).collect();
    let w: f32 = items.iter().map(|(k, l)| k.size().x + 14.0 + 8.0 + l.size().x).sum::<f32>() + 24.0 * (items.len() as f32 - 1.0);
    if w > r.width() {
        return;
    }
    let mut x = r.center().x - w * 0.5;
    for (k, l) in items {
        let kr = Rect::from_min_size(Pos2::new(x, r.center().y - 11.0), Vec2::new(k.size().x + 14.0, 22.0));
        p.rect_filled(kr, 5, pal::RAISED);
        p.rect_stroke(kr, 5, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
        p.galley(kr.center() - k.size() * 0.5, k, pal::TEXT2);
        x = kr.max.x + 8.0;
        p.galley(Pos2::new(x, r.center().y - l.size().y * 0.5), l.clone(), pal::TEXT3);
        x += l.size().x + 24.0;
    }
}

/// Select a type (and drop a start position it cannot use).
fn select_aircraft(st: &mut UiState, i: usize) {
    st.plan.aircraft = i;
    let d = &st.defs[i];
    if !st.plan.spec.valid_for(d) {
        st.plan.spec = if d.has_engine { Spec::Runway(String::new()) } else { Spec::Thermal };
    }
}

/// `text` shortened with an ellipsis to fit `max_w` points.
fn fit_text(p: &egui::Painter, text: &str, font: egui::FontId, max_w: f32) -> String {
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

/// A row with a label and a switch.
fn toggle_row(ui: &mut Ui, r: Rect, id: &str, label: &str, v: &mut bool) -> bool {
    let resp = ui.interact(r, ui.id().with(id), Sense::click());
    let p = ui.painter();
    p.rect_filled(r, sp::R_CTRL, if resp.hovered() { pal::RAISED_HI } else { pal::RAISED });
    p.text(Pos2::new(r.min.x + 14.0, r.center().y), Align2::LEFT_CENTER, label, f_ui(14.0), pal::TEXT);
    let sw = Rect::from_center_size(Pos2::new(r.max.x - 32.0, r.center().y), Vec2::new(40.0, 22.0));
    p.rect_filled(sw, 11.0, if *v { pal::ACCENT } else { Color32::from_gray(70) });
    p.circle_filled(Pos2::new(if *v { sw.max.x - 11.0 } else { sw.min.x + 11.0 }, sw.center().y), 8.5, Color32::WHITE);
    if resp.clicked() {
        *v = !*v;
        return true;
    }
    false
}

// ---------------------------------------------------------------- departure

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Search,
    Recent,
    Favourites,
}

pub struct Picker {
    pub query: String,
    last_query: String,
    last_filter: (usize, bool),
    pub results: Vec<usize>,
    pub tab: Tab,
    pub show_all: bool,
    pub view: OView,
    target: Option<(f64, f64, f64)>,
    pub selected: Option<usize>,
    pub air_point: Option<(f64, f64)>,
    pub air_alt_ft: f32,
    pub air_hdg: f32,
    pub metar: Option<airports::MetarFetch>,
    focus: bool,
    /// The runway end was chosen automatically: re-choose into wind once the METAR is in.
    auto_end: Option<String>,
    pub overview: overview::Overview,
}

impl Default for Picker {
    fn default() -> Self {
        let (cx, cy) = overview::merc(51.5, 0.0);
        Picker {
            query: String::new(),
            last_query: "\u{0}".into(),
            last_filter: (usize::MAX, false),
            results: Vec::new(),
            tab: Tab::Search,
            show_all: false,
            view: OView { cx, cy, zoom: 5.5, rect: Rect::from_min_size(Pos2::ZERO, Vec2::splat(100.0)) },
            target: None,
            selected: None,
            air_point: None,
            air_alt_ft: 3000.0,
            air_hdg: 0.0,
            metar: None,
            focus: true,
            auto_end: None,
            overview: overview::Overview::new(),
        }
    }
}

impl Picker {
    /// Select an airport (and fly the map there).
    pub fn select(&mut self, db: &Db, i: usize, zoom: f64) {
        self.selected = Some(i);
        let a = &db.airports[i];
        let (x, y) = overview::merc(a.lat, a.lon);
        self.target = Some((x, y, zoom));
        if self.metar.as_ref().is_none_or(|m| m.ident != a.ident) {
            self.metar = Some(airports::MetarFetch::start(&a.ident, &a.icao, a.lat, a.lon));
        }
    }

    fn animate(&mut self, dt: f32) {
        if let Some((x, y, z)) = self.target {
            let k = (dt as f64 * 6.0).min(1.0);
            let mut dx = x - self.view.cx;
            dx -= dx.round();
            self.view.cx += dx * k;
            self.view.cy += (y - self.view.cy) * k;
            self.view.zoom += (z - self.view.zoom) * k;
            if dx.abs() < 1e-7 && (y - self.view.cy).abs() < 1e-7 && (z - self.view.zoom).abs() < 1e-3 {
                self.target = None;
            }
        }
    }
}

/// Wind for runway-end badges: (from deg, kt, gust) from the fetched METAR.
fn picker_wind(pk: &Picker) -> Option<(f32, f32, Option<f32>, crate::weather::Metar)> {
    match pk.metar.as_ref()?.result.as_ref()? {
        Ok((m, _)) => Some((m.wind_dir.unwrap_or(0.0), m.wind_kt, m.gust_kt, m.clone())),
        Err(_) => None,
    }
}

pub fn departure(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    p.rect_filled(screen, 0, pal::BG);
    let Some(db) = st.db.clone() else {
        backdrop(&p, screen, 1.0);
        let msg = match &st.airport_status {
            Some(e) => format!("Airport data unavailable: {e}"),
            None => "Loading airports…".into(),
        };
        p.text(screen.center(), Align2::CENTER_CENTER, msg, f_ui(16.0), pal::TEXT2);
        if button(ui, Rect::from_center_size(screen.center() + Vec2::new(0.0, 60.0), Vec2::new(140.0, 44.0)), "back", "Back", Btn::Secondary, true).clicked() || esc(ui) {
            st.actions.push(Action::Goto(Screen::Aircraft));
        }
        return;
    };
    let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].clone();
    st.picker.animate(st.dt);
    if st.picker.selected.is_none() {
        if let Some(i) = db.index_of(&st.plan.airport) {
            st.picker.select(&db, i, 13.0);
            if matches!(&st.plan.spec, Spec::Runway(e) | Spec::Final(e, _) if e.is_empty()) {
                pick_default_spec(st, &db, i);
            }
            st.picker.view.cx = st.picker.target.map(|t| t.0).unwrap_or(st.picker.view.cx);
            st.picker.view.cy = st.picker.target.map(|t| t.1).unwrap_or(st.picker.view.cy);
        }
    }
    // ---- layout: list (left) | map (centre) | details (right)
    let list_w = 380.0;
    let det_w = 420.0;
    let left = Rect::from_min_max(screen.min, Pos2::new(screen.min.x + list_w, screen.max.y));
    let right = Rect::from_min_max(Pos2::new(screen.max.x - det_w, screen.min.y), screen.max);
    let map_r = Rect::from_min_max(Pos2::new(left.max.x, screen.min.y), Pos2::new(right.min.x, screen.max.y));
    st.picker.view.rect = map_r;
    // ---- map
    let dark = true;
    let ppp = ui.ctx().pixels_per_point();
    let taken = st.picker.overview.draw(&p, &mut st.canvas, "overview", ppp, &st.picker.view, dark);
    let map_resp = ui.interact(map_r, ui.id().with("ovmap"), Sense::click_and_drag());
    draw_airports_layer(ui, &p, st, &db, &def, map_r, &taken, &map_resp);
    if map_resp.dragged() {
        let d = map_resp.drag_delta();
        let s = st.picker.view.scale();
        st.picker.view.cx -= d.x as f64 / s;
        st.picker.view.cy = (st.picker.view.cy - d.y as f64 / s).clamp(0.0, 1.0);
        st.picker.target = None;
    }
    if map_resp.hovered() {
        let (scroll, pos) = ui.input(|i| (i.smooth_scroll_delta.y, i.pointer.hover_pos()));
        if scroll.abs() > 0.1 {
            if let Some(pos) = pos {
                st.picker.view.zoom_at(pos, scroll as f64 * 0.004);
                st.picker.target = None;
            }
        }
    }
    // map controls (zoom buttons)
    for (k, (lbl, dz)) in [("+", 1.0), ("−", -1.0)].into_iter().enumerate() {
        let r = Rect::from_min_size(Pos2::new(map_r.max.x - 52.0, map_r.min.y + 76.0 + k as f32 * 44.0), Vec2::splat(38.0));
        if button(ui, r, ("zoom", k), lbl, Btn::Secondary, true).clicked() {
            let c = st.picker.view.rect.center();
            st.picker.view.zoom_at(c, dz);
            st.picker.target = None;
        }
    }
    p.text(Pos2::new(map_r.max.x - 10.0, map_r.max.y - 8.0), Align2::RIGHT_BOTTOM, "© OpenMapTiles © OpenStreetMap contributors · OurAirports", f_ui(10.5), Color32::from_white_alpha(140));
    // ---- top bar over the map
    let bar = Rect::from_min_max(Pos2::new(map_r.min.x, map_r.min.y), Pos2::new(map_r.max.x, map_r.min.y + 64.0));
    gradient_rect(&p, bar, Color32::from_rgba_premultiplied(6, 8, 11, 220), Color32::TRANSPARENT);
    if let Some(s) = steps(ui, bar.min + Vec2::new(20.0, 16.0), 1) {
        st.actions.push(Action::Goto([Screen::Aircraft, Screen::Departure, Screen::Conditions][s]));
    }
    // ---- list panel
    list_panel(ui, st, &db, &def, left);
    // ---- details panel
    details_panel(ui, st, &db, &def, right);
}

fn draw_airports_layer(ui: &mut Ui, p: &egui::Painter, st: &mut UiState, db: &Arc<Db>, def: &AircraftDef, map_r: Rect, taken: &[Rect], resp: &Response) {
    let v = st.picker.view;
    let (lat0, lat1, lon0, lon1) = v.bounds();
    let z = v.zoom;
    let mp = p.with_clip_rect(map_r);
    let mut hover: Option<(usize, f32)> = None;
    let hp = ui.input(|i| i.pointer.hover_pos());
    let mut labels: Vec<Rect> = taken.to_vec();
    let mut idx = db.in_box(lat0.min(lat1) - 0.5, lat0.max(lat1) + 0.5, lon0.min(lon1) - 0.5, lon0.max(lon1) + 0.5);
    idx.sort_by_key(|&i| std::cmp::Reverse(db.airports[i].kind));
    let fav = |a: &Airport| st.settings.is_favourite(&a.ident);
    for i in idx {
        let a = &db.airports[i];
        let show = match a.kind {
            Kind::Large => true,
            Kind::Medium => z > 4.3,
            Kind::Small => z > 6.6,
            Kind::Seaplane => z > 8.0 && st.picker.show_all,
        };
        if !show && Some(i) != st.picker.selected {
            continue;
        }
        let ok = suitable(a, def);
        if !ok && !st.picker.show_all && Some(i) != st.picker.selected {
            continue;
        }
        let q = v.ll_to_screen(a.lat, a.lon);
        if !map_r.contains(q) {
            continue;
        }
        let r = match a.kind {
            Kind::Large => 5.5,
            Kind::Medium => 4.2,
            _ => 3.0,
        } * (z as f32 / 6.0).clamp(0.8, 1.4);
        let col = if !ok {
            Color32::from_gray(110)
        } else {
            match a.kind {
                Kind::Large => Color32::from_rgb(92, 178, 255),
                Kind::Medium => Color32::from_rgb(120, 200, 255),
                _ => Color32::from_rgb(214, 120, 214),
            }
        };
        if z < 11.0 || Some(i) != st.picker.selected {
            mp.circle_filled(q, r + 1.5, Color32::from_black_alpha(160));
            mp.circle_filled(q, r, col);
            if fav(a) {
                mp.text(q + Vec2::new(r + 2.0, -r - 2.0), Align2::LEFT_BOTTOM, "★", f_ui(11.0), pal::AMBER);
            }
        }
        if let Some(h) = hp {
            let d = h.distance(q);
            if d < 12.0 && hover.is_none_or(|(_, dd)| d < dd) {
                hover = Some((i, d));
            }
        }
        // code labels when there is room
        if z > 5.0 && (a.kind != Kind::Small || z > 8.0) && (z < 11.0 || Some(i) != st.picker.selected) {
            let g = mp.layout_no_wrap(a.code().to_string(), f_inst_b(10.5), col);
            let lr = Rect::from_min_size(q + Vec2::new(r + 4.0, -g.size().y * 0.5), g.size());
            if !labels.iter().any(|t| t.intersects(lr)) {
                labels.push(lr);
                mp.galley(lr.min, g, col);
            }
        }
    }
    // selected airport: diagram at high zoom, ring at low zoom
    if let Some(si) = st.picker.selected {
        let a = &db.airports[si];
        let q = v.ll_to_screen(a.lat, a.lon);
        if z >= 11.0 {
            airport_diagram(ui, &mp, st, a, &v);
        } else {
            mp.circle_stroke(q, 11.0, Stroke::new(2.5, pal::AMBER));
            mp.circle_stroke(q, 15.0, Stroke::new(1.0, alpha(pal::AMBER, 0.5)));
        }
    }
    // air start marker
    if let Some((la, lo)) = st.picker.air_point {
        let q = v.ll_to_screen(la, lo);
        let h = st.picker.air_hdg.to_radians();
        super::map::aircraft_symbol(&mp, q, h, 16.0, pal::AMBER);
    }
    // hover tooltip
    if let Some((i, _)) = hover.filter(|_| !resp.dragged()) {
        let a = &db.airports[i];
        let q = v.ll_to_screen(a.lat, a.lon);
        let text = format!("{}  {}", a.code(), a.name);
        let sub = format!("{} · longest {} m", if a.city.is_empty() { a.country.clone() } else { format!("{}, {}", a.city, a.country) }, thousands(a.longest_m()));
        let g1 = p.layout_no_wrap(text, f_semi(13.0), pal::TEXT);
        let g2 = p.layout_no_wrap(sub, f_ui(12.0), pal::TEXT2);
        let w = g1.size().x.max(g2.size().x) + 20.0;
        let r = Rect::from_min_size(q + Vec2::new(14.0, -24.0), Vec2::new(w, 46.0));
        panel_with(p, r, 8, pal::PANEL_SOLID);
        p.galley(r.min + Vec2::new(10.0, 6.0), g1, pal::TEXT);
        p.galley(r.min + Vec2::new(10.0, 25.0), g2, pal::TEXT2);
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        if resp.clicked() {
            st.picker.select(db, i, v.zoom.max(12.0));
            st.picker.air_point = None;
            pick_default_spec(st, db, i);
        }
    } else if resp.clicked() && z >= 6.0 {
        // empty map: an air start here (nearest airport becomes the reference)
        if let Some(pos) = resp.interact_pointer_pos() {
            let (la, lo) = v.to_ll(pos);
            st.picker.air_point = Some((la, lo));
            st.plan.spec = Spec::Air { lat: la, lon: lo, alt_m: st.picker.air_alt_ft / 3.280_84, hdg: st.picker.air_hdg };
            if let Some((ni, _)) = db.near(la, lo, 200.0).first() {
                st.picker.selected = Some(*ni);
                st.plan.airport = db.airports[*ni].ident.clone();
                let a = &db.airports[*ni];
                if st.picker.metar.as_ref().is_none_or(|m| m.ident != a.ident) {
                    st.picker.metar = Some(airports::MetarFetch::start(&a.ident, &a.icao, a.lat, a.lon));
                }
            }
        }
    }
}

/// Pick a sensible start at a newly selected airport: the runway end most into wind (when known) or the longest.
pub fn pick_default_spec(st: &mut UiState, db: &Db, i: usize) {
    let a = &db.airports[i];
    st.plan.airport = a.ident.clone();
    let def = &st.defs[st.plan.aircraft.min(st.defs.len() - 1)];
    if !def.has_engine {
        st.plan.spec = Spec::Thermal;
        return;
    }
    let keep_final = matches!(st.plan.spec, Spec::Final(..));
    let nm = if let Spec::Final(_, nm) = st.plan.spec { nm } else { 5.0 };
    let Some(rw) = a.runways.iter().filter(|r| !r.ends[0].ident.starts_with('H')).max_by(|x, y| x.length_m.total_cmp(&y.length_m)) else {
        st.plan.spec = Spec::Air { lat: a.lat, lon: a.lon, alt_m: 900.0, hdg: 0.0 };
        return;
    };
    let wind = picker_wind(&st.picker);
    let end = match (wind, rw.heading(0)) {
        (Some((from, kt, ..)), Some(h0)) if kt >= 3.0 => {
            if airports::wind_components(h0, from, kt).0 >= 0.0 { 0 } else { 1 }
        }
        _ => 0,
    };
    let id = rw.ends[end].ident.clone();
    st.plan.spec = if keep_final { Spec::Final(id, nm) } else { Spec::Runway(id) };
    // Until the weather is known the choice is provisional.
    st.picker.auto_end = if picker_wind(&st.picker).is_none() { Some(a.ident.clone()) } else { None };
}

/// Runways of the selected airport drawn to scale, with clickable ends.
fn airport_diagram(ui: &mut Ui, p: &egui::Painter, st: &mut UiState, a: &Airport, v: &OView) {
    let wind = picker_wind(&st.picker);
    for (ri, rw) in a.runways.iter().enumerate() {
        let (pa, pb) = match (rw.ends[0].lat, rw.ends[0].lon, rw.ends[1].lat, rw.ends[1].lon) {
            (Some(la), Some(lo), Some(lb), Some(lob)) => (v.ll_to_screen(la, lo), v.ll_to_screen(lb, lob)),
            _ => {
                let Some(h) = rw.heading(0) else { continue };
                let c = v.ll_to_screen(a.lat, a.lon);
                let mpp = 1.0 / (v.scale() / 40_075_000.0 * (1.0 / a.lat.to_radians().cos())) as f32;
                let d = Vec2::new(h.to_radians().sin(), -h.to_radians().cos()) * rw.length_m * 0.5 / mpp;
                (c - d, c + d)
            }
        };
        let m_per_pt = (40_075_000.0 * a.lat.to_radians().cos() / v.scale()) as f32;
        let w = (rw.width_m.max(18.0) / m_per_pt).max(4.0);
        let paved = rw.surface.paved();
        p.line_segment([pa, pb], Stroke::new(w + 2.0, Color32::from_black_alpha(200)));
        p.line_segment([pa, pb], Stroke::new(w, if paved { Color32::from_rgb(120, 126, 140) } else { Color32::from_rgb(96, 136, 92) }));
        let d = (pb - pa).normalized();
        // centreline
        if w > 6.0 {
            let len = (pb - pa).length();
            let mut t = 0.06;
            while t < 0.94 {
                p.line_segment([pa + d * len * t, pa + d * len * (t + 0.025)], Stroke::new(1.2, Color32::WHITE));
                t += 0.05;
            }
        }
        // displaced thresholds
        for (end, (pt, dir)) in [(pa, d), (pb, -d)].into_iter().enumerate() {
            let disp = rw.ends[end].displaced_m / m_per_pt;
            if disp > 1.0 {
                let q = pt + dir * disp;
                let n = dir.rot90() * w * 0.5;
                p.line_segment([q - n, q + n], Stroke::new(2.0, Color32::WHITE));
            }
        }
        // ends: badges with ident + wind component, clickable
        for (end, (pt, dir)) in [(pa, d), (pb, -d)].into_iter().enumerate() {
            let e = &rw.ends[end];
            if e.ident.is_empty() {
                continue;
            }
            let at = pt - dir * 26.0;
            let comp = wind.as_ref().and_then(|(from, kt, _, _)| rw.heading(end).map(|h| airports::wind_components(h, *from, *kt)));
            let selected = match &st.plan.spec {
                Spec::Runway(x) | Spec::Final(x, _) => *x == e.ident && st.plan.airport == a.ident,
                _ => false,
            };
            let col = match comp {
                Some((h, x)) if h >= 0.0 && x.abs() < 15.0 => pal::GREEN,
                Some((h, _)) if h < -3.0 => pal::RED,
                Some(_) => pal::AMBER,
                None => pal::TEXT,
            };
            let r = Rect::from_center_size(at, Vec2::new(46.0, 26.0));
            let resp = ui.interact(r, ui.id().with(("rwend", ri, end)), Sense::click());
            p.rect_filled(r, 6, if selected { pal::ACCENT_DEEP } else if resp.hovered() { pal::RAISED_HI } else { pal::PANEL_SOLID });
            p.rect_stroke(r, 6, Stroke::new(if selected { 2.0 } else { 1.0 }, if selected { pal::ACCENT } else { col }), egui::StrokeKind::Inside);
            p.text(r.center(), Align2::CENTER_CENTER, &e.ident, f_inst_b(13.0), pal::TEXT);
            if resp.clicked() {
                st.plan.airport = a.ident.clone();
                st.picker.air_point = None;
                st.picker.auto_end = None;
                st.plan.spec = match &st.plan.spec {
                    Spec::Final(_, nm) => Spec::Final(e.ident.clone(), *nm),
                    _ => Spec::Runway(e.ident.clone()),
                };
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }
        }
        // final approach path for the chosen end
        if let Spec::Final(x, nm) = &st.plan.spec {
            for (end, (pt, dir)) in [(pa, d), (pb, -d)].into_iter().enumerate() {
                if rw.ends[end].ident == *x && st.plan.airport == a.ident {
                    let m_per_pt = (40_075_000.0 * a.lat.to_radians().cos() / v.scale()) as f32;
                    let far = pt - dir * (nm * 1852.0 / m_per_pt);
                    for k in 0..20 {
                        let t0 = k as f32 / 20.0;
                        p.line_segment([pt + (far - pt) * t0, pt + (far - pt) * (t0 + 0.025)], Stroke::new(2.0, pal::MAGENTA));
                    }
                    super::map::aircraft_symbol(p, far, dir.x.atan2(-dir.y), 14.0, pal::AMBER);
                }
            }
        }
    }
}

fn list_panel(ui: &mut Ui, st: &mut UiState, db: &Arc<Db>, def: &AircraftDef, r: Rect) {
    let p = ui.painter().clone();
    p.rect_filled(r, 0, pal::PANEL_SOLID);
    p.line_segment([r.right_top(), r.right_bottom()], Stroke::new(1.0, pal::HAIRLINE_HI));
    let x = r.min.x + 20.0;
    p.text(Pos2::new(x, r.min.y + 22.0), Align2::LEFT_TOP, "Departure", f_display(26.0), pal::TEXT);
    // search box
    let sb = Rect::from_min_size(Pos2::new(x, r.min.y + 66.0), Vec2::new(r.width() - 40.0, 40.0));
    p.rect_filled(sb, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    p.rect_stroke(sb, sp::R_CTRL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
    let te = ui.put(sb.shrink2(Vec2::new(12.0, 8.0)), egui::TextEdit::singleline(&mut st.picker.query).hint_text("Search name, city, ICAO or IATA").frame(egui::Frame::NONE).font(f_ui(15.0)).text_color(pal::TEXT));
    if st.picker.focus {
        te.request_focus();
        st.picker.focus = false;
    }
    if te.changed() {
        st.picker.tab = Tab::Search;
    }
    // filter chips
    let (len, paved) = needs(def);
    let cy = sb.max.y + 12.0;
    let suit = format!("Suits {} · {} m{}", def.name, thousands(len), if paved { " paved" } else { "" });
    let g = p.layout_no_wrap(suit.clone(), f_semi(13.0), pal::TEXT);
    let c1 = Rect::from_min_size(Pos2::new(x, cy), Vec2::new(g.size().x + 26.0, 30.0));
    if chip_btn(ui, c1, "suit", &suit, !st.picker.show_all) {
        st.picker.show_all = false;
    }
    let c2 = Rect::from_min_size(Pos2::new(c1.max.x + 8.0, cy), Vec2::new(64.0, 30.0));
    if chip_btn(ui, c2, "all", "All", st.picker.show_all) {
        st.picker.show_all = true;
    }
    // tabs
    let ty = cy + 44.0;
    let tabs = ["Results", "Recent", "Favourites"];
    if let Some(t) = segmented(ui, Rect::from_min_size(Pos2::new(x, ty), Vec2::new(r.width() - 40.0, 34.0)), "tabs", &tabs, st.picker.tab as usize) {
        st.picker.tab = [Tab::Search, Tab::Recent, Tab::Favourites][t];
    }
    // results
    let filt = (st.plan.aircraft, st.picker.show_all);
    if st.picker.query != st.picker.last_query || filt != st.picker.last_filter {
        st.picker.last_query = st.picker.query.clone();
        st.picker.last_filter = filt;
        let show_all = st.picker.show_all;
        st.picker.results = if st.picker.query.trim().is_empty() {
            // nearby to the map centre: the biggest fields first
            let (lat, lon) = overview::unmerc(st.picker.view.cx.rem_euclid(1.0), st.picker.view.cy);
            // Nearby fields, with bigger ones pulled forward.
            let mut v: Vec<(usize, f64)> = db
                .near(lat, lon, 250.0)
                .into_iter()
                .filter(|&(i, _)| show_all || suitable(&db.airports[i], def))
                .map(|(i, d)| {
                    let k = match db.airports[i].kind {
                        Kind::Large => 0.35,
                        Kind::Medium => 0.6,
                        _ => 1.0,
                    };
                    (i, d * k)
                })
                .collect();
            v.sort_by(|a, b| a.1.total_cmp(&b.1));
            v.truncate(40);
            v.into_iter().map(|x| x.0).collect()
        } else {
            db.search(&st.picker.query, 60, |a| show_all || suitable(a, def))
        };
    }
    let list: Vec<usize> = match st.picker.tab {
        Tab::Search => st.picker.results.clone(),
        Tab::Recent => st.settings.recents.iter().filter_map(|c| db.index_of(c)).collect(),
        Tab::Favourites => st.settings.favourites.iter().filter_map(|c| db.index_of(c)).collect(),
    };
    let lr = Rect::from_min_max(Pos2::new(r.min.x, ty + 46.0), Pos2::new(r.max.x, r.max.y - 72.0));
    if list.is_empty() {
        let msg = match st.picker.tab {
            Tab::Search if st.picker.query.trim().is_empty() => "Pan the map or search",
            Tab::Search => "No matching airports",
            Tab::Recent => "Airports you fly from appear here",
            Tab::Favourites => "Star an airport to keep it here",
        };
        p.text(Pos2::new(r.center().x, lr.min.y + 40.0), Align2::CENTER_TOP, msg, f_ui(14.0), pal::TEXT3);
    }
    let mut clicked = None;
    let enter = te.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
    if enter {
        clicked = list.first().copied();
    }
    super::scroll::area(ui, lr, "airport_list", pal::PANEL_SOLID, |ui| {
        for &i in &list {
            let a = &db.airports[i];
            let (row, resp) = ui.allocate_exact_size(Vec2::new(lr.width(), 62.0), Sense::click());
            let p = ui.painter();
            let sel = st.picker.selected == Some(i);
            if sel {
                p.rect_filled(row.shrink2(Vec2::new(10.0, 2.0)), 8, pal::ACCENT_BG);
                p.rect_filled(Rect::from_min_size(row.min + Vec2::new(10.0, 10.0), Vec2::new(3.0, 42.0)), 2, pal::ACCENT);
            } else if resp.hovered() {
                p.rect_filled(row.shrink2(Vec2::new(10.0, 2.0)), 8, Color32::from_white_alpha(7));
            }
            let ok = suitable(a, def);
            p.text(Pos2::new(row.min.x + 24.0, row.min.y + 11.0), Align2::LEFT_TOP, a.code(), f_inst_b(14.0), if ok { pal::ACCENT } else { pal::TEXT3 });
            if !a.iata.is_empty() {
                p.text(Pos2::new(row.min.x + 82.0, row.min.y + 12.0), Align2::LEFT_TOP, &a.iata, f_inst(12.0), pal::TEXT3);
            }
            let name = clip_text(&a.name, 34);
            p.text(Pos2::new(row.min.x + 120.0, row.min.y + 10.0), Align2::LEFT_TOP, name, f_semi(14.0), pal::TEXT);
            let place = if a.city.is_empty() { a.country.clone() } else { format!("{}, {}", clip_text(&a.city, 22), a.country) };
            p.text(Pos2::new(row.min.x + 24.0, row.min.y + 36.0), Align2::LEFT_TOP, place, f_ui(12.5), pal::TEXT2);
            p.text(Pos2::new(row.max.x - 22.0, row.min.y + 36.0), Align2::RIGHT_TOP, format!("{} m", thousands(a.longest_m())), f_mono(12.0), if ok { pal::TEXT2 } else { pal::AMBER });
            if st.settings.is_favourite(&a.ident) {
                p.text(Pos2::new(row.max.x - 22.0, row.min.y + 10.0), Align2::RIGHT_TOP, "★", f_ui(13.0), pal::AMBER);
            }
            if resp.clicked() {
                clicked = Some(i);
            }
        }
    });
    if let Some(i) = clicked {
        st.picker.select(db, i, 12.5);
        st.picker.air_point = None;
        pick_default_spec(st, db, i);
    }
    // footer: random + back
    let fy = r.max.y - 60.0;
    if button(ui, Rect::from_min_size(Pos2::new(x, fy), Vec2::new(110.0, 44.0)), "back", "Back", Btn::Secondary, true).clicked() || esc(ui) {
        st.actions.push(Action::Goto(Screen::Aircraft));
    }
    if button(ui, Rect::from_min_size(Pos2::new(x + 120.0, fy), Vec2::new(r.width() - 160.0, 44.0)), "random", "Random airport", Btn::Ghost, true).clicked() {
        let pool: Vec<usize> = (0..db.airports.len()).filter(|&i| {
            let a = &db.airports[i];
            matches!(a.kind, Kind::Large | Kind::Medium) && suitable(a, def) && a.runways.iter().any(|r| r.surveyed())
        }).collect();
        if !pool.is_empty() {
            let k = (crate::scene::hash(st.frame_no ^ 0x9e37_79b9) as usize) % pool.len();
            st.picker.select(db, pool[k], 12.0);
            st.picker.air_point = None;
            pick_default_spec(st, db, pool[k]);
        }
    }
}

fn clip_text(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { s.chars().take(n - 1).collect::<String>() + "…" }
}

fn details_panel(ui: &mut Ui, st: &mut UiState, db: &Arc<Db>, def: &AircraftDef, r: Rect) {
    let p = ui.painter().clone();
    p.rect_filled(r, 0, pal::PANEL_SOLID);
    p.line_segment([r.left_top(), r.left_bottom()], Stroke::new(1.0, pal::HAIRLINE_HI));
    let x = r.min.x + 22.0;
    let w = r.width() - 44.0;
    let Some(si) = st.picker.selected else {
        p.text(Pos2::new(r.center().x, r.min.y + 120.0), Align2::CENTER_TOP, "Pick an airport on the map or in the list", f_ui(14.0), pal::TEXT3);
        return;
    };
    let a = db.airports[si].clone();
    if let Some(m) = &mut st.picker.metar {
        m.poll();
    }
    if st.picker.auto_end.as_deref() == Some(a.ident.as_str()) && picker_wind(&st.picker).is_some() && st.plan.airport == a.ident {
        pick_default_spec(st, db, si);
    }
    // Everything above the footer scrolls when the window is short (big
    // airports list all their runways).
    let fy = r.max.y - 60.0;
    let warn = !suitable(&a, def);
    let body = Rect::from_min_max(Pos2::new(r.min.x + 1.0, r.min.y), Pos2::new(r.max.x, fy - if warn { 40.0 } else { 14.0 }));
    super::scroll::page(ui, body, "airport_details", pal::PANEL_SOLID, |ui, o, pw| {
        let p = ui.painter().clone();
        let x = o.x + 21.0;
        let mut y = o.y + 22.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, a.code(), f_inst_b(14.0), pal::ACCENT);
        p.text(Pos2::new(x + 60.0, y + 1.0), Align2::LEFT_TOP, a.kind.label().to_uppercase(), f_inst(11.5), pal::TEXT3);
        // favourite star
        let star = Rect::from_min_size(Pos2::new(o.x + pw - 54.0, y - 6.0), Vec2::splat(34.0));
        let fav = st.settings.is_favourite(&a.ident);
        let sresp = ui.interact(star, ui.id().with("star"), Sense::click());
        p.text(star.center(), Align2::CENTER_CENTER, if fav { "★" } else { "☆" }, f_ui(22.0), if fav { pal::AMBER } else if sresp.hovered() { pal::TEXT } else { pal::TEXT3 });
        if sresp.clicked() {
            st.settings.toggle_favourite(&a.ident);
            st.settings_dirty = true;
        }
        y += 26.0;
        let g = p.layout(a.name.clone(), f_display(22.0), pal::TEXT, w - 30.0);
        let gh = g.size().y;
        p.galley(Pos2::new(x, y), g, pal::TEXT);
        y += gh + 4.0;
        let place = [a.city.as_str(), a.region.as_str(), a.country.as_str()].iter().filter(|s| !s.is_empty()).cloned().collect::<Vec<_>>().join(" · ");
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, format!("{place} · {} ft", thousands(a.elev_ft)), f_ui(13.0), pal::TEXT2);
        y += 28.0;
        // local time + daylight
        let now = crate::weather::unix_now();
        let rt = crate::weather::real_time(a.lat, a.lon, now, None);
        let sun = airports::sun_times(a.lat, a.lon, rt.day_of_year);
        let el = crate::atmosphere::sun_direction(a.lat, a.lon, rt.day_of_year, rt.local_hours).y.asin().to_degrees();
        let daylight = if el > 0.0 { "Daylight" } else if el > -6.0 { "Twilight" } else { "Night" };
        let card = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, 54.0));
        p.rect_filled(card, 8, pal::RAISED);
        p.text(card.min + Vec2::new(14.0, 9.0), Align2::LEFT_TOP, "LOCAL TIME", f_inst(10.5), pal::TEXT3);
        p.text(card.min + Vec2::new(14.0, 24.0), Align2::LEFT_TOP, hhmm(rt.local_hours), f_semi(19.0), pal::TEXT);
        p.text(card.min + Vec2::new(90.0, 28.0), Align2::LEFT_TOP, daylight, f_ui(12.5), if el > 0.0 { pal::AMBER } else { pal::ACCENT });
        if let Some((rise, set)) = sun {
            let off = rt.local_hours - crate::atmosphere::civil_offset_h(a.lat, a.lon, rt.day_of_year) * 0.0;
            let _ = off;
            p.text(Pos2::new(card.max.x - 14.0, card.min.y + 9.0), Align2::RIGHT_TOP, "SUNRISE · SUNSET", f_inst(10.5), pal::TEXT3);
            p.text(Pos2::new(card.max.x - 14.0, card.min.y + 26.0), Align2::RIGHT_TOP, format!("{}  –  {}", hhmm(rise), hhmm(set)), f_semi(15.0), pal::TEXT);
        }
        y = card.max.y + 10.0;
        // METAR
        let mc = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, 82.0));
        p.rect_filled(mc, 8, pal::RAISED);
        match st.picker.metar.as_ref().and_then(|m| m.result.clone()) {
            Some(Ok((m, km))) => {
                let wx = crate::weather::Wx::from_metar_str(&m.raw);
                let cat = wx.as_ref().map(|w| w.category()).unwrap_or("—");
                let catc = match cat {
                    "VFR" => pal::GREEN,
                    "MVFR" => pal::ACCENT,
                    "IFR" => pal::RED,
                    "LIFR" => pal::MAGENTA,
                    _ => pal::TEXT2,
                };
                super::chip(&p, mc.min + Vec2::new(12.0, 10.0), Align2::LEFT_TOP, cat, f_inst_b(11.0), Color32::from_rgb(6, 10, 14), catc);
                let wind = match m.wind_dir {
                    Some(d) if m.wind_kt >= 1.0 => format!("{:03.0}° {:.0}{} kt", d, m.wind_kt, m.gust_kt.map(|g| format!("G{g:.0}")).unwrap_or_default()),
                    None if m.wind_kt >= 1.0 => format!("VRB {:.0} kt", m.wind_kt),
                    _ => "Calm".into(),
                };
                p.text(mc.min + Vec2::new(70.0, 10.0), Align2::LEFT_TOP, wind, f_semi(14.0), pal::TEXT);
                let extra = format!("{}{}{}", m.temp_c.map(|t| format!("{t:.0}°C  ")).unwrap_or_default(), m.qnh_hpa.map(|q| format!("Q{q:.0}  ")).unwrap_or_default(), if km > 3.0 { format!("{} · {km:.0} km", m.station) } else { m.station.clone() });
                p.text(Pos2::new(mc.max.x - 12.0, mc.min.y + 12.0), Align2::RIGHT_TOP, extra, f_ui(12.0), pal::TEXT2);
                let g = p.layout(m.raw.clone(), f_mono(11.0), pal::TEXT3, w - 24.0);
                p.galley(mc.min + Vec2::new(12.0, 38.0), g, pal::TEXT3);
            }
            Some(Err(e)) => {
                p.text(mc.min + Vec2::new(12.0, 12.0), Align2::LEFT_TOP, "No METAR", f_semi(14.0), pal::TEXT2);
                p.text(mc.min + Vec2::new(12.0, 34.0), Align2::LEFT_TOP, clip_text(&e, 50), f_ui(12.0), pal::TEXT3);
            }
            None => {
                p.text(mc.min + Vec2::new(12.0, 12.0), Align2::LEFT_TOP, "Fetching weather…", f_ui(13.0), pal::TEXT3);
            }
        }
        y = mc.max.y + 18.0;
        // runways with per-end wind
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, "RUNWAYS", f_inst_b(11.0), pal::TEXT3);
        y += 22.0;
        let wind = picker_wind(&st.picker);
        let (need, need_paved) = needs(def);
        let mut rows = 0;
        for rw in a.runways.iter().filter(|r| r.length_m >= 100.0 && !r.ends[0].ident.starts_with('H')) {
            rows += 1;
            let ok = rw.length_m >= need * 0.95 && (!need_paved || rw.surface.paved());
            let rr = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, 56.0));
            p.rect_filled(rr, 8, pal::RAISED);
            p.text(rr.min + Vec2::new(12.0, 8.0), Align2::LEFT_TOP, rw.name(), f_inst_b(14.0), pal::TEXT);
            let info = format!("{} × {} m · {}{}", thousands(rw.length_m), rw.width_m.round(), rw.surface.name(), if rw.lighted { " · lit" } else { "" });
            p.text(rr.min + Vec2::new(12.0, 31.0), Align2::LEFT_TOP, info, f_ui(12.0), if ok { pal::TEXT2 } else { pal::AMBER });
            // end buttons with head/cross components
            for end in 0..2 {
                let e = &rw.ends[end];
                let b = Rect::from_min_size(Pos2::new(rr.max.x - 168.0 + end as f32 * 82.0, rr.min.y + 8.0), Vec2::new(76.0, 40.0));
                let selected = matches!(&st.plan.spec, Spec::Runway(x) | Spec::Final(x, _) if *x == e.ident) && st.plan.airport == a.ident;
                let resp = ui.interact(b, ui.id().with(("end", rw.name(), end)), Sense::click());
                p.rect_filled(b, 6, if selected { pal::ACCENT_DEEP } else if resp.hovered() { pal::RAISED_HI } else { Color32::from_rgb(20, 25, 33) });
                if selected {
                    p.rect_stroke(b, 6, Stroke::new(1.5, pal::ACCENT), egui::StrokeKind::Inside);
                }
                p.text(b.min + Vec2::new(8.0, 5.0), Align2::LEFT_TOP, &e.ident, f_inst_b(13.0), pal::TEXT);
                if let (Some((from, kt, gust, _)), Some(h)) = (wind.as_ref(), rw.heading(end)) {
                    let (hw, xw) = airports::wind_components(h, *from, *kt);
                    let col = if hw < -3.0 { pal::RED } else if xw.abs() > 15.0 { pal::AMBER } else { pal::GREEN };
                    let gx = gust.map(|g| airports::wind_components(h, *from, g).1.abs()).unwrap_or(0.0);
                    let _ = gx;
                    p.text(b.min + Vec2::new(8.0, 22.0), Align2::LEFT_TOP, format!("{}{:.0} X{:.0}", if hw >= 0.0 { "H" } else { "T" }, hw.abs(), xw.abs()), f_mono(11.0), col);
                }
                if resp.clicked() {
                    st.plan.airport = a.ident.clone();
                    st.picker.air_point = None;
                    st.picker.auto_end = None;
                    st.plan.spec = match &st.plan.spec {
                        Spec::Final(_, nm) => Spec::Final(e.ident.clone(), *nm),
                        _ => Spec::Runway(e.ident.clone()),
                    };
                }
            }
            y = rr.max.y + 8.0;
        }
        if rows == 0 {
            p.text(Pos2::new(x, y), Align2::LEFT_TOP, "No runway data", f_ui(13.0), pal::TEXT3);
            y += 24.0;
        }
        // start position
        y += 10.0;
        p.text(Pos2::new(x, y), Align2::LEFT_TOP, "START", f_inst_b(11.0), pal::TEXT3);
        y += 22.0;
        let end_ident = match &st.plan.spec {
            Spec::Runway(e) | Spec::Final(e, _) => Some(e.clone()),
            _ => a.runways.first().map(|r| r.ends[0].ident.clone()),
        };
        let mut opts: Vec<(String, Spec)> = Vec::new();
        if let Some(e) = &end_ident {
            if def.has_engine {
                opts.push((format!("Runway {e}"), Spec::Runway(e.clone())));
            }
            for nm in [3.0, 5.0, 10.0] {
                opts.push((format!("{nm:.0} NM final"), Spec::Final(e.clone(), nm)));
            }
        }
        if def.has_engine {
            opts.push(("Apron".into(), Spec::Apron));
        } else {
            opts.push(("Thermal".into(), Spec::Thermal));
        }
        let cw = (w - 8.0 * 2.0) / 3.0;
        for (k, (label, spec)) in opts.iter().enumerate() {
            let (row, col) = (k / 3, k % 3);
            let b = Rect::from_min_size(Pos2::new(x + col as f32 * (cw + 8.0), y + row as f32 * 40.0), Vec2::new(cw, 32.0));
            let on = st.plan.spec == *spec && st.plan.airport == a.ident && st.picker.air_point.is_none();
            if chip_btn(ui, b, ("start", k), label, on) {
                st.plan.airport = a.ident.clone();
                st.plan.spec = spec.clone();
                st.picker.air_point = None;
            }
        }
        y += ((opts.len() + 2) / 3) as f32 * 40.0 + 4.0;
        // air start
        let air_on = matches!(st.plan.spec, Spec::Air { .. });
        let ar = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, if air_on { 92.0 } else { 40.0 }));
        p.rect_filled(ar, 8, if air_on { pal::ACCENT_BG } else { pal::RAISED });
        p.text(ar.min + Vec2::new(12.0, 11.0), Align2::LEFT_TOP, if air_on { "Airborne at the marker" } else { "Airborne: click the map" }, f_semi(13.0), if air_on { pal::TEXT } else { pal::TEXT2 });
        if air_on {
            let mut changed = false;
            ui.scope_builder(UiBuilder::new().max_rect(Rect::from_min_size(ar.min + Vec2::new(12.0, 38.0), Vec2::new(w - 24.0, 46.0))), |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("ALT").font(f_inst(11.0)).color(pal::TEXT3));
                    changed |= ui.add(egui::DragValue::new(&mut st.picker.air_alt_ft).range(500.0..=40_000.0).speed(50.0).suffix(" ft")).changed();
                    ui.add_space(12.0);
                    ui.label(RichText::new("HDG").font(f_inst(11.0)).color(pal::TEXT3));
                    changed |= ui.add(egui::DragValue::new(&mut st.picker.air_hdg).range(0.0..=359.0).speed(1.0).suffix("°")).changed();
                });
            });
            if changed {
                if let Spec::Air { alt_m, hdg, .. } = &mut st.plan.spec {
                    *alt_m = st.picker.air_alt_ft / 3.280_84;
                    *hdg = st.picker.air_hdg;
                }
            }
        }
        ar.max.y + 16.0 - o.y
    });
    // suitability warning + next
    if warn {
        p.text(Pos2::new(x, fy - 30.0), Align2::LEFT_TOP, format!("Runways here are short for the {}", def.name), f_ui(12.5), pal::AMBER);
    }
    if button(ui, Rect::from_min_size(Pos2::new(x, fy), Vec2::new(w, 44.0)), "next", "Conditions  ›", Btn::Primary, true).clicked() {
        st.plan.airport = a.ident.clone();
        st.actions.push(Action::Goto(Screen::Conditions));
    }
}

// ---------------------------------------------------------------- conditions

pub fn conditions(ui: &mut Ui, st: &mut UiState) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, 1.0);
    let margin = (screen.width() * 0.05).max(40.0);
    let content = Rect::from_min_max(Pos2::new(screen.min.x + margin, screen.min.y + 36.0), Pos2::new(screen.max.x - margin, screen.max.y - 36.0));
    if let Some(s) = steps(ui, content.min, 2) {
        st.actions.push(Action::Goto([Screen::Aircraft, Screen::Departure, Screen::Conditions][s]));
    }
    let y = heading(&p, content.min + Vec2::new(0.0, 56.0), "Conditions", "Real weather and time at the airport, or set your own.");
    let colw = (content.width() - 24.0) * 0.5;
    let c = &mut st.settings.conditions;
    let card = |ui: &mut Ui, r: Rect, id: &str, title: &str, sub: &str, on: bool| -> bool {
        let resp = ui.interact(r, ui.id().with(id), Sense::click());
        let p = ui.painter();
        panel_with(p, r, sp::R_CARD, if on { Color32::from_rgb(16, 30, 46) } else if resp.hovered() { pal::RAISED } else { pal::PANEL_SOLID });
        if on {
            p.rect_stroke(r, sp::R_CARD, Stroke::new(2.0, pal::ACCENT), egui::StrokeKind::Inside);
        }
        p.circle_stroke(r.min + Vec2::new(30.0, 34.0), 9.0, Stroke::new(2.0, if on { pal::ACCENT } else { pal::TEXT3 }));
        if on {
            p.circle_filled(r.min + Vec2::new(30.0, 34.0), 5.0, pal::ACCENT);
        }
        p.text(r.min + Vec2::new(52.0, 22.0), Align2::LEFT_TOP, title, f_semi(17.0), pal::TEXT);
        p.text(r.min + Vec2::new(52.0, 46.0), Align2::LEFT_TOP, sub, f_ui(13.0), pal::TEXT2);
        resp.clicked()
    };
    let live_r = Rect::from_min_size(Pos2::new(content.min.x, y + 10.0), Vec2::new(colw, 84.0));
    let cust_r = Rect::from_min_size(Pos2::new(content.min.x + colw + 24.0, y + 10.0), Vec2::new(colw, 84.0));
    if card(ui, live_r, "live", "Live weather & time", "Latest METAR and forecast, the real local time", c.live) {
        c.live = true;
    }
    if card(ui, cust_r, "custom", "Custom", "Time of day, season and weather of your choice", !c.live) {
        c.live = false;
    }
    let body_h = if c.live { 240.0 } else { 420.0 };
    let body = Rect::from_min_max(Pos2::new(content.min.x, live_r.max.y + 24.0), Pos2::new(content.max.x, (live_r.max.y + 24.0 + body_h).min(content.max.y - 76.0).max(live_r.max.y + 24.0 + 120.0)));
    panel(&p, body, sp::R_CARD);
    super::scroll::page(ui, body.shrink(1.0), "conditions_body", Color32::from_rgba_premultiplied(12, 15, 20, 220), |ui, o, bw| {
        let p = ui.painter().clone();
        let x = o.x + 27.0;
        let mut yy = o.y + 23.0;
        let airport = st.db.as_ref().and_then(|d| d.get(&st.plan.airport).cloned());
        if c.live {
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "AT DEPARTURE", f_inst_b(11.0), pal::TEXT3);
            yy += 24.0;
            if let Some(a) = &airport {
                if st.picker.metar.as_ref().is_none_or(|m| m.ident != a.ident) {
                    st.picker.metar = Some(airports::MetarFetch::start(&a.ident, &a.icao, a.lat, a.lon));
                }
                if let Some(m) = st.picker.metar.as_mut() {
                    m.poll();
                }
                let rt = crate::weather::real_time(a.lat, a.lon, crate::weather::unix_now(), None);
                p.text(Pos2::new(x, yy), Align2::LEFT_TOP, format!("{} · {}", a.code(), a.name), f_semi(18.0), pal::TEXT);
                yy += 34.0;
                let el = crate::atmosphere::sun_direction(a.lat, a.lon, rt.day_of_year, rt.local_hours).y.asin().to_degrees();
                let light = if el > 0.0 { "daylight" } else if el > -6.0 { "twilight" } else { "night" };
                let mut tiles: Vec<(&str, String, Color32)> = vec![("LOCAL TIME", format!("{}  {light}", hhmm(rt.local_hours)), pal::TEXT)];
                match st.picker.metar.as_ref().and_then(|m| m.result.clone()) {
                    Some(Ok((m, _))) => {
                        if let Some(w) = crate::weather::Wx::from_metar_str(&m.raw) {
                            let catc = match w.category() {
                                "VFR" => pal::GREEN,
                                "MVFR" => pal::ACCENT,
                                "IFR" => pal::RED,
                                _ => pal::MAGENTA,
                            };
                            tiles.push(("CATEGORY", w.category().to_string(), catc));
                            tiles.push(("WIND", if w.wind_kt < 1.0 { "Calm".into() } else { format!("{:03.0}° {:.0} kt", w.wind_from, w.wind_kt) }, pal::TEXT));
                            tiles.push(("VISIBILITY", if w.vis_m >= 9999.0 { "10 km +".into() } else { format!("{:.1} km", w.vis_m / 1000.0) }, pal::TEXT));
                            tiles.push(("CLOUD", w.cloud_text(), pal::TEXT));
                            tiles.push(("TEMPERATURE", format!("{:.0} °C", w.temp_c), pal::TEXT));
                            tiles.push(("QNH", format!("{:.0} hPa", w.qnh_hpa), pal::TEXT));
                        }
                        let tw = (bw - 56.0 - 6.0 * 12.0) / 7.0;
                        for (k, (lab, val, col)) in tiles.iter().enumerate() {
                            let r = Rect::from_min_size(Pos2::new(x + k as f32 * (tw + 12.0), yy), Vec2::new(tw, 64.0));
                            p.rect_filled(r, 8, pal::RAISED);
                            p.text(r.min + Vec2::new(12.0, 10.0), Align2::LEFT_TOP, *lab, f_inst(10.5), pal::TEXT3);
                            p.text(r.min + Vec2::new(12.0, 30.0), Align2::LEFT_TOP, val, f_semi(16.0), *col);
                        }
                        yy += 82.0;
                        p.text(Pos2::new(x, yy), Align2::LEFT_TOP, m.raw.clone(), f_mono(13.0), pal::TEXT3);
                    }
                    Some(Err(_)) => {
                        p.text(Pos2::new(x, yy), Align2::LEFT_TOP, format!("Local time {} · no METAR nearby — forecast data is used", hhmm(rt.local_hours)), f_ui(14.0), pal::TEXT2);
                    }
                    None => {
                        p.text(Pos2::new(x, yy), Align2::LEFT_TOP, format!("Local time {} · fetching weather…", hhmm(rt.local_hours)), f_ui(14.0), pal::TEXT2);
                    }
                }
            }
        } else {
            // time of day with a sun gradient
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "TIME OF DAY", f_inst_b(11.0), pal::TEXT3);
            p.text(Pos2::new(x + 140.0, yy - 5.0), Align2::LEFT_TOP, hhmm(c.hours), f_semi(19.0), pal::TEXT);
            yy += 28.0;
            let bar = Rect::from_min_size(Pos2::new(x, yy), Vec2::new(bw - 56.0, 28.0));
            let (lat, lon) = airport.as_ref().map(|a| (a.lat, a.lon)).unwrap_or((51.5, 0.0));
            let n = 48;
            let mut m = epaint::Mesh::default();
            for i in 0..=n {
                let h = i as f32 / n as f32 * 24.0;
                let el = crate::atmosphere::sun_direction(lat, lon, c.day, h).y;
                let col = if el > 0.15 {
                    Color32::from_rgb(110, 170, 230)
                } else if el > 0.0 {
                    super::lerp_col(Color32::from_rgb(240, 150, 80), Color32::from_rgb(110, 170, 230), el / 0.15)
                } else if el > -0.1 {
                    super::lerp_col(Color32::from_rgb(30, 36, 70), Color32::from_rgb(240, 150, 80), (el + 0.1) / 0.1)
                } else {
                    Color32::from_rgb(16, 20, 38)
                };
                let xx = bar.min.x + bar.width() * i as f32 / n as f32;
                m.colored_vertex(Pos2::new(xx, bar.min.y), col);
                m.colored_vertex(Pos2::new(xx, bar.max.y), col);
                if i > 0 {
                    let k = i as u32 * 2;
                    m.add_triangle(k - 2, k - 1, k);
                    m.add_triangle(k - 1, k + 1, k);
                }
            }
            p.add(Shape::mesh(m));
            let resp = ui.interact(bar.expand(6.0), ui.id().with("timebar"), Sense::click_and_drag());
            if let Some(pos) = resp.interact_pointer_pos().filter(|_| resp.dragged() || resp.clicked()) {
                c.hours = ((pos.x - bar.min.x) / bar.width() * 24.0).clamp(0.0, 23.99);
                c.hours = (c.hours * 4.0).round() / 4.0;
            }
            let kx = bar.min.x + bar.width() * c.hours / 24.0;
            p.circle_filled(Pos2::new(kx, bar.center().y), 11.0, Color32::WHITE);
            p.circle_stroke(Pos2::new(kx, bar.center().y), 11.0, Stroke::new(2.0, pal::ACCENT_DEEP));
            for h in [0, 6, 12, 18] {
                p.text(Pos2::new(bar.min.x + bar.width() * h as f32 / 24.0, bar.max.y + 6.0), Align2::LEFT_TOP, format!("{h:02}:00"), f_ui(11.0), pal::TEXT3);
            }
            yy = bar.max.y + 36.0;
            // season
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "SEASON", f_inst_b(11.0), pal::TEXT3);
            yy += 24.0;
            let seasons = [("Spring", 105u32), ("Summer", 196), ("Autumn", 288), ("Winter", 15)];
            for (k, (name, doy)) in seasons.iter().enumerate() {
                let b = Rect::from_min_size(Pos2::new(x + k as f32 * 118.0, yy), Vec2::new(110.0, 34.0));
                let on = (c.day as i32 - *doy as i32).rem_euclid(365) < 46 || (*doy as i32 - c.day as i32).rem_euclid(365) < 46;
                if chip_btn(ui, b, ("season", k), name, on) {
                    c.day = *doy;
                }
            }
            let (m, d) = month_day(c.day);
            p.text(Pos2::new(x + 4.0 * 118.0 + 12.0, yy + 8.0), Align2::LEFT_TOP, format!("{d} {}", MONTHS[m as usize - 1]), f_ui(14.0), pal::TEXT2);
            yy += 56.0;
            // weather presets
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "WEATHER", f_inst_b(11.0), pal::TEXT3);
            yy += 24.0;
            for (k, w) in crate::clouds::Weather::ALL.iter().enumerate() {
                let b = Rect::from_min_size(Pos2::new(x + k as f32 * 118.0, yy), Vec2::new(110.0, 34.0));
                let on = c.metar.trim().is_empty() && c.preset == w.name();
                if chip_btn(ui, b, ("wx", k), w.name(), on) {
                    c.preset = w.name().to_string();
                    c.metar.clear();
                }
            }
            yy += 54.0;
            p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "OR A METAR", f_inst_b(11.0), pal::TEXT3);
            yy += 24.0;
            let mr = Rect::from_min_size(Pos2::new(x, yy), Vec2::new(bw - 56.0, 40.0));
            p.rect_filled(mr, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
            p.rect_stroke(mr, sp::R_CTRL, Stroke::new(1.0, pal::HAIRLINE_HI), egui::StrokeKind::Inside);
            ui.put(mr.shrink2(Vec2::new(12.0, 8.0)), egui::TextEdit::singleline(&mut c.metar).hint_text("EGLL 121150Z 24015G25KT 6000 -RA BKN012 OVC030 11/09 Q1004").frame(egui::Frame::NONE).font(f_mono(14.0)).text_color(pal::TEXT));
            yy = mr.max.y + 10.0;
            if !c.metar.trim().is_empty() {
                match crate::weather::Wx::from_metar_str(c.metar.trim()) {
                    Some(w) => {
                        p.text(Pos2::new(x, yy), Align2::LEFT_TOP, format!("{} · wind {:03.0}° {:.0} kt · visibility {:.1} km · cloud {}", w.category(), w.wind_from, w.wind_kt, w.vis_m / 1000.0, w.cloud_text()), f_ui(13.0), pal::GREEN);
                    }
                    None => {
                        p.text(Pos2::new(x, yy), Align2::LEFT_TOP, "Not a valid METAR", f_ui(13.0), pal::AMBER);
                    }
                }
            }
        }
        yy + 44.0 - o.y
    });
    // footer
    let fy = content.max.y - 52.0;
    if button(ui, Rect::from_min_size(Pos2::new(content.min.x, fy), Vec2::new(140.0, 48.0)), "back", "Back", Btn::Secondary, true).clicked() || esc(ui) {
        st.actions.push(Action::Goto(Screen::Departure));
    }
    // summary + fly
    let def = &st.defs[st.plan.aircraft.min(st.defs.len() - 1)];
    let summary = format!("{} · {} · {}", def.name, st.plan.airport, st.plan.spec.label());
    p.text(Pos2::new(content.max.x - 250.0, fy + 24.0), Align2::RIGHT_CENTER, summary, f_ui(14.0), pal::TEXT2);
    if button(ui, Rect::from_min_size(Pos2::new(content.max.x - 230.0, fy), Vec2::new(230.0, 48.0)), "fly", "Fly", Btn::Primary, !st.plan.airport.is_empty()).clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter) && !i.modifiers.any()) {
        st.plan.dogfight = None;
        st.actions.push(Action::Fly);
    }
}

const MONTHS: [&str; 12] = ["January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"];

fn month_day(doy: u32) -> (u32, u32) {
    let lens = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let mut d = doy.clamp(1, 365);
    for (i, l) in lens.iter().enumerate() {
        if d <= *l {
            return (i as u32 + 1, d);
        }
        d -= l;
    }
    (12, 31)
}

// ---------------------------------------------------------------- settings

pub fn settings(ui: &mut Ui, st: &mut UiState, in_flight: bool) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    backdrop(&p, screen, if in_flight { 0.85 } else { 1.0 });
    // Controls needs room for its binding table.
    let w = if st.settings_tab == 3 { 960.0 } else { 760.0f32 }.min(screen.width() - 80.0);
    let card = Rect::from_center_size(screen.center(), Vec2::new(w, (screen.height() - 120.0).min(if st.settings_tab == 3 { 820.0 } else { 760.0 })));
    panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
    let x = card.min.x + 32.0;
    p.text(Pos2::new(x, card.min.y + 26.0), Align2::LEFT_TOP, "Settings", f_display(28.0), pal::TEXT);
    let tabs = ["Display", "Units", "Sound", "Controls"];
    if let Some(t) = segmented(ui, Rect::from_min_size(Pos2::new(x, card.min.y + 76.0), Vec2::new(w - 64.0, 36.0)), "settabs", &tabs, st.settings_tab) {
        st.settings_tab = t;
    }
    let body = Rect::from_min_max(Pos2::new(x, card.min.y + 132.0), Pos2::new(card.max.x - 32.0, card.max.y - 80.0));
    let s0 = st.settings.clone();
    let lw = 220.0;
    let row_h = 52.0;
    let label = |p: &egui::Painter, y: f32, k: &str, hint: &str| {
        p.text(Pos2::new(x, y + 8.0), Align2::LEFT_TOP, k, f_semi(14.0), pal::TEXT);
        if !hint.is_empty() {
            p.text(Pos2::new(x, y + 28.0), Align2::LEFT_TOP, hint, f_ui(11.5), pal::TEXT3);
        }
    };
    let ctrl = |y: f32| Rect::from_min_size(Pos2::new(x + lw, y + 6.0), Vec2::new(body.max.x - x - lw, 34.0));
    if st.settings_tab == 3 {
        // Bindings, devices, mouse (ui/controls.rs).
        let foot = Rect::from_min_size(Pos2::new(x, card.max.y - 64.0), Vec2::new(w - 240.0, 44.0));
        super::controls::page(ui, st, body, foot);
    } else {
        // The other tabs scroll when the window is short (the bar floats right of the controls).
        let page_r = Rect::from_min_max(body.min, Pos2::new(card.max.x - 12.0, body.max.y));
        let tab = st.settings_tab;
        let audio_status = st.audio_status.clone();
        let s = &mut st.settings;
        super::scroll::page(ui, page_r, ["settings_display", "settings_units", "settings_sound"][tab.min(2)], pal::PANEL_SOLID, |ui, o, _| {
            let p = ui.painter().clone();
            let mut y = o.y;
            match tab {
                0 => {
                    label(&p, y, "Interface scale", "Relative to the window size");
                    let opts = ["80 %", "90 %", "100 %", "115 %", "130 %"];
                    let vals = [0.8, 0.9, 1.0, 1.15, 1.3];
                    let cur = vals.iter().position(|v| (v - s.ui_scale).abs() < 0.03).unwrap_or(2);
                    if let Some(i) = segmented(ui, ctrl(y), "scale", &opts, cur) {
                        s.ui_scale = vals[i];
                    }
                    y += row_h;
                    label(&p, y, "Instruments", "U cycles in flight");
                    let cur = [HudMode::Full, HudMode::Minimal, HudMode::Off].iter().position(|m| *m == s.hud).unwrap_or(0);
                    if let Some(i) = segmented(ui, ctrl(y), "hud", &["Full", "Minimal", "Off"], cur) {
                        s.hud = [HudMode::Full, HudMode::Minimal, HudMode::Off][i];
                    }
                    y += row_h;
                    label(&p, y, "Scenery detail", "Detailed scenery radius");
                    if let Some(i) = segmented(ui, ctrl(y), "detail", &DETAIL_NAMES, s.detail as usize) {
                        s.detail = i as u8;
                    }
                    y += row_h;
                    label(&p, y, "Road traffic", "Moving cars, buses and lorries");
                    if let Some(i) = segmented(ui, ctrl(y), "traffic", &crate::traffic::Level::NAMES, s.traffic as usize) {
                        s.traffic = i as u8;
                    }
                    y += row_h;
                    label(&p, y, "Vertical sync", "Off: lowest latency, may stutter");
                    if let Some(i) = segmented(ui, ctrl(y), "vsync", &["On", "Off"], if s.vsync { 0 } else { 1 }) {
                        s.vsync = i == 0;
                    }
                    y += row_h;
                    label(&p, y, "Map style", "Auto follows the sun");
                    let cur = [MapStyle::Auto, MapStyle::Day, MapStyle::Night].iter().position(|m| *m == s.map_style).unwrap_or(0);
                    if let Some(i) = segmented(ui, ctrl(y), "mapstyle", &["Auto", "Day", "Night"], cur) {
                        s.map_style = [MapStyle::Auto, MapStyle::Day, MapStyle::Night][i];
                    }
                    y += row_h;
                    label(&p, y, "Map orientation", "");
                    if let Some(i) = segmented(ui, ctrl(y), "north", &["North up", "Track up"], if s.north_up { 0 } else { 1 }) {
                        s.north_up = i == 0;
                    }
                }
                1 => {
                    label(&p, y, "Speed", "Panels and the status card; dials keep the aircraft's own units");
                    let cur = [SpeedUnit::Knots, SpeedUnit::Kmh, SpeedUnit::Mph].iter().position(|m| *m == s.speed).unwrap_or(0);
                    if let Some(i) = segmented(ui, ctrl(y), "spd", &["Knots", "km/h", "mph"], cur) {
                        s.speed = [SpeedUnit::Knots, SpeedUnit::Kmh, SpeedUnit::Mph][i];
                    }
                    y += row_h + 10.0;
                    label(&p, y, "Altitude", "");
                    if let Some(i) = segmented(ui, ctrl(y), "alt", &["Feet", "Metres"], if s.alt == AltUnit::Feet { 0 } else { 1 }) {
                        s.alt = if i == 0 { AltUnit::Feet } else { AltUnit::Metres };
                    }
                }
                _ => {
                    label(&p, y, "Mute", "N in flight");
                    if let Some(i) = segmented(ui, ctrl(y), "mute", &["Sound on", "Muted"], if s.sound.muted { 1 } else { 0 }) {
                        s.sound.muted = i == 1;
                    }
                    y += row_h;
                    let slider = |ui: &mut Ui, y: f32, k: &str, v: &mut f32, max: f32| {
                        label(&p, y, k, "");
                        let r = ctrl(y);
                        // (Slider and its value on one line.)
                        ui.scope_builder(UiBuilder::new().max_rect(r).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
                            ui.spacing_mut().slider_width = r.width() - 70.0;
                            ui.add(egui::Slider::new(v, 0.0..=max).show_value(false));
                            ui.label(RichText::new(format!("{:.0} %", *v / max * 100.0)).font(f_mono(12.0)).color(pal::TEXT2));
                        });
                    };
                    slider(ui, y, "Master", &mut s.sound.master, 1.0);
                    y += row_h;
                    slider(ui, y, "Engines", &mut s.sound.engines, 1.5);
                    y += row_h;
                    slider(ui, y, "Wind, ground, weather", &mut s.sound.environment, 1.5);
                    y += row_h;
                    slider(ui, y, "Cockpit systems", &mut s.sound.systems, 1.5);
                    y += row_h;
                    slider(ui, y, "Callouts", &mut s.sound.voice, 1.5);
                    y += row_h;
                    slider(ui, y, "ATC radio", &mut s.sound.radio, 1.5);
                    y += row_h;
                    slider(ui, y, "Interface", &mut s.sound.ui, 1.5);
                    y += row_h;
                    p.text(Pos2::new(x, y + 6.0), Align2::LEFT_TOP, audio_status.clone(), f_ui(12.0), pal::TEXT3);
                }
            }
            y + row_h + 8.0 - o.y
        });
    }
    if st.settings != s0 {
        st.settings_dirty = true;
    }
    let fy = card.max.y - 64.0;
    if button(ui, Rect::from_min_size(Pos2::new(card.max.x - 172.0, fy), Vec2::new(140.0, 44.0)), "done", "Done", Btn::Primary, true).clicked() || (esc(ui) && !super::controls::swallow_esc(st)) {
        st.input.listen = None;
        st.input.calib = None;
        st.actions.push(Action::Back);
    }
}

// ---------------------------------------------------------------- pause

pub fn pause(ui: &mut Ui, st: &mut UiState, title: &str, sub: &str) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(4, 6, 9, 150));
    let w = 380.0;
    let items: [(&str, Action, Btn); 8] = [
        ("Resume", Action::Resume, Btn::Primary),
        ("Restart", Action::Restart, Btn::Secondary),
        ("Systems & failures", Action::Failures, Btn::Secondary),
        ("Change aircraft", Action::Goto(Screen::Aircraft), Btn::Secondary),
        ("Change airport", Action::Goto(Screen::Departure), Btn::Secondary),
        ("Settings", Action::Goto(Screen::Settings), Btn::Secondary),
        ("Main menu", Action::Goto(Screen::Main), Btn::Ghost),
        ("Quit", Action::Quit, Btn::Ghost),
    ];
    let mut items: Vec<_> = items.into_iter().filter(|it| !(crate::platform::WEB && matches!(it.1, Action::Quit))).collect();
    // Multiplayer: the server sets the airport; leaving keeps the flight going alone.
    if st.mp.connected() {
        items.retain(|it| !matches!(it.1, Action::Goto(Screen::Departure)));
        items.insert(items.len().saturating_sub(2), ("Leave server", Action::LeaveServer, Btn::Secondary));
    }
    // Rows tighten on short windows so the card always fits.
    let step = ((screen.height() - 48.0 - 130.0) / items.len() as f32).clamp(44.0, 54.0);
    let h = 120.0 + items.len() as f32 * step + 10.0;
    let card = Rect::from_center_size(screen.center(), Vec2::new(w, h));
    panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
    p.text(card.min + Vec2::new(28.0, 24.0), Align2::LEFT_TOP, "Paused", f_display(28.0), pal::TEXT);
    p.text(card.min + Vec2::new(28.0, 64.0), Align2::LEFT_TOP, title, f_semi(14.0), pal::TEXT2);
    p.text(card.min + Vec2::new(28.0, 84.0), Align2::LEFT_TOP, sub, f_ui(12.5), pal::TEXT3);
    let mut y = card.min.y + 118.0;
    for (i, (label, act, kind)) in items.into_iter().enumerate() {
        let r = Rect::from_min_size(Pos2::new(card.min.x + 24.0, y), Vec2::new(w - 48.0, step - 8.0));
        if button(ui, r, ("pause", i), label, kind, true).clicked() {
            st.actions.push(act);
        }
        y += step;
    }
}

// ---------------------------------------------------------------- loading

pub fn loading(ui: &mut Ui, st: &mut UiState, title: &str, detail: &str, frac: Option<f32>) {
    let screen = ui.ctx().content_rect();
    let p = ui.painter().clone();
    p.rect_filled(screen, 0, pal::BG);
    // the destination on the overview map, dimmed
    let ppp = ui.ctx().pixels_per_point();
    let a = st.db.as_ref().and_then(|d| d.get(&st.plan.airport).cloned());
    if let Some(a) = &a {
        let (cx, cy) = overview::merc(a.lat, a.lon);
        let v = OView { cx, cy, zoom: 8.6, rect: screen };
        st.picker.overview.draw(&p, &mut st.canvas, "overview", ppp, &v, true);
        p.rect_filled(screen, 0, Color32::from_rgba_premultiplied(6, 8, 11, 150));
        let q = v.ll_to_screen(a.lat, a.lon);
        let pulse = (st.time * 2.0).sin() * 0.5 + 0.5;
        p.circle_stroke(q, 10.0 + pulse * 10.0, Stroke::new(2.0, alpha(pal::ACCENT, 1.0 - pulse * 0.7)));
        p.circle_filled(q, 5.0, pal::ACCENT);
    }
    let w = 520.0;
    let card = Rect::from_center_size(Pos2::new(screen.center().x, screen.max.y - 190.0), Vec2::new(w, 150.0));
    panel_with(&p, card, sp::R_CARD, pal::PANEL_SOLID);
    p.text(card.min + Vec2::new(28.0, 22.0), Align2::LEFT_TOP, title, f_display(22.0), pal::TEXT);
    p.text(card.min + Vec2::new(28.0, 56.0), Align2::LEFT_TOP, detail, f_ui(13.5), pal::TEXT2);
    let bar = Rect::from_min_size(card.min + Vec2::new(28.0, 96.0), Vec2::new(w - 56.0, 6.0));
    p.rect_filled(bar, 3, Color32::from_white_alpha(18));
    match frac {
        Some(f) => {
            let f = f.clamp(0.0, 1.0);
            p.rect_filled(Rect::from_min_size(bar.min, Vec2::new(bar.width() * f, bar.height())), 3, pal::ACCENT);
        }
        None => {
            let t = (st.time * 0.8).fract();
            let x0 = bar.min.x + bar.width() * (t * 1.4 - 0.4).clamp(0.0, 1.0);
            let x1 = bar.min.x + bar.width() * (t * 1.4).clamp(0.0, 1.0);
            p.rect_filled(Rect::from_min_max(Pos2::new(x0, bar.min.y), Pos2::new(x1, bar.max.y)), 3, pal::ACCENT);
        }
    }
    let pct = frac.map(|f| format!("{:.0} %", f.clamp(0.0, 1.0) * 100.0)).unwrap_or_default();
    p.text(card.min + Vec2::new(28.0, 116.0), Align2::LEFT_TOP, crate::hud::ATTRIBUTION, f_ui(11.0), pal::TEXT3);
    p.text(Pos2::new(card.max.x - 28.0, card.min.y + 116.0), Align2::RIGHT_TOP, pct, f_ui(11.0), pal::TEXT2);
    let _ = (CornerRadius::ZERO, Id::NULL);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_type_has_a_family() {
        let defs = crate::flight::catalogue();
        let of = |id: &str| Group::of(defs.iter().find(|d| d.id == id).expect(id));
        assert_eq!(of("kestrel"), Group::Light);
        assert_eq!(of("swift"), Group::Light);
        assert_eq!(of("meridian320"), Group::Airliner);
        assert_eq!(of("hercules"), Group::Military);
        assert_eq!(of("galaxy"), Group::Military);
        assert_eq!(of("vulcan"), Group::Military);
        for id in ["triple7", "crj700", "citationx", "concorde"] {
            assert_eq!(of(id), Group::Airliner, "{id}");
        }
        assert_eq!(of("ec130"), Group::Rotor);
        assert_eq!(of("fpv"), Group::Rotor);
        for id in ["extra500", "cub", "zlin", "otter"] {
            assert_eq!(of(id), Group::Light, "{id}");
        }
        assert_eq!(of("dc3"), Group::Airliner);
        for g in Group::ALL {
            assert!(!g.name().is_empty());
        }
    }

    #[test]
    fn card_grid_fits_any_width() {
        for w in [120.0, 300.0, 520.0, 800.0, 1130.0, 1600.0, 2400.0] {
            let (cols, cw, ch) = card_grid(w);
            assert!(cols >= 1 && ch > cw * 0.6);
            assert!(cols as f32 * cw + (cols as f32 - 1.0) * CARD_GAP <= w + 0.01, "{w}");
            assert!(cols == 1 || cw >= 228.0, "{w}: {cw}");
        }
    }
}
