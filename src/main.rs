//! Skyward — native Rust flight simulator over real OpenStreetMap scenery,
//! streamed endlessly from vector tiles around the aircraft.
//!
//! Windowed:   skyward [--airport ICAO | --lat L --lon L] [--aircraft ID] [--start runway|approach|city|thermal] [--start-at SPEC] [--no-menu]
//! Headless:   skyward --shot out.png [--width W --height H] [--cam chase|cockpit|tower|flyby]
//!                     [--seconds S --throttle T] [--heading DEG] [--eye x,y,z --look x,y,z] [--stats]
//!                     [--menu-screen main|aircraft|airport|conditions|settings|pause|loading|credits] [--hud full|minimal|off] [--map corner|large|off]
//!                     [--realtime [--warp N]]   (stream + render in real time, straight & level, report timings)
//!                     [--gear up|down] [--flaps N] [--spoilers F] [--reverse] [--fail BITS] [--ap] [--fps HZ]   (setup / test knobs)
//! Settings:   --menu-screen settings [--settings-category video|display|camera|flight|assists|weather|traffic|sound|controls|multiplayer|accessibility|interface|data|about] [--settings-search TEXT] [--settings-from pause] [--palette deuteranopia|protanopia|tritanopia] [--high-contrast]; SKYWARD_CONFIG_DIR=dir for the settings file
//! Systems:    --fail pitot,vacuum,fire2@30,... (names, optional @seconds to arm) [--cold-dark] [--panel failures|armed|systems]
//! Missions:   skyward --mission river|v1|decompression|... [--mission-bot]   (airline emergencies; --menu-screen missions|briefing|debrief)
//! Auto flight: skyward [--bot | --watch] ...        (hand the aircraft to the AI pilot; --watch adds the cinematic camera)
//! Dogfight:  skyward --aircraft spitfire|f16 --dogfight [1v1|2v2] [--opponent ID] [--difficulty rookie|average|veteran|ace] [--dogfight-ai] [--shot-when guns|hit|lock|missile|flares|smoke|eject|chute|kill] [--view-bandit] [--eject-at S]
//! Soak test:  skyward --shot out.png --bot --soak --seconds 1800 [--warp 8] [--shot-every 300] [--weather scattered]
//! Weather:    live METAR + real local time by default; --no-live, --metar "EGLC 121450Z 22012KT 3000 RA BKN008 09/08 Q1002", --time HH.MM, --date MM-DD, --weather clear|few|scattered|broken|overcast
//! Controllers: --controllers [SECONDS] lists devices and prints their inputs (SKYWARD_FAKE_INPUT=xbox,x52,pedals,tca adds virtual ones)
//! Sound:      --mute (N toggles in flight; volumes in Settings › Sound); offline: --audio-render out.wav --seconds S [--audio-script "2:cam=cockpit,9:gear"] [--audio-until Final]
//! Display:    --vsync on|off (default on: FIFO present, smoothed frame step); SKYWARD_FRAME_LOG=file logs per-frame timing
//! Traffic:    --traffic off|low|medium|high (road traffic density; SKYWARD_NO_TRAFFIC=1 forces it off)
//! Multiplayer: skyward --connect host[:port] --name CALLSIGN [--password PW]   (server: cargo run -p skyward-server -- --help)
//! Live traffic: real aircraft from adsb.lol (default on; off with --no-live / --no-adsb; headless only with --adsb); --adsb-fixture rec.json replays a recording

mod adsb;
mod airport;
mod airports;
mod atmosphere;
mod aircraft_asset;
mod assets;
mod atc;
mod audio;
mod autopilot;
mod nav;
mod bot;
mod clouds;
mod cockpit;
mod damage;
mod debris;
mod flight;
mod gfx;
mod gfx_aircraft;
mod hud;
mod jsbsim;
mod lights;
mod missions;
mod models;
mod mvt;
mod net;
mod osm;
mod rotor;
mod scenario;
mod scene;
mod input;
mod sim;
mod startpos;
mod systems;
mod terrain;
mod landmarks;
mod tiles;
mod traffic;
mod vfx;
mod water;
mod weapons;
mod ui;
mod weather;
mod world;
mod wx;
mod platform;

use gfx::{FrameView, Gpu, Renderer};
use glam::{vec2, vec3, Vec2, Vec3};
use sim::{CamMode, Input, Key, Sim, Start};
use std::sync::Arc;
use web_time::{Duration, Instant};
use ui::{Action, Screen, UiState};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};
use world::World;

/// Tile workers (fetch + decode + mesh build).
const WORKERS: usize = 4;
/// Radius around the spawn that must be loaded before flying (m).
const SPAWN_RADIUS: f32 = 3200.0;
/// Per-frame GPU upload budget while flying (bytes) — keeps frames hitch-free.
/// (Smaller in the browser: each buffer write is copied to the GPU process.)
const UPLOAD_BUDGET: u64 = if cfg!(target_arch = "wasm32") { 1 << 20 } else { 4 << 20 };

#[derive(Clone)]
struct Args {
    lat: f64,
    lon: f64,
    airport: Option<String>,
    /// OurAirports ident of `--airport` once resolved.
    airport_ident: Option<String>,
    aircraft: usize,
    aircraft_given: bool,
    start: Start,
    /// A departure-screen start ("runway:27L", "final:27L:5", "apron", "air:lat:lon:alt_m:hdg").
    start_at: Option<startpos::Spec>,
    shot: Option<String>,
    width: u32,
    height: u32,
    cam: CamMode,
    seconds: f32,
    throttle: Option<f32>,
    heading: Option<f32>,
    eye: Option<Vec3>,
    look: Option<Vec3>,
    /// Fixed camera by geography: (lat, lon, metres above the ground).
    eye_ll: Option<(f64, f64, f32)>,
    look_ll: Option<(f64, f64, f32)>,
    menu: bool,
    menu_screen: Option<Screen>,
    no_menu: bool,
    stats: bool,
    realtime: bool,
    warp: u32,
    radius: Option<f32>,
    /// Road traffic density (Settings, `--traffic off|low|medium|high`).
    traffic: Option<traffic::Level>,
    map_zoom: Option<usize>,
    map_mode: Option<ui::flight::MapMode>,
    hud_mode: Option<ui::settings::HudMode>,
    then_reset: bool,
    bot: bool,
    watch: bool,
    soak: bool,
    shot_every: f32,
    gear: Option<bool>,
    flaps: Option<usize>,
    spoilers: Option<f32>,
    reverse: bool,
    fail: u8,
    /// Systems failures by name (`--fail pitot,vacuum,fire2@30`), cold & dark start, panel to show.
    fail_names: Vec<String>,
    cold_dark: bool,
    panel: Option<usize>,
    /// Radios & flight-plan panel tab to open (headless captures).
    radio_panel: Option<usize>,
    /// Headless shots: open the ATC comms menu.
    comms_panel: bool,
    /// Headless settings captures: category, search text, opened from the pause menu, palette.
    settings_cat: Option<ui::settings_schema::Cat>,
    settings_search: Option<String>,
    settings_from_pause: bool,
    palette: Option<ui::settings::Palette>,
    high_contrast: bool,
    /// Keys to press after the spawn (`--press ap,ap_appr`; input/ ids).
    press: Vec<Key>,
    /// Flight plan (`--route BIG,LAM,EGLL/27R`, nav/).
    route: Option<String>,
    frame_dt: Option<f32>,
    ap: bool,
    no_hud: bool,
    hours: Option<f32>,
    day_of_year: Option<u32>,
    weather: Option<clouds::Weather>,
    /// Real weather & time (weather.rs); --no-live turns it off, --metar fixes the weather.
    live: bool,
    live_given: bool,
    metar: Option<String>,
    /// Test: render, move the floating origin 20 km away, render again, compare.
    rebase_check: bool,
    /// Sound (audio/): start muted; offline render to a WAV (+ PNG, report) and its cue script.
    mute: bool,
    audio_render: Option<String>,
    audio_script: String,
    /// Fast-forward (silently, 8x) until the bot reaches this phase, then record.
    audio_until: Option<String>,
    /// Present with vsync (None = the saved setting).
    vsync: Option<bool>,
    /// Location given on the command line (skip the saved last flight).
    location_given: bool,
    /// Multiplayer (net/): join this server ("host[:port]") as `mp_name`.
    mp_connect: Option<String>,
    mp_name: Option<String>,
    mp_password: String,
    /// Headless tests: hold at the start until this many players are in.
    mp_wait: usize,
    /// Headless tests: chat cues "SECONDS:text,..." after the start.
    mp_chat: String,
    /// Airline-emergency mission to fly (missions/), and fly it with the scripted pilot.
    mission: Option<String>,
    mission_bot: bool,
    /// Live traffic (adsb/): replay a recording; force on / off for this session.
    adsb_fixture: Option<String>,
    adsb: Option<bool>,
    /// Dogfight quick start (weapons/): format, opponent, difficulty, AI-flown.
    dogfight: Option<weapons::setup::Config>,
    /// Headless: fast-forward until this happens in the fight (guns, hit, lock, missile, flares, smoke, eject, kill), then shoot.
    shot_when: Option<String>,
    /// Headless: eject / bail out at this many seconds.
    eject_at: Option<f32>,
    /// Chase camera on the (first) opponent.
    view_bandit: bool,
}

fn parse_vec3(s: &str) -> Option<Vec3> {
    let v: Vec<f32> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
    (v.len() == 3).then(|| vec3(v[0], v[1], v[2]))
}

/// "HH.MM" or "HH:MM" -> hours.
/// "lat,lon,height_agl".
fn parse_geo(s: &str) -> Option<(f64, f64, f32)> {
    let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
    (v.len() == 3).then(|| (v[0], v[1], v[2] as f32))
}

fn parse_time(s: &str) -> Option<f32> {
    let mut it = s.split(|c| c == '.' || c == ':');
    let h: f32 = it.next()?.parse().ok()?;
    let m: f32 = it.next().map(|m| m.parse().unwrap_or(0.0)).unwrap_or(0.0);
    Some(h + m / 60.0)
}

fn parse_args() -> Args {
    parse_args_from(std::env::args().skip(1).collect())
}

/// Browser: the page URL's query is the command line
/// (`?airport=EGLC&aircraft=kestrel&bot` → `--airport EGLC --aircraft kestrel --bot`).
#[cfg(target_arch = "wasm32")]
fn web_args() -> Args {
    let mut argv = Vec::new();
    for (k, v) in platform::web::query() {
        argv.push(format!("--{k}"));
        if !v.is_empty() {
            argv.push(v);
        }
    }
    parse_args_from(argv)
}

fn parse_args_from(argv: Vec<String>) -> Args {
    let mut a = Args {
        lat: 51.5045,
        lon: 0.0300,
        airport: None,
        airport_ident: None,
        aircraft: 0,
        aircraft_given: false,
        start: Start::Runway,
        start_at: None,
        shot: None,
        width: 1920,
        height: 1080,
        cam: CamMode::Chase,
        seconds: 0.0,
        throttle: None,
        heading: None,
        eye: None,
        eye_ll: None,
        look_ll: None,
        look: None,
        menu: false,
        menu_screen: None,
        no_menu: false,
        stats: false,
        realtime: false,
        warp: 1,
        radius: None,
        traffic: None,
        map_zoom: None,
        map_mode: None,
        hud_mode: None,
        then_reset: false,
        bot: false,
        watch: false,
        soak: false,
        shot_every: 0.0,
        gear: None,
        flaps: None,
        spoilers: None,
        reverse: false,
        fail: 0,
        fail_names: Vec::new(),
        cold_dark: false,
        panel: None,
        radio_panel: None,
        comms_panel: false,
        settings_cat: None,
        settings_search: None,
        settings_from_pause: false,
        palette: None,
        high_contrast: false,
        press: Vec::new(),
        route: None,
        frame_dt: None,
        ap: false,
        no_hud: false,
        hours: None,
        day_of_year: None,
        weather: None,
        live: true,
        live_given: false,
        metar: None,
        rebase_check: false,
        mute: false,
        audio_render: None,
        audio_script: String::new(),
        audio_until: None,
        vsync: None,
        location_given: false,
        mp_connect: None,
        mp_name: None,
        mp_password: String::new(),
        mp_wait: 0,
        mp_chat: String::new(),
        mission: None,
        mission_bot: false,
        adsb_fixture: None,
        adsb: None,
        dogfight: None,
        shot_when: None,
        eject_at: None,
        view_bandit: false,
    };
    let mut i = 0;
    let next = |i: &mut usize| -> String {
        *i += 1;
        argv.get(*i).cloned().unwrap_or_default()
    };
    while i < argv.len() {
        match argv[i].as_str() {
            "--lat" => {
                a.lat = next(&mut i).parse().unwrap_or(a.lat);
                a.location_given = true;
            }
            "--lon" => {
                a.lon = next(&mut i).parse().unwrap_or(a.lon);
                a.location_given = true;
            }
            "--airport" => {
                a.airport = Some(next(&mut i));
                a.location_given = true;
            }
            "--aircraft" => {
                let v = next(&mut i);
                a.aircraft = flight::index_of(&v).unwrap_or(0);
                a.aircraft_given = true;
            }
            "--start" => a.start = Start::parse(&next(&mut i)).unwrap_or(Start::Runway),
            "--start-at" => a.start_at = startpos::Spec::decode(&next(&mut i)),
            "--shot" => a.shot = Some(next(&mut i)),
            "--dogfight" => {
                let mut cfg = a.dogfight.take().unwrap_or_default();
                cfg.opponent = String::new();
                if let Some(f) = argv.get(i + 1).and_then(|v| weapons::setup::Format::parse(v)) {
                    cfg.format = f;
                    i += 1;
                }
                a.dogfight = Some(cfg);
            }
            "--opponent" => {
                let v = flight::resolve_id(&next(&mut i));
                a.dogfight.get_or_insert_with(Default::default).opponent = v;
            }
            "--difficulty" => {
                let v = next(&mut i);
                a.dogfight.get_or_insert_with(Default::default).difficulty = weapons::ai::Difficulty::parse(&v).unwrap_or(weapons::ai::Difficulty::Average);
            }
            "--dogfight-ai" => a.dogfight.get_or_insert_with(Default::default).autopilot = true,
            "--shot-when" => a.shot_when = Some(next(&mut i)),
            "--eject-at" => a.eject_at = next(&mut i).parse().ok(),
            "--view-bandit" => a.view_bandit = true,
            "--width" => a.width = next(&mut i).parse().unwrap_or(1920),
            "--height" => a.height = next(&mut i).parse().unwrap_or(1080),
            "--cam" => a.cam = CamMode::parse(&next(&mut i)).unwrap_or(CamMode::Chase),
            "--bot" => a.bot = true,
            "--watch" => {
                a.bot = true;
                a.watch = true;
            }
            "--soak" => a.soak = true,
            "--ap" => a.ap = true,
            "--shot-every" => a.shot_every = next(&mut i).parse().unwrap_or(0.0),
            "--gear" => a.gear = Some(next(&mut i) != "up"),
            "--flaps" => a.flaps = next(&mut i).parse().ok(),
            "--spoilers" => a.spoilers = next(&mut i).parse().ok(),
            "--reverse" => a.reverse = true,
            "--fail" => {
                let v = next(&mut i);
                match v.parse() {
                    Ok(bits) => a.fail = bits,
                    Err(_) => a.fail_names.extend(v.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty())),
                }
            }
            "--cold-dark" => a.cold_dark = true,
            "--route" => a.route = Some(next(&mut i)),
            "--press" => a.press.extend(next(&mut i).split(',').filter_map(|k| input::key_from_id(k.trim()))),
            "--panel" if argv.get(i + 1).is_some_and(|v| v == "radios" || v == "fms") => a.radio_panel = Some(if next(&mut i) == "fms" { 1 } else { 0 }),
            "--panel" if argv.get(i + 1).is_some_and(|v| v == "comms") => {
                i += 1;
                a.comms_panel = true;
            }
            "--panel" => {
                a.panel = Some(match next(&mut i).as_str() {
                    "armed" | "schedule" | "random" => 1,
                    "systems" => 2,
                    _ => 0,
                })
            }
            // Realtime test at a fixed display rate (e.g. 144): steps the sim by 1/fps per frame.
            "--fps" => a.frame_dt = next(&mut i).parse::<f32>().ok().filter(|f| *f > 1.0).map(|f| 1.0 / f),
            "--seconds" => a.seconds = next(&mut i).parse().unwrap_or(0.0),
            "--throttle" => a.throttle = next(&mut i).parse().ok(),
            "--heading" => a.heading = next(&mut i).parse().ok(),
            "--eye" => a.eye = parse_vec3(&next(&mut i)),
            "--look" => a.look = parse_vec3(&next(&mut i)),
            "--eye-ll" => a.eye_ll = parse_geo(&next(&mut i)),
            "--look-ll" => a.look_ll = parse_geo(&next(&mut i)),
            "--menu" => a.menu = true,
            "--menu-screen" => a.menu_screen = Screen::parse(&next(&mut i)),
            "--settings-category" => a.settings_cat = ui::settings_schema::Cat::parse(&next(&mut i)),
            "--settings-search" => a.settings_search = Some(next(&mut i)),
            "--settings-from" => a.settings_from_pause = next(&mut i) == "pause",
            "--palette" => a.palette = ui::settings::Palette::parse(&next(&mut i)),
            "--high-contrast" => a.high_contrast = true,
            "--no-menu" => a.no_menu = true,
            "--stats" => a.stats = true,
            "--realtime" => a.realtime = true,
            "--warp" => a.warp = next(&mut i).parse().unwrap_or(1).clamp(1, 64),
            "--then-reset" => a.then_reset = true,
            "--rebase-check" => a.rebase_check = true,
            "--mute" => a.mute = true,
            "--audio-render" => a.audio_render = Some(next(&mut i)),
            "--audio-script" => a.audio_script = next(&mut i),
            "--audio-until" => a.audio_until = Some(next(&mut i)),
            "--no-hud" => a.no_hud = true,
            "--mission" => a.mission = Some(next(&mut i)),
            "--mission-bot" => a.mission_bot = true,
            "--hud" => {
                a.hud_mode = match next(&mut i).as_str() {
                    "minimal" => Some(ui::settings::HudMode::Minimal),
                    "off" => Some(ui::settings::HudMode::Off),
                    _ => Some(ui::settings::HudMode::Full),
                }
            }
            "--map" => {
                a.map_mode = match next(&mut i).as_str() {
                    "large" => Some(ui::flight::MapMode::Large),
                    "off" => Some(ui::flight::MapMode::Off),
                    _ => Some(ui::flight::MapMode::Corner),
                }
            }
            "--vsync" => a.vsync = Some(next(&mut i) != "off"),
            "--connect" | "--mp-connect" => a.mp_connect = Some(next(&mut i)),
            "--name" | "--mp-name" => a.mp_name = Some(next(&mut i)),
            "--password" | "--mp-password" => a.mp_password = next(&mut i),
            "--mp-wait" => a.mp_wait = next(&mut i).parse().unwrap_or(0),
            "--mp-chat" => a.mp_chat = next(&mut i),
            // --time HH.MM (local clock), --date MM-DD, --weather clear|few|scattered|broken|overcast
            "--time" => a.hours = parse_time(&next(&mut i)),
            "--date" => {
                let d = next(&mut i);
                let mut it = d.split('-').filter_map(|x| x.parse::<u32>().ok());
                if let (Some(m), Some(dd)) = (it.next(), it.next()) {
                    a.day_of_year = Some(atmosphere::day_of_year(m, dd));
                }
            }
            "--weather" => {
                let w = next(&mut i).to_lowercase();
                a.weather = clouds::Weather::ALL.into_iter().find(|x| x.name().to_lowercase() == w);
            }
            "--live" => {
                a.live = true;
                a.live_given = true;
            }
            "--no-live" => {
                a.live = false;
                a.live_given = true;
            }
            "--metar" => a.metar = Some(next(&mut i)),
            "--adsb-fixture" => a.adsb_fixture = Some(next(&mut i)),
            "--adsb" => a.adsb = Some(true),
            "--no-adsb" => a.adsb = Some(false),
            "--map-zoom" => a.map_zoom = next(&mut i).parse().ok(),
            "--radius-km" => a.radius = next(&mut i).parse::<f32>().ok().map(|k| k * 1000.0),
            "--traffic" => a.traffic = traffic::Level::parse(&next(&mut i)),
            "-h" | "--help" => {
                println!("{}", include_str!("main.rs").lines().take(20).collect::<Vec<_>>().join("\n"));
                std::process::exit(0);
            }
            other => eprintln!("ignoring unknown argument {other}"),
        }
        i += 1;
    }
    // Missions are single-player.
    if a.mission.is_some() && a.mp_connect.is_some() {
        eprintln!("--mission ignored: missions are single-player (drop --connect to fly one)");
        a.mission = None;
    }
    if let Some(id) = a.mission.clone() {
        match missions::defs::by_id(&id) {
            Some(m) => a = args_for_mission(&a, m),
            None => {
                eprintln!("unknown mission {id} (missions: {})", missions::defs::all().iter().map(|m| m.id.as_str()).collect::<Vec<_>>().join(", "));
                a.mission = None;
            }
        }
    }
    if let Some(s) = &a.start_at {
        a.start = s.as_start();
    }
    a
}

/// Arguments for flying mission `m`: its aircraft, the start (scenery around
/// it), conditions (time, date, METAR, no live weather).
fn args_for_mission(base: &Args, m: &missions::defs::Mission) -> Args {
    let mut a = base.clone();
    a.mission = Some(m.id.clone());
    a.aircraft = m.aircraft_index().unwrap_or(0);
    a.aircraft_given = true;
    let (lat, lon) = missions::start_latlon(m, (m.target.lat, m.target.lon));
    a.lat = lat;
    a.lon = lon;
    a.location_given = true;
    a.airport = None;
    a.airport_ident = Some(m.target.airport.clone());
    a.start_at = Some(if m.start.on_runway {
        startpos::Spec::Runway(m.target.runway.clone())
    } else {
        startpos::Spec::Air { lat, lon, alt_m: m.start.alt_ft * 0.3048, hdg: (m.target.course_deg + m.start.hdg_rel).rem_euclid(360.0) }
    });
    a.start = a.start_at.as_ref().map(|s| s.as_start()).unwrap_or(Start::Runway);
    a.live = false;
    a.live_given = true;
    a.hours = Some(m.hours);
    a.day_of_year = Some(m.day);
    a.metar = (!m.metar.is_empty()).then(|| m.metar.clone());
    a.weather = None;
    a.bot = false;
    a.watch = false;
    a.heading = None;
    a
}

/// Resolve `--airport`: OurAirports first (exact ident / ICAO / IATA, else
/// the best search hit), Nominatim as a fallback.
#[cfg_attr(target_arch = "wasm32", allow(clippy::needless_return))]
fn resolve_airport(a: &mut Args, db: Option<&airports::Db>) {
    let Some(code) = a.airport.clone() else { return };
    if let Some(db) = db {
        let hit = db.index_of(&code).or_else(|| db.search(&code, 1, |_| true).first().copied());
        if let Some(i) = hit {
            let ap = &db.airports[i];
            eprintln!("airport {code}: {} {} ({:.4}, {:.4})", ap.ident, ap.name, ap.lat, ap.lon);
            a.lat = ap.lat;
            a.lon = ap.lon;
            a.airport_ident = Some(ap.ident.clone());
            return;
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    match tiles::resolve_airport(&code) {
        Ok((lat, lon, name)) => {
            eprintln!("airport {code}: {name} ({lat:.4}, {lon:.4})");
            a.lat = lat;
            a.lon = lon;
        }
        Err(e) => eprintln!("could not resolve airport {code}: {e}; using {:.4}, {:.4}", a.lat, a.lon),
    }
}

/// The OurAirports airport nearest a point (for the window title / plan).
fn nearest_airport_ident(db: &airports::Db, lat: f64, lon: f64) -> Option<String> {
    db.near(lat, lon, 15.0).into_iter().map(|(i, _)| &db.airports[i]).find(|a| a.kind != airports::Kind::Seaplane).map(|a| a.ident.clone())
}

// ---------------------------------------------------------------- session

/// Loading stages before the aircraft exists.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    /// Loading tiles around the start location (finds the runways).
    Area,
    /// Loading tiles around the chosen spawn point.
    Spawn(Vec2),
    Flying,
}

struct Session {
    world: World,
    sim: Option<Sim>,
    stage: Stage,
    started: Instant,
    runway_version: u64,
    /// Airport detail lights uploaded (airport/ light version) and the runway lights they join.
    airport_lights: u64,
    runway_lights: Vec<lights::LightInst>,
    /// The last origin shift (consumed by the soak / realtime loggers).
    last_rebase: Option<world::Rebase>,
    /// Set after a restart/teleport far away: the sim waits until the
    /// scenery around this point has streamed in.
    catch_up: Option<Vec2>,
    /// A start picked in the departure screen.
    custom: Option<startpos::CustomStart>,
    /// The arguments it was started with (aircraft, start, bot, time).
    args: Args,
}

impl Session {
    fn new(args: &Args, db: Option<Arc<airports::Db>>, custom: Option<startpos::CustomStart>) -> Session {
        let mut world = World::new(args.lat, args.lon, WORKERS);
        if let Some(r) = args.radius {
            world.radius = r;
        }
        world.traffic.set_level(args.traffic.unwrap_or_else(traffic::Level::default_for_platform));
        if let Some(db) = db {
            world.set_airport_db(db);
        }
        // Real weather & time (weather.rs): on unless switched off; --time / --date
        // switch live time off, --weather / --metar fix the weather.
        world.set_live(args.live);
        if args.hours.is_some() || args.day_of_year.is_some() {
            world.live_time = false;
        }
        if args.day_of_year.is_some() {
            world.live_date = false;
        }
        if let Some(w) = args.weather {
            world.set_weather_preset(w);
        }
        if let Some(m) = &args.metar {
            match weather::Wx::from_metar_str(m) {
                Some(wx) => world.set_wx(wx),
                None => eprintln!("could not decode --metar {m}"),
            }
        }
        world.set_focus(Vec2::ZERO, Vec2::ZERO);
        // Browser: start downloading the aircraft alongside the scenery.
        let _ = aircraft_assets(args.aircraft);
        Session { world, sim: None, stage: Stage::Area, started: Instant::now(), runway_version: u64::MAX, airport_lights: u64::MAX, runway_lights: Vec::new(), last_rebase: None, catch_up: None, custom, args: args.clone() }
    }

    /// Advance loading; returns a status line while still loading.
    fn advance_loading(&mut self) -> Option<String> {
        let args = self.args.clone();
        let w = &mut self.world;
        match self.stage {
            Stage::Area => {
                // A chosen start with known runways: go straight to its spawn point.
                if let Some(p) = self.custom.as_ref().and_then(|c| startpos::point(w, c)) {
                    self.stage = Stage::Spawn(p);
                    return self.advance_loading();
                }
                w.set_focus(Vec2::ZERO, Vec2::ZERO);
                let (ready, total) = w.progress(Vec2::ZERO, SPAWN_RADIUS);
                if ready >= total && w.uploads.is_empty() && w.gpu_pending.is_empty() {
                    let p = Sim::spawn_point(w, args.start, args.aircraft);
                    eprintln!(
                        "area ready in {:.1}s: {} tiles, {} runways{}",
                        self.started.elapsed().as_secs_f32(),
                        w.tile_count(),
                        w.runways.len(),
                        w.runways.first().map(|r| format!(" (longest {} {:.0} m)", r.name, r.length())).unwrap_or_default()
                    );
                    self.stage = Stage::Spawn(p);
                    return self.advance_loading();
                }
                Some(format!("Loading scenery\n{ready} / {total} tiles"))
            }
            Stage::Spawn(p) => {
                w.set_focus(p, Vec2::ZERO);
                let (ready, total) = w.progress(p, SPAWN_RADIUS);
                if ready >= total && w.uploads.is_empty() && w.gpu_pending.is_empty() {
                    if let Some(m) = aircraft_assets(args.aircraft) {
                        return Some(m);
                    }
                    let mut sim = Sim::new(w, args.aircraft, args.start);
                    if let Some(c) = &self.custom {
                        sim.custom_start = Some(c.clone());
                        sim.reset(w);
                    }
                    if let Some(h) = args.heading {
                        sim.set_heading(h.to_radians());
                    }
                    // An airline-emergency mission takes the start over (missions/).
                    if let Some(id) = &args.mission {
                        sim.begin_mission(id, args.mission_bot);
                    }
                    sim.hours = args.hours.unwrap_or(sim.hours);
                    sim.day_of_year = args.day_of_year.unwrap_or(sim.day_of_year);
                    if args.bot {
                        sim.watch = args.watch;
                        sim.enable_bot(w);
                        if args.watch {
                            sim.cam = CamMode::Cinematic;
                        }
                    }
                    if let Some(cfg) = args.dogfight.clone() {
                        let mut cfg = cfg;
                        if cfg.opponent.is_empty() {
                            cfg.opponent = sim.def().id.to_string();
                        }
                        sim.start_dogfight(w, cfg);
                        if args.view_bandit {
                            sim.view_unit = sim.combat.as_ref().and_then(|c| c.units.iter().find(|u| u.team != 0).map(|u| u.id));
                        }
                    }
                    if let Some((msg, _)) = &sim.note {
                        eprintln!("note: {msg}");
                    }
                    eprintln!(
                        "spawn ready in {:.1}s: {} tiles (+{} far), {} buildings, {} trees; runway {:?}",
                        self.started.elapsed().as_secs_f32(),
                        w.tile_count(),
                        w.far_count(),
                        w.building_count(),
                        w.tree_count(),
                        sim.runway.as_ref().map(|r| (r.name.clone(), r.length().round(), r.width, r.source))
                    );
                    // Air starts far from the loaded spawn area (e.g. a 10 nm final) catch up first.
                    let ap = sim.ac.pos;
                    if vec2(ap.x, ap.z).distance(p) > 2000.0 {
                        self.catch_up = Some(vec2(ap.x, ap.z));
                    }
                    self.sim = Some(sim);
                    self.stage = Stage::Flying;
                    return None;
                }
                Some(format!("Loading scenery\n{ready} / {total} tiles"))
            }
            Stage::Flying => None,
        }
    }

    /// After a jump of the aircraft (restart, new start position), hold the
    /// sim until nearby tiles are loaded. Returns a status line while waiting.
    fn check_catch_up(&mut self, before: Option<Vec3>) -> Option<String> {
        let now = self.sim.as_ref().map(|s| s.ac.pos)?;
        if before.is_some_and(|b| vec2(b.x - now.x, b.z - now.z).length() > 2000.0)
            || (self.catch_up.is_none() && self.sim.as_ref().is_some_and(|s| s.needs_respawn))
        {
            self.catch_up = Some(vec2(now.x, now.z));
        }
        let p = self.catch_up?;
        let (ready, total) = self.world.progress(p, SPAWN_RADIUS);
        if ready >= total && self.world.uploads.is_empty() && self.world.gpu_pending.is_empty() {
            self.catch_up = None;
            // The chosen airfield has loaded: put the aircraft on its runway now.
            if let Some(sim) = self.sim.as_mut() {
                if sim.needs_respawn {
                    sim.reset(&self.world);
                    let n = sim.ac.pos;
                    if vec2(n.x, n.z).distance(p) > 2000.0 {
                        self.catch_up = Some(vec2(n.x, n.z));
                        return Some("Loading scenery".into());
                    }
                }
            }
            return None;
        }
        Some(format!("Loading scenery\n{ready} / {total} tiles"))
    }

    /// Floating origin: once the aircraft is far from the origin, re-centre
    /// the frame on it and shift everything (world, sim, renderer) together.
    fn maybe_rebase(&mut self, renderer: &mut Renderer) {
        let Some(sim) = self.sim.as_mut() else { return };
        let p = vec2(sim.ac.pos.x, sim.ac.pos.z);
        if !self.world.wants_rebase(p) || std::env::var_os("SKYWARD_NO_REBASE").is_some() {
            return;
        }
        let t0 = Instant::now();
        let r = self.world.rebase(p);
        sim.rebase(&r);
        if let Some(c) = &mut self.catch_up {
            *c = r.ground(*c);
        }
        renderer.rebase(r.delta);
        renderer.set_frame(self.world.proj, self.world.pattern_frame(), self.world.home);
        if std::env::var_os("SKYWARD_REBASE_LOG").is_some() || self.world.rebases <= 1 {
            eprintln!(
                "origin shift #{} to {:.4},{:.4}: delta ({:.0}, {:.0}) m, x scale {:+.4}%, {:.2} ms",
                self.world.rebases,
                self.world.proj.lat0,
                self.world.proj.lon0,
                r.delta.x,
                r.delta.z,
                (r.map.sx - 1.0) * 100.0,
                t0.elapsed().as_secs_f64() * 1000.0
            );
        }
        self.last_rebase = Some(r);
    }

    /// Point streaming at the aircraft (called every frame while flying).
    fn follow(&mut self) {
        if let Some(sim) = &mut self.sim {
            let p = sim.ac.pos;
            let v = sim.ac.vel;
            sim.lat_lon = self.world.proj.to_latlon(vec2(p.x, p.z));
            self.world.set_focus(vec2(p.x, p.z), vec2(v.x, v.z));
        }
    }
}

/// Browser: an aircraft's model and textures download when it is chosen.
/// None when ready (always natively); else a loading-screen status line.
#[cfg_attr(target_arch = "wasm32", allow(clippy::needless_return))]
fn aircraft_assets(sel: usize) -> Option<String> {
    #[cfg(target_arch = "wasm32")]
    {
        use platform::web::DirState;
        let cat = flight::catalogue();
        let def = cat.get(sel).or(cat.first())?;
        // (The FPV quad's model is procedural: nothing to download.)
        if def.rotor.as_ref().is_some_and(|r| r.quad().is_some()) {
            return None;
        }
        let id = def.id;
        let dir = format!("aircraft/{id}/");
        return match platform::web::dir_state(&dir) {
            DirState::Ready | DirState::Failed(_) => None,
            DirState::Loading(d, t) => Some(format!("Loading aircraft\n{:.1} / {:.1} MB", d as f64 / 1e6, (t as f64 / 1e6).max(0.1))),
            DirState::Absent => {
                platform::web::preload_dir(&dir);
                Some("Loading aircraft\n".into())
            }
        };
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let _ = sel;
        None
    }
}

/// Move finished tiles / runways / clouds to the GPU and HUD. `budget` limits
/// the bytes uploaded this frame (None = everything pending).
fn sync_gpu(s: &mut Session, renderer: &mut Renderer, ui: &mut Ui, gpu: &Gpu, budget: Option<u64>) -> u64 {
    let (device, queue) = (&gpu.device, &gpu.queue);
    s.maybe_rebase(renderer);
    let w = &mut s.world;
    renderer.set_frame(w.proj, w.pattern_frame(), w.home);
    w.update();
    for id in w.take_evicted() {
        renderer.remove_tile(id);
        ui.st.map_tiles.remove(&id);
    }
    for (id, m) in w.take_map_vectors() {
        ui.st.map_tiles.insert(id, m);
    }
    // Buffers are created at once (cheap); data is copied in budgeted chunks.
    while let Some(t) = w.uploads.pop_front() {
        renderer.queue_tile(device, t);
    }
    let spent = renderer.pump_uploads(queue, budget);
    w.gpu_pending = renderer.pending_tiles().collect();
    w.gpu_bytes = renderer.gpu_bytes();
    renderer.airport.sync(device, &w.airport_detail);
    let runways_changed = w.runway_version != s.runway_version;
    if runways_changed {
        s.runway_version = w.runway_version;
        renderer.set_runways(device, &scene::runway_mesh(&w.runways));
        s.runway_lights = lights::airport_lights_for(w);
    }
    // Runway lights + the airport detail lights (taxiways, stop bars, floods), current frame.
    if runways_changed || w.airport_detail.light_version != s.airport_lights {
        s.airport_lights = w.airport_detail.light_version;
        let mut all = s.runway_lights.clone();
        all.extend(w.airport_detail.lights(&w.proj));
        renderer.set_airport_lights(device, &all);
    }
    // Weather (weather.rs / world.rs): live data + real time, wind drift, season.
    match s.sim.as_mut() {
        Some(sim) => {
            if let Some((h, d)) = w.tick_weather(sim.time, (sim.hours, sim.day_of_year)) {
                sim.hours = h;
                sim.day_of_year = d;
            }
        }
        None => {
            w.tick_weather(0.0, (s.args.hours.unwrap_or(sim::DEFAULT_HOURS), s.args.day_of_year.unwrap_or(sim::DEFAULT_DAY)));
        }
    }
    // Turbulence setting, cockpit head shake, the aircraft's own wake (wx/).
    w.set_turbulence(ui.st.settings.wx.turbulence);
    if let Some(sim) = s.sim.as_mut() {
        let (span, mass) = (sim.def().wingspan, sim.def().mass);
        let gh = flight::Environment::ground_height(w, vec2(sim.ac.pos.x, sim.ac.pos.z));
        let airborne = !sim.ac.on_ground && sim.ac.crashed.is_none();
        w.wxs.wakes.push(wx::wake::OWN_AIRCRAFT, w.to_field(sim.ac.pos), sim.ac.vel, mass, span, airborne, gh, sim.time);
        w.wxs.wakes.wind = vec3(w.wind.aloft.x, 0.0, w.wind.aloft.y);
        // Other aircraft's wakes: multiplayer players and live ADS-B traffic.
        let t = sim.time;
        let mut gens: Vec<(u64, Vec3, Vec3, f32, f32)> = Vec::new();
        for (id, r) in &ui.st.mp.remotes {
            if let Some(d) = sim.defs.get(r.def).filter(|_| !r.crashed && !r.spectating) {
                gens.push((wx::wake::MP_BASE + *id as u64, r.pos, r.vel, d.mass, d.wingspan));
            }
        }
        for g in ui.st.adsb.wake_sources().into_iter().filter(|g| !g.on_ground) {
            let cat = format!("{:?}", g.wake).chars().next().unwrap_or('M');
            gens.push((wx::wake::ADSB_BASE + g.id as u64, g.pos, g.vel, wx::wake::mass_for_category(cat), g.span));
        }
        for (id, pos, vel, mass, span) in gens {
            let gh = flight::Environment::ground_height(w, vec2(pos.x, pos.z));
            let pf = w.to_field(pos);
            w.wxs.wakes.push(id, pf, vel, mass, span, pos.y - gh > 8.0, gh, t);
        }
    }
    renderer.set_weather(&w.render_weather(), &w.clouds);
    spent
}

/// Replace the session (a new departure far away): clear what the renderer
/// and the map hold of the old world, and make sure clouds / runways re-upload.
fn replace_session(old: Option<Session>, new: &mut Session, renderer: &mut Renderer, ui: &mut Ui) {
    if let Some(o) = old {
        for id in o.world.tile_ids() {
            renderer.remove_tile(id);
        }
        for id in renderer.pending_tiles().collect::<Vec<_>>() {
            renderer.remove_tile(id);
        }
        new.world.cloud_version = o.world.cloud_version + 1;
    }
    ui.st.map_tiles.clear();
    renderer.reset_history();
    new.runway_version = u64::MAX;
    new.airport_lights = u64::MAX;
}

// ---------------------------------------------------------------- shared frame

struct Ui {
    ctx: egui::Context,
    renderer: egui_wgpu::Renderer,
    canvases: ui::canvas::Canvases,
    /// Glass cockpit displays (offscreen, mapped onto the 3D cockpit screens).
    cockpit: cockpit::displays::Displays,
    st: UiState,
    hide_hud: bool,
    /// Aircraft thumbnails (kept alive here; ids in `st.thumbs`).
    thumbs: Vec<egui::TextureHandle>,
    /// Pointer clicks in the menus (UI click sound) and the sound device line.
    audio_clicks: u32,
    audio_status: String,
    fps: f32,
}

impl Ui {
    fn new(device: &wgpu::Device, format: wgpu::TextureFormat, settings: ui::settings::Settings) -> Ui {
        let ctx = egui::Context::default();
        ui::install_fonts(&ctx);
        ui::install_style(&ctx);
        Ui {
            ctx,
            renderer: egui_wgpu::Renderer::new(device, format, egui_wgpu::RendererOptions::default()),
            canvases: ui::canvas::Canvases::default(),
            cockpit: Default::default(),
            st: UiState::new(settings),
            hide_hud: false,
            thumbs: Vec::new(),
            audio_clicks: 0,
            audio_status: String::new(),
            fps: 0.0,
        }
    }
}

/// The loading screen's lines: destination, aircraft and start, progress.
fn loading_text(st: &UiState, status: &str) -> String {
    let a = st.db.as_ref().and_then(|d| d.get(&st.plan.airport)).map(|a| format!("{} · {}", a.code(), a.name)).unwrap_or_else(|| "Loading".into());
    let tiles = status.lines().last().unwrap_or("").to_string();
    let def = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].name;
    format!("{a}\n{def} · {}\n{tiles}", st.plan.spec.label())
}

/// "Title\n... a / b tiles" → (title, detail, fraction).
fn loading_info(msg: &str) -> (String, String, Option<f32>) {
    let mut lines = msg.lines();
    let title = lines.next().unwrap_or("Loading").to_string();
    let rest: Vec<&str> = lines.collect();
    let frac = rest.last().and_then(|l| {
        let mut it = l.split_whitespace();
        let a: f32 = it.next()?.parse().ok()?;
        it.next()?;
        let b: f32 = it.next()?.parse().ok()?;
        Some(a / b.max(1.0))
    });
    (title, rest.join(" · "), frac)
}

/// What the traffic simulation needs of this frame: time, local hours and
/// the view (level of detail, culling, lights after dusk).
fn traffic_frame(fv: &FrameView, time: f64, w: u32, h: u32, level: traffic::Level) -> traffic::Frame {
    let fwd = -fv.view.inverse().z_axis.truncate().normalize_or(Vec3::NEG_Z);
    let aspect = w as f32 / h.max(1) as f32;
    let half_diag = ((fv.fov_y * 0.5).tan() * (1.0 + aspect * aspect).sqrt()).atan();
    // Detailed meshes out to ~160 m at 1440p / 60°, scaled with the pixel density.
    let px_per_m = h as f32 * 0.5 / (fv.fov_y * 0.5).tan();
    let lod0 = (160.0 * px_per_m / 1250.0).clamp(70.0, 320.0);
    // Head / tail lights out to the simulated radius from dusk.
    let lights = fv.sun_dir.normalize_or(Vec3::Y).y < 0.08;
    let view = traffic::View {
        eye: fv.eye,
        fwd,
        cos_half: (half_diag + 0.2).min(std::f32::consts::PI).cos(),
        lod0_dist: lod0,
        draw_dist: level.draw_dist(),
        light_dist: if lights { level.radius() } else { 0.0 },
    };
    traffic::Frame { time, hours: fv.clock.local_hours, view }
}

#[allow(clippy::too_many_arguments)]
fn render_frame(
    gpu: &Gpu,
    renderer: &mut Renderer,
    ui: &mut Ui,
    session: Option<&mut Session>,
    target: &wgpu::TextureView,
    w: u32,
    h: u32,
    raw: egui::RawInput,
    ppp: f32,
    fps: f32,
    loading: Option<&str>,
    view_override: Option<(Vec3, Vec3)>,
) {
    let device = &gpu.device;
    let queue = &gpu.queue;
    let mut enc = device.create_command_encoder(&Default::default());
    ui.st.frame_no += 1;
    let zoom = ui::zoom_for(h as f32, ppp, ui.st.settings.ui_scale);
    ui.ctx.set_zoom_factor(zoom);
    let mut raw = raw;
    // Offscreen frames give the screen in pixels: express it in (zoomed) points.
    if let Some(r) = raw.screen_rect.filter(|r| (r.width() - w as f32).abs() < 0.5 && (r.height() - h as f32).abs() < 0.5 && ppp == 1.0) {
        raw.screen_rect = Some(egui::Rect::from_min_size(r.min, r.size() / zoom));
    }
    let flying = session.as_ref().is_some_and(|s| s.sim.is_some());
    if let Some(s) = session.as_ref().filter(|_| flying) {
        let _ = s;
    }
    let mut session = session;
    match session.as_deref_mut() {
        Some(s) if flying => {
            let sim = s.sim.as_mut().unwrap();
            sim.qnh_hpa = s.world.wx.qnh_hpa;
            // The player's settings the simulation reads (ui/settings.rs).
            sim.prefs = ui.st.settings.sim_prefs();
            if !sim.prefs_init {
                sim.prefs_init = true;
                let player = sim.bot.is_none() && sim.mission.is_none();
                if player && sim.prefs.cockpit_first && sim.cam == CamMode::Chase {
                    sim.cam = CamMode::Cockpit;
                }
                sim.failures.random.on |= player && sim.prefs.random_failures;
            }
            let (mut fv, mut dynm) = sim.frame(w as f32 / h as f32);
            // Multiplayer spectator (net/): our aircraft is hidden, the camera follows another.
            let mut view_override = view_override;
            if ui.st.mp.spectate.is_some() {
                fv.aircraft = None;
                fv.vfx = None;
                fv.acl.n_sprites = 0;
                fv.acl.n_spots = 0;
                dynm = scene::MeshData::default();
                if view_override.is_none() {
                    view_override = ui.st.mp.spectate_view(ui.st.dt);
                }
            }
            // Live glass cockpit screens (only drawn with the cockpit interior).
            // (SKYWARD_NO_DISPLAYS=1 leaves the screens dark, for A/B timing.)
            if let Some(af) = fv.aircraft.as_mut().filter(|af| af.draws.iter().any(|d| af.asset.chunks[d.chunk as usize].display.is_some())) {
                if std::env::var_os("SKYWARD_NO_DISPLAYS").is_none() {
                    ui.cockpit.update(device, queue, &mut enc, sim, &s.world, &ui.st.map_tiles, af);
                }
            }
            if let Some((eye, look)) = view_override {
                // The aircraft mesh is relative to the sim's eye: re-express it.
                let d = fv.eye - eye;
                for v in &mut dynm.verts {
                    v.pos = (Vec3::from(v.pos) + d).into();
                }
                if let Some(af) = fv.aircraft.as_mut() {
                    af.shift(d);
                }
                for af in &mut fv.others {
                    af.shift(d);
                }
                fv = FrameView { eye, view: glam::camera::rh::view::look_at_mat4(eye, look, Vec3::Y), ..fv };
            }
            // Other players' aircraft (net/): real models, lights, afterburners — eye-relative.
            let (mut frames, mut lights) = (Vec::new(), Vec::new());
            if ui.st.mp.connected() {
                let (f, l, flames) = ui.st.mp.render_traffic(fv.eye);
                dynm.append(&flames, glam::Mat4::IDENTITY);
                (frames, lights) = (f, l);
                ui.st.mp.cam = Some((fv.eye, fv.view, fv.fov_y));
            } else {
                ui.st.mp.cam = None;
            }
            // Live real-world traffic (adsb/): after the players (they take precedence).
            {
                let lt_settings = ui.st.settings.live_traffic.clone();
                ui.st.adsb.update(sim, &s.world, ui.st.dt, &lt_settings);
                let mut flames = scene::MeshData::default();
                let slots = 96usize.saturating_sub(frames.len());
                let t = sim.time % 3600.0;
                frames.extend(ui.st.adsb.render(fv.eye, t, slots, &mut lights, &mut flames));
                dynm.append(&flames, glam::Mat4::IDENTITY);
                ui.st.adsb.cam = Some((fv.eye, fv.view, fv.fov_y));
            }
            renderer.set_traffic(frames, lights);
            // Browser: the scene may render below the output size (upscaled by the tonemap pass).
            #[cfg(target_arch = "wasm32")]
            let (w, h) = platform::web::scene_size(w, h);
            {
                // Airport instances (parked aircraft, vehicles, windsocks in the real wind).
                let wd = &s.world;
                let mut af = wd.airport_detail.frame(&wd.proj, fv.eye, sim.time, &|p| flight::Environment::wind(wd, p), &|xz| flight::Environment::ground_height(wd, xz));
                // Stands taken by live aircraft (adsb/): their procedural occupants are hidden.
                airport::hide_parked(&mut af, &ui.st.adsb.hidden, fv.eye);
                renderer.set_airport_frame(af);
            }
            {
                // Road traffic (traffic/): this frame's vehicles.
                let wd = &mut s.world;
                renderer.set_traffic_level(wd.traffic.level());
                let snap = wd.traffic.frame(traffic_frame(&fv, sim.time as f64, w, h, wd.traffic.level()));
                renderer.road_traffic.prepare(device, queue, snap, sim.time as f64, fv.eye);
            }
            // Storms, lightning, lenticulars, windscreen rain and wipers (wx/).
            let cockpit = sim.cam == sim::CamMode::Cockpit && view_override.is_none();
            fv.wx = Some(std::sync::Arc::new(wx::gpu::prepare(&s.world, sim, fv.eye, cockpit, fv.aircraft.as_ref(), &mut dynm)));
            renderer.render(device, queue, &mut enc, target, w, h, &fv, &dynm);
        }
        _ => {
            let _ = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.004, g: 0.006, b: 0.010, a: 1.0 }), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
    }
    let hide_hud = ui.hide_hud;
    let st = &mut ui.st;
    let t_ui = Instant::now();
    let full = ui.ctx.run_ui(raw, |u| {
        let ctx = u.ctx().clone();
        let ppp_now = ctx.pixels_per_point();
        let on_screen = matches!(st.screen, Screen::Flying | Screen::Pause);
        if let Some(s) = session.as_deref_mut().filter(|_| flying) {
            let Session { sim, world, .. } = s;
            let sim = sim.as_mut().unwrap();
            if on_screen && !hide_hud {
                // (Spectating: the watched aircraft is the subject, not our parked one.)
                if st.mp.spectate.is_none() {
                    ui::flight::draw(&ctx, st, sim, world, ppp_now, fps);
                }
                ui::mp::draw(&ctx, st, sim);
                adsb::ui::tags(&ctx, &st.adsb, sim.ac.pos, st.settings.alt);
            }
            if let (Some(msg), Screen::Flying) = (loading, st.screen) {
                let (t, d, f) = loading_info(msg);
                let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::NULL.with("catchup")));
                let sr = ctx.content_rect();
                let r = egui::Rect::from_center_size(egui::pos2(sr.center().x, sr.min.y + 120.0), egui::vec2(300.0, 64.0));
                ui::panel(&p, r, ui::sp::R_PANEL);
                p.text(r.min + egui::vec2(18.0, 12.0), egui::Align2::LEFT_TOP, t, ui::f_semi(14.0), ui::pal::TEXT);
                p.text(r.min + egui::vec2(18.0, 34.0), egui::Align2::LEFT_TOP, d, ui::f_ui(12.0), ui::pal::TEXT2);
                if let Some(f) = f {
                    let bar = egui::Rect::from_min_size(egui::pos2(r.min.x + 18.0, r.max.y - 8.0), egui::vec2((r.width() - 36.0) * f.clamp(0.0, 1.0), 3.0));
                    p.rect_filled(bar, 1.5, ui::pal::ACCENT);
                }
            }
        }
        if st.screen != Screen::Flying {
            let info = loading.map(loading_info);
            let li = info.as_ref().map(|(t, d, f)| (t.as_str(), d.as_str(), *f));
            if !flying && !matches!(st.screen, Screen::Loading | Screen::Departure) {
                // No scene yet: the overview map of the departure as the backdrop.
                let sr = ctx.content_rect();
                let p = ctx.layer_painter(egui::LayerId::background());
                if let Some(a) = st.db.as_ref().and_then(|d| d.get(&st.plan.airport).cloned()) {
                    let (cx, cy) = ui::overview::merc(a.lat, a.lon);
                    let v = ui::overview::View { cx, cy, zoom: 9.0, rect: sr };
                    st.picker.overview.draw(&p, &mut st.canvas, "overview", ppp_now, &v, true);
                } else {
                    p.rect_filled(sr, 0, ui::pal::BG);
                }
            }
            ui::draw_menus(&ctx, st, flying, li);
        }
    });
    let mut full = full;
    let t_tess = Instant::now();
    let mut prims = ui.ctx.tessellate(std::mem::take(&mut full.shapes), full.pixels_per_point);
    // Accessibility palettes (ui/palette.rs): colour-blind variants, high contrast.
    let pal_mode = ui::palette::Mode { palette: ui.st.settings.palette, high_contrast: ui.st.settings.high_contrast };
    ui::palette::apply(&mut prims, pal_mode);
    ui.st.canvas.palette = pal_mode;
    let t_canvas = Instant::now();
    for (id, deltas) in &full.textures_delta.set {
        for delta in deltas.iter() {
            ui.renderer.update_texture(device, queue, *id, delta);
        }
    }
    let mut extra = ui.canvases.render(device, queue, &mut enc, &mut ui.renderer, &mut ui.st.canvas);
    let sd = egui_wgpu::ScreenDescriptor { size_in_pixels: [w, h], pixels_per_point: full.pixels_per_point };
    extra.extend(ui.renderer.update_buffers(device, queue, &mut enc, &prims, &sd));
    if std::env::var_os("SKYWARD_UI_TIMING").is_some() {
        let ms = |a: Instant, b: Instant| (b - a).as_secs_f32() * 1000.0;
        let now = Instant::now();
        eprintln!("ui: build {:.2} ms, tessellate {:.2} ms, canvases + upload {:.2} ms", ms(t_ui, t_tess), ms(t_tess, t_canvas), ms(t_canvas, now));
    }
    {
        let mut pass = enc
            .begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("egui"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            })
            .forget_lifetime();
        ui.renderer.render(&mut pass, &prims, &sd);
    }
    queue.submit(extra.into_iter().chain([enc.finish()]));
    for id in &full.textures_delta.free {
        ui.renderer.free_texture(id);
    }
    full.textures_delta.clear();
    if ui.st.screen != Screen::Flying && ui.ctx.input(|i| i.pointer.any_click()) {
        ui.audio_clicks += 1;
    }
}

/// Render every aircraft once over the current scenery (a front-quarter
/// air-to-air shot, lit by the sun) for the aircraft cards.
fn render_thumbs(gpu: &Gpu, renderer: &mut Renderer, ui: &mut Ui, s: &Session, format: wgpu::TextureFormat) {
    if !ui.thumbs.is_empty() || s.sim.is_none() {
        return;
    }
    let (w, h) = (640u32, 400u32);
    let tex = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("thumb"),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = tex.create_view(&Default::default());
    let t0 = Instant::now();
    let defs = flight::catalogue();
    let base = s.sim.as_ref().unwrap();
    for (i, def) in defs.iter().enumerate() {
        let mut sim = Sim::new(&s.world, i, Start::City);
        sim.hours = base.hours;
        sim.day_of_year = base.day_of_year;
        sim.lat_lon = base.lat_lon;
        let sun = sim.sun_dir();
        let sh = vec2(sun.x, sun.z).normalize_or(vec2(0.0, -1.0));
        // Camera on the sun side; nose turned 50° towards it (front quarter).
        let cam_az = sh.x.atan2(-sh.y) + 0.5;
        let hdg = cam_az + 50f32.to_radians();
        let home = s.world.home_pos();
        let alt = if def.category == flight::Category::Glider { 700.0 } else { 900.0 };
        sim.ac.pos = vec3(home.x + 1500.0, alt, home.y - 1500.0);
        sim.set_heading(hdg);
        let (fv, dynm) = sim.frame(w as f32 / h as f32);
        let (pos, _) = sim.render_pose();
        let dir = vec3(cam_az.sin(), 0.0, -cam_az.cos()) * 12f32.to_radians().cos() + Vec3::Y * 12f32.to_radians().sin();
        let dist = if def.category == flight::Category::Drone { 0.62 } else { sim.size * 2.2 + 3.0 };
        let eye = pos + dir * dist;
        let mut dynm = dynm;
        let d = fv.eye - eye;
        for v in &mut dynm.verts {
            v.pos = (Vec3::from(v.pos) + d).into();
        }
        let mut fv = fv;
        if let Some(af) = fv.aircraft.as_mut() {
            af.shift(d);
        }
        let fv = FrameView { eye, view: glam::camera::rh::view::look_at_mat4(eye, pos, Vec3::Y), fov_y: 38f32.to_radians(), ..fv };
        renderer.reset_history();
        for _ in 0..3 {
            let mut enc = gpu.device.create_command_encoder(&Default::default());
            renderer.render(&gpu.device, &gpu.queue, &mut enc, &view, w, h, &fv, &dynm);
            gpu.queue.submit([enc.finish()]);
        }
        let mut px = gfx::read_texture(gpu, &tex, w, h);
        if matches!(format, wgpu::TextureFormat::Bgra8UnormSrgb | wgpu::TextureFormat::Bgra8Unorm) {
            for c in px.chunks_mut(4) {
                c.swap(0, 2);
            }
        }
        for c in px.chunks_mut(4) {
            c[3] = 255;
        }
        // `SKYWARD_THUMBS_OUT=web/thumbs`: also write them as JPEGs (the
        // browser build ships these instead of downloading every aircraft).
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(dir) = std::env::var_os("SKYWARD_THUMBS_OUT") {
            let rgb: Vec<u8> = px.chunks(4).flat_map(|c| [c[0], c[1], c[2]]).collect();
            let path = std::path::Path::new(&dir).join(format!("{}.jpg", def.id));
            let r = std::fs::File::create(&path).map_err(|e| e.to_string()).and_then(|f| {
                image::codecs::jpeg::JpegEncoder::new_with_quality(std::io::BufWriter::new(f), 82).encode(&rgb, w, h, image::ExtendedColorType::Rgb8).map_err(|e| e.to_string())
            });
            if let Err(e) = r {
                eprintln!("thumbnail {}: {e}", path.display());
            }
        }
        let img = egui::ColorImage::from_rgba_unmultiplied([w as usize, h as usize], &px);
        let handle = ui.ctx.load_texture(format!("thumb_{}", def.id), img, egui::TextureOptions::LINEAR);
        ui.st.thumbs.insert(def.id, handle.id());
        ui.thumbs.push(handle);
    }
    renderer.reset_history();
    eprintln!("aircraft thumbnails: {} in {:.0} ms", defs.len(), t0.elapsed().as_secs_f32() * 1000.0);
}

/// The plan (aircraft, airport, start) implied by the command line.
fn plan_from_args(args: &Args, db: Option<&airports::Db>) -> ui::Plan {
    let airport = args.airport_ident.clone().or_else(|| db.and_then(|d| nearest_airport_ident(d, args.lat, args.lon))).unwrap_or_else(|| "EGLC".into());
    let defs = flight::catalogue();
    let def = &defs[args.aircraft.min(defs.len() - 1)];
    let spec = match &args.start_at {
        Some(s) => s.clone(),
        None => match args.start {
            Start::Thermal => startpos::Spec::Thermal,
            _ if !def.has_engine => startpos::Spec::Thermal,
            _ => startpos::Spec::Runway(String::new()),
        },
    };
    ui::Plan { aircraft: args.aircraft, airport, spec, watch: args.watch, dogfight: args.dogfight.clone() }
}

/// Arguments for flying a plan with the chosen conditions.
fn args_for_plan(base: &Args, st: &UiState) -> (Args, Option<startpos::CustomStart>) {
    let mut a = base.clone();
    let plan = &st.plan;
    if let Some(ap) = st.db.as_ref().and_then(|d| d.get(&plan.airport)) {
        a.lat = ap.lat;
        a.lon = ap.lon;
        a.airport_ident = Some(ap.ident.clone());
    }
    a.aircraft = plan.aircraft;
    a.start = plan.spec.as_start();
    a.bot = plan.watch;
    a.watch = plan.watch;
    a.dogfight = plan.dogfight.clone();
    a.heading = None;
    let c = &st.settings.conditions;
    a.live = c.live;
    if c.live {
        a.hours = None;
        a.day_of_year = None;
        a.weather = None;
        a.metar = None;
    } else {
        a.hours = Some(c.hours);
        a.day_of_year = Some(c.day);
        a.metar = (!c.metar.trim().is_empty() && weather::Wx::from_metar_str(c.metar.trim()).is_some()).then(|| c.metar.trim().to_string());
        a.weather = clouds::Weather::ALL.into_iter().find(|w| w.name() == c.preset);
    }
    a.radius = Some(ui::settings::detail_radius(st.settings.detail));
    a.traffic = Some(traffic::Level::from_u8(st.settings.traffic.road));
    // A runway start with no end chosen yet: let the sim pick (into wind, long enough).
    let custom = match &plan.spec {
        startpos::Spec::Runway(e) | startpos::Spec::Final(e, _) if e.is_empty() => None,
        startpos::Spec::Thermal => None,
        s => Some(startpos::CustomStart { airport: plan.airport.clone(), spec: s.clone() }),
    };
    (a, custom)
}

/// A finished flight choice: remember it.
/// A multiplayer join succeeded: fly from the server's airport.
fn mp_start_flight(st: &mut UiState) {
    let Some(sv) = st.mp.server.clone() else { return };
    let db = st.db.clone();
    match db.as_ref().and_then(|d| d.index_of(&sv.airport).map(|i| (d, i))) {
        Some((d, i)) => ui::menus::pick_default_spec(st, d, i),
        None => {
            st.plan.airport = sv.airport.clone();
            st.plan.spec = startpos::Spec::Runway(String::new());
        }
    }
    st.plan.watch = false;
    st.actions.push(Action::Fly);
}

fn remember_plan(st: &mut UiState) {
    let id = st.defs[st.plan.aircraft.min(st.defs.len() - 1)].id.to_string();
    st.settings.last = Some(ui::settings::LastFlight { aircraft: id, airport: st.plan.airport.clone(), start: st.plan.spec.encode(), watch: st.plan.watch });
    let a = st.plan.airport.clone();
    st.settings.push_recent(&a);
    st.settings_dirty = true;
}

// ---------------------------------------------------------------- headless

/// `--route`: the flight plan (nav/).
fn apply_route(args: &Args, sim: &mut sim::Sim, world: &world::World) {
    if let Some(spec) = &args.route {
        let here = world.proj.to_latlon(glam::vec2(sim.ac.pos.x, sim.ac.pos.z));
        let ndb = nav::db::db();
        match sim.nav.route_from_spec(spec, here, ndb.as_deref(), world.airport_db.as_deref()) {
            Ok(r) => eprintln!("route: {r}"),
            Err(e) => eprintln!("route: {e}"),
        }
    }
}

/// Live traffic for this session: a replayed recording (`--adsb-fixture`),
/// or forced on / off (`--adsb`, `--no-adsb`; `--no-live` turns it off;
/// headless captures leave it off unless asked).
fn setup_adsb(args: &Args, ui: &mut Ui, headless: bool) -> Result<(), String> {
    if let Some(path) = &args.adsb_fixture {
        let text = std::fs::read_to_string(path).map_err(|e| format!("--adsb-fixture {path}: {e}"))?;
        let frames = adsb::feed::parse_recording(&text).map_err(|e| format!("--adsb-fixture {path}: {e}"))?;
        eprintln!("live traffic: replaying {} recorded frames ({} aircraft)", frames.len(), frames[0].reports.len());
        ui.st.adsb.replay(frames);
        return Ok(());
    }
    ui.st.adsb.force = match args.adsb {
        Some(on) => Some(on),
        None if headless || (args.live_given && !args.live) => Some(false),
        None => None,
    };
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn headless(args: Args) -> Result<(), String> {
    let mut args = args;
    let path = args.shot.clone().unwrap();
    // Airports (OurAirports): runways, --airport and the menus.
    let mut loader = airports::Loader::start();
    let db = loader.wait(Duration::from_secs(120));
    if db.is_none() {
        eprintln!("airports: database unavailable — tile runways only");
    }
    resolve_airport(&mut args, db.as_deref());
    // Navaids (nav/): VOR / DME / NDB around the flight.
    if nav::db::wait(Duration::from_secs(60)).is_none() {
        eprintln!("navaids: database unavailable — ILS only");
    }
    // Airport frequencies for ATC (atc/).
    let _ = atc::freqs::wait(Duration::from_secs(60));
    let gpu = Gpu::new(None, Gpu::instance())?;
    eprintln!("GPU: {}", gpu.adapter_name());
    let format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let mut renderer = Renderer::new(&gpu.device, &gpu.queue, format);
    // Headless captures use default settings (reproducible), plus the flags.
    let mut ui = Ui::new(&gpu.device, format, ui::settings::Settings::default());
    // Layout checks at a larger interface scale: SKYWARD_UI_SCALE=1.3.
    if let Some(v) = std::env::var("SKYWARD_UI_SCALE").ok().and_then(|v| v.parse::<f32>().ok()) {
        ui.st.settings.ui_scale = v.clamp(ui::settings::UI_SCALE_MIN, ui::settings::UI_SCALE_MAX);
    }
    ui.hide_hud = args.no_hud;
    // (Captures neither read nor write the player's multiplayer favourites / recents.)
    ui.st.mp.prefs = net::prefs::Prefs { read_only: true, ..net::prefs::Prefs::from_json(&serde_json::Value::Null) };
    // Live traffic (adsb/): off in captures (reproducible) unless asked for, or a recording replayed.
    setup_adsb(&args, &mut ui, true)?;
    ui.st.db = db.clone();
    ui.st.plan = plan_from_args(&args, db.as_deref());
    if let Some(m) = args.hud_mode {
        ui.st.settings.hud = m;
    }
    if let Some(m) = args.map_mode {
        ui.st.map.mode = m;
    }
    {
        let c = &mut ui.st.settings.conditions;
        c.live = args.live;
        if let Some(h) = args.hours {
            c.hours = h;
        }
        if let Some(d) = args.day_of_year {
            c.day = d;
        }
        if let Some(w) = args.weather {
            c.preset = w.name().to_string();
        }
        if let Some(m) = &args.metar {
            c.metar = m.clone();
        }
    }
    ui.st.screen = match args.menu_screen {
        Some(s) => s,
        None if args.menu => Screen::Aircraft,
        None => Screen::Flying,
    };
    // Settings screenshots: SKYWARD_SETTINGS_TAB="3:0:pitch" (tab, Controls page, open axis);
    // virtual controllers from SKYWARD_FAKE_INPUT (input/).
    ui.st.input.add_fakes_from_env();
    ui.st.input.update(&mut ui.st.settings.controls, 0.0);
    if let Some(t) = args.panel {
        ui.st.fail.open = true;
        ui.st.fail.tab = t;
    }
    if let Some(t) = args.radio_panel {
        ui.st.radio.open = true;
        ui.st.radio.tab = t;
    }
    ui.st.comms.open = args.comms_panel;
    // (Captures neither enumerate audio devices nor walk the cache.)
    ui.st.settings_ui.headless = true;
    ui.st.gpu_name = gpu.adapter_name();
    if let Some(p) = args.palette {
        ui.st.settings.palette = p;
    }
    ui.st.settings.high_contrast |= args.high_contrast;
    if let Some(c) = args.settings_cat {
        ui.st.settings_ui.open_at(c, args.settings_search.as_deref().unwrap_or(""));
    } else if let Some(q) = &args.settings_search {
        ui.st.settings_ui.query = q.clone();
    }
    if args.settings_from_pause {
        ui.st.back_to = Screen::Pause;
    }
    if let Ok(t) = std::env::var("SKYWARD_SETTINGS_TAB") {
        let mut it = t.split(':');
        // (Old tab numbers: 0 Display → Video, 1 Units → Display & HUD, 2 Sound, 3 Controls.)
        use ui::settings_schema::Cat;
        let cat = [Cat::Video, Cat::Display, Cat::Sound, Cat::Controls].get(it.next().and_then(|x| x.parse::<usize>().ok()).unwrap_or(0)).copied().unwrap_or(Cat::Video);
        ui.st.settings_ui.open_at(cat, "");
        ui.st.controls_page = it.next().and_then(|x| x.parse().ok()).unwrap_or(0);
        ui.st.controls_axis = it.next().and_then(input::AxisAction::from_id).map(|a| (a, 0));
    }
    // Missions: the selected card / briefing, and the HUD of the one being flown.
    if let Some(id) = &args.mission {
        ui.st.missions.sel = missions::defs::all().iter().position(|m| &m.id == id).unwrap_or(0);
        ui.st.missions.active = Some(id.clone());
        ui.st.missions.bot = args.mission_bot;
        // (The weather panel makes room for the mission panel: I shows it.)
        ui.st.show_wx = false;
        // (Captures never touch the player's saved records.)
        ui.st.missions.records = Some(Default::default());
    }
    // Aircraft-screen screenshots: SKYWARD_AIRCRAFT_GROUP=light|warbird|jet|airliner|military|rotor.
    ui.st.aircraft_group = std::env::var("SKYWARD_AIRCRAFT_GROUP").ok().and_then(|g| ui::menus::Group::parse(&g));
    let custom = args.start_at.clone().map(|spec| startpos::CustomStart { airport: ui.st.plan.airport.clone(), spec });

    // Multiplayer (net/): join first; the handshake runs while the scenery loads.
    if let Some(addr) = &args.mp_connect {
        let def = &ui.st.defs[args.aircraft.min(ui.st.defs.len() - 1)];
        let name = args.mp_name.clone().unwrap_or_else(|| ui.st.mp.prefs.name.clone());
        ui.st.mp.join(addr, &args.mp_password, &name, def.id);
    }
    // Load the start area + spawn.
    let mut s = Session::new(&args, db.clone(), custom);
    // Captures simulate the traffic in this thread (reproducible); the
    // real-time / soak runs keep the worker thread (timings).
    if !args.realtime && !args.soak {
        s.world.traffic.set_inline();
    }
    let t0 = Instant::now();
    let mut last_msg = String::new();
    while s.sim.is_none() {
        ui.st.mp.poll();
        sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
        if let Some(m) = s.advance_loading() {
            if m != last_msg && t0.elapsed() > Duration::from_millis(500) {
                last_msg = m.clone();
            }
        }
        if t0.elapsed() > Duration::from_secs(180) {
            return Err(format!("timed out loading map ({last_msg})"));
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    // Air starts away from the spawn area (or a re-spawn at another airfield) wait for their scenery.
    while s.check_catch_up(None).is_some() {
        ui.st.mp.poll();
        s.follow();
        sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
        if t0.elapsed() > Duration::from_secs(240) {
            return Err("timed out loading the start position".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    eprintln!("loaded in {:.2}s ({} tiles: {} network, {} cache)", t0.elapsed().as_secs_f32(), s.world.tile_count(), s.world.stats.from_network, s.world.stats.from_cache);

    // Live weather: wait for the first fetch, save what was used next to the shot.
    if s.world.live_weather || s.world.live_time {
        s.world.live.wait(Duration::from_secs(12));
        sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
    }
    weather::save_report(&path, &s.world);
    // Live traffic asked for in a capture (`--adsb`): wait for the first answer (≤ 15 s).
    if args.adsb == Some(true) && args.adsb_fixture.is_none() {
        let tw = Instant::now();
        let lt_settings = ui.st.settings.live_traffic.clone();
        while tw.elapsed() < Duration::from_secs(15) && ui.st.adsb.fetched == 0 {
            if let Some(sim) = s.sim.as_ref() {
                ui.st.adsb.update(sim, &s.world, 0.05, &lt_settings);
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        eprintln!("{} after {:.1} s", ui.st.adsb.status, tw.elapsed().as_secs_f32());
    }
    let (w, h) = (args.width, args.height);
    let tex = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("shot"),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = tex.create_view(&Default::default());
    let raw = || egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(w as f32, h as f32))),
        ..Default::default()
    };
    let mut ov = match (args.eye, args.look) {
        (Some(e), Some(l)) => Some((e, l)),
        _ => None,
    };
    // Geographic camera (heights above the ground are resolved once the
    // scenery under it has loaded, below).
    if let (Some(e), Some(l)) = (args.eye_ll, args.look_ll) {
        let geo = |g: (f64, f64, f32)| {
            let p = s.world.proj.to_world(g.0, g.1);
            vec3(p.x, g.2, p.y)
        };
        ov = Some((geo(e), geo(l)));
    }
    {
        let sim = s.sim.as_mut().unwrap();
        if !args.watch || std::env::args().any(|a| a == "--cam") {
            sim.cam = args.cam;
        }
        // Cockpit head-look for captures: SKYWARD_LOOK="yaw_deg,pitch_deg" (right / up positive).
        if let Some(v) = std::env::var("SKYWARD_LOOK").ok().map(|s| s.split(',').filter_map(|x| x.trim().parse::<f32>().ok()).collect::<Vec<_>>()).filter(|v| v.len() == 2) {
            sim.pilot.free = glam::vec2(v[0].to_radians(), v[1].to_radians());
            sim.pilot.look = sim.pilot.free;
        }
        if let Some(z) = args.map_zoom {
            sim.map_zoom = z.min(sim::MAP_RANGES.len() - 1);
        }
        if let Some(t) = args.throttle {
            sim.ctl.throttle = t;
        }
        if let Some(g) = args.gear {
            sim.ctl.gear_down = g;
        }
        if let Some(f) = args.flaps {
            sim.set_flap_detent(f);
        }
        if let Some(sp) = args.spoilers {
            sim.set_spoilers(sp);
        }
        if args.reverse {
            sim.ctl.reverse = true;
        }
        if args.fail > 0 {
            sim.ctl.failed_engines = args.fail;
        }
        apply_systems_args(&args, sim);
        apply_route(&args, sim, &s.world);
        if args.ap && !sim.ac.on_ground {
            sim.ap.engage_hold_current(&sim.ac);
        }
        if args.realtime && sim.bot.is_none() {
            let alt = sim.ac.pos.y.max(300.0);
            sim.level_hold = Some((sim.ac.heading(), alt));
        }
        // Time warp: soak / realtime tests, and fast-forwarded shots (long flights).
        sim.warp = args.warp;
    }
    let mut input = Input::default();
    input.pressed.extend(args.press.iter().copied());
    if args.mp_connect.is_some() {
        mp_headless_start(&args, &mut s, &mut ui)?;
    }

    if args.soak {
        return soak_flight(&args, &gpu, &mut renderer, &mut ui, &mut s, &tex, &view, w, h, &raw, &mut input);
    } else if args.realtime {
        realtime_flight(&args, &gpu, &mut renderer, &mut ui, &mut s, &view, w, h, &raw, &mut input)?;
    } else {
        // Fast-forward the simulation; streaming keeps up in the background.
        // (`--audio-script` cues work here too: ATC requests for comms shots.)
        let mut cues = audio::render::parse_script(&args.audio_script);
        let steps = (args.seconds * 60.0) as usize;
        // Multiplayer: everyone shares one timeline, so fly in real time (paced to the wall clock).
        let mp = ui.st.mp.connected();
        let mut chat = mp_chat_cues(&args.mp_chat);
        let paced = Instant::now();
        for i in 0..steps.max(1) {
            if mp {
                let due = paced + Duration::from_secs_f64(i as f64 / 60.0);
                let now = Instant::now();
                if due > now {
                    std::thread::sleep(due - now);
                }
                ui.st.mp.poll();
                let t = i as f32 / 60.0;
                while chat.first().is_some_and(|c| c.0 <= t) {
                    let (_, text) = chat.remove(0);
                    ui.st.mp.say(&text);
                }
            }
            let sim = s.sim.as_mut().unwrap();
            for c in cues.due(i as f32 / 60.0, sim, &s.world) {
                audio::render::apply(&c, sim, &mut input, &s.world);
            }
            if let Some(t) = args.eject_at {
                let now = i as f32 / 60.0;
                if (now - t).abs() < 0.5 / 60.0 || (now - t - 0.2).abs() < 0.5 / 60.0 {
                    input.pressed.push(Key::Eject);
                }
            }
            sim.update(&s.world, &mut input, if steps == 0 { 0.0 } else { 1.0 / 60.0 });
            if let (Some(t), None) = (args.throttle, &sim.bot) {
                sim.ctl.throttle = t;
            }
            if mp {
                ui.st.mp.fly(sim, &mut s.world, false, 1.0 / 60.0);
            }
            if let Some(w) = &args.shot_when {
                if sim.combat_moment(w) {
                    eprintln!("shot-when {w}: at {:.1} s", i as f32 / 60.0);
                    s.follow();
                    break;
                }
            }
            if i % if mp { 10 } else { 30 } == 0 {
                s.follow();
                sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
            }
        }
    }

    // Missions: the outcome (and the debrief screen when asked for).
    if let Some(run) = s.sim.as_ref().and_then(|x| x.mission.as_ref()) {
        match &run.end {
            Some(d) => {
                eprintln!("mission {}: {} — {} ({:.0} points, {}, {:?}) after {:.0} s", run.m.id, d.headline, d.reason, d.score.total, d.score.grade, d.score.medal, d.time_s);
                let cp = s.sim.as_ref().and_then(|x| x.mission_checkpoint.as_ref()).map(|c| c.name.clone());
                ui.st.missions.last = Some((run.m.id.clone(), d.clone(), false, cp));
            }
            None => eprintln!("mission {}: running at {:.0} s", run.m.id, run.t),
        }
    }
    // Wait for the scenery around the final camera to finish streaming.
    s.follow();
    let focus = match ov {
        Some((e, _)) => vec2(e.x, e.z),
        None => s.world.focus(),
    };
    if ov.is_some() {
        s.world.set_focus(focus, Vec2::ZERO);
    }
    let tw = Instant::now();
    let mut last_step = Instant::now();
    loop {
        if ui.st.mp.connected() {
            let dt = last_step.elapsed().as_secs_f32().min(0.1);
            last_step = Instant::now();
            mp_headless_step(&mut s, &mut ui, &mut input, dt);
        }
        sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
        // (The floating origin may have moved: follow the world's focus.)
        let focus = if ov.is_some() { focus } else { s.world.focus() };
        let (ready, total) = s.world.progress(focus, s.world.effective_radius() - 300.0);
        if ready >= total && s.world.uploads.is_empty() && s.world.gpu_pending.is_empty() && s.world.far_ready() && s.world.airport_detail.idle() {
            break;
        }
        if tw.elapsed() > Duration::from_secs(120) {
            eprintln!("warning: rendering with {ready}/{total} tiles");
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    eprintln!("scenery around camera ready after {:.2}s more", tw.elapsed().as_secs_f32());
    if let (Some(e), Some(l), Some((eye, look))) = (args.eye_ll, args.look_ll, ov.as_mut()) {
        let gh = |p: Vec3| flight::Environment::ground_height(&s.world, vec2(p.x, p.z));
        eye.y = e.2 + gh(*eye);
        look.y = l.2 + gh(*look);
    }

    if args.stats {
        let ac = &s.sim.as_ref().unwrap().ac;
        eprintln!(
            "state: pos {:.1?} ias {:.1} kt alt {:.0} ft hdg {:.0} pitch {:.1} roll {:.1} ground {} crashed {:?}",
            ac.pos,
            ac.airspeed() * 1.9438,
            ac.pos.y * 3.2808,
            ac.heading().to_degrees(),
            ac.pitch().to_degrees(),
            ac.roll().to_degrees(),
            ac.on_ground,
            ac.crashed
        );
        // Spawn checks (QA smoke tests): height over the ground / any building, gear, flaps.
        let sim = s.sim.as_ref().unwrap();
        let xz = vec2(ac.pos.x, ac.pos.z);
        let gh = flight::Environment::ground_height(&s.world, xz);
        eprintln!(
            "state+: agl {:.1} m (wheels {:.1} m) building {:?} gear {:.2} (lever {}) flaps {:.2} (lever {:.2}) start {:?}",
            ac.pos.y - gh,
            ac.pos.y - sim.def().gear_height - gh,
            flight::Environment::obstacle_top(&s.world, xz).map(|t| t - gh),
            ac.gear_pos(),
            if sim.ctl.gear_down { "down" } else { "up" },
            ac.surfaces().flaps,
            sim.ctl.flaps,
            sim.start
        );
    }
    if args.rebase_check {
        rebase_check(&gpu, &mut renderer, &mut ui, &mut s, &tex, &view, w, h, &raw, &path, ov)?;
    }
    // Menu screens: thumbnails, the departure map's tiles and METAR.
    if matches!(ui.st.screen, Screen::Aircraft | Screen::Main | Screen::Missions | Screen::Briefing) {
        render_thumbs(&gpu, &mut renderer, &mut ui, &s, format);
    }
    // The server browser: let LAN discovery run (broadcast + mDNS + pings), then
    // select the first server so its details show.
    if ui.st.screen == Screen::Multiplayer {
        let tw = Instant::now();
        while tw.elapsed() < Duration::from_secs(7) {
            render_frame(&gpu, &mut renderer, &mut ui, Some(&mut s), &view, w, h, raw(), 1.0, 0.0, None, ov);
            let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
            if tw.elapsed() > Duration::from_millis(3500) && ui.st.browser.selected.is_none() {
                let first = ui.st.browser.visible(&ui.st.mp.prefs).first().map(|e| e.key.clone());
                ui.st.browser.selected = first;
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        eprintln!("server browser: {} servers listed", ui.st.browser.entries.len());
    }
    if matches!(ui.st.screen, Screen::Departure | Screen::Conditions | Screen::Loading) || (ui.st.screen == Screen::Main && s.sim.is_none()) {
        let tw = Instant::now();
        loop {
            render_frame(&gpu, &mut renderer, &mut ui, Some(&mut s), &view, w, h, raw(), 1.0, 0.0, None, ov);
            let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
            let metar_pending = matches!(ui.st.screen, Screen::Departure | Screen::Conditions) && ui.st.picker.metar.as_ref().is_none_or(|m| m.pending()) && tw.elapsed() < Duration::from_secs(10);
            if let Some(m) = ui.st.picker.metar.as_mut() {
                m.poll();
            }
            if (ui.st.picker.overview.busy() == 0 && !metar_pending && tw.elapsed() > Duration::from_secs(2)) || tw.elapsed() > Duration::from_secs(40) {
                break;
            }
            std::thread::sleep(Duration::from_millis(30));
        }
    }
    // Multiplayer captures: SKYWARD_MP_UI="players" | "wheel" | "chat:draft text" | "spectate:Name" | "tags-off".
    if let Ok(v) = std::env::var("SKYWARD_MP_UI") {
        for item in v.split(',') {
            let (k, arg) = item.split_once(':').unwrap_or((item, ""));
            let mp = &mut ui.st.mp;
            match k {
                "players" => mp.show_players = true,
                "wheel" => mp.wheel = Some(net::Wheel { sel: arg.parse().ok(), held: false }),
                "chat" => {
                    mp.chat_open = true;
                    mp.chat_draft = arg.to_string();
                }
                "tags-off" => mp.tags = false,
                "spectate" => {
                    if let Some(id) = mp.remotes.values().find(|r| r.name.eq_ignore_ascii_case(arg)).map(|r| r.id) {
                        mp.start_spectating(id);
                        // Let the pinned (hidden) aircraft pull the scenery and the origin over.
                        for _ in 0..20 {
                            mp_headless_step(&mut s, &mut ui, &mut Input::default(), 1.0 / 60.0);
                            sync_gpu(&mut s, &mut renderer, &mut ui, &gpu, None);
                        }
                    }
                }
                _ => {}
            }
        }
    }
    // Traffic: refill and settle from the resident tiles, so the picture
    // doesn't depend on the order the tiles streamed in.
    s.world.traffic.reset();
    // Two frames: the first lets egui load fonts + map textures.
    let t0 = Instant::now();
    let note_s = loading_text(&ui.st, "Loading scenery\n37 / 52 tiles");
    let note = (ui.st.screen == Screen::Loading).then_some(note_s.as_str());
    for _ in 0..3 {
        render_frame(&gpu, &mut renderer, &mut ui, Some(&mut s), &view, w, h, raw(), 1.0, 0.0, note, ov);
    }
    if s.world.traffic.level() != traffic::Level::Off {
        let t = s.world.traffic.stats();
        eprintln!("traffic: {} vehicles ({} drawn, {} active lanes), {:.2} ms", t.vehicles, t.drawn, t.active_lanes, t.sim_ms);
    }
    let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
    let t1 = Instant::now();
    let n = 10;
    for _ in 0..n {
        render_frame(&gpu, &mut renderer, &mut ui, Some(&mut s), &view, w, h, raw(), 1.0, 0.0, note, ov);
    }
    let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
    let rs = renderer.stats;
    eprintln!(
        "render: warmup {:.0} ms, steady {:.2} ms/frame at {w}x{h}; tiles {} (drawn {}), trees drawn {}, GPU {:.0} MB",
        (t1 - t0).as_secs_f32() * 1000.0,
        (t1.elapsed().as_secs_f32() * 1000.0) / n as f32,
        rs.tiles,
        rs.tiles_drawn,
        rs.tree_instances,
        rs.gpu_bytes as f64 / (1 << 20) as f64,
    );
    let st = &s.world.stats;
    eprintln!(
        "tiles: built {} (net {}, cache {}, failed {}), avg fetch {:.0} ms, avg build {:.0} ms (max {:.0}), {:.1} MB pbf, {} buildings, {} trees",
        st.tiles_built,
        st.from_network,
        st.from_cache,
        st.failures,
        st.fetch_ms / st.tiles_built.max(1) as f32,
        st.build_ms / st.tiles_built.max(1) as f32,
        st.max_build_ms,
        st.tile_bytes as f64 / 1e6,
        s.world.building_count(),
        s.world.tree_count()
    );
    let px = gfx::read_texture(&gpu, &tex, w, h);
    image::save_buffer(&path, &px, w, h, image::ColorType::Rgba8).map_err(|e| e.to_string())?;
    if ui.st.adsb.settings.enabled {
        let lt = &ui.st.adsb;
        eprintln!("live traffic: {} aircraft shown, update {:.3} ms, render {:.3} ms ({})", lt.shown(), lt.cpu_ms, lt.render_ms, lt.status);
        if std::env::var_os("SKYWARD_ADSB_DEBUG").is_some() {
            for line in lt.debug_lines(&s.world.proj) {
                eprintln!("  {line}");
            }
        }
    }
    eprintln!("wrote {path}");
    ui.st.mp.leave();
    Ok(())
}

/// "2:Hello,5.5:Follow me" → [(2.0, "Hello"), (5.5, "Follow me")].
fn mp_chat_cues(s: &str) -> Vec<(f32, String)> {
    let mut v: Vec<(f32, String)> = s.split(',').filter_map(|c| c.split_once(':')).filter_map(|(t, m)| Some((t.trim().parse().ok()?, m.trim().to_string()))).collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// Headless multiplayer: wait for the connection and (`--mp-wait N`) for N
/// players, sending our (held) state meanwhile, so test clients start together.
fn mp_headless_start(args: &Args, s: &mut Session, ui: &mut Ui) -> Result<(), String> {
    let t0 = Instant::now();
    while !ui.st.mp.connected() {
        ui.st.mp.poll();
        if let net::Status::Failed(e) | net::Status::Closed(e) = &ui.st.mp.status {
            return Err(format!("multiplayer: {e}"));
        }
        if t0.elapsed() > Duration::from_secs(20) {
            return Err("multiplayer: no connection after 20 s".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let want = args.mp_wait.max(1);
    let mut last = Instant::now();
    loop {
        ui.st.mp.poll();
        let dt = last.elapsed().as_secs_f32();
        last = Instant::now();
        if let Some(sim) = s.sim.as_mut() {
            ui.st.mp.fly(sim, &mut s.world, true, dt);
        }
        let ready = ui.st.mp.remotes.values().filter(|r| r.cur.is_some()).count() + 1;
        if ready >= want {
            break;
        }
        if t0.elapsed() > Duration::from_secs(180) {
            return Err(format!("multiplayer: only {ready} of {want} players after 180 s"));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    eprintln!("multiplayer: {} players in, starting", ui.st.mp.remotes.len() + 1);
    Ok(())
}

/// Headless multiplayer: one real-time step of the sim and the session.
fn mp_headless_step(s: &mut Session, ui: &mut Ui, input: &mut Input, dt: f32) {
    ui.st.mp.poll();
    if let Some(sim) = s.sim.as_mut() {
        sim.update(&s.world, input, dt);
        ui.st.mp.fly(sim, &mut s.world, false, dt);
    }
    s.follow();
}

#[cfg(not(target_arch = "wasm32"))]
/// Floating-origin invariance test: render the frozen scene, switch to a frame
/// whose origin is 20 km away (exact map + rigid shift, like a real rebase),
/// render again and compare with the frame-to-frame baseline.
#[allow(clippy::too_many_arguments)]
fn rebase_check(
    gpu: &Gpu,
    renderer: &mut Renderer,
    ui: &mut Ui,
    s: &mut Session,
    tex: &wgpu::Texture,
    view: &wgpu::TextureView,
    w: u32,
    h: u32,
    raw: &dyn Fn() -> egui::RawInput,
    path: &str,
    ov: Option<(Vec3, Vec3)>,
) -> Result<(), String> {
    let stem = path.trim_end_matches(".png").to_string();
    let grab = |s: &mut Session, renderer: &mut Renderer, ui: &mut Ui, n: usize, ov: Option<(Vec3, Vec3)>| {
        for _ in 0..n {
            render_frame(gpu, renderer, ui, Some(&mut *s), view, w, h, raw(), 1.0, 0.0, None, ov);
        }
        let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
        gfx::read_texture(gpu, tex, w, h)
    };
    let diff = |a: &[u8], b: &[u8]| {
        let (mut sum, mut big) = (0u64, 0u64);
        for (pa, pb) in a.chunks(4).zip(b.chunks(4)) {
            let d = (0..3).map(|k| (pa[k] as i32 - pb[k] as i32).unsigned_abs()).max().unwrap();
            sum += d as u64;
            if d > 12 {
                big += 1;
            }
        }
        let n = (a.len() / 4) as f64;
        (sum as f64 / n, 100.0 * big as f64 / n)
    };
    let mut ov = ov;
    let mut a = grab(s, renderer, ui, 12, ov);
    // Two shifts: 20 km east (pure translation: the frames share a latitude)
    // and the worst case of a real rebase, REBASE_NS m north (also rescales x).
    let offs = if std::env::var_os("REBASE_CHECK_ZERO").is_some() { [Vec2::ZERO, Vec2::ZERO] } else { [vec2(20_000.0, 0.0), vec2(0.0, -world::REBASE_NS)] };
    for (k, off) in offs.into_iter().enumerate() {
        let a2 = grab(s, renderer, ui, 1, ov);
        let (eye0, _) = s.sim.as_ref().unwrap().pose();
        let p = {
            let ac = s.sim.as_ref().unwrap().ac.pos;
            vec2(ac.x, ac.z)
        };
        let (lat, lon) = s.world.proj.to_latlon(p + off);
        let t0 = Instant::now();
        let r = s.world.rebase_to(osm::Proj::new(lat, lon), p);
        s.sim.as_mut().unwrap().rebase(&r);
        renderer.rebase(r.delta);
        renderer.set_frame(s.world.proj, s.world.pattern_frame(), s.world.home);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        ov = ov.map(|(e, l)| (r.shift(e), r.shift(l)));
        // Keep the streaming state in step (no new tiles should be needed).
        sync_gpu(s, renderer, ui, gpu, None);
        let b = grab(s, renderer, ui, 1, ov);
        let b12 = grab(s, renderer, ui, 11, ov);
        let (eye1, _) = s.sim.as_ref().unwrap().pose();
        let (base_mean, base_big) = diff(&a, &a2);
        let (d1_mean, d1_big) = diff(&a2, &b);
        let (d12_mean, d12_big) = diff(&a, &b12);
        eprintln!(
            "rebase check {}: shift ({:.1}, {:.1}) m, x scale {:+.4}% in {:.2} ms; aircraft pose error {:.4} m",
            k + 1,
            r.delta.x,
            r.delta.z,
            (r.map.sx - 1.0) * 100.0,
            ms,
            (eye1 - r.shift(eye0)).length()
        );
        eprintln!(
            "  image change (mean abs / % px > 12 levels): baseline next frame {base_mean:.3} / {base_big:.3}%; first frame after the shift {d1_mean:.3} / {d1_big:.3}%; 12 frames on {d12_mean:.3} / {d12_big:.3}%"
        );
        if k == 0 {
            image::save_buffer(format!("{stem}_before.png"), &a, w, h, image::ColorType::Rgba8).map_err(|e| e.to_string())?;
            image::save_buffer(format!("{stem}_after.png"), &b, w, h, image::ColorType::Rgba8).map_err(|e| e.to_string())?;
        }
        a = b12;
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
/// Real-time streaming test: fly straight and level for `--seconds`, rendering
/// every frame offscreen, and report frame times, upload costs and coverage.
#[allow(clippy::too_many_arguments)]
fn realtime_flight(
    args: &Args,
    gpu: &Gpu,
    renderer: &mut Renderer,
    ui: &mut Ui,
    s: &mut Session,
    view: &wgpu::TextureView,
    w: u32,
    h: u32,
    raw: &dyn Fn() -> egui::RawInput,
    input: &mut Input,
) -> Result<(), String> {
    let start = Instant::now();
    let p0 = s.sim.as_ref().unwrap().ac.pos;
    let geo0 = s.world.proj.to_latlon(vec2(p0.x, p0.z));
    // Origin shifts: (count, worst eye-step deviation from the previous frame's step (m), worst frame ms).
    let (mut shifts, mut shift_dev, mut shift_ms) = (0u32, 0.0f32, 0.0f32);
    let mut last_step = 0.0f32;
    // Aircraft shake: screen position of the nose (px) through the f32 world
    // path the GPU used before (view_proj · world) and the eye-relative path,
    // and its attitude (deg); judged by the frame-to-frame second difference.
    let mut shake: Vec<(Vec2, Vec2, Vec3)> = Vec::new();
    let mut last = Instant::now();
    let mut next_report = 1.0;
    let (mut frames, mut sum_ms, mut max_ms, mut max_sync_ms, mut max_up) = (0u32, 0.0f32, 0.0f32, 0.0f32, 0u64);
    let (mut all_frames, mut all_sum, mut all_max, mut hitches) = (0u32, 0.0f32, 0.0f32, 0u32);
    let mut worst_gap = 0usize;
    let mut gap_frames = 0u32;
    let mut cam_prev: Option<(Vec3, Vec3)> = None;
    let (mut cam_steps, mut cam_yaw, mut cam_pitch) = (Vec::<f32>::new(), Vec::<f32>::new(), Vec::<f32>::new());
    while start.elapsed().as_secs_f32() < args.seconds {
        let now = Instant::now();
        let dt = args.frame_dt.unwrap_or((now - last).as_secs_f32());
        last = now;
        let tf = Instant::now();
        {
            let sim = s.sim.as_mut().unwrap();
            sim.update(&s.world, input, dt);
            if let (Some(t), None) = (args.throttle, &sim.bot) {
                sim.ctl.throttle = t;
            }
        }
        s.follow();
        let ts = Instant::now();
        let up = sync_gpu(s, renderer, ui, gpu, Some(UPLOAD_BUDGET));
        let sync_ms = ts.elapsed().as_secs_f32() * 1000.0;
        render_frame(gpu, renderer, ui, Some(&mut *s), view, w, h, raw(), 1.0, 0.0, None, None);
        let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
        // An origin shift this frame: express the previous camera in the new frame.
        let shifted = s.last_rebase.take();
        if let (Some(r), Some((pe, _))) = (&shifted, &mut cam_prev) {
            *pe = r.shift(*pe);
        }
        // Camera smoothness: per-frame eye step and view-direction yaw/pitch change.
        if let Some(v) = s.sim.as_ref().and_then(|x| x.last_view.as_ref()) {
            let fwd = -glam::Vec3::new(v.view.x_axis.z, v.view.y_axis.z, v.view.z_axis.z);
            if let Some((pe, pf)) = cam_prev {
                let step: f32 = v.eye.distance(pe);
                if shifted.is_some() {
                    shifts += 1;
                    shift_dev = shift_dev.max((step - last_step).abs());
                    shift_ms = shift_ms.max(tf.elapsed().as_secs_f32() * 1000.0);
                }
                last_step = step;
                let yaw = (fwd.x.atan2(-fwd.z) - pf.x.atan2(-pf.z)).abs().to_degrees();
                let pit = (fwd.y.clamp(-1.0, 1.0).asin() - pf.y.clamp(-1.0, 1.0).asin()).abs().to_degrees();
                if start.elapsed().as_secs_f32() > 2.0 {
                    cam_steps.push(step);
                    cam_yaw.push(yaw.min(360.0 - yaw));
                    cam_pitch.push(pit);
                }
            }
            cam_prev = Some((v.eye, fwd));
        }
        if let Some(sim) = s.sim.as_ref()
            && let (Some(v), true) = (sim.last_view.as_ref(), start.elapsed().as_secs_f32() > 2.0)
        {
            let (rp, rr) = sim.render_pose();
            let nose = rp + rr * Vec3::Z * sim.size * 0.5;
            let proj = glam::camera::rh::proj::directx::perspective_infinite_reverse(v.fov_y, w as f32 / h as f32, 0.25);
            let px = |c: glam::Vec4| Vec2::new((c.x / c.w * 0.5 + 0.5) * w as f32, (0.5 - c.y / c.w * 0.5) * h as f32);
            let world = px((proj * v.view) * nose.extend(1.0));
            let rot = glam::Mat4::from_cols(v.view.x_axis, v.view.y_axis, v.view.z_axis, glam::Vec4::W);
            let rel = px((proj * rot) * (nose - v.eye).extend(1.0));
            let (yaw, pitch, roll) = rr.to_euler(glam::EulerRot::YXZ);
            if shifted.is_some() {
                shake.clear(); // a different frame: restart the differences
            }
            shake.push((world, rel, Vec3::new(yaw, pitch, roll) * (180.0 / std::f32::consts::PI)));
        }
        let ms = tf.elapsed().as_secs_f32() * 1000.0;
        renderer.collect_timings(&gpu.device);
        frames += 1;
        sum_ms += ms;
        max_ms = max_ms.max(ms);
        max_sync_ms = max_sync_ms.max(sync_ms);
        max_up = max_up.max(up);
        if start.elapsed().as_secs_f32() > 2.0 {
            all_frames += 1;
            all_sum += ms;
            all_max = all_max.max(ms);
            if ms > 33.0 {
                hitches += 1;
            }
        }
        // Coverage: tiles within 3 km of the aircraft that are not on the GPU yet.
        let ac = s.sim.as_ref().unwrap().ac.pos;
        let (ready, total) = s.world.progress(vec2(ac.x, ac.z), 3000.0);
        if ready < total {
            gap_frames += 1;
            worst_gap = worst_gap.max(total - ready);
        }
        let t = start.elapsed().as_secs_f32();
        if t >= next_report {
            next_report += 5.0;
            let (r7, t7) = s.world.progress(vec2(ac.x, ac.z), s.world.effective_radius() - 500.0);
            let sim = s.sim.as_ref().unwrap();
            eprintln!(
                "t {:5.1}s dist {:6.0} m (origin #{}) alt {:4.0} ias {:3.0} kt | tiles {:3} gpu {:3} queued {:3} up-pending {:2} | near {}/{} far {}/{} | frame avg {:5.2} max {:5.2} ms, sync max {:5.2} ms, up max {:.1} MB | {:.0} MB GPU, {:.1} MB dl{}",
                t,
                s.world.distance_to_geo(vec2(ac.x, ac.z), geo0.0, geo0.1),
                s.world.rebases,
                ac.y,
                sim.ac.airspeed() * 1.9438,
                s.world.tile_count(),
                renderer.stats.tiles,
                s.world.queued(),
                s.world.uploads.len() + s.world.gpu_pending.len(),
                ready,
                total,
                r7,
                t7,
                sum_ms / frames.max(1) as f32,
                max_ms,
                max_sync_ms,
                max_up as f64 / (1 << 20) as f64,
                renderer.gpu_bytes() as f64 / (1 << 20) as f64,
                s.world.downloaded_bytes() as f64 / 1e6,
                sim.ac.crashed.as_ref().map(|c| format!(" CRASHED {c}")).unwrap_or_default(),
            );
            if s.world.traffic.level() != traffic::Level::Off {
                let t = s.world.traffic.stats();
                eprintln!("  traffic: {} vehicles, {} drawn, main thread {:.3} + {:.3} ms (sim / upload), sim worker {:.2} ms", t.vehicles, t.drawn, s.world.traffic.main_ms, renderer.road_traffic.prepare_ms, t.sim_ms);
            }
            frames = 0;
            sum_ms = 0.0;
            max_ms = 0.0;
            max_sync_ms = 0.0;
            max_up = 0;
        }
    }
    if args.then_reset {
        // Exercise the restart path: R teleports back to the start; the sim
        // must hold until the scenery there has streamed back in.
        let before = s.sim.as_ref().map(|x| x.ac.pos);
        input.pressed.push(Key::Reset);
        s.sim.as_mut().unwrap().update(&s.world, input, 0.0);
        s.sim.as_mut().unwrap().level_hold = None;
        s.follow();
        let t = Instant::now();
        let mut first = before;
        let mut frames = 0;
        while s.check_catch_up(first.take()).is_some() {
            sync_gpu(s, renderer, ui, gpu, None);
            render_frame(gpu, renderer, ui, Some(&mut *s), view, w, h, raw(), 1.0, 0.0, Some("Loading scenery"), None);
            frames += 1;
        }
        eprintln!("reset: scenery at the start ready after {:.2}s ({frames} catch-up frames)", t.elapsed().as_secs_f32());
    }
    let stat = |v: &[f32]| {
        let n = v.len().max(1) as f32;
        let m = v.iter().sum::<f32>() / n;
        let sd = (v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / n).sqrt();
        // Frame-to-frame change of the step: the judder signal.
        let jit = v.windows(2).map(|w| (w[1] - w[0]).abs()).sum::<f32>() / n;
        (m, sd, jit, v.iter().cloned().fold(0.0, f32::max))
    };
    let (sm, ssd, sj, smax) = stat(&cam_steps);
    let (ym, _, yj, ymax) = stat(&cam_yaw);
    let (pm, _, pj, pmax) = stat(&cam_pitch);
    eprintln!(
        "camera smoothness ({} frames{}): eye step {:.3} m ± {:.3} (frame-to-frame jitter {:.3} m = {:.1}%, max {:.3}); yaw {:.4}°/frame (jitter {:.4}°, max {:.3}); pitch {:.4}°/frame (jitter {:.4}°, max {:.3})",
        cam_steps.len(),
        args.frame_dt.map(|d| format!(" at {:.0} Hz", 1.0 / d)).unwrap_or_default(),
        sm,
        ssd,
        sj,
        100.0 * sj / sm.max(1e-6),
        smax,
        ym,
        yj,
        ymax,
        pm,
        pj,
        pmax
    );
    if shake.len() > 3 {
        let n = (shake.len() - 2) as f32;
        let d2 = |f: &dyn Fn(&(Vec2, Vec2, Vec3)) -> Vec3| {
            let (mut sum, mut max) = (0.0f32, 0.0f32);
            for k in 2..shake.len() {
                let e = (f(&shake[k]) - 2.0 * f(&shake[k - 1]) + f(&shake[k - 2])).abs();
                let m = e.max_element();
                sum += m;
                max = max.max(m);
            }
            (sum / n, max)
        };
        let (wm, wx) = d2(&|x| x.0.extend(0.0));
        let (rm, rx) = d2(&|x| x.1.extend(0.0));
        let (am, ax) = d2(&|x| x.2);
        // Attitude rate noise: per-frame rate minus its ~0.25 s running mean (deg/frame), rms.
        let rates: Vec<Vec3> = shake.windows(2).map(|w| {
            let mut d = w[1].2 - w[0].2;
            d.x = (d.x + 180.0).rem_euclid(360.0) - 180.0;
            d
        }).collect();
        let k = 18usize.min(rates.len());
        let mut acc = 0.0f32;
        let mut cnt = 0;
        for i in k..rates.len() {
            let mean = rates[i - k..i].iter().copied().sum::<Vec3>() / k as f32;
            acc += (rates[i] - mean).length_squared();
            cnt += 1;
        }
        let rate_noise = (acc / cnt.max(1) as f32).sqrt();
        eprintln!(
            "aircraft shake (2nd difference per frame): nose on screen via world f32 {wm:.3} px (max {wx:.2}), eye-relative {rm:.3} px (max {rx:.2}); attitude {am:.4}° (max {ax:.3}°); attitude-rate noise {rate_noise:.4}°/frame rms"
        );
    }
    let ac = s.sim.as_ref().unwrap().ac.pos;
    let (lat, lon) = s.world.proj.to_latlon(vec2(ac.x, ac.z));
    eprintln!(
        "origin shifts: {shifts} (worst eye-step change across a shift {shift_dev:.4} m, worst shift frame {shift_ms:.2} ms); now at {lat:.4}, {lon:.4}, {:.1} km from the start",
        s.world.distance_to_geo(vec2(ac.x, ac.z), geo0.0, geo0.1) / 1000.0
    );
    eprintln!(
        "realtime summary: {} frames, avg {:.2} ms, max {:.2} ms, {} frames > 33 ms; frames with gaps within 3 km: {} (worst {} tiles)",
        all_frames,
        all_sum / all_frames.max(1) as f32,
        all_max,
        hitches,
        gap_frames,
        worst_gap
    );
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
/// Render the current state (two frames, so egui has its fonts) and write a PNG.
#[allow(clippy::too_many_arguments)]
fn save_png(
    gpu: &Gpu,
    renderer: &mut Renderer,
    ui: &mut Ui,
    s: &mut Session,
    tex: &wgpu::Texture,
    view: &wgpu::TextureView,
    w: u32,
    h: u32,
    raw: &dyn Fn() -> egui::RawInput,
    path: &str,
) -> Result<(), String> {
    for _ in 0..2 {
        render_frame(gpu, renderer, ui, Some(&mut *s), view, w, h, raw(), 1.0, 0.0, None, None);
    }
    let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
    let px = gfx::read_texture(gpu, tex, w, h);
    image::save_buffer(path, &px, w, h, image::ColorType::Rgba8).map_err(|e| e.to_string())?;
    eprintln!("wrote {path}");
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
/// Bot soak test: fly the auto-flight bot for `--seconds` of simulated time as
/// fast as the tile streaming allows (never ahead of the scenery within 2.5 km
/// of the aircraft), log every minute and write PNGs every `--shot-every` s.
#[allow(clippy::too_many_arguments)]
fn soak_flight(
    args: &Args,
    gpu: &Gpu,
    renderer: &mut Renderer,
    ui: &mut Ui,
    s: &mut Session,
    tex: &wgpu::Texture,
    view: &wgpu::TextureView,
    w: u32,
    h: u32,
    raw: &dyn Fn() -> egui::RawInput,
    input: &mut Input,
) -> Result<(), String> {
    let path = args.shot.clone().unwrap();
    let stem = path.trim_end_matches(".png").to_string();
    let wall = Instant::now();
    {
        let sim = s.sim.as_mut().unwrap();
        if sim.bot.is_none() {
            sim.enable_bot(&s.world);
        }
        eprintln!(
            "soak: {} from {:?}, weather {}, {:.0} s sim time at warp {}",
            sim.def().name,
            sim.start,
            s.world.clouds.weather().name(),
            args.seconds,
            sim.warp
        );
    }
    let frame_dt = 1.0 / 30.0;
    let mut next_log = 0.0;
    // SKYWARD_SOAK_LOG=S: log every S seconds (default a minute).
    let log_every = std::env::var("SKYWARD_SOAK_LOG").ok().and_then(|v| v.parse::<f32>().ok()).filter(|v| *v > 0.0).unwrap_or(60.0);
    let mut next_shot = if args.shot_every > 0.0 { args.shot_every } else { f32::MAX };
    let mut seen = 0;
    let mut waited = 0.0f32;
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    let mut crashed = None;
    let start_pos = s.sim.as_ref().unwrap().ac.pos;
    let start_geo = s.world.proj.to_latlon(vec2(start_pos.x, start_pos.z));
    loop {
        let sim = s.sim.as_mut().unwrap();
        let t = sim.time;
        if t >= args.seconds {
            break;
        }
        sim.update(&s.world, input, frame_dt);
        let ac = &sim.ac;
        if !ac.on_ground {
            lo = lo.min(ac.pos.y);
            hi = hi.max(ac.pos.y);
        }
        if let Some(b) = &sim.bot {
            for (te, e) in &b.events[seen..] {
                eprintln!("  [{:6.0} s] {e}", te);
            }
            seen = b.events.len();
        }
        if let Some(c) = &ac.crashed {
            crashed = Some(c.clone());
            eprintln!("  [{:6.0} s] CRASHED: {c}", sim.time);
            break;
        }
        let p = vec2(ac.pos.x, ac.pos.z);
        // BOT_TRACE=Flare,Rollout prints every frame in those phases.
        if let (Ok(tr), Some(b)) = (std::env::var("BOT_TRACE"), &sim.bot) {
            let ph = b.phase();
            if tr.split(',').any(|x| x == ph || x == "all") {
                eprintln!(
                    "    {:7.2} {:<11} h {:6.1} ias {:5.1} vs {:+5.2} pitch {:+5.1} roll {:+5.1} hdg {:5.1} el {:+.2} ai {:+.2} ru {:+.2} thr {:.2} fl {:.2} br {:.2} gnd {}",
                    t,
                    ph,
                    ac.pos.y,
                    ac.airspeed() / flight::KT,
                    ac.vertical_speed(),
                    ac.pitch().to_degrees(),
                    ac.roll().to_degrees(),
                    ac.heading().to_degrees(),
                    sim.ctl.elevator,
                    sim.ctl.aileron,
                    sim.ctl.rudder,
                    sim.ctl.throttle,
                    sim.ctl.flaps,
                    sim.ctl.brakes,
                    ac.on_ground
                );
            }
        }
        if t >= next_log {
            next_log += log_every;
            let b = sim.bot.as_ref();
            eprintln!(
                "t {:5.0}s  alt {:5.0} m  agl {:5.0} m  hdg {:3.0}  ias {:4.0} kt  vs {:+5.1} m/s  dist {:5.1} km  {:<12} | {}",
                t,
                ac.pos.y,
                ac.pos.y - flight::Environment::ground_height(&s.world, p),
                ac.heading().to_degrees().rem_euclid(360.0),
                ac.airspeed() / flight::KT,
                ac.vertical_speed(),
                s.world.distance_to_geo(vec2(ac.pos.x, ac.pos.z), start_geo.0, start_geo.1) / 1000.0,
                b.map(|b| b.phase()).unwrap_or_default(),
                b.map(|b| b.status.clone()).unwrap_or_default()
            );
        }
        s.follow();
        sync_gpu(s, renderer, ui, gpu, None);
        // Nothing is rendered between snapshots: submit so the upload staging memory is recycled.
        gpu.queue.submit(std::iter::empty());
        let _ = gpu.device.poll(wgpu::PollType::Poll);
        // Don't outrun the scenery (buildings are obstacles; runways must exist).
        let tw = Instant::now();
        loop {
            // (Recomputed: sync_gpu may just have moved the floating origin.)
            let p = s.sim.as_ref().map(|x| vec2(x.ac.pos.x, x.ac.pos.z)).unwrap_or(p);
            let (r, tot) = s.world.progress(p, 2500.0);
            if r >= tot && s.world.uploads.is_empty() {
                break;
            }
            if tw.elapsed() > Duration::from_secs(60) {
                eprintln!("warning: scenery not ready after 60 s ({r}/{tot}), continuing: {:?}", s.world.missing(p, 2500.0));
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
            sync_gpu(s, renderer, ui, gpu, None);
        }
        waited += tw.elapsed().as_secs_f32();
        if t >= next_shot {
            next_shot += args.shot_every;
            save_png(gpu, renderer, ui, s, tex, view, w, h, raw, &format!("{stem}_t{:05.0}.png", t))?;
        }
    }
    let sim = s.sim.as_ref().unwrap();
    let ac = &sim.ac;
    eprintln!("---- soak summary: {} ({:.0} s sim in {:.0} s wall, {:.0} s waiting for scenery)", sim.def().name, sim.time, wall.elapsed().as_secs_f32(), waited);
    if let Some(b) = &sim.bot {
        let st = &b.stats;
        eprintln!(
            "airborne {:.0} s ({:.1} min) | alt range {:.0}..{:.0} m | max {:.0} m | take-offs {} landings {} go-arounds {} cycles {} | worst touchdown {:.2} m/s",
            st.airborne_time,
            st.airborne_time / 60.0,
            if lo < f32::MAX { lo } else { 0.0 },
            if hi > f32::MIN { hi } else { 0.0 },
            st.max_alt,
            st.takeoffs,
            st.landings,
            st.go_arounds,
            st.cycles,
            st.worst_touchdown
        );
        if st.thermals > 0 {
            eprintln!(
                "thermals {} | {:.0} s circling, {:+.0} m gained, avg climb {:+.2} m/s",
                st.thermals,
                st.thermal_time,
                st.thermal_gain,
                st.thermal_gain / st.thermal_time.max(1.0)
            );
        }
        eprintln!("final: {} | {}", b.phase(), b.status);
    }
    eprintln!(
        "end state: alt {:.0} m, ias {:.0} kt, on ground {}, crashed {:?}",
        ac.pos.y,
        ac.airspeed() / flight::KT,
        ac.on_ground,
        crashed
    );
    let mut s2 = s;
    save_png(gpu, renderer, ui, &mut s2, tex, view, w, h, raw, &path)?;
    if crashed.is_some() {
        return Err("crashed".into());
    }
    Ok(())
}


// ---------------------------------------------------------------- windowed

/// Frame pacing: the simulation advances by a smoothed frame time (median
/// of recent frames) instead of raw wall-clock deltas, which jitter by a
/// millisecond or more around the display's vblank and made the aircraft
/// appear to shake. A slow correction keeps sim time locked to wall time;
/// real hitches (far from the median) pass through unsmoothed.
struct FramePacer {
    hist: std::collections::VecDeque<f32>,
    wall: f64,
    sim: f64,
    log: Option<std::fs::File>,
    n: u64,
}

impl FramePacer {
    fn new() -> FramePacer {
        let log = std::env::var_os("SKYWARD_FRAME_LOG").and_then(|p| std::fs::File::create(p).ok());
        FramePacer { hist: Default::default(), wall: 0.0, sim: 0.0, log, n: 0 }
    }

    /// Raw wall-clock frame time in, simulation step out.
    fn step(&mut self, raw: f32, mode: &str) -> f32 {
        let raw = raw.clamp(0.0, 0.25);
        self.hist.push_back(raw);
        if self.hist.len() > 31 {
            self.hist.pop_front();
        }
        let mut v: Vec<f32> = self.hist.iter().copied().collect();
        v.sort_by(|a, b| a.total_cmp(b));
        let median = v[v.len() / 2];
        self.wall += raw as f64;
        let mut step = if self.hist.len() >= 5 && (raw - median).abs() < median * 0.35 { median } else { raw };
        // Keep sim time locked to wall time (≤ 25 % of a frame per frame).
        let err = (self.wall - self.sim - step as f64) as f32;
        step += (err * 0.1).clamp(-median * 0.25, median * 0.25);
        if (self.wall - self.sim).abs() > 0.25 {
            // After a long stall: resynchronise instead of catching up.
            self.sim = self.wall - step as f64;
        }
        let step = step.clamp(0.0, 0.1);
        self.sim += step as f64;
        if let Some(f) = &mut self.log {
            use std::io::Write;
            let _ = writeln!(f, "{} {:.3} {:.3} {:.3} {}", self.n, raw * 1000.0, step * 1000.0, median * 1000.0, mode);
        }
        self.n += 1;
        step
    }
}

fn present_mode(caps: &wgpu::SurfaceCapabilities, vsync: bool) -> wgpu::PresentMode {
    if vsync {
        wgpu::PresentMode::Fifo
    } else if caps.present_modes.contains(&wgpu::PresentMode::Mailbox) {
        wgpu::PresentMode::Mailbox
    } else if caps.present_modes.contains(&wgpu::PresentMode::Immediate) {
        wgpu::PresentMode::Immediate
    } else {
        wgpu::PresentMode::Fifo
    }
}

struct Running {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    caps: wgpu::SurfaceCapabilities,
    gpu: Gpu,
    renderer: Renderer,
    ui: Ui,
    egui_state: egui_winit::State,
    session: Option<Session>,
    status: String,
    input: Input,
    last: Instant,
    pacer: FramePacer,
    right_drag: bool,
    last_cursor: Option<(f64, f64)>,
    /// Sound output (audio/); silent if there is no device.
    audio: audio::Audio,
    loader: airports::Loader,
    format: wgpu::TextureFormat,
    quit: bool,
    /// Multiplayer: how long the controller's camera button has been held (quick-chat wheel).
    cam_hold: Option<f32>,
    saved_at: Instant,
    applied: ui::settings::Settings,
    /// The window has the keyboard focus (Settings › Sound › Mute in background).
    focused: bool,
    /// Browser diagnostics: the longest frame since the last report.
    #[cfg(target_arch = "wasm32")]
    web_max_ms: f32,
    /// ... and while flying: the longest simulation + streaming step (before
    /// rendering), (frames, frames whose step took over 8 ms).
    #[cfg(target_arch = "wasm32")]
    web_max_update_ms: f32,
    #[cfg(target_arch = "wasm32")]
    web_flying: (u64, u64),
}

struct App {
    args: Args,
    run: Option<Running>,
    #[cfg(target_arch = "wasm32")]
    init: Option<PendingInit>,
}

/// A number key (top row or keypad) for the comms menu.
fn comms_digit(code: KeyCode) -> Option<u8> {
    use KeyCode as K;
    Some(match code {
        K::Digit0 | K::Numpad0 => 0,
        K::Digit1 | K::Numpad1 => 1,
        K::Digit2 | K::Numpad2 => 2,
        K::Digit3 | K::Numpad3 => 3,
        K::Digit4 | K::Numpad4 => 4,
        K::Digit5 | K::Numpad5 => 5,
        K::Digit6 | K::Numpad6 => 6,
        K::Digit7 | K::Numpad7 => 7,
        K::Digit8 | K::Numpad8 => 8,
        K::Digit9 | K::Numpad9 => 9,
        _ => return None,
    })
}

/// Interface-level actions (keyboard or controller); true when handled here.
fn ui_action(r: &mut Running, k: Key, flying: bool) -> bool {
    let st = &mut r.ui.st;
    match k {
        Key::Menu if flying => {
            match st.screen {
                Screen::Flying => st.go(Screen::Pause),
                Screen::Pause => st.go(Screen::Flying),
                _ => return true,
            }
            r.ui.audio_clicks += 1;
            r.input.held.clear();
        }
        Key::Mute => {
            st.settings.sound.muted = !st.settings.sound.muted;
            st.settings_dirty = true;
            let m = st.settings.sound.muted;
            st.toasts.push(ui::toast::Kind::Info, if m { "Sound muted" } else { "Sound on" }, "", 1.6);
        }
        _ if st.screen != Screen::Flying => return !matches!(k, Key::Menu),
        // Multiplayer (ui/mp.rs).
        Key::Chat if st.mp.connected() => {
            st.mp.chat_open = true;
            st.mp.chat_open_frame = st.frame_no;
            st.mp.wheel = None;
        }
        Key::PlayerList if st.mp.connected() => st.mp.show_players = !st.mp.show_players,
        Key::NameTags if st.mp.connected() => {
            st.mp.tags = !st.mp.tags;
            st.toasts.push(ui::toast::Kind::Info, if st.mp.tags { "Name tags on" } else { "Name tags off" }, "", 1.4);
        }
        // Live-traffic callsign tags (adsb/) when not in a multiplayer session.
        Key::NameTags if st.adsb.settings.enabled => {
            let t = &mut st.settings.live_traffic.tags;
            *t = !*t;
            st.settings_dirty = true;
            st.toasts.push(ui::toast::Kind::Info, if *t { "Traffic tags on" } else { "Traffic tags off" }, "", 1.4);
        }
        Key::QuickChat if st.mp.connected() => st.mp.wheel = if st.mp.wheel.is_some() { None } else { Some(net::Wheel::default()) },
        Key::Chat | Key::PlayerList | Key::NameTags | Key::QuickChat => {}
        Key::Map => st.map.mode = st.map.mode.next(),
        Key::Instruments => {
            st.settings.hud = st.settings.hud.next();
            st.settings_dirty = true;
            let n = st.settings.hud.name();
            st.toasts.push(ui::toast::Kind::Info, format!("Instruments: {n}"), "", 1.4);
        }
        Key::WeatherPanel => st.show_wx = !st.show_wx,
        Key::Failures => st.fail.open = !st.fail.open,
        Key::RadioPanel => st.radio.open = !st.radio.open,
        Key::CommsMenu => {
            st.comms.open = !st.comms.open;
            st.comms.page = 0;
            st.comms.sel = 0;
        }
        Key::CommsNext | Key::CommsPrev if st.comms.open => st.comms.nav += if k == Key::CommsNext { 1 } else { -1 },
        Key::CommsSelect if st.comms.open => st.comms.select = true,
        Key::CommsNext | Key::CommsPrev | Key::CommsSelect => {}
        Key::MouseYoke => {
            if !st.settings.controls.mouse.yoke {
                st.toasts.push(ui::toast::Kind::Info, "Mouse yoke is off", "Turn it on in Settings › Controls › Mouse & view", 3.0);
            } else {
                st.input.mouse_yoke_on = !st.input.mouse_yoke_on;
                let on = st.input.mouse_yoke_on;
                st.toasts.push(ui::toast::Kind::Info, if on { "Mouse yoke on" } else { "Mouse yoke off" }, if on { "The cursor's offset from the centre is the stick" } else { "" }, 2.4);
            }
        }
        Key::MapZoomIn | Key::MapZoomOut => {
            st.map_zoom_override = false;
            return false;
        }
        _ => return false,
    }
    true
}

impl App {
    /// The browser start is waiting for the GPU.
    fn starting(&self) -> bool {
        #[cfg(target_arch = "wasm32")]
        return self.init.is_some();
        #[cfg(not(target_arch = "wasm32"))]
        false
    }

    /// Open the window and start the GPU (asynchronously in the browser).
    fn start_app(&mut self, el: &ActiveEventLoop) {
        if self.run.is_some() || self.starting() {
            return;
        }
        let mut settings = ui::settings::Settings::load();
        if self.args.mute {
            settings.sound.muted = true;
        }
        if let Some(v) = self.args.vsync {
            settings.vsync = v;
        }
        if self.args.live_given || self.args.hours.is_some() || self.args.metar.is_some() || self.args.weather.is_some() {
            let c = &mut settings.conditions;
            c.live = self.args.live && self.args.hours.is_none() && self.args.metar.is_none() && self.args.weather.is_none();
            if let Some(h) = self.args.hours {
                c.hours = h;
            }
            if let Some(d) = self.args.day_of_year {
                c.day = d;
            }
            if let Some(w) = self.args.weather {
                c.preset = w.name().to_string();
            }
            if let Some(m) = &self.args.metar {
                c.metar = m.clone();
            }
        }
        // Airports: from the cache in a moment; a first run downloads them (≈17 MB).
        #[cfg_attr(target_arch = "wasm32", allow(unused_mut))]
        let mut loader = airports::Loader::start();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let db = loader.wait(Duration::from_secs(if self.args.airport.is_some() { 90 } else { 5 }));
            resolve_airport(&mut self.args, db.as_deref());
        }
        let attrs = Window::default_attributes().with_title("Skyward");
        #[cfg(not(target_arch = "wasm32"))]
        let attrs = attrs.with_inner_size(winit::dpi::LogicalSize::new(1600.0, 900.0));
        // Browser: the page's canvas (index.html), sized by CSS to the window.
        #[cfg(target_arch = "wasm32")]
        let attrs = {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;
            let canvas = web_sys::window().and_then(|w| w.document()).and_then(|d| d.get_element_by_id("skyward")).and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok());
            attrs.with_canvas(canvas).with_append(false).with_prevent_default(true).with_focusable(true)
        };
        let window = Arc::new(el.create_window(attrs).expect("create window"));
        #[cfg(not(target_arch = "wasm32"))]
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::VULKAN | wgpu::Backends::GL,
            ..wgpu::InstanceDescriptor::new_with_display_handle(Box::new(el.owned_display_handle()))
        });
        #[cfg(target_arch = "wasm32")]
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor { backends: wgpu::Backends::BROWSER_WEBGPU, ..wgpu::InstanceDescriptor::new_without_display_handle() });
        let surface = instance.create_surface(window.clone()).expect("create surface");
        #[cfg(not(target_arch = "wasm32"))]
        {
            let gpu = Gpu::new(Some(&surface), instance).expect("GPU init");
            self.finish_init(window, surface, gpu, settings, loader);
        }
        // Browser: adapter and device arrive asynchronously (about_to_wait
        // finishes the start once they, and a requested airport, are in).
        #[cfg(target_arch = "wasm32")]
        {
            platform::web::status("Starting WebGPU", -1.0);
            let slot = std::rc::Rc::new(std::cell::RefCell::new(None));
            let s2 = slot.clone();
            platform::web::spawn_local(async move {
                let r = Gpu::new_async(Some(&surface), instance).await.map(|g| (surface, g));
                *s2.borrow_mut() = Some(r);
            });
            window.request_redraw();
            self.init = Some(PendingInit { window, slot, settings, loader, t0: Instant::now() });
        }
    }
}

/// Browser start in progress: waiting for the GPU device (async) and, if
/// the page asked for an airport, the airport database.
#[cfg(target_arch = "wasm32")]
struct PendingInit {
    window: Arc<Window>,
    slot: std::rc::Rc<std::cell::RefCell<Option<Result<(wgpu::Surface<'static>, Gpu), String>>>>,
    settings: ui::settings::Settings,
    loader: airports::Loader,
    t0: Instant,
}

impl App {
    #[cfg(target_arch = "wasm32")]
    fn poll_init(&mut self) {
        let Some(p) = self.init.as_mut() else { return };
        p.loader.poll();
        p.window.request_redraw();
        let db_wait = self.args.airport.is_some() && p.loader.db.is_none() && !matches!(p.loader.status, airports::Status::Failed(_)) && p.t0.elapsed() < Duration::from_secs(30);
        if db_wait {
            platform::web::status("Loading the airport database", -1.0);
        }
        let ready = p.slot.borrow().is_some();
        if !ready || db_wait {
            return;
        }
        let p = self.init.take().unwrap();
        let r = p.slot.borrow_mut().take().unwrap();
        match r {
            Ok((surface, gpu)) => {
                resolve_airport(&mut self.args, p.loader.db.as_deref());
                self.finish_init(p.window, surface, gpu, p.settings, p.loader);
                platform::web::ready();
            }
            Err(e) => platform::web::fatal(&format!("Could not start WebGPU: {e}")),
        }
    }

    /// The rest of the start once the GPU is up: surface, renderer, UI, sound, the first session.
    fn finish_init(&mut self, window: Arc<Window>, surface: wgpu::Surface<'static>, gpu: Gpu, settings: ui::settings::Settings, loader: airports::Loader) {
        let db = loader.db.clone();
        eprintln!("GPU: {}", gpu.adapter_name());
        let size = window.inner_size();
        let caps = surface.get_capabilities(&gpu.adapter);
        let surface_format = caps.formats.iter().copied().find(|f| f.is_srgb()).unwrap_or(caps.formats[0]);
        // WebGPU canvases have no sRGB formats: render through an sRGB view.
        let format = surface_format.add_srgb_suffix();
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: present_mode(&caps, settings.vsync),
            desired_maximum_frame_latency: 2,
            alpha_mode: caps.alpha_modes[0],
            view_formats: if format != surface_format { vec![format] } else { vec![] },
            ..surface.get_default_config(&gpu.adapter, size.width.max(1), size.height.max(1)).unwrap()
        };
        eprintln!("present mode {:?}", config.present_mode);
        surface.configure(&gpu.device, &config);
        let renderer = Renderer::new(&gpu.device, &gpu.queue, format);
        let applied = settings.clone();
        let mut ui = Ui::new(&gpu.device, format, settings);
        let egui_state = egui_winit::State::new(ui.ctx.clone(), egui::ViewportId::ROOT, &window, Some(window.scale_factor() as f32), None, None);
        // Game controllers (hot-plug), plus virtual ones for testing (SKYWARD_FAKE_INPUT).
        ui.st.input.start();
        ui.st.input.add_fakes_from_env();
        eprintln!("{}", ui.st.input.status);
        ui.st.db = db.clone();
        if let Err(e) = setup_adsb(&self.args, &mut ui, false) {
            eprintln!("{e}");
        }
        if let Some(m) = self.args.hud_mode {
            ui.st.settings.hud = m;
        }
        if let Some(m) = self.args.map_mode {
            ui.st.map.mode = m;
        }
        // What to fly: the command line, else the last flight.
        ui.st.plan = plan_from_args(&self.args, db.as_deref());
        let explicit = self.args.location_given || self.args.aircraft_given || self.args.start_at.is_some() || self.args.bot;
        if !explicit {
            if let Some(l) = ui.st.settings.last.clone() {
                if let Some(i) = ui.st.defs.iter().position(|d| d.id == l.aircraft) {
                    ui.st.plan.aircraft = i;
                }
                if db.as_ref().is_some_and(|d| d.get(&l.airport).is_some()) {
                    ui.st.plan.airport = l.airport.clone();
                    ui.st.plan.spec = startpos::Spec::decode(&l.start).unwrap_or(startpos::Spec::Runway(String::new()));
                }
                ui.st.plan.watch = l.watch;
            }
        }
        // The departure loads behind the main menu (or straight away with --no-menu).
        let (args2, custom) = if explicit && ui.st.plan.airport.is_empty() { (self.args.clone(), None) } else { args_for_plan(&self.args, &ui.st) };
        let args2 = if explicit && self.args.airport_ident.is_none() && self.args.location_given {
            // A raw lat / lon: keep it.
            Args { lat: self.args.lat, lon: self.args.lon, ..args2 }
        } else {
            args2
        };
        // `--mission ID`: straight into it (its own start and conditions).
        let (args2, custom) = match &self.args.mission {
            Some(id) => {
                ui.st.missions.active = Some(id.clone());
                ui.st.missions.bot = self.args.mission_bot;
                ui.st.show_wx = false;
                let c = self.args.start_at.clone().map(|spec| startpos::CustomStart { airport: ui.st.plan.airport.clone(), spec });
                (self.args.clone(), c)
            }
            None => (args2, custom),
        };
        ui.st.screen = if self.args.no_menu || self.args.mission.is_some() { Screen::Loading } else { Screen::Main };
        // Settings that live elsewhere or act once at start-up (ui/settings.rs).
        ui.st.settings.player_name = ui.st.mp.prefs.name.clone();
        ui.st.mp.tags = ui.st.settings.mp.name_tags;
        ui.st.gpu_name = gpu.adapter_name();
        ui.st.settings_ui.launch = Some(ui.st.settings.clone());
        platform::set_offline(ui.st.settings.data.offline);
        #[cfg(not(target_arch = "wasm32"))]
        ui::settings_ui::prune_cache(ui.st.settings.data.cache_limit_mb);
        let mut audio = audio::Audio::new(ui.st.settings.sound.muted);
        audio.set_device(&ui.st.settings.audio_device);
        ui.audio_status = audio.status().to_string();
        ui.st.audio_status = ui.audio_status.clone();
        let session = Session::new(&args2, db, custom);
        self.run = Some(Running {
            window,
            surface,
            config,
            caps,
            gpu,
            renderer,
            ui,
            egui_state,
            session: Some(session),
            status: "Starting".into(),
            input: Input::default(),
            last: Instant::now(),
            pacer: FramePacer::new(),
            right_drag: false,
            last_cursor: None,
            audio,
            loader,
            format,
            quit: false,
            cam_hold: None,
            saved_at: Instant::now(),
            applied,
            focused: true,
            #[cfg(target_arch = "wasm32")]
            web_max_ms: 0.0,
            #[cfg(target_arch = "wasm32")]
            web_max_update_ms: 0.0,
            #[cfg(target_arch = "wasm32")]
            web_flying: (0, 0),
        });
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, el: &ActiveEventLoop) {
        self.start_app(el);
    }

    fn window_event(&mut self, el: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let Some(r) = self.run.as_mut() else { return };
        let resp = r.egui_state.on_window_event(&r.window, &event);
        let screen = r.ui.st.screen;
        let typing = r.ui.ctx.egui_wants_keyboard_input();
        match &event {
            WindowEvent::CloseRequested => {
                // (Changes not applied on the settings screen are not kept.)
                match r.ui.st.settings_ui.base.as_ref().filter(|b| **b != r.ui.st.settings) {
                    Some(b) => b.save(),
                    None => r.ui.st.settings.save(),
                }
                el.exit();
            }
            WindowEvent::Resized(s) => {
                r.config.width = s.width.max(1);
                r.config.height = s.height.max(1);
                r.surface.configure(&r.gpu.device, &r.config);
            }
            WindowEvent::KeyboardInput { event: ke, .. } => {
                if let PhysicalKey::Code(code) = ke.physical_key {
                    let pressed = ke.state == ElementState::Pressed && !ke.repeat;
                    let flying = r.session.as_ref().is_some_and(|s| s.sim.is_some());
                    // Settings › Controls waiting for a key (Esc cancels).
                    if let Some(input::Listen::Key(k)) = r.ui.st.input.listen {
                        if pressed {
                            r.ui.st.input.listen = None;
                            if code != KeyCode::Escape {
                                r.ui.st.input.captured = Some(input::Captured::Key(k, code));
                            }
                        }
                        return;
                    }
                    // The comms menu (F4) takes the number keys while it is open (ui/comms.rs).
                    if r.ui.st.comms.open && screen == Screen::Flying && !typing {
                        if let Some(d) = comms_digit(code) {
                            if pressed {
                                r.ui.st.comms.keys.push(d);
                            }
                            return;
                        }
                        if pressed && code == KeyCode::Escape {
                            r.ui.st.comms.open = false;
                            return;
                        }
                    }
                    // Multiplayer: Esc closes the chat / quick-chat wheel; 1–8 pick a quick message.
                    if screen == Screen::Flying && (r.ui.st.mp.chat_open || r.ui.st.mp.wheel.is_some()) {
                        if pressed && code == KeyCode::Escape {
                            r.ui.st.mp.chat_open = false;
                            r.ui.st.mp.wheel = None;
                            return;
                        }
                        if let (false, Some(d)) = (r.ui.st.mp.chat_open, comms_digit(code)) {
                            if pressed && (1..=net::QUICK_CHAT.len() as u8).contains(&d) {
                                r.ui.st.mp.say(net::QUICK_CHAT[d as usize - 1]);
                                r.ui.st.mp.wheel = None;
                            }
                            return;
                        }
                    }
                    if pressed && code == KeyCode::Escape && flying {
                        ui_action(r, Key::Menu, flying);
                        return;
                    }
                    // Bound actions (input/: rebindable in Settings › Controls).
                    let keys = r.ui.st.settings.controls.actions_for_key(code);
                    if screen != Screen::Flying || typing {
                        if ke.state == ElementState::Released {
                            for k in &keys {
                                r.input.held.remove(k);
                            }
                        } else if pressed && !typing {
                            for k in keys.iter().filter(|k| matches!(k, Key::Menu | Key::Mute)) {
                                ui_action(r, *k, flying);
                            }
                        }
                        return;
                    }
                    for k in keys {
                        match ke.state {
                            ElementState::Pressed => {
                                if (!ke.repeat || k.repeats()) && !ui_action(r, k, flying) {
                                    r.input.pressed.push(k);
                                }
                                r.input.held.insert(k);
                            }
                            ElementState::Released => {
                                r.input.held.remove(&k);
                            }
                        }
                    }
                }
            }
            WindowEvent::MouseInput { state, button: MouseButton::Right, .. } => {
                r.right_drag = *state == ElementState::Pressed && !resp.consumed && screen == Screen::Flying;
            }
            WindowEvent::CursorMoved { position, .. } => {
                if let (true, Some((lx, ly))) = (r.right_drag, r.last_cursor) {
                    r.input.orbit_delta += glam::vec2((position.x - lx) as f32, (position.y - ly) as f32);
                }
                r.last_cursor = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } if !resp.consumed && screen == Screen::Flying && !r.ui.ctx.is_pointer_over_egui() => {
                r.input.zoom_delta += match delta {
                    MouseScrollDelta::LineDelta(_, y) => *y,
                    MouseScrollDelta::PixelDelta(p) => p.y as f32 / 40.0,
                };
            }
            WindowEvent::Focused(f) => r.focused = *f,
            WindowEvent::RedrawRequested => self.frame(),
            _ => {}
        }
    }

    fn about_to_wait(&mut self, el: &ActiveEventLoop) {
        #[cfg(target_arch = "wasm32")]
        self.poll_init();
        if let Some(r) = &self.run {
            if r.quit {
                r.ui.st.settings.save();
                el.exit();
                return;
            }
            r.window.request_redraw();
        }
    }
}

/// Game controllers and the mouse yoke → this frame's input (input/).
fn poll_controllers(r: &mut Running, dt: f32) {
    let st = &mut r.ui.st;
    if st.input.update(&mut st.settings.controls, dt) {
        st.settings_dirty = true;
    }
    for n in std::mem::take(&mut st.input.arrived) {
        let p = st.input.devices.iter().find(|d| d.info.name == n).and_then(|d| st.settings.controls.profile(&d.info.key)).map(|p| p.preset.clone()).unwrap_or_default();
        st.toasts.push(ui::toast::Kind::Info, format!("{n} connected"), format!("{p} · Settings › Controls to change"), 3.5);
    }
    for n in std::mem::take(&mut st.input.left) {
        st.toasts.push(ui::toast::Kind::Warn, format!("{n} disconnected"), "", 3.5);
    }
    // Mouse yoke: the cursor's offset from the window centre (−1..1, +y down = pull).
    let flying = st.screen == Screen::Flying && !r.ui.ctx.is_pointer_over_egui();
    st.input.mouse_offset = match (flying, r.last_cursor) {
        (true, Some((x, y))) => {
            let (w, h) = (r.config.width as f64, r.config.height as f64);
            let s = 0.5 * w.min(h);
            Some((((x - 0.5 * w) / s).clamp(-1.0, 1.0) as f32, ((y - 0.5 * h) / s).clamp(-1.0, 1.0) as f32))
        }
        _ => None,
    };
    let in_flight = r.session.as_ref().is_some_and(|s| s.sim.is_some());
    // Multiplayer: holding the controller's camera (View) button opens the
    // quick-chat wheel (right stick picks, release sends); a tap still
    // changes the camera.
    let mp_pad = r.ui.st.mp.connected() && r.ui.st.screen == Screen::Flying;
    for k in r.ui.st.input.take_pressed() {
        if mp_pad && k == Key::Camera && r.ui.st.input.held.contains(&Key::Camera) {
            r.cam_hold = Some(0.0);
            continue;
        }
        if !ui_action(r, k, in_flight) && r.ui.st.screen == Screen::Flying {
            r.input.pressed.push(k);
        }
    }
    if let Some(t) = r.cam_hold {
        let st = &mut r.ui.st;
        if st.input.held.contains(&Key::Camera) {
            let t = t + dt;
            r.cam_hold = Some(t);
            if t > 0.35 && mp_pad {
                let x = st.input.analog.get(input::AxisAction::LookX).unwrap_or(0.0);
                let y = st.input.analog.get(input::AxisAction::LookY).unwrap_or(0.0);
                let mut w = st.mp.wheel.unwrap_or(net::Wheel { sel: None, held: true });
                w.held = true;
                if let Some(i) = ui::mp::wheel_pick(egui::vec2(x, -y)) {
                    w.sel = Some(i);
                }
                st.mp.wheel = Some(w);
            }
        } else {
            r.cam_hold = None;
            match st.mp.wheel.take() {
                Some(w) if w.held => {
                    if let Some(i) = w.sel {
                        st.mp.say(net::QUICK_CHAT[i]);
                    }
                }
                other => {
                    st.mp.wheel = other;
                    if t <= 0.35 {
                        r.input.pressed.push(Key::Camera);
                    }
                }
            }
        }
    }
    let st = &r.ui.st;
    r.input.pad_held = if st.screen == Screen::Flying { st.input.held.clone() } else { Default::default() };
    r.input.analog = st.input.analog;
    if st.mp.wheel.is_some_and(|w| w.held) {
        // (The right stick picks in the wheel instead of looking around.)
        r.input.analog.v[input::AxisAction::LookX.idx()] = None;
        r.input.analog.v[input::AxisAction::LookY.idx()] = None;
    }
    r.input.mouse = st.settings.controls.mouse.clone();
    r.input.dragging = r.right_drag;
}

impl App {
    /// Start (or switch to) the planned flight.
    fn start_flight(&mut self) {
        let Some(r) = self.run.as_mut() else { return };
        // Browser: download the aircraft first (the loading screen shows it).
        if platform::WEB && aircraft_assets(r.ui.st.plan.aircraft).is_some() {
            r.ui.st.go(Screen::Loading);
            r.ui.st.actions.push(Action::Fly);
            return;
        }
        remember_plan(&mut r.ui.st);
        let (mut args2, custom) = args_for_plan(&self.args, &r.ui.st);
        // A free flight: no mission.
        args2.mission = None;
        r.ui.st.missions.active = None;
        // Same departure area: keep the loaded world, just re-place / re-select.
        let reuse = r.session.as_ref().is_some_and(|s| {
            let home = s.world.home;
            airports::distance_km((home.lat0, home.lon0), (args2.lat, args2.lon)) < 3.0
        });
        if reuse {
            let s = r.session.as_mut().unwrap();
            s.args = args2.clone();
            s.custom = custom.clone();
            let w = &mut s.world;
            w.radius = args2.radius.unwrap_or(w.radius);
            if let Some(l) = args2.traffic {
                w.traffic.set_level(l);
            }
            if args2.live {
                w.set_live(true);
            } else {
                w.live_weather = false;
                w.live_time = false;
                w.live_date = false;
                match args2.metar.as_deref().and_then(weather::Wx::from_metar_str) {
                    Some(wx) => w.set_wx(wx),
                    None => w.set_weather_preset(args2.weather.unwrap_or(clouds::Weather::Scattered)),
                }
            }
            if let Some(sim) = s.sim.as_mut() {
                let before = Some(sim.ac.pos);
                sim.mission = None;
                sim.mission_pilot = None;
                sim.tenders = None;
                sim.custom_start = custom;
                sim.select(&s.world, args2.aircraft, args2.start, args2.watch);
                if let Some(cfg) = args2.dogfight.clone() {
                    sim.start_dogfight(&s.world, cfg);
                }
                if !args2.live {
                    sim.hours = args2.hours.unwrap_or(sim.hours);
                    sim.day_of_year = args2.day_of_year.unwrap_or(sim.day_of_year);
                }
                s.check_catch_up(before);
                r.ui.st.go(Screen::Flying);
            } else {
                r.ui.st.go(Screen::Loading);
            }
        } else {
            let mut new = Session::new(&args2, r.ui.st.db.clone(), custom);
            let old = r.session.take();
            replace_session(old, &mut new, &mut r.renderer, &mut r.ui);
            r.session = Some(new);
            r.ui.st.go(Screen::Loading);
        }
        r.input.held.clear();
        r.input.pressed.clear();
    }

    /// Fly the mission chosen in the briefing (missions/): a new session at its start.
    fn start_mission(&mut self) {
        let Some(r) = self.run.as_mut() else { return };
        // Missions are single-player: not while connected to a server.
        if r.ui.st.mp.connected() {
            r.ui.st.missions.pending = None;
            return;
        }
        let Some(m) = r.ui.st.missions.pending.take().and_then(|id| missions::defs::by_id(&id)) else { return };
        let args2 = args_for_mission(&self.args, m);
        r.ui.st.missions.active = Some(m.id.clone());
        r.ui.st.show_wx = false;
        r.ui.st.plan.aircraft = args2.aircraft;
        r.ui.st.plan.airport = m.target.airport.clone();
        if let Some(spec) = &args2.start_at {
            r.ui.st.plan.spec = spec.clone();
        }
        // Browser: the aircraft downloads first (the loading screen shows it).
        let _ = aircraft_assets(args2.aircraft);
        let custom = args2.start_at.clone().map(|spec| startpos::CustomStart { airport: m.target.airport.clone(), spec });
        let mut new = Session::new(&args2, r.ui.st.db.clone(), custom);
        let old = r.session.take();
        replace_session(old, &mut new, &mut r.renderer, &mut r.ui);
        r.session = Some(new);
        r.ui.st.go(Screen::Loading);
        r.input.held.clear();
        r.input.pressed.clear();
    }

    fn frame(&mut self) {
        let Some(r) = self.run.as_mut() else { return };
        #[cfg(target_arch = "wasm32")]
        {
            // Main-thread work (tile builds, decodes): a small slice of each
            // frame while flying, more behind the menus / loading screen.
            platform::web::frame_begin();
            let flying = r.ui.st.screen == Screen::Flying && r.session.as_ref().is_some_and(|s| s.sim.is_some() && s.catch_up.is_none());
            platform::web::set_budget_ms(if flying { 6.0 } else { 28.0 });
        }
        let now = Instant::now();
        let raw_dt = (now - r.last).as_secs_f32();
        r.last = now;
        let mode = format!("{:?}", r.config.present_mode);
        let dt = r.pacer.step(raw_dt, &mode);
        r.ui.fps = r.ui.fps * 0.95 + (1.0 / raw_dt.max(1e-4)) * 0.05;
        // Airport database arriving late (first run download).
        if r.loader.poll() {
            r.ui.st.db = r.loader.db.clone();
            if let (Some(s), Some(db)) = (r.session.as_mut(), r.loader.db.clone()) {
                if s.world.airport_db.is_none() {
                    s.world.set_airport_db(db);
                }
            }
        }
        if let airports::Status::Failed(e) = &r.loader.status {
            r.ui.st.airport_status = Some(e.clone());
        }
        r.ui.st.dt = dt;
        r.ui.st.time += dt;
        // Test hook: SKYWARD_UI_SCRIPT="3:aircraft,6:airport=EGKB,8:fly,20:pause" drives the menus.
        if let Ok(script) = std::env::var("SKYWARD_UI_SCRIPT") {
            for step in script.split(',') {
                let Some((t, cmd)) = step.split_once(':') else { continue };
                let t: f32 = t.parse().unwrap_or(f32::MAX);
                if t <= r.ui.st.time && t > r.ui.st.time - dt {
                    eprintln!("ui script: {cmd}");
                    let (k, v) = cmd.split_once('=').unwrap_or((cmd, ""));
                    match k {
                        "fly" => r.ui.st.actions.push(Action::Fly),
                        "airport" => r.ui.st.plan.airport = v.to_string(),
                        "start" => r.ui.st.plan.spec = startpos::Spec::decode(v).unwrap_or(startpos::Spec::Runway(String::new())),
                        "aircraft" if !v.is_empty() => {
                            if let Some(i) = r.ui.st.defs.iter().position(|d| d.id == v) {
                                r.ui.st.plan.aircraft = i;
                            }
                        }
                        "quit" => r.ui.st.actions.push(Action::Quit),
                        other => {
                            if let Some(sc) = Screen::parse(other) {
                                r.ui.st.actions.push(Action::Goto(sc));
                            }
                        }
                    }
                }
            }
        }
        r.ui.st.toasts.tick(dt);
        // Multiplayer (net/): network every frame; a join that succeeded starts
        // the flight at the server's airport.
        r.ui.st.mp.poll();
        if std::mem::take(&mut r.ui.st.mp.start_flight) {
            mp_start_flight(&mut r.ui.st);
        }
        for (k, t, b) in std::mem::take(&mut r.ui.st.mp.notices) {
            r.ui.st.toasts.push(k, t, b, 5.0);
        }
        poll_controllers(r, dt);
        let screen = r.ui.st.screen;
        let mut notice: Option<String> = None;
        if let Some(s) = r.session.as_mut() {
            if s.sim.is_some() {
                r.input.pressed.retain(|k| *k != Key::Screenshot);
                let before = s.sim.as_ref().map(|x| x.ac.pos);
                if screen == Screen::Flying && s.catch_up.is_none() {
                    let sim = s.sim.as_mut().unwrap();
                    // Damage / crash assists (missions override them in the sim).
                    sim.assists = damage::Assists { structural: r.ui.st.settings.structural_damage, crashes: r.ui.st.settings.crashes };
                    sim.update(&s.world, &mut r.input, dt);
                } else {
                    r.input.pressed.clear();
                    r.input.orbit_delta = glam::Vec2::ZERO;
                    r.input.zoom_delta = 0.0;
                }
                // Multiplayer: our state out, others in, the server's time and weather.
                if let Some(sim) = s.sim.as_mut() {
                    r.ui.st.mp.fly(sim, &mut s.world, screen != Screen::Flying, dt);
                    if std::mem::take(&mut r.ui.st.mp.want_restart) {
                        sim.paused = false;
                        r.input.pressed.push(Key::Reset);
                    }
                }
                s.follow();
                notice = s.check_catch_up(before);
                let budget = if notice.is_some() { None } else { Some(UPLOAD_BUDGET) };
                sync_gpu(s, &mut r.renderer, &mut r.ui, &r.gpu, budget);
                if let Some(sim) = s.sim.as_mut() {
                    // A mission has ended: the debrief (and the best result kept) a few seconds later.
                    if r.ui.st.screen == Screen::Flying && sim.mission_over_for().is_some_and(|t| t > 3.5) {
                        if let Some(run) = sim.mission.as_ref() {
                            if let Some(d) = run.end.clone() {
                                let id = run.m.id.clone();
                                let best = !r.ui.st.missions.bot && r.ui.st.missions.records().record(&id, &d);
                                if !r.ui.st.missions.bot {
                                    r.ui.st.missions.records().save();
                                }
                                let cp = sim.mission_checkpoint.as_ref().map(|c| c.name.clone());
                                r.ui.st.missions.last = Some((id, d, best, cp));
                                r.ui.st.go(Screen::Debrief);
                            }
                        }
                    }
                    if let Some((msg, _)) = sim.note.take() {
                        let kind = if msg.contains("too short") || msg.contains("tight") { ui::toast::Kind::Warn } else { ui::toast::Kind::Info };
                        r.ui.st.toasts.push(kind, msg, "", 7.0);
                    }
                }
            } else {
                sync_gpu(s, &mut r.renderer, &mut r.ui, &r.gpu, None);
                if let Some(m) = s.advance_loading() {
                    r.status = m;
                }
                notice = Some(r.status.clone());
                if s.sim.is_some() && r.ui.st.screen == Screen::Loading {
                    r.ui.st.go(Screen::Flying);
                    notice = None;
                }
            }
        }
        if r.ui.st.screen == Screen::Loading && notice.is_none() {
            notice = Some("Preparing\n".into());
        }
        // Browser: the chosen aircraft is still downloading.
        if r.ui.st.screen == Screen::Loading {
            if let Some(m) = aircraft_assets(r.ui.st.plan.aircraft) {
                notice = Some(m);
            }
        }
        // Loading screen title: the destination.
        let notice = notice.map(|n| if r.ui.st.screen == Screen::Loading { loading_text(&r.ui.st, &n) } else { n });
        // Thumbnails for the aircraft cards, once there is scenery to render them over.
        // (Not in the browser: it would download every aircraft.)
        if !platform::WEB && matches!(r.ui.st.screen, Screen::Aircraft | Screen::Main | Screen::Missions | Screen::Briefing) && r.ui.thumbs.is_empty() {
            if let Some(s) = r.session.as_ref().filter(|s| s.sim.is_some()) {
                render_thumbs(&r.gpu, &mut r.renderer, &mut r.ui, s, r.format);
            }
        }
        // (The browser loads pre-rendered ones: platform::web::thumbs_start.)
        #[cfg(target_arch = "wasm32")]
        {
            if matches!(r.ui.st.screen, Screen::Aircraft | Screen::Main) {
                platform::web::thumbs_start(&r.ui.st.defs);
            }
            for (id, w, h, px) in platform::web::thumbs_take() {
                let Some(def) = r.ui.st.defs.iter().find(|d| d.id == id) else { continue };
                let img = egui::ColorImage::from_rgba_unmultiplied([w as usize, h as usize], &px);
                let handle = r.ui.ctx.load_texture(format!("thumb_{id}"), img, egui::TextureOptions::LINEAR);
                r.ui.st.thumbs.insert(def.id, handle.id());
                r.ui.thumbs.push(handle);
            }
        }
        #[cfg(target_arch = "wasm32")]
        let t_sim = now.elapsed().as_secs_f32() * 1000.0;
        let frame = match r.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(f) | wgpu::CurrentSurfaceTexture::Suboptimal(f) => f,
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                r.surface.configure(&r.gpu.device, &r.config);
                return;
            }
            _ => return,
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor { format: Some(r.format), ..Default::default() });
        let raw = r.egui_state.take_egui_input(&r.window);
        let ppp = r.window.scale_factor() as f32;
        let (w, h) = (r.config.width, r.config.height);
        let fps = r.ui.fps;
        render_frame(&r.gpu, &mut r.renderer, &mut r.ui, r.session.as_mut(), &view, w, h, raw, ppp, fps, notice.as_deref(), None);
        // Sound: menu clicks, settings, this frame's sim state (camera set by render_frame).
        if r.ui.audio_clicks > 0 {
            r.audio.click();
            r.ui.audio_clicks = 0;
        }
        {
            let s = &r.ui.st.settings;
            let mut snd = s.sound;
            snd.muted |= s.mute_in_background && !r.focused;
            r.audio.set_settings(snd);
            r.audio.set_device(&s.audio_device);
        }
        {
            // Other players' engines; the spectator camera is the listener.
            let mp = &r.ui.st.mp;
            let ear = mp.cam.map(|c| c.0).unwrap_or_default();
            let listener = mp.cam.filter(|_| mp.spectate.is_some()).map(|(eye, view, _)| {
                let inv = view.inverse();
                audio::Listener { pos: eye, fwd: -inv.z_axis.truncate(), up: inv.y_axis.truncate() }
            });
            // Live traffic (adsb/) fills the slots the players leave.
            let mut sounds = mp.sounds(ear);
            let lt = &r.ui.st.adsb;
            if let Some(cam) = lt.cam.filter(|_| lt.settings.enabled) {
                lt.sounds(cam.0, &mut sounds);
            }
            r.audio.set_traffic(sounds, listener);
        }
        if let Some(s) = r.session.as_ref() {
            r.audio.update(s.sim.as_ref(), &s.world, dt, r.ui.st.screen != Screen::Flying);
        }
        if r.ui.audio_status != r.audio.status() {
            r.ui.audio_status = r.audio.status().to_string();
            r.ui.st.audio_status = r.ui.audio_status.clone();
        }
        r.window.pre_present_notify();
        r.gpu.queue.present(frame);
        #[cfg(target_arch = "wasm32")]
        {
            // Diagnostics for test harnesses (window.skywardStats).
            let total = now.elapsed().as_secs_f32() * 1000.0;
            r.web_max_ms = r.web_max_ms.max(total);
            // While flying: frames whose simulation + streaming step took over 8 ms (hitches).
            if r.ui.st.screen == Screen::Flying {
                r.web_flying.0 += 1;
                if t_sim > 8.0 {
                    r.web_flying.1 += 1;
                    if t_sim > 20.0 {
                        platform::web::log(&format!("slow frame: simulation + streaming {t_sim:.1} ms"));
                    }
                }
                r.web_max_update_ms = r.web_max_update_ms.max(t_sim);
            }
            if r.ui.st.frame_no % 30 == 0 {
                let (tiles, queued, drawn, st) = r.session.as_ref().map(|s| (s.world.tile_count(), s.world.queued(), r.renderer.stats.tiles_drawn, s.world.stats.clone())).unwrap_or_default();
                platform::web::publish_stats(&format!(
                    "{{\"cpu_ms\":{total:.2},\"max_cpu_ms\":{:.2},\"update_ms\":{t_sim:.2},\"flying_frames\":{},\"slow_updates\":{},\"max_update_ms\":{:.2},\"fps\":{:.1},\"tiles\":{tiles},\"queued\":{queued},\"drawn\":{drawn},\"built\":{},\"avg_build_ms\":{:.1},\"max_build_ms\":{:.1},\"screen\":\"{:?}\",\"size\":\"{}x{}\",\"audio\":\"{}\"}}",
                    r.web_max_ms,
                    r.web_flying.0,
                    r.web_flying.1,
                    r.web_max_update_ms,
                    r.ui.fps,
                    st.tiles_built,
                    st.build_ms / st.tiles_built.max(1) as f32,
                    st.max_build_ms,
                    r.ui.st.screen,
                    r.config.width,
                    r.config.height,
                    r.ui.audio_status.replace('"', "'")
                ));
                r.web_max_ms = 0.0;
            }
        }
        // Menu actions.
        let actions: Vec<Action> = std::mem::take(&mut r.ui.st.actions);
        for a in actions {
            let Some(r) = self.run.as_mut() else { return };
            match a {
                Action::Goto(s) => {
                    // Leaving a mission for the menus ends it.
                    if matches!(s, Screen::Missions | Screen::Main) && r.ui.st.missions.active.take().is_some() {
                        if let Some(sim) = r.session.as_mut().and_then(|x| x.sim.as_mut()) {
                            sim.mission = None;
                            sim.mission_pilot = None;
                            sim.tenders = None;
                        }
                    }
                    r.ui.st.go(s)
                }
                Action::StartMission => self.start_mission(),
                Action::RetryCheckpoint => {
                    if let Some(sim) = r.session.as_mut().and_then(|x| x.sim.as_mut()) {
                        sim.mission_retry_checkpoint();
                    }
                    r.ui.st.go(Screen::Flying);
                }
                Action::Back => {
                    let b = r.ui.st.back_to;
                    r.ui.st.go(b);
                }
                Action::Resume => r.ui.st.go(Screen::Flying),
                Action::Failures => {
                    r.ui.st.fail.open = true;
                    r.ui.st.go(Screen::Flying);
                }
                Action::Restart => {
                    r.input.pressed.push(Key::Reset);
                    r.ui.st.go(Screen::Flying);
                }
                Action::Quit => {
                    r.ui.st.mp.leave();
                    r.quit = true;
                }
                Action::LeaveServer => {
                    r.ui.st.mp.leave();
                    r.ui.st.toasts.push(ui::toast::Kind::Info, "Left the server", "Flying on alone", 3.0);
                    r.ui.st.go(Screen::Flying);
                }
                Action::FlyLast => {
                    if let Some(l) = r.ui.st.settings.last.clone() {
                        if let Some(i) = r.ui.st.defs.iter().position(|d| d.id == l.aircraft) {
                            r.ui.st.plan.aircraft = i;
                        }
                        r.ui.st.plan.airport = l.airport;
                        r.ui.st.plan.spec = startpos::Spec::decode(&l.start).unwrap_or(startpos::Spec::Runway(String::new()));
                        r.ui.st.plan.watch = l.watch;
                        r.ui.st.plan.dogfight = None;
                    }
                    self.start_flight();
                }
                Action::Fly => self.start_flight(),
            }
        }
        // Settings: apply what changed, save (debounced).
        let Some(r) = self.run.as_mut() else { return };
        let s = r.ui.st.settings.clone();
        if s.vsync != r.applied.vsync {
            r.config.present_mode = present_mode(&r.caps, s.vsync);
            r.surface.configure(&r.gpu.device, &r.config);
            eprintln!("present mode {:?}", r.config.present_mode);
        }
        if s.detail != r.applied.detail {
            if let Some(sess) = r.session.as_mut() {
                sess.world.radius = ui::settings::detail_radius(s.detail);
            }
        }
        if s.traffic.road != r.applied.traffic.road {
            if let Some(sess) = r.session.as_mut() {
                sess.world.traffic.set_level(traffic::Level::from_u8(s.traffic.road));
            }
        }
        platform::set_offline(s.data.offline);
        r.ui.st.mp.default_port = s.mp.default_port;
        r.applied = s;
        // (Unapplied changes on the settings screen wait for Apply.)
        if r.ui.st.settings_dirty && r.saved_at.elapsed() > Duration::from_secs(1) && !r.ui.st.settings_ui.pending(&r.ui.st.settings) {
            r.ui.st.settings.save();
            r.ui.st.settings_dirty = false;
            r.saved_at = Instant::now();
        }
        if let Some(s) = r.session.as_ref().and_then(|s| s.sim.as_ref()) {
            let t = format!("Skyward — {} · {}", s.def().name, r.ui.st.plan.airport);
            if r.ui.st.frame_no % 120 == 1 {
                r.window.set_title(&t);
            }
        }
        #[cfg(target_arch = "wasm32")]
        platform::web::run_deferred();
    }
}

// ---------------------------------------------------------------- offline sound

#[cfg(not(target_arch = "wasm32"))]
/// `--audio-render out.wav --seconds S`: fly the sim headlessly (no GPU, no
/// audio device) and record what the player would hear — WAV, spectrogram
/// PNG and a report of objective checks (audio/render.rs).
fn audio_render(args: Args) -> Result<(), String> {
    let path = args.audio_render.clone().unwrap();
    let mut args = args;
    let db = airports::Loader::start().wait(Duration::from_secs(120));
    resolve_airport(&mut args, db.as_deref());
    let _ = nav::db::wait(Duration::from_secs(60));
    let _ = atc::freqs::wait(Duration::from_secs(60));
    // `--start-at` (stand / apron, final...) as in the headless shots.
    let custom = args.start_at.clone().map(|spec| startpos::CustomStart { airport: plan_from_args(&args, db.as_deref()).airport, spec });
    let mut s = Session::new(&args, db, custom);
    let t0 = Instant::now();
    fn pump(s: &mut Session) {
        s.world.update();
        s.world.uploads.clear();
        s.world.gpu_pending.clear();
        let _ = s.world.take_evicted();
        let _ = s.world.take_map_vectors();
        if let Some(sim) = s.sim.as_mut() {
            if let Some((h, d)) = s.world.tick_weather(sim.time, (sim.hours, sim.day_of_year)) {
                sim.hours = h;
                sim.day_of_year = d;
            }
        }
    }
    while s.sim.is_none() {
        pump(&mut s);
        s.advance_loading();
        if t0.elapsed() > Duration::from_secs(180) {
            return Err("timed out loading the map".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    while s.check_catch_up(None).is_some() {
        s.follow();
        pump(&mut s);
        if t0.elapsed() > Duration::from_secs(240) {
            return Err("timed out loading the start position".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    if s.world.live_weather || s.world.live_time {
        s.world.live.wait(Duration::from_secs(12));
    }
    // ATC scripts: the airport layouts (taxiways, holds) complete first.
    if args.audio_script.contains("atc") {
        let t1 = Instant::now();
        while !s.world.airport_detail.idle() && t1.elapsed() < Duration::from_secs(60) {
            pump(&mut s);
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    pump(&mut s);
    {
        let sim = s.sim.as_mut().unwrap();
        if !args.watch || std::env::args().any(|a| a == "--cam") {
            sim.cam = args.cam;
        }
        if let Some(t) = args.throttle {
            sim.ctl.throttle = t;
        }
        if let Some(g) = args.gear {
            sim.ctl.gear_down = g;
        }
        if let Some(f) = args.flaps {
            sim.set_flap_detent(f);
        }
        if args.ap && !sim.ac.on_ground {
            sim.ap.engage_hold_current(&sim.ac);
        }
        apply_systems_args(&args, sim);
        apply_route(&args, sim, &s.world);
        sim.warp = args.warp;
    }
    // Optional silent fast-forward to a bot phase (e.g. "Final").
    if let Some(phase) = &args.audio_until {
        let mut input = Input::default();
        let mut n = 0usize;
        loop {
            let sim = s.sim.as_mut().unwrap();
            if sim.bot.as_ref().is_some_and(|b| b.phase().contains(phase.as_str())) || n > 60 * 60 * 10 || sim.ac.crashed.is_some() {
                eprintln!("fast-forwarded {:.0} s of sim time to phase {phase}", sim.time);
                break;
            }
            sim.warp = 8;
            sim.update(&s.world, &mut input, 1.0 / 60.0);
            let p = vec2(sim.ac.pos.x, sim.ac.pos.z);
            if s.world.wants_rebase(p) {
                let r = s.world.rebase(p);
                sim.rebase(&r);
            }
            if n % 30 == 0 {
                s.follow();
                pump(&mut s);
            }
            n += 1;
        }
        s.sim.as_mut().unwrap().warp = args.warp;
    }
    let mut cues = audio::render::parse_script(&args.audio_script);
    let mut rec = audio::render::Recorder::new(48_000);
    let mut input = Input::default();
    input.pressed.extend(args.press.iter().copied());
    let dt = 1.0 / 60.0;
    for i in 0..(args.seconds.max(1.0) * 60.0) as usize {
        let t = i as f32 * dt;
        {
            let sim = s.sim.as_mut().unwrap();
            for c in cues.due(t, sim, &s.world) {
                audio::render::apply(&c, sim, &mut input, &s.world);
                rec.mark(c);
            }
            sim.update(&s.world, &mut input, dt);
            let _ = sim.frame(16.0 / 9.0);
            let p = vec2(sim.ac.pos.x, sim.ac.pos.z);
            if s.world.wants_rebase(p) {
                let r = s.world.rebase(p);
                sim.rebase(&r);
            }
            rec.step(sim, &s.world, dt, false);
        }
        if i % 30 == 0 {
            s.follow();
            pump(&mut s);
        }
    }
    let rep = rec.finish(&path)?;
    eprintln!("{rep}");
    // The radio transcript (atc/).
    for l in &s.sim.as_ref().unwrap().atc.log {
        eprintln!("radio: [{}] {}", l.who, l.text);
    }
    let ac = &s.sim.as_ref().unwrap().ac;
    eprintln!("end: alt {:.0} m, ias {:.0} kt, on ground {}, crashed {:?}", ac.pos.y, ac.airspeed_kt(), ac.on_ground, ac.crashed);
    Ok(())
}


#[cfg(target_arch = "wasm32")]
fn main() {
    use winit::platform::web::EventLoopExtWebSys;
    platform::web::init();
    // A tile worker (web/worker.js) runs this module too: no app there.
    if platform::web::in_worker() {
        return;
    }
    let args = web_args();
    let el = EventLoop::new().expect("event loop");
    el.spawn_app(App { args, run: None, init: None });
}

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    // `--controllers [SECONDS]`: list game controllers and print their inputs (input/).
    if let Some(i) = std::env::args().position(|a| a == "--controllers") {
        input::probe(std::env::args().nth(i + 1).and_then(|s| s.parse().ok()).unwrap_or(10.0));
        return;
    }
    let args = parse_args();
    if args.audio_render.is_some() {
        if let Err(e) = audio_render(args) {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
        return;
    }
    if args.shot.is_some() {
        if let Err(e) = headless(args) {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
        return;
    }
    let el = EventLoop::new().expect("event loop");
    let mut app = App { args, run: None };
    el.run_app(&mut app).expect("run");
}

/// `--fail NAME[@SECONDS],...` and `--cold-dark` (systems/): fail now or arm.
fn apply_systems_args(args: &Args, sim: &mut sim::Sim) {
    let def = sim.def().clone();
    if args.cold_dark {
        sim.cold_dark = true;
        sim.ac.sys = systems::Systems::new(&def, true);
        sim.ctl.throttle = 0.0;
    }
    for spec in &args.fail_names {
        let (name, at) = match spec.split_once('@') {
            Some((n, t)) => (n, t.parse::<f32>().ok()),
            None => (spec.as_str(), None),
        };
        let Some((kind, idx)) = systems::failures::Kind::parse(name) else {
            eprintln!("--fail: unknown failure '{name}' (ids: {})", systems::failures::Kind::ALL.iter().map(|k| k.id()).collect::<Vec<_>>().join(", "));
            continue;
        };
        let f = systems::failures::Failure::new(kind, idx);
        if !kind.applies(&systems::defs::sys_def(&def), &def) {
            eprintln!("--fail: '{name}' does not apply to the {}", def.name);
            continue;
        }
        match at {
            Some(t) => sim.failures.arm(f, systems::failures::Trigger::At(sim.time + t)),
            None => systems::set_failure(&mut sim.ac, &def, f, true),
        }
    }
}
