//! Simulation session: aircraft, controls, autopilot / auto-flight bot,
//! cameras, and per-frame scene data.

use crate::autopilot::Autopilot;
use crate::bot::{self, Bot};
use crate::flight::{self, Aircraft, AircraftDef, Category, Controls, EngineKind, KT};
use crate::gfx::FrameView;
use crate::models::{self, AircraftModel, ModelInputs};
use crate::scene::{MeshData, Runway};
use crate::world::World;
use glam::{vec2, vec3, Mat4, Quat, Vec2, Vec3};
use std::collections::HashSet;
use std::f32::consts::PI;

pub const DT: f32 = 1.0 / 120.0;
/// Moving-map half-widths (m) the zoom keys step through.
pub const MAP_RANGES: [f32; 6] = [1000.0, 2000.0, 3500.0, 6000.0, 10000.0, 16000.0];
/// Spoiler / speed-brake lever detents.
pub const SPOILER_STEPS: [f32; 4] = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CamMode {
    Chase,
    Cockpit,
    Tower,
    Flyby,
    /// Auto-cycling cinematic shots (for watching the auto-flight bot).
    Cinematic,
}

impl CamMode {
    pub fn name(self) -> &'static str {
        match self {
            CamMode::Chase => "Chase",
            CamMode::Cockpit => "Cockpit",
            CamMode::Tower => "Tower",
            CamMode::Flyby => "Fly-by",
            CamMode::Cinematic => "Cinematic",
        }
    }
    fn next(self) -> CamMode {
        match self {
            CamMode::Chase => CamMode::Cockpit,
            CamMode::Cockpit => CamMode::Tower,
            CamMode::Tower => CamMode::Flyby,
            CamMode::Flyby => CamMode::Cinematic,
            CamMode::Cinematic => CamMode::Chase,
        }
    }
    pub fn parse(s: &str) -> Option<CamMode> {
        Some(match s {
            "chase" => CamMode::Chase,
            "cockpit" => CamMode::Cockpit,
            "tower" => CamMode::Tower,
            "flyby" => CamMode::Flyby,
            "cinematic" | "cine" => CamMode::Cinematic,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Start {
    Runway,
    Approach,
    City,
    Thermal,
}

impl Start {
    pub const ALL: [Start; 4] = [Start::Runway, Start::Approach, Start::City, Start::Thermal];
    /// Generic label (see [`Start::label_for`] for the per-type wording).
    pub fn label(self) -> &'static str {
        match self {
            Start::Runway => "Runway, ready for take-off",
            Start::Approach => "Final approach",
            Start::City => "Over the city",
            Start::Thermal => "Released at 650 m near a thermal",
        }
    }
    pub fn label_for(self, def: &AircraftDef) -> String {
        let jet = def.engine_kind == EngineKind::Jet;
        match (self, def.category) {
            (Start::Runway, Category::Helicopter) => "Helipad, rotors turning".into(),
            (Start::Runway, Category::Drone) => "On the ground, armed".into(),
            (Start::Runway, _) => "Runway, ready for take-off".into(),
            (Start::Approach, Category::Helicopter) => "1 nm final at 300 ft".into(),
            (Start::City, Category::Helicopter) => "Over the city at 1,000 ft".into(),
            (Start::City, Category::Drone) => "Over the city, 60 m up".into(),
            (Start::Approach, Category::Airliner) => "10 nm final at 3,000 ft".into(),
            (Start::Approach, Category::FastJet) => "6 nm final at 2,000 ft".into(),
            (Start::Approach, Category::Glider) => "Final glide, 4.5 km out".into(),
            (Start::Approach, _) => "Final approach, 2.5 nm out".into(),
            (Start::City, Category::FastJet) => "Over the city at 3,000 m".into(),
            (Start::City, Category::Airliner) => format!("Over the city at 3,000 ft{}", if jet { "" } else { "" }),
            (Start::City, Category::Warbird) => "Over the city at 2,000 ft".into(),
            (Start::City, Category::Glider) => "Released over the city at 900 m".into(),
            (Start::City, _) => "Over the city, 400 m".into(),
            (Start::Thermal, _) => "Released at 650 m near a thermal".into(),
        }
    }
    /// Starts that make sense for a type (a glider can't take off alone; only gliders get a thermal release).
    pub fn valid_for(self, def: &AircraftDef) -> bool {
        match self {
            Start::Runway => def.has_engine,
            Start::Thermal => !def.has_engine,
            _ => true,
        }
    }
    pub fn default_for(def: &AircraftDef) -> Start {
        if def.has_engine { Start::Runway } else { Start::Thermal }
    }
    pub fn parse(s: &str) -> Option<Start> {
        match s {
            "runway" => Some(Start::Runway),
            "approach" | "final" => Some(Start::Approach),
            "city" => Some(Start::City),
            "thermal" => Some(Start::Thermal),
            _ => None,
        }
    }
}

/// Keyboard/mouse/controller state fed in by the window layer (or a script).
#[derive(Default)]
pub struct Input {
    pub held: HashSet<Key>,
    pub pressed: Vec<Key>,
    pub orbit_delta: Vec2,
    pub zoom_delta: f32,
    /// Actions held on game controllers (input/).
    pub pad_held: HashSet<Key>,
    /// Analog axes from controllers / the mouse yoke (input/).
    pub analog: crate::input::Analog,
    /// Head-look settings, and whether the look / orbit drag is in progress.
    pub mouse: crate::input::MouseCfg,
    pub dragging: bool,
}

#[path = "input/sim_glue.rs"]
pub mod pilot;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Key {
    PitchDown,
    PitchUp,
    RollLeft,
    RollRight,
    YawLeft,
    YawRight,
    ThrottleUp,
    ThrottleDown,
    FlapsDown,
    FlapsUp,
    Brakes,
    TrimDown,
    TrimUp,
    Camera,
    Reset,
    Pause,
    Map,
    Help,
    Menu,
    Screenshot,
    TimeWarp,
    MapZoomIn,
    MapZoomOut,
    Gear,
    SpoilerOut,
    SpoilerIn,
    Reverse,
    ApToggle,
    ApHdgLeft,
    ApHdgRight,
    ApAltUp,
    ApAltDown,
    ApSpdUp,
    ApSpdDown,
    EngineFail,
    AutoFlight,
    /// Landing / taxi lights on / off (L).
    LandingLights,
    /// Acknowledge the master warning / caution (silence the chime).
    MasterAck,
    /// Systems & failures panel (interface).
    Failures,
    /// Glance while held (hat / keypad), centre the view (input/).
    LookLeft,
    LookRight,
    LookUp,
    LookDown,
    LookCenter,
    /// Mouse yoke on / off; interface toggles handled by the window layer.
    MouseYoke,
    /// Rotorcraft: stability assist (helicopter) / flight mode (drone).
    Assist,
    Instruments,
    WeatherPanel,
    Mute,
    /// Autopilot: LOC / VOR track (NAV 1), approach (LOC + G/S), LNAV, VNAV.
    ApLoc,
    ApAppr,
    ApLnav,
    ApVnav,
    /// NAV 1: swap active / standby, course (OBS) −/+, ident audio.
    Nav1Swap,
    ObsDec,
    ObsInc,
    Nav1Ident,
    /// Radio & flight-plan panel (interface).
    RadioPanel,
    /// Navigation display mode (ROSE ILS / VOR / NAV, ARC, PLAN).
    NdMode,
    /// ATC comms menu (interface); next / previous / select its rows (controllers).
    CommsMenu,
    CommsNext,
    CommsPrev,
    CommsSelect,
}

impl Key {
    /// Keys whose auto-repeat should register as repeated presses.
    pub fn repeats(self) -> bool {
        matches!(self, Key::ApHdgLeft | Key::ApHdgRight | Key::ApAltUp | Key::ApAltDown | Key::ApSpdUp | Key::ApSpdDown | Key::ObsDec | Key::ObsInc)
    }
}

/// A runway long enough for an aircraft type, or where to find one.
pub enum FieldPick {
    /// A loaded runway (with an advisory if it is short or not the nearest field).
    Here(Runway, Option<String>),
    /// A known airfield whose tiles are not loaded yet.
    Remote(Vec2, String),
}

/// Airfields with long runways the spawn logic may move an aircraft to when the
/// nearest field is too short: (ICAO, name, lat, lon, usable length m).
const KNOWN_FIELDS: &[(&str, &str, f64, f64, f32)] = &[
    ("EGLC", "London City", 51.5048, 0.0495, 1508.0),
    ("EGLL", "Heathrow", 51.4700, -0.4543, 3902.0),
    ("EGKK", "Gatwick", 51.1537, -0.1821, 3316.0),
    ("EGSS", "Stansted", 51.8850, 0.2350, 3048.0),
    ("EGGW", "Luton", 51.8747, -0.3683, 2162.0),
    ("EGLF", "Farnborough", 51.2758, -0.7763, 2440.0),
    ("EGMC", "Southend", 51.5714, 0.6956, 1856.0),
    ("EGKB", "Biggin Hill", 51.3308, 0.0325, 1548.0),
    ("EGWU", "Northolt", 51.5530, -0.4182, 1684.0),
];

/// Runway length an aircraft type needs (m).
pub fn min_runway(def: &AircraftDef) -> f32 {
    if def.min_runway_m > 0.0 {
        return def.min_runway_m;
    }
    match def.category {
        Category::Airliner => 1800.0,
        Category::FastJet => 1400.0,
        Category::Warbird => 700.0,
        Category::Trainer => 450.0,
        Category::Glider => 300.0,
        Category::Helicopter | Category::Drone => 25.0,
    }
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Pick the runway an aircraft type starts from: the nearest airfield if it is
/// long enough, else the nearest suitable one among the loaded runways and the
/// known airfields, else the nearest anyway (with a warning).
pub fn pick_field(world: &World, def: &AircraftDef) -> Option<FieldPick> {
    let need = min_runway(def);
    // The start point (the origin of the home frame; the floating origin may have moved).
    let home = world.home_pos();
    let main = world.spawn_runway(home).cloned();
    if let Some(m) = &main {
        if m.length() >= need {
            return Some(FieldPick::Here(m.clone(), None));
        }
    }
    let here = main.as_ref().map(|m| bot::field_name(world, m)).unwrap_or_else(|| "the nearest airfield".into());
    let short = main.as_ref().map(|m| m.length()).unwrap_or(0.0);
    let why = |to: &str, d: f32| {
        format!(
            "{} needs a {:.0} m runway; {} ({:.0} m) is too short — starting at {} ({:.0} km away).",
            def.name,
            need,
            here,
            short,
            to,
            d / 1000.0
        )
    };
    // Loaded runways nearby.
    if let Some(r) = world
        .runways
        .iter()
        .filter(|r| r.length() >= need && seg_dist(home, r.a, r.b) < 15_000.0)
        .min_by(|a, b| seg_dist(home, a.a, a.b).total_cmp(&seg_dist(home, b.a, b.b)))
    {
        let d = seg_dist(home, r.a, r.b);
        return Some(FieldPick::Here(r.clone(), Some(why(&bot::field_name(world, r), d))));
    }
    // Known long-runway airfields within ~90 km.
    let mut known: Vec<(Vec2, &str, &str)> = KNOWN_FIELDS
        .iter()
        .filter(|f| f.4 >= need)
        .map(|f| (world.proj.to_world(f.2, f.3), f.0, f.1))
        .filter(|f| f.0.distance(home) < 90_000.0)
        .collect();
    known.sort_by(|a, b| a.0.distance(home).total_cmp(&b.0.distance(home)));
    if let Some((p, icao, name)) = known.first() {
        let label = format!("{name} ({icao})");
        if let Some(r) = world.runways.iter().filter(|r| r.length() >= need * 0.9 && seg_dist(*p, r.a, r.b) < 3000.0).max_by(|a, b| a.length().total_cmp(&b.length())) {
            return Some(FieldPick::Here(r.clone(), Some(why(&label, p.distance(home)))));
        }
        return Some(FieldPick::Remote(*p, why(&label, p.distance(home))));
    }
    main.map(|m| {
        let msg = format!("No runway of {:.0} m known nearby: {} is only {:.0} m — it will be tight.", need, here, m.length());
        FieldPick::Here(m, Some(msg))
    })
}

pub struct Sim {
    pub defs: Vec<AircraftDef>,
    pub sel: usize,
    pub start: Start,
    pub ac: Aircraft,
    pub ctl: Controls,
    flap_idx: usize,
    pub spoiler_idx: usize,
    pub cam: CamMode,
    orbit_yaw: f32,
    orbit_pitch: f32,
    orbit_dist: f32,
    cam_heading: f32,
    flyby_pos: Option<Vec3>,
    pub paused: bool,
    pub show_map: bool,
    pub show_help: bool,
    pub time: f32,
    /// Local clock time of day (hours) and day of year for the sun position.
    pub hours: f32,
    pub day_of_year: u32,
    /// Where the aircraft is (lat, lon degrees; kept up to date by the app),
    /// for the sun position.
    pub lat_lon: (f64, f64),
    /// The start point: its civil time zone is the clock's (`hours`).
    pub home_ll: (f64, f64),
    pub warp: u32,
    acc: f32,
    model: AircraftModel,
    /// Bounding radius of the model (m): drives camera distances.
    pub size: f32,
    pub last_view: Option<FrameView>,
    /// Runway used for the current start (also anchors the tower camera).
    pub runway: Option<Runway>,
    /// Set when the chosen airfield's scenery is not loaded yet: re-spawn once it is.
    pub needs_respawn: bool,
    /// Moving-map zoom: index into MAP_RANGES.
    pub map_zoom: usize,
    /// Test harness: (heading rad, altitude m) to hold straight and level.
    pub level_hold: Option<(f32, f32)>,
    pub ap: Autopilot,
    /// Seconds left of the flashing "AP DISCONNECT" warning.
    pub ap_warn: f32,
    pub bot: Option<Bot>,
    /// Watch mode (auto flight from the menu): cinematic camera, auto relaunch.
    pub watch: bool,
    /// Transient advisory shown on the HUD: (text, seconds left).
    pub note: Option<(String, f32)>,
    /// Engine-failure practice step (0 = all running).
    pub fail_step: u8,
    /// A start picked in the departure screen (runway end, final, apron,
    /// air); restarts return here. None = the generic `start`.
    pub custom_start: Option<crate::startpos::CustomStart>,
    cine: Cine,
    real_time: f32,
    /// Ground height under the aircraft (terrain.rs), for heights above the ground.
    pub ground_y: f32,
    /// Altimeter setting for the cockpit instruments (the app copies the weather's QNH), hPa.
    pub qnh_hpa: f32,
    /// Physics state one fixed step back, for render interpolation.
    prev_pos: Vec3,
    prev_rot: Quat,
    /// Interpolated pose used by this frame's cameras and aircraft mesh.
    rpos: Vec3,
    rrot: Quat,
    /// Where the model's lights are (lights.rs).
    light_rig: crate::lights::AircraftLightRig,
    /// Landing / taxi lights: None = automatic (gear down / with the bot), Some = the pilot's choice (L).
    pub landing_lights: Option<bool>,
    /// Analog-input and head-look state (input/sim_glue.rs).
    pub pilot: pilot::Pilot,
    /// Loose parts and, after a crash, the wreck (debris.rs).
    pub debris: Option<crate::debris::Debris>,
    /// How bad the accident was (set when the aircraft crashes).
    pub severity: Option<crate::damage::Severity>,
    /// Sounds raised this frame by the wreckage (the audio director reads them).
    pub audio_events: Vec<crate::audio::params::Event>,
    /// Parts already handed to the debris simulation.
    known_lost: u16,
    /// Real seconds since the crash (slow motion, crash camera).
    crash_clock: f32,
    /// Scripted test flight (`SKYWARD_SCENARIO`, scenario.rs).
    pub scenario: Option<crate::scenario::Scenario>,
    /// Particles and atmospheric effects (vfx/).
    pub vfx: crate::vfx::Vfx,
    /// Armed / random failures (systems/failures.rs); the active ones live in `ac.sys`.
    pub failures: crate::systems::failures::Schedule,
    /// Start cold and dark (engines off, battery off) instead of ready to go.
    pub cold_dark: bool,
    /// Radios, flight plan and the guidance the autopilot couples to (nav/).
    pub nav: crate::nav::NavState,
    /// Air traffic control on the radios (atc/).
    pub atc: crate::atc::Atc,
}

/// Cinematic camera state.
#[derive(Clone, Copy, Debug, Default)]
struct Cine {
    shot: u32,
    until: f32,
    yaw: f32,
    pitch: f32,
    dist: f32,
    spin: f32,
    fixed: Option<Vec3>,
}

/// Default time of day: a mid-August London afternoon.
pub const DEFAULT_HOURS: f32 = 15.5;
pub const DEFAULT_DAY: u32 = 227;

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

fn wrap(mut d: f32) -> f32 {
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

/// Typical cruise IAS (m/s) for air starts.
fn cruise_ias(def: &AircraftDef) -> f32 {
    KT * match def.category {
        Category::Trainer => 100.0,
        Category::Glider => 55.0,
        Category::Warbird => 230.0,
        Category::FastJet => 350.0,
        Category::Airliner => {
            if def.engine_kind == EngineKind::Jet {
                250.0
            } else {
                200.0
            }
        }
        Category::Helicopter => 110.0,
        Category::Drone => 25.0,
    }
}

impl Sim {
    pub fn new(world: &World, sel: usize, start: Start) -> Sim {
        let defs = flight::catalogue();
        let sel = sel.min(defs.len() - 1);
        let (model, size) = load_model(&defs[sel]);
        let ac = Aircraft::airborne(&defs[sel], vec3(0.0, 500.0, 0.0), 0.0, 60.0);
        let mut s = Sim {
            defs,
            sel,
            start,
            ac,
            ctl: Controls::default(),
            flap_idx: 0,
            spoiler_idx: 0,
            cam: CamMode::Chase,
            orbit_yaw: 0.0,
            orbit_pitch: 10f32.to_radians(),
            orbit_dist: 0.0,
            cam_heading: 0.0,
            flyby_pos: None,
            paused: false,
            show_map: true,
            show_help: false,
            time: 0.0,
            hours: DEFAULT_HOURS,
            day_of_year: DEFAULT_DAY,
            lat_lon: (world.proj.lat0, world.proj.lon0),
            home_ll: (world.home.lat0, world.home.lon0),
            warp: 1,
            acc: 0.0,
            model,
            size,
            last_view: None,
            runway: None,
            needs_respawn: false,
            map_zoom: 2,
            level_hold: None,
            ap: Autopilot::new(),
            ap_warn: 0.0,
            bot: None,
            watch: false,
            note: None,
            fail_step: 0,
            custom_start: None,
            cine: Cine::default(),
            real_time: 0.0,
            ground_y: 0.0,
            qnh_hpa: 1013.25,
            prev_pos: Vec3::ZERO,
            prev_rot: Quat::IDENTITY,
            rpos: Vec3::ZERO,
            rrot: Quat::IDENTITY,
            light_rig: Default::default(),
            landing_lights: None,
            pilot: Default::default(),
            debris: None,
            severity: None,
            audio_events: Vec::new(),
            known_lost: 0,
            crash_clock: 0.0,
            scenario: crate::scenario::Scenario::from_env(),
            vfx: crate::vfx::Vfx::default(),
            failures: Default::default(),
            cold_dark: false,
            nav: Default::default(),
            atc: Default::default(),
        };
        s.nav = crate::nav::NavState::new(&s.defs[sel]);
        s.atc = crate::atc::Atc::new(&s.defs[sel]);
        s.light_rig = crate::lights::AircraftLightRig::from_model(&s.model);
        s.reset(world);
        s
    }

    pub fn def(&self) -> &AircraftDef {
        &self.defs[self.sel]
    }

    /// Switch aircraft / start. `watch` hands it straight to the auto-flight bot with the cinematic camera.
    pub fn select(&mut self, world: &World, sel: usize, start: Start, watch: bool) {
        self.sel = sel.min(self.defs.len() - 1);
        self.start = if start.valid_for(&self.defs[self.sel]) { start } else { Start::default_for(&self.defs[self.sel]) };
        (self.model, self.size) = load_model(&self.defs[self.sel]);
        self.light_rig = crate::lights::AircraftLightRig::from_model(&self.model);
        self.landing_lights = None;
        self.bot = None;
        self.nav = crate::nav::NavState::new(&self.defs[self.sel]);
        self.watch = watch;
        if watch {
            self.cam = CamMode::Cinematic;
        } else if self.cam == CamMode::Cinematic {
            self.cam = CamMode::Chase;
        }
        self.reset(world);
    }

    /// Set the flap lever to detent `i` (test harness).
    pub fn set_flap_detent(&mut self, i: usize) {
        let det = self.defs[self.sel].flap_detents;
        self.flap_idx = i.min(det.len().saturating_sub(1));
        self.ctl.flaps = det.get(self.flap_idx).copied().unwrap_or(0.0) / det.last().copied().unwrap_or(1.0).max(1e-3);
    }

    /// Set the spoiler lever (nearest detent; test harness).
    pub fn set_spoilers(&mut self, f: f32) {
        self.spoiler_idx = (0..SPOILER_STEPS.len()).min_by(|&a, &b| (SPOILER_STEPS[a] - f).abs().total_cmp(&(SPOILER_STEPS[b] - f).abs())).unwrap_or(0);
        self.ctl.spoilers = SPOILER_STEPS[self.spoiler_idx];
    }

    pub fn say(&mut self, s: impl Into<String>, secs: f32) {
        self.note = Some((s.into(), secs));
    }

    pub fn reset(&mut self, world: &World) {
        let def = self.defs[self.sel].clone();
        let keep = (self.atc.auto_readback, self.atc.assist, self.atc.bot_comms);
        self.atc = crate::atc::Atc::new(&def);
        (self.atc.auto_readback, self.atc.assist, self.atc.bot_comms) = keep;
        if !self.start.valid_for(&def) {
            self.start = Start::default_for(&def);
        }
        // Re-pick the runway: more pieces may have merged since the last spawn.
        self.needs_respawn = false;
        let mut advice = None;
        self.runway = match pick_field(world, &def) {
            Some(FieldPick::Here(r, note)) => {
                advice = note;
                Some(r)
            }
            Some(FieldPick::Remote(p, note)) => {
                // Only runway / approach starts have to wait for the remote field to load.
                if matches!(self.start, Start::Runway | Start::Approach) {
                    advice = Some(note);
                    self.needs_respawn = true;
                    // Provisional: parked at the airfield until its scenery has loaded.
                    Some(Runway { a: p - vec2(0.0, 900.0), b: p + vec2(0.0, 900.0), width: 45.0, name: String::new(), paved: true, ..Default::default() })
                } else {
                    world.spawn_runway(world.home_pos()).cloned()
                }
            }
            None => None,
        };
        let custom = self.custom_start.as_ref().and_then(|c| crate::startpos::spawn(&def, world, c));
        let (ac, ctl, flap_idx) = match custom {
            Some((ac, ctl, flap, rw)) => {
                if rw.is_some() {
                    self.runway = rw;
                }
                self.needs_respawn = false;
                advice = None;
                (ac, ctl, flap)
            }
            None => Self::spawn(&def, world, self.runway.as_ref(), self.start),
        };
        self.ac = ac;
        self.ctl = ctl;
        self.flap_idx = flap_idx;
        // Systems (systems/): ready to go, or cold and dark; armed failures are cleared.
        self.failures.clear();
        if self.cold_dark {
            self.ac.sys = crate::systems::Systems::new(&def, true);
            self.ctl.throttle = 0.0;
        }
        self.spoiler_idx = 0;
        self.fail_step = 0;
        self.ap = Autopilot::new();
        self.ap_warn = 0.0;
        self.setup_nav(world);
        self.orbit_dist = self.chase_dist();
        // Screenshot knob: SKYWARD_ORBIT="yaw_deg,pitch_deg,dist_m" sets the chase camera.
        if let Ok(o) = std::env::var("SKYWARD_ORBIT") {
            let v: Vec<f32> = o.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            if v.len() == 3 {
                self.orbit_yaw = v[0].to_radians();
                self.orbit_pitch = v[1].to_radians();
                if v[2] > 0.0 {
                    self.orbit_dist = v[2];
                }
            }
        }
        self.cam_heading = self.ac.heading();
        self.flyby_pos = None;
        self.cine = Cine::default();
        self.acc = 0.0;
        self.debris = None;
        self.severity = None;
        self.known_lost = 0;
        self.crash_clock = 0.0;
        self.audio_events.clear();
        self.vfx.clear();
        if let Some(sc) = self.scenario.as_mut() {
            sc.restart();
        }
        self.snap_pose();
        self.note = advice.map(|a| (a, 9.0));
        if self.watch && !self.needs_respawn {
            self.enable_bot(world);
        }
    }

    fn chase_dist(&self) -> f32 {
        if self.defs[self.sel].category == Category::Drone {
            return 1.8;
        }
        self.size * 2.6 + 6.0
    }

    /// Where a start puts the aircraft (used to pre-load tiles before spawning).
    pub fn spawn_point(world: &World, start: Start, sel: usize) -> Vec2 {
        let defs = flight::catalogue();
        let def = &defs[sel.min(defs.len() - 1)];
        let start = if start.valid_for(def) { start } else { Start::default_for(def) };
        let rw = match pick_field(world, def) {
            Some(FieldPick::Here(r, _)) => Some(r),
            Some(FieldPick::Remote(p, _)) if matches!(start, Start::Runway | Start::Approach) => return p,
            Some(FieldPick::Remote(..)) => None,
            None => None,
        };
        match start {
            Start::Runway | Start::Approach => match &rw {
                Some(r) => {
                    if start == Start::Runway {
                        // The same end `spawn` picks (wind / climb gradient), so the
                        // tiles under the wheels are loaded before the aircraft lands on them.
                        bot::best_takeoff_end(world, r).thr
                    } else {
                        let (end, _) = bot::best_landing_end(world, r, 200.0);
                        end.thr - end.dir * Self::approach_geometry(def).0
                    }
                }
                None => world.home_pos(),
            },
            Start::City => world.home_pos() + vec2(3400.0, 0.0),
            Start::Thermal => Self::best_thermal(world).unwrap_or(world.home_pos()),
        }
    }

    /// Strongest thermal near the start (under a cumulus of the current weather).
    fn best_thermal(world: &World) -> Option<Vec2> {
        world.clouds.thermals_near(world.home_pos(), 5000.0).into_iter().max_by(|a, b| a.2.total_cmp(&b.2)).map(|t| t.0)
    }

    /// Approach start: (distance from the threshold m, altitude m, IAS m/s, flap detent).
    fn approach_geometry(def: &AircraftDef) -> (f32, f32, f32, usize) {
        let last = def.flap_detents.len().saturating_sub(1);
        match def.category {
            Category::Airliner => (18_520.0, 914.0, (def.vref_kt + 45.0) * KT, 1.min(last)),
            Category::FastJet => (11_100.0, 610.0, (def.vref_kt + 35.0) * KT, 0),
            Category::Glider => (4500.0, 300.0, def.vs1_kt * 1.4 * KT, 0),
            // Rotorcraft: a 1 nm final at 300 ft (drone: 400 m, 30 m up).
            Category::Helicopter => (1852.0, 91.0, def.vref_kt * KT, 0),
            Category::Drone => (400.0, 30.0, 10.0, 0),
            Category::Warbird => (4630.0, 4630.0 * 3f32.to_radians().tan() + 30.0, def.vref_kt * 1.15 * KT, 0),
            Category::Trainer => (4630.0, 4630.0 * 3f32.to_radians().tan() + 20.0, def.vs1_kt * 1.4 * KT, 1.min(last)),
        }
    }

    fn spawn(def: &AircraftDef, world: &World, rw: Option<&Runway>, start: Start) -> (Aircraft, Controls, usize) {
        let air = |pos: Vec3, hdg: f32, v: f32| {
            let ac = Aircraft::airborne(def, pos, hdg, v);
            let mut ctl = flight::trimmed_controls_at(def, v, pos.y);
            ctl.gear_down = ac.gear_pos() > 0.5;
            (ac, ctl)
        };
        match start {
            Start::Runway if def.rotor.is_some() => {
                // Rotorcraft: a clear spot beside the runway (apron / grass), facing into wind.
                let (spot, hdg) = crate::rotor::defs::ground_spot(def, world, rw);
                (Aircraft::on_runway(def, spot, hdg, world), Controls::default(), 0)
            }
            Start::Runway => {
                let (thr, hdg) = match rw {
                    Some(r) => {
                        let end = bot::best_takeoff_end(world, r);
                        (end.thr + end.dir * (25.0 + def.gear.iter().map(|g| g.pos.z.abs()).fold(0.0, f32::max)), end.course())
                    }
                    None => (world.home_pos(), 0.0),
                };
                (Aircraft::on_runway(def, thr, hdg, world), Controls::default(), 0)
            }
            Start::Approach => {
                let (d, alt, v, flap) = Self::approach_geometry(def);
                let (thr, dir) = match rw {
                    Some(r) => {
                        let (end, _) = bot::best_landing_end(world, r, 200.0);
                        (end.thr, end.dir)
                    }
                    None => (world.home_pos(), vec2(0.0, -1.0)),
                };
                let p = thr - dir * d;
                // (Above the threshold / the terrain: terrain.rs.)
                let base = rw.map_or_else(|| crate::flight::Environment::ground_height(world, p), |r| r.height_at(thr));
                let (mut ac, mut ctl) = air(vec3(p.x, alt + base, p.y), heading_of(dir), v);
                if def.has_engine {
                    // A shallow descent with the approach flap set (already extended).
                    ctl.throttle *= 0.75;
                    ctl.flaps = def.flap_detents.get(flap).copied().unwrap_or(0.0) / def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
                    ac.set_flaps_now(ctl.flaps);
                }
                (ac, ctl, flap)
            }
            Start::City => {
                let p = world.home_pos() + vec2(3400.0, 0.0);
                let alt = match def.category {
                    Category::FastJet => 3000.0,
                    Category::Airliner => 914.0,
                    Category::Warbird => 610.0,
                    Category::Glider => 900.0,
                    Category::Trainer => 400.0,
                    Category::Helicopter => 300.0,
                    Category::Drone => 60.0,
                };
                let (ac, ctl) = air(vec3(p.x, alt + crate::flight::Environment::ground_height(world, p), p.y), heading_of(vec2(-1.0, 0.0)), cruise_ias(def));
                (ac, ctl, 0)
            }
            Start::Thermal => {
                let target = Self::best_thermal(world).unwrap_or(world.home_pos());
                let p = target + vec2(500.0, 300.0);
                let v = (def.vs1_kt * 1.4 * KT).max(25.0);
                let (ac, ctl) = air(vec3(p.x, 650.0 + crate::flight::Environment::ground_height(world, p), p.y), heading_of(target - p), v);
                (ac, ctl, 0)
            }
        }
    }

    /// The world origin moved (World::rebase): shift the aircraft, cameras,
    /// bot and runway with it so nothing visibly jumps.
    pub fn rebase(&mut self, r: &crate::world::Rebase) {
        self.ac.rebase(r.delta);
        self.atc.rebase(r);
        self.prev_pos = r.shift(self.prev_pos);
        self.rpos = r.shift(self.rpos);
        if let Some(p) = &mut self.flyby_pos {
            *p = r.shift(*p);
        }
        if let Some(p) = &mut self.cine.fixed {
            *p = r.shift(*p);
        }
        if let Some(rw) = &mut self.runway {
            rw.a = r.ground(rw.a);
            rw.b = r.ground(rw.b);
        }
        if let Some(v) = &mut self.last_view {
            v.eye = r.shift(v.eye);
            v.view *= Mat4::from_translation(-r.delta);
        }
        if let Some(b) = &mut self.bot {
            b.rebase(r);
        }
        if let Some(d) = &mut self.debris {
            d.rebase(r.delta);
        }
        self.vfx.rebase(r.delta);
    }

    /// Hand the aircraft to the auto-flight bot.
    pub fn enable_bot(&mut self, world: &World) {
        let def = &self.defs[self.sel];
        if def.rotor.is_some() {
            // The rotorcraft bot flies through the attitude-hold / angle assist.
            self.ac.rotor.assist = crate::rotor::Assist::Att;
        }
        self.bot = Some(Bot::new(def, &self.ac, world, self.runway.as_ref()));
        self.ap.disengage();
        self.ap_warn = 0.0;
    }

    /// Take control back from the bot (keeps the aircraft as it is).
    pub fn disable_bot(&mut self, why: &str) {
        if self.bot.take().is_some() {
            self.ap.disengage();
            self.ctl.reverse = false;
            // The bot's flap / spoiler settings become the pilot's levers.
            let det = self.defs[self.sel].flap_detents;
            let full = det.last().copied().unwrap_or(1.0).max(1e-3);
            self.flap_idx = (0..det.len()).min_by(|&a, &b| (det[a] / full - self.ctl.flaps).abs().total_cmp(&(det[b] / full - self.ctl.flaps).abs())).unwrap_or(0);
            self.spoiler_idx = (0..SPOILER_STEPS.len())
                .min_by(|&a, &b| (SPOILER_STEPS[a] - self.ctl.spoilers).abs().total_cmp(&(SPOILER_STEPS[b] - self.ctl.spoilers).abs()))
                .unwrap_or(0);
            self.say(format!("Auto flight off — {why}"), 4.0);
        }
    }

    /// Origin / arrival for the flight plan at a (re)start: the airport at
    /// the start; on an approach start the runway being flown to, as an ILS
    /// approach (auto-tuning NAV 1).
    fn setup_nav(&mut self, world: &World) {
        use crate::nav::fms::{ApprKind, Wpt, WptKind};
        self.nav.plan.active = 0;
        self.nav.plan.from = None;
        let Some(adb) = world.airport_db.clone() else { return };
        let p = vec2(self.ac.pos.x, self.ac.pos.z);
        let ll = world.proj.to_latlon(p);
        if let Some(a) = adb.near(ll.0, ll.1, 20.0).into_iter().map(|(i, _)| &adb.airports[i]).find(|a| a.kind != crate::airports::Kind::Seaplane) {
            if self.nav.plan.origin.is_none() || self.ac.on_ground {
                self.nav.plan.set_origin(Some(Wpt::new(a.code(), (a.lat, a.lon), WptKind::Airport)));
            }
        }
        if self.ac.on_ground || self.nav.plan.approach.is_some() {
            return;
        }
        let Some(rw) = self.runway.clone().filter(|r| !r.airport.is_empty()) else { return };
        // The end being flown to: the one the aircraft points along.
        let ha = bot::heading_of(rw.b - rw.a);
        let ident = if bot::wrap_pi(ha - self.ac.heading()).abs() < PI / 2.0 { rw.ident_a.clone() } else { rw.ident_b.clone() };
        if let Ok(msg) = self.nav.plan_approach(&adb, &rw.airport, &ident, ApprKind::Ils) {
            if self.def().category == Category::Airliner || self.nav.equipped {
                eprintln!("nav: {msg}");
            }
        }
    }

    /// Autopilot mode keys: LOC / APPR / LNAV / VNAV.
    fn ap_mode_key(&mut self, k: Key) {
        use crate::autopilot::Lat;
        if !self.nav.equipped {
            self.say("No navigation radios on this aircraft", 2.0);
            return;
        }
        if !self.ap.engaged && !self.ac.on_ground && self.bot.is_none() {
            self.ap.engage_hold_current(&self.ac);
            if !self.def().has_engine {
                self.ap.ias = Some(self.ac.airspeed());
            }
        }
        match k {
            Key::ApLoc => {
                let loc = self.nav.radios.nav[0].out.loc;
                self.ap.toggle_lat(if loc { Lat::Loc } else { Lat::Vor });
            }
            Key::ApAppr => self.ap.toggle_appr(),
            Key::ApLnav => {
                if self.nav.plan.legs.is_empty() {
                    self.say("LNAV: no flight plan (F3)", 2.5);
                } else {
                    self.ap.toggle_lat(Lat::Lnav);
                }
            }
            _ => {
                if self.nav.plan.approach.is_none() {
                    self.say("VNAV: needs an approach in the flight plan (F3)", 2.5);
                } else {
                    self.ap.toggle_vnav();
                }
            }
        }
        let f = self.ap.fma(self.def(), &self.ac);
        let txt: Vec<&str> = [f[3].as_str(), f[4].as_str(), f[1].as_str(), f[2].as_str()].into_iter().filter(|x| !x.is_empty()).collect();
        self.say(txt.join("  ·  "), 2.5);
    }

    /// Advance the simulation by real time `dt` (seconds).
    pub fn update(&mut self, world: &World, input: &mut Input, dt: f32) {
        let dt = dt.min(0.1);
        self.real_time += dt;
        self.ground_y = crate::flight::Environment::ground_height(world, vec2(self.ac.pos.x, self.ac.pos.z));
        if let Some((_, t)) = &mut self.note {
            *t -= dt;
            if *t <= 0.0 {
                self.note = None;
            }
        }
        self.ap_warn = (self.ap_warn - dt).max(0.0);
        let def = self.defs[self.sel].clone();
        // Analog inputs moved enough to take over from the bot / autopilot (input/sim_glue.rs).
        let analog = self.analog_takeover(input);
        let held = |k: Key| input.held.contains(&k) || input.pad_held.contains(&k);
        let axes = [Key::PitchDown, Key::PitchUp, Key::RollLeft, Key::RollRight, Key::YawLeft, Key::YawRight].into_iter().any(held) || analog.stick;
        let throttle_keys = held(Key::ThrottleUp) || held(Key::ThrottleDown) || analog.throttle;
        // Pilot input takes control back from the bot.
        if self.bot.is_some() {
            let levers = input
                .pressed
                .iter()
                .any(|k| matches!(k, Key::FlapsDown | Key::FlapsUp | Key::Gear | Key::SpoilerOut | Key::SpoilerIn | Key::Reverse | Key::ApToggle));
            if axes || throttle_keys || levers || analog.levers || held(Key::Brakes) {
                self.disable_bot("you have control");
                self.watch = false;
                if self.cam == CamMode::Cinematic {
                    self.cam = CamMode::Chase;
                }
                // Don't also act on the lever press that took control.
                input.pressed.retain(|k| !matches!(k, Key::ApToggle));
            }
        }
        // ...and disconnects the autopilot (with a warning), or the autothrottle.
        if self.bot.is_none() && self.ap.engaged {
            if axes {
                self.ap.disengage();
                self.ap_warn = 4.0;
                self.say("AP disconnected by pilot input", 3.0);
            } else if throttle_keys && self.ap.ias.is_some() {
                self.ap.ias = None;
                self.say("A/THR off — manual throttle", 3.0);
            }
        }
        for k in std::mem::take(&mut input.pressed) {
            match k {
                Key::Camera => {
                    self.cam = self.cam.next();
                    self.flyby_pos = None;
                    self.cine = Cine::default();
                }
                Key::Reset => {
                    self.watch = false;
                    self.bot = None;
                    self.reset(world);
                }
                Key::MapZoomIn => self.map_zoom = self.map_zoom.saturating_sub(1),
                Key::MapZoomOut => self.map_zoom = (self.map_zoom + 1).min(MAP_RANGES.len() - 1),
                Key::Pause => self.paused = !self.paused,
                Key::Map => self.show_map = !self.show_map,
                Key::Help => self.show_help = !self.show_help,
                Key::TimeWarp => self.warp = if self.warp >= 8 { 1 } else { self.warp * 2 },
                Key::FlapsDown => {
                    let n = def.flap_detents.len();
                    self.flap_idx = (self.flap_idx + 1).min(n.saturating_sub(1));
                }
                Key::FlapsUp => self.flap_idx = self.flap_idx.saturating_sub(1),
                Key::Gear | Key::Assist if def.rotor.is_some() => {
                    let quad = def.category == Category::Drone;
                    self.ac.rotor.assist = self.ac.rotor.assist.next(quad);
                    let what = if quad { "Flight mode" } else { "Stability assist" };
                    self.say(format!("{what}: {}", self.ac.rotor.assist.label(quad)), 2.5);
                }
                Key::Gear => {
                    if def.retractable_gear {
                        self.ctl.gear_down = !self.ctl.gear_down;
                        if !self.ctl.gear_down && self.ac.on_ground {
                            self.say("Gear lever up — the gear won't retract with weight on the wheels", 3.0);
                        }
                    } else {
                        self.say("Fixed landing gear", 2.0);
                    }
                }
                Key::SpoilerOut | Key::SpoilerIn => {
                    if def.has_spoilers {
                        self.spoiler_idx = if k == Key::SpoilerOut {
                            (self.spoiler_idx + 1).min(SPOILER_STEPS.len() - 1)
                        } else {
                            self.spoiler_idx.saturating_sub(1)
                        };
                    } else if !def.has_engine {
                        self.say("Glider airbrakes: F / V", 2.0);
                    } else {
                        self.say("No spoilers on this aircraft", 2.0);
                    }
                }
                Key::Reverse => {
                    if !def.has_reverser {
                        self.say("No thrust reversers on this aircraft", 2.0);
                    } else if !self.ac.on_ground && !self.ctl.reverse {
                        self.say("Reverse thrust: on the ground only", 2.0);
                    } else {
                        self.ctl.reverse = !self.ctl.reverse;
                        if self.ctl.reverse {
                            self.ctl.throttle = 0.0;
                        }
                    }
                }
                Key::ApToggle => {
                    if self.ap.engaged {
                        self.ap.disengage();
                        self.say("Autopilot off", 2.0);
                    } else if self.ac.on_ground {
                        self.say("Autopilot: not on the ground", 2.0);
                    } else {
                        self.ap.engage_hold_current(&self.ac);
                        if !def.has_engine {
                            self.ap.ias = Some(self.ac.airspeed());
                        }
                        self.ap_warn = 0.0;
                    }
                }
                Key::ApHdgLeft | Key::ApHdgRight => {
                    let h = self.ap.hdg.unwrap_or(self.ac.heading());
                    let step = if k == Key::ApHdgRight { 5f32 } else { -5f32 }.to_radians();
                    self.ap.hdg = Some(wrap(h + step));
                }
                Key::ApAltUp | Key::ApAltDown => {
                    let a = self.ap.alt.unwrap_or(self.ac.pos.y);
                    let step = 100.0 * 0.3048 * if k == Key::ApAltUp { 1.0 } else { -1.0 };
                    self.ap.alt = Some(((a + step) / 30.48).round() * 30.48);
                    self.ap.vs = None;
                }
                Key::ApSpdUp | Key::ApSpdDown => {
                    if def.has_engine || self.ap.ias.is_some() {
                        let v = self.ap.ias.unwrap_or(self.ac.airspeed());
                        let step = 5.0 * KT * if k == Key::ApSpdUp { 1.0 } else { -1.0 };
                        self.ap.ias = Some((v + step).max(def.vs1_kt * KT));
                    }
                }
                Key::ApLoc | Key::ApAppr | Key::ApLnav | Key::ApVnav => self.ap_mode_key(k),
                Key::Nav1Swap => {
                    self.nav.radios.nav[0].swap();
                    let f = self.nav.radios.nav[0].active;
                    self.say(format!("NAV 1 {}", crate::nav::db::freq_label(f, false)), 2.0);
                }
                Key::ObsDec | Key::ObsInc => {
                    let step = if k == Key::ObsInc { 1f32 } else { -1f32 }.to_radians();
                    let rx = &mut self.nav.radios.nav[0];
                    rx.obs = crate::nav::geo::wrap_2pi((rx.obs.to_degrees().round()).to_radians() + step);
                }
                Key::Nav1Ident => {
                    let rx = &mut self.nav.radios.nav[0];
                    rx.ident_audio = !rx.ident_audio;
                    let on = rx.ident_audio;
                    self.say(if on { "NAV 1 ident audio on" } else { "NAV 1 ident audio off" }, 2.0);
                }
                Key::NdMode => self.nav.nd_mode = self.nav.nd_mode.next(),
                Key::EngineFail => self.cycle_engine_failure(),
                Key::MasterAck => self.ac.sys.acknowledge(),
                Key::LandingLights => {
                    let on = !self.landing_lights_on();
                    self.landing_lights = Some(on);
                    self.say(if on { "Landing lights on" } else { "Landing lights off" }, 2.0);
                }
                Key::LookCenter => {
                    self.pilot.free = Vec2::ZERO;
                    if self.cam == CamMode::Chase {
                        self.orbit_yaw = 0.0;
                    }
                }
                Key::AutoFlight => {
                    if self.bot.is_some() {
                        self.disable_bot("you have control");
                        self.watch = false;
                    } else if self.ac.crashed.is_none() {
                        self.enable_bot(world);
                        self.say("Auto flight on — any control input takes over (J to hand back)", 4.0);
                    }
                }
                _ => {}
            }
        }
        // Camera orbit / head-look (input/sim_glue.rs), zoom.
        self.update_look(input, dt);
        if input.zoom_delta != 0.0 {
            let min = if def.category == Category::Drone { 0.8 } else { (self.size * 1.2).max(6.0) };
            self.orbit_dist = (self.orbit_dist * (1.0 - input.zoom_delta * 0.1)).clamp(min, 600.0);
            input.zoom_delta = 0.0;
        }

        // Controls with keyboard smoothing (skipped for what the AP / bot is flying).
        let bot_on = self.bot.is_some();
        let h = |k: Key| if input.held.contains(&k) || input.pad_held.contains(&k) { 1.0 } else { 0.0 };
        let approach = |cur: f32, target: f32, rate: f32| {
            let r = if target.abs() < 1e-3 { rate * 1.6 } else { rate };
            cur + (target - cur).clamp(-r * dt, r * dt)
        };
        if !bot_on && !self.ap.engaged {
            self.ctl.elevator = approach(self.ctl.elevator, h(Key::PitchUp) - h(Key::PitchDown), 2.2);
            self.ctl.aileron = approach(self.ctl.aileron, h(Key::RollRight) - h(Key::RollLeft), 3.0);
            self.ctl.rudder = approach(self.ctl.rudder, h(Key::YawRight) - h(Key::YawLeft), 2.5);
            self.ctl.trim = (self.ctl.trim + (h(Key::TrimUp) - h(Key::TrimDown)) * 0.25 * dt).clamp(-1.0, 1.0);
        }
        if def.has_engine && !bot_on && !(self.ap.engaged && self.ap.ias.is_some()) {
            self.ctl.throttle = (self.ctl.throttle + (h(Key::ThrottleUp) - h(Key::ThrottleDown)) * 0.6 * dt).clamp(0.0, 1.0);
        }
        // Controllers: analog axes straight to the controls, levers when moved.
        self.apply_analog(input, dt);
        if !bot_on {
            self.ctl.brakes = h(Key::Brakes).max(pilot::toe_brakes(&input.analog));
            let det = def.flap_detents;
            let full = det.last().copied().unwrap_or(1.0).max(1e-3);
            let target_flaps = det.get(self.flap_idx).copied().unwrap_or(0.0) / full;
            self.ctl.flaps = approach(self.ctl.flaps, target_flaps, 0.35);
            self.ctl.spoilers = approach(self.ctl.spoilers, SPOILER_STEPS[self.spoiler_idx], 1.5);
        } else {
            let det = def.flap_detents;
            let full = det.last().copied().unwrap_or(1.0).max(1e-3);
            self.flap_idx = (0..det.len()).min_by(|&a, &b| (det[a] / full - self.ctl.flaps).abs().total_cmp(&(det[b] / full - self.ctl.flaps).abs())).unwrap_or(0);
        }
        if let Some((hdg, alt)) = self.level_hold {
            // Crude wings-level / heading / altitude hold for scripted test flights.
            let e = wrap(hdg - self.ac.heading());
            let roll_cmd = (e * 1.5).clamp(-0.35, 0.35);
            self.ctl.aileron = ((roll_cmd - self.ac.roll()) * 2.5).clamp(-1.0, 1.0);
            let vs_cmd = ((alt - self.ac.pos.y) * 0.08).clamp(-4.0, 4.0);
            let pitch_cmd = (vs_cmd / self.ac.airspeed().max(20.0)).clamp(-0.15, 0.2) + 0.03;
            self.ctl.elevator = ((pitch_cmd - self.ac.pitch()) * 4.0).clamp(-1.0, 1.0);
        }

        if !self.paused && !self.needs_respawn {
            crate::scenario::control(self, world, dt);
        }
        self.audio_events.clear();
        let amb = crate::systems::ambient(world, &self.ac, self.ground_y);
        if !self.paused && self.ac.crashed.is_none() && !self.needs_respawn {
            self.acc += dt * self.warp as f32;
            let mut steps = 0;
            let ndb = crate::nav::db::db();
            let env = crate::nav::state::WorldEnv(world);
            while self.acc >= DT && steps < 256 {
                let def = &self.defs[self.sel];
                // Radios and flight plan, then the autopilot's nav inputs (nav/).
                self.nav.step(&env, &self.ac, def, ndb.as_deref(), world.airport_db.as_deref(), DT);
                self.ap.nav = self.nav.ap_input(&env, &self.ac, def, self.ground_y);
                if let Some(b) = self.bot.as_mut() {
                    b.update(def, &self.ac, &mut self.ctl, &mut self.ap, world, DT);
                } else if def.rotor.is_some() && self.ap.engaged {
                    // Rotorcraft autopilot: collective for height, pitch for speed (src/rotor/bot.rs).
                    if !crate::rotor::bot::autopilot(&mut self.ap, def, &self.ac, &mut self.ctl, DT) {
                        self.ap.disengage();
                        self.note = Some(("Autopilot disengaged".into(), 3.0));
                    }
                } else if self.ap.engaged && !self.ap.update(def, &self.ac, &mut self.ctl, DT) {
                    self.ap_warn = 4.0;
                    let why = if self.ap.at_minimums { "Minimums — autopilot disconnected, you have control" } else { "Autopilot disengaged" };
                    self.note = Some((why.into(), 3.0));
                    self.ap.at_minimums = false;
                }
                if self.ctl.reverse && !self.ac.on_ground && self.bot.is_none() {
                    self.ctl.reverse = false;
                }
                self.prev_pos = self.ac.pos;
                self.prev_rot = self.ac.rot;
                // Aircraft systems and failures (systems/), then the flight model.
                self.systems_step(&amb, DT);
                let def = &self.defs[self.sel];
                self.ac.step(def, &self.ctl, world, DT);
                self.acc -= DT;
                steps += 1;
                if self.ac.crashed.is_some() {
                    if let Some(b) = self.bot.as_mut() {
                        b.update(def, &self.ac, &mut self.ctl, &mut self.ap, world, 0.0);
                    }
                    break;
                }
            }
            self.time += dt * self.warp as f32;
        }
        self.systems_frame();
        // Air traffic control: the frequency, ATIS, the controllers (atc/).
        crate::atc::step(self, world, dt);
        // The bot's runway: NAV 1 on its ILS (it couples to it on final, bot.rs).
        if let Some(end) = self.bot.as_ref().and_then(|b| b.focus_runway()) {
            if let Some(rw) = world.runways.iter().find(|r| end.same_runway(r) && !r.airport.is_empty()) {
                let ident = if rw.a.distance(end.thr) < rw.b.distance(end.thr) { &rw.ident_a } else { &rw.ident_b };
                self.nav.tune_runway(&rw.airport, ident);
            }
        }
        // Watch mode: a landed glider is relaunched (as if by winch) after a pause.
        if self.watch && self.bot.as_ref().is_some_and(|b| b.wants_relaunch) {
            self.start = Start::default_for(&def);
            self.reset(world);
            self.say("Relaunched", 3.0);
        }
        if !self.paused {
            self.structures(world, dt);
            self.effects(world, dt);
        }
        if self.ac.crashed.is_some() {
            self.snap_pose();
        }
        // Camera heading lag (chase cam swings round smoothly; frame-rate independent),
        // following the interpolated attitude.
        let (_, rot) = self.pose();
        let f = rot * Vec3::Z;
        let target = if f.x.abs() + f.z.abs() > 1e-3 { f.x.atan2(-f.z) } else { self.ac.heading() };
        let d = wrap(target - self.cam_heading);
        self.cam_heading += d * (1.0 - (-4.0 * dt).exp());
    }

    /// Structural events, loose parts and the wreck (damage.rs, debris.rs).
    fn structures(&mut self, world: &World, dt: f32) {
        let def = self.defs[self.sel].clone();
        let sd = *crate::damage::struct_def(def.id);
        // Messages from the structure (overstress, jammed flaps, separations).
        let events = std::mem::take(&mut self.ac.dmg.events);
        if let Some(e) = events.iter().rev().find(|e| !e.caution).or(events.last()) {
            self.say(e.msg.clone(), 6.0);
        }
        // Parts that came off since the last frame become debris.
        let lost = self.ac.dmg.lost;
        let new = lost & !self.known_lost;
        if new != 0 && self.ac.crashed.is_none() {
            let geom = self.part_geom(&def);
            let d = self.debris.get_or_insert_with(|| crate::debris::Debris::new(&def, geom));
            // One body per torn-off assembly (a part with what hangs on it).
            let mut left = new;
            for p in crate::damage::Part::ALL {
                if left & p.bit() == 0 {
                    continue;
                }
                let parent_new = p.parent(&sd, &def).is_some_and(|q| left & q.bit() != 0 || (new & q.bit() != 0));
                if parent_new {
                    continue;
                }
                let mut group = p.bit();
                for q in crate::damage::Part::ALL {
                    let mut a = q.parent(&sd, &def);
                    while let Some(x) = a {
                        if x == p && new & q.bit() != 0 {
                            group |= q.bit();
                            break;
                        }
                        a = x.parent(&sd, &def);
                    }
                }
                left &= !group;
                d.separate_in_flight(group, self.ac.pos, self.ac.rot, self.ac.vel, self.ac.omega);
            }
            self.known_lost = lost;
        }
        // The crash: the wreck takes over from the frozen flight model.
        if let (Some(reason), Some(imp), None) = (self.ac.crashed.clone(), self.ac.impact, self.severity) {
            let sev = crate::damage::classify(&def, &reason, imp.vel, self.ac.dmg.failure.is_some());
            self.severity = Some(sev);
            self.crash_clock = 0.0;
            let geom = self.part_geom(&def);
            let d = self.debris.get_or_insert_with(|| crate::debris::Debris::new(&def, geom));
            // Parts that exist (with geometry, or gear legs the flight model has).
            let all: u16 = d.geom_has() | def.gear.iter().filter_map(|g| crate::damage::gear_part(&def, g)).fold(0, |m, p| m | p.bit()) | crate::damage::Part::Fuselage.bit();
            // (Parts lost in the crash itself — collapsed gear — are broken off by the wreck.)
            d.crash(&imp, &reason, sev, all & !self.known_lost, self.ac.gear_pos(), world);
            if std::env::var_os("SKYWARD_DEBUG_DEBRIS").is_some() {
                eprintln!("crash: {reason} — {sev:?}, impact v {:?} at {:?}; {} bodies, {} fires", imp.vel, imp.pos, d.bodies.len(), d.fires.len());
                for b in &d.bodies {
                    eprintln!("  body parts {:#06x} mass {:.0} com {:?} pos {:?} pts {}", b.parts, b.mass, b.com, b.pos, b.pts.len());
                }
            }
            self.known_lost = lost;
            if self.cam == CamMode::Cockpit {
                self.cam = CamMode::Chase;
            }
            let msg = match sev {
                crate::damage::Severity::Minor => "Aircraft damaged — everyone walks away",
                crate::damage::Severity::Substantial => "Substantial damage — survivable",
                crate::damage::Severity::Destroyed => "Aircraft destroyed",
            };
            self.say(msg, 8.0);
        }
        if self.ac.crashed.is_some() {
            self.crash_clock += dt;
            // The wings relax to their unloaded shape.
            let k = 1.0 - (-3.0 * dt).exp();
            for s in 0..2 {
                self.ac.dmg.flex[s] += (self.ac.dmg.set[s] - self.ac.dmg.flex[s]) * k;
            }
        }
        // Advance the wreckage (a moment of slow motion after a violent crash).
        let slow = if self.severity == Some(crate::damage::Severity::Destroyed) { (0.3 + 0.7 * ((self.crash_clock - 0.6) / 2.4).clamp(0.0, 1.0)).min(1.0) } else { 1.0 };
        let warp = (self.warp as f32).min(4.0);
        if let Some(d) = self.debris.as_mut() {
            let runways = &world.runways;
            let paved = |xz: Vec2| runways.iter().any(|r| seg_dist(xz, r.a, r.b) < r.width * 0.5 + 4.0);
            d.update(dt * warp * slow, world, &paved);
            self.audio_events.append(&mut d.audio);
            // The main wreck carries the aircraft (cameras, sound, HUD).
            if let Some(b) = d.main.and_then(|m| d.bodies.get(m)) {
                let (o, r) = b.origin();
                self.ac.pos = o;
                self.ac.rot = r;
                self.ac.vel = b.vel;
                self.ac.on_ground = b.contact || b.in_water;
            }
        }
    }

    /// The crash camera after a destroying accident: eye and target.
    fn crash_camera(&self) -> Option<(Vec3, Vec3)> {
        if self.severity != Some(crate::damage::Severity::Destroyed) {
            return None;
        }
        let (c, r) = self.debris.as_ref()?.extent()?;
        let dist = (self.orbit_dist * 0.8).max(r * 0.9 + self.size * 1.5 + 20.0).min(400.0);
        let yaw = self.cam_heading + self.orbit_yaw + 0.6 + self.crash_clock * 0.05;
        let p = self.orbit_pitch.max(13f32.to_radians());
        let back = vec3(-yaw.sin(), 0.0, yaw.cos());
        let mut eye = c + (back * p.cos() + Vec3::Y * p.sin()) * dist;
        eye.y = eye.y.max(c.y + 2.0);
        Some((eye, c + Vec3::Y * (r * 0.15).min(10.0)))
    }

    /// Effects data for this frame (vfx/).
    fn effects_frame(&self, eye: Vec3) -> crate::vfx::VfxFrame {
        let def = &self.defs[self.sel];
        let ac = &self.ac;
        let n = def.engine_count.min(4);
        let mut running = [false; 4];
        let mut thrust = [0.0; 4];
        let per = (def.static_thrust / n.max(1) as f32).max(1.0);
        for i in 0..n {
            running[i] = ac.engine_running(i) && ac.crashed.is_none() && ac.dmg.lost & crate::damage::Part::engine(i).bit() == 0;
            thrust[i] = (ac.engine_thrust(i) / per).clamp(0.0, 1.5);
        }
        let planform = self.model.parts.as_ref().map(|p| &p.planform[..]).unwrap_or(&[]);
        let ctx = crate::vfx::FrameCtx {
            def,
            eye,
            pos: self.rpos,
            rot: self.rrot,
            tips: ac.dmg.tip_deflection(crate::damage::struct_def(def.id)),
            lost: ac.dmg.lost,
            running,
            thrust,
            afterburner: ac.afterburner(),
            planform,
            fires: self.debris.as_ref().map(|d| &d.fires[..]),
        };
        self.vfx.frame(&ctx)
    }

    /// Advance the effects (after the simulation step).
    fn effects(&mut self, world: &World, dt: f32) {
        let def = self.defs[self.sel].clone();
        let slow = if self.severity == Some(crate::damage::Severity::Destroyed) { (0.3 + 0.7 * ((self.crash_clock - 0.6) / 2.4).clamp(0.0, 1.0)).min(1.0) } else { 1.0 };
        let (pos, rot) = self.pose();
        let tips = self.ac.dmg.tip_deflection(crate::damage::struct_def(def.id));
        let planform: Vec<(f32, f32, f32, f32)> = self.model.parts.as_ref().map(|p| p.planform.clone()).unwrap_or_default();
        let eye = self.last_view.as_ref().map(|v| v.eye).unwrap_or(pos);
        let ctx = crate::vfx::Ctx {
            def: &def,
            ac: &self.ac,
            pos,
            rot,
            world,
            debris: self.debris.as_mut(),
            time: self.time,
            dt: dt * slow * (self.warp as f32).min(4.0),
            eye,
            tips,
            planform: &planform,
        };
        self.vfx.update(ctx);
    }

    /// Part boxes from the model (or estimated from the flight geometry).
    fn part_geom(&self, def: &AircraftDef) -> crate::debris::PartGeom {
        match &self.model.parts {
            Some(p) => p.geom.clone(),
            None => crate::debris::PartGeom::estimate(def, crate::damage::struct_def(def.id)),
        }
    }

    /// Engine-failure practice: single-engine → fail / restore; multi-engine → fail
    /// engine 1, then engine 2 (1 restored), then all running.
    fn cycle_engine_failure(&mut self) {
        let def = self.def();
        let n = def.engine_count.min(8) as u8;
        if n == 0 {
            self.say("No engine to fail", 2.0);
            return;
        }
        self.fail_step = (self.fail_step + 1) % (n + 1);
        self.ctl.failed_engines = if self.fail_step == 0 { 0 } else { 1 << (self.fail_step - 1) };
        let msg = if self.fail_step == 0 {
            "Engine failure practice: all engines restored".to_string()
        } else if n == 1 {
            "ENGINE FAILURE (practice) — K to restore".to_string()
        } else {
            format!("ENGINE {} FAILURE (practice) — K for next", self.fail_step)
        };
        self.say(msg, 4.0);
    }

    pub fn flap_label(&self) -> String {
        let def = self.def();
        let v = def.flap_detents.get(self.flap_idx).copied().unwrap_or(0.0);
        // (The Vulcan's flap lever drives its airbrakes: shown like the glider's.)
        if def.has_engine && def.flap_label != "Airbrakes" {
            format!("{v:.0}°")
        } else if self.ctl.flaps < 0.02 {
            "closed".into()
        } else {
            format!("{:.0}%", self.ctl.flaps * 100.0)
        }
    }

    /// Re-launch the (airborne) aircraft on a new heading at its current speed.
    pub fn set_heading(&mut self, hdg: f32) {
        if self.ac.on_ground {
            return;
        }
        let def = &self.defs[self.sel];
        let v = self.ac.airspeed().max(25.0);
        self.ac = Aircraft::airborne(def, self.ac.pos, hdg, v);
        self.ctl.gear_down = self.ac.gear_pos() > 0.5;
        self.cam_heading = hdg;
        self.snap_pose();
    }

    /// Landing lights: the pilot's choice (L), else on with the gear down
    /// (fixed gear: below ~3,000 ft) or whenever the bot flies.
    pub fn landing_lights_on(&self) -> bool {
        // The FPV quad has no landing lights (only its LED bar).
        if self.defs[self.sel].category == Category::Drone {
            return false;
        }
        self.landing_lights.unwrap_or_else(|| {
            if self.bot.is_some() {
                self.ac.on_ground || self.ac.pos.y - self.ground_y < 3000.0
            } else if self.defs[self.sel].retractable_gear {
                self.ac.gear_pos() > 0.5
            } else {
                self.ac.pos.y - self.ground_y < 900.0
            }
        })
    }

    /// Direction toward the sun for the current date / time of day.
    pub fn sun_dir(&self) -> Vec3 {
        let zone = crate::atmosphere::civil_offset_h(self.home_ll.0, self.home_ll.1, self.day_of_year);
        crate::atmosphere::sun_direction_zoned(self.lat_lon.0, self.lat_lon.1, zone, self.day_of_year, self.hours)
    }

    fn snap_pose(&mut self) {
        self.prev_pos = self.ac.pos;
        self.prev_rot = self.ac.rot;
    }

    /// Pose to render: the 120 Hz physics state interpolated between the last two
    /// steps by the time left in the accumulator, so every display rate sees smooth
    /// motion (no 0/1/2-step judder). `SKYWARD_NO_INTERP=1` turns it off.
    /// The pose this frame's cameras and aircraft mesh used (set by `frame`).
    pub fn render_pose(&self) -> (Vec3, Quat) {
        (self.rpos, self.rrot)
    }

    pub fn pose(&self) -> (Vec3, Quat) {
        if std::env::var_os("SKYWARD_NO_INTERP").is_some() {
            return (self.ac.pos, self.ac.rot);
        }
        let a = (self.acc / DT).clamp(0.0, 1.0);
        if self.prev_pos.distance(self.ac.pos) > 200.0 {
            return (self.ac.pos, self.ac.rot);
        }
        (self.prev_pos.lerp(self.ac.pos, a), self.prev_rot.slerp(self.ac.rot, a).normalize())
    }

    pub fn map_range(&self) -> f32 {
        MAP_RANGES[self.map_zoom.min(MAP_RANGES.len() - 1)]
    }

    fn tower_eye(&self) -> Vec3 {
        let (a, b, w) = self.runway.as_ref().map(|r| (r.a, r.b, r.width)).unwrap_or((Vec2::ZERO, vec2(0.0, -1000.0), 45.0));
        let d = (b - a).normalize_or(vec2(0.0, -1.0));
        let side = vec2(-d.y, d.x);
        let mid = (a + b) * 0.5 + side * (w * 0.5 + 140.0 + self.size * 2.0);
        vec3(mid.x, 38.0, mid.y)
    }

    fn flyby_eye(&mut self) -> Vec3 {
        let ac = &self.ac;
        let pos = self.rpos;
        let vel = if ac.vel.length() > 3.0 { ac.vel.normalize() } else { self.rrot * Vec3::Z };
        // Placed ~6 s ahead (capped so fast jets stay framed); re-placed once
        // passed, or if it ends up much farther away than it was placed —
        // both limits scale with the placement distance, so a supersonic jet
        // still gets its pass instead of a re-place every frame.
        let speed = ac.vel.length();
        let lead = (speed * 6.0).min(1800.0) + 120.0 + self.size * 4.0;
        let need_new = match self.flyby_pos {
            None => true,
            Some(p) => (pos - p).dot(vel) > 250.0 + self.size * 4.0 + speed * 1.5 || p.distance(pos) > lead * 1.5 + 400.0 + self.size * 20.0,
        };
        if need_new {
            let side = vel.cross(Vec3::Y).normalize_or_zero();
            let mut p = pos + vel * lead + side * (30.0 + self.size * 1.6 + speed * 0.08);
            p.y = (p.y - 6.0 - self.size * 0.3).max(2.0);
            self.flyby_pos = Some(p);
        }
        self.flyby_pos.unwrap()
    }

    /// Cinematic camera: cycles between chase angles, fly-bys, tower, nose and wing shots.
    fn cinematic(&mut self) -> (Vec3, Vec3, Vec3, f32) {
        let pos = self.rpos;
        let r = self.size;
        if self.real_time >= self.cine.until {
            self.cine.shot = self.cine.shot.wrapping_add(1);
            let n = self.cine.shot;
            let h = crate::scene::hash(n as u64 * 7919 + 17);
            let f = |k: u32| ((h >> (k * 8)) & 0xff) as f32 / 255.0;
            self.cine.until = self.real_time + 7.0 + 5.0 * f(0);
            self.cine.yaw = (f(1) - 0.5) * 2.0 * 2.6;
            self.cine.pitch = (4.0 + 22.0 * f(2)).to_radians();
            self.cine.dist = r * (2.2 + 2.5 * f(3)) + 6.0;
            self.cine.spin = (f(4) - 0.5) * 0.25;
            self.cine.fixed = None;
            self.flyby_pos = None;
        }
        let near_runway = self.runway.as_ref().is_some_and(|rw| seg_dist(vec2(pos.x, pos.z), rw.a, rw.b) < 2500.0) && pos.y - self.ground_y < 400.0;
        let kind = match self.cine.shot % 6 {
            1 if near_runway => 3, // tower
            0 | 3 => 0,            // chase angles
            1 => 0,
            2 => 1, // fly-by
            4 => 2, // nose / wing
            _ => 4, // wing
        };
        let fwd = self.rrot * Vec3::Z;
        match kind {
            1 => {
                let eye = self.flyby_eye();
                let dist = eye.distance(pos).max(1.0);
                let fov = (2.0 * (r * 2.6 / dist).atan()).clamp(4f32.to_radians(), 70f32.to_radians());
                (eye, pos, Vec3::Y, fov)
            }
            3 => {
                let eye = self.tower_eye();
                let dist = eye.distance(pos).max(1.0);
                let fov = (2.0 * (r * 3.0 / dist).atan()).clamp(1.5f32.to_radians(), 60f32.to_radians());
                (eye, pos, Vec3::Y, fov)
            }
            2 => {
                // Ahead of the nose, looking back at the aircraft (air-to-air "lead" shot).
                let eye = pos + self.rrot * vec3(r * 0.35, r * 0.25, r * 2.4);
                (eye, pos, Vec3::Y, 50f32.to_radians())
            }
            4 => {
                // Off the wingtip, looking along the fuselage.
                let side = if self.cine.shot % 2 == 0 { 1.0 } else { -1.0 };
                let eye = pos + self.rrot * vec3(side * r * 1.15, r * 0.3, -r * 0.6);
                (eye, pos + fwd * r * 0.6, self.rrot * Vec3::Y, 58f32.to_radians())
            }
            _ => {
                self.cine.yaw += self.cine.spin * (1.0 / 60.0);
                let yaw = self.cam_heading + self.cine.yaw;
                let back = vec3(-yaw.sin(), 0.0, yaw.cos());
                let p = self.cine.pitch;
                let mut eye = pos + (back * p.cos() + Vec3::Y * p.sin()) * self.cine.dist;
                eye.y = eye.y.max(2.0);
                (eye, pos + Vec3::Y * r * 0.1, Vec3::Y, 55f32.to_radians())
            }
        }
    }

    /// Camera + world-space dynamic geometry for the current state.
    pub fn frame(&mut self, aspect: f32) -> (FrameView, MeshData) {
        (self.rpos, self.rrot) = self.pose();
        let pos = self.rpos;
        let mut fov = 60f32.to_radians();
        let show_aircraft = true;
        let (eye, target, cam_up) = match self.cam {
            CamMode::Chase => {
                let yaw = self.cam_heading + self.orbit_yaw;
                let back = vec3(-yaw.sin(), 0.0, yaw.cos()); // opposite of heading direction
                let p = self.orbit_pitch;
                let off = (back * p.cos() + Vec3::Y * p.sin()) * self.orbit_dist;
                let mut eye = pos + off;
                // (A small drone is framed from just above the ground.)
                let small = self.defs[self.sel].category == Category::Drone;
                eye.y = if small { eye.y.max(self.ground_y + 0.12) } else { eye.y.max(1.0) };
                let target = pos + Vec3::Y * if small { 0.02 } else { 0.5 + self.size * 0.08 };
                match self.crash_camera() {
                    // Destroyed: a slow orbit framing the whole wreckage (blended in).
                    Some((ce, ct)) => {
                        let k = (self.crash_clock / 1.5).clamp(0.0, 1.0);
                        let k = k * k * (3.0 - 2.0 * k);
                        (eye.lerp(ce, k), target.lerp(ct, k), Vec3::Y)
                    }
                    None => (eye, target, Vec3::Y),
                }
            }
            CamMode::Cockpit => {
                // SKYWARD_COCKPIT_FOV=<deg>: narrower view for instrument close-ups (captures).
                static FOV: std::sync::OnceLock<Option<f32>> = std::sync::OnceLock::new();
                let f = *FOV.get_or_init(|| std::env::var("SKYWARD_COCKPIT_FOV").ok().and_then(|v| v.parse::<f32>().ok()).filter(|v| (5.0..=120.0).contains(v)));
                fov = f.unwrap_or(72.0).to_radians();
                let e = pos + self.rrot * self.model.eye;
                // Head-look (input/sim_glue.rs).
                let head = self.rrot * self.pilot.look_rot();
                match self.defs[self.sel].rotor.as_ref().and_then(|r| r.quad()) {
                    // FPV quad: the camera is bolted on, tilted up, wide-angle.
                    Some(q) => {
                        fov = 2.0 * ((q.cam_fov * 0.5).tan() / aspect.max(0.5)).atan();
                        let cam = self.rrot * Quat::from_rotation_x(-q.cam_tilt);
                        (e, e + cam * Vec3::Z * 100.0, cam * Vec3::Y)
                    }
                    None => (e, e + head * Vec3::Z * 100.0, head * Vec3::Y),
                }
            }
            CamMode::Tower => {
                let eye = self.tower_eye();
                let dist = eye.distance(pos).max(1.0);
                fov = (2.0 * (self.size * 3.2 / dist).atan()).clamp(1.5f32.to_radians(), 60f32.to_radians());
                (eye, pos, Vec3::Y)
            }
            CamMode::Flyby => {
                let eye = self.flyby_eye();
                let dist = eye.distance(pos).max(1.0);
                fov = (2.0 * (self.size * 4.4 / dist).atan()).clamp(4f32.to_radians(), 70f32.to_radians());
                (eye, pos, Vec3::Y)
            }
            CamMode::Cinematic => {
                let (e, t, u, f) = self.cinematic();
                fov = f;
                (e, t, u)
            }
        };
        let _ = aspect;
        let view = glam::camera::rh::view::look_at_mat4(eye, target, cam_up);
        let zone = crate::atmosphere::civil_offset_h(self.home_ll.0, self.home_ll.1, self.day_of_year);
        let clock = crate::atmosphere::SkyClock::new(self.lat_lon.0, self.lat_lon.1, zone, self.day_of_year, self.hours);
        let on = self.landing_lights_on();
        let mut acl = crate::lights::AircraftLights::build(&self.light_rig, self.rpos - eye, self.rrot, self.time, on, on && self.ac.on_ground);
        if !show_aircraft {
            // From the cockpit the lights shine out, but their lenses aren't seen.
            acl.n_sprites = 0;
        }
        if self.defs[self.sel].category == Category::Drone {
            // The FPV quad has only its LED bar (an emissive part of the model).
            acl.n_sprites = 0;
            acl.n_spots = 0;
        }
        let mut fv = FrameView { eye, view, fov_y: fov, time: self.time, sun_dir: self.sun_dir(), exposure: 1.0, clock, acl, aircraft: None, vfx: None };
        let mut dynm = MeshData::default();
        if show_aircraft {
            // Eye-relative (see gfx: rendering is camera-relative, no f32 cancellation).
            // Close up (and from the cockpit) the interior is drawn too.
            let interior = self.cam == CamMode::Cockpit || eye.distance(pos) < self.size * 2.5 + 4.0;
            fv.aircraft = self.aircraft_mesh(&mut dynm, eye, interior);
        }
        if std::env::var_os("SKYWARD_NO_VFX").is_none() {
            let vf = self.effects_frame(eye);
            if std::env::var_os("SKYWARD_VFX_STATS").is_some() {
                eprintln!("vfx: {} particles, {} mesh verts, {} haze verts", vf.particles.len(), vf.mesh.len(), vf.haze.len());
            }
            fv.vfx = Some(std::sync::Arc::new(vf));
        }
        // Burning fuel lights up the scene (point lights in the aircraft-light slots).
        if let Some(vf) = fv.vfx.as_deref() {
            for f in vf.fires.iter().filter(|f| f[3] > 0.0) {
                let n = fv.acl.n_spots;
                if n < crate::lights::place::AC_SPOTS {
                    let pos = Vec3::new(f[0], f[1], f[2]) + vf.origin - eye;
                    // Inverse square out to ~0.3 lx; cos_inner < -1.5 marks a fire (no
                    // beam scattering in the air: the effects draw its lit smoke).
                    let range = (f[3] / 0.3).sqrt().clamp(60.0, 2500.0);
                    fv.acl.spots[n] = crate::lights::place::SpotLight { pos, dir: Vec3::Y, cd: Vec3::new(2.4, 0.66, 0.1) * f[3], cos_inner: -2.0, cos_outer: -1.0, range };
                    fv.acl.n_spots += 1;
                }
            }
        }
        self.last_view = Some(FrameView { aircraft: None, vfx: None, ..fv });
        (fv, dynm)
    }

    /// Animate the aircraft model for this frame and return its draw list
    /// (eye-relative); light glows and the afterburner flame go into `out`.
    fn aircraft_mesh(&mut self, out: &mut MeshData, eye: Vec3, interior: bool) -> Option<crate::gfx_aircraft::AircraftFrame> {
        let def = &self.defs[self.sel];
        let ac = &self.ac;
        let running = (0..ac.engine_count(def)).any(|i| ac.engine_running(i)) && def.has_engine;
        // Lights: navigation always, beacon with an engine running, strobes on the
        // runway / in flight, landing lights with the gear down below ~1,500 ft.
        let strobe = !ac.on_ground || self.ctl.throttle > 0.5;
        let beacon_flash = running && (self.time * 1.0).fract() < 0.12;
        let st = (self.time / 1.2).fract();
        let strobe_flash = strobe && (st < 0.05 || (0.12..0.17).contains(&st));
        let landing = ac.gear_pos() > 0.5 && ac.pos.y - self.ground_y < 460.0 && running;
        let daylight = crate::cockpit::daylight(self.sun_dir().y);
        self.model.update(&ModelInputs {
            def,
            ac,
            ctl: &self.ctl,
            time: self.time,
            nav: true,
            beacon: running,
            strobe,
            landing,
            ap: Some(&self.ap),
            qnh_hpa: self.qnh_hpa,
            hours: self.hours,
            daylight,
            radios: self.nav.equipped.then_some(&self.nav.radios),
        });
        let m = Mat4::from_rotation_translation(self.rrot, self.rpos - eye);
        let mut local = MeshData::default();
        // (Navigation / beacon / strobe lights: crate::lights, from the model's light positions.)
        let _ = (beacon_flash, strobe_flash);
        // Afterburner flame (brightness and length follow the reheat level, with a little flicker).
        let ab = ac.surfaces().afterburner;
        if def.has_afterburner && ab > 0.02 {
            let flick = 1.0 + 0.06 * (self.time * 37.0).sin() * (self.time * 23.0).cos();
            for &p in def.engine_pos {
                let mut flame = models::flame_mesh(Vec3::ZERO, 0.42);
                for v in &mut flame.verts {
                    v.uv[0] *= ab * flick;
                }
                let len = 0.55 + 0.45 * ab;
                local.append(&flame, Mat4::from_translation(p) * Mat4::from_scale(vec3(1.0, 1.0, len * flick)));
            }
        }
        // Linear vertex colours → the sRGB-encoded colours the scene shader expects.
        for v in &mut local.verts {
            for c in &mut v.col {
                *c = c.max(0.0).powf(1.0 / 2.2);
            }
        }
        out.append(&local, m);
        let mut f = self.model.frame(m, interior)?;
        f.dynamics = Some(Box::new(self.structure_frame(m, eye)));
        Some(f)
    }

    /// Wing bending, vibration and loose parts for the aircraft shader.
    fn structure_frame(&self, body: Mat4, eye: Vec3) -> crate::gfx_aircraft::AcDynamics {
        use crate::damage::Part;
        let def = &self.defs[self.sel];
        let sd = crate::damage::struct_def(def.id);
        let ac = &self.ac;
        let mut d = crate::gfx_aircraft::AcDynamics::default();
        let up = self.rrot * Vec3::Y;
        d.up = [up.x, up.y, up.z, self.time];
        let tip = ac.dmg.tip_deflection(sd);
        // (w: 1 / the flexing half-span, for the bent normals in the shader.)
        d.flex = [tip[0], tip[1], sd.twist_per_m, 1.0 / (def.wingspan * 0.5 - sd.fus_r).max(0.5)];
        // Rolling on the ground: a buzz that grows with speed (more on grass).
        if ac.on_ground && ac.crashed.is_none() {
            let gs = Vec2::new(ac.vel.x, ac.vel.z).length();
            let paved = self.runway.as_ref().is_some_and(|r| seg_dist(Vec2::new(ac.pos.x, ac.pos.z), r.a, r.b) < r.width * 0.5 + 2.0);
            let k = if paved { 0.0012 } else { 0.0035 };
            d.vib = [k * (gs / 25.0).min(2.0) * (self.size / 10.0).clamp(0.6, 2.5), if paved { 11.0 } else { 7.0 }, 0.0, 0.0];
        }
        let lost = ac.dmg.lost;
        let inv = body.inverse();
        for p in Part::ALL {
            let k = p.idx();
            let attached = lost & p.bit() == 0;
            let in_main = self.debris.as_ref().and_then(|db| db.main.map(|m| db.bodies[m].parts & p.bit() != 0)).unwrap_or(false);
            d.part_flex[k / 4][k % 4] = if attached && ac.crashed.is_none() { 1.0 } else if in_main { 1.0 } else { 0.0 };
            if let Some((o, r)) = self.debris.as_ref().filter(|_| !in_main).and_then(|db| db.part_pose(p)) {
                let w = Mat4::from_rotation_translation(r, o - eye);
                d.parts[k] = (w * inv).to_cols_array_2d();
                d.part_flex[k / 4][k % 4] = 0.0;
            } else if !attached && !in_main {
                // Lost but not simulated (shouldn't happen): hide it far below.
                d.parts[k] = Mat4::from_scale(Vec3::ZERO).to_cols_array_2d();
            }
        }
        d
    }
}

/// Load the model for an aircraft. Also returns the bounding radius.
fn load_model(def: &AircraftDef) -> (AircraftModel, f32) {
    let mut model = models::build(def.id, def.eye);
    if let Some(a) = model.asset.clone() {
        model.parts = Some(models::classify(&a, def));
    }
    // Nose (or steerable) wheel, for the taxi light.
    model.nose_gear = def.gear.iter().find(|g| matches!(g.kind, flight::ContactKind::Wheel { steerable: true, .. }) && g.pos.z > 0.0).map(|g| g.pos);
    let size = model.radius.max(def.wingspan * 0.5);
    (model, size)
}
