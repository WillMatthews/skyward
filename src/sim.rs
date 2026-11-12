//! Simulation session: aircraft, controls, autopilot / auto-flight bot,
//! cameras, and per-frame scene data.

use crate::autopilot::Autopilot;
use crate::bot::{self, Bot, RunwayEnd};
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
            (Start::Runway, _) => "Runway, ready for take-off".into(),
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
    /// Glance while held (hat / keypad), centre the view (input/).
    LookLeft,
    LookRight,
    LookUp,
    LookDown,
    LookCenter,
    /// Mouse yoke on / off; interface toggles handled by the window layer.
    MouseYoke,
    Instruments,
    WeatherPanel,
    Mute,
}

impl Key {
    /// Keys whose auto-repeat should register as repeated presses.
    pub fn repeats(self) -> bool {
        matches!(self, Key::ApHdgLeft | Key::ApHdgRight | Key::ApAltUp | Key::ApAltDown | Key::ApSpdUp | Key::ApSpdDown)
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
    match def.category {
        Category::Airliner => 1800.0,
        Category::FastJet => 1400.0,
        Category::Warbird => 700.0,
        Category::Trainer => 450.0,
        Category::Glider => 300.0,
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
            prev_pos: Vec3::ZERO,
            prev_rot: Quat::IDENTITY,
            rpos: Vec3::ZERO,
            rrot: Quat::IDENTITY,
            light_rig: Default::default(),
            landing_lights: None,
            pilot: Default::default(),
        };
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
        self.spoiler_idx = 0;
        self.fail_step = 0;
        self.ap = Autopilot::new();
        self.ap_warn = 0.0;
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
        self.snap_pose();
        self.note = advice.map(|a| (a, 9.0));
        if self.watch && !self.needs_respawn {
            self.enable_bot(world);
        }
    }

    fn chase_dist(&self) -> f32 {
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
                    let end = RunwayEnd::of(r, false);
                    if start == Start::Runway {
                        end.thr
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
                let (ac, mut ctl) = air(vec3(p.x, alt + base, p.y), heading_of(dir), v);
                if def.has_engine {
                    // A shallow descent with the approach flap set.
                    ctl.throttle *= 0.75;
                    ctl.flaps = def.flap_detents.get(flap).copied().unwrap_or(0.0) / def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
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
    }

    /// Hand the aircraft to the auto-flight bot.
    pub fn enable_bot(&mut self, world: &World) {
        let def = &self.defs[self.sel];
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
                Key::EngineFail => self.cycle_engine_failure(),
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
            self.orbit_dist = (self.orbit_dist * (1.0 - input.zoom_delta * 0.1)).clamp((self.size * 1.2).max(6.0), 600.0);
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
        self.apply_analog(input);
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

        if !self.paused && self.ac.crashed.is_none() && !self.needs_respawn {
            self.acc += dt * self.warp as f32;
            let mut steps = 0;
            while self.acc >= DT && steps < 256 {
                let def = &self.defs[self.sel];
                if let Some(b) = self.bot.as_mut() {
                    b.update(def, &self.ac, &mut self.ctl, &mut self.ap, world, DT);
                } else if self.ap.engaged && !self.ap.update(def, &self.ac, &mut self.ctl, DT) {
                    self.ap_warn = 4.0;
                    self.note = Some(("Autopilot disengaged".into(), 3.0));
                }
                if self.ctl.reverse && !self.ac.on_ground && self.bot.is_none() {
                    self.ctl.reverse = false;
                }
                self.prev_pos = self.ac.pos;
                self.prev_rot = self.ac.rot;
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
        // Watch mode: a landed glider is relaunched (as if by winch) after a pause.
        if self.watch && self.bot.as_ref().is_some_and(|b| b.wants_relaunch) {
            self.start = Start::default_for(&def);
            self.reset(world);
            self.say("Relaunched", 3.0);
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
        if def.has_engine {
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
                eye.y = eye.y.max(1.0);
                (eye, pos + Vec3::Y * (0.5 + self.size * 0.08), Vec3::Y)
            }
            CamMode::Cockpit => {
                fov = 72f32.to_radians();
                let e = pos + self.rrot * self.model.eye;
                // Head-look (input/sim_glue.rs).
                let head = self.rrot * self.pilot.look_rot();
                (e, e + head * Vec3::Z * 100.0, head * Vec3::Y)
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
        let mut fv = FrameView { eye, view, fov_y: fov, time: self.time, sun_dir: self.sun_dir(), exposure: 1.0, clock, acl, aircraft: None };
        let mut dynm = MeshData::default();
        if show_aircraft {
            // Eye-relative (see gfx: rendering is camera-relative, no f32 cancellation).
            // Close up (and from the cockpit) the interior is drawn too.
            let interior = self.cam == CamMode::Cockpit || eye.distance(pos) < self.size * 2.5 + 4.0;
            fv.aircraft = self.aircraft_mesh(&mut dynm, eye, interior);
        }
        self.last_view = Some(FrameView { aircraft: None, ..fv });
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
        self.model.update(&ModelInputs { def, ac, ctl: &self.ctl, time: self.time, nav: true, beacon: running, strobe, landing });
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
        self.model.frame(m, interior)
    }
}

/// Load the model for an aircraft. Also returns the bounding radius.
fn load_model(def: &AircraftDef) -> (AircraftModel, f32) {
    let mut model = models::build(def.id, def.eye);
    // Nose (or steerable) wheel, for the taxi light.
    model.nose_gear = def.gear.iter().find(|g| matches!(g.kind, flight::ContactKind::Wheel { steerable: true, .. }) && g.pos.z > 0.0).map(|g| g.pos);
    let size = model.radius.max(def.wingspan * 0.5);
    (model, size)
}
