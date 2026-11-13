//! The settings catalogue: every row of the settings screen as data — its
//! categories, label, one-line description, search keywords, widget, how to
//! read / write it on [`Settings`], whether it needs a restart, and whether
//! another track still has to wire its consumer. Search, the "changed" dots,
//! Reset category / Reset all and the navigation order all work from this
//! list (ui/settings_ui.rs draws it).

use super::settings::HudMode;
use super::settings::{Autobrake, DETAIL_NAMES, DefaultView, MapStyle, Palette, PressureUnit, RealismPreset, Settings, TempUnit, UI_SCALES};
use super::{AltUnit, SpeedUnit};

/// Settings categories, in rail order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Cat {
    Video,
    Display,
    Camera,
    Flight,
    Assists,
    Weather,
    Traffic,
    Sound,
    Controls,
    Multiplayer,
    Accessibility,
    Interface,
    Data,
    About,
}

impl Cat {
    pub const ALL: [Cat; 14] = [
        Cat::Video,
        Cat::Display,
        Cat::Camera,
        Cat::Flight,
        Cat::Assists,
        Cat::Weather,
        Cat::Traffic,
        Cat::Sound,
        Cat::Controls,
        Cat::Multiplayer,
        Cat::Accessibility,
        Cat::Interface,
        Cat::Data,
        Cat::About,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Cat::Video => "Video",
            Cat::Display => "Display & HUD",
            Cat::Camera => "Camera",
            Cat::Flight => "Flight & Realism",
            Cat::Assists => "Assists",
            Cat::Weather => "Weather & Time",
            Cat::Traffic => "Traffic",
            Cat::Sound => "Sound",
            Cat::Controls => "Controls",
            Cat::Multiplayer => "Multiplayer",
            Cat::Accessibility => "Accessibility",
            Cat::Interface => "Interface & Language",
            Cat::Data => "Data & Storage",
            Cat::About => "About",
        }
    }
    /// One line under the category title.
    pub fn blurb(self) -> &'static str {
        match self {
            Cat::Video => "Graphics quality, frame pacing and scenery detail",
            Cat::Display => "Instruments, units, map and on-screen text",
            Cat::Camera => "Field of view, head-look and the chase camera",
            Cat::Flight => "How forgiving the simulation is",
            Cat::Assists => "Help while you learn",
            Cat::Weather => "Default conditions and time acceleration",
            Cat::Traffic => "Other aircraft and vehicles in the world",
            Cat::Sound => "Volumes and the output device",
            Cat::Controls => "Keyboard, controllers, mouse and head-look",
            Cat::Multiplayer => "Your name, servers and chat",
            Cat::Accessibility => "Scale, colour, motion and input comfort",
            Cat::Interface => "Language and the interface itself",
            Cat::Data => "Downloaded scenery, cache and network use",
            Cat::About => "Version, licence and credits",
        }
    }
    /// Command-line / env id (`--settings-category`).
    pub fn id(self) -> &'static str {
        match self {
            Cat::Video => "video",
            Cat::Display => "display",
            Cat::Camera => "camera",
            Cat::Flight => "flight",
            Cat::Assists => "assists",
            Cat::Weather => "weather",
            Cat::Traffic => "traffic",
            Cat::Sound => "sound",
            Cat::Controls => "controls",
            Cat::Multiplayer => "multiplayer",
            Cat::Accessibility => "accessibility",
            Cat::Interface => "interface",
            Cat::Data => "data",
            Cat::About => "about",
        }
    }
    pub fn parse(s: &str) -> Option<Cat> {
        let s = s.to_ascii_lowercase();
        Cat::ALL.into_iter().find(|c| c.id() == s || c.name().to_ascii_lowercase() == s)
    }
    /// Words that find the whole category in search (its own rows aside).
    pub fn keywords(self) -> &'static str {
        match self {
            Cat::Video => "graphics quality preset resolution vsync detail shadows clouds anti-aliasing fps performance",
            Cat::Controls => "keys keyboard bindings rebind gamepad joystick hotas pedals mouse yoke deadzone curve calibrate",
            Cat::About => "version licence license credits gpu build",
            _ => "",
        }
    }
}

/// A row's value.
#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    B(bool),
    I(usize),
    F(f32),
    S(String),
    None,
}

impl Val {
    pub fn b(&self) -> bool {
        matches!(self, Val::B(true))
    }
    pub fn i(&self) -> usize {
        if let Val::I(i) = self { *i } else { 0 }
    }
    pub fn f(&self) -> f32 {
        if let Val::F(f) = self { *f } else { 0.0 }
    }
    pub fn s(&self) -> &str {
        if let Val::S(s) = self { s } else { "" }
    }
}

/// Buttons in rows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Act {
    ClearTiles,
    ClearTerrain,
    OpenFailures,
    Credits,
}

/// Read-only rows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Info {
    Version,
    Gpu,
    ConfigPath,
    CachePath,
    CacheSize,
    Licence,
    Audio,
    /// The status colours in the chosen palette.
    Palette,
}

#[derive(Clone, Copy, Debug)]
pub enum Kind {
    /// On / off switch.
    Toggle,
    /// Segmented control (≤ 4 short options) or a dropdown.
    Choice(&'static [&'static str]),
    /// Slider with its value and unit.
    Slider {
        min: f32,
        max: f32,
        step: f32,
        fmt: fn(f32) -> String,
    },
    /// One-line text field (`digits`: numbers only).
    Text {
        max: usize,
        digits: bool,
    },
    /// The audio output device (a dropdown filled at run time).
    Device,
    /// A button (`confirm`: asks first).
    Action {
        act: Act,
        button: &'static str,
        confirm: bool,
    },
    Info(Info),
}

impl Kind {
    /// Rows that hold a value (reset, changed dot).
    pub fn has_value(self) -> bool {
        !matches!(self, Kind::Action { .. } | Kind::Info(_))
    }
}

pub struct Item {
    pub id: &'static str,
    /// Categories it appears in; the first is its home (shown in search results).
    pub cats: &'static [Cat],
    /// Section heading within the category.
    pub group: &'static str,
    pub label: &'static str,
    pub desc: &'static str,
    pub keywords: &'static str,
    pub kind: Kind,
    pub get: fn(&Settings) -> Val,
    pub set: fn(&mut Settings, Val),
    /// Takes effect at the next start.
    pub restart: bool,
    /// Another track wires what reads it (shown as "Soon"): the track's name.
    pub pending: Option<&'static str>,
    /// Not in the browser build (no file system / devices / sockets there).
    pub native_only: bool,
    /// Only while flying.
    pub in_flight_only: bool,
}

fn no_get(_: &Settings) -> Val {
    Val::None
}
fn no_set(_: &mut Settings, _: Val) {}

const BASE: Item = Item {
    id: "",
    cats: &[],
    group: "",
    label: "",
    desc: "",
    keywords: "",
    kind: Kind::Toggle,
    get: no_get,
    set: no_set,
    restart: false,
    pending: None,
    native_only: false,
    in_flight_only: false,
};

macro_rules! toggle {
    ($id:expr, $cats:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, |$s:ident| $field:expr) => {
        Item {
            id: $id,
            cats: $cats,
            group: $group,
            label: $label,
            desc: $desc,
            keywords: $kw,
            kind: Kind::Toggle,
            get: |$s| Val::B($field),
            set: |$s, v| {
                if let Val::B(b) = v {
                    $field = b;
                }
            },
            ..BASE
        }
    };
}

macro_rules! slider {
    ($id:expr, $cats:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, $min:expr, $max:expr, $step:expr, $fmt:expr, |$s:ident| $field:expr) => {
        Item {
            id: $id,
            cats: $cats,
            group: $group,
            label: $label,
            desc: $desc,
            keywords: $kw,
            kind: Kind::Slider { min: $min, max: $max, step: $step, fmt: $fmt },
            get: |$s| Val::F($field),
            set: |$s, v| {
                if let Val::F(f) = v {
                    $field = f.clamp($min, $max);
                }
            },
            ..BASE
        }
    };
}

/// A choice over `values` (compared with ==): index in, index out.
macro_rules! choice {
    ($id:expr, $cats:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, $opts:expr, $vals:expr, |$s:ident| $field:expr) => {
        Item {
            id: $id,
            cats: $cats,
            group: $group,
            label: $label,
            desc: $desc,
            keywords: $kw,
            kind: Kind::Choice($opts),
            get: |$s| Val::I($vals.iter().position(|x| *x == $field).unwrap_or(0)),
            set: |$s, v| {
                if let Val::I(i) = v {
                    if let Some(x) = $vals.get(i) {
                        $field = *x;
                    }
                }
            },
            ..BASE
        }
    };
}

fn pct(v: f32) -> String {
    format!("{:.0} %", v * 100.0)
}
fn deg(v: f32) -> String {
    format!("{v:.0}°")
}
fn times(v: f32) -> String {
    format!("{v:.1}×")
}
fn nm(v: f32) -> String {
    format!("{v:.0} nm")
}

const MAIN: &[Cat] = &[Cat::Video];
const DISPLAY: &[Cat] = &[Cat::Display];
const CAMERA: &[Cat] = &[Cat::Camera];
const FLIGHT: &[Cat] = &[Cat::Flight];
const ASSISTS: &[Cat] = &[Cat::Assists];
const WEATHER: &[Cat] = &[Cat::Weather];
const TRAFFIC: &[Cat] = &[Cat::Traffic];
const SOUND: &[Cat] = &[Cat::Sound];
const MP: &[Cat] = &[Cat::Multiplayer];
const DATA: &[Cat] = &[Cat::Data];
const ABOUT: &[Cat] = &[Cat::About];
const INTERFACE: &[Cat] = &[Cat::Interface];

const HUDS: [HudMode; 3] = [HudMode::Full, HudMode::Minimal, HudMode::Off];
const SPEEDS: [SpeedUnit; 3] = [SpeedUnit::Knots, SpeedUnit::Kmh, SpeedUnit::Mph];
const ALTS: [AltUnit; 2] = [AltUnit::Feet, AltUnit::Metres];
const PRESSURES: [PressureUnit; 2] = [PressureUnit::Hpa, PressureUnit::InHg];
const TEMPS: [TempUnit; 2] = [TempUnit::Celsius, TempUnit::Fahrenheit];
const MAPS: [MapStyle; 3] = [MapStyle::Auto, MapStyle::Day, MapStyle::Night];
const BOOLS_YES: [bool; 2] = [true, false];
const BOOLS_NO: [bool; 2] = [false, true];
const AUTOBRAKES: [Autobrake; 4] = [Autobrake::Off, Autobrake::Low, Autobrake::Medium, Autobrake::Max];
const VIEWS: [DefaultView; 2] = [DefaultView::Chase, DefaultView::Cockpit];
const WARPS: [u32; 5] = [1, 2, 4, 8, 16];
const ROAD_LEVELS: [u8; 4] = [0, 1, 2, 3];
const DETAILS: [u8; 4] = [0, 1, 2, 3];
const CACHE_LIMITS: [u32; 5] = [0, 1024, 2048, 5120, 10240];
const LANGS: [&str; 1] = ["en"];

/// Every settings row, in display order within each category.
pub fn items() -> &'static [Item] {
    static ITEMS: std::sync::OnceLock<Vec<Item>> = std::sync::OnceLock::new();
    ITEMS.get_or_init(build)
}

fn build() -> Vec<Item> {
    let scale_opts: &'static [&'static str] = &["80 %", "90 %", "100 %", "115 %", "130 %", "150 %", "175 %", "200 %"];
    vec![
        // ---------------------------------------------------------- video (stub: GFX owns the tab)
        Item {
            kind: Kind::Choice(&["On", "Off"]),
            ..choice!(
                "vsync",
                MAIN,
                "Frame pacing",
                "Vertical sync",
                "Off gives the lowest latency but may tear or stutter",
                "vsync tearing latency present fps",
                &["On", "Off"],
                BOOLS_YES,
                |s| s.vsync
            )
        },
        choice!(
            "detail",
            MAIN,
            "Scenery",
            "Scenery detail",
            "How far detailed buildings and roads stream in",
            "detail distance radius tiles buildings lod",
            &DETAIL_NAMES,
            DETAILS,
            |s| s.detail
        ),
        // ---------------------------------------------------------- display & HUD
        choice!(
            "hud",
            DISPLAY,
            "Instruments",
            "Instruments",
            "How much of the cockpit overlay is shown (U cycles in flight)",
            "hud overlay gauges panel pfd",
            &["Full", "Minimal", "Off"],
            HUDS,
            |s| s.hud
        ),
        choice!(
            "speed_unit",
            DISPLAY,
            "Units",
            "Speed",
            "Panels and the status card; dials keep the aircraft's own units",
            "units knots kmh mph airspeed",
            &["Knots", "km/h", "mph"],
            SPEEDS,
            |s| s.speed
        ),
        choice!("alt_unit", DISPLAY, "Units", "Altitude", "Heights in panels and on the map", "units feet metres meters height", &["Feet", "Metres"], ALTS, |s| s.alt),
        choice!(
            "pressure_unit",
            DISPLAY,
            "Units",
            "Pressure",
            "QNH in the weather panel and briefings",
            "units qnh altimeter hpa inhg millibar baro",
            &["hPa", "inHg"],
            PRESSURES,
            |s| s.pressure
        ),
        choice!("temp_unit", DISPLAY, "Units", "Temperature", "Outside air temperature and dew point", "units celsius fahrenheit oat", &["°C", "°F"], TEMPS, |s| s.temp),
        choice!("map_style", DISPLAY, "Map", "Map style", "Auto follows the sun", "map day night dark theme", &["Auto", "Day", "Night"], MAPS, |s| s.map_style),
        choice!(
            "map_orient",
            DISPLAY,
            "Map",
            "Map orientation",
            "Track up turns the map with you",
            "map north up track heading rotate",
            &["North up", "Track up"],
            BOOLS_YES,
            |s| s.north_up
        ),
        toggle!(
            "name_tags",
            &[Cat::Display, Cat::Multiplayer],
            "On screen",
            "Name tags",
            "Callsigns over other players' aircraft (Num * in flight)",
            "names labels players multiplayer",
            |s| s.mp.name_tags
        ),
        toggle!("subtitles", &[Cat::Display, Cat::Accessibility], "On screen", "Subtitles", "ATC, ATIS and crew lines as text", "captions text atc radio hearing", |s| s.subtitles),
        Item {
            kind: Kind::Choice(scale_opts),
            ..choice!(
                "ui_scale",
                &[Cat::Display, Cat::Accessibility, Cat::Interface],
                "On screen",
                "Interface scale",
                "Size of menus, panels and text",
                "ui size zoom text bigger larger font dpi",
                scale_opts,
                UI_SCALES,
                |s| s.ui_scale
            )
        },
        choice!(
            "palette",
            &[Cat::Display, Cat::Accessibility],
            "Colour",
            "Colour palette",
            "Status colours tuned for colour vision deficiency",
            "colour color blind colorblind deuteranopia protanopia tritanopia accessibility",
            &["Standard", "Deuteranopia", "Protanopia", "Tritanopia"],
            Palette::ALL,
            |s| s.palette
        ),
        Item {
            id: "palette_preview",
            cats: &[Cat::Display, Cat::Accessibility],
            group: "Colour",
            label: "Preview",
            desc: "",
            keywords: "colour color preview",
            kind: Kind::Info(Info::Palette),
            ..BASE
        },
        toggle!(
            "high_contrast",
            &[Cat::Display, Cat::Accessibility],
            "Colour",
            "High contrast",
            "Brighter secondary text and more opaque panels",
            "contrast legibility readability vision",
            |s| s.high_contrast
        ),
        // ---------------------------------------------------------- camera
        slider!(
            "cockpit_fov",
            CAMERA,
            "Field of view",
            "Cockpit field of view",
            "Vertical angle seen from the pilot's seat",
            "fov zoom angle cockpit",
            40.0,
            110.0,
            1.0,
            deg,
            |s| s.camera.cockpit_fov
        ),
        slider!("chase_fov", CAMERA, "Field of view", "Outside field of view", "Vertical angle of the chase camera", "fov zoom angle chase external", 30.0, 100.0, 1.0, deg, |s| s
            .camera
            .chase_fov),
        slider!("look_sens", CAMERA, "Head-look", "Head-look speed", "Right-drag in the cockpit to look around", "mouse look sensitivity view", 0.3, 2.5, 0.1, times, |s| s
            .controls
            .mouse
            .look_sens),
        choice!(
            "look_invert",
            CAMERA,
            "Head-look",
            "Head-look vertical",
            "Which way dragging up moves the view",
            "invert mouse look y axis",
            &["Normal", "Inverted"],
            BOOLS_NO,
            |s| s.controls.mouse.invert_look
        ),
        toggle!(
            "head_motion",
            &[Cat::Camera, Cat::Accessibility],
            "Head-look",
            "Cockpit head motion",
            "Your head moves with g in turns, bumps and touchdowns",
            "g head bob shake camera motion sickness",
            |s| s.camera.head_motion
        ),
        slider!(
            "chase_distance",
            CAMERA,
            "Chase camera",
            "Chase distance",
            "How far behind the aircraft the chase camera sits",
            "camera distance zoom external",
            0.5,
            3.0,
            0.1,
            times,
            |s| s.camera.chase_distance
        ),
        slider!("chase_lag", CAMERA, "Chase camera", "Chase lag", "How lazily the camera swings round in turns", "camera smoothing follow inertia", 0.0, 1.0, 0.05, pct, |s| s
            .camera
            .chase_lag),
        choice!(
            "default_view",
            CAMERA,
            "Chase camera",
            "Starting view",
            "The view a new flight opens in (C cycles views)",
            "camera cockpit chase first person",
            &["Chase", "Cockpit"],
            VIEWS,
            |s| s.camera.default_view
        ),
        // ---------------------------------------------------------- flight & realism
        Item {
            id: "realism_preset",
            cats: FLIGHT,
            group: "Preset",
            label: "Realism",
            desc: "Sets the options below; change any to make your own",
            keywords: "difficulty casual standard hard easy realistic preset",
            kind: Kind::Choice(&["Casual", "Standard", "Hard", "Custom"]),
            get: |s| Val::I(s.realism_preset() as usize),
            set: |s, v| {
                let p = match v.i() {
                    0 => RealismPreset::Casual,
                    1 => RealismPreset::Standard,
                    2 => RealismPreset::Hard,
                    _ => return,
                };
                s.apply_realism_preset(p);
            },
            ..BASE
        },
        choice!(
            "structural_damage",
            FLIGHT,
            "Damage",
            "Structural damage",
            "Forgiving: higher load margins · Off: indestructible",
            "damage g overstress wings break limits forgiving indestructible",
            &["Realistic", "Forgiving", "Off"],
            crate::damage::Realism::ALL,
            |s| s.structural_damage
        ),
        choice!(
            "crashes",
            FLIGHT,
            "Damage",
            "Crashes",
            "Forgiving: survivable hard landings · Off: no crash, the aircraft resets",
            "crash collision crash detection invincible reset forgiving",
            &["Realistic", "Forgiving", "Off"],
            crate::damage::Realism::ALL,
            |s| s.crashes
        ),
        Item {
            pending: Some("DMG"),
            ..toggle!("engine_damage", FLIGHT, "Damage", "Engine damage", "Engines can be torn off by overload or impact", "damage engine pylon", |s| s.realism.engine_damage)
        },
        toggle!("g_effects", FLIGHT, "Pilot", "G-effects", "Grey-out and blackout at high g, red-out when negative", "g force blackout redout greyout gloc vision", |s| s
            .realism
            .g_effects),
        toggle!(
            "random_failures",
            FLIGHT,
            "Systems",
            "Random failures",
            "New flights arm random failures (set the window in the failures panel)",
            "failures random mtbf emergency systems",
            |s| s.realism.random_failures
        ),
        Item {
            id: "open_failures",
            cats: FLIGHT,
            group: "Systems",
            label: "Failures panel",
            desc: "Trigger, arm or repair failures on this flight",
            keywords: "failures systems repair arm",
            kind: Kind::Action { act: Act::OpenFailures, button: "Open", confirm: false },
            in_flight_only: true,
            ..BASE
        },
        toggle!("unlimited_fuel", FLIGHT, "Systems", "Unlimited fuel", "The tanks never drain below what you started with", "fuel infinite refuel", |s| s.realism.unlimited_fuel),
        toggle!("auto_rudder", FLIGHT, "Handling", "Auto-rudder", "The rudder coordinates turns while the pedals are centred", "rudder yaw coordination pedals assist", |s| s
            .realism
            .auto_rudder),
        toggle!("auto_trim", FLIGHT, "Handling", "Auto-trim", "Trim holds the attitude when you let go of the stick", "trim elevator pitch hold assist", |s| s.realism.auto_trim),
        Item {
            ..choice!(
                "autobrake",
                FLIGHT,
                "Handling",
                "Autobrake",
                "Armed for landing on aircraft that have it",
                "autobrake brakes landing rollout",
                &["Off", "Low", "Medium", "Max"],
                AUTOBRAKES,
                |s| s.realism.autobrake
            )
        },
        // ---------------------------------------------------------- assists
        Item {
            ..toggle!("taxi_ribbon", ASSISTS, "Ground", "Taxi ribbon", "A line along your cleared taxi route", "taxi route path ground guidance arrows", |s| s.assists.taxi_ribbon)
        },
        Item {
            pending: Some("GH"),
            ..toggle!("landing_guidance", ASSISTS, "Approach", "Landing guidance", "A glide-path cue on final approach", "landing approach glide slope path guidance", |s| s
                .assists
                .landing_guidance)
        },
        toggle!("stall_warning", ASSISTS, "Warnings", "Stall warning", "A large STALL cue in the middle of the screen", "stall warning alert cue", |s| s.assists.stall_warning),
        toggle!("overspeed_warning", ASSISTS, "Warnings", "Overspeed warning", "A large OVERSPEED cue past the aircraft's limits", "overspeed vne mmo warning alert", |s| s
            .assists
            .overspeed_warning),
        // ---------------------------------------------------------- weather & time
        choice!(
            "wx_default",
            WEATHER,
            "Defaults",
            "Default weather",
            "Live uses real METARs and the local time",
            "live real weather metar fixed preset",
            &["Live", "Fixed"],
            BOOLS_YES,
            |s| s.conditions.live
        ),
        Item {
            id: "wx_preset",
            cats: WEATHER,
            group: "Defaults",
            label: "Fixed clouds",
            desc: "The sky when the weather is fixed and no METAR is given",
            keywords: "clouds clear overcast broken scattered preset",
            kind: Kind::Choice(&["Clear", "Few", "Scattered", "Broken", "Overcast"]),
            get: |s| Val::I(crate::clouds::Weather::ALL.iter().position(|w| w.name() == s.conditions.preset).unwrap_or(2)),
            set: |s, v| {
                if let Some(w) = crate::clouds::Weather::ALL.get(v.i()) {
                    s.conditions.preset = w.name().to_string();
                }
            },
            ..BASE
        },
        Item {
            pending: Some("WX"),
            ..slider!(
                "turbulence",
                WEATHER,
                "Air",
                "Turbulence",
                "Strength of bumps in cloud, wind and thermals (50 % = as reported)",
                "turbulence bumps gusts wind shear",
                0.0,
                1.0,
                0.05,
                pct,
                |s| s.wx.turbulence
            )
        },
        choice!(
            "max_warp",
            WEATHER,
            "Time",
            "Time acceleration limit",
            "The fastest W cycles to",
            "time warp acceleration speed fast forward",
            &["1×", "2×", "4×", "8×", "16×"],
            WARPS,
            |s| s.wx.max_warp
        ),
        // ---------------------------------------------------------- traffic
        choice!(
            "road_traffic",
            TRAFFIC,
            "Ground",
            "Road traffic",
            "Cars, buses and lorries on the roads (Off frees their cost)",
            "cars vehicles roads density lorries buses performance",
            &crate::traffic::Level::NAMES,
            ROAD_LEVELS,
            |s| s.traffic.road
        ),
        Item {
            pending: Some("A"),
            ..slider!(
                "parked_density",
                TRAFFIC,
                "Ground",
                "Parked aircraft",
                "Airliners and light aircraft at stands",
                "parked static aircraft stands apron",
                0.0,
                1.0,
                0.05,
                pct,
                |s| s.traffic.parked_density
            )
        },
        // Live traffic (adsb/) — owned by X.
        Item {
            native_only: true,
            ..toggle!("adsb", TRAFFIC, "Air", "Live air traffic", "Real aircraft around you from adsb.lol (ODbL)", "adsb live real traffic ai aircraft", |s| s.live_traffic.enabled)
        },
        Item {
            native_only: true,
            ..slider!("adsb_radius", TRAFFIC, "Air", "Live traffic radius", "How far around you to fetch, more ahead", "adsb radius range distance", 5.0, 250.0, 5.0, nm, |s| s
                .live_traffic
                .radius_nm)
        },
        Item {
            native_only: true,
            ..choice!(
                "adsb_count",
                TRAFFIC,
                "Air",
                "Live traffic count",
                "Most aircraft shown, nearest first",
                "adsb budget count number aircraft",
                &["50", "100", "150", "250"],
                crate::adsb::BUDGETS,
                |s| s.live_traffic.max_aircraft
            )
        },
        Item {
            native_only: true,
            ..toggle!("adsb_ground", TRAFFIC, "Air", "Live ground traffic", "Taxiing and parked real aircraft", "adsb ground taxi parked", |s| s.live_traffic.ground)
        },
        Item {
            native_only: true,
            ..toggle!("adsb_tags", TRAFFIC, "Air", "Live traffic tags", "Callsigns over the aircraft (Num *)", "adsb tags labels callsign names", |s| s.live_traffic.tags)
        },
        // ---------------------------------------------------------- sound
        toggle!("muted", SOUND, "Output", "Mute", "Silence everything (N in flight)", "mute silent quiet off", |s| s.sound.muted),
        Item {
            id: "audio_device",
            cats: SOUND,
            group: "Output",
            label: "Output device",
            desc: "Where the sound plays",
            keywords: "speakers headphones device output",
            kind: Kind::Device,
            get: |s| Val::S(s.audio_device.clone()),
            set: |s, v| s.audio_device = v.s().to_string(),
            native_only: true,
            ..BASE
        },
        toggle!("mute_bg", SOUND, "Output", "Mute in background", "Silent while the window is not focused", "focus background alt tab minimise", |s| s.mute_in_background),
        slider!("vol_master", SOUND, "Volume", "Master", "Everything together", "volume loud", 0.0, 1.0, 0.01, pct, |s| s.sound.master),
        slider!("vol_engines", SOUND, "Volume", "Engines", "Engines, props, rotors and jets", "volume engine", 0.0, 1.5, 0.01, pct, |s| s.sound.engines),
        slider!("vol_env", SOUND, "Volume", "Wind, ground, weather", "Airflow, rain, tyres and impacts", "volume wind rain environment", 0.0, 1.5, 0.01, pct, |s| s
            .sound
            .environment),
        slider!("vol_systems", SOUND, "Volume", "Cockpit systems", "Switches, pumps, gear and warnings", "volume systems cockpit", 0.0, 1.5, 0.01, pct, |s| s.sound.systems),
        slider!(
            "vol_voice",
            SOUND,
            "Volume",
            "Callouts & cabin PA",
            "Altitude callouts, crew and cabin announcements",
            "volume voice callouts pa announcements",
            0.0,
            1.5,
            0.01,
            pct,
            |s| s.sound.voice
        ),
        slider!("vol_radio", SOUND, "Volume", "ATC radio", "Controllers and ATIS in your headset", "volume atc radio atis", 0.0, 1.5, 0.01, pct, |s| s.sound.radio),
        slider!("vol_ui", &[Cat::Sound, Cat::Interface], "Volume", "Interface", "Menu clicks", "volume ui menu clicks", 0.0, 1.5, 0.01, pct, |s| s.sound.ui),
        Item {
            id: "audio_status",
            cats: SOUND,
            group: "Output",
            label: "Status",
            desc: "",
            keywords: "audio device status",
            kind: Kind::Info(Info::Audio),
            native_only: true,
            ..BASE
        },
        // ---------------------------------------------------------- controls (the bindings page follows)
        choice!(
            "toggle_brakes",
            &[Cat::Accessibility],
            "Input",
            "Wheel brakes",
            "Hold the brake key, or press once to latch them",
            "brakes hold toggle latch",
            &["Hold", "Toggle"],
            BOOLS_NO,
            |s| s.access.toggle_brakes
        ),
        // ---------------------------------------------------------- multiplayer
        Item {
            id: "player_name",
            cats: MP,
            group: "You",
            label: "Player name",
            desc: "Shown to other pilots on a server",
            keywords: "name callsign nickname",
            kind: Kind::Text { max: 20, digits: false },
            get: |s| Val::S(s.player_name.clone()),
            set: |s, v| s.player_name = v.s().to_string(),
            native_only: true,
            ..BASE
        },
        Item {
            id: "default_port",
            cats: MP,
            group: "Servers",
            label: "Default port",
            desc: "Used when an address has no port",
            keywords: "port udp network address server",
            kind: Kind::Text { max: 5, digits: true },
            get: |s| Val::S(s.mp.default_port.to_string()),
            set: |s, v| {
                if let Some(p) = v.s().parse::<u16>().ok().filter(|p| *p > 0) {
                    s.mp.default_port = p;
                }
            },
            native_only: true,
            ..BASE
        },
        toggle!("chat_filter", MP, "Chat", "Chat filter", "Masks offensive words in chat", "chat profanity filter swear words", |s| s.mp.chat_filter),
        // ---------------------------------------------------------- accessibility
        toggle!(
            "reduced_motion",
            &[Cat::Accessibility],
            "Motion",
            "Reduced motion",
            "Keeps the cockpit camera still: no head motion",
            "motion sickness animation shake vestibular",
            |s| s.access.reduced_motion
        ),
        // ---------------------------------------------------------- interface & language
        Item {
            id: "language",
            cats: INTERFACE,
            group: "Language",
            label: "Language",
            desc: "More languages are on the way",
            keywords: "language translation locale english",
            kind: Kind::Choice(&["English"]),
            get: |s| Val::I(LANGS.iter().position(|l| *l == s.language).unwrap_or(0)),
            set: |s, v| s.language = LANGS.get(v.i()).unwrap_or(&"en").to_string(),
            ..BASE
        },
        // ---------------------------------------------------------- data & storage
        Item {
            id: "cache_path",
            cats: DATA,
            group: "Cache",
            label: "Location",
            desc: "",
            keywords: "cache folder path directory disk",
            kind: Kind::Info(Info::CachePath),
            native_only: true,
            ..BASE
        },
        Item {
            id: "cache_size",
            cats: DATA,
            group: "Cache",
            label: "Size on disk",
            desc: "",
            keywords: "cache size disk space",
            kind: Kind::Info(Info::CacheSize),
            native_only: true,
            ..BASE
        },
        Item {
            restart: true,
            native_only: true,
            ..choice!(
                "cache_limit",
                DATA,
                "Cache",
                "Cache limit",
                "The oldest scenery is removed at start-up beyond this",
                "cache size limit disk space",
                &["Unlimited", "1 GB", "2 GB", "5 GB", "10 GB"],
                CACHE_LIMITS,
                |s| s.data.cache_limit_mb
            )
        },
        Item {
            id: "clear_tiles",
            cats: DATA,
            group: "Cache",
            label: "Map tiles",
            desc: "Roads, buildings and airport layouts download again as you fly",
            keywords: "clear delete cache tiles map",
            kind: Kind::Action { act: Act::ClearTiles, button: "Clear", confirm: true },
            native_only: true,
            ..BASE
        },
        Item {
            id: "clear_terrain",
            cats: DATA,
            group: "Cache",
            label: "Terrain",
            desc: "Elevation data downloads again as you fly",
            keywords: "clear delete cache terrain elevation dem",
            kind: Kind::Action { act: Act::ClearTerrain, button: "Clear", confirm: true },
            native_only: true,
            ..BASE
        },
        Item {
            native_only: true,
            ..toggle!("offline", DATA, "Network", "Offline mode", "Fly on cached scenery only: no downloads, no live weather", "offline network internet download data", |s| s
                .data
                .offline)
        },
        Item {
            id: "config_path",
            cats: DATA,
            group: "Files",
            label: "Settings file",
            desc: "",
            keywords: "settings file config json path",
            kind: Kind::Info(Info::ConfigPath),
            native_only: true,
            ..BASE
        },
        // ---------------------------------------------------------- about
        Item { id: "version", cats: ABOUT, group: "Skyward", label: "Version", desc: "", keywords: "version build", kind: Kind::Info(Info::Version), ..BASE },
        Item { id: "gpu", cats: ABOUT, group: "Skyward", label: "Graphics adapter", desc: "", keywords: "gpu graphics card adapter", kind: Kind::Info(Info::Gpu), ..BASE },
        Item { id: "licence", cats: ABOUT, group: "Skyward", label: "Licence", desc: "", keywords: "licence license gpl", kind: Kind::Info(Info::Licence), ..BASE },
        Item {
            id: "credits",
            cats: ABOUT,
            group: "Skyward",
            label: "Credits",
            desc: "Map data, aircraft, sounds, fonts and services",
            keywords: "credits attribution thanks",
            kind: Kind::Action { act: Act::Credits, button: "View", confirm: false },
            ..BASE
        },
    ]
}

pub fn item(id: &str) -> Option<&'static Item> {
    items().iter().find(|i| i.id == id)
}

/// Whether a row is shown on this build / screen.
pub fn available(i: &Item, in_flight: bool) -> bool {
    !(i.native_only && crate::platform::WEB) && (!i.in_flight_only || in_flight)
}

/// The rows of a category, in order.
pub fn rows(cat: Cat, in_flight: bool) -> Vec<&'static Item> {
    items().iter().filter(|i| i.cats.contains(&cat) && available(i, in_flight)).collect()
}

/// Search every category: rows whose label, description, keywords or
/// category match all the words of `query`, best first (label hits first).
pub fn search(query: &str, in_flight: bool) -> Vec<&'static Item> {
    let words: Vec<String> = query.split_whitespace().map(|w| w.to_lowercase()).collect();
    if words.is_empty() {
        return Vec::new();
    }
    let mut hits: Vec<(u32, usize, &'static Item)> = Vec::new();
    for (n, i) in items().iter().enumerate().filter(|(_, i)| available(i, in_flight)) {
        let label = i.label.to_lowercase();
        let cats: String = i.cats.iter().map(|c| c.name().to_lowercase() + " ").collect();
        let hay = format!("{label} {} {} {cats} {}", i.desc.to_lowercase(), i.keywords, i.group.to_lowercase());
        if !words.iter().all(|w| hay.contains(w.as_str())) {
            continue;
        }
        let first = &words[0];
        let score = if label.starts_with(first.as_str()) {
            0
        } else if label.split_whitespace().any(|t| t.starts_with(first.as_str())) {
            1
        } else if label.contains(first.as_str()) {
            2
        } else if i.keywords.split_whitespace().any(|k| k.starts_with(first.as_str())) {
            3
        } else {
            4
        };
        hits.push((score, n, i));
    }
    hits.sort_by_key(|h| (h.0, h.1));
    hits.into_iter().map(|h| h.2).collect()
}

/// Categories whose own keywords match (search offers to open them: Video, Controls, About).
pub fn search_categories(query: &str) -> Vec<Cat> {
    let words: Vec<String> = query.split_whitespace().map(|w| w.to_lowercase()).collect();
    if words.is_empty() {
        return Vec::new();
    }
    Cat::ALL.into_iter().filter(|c| !c.keywords().is_empty() && words.iter().all(|w| c.keywords().contains(w.as_str()) || c.name().to_lowercase().contains(w.as_str()))).collect()
}

/// The row differs from `base` (the settings when the screen opened / last applied).
pub fn changed(i: &Item, cur: &Settings, base: &Settings) -> bool {
    i.kind.has_value() && (i.get)(cur) != (i.get)(base)
}

/// The row is not at its default.
pub fn modified(i: &Item, cur: &Settings) -> bool {
    i.kind.has_value() && (i.get)(cur) != (i.get)(&Settings::default())
}

/// Any pending change in a category (its rows, or the Controls bindings).
pub fn cat_changed(cat: Cat, cur: &Settings, base: &Settings) -> bool {
    (cat == Cat::Controls && cur.controls != base.controls) || items().iter().filter(|i| i.cats.contains(&cat)).any(|i| changed(i, cur, base))
}

/// Reset a category's rows (and the Controls bindings) to their defaults.
pub fn reset_category(s: &mut Settings, cat: Cat) {
    let d = Settings::default();
    for i in items().iter().filter(|i| i.cats.contains(&cat) && i.kind.has_value()) {
        // (The player's name is theirs, not a preference with a default.)
        if i.id == "player_name" {
            continue;
        }
        (i.set)(s, (i.get)(&d));
    }
    if cat == Cat::Controls {
        s.controls = d.controls;
    }
}

/// Reset every category. Flight choices (last flight, recents, favourites,
/// time of day / METAR) and other tracks' unknown keys are kept.
pub fn reset_all(s: &mut Settings) {
    for c in Cat::ALL {
        reset_category(s, c);
    }
}

/// Rows that differ between two settings and need a restart.
pub fn restart_needed(cur: &Settings, at_start: &Settings) -> Vec<&'static Item> {
    items().iter().filter(|i| i.restart && changed(i, cur, at_start)).collect()
}

/// Keyboard / gamepad focus order within a category: its value and button
/// rows top to bottom (read-only rows are skipped), then the footer.
pub fn nav_order(cat: Cat, in_flight: bool) -> Vec<&'static str> {
    let mut v: Vec<&'static str> = rows(cat, in_flight).into_iter().filter(|i| !matches!(i.kind, Kind::Info(_))).map(|i| i.id).collect();
    v.extend(FOOTER);
    v
}

/// Footer buttons in focus order.
pub const FOOTER: [&str; 4] = ["@reset_cat", "@reset_all", "@revert", "@apply"];

/// Words masked by the multiplayer chat filter (a short list of the common
/// English ones; matching ignores case and stray punctuation).
const BLOCKED: &[&str] = &["fuck", "shit", "cunt", "bitch", "bastard", "wanker", "twat", "dick", "prick", "asshole", "arsehole", "slut", "whore", "fag", "nigger", "retard"];

/// `text` with blocked words replaced by asterisks.
pub fn filter_chat(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for (k, word) in text.split(' ').enumerate() {
        if k > 0 {
            out.push(' ');
        }
        let core: String = word.chars().filter(|c| c.is_alphanumeric()).collect::<String>().to_lowercase();
        if BLOCKED.iter().any(|b| core.starts_with(b)) {
            out.extend(word.chars().map(|c| if c.is_alphanumeric() { '*' } else { c }));
        } else {
            out.push_str(word);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_unique_and_rows_complete() {
        let mut seen = std::collections::HashSet::new();
        for i in items() {
            assert!(seen.insert(i.id), "duplicate id {}", i.id);
            assert!(!i.cats.is_empty(), "{}", i.id);
            assert!(!i.label.is_empty(), "{}", i.id);
            if i.kind.has_value() {
                assert!(!i.desc.is_empty(), "{} needs a one-line description", i.id);
                // A value row reads back what it writes.
                let mut s = Settings::default();
                let v = (i.get)(&s);
                (i.set)(&mut s, v.clone());
                assert_eq!((i.get)(&s), v, "{}", i.id);
            }
            if let Kind::Choice(o) = i.kind {
                assert!(!o.is_empty());
            }
        }
        // Every category but Controls (its own page) has rows of its own.
        for c in Cat::ALL.into_iter().filter(|c| *c != Cat::Controls) {
            assert!(!rows(c, true).is_empty(), "{c:?} is empty");
        }
    }

    #[test]
    fn search_finds_across_categories() {
        let ids = |q: &str| search(q, true).iter().map(|i| i.id).collect::<Vec<_>>();
        // Label match first.
        assert_eq!(ids("fov").first(), Some(&"cockpit_fov"));
        assert!(ids("field of view").contains(&"chase_fov"));
        // Keywords and descriptions.
        assert!(ids("colorblind").contains(&"palette"));
        assert!(ids("blackout").contains(&"g_effects"));
        assert!(ids("inhg").contains(&"pressure_unit"));
        // Case and word order do not matter; all words must match.
        assert_eq!(ids("SUBTITLES"), ids("subtitles"));
        assert!(ids("volume engine").contains(&"vol_engines"));
        assert!(!ids("volume engine").contains(&"vol_radio"));
        // Rows shared by several categories appear once.
        let scale = ids("interface scale");
        assert_eq!(scale.iter().filter(|i| **i == "ui_scale").count(), 1);
        assert!(ids("zzzz").is_empty());
        assert!(ids("   ").is_empty());
        // Category-only matches (Video / Controls own their pages).
        assert!(search_categories("rebind").contains(&Cat::Controls));
        assert!(search_categories("graphics").contains(&Cat::Video));
        // In-flight-only rows only in flight.
        assert!(search("failures panel", true).iter().any(|i| i.id == "open_failures"));
        assert!(!search("failures panel", false).iter().any(|i| i.id == "open_failures"));
    }

    #[test]
    fn reset_semantics() {
        let mut s = Settings::default();
        s.camera.cockpit_fov = 90.0;
        s.ui_scale = 1.5;
        s.palette = Palette::Protanopia;
        s.sound.master = 0.2;
        s.controls.mouse.look_sens = 2.0;
        s.realism.unlimited_fuel = true;
        s.push_recent("EGLL");
        s.toggle_favourite("EGKB");
        s.conditions.metar = "EGLL 121150Z 24010KT 9999 FEW030 15/08 Q1015".into();
        s.player_name = "Maverick".into();
        // Camera: only camera rows.
        reset_category(&mut s, Cat::Camera);
        assert_eq!(s.camera.cockpit_fov, 72.0);
        assert_eq!(s.controls.mouse.look_sens, 1.0, "head-look speed is a Camera row");
        assert_eq!(s.ui_scale, 1.5);
        assert!(s.realism.unlimited_fuel);
        // A row shared by Display and Accessibility resets from either.
        reset_category(&mut s, Cat::Accessibility);
        assert_eq!(s.ui_scale, 1.0);
        assert_eq!(s.palette, Palette::Standard);
        // Reset all: every row at its default, flight choices and the name kept.
        reset_all(&mut s);
        let d = Settings::default();
        for i in items().iter().filter(|i| i.kind.has_value() && i.id != "player_name") {
            assert_eq!((i.get)(&s), (i.get)(&d), "{}", i.id);
        }
        assert_eq!(s.recents, vec!["EGLL"]);
        assert_eq!(s.favourites, vec!["EGKB"]);
        assert!(!s.conditions.metar.is_empty());
        assert_eq!(s.player_name, "Maverick");
        assert_eq!(s.controls, d.controls);
    }

    #[test]
    fn changed_and_restart() {
        let base = Settings::default();
        let mut s = base.clone();
        assert!(!Cat::ALL.iter().any(|c| cat_changed(*c, &s, &base)));
        s.data.cache_limit_mb = 2048;
        s.sound.engines = 0.5;
        assert!(cat_changed(Cat::Data, &s, &base));
        assert!(cat_changed(Cat::Sound, &s, &base));
        assert!(!cat_changed(Cat::Camera, &s, &base));
        assert!(changed(item("vol_engines").unwrap(), &s, &base));
        assert!(modified(item("vol_engines").unwrap(), &s));
        assert_eq!(restart_needed(&s, &base).iter().map(|i| i.id).collect::<Vec<_>>(), ["cache_limit"]);
        s.controls.mouse.yoke = true;
        assert!(cat_changed(Cat::Controls, &s, &base));
    }

    #[test]
    fn realism_preset_row() {
        let i = item("realism_preset").unwrap();
        let mut s = Settings::default();
        assert_eq!((i.get)(&s), Val::I(1));
        s.realism.random_failures = true;
        (i.set)(&mut s, Val::I(0));
        assert_eq!(s.realism_preset(), RealismPreset::Casual);
        assert!(s.realism.unlimited_fuel && s.crashes == crate::damage::Realism::Forgiving && s.structural_damage == crate::damage::Realism::Off);
        assert!(s.realism.random_failures, "the preset keeps the random-failures choice");
        // Custom is shown, not chosen.
        (i.set)(&mut s, Val::I(3));
        assert_eq!(s.realism_preset(), RealismPreset::Casual);
        s.realism.auto_trim = false;
        assert_eq!((i.get)(&s), Val::I(3));
    }

    #[test]
    fn navigation_order() {
        let order = nav_order(Cat::Data, false);
        // Read-only rows (location, size, file) are skipped; the footer comes last.
        assert!(!order.contains(&"cache_path") && !order.contains(&"cache_size"));
        let rows: Vec<_> = order.iter().copied().filter(|i| !i.starts_with('@')).collect();
        assert_eq!(rows, ["cache_limit", "clear_tiles", "clear_terrain", "offline"]);
        assert_eq!(&order[order.len() - 4..], &FOOTER);
        // Rows keep their on-screen order.
        let cam = nav_order(Cat::Camera, false);
        assert_eq!(&cam[..3], &["cockpit_fov", "chase_fov", "look_sens"]);
        // In-flight extras slot in where they are drawn.
        let f = nav_order(Cat::Flight, true);
        let at = |id: &str| f.iter().position(|x| *x == id).unwrap();
        assert!(at("random_failures") + 1 == at("open_failures"));
        assert!(!nav_order(Cat::Flight, false).contains(&"open_failures"));
    }

    #[test]
    fn chat_filter() {
        assert_eq!(filter_chat("nice landing"), "nice landing");
        assert_eq!(filter_chat("what the Fuck!"), "what the ****!");
        assert_eq!(filter_chat("Scunthorpe tower"), "Scunthorpe tower");
        assert_eq!(filter_chat(""), "");
    }
}
