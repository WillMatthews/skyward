//! Persisted preferences: video, display & HUD, flight & realism, assists,
//! weather & time, traffic, camera, sound, controls, multiplayer, data and
//! accessibility, plus the last flight and recent / favourite airports.
//! A small JSON file in the user's config directory
//! (`$SKYWARD_CONFIG_DIR/settings.json`, else `$XDG_CONFIG_HOME/skyward/`,
//! else `~/.config/skyward/`, else `cache/`); `localStorage` in the browser.
//!
//! The file is versioned ([`VERSION`]) and read forgivingly: missing keys take
//! their defaults (older files), keys this build does not know are kept and
//! written back (newer files, other tracks' settings).
//!
//! The settings screen is `ui/settings_ui.rs`; its catalogue of rows (search,
//! reset, restart badges) is `ui/settings_schema.rs`.

use super::{AltUnit, SpeedUnit};
use serde_json::{Map, Value, json};
use std::path::PathBuf;

/// Settings file format version (1: before the settings redesign, no "version" key).
pub const VERSION: u64 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HudMode {
    #[default]
    Full,
    Minimal,
    Off,
}

impl HudMode {
    pub fn next(self) -> HudMode {
        match self {
            HudMode::Full => HudMode::Minimal,
            HudMode::Minimal => HudMode::Off,
            HudMode::Off => HudMode::Full,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            HudMode::Full => "Full",
            HudMode::Minimal => "Minimal",
            HudMode::Off => "Off",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MapStyle {
    /// Day by day, night by night (sun below −4°).
    #[default]
    Auto,
    Day,
    Night,
}

/// Barometric pressure unit for QNH read-outs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PressureUnit {
    #[default]
    Hpa,
    InHg,
}

impl PressureUnit {
    pub fn fmt(self, hpa: f32) -> String {
        match self {
            PressureUnit::Hpa => format!("{hpa:.0} hPa"),
            PressureUnit::InHg => format!("{:.2} inHg", hpa * 0.029_53),
        }
    }
    /// METAR-style short form ("Q1013" / "A2992").
    pub fn short(self, hpa: f32) -> String {
        match self {
            PressureUnit::Hpa => format!("Q{hpa:.0}"),
            PressureUnit::InHg => format!("A{:.0}", hpa * 2.953),
        }
    }
}

/// Temperature unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TempUnit {
    #[default]
    Celsius,
    Fahrenheit,
}

impl TempUnit {
    pub fn conv(self, c: f32) -> f32 {
        match self {
            TempUnit::Celsius => c,
            TempUnit::Fahrenheit => c * 1.8 + 32.0,
        }
    }
    pub fn fmt(self, c: f32) -> String {
        format!("{:.0} {}", self.conv(c), self.label())
    }
    pub fn label(self) -> &'static str {
        match self {
            TempUnit::Celsius => "°C",
            TempUnit::Fahrenheit => "°F",
        }
    }
}

/// Interface colour palette (accessibility; ui/palette.rs).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Hash)]
pub enum Palette {
    #[default]
    Standard,
    /// Red-green (green-weak, the most common).
    Deuteranopia,
    /// Red-green (red-weak: reds look dark).
    Protanopia,
    /// Blue-yellow.
    Tritanopia,
}

impl Palette {
    pub const ALL: [Palette; 4] = [Palette::Standard, Palette::Deuteranopia, Palette::Protanopia, Palette::Tritanopia];
    pub fn id(self) -> &'static str {
        match self {
            Palette::Standard => "standard",
            Palette::Deuteranopia => "deuteranopia",
            Palette::Protanopia => "protanopia",
            Palette::Tritanopia => "tritanopia",
        }
    }
    pub fn parse(s: &str) -> Option<Palette> {
        Palette::ALL.into_iter().find(|p| p.id() == s.to_ascii_lowercase())
    }
}

/// Realism presets (Flight & Realism). `Custom` once a knob differs from a preset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RealismPreset {
    Casual,
    #[default]
    Standard,
    Hard,
    Custom,
}

/// Autobrake setting armed for landing by default.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Autobrake {
    #[default]
    Off,
    Low,
    Medium,
    Max,
}

/// Structural damage model (Flight & Realism). Consumers: damage.rs (owned by DMG).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StructuralDamage {
    /// Published limits: overstress, flutter and overspeed tear parts off.
    #[default]
    Realistic,
    /// Higher load margins before anything bends or breaks.
    Forgiving,
    /// Indestructible airframe.
    Off,
}

/// What an impact does (Flight & Realism). Consumers: flight.rs / sim.rs crash code (owned by DMG).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Crashes {
    /// A hard enough impact is a crash, with a wreck and a debrief.
    #[default]
    Realistic,
    /// Hard landings are survivable; only real impacts crash.
    Forgiving,
    /// No crashes: the aircraft resets to the start instead.
    Off,
}

impl StructuralDamage {
    pub const ALL: [StructuralDamage; 3] = [StructuralDamage::Realistic, StructuralDamage::Forgiving, StructuralDamage::Off];
    pub fn id(self) -> &'static str {
        match self {
            StructuralDamage::Realistic => "realistic",
            StructuralDamage::Forgiving => "forgiving",
            StructuralDamage::Off => "off",
        }
    }
}

impl Crashes {
    pub const ALL: [Crashes; 3] = [Crashes::Realistic, Crashes::Forgiving, Crashes::Off];
    pub fn id(self) -> &'static str {
        match self {
            Crashes::Realistic => "realistic",
            Crashes::Forgiving => "forgiving",
            Crashes::Off => "off",
        }
    }
}

/// Flight & Realism.
#[derive(Clone, Debug, PartialEq)]
pub struct Realism {
    /// Structural damage: Realistic / Forgiving / Off. // consumer owned by DMG.
    pub structural_damage: StructuralDamage,
    /// Crashes: Realistic / Forgiving / Off (resets). // consumer owned by DMG.
    pub crashes: Crashes,
    /// Engines can be torn off by overload or impact. // consumer owned by DMG (pending).
    pub engine_damage: bool,
    /// Grey-out / blackout and red-out at high positive / negative g (screen effect).
    pub g_effects: bool,
    /// Random failures by default for new flights (the failures panel's MTBF window).
    pub random_failures: bool,
    /// Tanks never drain below what the flight started with.
    pub unlimited_fuel: bool,
    /// Rudder coordinates turns when the pedals are centred.
    pub auto_rudder: bool,
    /// Trim holds the attitude when the stick is released.
    pub auto_trim: bool,
    /// Autobrake armed for landing by default. // owned by GH (ground handling): consumer pending.
    pub autobrake: Autobrake,
}

impl Default for Realism {
    fn default() -> Self {
        Realism::preset(RealismPreset::Standard)
    }
}

impl Realism {
    pub fn preset(p: RealismPreset) -> Realism {
        use {Crashes as C, StructuralDamage as D};
        let (structure, crash, engines, g, auto) = match p {
            RealismPreset::Casual => (D::Off, C::Forgiving, false, false, true),
            RealismPreset::Hard => (D::Realistic, C::Realistic, true, true, false),
            // (Standard is the defaults: how Skyward flew before the presets.)
            _ => (D::Realistic, C::Realistic, true, false, false),
        };
        Realism {
            structural_damage: structure,
            crashes: crash,
            engine_damage: engines,
            g_effects: g,
            random_failures: false,
            unlimited_fuel: p == RealismPreset::Casual,
            auto_rudder: auto,
            auto_trim: auto,
            autobrake: if p == RealismPreset::Casual { Autobrake::Medium } else { Autobrake::Off },
        }
    }
    /// The preset these knobs match (Custom: none).
    pub fn current_preset(&self) -> RealismPreset {
        [RealismPreset::Casual, RealismPreset::Standard, RealismPreset::Hard]
            .into_iter()
            .find(|p| Realism { random_failures: self.random_failures, ..Realism::preset(*p) } == *self)
            .unwrap_or(RealismPreset::Custom)
    }
}

/// Assists.
#[derive(Clone, Debug, PartialEq)]
pub struct Assists {
    /// A ribbon along the cleared taxi route. // owned by GH (ground handling): consumer pending.
    pub taxi_ribbon: bool,
    /// Glide-path guidance cue on final. // owned by GH / F follow-up: consumer pending.
    pub landing_guidance: bool,
    /// Big STALL / OVERSPEED cues on screen (the aircraft's own warnings stay).
    pub stall_warning: bool,
    pub overspeed_warning: bool,
}

impl Default for Assists {
    fn default() -> Self {
        Assists { taxi_ribbon: false, landing_guidance: false, stall_warning: true, overspeed_warning: true }
    }
}

/// Weather & Time.
#[derive(Clone, Debug, PartialEq)]
pub struct WxPrefs {
    /// Turbulence intensity 0..1 (0.5 = as reported). // owned by WX: consumer pending.
    pub turbulence: f32,
    /// Highest time acceleration the W key cycles to (1, 2, 4, 8, 16).
    pub max_warp: u32,
}

impl Default for WxPrefs {
    fn default() -> Self {
        WxPrefs { turbulence: 0.5, max_warp: 8 }
    }
}

/// Traffic.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficPrefs {
    /// Road traffic density 0..1. // owned by RT: consumer pending.
    pub road_density: f32,
    /// ADS-B live traffic as AI aircraft. // owned by X: consumer pending.
    pub adsb: bool,
    /// ADS-B radius (nm). // owned by X: consumer pending.
    pub adsb_radius_nm: f32,
    /// Parked aircraft at stands 0..1. // owned by A (airport fleet): consumer pending.
    pub parked_density: f32,
}

impl Default for TrafficPrefs {
    fn default() -> Self {
        TrafficPrefs { road_density: 0.6, adsb: false, adsb_radius_nm: 40.0, parked_density: 1.0 }
    }
}

/// First view of a new flight.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DefaultView {
    #[default]
    Chase,
    Cockpit,
}

/// Camera.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraPrefs {
    /// Vertical field of view in the cockpit / outside (degrees).
    pub cockpit_fov: f32,
    pub chase_fov: f32,
    /// The pilot's head moves with acceleration in the cockpit.
    pub head_motion: bool,
    /// Chase camera distance (× the type's default) and how lazily it follows the turn (0 stiff .. 1 lazy).
    pub chase_distance: f32,
    pub chase_lag: f32,
    pub default_view: DefaultView,
}

impl Default for CameraPrefs {
    fn default() -> Self {
        CameraPrefs { cockpit_fov: 72.0, chase_fov: 60.0, head_motion: true, chase_distance: 1.0, chase_lag: 0.5, default_view: DefaultView::Chase }
    }
}

/// Multiplayer (the player name lives in `multiplayer.json`, net/prefs.rs).
#[derive(Clone, Debug, PartialEq)]
pub struct MpPrefs {
    /// Port tried when an address has none.
    pub default_port: u16,
    /// Name tags over other aircraft when joining.
    pub name_tags: bool,
    /// Mask offensive words in chat.
    pub chat_filter: bool,
}

impl Default for MpPrefs {
    fn default() -> Self {
        MpPrefs { default_port: skyward_net::proto::DEFAULT_PORT, name_tags: true, chat_filter: true }
    }
}

/// Data & Storage.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct DataPrefs {
    /// Tile / terrain cache limit in MB (0: unlimited); oldest files are pruned at start-up.
    pub cache_limit_mb: u32,
    /// No network: cached scenery, no live weather / Overpass downloads.
    pub offline: bool,
}

/// Accessibility (UI scale, subtitles and palettes are shared with Display & HUD).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Access {
    /// Keeps the cockpit camera still (no head motion); more motion effects will follow it.
    pub reduced_motion: bool,
    /// Wheel brakes latch on a press instead of while held.
    pub toggle_brakes: bool,
}

/// Conditions picked before a flight.
#[derive(Clone, Debug, PartialEq)]
pub struct Conditions {
    pub live: bool,
    /// Local clock hours.
    pub hours: f32,
    /// Day of year.
    pub day: u32,
    /// Cloud preset name ("Scattered", ...), used when not live and no METAR.
    pub preset: String,
    /// Custom METAR (not live): overrides the preset when it decodes.
    pub metar: String,
}

impl Default for Conditions {
    fn default() -> Self {
        Conditions { live: true, hours: crate::sim::DEFAULT_HOURS, day: crate::sim::DEFAULT_DAY, preset: "Scattered".into(), metar: String::new() }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LastFlight {
    pub aircraft: String,
    pub airport: String,
    pub start: String,
    pub watch: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub ui_scale: f32,
    pub hud: HudMode,
    /// Present with vsync (FIFO). Off = mailbox / immediate (lowest latency, may tear or judder).
    pub vsync: bool,
    /// Scenery detail: 0 low, 1 medium, 2 high, 3 ultra (tile load radius).
    pub detail: u8,
    pub speed: SpeedUnit,
    pub alt: AltUnit,
    pub pressure: PressureUnit,
    pub temp: TempUnit,
    pub map_style: MapStyle,
    pub north_up: bool,
    /// ATC / ATIS / crew subtitles.
    pub subtitles: bool,
    pub palette: Palette,
    pub high_contrast: bool,
    /// Interface language (only "en" so far; strings are not yet externalised).
    pub language: String,
    pub sound: crate::audio::Settings,
    /// Output device id ("" = the system default).
    pub audio_device: String,
    pub mute_in_background: bool,
    /// Key / controller bindings, device profiles, mouse (input/).
    pub controls: crate::input::Config,
    pub realism: Realism,
    pub assists: Assists,
    pub wx: WxPrefs,
    pub traffic: TrafficPrefs,
    pub camera: CameraPrefs,
    pub mp: MpPrefs,
    /// Multiplayer name: lives in `multiplayer.json` (net/prefs.rs), not in this
    /// file; mirrored here so the settings screen can Apply / Revert it.
    pub player_name: String,
    pub data: DataPrefs,
    pub access: Access,
    pub conditions: Conditions,
    pub last: Option<LastFlight>,
    pub recents: Vec<String>,
    pub favourites: Vec<String>,
    /// Keys this build does not know (a newer file, other tracks): written back untouched.
    pub extra: Map<String, Value>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            ui_scale: 1.0,
            hud: HudMode::Full,
            vsync: true,
            // The browser streams on the main thread: lighter by default.
            detail: if crate::platform::WEB { 0 } else { 2 },
            speed: SpeedUnit::Knots,
            alt: AltUnit::Feet,
            pressure: PressureUnit::Hpa,
            temp: TempUnit::Celsius,
            map_style: MapStyle::Auto,
            north_up: true,
            subtitles: true,
            palette: Palette::Standard,
            high_contrast: false,
            language: "en".into(),
            sound: crate::audio::Settings::default(),
            audio_device: String::new(),
            mute_in_background: false,
            controls: crate::input::Config::default(),
            realism: Realism::default(),
            assists: Assists::default(),
            wx: WxPrefs::default(),
            traffic: TrafficPrefs::default(),
            camera: CameraPrefs::default(),
            mp: MpPrefs::default(),
            player_name: String::new(),
            data: DataPrefs::default(),
            access: Access::default(),
            conditions: Conditions::default(),
            last: None,
            recents: Vec::new(),
            favourites: Vec::new(),
            extra: Map::new(),
        }
    }
}

/// The settings file. `SKYWARD_CONFIG_DIR` overrides the directory (tests,
/// headless runs: never the player's own file).
pub fn path() -> PathBuf {
    if let Some(d) = std::env::var_os("SKYWARD_CONFIG_DIR").filter(|d| !d.is_empty()) {
        return PathBuf::from(d).join("settings.json");
    }
    match std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config"))) {
        Some(d) => d.join("skyward").join("settings.json"),
        None => PathBuf::from("cache").join("settings.json"),
    }
}

/// Tile load radius (m) for a detail level.
pub fn detail_radius(d: u8) -> f32 {
    match d {
        0 => 4500.0,
        1 => 6000.0,
        2 => 7500.0,
        _ => 9500.0,
    }
}

pub const DETAIL_NAMES: [&str; 4] = ["Low", "Medium", "High", "Ultra"];

/// Interface scales offered (the layout is designed for 100–200 %).
pub const UI_SCALES: [f32; 8] = [0.8, 0.9, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0];
pub const UI_SCALE_MIN: f32 = 0.7;
pub const UI_SCALE_MAX: f32 = 2.0;

/// What the simulation reads from the settings each frame (sim.rs `prefs`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimPrefs {
    pub structural_damage: StructuralDamage,
    pub crashes: Crashes,
    pub engine_damage: bool,
    pub unlimited_fuel: bool,
    pub auto_rudder: bool,
    pub auto_trim: bool,
    pub random_failures: bool,
    pub max_warp: u32,
    /// Degrees.
    pub cockpit_fov: f32,
    pub chase_fov: f32,
    pub head_motion: bool,
    pub chase_distance: f32,
    pub chase_lag: f32,
    pub cockpit_first: bool,
    pub toggle_brakes: bool,
}

impl Default for SimPrefs {
    fn default() -> Self {
        Settings::default().sim_prefs()
    }
}

/// Small typed reads from a JSON object (missing / wrong type: the default).
#[derive(Clone, Copy)]
struct J<'a>(&'a Value);

impl<'a> J<'a> {
    fn get(self, k: &str) -> Option<&'a Value> {
        self.0.get(k).filter(|v| !v.is_null())
    }
    fn b(self, k: &str, d: bool) -> bool {
        self.get(k).and_then(Value::as_bool).unwrap_or(d)
    }
    fn f(self, k: &str, d: f32) -> f32 {
        self.get(k).and_then(Value::as_f64).map(|x| x as f32).filter(|x| x.is_finite()).unwrap_or(d)
    }
    fn s(self, k: &str) -> &'a str {
        self.get(k).and_then(Value::as_str).unwrap_or("")
    }
    fn o(self, k: &str) -> J<'a> {
        static NULL: Value = Value::Null;
        J(self.get(k).unwrap_or(&NULL))
    }
}

/// Top-level keys this build reads and writes (the rest is kept in `extra`).
const KNOWN: &[&str] = &[
    "version",
    "ui_scale",
    "hud",
    "vsync",
    "detail",
    "speed",
    "alt",
    "pressure",
    "temp",
    "map_style",
    "north_up",
    "subtitles",
    "palette",
    "high_contrast",
    "language",
    "sound",
    "audio_device",
    "mute_in_background",
    "controls",
    "realism",
    "assists",
    "weather",
    "traffic",
    "camera",
    "multiplayer",
    "data",
    "accessibility",
    "conditions",
    "last",
    "recents",
    "favourites",
];

impl Settings {
    pub fn load() -> Settings {
        // Browser: localStorage.
        #[cfg(target_arch = "wasm32")]
        let Some(text) = crate::platform::web::storage_get("skyward.settings") else { return Settings::default() };
        #[cfg(not(target_arch = "wasm32"))]
        let Ok(text) = std::fs::read_to_string(path()) else { return Settings::default() };
        Settings::from_str(&text)
    }

    /// Parse a settings file (any version; garbage: the defaults).
    pub fn from_str(text: &str) -> Settings {
        match serde_json::from_str::<Value>(text) {
            Ok(v) if v.is_object() => Settings::from_json(&v),
            _ => Settings::default(),
        }
    }

    pub fn save(&self) {
        #[cfg(target_arch = "wasm32")]
        crate::platform::web::storage_set("skyward.settings", &serde_json::to_string(&self.to_json()).unwrap_or_default());
        #[cfg(not(target_arch = "wasm32"))]
        {
            let p = path();
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let tmp = p.with_extension("tmp");
            if std::fs::write(&tmp, serde_json::to_string_pretty(&self.to_json()).unwrap_or_default()).is_ok() {
                let _ = std::fs::rename(&tmp, &p);
            }
        }
    }

    pub fn push_recent(&mut self, ident: &str) {
        self.recents.retain(|r| r != ident);
        self.recents.insert(0, ident.to_string());
        self.recents.truncate(12);
    }

    pub fn toggle_favourite(&mut self, ident: &str) {
        if let Some(i) = self.favourites.iter().position(|f| f == ident) {
            self.favourites.remove(i);
        } else {
            self.favourites.push(ident.to_string());
        }
    }

    pub fn is_favourite(&self, ident: &str) -> bool {
        self.favourites.iter().any(|f| f == ident)
    }

    /// What the simulation needs (copied into `Sim::prefs` every frame).
    pub fn sim_prefs(&self) -> SimPrefs {
        let r = &self.realism;
        let c = &self.camera;
        SimPrefs {
            structural_damage: r.structural_damage,
            crashes: r.crashes,
            engine_damage: r.engine_damage,
            unlimited_fuel: r.unlimited_fuel,
            auto_rudder: r.auto_rudder,
            auto_trim: r.auto_trim,
            random_failures: r.random_failures,
            max_warp: self.wx.max_warp.clamp(1, 16),
            cockpit_fov: c.cockpit_fov,
            chase_fov: c.chase_fov,
            head_motion: c.head_motion && !self.access.reduced_motion,
            chase_distance: c.chase_distance,
            chase_lag: c.chase_lag,
            cockpit_first: c.default_view == DefaultView::Cockpit,
            toggle_brakes: self.access.toggle_brakes,
        }
    }

    pub fn to_json(&self) -> Value {
        let s = &self.sound;
        let c = &self.conditions;
        let r = &self.realism;
        let a = &self.assists;
        let t = &self.traffic;
        let cam = &self.camera;
        let mut v = json!({
            "version": VERSION,
            "ui_scale": self.ui_scale,
            "hud": self.hud.name(),
            "vsync": self.vsync,
            "detail": self.detail,
            "speed": match self.speed { SpeedUnit::Knots => "kt", SpeedUnit::Kmh => "kmh", SpeedUnit::Mph => "mph" },
            "alt": match self.alt { AltUnit::Feet => "ft", AltUnit::Metres => "m" },
            "pressure": match self.pressure { PressureUnit::Hpa => "hpa", PressureUnit::InHg => "inhg" },
            "temp": match self.temp { TempUnit::Celsius => "c", TempUnit::Fahrenheit => "f" },
            "map_style": match self.map_style { MapStyle::Auto => "auto", MapStyle::Day => "day", MapStyle::Night => "night" },
            "north_up": self.north_up,
            "subtitles": self.subtitles,
            "palette": self.palette.id(),
            "high_contrast": self.high_contrast,
            "language": self.language,
            "sound": { "muted": s.muted, "master": s.master, "engines": s.engines, "environment": s.environment, "systems": s.systems, "voice": s.voice, "ui": s.ui, "radio": s.radio },
            "audio_device": self.audio_device,
            "mute_in_background": self.mute_in_background,
            "controls": self.controls.to_json(),
            "realism": {
                "structural_damage": r.structural_damage.id(), "crashes": r.crashes.id(), "engine_damage": r.engine_damage, "g_effects": r.g_effects,
                "random_failures": r.random_failures, "unlimited_fuel": r.unlimited_fuel, "auto_rudder": r.auto_rudder, "auto_trim": r.auto_trim,
                "autobrake": match r.autobrake { Autobrake::Off => "off", Autobrake::Low => "low", Autobrake::Medium => "medium", Autobrake::Max => "max" },
            },
            "assists": { "taxi_ribbon": a.taxi_ribbon, "landing_guidance": a.landing_guidance, "stall_warning": a.stall_warning, "overspeed_warning": a.overspeed_warning },
            "weather": { "turbulence": self.wx.turbulence, "max_warp": self.wx.max_warp },
            "traffic": { "road_density": t.road_density, "adsb": t.adsb, "adsb_radius_nm": t.adsb_radius_nm, "parked_density": t.parked_density },
            "camera": {
                "cockpit_fov": cam.cockpit_fov, "chase_fov": cam.chase_fov, "head_motion": cam.head_motion, "chase_distance": cam.chase_distance, "chase_lag": cam.chase_lag,
                "default_view": match cam.default_view { DefaultView::Chase => "chase", DefaultView::Cockpit => "cockpit" },
            },
            "multiplayer": { "default_port": self.mp.default_port, "name_tags": self.mp.name_tags, "chat_filter": self.mp.chat_filter },
            "data": { "cache_limit_mb": self.data.cache_limit_mb, "offline": self.data.offline },
            "accessibility": { "reduced_motion": self.access.reduced_motion, "toggle_brakes": self.access.toggle_brakes },
            "conditions": { "live": c.live, "hours": c.hours, "day": c.day, "preset": c.preset, "metar": c.metar },
            "last": self.last.as_ref().map(|l| json!({ "aircraft": l.aircraft, "airport": l.airport, "start": l.start, "watch": l.watch })),
            "recents": self.recents,
            "favourites": self.favourites,
        });
        if let Some(o) = v.as_object_mut() {
            for (k, x) in &self.extra {
                o.entry(k.clone()).or_insert_with(|| x.clone());
            }
        }
        v
    }

    pub fn from_json(v: &Value) -> Settings {
        let d = Settings::default();
        let j = J(v);
        let list = |k: &str| v.get(k).and_then(|x| x.as_array()).map(|a| a.iter().filter_map(|s| s.as_str().map(String::from)).collect()).unwrap_or_default();
        let mut sound = d.sound;
        let s = j.o("sound");
        sound.muted = s.b("muted", sound.muted);
        sound.master = s.f("master", sound.master).clamp(0.0, 1.0);
        sound.engines = s.f("engines", sound.engines).clamp(0.0, 1.5);
        sound.environment = s.f("environment", sound.environment).clamp(0.0, 1.5);
        sound.systems = s.f("systems", sound.systems).clamp(0.0, 1.5);
        sound.voice = s.f("voice", sound.voice).clamp(0.0, 1.5);
        sound.radio = s.f("radio", sound.radio).clamp(0.0, 1.5);
        sound.ui = s.f("ui", sound.ui).clamp(0.0, 1.5);
        let c = j.o("conditions");
        let dc = &d.conditions;
        let conditions = Conditions {
            live: c.b("live", true),
            hours: c.f("hours", dc.hours).rem_euclid(24.0),
            day: c.get("day").and_then(Value::as_u64).map(|x| (x as u32).clamp(1, 366)).unwrap_or(dc.day),
            preset: Some(c.s("preset")).filter(|p| !p.is_empty()).unwrap_or("Scattered").to_string(),
            metar: c.s("metar").to_string(),
        };
        let last = v.get("last").filter(|l| l.is_object()).map(|l| {
            let l = J(l);
            LastFlight { aircraft: l.s("aircraft").into(), airport: l.s("airport").into(), start: l.s("start").into(), watch: l.b("watch", false) }
        });
        let r = j.o("realism");
        let dr = &d.realism;
        let realism = Realism {
            structural_damage: StructuralDamage::ALL.into_iter().find(|x| x.id() == r.s("structural_damage")).unwrap_or(dr.structural_damage),
            crashes: Crashes::ALL.into_iter().find(|x| x.id() == r.s("crashes")).unwrap_or(dr.crashes),
            engine_damage: r.b("engine_damage", dr.engine_damage),
            g_effects: r.b("g_effects", dr.g_effects),
            random_failures: r.b("random_failures", dr.random_failures),
            unlimited_fuel: r.b("unlimited_fuel", dr.unlimited_fuel),
            auto_rudder: r.b("auto_rudder", dr.auto_rudder),
            auto_trim: r.b("auto_trim", dr.auto_trim),
            autobrake: match r.s("autobrake") {
                "low" => Autobrake::Low,
                "medium" => Autobrake::Medium,
                "max" => Autobrake::Max,
                _ => Autobrake::Off,
            },
        };
        let a = j.o("assists");
        let da = &d.assists;
        let assists = Assists {
            taxi_ribbon: a.b("taxi_ribbon", da.taxi_ribbon),
            landing_guidance: a.b("landing_guidance", da.landing_guidance),
            stall_warning: a.b("stall_warning", da.stall_warning),
            overspeed_warning: a.b("overspeed_warning", da.overspeed_warning),
        };
        let w = j.o("weather");
        let wx = WxPrefs { turbulence: w.f("turbulence", d.wx.turbulence).clamp(0.0, 1.0), max_warp: (w.f("max_warp", d.wx.max_warp as f32) as u32).clamp(1, 16) };
        let t = j.o("traffic");
        let dt = &d.traffic;
        let traffic = TrafficPrefs {
            road_density: t.f("road_density", dt.road_density).clamp(0.0, 1.0),
            adsb: t.b("adsb", dt.adsb),
            adsb_radius_nm: t.f("adsb_radius_nm", dt.adsb_radius_nm).clamp(5.0, 250.0),
            parked_density: t.f("parked_density", dt.parked_density).clamp(0.0, 1.0),
        };
        let cm = j.o("camera");
        let dcam = &d.camera;
        let camera = CameraPrefs {
            cockpit_fov: cm.f("cockpit_fov", dcam.cockpit_fov).clamp(40.0, 110.0),
            chase_fov: cm.f("chase_fov", dcam.chase_fov).clamp(30.0, 100.0),
            head_motion: cm.b("head_motion", dcam.head_motion),
            chase_distance: cm.f("chase_distance", dcam.chase_distance).clamp(0.5, 3.0),
            chase_lag: cm.f("chase_lag", dcam.chase_lag).clamp(0.0, 1.0),
            default_view: if cm.s("default_view") == "cockpit" { DefaultView::Cockpit } else { DefaultView::Chase },
        };
        let m = j.o("multiplayer");
        let mp = MpPrefs {
            default_port: m.get("default_port").and_then(Value::as_u64).filter(|p| (1..=65535).contains(p)).map(|p| p as u16).unwrap_or(d.mp.default_port),
            name_tags: m.b("name_tags", d.mp.name_tags),
            chat_filter: m.b("chat_filter", d.mp.chat_filter),
        };
        let dd = j.o("data");
        let data = DataPrefs { cache_limit_mb: dd.get("cache_limit_mb").and_then(Value::as_u64).map(|x| x.min(1 << 20) as u32).unwrap_or(0), offline: dd.b("offline", false) };
        let ac = j.o("accessibility");
        let access = Access { reduced_motion: ac.b("reduced_motion", false), toggle_brakes: ac.b("toggle_brakes", false) };
        let extra = v.as_object().map(|o| o.iter().filter(|(k, _)| !KNOWN.contains(&k.as_str())).map(|(k, x)| (k.clone(), x.clone())).collect()).unwrap_or_default();
        Settings {
            ui_scale: j.f("ui_scale", 1.0).clamp(UI_SCALE_MIN, UI_SCALE_MAX),
            hud: match j.s("hud") {
                "Minimal" => HudMode::Minimal,
                "Off" => HudMode::Off,
                _ => HudMode::Full,
            },
            vsync: j.b("vsync", true),
            detail: (j.f("detail", d.detail as f32).max(0.0) as u8).min(3),
            speed: match j.s("speed") {
                "kmh" => SpeedUnit::Kmh,
                "mph" => SpeedUnit::Mph,
                _ => SpeedUnit::Knots,
            },
            alt: if j.s("alt") == "m" { AltUnit::Metres } else { AltUnit::Feet },
            pressure: if j.s("pressure") == "inhg" { PressureUnit::InHg } else { PressureUnit::Hpa },
            temp: if j.s("temp") == "f" { TempUnit::Fahrenheit } else { TempUnit::Celsius },
            map_style: match j.s("map_style") {
                "day" => MapStyle::Day,
                "night" => MapStyle::Night,
                _ => MapStyle::Auto,
            },
            north_up: j.b("north_up", true),
            subtitles: j.b("subtitles", true),
            palette: Palette::parse(j.s("palette")).unwrap_or_default(),
            high_contrast: j.b("high_contrast", false),
            language: Some(j.s("language")).filter(|l| !l.is_empty()).unwrap_or("en").to_string(),
            sound,
            audio_device: j.s("audio_device").to_string(),
            mute_in_background: j.b("mute_in_background", false),
            controls: v.get("controls").map(crate::input::Config::from_json).unwrap_or_default(),
            realism,
            assists,
            wx,
            traffic,
            camera,
            mp,
            player_name: String::new(),
            data,
            access,
            conditions,
            last,
            recents: list("recents"),
            favourites: list("favourites"),
            extra,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut s = Settings { hud: HudMode::Minimal, vsync: false, detail: 1, speed: SpeedUnit::Kmh, alt: AltUnit::Metres, ..Settings::default() };
        s.push_recent("EGLL");
        s.push_recent("EGLC");
        s.push_recent("EGLL");
        s.toggle_favourite("EGKB");
        s.last = Some(LastFlight { aircraft: "c172".into(), airport: "EGKB".into(), start: "runway:21".into(), watch: false });
        s.sound.master = 0.4;
        s.conditions.live = false;
        s.conditions.metar = "EGLL 121150Z 24010KT 9999 FEW030 15/08 Q1015".into();
        let back = Settings::from_json(&s.to_json());
        assert_eq!(back, s);
        assert_eq!(back.recents, vec!["EGLL", "EGLC"]);
    }

    /// Every new setting survives a save / load.
    #[test]
    fn round_trip_every_category() {
        let mut s = Settings {
            ui_scale: 1.75,
            pressure: PressureUnit::InHg,
            temp: TempUnit::Fahrenheit,
            subtitles: false,
            palette: Palette::Tritanopia,
            high_contrast: true,
            audio_device: "alsa:hw:1".into(),
            mute_in_background: true,
            ..Settings::default()
        };
        s.realism = Realism {
            random_failures: true,
            autobrake: Autobrake::Max,
            structural_damage: StructuralDamage::Forgiving,
            crashes: Crashes::Off,
            ..Realism::preset(RealismPreset::Hard)
        };
        s.realism.auto_trim = true;
        s.assists = Assists { taxi_ribbon: true, landing_guidance: true, stall_warning: false, overspeed_warning: false };
        s.wx = WxPrefs { turbulence: 0.8, max_warp: 16 };
        s.traffic = TrafficPrefs { road_density: 0.25, adsb: true, adsb_radius_nm: 80.0, parked_density: 0.5 };
        s.camera = CameraPrefs { cockpit_fov: 85.0, chase_fov: 50.0, head_motion: false, chase_distance: 1.5, chase_lag: 0.2, default_view: DefaultView::Cockpit };
        s.mp = MpPrefs { default_port: 50000, name_tags: false, chat_filter: false };
        s.data = DataPrefs { cache_limit_mb: 2048, offline: true };
        s.access = Access { reduced_motion: true, toggle_brakes: true };
        s.extra.insert("graphics".into(), json!({ "preset": "ultra" }));
        let text = serde_json::to_string_pretty(&s.to_json()).unwrap();
        let back = Settings::from_str(&text);
        assert_eq!(back, s);
        assert_eq!(back.realism.current_preset(), RealismPreset::Custom);
        assert!(text.contains("\"graphics\""), "unknown keys are written back");
    }

    /// A settings file from before the redesign (v1) loads with its values
    /// kept and every new setting at its default.
    #[test]
    fn migrates_v1_file() {
        let s = Settings::from_str(include_str!("fixtures/settings_v1.json"));
        assert_eq!(s.ui_scale, 1.15);
        assert_eq!(s.hud, HudMode::Minimal);
        assert!(!s.vsync);
        assert_eq!(s.detail, 3);
        assert_eq!(s.speed, SpeedUnit::Kmh);
        assert_eq!(s.alt, AltUnit::Metres);
        assert_eq!(s.map_style, MapStyle::Night);
        assert!(!s.north_up);
        assert!((s.sound.master - 0.55).abs() < 1e-6);
        // (v1 files written before the ATC track have no radio volume.)
        assert_eq!(s.sound.radio, crate::audio::Settings::default().radio);
        assert!(!s.conditions.live);
        assert_eq!(s.conditions.preset, "Overcast");
        assert_eq!(s.recents, vec!["EGLL", "EGKB"]);
        assert_eq!(s.favourites, vec!["LOWI"]);
        assert_eq!(s.last.as_ref().map(|l| l.aircraft.as_str()), Some("spitfire"));
        let d = Settings::default();
        assert_eq!((&s.realism, &s.assists, &s.camera, &s.traffic, &s.wx), (&d.realism, &d.assists, &d.camera, &d.traffic, &d.wx));
        assert_eq!((s.palette, s.pressure, s.temp, s.subtitles), (Palette::Standard, PressureUnit::Hpa, TempUnit::Celsius, true));
        assert!(s.extra.is_empty(), "{:?}", s.extra.keys().collect::<Vec<_>>());
        // Saved again it is a v2 file.
        assert_eq!(s.to_json()["version"], json!(VERSION));
        // The pre-redesign defaults are the realism "Standard" preset (same flying as before).
        assert_eq!(s.realism.current_preset(), RealismPreset::Standard);
    }

    #[test]
    fn garbage_and_out_of_range_values_are_safe() {
        assert_eq!(Settings::from_str("not json"), Settings::default());
        assert_eq!(Settings::from_str("[1,2]"), Settings::default());
        let s = Settings::from_str(
            r#"{"ui_scale": 9, "detail": -4, "camera": {"cockpit_fov": 500}, "weather": {"max_warp": 0}, "multiplayer": {"default_port": 99999}, "palette": "plaid"}"#,
        );
        assert_eq!(s.ui_scale, UI_SCALE_MAX);
        assert_eq!(s.detail, 0);
        assert_eq!(s.camera.cockpit_fov, 110.0);
        assert_eq!(s.wx.max_warp, 1);
        assert_eq!(s.mp.default_port, MpPrefs::default().default_port);
        assert_eq!(s.palette, Palette::Standard);
    }

    #[test]
    fn presets_are_distinct_and_recognised() {
        for p in [RealismPreset::Casual, RealismPreset::Standard, RealismPreset::Hard] {
            assert_eq!(Realism::preset(p).current_preset(), p);
        }
        assert_ne!(Realism::preset(RealismPreset::Casual), Realism::preset(RealismPreset::Hard));
        // Random failures are a separate choice: they do not make a preset "Custom".
        let r = Realism { random_failures: true, ..Realism::preset(RealismPreset::Hard) };
        assert_eq!(r.current_preset(), RealismPreset::Hard);
    }

    #[test]
    fn units_format() {
        assert_eq!(PressureUnit::Hpa.fmt(1013.2), "1013 hPa");
        assert_eq!(PressureUnit::InHg.fmt(1013.25), "29.92 inHg");
        assert_eq!(PressureUnit::InHg.short(1013.25), "A2992");
        assert_eq!(TempUnit::Fahrenheit.fmt(20.0), "68 °F");
    }

    #[test]
    fn config_dir_override() {
        // (Only this test sets the variable; nothing else in the suite reads the path.)
        let dir = std::env::temp_dir().join(format!("skyward-settings-test-{}", std::process::id()));
        // SAFETY: single-threaded use of this variable within the test binary.
        unsafe { std::env::set_var("SKYWARD_CONFIG_DIR", &dir) };
        assert_eq!(path(), dir.join("settings.json"));
        let s = Settings { ui_scale: 1.5, palette: Palette::Protanopia, ..Settings::default() };
        s.save();
        let back = Settings::load();
        unsafe { std::env::remove_var("SKYWARD_CONFIG_DIR") };
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(back, s);
    }
}
