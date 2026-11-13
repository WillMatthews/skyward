//! Persisted preferences: display, units, sound, the last flight,
//! recent and favourite airports. A small JSON file in the user's config
//! directory (`$XDG_CONFIG_HOME/skyward/settings.json`, else `cache/`).

use super::{AltUnit, SpeedUnit};
use serde_json::{json, Value};
use std::path::PathBuf;

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
    /// Road traffic: 0 off, 1 low, 2 medium, 3 high (traffic::Level).
    pub traffic: u8,
    pub speed: SpeedUnit,
    pub alt: AltUnit,
    pub map_style: MapStyle,
    pub north_up: bool,
    pub sound: crate::audio::Settings,
    /// Key / controller bindings, device profiles, mouse (input/).
    pub controls: crate::input::Config,
    pub conditions: Conditions,
    pub last: Option<LastFlight>,
    pub recents: Vec<String>,
    pub favourites: Vec<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            ui_scale: 1.0,
            hud: HudMode::Full,
            vsync: true,
            // The browser streams on the main thread: lighter by default.
            detail: if crate::platform::WEB { 0 } else { 2 },
            traffic: crate::traffic::Level::default_for_platform().as_u8(),
            speed: SpeedUnit::Knots,
            alt: AltUnit::Feet,
            map_style: MapStyle::Auto,
            north_up: true,
            sound: crate::audio::Settings::default(),
            controls: crate::input::Config::default(),
            conditions: Conditions::default(),
            last: None,
            recents: Vec::new(),
            favourites: Vec::new(),
        }
    }
}

pub fn path() -> PathBuf {
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

impl Settings {
    pub fn load() -> Settings {
        // Browser: localStorage.
        #[cfg(target_arch = "wasm32")]
        let Some(text) = crate::platform::web::storage_get("skyward.settings") else { return Settings::default() };
        #[cfg(not(target_arch = "wasm32"))]
        let Ok(text) = std::fs::read_to_string(path()) else { return Settings::default() };
        let Ok(v) = serde_json::from_str::<Value>(&text) else { return Settings::default() };
        Settings::from_json(&v)
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

    fn to_json(&self) -> Value {
        let s = &self.sound;
        let c = &self.conditions;
        json!({
            "ui_scale": self.ui_scale,
            "hud": self.hud.name(),
            "vsync": self.vsync,
            "detail": self.detail,
            "traffic": self.traffic,
            "speed": match self.speed { SpeedUnit::Knots => "kt", SpeedUnit::Kmh => "kmh", SpeedUnit::Mph => "mph" },
            "alt": match self.alt { AltUnit::Feet => "ft", AltUnit::Metres => "m" },
            "map_style": match self.map_style { MapStyle::Auto => "auto", MapStyle::Day => "day", MapStyle::Night => "night" },
            "north_up": self.north_up,
            "sound": { "muted": s.muted, "master": s.master, "engines": s.engines, "environment": s.environment, "systems": s.systems, "voice": s.voice, "ui": s.ui, "radio": s.radio },
            "controls": self.controls.to_json(),
            "conditions": { "live": c.live, "hours": c.hours, "day": c.day, "preset": c.preset, "metar": c.metar },
            "last": self.last.as_ref().map(|l| json!({ "aircraft": l.aircraft, "airport": l.airport, "start": l.start, "watch": l.watch })),
            "recents": self.recents,
            "favourites": self.favourites,
        })
    }

    fn from_json(v: &Value) -> Settings {
        let d = Settings::default();
        let f = |k: &str, def: f32| v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(def);
        let b = |k: &str, def: bool| v.get(k).and_then(|x| x.as_bool()).unwrap_or(def);
        let st = |k: &str| v.get(k).and_then(|x| x.as_str()).unwrap_or("").to_string();
        let list = |k: &str| v.get(k).and_then(|x| x.as_array()).map(|a| a.iter().filter_map(|s| s.as_str().map(String::from)).collect()).unwrap_or_default();
        let mut sound = d.sound;
        if let Some(s) = v.get("sound") {
            let g = |k: &str, def: f32| s.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(def);
            sound.muted = s.get("muted").and_then(|x| x.as_bool()).unwrap_or(sound.muted);
            sound.master = g("master", sound.master);
            sound.engines = g("engines", sound.engines);
            sound.environment = g("environment", sound.environment);
            sound.systems = g("systems", sound.systems);
            sound.voice = g("voice", sound.voice);
            sound.radio = g("radio", sound.radio);
            sound.ui = g("ui", sound.ui);
        }
        let mut conditions = d.conditions.clone();
        if let Some(c) = v.get("conditions") {
            conditions.live = c.get("live").and_then(|x| x.as_bool()).unwrap_or(true);
            conditions.hours = c.get("hours").and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(conditions.hours);
            conditions.day = c.get("day").and_then(|x| x.as_u64()).map(|x| x as u32).unwrap_or(conditions.day);
            conditions.preset = c.get("preset").and_then(|x| x.as_str()).unwrap_or("Scattered").to_string();
            conditions.metar = c.get("metar").and_then(|x| x.as_str()).unwrap_or("").to_string();
        }
        let last = v.get("last").filter(|l| !l.is_null()).map(|l| LastFlight {
            aircraft: l.get("aircraft").and_then(|x| x.as_str()).unwrap_or("").into(),
            airport: l.get("airport").and_then(|x| x.as_str()).unwrap_or("").into(),
            start: l.get("start").and_then(|x| x.as_str()).unwrap_or("").into(),
            watch: l.get("watch").and_then(|x| x.as_bool()).unwrap_or(false),
        });
        Settings {
            ui_scale: f("ui_scale", 1.0).clamp(0.7, 1.6),
            hud: match st("hud").as_str() {
                "Minimal" => HudMode::Minimal,
                "Off" => HudMode::Off,
                _ => HudMode::Full,
            },
            vsync: b("vsync", true),
            detail: (f("detail", if crate::platform::WEB { 0.0 } else { 2.0 }) as u8).min(3),
            traffic: (f("traffic", d.traffic as f32) as u8).min(3),
            speed: match st("speed").as_str() {
                "kmh" => SpeedUnit::Kmh,
                "mph" => SpeedUnit::Mph,
                _ => SpeedUnit::Knots,
            },
            alt: if st("alt") == "m" { AltUnit::Metres } else { AltUnit::Feet },
            map_style: match st("map_style").as_str() {
                "day" => MapStyle::Day,
                "night" => MapStyle::Night,
                _ => MapStyle::Auto,
            },
            north_up: b("north_up", true),
            sound,
            controls: v.get("controls").map(crate::input::Config::from_json).unwrap_or_default(),
            conditions,
            last,
            recents: list("recents"),
            favourites: list("favourites"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut s = Settings { hud: HudMode::Minimal, vsync: false, detail: 1, traffic: 3, speed: SpeedUnit::Kmh, alt: AltUnit::Metres, ..Settings::default() };
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
}
