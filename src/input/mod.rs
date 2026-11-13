//! Input devices: keyboard bindings, game controllers (gamepads, joysticks /
//! HOTAS, rudder pedals, throttle quadrants) and the mouse, mapped onto the
//! simulator's actions.
//!
//! * `backend` turns gilrs (Linux evdev, Windows WGI, macOS IOKit, the web
//!   Gamepad API) — plus, on Linux, raw evdev for button-less axis devices
//!   such as rudder pedals, which gilrs skips — into [`backend::Event`]s.
//! * Each device has a [`Profile`] (axis and button bindings) saved in the
//!   settings file under its USB id, made from [`defaults`] on first sight.
//! * [`InputSystem::update`] applies the profiles: analog axes go through
//!   calibration → invert → deadzone → expo → sensitivity ([`AxisCfg::apply`])
//!   into [`Analog`]; buttons (and axes / hats used as buttons) become
//!   [`Key`] presses exactly like the keyboard's.
//! * Rebinding: set [`InputSystem::listen`]; the next input moved is
//!   returned as a [`Captured`] binding.

pub mod backend;
pub mod defaults;
pub mod key_pitch;
pub mod keys;
#[cfg(test)]
mod tests;

use crate::sim::Key;
use serde_json::{Value, json};
use std::collections::{HashMap, HashSet};
use winit::keyboard::KeyCode;

// ---------------------------------------------------------------- actions

/// Continuous (analog) actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AxisAction {
    Pitch,
    Roll,
    Yaw,
    Throttle,
    Throttle1,
    Throttle2,
    Flaps,
    Spoilers,
    Brakes,
    BrakeLeft,
    BrakeRight,
    Trim,
    LookX,
    LookY,
}

impl AxisAction {
    pub const COUNT: usize = 14;
    pub const ALL: [AxisAction; AxisAction::COUNT] = [
        AxisAction::Pitch,
        AxisAction::Roll,
        AxisAction::Yaw,
        AxisAction::Throttle,
        AxisAction::Throttle1,
        AxisAction::Throttle2,
        AxisAction::Flaps,
        AxisAction::Spoilers,
        AxisAction::Brakes,
        AxisAction::BrakeLeft,
        AxisAction::BrakeRight,
        AxisAction::Trim,
        AxisAction::LookX,
        AxisAction::LookY,
    ];
    pub fn idx(self) -> usize {
        self as usize
    }
    pub fn id(self) -> &'static str {
        match self {
            AxisAction::Pitch => "pitch",
            AxisAction::Roll => "roll",
            AxisAction::Yaw => "yaw",
            AxisAction::Throttle => "throttle",
            AxisAction::Throttle1 => "throttle1",
            AxisAction::Throttle2 => "throttle2",
            AxisAction::Flaps => "flaps",
            AxisAction::Spoilers => "spoilers",
            AxisAction::Brakes => "brakes",
            AxisAction::BrakeLeft => "brake_left",
            AxisAction::BrakeRight => "brake_right",
            AxisAction::Trim => "trim",
            AxisAction::LookX => "look_x",
            AxisAction::LookY => "look_y",
        }
    }
    pub fn from_id(s: &str) -> Option<AxisAction> {
        AxisAction::ALL.into_iter().find(|a| a.id() == s)
    }
    pub fn label(self) -> &'static str {
        match self {
            AxisAction::Pitch => "Pitch",
            AxisAction::Roll => "Roll",
            AxisAction::Yaw => "Rudder",
            AxisAction::Throttle => "Throttle (all)",
            AxisAction::Throttle1 => "Throttle 1",
            AxisAction::Throttle2 => "Throttle 2",
            AxisAction::Flaps => "Flaps lever",
            AxisAction::Spoilers => "Speed brake",
            AxisAction::Brakes => "Brakes",
            AxisAction::BrakeLeft => "Toe brake left",
            AxisAction::BrakeRight => "Toe brake right",
            AxisAction::Trim => "Elevator trim",
            AxisAction::LookX => "Look left / right",
            AxisAction::LookY => "Look up / down",
        }
    }
    /// 0..1 levers (throttles, flaps, brakes) rather than centred −1..1 axes.
    pub fn unipolar(self) -> bool {
        matches!(
            self,
            AxisAction::Throttle
                | AxisAction::Throttle1
                | AxisAction::Throttle2
                | AxisAction::Flaps
                | AxisAction::Spoilers
                | AxisAction::Brakes
                | AxisAction::BrakeLeft
                | AxisAction::BrakeRight
        )
    }
    /// Self-centring stick-type axes (applied continuously) vs levers that stay
    /// where they are put (applied when they move, so keys can still adjust).
    pub fn centred(self) -> bool {
        matches!(self, AxisAction::Pitch | AxisAction::Roll | AxisAction::Yaw | AxisAction::LookX | AxisAction::LookY)
    }
}

/// Button actions: every simulator / interface command, in display order,
/// with a stable id (settings file), a label and a group.
pub const BUTTONS: &[(Key, &str, &str, &str)] = &[
    (Key::PitchDown, "pitch_down", "Pitch down", "Flight controls"),
    (Key::PitchUp, "pitch_up", "Pitch up", "Flight controls"),
    (Key::RollLeft, "roll_left", "Roll left", "Flight controls"),
    (Key::RollRight, "roll_right", "Roll right", "Flight controls"),
    (Key::YawLeft, "yaw_left", "Rudder left", "Flight controls"),
    (Key::YawRight, "yaw_right", "Rudder right", "Flight controls"),
    (Key::TrimDown, "trim_down", "Trim nose down", "Flight controls"),
    (Key::TrimUp, "trim_up", "Trim nose up", "Flight controls"),
    (Key::Brakes, "brakes", "Brakes (hold)", "Flight controls"),
    (Key::ThrottleUp, "throttle_up", "Throttle up", "Engines & levers"),
    (Key::ThrottleDown, "throttle_down", "Throttle down", "Engines & levers"),
    (Key::ThrottleLeftUp, "throttle_left_up", "Left engine(s) throttle up", "Engines & levers"),
    (Key::ThrottleLeftDown, "throttle_left_down", "Left engine(s) throttle down", "Engines & levers"),
    (Key::ThrottleRightUp, "throttle_right_up", "Right engine(s) throttle up", "Engines & levers"),
    (Key::ThrottleRightDown, "throttle_right_down", "Right engine(s) throttle down", "Engines & levers"),
    (Key::ThrottleSync, "throttle_sync", "Throttles together (end split)", "Engines & levers"),
    (Key::Reverse, "reverse", "Thrust reverse", "Engines & levers"),
    (Key::FlapsDown, "flaps_down", "Flaps down", "Engines & levers"),
    (Key::FlapsUp, "flaps_up", "Flaps up", "Engines & levers"),
    (Key::SpoilerOut, "spoiler_out", "Speed brake out", "Engines & levers"),
    (Key::SpoilerIn, "spoiler_in", "Speed brake in", "Engines & levers"),
    (Key::Gear, "gear", "Landing gear", "Engines & levers"),
    (Key::LandingLights, "landing_lights", "Landing lights", "Engines & levers"),
    (Key::EngineFail, "engine_fail", "Engine failure practice", "Engines & levers"),
    (Key::MasterAck, "master_ack", "Master warning / caution reset", "Engines & levers"),
    (Key::Assist, "assist", "Rotorcraft: stability assist / flight mode", "Flight controls"),
    (Key::ApToggle, "ap", "Autopilot on / off", "Autopilot"),
    (Key::ApHdgLeft, "ap_hdg_left", "Heading −5°", "Autopilot"),
    (Key::ApHdgRight, "ap_hdg_right", "Heading +5°", "Autopilot"),
    (Key::ApAltUp, "ap_alt_up", "Altitude +100 ft", "Autopilot"),
    (Key::ApAltDown, "ap_alt_down", "Altitude −100 ft", "Autopilot"),
    (Key::ApSpdUp, "ap_spd_up", "Speed +5 kt", "Autopilot"),
    (Key::ApSpdDown, "ap_spd_down", "Speed −5 kt", "Autopilot"),
    (Key::ApLoc, "ap_loc", "LOC / VOR track (NAV 1)", "Autopilot"),
    (Key::ApAppr, "ap_appr", "Approach (LOC + G/S)", "Autopilot"),
    (Key::ApLnav, "ap_lnav", "LNAV (flight plan)", "Autopilot"),
    (Key::ApVnav, "ap_vnav", "VNAV (descent path)", "Autopilot"),
    (Key::AutoFlight, "auto_flight", "Auto flight (bot)", "Autopilot"),
    (Key::Nav1Swap, "nav1_swap", "NAV 1 active ↔ standby", "Radios"),
    (Key::ObsDec, "obs_dec", "NAV 1 course −1°", "Radios"),
    (Key::ObsInc, "obs_inc", "NAV 1 course +1°", "Radios"),
    (Key::Nav1Ident, "nav1_ident", "NAV 1 ident audio", "Radios"),
    (Key::RadioPanel, "radio_panel", "Radios & flight plan", "Radios"),
    (Key::NdMode, "nd_mode", "ND mode (ROSE / ARC / PLAN)", "Radios"),
    (Key::CommsMenu, "comms_menu", "ATC comms menu", "Radios"),
    (Key::CommsNext, "comms_next", "Comms menu: next", "Radios"),
    (Key::CommsPrev, "comms_prev", "Comms menu: previous", "Radios"),
    (Key::CommsSelect, "comms_select", "Comms menu: select", "Radios"),
    (Key::GunTrigger, "gun_trigger", "Guns (hold)", "Weapons"),
    (Key::WeaponRelease, "weapon_release", "Weapon release / Spitfire cannon (hold)", "Weapons"),
    (Key::WeaponSelect, "weapon_select", "Master mode / gun selector", "Weapons"),
    (Key::TargetLock, "target_lock", "Radar lock / seeker uncage", "Weapons"),
    (Key::Countermeasures, "countermeasures", "Flares + chaff", "Weapons"),
    (Key::SightMode, "sight_mode", "Gun harmonisation range", "Weapons"),
    (Key::Eject, "eject", "Eject / bail out (press twice)", "Weapons"),
    (Key::Camera, "camera", "Next camera", "View"),
    (Key::LookLeft, "look_left", "Glance left (hold)", "View"),
    (Key::LookRight, "look_right", "Glance right (hold)", "View"),
    (Key::LookUp, "look_up", "Glance up (hold)", "View"),
    (Key::LookDown, "look_down", "Glance down (hold)", "View"),
    (Key::LookCenter, "look_center", "Centre view", "View"),
    (Key::MouseYoke, "mouse_yoke", "Mouse yoke on / off", "View"),
    (Key::Pause, "pause", "Pause", "Simulation"),
    (Key::TimeWarp, "time_warp", "Time warp", "Simulation"),
    (Key::Reset, "reset", "Restart", "Simulation"),
    (Key::Screenshot, "screenshot", "Screenshot", "Simulation"),
    (Key::Menu, "menu", "Pause menu", "Interface"),
    (Key::Help, "help", "Help", "Interface"),
    (Key::Map, "map", "Moving map size", "Interface"),
    (Key::MapZoomOut, "map_out", "Map range out", "Interface"),
    (Key::MapZoomIn, "map_in", "Map range in", "Interface"),
    (Key::Instruments, "instruments", "Instruments", "Interface"),
    (Key::WeatherPanel, "weather_panel", "Weather panel", "Interface"),
    (Key::Failures, "failures", "Systems & failures", "Interface"),
    (Key::Mute, "mute", "Mute", "Interface"),
    (Key::Chat, "mp_chat", "Chat", "Multiplayer"),
    (Key::PlayerList, "mp_players", "Players & spectate", "Multiplayer"),
    (Key::NameTags, "mp_tags", "Name tags on / off", "Multiplayer"),
    (Key::QuickChat, "mp_quick_chat", "Quick-chat wheel", "Multiplayer"),
];

/// Actions that may share a key because they never apply at the same time:
/// the wheel brakes (on the ground) and the gun trigger (in the air, armed
/// aircraft only), or the split-throttle keys (multi-engine) and the
/// weapon controls (single-engine fighters).
pub const SHARED: &[(Key, Key)] = &[
    (Key::Brakes, Key::GunTrigger),
    (Key::ThrottleSync, Key::WeaponSelect),
    (Key::ThrottleLeftDown, Key::TargetLock),
    (Key::ThrottleLeftUp, Key::Countermeasures),
    (Key::ThrottleRightDown, Key::SightMode),
    (Key::ThrottleRightUp, Key::Eject),
];

fn shared_ok(ks: &[Key]) -> bool {
    ks.len() == 2 && SHARED.iter().any(|&(a, b)| (ks[0] == a && ks[1] == b) || (ks[0] == b && ks[1] == a))
}

pub fn key_id(k: Key) -> &'static str {
    BUTTONS.iter().find(|b| b.0 == k).map(|b| b.1).unwrap_or("?")
}
pub fn key_from_id(s: &str) -> Option<Key> {
    BUTTONS.iter().find(|b| b.1 == s).map(|b| b.0)
}
pub fn key_label(k: Key) -> &'static str {
    BUTTONS.iter().find(|b| b.0 == k).map(|b| b.2).unwrap_or("?")
}

// ---------------------------------------------------------------- bindings

/// Which part of a device input a binding uses: all of it, or one direction
/// (a hat or stick direction as a button, a trigger as half an axis).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Half {
    Full,
    Pos,
    Neg,
}

impl Half {
    fn id(self) -> &'static str {
        match self {
            Half::Full => "full",
            Half::Pos => "+",
            Half::Neg => "-",
        }
    }
    fn from_id(s: &str) -> Half {
        match s {
            "+" => Half::Pos,
            "-" => Half::Neg,
            _ => Half::Full,
        }
    }
    /// Value after taking this part (Pos / Neg give 0..1).
    pub fn take(self, v: f32) -> f32 {
        match self {
            Half::Full => v,
            Half::Pos => v.max(0.0),
            Half::Neg => (-v).max(0.0),
        }
    }
}

/// Response shaping for one analog binding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisCfg {
    pub invert: bool,
    /// Fraction of travel (0..0.5) around centre (levers: at the bottom) that reads zero.
    pub deadzone: f32,
    /// 0 = linear .. 1 = cubic: finer control near centre.
    pub expo: f32,
    /// Output scale (0.2..2), clamped to the action's range.
    pub sens: f32,
    /// Raw [min, centre, max] measured by calibration (None: −1, 0, 1).
    pub cal: Option<[f32; 3]>,
}

impl Default for AxisCfg {
    fn default() -> Self {
        AxisCfg { invert: false, deadzone: 0.04, expo: 0.0, sens: 1.0, cal: None }
    }
}

impl AxisCfg {
    /// Raw device value (−1..1; 0..1 for analog buttons) → action value
    /// (−1..1, or 0..1 for unipolar actions).
    pub fn apply(&self, raw: f32, half: Half, unipolar: bool) -> f32 {
        let mut v = raw;
        if let Some([lo, mid, hi]) = self.cal {
            v = if v >= mid { (v - mid) / (hi - mid).max(1e-3) } else { (v - mid) / (mid - lo).max(1e-3) };
            v = v.clamp(-1.0, 1.0);
        }
        v = half.take(v);
        if self.invert {
            v = -v;
        }
        let dz = self.deadzone.clamp(0.0, 0.5);
        let lever = unipolar && half == Half::Full;
        if lever {
            // A lever's whole travel is 0..1; the deadzone sits at idle.
            v = (v + 1.0) * 0.5;
            v = ((v - dz) / (1.0 - dz)).max(0.0);
        } else {
            let a = v.abs();
            v = v.signum() * ((a - dz) / (1.0 - dz)).max(0.0);
        }
        let e = self.expo.clamp(0.0, 1.0);
        let a = v.abs();
        v = v.signum() * ((1.0 - e) * a + e * a * a * a);
        v *= self.sens.clamp(0.1, 3.0);
        if unipolar { v.clamp(0.0, 1.0) } else { v.clamp(-1.0, 1.0) }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AxisBind {
    pub action: AxisAction,
    pub code: u32,
    pub half: Half,
    pub cfg: AxisCfg,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ButtonBind {
    pub action: Key,
    pub code: u32,
    pub half: Half,
}

/// Bindings for one device model (keyed by its USB / SDL id in [`Config`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    pub name: String,
    pub enabled: bool,
    /// Which default layout it started from ("Xbox-style gamepad", ...).
    pub preset: String,
    pub axes: Vec<AxisBind>,
    pub buttons: Vec<ButtonBind>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MouseCfg {
    /// Mouse yoke available (toggled in flight with its key, default Y).
    pub yoke: bool,
    pub yoke_sens: f32,
    /// Head-look speed (right-drag in the cockpit) and vertical invert.
    pub look_sens: f32,
    pub invert_look: bool,
    /// The view springs back to centre when the drag ends.
    pub look_return: bool,
}

impl Default for MouseCfg {
    fn default() -> Self {
        MouseCfg { yoke: false, yoke_sens: 1.0, look_sens: 1.0, invert_look: false, look_return: false }
    }
}

/// Everything persisted under "controls" in the settings file.
#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    /// Keyboard: each action's keys.
    pub keys: Vec<(Key, Vec<KeyCode>)>,
    /// Device profiles by device id.
    pub devices: Vec<(String, Profile)>,
    pub mouse: MouseCfg,
}

impl Default for Config {
    fn default() -> Self {
        Config { keys: defaults::keyboard(), devices: Vec::new(), mouse: MouseCfg::default() }
    }
}

impl Config {
    /// Actions bound to a key (several when bindings conflict).
    pub fn actions_for_key(&self, code: KeyCode) -> Vec<Key> {
        self.keys.iter().filter(|(_, cs)| cs.contains(&code)).map(|(k, _)| *k).collect()
    }
    pub fn keys_of(&self, k: Key) -> &[KeyCode] {
        self.keys.iter().find(|e| e.0 == k).map(|e| e.1.as_slice()).unwrap_or(&[])
    }
    pub fn keys_of_mut(&mut self, k: Key) -> &mut Vec<KeyCode> {
        if let Some(i) = self.keys.iter().position(|e| e.0 == k) {
            return &mut self.keys[i].1;
        }
        self.keys.push((k, Vec::new()));
        &mut self.keys.last_mut().unwrap().1
    }
    pub fn profile(&self, dev: &str) -> Option<&Profile> {
        self.devices.iter().find(|d| d.0 == dev).map(|d| &d.1)
    }
    pub fn profile_mut(&mut self, dev: &str) -> Option<&mut Profile> {
        self.devices.iter_mut().find(|d| d.0 == dev).map(|d| &mut d.1)
    }

    /// Bind a key to an action (a key on several actions is shown as a conflict).
    pub fn bind_key(&mut self, k: Key, code: KeyCode) {
        let list = self.keys_of_mut(k);
        if !list.contains(&code) {
            list.push(code);
        }
    }

    /// Bind a device input (as a button) to an action.
    pub fn bind_button(&mut self, dev: &str, k: Key, code: u32, half: Half) {
        if let Some(p) = self.profile_mut(dev)
            && !p.buttons.iter().any(|b| b.action == k && b.code == code && b.half == half)
        {
            p.buttons.push(ButtonBind { action: k, code, half });
        }
    }

    /// Bind a device axis to an action, replacing that device's previous
    /// binding for the action (keeping its response settings).
    pub fn bind_axis(&mut self, dev: &str, a: AxisAction, code: u32, half: Half) {
        if let Some(p) = self.profile_mut(dev) {
            let cfg = p.axes.iter().find(|b| b.action == a).map(|b| AxisCfg { cal: None, invert: false, ..b.cfg }).unwrap_or_default();
            p.axes.retain(|b| b.action != a);
            p.axes.push(AxisBind { action: a, code, half, cfg });
        }
    }

    /// Conflicts: (description, actions) for each input bound to more than one action.
    pub fn conflicts(&self) -> Vec<(String, Vec<Key>)> {
        let mut out = Vec::new();
        let mut by_key: HashMap<KeyCode, Vec<Key>> = HashMap::new();
        for (k, cs) in &self.keys {
            for c in cs {
                by_key.entry(*c).or_default().push(*k);
            }
        }
        for (c, ks) in by_key {
            if ks.len() > 1 && !shared_ok(&ks) {
                out.push((keys::label(c), ks));
            }
        }
        for (_, p) in &self.devices {
            let mut by: HashMap<(u32, Half), Vec<Key>> = HashMap::new();
            for b in &p.buttons {
                by.entry((b.code, b.half)).or_default().push(b.action);
            }
            for ((code, half), ks) in by {
                if ks.len() > 1 {
                    out.push((format!("{} · {}", p.name, code_label(None, code, half)), ks));
                }
            }
        }
        out.sort_by(|a, b| a.0.cmp(&b.0));
        out
    }

    pub fn to_json(&self) -> Value {
        let keys: serde_json::Map<String, Value> =
            self.keys.iter().map(|(k, cs)| (key_id(*k).to_string(), Value::from(cs.iter().filter_map(|c| keys::name(*c)).collect::<Vec<_>>()))).collect();
        let devices: serde_json::Map<String, Value> = self
            .devices
            .iter()
            .map(|(id, p)| {
                let axes: Vec<Value> = p
                    .axes
                    .iter()
                    .map(|b| {
                        json!({ "action": b.action.id(), "code": b.code, "half": b.half.id(), "invert": b.cfg.invert,
                            "deadzone": b.cfg.deadzone, "expo": b.cfg.expo, "sens": b.cfg.sens, "cal": b.cfg.cal })
                    })
                    .collect();
                let buttons: Vec<Value> = p.buttons.iter().map(|b| json!({ "action": key_id(b.action), "code": b.code, "half": b.half.id() })).collect();
                (id.clone(), json!({ "name": p.name, "enabled": p.enabled, "preset": p.preset, "axes": axes, "buttons": buttons }))
            })
            .collect();
        let m = &self.mouse;
        json!({
            "keys": keys,
            "devices": devices,
            "mouse": { "yoke": m.yoke, "yoke_sens": m.yoke_sens, "look_sens": m.look_sens, "invert_look": m.invert_look, "look_return": m.look_return },
        })
    }

    pub fn from_json(v: &Value) -> Config {
        let mut c = Config::default();
        if let Some(ks) = v.get("keys").and_then(|x| x.as_object()) {
            for (id, list) in ks {
                let Some(k) = key_from_id(id) else { continue };
                let codes: Vec<KeyCode> = list.as_array().map(|a| a.iter().filter_map(|s| s.as_str().and_then(keys::from_name)).collect()).unwrap_or_default();
                *c.keys_of_mut(k) = codes;
            }
        }
        let f = |o: &Value, k: &str, d: f32| o.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(d);
        if let Some(ds) = v.get("devices").and_then(|x| x.as_object()) {
            for (id, p) in ds {
                let axes = p
                    .get("axes")
                    .and_then(|x| x.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|b| {
                                let action = AxisAction::from_id(b.get("action")?.as_str()?)?;
                                let cal = b.get("cal").and_then(|x| x.as_array()).and_then(|a| {
                                    let v: Vec<f32> = a.iter().filter_map(|x| x.as_f64().map(|x| x as f32)).collect();
                                    (v.len() == 3).then(|| [v[0], v[1], v[2]])
                                });
                                let d = AxisCfg::default();
                                Some(AxisBind {
                                    action,
                                    code: b.get("code")?.as_u64()? as u32,
                                    half: Half::from_id(b.get("half").and_then(|x| x.as_str()).unwrap_or("full")),
                                    cfg: AxisCfg {
                                        invert: b.get("invert").and_then(|x| x.as_bool()).unwrap_or(false),
                                        deadzone: f(b, "deadzone", d.deadzone),
                                        expo: f(b, "expo", d.expo),
                                        sens: f(b, "sens", d.sens),
                                        cal,
                                    },
                                })
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                let buttons = p
                    .get("buttons")
                    .and_then(|x| x.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|b| {
                                Some(ButtonBind {
                                    action: key_from_id(b.get("action")?.as_str()?)?,
                                    code: b.get("code")?.as_u64()? as u32,
                                    half: Half::from_id(b.get("half").and_then(|x| x.as_str()).unwrap_or("full")),
                                })
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                let s = |k: &str| p.get(k).and_then(|x| x.as_str()).unwrap_or("").to_string();
                c.devices.push((id.clone(), Profile { name: s("name"), enabled: p.get("enabled").and_then(|x| x.as_bool()).unwrap_or(true), preset: s("preset"), axes, buttons }));
            }
        }
        if let Some(m) = v.get("mouse") {
            let d = MouseCfg::default();
            let b = |k: &str, def: bool| m.get(k).and_then(|x| x.as_bool()).unwrap_or(def);
            c.mouse = MouseCfg {
                yoke: b("yoke", d.yoke),
                yoke_sens: f(m, "yoke_sens", d.yoke_sens).clamp(0.2, 3.0),
                look_sens: f(m, "look_sens", d.look_sens).clamp(0.2, 3.0),
                invert_look: b("invert_look", d.invert_look),
                look_return: b("look_return", d.look_return),
            };
        }
        c
    }
}

// ---------------------------------------------------------------- devices

/// Standard controls a device may report (for default layouts and labels).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Std {
    X,
    Y,
    Z,
    Rx,
    Ry,
    Rz,
    Throttle,
    Rudder,
    Wheel,
    Gas,
    Brake,
    Slider,
    HatX,
    HatY,
    South,
    East,
    North,
    West,
    LB,
    RB,
    LT2,
    RT2,
    Select,
    Start,
    Mode,
    LThumb,
    RThumb,
    DUp,
    DDown,
    DLeft,
    DRight,
    /// Generic joystick button n (0 = trigger).
    Joy(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DevKind {
    Gamepad,
    Joystick,
    Throttle,
    Pedals,
}

impl DevKind {
    pub fn name(self) -> &'static str {
        match self {
            DevKind::Gamepad => "Gamepad",
            DevKind::Joystick => "Joystick",
            DevKind::Throttle => "Throttle",
            DevKind::Pedals => "Rudder pedals",
        }
    }
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    /// Backend-unique id for this connection.
    pub id: u64,
    /// Profile key: the device model (bus / vendor / product / version).
    pub key: String,
    pub name: String,
    pub kind: DevKind,
    /// The backend knows it as a standard gamepad (SDL mapping).
    pub mapped: bool,
    pub backend: &'static str,
    pub std: Vec<(Std, u32)>,
}

impl DeviceInfo {
    pub fn std(&self, s: Std) -> Option<u32> {
        self.std.iter().find(|e| e.0 == s).map(|e| e.1)
    }
}

/// A device input's (code, value, is a button).
pub type RawValue = (u32, f32, bool);

/// A connected device and the last value of each input it has reported.
#[derive(Clone, Debug)]
pub struct Device {
    pub info: DeviceInfo,
    /// (code, value, is a button), in first-seen order.
    pub values: Vec<(u32, f32, bool)>,
}

impl Device {
    pub fn value(&self, code: u32) -> Option<f32> {
        self.values.iter().find(|v| v.0 == code).map(|v| v.1)
    }
}

/// Human label for a device input ("Axis X", "Hat ←", "Button 3"; a gamepad's
/// face buttons by their Xbox / PlayStation names).
pub fn code_label(info: Option<&DeviceInfo>, code: u32, half: Half) -> String {
    let dir = |neg: &str, pos: &str| match half {
        Half::Full => String::new(),
        Half::Pos => format!(" {pos}"),
        Half::Neg => format!(" {neg}"),
    };
    let std = info.and_then(|i| i.std.iter().find(|e| e.1 == code).map(|e| e.0)).or_else(|| backend::linux_std(code));
    let pad = info.is_some_and(|i| i.kind == DevKind::Gamepad);
    match std {
        Some(Std::X) if pad => format!("Left stick X{}", dir("←", "→")),
        Some(Std::Y) if pad => format!("Left stick Y{}", dir("↓", "↑")),
        Some(Std::Rx) if pad => format!("Right stick X{}", dir("←", "→")),
        Some(Std::Ry) if pad => format!("Right stick Y{}", dir("↓", "↑")),
        Some(Std::X) => format!("Axis X{}", dir("←", "→")),
        Some(Std::Y) => format!("Axis Y{}", dir("↓", "↑")),
        Some(Std::Z) => format!("Axis Z{}", dir("−", "+")),
        Some(Std::Rx) => format!("Axis Rx{}", dir("−", "+")),
        Some(Std::Ry) => format!("Axis Ry{}", dir("−", "+")),
        Some(Std::Rz) => format!("Axis Rz{}", dir("−", "+")),
        Some(Std::Throttle) => format!("Throttle{}", dir("−", "+")),
        Some(Std::Rudder) => format!("Rudder{}", dir("−", "+")),
        Some(Std::Wheel) => format!("Wheel{}", dir("−", "+")),
        Some(Std::Gas) => format!("Gas{}", dir("−", "+")),
        Some(Std::Brake) => format!("Brake{}", dir("−", "+")),
        Some(Std::Slider) => format!("Slider{}", dir("−", "+")),
        Some(Std::HatX) => format!("Hat{}", if half == Half::Full { " X".into() } else { dir("←", "→") }),
        Some(Std::HatY) => format!("Hat{}", if half == Half::Full { " Y".into() } else { dir("↓", "↑") }),
        Some(Std::South) => "A / Cross".into(),
        Some(Std::East) => "B / Circle".into(),
        Some(Std::North) => "Y / Triangle".into(),
        Some(Std::West) => "X / Square".into(),
        Some(Std::LB) => "LB / L1".into(),
        Some(Std::RB) => "RB / R1".into(),
        Some(Std::LT2) => "LT / L2".into(),
        Some(Std::RT2) => "RT / R2".into(),
        Some(Std::Select) => "View / Share".into(),
        Some(Std::Start) => "Menu / Options".into(),
        Some(Std::Mode) => "Guide / PS".into(),
        Some(Std::LThumb) => "Left stick click".into(),
        Some(Std::RThumb) => "Right stick click".into(),
        Some(Std::DUp) => "D-pad ↑".into(),
        Some(Std::DDown) => "D-pad ↓".into(),
        Some(Std::DLeft) => "D-pad ←".into(),
        Some(Std::DRight) => "D-pad →".into(),
        Some(Std::Joy(n)) => format!("Button {}", n as u32 + 1),
        None => format!("Input {}{}", code & 0xffff, dir("−", "+")),
    }
}

// ---------------------------------------------------------------- runtime

/// Analog action values this frame (None: nothing bound / connected).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Analog {
    pub v: [Option<f32>; AxisAction::COUNT],
}

impl Analog {
    pub fn get(&self, a: AxisAction) -> Option<f32> {
        self.v[a.idx()]
    }
    #[cfg(test)]
    pub fn set(&mut self, a: AxisAction, x: f32) {
        self.v[a.idx()] = Some(x);
    }
}

/// Range seen on one device input while calibrating.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calib {
    pub dev: u64,
    pub code: u32,
    pub lo: f32,
    pub hi: f32,
    pub now: f32,
}

impl Calib {
    /// Calibration result: [min, centre (where it rests now), max]; None if it hardly moved.
    pub fn result(&self) -> Option<[f32; 3]> {
        (self.hi - self.lo > 0.3).then(|| [self.lo, self.now.clamp(self.lo + 0.05, self.hi - 0.05), self.hi])
    }
}

/// What the settings screen is waiting for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Listen {
    Key(Key),
    Button(Key),
    Axis(AxisAction),
}

/// A binding captured while listening.
#[derive(Clone, Debug, PartialEq)]
pub enum Captured {
    Key(Key, KeyCode),
    Button { action: Key, dev: String, code: u32, half: Half },
    Axis { action: AxisAction, dev: String, code: u32, half: Half },
}

pub struct InputSystem {
    backend: Option<backend::Backends>,
    /// Events from virtual devices (tests, screenshots) waiting to be applied.
    queued: Vec<backend::Event>,
    pub devices: Vec<Device>,
    /// Device inputs currently down as buttons: (device id, code, half) → action.
    down: HashMap<(u64, u32, Half), Key>,
    /// Actions held on devices.
    pub held: HashSet<Key>,
    pressed: Vec<Key>,
    pub analog: Analog,
    pub listen: Option<Listen>,
    listen_age: f32,
    listen_from: HashMap<(u64, u32), f32>,
    pub captured: Option<Captured>,
    /// Axis calibration in progress (Settings › Controls).
    pub calib: Option<Calib>,
    /// Mouse yoke: on in flight (toggled by its key) and the cursor offset
    /// from the window centre (−1..1, +y = down) set by the window layer.
    pub mouse_yoke_on: bool,
    pub mouse_offset: Option<(f32, f32)>,
    /// Devices that came and went since last asked (for toasts).
    pub arrived: Vec<String>,
    pub left: Vec<String>,
    /// Input devices the backends reported that are not game controllers.
    pub ignored: Vec<String>,
    pub status: String,
}

impl Default for InputSystem {
    fn default() -> Self {
        InputSystem::new()
    }
}

impl InputSystem {
    /// No device backend yet (headless / tests): see [`InputSystem::start`].
    pub fn new() -> InputSystem {
        InputSystem {
            backend: None,
            queued: Vec::new(),
            devices: Vec::new(),
            down: HashMap::new(),
            held: HashSet::new(),
            pressed: Vec::new(),
            analog: Analog::default(),
            listen: None,
            listen_age: 0.0,
            listen_from: HashMap::new(),
            captured: None,
            calib: None,
            mouse_yoke_on: false,
            mouse_offset: None,
            arrived: Vec::new(),
            left: Vec::new(),
            ignored: Vec::new(),
            status: "Controllers: not started".into(),
        }
    }

    /// Open the device backends (hot-plug included).
    pub fn start(&mut self) {
        let b = backend::Backends::new();
        self.status = b.status();
        self.backend = Some(b);
    }

    /// Virtual devices (`SKYWARD_FAKE_INPUT=xbox,x52,pedals`): exercise the
    /// whole path and the settings screen without hardware.
    pub fn add_fakes_from_env(&mut self) {
        if let Ok(list) = std::env::var("SKYWARD_FAKE_INPUT") {
            for (i, name) in list.split(',').filter(|s| !s.is_empty()).enumerate() {
                if let Some((info, values)) = defaults::fake_device(name.trim(), 900 + i as u64) {
                    self.connect_virtual(info, &values);
                }
            }
        }
    }

    /// Plug in a virtual device with initial input values.
    pub fn connect_virtual(&mut self, info: DeviceInfo, values: &[(u32, f32, bool)]) {
        let id = info.id;
        self.queued.push(backend::Event::Connected(info));
        for &(code, value, button) in values {
            self.queued.push(backend::Event::Value { id, code, value, button });
        }
    }

    /// Feed a virtual device input (tests).
    #[cfg(test)]
    pub fn inject(&mut self, id: u64, code: u32, value: f32, button: bool) {
        self.queued.push(backend::Event::Value { id, code, value, button });
    }

    #[cfg(test)]
    pub fn disconnect_virtual(&mut self, id: u64) {
        self.queued.push(backend::Event::Disconnected(id));
    }

    /// Button presses since the last call.
    pub fn take_pressed(&mut self) -> Vec<Key> {
        std::mem::take(&mut self.pressed)
    }

    pub fn device(&self, key: &str) -> Option<&Device> {
        self.devices.iter().find(|d| d.info.key == key)
    }

    /// Give every connected device without a profile its default one.
    pub fn ensure_profiles(&self, cfg: &mut Config) {
        for d in &self.devices {
            if cfg.profile(&d.info.key).is_none() {
                cfg.devices.push((d.info.key.clone(), defaults::profile(&d.info)));
            }
        }
    }

    pub fn start_calibration(&mut self, dev: &str, code: u32) {
        if let Some(d) = self.device(dev) {
            let v = d.value(code).unwrap_or(0.0);
            self.calib = Some(Calib { dev: d.info.id, code, lo: v, hi: v, now: v });
        }
    }

    pub fn start_listening(&mut self, l: Listen) {
        self.listen = Some(l);
        self.listen_age = 0.0;
        self.captured = None;
        self.listen_from = self.devices.iter().flat_map(|d| d.values.iter().map(move |v| ((d.info.id, v.0), v.1))).collect();
    }

    /// Poll the backends and apply the bindings. Returns true when `cfg`
    /// changed (a new device got its default profile).
    pub fn update(&mut self, cfg: &mut Config, dt: f32) -> bool {
        let mut events = std::mem::take(&mut self.queued);
        if let Some(b) = self.backend.as_mut() {
            b.poll(&mut events);
        }
        let mut changed = false;
        for ev in events {
            match ev {
                backend::Event::Connected(info) => {
                    if self.devices.iter().any(|d| d.info.id == info.id) {
                        continue;
                    }
                    if !defaults::is_controller(&info.name) {
                        if !self.ignored.contains(&info.name) {
                            self.ignored.push(info.name);
                        }
                        continue;
                    }
                    if cfg.profile(&info.key).is_none() {
                        cfg.devices.push((info.key.clone(), defaults::profile(&info)));
                        changed = true;
                    }
                    self.arrived.push(info.name.clone());
                    self.devices.push(Device { info, values: Vec::new() });
                }
                backend::Event::Disconnected(id) => {
                    if let Some(i) = self.devices.iter().position(|d| d.info.id == id) {
                        let d = self.devices.remove(i);
                        self.left.push(d.info.name);
                    }
                    self.down.retain(|k, _| k.0 != id);
                }
                backend::Event::Value { id, code, value, button } => self.on_value(cfg, id, code, value, button),
            }
        }
        if self.listen.is_some() {
            self.listen_age += dt;
            if self.listen_age > 10.0 {
                self.listen = None;
            }
        }
        self.held = self.down.values().copied().collect();
        self.analog = self.compute_analog(cfg);
        changed
    }

    fn on_value(&mut self, cfg: &Config, id: u64, code: u32, value: f32, button: bool) {
        let Some(di) = self.devices.iter().position(|d| d.info.id == id) else { return };
        let dev = &mut self.devices[di];
        match dev.values.iter_mut().find(|v| v.0 == code) {
            Some(v) => v.1 = value,
            None => dev.values.push((code, value, button)),
        }
        let key = dev.info.key.clone();
        if let Some(c) = self.calib.as_mut().filter(|c| c.dev == id && c.code == code) {
            c.lo = c.lo.min(value);
            c.hi = c.hi.max(value);
            c.now = value;
        }
        // Rebinding: the first input clearly moved from where it was.
        if let Some(l) = self.listen {
            // Unseen inputs: buttons and hats rest at 0; an axis being bound starts where it is.
            let from = *self.listen_from.entry((id, code)).or_insert(if button || matches!(l, Listen::Button(_)) { 0.0 } else { value });
            let moved = value - from;
            match l {
                Listen::Button(action) if (button && value > 0.5) || (!button && moved.abs() > 0.6 && value.abs() > 0.5) => {
                    let half = if button {
                        Half::Full
                    } else if value > 0.0 {
                        Half::Pos
                    } else {
                        Half::Neg
                    };
                    self.captured = Some(Captured::Button { action, dev: key.clone(), code, half });
                    self.listen = None;
                    return;
                }
                Listen::Axis(action) if moved.abs() > 0.5 => {
                    let half = if button { Half::Pos } else { Half::Full };
                    self.captured = Some(Captured::Axis { action, dev: key.clone(), code, half });
                    self.listen = None;
                    return;
                }
                _ => {}
            }
            return;
        }
        let Some(p) = cfg.profile(&key).filter(|p| p.enabled) else { return };
        for b in p.buttons.iter().filter(|b| b.code == code) {
            let on = b.half.take(value) > 0.5;
            let k = (id, code, b.half);
            if on && !self.down.contains_key(&k) {
                self.down.insert(k, b.action);
                self.pressed.push(b.action);
            } else if !on {
                self.down.remove(&k);
            }
        }
    }

    fn compute_analog(&self, cfg: &Config) -> Analog {
        let mut out = Analog::default();
        for d in &self.devices {
            let Some(p) = cfg.profile(&d.info.key).filter(|p| p.enabled) else { continue };
            for b in &p.axes {
                if let Some(raw) = d.value(b.code) {
                    let v = b.cfg.apply(raw, b.half, b.action.unipolar());
                    let slot = &mut out.v[b.action.idx()];
                    *slot = Some(slot.unwrap_or(0.0) + v);
                }
            }
        }
        if let (true, true, Some((x, y))) = (cfg.mouse.yoke, self.mouse_yoke_on, self.mouse_offset) {
            // Cursor offset from the centre is the stick; a little dead band in the middle.
            let shape = |v: f32| {
                let a = ((v.abs() - 0.03) / 0.97).max(0.0) * cfg.mouse.yoke_sens;
                v.signum() * (0.35 * a + 0.65 * a * a).min(1.0)
            };
            out.v[AxisAction::Roll.idx()] = Some(out.v[AxisAction::Roll.idx()].unwrap_or(0.0) + shape(x));
            out.v[AxisAction::Pitch.idx()] = Some(out.v[AxisAction::Pitch.idx()].unwrap_or(0.0) + shape(y));
        }
        for a in AxisAction::ALL {
            if let Some(v) = &mut out.v[a.idx()] {
                *v = if a.unipolar() { v.clamp(0.0, 1.0) } else { v.clamp(-1.0, 1.0) };
            }
        }
        out
    }
}

/// `--controllers`: open the backends, report devices (with the layout they
/// get) and every action they produce, for `seconds`.
pub fn probe(seconds: f32) {
    let mut sys = InputSystem::new();
    sys.start();
    sys.add_fakes_from_env();
    println!("{}", sys.status);
    let mut cfg = crate::ui::settings::Settings::load().controls;
    let t0 = web_time::Instant::now();
    let mut last = Analog::default();
    while t0.elapsed().as_secs_f32() < seconds {
        sys.update(&mut cfg, 0.01);
        for n in std::mem::take(&mut sys.arrived) {
            let d = sys.devices.iter().find(|d| d.info.name == n).unwrap();
            let p = cfg.profile(&d.info.key).unwrap();
            println!("+ {} [{}] {} via {} — {} ({} axes, {} buttons)", n, d.info.key, d.info.kind.name(), d.info.backend, p.preset, p.axes.len(), p.buttons.len());
        }
        for n in std::mem::take(&mut sys.left) {
            println!("- {n}");
        }
        for k in sys.take_pressed() {
            println!("  button: {}", key_label(k));
        }
        for a in AxisAction::ALL {
            let (o, n) = (last.get(a), sys.analog.get(a));
            if o.zip(n).is_none_or(|(o, n)| (o - n).abs() > 0.05) && o != n {
                println!("  {:<16} {}", a.label(), n.map(|v| format!("{v:+.2}")).unwrap_or("—".into()));
                last.v[a.idx()] = n;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    for n in &sys.ignored {
        println!("  (ignored, not a game controller: {n})");
    }
    if sys.devices.is_empty() {
        println!("no controllers found");
    }
}
