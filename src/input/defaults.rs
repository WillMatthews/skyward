//! Default bindings: the keyboard layout, and per-device layouts chosen by
//! device type / name (Xbox and PlayStation pads, sticks such as the
//! T.16000M / X52 / Extreme 3D, throttles such as the TCA quadrant / X56 /
//! TWCS, rudder pedals). Unknown devices get the generic layout of their kind.

use super::{AxisAction, AxisBind, AxisCfg, ButtonBind, DevKind, DeviceInfo, Half, Profile, Std};
use crate::sim::Key;
use winit::keyboard::KeyCode as K;

/// The keyboard layout (the README's controls table).
pub fn keyboard() -> Vec<(Key, Vec<K>)> {
    vec![
        (Key::PitchDown, vec![K::ArrowUp, K::KeyW]),
        (Key::PitchUp, vec![K::ArrowDown, K::KeyS]),
        (Key::RollLeft, vec![K::ArrowLeft, K::KeyA]),
        (Key::RollRight, vec![K::ArrowRight, K::KeyD]),
        (Key::YawLeft, vec![K::KeyQ]),
        (Key::YawRight, vec![K::KeyE]),
        (Key::TrimDown, vec![K::BracketLeft]),
        (Key::TrimUp, vec![K::BracketRight]),
        (Key::Brakes, vec![K::KeyB, K::Space]),
        (Key::ThrottleUp, vec![K::ShiftLeft, K::ShiftRight, K::Equal, K::NumpadAdd, K::PageUp]),
        (Key::ThrottleDown, vec![K::ControlLeft, K::ControlRight, K::Minus, K::NumpadSubtract, K::PageDown]),
        (Key::Reverse, vec![K::KeyT]),
        (Key::FlapsDown, vec![K::KeyF]),
        (Key::FlapsUp, vec![K::KeyV]),
        (Key::SpoilerOut, vec![K::Slash]),
        (Key::SpoilerIn, vec![K::Backslash, K::Quote]),
        (Key::Gear, vec![K::KeyG]),
        (Key::LandingLights, vec![K::KeyL]),
        (Key::EngineFail, vec![K::KeyK]),
        (Key::MasterAck, vec![K::Backquote]),
        (Key::ApToggle, vec![K::KeyZ]),
        (Key::ApHdgLeft, vec![K::Insert]),
        (Key::ApHdgRight, vec![K::Delete]),
        (Key::ApAltUp, vec![K::Home]),
        (Key::ApAltDown, vec![K::End]),
        (Key::ApSpdUp, vec![K::Digit0]),
        (Key::ApSpdDown, vec![K::Digit9]),
        (Key::AutoFlight, vec![K::KeyJ]),
        (Key::Camera, vec![K::KeyC]),
        (Key::LookLeft, vec![K::Numpad4]),
        (Key::LookRight, vec![K::Numpad6]),
        (Key::LookUp, vec![K::Numpad8]),
        (Key::LookDown, vec![K::Numpad2]),
        (Key::LookCenter, vec![K::Numpad5]),
        (Key::MouseYoke, vec![K::KeyY]),
        (Key::Pause, vec![K::KeyP]),
        (Key::TimeWarp, vec![K::KeyX]),
        (Key::Reset, vec![K::KeyR]),
        (Key::Screenshot, vec![K::F12]),
        (Key::Menu, vec![K::Tab]),
        (Key::Help, vec![K::KeyH, K::F1]),
        (Key::Map, vec![K::KeyM]),
        (Key::MapZoomOut, vec![K::Comma]),
        (Key::MapZoomIn, vec![K::Period]),
        (Key::Instruments, vec![K::KeyU]),
        (Key::WeatherPanel, vec![K::KeyI]),
        (Key::Failures, vec![K::KeyO, K::F2]),
        (Key::Mute, vec![K::KeyN]),
        (Key::Assist, vec![K::Semicolon]),
    ]
}

/// Devices the backends report that are not game controllers: keyboards'
/// media / system-control interfaces, mice, laptop hotkeys, sensors.
pub fn is_controller(name: &str) -> bool {
    let n = name.to_lowercase();
    const NOT: [&str; 20] = [
        "system control",
        "consumer control",
        "keyboard",
        "mouse",
        "touchpad",
        "trackpad",
        "video bus",
        "power button",
        "sleep button",
        "headset",
        "headphone",
        "webcam",
        "camera",
        "accelerometer",
        "motion sensor",
        "hdmi",
        "pc speaker",
        "lid switch",
        "wmi hotkeys",
        "hotkeys",
    ];
    !NOT.iter().any(|w| n.contains(w))
}

/// Device type from its name and whether the backend maps it as a gamepad.
pub fn classify(name: &str, mapped: bool) -> DevKind {
    let n = name.to_lowercase();
    let has = |ws: &[&str]| ws.iter().any(|w| n.contains(w));
    if has(&["pedal", "rudder", "crosswind", "t-rudder", "tfrp"]) {
        DevKind::Pedals
    } else if has(&["throttle", "twcs", "q-eng", "quadrant", "tqs"]) {
        DevKind::Throttle
    } else if has(&[
        "x52",
        "x-52",
        "x56",
        "x-56",
        "t.16000",
        "t16000",
        "hotas",
        "extreme 3d",
        "flight",
        "stick",
        "joystick",
        "warthog",
        "t.flight",
        "vkb",
        "virpil",
        "winwing",
        "sidewinder",
    ]) {
        DevKind::Joystick
    } else if mapped
        || has(&["xbox", "x-box", "playstation", "dualsense", "dualshock", "wireless controller", "gamepad", "game pad", "8bitdo", "pro controller", "joy-con", "stadia"])
    {
        DevKind::Gamepad
    } else {
        DevKind::Joystick
    }
}

fn axis(info: &DeviceInfo, s: Std, action: AxisAction, half: Half, invert: bool, deadzone: f32, expo: f32) -> Option<AxisBind> {
    Some(AxisBind { action, code: info.std(s)?, half, cfg: AxisCfg { invert, deadzone, expo, ..AxisCfg::default() } })
}

fn btn(info: &DeviceInfo, s: Std, action: Key) -> Option<ButtonBind> {
    Some(ButtonBind { action, code: info.std(s)?, half: Half::Full })
}

/// A hat direction as a button (falls back to the d-pad buttons when the
/// device reports those instead).
fn hat(info: &DeviceInfo, out: &mut Vec<ButtonBind>, dpad: [Key; 4]) {
    let dirs = [(Std::DUp, Std::HatY, Half::Pos), (Std::DDown, Std::HatY, Half::Neg), (Std::DLeft, Std::HatX, Half::Neg), (Std::DRight, Std::HatX, Half::Pos)];
    for (i, (b, a, half)) in dirs.into_iter().enumerate() {
        if let Some(code) = info.std(b) {
            out.push(ButtonBind { action: dpad[i], code, half: Half::Full });
        }
        if let Some(code) = info.std(a) {
            out.push(ButtonBind { action: dpad[i], code, half });
        }
    }
}

/// Default profile for a device.
pub fn profile(info: &DeviceInfo) -> Profile {
    let n = info.name.to_lowercase();
    let has = |ws: &[&str]| ws.iter().any(|w| n.contains(w));
    let mut axes: Vec<Option<AxisBind>> = Vec::new();
    let mut buttons: Vec<Option<ButtonBind>> = Vec::new();
    let mut hats = Vec::new();
    let preset;
    match info.kind {
        DevKind::Gamepad => {
            preset = if !has(&["xbox", "x-box"]) && has(&["playstation", "dualsense", "dualshock", "wireless controller", "ps4", "ps5"]) {
                "PlayStation-style gamepad"
            } else {
                "Xbox-style gamepad"
            };
            // Left stick flies (with some expo: thumbsticks are short), triggers are the rudder,
            // right stick looks around, bumpers ramp the throttle like the keys.
            axes.push(axis(info, Std::X, AxisAction::Roll, Half::Full, false, 0.10, 0.35));
            axes.push(axis(info, Std::Y, AxisAction::Pitch, Half::Full, true, 0.10, 0.35));
            axes.push(axis(info, Std::RT2, AxisAction::Yaw, Half::Pos, false, 0.05, 0.2));
            axes.push(axis(info, Std::LT2, AxisAction::Yaw, Half::Pos, true, 0.05, 0.2));
            axes.push(axis(info, Std::Rx, AxisAction::LookX, Half::Full, false, 0.15, 0.3));
            axes.push(axis(info, Std::Ry, AxisAction::LookY, Half::Full, false, 0.15, 0.3));
            for (s, k) in [
                (Std::South, Key::Brakes),
                (Std::East, Key::Gear),
                (Std::West, Key::FlapsDown),
                (Std::North, Key::FlapsUp),
                (Std::LB, Key::ThrottleDown),
                (Std::RB, Key::ThrottleUp),
                (Std::Select, Key::Camera),
                (Std::Start, Key::Menu),
                (Std::Mode, Key::AutoFlight),
                (Std::LThumb, Key::ApToggle),
                (Std::RThumb, Key::LookCenter),
            ] {
                buttons.push(btn(info, s, k));
            }
            hat(info, &mut hats, [Key::TrimDown, Key::TrimUp, Key::SpoilerIn, Key::SpoilerOut]);
        }
        DevKind::Pedals => {
            preset = "Rudder pedals";
            axes.push(axis(info, Std::Rz, AxisAction::Yaw, Half::Full, false, 0.03, 0.15));
            axes.push(axis(info, Std::X, AxisAction::BrakeLeft, Half::Full, false, 0.05, 0.0));
            // The Y axis is reported "up = positive" (flipped), so a released toe brake reads +1.
            axes.push(axis(info, Std::Y, AxisAction::BrakeRight, Half::Full, true, 0.05, 0.0));
        }
        DevKind::Throttle => {
            if has(&["q-eng", "quadrant", "x-56", "x56", "tqs"]) {
                preset = "Twin throttle quadrant";
                axes.push(axis(info, Std::X, AxisAction::Throttle1, Half::Full, true, 0.02, 0.0));
                axes.push(axis(info, Std::Y, AxisAction::Throttle2, Half::Full, false, 0.02, 0.0));
            } else {
                preset = "Throttle";
                axes.push(axis(info, Std::Z, AxisAction::Throttle, Half::Full, true, 0.02, 0.0));
                axes.push(axis(info, Std::Rz, AxisAction::Yaw, Half::Full, false, 0.06, 0.2));
            }
            for (i, k) in [Key::Reverse, Key::SpoilerOut, Key::SpoilerIn, Key::FlapsDown, Key::FlapsUp, Key::Gear].into_iter().enumerate() {
                buttons.push(btn(info, Std::Joy(i as u8), k));
            }
        }
        DevKind::Joystick => {
            let combined = has(&["x52", "x-52", "hotas", "t.flight"]);
            preset = if combined { "HOTAS (stick + throttle)" } else { "Joystick" };
            axes.push(axis(info, Std::X, AxisAction::Roll, Half::Full, false, 0.03, 0.2));
            axes.push(axis(info, Std::Y, AxisAction::Pitch, Half::Full, true, 0.03, 0.2));
            axes.push(axis(info, Std::Rz, AxisAction::Yaw, Half::Full, false, 0.08, 0.25));
            let thr = if combined {
                Std::Z
            } else if info.std(Std::Throttle).is_some() {
                Std::Throttle
            } else {
                Std::Z
            };
            axes.push(axis(info, thr, AxisAction::Throttle, Half::Full, true, 0.02, 0.0));
            for (i, k) in [Key::Brakes, Key::Gear, Key::FlapsUp, Key::FlapsDown, Key::SpoilerIn, Key::SpoilerOut, Key::TrimDown, Key::TrimUp, Key::Camera, Key::ApToggle]
                .into_iter()
                .enumerate()
            {
                buttons.push(btn(info, Std::Joy(i as u8), k));
            }
            hat(info, &mut hats, [Key::LookUp, Key::LookDown, Key::LookLeft, Key::LookRight]);
        }
    }
    let mut buttons: Vec<ButtonBind> = buttons.into_iter().flatten().collect();
    buttons.extend(hats);
    Profile { name: info.name.clone(), enabled: true, preset: preset.into(), axes: axes.into_iter().flatten().collect(), buttons }
}

// Linux evdev codes as reported by the backends: (event type << 16) | code.
const ABS: u32 = 3 << 16;
const KEY: u32 = 1 << 16;

/// Standard controls at their Linux evdev codes.
pub fn linux_table() -> Vec<(Std, u32)> {
    let mut t = vec![
        (Std::X, ABS),
        (Std::Y, ABS | 1),
        (Std::Z, ABS | 2),
        (Std::Rx, ABS | 3),
        (Std::Ry, ABS | 4),
        (Std::Rz, ABS | 5),
        (Std::Throttle, ABS | 6),
        (Std::Rudder, ABS | 7),
        (Std::Wheel, ABS | 8),
        (Std::Gas, ABS | 9),
        (Std::Brake, ABS | 10),
        (Std::HatX, ABS | 0x10),
        (Std::HatY, ABS | 0x11),
        (Std::Slider, ABS | 0x28),
        (Std::South, KEY | 0x130),
        (Std::East, KEY | 0x131),
        (Std::North, KEY | 0x133),
        (Std::West, KEY | 0x134),
        (Std::LB, KEY | 0x136),
        (Std::RB, KEY | 0x137),
        (Std::Select, KEY | 0x13a),
        (Std::Start, KEY | 0x13b),
        (Std::Mode, KEY | 0x13c),
        (Std::LThumb, KEY | 0x13d),
        (Std::RThumb, KEY | 0x13e),
        (Std::DUp, KEY | 0x220),
        (Std::DDown, KEY | 0x221),
        (Std::DLeft, KEY | 0x222),
        (Std::DRight, KEY | 0x223),
    ];
    // BTN_TRIGGER .. BTN_DEAD (joystick buttons 1-16), then BTN_TRIGGER_HAPPY1.. (17-56).
    t.extend((0..16u8).map(|i| (Std::Joy(i), KEY | (0x120 + i as u32))));
    t.extend((0..40u8).map(|i| (Std::Joy(16 + i), KEY | (0x2c0 + i as u32))));
    t
}

/// Virtual devices for screenshots and tests, with Linux codes and resting values.
pub fn fake_device(name: &str, id: u64) -> Option<(DeviceInfo, Vec<super::RawValue>)> {
    let l = linux_table();
    let code = |s: Std| l.iter().find(|e| e.0 == s).unwrap().1;
    type Spec = (&'static str, bool, Vec<(Std, u32)>, Vec<(Std, f32, bool)>);
    let (full, mapped, mut std, values): Spec = match name {
        "xbox" => (
            "Xbox Wireless Controller",
            true,
            // xpad: triggers on Z / Rz.
            vec![(Std::LT2, ABS | 2), (Std::RT2, ABS | 5)],
            vec![(Std::X, 0.32, false), (Std::Y, -0.18, false), (Std::Rx, 0.0, false), (Std::Ry, 0.0, false), (Std::LT2, 0.0, true), (Std::RT2, 0.15, true)],
        ),
        "x52" => ("Saitek X52 Flight Control System", false, vec![], vec![(Std::X, -0.12, false), (Std::Y, 0.22, false), (Std::Z, -0.45, false), (Std::Rz, 0.04, false)]),
        "t16000m" => ("Thrustmaster T.16000M", false, vec![], vec![(Std::X, 0.0, false), (Std::Y, 0.0, false), (Std::Rz, 0.0, false), (Std::Throttle, -1.0, false)]),
        "tca" => ("Thrustmaster TCA Q-Eng 1&2", false, vec![], vec![(Std::X, -0.6, false), (Std::Y, 0.6, false)]),
        "pedals" => ("Saitek Pro Flight Rudder Pedals", false, vec![], vec![(Std::X, -1.0, false), (Std::Y, 1.0, false), (Std::Rz, 0.18, false)]),
        _ => return None,
    };
    // The xpad d-pad is a hat: no d-pad buttons.
    let pad_hat = |s: &Std| mapped && matches!(s, Std::DUp | Std::DDown | Std::DLeft | Std::DRight);
    let extra: Vec<(Std, u32)> = l.iter().copied().filter(|(s, _)| !std.iter().any(|e| e.0 == *s) && !pad_hat(s)).collect();
    std.extend(extra);
    let info = DeviceInfo { id, key: format!("virtual-{name}"), name: full.into(), kind: classify(full, mapped), mapped, backend: "virtual", std };
    let values = values.into_iter().map(|(s, v, b)| (info.std(s).unwrap_or_else(|| code(s)), v, b)).collect();
    Some((info, values))
}
