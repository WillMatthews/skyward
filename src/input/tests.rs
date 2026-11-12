//! Mapping layer tests with synthetic devices and events (no hardware).

use super::*;

const ABS: u32 = 3 << 16;
const KEY: u32 = 1 << 16;

fn fake(sys: &mut InputSystem, cfg: &mut Config, name: &str, id: u64) -> DeviceInfo {
    let (info, values) = defaults::fake_device(name, id).unwrap();
    sys.connect_virtual(info.clone(), &values);
    sys.update(cfg, 0.016);
    info
}

#[test]
fn deadzone_expo_invert_sens() {
    let c = AxisCfg { deadzone: 0.1, ..AxisCfg::default() };
    assert_eq!(c.apply(0.05, Half::Full, false), 0.0);
    assert_eq!(c.apply(-0.1, Half::Full, false), 0.0);
    assert!((c.apply(0.55, Half::Full, false) - 0.5).abs() < 1e-5, "rescaled past the deadzone");
    assert_eq!(c.apply(1.0, Half::Full, false), 1.0);
    assert_eq!(c.apply(-1.0, Half::Full, false), -1.0);
    let inv = AxisCfg { invert: true, deadzone: 0.0, ..AxisCfg::default() };
    assert_eq!(inv.apply(0.4, Half::Full, false), -0.4);
    // Expo: softer near centre, same at full deflection, monotonic.
    let e = AxisCfg { expo: 0.5, deadzone: 0.0, ..AxisCfg::default() };
    assert!(e.apply(0.3, Half::Full, false) < 0.3 * 0.7);
    assert_eq!(e.apply(1.0, Half::Full, false), 1.0);
    let mut last = -2.0;
    for i in -20..=20 {
        let v = e.apply(i as f32 / 20.0, Half::Full, false);
        assert!(v >= last);
        last = v;
    }
    let s = AxisCfg { sens: 2.0, deadzone: 0.0, ..AxisCfg::default() };
    assert_eq!(s.apply(0.25, Half::Full, false), 0.5);
    assert_eq!(s.apply(0.9, Half::Full, false), 1.0, "clamped");
}

#[test]
fn levers_and_halves() {
    // A lever −1..1 → 0..1 with the deadzone at idle.
    let c = AxisCfg { deadzone: 0.05, ..AxisCfg::default() };
    assert_eq!(c.apply(-1.0, Half::Full, true), 0.0);
    assert_eq!(c.apply(-0.95, Half::Full, true), 0.0);
    assert_eq!(c.apply(1.0, Half::Full, true), 1.0);
    assert!((c.apply(0.0, Half::Full, true) - (0.5 - 0.05) / 0.95).abs() < 1e-5);
    // Inverted lever (forward = −1 on most throttles) reads full forward.
    let inv = AxisCfg { invert: true, deadzone: 0.0, ..AxisCfg::default() };
    assert_eq!(inv.apply(-1.0, Half::Full, true), 1.0);
    // Trigger halves for a centred action: left trigger negative, right positive.
    let t = AxisCfg { deadzone: 0.0, ..AxisCfg::default() };
    assert_eq!(t.apply(0.6, Half::Pos, false), 0.6);
    assert_eq!(AxisCfg { invert: true, ..t }.apply(0.6, Half::Pos, false), -0.6);
    assert_eq!(t.apply(-0.6, Half::Pos, false), 0.0);
    assert_eq!(t.apply(-0.6, Half::Neg, false), 0.6);
}

#[test]
fn calibration_maps_measured_range() {
    let c = AxisCfg { deadzone: 0.0, cal: Some([-0.8, 0.1, 0.9]), ..AxisCfg::default() };
    assert_eq!(c.apply(0.1, Half::Full, false), 0.0);
    assert_eq!(c.apply(0.9, Half::Full, false), 1.0);
    assert_eq!(c.apply(-0.8, Half::Full, false), -1.0);
    assert!((c.apply(0.5, Half::Full, false) - 0.5).abs() < 1e-5);
}

#[test]
fn gamepad_defaults_drive_analog_and_buttons() {
    let mut sys = InputSystem::new();
    let mut cfg = Config::default();
    let info = fake(&mut sys, &mut cfg, "xbox", 1);
    assert_eq!(info.kind, DevKind::Gamepad);
    let p = cfg.profile("virtual-xbox").expect("default profile created on connect");
    assert_eq!(p.preset, "Xbox-style gamepad");
    // Left stick: X 0.32 → roll right; Y −0.18 (pulled back) → nose up.
    let a = sys.analog;
    assert!(a.get(AxisAction::Roll).unwrap() > 0.05);
    assert!(a.get(AxisAction::Pitch).unwrap() > 0.0);
    // Right trigger 0.15 → a little right rudder.
    assert!(a.get(AxisAction::Yaw).unwrap() > 0.0);
    sys.inject(1, ABS | 2, 0.9, true); // left trigger hard
    sys.update(&mut cfg, 0.016);
    assert!(sys.analog.get(AxisAction::Yaw).unwrap() < -0.4);
    // East (B) = gear: one press edge, held while down.
    sys.inject(1, KEY | 0x131, 1.0, true);
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.take_pressed(), vec![Key::Gear]);
    sys.update(&mut cfg, 0.016);
    assert!(sys.take_pressed().is_empty(), "no repeat while held");
    sys.inject(1, KEY | 0x131, 0.0, true);
    sys.update(&mut cfg, 0.016);
    assert!(sys.held.is_empty());
    // D-pad via hat axis: up = trim nose down.
    sys.inject(1, ABS | 0x11, 1.0, false);
    sys.update(&mut cfg, 0.016);
    assert!(sys.held.contains(&Key::TrimDown));
    // Unplugged: analog goes away, nothing stuck.
    sys.disconnect_virtual(1);
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.analog.get(AxisAction::Roll), None);
    assert!(sys.held.is_empty());
    assert_eq!(sys.left, vec!["Xbox Wireless Controller".to_string()]);
}

#[test]
fn hotas_pedals_and_quadrant_together() {
    let mut sys = InputSystem::new();
    let mut cfg = Config::default();
    fake(&mut sys, &mut cfg, "x52", 1);
    fake(&mut sys, &mut cfg, "pedals", 2);
    fake(&mut sys, &mut cfg, "tca", 3);
    assert_eq!(cfg.devices.len(), 3);
    assert_eq!(cfg.profile("virtual-x52").unwrap().preset, "HOTAS (stick + throttle)");
    assert_eq!(cfg.profile("virtual-pedals").unwrap().preset, "Rudder pedals");
    assert_eq!(cfg.profile("virtual-tca").unwrap().preset, "Twin throttle quadrant");
    let a = sys.analog;
    // X52 throttle on Z at −0.45 (inverted: forward = −1) → ~72 %.
    let thr = a.get(AxisAction::Throttle).unwrap();
    assert!((thr - 0.71).abs() < 0.03, "{thr}");
    // Pedals at rest: brakes released, a little right rudder (plus the stick's twist).
    assert_eq!(a.get(AxisAction::BrakeLeft), Some(0.0));
    assert_eq!(a.get(AxisAction::BrakeRight), Some(0.0));
    assert!(a.get(AxisAction::Yaw).unwrap() > 0.1);
    // Quadrant: both levers read the same position.
    let (t1, t2) = (a.get(AxisAction::Throttle1).unwrap(), a.get(AxisAction::Throttle2).unwrap());
    assert!((t1 - t2).abs() < 1e-5 && (t1 - 0.8).abs() < 0.03, "{t1} {t2}");
    // Toe brake pressed (Y flipped: pressed = −1).
    sys.inject(2, ABS | 1, -1.0, false);
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.analog.get(AxisAction::BrakeRight), Some(1.0));
    // Disabling a device removes its contribution.
    cfg.profile_mut("virtual-pedals").unwrap().enabled = false;
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.analog.get(AxisAction::BrakeRight), None);
}

#[test]
fn listen_captures_axis_and_button() {
    let mut sys = InputSystem::new();
    let mut cfg = Config::default();
    fake(&mut sys, &mut cfg, "t16000m", 7);
    sys.start_listening(Listen::Axis(AxisAction::Flaps));
    // Noise does not bind; a real movement does.
    sys.inject(7, ABS | 6, -0.9, false);
    sys.update(&mut cfg, 0.016);
    assert!(sys.captured.is_none());
    sys.inject(7, ABS | 6, 0.2, false);
    sys.update(&mut cfg, 0.016);
    let Some(Captured::Axis { action, dev, code, half }) = sys.captured.take() else { panic!("no capture") };
    assert_eq!((action, code, half), (AxisAction::Flaps, ABS | 6, Half::Full));
    cfg.bind_axis(&dev, action, code, half);
    assert!(cfg.profile(&dev).unwrap().axes.iter().any(|b| b.action == AxisAction::Flaps && b.code == ABS | 6));
    // A hat direction captured as a button.
    sys.start_listening(Listen::Button(Key::Gear));
    sys.inject(7, ABS | 0x10, -1.0, false);
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.captured, Some(Captured::Button { action: Key::Gear, dev: "virtual-t16000m".into(), code: ABS | 0x10, half: Half::Neg }));
    // Listening swallows the input (no action fired).
    assert!(sys.take_pressed().is_empty());
}

#[test]
fn conflicts_and_round_trip() {
    let mut cfg = Config::default();
    assert!(cfg.conflicts().is_empty(), "defaults are conflict-free: {:?}", cfg.conflicts());
    cfg.bind_key(Key::Gear, KeyCode::KeyF);
    let c = cfg.conflicts();
    assert_eq!(c.len(), 1);
    assert!(c[0].1.contains(&Key::Gear) && c[0].1.contains(&Key::FlapsDown));
    let (info, _) = defaults::fake_device("x52", 1).unwrap();
    cfg.devices.push((info.key.clone(), defaults::profile(&info)));
    cfg.profile_mut(&info.key).unwrap().axes[0].cfg = AxisCfg { invert: true, deadzone: 0.12, expo: 0.4, sens: 1.3, cal: Some([-0.9, 0.0, 0.95]) };
    cfg.mouse.yoke = true;
    let back = Config::from_json(&cfg.to_json());
    assert_eq!(back, cfg);
    // Every action has an id, every default key a name.
    for (k, codes) in defaults::keyboard() {
        assert_ne!(key_id(k), "?");
        for c in codes {
            assert!(keys::name(c).is_some(), "{c:?}");
        }
    }
}

#[test]
fn non_controllers_are_ignored() {
    assert!(!defaults::is_controller("Keychron Keychron V6 System Control"));
    assert!(!defaults::is_controller("Logitech G502 HERO Gaming Mouse"));
    assert!(defaults::is_controller("Xbox Wireless Controller"));
    assert!(defaults::is_controller("Saitek Pro Flight Rudder Pedals"));
    let mut sys = InputSystem::new();
    let mut cfg = Config::default();
    let (mut info, _) = defaults::fake_device("x52", 5).unwrap();
    info.name = "Keychron Keychron V6 System Control".into();
    sys.connect_virtual(info, &[]);
    sys.update(&mut cfg, 0.016);
    assert!(sys.devices.is_empty() && cfg.devices.is_empty());
    assert_eq!(sys.ignored.len(), 1);
}

#[test]
fn classify_names() {
    use defaults::classify;
    assert_eq!(classify("Saitek Pro Flight Rudder Pedals", false), DevKind::Pedals);
    assert_eq!(classify("Thrustmaster T-Rudder", false), DevKind::Pedals);
    assert_eq!(classify("Thrustmaster TCA Q-Eng 1&2", false), DevKind::Throttle);
    assert_eq!(classify("Saitek Pro Flight X-56 Rhino Throttle", false), DevKind::Throttle);
    assert_eq!(classify("Logitech X56 H.O.T.A.S. Stick", true), DevKind::Joystick);
    assert_eq!(classify("Logitech Extreme 3D pro", false), DevKind::Joystick);
    assert_eq!(classify("Sony Interactive Entertainment DualSense Wireless Controller", false), DevKind::Gamepad);
    assert_eq!(classify("Some Pad", true), DevKind::Gamepad);
}

#[test]
fn mouse_yoke_needs_setting_and_toggle() {
    let mut sys = InputSystem::new();
    let mut cfg = Config::default();
    sys.mouse_offset = Some((0.5, -0.5));
    sys.mouse_yoke_on = true;
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.analog.get(AxisAction::Roll), None, "off in settings");
    cfg.mouse.yoke = true;
    sys.update(&mut cfg, 0.016);
    assert!(sys.analog.get(AxisAction::Roll).unwrap() > 0.2);
    assert!(sys.analog.get(AxisAction::Pitch).unwrap() < -0.2, "cursor above centre = push");
    sys.mouse_offset = Some((0.01, 0.0));
    sys.update(&mut cfg, 0.016);
    assert_eq!(sys.analog.get(AxisAction::Roll), Some(0.0), "dead band in the middle");
}
