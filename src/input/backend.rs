//! Device backends → [`Event`]s with platform input codes.
//!
//! * gilrs: gamepads and joysticks on Linux (evdev), Windows (Windows.Gaming.Input),
//!   macOS (IOKit) and the web (Gamepad API); hot-plug; SDL mappings for pads.
//! * Linux only: raw evdev for axis-only devices (most rudder pedals, some
//!   throttle quadrants) — gilrs ignores devices without buttons.
//!
//! Codes are what the platform reports (on Linux `(type << 16) | code`, the
//! same in both backends, so a profile works whichever one found the device).

use super::{DeviceInfo, Std, defaults};

#[derive(Clone, Debug)]
pub enum Event {
    Connected(DeviceInfo),
    Disconnected(u64),
    /// Axis −1..1 (Y axes up = positive) or button 0..1.
    Value {
        id: u64,
        code: u32,
        value: f32,
        button: bool,
    },
}

/// Standard meaning of a Linux input code (labels; other platforms: none).
pub fn linux_std(code: u32) -> Option<Std> {
    if cfg!(target_os = "linux") { defaults::linux_table().into_iter().find(|e| e.1 == code).map(|e| e.0) } else { None }
}

/// Profile key from a device's bus / vendor / product / version, laid out like
/// gilrs' (SDL-style) id so both backends key a device the same way.
pub fn uuid_key(bus: u16, vendor: u16, product: u16, version: u16) -> String {
    let mut b = [0u8; 16];
    b[0..4].copy_from_slice(&(bus as u32).to_le_bytes());
    b[4..6].copy_from_slice(&vendor.to_le_bytes());
    b[8..10].copy_from_slice(&product.to_le_bytes());
    b[12..14].copy_from_slice(&version.to_le_bytes());
    hex(&b)
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

pub struct Backends {
    gilrs: Option<gilrs::Gilrs>,
    error: Option<String>,
    #[cfg(target_os = "linux")]
    evdev: evdev_axes::Scanner,
    /// Connected devices found by gilrs (to report them once).
    pending: Vec<Event>,
}

impl Backends {
    pub fn new() -> Backends {
        let (gilrs, error) = match gilrs::GilrsBuilder::new().with_default_filters(false).build() {
            Ok(g) => (Some(g), None),
            Err(gilrs::Error::NotImplemented(g)) => (Some(g), Some("controllers are not supported on this platform".to_string())),
            Err(e) => (None, Some(e.to_string())),
        };
        let mut pending = Vec::new();
        if let Some(g) = &gilrs {
            for (_, pad) in g.gamepads() {
                pending.push(Event::Connected(gilrs_info(&pad)));
            }
        }
        Backends {
            gilrs,
            error,
            #[cfg(target_os = "linux")]
            evdev: evdev_axes::Scanner::default(),
            pending,
        }
    }

    pub fn status(&self) -> String {
        match &self.error {
            Some(e) => format!("Controllers unavailable: {e}"),
            None => "Controllers: listening (hot-plug)".into(),
        }
    }

    pub fn poll(&mut self, out: &mut Vec<Event>) {
        out.append(&mut self.pending);
        if let Some(g) = self.gilrs.as_mut() {
            use gilrs::EventType as E;
            while let Some(ev) = g.next_event() {
                let id = usize::from(ev.id) as u64;
                let state_value = |g: &gilrs::Gilrs, c: gilrs::ev::Code, def: f32| g.gamepad(ev.id).state().button_data(c).map(|d| d.value()).unwrap_or(def);
                match ev.event {
                    E::Connected => out.push(Event::Connected(gilrs_info(&g.gamepad(ev.id)))),
                    E::Disconnected => out.push(Event::Disconnected(id)),
                    // Analog buttons (triggers) send their value; plain ones 0 / 1.
                    E::ButtonPressed(_, c) => out.push(Event::Value { id, code: c.into_u32(), value: state_value(g, c, 1.0).max(0.51), button: true }),
                    E::ButtonReleased(_, c) => out.push(Event::Value { id, code: c.into_u32(), value: state_value(g, c, 0.0).min(0.49), button: true }),
                    E::ButtonChanged(_, v, c) => out.push(Event::Value { id, code: c.into_u32(), value: v, button: true }),
                    E::AxisChanged(_, v, c) => out.push(Event::Value { id, code: c.into_u32(), value: v, button: false }),
                    _ => {}
                }
            }
        }
        #[cfg(target_os = "linux")]
        self.evdev.poll(out);
    }
}

fn gilrs_info(g: &gilrs::Gamepad) -> DeviceInfo {
    use gilrs::{Axis as A, Button as B};
    let mapped = matches!(g.mapping_source(), gilrs::MappingSource::SdlMappings);
    let mut std: Vec<(Std, u32)> = Vec::new();
    for (s, a) in [
        (Std::X, A::LeftStickX),
        (Std::Y, A::LeftStickY),
        (Std::Z, A::LeftZ),
        (Std::Rx, A::RightStickX),
        (Std::Ry, A::RightStickY),
        (Std::Rz, A::RightZ),
        (Std::HatX, A::DPadX),
        (Std::HatY, A::DPadY),
    ] {
        if let Some(c) = g.axis_code(a) {
            std.push((s, c.into_u32()));
        }
    }
    for (s, b) in [
        (Std::South, B::South),
        (Std::East, B::East),
        (Std::North, B::North),
        (Std::West, B::West),
        (Std::LB, B::LeftTrigger),
        (Std::RB, B::RightTrigger),
        (Std::LT2, B::LeftTrigger2),
        (Std::RT2, B::RightTrigger2),
        (Std::Select, B::Select),
        (Std::Start, B::Start),
        (Std::Mode, B::Mode),
        (Std::LThumb, B::LeftThumb),
        (Std::RThumb, B::RightThumb),
        (Std::DUp, B::DPadUp),
        (Std::DDown, B::DPadDown),
        (Std::DLeft, B::DPadLeft),
        (Std::DRight, B::DPadRight),
    ] {
        if let Some(c) = g.button_code(b) {
            std.push((s, c.into_u32()));
        }
    }
    if cfg!(target_os = "linux") {
        // Codes gilrs has no name for (throttle, rudder, joystick buttons...).
        for e in defaults::linux_table() {
            if !std.iter().any(|x| x.0 == e.0) {
                std.push(e);
            }
        }
    }
    let uuid = g.uuid();
    let name = g.name().to_string();
    let key = if uuid.iter().all(|b| *b == 0) { format!("name:{name}") } else { hex(&uuid) };
    DeviceInfo { id: usize::from(g.id()) as u64, key, kind: defaults::classify(&name, mapped), name, mapped, backend: "gilrs", std }
}

#[cfg(target_os = "linux")]
mod evdev_axes {
    //! Axis-only evdev devices (rudder pedals...). Rescans /dev/input when it changes.
    use super::{DeviceInfo, Event};
    use evdev::{AbsoluteAxisCode as Abs, Device, EventType};
    use std::collections::HashSet;
    use std::path::PathBuf;
    use std::time::{Duration, Instant, SystemTime};

    struct Open {
        path: PathBuf,
        dev: Device,
        id: u64,
        /// (abs code, min, max)
        ranges: Vec<(u16, i32, i32)>,
    }

    #[derive(Default)]
    pub struct Scanner {
        open: Vec<Open>,
        tried: HashSet<PathBuf>,
        dir_mtime: Option<SystemTime>,
        checked: Option<Instant>,
        next_id: u64,
    }

    const JOY_AXES: [Abs; 11] =
        [Abs::ABS_X, Abs::ABS_Y, Abs::ABS_Z, Abs::ABS_RX, Abs::ABS_RY, Abs::ABS_RZ, Abs::ABS_THROTTLE, Abs::ABS_RUDDER, Abs::ABS_WHEEL, Abs::ABS_GAS, Abs::ABS_BRAKE];

    /// gilrs' normalisation: min..max → −1..1, Y axes flipped (up = positive).
    pub fn normalise(code: u16, v: i32, min: i32, max: i32) -> f32 {
        let mut range = max as f32 - min as f32;
        let mut x = v as f32 - min as f32;
        if (max as i64 - min as i64) % 2 == 1 {
            range += 1.0;
            x += 1.0;
        }
        let mut n = if range > 0.0 { x / range * 2.0 - 1.0 } else { 0.0 };
        if matches!(code, 0x01 | 0x04 | 0x11) && n != 0.0 {
            n = -n;
        }
        n.clamp(-1.0, 1.0)
    }

    impl Scanner {
        pub fn poll(&mut self, out: &mut Vec<Event>) {
            if self.checked.is_none_or(|t| t.elapsed() > Duration::from_secs(2)) {
                self.checked = Some(Instant::now());
                let m = std::fs::metadata("/dev/input").and_then(|m| m.modified()).ok();
                if m != self.dir_mtime {
                    self.dir_mtime = m;
                    self.tried.clear();
                    self.rescan(out);
                }
            }
            let mut gone = Vec::new();
            for (i, o) in self.open.iter_mut().enumerate() {
                match o.dev.fetch_events() {
                    Ok(evs) => {
                        for e in evs {
                            if e.event_type() == EventType::ABSOLUTE
                                && let Some(r) = o.ranges.iter().find(|r| r.0 == e.code())
                            {
                                out.push(Event::Value { id: o.id, code: (3 << 16) | e.code() as u32, value: normalise(e.code(), e.value(), r.1, r.2), button: false });
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(_) => gone.push(i),
                }
            }
            for i in gone.into_iter().rev() {
                let o = self.open.remove(i);
                self.tried.remove(&o.path);
                out.push(Event::Disconnected(o.id));
            }
        }

        fn rescan(&mut self, out: &mut Vec<Event>) {
            let Ok(dir) = std::fs::read_dir("/dev/input") else { return };
            let mut paths: Vec<PathBuf> = dir.flatten().map(|e| e.path()).filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("event"))).collect();
            paths.sort();
            for p in paths {
                if self.open.iter().any(|o| o.path == p) || !self.tried.insert(p.clone()) {
                    continue;
                }
                let Ok(dev) = Device::open(&p) else { continue };
                if !accept(&dev) || dev.set_nonblocking(true).is_err() {
                    continue;
                }
                let Ok(abs) = dev.get_absinfo() else { continue };
                let abs: Vec<(u16, i32, i32, i32)> = abs.map(|(c, i)| (c.0, i.minimum(), i.maximum(), i.value())).collect();
                let id = (1u64 << 40) | self.next_id;
                self.next_id += 1;
                let iid = dev.input_id();
                let name = dev.name().unwrap_or("Axis device").to_string();
                let info = DeviceInfo {
                    id,
                    key: super::uuid_key(iid.bus_type().0, iid.vendor(), iid.product(), iid.version()),
                    kind: super::defaults::classify(&name, false),
                    name,
                    mapped: false,
                    backend: "evdev",
                    std: super::defaults::linux_table(),
                };
                out.push(Event::Connected(info));
                for &(c, min, max, v) in &abs {
                    out.push(Event::Value { id, code: (3 << 16) | c as u32, value: normalise(c, v, min, max), button: false });
                }
                self.open.push(Open { path: p, dev, id, ranges: abs.iter().map(|a| (a.0, a.1, a.2)).collect() });
            }
        }
    }

    /// Joystick-type axes and no buttons, keys, relative axes, touch or motion
    /// sensors (those are gilrs', a keyboard / mouse, a touchpad or an IMU).
    fn accept(d: &Device) -> bool {
        let Some(abs) = d.supported_absolute_axes() else { return false };
        let no_keys = d.supported_keys().is_none_or(|k| k.iter().next().is_none());
        let no_rel = d.supported_relative_axes().is_none_or(|r| r.iter().next().is_none());
        let touch = abs.contains(Abs::ABS_MT_SLOT) || abs.contains(Abs::ABS_MT_POSITION_X);
        let imu = d.properties().contains(evdev::PropType::ACCELEROMETER) || d.name().is_some_and(|n| n.contains("Motion") || n.contains("Accel"));
        no_keys && no_rel && !touch && !imu && JOY_AXES.iter().any(|a| abs.contains(*a))
    }

    #[cfg(test)]
    mod tests {
        #[test]
        fn normalise_like_gilrs() {
            assert!(super::normalise(0, 0, 0, 1023) < -0.99);
            assert!(super::normalise(0, 1023, 0, 1023) > 0.99);
            assert!(super::normalise(0, 511, 0, 1023).abs() < 0.01);
            // Y is flipped: pushed forward (min) = +1.
            assert!(super::normalise(1, 0, 0, 255) > 0.99);
            assert!(super::normalise(5, -32768, -32768, 32767) < -0.99);
        }
    }
}
