//! Analog inputs and head-look inside the simulation (a child module of
//! `sim`, so it can reach `Sim`'s private state; see `input/mod.rs` for the
//! device side).
//!
//! * Stick-type axes (pitch / roll / rudder) drive the controls directly —
//!   no keyboard smoothing — unless a key for the same axis is held.
//! * Levers (throttles, flaps, speed brake, trim) apply when they move, so
//!   the keys can still adjust in between and a lever left at idle does not
//!   cut the power of an air start.
//! * Automation (autopilot, autothrottle, the bot) keeps control until an
//!   input moves clearly away from where it was when automation took over:
//!   analog noise and a resting stick never disconnect anything.

use super::{CamMode, Input, Key, SPOILER_STEPS, Sim};
use crate::input::{Analog, AxisAction as A};
use glam::{Quat, Vec2};

/// Pilot-input state kept by the simulation.
#[derive(Clone, Debug, Default)]
pub struct Pilot {
    /// Each lever's value when it was last applied (None: not seen yet).
    lever: [Option<f32>; A::COUNT],
    /// Each input's value when automation took over.
    auto_ref: [Option<f32>; A::COUNT],
    /// Head direction in the cockpit (yaw right, pitch up; radians), smoothed.
    pub look: Vec2,
    /// Where the mouse has put the head.
    pub free: Vec2,
}

/// Inputs that moved enough to take control back from automation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Takeover {
    pub stick: bool,
    pub throttle: bool,
    pub levers: bool,
}

/// How far an input must move (from where it was when automation took over)
/// to count as the pilot taking control.
pub fn takeover_threshold(a: A) -> f32 {
    match a {
        A::Pitch | A::Roll => 0.25,
        A::Yaw => 0.35,
        A::Throttle | A::Throttle1 | A::Throttle2 => 0.08,
        A::Flaps | A::Spoilers | A::Trim => 0.15,
        A::Brakes | A::BrakeLeft | A::BrakeRight => 0.5,
        A::LookX | A::LookY => f32::INFINITY,
    }
}

impl Pilot {
    /// `automated(a)`: is automation flying what this input controls?
    pub fn takeover(&mut self, an: &Analog, automated: impl Fn(A) -> bool) -> Takeover {
        let mut t = Takeover::default();
        for a in A::ALL {
            let i = a.idx();
            let (Some(v), true) = (an.get(a), automated(a)) else {
                self.auto_ref[i] = None;
                continue;
            };
            let r = *self.auto_ref[i].get_or_insert(v);
            if (v - r).abs() > takeover_threshold(a) {
                match a {
                    A::Pitch | A::Roll | A::Yaw => t.stick = true,
                    A::Throttle | A::Throttle1 | A::Throttle2 => t.throttle = true,
                    _ => t.levers = true,
                }
            }
        }
        if t != Takeover::default() {
            self.auto_ref = [None; A::COUNT];
        }
        t
    }

    /// A lever's new value if it moved since last applied (first sight only records it).
    pub fn latch(&mut self, a: A, v: Option<f32>) -> Option<f32> {
        let v = v?;
        let slot = &mut self.lever[a.idx()];
        match *slot {
            Some(old) if (v - old).abs() <= 0.01 => None,
            Some(_) => {
                *slot = Some(v);
                Some(v)
            }
            None => {
                *slot = Some(v);
                None
            }
        }
    }

    /// Head orientation relative to the aircraft.
    pub fn look_rot(&self) -> Quat {
        Quat::from_rotation_y(-self.look.x) * Quat::from_rotation_x(-self.look.y)
    }
}

/// Throttle lever: the all-engines axis, else the mean of the per-engine ones.
pub fn throttle_lever(an: &Analog) -> Option<f32> {
    an.get(A::Throttle).or_else(|| match (an.get(A::Throttle1), an.get(A::Throttle2)) {
        (Some(a), Some(b)) => Some(0.5 * (a + b)),
        (a, b) => a.or(b),
    })
}

/// Wheel brakes from the brake axes (toe brakes: the harder pressed; the
/// flight model has one brake channel, differential braking comes from the rudder).
pub fn toe_brakes(an: &Analog) -> f32 {
    [A::Brakes, A::BrakeLeft, A::BrakeRight].into_iter().filter_map(|a| an.get(a)).fold(0.0f32, f32::max)
}

impl Sim {
    /// Analog takeover from the bot / autopilot (before they fly this step).
    pub(super) fn analog_takeover(&mut self, input: &Input) -> Takeover {
        let bot = self.bot.is_some();
        let ap = self.ap.engaged;
        let athr = ap && self.ap.ias.is_some();
        self.pilot.takeover(&input.analog, |a| match a {
            A::Pitch | A::Roll | A::Yaw | A::Trim => bot || ap,
            A::Throttle | A::Throttle1 | A::Throttle2 => bot || athr,
            _ => bot,
        })
    }

    /// Analog axes → controls (after the keyboard's smoothing).
    pub(super) fn apply_analog(&mut self, input: &Input, dt: f32) {
        let an = input.analog;
        let held = |k: Key| input.held.contains(&k) || input.pad_held.contains(&k);
        let thr = throttle_lever(&an);
        let (flaps, spoil, trim) = (an.get(A::Flaps), an.get(A::Spoilers), an.get(A::Trim));
        if self.bot.is_some() {
            // Track the levers so handing back doesn't jump.
            for (a, v) in [(A::Throttle, thr), (A::Flaps, flaps), (A::Spoilers, spoil), (A::Trim, trim)] {
                if let Some(v) = v {
                    self.pilot.lever[a.idx()] = Some(v);
                }
            }
            return;
        }
        let def = self.defs[self.sel].clone();
        // Rotorcraft on a gamepad (both sticks): "mode 2" (src/rotor/defs.rs).
        if def.rotor.is_some() && (an.get(A::LookX).is_some() || an.get(A::LookY).is_some()) && !self.ap.engaged {
            crate::rotor::defs::gamepad_mode2(&def, &mut self.ctl, an.get(A::Pitch), an.get(A::Roll), an.get(A::LookY), an.get(A::LookX), dt);
            return;
        }
        if !self.ap.engaged {
            if let Some(v) = an.get(A::Pitch).filter(|_| !held(Key::PitchUp) && !held(Key::PitchDown)) {
                self.ctl.elevator = v;
            }
            if let Some(v) = an.get(A::Roll).filter(|_| !held(Key::RollLeft) && !held(Key::RollRight)) {
                self.ctl.aileron = v;
            }
            if let Some(v) = an.get(A::Yaw).filter(|_| !held(Key::YawLeft) && !held(Key::YawRight)) {
                self.ctl.rudder = v;
            }
            if let Some(v) = self.pilot.latch(A::Trim, trim) {
                self.ctl.trim = v;
            }
        }
        if let Some(v) = self.pilot.latch(A::Throttle, thr)
            && def.has_engine
            && !(self.ap.engaged && self.ap.ias.is_some())
        {
            self.ctl.throttle = v;
            // Two engine levers on the quadrant set apart: split throttles (Track M).
            if let (Some(a), Some(b)) = (an.get(A::Throttle1), an.get(A::Throttle2))
                && crate::missions::throttles::can_split(&def)
            {
                if (a - b).abs() > 0.03 {
                    crate::missions::throttles::set_sides(&def, &mut self.ctl, a, b);
                } else {
                    self.ctl.engine_throttle = None;
                }
            }
        }
        if let Some(v) = self.pilot.latch(A::Flaps, flaps) {
            let det = def.flap_detents;
            let full = det.last().copied().unwrap_or(1.0).max(1e-3);
            self.flap_idx = (0..det.len()).min_by(|&a, &b| (det[a] / full - v).abs().total_cmp(&(det[b] / full - v).abs())).unwrap_or(0);
        }
        if let Some(v) = self.pilot.latch(A::Spoilers, spoil) {
            self.spoiler_idx = (0..SPOILER_STEPS.len()).min_by(|&a, &b| (SPOILER_STEPS[a] - v).abs().total_cmp(&(SPOILER_STEPS[b] - v).abs())).unwrap_or(0);
        }
    }

    /// Head-look (cockpit) and camera orbit (outside views) from the mouse,
    /// the look axes and the glance keys / hat.
    pub(super) fn update_look(&mut self, input: &mut Input, dt: f32) {
        let m = &input.mouse;
        let held = |k: Key| input.held.contains(&k) || input.pad_held.contains(&k);
        // (Rotorcraft fly with the right stick: see apply_analog.)
        let flies = self.defs[self.sel].rotor.is_some();
        let (lx, ly) = if flies { (0.0, 0.0) } else { (input.analog.get(A::LookX).unwrap_or(0.0), input.analog.get(A::LookY).unwrap_or(0.0)) };
        let d = input.orbit_delta;
        if self.cam == CamMode::Cockpit {
            let inv = if m.invert_look { -1.0 } else { 1.0 };
            self.pilot.free += Vec2::new(d.x, -d.y * inv) * 0.0035 * m.look_sens;
            if m.look_return && !input.dragging {
                self.pilot.free *= (-6.0 * dt).exp();
            }
            self.pilot.free = Vec2::new(self.pilot.free.x.clamp(-2.8, 2.8), self.pilot.free.y.clamp(-1.0, 1.3));
        } else {
            self.orbit_yaw += d.x * 0.005 + lx * 2.2 * dt;
            self.orbit_pitch = (self.orbit_pitch + d.y * 0.005 - ly * 1.2 * dt).clamp(-0.3, 1.45);
        }
        input.orbit_delta = Vec2::ZERO;
        // Glance (hat / keys) while held, else the mouse position plus the look axes.
        let g = |a: Key, b: Key| (held(b) as i32 - held(a) as i32) as f32;
        let (gx, gy) = (g(Key::LookLeft, Key::LookRight), g(Key::LookDown, Key::LookUp));
        let target = if gx != 0.0 || gy != 0.0 {
            Vec2::new(gx * if gy != 0.0 { 0.8 } else { 1.57 }, gy * 0.6)
        } else if self.cam == CamMode::Cockpit {
            self.pilot.free + Vec2::new(lx * 2.4, ly * 1.1)
        } else {
            self.pilot.free
        };
        self.pilot.look += (target - self.pilot.look) * (1.0 - (-12.0 * dt).exp());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn takeover_ignores_noise_and_rest() {
        let mut p = Pilot::default();
        let mut an = Analog::default();
        an.set(A::Pitch, 0.05);
        an.set(A::Throttle, 0.4);
        assert_eq!(p.takeover(&an, |_| true), Takeover::default());
        an.set(A::Pitch, 0.12); // jitter / light touch
        an.set(A::Throttle, 0.43);
        assert_eq!(p.takeover(&an, |_| true), Takeover::default());
        an.set(A::Throttle, 0.55);
        assert_eq!(p.takeover(&an, |_| true), Takeover { throttle: true, ..Default::default() });
        // A stick held off-centre when the AP engaged: only moving it further counts.
        let mut p = Pilot::default();
        an.set(A::Pitch, 0.5);
        an.set(A::Throttle, 0.55);
        assert!(!p.takeover(&an, |_| true).stick);
        an.set(A::Pitch, 0.3);
        assert!(!p.takeover(&an, |_| true).stick);
        an.set(A::Pitch, -0.1);
        assert!(p.takeover(&an, |_| true).stick);
        // Not automated: never a takeover.
        an.set(A::Pitch, 1.0);
        assert_eq!(Pilot::default().takeover(&an, |_| false), Takeover::default());
    }

    #[test]
    fn levers_apply_on_movement_only() {
        let mut p = Pilot::default();
        assert_eq!(p.latch(A::Throttle, Some(0.0)), None, "first sight is recorded, not applied");
        assert_eq!(p.latch(A::Throttle, Some(0.005)), None, "noise");
        assert_eq!(p.latch(A::Throttle, Some(0.3)), Some(0.3));
        assert_eq!(p.latch(A::Throttle, Some(0.3)), None);
        assert_eq!(p.latch(A::Throttle, None), None);
    }

    #[test]
    fn per_engine_levers_average() {
        let mut an = Analog::default();
        an.set(A::Throttle1, 0.2);
        an.set(A::Throttle2, 0.6);
        assert!((throttle_lever(&an).unwrap() - 0.4).abs() < 1e-6);
        an.set(A::Throttle, 0.9);
        assert_eq!(throttle_lever(&an), Some(0.9));
    }

    #[test]
    fn look_directions() {
        use glam::Vec3;
        let mut p = Pilot { look: Vec2::new(std::f32::consts::FRAC_PI_2, 0.0), ..Default::default() };
        // Aircraft frame: +Z forward, +Y up, so right is −X.
        assert!((p.look_rot() * Vec3::Z - Vec3::NEG_X).length() < 1e-5);
        p.look = Vec2::new(0.0, 0.5);
        assert!((p.look_rot() * Vec3::Z).y > 0.4);
    }
}
