//! Pushback: a tug on the nose gear pushes the aircraft back off the stand —
//! straight back, or turning so the tail goes left / right (the aircraft ends
//! up facing 90° to the right / left of the stand heading) — then stops and
//! waits for the crew to set the parking brake before the tow bar is
//! disconnected and the tug drives away.
//!
//! The tug drives the nose wheel's speed along its heading and holds its
//! steering angle (`ground::Tow`: the nose-wheel steering is bypassed while
//! it is connected); the aircraft's own tyres, brakes and inertia do the
//! rest, so a set parking brake stalls the push.

use super::Tow;
use crate::flight::{Aircraft, AircraftDef, ContactKind, Controls, GRAVITY};
use glam::{Vec2, Vec3};

/// Pushback direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PushDir {
    Straight,
    /// Tail swings to the pilot's left: ends facing 90° right of the stand heading.
    TailLeft,
    TailRight,
}

impl PushDir {
    pub fn label(self) -> &'static str {
        match self {
            PushDir::Straight => "straight back",
            PushDir::TailLeft => "tail left",
            PushDir::TailRight => "tail right",
        }
    }
    pub fn next(self) -> PushDir {
        match self {
            PushDir::Straight => PushDir::TailLeft,
            PushDir::TailLeft => PushDir::TailRight,
            PushDir::TailRight => PushDir::Straight,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Phase {
    /// Tug and tow bar being connected (s left).
    Connecting(f32),
    /// Pushing straight back (to clear the stand).
    Back,
    /// Turning (tail left / right).
    Turn,
    /// Straightening the nose wheel for the last few metres.
    Straighten,
    /// Slowing to a stop.
    Stopping,
    /// Stopped: "set parking brake".
    WaitBrake,
    /// Tow bar coming off (s left).
    Disconnecting(f32),
    /// The tug drives away (s since).
    Leaving(f32),
    Done,
}

/// The tug's placement for drawing: ground position (world XZ), heading (rad).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TugPose {
    pub pos: Vec2,
    pub hdg: f32,
}

#[derive(Clone, Debug)]
pub struct Pushback {
    pub dir: PushDir,
    pub phase: Phase,
    /// Where it started (main-gear midpoint) and the stand heading.
    pub start: Vec2,
    pub start_hdg: f32,
    /// Nose-wheel speed (m/s, − = backwards) and steering (rad) the tug holds now.
    pub speed: f32,
    pub steer: f32,
    /// Straight distance before turning / in all (m).
    pub back_len: f32,
    pub tug: TugPose,
    /// Seconds the push has been stalled (parking brake set).
    stalled: f32,
    /// Message for the crew (sequence, text).
    pub msg_seq: u32,
    pub msg: String,
    /// The pilot asked to stop.
    pub stop_req: bool,
    /// Nose-gear position (body) and wheelbase.
    nose: Vec3,
    wheelbase: f32,
}

/// Push speed (m/s): walking pace, as a tug driver does it.
pub const PUSH_SPEED: f32 = 1.3;
/// Nose-wheel angle the tug turns with (rad).
const TURN_STEER: f32 = 0.95;

fn wrap(a: f32) -> f32 {
    let mut d = a.rem_euclid(std::f32::consts::TAU);
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    d
}

/// Nose / tail wheel and main-gear midpoint of a tricycle undercarriage (body frame).
pub fn geometry(def: &AircraftDef) -> Option<(Vec3, Vec3)> {
    let nose = def.gear.iter().find(|g| matches!(g.kind, ContactKind::Wheel { steerable: true, .. }) && g.pos.z > 0.0)?.pos;
    let mains: Vec<Vec3> = def.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { braked: true, .. })).map(|g| g.pos).collect();
    if mains.is_empty() {
        return None;
    }
    let mid = mains.iter().copied().sum::<Vec3>() / mains.len() as f32;
    Some((nose, mid))
}

/// Can this type be pushed back (a tricycle undercarriage with a steerable nose wheel)?
pub fn can_push(def: &AircraftDef) -> bool {
    !def.taildragger && def.rotor.is_none() && !def.on_water() && geometry(def).is_some()
}

impl Pushback {
    /// Push back onto the taxilane behind the stand: `lanes` are taxiway
    /// centreline segments (world XZ); the tug pushes straight back until the
    /// main gear nears the first one crossing the path behind, then turns onto it.
    pub fn onto_lane(def: &AircraftDef, ac: &Aircraft, dir: PushDir, lanes: &[(Vec2, Vec2)]) -> Option<Pushback> {
        let mut pb = Pushback::new(def, ac, dir)?;
        let back = -Vec2::new(ac.heading().sin(), -ac.heading().cos());
        let mut best: Option<f32> = None;
        for (a, b) in lanes {
            let e = *b - *a;
            let den = back.perp_dot(e);
            if den.abs() < 1e-3 {
                continue;
            }
            let w = *a - pb.start;
            let t = w.perp_dot(e) / den;
            let u = w.perp_dot(back) / den;
            if (0.0..=1.0).contains(&u) && t > 12.0 && t < 160.0 && best.is_none_or(|x| t < x) {
                best = Some(t);
            }
        }
        if let Some(t) = best {
            // Straight: mains onto the line; turning: start the turn about a
            // turn radius short of it so the arc ends on the line.
            let r = pb.wheelbase / TURN_STEER.tan();
            pb.back_len = if dir == PushDir::Straight { t } else { (t - 0.8 * r).max(8.0) };
        }
        Some(pb)
    }

    pub fn new(def: &AircraftDef, ac: &Aircraft, dir: PushDir) -> Option<Pushback> {
        let (nose, mid) = geometry(def)?;
        let p = ac.pos + ac.rot * mid;
        let wheelbase = (nose.z - mid.z).max(1.0);
        let mut pb = Pushback {
            dir,
            phase: Phase::Connecting(6.0),
            start: Vec2::new(p.x, p.z),
            start_hdg: ac.heading(),
            speed: 0.0,
            steer: 0.0,
            // Clear the stand: about half the fuselage back before turning, a
            // fuselage length straight back.
            back_len: if dir == PushDir::Straight { (wheelbase * 2.4).max(30.0) } else { (wheelbase * 0.9 + def.wingspan * 0.2).max(12.0) },
            tug: TugPose { pos: Vec2::ZERO, hdg: 0.0 },
            stalled: 0.0,
            msg_seq: 0,
            msg: String::new(),
            stop_req: false,
            nose,
            wheelbase,
        };
        pb.say(format!("Ground: tug connecting — pushback {}, release parking brake when ready", dir.label()));
        pb.place_tug(ac, 0.0);
        Some(pb)
    }

    fn say(&mut self, s: impl Into<String>) {
        self.msg = s.into();
        self.msg_seq += 1;
    }

    /// The tug is attached (pushing or about to).
    pub fn attached(&self) -> bool {
        !matches!(self.phase, Phase::Leaving(_) | Phase::Done)
    }

    /// Ask the tug to stop where it is.
    pub fn stop(&mut self) {
        if matches!(self.phase, Phase::Back | Phase::Turn | Phase::Straighten | Phase::Connecting(_)) {
            self.stop_req = true;
        }
    }

    /// Distance pushed so far (main-gear midpoint from the start, m).
    pub fn pushed(&self, def: &AircraftDef, ac: &Aircraft) -> f32 {
        let mid = geometry(def).map(|g| g.1).unwrap_or(Vec3::ZERO);
        let p = ac.pos + ac.rot * mid;
        Vec2::new(p.x, p.z).distance(self.start)
    }

    /// Heading change so far (rad).
    pub fn turned(&self, ac: &Aircraft) -> f32 {
        wrap(ac.heading() - self.start_hdg)
    }

    /// One step: returns the tow on the nose gear (None once disconnected).
    pub fn update(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &Controls, dt: f32) -> Option<Tow> {
        let w = def.mass * GRAVITY;
        // Nose-wheel speed along its heading (+ = forward).
        let nose_v = {
            let v = ac.vel + ac.rot * ac.omega.cross(self.nose);
            let f = ac.rot * Vec3::new(-self.steer.sin(), 0.0, self.steer.cos());
            Vec2::new(v.x, v.z).dot(Vec2::new(f.x, f.z).normalize_or_zero())
        };
        let accel = 0.25 * dt;
        let mut target_v = 0.0;
        match self.phase {
            Phase::Connecting(t) => {
                self.steer = 0.0;
                if t - dt <= 0.0 {
                    self.phase = if self.stop_req { Phase::Stopping } else { Phase::Back };
                    self.say("Ground: pushing back");
                } else {
                    self.phase = Phase::Connecting(t - dt);
                }
            }
            Phase::Back => {
                target_v = -PUSH_SPEED;
                self.steer = 0.0;
                if self.pushed(def, ac) >= self.back_len {
                    self.phase = if self.dir == PushDir::Straight { Phase::Stopping } else { Phase::Turn };
                }
            }
            Phase::Turn => {
                target_v = -PUSH_SPEED;
                // Heading goes up for "tail left": backing with the nose wheel turned left.
                let want = if self.dir == PushDir::TailLeft { 1.0 } else { -1.0 };
                let done = self.turned(ac) * want;
                // Lead the straightening (the nose wheel takes time to come back).
                if done > 90f32.to_radians() - 0.13 {
                    self.phase = Phase::Straighten;
                } else {
                    let s = -want * TURN_STEER;
                    self.steer += (s - self.steer).clamp(-0.5 * dt, 0.5 * dt);
                }
            }
            Phase::Straighten => {
                target_v = -PUSH_SPEED * 0.7;
                self.steer += (0.0 - self.steer).clamp(-0.5 * dt, 0.5 * dt);
                if self.steer.abs() < 0.01 {
                    self.phase = Phase::Stopping;
                }
            }
            Phase::Stopping => {
                self.steer += (0.0 - self.steer).clamp(-0.5 * dt, 0.5 * dt);
                if nose_v.abs() < 0.05 && self.speed.abs() < 0.05 {
                    self.phase = Phase::WaitBrake;
                    self.say("Ground: pushback complete — set parking brake");
                }
            }
            Phase::WaitBrake => {
                if ctl.parking_brake {
                    self.phase = Phase::Disconnecting(8.0);
                    self.say("Ground: parking brake set — disconnecting, bypass pin removed");
                }
            }
            Phase::Disconnecting(t) => {
                if t - dt <= 0.0 {
                    self.phase = Phase::Leaving(0.0);
                    self.say("Ground: tug clear — hand signals on your left, good day");
                } else {
                    self.phase = Phase::Disconnecting(t - dt);
                }
            }
            Phase::Leaving(t) => {
                // The tug backs off, turns and drives away (for drawing only).
                let back = if t < 4.0 { 1.2 } else { 0.0 };
                let fwd = Vec2::new(self.tug.hdg.sin(), -self.tug.hdg.cos());
                if t >= 4.0 {
                    self.tug.hdg += 0.35 * dt * if t < 8.0 { 1.0 } else { 0.0 };
                    self.tug.pos += fwd * 3.5 * dt;
                } else {
                    self.tug.pos -= fwd * back * dt;
                }
                self.phase = if t > 25.0 { Phase::Done } else { Phase::Leaving(t + dt) };
                return None;
            }
            Phase::Done => return None,
        }
        if self.stop_req && matches!(self.phase, Phase::Back | Phase::Turn | Phase::Straighten) {
            self.phase = Phase::Stopping;
            self.say("Ground: stopping the push");
        }
        self.speed += (target_v - self.speed).clamp(-accel, accel);
        // Brakes on (parking or pedals) stall the tug.
        let braked = ctl.parking_brake || ctl.brakes > 0.3 || ctl.brake_left.max(ctl.brake_right) > 0.3;
        if braked && self.speed.abs() > 0.2 && nose_v.abs() < 0.1 && matches!(self.phase, Phase::Back | Phase::Turn | Phase::Straighten) {
            self.stalled += dt;
            if self.stalled > 2.5 {
                self.stalled = -10.0;
                self.say("Ground: release the parking brake");
            }
        } else if self.stalled > 0.0 {
            self.stalled = 0.0;
        }
        self.place_tug(ac, self.steer);
        Some(Tow { speed: self.speed, steer: self.steer, max_force: 0.22 * w })
    }

    /// The tug sits in front of the nose wheel, facing the aircraft, tow bar along the wheel.
    fn place_tug(&mut self, ac: &Aircraft, steer: f32) {
        let nose = ac.pos + ac.rot * self.nose;
        let dir = ac.rot * Vec3::new(-steer.sin(), 0.0, steer.cos());
        let d = Vec2::new(dir.x, dir.z).normalize_or(Vec2::Y);
        // The tow bar (to 5.5 m ahead of the tug's centre) on the nose-wheel axle.
        let pos = Vec2::new(nose.x, nose.z) + d * 5.7;
        // Facing back toward the aircraft.
        let hdg = (-d.x).atan2(d.y);
        self.tug = TugPose { pos, hdg };
    }
}
