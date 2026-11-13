//! Leaving the aircraft: the F-16's ACES II ejection seat (canopy
//! jettison, catapult + rocket, drogue, seat–man separation, main canopy,
//! with the high-altitude mode holding the main until 14,000 ft) and the
//! Spitfire bail-out (canopy jettisoned, roll out, free fall, ripcord), then
//! the parachute descent to the ground.

use crate::flight::GRAVITY;
use glam::{Quat, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Seat on the rails / rocket burning (or climbing out of the cockpit).
    Exit,
    /// Seat-stabilised / free fall (drogue on the seat).
    Fall,
    /// Main canopy inflating / open.
    Canopy,
    Landed,
}

/// A jettisoned canopy (tumbling glass).
#[derive(Clone, Copy, Debug)]
pub struct Canopy {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    pub omega: Vec3,
    pub t: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Chute {
    pub owner: u32,
    pub seat: bool,
    pub pos: Vec3,
    pub vel: Vec3,
    /// Body attitude (the pilot / seat), body frame like an aircraft.
    pub rot: Quat,
    pub t: f32,
    pub stage: Stage,
    /// Main canopy inflation 0..1, and the seat (dropped at separation).
    pub open: f32,
    pub seat_attached: bool,
    pub seat_pos: Vec3,
    pub seat_vel: Vec3,
    pub seat_rot: Quat,
    /// Rocket burning (flame).
    pub rocket: f32,
    pub landed_t: f32,
}

/// ACES II: the main deploys at once below 15,000 ft / 250 KEAS; above, the
/// drogue holds until 14,000 ft (mode 3).
const MAIN_ALT: f32 = 4270.0;

impl Chute {
    /// Leave an aircraft at `pos` / `vel` / `rot` (ejection seat or bail-out).
    pub fn new(owner: u32, seat: bool, pos: Vec3, vel: Vec3, rot: Quat, eye: Vec3) -> (Chute, Canopy) {
        let up = rot * Vec3::Y;
        let p = pos + rot * eye;
        let canopy = Canopy { pos: p + up * 0.6, vel: vel + up * 14.0 - (rot * Vec3::Z) * 4.0, rot, omega: Vec3::new(3.0, 0.4, 0.8), t: 0.0 };
        let c = Chute {
            owner,
            seat,
            pos: p,
            vel: if seat { vel } else { vel + up * 2.0 + (rot * Vec3::X) * 2.5 },
            rot,
            t: 0.0,
            stage: Stage::Exit,
            open: 0.0,
            seat_attached: seat,
            seat_pos: p,
            seat_vel: vel,
            seat_rot: rot,
            rocket: 0.0,
            landed_t: 0.0,
        };
        (c, canopy)
    }

    /// Drag area (m²): man + seat tumbling, drogue, or the main canopy.
    fn cda(&self) -> f32 {
        match self.stage {
            Stage::Exit => 0.7,
            Stage::Fall => {
                if self.seat_attached {
                    1.6
                } else {
                    0.75
                }
            }
            Stage::Canopy => 0.8 + 38.0 * self.open * self.open,
            Stage::Landed => 0.0,
        }
    }
    fn mass(&self) -> f32 {
        if self.seat_attached { 165.0 } else { 100.0 }
    }

    pub fn step(&mut self, ground: f32, wind: Vec3, dt: f32) {
        self.t += dt;
        if self.stage == Stage::Landed {
            self.landed_t += dt;
            return;
        }
        let up = self.rot * Vec3::Y;
        let mut thrust = Vec3::ZERO;
        self.rocket = 0.0;
        match self.stage {
            Stage::Exit => {
                if self.seat {
                    // Catapult (0..0.15 s, ~15 g) then the STAPAC rocket (0.15..0.45 s, ~12 g).
                    if self.t < 0.15 {
                        thrust = up * 15.0 * GRAVITY;
                    } else if self.t < 0.45 {
                        thrust = up * 12.0 * GRAVITY;
                        self.rocket = 1.0;
                    } else {
                        self.stage = Stage::Fall;
                    }
                } else if self.t > 1.2 {
                    // Rolled out over the side.
                    self.stage = Stage::Fall;
                }
            }
            Stage::Fall => {
                let alt_ok = self.pos.y < MAIN_ALT;
                let slow = self.vel.length() < 130.0;
                // Seat: separation + main ~1.2 s after the rocket (low-speed mode);
                // bail-out: the pilot counts to three, then pulls the ripcord.
                let ready = if self.seat { self.t > 1.6 } else { self.t > 3.5 };
                if self.seat_attached && self.t > 1.5 && alt_ok {
                    self.seat_attached = false;
                    self.seat_pos = self.pos;
                    self.seat_vel = self.vel + Vec3::new(0.0, -2.0, 0.0);
                    self.seat_rot = self.rot;
                }
                if ready && alt_ok && slow && !self.seat_attached {
                    self.stage = Stage::Canopy;
                }
            }
            Stage::Canopy => {
                self.open = (self.open + dt / 1.6).min(1.0);
            }
            Stage::Landed => {}
        }
        let rho = crate::flight::isa(self.pos.y).density;
        let va = self.vel - wind;
        let drag = -va * (0.5 * rho * va.length() * self.cda() / self.mass());
        self.vel += (drag + thrust + Vec3::new(0.0, -GRAVITY, 0.0)) * dt;
        self.pos += self.vel * dt;
        // Attitude: tumbles in free fall, hangs under the canopy.
        if self.stage == Stage::Canopy {
            let target = Quat::from_rotation_arc(Vec3::Y, Vec3::Y);
            let yaw = Quat::from_rotation_y(self.t * 0.15);
            self.rot = self.rot.slerp(yaw * target, (dt * 1.5).min(1.0));
        } else if self.stage == Stage::Fall {
            self.rot = (Quat::from_axis_angle(Vec3::new(0.7, 0.2, 0.4).normalize(), dt * 1.8) * self.rot).normalize();
        }
        if !self.seat_attached && self.seat {
            self.seat_vel += (Vec3::new(0.0, -GRAVITY, 0.0) - self.seat_vel * (0.5 * rho * self.seat_vel.length() * 0.5 / 65.0)) * dt;
            self.seat_pos += self.seat_vel * dt;
            self.seat_rot = (Quat::from_axis_angle(Vec3::X, dt * 4.0) * self.seat_rot).normalize();
            if self.seat_pos.y < ground {
                self.seat_pos.y = ground;
                self.seat_vel = Vec3::ZERO;
            }
        }
        if self.pos.y <= ground + 0.9 {
            self.pos.y = ground + 0.9;
            self.vel = Vec3::ZERO;
            self.stage = Stage::Landed;
        }
    }

    pub fn rebase(&mut self, d: Vec3) {
        self.pos += d;
        self.seat_pos += d;
    }
}

impl Canopy {
    pub fn step(&mut self, ground: f32, dt: f32) {
        self.t += dt;
        let rho = crate::flight::isa(self.pos.y).density;
        let s = self.vel.length();
        self.vel += (Vec3::new(0.0, -GRAVITY, 0.0) - self.vel * (0.5 * rho * s * 1.0 / 40.0)) * dt;
        self.pos += self.vel * dt;
        let w = self.omega * dt;
        self.rot = (Quat::from_scaled_axis(w) * self.rot).normalize();
        if self.pos.y < ground {
            self.pos.y = ground;
            self.vel = Vec3::ZERO;
            self.omega = Vec3::ZERO;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aces_ii_low_altitude_and_bail_out_descents() {
        // F-16 at 300 m, 150 m/s level: the seat clears the fin, the main opens
        // within ~2.5 s, descent ≈ 6–7 m/s, lands safely.
        let (mut c, mut can) = Chute::new(0, true, Vec3::new(0.0, 300.0, 0.0), Vec3::new(0.0, 0.0, 150.0), Quat::IDENTITY, Vec3::new(0.0, 1.07, 4.16));
        let dt = 1.0 / 120.0;
        let mut apex: f32 = 0.0;
        let mut t_open = None;
        for i in 0..(120 * 120) {
            c.step(0.0, Vec3::ZERO, dt);
            can.step(0.0, dt);
            apex = apex.max(c.pos.y);
            if c.stage == Stage::Canopy && t_open.is_none() {
                t_open = Some(i as f32 * dt);
            }
            if c.stage == Stage::Landed {
                break;
            }
        }
        let t_open = t_open.expect("main canopy");
        eprintln!("ACES II from 300 m: apex {:.0} m, main at {t_open:.1} s, landed at {:.0} s", apex, c.t);
        assert!(apex > 340.0, "apex {apex}");
        assert!(t_open < 3.0);
        assert_eq!(c.stage, Stage::Landed);
        // Steady descent rate under the canopy.
        let (mut c, _) = Chute::new(0, false, Vec3::new(0.0, 2000.0, 0.0), Vec3::new(0.0, 0.0, 90.0), Quat::IDENTITY, Vec3::ZERO);
        for _ in 0..(120 * 40) {
            c.step(0.0, Vec3::ZERO, dt);
        }
        assert_eq!(c.stage, Stage::Canopy);
        let v = -c.vel.y;
        eprintln!("bail-out descent under canopy: {v:.1} m/s");
        assert!((5.0..8.0).contains(&v), "{v}");
        // High altitude: the drogue holds the main until 14,000 ft.
        let (mut c, _) = Chute::new(0, true, Vec3::new(0.0, 9000.0, 0.0), Vec3::new(0.0, 0.0, 200.0), Quat::IDENTITY, Vec3::ZERO);
        for _ in 0..(120 * 30) {
            c.step(0.0, Vec3::ZERO, dt);
            if c.pos.y > MAIN_ALT + 50.0 {
                assert_ne!(c.stage, Stage::Canopy);
            }
        }
    }
}
