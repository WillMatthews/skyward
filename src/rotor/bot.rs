//! Auto flight for rotorcraft, flying the real model through the pilot's
//! controls (with the attitude-hold / angle assist, as a pilot would):
//!
//! * helicopter — run-up on the ground, lift to a 3 m hover, hover-taxi,
//!   transition (nose down, translational lift, climb at 60–70 kt), a
//!   sightseeing tour at ~1,000 ft above the ground and tall buildings,
//!   return, a decelerating approach on a ~6° slope to a hover
//!   over the spot, vertical landing, rotors running down to flight idle;
//!   repeat;
//! * quadcopter — take off, climb, a scenic orbit around the nearest
//!   landmark (or the take-off point) facing the centre, return and land.
//!
//! Also the rotorcraft autopilot behind Z (hold heading, altitude, speed).

use super::Assist;
use crate::autopilot::Autopilot;
use crate::bot::{BotStats, Landmark, dir_of, heading_of, plan_tour, wrap_pi};
use crate::flight::{Aircraft, AircraftDef, Controls, Environment, GRAVITY, KT};
use crate::scene::Runway;
use crate::world::World;
use glam::{Vec2, vec2};

fn xz(v: glam::Vec3) -> Vec2 {
    vec2(v.x, v.z)
}

/// Stick inputs that make the attitude-hold assist fly `pitch_t` / `roll_t`
/// (rad) — whichever regime (hover attitude command / forward-flight rate
/// command) it is in.
pub fn att_stick(ac: &Aircraft, pitch_t: f32, roll_t: f32) -> (f32, f32) {
    let s = &ac.rotor.heli.sas;
    let (pitch, roll) = (ac.pitch(), ac.roll());
    if s.hover {
        let lon = (pitch_t - s.pitch_bias) / 0.30;
        let lat = (roll_t - s.roll_bias) / 0.35;
        // Keep out of the centred dead band (that would be velocity hold).
        let nz = |x: f32| if x.abs() < 0.05 { 0.05f32.copysign(if x == 0.0 { 1.0 } else { x }) } else { x };
        (nz(lon).clamp(-1.0, 1.0), nz(lat).clamp(-1.0, 1.0))
    } else {
        (((pitch_t - pitch) * 2.5 / 0.5).clamp(-1.0, 1.0), ((roll_t - roll) * 3.0 / 0.9).clamp(-1.0, 1.0))
    }
}

/// Pitch attitude (rad) that holds `v` m/s forward in level flight (EC130 trim).
fn heli_trim_pitch(v: f32) -> f32 {
    0.08 - 0.0018 * v.clamp(0.0, 80.0)
}

/// Collective law: hold a climb rate (feed-forward learnt by the integrator).
#[derive(Clone, Copy, Debug, Default)]
struct Vs {
    i: f32,
    init: bool,
}

impl Vs {
    fn coll(&mut self, cur: f32, vs: f32, target: f32, dt: f32) -> f32 {
        if !self.init {
            self.i = cur;
            self.init = true;
        }
        let e = target - vs;
        self.i = (self.i + e * dt * 0.06).clamp(0.0, 1.0);
        (self.i + 0.05 * e).clamp(0.0, 1.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Ground,
    Lift,
    Takeoff,
    Enroute,
    Approach,
    Hover,
    Land,
    Landed,
    // Quadcopter.
    Climb,
    Orbit,
    Return,
}

pub struct RotorBot {
    quad: bool,
    phase: Phase,
    t_phase: f32,
    /// Landing spot (the take-off point) and its ground height.
    home: Vec2,
    home_y: f32,
    tour: Vec<Landmark>,
    wp: usize,
    vs: Vs,
    hdg_t: f32,
    /// Approach: start point and direction (into wind).
    app_dir: Vec2,
    /// Height the approach starts from (it never climbs to meet the slope).
    cruise_y: f32,
    /// Quad orbit centre / radius / angle.
    orbit_c: Vec2,
    orbit_r: f32,
    orbit_a: f32,
    pub wind: Vec2,
    /// Tours flown (each one another loop).
    laps: u32,
    /// Smoothed safe altitude en route (m).
    alt_t: f32,
}

/// The helicopter's sightseeing tour: in the mountains a loop through the
/// valleys (low routes between the lowest ground in four sectors — it never
/// tries to out-climb a ridge), elsewhere the landmarks.
fn heli_tour(world: &World, home: Vec2, lap: u32) -> Vec<Landmark> {
    use crate::nav::procedures as pr;
    let g = pr::Grid::build(|q| world.ground_height_known(q), home, 14_000.0, 150.0);
    if pr::relief(&g, home, 8000.0) < 500.0 {
        return plan_tour(world, home, 1500.0, 9000.0, 4);
    }
    let mut k = 0;
    pr::valley_tour(&g, home, 4, 3000.0, 10_000.0, lap)
        .into_iter()
        .map(|(pos, main)| {
            if main {
                k += 1;
            }
            Landmark { name: if main { format!("Valley {k}") } else { "along the valley".into() }, pos }
        })
        .collect()
}

impl RotorBot {
    pub fn new(def: &AircraftDef, ac: &Aircraft, world: &World, _home: Option<&Runway>) -> RotorBot {
        let quad = def.rotor.as_ref().is_some_and(|r| r.quad().is_some());
        let p = xz(ac.pos);
        let gh = world.ground_height(p);
        let tour = if quad { plan_tour(world, p, 0.0, 700.0, 1) } else { heli_tour(world, p, 0) };
        let phase = if ac.on_ground {
            Phase::Ground
        } else if quad {
            Phase::Climb
        } else {
            Phase::Enroute
        };
        // The drone orbits a landmark close by, else the tallest building
        // within ~500 m, else its take-off point.
        let mut tour = tour;
        if quad {
            tour.retain(|l| l.pos.distance(p) < 800.0);
            if tour.is_empty() {
                let mut best: Option<(f32, Vec2)> = None;
                for i in -12..=12 {
                    for j in -12..=12 {
                        let q = p + vec2(i as f32, j as f32) * 40.0;
                        if q.distance(p) > 500.0 {
                            continue;
                        }
                        if let Some(top) = world.obstacle_top(q) {
                            let h = top - world.ground_height(q);
                            if h > 15.0 && best.is_none_or(|b| h > b.0) {
                                best = Some((h, q));
                            }
                        }
                    }
                }
                if let Some((_, q)) = best {
                    tour.push(Landmark { name: "the tallest building nearby".into(), pos: q });
                }
            }
        }
        let orbit_c = tour.first().map(|l| l.pos).unwrap_or(p);
        RotorBot {
            quad,
            phase,
            t_phase: 0.0,
            home: p,
            home_y: gh,
            tour,
            wp: 0,
            vs: Vs::default(),
            hdg_t: ac.heading(),
            app_dir: vec2(0.0, -1.0),
            cruise_y: 300.0,
            orbit_c,
            orbit_r: 60.0,
            orbit_a: 0.0,
            wind: Vec2::ZERO,
            laps: 0,
            alt_t: 0.0,
        }
    }

    pub fn phase_name(&self) -> String {
        format!("{:?}", self.phase)
    }

    pub fn waypoint(&self) -> Option<(String, Vec2)> {
        match self.phase {
            Phase::Enroute => self.tour.get(self.wp).map(|l| (l.name.clone(), l.pos)).or(Some(("Home".into(), self.home))),
            Phase::Orbit => self.tour.first().map(|l| (l.name.clone(), l.pos)),
            _ => None,
        }
    }

    pub fn route(&self) -> Vec<(String, Vec2)> {
        match self.phase {
            Phase::Enroute => {
                let mut v: Vec<(String, Vec2)> = self.tour.iter().skip(self.wp).map(|l| (l.name.clone(), l.pos)).collect();
                v.push(("Home".into(), self.home));
                v
            }
            _ => Vec::new(),
        }
    }

    pub fn rebase(&mut self, r: &crate::world::Rebase) {
        self.home = r.ground(self.home);
        self.orbit_c = r.ground(self.orbit_c);
        for l in &mut self.tour {
            l.pos = r.ground(l.pos);
        }
    }

    fn set(&mut self, p: Phase, ev: &mut Vec<String>) {
        if p != self.phase {
            ev.push(format!("{:?} → {:?}", self.phase, p));
            self.phase = p;
            self.t_phase = 0.0;
        }
    }

    /// Height to fly at over `p`: above the terrain and the buildings ahead.
    fn safe_alt(world: &World, p: Vec2, dir: Vec2, agl: f32) -> f32 {
        let mut top = world.ground_height(p) + agl;
        // 2.4 km ahead (rising valleys: start the climb early), less needed
        // the further it is (a ~9 % climb gradient is easy at 110 kt).
        for k in 0..20 {
            let d = k as f32 * 120.0;
            let q = p + dir * d;
            let g = world.ground_height(q);
            top = top.max(g + agl * 0.6 - (d - 960.0).max(0.0) * 0.09);
            if let Some(b) = world.obstacle_top(q) {
                top = top.max(b + 90.0);
            }
        }
        top
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        ap: &mut Autopilot,
        world: &World,
        stats: &mut BotStats,
        ev: &mut Vec<String>,
        status: &mut String,
        dt: f32,
    ) {
        ap.disengage();
        self.t_phase += dt;
        if self.quad {
            self.update_quad(def, ac, ctl, world, stats, ev, status, dt);
        } else {
            self.update_heli(def, ac, ctl, world, stats, ev, status, dt);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn update_heli(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, world: &World, stats: &mut BotStats, ev: &mut Vec<String>, status: &mut String, dt: f32) {
        let p = xz(ac.pos);
        let gh = world.ground_height(p);
        let skid_h = ac.pos.y - def.gear_height - gh;
        let vel = xz(ac.vel);
        let hdg = ac.heading();
        let fwd = dir_of(hdg);
        let right = vec2(-fwd.y, fwd.x);
        let (gs_f, gs_r) = (vel.dot(fwd), vel.dot(right));
        let vs = ac.vel.y;
        let into_wind = if self.wind.length() > 1.5 { -self.wind.normalize() } else { fwd };
        let pitch_t;
        let mut roll_t = 0.0;
        let mut pedal = 0.0;
        let mut vs_t;
        let hover_pitch = ac.rotor.heli.sas.pitch_bias;
        let hover_roll = ac.rotor.heli.sas.roll_bias;
        match self.phase {
            Phase::Ground => {
                *status = "Run-up".into();
                ctl.throttle = 0.0;
                ctl.elevator = 0.0;
                ctl.aileron = 0.0;
                ctl.rudder = 0.0;
                self.vs = Vs::default();
                self.hdg_t = heading_of(into_wind);
                if self.t_phase > 3.0 && ac.rotor.out.nr_pct > 97.0 {
                    self.home = p;
                    self.home_y = gh;
                    self.set(Phase::Lift, ev);
                }
                return;
            }
            Phase::Lift => {
                *status = "Lift to the hover".into();
                // Smooth collective until light on the skids, then climb to 3 m.
                vs_t = if skid_h < 2.5 { 0.6 } else { (3.0 - skid_h) * 0.8 };
                pitch_t = hover_pitch + 0.04 * gs_f;
                roll_t = hover_roll - 0.04 * gs_r;
                // Turn into wind in the hover.
                pedal = (1.2 * wrap_pi(self.hdg_t - hdg)).clamp(-0.6, 0.6);
                if ac.on_ground {
                    ctl.throttle = (ctl.throttle + dt * 0.18).min(0.7);
                    self.vs = Vs { i: ctl.throttle, init: true };
                    ctl.elevator = 0.0;
                    ctl.aileron = 0.0;
                    ctl.rudder = 0.0;
                    return;
                }
                if self.t_phase > 6.0 && (skid_h - 3.0).abs() < 1.0 && wrap_pi(self.hdg_t - hdg).abs() < 0.15 {
                    stats.takeoffs += 1;
                    self.set(Phase::Takeoff, ev);
                }
            }
            Phase::Takeoff => {
                *status = "Take-off: translational lift, climb".into();
                let v = ac.airspeed();
                // Accelerate in ground effect first, then climb at ~65 kt.
                let v_t = 65.0 * KT;
                pitch_t = (heli_trim_pitch(v) - 0.10).max(heli_trim_pitch(v) + 0.03 * (v - v_t));
                vs_t = if v < 25.0 * KT { 0.5 } else { 5.0 };
                let tgt = self.tour.get(self.wp).map(|l| l.pos).unwrap_or(self.home + into_wind * 3000.0);
                if skid_h > 60.0 {
                    let h = heading_of(tgt - p);
                    roll_t = (1.2 * wrap_pi(h - hdg)).clamp(-0.35, 0.35);
                }
                if skid_h > 150.0 && v > 55.0 * KT {
                    self.set(Phase::Enroute, ev);
                }
            }
            Phase::Enroute => {
                let (name, tgt) = match self.tour.get(self.wp) {
                    Some(l) => (l.name.clone(), l.pos),
                    None => ("home".into(), self.home),
                };
                let to = tgt - p;
                let d = to.length();
                *status = format!("Tour: {name} ({:.1} km)", d / 1000.0);
                let v_t = 110.0 * KT;
                let v = ac.airspeed();
                pitch_t = heli_trim_pitch(v_t) + 0.010 * (v - v_t);
                let h = heading_of(to);
                roll_t = (1.0 * wrap_pi(h - hdg)).clamp(-0.45, 0.45);
                // (The safe height falls no faster than 3 m/s — turning away from a
                // hillside drops it suddenly, and a collective dump at speed is a
                // negative-g push.)
                let alt = Self::safe_alt(world, p, fwd, 300.0).max(self.alt_t - 3.0 * dt);
                self.alt_t = alt;
                vs_t = (0.2 * (alt - ac.pos.y)).clamp(-3.5, 6.0);
                if d < 400.0 {
                    if self.wp < self.tour.len() {
                        ev.push(format!("passed {name}"));
                        self.wp += 1;
                    }
                }
                // Home: the approach begins ~1.8 km out, on the direct track.
                let to_home = self.home - p;
                if self.wp >= self.tour.len() && to_home.length() < 2400.0 {
                    self.app_dir = to_home.normalize_or(fwd);
                    self.cruise_y = ac.pos.y;
                    self.set(Phase::Approach, ev);
                }
            }
            Phase::Approach => {
                // Decelerating approach on a fixed final track and a ~6° slope
                // to a 3 m hover over the spot.
                let dir = self.app_dir;
                let rel = p - self.home;
                let d = (-rel.dot(dir)).max(0.0);
                let cross = rel.dot(vec2(-dir.y, dir.x));
                *status = format!("Approach: {:.0} m, {:.0} kt", d, gs_f / KT);
                // Ground speed that stops at the spot (0.45 m/s² deceleration).
                let v_t = (2.0 * 0.45 * (d - 6.0).max(0.0)).sqrt().min(70.0 * KT);
                let v = vel.dot(dir);
                pitch_t = (heli_trim_pitch(v.max(0.0)) + 0.02 * (v - v_t)).clamp(-0.2, 0.25);
                let course = heading_of(dir) - (cross * 0.01).clamp(-0.5, 0.5);
                if v > 12.0 {
                    roll_t = (1.0 * wrap_pi(course - hdg)).clamp(-0.35, 0.35);
                } else {
                    roll_t = hover_roll - 0.05 * vel.dot(vec2(-dir.y, dir.x)) - (cross * 0.01).clamp(-0.08, 0.08);
                    pedal = (1.0 * wrap_pi(heading_of(dir) - hdg)).clamp(-0.4, 0.4);
                }
                let slope = self.home_y + def.gear_height + 3.0 + (d - 10.0).max(0.0) * 0.105;
                let h_t = slope.min(self.cruise_y);
                vs_t = (0.3 * (h_t - ac.pos.y)).clamp(-5.0, 3.0);
                if d < 30.0 && vel.length() < 6.0 {
                    self.hdg_t = hdg;
                    self.set(Phase::Hover, ev);
                }
            }
            Phase::Hover => {
                *status = "Hover over the spot".into();
                let to = self.home - p;
                let v_t = (to * 0.2).clamp_length_max(4.0);
                pitch_t = hover_pitch + 0.05 * (vel.dot(fwd) - v_t.dot(fwd));
                roll_t = hover_roll - 0.05 * (vel.dot(right) - v_t.dot(right));
                pedal = (1.2 * wrap_pi(self.hdg_t - hdg)).clamp(-0.5, 0.5);
                vs_t = (0.6 * (3.0 - skid_h)).clamp(-1.0, 1.0);
                if self.t_phase > 4.0 && to.length() < 3.0 && vel.length() < 0.8 {
                    self.set(Phase::Land, ev);
                }
            }
            Phase::Land => {
                *status = "Landing".into();
                let to = self.home - p;
                pitch_t = hover_pitch + 0.05 * (vel.dot(fwd) - 0.15 * to.dot(fwd));
                roll_t = hover_roll - 0.05 * (vel.dot(right) - 0.15 * to.dot(right));
                pedal = (1.2 * wrap_pi(self.hdg_t - hdg)).clamp(-0.5, 0.5);
                vs_t = if skid_h > 1.0 { -0.7 } else { -0.35 };
                if ac.on_ground {
                    ctl.throttle = (ctl.throttle - dt * 0.25).max(0.0);
                    ctl.elevator = 0.0;
                    ctl.aileron = 0.0;
                    ctl.rudder = 0.0;
                    if ctl.throttle <= 0.0 {
                        stats.landings += 1;
                        stats.cycles += 1;
                        self.set(Phase::Landed, ev);
                    }
                    return;
                }
            }
            Phase::Landed => {
                *status = "Landed — next tour shortly".into();
                ctl.throttle = 0.0;
                ctl.elevator = 0.0;
                ctl.aileron = 0.0;
                ctl.rudder = 0.0;
                if self.t_phase > 10.0 {
                    self.laps += 1;
                    self.tour = heli_tour(world, p, self.laps);
                    self.wp = 0;
                    self.set(Phase::Ground, ev);
                }
                return;
            }
            _ => {
                self.phase = Phase::Enroute;
                return;
            }
        }
        // Keep the vortex ring away: limit the descent rate at low speed.
        if ac.airspeed() < 20.0 * KT {
            vs_t = vs_t.max(-2.0);
        }
        let (lon, lat) = att_stick(ac, pitch_t.clamp(-0.35, 0.3), roll_t.clamp(-0.6, 0.6));
        ctl.elevator = lon;
        ctl.aileron = lat;
        ctl.rudder = pedal;
        ctl.throttle = self.vs.coll(ctl.throttle, vs, vs_t, dt);
        let _ = GRAVITY;
    }

    #[allow(clippy::too_many_arguments)]
    fn update_quad(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, world: &World, stats: &mut BotStats, ev: &mut Vec<String>, status: &mut String, dt: f32) {
        let q = def.rotor.as_ref().and_then(|r| r.quad()).expect("quad");
        let p = xz(ac.pos);
        let gh = world.ground_height(p);
        let agl = ac.pos.y - def.gear_height - gh;
        let vel = xz(ac.vel);
        let hdg = ac.heading();
        let fwd = dir_of(hdg);
        let right = vec2(-fwd.y, fwd.x);
        let hover = super::quad::hover_throttle(q, def);
        // Desired ground velocity and height.
        let (v_t, h_t, look): (Vec2, f32, f32);
        match self.phase {
            Phase::Ground => {
                *status = "Armed".into();
                ctl.throttle = 0.0;
                ctl.elevator = 0.0;
                ctl.aileron = 0.0;
                ctl.rudder = 0.0;
                if self.t_phase > 2.0 {
                    self.home = p;
                    self.home_y = gh;
                    self.vs = Vs { i: 0.0, init: true };
                    stats.takeoffs += 1;
                    self.set(Phase::Climb, ev);
                }
                return;
            }
            Phase::Climb => {
                *status = "Climbing".into();
                v_t = Vec2::ZERO;
                h_t = self.home_y + 35.0;
                look = hdg;
                if ac.pos.y > h_t - 3.0 {
                    self.orbit_a = 0.0;
                    self.orbit_r = if self.tour.is_empty() { 45.0 } else { 70.0 };
                    self.set(Phase::Orbit, ev);
                }
            }
            Phase::Orbit => {
                let name = self.tour.first().map(|l| l.name.clone()).unwrap_or("the take-off point".into());
                // Fly to the circle, then round it at 9 m/s facing the centre.
                let rel = p - self.orbit_c;
                let r_now = rel.length().max(1.0);
                let radial = rel / r_now;
                let tangent = vec2(-radial.y, radial.x);
                let on_circle = r_now < self.orbit_r * 1.4;
                if on_circle {
                    *status = format!("Orbiting {name}");
                    self.orbit_a += dt;
                    v_t = tangent * 9.0 + radial * ((self.orbit_r - r_now) * 0.4).clamp(-6.0, 6.0);
                    look = heading_of(-radial);
                } else {
                    *status = format!("To {name} ({:.0} m)", r_now - self.orbit_r);
                    v_t = -radial * 16.0;
                    look = heading_of(-radial);
                }
                let top = world.obstacle_top(self.orbit_c).unwrap_or(gh);
                h_t = (self.home_y + 35.0).max(top + 25.0).max(gh + 25.0);
                // Two laps (orbit_a counts seconds on the circle).
                if self.orbit_a > 2.0 * std::f32::consts::TAU * self.orbit_r / 9.0 || self.t_phase > 240.0 {
                    self.set(Phase::Return, ev);
                }
            }
            Phase::Return => {
                let to = self.home - p;
                *status = format!("Returning ({:.0} m)", to.length());
                v_t = to.normalize_or(Vec2::ZERO) * (to.length() * 0.3).min(16.0);
                h_t = (self.home_y + 25.0).max(gh + 20.0);
                look = if to.length() > 8.0 { heading_of(to) } else { hdg };
                if to.length() < 4.0 && vel.length() < 1.5 {
                    self.set(Phase::Land, ev);
                }
            }
            Phase::Land => {
                *status = "Landing".into();
                let to = self.home - p;
                v_t = to * 0.3;
                h_t = gh - 2.0;
                look = hdg;
                if ac.on_ground {
                    ctl.throttle = 0.0;
                    stats.landings += 1;
                    stats.cycles += 1;
                    self.set(Phase::Landed, ev);
                    return;
                }
            }
            _ => {
                *status = "Landed".into();
                ctl.throttle = 0.0;
                ctl.elevator = 0.0;
                ctl.aileron = 0.0;
                if self.t_phase > 8.0 {
                    self.set(Phase::Ground, ev);
                }
                return;
            }
        }
        // Angle mode: stick = tilt / max tilt. Velocity loop → tilt.
        let e = v_t - vel;
        let acc_f = (e.dot(fwd) * 0.8).clamp(-6.0, 6.0);
        let acc_r = (e.dot(right) * 0.8).clamp(-6.0, 6.0);
        let tilt_f = (acc_f / GRAVITY).atan();
        let tilt_r = (acc_r / GRAVITY).atan();
        let angle = ac.rotor.assist == Assist::Att;
        if angle {
            ctl.elevator = (-tilt_f / q.angle_max).clamp(-0.7, 0.7);
            ctl.aileron = (tilt_r / q.angle_max).clamp(-0.7, 0.7);
        } else {
            ctl.elevator = ((-tilt_f - ac.pitch()) * 0.5).clamp(-0.3, 0.3);
            ctl.aileron = ((tilt_r - ac.roll()) * 0.5).clamp(-0.3, 0.3);
        }
        ctl.rudder = (wrap_pi(look - hdg) * 1.2).clamp(-0.5, 0.5);
        let vs_t = if self.phase == Phase::Land { if agl > 3.0 { -1.5 } else { -0.6 } } else { ((h_t - ac.pos.y) * 0.6).clamp(-3.0, 4.0) };
        // Throttle: hover feed-forward (tilt-compensated) + PI on the climb rate.
        let tilt = (1.0 / ac.up().y.max(0.5)).min(1.5);
        let e = vs_t - ac.vel.y;
        self.vs.i = (self.vs.i + e * dt * 0.01).clamp(-0.1, 0.1);
        ctl.throttle = ((hover + self.vs.i) * tilt + 0.03 * e).clamp(0.0, 1.0);
    }
}

/// The rotorcraft autopilot (Z): holds heading (bank to turn), altitude
/// (collective) and speed (pitch); in the hover it holds position and height.
pub fn autopilot(ap: &mut Autopilot, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, dt: f32) -> bool {
    if !ap.engaged || ac.crashed.is_some() || ac.on_ground {
        return false;
    }
    let quad = def.rotor.as_ref().is_some_and(|r| r.quad().is_some());
    let hdg = ac.heading();
    let alt_t = ap.alt.unwrap_or(ac.pos.y);
    let vs_t = ap.vs.unwrap_or(((alt_t - ac.pos.y) * 0.25).clamp(-5.0, 5.0));
    if quad {
        let q = def.rotor.as_ref().and_then(|r| r.quad()).unwrap();
        let v_t = ap.ias.unwrap_or(0.0);
        let fwd = dir_of(hdg);
        let e = v_t - xz(ac.vel).dot(fwd);
        ctl.elevator = (-(e * 0.8 / GRAVITY).atan() / q.angle_max).clamp(-0.6, 0.6);
        let right = vec2(-fwd.y, fwd.x);
        ctl.aileron = ((-xz(ac.vel).dot(right) * 0.8 / GRAVITY).atan() / q.angle_max).clamp(-0.6, 0.6);
        ctl.rudder = (wrap_pi(ap.hdg.unwrap_or(hdg) - hdg) * 1.2).clamp(-0.5, 0.5);
        let hover = super::quad::hover_throttle(q, def);
        ctl.throttle = (hover / ac.up().y.max(0.5) + 0.05 * (vs_t - ac.vel.y)).clamp(0.0, 1.0);
        return true;
    }
    let v = ac.airspeed();
    let v_t = ap.ias.unwrap_or(v).max(0.0);
    let pitch_t = (heli_trim_pitch(v) + 0.025 * (v - v_t)).clamp(-0.25, 0.2);
    let roll_t = if v > 20.0 * KT { (1.0 * wrap_pi(ap.hdg.unwrap_or(hdg) - hdg)).clamp(-0.4, 0.4) } else { ac.rotor.heli.sas.roll_bias };
    let (lon, lat) = att_stick(ac, pitch_t, roll_t);
    ctl.elevator = lon;
    ctl.aileron = lat;
    ctl.rudder = if v > 20.0 * KT { 0.0 } else { (wrap_pi(ap.hdg.unwrap_or(hdg) - hdg) * 1.0).clamp(-0.4, 0.4) };
    let e = vs_t - ac.vel.y;
    ap_coll(ctl, e, dt);
    true
}

fn ap_coll(ctl: &mut Controls, e: f32, dt: f32) {
    ctl.throttle = (ctl.throttle + (0.015 * e) * dt * 4.0).clamp(0.0, 1.0);
}
