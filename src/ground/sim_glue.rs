//! Ground handling in the simulation (a child module of `sim`, so it can
//! reach `Sim`'s state): the ground keys (parking brake, autobrake, pushback,
//! chocks + ground power, tail-wheel lock, taxi guidance), tiller / toe-brake
//! inputs, the tug and chocks each physics step, the pilot aids (rudder–tiller
//! link at taxi speed, skid release on types without anti-skid), and the
//! ground crew (chocks and ground power once parked with the engines off).

use super::{Input, Key, Sim};
use crate::flight::{Category, KT};
use crate::ground::ops::GroundReq;
use crate::ground::pushback::{self, Phase, PushDir, Pushback};
use crate::ground::{Autobrake, Steer, guide, spec};
use crate::input::AxisAction as A;
use crate::world::World;
use glam::{Vec2, vec2};

impl Sim {
    /// A ground-handling key was pressed. Returns false if it is not one.
    pub(super) fn ground_key(&mut self, world: &World, k: Key) -> bool {
        let def = self.defs[self.sel].clone();
        let sp = spec::spec(&def);
        match k {
            Key::ParkingBrake => {
                self.ctl.parking_brake = !self.ctl.parking_brake;
                self.say(if self.ctl.parking_brake { "Parking brake set" } else { "Parking brake released" }, 2.0);
            }
            Key::Autobrake => {
                let pos = sp.autobrake_positions();
                if pos.len() <= 1 {
                    self.say("No autobrake on this aircraft", 2.0);
                } else {
                    let i = pos.iter().position(|p| *p == self.ctl.autobrake).unwrap_or(0);
                    self.ctl.autobrake = pos[(i + 1) % pos.len()];
                    self.say(format!("Autobrake {}", self.ctl.autobrake.label()), 2.0);
                }
            }
            Key::TailwheelLock => {
                if matches!(sp.steer, Steer::TailCaster { lockable: true }) {
                    self.ctl.tail_lock = !self.ctl.tail_lock;
                    self.say(if self.ctl.tail_lock { "Tail wheel locked" } else { "Tail wheel unlocked (castering)" }, 2.0);
                } else {
                    self.say("No tail-wheel lock on this aircraft", 2.0);
                }
            }
            Key::GroundServices => {
                let gs = Vec2::new(self.ac.vel.x, self.ac.vel.z).length();
                if !self.ac.on_ground || gs > 0.3 {
                    self.say("Ground crew: not while moving", 2.0);
                } else if self.ground.chocks || self.ground.gpu {
                    self.ground.chocks = false;
                    self.ground.gpu = false;
                    self.say("Ground: chocks removed, ground power disconnected", 3.0);
                } else {
                    self.ground.chocks = true;
                    self.ground.gpu = def.has_engine && def.category != Category::Glider;
                    self.say(if self.ground.gpu { "Ground: chocks in, ground power connected" } else { "Ground: chocks in" }, 3.0);
                }
            }
            Key::Pushback => self.pushback_key(&def),
            Key::TaxiGuide => {
                if self.ground.guide.take().is_some() {
                    self.say("Taxi guidance off", 2.0);
                } else {
                    self.start_taxi_guide(world);
                }
            }
            _ => return false,
        }
        true
    }

    /// The ground menu's rows now: (request, label, state).
    pub fn ground_menu(&self) -> Vec<(GroundReq, String, String)> {
        let def = self.def();
        let sp = spec::spec(def);
        let gs = Vec2::new(self.ac.vel.x, self.ac.vel.z).length();
        let parked = self.ac.on_ground && gs < 0.5;
        let mut v = Vec::new();
        let pushing = self.ground.push.as_ref().is_some_and(|p| p.attached() && !matches!(p.phase, Phase::WaitBrake | Phase::Disconnecting(_)));
        if pushing {
            v.push((GroundReq::PushStop, "Stop the push".into(), String::new()));
        } else if parked && pushback::can_push(def) && self.ground.push.is_none() {
            for d in [PushDir::Straight, PushDir::TailLeft, PushDir::TailRight] {
                v.push((GroundReq::Push(d), format!("Push back, {}", d.label()), String::new()));
            }
        }
        if self.ac.on_ground {
            v.push((GroundReq::ParkingBrake, "Parking brake".into(), if self.ctl.parking_brake { "SET".into() } else { "OFF".into() }));
        }
        if parked && def.has_engine {
            let on = self.ground.chocks || self.ground.gpu;
            v.push((GroundReq::Services, "Chocks + ground power".into(), if on { "IN".into() } else { "OUT".into() }));
        }
        if sp.autobrake.is_some() {
            v.push((GroundReq::Autobrake, "Autobrake".into(), self.ctl.autobrake.label().into()));
        }
        if matches!(sp.steer, Steer::TailCaster { lockable: true }) {
            v.push((GroundReq::TailLock, "Tail-wheel lock".into(), if self.ctl.tail_lock { "LOCKED".into() } else { "FREE".into() }));
        }
        if self.ac.on_ground {
            v.push((GroundReq::TaxiGuide, "Taxi guidance + follow the greens".into(), if self.ground.guide.is_some() { "ON".into() } else { "OFF".into() }));
        }
        v
    }

    /// A ground-menu row was picked.
    pub fn ground_request(&mut self, world: &World, r: GroundReq) {
        match r {
            GroundReq::Push(d) => {
                let def = self.defs[self.sel].clone();
                self.ground.pending = None;
                self.pushback_key(&def);
                if self.ground.pending.is_some() {
                    self.ground.pending = Some((d, 1.0));
                    self.say(format!("Ground: pushback {}, tug on its way", d.label()), 3.0);
                }
            }
            GroundReq::PushStop => {
                if let Some(p) = &mut self.ground.push {
                    p.stop();
                }
            }
            GroundReq::ParkingBrake => {
                self.ground_key(world, Key::ParkingBrake);
            }
            GroundReq::Services => {
                self.ground_key(world, Key::GroundServices);
            }
            GroundReq::Autobrake => {
                self.ground_key(world, Key::Autobrake);
            }
            GroundReq::TailLock => {
                self.ground_key(world, Key::TailwheelLock);
            }
            GroundReq::TaxiGuide => {
                self.ground_key(world, Key::TaxiGuide);
            }
        }
    }

    fn pushback_key(&mut self, def: &crate::flight::AircraftDef) {
        if let Some(p) = &mut self.ground.push {
            if p.attached() && !matches!(p.phase, Phase::WaitBrake | Phase::Disconnecting(_)) {
                p.stop();
                return;
            }
        }
        if !pushback::can_push(def) {
            self.say("Pushback: tricycle-gear aircraft only", 2.5);
            return;
        }
        let gs = Vec2::new(self.ac.vel.x, self.ac.vel.z).length();
        if !self.ac.on_ground || gs > 0.5 {
            self.say("Pushback: stop first", 2.0);
            return;
        }
        let dir = match self.ground.pending {
            Some((d, _)) => d.next(),
            None => PushDir::Straight,
        };
        self.ground.pending = Some((dir, 3.0));
        self.say(format!("Pushback {} — press again to change, starting in 3 s", dir.label()), 3.0);
    }

    /// Taxi guidance: the ATC clearance's route if there is one, else to the
    /// departure holding point (on the ground before flying) or the nearest stand.
    pub(super) fn start_taxi_guide(&mut self, world: &World) {
        let def = self.defs[self.sel].clone();
        let nose = guide::nose_xz(&def, &self.ac);
        let atc_route = self.atc.dep.as_ref().filter(|d| d.route.len() >= 2 && d.route.last().is_some_and(|e| e.distance(nose) > 50.0));
        let g = match atc_route {
            Some(d) => Some(guide::Guide::new(
                d.route.clone(),
                d.via.clone(),
                format!("holding point {} RWY {}", d.hold.as_ref().map(|h| h.taxiway.clone()).unwrap_or_default(), d.runway),
                true,
            )),
            None => {
                // After a flight: to a stand; before one: to the departure holding point.
                if self.ground.flown { guide::plan_to_stand(world, nose, def.wingspan) } else { guide::plan_departure(world, nose, None) }
            }
        };
        if std::env::var_os("SKYWARD_GROUND_DEBUG").is_some() {
            eprintln!("ground: taxi guidance {:?}", g.as_ref().map(|g| (g.dest.clone(), g.length(), g.via.clone())));
        }
        match g {
            Some(g) => {
                self.say(format!("Taxi to {} — {:.0} m, follow the greens", g.dest, g.length()), 4.0);
                self.ground.guide = Some(g);
            }
            None => self.say("Taxi guidance: no taxiway layout here", 3.0),
        }
    }

    /// Tiller / toe brakes from the keys and axes (after the other controls).
    pub(super) fn ground_controls(&mut self, input: &Input, dt: f32) {
        let held = |k: Key| input.held.contains(&k) || input.pad_held.contains(&k);
        let h = |k: Key| if held(k) { 1.0 } else { 0.0 };
        let bot_on = self.bot.is_some() || self.mission_pilot.is_some();
        if bot_on {
            return;
        }
        let keys = h(Key::TillerRight) - h(Key::TillerLeft);
        let axis = input.analog.get(A::Tiller);
        if keys != 0.0 {
            let r = 1.8 * dt;
            self.ctl.tiller += (keys - self.ctl.tiller).clamp(-r, r);
            self.ground.tiller_input = 2.0;
        } else if let Some(v) = axis {
            self.ctl.tiller = v;
            if v.abs() > 0.02 {
                self.ground.tiller_input = 2.0;
            }
        } else {
            let r = 2.5 * dt;
            self.ctl.tiller -= self.ctl.tiller.clamp(-r, r);
        }
        self.ground.tiller_input = (self.ground.tiller_input - dt).max(if axis.is_some() { 1.0 } else { 0.0 });
        self.ctl.brake_left = h(Key::BrakeLeft).max(input.analog.get(A::BrakeLeft).unwrap_or(0.0));
        self.ctl.brake_right = h(Key::BrakeRight).max(input.analog.get(A::BrakeRight).unwrap_or(0.0));
    }

    /// Each physics step, before the flight model: the tug, chocks, ground
    /// power, and the pilot aids.
    pub(super) fn ground_step(&mut self, dt: f32) {
        let def = self.defs[self.sel].clone();
        let sp = spec::spec(&def);
        // The tug.
        let mut tow = None;
        if let Some(p) = &mut self.ground.push {
            tow = p.update(&def, &self.ac, &self.ctl, dt);
            if p.phase == Phase::Done {
                self.ground.push = None;
            }
        }
        // (The bot's own pushback: src/ground/bot_taxi.rs.)
        let bot_ground = self.bot.as_ref().and_then(|b| b.ground());
        self.ac.ground.tow = tow.or(bot_ground.and_then(|g| g.tow));
        let bot_taxiing = bot_ground.is_some();
        if bot_ground.is_some_and(|g| !matches!(g.stage, crate::ground::bot_taxi::Stage::Parked(_))) {
            // The bot's ground crew pulls the chocks and the ground power for the push.
            self.ground.chocks = false;
            self.ground.gpu = false;
        }
        self.ac.ground.chocks = self.ground.chocks;
        self.ac.sys.ext_power = self.ground.gpu;
        let gs = Vec2::new(self.ac.vel.x, self.ac.vel.z).length();
        // Rudder–tiller link: without a tiller, the rudder keys / pedals steer
        // the nose wheel through the tiller at taxi speed (full below 6 kt,
        // fading out by 40 kt), as most desk pilots expect.
        if self.ground.tiller_link && self.ground.tiller_input <= 0.0 && matches!(sp.steer, Steer::Tiller { .. }) && self.ac.on_ground && !bot_taxiing {
            let kt = gs / KT;
            let k = if kt < 6.0 { 1.0 } else { (1.0 - (kt - 6.0) / 34.0).clamp(0.0, 1.0) * 0.6 };
            if self.ctl.tiller.abs() < 1e-3 || self.bot.is_some() {
                self.ctl.tiller = self.ctl.rudder * k;
            }
        }
        // Skid release (like a pilot's feet): ease off a locking wheel.
        if self.ground.brake_assist && !sp.antiskid && gs > 3.0 && self.ac.ground.wheels.iter().any(|w| w.locked && w.contact) {
            self.ctl.brakes *= 0.5;
            self.ctl.brake_left *= 0.5;
            self.ctl.brake_right *= 0.5;
        }
    }

    /// Once a frame: crew messages, the autobrake selector after a disarm, the
    /// pushback start, the ground crew, taxi guidance.
    pub(super) fn ground_frame(&mut self, world: &World, dt: f32) {
        // A stand start waits for the airport layout (stands): retry every
        // 2 s; after a minute settle for the apron.
        if self.needs_respawn && self.custom_start.as_ref().is_some_and(|c| matches!(c.spec, crate::startpos::Spec::Stand(_))) {
            let step = dt.max(1.0 / 60.0);
            let t = self.ground.respawn_t + step;
            if t > 60.0 {
                if let Some(c) = self.custom_start.as_mut() {
                    c.spec = crate::startpos::Spec::Apron;
                }
                self.say("No stand data here yet — starting on the apron", 4.0);
            }
            if t > 60.0 || (t % 2.0) < step {
                self.reset(world);
                if !self.needs_respawn && self.bot.is_some() {
                    // The auto pilot starts over from where the aircraft now is.
                    self.enable_bot(world);
                }
            }
            self.ground.respawn_t = t;
            return;
        }
        let def = self.defs[self.sel].clone();
        // Settings › Flight & Realism › Autobrake: armed on each new flight.
        if !self.ground.prefs_applied {
            self.ground.prefs_applied = true;
            let pos = spec::spec(&def).autobrake_positions();
            if self.bot.is_none() && pos.contains(&self.prefs.autobrake) {
                self.ctl.autobrake = self.prefs.autobrake;
            }
        }
        // Settings › Assists › Taxi ribbon: guidance along an ATC taxi clearance.
        if self.prefs.taxi_ribbon && self.ground.guide.is_none() && self.ac.on_ground && self.bot.is_none() {
            let key = self.atc.dep.as_ref().filter(|d| d.route.len() >= 2).map(|d| ((d.route.len() as u64) << 32) ^ (d.route[0].x.to_bits() as u64));
            if key.is_some() && key != self.ground.ribbon_route {
                self.ground.ribbon_route = key;
                self.start_taxi_guide(world);
            }
        }
        // Advisories from the tyres / brakes and the tug driver.
        if self.ac.ground.note_seq != self.ground.seen_note {
            self.ground.seen_note = self.ac.ground.note_seq;
            let n = self.ac.ground.note;
            if !n.is_empty() {
                self.say(n, 3.5);
            }
        }
        if self.ac.ground.ab.disarmed && self.ctl.autobrake != Autobrake::Off {
            self.ctl.autobrake = Autobrake::Off;
        }
        if let Some(msg) = self.ground.push.as_ref().filter(|p| p.msg_seq != self.ground.seen_push).map(|p| (p.msg_seq, p.msg.clone())) {
            self.ground.seen_push = msg.0;
            self.say(msg.1, 4.0);
        }
        // Pushback starts after the direction has been picked.
        if let Some((dir, t)) = self.ground.pending {
            let t = t - dt;
            if t <= 0.0 {
                self.ground.pending = None;
                if self.ground.chocks || self.ground.gpu {
                    self.ground.chocks = false;
                    self.ground.gpu = false;
                }
                // Onto the taxilane behind the stand when the layout is known.
                let p = glam::Vec2::new(self.ac.pos.x, self.ac.pos.z);
                let lanes: Vec<(Vec2, Vec2)> = world.pavement.taxi.iter().filter(|t| crate::ground::surface::seg_dist(p, t.0, t.1) < 200.0).map(|t| (t.0, t.1)).collect();
                self.ground.push = Pushback::onto_lane(&def, &self.ac, dir, &lanes);
            } else {
                self.ground.pending = Some((dir, t));
            }
        }
        // The tug driver stops short of an obstruction (a building behind).
        let gs_now = Vec2::new(self.ac.vel.x, self.ac.vel.z);
        if let Some(p) = self.ground.push.as_mut().filter(|p| p.attached() && gs_now.length() > 0.2) {
            let ahead = gs_now.normalize() * 6.0;
            let blocked = def.hard_points.iter().any(|h| {
                let q = self.ac.pos + self.ac.rot * h.pos;
                crate::flight::Environment::obstacle_top(world, Vec2::new(q.x, q.z) + ahead).is_some()
            });
            if blocked && !p.stop_req {
                p.stop();
                self.say("Ground: stopping the push — obstruction behind", 3.0);
            }
        }
        let tug = self.ground.push.as_ref().or_else(|| self.bot.as_ref().and_then(|b| b.ground()).and_then(|g| g.pushback())).map(|p| p.tug.pos);
        if let Some(t) = tug {
            self.ground.tug_y = crate::flight::Environment::ground_height(world, t);
            if std::env::var_os("SKYWARD_GROUND_DEBUG").is_some() {
                eprintln!(
                    "ground: tug at ({:.1}, {:.1}, {:.1}), aircraft at ({:.1}, {:.1}, {:.1}) hdg {:.0}°",
                    t.x,
                    self.ground.tug_y,
                    t.y,
                    self.ac.pos.x,
                    self.ac.pos.y,
                    self.ac.pos.z,
                    self.ac.heading().to_degrees()
                );
            }
        }
        // Ground crew: parked (parking brake, stopped, engines off) for 20 s → chocks and ground power.
        let gs = Vec2::new(self.ac.vel.x, self.ac.vel.z).length();
        let engines = (0..def.engine_count).any(|i| self.ac.engine_running(i));
        let parked = self.ac.on_ground && gs < 0.1 && self.ctl.parking_brake && !engines && def.has_engine && self.ac.crashed.is_none() && self.bot.is_none();
        self.ground.parked_t = if parked { self.ground.parked_t + dt } else { 0.0 };
        if self.ground.parked_t > 20.0 && !self.ground.chocks {
            self.ground.chocks = true;
            self.ground.gpu = def.category != Category::Glider;
            self.say("Ground: chocks in, ground power connected", 3.5);
        }
        if self.ground.chocks && !self.ctl.parking_brake && self.ctl.throttle > 0.3 && gs < 0.1 && self.bot.is_none() {
            self.say("The chocks are in — ground services (Caps Lock) to remove them", 2.0);
        }
        if !self.ac.on_ground {
            self.ground.chocks = false;
            self.ground.gpu = false;
            self.ground.flown = true;
        }
        // Taxi guidance.
        let straight = crate::ground::bot_taxi::straight_speed(&def);
        let nose = guide::nose_xz(&def, &self.ac);
        let done = match &mut self.ground.guide {
            Some(g) => {
                g.update(nose, straight);
                (g.remaining() < 8.0 && gs < 0.5) || !self.ac.on_ground
            }
            None => false,
        };
        if done {
            if let Some(g) = self.ground.guide.take() {
                if self.ac.on_ground {
                    self.say(format!("Arrived: {}", g.dest), 3.0);
                }
            }
        }
        let _ = vec2;
    }
}
