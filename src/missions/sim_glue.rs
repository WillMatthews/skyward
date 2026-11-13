//! Missions inside the app: the [`Sim`] side of `missions/` (a child module
//! of `sim`, like `input/sim_glue.rs`, so it can reset private state).
//!
//! * runway [`Strip`]s from the world's runways (+ the target from the airport
//!   database if its tiles have not merged yet),
//! * the mission step after the physics, its requests (autopilot off,
//!   checkpoints), the scripted pilot (`--mission-bot`),
//! * the microburst as an [`Environment`] wrapper around the world for the
//!   flight model,
//! * checkpoints: retry from the last one.

use super::Sim;
use crate::missions::{Checkpoint, Ctx, MissionRun, Request, Strip};
use crate::world::World;

/// The world plus the mission's air (microburst) for the flight model.
pub type MissionEnv<'a> = crate::missions::AirEnv<'a>;

/// Runway strips near the aircraft (the world's runways), plus the target.
pub fn strips(world: &World, run: &MissionRun) -> Vec<Strip> {
    let mut v: Vec<Strip> = world
        .runways
        .iter()
        .filter(|r| r.paved || !r.airport.is_empty())
        .map(|r| Strip { airport: r.airport.clone(), ident_a: r.ident_a.clone(), ident_b: r.ident_b.clone(), a: r.a, b: r.b, width: r.width, elev: 0.5 * (r.elev[0] + r.elev[1]) })
        .collect();
    let t = &run.m.target;
    if run.target_strip(&v).is_none()
        && let Some((r, _)) = crate::startpos::find_runway(world, &t.airport, &t.runway)
    {
        let elev = r.height_at(r.a);
        v.push(Strip { airport: r.airport.clone(), ident_a: r.ident_a.clone(), ident_b: r.ident_b.clone(), a: r.a, b: r.b, width: r.width, elev });
    }
    v
}

impl Sim {
    /// Start a mission on this sim (the session has spawned at its start).
    pub fn begin_mission(&mut self, id: &str, bot: bool) -> bool {
        let Some(run) = MissionRun::by_id(id) else { return false };
        self.mission = Some(run);
        self.mission_checkpoint = None;
        self.mission_pilot = bot.then(crate::missions::pilot::MissionPilot::default);
        self.bot = None;
        self.watch = false;
        true
    }

    /// The pilot (bot) for one physics step, before the flight model.
    pub(super) fn mission_pilot_step(&mut self, world: &World, dt: f32) {
        let (Some(p), Some(run)) = (self.mission_pilot.as_mut(), self.mission.as_ref()) else { return };
        if !run.started || run.finished() {
            return;
        }
        let st = strips(world, run);
        let def = &self.defs[self.sel];
        let env = MissionEnv { inner: world, run: Some(run) };
        let mut req = Vec::new();
        p.fly(def, &mut self.ac, &mut self.ctl, run, &st, &env, dt, &mut req);
        for r in req {
            if r == crate::missions::pilot::PilotReq::Mayday {
                self.mission_mayday = true;
            }
        }
    }

    /// The mission after this frame's physics (`dt` simulated seconds).
    pub(super) fn mission_step(&mut self, world: &World, dt: f32) {
        let Some(mut run) = self.mission.take() else { return };
        if self.needs_respawn {
            self.mission = Some(run);
            return;
        }
        let first = !run.started;
        let st = strips(world, &run);
        let def = self.defs[self.sel].clone();
        let mayday = self.mission_mayday || self.atc.emergency.is_some() || self.nav.radios.squawk() == 7700;
        let mut ctx = Ctx { def: &def, ac: &mut self.ac, ctl: &mut self.ctl, env: world, strips: &st, severity: self.severity, mayday, dt };
        if first {
            let flap = run.setup(&mut ctx);
            self.flap_idx = flap;
            self.spoiler_idx = 0;
            self.ap = crate::autopilot::Autopilot::new();
            self.level_hold = None;
            if let Some((thr, dir, _)) = run.frame
                && let Some(s) = run.target_strip(&st)
            {
                let _ = (thr, dir);
                if let Some((r, _)) = crate::startpos::find_runway(world, &s.airport, &run.m.target.runway) {
                    self.runway = Some(r);
                }
            }
            self.snap_pose();
            // A checkpoint at the start: retry is always possible.
            self.mission_checkpoint = Some(Box::new(Checkpoint { name: "Start".into(), ac: self.ac.clone(), ctl: self.ctl, run: Box::new(run.clone()) }));
        } else {
            run.step(&mut ctx);
        }
        for r in std::mem::take(&mut run.requests) {
            match r {
                Request::ApOff => {
                    if self.ap.engaged {
                        self.ap.disengage();
                        self.ap_warn = 4.0;
                    }
                }
                Request::Checkpoint(name) => {
                    if self.ac.crashed.is_none() {
                        self.mission_checkpoint = Some(Box::new(Checkpoint { name, ac: self.ac.clone(), ctl: self.ctl, run: Box::new(run.clone()) }));
                    }
                }
                Request::Pa(_) => {}
            }
        }
        // The fire service rolls out to the aircraft once it has stopped after an emergency landing.
        if run.stopped_on_runway.is_some() && self.tenders.is_none() && run.fired.iter().any(|f| f.is_some()) {
            self.tenders = Some(crate::missions::tenders::Tenders::dispatch(world, self.ac.pos, self.runway.as_ref()));
        }
        self.mission = Some(run);
    }

    /// Retry from the last checkpoint (the start if none was passed).
    pub fn mission_retry_checkpoint(&mut self) -> bool {
        let Some(cp) = self.mission_checkpoint.clone() else { return false };
        self.ac = cp.ac.clone();
        self.ctl = cp.ctl;
        self.mission = Some((*cp.run).clone());
        let def = &self.defs[self.sel];
        let full = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
        let det = def.flap_detents;
        self.flap_idx = (0..det.len()).min_by(|&a, &b| (det[a] / full - self.ctl.flaps).abs().total_cmp(&(det[b] / full - self.ctl.flaps).abs())).unwrap_or(0);
        self.spoiler_idx = 0;
        self.ap = crate::autopilot::Autopilot::new();
        self.debris = None;
        self.severity = None;
        self.known_lost = 0;
        self.crash_clock = 0.0;
        self.audio_events.clear();
        self.vfx.clear();
        self.tenders = None;
        self.mission_mayday = false;
        self.paused = false;
        self.acc = 0.0;
        self.snap_pose();
        if let Some(p) = self.mission_pilot.as_mut() {
            *p = Default::default();
        }
        self.say(format!("Retrying from checkpoint: {}", cp.name), 3.0);
        true
    }

    /// Restart the mission from the beginning (with the sim's reset).
    pub(super) fn mission_restart(&mut self) {
        if let Some(r) = self.mission.as_mut() {
            *r = MissionRun::new(r.m);
        }
        self.mission_checkpoint = None;
        self.mission_mayday = false;
        self.tenders = None;
        if let Some(p) = self.mission_pilot.as_mut() {
            *p = Default::default();
        }
    }

    /// Seconds since the mission ended (None while it runs).
    pub fn mission_over_for(&self) -> Option<f32> {
        let r = self.mission.as_ref()?;
        r.end_t.map(|e| r.t - e)
    }
}
