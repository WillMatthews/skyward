//! Session glue: the sim steps the systems before each physics step, fires
//! armed / random failures, and once a frame updates the alerts, advisories
//! and what the autopilot / bot do about them.

use super::failures::Kind;
use super::{Ambient, flag};
use crate::sim::Sim;

impl Sim {
    /// One fixed physics step of the systems (before the flight model).
    pub(crate) fn systems_step(&mut self, amb: &Ambient, dt: f32) {
        let def = &self.defs[self.sel];
        // The auto-flight bot manages anti-ice / carb heat and never runs dry.
        let bot = self.bot.is_some();
        self.ac.sys.protect = bot;
        self.ac.sys.fuel_floor = if bot { 0.3 } else { 0.0 };
        // Handed over cold and dark on the ground: the bot does the start (instantly).
        if bot
            && self.ac.on_ground
            && self.ac.sys.init
            && self.ac.sys.autostart == 0
            && (0..self.ac.sys.n_eng).any(|i| !self.ac.sys.eng[i].run && !self.ac.sys.eng[i].dead && self.ac.sys.eng[i].fire <= 0.0)
        {
            self.ac.sys.make_ready();
        }
        super::step(&mut self.ac, def, &self.ctl, amb, dt);
        if !self.ac.sys.init {
            return;
        }
        let ias = self.ac.airspeed();
        let fired = self.failures.tick(&self.ac.sys.sd, def, self.time, self.ac.pos.y, ias, !self.ac.on_ground, dt);
        for f in fired {
            super::set_failure(&mut self.ac, def, f, true);
        }
    }

    /// Once a frame: alerts, advisories, automation reacting to failures.
    pub(crate) fn systems_frame(&mut self) {
        let def = self.defs[self.sel].clone();
        super::with_sys(&mut self.ac, |s, a| s.update_alerts(&def, a, &self.ctl));
        if self.ac.sys.note_seq != self.ac.sys.note_seen {
            self.ac.sys.note_seen = self.ac.sys.note_seq;
            let n = self.ac.sys.note;
            if !n.is_empty() {
                self.say(n, 5.0);
            }
        }
        // The autopilot needs its attitude / air data (and power).
        if self.ap.engaged && self.bot.is_none() {
            let ad = self.ac.sys.air_data(&self.ac);
            let sys = &self.ac.sys;
            let lost = ad.has(flag::ATT) || ad.has(flag::SPD) || (sys.sd.glass && !sys.bus_ok()) || sys.fail.any_of(Kind::ControlJam);
            if lost {
                self.ap.disengage();
                self.ap_warn = 4.0;
                self.say("Autopilot disconnected — systems failure", 3.0);
            }
        }
    }
}
