//! Airline-emergency missions (Track M): the player is dropped into a
//! developing emergency and must fly it to a safe outcome.
//!
//! * [`defs`]: data-driven mission definitions (`assets/missions/*.json`):
//!   aircraft, target runway, start state, conditions, a timeline of events
//!   (time / altitude / speed / distance / phase / player-action triggers that
//!   inject failures, damage, decompression, a microburst, cabin PA, crew and
//!   ATC lines, checkpoints), objectives, limits.
//! * [`MissionRun`]: the runtime — places the aircraft, fires the events,
//!   tracks objectives and the touchdown / ditching, decides the outcome and
//!   scores it ([`score`]). It sees the world only through [`Ctx`] (an
//!   `Environment` plus runway [`Strip`]s), so the same code runs inside the
//!   app (`sim_glue.rs`) and headless in the tests with a scripted pilot
//!   ([`pilot`]).
//! * [`ditching`]: survivable water landings; [`wind`]: the microburst;
//!   [`throttles`]: split thrust levers; [`records`]: best results;
//!   [`tenders`]: the airport fire service rolling out after an emergency
//!   landing.

pub mod defs;
pub mod ditching;
pub mod pilot;
pub mod records;
pub mod score;
pub mod tenders;
#[cfg(test)]
mod tests;
pub mod throttles;
pub mod wind;

use crate::damage::Severity;
use crate::flight::{Aircraft, AircraftDef, Controls, Environment, FPM, KT};
use defs::{Action, Burst, Mission, ObjKind, PlayerAct, Trigger};
use glam::{Vec2, Vec3, vec2, vec3};

const FT: f32 = 0.3048;
const NM: f32 = 1852.0;

/// Engines on the type (≤ MAX_ENGINES).
pub fn n_eng(def: &AircraftDef) -> usize {
    if def.has_engine { def.engine_count.min(def.engine_pos.len()).min(crate::flight::MAX_ENGINES) } else { 0 }
}

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// A runway the runtime knows: ends `a` → `b` (xz), width, surface height.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Strip {
    pub airport: String,
    pub ident_a: String,
    pub ident_b: String,
    pub a: Vec2,
    pub b: Vec2,
    pub width: f32,
    pub elev: f32,
}

impl Strip {
    pub fn length(&self) -> f32 {
        self.a.distance(self.b)
    }
    /// Landing frame for travel along `dir`: (threshold, unit direction, ident).
    pub fn frame_for(&self, dir: Vec2) -> (Vec2, Vec2, &str) {
        let d = (self.b - self.a).normalize_or(vec2(0.0, -1.0));
        if d.dot(dir) >= 0.0 { (self.a, d, &self.ident_a) } else { (self.b, -d, &self.ident_b) }
    }
    /// Threshold and direction of the end called `ident`.
    pub fn end(&self, ident: &str) -> Option<(Vec2, Vec2)> {
        let d = (self.b - self.a).normalize_or(vec2(0.0, -1.0));
        if self.ident_a == ident {
            Some((self.a, d))
        } else if self.ident_b == ident {
            Some((self.b, -d))
        } else {
            None
        }
    }
    /// (along from `thr`, right of the centreline) in the frame (thr, dir).
    pub fn local(p: Vec2, thr: Vec2, dir: Vec2) -> (f32, f32) {
        let r = p - thr;
        (r.dot(dir), r.dot(vec2(-dir.y, dir.x)))
    }
    /// On the paved surface (with `margin` m).
    pub fn contains(&self, p: Vec2, margin: f32) -> bool {
        let d = self.b - self.a;
        let t = ((p - self.a).dot(d) / d.length_squared().max(1.0)).clamp(0.0, 1.0);
        let along_ok = {
            let u = (p - self.a).dot(d) / d.length().max(1.0);
            u >= -margin && u <= d.length() + margin
        };
        along_ok && p.distance(self.a + d * t) <= self.width * 0.5 + margin
    }
}

/// An environment plus the mission's air (the microburst), for the flight model.
pub struct AirEnv<'a> {
    pub inner: &'a dyn Environment,
    pub run: Option<&'a MissionRun>,
}

impl Environment for AirEnv<'_> {
    fn ground_height(&self, xz: Vec2) -> f32 {
        self.inner.ground_height(xz)
    }
    fn obstacle_top(&self, xz: Vec2) -> Option<f32> {
        self.inner.obstacle_top(xz)
    }
    fn is_water(&self, xz: Vec2) -> bool {
        self.inner.is_water(xz)
    }
    fn surface(&self, xz: Vec2) -> crate::ground::Surface {
        self.inner.surface(xz)
    }
    fn updraft(&self, p: Vec3) -> f32 {
        let base = self.inner.updraft(p);
        match self.run {
            Some(r) if r.burst.is_some() => base + r.air(p, self.inner.ground_height(vec2(p.x, p.z))).1,
            _ => base,
        }
    }
    fn wind(&self, p: Vec3) -> Vec3 {
        let base = self.inner.wind(p);
        match self.run {
            Some(r) if r.burst.is_some() => {
                let (w, _) = r.air(p, self.inner.ground_height(vec2(p.x, p.z)));
                base + vec3(w.x, 0.0, w.y)
            }
            _ => base,
        }
    }
}

/// What the mission needs from the world this step.
pub struct Ctx<'a> {
    pub def: &'a AircraftDef,
    pub ac: &'a mut Aircraft,
    pub ctl: &'a mut Controls,
    pub env: &'a dyn Environment,
    pub strips: &'a [Strip],
    /// Crash severity once crashed (the session classifies it).
    pub severity: Option<Severity>,
    /// MAYDAY / PAN declared or squawk 7700.
    pub mayday: bool,
    /// Simulated seconds since the last step.
    pub dt: f32,
}

/// Requests the session carries out (things outside the aircraft).
#[derive(Clone, Debug, PartialEq)]
pub enum Request {
    ApOff,
    Checkpoint(String),
    /// Cabin PA clip id, with its words for the subtitles.
    Pa(String),
}

/// A line shown as a subtitle: (who, text).
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub who: &'static str,
    pub text: String,
    pub t: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObjState {
    pub visible: bool,
    /// Counting from (mission time).
    pub from: f32,
    pub done: Option<f32>,
    pub failed: bool,
    /// In-control objectives: the clock restarts after a loss of control.
    pub ok_since: f32,
}

/// The touchdown as measured.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Touchdown {
    pub t: f32,
    pub landing: score::Landing,
    pub runway: String,
    pub on_runway: bool,
    pub target: bool,
    pub ias_kt: f32,
    pub pitch_deg: f32,
}

/// The end of a mission.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Debrief {
    pub success: bool,
    pub headline: String,
    pub reason: String,
    pub score: score::Score,
    pub time_s: f32,
    pub pax: u32,
    pub injured: u32,
    pub fatal: u32,
    pub survival: score::Survival,
    /// (label, value) lines: touchdown numbers, procedures.
    pub stats: Vec<(String, String)>,
    /// Objectives: (text, done, required).
    pub objectives: Vec<(String, bool, bool)>,
}

/// A saved moment to retry from.
#[derive(Clone, Debug)]
pub struct Checkpoint {
    pub name: String,
    pub ac: Aircraft,
    pub ctl: Controls,
    pub run: Box<MissionRun>,
}

#[derive(Clone, Debug)]
pub struct MissionRun {
    pub m: &'static Mission,
    /// Mission clock (simulated seconds since the start).
    pub t: f32,
    pub started: bool,
    /// When each event fired.
    pub fired: Vec<Option<f32>>,
    pub obj: Vec<ObjState>,
    pub lines: Vec<Line>,
    pub requests: Vec<Request>,
    /// PA announcements (sequence for the sound, last clip).
    pub pa_seq: u32,
    pub pa: String,
    /// The target runway end (threshold, direction, elevation).
    pub frame: Option<(Vec2, Vec2, f32)>,
    pub burst: Option<(Burst, f32)>,
    /// Windshear warning is sounding (reactive detection).
    pub shear: bool,
    pub shear_seq: u32,
    was_airborne: bool,
    last_vs: f32,
    pub touchdown: Option<Touchdown>,
    td_g_until: f32,
    pub ditch: Option<ditching::Assessment>,
    pub max_g: f32,
    pub min_g: f32,
    pub stall_s: f32,
    pub overspeed_s: f32,
    pub lowest_agl: f32,
    crashed_t: Option<f32>,
    stopped_t: Option<f32>,
    pub stopped_on_runway: Option<bool>,
    pub end: Option<Debrief>,
    /// Mission time when it ended (the debrief shows a few seconds later).
    pub end_t: Option<f32>,
    pub airborne_since_start: bool,
    /// The aircraft has been moving (a stop counts only after that).
    pub moved: bool,
    /// Checkpoints this run has saved (the session keeps the snapshot).
    pub checkpoints: u32,
}

impl MissionRun {
    pub fn new(m: &'static Mission) -> MissionRun {
        MissionRun {
            m,
            t: 0.0,
            started: false,
            fired: vec![None; m.events.len()],
            obj: vec![ObjState::default(); m.objectives.len()],
            lines: Vec::new(),
            requests: Vec::new(),
            pa_seq: 0,
            pa: String::new(),
            frame: None,
            burst: None,
            shear: false,
            shear_seq: 0,
            was_airborne: false,
            last_vs: 0.0,
            touchdown: None,
            td_g_until: 0.0,
            ditch: None,
            max_g: 1.0,
            min_g: 1.0,
            stall_s: 0.0,
            overspeed_s: 0.0,
            lowest_agl: f32::INFINITY,
            crashed_t: None,
            stopped_t: None,
            stopped_on_runway: None,
            end: None,
            end_t: None,
            airborne_since_start: false,
            moved: false,
            checkpoints: 0,
        }
    }

    pub fn by_id(id: &str) -> Option<MissionRun> {
        defs::by_id(id).map(MissionRun::new)
    }

    pub fn finished(&self) -> bool {
        self.end.is_some()
    }

    /// Seconds left on the mission's limit (fire, oxygen…).
    pub fn time_left(&self) -> Option<f32> {
        self.m.time_limit_s.map(|l| (l - self.t).max(0.0))
    }

    /// The target strip for this mission among `strips`.
    pub fn target_strip<'s>(&self, strips: &'s [Strip]) -> Option<&'s Strip> {
        let t = &self.m.target;
        strips.iter().find(|s| s.airport == t.airport && (s.ident_a == t.runway || s.ident_b == t.runway))
    }

    /// A synthetic strip for the target (headless tests): threshold at `thr`.
    pub fn synthetic_strip(m: &Mission, thr: Vec2) -> Strip {
        let c = m.target.course_deg.to_radians();
        let dir = vec2(c.sin(), -c.cos());
        let recip = |r: &str| -> String {
            let n: String = r.chars().take_while(|c| c.is_ascii_digit()).collect();
            let suffix: String = r.chars().skip_while(|c| c.is_ascii_digit()).collect();
            let n: i32 = n.parse().unwrap_or(9);
            let o = ((n + 17) % 36) + 1;
            let s = match suffix.as_str() {
                "L" => "R",
                "R" => "L",
                x => x,
            };
            format!("{o:02}{s}")
        };
        Strip {
            airport: m.target.airport.clone(),
            ident_a: m.target.runway.clone(),
            ident_b: recip(&m.target.runway),
            a: thr,
            b: thr + dir * m.target.length_m,
            width: m.target.width_m,
            elev: m.target.elev_ft * FT,
        }
    }

    fn say(&mut self, who: &'static str, text: impl Into<String>) {
        self.lines.push(Line { who, text: text.into(), t: self.t });
    }

    /// Place the aircraft for the start (first step). Returns the flap detent.
    pub fn setup(&mut self, ctx: &mut Ctx) -> usize {
        let m = self.m;
        let def = ctx.def;
        self.started = true;
        let (thr, dir, elev) = match self.target_strip(ctx.strips).and_then(|s| s.end(&m.target.runway).map(|(t, d)| (t, d, s.elev))) {
            Some(f) => f,
            None => {
                let c = m.target.course_deg.to_radians();
                (vec2(ctx.ac.pos.x, ctx.ac.pos.z), vec2(c.sin(), -c.cos()), m.target.elev_ft * FT)
            }
        };
        self.frame = Some((thr, dir, elev));
        let st = &m.start;
        let right = vec2(-dir.y, dir.x);
        let hdg = heading_of(dir) + st.hdg_rel.to_radians();
        let full = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
        let flap = st.flaps.min(def.flap_detents.len().saturating_sub(1));
        let flaps = def.flap_detents.get(flap).copied().unwrap_or(0.0) / full;
        if st.on_runway {
            let p = thr + dir * st.x_m;
            *ctx.ac = Aircraft::on_runway(def, p, heading_of(dir), ctx.env);
            *ctx.ctl = Controls { flaps, gear_down: true, throttle: st.throttle.max(0.0), ..Default::default() };
            ctx.ac.set_flaps_now(flaps);
        } else {
            let p = thr + dir * st.x_nm * NM + right * st.y_nm * NM;
            let alt = st.alt_ft * FT;
            let v = st.kias * KT;
            *ctx.ac = Aircraft::airborne(def, vec3(p.x, alt, p.y), hdg, v);
            let mut ctl = crate::flight::trimmed_controls_at(def, v, alt);
            ctl.flaps = flaps;
            ctl.gear_down = st.gear_down;
            if st.throttle >= 0.0 {
                ctl.throttle = st.throttle;
            }
            ctx.ac.set_flaps_now(flaps);
            if st.gear_down {
                ctx.ac.extend_gear_now();
            } else {
                ctx.ac.retract_gear_now();
            }
            *ctx.ctl = ctl;
        }
        // Systems ready, fuel as briefed.
        let mut sys = crate::systems::Systems::new(def, false);
        for k in 0..3 {
            sys.tank[k] = sys.tank_cap[k] * m.fuel_frac;
        }
        sys.cabin = crate::systems::cabin::Cabin::new(def, ctx.ac.pos.y);
        ctx.ac.sys = sys;
        self.was_airborne = !ctx.ac.on_ground;
        self.airborne_since_start = !ctx.ac.on_ground;
        // Objectives with no "after" are visible from the start.
        for (k, o) in m.objectives.iter().enumerate() {
            self.obj[k].visible = o.after.is_none();
        }
        flap
    }

    /// Air added by the mission (microburst) at `p`: (wind xz, vertical).
    pub fn air(&self, p: Vec3, ground: f32) -> (Vec2, f32) {
        match (self.burst, self.frame) {
            (Some((b, t0)), Some((thr, dir, _))) => {
                let c = thr - dir * b.at_nm * NM;
                let k = ((self.t - t0) / b.ramp_s.max(0.1)).clamp(0.0, 1.0);
                wind::burst_air(&b, c, ground, p, k)
            }
            _ => (Vec2::ZERO, 0.0),
        }
    }

    fn act_now(&self, a: PlayerAct, ctx: &Ctx) -> bool {
        let sys = &ctx.ac.sys;
        let n = n_eng(ctx.def);
        match a {
            PlayerAct::Mayday => ctx.mayday,
            PlayerAct::FireHandle => sys.sw.fire_handle.iter().take(n).any(|&h| h),
            PlayerAct::AltGear => sys.sw.alt_gear,
            PlayerAct::GearDown => ctx.ctl.gear_down,
            PlayerAct::ThrottleIdle => (0..n.max(1)).all(|i| ctx.ctl.lever(i) < 0.05) || ctx.ctl.reverse,
            PlayerAct::Toga => (0..n.max(1)).all(|i| ctx.ctl.lever(i) > 0.92),
            PlayerAct::SpeedBrakes => ctx.ctl.spoilers > 0.5,
            PlayerAct::SplitThrottles => ctx.ctl.engine_throttle.is_some(),
            PlayerAct::Brakes => ctx.ac.on_ground && ctx.ctl.brakes > 0.5,
        }
    }

    fn triggered(&self, w: &Trigger, ctx: &Ctx) -> bool {
        let ac = &*ctx.ac;
        let alt_ft = ac.pos.y / FT;
        let agl_ft = ac.agl(ctx.env) / FT;
        let ias = ac.airspeed_kt();
        if w.t.is_some_and(|t| self.t < t) {
            return false;
        }
        if let Some(a) = &w.after {
            match self.m.events.iter().position(|e| &e.id == a).and_then(|k| self.fired[k]) {
                Some(t0) if self.t >= t0 + w.delay => {}
                _ => return false,
            }
        }
        if w.alt_above_ft.is_some_and(|x| alt_ft <= x) || w.alt_below_ft.is_some_and(|x| alt_ft >= x) || w.agl_below_ft.is_some_and(|x| agl_ft >= x) {
            return false;
        }
        if w.ias_above_kt.is_some_and(|x| ias <= x) || w.ias_below_kt.is_some_and(|x| ias >= x) {
            return false;
        }
        if let (Some(d), Some((thr, _, _))) = (w.dist_below_nm, self.frame)
            && vec2(ac.pos.x, ac.pos.z).distance(thr) / NM >= d
        {
            return false;
        }
        if w.on_ground.is_some_and(|g| g != ac.on_ground) {
            return false;
        }
        if w.player.is_some_and(|p| !self.act_now(p, ctx)) {
            return false;
        }
        true
    }

    fn apply(&mut self, a: &Action, ctx: &mut Ctx) {
        let def = ctx.def;
        match a {
            Action::Fail(id) | Action::Repair(id) => {
                if let Some((k, i)) = crate::systems::failures::Kind::parse(id) {
                    let on = matches!(a, Action::Fail(_));
                    crate::systems::set_failure(ctx.ac, def, crate::systems::failures::Failure::new(k, i), on);
                }
            }
            Action::EngineDead(i) => {
                let e = &mut ctx.ac.sys.eng[*i];
                e.run = false;
                e.dead = true;
                e.start = 0.0;
                e.bird = e.bird.max(0.9);
            }
            Action::FuelFrac(f) => {
                let sys = &mut ctx.ac.sys;
                for k in 0..3 {
                    sys.tank[k] = sys.tank_cap[k] * f.clamp(0.0, 1.0);
                }
            }
            Action::CabinLeak(tau) => {
                crate::systems::set_failure(ctx.ac, def, crate::systems::failures::Failure::new(crate::systems::failures::Kind::Decompression, 0), true);
                ctx.ac.sys.cabin.decompress(*tau);
            }
            Action::AltGearFail => ctx.ac.sys.alt_gear_fail = true,
            Action::BrakeFactor(f) => ctx.ac.sys.brake_factor = *f,
            Action::ApOff => self.requests.push(Request::ApOff),
            Action::Microburst(b) => self.burst = Some((*b, self.t)),
            Action::Pa(c) => {
                self.pa_seq = self.pa_seq.wrapping_add(1);
                self.pa = c.clone();
                let words = crate::audio::params::Clip::pa(c).map(|c| c.words()).unwrap_or("");
                self.say("PA", words);
                self.requests.push(Request::Pa(c.clone()));
            }
            Action::Note(text) => {
                let (who, t) = match text.split_once(": ") {
                    Some((w, t)) if w.len() <= 12 => (w, t),
                    _ => ("", text.as_str()),
                };
                let who: &'static str = match who {
                    "ATC" => "ATC",
                    "Purser" => "Purser",
                    "F/O" => "F/O",
                    "Captain" => "Captain",
                    "Cabin" => "Cabin",
                    "Engineer" => "Engineer",
                    _ => "",
                };
                self.say(who, t);
            }
            Action::Checkpoint(n) => {
                self.checkpoints += 1;
                self.requests.push(Request::Checkpoint(n.clone()));
            }
        }
    }

    /// Advance the mission by `ctx.dt` (simulated seconds).
    pub fn step(&mut self, ctx: &mut Ctx) {
        if self.end.is_some() {
            self.t += ctx.dt;
            return;
        }
        if !self.started {
            self.setup(ctx);
        }
        self.t += ctx.dt;
        let m = self.m;
        // ---- events
        for k in 0..m.events.len() {
            if self.fired[k].is_none() && self.triggered(&m.events[k].when, ctx) {
                self.fired[k] = Some(self.t);
                for a in m.events[k].actions.clone() {
                    self.apply(&a, ctx);
                }
                for (j, o) in m.objectives.iter().enumerate() {
                    if o.after.as_deref() == Some(m.events[k].id.as_str()) {
                        self.obj[j].visible = true;
                        self.obj[j].from = self.t;
                        self.obj[j].ok_since = self.t;
                    }
                }
            }
        }
        self.measure(ctx);
        self.objectives(ctx);
        self.decide(ctx);
    }

    /// Flight metrics: g, stalls, overspeed, windshear, touchdown, roll-out.
    fn measure(&mut self, ctx: &mut Ctx) {
        let ac = &*ctx.ac;
        let def = ctx.def;
        let dt = ctx.dt;
        if ac.crashed.is_none() {
            let g = ac.g_load();
            if !ac.on_ground {
                self.max_g = self.max_g.max(g);
                self.min_g = self.min_g.min(g);
            }
            if ac.stall_warning() {
                self.stall_s += dt;
            }
            let vmo = if def.vne_kt > 0.0 { def.vne_kt } else { f32::INFINITY };
            if ac.airspeed_kt() > vmo + 2.0 {
                self.overspeed_s += dt;
            }
            if !ac.on_ground {
                self.lowest_agl = self.lowest_agl.min(ac.agl(ctx.env));
                self.airborne_since_start = true;
            }
        }
        // Reactive windshear detection: strong sink air or a fast tailwind gain.
        let shear = self.burst.is_some() && !ac.on_ground && {
            let (w, up) = self.air(ac.pos, ctx.env.ground_height(vec2(ac.pos.x, ac.pos.z)));
            let head = -w.dot(vec2(ac.forward().x, ac.forward().z).normalize_or_zero());
            up < -3.0 || head < -6.0
        };
        if shear && !self.shear {
            self.shear_seq = self.shear_seq.wrapping_add(1);
            self.say("", "WINDSHEAR — WINDSHEAR — WINDSHEAR");
        }
        self.shear = shear;
        // ---- touchdown
        let xz = vec2(ac.pos.x, ac.pos.z);
        if ac.on_ground && self.was_airborne && ac.crashed.is_none() && self.touchdown.is_none() {
            let fwd = vec2(ac.forward().x, ac.forward().z).normalize_or(vec2(0.0, -1.0));
            let strip = ctx.strips.iter().filter(|s| s.contains(xz, 15.0)).min_by(|a, b| {
                let da = a.frame_for(fwd).1.dot(fwd);
                let db = b.frame_for(fwd).1.dot(fwd);
                db.total_cmp(&da)
            });
            let (landing, rw, on, target) = match strip {
                Some(s) => {
                    let (thr, d, id) = s.frame_for(fwd);
                    let (along, side) = Strip::local(xz, thr, d);
                    let target = s.airport == self.m.target.airport;
                    let l = score::Landing {
                        sink_fpm: (-self.last_vs).max(0.0) / FPM,
                        side_m: side,
                        along_m: along,
                        runway_len: s.length(),
                        runway_width: s.width,
                        speed_dev_kt: ac.airspeed_kt() - def.vref_kt,
                        g: ac.g_load(),
                        bank_deg: ac.roll().to_degrees().abs(),
                        stopped_on_runway: false,
                    };
                    (l, format!("{} {id}", s.airport), s.contains(xz, 0.0), target)
                }
                None => (
                    score::Landing { sink_fpm: (-self.last_vs).max(0.0) / FPM, g: ac.g_load(), bank_deg: ac.roll().to_degrees().abs(), ..Default::default() },
                    String::new(),
                    false,
                    false,
                ),
            };
            self.touchdown = Some(Touchdown { t: self.t, landing, runway: rw, on_runway: on, target, ias_kt: ac.airspeed_kt(), pitch_deg: ac.pitch().to_degrees() });
            self.td_g_until = self.t + 1.0;
        }
        if let Some(td) = self.touchdown.as_mut()
            && self.t <= self.td_g_until
            && ac.crashed.is_none()
        {
            td.landing.g = td.landing.g.max(ac.g_load());
        }
        if !ac.on_ground {
            self.last_vs = ac.vertical_speed();
        }
        self.was_airborne = !ac.on_ground;
        // ---- crash / ditching
        if ac.crashed.is_some() && self.crashed_t.is_none() {
            self.crashed_t = Some(self.t);
            let reason = ac.crashed.clone().unwrap_or_default();
            if reason.contains("Ditched")
                && let Some(imp) = &ac.impact
            {
                self.ditch = Some(ditching::assess(def, imp));
            } else if let (Some(imp), None) = (&ac.impact, &self.touchdown) {
                // A crash landing (gear up, collapse): measure the arrival like a touchdown.
                let p = vec2(imp.pos.x, imp.pos.z);
                let fwd = vec2((imp.rot * Vec3::Z).x, (imp.rot * Vec3::Z).z).normalize_or(vec2(0.0, -1.0));
                if let Some(s) = ctx.strips.iter().find(|s| s.contains(p, 10.0)) {
                    let (thr, d, id) = s.frame_for(fwd);
                    let (along, side) = Strip::local(p, thr, d);
                    let u = imp.rot * Vec3::Y;
                    let r = imp.rot * Vec3::NEG_X;
                    let landing = score::Landing {
                        sink_fpm: (-imp.vel.y).max(0.0) / FPM,
                        side_m: side,
                        along_m: along,
                        runway_len: s.length(),
                        runway_width: s.width,
                        speed_dev_kt: vec2(imp.vel.x, imp.vel.z).length() / KT - def.vref_kt,
                        g: self.max_g.max(1.0),
                        bank_deg: (-r.y).atan2(u.y).to_degrees().abs(),
                        stopped_on_runway: false,
                    };
                    let ias = vec2(imp.vel.x, imp.vel.z).length() / KT;
                    self.touchdown = Some(Touchdown {
                        t: self.t,
                        landing,
                        runway: format!("{} {id}", s.airport),
                        on_runway: s.contains(p, 0.0),
                        target: s.airport == self.m.target.airport,
                        ias_kt: ias,
                        pitch_deg: (imp.rot * Vec3::Z).y.asin().to_degrees(),
                    });
                }
            }
        }
        // ---- stopped (on the wheels, or the wreck has settled)
        let gs = vec2(ac.vel.x, ac.vel.z).length();
        if gs > 10.0 {
            self.moved = true;
        }
        let stopped = self.moved && ((ac.on_ground && ac.crashed.is_none() && gs < 1.0) || self.crashed_t.is_some_and(|c| self.t - c > 4.0 && gs < 2.0));
        if stopped && self.stopped_t.is_none() && (self.airborne_since_start || ac.crashed.is_some() || self.m.start.on_runway) {
            self.stopped_t = Some(self.t);
            let on = ctx.strips.iter().any(|s| s.contains(xz, 2.0)) && !ctx.env.is_water(xz);
            self.stopped_on_runway = Some(on);
            if let Some(td) = self.touchdown.as_mut() {
                td.landing.stopped_on_runway = on;
            }
        }
    }

    fn loss_of_control(&self, ctx: &Ctx) -> bool {
        let ac = &*ctx.ac;
        ac.crashed.is_some() || ac.stall_warning() || ac.roll().abs() > 60f32.to_radians() || (ctx.def.vne_kt > 0.0 && ac.airspeed_kt() > ctx.def.vne_kt + 5.0)
    }

    fn objectives(&mut self, ctx: &Ctx) {
        let m = self.m;
        let ac = &*ctx.ac;
        let survivable = ctx.severity.is_none_or(|s| s.survivable());
        let crashed_ok = ac.crashed.is_none() || (m.survivable_ok && survivable);
        let loc = self.loss_of_control(ctx);
        for (k, o) in m.objectives.iter().enumerate() {
            let s = self.obj[k];
            if !s.visible || s.done.is_some() || s.failed {
                continue;
            }
            let elapsed = self.t - s.from;
            let landed_on = |target_only: bool| -> bool {
                self.stopped_on_runway == Some(true)
                    && crashed_ok
                    && (self.touchdown.as_ref().is_some_and(|td| td.on_runway && (!target_only || td.target)) || (ac.crashed.is_some() && m.survivable_ok))
            };
            let done = match &o.kind {
                ObjKind::Land { target_only } => landed_on(*target_only),
                ObjKind::LandOrDitch => landed_on(false) || (self.ditch.is_some_and(|d| d.severity.survivable()) && self.stopped_t.is_some()),
                ObjKind::StopOnRunway => self.stopped_on_runway == Some(true) && ac.crashed.is_none(),
                ObjKind::AltBelow(ft) => ac.pos.y < ft * FT && ac.crashed.is_none(),
                ObjKind::AglAbove(ft) => ac.agl(ctx.env) > ft * FT && ac.crashed.is_none(),
                ObjKind::InControl(secs) => {
                    if loc {
                        self.obj[k].ok_since = self.t;
                    }
                    self.t - self.obj[k].ok_since >= *secs && ac.crashed.is_none()
                }
                ObjKind::Player(a) => self.act_now(*a, ctx),
                ObjKind::FireOut(i) => ac.sys.eng[*i].fire <= 0.0 && ac.crashed.is_none(),
                ObjKind::NoLowerThan(_) => false,
            };
            if done {
                self.obj[k].done = Some(self.t);
                continue;
            }
            let failed = match &o.kind {
                ObjKind::NoLowerThan(ft) => !ac.on_ground && ac.crashed.is_none() && ac.agl(ctx.env) < ft * FT,
                ObjKind::Land { .. } | ObjKind::LandOrDitch | ObjKind::StopOnRunway => self.stopped_t.is_some() || (ac.crashed.is_some() && !crashed_ok),
                _ => ac.crashed.is_some() && !matches!(o.kind, ObjKind::Player(_)),
            };
            if failed || o.within_s.is_some_and(|w| elapsed > w) {
                self.obj[k].failed = true;
            }
        }
    }

    fn required_ok(&self) -> bool {
        self.m.objectives.iter().zip(&self.obj).filter(|(o, _)| o.required).all(|(o, s)| s.done.is_some() || (matches!(o.kind, ObjKind::NoLowerThan(_)) && !s.failed && s.visible))
    }

    /// Decide whether the mission is over, and how it went.
    fn decide(&mut self, ctx: &Ctx) {
        let m = self.m;
        let ac = &*ctx.ac;
        let destroyed = ctx.severity == Some(Severity::Destroyed);
        // Required objectives that can still complete (no-lower-than counts only with another one done).
        let others_done = m.objectives.iter().zip(&self.obj).filter(|(o, _)| o.required && !matches!(o.kind, ObjKind::NoLowerThan(_))).all(|(_, s)| s.done.is_some());
        let failed_req = m.objectives.iter().zip(&self.obj).find(|(o, s)| o.required && s.failed).map(|(o, _)| o.text.clone());
        let over_time = m.time_limit_s.is_some_and(|l| self.t > l);
        let result: Option<(bool, String, String)> = if destroyed {
            Some((false, "Aircraft destroyed".into(), ac.crashed.clone().unwrap_or_default()))
        } else if over_time {
            Some((false, "Out of time".into(), if m.limit_text.is_empty() { "The time limit ran out".into() } else { m.limit_text.clone() }))
        } else if let Some(t) = failed_req {
            let why = ac.crashed.clone().unwrap_or_else(|| format!("Objective failed: {t}"));
            Some((false, if ac.crashed.is_some() { "Crash landing".into() } else { "Mission failed".into() }, why))
        } else if others_done && self.required_ok() {
            let crashed = ac.crashed.clone();
            let head = match (&crashed, self.ditch.is_some()) {
                (_, true) => "Ditched — everyone survives",
                (Some(_), false) => "Down and survivable",
                (None, false) => "Aircraft recovered",
            };
            Some((true, head.into(), crashed.unwrap_or_else(|| "All required objectives complete".into())))
        } else if ac.crashed.is_some() && self.crashed_t.is_some_and(|c| self.t - c > 8.0) {
            // A survivable crash that the mission does not accept.
            Some((false, "Crash landing".into(), ac.crashed.clone().unwrap_or_default()))
        } else {
            None
        };
        if let Some((success, head, reason)) = result {
            self.end = Some(self.debrief(ctx, success, head, reason));
            self.end_t = Some(self.t);
        }
    }

    fn debrief(&self, ctx: &Ctx, success: bool, headline: String, reason: String) -> Debrief {
        let m = self.m;
        let ac = &*ctx.ac;
        let def = ctx.def;
        let survival = match (ctx.severity, ac.crashed.is_some()) {
            (Some(Severity::Destroyed), _) => score::Survival::Destroyed,
            (Some(Severity::Substantial), _) => score::Survival::HullLoss,
            (Some(Severity::Minor), _) | (None, true) => score::Survival::Damaged,
            (None, false) => score::Survival::Intact,
        };
        // Injuries: hard arrivals, hypoxia, smoke, the ditching / evacuation.
        let hyp = ac.sys.cabin.pax_hypoxia / 600.0;
        let td_g = self.touchdown.as_ref().map_or(1.0, |t| t.landing.g);
        let g = ((td_g.max(self.max_g) - 2.2) / 3.0).max(0.0) + ((-self.min_g - 0.2) / 2.0).max(0.0);
        let base = match survival {
            score::Survival::Destroyed => 1.0,
            score::Survival::HullLoss => 0.06,
            score::Survival::Damaged => 0.02,
            score::Survival::Intact => 0.0,
        };
        let injured_f = (base + hyp + g).clamp(0.0, 1.0);
        let fatal = if survival == score::Survival::Destroyed { m.pax } else { 0 };
        let injured = if survival == score::Survival::Destroyed { 0 } else { (m.pax as f32 * injured_f).round() as u32 };
        let mut proc_points = 0.0;
        let mut proc_max = 0.0;
        for (o, s) in m.objectives.iter().zip(&self.obj) {
            if o.points > 0.0 {
                proc_max += o.points;
                let ok = s.done.is_some() || (matches!(o.kind, ObjKind::NoLowerThan(_)) && s.visible && !s.failed);
                if ok {
                    proc_points += o.points;
                }
            }
        }
        let landing = self.touchdown.as_ref().filter(|t| t.on_runway).map(|t| t.landing);
        let ditch = self.ditch.map(|d| d.quality);
        let handling = if landing.is_none() && ditch.is_none() {
            let h =
                1.0 - (self.stall_s / 10.0).min(0.6) - (self.overspeed_s / 20.0).min(0.4) - ((self.max_g - 2.0).max(0.0) / 1.5).min(0.4) - ((0.0 - self.min_g).max(0.0)).min(0.3);
            Some(h.clamp(0.0, 1.0))
        } else {
            None
        };
        let inputs = score::Inputs {
            success,
            survival,
            injured: if survival == score::Survival::Destroyed { 1.0 } else { injured_f },
            landing,
            ditch,
            handling,
            proc_points,
            proc_max,
            time_s: self.t,
            par_s: m.par_s,
            max_sink_fpm: def.max_touchdown_sink / FPM,
        };
        let sc = score::score(&inputs);
        let mut stats = Vec::new();
        if let Some(td) = &self.touchdown {
            let l = &td.landing;
            stats.push(("Touchdown".into(), if td.runway.is_empty() { "off the runway".into() } else { format!("runway {}", td.runway) }));
            stats.push(("Sink rate".into(), format!("{:.0} ft/min", l.sink_fpm)));
            stats.push(("Speed".into(), format!("{:.0} kt (Vref {:+.0})", td.ias_kt, l.speed_dev_kt)));
            if td.on_runway {
                stats.push(("Centreline".into(), format!("{:.1} m {}", l.side_m.abs(), if l.side_m >= 0.0 { "right" } else { "left" })));
                stats.push(("Past threshold".into(), format!("{:.0} m", l.along_m)));
            }
            stats.push(("Load factor".into(), format!("{:.2} g", l.g)));
            stats.push(("Bank".into(), format!("{:.1}°", l.bank_deg)));
            if let Some(on) = self.stopped_on_runway {
                stats.push(("Stopped".into(), if on { "on the runway".into() } else { "off the runway".into() }));
            }
        }
        if let Some(d) = &self.ditch {
            stats.push(("Ditching".into(), d.verdict.into()));
            stats.push(("Water entry".into(), format!("{:.0} kt, {:.0} ft/min, pitch {:.1}°, bank {:.1}°", d.speed_kt, d.sink_fpm, d.pitch_deg, d.bank_deg)));
        }
        if ac.sys.cabin.peak > 10_000.0 * FT {
            stats.push(("Peak cabin altitude".into(), format!("{:.0} ft", ac.sys.cabin.peak / FT)));
        }
        stats.push(("Max load factor".into(), format!("{:.2} g", self.max_g)));
        if self.stall_s > 0.0 {
            stats.push(("Stall warning".into(), format!("{:.0} s", self.stall_s)));
        }
        if self.overspeed_s > 0.0 {
            stats.push(("Overspeed".into(), format!("{:.0} s", self.overspeed_s)));
        }
        let objectives = m
            .objectives
            .iter()
            .zip(&self.obj)
            .map(|(o, s)| (o.text.clone(), s.done.is_some() || (matches!(o.kind, ObjKind::NoLowerThan(_)) && s.visible && !s.failed), o.required))
            .collect();
        Debrief { success, headline, reason, score: sc, time_s: self.t, pax: m.pax, injured, fatal, survival, stats, objectives }
    }

    /// The objectives to show now: (text, state, required, seconds left).
    pub fn hud_objectives(&self) -> Vec<(String, ObjShow, bool, Option<f32>)> {
        self.m
            .objectives
            .iter()
            .zip(&self.obj)
            .filter(|(_, s)| s.visible)
            .map(|(o, s)| {
                let state = if s.done.is_some() {
                    ObjShow::Done
                } else if s.failed {
                    ObjShow::Failed
                } else {
                    ObjShow::Open
                };
                let left = o.within_s.map(|w| (s.from + w - self.t).max(0.0)).filter(|_| state == ObjShow::Open);
                (o.text.clone(), state, o.required, left)
            })
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjShow {
    Open,
    Done,
    Failed,
}

/// Lat / lon of a point in the target runway frame (for the start's scenery
/// load): `x_nm` along the runway course from the threshold, `y_nm` right.
pub fn start_latlon(m: &Mission, thr: (f64, f64)) -> (f64, f64) {
    let c = (m.target.course_deg as f64).to_radians();
    let (x, y) = (m.start.x_nm as f64 * 1852.0, m.start.y_nm as f64 * 1852.0);
    let (x, y) = if m.start.on_runway { (m.start.x_m as f64, 0.0) } else { (x, y) };
    let north = x * c.cos() - y * c.sin();
    let east = x * c.sin() + y * c.cos();
    let lat = thr.0 + north / 111_320.0;
    let lon = thr.1 + east / (111_320.0 * thr.0.to_radians().cos());
    (lat, lon)
}
