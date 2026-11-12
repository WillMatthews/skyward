//! Skyward autopilot: heading / altitude / vertical-speed / airspeed hold with
//! autothrottle, for every aircraft in [`crate::flight::catalogue`].
//!
//! The app calls [`Autopilot::update`] once per frame *after* reading the
//! keyboard into `Controls`; while engaged it overwrites elevator, aileron,
//! rudder, trim and (with autothrottle) throttle. Disengage it on any large
//! pilot input.
//!
//! Modes
//! -----
//! * Lateral: `hdg = Some(rad)` turns onto and holds a heading (smooth,
//!   rate-limited bank, at most 25° for airliners, 30° otherwise); `None`
//!   holds the wings level.
//! * Vertical: `alt = Some(m)` climbs/descends to and captures an altitude
//!   (at `vs` if given, else at the type's normal rate), with a constant-
//!   acceleration capture so there is no overshoot; `alt = None, vs = Some`
//!   holds a vertical speed; neither holds the pitch attitude at engagement.
//!   Gliders (no engine) with `ias = Some` fly that speed on the elevator.
//! * Autothrottle: `ias = Some(m/s indicated)` with an engine. When the
//!   engines cannot hold the speed in a climb the climb rate is reduced
//!   (speed has priority); without autothrottle the climb rate is limited to
//!   keep at least 1.3 × the stall speed.
//!
//! The inner loops are per control law: on direct-law aircraft the elevator
//! is a PD on pitch with the integrator moved onto the trim (so disengaging
//! leaves the aircraft in trim); on fly-by-wire aircraft the elevator is a
//! load-factor demand and the law's own flight-path hold does the rest.

use crate::flight::{Aircraft, AircraftDef, Category, ControlLaw, Controls, GRAVITY, KT};
use std::f32::consts::PI;

/// Autopilot targets and internal loop state.
#[derive(Clone, Debug, Default)]
pub struct Autopilot {
    pub engaged: bool,
    /// Heading to hold, radians (0 = north, π/2 = east). `None` = wings level.
    pub hdg: Option<f32>,
    /// Altitude to capture and hold, metres.
    pub alt: Option<f32>,
    /// Indicated airspeed to hold with the autothrottle, m/s.
    pub ias: Option<f32>,
    /// Vertical speed, m/s (+ = climb). With `alt` it is the rate used to get
    /// there (sign taken from the direction of the target).
    pub vs: Option<f32>,
    // --- internal state ---
    init: bool,
    pitch_hold: f32,
    pitch_i: f32,
    vs_cmd: f32,
    bank_cmd: f32,
    thr_i: f32,
    prev_ias: f32,
    accel_f: f32,
}

/// Per-type tuning.
struct Gains {
    bank_max: f32,
    pitch_up: f32,
    pitch_down: f32,
    vs_climb: f32,
    vs_descent: f32,
    /// Vertical acceleration for captures, m/s².
    vs_accel: f32,
    kp: f32,
    kd: f32,
    ki: f32,
    /// Autothrottle P (per m/s), I (per m), D (per m/s²).
    at_p: f32,
    at_i: f32,
    at_d: f32,
}

fn gains(def: &AircraftDef) -> Gains {
    let deg = |d: f32| d.to_radians();
    match def.category {
        Category::Airliner => Gains {
            bank_max: deg(25.0),
            pitch_up: deg(18.0),
            pitch_down: deg(-10.0),
            vs_climb: if def.engine_kind == crate::flight::EngineKind::Jet { 10.0 } else { 6.5 },
            vs_descent: -10.0,
            vs_accel: 0.35,
            kp: 2.2,
            kd: 2.0,
            ki: 0.35,
            at_p: 0.035,
            at_i: 0.006,
            at_d: 0.15,
        },
        Category::FastJet => Gains {
            bank_max: deg(30.0),
            pitch_up: deg(25.0),
            pitch_down: deg(-20.0),
            vs_climb: 30.0,
            vs_descent: -30.0,
            vs_accel: 1.5,
            kp: 2.0,
            kd: 1.0,
            ki: 0.4,
            at_p: 0.04,
            at_i: 0.008,
            at_d: 0.10,
        },
        Category::Warbird => Gains {
            bank_max: deg(30.0),
            pitch_up: deg(20.0),
            pitch_down: deg(-15.0),
            vs_climb: 10.0,
            vs_descent: -10.0,
            vs_accel: 0.8,
            kp: 2.0,
            kd: 0.9,
            ki: 0.5,
            at_p: 0.06,
            at_i: 0.012,
            at_d: 0.10,
        },
        Category::Trainer | Category::Glider => Gains {
            bank_max: deg(30.0),
            pitch_up: deg(15.0),
            pitch_down: deg(-15.0),
            vs_climb: 3.5,
            vs_descent: -4.0,
            vs_accel: 0.6,
            kp: 2.0,
            kd: 0.7,
            ki: 0.6,
            at_p: 0.08,
            at_i: 0.02,
            at_d: 0.10,
        },
    }
}

fn wrap_pi(a: f32) -> f32 {
    let mut d = a.rem_euclid(2.0 * PI);
    if d > PI {
        d -= 2.0 * PI;
    }
    d
}

fn approach(cur: f32, target: f32, max_step: f32) -> f32 {
    cur + (target - cur).clamp(-max_step, max_step)
}

impl Autopilot {
    pub fn new() -> Self {
        Self::default()
    }

    /// Engage holding the present heading, altitude and (with an engine)
    /// indicated airspeed.
    pub fn engage_hold_current(&mut self, ac: &Aircraft) {
        self.engaged = true;
        self.hdg = Some(ac.heading());
        self.alt = Some(ac.pos.y);
        self.ias = Some(ac.airspeed());
        self.vs = None;
        self.init = false;
    }

    /// Disengage (targets are kept for the next engagement).
    pub fn disengage(&mut self) {
        self.engaged = false;
        self.init = false;
    }

    fn reset(&mut self, ac: &Aircraft, ctl: &Controls) {
        self.init = true;
        self.pitch_hold = ac.pitch();
        self.pitch_i = 0.0;
        self.vs_cmd = ac.vertical_speed();
        self.bank_cmd = ac.roll();
        self.thr_i = ctl.throttle;
        self.prev_ias = ac.airspeed();
        self.accel_f = 0.0;
    }

    /// Run the autopilot for one frame and write its demands into `ctl`.
    /// Returns whether it is still engaged (it drops out on the ground or
    /// after a crash).
    pub fn update(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, dt: f32) -> bool {
        if !self.engaged {
            self.init = false;
            return false;
        }
        if ac.crashed.is_some() || ac.on_ground || !dt.is_finite() || dt <= 0.0 {
            self.disengage();
            return false;
        }
        let dt = dt.min(0.1);
        if !self.init {
            self.reset(ac, ctl);
            // Start the trim integrator from the present trim so engagement is bumpless.
            if def.control_law == ControlLaw::Direct {
                self.pitch_i = ctl.trim * def.trim_authority + ctl.elevator;
            }
        }
        let g = gains(def);
        let tas = ac.tas().max(10.0);
        let ias = ac.airspeed();
        let fbw = def.control_law != ControlLaw::Direct;
        let flap = ac.surfaces().flaps;
        let v_min = 1.2 * def.stall_speed(flap);

        // Smoothed acceleration (for the autothrottle damping).
        let a_raw = (ias - self.prev_ias) / dt;
        self.prev_ias = ias;
        self.accel_f += (a_raw - self.accel_f) * (1.0 - (-dt / 0.5f32).exp());

        // ---------------- autothrottle ----------------
        let autothrottle = def.has_engine && self.ias.is_some();
        let thr_max = if def.has_afterburner { 0.9 } else { 1.0 };
        let mut thr_saturated_hi = false;
        if autothrottle {
            let target = self.ias.unwrap_or(ias);
            let e = target - ias;
            // Feed-forward of the extra thrust a climb (or descent) needs.
            let t_avail = (def.thrust(tas, 1.0) * crate::flight::isa(ac.pos.y).sigma().powf(0.75)).max(1.0);
            let ff = (self.vs_cmd / tas * def.mass * GRAVITY / t_avail).clamp(-0.5, 0.8);
            self.thr_i = (self.thr_i + g.at_i * e * dt).clamp(-0.3, thr_max);
            let thr = (self.thr_i + ff + g.at_p * e - g.at_d * self.accel_f).clamp(0.0, thr_max);
            ctl.throttle = approach(ctl.throttle, thr, 0.35 * dt);
            thr_saturated_hi = thr >= thr_max - 1e-3 && e > 1.0;
        }

        // ---------------- vertical ----------------
        let speed_on_pitch = !def.has_engine && self.ias.is_some();
        let mut gamma_cmd = None; // flight-path demand (rad)
        let mut pitch_cmd = None; // direct pitch demand (rad)
        if speed_on_pitch {
            // Glider: fly the speed with the elevator (fast → pitch up).
            let e = ias - self.ias.unwrap_or(ias);
            self.pitch_hold = (self.pitch_hold + 0.01 * e * dt).clamp(g.pitch_down, g.pitch_up);
            pitch_cmd = Some((self.pitch_hold + 0.02 * e).clamp(g.pitch_down, g.pitch_up));
        } else if self.alt.is_some() || self.vs.is_some() {
            let mut vs_t = match (self.alt, self.vs) {
                (Some(h), vs) => {
                    let err = h - ac.pos.y;
                    let rate = match vs {
                        Some(v) if v.abs() > 0.1 => v.abs(),
                        _ => if err > 0.0 { g.vs_climb } else { -g.vs_descent },
                    };
                    // Constant-deceleration capture (no overshoot), linear near the target.
                    let cap = (2.0 * 0.5 * g.vs_accel * err.abs()).sqrt().min(0.25 * err.abs());
                    err.signum() * rate.min(cap)
                }
                (None, Some(v)) => v,
                (None, None) => 0.0,
            };
            // Speed protection: give up climb rate rather than airspeed.
            if vs_t > 0.0 {
                let floor = if autothrottle { self.ias.unwrap_or(ias) - 3.0 * KT } else { 1.12 * v_min };
                if ias < floor || thr_saturated_hi {
                    let deficit = (floor - ias).max(0.0) + if thr_saturated_hi { 0.5 } else { 0.0 };
                    vs_t = (vs_t - 1.5 * deficit).max(-1.0);
                }
            }
            if ias < v_min {
                vs_t = vs_t.min(-0.5 * (v_min - ias));
            }
            // ...and on the way down, with the throttles closed, give up
            // descent rate rather than overspeed.
            if vs_t < 0.0 && autothrottle && ctl.throttle < 0.02 {
                let excess = ias - (self.ias.unwrap_or(ias) + 2.0 * KT);
                if excess > 0.0 {
                    vs_t *= (1.0 - excess / (5.0 * KT)).clamp(0.0, 1.0);
                }
            }
            vs_t = vs_t.clamp(g.vs_descent.min(-1.0) * 1.5, g.vs_climb.max(1.0) * 1.5);
            self.vs_cmd = approach(self.vs_cmd, vs_t, g.vs_accel * dt);
            gamma_cmd = Some((self.vs_cmd / tas).clamp(-0.5, 0.5).asin());
        } else {
            pitch_cmd = Some(self.pitch_hold);
        }

        // Pitch / flight-path loop → elevator (and trim on direct-law aircraft).
        let gamma = (ac.vertical_speed() / ac.vel.length().max(1.0)).clamp(-1.0, 1.0).asin();
        let theta = ac.pitch();
        let q = ac.rates().y;
        let phi = ac.roll();
        // Pitch error, rad (flight-path error is turned into an equivalent pitch error).
        let mut e_pitch = match (gamma_cmd, pitch_cmd) {
            (Some(gc), _) => 1.2 * (gc - gamma),
            (None, Some(pc)) => pc - theta,
            _ => 0.0,
        };
        // Pitch attitude limits.
        e_pitch = e_pitch.min(g.pitch_up - theta).max(g.pitch_down - theta);
        if fbw {
            let comp = 1.0 / phi.cos().max(0.5);
            let nz_neutral = gamma.cos() * comp;
            // Demand a flight-path rate proportional to the error.
            let dnz = 0.6 * e_pitch * tas / GRAVITY - 0.3 * q * tas / GRAVITY;
            let range = if dnz >= 0.0 { (def.nz_max - nz_neutral).max(0.3) } else { (nz_neutral - def.nz_min).max(0.3) };
            let mut x = (dnz / range).clamp(-1.0, 1.0);
            if def.control_law == ControlLaw::FastJet {
                // Undo the fighter law's progressive stick shaping s·(0.35 + 0.65 s²).
                let target = x;
                for _ in 0..4 {
                    let f = x * (0.35 + 0.65 * x * x) - target;
                    x -= f / (0.35 + 1.95 * x * x);
                }
                x = x.clamp(-1.0, 1.0);
            }
            ctl.elevator = x;
        } else {
            // Turn compensation: a little extra up elevator in a bank.
            let turn = (1.0 / phi.cos().max(0.6) - 1.0) * 0.6;
            let e_int = e_pitch + 0.15 * turn;
            self.pitch_i = (self.pitch_i + g.ki * g.kp * e_int * dt).clamp(-def.trim_authority, def.trim_authority);
            let pd = (g.kp * e_int - g.kd * q).clamp(-0.6, 0.6);
            ctl.trim = (self.pitch_i / def.trim_authority.max(1e-3)).clamp(-1.0, 1.0);
            ctl.elevator = pd;
        }

        // ---------------- lateral ----------------
        let bank_t = match self.hdg {
            Some(h) => {
                let e = wrap_pi(h - ac.heading());
                // Turn-rate demand (≤ 3°/s, eased in near the target), as bank.
                let rate = (0.25 * e).clamp(-3f32.to_radians(), 3f32.to_radians());
                (tas * rate / GRAVITY).atan().clamp(-g.bank_max, g.bank_max)
            }
            None => 0.0,
        };
        self.bank_cmd = approach(self.bank_cmd, bank_t, 5f32.to_radians() * dt);
        let p = ac.rates().x;
        if fbw {
            ctl.aileron = (1.5 * (self.bank_cmd - phi) / def.roll_rate_max.max(0.05)).clamp(-1.0, 1.0);
            ctl.rudder = 0.0;
        } else {
            let (ka, kr) = if def.category == Category::Airliner { (2.5, 1.5) } else { (2.0, 0.5) };
            ctl.aileron = (ka * (self.bank_cmd - phi) - kr * p).clamp(-1.0, 1.0);
            // Turn coordinator.
            ctl.rudder = (3.0 * ac.beta()).clamp(-1.0, 1.0);
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::{catalogue, find, trimmed_controls_at, Environment, FPM};
    use glam::{Vec2, Vec3};

    const DT: f32 = 1.0 / 120.0;

    struct Flat;
    impl Environment for Flat {
        fn ground_height(&self, _: Vec2) -> f32 {
            0.0
        }
        fn obstacle_top(&self, _: Vec2) -> Option<f32> {
            None
        }
        fn is_water(&self, _: Vec2) -> bool {
            false
        }
        fn updraft(&self, _: Vec3) -> f32 {
            0.0
        }
    }

    fn start(def: &AircraftDef) -> (f32, f32) {
        match def.category {
            Category::Trainer => (90.0, 1500.0),
            Category::Glider => (55.0, 1500.0),
            Category::Warbird => (200.0, 2000.0),
            Category::FastJet => (350.0, 3000.0),
            Category::Airliner => (if def.engine_kind == crate::flight::EngineKind::Turboprop { 180.0 } else { 250.0 }, 3000.0),
        }
    }

    #[derive(Default, Debug)]
    struct Trace {
        max_bank: f32,
        max_pitch: f32,
        min_pitch: f32,
        max_alt: f32,
        min_alt: f32,
        max_g: f32,
        min_g: f32,
        min_ias: f32,
    }

    fn fly(def: &AircraftDef, a: &mut Aircraft, ap: &mut Autopilot, ctl: &mut Controls, secs: f32) -> Trace {
        let mut t = Trace { min_pitch: 9.0, min_alt: 1e9, max_alt: -1e9, min_g: 9.0, min_ias: 1e9, ..Default::default() };
        for _ in 0..(secs / DT) as usize {
            assert!(ap.update(def, a, ctl, DT), "{} AP dropped out", def.id);
            a.step(def, ctl, &Flat, DT);
            assert!(a.crashed.is_none(), "{} crashed: {:?}", def.id, a.crashed);
            t.max_bank = t.max_bank.max(a.roll().abs());
            t.max_pitch = t.max_pitch.max(a.pitch());
            t.min_pitch = t.min_pitch.min(a.pitch());
            t.max_alt = t.max_alt.max(a.pos.y);
            t.min_alt = t.min_alt.min(a.pos.y);
            t.max_g = t.max_g.max(a.g_load());
            t.min_g = t.min_g.min(a.g_load());
            t.min_ias = t.min_ias.min(a.airspeed());
        }
        t
    }

    fn hdg_err(a: &Aircraft, h: f32) -> f32 {
        wrap_pi(a.heading() - h).abs().to_degrees()
    }

    /// Engage at cruise, then a combined climb + turn + speed change, then a
    /// combined descent + turn back + slow-down.
    #[test]
    fn captures_and_holds_alt_hdg_speed() {
        for def in catalogue().into_iter().filter(|d| d.has_engine) {
            let (kt, alt0) = start(&def);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt0, 0.0), 0.3, kt * KT);
            let mut ctl = trimmed_controls_at(&def, kt * KT, alt0);
            ctl.gear_down = a.gear_pos() > 0.5;
            let mut ap = Autopilot::new();
            ap.engage_hold_current(&a);
            assert!(ap.engaged);
            // Settle 10 s holding the present state.
            let t0 = fly(&def, &mut a, &mut ap, &mut ctl, 10.0);
            assert!((t0.max_alt - alt0).abs() < 3.0 && (t0.min_alt - alt0).abs() < 3.0, "{} hold {:?}", def.id, t0);

            let bank_lim = if def.category == Category::Airliner { 25.0 } else { 30.0 };
            let dh = if def.category == Category::FastJet { 1500.0 } else { 300.0 };
            // The trainer climbs slower and descends faster; the others speed up
            // in the climb and slow down in the descent.
            let legs = match def.category {
                Category::Trainer => [(dh, 1.0f32, -10.0), (-2.0 * dh, -1.0, 15.0)],
                Category::FastJet => [(dh, 1.0, 40.0), (-2.0 * dh, -1.0, -40.0)],
                _ => [(dh, 1.0, 15.0), (-2.0 * dh, -1.0, -15.0)],
            };
            let mut h = a.heading();
            let mut alt = alt0;
            let mut v = kt;
            for (i, (d_alt, turn, d_kt)) in legs.into_iter().enumerate() {
                h = wrap_pi(h + turn * 90f32.to_radians());
                alt += d_alt;
                v += d_kt;
                ap.hdg = Some(h);
                ap.alt = Some(alt);
                ap.ias = Some(v * KT);
                let secs = 260.0;
                let tr = fly(&def, &mut a, &mut ap, &mut ctl, secs);
                let overshoot = if d_alt > 0.0 { tr.max_alt - alt } else { alt - tr.min_alt };
                eprintln!(
                    "{} leg {i}: alt {:.1} m (target {:.0}, overshoot {:.1}), hdg err {:.2}°, ias {:.1} kt (target {:.0}), vs {:.2} m/s, \
                     max bank {:.1}°, pitch {:.1}..{:.1}°, g {:.2}..{:.2}, thr {:.2}",
                    def.id,
                    a.pos.y,
                    alt,
                    overshoot,
                    hdg_err(&a, h),
                    a.airspeed_kt(),
                    v,
                    a.vertical_speed(),
                    tr.max_bank.to_degrees(),
                    tr.min_pitch.to_degrees(),
                    tr.max_pitch.to_degrees(),
                    tr.min_g,
                    tr.max_g,
                    ctl.throttle
                );
                assert!((a.pos.y - alt).abs() < 6.0, "{} leg {i} alt {}", def.id, a.pos.y);
                assert!(overshoot < 20.0, "{} leg {i} overshoot {overshoot}", def.id);
                assert!(hdg_err(&a, h) < 1.0, "{} leg {i} heading", def.id);
                assert!((a.airspeed_kt() - v).abs() < 3.0, "{} leg {i} speed {}", def.id, a.airspeed_kt());
                assert!(a.vertical_speed().abs() < 0.6);
                assert!(tr.max_bank.to_degrees() < bank_lim + 2.0, "{} bank {}", def.id, tr.max_bank.to_degrees());
                assert!(tr.max_g < 1.6 && tr.min_g > 0.5, "{} g {:?}", def.id, tr);
            }
        }
    }

    #[test]
    fn vertical_speed_mode() {
        for id in ["kestrel", "aerolink", "meridian320", "meridian190", "lancer", "peregrine"] {
            let def = find(id).unwrap();
            let (kt, alt0) = start(&def);
            let alt0 = alt0 + if def.category == Category::Trainer { 300.0 } else { 1500.0 };
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt0, 0.0), 0.0, kt * KT);
            let mut ctl = trimmed_controls_at(&def, kt * KT, alt0);
            ctl.gear_down = a.gear_pos() > 0.5;
            let mut ap = Autopilot::new();
            ap.engage_hold_current(&a);
            ap.alt = None;
            let vs = -1000.0 * FPM;
            ap.vs = Some(vs);
            fly(&def, &mut a, &mut ap, &mut ctl, 40.0);
            let mut worst = 0.0f32;
            for _ in 0..(20.0 / DT) as usize {
                ap.update(&def, &a, &mut ctl, DT);
                a.step(&def, &ctl, &Flat, DT);
                worst = worst.max((a.vertical_speed() - vs).abs());
            }
            eprintln!("{id} V/S -1000 fpm: worst error {:.0} fpm, ias {:.0} kt", worst / FPM, a.airspeed_kt());
            assert!(worst < 100.0 * FPM, "{id} vs error {}", worst / FPM);
            assert!((a.airspeed_kt() - kt).abs() < 4.0, "{id} speed {}", a.airspeed_kt());
            // Altitude preselect with a commanded rate: climbs at that rate, captures.
            let rate = match def.engine_kind {
                _ if def.category == Category::Trainer => 200.0 * FPM,
                crate::flight::EngineKind::Turboprop => 400.0 * FPM,
                _ => 800.0 * FPM,
            };
            let target = a.pos.y + rate * 40.0;
            ap.alt = Some(target);
            ap.vs = Some(rate);
            // (From -1,000 fpm the rate change takes up to ~30 s at the airliners' 0.035 g.)
            let target = target + rate * 10.0;
            ap.alt = Some(target);
            fly(&def, &mut a, &mut ap, &mut ctl, 35.0);
            assert!((a.vertical_speed() - rate).abs() < 100.0 * FPM, "{id} preselect vs {}", a.vertical_speed() / FPM);
            let tr = fly(&def, &mut a, &mut ap, &mut ctl, 90.0);
            assert!((a.pos.y - target).abs() < 4.0 && tr.max_alt < target + 10.0, "{id} capture {} max {}", a.pos.y, tr.max_alt);
        }
    }

    #[test]
    fn recovers_from_upset_and_holds() {
        for def in catalogue().into_iter().filter(|d| d.has_engine) {
            let (kt, alt0) = start(&def);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt0, 0.0), 1.0, kt * KT);
            let mut ctl = trimmed_controls_at(&def, kt * KT, alt0);
            ctl.gear_down = a.gear_pos() > 0.5;
            let mut ap = Autopilot::new();
            ap.engage_hold_current(&a);
            // Gust: roll 20° and pitch up.
            a.rot = crate::flight::attitude(1.0, a.pitch() + 0.08, 0.35);
            a.omega = Vec3::new(-0.05, 0.0, 0.1);
            let tr = fly(&def, &mut a, &mut ap, &mut ctl, 90.0);
            eprintln!(
                "{} upset: alt band {:.1}..{:.1} (from {alt0}), now {:.1}, hdg err {:.2}°, bank {:.2}°, ias {:.1}",
                def.id,
                tr.min_alt,
                tr.max_alt,
                a.pos.y,
                hdg_err(&a, 1.0),
                a.roll().to_degrees(),
                a.airspeed_kt()
            );
            assert!((a.pos.y - alt0).abs() < 5.0, "{} alt", def.id);
            assert!(hdg_err(&a, 1.0) < 1.0);
            assert!(a.roll().abs() < 1f32.to_radians());
            assert!((a.airspeed_kt() - kt).abs() < 3.0, "{} ias {}", def.id, a.airspeed_kt());
        }
    }

    #[test]
    fn glider_flies_speed_on_pitch() {
        let def = find("swift").unwrap();
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, 55.0 * KT);
        let mut ctl = trimmed_controls_at(&def, 55.0 * KT, 1500.0);
        let mut ap = Autopilot::new();
        ap.engage_hold_current(&a);
        ap.ias = Some(70.0 * KT);
        ap.hdg = Some(1.5);
        let tr = fly(&def, &mut a, &mut ap, &mut ctl, 90.0);
        eprintln!("swift AP: {:.1} kt, hdg err {:.2}°, bank max {:.1}°", a.airspeed_kt(), hdg_err(&a, 1.5), tr.max_bank.to_degrees());
        assert!((a.airspeed_kt() - 70.0).abs() < 2.0);
        assert!(hdg_err(&a, 1.5) < 1.0);
        assert_eq!(ctl.throttle, 0.0);
    }

    #[test]
    fn speed_priority_when_thrust_limited() {
        // Ask the Kestrel for a big climb at a speed it can only just hold level
        // in: it must give up climb rate, not airspeed.
        let def = find("kestrel").unwrap();
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, 100.0 * KT);
        let mut ctl = trimmed_controls_at(&def, 100.0 * KT, 1500.0);
        let mut ap = Autopilot::new();
        ap.engage_hold_current(&a);
        ap.ias = Some(105.0 * KT);
        ap.alt = Some(2500.0);
        let tr = fly(&def, &mut a, &mut ap, &mut ctl, 120.0);
        eprintln!("kestrel thrust-limited climb: min ias {:.1} kt, alt {:.0}", tr.min_ias / KT, a.pos.y);
        assert!(tr.min_ias > 97.0 * KT);
        // And a large climb without autothrottle keeps above 1.3 Vs.
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, 75.0 * KT);
        let mut ctl = trimmed_controls_at(&def, 75.0 * KT, 1500.0);
        ap.engage_hold_current(&a);
        ap.ias = None;
        ap.alt = Some(2500.0);
        let tr = fly(&def, &mut a, &mut ap, &mut ctl, 120.0);
        eprintln!("kestrel climb without A/T: min ias {:.1} kt", tr.min_ias / KT);
        assert!(tr.min_ias > 1.25 * def.stall_speed(0.0));
    }

    #[test]
    fn disengages_on_ground_and_when_off() {
        let def = find("meridian320").unwrap();
        let a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &Flat);
        let mut ap = Autopilot::new();
        let mut ctl = Controls::default();
        assert!(!ap.update(&def, &a, &mut ctl, DT));
        ap.engage_hold_current(&a);
        assert!(!ap.update(&def, &a, &mut ctl, DT));
        assert!(!ap.engaged);
        assert_eq!(ctl, Controls::default());
    }
}
