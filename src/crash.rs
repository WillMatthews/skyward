//! Graded crashes (Track CR): damage is state, the end of the flight is a
//! terminal condition.
//!
//! A contact that hurts the airframe — a firm arrival, a wingtip or a
//! propeller touching, a gear leg folding, a wing through a building — adds
//! to the per-part damage (`damage::Damage::hit`) and the flight goes on with
//! whatever the physics leaves the pilot: less lift on one side, less
//! elevator, less power, no gear. The flight only ends when
//!
//! * the aircraft has come to rest on the ground wrecked (gear gone, on its
//!   belly, a wing off) — [`Wreck`],
//! * it went into the water (it decelerates hard, floats, settles),
//! * or an impact was beyond anything survivable ([`GearLimits::breakup`],
//!   the canopy into the ground, the fuselage into a building).
//!
//! Severity ([`Severity`]: Incident / Accident / Hull loss) and a cause
//! ([`CrashCause`]) go on the final crash report for the crash screen.

use crate::damage::Severity;
use crate::flight::{AircraftDef, Category, Impact, KT};

/// Sink-rate thresholds as multiples of the type's design touchdown sink
/// (`AircraftDef::max_touchdown_sink`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearLimits {
    /// Beyond this the gear is damaged (bent, "GEAR UNSAFE").
    pub damage: f32,
    /// Beyond this the gear collapses: it slides on its belly.
    pub collapse: f32,
    /// Beyond this the airframe breaks up: the flight ends.
    pub breakup: f32,
}

/// Gear limits by aircraft class. Light aircraft (spring-steel / bungee legs)
/// take a lot of overload before they fold; transport gear is designed to a
/// 10 ft/s limit with a 20 % reserve, and fails cleanly (fuse pins) not long
/// after; fast-jet gear is already rated for carrier-style arrivals.
pub fn gear_limits(def: &AircraftDef) -> GearLimits {
    let (damage, collapse, breakup) = match def.category {
        Category::Trainer => (1.0, 1.6, 2.8),
        Category::Glider => (1.0, 1.4, 2.2),
        Category::Warbird => (1.0, 1.45, 2.6),
        Category::FastJet => (1.0, 1.35, 2.4),
        Category::Airliner => (1.0, 1.4, 2.3),
        Category::Helicopter => (1.0, 1.6, 2.8),
        Category::Drone => (1.0, 1.0, 1.5),
    };
    GearLimits { damage, collapse, breakup }
}

/// What a touchdown at `sink` (m/s) does to the gear.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Touchdown {
    Normal,
    /// The gear is bent: damage 0..1 to each leg that took the load.
    Hard {
        gear: f32,
    },
    /// The gear legs fold: the aircraft slides on its belly.
    Collapse,
    /// The airframe broke up (terminal).
    BreakUp,
    /// Flown into the ground (terminal).
    Cfit,
}

/// Grade a touchdown by its sink rate.
pub fn touchdown(def: &AircraftDef, sink: f32) -> Touchdown {
    let ms = def.max_touchdown_sink.max(0.5);
    let l = gear_limits(def);
    let r = sink / ms;
    if !r.is_finite() || r <= l.damage {
        Touchdown::Normal
    } else if r > 3.0 {
        Touchdown::Cfit
    } else if r > l.breakup {
        Touchdown::BreakUp
    } else if r > l.collapse {
        Touchdown::Collapse
    } else {
        Touchdown::Hard { gear: (0.15 + 0.75 * (r - l.damage) / (l.collapse - l.damage).max(0.05)).min(0.9) }
    }
}

/// Is a strike on a non-gear point (wingtip, propeller, belly, tail) at a
/// normal (into-the-ground) speed `vn` fatal? `top`: a point on top of the
/// airframe (canopy / fin top: the aircraft is upside down), `along`: speed
/// along the ground there.
pub fn strike_fatal(def: &AircraftDef, vn: f32, along: f32, top: bool) -> bool {
    let ms = def.max_touchdown_sink.max(0.5);
    if top {
        return along > 12.0 || vn > ms;
    }
    vn > gear_limits(def).breakup * ms
}

/// Coulomb friction of metal sliding on a surface (aluminium skin on
/// concrete / asphalt, on grass and soil — where it also ploughs).
pub fn slide_mu(paved: bool) -> f32 {
    if paved { 0.38 } else { 0.55 }
}

/// Deceleration (m/s², + = slowing) of a body sliding on its belly with
/// friction `mu` on a slope `slope` (rise per metre along the direction of
/// travel, + = uphill).
pub fn slide_decel(mu: f32, slope: f32) -> f32 {
    let th = slope.atan();
    crate::flight::GRAVITY * (mu * th.cos() + th.sin())
}

/// Distance (m) to slide to a stop from `v` m/s.
pub fn slide_distance(v: f32, mu: f32, slope: f32) -> f32 {
    let a = slide_decel(mu, slope);
    if a <= 0.0 { f32::INFINITY } else { v * v / (2.0 * a) }
}

/// What the accident was, for the crash screen.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CrashCause {
    HardLanding,
    GearCollapse,
    GearUp,
    NoseOver,
    WingtipStrike,
    GroundLoop,
    PropStrike,
    TailStrike,
    /// Controlled flight into terrain.
    Cfit,
    /// Stalled / spinning into the ground.
    StallSpin,
    Ditching,
    Obstacle,
    InFlightBreakup,
    Rollover,
    #[default]
    Other,
}

impl CrashCause {
    pub fn label(self) -> &'static str {
        match self {
            CrashCause::HardLanding => "hard landing",
            CrashCause::GearCollapse => "gear collapse",
            CrashCause::GearUp => "gear-up landing",
            CrashCause::NoseOver => "nose-over",
            CrashCause::WingtipStrike => "wingtip strike",
            CrashCause::GroundLoop => "ground loop",
            CrashCause::PropStrike => "prop strike",
            CrashCause::TailStrike => "tail strike",
            CrashCause::Cfit => "CFIT",
            CrashCause::StallSpin => "stall/spin into terrain",
            CrashCause::Ditching => "ditching",
            CrashCause::Obstacle => "obstacle strike",
            CrashCause::InFlightBreakup => "in-flight break-up",
            CrashCause::Rollover => "dynamic rollover",
            CrashCause::Other => "accident",
        }
    }

    /// From the crash reason and the state at impact.
    pub fn infer(def: &AircraftDef, reason: &str, imp: &Impact) -> CrashCause {
        let r = reason.to_ascii_lowercase();
        let has = |k: &str| r.contains(k);
        if has("structural") || has("broke up in") || has("came apart") {
            return CrashCause::InFlightBreakup;
        }
        if has("water") || has("ditch") || has("capsiz") || has("float") {
            return CrashCause::Ditching;
        }
        if has("building") || has("obstacle") || has("tree") {
            return CrashCause::Obstacle;
        }
        if has("rollover") || has("turtled") {
            return CrashCause::Rollover;
        }
        // Into the ground out of control: stalled (high alpha) or spinning.
        let into_ground = has("flew into") || has("hit the ground") || has("inverted") || has("broke up");
        if into_ground && def.rotor.is_none() && (imp.alpha.abs() > 0.3 || imp.omega.y.abs() > 0.8) {
            return CrashCause::StallSpin;
        }
        if has("gear-up") || has("belly") {
            return CrashCause::GearUp;
        }
        if has("ground loop") {
            return CrashCause::GroundLoop;
        }
        if has("nose-gear") || has("nose gear") || has("nose-over") || has("nosed over") {
            return CrashCause::NoseOver;
        }
        if has("collapse") || has("skids") {
            return CrashCause::GearCollapse;
        }
        if has("hard landing") {
            return CrashCause::HardLanding;
        }
        if has("wingtip") || has("cartwheel") || has("wing ") {
            return CrashCause::WingtipStrike;
        }
        if has("prop") || has("pod strike") {
            return CrashCause::PropStrike;
        }
        if has("tail") {
            return CrashCause::TailStrike;
        }
        if into_ground {
            return CrashCause::Cfit;
        }
        CrashCause::Other
    }
}

/// The final crash event (for the crash screen, missions, logs).
#[derive(Clone, Debug, PartialEq)]
pub struct CrashReport {
    pub severity: Severity,
    pub cause: CrashCause,
    pub reason: String,
    /// Speed (kt) and sink (fpm) at the impact.
    pub speed_kt: f32,
    pub sink_fpm: f32,
}

impl CrashReport {
    pub fn new(def: &AircraftDef, reason: &str, imp: &Impact, severity: Severity) -> CrashReport {
        CrashReport {
            severity,
            cause: CrashCause::infer(def, reason, imp),
            reason: reason.to_string(),
            speed_kt: imp.vel.length() / KT,
            sink_fpm: (-imp.vel.y).max(0.0) / crate::flight::FPM,
        }
    }
    /// One line for the crash screen: "Accident · gear collapse".
    pub fn line(&self) -> String {
        format!("{} · {}", self.severity.label(), self.cause.label())
    }
}

/// A wrecked aircraft still moving on the ground: the flight goes on (the
/// pilot keeps the controls) until it has come to rest.
#[derive(Clone, Debug, Default)]
pub struct Wreck {
    /// What happened (becomes the crash reason).
    pub reason: String,
    /// Seconds at rest so far.
    pub still: f32,
    /// Distance slid (m).
    pub slid: f32,
}

impl Wreck {
    pub fn new(reason: String) -> Wreck {
        Wreck { reason, still: 0.0, slid: 0.0 }
    }
    /// Advance; true once it has been at rest long enough to end the flight.
    pub fn step(&mut self, ground_speed: f32, dt: f32) -> bool {
        self.slid += ground_speed * dt;
        if ground_speed < 0.8 {
            self.still += dt
        } else {
            self.still = 0.0
        }
        self.still > 1.5
    }
    /// The crash reason once at rest.
    pub fn final_reason(&self) -> String {
        if self.slid > 15.0 { format!("{} — slid {:.0} m to a stop", self.reason, self.slid) } else { self.reason.clone() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::{GRAVITY, catalogue, find};
    use glam::{Quat, Vec3};

    fn imp(vel: Vec3) -> Impact {
        Impact { pos: Vec3::ZERO, vel, rot: Quat::IDENTITY, omega: Vec3::ZERO, on_ground: true, alpha: 0.05 }
    }

    #[test]
    fn touchdown_grades_by_sink_and_class() {
        let cat = catalogue();
        let c172 = &find("kestrel").unwrap();
        let a320 = &find("meridian320").unwrap();
        for def in [c172, a320] {
            let ms = def.max_touchdown_sink;
            assert_eq!(touchdown(def, 0.5 * ms), Touchdown::Normal, "{}", def.id);
            assert!(matches!(touchdown(def, 1.1 * ms), Touchdown::Hard { gear } if gear > 0.0 && gear < 0.5), "{}", def.id);
            assert_eq!(touchdown(def, 1.7 * ms), Touchdown::Collapse, "{}", def.id);
            assert_eq!(touchdown(def, 2.9 * ms), Touchdown::BreakUp, "{}", def.id);
            assert_eq!(touchdown(def, 3.5 * ms), Touchdown::Cfit, "{}", def.id);
        }
        // Light spring-steel gear takes more overload than transport gear.
        let (l, t) = (gear_limits(c172), gear_limits(a320));
        assert!(l.collapse > t.collapse && l.breakup > t.breakup);
        // 1.5x the design sink: bent on a trainer, folded on an airliner.
        assert!(matches!(touchdown(c172, 1.5 * c172.max_touchdown_sink), Touchdown::Hard { .. }));
        assert_eq!(touchdown(a320, 1.5 * a320.max_touchdown_sink), Touchdown::Collapse);
        // Every type has ordered limits.
        for def in &cat {
            let l = gear_limits(def);
            assert!(l.damage <= l.collapse && l.collapse < l.breakup && l.breakup <= 3.0, "{}", def.id);
        }
    }

    #[test]
    fn slide_deceleration_and_distance() {
        // Flat concrete: ~0.38 g; grass more; uphill more, downhill less.
        let flat = slide_decel(slide_mu(true), 0.0);
        assert!((flat - 0.38 * GRAVITY).abs() < 1e-3);
        assert!(slide_decel(slide_mu(false), 0.0) > flat);
        assert!(slide_decel(slide_mu(true), 0.05) > flat);
        assert!(slide_decel(slide_mu(true), -0.05) < flat);
        // An airliner belly landing at 130 kt slides ~600 m on concrete
        // (real gear-up landings: 500-900 m).
        let d = slide_distance(130.0 * KT, slide_mu(true), 0.0);
        assert!((450.0..900.0).contains(&d), "{d}");
        // Steeper than the friction angle downhill: it never stops.
        assert!(slide_distance(30.0, 0.38, -0.6).is_infinite());
    }

    #[test]
    fn strikes_and_causes() {
        let def = &find("kestrel").unwrap();
        let ms = def.max_touchdown_sink;
        // A wingtip / prop touching at a gentle sink is survivable damage.
        assert!(!strike_fatal(def, 0.5 * ms, 30.0, false));
        assert!(strike_fatal(def, 3.0 * ms, 30.0, false));
        // Upside down into the ground at flying speed is not.
        assert!(strike_fatal(def, 0.0, 30.0, true));
        let c = |r: &str, v: Vec3| CrashCause::infer(def, r, &imp(v));
        assert_eq!(c("Ditched in the water", Vec3::Z * 30.0), CrashCause::Ditching);
        assert_eq!(c("Gear-up landing — slid 300 m to a stop", Vec3::Z), CrashCause::GearUp);
        assert_eq!(c("Hard landing: touched down at 900 fpm — landing gear collapsed", Vec3::Z), CrashCause::GearCollapse);
        assert_eq!(c("Flew into a building", Vec3::Z * 40.0), CrashCause::Obstacle);
        assert_eq!(c("Flew into the ground at 3000 fpm", Vec3::new(0.0, -15.0, 40.0)), CrashCause::Cfit);
        let mut spin = imp(Vec3::new(0.0, -20.0, 5.0));
        spin.alpha = 0.7;
        assert_eq!(CrashCause::infer(def, "Flew into the ground at 4000 fpm", &spin), CrashCause::StallSpin);
        let rep = CrashReport::new(def, "Gear-up landing", &imp(Vec3::Z * 30.0), Severity::Minor);
        assert_eq!(rep.line(), "Incident · gear-up landing");
        assert_eq!(Severity::Substantial.label(), "Accident");
        assert_eq!(Severity::Destroyed.label(), "Hull loss");
    }

    #[test]
    fn wreck_ends_only_at_rest() {
        let mut w = Wreck::new("Gear-up landing".into());
        let dt = 1.0 / 120.0;
        let mut v = 60.0f32;
        let mut t = 0.0;
        let decel = slide_decel(slide_mu(true), 0.0);
        let mut done = false;
        while t < 30.0 {
            v = (v - decel * dt).max(0.0);
            if w.step(v, dt) {
                done = true;
                break;
            }
            t += dt;
        }
        assert!(done);
        // It ended after it stopped (60 m/s / 3.7 m/s² ≈ 16 s), not before.
        assert!(t > 60.0 / decel, "{t}");
        assert!((w.slid - slide_distance(60.0, slide_mu(true), 0.0)).abs() < 10.0, "{}", w.slid);
        assert!(w.final_reason().contains("slid"));
    }
}
