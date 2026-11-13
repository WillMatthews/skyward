//! Ditching: whether a water landing is survivable, from the state at first
//! contact. Follows the airliner ditching guidance (FCOM-style "DITCHING"
//! procedure): wings level, nose up about 11° (tail touches first, the
//! fuselage planes), sink rate below ~500 ft/min, the lowest practical speed
//! (just above the stall). Nose-low entries dig in and break the fuselage; a
//! wing tip catching the water cartwheels it; a high sink rate breaks it.
//!
//! The flight model reports "Ditched in the water" at the first contact; the
//! session asks [`classify`] (in place of `damage::classify`) how bad it was,
//! and the wreck then floats on the debris model (buoyancy, water drag,
//! flooding).

use crate::damage::Severity;
use crate::flight::{AircraftDef, FPM, Impact, KT};

/// Ditching quality at first contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Assessment {
    pub severity: Severity,
    pub pitch_deg: f32,
    pub bank_deg: f32,
    pub sink_fpm: f32,
    pub speed_kt: f32,
    /// 0..1 how close to the textbook ditching.
    pub quality: f32,
    /// One-line verdict for the debrief.
    pub verdict: &'static str,
}

/// Attitude (pitch, bank) in radians from a body→world rotation.
fn attitude(rot: glam::Quat) -> (f32, f32) {
    let f = rot * glam::Vec3::Z;
    let r = rot * glam::Vec3::NEG_X;
    let u = rot * glam::Vec3::Y;
    (f.y.clamp(-1.0, 1.0).asin(), (-r.y).atan2(u.y))
}

pub fn assess(def: &AircraftDef, imp: &Impact) -> Assessment {
    let (pitch, bank) = attitude(imp.rot);
    let pitch_deg = pitch.to_degrees();
    let bank_deg = bank.to_degrees().abs();
    let sink = (-imp.vel.y).max(0.0);
    let sink_fpm = sink / FPM;
    let speed_kt = glam::Vec2::new(imp.vel.x, imp.vel.z).length() / KT;
    let vref = if def.vref_kt > 0.0 { def.vref_kt } else { def.vs1_kt * 1.3 }.max(40.0);
    let (severity, verdict) = if bank_deg > 12.0 {
        (Severity::Destroyed, "A wing tip caught the water and the aircraft cartwheeled")
    } else if pitch_deg < 2.0 && speed_kt > 0.85 * vref {
        (Severity::Destroyed, "Nose-first into the water: the fuselage dug in and broke up")
    } else if sink_fpm > 1500.0 {
        (Severity::Destroyed, "Hit the water too hard: the fuselage broke up")
    } else if speed_kt > 1.45 * vref {
        (Severity::Destroyed, "Far too fast: the aircraft broke up on the water")
    } else if pitch_deg > 17.0 {
        (Severity::Substantial, "Tail-first: the aft fuselage tore open, but she floats")
    } else {
        (Severity::Substantial, "A survivable ditching — the aircraft is afloat")
    };
    // Quality: pitch near 9–12°, sink under 500 fpm, wings level, slow.
    let p = 1.0 - ((pitch_deg - 10.5).abs() - 2.0).max(0.0) / 8.0;
    let s = 1.0 - ((sink_fpm - 300.0).max(0.0) / 1000.0);
    let b = 1.0 - (bank_deg / 10.0);
    let v = 1.0 - ((speed_kt / vref - 0.95).max(0.0) / 0.5);
    let quality =
        if severity == Severity::Destroyed { 0.0 } else { (p.clamp(0.0, 1.0) * 0.3 + s.clamp(0.0, 1.0) * 0.3 + b.clamp(0.0, 1.0) * 0.2 + v.clamp(0.0, 1.0) * 0.2).clamp(0.0, 1.0) };
    Assessment { severity, pitch_deg, bank_deg, sink_fpm, speed_kt, quality, verdict }
}

/// Severity of a crash: ditchings by [`assess`], everything else as before.
pub fn classify(def: &AircraftDef, reason: &str, imp: &Impact, structural_failure: bool) -> Severity {
    if reason.contains("Ditched") && !structural_failure { assess(def, imp).severity } else { crate::damage::classify(def, reason, imp.vel, structural_failure) }
}
