//! Split thrust levers (differential thrust): which side each engine is on,
//! and the keyboard / axis glue. `Controls::engine_throttle` is `None` (one
//! lever for every engine) until the pilot moves one side on its own.

use crate::flight::{AircraftDef, Controls, MAX_ENGINES};

/// Side of engine `i`: +1 left wing, −1 right wing, 0 on the centreline
/// (body +X is the left wing).
pub fn side(def: &AircraftDef, i: usize) -> i8 {
    match def.engine_pos.get(i) {
        Some(p) if p.x > 0.5 => 1,
        Some(p) if p.x < -0.5 => -1,
        _ => 0,
    }
}

/// Can this type steer on differential thrust (engines on both wings)?
pub fn can_split(def: &AircraftDef) -> bool {
    let n = crate::missions::n_eng(def);
    def.has_engine && n > 1 && (0..n).any(|i| side(def, i) > 0) && (0..n).any(|i| side(def, i) < 0)
}

/// Keyboard: `common` moves every lever (keeping a split), `sides` = (left,
/// right) key directions −1..1 move one side's levers (rate 0.6 / s).
pub fn keys(def: &AircraftDef, ctl: &mut Controls, common: f32, sides: (f32, f32), dt: f32) {
    let n = crate::missions::n_eng(def).min(MAX_ENGINES);
    if let Some(t) = ctl.engine_throttle.as_mut() {
        for x in t.iter_mut().take(n) {
            *x = (*x + common).clamp(0.0, 1.0);
        }
    }
    if (sides.0 != 0.0 || sides.1 != 0.0) && can_split(def) {
        for i in 0..n {
            let d = match side(def, i) {
                1 => sides.0,
                -1 => sides.1,
                _ => 0.5 * (sides.0 + sides.1),
            };
            if d != 0.0 {
                ctl.nudge_lever(i, d * 0.6 * dt);
            }
        }
    }
    // The common lever shows the mean (displays, sound, the autothrottle hand-back).
    if let Some(t) = ctl.engine_throttle {
        ctl.throttle = t.iter().take(n.max(1)).sum::<f32>() / n.max(1) as f32;
    }
}

/// Set the left / right levers directly (bots, per-engine axes).
pub fn set_sides(def: &AircraftDef, ctl: &mut Controls, left: f32, right: f32) {
    let n = crate::missions::n_eng(def).min(MAX_ENGINES);
    let mut t = [0.0; MAX_ENGINES];
    for (i, x) in t.iter_mut().enumerate().take(n) {
        *x = match side(def, i) {
            1 => left,
            -1 => right,
            _ => 0.5 * (left + right),
        }
        .clamp(0.0, 1.0);
    }
    ctl.engine_throttle = Some(t);
    ctl.throttle = t.iter().take(n.max(1)).sum::<f32>() / n.max(1) as f32;
}
