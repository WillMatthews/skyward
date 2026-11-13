//! Cabin pressurisation (Track M): the cabin altitude schedule from engine /
//! APU bleed, slow leaks without bleed, rapid decompression through a hole
//! (the cabin follows the outside pressure with the hole's time constant),
//! the passenger oxygen masks (auto-deploy above 14,000 ft cabin, chemical
//! generators for ~13 minutes), the crew's quick-donning masks, and hypoxia
//! exposure for the debrief.

use crate::flight::{AircraftDef, Category, EngineKind, isa};

const FT: f32 = 0.3048;
/// Cabin altitude warning (ECAM "CAB PR EXCESS CAB ALT"), m.
pub const WARN_ALT: f32 = 9550.0 * FT;
/// Passenger masks drop above this cabin altitude, m.
pub const MASKS_ALT: f32 = 14_000.0 * FT;
/// Chemical oxygen generator duration, s.
pub const PAX_OXY_S: f32 = 13.0 * 60.0;
/// Maximum cabin altitude in normal operation (at the ceiling), m.
const MAX_CABIN: f32 = 8000.0 * FT;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cabin {
    /// Initialised for the aircraft (first systems step).
    pub ready: bool,
    /// Pressurised type.
    pub pressurised: bool,
    /// Cabin pressure altitude, m.
    pub alt: f32,
    /// A hole: the cabin equalises with this time constant (s).
    pub leak_tau: Option<f32>,
    /// Passenger masks out, and their oxygen left (s).
    pub pax_masks: bool,
    pub pax_oxy_s: f32,
    /// The crew have their masks on.
    pub crew_masks: bool,
    /// Seconds the warning has been on (the crew don masks within ~5 s).
    pub warn_t: f32,
    /// Passenger hypoxia exposure (s at cabin altitudes above 10,000 ft
    /// without oxygen, weighted by how high).
    pub pax_hypoxia: f32,
    /// Peak cabin altitude (m).
    pub peak: f32,
}

impl Default for Cabin {
    fn default() -> Self {
        Cabin { ready: false, pressurised: false, alt: 0.0, leak_tau: None, pax_masks: false, pax_oxy_s: 0.0, crew_masks: false, warn_t: 0.0, pax_hypoxia: 0.0, peak: 0.0 }
    }
}

/// Pressure altitude (m) of a static pressure (Pa), ISA (troposphere + isothermal layer).
pub fn pressure_alt(p: f32) -> f32 {
    let p = p.max(1.0);
    if p > 22_632.0 { 44_330.8 * (1.0 - (p / 101_325.0).powf(0.190_263)) } else { 11_000.0 + (22_632.0 / p).ln() * 6341.6 }
}

/// Does this type have a pressurised cabin?
pub fn pressurised(def: &AircraftDef) -> bool {
    def.has_engine && matches!(def.engine_kind, EngineKind::Jet | EngineKind::Turboprop) && matches!(def.category, Category::Airliner | Category::FastJet)
}

impl Cabin {
    pub fn new(def: &AircraftDef, alt: f32) -> Cabin {
        let pressurised = pressurised(def);
        let a = if pressurised { schedule(alt) } else { alt };
        Cabin { ready: true, pressurised, alt: a, peak: a, ..Default::default() }
    }

    /// The hole is open (rapid / slow decompression).
    pub fn decompress(&mut self, tau: f32) {
        self.leak_tau = Some(tau.max(0.5));
    }

    pub fn warning(&self) -> bool {
        self.pressurised && self.alt > WARN_ALT
    }

    /// One step: `alt` aircraft altitude (m), `bleed` engine / APU air available.
    pub fn step(&mut self, alt: f32, bleed: bool, dt: f32) {
        if !self.pressurised {
            self.alt = alt;
            return;
        }
        let p_out = isa(alt).pressure;
        let p_cab = isa(self.alt).pressure;
        let p_new = match self.leak_tau {
            // The hole: the packs cannot keep up, the cabin follows outside.
            Some(tau) => p_cab + (p_out - p_cab) * (1.0 - (-dt / tau).exp()),
            None if bleed => {
                // The controller flies the schedule at ≤ 500 ft/min of cabin rate (climb), 300 (descent).
                let target = isa(schedule(alt)).pressure;
                let rate = 600.0 * dt; // Pa per s ≈ 500 ft/min near sea level
                p_cab + (target - p_cab).clamp(-rate, rate * 0.6)
            }
            // No bleed: the outflow valve closes, the hull leaks slowly.
            None => p_cab + (p_out - p_cab) * (1.0 - (-dt / 600.0).exp()),
        };
        self.alt = pressure_alt(p_new).max(-300.0);
        if self.leak_tau.is_none() && self.alt > alt {
            self.alt = alt;
        }
        self.peak = self.peak.max(self.alt);
        // Warning, crew masks, passenger masks.
        if self.warning() {
            self.warn_t += dt;
            if self.warn_t > 4.0 {
                self.crew_masks = true;
            }
        } else {
            self.warn_t = 0.0;
        }
        if !self.pax_masks && self.alt > MASKS_ALT {
            self.pax_masks = true;
            self.pax_oxy_s = PAX_OXY_S;
        }
        if self.pax_masks {
            self.pax_oxy_s = (self.pax_oxy_s - dt).max(0.0);
        }
        // Without oxygen above 10,000 ft people start to suffer (worse higher).
        let over = (self.alt - 10_000.0 * FT) / (5000.0 * FT);
        if over > 0.0 && !(self.pax_masks && self.pax_oxy_s > 0.0) {
            self.pax_hypoxia += dt * over.min(4.0);
        }
    }
}

/// Normal cabin altitude for an aircraft altitude (m): about 8,000 ft at the ceiling.
pub fn schedule(alt: f32) -> f32 {
    (alt * 0.2).clamp(0.0, MAX_CABIN).min(alt.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pressure_alt_inverts_isa() {
        for h in [0.0f32, 1000.0, 3000.0, 8000.0, 11_000.0, 12_500.0] {
            let p = isa(h).pressure;
            assert!((pressure_alt(p) - h).abs() < 15.0, "{h}: {}", pressure_alt(p));
        }
    }

    #[test]
    fn decompression_equalises_and_drops_masks() {
        let def = crate::flight::find("meridian320").unwrap();
        let alt = 37_000.0 * FT;
        let mut c = Cabin::new(&def, alt);
        // Normal: about 7,400 ft cabin at FL370.
        assert!(c.alt < 8000.0 * FT && c.alt > 6000.0 * FT, "{}", c.alt / FT);
        for _ in 0..600 {
            c.step(alt, true, 0.05);
        }
        assert!(!c.warning() && !c.pax_masks);
        c.decompress(3.0);
        let mut t = 0.0;
        while !c.pax_masks && t < 30.0 {
            c.step(alt, true, 0.05);
            t += 0.05;
        }
        assert!(c.pax_masks && t < 6.0, "masks after {t} s");
        for _ in 0..400 {
            c.step(alt, true, 0.05);
        }
        assert!((c.alt - alt).abs() < 400.0, "cabin {} ft", c.alt / FT);
        assert!(c.crew_masks && c.warning());
        // Descending below 10,000 ft clears the warning (the cabin follows outside).
        for _ in 0..2000 {
            c.step(9000.0 * FT, true, 0.05);
        }
        assert!(!c.warning());
    }
}
