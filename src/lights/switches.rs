//! Which exterior lights are on: the automatic, phase-of-flight state (as a
//! crew would set the switches) under any manual overrides.
//!
//! | Phase | nav | beacon | taxi | strobe | landing |
//! |---|---|---|---|---|---|
//! | cold and dark (no power) | - | - | - | - | - |
//! | parked, engines off | on | - | - | - | - |
//! | pushback / engines running | on | on | - | - | - |
//! | taxiing | on | on | on | - | - |
//! | on the runway (lined up, rolling) | on | on | on | on | on |
//! | airborne below 10,000 ft | on | on | - | on | on |
//! | airborne above 10,000 ft | on | on | - | on | - |

/// Above this (MSL) the landing lights go off (the 10,000 ft convention).
pub const LANDING_OFF_FT: f32 = 10_000.0;
/// Ground speed (m/s) above which the aircraft counts as taxiing.
const TAXI_GS: f32 = 0.5;
/// Ground speed (m/s) of a take-off or landing roll (~30 kt).
const ROLL_GS: f32 = 15.0;

/// What the light logic needs to know about the aircraft.
#[derive(Clone, Copy, Debug, Default)]
pub struct Phase {
    /// Electrical power on (not cold and dark).
    pub powered: bool,
    /// The type has an engine (gliders: no beacon / strobes / landing lights).
    pub has_engine: bool,
    /// Any engine running.
    pub running: bool,
    pub on_ground: bool,
    /// Ground speed (m/s).
    pub gs: f32,
    pub throttle: f32,
    pub parking_brake: bool,
    /// Being pushed back.
    pub pushback: bool,
    /// On the runway (lined up or rolling).
    pub on_runway: bool,
    /// Altitude above mean sea level (ft).
    pub alt_ft: f32,
}

/// The exterior lights' state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExtLights {
    pub nav: bool,
    pub beacon: bool,
    pub strobe: bool,
    pub landing: bool,
    pub taxi: bool,
}

impl ExtLights {
    /// The automatic state for a phase of flight.
    pub fn auto(p: &Phase) -> ExtLights {
        if !p.powered {
            return ExtLights::default();
        }
        let nav = true;
        if !p.has_engine {
            // Gliders: navigation lights only.
            return ExtLights { nav, ..Default::default() };
        }
        let beacon = p.running || p.pushback;
        if p.on_ground {
            let rolling = p.gs > ROLL_GS || (p.running && p.throttle > 0.5 && !p.parking_brake);
            let runway = p.on_runway || rolling;
            let taxiing = p.running && !p.pushback && (p.gs > TAXI_GS || !p.parking_brake);
            ExtLights { nav, beacon, strobe: runway && p.running, landing: runway && p.running, taxi: taxiing || (runway && p.running) }
        } else {
            ExtLights { nav, beacon, strobe: true, landing: p.alt_ft < LANDING_OFF_FT, taxi: false }
        }
    }
}

/// The pilot's switches: `None` = automatic.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LightSwitches {
    pub nav: Option<bool>,
    pub beacon: Option<bool>,
    pub strobe: Option<bool>,
    pub landing: Option<bool>,
    pub taxi: Option<bool>,
}

impl LightSwitches {
    /// The lights as shown: each switch the pilot set, else the automatic state.
    pub fn resolve(&self, p: &Phase) -> ExtLights {
        let a = ExtLights::auto(p);
        ExtLights {
            nav: self.nav.unwrap_or(a.nav),
            beacon: self.beacon.unwrap_or(a.beacon),
            strobe: self.strobe.unwrap_or(a.strobe),
            landing: self.landing.unwrap_or(a.landing),
            taxi: self.taxi.unwrap_or(a.taxi),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parked() -> Phase {
        Phase { powered: true, has_engine: true, running: false, on_ground: true, gs: 0.0, throttle: 0.0, parking_brake: true, pushback: false, on_runway: false, alt_ft: 80.0 }
    }

    #[test]
    fn cold_and_dark_is_dark() {
        let p = Phase { powered: false, ..parked() };
        assert_eq!(ExtLights::auto(&p), ExtLights::default());
    }

    #[test]
    fn parked_nav_only() {
        assert_eq!(ExtLights::auto(&parked()), ExtLights { nav: true, ..Default::default() });
    }

    #[test]
    fn at_the_gate_with_engines_running_beacon_only() {
        let p = Phase { running: true, ..parked() };
        assert_eq!(ExtLights::auto(&p), ExtLights { nav: true, beacon: true, ..Default::default() });
    }

    #[test]
    fn pushback_beacon_no_taxi_light() {
        let p = Phase { pushback: true, parking_brake: false, gs: 1.0, ..parked() };
        let l = ExtLights::auto(&p);
        assert!(l.beacon && !l.taxi && !l.strobe && !l.landing);
    }

    #[test]
    fn taxiing_taxi_light_no_strobes() {
        let p = Phase { running: true, parking_brake: false, gs: 7.0, throttle: 0.25, ..parked() };
        assert_eq!(ExtLights::auto(&p), ExtLights { nav: true, beacon: true, taxi: true, ..Default::default() });
    }

    #[test]
    fn runway_everything() {
        let lined_up = Phase { running: true, parking_brake: false, on_runway: true, ..parked() };
        let rolling = Phase { running: true, parking_brake: false, gs: 40.0, throttle: 0.9, ..parked() };
        for p in [lined_up, rolling] {
            let l = ExtLights::auto(&p);
            assert!(l.nav && l.beacon && l.strobe && l.landing && l.taxi, "{l:?}");
        }
    }

    #[test]
    fn landing_lights_off_above_10000_ft() {
        let climb = Phase { running: true, on_ground: false, parking_brake: false, gs: 140.0, throttle: 0.9, alt_ft: 6_000.0, ..parked() };
        let cruise = Phase { alt_ft: 11_000.0, ..climb };
        let a = ExtLights::auto(&climb);
        let b = ExtLights::auto(&cruise);
        assert!(a.landing && a.strobe && a.beacon && !a.taxi);
        assert!(!b.landing && b.strobe && b.beacon && b.nav);
    }

    #[test]
    fn glider_nav_only() {
        let p = Phase { has_engine: false, on_ground: false, alt_ft: 3_000.0, ..parked() };
        assert_eq!(ExtLights::auto(&p), ExtLights { nav: true, ..Default::default() });
    }

    #[test]
    fn manual_switches_win() {
        let cruise = Phase { running: true, on_ground: false, parking_brake: false, alt_ft: 35_000.0, ..parked() };
        let sw = LightSwitches { landing: Some(true), strobe: Some(false), ..Default::default() };
        let l = sw.resolve(&cruise);
        assert!(l.landing && !l.strobe && l.beacon && l.nav);
        let sw = LightSwitches { landing: Some(false), ..Default::default() };
        let low = Phase { alt_ft: 2_000.0, ..cruise };
        assert!(!sw.resolve(&low).landing);
    }
}
