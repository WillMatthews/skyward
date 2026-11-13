//! Mission scoring (0..1000): survival 400, landing / ditching / handling
//! 300, procedures 200, time 100. Grades and medals from the total.

/// The touchdown, measured in the landing runway's frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Landing {
    pub sink_fpm: f32,
    /// Distance off the centreline (m) at touchdown.
    pub side_m: f32,
    /// Touchdown point past the threshold (m).
    pub along_m: f32,
    pub runway_len: f32,
    pub runway_width: f32,
    /// IAS − Vref at touchdown (kt).
    pub speed_dev_kt: f32,
    /// Peak load factor in the first second on the ground.
    pub g: f32,
    pub bank_deg: f32,
    /// Came to a stop on the paved runway (no overrun / excursion).
    pub stopped_on_runway: bool,
}

/// How the aircraft and its occupants came out of it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Survival {
    /// Hull loss, fatalities.
    Destroyed,
    /// Hull loss, survivable (ditching, crash landing).
    HullLoss,
    /// Damaged (belly landing, burst tyres) — everyone walks away.
    Damaged,
    /// No damage.
    #[default]
    Intact,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Inputs {
    pub success: bool,
    pub survival: Survival,
    /// Injured fraction of the occupants 0..1 (g, hypoxia, smoke).
    pub injured: f32,
    pub landing: Option<Landing>,
    /// Ditching quality 0..1 (ditching.rs).
    pub ditch: Option<f32>,
    /// Handling score 0..1 for missions that end in the air (stalls, overspeeds, g).
    pub handling: Option<f32>,
    pub proc_points: f32,
    pub proc_max: f32,
    pub time_s: f32,
    pub par_s: f32,
    /// Max touchdown sink rate for the type (fpm) — the gear's limit.
    pub max_sink_fpm: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Score {
    pub survival: f32,
    pub landing: f32,
    pub procedures: f32,
    pub time: f32,
    pub total: f32,
    /// Landing factors (name, 0..1) for the debrief.
    pub factors: Vec<(&'static str, f32)>,
    pub grade: &'static str,
    pub medal: Medal,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Medal {
    #[default]
    None,
    Bronze,
    Silver,
    Gold,
}

impl Medal {
    pub fn name(self) -> &'static str {
        match self {
            Medal::None => "",
            Medal::Bronze => "Bronze",
            Medal::Silver => "Silver",
            Medal::Gold => "Gold",
        }
    }
    pub fn from_name(s: &str) -> Medal {
        match s {
            "Gold" => Medal::Gold,
            "Silver" => Medal::Silver,
            "Bronze" => Medal::Bronze,
            _ => Medal::None,
        }
    }
}

/// 1 at or better than `good`, 0 at or beyond `bad` (either direction), linear between.
pub fn ramp(x: f32, good: f32, bad: f32) -> f32 {
    if (bad - good).abs() < 1e-6 {
        return if x <= good { 1.0 } else { 0.0 };
    }
    (1.0 - (x - good) / (bad - good)).clamp(0.0, 1.0)
}

/// Landing quality factors (each 0..1) and the weighted result.
pub fn landing_factors(l: &Landing, max_sink_fpm: f32) -> (f32, Vec<(&'static str, f32)>) {
    let sink = ramp(l.sink_fpm, 240.0, max_sink_fpm.max(400.0));
    let centre = ramp(l.side_m.abs(), 2.0, (l.runway_width * 0.5).max(10.0));
    // Touchdown zone: 150–600 m past the threshold (within the first third).
    let zone_end = (l.runway_len / 3.0).clamp(450.0, 900.0);
    let zone = if l.along_m < 0.0 {
        0.0
    } else if l.along_m < 150.0 {
        ramp(150.0 - l.along_m, 0.0, 150.0) * 0.5 + 0.5
    } else {
        ramp(l.along_m, zone_end, zone_end + 600.0)
    };
    let speed = ramp(l.speed_dev_kt.abs(), 5.0, 25.0);
    let g = ramp(l.g, 1.35, 2.2);
    let bank = ramp(l.bank_deg, 2.0, 8.0);
    let f = vec![("Sink rate", sink), ("Centreline", centre), ("Touchdown zone", zone), ("Speed", speed), ("Load factor", g), ("Wings level", bank)];
    let w = [0.25, 0.2, 0.2, 0.15, 0.1, 0.1];
    let mut q: f32 = f.iter().zip(w).map(|((_, v), w)| v * w).sum();
    if !l.stopped_on_runway {
        q *= 0.5;
    }
    (q, f)
}

pub fn score(i: &Inputs) -> Score {
    let base = match i.survival {
        Survival::Intact => 400.0,
        Survival::Damaged => 340.0,
        Survival::HullLoss => 280.0,
        Survival::Destroyed => 0.0,
    };
    let survival = if i.survival == Survival::Destroyed { 0.0 } else { base * (1.0 - 0.5 * i.injured.clamp(0.0, 1.0)) };
    let mut factors = Vec::new();
    let landing = if let Some(l) = &i.landing {
        let (q, f) = landing_factors(l, i.max_sink_fpm);
        factors = f;
        300.0 * q
    } else if let Some(d) = i.ditch {
        factors.push(("Ditching", d));
        300.0 * d
    } else if let Some(h) = i.handling {
        factors.push(("Handling", h));
        300.0 * h
    } else {
        0.0
    };
    let procedures = if i.proc_max > 0.0 {
        200.0 * (i.proc_points / i.proc_max).clamp(0.0, 1.0)
    } else if i.success {
        200.0
    } else {
        0.0
    };
    let time = if i.success && i.time_s > 0.0 { 100.0 * (i.par_s / i.time_s).clamp(0.0, 1.0) } else { 0.0 };
    let mut total = survival + landing + procedures + time;
    if !i.success {
        total = total.min(399.0);
    }
    let total = total.round();
    let grade = match total as i32 {
        900.. => "A+",
        800.. => "A",
        700.. => "B",
        550.. => "C",
        400.. => "D",
        _ => "F",
    };
    let medal = if !i.success {
        Medal::None
    } else if total >= 850.0 {
        Medal::Gold
    } else if total >= 700.0 {
        Medal::Silver
    } else {
        Medal::Bronze
    };
    Score { survival: survival.round(), landing: landing.round(), procedures: procedures.round(), time: time.round(), total, factors, grade, medal }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good() -> Landing {
        Landing { sink_fpm: 180.0, side_m: 1.0, along_m: 400.0, runway_len: 3000.0, runway_width: 45.0, speed_dev_kt: 3.0, g: 1.2, bank_deg: 1.0, stopped_on_runway: true }
    }

    #[test]
    fn ramps() {
        assert_eq!(ramp(100.0, 240.0, 600.0), 1.0);
        assert_eq!(ramp(600.0, 240.0, 600.0), 0.0);
        assert!((ramp(420.0, 240.0, 600.0) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn perfect_landing_is_gold() {
        let i = Inputs {
            success: true,
            survival: Survival::Intact,
            landing: Some(good()),
            proc_points: 100.0,
            proc_max: 100.0,
            time_s: 300.0,
            par_s: 300.0,
            max_sink_fpm: 600.0,
            ..Default::default()
        };
        let s = score(&i);
        assert_eq!(s.total, 1000.0);
        assert_eq!((s.grade, s.medal), ("A+", Medal::Gold));
    }

    #[test]
    fn landing_factors_penalise() {
        let (q0, _) = landing_factors(&good(), 600.0);
        let hard = Landing { sink_fpm: 520.0, ..good() };
        let off = Landing { side_m: 15.0, ..good() };
        let long = Landing { along_m: 1500.0, ..good() };
        let overrun = Landing { stopped_on_runway: false, ..good() };
        for (name, l) in [("hard", hard), ("off", off), ("long", long), ("overrun", overrun)] {
            let (q, _) = landing_factors(&l, 600.0);
            assert!(q < q0 - 0.05, "{name}: {q} vs {q0}");
        }
        let (q, _) = landing_factors(&overrun, 600.0);
        assert!((q - q0 * 0.5).abs() < 1e-5);
    }

    #[test]
    fn failure_caps_and_no_medal() {
        let i = Inputs { success: false, survival: Survival::Intact, handling: Some(1.0), proc_points: 1.0, proc_max: 1.0, ..Default::default() };
        let s = score(&i);
        assert!(s.total <= 399.0 && s.medal == Medal::None && s.grade == "F");
        // Destroyed scores nothing for survival.
        let d = score(&Inputs { success: false, survival: Survival::Destroyed, ..Default::default() });
        assert_eq!(d.survival, 0.0);
    }

    #[test]
    fn survival_ordering() {
        let s = |v| score(&Inputs { success: true, survival: v, ..Default::default() }).survival;
        assert!(s(Survival::Intact) > s(Survival::Damaged) && s(Survival::Damaged) > s(Survival::HullLoss) && s(Survival::HullLoss) > s(Survival::Destroyed));
    }
}
