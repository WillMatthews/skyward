//! Mission definitions: data-driven JSON (`assets/missions/*.json`, embedded),
//! parsed into [`Mission`] and validated against the aircraft catalogue and
//! the failure catalogue.
//!
//! A mission is: an aircraft (by catalogue id, so it can be retargeted), a
//! target runway (airport + runway end, with its geometry for headless tests),
//! a start placed in that runway's frame, conditions (time, date, METAR, fuel),
//! a timeline of events (trigger → actions), objectives and limits.
//! See `docs/MISSIONS.md`-style notes in `README.md` for the JSON keys.

use serde_json::Value;

/// A trigger: every condition given must hold (an empty trigger fires at once).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trigger {
    /// Mission time (s) at or after.
    pub t: Option<f32>,
    /// After event `id` fired (+ `delay` s).
    pub after: Option<String>,
    pub delay: f32,
    /// Altitude (ft above mean sea level) above / below.
    pub alt_above_ft: Option<f32>,
    pub alt_below_ft: Option<f32>,
    /// Height above the ground (ft) below.
    pub agl_below_ft: Option<f32>,
    /// Indicated airspeed (kt) above / below.
    pub ias_above_kt: Option<f32>,
    pub ias_below_kt: Option<f32>,
    /// Distance to the target threshold (NM) below.
    pub dist_below_nm: Option<f32>,
    /// On the ground (true) / airborne (false).
    pub on_ground: Option<bool>,
    /// A player action has happened: see [`PlayerAct`].
    pub player: Option<PlayerAct>,
}

/// Things the player does that triggers and objectives can wait for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlayerAct {
    /// Declared MAYDAY / PAN or squawked 7700.
    Mayday,
    /// Pulled / pushed any fire handle.
    FireHandle,
    /// Alternate (gravity) gear extension selected.
    AltGear,
    /// Gear lever down.
    GearDown,
    /// All thrust levers at idle.
    ThrottleIdle,
    /// Thrust levers at take-off / go-around power.
    Toga,
    /// Speed brakes extended.
    SpeedBrakes,
    /// Split the thrust levers (differential thrust).
    SplitThrottles,
    /// Brakes applied on the ground.
    Brakes,
}

impl PlayerAct {
    pub fn parse(s: &str) -> Option<PlayerAct> {
        Some(match s {
            "mayday" => PlayerAct::Mayday,
            "fire_handle" => PlayerAct::FireHandle,
            "alt_gear" => PlayerAct::AltGear,
            "gear_down" => PlayerAct::GearDown,
            "throttle_idle" => PlayerAct::ThrottleIdle,
            "toga" => PlayerAct::Toga,
            "speedbrakes" => PlayerAct::SpeedBrakes,
            "split_throttles" => PlayerAct::SplitThrottles,
            "brakes" => PlayerAct::Brakes,
            _ => return None,
        })
    }
}

/// A microburst (downburst) on the approach: a ring vortex model with an
/// outflow near the ground and a core downdraft (see `wind.rs`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Burst {
    /// Centre in the target runway frame (NM along the final from the threshold, + = towards the aircraft).
    pub at_nm: f32,
    /// Core radius (m).
    pub radius_m: f32,
    /// Peak downdraft (m/s, positive down).
    pub down_ms: f32,
    /// Peak horizontal outflow (m/s).
    pub out_ms: f32,
    /// Seconds to build up to full strength.
    pub ramp_s: f32,
}

/// What an event does.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// A systems failure by `--fail` id ("fire1", "hyd2", "pitot"...).
    Fail(String),
    /// Clear a failure (e.g. the pitot ice melts).
    Repair(String),
    /// Engine `i` destroyed (no relight): bird ingestion / uncontained failure.
    EngineDead(usize),
    /// Set the fuel left to this fraction of capacity (all tanks).
    FuelFrac(f32),
    /// Cabin hole: pressure leaks with this time constant (s).
    CabinLeak(f32),
    /// The alternate gear extension will not work either.
    AltGearFail,
    /// The autopilot and autothrottle drop out.
    ApOff,
    /// A microburst on the approach.
    Microburst(Burst),
    /// A cabin PA announcement (clip id: see `audio/params.rs` `Clip::pa`).
    Pa(String),
    /// A line from the crew / cabin / ATC shown as a subtitle.
    Note(String),
    /// Save a checkpoint here (retry from it).
    Checkpoint(String),
    /// Brakes lose effect (anti-skid / partial): factor 0..1 of normal.
    BrakeFactor(f32),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    pub id: String,
    pub when: Trigger,
    pub actions: Vec<Action>,
}

/// What an objective checks.
#[derive(Clone, Debug, PartialEq)]
pub enum ObjKind {
    /// Land on a runway (any, or the target if `target_only`) and stop.
    Land { target_only: bool },
    /// Land on a runway and stop, or ditch survivably.
    LandOrDitch,
    /// Stop on the runway without leaving it (rejected take-off).
    StopOnRunway,
    /// Below an altitude (ft MSL).
    AltBelow(f32),
    /// Above a height above the ground (ft) — e.g. a go-around climb.
    AglAbove(f32),
    /// Keep control (no stall warning, no overspeed, bank < 60°) for this long.
    InControl(f32),
    /// A player action.
    Player(PlayerAct),
    /// The fire in engine `i` is out.
    FireOut(usize),
    /// Never get below this height above the ground (ft) — escaping a shear.
    NoLowerThan(f32),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Objective {
    pub id: String,
    pub text: String,
    pub kind: ObjKind,
    /// The mission fails without it.
    pub required: bool,
    /// Points (procedure score) when done.
    pub points: f32,
    /// Must be done within this many seconds of `after` (or the start).
    pub within_s: Option<f32>,
    /// Counts from this event (it is hidden until then).
    pub after: Option<String>,
}

/// The target runway: airport + end, and its geometry (used headlessly and
/// to cross-check the airport database).
#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    pub airport: String,
    pub runway: String,
    pub course_deg: f32,
    pub length_m: f32,
    pub width_m: f32,
    pub elev_ft: f32,
    /// Threshold position (degrees), for start positions when no database is loaded.
    pub lat: f64,
    pub lon: f64,
}

/// Start state, in the target runway's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct StartState {
    /// NM along the landing direction from the threshold (− = on the approach side).
    pub x_nm: f32,
    /// NM to the right of the runway direction.
    pub y_nm: f32,
    /// Altitude, ft above mean sea level.
    pub alt_ft: f32,
    /// Heading relative to the runway course (deg).
    pub hdg_rel: f32,
    pub kias: f32,
    pub flaps: usize,
    pub gear_down: bool,
    pub throttle: f32,
    /// On the runway (lined up `x_m` beyond the threshold), not airborne.
    pub on_runway: bool,
    pub x_m: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mission {
    pub id: String,
    pub title: String,
    pub tagline: String,
    pub aircraft: String,
    /// 1 (straightforward) .. 5 (expert).
    pub difficulty: u8,
    pub minutes: f32,
    /// Neutral, factual "inspired by" history note.
    pub history: String,
    pub situation: String,
    /// What has failed / will fail (briefing bullets).
    pub failures: Vec<String>,
    pub tips: Vec<String>,
    pub target: Target,
    pub start: StartState,
    pub fuel_frac: f32,
    /// Local clock (hours) and day of year.
    pub hours: f32,
    pub day: u32,
    pub metar: String,
    pub pax: u32,
    pub time_limit_s: Option<f32>,
    pub limit_text: String,
    /// Clock shown on the briefing / HUD (e.g. "Fire spreads in").
    pub clock_label: String,
    /// A survivable crash (belly landing, ditching) still counts as recovered.
    pub survivable_ok: bool,
    /// Par time (s) for the time score.
    pub par_s: f32,
    pub events: Vec<Event>,
    pub objectives: Vec<Objective>,
}

// ---------------------------------------------------------------- parsing

fn s(v: &Value, k: &str) -> String {
    v.get(k).and_then(|x| x.as_str()).unwrap_or("").to_string()
}
fn f(v: &Value, k: &str) -> Option<f32> {
    v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32)
}
fn strs(v: &Value, k: &str) -> Vec<String> {
    v.get(k).and_then(|x| x.as_array()).map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect()).unwrap_or_default()
}

fn trigger(v: &Value) -> Result<Trigger, String> {
    let player = match v.get("player").and_then(|x| x.as_str()) {
        Some(p) => Some(PlayerAct::parse(p).ok_or_else(|| format!("unknown player action \"{p}\""))?),
        None => None,
    };
    Ok(Trigger {
        t: f(v, "t"),
        after: v.get("after").and_then(|x| x.as_str()).map(str::to_string),
        delay: f(v, "delay").unwrap_or(0.0),
        alt_above_ft: f(v, "alt_above_ft"),
        alt_below_ft: f(v, "alt_below_ft"),
        agl_below_ft: f(v, "agl_below_ft"),
        ias_above_kt: f(v, "ias_above_kt"),
        ias_below_kt: f(v, "ias_below_kt"),
        dist_below_nm: f(v, "dist_below_nm"),
        on_ground: v.get("on_ground").and_then(|x| x.as_bool()),
        player,
    })
}

fn action(v: &Value) -> Result<Action, String> {
    let obj = v.as_object().ok_or("action must be an object")?;
    let (k, x) = obj.iter().next().ok_or("empty action")?;
    let num = || x.as_f64().map(|n| n as f32).ok_or_else(|| format!("{k}: number expected"));
    let text = || x.as_str().map(str::to_string).ok_or_else(|| format!("{k}: text expected"));
    Ok(match k.as_str() {
        "fail" => Action::Fail(text()?),
        "repair" => Action::Repair(text()?),
        "engine_dead" => Action::EngineDead(num()? as usize),
        "fuel_frac" => Action::FuelFrac(num()?),
        "cabin_leak" => Action::CabinLeak(num()?),
        "alt_gear_fail" => Action::AltGearFail,
        "ap_off" => Action::ApOff,
        "pa" => Action::Pa(text()?),
        "note" => Action::Note(text()?),
        "checkpoint" => Action::Checkpoint(text()?),
        "brake_factor" => Action::BrakeFactor(num()?),
        "microburst" => Action::Microburst(Burst {
            at_nm: f(x, "at_nm").unwrap_or(2.0),
            radius_m: f(x, "radius_m").unwrap_or(900.0),
            down_ms: f(x, "down_ms").unwrap_or(10.0),
            out_ms: f(x, "out_ms").unwrap_or(15.0),
            ramp_s: f(x, "ramp_s").unwrap_or(3.0),
        }),
        other => return Err(format!("unknown action \"{other}\"")),
    })
}

fn objective(v: &Value) -> Result<Objective, String> {
    let kind_s = s(v, "kind");
    let kind = match kind_s.as_str() {
        "land" => ObjKind::Land { target_only: v.get("target_only").and_then(|x| x.as_bool()).unwrap_or(false) },
        "land_or_ditch" => ObjKind::LandOrDitch,
        "stop_on_runway" => ObjKind::StopOnRunway,
        "alt_below" => ObjKind::AltBelow(f(v, "ft").ok_or("alt_below needs ft")?),
        "agl_above" => ObjKind::AglAbove(f(v, "ft").ok_or("agl_above needs ft")?),
        "in_control" => ObjKind::InControl(f(v, "s").ok_or("in_control needs s")?),
        "no_lower_than" => ObjKind::NoLowerThan(f(v, "ft").ok_or("no_lower_than needs ft")?),
        "fire_out" => ObjKind::FireOut(f(v, "engine").unwrap_or(0.0) as usize),
        "player" => {
            let a = s(v, "act");
            ObjKind::Player(PlayerAct::parse(&a).ok_or_else(|| format!("unknown player act \"{a}\""))?)
        }
        other => return Err(format!("unknown objective kind \"{other}\"")),
    };
    Ok(Objective {
        id: s(v, "id"),
        text: s(v, "text"),
        kind,
        required: v.get("required").and_then(|x| x.as_bool()).unwrap_or(false),
        points: f(v, "points").unwrap_or(0.0),
        within_s: f(v, "within_s"),
        after: v.get("after").and_then(|x| x.as_str()).map(str::to_string),
    })
}

fn parse_hhmm(s: &str) -> Option<f32> {
    let (h, m) = s.split_once(':')?;
    Some(h.trim().parse::<f32>().ok()? + m.trim().parse::<f32>().ok()? / 60.0)
}

fn parse_date(s: &str) -> Option<u32> {
    let (m, d) = s.split_once('-')?;
    let (m, d): (u32, u32) = (m.parse().ok()?, d.parse().ok()?);
    const START: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    (1..=12).contains(&m).then(|| START[(m - 1) as usize] + d.clamp(1, 31))
}

impl Mission {
    pub fn parse(text: &str) -> Result<Mission, String> {
        let v: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let t = v.get("target").ok_or("no target")?;
        let st = v.get("start").ok_or("no start")?;
        let target = Target {
            airport: s(t, "airport"),
            runway: s(t, "runway"),
            course_deg: f(t, "course_deg").ok_or("target.course_deg")?,
            length_m: f(t, "length_m").ok_or("target.length_m")?,
            width_m: f(t, "width_m").unwrap_or(45.0),
            elev_ft: f(t, "elev_ft").unwrap_or(0.0),
            lat: t.get("lat").and_then(|x| x.as_f64()).ok_or("target.lat")?,
            lon: t.get("lon").and_then(|x| x.as_f64()).ok_or("target.lon")?,
        };
        let start = StartState {
            x_nm: f(st, "x_nm").unwrap_or(0.0),
            y_nm: f(st, "y_nm").unwrap_or(0.0),
            alt_ft: f(st, "alt_ft").unwrap_or(target.elev_ft),
            hdg_rel: f(st, "hdg_rel").unwrap_or(0.0),
            kias: f(st, "kias").unwrap_or(0.0),
            flaps: f(st, "flaps").unwrap_or(0.0) as usize,
            gear_down: st.get("gear_down").and_then(|x| x.as_bool()).unwrap_or(false),
            throttle: f(st, "throttle").unwrap_or(-1.0),
            on_runway: st.get("on_runway").and_then(|x| x.as_bool()).unwrap_or(false),
            x_m: f(st, "x_m").unwrap_or(40.0),
        };
        let mut events = Vec::new();
        for e in v.get("events").and_then(|x| x.as_array()).into_iter().flatten() {
            let when = trigger(e.get("when").unwrap_or(&Value::Null))?;
            let mut actions = Vec::new();
            for a in e.get("do").and_then(|x| x.as_array()).into_iter().flatten() {
                actions.push(action(a).map_err(|m| format!("event {}: {m}", s(e, "id")))?);
            }
            events.push(Event { id: s(e, "id"), when, actions });
        }
        let mut objectives = Vec::new();
        for o in v.get("objectives").and_then(|x| x.as_array()).into_iter().flatten() {
            objectives.push(objective(o)?);
        }
        let m = Mission {
            id: s(&v, "id"),
            title: s(&v, "title"),
            tagline: s(&v, "tagline"),
            aircraft: s(&v, "aircraft"),
            difficulty: f(&v, "difficulty").unwrap_or(3.0).clamp(1.0, 5.0) as u8,
            minutes: f(&v, "minutes").unwrap_or(10.0),
            history: s(&v, "history"),
            situation: s(&v, "situation"),
            failures: strs(&v, "failures"),
            tips: strs(&v, "tips"),
            target,
            start,
            fuel_frac: f(&v, "fuel_frac").unwrap_or(0.5),
            hours: parse_hhmm(&s(&v, "time")).unwrap_or(14.0),
            day: parse_date(&s(&v, "date")).unwrap_or(172),
            metar: s(&v, "metar"),
            pax: f(&v, "pax").unwrap_or(150.0) as u32,
            time_limit_s: f(&v, "time_limit_s"),
            limit_text: s(&v, "limit_text"),
            clock_label: s(&v, "clock_label"),
            survivable_ok: v.get("survivable_ok").and_then(|x| x.as_bool()).unwrap_or(false),
            par_s: f(&v, "par_s").unwrap_or(600.0),
            events,
            objectives,
        };
        m.validate()?;
        Ok(m)
    }

    /// Consistency checks: the aircraft exists, failures parse and apply to
    /// it, references resolve, the weather decodes, something is required.
    pub fn validate(&self) -> Result<(), String> {
        let id = &self.id;
        if id.is_empty() || self.title.is_empty() || self.situation.is_empty() {
            return Err("id, title and situation are required".into());
        }
        let def = crate::flight::find(&self.aircraft).ok_or_else(|| format!("{id}: unknown aircraft {}", self.aircraft))?;
        let sd = crate::systems::defs::sys_def(&def);
        let mut known: Vec<&str> = Vec::new();
        for e in &self.events {
            if e.id.is_empty() {
                return Err(format!("{id}: an event has no id"));
            }
            if known.contains(&e.id.as_str()) {
                return Err(format!("{id}: duplicate event {}", e.id));
            }
            if let Some(a) = &e.when.after
                && !known.contains(&a.as_str())
            {
                return Err(format!("{id}: event {} waits for {a}, which is not defined before it", e.id));
            }
            known.push(&e.id);
            for a in &e.actions {
                match a {
                    Action::Fail(n) | Action::Repair(n) => {
                        let (k, i) = crate::systems::failures::Kind::parse(n).ok_or_else(|| format!("{id}: unknown failure {n}"))?;
                        if !k.applies(&sd, &def) || i >= k.count(&sd, &def).max(1) {
                            return Err(format!("{id}: failure {n} does not apply to {}", def.id));
                        }
                    }
                    Action::EngineDead(i) if *i >= crate::missions::n_eng(&def) => return Err(format!("{id}: engine {i} out of range")),
                    Action::Pa(c) if crate::audio::params::Clip::pa(c).is_none() => return Err(format!("{id}: unknown PA clip {c}")),
                    _ => {}
                }
            }
        }
        for o in &self.objectives {
            if let Some(a) = &o.after
                && !known.contains(&a.as_str())
            {
                return Err(format!("{id}: objective {} waits for unknown event {a}", o.id));
            }
            if o.text.is_empty() || o.id.is_empty() {
                return Err(format!("{id}: objective without id / text"));
            }
            if let ObjKind::FireOut(i) = o.kind
                && i >= crate::missions::n_eng(&def)
            {
                return Err(format!("{id}: fire_out engine {i} out of range"));
            }
        }
        if !self.objectives.iter().any(|o| o.required) {
            return Err(format!("{id}: no required objective"));
        }
        if !self.metar.is_empty() && crate::weather::Wx::from_metar_str(&self.metar).is_none() {
            return Err(format!("{id}: METAR does not decode"));
        }
        if self.start.flaps >= def.flap_detents.len().max(1) {
            return Err(format!("{id}: flap detent {} out of range", self.start.flaps));
        }
        if !(0.0..=1.0).contains(&self.fuel_frac) {
            return Err(format!("{id}: fuel_frac out of range"));
        }
        Ok(())
    }

    /// The aircraft definition (catalogue index).
    pub fn aircraft_index(&self) -> Option<usize> {
        let id = crate::flight::find(&self.aircraft)?.id;
        crate::flight::catalogue().iter().position(|d| d.id == id)
    }
}

/// The built-in missions (embedded, so the browser build has them too).
pub const SOURCES: &[(&str, &str)] = &[
    ("v1", include_str!("../../assets/missions/v1.json")),
    ("decompression", include_str!("../../assets/missions/decompression.json")),
    ("bellyland", include_str!("../../assets/missions/bellyland.json")),
    ("pitot", include_str!("../../assets/missions/pitot.json")),
    ("microburst", include_str!("../../assets/missions/microburst.json")),
    ("uncontained", include_str!("../../assets/missions/uncontained.json")),
    ("river", include_str!("../../assets/missions/river.json")),
    ("smoke", include_str!("../../assets/missions/smoke.json")),
    ("flapjam", include_str!("../../assets/missions/flapjam.json")),
    ("deadstick", include_str!("../../assets/missions/deadstick.json")),
    ("hydraulics", include_str!("../../assets/missions/hydraulics.json")),
    ("hydraulics4", include_str!("../../assets/missions/hydraulics4.json")),
];

/// All built-in missions, parsed once (invalid ones are reported and skipped).
pub fn all() -> &'static [Mission] {
    static ALL: std::sync::OnceLock<Vec<Mission>> = std::sync::OnceLock::new();
    ALL.get_or_init(|| {
        SOURCES
            .iter()
            .filter_map(|(n, t)| match Mission::parse(t) {
                Ok(m) => Some(m),
                Err(e) => {
                    eprintln!("mission {n}: {e}");
                    None
                }
            })
            .collect()
    })
}

pub fn by_id(id: &str) -> Option<&'static Mission> {
    all().iter().find(|m| m.id == id)
}
