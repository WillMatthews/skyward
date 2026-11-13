//! Traffic information and arrival sequencing against the other aircraft in
//! the picture (live ADS-B traffic, `adsb/`, and other players):
//!
//! * "Skyward 123, traffic, two o'clock, five miles, opposite direction,
//!   two thousand feet above, Airbus" (subtitle shows the type, "A320") —
//!   UK CAP 413 order: position by the clock, distance, relative movement,
//!   level, type. Radar units and towers pass it for the most urgent
//!   intruder (closing, within 8 NM and 2,500 ft), once per aircraft.
//! * While vectoring for an approach, a real arrival already on final that
//!   would end up less than 4 NM ahead of us makes the director extend our
//!   pattern: "extend downwind, number two, following Airbus on final, six
//!   miles" (the downwind / base points move out by the missing spacing).

use super::freqs::Fac;
use super::phrase::{Utt, digits};
use super::vectors::{Final, NM, hdg_diff};
use super::{Atc, Ctx, Phase, Station};
use crate::net::Traffic;
use glam::{Vec3, vec2};

const FT: f32 = 0.3048;
/// Wanted spacing on final behind real arrivals, NM.
pub const SPACING_NM: f32 = 4.0;

/// Clock position (1..=12) of a bearing relative to our track (degrees).
pub fn clock(own_track_deg: f32, bearing_deg: f32) -> u32 {
    let rel = (bearing_deg - own_track_deg).rem_euclid(360.0);
    let c = (rel / 30.0).round() as u32 % 12;
    if c == 0 { 12 } else { c }
}

/// Spoken clock number ("two", "ten", "twelve").
pub fn clock_word(c: u32) -> &'static str {
    ["twelve", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve"][c.min(12) as usize]
}

/// Relative movement from the two tracks (degrees).
pub fn movement(own_track: f32, other_track: f32) -> &'static str {
    let d = hdg_diff(other_track, own_track);
    if d.abs() > 135.0 {
        "opposite direction"
    } else if d.abs() < 45.0 {
        "same direction"
    } else {
        // Its track turned clockwise from ours: moving to our right.
        if d > 0.0 { "crossing left to right" } else { "crossing right to left" }
    }
}

/// How the type is spoken and shown: live traffic by its ICAO designator,
/// players by the class of their aircraft.
pub fn type_words(t: &Traffic) -> (String, String) {
    if t.live {
        let s = crate::adsb::types::lookup(&t.kind).unwrap_or_else(|| crate::adsb::types::stand_in("", "", 0.0, 0.0));
        let shown = if t.kind.is_empty() { s.spoken.to_string() } else { t.kind.clone() };
        return (s.spoken.to_string(), shown);
    }
    use crate::flight::Category::*;
    let cat = crate::flight::find(&t.aircraft).map(|d| d.category).unwrap_or_default();
    let w = match cat {
        Airliner => "airliner",
        Glider => "glider",
        Helicopter => "helicopter",
        FastJet => "military jet",
        Drone => "drone",
        Trainer | Warbird => "light aircraft",
    };
    (w.to_string(), w.to_string())
}

fn track_of(v: Vec3) -> f32 {
    v.x.atan2(-v.z).to_degrees().rem_euclid(360.0)
}

/// The traffic information transmission (without our callsign).
pub fn info(own: Vec3, own_track: f32, t: &Traffic) -> Utt {
    let d = vec2(t.pos.x - own.x, t.pos.z - own.z);
    let nm = (d.length() / NM).round().max(1.0) as u32;
    let brg = d.x.atan2(-d.y).to_degrees();
    let c = clock(own_track, brg);
    let other_track = if vec2(t.vel.x, t.vel.z).length() > 5.0 { track_of(t.vel) } else { t.heading.to_degrees() };
    let mv = movement(own_track, other_track);
    let dh = ((t.pos.y - own.y) / FT / 100.0).round() * 100.0;
    let (spoken, shown) = type_words(t);
    let mut u = Utt::new();
    u.say("traffic").comma();
    u.say_as(&format!("{} o'clock", clock_word(c)), &format!("{c} o'clock")).comma();
    u.say_as(&format!("{} miles", digits(nm)), &format!("{nm} miles")).comma();
    u.say(mv).comma();
    if dh.abs() < 300.0 {
        u.say("same level");
    } else {
        u.feet(dh.abs()).say(if dh > 0.0 { "above" } else { "below" });
    }
    u.comma().say_as(&spoken, &shown);
    u.finish()
}

/// The intruder most worth a call: airborne, within 8 NM and 2,500 ft,
/// closing (time to closest approach under 150 s) or within 3 NM, not
/// called recently.
pub fn pick<'a>(own: Vec3, own_vel: Vec3, traffic: &'a [Traffic], recently: &dyn Fn(u16) -> bool) -> Option<&'a Traffic> {
    let mut best: Option<(f32, &Traffic)> = None;
    for t in traffic {
        if t.on_ground || recently(t.id) {
            continue;
        }
        let r = t.pos - own;
        let rh = vec2(r.x, r.z);
        let range = rh.length();
        if range > 8.0 * NM || (r.y / FT).abs() > 2500.0 {
            continue;
        }
        let vrel = vec2(t.vel.x - own_vel.x, t.vel.z - own_vel.z);
        let closing = -rh.dot(vrel) / range.max(1.0);
        let tau = if closing > 1.0 { range / closing } else { f32::INFINITY };
        if tau > 150.0 && range > 3.0 * NM {
            continue;
        }
        let score = tau.min(range / 60.0);
        if best.is_none_or(|b| score < b.0) {
            best = Some((score, t));
        }
    }
    best.map(|b| b.1)
}

/// A real arrival on final that we would follow too closely: (its distance
/// to the threshold in NM, the traffic, NM of extension needed). `own_nm`:
/// our track miles to the threshold.
pub fn sequence<'a>(fin: &Final, elev_m: f32, own_nm: f32, traffic: &'a [Traffic]) -> Option<(f32, &'a Traffic, f32)> {
    let course = fin.course();
    let mut best: Option<(f32, &Traffic, f32)> = None;
    for t in traffic.iter().filter(|t| !t.on_ground) {
        let p = vec2(t.pos.x, t.pos.z);
        let along = fin.along(p) / NM;
        let lat = fin.lateral(p).abs() / NM;
        let trk = track_of(t.vel);
        let aligned = hdg_diff(trk, course).abs() < 30.0;
        let low = (t.pos.y - elev_m) / FT < 5000.0;
        if along <= 0.3 || along > 15.0 || lat > 1.0 || !aligned || !low {
            continue;
        }
        // It lands first if it is ahead of us along the track miles.
        let gap = own_nm - along;
        if gap < SPACING_NM && gap > -1.0 {
            let need = SPACING_NM - gap;
            if best.is_none_or(|b| along > b.0) {
                best = Some((along, t, need));
            }
        }
    }
    best
}

impl Atc {
    /// Traffic information / sequencing calls (from `monitor`, with the
    /// frequency quiet). True when a call was made.
    pub(super) fn traffic_calls(&mut self, st: &Station, ctx: &Ctx, dt: f32) -> bool {
        self.traffic_check -= dt.max(0.0);
        self.traffic_seen.retain(|_, t| *t > -240.0);
        for t in self.traffic_seen.values_mut() {
            *t -= dt;
        }
        if self.traffic_check > 0.0 {
            return false;
        }
        self.traffic_check = 4.0;
        let ac = ctx.ac;
        if ac.on_ground || !(st.fac.is_radar() || st.fac == Fac::Tower) {
            return false;
        }
        if !matches!(self.phase, Phase::Departure | Phase::Enroute | Phase::Vectors | Phase::Approach) {
            return false;
        }
        let traffic = crate::net::traffic_picture();
        if traffic.is_empty() {
            return false;
        }
        let own = ac.pos;
        let own_track = track_of(ac.vel);
        // Sequencing: once per approach, while being vectored.
        if self.phase == Phase::Vectors && !self.seq_given {
            if let Some(arr) = self.arr.as_mut() {
                if let Some(dir) = arr.director.as_mut().filter(|d| !d.intercept_given) {
                    let p = vec2(own.x, own.z);
                    let own_nm = dir.track_miles(p);
                    let elev = dir.fin.elev_ft * FT;
                    if let Some((on_final, t, need)) = sequence(&dir.fin, elev, own_nm, &traffic) {
                        let downwind = dir.plan.wps.len() >= 2 && dir.wp + 1 < dir.plan.wps.len();
                        dir.extend(need * NM);
                        dir.issued = None;
                        let (spoken, shown) = type_words(t);
                        let miles = on_final.round().max(1.0) as u32;
                        let mut b = Utt::new();
                        b.say(if downwind { "extend downwind" } else { "vectors for spacing" }).comma();
                        b.say_as("number two", "number 2").comma().say("following").say_as(&spoken, &shown).say("on final").comma();
                        b.say_as(&format!("{} miles", digits(miles)), &format!("{miles} miles"));
                        let mut rb = Utt::new();
                        if downwind {
                            rb.say("extend downwind").comma();
                        }
                        rb.say_as("number two", "number 2");
                        self.seq_given = true;
                        self.traffic_seen.insert(t.id, 0.0);
                        self.instruct(st, b, Some(rb), vec![]);
                        return true;
                    }
                }
            }
        }
        let seen = self.traffic_seen.clone();
        let recently = |id: u16| seen.contains_key(&id);
        if let Some(t) = pick(own, ac.vel, &traffic, &recently) {
            let body = info(own, own_track, t);
            self.traffic_seen.insert(t.id, 0.0);
            let rb = Utt::new().say("looking out").clone();
            self.instruct(st, body, Some(rb), vec![]);
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tr(pos: Vec3, vel: Vec3, kind: &str) -> Traffic {
        Traffic { id: 7, name: "BAW12".into(), aircraft: "meridian320".into(), pos, vel, heading: 0.0, on_ground: false, kind: kind.into(), live: true }
    }

    #[test]
    fn clock_positions() {
        assert_eq!(clock(0.0, 0.0), 12);
        assert_eq!(clock(0.0, 60.0), 2);
        assert_eq!(clock(90.0, 0.0), 9);
        assert_eq!(clock(350.0, 20.0), 1);
        assert_eq!(clock(0.0, 300.0), 10);
        assert_eq!(clock_word(10), "ten");
    }

    #[test]
    fn movements() {
        assert_eq!(movement(0.0, 180.0), "opposite direction");
        assert_eq!(movement(90.0, 100.0), "same direction");
        // We go north; traffic heading east moves left to right.
        assert_eq!(movement(0.0, 90.0), "crossing left to right");
        assert_eq!(movement(0.0, 270.0), "crossing right to left");
    }

    /// "traffic, two o'clock, five miles, opposite direction, two thousand feet above, Airbus".
    #[test]
    fn traffic_information_phrase() {
        // We fly north at 1,000 m; an A320 5 NM away at our 2 o'clock, 2,000 ft above, southbound.
        let own = Vec3::new(0.0, 1000.0, 0.0);
        let b = 60f32.to_radians();
        let p = Vec3::new(b.sin() * 5.0 * NM, 1000.0 + 2000.0 * FT, -b.cos() * 5.0 * NM);
        let u = info(own, 0.0, &tr(p, Vec3::new(0.0, 0.0, 120.0), "A320"));
        assert_eq!(u.text, "Traffic, 2 o'clock, 5 miles, opposite direction, 2,000 feet above, A320.");
        assert!(u.missing.is_empty(), "not in the vocabulary: {:?}", u.missing);
        // Same level, crossing, a helicopter.
        let p = Vec3::new(-3.0 * NM, 1050.0, 0.0);
        let u = info(own, 0.0, &tr(p, Vec3::new(40.0, 0.0, 0.0), "EC35"));
        assert_eq!(u.text, "Traffic, 9 o'clock, 3 miles, crossing left to right, same level, EC35.");
        assert!(u.missing.is_empty(), "{:?}", u.missing);
        // Below, unknown type → class words.
        let p = Vec3::new(0.0, 500.0, -4.0 * NM);
        let u = info(own, 0.0, &tr(p, Vec3::new(0.0, 0.0, -50.0), ""));
        assert!(u.text.contains("12 o'clock, 4 miles, same direction, 1,600 feet below"), "{}", u.text);
        assert!(u.missing.is_empty(), "{:?}", u.missing);
    }

    #[test]
    fn picks_the_closing_intruder() {
        let own = Vec3::new(0.0, 1000.0, 0.0);
        let v = Vec3::new(0.0, 0.0, -100.0);
        let head_on = Traffic { id: 1, ..tr(Vec3::new(0.0, 1100.0, -6.0 * NM), Vec3::new(0.0, 0.0, 100.0), "A320") };
        let far = Traffic { id: 2, ..tr(Vec3::new(0.0, 1100.0, -12.0 * NM), Vec3::new(0.0, 0.0, 100.0), "A320") };
        let high = Traffic { id: 3, ..tr(Vec3::new(0.0, 2500.0, -3.0 * NM), Vec3::new(0.0, 0.0, 100.0), "A320") };
        let diverging = Traffic { id: 4, ..tr(Vec3::new(5.0 * NM, 1000.0, 0.0), Vec3::new(150.0, 0.0, 0.0), "A320") };
        let list = vec![far, high, diverging, head_on];
        let none = |_: u16| false;
        assert_eq!(pick(own, v, &list, &none).map(|t| t.id), Some(1));
        let called = |id: u16| id == 1;
        assert_eq!(pick(own, v, &list, &called).map(|t| t.id), None);
    }

    #[test]
    fn sequencing_behind_an_arrival() {
        // Runway 27 at the origin: landing west, final to the east.
        let fin = Final { thr: vec2(0.0, 0.0), dir: vec2(-1.0, 0.0), elev_ft: 80.0, gs_deg: 3.0 };
        let on_final = tr(Vec3::new(6.0 * NM, 600.0, 0.0), Vec3::new(-70.0, -3.5, 0.0), "A20N");
        let list = vec![on_final];
        // We are 8 track miles out: only 2 NM behind → extend by 2 NM.
        let (d, t, need) = sequence(&fin, 25.0, 8.0, &list).unwrap();
        assert!((d - 6.0).abs() < 0.01 && t.kind == "A20N" && (need - 2.0).abs() < 0.01);
        // 12 track miles: already 6 NM behind.
        assert!(sequence(&fin, 25.0, 12.0, &list).is_none());
        // Departing traffic (wrong direction) is not an arrival.
        let dep = vec![tr(Vec3::new(6.0 * NM, 600.0, 0.0), Vec3::new(70.0, 5.0, 0.0), "A20N")];
        assert!(sequence(&fin, 25.0, 8.0, &dep).is_none());
        // The director moves the pattern out.
        let mut dir = super::super::vectors::Director::new(fin.clone(), vec2(3.0 * NM, -6.0 * NM));
        let before = dir.track_miles(vec2(3.0 * NM, -6.0 * NM));
        dir.extend(2.0 * NM);
        let after = dir.track_miles(vec2(3.0 * NM, -6.0 * NM));
        assert!(after - before > 1.5 && after - before < 4.5, "{before:.1} → {after:.1}");
    }
}
