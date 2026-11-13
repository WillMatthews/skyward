//! ATIS: the runway in use chosen from the wind, the transition level, and
//! the broadcast assembled from the live (or fixed) METAR in UK style:
//!
//! > Heathrow information Bravo, time 1250. Landing runway 27R, departing
//! > runway 27L. Surface wind 240 degrees 12 knots. Visibility 10 kilometres
//! > or more. Light rain. Cloud few 2,000 feet, broken 3,500 feet.
//! > Temperature 15, dew point 8. QNH 1013 hectopascals. Transition level
//! > flight level 70. Expect ILS approach. Advise on initial contact you
//! > have information Bravo.

use super::phrase::{self, Utt};
use crate::airports::Airport;
use crate::weather::{Cover, Wx};

/// The runways in use.
#[derive(Clone, Debug, PartialEq)]
pub struct InUse {
    pub arr: String,
    pub dep: String,
    /// Arrival runway end: true heading (deg).
    pub hdg: f32,
}

/// Runway ends usable for the airport's main traffic: (ident, true heading, length m, paved).
fn ends(ap: &Airport) -> Vec<(String, f32, f32, bool)> {
    let longest = ap.runways.iter().filter(|r| !r.ends[0].ident.starts_with('H')).map(|r| r.length_m).fold(0.0, f32::max);
    let mut v = Vec::new();
    for r in &ap.runways {
        if r.ends.iter().any(|e| e.ident.starts_with('H') || e.ident.is_empty()) || r.length_m < 0.55 * longest {
            continue;
        }
        for i in 0..2 {
            if let Some(h) = r.heading(i) {
                v.push((r.ends[i].ident.clone(), h, r.length_m, r.surface.paved()));
            }
        }
    }
    v
}

/// The runway in use from the surface wind: the end with the most headwind
/// (ties → the longer runway); in light winds (< 5 kt) a westerly
/// preference, as at the London airports. Parallel runways: landing on the
/// right-hand one heading west, the left-hand one heading east, departing on
/// the other (segregated mode).
pub fn runway_in_use(ap: &Airport, wind_from: f32, wind_kt: f32) -> Option<InUse> {
    let v = ends(ap);
    let paved = v.iter().any(|e| e.3);
    let score = |e: &(String, f32, f32, bool)| -> f32 {
        let hw = wind_kt * (wind_from - e.1).to_radians().cos();
        let len = e.2 / 1000.0;
        let surf = if paved && !e.3 { -5.0 } else { 0.0 };
        if wind_kt < 5.0 {
            // Up to 5 kt of tailwind is accepted for the preferred direction.
            let west = if (180.0..360.0).contains(&e.1.rem_euclid(360.0)) { 2.0 } else { 0.0 };
            len + west + surf
        } else {
            hw * 2.0 + len * 0.5 + surf
        }
    };
    let best = v.iter().max_by(|a, b| score(a).total_cmp(&score(b)))?;
    let num = |s: &str| s.chars().take_while(|c| c.is_ascii_digit()).collect::<String>();
    let parallel: Vec<&(String, f32, f32, bool)> = v.iter().filter(|e| num(&e.0) == num(&best.0) && e.0 != best.0).collect();
    let (arr, dep) = match parallel.first() {
        Some(p) if best.0.len() > num(&best.0).len() => {
            let west = (180.0..360.0).contains(&best.1.rem_euclid(360.0));
            let want = if west { 'R' } else { 'L' };
            if best.0.ends_with(want) { (best.0.clone(), p.0.clone()) } else { (p.0.clone(), best.0.clone()) }
        }
        _ => (best.0.clone(), best.0.clone()),
    };
    let hdg = v.iter().find(|e| e.0 == arr).map_or(best.1, |e| e.1);
    Some(InUse { arr, dep, hdg })
}

/// Transition altitude (ft) by country / area: 6,000 ft in the London TMA,
/// 3,000 ft elsewhere in the UK and Ireland, 18,000 ft in North America,
/// 5,000 ft otherwise.
pub fn transition_altitude(ap: &Airport) -> u32 {
    match ap.country.as_str() {
        "GB" => {
            let london = crate::airports::distance_km((ap.lat, ap.lon), (51.5, -0.12)) < 75.0;
            if london { 6000 } else { 3000 }
        }
        "IE" => 5000,
        "US" | "CA" => 18_000,
        "NL" | "BE" => 3000,
        "AT" | "CH" => 10_000,
        _ => 5000,
    }
}

/// Transition level (flight level number): the lowest flight level at least
/// 1,000 ft above the transition altitude at this QNH.
pub fn transition_level(ta_ft: u32, qnh: f32) -> u32 {
    let pa = ta_ft as f32 + (1013.0 - qnh.round()) * 27.0 + 1000.0;
    ((pa / 1000.0 - 1e-3).ceil() * 10.0) as u32
}

/// The station's spoken / shown name ("heathrow" / "Heathrow"), else the ICAO spelled.
pub fn station_name(ap: &Airport) -> (String, String) {
    match phrase::airport_name(ap.code()) {
        Some((s, t)) => (s.to_string(), t.to_string()),
        None => (phrase::spell(ap.code()), ap.code().to_string()),
    }
}

/// Present weather as spoken words ("-SHRA" → "light rain showers").
pub fn wx_words(group: &str) -> Option<String> {
    let mut g = group;
    let mut out: Vec<&str> = Vec::new();
    let inten = if let Some(r) = g.strip_prefix('-') {
        g = r;
        Some("light")
    } else if let Some(r) = g.strip_prefix('+') {
        g = r;
        Some("heavy")
    } else {
        None
    };
    let vc = if let Some(r) = g.strip_prefix("VC") {
        g = r;
        true
    } else {
        false
    };
    out.extend(inten);
    let mut showers = false;
    let mut ts = false;
    let mut i = 0;
    let b = g.as_bytes();
    let mut phen = Vec::new();
    while i + 2 <= b.len() {
        match &g[i..i + 2] {
            "SH" => showers = true,
            "TS" => ts = true,
            "FZ" => out.push("freezing"),
            "RA" => phen.push("rain"),
            "DZ" => phen.push("drizzle"),
            "SN" | "SG" => phen.push("snow"),
            "GR" | "GS" => phen.push("hail"),
            "BR" => phen.push("mist"),
            "FG" => phen.push("fog"),
            "HZ" => phen.push("haze"),
            _ => {}
        }
        i += 2;
    }
    if ts {
        out.push("thunderstorm");
    }
    out.extend(phen.iter());
    if showers {
        out.push("showers");
    }
    if out.iter().all(|w| *w == "light" || *w == "heavy") {
        return None;
    }
    if vc {
        out.push("in the vicinity");
    }
    Some(out.join(" "))
}

/// The ATIS broadcast for `ap` in weather `wx`, information `letter`, observation time (UTC h, m).
pub fn broadcast(ap: &Airport, wx: &Wx, letter: char, time: (u32, u32), rwy: &InUse) -> Utt {
    let (sn, st) = station_name(ap);
    let lw = phrase::letter_word(letter).unwrap_or("alpha");
    let lt = super::cap(lw);
    let mut u = Utt::new();
    u.say_as(&format!("{sn} information {lw}"), &format!("{st} information {lt}")).comma();
    let hhmm = format!("{:02}{:02}", time.0 % 24, time.1 % 60);
    u.say_as(&format!("time {}", phrase::spell(&hhmm)), &format!("time {hhmm}")).stop();
    // Runways.
    if rwy.arr == rwy.dep {
        u.say_as(&format!("runway in use {}", phrase::runway_words(&rwy.arr)), &format!("runway in use {}", rwy.arr)).stop();
    } else {
        u.say_as(&format!("landing runway {}", phrase::runway_words(&rwy.arr)), &format!("landing runway {}", rwy.arr)).comma();
        u.say_as(&format!("departing runway {}", phrase::runway_words(&rwy.dep)), &format!("departing runway {}", rwy.dep)).stop();
    }
    // Wind.
    let m = wx.metar.as_ref();
    let var = m.is_some_and(|m| m.wind_dir.is_none());
    u.say("surface wind").wind(if var { None } else { Some(wx.wind_from) }, wx.wind_kt, (wx.gust_kt > 0.0).then_some(wx.gust_kt));
    if let Some((a, b)) = m.and_then(|m| m.wind_var) {
        let (a, b) = (((a / 10.0).round() * 10.0) as u32, ((b / 10.0).round() * 10.0) as u32);
        u.comma().say_as(
            &format!("variable between {} and {} degrees", phrase::spell(&format!("{a:03}")), phrase::spell(&format!("{b:03}"))),
            &format!("variable between {a:03} and {b:03} degrees"),
        );
    }
    u.stop();
    let cavok = m.is_some_and(|m| m.cavok);
    if cavok {
        u.say_as("cavok", "CAVOK").stop();
    } else {
        // Visibility.
        let v = wx.vis_m;
        if v >= 9999.0 {
            u.say_as("visibility one zero kilometres or more", "visibility 10 kilometres or more");
        } else if v >= 5000.0 {
            let km = (v / 1000.0).floor() as u32;
            u.say_as(&format!("visibility {} kilometres", phrase::digits(km)), &format!("visibility {km} kilometres"));
        } else {
            let mm = ((v / 100.0).round().max(1.0) * 100.0) as u32;
            u.say_as(&format!("visibility {} metres", phrase::alt_words(mm)), &format!("visibility {} metres", phrase::thousands(mm)));
        }
        u.stop();
        // Present weather.
        let groups: Vec<String> = match m {
            Some(m) => m.wx.clone(),
            None => synth_wx(wx),
        };
        let mut any = false;
        for g in &groups {
            if let Some(w) = wx_words(g) {
                if any {
                    u.comma();
                }
                u.say(&w);
                any = true;
            }
        }
        if any {
            u.stop();
        }
        // Cloud.
        if wx.layers.is_empty() {
            u.say(if m.is_some_and(|m| m.ncd) { "no cloud detected" } else { "no significant cloud" }).stop();
        } else {
            u.say("cloud");
            for (i, l) in wx.layers.iter().take(4).enumerate() {
                if i > 0 {
                    u.comma();
                }
                let cov = match l.cover {
                    Cover::Few => "few",
                    Cover::Sct => "scattered",
                    Cover::Bkn => "broken",
                    Cover::Ovc => "overcast",
                    Cover::Vv => "vertical visibility",
                };
                u.say(cov).feet(l.base_ft.max(100.0));
                if l.convective {
                    u.say("cumulonimbus");
                }
            }
            u.stop();
        }
    }
    // Temperature / dew point.
    let t = |u: &mut Utt, c: f32| {
        let n = c.round() as i32;
        if n < 0 {
            u.say_as(&format!("minus {}", phrase::digits(n.unsigned_abs())), &n.to_string());
        } else {
            u.num(n as u32);
        }
    };
    u.say("temperature");
    t(&mut u, wx.temp_c);
    u.comma().say("dew point");
    t(&mut u, wx.dew_c);
    u.stop();
    u.qnh(wx.qnh_hpa).say("hectopascals").stop();
    let tl = transition_level(transition_altitude(ap), wx.qnh_hpa);
    if tl < 180 {
        u.say_as(&format!("transition level flight level {}", phrase::fl_words(tl)), &format!("transition level flight level {tl}")).stop();
    }
    if crate::nav::db::ils_at(ap).iter().any(|n| n.ils.as_ref().is_some_and(|i| i.runway == rwy.arr) && n.kind == crate::nav::db::Kind::Ils) {
        u.say_as("expect ils approach", "expect ILS approach").stop();
    }
    if wx.precip != crate::weather::Precip::None || wx.recent_rain {
        u.say("runway surface wet").stop();
    }
    u.say_as(&format!("advise on initial contact you have information {lw}"), &format!("advise on initial contact you have information {lt}")).stop();
    u.finish()
}

/// Present-weather groups for weather without a METAR (presets / Open-Meteo).
fn synth_wx(wx: &Wx) -> Vec<String> {
    use crate::weather::Precip;
    let mut v = Vec::new();
    let i = if wx.intensity < 0.4 {
        "-"
    } else if wx.intensity > 0.8 {
        "+"
    } else {
        ""
    };
    let sh = if wx.showers { "SH" } else { "" };
    match wx.precip {
        Precip::Rain => v.push(format!("{i}{sh}RA")),
        Precip::Drizzle => v.push(format!("{i}DZ")),
        Precip::Snow | Precip::Sleet => v.push(format!("{i}{sh}SN")),
        Precip::Hail => v.push(format!("{i}SHGR")),
        _ => {}
    }
    if wx.thunder {
        v.push("TS".into());
    }
    if wx.fog {
        v.push("FG".into());
    } else if wx.mist {
        v.push("BR".into());
    }
    if wx.haze {
        v.push("HZ".into());
    }
    v
}

/// A digest of what the ATIS reports: a change advances the letter.
pub fn digest(wx: &Wx, rwy: &InUse) -> String {
    format!(
        "{}|{}|{:.0}|{:.0}|{:.0}|{}|{}|{:.0}",
        rwy.arr,
        rwy.dep,
        (wx.wind_from / 10.0).round(),
        (wx.wind_kt / 5.0).round(),
        wx.qnh_hpa,
        wx.cloud_text(),
        (wx.vis_m.min(9999.0) / 1000.0).floor(),
        wx.temp_c
    )
}
