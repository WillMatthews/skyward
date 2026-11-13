//! The ADS-B feed: adsb.lol's open API (readsb "v2" JSON, ODbL) parsed into
//! [`Report`]s. Only what the traffic needs is kept — no registrations, no
//! receiver data.
//!
//! Endpoint: `GET https://api.adsb.lol/v2/point/{lat}/{lon}/{radius_nm}`
//! (radius ≤ 250 NM, no key). Response:
//! `{"ac": [{hex, type, flight, t, alt_baro, alt_geom, gs, track, true_heading,
//! roll, track_rate, baro_rate, geom_rate, squawk, category, emergency, lat,
//! lon, seen_pos, seen, nac_p, ...}], "now": ms, "total": n, "msg": "No error"}`.
//! `alt_baro` is a number (ft, standard pressure) or `"ground"`.

use serde_json::Value;

/// The API's base URL (point query: `{BASE}/{lat}/{lon}/{radius_nm}`).
pub const BASE: &str = "https://api.adsb.lol/v2/point";
/// Largest radius the API serves, NM.
pub const MAX_RADIUS_NM: f32 = 250.0;
/// Positions older than this (s) when fetched are dropped.
pub const MAX_POS_AGE: f64 = 45.0;

/// How the position was obtained (accuracy: ADS-B ≫ MLAT ≫ TIS-B).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Source {
    #[default]
    Adsb,
    /// Multilateration (noisier, ~50–200 m).
    Mlat,
    /// Rebroadcast / radar-derived.
    Other,
}

/// One aircraft position report.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Report {
    /// ICAO 24-bit address (non-ICAO / anonymous addresses get bit 24 set).
    pub hex: u32,
    /// Callsign as broadcast (trimmed, may be empty).
    pub callsign: String,
    /// ICAO type designator ("A320"), may be empty.
    pub icao_type: String,
    /// Emitter category ("A3"), may be empty.
    pub category: String,
    /// Feed clock (s, Unix) the position is valid at.
    pub t: f64,
    pub lat: f64,
    pub lon: f64,
    /// Barometric (pressure) altitude, ft — None on the ground or unknown.
    pub alt_baro_ft: Option<f32>,
    /// Geometric (GNSS, WGS84 ellipsoid) altitude, ft.
    pub alt_geom_ft: Option<f32>,
    pub on_ground: bool,
    pub gs_kt: Option<f32>,
    /// Track over the ground, degrees true.
    pub track_deg: Option<f32>,
    pub true_heading_deg: Option<f32>,
    /// Bank, degrees (+ right wing down), when the aircraft sends it.
    pub roll_deg: Option<f32>,
    /// Track rate, degrees per second (+ right).
    pub track_rate: Option<f32>,
    /// Vertical rate, ft/min (barometric, else geometric).
    pub vrate_fpm: Option<f32>,
    pub squawk: Option<u16>,
    /// Declared emergency ("general", "lifeguard", ...), None = none.
    pub emergency: Option<String>,
    pub source: Source,
}

/// One fetched snapshot.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// Feed clock when served (s, Unix).
    pub now: f64,
    pub reports: Vec<Report>,
}

fn f(v: &Value, k: &str) -> Option<f64> {
    v.get(k).and_then(|x| x.as_f64()).filter(|x| x.is_finite())
}

fn s<'a>(v: &'a Value, k: &str) -> Option<&'a str> {
    v.get(k).and_then(|x| x.as_str())
}

/// "4ca7b5" → 0x4ca7b5; "~1a2b3c" (non-ICAO) → 0x11a2b3c.
pub fn parse_hex(h: &str) -> Option<u32> {
    let (tilde, h) = match h.strip_prefix('~') {
        Some(r) => (true, r),
        None => (false, h),
    };
    let v = u32::from_str_radix(h, 16).ok().filter(|v| *v <= 0xff_ffff)?;
    Some(if tilde { v | 0x100_0000 } else { v })
}

/// One aircraft object of the response (None: no usable position).
pub fn parse_aircraft(v: &Value, now: f64) -> Option<Report> {
    let hex = parse_hex(s(v, "hex")?)?;
    let lat = f(v, "lat")?;
    let lon = f(v, "lon")?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    let age = f(v, "seen_pos").or_else(|| f(v, "seen")).unwrap_or(0.0).max(0.0);
    if age > MAX_POS_AGE {
        return None;
    }
    let (alt_baro_ft, ground_flag) = match v.get("alt_baro") {
        Some(Value::String(g)) if g == "ground" => (None, true),
        Some(x) => (x.as_f64().map(|a| a as f32), false),
        None => (None, false),
    };
    let alt_geom_ft = f(v, "alt_geom").map(|a| a as f32);
    let kind = s(v, "type").unwrap_or("adsb_icao");
    let source = if kind.starts_with("adsb") || kind.starts_with("adsr") {
        Source::Adsb
    } else if kind == "mlat" {
        Source::Mlat
    } else {
        Source::Other
    };
    let vrate_fpm = f(v, "baro_rate").or_else(|| f(v, "geom_rate")).map(|x| x as f32);
    let squawk = s(v, "squawk").and_then(|q| q.parse::<u16>().ok());
    let emergency = s(v, "emergency").filter(|e| !e.is_empty() && *e != "none").map(String::from);
    Some(Report {
        hex,
        callsign: s(v, "flight").unwrap_or("").trim().to_string(),
        icao_type: s(v, "t").unwrap_or("").trim().to_ascii_uppercase(),
        category: s(v, "category").unwrap_or("").trim().to_ascii_uppercase(),
        t: now - age,
        lat,
        lon,
        alt_baro_ft,
        alt_geom_ft,
        on_ground: ground_flag,
        gs_kt: f(v, "gs").map(|x| x as f32),
        track_deg: f(v, "track").map(|x| x as f32),
        true_heading_deg: f(v, "true_heading").map(|x| x as f32),
        roll_deg: f(v, "roll").map(|x| x as f32),
        track_rate: f(v, "track_rate").map(|x| x as f32),
        vrate_fpm,
        squawk,
        emergency,
        source,
    })
}

/// A whole response. `Err` for anything that is not the API's JSON.
pub fn parse(text: &str) -> Result<Snapshot, String> {
    let v: Value = serde_json::from_str(text).map_err(|e| format!("bad JSON: {e}"))?;
    parse_value(&v)
}

pub fn parse_value(v: &Value) -> Result<Snapshot, String> {
    let ac = v.get("ac").and_then(|a| a.as_array()).ok_or_else(|| "no \"ac\" list".to_string())?;
    let now = f(v, "now").map(|ms| ms / 1000.0).ok_or_else(|| "no \"now\"".to_string())?;
    // Ground vehicles (C1..C3), towers and obstacles are not aircraft.
    let reports = ac
        .iter()
        .filter_map(|a| parse_aircraft(a, now))
        .filter(|r| !r.category.starts_with('C') && !matches!(r.icao_type.as_str(), "TWR" | "GND" | "SERV" | "EMER" | "OBST"))
        .collect();
    Ok(Snapshot { now, reports })
}

/// A recorded session (`--adsb-fixture`): either one response or
/// `{"frames": [response, ...]}` (fetched every few seconds, in order).
pub fn parse_recording(text: &str) -> Result<Vec<Snapshot>, String> {
    let v: Value = serde_json::from_str(text).map_err(|e| format!("bad JSON: {e}"))?;
    let mut frames = match v.get("frames").and_then(|f| f.as_array()) {
        Some(fr) => fr.iter().map(parse_value).collect::<Result<Vec<_>, _>>()?,
        None => vec![parse_value(&v)?],
    };
    frames.sort_by(|a, b| a.now.total_cmp(&b.now));
    if frames.is_empty() {
        return Err("no frames".into());
    }
    Ok(frames)
}

/// The request URL for a point query. Only the position and radius are sent
/// (rounded to 0.01°, ~1 km: nothing more precise than needed).
pub fn url(lat: f64, lon: f64, radius_nm: f32) -> String {
    format!("{BASE}/{:.2}/{:.2}/{:.0}", lat, lon, radius_nm.clamp(1.0, MAX_RADIUS_NM))
}

/// The User-Agent sent with every request (generic: no user or machine details).
pub fn user_agent() -> String {
    format!("skyward-flightsim/{}", env!("CARGO_PKG_VERSION"))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = include_str!("../../tests/adsb/sample.json");

    #[test]
    fn parses_the_sample_response() {
        let s = parse(SAMPLE).unwrap();
        assert!((s.now - 1_794_588_119.0).abs() < 1e-6);
        // 7 objects: one without a position, one ground vehicle → 5 aircraft.
        assert_eq!(s.reports.len(), 5, "{:#?}", s.reports.iter().map(|r| &r.callsign).collect::<Vec<_>>());
        let a = &s.reports[0];
        assert_eq!(a.hex, 0x40ab01);
        assert_eq!(a.callsign, "SKY123");
        assert_eq!(a.icao_type, "A339");
        assert_eq!(a.category, "A5");
        assert_eq!(a.alt_baro_ft, Some(16175.0));
        assert_eq!(a.alt_geom_ft, Some(16300.0));
        assert!(!a.on_ground);
        assert_eq!(a.gs_kt, Some(328.4));
        assert_eq!(a.track_deg, Some(290.31));
        assert_eq!(a.true_heading_deg, Some(294.18));
        assert_eq!(a.roll_deg, Some(0.35));
        assert_eq!(a.vrate_fpm, Some(2304.0));
        assert_eq!(a.squawk, Some(1456));
        assert_eq!(a.emergency, None);
        assert_eq!(a.source, Source::Adsb);
        assert!((a.t - (s.now - 0.4)).abs() < 1e-6);
        // On the ground: "ground" altitude.
        let g = s.reports.iter().find(|r| r.callsign == "SKY77K").unwrap();
        assert!(g.on_ground);
        assert_eq!(g.alt_baro_ft, None);
        // MLAT, non-ICAO address, geometric rate fallback, emergency.
        let m = s.reports.iter().find(|r| r.callsign == "GSKYA").unwrap();
        assert_eq!(m.source, Source::Mlat);
        assert_eq!(m.hex, 0x1abc123);
        assert_eq!(m.vrate_fpm, Some(-512.0));
        let e = s.reports.iter().find(|r| r.callsign == "SKY9E").unwrap();
        assert_eq!(e.emergency.as_deref(), Some("general"));
        assert_eq!(e.squawk, Some(7700));
        // No registration survives parsing.
        assert!(!format!("{:?}", s.reports).contains("G-"));
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse("not json").is_err());
        assert!(parse("{\"msg\":\"rate limited\"}").is_err());
        assert!(parse("{\"ac\":[],\"now\":1000}").unwrap().reports.is_empty());
        // Stale positions are dropped.
        let v: Value = serde_json::from_str(r#"{"hex":"abcdef","lat":51.0,"lon":0.0,"seen_pos":120.0}"#).unwrap();
        assert!(parse_aircraft(&v, 0.0).is_none());
        assert_eq!(parse_hex("zz"), None);
        assert_eq!(parse_hex("1000000"), None);
    }

    #[test]
    fn recordings_and_urls() {
        let frames = parse_recording(include_str!("../../tests/adsb/egll_arrivals.json")).unwrap();
        assert!(frames.len() >= 4);
        assert!(frames.windows(2).all(|w| w[1].now > w[0].now));
        assert!(frames.iter().all(|f| f.reports.len() > 20));
        assert_eq!(url(51.4706, -0.4619, 50.0), "https://api.adsb.lol/v2/point/51.47/-0.46/50");
        assert_eq!(url(0.0, 0.0, 999.0), "https://api.adsb.lol/v2/point/0.00/0.00/250");
        assert!(user_agent().starts_with("skyward-flightsim/"));
    }
}
