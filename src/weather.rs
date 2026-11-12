//! Real weather, real local time and the season.
//!
//! * **Data**: the nearest METAR (aviationweather.gov data API, JSON) for
//!   cloud layers, visibility, wind, temperature, QNH and present weather, and
//!   Open-Meteo (no key) for the local UTC offset, the wind aloft (850 hPa),
//!   low / mid / high cloud cover, precipitation, visibility and snow depth.
//!   A background thread refreshes both every ~10 minutes; the last good
//!   result is cached in `cache/weather.json` and used offline.
//! * **Decoding**: [`Metar::parse`] decodes a raw METAR; [`Wx`] merges the
//!   sources into one description of the weather ([`Wx::from_sources`]).
//! * **Mapping**: [`Wx::cloud_spec`] → the volumetric cloud layers,
//!   [`Wx::render`] → haze / fog / rain / snow / wet ground for the renderer,
//!   [`Season::at`] → foliage, grass and crop state for a date and latitude.

use crate::clouds::{CloudSpec, Weather};
use glam::{vec2, Vec2};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use web_time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const USER_AGENT: &str = "skyward-flightsim/0.2";
/// How often live data is refreshed while flying.
pub const REFRESH: Duration = Duration::from_secs(600);
const FT: f32 = 0.3048;
const KT: f32 = 0.514_444;

// ================================================================== METAR

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cover {
    Few,
    Sct,
    Bkn,
    Ovc,
    /// Vertical visibility (sky obscured, e.g. fog).
    Vv,
}

impl Cover {
    /// Sky fraction (mid-range of the okta band).
    pub fn fraction(self) -> f32 {
        match self {
            Cover::Few => 0.19,
            Cover::Sct => 0.44,
            Cover::Bkn => 0.75,
            Cover::Ovc | Cover::Vv => 1.0,
        }
    }
    pub fn code(self) -> &'static str {
        match self {
            Cover::Few => "FEW",
            Cover::Sct => "SCT",
            Cover::Bkn => "BKN",
            Cover::Ovc => "OVC",
            Cover::Vv => "VV",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layer {
    pub cover: Cover,
    /// Base above the station (ft).
    pub base_ft: f32,
    /// Cumulonimbus / towering cumulus.
    pub convective: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Precip {
    None,
    Drizzle,
    Rain,
    Sleet,
    Snow,
    Hail,
}

impl Precip {
    pub fn name(self) -> &'static str {
        match self {
            Precip::None => "none",
            Precip::Drizzle => "drizzle",
            Precip::Rain => "rain",
            Precip::Sleet => "sleet",
            Precip::Snow => "snow",
            Precip::Hail => "hail",
        }
    }
}

/// A decoded METAR (only the parts the sim uses).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Metar {
    pub station: String,
    pub raw: String,
    /// Observation day of month / hour / minute (UTC).
    pub time: Option<(u32, u32, u32)>,
    pub auto: bool,
    /// Degrees true the wind blows *from* (None = variable).
    pub wind_dir: Option<f32>,
    pub wind_kt: f32,
    pub gust_kt: Option<f32>,
    /// Variable direction range (from, to).
    pub wind_var: Option<(f32, f32)>,
    /// Prevailing visibility (m); 10 000 for "9999" / CAVOK.
    pub vis_m: Option<f32>,
    pub cavok: bool,
    pub layers: Vec<Layer>,
    /// NCD / NSC / SKC / CLR / CAVOK: no cloud reported. `ncd` = "not
    /// detected" by an automatic station (there may be cloud above it).
    pub no_cloud: bool,
    pub ncd: bool,
    pub temp_c: Option<f32>,
    pub dew_c: Option<f32>,
    pub qnh_hpa: Option<f32>,
    /// Present weather groups as reported (e.g. "-RA", "BR", "+TSRA").
    pub wx: Vec<String>,
    /// Recent weather (RERA, RESHRA, ...).
    pub recent: Vec<String>,
}

fn num(s: &str) -> Option<f32> {
    s.parse::<f32>().ok()
}

/// "1 1/2", "1/4", "3" statute miles → metres.
fn sm_to_m(s: &str) -> Option<f32> {
    let s = s.trim_start_matches('M').trim_start_matches('P');
    let v = if let Some((a, b)) = s.split_once('/') { num(a)? / num(b)? } else { num(s)? };
    Some(v * 1609.34)
}

impl Metar {
    /// Decode a raw METAR / SPECI. Returns None if it has no station.
    pub fn parse(raw: &str) -> Option<Metar> {
        let mut m = Metar { raw: raw.trim().to_string(), ..Default::default() };
        let toks: Vec<&str> = raw.split_whitespace().collect();
        let mut i = 0;
        while i < toks.len() && matches!(toks[i], "METAR" | "SPECI") {
            i += 1;
        }
        let st = toks.get(i)?;
        if st.len() != 4 || !st.chars().all(|c| c.is_ascii_alphanumeric()) {
            return None;
        }
        m.station = st.to_string();
        i += 1;
        let mut pending_whole_sm: Option<f32> = None;
        while i < toks.len() {
            let t = toks[i];
            i += 1;
            if matches!(t, "RMK" | "TEMPO" | "BECMG" | "NOSIG" | "TREND" | "FM" | "PROB30" | "PROB40") {
                break;
            }
            let b = t.as_bytes();
            // Time: DDHHMMZ
            if t.len() == 7 && t.ends_with('Z') && b[..6].iter().all(u8::is_ascii_digit) {
                m.time = Some((t[0..2].parse().ok()?, t[2..4].parse().ok()?, t[4..6].parse().ok()?));
                continue;
            }
            if t == "AUTO" || t == "COR" || t == "NIL" {
                m.auto |= t == "AUTO";
                continue;
            }
            // Wind: dddff(Gff)KT / VRBffKT / MPS / KMH
            for (unit, k) in [("KT", 1.0), ("MPS", 1.0 / KT), ("KMH", 0.539_957)] {
                if t.ends_with(unit) && t.len() >= 5 + unit.len() {
                    let body = &t[..t.len() - unit.len()];
                    let (dir, rest) = body.split_at(3);
                    let (spd, gust) = match rest.split_once('G') {
                        Some((s, g)) => (s, Some(g)),
                        None => (rest, None),
                    };
                    let d = if dir == "VRB" { Some(None) } else { num(dir).map(Some) };
                    if let (Some(d), Some(s)) = (d, num(spd)) {
                        m.wind_dir = d.map(|x| x % 360.0);
                        m.wind_kt = s * k as f32;
                        m.gust_kt = gust.and_then(num).map(|g| g * k as f32);
                    }
                }
            }
            if t.ends_with("KT") || t.ends_with("MPS") || t.ends_with("KMH") {
                continue;
            }
            // Variable direction dddVddd
            if t.len() == 7 && b[3] == b'V' && b[..3].iter().all(u8::is_ascii_digit) && b[4..].iter().all(u8::is_ascii_digit) {
                m.wind_var = Some((num(&t[..3])?, num(&t[4..])?));
                continue;
            }
            if t == "CAVOK" {
                m.cavok = true;
                m.no_cloud = true;
                m.vis_m = Some(10_000.0);
                continue;
            }
            // Visibility in metres (4 digits, optional NDV / direction).
            if t.len() >= 4 && b[..4].iter().all(u8::is_ascii_digit) && (t.len() == 4 || t.ends_with("NDV") || t[4..].chars().all(|c| "NSEW".contains(c))) {
                if m.vis_m.is_none() {
                    m.vis_m = Some(num(&t[..4])?.max(50.0));
                }
                continue;
            }
            // US visibility: "10SM", "1/2SM", "1 1/2SM", "P6SM".
            if let Some(v) = t.strip_suffix("SM") {
                if let Some(x) = sm_to_m(v) {
                    m.vis_m = Some(x + pending_whole_sm.take().unwrap_or(0.0) * 1609.34);
                }
                continue;
            }
            if t.len() == 1 && b[0].is_ascii_digit() && toks.get(i).is_some_and(|n| n.ends_with("SM") && n.contains('/')) {
                pending_whole_sm = num(t);
                continue;
            }
            if t.starts_with('R') && t.contains('/') && t.len() > 4 && b[1].is_ascii_digit() {
                continue; // runway visual range
            }
            // Cloud groups.
            if matches!(t, "NSC" | "NCD" | "SKC" | "CLR" | "NCD///") {
                m.no_cloud = true;
                m.ncd |= t.starts_with("NCD") || (t == "CLR" && m.auto);
                continue;
            }
            let cover = [("FEW", Cover::Few), ("SCT", Cover::Sct), ("BKN", Cover::Bkn), ("OVC", Cover::Ovc), ("VV", Cover::Vv)]
                .into_iter()
                .find(|(p, _)| t.starts_with(p) && t.len() >= p.len() + 3);
            if let Some((p, c)) = cover {
                let h = &t[p.len()..p.len() + 3];
                let base = if h == "///" { if c == Cover::Vv { 1.0 } else { continue } } else { match num(h) { Some(x) => x, None => continue } };
                let tail = &t[p.len() + 3..];
                m.layers.push(Layer { cover: c, base_ft: base * 100.0, convective: tail.starts_with("CB") || tail.starts_with("TCU") });
                continue;
            }
            // Temperature / dew point: TT/DD with M for minus.
            if let Some((a, d)) = t.split_once('/') {
                let tc = |s: &str| -> Option<f32> {
                    if s.is_empty() || s.starts_with("//") {
                        return None;
                    }
                    let (neg, v) = if let Some(v) = s.strip_prefix('M') { (true, v) } else { (false, s) };
                    (v.len() == 2 && v.bytes().all(|c| c.is_ascii_digit())).then(|| num(v).unwrap() * if neg { -1.0 } else { 1.0 })
                };
                let ok_a = a.len() >= 2 && a.len() <= 3 && (a.starts_with('M') || a.as_bytes()[0].is_ascii_digit());
                if ok_a && !t.contains("SM") {
                    m.temp_c = tc(a);
                    m.dew_c = tc(d);
                    continue;
                }
            }
            // Pressure: Q1013 (hPa) / A2992 (inHg).
            if t.len() == 5 && b[1..].iter().all(u8::is_ascii_digit) {
                if b[0] == b'Q' {
                    m.qnh_hpa = num(&t[1..]);
                    continue;
                }
                if b[0] == b'A' {
                    m.qnh_hpa = num(&t[1..]).map(|v| v / 100.0 * 33.8639);
                    continue;
                }
            }
            if let Some(r) = t.strip_prefix("RE") {
                if is_wx(r) {
                    m.recent.push(r.to_string());
                }
                continue;
            }
            if is_wx(t) {
                m.wx.push(t.to_string());
            }
        }
        Some(m)
    }

    /// Precipitation type and intensity (0.33 light, 0.66 moderate, 1 heavy).
    pub fn precip(&self) -> (Precip, f32, bool) {
        let mut best = (Precip::None, 0.0f32, false);
        for g in &self.wx {
            if g.starts_with("VC") {
                continue;
            }
            let inten = if g.starts_with('-') { 0.33 } else if g.starts_with('+') { 1.0 } else { 0.66 };
            let body = g.trim_start_matches(['-', '+']);
            let showers = body.contains("SH") || body.contains("TS");
            let kind = if body.contains("GR") || body.contains("GS") {
                Precip::Hail
            } else if body.contains("SN") && body.contains("RA") || body.contains("PL") {
                Precip::Sleet
            } else if body.contains("SN") || body.contains("SG") {
                Precip::Snow
            } else if body.contains("RA") || body.contains("UP") {
                Precip::Rain
            } else if body.contains("DZ") {
                Precip::Drizzle
            } else {
                continue;
            };
            if kind > best.0 || (kind == best.0 && inten > best.1) {
                best = (kind, inten, showers);
            }
        }
        best
    }

    pub fn has(&self, code: &str) -> bool {
        self.wx.iter().any(|g| !g.starts_with("VC") && g.contains(code))
    }
}

/// Is `t` a present-weather group ("-SHRA", "BR", "VCTS", "+TSRAGR", ...)?
fn is_wx(t: &str) -> bool {
    let mut s = t.trim_start_matches(['-', '+']);
    s = s.strip_prefix("VC").unwrap_or(s);
    if s.is_empty() || s.len() % 2 != 0 {
        return false;
    }
    const CODES: [&str; 32] = [
        "MI", "BC", "PR", "DR", "BL", "SH", "TS", "FZ", "DZ", "RA", "SN", "SG", "IC", "PL", "GR", "GS", "UP", "BR", "FG", "FU", "VA", "DU", "SA", "HZ", "PY", "PO", "SQ",
        "FC", "SS", "DS", "//", "NS",
    ];
    s.as_bytes().chunks(2).all(|c| CODES.contains(&std::str::from_utf8(c).unwrap_or("")))
}

// ================================================================== Open-Meteo

/// The parts of an Open-Meteo forecast response the sim uses.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OpenMeteo {
    pub utc_offset_s: i64,
    pub tz: String,
    pub temp_c: Option<f32>,
    pub dew_c: Option<f32>,
    pub precip_mm: f32,
    pub rain_mm: f32,
    pub showers_mm: f32,
    pub snowfall_cm: f32,
    pub code: i32,
    pub cloud: f32,
    pub cloud_low: f32,
    pub cloud_mid: f32,
    pub cloud_high: f32,
    pub pressure_hpa: Option<f32>,
    pub wind_kt: Option<f32>,
    pub wind_dir: Option<f32>,
    pub gust_kt: Option<f32>,
    pub vis_m: Option<f32>,
    pub aloft_kt: Option<f32>,
    pub aloft_dir: Option<f32>,
    pub cape: f32,
    pub snow_depth_m: f32,
}

impl OpenMeteo {
    pub fn parse(body: &str) -> Option<OpenMeteo> {
        let v: serde_json::Value = serde_json::from_str(body).ok()?;
        let c = v.get("current")?;
        let f = |k: &str| c.get(k).and_then(|x| x.as_f64()).map(|x| x as f32);
        // First hourly value (the current hour).
        let h = |k: &str| v.get("hourly").and_then(|x| x.get(k)).and_then(|x| x.get(0)).and_then(|x| x.as_f64()).map(|x| x as f32);
        Some(OpenMeteo {
            utc_offset_s: v.get("utc_offset_seconds").and_then(|x| x.as_i64()).unwrap_or(0),
            tz: v.get("timezone_abbreviation").and_then(|x| x.as_str()).unwrap_or("").to_string(),
            temp_c: f("temperature_2m"),
            dew_c: f("dew_point_2m"),
            precip_mm: f("precipitation").unwrap_or(0.0),
            rain_mm: f("rain").unwrap_or(0.0),
            showers_mm: f("showers").unwrap_or(0.0),
            snowfall_cm: f("snowfall").unwrap_or(0.0),
            code: f("weather_code").unwrap_or(0.0) as i32,
            cloud: f("cloud_cover").unwrap_or(0.0) / 100.0,
            cloud_low: f("cloud_cover_low").unwrap_or(0.0) / 100.0,
            cloud_mid: f("cloud_cover_mid").unwrap_or(0.0) / 100.0,
            cloud_high: f("cloud_cover_high").unwrap_or(0.0) / 100.0,
            pressure_hpa: f("pressure_msl"),
            wind_kt: f("wind_speed_10m"),
            wind_dir: f("wind_direction_10m"),
            gust_kt: f("wind_gusts_10m"),
            vis_m: f("visibility"),
            aloft_kt: h("wind_speed_850hPa"),
            aloft_dir: h("wind_direction_850hPa"),
            cape: h("cape").unwrap_or(0.0),
            snow_depth_m: h("snow_depth").unwrap_or(0.0),
        })
    }
}

// ================================================================== merged weather

/// One description of the weather, from live data, a METAR string or a preset.
#[derive(Clone, Debug, PartialEq)]
pub struct Wx {
    /// Where it came from ("EGLC METAR + Open-Meteo", "preset Scattered", ...).
    pub source: String,
    pub metar: Option<Metar>,
    /// Distance from the spawn to the METAR station (km).
    pub station_km: f32,
    pub om: Option<OpenMeteo>,
    /// Surface wind: from (degrees true), speed and gust (kt).
    pub wind_from: f32,
    pub wind_kt: f32,
    pub gust_kt: f32,
    /// Wind at ~1500 m (850 hPa): from, kt.
    pub aloft_from: f32,
    pub aloft_kt: f32,
    pub vis_m: f32,
    pub temp_c: f32,
    pub dew_c: f32,
    pub qnh_hpa: f32,
    /// Cloud layers (base ft AGL), lowest first.
    pub layers: Vec<Layer>,
    pub precip: Precip,
    pub intensity: f32,
    pub showers: bool,
    pub thunder: bool,
    pub fog: bool,
    pub mist: bool,
    pub haze: bool,
    /// Recent rain (wet ground, no falling rain).
    pub recent_rain: bool,
    pub snow_depth_m: f32,
    /// Convective potential (J/kg).
    pub cape: f32,
    /// Seconds east of UTC at the location.
    pub utc_offset_s: Option<i64>,
    pub tz: String,
    /// Unix time of the observation / fetch.
    pub obs_unix: u64,
}

impl Default for Wx {
    fn default() -> Self {
        Wx::preset(Weather::Scattered)
    }
}

/// Wind as a world-space velocity (m/s, +X east, -Z north) from "from" degrees + knots.
pub fn wind_vec(from_deg: f32, kt: f32) -> Vec2 {
    let to = (from_deg + 180.0).to_radians();
    vec2(to.sin(), -to.cos()) * kt * KT
}

impl Wx {
    /// A fair-weather default for a cloud preset (the menu radio buttons / --weather).
    pub fn preset(w: Weather) -> Wx {
        let (layers, vis) = match w {
            Weather::Clear => (vec![], 45_000.0),
            Weather::Few => (vec![Layer { cover: Cover::Few, base_ft: 3800.0, convective: false }], 40_000.0),
            Weather::Scattered => (vec![Layer { cover: Cover::Sct, base_ft: 3600.0, convective: false }], 35_000.0),
            Weather::Broken => (vec![Layer { cover: Cover::Bkn, base_ft: 3400.0, convective: false }], 25_000.0),
            Weather::Overcast => (vec![Layer { cover: Cover::Ovc, base_ft: 3100.0, convective: false }], 18_000.0),
        };
        Wx {
            source: format!("preset {}", w.name()),
            metar: None,
            station_km: 0.0,
            om: None,
            wind_from: 240.0,
            wind_kt: 8.0,
            gust_kt: 0.0,
            aloft_from: 250.0,
            aloft_kt: 18.0,
            vis_m: vis,
            temp_c: 17.0,
            dew_c: 9.0,
            qnh_hpa: 1015.0,
            layers,
            precip: Precip::None,
            intensity: 0.0,
            showers: false,
            thunder: false,
            fog: false,
            mist: false,
            haze: false,
            recent_rain: false,
            snow_depth_m: 0.0,
            cape: 200.0,
            utc_offset_s: None,
            tz: String::new(),
            obs_unix: 0,
        }
    }

    /// Merge a METAR (nearest station), optional fill-in cloud from another
    /// nearby station, and Open-Meteo.
    pub fn from_sources(metar: Option<(Metar, f32)>, fill: Option<&Metar>, om: Option<OpenMeteo>, obs_unix: u64) -> Wx {
        let mut w = Wx::preset(Weather::Clear);
        w.obs_unix = obs_unix;
        w.layers.clear();
        w.vis_m = 30_000.0;
        let mut src = Vec::new();
        if let Some(o) = &om {
            w.utc_offset_s = Some(o.utc_offset_s);
            w.tz = o.tz.clone();
            if let Some(t) = o.temp_c {
                w.temp_c = t;
            }
            if let Some(d) = o.dew_c {
                w.dew_c = d;
            }
            if let Some(p) = o.pressure_hpa {
                w.qnh_hpa = p;
            }
            if let (Some(k), Some(d)) = (o.wind_kt, o.wind_dir) {
                w.wind_kt = k;
                w.wind_from = d;
                w.gust_kt = o.gust_kt.unwrap_or(0.0);
            }
            if let Some(v) = o.vis_m {
                w.vis_m = v.clamp(50.0, 60_000.0);
            }
            w.cape = o.cape;
            w.snow_depth_m = o.snow_depth_m;
            // WMO weather code → precipitation (used when no METAR says otherwise).
            let (p, i) = match o.code {
                51 | 53 | 55 | 56 | 57 => (Precip::Drizzle, [0.33, 0.66, 1.0][((o.code - 51) / 2).clamp(0, 2) as usize]),
                61 | 63 | 65 | 66 | 67 => (Precip::Rain, [0.33, 0.66, 1.0][((o.code - 61) / 2).clamp(0, 2) as usize]),
                71 | 73 | 75 | 77 => (Precip::Snow, [0.33, 0.66, 1.0][((o.code - 71) / 2).clamp(0, 2) as usize]),
                80..=82 => (Precip::Rain, [0.33, 0.66, 1.0][(o.code - 80) as usize]),
                85 | 86 => (Precip::Snow, if o.code == 85 { 0.4 } else { 0.9 }),
                95..=99 => (Precip::Rain, 0.9),
                _ => (Precip::None, 0.0),
            };
            w.precip = p;
            w.intensity = i;
            w.showers = (80..=86).contains(&o.code) || o.code >= 95;
            w.thunder = o.code >= 95;
            w.fog = o.code == 45 || o.code == 48;
            w.recent_rain = o.precip_mm > 0.05;
            if let (Some(k), Some(d)) = (o.aloft_kt, o.aloft_dir) {
                w.aloft_kt = k;
                w.aloft_from = d;
            } else {
                w.aloft_kt = w.wind_kt * 1.8 + 4.0;
                w.aloft_from = w.wind_from + 15.0;
            }
            src.push("Open-Meteo".to_string());
        }
        if let Some((m, km)) = &metar {
            w.station_km = *km;
            if let Some(t) = m.temp_c {
                w.temp_c = t;
            }
            if let Some(d) = m.dew_c {
                w.dew_c = d;
            }
            if let Some(q) = m.qnh_hpa {
                w.qnh_hpa = q;
            }
            w.wind_kt = m.wind_kt;
            w.gust_kt = m.gust_kt.unwrap_or(0.0);
            if let Some(d) = m.wind_dir {
                w.wind_from = d;
            } else if let Some(o) = &om {
                w.wind_from = o.wind_dir.unwrap_or(w.wind_from);
            }
            if om.as_ref().and_then(|o| o.aloft_kt).is_none() {
                w.aloft_kt = w.wind_kt * 1.8 + 4.0;
                w.aloft_from = w.wind_from + 15.0;
            }
            if let Some(v) = m.vis_m {
                // "9999" means 10 km or more: keep a better model estimate.
                // Without a model value: drier air is clearer.
                let dry = ((w.temp_c - w.dew_c) / 12.0).clamp(0.0, 1.0);
                w.vis_m = if v >= 9999.0 { om.as_ref().and_then(|o| o.vis_m).map_or(22_000.0 + 33_000.0 * dry, |x| x.clamp(10_000.0, 60_000.0)) } else { v };
            }
            let (p, i, sh) = m.precip();
            // The METAR is the truth for what is falling right now.
            w.precip = p;
            w.intensity = i;
            w.showers = sh;
            w.thunder = m.has("TS");
            w.fog = m.has("FG") && !m.has("MIFG") && !m.has("BCFG") && !m.has("PRFG");
            w.mist = m.has("BR");
            w.haze = m.has("HZ") || m.has("FU") || m.has("DU");
            w.recent_rain |= m.recent.iter().any(|r| r.contains("RA") || r.contains("DZ")) || p != Precip::None;
            w.layers = m.layers.clone();
            if m.ncd && w.layers.is_empty() {
                // Automatic station saw nothing: borrow a manned neighbour's cloud.
                if let Some(f) = fill {
                    w.layers = f.layers.clone();
                    src.push(format!("cloud from {}", f.station));
                }
            }
            src.insert(0, format!("{} METAR", m.station));
            w.metar = Some(m.clone());
        }
        // No cloud information from a METAR (or only an AUTO "none detected"):
        // fill in from Open-Meteo's low / mid / high cover.
        let metar_says_clear = metar.as_ref().is_some_and(|(m, _)| m.no_cloud && !m.ncd);
        if w.layers.is_empty() && !metar_says_clear {
            if let Some(o) = &om {
                let cov = |c: f32| match c {
                    c if c < 0.1 => None,
                    c if c < 0.3 => Some(Cover::Few),
                    c if c < 0.6 => Some(Cover::Sct),
                    c if c < 0.92 => Some(Cover::Bkn),
                    _ => Some(Cover::Ovc),
                };
                // Cumulus base from the temperature / dew-point spread (~400 ft per °C).
                let lcl = ((w.temp_c - w.dew_c).max(1.0) * 400.0).clamp(800.0, 7000.0);
                if let Some(c) = cov(o.cloud_low) {
                    w.layers.push(Layer { cover: c, base_ft: lcl, convective: o.cape > 600.0 });
                }
                if let Some(c) = cov(o.cloud_mid) {
                    w.layers.push(Layer { cover: c, base_ft: 11_000.0, convective: false });
                }
                if let Some(c) = cov(o.cloud_high) {
                    w.layers.push(Layer { cover: c, base_ft: 26_000.0, convective: false });
                }
            }
        }
        w.layers.sort_by(|a, b| a.base_ft.total_cmp(&b.base_ft));
        w.om = om;
        w.source = if src.is_empty() { "no data".into() } else { src.join(" + ") };
        w
    }

    /// Weather from a raw METAR string (e.g. `--metar "EGLC 121450Z 22012G22KT 3000 RA BR BKN006 OVC012 09/08 Q1002"`).
    pub fn from_metar_str(raw: &str) -> Option<Wx> {
        let m = Metar::parse(raw)?;
        let mut w = Wx::from_sources(Some((m, 0.0)), None, None, 0);
        w.source = format!("{} METAR (fixed)", w.metar.as_ref().map_or("", |m| m.station.as_str()));
        Some(w)
    }

    pub fn wind(&self) -> Vec2 {
        wind_vec(self.wind_from, self.wind_kt)
    }
    pub fn wind_aloft(&self) -> Vec2 {
        wind_vec(self.aloft_from, self.aloft_kt)
    }

    /// Lowest BKN / OVC / VV base (ft), the ceiling.
    pub fn ceiling_ft(&self) -> Option<f32> {
        self.layers.iter().filter(|l| matches!(l.cover, Cover::Bkn | Cover::Ovc | Cover::Vv)).map(|l| l.base_ft).reduce(f32::min)
    }

    /// Flight category (VFR / MVFR / IFR / LIFR).
    pub fn category(&self) -> &'static str {
        let c = self.ceiling_ft().unwrap_or(99_999.0);
        let v = self.vis_m / 1609.34;
        if c < 500.0 || v < 1.0 {
            "LIFR"
        } else if c < 1000.0 || v < 3.0 {
            "IFR"
        } else if c <= 3000.0 || v <= 5.0 {
            "MVFR"
        } else {
            "VFR"
        }
    }

    /// Short human text for the cloud ("FEW034 BKN120", "clear").
    pub fn cloud_text(&self) -> String {
        if self.layers.is_empty() {
            return "clear".into();
        }
        self.layers
            .iter()
            .map(|l| format!("{}{:03}{}", l.cover.code(), (l.base_ft / 100.0).round() as i32, if l.convective { "CB" } else { "" }))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// The volumetric cloud layers for this weather. `sun_up` 0..1 (sin of
    /// the sun's elevation) and the season scale the thermals / cumulus depth.
    pub fn cloud_spec(&self, sun_up: f32, season: &Season) -> CloudSpec {
        let mut s = CloudSpec::clear();
        // Convection: sunshine (season + time of day), instability and showers.
        let insol = (sun_up / 0.75).clamp(0.0, 1.0) * (0.45 + 0.55 * season.summer);
        let unstable = (self.cape / 800.0).clamp(0.0, 1.0);
        let conv = (0.25 + 0.75 * insol + 0.5 * unstable + if self.showers { 0.4 } else { 0.0 }).min(1.3);
        s.cu_depth = 500.0 + 1300.0 * conv;
        s.thermal = conv * 0.9 + 0.1;
        let mut low_used = false;
        let mut strato_used = false;
        for l in &self.layers {
            let base = (l.base_ft * FT).max(60.0);
            let c = l.cover.fraction();
            if l.base_ft < 6500.0 {
                if !low_used {
                    low_used = true;
                    s.cu_base = base;
                    if l.convective {
                        // CB / TCU: big towers, strong lift.
                        s.cumulus = (c * 1.1 + 0.3).min(0.95);
                        s.cu_depth = if self.thunder { 7000.0 } else { 3500.0 };
                        s.thermal = s.thermal.max(1.2);
                        continue;
                    }
                    match l.cover {
                        Cover::Few | Cover::Sct => {
                            s.cumulus = if l.cover == Cover::Few { 0.25 } else { 0.5 };
                            if self.showers {
                                s.cumulus += 0.15;
                                s.cu_depth = s.cu_depth.max(2500.0);
                            }
                        }
                        Cover::Bkn => {
                            s.cumulus = 0.6;
                            s.strato = c;
                            s.st_base = base;
                            s.st_thick = 350.0 + 250.0 * conv;
                            strato_used = true;
                        }
                        Cover::Ovc | Cover::Vv => {
                            s.cumulus = 0.35;
                            s.cu_depth = s.cu_depth.min(700.0);
                            s.strato = 1.0;
                            s.st_base = base;
                            // Deep, rainy decks are thick (nimbostratus); fog tops are shallow.
                            s.st_thick = if l.cover == Cover::Vv {
                                250.0
                            } else if self.precip != Precip::None {
                                1100.0 + 600.0 * self.intensity
                            } else {
                                550.0
                            };
                            s.thermal *= 0.3;
                            strato_used = true;
                        }
                    }
                    continue;
                }
                if !strato_used {
                    strato_used = true;
                    s.strato = c;
                    s.st_base = base.max(s.cu_base + 150.0);
                    s.st_thick = if l.cover == Cover::Ovc { 650.0 } else { 450.0 };
                    continue;
                }
            }
            if l.base_ft < 20_000.0 {
                if s.mid < c {
                    s.mid = c;
                    s.mid_base = base.max(s.st_base + s.st_thick + 200.0).max(s.cu_base + 300.0);
                    s.mid_thick = if l.cover == Cover::Ovc { 900.0 } else { 600.0 };
                }
            } else {
                s.cirrus = s.cirrus.max([0.35, 0.55, 0.8, 1.0][match l.cover {
                    Cover::Few => 0,
                    Cover::Sct => 1,
                    Cover::Bkn => 2,
                    _ => 3,
                }]);
            }
        }
        // Steady (non-shower) precipitation: ragged, shallow cloud under the
        // deck rather than fair-weather towers.
        if self.precip != Precip::None && !self.showers {
            s.cumulus = s.cumulus.min(0.35);
            s.cu_depth = s.cu_depth.min(450.0);
            s.st_thick = s.st_thick.max(900.0);
        }
        // Overcast starves thermals.
        let shade = s.strato.max(s.mid * 0.8);
        s.thermal *= 1.0 - 0.7 * shade;
        if s.st_base <= 0.0 {
            s.st_base = s.cu_base;
        }
        s
    }

    /// Render parameters (haze, fog, precipitation, wet / snowy ground).
    pub fn render(&self, sun_up: f32) -> WxRender {
        // Meteorological visibility V = 3.0 / extinction (5% contrast).
        // Ground-level extinction (/km): Rayleigh 0.0131 + aerosol 0.0133 * haze.
        let sigma = 3.0 / (self.vis_m.max(30.0) * 0.001);
        let humid = 1.0 - ((self.temp_c - self.dew_c) / 12.0).clamp(0.0, 1.0);
        let base_haze = 2.0 + 6.0 * humid;
        let haze_of = |s: f32| ((s - 0.0131) / 0.0133).max(1.0);
        let low_base = self.layers.first().map(|l| l.base_ft * FT).unwrap_or(3000.0);
        let (mut haze, mut fog_ext, mut fog_top) = (haze_of(sigma).clamp(1.2, 14.0), 0.0, 0.0);
        if self.vis_m < 8000.0 {
            if self.haze && !self.fog && !self.mist && self.precip == Precip::None {
                // Dry haze / smoke: spread through the mixed layer.
                haze = haze_of(sigma).clamp(1.2, 60.0);
            } else {
                haze = base_haze.min(haze_of(sigma));
                fog_ext = (sigma - (0.0131 + 0.0133 * haze * 3.0)).max(0.0);
                fog_top = if self.fog {
                    (low_base.min(180.0)).max(60.0)
                } else if self.mist {
                    low_base.clamp(150.0, 450.0)
                } else {
                    // Precipitation fills the air under the cloud base.
                    low_base.clamp(200.0, 1500.0)
                };
            }
        }
        let falling = if self.precip == Precip::None { 0.0 } else { (0.25 + 0.75 * self.intensity) * if self.precip == Precip::Drizzle { 0.6 } else { 1.0 } };
        let snowing = matches!(self.precip, Precip::Snow | Precip::Sleet) || (self.precip != Precip::None && self.temp_c < 1.0);
        let wet = if self.precip != Precip::None && !(snowing && self.temp_c < 0.0) {
            0.55 + 0.45 * self.intensity
        } else if self.recent_rain {
            0.45
        } else if self.fog || (self.temp_c - self.dew_c) < 1.0 {
            0.2
        } else {
            0.0
        };
        let snow_cover = if self.snow_depth_m > 0.005 {
            (self.snow_depth_m / 0.04).clamp(0.35, 1.0)
        } else if snowing && self.temp_c <= 1.0 {
            0.3 + 0.4 * self.intensity
        } else {
            0.0
        };
        let frost = if self.temp_c <= 1.0 && self.dew_c <= 0.0 && snow_cover == 0.0 { ((1.0 - self.temp_c) / 4.0).clamp(0.2, 0.8) * (1.0 - sun_up).max(0.4) } else { 0.0 };
        WxRender {
            haze,
            fog_ext,
            fog_top,
            rain: if snowing { 0.0 } else { falling },
            snow: if snowing { falling } else { 0.0 },
            drizzle: self.precip == Precip::Drizzle,
            wet,
            snow_cover,
            frost,
            wind: self.wind(),
            wind_aloft: self.wind_aloft(),
            gust: ((self.gust_kt - self.wind_kt).max(0.0)) * KT,
            ..Default::default()
        }
    }
}

/// Everything the renderer needs about the weather (besides the cloud field).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WxRender {
    /// Boundary-layer aerosol multiplier (1 = very clear air).
    pub haze: f32,
    /// Extra fog / mist / precipitation extinction near the ground (per km)
    /// and the top of that layer (m).
    pub fog_ext: f32,
    pub fog_top: f32,
    /// Falling rain / snow 0..1.
    pub rain: f32,
    pub snow: f32,
    pub drizzle: bool,
    /// Ground wetness 0..1, snow cover 0..1, hoar frost 0..1.
    pub wet: f32,
    pub snow_cover: f32,
    pub frost: f32,
    /// Surface wind and wind aloft (world m/s, the direction the air moves).
    pub wind: Vec2,
    pub wind_aloft: Vec2,
    /// Gust increment (m/s).
    pub gust: f32,
    /// Cloud drift since t = 0 (m), see `CloudField::drift`.
    pub drift: Vec2,
    /// Extra wind drift of the (faster) cirrus layer (m).
    pub cirrus_drift: Vec2,
    pub season: Season,
    /// Bumped whenever the cloud field changes.
    pub cloud_version: u64,
}

// ================================================================== season

/// Vegetation state for a date and latitude (shader inputs).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Season {
    /// Deciduous leaf cover 0 (bare) .. 1 (full).
    pub leaf: f32,
    /// Autumn colouring 0 (green) .. 1 (fully turned).
    pub autumn: f32,
    /// Fresh spring green 0..1.
    pub spring: f32,
    /// Grass dormancy 0 (lush) .. 1 (dull winter grass).
    pub dormant: f32,
    /// Late-summer dryness 0..1.
    pub dry: f32,
    /// Summer-ness (sunshine / convection) 0 winter .. 1 midsummer.
    pub summer: f32,
    /// Year phase 0..1 (northern-hemisphere equivalent: 0 = 1 Jan), for crops.
    pub year: f32,
    /// 0 in the tropics (evergreen) .. 1 at mid latitudes.
    pub strength: f32,
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

impl Season {
    pub fn at(day_of_year: u32, lat: f64) -> Season {
        let mut d = day_of_year as f32;
        if lat < 0.0 {
            d = (d + 182.0) % 365.0;
        }
        let alat = lat.abs() as f32;
        // Later spring / earlier autumn further north (≈2 days per degree).
        let shift = ((alat - 51.5) * 2.0).clamp(-25.0, 25.0);
        let strength = smooth(20.0, 38.0, alat);
        let leaf_out = smooth(100.0 + shift, 135.0 + shift, d);
        let leaf_fall = smooth(292.0 - shift, 335.0 - shift, d);
        let leaf = 1.0 - strength * (1.0 - leaf_out * (1.0 - leaf_fall)).clamp(0.0, 1.0);
        let autumn = strength * smooth(268.0 - shift, 312.0 - shift, d) * (1.0 - smooth(345.0, 360.0, d));
        let spring = strength * smooth(100.0 + shift, 125.0 + shift, d) * (1.0 - smooth(150.0 + shift, 185.0 + shift, d));
        let dormant = strength * ((1.0 - smooth(55.0 + shift, 115.0 + shift, d)) + smooth(295.0 - shift, 345.0 - shift, d)).min(1.0);
        let dry = strength * smooth(180.0, 215.0, d) * (1.0 - smooth(245.0, 275.0, d));
        let summer = 0.5 + 0.5 * ((d - 172.0) / 365.0 * std::f32::consts::TAU).cos() * (0.3 + 0.7 * strength);
        Season { leaf, autumn, spring, dormant, dry, summer, year: d / 365.0, strength }
    }
}

// ================================================================== time

/// Civil date-time from unix seconds (UTC): (year, month, day, hour, minute, second).
pub fn civil(unix: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = unix.div_euclid(86_400);
    let secs = unix.rem_euclid(86_400);
    // Howard Hinnant's days_from_civil inverse.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d, (secs / 3600) as u32, (secs / 60 % 60) as u32, (secs % 60) as u32)
}

pub fn unix_now() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

/// UTC offset estimate when no time zone is known: nearest 15° of longitude,
/// plus European/US-style summer time in the mid latitudes.
pub fn guess_offset_s(lat: f64, lon: f64, unix: i64) -> i64 {
    let (_, m, d, ..) = civil(unix);
    let doy = crate::atmosphere::day_of_year(m, d) as f64;
    let summer = if lat >= 25.0 { (86.0..=300.0).contains(&doy) } else if lat <= -25.0 { !(86.0..=300.0).contains(&doy) } else { false };
    (((lon / 15.0).round() + if summer { 1.0 } else { 0.0 }) * 3600.0) as i64
}

/// The real time at a place: (local clock hours, day of year, local date
/// text, UTC offset seconds) and the sim's sun clock (hours in the time
/// convention of `atmosphere::sun_direction`, so the sun is exactly where it
/// is now).
#[derive(Clone, Debug, PartialEq)]
pub struct RealTime {
    pub local_hours: f32,
    pub day_of_year: u32,
    pub sun_hours: f32,
    pub sun_doy: u32,
    pub text: String,
}

pub fn real_time(lat: f64, lon: f64, unix: i64, offset_s: Option<i64>) -> RealTime {
    let off = offset_s.unwrap_or_else(|| guess_offset_s(lat, lon, unix));
    let (y, m, d, hh, mm, _) = civil(unix + off);
    let local_hours = hh as f32 + mm as f32 / 60.0;
    // sun_direction uses its own civil-time convention for (lon, doy): feed it
    // UTC shifted by that convention's offset.
    let (_, um, ud, uh, umin, us) = civil(unix);
    let utc_doy = crate::atmosphere::day_of_year(um, ud);
    let conv = crate::atmosphere::civil_offset_h(lat, lon, utc_doy);
    let mut sun_hours = uh as f32 + umin as f32 / 60.0 + us as f32 / 3600.0 + conv;
    let mut sun_doy = utc_doy;
    if sun_hours >= 24.0 {
        sun_hours -= 24.0;
        sun_doy = sun_doy % 365 + 1;
    } else if sun_hours < 0.0 {
        sun_hours += 24.0;
        sun_doy = if sun_doy == 1 { 365 } else { sun_doy - 1 };
    }
    let oh = off as f32 / 3600.0;
    RealTime {
        local_hours,
        day_of_year: crate::atmosphere::day_of_year(m, d),
        sun_hours,
        sun_doy,
        text: format!("{y}-{m:02}-{d:02} {hh:02}:{mm:02} (UTC{}{})", if oh >= 0.0 { "+" } else { "" }, if oh.fract() == 0.0 { format!("{oh:.0}") } else { format!("{oh:.1}") }),
    }
}

// ================================================================== fetching

#[cfg(not(target_arch = "wasm32"))]
fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(6)).timeout(Duration::from_secs(15)).user_agent(USER_AGENT).build()
}

#[cfg(not(target_arch = "wasm32"))]
fn get(agent: &ureq::Agent, url: &str) -> Result<String, String> {
    agent.get(url).call().map_err(|e| e.to_string())?.into_string().map_err(|e| e.to_string())
}

/// Raw responses from one fetch (what is cached on disk).
#[derive(Clone, Debug, Default)]
pub struct Fetched {
    pub unix: i64,
    pub lat: f64,
    pub lon: f64,
    pub metar_json: Option<String>,
    pub om_json: Option<String>,
}

fn haversine_km(a: (f64, f64), b: (f64, f64)) -> f32 {
    let (la1, lo1, la2, lo2) = (a.0.to_radians(), a.1.to_radians(), b.0.to_radians(), b.1.to_radians());
    let h = ((la2 - la1) / 2.0).sin().powi(2) + la1.cos() * la2.cos() * ((lo2 - lo1) / 2.0).sin().powi(2);
    (2.0 * 6371.0 * h.sqrt().asin()) as f32
}

/// METAR stations in a box around a place (`box_i` 0: near, 1: wider).
fn metar_url(lat: f64, lon: f64, box_i: usize) -> String {
    let (dlat, dlon) = [(0.6, 1.0), (1.6, 2.6)][box_i];
    format!("https://aviationweather.gov/api/data/metar?bbox={:.3},{:.3},{:.3},{:.3}&format=json", lat - dlat, lon - dlon, lat + dlat, lon + dlon)
}

fn open_meteo_url(lat: f64, lon: f64) -> String {
    format!(
        "https://api.open-meteo.com/v1/forecast?latitude={lat:.4}&longitude={lon:.4}&current=temperature_2m,dew_point_2m,precipitation,rain,showers,snowfall,weather_code,cloud_cover,cloud_cover_low,cloud_cover_mid,cloud_cover_high,pressure_msl,wind_speed_10m,wind_direction_10m,wind_gusts_10m,visibility&hourly=wind_speed_850hPa,wind_direction_850hPa,cape,snow_depth&forecast_hours=1&wind_speed_unit=kn&timezone=auto"
    )
}

impl Fetched {
    /// Browser: Open-Meteo only — aviationweather.gov sends no CORS headers,
    /// so pages cannot read its METARs (the weather is built from the
    /// forecast model alone, as natively when METAR is down).
    #[cfg(target_arch = "wasm32")]
    async fn fetch_web(lat: f64, lon: f64) -> Result<Fetched, String> {
        use crate::platform::web;
        let t = Duration::from_secs(15);
        let mut f = Fetched { unix: unix_now(), lat, lon, ..Default::default() };
        let mut errs = Vec::new();
        match web::get(&open_meteo_url(lat, lon), t).await.map(|b| String::from_utf8_lossy(&b).into_owned()) {
            Ok(b) if b.contains("\"current\"") => f.om_json = Some(b),
            Ok(b) => errs.push(format!("Open-Meteo: {}", b.chars().take(120).collect::<String>())),
            Err(e) => errs.push(format!("Open-Meteo: {e}")),
        }
        if f.metar_json.is_none() && f.om_json.is_none() {
            return Err(errs.join("; "));
        }
        Ok(f)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn fetch(lat: f64, lon: f64) -> Result<Fetched, String> {
        let ag = agent();
        let mut f = Fetched { unix: unix_now(), lat, lon, ..Default::default() };
        let mut errs = Vec::new();
        // Stations in a box around the spawn, widened if it is empty.
        for i in 0..2 {
            let url = metar_url(lat, lon, i);
            match get(&ag, &url) {
                Ok(b) if b.trim_start().starts_with('[') && b.trim() != "[]" => {
                    f.metar_json = Some(b);
                    break;
                }
                Ok(_) => {}
                Err(e) => {
                    errs.push(format!("METAR: {e}"));
                    break;
                }
            }
        }
        let url = open_meteo_url(lat, lon);
        match get(&ag, &url) {
            Ok(b) if b.contains("\"current\"") => f.om_json = Some(b),
            Ok(b) => errs.push(format!("Open-Meteo: {}", b.chars().take(120).collect::<String>())),
            Err(e) => errs.push(format!("Open-Meteo: {e}")),
        }
        if f.metar_json.is_none() && f.om_json.is_none() {
            return Err(errs.join("; "));
        }
        Ok(f)
    }

    fn path() -> PathBuf {
        PathBuf::from("cache").join("weather.json")
    }

    fn save(&self) {
        let v = serde_json::json!({
            "unix": self.unix, "lat": self.lat, "lon": self.lon,
            "metar": self.metar_json, "open_meteo": self.om_json,
        });
        let _ = std::fs::create_dir_all("cache");
        let _ = std::fs::write(Self::path(), serde_json::to_string(&v).unwrap_or_default());
    }

    fn load() -> Option<Fetched> {
        let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(Self::path()).ok()?).ok()?;
        Some(Fetched {
            unix: v.get("unix")?.as_i64()?,
            lat: v.get("lat")?.as_f64()?,
            lon: v.get("lon")?.as_f64()?,
            metar_json: v.get("metar").and_then(|x| x.as_str()).map(String::from),
            om_json: v.get("open_meteo").and_then(|x| x.as_str()).map(String::from),
        })
    }

    /// Decode into one weather description for the spawn location.
    pub fn decode(&self) -> Wx {
        let mut stations: Vec<(Metar, f32)> = Vec::new();
        if let Some(Ok(serde_json::Value::Array(a))) = self.metar_json.as_deref().map(serde_json::from_str::<serde_json::Value>) {
            for s in a {
                let (Some(raw), Some(la), Some(lo)) = (s.get("rawOb").and_then(|x| x.as_str()), s.get("lat").and_then(|x| x.as_f64()), s.get("lon").and_then(|x| x.as_f64())) else {
                    continue;
                };
                if let Some(m) = Metar::parse(raw) {
                    stations.push((m, haversine_km((self.lat, self.lon), (la, lo))));
                }
            }
        }
        stations.sort_by(|a, b| a.1.total_cmp(&b.1));
        let fill = stations.iter().skip(1).find(|(m, km)| *km < 60.0 && !m.layers.is_empty()).map(|(m, _)| m);
        let om = self.om_json.as_deref().and_then(OpenMeteo::parse);
        Wx::from_sources(stations.first().cloned(), fill, om, self.unix as u64)
    }
}

/// Background live-weather fetcher with an on-disk cache.
pub struct Live {
    lat: f64,
    lon: f64,
    rx: Option<Receiver<Result<Fetched, String>>>,
    /// Most recent data (fresh or from the cache).
    pub data: Option<Fetched>,
    /// The decoded weather of `data`.
    pub wx: Option<Wx>,
    /// Bumped whenever `wx` changes.
    pub version: u64,
    pub status: String,
    pub last_ok: Option<Instant>,
}

impl Live {
    /// Start fetching for a location (if `enabled`). A cached result for the
    /// same area (< 3 h old) is available immediately.
    pub fn new(lat: f64, lon: f64, enabled: bool) -> Live {
        let mut l = Live { lat, lon, rx: None, data: None, wx: None, version: 0, status: "live weather off".into(), last_ok: None };
        if enabled {
            l.start();
        }
        l
    }

    pub fn running(&self) -> bool {
        self.rx.is_some()
    }

    /// Follow a long flight: once `lat`/`lon` is more than 50 km from where
    /// the weather was fetched, fetch for the new place instead (the current
    /// weather stays until the new data arrives).
    pub fn relocate(&mut self, lat: f64, lon: f64) {
        if haversine_km((self.lat, self.lon), (lat, lon)) < 50.0 {
            return;
        }
        let was_running = self.rx.is_some();
        self.lat = lat;
        self.lon = lon;
        self.rx = None; // the old thread stops at its next send
        if was_running {
            self.start();
        }
    }

    /// Start the fetch thread (no-op when running).
    pub fn start(&mut self) {
        if self.rx.is_some() {
            return;
        }
        if let Some(c) = Fetched::load() {
            let age = unix_now() - c.unix;
            if age < 3 * 3600 && haversine_km((c.lat, c.lon), (self.lat, self.lon)) < 40.0 && self.data.is_none() {
                self.status = format!("cached weather ({} min old)", age / 60);
                self.set(c);
            }
        }
        let (tx, rx) = channel();
        let (lat, lon) = (self.lat, self.lon);
        #[cfg(target_arch = "wasm32")]
        crate::platform::web::spawn_local(async move {
            loop {
                let r = Fetched::fetch_web(lat, lon).await;
                let ok = r.is_ok();
                if tx.send(r).is_err() {
                    return;
                }
                crate::platform::web::sleep(if ok { REFRESH } else { Duration::from_secs(60) }).await;
            }
        });
        #[cfg(not(target_arch = "wasm32"))]
        std::thread::Builder::new()
            .name("weather".into())
            .spawn(move || loop {
                let r = Fetched::fetch(lat, lon);
                if let Ok(f) = &r {
                    f.save();
                }
                let ok = r.is_ok();
                if tx.send(r).is_err() {
                    return;
                }
                // Retry sooner after a failure.
                std::thread::sleep(if ok { REFRESH } else { Duration::from_secs(60) });
            })
            .ok();
        self.rx = Some(rx);
        if self.data.is_none() {
            self.status = "fetching live weather…".into();
        }
    }

    fn set(&mut self, f: Fetched) {
        let wx = f.decode();
        self.data = Some(f);
        if self.wx.as_ref() != Some(&wx) {
            self.wx = Some(wx);
            self.version += 1;
        }
    }

    /// Take any finished fetch. True if the weather changed.
    pub fn poll(&mut self) -> bool {
        let v0 = self.version;
        while let Some(rx) = &self.rx {
            match rx.try_recv() {
                Ok(Ok(f)) => {
                    self.status = format!("live: {}", f.decode().source);
                    self.last_ok = Some(Instant::now());
                    self.set(f);
                }
                Ok(Err(e)) => {
                    self.status = format!("offline ({}){}", e.chars().take(80).collect::<String>(), if self.data.is_some() { ", using cached weather" } else { "" });
                    eprintln!("weather: {e}");
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    self.rx = None;
                }
            }
        }
        self.version != v0
    }

    #[cfg(not(target_arch = "wasm32"))]
    /// Block until the first fetch has finished (or `timeout`), for headless captures.
    pub fn wait(&mut self, timeout: Duration) {
        let t = Instant::now();
        while self.rx.is_some() && self.last_ok.is_none() && t.elapsed() < timeout {
            let before = self.status.clone();
            self.poll();
            if self.status != before && self.status.starts_with("offline") {
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

// ================================================================== tests

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_london_city() {
        let m = Metar::parse("METAR EGLC 121520Z AUTO 35009KT 310V040 9999 NCD 08/01 Q1007").unwrap();
        assert_eq!(m.station, "EGLC");
        assert_eq!(m.time, Some((12, 15, 20)));
        assert!(m.auto && m.ncd && m.no_cloud);
        assert_eq!(m.wind_dir, Some(350.0));
        assert_eq!(m.wind_kt, 9.0);
        assert_eq!(m.wind_var, Some((310.0, 40.0)));
        assert_eq!(m.vis_m, Some(9999.0));
        assert_eq!((m.temp_c, m.dew_c, m.qnh_hpa), (Some(8.0), Some(1.0), Some(1007.0)));
    }

    #[test]
    fn parse_wet_low_cloud() {
        let m = Metar::parse("EGLL 121450Z 22015G27KT 190V250 3000 -SHRA BR FEW008 BKN012CB OVC025 M01/M03 Q0998 RERA NOSIG").unwrap();
        assert_eq!(m.gust_kt, Some(27.0));
        assert_eq!(m.vis_m, Some(3000.0));
        assert_eq!(m.layers.len(), 3);
        assert_eq!(m.layers[1], Layer { cover: Cover::Bkn, base_ft: 1200.0, convective: true });
        assert_eq!((m.temp_c, m.dew_c), (Some(-1.0), Some(-3.0)));
        assert_eq!(m.qnh_hpa, Some(998.0));
        let (p, i, sh) = m.precip();
        assert_eq!((p, sh), (Precip::Rain, true));
        assert!((i - 0.33).abs() < 0.01);
        assert!(m.has("BR"));
    }

    #[test]
    fn parse_us_and_fog() {
        let m = Metar::parse("METAR KSFO 121556Z 00000KT 1/4SM FG VV002 12/12 A3001").unwrap();
        assert!((m.vis_m.unwrap() - 402.0).abs() < 2.0);
        assert_eq!(m.layers[0].cover, Cover::Vv);
        assert!((m.qnh_hpa.unwrap() - 1016.3).abs() < 0.2);
        let m = Metar::parse("KJFK 121551Z VRB03KT 1 1/2SM -SN BR OVC008 M02/M03 A2990").unwrap();
        assert_eq!(m.wind_dir, None);
        assert!((m.vis_m.unwrap() - 2414.0).abs() < 2.0);
        assert_eq!(m.precip().0, Precip::Snow);
        let m = Metar::parse("LFPG 121530Z 27012MPS CAVOK 15/05 Q1020").unwrap();
        assert!(m.cavok && m.layers.is_empty());
        assert!((m.wind_kt - 23.3).abs() < 0.2);
    }

    #[test]
    fn spec_from_layers() {
        let s = Season::at(316, 51.5);
        let w = Wx::from_metar_str("EGLC 121450Z 22012KT 9999 FEW030 SCT045 BKN120 BKN280 10/04 Q1012").unwrap();
        let c = w.cloud_spec(0.3, &s);
        assert!((c.cumulus - 0.25).abs() < 1e-3 && (c.cu_base - 914.0).abs() < 2.0);
        assert!((c.strato - 0.44).abs() < 1e-3 && c.st_base > 1300.0);
        assert!(c.mid > 0.7 && c.mid_base > 3600.0);
        assert!(c.cirrus > 0.7);
        let w = Wx::from_metar_str("EGLC 121450Z 22012KT 3000 RA BR OVC006 09/08 Q1002").unwrap();
        let c = w.cloud_spec(0.3, &s);
        assert_eq!(c.strato, 1.0);
        assert!((c.st_base - 183.0).abs() < 2.0);
        let r = w.render(0.3);
        assert!(r.rain > 0.5 && r.wet > 0.7 && r.fog_ext > 0.3 && r.fog_top < 400.0, "{r:?}");
        let w = Wx::from_metar_str("EGLC 121450Z 00000KT 0200 FG VV001 04/04 Q1022").unwrap();
        let r = w.render(0.1);
        assert!(r.fog_ext > 10.0 && r.fog_top <= 180.0, "{r:?}");
    }

    #[test]
    fn ncd_takes_neighbour_cloud_and_om() {
        let a = Metar::parse("METAR EGLC 121520Z AUTO 35009KT 9999 NCD 08/01 Q1007").unwrap();
        let b = Metar::parse("METAR EGKB 121520Z 01010KT 9999 FEW034 06/00 Q1006").unwrap();
        let w = Wx::from_sources(Some((a.clone(), 1.0)), Some(&b), None, 0);
        assert_eq!(w.cloud_text(), "FEW034");
        let om = OpenMeteo { cloud_low: 0.5, cloud_high: 0.2, ..Default::default() };
        let w = Wx::from_sources(Some((a, 1.0)), None, Some(om), 0);
        assert_eq!(w.layers.len(), 2);
        assert_eq!(w.layers[0].cover, Cover::Sct);
        assert!(w.layers[0].base_ft > 2000.0, "LCL from the spread: {:?}", w.layers);
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil(0), (1970, 1, 1, 0, 0, 0));
        assert_eq!(civil(1_794_497_580), (2026, 11, 12, 15, 33, 0));
        assert_eq!(civil(951_782_400), (2000, 2, 29, 0, 0, 0));
        // London in November is on GMT, in July on BST.
        assert_eq!(guess_offset_s(51.5, 0.0, 1_794_497_580), 0);
        assert_eq!(guess_offset_s(51.5, 0.0, 1_783_000_000), 3600);
        let t = real_time(51.5, 0.0, 1_794_497_580, Some(0));
        assert!((t.local_hours - 15.55).abs() < 0.01 && t.day_of_year == 316 && t.sun_doy == 316);
        assert!((t.sun_hours - 15.55).abs() < 0.01);
    }

    #[test]
    fn seasons() {
        let nov = Season::at(316, 51.5);
        assert!(nov.leaf > 0.25 && nov.leaf < 0.7 && nov.autumn > 0.9, "{nov:?}");
        let jul = Season::at(196, 51.5);
        assert!(jul.leaf > 0.99 && jul.autumn < 0.01 && jul.summer > 0.9);
        let jan = Season::at(15, 51.5);
        assert!(jan.leaf < 0.01 && jan.dormant > 0.9);
        // Southern hemisphere is half a year out; the tropics stay green.
        assert!(Season::at(196, -35.0).leaf < 0.3);
        assert!(Season::at(15, 5.0).leaf > 0.99);
    }
}

// ================================================================== wind field

/// The wind the aircraft flies through: a mean profile (surface wind → wind
/// aloft, log-law gradient near the ground), gusts and turbulence carried
/// along with the air ("frozen" turbulence advected by the mean wind), with
/// mechanical turbulence over rough (city) ground and convective turbulence
/// near cumulus / thermals.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindField {
    /// Wind at 10 m and at ~1500 m (m/s, world xz, direction of motion).
    pub surface: Vec2,
    pub aloft: Vec2,
    /// Gust increment (m/s): METAR gust − mean.
    pub gust: f32,
    /// Background turbulence intensity (m/s rms).
    pub turb: f32,
}

impl Default for WindField {
    fn default() -> Self {
        WindField { surface: Vec2::ZERO, aloft: Vec2::ZERO, gust: 0.0, turb: 0.0 }
    }
}

fn hash3(x: i32, y: i32, z: i32, k: u32) -> f32 {
    let mut h = (x as u32).wrapping_mul(0x8da6_b343) ^ (y as u32).wrapping_mul(0xd816_3841) ^ (z as u32).wrapping_mul(0xcb1a_b31f) ^ k.wrapping_mul(0x9e37_79b9);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2c1b_3c6d);
    h ^= h >> 12;
    h = h.wrapping_mul(0x297a_2d39);
    h ^= h >> 15;
    (h >> 8) as f32 * (1.0 / 16_777_216.0)
}

/// Smooth value noise in [-1, 1].
fn vnoise3(p: glam::Vec3, k: u32) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = f * f * (glam::Vec3::splat(3.0) - 2.0 * f);
    let (x, y, z) = (i.x as i32, i.y as i32, i.z as i32);
    let mut acc = 0.0;
    for c in 0..8 {
        let (dx, dy, dz) = (c & 1, (c >> 1) & 1, (c >> 2) & 1);
        let w = (if dx == 1 { u.x } else { 1.0 - u.x }) * (if dy == 1 { u.y } else { 1.0 - u.y }) * (if dz == 1 { u.z } else { 1.0 - u.z });
        acc += w * hash3(x + dx, y + dy, z + dz, k);
    }
    acc * 2.0 - 1.0
}

impl WindField {
    pub fn from_wx(wx: &Wx) -> WindField {
        let surface = wx.wind();
        let gust = ((wx.gust_kt - wx.wind_kt).max(0.0)) * KT;
        WindField { surface, aloft: wx.wind_aloft(), gust, turb: 0.25 + 0.04 * surface.length() + 0.2 * gust }
    }

    /// Mean wind at height `h` (m) over ground of roughness `rough` (0 open .. 1 city).
    pub fn mean(&self, h: f32, rough: f32) -> Vec2 {
        let h = h.max(0.0);
        let z0 = 0.03 + 0.6 * rough.clamp(0.0, 1.0);
        let ref_h = 10.0f32.max(z0 * 4.0);
        if h < ref_h {
            let f = ((h.max(z0 * 1.2) / z0).ln() / (ref_h / z0).ln()).clamp(0.0, 1.0);
            return self.surface * f;
        }
        // Boundary layer: stronger and veering with height up to ~1500 m.
        let t = ((h - ref_h) / 1500.0).clamp(0.0, 1.0);
        let k = t.powf(0.6);
        self.surface.lerp(self.aloft, k)
    }

    /// Wind vector at `p` and time `t` (s): mean + gusts + turbulence.
    /// `conv` 0..1: convective turbulence (in / under cumulus, near thermals).
    pub fn at(&self, p: glam::Vec3, t: f32, rough: f32, conv: f32) -> glam::Vec3 {
        if !p.is_finite() || !t.is_finite() {
            return glam::Vec3::ZERO;
        }
        let h = p.y.max(0.0);
        let m = self.mean(h, rough);
        let spd = m.length();
        let dir = if spd > 0.1 { m / spd } else { Vec2::X };
        // Turbulence is carried with the air: sample a frozen field at p - m t.
        let a = vec2(p.x, p.z) - m * (t % 100_000.0);
        // Gusts: slow (~10-20 s) surges along the wind.
        let g = vnoise3(glam::Vec3::new(a.x / 220.0, h / 300.0, t * 0.06), 11);
        let surge = smooth(0.15, 0.85, g) * self.gust * (1.0 - smooth(800.0, 2000.0, h));
        // Turbulence intensity: background + mechanical (wind over rough
        // ground, decaying with height) + convective.
        let mech = (0.08 + 0.12 * rough.clamp(0.0, 1.0)) * spd * (1.0 - smooth(50.0, 600.0, h));
        let sigma = (self.turb * (1.0 - 0.6 * smooth(1500.0, 3000.0, h)) + mech + 1.4 * conv.clamp(0.0, 1.0)).min(6.0);
        let q = glam::Vec3::new(a.x, h, a.y);
        let n = |k: u32| vnoise3(q / 70.0 + glam::Vec3::splat(t * 0.02), k) * 0.75 + vnoise3(q / 18.0, k + 7) * 0.25;
        // Vertical gusts die out at the ground.
        let wv = n(3) * 0.7 * smooth(0.0, 30.0, h);
        let turb = glam::Vec3::new(n(1), wv, n(2)) * sigma;
        glam::Vec3::new(m.x + dir.x * surge, 0.0, m.y + dir.y * surge) + turb
    }
}

#[cfg(test)]
mod wind_tests {
    use super::*;
    use glam::vec3;

    #[test]
    fn profile_and_bounds() {
        let wx = Wx::from_metar_str("EGLC 121450Z 22015G30KT 9999 SCT030 10/04 Q1012").unwrap();
        let wf = WindField::from_wx(&wx);
        // Wind gradient: calmer near the ground, stronger aloft.
        let w10 = wf.mean(10.0, 0.0).length();
        assert!((w10 - 15.0 * KT).abs() < 0.2);
        assert!(wf.mean(1.0, 0.0).length() < 0.7 * w10);
        assert!(wf.mean(0.0, 0.0).length() < 0.25 * w10);
        assert!(wf.mean(2000.0, 0.0).length() > w10);
        // Blowing from the south-west: toward the north-east (+X, -Z).
        let m = wf.mean(10.0, 0.0);
        assert!(m.x > 0.0 && m.y < 0.0);
        // Gusts and turbulence: bounded, finite, zero-mean-ish around the mean.
        let mut max_dev: f32 = 0.0;
        let mut sum = glam::Vec3::ZERO;
        let n = 20_000;
        for i in 0..n {
            let t = i as f32 * 0.05;
            let p = vec3((i * 37 % 5000) as f32 - 2500.0, 20.0 + (i % 50) as f32 * 30.0, (i * 91 % 7000) as f32);
            let w = wf.at(p, t, (i % 3) as f32 * 0.5, (i % 5) as f32 * 0.25);
            assert!(w.is_finite());
            let dev = w - glam::Vec3::new(wf.mean(p.y, (i % 3) as f32 * 0.5).x, 0.0, wf.mean(p.y, (i % 3) as f32 * 0.5).y);
            max_dev = max_dev.max(dev.length());
            sum += w;
        }
        assert!(max_dev < 15.0 * KT + 12.0, "gust/turbulence excursion {max_dev}");
        assert!(max_dev > 3.0, "it should actually be gusty: {max_dev}");
        assert!((sum / n as f32).y.abs() < 0.5);
        assert!(!wf.at(vec3(f32::NAN, 0.0, 0.0), 1.0, 0.0, 0.0).is_nan());
        // Calm day: next to nothing.
        let calm = WindField::from_wx(&Wx::from_metar_str("EGLC 121450Z 00000KT CAVOK 10/04 Q1012").unwrap());
        assert!(calm.at(vec3(10.0, 300.0, 5.0), 3.0, 0.0, 0.0).length() < 2.0);
    }
}

// ================================================================== HUD

fn season_name(s: &Season, doy: u32) -> String {
    if s.strength < 0.3 {
        return "tropical".into();
    }
    let n = if s.leaf < 0.08 && s.dormant > 0.6 {
        "winter"
    } else if s.leaf < 0.9 && s.autumn > 0.5 {
        if s.leaf < 0.5 { "late autumn" } else { "autumn" }
    } else if s.spring > 0.3 || (s.leaf < 0.95 && doy < 200) {
        "spring"
    } else if s.dry > 0.4 {
        "late summer"
    } else {
        "summer"
    };
    format!("{n} (leaves {:.0}%)", s.leaf * 100.0)
}

fn dir_text(from: f32, kt: f32, gust: f32) -> String {
    if kt < 1.0 {
        return "calm".into();
    }
    let g = if gust > kt + 1.0 { format!(" G{gust:.0}") } else { String::new() };
    format!("{:03.0}° {:.0} kt{g}", (from.rem_euclid(360.0) / 10.0).round() * 10.0 % 360.0, kt)
}

fn vis_text(m: f32) -> String {
    if m >= 9999.0 {
        format!("{:.0} km", (m / 1000.0).min(60.0))
    } else if m >= 5000.0 {
        format!("{:.0} km", m / 1000.0)
    } else {
        format!("{:.0} m", (m / 50.0).round() * 50.0)
    }
}

/// Wind arrow (pointing where the air goes) with a label, in screen space.
fn wind_arrow(p: &egui::Painter, c: egui::Pos2, r: f32, dir: egui::Vec2, col: egui::Color32) {
    use egui::{Shape, Stroke};
    let d = if dir.length() > 1e-3 { dir.normalized() } else { egui::Vec2::new(0.0, 1.0) };
    let n = egui::Vec2::new(-d.y, d.x);
    let (a, b) = (c - d * r, c + d * r);
    p.line_segment([a, b - d * 4.0], Stroke::new(2.5, col));
    p.add(Shape::convex_polygon(vec![b, b - d * 9.0 + n * 5.0, b - d * 9.0 - n * 5.0], col, Stroke::NONE));
    // Feathered tail.
    p.line_segment([a, a - d * 5.0 + n * 5.0], Stroke::new(2.0, col));
    p.line_segment([a, a - d * 5.0 - n * 5.0], Stroke::new(2.0, col));
}

/// Wind arrow + speed in the top-left corner of the (north-up) moving map.
pub fn map_wind(p: &egui::Painter, mr: egui::Rect, world: &crate::world::World) {
    use egui::{Align2, Color32, FontId, Pos2};
    let w = &world.wx;
    let c = Pos2::new(mr.min.x + 26.0, mr.min.y + 26.0);
    p.circle_filled(c, 20.0, Color32::from_rgba_unmultiplied(10, 12, 16, 170));
    let v = w.wind();
    if w.wind_kt >= 1.0 {
        wind_arrow(p, c, 13.0, egui::Vec2::new(v.x, v.y), Color32::from_rgb(120, 210, 255));
    }
    let label = dir_text(w.wind_from, w.wind_kt, w.gust_kt);
    let at = Pos2::new(mr.min.x + 5.0, c.y + 23.0);
    let r = p.text(at, Align2::LEFT_TOP, &label, FontId::monospace(11.0), Color32::TRANSPARENT);
    p.rect_filled(r.expand(2.0), 3.0, Color32::from_rgba_unmultiplied(10, 12, 16, 170));
    p.text(at, Align2::LEFT_TOP, label, FontId::monospace(11.0), Color32::from_rgb(120, 210, 255));
}

/// Weather info box (toggled with I) and a small wind indicator relative to the nose.
#[allow(deprecated)]
pub fn hud(ctx: &egui::Context, sim: &crate::sim::Sim, world: &crate::world::World) {
    use egui::{Align2, Color32, FontId, Pos2, RichText};
    let id = egui::Id::new("skyward_wx_info");
    if ctx.input(|i| i.key_pressed(egui::Key::I) && !i.modifiers.any()) && !ctx.egui_wants_keyboard_input() {
        ctx.data_mut(|d| {
            let v = d.get_temp_mut_or(id, true);
            *v = !*v;
        });
    }
    let open = ctx.data(|d| d.get_temp::<bool>(id).unwrap_or(true));
    let w = &world.wx;
    let ac = &sim.ac;
    let gs = Vec2::new(ac.vel.x, ac.vel.z);
    let hdg = ac.heading();
    // Wind relative to the nose: screen up = straight ahead.
    let wv = w.wind();
    let rel = {
        let (s, c) = (-hdg).sin_cos();
        // Rotate the world (east, south) vector by -heading so the nose points up.
        egui::Vec2::new(wv.x * c - wv.y * s, wv.x * s + wv.y * c)
    };
    let screen = ctx.content_rect();
    let fg = Color32::from_rgb(225, 230, 235);
    let dim = Color32::from_rgb(150, 160, 170);
    egui::Area::new(egui::Id::new("skyward_wx_area")).anchor(Align2::LEFT_TOP, egui::vec2(12.0, 104.0)).interactable(false).show(ctx, |ui| {
        egui::Frame::new().fill(Color32::from_rgba_unmultiplied(8, 10, 14, 200)).corner_radius(6.0).inner_margin(8.0).show(ui, |ui| {
            ui.set_max_width(if open { 330.0 } else { 120.0 });
            let (rect, _) = ui.allocate_exact_size(egui::vec2(if open { 314.0 } else { 104.0 }, 46.0), egui::Sense::hover());
            let p = ui.painter();
            let c = Pos2::new(rect.min.x + 23.0, rect.center().y);
            p.circle_stroke(c, 20.0, egui::Stroke::new(1.0, dim));
            p.text(Pos2::new(c.x, c.y - 20.0), Align2::CENTER_BOTTOM, "▲", FontId::proportional(8.0), dim);
            if w.wind_kt >= 1.0 {
                wind_arrow(p, c, 14.0, rel, Color32::from_rgb(120, 210, 255));
            }
            let head = rel.y * -1.0; // + = headwind component (air coming toward the nose)
            let cross = rel.x;
            p.text(Pos2::new(c.x + 28.0, rect.min.y + 2.0), Align2::LEFT_TOP, dir_text(w.wind_from, w.wind_kt, w.gust_kt), FontId::monospace(13.0), fg);
            p.text(
                Pos2::new(c.x + 28.0, rect.min.y + 20.0),
                Align2::LEFT_TOP,
                format!("{}{:.0} {}{:.0} kt", if head <= 0.0 { "H" } else { "T" }, (head / KT).abs(), if cross >= 0.0 { "X→" } else { "X←" }, (cross / KT).abs()),
                FontId::monospace(11.0),
                dim,
            );
            if open {
                p.text(
                    Pos2::new(rect.max.x, rect.min.y + 2.0),
                    Align2::RIGHT_TOP,
                    format!("GS {:.0}  TAS {:.0}", gs.length() / KT, ac.tas() / KT),
                    FontId::monospace(12.0),
                    fg,
                );
                let drift = if gs.length() > 5.0 {
                    let trk = gs.x.atan2(-gs.y);
                    let mut d = (trk - hdg).to_degrees();
                    d = (d + 540.0).rem_euclid(360.0) - 180.0;
                    format!("drift {:.0}° {}", d.abs(), if d >= 0.0 { "R" } else { "L" })
                } else {
                    String::new()
                };
                p.text(Pos2::new(rect.max.x, rect.min.y + 20.0), Align2::RIGHT_TOP, drift, FontId::monospace(11.0), dim);
            }
            if !open {
                return;
            }
            let live = if world.live_weather { "LIVE" } else { "fixed" };
            let st = w.metar.as_ref().map(|m| format!("{} ({:.0} km)", m.station, w.station_km)).unwrap_or_else(|| w.source.clone());
            ui.label(RichText::new(format!("WEATHER  {live} · {st} · {}", w.category())).color(Color32::from_rgb(255, 205, 90)).monospace().size(12.0));
            if let Some(m) = &w.metar {
                ui.label(RichText::new(&m.raw).color(dim).monospace().size(10.5));
            }
            let row = |ui: &mut egui::Ui, k: &str, v: String| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(format!("{k:<6}")).color(dim).monospace().size(12.0));
                    ui.label(RichText::new(v).color(fg).monospace().size(12.0));
                });
            };
            row(ui, "Wind", format!("{}   aloft {}", dir_text(w.wind_from, w.wind_kt, w.gust_kt), dir_text(w.aloft_from, w.aloft_kt, 0.0)));
            row(ui, "Vis", format!("{}   cloud {}", vis_text(w.vis_m), w.cloud_text()));
            let wxs: Vec<String> = w.metar.as_ref().map(|m| m.wx.clone()).unwrap_or_default();
            let pw = if !wxs.is_empty() {
                wxs.join(" ")
            } else if w.precip != Precip::None {
                format!("{} ({:.0}%)", w.precip.name(), w.intensity * 100.0)
            } else {
                "—".into()
            };
            row(ui, "Wx", pw);
            row(ui, "Temp", format!("{:.0}°C  dew {:.0}°C   QNH {:.0} hPa", w.temp_c, w.dew_c, w.qnh_hpa));
            let sun = sim.sun_dir().y.asin().to_degrees();
            let time = match &world.real_time {
                Some(rt) => format!("{} live", rt.text),
                None => {
                    let h = sim.hours.rem_euclid(24.0);
                    format!("{:02}:{:02} day {} (set)", h.floor() as u32, ((h.fract() * 60.0) as u32).min(59), sim.day_of_year)
                }
            };
            row(ui, "Time", format!("{time}  sun {sun:+.0}°"));
            row(ui, "Season", season_name(&world.season, sim.day_of_year));
            let status = if world.live_weather { world.live.status.clone() } else { format!("fixed weather: {}", w.source) };
            ui.label(RichText::new(format!("{status}  ·  I: hide")).color(dim).size(10.0));
        });
    });
    let _ = screen;
}

/// Write what weather / time a capture used next to it (`<shot>.weather.txt`).
pub fn save_report(shot: &str, world: &crate::world::World) {
    let w = &world.wx;
    let r = world.render_weather();
    let spec = w.cloud_spec(0.3, &world.season);
    let mut s = String::new();
    use std::fmt::Write;
    let _ = writeln!(s, "source: {}  (live weather {}, live time {})", w.source, world.live_weather, world.live_time);
    let _ = writeln!(s, "status: {}", world.live.status);
    if let Some(m) = &w.metar {
        let _ = writeln!(s, "METAR: {}  (station {:.1} km away)", m.raw, w.station_km);
    }
    if let Some(rt) = &world.real_time {
        let _ = writeln!(s, "local time: {}  (sun clock {:.2} h, day {})", rt.text, rt.sun_hours, rt.sun_doy);
    }
    let _ = writeln!(s, "wind {}  aloft {}  vis {}  cloud {}  {}", dir_text(w.wind_from, w.wind_kt, w.gust_kt), dir_text(w.aloft_from, w.aloft_kt, 0.0), vis_text(w.vis_m), w.cloud_text(), w.category());
    let _ = writeln!(s, "temp {:.1} dew {:.1} QNH {:.1}  precip {} {:.2} showers {} thunder {} fog {} mist {} haze {}", w.temp_c, w.dew_c, w.qnh_hpa, w.precip.name(), w.intensity, w.showers, w.thunder, w.fog, w.mist, w.haze);
    let _ = writeln!(s, "season: {:?}", world.season);
    let _ = writeln!(s, "clouds: {:?}  (sky cover {:.2}, lowest base {:.0} m)", spec, world.clouds.sky_cover(), world.clouds.base());
    let _ = writeln!(s, "render: {r:?}");
    if let Some(o) = &w.om {
        let _ = writeln!(s, "open-meteo: {o:?}");
    }
    let _ = std::fs::write(format!("{shot}.weather.txt"), s);
}
