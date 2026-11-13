//! Airport frequencies from OurAirports' `airport-frequencies.csv` (public
//! domain, <https://ourairports.com/data/>): ATIS, delivery, ground, tower,
//! approach / director / radar, departure. Downloaded once and compacted into
//! `cache/frequencies.tsv.gz`; the browser fetches the CSV (GitHub Pages sends
//! CORS headers). Only VHF COM frequencies (118–137 MHz) are kept: some ATIS
//! are broadcast on VORs, which the COM radios can't receive.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use web_time::Duration;

pub const URL: &str = "https://davidmegginson.github.io/ourairports-data/airport-frequencies.csv";
pub const CREDIT: &str = "Airport frequencies: OurAirports (public domain)";
const CACHE_VERSION: &str = "skyward-freqs-1";
const MAX_AGE: Duration = Duration::from_secs(45 * 24 * 3600);

/// A unit on the frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Fac {
    Atis,
    Delivery,
    Ground,
    Tower,
    Approach,
    Director,
    Radar,
    Departure,
    Control,
}

impl Fac {
    fn parse(t: &str, desc: &str) -> Option<Fac> {
        let t = t.to_ascii_uppercase();
        let d = desc.to_ascii_uppercase();
        Some(match t.as_str() {
            "ATIS" | "D-ATIS" => Fac::Atis,
            "CLD" | "DEL" | "CLNC" | "CLR" | "CLNC DEL" => Fac::Delivery,
            "GND" | "GROUND" | "SMC" => Fac::Ground,
            "TWR" | "TOWER" | "ADC" => Fac::Tower,
            "APP" | "ARR" | "APCH" | "APPROACH" => {
                if d.contains("RADAR") || d.contains("RDR") {
                    Fac::Radar
                } else {
                    Fac::Approach
                }
            }
            "DIR" | "DIRECTOR" => Fac::Director,
            "RDR" | "RADAR" => {
                if d.contains("DIR") {
                    Fac::Director
                } else {
                    Fac::Radar
                }
            }
            "DEP" | "DEPARTURE" => Fac::Departure,
            "CNTR" | "ACC" | "CTR" | "CONTROL" => Fac::Control,
            _ => return None,
        })
    }
    fn code(self) -> &'static str {
        match self {
            Fac::Atis => "ATIS",
            Fac::Delivery => "DEL",
            Fac::Ground => "GND",
            Fac::Tower => "TWR",
            Fac::Approach => "APP",
            Fac::Director => "DIR",
            Fac::Radar => "RDR",
            Fac::Departure => "DEP",
            Fac::Control => "CTR",
        }
    }
    /// The spoken / shown unit word ("tower").
    pub fn word(self) -> &'static str {
        match self {
            Fac::Atis => "information",
            Fac::Delivery => "delivery",
            Fac::Ground => "ground",
            Fac::Tower => "tower",
            Fac::Approach => "approach",
            Fac::Director => "director",
            Fac::Radar => "radar",
            Fac::Departure => "departure",
            Fac::Control => "control",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Fac::Atis => "ATIS",
            Fac::Delivery => "Delivery",
            Fac::Ground => "Ground",
            Fac::Tower => "Tower",
            Fac::Approach => "Approach",
            Fac::Director => "Director",
            Fac::Radar => "Radar",
            Fac::Departure => "Departure",
            Fac::Control => "Control",
        }
    }
    /// Approach-type units (radar vectoring).
    pub fn is_radar(self) -> bool {
        matches!(self, Fac::Approach | Fac::Director | Fac::Radar | Fac::Departure | Fac::Control)
    }
    /// Service range (NM) for reception.
    pub fn range_nm(self) -> f32 {
        match self {
            Fac::Ground | Fac::Delivery => 15.0,
            Fac::Tower => 30.0,
            Fac::Atis => 60.0,
            _ => 80.0,
        }
    }
}

/// One frequency of an airport.
#[derive(Clone, Debug, PartialEq)]
pub struct Freq {
    pub airport: String,
    pub fac: Fac,
    pub khz: u32,
    /// The description as published ("Heathrow Director", "THAMES RDR").
    pub desc: String,
}

pub fn com_band(khz: u32) -> bool {
    (118_000..=136_990).contains(&khz)
}

/// Parse OurAirports' airport-frequencies.csv.
pub fn parse_csv(text: &str) -> Vec<Freq> {
    let mut rows = crate::airports::csv_rows(text);
    let head = rows.next().unwrap_or_default();
    let col = |n: &str| head.iter().position(|h| h == n);
    let c = ["airport_ident", "type", "description", "frequency_mhz"].map(col);
    let get = |r: &Vec<String>, i: usize| c[i].and_then(|k| r.get(k)).map(|s| s.trim().to_string()).unwrap_or_default();
    let mut out = Vec::new();
    for r in rows {
        let desc = get(&r, 2);
        let Some(fac) = Fac::parse(&get(&r, 1), &desc) else { continue };
        let Ok(mhz) = get(&r, 3).parse::<f64>() else { continue };
        let khz = (mhz * 1000.0).round() as u32;
        if !com_band(khz) {
            continue;
        }
        let airport = get(&r, 0);
        if airport.is_empty() {
            continue;
        }
        out.push(Freq { airport, fac, khz, desc });
    }
    out
}

fn clean(s: &str) -> String {
    s.replace(['\t', '\n', '\r'], " ")
}

pub fn encode(list: &[Freq]) -> String {
    let mut s = String::with_capacity(list.len() * 32);
    s.push_str(CACHE_VERSION);
    s.push('\n');
    for f in list {
        s.push_str(&format!("{}\t{}\t{}\t{}\n", clean(&f.airport), f.fac.code(), f.khz, clean(&f.desc)));
    }
    s
}

pub fn decode(text: &str) -> Option<Vec<Freq>> {
    let mut lines = text.lines();
    if lines.next()? != CACHE_VERSION {
        return None;
    }
    let mut out = Vec::new();
    for l in lines {
        let f: Vec<&str> = l.split('\t').collect();
        if f.len() < 4 {
            continue;
        }
        let fac = match f[1] {
            "ATIS" => Fac::Atis,
            "DEL" => Fac::Delivery,
            "GND" => Fac::Ground,
            "TWR" => Fac::Tower,
            "APP" => Fac::Approach,
            "DIR" => Fac::Director,
            "RDR" => Fac::Radar,
            "DEP" => Fac::Departure,
            "CTR" => Fac::Control,
            _ => continue,
        };
        out.push(Freq { airport: f[0].into(), fac, khz: f[2].parse().ok()?, desc: f[3].into() });
    }
    Some(out)
}

/// Frequencies by airport ident.
pub struct FreqDb {
    by_airport: HashMap<String, Vec<Freq>>,
}

impl FreqDb {
    pub fn new(list: Vec<Freq>) -> FreqDb {
        let mut by_airport: HashMap<String, Vec<Freq>> = HashMap::new();
        for f in list {
            let v = by_airport.entry(f.airport.to_ascii_uppercase()).or_default();
            if !v.iter().any(|g| g.khz == f.khz && g.fac == f.fac) {
                v.push(f);
            }
        }
        FreqDb { by_airport }
    }
    pub fn at(&self, airport: &str) -> &[Freq] {
        self.by_airport.get(&airport.to_ascii_uppercase()).map(|v| &v[..]).unwrap_or(&[])
    }
    pub fn len(&self) -> usize {
        self.by_airport.values().map(|v| v.len()).sum()
    }
}

// ---------------------------------------------------------------- loading

enum Load {
    Idle,
    Loading,
    Ready(Arc<FreqDb>),
    Failed(String),
}

static STATE: Mutex<Load> = Mutex::new(Load::Idle);

/// Install a database directly (tests, offline renders).
pub fn install(db: FreqDb) {
    *STATE.lock().unwrap_or_else(|e| e.into_inner()) = Load::Ready(Arc::new(db));
}

/// The frequency database once loaded (starts loading on first call). Unit
/// tests never download.
pub fn db() -> Option<Arc<FreqDb>> {
    let mut st = STATE.lock().unwrap_or_else(|e| e.into_inner());
    match &*st {
        Load::Ready(d) => Some(d.clone()),
        Load::Loading | Load::Failed(_) => None,
        Load::Idle => {
            if cfg!(test) {
                return None;
            }
            *st = Load::Loading;
            drop(st);
            start();
            None
        }
    }
}

pub fn status() -> String {
    match &*STATE.lock().unwrap_or_else(|e| e.into_inner()) {
        Load::Idle => "not loaded".into(),
        Load::Loading => "loading".into(),
        Load::Ready(d) => format!("{} airport frequencies (OurAirports)", d.len()),
        Load::Failed(e) => format!("unavailable ({e})"),
    }
}

fn finish(r: Result<Vec<Freq>, String>) {
    let mut st = STATE.lock().unwrap_or_else(|e| e.into_inner());
    match r {
        Ok(list) => *st = Load::Ready(Arc::new(FreqDb::new(list))),
        Err(e) => {
            eprintln!("frequencies: {e}");
            if !matches!(*st, Load::Ready(_)) {
                *st = Load::Failed(e);
            }
        }
    }
}

/// Block until loaded (headless runs). Native only.
#[cfg(not(target_arch = "wasm32"))]
pub fn wait(timeout: Duration) -> Option<Arc<FreqDb>> {
    let t = web_time::Instant::now();
    loop {
        if let Some(d) = db() {
            return Some(d);
        }
        if matches!(*STATE.lock().unwrap_or_else(|e| e.into_inner()), Load::Failed(_)) || t.elapsed() > timeout {
            return None;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn cache_path() -> std::path::PathBuf {
    std::path::PathBuf::from("cache").join("frequencies.tsv.gz")
}

#[cfg(not(target_arch = "wasm32"))]
fn read_cache() -> Option<(Vec<Freq>, bool)> {
    let path = cache_path();
    let raw = std::fs::read(&path).ok()?;
    let mut text = String::new();
    flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).ok()?;
    let stale = std::fs::metadata(&path).ok().and_then(|m| m.modified().ok()).and_then(|t| t.elapsed().ok()).is_some_and(|age| age > MAX_AGE);
    Some((decode(&text)?, stale))
}

pub fn compact(list: &[Freq]) -> Result<Vec<u8>, String> {
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    gz.write_all(encode(list).as_bytes()).map_err(|e| e.to_string())?;
    gz.finish().map_err(|e| e.to_string())
}

#[cfg(not(target_arch = "wasm32"))]
fn download() -> Result<Vec<Freq>, String> {
    use crate::airports::Fetcher;
    let raw = crate::airports::HttpFetcher.get(URL)?;
    let list = parse_csv(&String::from_utf8_lossy(&raw));
    if list.len() < 1000 {
        return Err(format!("frequency list too short ({})", list.len()));
    }
    let bytes = compact(&list)?;
    let _ = std::fs::create_dir_all("cache");
    let tmp = cache_path().with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, cache_path());
    }
    eprintln!("frequencies: downloaded OurAirports ({:.1} MB csv) → {} COM frequencies, {:.2} MB cache", raw.len() as f64 / 1e6, list.len(), bytes.len() as f64 / 1e6);
    Ok(list)
}

#[cfg(not(target_arch = "wasm32"))]
fn start() {
    let _ = std::thread::Builder::new().name("frequencies".into()).spawn(|| match read_cache() {
        Some((list, stale)) => {
            finish(Ok(list));
            if stale {
                if let Ok(l) = download() {
                    finish(Ok(l));
                }
            }
        }
        None => finish(download()),
    });
}

#[cfg(target_arch = "wasm32")]
fn start() {
    crate::platform::web::spawn_local(async move {
        use crate::platform::web;
        let t = Duration::from_secs(90);
        let compact = web::load_manifest().await.is_ok() && web::has_extra("data/frequencies.tsv.gz");
        if compact {
            if let Ok(raw) = web::get("data/frequencies.tsv.gz", t).await {
                let mut text = String::new();
                if flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).is_ok() {
                    if let Some(list) = decode(&text) {
                        finish(Ok(list));
                        return;
                    }
                }
            }
        }
        let r = web::get(URL, t).await.map(|raw| parse_csv(&String::from_utf8_lossy(&raw)));
        finish(r);
    });
}
