//! Navaid database.
//!
//! * **VOR, VOR-DME, VORTAC, TACAN, DME, NDB, NDB-DME** from OurAirports'
//!   `navaids.csv` (public domain, <https://ourairports.com/data/>):
//!   frequencies, positions, elevations, DME antennas, power class. Downloaded
//!   once and compacted into `cache/navaids.tsv.gz` (≈ 0.3 MB); the browser
//!   build fetches the CSV (GitHub Pages sends CORS headers) or a compact copy
//!   shipped next to the page (`data/navaids.tsv.gz`).
//! * **ILS** (localizer + glideslope + DME): no licence-clean worldwide ILS
//!   dataset exists, so every instrument runway (paved, lit, ≥ 1200 m,
//!   surveyed ends, at an airport / major airport) gets a **synthetic** ILS
//!   derived from its OurAirports geometry — localizer 300 m beyond the stop
//!   end on the runway course, glideslope abeam the touchdown zone for a
//!   50 ft threshold crossing height, DME at the glideslope, a frequency from
//!   the 40 ILS channels picked by hashing the airport / runway, ident
//!   "I" + airport + runway. A short table of real facts (frequency, ident,
//!   glidepath angle) from public charts overrides the synthetic values for
//!   a few London airports. Synthetic records are flagged (`synthetic`).

use super::geo::{self, LL};
use crate::airports::{self, Airport, Rwy};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use web_time::Duration;

pub const NAVAIDS_URL: &str = "https://davidmegginson.github.io/ourairports-data/navaids.csv";
/// Shown in the credits.
pub const CREDIT: &str = "Navaids: OurAirports (public domain); ILS: synthetic from runway data";
const CACHE_VERSION: &str = "skyward-navaids-1";
const MAX_AGE: Duration = Duration::from_secs(45 * 24 * 3600);
const FT: f32 = 0.3048;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    Vor,
    VorDme,
    Vortac,
    Tacan,
    Dme,
    Ndb,
    NdbDme,
    /// Localizer + glideslope.
    Ils,
    /// Localizer only (no usable glideslope).
    Loc,
}

impl Kind {
    fn parse(s: &str) -> Option<Kind> {
        Some(match s {
            "VOR" => Kind::Vor,
            "VOR-DME" => Kind::VorDme,
            "VORTAC" => Kind::Vortac,
            "TACAN" => Kind::Tacan,
            "DME" => Kind::Dme,
            "NDB" => Kind::Ndb,
            "NDB-DME" => Kind::NdbDme,
            _ => return None,
        })
    }
    fn code(self) -> &'static str {
        match self {
            Kind::Vor => "V",
            Kind::VorDme => "VD",
            Kind::Vortac => "VT",
            Kind::Tacan => "T",
            Kind::Dme => "D",
            Kind::Ndb => "N",
            Kind::NdbDme => "ND",
            Kind::Ils => "I",
            Kind::Loc => "L",
        }
    }
    fn from_code(s: &str) -> Option<Kind> {
        Some(match s {
            "V" => Kind::Vor,
            "VD" => Kind::VorDme,
            "VT" => Kind::Vortac,
            "T" => Kind::Tacan,
            "D" => Kind::Dme,
            "N" => Kind::Ndb,
            "ND" => Kind::NdbDme,
            "I" => Kind::Ils,
            "L" => Kind::Loc,
            _ => return None,
        })
    }
    pub fn label(self) -> &'static str {
        match self {
            Kind::Vor => "VOR",
            Kind::VorDme => "VOR/DME",
            Kind::Vortac => "VORTAC",
            Kind::Tacan => "TACAN",
            Kind::Dme => "DME",
            Kind::Ndb => "NDB",
            Kind::NdbDme => "NDB/DME",
            Kind::Ils => "ILS",
            Kind::Loc => "LOC",
        }
    }
    /// Received on a NAV (VHF) receiver.
    pub fn vhf(self) -> bool {
        matches!(self, Kind::Vor | Kind::VorDme | Kind::Vortac | Kind::Ils | Kind::Loc | Kind::Dme | Kind::Tacan)
    }
    /// Gives a bearing on a NAV receiver (VOR or localizer).
    pub fn bearing(self) -> bool {
        matches!(self, Kind::Vor | Kind::VorDme | Kind::Vortac | Kind::Ils | Kind::Loc)
    }
    pub fn is_vor(self) -> bool {
        matches!(self, Kind::Vor | Kind::VorDme | Kind::Vortac)
    }
    pub fn is_loc(self) -> bool {
        matches!(self, Kind::Ils | Kind::Loc)
    }
    pub fn is_ndb(self) -> bool {
        matches!(self, Kind::Ndb | Kind::NdbDme)
    }
    pub fn has_dme(self) -> bool {
        matches!(self, Kind::VorDme | Kind::Vortac | Kind::Tacan | Kind::Dme | Kind::NdbDme | Kind::Ils | Kind::Loc)
    }
}

/// Localizer / glideslope definition of an ILS.
#[derive(Clone, Debug, PartialEq)]
pub struct Ils {
    pub airport: String,
    /// Runway end ident ("27R").
    pub runway: String,
    /// Landing threshold (displaced) and its elevation (m, the airport's).
    pub thr: LL,
    pub thr_elev: f32,
    /// Front course (true, radians): the landing direction.
    pub course: f32,
    /// Localizer antenna (beyond the stop end).
    pub loc: LL,
    /// Glideslope antenna site (None = localizer only).
    pub gs: Option<LL>,
    /// Glidepath angle (radians).
    pub gs_angle: f32,
    /// Half course width (radians): full-scale deflection.
    pub half_width: f32,
    /// Runway length beyond the threshold (m).
    pub length: f32,
    /// Derived from runway data (not a published facility).
    pub synthetic: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Navaid {
    pub kind: Kind,
    pub ident: String,
    pub name: String,
    /// kHz: VHF navaids e.g. 115100 (= 115.10 MHz), NDBs e.g. 373.
    pub freq_khz: u32,
    pub lat: f64,
    pub lon: f64,
    pub elev_m: f32,
    /// DME antenna if it is not co-located.
    pub dme: Option<LL>,
    /// Designed service range (NM).
    pub range_nm: f32,
    pub airport: String,
    pub ils: Option<Ils>,
}

impl Navaid {
    pub fn ll(&self) -> LL {
        (self.lat, self.lon)
    }
    /// "115.10" / "373" / "110.30".
    pub fn freq_label(&self) -> String {
        freq_label(self.freq_khz, self.kind.is_ndb())
    }
    pub fn dme_ll(&self) -> LL {
        self.dme.unwrap_or((self.lat, self.lon))
    }
}

pub fn freq_label(khz: u32, ndb: bool) -> String {
    if ndb { format!("{khz}") } else { format!("{}.{:02}", khz / 1000, (khz % 1000) / 10) }
}

/// Is `khz` an ILS channel (108.10–111.95, odd tenths)?
pub fn is_ils_freq(khz: u32) -> bool {
    (108_000..112_000).contains(&khz) && (khz / 100) % 2 == 1
}

/// The 40 ILS localizer channels.
pub fn ils_channels() -> Vec<u32> {
    let mut v = Vec::with_capacity(40);
    for mhz in 108..112u32 {
        for t in [1u32, 3, 5, 7, 9] {
            v.push(mhz * 1000 + t * 100);
            v.push(mhz * 1000 + t * 100 + 50);
        }
    }
    v
}

fn range_for(kind: Kind, power: &str, usage: &str) -> f32 {
    let p = power.to_ascii_uppercase();
    match kind {
        Kind::Ndb | Kind::NdbDme => match p.as_str() {
            "HIGH" => 75.0,
            "LOW" => 25.0,
            _ => 50.0,
        },
        _ => {
            if usage == "TERMINAL" || p == "LOW" {
                25.0
            } else if p == "HIGH" || usage == "HI" || usage == "BOTH" {
                130.0
            } else {
                60.0
            }
        }
    }
}

// ---------------------------------------------------------------- CSV / cache

/// Parse OurAirports' navaids.csv.
pub fn parse_csv(text: &str) -> Vec<Navaid> {
    let mut rows = airports::csv_rows(text);
    let head = rows.next().unwrap_or_default();
    let col = |n: &str| head.iter().position(|h| h == n);
    let c = [
        "ident",
        "name",
        "type",
        "frequency_khz",
        "latitude_deg",
        "longitude_deg",
        "elevation_ft",
        "dme_latitude_deg",
        "dme_longitude_deg",
        "usageType",
        "power",
        "associated_airport",
    ]
    .map(col);
    let get = |r: &Vec<String>, i: usize| c[i].and_then(|k| r.get(k)).map(|s| s.trim().to_string()).unwrap_or_default();
    let mut out = Vec::new();
    for r in rows {
        let Some(kind) = Kind::parse(&get(&r, 2)) else { continue };
        let (Ok(lat), Ok(lon)) = (get(&r, 4).parse::<f64>(), get(&r, 5).parse::<f64>()) else { continue };
        let Ok(freq) = get(&r, 3).parse::<f64>() else { continue };
        let ident = get(&r, 0);
        if ident.is_empty() || freq <= 0.0 {
            continue;
        }
        let dme = match (get(&r, 7).parse::<f64>(), get(&r, 8).parse::<f64>()) {
            (Ok(a), Ok(b)) if (a - lat).abs() + (b - lon).abs() > 1e-4 => Some((a, b)),
            _ => None,
        };
        out.push(Navaid {
            kind,
            ident,
            name: get(&r, 1),
            freq_khz: freq.round() as u32,
            lat,
            lon,
            elev_m: get(&r, 6).parse::<f32>().unwrap_or(0.0) * FT,
            dme,
            range_nm: range_for(kind, &get(&r, 10), &get(&r, 9)),
            airport: get(&r, 11),
            ils: None,
        });
    }
    out
}

fn clean(s: &str) -> String {
    s.replace(['\t', '\n', '\r'], " ")
}

pub fn encode(list: &[Navaid]) -> String {
    let mut s = String::with_capacity(list.len() * 70);
    s.push_str(CACHE_VERSION);
    s.push('\n');
    for n in list.iter().filter(|n| n.ils.is_none()) {
        let (dl, dn) = n.dme.map(|d| (format!("{:.5}", d.0), format!("{:.5}", d.1))).unwrap_or_default();
        s.push_str(&format!(
            "{}\t{}\t{}\t{}\t{:.5}\t{:.5}\t{:.0}\t{}\t{}\t{:.0}\t{}\n",
            n.kind.code(),
            clean(&n.ident),
            clean(&n.name),
            n.freq_khz,
            n.lat,
            n.lon,
            n.elev_m,
            dl,
            dn,
            n.range_nm,
            clean(&n.airport)
        ));
    }
    s
}

pub fn decode(text: &str) -> Option<Vec<Navaid>> {
    let mut lines = text.lines();
    if lines.next()? != CACHE_VERSION {
        return None;
    }
    let mut out = Vec::new();
    for l in lines {
        let f: Vec<&str> = l.split('\t').collect();
        if f.len() < 11 {
            continue;
        }
        let Some(kind) = Kind::from_code(f[0]) else { continue };
        let dme = match (f[7].parse::<f64>(), f[8].parse::<f64>()) {
            (Ok(a), Ok(b)) => Some((a, b)),
            _ => None,
        };
        out.push(Navaid {
            kind,
            ident: f[1].into(),
            name: f[2].into(),
            freq_khz: f[3].parse().ok()?,
            lat: f[4].parse().ok()?,
            lon: f[5].parse().ok()?,
            elev_m: f[6].parse().unwrap_or(0.0),
            dme,
            range_nm: f[9].parse().unwrap_or(40.0),
            airport: f[10].into(),
            ils: None,
        });
    }
    Some(out)
}

// ---------------------------------------------------------------- database

pub struct NavDb {
    pub navaids: Vec<Navaid>,
    grid: HashMap<(i32, i32), Vec<u32>>,
}

fn cell(lat: f64, lon: f64) -> (i32, i32) {
    (lat.floor() as i32, lon.floor() as i32)
}

impl NavDb {
    pub fn new(navaids: Vec<Navaid>) -> NavDb {
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, n) in navaids.iter().enumerate() {
            grid.entry(cell(n.lat, n.lon)).or_default().push(i as u32);
        }
        NavDb { navaids, grid }
    }

    /// Navaids within `km` of a point (unsorted): (index, distance km).
    pub fn near(&self, ll: LL, km: f64) -> Vec<(usize, f64)> {
        let dlat = (km / 111.0).ceil() as i32 + 1;
        let dlon = (km / (111.0 * ll.0.to_radians().cos().max(0.05))).ceil().min(180.0) as i32 + 1;
        let (cy, cx) = cell(ll.0, ll.1);
        let mut out = Vec::new();
        for y in cy - dlat..=cy + dlat {
            for x in cx - dlon..=cx + dlon {
                let xw = (x + 180).rem_euclid(360) - 180;
                if let Some(v) = self.grid.get(&(y, xw)) {
                    for &i in v {
                        let n = &self.navaids[i as usize];
                        let d = geo::gc_dist(ll, n.ll()) / 1000.0;
                        if d <= km {
                            out.push((i as usize, d));
                        }
                    }
                }
            }
        }
        out
    }

    /// The navaid called `ident` nearest to `ll` (within 1500 km).
    pub fn find(&self, ident: &str, ll: LL) -> Option<&Navaid> {
        let id = ident.trim().to_ascii_uppercase();
        self.near(ll, 1500.0).into_iter().filter(|(i, _)| self.navaids[*i].ident == id).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(i, _)| &self.navaids[i])
    }
}

// ---------------------------------------------------------------- ILS

/// Published ILS facts from public charts: (airport, runway end, frequency kHz,
/// ident, glidepath °). Everything else is synthetic.
const PUBLISHED: &[(&str, &str, u32, &str, f32)] = &[
    ("EGLL", "27R", 110_300, "IRR", 3.0),
    ("EGLL", "09L", 110_300, "IAA", 3.0),
    ("EGLL", "27L", 109_500, "ILL", 3.0),
    ("EGLL", "09R", 109_500, "IBB", 3.0),
    ("EGKK", "26L", 110_900, "IGG", 3.0),
    ("EGKK", "08R", 110_900, "IWW", 3.0),
    ("EGLC", "27", 111_150, "LST", 5.5),
    ("EGLC", "09", 111_150, "LSR", 5.5),
    ("EGSS", "22", 110_500, "ISX", 3.0),
    ("EGSS", "04", 110_500, "ISED", 3.0),
];

fn hash(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3))
}

/// Does this runway get an instrument landing system?
pub fn instrument_runway(ap: &Airport, rw: &Rwy) -> bool {
    matches!(ap.kind, airports::Kind::Large | airports::Kind::Medium)
        && rw.surface.paved()
        && rw.lighted
        && rw.length_m >= 1200.0
        && rw.surveyed()
        && rw.ends.iter().all(|e| !e.ident.starts_with('H'))
}

/// Synthetic ident: "I" + the airport's last two letters + the runway's side
/// letter or last digit ("EGLL" "27R" → "ILLR").
fn synthetic_ident(ap: &str, rwy: &str) -> String {
    let a: String = ap.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
    let tail: String = a.chars().skip(a.chars().count().saturating_sub(2)).collect();
    let suffix = rwy.chars().rev().find(|c| c.is_ascii_alphabetic()).or_else(|| rwy.chars().rev().find(|c| c.is_ascii_digit())).unwrap_or('X');
    format!("I{}{}", tail, suffix).to_ascii_uppercase()
}

/// The ILS of runway end `end` (0 = low end) of `rw` at `ap`, if it has one.
pub fn ils_for(ap: &Airport, rw: &Rwy, end: usize) -> Option<Navaid> {
    if !instrument_runway(ap, rw) {
        return None;
    }
    let (a, b) = (&rw.ends[end], &rw.ends[1 - end]);
    let pa = (a.lat?, a.lon?);
    let pb = (b.lat?, b.lon?);
    let course = geo::gc_bearing(pa, pb);
    let len = geo::gc_dist(pa, pb) as f32;
    if len < 600.0 {
        return None;
    }
    let thr = geo::gc_dest(pa, course, a.displaced_m as f64);
    let length = len - a.displaced_m;
    let code = ap.code().to_string();
    let published = PUBLISHED.iter().find(|p| p.0 == code && p.1 == a.ident);
    let (freq, ident, gs_deg, synthetic) = match published {
        Some(p) => (p.2, p.3.to_string(), p.4, false),
        None => {
            // Both ends of a runway share the channel (only one radiates at a time).
            let pair = if a.ident < b.ident { format!("{}{}", a.ident, b.ident) } else { format!("{}{}", b.ident, a.ident) };
            let ch = ils_channels();
            let f = ch[(hash(&format!("{code}/{pair}")) % ch.len() as u64) as usize];
            (f, synthetic_ident(&code, &a.ident), 3.0, true)
        }
    };
    let gs_angle = gs_deg.to_radians();
    // Localizer 300 m beyond the stop end; glideslope abeam the touchdown
    // zone for a 50 ft threshold crossing height, 120 m to the side.
    let loc = geo::gc_dest(pb, course, 300.0);
    let gs_along = 50.0 * FT / gs_angle.tan();
    let gs_c = geo::gc_dest(thr, course, gs_along as f64);
    let gs = geo::gc_dest(gs_c, geo::wrap_2pi(course - std::f32::consts::FRAC_PI_2), 120.0);
    // Course width: ±105 m (350 ft) at the threshold, 3°..6° wide.
    let d_thr = len - a.displaced_m + 300.0;
    let half_width = (105.0 / d_thr).atan().clamp(1.5f32.to_radians(), 3.0f32.to_radians());
    let elev = ap.elev_ft * FT;
    Some(Navaid {
        kind: Kind::Ils,
        ident,
        name: format!("{} RWY {}", ap.name, a.ident),
        freq_khz: freq,
        lat: loc.0,
        lon: loc.1,
        elev_m: elev,
        dme: Some(gs),
        range_nm: 18.0,
        airport: ap.ident.clone(),
        ils: Some(Ils { airport: ap.ident.clone(), runway: a.ident.clone(), thr, thr_elev: elev, course, loc, gs: Some(gs), gs_angle, half_width, length, synthetic }),
    })
}

/// Every ILS at the airports within `km` of `ll`.
pub fn ils_near(adb: &airports::Db, ll: LL, km: f64) -> Vec<Navaid> {
    let mut out = Vec::new();
    for (i, _) in adb.near(ll.0, ll.1, km) {
        out.extend(ils_at(&adb.airports[i]));
    }
    out
}

/// Every ILS of one airport.
pub fn ils_at(ap: &Airport) -> Vec<Navaid> {
    let mut out = Vec::new();
    for rw in &ap.runways {
        for e in 0..2 {
            out.extend(ils_for(ap, rw, e));
        }
    }
    out
}

// ---------------------------------------------------------------- loading

enum Load {
    Idle,
    Loading,
    Ready(Arc<NavDb>),
    Failed(String),
}

static STATE: Mutex<Load> = Mutex::new(Load::Idle);

/// The navaid database once loaded (starts loading on first call). Unit
/// tests never download: they build a [`NavDb`] themselves.
pub fn db() -> Option<Arc<NavDb>> {
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

/// Status line for the credits / diagnostics.
pub fn status() -> String {
    match &*STATE.lock().unwrap_or_else(|e| e.into_inner()) {
        Load::Idle => "not loaded".into(),
        Load::Loading => "loading".into(),
        Load::Ready(d) => format!("{} navaids (OurAirports) · ILS from runway data", d.navaids.len()),
        Load::Failed(e) => format!("unavailable ({e})"),
    }
}

fn finish(r: Result<Vec<Navaid>, String>) {
    let mut st = STATE.lock().unwrap_or_else(|e| e.into_inner());
    match r {
        Ok(list) => *st = Load::Ready(Arc::new(NavDb::new(list))),
        Err(e) => {
            eprintln!("navaids: {e}");
            if !matches!(*st, Load::Ready(_)) {
                *st = Load::Failed(e);
            }
        }
    }
}

/// Block until loaded (headless runs). Native only.
#[cfg(not(target_arch = "wasm32"))]
pub fn wait(timeout: Duration) -> Option<Arc<NavDb>> {
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
    std::path::PathBuf::from("cache").join("navaids.tsv.gz")
}

#[cfg(not(target_arch = "wasm32"))]
fn read_cache() -> Option<(Vec<Navaid>, bool)> {
    let path = cache_path();
    let raw = std::fs::read(&path).ok()?;
    let mut text = String::new();
    flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).ok()?;
    let stale = std::fs::metadata(&path).ok().and_then(|m| m.modified().ok()).and_then(|t| t.elapsed().ok()).is_some_and(|age| age > MAX_AGE);
    Some((decode(&text)?, stale))
}

fn compact(list: &[Navaid]) -> Result<Vec<u8>, String> {
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    gz.write_all(encode(list).as_bytes()).map_err(|e| e.to_string())?;
    gz.finish().map_err(|e| e.to_string())
}

#[cfg(not(target_arch = "wasm32"))]
fn download() -> Result<Vec<Navaid>, String> {
    use airports::Fetcher;
    let raw = airports::HttpFetcher.get(NAVAIDS_URL)?;
    let list = parse_csv(&String::from_utf8_lossy(&raw));
    if list.len() < 1000 {
        return Err(format!("navaid list too short ({})", list.len()));
    }
    let bytes = compact(&list)?;
    let _ = std::fs::create_dir_all("cache");
    let tmp = cache_path().with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, cache_path());
    }
    eprintln!("navaids: downloaded OurAirports ({:.1} MB csv) → {} navaids, {:.2} MB cache", raw.len() as f64 / 1e6, list.len(), bytes.len() as f64 / 1e6);
    Ok(list)
}

#[cfg(not(target_arch = "wasm32"))]
fn start() {
    let _ = std::thread::Builder::new().name("navaids".into()).spawn(|| match read_cache() {
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
        let compact = web::load_manifest().await.is_ok() && web::has_extra("data/navaids.tsv.gz");
        if compact {
            if let Ok(raw) = web::get("data/navaids.tsv.gz", t).await {
                let mut text = String::new();
                if flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).is_ok() {
                    if let Some(list) = decode(&text) {
                        finish(Ok(list));
                        return;
                    }
                }
            }
        }
        let r = web::get(NAVAIDS_URL, t).await.map(|raw| parse_csv(&String::from_utf8_lossy(&raw)));
        finish(r);
    });
}

/// Compact copy for the web build (`scripts/web-build.sh` may ship it).
#[allow(dead_code)]
pub fn compact_bytes(list: &[Navaid]) -> Result<Vec<u8>, String> {
    compact(list)
}
