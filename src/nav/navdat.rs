//! Real ILS from FlightGear's `nav.dat` (X-Plane 810 / 1100 format, Robin
//! A. Peel's data, GPL-2.0-or-later — the header of the file says so).
//!
//! * [`parse`] reads every row the sim cares about: NDB (2), VOR (3),
//!   localizers with (4) and without (5) a glideslope, glideslopes (6),
//!   outer / middle / inner markers (7–9) and DMEs (12 paired, 13 alone).
//! * [`ils_records`] joins the localizer, glideslope, DME and marker rows of
//!   each approach into one [`RealIls`]; [`encode`] / [`decode`] keep those
//!   in a compact list (`cache/ils.tsv.gz` natively, `data/ils.tsv.gz` next
//!   to the browser page — GitLab sends no CORS headers, so the page never
//!   reads nav.dat itself; `scripts/web-data.mjs` builds the list in CI).
//! * [`ils_db`] loads that list in the background. [`super::db::ils_at`]
//!   prefers it over the synthetic ILS, which stays the fallback for
//!   runway ends with no entry.
//!
//! VOR / NDB / DME rows parse too ([`navaids`]), for cross-checking
//! OurAirports' navaids; the radios still use OurAirports (more current: the
//! fgdata copy is AIRAC 2013.10).

use super::db::{Kind, Navaid};
use super::geo::LL;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};

/// fgdata's nav.dat, pinned to the commit that last changed it (2014-03-27,
/// AIRAC 2013.10, "Calibrate the GS antenna height to match with FG
/// scenery"): the list is the same on every machine.
pub const NAVDAT_URL: &str = "https://gitlab.com/flightgear/fgdata/-/raw/d0db413dc06ac7d32522ecccfeab58bf5201a335/Navaids/nav.dat.gz";
/// Shown in the credits.
pub const CREDIT: &str = "ILS: FlightGear nav.dat (Robin A. Peel, GPL-2.0-or-later)";
const CACHE_VERSION: &str = "skyward-ils-1";
const FT: f32 = 0.3048;

/// One nav.dat row (fields as in the file; `freq` in its own unit).
#[derive(Clone, Debug, PartialEq)]
pub struct Rec {
    pub code: u8,
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    /// 10 kHz units for VHF rows (11030 = 110.30 MHz), kHz for NDBs, 0 for markers.
    pub freq: u32,
    pub range_nm: f32,
    /// Row-specific: VOR slaved variation, LOC / marker true course, GS
    /// angle × 100 000 + course, DME bias (NM).
    pub extra: f64,
    pub ident: String,
    /// ICAO airport (localizer / glideslope / marker / ILS DME rows).
    pub airport: String,
    pub runway: String,
    pub name: String,
}

impl Rec {
    pub fn freq_khz(&self) -> u32 {
        match self.code {
            2 => self.freq,
            7..=9 => 0,
            _ => self.freq * 10,
        }
    }
    /// Glideslope rows: (angle °, course ° true).
    pub fn gs_angle_course(&self) -> (f32, f32) {
        let a = (self.extra / 1000.0).floor();
        let deg = (a / 100.0) as f32;
        (if deg > 0.5 { deg } else { 3.0 }, (self.extra - a * 1000.0) as f32)
    }
}

/// Parse nav.dat text (810 or 1100 / 1200 layout, from the version line).
/// Rows of other kinds and malformed rows are skipped.
pub fn parse(text: &str) -> Vec<Rec> {
    let mut version = 810;
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let t: Vec<&str> = line.split_whitespace().collect();
        if n < 4 && t.get(1) == Some(&"Version") {
            version = t[0].parse().unwrap_or(810);
            continue;
        }
        if let Some(r) = parse_row(&t, version >= 1100) {
            out.push(r);
        }
    }
    out
}

fn parse_row(t: &[&str], xp11: bool) -> Option<Rec> {
    let code: u8 = t.first()?.parse().ok()?;
    if !matches!(code, 2..=9 | 12 | 13) || t.len() < 9 {
        return None;
    }
    let num = |i: usize| t.get(i).and_then(|s| s.parse::<f64>().ok());
    let (lat, lon) = (num(1)?, num(2)?);
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    let ident = t[7].to_string();
    let rest = |i: usize| t.get(i..).map(|s| s.join(" ")).unwrap_or_default();
    // (airport, runway, name)
    let (airport, runway, name) = match (code, xp11) {
        (2 | 3, false) => (String::new(), String::new(), rest(8)),
        // 1100: ident, airport / "ENRT", ICAO region, name.
        (2 | 3, true) => (String::new(), String::new(), rest(10)),
        (4..=9, false) => (t[8].to_string(), t.get(9)?.to_string(), rest(10)),
        (4..=9, true) => (t[8].to_string(), t.get(10)?.to_string(), rest(11)),
        // 810 ILS DMEs: ident airport runway "DME-ILS"; others: ident name.
        (_, false) if t.len() >= 11 && t.last() == Some(&"DME-ILS") => (t[8].to_string(), t[9].to_string(), rest(10)),
        (_, false) => (String::new(), String::new(), rest(8)),
        (_, true) => (if t[8] == "ENRT" { String::new() } else { t[8].to_string() }, String::new(), rest(10)),
    };
    Some(Rec { code, lat, lon, elev_ft: num(3)? as f32, freq: num(4)?.max(0.0) as u32, range_nm: num(5)? as f32, extra: num(6)?, ident, airport, runway, name })
}

/// nav.dat bytes, gzipped or not.
pub fn text_of(raw: &[u8]) -> String {
    if raw.starts_with(&[0x1f, 0x8b]) {
        let mut s = String::new();
        let mut buf = Vec::new();
        if flate2::read::GzDecoder::new(raw).read_to_end(&mut buf).is_ok() {
            s = String::from_utf8_lossy(&buf).into_owned();
        }
        s
    } else {
        String::from_utf8_lossy(raw).into_owned()
    }
}

// ---------------------------------------------------------------- ILS records

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkerKind {
    Outer,
    Middle,
    Inner,
}

impl MarkerKind {
    fn code(self) -> char {
        match self {
            MarkerKind::Outer => 'O',
            MarkerKind::Middle => 'M',
            MarkerKind::Inner => 'I',
        }
    }
    fn from_code(c: char) -> Option<MarkerKind> {
        Some(match c {
            'O' => MarkerKind::Outer,
            'M' => MarkerKind::Middle,
            'I' => MarkerKind::Inner,
            _ => return None,
        })
    }
}

/// A marker beacon on an approach (75 MHz fan marker).
#[derive(Clone, Debug, PartialEq)]
pub struct Marker {
    pub kind: MarkerKind,
    pub ll: LL,
}

/// One published localizer approach (nav.dat rows 4 / 5 with their 6, 12
/// and 7–9 rows).
#[derive(Clone, Debug, PartialEq)]
pub struct RealIls {
    pub airport: String,
    pub runway: String,
    pub ident: String,
    pub freq_khz: u32,
    /// Front course (° true).
    pub course_deg: f32,
    /// "ILS-cat-I", "LOC", "LDA", "SDF", "IGS"...
    pub facility: String,
    pub loc: LL,
    pub elev_m: f32,
    pub range_nm: f32,
    /// Glideslope antenna and angle (°).
    pub gs: Option<(LL, f32)>,
    pub dme: Option<LL>,
    pub markers: Vec<Marker>,
}

/// Same runway end? ("09L" = "9L").
pub fn rwy_eq(a: &str, b: &str) -> bool {
    a.trim_start_matches('0').eq_ignore_ascii_case(b.trim_start_matches('0'))
}

/// Join the rows of each localizer approach.
pub fn ils_records(recs: &[Rec]) -> Vec<RealIls> {
    let key = |r: &Rec| (r.airport.clone(), r.ident.clone(), r.freq);
    let mut gs: HashMap<_, &Rec> = HashMap::new();
    let mut dme: HashMap<_, &Rec> = HashMap::new();
    let mut markers: HashMap<(String, String), Vec<Marker>> = HashMap::new();
    for r in recs {
        match r.code {
            6 => {
                gs.entry(key(r)).or_insert(r);
            }
            12 | 13 if !r.airport.is_empty() => {
                dme.entry(key(r)).or_insert(r);
            }
            7..=9 => {
                let kind = [MarkerKind::Outer, MarkerKind::Middle, MarkerKind::Inner][(r.code - 7) as usize];
                markers.entry((r.airport.clone(), r.runway.trim_start_matches('0').to_ascii_uppercase())).or_default().push(Marker { kind, ll: (r.lat, r.lon) });
            }
            _ => {}
        }
    }
    let mut out = Vec::new();
    for r in recs.iter().filter(|r| matches!(r.code, 4 | 5) && !r.airport.is_empty() && r.freq > 0) {
        let k = key(r);
        // A glideslope only belongs to a code-4 localizer.
        let g = if r.code == 4 { gs.get(&k).map(|g| ((g.lat, g.lon), g.gs_angle_course().0)) } else { None };
        let mk = markers.get(&(r.airport.clone(), r.runway.trim_start_matches('0').to_ascii_uppercase())).cloned().unwrap_or_default();
        out.push(RealIls {
            airport: r.airport.clone(),
            runway: r.runway.clone(),
            ident: r.ident.clone(),
            freq_khz: r.freq_khz(),
            course_deg: (r.extra as f32).rem_euclid(360.0),
            facility: if r.name.is_empty() { if r.code == 4 { "ILS" } else { "LOC" }.into() } else { r.name.clone() },
            loc: (r.lat, r.lon),
            elev_m: r.elev_ft * FT,
            range_nm: if r.range_nm > 0.0 { r.range_nm } else { 18.0 },
            gs: g,
            dme: dme.get(&k).map(|d| (d.lat, d.lon)),
            markers: mk,
        });
    }
    out
}

/// The VOR / NDB / DME rows as navaids (cross-checks; the sim's list is
/// OurAirports'). The DME half of a VOR-DME is folded into its VOR.
pub fn navaids(recs: &[Rec]) -> Vec<Navaid> {
    let mut out = Vec::new();
    for r in recs {
        let up = r.name.to_ascii_uppercase();
        let kind = match r.code {
            2 if up.ends_with("NDB-DME") => Kind::NdbDme,
            2 => Kind::Ndb,
            3 if up.ends_with("VORTAC") => Kind::Vortac,
            3 if up.ends_with("VOR-DME") || up.ends_with("VOR/DME") => Kind::VorDme,
            3 => Kind::Vor,
            13 if up.ends_with("TACAN") => Kind::Tacan,
            13 => Kind::Dme,
            _ => continue,
        };
        out.push(Navaid {
            kind,
            ident: r.ident.clone(),
            name: r.name.clone(),
            freq_khz: r.freq_khz(),
            lat: r.lat,
            lon: r.lon,
            elev_m: r.elev_ft * FT,
            dme: None,
            range_nm: r.range_nm,
            airport: String::new(),
            ils: None,
        });
    }
    out
}

// ---------------------------------------------------------------- compact list

fn opt_ll(ll: Option<LL>) -> (String, String) {
    ll.map(|l| (format!("{:.6}", l.0), format!("{:.6}", l.1))).unwrap_or_default()
}

fn clean(s: &str) -> String {
    s.replace(['\t', '\n', '\r', ',', ':'], " ")
}

pub fn encode(list: &[RealIls]) -> String {
    let mut s = String::with_capacity(list.len() * 120);
    s.push_str(CACHE_VERSION);
    s.push('\n');
    for r in list {
        let (gl, gn) = opt_ll(r.gs.map(|g| g.0));
        let ga = r.gs.map(|g| format!("{:.2}", g.1)).unwrap_or_default();
        let (dl, dn) = opt_ll(r.dme);
        let mk: Vec<String> = r.markers.iter().map(|m| format!("{}:{:.6}:{:.6}", m.kind.code(), m.ll.0, m.ll.1)).collect();
        s.push_str(&format!(
            "{}\t{}\t{}\t{}\t{:.3}\t{}\t{:.6}\t{:.6}\t{:.0}\t{:.0}\t{gl}\t{gn}\t{ga}\t{dl}\t{dn}\t{}\n",
            clean(&r.airport),
            clean(&r.runway),
            clean(&r.ident),
            r.freq_khz,
            r.course_deg,
            clean(&r.facility),
            r.loc.0,
            r.loc.1,
            r.elev_m,
            r.range_nm,
            mk.join(",")
        ));
    }
    s
}

pub fn decode(text: &str) -> Option<Vec<RealIls>> {
    let mut lines = text.lines();
    if lines.next()? != CACHE_VERSION {
        return None;
    }
    let mut out = Vec::new();
    for l in lines {
        let f: Vec<&str> = l.split('\t').collect();
        if f.len() < 16 {
            continue;
        }
        let ll = |a: &str, b: &str| -> Option<LL> { Some((a.parse().ok()?, b.parse().ok()?)) };
        let Some(loc) = ll(f[6], f[7]) else { continue };
        let markers = f[15]
            .split(',')
            .filter_map(|m| {
                let p: Vec<&str> = m.split(':').collect();
                Some(Marker { kind: MarkerKind::from_code(p.first()?.chars().next()?)?, ll: ll(p.get(1)?, p.get(2)?)? })
            })
            .collect();
        out.push(RealIls {
            airport: f[0].into(),
            runway: f[1].into(),
            ident: f[2].into(),
            freq_khz: f[3].parse().ok()?,
            course_deg: f[4].parse().ok()?,
            facility: f[5].into(),
            loc,
            elev_m: f[8].parse().unwrap_or(0.0),
            range_nm: f[9].parse().unwrap_or(18.0),
            gs: ll(f[10], f[11]).map(|g| (g, f[12].parse().unwrap_or(3.0))),
            dme: ll(f[13], f[14]),
            markers,
        });
    }
    Some(out)
}

pub fn compact(list: &[RealIls]) -> Result<Vec<u8>, String> {
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    gz.write_all(encode(list).as_bytes()).map_err(|e| e.to_string())?;
    gz.finish().map_err(|e| e.to_string())
}

/// nav.dat(.gz) bytes → the compact gzipped ILS list (None if it looks wrong).
pub fn compact_from_navdat(raw: &[u8]) -> Option<(Vec<u8>, usize)> {
    let list = ils_records(&parse(&text_of(raw)));
    (list.len() >= 1000).then(|| compact(&list).ok().map(|b| (b, list.len())))?
}

// ---------------------------------------------------------------- database

/// Real ILS by airport (ICAO, upper case).
pub struct IlsDb {
    by_airport: HashMap<String, Vec<RealIls>>,
    pub count: usize,
}

impl IlsDb {
    pub fn new(list: Vec<RealIls>) -> IlsDb {
        let count = list.len();
        let mut by_airport: HashMap<String, Vec<RealIls>> = HashMap::new();
        for r in list {
            by_airport.entry(r.airport.to_ascii_uppercase()).or_default().push(r);
        }
        IlsDb { by_airport, count }
    }
    /// The approaches of an airport (by its ICAO code, else its ident).
    pub fn at(&self, ap: &crate::airports::Airport) -> &[RealIls] {
        [ap.icao.as_str(), ap.ident.as_str()].iter().filter(|c| !c.is_empty()).find_map(|c| self.by_airport.get(&c.to_ascii_uppercase())).map(|v| v.as_slice()).unwrap_or(&[])
    }
}

enum Load {
    Idle,
    Loading,
    Ready(Arc<IlsDb>),
    Failed(String),
}

static STATE: Mutex<Load> = Mutex::new(Load::Idle);

/// The real ILS list once loaded (starts loading on first call). Unit tests
/// never download (they build an [`IlsDb`] from a fixture).
pub fn ils_db() -> Option<Arc<IlsDb>> {
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

/// Finished loading (ready or failed): headless runs wait for this.
pub fn settled() -> bool {
    matches!(*STATE.lock().unwrap_or_else(|e| e.into_inner()), Load::Ready(_) | Load::Failed(_))
}

pub fn status() -> String {
    match &*STATE.lock().unwrap_or_else(|e| e.into_inner()) {
        Load::Idle => "ILS not loaded".into(),
        Load::Loading => "ILS loading".into(),
        Load::Ready(d) => format!("{} ILS / LOC (nav.dat), synthetic elsewhere", d.count),
        Load::Failed(_) => "ILS synthetic (nav.dat unavailable)".into(),
    }
}

fn finish(r: Result<Vec<RealIls>, String>) {
    let mut st = STATE.lock().unwrap_or_else(|e| e.into_inner());
    match r {
        Ok(list) => *st = Load::Ready(Arc::new(IlsDb::new(list))),
        Err(e) => {
            eprintln!("ils: {e} (synthetic ILS only)");
            *st = Load::Failed(e);
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn cache_path() -> std::path::PathBuf {
    std::path::PathBuf::from("cache").join("ils.tsv.gz")
}

#[cfg(not(target_arch = "wasm32"))]
fn read_cache() -> Option<Vec<RealIls>> {
    let raw = std::fs::read(cache_path()).ok()?;
    decode(&text_of(&raw))
}

#[cfg(not(target_arch = "wasm32"))]
fn download() -> Result<Vec<RealIls>, String> {
    use crate::airports::Fetcher;
    let raw = crate::airports::HttpFetcher.get(NAVDAT_URL)?;
    let list = ils_records(&parse(&text_of(&raw)));
    if list.len() < 1000 {
        return Err(format!("ILS list too short ({})", list.len()));
    }
    let bytes = compact(&list)?;
    let _ = std::fs::create_dir_all("cache");
    let tmp = cache_path().with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, cache_path());
    }
    eprintln!("ils: downloaded nav.dat ({:.1} MB gz) → {} ILS / LOC, {:.2} MB cache", raw.len() as f64 / 1e6, list.len(), bytes.len() as f64 / 1e6);
    Ok(list)
}

/// The list is pinned (see [`NAVDAT_URL`]): a cache never goes stale, only
/// a new [`CACHE_VERSION`] replaces it.
#[cfg(not(target_arch = "wasm32"))]
fn start() {
    let _ = std::thread::Builder::new().name("ils".into()).spawn(|| match read_cache() {
        Some(list) => finish(Ok(list)),
        None => finish(download()),
    });
}

#[cfg(target_arch = "wasm32")]
fn start() {
    crate::platform::web::spawn_local(async move {
        use crate::platform::web;
        let t = web_time::Duration::from_secs(90);
        let r = if web::load_manifest().await.is_ok() && web::has_extra("data/ils.tsv.gz") {
            web::get("data/ils.tsv.gz", t).await.and_then(|raw| decode(&text_of(&raw)).ok_or_else(|| "bad ILS list".to_string()))
        } else {
            Err("no compact ILS list on this page".into())
        };
        finish(r);
    });
}
