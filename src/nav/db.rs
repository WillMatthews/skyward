//! Navaid database.
//!
//! * **VOR, VOR-DME, VORTAC, TACAN, DME, NDB, NDB-DME** from OurAirports'
//!   `navaids.csv` (public domain, <https://ourairports.com/data/>):
//!   frequencies, positions, elevations, DME antennas, power class. Downloaded
//!   once and compacted into `cache/navaids.tsv.gz` (≈ 0.3 MB); the browser
//!   build fetches the CSV (GitHub Pages sends CORS headers) or a compact copy
//!   shipped next to the page (`data/navaids.tsv.gz`).
//! * **ILS** (localizer + glideslope + DME + markers): the published
//!   facilities from FlightGear's `nav.dat` ([`super::navdat`]; real
//!   frequencies, idents, front courses, localizer / glideslope / DME antenna
//!   sites, glidepath angles), the threshold from the OurAirports runway.
//!   At an airport nav.dat has no ILS for (or every airport, before /
//!   without the list), every instrument runway end (paved, lit, ≥ 1200 m,
//!   surveyed ends, at an airport / major airport) gets a **synthetic** ILS derived from its
//!   OurAirports geometry: localizer 300 m beyond the stop end on the runway
//!   course, glideslope abeam the touchdown zone for a 50 ft threshold
//!   crossing height, DME at the glideslope, a frequency from the 40 ILS
//!   channels picked by hashing the airport / runway (clear of the airport's
//!   real ones), ident "I" + airport + runway. A short table of facts
//!   (frequency, ident, glidepath angle) from public charts overrides both
//!   for a few London airports (nav.dat is AIRAC 2013.10, and has EGLC 09 at
//!   5.05°). Synthetic records are flagged (`synthetic`).

use super::geo::{self, LL};
use super::navdat::{self, IlsDb, Marker, RealIls};
use crate::airports::{self, Airport, Rwy};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use web_time::Duration;

pub const NAVAIDS_URL: &str = "https://davidmegginson.github.io/ourairports-data/navaids.csv";
/// Shown in the credits.
pub const CREDIT: &str = "Navaids: OurAirports (public domain); ILS: FlightGear nav.dat (Robin A. Peel, GPL-2.0-or-later), synthetic elsewhere";
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
    /// Marker beacons (nav.dat; none on synthetic ILS).
    pub markers: Vec<Marker>,
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

/// The synthetic ILS of runway end `end` (0 = low end) of `rw` at `ap`, if
/// it is an instrument runway.
pub fn ils_for(ap: &Airport, rw: &Rwy, end: usize) -> Option<Navaid> {
    synthetic_ils(ap, rw, end, &[])
}

/// [`ils_for`] on a channel not in `avoid` (the airport's real ILS).
fn synthetic_ils(ap: &Airport, rw: &Rwy, end: usize, avoid: &[u32]) -> Option<Navaid> {
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
            let k = (hash(&format!("{code}/{pair}")) % ch.len() as u64) as usize;
            let f = (0..ch.len()).map(|i| ch[(k + i) % ch.len()]).find(|f| !avoid.contains(f)).unwrap_or(ch[k]);
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
        ils: Some(Ils {
            airport: ap.ident.clone(),
            runway: a.ident.clone(),
            thr,
            thr_elev: elev,
            course,
            loc,
            gs: Some(gs),
            gs_angle,
            half_width: half_width(geo::gc_dist(thr, loc) as f32),
            length,
            synthetic,
            markers: Vec::new(),
        }),
    })
}

/// Course width: ±105 m (350 ft) at the threshold, 3°..6° wide.
fn half_width(thr_to_loc: f32) -> f32 {
    (105.0 / thr_to_loc.max(1.0)).atan().clamp(1.5f32.to_radians(), 3.0f32.to_radians())
}

/// A published ILS / LOC (nav.dat) at `ap`. The threshold comes from the
/// OurAirports runway end of the same ident when it is surveyed; else from
/// the glideslope site (50 ft crossing height), else the runway length
/// before the localizer. Frequencies, idents and angles in [`PUBLISHED`]
/// override the (older) nav.dat ones.
pub fn real_ils(ap: &Airport, r: &RealIls) -> Navaid {
    let course = r.course_deg.to_radians();
    let back = geo::wrap_2pi(course + std::f32::consts::PI);
    // The runway end of that ident, else (renumbered since the AIRAC cycle)
    // the surveyed end whose centreline the localizer sits on.
    let on_centreline = |rw: &Rwy, e: usize| {
        let (a, b) = (&rw.ends[e], &rw.ends[1 - e]);
        let (Some(pa), Some(pb)) = (a.lat.zip(a.lon), b.lat.zip(b.lon)) else { return false };
        let c = geo::gc_bearing(pa, pb);
        let (d, rel) = (geo::gc_dist(pb, r.loc) as f32, geo::wrap_pi(geo::gc_bearing(pb, r.loc) - c));
        geo::wrap_pi(c - course).abs() < 5f32.to_radians() && (-500.0..3000.0).contains(&(d * rel.cos())) && (d * rel.sin()).abs() < 250.0
    };
    let end = ap
        .runways
        .iter()
        .find_map(|rw| (0..2).find(|&e| navdat::rwy_eq(&rw.ends[e].ident, &r.runway)).map(|e| (rw, e)))
        .or_else(|| ap.runways.iter().find_map(|rw| (0..2).find(|&e| on_centreline(rw, e)).map(|e| (rw, e))));
    let runway = end.map(|(rw, e)| rw.ends[e].ident.clone()).unwrap_or_else(|| r.runway.clone());
    let published = PUBLISHED.iter().find(|p| p.0 == ap.code() && p.1 == runway);
    let gs_deg = published.map(|p| p.4).or(r.gs.map(|g| g.1)).unwrap_or(3.0);
    let gs_angle = gs_deg.to_radians();
    let surveyed = end.and_then(|(rw, e)| {
        let (a, b) = (&rw.ends[e], &rw.ends[1 - e]);
        let (pa, pb) = ((a.lat?, a.lon?), (b.lat?, b.lon?));
        let len = geo::gc_dist(pa, pb) as f32;
        Some((geo::gc_dest(pa, geo::gc_bearing(pa, pb), a.displaced_m as f64), len - a.displaced_m))
    });
    let (thr, length) = match surveyed {
        Some(t) => t,
        None => {
            let length = end.map_or(2500.0, |(rw, e)| rw.length_m - rw.ends[e].displaced_m).max(600.0);
            let from_loc = match r.gs {
                Some((g, _)) => {
                    let abeam = geo::gc_dist(r.loc, g) as f32 * geo::wrap_pi(geo::gc_bearing(r.loc, g) - back).cos();
                    abeam + 50.0 * FT / gs_angle.tan()
                }
                None => length + 300.0,
            };
            (geo::gc_dest(r.loc, back, from_loc as f64), length)
        }
    };
    let gs = r.gs.map(|g| g.0);
    let elev = ap.elev_ft * FT;
    let (freq, ident) = published.map_or((r.freq_khz, r.ident.clone()), |p| (p.2, p.3.to_string()));
    Navaid {
        kind: if gs.is_some() { Kind::Ils } else { Kind::Loc },
        ident,
        name: format!("{} RWY {}", ap.name, runway),
        freq_khz: freq,
        lat: r.loc.0,
        lon: r.loc.1,
        elev_m: r.elev_m,
        dme: r.dme.or(gs),
        range_nm: r.range_nm,
        airport: ap.ident.clone(),
        ils: Some(Ils {
            airport: ap.ident.clone(),
            runway,
            thr,
            thr_elev: elev,
            course,
            loc: r.loc,
            gs,
            gs_angle,
            half_width: half_width(geo::gc_dist(thr, r.loc) as f32),
            length,
            synthetic: false,
            markers: r.markers.clone(),
        }),
    }
}

/// nav.dat's ILS list is loaded (starts loading it).
pub fn real_ils_ready() -> bool {
    navdat::ils_db().is_some()
}

/// Every ILS at the airports within `km` of `ll`.
pub fn ils_near(adb: &airports::Db, ll: LL, km: f64) -> Vec<Navaid> {
    let real = navdat::ils_db();
    let mut out = Vec::new();
    for (i, _) in adb.near(ll.0, ll.1, km) {
        out.extend(ils_at_with(&adb.airports[i], real.as_deref()));
    }
    out
}

/// Every ILS of one airport: nav.dat's once loaded, synthetic elsewhere.
pub fn ils_at(ap: &Airport) -> Vec<Navaid> {
    ils_at_with(ap, navdat::ils_db().as_deref())
}

/// [`ils_at`] with a given real-ILS list (None = synthetic only). Real ILS
/// come first (ILS before localizer-only). An airport nav.dat covers keeps
/// only its real ILS (plus the chart table's runway ends): the runway ends it
/// lists none for have none in the world. Airports it does not cover get a
/// synthetic ILS on each instrument runway end.
pub fn ils_at_with(ap: &Airport, real: Option<&IlsDb>) -> Vec<Navaid> {
    let mut out: Vec<Navaid> = real.map_or(&[][..], |d| d.at(ap)).iter().map(|r| real_ils(ap, r)).collect();
    out.sort_by_key(|n| n.kind != Kind::Ils);
    let covered = !out.is_empty();
    let code = ap.code();
    let used: Vec<u32> = out.iter().map(|n| n.freq_khz).collect();
    for rw in &ap.runways {
        for e in 0..2 {
            let id = &rw.ends[e].ident;
            if covered && !PUBLISHED.iter().any(|p| p.0 == code && p.1 == *id) {
                continue;
            }
            if !out.iter().any(|n| n.ils.as_ref().is_some_and(|i| navdat::rwy_eq(&i.runway, id))) {
                out.extend(synthetic_ils(ap, rw, e, &used));
            }
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

/// Being read or downloaded (the menus' background indicator).
pub fn loading() -> bool {
    matches!(*STATE.lock().unwrap_or_else(|e| e.into_inner()), Load::Loading)
}

/// Status line for the credits / diagnostics.
pub fn status() -> String {
    match &*STATE.lock().unwrap_or_else(|e| e.into_inner()) {
        Load::Idle => "not loaded".into(),
        Load::Loading => "loading".into(),
        Load::Ready(d) => format!("{} navaids (OurAirports) · {}", d.navaids.len(), navdat::status()),
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

/// Block until loaded (headless runs), the real ILS list too (so a run
/// does not change ILS halfway). Native only.
#[cfg(not(target_arch = "wasm32"))]
pub fn wait(timeout: Duration) -> Option<Arc<NavDb>> {
    let t = web_time::Instant::now();
    loop {
        let ils = navdat::ils_db().is_some() || navdat::settled() || t.elapsed() > timeout;
        if let Some(d) = db().filter(|_| ils) {
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
