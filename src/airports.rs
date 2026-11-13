//! Airport and runway database from OurAirports (public domain,
//! <https://ourairports.com/data/>): exact runway ends, headings, widths,
//! displaced thresholds, surface and lighting for ~50 000 airfields.
//!
//! The two CSVs (≈17 MB) are downloaded once and compacted into
//! `cache/ourairports.tsv.gz` (≈2 MB); later runs read only that file.
//! Loading goes through [`Fetcher`] (plain HTTP GET of a URL → bytes) on a
//! background thread, so a wasm build can plug in a browser fetch.
//!
//! Also here: fuzzy search, spatial queries, conversion of runways into the
//! sim's world frame, per-runway wind components and a METAR lookup for a
//! single airport (decoded with `weather::Metar`).

use crate::osm::Proj;
use crate::scene::{Runway, RunwaySource};
use glam::Vec2;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::Arc;
#[cfg(not(target_arch = "wasm32"))]
use web_time::Instant;
use web_time::Duration;

pub const AIRPORTS_URL: &str = "https://davidmegginson.github.io/ourairports-data/airports.csv";
pub const RUNWAYS_URL: &str = "https://davidmegginson.github.io/ourairports-data/runways.csv";
/// Shown in the credits.
pub const CREDIT: &str = "Airports: OurAirports (public domain)";
const USER_AGENT: &str = "skyward-flightsim/0.2";
const FT: f32 = 0.3048;
/// Cache format version (bump when the compact format changes).
const CACHE_VERSION: &str = "skyward-ourairports-1";
/// Refresh the download after this long (the old copy keeps working).
const MAX_AGE: Duration = Duration::from_secs(45 * 24 * 3600);

// ---------------------------------------------------------------- types

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    Large,
    Medium,
    Small,
    Seaplane,
}

impl Kind {
    fn parse(s: &str) -> Option<Kind> {
        Some(match s {
            "large_airport" => Kind::Large,
            "medium_airport" => Kind::Medium,
            "small_airport" => Kind::Small,
            "seaplane_base" => Kind::Seaplane,
            _ => return None,
        })
    }
    fn code(self) -> &'static str {
        match self {
            Kind::Large => "L",
            Kind::Medium => "M",
            Kind::Small => "S",
            Kind::Seaplane => "W",
        }
    }
    fn from_code(s: &str) -> Kind {
        match s {
            "L" => Kind::Large,
            "M" => Kind::Medium,
            "W" => Kind::Seaplane,
            _ => Kind::Small,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Kind::Large => "Major airport",
            Kind::Medium => "Airport",
            Kind::Small => "Airfield",
            Kind::Seaplane => "Seaplane base",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Surface {
    Asphalt,
    Concrete,
    Grass,
    Gravel,
    Dirt,
    Water,
    Snow,
    Other,
    Unknown,
}

impl Surface {
    /// OurAirports surface strings are free text ("ASP", "Asphalt", "CON", "TURF", "GRS", "ASPH-G", ...).
    pub fn parse(s: &str) -> Surface {
        let u = s.trim().to_ascii_uppercase();
        let has = |k: &str| u.contains(k);
        if u.is_empty() {
            Surface::Unknown
        } else if has("ASP") || has("BIT") || has("TAR") || has("PAV") || has("PEM") || u == "ASPHALT" {
            Surface::Asphalt
        } else if has("CON") || has("PCC") || has("CEM") {
            Surface::Concrete
        } else if has("WAT") {
            Surface::Water
        } else if has("GRS") || has("GRA") && !has("GRAV") && !has("GRAVEL") || has("TURF") || has("GRASS") {
            Surface::Grass
        } else if has("GRV") || has("GRAV") || has("GVL") || has("COR") || has("ROCK") {
            Surface::Gravel
        } else if has("DIRT") || has("SAND") || has("CLAY") || has("SOIL") || has("EARTH") || has("LAT") || has("SAN") {
            Surface::Dirt
        } else if has("SNOW") || has("ICE") {
            Surface::Snow
        } else {
            Surface::Other
        }
    }
    pub fn paved(self) -> bool {
        matches!(self, Surface::Asphalt | Surface::Concrete)
    }
    pub fn name(self) -> &'static str {
        match self {
            Surface::Asphalt => "Asphalt",
            Surface::Concrete => "Concrete",
            Surface::Grass => "Grass",
            Surface::Gravel => "Gravel",
            Surface::Dirt => "Dirt",
            Surface::Water => "Water",
            Surface::Snow => "Snow",
            Surface::Other => "Other",
            Surface::Unknown => "Unknown",
        }
    }
    fn code(self) -> &'static str {
        match self {
            Surface::Asphalt => "A",
            Surface::Concrete => "C",
            Surface::Grass => "G",
            Surface::Gravel => "V",
            Surface::Dirt => "D",
            Surface::Water => "W",
            Surface::Snow => "S",
            Surface::Other => "O",
            Surface::Unknown => "U",
        }
    }
    fn from_code(s: &str) -> Surface {
        match s {
            "A" => Surface::Asphalt,
            "C" => Surface::Concrete,
            "G" => Surface::Grass,
            "V" => Surface::Gravel,
            "D" => Surface::Dirt,
            "W" => Surface::Water,
            "S" => Surface::Snow,
            "O" => Surface::Other,
            _ => Surface::Unknown,
        }
    }
}

/// One end of a runway ("le" = low end, "he" = high end in OurAirports).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct End {
    pub ident: String,
    /// Physical runway end (start of the pavement), if surveyed.
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    /// Degrees true, pointing down the runway from this end.
    pub hdg_true: Option<f32>,
    /// Displaced threshold (m from the physical end).
    pub displaced_m: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Rwy {
    pub length_m: f32,
    pub width_m: f32,
    pub surface: Surface,
    pub lighted: bool,
    pub ends: [End; 2],
}

impl Rwy {
    pub fn name(&self) -> String {
        format!("{}/{}", self.ends[0].ident, self.ends[1].ident)
    }
    /// True heading of end `i` (from the surveyed ends, else the data, else the ident).
    pub fn heading(&self, i: usize) -> Option<f32> {
        let (a, b) = (&self.ends[i], &self.ends[1 - i]);
        if let (Some(la), Some(lo), Some(lb), Some(lob)) = (a.lat, a.lon, b.lat, b.lon) {
            let k = (la.to_radians()).cos();
            let (dx, dy) = ((lob - lo) * k, lb - la);
            if dx.abs() + dy.abs() > 1e-6 {
                return Some((dx.atan2(dy).to_degrees() as f32).rem_euclid(360.0));
            }
        }
        if let Some(h) = a.hdg_true {
            return Some(h);
        }
        ident_heading(&a.ident).or_else(|| ident_heading(&b.ident).map(|h| (h + 180.0).rem_euclid(360.0)))
    }
    pub fn surveyed(&self) -> bool {
        self.ends.iter().all(|e| e.lat.is_some() && e.lon.is_some())
    }
}

/// "09L" → 90°, "36" → 360°, "N" / "H1" → None.
pub fn ident_heading(id: &str) -> Option<f32> {
    let d: String = id.chars().take_while(|c| c.is_ascii_digit()).collect();
    let n: f32 = d.parse().ok()?;
    (1.0..=36.0).contains(&n).then_some(n * 10.0)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Airport {
    pub ident: String,
    pub kind: Kind,
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    pub country: String,
    pub region: String,
    pub city: String,
    pub scheduled: bool,
    pub icao: String,
    pub iata: String,
    pub runways: Vec<Rwy>,
}

impl Airport {
    /// The code to show: ICAO, else the ident.
    pub fn code(&self) -> &str {
        if !self.icao.is_empty() { &self.icao } else { &self.ident }
    }
    pub fn longest_m(&self) -> f32 {
        self.runways.iter().map(|r| r.length_m).fold(0.0, f32::max)
    }
    pub fn longest_paved_m(&self) -> f32 {
        self.runways.iter().filter(|r| r.surface.paved()).map(|r| r.length_m).fold(0.0, f32::max)
    }
    /// Longest runway usable by an aircraft that needs `paved` surfaces.
    pub fn usable_m(&self, paved: bool) -> f32 {
        if paved { self.longest_paved_m() } else { self.longest_m() }
    }
}

/// Distance (km) on the sphere.
pub fn distance_km(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (la1, lo1, la2, lo2) = (a.0.to_radians(), a.1.to_radians(), b.0.to_radians(), b.1.to_radians());
    let h = ((la2 - la1) / 2.0).sin().powi(2) + la1.cos() * la2.cos() * ((lo2 - lo1) / 2.0).sin().powi(2);
    2.0 * 6371.0 * h.sqrt().asin()
}

// ---------------------------------------------------------------- database

#[allow(dead_code)]
pub struct Db {
    pub airports: Vec<Airport>,
    by_ident: HashMap<String, usize>,
    /// 1° cells → airport indices.
    grid: HashMap<(i32, i32), Vec<u32>>,
    /// Lower-cased search text per airport: "ident icao iata name city region country".
    hay: Vec<String>,
    pub source_bytes: usize,
}

fn cell(lat: f64, lon: f64) -> (i32, i32) {
    (lat.floor() as i32, lon.floor() as i32)
}

impl Db {
    pub fn new(airports: Vec<Airport>, source_bytes: usize) -> Db {
        let mut by_ident = HashMap::new();
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        let mut hay = Vec::with_capacity(airports.len());
        for (i, a) in airports.iter().enumerate() {
            by_ident.insert(a.ident.to_ascii_uppercase(), i);
            if !a.icao.is_empty() {
                by_ident.entry(a.icao.to_ascii_uppercase()).or_insert(i);
            }
            grid.entry(cell(a.lat, a.lon)).or_default().push(i as u32);
            hay.push(fold(&format!("{} {} {} {} {} {} {}", a.ident, a.icao, a.iata, a.name, a.city, a.region, a.country)));
        }
        Db { airports, by_ident, grid, hay, source_bytes }
    }

    pub fn get(&self, ident: &str) -> Option<&Airport> {
        self.by_ident.get(&ident.trim().to_ascii_uppercase()).map(|&i| &self.airports[i])
    }
    pub fn index_of(&self, ident: &str) -> Option<usize> {
        self.by_ident.get(&ident.trim().to_ascii_uppercase()).copied()
    }

    /// Airports within `km` of a point, nearest first.
    pub fn near(&self, lat: f64, lon: f64, km: f64) -> Vec<(usize, f64)> {
        let dlat = (km / 111.0).ceil() as i32 + 1;
        let dlon = (km / (111.0 * lat.to_radians().cos().max(0.05))).ceil().min(180.0) as i32 + 1;
        let (cy, cx) = cell(lat, lon);
        let mut out = Vec::new();
        for y in cy - dlat..=cy + dlat {
            for x in cx - dlon..=cx + dlon {
                let xw = (x + 180).rem_euclid(360) - 180;
                if let Some(v) = self.grid.get(&(y, xw)) {
                    for &i in v {
                        let a = &self.airports[i as usize];
                        let d = distance_km((lat, lon), (a.lat, a.lon));
                        if d <= km {
                            out.push((i as usize, d));
                        }
                    }
                }
            }
        }
        out.sort_by(|a, b| a.1.total_cmp(&b.1));
        out
    }

    /// Airports inside a lat/lon box (for the map), any order.
    pub fn in_box(&self, lat0: f64, lat1: f64, lon0: f64, lon1: f64) -> Vec<usize> {
        let mut out = Vec::new();
        let (y0, y1) = (lat0.floor() as i32, lat1.floor() as i32);
        let wide = lon1 - lon0 >= 359.0;
        let (x0, x1) = if wide { (-180, 179) } else { (lon0.floor() as i32, lon1.floor() as i32) };
        for y in y0..=y1 {
            for x in x0..=x1 {
                let xw = (x + 180).rem_euclid(360) - 180;
                if let Some(v) = self.grid.get(&(y, xw)) {
                    out.extend(v.iter().map(|&i| i as usize));
                }
            }
        }
        out
    }

    /// Fuzzy search by name, city, ICAO, IATA or ident. Best first.
    pub fn search(&self, query: &str, limit: usize, filter: impl Fn(&Airport) -> bool) -> Vec<usize> {
        let q = fold(query);
        let toks: Vec<&str> = q.split_whitespace().collect();
        if toks.is_empty() {
            return Vec::new();
        }
        let up = query.trim().to_ascii_uppercase();
        let mut hits: Vec<(i32, usize)> = Vec::new();
        for (i, a) in self.airports.iter().enumerate() {
            let code_score = if a.ident.eq_ignore_ascii_case(&up) || a.icao.eq_ignore_ascii_case(&up) {
                2000
            } else if !a.iata.is_empty() && a.iata.eq_ignore_ascii_case(&up) {
                1900
            } else if up.len() >= 2 && (a.ident.starts_with(&up) || (!a.icao.is_empty() && a.icao.starts_with(&up))) {
                900
            } else {
                0
            };
            let mut score = code_score;
            if score == 0 {
                let h = &self.hay[i];
                let mut total = 0;
                for t in &toks {
                    let s = token_score(h, t);
                    if s == 0 {
                        total = 0;
                        break;
                    }
                    total += s;
                }
                score = total;
            }
            if score == 0 || !filter(a) {
                continue;
            }
            score += match a.kind {
                Kind::Large => 120,
                Kind::Medium => 60,
                Kind::Small => 0,
                Kind::Seaplane => -20,
            } + if a.scheduled { 40 } else { 0 };
            hits.push((score, i));
        }
        hits.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        hits.truncate(limit);
        hits.into_iter().map(|h| h.1).collect()
    }

    /// Runways of the airports within `km` of a point, in the frame `proj`.
    pub fn runways_near(&self, proj: &Proj, lat: f64, lon: f64, km: f64) -> Vec<Runway> {
        let mut out = Vec::new();
        for (i, _) in self.near(lat, lon, km) {
            let a = &self.airports[i];
            if a.kind == Kind::Seaplane {
                continue;
            }
            for r in &a.runways {
                if let Some(rw) = to_world(a, r, proj) {
                    out.push(rw);
                }
            }
        }
        out
    }
}

/// Lower-case and strip common diacritics.
pub fn fold(s: &str) -> String {
    let mut o = String::with_capacity(s.len());
    for c in s.chars() {
        let m = match c {
            'á' | 'à' | 'â' | 'ä' | 'ã' | 'å' | 'Á' | 'À' | 'Â' | 'Ä' | 'Ã' | 'Å' => 'a',
            'é' | 'è' | 'ê' | 'ë' | 'É' | 'È' | 'Ê' | 'Ë' => 'e',
            'í' | 'ì' | 'î' | 'ï' | 'Í' | 'Ì' | 'Î' | 'Ï' => 'i',
            'ó' | 'ò' | 'ô' | 'ö' | 'õ' | 'ø' | 'Ó' | 'Ò' | 'Ô' | 'Ö' | 'Õ' | 'Ø' => 'o',
            'ú' | 'ù' | 'û' | 'ü' | 'Ú' | 'Ù' | 'Û' | 'Ü' => 'u',
            'ç' | 'Ç' => 'c',
            'ñ' | 'Ñ' => 'n',
            'ß' => 's',
            '-' | '/' | ',' | '(' | ')' | '.' | '\'' => ' ',
            c => c.to_ascii_lowercase(),
        };
        o.push(m);
    }
    o
}

/// How well one query token matches the haystack (0 = not at all).
fn token_score(h: &str, t: &str) -> i32 {
    if let Some(pos) = h.find(t) {
        let word_start = pos == 0 || h.as_bytes()[pos - 1] == b' ';
        let word_end = h.as_bytes().get(pos + t.len()).is_none_or(|c| *c == b' ');
        return match (word_start, word_end) {
            (true, true) => 400,
            (true, false) => 300,
            _ => 150,
        } + if pos < 12 { 30 } else { 0 };
    }
    // Typo tolerance: one substitution / deletion in a word start (tokens ≥ 4).
    if t.len() >= 4 {
        for w in h.split(' ') {
            if w.len() + 1 >= t.len() && near_match(w, t) {
                return 90;
            }
        }
    }
    0
}

/// Prefix of `w` within one edit (substitution, swap, insertion, deletion) of `t`.
fn near_match(w: &str, t: &str) -> bool {
    let (w, t) = (w.as_bytes(), t.as_bytes());
    let n = t.len();
    // substitution
    if w.len() >= n && (0..n).filter(|&i| w[i] != t[i]).count() <= 1 {
        return true;
    }
    // two neighbouring characters swapped
    if w.len() >= n {
        for i in 0..n.saturating_sub(1) {
            if w[i] == t[i + 1] && w[i + 1] == t[i] && w[..i] == t[..i] && w[i + 2..n] == t[i + 2..] {
                return true;
            }
        }
    }
    // deletion from the query (extra char typed) / insertion (missed char)
    for skip in 0..n {
        let tq: Vec<u8> = t.iter().enumerate().filter(|(i, _)| *i != skip).map(|(_, c)| *c).collect();
        if w.len() >= tq.len() && w[..tq.len()] == tq[..] {
            return true;
        }
        if w.len() > n && w[..skip] == t[..skip] && w[skip + 1..=n] == t[skip..] {
            return true;
        }
    }
    false
}

/// One runway in a world frame. Surveyed runways use their exact ends;
/// others are placed through the airport reference point along the ident heading.
pub fn to_world(a: &Airport, r: &Rwy, proj: &Proj) -> Option<Runway> {
    if r.length_m < 100.0 || r.ends.iter().any(|e| e.ident.starts_with('H')) {
        return None;
    }
    let width = if r.width_m > 5.0 { r.width_m } else { Runway::width_for(r.length_m) };
    let (pa, pb, source) = match (r.ends[0].lat, r.ends[0].lon, r.ends[1].lat, r.ends[1].lon) {
        (Some(la), Some(lo), Some(lb), Some(lob)) => (proj.to_world(la, lo), proj.to_world(lb, lob), RunwaySource::OurAirports),
        _ => {
            let h = r.heading(0)?.to_radians();
            let d = glam::vec2(h.sin(), -h.cos());
            let c = proj.to_world(a.lat, a.lon);
            (c - d * r.length_m * 0.5, c + d * r.length_m * 0.5, RunwaySource::Estimated)
        }
    };
    if pa.distance(pb) < 100.0 {
        return None;
    }
    Some(Runway {
        a: pa,
        b: pb,
        width,
        name: r.name(),
        ident_a: r.ends[0].ident.clone(),
        ident_b: r.ends[1].ident.clone(),
        displaced_a: r.ends[0].displaced_m,
        displaced_b: r.ends[1].displaced_m,
        paved: r.surface.paved(),
        surface: r.surface.name().to_string(),
        lighted: r.lighted,
        airport: a.ident.clone(),
        source,
        ..Default::default()
    })
}

// ---------------------------------------------------------------- CSV import

/// Minimal RFC-4180 CSV row splitter (quotes, doubled quotes, commas in fields).
pub(crate) fn csv_rows(text: &str) -> impl Iterator<Item = Vec<String>> + '_ {
    let mut chars = text.chars().peekable();
    std::iter::from_fn(move || {
        chars.peek()?;
        let mut row = Vec::new();
        let mut field = String::new();
        let mut quoted = false;
        loop {
            match chars.next() {
                None => {
                    row.push(std::mem::take(&mut field));
                    return Some(row);
                }
                Some('"') if quoted => {
                    if chars.peek() == Some(&'"') {
                        chars.next();
                        field.push('"');
                    } else {
                        quoted = false;
                    }
                }
                Some('"') if field.is_empty() => quoted = true,
                Some(',') if !quoted => row.push(std::mem::take(&mut field)),
                Some('\r') if !quoted => {}
                Some('\n') if !quoted => {
                    row.push(std::mem::take(&mut field));
                    return Some(row);
                }
                Some(c) => field.push(c),
            }
        }
    })
}

/// Parse the two OurAirports CSVs (heliports, balloon ports and closed fields dropped).
pub fn parse_csv(airports_csv: &str, runways_csv: &str) -> Vec<Airport> {
    let mut rows = csv_rows(airports_csv);
    let head = rows.next().unwrap_or_default();
    let col = |n: &str| head.iter().position(|h| h == n);
    let (c_ident, c_type, c_name, c_lat, c_lon, c_elev) = (col("ident"), col("type"), col("name"), col("latitude_deg"), col("longitude_deg"), col("elevation_ft"));
    let (c_ctry, c_reg, c_city, c_sched, c_icao, c_iata, c_gps) =
        (col("iso_country"), col("iso_region"), col("municipality"), col("scheduled_service"), col("icao_code"), col("iata_code"), col("gps_code"));
    let get = |r: &Vec<String>, c: Option<usize>| c.and_then(|i| r.get(i)).cloned().unwrap_or_default();
    let mut out: Vec<Airport> = Vec::new();
    let mut idx: HashMap<String, usize> = HashMap::new();
    for r in rows {
        let Some(kind) = Kind::parse(&get(&r, c_type)) else { continue };
        let (Ok(lat), Ok(lon)) = (get(&r, c_lat).parse::<f64>(), get(&r, c_lon).parse::<f64>()) else { continue };
        let ident = get(&r, c_ident);
        let mut icao = get(&r, c_icao);
        if icao.is_empty() {
            let g = get(&r, c_gps);
            if g.len() == 4 && g.chars().all(|c| c.is_ascii_alphabetic()) {
                icao = g;
            }
        }
        idx.insert(ident.clone(), out.len());
        out.push(Airport {
            ident,
            kind,
            name: get(&r, c_name),
            lat,
            lon,
            elev_ft: get(&r, c_elev).parse().unwrap_or(0.0),
            country: get(&r, c_ctry),
            region: get(&r, c_reg),
            city: get(&r, c_city),
            scheduled: get(&r, c_sched) == "yes",
            icao,
            iata: get(&r, c_iata),
            runways: Vec::new(),
        });
    }
    let mut rows = csv_rows(runways_csv);
    let head = rows.next().unwrap_or_default();
    let col = |n: &str| head.iter().position(|h| h == n);
    let cols = ["airport_ident", "length_ft", "width_ft", "surface", "lighted", "closed"].map(col);
    let end_cols = |p: &str| ["ident", "latitude_deg", "longitude_deg", "heading_degT", "displaced_threshold_ft"].map(|n| col(&format!("{p}_{n}")));
    let (le, he) = (end_cols("le"), end_cols("he"));
    for r in rows {
        let Some(&ai) = idx.get(&get(&r, cols[0])) else { continue };
        if get(&r, cols[5]) == "1" {
            continue;
        }
        let end = |c: &[Option<usize>; 5]| End {
            ident: get(&r, c[0]),
            lat: get(&r, c[1]).parse().ok(),
            lon: get(&r, c[2]).parse().ok(),
            hdg_true: get(&r, c[3]).parse().ok(),
            displaced_m: get(&r, c[4]).parse::<f32>().unwrap_or(0.0) * FT,
        };
        out[ai].runways.push(Rwy {
            length_m: get(&r, cols[1]).parse::<f32>().unwrap_or(0.0) * FT,
            width_m: get(&r, cols[2]).parse::<f32>().unwrap_or(0.0) * FT,
            surface: Surface::parse(&get(&r, cols[3])),
            lighted: get(&r, cols[4]) == "1",
            ends: [end(&le), end(&he)],
        });
    }
    for a in &mut out {
        a.runways.sort_by(|x, y| y.length_m.total_cmp(&x.length_m));
    }
    out
}

// ---------------------------------------------------------------- compact cache

fn clean(s: &str) -> String {
    s.replace(['\t', '\n', '\r'], " ")
}

fn opt<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map(|x| x.to_string()).unwrap_or_default()
}

/// Compact text form: one `A` line per airport followed by its `R` lines.
pub fn encode(airports: &[Airport]) -> String {
    let mut s = String::with_capacity(airports.len() * 120);
    s.push_str(CACHE_VERSION);
    s.push('\n');
    for a in airports {
        s.push_str(&format!(
            "A\t{}\t{}\t{}\t{:.6}\t{:.6}\t{:.0}\t{}\t{}\t{}\t{}\t{}\t{}\n",
            clean(&a.ident),
            a.kind.code(),
            clean(&a.name),
            a.lat,
            a.lon,
            a.elev_ft,
            clean(&a.country),
            clean(&a.region),
            clean(&a.city),
            if a.scheduled { 1 } else { 0 },
            clean(&a.icao),
            clean(&a.iata)
        ));
        for r in &a.runways {
            s.push_str(&format!("R\t{:.0}\t{:.0}\t{}\t{}", r.length_m, r.width_m, r.surface.code(), if r.lighted { 1 } else { 0 }));
            for e in &r.ends {
                s.push_str(&format!(
                    "\t{}\t{}\t{}\t{}\t{:.0}",
                    clean(&e.ident),
                    opt(e.lat.map(|v| format!("{v:.6}"))),
                    opt(e.lon.map(|v| format!("{v:.6}"))),
                    opt(e.hdg_true.map(|v| format!("{v:.1}"))),
                    e.displaced_m
                ));
            }
            s.push('\n');
        }
    }
    s
}

pub fn decode(text: &str) -> Option<Vec<Airport>> {
    let mut lines = text.lines();
    if lines.next()? != CACHE_VERSION {
        return None;
    }
    let mut out: Vec<Airport> = Vec::new();
    for l in lines {
        let f: Vec<&str> = l.split('\t').collect();
        match f.first() {
            Some(&"A") if f.len() >= 13 => out.push(Airport {
                ident: f[1].into(),
                kind: Kind::from_code(f[2]),
                name: f[3].into(),
                lat: f[4].parse().ok()?,
                lon: f[5].parse().ok()?,
                elev_ft: f[6].parse().unwrap_or(0.0),
                country: f[7].into(),
                region: f[8].into(),
                city: f[9].into(),
                scheduled: f[10] == "1",
                icao: f[11].into(),
                iata: f[12].into(),
                runways: Vec::new(),
            }),
            Some(&"R") if f.len() >= 15 => {
                let end = |k: usize| End {
                    ident: f[k].into(),
                    lat: f[k + 1].parse().ok(),
                    lon: f[k + 2].parse().ok(),
                    hdg_true: f[k + 3].parse().ok(),
                    displaced_m: f[k + 4].parse().unwrap_or(0.0),
                };
                let r = Rwy {
                    length_m: f[1].parse().unwrap_or(0.0),
                    width_m: f[2].parse().unwrap_or(0.0),
                    surface: Surface::from_code(f[3]),
                    lighted: f[4] == "1",
                    ends: [end(5), end(10)],
                };
                out.last_mut()?.runways.push(r);
            }
            _ => {}
        }
    }
    Some(out)
}

// ---------------------------------------------------------------- loading

/// Bytes from a URL. Native: blocking HTTP on a worker thread; a wasm build
/// supplies a browser-fetch implementation.
pub trait Fetcher: Send + Sync + 'static {
    fn get(&self, url: &str) -> Result<Vec<u8>, String>;
}

#[cfg(not(target_arch = "wasm32"))]
pub struct HttpFetcher;

#[cfg(not(target_arch = "wasm32"))]
impl Fetcher for HttpFetcher {
    fn get(&self, url: &str) -> Result<Vec<u8>, String> {
        let ag = ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(90)).user_agent(USER_AGENT).build();
        let r = ag.get(url).call().map_err(|e| e.to_string())?;
        let mut b = Vec::new();
        r.into_reader().take(64 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
        Ok(b)
    }
}

fn cache_path() -> PathBuf {
    PathBuf::from("cache").join("ourairports.tsv.gz")
}

fn read_cache() -> Option<(Vec<Airport>, usize, bool)> {
    let path = cache_path();
    let raw = std::fs::read(&path).ok()?;
    let mut text = String::new();
    flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).ok()?;
    let stale = std::fs::metadata(&path).ok().and_then(|m| m.modified().ok()).and_then(|t| t.elapsed().ok()).is_some_and(|age| age > MAX_AGE);
    Some((decode(&text)?, raw.len(), stale))
}

fn download(f: &dyn Fetcher) -> Result<(Vec<Airport>, usize), String> {
    let a = f.get(AIRPORTS_URL)?;
    let r = f.get(RUNWAYS_URL)?;
    let list = parse_csv(&String::from_utf8_lossy(&a), &String::from_utf8_lossy(&r));
    if list.len() < 1000 {
        return Err(format!("airport list too short ({})", list.len()));
    }
    let text = encode(&list);
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    gz.write_all(text.as_bytes()).map_err(|e| e.to_string())?;
    let bytes = gz.finish().map_err(|e| e.to_string())?;
    let _ = std::fs::create_dir_all("cache");
    let tmp = cache_path().with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, cache_path());
    }
    eprintln!("airports: downloaded OurAirports ({:.1} MB csv) → {} airfields, {:.1} MB cache", (a.len() + r.len()) as f64 / 1e6, list.len(), bytes.len() as f64 / 1e6);
    Ok((list, bytes.len()))
}

#[cfg(target_arch = "wasm32")]
pub(crate) async fn web_download() -> Result<(Vec<Airport>, usize), String> {
    use crate::platform::web;
    let t = Duration::from_secs(90);
    // (Listed in the asset manifest when the build shipped it.)
    let compact = web::load_manifest().await.is_ok() && web::has_extra("data/ourairports.tsv.gz");
    if let Some(raw) = if compact { web::get("data/ourairports.tsv.gz", t).await.ok() } else { None } {
        let mut text = String::new();
        if flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut text).is_ok() {
            if let Some(list) = decode(&text) {
                return Ok((list, raw.len()));
            }
        }
    }
    let a = web::get(AIRPORTS_URL, t).await?;
    let r = web::get(RUNWAYS_URL, t).await?;
    let t0 = web_time::Instant::now();
    let list = parse_csv(&String::from_utf8_lossy(&a), &String::from_utf8_lossy(&r));
    web::log(&format!("airports: {} from OurAirports' CSVs, parsed in {:.0} ms", list.len(), t0.elapsed().as_secs_f32() * 1000.0));
    if list.len() < 1000 {
        return Err(format!("airport list too short ({})", list.len()));
    }
    Ok((list, a.len() + r.len()))
}

#[allow(dead_code)]
pub enum Status {
    Loading,
    Ready(Arc<Db>),
    Failed(String),
}

/// Background loader: the cache if present (refreshed in the background
/// when stale), else a download.
pub struct Loader {
    rx: Option<Receiver<Result<Arc<Db>, String>>>,
    pub status: Status,
    pub db: Option<Arc<Db>>,
}

impl Loader {
    #[cfg(not(target_arch = "wasm32"))]
    pub fn start() -> Loader {
        Self::start_with(Arc::new(HttpFetcher))
    }

    /// Browser: a compact copy next to the page if the build shipped one
    /// (`data/ourairports.tsv.gz`), else OurAirports' CSVs (CORS-enabled).
    #[cfg(target_arch = "wasm32")]
    pub fn start() -> Loader {
        let (tx, rx) = channel();
        crate::platform::web::spawn_local(async move {
            let _ = tx.send(web_download().await.map(|(list, bytes)| Arc::new(Db::new(list, bytes))));
        });
        Loader { rx: Some(rx), status: Status::Loading, db: None }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn start_with(fetcher: Arc<dyn Fetcher>) -> Loader {
        let (tx, rx) = channel();
        std::thread::Builder::new()
            .name("airports".into())
            .spawn(move || {
                let t0 = Instant::now();
                match read_cache() {
                    Some((list, bytes, stale)) => {
                        let n = list.len();
                        let _ = tx.send(Ok(Arc::new(Db::new(list, bytes))));
                        eprintln!("airports: {n} airfields from the cache in {:.0} ms", t0.elapsed().as_secs_f32() * 1000.0);
                        if stale {
                            if let Ok((list, bytes)) = download(fetcher.as_ref()) {
                                let _ = tx.send(Ok(Arc::new(Db::new(list, bytes))));
                            }
                        }
                    }
                    None => {
                        let r = download(fetcher.as_ref()).map(|(list, bytes)| Arc::new(Db::new(list, bytes)));
                        let _ = tx.send(r);
                    }
                }
            })
            .ok();
        Loader { rx: Some(rx), status: Status::Loading, db: None }
    }

    /// Take finished work; returns true when a (new) database arrived.
    pub fn poll(&mut self) -> bool {
        let mut got = false;
        while let Some(rx) = &self.rx {
            match rx.try_recv() {
                Ok(Ok(db)) => {
                    self.db = Some(db.clone());
                    self.status = Status::Ready(db);
                    got = true;
                }
                Ok(Err(e)) => {
                    eprintln!("airports: {e}");
                    if self.db.is_none() {
                        self.status = Status::Failed(e);
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => self.rx = None,
            }
        }
        got
    }

    #[cfg(not(target_arch = "wasm32"))]
    /// Block until loaded (headless runs).
    pub fn wait(&mut self, timeout: Duration) -> Option<Arc<Db>> {
        let t = Instant::now();
        while self.db.is_none() && t.elapsed() < timeout && !matches!(self.status, Status::Failed(_)) {
            self.poll();
            std::thread::sleep(Duration::from_millis(10));
        }
        self.db.clone()
    }
}

// ---------------------------------------------------------------- wind / METAR

/// Wind components on a runway heading: (headwind kt, crosswind kt; + = from the right).
pub fn wind_components(rwy_hdg_true: f32, wind_from: f32, wind_kt: f32) -> (f32, f32) {
    let d = (wind_from - rwy_hdg_true).to_radians();
    (wind_kt * d.cos(), wind_kt * d.sin())
}

/// The latest METAR for one airport (its own, else the nearest station
/// within ~60 km), fetched on a thread.
pub struct MetarFetch {
    pub ident: String,
    rx: Option<Receiver<Result<(crate::weather::Metar, f64), String>>>,
    pub result: Option<Result<(crate::weather::Metar, f64), String>>,
}

impl MetarFetch {
    pub fn start(ident: &str, icao: &str, lat: f64, lon: f64) -> MetarFetch {
        let (tx, rx) = channel();
        let id = if icao.is_empty() { ident.to_string() } else { icao.to_string() };
        #[cfg(not(target_arch = "wasm32"))]
        std::thread::Builder::new()
            .name("metar".into())
            .spawn(move || {
                let _ = tx.send(fetch_metar(&id, lat, lon));
            })
            .ok();
        #[cfg(target_arch = "wasm32")]
        crate::platform::web::spawn_local(async move {
            let _ = tx.send(fetch_metar_web(&id, lat, lon).await);
        });
        MetarFetch { ident: ident.to_string(), rx: Some(rx), result: None }
    }
    /// A known METAR, no fetch (headless captures with a given METAR).
    pub fn fixed(ident: &str, raw: &str) -> Option<MetarFetch> {
        let m = crate::weather::Metar::parse(raw)?;
        Some(MetarFetch { ident: ident.to_string(), rx: None, result: Some(Ok((m, 0.0))) })
    }
    pub fn poll(&mut self) -> Option<&Result<(crate::weather::Metar, f64), String>> {
        if let Some(rx) = &self.rx {
            if let Ok(r) = rx.try_recv() {
                self.result = Some(r);
                self.rx = None;
            }
        }
        self.result.as_ref()
    }
    pub fn pending(&self) -> bool {
        self.rx.is_some()
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_metar(id: &str, lat: f64, lon: f64) -> Result<(crate::weather::Metar, f64), String> {
    if crate::platform::offline() {
        return Err("offline mode".into());
    }
    let f = HttpFetcher;
    let parse = |body: &[u8]| -> Vec<(crate::weather::Metar, f64)> {
        let Ok(serde_json::Value::Array(a)) = serde_json::from_slice::<serde_json::Value>(body) else { return Vec::new() };
        let mut v: Vec<(crate::weather::Metar, f64)> = a
            .iter()
            .filter_map(|s| {
                let raw = s.get("rawOb")?.as_str()?;
                let (la, lo) = (s.get("lat")?.as_f64()?, s.get("lon")?.as_f64()?);
                Some((crate::weather::Metar::parse(raw)?, distance_km((lat, lon), (la, lo))))
            })
            .collect();
        v.sort_by(|a, b| a.1.total_cmp(&b.1));
        v
    };
    if id.len() == 4 && id.chars().all(|c| c.is_ascii_alphanumeric()) {
        let url = format!("https://aviationweather.gov/api/data/metar?ids={id}&format=json");
        if let Ok(b) = f.get(&url) {
            if let Some(m) = parse(&b).into_iter().next() {
                return Ok(m);
            }
        }
    }
    let url = format!("https://aviationweather.gov/api/data/metar?bbox={:.3},{:.3},{:.3},{:.3}&format=json", lat - 0.6, lon - 1.0, lat + 0.6, lon + 1.0);
    let b = f.get(&url)?;
    parse(&b).into_iter().find(|m| m.1 < 80.0).ok_or_else(|| "no METAR nearby".to_string())
}

/// Browser: the airport's own METAR, else the nearest within 80 km, from
/// VATSIM's CORS-enabled METAR service (weather::web_metars).
#[cfg(target_arch = "wasm32")]
async fn fetch_metar_web(id: &str, lat: f64, lon: f64) -> Result<(crate::weather::Metar, f64), String> {
    let json = crate::weather::web_metars(lat, lon, 80.0, Some(id)).await?.ok_or("no METAR nearby")?;
    let Ok(serde_json::Value::Array(a)) = serde_json::from_str::<serde_json::Value>(&json) else { return Err("no METAR nearby".into()) };
    let mut v: Vec<(crate::weather::Metar, f64, bool)> = a
        .iter()
        .filter_map(|s| {
            let raw = s.get("rawOb")?.as_str()?;
            let (la, lo) = (s.get("lat")?.as_f64()?, s.get("lon")?.as_f64()?);
            let own = s.get("icaoId").and_then(|x| x.as_str()) == Some(id);
            Some((crate::weather::Metar::parse(raw)?, distance_km((lat, lon), (la, lo)), own))
        })
        .collect();
    // Its own report first, then by distance.
    v.sort_by(|a, b| b.2.cmp(&a.2).then(a.1.total_cmp(&b.1)));
    v.into_iter().find(|m| m.2 || m.1 < 80.0).map(|m| (m.0, m.1)).ok_or_else(|| "no METAR nearby".to_string())
}

// ---------------------------------------------------------------- sun

/// Today's sunrise / sunset (local clock hours in the sun model's zone) at a
/// place, or None for polar day / night.
pub fn sun_times(lat: f64, lon: f64, doy: u32) -> Option<(f32, f32)> {
    let el = |h: f32| crate::atmosphere::sun_direction(lat, lon, doy, h).y;
    let (mut rise, mut set) = (None, None);
    let mut prev = el(0.0);
    let n = 24 * 12;
    for i in 1..=n {
        let h = i as f32 / 12.0;
        let e = el(h);
        let thr = -0.0145; // −0.83°
        if prev < thr && e >= thr && rise.is_none() {
            rise = Some(h - 1.0 / 12.0 * (e - thr) / (e - prev));
        }
        if prev >= thr && e < thr && set.is_none() {
            set = Some(h - 1.0 / 12.0 * (e - thr) / (e - prev));
        }
        prev = e;
    }
    Some((rise?, set?))
}

/// Position of a runway threshold (displaced) in a frame: (threshold, unit direction along the runway).
pub fn threshold(rw: &Runway, from_b: bool) -> (Vec2, Vec2) {
    let (p, q, disp) = if from_b { (rw.b, rw.a, rw.displaced_b) } else { (rw.a, rw.b, rw.displaced_a) };
    let d = (q - p).normalize_or(glam::vec2(0.0, -1.0));
    (p + d * disp, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    const AIRPORTS: &str = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\",\"continent\",\"iso_country\",\"iso_region\",\"municipality\",\"scheduled_service\",\"icao_code\",\"iata_code\",\"gps_code\",\"local_code\",\"home_link\",\"wikipedia_link\",\"keywords\"
2430,\"EGLC\",\"medium_airport\",\"London City Airport\",51.505299,0.055278,19,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLC\",\"LCY\",\"EGLC\",,,,
2434,\"EGLL\",\"large_airport\",\"London Heathrow Airport\",51.4706,-0.461941,83,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLL\",\"LHR\",\"EGLL\",,,,\"LON, Londres\"
1,\"XHEL\",\"heliport\",\"Some Heliport\",51.0,0.0,10,\"EU\",\"GB\",\"GB-ENG\",\"X\",\"no\",,,,,,,
9,\"EGKB\",\"medium_airport\",\"London Biggin Hill Airport\",51.3308,0.0325,598,\"EU\",\"GB\",\"GB-ENG\",\"London, \"\"Bromley\"\"\",\"no\",\"EGKB\",\"BQH\",\"EGKB\",,,,
";
    const RUNWAYS: &str = "\"id\",\"airport_ref\",\"airport_ident\",\"length_ft\",\"width_ft\",\"surface\",\"lighted\",\"closed\",\"le_ident\",\"le_latitude_deg\",\"le_longitude_deg\",\"le_elevation_ft\",\"le_heading_degT\",\"le_displaced_threshold_ft\",\"he_ident\",\"he_latitude_deg\",\"he_longitude_deg\",\"he_elevation_ft\",\"he_heading_degT\",\"he_displaced_threshold_ft\"
239448,2430,\"EGLC\",4948,98,\"CON\",1,0,\"09\",51.505576,0.044333,16,93,,\"27\",51.504894,0.066026,19,273,
239399,2434,\"EGLL\",12799,164,\"ASP\",1,0,\"09L\",51.47749,-0.489439,79,90,1007,\"27R\",51.477681,-0.433227,78,270,
239398,2434,\"EGLL\",12001,164,\"ASP\",1,0,\"09R\",51.46478,-0.486808,75,90,1013,\"27L\",51.464957,-0.434048,77,270,
5,9,\"EGKB\",5928,148,\"ASP\",1,0,\"03\",,,,,,\"21\",,,,,
6,9,\"EGKB\",2697,59,\"GRS\",0,1,\"11\",,,,,,\"29\",,,,,
";

    fn db() -> Db {
        Db::new(parse_csv(AIRPORTS, RUNWAYS), 0)
    }

    #[test]
    fn parses_and_round_trips() {
        let list = parse_csv(AIRPORTS, RUNWAYS);
        assert_eq!(list.len(), 3, "heliport dropped");
        let lhr = list.iter().find(|a| a.ident == "EGLL").unwrap();
        assert_eq!(lhr.runways.len(), 2);
        assert!((lhr.runways[0].length_m - 12799.0 * 0.3048).abs() < 0.5);
        assert!((lhr.runways[0].ends[0].displaced_m - 1007.0 * 0.3048).abs() < 0.5);
        assert_eq!(lhr.runways[0].surface, Surface::Asphalt);
        let bqh = list.iter().find(|a| a.ident == "EGKB").unwrap();
        assert_eq!(bqh.city, "London, \"Bromley\"");
        assert_eq!(bqh.runways.len(), 1, "closed runway dropped");
        let back = decode(&encode(&list)).unwrap();
        assert_eq!(back.len(), list.len());
        for (a, b) in back.iter().zip(&list) {
            assert_eq!(a.ident, b.ident);
            assert_eq!(a.runways.len(), b.runways.len());
            assert!((a.lat - b.lat).abs() < 1e-6);
        }
    }

    #[test]
    fn search_finds_codes_names_and_typos() {
        let d = db();
        let first = |q: &str| d.search(q, 5, |_| true).first().map(|&i| d.airports[i].ident.clone());
        assert_eq!(first("EGLC").as_deref(), Some("EGLC"));
        assert_eq!(first("lhr").as_deref(), Some("EGLL"));
        assert_eq!(first("heathrow").as_deref(), Some("EGLL"));
        assert_eq!(first("heathorw").as_deref(), Some("EGLL"), "typo");
        assert_eq!(first("biggin").as_deref(), Some("EGKB"));
        assert_eq!(first("london city").as_deref(), Some("EGLC"));
        assert!(d.search("zzzz qqq", 5, |_| true).is_empty());
        assert_eq!(d.near(51.5, 0.05, 10.0).first().map(|x| d.airports[x.0].ident.as_str()), Some("EGLC"));
    }

    #[test]
    fn runways_in_world_frame() {
        let d = db();
        let proj = Proj::new(51.5045, 0.03);
        let rws = d.runways_near(&proj, 51.5045, 0.03, 40.0);
        let lc = rws.iter().find(|r| r.airport == "EGLC").unwrap();
        assert!((lc.length() - 1508.0).abs() < 25.0, "length {}", lc.length());
        // 09 points east (a → b along +x).
        assert!(lc.b.x > lc.a.x && (lc.b.y - lc.a.y).abs() < 100.0);
        assert_eq!(lc.source, RunwaySource::OurAirports);
        assert!(lc.width > 29.0 && lc.width < 31.0);
        let bqh = rws.iter().find(|r| r.airport == "EGKB").unwrap();
        assert_eq!(bqh.source, RunwaySource::Estimated);
        let (thr, _) = threshold(rws.iter().find(|r| r.ident_a == "09L").unwrap(), false);
        let a = rws.iter().find(|r| r.ident_a == "09L").unwrap().a;
        assert!((thr.distance(a) - 307.0).abs() < 1.0);
    }

    #[test]
    fn wind_and_surfaces() {
        let (h, x) = wind_components(90.0, 120.0, 20.0);
        assert!((h - 17.32).abs() < 0.01 && (x - 10.0).abs() < 0.01);
        assert_eq!(Surface::parse("ASPH-G"), Surface::Asphalt);
        assert_eq!(Surface::parse("Grass"), Surface::Grass);
        assert_eq!(Surface::parse("TURF-G"), Surface::Grass);
        assert_eq!(Surface::parse("GRVL"), Surface::Gravel);
        assert_eq!(ident_heading("27R"), Some(270.0));
        let (r, s) = sun_times(51.5, 0.0, 172).unwrap();
        assert!((3.5..5.5).contains(&r) && (20.0..22.0).contains(&s), "{r} {s}");
    }
}
