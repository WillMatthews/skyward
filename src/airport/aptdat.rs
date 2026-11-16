//! Real airport layouts from X-Plane `apt.dat` (format 1100 / 1200): the
//! ATC taxi route network (rows 1200 / 1201 / 1202 / 1204), startup / gate
//! locations (1300 / 1301, legacy 15), taxiway signs (20), windsocks (19),
//! tower viewpoints (14), pavement (110 with its 111–116 Bezier rings) and
//! linear features (120: painted lines and taxiway lights) — drawn by
//! `aptgeo.rs` / `layout.rs`.
//!
//! Source: the X-Plane Scenery Gateway's community airports as shipped in
//! FlightGear's `fgdata` (`Airports/apt.dat.ws3.gz`, pinned to a commit
//! below), GNU GPL — the Gateway states "Airport data on the Gateway is
//! licensed under the GNU GPL", the base data © Robin A. Peel (GPL v2 or
//! later). The Gateway's own API is not used (its terms keep it for
//! X-Plane products); FlightGear's copy needs no account or key.
//!
//! Data path (mirrors OurAirports, `airports.rs`): native runs download the
//! file once (≈156 MB gz) on a background thread and split it into small
//! per-airport chunks — still apt.dat text, only the rows used here — under
//! `cache/aptdat/<IDENT>.dat.gz` plus `index.tsv`; later runs read one
//! chunk per airport. The browser build gets the same chunks pre-made by
//! `scripts/web-data.mjs` (`data/aptdat/`; GitLab sends no CORS headers).
//! `SKYWARD_APTDAT=<apt.dat[.gz]>` imports a local copy instead of the
//! download; `SKYWARD_NO_APTDAT=1` turns it all off (goldens, offline tests).

use std::collections::HashMap;
use std::io::{BufRead, Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// fgdata commit "latest airports" (2023-11-03) of `Airports/apt.dat.ws3.gz`.
pub const SOURCE_COMMIT: &str = "228ac3d1685f355638579e266b415cd9c10ca81a";
pub const SOURCE_URL: &str = "https://gitlab.com/flightgear/fgdata/-/raw/228ac3d1685f355638579e266b415cd9c10ca81a/Airports/apt.dat.ws3.gz";
/// Shown in the credits.
pub const CREDIT: &str = "X-Plane Scenery Gateway airports (Robin A. Peel, Laminar Research, Gateway artists) via FlightGear fgdata — GNU GPL";
/// Index header: bump when the chunk selection changes.
const INDEX_VERSION: &str = "skyward-aptdat-2";
/// Rows kept in a chunk (the rest — runways, frequencies, boundaries, light
/// objects — is dropped until something uses it).
const KEEP: [&str; 21] = ["1", "16", "17", "14", "15", "19", "20", "110", "111", "112", "113", "114", "115", "116", "120", "1201", "1202", "1204", "1300", "1301", "1302"];

// ---------------------------------------------------------------- types

#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub lat: f64,
    pub lon: f64,
}

/// A hot zone on a taxi route edge (row 1204): the edge lies in a runway's
/// protected area.
#[derive(Clone, Debug, PartialEq)]
pub struct Hot {
    /// "departure", "arrival" or "ils".
    pub kind: String,
    /// Runway end identifiers ("09L", "27R").
    pub runways: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    /// Node indices (into [`AptAirport::nodes`]).
    pub a: u32,
    pub b: u32,
    /// One-way from `a` to `b`.
    pub oneway: bool,
    /// A runway (not a taxiway) segment.
    pub runway: bool,
    /// ICAO width class letter of a taxiway ('A'..'F'), if given.
    pub class: Option<char>,
    pub name: String,
    pub hot: Vec<Hot>,
}

/// A startup location (row 1300, or a legacy row 15).
#[derive(Clone, Debug, PartialEq)]
pub struct Startup {
    pub lat: f64,
    pub lon: f64,
    /// True heading (deg).
    pub hdg: f32,
    /// "gate", "hangar", "tie_down" or "misc" ("misc" for row 15).
    pub kind: String,
    /// Aircraft it takes ("heavy", "jets", "turboprops", "props", "helos", "fighters", "all").
    pub types: Vec<String>,
    pub name: String,
    /// ICAO width code ('A'..'F', row 1301).
    pub width: Option<char>,
    /// Operation type ("airline", "cargo", "general_aviation", "military", "none").
    pub ops: String,
    pub airlines: Vec<String>,
}

impl Startup {
    /// Largest wingspan (m) it takes: from the ICAO width code, else the types.
    pub fn span(&self) -> f32 {
        match self.width {
            Some('A') => 15.0,
            Some('B') => 24.0,
            Some('C') => 36.0,
            Some('D') => 52.0,
            Some('E') => 65.0,
            Some('F') => 80.0,
            _ => {
                let has = |t: &str| self.types.iter().any(|x| x == t);
                if has("heavy") {
                    65.0
                } else if has("jets") || has("turboprops") || has("all") {
                    36.0
                } else {
                    15.0
                }
            }
        }
    }

    /// For helicopters only.
    pub fn helos_only(&self) -> bool {
        !self.types.is_empty() && self.types.iter().all(|t| t == "helos")
    }
}

/// A taxiway sign (row 20): position, heading (deg), size (1–5), the sign text code.
#[derive(Clone, Debug, PartialEq)]
pub struct Sign {
    pub lat: f64,
    pub lon: f64,
    pub hdg: f32,
    pub size: u8,
    pub text: String,
}

/// A node of a pavement ring or linear feature (rows 111–116).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathNode {
    pub lat: f64,
    pub lon: f64,
    /// Bezier control point after the node (112 / 114 / 116); the one
    /// before it is its mirror image through the node.
    pub ctrl: Option<(f64, f64)>,
    /// Line type of the segment starting here (0 none; 1–9 yellow, 20–25
    /// white, 30–32 red, 40–43 others, +50 with a black border).
    pub line: u8,
    /// Light type of that segment (0 none, else 101–108 − 100).
    pub light: u8,
}

/// A chain of nodes: closed (113 / 114) or open (115 / 116).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Path {
    pub nodes: Vec<PathNode>,
    pub closed: bool,
}

/// A pavement polygon (row 110): surface code, texture heading (deg),
/// boundary first, then its holes.
#[derive(Clone, Debug, PartialEq)]
pub struct Pavement {
    pub surface: u8,
    pub hdg: f32,
    pub rings: Vec<Path>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AptAirport {
    pub ident: String,
    /// The ICAO code (row 1302 `icao_code`) when it differs from the ident.
    pub icao: String,
    pub name: String,
    pub elev_ft: f32,
    /// 1 land airport, 16 seaplane base, 17 heliport.
    pub code: u16,
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    pub starts: Vec<Startup>,
    pub signs: Vec<Sign>,
    pub windsocks: Vec<(f64, f64)>,
    /// Tower viewpoint: position and height above ground (m).
    pub tower: Option<((f64, f64), f32)>,
    /// Pavement polygons in drawing order (later ones on top).
    pub pavements: Vec<Pavement>,
    /// Linear features (row 120): painted lines and lights.
    pub lines: Vec<Path>,
}

impl AptAirport {
    /// Has a taxi route network worth routing on.
    pub fn has_network(&self) -> bool {
        self.edges.iter().filter(|e| !e.runway).count() >= 3
    }
}

// ---------------------------------------------------------------- parser

/// Row code of a line ("" for blank lines).
fn code(line: &str) -> &str {
    line.split_ascii_whitespace().next().unwrap_or("")
}

/// The rest of a line after `n` whitespace-separated fields (names with spaces).
fn rest(line: &str, n: usize) -> String {
    let mut s = line.trim_start();
    for _ in 0..n {
        let Some(i) = s.find(|c: char| c.is_ascii_whitespace()) else { return String::new() };
        s = s[i..].trim_start();
    }
    s.trim().to_string()
}

/// Parse apt.dat text (a whole file or a chunk) into its airports. Unknown
/// and malformed rows are skipped (the file is untrusted input).
pub fn parse(text: &str) -> Vec<AptAirport> {
    let mut out: Vec<AptAirport> = Vec::new();
    let mut ids: HashMap<i64, u32> = HashMap::new();
    let mut pending: Vec<(i64, i64, bool, bool, Option<char>, String)> = Vec::new();
    let mut hot: Vec<Vec<Hot>> = Vec::new();
    // The pavement / linear feature the 111–116 nodes belong to.
    #[derive(PartialEq)]
    enum Feat {
        None,
        Pave,
        Line,
    }
    let mut feat = Feat::None;
    let mut ring: Vec<PathNode> = Vec::new();
    let flush = |ap: &mut AptAirport, ids: &mut HashMap<i64, u32>, pending: &mut Vec<(i64, i64, bool, bool, Option<char>, String)>, hot: &mut Vec<Vec<Hot>>| {
        for (k, (a, b, oneway, runway, class, name)) in pending.drain(..).enumerate() {
            let (Some(&a), Some(&b)) = (ids.get(&a), ids.get(&b)) else { continue };
            if a == b {
                continue;
            }
            ap.edges.push(Edge { a, b, oneway, runway, class, name, hot: hot.get_mut(k).map(std::mem::take).unwrap_or_default() });
        }
        hot.clear();
        ids.clear();
    };
    for line in text.lines() {
        let c = code(line);
        let f: Vec<&str> = line.split_ascii_whitespace().collect();
        let num = |i: usize| f.get(i).and_then(|x| x.parse::<f64>().ok());
        if !matches!(c, "111" | "112" | "113" | "114" | "115" | "116") {
            ring.clear();
            if !c.is_empty() {
                feat = Feat::None;
            }
        }
        match c {
            "1" | "16" | "17" => {
                if let Some(ap) = out.last_mut() {
                    flush(ap, &mut ids, &mut pending, &mut hot);
                }
                if f.len() < 5 {
                    continue;
                }
                out.push(AptAirport { ident: f[4].to_string(), name: rest(line, 5), elev_ft: num(1).unwrap_or(0.0) as f32, code: c.parse().unwrap_or(1), ..Default::default() });
            }
            "99" => {
                if let Some(ap) = out.last_mut() {
                    flush(ap, &mut ids, &mut pending, &mut hot);
                }
            }
            _ => {
                let Some(ap) = out.last_mut() else { continue };
                match c {
                    "1302" if f.get(1) == Some(&"icao_code") => {
                        if let Some(v) = f.get(2).filter(|v| **v != ap.ident) {
                            ap.icao = v.to_string();
                        }
                    }
                    "1201" => {
                        let (Some(lat), Some(lon), Some(id)) = (num(1), num(2), f.get(4).and_then(|x| x.parse::<i64>().ok())) else { continue };
                        if !lat.is_finite() || !lon.is_finite() || lat.abs() > 90.0 || lon.abs() > 180.0 {
                            continue;
                        }
                        ids.insert(id, ap.nodes.len() as u32);
                        ap.nodes.push(Node { lat, lon });
                    }
                    "1202" => {
                        let (Some(a), Some(b)) = (f.get(1).and_then(|x| x.parse::<i64>().ok()), f.get(2).and_then(|x| x.parse::<i64>().ok())) else { continue };
                        let oneway = f.get(3) == Some(&"oneway");
                        let kind = f.get(4).copied().unwrap_or("taxiway");
                        let runway = kind == "runway";
                        let class = kind.strip_prefix("taxiway_").and_then(|x| x.chars().next()).filter(|x| ('A'..='F').contains(x));
                        pending.push((a, b, oneway, runway, class, rest(line, 5)));
                        hot.push(Vec::new());
                    }
                    "1204" => {
                        let (Some(k), Some(r)) = (f.get(1), f.get(2)) else { continue };
                        if let Some(h) = hot.last_mut() {
                            h.push(Hot { kind: k.to_string(), runways: r.split(',').map(|x| x.trim().to_string()).filter(|x| !x.is_empty()).collect() });
                        }
                    }
                    "1300" => {
                        let (Some(lat), Some(lon), Some(hdg)) = (num(1), num(2), num(3)) else { continue };
                        ap.starts.push(Startup {
                            lat,
                            lon,
                            hdg: hdg.rem_euclid(360.0) as f32,
                            kind: f.get(4).unwrap_or(&"misc").to_string(),
                            types: f.get(5).map(|t| t.split('|').map(String::from).collect()).unwrap_or_default(),
                            name: rest(line, 6),
                            width: None,
                            ops: String::new(),
                            airlines: Vec::new(),
                        });
                    }
                    "1301" => {
                        if let Some(s) = ap.starts.last_mut() {
                            s.width = f.get(1).and_then(|x| x.chars().next()).filter(|x| ('A'..='F').contains(x));
                            s.ops = f.get(2).unwrap_or(&"").to_string();
                            s.airlines = f.get(3).map(|a| a.split(',').map(String::from).filter(|x| !x.is_empty()).collect()).unwrap_or_default();
                        }
                    }
                    "15" => {
                        let (Some(lat), Some(lon), Some(hdg)) = (num(1), num(2), num(3)) else { continue };
                        ap.starts.push(Startup {
                            lat,
                            lon,
                            hdg: hdg.rem_euclid(360.0) as f32,
                            kind: "misc".into(),
                            types: Vec::new(),
                            name: rest(line, 4),
                            width: None,
                            ops: String::new(),
                            airlines: Vec::new(),
                        });
                    }
                    "20" => {
                        let (Some(lat), Some(lon), Some(hdg)) = (num(1), num(2), num(3)) else { continue };
                        ap.signs.push(Sign { lat, lon, hdg: hdg as f32, size: num(5).unwrap_or(2.0) as u8, text: rest(line, 6) });
                    }
                    "110" => {
                        let (Some(surf), Some(hdg)) = (num(1), num(3)) else { continue };
                        ap.pavements.push(Pavement { surface: surf.clamp(0.0, 255.0) as u8, hdg: hdg as f32, rings: Vec::new() });
                        feat = Feat::Pave;
                    }
                    "120" => {
                        feat = Feat::Line;
                    }
                    "111" | "112" | "113" | "114" | "115" | "116" if feat != Feat::None => {
                        let bez = matches!(c, "112" | "114" | "116");
                        let (Some(lat), Some(lon)) = (num(1), num(2)) else { continue };
                        if !lat.is_finite() || !lon.is_finite() || lat.abs() > 90.0 || lon.abs() > 180.0 {
                            continue;
                        }
                        let ctrl = if bez { num(3).zip(num(4)).filter(|(a, b)| a.is_finite() && b.is_finite() && a.abs() <= 90.0 && b.abs() <= 180.0) } else { None };
                        let first = if bez { 5 } else { 3 };
                        let (mut line, mut light) = (0u8, 0u8);
                        if !matches!(c, "115" | "116") {
                            for k in first..first + 2 {
                                match f.get(k).and_then(|x| x.parse::<u16>().ok()) {
                                    Some(v @ 1..=99) => line = v as u8,
                                    Some(v @ 101..=199) => light = (v - 100) as u8,
                                    _ => {}
                                }
                            }
                        }
                        ring.push(PathNode { lat, lon, ctrl, line, light });
                        if matches!(c, "113" | "114" | "115" | "116") {
                            let p = Path { nodes: std::mem::take(&mut ring), closed: matches!(c, "113" | "114") };
                            match feat {
                                Feat::Pave => {
                                    if let Some(pv) = ap.pavements.last_mut() {
                                        if p.nodes.len() >= 3 {
                                            pv.rings.push(p);
                                        }
                                    }
                                }
                                _ => {
                                    if p.nodes.len() >= 2 {
                                        ap.lines.push(p);
                                    }
                                }
                            }
                        }
                    }
                    "19" => {
                        if let (Some(lat), Some(lon)) = (num(1), num(2)) {
                            ap.windsocks.push((lat, lon));
                        }
                    }
                    "14" => {
                        if let (Some(lat), Some(lon), Some(h)) = (num(1), num(2), num(3)) {
                            ap.tower = Some(((lat, lon), (h as f32 * 0.3048).max(0.0)));
                        }
                    }
                    _ => {}
                }
            }
        }
    }
    if let Some(ap) = out.last_mut() {
        flush(ap, &mut ids, &mut pending, &mut hot);
    }
    for ap in &mut out {
        ap.pavements.retain(|p| !p.rings.is_empty());
    }
    out
}

// ---------------------------------------------------------------- splitting into chunks

/// Chunk file stem for an ident (uppercase, safe characters only).
pub fn stem(ident: &str) -> String {
    ident.chars().filter(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == '_').map(|c| c.to_ascii_uppercase()).take(16).collect()
}

/// One airport's chunk: (ident, ICAO code if different, apt.dat text of the kept rows).
pub struct Chunk {
    pub ident: String,
    pub icao: String,
    pub text: String,
    /// Has pavement polygons (the map's own pavement gives way: `tile.rs`).
    pub paved: bool,
}

/// Split a whole apt.dat (decompressed, read line by line — the file is
/// ≈630 MB) into per-airport chunks of the rows used here, calling `sink`
/// for each airport that has a taxi network or at least three startup
/// locations (fewer make no stands: `layout.rs`).
pub fn split(r: impl BufRead, sink: &mut dyn FnMut(Chunk)) -> std::io::Result<usize> {
    let mut cur: Option<Chunk> = None;
    let (mut net, mut starts) = (false, 0u32);
    let mut n = 0;
    let mut emit = |c: Option<Chunk>, useful: bool, n: &mut usize| {
        if let Some(c) = c.filter(|_| useful) {
            if !stem(&c.ident).is_empty() {
                *n += 1;
                sink(c);
            }
        }
    };
    for line in r.split(b'\n') {
        let line = line?;
        let line = String::from_utf8_lossy(&line);
        let line = line.trim_end();
        let c = code(line);
        if c == "1" || c == "16" || c == "17" {
            emit(cur.take(), net || starts >= 3, &mut n);
            (net, starts) = (false, 0);
            let ident = line.split_ascii_whitespace().nth(4).unwrap_or("").to_string();
            cur = Some(Chunk { ident, icao: String::new(), text: String::new(), paved: false });
        } else if c == "99" {
            emit(cur.take(), net || starts >= 3, &mut n);
            (net, starts) = (false, 0);
            continue;
        }
        let Some(ch) = cur.as_mut() else { continue };
        if !KEEP.contains(&c) {
            continue;
        }
        if c == "1302" {
            let f: Vec<&str> = line.split_ascii_whitespace().collect();
            if f.get(1) == Some(&"icao_code") {
                if let Some(v) = f.get(2).filter(|v| **v != ch.ident) {
                    ch.icao = v.to_string();
                    ch.text.push_str(line);
                    ch.text.push('\n');
                }
            }
            continue;
        }
        net |= c == "1202";
        ch.paved |= c == "110";
        starts += (c == "1300" || c == "15") as u32;
        if matches!(c, "111" | "112" | "113" | "114" | "115" | "116") {
            compact_node(line, &mut ch.text);
        } else {
            ch.text.push_str(line);
        }
        ch.text.push('\n');
    }
    emit(cur.take(), net || starts >= 3, &mut n);
    Ok(n)
}

/// A pavement / line node row with its coordinates cut to 7 decimals
/// (≈1 cm) and single spaces: the chunks shrink by about a fifth.
fn compact_node(line: &str, out: &mut String) {
    let f: Vec<&str> = line.split_ascii_whitespace().collect();
    let coords = if matches!(f[0], "112" | "114" | "116") { 4 } else { 2 };
    for (i, x) in f.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        match x.parse::<f64>() {
            Ok(v) if i >= 1 && i <= coords && v.is_finite() => {
                let t = format!("{v:.7}");
                let t = t.trim_end_matches('0').trim_end_matches('.');
                out.push_str(if t == "-0" { "0" } else { t });
            }
            _ => out.push_str(x),
        }
    }
}

fn gzip(s: &[u8]) -> std::io::Result<Vec<u8>> {
    let mut e = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    e.write_all(s)?;
    e.finish()
}

fn gunzip(b: &[u8]) -> Option<String> {
    let mut s = String::new();
    flate2::read::GzDecoder::new(b).read_to_string(&mut s).ok()?;
    Some(s)
}

/// Index file text: version line, then "IDENT\tICAO\tP" per chunk (P: has pavement).
fn index_text(entries: &[(String, String, bool)]) -> String {
    let mut s = format!("{INDEX_VERSION}\t{SOURCE_COMMIT}\n");
    for (i, c, p) in entries {
        s.push_str(&format!("{i}\t{c}\t{}\n", if *p { "P" } else { "" }));
    }
    s
}

/// The chunk index: OurAirports ident or ICAO code → chunk stem; the stems
/// of the airports with pavement.
#[derive(Default)]
struct Index {
    stems: HashMap<String, String>,
    paved: std::collections::HashSet<String>,
}

impl Index {
    fn get(&self, key: &str) -> Option<&String> {
        self.stems.get(key)
    }
}

fn parse_index(text: &str) -> Option<Index> {
    let mut lines = text.lines();
    if !lines.next()?.starts_with(INDEX_VERSION) {
        return None;
    }
    let mut ix = Index::default();
    for l in lines {
        let mut f = l.split('\t');
        let ident = f.next().unwrap_or("").trim();
        if ident.is_empty() {
            continue;
        }
        let st = stem(ident);
        if let Some(icao) = f.next().map(str::trim).filter(|x| !x.is_empty()) {
            ix.stems.entry(icao.to_ascii_uppercase()).or_insert_with(|| st.clone());
        }
        if f.next().is_some_and(|p| p.trim() == "P") {
            ix.paved.insert(st.clone());
        }
        ix.stems.insert(ident.to_ascii_uppercase(), st);
    }
    Some(ix)
}

/// The compact data set as one archive (the browser build step,
/// `skyward_compact_aptdat`): records of [name len u16][name][data len u32][data]
/// (little-endian) — `index.tsv` first, then `<STEM>.dat.gz` per airport.
pub fn compact_archive(gz: &[u8]) -> Result<Vec<u8>, String> {
    let r = std::io::BufReader::new(flate2::read::MultiGzDecoder::new(gz));
    let mut out = Vec::new();
    let mut entries = Vec::new();
    let mut files: Vec<(String, Vec<u8>)> = Vec::new();
    let mut err = None;
    split(r, &mut |c| match gzip(c.text.as_bytes()) {
        Ok(b) => {
            files.push((format!("{}.dat.gz", stem(&c.ident)), b));
            entries.push((c.ident, c.icao, c.paved));
        }
        Err(e) => err = Some(e.to_string()),
    })
    .map_err(|e| e.to_string())?;
    if let Some(e) = err {
        return Err(e);
    }
    if entries.len() < 100 {
        return Err(format!("only {} airports with layouts", entries.len()));
    }
    let mut put = |name: &str, data: &[u8]| {
        out.extend_from_slice(&(name.len() as u16).to_le_bytes());
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(data);
    };
    put("index.tsv", index_text(&entries).as_bytes());
    for (n, d) in &files {
        put(n, d);
    }
    Ok(out)
}

// ---------------------------------------------------------------- store

/// A lookup's state.
pub enum Lookup {
    /// Being fetched (the browser build): ask again after [`generation`] moves.
    Pending,
    Done(Option<Arc<AptAirport>>),
}

#[derive(Default)]
struct Store {
    /// ident / ICAO → chunk stem; None until loaded.
    index: Option<Arc<Index>>,
    /// Started loading the index (or the download / import).
    started: bool,
    parsed: HashMap<String, Option<Arc<AptAirport>>>,
    #[allow(dead_code)]
    fetching: std::collections::HashSet<String>,
}

static STORE: Mutex<Option<Store>> = Mutex::new(None);
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// Bumped whenever more data became available (the index, a chunk).
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Relaxed)
}

pub fn disabled() -> bool {
    std::env::var_os("SKYWARD_NO_APTDAT").is_some()
}

fn with_store<R>(f: impl FnOnce(&mut Store) -> R) -> R {
    let mut g = STORE.lock().unwrap_or_else(|e| e.into_inner());
    f(g.get_or_insert_with(Store::default))
}

/// Parse one chunk into its airport (the first one in it).
fn from_chunk(text: &str) -> Option<Arc<AptAirport>> {
    parse(text).into_iter().next().map(Arc::new)
}

/// Does the airport's apt.dat layout have pavement? None while the index
/// is not loaded (the map's pavement is drawn then). Cheap: index only.
pub fn paved(ident: &str) -> Option<bool> {
    if disabled() {
        return Some(false);
    }
    #[cfg(not(target_arch = "wasm32"))]
    imp::load_index();
    let key = ident.to_ascii_uppercase();
    with_store(|s| s.index.as_ref().map(|ix| ix.get(&key).is_some_and(|st| ix.paved.contains(st))))
}

/// The apt.dat layout of an airport (OurAirports ident), if the data has one.
pub fn lookup(ident: &str) -> Lookup {
    if disabled() {
        return Lookup::Done(None);
    }
    let key = ident.to_ascii_uppercase();
    if let Some(p) = with_store(|s| s.parsed.get(&key).cloned()) {
        return Lookup::Done(p);
    }
    imp::lookup(&key)
}

#[cfg(not(target_arch = "wasm32"))]
mod imp {
    use super::*;
    use std::path::PathBuf;

    pub fn dir() -> PathBuf {
        PathBuf::from("cache").join("aptdat")
    }

    /// Read the index (once), or start the download / import.
    pub fn load_index() -> Option<Arc<Index>> {
        with_store(|s| {
            if s.index.is_none() && !s.started {
                s.started = true;
                match std::fs::read_to_string(dir().join("index.tsv")).ok().and_then(|t| parse_index(&t)) {
                    Some(ix) => s.index = Some(Arc::new(ix)),
                    None => start_import(),
                }
            }
            s.index.clone()
        })
    }

    pub fn lookup(key: &str) -> Lookup {
        let index = load_index();
        let Some(index) = index else { return Lookup::Done(None) };
        let ap = index.get(key).and_then(|st| std::fs::read(dir().join(format!("{st}.dat.gz"))).ok()).and_then(|b| gunzip(&b)).and_then(|t| from_chunk(&t));
        with_store(|s| s.parsed.insert(key.to_string(), ap.clone()));
        Lookup::Done(ap)
    }

    /// Download (or import `SKYWARD_APTDAT`) and split, on a background thread.
    fn start_import() {
        let local = std::env::var_os("SKYWARD_APTDAT").map(PathBuf::from);
        let fail = dir().with_extension("failed");
        if cfg!(test) {
            return;
        }
        if local.is_none() {
            if crate::platform::offline() {
                return;
            }
            let recent = std::fs::metadata(&fail).ok().and_then(|m| m.modified().ok()).and_then(|t| t.elapsed().ok()).is_some_and(|e| e.as_secs() < 3600);
            if recent {
                return;
            }
        }
        let _ = std::thread::Builder::new().name("aptdat".into()).spawn(move || {
            let t0 = web_time::Instant::now();
            let r = import(local.as_deref());
            match r {
                Ok((n, bytes)) => {
                    let _ = std::fs::remove_file(&fail);
                    eprintln!("aptdat: {n} airport layouts ({:.1} MB of chunks) in {:.0} s", bytes as f64 / 1e6, t0.elapsed().as_secs_f32());
                    if let Some(ix) = std::fs::read_to_string(dir().join("index.tsv")).ok().and_then(|t| parse_index(&t)) {
                        with_store(|s| {
                            s.index = Some(Arc::new(ix));
                            s.parsed.retain(|_, v| v.is_some());
                        });
                        GENERATION.fetch_add(1, Ordering::Relaxed);
                    }
                }
                Err(e) => {
                    eprintln!("aptdat: {e} (OSM / procedural taxiways)");
                    if local.is_none() {
                        let _ = std::fs::create_dir_all("cache");
                        let _ = std::fs::write(&fail, b"");
                    }
                }
            }
        });
    }

    /// Import a local apt.dat (plain or .gz), else download, into `cache/aptdat/`
    /// (blocking): (airports, bytes of chunks).
    pub fn import(local: Option<&std::path::Path>) -> Result<(usize, u64), String> {
        match local {
            Some(p) => std::fs::File::open(p).map_err(|e| format!("{}: {e}", p.display())).and_then(|f| {
                let gz = p.extension().is_some_and(|x| x == "gz");
                let r: Box<dyn Read> = if gz { Box::new(flate2::read::MultiGzDecoder::new(f)) } else { Box::new(f) };
                write_chunks(std::io::BufReader::with_capacity(1 << 20, r))
            }),
            None => download(),
        }
    }

    fn download() -> Result<(usize, u64), String> {
        eprintln!("aptdat: downloading airport layouts (≈156 MB, once) from FlightGear fgdata");
        let ag = ureq::AgentBuilder::new()
            .timeout_connect(std::time::Duration::from_secs(10))
            .timeout_read(std::time::Duration::from_secs(60))
            .user_agent("skyward-flightsim/0.3")
            .build();
        let r = ag.get(SOURCE_URL).call().map_err(|e| e.to_string())?;
        let gz = flate2::read::MultiGzDecoder::new(r.into_reader().take(400 << 20));
        write_chunks(std::io::BufReader::with_capacity(1 << 20, gz))
    }

    /// Split into a fresh directory, then swap it in.
    fn write_chunks(r: impl BufRead) -> Result<(usize, u64), String> {
        let tmp = dir().with_extension(format!("tmp{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).map_err(|e| e.to_string())?;
        let mut entries = Vec::new();
        let mut bytes = 0u64;
        let mut err: Option<String> = None;
        let n = split(r, &mut |c| {
            if err.is_some() {
                return;
            }
            match gzip(c.text.as_bytes()).and_then(|b| {
                bytes += b.len() as u64;
                std::fs::write(tmp.join(format!("{}.dat.gz", stem(&c.ident))), b)
            }) {
                Ok(()) => entries.push((c.ident, c.icao, c.paved)),
                Err(e) => err = Some(e.to_string()),
            }
        })
        .map_err(|e| e.to_string())?;
        if let Some(e) = err {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(e);
        }
        if n < 100 {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(format!("only {n} airports with layouts"));
        }
        std::fs::write(tmp.join("index.tsv"), index_text(&entries)).map_err(|e| e.to_string())?;
        let _ = std::fs::remove_dir_all(dir());
        std::fs::rename(&tmp, dir()).map_err(|e| e.to_string())?;
        Ok((n, bytes))
    }
}

#[cfg(target_arch = "wasm32")]
mod imp {
    use super::*;
    use crate::platform::web;
    use web_time::Duration;

    pub fn lookup(key: &str) -> Lookup {
        let (index, start) = with_store(|s| {
            let start = !s.started;
            s.started = true;
            (s.index.clone(), start)
        });
        if start {
            web::spawn_local(async {
                let ix = async {
                    web::load_manifest().await.ok()?;
                    if !web::has_extra("data/aptdat/index.tsv") {
                        return None;
                    }
                    let b = web::get("data/aptdat/index.tsv", Duration::from_secs(60)).await.ok()?;
                    parse_index(&String::from_utf8_lossy(&b))
                }
                .await;
                with_store(|s| s.index = Some(Arc::new(ix.unwrap_or_default())));
                GENERATION.fetch_add(1, Ordering::Relaxed);
            });
        }
        let Some(index) = index else { return Lookup::Pending };
        let Some(st) = index.get(key).cloned() else {
            with_store(|s| s.parsed.insert(key.to_string(), None));
            return Lookup::Done(None);
        };
        let go = with_store(|s| s.fetching.insert(key.to_string()));
        if go {
            let key = key.to_string();
            web::spawn_local(async move {
                let ap = web::get(&format!("data/aptdat/{st}.dat.gz"), Duration::from_secs(60)).await.ok().and_then(|b| gunzip(&b)).and_then(|t| from_chunk(&t));
                with_store(|s| {
                    s.fetching.remove(&key);
                    s.parsed.insert(key, ap);
                });
                GENERATION.fetch_add(1, Ordering::Relaxed);
            });
        }
        Lookup::Pending
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A trimmed piece of London City (EGLC) from the Gateway data, plus a
    /// few hand-made rows for the edge cases.
    pub const EGLC: &str = include_str!("../../tests/data/aptdat_eglc.dat");

    #[test]
    fn parses_network_starts_and_signs() {
        let aps = parse(EGLC);
        assert_eq!(aps.len(), 1);
        let a = &aps[0];
        assert_eq!(a.ident, "EGLC");
        assert_eq!(a.name, "London City");
        assert!(a.has_network());
        assert!(a.nodes.len() >= 50, "{} nodes", a.nodes.len());
        assert!(a.edges.len() >= 40, "{} edges", a.edges.len());
        assert!(a.edges.iter().any(|e| e.runway && e.name == "09/27"));
        let d = a.edges.iter().find(|e| e.name == "D").expect("taxiway D");
        assert_eq!(d.class, Some('C'));
        assert!(!d.oneway);
        // Hot zones follow their edge.
        assert!(a.edges.iter().any(|e| e.hot.iter().any(|h| h.kind == "departure" && h.runways.contains(&"27".to_string()))));
        // Stands with their 1301 width code.
        let s16 = a.starts.iter().find(|s| s.name == "Stand 16").expect("stand 16");
        assert_eq!(s16.kind, "gate");
        assert_eq!(s16.width, Some('B'));
        assert_eq!(s16.ops, "airline");
        assert!((s16.hdg - 328.4).abs() < 0.01, "{}", s16.hdg);
        assert_eq!(s16.span(), 24.0);
        assert!(a.signs.iter().any(|s| s.text.contains("09-27")));
        assert!(!a.windsocks.is_empty());
        assert!(a.tower.is_some());
    }

    /// An OurAirports-style airport with these runways (ident pairs, end lat / lon, width).
    pub fn airport(ident: &str, lat: f64, lon: f64, rws: &[(&str, &str, (f64, f64), (f64, f64), f32)]) -> crate::airports::Airport {
        use crate::airports::{Airport, End, Kind, Rwy, Surface};
        let end = |id: &str, p: (f64, f64)| End { ident: id.into(), lat: Some(p.0), lon: Some(p.1), hdg_true: None, displaced_m: 0.0 };
        let proj = crate::osm::Proj::new(lat, lon);
        Airport {
            ident: ident.into(),
            kind: Kind::Large,
            name: ident.into(),
            lat,
            lon,
            elev_ft: 0.0,
            country: String::new(),
            region: String::new(),
            city: String::new(),
            scheduled: true,
            icao: ident.into(),
            iata: String::new(),
            runways: rws
                .iter()
                .map(|r| Rwy {
                    length_m: proj.to_world(r.2.0, r.2.1).distance(proj.to_world(r.3.0, r.3.1)),
                    width_m: r.4,
                    surface: Surface::Asphalt,
                    lighted: true,
                    ends: [end(r.0, r.2), end(r.1, r.3)],
                })
                .collect(),
        }
    }

    pub fn egll() -> (crate::airports::Airport, AptAirport) {
        let text = gunzip(include_bytes!("../../tests/data/aptdat_egll.dat.gz")).unwrap();
        let ap = airport(
            "EGLL",
            51.4706,
            -0.461941,
            &[("09L", "27R", (51.4775, -0.484989), (51.477649, -0.433258), 50.0), ("09R", "27L", (51.464949, -0.482271), (51.46508, -0.434122), 50.0)],
        );
        (ap, parse(&text).remove(0))
    }

    fn eglc() -> (crate::airports::Airport, AptAirport) {
        let ap = airport("EGLC", 51.505278, 0.055278, &[("09", "27", (51.50557055, 0.04440189), (51.5048861, 0.06600568), 33.5)]);
        (ap, parse(EGLC).remove(0))
    }

    /// The departure route from a stand to the best holding point of a
    /// runway end, as the ground guide plans it (layout → graph → holds).
    fn departure(ap: &crate::airports::Airport, apt: &AptAirport, stand: &str, rwy_end: &str) -> (crate::atc::taxi::HoldPt, crate::atc::taxi::Route) {
        use crate::atc::taxi;
        let l = super::super::layout::build(ap, &[], None, Some(apt));
        assert!(l.net.is_some(), "no network");
        let proj = l.frame;
        let g = l.taxi_graph(&proj);
        let st = l.stands.iter().find(|s| s.name == stand).unwrap_or_else(|| panic!("stand {stand} in {:?}", l.stands.iter().map(|s| &s.name).collect::<Vec<_>>()));
        let rws: Vec<crate::scene::Runway> = ap.runways.iter().filter_map(|r| crate::airports::to_world(ap, r, &proj)).collect();
        let rw = rws.iter().find(|r| r.ident_a == rwy_end || r.ident_b == rwy_end).unwrap();
        let (thr, dir) = crate::airports::threshold(rw, rw.ident_b == rwy_end);
        let boxes: Vec<taxi::RwyBox> = rws.iter().map(|r| taxi::RwyBox { a: r.a, b: r.b, width: r.width, name: r.name.clone() }).collect();
        let holds: Vec<(glam::Vec2, String, String)> = l.holds.iter().map(|h| (h.at, h.taxiway.clone(), h.runway.clone())).collect();
        let mut best: Option<(f32, taxi::HoldPt, taxi::Route)> = None;
        for (h, along) in taxi::departure_holds(&holds, &rw.name, thr, dir) {
            let Some(r) = g.route(st.stop, h.at, 150.0, &boxes) else { continue };
            let score = r.length + 1.5 * along + if r.crosses.contains(&rw.name) { 5000.0 } else { 0.0 };
            if best.as_ref().is_none_or(|b| score < b.0) {
                best = Some((score, h, r));
            }
        }
        let (_, h, r) = best.unwrap_or_else(|| panic!("no route to a {rwy_end} hold; holds {:?}", l.holds.iter().map(|h| (&h.taxiway, &h.runway)).collect::<Vec<_>>()));
        (h, r)
    }

    #[test]
    fn heathrow_stands_holds_and_a_sensible_taxi_route() {
        let (ap, apt) = egll();
        let l = super::super::layout::build(&ap, &[], None, Some(&apt));
        // Real stands (MARS alternatives folded), holds from the hot zones on both runways.
        assert!(l.stands.len() > 150, "{} stands", l.stands.len());
        assert!(l.stands.iter().any(|s| s.name == "211"));
        for rw in ["09L/27R", "09R/27L"] {
            assert!(l.holds.iter().filter(|h| h.runway == rw).count() >= 8, "{rw}: {:?}", l.holds.iter().map(|h| &h.runway).collect::<Vec<_>>());
        }
        assert!(!l.parked.is_empty());
        // Terminal 5 (stand 501, west of the central area) to runway 27R
        // (the northern runway, threshold at the east end): along the northern
        // parallel taxiway, not across the departure runway, no detour.
        let (h, r) = departure(&ap, &apt, "501", "27R");
        let st = l.stands.iter().find(|s| s.name == "501").unwrap();
        let crow = st.stop.distance(h.at);
        eprintln!("EGLL 501 → 27R: hold {} after {:.0} m (crow {crow:.0}) via {:?} crossing {:?}", h.taxiway, r.length, r.via, r.crosses);
        assert!(r.length > crow && r.length < crow * 1.6 + 800.0, "{:.0} m vs {crow:.0}", r.length);
        assert!(!r.crosses.contains(&"09L/27R".to_string()), "{:?}", r.crosses);
        assert!(!r.via.is_empty() && r.via.len() <= 6, "{:?}", r.via);
        assert!(r.via.iter().all(|v| !v.is_empty() && v.len() <= 4), "{:?}", r.via);
        assert!(r.via.iter().any(|v| v == "A" || v == "B"), "the inner / outer taxiway: {:?}", r.via);
    }

    #[test]
    fn london_city_departure_09() {
        let (ap, apt) = eglc();
        let l = super::super::layout::build(&ap, &[], None, Some(&apt));
        assert_eq!(l.stands.len(), apt.starts.iter().filter(|s| !s.helos_only()).count().min(l.stands.len()));
        assert!(l.stands.len() >= 10, "{}", l.stands.len());
        let (h, r) = departure(&ap, &apt, "Stand 16", "09");
        eprintln!("EGLC Stand 16 → 09: hold {} after {:.0} m via {:?} crossing {:?}", h.taxiway, r.length, r.via, r.crosses);
        assert!(r.length < 1200.0, "{:.0}", r.length);
        assert!(r.crosses.is_empty(), "{:?}", r.crosses);
    }

    #[test]
    fn one_way_links_are_one_way() {
        use crate::atc::taxi::{Graph, Net, NetEdge};
        use glam::vec2;
        // A square: a one-way link 0 → 1 and the long way round 1 → 2 → 3 → 0.
        let nodes = vec![vec2(0.0, 0.0), vec2(100.0, 0.0), vec2(100.0, 100.0), vec2(0.0, 100.0)];
        let e = |a, b, oneway| NetEdge { a, b, oneway, runway: false, name: "A".into() };
        let g = Graph::from_net(&Net { nodes: nodes.clone(), edges: vec![e(0, 1, true), e(1, 2, false), e(2, 3, false), e(3, 0, false)] });
        let fwd = g.route(nodes[0], nodes[1], 1.0, &[]).unwrap();
        let back = g.route(nodes[1], nodes[0], 1.0, &[]).unwrap();
        assert!((fwd.length - 100.0).abs() < 0.1, "{}", fwd.length);
        assert!((back.length - 300.0).abs() < 0.1, "{}", back.length);
        assert_eq!(fwd.via, vec!["A"]);
    }

    /// Dev tool: build `cache/aptdat/` from a local file —
    /// `SKYWARD_APTDAT=…/apt.dat.ws3.gz cargo test --release aptdat_import -- --ignored`.
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn aptdat_import() {
        let p = std::env::var_os("SKYWARD_APTDAT").expect("SKYWARD_APTDAT=<apt.dat[.gz]>");
        let t0 = std::time::Instant::now();
        let (n, bytes) = imp::import(Some(std::path::Path::new(&p))).unwrap();
        eprintln!("{n} airports, {:.1} MB of chunks in {:.1} s", bytes as f64 / 1e6, t0.elapsed().as_secs_f32());
        assert!(n > 1000);
    }

    #[test]
    fn malformed_rows_are_skipped() {
        let t = "I\n1100 Version\n\n1 10 0 0 XTST Test field\n1201 51.0 0.0 both 1 a\n1201 nope 0.0 both 2 b\n1201 51.001 0.001 both 3 c\n\
                 1202 1 3 oneway taxiway_B A\n1204 departure 09,27\n1202 1 2 twoway taxiway B\n1202 3 3 twoway taxiway C\n1301 C airline\n\
                 1300 51.0 0.0 90.0 tie_down props GA 1\n1301 X general_aviation\n15 51.0 0.0 450 Old ramp\n99\n";
        let a = &parse(t)[0];
        assert_eq!(a.nodes.len(), 2);
        assert_eq!(a.edges.len(), 1, "{:?}", a.edges);
        assert!(a.edges[0].oneway);
        assert_eq!(a.edges[0].hot[0].runways, vec!["09", "27"]);
        assert_eq!(a.starts.len(), 2);
        assert_eq!(a.starts[0].name, "GA 1");
        assert_eq!(a.starts[0].width, None);
        assert_eq!(a.starts[0].span(), 15.0);
        assert_eq!(a.starts[1].hdg, 90.0);
    }

    #[test]
    fn split_keeps_only_airports_with_layouts() {
        let t = format!(
            "I\n1200 Version\n\n1 5 0 0 NONE Nothing here\n100 30 1 0 0.25 0 0 0 09 51 0 0 0 0 0 0 0 27 51 0.01 0 0 0 0 0 0\n{EGLC}\n1 5 0 0 XGA GA only\n1302 icao_code EGXX\n15 51 0 90 Ramp\n15 51 0.001 90 Ramp 2\n15 51 0.002 90 Ramp 3\n1 5 0 0 XONE One ramp\n15 51 0 90 Ramp\n110 1 0.25 90 pavement\n99\n"
        );
        let mut got = Vec::new();
        let n = split(std::io::Cursor::new(t.as_bytes()), &mut |c| got.push(c)).unwrap();
        assert_eq!(n, 2);
        assert_eq!(got[0].ident, "EGLC");
        assert!(got[0].text.contains("1202 "));
        assert!(got[0].text.contains("\n111 51.5043581 0.065751 54\n"), "pavement rows dropped / not compacted: {}", &got[0].text[got[0].text.len() - 200..]);
        assert!(got[0].paved);
        assert_eq!(got[1].ident, "XGA");
        assert_eq!(got[1].icao, "EGXX");
        assert!(!got[1].text.contains("110 ") && !got[1].paved);
        assert!(!got[0].text.contains("\n100 "), "runway rows kept");
        // The chunk parses back to the same airport.
        assert_eq!(parse(&got[0].text)[0].edges, parse(EGLC)[0].edges);
        let ix = parse_index(&index_text(&[("XGA".into(), "EGXX".into(), false), ("EGLC".into(), String::new(), true)])).unwrap();
        assert_eq!(ix.get("EGXX").map(String::as_str), Some("XGA"));
        assert_eq!(ix.get("XGA").map(String::as_str), Some("XGA"));
        assert!(ix.paved.contains("EGLC") && !ix.paved.contains("XGA"));
        assert_eq!(stem("k/../x"), "KX");
    }
}
