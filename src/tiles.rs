//! Vector-tile download + disk cache + background worker pool.
//!
//! Source: OpenFreeMap (OpenMapTiles schema, z14 max), no API key.
//! TileJSON is fetched once per run (cached copy used when offline); tiles are
//! cached forever under `cache/tiles/{z}/{x}/{y}.pbf`.
//!
//! Workers fetch → decode → extract → build meshes/collision/lights entirely
//! off the main thread; the main thread only receives finished `TileBuild`s.

use crate::osm::{Proj, TileId};
use crate::scene::{self, TileBuild};
use std::collections::HashSet;
use std::io::Read;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

const TILEJSON_URL: &str = "https://tiles.openfreemap.org/planet";
const USER_AGENT: &str = "skyward-flightsim/0.2";
/// Disk cache budget; oldest tiles are pruned at startup beyond this.
const CACHE_LIMIT: u64 = 1_500_000_000;

fn cache_root() -> PathBuf {
    PathBuf::from("cache").join("tiles")
}

fn tile_path(id: TileId) -> PathBuf {
    cache_root().join(id.z.to_string()).join(id.x.to_string()).join(format!("{}.pbf", id.y))
}

fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(8))
        .timeout(Duration::from_secs(25))
        .user_agent(USER_AGENT)
        .build()
}

/// Resolve the tile URL template from TileJSON (falls back to the cached copy).
fn tile_template(agent: &ureq::Agent) -> Result<String, String> {
    let path = cache_root().join("tilejson.json");
    let parse = |text: &str| -> Option<String> {
        let v: serde_json::Value = serde_json::from_str(text).ok()?;
        Some(v.get("tiles")?.get(0)?.as_str()?.to_string())
    };
    let net = agent
        .get(TILEJSON_URL)
        .call()
        .map_err(|e| e.to_string())
        .and_then(|r| r.into_string().map_err(|e| e.to_string()));
    match net {
        Ok(text) => {
            if let Some(t) = parse(&text) {
                let _ = std::fs::create_dir_all(cache_root());
                let _ = std::fs::write(&path, &text);
                return Ok(t);
            }
            Err("TileJSON has no tile URL".into())
        }
        Err(e) => {
            let cached = std::fs::read_to_string(&path).ok().and_then(|t| parse(&t));
            cached.ok_or(format!("TileJSON unavailable ({e}) and no cached copy"))
        }
    }
}

fn gunzip_if_needed(b: Vec<u8>) -> Result<Vec<u8>, String> {
    if b.len() > 2 && b[0] == 0x1f && b[1] == 0x8b {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(&b[..]).read_to_end(&mut out).map_err(|e| format!("gunzip: {e}"))?;
        Ok(out)
    } else {
        Ok(b)
    }
}

pub enum Source {
    Cache,
    Network,
}

/// Fetch one tile: disk cache first, then the network.
fn fetch(agent: &ureq::Agent, template: Option<&str>, id: TileId) -> Result<(Vec<u8>, Source), String> {
    let path = tile_path(id);
    if let Ok(b) = std::fs::read(&path) {
        return Ok((gunzip_if_needed(b)?, Source::Cache));
    }
    let t = template.ok_or("offline: no tile URL and tile not cached")?;
    let url = t.replace("{z}", &id.z.to_string()).replace("{x}", &id.x.to_string()).replace("{y}", &id.y.to_string());
    let bytes = match agent.get(&url).call() {
        Ok(r) => {
            let mut b = Vec::with_capacity(160_000);
            r.into_reader().take(16 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
            gunzip_if_needed(b)?
        }
        // No data here (open ocean etc.): an empty tile is a valid answer.
        Err(ureq::Error::Status(204 | 404, _)) => Vec::new(),
        Err(e) => return Err(e.to_string()),
    };
    if let Some(dir) = path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let tmp = path.with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, &path);
    }
    Ok((bytes, Source::Network))
}

/// Delete the oldest cached tiles if the cache exceeds its budget.
fn prune_cache() {
    let mut files = Vec::new();
    let mut total = 0u64;
    let mut stack = vec![cache_root()];
    while let Some(d) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        for e in rd.flatten() {
            let Ok(md) = e.metadata() else { continue };
            if md.is_dir() {
                stack.push(e.path());
            } else if e.path().extension().is_some_and(|x| x == "pbf") {
                total += md.len();
                files.push((md.modified().unwrap_or(std::time::SystemTime::UNIX_EPOCH), md.len(), e.path()));
            }
        }
    }
    if total <= CACHE_LIMIT {
        return;
    }
    files.sort();
    for (_, len, p) in files {
        if total <= CACHE_LIMIT * 8 / 10 {
            break;
        }
        if std::fs::remove_file(&p).is_ok() {
            total -= len;
        }
    }
}

// ---------------------------------------------------------------- pool

pub struct TileResult {
    pub id: TileId,
    pub build: Result<TileBuild, String>,
    /// Vector moving-map geometry (tile frame), built alongside.
    pub map: Option<std::sync::Arc<crate::ui::mapdata::MapVectors>>,
    pub fetch_ms: f32,
    pub build_ms: f32,
    pub from_cache: bool,
    pub bytes: usize,
}

struct QState {
    /// Pending jobs, best LAST (popped from the end).
    jobs: Vec<TileId>,
    active: HashSet<TileId>,
    quit: bool,
}

struct Shared {
    q: Mutex<QState>,
    cv: Condvar,
    /// (resolved URL template, time of the last TileJSON attempt)
    template: Mutex<(Option<String>, Option<Instant>)>,
    downloaded: AtomicU64,
}

pub struct Pool {
    shared: Arc<Shared>,
    rx: mpsc::Receiver<TileResult>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Pool {
    pub fn new(workers: usize) -> Pool {
        std::thread::spawn(prune_cache);
        let shared = Arc::new(Shared {
            q: Mutex::new(QState { jobs: Vec::new(), active: HashSet::new(), quit: false }),
            cv: Condvar::new(),
            template: Mutex::new((None, None)),
            downloaded: AtomicU64::new(0),
        });
        let (tx, rx) = mpsc::channel();
        let threads = (0..workers)
            .map(|i| {
                let shared = shared.clone();
                let tx = tx.clone();
                std::thread::Builder::new()
                    .name(format!("tile-{i}"))
                    .spawn(move || worker(shared, tx))
                    .expect("spawn tile worker")
            })
            .collect();
        Pool { shared, rx, threads }
    }

    /// Replace the job queue. `jobs` is ordered best-first; tiles already in
    /// flight are skipped by the caller.
    pub fn set_jobs(&self, mut jobs: Vec<TileId>) {
        jobs.reverse();
        let mut q = self.shared.q.lock().unwrap();
        jobs.retain(|j| !q.active.contains(j));
        q.jobs = jobs;
        drop(q);
        self.shared.cv.notify_all();
    }

    pub fn in_flight(&self) -> HashSet<TileId> {
        self.shared.q.lock().unwrap().active.clone()
    }

    pub fn queued(&self) -> usize {
        self.shared.q.lock().unwrap().jobs.len()
    }

    pub fn try_recv(&self) -> Option<TileResult> {
        self.rx.try_recv().ok()
    }

    pub fn downloaded_bytes(&self) -> u64 {
        self.shared.downloaded.load(Ordering::Relaxed)
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.shared.q.lock().unwrap().quit = true;
        self.shared.cv.notify_all();
        // Workers may be mid-download; don't block shutdown on them.
        for t in self.threads.drain(..) {
            drop(t);
        }
    }
}

fn worker(shared: Arc<Shared>, tx: mpsc::Sender<TileResult>) {
    let agent = agent();
    loop {
        let id = {
            let mut q = shared.q.lock().unwrap();
            loop {
                if q.quit {
                    return;
                }
                if let Some(id) = q.jobs.pop() {
                    q.active.insert(id);
                    break id;
                }
                q = shared.cv.wait(q).unwrap();
            }
        };
        let t0 = Instant::now();
        let template = {
            let mut t = shared.template.lock().unwrap();
            let due = t.1.is_none_or(|i| i.elapsed() > Duration::from_secs(30));
            if t.0.is_none() && due && !tile_path(id).exists() {
                t.1 = Some(Instant::now());
                match tile_template(&agent) {
                    Ok(s) => t.0 = Some(s),
                    Err(e) => eprintln!("tiles: {e}"),
                }
            }
            t.0.clone()
        };
        let fetched = fetch(&agent, template.as_deref(), id);
        let t1 = Instant::now();
        let mut map = None;
        let (build, from_cache, bytes) = match fetched {
            Ok((bytes, src)) => {
                if matches!(src, Source::Network) {
                    shared.downloaded.fetch_add(bytes.len() as u64, Ordering::Relaxed);
                }
                let n = bytes.len();
                let b = crate::mvt::decode(&bytes).map(|layers| {
                    // Built in the tile's own frame (small coordinates); the
                    // world places it with an exact transform (osm::Xform2).
                    let proj = Proj::tile_frame(id);
                    let feats = crate::osm::extract(&layers, id, &proj);
                    map = Some(std::sync::Arc::new(crate::ui::mapdata::build(id, &proj, &feats, &layers)));
                    if id.is_far() {
                        scene::build_far_tile(id, &proj, feats)
                    } else {
                        scene::build_tile(id, &proj, feats)
                    }
                });
                (b, matches!(src, Source::Cache), n)
            }
            Err(e) => (Err(e), false, 0),
        };
        let r = TileResult {
            id,
            build,
            map,
            fetch_ms: (t1 - t0).as_secs_f32() * 1000.0,
            build_ms: t1.elapsed().as_secs_f32() * 1000.0,
            from_cache,
            bytes,
        };
        shared.q.lock().unwrap().active.remove(&id);
        if tx.send(r).is_err() {
            return;
        }
    }
}

// ---------------------------------------------------------------- any zoom

static TEMPLATE: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// One tile at any zoom (disk cache, then network), decompressed — for the
/// overview map in the menus. Blocking: call from a worker thread.
pub fn fetch_any(id: TileId) -> Result<Vec<u8>, String> {
    let ag = agent();
    if tile_path(id).exists() {
        return fetch(&ag, None, id).map(|r| r.0);
    }
    let t = {
        let mut g = TEMPLATE.lock().unwrap();
        if g.is_none() {
            *g = Some(tile_template(&ag)?);
        }
        g.clone()
    };
    fetch(&ag, t.as_deref(), id).map(|r| r.0)
}

// ---------------------------------------------------------------- airport lookup

/// Resolve an ICAO/IATA code (or any airport name) to lat/lon via Nominatim.
/// Results are cached in `cache/airports.json`.
pub fn resolve_airport(code: &str) -> Result<(f64, f64, String), String> {
    let path = PathBuf::from("cache").join("airports.json");
    let mut db: serde_json::Map<String, serde_json::Value> = std::fs::read_to_string(&path)
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_default();
    let key = code.trim().to_uppercase();
    if let Some(v) = db.get(&key) {
        if let (Some(lat), Some(lon)) = (v.get("lat").and_then(|x| x.as_f64()), v.get("lon").and_then(|x| x.as_f64())) {
            let name = v.get("name").and_then(|x| x.as_str()).unwrap_or(&key).to_string();
            return Ok((lat, lon, name));
        }
    }
    let url = format!("https://nominatim.openstreetmap.org/search?format=jsonv2&limit=5&q={}", key.replace(' ', "+"));
    let text = agent().get(&url).call().map_err(|e| format!("airport lookup: {e}"))?.into_string().map_err(|e| e.to_string())?;
    let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let hit = v
        .as_array()
        .and_then(|a| a.iter().find(|x| x.get("category").and_then(|c| c.as_str()) == Some("aeroway")).or(a.first()))
        .ok_or(format!("airport {key} not found"))?;
    let num = |k: &str| hit.get(k).and_then(|x| x.as_str()).and_then(|s| s.parse::<f64>().ok());
    let (lat, lon) = (num("lat").ok_or("bad lat")?, num("lon").ok_or("bad lon")?);
    let name = hit.get("name").and_then(|x| x.as_str()).unwrap_or(&key).to_string();
    db.insert(key, serde_json::json!({ "lat": lat, "lon": lon, "name": name }));
    let _ = std::fs::create_dir_all("cache");
    let _ = std::fs::write(&path, serde_json::to_string_pretty(&db).unwrap_or_default());
    Ok((lat, lon, name))
}
