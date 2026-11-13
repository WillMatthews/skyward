//! Terrain elevation: DEM tiles, per-tile height grids, conforming drape of
//! the flat map layers, airport flattening.
//!
//! **Data**: AWS Terrain Tiles (Mapzen / Tilezen "joerd"), Terrarium PNG
//! encoding (`h = R·256 + G + B/256 − 32768` m), fetched from
//! `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`
//! and cached under `cache/terrain/` (pruned to [`CACHE_LIMIT`]). Sources:
//! SRTM, GMTED2010, 3DEP (USGS), ETOPO1 (NOAA), EU-DEM (Copernicus), UK
//! Environment Agency LIDAR, and others — see the README credits.
//!
//! **Surfaces** (one per level of detail, each a regular grid over a map
//! tile, every cell split along the same diagonal):
//! * z14 detail tiles — [`GRID_N`]² cells (~24 m at 51°) from the z12 DEM
//!   (one DEM pixel per cell), water bodies flattened, runways flattened to
//!   their profile and blended into the hills ([`Zone`]);
//! * z10 far tiles — [`GRID_N`]² cells (~380 m) from the z8 DEM;
//! * z9 horizon tiles — terrain only, [`GRID_N`]² cells (~760 m), z7 DEM.
//!
//! The flat map layers (landuse, water, roads, ...) are clipped to the grid's
//! triangles ([`drape`]) so they lie exactly on the surface; the renderer
//! draws every tile's *base* surface first (depth write) and the draped
//! layers after it, pulled a hair toward the eye without depth write — no
//! z-fighting, no layer ever shows through a hill. A coarser level is
//! bucketed by the finer level's tiles (16×16 z14 cells per far tile, 2×2
//! z10 cells per horizon tile) so the renderer skips exactly the cells a
//! finer tile covers; skirts hanging below each tile's edges hide the seams.

use crate::osm::{point_in_ring, Area, Proj, TileId};
use crate::scene::{mat, MeshData, Vert};
use glam::{vec2, vec3, Vec2, Vec3};
use std::collections::HashMap;
#[cfg(not(target_arch = "wasm32"))]
use std::io::Read;
use std::path::PathBuf;
use std::sync::{Arc, Condvar, LazyLock, Mutex, RwLock};
use web_time::{Duration, Instant};

const URL: &str = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium";
const USER_AGENT: &str = "skyward-flightsim/0.2";
/// Disk cache budget for DEM tiles (oldest pruned at startup).
const CACHE_LIMIT: u64 = 400_000_000;
/// Decoded DEM tiles kept in memory (256 KB each).
const MEM_TILES: usize = 320;
/// Grid cells per tile side (every level).
pub const GRID_N: usize = 64;
/// DEM zoom per level: detail (z14 tiles), far (z10), horizon (z9).
pub const DEM_NEAR: u32 = 12;
pub const DEM_FAR: u32 = 8;
pub const DEM_HORIZON: u32 = 7;
/// Zoom of the terrain-only horizon tiles.
pub const HORIZON_ZOOM: u32 = 9;
/// Lowest terrain kept (m): ocean bathymetry is cut off here.
const FLOOR: f32 = -12.0;

/// Flat world (sea level everywhere): unit tests, or `SKYWARD_FLAT_TERRAIN=1`.
static FLAT: LazyLock<bool> = LazyLock::new(|| cfg!(test) || std::env::var_os("SKYWARD_FLAT_TERRAIN").is_some());

// ---------------------------------------------------------------- DEM tiles

struct DemTile {
    h: Vec<f32>,
}

enum Entry {
    Loading,
    Ready(Arc<DemTile>, u64),
    Failed(Instant),
}

type DemMap = (HashMap<(u32, u32, u32), Entry>, u64);

struct Dem {
    map: Mutex<DemMap>,
    cv: Condvar,
    #[cfg(not(target_arch = "wasm32"))]
    agent: ureq::Agent,
    warned: std::sync::atomic::AtomicBool,
}

static DEM: LazyLock<Dem> = LazyLock::new(|| {
    #[cfg(not(target_arch = "wasm32"))]
    std::thread::spawn(prune_cache);
    Dem {
        map: Mutex::new((HashMap::new(), 0)),
        cv: Condvar::new(),
        #[cfg(not(target_arch = "wasm32"))]
        agent: ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(25)).user_agent(USER_AGENT).build(),
        warned: Default::default(),
    }
});

fn cache_root() -> PathBuf {
    PathBuf::from("cache").join("terrain")
}

fn tile_path(z: u32, x: u32, y: u32) -> PathBuf {
    cache_root().join(z.to_string()).join(x.to_string()).join(format!("{y}.png"))
}

fn decode(bytes: &[u8]) -> Result<DemTile, String> {
    let img = image::load_from_memory_with_format(bytes, image::ImageFormat::Png).map_err(|e| e.to_string())?.to_rgb8();
    if img.width() != 256 || img.height() != 256 {
        return Err(format!("DEM tile is {}x{}", img.width(), img.height()));
    }
    let h = img.pixels().map(|p| (p[0] as f32 * 256.0 + p[1] as f32 + p[2] as f32 / 256.0 - 32768.0).max(FLOOR)).collect();
    Ok(DemTile { h })
}

#[cfg(not(target_arch = "wasm32"))]
fn load(z: u32, x: u32, y: u32) -> Result<DemTile, String> {
    let path = tile_path(z, x, y);
    if let Ok(b) = std::fs::read(&path) {
        if let Ok(t) = decode(&b) {
            return Ok(t);
        }
    }
    let url = format!("{URL}/{z}/{x}/{y}.png");
    if crate::platform::offline() {
        return Err("offline mode: terrain not cached".into());
    }
    let bytes = match DEM.agent.get(&url).call() {
        Ok(r) => {
            let mut b = Vec::with_capacity(100_000);
            r.into_reader().take(4 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
            b
        }
        // No tile: sea level.
        Err(ureq::Error::Status(403 | 404, _)) => return Ok(DemTile { h: vec![0.0; 256 * 256] }),
        Err(e) => return Err(e.to_string()),
    };
    let t = decode(&bytes)?;
    if let Some(dir) = path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let tmp = path.with_extension(format!("tmp{}", std::process::id()));
    if std::fs::write(&tmp, &bytes).is_ok() {
        let _ = std::fs::rename(&tmp, &path);
    }
    Ok(t)
}

#[cfg(not(target_arch = "wasm32"))]
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
            } else if e.path().extension().is_some_and(|x| x == "png") {
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

/// One DEM tile. `block`: wait for (or do) the download; otherwise only
/// what is already in memory.
fn dem_tile(z: u32, x: u32, y: u32, block: bool) -> Option<Arc<DemTile>> {
    let d = &*DEM;
    let k = (z, x, y);
    let mut g = d.map.lock().unwrap();
    loop {
        g.1 += 1;
        let tick = g.1;
        match g.0.get_mut(&k) {
            Some(Entry::Ready(t, used)) => {
                *used = tick;
                return Some(t.clone());
            }
            Some(Entry::Loading) => {
                #[cfg(target_arch = "wasm32")]
                if block {
                    web::miss(k);
                }
                if !block || cfg!(target_arch = "wasm32") {
                    return None;
                }
                g = d.cv.wait(g).unwrap();
                continue;
            }
            Some(Entry::Failed(t)) if t.elapsed() < Duration::from_secs(30) => return None,
            _ => {}
        }
        if !block {
            return None;
        }
        g.0.insert(k, Entry::Loading);
        drop(g);
        // Browser: no blocking download; start it and note the miss (the
        // tile builder waits for it and builds again).
        #[cfg(target_arch = "wasm32")]
        {
            web::miss(k);
            web::spawn_load(k);
            return None;
        }
        #[cfg(not(target_arch = "wasm32"))]
        return store(k, load(z, x, y));
    }
}

/// Keep a loaded (or failed) DEM tile in memory, evicting the least recently used.
fn store(k: (u32, u32, u32), r: Result<DemTile, String>) -> Option<Arc<DemTile>> {
    let d = &*DEM;
    let (z, x, y) = k;
    let mut g = d.map.lock().unwrap();
    let out = match r {
        Ok(t) => {
            let t = Arc::new(t);
            g.1 += 1;
            let tick = g.1;
            g.0.insert(k, Entry::Ready(t.clone(), tick));
            Some(t)
        }
        Err(e) => {
            if !d.warned.swap(true, std::sync::atomic::Ordering::Relaxed) {
                eprintln!("terrain: DEM {z}/{x}/{y}: {e} (flat ground where missing)");
            }
            g.0.insert(k, Entry::Failed(Instant::now()));
            None
        }
    };
    // Evict the least recently used tiles.
    let ready = g.0.values().filter(|e| matches!(e, Entry::Ready(..))).count();
    if ready > MEM_TILES {
        let mut ages: Vec<(u64, (u32, u32, u32))> = g.0.iter().filter_map(|(k, e)| if let Entry::Ready(_, u) = e { Some((*u, *k)) } else { None }).collect();
        ages.sort();
        for (_, k) in ages.into_iter().take(ready - MEM_TILES) {
            g.0.remove(&k);
        }
    }
    d.cv.notify_all();
    out
}

/// Load the detail DEM tile at a point in the background (if not yet loaded).
pub fn prefetch(lat: f64, lon: f64) {
    let (fx, fy) = TileId::frac(lat, lon, DEM_NEAR);
    let (x, y) = (fx as u32, fy as u32);
    {
        let g = DEM.map.lock().unwrap();
        if g.0.contains_key(&(DEM_NEAR, x, y)) {
            return;
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    std::thread::spawn(move || {
        dem_tile(DEM_NEAR, x, y, true);
    });
    #[cfg(target_arch = "wasm32")]
    web::start_load((DEM_NEAR, x, y));
}

/// Browser DEM loading: async fetches; builds that hit a tile still loading
/// note it ([`web::miss`]) and are built again once it is in
/// (tiles_web.rs).
#[cfg(target_arch = "wasm32")]
pub mod web {
    use super::*;
    use std::cell::RefCell;

    type Key = (u32, u32, u32);

    thread_local! {
        static MISSES: RefCell<Vec<Key>> = const { RefCell::new(Vec::new()) };
    }

    pub(super) fn miss(k: Key) {
        MISSES.with(|m| {
            let mut m = m.borrow_mut();
            if !m.contains(&k) {
                m.push(k);
            }
        });
    }

    /// DEM tiles that were needed but not loaded since the last call.
    pub fn take_misses() -> Vec<Key> {
        MISSES.with(|m| std::mem::take(&mut *m.borrow_mut()))
    }

    /// Start loading a DEM tile unless loaded, loading or recently failed.
    pub fn start_load(k: Key) {
        let mut g = DEM.map.lock().unwrap();
        match g.0.get(&k) {
            Some(Entry::Ready(..) | Entry::Loading) => return,
            Some(Entry::Failed(t)) if t.elapsed() < Duration::from_secs(30) => return,
            _ => {}
        }
        g.0.insert(k, Entry::Loading);
        drop(g);
        spawn_load(k);
    }

    /// Fetch + decode + store (the entry is already `Loading`).
    pub(super) fn spawn_load(k: Key) {
        crate::platform::web::spawn_local(async move {
            let r = load(k).await;
            store(k, r);
        });
    }

    async fn load((z, x, y): Key) -> Result<DemTile, String> {
        let url = format!("{URL}/{z}/{x}/{y}.png");
        let r = crate::platform::web::fetch_cached(&url, Duration::from_secs(25)).await?;
        match r.status {
            200..=299 => decode(&r.body),
            // No tile: sea level.
            403 | 404 => Ok(DemTile { h: vec![0.0; 256 * 256] }),
            s => Err(format!("HTTP {s}")),
        }
    }

    /// The DEM tiles a map tile's build reads: its level's DEM over the tile
    /// (plus a pixel of margin) and its parent level's.
    pub fn keys_for(id: TileId) -> Vec<Key> {
        let mut out = Vec::new();
        let mut add = |t: TileId| {
            let dz = Level::of(t).dem_z();
            if t.z < dz {
                return;
            }
            let scale = (1u64 << (t.z - dz)) as f64;
            let n = 1i64 << dz;
            let px = |v: u32| v as f64 * 256.0 / scale;
            let (x0, x1) = (((px(t.x) - 1.0) / 256.0).floor() as i64, ((px(t.x + 1) + 1.0) / 256.0).floor() as i64);
            let (y0, y1) = (((px(t.y) - 1.0) / 256.0).floor() as i64, ((px(t.y + 1) + 1.0) / 256.0).floor() as i64);
            for y in y0.max(0)..=y1.min(n - 1) {
                for x in x0..=x1 {
                    let k = (dz, x.rem_euclid(n) as u32, y as u32);
                    if !out.contains(&k) {
                        out.push(k);
                    }
                }
            }
        };
        add(id);
        if let Some(p) = parent(id) {
            add(p);
        }
        out
    }

    /// Load `keys` and wait until none is still loading (loaded or failed).
    pub async fn ensure(keys: &[Key]) {
        if *FLAT {
            return;
        }
        for k in keys {
            start_load(*k);
        }
        loop {
            let busy = {
                let g = DEM.map.lock().unwrap();
                keys.iter().any(|k| matches!(g.0.get(k), Some(Entry::Loading)))
            };
            if !busy {
                return;
            }
            crate::platform::web::sleep(Duration::from_millis(25)).await;
        }
    }
}

/// Bilinear DEM sampling at one zoom, with a few tiles held locally.
pub struct Sampler {
    z: u32,
    block: bool,
    tiles: Vec<((u32, u32), Option<Arc<DemTile>>)>,
    /// Some pixel had no data.
    pub missing: bool,
}

impl Sampler {
    pub fn new(z: u32, block: bool) -> Self {
        Sampler { z, block, tiles: Vec::new(), missing: false }
    }
    fn px(&mut self, x: i64, y: i64) -> f32 {
        if *FLAT {
            return 0.0;
        }
        let n = (1i64 << self.z) * 256;
        let (x, y) = (x.rem_euclid(n), y.clamp(0, n - 1));
        let key = ((x / 256) as u32, (y / 256) as u32);
        let t = match self.tiles.iter().find(|t| t.0 == key) {
            Some(t) => t.1.clone(),
            None => {
                let t = dem_tile(self.z, key.0, key.1, self.block);
                self.tiles.push((key, t.clone()));
                t
            }
        };
        match t {
            Some(t) => t.h[((y % 256) * 256 + x % 256) as usize],
            None => {
                self.missing = true;
                0.0
            }
        }
    }
    /// Height (m) at a lat / lon: bilinear between pixel centres.
    pub fn at(&mut self, lat: f64, lon: f64) -> f32 {
        let (fx, fy) = TileId::frac(lat, lon, self.z);
        let (px, py) = (fx * 256.0 - 0.5, fy * 256.0 - 0.5);
        let (x0, y0) = (px.floor(), py.floor());
        let (tx, ty) = ((px - x0) as f32, (py - y0) as f32);
        let (x0, y0) = (x0 as i64, y0 as i64);
        let a = self.px(x0, y0);
        let b = self.px(x0 + 1, y0);
        let c = self.px(x0, y0 + 1);
        let d = self.px(x0 + 1, y0 + 1);
        let top = a + (b - a) * tx;
        let bot = c + (d - c) * tx;
        top + (bot - top) * ty
    }
}

/// Non-blocking height from the DEM tiles already in memory (detail zoom,
/// then coarser ones).
pub fn cached_height(lat: f64, lon: f64) -> Option<f32> {
    for z in [DEM_NEAR, DEM_FAR, DEM_HORIZON] {
        let mut s = Sampler::new(z, false);
        let h = s.at(lat, lon);
        if !s.missing {
            return Some(h);
        }
    }
    None
}

// ---------------------------------------------------------------- grid

/// A tile's terrain surface: (n+1)² node heights over the tile rectangle (in
/// the tile's frame); every cell is split along its (i, j)–(i+1, j+1) diagonal.
#[derive(Clone, Debug)]
pub struct Grid {
    pub n: usize,
    pub lo: Vec2,
    pub size: Vec2,
    pub h: Vec<f32>,
    pub nrm: Vec<[f32; 3]>,
    pub min: f32,
    pub max: f32,
}

impl Grid {
    /// A flat grid at height 0 (offline / tests).
    pub fn flat(lo: Vec2, size: Vec2, n: usize) -> Grid {
        let m = (n + 1) * (n + 1);
        Grid { n, lo, size, h: vec![0.0; m], nrm: vec![[0.0, 1.0, 0.0]; m], min: 0.0, max: 0.0 }
    }
    pub fn cell(&self) -> Vec2 {
        self.size / self.n as f32
    }
    fn node(&self, i: usize, j: usize) -> f32 {
        self.h[j * (self.n + 1) + i]
    }
    pub fn node_pos(&self, i: usize, j: usize) -> Vec2 {
        self.lo + vec2(i as f32, j as f32) * self.cell()
    }
    /// Cell (i, j) and local (u, v) (unclamped outside the grid).
    fn locate(&self, p: Vec2) -> (usize, usize, f32, f32) {
        let q = (p - self.lo) / self.cell();
        let i = (q.x.floor().max(0.0) as usize).min(self.n - 1);
        let j = (q.y.floor().max(0.0) as usize).min(self.n - 1);
        (i, j, q.x - i as f32, q.y - j as f32)
    }
    /// Barycentric weights of `p` in its terrain triangle: node indices and weights.
    fn weights(&self, p: Vec2) -> ([usize; 3], [f32; 3]) {
        let (i, j, u, v) = self.locate(p);
        let w = self.n + 1;
        let (n00, n10, n01, n11) = (j * w + i, j * w + i + 1, (j + 1) * w + i, (j + 1) * w + i + 1);
        if u >= v {
            ([n00, n10, n11], [1.0 - u, u - v, v])
        } else {
            ([n00, n11, n01], [1.0 - v, u, v - u])
        }
    }
    /// Height of the rendered surface at `p` (tile frame).
    pub fn height(&self, p: Vec2) -> f32 {
        let (n, w) = self.weights(p);
        self.h[n[0]] * w[0] + self.h[n[1]] * w[1] + self.h[n[2]] * w[2]
    }
    /// Smooth shading normal at `p` (same interpolation as the mesh).
    pub fn normal(&self, p: Vec2) -> Vec3 {
        let (n, w) = self.weights(p);
        let a = Vec3::from(self.nrm[n[0]]) * w[0] + Vec3::from(self.nrm[n[1]]) * w[1] + Vec3::from(self.nrm[n[2]]) * w[2];
        a.normalize_or(Vec3::Y)
    }
    /// Lowest surface height over a footprint (its vertices and points along
    /// its edges at about half the cell size).
    pub fn min_over(&self, ring: &[Vec2]) -> f32 {
        let step = self.cell().min_element().max(1.0) * 0.5;
        let mut m = f32::MAX;
        let n = ring.len();
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            let k = ((a.distance(b) / step).ceil() as usize).clamp(1, 64);
            for s in 0..k {
                m = m.min(self.height(a.lerp(b, s as f32 / k as f32)));
            }
        }
        if m == f32::MAX { 0.0 } else { m }
    }
}

/// Which surface a grid samples.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Level {
    Near,
    Far,
    Horizon,
}

impl Level {
    fn dem_z(self) -> u32 {
        match self {
            Level::Near => DEM_NEAR,
            Level::Far => DEM_FAR,
            Level::Horizon => DEM_HORIZON,
        }
    }
    pub fn of(id: TileId) -> Level {
        if id.z >= crate::osm::ZOOM {
            Level::Near
        } else if id.z == crate::osm::FAR_ZOOM {
            Level::Far
        } else {
            Level::Horizon
        }
    }
}

/// Build a tile's grid. `water`: polygons whose nodes are flattened to the
/// local water level; runway [`Zone`]s are applied (detail level).
pub fn build_grid(id: TileId, water: &[&Area], zones: &[Zone]) -> Grid {
    let level = Level::of(id);
    let proj = Proj::tile_frame(id);
    let (lo, hi) = id.rect(&proj);
    let n = GRID_N;
    let size = hi - lo;
    let cell = size / n as f32;
    // Heights with a two-node border: normals and the water filter across
    // tile edges then agree exactly with the neighbours'.
    const B: usize = 2;
    let m = n + 1 + 2 * B;
    let mut s = Sampler::new(level.dem_z(), true);
    let pos = |i: usize, j: usize| lo + vec2(i as f32 - B as f32, j as f32 - B as f32) * cell;
    let mut h = vec![0f32; m * m];
    for j in 0..m {
        for i in 0..m {
            let (lat, lon) = proj.to_latlon(pos(i, j));
            h[j * m + i] = s.at(lat, lon);
        }
    }
    // Water bodies: the local minimum (lakes flat, rivers smoothed, the sea at 0),
    // on the nodes in or next to the water (so the banks' triangles stay flat too).
    let mut wet_nodes: Vec<bool> = Vec::new();
    if !water.is_empty() {
        let raw = h.clone();
        let mut wet = vec![false; m * m];
        for a in water {
            let (blo, bhi) = bbox(&a.outer);
            for j in 0..m {
                for i in 0..m {
                    let p = pos(i, j);
                    if !wet[j * m + i]
                        && p.x >= blo.x
                        && p.x <= bhi.x
                        && p.y >= blo.y
                        && p.y <= bhi.y
                        && point_in_ring(p, &a.outer)
                        && !a.holes.iter().any(|hh| point_in_ring(p, hh))
                    {
                        wet[j * m + i] = true;
                    }
                }
            }
        }
        let mut grown = wet.clone();
        for j in 1..m - 1 {
            for i in 1..m - 1 {
                if wet[j * m + i] {
                    for (di, dj) in [(0i32, 1i32), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)] {
                        grown[(j as i32 + dj) as usize * m + (i as i32 + di) as usize] = true;
                    }
                }
            }
        }
        wet_nodes = grown.clone();
        for j in B..m - B {
            for i in B..m - B {
                if grown[j * m + i] {
                    let mut lv = f32::MAX;
                    for y in j - B..=j + B {
                        for x in i - B..=i + B {
                            lv = lv.min(raw[y * m + x]);
                        }
                    }
                    h[j * m + i] = if lv < 0.5 { lv.max(0.0) } else { lv };
                }
            }
        }
    }
    // Runway planes must be met exactly (the runway mesh lies on them).
    let mut strict = vec![false; m * m];
    if !zones.is_empty() {
        for j in 0..m {
            for i in 0..m {
                let (w, v) = flatten_w(&proj, zones, pos(i, j), h[j * m + i], !wet_nodes.get(j * m + i).copied().unwrap_or(false));
                h[j * m + i] = v;
                strict[j * m + i] = w > 0.0;
            }
        }
    }
    // Full-resolution heights and normals of the tile's nodes.
    let w = n + 1;
    let mut fh = vec![0f32; w * w];
    let mut fnrm = vec![[0f32; 3]; w * w];
    let mut fstrict = vec![false; w * w];
    for j in 0..w {
        for i in 0..w {
            let (ii, jj) = (i + B, j + B);
            fh[j * w + i] = h[jj * m + ii];
            fstrict[j * w + i] = strict[jj * m + ii];
            let dx = (h[jj * m + ii + 1] - h[jj * m + ii - 1]) / (2.0 * cell.x);
            let dz = (h[(jj + 1) * m + ii] - h[(jj - 1) * m + ii]) / (2.0 * cell.y);
            fnrm[j * w + i] = vec3(-dx, 1.0, -dz).normalize().into();
        }
    }
    // The coarsest grid (64 / s cells) that stays within the level's tolerance
    // of the full surface everywhere (much less where runways are): flat
    // ground drapes into far fewer pieces.
    let (tol, min_n) = match level {
        Level::Near => (5.0, 8),
        Level::Far => (15.0, 16),
        Level::Horizon => (30.0, 16),
    };
    let mut s = 1;
    for cand in [8usize, 4, 2] {
        if n / cand < min_n {
            continue;
        }
        let nc = n / cand;
        let wc = nc + 1;
        let ch = |i: usize, j: usize| fh[(j * cand) * w + i * cand];
        let ok = (0..w).all(|jj| {
            (0..w).all(|ii| {
                let (i, j) = ((ii / cand).min(nc - 1), (jj / cand).min(nc - 1));
                let (u, v) = ((ii - i * cand) as f32 / cand as f32, (jj - j * cand) as f32 / cand as f32);
                let _ = wc;
                let c = if u >= v {
                    ch(i, j) + u * (ch(i + 1, j) - ch(i, j)) + v * (ch(i + 1, j + 1) - ch(i + 1, j))
                } else {
                    ch(i, j) + v * (ch(i, j + 1) - ch(i, j)) + u * (ch(i + 1, j + 1) - ch(i, j + 1))
                };
                let lim = if fstrict[jj * w + ii] { 0.03 } else { tol };
                (c - fh[jj * w + ii]).abs() <= lim
            })
        });
        if ok {
            s = cand;
            break;
        }
    }
    let nc = n / s;
    let wc = nc + 1;
    let mut gh = vec![0f32; wc * wc];
    let mut nrm = vec![[0f32; 3]; wc * wc];
    let (mut mn, mut mx) = (f32::MAX, f32::MIN);
    for j in 0..wc {
        for i in 0..wc {
            let k = (j * s) * w + i * s;
            gh[j * wc + i] = fh[k];
            nrm[j * wc + i] = fnrm[k];
            mn = mn.min(fh[k]);
            mx = mx.max(fh[k]);
        }
    }
    Grid { n: nc, lo, size, h: gh, nrm, min: mn, max: mx }
}

/// The far / horizon grid of a coarse tile (memoised: finer tiles use it to
/// size their skirts).
pub fn coarse_grid(id: TileId) -> Arc<Grid> {
    static MEMO: LazyLock<Mutex<HashMap<TileId, (Arc<Grid>, u64)>>> = LazyLock::new(Default::default);
    static TICK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let t = TICK.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    if let Some((g, u)) = MEMO.lock().unwrap().get_mut(&id) {
        *u = t;
        return g.clone();
    }
    let g = Arc::new(build_grid(id, &[], &[]));
    let mut m = MEMO.lock().unwrap();
    m.insert(id, (g.clone(), t));
    if m.len() > 64 {
        let oldest = m.iter().min_by_key(|e| e.1 .1).map(|e| *e.0).unwrap();
        m.remove(&oldest);
    }
    g
}

/// The next coarser level's tile containing a tile (none for horizon tiles).
pub fn parent(id: TileId) -> Option<TileId> {
    let z = match Level::of(id) {
        Level::Near => crate::osm::FAR_ZOOM,
        Level::Far => HORIZON_ZOOM,
        Level::Horizon => return None,
    };
    let s = id.z - z;
    Some(TileId { z, x: id.x >> s, y: id.y >> s })
}

fn bbox(pts: &[Vec2]) -> (Vec2, Vec2) {
    pts.iter().fold((Vec2::splat(f32::MAX), Vec2::splat(f32::MIN)), |(a, b), p| (a.min(*p), b.max(*p)))
}

// ---------------------------------------------------------------- airports

/// A runway flattening zone: the runway's plane (profile `ha` → `hb` along
/// a → b, level across) over the pavement and a margin, blended into the
/// terrain around it. `grade` (m, from the centreline) is the half-width of
/// the graded airfield around it — aprons, taxiways, stands: there the
/// ground is pulled to the same plane (level across the runway, the way
/// airfields are graded), but only where the natural terrain is within a few
/// metres of it, so valley sides and embankments are left alone.
#[derive(Clone, Debug)]
pub struct Zone {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub width: f32,
    pub ha: f32,
    pub hb: f32,
    pub grade: f32,
}

/// Flat margin beyond the pavement edge (≥ a grid cell diagonal, so every
/// terrain triangle touching the runway lies in the runway plane), beyond
/// the ends, and the blend length into the natural terrain.
const ZONE_MARGIN: f32 = 40.0;
const ZONE_END: f32 = 90.0;
const ZONE_BLEND: f32 = 260.0;
/// Graded airfield: blend length beyond `grade`, extra length beyond the
/// runway ends, and the height differences (natural terrain vs the plane,
/// m) over which grading fades out.
const GRADE_BLEND: f32 = 240.0;
const GRADE_END: f32 = 300.0;
const GRADE_DH: (f32, f32) = (6.0, 22.0);

/// Graded half-width (m from the centreline) of the airfield around a
/// runway of an airport of this kind (aprons and taxiways lie within it).
pub fn grade_width(kind: crate::airports::Kind, len_m: f32, paved: bool) -> f32 {
    use crate::airports::Kind;
    match kind {
        Kind::Large => 1000.0,
        Kind::Medium => (300.0 + len_m * 0.12).min(650.0),
        Kind::Small if paved => 160.0,
        Kind::Small => 90.0,
        Kind::Seaplane => 0.0,
    }
}

fn smooth01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

impl Zone {
    /// (weight 0..1, plane height) at a point of the frame `proj`.
    fn at(&self, proj: &Proj, p: Vec2) -> (f32, f32) {
        let (w, _, h) = self.at2(proj, p);
        (w, h)
    }
    /// (runway weight, graded-airfield weight before the height test, plane height).
    fn at2(&self, proj: &Proj, p: Vec2) -> (f32, f32, f32) {
        let a = proj.to_world(self.a.0, self.a.1);
        let b = proj.to_world(self.b.0, self.b.1);
        let len = a.distance(b).max(1.0);
        let d = (b - a) / len;
        let q = p - a;
        let s = q.dot(d);
        let l = q.perp_dot(d).abs();
        let h = self.ha + (self.hb - self.ha) * (s / len).clamp(0.0, 1.0);
        let out_l = (l - (self.width * 0.5 + ZONE_MARGIN)).max(0.0);
        let out_s = (-s - ZONE_END).max(0.0).max(s - len - ZONE_END);
        let out = (out_l * out_l + out_s * out_s).sqrt();
        let t = (out / ZONE_BLEND).clamp(0.0, 1.0);
        let wg = if self.grade > 0.0 {
            let gl = (l - self.grade).max(0.0);
            let gs = (-s - GRADE_END).max(0.0).max(s - len - GRADE_END);
            1.0 - smooth01((gl * gl + gs * gs).sqrt() / GRADE_BLEND)
        } else {
            0.0
        };
        (1.0 - t * t * (3.0 - 2.0 * t), wg, h)
    }
    /// Plane height of the runway at a point (ends extended level).
    pub fn plane(&self, proj: &Proj, p: Vec2) -> f32 {
        self.at(proj, p).1
    }
}

fn flatten(proj: &Proj, zones: &[Zone], p: Vec2, h: f32) -> f32 {
    flatten_w(proj, zones, p, h, true).1
}

/// (runway weight, flattened height). `grade`: also grade the airfield
/// around the runways (not on water).
fn flatten_w(proj: &Proj, zones: &[Zone], p: Vec2, h: f32, grade: bool) -> (f32, f32) {
    let mut best = (0.0f32, h);
    // Graded airfield: the planes of every runway around, weighted (crossing
    // runways' planes meet smoothly), each faded by its height difference.
    let (mut sw, mut swh, mut wmax) = (0.0f32, 0.0f32, 0.0f32);
    for z in zones {
        let (w, wg, zh) = z.at2(proj, p);
        if w > best.0 {
            best = (w, zh);
        }
        if grade && wg > 0.0 {
            let wg = wg * (1.0 - smooth01(((zh - h).abs() - GRADE_DH.0) / (GRADE_DH.1 - GRADE_DH.0)));
            if wg > 0.0 {
                sw += wg;
                swh += wg * zh;
                wmax = wmax.max(wg);
            }
        }
    }
    let hg = if sw > 0.0 { h + (swh / sw - h) * wmax } else { h };
    (best.0, hg + (best.1 - hg) * best.0)
}

static DB: RwLock<Option<Arc<crate::airports::Db>>> = RwLock::new(None);

/// The airport database the tile workers flatten runways from.
pub fn set_airport_db(db: Arc<crate::airports::Db>) {
    *DB.write().unwrap() = Some(db);
}

/// The airport database the workers use (airport detail: tile styling).
pub fn airport_db() -> Option<Arc<crate::airports::Db>> {
    DB.read().unwrap().clone()
}

pub fn has_airport_db() -> bool {
    DB.read().unwrap().is_some()
}

type Profiles = Arc<Vec<(String, Zone)>>;
static PROFILES: LazyLock<Mutex<HashMap<String, Profiles>>> = LazyLock::new(Default::default);

/// Runway planes of one airport, by runway name: a least-squares line
/// through the (raw) detail DEM along each runway, then crossing runways
/// shifted to meet at their intersection. Deterministic and memoised, so
/// the workers' flattening and the runway meshes agree.
/// `block` = false: only from DEM tiles already in memory (None if missing).
pub fn airport_profiles(a: &crate::airports::Airport, block: bool) -> Option<Profiles> {
    if let Some(p) = PROFILES.lock().unwrap().get(&a.ident) {
        return Some(p.clone());
    }
    let proj = Proj::new(a.lat, a.lon);
    let mut s = Sampler::new(DEM_NEAR, block);
    let mut out: Vec<(String, Zone, Vec2, Vec2)> = Vec::new();
    for r in &a.runways {
        let Some(rw) = crate::airports::to_world(a, r, &proj) else { continue };
        let len = rw.length();
        let d = (rw.b - rw.a) / len;
        let n = vec2(-d.y, d.x);
        // Least squares h = c0 + c1 t (t 0..1) over the centreline and both sides.
        let k = ((len / 25.0) as usize).clamp(4, 400);
        let (mut st, mut sh, mut stt, mut sth, mut cnt) = (0f64, 0f64, 0f64, 0f64, 0f64);
        for i in 0..=k {
            let t = i as f32 / k as f32;
            for o in [-0.4f32, 0.0, 0.4] {
                let p = rw.a + d * (len * t) + n * (rw.width * o);
                let (lat, lon) = proj.to_latlon(p);
                let h = s.at(lat, lon) as f64;
                let t = t as f64;
                st += t;
                sh += h;
                stt += t * t;
                sth += t * h;
                cnt += 1.0;
            }
        }
        if s.missing {
            return None;
        }
        let den = cnt * stt - st * st;
        let c1 = if den.abs() > 1e-9 { (cnt * sth - st * sh) / den } else { 0.0 };
        // Keep the terrain's slope (Lukla ≈ 12 %), but nothing steeper than 14 %.
        let max_rise = (len * 0.14) as f64;
        let c1 = c1.clamp(-max_rise, max_rise);
        let c0 = (sh - c1 * st) / cnt;
        let la = proj.to_latlon(rw.a);
        let lb = proj.to_latlon(rw.b);
        let grade = grade_width(a.kind, len, r.surface.paved());
        out.push((rw.name.clone(), Zone { a: la, b: lb, width: rw.width, ha: c0 as f32, hb: (c0 + c1) as f32, grade }, rw.a, rw.b));
    }
    // Crossing runways meet at the same height.
    for _ in 0..3 {
        for i in 0..out.len() {
            for j in i + 1..out.len() {
                let (a0, a1, b0, b1) = (out[i].2, out[i].3, out[j].2, out[j].3);
                if let Some(x) = seg_intersect(a0, a1, b0, b1) {
                    let hi = out[i].1.plane(&proj, x);
                    let hj = out[j].1.plane(&proj, x);
                    let e = (hi - hj) * 0.5;
                    out[i].1.ha -= e;
                    out[i].1.hb -= e;
                    out[j].1.ha += e;
                    out[j].1.hb += e;
                }
            }
        }
    }
    let p: Profiles = Arc::new(out.into_iter().map(|(n, z, _, _)| (n, z)).collect());
    PROFILES.lock().unwrap().insert(a.ident.clone(), p.clone());
    Some(p)
}

fn seg_intersect(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> Option<Vec2> {
    let r = a1 - a0;
    let s = b1 - b0;
    let den = r.perp_dot(s);
    if den.abs() < 1e-6 {
        return None;
    }
    let t = (b0 - a0).perp_dot(s) / den;
    let u = (b0 - a0).perp_dot(r) / den;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then(|| a0 + r * t)
}

/// Flattening zones of the runways near a tile (blocking: computes profiles).
pub fn zones_for(id: TileId) -> Vec<Zone> {
    let Some(db) = DB.read().unwrap().clone() else { return Vec::new() };
    let proj = Proj::tile_frame(id);
    let (lo, hi) = id.rect(&proj);
    let (lat, lon) = proj.to_latlon((lo + hi) * 0.5);
    let reach = (hi - lo).length() * 0.5 + 6000.0;
    let mut out = Vec::new();
    for (i, _) in db.near(lat, lon, reach as f64 / 1000.0) {
        let a = &db.airports[i];
        if a.kind == crate::airports::Kind::Seaplane || a.runways.is_empty() {
            continue;
        }
        let Some(p) = airport_profiles(a, true) else { continue };
        for (_, z) in p.iter() {
            // Only zones whose influence reaches the tile.
            let (pa, pb) = (proj.to_world(z.a.0, z.a.1), proj.to_world(z.b.0, z.b.1));
            let r = (z.width * 0.5 + ZONE_MARGIN + ZONE_BLEND + ZONE_END).max(z.grade + GRADE_BLEND + GRADE_END);
            let (zlo, zhi) = (pa.min(pb) - Vec2::splat(r), pa.max(pb) + Vec2::splat(r));
            if zhi.x >= lo.x - 50.0 && zlo.x <= hi.x + 50.0 && zhi.y >= lo.y - 50.0 && zlo.y <= hi.y + 50.0 {
                out.push(z.clone());
            }
        }
    }
    out
}

/// Plane heights (at a, b) of a database runway, if its profile is known
/// (non-blocking: from DEM tiles in memory).
pub fn runway_elevation(rw: &crate::scene::Runway, proj: &Proj) -> Option<(f32, f32)> {
    let db = DB.read().unwrap().clone()?;
    let ap = db.get(&rw.airport)?;
    let p = airport_profiles(ap, false)?;
    let (_, z) = p.iter().find(|(n, _)| *n == rw.name)?;
    Some((z.plane(proj, rw.a), z.plane(proj, rw.b)))
}

/// DEM-fitted plane heights of a runway without a database profile
/// (non-blocking).
pub fn fit_elevation(rw: &crate::scene::Runway, proj: &Proj) -> Option<(f32, f32)> {
    let mut s = Sampler::new(DEM_NEAR, false);
    let k = 12;
    let hs: Vec<f32> = (0..=k)
        .map(|i| {
            let (lat, lon) = proj.to_latlon(rw.a.lerp(rw.b, i as f32 / k as f32));
            s.at(lat, lon)
        })
        .collect();
    if s.missing {
        return None;
    }
    let n = hs.len() as f32;
    let mean_h = hs.iter().sum::<f32>() / n;
    let (mut num, mut den) = (0.0, 0.0);
    for (i, h) in hs.iter().enumerate() {
        let t = i as f32 / k as f32 - 0.5;
        num += t * (h - mean_h);
        den += t * t;
    }
    let c1 = num / den;
    Some((mean_h - c1 * 0.5, mean_h + c1 * 0.5))
}

// ---------------------------------------------------------------- meshes

/// Index ranges of a tile's flat mesh: base surface (drawn with depth write)
/// then draped layers, each split into `b`×`b` buckets (the next finer
/// level's tiles; 1 for detail tiles): `base[k]..base[k+1]`, `over[k]..over[k+1]`.
#[derive(Clone, Debug, Default)]
pub struct FlatRanges {
    pub b: u32,
    pub base: Vec<u32>,
    pub over: Vec<u32>,
}

/// Buckets per side for a tile's level.
pub fn buckets_per_side(id: TileId) -> usize {
    match Level::of(id) {
        Level::Near => 1,
        Level::Far => 1 << (crate::osm::ZOOM - crate::osm::FAR_ZOOM),
        Level::Horizon => 1 << (crate::osm::FAR_ZOOM - HORIZON_ZOOM),
    }
}

/// Assemble a tile's flat mesh: the base surface (grid + skirts) and the
/// draped `layers` (built flat at y = 0, in painter's order).
pub fn assemble(id: TileId, grid: &Grid, layers: &MeshData, base_col: [f32; 3]) -> (MeshData, FlatRanges) {
    let n = grid.n;
    let b = buckets_per_side(id);
    let per = n / b;
    let bucket = |i: usize, j: usize| (j / per) * b + i / per;
    let mut out = MeshData::default();
    let mut base_b: Vec<Vec<u32>> = vec![Vec::new(); b * b];
    let mut over_b: Vec<Vec<u32>> = vec![Vec::new(); b * b];
    // Base grid: procedural fields (the layers above give it land use).
    let w = n + 1;
    out.mat = mat::GROUND;
    for j in 0..w {
        for i in 0..w {
            let p = grid.node_pos(i, j);
            out.v(vec3(p.x, grid.node(i, j), p.y), Vec3::from(grid.nrm[j * w + i]), base_col);
        }
    }
    for j in 0..n {
        for i in 0..n {
            let (a, bb, c, d) = ((j * w + i) as u32, (j * w + i + 1) as u32, ((j + 1) * w + i) as u32, ((j + 1) * w + i + 1) as u32);
            base_b[bucket(i, j)].extend([a, bb, d, a, d, c]);
        }
    }
    // Skirts below the edges, down past the next coarser surface.
    let coarse = parent(id).map(|p| (coarse_grid(p), Proj::tile_frame(p)));
    let proj = Proj::tile_frame(id);
    let below = |p: Vec2, h: f32| -> f32 {
        let c = match &coarse {
            Some((g, cp)) => {
                let (lat, lon) = proj.to_latlon(p);
                g.height(cp.to_world(lat, lon))
            }
            None => h,
        };
        h.min(c) - 12.0 - 0.02 * grid.cell().max_element()
    };
    // (Shaded like the ground at the edge, so a seen skirt reads as a slope.)
    out.mat = mat::GROUND;
    for e in 0..4 {
        let f: fn(usize, usize) -> (usize, usize) = match e {
            0 => |k, _| (k, 0),
            1 => |k, n| (k, n),
            2 => |k, _| (0, k),
            _ => |k, n| (n, k),
        };
        for k in 0..n {
            let (i0, j0) = f(k, n);
            let (i1, j1) = f(k + 1, n);
            let (p0, p1) = (grid.node_pos(i0, j0), grid.node_pos(i1, j1));
            let (h0, h1) = (grid.node(i0, j0), grid.node(i1, j1));
            let base = out.verts.len() as u32;
            let (n0, n1) = (Vec3::from(grid.nrm[j0 * w + i0]), Vec3::from(grid.nrm[j1 * w + i1]));
            out.v(vec3(p0.x, h0, p0.y), n0, base_col);
            out.v(vec3(p1.x, h1, p1.y), n1, base_col);
            out.v(vec3(p1.x, below(p1, h1), p1.y), n1, base_col);
            out.v(vec3(p0.x, below(p0, h0), p0.y), n0, base_col);
            base_b[bucket(i0.min(n - 1), j0.min(n - 1))].extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }
    let t0 = Instant::now();
    let v0 = out.verts.len();
    drape(layers, grid, &mut out, |i, j, idx| over_b[bucket(i, j)].extend_from_slice(idx));
    if std::env::var_os("SKYWARD_TERRAIN_DEBUG").is_some() {
        eprintln!(
            "terrain {}/{}/{}: n {} h {:.0}..{:.0} layers {} tris -> {} tris {} verts ({:.1} ms drape)",
            id.z, id.x, id.y, grid.n, grid.min, grid.max, layers.idx.len() / 3, over_b.iter().map(|v| v.len()).sum::<usize>() / 3, out.verts.len() - v0, t0.elapsed().as_secs_f32() * 1000.0
        );
    }
    let mut ranges = FlatRanges { b: b as u32, base: vec![0], over: Vec::new() };
    let mut idx = Vec::with_capacity(base_b.iter().chain(&over_b).map(|v| v.len()).sum());
    for v in &base_b {
        idx.extend_from_slice(v);
        ranges.base.push(idx.len() as u32);
    }
    ranges.over.push(idx.len() as u32);
    for v in &over_b {
        idx.extend_from_slice(v);
        ranges.over.push(idx.len() as u32);
    }
    out.idx = idx;
    (out, ranges)
}

/// Keep the part of a convex polygon where `a·p + c >= 0`.
fn clip_half(poly: &[Vec2], a: Vec2, c: f32, out: &mut Vec<Vec2>) {
    out.clear();
    let n = poly.len();
    for k in 0..n {
        let p = poly[k];
        let q = poly[(k + 1) % n];
        let dp = a.dot(p) + c;
        let dq = a.dot(q) + c;
        if dp >= 0.0 {
            out.push(p);
        }
        if (dp >= 0.0) != (dq >= 0.0) {
            let t = dp / (dp - dq);
            out.push(p + (q - p) * t);
        }
    }
}

/// Clip every triangle of `src` (flat, y ignored) to the terrain triangles of
/// `grid` and emit the pieces into `out` on the surface (terrain heights and
/// normals; colours / uvs / materials interpolated from the source triangle).
/// `push(i, j, indices)` receives each piece's indices with its grid cell.
pub fn drape(src: &MeshData, grid: &Grid, out: &mut MeshData, mut push: impl FnMut(usize, usize, &[u32])) {
    let n = grid.n;
    let cell = grid.cell();
    let lo = grid.lo;
    let (mut row, mut col, mut piece, mut tmp) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut idx: Vec<u32> = Vec::new();
    // Pieces share vertices (same point, same attributes).
    let mut seen: HashMap<(i32, i32, u64), u32> = HashMap::new();
    let ci = |x: f32| (((x - lo.x) / cell.x).floor().max(0.0) as usize).min(n - 1);
    let cj = |z: f32| (((z - lo.y) / cell.y).floor().max(0.0) as usize).min(n - 1);
    for t in src.idx.chunks_exact(3) {
        let vs = [src.verts[t[0] as usize], src.verts[t[1] as usize], src.verts[t[2] as usize]];
        let p = [vec2(vs[0].pos[0], vs[0].pos[2]), vec2(vs[1].pos[0], vs[1].pos[2]), vec2(vs[2].pos[0], vs[2].pos[2])];
        let area = (p[1] - p[0]).perp_dot(p[2] - p[0]);
        if area.abs() < 1e-6 || !area.is_finite() {
            continue;
        }
        let inv = 1.0 / area;
        let (blo, bhi) = (p[0].min(p[1]).min(p[2]), p[0].max(p[1]).max(p[2]));
        let (i0, i1, j0, j1) = (ci(blo.x), ci(bhi.x), cj(blo.y), cj(bhi.y));
        for j in j0..=j1 {
            // Row strip (the outer rows / columns extend beyond the grid).
            let z0 = lo.y + j as f32 * cell.y;
            let z1 = z0 + cell.y;
            if j > 0 {
                clip_half(&p, vec2(0.0, 1.0), -z0, &mut tmp);
            } else {
                tmp.clear();
                tmp.extend_from_slice(&p);
            }
            if j + 1 < n {
                clip_half(&tmp, vec2(0.0, -1.0), z1, &mut row);
            } else {
                std::mem::swap(&mut row, &mut tmp);
            }
            if row.len() < 3 {
                continue;
            }
            for i in i0..=i1 {
                let x0 = lo.x + i as f32 * cell.x;
                let x1 = x0 + cell.x;
                if i > 0 {
                    clip_half(&row, vec2(1.0, 0.0), -x0, &mut tmp);
                } else {
                    tmp.clear();
                    tmp.extend_from_slice(&row);
                }
                if i + 1 < n {
                    clip_half(&tmp, vec2(-1.0, 0.0), x1, &mut col);
                } else {
                    col.clear();
                    col.extend_from_slice(&tmp);
                }
                if col.len() < 3 {
                    continue;
                }
                // Split along the cell diagonal u = v: (x - x0)/cw - (z - z0)/ch >= 0.
                let dn = vec2(1.0 / cell.x, -1.0 / cell.y);
                let dc = -x0 / cell.x + z0 / cell.y;
                for side in [1.0f32, -1.0] {
                    clip_half(&col, dn * side, dc * side, &mut piece);
                    if piece.len() < 3 {
                        continue;
                    }
                    idx.clear();
                    let mut vi: [u32; 16] = [0; 16];
                    if piece.len() > 16 {
                        continue;
                    }
                    for (pi, q) in piece.iter().enumerate() {
                        // Barycentric in the source triangle.
                        let l1 = (*q - p[0]).perp_dot(p[2] - p[0]) * inv;
                        let l2 = (p[1] - p[0]).perp_dot(*q - p[0]) * inv;
                        let l0 = 1.0 - l1 - l2;
                        let lerp3 = |a: [f32; 3], b: [f32; 3], c: [f32; 3]| [a[0] * l0 + b[0] * l1 + c[0] * l2, a[1] * l0 + b[1] * l1 + c[1] * l2, a[2] * l0 + b[2] * l1 + c[2] * l2];
                        let colr = lerp3(vs[0].col, vs[1].col, vs[2].col);
                        let uv = [vs[0].uv[0] * l0 + vs[1].uv[0] * l1 + vs[2].uv[0] * l2, vs[0].uv[1] * l0 + vs[1].uv[1] * l1 + vs[2].uv[1] * l2];
                        let key = (
                            (q.x * 500.0).round() as i32,
                            (q.y * 500.0).round() as i32,
                            crate::scene::hash(colr[0].to_bits() as u64 ^ (colr[1].to_bits() as u64) << 21 ^ (colr[2].to_bits() as u64) << 42)
                                ^ crate::scene::hash((uv[0].to_bits() as u64) << 32 | uv[1].to_bits() as u64 ^ (vs[0].mat.to_bits() as u64) << 7),
                        );
                        vi[pi] = *seen.entry(key).or_insert_with(|| {
                            out.verts.push(Vert { pos: [q.x, grid.height(*q), q.y], nrm: grid.normal(*q).into(), col: colr, uv, mat: vs[0].mat });
                            (out.verts.len() - 1) as u32
                        });
                    }
                    for k in 1..piece.len() - 1 {
                        let (a, b, c) = (vi[0], vi[k], vi[k + 1]);
                        if a != b && b != c && a != c {
                            idx.extend([a, b, c]);
                        }
                    }
                    push(i, j, &idx);
                }
            }
        }
    }
}

// ---------------------------------------------------------------- horizon

/// A terrain-only horizon tile (z9): base surface + skirts, bucketed by z10 cells.
pub fn build_horizon_tile(id: TileId) -> crate::scene::TileBuild {
    let grid = build_grid(id, &[], &[]);
    let (flat, ranges) = assemble(id, &grid, &MeshData::default(), crate::scene::LAND);
    let proj = Proj::tile_frame(id);
    let rect = id.rect(&proj);
    crate::scene::TileBuild::terrain_only(id, rect, flat, ranges, Arc::new(grid))
}

// ---------------------------------------------------------------- wire

/// (Browser tile workers: platform/wire.rs.)
mod wire_impls {
    use super::*;
    use crate::platform::wire::{get_pod, put_pod, Res, Wire, R, W};

    impl Wire for FlatRanges {
        fn put(&self, w: &mut W) {
            self.b.put(w);
            put_pod(w, &self.base);
            put_pod(w, &self.over);
        }
        fn get(r: &mut R) -> Res<Self> {
            Ok(FlatRanges { b: u32::get(r)?, base: get_pod(r)?, over: get_pod(r)? })
        }
    }

    impl Wire for Grid {
        fn put(&self, w: &mut W) {
            self.n.put(w);
            self.lo.put(w);
            self.size.put(w);
            put_pod(w, &self.h);
            put_pod(w, &self.nrm);
            self.min.put(w);
            self.max.put(w);
        }
        fn get(r: &mut R) -> Res<Self> {
            Ok(Grid { n: usize::get(r)?, lo: Vec2::get(r)?, size: Vec2::get(r)?, h: get_pod(r)?, nrm: get_pod(r)?, min: f32::get(r)?, max: f32::get(r)? })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wavy(n: usize) -> Grid {
        let mut g = Grid::flat(Vec2::ZERO, vec2(640.0, 640.0), n);
        for j in 0..=n {
            for i in 0..=n {
                g.h[j * (n + 1) + i] = (i as f32 * 1.7 + j as f32 * 0.6).sin() * 20.0 + i as f32;
            }
        }
        g
    }

    fn tri_area(m: &MeshData) -> f32 {
        m.idx
            .chunks(3)
            .map(|t| {
                let p = |i: u32| vec2(m.verts[i as usize].pos[0], m.verts[i as usize].pos[2]);
                (p(t[1]) - p(t[0])).perp_dot(p(t[2]) - p(t[0])).abs() * 0.5
            })
            .sum()
    }

    #[test]
    fn drape_lies_on_the_surface() {
        let g = wavy(8);
        let mut src = MeshData::default();
        src.flat_poly(&[vec2(13.0, 7.0), vec2(600.0, 70.0), vec2(500.0, 630.0), vec2(40.0, 420.0)], &[], 0.0, Vec3::Y, [0.5; 3]);
        // Every vertex is on the surface and every piece lies in one terrain triangle.
        let mut m3 = MeshData::default();
        let mut pieces = Vec::new();
        drape(&src, &g, &mut m3, |_, _, idx| pieces.extend_from_slice(idx));
        m3.idx = pieces;
        assert!(m3.idx.len() / 3 > 64);
        for t in m3.idx.chunks(3) {
            let ps: Vec<Vec3> = t.iter().map(|&i| Vec3::from(m3.verts[i as usize].pos)).collect();
            for p in &ps {
                assert!((p.y - g.height(vec2(p.x, p.z))).abs() < 1e-3);
            }
            let c = (ps[0] + ps[1] + ps[2]) / 3.0;
            assert!((c.y - g.height(vec2(c.x, c.z))).abs() < 1e-3, "piece not planar with the terrain");
        }
        let (a_src, a_out) = (tri_area(&src), tri_area(&m3));
        assert!((a_src - a_out).abs() / a_src < 1e-3, "{a_src} vs {a_out}");
    }

    #[test]
    fn zone_is_flat_on_the_runway_and_blends_out() {
        let proj = Proj::new(51.33, 0.03);
        let a = proj.to_latlon(vec2(0.0, 0.0));
        let b = proj.to_latlon(vec2(1800.0, 0.0));
        let z = Zone { a, b, width: 45.0, ha: 100.0, hb: 118.0, grade: 0.0 };
        let (w, h) = z.at(&proj, vec2(900.0, 20.0));
        assert!((w - 1.0).abs() < 1e-6 && (h - 109.0).abs() < 0.01);
        let (w, _) = z.at(&proj, vec2(900.0, 800.0));
        assert!(w < 1e-6);
        assert!((flatten(&proj, std::slice::from_ref(&z), vec2(900.0, 0.0), 50.0) - 109.0).abs() < 0.01);
    }

    #[test]
    fn graded_airfield_is_level_with_the_runway_but_spares_hills() {
        let proj = Proj::new(51.33, 0.03);
        let a = proj.to_latlon(vec2(0.0, 0.0));
        let b = proj.to_latlon(vec2(2000.0, 0.0));
        let z = Zone { a, b, width: 45.0, ha: 100.0, hb: 104.0, grade: 500.0 };
        let zs = std::slice::from_ref(&z);
        // An apron 400 m off the centreline, 3 m below the plane: graded to it.
        assert!((flatten(&proj, zs, vec2(1000.0, 400.0), 99.0) - 102.0).abs() < 0.01);
        // A hillside 60 m above the plane: untouched.
        assert!((flatten(&proj, zs, vec2(1000.0, 400.0), 160.0) - 160.0).abs() < 0.01);
        // Far outside: untouched.
        assert!((flatten(&proj, zs, vec2(1000.0, 1500.0), 99.0) - 99.0).abs() < 0.01);
    }

    #[test]
    fn grid_height_matches_its_triangles() {
        let g = wavy(4);
        // Node values exactly at nodes, linear along edges.
        for j in 0..=4 {
            for i in 0..=4 {
                assert!((g.height(g.node_pos(i, j)) - g.node(i, j)).abs() < 1e-3);
            }
        }
        let p = g.node_pos(1, 1).lerp(g.node_pos(2, 2), 0.3);
        let want = g.node(1, 1) + (g.node(2, 2) - g.node(1, 1)) * 0.3;
        assert!((g.height(p) - want).abs() < 1e-3);
    }
}
