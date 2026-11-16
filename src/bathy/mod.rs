//! Bathymetry: how deep the sea is, so the water shader can colour it —
//! turquoise over sand in clear tropical shallows, muddy green-brown in
//! turbid estuaries, dark blue over deep water, sandbanks and dredged
//! channels showing through, surf where swell breaks on the shallows.
//!
//! **Data** (both range-free single GETs of float GeoTIFFs, CORS `*`, no key):
//! * Europe: the EMODnet Bathymetry DTM (1/16′ ≈ 115 m, CC BY 4.0) from its
//!   WCS ([`source::emodnet_url`]); its gaps (exactly 0.0) are filled from ...
//! * everywhere: NOAA NCEI's global DEM mosaic image service — ETOPO 2022
//!   15″ (whose bathymetry is GEBCO's) plus NOAA's high-resolution coastal
//!   DEMs around the US ([`source::noaa_url`]). GEBCO itself has no
//!   subsettable, CORS-enabled service (only bulk downloads and a picture WMS).
//!
//! **Flow**: [`set_focus`] (from `World::update`) notes where the view is;
//! a background job ([`Job`]: a thread natively, an async task in the
//! browser) fetches the depth tiles ([`TileKey`]: fixed lat / lon squares,
//! two levels) around it, cached under `cache/bathy/` (native) or the
//! browser's tile cache, and resamples them into two regional images
//! ([`Region`]: ±48 km at ≈ 94 m, ±256 km at ≈ 500 m) — depth and a "this is
//! sea" weight. The renderer ([`gpu`]) uploads them to a two-layer texture and
//! the water shader samples it by position. OSM water polygons still decide
//! where water is; the depth only colours it. Land near the sea counts as
//! very shallow sea for a texel (the shore shoals smoothly), and up to
//! ~4.5 km from it as a flat or a lagoon (tidal flats and reef flats the
//! coarse DEMs call land, atoll lagoons they get wrong); rivers and lakes
//! further inland, or above sea level, get no depth: the plain look.
//!
//! **Degrades**: offline with nothing cached, a failed fetch, unit tests or
//! `SKYWARD_NO_BATHY=1` → no depth; the water looks as before.

pub mod gpu;
pub(crate) mod source;

use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

/// Off in unit tests and with `SKYWARD_NO_BATHY`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| !cfg!(test) && std::env::var_os("SKYWARD_NO_BATHY").is_none());
    *ON
}

/// Attribution (Credits screen; also README, NOTICE, web/credits.html).
pub const CREDIT: &str = "EMODnet Bathymetry Consortium (2024): EMODnet Digital Bathymetry (DTM 2024), doi:10.12770/cf51df64-56f9-4a99-b1aa-36b8d7b743a1 — CC BY 4.0 · NOAA NCEI (2022): ETOPO 2022 15 Arc-Second Global Relief Model, doi:10.25921/fd45-gt74, with GEBCO Grid bathymetry (GEBCO Compilation Group) and NOAA coastal DEMs — not for navigation";

// ---------------------------------------------------------------- tiles

/// One pyramid level: square lat / lon tiles of `tile_deg`, `px`² samples
/// each, resampled into a region image of ±`half_m` around the view.
#[derive(Clone, Copy, Debug)]
pub struct Level {
    pub tile_deg: f64,
    pub px: u32,
    pub half_m: f64,
    /// Land texels within this many texels of the sea count as shore ...
    pub shore: i32,
    /// ... and within this many as flats / lagoons ([`classify`]).
    pub flats: i32,
    /// Patches of "sea" smaller than this many texels, not touching the
    /// region's edge, are dropped: pits and polders below sea level inland
    /// (their lakes keep the plain look).
    pub min_sea: usize,
}

/// Fine (≈ 94 m texels, EMODnet's own 1/16′ grid in Europe) and coarse.
pub const LEVELS: [Level; 2] =
    [Level { tile_deg: 0.25, px: 240, half_m: 48_000.0, shore: 1, flats: 48, min_sea: 200 }, Level { tile_deg: 2.0, px: 240, half_m: 256_000.0, shore: 1, flats: 8, min_sea: 8 }];

/// Region image size (texels per side).
pub const RES: u32 = if cfg!(target_arch = "wasm32") { 512 } else { 1024 };

/// A depth tile: `level`, and its south-west corner in tile units (lon / lat
/// ÷ the level's `tile_deg`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileKey {
    pub level: u8,
    pub ix: i32,
    pub iy: i32,
}

impl TileKey {
    pub fn deg(&self) -> f64 {
        LEVELS[self.level as usize].tile_deg
    }
    /// (west, south, east, north) in degrees.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let d = self.deg();
        (self.ix as f64 * d, self.iy as f64 * d, (self.ix + 1) as f64 * d, (self.iy + 1) as f64 * d)
    }
}

/// Elevations (m, + up; NaN = no data), `n`² samples, row 0 = north, cell
/// centres (the request's bbox is the outer edge of the grid).
#[derive(Clone, Debug, PartialEq)]
pub struct Grid {
    pub n: u32,
    pub elev: Vec<f32>,
}

impl Grid {
    /// Bilinear elevation at fractional position (`fx`, `fy`) in 0..1 across
    /// the tile (x east, y south); NaN where any corner has no data.
    pub fn sample(&self, fx: f64, fy: f64) -> f32 {
        let n = self.n as i64;
        let x = (fx * n as f64 - 0.5).clamp(0.0, (n - 1) as f64);
        let y = (fy * n as f64 - 0.5).clamp(0.0, (n - 1) as f64);
        let (x0, y0) = (x.floor() as i64, y.floor() as i64);
        let (x1, y1) = ((x0 + 1).min(n - 1), (y0 + 1).min(n - 1));
        let (tx, ty) = ((x - x0 as f64) as f32, (y - y0 as f64) as f32);
        let at = |x: i64, y: i64| self.elev[(y * n + x) as usize];
        let top = at(x0, y0) * (1.0 - tx) + at(x1, y0) * tx;
        let bot = at(x0, y1) * (1.0 - tx) + at(x1, y1) * tx;
        top * (1.0 - ty) + bot * ty
    }
}

// ---------------------------------------------------------------- regions

/// A regional image's footprint: centre and extent in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Region {
    pub level: u8,
    pub lat: f64,
    pub lon: f64,
    /// Half extents (degrees).
    pub half_lat: f64,
    pub half_lon: f64,
}

impl Region {
    /// The region of `level` centred near (`lat`, `lon`), snapped to a coarse
    /// grid (an eighth of its size) so nearby views share it.
    pub fn around(level: u8, lat: f64, lon: f64) -> Region {
        let lv = LEVELS[level as usize];
        let half_lat = lv.half_m / crate::osm::M_PER_DEG_LAT;
        let snap = half_lat / 4.0;
        let lat = ((lat / snap).round() * snap).clamp(-85.0, 85.0);
        let half_lon = (lv.half_m / (crate::osm::M_PER_DEG_LAT * lat.to_radians().cos().max(0.05))).min(179.0);
        let lon = (lon / snap).round() * snap;
        Region { level, lat, lon, half_lat, half_lon }
    }
    /// (west, south, east, north).
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        (self.lon - self.half_lon, self.lat - self.half_lat, self.lon + self.half_lon, self.lat + self.half_lat)
    }
    /// True if (`lat`, `lon`) has moved far enough from the centre to want a
    /// new region (a third of the half size).
    pub fn stale_for(&self, lat: f64, lon: f64) -> bool {
        ((lat - self.lat) / self.half_lat).abs() > 0.33 || ((lon - self.lon) / self.half_lon).abs() > 0.33
    }
    /// The tiles the region needs.
    pub fn tiles(&self) -> Vec<TileKey> {
        let d = LEVELS[self.level as usize].tile_deg;
        let (w, s, e, n) = self.bounds();
        let mut v = Vec::new();
        for iy in (s / d).floor() as i32..=((n / d).ceil() as i32 - 1) {
            if !(-90.0..90.0).contains(&(iy as f64 * d)) {
                continue;
            }
            for ix in (w / d).floor() as i32..=((e / d).ceil() as i32 - 1) {
                v.push(TileKey { level: self.level, ix, iy });
            }
        }
        v
    }
}

/// A finished region image: per texel (depth m × weight, weight), row 0 =
/// north, as f16 bits (the GPU texture's `Rg16Float`).
pub struct Image {
    pub region: Region,
    pub res: u32,
    pub texels: Vec<[u16; 2]>,
    /// True if every tile arrived.
    pub complete: bool,
}

/// Below this elevation (m) a texel is sea (EMODnet: lowest astronomical
/// tide; ETOPO: mean sea level). Drying banks just above it are reached by
/// the shore band.
pub const SEA_BELOW: f32 = 0.0;

/// Resample `tiles` (None = missing) into the region image.
pub fn build_image(region: Region, res: u32, tile_at: impl Fn(TileKey) -> Option<Arc<Grid>>) -> Image {
    let lv = LEVELS[region.level as usize];
    let (w, _, _, n) = region.bounds();
    let (dlon, dlat) = (2.0 * region.half_lon / res as f64, 2.0 * region.half_lat / res as f64);
    let r = res as usize;
    let mut elev = vec![f32::NAN; r * r];
    let mut cur: Option<(TileKey, Option<Arc<Grid>>)> = None;
    for j in 0..r {
        let lat = n - (j as f64 + 0.5) * dlat;
        for i in 0..r {
            let lon = w + (i as f64 + 0.5) * dlon;
            let lonw = (lon + 180.0).rem_euclid(360.0) - 180.0;
            let key = TileKey { level: region.level, ix: (lonw / lv.tile_deg).floor() as i32, iy: (lat / lv.tile_deg).floor() as i32 };
            if cur.as_ref().is_none_or(|c| c.0 != key) {
                cur = Some((key, tile_at(key)));
            }
            if let Some((k, Some(g))) = &cur {
                let (tw, _, _, tn) = k.bounds();
                elev[j * r + i] = g.sample((lonw - tw) / lv.tile_deg, (tn - lat) / lv.tile_deg);
            }
        }
    }
    let texels = classify(&elev, r, lv.shore, lv.flats, lv.min_sea);
    Image { region, res, texels, complete: true }
}

/// Land near the sea under OSM water is either a tidal / reef flat the
/// coarse DEMs call land (below [`FLAT_BELOW`] m: drawn [`FLAT_DEPTH`] m
/// deep), or the DEM is simply wrong there (atoll lagoons in ETOPO are tens
/// of metres "high"): drawn [`UNSURE_DEPTH`] m deep.
pub const FLAT_BELOW: f32 = 2.5;
pub const FLAT_DEPTH: f32 = 0.8;
pub const UNSURE_DEPTH: f32 = 8.0;
/// Depth (m) of the shore band (the coast shoals to nearly nothing).
pub const SHORE_DEPTH: f32 = 0.3;

/// Per texel (depth × weight, weight) from elevations: sea is weight 1 at
/// its depth; land within `shore` texels of the sea is [`SHORE_DEPTH`],
/// further within `flats` texels a flat or an unsure lagoon (above); other
/// land and no data are weight 0 (rivers and lakes inland keep their plain
/// look).
pub fn classify(elev: &[f32], r: usize, shore: i32, flats: i32, min_sea: usize) -> Vec<[u16; 2]> {
    let is_sea = sea_mask(elev, r, min_sea);
    let mut out = vec![[0u16; 2]; r * r];
    // Distance (texels, chessboard) to the nearest sea texel, up to the wider band.
    let band = shore.max(flats).clamp(0, 254) as u8;
    let mut near = vec![u8::MAX; r * r];
    let mut queue = std::collections::VecDeque::new();
    for (k, &s) in is_sea.iter().enumerate() {
        if s {
            near[k] = 0;
            queue.push_back(k);
        }
    }
    while let Some(k) = queue.pop_front() {
        let d = near[k];
        if d >= band {
            continue;
        }
        let (j, i) = ((k / r) as i64, (k % r) as i64);
        for (dj, di) in [(-1i64, 0i64), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)] {
            let (jj, ii) = (j + dj, i + di);
            if jj >= 0 && ii >= 0 && (jj as usize) < r && (ii as usize) < r {
                let n = jj as usize * r + ii as usize;
                if near[n] == u8::MAX {
                    near[n] = d + 1;
                    queue.push_back(n);
                }
            }
        }
    }
    let one = f16_bits(1.0);
    for k in 0..r * r {
        let e = elev[k];
        if is_sea[k] {
            out[k] = [f16_bits(-e), one];
        } else if e.is_nan() || near[k] == u8::MAX {
        } else if (near[k] as i32) <= shore {
            out[k] = [f16_bits(SHORE_DEPTH), one];
        } else if (near[k] as i32) <= flats {
            out[k] = [f16_bits(if e < FLAT_BELOW { FLAT_DEPTH } else { UNSURE_DEPTH }), one];
        }
    }
    out
}

/// Texels below [`SEA_BELOW`], without the small patches that do not touch
/// the edge (4-connected).
fn sea_mask(elev: &[f32], r: usize, min_sea: usize) -> Vec<bool> {
    let mut sea: Vec<bool> = elev.iter().map(|&e| e < SEA_BELOW).collect();
    let mut seen = vec![false; r * r];
    let mut stack = Vec::new();
    let mut patch = Vec::new();
    for start in 0..r * r {
        if !sea[start] || seen[start] {
            continue;
        }
        seen[start] = true;
        stack.push(start);
        patch.clear();
        let mut edge = false;
        while let Some(k) = stack.pop() {
            patch.push(k);
            let (j, i) = (k / r, k % r);
            edge |= i == 0 || j == 0 || i == r - 1 || j == r - 1;
            let mut go = |n: usize| {
                if sea[n] && !seen[n] {
                    seen[n] = true;
                    stack.push(n);
                }
            };
            if i > 0 {
                go(k - 1);
            }
            if i + 1 < r {
                go(k + 1);
            }
            if j > 0 {
                go(k - r);
            }
            if j + 1 < r {
                go(k + r);
            }
        }
        if !edge && patch.len() < min_sea {
            for &k in &patch {
                sea[k] = false;
            }
        }
    }
    sea
}

/// IEEE half-precision bits of `v` (round to nearest; overflow → max, below
/// the normal range (6e-5) → 0).
pub fn f16_bits(v: f32) -> u16 {
    let b = v.to_bits();
    let sign = ((b >> 16) & 0x8000) as u16;
    let exp = ((b >> 23) & 0xff) as i32 - 127 + 15;
    let man = b & 0x7f_ffff;
    if v.is_nan() {
        return 0x7e00;
    }
    if exp >= 31 {
        return sign | 0x7bff;
    }
    if exp <= 0 {
        return sign;
    }
    let r = (man + 0x0fff + ((man >> 13) & 1)) >> 13;
    // (A mantissa carry rolls into the exponent, as it should.)
    (sign as u32 | (((exp as u32) << 10) + r)).min(sign as u32 | 0x7bff) as u16
}

/// f32 from half bits (tests, debug dumps).
pub fn f16_value(h: u16) -> f32 {
    let sign = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((h >> 10) & 0x1f) as i32;
    let man = (h & 0x3ff) as f32;
    sign * match exp {
        0 => man * 2f32.powi(-24),
        31 => f32::INFINITY,
        e => (1.0 + man / 1024.0) * 2f32.powi(e - 15),
    }
}

/// How clear the sea water is, 0 (turbid temperate coast: green-brown, the
/// bottom shows only in the shallowest metre) .. 1 (clear tropical sea:
/// turquoise over sand down to ~20 m). A climate guess from the place: warm
/// low-latitude seas are clear, the Mediterranean fairly clear, shelf seas
/// further from the equator turbid.
pub fn clarity(lat: f64, lon: f64) -> f32 {
    let a = lat.abs();
    let tropic = 1.0 - smooth(22.0, 34.0, a);
    // The Mediterranean (and its clear, sediment-poor water).
    let med = if (30.0..46.0).contains(&lat) && (-5.5..36.5).contains(&lon) { 0.6 } else { 0.0 };
    tropic.max(med) as f32
}

fn smooth(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

// ---------------------------------------------------------------- driver

/// Shared state of the background work.
#[derive(Default)]
struct State {
    /// Where the view is (lat, lon).
    focus: Option<(f64, f64)>,
    /// A job is running.
    busy: bool,
    /// Regions built (or being built) per level, and when an incomplete one
    /// may be retried.
    have: [Option<(Region, Option<Instant>)>; 2],
    ready: Vec<Image>,
}

static STATE: std::sync::LazyLock<Mutex<State>> = std::sync::LazyLock::new(|| Mutex::new(State::default()));

/// Retry regions with missing tiles after this long.
const RETRY: Duration = Duration::from_secs(60);

/// Levels that want a new region for a view at (`lat`, `lon`).
fn wanted(st: &State, lat: f64, lon: f64) -> Vec<Region> {
    let mut v = Vec::new();
    for (l, have) in st.have.iter().enumerate() {
        let stale = match have {
            None => true,
            Some((r, retry)) => r.stale_for(lat, lon) || retry.is_some_and(|t| Instant::now() >= t),
        };
        if stale {
            v.push(Region::around(l as u8, lat, lon));
        }
    }
    v
}

/// The view is at (`lat`, `lon`): start fetching / building what it needs.
/// Cheap; call every frame.
pub fn set_focus(lat: f64, lon: f64) {
    if !enabled() {
        return;
    }
    let mut st = STATE.lock().unwrap();
    st.focus = Some((lat, lon));
    if st.busy {
        return;
    }
    let want = wanted(&st, lat, lon);
    if want.is_empty() {
        return;
    }
    for r in &want {
        st.have[r.level as usize] = Some((*r, None));
    }
    st.busy = true;
    drop(st);
    Job { regions: want }.start();
}

/// True when nothing is being fetched or built for the current view (shots
/// wait for this).
pub fn idle() -> bool {
    if !enabled() {
        return true;
    }
    let st = STATE.lock().unwrap();
    !st.busy && st.focus.is_none_or(|(lat, lon)| wanted(&st, lat, lon).is_empty())
}

/// Finished images (the renderer uploads them).
pub fn take_ready() -> Vec<Image> {
    if !enabled() {
        return Vec::new();
    }
    std::mem::take(&mut STATE.lock().unwrap().ready)
}

/// A unit of background work: build these regions.
struct Job {
    regions: Vec<Region>,
}

impl Job {
    fn start(self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            if !crate::platform::spawn("bathy", move || self.run()) {
                STATE.lock().unwrap().busy = false;
            }
        }
        #[cfg(target_arch = "wasm32")]
        crate::platform::web::spawn_local(async move { self.run_web().await });
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn run(self) {
        for region in self.regions {
            let keys = region.tiles();
            // A few downloads at once.
            let got: Mutex<std::collections::HashMap<TileKey, Option<Arc<Grid>>>> = Mutex::new(Default::default());
            let next = std::sync::atomic::AtomicUsize::new(0);
            std::thread::scope(|s| {
                for _ in 0..4 {
                    s.spawn(|| {
                        loop {
                            let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some(&k) = keys.get(i) else { break };
                            let g = source::tile(k);
                            got.lock().unwrap().insert(k, g);
                        }
                    });
                }
            });
            let got = got.into_inner().unwrap();
            self::finish(region, &got);
        }
        STATE.lock().unwrap().busy = false;
    }

    #[cfg(target_arch = "wasm32")]
    async fn run_web(self) {
        for region in self.regions {
            let mut got = std::collections::HashMap::new();
            for k in region.tiles() {
                let g = source::tile_web(k).await;
                got.insert(k, g);
            }
            self::finish(region, &got);
        }
        STATE.lock().unwrap().busy = false;
    }
}

/// Build a region from its fetched tiles and hand it to the renderer.
fn finish(region: Region, got: &std::collections::HashMap<TileKey, Option<Arc<Grid>>>) {
    let t0 = Instant::now();
    let complete = got.values().all(|g| g.is_some());
    let any = got.values().any(|g| g.is_some());
    let mut img = build_image(region, RES, |k| got.get(&k).cloned().flatten());
    img.complete = complete;
    if std::env::var_os("SKYWARD_BATHY_DEBUG").is_some() {
        eprintln!(
            "bathy level {}: {}/{} tiles, image {:.0} ms ({:.3}, {:.3})",
            region.level,
            got.values().filter(|g| g.is_some()).count(),
            got.len(),
            t0.elapsed().as_secs_f32() * 1e3,
            region.lat,
            region.lon
        );
        #[cfg(not(target_arch = "wasm32"))]
        source::debug_dump(&img);
    }
    let mut st = STATE.lock().unwrap();
    if let Some(h) = st.have[region.level as usize].as_mut().filter(|h| h.0 == region) {
        h.1 = (!complete).then(|| Instant::now() + RETRY);
    }
    if any {
        st.ready.retain(|i| i.region.level != region.level);
        st.ready.push(img);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn half_floats_round_trip() {
        for v in [0.0f32, 1.0, 0.5, 3.25, 12.0, 147.5, 2048.0, 10_994.0] {
            let h = f16_bits(v);
            let back = f16_value(h);
            assert!((back - v).abs() <= v.abs() * 1e-3 + 1e-6, "{v} -> {back}");
        }
        assert_eq!(f16_bits(1.0), 0x3c00);
        assert_eq!(f16_bits(-2.0), 0xc000);
        assert_eq!(f16_bits(65504.0), 0x7bff);
        assert_eq!(f16_bits(1e9), 0x7bff);
        assert_eq!(f16_bits(1e-6), 0);
    }

    #[test]
    fn regions_cover_their_tiles_and_go_stale() {
        let r = Region::around(0, 51.5, 0.9);
        let (w, s, e, n) = r.bounds();
        // ±48 km: ≈ 0.43° north-south, more east-west at 51°.
        assert!((n - s - 2.0 * 48_000.0 / 111_320.0).abs() < 1e-9);
        assert!(e - w > (n - s) * 1.5);
        let tiles = r.tiles();
        for t in &tiles {
            let (tw, ts, te, tn) = t.bounds();
            assert!(te > w && tw < e && tn > s && ts < n, "{t:?}");
        }
        // Every corner of the region is in some tile.
        for (lat, lon) in [(s, w), (s, e - 1e-9), (n - 1e-9, w), (n - 1e-9, e - 1e-9)] {
            assert!(tiles.iter().any(|t| {
                let (tw, ts, te, tn) = t.bounds();
                (tw..te).contains(&lon) && (ts..tn).contains(&lat)
            }));
        }
        assert!(!r.stale_for(51.55, 0.95));
        assert!(r.stale_for(51.75, 0.9));
        // Nearby views share a region (snapping).
        assert_eq!(Region::around(0, 51.5, 0.9), Region::around(0, 51.5001, 0.9002));
        // The coarse level is the same idea, bigger.
        let c = Region::around(1, 51.5, 0.9);
        assert!(c.half_lat > r.half_lat * 5.0);
        assert!(c.tiles().iter().all(|t| t.level == 1));
    }

    #[test]
    fn grid_samples_cell_centres() {
        // 2×2: north row 0, 10; south row 20, 30.
        let g = Grid { n: 2, elev: vec![0.0, 10.0, 20.0, 30.0] };
        assert_eq!(g.sample(0.25, 0.25), 0.0);
        assert_eq!(g.sample(0.75, 0.75), 30.0);
        assert_eq!(g.sample(0.5, 0.25), 5.0);
        assert_eq!(g.sample(0.5, 0.5), 15.0);
        // Clamped at the edges.
        assert_eq!(g.sample(0.0, 0.0), 0.0);
        assert!(Grid { n: 2, elev: vec![f32::NAN, 1.0, 1.0, 1.0] }.sample(0.5, 0.5).is_nan());
    }

    #[test]
    fn classify_sea_shore_and_land() {
        // 6 texels in a row, west to east: deep sea, shallow sea, beach, land, land, no data.
        let r = 6;
        let mut elev = vec![50.0f32; r * r];
        for j in 0..r {
            elev[j * r..j * r + 6].copy_from_slice(&[-40.0, -2.0, 1.0, 20.0, 30.0, f32::NAN]);
        }
        let near = |a: f32, b: f32| (a - b).abs() < 2e-3 * b.max(1.0);
        let t = classify(&elev, r, 1, 0, 0);
        let row: Vec<(f32, f32)> = t[..6].iter().map(|p| (f16_value(p[0]), f16_value(p[1]))).collect();
        assert_eq!(row[0], (40.0, 1.0));
        assert_eq!(row[1], (2.0, 1.0));
        // The beach next to the sea: very shallow (the shore shoals smoothly).
        assert!(near(row[2].0, SHORE_DEPTH) && row[2].1 == 1.0);
        // Inland and no data: not sea (rivers / lakes keep their plain look).
        assert_eq!(row[3], (0.0, 0.0));
        assert_eq!(row[5], (0.0, 0.0));
        // A wider shore band reaches further.
        let t2 = classify(&elev, r, 2, 0, 0);
        assert_eq!(f16_value(t2[3][1]), 1.0);
        assert_eq!(f16_value(t2[4][1]), 0.0);
        // Within the flats band: low land is a flat, higher "land" (under
        // OSM water, a DEM error) a lagoon; beyond it nothing.
        let mut e2 = vec![50.0f32; r * r];
        for j in 0..r {
            e2[j * r..j * r + 6].copy_from_slice(&[-40.0, 1.0, 1.0, 30.0, 1.0, 1.0]);
        }
        let t3 = classify(&e2, r, 1, 4, 0);
        assert!(near(f16_value(t3[1][0]), SHORE_DEPTH));
        assert!(near(f16_value(t3[2][0]), FLAT_DEPTH));
        assert!(near(f16_value(t3[3][0]), UNSURE_DEPTH));
        assert!(near(f16_value(t3[4][0]), FLAT_DEPTH));
        assert_eq!(t3[5][1], 0);
        // A pit below sea level inland (not touching the edge) is no sea ...
        let r = 8;
        let mut elev = vec![10.0f32; r * r];
        elev[3 * r + 3] = -2.0;
        elev[3 * r + 4] = -2.0;
        assert!(classify(&elev, r, 1, 0, 5).iter().all(|p| p[1] == 0));
        // ... unless big enough, or open to the edge.
        assert_eq!(f16_value(classify(&elev, r, 1, 0, 2)[3 * r + 3][0]), 2.0);
        elev[3 * r] = -2.0;
        elev[3 * r + 1] = -2.0;
        elev[3 * r + 2] = -2.0;
        assert_eq!(f16_value(classify(&elev, r, 1, 0, 50)[3 * r + 3][0]), 2.0);
    }

    #[test]
    fn image_resamples_tiles() {
        // One fine tile (0.25°) at 51.25..51.5 N, 0.75..1.0 E: sea 10 m deep
        // in its west half, land in its east half.
        let key = TileKey { level: 0, ix: 3, iy: 205 };
        assert_eq!(key.bounds(), (0.75, 51.25, 1.0, 51.5));
        let n = 8;
        let elev = (0..n * n).map(|k| if k % n < n / 2 { -10.0 } else { 15.0 }).collect();
        let tile = Arc::new(Grid { n: n as u32, elev });
        let region = Region { level: 0, lat: 51.375, lon: 0.875, half_lat: 0.1, half_lon: 0.1 };
        let img = build_image(region, 16, |k| (k == key).then(|| tile.clone()));
        let at = |i: usize, j: usize| {
            let p = img.texels[j * 16 + i];
            (f16_value(p[0]), f16_value(p[1]))
        };
        assert_eq!(at(1, 8), (10.0, 1.0));
        // Land near the sea (within the flats band): an "unsure" lagoon depth.
        assert_eq!(at(14, 8), (UNSURE_DEPTH, 1.0));
        // No tile → nothing.
        let none = build_image(region, 16, |_| None);
        assert!(none.texels.iter().all(|p| p[1] == 0));
    }

    #[test]
    fn clarity_by_climate() {
        // Maldives, Bahamas: clear; the Thames estuary: turbid; the Med: between.
        assert!(clarity(4.2, 73.5) > 0.95);
        assert!(clarity(24.5, -77.5) > 0.85);
        assert_eq!(clarity(51.5, 1.0), 0.0);
        let med = clarity(37.0, 25.0);
        assert!(med > 0.3 && med < 0.9);
    }

    #[test]
    fn off_in_tests() {
        assert!(!enabled());
        assert!(idle());
        assert!(take_ready().is_empty());
    }
}
