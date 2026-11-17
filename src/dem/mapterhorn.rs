//! Mapterhorn terrain tiles: the elevation base at every level of detail.
//!
//! `https://tiles.mapterhorn.com/{z}/{x}/{y}.webp` (TileJSON at
//! `/tilejson.json`; `Access-Control-Allow-Origin: *`, so the browser build
//! reads it too): web-Mercator XYZ tiles of **512²** pixels, **lossless WebP**
//! (VP8L) in the same Terrarium encoding as AWS Terrain Tiles
//! (`h = R·256 + G + B/256 − 32768` m). Mapterhorn merges 151 open sources:
//! Copernicus GLO-30 everywhere, and national DTMs — mostly 0.25–1 m airborne
//! LIDAR — over much of Europe, the US, Canada, Australia, New Zealand and
//! Japan ([`ATTRIBUTION_URL`]). The pyramid stops where the data does: z12
//! over GLO-30-only land, z13–z17 where a national model exists; ocean has
//! no tile (404).
//!
//! **Why it replaces Terrain Tiles as the base** (rather than refining it like
//! [`super::copdem`] / [`super::lidar`]): it is the same encoding and tile
//! scheme, global, and at the zooms Skyward reads it is at least as good
//! everywhere — GLO-30 where Terrain Tiles had SRTM / GMTED (GLO-30 is what
//! [`super::copdem`] blends in on slopes anyway), bare-earth national DTMs
//! where Terrain Tiles had none (Scotland, Wales, Norway, the Alps, ...), and
//! the same Environment Agency composite over England that
//! [`super::lidar`] reads. One source for every level also means the detail,
//! far and horizon surfaces agree. So a detail tile served by Mapterhorn is
//! **not** refined by the other [`super::Source`]s (that would put GLO-30
//! back over a LIDAR slope); they still run, as before, on tiles that fall
//! back to Terrain Tiles.
//!
//! **Tiles**: a 512² tile at zoom `z − 1` has exactly the pixel grid of the
//! four 256² tiles at zoom `z` it covers, so the terrain loader keeps its 256²
//! zoom-`z` tiles and takes a quadrant of the parent ([`parent`],
//! [`quadrant`]).
//!
//! **Fallback to Terrain Tiles** (per tile, silently after one log line):
//! `SKYWARD_NO_MAPTERHORN=1`, a flat world, unit tests, Japan ([`excluded`]),
//! no tile (404: ocean, or a hole), and network / server errors (then a
//! one-minute back-off so a dead server costs one timeout, not one per tile).
//!
//! **Japan**: GSI's DEMs are in Mapterhorn "under Survey Act approval
//! R 8JHs 131", an approval granted to Mapterhorn for its derived product.
//! Whether a further derived, cached copy (ours) needs its own approval is not
//! clear, so tiles touching Japan ([`JAPAN`]) stay on Terrain Tiles.
//!
//! **Politeness**: no published rate limit or terms; at most [`MAX_INFLIGHT`]
//! requests at a time, an identifying User-Agent, every tile (and every 404)
//! cached on disk under `cache/mapterhorn/` (pruned to [`CACHE_LIMIT`]),
//! decoded tiles in a small LRU. In the browser, the shared tile Cache
//! Storage holds the responses.

use std::sync::LazyLock;

pub const URL: &str = "https://tiles.mapterhorn.com";
/// The per-source attribution list (also `https://download.mapterhorn.com/attribution.json`).
pub const ATTRIBUTION_URL: &str = "https://mapterhorn.com/attribution";

/// Attribution (Credits screen; also README, NOTICE, web/credits.html).
pub const CREDIT: &str = "Mapterhorn (mapterhorn.com, data © its sources: Copernicus GLO-30, Environment Agency, Scottish Government, Welsh Government / NRW, swisstopo, Kartverket, USGS 3DEP, IGN, BKG / German states, BEV and Austrian states, AHN and others; see mapterhorn.com/attribution)";

/// Off: tests, a flat world, or `SKYWARD_NO_MAPTERHORN`. On in the browser too.
pub fn enabled() -> bool {
    static ON: LazyLock<bool> = LazyLock::new(|| !cfg!(test) && std::env::var_os("SKYWARD_NO_MAPTERHORN").is_none() && std::env::var_os("SKYWARD_FLAT_TERRAIN").is_none());
    *ON
}

/// Japan, generously, as (lat min, lat max, lon min, lon max) boxes: the
/// Nansei islands, Kyushu with Tsushima (west of it only sea; Geoje and Jeju
/// stay out), western Honshu up to 36° N (Ulleungdo stays out), Honshu and
/// Shikoku east of 132° E, Hokkaido, and the Izu–Ogasawara arc.
pub const JAPAN: [(f64, f64, f64, f64); 6] =
    [(24.0, 30.5, 122.6, 132.0), (30.5, 34.8, 128.9, 132.0), (34.8, 36.0, 130.8, 132.0), (30.5, 41.6, 132.0, 142.2), (41.3, 45.6, 139.3, 145.9), (20.0, 30.5, 135.5, 154.5)];

/// (lat min, lat max, lon min, lon max) of tile (`z`, `x`, `y`).
pub fn tile_bounds(z: u32, x: u32, y: u32) -> (f64, f64, f64, f64) {
    let n = (1u64 << z) as f64;
    let lat = |y: f64| (std::f64::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees();
    (lat(y as f64 + 1.0), lat(y as f64), x as f64 / n * 360.0 - 180.0, (x as f64 + 1.0) / n * 360.0 - 180.0)
}

/// True if a tile touches Japan (it is then read from Terrain Tiles).
pub fn excluded(z: u32, x: u32, y: u32) -> bool {
    let (s, n, w, e) = tile_bounds(z, x, y);
    JAPAN.iter().any(|&(bs, bn, bw, be)| s < bn && n > bs && w < be && e > bw)
}

/// The 512² Mapterhorn tile holding 256² tile (`z`, `x`, `y`), and which
/// quadrant of it (column, row: 0 or 1).
pub fn parent(z: u32, x: u32, y: u32) -> Option<((u32, u32, u32), (u32, u32))> {
    (z > 0).then_some(((z - 1, x / 2, y / 2), (x % 2, y % 2)))
}

/// Heights (m) of a Mapterhorn tile: 512², row-major.
pub fn decode(bytes: &[u8]) -> Result<Vec<f32>, String> {
    let img = image::load_from_memory_with_format(bytes, image::ImageFormat::WebP).map_err(|e| format!("Mapterhorn tile: {e}"))?.to_rgb8();
    if img.width() != 512 || img.height() != 512 {
        return Err(format!("Mapterhorn tile is {}x{}", img.width(), img.height()));
    }
    Ok(img.pixels().map(|p| p[0] as f32 * 256.0 + p[1] as f32 + p[2] as f32 / 256.0 - 32768.0).collect())
}

/// Quadrant (`qx`, `qy`) of a 512² tile: the 256² tile one zoom down.
pub fn quadrant(h: &[f32], qx: u32, qy: u32) -> Vec<f32> {
    let (x0, y0) = (qx as usize * 256, qy as usize * 256);
    let mut out = Vec::with_capacity(256 * 256);
    for j in 0..256 {
        let r = (y0 + j) * 512 + x0;
        out.extend_from_slice(&h[r..r + 256]);
    }
    out
}

fn url(z: u32, x: u32, y: u32) -> String {
    format!("{URL}/{z}/{x}/{y}.webp")
}

/// Log the first failure (then fall back quietly).
fn warn(e: &str) {
    static WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
        eprintln!("terrain: Mapterhorn: {e} (Terrain Tiles where missing)");
    }
}

// ---------------------------------------------------------------- native

/// Disk budget for `cache/mapterhorn/` (oldest pruned at startup): ~4000
/// tiles of 100–160 kB.
#[cfg(not(target_arch = "wasm32"))]
pub const CACHE_LIMIT: u64 = 500_000_000;
/// Requests in flight at once.
pub const MAX_INFLIGHT: usize = 4;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::*;
    use std::io::Read;
    use std::path::PathBuf;
    use std::sync::{Condvar, Mutex};
    use web_time::{Duration, Instant};

    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(25)).user_agent("skyward-flightsim/0.2").build());
    static FAILED_AT: Mutex<Option<Instant>> = Mutex::new(None);
    const BACKOFF: Duration = Duration::from_secs(60);
    static GATE: (Mutex<usize>, Condvar) = (Mutex::new(0), Condvar::new());

    fn root() -> PathBuf {
        static PRUNE: std::sync::Once = std::sync::Once::new();
        let r = PathBuf::from("cache").join("mapterhorn");
        PRUNE.call_once(|| {
            let r = r.clone();
            std::thread::spawn(move || crate::cog::net::prune(r, CACHE_LIMIT));
        });
        r
    }

    /// Fetch a tile: `Ok(None)` if there is none (404).
    fn fetch(z: u32, x: u32, y: u32) -> Result<Option<Vec<u8>>, String> {
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        if FAILED_AT.lock().unwrap().is_some_and(|t| t.elapsed() < BACKOFF) {
            return Err("backing off after a failure".into());
        }
        let (m, cv) = &GATE;
        let mut n = m.lock().unwrap();
        while *n >= MAX_INFLIGHT {
            n = cv.wait(n).unwrap();
        }
        *n += 1;
        drop(n);
        let r = AGENT.get(&url(z, x, y)).call();
        let out = match r {
            Ok(r) => {
                let mut b = Vec::with_capacity(200_000);
                r.into_reader().take(8 << 20).read_to_end(&mut b).map(|_| Some(b)).map_err(|e| e.to_string())
            }
            Err(ureq::Error::Status(403 | 404, _)) => Ok(None),
            Err(e) => {
                if matches!(e, ureq::Error::Transport(_) | ureq::Error::Status(429 | 500..=599, _)) {
                    *FAILED_AT.lock().unwrap() = Some(Instant::now());
                }
                Err(e.to_string())
            }
        };
        *m.lock().unwrap() -= 1;
        cv.notify_one();
        out
    }

    /// A 512² tile's heights from the disk cache or the server; `Ok(None)`:
    /// no tile there.
    fn load(z: u32, x: u32, y: u32) -> Result<Option<Vec<f32>>, String> {
        let dir = root().join(z.to_string()).join(x.to_string());
        let (path, none) = (dir.join(format!("{y}.webp")), dir.join(format!("{y}.none")));
        if none.exists() {
            return Ok(None);
        }
        if let Some(b) = crate::cog::net::read(&path) {
            if let Ok(h) = decode(&b) {
                return Ok(Some(h));
            }
        }
        match fetch(z, x, y)? {
            Some(b) => {
                let h = decode(&b)?;
                crate::cog::net::write(&path, &b);
                Ok(Some(h))
            }
            None => {
                crate::cog::net::write(&none, b"");
                Ok(None)
            }
        }
    }

    /// Decoded 512² tiles (1 MB each): the four quadrants of one are usually
    /// loaded together.
    static TILES: LazyLock<crate::cog::Store<(u32, u32, u32), Vec<f32>>> = LazyLock::new(|| crate::cog::Store::new(16, "terrain: Mapterhorn"));

    /// Heights of 256² tile (`z`, `x`, `y`) from Mapterhorn, or `None` where
    /// it does not serve it (off, Japan, no tile, failure: use Terrain Tiles).
    pub fn tile256(z: u32, x: u32, y: u32) -> Option<Vec<f32>> {
        if !enabled() || excluded(z, x, y) {
            return None;
        }
        let ((pz, px, py), (qx, qy)) = parent(z, x, y)?;
        let mut err = None;
        let t = TILES.get(&(pz, px, py), || load(pz, px, py).inspect_err(|e| err = Some(e.clone())));
        if let Some(e) = err {
            warn(&e);
        }
        t.map(|t| quadrant(&t, qx, qy))
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use native::tile256;

// ---------------------------------------------------------------- browser

/// Browser: [`tile256`] with an async fetch through the shared tile cache;
/// the four quadrant loads of one tile share one fetch and decode.
#[cfg(target_arch = "wasm32")]
pub async fn tile256(z: u32, x: u32, y: u32) -> Option<Vec<f32>> {
    use std::cell::RefCell;
    use web_time::Duration;
    enum Slot {
        Loading,
        Done(Option<std::sync::Arc<Vec<f32>>>),
    }
    thread_local! {
        static TILES: RefCell<Vec<((u32, u32, u32), Slot)>> = const { RefCell::new(Vec::new()) };
    }
    if !enabled() || excluded(z, x, y) {
        return None;
    }
    let (k, (qx, qy)) = parent(z, x, y)?;
    loop {
        let state = TILES.with(|t| {
            let mut t = t.borrow_mut();
            match t.iter().find(|e| e.0 == k) {
                Some((_, Slot::Done(v))) => Some(Some(v.clone())),
                Some((_, Slot::Loading)) => Some(None),
                None => {
                    t.push((k, Slot::Loading));
                    None
                }
            }
        });
        match state {
            Some(Some(v)) => return v.map(|t| quadrant(&t, qx, qy)),
            Some(None) => crate::platform::web::sleep(Duration::from_millis(25)).await,
            None => break,
        }
    }
    let r = match crate::platform::web::fetch_cached(&url(k.0, k.1, k.2), Duration::from_secs(25)).await {
        Ok(r) if (200..300).contains(&r.status) => decode(&r.body).map(|h| Some(std::sync::Arc::new(h))),
        Ok(r) if r.status == 404 || r.status == 403 => Ok(None),
        Ok(r) => Err(format!("HTTP {}", r.status)),
        Err(e) => Err(e),
    };
    TILES.with(|t| {
        let mut t = t.borrow_mut();
        t.retain(|e| e.0 != k);
        if let Ok(v) = &r {
            t.push((k, Slot::Done(v.clone())));
            // Keep the last few decoded tiles (1 MB each).
            let done = t.iter().filter(|e| matches!(e.1, Slot::Done(_))).count();
            if done > 8 {
                if let Some(i) = t.iter().position(|e| matches!(e.1, Slot::Done(_))) {
                    t.remove(i);
                }
            }
        }
    });
    match r {
        Ok(v) => v.map(|t| quadrant(&t, qx, qy)),
        Err(e) => {
            warn(&e);
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A lossless WebP of 512² Terrarium pixels with heights `f(col, row)`.
    fn webp(f: impl Fn(u32, u32) -> f32) -> Vec<u8> {
        let img = image::RgbImage::from_fn(512, 512, |i, j| {
            let v = ((f(i, j) + 32768.0) * 256.0).round() as u32;
            image::Rgb([(v >> 16) as u8, (v >> 8) as u8, v as u8])
        });
        let mut out = Vec::new();
        image::codecs::webp::WebPEncoder::new_lossless(&mut out).encode(img.as_raw(), 512, 512, image::ExtendedColorType::Rgb8).unwrap();
        out
    }

    #[test]
    fn decodes_terrarium_webp_and_splits_quadrants() {
        let f = |i: u32, j: u32| -20.0 + i as f32 * 3.25 + j as f32 * 0.125;
        let h = decode(&webp(f)).unwrap();
        assert_eq!(h.len(), 512 * 512);
        for (i, j) in [(0, 0), (511, 0), (0, 511), (300, 77), (511, 511)] {
            assert_eq!(h[j * 512 + i], f(i as u32, j as u32), "{i} {j}");
        }
        // Quadrant (1, 0) is the top-right 256².
        let q = quadrant(&h, 1, 0);
        assert_eq!(q.len(), 256 * 256);
        assert_eq!(q[0], f(256, 0));
        assert_eq!(q[255 * 256 + 255], f(511, 255));
        let q = quadrant(&h, 0, 1);
        assert_eq!(q[10 * 256 + 3], f(3, 266));
        // Not a 512² tile / not WebP.
        assert!(decode(b"RIFF....WEBPVP8L").is_err());
    }

    /// The quadrant of the parent holds the same ground as the 256² tile:
    /// pixel centres agree.
    #[test]
    fn quadrant_pixels_match_the_256_tile() {
        let (z, x, y) = (12u32, 2047u32, 1361u32);
        let ((pz, px, py), (qx, qy)) = parent(z, x, y).unwrap();
        assert_eq!((pz, px, py, qx, qy), (11, 1023, 680, 1, 1));
        // Local pixel (i, j) of the 256² tile = pixel (qx·256 + i, qy·256 + j) of the 512².
        let (i, j) = (17i64, 200i64);
        let a = crate::dem::pixel_latlon(z, x as i64 * 256 + i, y as i64 * 256 + j);
        let (gx, gy) = (px as i64 * 512 + qx as i64 * 256 + i, py as i64 * 512 + qy as i64 * 256 + j);
        // A 512² zoom-11 grid is a 256² zoom-12 grid.
        let b = crate::dem::pixel_latlon(pz + 1, gx, gy);
        assert_eq!(a, b);
        assert!(parent(0, 0, 0).is_none());
    }

    #[test]
    fn japan_is_excluded_and_its_neighbours_are_not() {
        let at = |lat: f64, lon: f64, z: u32| {
            let (fx, fy) = crate::osm::TileId::frac(lat, lon, z);
            excluded(z, fx as u32, fy as u32)
        };
        for (lat, lon) in [(35.68, 139.77), (34.69, 135.50), (43.06, 141.35), (26.21, 127.68), (33.59, 130.40), (34.20, 129.30), (27.09, 142.19), (35.36, 138.73)] {
            assert!(at(lat, lon, 12), "{lat} {lon} should be Japan");
        }
        // Seoul, Busan, Jeju, Ulleungdo, Taipei, Vladivostok, Sakhalin, London, Glencoe.
        for (lat, lon) in [(37.57, 126.98), (35.18, 129.08), (33.40, 126.55), (37.50, 130.87), (25.03, 121.56), (43.12, 131.89), (46.96, 142.73), (51.5, -0.1), (56.68, -5.0)] {
            assert!(!at(lat, lon, 12), "{lat} {lon} should not be Japan");
        }
    }

    #[test]
    #[cfg(not(target_arch = "wasm32"))]
    fn off_in_tests() {
        assert!(!enabled());
        assert!(tile256(12, 2047, 1361).is_none());
    }

    /// Live data (network; `cargo test --release -- --ignored live_`): a
    /// real tile decodes to plausible heights (the hills around Glencoe and
    /// Loch Leven, ~1100 m at this zoom).
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_mapterhorn_tile() {
        let b = ureq::get(&url(11, 995, 630)).call().unwrap();
        let mut v = Vec::new();
        std::io::Read::read_to_end(&mut b.into_reader(), &mut v).unwrap();
        let h = decode(&v).unwrap();
        let mx = h.iter().copied().fold(f32::MIN, f32::max);
        assert!(mx > 900.0 && mx < 1400.0, "{mx}");
    }
}
