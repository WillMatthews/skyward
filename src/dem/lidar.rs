//! Environment Agency National LIDAR Programme DTM (England, bare earth).
//!
//! The "LIDAR Composite Digital Terrain Model (DTM) 1m" on the Defra Data
//! Services Platform, read on demand from its OGC WCS 2.0.1 endpoint
//! ([`WCS`], GeoServer; no key, `Access-Control-Allow-Origin: *`): one
//! GetCoverage per 10 km National Grid square ([`SQ`]), in the coverage's own
//! CRS (EPSG:27700), scaled server-side to [`PX`]² pixels (10 m), DEFLATE
//! float GeoTIFF of ODN heights (≈ 1.5–3 MB). Native requests answer in
//! ~1–2 s, but the service now and then stalls (a request over empty mosaic,
//! open sea, can hang for minutes): squares are only asked for over England
//! ([`ENGLAND`]) and land, at most two at a time, with a 25 s timeout and one
//! retry.
//!
//! **Use** ([`Lidar::refine`]): each detail-tile pixel's centre (and a 3×3
//! sub-grid over its ≈ 24 m footprint, a box filter of the 10 m samples) is
//! taken to the National Grid ([`super::osgb`]) and sampled bilinearly. The
//! DTM replaces the earlier heights (Terrain Tiles / GLO-30) wherever it
//! has data, ramping in over [`RAMP`] pixels from the edge of its coverage
//! (no data, outside England) so the seam is smooth; the ramp is computed on
//! a margin past the tile edge, so neighbouring tiles agree. Runway
//! flattening runs on the result.
//!
//! The terrain mesh is one vertex per DEM pixel (≈ 24 m at 51.5° N), so what
//! the DTM brings at this level is bare-earth accuracy (no roofs or
//! canopy, embankments, cuttings, river terraces, chalk escarpments placed
//! to a few metres), not 1 m detail.
//!
//! **Cache**: the server's GeoTIFFs under `cache/lidar/` (pruned to
//! [`CACHE_LIMIT`]), decoded squares (4 MB each) in a small LRU.
//! **Off**: the browser (no detail-source hook there yet), tests, a flat
//! world, or `SKYWARD_NO_LIDAR=1`.

use super::DetailTile;
use crate::cog::Store;
use std::sync::{Arc, LazyLock};

/// The coverage's WCS endpoint and id (dataset 13787b9a-…: the current
/// composite, 2022 onwards).
const WCS: &str = "https://environment.data.gov.uk/geoservices/datasets/13787b9a-26a4-4775-8523-806d13af58fc/wcs";
const COVERAGE: &str = "13787b9a-26a4-4775-8523-806d13af58fc__Lidar_Composite_Elevation_DTM_1m";

/// Attribution (Credits screen; also README, NOTICE, web/credits.html), as the
/// Open Government Licence v3.0 and the dataset's terms ask.
pub const CREDIT: &str = "Environment Agency LIDAR Composite DTM: © Environment Agency copyright and/or database right 2022. All rights reserved. Contains public sector information licensed under the Open Government Licence v3.0 (desktop version)";

/// Square side (m) and pixels per side (10 m pixels).
pub const SQ: f64 = 10_000.0;
pub const PX: usize = 1000;
const RES: f64 = SQ / PX as f64;
/// Width (detail pixels) of the ramp from the edge of coverage to full DTM.
pub const RAMP: usize = 8;

/// Off: tests, the browser, a flat world, or `SKYWARD_NO_LIDAR`.
pub fn enabled() -> bool {
    static ON: LazyLock<bool> =
        LazyLock::new(|| !cfg!(test) && !cfg!(target_arch = "wasm32") && std::env::var_os("SKYWARD_NO_LIDAR").is_none() && std::env::var_os("SKYWARD_FLAT_TERRAIN").is_none());
    *ON
}

/// England, generously (lon, lat): the Scottish and Welsh borders, and lines
/// out at sea around the coast (no Isle of Man, Channel Islands or France).
/// The DTM itself has no data offshore; this only keeps requests to England.
const ENGLAND: [(f64, f64); 36] = [
    (-1.90, 55.85),
    (-2.04, 55.81),
    (-2.22, 55.64),
    (-2.33, 55.47),
    (-2.68, 55.25),
    (-2.86, 55.10),
    (-3.05, 54.99),
    (-3.50, 54.90),
    (-3.70, 54.60),
    (-3.60, 54.00),
    (-3.20, 53.45),
    (-3.08, 53.25),
    (-2.95, 53.17),
    (-2.73, 52.98),
    (-3.12, 52.82),
    (-3.02, 52.60),
    (-3.12, 52.40),
    (-3.08, 52.10),
    (-2.95, 51.90),
    (-2.66, 51.82),
    (-2.66, 51.62),
    (-2.95, 51.45),
    (-3.40, 51.25),
    (-4.90, 51.25),
    (-6.60, 50.00),
    (-6.30, 49.80),
    (-5.00, 49.80),
    (-1.00, 50.50),
    (1.20, 50.85),
    (1.55, 51.10),
    (1.90, 51.40),
    (1.95, 52.50),
    (1.80, 53.00),
    (0.50, 53.50),
    (0.30, 54.20),
    (-1.10, 55.30),
];

/// [`ENGLAND`] on the National Grid.
static ENGLAND_EN: LazyLock<Vec<(f64, f64)>> = LazyLock::new(|| ENGLAND.iter().map(|&(lo, la)| super::osgb::from_wgs84(la, lo)).collect());

/// Even-odd point-in-polygon.
fn inside(poly: &[(f64, f64)], x: f64, y: f64) -> bool {
    let mut c = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let ((xi, yi), (xj, yj)) = (poly[i], poly[j]);
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            c = !c;
        }
        j = i;
    }
    c
}

/// True if a National Grid point is in England (by [`ENGLAND`]).
pub fn in_england(e: f64, n: f64) -> bool {
    // Bounding box first (most of the world).
    (80_000.0..660_000.0).contains(&e) && (0.0..660_000.0).contains(&n) && inside(&ENGLAND_EN, e, n)
}

/// One 10 km square: [`PX`]² heights, north row first; NaN = no data.
pub type Square = Vec<f32>;

/// Where squares come from: the service (live), or fixed data (tests).
pub trait Squares {
    /// Square (`sx`, `sy`) (south-west corner `sx·SQ`, `sy·SQ`); `None`: no data.
    fn square(&mut self, sx: i32, sy: i32) -> Option<Arc<Square>>;
}

/// Decode a GetCoverage answer for square (`sx`, `sy`): a float GeoTIFF of
/// [`PX`]² pixels whose georeference (if given) is the square's.
pub fn decode_square(sx: i32, sy: i32, bytes: &[u8]) -> Result<Square, String> {
    use crate::cog::tiff;
    let tiff::Parse::Ok { big_endian, ifds } = tiff::parse(bytes)? else { return Err("truncated GeoTIFF".into()) };
    let d = ifds.into_iter().next().ok_or("no image")?;
    if d.width as usize != PX || d.height as usize != PX || d.bits != 32 || d.format != 3 {
        return Err(format!("unexpected coverage {}×{} ({} bits, format {})", d.width, d.height, d.bits, d.format));
    }
    if let (Some(s), Some(t)) = (d.scale, d.tie) {
        let (x0, y1) = (sx as f64 * SQ, (sy + 1) as f64 * SQ);
        if (s[0] - RES).abs() > 1e-6 || (s[1] - RES).abs() > 1e-6 || (t[3] - t[0] * RES - x0).abs() > 0.01 || (t[4] + t[1] * RES - y1).abs() > 0.01 {
            return Err(format!("coverage georeference {s:?} {t:?} is not square {sx},{sy}"));
        }
    }
    let mut out = vec![f32::NAN; PX * PX];
    let across = d.blocks_across() as usize;
    for (k, (&off, &len)) in d.offsets.iter().zip(&d.counts).enumerate() {
        let (o, l) = (off as usize, len as usize);
        let raw = bytes.get(o..o.saturating_add(l)).ok_or("block past the end")?;
        let b = tiff::decode_block_f32(&d, big_endian, raw)?;
        let (bx, by) = ((k % across) * d.block_w as usize, (k / across) * d.block_h as usize);
        for r in 0..d.block_h as usize {
            let row = by + r;
            if row >= PX {
                break;
            }
            for c in 0..d.block_w as usize {
                let col = bx + c;
                if col >= PX {
                    break;
                }
                // No-data is −3.4e38; England's land lies within −12 .. 1000 m.
                let v = b[r * d.block_w as usize + c];
                out[row * PX + col] = if v.is_finite() && v > -50.0 && v < 2000.0 { v } else { f32::NAN };
            }
        }
    }
    Ok(out)
}

/// Bilinear DTM heights on the National Grid for one tile build (keeps the
/// squares it touched).
pub struct Sampler<S: Squares> {
    src: S,
    got: Vec<((i32, i32), Option<Arc<Square>>)>,
}

impl<S: Squares> Sampler<S> {
    pub fn new(src: S) -> Self {
        Sampler { src, got: Vec::new() }
    }

    fn square(&mut self, sx: i32, sy: i32) -> Option<Arc<Square>> {
        if let Some(s) = self.got.iter().find(|s| s.0 == (sx, sy)) {
            return s.1.clone();
        }
        let s = self.src.square(sx, sy);
        self.got.push(((sx, sy), s.clone()));
        s
    }

    /// Global pixel (column from the grid's false origin eastwards, row
    /// from the north southwards: row `r` of square `sy` is global row
    /// `-(sy + 1)·PX + r`).
    fn pixel(&mut self, gc: i64, gr: i64) -> f32 {
        let (px, rows) = (PX as i64, PX as i64);
        let (sx, c) = (gc.div_euclid(px), gc.rem_euclid(px));
        let (q, r) = (gr.div_euclid(rows), gr.rem_euclid(rows));
        let sy = -q - 1;
        let Some(s) = self.square(sx as i32, sy as i32) else { return f32::NAN };
        s[(r * px + c) as usize]
    }

    /// Height (m) at a National Grid point; NaN where there is no data.
    pub fn at(&mut self, e: f64, n: f64) -> f32 {
        // Pixel centres sit at (k + 0.5)·RES.
        let fx = e / RES - 0.5;
        let fy = -n / RES - 0.5;
        let (c0, r0) = (fx.floor() as i64, fy.floor() as i64);
        let (tx, ty) = ((fx - c0 as f64) as f32, (fy - r0 as f64) as f32);
        let v = [self.pixel(c0, r0), self.pixel(c0 + 1, r0), self.pixel(c0, r0 + 1), self.pixel(c0 + 1, r0 + 1)];
        let top = v[0] + (v[1] - v[0]) * tx;
        let bot = v[2] + (v[3] - v[2]) * tx;
        top + (bot - top) * ty
    }
}

fn smooth01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Distance (pixels, chamfer 3-4 / 3) from each cell of a `w`×`w` grid to the
/// nearest invalid cell, capped at `cap`.
pub fn distance(valid: &[bool], w: usize, cap: usize) -> Vec<f32> {
    let big = (cap * 3 + 3) as u32;
    let mut d: Vec<u32> = valid.iter().map(|&v| if v { big } else { 0 }).collect();
    for j in 0..w {
        for i in 0..w {
            let k = j * w + i;
            let mut m = d[k];
            if i > 0 {
                m = m.min(d[k - 1] + 3);
            }
            if j > 0 {
                m = m.min(d[k - w] + 3);
                if i > 0 {
                    m = m.min(d[k - w - 1] + 4);
                }
                if i + 1 < w {
                    m = m.min(d[k - w + 1] + 4);
                }
            }
            d[k] = m;
        }
    }
    for j in (0..w).rev() {
        for i in (0..w).rev() {
            let k = j * w + i;
            let mut m = d[k];
            if i + 1 < w {
                m = m.min(d[k + 1] + 3);
            }
            if j + 1 < w {
                m = m.min(d[k + w] + 3);
                if i + 1 < w {
                    m = m.min(d[k + w + 1] + 4);
                }
                if i > 0 {
                    m = m.min(d[k + w - 1] + 4);
                }
            }
            d[k] = m;
        }
    }
    d.into_iter().map(|v| (v as f32 / 3.0).min(cap as f32)).collect()
}

/// Blend the DTM into a detail tile (see the module docs). Returns the number
/// of pixels changed.
pub fn blend<S: Squares>(tile: &DetailTile, h: &mut [f32], src: S) -> usize {
    const N: usize = 256;
    // Margin: the ramp, plus one ring for the footprint's neighbours.
    const M: usize = RAMP + 1;
    let w = N + 2 * M;
    let idx = |i: usize, j: usize| j * w + i;
    // National Grid position of every pixel centre in the window.
    let mut en = vec![(0f64, 0f64); w * w];
    let mut eng = vec![false; w * w];
    let mut any = false;
    for j in 0..w {
        for i in 0..w {
            let (lat, lon) = tile.latlon(i as i64 - M as i64, j as i64 - M as i64);
            let p = super::osgb::from_wgs84(lat, lon);
            en[idx(i, j)] = p;
            let e = in_england(p.0, p.1);
            eng[idx(i, j)] = e;
            any |= e;
        }
    }
    if !any {
        return 0;
    }
    let base = |i: usize, j: usize| -> f32 {
        let (ii, jj) = (i as i64 - M as i64, j as i64 - M as i64);
        if (0..N as i64).contains(&ii) && (0..N as i64).contains(&jj) { h[jj as usize * N + ii as usize] } else { tile.base_at(ii, jj).unwrap_or(f32::NAN) }
    };
    // Squares worth asking for: those under English land (Terrain Tiles above
    // the sea) — the service can hang on empty mosaic.
    let mut wanted: Vec<(i32, i32)> = Vec::new();
    for j in 0..w {
        for i in 0..w {
            let k = idx(i, j);
            if eng[k] && base(i, j) > 0.5 {
                let s = ((en[k].0 / SQ).floor() as i32, (en[k].1 / SQ).floor() as i32);
                if !wanted.contains(&s) {
                    wanted.push(s);
                }
            }
        }
    }
    if wanted.is_empty() {
        return 0;
    }
    struct Only<S: Squares>(S, Vec<(i32, i32)>);
    impl<S: Squares> Squares for Only<S> {
        fn square(&mut self, sx: i32, sy: i32) -> Option<Arc<Square>> {
            if self.1.contains(&(sx, sy)) { self.0.square(sx, sy) } else { None }
        }
    }
    let mut s = Sampler::new(Only(src, wanted));
    // DTM per pixel: mean of a 3×3 sub-grid over the pixel's footprint (the
    // footprint's axes from the neighbouring centres); NaN if any is missing.
    let mut dtm = vec![f32::NAN; w * w];
    for j in 1..w - 1 {
        for i in 1..w - 1 {
            let k = idx(i, j);
            if !eng[k] {
                continue;
            }
            let (c, l, r, u, d) = (en[k], en[k - 1], en[k + 1], en[k - w], en[k + w]);
            let ax = ((r.0 - l.0) / 2.0, (r.1 - l.1) / 2.0);
            let ay = ((d.0 - u.0) / 2.0, (d.1 - u.1) / 2.0);
            let mut sum = 0.0;
            let mut ok = true;
            'sub: for sy in [-1.0, 0.0, 1.0] {
                for sx in [-1.0, 0.0, 1.0] {
                    let (fx, fy) = (sx / 3.0, sy / 3.0);
                    let v = s.at(c.0 + ax.0 * fx + ay.0 * fy, c.1 + ax.1 * fx + ay.1 * fy);
                    if !v.is_finite() {
                        ok = false;
                        break 'sub;
                    }
                    sum += v;
                }
            }
            if ok {
                dtm[k] = sum / 9.0;
            }
        }
    }
    let valid: Vec<bool> = dtm.iter().map(|v| v.is_finite()).collect();
    if !valid.iter().any(|&v| v) {
        return 0;
    }
    let dist = distance(&valid, w, RAMP);
    let mut changed = 0;
    for j in 0..N {
        for i in 0..N {
            let k = idx(i + M, j + M);
            let wt = smooth01(dist[k] / RAMP as f32);
            if wt <= 0.0 {
                continue;
            }
            let o = j * N + i;
            h[o] += (dtm[k] - h[o]) * wt;
            changed += 1;
        }
    }
    changed
}

// ------------------------------------------------------------------ live

/// Disk budget for `cache/lidar/` (oldest pruned at startup).
#[cfg(not(target_arch = "wasm32"))]
const CACHE_LIMIT: u64 = 800_000_000;

#[cfg(not(target_arch = "wasm32"))]
fn root() -> std::path::PathBuf {
    static PRUNE: std::sync::Once = std::sync::Once::new();
    let r = std::path::PathBuf::from("cache").join("lidar");
    PRUNE.call_once(|| {
        let r = r.clone();
        std::thread::spawn(move || crate::cog::net::prune(r, CACHE_LIMIT));
    });
    r
}

const FAIL: &str = "terrain: Environment Agency LIDAR (earlier heights where missing)";
/// Decoded squares (4 MB each).
static SQUARES: LazyLock<Store<(i32, i32), Square>> = LazyLock::new(|| Store::new(16, FAIL));

/// GetCoverage URL for square (`sx`, `sy`).
pub fn url(sx: i32, sy: i32) -> String {
    let (x0, y0) = (sx as f64 * SQ, sy as f64 * SQ);
    format!(
        "{WCS}?service=WCS&version=2.0.1&request=GetCoverage&CoverageId={COVERAGE}&format=image/tiff&subset=E({x0:.1},{:.1})&subset=N({y0:.1},{:.1})&scaleSize=i({PX}),j({PX})&geotiff:compression=Deflate",
        x0 + SQ,
        y0 + SQ
    )
}

#[cfg(not(target_arch = "wasm32"))]
mod fetch {
    use std::io::Read;
    use std::sync::{LazyLock, Mutex};
    use web_time::{Duration, Instant};

    /// One request at a time, at least [`GAP`] apart: the service
    /// rate-limits (HTTP 429 after a handful of quick requests).
    static TURN: Mutex<Option<Instant>> = Mutex::new(None);
    const GAP: Duration = Duration::from_secs(4);
    /// No requests until then (after a failure).
    static PAUSED: Mutex<Option<Instant>> = Mutex::new(None);
    /// Pause after a connection failure / server error, and after HTTP 429.
    const BACKOFF: Duration = Duration::from_secs(60);
    const BACKOFF_429: Duration = Duration::from_secs(600);

    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(25)).user_agent("skyward-flightsim/0.2").build());

    fn pause(d: Duration) {
        *PAUSED.lock().unwrap() = Some(Instant::now() + d);
    }

    fn paused() -> bool {
        PAUSED.lock().unwrap().is_some_and(|t| Instant::now() < t)
    }

    /// GET a coverage (≤ 24 MB), trying again once after a timeout (the
    /// service stalls now and then). Fails fast offline and while paused.
    pub fn get(url: &str) -> Result<Vec<u8>, String> {
        match get_once(url)? {
            Some(b) => Ok(b),
            None => get_once(url)?.ok_or_else(|| "timed out twice".into()),
        }
    }

    /// `Ok(None)`: timed out.
    fn get_once(url: &str) -> Result<Option<Vec<u8>>, String> {
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        let mut turn = TURN.lock().unwrap();
        // (Checked after the wait for the turn too: the request before may
        // have been refused.)
        if paused() {
            return Err("service unavailable or rate-limited (pausing)".into());
        }
        if let Some(wait) = turn.map(|t| GAP.saturating_sub(t.elapsed())) {
            std::thread::sleep(wait);
        }
        *turn = Some(Instant::now());
        match AGENT.get(url).call() {
            Ok(r) => {
                let tiff = r.content_type().starts_with("image/tiff");
                let mut b = Vec::new();
                match r.into_reader().take(24 << 20).read_to_end(&mut b) {
                    Ok(_) if tiff => Ok(Some(b)),
                    Ok(_) => Err(format!("not a GeoTIFF: {}", String::from_utf8_lossy(&b[..b.len().min(200)]))),
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut || e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
                    Err(e) => Err(e.to_string()),
                }
            }
            Err(ureq::Error::Transport(t)) if t.kind() == ureq::ErrorKind::Io => Ok(None),
            Err(ureq::Error::Transport(t)) => {
                pause(BACKOFF);
                Err(t.to_string())
            }
            Err(ureq::Error::Status(429, _)) => {
                pause(BACKOFF_429);
                Err("HTTP 429 (rate-limited; scripts/lidar-fetch.sh fetches areas ahead of time)".into())
            }
            Err(ureq::Error::Status(s @ 500..=599, _)) => {
                pause(BACKOFF);
                Err(format!("HTTP {s}"))
            }
            Err(e) => Err(e.to_string()),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn load_square(sx: i32, sy: i32) -> Result<Option<Square>, String> {
    use crate::cog::net;
    let dir = root();
    let path = dir.join(format!("{sx}_{sy}.tif"));
    if let Some(b) = net::read(&path) {
        if let Ok(s) = decode_square(sx, sy, &b) {
            return Ok(Some(s));
        }
    }
    let b = fetch::get(&url(sx, sy)).map_err(|e| format!("square {sx},{sy}: {e}"))?;
    let s = decode_square(sx, sy, &b).map_err(|e| format!("square {sx},{sy}: {e}"))?;
    net::write(&path, &b);
    Ok(Some(s))
}

#[cfg(target_arch = "wasm32")]
fn load_square(_sx: i32, _sy: i32) -> Result<Option<Square>, String> {
    Ok(None)
}

/// The live service through the disk cache and the shared LRU.
pub struct Live;

impl Squares for Live {
    fn square(&mut self, sx: i32, sy: i32) -> Option<Arc<Square>> {
        SQUARES.get(&(sx, sy), || load_square(sx, sy))
    }
}

/// The Environment Agency LIDAR DTM source.
pub struct Lidar;

impl super::Source for Lidar {
    fn name(&self) -> &'static str {
        "Environment Agency LIDAR DTM"
    }
    fn refine(&self, tile: &DetailTile, h: &mut [f32]) -> Result<(), String> {
        let t0 = web_time::Instant::now();
        let n = blend(tile, h, Live);
        if std::env::var_os("SKYWARD_LIDAR_DEBUG").is_some() && n > 0 {
            eprintln!("lidar {}/{}/{}: {n} px blended in {:.1} ms", tile.z, tile.x, tile.y, t0.elapsed().as_secs_f32() * 1e3);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cog::tiff::fixture::tiff_f32;

    /// Synthetic squares from a height function of (E, N); `None` outside `has`.
    struct Synth<F: Fn(f64, f64) -> f32, H: Fn(i32, i32) -> bool>(F, H);

    impl<F: Fn(f64, f64) -> f32, H: Fn(i32, i32) -> bool> Squares for &Synth<F, H> {
        fn square(&mut self, sx: i32, sy: i32) -> Option<Arc<Square>> {
            if !(self.1)(sx, sy) {
                return None;
            }
            let (x0, y1) = (sx as f64 * SQ, (sy + 1) as f64 * SQ);
            Some(Arc::new((0..PX * PX).map(|k| (self.0)(x0 + ((k % PX) as f64 + 0.5) * RES, y1 - ((k / PX) as f64 + 0.5) * RES)).collect()))
        }
    }

    #[test]
    fn england_mask() {
        let at = |lat: f64, lon: f64| {
            let (e, n) = super::super::osgb::from_wgs84(lat, lon);
            in_england(e, n)
        };
        // London City, the Chilterns, the South Downs, Newcastle, Penzance, Carlisle.
        for (la, lo) in [(51.505, 0.055), (51.70, -0.80), (50.90, -0.40), (54.97, -1.61), (50.12, -5.54), (54.89, -2.93)] {
            assert!(at(la, lo), "{la} {lo} should be England");
        }
        // Cardiff, Snowdon, Edinburgh, Glasgow, Douglas (IoM), St Helier, Calais, Dublin.
        for (la, lo) in [(51.48, -3.18), (53.07, -4.08), (55.95, -3.19), (55.86, -4.25), (54.15, -4.48), (49.19, -2.11), (50.95, 1.86), (53.35, -6.26)] {
            assert!(!at(la, lo), "{la} {lo} should not be England");
        }
    }

    #[test]
    fn samples_bilinear_across_squares() {
        let plane = |e: f64, n: f64| (0.001 * e + 0.002 * n) as f32;
        let sy = Synth(plane, |_, _| true);
        let mut s = Sampler::new(&sy);
        for (e, n) in [(542_345.6, 181_234.5), (549_999.0, 180_001.0), (550_000.0, 180_000.0), (540_003.0, 189_996.0)] {
            let v = s.at(e, n);
            assert!((v - plane(e, n)).abs() < 1e-3, "{e} {n}: {v} vs {}", plane(e, n));
        }
        // Each square is fetched once per sampler.
        let mut keys: Vec<_> = s.got.iter().map(|g| g.0).collect();
        keys.dedup();
        assert_eq!(keys.len(), s.got.len());
        let none = Synth(plane, |_, _| false);
        assert!(Sampler::new(&none).at(542_000.0, 181_000.0).is_nan());
    }

    #[test]
    fn decodes_a_square_and_checks_its_georeference() {
        let px = |c: u32, r: u32| if c == 0 && r == 0 { -3.4e38 } else { (c + r) as f32 * 0.1 };
        let b = tiff_f32(PX as u32, PX as u32, 512, 512, px, [RES, RES, 0.0], [0.0, 0.0, 0.0, 540_000.0, 190_000.0, 0.0], false);
        let s = decode_square(54, 18, &b).unwrap();
        assert!(s[0].is_nan());
        assert!((s[PX * 3 + 2] - 0.5).abs() < 1e-6);
        assert!((s[PX * PX - 1] - 199.8).abs() < 1e-3);
        // A georeference for another square, or another resolution, is refused.
        assert!(decode_square(55, 18, &b).is_err());
        let b5 = tiff_f32(PX as u32, PX as u32, 512, 512, px, [5.0, 5.0, 0.0], [0.0, 0.0, 0.0, 540_000.0, 190_000.0, 0.0], false);
        assert!(decode_square(54, 18, &b5).is_err());
        assert!(decode_square(54, 18, b"<?xml version=\"1.0\"?>").is_err());
    }

    #[test]
    fn chamfer_distance() {
        let w = 9;
        let mut v = vec![true; w * w];
        v[4 * w + 4] = false;
        let d = distance(&v, w, 3);
        assert_eq!(d[4 * w + 4], 0.0);
        assert!((d[4 * w + 5] - 1.0).abs() < 1e-6);
        assert!((d[4 * w + 6] - 2.0).abs() < 1e-6);
        assert!((d[5 * w + 5] - 4.0 / 3.0).abs() < 1e-6);
        assert_eq!(d[0], 3.0);
    }

    /// The z12 tile over London City airport: the DTM replaces the base where
    /// it has data and ramps in smoothly at a hole; ramps agree across the
    /// tile edge; a tile over Wales is left alone.
    #[test]
    fn blends_with_a_smooth_edge() {
        let (fx, fy) = crate::osm::TileId::frac(51.5053, 0.0553, 12);
        let (x, y) = (fx as u32, fy as u32);
        let land = |_: i64, _: i64| Some(10.0f32);
        let tile = DetailTile { z: 12, x, y, base: &land };
        // DTM: 30 m everywhere except a 600 m hole around the tile's centre.
        let (clat, clon) = tile.latlon(128, 128);
        let (ce, cn) = super::super::osgb::from_wgs84(clat, clon);
        let f = move |e: f64, n: f64| if (e - ce).abs() < 300.0 && (n - cn).abs() < 300.0 { f32::NAN } else { 30.0 };
        let sy = Synth(f, |_, _| true);
        let mut h = vec![10.0f32; 256 * 256];
        let n = blend(&tile, &mut h, &sy);
        assert!(n > 60_000, "{n}");
        assert!((h[10 * 256 + 10] - 30.0).abs() < 1e-4, "{}", h[10 * 256 + 10]);
        assert_eq!(h[128 * 256 + 128], 10.0);
        // Along a row out of the hole: monotonic, no step larger than the ramp allows.
        let row: Vec<f32> = (128..200).map(|i| h[128 * 256 + i]).collect();
        assert!(row.windows(2).all(|p| p[1] >= p[0] - 1e-4), "{row:?}");
        assert!(row.windows(2).all(|p| p[1] - p[0] < 20.0 / RAMP as f32 * 1.6), "{row:?}");
        assert!((row.last().unwrap() - 30.0).abs() < 1e-4);

        // The right neighbour's left column continues this tile's right column.
        let mut h2 = vec![10.0f32; 256 * 256];
        let t2 = DetailTile { z: 12, x: x + 1, y, base: &land };
        blend(&t2, &mut h2, &sy);
        for j in [0, 100, 255] {
            assert!((h[j * 256 + 255] - h2[j * 256]).abs() < 1e-3);
        }

        // Cardiff: outside England, untouched.
        let (fx, fy) = crate::osm::TileId::frac(51.48, -3.18, 12);
        let t3 = DetailTile { z: 12, x: fx as u32, y: fy as u32, base: &land };
        let mut h3 = vec![10.0f32; 256 * 256];
        assert_eq!(blend(&t3, &mut h3, &sy), 0);
        assert!(h3.iter().all(|&v| v == 10.0));

        // Over the sea (base ≤ 0): no squares asked for.
        let sea = |_: i64, _: i64| Some(0.0f32);
        let t4 = DetailTile { z: 12, x, y, base: &sea };
        let mut h4 = vec![0.0f32; 256 * 256];
        assert_eq!(blend(&t4, &mut h4, &sy), 0);
    }

    /// Live service (network; `cargo test --release -- --ignored live_`):
    /// heights near London City (river-side ground a few metres up) and on
    /// the Chilterns escarpment (Coombe Hill ≈ 260 m).
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_lidar_heights() {
        let mut s = Sampler::new(Live);
        let (e, n) = super::super::osgb::from_wgs84(51.5048, 0.0495);
        let v = s.at(e, n);
        assert!(v.is_finite() && v > -2.0 && v < 15.0, "EGLC {v}");
        // Within 600 m of Coombe Hill's monument (SP 849 063).
        let peak = (0..61 * 61).map(|k| s.at(484_900.0 + ((k % 61) as f64 - 30.0) * 20.0, 206_300.0 + ((k / 61) as f64 - 30.0) * 20.0)).fold(f32::MIN, f32::max);
        assert!((peak - 260.0).abs() < 15.0, "Coombe Hill {peak}");
    }
}
