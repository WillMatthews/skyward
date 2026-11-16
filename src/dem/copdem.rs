//! Copernicus DEM GLO-30 (30 m global surface model from TanDEM-X).
//!
//! `s3://copernicus-dem-30m/{name}/{name}.tif` (eu-central-1, anonymous, no
//! CORS headers) with `name` = `Copernicus_DSM_COG_10_N47_00_E011_00_DEM`:
//! 1°×1° tiles named after their south-west corner, EPSG:4326 COGs of 32-bit
//! float heights (m, EGM2008), 1024² DEFLATE tiles with the floating-point
//! predictor, three overviews. 3600 rows of 1"; 3600 columns up to 50°,
//! 2400 to 60°, 1800 to 70°, 1200 to 80°, then 720 / 360. Pixel-is-point:
//! the tie point is the centre of the north-west pixel. Ocean has no file
//! (404), nor do a few unreleased tiles (Armenia, Azerbaijan).
//!
//! **Use** ([`CopDem::refine`]): GLO-30 is a *surface* model (roofs, tree
//! tops), so it is blended into the Terrain Tiles heights by terrain
//! roughness: valley floors, plains and cities keep Terrain Tiles (often a
//! LIDAR / national bare-earth model there), slopes and ridges take GLO-30
//! (sharper, far fewer spikes in high mountains). North of 60° N, where
//! Terrain Tiles falls back to coarser sources, GLO-30 is used everywhere —
//! with a small morphological opening (5 px) on flat ground to knock down single
//! buildings and tree clumps. Runway flattening runs on the result, so
//! airports stay flat.
//!
//! **Cache**: headers and the compressed 1024² blocks (≈ 2.5 MB each) under
//! `cache/copdem/` (pruned to [`CACHE_LIMIT`]), decoded blocks in a small LRU.

use super::DetailTile;
use crate::cog::Store;
use crate::cog::tiff::Ifd;
use std::sync::{Arc, LazyLock};

const BASE: &str = "https://copernicus-dem-30m.s3.eu-central-1.amazonaws.com";

/// Attribution (Credits screen; also README, NOTICE, web/credits.html). The
/// licence's wording for adapted / modified data.
pub const CREDIT: &str = "Produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved (desktop version)";

/// Off: tests, the browser (no CORS), a flat world, or `SKYWARD_NO_COPDEM`.
pub fn enabled() -> bool {
    static ON: LazyLock<bool> =
        LazyLock::new(|| !cfg!(test) && !cfg!(target_arch = "wasm32") && std::env::var_os("SKYWARD_NO_COPDEM").is_none() && std::env::var_os("SKYWARD_FLAT_TERRAIN").is_none());
    *ON
}

/// File name (and directory) of the 1° tile with south-west corner (`la`, `lo`).
pub fn tile_name(la: i32, lo: i32) -> String {
    let ns = if la < 0 { 'S' } else { 'N' };
    let ew = if lo < 0 { 'W' } else { 'E' };
    format!("Copernicus_DSM_COG_10_{ns}{:02}_00_{ew}{:03}_00_DEM", la.abs(), lo.abs())
}

/// A file's full-resolution image.
pub struct Header {
    pub ifd: Ifd,
    pub big_endian: bool,
}

impl Header {
    /// Fractional pixel (column, row) of a point: integers at pixel centres.
    pub fn pixel(&self, lat: f64, lon: f64) -> (f64, f64) {
        let (s, t) = (self.ifd.scale.unwrap_or([1.0; 3]), self.ifd.tie.unwrap_or([0.0; 6]));
        let half = if self.ifd.pixel_is_point { 0.0 } else { 0.5 };
        ((lon - t[3]) / s[0] + t[0] - half, (t[4] - lat) / s[1] + t[1] - half)
    }
    /// (lat, lon) of a pixel centre.
    pub fn centre(&self, col: i64, row: i64) -> (f64, f64) {
        let (s, t) = (self.ifd.scale.unwrap_or([1.0; 3]), self.ifd.tie.unwrap_or([0.0; 6]));
        let half = if self.ifd.pixel_is_point { 0.0 } else { 0.5 };
        (t[4] - (row as f64 - t[1] + half) * s[1], t[3] + (col as f64 - t[0] + half) * s[0])
    }
}

/// Check a parsed file: one float band, georeferenced, sane block table.
pub fn header_from(big_endian: bool, ifds: Vec<Ifd>) -> Result<Header, String> {
    let ifd = ifds.into_iter().next().ok_or("no image")?;
    if ifd.bits != 32 || ifd.format != 3 {
        return Err(format!("expected float32 heights, got {} bits format {}", ifd.bits, ifd.format));
    }
    match (ifd.scale, ifd.tie) {
        (Some(s), Some(_)) if s[0] > 0.0 && s[1] > 0.0 => Ok(Header { ifd, big_endian }),
        _ => Err("no georeference".into()),
    }
}

/// Disk budget for `cache/copdem/` (oldest pruned at startup).
#[cfg(not(target_arch = "wasm32"))]
const CACHE_LIMIT: u64 = 1_200_000_000;

#[cfg(not(target_arch = "wasm32"))]
fn root() -> std::path::PathBuf {
    static PRUNE: std::sync::Once = std::sync::Once::new();
    let r = std::path::PathBuf::from("cache").join("copdem");
    PRUNE.call_once(|| {
        let r = r.clone();
        std::thread::spawn(move || crate::cog::net::prune(r, CACHE_LIMIT));
    });
    r
}

const FAIL: &str = "terrain: Copernicus DEM (Terrain Tiles heights where missing)";
static HEADERS: LazyLock<Store<(i32, i32), Header>> = LazyLock::new(|| Store::new(96, FAIL));
/// Decoded 1024² blocks (4 MB each).
static BLOCKS: LazyLock<Store<(i32, i32, u32), Vec<f32>>> = LazyLock::new(|| Store::new(20, FAIL));

#[cfg(not(target_arch = "wasm32"))]
fn load_header(la: i32, lo: i32) -> Result<Option<Header>, String> {
    let name = tile_name(la, lo);
    let url = format!("{BASE}/{name}/{name}.tif");
    let dir = root();
    match crate::cog::net::header(&url, &dir.join(format!("{name}.hdr")), &dir.join(format!("{name}.none")), &name)? {
        Some((be, ifds)) => header_from(be, ifds).map(Some).map_err(|e| format!("{name}: {e}")),
        None => Ok(None),
    }
}

#[cfg(target_arch = "wasm32")]
fn load_header(_la: i32, _lo: i32) -> Result<Option<Header>, String> {
    Ok(None)
}

#[cfg(not(target_arch = "wasm32"))]
fn load_block(la: i32, lo: i32, hdr: &Header, idx: u32) -> Result<Option<Vec<f32>>, String> {
    use crate::cog::net;
    let d = &hdr.ifd;
    let name = tile_name(la, lo);
    let (off, len) = (d.offsets[idx as usize], d.counts[idx as usize]);
    if len == 0 {
        return Ok(None);
    }
    if len > 24 << 20 {
        return Err(format!("{name}: block of {len} bytes"));
    }
    let path = root().join(&name).join(format!("{idx}.bin"));
    let raw = match net::read(&path).filter(|b| b.len() as u64 == len) {
        Some(b) => b,
        None => {
            let url = format!("{BASE}/{name}/{name}.tif");
            let b = net::range(&url, off, len)?.ok_or_else(|| format!("{name}: file vanished"))?;
            if b.len() as u64 != len {
                return Err(format!("{name}: short block"));
            }
            net::write(&path, &b);
            b
        }
    };
    crate::cog::tiff::decode_block_f32(d, hdr.big_endian, &raw).map(Some)
}

#[cfg(target_arch = "wasm32")]
fn load_block(_la: i32, _lo: i32, _hdr: &Header, _idx: u32) -> Result<Option<Vec<f32>>, String> {
    Ok(None)
}

/// Where GLO-30 heights come from: the bucket (live), or fixed files (tests).
pub trait Files {
    fn header(&mut self, la: i32, lo: i32) -> Option<Arc<Header>>;
    fn block(&mut self, la: i32, lo: i32, hdr: &Header, idx: u32) -> Option<Arc<Vec<f32>>>;
}

/// The live bucket through the disk cache and the shared LRUs.
pub struct Live;

impl Files for Live {
    fn header(&mut self, la: i32, lo: i32) -> Option<Arc<Header>> {
        HEADERS.get(&(la, lo), || load_header(la, lo))
    }
    fn block(&mut self, la: i32, lo: i32, hdr: &Header, idx: u32) -> Option<Arc<Vec<f32>>> {
        BLOCKS.get(&(la, lo, idx), || load_block(la, lo, hdr, idx))
    }
}

/// Bilinear GLO-30 heights for one tile build (keeps the files and blocks it
/// touched). NaN where there is no data (ocean, unreleased tiles, offline).
pub struct Sampler<F: Files> {
    files: F,
    tiles: Vec<((i32, i32), Option<Arc<Header>>)>,
    blocks: Vec<((i32, i32, u32), Option<Arc<Vec<f32>>>)>,
    last: usize,
}

impl<F: Files> Sampler<F> {
    pub fn new(files: F) -> Self {
        Sampler { files, tiles: Vec::new(), blocks: Vec::new(), last: 0 }
    }

    fn tile(&mut self, la: i32, lo: i32) -> Option<Arc<Header>> {
        if let Some(t) = self.tiles.iter().find(|t| t.0 == (la, lo)) {
            return t.1.clone();
        }
        let h = if (-90..90).contains(&la) && (-180..180).contains(&lo) { self.files.header(la, lo) } else { None };
        self.tiles.push(((la, lo), h.clone()));
        h
    }

    /// Height of pixel (`col`, `row`) of file (`la`, `lo`) (in range).
    fn read(&mut self, la: i32, lo: i32, hdr: &Header, col: u32, row: u32) -> f32 {
        let d = &hdr.ifd;
        let idx = (row / d.block_h) * d.blocks_across() + col / d.block_w;
        let key = (la, lo, idx);
        let k = match self.blocks.get(self.last).filter(|b| b.0 == key) {
            Some(_) => self.last,
            None => match self.blocks.iter().position(|b| b.0 == key) {
                Some(i) => i,
                None => {
                    let b = self.files.block(la, lo, hdr, idx);
                    self.blocks.push((key, b));
                    self.blocks.len() - 1
                }
            },
        };
        self.last = k;
        let Some(b) = &self.blocks[k].1 else { return f32::NAN };
        let v = b.get(((row % d.block_h) * d.block_w + col % d.block_w) as usize).copied().unwrap_or(f32::NAN);
        // No-data is -32767; nothing on Earth is below -1000 m.
        if v.is_finite() && v > -1000.0 { v } else { f32::NAN }
    }

    /// The pixel nearest a point, looking in the neighbouring file when the
    /// point lies on a file's edge.
    fn nearest(&mut self, lat: f64, lon: f64) -> f32 {
        let (mut la, mut lo) = (lat.ceil() as i32 - 1, lon.floor() as i32);
        for _ in 0..3 {
            let Some(h) = self.tile(la, lo) else { return f32::NAN };
            let (fx, fy) = h.pixel(lat, lon);
            let (c, r) = (fx.round() as i64, fy.round() as i64);
            let (w, hh) = (h.ifd.width as i64, h.ifd.height as i64);
            if c < 0 {
                lo -= 1;
            } else if c >= w {
                lo += 1;
            } else if r < 0 {
                la += 1;
            } else if r >= hh {
                la -= 1;
            } else {
                return self.read(la, lo, &h, c as u32, r as u32);
            }
        }
        f32::NAN
    }

    /// Height (m) at a point: bilinear between pixel centres. (Files hold
    /// rows from their north edge down to just above their south edge.)
    pub fn at(&mut self, lat: f64, lon: f64) -> f32 {
        let (la, lo) = (lat.ceil() as i32 - 1, lon.floor() as i32);
        let Some(h) = self.tile(la, lo) else { return f32::NAN };
        let (fx, fy) = h.pixel(lat, lon);
        let (x0, y0) = (fx.floor() as i64, fy.floor() as i64);
        let (tx, ty) = ((fx - x0 as f64) as f32, (fy - y0 as f64) as f32);
        let (w, hh) = (h.ifd.width as i64, h.ifd.height as i64);
        let mut v = [0f32; 4];
        for (k, (dx, dy)) in [(0, 0), (1, 0), (0, 1), (1, 1)].into_iter().enumerate() {
            let (c, r) = (x0 + dx, y0 + dy);
            v[k] = if c >= 0 && r >= 0 && c < w && r < hh {
                self.read(la, lo, &h, c as u32, r as u32)
            } else {
                // Across the file's edge: the neighbour's pixel there.
                let (plat, plon) = h.centre(c, r);
                self.nearest(plat, plon)
            };
        }
        let top = v[0] + (v[1] - v[0]) * tx;
        let bot = v[2] + (v[3] - v[2]) * tx;
        top + (bot - top) * ty
    }
}

fn smooth01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Terrain Tiles relief (max − min, m) over a 7×7-pixel window (≈ 170 m at
/// 47°) at which GLO-30 starts / fully takes over.
const RELIEF: (f32, f32) = (15.0, 60.0);
/// Latitudes (°N) over which GLO-30 takes over everywhere.
const NORTH: (f64, f64) = (59.5, 60.5);

/// Blend GLO-30 into a detail tile (see the module docs). `files`: where the
/// GLO-30 heights come from. Returns the number of pixels changed.
pub fn blend<F: Files>(tile: &DetailTile, h: &mut [f32], files: F) -> usize {
    const N: usize = 256;
    // GLO-30 on the tile's pixel centres, with a margin for the opening.
    const GM: usize = 2;
    let gw = N + 2 * GM;
    let mut s = Sampler::new(files);
    let mut g = vec![f32::NAN; gw * gw];
    let mut any = false;
    for j in 0..gw {
        for i in 0..gw {
            let (lat, lon) = tile.latlon(i as i64 - GM as i64, j as i64 - GM as i64);
            let v = s.at(lat, lon);
            any |= v.is_finite();
            g[j * gw + i] = v;
        }
    }
    if !any {
        return 0;
    }
    // Opening (erode, then dilate): removes bumps narrower than 5 px
    // (≈ 65–130 m: a GLO-30 pixel or two — single buildings, tree clumps).
    let opened = super::window(&super::window(&g, gw, GM, false), gw, GM, true);
    // Roughness weight from Terrain Tiles relief (neighbour tiles for the
    // margin, so the weight is continuous across tile edges), smoothed.
    const RR: usize = 3;
    const RS: usize = 2;
    let tm = RR + RS;
    let tw = N + 2 * tm;
    let mut t = vec![f32::NAN; tw * tw];
    for j in 0..tw {
        for i in 0..tw {
            let (ii, jj) = (i as i64 - tm as i64, j as i64 - tm as i64);
            t[j * tw + i] = if (0..N as i64).contains(&ii) && (0..N as i64).contains(&jj) { h[jj as usize * N + ii as usize] } else { tile.base_at(ii, jj).unwrap_or(f32::NAN) };
        }
    }
    let (mx, mn) = (super::window(&t, tw, RR, true), super::window(&t, tw, RR, false));
    let rough: Vec<f32> = mx.iter().zip(&mn).map(|(a, b)| if a.is_finite() && b.is_finite() { smooth01((a - b - RELIEF.0) / (RELIEF.1 - RELIEF.0)) } else { 0.0 }).collect();
    let rough = super::box_mean(&rough, tw, RS);
    let mut changed = 0;
    for j in 0..N {
        let (lat, _) = tile.latlon(0, j as i64);
        let north = smooth01(((lat - NORTH.0) / (NORTH.1 - NORTH.0)) as f32);
        for i in 0..N {
            let gk = (j + GM) * gw + i + GM;
            let (raw, op) = (g[gk], opened[gk]);
            if !raw.is_finite() {
                continue;
            }
            let wr = rough[(j + tm) * tw + i + tm];
            let w = wr.max(north);
            if w <= 0.0 {
                continue;
            }
            // Flat ground: the opened surface (no roofs / tree clumps).
            let glo = if op.is_finite() { op + (raw - op) * wr } else { raw };
            let k = j * N + i;
            h[k] += (glo - h[k]) * w;
            changed += 1;
        }
    }
    changed
}

/// The Copernicus GLO-30 source.
pub struct CopDem;

impl super::Source for CopDem {
    fn name(&self) -> &'static str {
        "Copernicus DEM GLO-30"
    }
    fn refine(&self, tile: &DetailTile, h: &mut [f32]) -> Result<(), String> {
        let t0 = web_time::Instant::now();
        let n = blend(tile, h, Live);
        if std::env::var_os("SKYWARD_COPDEM_DEBUG").is_some() {
            eprintln!("copdem {}/{}/{}: {n} px blended in {:.1} ms", tile.z, tile.x, tile.y, t0.elapsed().as_secs_f32() * 1e3);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cog::tiff;
    use crate::cog::tiff::fixture::tiff_f32;

    /// Fixed files: (south-west corner, file bytes), parsed once.
    struct Fixed(Vec<((i32, i32), Arc<Header>, Vec<u8>)>);

    impl Fixed {
        fn new(files: Vec<((i32, i32), Vec<u8>)>) -> Self {
            Fixed(
                files
                    .into_iter()
                    .map(|(k, b)| {
                        let tiff::Parse::Ok { big_endian, ifds } = tiff::parse(&b).unwrap() else { panic!("short") };
                        (k, Arc::new(header_from(big_endian, ifds).unwrap()), b)
                    })
                    .collect(),
            )
        }
    }

    impl Files for &Fixed {
        fn header(&mut self, la: i32, lo: i32) -> Option<Arc<Header>> {
            self.0.iter().find(|f| f.0 == (la, lo)).map(|f| f.1.clone())
        }
        fn block(&mut self, la: i32, lo: i32, hdr: &Header, idx: u32) -> Option<Arc<Vec<f32>>> {
            let f = self.0.iter().find(|f| f.0 == (la, lo))?;
            let (o, c) = (hdr.ifd.offsets[idx as usize] as usize, hdr.ifd.counts[idx as usize] as usize);
            Some(Arc::new(tiff::decode_block_f32(&hdr.ifd, hdr.big_endian, &f.2[o..o + c]).unwrap()))
        }
    }

    /// A tiny "1°" file: `n`² pixels, pixel-is-point like the real ones, heights from `f(lat, lon)`.
    fn file(la: i32, lo: i32, n: u32, f: impl Fn(f64, f64) -> f32) -> ((i32, i32), Vec<u8>) {
        let s = 1.0 / n as f64;
        let px = |c: u32, r: u32| f(la as f64 + 1.0 - r as f64 * s, lo as f64 + c as f64 * s);
        ((la, lo), tiff_f32(n, n, 16, 16, px, [s, s, 0.0], [0.0, 0.0, 0.0, lo as f64, la as f64 + 1.0, 0.0], true))
    }

    /// Live data (network; `cargo test --release -- --ignored live_`): the
    /// real files decode and sit where they should — Innsbruck airport's
    /// apron (581 m), the Hafelekarspitze (2334 m), open sea has no file.
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_glo30_heights() {
        let mut s = Sampler::new(Live);
        let apron = s.at(47.2595, 11.3460);
        assert!((apron - 581.0).abs() < 12.0, "LOWI {apron}");
        // Highest point within ~250 m of the summit's listed position.
        let peak = (0..21 * 21).map(|k| s.at(47.3123 + ((k / 21) as f64 - 10.0) * 2.5e-4, 11.3858 + ((k % 21) as f64 - 10.0) * 3.5e-4)).fold(f32::MIN, f32::max);
        assert!((peak - 2334.0).abs() < 25.0, "Hafelekar {peak}");
        assert!(s.at(50.0, -20.0).is_nan());
        // Above 60° N (1800 columns per file).
        let lyngen = s.at(69.58, 20.2);
        assert!(lyngen.is_finite() && lyngen > -5.0 && lyngen < 1900.0, "{lyngen}");
    }

    #[test]
    fn names() {
        assert_eq!(tile_name(47, 11), "Copernicus_DSM_COG_10_N47_00_E011_00_DEM");
        assert_eq!(tile_name(-1, -60), "Copernicus_DSM_COG_10_S01_00_W060_00_DEM");
    }

    #[test]
    fn samples_bilinear_across_file_edges() {
        // A plane h = 100 lat + 10 lon over two files side by side and one below.
        let plane = |lat: f64, lon: f64| (100.0 * lat + 10.0 * lon) as f32;
        let fx = Fixed::new(vec![file(47, 11, 40, plane), file(47, 12, 40, plane), file(46, 11, 40, plane)]);
        let mut s = Sampler::new(&fx);
        for (lat, lon) in [(47.5, 11.5), (47.3013, 11.9871), (47.0101, 11.2), (47.999, 11.001), (47.004, 11.5), (47.5, 11.99)] {
            let v = s.at(lat, lon);
            assert!((v - plane(lat, lon)).abs() < 0.01, "{lat} {lon}: {v} vs {}", plane(lat, lon));
        }
        // Pixel-is-point: pixel (0, 0) is centred on the corner.
        assert_eq!(s.at(48.0, 11.0), plane(48.0, 11.0));
        // No file (ocean): NaN.
        assert!(s.at(47.5, 10.5).is_nan());
        assert!(s.at(45.5, 11.5).is_nan());
    }

    /// A detail tile over (or near) Innsbruck: rough terrain takes GLO-30, flat
    /// ground keeps the base, and a lone "building" on flat ground in the
    /// north is opened away.
    #[test]
    fn blends_by_roughness_and_latitude() {
        // z12 tile containing 47.26° N 11.34° E.
        let (fx, fy) = crate::osm::TileId::frac(47.26, 11.34, 12);
        let (x, y) = (fx as u32, fy as u32);
        // Base: flat at 580 m west of the tile's middle, a 45° ramp east of it.
        let base_px = |px: i64, _py: i64| -> Option<f32> {
            let i = px - x as i64 * 256;
            Some(if i < 128 { 580.0 } else { 580.0 + (i - 128) as f32 * 25.0 })
        };
        let mut h: Vec<f32> = (0..256 * 256).map(|k| base_px(x as i64 * 256 + (k % 256) as i64, 0).unwrap()).collect();
        let base = h.clone();
        // GLO-30: base + 7 m everywhere (a DSM offset).
        let tile = DetailTile { z: 12, x, y, base: &base_px };
        let f = |lat: f64, lon: f64| {
            let (fx, _) = crate::osm::TileId::frac(lat, lon, 12);
            let i = ((fx - x as f64) * 256.0 - 0.5).round() as i64;
            (if i < 128 { 580.0 } else { 580.0 + (i - 128) as f32 * 25.0 }) + 7.0
        };
        let fx = Fixed::new(vec![file(47, 11, 400, f)]);
        let n = blend(&tile, &mut h, &fx);
        assert!(n > 0);
        // Flat west: unchanged. Steep east: GLO-30 (within interpolation).
        assert_eq!(h[100 * 256 + 20], base[100 * 256 + 20]);
        let k = 100 * 256 + 220;
        assert!((h[k] - base[k] - 7.0).abs() < 3.0, "{} vs {}", h[k], base[k]);

        // North of 60°: GLO-30 everywhere, a single-pixel spike opened away.
        let (fx, fy) = crate::osm::TileId::frac(69.6, 20.2, 12);
        let (x, y) = (fx as u32, fy as u32);
        let flat = |_: i64, _: i64| Some(100.0f32);
        let tile = DetailTile { z: 12, x, y, base: &flat };
        let (clat, clon) = tile.latlon(128, 128);
        let f = move |lat: f64, lon: f64| if (lat - clat).abs() < 0.00027 && (lon - clon).abs() < 0.00027 { 140.0 } else { 120.0 };
        let fx = Fixed::new(vec![file(69, 20, 1800, f)]);
        let mut h = vec![100.0f32; 256 * 256];
        blend(&tile, &mut h, &fx);
        assert!((h[10 * 256 + 10] - 120.0).abs() < 0.01, "{}", h[10 * 256 + 10]);
        assert!(h[128 * 256 + 128] < 121.0, "spike kept: {}", h[128 * 256 + 128]);
    }
}
