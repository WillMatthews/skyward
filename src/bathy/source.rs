//! Depth tiles: where they come from, how they are decoded and cached.
//!
//! * EMODnet Bathymetry WCS (GeoServer): `GetCoverage` of `emodnet:mean` as
//!   a big-endian, tiled, uncompressed float32 GeoTIFF; elevation (m, + up,
//!   land included) relative to lowest astronomical tide; exactly 0.0 where
//!   the DTM has no data. Sends `Access-Control-Allow-Origin: *`.
//! * NOAA NCEI `DEM_global_mosaic` ArcGIS image service: `exportImage` as a
//!   little-endian, 128² tiled, LZW float32 GeoTIFF (ETOPO 2022 15″ under
//!   NOAA's coastal DEMs); global. Sends `Access-Control-Allow-Origin: *`.
//!
//! Both answers are untrusted bytes: the TIFF reader bounds-checks
//! everything, sizes must match the request, values are range-checked.

use super::{Grid, TileKey};
use crate::cog::tiff;
use std::sync::{Arc, LazyLock};

/// The WCS's coverage envelope (west, south, east, north); outside it only NOAA.
const EMODNET_BOX: (f64, f64, f64, f64) = (-70.5, 11.0, 43.0, 90.0);

pub fn emodnet_url(k: TileKey) -> String {
    let (w, s, e, n) = k.bounds();
    let px = super::LEVELS[k.level as usize].px;
    format!(
        "https://ows.emodnet-bathymetry.eu/wcs?service=WCS&version=1.0.0&request=GetCoverage&coverage=emodnet:mean&crs=EPSG:4326&bbox={w},{s},{e},{n}&format=image/tiff&width={px}&height={px}"
    )
}

pub fn noaa_url(k: TileKey) -> String {
    let (w, s, e, n) = k.bounds();
    let px = super::LEVELS[k.level as usize].px;
    format!(
        "https://gis.ngdc.noaa.gov/arcgis/rest/services/DEM_mosaics/DEM_global_mosaic/ImageServer/exportImage?bbox={w},{s},{e},{n}&bboxSR=4326&imageSR=4326&size={px},{px}&format=tiff&pixelType=F32&compression=LZ77&interpolation=RSP_BilinearInterpolation&f=image"
    )
}

/// True if EMODnet may have data for the tile.
pub fn in_emodnet(k: TileKey) -> bool {
    let (w, s, e, n) = k.bounds();
    let (bw, bs, be, bn) = EMODNET_BOX;
    w < be && e > bw && s < bn && n > bs
}

/// Which service answered (decoding differs only in the no-data value).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Svc {
    Emodnet,
    Noaa,
}

/// Decode an answer to a `px`² request into elevations (NaN = no data).
pub fn decode(bytes: &[u8], px: u32, svc: Svc) -> Result<Vec<f32>, String> {
    let (big_endian, ifds) = match tiff::parse(bytes)? {
        tiff::Parse::Ok { big_endian, ifds } => (big_endian, ifds),
        tiff::Parse::Need(_) => return Err("truncated TIFF".into()),
    };
    let ifd = &ifds[0];
    if ifd.width != px || ifd.height != px {
        return Err(format!("unexpected size {}×{} (asked {px}²)", ifd.width, ifd.height));
    }
    let mut v = read_f32(bytes, big_endian, ifd)?;
    for e in &mut v {
        // Deeper than the Challenger Deep / higher than Everest: no data
        // (ArcGIS uses ±3.4e38); EMODnet's gaps are exactly 0.
        if !e.is_finite() || !(-11_500.0..9_000.0).contains(e) || (svc == Svc::Emodnet && *e == 0.0) {
            *e = f32::NAN;
        }
    }
    Ok(v)
}

/// A whole single-band float image (all blocks of `ifd`, row-major).
pub(crate) fn read_f32(bytes: &[u8], big_endian: bool, ifd: &tiff::Ifd) -> Result<Vec<f32>, String> {
    let (w, h) = (ifd.width as usize, ifd.height as usize);
    if w * h > 1 << 24 {
        return Err("float TIFF too large".into());
    }
    let mut out = vec![f32::NAN; w * h];
    let (bw, bh) = (ifd.block_w as usize, ifd.block_h as usize);
    let across = ifd.blocks_across() as usize;
    for (k, (&off, &cnt)) in ifd.offsets.iter().zip(&ifd.counts).enumerate() {
        let end = off.checked_add(cnt).filter(|&e| e <= bytes.len() as u64).ok_or("TIFF block past the end")?;
        let block = tiff::decode_block_f32(ifd, big_endian, &bytes[off as usize..end as usize])?;
        let (bx, by) = ((k % across) * bw, (k / across) * bh);
        for r in 0..bh.min(h.saturating_sub(by)) {
            let n = bw.min(w - bx);
            out[(by + r) * w + bx..][..n].copy_from_slice(&block[r * bw..][..n]);
        }
    }
    Ok(out)
}

/// Fill `a`'s gaps from `b` (same grid).
pub fn fill(a: &mut [f32], b: &[f32]) {
    for (x, y) in a.iter_mut().zip(b) {
        if x.is_nan() {
            *x = *y;
        }
    }
}

// ---------------------------------------------------------------- cache format

/// Disk form: "SKBY" + version, n (u16), then DEFLATE'd i16 quarter metres
/// (i16::MIN = no data; depths past 8 km clamp — all the same deep blue).
const MAGIC: &[u8; 5] = b"SKBY1";

pub fn encode_cache(g: &Grid) -> Vec<u8> {
    use std::io::Write;
    let mut raw = Vec::with_capacity(g.elev.len() * 2);
    for &e in &g.elev {
        let q = if e.is_nan() { i16::MIN } else { (e * 4.0).round().clamp(-32_000.0, 32_000.0) as i16 };
        raw.extend(q.to_le_bytes());
    }
    let mut out = MAGIC.to_vec();
    out.extend((g.n as u16).to_le_bytes());
    let mut z = flate2::write::DeflateEncoder::new(out, flate2::Compression::default());
    let _ = z.write_all(&raw);
    z.finish().unwrap_or_default()
}

pub fn decode_cache(b: &[u8], px: u32) -> Option<Grid> {
    use std::io::Read;
    if b.len() < 7 || &b[..5] != MAGIC || u16::from_le_bytes([b[5], b[6]]) as u32 != px {
        return None;
    }
    let want = (px * px * 2) as usize;
    let mut raw = Vec::with_capacity(want);
    flate2::read::DeflateDecoder::new(&b[7..]).take(want as u64).read_to_end(&mut raw).ok()?;
    if raw.len() != want {
        return None;
    }
    let elev = raw
        .chunks_exact(2)
        .map(|c| {
            let q = i16::from_le_bytes([c[0], c[1]]);
            if q == i16::MIN { f32::NAN } else { q as f32 / 4.0 }
        })
        .collect();
    Some(Grid { n: px, elev })
}

/// Loaded tiles (≈ 230 kB each), shared by the region builds (one job runs
/// at a time; failures are not kept: the region retries later).
static TILES: LazyLock<std::sync::Mutex<std::collections::HashMap<TileKey, Arc<Grid>>>> = LazyLock::new(Default::default);
const TILES_CAP: usize = 96;

fn cached(k: TileKey) -> Option<Arc<Grid>> {
    TILES.lock().unwrap().get(&k).cloned()
}

fn keep(k: TileKey, g: Grid) -> Arc<Grid> {
    let g = Arc::new(g);
    let mut m = TILES.lock().unwrap();
    if m.len() >= TILES_CAP {
        // (Crude but rare: a region needs at most ~40.)
        m.clear();
    }
    m.insert(k, g.clone());
    g
}

// ---------------------------------------------------------------- native

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::*;
    use std::io::Read;
    use std::path::PathBuf;
    use std::sync::Mutex;
    use web_time::{Duration, Instant};

    static FAILED_AT: Mutex<Option<Instant>> = Mutex::new(None);
    const BACKOFF: Duration = Duration::from_secs(60);
    /// Disk budget for `cache/bathy/` (oldest pruned at startup).
    const CACHE_LIMIT: u64 = 300_000_000;

    static AGENT: LazyLock<ureq::Agent> = LazyLock::new(|| {
        std::thread::spawn(prune_cache);
        ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(40)).user_agent("skyward-flightsim/0.2").build()
    });

    pub fn root() -> PathBuf {
        PathBuf::from("cache").join("bathy")
    }

    fn path(k: TileKey) -> PathBuf {
        root().join(k.level.to_string()).join(format!("{}_{}.bin", k.iy, k.ix))
    }

    pub fn get(url: &str) -> Result<Vec<u8>, String> {
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        if FAILED_AT.lock().unwrap().is_some_and(|t| t.elapsed() < BACKOFF) {
            return Err("network unavailable (backing off)".into());
        }
        match AGENT.get(url).call() {
            Ok(r) => {
                let mut b = Vec::new();
                r.into_reader().take(8 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
                Ok(b)
            }
            Err(e) => {
                if let ureq::Error::Transport(_) = &e {
                    *FAILED_AT.lock().unwrap() = Some(Instant::now());
                }
                Err(e.to_string())
            }
        }
    }

    pub fn load(k: TileKey) -> Result<Grid, String> {
        let px = crate::bathy::LEVELS[k.level as usize].px;
        let p = path(k);
        if let Some(g) = std::fs::read(&p).ok().and_then(|b| decode_cache(&b, px)) {
            return Ok(g);
        }
        let mut elev = if in_emodnet(k) { get(&emodnet_url(k)).and_then(|b| decode(&b, px, Svc::Emodnet)).ok() } else { None };
        let mut whole = true;
        if elev.as_ref().is_none_or(|e| e.iter().any(|x| x.is_nan())) {
            match (get(&noaa_url(k)).and_then(|b| decode(&b, px, Svc::Noaa)), &mut elev) {
                (Ok(n), Some(e)) => fill(e, &n),
                (Ok(n), None) => elev = Some(n),
                // EMODnet with gaps, NOAA down: use it, cache it later.
                (Err(_), Some(_)) => whole = false,
                (Err(e), None) => return Err(e),
            }
        }
        let g = Grid { n: px, elev: elev.unwrap_or_default() };
        if whole {
            crate::cog::net::write(&p, &encode_cache(&g));
        }
        Ok(g)
    }

    fn prune_cache() {
        crate::cog::net::prune(root(), CACHE_LIMIT);
    }
}

/// A tile, loading it (cache, then network) if needed. Blocks (job threads).
#[cfg(not(target_arch = "wasm32"))]
pub fn tile(k: TileKey) -> Option<Arc<Grid>> {
    if let Some(g) = cached(k) {
        return Some(g);
    }
    match native::load(k) {
        Ok(g) => Some(keep(k, g)),
        Err(e) => {
            static WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                eprintln!("bathymetry: {e} (the water keeps its plain colour there)");
            }
            None
        }
    }
}

/// A tile in the browser (the tile cache keeps the answers across visits).
#[cfg(target_arch = "wasm32")]
pub async fn tile_web(k: TileKey) -> Option<Arc<Grid>> {
    use crate::platform::web;
    if let Some(g) = cached(k) {
        return Some(g);
    }
    let t = web_time::Duration::from_secs(40);
    let px = super::LEVELS[k.level as usize].px;
    let get = |url: String| async move {
        if crate::platform::offline() {
            return Err("offline mode".to_string());
        }
        let r = web::fetch_cached(&url, t).await?;
        if !(200..300).contains(&r.status) {
            return Err(format!("HTTP {}", r.status));
        }
        Ok(r.body)
    };
    let mut elev = if in_emodnet(k) { get(emodnet_url(k)).await.and_then(|b| decode(&b, px, Svc::Emodnet)).ok() } else { None };
    if elev.as_ref().is_none_or(|e| e.iter().any(|x| x.is_nan())) {
        let n = get(noaa_url(k)).await.and_then(|b| decode(&b, px, Svc::Noaa)).ok()?;
        match &mut elev {
            Some(e) => fill(e, &n),
            None => elev = Some(n),
        }
    }
    Some(keep(k, Grid { n: px, elev: elev? }))
}

/// `SKYWARD_BATHY_DEBUG`: write the region image as a PNG (depth shades,
/// red = shore band, black = not sea) under `cache/bathy/`.
#[cfg(not(target_arch = "wasm32"))]
pub fn debug_dump(img: &super::Image) {
    let r = img.res;
    let mut rgb = Vec::with_capacity((r * r * 3) as usize);
    for p in &img.texels {
        let (d, w) = (super::f16_value(p[0]), super::f16_value(p[1]));
        if w <= 0.0 {
            rgb.extend([0, 0, 0]);
        } else if d <= 0.0 {
            rgb.extend([200, 40, 40]);
        } else {
            // Log depth: 1 m bright .. 4 km dark.
            let t = ((d.max(1.0).ln() / 4000f32.ln()) * 255.0).clamp(0.0, 255.0) as u8;
            rgb.extend([255 - t, 255 - t / 2, 255]);
        }
    }
    let p = native::root().join(format!("debug_level{}.png", img.region.level));
    let _ = std::fs::create_dir_all(native::root());
    if let Some(buf) = image::RgbImage::from_raw(r, r, rgb) {
        let _ = buf.save(&p);
        eprintln!("bathy: wrote {}", p.display());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A big-endian, tiled, uncompressed float32 TIFF like the EMODnet WCS
    /// sends (tiles wider than the image, as GeoServer pads them).
    fn float_tiff(w: u32, h: u32, tw: u32, th: u32, px: impl Fn(u32, u32) -> f32) -> Vec<u8> {
        let (ta, td) = (w.div_ceil(tw), h.div_ceil(th));
        let n = ta * td;
        let tags: u32 = 10;
        let arr = 8 + 2 + tags * 12 + 4;
        let mut data = arr + n * 8;
        let mut out = b"MM".to_vec();
        out.extend(42u16.to_be_bytes());
        out.extend(8u32.to_be_bytes());
        out.extend((tags as u16).to_be_bytes());
        let tag = |out: &mut Vec<u8>, t: u16, typ: u16, cnt: u32, val: u32| {
            out.extend(t.to_be_bytes());
            out.extend(typ.to_be_bytes());
            out.extend(cnt.to_be_bytes());
            if typ == 3 && cnt == 1 {
                out.extend((val as u16).to_be_bytes());
                out.extend([0, 0]);
            } else {
                out.extend(val.to_be_bytes());
            }
        };
        tag(&mut out, 256, 3, 1, w);
        tag(&mut out, 257, 3, 1, h);
        tag(&mut out, 258, 3, 1, 32);
        tag(&mut out, 259, 3, 1, 1);
        tag(&mut out, 277, 3, 1, 1);
        tag(&mut out, 322, 3, 1, tw);
        tag(&mut out, 323, 3, 1, th);
        tag(&mut out, 324, 4, n, arr);
        tag(&mut out, 325, 4, n, arr + n * 4);
        tag(&mut out, 339, 3, 1, 3);
        out.extend(0u32.to_be_bytes());
        let bytes = tw * th * 4;
        for _ in 0..n {
            out.extend(data.to_be_bytes());
            data += bytes;
        }
        for _ in 0..n {
            out.extend(bytes.to_be_bytes());
        }
        for by in 0..td {
            for bx in 0..ta {
                for y in 0..th {
                    for x in 0..tw {
                        let (gx, gy) = (bx * tw + x, by * th + y);
                        out.extend(if gx < w && gy < h { px(gx, gy) } else { 0.0 }.to_be_bytes());
                    }
                }
            }
        }
        out
    }

    #[test]
    fn decodes_float_tiffs_and_no_data() {
        let b = float_tiff(6, 5, 4, 4, |x, y| if x == 5 && y == 4 { 0.0 } else { -(x as f32) - 10.0 * y as f32 });
        let v = decode(&b, 6, Svc::Noaa).unwrap_err();
        assert!(v.contains("unexpected size"), "{v}");
        let b = float_tiff(6, 6, 4, 4, |x, y| if x == 5 && y == 5 { 0.0 } else { -(x as f32) - 10.0 * y as f32 - 1.0 });
        let v = decode(&b, 6, Svc::Emodnet).unwrap();
        assert_eq!(v[0], -1.0);
        assert_eq!(v[6 * 2 + 3], -24.0);
        // EMODnet's no-data zero; NOAA keeps a real zero.
        assert!(v[35].is_nan());
        assert_eq!(decode(&b, 6, Svc::Noaa).unwrap()[35], 0.0);
        // Garbage and truncation are errors, not panics.
        assert!(decode(&b[..b.len() / 2], 6, Svc::Noaa).is_err());
        assert!(decode(b"not a tiff at all......", 6, Svc::Noaa).is_err());
        // Silly values are no data.
        let b = float_tiff(2, 2, 1, 2, |x, _| if x == 0 { -3.4e38 } else { f32::INFINITY });
        assert!(decode(&b, 2, Svc::Noaa).unwrap().iter().all(|x| x.is_nan()));
    }

    #[test]
    fn cache_round_trip() {
        let g = Grid { n: 3, elev: vec![-10.25, 0.0, 12.5, f32::NAN, -10_994.0, 3.0, -0.5, 1.0, 2.0] };
        let b = encode_cache(&g);
        let back = decode_cache(&b, 3).unwrap();
        for (a, b) in g.elev.iter().zip(&back.elev) {
            if a.is_nan() {
                assert!(b.is_nan());
            } else {
                assert!((a.max(-8000.0) - b).abs() <= 0.125, "{a} {b}");
            }
        }
        // Wrong size / corrupt: rejected.
        assert!(decode_cache(&b, 4).is_none());
        assert!(decode_cache(&b[..b.len() - 3], 3).is_none());
        assert!(decode_cache(b"SKBY1", 3).is_none());
    }

    #[test]
    fn urls_and_coverage() {
        let k = TileKey { level: 0, ix: 3, iy: 205 };
        let u = emodnet_url(k);
        assert!(u.contains("bbox=0.75,51.25,1,51.5") && u.contains("width=240&height=240"), "{u}");
        assert!(noaa_url(k).contains("bbox=0.75,51.25,1,51.5&bboxSR=4326"));
        assert!(in_emodnet(k));
        // The Maldives: NOAA only.
        assert!(!in_emodnet(TileKey { level: 0, ix: 294, iy: 16 }));
        let mut a = vec![1.0, f32::NAN];
        fill(&mut a, &[5.0, 6.0]);
        assert_eq!(a, vec![1.0, 6.0]);
    }

    /// Live data (network; `cargo test --release -- --ignored live_`): the
    /// Thames estuary's Knock John Channel is dredged deeper than the banks
    /// beside it; the Maldives atoll lagoon is shallow inside deep ocean.
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_thames_and_maldives() {
        let k = TileKey { level: 0, ix: 4, iy: 206 };
        let px = 240;
        let e = decode(&native::get(&emodnet_url(k)).unwrap(), px, Svc::Emodnet).unwrap();
        let sea = e.iter().filter(|x| **x < 0.0).count();
        assert!(sea > e.len() / 2, "mostly sea: {sea}");
        let n = decode(&native::get(&noaa_url(TileKey { level: 1, ix: 36, iy: 2 })).unwrap(), px, Svc::Noaa).unwrap();
        let deep = n.iter().filter(|x| **x < -1000.0).count();
        let shallow = n.iter().filter(|x| (-30.0..0.0).contains(*x)).count();
        assert!(deep > 0 && shallow > 0, "{deep} {shallow}");
    }
}
