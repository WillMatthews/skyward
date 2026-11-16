//! Where the tint raster's imagery comes from, and its caches.
//!
//! * **EOX Sentinel-2 cloudless 2016** (`s2cloudless_3857` on EOX's public
//!   WMTS): 256² JPEG tiles in the Google / OSM tile grid, CC BY 4.0. EOX's
//!   FAQ allows apps on the free 2016 endpoint "provided traffic is
//!   reasonable", asks clients to cache and to spread requests out: we read
//!   level `z − 2` (one request per 16 map tiles), keep every tile on disk,
//!   allow two requests in flight, and back off for a minute after a failure.
//!   The open ocean answers with a tiny transparent PNG (no data).
//! * **NASA GIBS Blue Marble: Next Generation** (public domain, ≈ 500 m,
//!   levels ≤ 8): the fallback when EOX fails (or with
//!   `SKYWARD_SATTINT_SOURCE=gibs`); deeper levels crop and upsample the
//!   level-8 ancestor.
//!
//! Native: blocking fetches on the tile workers, raw tiles under
//! `cache/sattint/` (pruned to [`CACHE_LIMIT`]), reduced [`TintTile`]s in a
//! load-once LRU. Browser: [`web::ensure`] fetches (through Cache Storage)
//! before a build; the build reads what is loaded.

use super::{Key, TEX, TintTile};
use std::sync::{Arc, LazyLock};

const EOX: &str = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g";
const GIBS: &str = "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/BlueMarble_NextGeneration/default/GoogleMapsCompatible_Level8";
/// GIBS Blue Marble's deepest level.
const GIBS_MAX: u32 = 8;
/// Pixels per side of a WMTS tile.
const SIDE: usize = TEX * 4;

/// Logged with the first failed load.
const FAIL: &str = "sattint (ground keeps its procedural colours)";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Src {
    Eox,
    Gibs,
}

impl Src {
    fn url(self, (z, x, y): Key) -> String {
        match self {
            Src::Eox => format!("{EOX}/{z}/{y}/{x}.jpg"),
            Src::Gibs => format!("{GIBS}/{z}/{y}/{x}.jpeg"),
        }
    }
    fn dir(self) -> &'static str {
        match self {
            Src::Eox => "eox2016",
            Src::Gibs => "gibs-bmng",
        }
    }
}

fn eox_first() -> bool {
    static EOX_ON: LazyLock<bool> = LazyLock::new(|| std::env::var("SKYWARD_SATTINT_SOURCE").map_or(true, |s| !s.eq_ignore_ascii_case("gibs")));
    *EOX_ON
}

/// Decode a WMTS tile to 256² RGBA; `None` if it is not one.
fn decode(bytes: &[u8]) -> Option<Vec<u8>> {
    let img = image::load_from_memory(bytes).ok()?.to_rgba8();
    (img.width() as usize == SIDE && img.height() as usize == SIDE).then(|| img.into_raw())
}

/// The `key` area cut out of its ancestor at a shallower level and
/// upsampled (bilinear) to a full tile.
fn upsample(anc: &[u8], (z, x, y): Key, az: u32) -> Vec<u8> {
    let d = z - az;
    let span = SIDE as f32 / (1u32 << d) as f32;
    let (ox, oy) = ((x % (1 << d)) as f32 * span, (y % (1 << d)) as f32 * span);
    let mut out = vec![0u8; SIDE * SIDE * 4];
    let px = |x: usize, y: usize| &anc[(y.min(SIDE - 1) * SIDE + x.min(SIDE - 1)) * 4..][..4];
    for j in 0..SIDE {
        for i in 0..SIDE {
            let fx = (ox + (i as f32 + 0.5) / SIDE as f32 * span - 0.5).max(0.0);
            let fy = (oy + (j as f32 + 0.5) / SIDE as f32 * span - 0.5).max(0.0);
            let (x0, y0) = (fx as usize, fy as usize);
            let (tx, ty) = (fx - x0 as f32, fy - y0 as f32);
            let (a, b, c, e) = (px(x0, y0), px(x0 + 1, y0), px(x0, y0 + 1), px(x0 + 1, y0 + 1));
            for k in 0..4 {
                let v = (a[k] as f32 * (1.0 - tx) + b[k] as f32 * tx) * (1.0 - ty) + (c[k] as f32 * (1.0 - tx) + e[k] as f32 * tx) * ty;
                out[(j * SIDE + i) * 4 + k] = v.round() as u8;
            }
        }
    }
    out
}

/// Reduce a tile image to texels, with land-cover classes where available.
fn reduce(rgba: &[u8], src: Src, (z, x, y): Key) -> TintTile {
    let use_wc = crate::landcover::enabled();
    // Texel size (m) at the tile's latitude: the WorldCover overview to read.
    let (lat, _) = super::texel_latlon((x as usize * TEX) as f64 + 32.0, (y as usize * TEX) as f64 + 32.0, z);
    let texel_m = 40_075_016.0 / ((TEX as f64) * (1u64 << z) as f64) * lat.to_radians().cos();
    let mut wc = crate::landcover::worldcover::Sampler::new(texel_m as f32);
    TintTile::from_image(rgba, src == Src::Gibs, |tx, ty| {
        if !use_wc {
            return crate::landcover::class::NONE;
        }
        let (la, lo) = super::texel_latlon((x as usize * TEX + tx) as f64 + 0.5, (y as usize * TEX + ty) as f64 + 0.5, z);
        wc.at(la, lo)
    })
}

static TILES: LazyLock<crate::cog::Store<Key, TintTile>> = LazyLock::new(|| crate::cog::Store::new(160, FAIL));

/// The reduced tile for a key (native: loads it, blocking; browser: only if
/// [`web::ensure`] loaded it). `None`: no imagery.
pub fn tile(key: Key) -> Option<Arc<TintTile>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        TILES.get(&key, || Ok(image(key).map(|(img, src)| reduce(&img, src, key))))
    }
    #[cfg(target_arch = "wasm32")]
    {
        web::loaded(key)
    }
}

// ---------------------------------------------------------------- native

/// Disk budget for everything under `cache/sattint/` (oldest pruned at startup).
#[cfg(not(target_arch = "wasm32"))]
const CACHE_LIMIT: u64 = 300_000_000;

/// The RGBA image of a WMTS tile and its source, EOX first, else GIBS;
/// `None` if neither has it (or the network is down and nothing is cached).
#[cfg(not(target_arch = "wasm32"))]
pub fn image(key: Key) -> Option<(Vec<u8>, Src)> {
    if eox_first() {
        match native::fetch(Src::Eox, key) {
            Ok(Some(b)) => {
                // Unreadable / partial answers (the ocean's empty PNG decodes fine).
                if let Some(img) = decode(&b) {
                    return Some((img, Src::Eox));
                }
            }
            Ok(None) => return None,
            Err(_) => {}
        }
    }
    gibs_image(key).map(|img| (img, Src::Gibs))
}

#[cfg(not(target_arch = "wasm32"))]
fn gibs_image(key: Key) -> Option<Vec<u8>> {
    static ANC: LazyLock<crate::cog::Store<Key, Vec<u8>>> = LazyLock::new(|| crate::cog::Store::new(16, FAIL));
    let (z, x, y) = key;
    let az = z.min(GIBS_MAX);
    let akey = (az, x >> (z - az), y >> (z - az));
    let anc = ANC.get(&akey, || match native::fetch(Src::Gibs, akey)? {
        Some(b) => Ok(decode(&b)),
        None => Ok(None),
    })?;
    Some(if az == z { anc.to_vec() } else { upsample(&anc, key, az) })
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::{CACHE_LIMIT, Key, Src};
    use std::io::Read;
    use std::path::PathBuf;
    use std::sync::{Condvar, LazyLock, Mutex};
    use web_time::{Duration, Instant};

    /// Requests in flight at once (all sources).
    const PARALLEL: u32 = 2;
    const BACKOFF: Duration = Duration::from_secs(60);

    static SLOTS: (Mutex<u32>, Condvar) = (Mutex::new(0), Condvar::new());
    static FAILED_AT: Mutex<[Option<Instant>; 2]> = Mutex::new([None, None]);
    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(25)).user_agent("skyward-flightsim/0.2").build());

    fn root() -> PathBuf {
        static PRUNE: std::sync::Once = std::sync::Once::new();
        PRUNE.call_once(|| {
            std::thread::spawn(|| crate::cog::net::prune(PathBuf::from("cache").join("sattint"), CACHE_LIMIT));
        });
        PathBuf::from("cache").join("sattint")
    }

    /// A tile's bytes, from the disk cache or the server. `Ok(None)`: the
    /// server has no such tile (remembered on disk).
    pub fn fetch(src: Src, key: Key) -> Result<Option<Vec<u8>>, String> {
        let (z, x, y) = key;
        let dir = root().join(src.dir()).join(z.to_string()).join(x.to_string());
        let (path, none) = (dir.join(format!("{y}.img")), dir.join(format!("{y}.none")));
        if none.exists() {
            return Ok(None);
        }
        if let Some(b) = crate::cog::net::read(&path).filter(|b| !b.is_empty()) {
            return Ok(Some(b));
        }
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        let si = src as usize;
        if FAILED_AT.lock().unwrap()[si].is_some_and(|t| t.elapsed() < BACKOFF) {
            return Err("backing off".into());
        }
        // Politeness: a couple of requests at a time.
        {
            let mut n = SLOTS.0.lock().unwrap();
            while *n >= PARALLEL {
                n = SLOTS.1.wait(n).unwrap();
            }
            *n += 1;
        }
        let r = AGENT.get(&src.url(key)).call();
        *SLOTS.0.lock().unwrap() -= 1;
        SLOTS.1.notify_one();
        match r {
            Ok(r) => {
                let mut b = Vec::new();
                r.into_reader().take(4 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
                crate::cog::net::write(&path, &b);
                Ok(Some(b))
            }
            // No tile there (GIBS answers a bad level with 400).
            Err(ureq::Error::Status(400 | 404, _)) => {
                crate::cog::net::write(&none, b"");
                Ok(None)
            }
            Err(e) => {
                FAILED_AT.lock().unwrap()[si] = Some(Instant::now());
                Err(format!("{}: {e}", src.dir()))
            }
        }
    }
}

// ---------------------------------------------------------------- browser

/// Browser loads: [`ensure`](web::ensure) the keys a build reads (async,
/// through Cache Storage), then build. Both servers send CORS headers.
#[cfg(target_arch = "wasm32")]
pub mod web {
    use super::{Arc, Key, Src, TintTile};
    use std::collections::HashMap;
    use std::sync::Mutex;
    use web_time::{Duration, Instant};

    enum Entry {
        Loading,
        Ready(Option<Arc<TintTile>>),
        Failed(Instant),
    }

    static MAP: Mutex<Option<HashMap<Key, Entry>>> = Mutex::new(None);

    fn with<R>(f: impl FnOnce(&mut HashMap<Key, Entry>) -> R) -> R {
        let mut g = MAP.lock().unwrap();
        f(g.get_or_insert_with(HashMap::new))
    }

    pub(super) fn loaded(key: Key) -> Option<Arc<TintTile>> {
        with(|m| match m.get(&key) {
            Some(Entry::Ready(t)) => t.clone(),
            _ => None,
        })
    }

    async fn get(src: Src, key: Key) -> Result<Option<Vec<u8>>, String> {
        let r = crate::platform::web::fetch_cached(&src.url(key), Duration::from_secs(25)).await?;
        match r.status {
            200..=299 => Ok(super::decode(&r.body)),
            400 | 404 => Ok(None),
            s => Err(format!("HTTP {s}")),
        }
    }

    async fn load(key: Key) -> Result<Option<TintTile>, String> {
        if super::eox_first() {
            match get(Src::Eox, key).await {
                Ok(Some(img)) => return Ok(Some(super::reduce(&img, Src::Eox, key))),
                Ok(None) => return Ok(None),
                Err(_) => {}
            }
        }
        let (z, x, y) = key;
        let az = z.min(super::GIBS_MAX);
        let Some(anc) = get(Src::Gibs, (az, x >> (z - az), y >> (z - az))).await? else { return Ok(None) };
        let img = if az == z { anc } else { super::upsample(&anc, key, az) };
        Ok(Some(super::reduce(&img, Src::Gibs, key)))
    }

    /// Start loading `keys` unless loaded, loading or recently failed.
    pub fn start(keys: &[Key]) {
        if !super::super::enabled() {
            return;
        }
        for &k in keys {
            let start = with(|m| match m.get(&k) {
                Some(Entry::Ready(_) | Entry::Loading) => false,
                Some(Entry::Failed(t)) if t.elapsed() < Duration::from_secs(60) => false,
                _ => {
                    m.insert(k, Entry::Loading);
                    true
                }
            });
            if start {
                crate::platform::web::spawn_local(async move {
                    let e = match load(k).await {
                        Ok(t) => Entry::Ready(t.map(Arc::new)),
                        Err(_) => Entry::Failed(Instant::now()),
                    };
                    with(|m| m.insert(k, e));
                });
            }
        }
    }

    /// [`start`] `keys` and wait until none is still loading.
    pub async fn ensure(keys: &[Key]) {
        start(keys);
        loop {
            if !with(|m| keys.iter().any(|k| matches!(m.get(k), Some(Entry::Loading)))) {
                return;
            }
            crate::platform::web::sleep(Duration::from_millis(25)).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn urls_follow_the_wmts_grids() {
        assert_eq!(Src::Eox.url((12, 2047, 1363)), "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g/12/1363/2047.jpg");
        assert!(Src::Gibs.url((8, 127, 85)).ends_with("/GoogleMapsCompatible_Level8/8/85/127.jpeg"));
    }

    #[test]
    fn upsampling_crops_the_right_quarter() {
        // Ancestor: left half dark, right half bright; the child (d = 1, x odd) is the right half.
        let mut anc = vec![0u8; SIDE * SIDE * 4];
        for (k, p) in anc.chunks_exact_mut(4).enumerate() {
            let v = if k % SIDE >= SIDE / 2 { 200 } else { 20 };
            p.copy_from_slice(&[v, v, v, 255]);
        }
        let right = upsample(&anc, (9, 3, 2), 8);
        // (The first column blends across the cut, like the ancestor's own pixels.)
        assert!(right.chunks_exact(4).enumerate().all(|(k, p)| p[3] == 255 && (k % SIDE < 2 || p[0] == 200)));
        let left = upsample(&anc, (9, 2, 2), 8);
        assert!(left.chunks_exact(4).enumerate().all(|(k, p)| k % SIDE > SIDE - 3 || p[0] == 20));
    }

    #[test]
    fn decode_rejects_non_tiles() {
        assert!(decode(b"not an image").is_none());
        let mut png = Vec::new();
        image::RgbaImage::new(SIDE as u32, SIDE as u32).write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
        let d = decode(&png).unwrap();
        assert_eq!(d.len(), SIDE * SIDE * 4);
        assert!(d.chunks_exact(4).all(|p| p[3] == 0));
    }
}
