//! Where ships are: per-class vessel density, fetched per region on demand.
//!
//! **Europe** — EMODnet Human Activities vessel density (CC BY 4.0, Cogea
//! for EMODnet; 1 km, annual average of the monthly maps, by ship type).
//! The unit is hours spent per km² per month, so `density / 730` is the
//! expected number of ships of the type in a km² at any instant — exactly
//! what spawning needs. Served by a GeoServer WCS 2.0.1 in Web Mercator
//! with CORS `*`: one `GetCoverage` per layer and [`Key`] (a z8 slippy-map
//! tile, ≈ 100 km at 50° N) resampled to [`PX`]² float32 GeoTIFF texels,
//! about 64 kB each. Cached under `cache/ships/emodnet/` natively (the
//! browser's tile cache on the web), so a region costs [`LAYERS`]
//! requests once. No data (land, outside the coverage) is NaN.
//!
//! **Elsewhere** (no EMODnet data) — [`generic`]: a thin, plausible mix by
//! depth (cargo and tankers offshore, fishing and leisure near the coast).
//! The World Bank / IMF global shipping density (CC BY 4.0) would fill
//! this in; it is only published as 0.5–1 GB GeoTIFF zips (no subsetting
//! service), so it would need an offline bake into `cache/ships/` first.

use super::types::Class;
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex};

/// Attribution (CC BY 4.0).
pub const CREDIT: &str = "Vessel density: EMODnet Human Activities (Cogea), CC BY 4.0";

/// Slippy-map zoom of the density tiles.
pub const Z: u32 = 8;
/// Texels per tile side (≈ 750 m at 50° N; the data is 1 km).
pub const PX: u32 = 128;

/// The density layers fetched (EMODnet ship-type code) and the class each
/// one spawns ([`spawn_class`] splits some by length).
pub const LAYERS: [(Layer, &str); 8] = [
    (Layer::Cargo, "09"),
    (Layer::Tanker, "10"),
    (Layer::Passenger, "08"),
    (Layer::HighSpeed, "06"),
    (Layer::Fishing, "01"),
    (Layer::Tug, "07"),
    (Layer::Pleasure, "05"),
    (Layer::Sailing, "04"),
];
pub const N: usize = LAYERS.len();

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Layer {
    Cargo,
    Tanker,
    Passenger,
    HighSpeed,
    Fishing,
    Tug,
    Pleasure,
    Sailing,
}

impl Layer {
    pub fn index(self) -> usize {
        LAYERS.iter().position(|l| l.0 == self).unwrap()
    }
}

/// The class a ship spawned from a layer gets (`u` uniform 0..1).
pub fn spawn_class(l: Layer, u: f32) -> Class {
    match l {
        // Box ships are about a third of the cargo traffic around Europe.
        Layer::Cargo => {
            if u < 0.3 {
                Class::Container
            } else if u < 0.36 {
                Class::Barge
            } else {
                Class::Cargo
            }
        }
        Layer::Tanker => Class::Tanker,
        Layer::Passenger => {
            if u < 0.12 {
                Class::Cruise
            } else {
                Class::Passenger
            }
        }
        Layer::HighSpeed => Class::HighSpeed,
        Layer::Fishing => Class::Fishing,
        Layer::Tug => {
            if u < 0.7 {
                Class::Tug
            } else {
                Class::Service
            }
        }
        Layer::Pleasure => Class::Pleasure,
        Layer::Sailing => Class::Sailing,
    }
}

/// A z8 tile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Key {
    pub x: u32,
    pub y: u32,
}

const MERC_HALF: f64 = 20_037_508.342_789_244;

/// Web Mercator metres of (lat, lon).
pub fn merc(lat: f64, lon: f64) -> (f64, f64) {
    let lat = lat.clamp(-85.0, 85.0).to_radians();
    (lon.to_radians() * 6_378_137.0, (std::f64::consts::FRAC_PI_4 + lat * 0.5).tan().ln() * 6_378_137.0)
}

impl Key {
    /// The tile holding (lat, lon) and the position in it (0..1, x east, y south).
    pub fn at(lat: f64, lon: f64) -> (Key, f64, f64) {
        let (mx, my) = merc(lat, lon);
        let n = (1u32 << Z) as f64;
        let fx = (mx + MERC_HALF) / (2.0 * MERC_HALF) * n;
        let fy = (MERC_HALF - my) / (2.0 * MERC_HALF) * n;
        let (x, y) = (fx.floor().clamp(0.0, n - 1.0), fy.floor().clamp(0.0, n - 1.0));
        (Key { x: x as u32, y: y as u32 }, fx - x, fy - y)
    }
    /// Mercator bounds (x0, y0, x1, y1): west, south, east, north.
    pub fn bounds(self) -> (f64, f64, f64, f64) {
        let size = 2.0 * MERC_HALF / (1u32 << Z) as f64;
        let x0 = -MERC_HALF + self.x as f64 * size;
        let y1 = MERC_HALF - self.y as f64 * size;
        (x0, y1 - size, x0 + size, y1)
    }
}

/// WCS request for one layer of a tile.
pub fn url(code: &str, k: Key) -> String {
    let (x0, y0, x1, y1) = k.bounds();
    format!(
        "https://ows.emodnet-humanactivities.eu/wcs?service=WCS&version=2.0.1&request=GetCoverage&coverageId=emodnet__vesseldensity_{code}avg&format=image/tiff&subset=time(%222024-01-01T00:00:00.000Z%22)&subset=X({x0:.2},{x1:.2})&subset=Y({y0:.2},{y1:.2})&scalesize=i({PX}),j({PX})"
    )
}

/// One tile: per layer, expected ships per km² ([`PX`]², row 0 = north;
/// NaN = no data).
#[derive(Clone, Debug, PartialEq)]
pub struct Tile {
    /// Where the texels are (Web Mercator): west edge, north edge, texel
    /// width, texel height. GeoServer answers with more than the subset asked
    /// for (it cuts in its native ETRS89-LAEA grid), so this is read from the
    /// GeoTIFF, not assumed from the [`Key`].
    pub geo: [f64; 4],
    pub layers: Vec<Vec<f32>>,
}

impl Tile {
    /// True if the tile has any data (else: outside the coverage).
    pub fn any(&self) -> bool {
        self.layers.iter().any(|l| l.iter().any(|v| !v.is_nan()))
    }
    /// Nearest-texel value of layer `l` at Web Mercator (mx, my); NaN outside.
    pub fn at(&self, l: usize, mx: f64, my: f64) -> f32 {
        let [x0, y0, sx, sy] = self.geo;
        let (i, j) = (((mx - x0) / sx).floor(), ((y0 - my) / sy).floor());
        if i < 0.0 || j < 0.0 || i >= PX as f64 || j >= PX as f64 {
            return f32::NAN;
        }
        self.layers[l][j as usize * PX as usize + i as usize]
    }
}

/// Decode a WCS answer (float32 GeoTIFF, [`PX`]²) into ships per km² and
/// its georeferencing ([`Tile::geo`]).
pub fn decode(bytes: &[u8]) -> Result<(Vec<f32>, [f64; 4]), String> {
    use crate::cog::tiff;
    let (big_endian, ifds) = match tiff::parse(bytes)? {
        tiff::Parse::Ok { big_endian, ifds } => (big_endian, ifds),
        tiff::Parse::Need(_) => return Err("truncated TIFF".into()),
    };
    let ifd = &ifds[0];
    if ifd.width != PX || ifd.height != PX {
        return Err(format!("unexpected size {}×{}", ifd.width, ifd.height));
    }
    let (Some(sc), Some(tie)) = (ifd.scale, ifd.tie) else { return Err("TIFF without georeferencing".into()) };
    if !(sc[0] > 0.0 && sc[1] > 0.0) {
        return Err("bad TIFF georeferencing".into());
    }
    let geo = [tie[3] - tie[0] * sc[0], tie[4] + tie[1] * sc[1], sc[0], sc[1]];
    let mut v = crate::bathy::source::read_f32(bytes, big_endian, ifd)?;
    for x in &mut v {
        // (GeoServer's no-data is -3.4e38.)
        *x = if x.is_finite() && *x >= 0.0 && *x < 1.0e6 { *x / 730.0 } else { f32::NAN };
    }
    Ok((v, geo))
}

/// Disk form: "SKSD2", px (u16), layers (u8), geo (4 × f64), then DEFLATE'd f16 per texel.
const MAGIC: &[u8; 5] = b"SKSD2";

pub fn encode_cache(t: &Tile) -> Vec<u8> {
    use std::io::Write;
    let mut raw = Vec::new();
    for l in &t.layers {
        for &v in l {
            raw.extend(crate::bathy::f16_bits(v).to_le_bytes());
        }
    }
    let mut out = MAGIC.to_vec();
    out.extend((PX as u16).to_le_bytes());
    out.push(t.layers.len() as u8);
    for g in t.geo {
        out.extend(g.to_le_bytes());
    }
    let mut z = flate2::write::DeflateEncoder::new(out, flate2::Compression::default());
    let _ = z.write_all(&raw);
    z.finish().unwrap_or_default()
}

pub fn decode_cache(b: &[u8]) -> Option<Tile> {
    use std::io::Read;
    if b.len() < 40 || &b[..5] != MAGIC || u16::from_le_bytes([b[5], b[6]]) as u32 != PX || b[7] as usize != N {
        return None;
    }
    let mut geo = [0.0f64; 4];
    for (k, g) in geo.iter_mut().enumerate() {
        *g = f64::from_le_bytes(b[8 + 8 * k..16 + 8 * k].try_into().ok()?);
    }
    let per = (PX * PX) as usize;
    let want = per * N * 2;
    let mut raw = Vec::with_capacity(want);
    flate2::read::DeflateDecoder::new(&b[40..]).take(want as u64).read_to_end(&mut raw).ok()?;
    if raw.len() != want {
        return None;
    }
    let vals: Vec<f32> = raw
        .chunks_exact(2)
        .map(|c| {
            let h = u16::from_le_bytes([c[0], c[1]]);
            if h & 0x7c00 == 0x7c00 { f32::NAN } else { crate::bathy::f16_value(h) }
        })
        .collect();
    Some(Tile { geo, layers: vals.chunks(per).map(|c| c.to_vec()).collect() })
}

// ---------------------------------------------------------------- state

enum Entry {
    Loading,
    /// None: no data there (outside EMODnet).
    Ready(Option<Arc<Tile>>),
    Failed(web_time::Instant),
}

static TILES: LazyLock<Mutex<HashMap<Key, Entry>>> = LazyLock::new(Default::default);

/// Off with `SKYWARD_NO_SHIP_DENSITY` (generic density only) and in unit tests.
pub fn enabled() -> bool {
    static ON: LazyLock<bool> = LazyLock::new(|| !cfg!(test) && std::env::var_os("SKYWARD_NO_SHIP_DENSITY").is_none());
    *ON
}

/// Tiles within `radius_m` of (lat, lon).
pub fn keys_around(lat: f64, lon: f64, radius_m: f64) -> Vec<Key> {
    let dlat = radius_m / crate::osm::M_PER_DEG_LAT;
    let dlon = radius_m / (crate::osm::M_PER_DEG_LAT * lat.to_radians().cos().max(0.05));
    let mut v = Vec::new();
    for (a, b) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0), (0.0, 0.0)] {
        let k = Key::at(lat + a * dlat, lon + b * dlon).0;
        if !v.contains(&k) {
            v.push(k);
        }
    }
    v
}

/// Start loading the tiles around (lat, lon). Cheap; call often.
pub fn set_focus(lat: f64, lon: f64, radius_m: f64) {
    if !enabled() {
        return;
    }
    for k in keys_around(lat, lon, radius_m) {
        let start = {
            let mut m = TILES.lock().unwrap();
            match m.get(&k) {
                Some(Entry::Ready(_) | Entry::Loading) => false,
                Some(Entry::Failed(t)) if t.elapsed().as_secs() < 120 => false,
                _ => {
                    m.insert(k, Entry::Loading);
                    true
                }
            }
        };
        if start {
            load(k);
        }
    }
}

fn finish(k: Key, r: Result<Option<Tile>, String>) {
    if let Err(e) = &r {
        eprintln!("ships: density tile {}/{}: {e}", k.x, k.y);
    }
    let e = match r {
        Ok(t) => Entry::Ready(t.filter(|t| t.any()).map(Arc::new)),
        Err(_) => Entry::Failed(web_time::Instant::now()),
    };
    let mut m = TILES.lock().unwrap();
    // A few hundred kB each: keep the newest few dozen.
    if m.len() > 48 {
        m.retain(|_, e| matches!(e, Entry::Loading));
    }
    m.insert(k, e);
}

#[cfg(not(target_arch = "wasm32"))]
fn load(k: Key) {
    if !crate::platform::spawn("ship-density", move || finish(k, native::tile(k))) {
        TILES.lock().unwrap().remove(&k);
    }
}

#[cfg(target_arch = "wasm32")]
fn load(k: Key) {
    crate::platform::web::spawn_local(async move {
        let mut layers = Vec::new();
        let mut geo = [0.0; 4];
        for (_, code) in LAYERS {
            let r = match crate::platform::web::fetch_cached(&url(code, k), web_time::Duration::from_secs(40)).await {
                Ok(r) if (200..300).contains(&r.status) => decode(&r.body),
                Ok(r) => Err(format!("HTTP {}", r.status)),
                Err(e) => Err(e),
            };
            match r {
                Ok((v, g)) => {
                    layers.push(v);
                    geo = g;
                }
                Err(e) => {
                    finish(k, Err(e));
                    return;
                }
            }
        }
        finish(k, Ok(Some(Tile { geo, layers })));
    });
}

/// True while no tile around the focus is still loading (shots wait for it).
pub fn idle() -> bool {
    !TILES.lock().unwrap().values().any(|e| matches!(e, Entry::Loading))
}

/// What the density data says at (lat, lon).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sample {
    /// Not loaded (yet), or failed.
    Unknown,
    /// Outside EMODnet: use [`generic`].
    Uncovered,
    /// No data here inside the coverage: land (at 1 km).
    Land,
    /// Ships per km² by layer.
    Ships([f32; N]),
}

pub fn sample(lat: f64, lon: f64) -> Sample {
    if !enabled() {
        return Sample::Uncovered;
    }
    let (k, _, _) = Key::at(lat, lon);
    let m = TILES.lock().unwrap();
    match m.get(&k) {
        Some(Entry::Ready(Some(t))) => {
            let (mx, my) = merc(lat, lon);
            sample_tile(t, mx, my)
        }
        Some(Entry::Ready(None)) => Sample::Uncovered,
        _ => Sample::Unknown,
    }
}

pub fn sample_tile(t: &Tile, mx: f64, my: f64) -> Sample {
    let mut v = [0.0f32; N];
    let mut any = false;
    for (l, out) in v.iter_mut().enumerate() {
        let x = t.at(l, mx, my);
        if !x.is_nan() {
            *out = x;
            any = true;
        }
    }
    if any { Sample::Ships(v) } else { Sample::Land }
}

/// A plausible thin mix where there is no density data: ships per km² by
/// layer at a sea depth (None: unknown — a lake, a river or no bathymetry)
/// and month (1..12; leisure peaks in summer).
pub fn generic(depth: Option<f32>, month: u32) -> [f32; N] {
    let summer = if (5..=9).contains(&month) { 1.0 } else { 0.3 };
    let mut v = [0.0f32; N];
    let mut set = |l: Layer, x: f32| v[l.index()] = x;
    match depth {
        Some(d) if d > 25.0 => {
            set(Layer::Cargo, 0.004);
            set(Layer::Tanker, 0.002);
            set(Layer::Fishing, 0.002);
            set(Layer::Passenger, 0.0005);
            set(Layer::Sailing, 0.0005 * summer);
        }
        Some(_) => {
            set(Layer::Cargo, 0.0015);
            set(Layer::Fishing, 0.004);
            set(Layer::Tug, 0.001);
            set(Layer::Pleasure, 0.008 * summer);
            set(Layer::Sailing, 0.006 * summer);
            set(Layer::Passenger, 0.0008);
        }
        None => {
            set(Layer::Pleasure, 0.004 * summer);
            set(Layer::Sailing, 0.002 * summer);
            set(Layer::Fishing, 0.001);
        }
    }
    v
}

// ---------------------------------------------------------------- native

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::*;
    use std::io::Read;
    use std::path::PathBuf;
    use web_time::Duration;

    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(40)).user_agent("skyward-flightsim/0.2").build());
    /// One tile's layers are fetched one after another (polite), tiles in parallel threads.
    static ONE: Mutex<()> = Mutex::new(());

    fn path(k: Key) -> PathBuf {
        PathBuf::from("cache").join("ships").join("emodnet").join(Z.to_string()).join(format!("{}_{}.bin", k.x, k.y))
    }

    pub fn tile(k: Key) -> Result<Option<Tile>, String> {
        let p = path(k);
        if let Some(t) = crate::cog::net::read(&p).and_then(|b| decode_cache(&b)) {
            return Ok(Some(t));
        }
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        let _one = ONE.lock().unwrap_or_else(|e| e.into_inner());
        let mut layers = Vec::new();
        let mut geo = [0.0; 4];
        for (_, code) in LAYERS {
            let r = AGENT.get(&url(code, k)).call().map_err(|e| e.to_string())?;
            let mut b = Vec::new();
            r.into_reader().take(4 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
            let (v, g) = decode(&b)?;
            layers.push(v);
            geo = g;
        }
        let t = Tile { geo, layers };
        crate::cog::net::write(&p, &encode_cache(&t));
        Ok(Some(t))
    }
}
