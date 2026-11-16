//! Far-field night lights (Track NL, issue #12): NASA Black Marble (VIIRS
//! day/night band, 2016 composite) so the towns and cities beyond the loaded
//! map tiles glow on the horizon at night.
//!
//! **Data**: NASA GIBS WMTS, layer [`LAYER`] (`VIIRS_Night_Lights`: the lights
//! alone, the radiance in the PNG's alpha), Web Mercator
//! (`GoogleMapsCompatible_Level8`, zoom [`Z`] = 8: ≈ 610 m / px at the
//! equator, ≈ 380 m at 51° N). Public domain (NASA); GIBS asks for an
//! acknowledgement ([`CREDIT`]). GIBS sends `Access-Control-Allow-Origin: *`,
//! so the browser build reads it too.
//!
//! **Flow**: [`NightLights::update`] picks a grid around the eye (±[`HALF`]
//! m, re-centred in [`RECENTRE`] steps), fetches the Black Marble tiles it
//! needs in the background (disk cache `cache/nightlights/`, the browser's
//! tile cache on the web), resamples them into a lat/lon-linear grid (the
//! world frames are equirectangular, so the shader maps eye-relative metres
//! to it with one scale + offset), builds the mip chain and uploads it
//! (R16Float). A small mask texture over the same grid marks the area the
//! resident map tiles cover: their own street / far-city lights and skyglow
//! (place.rs, sky.rs) light it, so the Black Marble light fades out there
//! (no double counting).
//!
//! **Use** (`common.wgsl`, `scene.wgsl`): emissive light on far terrain at
//! night; in the sky pass, the light domes above lit areas (single
//! scattering of the upward light, marched along the view ray) and the lit
//! ground beyond the terrain's reach seen from altitude.
//!
//! **Degrades**: day (street lights off), offline with nothing cached,
//! missing tiles (dark there), unit tests or `SKYWARD_NO_NIGHTLIGHTS=1` →
//! nothing added; the frame is exactly as before.

use crate::osm::Proj;
use glam::{Vec2, Vec3};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// GIBS layer, time and tile matrix set.
pub const LAYER: &str = "VIIRS_Night_Lights";
pub const TIME: &str = "2016-01-01";
pub const MATRIX_SET: &str = "GoogleMapsCompatible_Level8";
/// Zoom level read (the layer's deepest).
pub const Z: u32 = 8;
const TILE: usize = 256;

/// Attribution (Credits screen; also README, NOTICE, web/credits.html).
pub const CREDIT: &str = "NASA Black Marble 2016 (Suomi NPP VIIRS; NASA Earth Observatory / Goddard Space Flight Center): imagery from NASA's Global Imagery Browse Services (GIBS), part of NASA's Earth Science Data and Information System (ESDIS) — public domain";

/// Grid texels per side, and its half-size (m).
pub const N: usize = if crate::platform::WEB { 512 } else { 1024 };
pub const HALF: f64 = 200_000.0;
/// The grid is re-centred on a lattice this coarse (m).
pub const RECENTRE: f64 = 40_000.0;
/// Coverage mask texels per side.
pub const MASK_N: usize = 128;
/// Upward luminance (cd/m²) of a fully lit Black Marble pixel (alpha 255):
/// matched to the city model's upward radiance over central London (sky.rs).
pub const PEAK_CD: f32 = 0.25;

/// Disabled: tests, or `SKYWARD_NO_NIGHTLIGHTS`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| !cfg!(test) && std::env::var_os("SKYWARD_NO_NIGHTLIGHTS").is_none());
    *ON
}

/// The tile's URL (GIBS REST: {TileMatrix}/{TileRow}/{TileCol}).
pub fn tile_url(x: u32, y: u32) -> String {
    format!("https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/{LAYER}/default/{TIME}/{MATRIX_SET}/{Z}/{y}/{x}.png")
}

/// Web Mercator pixel coordinates of a lat/lon at zoom [`Z`] (lat clamped to the projection's ±85.05°).
pub fn merc_px(lat: f64, lon: f64) -> (f64, f64) {
    let world = (TILE << Z) as f64;
    let lat = lat.clamp(-85.051_128, 85.051_128).to_radians();
    let x = (lon + 180.0).rem_euclid(360.0) / 360.0 * world;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) * 0.5 * world;
    (x, y.clamp(0.0, world - 1e-6))
}

/// Black Marble alpha (display-stretched, ~ sRGB) → relative radiance 0..1.
pub fn radiance(a: u8) -> f32 {
    (a as f32 / 255.0).powf(2.2)
}

/// A tile PNG → its 256 x 256 alpha plane. None if it is not one (untrusted input).
pub fn decode(bytes: &[u8]) -> Option<Vec<u8>> {
    if bytes.len() > 4 << 20 {
        return None;
    }
    let mut r = image::ImageReader::with_format(std::io::Cursor::new(bytes), image::ImageFormat::Png);
    let mut lim = image::Limits::default();
    lim.max_image_width = Some(TILE as u32);
    lim.max_image_height = Some(TILE as u32);
    lim.max_alloc = Some(8 << 20);
    r.limits(lim);
    let img = r.decode().ok()?;
    if img.width() as usize != TILE || img.height() as usize != TILE {
        return None;
    }
    // (Lights-only layer: the radiance is the alpha; an opaque tile would be the wrong layer.)
    let a: Vec<u8> = img.to_rgba8().pixels().map(|p| p.0[3]).collect();
    Some(a)
}

// ---------------------------------------------------------------- grid

/// A lat/lon-linear grid of [`N`]² texels: north-west corner and texel size (degrees).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridSpec {
    pub lat_n: f64,
    pub lon_w: f64,
    pub dlat: f64,
    pub dlon: f64,
    pub n: usize,
}

impl GridSpec {
    /// The grid for an eye at `lat`, `lon`: centred on the nearest
    /// [`RECENTRE`] lattice point (so it changes only every few tens of km).
    pub fn around(lat: f64, lon: f64, n: usize) -> GridSpec {
        let m = crate::osm::M_PER_DEG_LAT;
        let slat = RECENTRE / m;
        let clat = ((lat / slat).round() * slat).clamp(-84.0, 84.0);
        let mlon = m * clat.to_radians().cos().max(0.05);
        let slon = RECENTRE / mlon;
        let clon = (lon / slon).round() * slon;
        let dlat = 2.0 * HALF / m / n as f64;
        let dlon = 2.0 * HALF / mlon / n as f64;
        GridSpec { lat_n: clat + dlat * n as f64 * 0.5, lon_w: clon - dlon * n as f64 * 0.5, dlat, dlon, n }
    }
    pub fn centre(&self) -> (f64, f64) {
        (self.lat_n - self.dlat * self.n as f64 * 0.5, self.lon_w + self.dlon * self.n as f64 * 0.5)
    }
    /// Grid uv (0..1, v down from the north edge) of a lat/lon.
    pub fn uv(&self, lat: f64, lon: f64) -> (f64, f64) {
        ((lon - self.lon_w) / (self.dlon * self.n as f64), (self.lat_n - lat) / (self.dlat * self.n as f64))
    }
    /// The Black Marble tiles (x, y at zoom [`Z`]) the grid covers.
    pub fn tiles(&self) -> Vec<(u32, u32)> {
        let lat_s = self.lat_n - self.dlat * self.n as f64;
        let lon_e = self.lon_w + self.dlon * self.n as f64;
        let (_, y0) = merc_px(self.lat_n, 0.0);
        let (_, y1) = merc_px(lat_s, 0.0);
        let count = 1i64 << Z;
        let (x0, x1) = (((self.lon_w + 180.0) / 360.0 * count as f64).floor() as i64, ((lon_e + 180.0) / 360.0 * count as f64).floor() as i64);
        let (ty0, ty1) = ((y0 / TILE as f64) as i64, (y1 / TILE as f64) as i64);
        let mut out = Vec::new();
        for ty in ty0.max(0)..=ty1.min(count - 1) {
            for tx in x0..=x1 {
                out.push((tx.rem_euclid(count) as u32, ty as u32));
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Alpha planes of loaded tiles (None: no data there).
pub type Tiles = HashMap<(u32, u32), Option<Arc<Vec<u8>>>>;

/// Resample the tiles into the grid (relative radiance, row-major, north row first).
pub fn build(spec: &GridSpec, tiles: &Tiles) -> Vec<f32> {
    let n = spec.n;
    let world = (TILE << Z) as i64;
    // Mercator pixel x per column, y per row (separable).
    let xs: Vec<f64> = (0..n).map(|i| merc_px(0.0, spec.lon_w + (i as f64 + 0.5) * spec.dlon).0 - 0.5).collect();
    let ys: Vec<f64> = (0..n).map(|j| merc_px(spec.lat_n - (j as f64 + 0.5) * spec.dlat, 0.0).1 - 0.5).collect();
    let at = |px: i64, py: i64| -> f32 {
        let px = px.rem_euclid(world);
        let py = py.clamp(0, world - 1);
        let key = ((px / TILE as i64) as u32, (py / TILE as i64) as u32);
        match tiles.get(&key) {
            Some(Some(a)) => radiance(a[(py as usize % TILE) * TILE + px as usize % TILE]),
            _ => 0.0,
        }
    };
    let mut out = vec![0.0f32; n * n];
    for (j, &y) in ys.iter().enumerate() {
        let (y0, fy) = (y.floor() as i64, (y - y.floor()) as f32);
        for (i, &x) in xs.iter().enumerate() {
            let (x0, fx) = (x.floor() as i64, (x - x.floor()) as f32);
            let a = at(x0, y0) * (1.0 - fx) + at(x0 + 1, y0) * fx;
            let b = at(x0, y0 + 1) * (1.0 - fx) + at(x0 + 1, y0 + 1) * fx;
            out[j * n + i] = a * (1.0 - fy) + b * fy;
        }
    }
    // The layer is upsampled with nearest-neighbour from ≈ 1.5 km cells:
    // soften the blocks (σ ≈ 1.7 texels).
    if n >= 8 {
        for _ in 0..3 {
            blur(&mut out, n);
        }
    }
    out
}

/// One separable [1 4 6 4 1] / 16 pass (edges clamped).
fn blur(g: &mut [f32], n: usize) {
    const K: [f32; 5] = [1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0];
    let mut tmp = vec![0.0f32; n * n];
    let c = |i: usize, d: usize| (i + d).saturating_sub(2).min(n - 1);
    for j in 0..n {
        for i in 0..n {
            tmp[j * n + i] = (0..5).map(|d| K[d] * g[j * n + c(i, d)]).sum();
        }
    }
    for j in 0..n {
        for i in 0..n {
            g[j * n + i] = (0..5).map(|d| K[d] * tmp[c(j, d) * n + i]).sum();
        }
    }
}

/// Box-filtered mip chain (level 0 first), as f16 bits.
pub fn mips(base: &[f32], n: usize) -> Vec<Vec<u16>> {
    let mut levels = vec![base.iter().map(|&v| f16(v)).collect::<Vec<u16>>()];
    let mut cur = base.to_vec();
    let mut s = n;
    while s > 1 {
        let h = s / 2;
        let mut next = vec![0.0f32; h * h];
        for j in 0..h {
            for i in 0..h {
                next[j * h + i] = 0.25 * (cur[2 * j * s + 2 * i] + cur[2 * j * s + 2 * i + 1] + cur[(2 * j + 1) * s + 2 * i] + cur[(2 * j + 1) * s + 2 * i + 1]);
            }
        }
        levels.push(next.iter().map(|&v| f16(v)).collect());
        cur = next;
        s = h;
    }
    levels
}

/// f32 → IEEE half bits (non-negative finite values; tiny ones flush to 0).
pub fn f16(v: f32) -> u16 {
    if v.is_nan() || v <= 0.0 {
        return 0;
    }
    let b = v.min(65504.0).to_bits();
    let e = ((b >> 23) & 0xff) as i32 - 127 + 15;
    if e <= 0 {
        return 0;
    }
    let m = (b >> 13) & 0x3ff;
    // Round to nearest.
    let r = ((e as u32) << 10 | m) + ((b >> 12) & 1);
    r.min(0x7bff) as u16
}

/// Coverage mask over the grid (`MASK_N`²): 255 where a resident map tile
/// (its rectangle `lo..hi` in `proj`'s frame) has its own lights, grown by
/// a texel and softened just outside the tiles (no hard tile-shaped edges).
pub fn mask(spec: &GridSpec, proj: &Proj, rects: &[(Vec2, Vec2)]) -> Vec<u8> {
    let mn = MASK_N as i64;
    let hard = mask_hard(spec, proj, rects);
    let at = |m: &[u8], i: i64, j: i64| m[(j.clamp(0, mn - 1) * mn + i.clamp(0, mn - 1)) as usize];
    // Dilate (3 x 3 max), then two 3 x 3 box blurs.
    let mut cur: Vec<u8> =
        (0..mn * mn).map(|k| (-1..=1).flat_map(|dj| (-1..=1).map(move |di| (di, dj))).map(|(di, dj)| at(&hard, k % mn + di, k / mn + dj)).max().unwrap_or(0)).collect();
    for _ in 0..2 {
        cur = (0..mn * mn)
            .map(|k| {
                let s: u32 = (-1..=1).flat_map(|dj| (-1..=1).map(move |di| (di, dj))).map(|(di, dj)| at(&cur, k % mn + di, k / mn + dj) as u32).sum();
                (s as f32 / 9.0).round() as u8
            })
            .collect();
    }
    // (Never less than the tiles themselves: no Black Marble inside them.)
    cur.iter().zip(&hard).map(|(&c, &h)| c.max(h)).collect()
}

fn mask_hard(spec: &GridSpec, proj: &Proj, rects: &[(Vec2, Vec2)]) -> Vec<u8> {
    let mn = MASK_N;
    let mut out = vec![0u8; mn * mn];
    for (lo, hi) in rects {
        let (la0, lo0) = proj.to_latlon(*lo);
        let (la1, lo1) = proj.to_latlon(*hi);
        let (u0, v0) = spec.uv(la0, lo0);
        let (u1, v1) = spec.uv(la1, lo1);
        // Texels whose centres fall inside.
        let r = |a: f64, b: f64| {
            let (a, b) = (a.min(b) * mn as f64 - 0.5, a.max(b) * mn as f64 - 0.5);
            (a.ceil().max(0.0) as usize, (b.floor().min(mn as f64 - 1.0)) as i64)
        };
        let (i0, i1) = r(u0, u1);
        let (j0, j1) = r(v0, v1);
        if i1 < i0 as i64 || j1 < j0 as i64 {
            continue;
        }
        for j in j0..=j1 as usize {
            out[j * mn + i0..=j * mn + i1 as usize].fill(255);
        }
    }
    out
}

/// The shader's grid mapping for this frame: (u, v of the eye, du / dx, dv / dz per metre).
pub fn uv_map(spec: &GridSpec, proj: &Proj, eye: Vec3) -> [f32; 4] {
    let (lat, lon) = proj.to_latlon(Vec2::new(eye.x, eye.z));
    let (u, v) = spec.uv(lat, lon);
    // Metres per degree of longitude in this frame (equirectangular: exact).
    let (x0, _) = proj.to_world64(proj.lat0, proj.lon0);
    let (x1, _) = proj.to_world64(proj.lat0, proj.lon0 + 1.0);
    let mlon = x1 - x0;
    let n = spec.n as f64;
    [u as f32, v as f32, (1.0 / (spec.dlon * n * mlon)) as f32, (1.0 / (spec.dlat * n * crate::osm::M_PER_DEG_LAT)) as f32]
}

// ---------------------------------------------------------------- downloads

enum Slot {
    Loading,
    Ready(Option<Arc<Vec<u8>>>),
    Failed(web_time::Instant),
}

/// Retry a failed tile after this long.
const RETRY: web_time::Duration = web_time::Duration::from_secs(90);

#[derive(Default)]
struct Shared {
    tiles: HashMap<(u32, u32), Slot>,
    /// A grid built in the background, waiting for upload.
    built: Option<(GridSpec, Vec<Vec<u16>>)>,
    /// Builds in flight, and the grid asked for last (an older build
    /// finishing late is dropped).
    busy: u32,
    latest: Option<GridSpec>,
}

/// Fetch one tile (disk cache → network). `Ok(None)`: no image there.
#[cfg(not(target_arch = "wasm32"))]
fn load_tile(x: u32, y: u32) -> Result<Option<Vec<u8>>, String> {
    use std::io::Read;
    use std::sync::LazyLock;
    use web_time::Duration;
    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(30)).user_agent("skyward-flightsim/0.3").build());
    let path = std::path::PathBuf::from("cache").join("nightlights").join(format!("{Z}")).join(format!("{y}")).join(format!("{x}.png"));
    if let Ok(b) = std::fs::read(&path) {
        if let Some(a) = decode(&b) {
            return Ok(Some(a));
        }
    }
    if crate::platform::offline() {
        return Err("offline mode".into());
    }
    match AGENT.get(&tile_url(x, y)).call() {
        Ok(r) => {
            let mut b = Vec::new();
            r.into_reader().take(4 << 20).read_to_end(&mut b).map_err(|e| e.to_string())?;
            let a = decode(&b).ok_or("not a 256² PNG")?;
            if let Some(d) = path.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let tmp = path.with_extension(format!("tmp{}", std::process::id()));
            if std::fs::write(&tmp, &b).is_ok() {
                let _ = std::fs::rename(&tmp, &path);
            }
            Ok(Some(a))
        }
        Err(ureq::Error::Status(404, _)) => Ok(None),
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(target_arch = "wasm32")]
async fn load_tile(x: u32, y: u32) -> Result<Option<Vec<u8>>, String> {
    if crate::platform::offline() {
        return Err("offline mode".into());
    }
    let r = crate::platform::web::fetch_cached(&tile_url(x, y), web_time::Duration::from_secs(20)).await?;
    match r.status {
        200..=299 => Ok(Some(decode(&r.body).ok_or("not a 256² PNG")?)),
        404 => Ok(None),
        s => Err(format!("HTTP {s}")),
    }
}

/// Fetch the missing tiles of `spec`, then build its grid.
fn job(shared: Arc<Mutex<Shared>>, spec: GridSpec, want: Vec<(u32, u32)>) {
    let finish = move |shared: &Mutex<Shared>| {
        let tiles: Tiles = {
            let s = shared.lock().unwrap();
            spec.tiles().into_iter().map(|k| (k, if let Some(Slot::Ready(a)) = s.tiles.get(&k) { a.clone() } else { None })).collect()
        };
        let any = tiles.values().any(|t| t.is_some());
        let levels = any.then(|| mips(&build(&spec, &tiles), spec.n));
        let mut s = shared.lock().unwrap();
        if let Some(l) = levels.filter(|_| s.latest == Some(spec)) {
            s.built = Some((spec, l));
        }
        s.busy -= 1;
    };
    let store = |shared: &Mutex<Shared>, k: (u32, u32), r: Result<Option<Vec<u8>>, String>| {
        let mut s = shared.lock().unwrap();
        match r {
            Ok(a) => {
                s.tiles.insert(k, Slot::Ready(a.map(Arc::new)));
            }
            Err(e) => {
                if std::env::var_os("SKYWARD_NIGHT_DEBUG").is_some() {
                    eprintln!("nightlights: tile {Z}/{}/{}: {e}", k.1, k.0);
                }
                s.tiles.insert(k, Slot::Failed(web_time::Instant::now()));
            }
        }
    };
    #[cfg(not(target_arch = "wasm32"))]
    {
        // A few connections at once.
        let want = Arc::new(Mutex::new(want));
        std::thread::scope(|sc| {
            for _ in 0..4 {
                let (want, shared) = (want.clone(), shared.clone());
                sc.spawn(move || {
                    while let Some(k) = want.lock().unwrap().pop() {
                        store(&shared, k, load_tile(k.0, k.1));
                    }
                });
            }
        });
        finish(&shared);
    }
    #[cfg(target_arch = "wasm32")]
    crate::platform::web::spawn_local(async move {
        for k in want {
            let r = load_tile(k.0, k.1).await;
            store(&shared, k, r);
        }
        finish(&shared);
    });
}

// ---------------------------------------------------------------- GPU

/// The night-lights grid on the GPU and what feeds it.
pub struct NightLights {
    shared: Arc<Mutex<Shared>>,
    tex: wgpu::Texture,
    view: wgpu::TextureView,
    mask_tex: wgpu::Texture,
    mask_view: wgpu::TextureView,
    /// The grid last asked for, and the one on the GPU.
    asked: Option<GridSpec>,
    shown: Option<GridSpec>,
    /// Coverage-mask key (grid + resident tiles), to skip unchanged uploads.
    mask_key: u64,
}

impl NightLights {
    pub fn new(device: &wgpu::Device) -> Self {
        let n = if enabled() { N as u32 } else { 1 };
        let levels = n.ilog2() + 1;
        let mk = |label, size: u32, mips: u32, format| {
            let t = device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d { width: size, height: size, depth_or_array_layers: 1 },
                mip_level_count: mips,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            let v = t.create_view(&Default::default());
            (t, v)
        };
        let (tex, view) = mk("night lights", n, levels, wgpu::TextureFormat::R16Float);
        let (mask_tex, mask_view) = mk("night lights mask", if enabled() { MASK_N as u32 } else { 1 }, 1, wgpu::TextureFormat::R8Unorm);
        NightLights { shared: Arc::default(), tex, view, mask_tex, mask_view, asked: None, shown: None, mask_key: 0 }
    }

    pub fn view(&self) -> &wgpu::TextureView {
        &self.view
    }
    pub fn mask_view(&self) -> &wgpu::TextureView {
        &self.mask_view
    }

    /// Ask for the grid around `eye` (fetches in the background); upload a
    /// finished one. Returns the shader's uv map, or None (nothing to show yet).
    pub fn update(&mut self, queue: &wgpu::Queue, proj: &Proj, eye: Vec3) -> Option<[f32; 4]> {
        if !enabled() {
            return None;
        }
        let (lat, lon) = proj.to_latlon(Vec2::new(eye.x, eye.z));
        let spec = GridSpec::around(lat, lon, N);
        let retry = {
            let s = self.shared.lock().unwrap();
            s.busy == 0 && spec.tiles().iter().any(|k| matches!(s.tiles.get(k), Some(Slot::Failed(t)) if t.elapsed() > RETRY))
        };
        if self.asked != Some(spec) || retry {
            self.asked = Some(spec);
            if std::env::var_os("SKYWARD_NIGHT_DEBUG").is_some() {
                eprintln!("nightlights: eye at {lat:.3}, {lon:.3}: asking for the grid around {:?}", spec.centre());
            }
            let want: Vec<(u32, u32)> = {
                let mut s = self.shared.lock().unwrap();
                let want: Vec<_> = spec.tiles().into_iter().filter(|k| !matches!(s.tiles.get(k), Some(Slot::Ready(_) | Slot::Loading))).collect();
                for k in &want {
                    s.tiles.insert(*k, Slot::Loading);
                }
                // Keep memory bounded: drop tiles far from this grid (≈ 25–80 of 64 KB).
                if s.tiles.len() > 400 {
                    let keep: std::collections::HashSet<_> = spec.tiles().into_iter().collect();
                    s.tiles.retain(|k, v| keep.contains(k) || matches!(v, Slot::Loading));
                }
                s.busy += 1;
                s.latest = Some(spec);
                want
            };
            let shared = self.shared.clone();
            if !crate::platform::spawn("nightlights", move || job(shared, spec, want)) {
                self.shared.lock().unwrap().busy -= 1;
            }
        }
        if let Some((spec, levels)) = self.shared.lock().unwrap().built.take() {
            for (i, l) in levels.iter().enumerate() {
                let s = (spec.n >> i).max(1) as u32;
                queue.write_texture(
                    wgpu::TexelCopyTextureInfo { texture: &self.tex, mip_level: i as u32, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                    bytemuck::cast_slice(l),
                    wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(s * 2), rows_per_image: Some(s) },
                    wgpu::Extent3d { width: s, height: s, depth_or_array_layers: 1 },
                );
            }
            if std::env::var_os("SKYWARD_NIGHT_DEBUG").is_some() {
                let (lat, lon) = spec.centre();
                eprintln!("nightlights: grid {}² around {lat:.3}, {lon:.3} uploaded", spec.n);
            }
            self.shown = Some(spec);
            self.mask_key = 0;
        }
        self.shown.map(|s| uv_map(&s, proj, eye))
    }

    /// Mark where resident map tiles (rectangles in `proj`'s frame, keyed by
    /// `key`) light the ground themselves.
    pub fn set_mask(&mut self, queue: &wgpu::Queue, proj: &Proj, key: u64, rects: impl FnOnce() -> Vec<(Vec2, Vec2)>) {
        let Some(spec) = self.shown else { return };
        let key = key ^ spec.lat_n.to_bits().rotate_left(7) ^ spec.lon_w.to_bits().rotate_left(29) | 1;
        if key == self.mask_key {
            return;
        }
        self.mask_key = key;
        let m = mask(&spec, proj, &rects());
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &self.mask_tex, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            &m,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(MASK_N as u32), rows_per_image: Some(MASK_N as u32) },
            wgpu::Extent3d { width: MASK_N as u32, height: MASK_N as u32, depth_or_array_layers: 1 },
        );
    }

    /// True when nothing is being fetched or built (headless captures wait for it).
    pub fn idle(&self) -> bool {
        let s = self.shared.lock().unwrap();
        s.busy == 0 && s.built.is_none()
    }

    /// Texel size of the grid (m).
    pub fn texel_m(&self) -> f32 {
        self.shown.map_or(1.0, |s| (s.dlat * crate::osm::M_PER_DEG_LAT) as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mercator_tiles_match_gibs() {
        // London is tile 8/85/127 (row 85, column 127).
        let (x, y) = merc_px(51.5074, -0.1278);
        assert_eq!(((x / 256.0) as u32, (y / 256.0) as u32), (127, 85));
        // Equator / prime meridian is the centre of the world.
        let (x, y) = merc_px(0.0, 0.0);
        assert!((x - 32768.0).abs() < 1e-6 && (y - 32768.0).abs() < 1e-6);
        assert_eq!(tile_url(127, 85), "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/VIIRS_Night_Lights/default/2016-01-01/GoogleMapsCompatible_Level8/8/85/127.png");
    }

    #[test]
    fn grid_recentres_in_steps_and_covers_its_tiles() {
        let a = GridSpec::around(51.50, -0.12, 1024);
        let b = GridSpec::around(51.52, -0.10, 1024);
        assert_eq!(a, b, "small moves keep the grid");
        let (clat, clon) = a.centre();
        assert!((clat - 51.5).abs() < 0.2 && (clon + 0.12).abs() < 0.4, "{clat} {clon}");
        // ±200 km.
        let km = a.dlat * 1024.0 * crate::osm::M_PER_DEG_LAT / 1000.0;
        assert!((km - 400.0).abs() < 1.0, "{km}");
        let t = a.tiles();
        assert!((16..=49).contains(&t.len()), "{}", t.len());
        assert!(t.contains(&(127, 85)) && t.contains(&(128, 85)));
        // Across the antimeridian: columns wrap.
        let w = GridSpec::around(0.0, 179.9, 1024).tiles();
        assert!(w.iter().any(|k| k.0 == 0) && w.iter().any(|k| k.0 == 255));
    }

    #[test]
    fn grid_resamples_tiles() {
        let spec = GridSpec::around(51.5, -0.12, 64);
        // One lit tile (London's), the rest dark.
        let mut tiles: Tiles = spec.tiles().into_iter().map(|k| (k, None)).collect();
        tiles.insert((127, 85), Some(Arc::new(vec![255u8; 256 * 256])));
        let g = build(&spec, &tiles);
        let at = |lat: f64, lon: f64| {
            let (u, v) = spec.uv(lat, lon);
            g[(v * 64.0) as usize * 64 + (u * 64.0) as usize]
        };
        assert!(at(51.2, -0.7) > 0.99, "inside the lit tile");
        assert!(at(51.5, 1.0) < 1e-6, "east of it: dark");
        assert!(at(52.5, -0.5) < 1e-6, "north of it: dark");
    }

    #[test]
    fn decode_rejects_junk_and_reads_alpha() {
        assert!(decode(b"not a png").is_none());
        let mut img = image::RgbaImage::new(256, 256);
        img.put_pixel(3, 2, image::Rgba([255, 255, 255, 200]));
        let mut png = Vec::new();
        img.write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
        let a = decode(&png).unwrap();
        assert_eq!((a.len(), a[2 * 256 + 3], a[0]), (65536, 200, 0));
        let small = image::RgbaImage::new(16, 16);
        let mut png = Vec::new();
        small.write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
        assert!(decode(&png).is_none(), "wrong size");
        assert_eq!(radiance(0), 0.0);
        assert_eq!(radiance(255), 1.0);
    }

    #[test]
    fn half_floats_and_mips() {
        assert_eq!(f16(1.0), 0x3c00);
        assert_eq!(f16(0.5), 0x3800);
        assert_eq!(f16(0.0), 0);
        assert_eq!(f16(-1.0), 0);
        assert_eq!(f16(1e9), 0x7bff);
        let m = mips(&[1.0, 0.0, 0.0, 1.0], 2);
        assert_eq!(m.len(), 2);
        assert_eq!(m[1], vec![0x3800]);
    }

    #[test]
    fn mask_marks_resident_tiles() {
        let proj = Proj::new(51.5, -0.12);
        let spec = GridSpec::around(51.5, -0.12, 1024);
        // A 40 km square around the eye is covered; far away is not.
        let m = mask(&spec, &proj, &[(Vec2::splat(-20_000.0), Vec2::splat(20_000.0))]);
        let at = |p: Vec2| {
            let (lat, lon) = proj.to_latlon(p);
            let (u, v) = spec.uv(lat, lon);
            m[(v * MASK_N as f64) as usize * MASK_N + (u * MASK_N as f64) as usize]
        };
        assert_eq!(at(Vec2::ZERO), 255);
        assert_eq!(at(Vec2::new(15_000.0, -15_000.0)), 255);
        assert_eq!(at(Vec2::new(60_000.0, 0.0)), 0);
        assert_eq!(at(Vec2::new(0.0, -90_000.0)), 0);
        // The uv map agrees with the grid: eye-relative metres → uv.
        let eye = Vec3::new(1000.0, 3000.0, -2000.0);
        let k = uv_map(&spec, &proj, eye);
        let p = Vec2::new(30_000.0, 25_000.0);
        let (lat, lon) = proj.to_latlon(p);
        let (u, v) = spec.uv(lat, lon);
        let (su, sv) = (k[0] + (p.x - eye.x) * k[2], k[1] + (p.y - eye.z) * k[3]);
        assert!((su as f64 - u).abs() < 1e-5 && (sv as f64 - v).abs() < 1e-5, "{su} {u} {sv} {v}");
    }
}
