//! Satellite tint: low-frequency hue / brightness from satellite imagery,
//! applied as a gentle multiplier on the procedural ground colours, so
//! regions read right from the air (arid vs lush, red soils, chalk downs)
//! without baking the imagery's shadows or its blur into the ground.
//!
//! **Data** ([`source`]): EOX Sentinel-2 cloudless **2016** (the CC BY 4.0
//! vintage; 2018+ are CC BY-NC-SA, not usable here) from EOX's public WMTS,
//! with NASA GIBS Blue Marble: Next Generation (public domain, ≈ 500 m) as
//! the fallback when EOX is unreachable. Both send CORS headers, so the
//! browser build has it too.
//!
//! **Tint raster**: a WMTS tile at level `L` (256² JPEG) is reduced to a 64²
//! [`TintTile`]: per texel (4×4 px) the mean satellite colour, the colour our
//! palette's reference imagery would show for its land-cover class
//! ([`expected`]; WorldCover where available) and a weight (0 on water,
//! built-up, snow, clouds, no data). The texels of all tiles at a level form
//! one global raster, so neighbouring map tiles read identical values on
//! their shared edges (no seams).
//!
//! **Per map tile** ([`for_tile`]): a map tile at zoom `z` reads level `z − 2`
//! (16 texels across the tile: ≈ 100 m on a detail tile, ≈ 1.5 km on a far
//! tile). Each node of a 17² grid on the texel corners takes a 3×3-tent blur
//! of the raster (sums of weighted satellite / expected colours), then
//! [`derive`] turns the ratio into a tint: the luminance ratio is compressed
//! and tightly clamped (de-shadowing: hill shade and terrain shadows mostly
//! move brightness, and only a little of it survives), the chroma ratio is
//! clamped, and the result is mixed toward neutral by [`STRENGTH`] and by
//! how much valid imagery the node saw. The tile builder multiplies the
//! colours of natural-ground vertices by it (`scene.rs`; never roads,
//! buildings, water, runways, urban ground or land-cover snow), and the field
//! shader reads its share from the vertex colour (`scene.wgsl`): the tint
//! modulates the seasonal palette, and snow (snow line, snow cover) is mixed
//! in after it, so it still wins.
//!
//! **Degrades**: offline with nothing cached, unreachable servers, ocean,
//! unit tests, or `SKYWARD_NO_SATTINT=1` → no tint (vertex colours untouched).

pub mod source;

use crate::osm::TileId;
use glam::Vec2;

/// Disabled in unit tests or with `SKYWARD_NO_SATTINT`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| !cfg!(test) && std::env::var_os("SKYWARD_NO_SATTINT").is_none());
    *ON
}

/// Attributions (Credits screen; also README, NOTICE, web/credits.html).
pub const EOX_CREDIT: &str = "EOxCloudless 2016 https://cloudless.eox.at by EOX IT Services GmbH (Contains modified Copernicus Sentinel data 2016) — CC BY 4.0";
pub const GIBS_CREDIT: &str = "NASA Blue Marble: Next Generation (NASA Earth Observatory), served by NASA GIBS (ESDIS) — public domain";

/// Texels per side of a [`TintTile`] (4×4 pixels of a 256² WMTS tile each).
pub const TEX: usize = 64;
/// Map tile zoom − WMTS level: 16 texels across a map tile.
pub const LEVEL_DROP: u32 = 2;
/// Grid cells per side of a map tile's tint grid (one per texel).
pub const GRID_N: usize = TEX >> LEVEL_DROP;

/// How strongly the tint applies (0 = off, 1 = the full clamped ratio).
pub const STRENGTH: f32 = 0.8;
/// Luminance ratio: compressed by this power, then clamped (de-shadowing).
const LUM_POW: f32 = 0.5;
const LUM_MIN: f32 = 0.86;
const LUM_MAX: f32 = 1.22;
/// Per-channel chroma ratio clamp.
const CHROMA_MIN: f32 = 0.65;
const CHROMA_MAX: f32 = 1.45;

/// Final per-channel clamp of a tint.
const TINT_MIN: f32 = 0.62;
const TINT_MAX: f32 = 1.45;

/// sRGB-space luma weights (the tint lives in the palette's sRGB space).
const Y: [f32; 3] = [0.2126, 0.7152, 0.0722];

fn luma(c: [f32; 3]) -> f32 {
    c[0] * Y[0] + c[1] * Y[1] + c[2] * Y[2]
}

// ---------------------------------------------------------------- expectation

/// The satellite colour (sRGB 0..1, EOX 2016 mosaic) of each land-cover class
/// where the procedural palette looks right: temperate north-west Europe
/// (fields, grass and woods were tuned on England) and, for bare rock and
/// tundra, the Alps. Measured with `live_calibrate` (cargo test --release --
/// --ignored; ~10–20k texels per class). A tint is the region's colour
/// relative to this, so the mosaic's own colour grading cancels. `gibs`:
/// the fallback imagery, graded quite differently (warmer, far less blue).
pub fn expected(gibs: bool, class: u8) -> [f32; 3] {
    use crate::landcover::class as c;
    if gibs {
        return match class {
            c::TREE | c::MANGROVE => [0.209, 0.233, 0.100],
            c::SHRUB => [0.215, 0.241, 0.104],
            c::GRASS | c::WETLAND => [0.221, 0.249, 0.108],
            c::CROP => [0.301, 0.294, 0.154],
            c::BARE => [0.589, 0.584, 0.542],
            c::MOSS => [0.470, 0.465, 0.403],
            _ => [0.245, 0.259, 0.122],
        };
    }
    match class {
        c::TREE | c::MANGROVE => [0.158, 0.238, 0.209],
        // (Rare in the reference regions: between woods and grass.)
        c::SHRUB => [0.176, 0.262, 0.221],
        c::GRASS | c::WETLAND => [0.193, 0.286, 0.233],
        c::CROP => [0.220, 0.286, 0.240],
        c::BARE => [0.497, 0.483, 0.403],
        c::MOSS => [0.413, 0.404, 0.317],
        // No class (no land cover here, or the browser build): the
        // reference regions' mix of woods, grass and crops.
        _ => [0.196, 0.276, 0.230],
    }
}

/// Weight of a texel for the tint: 0 where the imagery says nothing about
/// the procedural ground we tint (water, built-up, snow / ice) or is
/// unreliable (bright cloud / snow left in the mosaic, very dark water where
/// there is no land cover to tell).
pub fn weight(class: u8, sat: [f32; 3]) -> f32 {
    use crate::landcover::class as c;
    match class {
        c::WATER | c::BUILT | c::SNOW => return 0.0,
        _ => {}
    }
    let l = luma(sat);
    let (mx, mn) = (sat[0].max(sat[1]).max(sat[2]), sat[0].min(sat[1]).min(sat[2]));
    // Clouds / snow: bright and grey.
    if l > 0.62 && mx - mn < 0.12 {
        return 0.0;
    }
    // Water without a class to say so: dark and blue (EOX), or near black (GIBS).
    if class == c::NONE && ((sat[2] > sat[1] && sat[2] > sat[0] && l < 0.3) || l < 0.075) {
        return 0.0;
    }
    1.0
}

// ---------------------------------------------------------------- tint math

/// The tint (per-channel multiplier on sRGB colours) for a region whose
/// imagery averages `sat` where our palette's reference imagery would show
/// `exp`; `conf` 0..1 is how much valid imagery the estimate rests on.
pub fn derive(sat: [f32; 3], exp: [f32; 3], conf: f32) -> [f32; 3] {
    let (ls, le) = (luma(sat).max(1e-3), luma(exp).max(1e-3));
    // Brightness: compressed and tightly clamped, so terrain shading (which
    // moves brightness, not hue) mostly cancels; a chalk down or a desert
    // still reads lighter, a dark heath darker.
    let lum = (ls / le).powf(LUM_POW).clamp(LUM_MIN, LUM_MAX);
    // Hue: the chromaticity ratio, clamped per channel ...
    let mut c = [0f32; 3];
    for i in 0..3 {
        c[i] = ((sat[i] / ls).max(1e-3) / (exp[i] / le).max(1e-3)).clamp(CHROMA_MIN, CHROMA_MAX);
    }
    // ... renormalised so it carries no brightness of its own.
    let k = luma([exp[0] * c[0], exp[1] * c[1], exp[2] * c[2]]) / le;
    let s = STRENGTH * conf.clamp(0.0, 1.0);
    let mut t = [0f32; 3];
    for i in 0..3 {
        t[i] = (1.0 + (c[i] / k * lum - 1.0) * s).clamp(TINT_MIN, TINT_MAX);
    }
    t
}

/// A tint's hue alone (unit luminance): for already-bright ground (bare
/// earth, sand, rock), which a regional brightening would wash out.
pub fn hue_only(t: [f32; 3]) -> [f32; 3] {
    let l = luma(t).max(1e-3);
    t.map(|v| v / l)
}

// ---------------------------------------------------------------- raster

/// A WMTS tile reduced to [`TEX`]² texels: weighted satellite colour
/// (`w·sat`), weighted expected colour (`w·exp`), weight `w`.
#[derive(Clone, Debug)]
pub struct TintTile {
    pub texels: Vec<[f32; 7]>,
}

impl TintTile {
    /// From a 256² RGBA image (sRGB; `gibs`: the fallback imagery) and the
    /// land-cover class of each texel (`class(tx, ty)`, texel centres).
    pub fn from_image(rgba: &[u8], gibs: bool, mut class: impl FnMut(usize, usize) -> u8) -> TintTile {
        let side = TEX * 4;
        assert_eq!(rgba.len(), side * side * 4);
        let mut texels = vec![[0f32; 7]; TEX * TEX];
        for ty in 0..TEX {
            for tx in 0..TEX {
                let (mut s, mut n) = ([0f32; 3], 0u32);
                for py in ty * 4..ty * 4 + 4 {
                    for px in tx * 4..tx * 4 + 4 {
                        let p = &rgba[(py * side + px) * 4..][..4];
                        // Transparent: no data (EOX answers the open ocean with an empty PNG).
                        if p[3] >= 128 {
                            for i in 0..3 {
                                s[i] += p[i] as f32 / 255.0;
                            }
                            n += 1;
                        }
                    }
                }
                if n < 8 {
                    continue;
                }
                let sat = s.map(|v| v / n as f32);
                let cl = class(tx, ty);
                let w = weight(cl, sat);
                if w <= 0.0 {
                    continue;
                }
                let e = expected(gibs, cl);
                texels[ty * TEX + tx] = [sat[0] * w, sat[1] * w, sat[2] * w, e[0] * w, e[1] * w, e[2] * w, w];
            }
        }
        TintTile { texels }
    }
}

/// Global texel coordinates (x east, y south; texel units) of a point at a level.
pub fn merc_texel(lat: f64, lon: f64, level: u32) -> (f64, f64) {
    let n = (TEX << level) as f64;
    let lat = lat.clamp(-85.05, 85.05).to_radians();
    let x = (lon + 180.0) / 360.0 * n;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
    (x, y)
}

/// Latitude / longitude of a global texel position at a level.
pub fn texel_latlon(x: f64, y: f64, level: u32) -> (f64, f64) {
    let n = (TEX << level) as f64;
    let lon = x / n * 360.0 - 180.0;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees();
    (lat, lon)
}

/// WMTS tile key: (level, x, y).
pub type Key = (u32, u32, u32);

/// Reads the global raster at one level, holding the tiles it touched.
pub struct Raster<F: FnMut(Key) -> Option<std::sync::Arc<TintTile>>> {
    level: u32,
    tiles: Vec<(Key, Option<std::sync::Arc<TintTile>>)>,
    load: F,
}

impl<F: FnMut(Key) -> Option<std::sync::Arc<TintTile>>> Raster<F> {
    pub fn new(level: u32, load: F) -> Self {
        Raster { level, tiles: Vec::new(), load }
    }

    /// Texel (x wraps around the globe; outside the poles: nothing).
    fn texel(&mut self, tx: i64, ty: i64) -> [f32; 7] {
        let n = (TEX as i64) << self.level;
        if ty < 0 || ty >= n {
            return [0.0; 7];
        }
        let tx = tx.rem_euclid(n);
        let key = (self.level, (tx / TEX as i64) as u32, (ty / TEX as i64) as u32);
        let i = match self.tiles.iter().position(|t| t.0 == key) {
            Some(i) => i,
            None => {
                let t = (self.load)(key);
                self.tiles.push((key, t));
                self.tiles.len() - 1
            }
        };
        match &self.tiles[i].1 {
            Some(t) => t.texels[(ty as usize % TEX) * TEX + tx as usize % TEX],
            None => [0.0; 7],
        }
    }

    /// The tent-blurred sums at a texel corner (`cx`, `cy`: global corner
    /// indices): texels at offsets −2..1 with weights 1, 3, 3, 1 per axis
    /// (a ≈ 3-texel low-pass, symmetric about the corner).
    pub fn corner(&mut self, cx: i64, cy: i64) -> [f32; 7] {
        const K: [f32; 4] = [1.0, 3.0, 3.0, 1.0];
        let mut out = [0f32; 7];
        for (j, kj) in K.iter().enumerate() {
            for (i, ki) in K.iter().enumerate() {
                let t = self.texel(cx - 2 + i as i64, cy - 2 + j as i64);
                let k = ki * kj / 64.0;
                for c in 0..7 {
                    out[c] += t[c] * k;
                }
            }
        }
        out
    }
}

/// The tint from blurred sums (neutral where there is no valid imagery).
pub fn tint_of(s: [f32; 7]) -> [f32; 3] {
    let w = s[6];
    if w < 1e-3 {
        return [1.0; 3];
    }
    let sat = [s[0] / w, s[1] / w, s[2] / w];
    let exp = [s[3] / w, s[4] / w, s[5] / w];
    // Half the kernel valid is full confidence.
    derive(sat, exp, w * 2.0)
}

// ---------------------------------------------------------------- tile grids

/// A map tile's tint on a ([`GRID_N`] + 1)² node grid over its rectangle (tile frame).
#[derive(Clone, Debug)]
pub struct TintGrid {
    pub lo: Vec2,
    pub size: Vec2,
    /// Node tints, row-major (z then x).
    pub nodes: Vec<[f32; 3]>,
}

impl TintGrid {
    /// Bilinear tint at a point (clamped to the tile).
    pub fn at(&self, p: Vec2) -> [f32; 3] {
        let n = GRID_N;
        let t = ((p - self.lo) / self.size * n as f32).clamp(Vec2::ZERO, Vec2::splat(n as f32));
        let (x0, z0) = ((t.x.floor() as usize).min(n - 1), (t.y.floor() as usize).min(n - 1));
        let (fx, fz) = (t.x - x0 as f32, t.y - z0 as f32);
        let w = n + 1;
        let g = |x: usize, z: usize| self.nodes[z * w + x];
        let (a, b, c, d) = (g(x0, z0), g(x0 + 1, z0), g(x0, z0 + 1), g(x0 + 1, z0 + 1));
        let mut o = [0f32; 3];
        for i in 0..3 {
            o[i] = (a[i] * (1.0 - fx) + b[i] * fx) * (1.0 - fz) + (c[i] * (1.0 - fx) + d[i] * fx) * fz;
        }
        o
    }

    /// True if no node moves the colour noticeably.
    pub fn is_neutral(&self) -> bool {
        self.nodes.iter().all(|t| t.iter().all(|v| (v - 1.0).abs() < 2e-3))
    }
}

/// The WMTS level a map tile reads.
pub fn level_of(id: TileId) -> u32 {
    id.z.saturating_sub(LEVEL_DROP)
}

/// Global corner indices of a map tile's node (i, j) at its level.
fn node_corner(id: TileId, i: usize, j: usize) -> (i64, i64) {
    let per = (TEX >> LEVEL_DROP) as i64;
    (id.x as i64 * per + i as i64, id.y as i64 * per + j as i64)
}

/// The WMTS tiles a map tile's grid reads (its texels plus the blur margin).
pub fn keys_for(id: TileId) -> Vec<Key> {
    let level = level_of(id);
    let n = (TEX as i64) << level;
    let tiles = 1i64 << level;
    let (x0, y0) = node_corner(id, 0, 0);
    let (x1, y1) = node_corner(id, GRID_N, GRID_N);
    let mut out = Vec::new();
    let (ty0, ty1) = ((y0 - 2).max(0) / TEX as i64, (y1 + 1).min(n - 1) / TEX as i64);
    for ty in ty0..=ty1 {
        for tx in (x0 - 2).div_euclid(TEX as i64)..=(x1 + 1).div_euclid(TEX as i64) {
            let k = (level, tx.rem_euclid(tiles) as u32, ty as u32);
            if !out.contains(&k) {
                out.push(k);
            }
        }
    }
    out
}

/// A map tile's tint grid from a raster loader; `None` if it is neutral.
pub fn grid_with(id: TileId, rect: (Vec2, Vec2), load: impl FnMut(Key) -> Option<std::sync::Arc<TintTile>>) -> Option<TintGrid> {
    let mut r = Raster::new(level_of(id), load);
    let w = GRID_N + 1;
    let mut nodes = Vec::with_capacity(w * w);
    for j in 0..w {
        for i in 0..w {
            let (cx, cy) = node_corner(id, i, j);
            nodes.push(tint_of(r.corner(cx, cy)));
        }
    }
    let g = TintGrid { lo: rect.0, size: rect.1 - rect.0, nodes };
    (!g.is_neutral()).then_some(g)
}

/// The tint grid of a map tile (`rect`: its rectangle in the build's frame).
/// Native: blocks on downloads (tile workers only). Browser: reads what
/// [`source::web::ensure`] loaded. `None`: disabled, no imagery, or neutral.
pub fn for_tile(id: TileId, rect: (Vec2, Vec2)) -> Option<TintGrid> {
    if !enabled() {
        return None;
    }
    let t0 = web_time::Instant::now();
    let g = grid_with(id, rect, source::tile);
    if std::env::var_os("SKYWARD_SATTINT_DEBUG").is_some() {
        eprintln!("sattint {}/{}/{}: {:.1} ms{}", id.z, id.x, id.y, t0.elapsed().as_secs_f32() * 1e3, if g.is_some() { "" } else { " (none)" });
    }
    g
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::landcover::class;

    fn close(a: [f32; 3], b: [f32; 3], eps: f32) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < eps)
    }

    #[test]
    fn derive_is_neutral_on_reference_imagery() {
        for cl in [class::TREE, class::GRASS, class::CROP, class::BARE, class::NONE] {
            let e = expected(false, cl);
            assert!(close(derive(e, e, 1.0), [1.0; 3], 1e-4), "{cl}");
        }
        // No confidence: neutral whatever the imagery says.
        assert!(close(derive([0.6, 0.3, 0.1], expected(false, class::CROP), 0.0), [1.0; 3], 1e-6));
        // Hue only: same direction, no brightening.
        let h = hue_only([1.4, 1.1, 0.9]);
        assert!((luma(h) - 1.0).abs() < 1e-5 && h[0] > h[1] && h[1] > h[2]);
    }

    #[test]
    fn derive_clamps_brightness_and_keeps_hue() {
        let e = expected(false, class::CROP);
        // Deep shadow (half as bright, same hue): only a little darker.
        let t = derive(e.map(|v| v * 0.5), e, 1.0);
        let l = luma([e[0] * t[0], e[1] * t[1], e[2] * t[2]]) / luma(e);
        assert!(l > 0.85 && l < 0.95, "{l}");
        assert!((t[0] - t[1]).abs() < 1e-3 && (t[1] - t[2]).abs() < 1e-3, "{t:?}");
        // A blinding white field: capped.
        let t = derive([0.9, 0.9, 0.9], e, 1.0);
        assert!(t.iter().all(|&v| v < 1.6), "{t:?}");
        // Red outback soil on shrubland: redder (and lighter).
        let t = derive([0.55, 0.33, 0.20], expected(false, class::SHRUB), 1.0);
        assert!(t[0] > 1.3 && t[0] / t[2] > 1.35 && t[0] / t[1] > 1.3, "{t:?}");
        // Parched Andalusian cropland: warmer and lighter.
        let t = derive([0.47, 0.42, 0.33], e, 1.0);
        assert!(t[0] > t[1] + 0.1 && luma(t) > 1.05, "{t:?}");
        // Lush Irish grass: greener.
        let t = derive([0.20, 0.33, 0.16], expected(false, class::GRASS), 1.0);
        assert!(t[1] > t[0] && t[1] > t[2], "{t:?}");
    }

    #[test]
    fn weights_skip_what_we_do_not_tint() {
        let green = [0.2, 0.3, 0.15];
        assert_eq!(weight(class::WATER, green), 0.0);
        assert_eq!(weight(class::BUILT, green), 0.0);
        assert_eq!(weight(class::SNOW, green), 0.0);
        assert_eq!(weight(class::CROP, green), 1.0);
        // Cloud / snow left in the mosaic.
        assert_eq!(weight(class::GRASS, [0.85, 0.86, 0.88]), 0.0);
        // Dark blue: water, unless the land cover says otherwise.
        assert_eq!(weight(class::NONE, [0.05, 0.1, 0.2]), 0.0);
        assert_eq!(weight(class::TREE, [0.05, 0.1, 0.2]), 1.0);
        // Chalk / sand stay in.
        assert_eq!(weight(class::BARE, [0.62, 0.58, 0.50]), 1.0);
    }

    #[test]
    fn tint_tile_from_image() {
        let side = TEX * 4;
        let mut img = vec![0u8; side * side * 4];
        for (k, p) in img.chunks_exact_mut(4).enumerate() {
            let (x, y) = (k % side, k / side);
            // Left half red soil, right half transparent (no data); bottom row water class.
            if x < side / 2 {
                p.copy_from_slice(&[140, 84, 51, 255]);
            }
            let _ = y;
        }
        let t = TintTile::from_image(&img, false, |_, ty| if ty == TEX - 1 { class::WATER } else { class::SHRUB });
        let a = t.texels[0];
        assert!((a[6] - 1.0).abs() < 1e-6 && (a[0] - 140.0 / 255.0).abs() < 1e-4);
        assert!((a[3] - expected(false, class::SHRUB)[0]).abs() < 1e-6);
        assert_eq!(t.texels[TEX - 1][6], 0.0);
        assert_eq!(t.texels[(TEX - 1) * TEX][6], 0.0);
    }

    #[test]
    fn mercator_round_trip() {
        for (lat, lon) in [(51.5, -0.12), (-25.3, 131.0), (37.0, -4.5), (0.0, 0.0)] {
            let (x, y) = merc_texel(lat, lon, 12);
            let (la, lo) = texel_latlon(x, y, 12);
            assert!((la - lat).abs() < 1e-9 && (lo - lon).abs() < 1e-9);
        }
        // Texel corners line up with slippy-map tiles: map tile z at level z − 2 spans 16 texels.
        let id = TileId { z: 14, x: 8180, y: 5448 };
        let (x, y) = node_corner(id, 0, 0);
        assert_eq!((x, y), (8180 * 16, 5448 * 16));
        assert_eq!(level_of(id), 12);
    }

    #[test]
    fn keys_cover_the_blur_margin() {
        // In the middle of a WMTS tile: one key.
        let id = TileId { z: 14, x: 4 * 100 + 1, y: 4 * 50 + 2 };
        assert_eq!(keys_for(id), vec![(12, 100, 50)]);
        // On a WMTS tile's corner: its neighbours too.
        let id = TileId { z: 14, x: 4 * 100, y: 4 * 50 };
        let k = keys_for(id);
        assert_eq!(k.len(), 4);
        assert!(k.contains(&(12, 99, 49)) && k.contains(&(12, 100, 50)));
        // Wraps at the antimeridian.
        let id = TileId { z: 10, x: 0, y: 300 };
        assert!(keys_for(id).iter().any(|k| k.1 == (1 << 8) - 1));
    }

    fn solid(sat: [f32; 3], cl: u8) -> std::sync::Arc<TintTile> {
        let w = weight(cl, sat);
        let e = expected(false, cl);
        std::sync::Arc::new(TintTile { texels: vec![[sat[0] * w, sat[1] * w, sat[2] * w, e[0] * w, e[1] * w, e[2] * w, w]; TEX * TEX] })
    }

    #[test]
    fn grids_tint_and_match_across_edges() {
        let red = [0.55, 0.33, 0.20];
        let rect = (Vec2::new(-100.0, -100.0), Vec2::new(100.0, 100.0));
        // Uniform imagery: a uniform tint.
        let g = grid_with(TileId { z: 14, x: 401, y: 202 }, rect, |_| Some(solid(red, class::SHRUB))).unwrap();
        let t = derive(red, expected(false, class::SHRUB), 1.0);
        assert!(close(g.at(Vec2::ZERO), t, 1e-4) && close(g.at(Vec2::new(99.0, -99.0)), t, 1e-4));
        // Reference imagery or none at all: neutral → no grid.
        assert!(grid_with(TileId { z: 14, x: 401, y: 202 }, rect, |_| Some(solid(expected(false, class::CROP), class::CROP))).is_none());
        assert!(grid_with(TileId { z: 14, x: 401, y: 202 }, rect, |_| None).is_none());
        // Two different WMTS tiles meet between map tiles (x 403 | 404): the
        // shared edge has the same tint on both sides.
        let load = |k: Key| Some(if k.1 == 100 { solid(red, class::SHRUB) } else { solid([0.2, 0.33, 0.16], class::GRASS) });
        let a = grid_with(TileId { z: 14, x: 403, y: 202 }, rect, load).unwrap();
        let b = grid_with(TileId { z: 14, x: 404, y: 202 }, rect, load).unwrap();
        for j in 0..=GRID_N {
            let w = GRID_N + 1;
            assert!(close(a.nodes[j * w + GRID_N], b.nodes[j * w], 1e-6));
        }
        // And the tint changes smoothly: the edge is between the two looks (blue: unclamped here).
        let between = |c: usize| (a.nodes[GRID_N][c] - a.nodes[0][c]) * (a.nodes[GRID_N][c] - b.nodes[GRID_N][c]) < 0.0;
        assert!(between(2), "{:?} {:?} {:?}", a.nodes[0], a.nodes[GRID_N], b.nodes[GRID_N]);
    }

    /// Live data (network; `cargo test --release -- --ignored live_`): the
    /// EOX 2016 mosaic answers, decodes, and tints the outback redder than
    /// the Irish midlands.
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_tints_contrasting_regions() {
        let tint = |lat: f64, lon: f64| {
            let (x, y) = merc_texel(lat, lon, 8);
            let id = TileId { z: 10, x: (x / 16.0) as u32, y: (y / 16.0) as u32 };
            let g = grid_with(id, (Vec2::ZERO, Vec2::ONE), source::tile).expect("tint");
            g.at(Vec2::splat(0.5))
        };
        let outback = tint(-25.0, 133.0);
        let ireland = tint(53.4, -7.8);
        eprintln!("outback {outback:?} ireland {ireland:?}");
        // Outback: red. (Also with SKYWARD_SATTINT_SOURCE=gibs.)
        assert!(outback[0] > 1.3 && outback[0] > outback[1] + 0.2 && outback[0] > outback[2] + 0.15, "{outback:?}");
        // Ireland looks much like the reference: near neutral.
        assert!(ireland.iter().all(|v| (v - 1.0).abs() < 0.25), "{ireland:?}");
    }

    /// Calibration of [`expected`] (prints per-class mean imagery colours over
    /// south-east England, northern France and the Low Countries).
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_calibrate() {
        let mut sums = std::collections::BTreeMap::<u8, ([f64; 3], u64)>::new();
        let mut spots =
            vec![(51.2, -0.5), (51.6, -1.3), (52.2, 0.2), (50.9, 0.9), (52.0, -2.0), (49.9, 2.3), (50.5, 3.6), (51.8, 5.5), (52.5, 6.2), (53.0, -1.0), (51.0, -3.0), (50.7, -2.4)];
        // Or SKYWARD_SATTINT_SPOTS="lat,lon;lat,lon".
        if let Ok(s) = std::env::var("SKYWARD_SATTINT_SPOTS") {
            spots = s.split(';').filter_map(|p| p.split_once(',')).filter_map(|(a, b)| Some((a.trim().parse().ok()?, b.trim().parse().ok()?))).collect();
        }
        for (lat, lon) in spots {
            let (x, y) = merc_texel(lat, lon, 12);
            let key = (12, (x / TEX as f64) as u32, (y / TEX as f64) as u32);
            let Some((img, _)) = source::image(key) else { continue };
            let side = TEX * 4;
            let mut wc = crate::landcover::worldcover::Sampler::new(40.0);
            for ty in 0..TEX {
                for tx in 0..TEX {
                    let (la, lo) = texel_latlon((key.1 as usize * TEX + tx) as f64 + 0.5, (key.2 as usize * TEX + ty) as f64 + 0.5, 12);
                    let cl = wc.at(la, lo);
                    let mut s = [0f64; 3];
                    for py in ty * 4..ty * 4 + 4 {
                        for px in tx * 4..tx * 4 + 4 {
                            for i in 0..3 {
                                s[i] += img[(py * side + px) * 4 + i] as f64 / 255.0 / 16.0;
                            }
                        }
                    }
                    let e = sums.entry(cl).or_default();
                    for i in 0..3 {
                        e.0[i] += s[i];
                    }
                    e.1 += 1;
                }
            }
        }
        for (cl, (s, n)) in sums {
            eprintln!("class {cl:3}: n {n:6} mean [{:.3}, {:.3}, {:.3}]", s[0] / n as f64, s[1] / n as f64, s[2] / n as f64);
        }
    }
}
