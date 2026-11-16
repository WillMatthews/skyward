//! Land cover and canopy height: where OpenStreetMap has no landuse, what the
//! ground is (forest, shrub, grass, crops, built-up, bare, snow, wetland) and
//! how tall and dense the trees are.
//!
//! **Data** (both CC BY 4.0, range-read from AWS Open Data, native builds only):
//! * [`worldcover`]: ESA WorldCover 2021 v200, 10 m classes, 3°×3° COGs
//!   (1024² DEFLATE tiles, six overviews) in the `esa-worldcover` bucket.
//! * [`canopy`]: Meta / WRI High Resolution Canopy Height Maps, the 1/4000°
//!   (≈ 28 m) EPSG:4326 aggregate — 95th-percentile height (cm) and canopy
//!   cover (‰) — in 10°×10° one-row-strip LZW TIFFs in `dataforgood-fb-data`.
//!   (The 1 m quadkey rasters are 0.5 GB one-row strips each: no overviews,
//!   too heavy to range-read per tile.)
//!
//! **Flow**: a tile worker ([`crate::scene::build_tile`]) asks [`for_tile`]
//! for its [`TileCover`] — a class grid (and, for detail tiles, a canopy
//! grid) in the tile frame. The needed COG blocks / strip bands are fetched
//! once with HTTP range requests (other workers wanting the same block wait
//! for it), kept on disk under `cache/landcover/` and in a small in-memory
//! LRU. The tile builder paints the classes under the OSM layers (OSM wins
//! where it maps landuse: [`surface`]) and places trees from them ([`trees`]).
//!
//! **Degrades**: offline with nothing cached, ocean (no WorldCover file),
//! missing canopy file, the browser build (neither bucket sends CORS headers),
//! unit tests, or `SKYWARD_NO_LANDCOVER=1` → no cover; the tile builds exactly
//! as before.

pub mod canopy;
pub mod worldcover;

use crate::osm::{Proj, TileId};
use glam::{Vec2, vec2};

/// ESA WorldCover classes (the raster's codes).
pub mod class {
    pub const NONE: u8 = 0;
    pub const TREE: u8 = 10;
    pub const SHRUB: u8 = 20;
    pub const GRASS: u8 = 30;
    pub const CROP: u8 = 40;
    pub const BUILT: u8 = 50;
    pub const BARE: u8 = 60;
    pub const SNOW: u8 = 70;
    pub const WATER: u8 = 80;
    pub const WETLAND: u8 = 90;
    pub const MANGROVE: u8 = 95;
    pub const MOSS: u8 = 100;
}

/// Disabled: tests, the browser (no CORS), or `SKYWARD_NO_LANDCOVER`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| !cfg!(test) && !cfg!(target_arch = "wasm32") && std::env::var_os("SKYWARD_NO_LANDCOVER").is_none());
    *ON
}

/// Attributions (Credits screen; also README, NOTICE, web/credits.html).
pub const WORLDCOVER_CREDIT: &str =
    "© ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium — CC BY 4.0 (desktop version)";
pub const CANOPY_CREDIT: &str =
    "Meta and World Resources Institute (WRI) - 2024. High Resolution Canopy Height Maps (CHM). Source imagery for CHM © 2016 Maxar. — CC BY 4.0 (desktop version)";

// ---------------------------------------------------------------- tile grids

/// A tile's land cover, sampled on a regular grid over its rectangle (tile frame).
#[derive(Clone, Debug)]
pub struct TileCover {
    pub n: usize,
    pub lo: Vec2,
    pub size: Vec2,
    /// WorldCover class per cell (row-major, z then x); [`class::NONE`] = no data.
    pub class: Vec<u8>,
    pub canopy: Option<CanopyGrid>,
}

/// Canopy height (m, 95th percentile) and cover (0..1 as 0..255) per cell.
#[derive(Clone, Debug)]
pub struct CanopyGrid {
    pub n: usize,
    pub height: Vec<u8>,
    pub cover: Vec<u8>,
}

impl TileCover {
    fn cell(&self, n: usize, p: Vec2) -> Option<usize> {
        let t = (p - self.lo) / self.size * n as f32;
        let (x, z) = (t.x.floor() as i64, t.y.floor() as i64);
        (x >= 0 && z >= 0 && (x as usize) < n && (z as usize) < n).then(|| z as usize * n + x as usize)
    }
    pub fn class_at(&self, p: Vec2) -> u8 {
        self.cell(self.n, p).map_or(class::NONE, |i| self.class[i])
    }
    /// (height m, cover 0..1) where the canopy map has data.
    pub fn canopy_at(&self, p: Vec2) -> Option<(f32, f32)> {
        let c = self.canopy.as_ref()?;
        let i = self.cell(c.n, p)?;
        (c.cover[i] != 255 || c.height[i] != 255).then(|| (c.height[i] as f32, c.cover[i] as f32 / 254.0))
    }
    /// True if any cell has data.
    pub fn any(&self) -> bool {
        self.class.iter().any(|&c| c != class::NONE)
    }
}

/// Cells per side of the class grid: detail tiles ≈ 12 m, far tiles ≈ 190 m.
pub const CLASS_N: usize = 128;
/// Cells per side of the canopy grid on detail tiles (≈ 16 m at 51°; the
/// source is ≈ 28 × 17 m).
pub const CANOPY_N: usize = 96;

/// The land cover of a map tile (built in `proj`, its tile frame). Blocks on
/// downloads (tile workers only); `None` when there is none (see the module docs).
pub fn for_tile(id: TileId, proj: &Proj) -> Option<TileCover> {
    if !enabled() {
        return None;
    }
    let t0 = web_time::Instant::now();
    let (lo, hi) = id.rect(proj);
    let size = hi - lo;
    let n = CLASS_N;
    // Pixel size wanted: one cell.
    let cell_m = size.x.min(size.y) / n as f32;
    let mut wc = worldcover::Sampler::new(cell_m);
    let mut class = vec![class::NONE; n * n];
    for z in 0..n {
        for x in 0..n {
            let p = lo + vec2((x as f32 + 0.5) / n as f32, (z as f32 + 0.5) / n as f32) * size;
            let (lat, lon) = proj.to_latlon(p);
            class[z * n + x] = wc.at(lat, lon);
        }
    }
    let t1 = t0.elapsed();
    let canopy = if id.is_far() {
        None
    } else {
        let cn = CANOPY_N;
        let mut s = canopy::Sampler::default();
        let mut g = CanopyGrid { n: cn, height: vec![255; cn * cn], cover: vec![255; cn * cn] };
        let mut any = false;
        for z in 0..cn {
            for x in 0..cn {
                let p = lo + vec2((x as f32 + 0.5) / cn as f32, (z as f32 + 0.5) / cn as f32) * size;
                let (lat, lon) = proj.to_latlon(p);
                if let Some((h, c)) = s.at(lat, lon) {
                    g.height[z * cn + x] = h.round().clamp(0.0, 254.0) as u8;
                    g.cover[z * cn + x] = (c * 254.0).round().clamp(0.0, 254.0) as u8;
                    any = true;
                }
            }
        }
        any.then_some(g)
    };
    if std::env::var_os("SKYWARD_LANDCOVER_DEBUG").is_some() {
        eprintln!("landcover {}/{}/{}: classes {:.1} ms, canopy {:.1} ms", id.z, id.x, id.y, t1.as_secs_f32() * 1e3, (t0.elapsed() - t1).as_secs_f32() * 1e3);
    }
    let tc = TileCover { n, lo, size, class, canopy };
    (tc.any() || tc.canopy.is_some()).then_some(tc)
}

// ---------------------------------------------------------------- blending

/// What OSM says about a spot (rasterised from the tile's areas).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Osm {
    /// No OSM landuse here: the land-cover class decides.
    None,
    /// OSM maps woodland here (its own trees are placed).
    Wood,
    /// OSM maps open ground (farmland, grass, park, scrub, heath, ...).
    Open,
    /// Built-up / paved / water: no land-cover trees, no land-cover ground.
    Hard,
}

/// Ground the land-cover layer paints in a cell (OSM layers draw over it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    /// Leave the base ground (procedural fields / crops).
    Base,
    /// Fields, but pasture only (no crops): grassland.
    Pasture,
    Forest,
    Shrub,
    Built,
    Bare,
    Snow,
    Wetland,
    Moss,
}

/// The ground for a cell: OSM wins where it maps anything; otherwise the
/// WorldCover class picks the surface (cropland keeps the field shader;
/// WorldCover water is left to OSM water and the flattened terrain).
pub fn surface(osm: Osm, wc: u8) -> Surface {
    if osm != Osm::None {
        return Surface::Base;
    }
    match wc {
        class::TREE | class::MANGROVE => Surface::Forest,
        class::SHRUB => Surface::Shrub,
        class::GRASS => Surface::Pasture,
        class::BUILT => Surface::Built,
        class::BARE => Surface::Bare,
        class::SNOW => Surface::Snow,
        class::WETLAND => Surface::Wetland,
        class::MOSS => Surface::Moss,
        _ => Surface::Base,
    }
}

/// Tree stand at a spot: density (trees / m²), conifer fraction and a height
/// range (m). `None`: no land-cover trees here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stand {
    pub density: f32,
    pub conifer: f32,
    pub h: (f32, f32),
}

/// Woodland density (trees / m²) at full canopy cover, as the OSM woods.
pub const FOREST_DENSITY: f32 = 1.0 / 200.0;

/// Trees the land cover adds at a spot. OSM woods place their own trees (the
/// canopy only shapes them: [`canopy_tree`]); built-up / paved / water get
/// none. Elsewhere the canopy map — where it has data — is the evidence for
/// trees (forest, hedgerows, lone trees in fields); without it the WorldCover
/// tree / shrub classes stand in. `cold`: high latitude or altitude (more conifers).
pub fn stand(osm: Osm, wc: u8, canopy: Option<(f32, f32)>, cold: bool) -> Option<Stand> {
    if matches!(osm, Osm::Wood | Osm::Hard) || matches!(wc, class::BUILT | class::WATER | class::SNOW) {
        return None;
    }
    let conifer = if cold { 0.75 } else { 0.25 };
    match canopy {
        Some((h, cover)) => {
            // Under ~3 m is shrub / crop / noise: no trees.
            if h < 3.0 || cover < 0.06 {
                return None;
            }
            let shrubby = h < 6.0;
            // Closed canopy reads ~80–100 % cover.
            Some(Stand { density: FOREST_DENSITY * (cover / 0.8).min(1.0), conifer: if shrubby { 0.1 } else { conifer }, h: canopy_heights(h) })
        }
        None => match wc {
            class::TREE => Some(Stand { density: FOREST_DENSITY * if osm == Osm::Open { 0.0 } else { 0.85 }, conifer, h: (9.0, 19.0) }).filter(|s| s.density > 0.0),
            class::MANGROVE => Some(Stand { density: FOREST_DENSITY * 0.8, conifer: 0.0, h: (4.0, 10.0) }),
            class::SHRUB if osm == Osm::None => Some(Stand { density: 1.0 / 900.0, conifer: 0.1, h: (2.5, 6.0) }),
            _ => None,
        },
    }
}

/// Instance heights from a canopy top height (the 95th percentile is the
/// tallest crowns; the stand spreads below it).
pub fn canopy_heights(p95: f32) -> (f32, f32) {
    let top = p95.clamp(2.5, 60.0);
    (top * 0.62, top)
}

/// An OSM-wood tree at a spot with canopy data: `None` = a clearing (drop
/// it), else its height from the canopy. `u`, `v`: uniform randoms.
pub fn canopy_tree(canopy: (f32, f32), u: f32, v: f32) -> Option<f32> {
    let (h, cover) = canopy;
    // Full woodland from ~35 % cover up; thinner cover thins the wood.
    if h < 2.0 || u > (cover / 0.35).min(1.0) {
        return None;
    }
    let (lo, hi) = canopy_heights(h);
    Some(lo + (hi - lo) * v)
}

// ---------------------------------------------------------------- downloads

pub(crate) use crate::cog::Store;
pub use crate::cog::tiff;

/// Range reads + disk cache under `cache/landcover/` (native).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) mod net {
    pub use crate::cog::net::{range, read, write};
    use std::path::PathBuf;

    /// Disk budget for everything under `cache/landcover/` (oldest pruned at startup).
    const CACHE_LIMIT: u64 = 800_000_000;

    pub fn root() -> PathBuf {
        static PRUNE: std::sync::Once = std::sync::Once::new();
        PRUNE.call_once(|| {
            std::thread::spawn(|| crate::cog::net::prune(PathBuf::from("cache").join("landcover"), CACHE_LIMIT));
        });
        PathBuf::from("cache").join("landcover")
    }
}

/// Logged with the first failed land-cover load.
pub(crate) const FAIL: &str = "landcover (ground falls back to OSM + procedural fields)";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osm_wins_over_land_cover() {
        // Anything OSM maps keeps its own look.
        for osm in [Osm::Wood, Osm::Open, Osm::Hard] {
            for wc in [class::TREE, class::BUILT, class::GRASS, class::SNOW] {
                assert_eq!(surface(osm, wc), Surface::Base, "{osm:?} {wc}");
            }
        }
        // The gaps take the land-cover class.
        assert_eq!(surface(Osm::None, class::TREE), Surface::Forest);
        assert_eq!(surface(Osm::None, class::MANGROVE), Surface::Forest);
        assert_eq!(surface(Osm::None, class::GRASS), Surface::Pasture);
        assert_eq!(surface(Osm::None, class::CROP), Surface::Base);
        assert_eq!(surface(Osm::None, class::BUILT), Surface::Built);
        assert_eq!(surface(Osm::None, class::SNOW), Surface::Snow);
        assert_eq!(surface(Osm::None, class::WATER), Surface::Base);
        assert_eq!(surface(Osm::None, class::NONE), Surface::Base);
    }

    #[test]
    fn trees_follow_canopy_then_class() {
        // OSM woods place their own; hard ground and built-up get none.
        assert!(stand(Osm::Wood, class::TREE, Some((20.0, 1.0)), false).is_none());
        assert!(stand(Osm::Hard, class::TREE, Some((20.0, 1.0)), false).is_none());
        assert!(stand(Osm::None, class::BUILT, Some((20.0, 1.0)), false).is_none());
        // Canopy data: a hedgerow in OSM farmland gets trees, sized by the canopy.
        let s = stand(Osm::Open, class::CROP, Some((12.0, 0.3)), false).unwrap();
        assert!((s.density - FOREST_DENSITY * 0.3 / 0.8).abs() < 1e-6);
        assert_eq!(s.h, canopy_heights(12.0));
        // Canopy says no trees, even in the tree class.
        assert!(stand(Osm::None, class::TREE, Some((1.0, 0.0)), false).is_none());
        // No canopy data: the class decides; OSM open ground stays open.
        assert!(stand(Osm::None, class::TREE, None, false).is_some());
        assert!(stand(Osm::Open, class::TREE, None, false).is_none());
        assert!(stand(Osm::None, class::CROP, None, false).is_none());
        // Colder ground: more conifers.
        assert!(stand(Osm::None, class::TREE, None, true).unwrap().conifer > stand(Osm::None, class::TREE, None, false).unwrap().conifer);
    }

    #[test]
    fn canopy_height_sets_tree_scale() {
        let (lo, hi) = canopy_heights(25.0);
        assert_eq!(hi, 25.0);
        assert!(lo > 12.0 && lo < 20.0);
        // Taller canopy → taller trees; clamped to sane sizes.
        assert!(canopy_heights(30.0).1 > canopy_heights(10.0).1);
        assert_eq!(canopy_heights(200.0).1, 60.0);
        // In an OSM wood: dense canopy keeps the tree at the canopy's height ...
        let h = canopy_tree((20.0, 1.0), 0.5, 1.0).unwrap();
        assert!((h - 20.0).abs() < 1e-4);
        // ... a clearing (no canopy) drops it; thin cover drops some, a
        // moderately open wood keeps them all.
        assert!(canopy_tree((0.5, 0.0), 0.01, 0.5).is_none());
        assert!(canopy_tree((15.0, 0.1), 0.5, 0.5).is_none());
        assert!(canopy_tree((15.0, 0.1), 0.2, 0.5).is_some());
        assert!(canopy_tree((15.0, 0.5), 0.99, 0.5).is_some());
    }

    #[test]
    fn tile_cover_lookup() {
        let mut class = vec![class::CROP; 4];
        class[3] = class::TREE;
        let c = TileCover { n: 2, lo: vec2(100.0, 200.0), size: vec2(10.0, 10.0), class, canopy: Some(CanopyGrid { n: 1, height: vec![18], cover: vec![127] }) };
        assert_eq!(c.class_at(vec2(101.0, 201.0)), class::CROP);
        assert_eq!(c.class_at(vec2(108.0, 209.0)), class::TREE);
        assert_eq!(c.class_at(vec2(99.0, 201.0)), class::NONE);
        let (h, cov) = c.canopy_at(vec2(105.0, 205.0)).unwrap();
        assert_eq!(h, 18.0);
        assert!((cov - 0.5).abs() < 0.01);
        assert!(c.canopy_at(vec2(150.0, 205.0)).is_none());
    }

    /// Live data (network; `cargo test --release -- --ignored live_`): Leith
    /// Hill's woods (Surrey) are WorldCover tree cover under a tall canopy;
    /// open sea has no WorldCover file.
    #[test]
    #[ignore]
    #[cfg(not(target_arch = "wasm32"))]
    fn live_samples_real_data() {
        let mut wc = worldcover::Sampler::new(12.0);
        assert_eq!(wc.at(51.1785, -0.3640), class::TREE);
        let mut far = worldcover::Sampler::new(190.0);
        assert_ne!(far.at(51.18, -0.37), class::NONE);
        assert_eq!(wc.at(50.0, -20.0), class::NONE);
        let mut cs = canopy::Sampler::default();
        let (h, c) = cs.at(51.1785, -0.3640).unwrap();
        assert!(h > 10.0 && h < 45.0 && c > 0.5, "{h} {c}");
    }
}
