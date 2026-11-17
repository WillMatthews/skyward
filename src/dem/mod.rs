//! Elevation sources: the Mapterhorn base, and detail sources layered over
//! AWS Terrain Tiles where Mapterhorn is not used.
//!
//! **Priority** (finest real data first, one source per DEM tile):
//! 1. [`mapterhorn`] — every level, both builds: national DTMs (0.25–10 m,
//!    mostly LIDAR; England's is the same EA composite [`lidar`] reads) where
//!    they exist, GLO-30 elsewhere. A tile it serves is used as is: the
//!    sources below would only re-add the same EA data, or put GLO-30 back
//!    over a LIDAR slope. Mapterhorn blends its own source borders, so the
//!    England / Scotland or Swiss / Italian borders have no seams.
//! 2. Otherwise (`SKYWARD_NO_MAPTERHORN=1`, Japan, no tile, offline / server
//!    trouble): Terrain Tiles, refined by the [`Source`]s below exactly as
//!    before Mapterhorn. A fallback tile beside a Mapterhorn one can differ by
//!    a few metres (both are bilinearly sampled across the tile edge, so no
//!    cliff); Japan's edge is at sea.
//!
//! [`crate::terrain`] otherwise reads every level of detail from Terrain Tiles
//! (terrarium PNGs). On native builds, each *detail* DEM tile (zoom
//! [`crate::terrain::DEM_NEAR`], 256² web-Mercator pixels) is then handed to
//! the [`Source`]s here, in order, to refine in place before anything samples
//! it: grids, runway profiles, skirts and `cached_height` all see the refined
//! heights, and the far / horizon levels stay on Terrain Tiles.
//!
//! Sources ([`SOURCES`]):
//! * [`copdem`]: Copernicus DEM GLO-30 (30 m TanDEM-X surface model), blended
//!   in on rough terrain and north of 60° N.
//! * [`lidar`]: the Environment Agency's LIDAR composite DTM (England, bare
//!   earth), over everything else where it has data, ramped in at its edges.
//!
//! A per-area source is one more [`Source`] appended after the global ones:
//! it overwrites the pixels it covers and leaves the rest alone.
//!
//! **Off**: the browser build (the Copernicus bucket sends no CORS headers;
//! the browser has no detail-source hook), unit tests, a flat world, or
//! `SKYWARD_NO_COPDEM=1` / `SKYWARD_NO_LIDAR=1` (that source only).

pub mod copdem;
pub mod lidar;
pub mod mapterhorn;
pub mod osgb;

/// A detail DEM tile being refined.
pub struct DetailTile<'a> {
    pub z: u32,
    pub x: u32,
    pub y: u32,
    /// Terrain Tiles height (m) at a global pixel (column, row) of zoom `z`,
    /// this tile's or a neighbour's; `None` where none is available.
    pub base: &'a dyn Fn(i64, i64) -> Option<f32>,
}

impl DetailTile<'_> {
    /// (lat, lon) of the centre of local pixel (`i`, `j`) (outside 0..256 = a
    /// neighbour's pixel).
    pub fn latlon(&self, i: i64, j: i64) -> (f64, f64) {
        pixel_latlon(self.z, self.x as i64 * 256 + i, self.y as i64 * 256 + j)
    }
    /// Terrain Tiles height at local pixel (`i`, `j`).
    pub fn base_at(&self, i: i64, j: i64) -> Option<f32> {
        (self.base)(self.x as i64 * 256 + i, self.y as i64 * 256 + j)
    }
}

/// (lat, lon) of the centre of global web-Mercator pixel (`px`, `py`) at zoom `z`.
pub fn pixel_latlon(z: u32, px: i64, py: i64) -> (f64, f64) {
    let n = (256u64 << z) as f64;
    let lon = (px as f64 + 0.5) / n * 360.0 - 180.0;
    let y = std::f64::consts::PI * (1.0 - 2.0 * (py as f64 + 0.5) / n);
    (y.sinh().atan().to_degrees(), lon)
}

/// One elevation source.
pub trait Source: Send + Sync {
    fn name(&self) -> &'static str;
    /// Refine the tile's heights `h` (256², row-major, already holding the
    /// Terrain Tiles heights and any earlier source's refinement) where this
    /// source has something better. Errors are logged; `h` is used as it is.
    fn refine(&self, tile: &DetailTile, h: &mut [f32]) -> Result<(), String>;
}

/// The sources, in order (later ones win where they have data).
static SOURCES: std::sync::LazyLock<Vec<Box<dyn Source>>> = std::sync::LazyLock::new(|| {
    let mut v: Vec<Box<dyn Source>> = Vec::new();
    if copdem::enabled() {
        v.push(Box::new(copdem::CopDem));
    }
    if lidar::enabled() {
        v.push(Box::new(lidar::Lidar));
    }
    v
});

/// True if any detail source is on (the terrain loader then refines its
/// detail tiles).
pub fn active() -> bool {
    !SOURCES.is_empty()
}

/// Run the sources over a detail tile.
pub fn refine(tile: &DetailTile, h: &mut [f32]) {
    for s in SOURCES.iter() {
        if let Err(e) = s.refine(tile, h) {
            static WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                eprintln!("terrain: {}: {e} (Terrain Tiles heights where missing)", s.name());
            }
        }
    }
}

/// Min (`max` = false) or max filter of radius `r` over a `w`×`w` grid
/// (NaN ignored; the window is clamped at the edges).
pub fn window(a: &[f32], w: usize, r: usize, max: bool) -> Vec<f32> {
    let pick = |x: f32, y: f32| {
        if x.is_nan() {
            y
        } else if y.is_nan() {
            x
        } else if max {
            x.max(y)
        } else {
            x.min(y)
        }
    };
    // Separable: rows, then columns.
    let mut t = vec![f32::NAN; w * w];
    for j in 0..w {
        for i in 0..w {
            let mut v = f32::NAN;
            for k in i.saturating_sub(r)..=(i + r).min(w - 1) {
                v = pick(v, a[j * w + k]);
            }
            t[j * w + i] = v;
        }
    }
    let mut out = vec![f32::NAN; w * w];
    for j in 0..w {
        for i in 0..w {
            let mut v = f32::NAN;
            for k in j.saturating_sub(r)..=(j + r).min(w - 1) {
                v = pick(v, t[k * w + i]);
            }
            out[j * w + i] = v;
        }
    }
    out
}

/// Box mean of radius `r` over a `w`×`w` grid (edges clamped).
pub fn box_mean(a: &[f32], w: usize, r: usize) -> Vec<f32> {
    let mut t = vec![0f32; w * w];
    for j in 0..w {
        for i in 0..w {
            let (lo, hi) = (i.saturating_sub(r), (i + r).min(w - 1));
            t[j * w + i] = (lo..=hi).map(|k| a[j * w + k]).sum::<f32>() / (hi - lo + 1) as f32;
        }
    }
    let mut out = vec![0f32; w * w];
    for j in 0..w {
        for i in 0..w {
            let (lo, hi) = (j.saturating_sub(r), (j + r).min(w - 1));
            out[j * w + i] = (lo..=hi).map(|k| t[k * w + i]).sum::<f32>() / (hi - lo + 1) as f32;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixel_centres_invert_the_tile_fraction() {
        let (lat, lon) = (47.2603, 11.3439);
        let (fx, fy) = crate::osm::TileId::frac(lat, lon, 12);
        let (px, py) = ((fx * 256.0) as i64, (fy * 256.0) as i64);
        let (la, lo) = pixel_latlon(12, px, py);
        // Within half a pixel (≈ 13 × 19 m here).
        assert!((la - lat).abs() < 1.5e-4 && (lo - lon).abs() < 2.5e-4, "{la} {lo}");
        let (fx2, fy2) = crate::osm::TileId::frac(la, lo, 12);
        assert!((fx2 * 256.0 - (px as f64 + 0.5)).abs() < 1e-6 && (fy2 * 256.0 - (py as f64 + 0.5)).abs() < 1e-6);
    }

    #[test]
    fn filters() {
        let a = [1.0, 5.0, 2.0, f32::NAN, 0.0, 3.0, 4.0, 1.0, 9.0];
        let mn = window(&a, 3, 1, false);
        let mx = window(&a, 3, 1, true);
        assert_eq!(mn[4], 0.0);
        assert_eq!(mx[4], 9.0);
        assert_eq!(mx[0], 5.0);
        assert_eq!(mn[8], 0.0);
        let m = box_mean(&[0.0, 3.0, 0.0, 3.0], 2, 1);
        assert!(m.iter().all(|&v| (v - 1.5).abs() < 1e-6));
    }
}
