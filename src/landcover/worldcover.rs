//! ESA WorldCover 2021 v200 (10 m land cover, CC BY 4.0).
//!
//! `s3://esa-worldcover/v200/2021/map/ESA_WorldCover_10m_2021_v200_{tile}_Map.tif`
//! (eu-central-1; no CORS headers): 3°×3° tiles named after their south-west
//! corner (`N51W003` covers 51–54° N, 3° W–0°), 36000² px EPSG:4326 COGs with
//! 1024² DEFLATE tiles, 8-bit class codes, overviews at 20 / 40 / 80 / 160 /
//! 320 / 640 m. Ocean has no file. We read the header once (cached) and then
//! single 1024² blocks of the overview that suits the tile's cell size.

use super::class;
#[cfg(not(target_arch = "wasm32"))]
use super::tiff;
use super::tiff::Ifd;
use std::sync::{Arc, LazyLock};

const BASE: &str = "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map";

/// Tile name for the 3° tile holding a point, and its south-west corner.
pub fn tile_name(lat: f64, lon: f64) -> (String, i32, i32) {
    let la = ((lat / 3.0).floor() * 3.0) as i32;
    let lo = ((lon / 3.0).floor() * 3.0) as i32;
    let ns = if la < 0 { 'S' } else { 'N' };
    let ew = if lo < 0 { 'W' } else { 'E' };
    (format!("{ns}{:02}{ew}{:03}", la.abs(), lo.abs()), la, lo)
}

fn url(name: &str) -> String {
    format!("{BASE}/ESA_WorldCover_10m_2021_v200_{name}_Map.tif")
}

/// A file's parsed header.
pub struct Header {
    pub ifds: Vec<Ifd>,
    pub big_endian: bool,
}

/// The overview to read for cells of `cell_m` metres: the coarsest whose
/// pixels are no bigger than a cell (level 0 = 10 m, level k = 10·2^k m).
pub fn level_for(cell_m: f32, levels: usize) -> usize {
    let mut l = 0;
    while l + 1 < levels && 10.0 * (1u32 << (l + 1)) as f32 <= cell_m {
        l += 1;
    }
    l
}

/// Pixel (column, row) of a point in a level of `levels` overviews of a tile
/// with south-west corner (`la`, `lo`): the full image spans 3° in `w` px.
pub fn pixel(lat: f64, lon: f64, la: i32, lo: i32, w: u32, h: u32) -> Option<(u32, u32)> {
    let fx = (lon - lo as f64) / 3.0 * w as f64;
    let fy = (la as f64 + 3.0 - lat) / 3.0 * h as f64;
    (fx >= 0.0 && fy >= 0.0 && fx < w as f64 && fy < h as f64).then_some((fx as u32, fy as u32))
}

static HEADERS: LazyLock<super::Store<String, Header>> = LazyLock::new(|| super::Store::new(64, super::FAIL));
/// Decoded 1024² blocks (1 MB each).
static BLOCKS: LazyLock<super::Store<(String, u8, u32), Vec<u8>>> = LazyLock::new(|| super::Store::new(48, super::FAIL));

#[cfg(not(target_arch = "wasm32"))]
fn load_header(name: &str) -> Result<Option<Header>, String> {
    use super::net;
    let dir = net::root().join("worldcover");
    let path = dir.join(format!("{name}.hdr"));
    let none = dir.join(format!("{name}.none"));
    if none.exists() {
        return Ok(None);
    }
    let mut bytes = net::read(&path).unwrap_or_default();
    let mut fetched = false;
    loop {
        match tiff::parse(&bytes) {
            Ok(tiff::Parse::Ok { big_endian, ifds }) => {
                if fetched {
                    net::write(&path, &bytes);
                }
                return Ok(Some(Header { ifds, big_endian }));
            }
            Ok(tiff::Parse::Need(n)) => {
                // Headers are ~25 kB; never more than a few MB.
                if n > 8 << 20 {
                    return Err(format!("WorldCover {name}: header too large"));
                }
                let want = n.max(64 << 10).max(bytes.len() as u64 * 2);
                match net::range(&url(name), 0, want)? {
                    Some(b) if b.len() as u64 > bytes.len() as u64 => bytes = b,
                    Some(_) => return Err(format!("WorldCover {name}: truncated header")),
                    None => {
                        // No file: ocean (or outside the product).
                        net::write(&none, b"");
                        return Ok(None);
                    }
                }
                fetched = true;
            }
            Err(e) => return Err(format!("WorldCover {name}: {e}")),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn load_block(name: &str, hdr: &Header, level: usize, idx: u32) -> Result<Option<Vec<u8>>, String> {
    use super::net;
    let d = &hdr.ifds[level];
    let (off, len) = (d.offsets[idx as usize], d.counts[idx as usize]);
    if len == 0 {
        // Sparse block: no data.
        return Ok(Some(vec![class::NONE; d.block_samples()]));
    }
    if len > 16 << 20 {
        return Err(format!("WorldCover {name}: block of {len} bytes"));
    }
    let path = net::root().join("worldcover").join(name).join(format!("{level}_{idx}.bin"));
    let cached = net::read(&path).filter(|b| b.len() as u64 == len);
    let raw = match cached {
        Some(b) => b,
        None => {
            let b = net::range(&url(name), off, len)?.ok_or_else(|| format!("WorldCover {name}: file vanished"))?;
            if b.len() as u64 != len {
                return Err(format!("WorldCover {name}: short block"));
            }
            net::write(&path, &b);
            b
        }
    };
    let v = tiff::decode_block(d, hdr.big_endian, &raw, None)?;
    Ok(Some(v.into_iter().map(|s| s as u8).collect()))
}

/// Class lookups for one tile build (keeps the blocks it touched).
pub struct Sampler {
    cell_m: f32,
    /// Files by south-west corner.
    files: Vec<((i32, i32), Option<Arc<Header>>)>,
    /// Blocks by (corner, level, index); the last one hit is checked first.
    blocks: Vec<((i32, i32, u8, u32), Option<Arc<Vec<u8>>>)>,
    last: usize,
}

impl Sampler {
    pub fn new(cell_m: f32) -> Self {
        Sampler { cell_m, files: Vec::new(), blocks: Vec::new(), last: 0 }
    }

    /// WorldCover class at a point ([`class::NONE`] where there is none).
    pub fn at(&mut self, lat: f64, lon: f64) -> u8 {
        #[cfg(target_arch = "wasm32")]
        {
            let _ = (lat, lon, self.cell_m, &self.files, &self.blocks, self.last);
            class::NONE
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (la, lo) = (((lat / 3.0).floor() * 3.0) as i32, ((lon / 3.0).floor() * 3.0) as i32);
            let hdr = match self.files.iter().find(|f| f.0 == (la, lo)) {
                Some(f) => f.1.clone(),
                None => {
                    let name = tile_name(lat, lon).0;
                    let h = HEADERS.get(&name, || load_header(&name));
                    self.files.push(((la, lo), h.clone()));
                    h
                }
            };
            let Some(hdr) = hdr else { return class::NONE };
            let level = level_for(self.cell_m, hdr.ifds.len());
            let d = &hdr.ifds[level];
            let Some((px, py)) = pixel(lat, lon, la, lo, d.width, d.height) else { return class::NONE };
            let idx = (py / d.block_h) * d.blocks_across() + px / d.block_w;
            let key = (la, lo, level as u8, idx);
            let k = match self.blocks.get(self.last).filter(|b| b.0 == key) {
                Some(_) => self.last,
                None => match self.blocks.iter().position(|b| b.0 == key) {
                    Some(i) => i,
                    None => {
                        let name = tile_name(lat, lon).0;
                        let b = BLOCKS.get(&(name.clone(), level as u8, idx), || load_block(&name, &hdr, level, idx));
                        self.blocks.push((key, b));
                        self.blocks.len() - 1
                    }
                },
            };
            self.last = k;
            let Some(blk) = &self.blocks[k].1 else { return class::NONE };
            blk.get(((py % d.block_h) * d.block_w + px % d.block_w) as usize).copied().unwrap_or(class::NONE)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_names_follow_the_south_west_corner() {
        assert_eq!(tile_name(51.2, -0.2), ("N51W003".to_string(), 51, -3));
        assert_eq!(tile_name(47.26, 11.39), ("N45E009".to_string(), 45, 9));
        assert_eq!(tile_name(-1.0, -60.0), ("S03W060".to_string(), -3, -60));
        assert_eq!(tile_name(0.5, 0.5), ("N00E000".to_string(), 0, 0));
    }

    #[test]
    fn pixels_and_levels() {
        // 36000 px over 3°: 51.5° N, 1.5° W is the centre of N51W003.
        assert_eq!(pixel(52.5, -1.5, 51, -3, 36000, 36000), Some((18000, 18000)));
        assert_eq!(pixel(54.0 - 1e-9, -3.0, 51, -3, 36000, 36000), Some((0, 0)));
        assert_eq!(pixel(50.9, -1.5, 51, -3, 36000, 36000), None);
        assert_eq!(level_for(12.0, 7), 0);
        assert_eq!(level_for(25.0, 7), 1);
        assert_eq!(level_for(190.0, 7), 4);
        assert_eq!(level_for(1e6, 7), 6);
    }
}
