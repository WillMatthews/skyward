//! Meta / WRI High Resolution Canopy Height Maps (CC BY 4.0), the 1/4000°
//! EPSG:4326 aggregate.
//!
//! `s3://dataforgood-fb-data/forests/v1/alsgedi_global_v6_float_epsg4326_v3_10deg/meta_chm_lat={top}_lon={left}_{param}.tif`
//! (us-east-1; no CORS headers): 10°×10° files named after their north-west
//! corner, 40000² px of uint16 (65535 = no data), one LZW strip per row.
//! We read `p95` (95th-percentile canopy height, cm) and `cover` (fraction of
//! the cell with canopy over 1 m, ‰). Rows are fetched in bands of
//! [`BAND`] (one range request per band and parameter, ≈ 1.5–2 MB in a
//! wooded temperate band), cached on disk as fetched, and decoded only up to
//! the columns a tile needs.

#[cfg(not(target_arch = "wasm32"))]
use super::tiff;
use super::tiff::Ifd;
use std::sync::{Arc, LazyLock};

const BASE: &str = "https://dataforgood-fb-data.s3.amazonaws.com/forests/v1/alsgedi_global_v6_float_epsg4326_v3_10deg";
/// Rows per fetched band (≈ 7 km of latitude).
pub const BAND: u32 = 256;
/// Pixels per degree.
pub const PPD: f64 = 4000.0;
const NODATA: u16 = 65535;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Param {
    P95,
    Cover,
}

impl Param {
    fn suffix(self) -> &'static str {
        match self {
            Param::P95 => "p95",
            Param::Cover => "cover",
        }
    }
}

/// The 10° file holding a point: (name stem, top latitude, left longitude).
/// The product writes the top edge `0` of the band just south of the equator as "-0.0".
pub fn file_name(lat: f64, lon: f64) -> (String, i32, i32) {
    let top = ((lat / 10.0).ceil() * 10.0) as i32;
    let left = ((lon / 10.0).floor() * 10.0) as i32;
    let t = if top == 0 { "-0.0".to_string() } else { format!("{top}.0") };
    (format!("meta_chm_lat={t}_lon={left}.0"), top, left)
}

fn url(stem: &str, p: Param) -> String {
    format!("{BASE}/{stem}_{}.tif", p.suffix())
}

/// Pixel (column, row) of a point in a file with north-west corner (top, left).
pub fn pixel(lat: f64, lon: f64, top: i32, left: i32) -> Option<(u32, u32)> {
    let fx = (lon - left as f64) * PPD;
    let fy = (top as f64 - lat) * PPD;
    let n = 10.0 * PPD;
    (fx >= 0.0 && fy >= 0.0 && fx < n && fy < n).then_some((fx as u32, fy as u32))
}

pub struct Header {
    pub ifd: Ifd,
    pub big_endian: bool,
}

/// One band of compressed rows: `rows[k]` = byte range of row `band·BAND + k` in `data`.
pub struct Band {
    data: Vec<u8>,
    rows: Vec<(usize, usize)>,
}

/// Pack a band for the disk cache: u32 row count, (u32 len) per row, then the rows.
pub fn pack(rows: &[&[u8]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(4 + rows.len() * 4 + rows.iter().map(|r| r.len()).sum::<usize>());
    out.extend((rows.len() as u32).to_le_bytes());
    for r in rows {
        out.extend((r.len() as u32).to_le_bytes());
    }
    for r in rows {
        out.extend_from_slice(r);
    }
    out
}

pub fn unpack(b: Vec<u8>) -> Option<Band> {
    let n = u32::from_le_bytes(b.get(..4)?.try_into().ok()?) as usize;
    if n > 65536 {
        return None;
    }
    let mut pos = 4 + n * 4;
    let mut rows = Vec::with_capacity(n);
    for k in 0..n {
        let l = u32::from_le_bytes(b.get(4 + k * 4..8 + k * 4)?.try_into().ok()?) as usize;
        let end = pos.checked_add(l)?;
        if end > b.len() {
            return None;
        }
        rows.push((pos, end));
        pos = end;
    }
    (pos == b.len()).then_some(Band { data: b, rows })
}

static HEADERS: LazyLock<super::Store<(String, Param), Header>> = LazyLock::new(|| super::Store::new(16, super::FAIL));
/// Compressed bands (≈ 1–3 MB each).
static BANDS: LazyLock<super::Store<(String, Param, u32), Band>> = LazyLock::new(|| super::Store::new(12, super::FAIL));

#[cfg(not(target_arch = "wasm32"))]
fn load_header(stem: &str, p: Param) -> Result<Option<Header>, String> {
    use super::net;
    let dir = net::root().join("canopy");
    let path = dir.join(format!("{stem}_{}.hdr", p.suffix()));
    let none = dir.join(format!("{stem}_{}.none", p.suffix()));
    if none.exists() {
        return Ok(None);
    }
    let mut bytes = net::read(&path).unwrap_or_default();
    let mut fetched = false;
    loop {
        match tiff::parse(&bytes) {
            Ok(tiff::Parse::Ok { big_endian, mut ifds }) => {
                if fetched {
                    net::write(&path, &bytes);
                }
                let ifd = ifds.swap_remove(0);
                if ifd.width != 40000 || ifd.height != 40000 || ifd.bits != 16 || ifd.block_h != 1 {
                    return Err(format!("canopy {stem}: unexpected layout {}x{} {}-bit {} rows/strip", ifd.width, ifd.height, ifd.bits, ifd.block_h));
                }
                return Ok(Some(Header { ifd, big_endian }));
            }
            Ok(tiff::Parse::Need(n)) => {
                // ≈ 330 kB (two 40000-entry tables).
                if n > 4 << 20 {
                    return Err(format!("canopy {stem}: header too large"));
                }
                let want = n.max(340_000).max(bytes.len() as u64 * 2);
                match net::range(&url(stem, p), 0, want)? {
                    Some(b) if b.len() > bytes.len() => bytes = b,
                    Some(_) => return Err(format!("canopy {stem}: truncated header")),
                    None => {
                        // No file: open ocean / outside the product.
                        net::write(&none, b"");
                        return Ok(None);
                    }
                }
                fetched = true;
            }
            Err(e) => return Err(format!("canopy {stem}: {e}")),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn load_band(stem: &str, p: Param, hdr: &Header, band: u32) -> Result<Option<Band>, String> {
    use super::net;
    let path = net::root().join("canopy").join(format!("{stem}_{}", p.suffix())).join(format!("{band}.bin"));
    if let Some(b) = net::read(&path).and_then(unpack) {
        return Ok(Some(b));
    }
    let d = &hdr.ifd;
    let r0 = (band * BAND) as usize;
    let r1 = ((band + 1) * BAND).min(d.height) as usize;
    if r0 >= r1 {
        return Ok(None);
    }
    let (offs, cnts) = (&d.offsets[r0..r1], &d.counts[r0..r1]);
    let lo = *offs.iter().min().unwrap();
    let hi = offs.iter().zip(cnts).map(|(o, c)| o + c).max().unwrap();
    let sum: u64 = cnts.iter().sum();
    if sum > 64 << 20 {
        return Err(format!("canopy {stem}: band of {sum} bytes"));
    }
    // Rows are stored in order: one request for the band (else one per row).
    let rows: Vec<Vec<u8>> = if hi - lo <= sum + sum / 2 + (64 << 10) {
        let b = net::range(&url(stem, p), lo, hi - lo)?.ok_or_else(|| format!("canopy {stem}: file vanished"))?;
        if (b.len() as u64) < hi - lo {
            return Err(format!("canopy {stem}: short band"));
        }
        offs.iter().zip(cnts).map(|(o, c)| b[(o - lo) as usize..(o - lo + c) as usize].to_vec()).collect()
    } else {
        let mut v = Vec::with_capacity(r1 - r0);
        for (o, c) in offs.iter().zip(cnts) {
            v.push(net::range(&url(stem, p), *o, *c)?.ok_or_else(|| format!("canopy {stem}: file vanished"))?);
        }
        v
    };
    let packed = pack(&rows.iter().map(|r| r.as_slice()).collect::<Vec<_>>());
    net::write(&path, &packed);
    Ok(unpack(packed))
}

/// Columns per decoded chunk of a row (≈ 1° of longitude).
pub const CHUNK: u32 = 4096;

/// Quantise a sample to a byte: height cm → m, cover ‰ → ‰ / 4; 255 = no data.
pub fn quantise(p: Param, v: u16) -> u8 {
    if v == NODATA {
        return 255;
    }
    match p {
        Param::P95 => ((v as u32 + 50) / 100).min(254) as u8,
        Param::Cover => ((v as u32 + 2) / 4).min(250) as u8,
    }
}

/// Decoded row chunks, quantised (4 kB each): rows are LZW strips that only
/// decode from their start, so every chunk a tile reads is kept for its
/// neighbours along the same rows.
static CHUNKS: LazyLock<super::Store<(String, Param, u32, u32), Vec<u8>>> = LazyLock::new(|| super::Store::new(3000, super::FAIL));

/// Canopy lookups for one tile build (keeps the row chunks it touched).
#[derive(Default)]
pub struct Sampler {
    /// Chunks by (file corner, parameter, row, chunk); the last hit per parameter is checked first.
    chunks: Vec<((i32, i32, Param, u32, u32), Option<Arc<Vec<u8>>>)>,
    headers: Vec<((i32, i32, Param), Option<Arc<Header>>)>,
    last: [usize; 2],
}

impl Sampler {
    /// (95th-percentile height m, cover 0..1) at a point; `None` without data.
    pub fn at(&mut self, lat: f64, lon: f64) -> Option<(f32, f32)> {
        let h = self.value(Param::P95, lat, lon)?;
        let c = self.value(Param::Cover, lat, lon)?;
        Some((h as f32, (c as f32 * 4.0 / 1000.0).min(1.0)))
    }

    /// Quantised sample ([`quantise`]); `None` without data.
    fn value(&mut self, p: Param, lat: f64, lon: f64) -> Option<u8> {
        #[cfg(target_arch = "wasm32")]
        {
            let _ = (p, lat, lon, &self.chunks, &self.headers, self.last);
            None
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let top = ((lat / 10.0).ceil() * 10.0) as i32;
            let left = ((lon / 10.0).floor() * 10.0) as i32;
            let (col, row) = pixel(lat, lon, top, left)?;
            let chunk = col / CHUNK;
            let key = (top, left, p, row, chunk);
            let li = p as usize;
            let k = if self.chunks.get(self.last[li]).is_some_and(|c| c.0 == key) {
                self.last[li]
            } else if let Some(i) = self.chunks.iter().position(|c| c.0 == key) {
                i
            } else {
                let hk = (top, left, p);
                let hdr = match self.headers.iter().find(|h| h.0 == hk) {
                    Some(h) => h.1.clone(),
                    None => {
                        let stem = file_name(lat, lon).0;
                        let h = HEADERS.get(&(stem.clone(), p), || load_header(&stem, p));
                        self.headers.push((hk, h.clone()));
                        h
                    }
                };
                let hdr = hdr?;
                let stem = file_name(lat, lon).0;
                let c = CHUNKS.get(&(stem.clone(), p, row, chunk), || {
                    let band = row / BAND;
                    let Some(b) = BANDS.get(&(stem.clone(), p, band), || load_band(&stem, p, &hdr, band)) else { return Ok(None) };
                    let Some(&(s, e)) = b.rows.get((row % BAND) as usize) else { return Ok(None) };
                    let end = ((chunk + 1) * CHUNK).min(hdr.ifd.width);
                    let v = tiff::decode_block(&hdr.ifd, hdr.big_endian, &b.data[s..e], Some(end as usize))?;
                    Ok(Some(v[(chunk * CHUNK) as usize..].iter().map(|&x| quantise(p, x)).collect()))
                });
                self.chunks.push((key, c));
                self.chunks.len() - 1
            };
            self.last[li] = k;
            let v = *self.chunks[k].1.as_ref()?.get((col - chunk * CHUNK) as usize)?;
            (v != 255).then_some(v)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn file_names_follow_the_north_west_corner() {
        assert_eq!(file_name(51.18, -0.37).0, "meta_chm_lat=60.0_lon=-10.0");
        assert_eq!(file_name(47.26, 11.39).0, "meta_chm_lat=50.0_lon=10.0");
        assert_eq!(file_name(-3.0, 15.0).0, "meta_chm_lat=-0.0_lon=10.0");
        assert_eq!(file_name(-13.0, -55.0).0, "meta_chm_lat=-10.0_lon=-60.0");
        assert_eq!(file_name(50.0, 0.0).0, "meta_chm_lat=50.0_lon=0.0");
    }

    #[test]
    fn pixels() {
        assert_eq!(pixel(51.5, -0.5, 60, -10), Some((38000, 34000)));
        assert_eq!(pixel(50.0, 0.0, 50, 0), Some((0, 0)));
        assert_eq!(pixel(61.0, -5.0, 60, -10), None);
    }

    #[test]
    fn quantises_to_bytes() {
        assert_eq!(quantise(Param::P95, 2549), 25);
        assert_eq!(quantise(Param::P95, 60000), 254);
        assert_eq!(quantise(Param::Cover, 1000), 250);
        assert_eq!(quantise(Param::Cover, 0), 0);
        assert_eq!(quantise(Param::Cover, NODATA), 255);
    }

    #[test]
    fn bands_pack_and_unpack() {
        let p = pack(&[b"abc", b"", b"de"]);
        let b = unpack(p).unwrap();
        assert_eq!(b.rows.len(), 3);
        let (s, e) = b.rows[2];
        assert_eq!(&b.data[s..e], b"de");
        // Truncated / garbage packs are rejected.
        let mut bad = pack(&[b"abc"]);
        bad.pop();
        assert!(unpack(bad).is_none());
        assert!(unpack(vec![255, 255, 255, 255]).is_none());
    }
}
