//! Minimal (Big)TIFF reader for the raster layers (land cover, canopy
//! height, Copernicus DEM): enough of the format to range-read
//! Cloud-Optimised GeoTIFFs — the image file directories (one per overview),
//! tile / strip offsets, the GeoTIFF pixel scale and tie point (or a plain
//! scale + offset ModelTransformation), raster type —
//! and to decode one tile or strip (none / DEFLATE / LZW; horizontal or
//! floating-point predictor; 8- or 16-bit integer or 32-bit float
//! single-band samples).
//!
//! Everything read here comes off the network: every offset, count and size is
//! bounds-checked, decoded blocks are capped at their declared size, and
//! malformed input is an error, never a panic.

use std::io::Read;

/// One image (full resolution or an overview).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ifd {
    pub width: u32,
    pub height: u32,
    /// Block size: tiles, or (width × rows-per-strip) for stripped images.
    pub block_w: u32,
    pub block_h: u32,
    pub tiled: bool,
    pub bits: u16,
    pub compression: u16,
    pub predictor: u16,
    pub offsets: Vec<u64>,
    pub counts: Vec<u64>,
    /// GeoTIFF ModelPixelScale (x, y, z) and ModelTiepoint (i, j, k, x, y, z).
    pub scale: Option<[f64; 3]>,
    pub tie: Option<[f64; 6]>,
    /// SampleFormat: 1 unsigned integer (default), 2 signed, 3 IEEE float.
    pub format: u16,
    /// GeoTIFF GTRasterTypeGeoKey is RasterPixelIsPoint: the tie point is a
    /// pixel *centre* (otherwise its corner).
    pub pixel_is_point: bool,
}

impl Ifd {
    pub fn blocks_across(&self) -> u32 {
        self.width.div_ceil(self.block_w.max(1))
    }
    pub fn blocks_down(&self) -> u32 {
        self.height.div_ceil(self.block_h.max(1))
    }
    /// Samples in one decoded block (a stripped image's last strip may be short,
    /// but is decoded to the full block like a tile).
    pub fn block_samples(&self) -> usize {
        self.block_w as usize * self.block_h as usize
    }
}

/// Header parse result.
#[derive(Debug, PartialEq)]
pub enum Parse {
    Ok {
        big_endian: bool,
        ifds: Vec<Ifd>,
    },
    /// The header bytes end too early: fetch at least this many and retry.
    Need(u64),
}

const MAX_IFDS: usize = 16;
const MAX_BLOCKS: u64 = 1 << 20;

struct Rd<'a> {
    b: &'a [u8],
    be: bool,
}

impl Rd<'_> {
    fn bytes(&self, off: u64, n: u64) -> Result<&[u8], u64> {
        let end = off.checked_add(n).ok_or(u64::MAX)?;
        if end > self.b.len() as u64 {
            return Err(end);
        }
        Ok(&self.b[off as usize..end as usize])
    }
    fn u(&self, off: u64, n: u64) -> Result<u64, u64> {
        let s = self.bytes(off, n)?;
        let mut v = 0u64;
        for k in 0..n as usize {
            let byte = if self.be { s[k] } else { s[n as usize - 1 - k] };
            v = v << 8 | byte as u64;
        }
        Ok(v)
    }
}

fn type_size(t: u16) -> Option<u64> {
    Some(match t {
        1 | 2 | 6 | 7 => 1,
        3 | 8 => 2,
        4 | 9 | 11 => 4,
        5 | 10 | 12 | 16 | 17 | 18 => 8,
        _ => return None,
    })
}

/// Parse the image file directories from the first bytes of a TIFF.
pub fn parse(b: &[u8]) -> Result<Parse, String> {
    if b.len() < 16 {
        return Ok(Parse::Need(16));
    }
    let be = match &b[..2] {
        b"II" => false,
        b"MM" => true,
        _ => return Err("not a TIFF".into()),
    };
    let r = Rd { b, be };
    let need = |e: u64| -> Result<Parse, String> { if e == u64::MAX { Err("TIFF offset overflow".into()) } else { Ok(Parse::Need(e)) } };
    let magic = r.u(2, 2).unwrap_or(0);
    let big = match magic {
        42 => false,
        43 => true,
        _ => return Err(format!("bad TIFF magic {magic}")),
    };
    let mut off = if big { r.u(8, 8).unwrap_or(0) } else { r.u(4, 4).unwrap_or(0) };
    let mut ifds = Vec::new();
    while off != 0 && ifds.len() < MAX_IFDS {
        let (n, p, es) = if big {
            match r.u(off, 8) {
                Ok(n) => (n, off + 8, 20u64),
                Err(e) => return need(e),
            }
        } else {
            match r.u(off, 2) {
                Ok(n) => (n, off + 2, 12u64),
                Err(e) => return need(e),
            }
        };
        if n > 4096 {
            return Err("TIFF directory too large".into());
        }
        let mut d = Ifd { compression: 1, predictor: 1, bits: 8, format: 1, ..Default::default() };
        let mut rows_per_strip = None;
        let (mut tw, mut th) = (None, None);
        for i in 0..n {
            let e = p + i * es;
            let (tag, typ) = match (r.u(e, 2), r.u(e + 2, 2)) {
                (Ok(t), Ok(y)) => (t as u16, y as u16),
                (Err(x), _) | (_, Err(x)) => return need(x),
            };
            let cnt = match r.u(e + 4, if big { 8 } else { 4 }) {
                Ok(c) => c,
                Err(x) => return need(x),
            };
            let Some(sz) = type_size(typ) else { continue };
            let total = sz.checked_mul(cnt).ok_or("TIFF tag size overflow")?;
            let inline = if big { 8 } else { 4 };
            let voff = e + if big { 12 } else { 8 };
            let data_off = if total <= inline {
                voff
            } else {
                match r.u(voff, inline) {
                    Ok(o) => o,
                    Err(x) => return need(x),
                }
            };
            // Only the tags we use are read (and may require more bytes).
            let wanted = matches!(tag, 256 | 257 | 258 | 259 | 273 | 278 | 279 | 317 | 322 | 323 | 324 | 325 | 339 | 33550 | 33922 | 34264 | 34735);
            if !wanted {
                continue;
            }
            if cnt > MAX_BLOCKS {
                return Err("TIFF tag count too large".into());
            }
            let vals = || -> Result<Vec<u64>, u64> { (0..cnt).map(|k| r.u(data_off + k * sz, sz)).collect() };
            let dbl = || -> Result<Vec<f64>, u64> { (0..cnt).map(|k| r.u(data_off + k * 8, 8).map(f64::from_bits)).collect() };
            let one = |v: Result<Vec<u64>, u64>| v.map(|v| v.first().copied().unwrap_or(0));
            let res: Result<(), u64> = (|| {
                match tag {
                    256 => d.width = one(vals())? as u32,
                    257 => d.height = one(vals())? as u32,
                    258 => d.bits = one(vals())? as u16,
                    259 => d.compression = one(vals())? as u16,
                    278 => rows_per_strip = Some(one(vals())? as u32),
                    317 => d.predictor = one(vals())? as u16,
                    339 => d.format = one(vals())? as u16,
                    // GeoKeyDirectory: 4-short header, then (key, location, count, value).
                    34735 if typ == 3 => {
                        let v = vals()?;
                        d.pixel_is_point = v.get(4..).unwrap_or(&[]).chunks_exact(4).any(|k| k[0] == 1025 && k[1] == 0 && k[3] == 2);
                    }
                    322 => tw = Some(one(vals())? as u32),
                    323 => th = Some(one(vals())? as u32),
                    273 | 324 => d.offsets = vals()?,
                    279 | 325 => d.counts = vals()?,
                    33550 if typ == 12 && cnt >= 3 => {
                        let v = dbl()?;
                        d.scale = Some([v[0], v[1], v[2]]);
                    }
                    33922 if typ == 12 && cnt >= 6 => {
                        let v = dbl()?;
                        d.tie = Some([v[0], v[1], v[2], v[3], v[4], v[5]]);
                    }
                    // ModelTransformation (a 4×4 matrix; e.g. GeoServer's WCS
                    // output): read when it is a plain scale + offset.
                    34264 if typ == 12 && cnt == 16 => {
                        let v = dbl()?;
                        if v[1] == 0.0 && v[4] == 0.0 && v[0] > 0.0 && v[5] < 0.0 && d.scale.is_none() && d.tie.is_none() {
                            d.scale = Some([v[0], -v[5], 0.0]);
                            d.tie = Some([0.0, 0.0, 0.0, v[3], v[7], 0.0]);
                        }
                    }
                    _ => {}
                }
                Ok(())
            })();
            if let Err(x) = res {
                return need(x);
            }
        }
        match (tw, th) {
            (Some(w), Some(h)) => {
                d.tiled = true;
                d.block_w = w;
                d.block_h = h;
            }
            _ => {
                d.block_w = d.width;
                d.block_h = rows_per_strip.unwrap_or(d.height).min(d.height.max(1));
            }
        }
        if d.width == 0 || d.height == 0 || d.block_w == 0 || d.block_h == 0 || d.block_w > 1 << 17 || d.block_h > 1 << 17 {
            return Err("bad TIFF image size".into());
        }
        let blocks = d.blocks_across() as u64 * d.blocks_down() as u64;
        if d.offsets.len() as u64 != blocks || d.counts.len() as u64 != blocks {
            return Err(format!("TIFF block table: {} offsets, {} counts, {} blocks", d.offsets.len(), d.counts.len(), blocks));
        }
        if !matches!((d.bits, d.format), (8 | 16, 1 | 2) | (32, 3)) {
            return Err(format!("unsupported TIFF samples: {} bits, format {}", d.bits, d.format));
        }
        let pe = p + n * es;
        off = match r.u(pe, if big { 8 } else { 4 }) {
            Ok(o) => o,
            Err(x) => return need(x),
        };
        ifds.push(d);
    }
    if ifds.is_empty() {
        return Err("TIFF has no images".into());
    }
    Ok(Parse::Ok { big_endian: be, ifds })
}

/// Decode one block's compressed bytes into its samples (as u16; 8-bit
/// samples widen). `max_samples` stops early (rows are decoded in order).
pub fn decode_block(ifd: &Ifd, big_endian: bool, data: &[u8], max_samples: Option<usize>) -> Result<Vec<u16>, String> {
    if ifd.bits > 16 {
        return Err(format!("{}-bit samples are not integer samples", ifd.bits));
    }
    let full = ifd.block_samples();
    let want = max_samples.map_or(full, |m| m.min(full));
    let bps = (ifd.bits / 8) as usize;
    // The predictor works on whole rows: decode whole rows.
    let row = ifd.block_w as usize;
    let want_rows = want.div_ceil(row);
    let raw = inflate(ifd, data, want_rows * row * bps)?;
    // Short blocks (truncated data) are padded with zeros.
    let mut v: Vec<u16> = match bps {
        1 => raw.iter().map(|&b| b as u16).collect(),
        _ => raw.chunks_exact(2).map(|c| if big_endian { u16::from_be_bytes([c[0], c[1]]) } else { u16::from_le_bytes([c[0], c[1]]) }).collect(),
    };
    v.resize(want_rows * row, 0);
    if ifd.predictor == 2 {
        let mask = if bps == 1 { 0xff } else { 0xffff };
        for r in v.chunks_exact_mut(row) {
            for i in 1..r.len() {
                r[i] = r[i].wrapping_add(r[i - 1]) & mask;
            }
        }
    } else if ifd.predictor != 1 {
        return Err(format!("unsupported TIFF predictor {}", ifd.predictor));
    }
    v.truncate(want);
    Ok(v)
}

/// Uncompress a block's bytes (at most `limit`).
fn inflate(ifd: &Ifd, data: &[u8], limit: usize) -> Result<Vec<u8>, String> {
    Ok(match ifd.compression {
        1 => data[..data.len().min(limit)].to_vec(),
        8 | 32946 => {
            let mut out = Vec::with_capacity(limit);
            flate2::read::ZlibDecoder::new(data).take(limit as u64).read_to_end(&mut out).map_err(|e| format!("deflate: {e}"))?;
            out
        }
        5 => lzw(data, limit)?,
        c => return Err(format!("unsupported TIFF compression {c}")),
    })
}

/// Decode one block of 32-bit float samples (the whole block; truncated data
/// is padded with NaN). Predictors: none, or floating point (3: per row, the
/// bytes of all samples are split into planes, most significant first, and
/// differenced byte by byte — TIFF Technical Note 3).
pub fn decode_block_f32(ifd: &Ifd, big_endian: bool, data: &[u8]) -> Result<Vec<f32>, String> {
    if ifd.bits != 32 || ifd.format != 3 {
        return Err(format!("not float samples ({} bits, format {})", ifd.bits, ifd.format));
    }
    let row = ifd.block_w as usize;
    let n = ifd.block_samples();
    let raw = inflate(ifd, data, n * 4)?;
    let rows = raw.len() / (row * 4);
    let mut v = Vec::with_capacity(n);
    match ifd.predictor {
        1 => {
            for c in raw[..rows * row * 4].chunks_exact(4) {
                let b = [c[0], c[1], c[2], c[3]];
                v.push(if big_endian { f32::from_be_bytes(b) } else { f32::from_le_bytes(b) });
            }
        }
        3 => {
            let mut r = vec![0u8; row * 4];
            for src in raw.chunks_exact(row * 4) {
                r.copy_from_slice(src);
                for i in 1..r.len() {
                    r[i] = r[i].wrapping_add(r[i - 1]);
                }
                for i in 0..row {
                    v.push(f32::from_be_bytes([r[i], r[row + i], r[2 * row + i], r[3 * row + i]]));
                }
            }
        }
        p => return Err(format!("unsupported TIFF predictor {p} for float samples")),
    }
    v.resize(n, f32::NAN);
    Ok(v)
}

/// TIFF LZW (MSB-first codes, "early change"), output capped at `limit` bytes.
pub fn lzw(data: &[u8], limit: usize) -> Result<Vec<u8>, String> {
    const CLEAR: u16 = 256;
    const EOI: u16 = 257;
    // Table entry: (prefix code, last byte, length).
    let mut prefix = vec![0u16; 4096];
    let mut suffix = vec![0u8; 4096];
    let mut len = vec![0u16; 4096];
    for i in 0..256 {
        suffix[i] = i as u8;
        len[i] = 1;
    }
    let mut next = 258u16;
    let mut width = 9u32;
    let mut out: Vec<u8> = Vec::with_capacity(limit);
    let (mut acc, mut nbits, mut pos) = (0u32, 0u32, 0usize);
    let mut prev: Option<u16> = None;
    // Emit an entry by walking its prefix chain backwards.
    let emit = |out: &mut Vec<u8>, code: u16, prefix: &[u16], suffix: &[u8], len: &[u16]| {
        let n = len[code as usize] as usize;
        let start = out.len();
        out.resize(start + n, 0);
        let mut c = code;
        for k in (0..n).rev() {
            out[start + k] = suffix[c as usize];
            c = prefix[c as usize];
        }
        out[start]
    };
    while out.len() < limit {
        while nbits < width {
            let Some(&b) = data.get(pos) else { return Ok(out) };
            acc = acc << 8 | b as u32;
            pos += 1;
            nbits += 8;
        }
        let code = ((acc >> (nbits - width)) & ((1 << width) - 1)) as u16;
        nbits -= width;
        if code == CLEAR {
            next = 258;
            width = 9;
            prev = None;
            continue;
        }
        if code == EOI {
            break;
        }
        let Some(p) = prev else {
            if code > 255 {
                return Err("LZW: bad first code".into());
            }
            out.push(code as u8);
            prev = Some(code);
            continue;
        };
        if next as usize >= 4096 {
            return Err("LZW: table overflow".into());
        }
        let first = if code < next {
            if len[code as usize] == 0 {
                return Err("LZW: bad code".into());
            }
            emit(&mut out, code, &prefix, &suffix, &len)
        } else if code == next {
            // KwKwK: the previous string plus its own first byte.
            let f = emit(&mut out, p, &prefix, &suffix, &len);
            out.push(f);
            f
        } else {
            return Err("LZW: code out of range".into());
        };
        prefix[next as usize] = p;
        suffix[next as usize] = first;
        len[next as usize] = len[p as usize].saturating_add(1);
        next += 1;
        // Early change: widen one code before the table fills the width.
        if next as u32 + 1 >= 1 << width && width < 12 {
            width += 1;
        }
        prev = Some(code);
    }
    out.truncate(limit);
    Ok(out)
}

/// Test fixtures: a tiny tiled / stripped TIFF writer and an LZW encoder.
#[cfg(test)]
pub mod fixture {
    use std::io::Write;

    /// TIFF LZW encoder (for the decoder's round-trip tests).
    pub fn lzw_encode(data: &[u8]) -> Vec<u8> {
        use std::collections::HashMap;
        struct Bits {
            out: Vec<u8>,
            acc: u64,
            n: u32,
        }
        impl Bits {
            fn put(&mut self, code: u32, width: u32) {
                self.acc = self.acc << width | code as u64;
                self.n += width;
                while self.n >= 8 {
                    self.out.push((self.acc >> (self.n - 8)) as u8);
                    self.n -= 8;
                }
            }
        }
        let mut b = Bits { out: Vec::new(), acc: 0, n: 0 };
        let mut table: HashMap<Vec<u8>, u32> = HashMap::new();
        let reset = |t: &mut HashMap<Vec<u8>, u32>| {
            t.clear();
            for i in 0..256u32 {
                t.insert(vec![i as u8], i);
            }
        };
        reset(&mut table);
        let mut next = 258u32;
        let mut width = 9u32;
        b.put(256, width);
        let mut w: Vec<u8> = Vec::new();
        for &c in data {
            let mut wc = w.clone();
            wc.push(c);
            if table.contains_key(&wc) {
                w = wc;
                continue;
            }
            b.put(table[&w], width);
            table.insert(wc, next);
            next += 1;
            if next + 1 > (1 << width) && width < 12 {
                width += 1;
            }
            if next >= 4093 {
                b.put(256, width);
                reset(&mut table);
                next = 258;
                width = 9;
            }
            w = vec![c];
        }
        if !w.is_empty() {
            b.put(table[&w], width);
            next += 1;
            if next + 1 > (1 << width) && width < 12 {
                width += 1;
            }
        }
        b.put(257, width);
        if b.n > 0 {
            let pad = 8 - b.n;
            b.put(0, pad);
        }
        b.out
    }

    /// A little-endian classic TIFF: `w`×`h` 8-bit image in `bw`×`bh` tiles
    /// (`tiled`) or `bh`-row strips, DEFLATE- (8) or LZW- (5) compressed, with
    /// a GeoTIFF scale / tie point. Returns the file bytes.
    pub fn tiff(w: u32, h: u32, bw: u32, bh: u32, tiled: bool, compression: u16, px: impl Fn(u32, u32) -> u8, scale: [f64; 3], tie: [f64; 6]) -> Vec<u8> {
        let (bw, ba, bd) = if tiled { (bw, w.div_ceil(bw), h.div_ceil(bh)) } else { (w, 1, h.div_ceil(bh)) };
        let mut blocks = Vec::new();
        for by in 0..bd {
            for bx in 0..ba {
                let mut raw = Vec::new();
                for y in 0..bh {
                    for x in 0..bw {
                        let (gx, gy) = (bx * bw + x, by * bh + y);
                        raw.push(if gx < w && gy < h { px(gx, gy) } else { 0 });
                    }
                }
                let c = match compression {
                    8 => {
                        let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
                        e.write_all(&raw).unwrap();
                        e.finish().unwrap()
                    }
                    5 => lzw_encode(&raw),
                    _ => raw,
                };
                blocks.push(c);
            }
        }
        let n = blocks.len() as u32;
        // Layout: header (8) | IFD | arrays | geo | block data.
        let tags = 11u32;
        let ifd_off = 8u32;
        let arr_off = ifd_off + 2 + tags * 12 + 4;
        let geo_off = arr_off + n * 8;
        let mut data_off = geo_off + 72;
        let mut out = b"II".to_vec();
        out.extend(42u16.to_le_bytes());
        out.extend(ifd_off.to_le_bytes());
        out.extend((tags as u16).to_le_bytes());
        let tag = |out: &mut Vec<u8>, t: u16, typ: u16, cnt: u32, val: u32| {
            out.extend(t.to_le_bytes());
            out.extend(typ.to_le_bytes());
            out.extend(cnt.to_le_bytes());
            if typ == 3 && cnt == 1 {
                out.extend((val as u16).to_le_bytes());
                out.extend([0, 0]);
            } else {
                out.extend(val.to_le_bytes());
            }
        };
        tag(&mut out, 256, 3, 1, w);
        tag(&mut out, 257, 3, 1, h);
        tag(&mut out, 258, 3, 1, 8);
        tag(&mut out, 259, 3, 1, compression as u32);
        if tiled {
            tag(&mut out, 322, 3, 1, bw);
            tag(&mut out, 323, 3, 1, bh);
            tag(&mut out, 324, 4, n, arr_off);
            tag(&mut out, 325, 4, n, arr_off + n * 4);
        } else {
            tag(&mut out, 273, 4, n, arr_off);
            tag(&mut out, 278, 3, 1, bh);
            tag(&mut out, 279, 4, n, arr_off + n * 4);
            tag(&mut out, 284, 3, 1, 1);
        }
        tag(&mut out, 277, 3, 1, 1);
        tag(&mut out, 33550, 12, 3, geo_off);
        tag(&mut out, 33922, 12, 6, geo_off + 24);
        out.extend(0u32.to_le_bytes());
        assert_eq!(out.len() as u32, arr_off);
        for b in &blocks {
            out.extend(data_off.to_le_bytes());
            data_off += b.len() as u32;
        }
        for b in &blocks {
            out.extend((b.len() as u32).to_le_bytes());
        }
        for v in scale.iter().chain(tie.iter()) {
            out.extend(v.to_le_bytes());
        }
        for b in &blocks {
            out.extend(b);
        }
        out
    }
    /// A little-endian tiled float TIFF (DEFLATE, floating-point predictor
    /// 3, SampleFormat 3, GeoKeyDirectory with RasterPixelIsPoint if `point`),
    /// laid out like the Copernicus DEM COGs.
    pub fn tiff_f32(w: u32, h: u32, bw: u32, bh: u32, px: impl Fn(u32, u32) -> f32, scale: [f64; 3], tie: [f64; 6], point: bool) -> Vec<u8> {
        let (ba, bd) = (w.div_ceil(bw), h.div_ceil(bh));
        let mut blocks = Vec::new();
        for by in 0..bd {
            for bx in 0..ba {
                let mut raw = Vec::new();
                for y in 0..bh {
                    let vals: Vec<[u8; 4]> = (0..bw)
                        .map(|x| {
                            let (gx, gy) = (bx * bw + x, by * bh + y);
                            (if gx < w && gy < h { px(gx, gy) } else { 0.0 }).to_be_bytes()
                        })
                        .collect();
                    // Byte planes, most significant first, then byte differences.
                    let mut r: Vec<u8> = (0..4).flat_map(|k| vals.iter().map(move |v| v[k])).collect();
                    for i in (1..r.len()).rev() {
                        r[i] = r[i].wrapping_sub(r[i - 1]);
                    }
                    raw.extend(r);
                }
                let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
                e.write_all(&raw).unwrap();
                blocks.push(e.finish().unwrap());
            }
        }
        let n = blocks.len() as u32;
        let tags = 14u32;
        let ifd_off = 8u32;
        let arr_off = ifd_off + 2 + tags * 12 + 4;
        let geo_off = arr_off + n * 8;
        let keys_off = geo_off + 72;
        let mut data_off = keys_off + 16;
        let mut out = b"II".to_vec();
        out.extend(42u16.to_le_bytes());
        out.extend(ifd_off.to_le_bytes());
        out.extend((tags as u16).to_le_bytes());
        let tag = |out: &mut Vec<u8>, t: u16, typ: u16, cnt: u32, val: u32| {
            out.extend(t.to_le_bytes());
            out.extend(typ.to_le_bytes());
            out.extend(cnt.to_le_bytes());
            if typ == 3 && cnt == 1 {
                out.extend((val as u16).to_le_bytes());
                out.extend([0, 0]);
            } else {
                out.extend(val.to_le_bytes());
            }
        };
        tag(&mut out, 256, 3, 1, w);
        tag(&mut out, 257, 3, 1, h);
        tag(&mut out, 258, 3, 1, 32);
        tag(&mut out, 259, 3, 1, 8);
        tag(&mut out, 277, 3, 1, 1);
        tag(&mut out, 317, 3, 1, 3);
        tag(&mut out, 322, 3, 1, bw);
        tag(&mut out, 323, 3, 1, bh);
        tag(&mut out, 324, 4, n, arr_off);
        tag(&mut out, 325, 4, n, arr_off + n * 4);
        tag(&mut out, 339, 3, 1, 3);
        tag(&mut out, 33550, 12, 3, geo_off);
        tag(&mut out, 33922, 12, 6, geo_off + 24);
        tag(&mut out, 34735, 3, 8, keys_off);
        out.extend(0u32.to_le_bytes());
        assert_eq!(out.len() as u32, arr_off);
        for b in &blocks {
            out.extend(data_off.to_le_bytes());
            data_off += b.len() as u32;
        }
        for b in &blocks {
            out.extend((b.len() as u32).to_le_bytes());
        }
        for v in scale.iter().chain(tie.iter()) {
            out.extend(v.to_le_bytes());
        }
        for k in [1u16, 1, 0, 1, 1025, 0, 1, if point { 2 } else { 1 }] {
            out.extend(k.to_le_bytes());
        }
        for b in &blocks {
            out.extend(b);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::fixture::*;
    use super::*;

    fn ok(p: Parse) -> (bool, Vec<Ifd>) {
        match p {
            Parse::Ok { big_endian, ifds } => (big_endian, ifds),
            Parse::Need(n) => panic!("needs {n} bytes"),
        }
    }

    #[test]
    fn lzw_round_trips() {
        let mut data = Vec::new();
        let mut s = 7u32;
        for i in 0..20000u32 {
            s = s.wrapping_mul(1103515245).wrapping_add(12345);
            // Runs and noise, enough to overflow the table and clear it.
            data.push(if i % 300 < 150 { (i / 40) as u8 } else { (s >> 24) as u8 });
        }
        let enc = lzw_encode(&data);
        assert_eq!(lzw(&enc, data.len()).unwrap(), data);
        // Early stop.
        assert_eq!(lzw(&enc, 1000).unwrap(), data[..1000]);
        assert_eq!(lzw(&lzw_encode(b"ABABABABABAB"), 100).unwrap(), b"ABABABABABAB");
    }

    #[test]
    fn decodes_a_tiled_deflate_tiff() {
        let px = |x: u32, y: u32| ((x * 7 + y * 3) % 251) as u8;
        let f = tiff(40, 30, 16, 16, true, 8, px, [0.5, 0.5, 0.0], [0.0, 0.0, 0.0, -3.0, 54.0, 0.0]);
        // A short read asks for more.
        assert!(matches!(parse(&f[..20]).unwrap(), Parse::Need(_)));
        let (be, ifds) = ok(parse(&f).unwrap());
        let d = &ifds[0];
        assert!(!be && d.tiled);
        assert_eq!((d.width, d.height, d.block_w, d.block_h, d.blocks_across(), d.blocks_down()), (40, 30, 16, 16, 3, 2));
        assert_eq!(d.scale, Some([0.5, 0.5, 0.0]));
        assert_eq!(d.tie.unwrap()[4], 54.0);
        // Tile (2, 1): pixels x 32..48, y 16..32 (zero past the edge).
        let k = 3 + 2;
        let blk = &f[d.offsets[k] as usize..(d.offsets[k] + d.counts[k]) as usize];
        let v = decode_block(d, be, blk, None).unwrap();
        assert_eq!(v.len(), 256);
        assert_eq!(v[0], px(32, 16) as u16);
        assert_eq!(v[16 * 5 + 7], px(39, 21) as u16);
        assert_eq!(v[15], 0);
    }

    #[test]
    fn decodes_lzw_strips() {
        let px = |x: u32, y: u32| ((x ^ y) & 0xff) as u8;
        let f = tiff(300, 5, 0, 1, false, 5, px, [1.0, 1.0, 0.0], [0.0; 6]);
        let (be, ifds) = ok(parse(&f).unwrap());
        let d = &ifds[0];
        assert!(!d.tiled);
        assert_eq!((d.block_w, d.block_h, d.offsets.len()), (300, 1, 5));
        let blk = &f[d.offsets[3] as usize..(d.offsets[3] + d.counts[3]) as usize];
        let v = decode_block(d, be, blk, Some(120)).unwrap();
        assert_eq!(v.len(), 120);
        assert!(v.iter().enumerate().all(|(x, &s)| s == px(x as u32, 3) as u16));
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse(b"not a tiff at all!!").is_err());
        let mut f = tiff(8, 8, 8, 8, true, 8, |_, _| 1, [1.0; 3], [0.0; 6]);
        // Corrupt the width: the block table no longer matches.
        f[8 + 2 + 8] = 200;
        assert!(parse(&f).is_err());
        let d = Ifd { width: 4, height: 1, block_w: 4, block_h: 1, bits: 8, compression: 8, predictor: 1, ..Default::default() };
        assert!(decode_block(&d, false, b"\x00\x01garbage", None).is_err());
    }

    #[test]
    fn horizontal_predictor() {
        let d = Ifd { width: 4, height: 2, block_w: 4, block_h: 2, bits: 16, compression: 1, predictor: 2, ..Default::default() };
        let raw: Vec<u8> = [100u16, 1, 2, 65535, 5, 5, 5, 5].iter().flat_map(|v| v.to_le_bytes()).collect();
        assert_eq!(decode_block(&d, false, &raw, None).unwrap(), vec![100, 101, 103, 102, 5, 10, 15, 20]);
    }

    #[test]
    fn decodes_float_samples_with_the_floating_point_predictor() {
        let px = |x: u32, y: u32| 1000.0 + x as f32 * 0.37 - y as f32 * 12.5 + if (x + y) % 7 == 0 { -3.25 } else { 0.0 };
        let f = tiff_f32(40, 20, 16, 16, px, [0.25, 0.25, 0.0], [0.0, 0.0, 0.0, 11.0, 48.0, 0.0], true);
        let (be, ifds) = ok(parse(&f).unwrap());
        let d = &ifds[0];
        assert_eq!((d.bits, d.format, d.predictor, d.pixel_is_point), (32, 3, 3, true));
        let k = 3 + 1;
        let blk = &f[d.offsets[k] as usize..(d.offsets[k] + d.counts[k]) as usize];
        let v = decode_block_f32(d, be, blk).unwrap();
        assert_eq!(v.len(), 256);
        assert_eq!(v[0], px(16, 16));
        assert_eq!(v[3 * 16 + 9], px(25, 19));
        // Integer decoding refuses float samples (and vice versa).
        assert!(decode_block(d, be, blk, None).is_err());
        let (_, ifds) = ok(parse(&tiff_f32(8, 8, 8, 8, |_, _| 1.0, [1.0; 3], [0.0; 6], false)).unwrap());
        assert!(!ifds[0].pixel_is_point);
        let i8 = Ifd { width: 4, height: 1, block_w: 4, block_h: 1, bits: 8, format: 1, compression: 1, predictor: 1, ..Default::default() };
        assert!(decode_block_f32(&i8, false, &[0; 4]).is_err());
    }
}
