//! Minimal PMTiles v3 reader: header, directories, tile lookup.
//! Spec: https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md
//!
//! Only the parsing lives here (pure, bounds-checked: archives are untrusted
//! network data); the range reads are the caller's.

use std::io::Read;

/// Bytes to read for the header plus (in practice) the root directory.
pub const HEAD_LEN: u64 = 16_384;
const HEADER_LEN: usize = 127;
/// Sanity caps on directory sizes (a real leaf holds a few thousand entries).
const MAX_ENTRIES: u64 = 1 << 20;
const MAX_INFLATED: u64 = 64 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Compression {
    None,
    Gzip,
}

#[derive(Clone, Debug)]
pub struct Header {
    pub root: (u64, u64),
    pub leaves_offset: u64,
    pub data_offset: u64,
    pub internal: Compression,
    pub tile: Compression,
    /// 1 = MVT.
    pub tile_type: u8,
    pub min_zoom: u8,
    pub max_zoom: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    pub tile_id: u64,
    pub offset: u64,
    pub length: u32,
    /// 0 = a leaf directory, else the number of consecutive tiles sharing the data.
    pub run_length: u32,
}

fn u64_at(b: &[u8], i: usize) -> u64 {
    u64::from_le_bytes(b[i..i + 8].try_into().unwrap())
}

fn compression(c: u8) -> Result<Compression, String> {
    match c {
        1 => Ok(Compression::None),
        2 => Ok(Compression::Gzip),
        c => Err(format!("pmtiles: unsupported compression {c}")),
    }
}

pub fn header(b: &[u8]) -> Result<Header, String> {
    if b.len() < HEADER_LEN || &b[..7] != b"PMTiles" {
        return Err("pmtiles: not a PMTiles archive".into());
    }
    if b[7] != 3 {
        return Err(format!("pmtiles: version {} (want 3)", b[7]));
    }
    Ok(Header {
        root: (u64_at(b, 8), u64_at(b, 16)),
        leaves_offset: u64_at(b, 40),
        data_offset: u64_at(b, 56),
        internal: compression(b[97])?,
        tile: compression(b[98])?,
        tile_type: b[99],
        min_zoom: b[100],
        max_zoom: b[101],
    })
}

/// Undo `c` (bounded: at most 64 MB out).
pub fn inflate(b: &[u8], c: Compression) -> Result<Vec<u8>, String> {
    match c {
        Compression::None => Ok(b.to_vec()),
        Compression::Gzip => {
            let mut out = Vec::with_capacity(b.len() * 4);
            flate2::read::GzDecoder::new(b).take(MAX_INFLATED).read_to_end(&mut out).map_err(|e| format!("pmtiles: gunzip: {e}"))?;
            Ok(out)
        }
    }
}

fn varint(b: &[u8], i: &mut usize) -> Result<u64, String> {
    let mut v = 0u64;
    let mut shift = 0;
    loop {
        let byte = *b.get(*i).ok_or("pmtiles: truncated directory")?;
        *i += 1;
        v |= ((byte & 0x7f) as u64) << shift;
        if byte & 0x80 == 0 {
            return Ok(v);
        }
        shift += 7;
        if shift > 63 {
            return Err("pmtiles: bad varint".into());
        }
    }
}

/// Decode a (decompressed) directory.
pub fn directory(b: &[u8]) -> Result<Vec<Entry>, String> {
    let mut i = 0;
    let n = varint(b, &mut i)?;
    if n > MAX_ENTRIES {
        return Err(format!("pmtiles: directory of {n} entries"));
    }
    let n = n as usize;
    let mut e = vec![Entry { tile_id: 0, offset: 0, length: 0, run_length: 0 }; n];
    let mut last = 0u64;
    for x in e.iter_mut() {
        last = last.checked_add(varint(b, &mut i)?).ok_or("pmtiles: tile id overflow")?;
        x.tile_id = last;
    }
    for x in e.iter_mut() {
        x.run_length = varint(b, &mut i)? as u32;
    }
    for x in e.iter_mut() {
        x.length = varint(b, &mut i)? as u32;
    }
    for k in 0..n {
        let v = varint(b, &mut i)?;
        e[k].offset = if v == 0 && k > 0 { e[k - 1].offset + e[k - 1].length as u64 } else { v.checked_sub(1).ok_or("pmtiles: bad offset")? };
    }
    Ok(e)
}

/// Hilbert tile id of z/x/y.
pub fn tile_id(z: u8, x: u32, y: u32) -> u64 {
    let mut acc = ((1u64 << (2 * z as u32)) - 1) / 3;
    let (mut x, mut y) = (x as u64, y as u64);
    for a in (0..z as u32).rev() {
        let s = 1u64 << a;
        let rx = (x & s != 0) as u64;
        let ry = (y & s != 0) as u64;
        acc += ((3 * rx) ^ ry) << (2 * a);
        if ry == 0 {
            if rx == 1 {
                x = (s - 1).wrapping_sub(x);
                y = (s - 1).wrapping_sub(y);
            }
            std::mem::swap(&mut x, &mut y);
        }
    }
    acc
}

/// The entry for `tid` in a directory: its tile data, a leaf directory to
/// search next, or `None` (no such tile: empty).
pub fn find(dir: &[Entry], tid: u64) -> Option<Entry> {
    let i = dir.partition_point(|e| e.tile_id <= tid);
    let e = *dir.get(i.checked_sub(1)?)?;
    (e.run_length == 0 || tid - e.tile_id < e.run_length as u64).then_some(e)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::io::Write;

    fn put_varint(out: &mut Vec<u8>, mut v: u64) {
        while v >= 0x80 {
            out.push((v as u8 & 0x7f) | 0x80);
            v >>= 7;
        }
        out.push(v as u8);
    }

    pub fn encode_directory(e: &[Entry]) -> Vec<u8> {
        let mut b = Vec::new();
        put_varint(&mut b, e.len() as u64);
        let mut last = 0;
        for x in e {
            put_varint(&mut b, x.tile_id - last);
            last = x.tile_id;
        }
        for x in e {
            put_varint(&mut b, x.run_length as u64);
        }
        for x in e {
            put_varint(&mut b, x.length as u64);
        }
        for (k, x) in e.iter().enumerate() {
            let contiguous = k > 0 && x.offset == e[k - 1].offset + e[k - 1].length as u64;
            put_varint(&mut b, if contiguous { 0 } else { x.offset + 1 });
        }
        b
    }

    pub fn gzip(b: &[u8]) -> Vec<u8> {
        let mut g = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        g.write_all(b).unwrap();
        g.finish().unwrap()
    }

    /// A one-directory gzip archive holding `tiles` ((z, x, y), MVT bytes).
    pub fn archive(tiles: &[((u8, u32, u32), Vec<u8>)]) -> Vec<u8> {
        let mut ts: Vec<(u64, Vec<u8>)> = tiles.iter().map(|((z, x, y), b)| (tile_id(*z, *x, *y), gzip(b))).collect();
        ts.sort_by_key(|t| t.0);
        let mut data = Vec::new();
        let mut entries = Vec::new();
        for (id, b) in &ts {
            entries.push(Entry { tile_id: *id, offset: data.len() as u64, length: b.len() as u32, run_length: 1 });
            data.extend_from_slice(b);
        }
        let root = gzip(&encode_directory(&entries));
        let mut h = vec![0u8; HEADER_LEN];
        h[..7].copy_from_slice(b"PMTiles");
        h[7] = 3;
        let root_off = HEADER_LEN as u64;
        let data_off = root_off + root.len() as u64;
        for (i, v) in [(8, root_off), (16, root.len() as u64), (40, data_off), (56, data_off), (64, data.len() as u64)] {
            h[i..i + 8].copy_from_slice(&u64::to_le_bytes(v));
        }
        h[97] = 2;
        h[98] = 2;
        h[99] = 1;
        h[101] = 14;
        h.extend_from_slice(&root);
        h.extend_from_slice(&data);
        h
    }

    #[test]
    fn hilbert_ids_match_the_spec() {
        assert_eq!(tile_id(0, 0, 0), 0);
        assert_eq!(tile_id(1, 0, 0), 1);
        assert_eq!(tile_id(1, 0, 1), 2);
        assert_eq!(tile_id(1, 1, 1), 3);
        assert_eq!(tile_id(1, 1, 0), 4);
        assert_eq!(tile_id(2, 0, 0), 5);
        // Every id at a zoom is distinct and in its zoom's range.
        let mut seen: Vec<u64> = (0..16).flat_map(|x| (0..16).map(move |y| tile_id(4, x, y))).collect();
        seen.sort();
        seen.dedup();
        assert_eq!(seen.len(), 256);
        assert_eq!(seen[0], 85);
        assert_eq!(seen[255], 85 + 255);
    }

    #[test]
    fn directory_round_trip_and_lookup() {
        let e = vec![
            Entry { tile_id: 10, offset: 0, length: 5, run_length: 1 },
            Entry { tile_id: 11, offset: 5, length: 7, run_length: 3 },
            Entry { tile_id: 20, offset: 100, length: 9, run_length: 0 },
        ];
        let d = directory(&encode_directory(&e)).unwrap();
        assert_eq!(d, e);
        assert_eq!(find(&d, 10).unwrap().offset, 0);
        assert_eq!(find(&d, 13).unwrap().offset, 5); // inside a run
        assert!(find(&d, 14).is_none()); // past the run: no tile
        assert!(find(&d, 9).is_none());
        assert_eq!(find(&d, 500).unwrap().run_length, 0); // leaf
        // Truncated / hostile input fails cleanly.
        assert!(directory(&[0xff]).is_err());
        assert!(directory(&[0xff, 0xff, 0xff, 0xff, 0x7f]).is_err());
    }

    #[test]
    fn reads_a_built_archive() {
        let a = archive(&[((14, 3, 5), b"hello".to_vec())]);
        let h = header(&a).unwrap();
        assert_eq!((h.internal, h.tile, h.tile_type, h.max_zoom), (Compression::Gzip, Compression::Gzip, 1, 14));
        let root = directory(&inflate(&a[h.root.0 as usize..(h.root.0 + h.root.1) as usize], h.internal).unwrap()).unwrap();
        let e = find(&root, tile_id(14, 3, 5)).unwrap();
        let o = (h.data_offset + e.offset) as usize;
        assert_eq!(inflate(&a[o..o + e.length as usize], h.tile).unwrap(), b"hello");
        assert!(find(&root, tile_id(14, 3, 6)).is_none());
        assert!(header(b"PMTiles").is_err());
    }
}
