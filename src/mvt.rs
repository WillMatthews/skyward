//! Minimal Mapbox Vector Tile (protobuf) decoder.
//! Spec: https://github.com/mapbox/vector-tile-spec/tree/master/2.1

use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Str(String),
    Num(f64),
    Bool(bool),
}

impl Value {
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            Value::Str(s) => s.parse().ok(),
            Value::Bool(b) => Some(*b as i32 as f64),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeomType {
    Unknown,
    Point,
    Line,
    Polygon,
}

#[derive(Clone, Debug)]
pub struct Feature {
    pub id: u64,
    pub geom_type: GeomType,
    pub tags: HashMap<String, Value>,
    /// Decoded geometry: list of parts (points / lines / rings) in tile units.
    pub parts: Vec<Vec<[f32; 2]>>,
}

impl Feature {
    pub fn str(&self, k: &str) -> Option<&str> {
        self.tags.get(k).and_then(|v| v.as_str())
    }
    pub fn num(&self, k: &str) -> Option<f64> {
        self.tags.get(k).and_then(|v| v.as_f64())
    }
}

#[derive(Clone, Debug)]
pub struct Layer {
    pub name: String,
    pub extent: u32,
    pub features: Vec<Feature>,
}

struct Reader<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Reader<'a> {
    fn new(b: &'a [u8]) -> Self {
        Reader { b, i: 0 }
    }
    fn done(&self) -> bool {
        self.i >= self.b.len()
    }
    fn varint(&mut self) -> Result<u64, String> {
        let mut v = 0u64;
        let mut shift = 0;
        loop {
            let byte = *self.b.get(self.i).ok_or("truncated varint")?;
            self.i += 1;
            v |= ((byte & 0x7f) as u64) << shift;
            if byte & 0x80 == 0 {
                return Ok(v);
            }
            shift += 7;
            if shift > 63 {
                return Err("varint too long".into());
            }
        }
    }
    fn bytes(&mut self) -> Result<&'a [u8], String> {
        let n = self.varint()? as usize;
        let s = self.b.get(self.i..self.i + n).ok_or("truncated bytes")?;
        self.i += n;
        Ok(s)
    }
    fn fixed32(&mut self) -> Result<u32, String> {
        let s = self.b.get(self.i..self.i + 4).ok_or("truncated fixed32")?;
        self.i += 4;
        Ok(u32::from_le_bytes(s.try_into().unwrap()))
    }
    fn fixed64(&mut self) -> Result<u64, String> {
        let s = self.b.get(self.i..self.i + 8).ok_or("truncated fixed64")?;
        self.i += 8;
        Ok(u64::from_le_bytes(s.try_into().unwrap()))
    }
    fn skip(&mut self, wire: u64) -> Result<(), String> {
        match wire {
            0 => {
                self.varint()?;
            }
            1 => self.i += 8,
            2 => {
                self.bytes()?;
            }
            5 => self.i += 4,
            w => return Err(format!("unsupported wire type {w}")),
        }
        Ok(())
    }
    fn packed(&mut self) -> Result<Vec<u32>, String> {
        let mut r = Reader::new(self.bytes()?);
        let mut out = Vec::new();
        while !r.done() {
            out.push(r.varint()? as u32);
        }
        Ok(out)
    }
}

fn zigzag(n: u32) -> i32 {
    ((n >> 1) as i32) ^ -((n & 1) as i32)
}

fn decode_value(b: &[u8]) -> Result<Value, String> {
    let mut r = Reader::new(b);
    let mut v = Value::Bool(false);
    while !r.done() {
        let key = r.varint()?;
        match (key >> 3, key & 7) {
            (1, 2) => v = Value::Str(String::from_utf8_lossy(r.bytes()?).into_owned()),
            (2, 5) => v = Value::Num(f32::from_bits(r.fixed32()?) as f64),
            (3, 1) => v = Value::Num(f64::from_bits(r.fixed64()?)),
            (4, 0) => v = Value::Num(r.varint()? as i64 as f64),
            (5, 0) => v = Value::Num(r.varint()? as f64),
            (6, 0) => {
                let n = r.varint()?;
                v = Value::Num((((n >> 1) as i64) ^ -((n & 1) as i64)) as f64)
            }
            (7, 0) => v = Value::Bool(r.varint()? != 0),
            (_, w) => r.skip(w)?,
        }
    }
    Ok(v)
}

fn decode_geometry(cmds: &[u32]) -> Vec<Vec<[f32; 2]>> {
    let mut parts: Vec<Vec<[f32; 2]>> = Vec::new();
    let (mut x, mut y) = (0i32, 0i32);
    let mut i = 0;
    while i < cmds.len() {
        let c = cmds[i];
        i += 1;
        let id = c & 7;
        let count = (c >> 3) as usize;
        match id {
            1 | 2 => {
                for k in 0..count {
                    if i + 1 >= cmds.len() + 1 || i + 1 > cmds.len() {
                        break;
                    }
                    x += zigzag(cmds[i]);
                    y += zigzag(*cmds.get(i + 1).unwrap_or(&0));
                    i += 2;
                    if id == 1 && k == 0 {
                        parts.push(Vec::new());
                    }
                    if let Some(p) = parts.last_mut() {
                        p.push([x as f32, y as f32]);
                    }
                }
            }
            7 => {} // ClosePath: rings are implicitly closed
            _ => break,
        }
    }
    parts
}

pub fn decode(tile: &[u8]) -> Result<Vec<Layer>, String> {
    let mut r = Reader::new(tile);
    let mut layers = Vec::new();
    while !r.done() {
        let key = r.varint()?;
        if key >> 3 == 3 && key & 7 == 2 {
            layers.push(decode_layer(r.bytes()?)?);
        } else {
            r.skip(key & 7)?;
        }
    }
    Ok(layers)
}

fn decode_layer(b: &[u8]) -> Result<Layer, String> {
    let mut r = Reader::new(b);
    let mut name = String::new();
    let mut extent = 4096;
    let mut keys = Vec::new();
    let mut values = Vec::new();
    let mut raw_features: Vec<&[u8]> = Vec::new();
    while !r.done() {
        let key = r.varint()?;
        match (key >> 3, key & 7) {
            (1, 2) => name = String::from_utf8_lossy(r.bytes()?).into_owned(),
            (2, 2) => raw_features.push(r.bytes()?),
            (3, 2) => keys.push(String::from_utf8_lossy(r.bytes()?).into_owned()),
            (4, 2) => values.push(decode_value(r.bytes()?)?),
            (5, 0) => extent = r.varint()? as u32,
            (_, w) => r.skip(w)?,
        }
    }
    let mut features = Vec::with_capacity(raw_features.len());
    for fb in raw_features {
        let mut r = Reader::new(fb);
        let mut f = Feature { id: 0, geom_type: GeomType::Unknown, tags: HashMap::new(), parts: Vec::new() };
        while !r.done() {
            let key = r.varint()?;
            match (key >> 3, key & 7) {
                (1, 0) => f.id = r.varint()?,
                (2, 2) => {
                    let t = r.packed()?;
                    for kv in t.chunks(2) {
                        if let [k, v] = kv {
                            if let (Some(k), Some(v)) = (keys.get(*k as usize), values.get(*v as usize)) {
                                f.tags.insert(k.clone(), v.clone());
                            }
                        }
                    }
                }
                (3, 0) => {
                    f.geom_type = match r.varint()? {
                        1 => GeomType::Point,
                        2 => GeomType::Line,
                        3 => GeomType::Polygon,
                        _ => GeomType::Unknown,
                    }
                }
                (4, 2) => f.parts = decode_geometry(&r.packed()?),
                (_, w) => r.skip(w)?,
            }
        }
        features.push(f);
    }
    Ok(Layer { name, extent, features })
}

/// Signed area of a ring in tile coordinates (y down). Per the MVT spec,
/// exterior rings have positive area with this formula.
pub fn ring_area(r: &[[f32; 2]]) -> f32 {
    let n = r.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (r[i], r[(i + 1) % n]);
        a += p[0] * q[1] - q[0] * p[1];
    }
    a * 0.5
}

/// Group polygon rings into (exterior, holes) per the MVT winding rule.
pub fn polygons(parts: &[Vec<[f32; 2]>]) -> Vec<(Vec<[f32; 2]>, Vec<Vec<[f32; 2]>>)> {
    let mut out: Vec<(Vec<[f32; 2]>, Vec<Vec<[f32; 2]>>)> = Vec::new();
    for ring in parts {
        if ring.len() < 3 {
            continue;
        }
        let a = ring_area(ring);
        if a > 0.0 {
            out.push((ring.clone(), Vec::new()));
        } else if a < 0.0 {
            if let Some(last) = out.last_mut() {
                last.1.push(ring.clone());
            }
        }
    }
    out
}
