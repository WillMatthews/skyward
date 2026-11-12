//! A small binary codec for handing finished tiles between wasm instances
//! (the browser build builds tiles in Web Workers, each its own instance
//! with its own memory, and posts the result to the page as bytes).
//!
//! Little-endian, no schema: writer and reader are the same build. Bulk
//! plain-old-data arrays (vertices, indices, instances) go as one copy.

use glam::{Vec2, Vec3};
use std::sync::Arc;

#[derive(Default)]
pub struct W(pub Vec<u8>);

pub struct R<'a> {
    b: &'a [u8],
    i: usize,
}

pub type Res<T> = Result<T, String>;

impl<'a> R<'a> {
    pub fn new(b: &'a [u8]) -> R<'a> {
        R { b, i: 0 }
    }
    pub fn take(&mut self, n: usize) -> Res<&'a [u8]> {
        if self.b.len() - self.i < n {
            return Err("wire: truncated".into());
        }
        let s = &self.b[self.i..self.i + n];
        self.i += n;
        Ok(s)
    }
    pub fn done(&self) -> bool {
        self.i == self.b.len()
    }
}

pub trait Wire: Sized {
    fn put(&self, w: &mut W);
    fn get(r: &mut R) -> Res<Self>;
}

/// Encode a value to bytes.
pub fn encode<T: Wire>(v: &T) -> Vec<u8> {
    let mut w = W::default();
    v.put(&mut w);
    w.0
}

/// Decode bytes made by [`encode`].
pub fn decode<T: Wire>(b: &[u8]) -> Res<T> {
    let mut r = R::new(b);
    let v = T::get(&mut r)?;
    if !r.done() {
        return Err("wire: trailing bytes".into());
    }
    Ok(v)
}

/// A slice of plain-old-data values as one copy.
pub fn put_pod<T: bytemuck::Pod>(w: &mut W, v: &[T]) {
    (v.len() as u64).put(w);
    w.0.extend_from_slice(bytemuck::cast_slice(v));
}

pub fn get_pod<T: bytemuck::Pod>(r: &mut R) -> Res<Vec<T>> {
    let n = u64::get(r)? as usize;
    let bytes = r.take(n.checked_mul(std::mem::size_of::<T>()).ok_or("wire: size")?)?;
    let mut v = vec![T::zeroed(); n];
    bytemuck::cast_slice_mut::<T, u8>(&mut v).copy_from_slice(bytes);
    Ok(v)
}

macro_rules! num {
    ($($t:ty),*) => {$(
        impl Wire for $t {
            fn put(&self, w: &mut W) {
                w.0.extend_from_slice(&self.to_le_bytes());
            }
            fn get(r: &mut R) -> Res<Self> {
                Ok(<$t>::from_le_bytes(r.take(std::mem::size_of::<$t>())?.try_into().unwrap()))
            }
        }
    )*};
}
num!(u8, u16, u32, u64, i32, f32, f64);

impl Wire for usize {
    fn put(&self, w: &mut W) {
        (*self as u64).put(w)
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(u64::get(r)? as usize)
    }
}

impl Wire for bool {
    fn put(&self, w: &mut W) {
        (*self as u8).put(w)
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(u8::get(r)? != 0)
    }
}

impl Wire for String {
    fn put(&self, w: &mut W) {
        put_pod(w, self.as_bytes())
    }
    fn get(r: &mut R) -> Res<Self> {
        String::from_utf8(get_pod::<u8>(r)?).map_err(|e| e.to_string())
    }
}

impl Wire for Vec2 {
    fn put(&self, w: &mut W) {
        self.to_array().put(w)
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(Vec2::from_array(<[f32; 2]>::get(r)?))
    }
}

impl Wire for Vec3 {
    fn put(&self, w: &mut W) {
        self.to_array().put(w)
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(Vec3::from_array(<[f32; 3]>::get(r)?))
    }
}

impl<const N: usize> Wire for [f32; N] {
    fn put(&self, w: &mut W) {
        w.0.extend_from_slice(bytemuck::cast_slice(self));
    }
    fn get(r: &mut R) -> Res<Self> {
        let mut a = [0f32; N];
        bytemuck::cast_slice_mut::<f32, u8>(&mut a).copy_from_slice(r.take(4 * N)?);
        Ok(a)
    }
}

impl<T: Wire> Wire for Vec<T> {
    fn put(&self, w: &mut W) {
        self.len().put(w);
        for x in self {
            x.put(w);
        }
    }
    fn get(r: &mut R) -> Res<Self> {
        let n = usize::get(r)?;
        let mut v = Vec::with_capacity(n.min(1 << 20));
        for _ in 0..n {
            v.push(T::get(r)?);
        }
        Ok(v)
    }
}

impl<T: Wire> Wire for Option<T> {
    fn put(&self, w: &mut W) {
        match self {
            Some(x) => {
                1u8.put(w);
                x.put(w);
            }
            None => 0u8.put(w),
        }
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(if u8::get(r)? != 0 { Some(T::get(r)?) } else { None })
    }
}

impl<T: Wire> Wire for Arc<T> {
    fn put(&self, w: &mut W) {
        (**self).put(w)
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(Arc::new(T::get(r)?))
    }
}

impl<A: Wire, B: Wire> Wire for (A, B) {
    fn put(&self, w: &mut W) {
        self.0.put(w);
        self.1.put(w);
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok((A::get(r)?, B::get(r)?))
    }
}

impl<T: Wire, E: Wire> Wire for Result<T, E> {
    fn put(&self, w: &mut W) {
        match self {
            Ok(x) => {
                1u8.put(w);
                x.put(w);
            }
            Err(e) => {
                0u8.put(w);
                e.put(w);
            }
        }
    }
    fn get(r: &mut R) -> Res<Self> {
        Ok(if u8::get(r)? != 0 { Ok(T::get(r)?) } else { Err(E::get(r)?) })
    }
}

/// Wire for a fieldwise struct: `wire_struct!(Type { a, b, c });`
#[macro_export]
macro_rules! wire_struct {
    ($t:ty { $($f:ident),* $(,)? }) => {
        impl $crate::platform::wire::Wire for $t {
            fn put(&self, w: &mut $crate::platform::wire::W) {
                $( $crate::platform::wire::Wire::put(&self.$f, w); )*
            }
            fn get(r: &mut $crate::platform::wire::R) -> $crate::platform::wire::Res<Self> {
                Ok(Self { $( $f: $crate::platform::wire::Wire::get(r)?, )* })
            }
        }
    };
}

/// Wire for a fieldless enum listed in order: `wire_enum!(Kind, [Kind::A, Kind::B]);`
#[macro_export]
macro_rules! wire_enum {
    ($t:ty, $all:expr) => {
        impl $crate::platform::wire::Wire for $t {
            fn put(&self, w: &mut $crate::platform::wire::W) {
                let i = $all.iter().position(|x| x == self).unwrap_or(0) as u8;
                $crate::platform::wire::Wire::put(&i, w);
            }
            fn get(r: &mut $crate::platform::wire::R) -> $crate::platform::wire::Res<Self> {
                let i = <u8 as $crate::platform::wire::Wire>::get(r)? as usize;
                $all.get(i).copied().ok_or_else(|| "wire: bad enum".to_string())
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::{Area, Kind, Line, Proj, TileFeatures, TileId};
    use glam::vec2;

    /// A tile with buildings, a park, water, a road and a runway survives
    /// encode → decode bit for bit (what the browser's tile workers post).
    #[test]
    fn tile_wire_roundtrip() {
        let id = TileId { z: 14, x: 8192, y: 5448 };
        let proj = Proj::tile_frame(id);
        let (lo, hi) = id.rect(&proj);
        let c = (lo + hi) * 0.5;
        let quad = |o: Vec2, s: f32| vec![o + vec2(-s, -s), o + vec2(s, -s), o + vec2(s, s), o + vec2(-s, s)];
        let area = |kind: Kind, outer: Vec<Vec2>, key: u64| Area { border: vec![false; outer.len()], kind, outer, holes: Vec::new(), hole_border: Vec::new(), key };
        let mut f = TileFeatures::default();
        for i in 0..6 {
            f.areas.push(area(Kind::Building { height: 12.0 + i as f32, min_height: 0.0, colour: None }, quad(c + vec2(i as f32 * 40.0, 60.0), 12.0), 10 + i));
        }
        f.areas.push(area(Kind::Park, quad(c - vec2(200.0, 200.0), 120.0), 1));
        f.areas.push(area(Kind::Water, quad(c + vec2(250.0, -250.0), 90.0), 2));
        f.lines.push(Line { kind: Kind::Road { width: 8.0, major: true }, pts: vec![lo + vec2(10.0, 300.0), hi - vec2(10.0, 300.0)], cut: [false; 2] });
        f.runways.push(crate::osm::RunwayPiece { a: c - vec2(400.0, 0.0), b: c + vec2(400.0, 0.0), name: "09/27".into() });
        let b = crate::scene::build_tile(id, &proj, f);
        assert!(!b.flat.verts.is_empty() && !b.buildings.verts.is_empty());
        let bytes = encode(&b);
        let b2: crate::scene::TileBuild = decode(&bytes).unwrap();
        assert_eq!(encode(&b2), bytes);
        assert_eq!(b2.buildings.idx, b.buildings.idx);
        // A tile result as the workers send it.
        let m = crate::ui::mapdata::MapVectors::default();
        let msg: (Result<crate::scene::TileBuild, String>, Option<Arc<crate::ui::mapdata::MapVectors>>) = (Ok(b2), Some(Arc::new(m)));
        let bytes = encode(&msg);
        let back: (Result<crate::scene::TileBuild, String>, Option<Arc<crate::ui::mapdata::MapVectors>>) = decode(&bytes).unwrap();
        assert_eq!(encode(&back), bytes);
        assert!(decode::<crate::scene::TileBuild>(&bytes[..bytes.len() / 2]).is_err());
    }
}
