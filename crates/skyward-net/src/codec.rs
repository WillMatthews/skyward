//! Little-endian byte writer / bounds-checked reader. Every read returns a
//! [`DecodeError`] instead of panicking, whatever the input.

use std::fmt;

/// Why a packet could not be decoded (the packet is dropped).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Ran past the end of the buffer.
    Truncated,
    /// Not a Skyward packet (bad magic).
    Magic,
    /// Unknown packet / message kind.
    Kind(u8),
    /// A field had an invalid value (enum tag, length over its cap, NaN...).
    Invalid(&'static str),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Truncated => write!(f, "truncated packet"),
            DecodeError::Magic => write!(f, "not a Skyward packet"),
            DecodeError::Kind(k) => write!(f, "unknown kind {k}"),
            DecodeError::Invalid(w) => write!(f, "invalid {w}"),
        }
    }
}

impl std::error::Error for DecodeError {}

pub type Result<T> = std::result::Result<T, DecodeError>;

#[derive(Default, Clone, Debug)]
pub struct Writer {
    pub buf: Vec<u8>,
}

impl Writer {
    pub fn new() -> Writer {
        Writer { buf: Vec::with_capacity(256) }
    }
    pub fn len(&self) -> usize {
        self.buf.len()
    }
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    pub fn i8(&mut self, v: i8) {
        self.buf.push(v as u8);
    }
    pub fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn i16(&mut self, v: i16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u64(&mut self, v: u64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn f32(&mut self, v: f32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn f64(&mut self, v: f64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn bytes(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }
    pub fn bool(&mut self, b: bool) {
        self.buf.push(b as u8);
    }
    /// A string of at most `max` bytes (cut at a char boundary), u16 length prefix.
    pub fn str(&mut self, s: &str, max: usize) {
        let s = clip(s, max);
        self.u16(s.len() as u16);
        self.buf.extend_from_slice(s.as_bytes());
    }
    /// Pad with zeros up to `n` bytes in total.
    pub fn pad_to(&mut self, n: usize) {
        if self.buf.len() < n {
            self.buf.resize(n, 0);
        }
    }
    /// Value in -1..1 as an i8 (1/127 steps).
    pub fn snorm8(&mut self, v: f32) {
        self.i8((v.clamp(-1.0, 1.0) * 127.0).round() as i8);
    }
    /// Value in 0..1 as a u8 (1/255 steps).
    pub fn unorm8(&mut self, v: f32) {
        self.u8((v.clamp(0.0, 1.0) * 255.0).round() as u8);
    }
}

/// Cut `s` to at most `max` bytes on a char boundary.
pub fn clip(s: &str, max: usize) -> &str {
    if s.len() <= max {
        return s;
    }
    let mut end = max;
    while !s.is_char_boundary(end) {
        end -= 1;
    }
    &s[..end]
}

pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Reader<'a> {
        Reader { buf, pos: 0 }
    }
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }
    pub fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        if self.remaining() < n {
            return Err(DecodeError::Truncated);
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }
    fn arr<const N: usize>(&mut self) -> Result<[u8; N]> {
        let s = self.take(N)?;
        let mut a = [0u8; N];
        a.copy_from_slice(s);
        Ok(a)
    }
    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    pub fn i8(&mut self) -> Result<i8> {
        Ok(self.u8()? as i8)
    }
    pub fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.arr()?))
    }
    pub fn i16(&mut self) -> Result<i16> {
        Ok(i16::from_le_bytes(self.arr()?))
    }
    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.arr()?))
    }
    pub fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.arr()?))
    }
    /// A finite f32 (NaN / infinity are rejected).
    pub fn f32(&mut self) -> Result<f32> {
        let v = f32::from_le_bytes(self.arr()?);
        if v.is_finite() { Ok(v) } else { Err(DecodeError::Invalid("float")) }
    }
    /// A finite f64.
    pub fn f64(&mut self) -> Result<f64> {
        let v = f64::from_le_bytes(self.arr()?);
        if v.is_finite() { Ok(v) } else { Err(DecodeError::Invalid("float")) }
    }
    pub fn bool(&mut self) -> Result<bool> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(DecodeError::Invalid("bool")),
        }
    }
    /// A UTF-8 string of at most `max` bytes.
    pub fn str(&mut self, max: usize) -> Result<String> {
        let n = self.u16()? as usize;
        if n > max {
            return Err(DecodeError::Invalid("string length"));
        }
        let b = self.take(n)?;
        std::str::from_utf8(b).map(|s| s.to_string()).map_err(|_| DecodeError::Invalid("utf-8"))
    }
    pub fn snorm8(&mut self) -> Result<f32> {
        Ok((self.i8()? as f32 / 127.0).clamp(-1.0, 1.0))
    }
    pub fn unorm8(&mut self) -> Result<f32> {
        Ok(self.u8()? as f32 / 255.0)
    }
}

/// Printable, trimmed, length-capped text (names, chat): control characters
/// and line breaks become spaces, runs of spaces collapse.
pub fn sanitize(s: &str, max_chars: usize) -> String {
    let mut out = String::with_capacity(s.len().min(max_chars * 4));
    let mut n = 0;
    let mut space = false;
    for c in s.chars() {
        let c = if c.is_control() || c.is_whitespace() { ' ' } else { c };
        // (Unicode bidi overrides and zero-width characters would let a name
        // impersonate another or hide text.)
        if matches!(c, '\u{200B}'..='\u{200F}' | '\u{202A}'..='\u{202E}' | '\u{2066}'..='\u{2069}' | '\u{FEFF}') {
            continue;
        }
        if c == ' ' {
            if space || out.is_empty() {
                continue;
            }
            space = true;
        } else {
            space = false;
        }
        if n >= max_chars {
            break;
        }
        out.push(c);
        n += 1;
    }
    out.trim_end().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_scalars() {
        let mut w = Writer::new();
        w.u8(7);
        w.i16(-1234);
        w.u32(0xDEADBEEF);
        w.u64(u64::MAX - 3);
        w.f32(1.5);
        w.f64(-2.25e10);
        w.str("héllo", 64);
        w.snorm8(-0.5);
        w.unorm8(0.25);
        let mut r = Reader::new(&w.buf);
        assert_eq!(r.u8().unwrap(), 7);
        assert_eq!(r.i16().unwrap(), -1234);
        assert_eq!(r.u32().unwrap(), 0xDEADBEEF);
        assert_eq!(r.u64().unwrap(), u64::MAX - 3);
        assert_eq!(r.f32().unwrap(), 1.5);
        assert_eq!(r.f64().unwrap(), -2.25e10);
        assert_eq!(r.str(64).unwrap(), "héllo");
        assert!((r.snorm8().unwrap() + 0.5).abs() < 0.01);
        assert!((r.unorm8().unwrap() - 0.25).abs() < 0.01);
        assert_eq!(r.u8(), Err(DecodeError::Truncated));
    }

    #[test]
    fn rejects_bad_input() {
        let mut r = Reader::new(&[0xff, 0xff, b'a']);
        assert!(r.str(10).is_err());
        let nan = f32::NAN.to_le_bytes();
        assert!(Reader::new(&nan).f32().is_err());
        let bad_utf8 = [2, 0, 0xc3, 0x28];
        assert_eq!(Reader::new(&bad_utf8).str(10), Err(DecodeError::Invalid("utf-8")));
    }

    #[test]
    fn clip_and_sanitize() {
        assert_eq!(clip("héllo", 2), "h");
        assert_eq!(sanitize("  Ali\nce \t\u{202E}gnp ", 32), "Ali ce gnp");
        assert_eq!(sanitize("abcdef", 3), "abc");
    }
}
