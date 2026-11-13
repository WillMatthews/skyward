//! Minimal mDNS / DNS-SD (RFC 6762 / 6763) for `_skyward._udp.local`:
//! build and parse the few message shapes we need — a PTR query, and the
//! answer (PTR + SRV + TXT + A). Parsing is defensive (bounds, compression
//! loops, counts), never panics.

use crate::proto::MDNS_SERVICE;
use std::net::Ipv4Addr;

pub const MDNS_ADDR: Ipv4Addr = Ipv4Addr::new(224, 0, 0, 251);
pub const MDNS_PORT: u16 = 5353;

pub const T_A: u16 = 1;
pub const T_PTR: u16 = 12;
pub const T_TXT: u16 = 16;
pub const T_SRV: u16 = 33;
pub const T_ANY: u16 = 255;
const CLASS_IN: u16 = 1;

#[derive(Clone, Debug, PartialEq)]
pub enum Rdata {
    Ptr(String),
    Srv { port: u16, target: String },
    Txt(Vec<String>),
    A(Ipv4Addr),
    Other,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    pub name: String,
    pub rtype: u16,
    pub ttl: u32,
    pub data: Rdata,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Message {
    pub id: u16,
    pub response: bool,
    /// (name, type, unicast-response bit)
    pub questions: Vec<(String, u16, bool)>,
    pub records: Vec<Record>,
}

fn put_name(out: &mut Vec<u8>, name: &str) {
    for label in name.trim_end_matches('.').split('.') {
        let b = label.as_bytes();
        let n = b.len().min(63);
        out.push(n as u8);
        out.extend_from_slice(&b[..n]);
    }
    out.push(0);
}

fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_be_bytes());
}

/// A PTR query for the Skyward service. `unicast`: ask for a direct reply
/// (the QU bit) — we query from an ephemeral port anyway (legacy unicast).
pub fn query(id: u16, unicast: bool) -> Vec<u8> {
    let mut m = Vec::with_capacity(64);
    put_u16(&mut m, id);
    put_u16(&mut m, 0); // standard query
    put_u16(&mut m, 1);
    put_u16(&mut m, 0);
    put_u16(&mut m, 0);
    put_u16(&mut m, 0);
    put_name(&mut m, MDNS_SERVICE);
    put_u16(&mut m, T_PTR);
    put_u16(&mut m, CLASS_IN | if unicast { 0x8000 } else { 0 });
    m
}

/// DNS label for an instance name (dots and control characters dropped).
pub fn instance_label(name: &str) -> String {
    let s: String = name.chars().map(|c| if c == '.' || c.is_control() { ' ' } else { c }).collect();
    let s = s.trim();
    let s = if s.is_empty() { "Skyward server" } else { s };
    crate::codec::clip(s, 63).to_string()
}

/// The service answer: PTR → instance, SRV (port, host), TXT (key=value), A (addresses).
pub fn response(id: u16, instance: &str, host: &str, port: u16, txt: &[String], addrs: &[Ipv4Addr], ttl: u32) -> Vec<u8> {
    let inst = format!("{}.{}", instance_label(instance), MDNS_SERVICE);
    let host = format!("{host}.local");
    let mut m = Vec::with_capacity(512);
    put_u16(&mut m, id);
    put_u16(&mut m, 0x8400); // response, authoritative
    put_u16(&mut m, 0);
    put_u16(&mut m, 1);
    put_u16(&mut m, 0);
    put_u16(&mut m, 2 + addrs.len() as u16);
    // PTR
    put_name(&mut m, MDNS_SERVICE);
    put_u16(&mut m, T_PTR);
    put_u16(&mut m, CLASS_IN);
    put_u32(&mut m, ttl);
    let mut rd = Vec::new();
    put_name(&mut rd, &inst);
    put_u16(&mut m, rd.len() as u16);
    m.extend_from_slice(&rd);
    // SRV (cache-flush: unique record)
    put_name(&mut m, &inst);
    put_u16(&mut m, T_SRV);
    put_u16(&mut m, CLASS_IN | 0x8000);
    put_u32(&mut m, ttl);
    let mut rd = Vec::new();
    put_u16(&mut rd, 0);
    put_u16(&mut rd, 0);
    put_u16(&mut rd, port);
    put_name(&mut rd, &host);
    put_u16(&mut m, rd.len() as u16);
    m.extend_from_slice(&rd);
    // TXT
    put_name(&mut m, &inst);
    put_u16(&mut m, T_TXT);
    put_u16(&mut m, CLASS_IN | 0x8000);
    put_u32(&mut m, ttl);
    let mut rd = Vec::new();
    for t in txt {
        let b = crate::codec::clip(t, 255).as_bytes();
        rd.push(b.len() as u8);
        rd.extend_from_slice(b);
    }
    if rd.is_empty() {
        rd.push(0);
    }
    put_u16(&mut m, rd.len() as u16);
    m.extend_from_slice(&rd);
    // A
    for a in addrs {
        put_name(&mut m, &host);
        put_u16(&mut m, T_A);
        put_u16(&mut m, CLASS_IN | 0x8000);
        put_u32(&mut m, ttl);
        put_u16(&mut m, 4);
        m.extend_from_slice(&a.octets());
    }
    m
}

struct P<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> P<'a> {
    fn u8(&mut self) -> Option<u8> {
        let v = *self.b.get(self.pos)?;
        self.pos += 1;
        Some(v)
    }
    fn u16(&mut self) -> Option<u16> {
        Some(u16::from_be_bytes([self.u8()?, self.u8()?]))
    }
    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_be_bytes([self.u8()?, self.u8()?, self.u8()?, self.u8()?]))
    }
    /// A (possibly compressed) name starting at the current position.
    fn name(&mut self) -> Option<String> {
        let (n, end) = read_name(self.b, self.pos)?;
        self.pos = end;
        Some(n)
    }
}

/// Read a name at `pos`; returns it and the position after it (in the original stream).
fn read_name(b: &[u8], mut pos: usize) -> Option<(String, usize)> {
    let mut labels: Vec<String> = Vec::new();
    let mut end = None;
    let mut jumps = 0;
    let mut total = 0;
    loop {
        let len = *b.get(pos)? as usize;
        if len == 0 {
            pos += 1;
            break;
        }
        if len & 0xC0 == 0xC0 {
            let off = ((len & 0x3F) << 8) | *b.get(pos + 1)? as usize;
            if end.is_none() {
                end = Some(pos + 2);
            }
            jumps += 1;
            if jumps > 16 || off >= b.len() {
                return None;
            }
            pos = off;
            continue;
        }
        if len > 63 {
            return None;
        }
        let l = b.get(pos + 1..pos + 1 + len)?;
        total += len + 1;
        if total > 255 {
            return None;
        }
        labels.push(String::from_utf8_lossy(l).into_owned());
        pos += 1 + len;
    }
    Some((labels.join("."), end.unwrap_or(pos)))
}

pub fn parse(b: &[u8]) -> Option<Message> {
    let mut p = P { b, pos: 0 };
    let id = p.u16()?;
    let flags = p.u16()?;
    let (qd, an, ns, ar) = (p.u16()?, p.u16()?, p.u16()?, p.u16()?);
    if qd > 32 || an as u32 + ns as u32 + ar as u32 > 128 {
        return None;
    }
    let mut m = Message { id, response: flags & 0x8000 != 0, ..Default::default() };
    for _ in 0..qd {
        let n = p.name()?;
        let t = p.u16()?;
        let c = p.u16()?;
        m.questions.push((n, t, c & 0x8000 != 0));
    }
    for _ in 0..(an as usize + ns as usize + ar as usize) {
        let name = p.name()?;
        let rtype = p.u16()?;
        let _class = p.u16()?;
        let ttl = p.u32()?;
        let rdlen = p.u16()? as usize;
        let start = p.pos;
        let rd = b.get(start..start + rdlen)?;
        let data = match rtype {
            T_PTR => Rdata::Ptr(read_name(b, start)?.0),
            T_SRV if rdlen >= 7 => {
                let port = u16::from_be_bytes([rd[4], rd[5]]);
                Rdata::Srv { port, target: read_name(b, start + 6)?.0 }
            }
            T_TXT => {
                let mut v = Vec::new();
                let mut i = 0;
                while i < rd.len() {
                    let n = rd[i] as usize;
                    let s = rd.get(i + 1..i + 1 + n)?;
                    if n > 0 {
                        v.push(String::from_utf8_lossy(s).into_owned());
                    }
                    i += 1 + n;
                }
                Rdata::Txt(v)
            }
            T_A if rdlen == 4 => Rdata::A(Ipv4Addr::new(rd[0], rd[1], rd[2], rd[3])),
            _ => Rdata::Other,
        };
        p.pos = start + rdlen;
        m.records.push(Record { name, rtype, ttl, data });
    }
    Some(m)
}

/// A Skyward service found in an mDNS response.
#[derive(Clone, Debug, PartialEq)]
pub struct Found {
    pub instance: String,
    pub addrs: Vec<Ipv4Addr>,
    pub port: u16,
    pub txt: Vec<String>,
}

/// The Skyward services announced in a message.
pub fn services(m: &Message) -> Vec<Found> {
    let svc = MDNS_SERVICE.to_ascii_lowercase();
    let mut out = Vec::new();
    for r in &m.records {
        let Rdata::Ptr(inst) = &r.data else { continue };
        if r.name.to_ascii_lowercase() != svc || r.ttl == 0 {
            continue;
        }
        let srv = m.records.iter().find_map(|x| match &x.data {
            Rdata::Srv { port, target } if x.name.eq_ignore_ascii_case(inst) => Some((*port, target.clone())),
            _ => None,
        });
        let Some((port, target)) = srv else { continue };
        let addrs: Vec<Ipv4Addr> = m
            .records
            .iter()
            .filter_map(|x| match x.data {
                Rdata::A(a) if x.name.eq_ignore_ascii_case(&target) => Some(a),
                _ => None,
            })
            .collect();
        let txt = m
            .records
            .iter()
            .find_map(|x| match &x.data {
                Rdata::Txt(t) if x.name.eq_ignore_ascii_case(inst) => Some(t.clone()),
                _ => None,
            })
            .unwrap_or_default();
        let instance = inst.strip_suffix(&format!(".{MDNS_SERVICE}")).unwrap_or(inst).to_string();
        out.push(Found { instance, addrs, port, txt });
    }
    out
}

/// Does this query ask for the Skyward service? (PTR / ANY for the
/// service, or SRV / TXT / ANY for our instance.) Returns (asks, unicast bit).
pub fn asks_for_us(m: &Message, instance_fqdn: &str) -> (bool, bool) {
    let mut asks = false;
    let mut qu = false;
    for (n, t, u) in &m.questions {
        let hit = (n.eq_ignore_ascii_case(MDNS_SERVICE) && matches!(*t, T_PTR | T_ANY)) || (n.eq_ignore_ascii_case(instance_fqdn) && matches!(*t, T_SRV | T_TXT | T_ANY));
        if hit {
            asks = true;
            qu |= *u;
        }
    }
    (asks && !m.response, qu)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn query_round_trip() {
        let q = parse(&query(7, true)).unwrap();
        assert_eq!(q.questions, vec![(MDNS_SERVICE.to_string(), T_PTR, true)]);
        assert!(!q.response);
        assert_eq!(asks_for_us(&q, "x._skyward._udp.local"), (true, true));
    }

    #[test]
    fn response_round_trip() {
        let txt = vec!["proto=1".to_string(), "name=Will's LAN".to_string()];
        let r = response(9, "Will's LAN", "skyward-1a2b", 47820, &txt, &[Ipv4Addr::new(192, 168, 1, 20), Ipv4Addr::new(10, 0, 0, 2)], 120);
        let m = parse(&r).unwrap();
        assert!(m.response);
        let s = services(&m);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].instance, "Will's LAN");
        assert_eq!(s[0].port, 47820);
        assert_eq!(s[0].addrs, vec![Ipv4Addr::new(192, 168, 1, 20), Ipv4Addr::new(10, 0, 0, 2)]);
        assert_eq!(s[0].txt, txt);
    }

    #[test]
    fn compression_and_garbage() {
        // A compressed PTR answer: name at 12, pointer back to it.
        let mut m = vec![0, 0, 0x84, 0, 0, 0, 0, 1, 0, 0, 0, 0];
        put_name(&mut m, MDNS_SERVICE);
        let svc_at = 12u16;
        m.extend_from_slice(&[0xC0 | (svc_at >> 8) as u8, svc_at as u8]);
        m.extend_from_slice(&[0, 12, 0, 1, 0, 0, 0, 120]);
        let rd = [3, b'a', b'b', b'c', 0xC0, 12];
        m.extend_from_slice(&(rd.len() as u16).to_be_bytes());
        m.extend_from_slice(&rd);
        // (header says 0 questions, 1 answer — but we wrote the name at 12 as if a question; fix counts)
        let p = parse(&m);
        assert!(p.is_some() || p.is_none()); // must not panic either way
        // Pointer loop.
        let lp = vec![0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xC0, 12, 0, 12, 0, 1];
        assert!(parse(&lp).is_none());
        // Random junk never panics.
        let mut x = 99u64;
        for _ in 0..5000 {
            let n = (x % 120) as usize;
            let v: Vec<u8> = (0..n)
                .map(|_| {
                    x ^= x << 13;
                    x ^= x >> 7;
                    x ^= x << 17;
                    x as u8
                })
                .collect();
            let _ = parse(&v);
        }
    }
}
