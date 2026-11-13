//! A small reliability layer over UDP datagrams (sans-IO).
//!
//! * Every `Data` packet has a 16-bit sequence number and acknowledges the
//!   peer's latest packet plus the 32 before it (bit field), so acks are
//!   redundant and survive loss without extra traffic.
//! * Unreliable messages (aircraft states, snapshots, pings) are sent once:
//!   the newest state always supersedes a lost one.
//! * Reliable messages (chat, joins / leaves, events, world sync) carry a
//!   16-bit message id, ride along in packets until a packet that carried
//!   them is acknowledged (resent every ~RTT), and are delivered in order,
//!   exactly once.
//! * RTT and loss are estimated from the acks.

use crate::codec::{Reader, Writer};
use crate::proto::{AckHeader, MAX_PACKET, Msg, msg};
use std::collections::{HashMap, VecDeque};

/// Bytes before the payload in a `Data` packet (magic, kind, token, ack header).
pub const DATA_HEADER: usize = 4 + 1 + 8 + 8;
/// Room for messages in one packet.
pub const MAX_PAYLOAD: usize = MAX_PACKET - DATA_HEADER;
/// Unacknowledged reliable messages before the connection is declared broken.
pub const MAX_PENDING: usize = 2048;
/// Out-of-order reliable messages held for in-order delivery.
const REORDER_WINDOW: u16 = 512;
/// Sent-packet records kept for acknowledgement.
const SENT_WINDOW: usize = 1024;

/// `a` is newer than `b` (with wrap-around).
pub fn seq_gt(a: u16, b: u16) -> bool {
    a != b && a.wrapping_sub(b) < 0x8000
}

struct Pending {
    rid: u16,
    t: u8,
    body: Vec<u8>,
    last_sent: f64,
    sends: u32,
}

struct SentRec {
    t: f64,
    rids: Vec<u16>,
    acked: bool,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct ChannelStats {
    pub packets_sent: u64,
    pub packets_received: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub resends: u64,
    /// Duplicate or too-old packets dropped.
    pub dropped: u64,
}

pub struct Channel {
    next_seq: u16,
    sent: HashMap<u16, SentRec>,
    sent_order: VecDeque<u16>,
    next_rid: u16,
    pending: VecDeque<Pending>,
    // receive side
    any_received: bool,
    recv_seq: u16,
    recv_bits: u32,
    next_deliver: u16,
    reorder: HashMap<u16, Msg>,
    // estimates
    pub srtt: f64,
    pub rttvar: f64,
    rtt_init: bool,
    /// Recent sent packets (time, acked) for the loss estimate.
    loss_hist: VecDeque<(f64, u16)>,
    pub stats: ChannelStats,
}

impl Default for Channel {
    fn default() -> Self {
        Channel::new()
    }
}

impl Channel {
    pub fn new() -> Channel {
        Channel {
            next_seq: 1,
            sent: HashMap::new(),
            sent_order: VecDeque::new(),
            next_rid: 0,
            pending: VecDeque::new(),
            any_received: false,
            recv_seq: 0,
            recv_bits: 0,
            next_deliver: 0,
            reorder: HashMap::new(),
            srtt: 0.1,
            rttvar: 0.05,
            rtt_init: false,
            loss_hist: VecDeque::new(),
            stats: ChannelStats::default(),
        }
    }

    /// Queue a reliable message (unreliable ones go straight to [`Channel::send`]).
    pub fn queue_reliable(&mut self, m: &Msg) {
        debug_assert!(m.reliable());
        let rid = self.next_rid;
        self.next_rid = self.next_rid.wrapping_add(1);
        self.pending.push_back(Pending { rid, t: m.type_id(), body: m.encode_body(), last_sent: f64::NEG_INFINITY, sends: 0 });
    }

    /// Too many reliable messages unacknowledged: the peer is gone or flooded.
    pub fn overflowed(&self) -> bool {
        self.pending.len() > MAX_PENDING
    }

    pub fn pending_reliable(&self) -> usize {
        self.pending.len()
    }

    /// Resend interval for reliable messages.
    fn rto(&self) -> f64 {
        (self.srtt + 2.0 * self.rttvar + 0.01).clamp(0.04, 1.0)
    }

    /// Build this tick's packets: due reliable messages first, then the
    /// unreliable ones (dropped if they no longer fit, newest first is the
    /// caller's order). `keepalive`: send an (ack-only) packet even if empty.
    /// Returns (header, payload) per packet, at most `max_packets`.
    pub fn send(&mut self, now: f64, unreliable: &[Msg], keepalive: bool, max_packets: usize) -> Vec<(AckHeader, Vec<u8>)> {
        let rto = self.rto();
        let mut packets: Vec<(Vec<u8>, Vec<u16>)> = Vec::new();
        let mut cur = Writer::new();
        let mut cur_rids = Vec::new();
        // Reliable: due ones, oldest first.
        for p in self.pending.iter_mut() {
            if now - p.last_sent < rto {
                continue;
            }
            let need = 1 + 2 + 2 + p.body.len();
            if need > MAX_PAYLOAD {
                continue; // (cannot happen: bodies are bounded)
            }
            if cur.len() + need > MAX_PAYLOAD {
                if packets.len() + 1 >= max_packets {
                    break;
                }
                packets.push((std::mem::take(&mut cur.buf), std::mem::take(&mut cur_rids)));
            }
            cur.u8(p.t);
            cur.u16(p.rid);
            cur.u16(p.body.len() as u16);
            cur.bytes(&p.body);
            cur_rids.push(p.rid);
            if p.sends > 0 {
                self.stats.resends += 1;
            }
            p.sends += 1;
            p.last_sent = now;
        }
        for m in unreliable {
            let body = m.encode_body();
            let need = 1 + 2 + body.len();
            if need > MAX_PAYLOAD {
                continue;
            }
            if cur.len() + need > MAX_PAYLOAD {
                if packets.len() + 1 >= max_packets {
                    break;
                }
                packets.push((std::mem::take(&mut cur.buf), std::mem::take(&mut cur_rids)));
            }
            cur.u8(m.type_id());
            cur.u16(body.len() as u16);
            cur.bytes(&body);
        }
        if !cur.is_empty() || (packets.is_empty() && keepalive) {
            packets.push((cur.buf, cur_rids));
        }
        let ack = self.recv_seq;
        let bits = self.recv_bits;
        packets
            .into_iter()
            .map(|(payload, rids)| {
                // (Sequence 0 is never used: an ack of 0 means "nothing received yet".)
                let seq = self.next_seq;
                self.next_seq = match self.next_seq.wrapping_add(1) {
                    0 => 1,
                    n => n,
                };
                self.sent.insert(seq, SentRec { t: now, rids, acked: false });
                self.sent_order.push_back(seq);
                while self.sent_order.len() > SENT_WINDOW {
                    if let Some(old) = self.sent_order.pop_front() {
                        self.sent.remove(&old);
                    }
                }
                self.loss_hist.push_back((now, seq));
                if self.loss_hist.len() > 256 {
                    self.loss_hist.pop_front();
                }
                self.stats.packets_sent += 1;
                self.stats.bytes_sent += (DATA_HEADER + payload.len()) as u64;
                (AckHeader { seq, ack, ack_bits: bits }, payload)
            })
            .collect()
    }

    fn on_acked(&mut self, seq: u16, now: f64, direct: bool) {
        let Some(rec) = self.sent.get_mut(&seq) else { return };
        if rec.acked {
            return;
        }
        rec.acked = true;
        if direct {
            let sample = (now - rec.t).max(0.0);
            if !self.rtt_init {
                self.srtt = sample;
                self.rttvar = sample * 0.5;
                self.rtt_init = true;
            } else {
                self.rttvar = 0.75 * self.rttvar + 0.25 * (self.srtt - sample).abs();
                self.srtt = 0.875 * self.srtt + 0.125 * sample;
            }
        }
        let rids = std::mem::take(&mut rec.rids);
        if !rids.is_empty() {
            self.pending.retain(|p| !rids.contains(&p.rid));
        }
    }

    /// Process one received packet. Returns its messages in delivery order
    /// (unreliable as they come, reliable in order and exactly once); a
    /// duplicate packet returns nothing. Malformed payloads are an error
    /// (the whole packet is dropped).
    pub fn receive(&mut self, now: f64, h: AckHeader, payload: &[u8]) -> Result<Vec<Msg>, crate::codec::DecodeError> {
        // Parse first: a malformed packet must not move any state.
        let mut parsed: Vec<(Option<u16>, Msg)> = Vec::new();
        let mut r = Reader::new(payload);
        while r.remaining() > 0 {
            let t = r.u8()?;
            let rid = if msg::reliable(t) { Some(r.u16()?) } else { None };
            let n = r.u16()? as usize;
            let body = r.take(n)?;
            parsed.push((rid, Msg::decode_body(t, body)?));
        }
        // Sequence window.
        let s = h.seq;
        if s == 0 {
            self.stats.dropped += 1;
            return Ok(Vec::new());
        }
        if !self.any_received {
            self.any_received = true;
            self.recv_seq = s;
            self.recv_bits = 0;
        } else if seq_gt(s, self.recv_seq) {
            let shift = s.wrapping_sub(self.recv_seq) as u32;
            self.recv_bits = if shift > 32 { 0 } else { ((self.recv_bits as u64) << shift) as u32 | (1u64 << (shift - 1)) as u32 };
            self.recv_seq = s;
        } else {
            let d = self.recv_seq.wrapping_sub(s) as u32;
            if d == 0 || d > 32 || self.recv_bits & (1 << (d - 1)) != 0 {
                self.stats.dropped += 1;
                return Ok(Vec::new());
            }
            self.recv_bits |= 1 << (d - 1);
        }
        self.stats.packets_received += 1;
        self.stats.bytes_received += (DATA_HEADER + payload.len()) as u64;
        // Acks of our packets (0: the peer has received nothing yet).
        if h.ack != 0 {
            self.on_acked(h.ack, now, true);
            for i in 0..32 {
                if h.ack_bits & (1 << i) != 0 {
                    self.on_acked(h.ack.wrapping_sub(i + 1), now, false);
                }
            }
        }
        // Deliver.
        let mut out = Vec::new();
        for (rid, m) in parsed {
            match rid {
                None => out.push(m),
                Some(rid) => {
                    let ahead = rid.wrapping_sub(self.next_deliver);
                    if ahead >= REORDER_WINDOW {
                        continue; // already delivered (behind) or too far ahead (resent later)
                    }
                    self.reorder.entry(rid).or_insert(m);
                }
            }
        }
        while let Some(m) = self.reorder.remove(&self.next_deliver) {
            out.push(m);
            self.next_deliver = self.next_deliver.wrapping_add(1);
        }
        Ok(out)
    }

    /// Fraction of the recent packets (older than ~2 RTT) never acknowledged.
    pub fn loss(&self, now: f64) -> f32 {
        let horizon = now - (self.srtt * 2.0 + 0.15);
        let (mut n, mut lost) = (0, 0);
        for &(t, seq) in &self.loss_hist {
            if t > horizon {
                continue;
            }
            if let Some(rec) = self.sent.get(&seq) {
                n += 1;
                if !rec.acked {
                    lost += 1;
                }
            }
        }
        if n == 0 { 0.0 } else { lost as f32 / n as f32 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chat(s: &str) -> Msg {
        Msg::Chat { from: 1, text: s.into() }
    }

    #[test]
    fn seq_wraps() {
        assert!(seq_gt(1, 0));
        assert!(seq_gt(0, 65535));
        assert!(!seq_gt(65535, 0));
        assert!(!seq_gt(5, 5));
    }

    #[test]
    fn reliable_in_order_despite_loss_and_reordering() {
        let mut a = Channel::new();
        let mut b = Channel::new();
        let mut got = Vec::new();
        let mut t = 0.0;
        for i in 0..200 {
            a.queue_reliable(&chat(&format!("m{i}")));
        }
        let mut in_flight: Vec<(f64, AckHeader, Vec<u8>)> = Vec::new();
        let mut rng = 12345u64;
        let mut rnd = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng % 1000) as f64 / 1000.0
        };
        for _ in 0..2000 {
            t += 0.02;
            for (h, p) in a.send(t, &[], true, 4) {
                // 30 % loss, random delay 0..120 ms (reordering).
                if rnd() > 0.3 {
                    in_flight.push((t + 0.02 + rnd() * 0.12, h, p));
                }
            }
            in_flight.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            while in_flight.first().is_some_and(|f| f.0 <= t) {
                let (_, h, p) = in_flight.remove(0);
                for m in b.receive(t, h, &p).unwrap() {
                    if let Msg::Chat { text, .. } = m {
                        got.push(text);
                    }
                }
            }
            // b acks back (lossy too).
            for (h, p) in b.send(t, &[], true, 1) {
                if rnd() > 0.3 {
                    let _ = a.receive(t, h, &p);
                }
            }
            if got.len() == 200 && a.pending_reliable() == 0 {
                break;
            }
        }
        assert_eq!(got.len(), 200);
        for (i, s) in got.iter().enumerate() {
            assert_eq!(s, &format!("m{i}"));
        }
        assert_eq!(a.pending_reliable(), 0, "all acknowledged");
    }

    #[test]
    fn no_ack_before_receiving() {
        // A peer that has received nothing must not acknowledge our first packet.
        let mut a = Channel::new();
        let mut b = Channel::new();
        a.queue_reliable(&chat("first"));
        let _lost = a.send(0.0, &[], false, 1);
        for (h, p) in b.send(0.01, &[Msg::Ping { t: 0.0 }], true, 1) {
            let _ = a.receive(0.02, h, &p);
        }
        assert_eq!(a.pending_reliable(), 1, "still unacknowledged");
    }

    #[test]
    fn duplicates_dropped() {
        let mut a = Channel::new();
        let mut b = Channel::new();
        a.queue_reliable(&chat("hi"));
        let pk = a.send(0.0, &[Msg::Ping { t: 1.0 }], false, 4);
        let (h, p) = &pk[0];
        assert_eq!(b.receive(0.01, *h, p).unwrap().len(), 2);
        assert!(b.receive(0.02, *h, p).unwrap().is_empty());
    }

    #[test]
    fn malformed_payload_is_an_error_and_moves_nothing() {
        let mut b = Channel::new();
        let h = AckHeader { seq: 5, ack: 0, ack_bits: 0 };
        assert!(b.receive(0.0, h, &[msg::CHAT, 0, 0, 200, 0, 1]).is_err());
        assert!(b.receive(0.0, h, &[250]).is_err());
        assert_eq!(b.stats.packets_received, 0);
    }

    #[test]
    fn rtt_estimate() {
        let mut a = Channel::new();
        let mut b = Channel::new();
        let mut t = 0.0;
        for _ in 0..100 {
            for (h, p) in a.send(t, &[Msg::Ping { t }], true, 1) {
                let _ = b.receive(t + 0.04, h, &p);
            }
            for (h, p) in b.send(t + 0.04, &[], true, 1) {
                let _ = a.receive(t + 0.08, h, &p);
            }
            t += 0.05;
        }
        assert!((a.srtt - 0.08).abs() < 0.01, "srtt {}", a.srtt);
        assert!(a.loss(t) < 0.01);
    }
}
