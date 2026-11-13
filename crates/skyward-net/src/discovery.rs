//! LAN discovery over real sockets (native only).
//!
//! * The browser's [`Scanner`] broadcasts a padded `Discover` to the
//!   discovery port on every local subnet (and 255.255.255.255 and
//!   localhost), asks mDNS for `_skyward._udp.local`, and probes manual /
//!   favourite addresses directly. Replies carry the server's info; their
//!   round-trip time is the ping.
//! * The server side: [`bind_shared`] opens the discovery port with
//!   address reuse (several servers per host all hear the broadcast) and
//!   [`MdnsResponder`] answers mDNS queries.
//!
//! "Scan my network" is exactly this: one broadcast per subnet and one
//! multicast query — no port scanning.

use crate::mdns;
use crate::proto::{DISCOVERY_PORT, MAX_RECV, PROTOCOL, Packet, ServerDetails, ServerInfo};
use socket2::{Domain, Protocol, Socket, Type};
use std::collections::HashMap;
use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, ToSocketAddrs, UdpSocket};

/// How a server was found.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Via {
    Broadcast,
    Mdns,
    /// A probe of a known address (manual, favourite, recent).
    Direct,
}

#[derive(Clone, Debug)]
pub enum ScanEvent {
    /// A server answered: its game address, protocol / version, info (when
    /// the protocol matches) and the round trip, ms.
    Server {
        addr: SocketAddr,
        proto: u16,
        version: String,
        info: Option<ServerInfo>,
        rtt_ms: f32,
        via: Via,
    },
    Details {
        addr: SocketAddr,
        details: ServerDetails,
    },
}

struct Sent {
    at: f64,
    via: Via,
    /// For direct probes: the address probed (its game address).
    to: Option<SocketAddr>,
}

pub struct Scanner {
    sock: UdpSocket,
    /// Datagrams with their arrival time (a receiver thread timestamps them,
    /// so pings don't include the game's frame time).
    rx: std::sync::mpsc::Receiver<(f64, SocketAddr, Vec<u8>)>,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    epoch: std::time::Instant,
    sent: HashMap<u64, Sent>,
    details: HashMap<u64, SocketAddr>,
    nonce: u64,
    /// Discovery port the broadcasts go to (tests use another).
    pub port: u16,
}

impl Scanner {
    pub fn new(seed: u64) -> io::Result<Scanner> {
        let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        sock.set_broadcast(true)?;
        sock.set_read_timeout(Some(std::time::Duration::from_millis(100)))?;
        let _ = sock.set_multicast_ttl_v4(255);
        let epoch = std::time::Instant::now();
        let (tx, rx) = std::sync::mpsc::channel();
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let (rs, st) = (sock.try_clone()?, stop.clone());
        std::thread::Builder::new().name("lan-discovery".into()).spawn(move || {
            let mut buf = [0u8; 2048];
            while !st.load(std::sync::atomic::Ordering::Relaxed) {
                if let Ok((n, from)) = rs.recv_from(&mut buf) {
                    if n <= MAX_RECV && tx.send((epoch.elapsed().as_secs_f64(), from, buf[..n].to_vec())).is_err() {
                        break;
                    }
                }
            }
        })?;
        Ok(Scanner { sock, rx, stop, epoch, sent: HashMap::new(), details: HashMap::new(), nonce: seed | 1, port: DISCOVERY_PORT })
    }

    /// The scanner's clock (s): send and arrival times are on it.
    fn clock(&self) -> f64 {
        self.epoch.elapsed().as_secs_f64()
    }

    fn next_nonce(&mut self) -> u64 {
        self.nonce = self.nonce.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.nonce
    }

    fn remember(&mut self, n: u64, s: Sent) {
        if self.sent.len() > 512 {
            let cut = s.at - 10.0;
            self.sent.retain(|_, v| v.at > cut);
        }
        self.sent.insert(n, s);
    }

    /// Broadcast on every subnet + localhost, and an mDNS query.
    pub fn scan(&mut self, _now: f64) {
        let now = self.clock();
        let n = self.next_nonce();
        self.remember(n, Sent { at: now, via: Via::Broadcast, to: None });
        let pk = Packet::Discover { proto: PROTOCOL, nonce: n }.encode();
        let mut targets = crate::ifaces::broadcast_targets();
        targets.push(Ipv4Addr::LOCALHOST);
        for t in targets {
            let _ = self.sock.send_to(&pk, SocketAddrV4::new(t, self.port));
        }
        // mDNS: from our ephemeral port (legacy unicast: answers come back here),
        // out of every interface.
        let q = mdns::query((n & 0xffff) as u16, true);
        let dst = SocketAddrV4::new(mdns::MDNS_ADDR, mdns::MDNS_PORT);
        let ifs: Vec<Ipv4Addr> = crate::ifaces::ipv4().into_iter().map(|i| i.addr).collect();
        let sock = socket2::SockRef::from(&self.sock);
        for a in &ifs {
            let _ = sock.set_multicast_if_v4(a);
            let _ = self.sock.send_to(&q, dst);
        }
        if ifs.is_empty() {
            let _ = self.sock.send_to(&q, dst);
        }
    }

    /// Ask one server directly (ping / info).
    pub fn probe(&mut self, addr: SocketAddr, _now: f64, via: Via) {
        let now = self.clock();
        let n = self.next_nonce();
        self.remember(n, Sent { at: now, via, to: Some(addr) });
        let _ = self.sock.send_to(&Packet::Discover { proto: PROTOCOL, nonce: n }.encode(), addr);
    }

    /// Ask for the MOTD, players and conditions.
    pub fn request_details(&mut self, addr: SocketAddr) {
        let n = self.next_nonce();
        if self.details.len() > 64 {
            self.details.clear();
        }
        self.details.insert(n, addr);
        let _ = self.sock.send_to(&Packet::Details { proto: PROTOCOL, nonce: n }.encode(), addr);
    }

    /// Everything that arrived.
    pub fn poll(&mut self, _now: f64) -> Vec<ScanEvent> {
        let mut out = Vec::new();
        let mut mdns_found: Vec<SocketAddr> = Vec::new();
        for _ in 0..256 {
            let Ok((now, from, data)) = self.rx.try_recv() else { break };
            let data = &data[..];
            if from.port() == mdns::MDNS_PORT || !data.starts_with(&crate::proto::MAGIC) {
                if let Some(m) = mdns::parse(data) {
                    for s in mdns::services(&m) {
                        // Prefer the address that answered (it routes to us), then the A records.
                        let mut addrs: Vec<Ipv4Addr> = Vec::new();
                        if let IpAddr::V4(a) = from.ip() {
                            if s.addrs.is_empty() || s.addrs.contains(&a) {
                                addrs.push(a);
                            }
                        }
                        for a in &s.addrs {
                            if !addrs.contains(a) {
                                addrs.push(*a);
                            }
                        }
                        if let Some(a) = addrs.first() {
                            mdns_found.push(SocketAddr::V4(SocketAddrV4::new(*a, s.port)));
                        }
                    }
                }
                continue;
            }
            match Packet::decode(data) {
                Ok(Packet::DiscoverReply { nonce, proto, version, game_port, info }) => {
                    let Some(s) = self.sent.get(&nonce) else { continue };
                    let rtt_ms = ((now - s.at) * 1000.0).max(0.0) as f32;
                    // Direct probes know the game address; broadcast replies come from
                    // the discovery port of the server's host.
                    let addr = match s.to {
                        Some(a) => a,
                        None => SocketAddr::new(from.ip(), game_port),
                    };
                    let via = s.via;
                    if s.to.is_some() {
                        self.sent.remove(&nonce);
                    }
                    out.push(ScanEvent::Server { addr, proto, version, info, rtt_ms, via });
                }
                Ok(Packet::DetailsReply { nonce, details }) => {
                    if let Some(addr) = self.details.remove(&nonce) {
                        out.push(ScanEvent::Details { addr, details });
                    }
                }
                _ => {}
            }
        }
        for a in mdns_found {
            self.probe(a, 0.0, Via::Mdns);
        }
        out
    }
}

impl Drop for Scanner {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
    }
}

/// "host", "host:port", "1.2.3.4", "[::1]:47820" → socket address (blocking DNS).
pub fn resolve(s: &str, default_port: u16) -> io::Result<SocketAddr> {
    let s = s.trim();
    if s.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "empty address"));
    }
    if let Ok(a) = s.parse::<SocketAddr>() {
        return Ok(a);
    }
    if let Ok(ip) = s.parse::<IpAddr>() {
        return Ok(SocketAddr::new(ip, default_port));
    }
    let with_port = if s.rsplit_once(':').is_some_and(|(_, p)| p.parse::<u16>().is_ok()) { s.to_string() } else { format!("{s}:{default_port}") };
    let mut it = with_port.to_socket_addrs()?;
    let all: Vec<SocketAddr> = it.by_ref().collect();
    all.iter().find(|a| a.is_ipv4()).or(all.first()).copied().ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, format!("{s}: no address")))
}

/// A UDP socket on a port several processes may share (SO_REUSEADDR, and
/// SO_REUSEPORT on Unix), with broadcast enabled, non-blocking.
pub fn bind_shared(addr: SocketAddrV4) -> io::Result<UdpSocket> {
    let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    s.set_reuse_address(true)?;
    #[cfg(all(unix, not(any(target_os = "solaris", target_os = "illumos"))))]
    s.set_reuse_port(true)?;
    s.set_broadcast(true)?;
    s.bind(&SocketAddr::V4(addr).into())?;
    s.set_nonblocking(true)?;
    Ok(s.into())
}

/// Answers mDNS queries for the server (shares port 5353 with the system's
/// responder, e.g. Avahi / Bonjour).
pub struct MdnsResponder {
    sock: UdpSocket,
    pub instance: String,
    pub host: String,
    pub port: u16,
    last_announce: f64,
    announces: u32,
}

impl MdnsResponder {
    pub fn new(instance: &str, host: &str, port: u16) -> io::Result<MdnsResponder> {
        let sock = bind_shared(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, mdns::MDNS_PORT))?;
        let mut joined = 0;
        for i in crate::ifaces::ipv4() {
            if sock.join_multicast_v4(&mdns::MDNS_ADDR, &i.addr).is_ok() {
                joined += 1;
            }
        }
        if joined == 0 {
            sock.join_multicast_v4(&mdns::MDNS_ADDR, &Ipv4Addr::UNSPECIFIED)?;
        }
        let _ = sock.set_multicast_ttl_v4(255);
        let _ = sock.set_multicast_loop_v4(true);
        Ok(MdnsResponder { sock, instance: mdns::instance_label(instance), host: host.into(), port, last_announce: f64::NEG_INFINITY, announces: 0 })
    }

    fn answer(&self, id: u16, txt: &[String], ttl: u32) -> Vec<u8> {
        let addrs: Vec<Ipv4Addr> = crate::ifaces::ipv4().into_iter().filter(|i| !i.loopback).map(|i| i.addr).collect();
        let addrs = if addrs.is_empty() { vec![Ipv4Addr::LOCALHOST] } else { addrs };
        mdns::response(id, &self.instance, &self.host, self.port, txt, &addrs, ttl)
    }

    /// Answer queries; announce at start (RFC 6762 §8.3). `txt`: key=value pairs.
    pub fn poll(&mut self, now: f64, txt: &[String]) {
        if self.announces < 2 && now - self.last_announce > 1.0 {
            self.announces += 1;
            self.last_announce = now;
            let _ = self.sock.send_to(&self.answer(0, txt, 120), SocketAddrV4::new(mdns::MDNS_ADDR, mdns::MDNS_PORT));
        }
        let fqdn = format!("{}.{}", self.instance, crate::proto::MDNS_SERVICE);
        let mut buf = [0u8; 2048];
        for _ in 0..64 {
            let Ok((n, from)) = self.sock.recv_from(&mut buf) else { break };
            let Some(m) = mdns::parse(&buf[..n]) else { continue };
            let (asks, qu) = mdns::asks_for_us(&m, &fqdn);
            if !asks {
                continue;
            }
            // Legacy unicast (source port ≠ 5353) must get a direct reply echoing the id.
            if from.port() != mdns::MDNS_PORT {
                let _ = self.sock.send_to(&self.answer(m.id, txt, 10), from);
            } else if qu {
                let _ = self.sock.send_to(&self.answer(0, txt, 120), from);
            } else {
                let _ = self.sock.send_to(&self.answer(0, txt, 120), SocketAddrV4::new(mdns::MDNS_ADDR, mdns::MDNS_PORT));
            }
        }
    }

    /// Goodbye (TTL 0) so browsers drop the entry at once.
    pub fn goodbye(&self, txt: &[String]) {
        let _ = self.sock.send_to(&self.answer(0, txt, 0), SocketAddrV4::new(mdns::MDNS_ADDR, mdns::MDNS_PORT));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_forms() {
        assert_eq!(resolve("127.0.0.1", 47820).unwrap(), "127.0.0.1:47820".parse().unwrap());
        assert_eq!(resolve("127.0.0.1:5000", 47820).unwrap(), "127.0.0.1:5000".parse().unwrap());
        assert_eq!(resolve("[::1]:5000", 47820).unwrap(), "[::1]:5000".parse().unwrap());
        assert_eq!(resolve("localhost", 47820).unwrap().port(), 47820);
        assert!(resolve("", 1).is_err());
    }
}
