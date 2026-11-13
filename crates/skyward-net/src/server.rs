//! The server (sans-IO): sessions, handshake, relaying states as snapshots,
//! chat / events, moderation. The `skyward-server` binary wraps it with
//! sockets, a console and logging; tests drive it directly.
//!
//! Robustness: every datagram is size-capped, decoded with bounds checks
//! (malformed → dropped and counted), rate-limited per source address, and
//! nothing in here panics on input. The handshake is stateless until the
//! client echoes a keyed cookie (no state for spoofed sources), requests
//! are padded so replies never amplify, and a session is bound to its
//! address *and* a random token.

use crate::channel::{Channel, MAX_PAYLOAD};
use crate::codec::sanitize;
use crate::proto::*;
use crate::sha256;
use std::collections::{HashMap, HashSet};
use std::net::{IpAddr, SocketAddr};

/// No packet from a client for this long: dropped.
pub const CLIENT_TIMEOUT: f64 = 10.0;
/// The world sync is re-sent this often (clients correct drift).
const WORLD_EVERY: f64 = 60.0;
/// Handshake cookies are valid for two buckets of this length.
const COOKIE_BUCKET: f64 = 30.0;

#[derive(Clone, Debug)]
pub struct ServerConfig {
    pub name: String,
    pub max_players: u16,
    /// Empty: no password.
    pub password: String,
    pub motd: String,
    pub airport: String,
    pub region: String,
    /// Snapshot rate, Hz.
    pub tick_hz: u8,
    pub world: WorldSync,
    /// Game version string (shown to mismatched clients).
    pub version: String,
    pub game_port: u16,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig {
            name: "Skyward server".into(),
            max_players: 16,
            password: String::new(),
            motd: String::new(),
            airport: "EGLC".into(),
            region: String::new(),
            tick_hz: 30,
            world: WorldSync::default(),
            version: env!("CARGO_PKG_VERSION").into(),
            game_port: DEFAULT_PORT,
        }
    }
}

/// Token bucket.
#[derive(Clone, Copy, Debug)]
struct Bucket {
    tokens: f64,
    last: f64,
}

impl Bucket {
    fn new(now: f64, burst: f64) -> Bucket {
        Bucket { tokens: burst, last: now }
    }
    fn take(&mut self, now: f64, rate: f64, burst: f64) -> bool {
        self.tokens = (self.tokens + (now - self.last).max(0.0) * rate).min(burst);
        self.last = now;
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }
}

pub struct Peer {
    pub id: u16,
    pub addr: SocketAddr,
    token: u64,
    nonce: u64,
    pub name: String,
    pub aircraft: String,
    pub spectating: bool,
    pub chan: Channel,
    last_recv: f64,
    pub joined: f64,
    /// Newest state and a counter that grows with each new one.
    pub latest: Option<AcState>,
    latest_n: u64,
    /// Per other player: the `latest_n` last sent to this peer.
    sent_n: HashMap<u16, u64>,
    chat: Bucket,
    pongs: Vec<Msg>,
    /// Smoothed round trip, s (from the reliability layer).
    pub rtt: f64,
}

/// A log line for the host (the binary formats them).
#[derive(Clone, Debug, PartialEq)]
pub struct LogEvent {
    pub level: Level,
    pub msg: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Debug,
    Info,
    Warn,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct ServerStats {
    pub packets_in: u64,
    pub packets_out: u64,
    pub bytes_in: u64,
    pub bytes_out: u64,
    pub malformed: u64,
    pub rate_limited: u64,
}

pub struct Server {
    pub cfg: ServerConfig,
    pub instance: u64,
    secret: [u8; 32],
    pub peers: HashMap<SocketAddr, Peer>,
    by_id: HashMap<u16, SocketAddr>,
    next_id: u16,
    pub bans: HashSet<IpAddr>,
    limits: HashMap<IpAddr, Bucket>,
    next_tick: f64,
    next_world: f64,
    started: f64,
    pub out: Vec<(SocketAddr, Vec<u8>)>,
    pub log: Vec<LogEvent>,
    pub stats: ServerStats,
    /// Chat lines (for the console / tests): (from name, text).
    pub chat_log: Vec<(String, String)>,
}

impl Server {
    /// `seed`: random bytes (keys the cookies and tokens).
    pub fn new(cfg: ServerConfig, seed: [u8; 32], now: f64) -> Server {
        let instance = sha256::digest64(&[&seed[..], b"instance"].concat());
        Server {
            cfg,
            instance,
            secret: seed,
            peers: HashMap::new(),
            by_id: HashMap::new(),
            next_id: 1,
            bans: HashSet::new(),
            limits: HashMap::new(),
            next_tick: now,
            next_world: now + WORLD_EVERY,
            started: now,
            out: Vec::new(),
            log: Vec::new(),
            stats: ServerStats::default(),
            chat_log: Vec::new(),
        }
    }

    fn info(&mut self, msg: impl Into<String>) {
        self.log.push(LogEvent { level: Level::Info, msg: msg.into() });
    }
    fn warn(&mut self, msg: impl Into<String>) {
        self.log.push(LogEvent { level: Level::Warn, msg: msg.into() });
    }
    fn debug(&mut self, msg: impl Into<String>) {
        self.log.push(LogEvent { level: Level::Debug, msg: msg.into() });
    }

    fn send(&mut self, to: SocketAddr, p: &Packet) {
        let b = p.encode();
        self.stats.packets_out += 1;
        self.stats.bytes_out += b.len() as u64;
        self.out.push((to, b));
    }

    pub fn server_info(&self) -> ServerInfo {
        ServerInfo {
            instance: self.instance,
            name: self.cfg.name.clone(),
            players: self.peers.len() as u16,
            max_players: self.cfg.max_players,
            airport: self.cfg.airport.clone(),
            region: self.cfg.region.clone(),
            password: !self.cfg.password.is_empty(),
            game_port: self.cfg.game_port,
            conditions: self.cfg.world.summary(),
        }
    }

    fn details(&self, now: f64) -> ServerDetails {
        let mut players: Vec<PlayerSummary> =
            self.peers.values().map(|p| PlayerSummary { name: p.name.clone(), aircraft: p.aircraft.clone(), ping_ms: (p.rtt * 1000.0).min(9999.0) as u16 }).collect();
        players.sort_by_key(|p| p.name.to_lowercase());
        ServerDetails { motd: self.cfg.motd.clone(), players, world: self.world_now(now), uptime_s: (now - self.started).max(0.0) as u32 }
    }

    /// The world sync stamped with the current UTC.
    fn world_now(&self, _now: f64) -> WorldSync {
        let mut w = self.cfg.world.clone();
        w.utc = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
        w
    }

    fn cookie(&self, addr: SocketAddr, nonce: u64, bucket: i64) -> u64 {
        let mut d = Vec::with_capacity(80);
        d.extend_from_slice(&self.secret);
        d.extend_from_slice(addr.to_string().as_bytes());
        d.extend_from_slice(&nonce.to_le_bytes());
        d.extend_from_slice(&bucket.to_le_bytes());
        sha256::digest64(&d)
    }

    fn allow(&mut self, ip: IpAddr, now: f64, connected: bool) -> bool {
        // Connected peers send ~30–60 packets/s; strangers get far less.
        let (rate, burst) = if connected { (240.0, 480.0) } else { (20.0, 40.0) };
        if self.limits.len() > 4096 {
            // Bounded memory under a flood of sources: forget idle buckets.
            self.limits.retain(|_, b| now - b.last < 10.0);
        }
        let b = self.limits.entry(ip).or_insert_with(|| Bucket::new(now, burst));
        b.take(now, rate, burst)
    }

    /// One datagram from `from`.
    pub fn handle(&mut self, from: SocketAddr, data: &[u8], now: f64) {
        self.stats.packets_in += 1;
        self.stats.bytes_in += data.len() as u64;
        let connected = self.peers.contains_key(&from);
        if !self.allow(from.ip(), now, connected) {
            self.stats.rate_limited += 1;
            return;
        }
        let p = match Packet::decode(data) {
            Ok(p) => p,
            Err(e) => {
                self.stats.malformed += 1;
                if self.stats.malformed.is_power_of_two() {
                    self.debug(format!("malformed packet from {from}: {e} ({} so far)", self.stats.malformed));
                }
                return;
            }
        };
        match p {
            Packet::Discover { nonce, .. } => {
                let reply = Packet::DiscoverReply { nonce, proto: PROTOCOL, version: self.cfg.version.clone(), game_port: self.cfg.game_port, info: Some(self.server_info()) };
                self.send(from, &reply);
            }
            Packet::Details { nonce, .. } => {
                let reply = Packet::DetailsReply { nonce, details: self.details(now) };
                self.send(from, &reply);
            }
            Packet::Hello { proto, nonce } => self.on_hello(from, proto, nonce, now),
            Packet::Join { nonce, cookie, name, aircraft, version, proof } => self.on_join(from, nonce, cookie, &name, &aircraft, &version, proof, now),
            Packet::Data { token, ack, payload } => self.on_data(from, token, ack, &payload, now),
            Packet::Disconnect { token, .. } => {
                if self.peers.get(&from).is_some_and(|p| p.token == token) {
                    self.remove(from, LeaveReason::Quit);
                }
            }
            // Server-to-client packets arriving here: ignore.
            _ => self.stats.malformed += 1,
        }
    }

    fn reject(&mut self, to: SocketAddr, nonce: u64, reason: RejectReason, text: &str) {
        let p = Packet::Reject { nonce, reason, server_proto: PROTOCOL, server_version: self.cfg.version.clone(), text: text.into() };
        self.send(to, &p);
    }

    fn on_hello(&mut self, from: SocketAddr, proto: u16, nonce: u64, now: f64) {
        if proto != PROTOCOL {
            self.info(format!("{from}: version mismatch (protocol {proto}, ours {PROTOCOL})"));
            self.reject(from, nonce, RejectReason::VersionMismatch, "");
            return;
        }
        if self.bans.contains(&from.ip()) {
            self.reject(from, nonce, RejectReason::Banned, "");
            return;
        }
        let bucket = (now / COOKIE_BUCKET).floor() as i64;
        let cookie = self.cookie(from, nonce, bucket);
        self.send(from, &Packet::Challenge { nonce, cookie });
    }

    fn on_join(&mut self, from: SocketAddr, nonce: u64, cookie: u64, name: &str, aircraft: &str, version: &str, proof: [u8; 32], now: f64) {
        // A retransmitted Join of an accepted session: accept again.
        if let Some(p) = self.peers.get(&from) {
            if p.nonce == nonce {
                let (id, token) = (p.id, p.token);
                let welcome = self.welcome(now);
                self.send(from, &Packet::Accept { nonce, player: id, token, welcome });
            }
            return;
        }
        let bucket = (now / COOKIE_BUCKET).floor() as i64;
        if cookie != self.cookie(from, nonce, bucket) && cookie != self.cookie(from, nonce, bucket - 1) {
            self.stats.malformed += 1;
            return; // forged / stale: no state, no reply
        }
        if self.bans.contains(&from.ip()) {
            self.reject(from, nonce, RejectReason::Banned, "");
            return;
        }
        if !self.cfg.password.is_empty() && proof != password_proof(cookie, &self.cfg.password) {
            self.info(format!("{from}: wrong password"));
            self.reject(from, nonce, RejectReason::BadPassword, "");
            return;
        }
        if self.peers.len() >= self.cfg.max_players as usize {
            self.reject(from, nonce, RejectReason::Full, "");
            return;
        }
        let Ok(aircraft) = aircraft_id(aircraft) else {
            self.reject(from, nonce, RejectReason::Protocol, "bad aircraft id");
            return;
        };
        let name = self.unique_name(&sanitize(name, MAX_NAME));
        let Some(id) = self.alloc_id() else {
            self.reject(from, nonce, RejectReason::Full, "");
            return;
        };
        let mut td = Vec::new();
        td.extend_from_slice(&self.secret);
        td.extend_from_slice(&nonce.to_le_bytes());
        td.extend_from_slice(&id.to_le_bytes());
        td.extend_from_slice(&now.to_le_bytes());
        td.extend_from_slice(b"token");
        let token = sha256::digest64(&td);
        let mut peer = Peer {
            id,
            addr: from,
            token,
            nonce,
            name: name.clone(),
            aircraft: aircraft.clone(),
            spectating: false,
            chan: Channel::new(),
            last_recv: now,
            joined: now,
            latest: None,
            latest_n: 0,
            sent_n: HashMap::new(),
            chat: Bucket::new(now, 5.0),
            pongs: Vec::new(),
            rtt: 0.0,
        };
        // Tell the newcomer who is here, and the time / weather.
        for p in self.peers.values() {
            peer.chan.queue_reliable(&Msg::Player(PlayerInfo { id: p.id, name: p.name.clone(), aircraft: p.aircraft.clone(), spectating: p.spectating }));
        }
        peer.chan.queue_reliable(&Msg::World(self.world_now(now)));
        let welcome = self.welcome(now);
        self.peers.insert(from, peer);
        self.by_id.insert(id, from);
        self.send(from, &Packet::Accept { nonce, player: id, token, welcome });
        self.broadcast(Some(id), &Msg::Player(PlayerInfo { id, name: name.clone(), aircraft: aircraft.clone(), spectating: false }));
        self.broadcast(None, &Msg::Chat { from: SERVER_ID, text: format!("{name} joined") });
        let v = if version.is_empty() { String::new() } else { format!(", v{}", sanitize(version, 32)) };
        self.info(format!("join #{id} \"{name}\" from {from} ({aircraft}{v}); {} / {} players", self.peers.len(), self.cfg.max_players));
    }

    fn welcome(&self, now: f64) -> Welcome {
        let mut info = self.server_info();
        info.players = info.players.max(1);
        Welcome { info, motd: self.cfg.motd.clone(), tick_hz: self.cfg.tick_hz.clamp(10, 60), world: self.world_now(now) }
    }

    fn alloc_id(&mut self) -> Option<u16> {
        for _ in 0..0xFFF0 {
            let id = self.next_id;
            self.next_id = if self.next_id >= 0xFFF0 { 1 } else { self.next_id + 1 };
            if !self.by_id.contains_key(&id) {
                return Some(id);
            }
        }
        None
    }

    fn unique_name(&self, base: &str) -> String {
        let base = if base.is_empty() { "Pilot" } else { base };
        let taken = |n: &str| self.peers.values().any(|p| p.name.eq_ignore_ascii_case(n));
        if !taken(base) {
            return base.to_string();
        }
        for k in 2.. {
            let n = format!("{} ({k})", crate::codec::clip(base, MAX_NAME - 4));
            if !taken(&n) {
                return n;
            }
        }
        unreachable!()
    }

    /// Reliable message to everyone (except `except`).
    pub fn broadcast(&mut self, except: Option<u16>, m: &Msg) {
        for p in self.peers.values_mut() {
            if Some(p.id) != except {
                p.chan.queue_reliable(m);
            }
        }
    }

    fn on_data(&mut self, from: SocketAddr, token: u64, ack: AckHeader, payload: &[u8], now: f64) {
        let Some(peer) = self.peers.get_mut(&from) else { return };
        if peer.token != token {
            self.stats.malformed += 1;
            return;
        }
        let msgs = match peer.chan.receive(now, ack, payload) {
            Ok(m) => m,
            Err(_) => {
                self.stats.malformed += 1;
                return;
            }
        };
        peer.last_recv = now;
        peer.rtt = peer.chan.srtt;
        let id = peer.id;
        let mut relay: Vec<Msg> = Vec::new();
        for m in msgs {
            let Some(peer) = self.peers.get_mut(&from) else { return };
            match m {
                Msg::State(mut s) => {
                    // The client stamps with our clock; keep it honest.
                    let lo = now - peer.chan.srtt.min(2.0) - 0.5;
                    if !(lo..=now + 0.05).contains(&s.t) {
                        s.t = now - peer.chan.srtt * 0.5;
                    }
                    if peer.latest.is_some_and(|l| s.t <= l.t) {
                        continue; // reordered: older than what we have
                    }
                    peer.latest = Some(s);
                    peer.latest_n += 1;
                }
                Msg::Ping { t } => peer.pongs.push(Msg::Pong { t, server: now }),
                Msg::Chat { text, .. } => {
                    if !peer.chat.take(now, 1.0, 5.0) {
                        continue; // flood
                    }
                    let text = sanitize(&text, MAX_CHAT);
                    if !text.is_empty() {
                        let name = peer.name.clone();
                        relay.push(Msg::Chat { from: id, text: text.clone() });
                        self.chat_log.push((name.clone(), text.clone()));
                        self.log.push(LogEvent { level: Level::Info, msg: format!("chat <{name}> {text}") });
                    }
                }
                Msg::Event { event, .. } => {
                    relay.push(Msg::Event { from: id, event });
                    if let GameEvent::Crash { severity } = event {
                        let name = peer.name.clone();
                        self.log.push(LogEvent { level: Level::Info, msg: format!("#{id} \"{name}\" crashed (severity {severity})") });
                    }
                }
                Msg::SetAircraft { aircraft, spectating } => {
                    if peer.aircraft != aircraft || peer.spectating != spectating {
                        peer.aircraft = aircraft.clone();
                        peer.spectating = spectating;
                        relay.push(Msg::Player(PlayerInfo { id, name: peer.name.clone(), aircraft, spectating }));
                    }
                }
                // Server-to-client messages from a client: ignore.
                _ => {}
            }
        }
        for m in relay {
            // Chat goes to everyone (the sender sees the server's echo); the rest to the others.
            let except = if matches!(m, Msg::Chat { .. }) { None } else { Some(id) };
            self.broadcast(except, &m);
        }
        // Answer pings at once (clock sync wants no tick delay).
        if let Some(peer) = self.peers.get_mut(&from) {
            if !peer.pongs.is_empty() {
                let pongs = std::mem::take(&mut peer.pongs);
                let pk = peer.chan.send(now, &pongs, false, 2);
                let token = peer.token;
                for (h, payload) in pk {
                    self.send(from, &Packet::Data { token, ack: h, payload });
                }
            }
        }
    }

    fn remove(&mut self, addr: SocketAddr, why: LeaveReason) {
        let Some(p) = self.peers.remove(&addr) else { return };
        self.by_id.remove(&p.id);
        for q in self.peers.values_mut() {
            q.sent_n.remove(&p.id);
        }
        self.broadcast(None, &Msg::Left { id: p.id, reason: why });
        self.broadcast(None, &Msg::Chat { from: SERVER_ID, text: format!("{} {}", p.name, why.verb()) });
        self.info(format!("leave #{} \"{}\" ({}) after {:.0} s; {} players", p.id, p.name, why.verb(), (p.last_recv - p.joined).max(0.0), self.peers.len()));
    }

    /// Advance: snapshots at the tick rate, resends, time-outs. Call often (≥ tick rate).
    pub fn tick(&mut self, now: f64) {
        // Time-outs and broken channels.
        let dead: Vec<(SocketAddr, LeaveReason)> =
            self.peers.iter().filter(|(_, p)| now - p.last_recv > CLIENT_TIMEOUT || p.chan.overflowed()).map(|(a, _)| (*a, LeaveReason::TimedOut)).collect();
        for (a, why) in dead {
            self.remove(a, why);
        }
        if now >= self.next_world {
            self.next_world = now + WORLD_EVERY;
            let w = self.world_now(now);
            self.broadcast(None, &Msg::World(w));
        }
        if now < self.next_tick {
            return;
        }
        let interval = 1.0 / self.cfg.tick_hz.clamp(10, 60) as f64;
        self.next_tick = (self.next_tick + interval).max(now);
        // Everyone's newest state, once per new sample.
        let states: Vec<(u16, AcState, u64)> = self.peers.values().filter_map(|p| p.latest.map(|s| (p.id, s, p.latest_n))).collect();
        let addrs: Vec<SocketAddr> = self.peers.keys().copied().collect();
        for addr in addrs {
            let Some(peer) = self.peers.get_mut(&addr) else { continue };
            let mut entries: Vec<(u16, AcState)> = Vec::new();
            for (id, s, n) in &states {
                if *id == peer.id {
                    continue;
                }
                if peer.sent_n.get(id).is_some_and(|&k| k >= *n) {
                    continue;
                }
                peer.sent_n.insert(*id, *n);
                entries.push((*id, *s));
            }
            // Split into messages that fit a packet each.
            let per = ((MAX_PAYLOAD - 16) / (MAX_STATE_BYTES + 2)).max(1);
            let mut msgs: Vec<Msg> = entries.chunks(per).map(|c| Msg::Snapshot { time: now, entries: c.to_vec() }).collect();
            msgs.append(&mut peer.pongs);
            let max_packets = msgs.len() + 4;
            let pk = peer.chan.send(now, &msgs, true, max_packets);
            let token = peer.token;
            for (h, payload) in pk {
                self.send(addr, &Packet::Data { token, ack: h, payload });
            }
        }
    }

    // ------------------------------------------------------------ console commands

    /// Find a player by id ("#3" or "3") or name (case-insensitive prefix).
    pub fn find(&self, who: &str) -> Option<SocketAddr> {
        let w = who.trim().trim_start_matches('#');
        if let Ok(id) = w.parse::<u16>() {
            if let Some(a) = self.by_id.get(&id) {
                return Some(*a);
            }
        }
        let lw = who.trim().to_lowercase();
        let exact = self.peers.iter().find(|(_, p)| p.name.to_lowercase() == lw).map(|(a, _)| *a);
        exact.or_else(|| {
            let m: Vec<SocketAddr> = self.peers.iter().filter(|(_, p)| p.name.to_lowercase().starts_with(&lw)).map(|(a, _)| *a).collect();
            (m.len() == 1).then(|| m[0])
        })
    }

    /// Kick (and with `ban`, ban the address). Returns a message for the console.
    pub fn kick(&mut self, who: &str, reason: &str, ban: bool) -> Result<String, String> {
        let Some(addr) = self.find(who) else {
            // Ban by address even if nobody is connected from it.
            if ban {
                if let Ok(ip) = who.trim().parse::<IpAddr>() {
                    self.bans.insert(ip);
                    self.info(format!("banned {ip}"));
                    return Ok(format!("banned {ip}"));
                }
            }
            return Err(format!("no player \"{who}\""));
        };
        let p = &self.peers[&addr];
        let (name, token) = (p.name.clone(), p.token);
        let why = if ban { RejectReason::Banned } else { RejectReason::Kicked };
        let pk = Packet::Reject { nonce: token, reason: why, server_proto: PROTOCOL, server_version: self.cfg.version.clone(), text: reason.into() };
        for _ in 0..3 {
            self.send(addr, &pk);
        }
        if ban {
            self.bans.insert(addr.ip());
        }
        self.remove(addr, if ban { LeaveReason::Banned } else { LeaveReason::Kicked });
        let what = if ban { "banned" } else { "kicked" };
        self.warn(format!("{what} \"{name}\" ({}){}", addr.ip(), if reason.is_empty() { String::new() } else { format!(": {reason}") }));
        Ok(format!("{what} {name}"))
    }

    /// Chat from the console.
    pub fn say(&mut self, text: &str) {
        let text = sanitize(text, MAX_CHAT);
        if text.is_empty() {
            return;
        }
        self.chat_log.push(("server".into(), text.clone()));
        self.broadcast(None, &Msg::Chat { from: SERVER_ID, text });
    }

    /// Change the time / weather for everyone.
    pub fn set_world(&mut self, w: WorldSync, now: f64) {
        self.cfg.world = w;
        let w = self.world_now(now);
        self.broadcast(None, &Msg::World(w));
    }

    /// Server shutting down: tell everyone (they get a clear message, not a time-out).
    pub fn shutdown(&mut self, text: &str) {
        let addrs: Vec<(SocketAddr, u64)> = self.peers.iter().map(|(a, p)| (*a, p.token)).collect();
        for (a, token) in addrs {
            let pk = Packet::Reject { nonce: token, reason: RejectReason::ServerShutdown, server_proto: PROTOCOL, server_version: self.cfg.version.clone(), text: text.into() };
            for _ in 0..2 {
                self.send(a, &pk);
            }
        }
        self.peers.clear();
        self.by_id.clear();
    }

    /// (id, name, aircraft, address, ping ms, connected seconds) for `list`.
    pub fn list(&self, now: f64) -> Vec<(u16, String, String, SocketAddr, u32, u32)> {
        let mut v: Vec<_> = self.peers.values().map(|p| (p.id, p.name.clone(), p.aircraft.clone(), p.addr, (p.rtt * 1000.0) as u32, (now - p.joined).max(0.0) as u32)).collect();
        v.sort_by_key(|r| r.0);
        v
    }
}
