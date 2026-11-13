//! The client side of a connection (sans-IO): handshake, reliability,
//! clock sync and keep-alive. Feed it received datagrams with
//! [`Client::handle`], call [`Client::update`] every frame, and send what
//! [`Client::take_outgoing`] returns to the server address.

use crate::channel::Channel;
use crate::clock::ClockSync;
use crate::proto::{AcState, MAX_NAME, Msg, PROTOCOL, Packet, RejectReason, Welcome, password_proof};

/// Handshake retry interval and attempts.
const RETRY: f64 = 0.5;
const ATTEMPTS: u32 = 10;
/// No packet from the server for this long: connection lost.
pub const TIMEOUT: f64 = 10.0;
const PING_EVERY: f64 = 0.5;

/// What the player asks to join with.
#[derive(Clone, Debug)]
pub struct JoinRequest {
    pub name: String,
    pub aircraft: String,
    pub password: String,
    /// Game version string (shown in mismatch messages).
    pub version: String,
}

/// Why connecting failed.
#[derive(Clone, Debug, PartialEq)]
pub enum ConnectError {
    /// No answer to the handshake.
    Unreachable,
    /// The server refused (full, wrong version, bad password, banned...).
    Rejected { reason: RejectReason, server_version: String, text: String },
    /// Could not resolve / bind (from the socket layer).
    Io(String),
}

impl ConnectError {
    pub fn describe(&self) -> String {
        match self {
            ConnectError::Unreachable => "No answer from the server — check the address, that it is running, and the firewall (UDP).".into(),
            ConnectError::Rejected { reason: RejectReason::VersionMismatch, server_version, .. } => {
                format!("Version mismatch: the server runs Skyward {server_version} (protocol differs from this {}).", env!("CARGO_PKG_VERSION"))
            }
            ConnectError::Rejected { reason, text, .. } if !text.is_empty() && *reason == RejectReason::Kicked => format!("{} {}", reason.describe(), text),
            ConnectError::Rejected { reason, .. } => reason.describe().to_string(),
            ConnectError::Io(e) => e.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Phase {
    Hello {
        attempts: u32,
        last: f64,
    },
    Join {
        cookie: u64,
        attempts: u32,
        last: f64,
    },
    Connected,
    Failed(ConnectError),
    /// Ended (kicked, server shut down, timed out, or we left).
    Closed {
        reason: RejectReason,
        text: String,
    },
}

#[derive(Clone, Debug)]
pub enum ClientEvent {
    Connected {
        player: u16,
        welcome: Box<Welcome>,
    },
    Failed(ConnectError),
    /// A delivered message (pings / pongs are handled here).
    Msg(Msg),
    Closed {
        reason: RejectReason,
        text: String,
    },
}

pub struct Client {
    pub phase: Phase,
    req: JoinRequest,
    nonce: u64,
    token: u64,
    pub player: u16,
    pub chan: Channel,
    pub clock: ClockSync,
    pub welcome: Option<Welcome>,
    last_recv: f64,
    next_send: f64,
    next_ping: f64,
    send_interval: f64,
    state: Option<AcState>,
    out: Vec<Vec<u8>>,
    last_update: f64,
}

impl Client {
    /// Start connecting (`nonce`: random, identifies this attempt).
    pub fn connect(mut req: JoinRequest, now: f64, nonce: u64) -> Client {
        req.name = crate::codec::sanitize(&req.name, MAX_NAME);
        if req.name.is_empty() {
            req.name = "Pilot".into();
        }
        let mut c = Client {
            phase: Phase::Hello { attempts: 0, last: f64::NEG_INFINITY },
            req,
            nonce,
            token: 0,
            player: 0,
            chan: Channel::new(),
            clock: ClockSync::default(),
            welcome: None,
            last_recv: now,
            next_send: now,
            next_ping: now,
            send_interval: 1.0 / 30.0,
            state: None,
            out: Vec::new(),
            last_update: now,
        };
        c.update(now);
        c
    }

    pub fn connected(&self) -> bool {
        self.phase == Phase::Connected
    }

    /// Server time now (valid once connected and synced).
    pub fn server_time(&self, now: f64) -> f64 {
        self.clock.server_time(now)
    }

    /// The own state sampled at local time `now` (stamped with the server
    /// clock then); the newest is sent at the tick rate.
    pub fn set_state(&mut self, mut s: AcState, now: f64) {
        if self.clock.synced() {
            s.t = self.clock.server_time(now);
            self.state = Some(s);
        }
    }

    /// A reliable message (chat, events, aircraft change).
    pub fn send(&mut self, m: Msg) {
        if self.connected() {
            self.chan.queue_reliable(&m);
        }
    }

    /// Leave politely (the server frees the slot at once).
    pub fn disconnect(&mut self) {
        if self.connected() {
            let p = Packet::Disconnect { token: self.token, text: "quit".into() };
            for _ in 0..2 {
                self.out.push(p.encode());
            }
        }
        if matches!(self.phase, Phase::Connected | Phase::Hello { .. } | Phase::Join { .. }) {
            self.phase = Phase::Closed { reason: RejectReason::Kicked, text: "left".into() };
        }
    }

    pub fn take_outgoing(&mut self) -> Vec<Vec<u8>> {
        std::mem::take(&mut self.out)
    }

    /// A datagram from the server address.
    pub fn handle(&mut self, data: &[u8], now: f64) -> Vec<ClientEvent> {
        let mut ev = Vec::new();
        let Ok(p) = Packet::decode(data) else { return ev };
        match (p, self.phase.clone()) {
            (Packet::Challenge { nonce, cookie }, Phase::Hello { .. }) if nonce == self.nonce => {
                self.last_recv = now;
                self.phase = Phase::Join { cookie, attempts: 0, last: f64::NEG_INFINITY };
                self.update(now);
            }
            (Packet::Accept { nonce, player, token, welcome }, Phase::Join { .. } | Phase::Hello { .. }) if nonce == self.nonce => {
                self.last_recv = now;
                self.phase = Phase::Connected;
                self.player = player;
                self.token = token;
                self.send_interval = 1.0 / welcome.tick_hz.clamp(10, 60) as f64;
                self.welcome = Some(welcome.clone());
                self.next_ping = now;
                ev.push(ClientEvent::Connected { player, welcome: Box::new(welcome) });
            }
            (Packet::Reject { nonce, reason, server_version, text, .. }, Phase::Hello { .. } | Phase::Join { .. }) if nonce == self.nonce => {
                let e = ConnectError::Rejected { reason, server_version, text };
                self.phase = Phase::Failed(e.clone());
                ev.push(ClientEvent::Failed(e));
            }
            // Mid-session refusal (kick / ban / shutdown): the server sends the session token as the nonce.
            (Packet::Reject { nonce, reason, text, .. }, Phase::Connected) if nonce == self.token => {
                self.phase = Phase::Closed { reason, text: text.clone() };
                ev.push(ClientEvent::Closed { reason, text });
            }
            (Packet::Disconnect { token, text }, Phase::Connected) if token == self.token => {
                self.phase = Phase::Closed { reason: RejectReason::ServerShutdown, text: text.clone() };
                ev.push(ClientEvent::Closed { reason: RejectReason::ServerShutdown, text });
            }
            (Packet::Data { token, ack, payload }, Phase::Connected) if token == self.token => {
                let Ok(msgs) = self.chan.receive(now, ack, &payload) else { return ev };
                self.last_recv = now;
                for m in msgs {
                    match m {
                        Msg::Pong { t, server } => self.clock.sample(t, server, now),
                        Msg::Ping { .. } | Msg::State(_) | Msg::SetAircraft { .. } => {}
                        m => ev.push(ClientEvent::Msg(m)),
                    }
                }
            }
            _ => {}
        }
        ev
    }

    /// Retries, time-outs, pings and this tick's packet.
    pub fn update(&mut self, now: f64) -> Vec<ClientEvent> {
        let mut ev = Vec::new();
        let dt = (now - self.last_update).clamp(0.0, 1.0);
        self.last_update = now;
        match self.phase.clone() {
            Phase::Hello { attempts, last } if now - last >= RETRY => {
                if attempts >= ATTEMPTS {
                    self.phase = Phase::Failed(ConnectError::Unreachable);
                    ev.push(ClientEvent::Failed(ConnectError::Unreachable));
                } else {
                    self.out.push(Packet::Hello { proto: PROTOCOL, nonce: self.nonce }.encode());
                    self.phase = Phase::Hello { attempts: attempts + 1, last: now };
                }
            }
            Phase::Join { cookie, attempts, last } if now - last >= RETRY => {
                if attempts >= ATTEMPTS {
                    self.phase = Phase::Failed(ConnectError::Unreachable);
                    ev.push(ClientEvent::Failed(ConnectError::Unreachable));
                } else {
                    let proof = password_proof(cookie, &self.req.password);
                    self.out.push(
                        Packet::Join { nonce: self.nonce, cookie, name: self.req.name.clone(), aircraft: self.req.aircraft.clone(), version: self.req.version.clone(), proof }
                            .encode(),
                    );
                    self.phase = Phase::Join { cookie, attempts: attempts + 1, last: now };
                }
            }
            Phase::Connected => {
                self.clock.update(dt);
                if now - self.last_recv > TIMEOUT || self.chan.overflowed() {
                    self.phase = Phase::Closed { reason: RejectReason::TimedOut, text: String::new() };
                    ev.push(ClientEvent::Closed { reason: RejectReason::TimedOut, text: String::new() });
                    return ev;
                }
                if now >= self.next_send {
                    // Keep the cadence (no drift), but never burst to catch up.
                    self.next_send = (self.next_send + self.send_interval).max(now);
                    let mut unrel = Vec::new();
                    if now >= self.next_ping {
                        self.next_ping = now + PING_EVERY;
                        unrel.push(Msg::Ping { t: now });
                    }
                    if let Some(s) = self.state.take() {
                        unrel.push(Msg::State(s));
                    }
                    for (h, payload) in self.chan.send(now, &unrel, true, 4) {
                        self.out.push(Packet::Data { token: self.token, ack: h, payload }.encode());
                    }
                }
            }
            _ => {}
        }
        ev
    }

    /// Round-trip time to the server, ms.
    pub fn ping_ms(&self) -> f32 {
        (self.clock.rtt * 1000.0) as f32
    }
}
