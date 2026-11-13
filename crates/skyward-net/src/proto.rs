//! The wire protocol: packets (connectionless discovery / handshake, and
//! connected `Data` packets carrying messages) and their encodings.
//!
//! Every packet starts with `MAGIC` and a kind byte. Positions on the wire
//! are geodetic (latitude / longitude f64, altitude m) — never the game's
//! floating-origin local frame — and attitudes / velocities are in the local
//! east-up-south tangent frame (the game's world axes: +X east, +Y up,
//! −Z north), which every client shares wherever its origin is.
//!
//! Compatibility: the `Discover` / `DiscoverReply` header (nonce, protocol,
//! version) and the `Reject` packet are frozen, so a client of any version
//! can list a server and show *why* it cannot join ("version mismatch").

use crate::codec::{DecodeError, Reader, Result, Writer, clip, sanitize};

pub const MAGIC: [u8; 4] = *b"SKWD";
/// Bumped on any incompatible change of the connected protocol.
pub const PROTOCOL: u16 = 1;
/// Game port (UDP), configurable on the server.
pub const DEFAULT_PORT: u16 = 47820;
/// LAN discovery broadcast port (UDP, shared by every server on a host).
pub const DISCOVERY_PORT: u16 = 47821;
/// mDNS / DNS-SD service type.
pub const MDNS_SERVICE: &str = "_skyward._udp.local";
/// Largest datagram either side sends (fits any path MTU with headroom).
pub const MAX_PACKET: usize = 1200;
/// Larger datagrams are dropped unread.
pub const MAX_RECV: usize = 1500;
/// Connectionless requests are padded to this size, so a reply is never
/// much larger than the request (no amplification).
pub const MIN_REQUEST: usize = 128;
pub const MAX_NAME: usize = 24;
pub const MAX_CHAT: usize = 200;
pub const MAX_MOTD: usize = 600;
pub const MAX_AIRCRAFT_ID: usize = 32;
pub const MAX_ENGINES: usize = 4;
/// Metres per degree of latitude (the game's equirectangular frames use the same).
pub const M_PER_DEG_LAT: f64 = 111_320.0;

/// The reserved "player id" of the server (chat from the console).
pub const SERVER_ID: u16 = 0xFFFF;

// ---------------------------------------------------------------- aircraft state

/// Rotorcraft state for the model's rotor animation and sound.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotorState {
    pub nr_rpm: f32,
    pub nr_pct: f32,
    pub tail_rpm: f32,
    pub coning_deg: f32,
    pub collective_deg: f32,
    pub tilt_aft_deg: f32,
    pub tilt_right_deg: f32,
    pub tail_pitch_deg: f32,
}

/// One engine, for the model (props, fans, nozzles) and the engine sound.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EngineState {
    /// Gauge RPM (piston crank, turboprop Np, jet N1 rpm).
    pub rpm: f32,
    /// Power fraction (props) or N1 fraction (jets), 0..1.
    pub spool: f32,
    /// Thrust / max dry thrust, −1..1 (reverse negative).
    pub thrust: f32,
    pub running: bool,
}

pub mod lights {
    pub const NAV: u8 = 1;
    pub const BEACON: u8 = 2;
    pub const STROBE: u8 = 4;
    pub const LANDING: u8 = 8;
    pub const TAXI: u8 = 16;
}

pub mod flags {
    pub const ON_GROUND: u16 = 1;
    pub const CRASHED: u16 = 2;
    /// The atmosphere at the aircraft makes contrails (Schmidt–Appleman).
    pub const CONTRAIL: u16 = 4;
    /// Trailing smoke (engine fire, damage, airshow smoke).
    pub const SMOKE: u16 = 8;
    pub const ENGINE_FIRE: u16 = 16;
    pub const ROTORCRAFT: u16 = 32;
    /// Paused / in the menu: others see it frozen, not extrapolated.
    pub const PAUSED: u16 = 64;
    /// Spectating another player: not drawn.
    pub const SPECTATING: u16 = 128;
    /// Gear down lever (for the gear-warning-free "on approach" look).
    pub const GEAR_LEVER: u16 = 256;
}

/// The state of one aircraft at one instant: everything another client needs
/// to draw it with the real model, animate it and make its sound.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AcState {
    /// Sample time on the server clock, s.
    pub t: f64,
    /// Geodetic position: degrees, metres above mean sea level.
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
    /// Body → local tangent frame (x east, y up, z south), quaternion xyzw.
    pub rot: [f32; 4],
    /// Velocity in the local tangent frame, m/s.
    pub vel: [f32; 3],
    /// Angular velocity in the local tangent frame, rad/s.
    pub omega: [f32; 3],
    /// Control surfaces: −1..1 (aileron / elevator / rudder, `Surfaces` signs).
    pub aileron: f32,
    pub elevator: f32,
    pub rudder: f32,
    /// 0..1.
    pub flaps: f32,
    pub spoilers: f32,
    /// Gear 0 = up .. 1 = down.
    pub gear: f32,
    pub reverser: f32,
    pub afterburner: f32,
    pub throttle: f32,
    pub brakes: f32,
    /// True airspeed m/s (sound, instruments of the spectator view).
    pub tas: f32,
    pub n_engines: u8,
    pub engines: [EngineState; MAX_ENGINES],
    pub rotor: RotorState,
    /// `lights::*` bits (switches; flashing is derived from the clock).
    pub lights: u8,
    /// `flags::*` bits.
    pub flags: u16,
    /// Structural parts lost (`damage::Part` bits).
    pub lost_parts: u16,
}

fn q16(v: f32, scale: f32) -> i16 {
    (v * scale).round().clamp(-32767.0, 32767.0) as i16
}

impl AcState {
    pub fn has(&self, f: u16) -> bool {
        self.flags & f != 0
    }

    pub fn encode(&self, w: &mut Writer) {
        w.f64(self.t);
        w.f64(self.lat);
        w.f64(self.lon);
        w.f32(self.alt);
        // Unit quaternion: 4 × i16 (≈ 0.004° resolution); the sign is free.
        for c in self.rot {
            w.i16(q16(c, 32767.0));
        }
        // Velocity 1/20 m/s (±1,638 m/s), angular velocity 1/1000 rad/s (±32 rad/s).
        for c in self.vel {
            w.i16(q16(c, 20.0));
        }
        for c in self.omega {
            w.i16(q16(c, 1000.0));
        }
        w.snorm8(self.aileron);
        w.snorm8(self.elevator);
        w.snorm8(self.rudder);
        for v in [self.flaps, self.spoilers, self.gear, self.reverser, self.afterburner, self.throttle, self.brakes] {
            w.unorm8(v);
        }
        w.u16((self.tas.clamp(0.0, 6553.5) * 10.0).round() as u16);
        w.u8(self.lights);
        w.u16(self.flags);
        w.u16(self.lost_parts);
        let n = (self.n_engines as usize).min(MAX_ENGINES);
        let mut running = 0u8;
        for (i, e) in self.engines[..n].iter().enumerate() {
            if e.running {
                running |= 1 << i;
            }
        }
        w.u8(n as u8 | running << 4);
        for e in &self.engines[..n] {
            w.u16(e.rpm.clamp(0.0, 65535.0).round() as u16);
            w.unorm8(e.spool);
            w.snorm8(e.thrust);
        }
        if self.has(flags::ROTORCRAFT) {
            let r = &self.rotor;
            w.u16((r.nr_rpm.clamp(0.0, 6553.5) * 10.0).round() as u16);
            w.u16((r.tail_rpm.clamp(0.0, 65535.0)).round() as u16);
            w.u8(r.nr_pct.clamp(0.0, 255.0).round() as u8);
            for v in [r.coning_deg, r.collective_deg, r.tilt_aft_deg, r.tilt_right_deg, r.tail_pitch_deg] {
                w.i16(q16(v, 100.0));
            }
        }
    }

    pub fn decode(r: &mut Reader) -> Result<AcState> {
        let mut s = AcState { t: r.f64()?, lat: r.f64()?, lon: r.f64()?, alt: r.f32()?, ..Default::default() };
        if !(-90.0..=90.0).contains(&s.lat) || !(-540.0..=540.0).contains(&s.lon) || !(-2000.0..=100_000.0).contains(&s.alt) {
            return Err(DecodeError::Invalid("position"));
        }
        let mut q = [0f32; 4];
        for c in &mut q {
            *c = r.i16()? as f32 / 32767.0;
        }
        let n = (q.iter().map(|c| c * c).sum::<f32>()).sqrt();
        if n < 0.5 {
            return Err(DecodeError::Invalid("attitude"));
        }
        s.rot = q.map(|c| c / n);
        for c in &mut s.vel {
            *c = r.i16()? as f32 / 20.0;
        }
        for c in &mut s.omega {
            *c = r.i16()? as f32 / 1000.0;
        }
        s.aileron = r.snorm8()?;
        s.elevator = r.snorm8()?;
        s.rudder = r.snorm8()?;
        s.flaps = r.unorm8()?;
        s.spoilers = r.unorm8()?;
        s.gear = r.unorm8()?;
        s.reverser = r.unorm8()?;
        s.afterburner = r.unorm8()?;
        s.throttle = r.unorm8()?;
        s.brakes = r.unorm8()?;
        s.tas = r.u16()? as f32 / 10.0;
        s.lights = r.u8()?;
        s.flags = r.u16()?;
        s.lost_parts = r.u16()?;
        let ne = r.u8()?;
        let n = (ne & 0x0f) as usize;
        if n > MAX_ENGINES {
            return Err(DecodeError::Invalid("engine count"));
        }
        s.n_engines = n as u8;
        for i in 0..n {
            s.engines[i] = EngineState { rpm: r.u16()? as f32, spool: r.unorm8()?, thrust: r.snorm8()?, running: ne >> 4 & (1 << i) != 0 };
        }
        if s.has(flags::ROTORCRAFT) {
            let rs = &mut s.rotor;
            rs.nr_rpm = r.u16()? as f32 / 10.0;
            rs.tail_rpm = r.u16()? as f32;
            rs.nr_pct = r.u8()? as f32;
            for v in [&mut rs.coning_deg, &mut rs.collective_deg, &mut rs.tilt_aft_deg, &mut rs.tilt_right_deg, &mut rs.tail_pitch_deg] {
                *v = r.i16()? as f32 / 100.0;
            }
        }
        Ok(s)
    }
}

// ---------------------------------------------------------------- server description

/// How the server sets the time of day.
#[derive(Clone, Debug, PartialEq)]
pub enum TimeSync {
    /// Real time: everyone sees the sun of the current UTC instant.
    Real,
    /// A fixed local time of day (hours) and day of year, advancing in real time.
    Fixed { hours: f32, day: u16 },
}

/// The world conditions every client applies (time and weather).
#[derive(Clone, Debug, PartialEq)]
pub struct WorldSync {
    pub time: TimeSync,
    /// UTC at the server when this was sent (Unix seconds).
    pub utc: f64,
    /// The weather as a METAR (live: the server's latest fetch for its
    /// airport; fixed: configured). Empty = clients keep their own.
    pub metar: String,
    /// Live weather (re-fetched by the server) or fixed.
    pub live_weather: bool,
}

impl Default for WorldSync {
    fn default() -> Self {
        WorldSync { time: TimeSync::Real, utc: 0.0, metar: String::new(), live_weather: false }
    }
}

impl WorldSync {
    fn encode(&self, w: &mut Writer) {
        match self.time {
            TimeSync::Real => w.u8(0),
            TimeSync::Fixed { hours, day } => {
                w.u8(1);
                w.f32(hours);
                w.u16(day);
            }
        }
        w.f64(self.utc);
        w.str(&self.metar, 256);
        w.bool(self.live_weather);
    }
    fn decode(r: &mut Reader) -> Result<WorldSync> {
        let time = match r.u8()? {
            0 => TimeSync::Real,
            1 => {
                let hours = r.f32()?;
                let day = r.u16()?;
                if !(0.0..24.0).contains(&hours) || !(1..=366).contains(&day) {
                    return Err(DecodeError::Invalid("time"));
                }
                TimeSync::Fixed { hours, day }
            }
            _ => return Err(DecodeError::Invalid("time mode")),
        };
        Ok(WorldSync { time, utc: r.f64()?, metar: r.str(256)?, live_weather: r.bool()? })
    }
    /// "Real time · live weather", "Fixed 14:00 · METAR" for the browser.
    pub fn summary(&self) -> String {
        let t = match self.time {
            TimeSync::Real => "Real time".to_string(),
            TimeSync::Fixed { hours, .. } => {
                let h = hours.rem_euclid(24.0);
                format!("{:02}:{:02} local", h as u32, ((h.fract()) * 60.0) as u32)
            }
        };
        let w = if self.metar.is_empty() {
            "own weather"
        } else if self.live_weather {
            "live weather"
        } else {
            "fixed weather"
        };
        format!("{t} · {w}")
    }
}

/// What a server advertises (discovery replies, the browser list).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct ServerInfo {
    /// Random per server process: dedupes a server found several ways.
    pub instance: u64,
    pub name: String,
    pub players: u16,
    pub max_players: u16,
    /// Departure airport (ICAO / OurAirports ident) everyone starts at.
    pub airport: String,
    /// Free text: "London", "Alps"...
    pub region: String,
    pub password: bool,
    pub game_port: u16,
    /// Time / weather summary ("Real time · live weather").
    pub conditions: String,
}

impl ServerInfo {
    fn encode(&self, w: &mut Writer) {
        w.u64(self.instance);
        w.str(&self.name, 64);
        w.u16(self.players);
        w.u16(self.max_players);
        w.str(&self.airport, 8);
        w.str(&self.region, 48);
        w.bool(self.password);
        w.u16(self.game_port);
        w.str(&self.conditions, 64);
    }
    fn decode(r: &mut Reader) -> Result<ServerInfo> {
        Ok(ServerInfo {
            instance: r.u64()?,
            name: sanitize(&r.str(64)?, 48),
            players: r.u16()?,
            max_players: r.u16()?,
            airport: sanitize(&r.str(8)?, 8),
            region: sanitize(&r.str(48)?, 40),
            password: r.bool()?,
            game_port: r.u16()?,
            conditions: sanitize(&r.str(64)?, 64),
        })
    }
}

/// A player as listed in the server details.
#[derive(Clone, Debug, PartialEq)]
pub struct PlayerSummary {
    pub name: String,
    pub aircraft: String,
    /// Round-trip time to the server, ms.
    pub ping_ms: u16,
}

/// The browser's details panel: MOTD, players, conditions.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct ServerDetails {
    pub motd: String,
    pub players: Vec<PlayerSummary>,
    pub world: WorldSync,
    pub uptime_s: u32,
}

impl ServerDetails {
    fn encode(&self, w: &mut Writer) {
        w.str(&self.motd, MAX_MOTD);
        self.world.encode(w);
        w.u32(self.uptime_s);
        // As many players as fit in the datagram.
        let fit: Vec<&PlayerSummary> = {
            let mut room = MAX_PACKET.saturating_sub(w.len() + 8);
            self.players
                .iter()
                .take_while(|p| {
                    let n = 2 + p.name.len() + 2 + p.aircraft.len() + 2;
                    if n > room {
                        return false;
                    }
                    room -= n;
                    true
                })
                .collect()
        };
        w.u16(fit.len() as u16);
        for p in fit {
            w.str(&p.name, MAX_NAME * 4);
            w.str(&p.aircraft, MAX_AIRCRAFT_ID);
            w.u16(p.ping_ms);
        }
    }
    fn decode(r: &mut Reader) -> Result<ServerDetails> {
        let motd = r.str(MAX_MOTD)?;
        let world = WorldSync::decode(r)?;
        let uptime_s = r.u32()?;
        let n = r.u16()? as usize;
        if n > 256 {
            return Err(DecodeError::Invalid("player count"));
        }
        let mut players = Vec::with_capacity(n);
        for _ in 0..n {
            players.push(PlayerSummary { name: sanitize(&r.str(MAX_NAME * 4)?, MAX_NAME), aircraft: r.str(MAX_AIRCRAFT_ID)?, ping_ms: r.u16()? });
        }
        Ok(ServerDetails { motd, players, world, uptime_s })
    }
}

/// Why the server refused (or ended) a connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectReason {
    VersionMismatch,
    Full,
    BadPassword,
    Banned,
    Kicked,
    /// Malformed or out-of-order handshake.
    Protocol,
    ServerShutdown,
    TimedOut,
    RateLimited,
}

impl RejectReason {
    pub fn code(self) -> u8 {
        match self {
            RejectReason::VersionMismatch => 1,
            RejectReason::Full => 2,
            RejectReason::BadPassword => 3,
            RejectReason::Banned => 4,
            RejectReason::Kicked => 5,
            RejectReason::Protocol => 6,
            RejectReason::ServerShutdown => 7,
            RejectReason::TimedOut => 8,
            RejectReason::RateLimited => 9,
        }
    }
    pub fn from_code(c: u8) -> RejectReason {
        match c {
            1 => RejectReason::VersionMismatch,
            2 => RejectReason::Full,
            3 => RejectReason::BadPassword,
            4 => RejectReason::Banned,
            5 => RejectReason::Kicked,
            7 => RejectReason::ServerShutdown,
            8 => RejectReason::TimedOut,
            9 => RejectReason::RateLimited,
            _ => RejectReason::Protocol,
        }
    }
    /// A sentence for the player.
    pub fn describe(self) -> &'static str {
        match self {
            RejectReason::VersionMismatch => "Version mismatch: the server runs a different version of Skyward.",
            RejectReason::Full => "The server is full.",
            RejectReason::BadPassword => "Wrong password.",
            RejectReason::Banned => "You are banned from this server.",
            RejectReason::Kicked => "You were kicked from the server.",
            RejectReason::Protocol => "The server did not understand the connection.",
            RejectReason::ServerShutdown => "The server shut down.",
            RejectReason::TimedOut => "Connection lost (timed out).",
            RejectReason::RateLimited => "Too many connection attempts — try again in a moment.",
        }
    }
}

/// Sent with the `Accept`: everything a client needs to start.
#[derive(Clone, Debug, PartialEq)]
pub struct Welcome {
    pub info: ServerInfo,
    pub motd: String,
    /// Snapshot rate, Hz.
    pub tick_hz: u8,
    pub world: WorldSync,
}

// ---------------------------------------------------------------- packets

pub mod kind {
    pub const DISCOVER: u8 = 1;
    pub const DISCOVER_REPLY: u8 = 2;
    pub const DETAILS: u8 = 3;
    pub const DETAILS_REPLY: u8 = 4;
    pub const HELLO: u8 = 5;
    pub const CHALLENGE: u8 = 6;
    pub const JOIN: u8 = 7;
    pub const ACCEPT: u8 = 8;
    pub const REJECT: u8 = 9;
    pub const DATA: u8 = 10;
    pub const DISCONNECT: u8 = 11;
}

/// Acknowledgement header of every `Data` packet (see `channel.rs`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AckHeader {
    pub seq: u16,
    /// Latest packet received from the peer, and the 32 before it as bits.
    pub ack: u16,
    pub ack_bits: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Packet {
    /// "Any Skyward servers here?" (broadcast / unicast, padded).
    Discover {
        proto: u16,
        nonce: u64,
    },
    /// The reply. `proto` / `version` / `game_port` lead (frozen layout);
    /// `info` is only decoded when the protocol matches.
    DiscoverReply {
        nonce: u64,
        proto: u16,
        version: String,
        game_port: u16,
        info: Option<ServerInfo>,
    },
    Details {
        proto: u16,
        nonce: u64,
    },
    DetailsReply {
        nonce: u64,
        details: ServerDetails,
    },
    /// Connection request: the server answers with a stateless cookie (the
    /// client proves it receives at its address before the server keeps state).
    Hello {
        proto: u16,
        nonce: u64,
    },
    Challenge {
        nonce: u64,
        cookie: u64,
    },
    Join {
        nonce: u64,
        cookie: u64,
        name: String,
        aircraft: String,
        version: String,
        proof: [u8; 32],
    },
    Accept {
        nonce: u64,
        player: u16,
        token: u64,
        welcome: Welcome,
    },
    /// Refusal or end of a connection (frozen layout).
    Reject {
        nonce: u64,
        reason: RejectReason,
        server_proto: u16,
        server_version: String,
        text: String,
    },
    /// Connected traffic: `token` identifies the session; `payload` = messages.
    Data {
        token: u64,
        ack: AckHeader,
        payload: Vec<u8>,
    },
    Disconnect {
        token: u64,
        text: String,
    },
}

impl Packet {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = Writer::new();
        w.bytes(&MAGIC);
        match self {
            Packet::Discover { proto, nonce } => {
                w.u8(kind::DISCOVER);
                w.u64(*nonce);
                w.u16(*proto);
                w.pad_to(MIN_REQUEST);
            }
            Packet::DiscoverReply { nonce, proto, version, game_port, info } => {
                w.u8(kind::DISCOVER_REPLY);
                w.u64(*nonce);
                w.u16(*proto);
                w.str(version, 32);
                w.u16(*game_port);
                if let Some(i) = info {
                    i.encode(&mut w);
                }
            }
            Packet::Details { proto, nonce } => {
                w.u8(kind::DETAILS);
                w.u64(*nonce);
                w.u16(*proto);
                w.pad_to(MIN_REQUEST);
            }
            Packet::DetailsReply { nonce, details } => {
                w.u8(kind::DETAILS_REPLY);
                w.u64(*nonce);
                details.encode(&mut w);
            }
            Packet::Hello { proto, nonce } => {
                w.u8(kind::HELLO);
                w.u64(*nonce);
                w.u16(*proto);
                w.pad_to(MIN_REQUEST);
            }
            Packet::Challenge { nonce, cookie } => {
                w.u8(kind::CHALLENGE);
                w.u64(*nonce);
                w.u64(*cookie);
            }
            Packet::Join { nonce, cookie, name, aircraft, version, proof } => {
                w.u8(kind::JOIN);
                w.u64(*nonce);
                w.u64(*cookie);
                w.str(name, MAX_NAME * 4);
                w.str(aircraft, MAX_AIRCRAFT_ID);
                w.str(version, 32);
                w.bytes(proof);
                w.pad_to(MIN_REQUEST);
            }
            Packet::Accept { nonce, player, token, welcome } => {
                w.u8(kind::ACCEPT);
                w.u64(*nonce);
                w.u16(*player);
                w.u64(*token);
                welcome.info.encode(&mut w);
                w.str(&welcome.motd, MAX_MOTD);
                w.u8(welcome.tick_hz);
                welcome.world.encode(&mut w);
            }
            Packet::Reject { nonce, reason, server_proto, server_version, text } => {
                w.u8(kind::REJECT);
                w.u64(*nonce);
                w.u8(reason.code());
                w.u16(*server_proto);
                w.str(server_version, 32);
                w.str(text, 200);
            }
            Packet::Data { token, ack, payload } => {
                w.u8(kind::DATA);
                w.u64(*token);
                w.u16(ack.seq);
                w.u16(ack.ack);
                w.u32(ack.ack_bits);
                w.bytes(payload);
            }
            Packet::Disconnect { token, text } => {
                w.u8(kind::DISCONNECT);
                w.u64(*token);
                w.str(text, 200);
            }
        }
        w.buf
    }

    pub fn decode(b: &[u8]) -> Result<Packet> {
        if b.len() > MAX_RECV {
            return Err(DecodeError::Invalid("packet size"));
        }
        let mut r = Reader::new(b);
        if r.take(4)? != MAGIC {
            return Err(DecodeError::Magic);
        }
        let k = r.u8()?;
        // Requests must arrive padded (no amplification through spoofed sources).
        if matches!(k, kind::DISCOVER | kind::DETAILS | kind::HELLO | kind::JOIN) && b.len() < MIN_REQUEST {
            return Err(DecodeError::Invalid("unpadded request"));
        }
        Ok(match k {
            kind::DISCOVER => Packet::Discover { nonce: r.u64()?, proto: r.u16()? },
            kind::DISCOVER_REPLY => {
                let nonce = r.u64()?;
                let proto = r.u16()?;
                let version = sanitize(&r.str(32)?, 32);
                let game_port = r.u16()?;
                let info = if proto == PROTOCOL { Some(ServerInfo::decode(&mut r)?) } else { None };
                Packet::DiscoverReply { nonce, proto, version, game_port, info }
            }
            kind::DETAILS => Packet::Details { nonce: r.u64()?, proto: r.u16()? },
            kind::DETAILS_REPLY => Packet::DetailsReply { nonce: r.u64()?, details: ServerDetails::decode(&mut r)? },
            kind::HELLO => Packet::Hello { nonce: r.u64()?, proto: r.u16()? },
            kind::CHALLENGE => Packet::Challenge { nonce: r.u64()?, cookie: r.u64()? },
            kind::JOIN => {
                let nonce = r.u64()?;
                let cookie = r.u64()?;
                let name = r.str(MAX_NAME * 4)?;
                let aircraft = r.str(MAX_AIRCRAFT_ID)?;
                let version = r.str(32)?;
                let mut proof = [0u8; 32];
                proof.copy_from_slice(r.take(32)?);
                Packet::Join { nonce, cookie, name, aircraft, version, proof }
            }
            kind::ACCEPT => {
                let nonce = r.u64()?;
                let player = r.u16()?;
                let token = r.u64()?;
                let info = ServerInfo::decode(&mut r)?;
                let motd = r.str(MAX_MOTD)?;
                let tick_hz = r.u8()?;
                let world = WorldSync::decode(&mut r)?;
                Packet::Accept { nonce, player, token, welcome: Welcome { info, motd, tick_hz: tick_hz.clamp(1, 60), world } }
            }
            kind::REJECT => Packet::Reject {
                nonce: r.u64()?,
                reason: RejectReason::from_code(r.u8()?),
                server_proto: r.u16()?,
                server_version: sanitize(&r.str(32)?, 32),
                text: sanitize(&r.str(200)?, 200),
            },
            kind::DATA => {
                let token = r.u64()?;
                let ack = AckHeader { seq: r.u16()?, ack: r.u16()?, ack_bits: r.u32()? };
                let payload = r.take(r.remaining())?.to_vec();
                Packet::Data { token, ack, payload }
            }
            kind::DISCONNECT => Packet::Disconnect { token: r.u64()?, text: sanitize(&r.str(200)?, 200) },
            k => return Err(DecodeError::Kind(k)),
        })
    }
}

// ---------------------------------------------------------------- messages

/// Why a player left (shown to the others).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeaveReason {
    Quit,
    TimedOut,
    Kicked,
    Banned,
}

impl LeaveReason {
    fn code(self) -> u8 {
        match self {
            LeaveReason::Quit => 0,
            LeaveReason::TimedOut => 1,
            LeaveReason::Kicked => 2,
            LeaveReason::Banned => 3,
        }
    }
    fn from_code(c: u8) -> LeaveReason {
        match c {
            1 => LeaveReason::TimedOut,
            2 => LeaveReason::Kicked,
            3 => LeaveReason::Banned,
            _ => LeaveReason::Quit,
        }
    }
    pub fn verb(self) -> &'static str {
        match self {
            LeaveReason::Quit => "left",
            LeaveReason::TimedOut => "timed out",
            LeaveReason::Kicked => "was kicked",
            LeaveReason::Banned => "was banned",
        }
    }
}

/// Things that happen to an aircraft (relayed reliably to everyone).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GameEvent {
    /// Crashed: 0 = survivable (gear-up, collapse) .. 2 = destroyed.
    Crash {
        severity: u8,
    },
    Respawn,
    /// A structural part separated (`damage::Part` index).
    Separation {
        part: u8,
    },
    /// Ejected / bailed out.
    Eject,
}

impl GameEvent {
    fn encode(&self, w: &mut Writer) {
        match *self {
            GameEvent::Crash { severity } => {
                w.u8(0);
                w.u8(severity);
            }
            GameEvent::Respawn => w.u8(1),
            GameEvent::Separation { part } => {
                w.u8(2);
                w.u8(part);
            }
            GameEvent::Eject => w.u8(3),
        }
    }
    fn decode(r: &mut Reader) -> Result<GameEvent> {
        Ok(match r.u8()? {
            0 => GameEvent::Crash { severity: r.u8()?.min(2) },
            1 => GameEvent::Respawn,
            2 => GameEvent::Separation { part: r.u8()?.min(15) },
            3 => GameEvent::Eject,
            _ => return Err(DecodeError::Invalid("event")),
        })
    }
}

/// A player as the others see them.
#[derive(Clone, Debug, PartialEq)]
pub struct PlayerInfo {
    pub id: u16,
    pub name: String,
    pub aircraft: String,
    pub spectating: bool,
}

/// Messages inside `Data` packets.
#[derive(Clone, Debug, PartialEq)]
pub enum Msg {
    // -- unreliable (latest wins)
    /// Client → server: my aircraft now.
    State(AcState),
    /// Server → client: other aircraft (several per tick if they don't fit).
    Snapshot {
        time: f64,
        entries: Vec<(u16, AcState)>,
    },
    /// Clock sync: client time out, server time back.
    Ping {
        t: f64,
    },
    Pong {
        t: f64,
        server: f64,
    },
    // -- reliable, ordered
    /// Server → client: a player joined or changed (aircraft, spectating).
    Player(PlayerInfo),
    /// Server → client: a player left.
    Left {
        id: u16,
        reason: LeaveReason,
    },
    /// Chat. Client → server: `from` ignored (the server stamps it);
    /// server → client: `from` = player id or `SERVER_ID`.
    Chat {
        from: u16,
        text: String,
    },
    /// Server → client: time and weather.
    World(WorldSync),
    /// Something happened to `from`'s aircraft.
    Event {
        from: u16,
        event: GameEvent,
    },
    /// Client → server: I changed aircraft / started or stopped spectating.
    SetAircraft {
        aircraft: String,
        spectating: bool,
    },
}

pub mod msg {
    pub const STATE: u8 = 1;
    pub const SNAPSHOT: u8 = 2;
    pub const PING: u8 = 3;
    pub const PONG: u8 = 4;
    pub const PLAYER: u8 = 64;
    pub const LEFT: u8 = 65;
    pub const CHAT: u8 = 66;
    pub const WORLD: u8 = 67;
    pub const EVENT: u8 = 68;
    pub const SET_AIRCRAFT: u8 = 69;
    /// Types ≥ 64 are reliable (carry a message id).
    pub fn reliable(t: u8) -> bool {
        t >= 64
    }
}

/// Snapshot entry size bound (for splitting snapshots into datagrams).
pub const MAX_STATE_BYTES: usize = 2 + 8 * 3 + 4 + 8 + 6 + 6 + 3 + 7 + 2 + 1 + 2 + 2 + 1 + 4 * 4 + 2 + 2 + 1 + 10;

impl Msg {
    pub fn type_id(&self) -> u8 {
        match self {
            Msg::State(_) => msg::STATE,
            Msg::Snapshot { .. } => msg::SNAPSHOT,
            Msg::Ping { .. } => msg::PING,
            Msg::Pong { .. } => msg::PONG,
            Msg::Player(_) => msg::PLAYER,
            Msg::Left { .. } => msg::LEFT,
            Msg::Chat { .. } => msg::CHAT,
            Msg::World(_) => msg::WORLD,
            Msg::Event { .. } => msg::EVENT,
            Msg::SetAircraft { .. } => msg::SET_AIRCRAFT,
        }
    }

    pub fn reliable(&self) -> bool {
        msg::reliable(self.type_id())
    }

    /// The body (without type / id / length framing).
    pub fn encode_body(&self) -> Vec<u8> {
        let mut w = Writer::new();
        match self {
            Msg::State(s) => s.encode(&mut w),
            Msg::Snapshot { time, entries } => {
                w.f64(*time);
                w.u8(entries.len().min(255) as u8);
                for (id, s) in entries.iter().take(255) {
                    w.u16(*id);
                    s.encode(&mut w);
                }
            }
            Msg::Ping { t } => w.f64(*t),
            Msg::Pong { t, server } => {
                w.f64(*t);
                w.f64(*server);
            }
            Msg::Player(p) => {
                w.u16(p.id);
                w.str(&p.name, MAX_NAME * 4);
                w.str(&p.aircraft, MAX_AIRCRAFT_ID);
                w.bool(p.spectating);
            }
            Msg::Left { id, reason } => {
                w.u16(*id);
                w.u8(reason.code());
            }
            Msg::Chat { from, text } => {
                w.u16(*from);
                w.str(clip(text, MAX_CHAT * 4), MAX_CHAT * 4);
            }
            Msg::World(ws) => ws.encode(&mut w),
            Msg::Event { from, event } => {
                w.u16(*from);
                event.encode(&mut w);
            }
            Msg::SetAircraft { aircraft, spectating } => {
                w.str(aircraft, MAX_AIRCRAFT_ID);
                w.bool(*spectating);
            }
        }
        w.buf
    }

    pub fn decode_body(t: u8, b: &[u8]) -> Result<Msg> {
        let mut r = Reader::new(b);
        let m = match t {
            msg::STATE => Msg::State(AcState::decode(&mut r)?),
            msg::SNAPSHOT => {
                let time = r.f64()?;
                let n = r.u8()? as usize;
                let mut entries = Vec::with_capacity(n.min(64));
                for _ in 0..n {
                    entries.push((r.u16()?, AcState::decode(&mut r)?));
                }
                Msg::Snapshot { time, entries }
            }
            msg::PING => Msg::Ping { t: r.f64()? },
            msg::PONG => Msg::Pong { t: r.f64()?, server: r.f64()? },
            msg::PLAYER => {
                Msg::Player(PlayerInfo { id: r.u16()?, name: sanitize(&r.str(MAX_NAME * 4)?, MAX_NAME), aircraft: aircraft_id(&r.str(MAX_AIRCRAFT_ID)?)?, spectating: r.bool()? })
            }
            msg::LEFT => Msg::Left { id: r.u16()?, reason: LeaveReason::from_code(r.u8()?) },
            msg::CHAT => Msg::Chat { from: r.u16()?, text: sanitize(&r.str(MAX_CHAT * 4)?, MAX_CHAT) },
            msg::WORLD => Msg::World(WorldSync::decode(&mut r)?),
            msg::EVENT => Msg::Event { from: r.u16()?, event: GameEvent::decode(&mut r)? },
            msg::SET_AIRCRAFT => Msg::SetAircraft { aircraft: aircraft_id(&r.str(MAX_AIRCRAFT_ID)?)?, spectating: r.bool()? },
            t => return Err(DecodeError::Kind(t)),
        };
        Ok(m)
    }
}

/// Aircraft ids are lowercase ASCII identifiers (`kestrel`, `meridian320`).
pub fn aircraft_id(s: &str) -> Result<String> {
    if s.is_empty() || s.len() > MAX_AIRCRAFT_ID || !s.bytes().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == b'_' || c == b'-') {
        return Err(DecodeError::Invalid("aircraft id"));
    }
    Ok(s.to_string())
}

/// Password proof: SHA-256(cookie ‖ password). The password never crosses
/// the wire, and a captured proof is useless for another handshake.
pub fn password_proof(cookie: u64, password: &str) -> [u8; 32] {
    let mut data = Vec::with_capacity(8 + password.len());
    data.extend_from_slice(&cookie.to_le_bytes());
    data.extend_from_slice(password.as_bytes());
    crate::sha256::digest(&data)
}
