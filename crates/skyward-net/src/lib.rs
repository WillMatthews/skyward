//! Skyward multiplayer networking, shared by the game and `skyward-server`.
//!
//! * [`proto`] — the wire protocol (packets, messages, aircraft state).
//! * [`channel`] — the reliability layer over UDP (acks, resends, ordering).
//! * [`clock`] — client ↔ server clock synchronisation.
//! * [`interp`] — snapshot interpolation / extrapolation of remote aircraft.
//! * [`client`] / [`server`] — the two ends of a connection, sans-IO.
//! * [`mdns`], [`discovery`], [`ifaces`] — LAN discovery (broadcast + mDNS).
//! * [`udp`] — a client on a real socket; [`link`] — a simulated lossy link.
//!
//! Why our own small layer rather than QUIC (`quinn`) or `renet`: the game
//! needs exactly two delivery classes — latest-wins state at 30 Hz and a
//! trickle of ordered reliable events — which a 16-bit sequence / ack-bits
//! scheme gives in a few hundred lines with no async runtime, no TLS
//! certificates on a LAN box, identical behaviour on every OS, a dedicated
//! server that builds in seconds, and protocol / interpolation code that
//! also compiles for the browser build. Everything here is sans-IO and
//! deterministic, so loss / latency / 32-client behaviour is unit-tested.

pub mod channel;
pub mod client;
pub mod clock;
pub mod codec;
pub mod interp;
pub mod link;
pub mod mdns;
pub mod proto;
pub mod server;
pub mod sha256;

#[cfg(not(target_arch = "wasm32"))]
pub mod discovery;
#[cfg(not(target_arch = "wasm32"))]
pub mod ifaces;
#[cfg(not(target_arch = "wasm32"))]
pub mod udp;

pub use proto::{AcState, DEFAULT_PORT, DISCOVERY_PORT, PROTOCOL};

/// Version string sent in the handshake (the game and the server share it).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
