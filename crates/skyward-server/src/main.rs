//! `skyward-server` — the Skyward dedicated multiplayer server.
//!
//! Headless (no graphics, audio or game code: it links only `skyward-net`),
//! one UDP port for the game, the shared LAN discovery port and mDNS.
//! Configuration from flags and / or a TOML file; an interactive console
//! (`help`); structured logs on stderr; Ctrl-C / SIGTERM shut down cleanly
//! (players get "server shut down", LAN browsers drop the entry).

#[cfg(not(target_arch = "wasm32"))]
mod native;

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    std::process::exit(native::run());
}

#[cfg(target_arch = "wasm32")]
fn main() {
    // The browser build compiles the workspace; the server is native only.
}
