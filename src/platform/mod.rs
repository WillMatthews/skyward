//! Platform layer: what differs between the native build and the browser
//! (wasm32 + WebGPU) build.
//!
//! The browser build has no threads (GitHub Pages cannot send the COOP/COEP
//! headers that `SharedArrayBuffer` needs), no file system and no blocking
//! I/O. Work that runs on worker threads natively runs on the page's main
//! thread instead: network requests as async tasks (`fetch`), CPU work
//! either deferred to the next frames under a per-frame time budget
//! ([`spawn`], the tile builder in tiles.rs) or done at once where it is
//! small. Settings live in `localStorage`; downloads are cached by the
//! browser's HTTP cache.

#[cfg(target_arch = "wasm32")]
pub mod web;
pub mod wire;

/// Run `f` in the background: natively on its own thread; in the browser on
/// the main thread, between frames (see `web::run_deferred`). Returns false
/// if it could not be started.
pub fn spawn(name: &str, f: impl FnOnce() + Send + 'static) -> bool {
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::thread::Builder::new().name(name.into()).spawn(f).is_ok()
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = name;
        web::defer(Box::new(f));
        true
    }
}

/// True in the browser build.
pub const WEB: bool = cfg!(target_arch = "wasm32");

/// Offline mode (Settings › Data & Storage): no downloads; cached data only.
static OFFLINE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn set_offline(on: bool) {
    OFFLINE.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// True when the player chose offline mode: network fetches fail fast.
pub fn offline() -> bool {
    OFFLINE.load(std::sync::atomic::Ordering::Relaxed)
}
