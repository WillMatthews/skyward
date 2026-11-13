//! Asset loading abstraction.
//!
//! Core code asks for bytes by a path relative to the asset root
//! (e.g. `aircraft/kestrel/model.skac`). Bytes come from an in-memory store
//! first (filled by the platform layer, e.g. a wasm build that fetches the
//! files before starting), then — natively — from the file system: the
//! directory in `SKYWARD_ASSETS`, `assets/` next to the executable, `assets/`
//! in the working directory, and the source tree's `assets/`.

use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

fn store() -> &'static Mutex<HashMap<String, std::sync::Arc<Vec<u8>>>> {
    static S: OnceLock<Mutex<HashMap<String, std::sync::Arc<Vec<u8>>>>> = OnceLock::new();
    S.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Make `bytes` available under `path` (platform layers / tests).
#[allow(dead_code)]
pub fn insert(path: &str, bytes: Vec<u8>) {
    store().lock().unwrap().insert(path.to_string(), std::sync::Arc::new(bytes));
}

/// Remove a stored asset and return it (data that is used once, e.g. a
/// texture decoded ahead of time by the browser build).
#[allow(dead_code)]
pub fn take(path: &str) -> Option<std::sync::Arc<Vec<u8>>> {
    store().lock().unwrap().remove(path)
}

/// Read an asset by its path relative to the asset root.
pub fn read(path: &str) -> Result<std::sync::Arc<Vec<u8>>, String> {
    if let Some(b) = store().lock().unwrap().get(path) {
        return Ok(b.clone());
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let mut roots: Vec<std::path::PathBuf> = Vec::new();
        if let Ok(r) = std::env::var("SKYWARD_ASSETS") {
            roots.push(r.into());
        }
        if let Ok(exe) = std::env::current_exe() {
            if let Some(d) = exe.parent() {
                roots.push(d.join("assets"));
            }
        }
        roots.push("assets".into());
        roots.push(concat!(env!("CARGO_MANIFEST_DIR"), "/assets").into());
        for r in &roots {
            let p = r.join(path);
            if let Ok(b) = std::fs::read(&p) {
                return Ok(std::sync::Arc::new(b));
            }
        }
        Err(format!("asset not found: {path}"))
    }
    #[cfg(target_arch = "wasm32")]
    {
        Err(format!("asset not preloaded: {path}"))
    }
}
