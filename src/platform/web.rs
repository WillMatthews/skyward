//! Browser glue (wasm32): fetch, timers, the main-thread work budget,
//! localStorage, lazily downloaded assets, the page's loading overlay.

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::future::Future;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;
use web_time::{Duration, Instant};

pub fn spawn_local(f: impl Future<Output = ()> + 'static) {
    wasm_bindgen_futures::spawn_local(f)
}

/// `console.log`.
pub fn log(s: &str) {
    web_sys_console(s);
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = log)]
    fn web_sys_console(s: &str);
    /// The page's loading overlay (index.html): status text and progress
    /// 0..1 (negative: indeterminate).
    #[wasm_bindgen(js_namespace = skyward, js_name = status, catch)]
    fn page_status(text: &str, progress: f64) -> Result<(), JsValue>;
    /// Hide the page's loading overlay (the app draws its own screens now).
    #[wasm_bindgen(js_namespace = skyward, js_name = ready, catch)]
    fn page_ready() -> Result<(), JsValue>;
    /// Show a fatal error on the page.
    #[wasm_bindgen(js_namespace = skyward, js_name = fatal, catch)]
    fn page_fatal(text: &str) -> Result<(), JsValue>;
}

pub fn status(text: &str, progress: f64) {
    let _ = page_status(text, progress);
}
pub fn ready() {
    let _ = page_ready();
}
pub fn fatal(text: &str) {
    log(text);
    let _ = page_fatal(text);
}

// ---------------------------------------------------------------- frame budget

thread_local! {
    static DEFERRED: RefCell<VecDeque<Box<dyn FnOnce()>>> = RefCell::new(VecDeque::new());
    static FRAME_T0: Cell<Option<Instant>> = const { Cell::new(None) };
    static WORKED: Cell<bool> = const { Cell::new(false) };
    static BUDGET_MS: Cell<f32> = const { Cell::new(6.0) };
}

/// Queue CPU work to run between frames ([`run_deferred`]).
pub fn defer(f: Box<dyn FnOnce()>) {
    DEFERRED.with(|d| d.borrow_mut().push_back(f));
}

/// Start of a frame: the per-frame budget for heavy steps restarts.
pub fn frame_begin() {
    FRAME_T0.set(Some(Instant::now()));
    WORKED.set(false);
}

/// Time budget (ms of the frame) for heavy main-thread steps: small while
/// flying (no hitches), larger behind a loading screen.
pub fn set_budget_ms(ms: f32) {
    BUDGET_MS.set(ms);
}

/// May a heavy step (a tile build, a decode) start now? At least one per
/// frame, more while the frame is still inside its budget.
pub fn may_work() -> bool {
    if !WORKED.get() {
        return true;
    }
    FRAME_T0.get().is_none_or(|t| t.elapsed().as_secs_f32() * 1000.0 < BUDGET_MS.get())
}

/// A heavy step ran.
pub fn note_work() {
    WORKED.set(true);
}

/// Run queued background work while the frame budget lasts.
pub fn run_deferred() {
    while may_work() {
        let Some(f) = DEFERRED.with(|d| d.borrow_mut().pop_front()) else { break };
        let t = Instant::now();
        f();
        note_work();
        let ms = t.elapsed().as_secs_f32() * 1000.0;
        if ms > 20.0 {
            log(&format!("slow frame: background job {ms:.1} ms"));
        }
    }
}

// ---------------------------------------------------------------- offload

/// A finished offloaded job: its result bytes, or None if it failed.
pub type Done = Box<dyn FnOnce(Option<Vec<u8>>)>;
type Offload = Box<dyn Fn(&str, Vec<u8>, Done) -> bool>;

thread_local! {
    static OFFLOAD: RefCell<Option<Offload>> = const { RefCell::new(None) };
}

/// The worker pool (tiles_web.rs) registers how to run jobs in its workers.
pub fn set_offload(f: Option<Offload>) {
    OFFLOAD.with(|o| *o.borrow_mut() = f);
}

/// Run job `kind` (see tiles_web.rs `skyward_worker_job`) in a Web Worker;
/// `done` gets the result. False if no worker can take it (run it here).
pub fn offload(kind: &str, payload: Vec<u8>, done: Done) -> bool {
    OFFLOAD.with(|o| o.borrow().as_ref().is_some_and(|f| f(kind, payload, done)))
}

// ---------------------------------------------------------------- timers

/// True inside a Web Worker (the tile builders), false on the page.
pub fn in_worker() -> bool {
    web_sys::window().is_none()
}

/// `setTimeout` on the page or in a worker.
fn set_timeout(f: &js_sys::Function, ms: i32) {
    let g = js_sys::global();
    if let Ok(st) = js_sys::Reflect::get(&g, &JsValue::from_str("setTimeout")).and_then(|f| f.dyn_into::<js_sys::Function>()) {
        let _ = st.call2(&g, f, &JsValue::from(ms));
    }
}

/// Resolve after `d` (setTimeout).
pub async fn sleep(d: Duration) {
    let ms = d.as_millis().min(i32::MAX as u128) as i32;
    let p = js_sys::Promise::new(&mut |resolve, _| set_timeout(&resolve, ms));
    let _ = JsFuture::from(p).await;
}

// ---------------------------------------------------------------- fetch

pub struct HttpResponse {
    pub status: u16,
    pub body: Vec<u8>,
}

fn js_err(e: JsValue) -> String {
    e.as_string().or_else(|| js_sys::JSON::stringify(&e).ok().and_then(|s| s.as_string())).unwrap_or_else(|| format!("{e:?}"))
}

/// Start an HTTP GET through the browser (CORS) now; await the result.
fn begin(url: &str, timeout: Duration) -> Result<JsFuture, String> {
    let init = web_sys::RequestInit::new();
    init.set_method("GET");
    init.set_mode(web_sys::RequestMode::Cors);
    if let Ok(a) = web_sys::AbortController::new() {
        init.set_signal(Some(&a.signal()));
        let cb = Closure::once_into_js(move || a.abort());
        set_timeout(cb.unchecked_ref(), timeout.as_millis().min(i32::MAX as u128) as i32);
    }
    let req = web_sys::Request::new_with_str_and_init(url, &init).map_err(js_err)?;
    // `fetch` of the page or of a worker.
    let g = js_sys::global();
    let fetch = js_sys::Reflect::get(&g, &JsValue::from_str("fetch")).map_err(js_err)?.dyn_into::<js_sys::Function>().map_err(js_err)?;
    let p = fetch.call1(&g, &req).map_err(js_err)?.dyn_into::<js_sys::Promise>().map_err(js_err)?;
    Ok(JsFuture::from(p))
}

async fn finish(url: &str, f: JsFuture) -> Result<HttpResponse, String> {
    let resp: web_sys::Response = f.await.map_err(|e| format!("{url}: {}", js_err(e)))?.dyn_into().map_err(js_err)?;
    let status = resp.status();
    let buf = JsFuture::from(resp.array_buffer().map_err(js_err)?).await.map_err(js_err)?;
    let body = js_sys::Uint8Array::new(&buf).to_vec();
    Ok(HttpResponse { status, body })
}

/// HTTP GET through the browser (CORS). Any status is returned; network /
/// CORS failures and timeouts are errors.
pub async fn fetch(url: &str, timeout: Duration) -> Result<HttpResponse, String> {
    let f = begin(url, timeout)?;
    finish(url, f).await
}

/// GET that must succeed (2xx).
pub async fn get(url: &str, timeout: Duration) -> Result<Vec<u8>, String> {
    let r = fetch(url, timeout).await?;
    if !(200..300).contains(&r.status) {
        return Err(format!("HTTP {} for {url}", r.status));
    }
    Ok(r.body)
}

// ---------------------------------------------------------------- page

/// `localStorage[key]`.
pub fn storage_get(key: &str) -> Option<String> {
    web_sys::window()?.local_storage().ok()??.get_item(key).ok()?
}

pub fn storage_set(key: &str, value: &str) {
    if let Some(s) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
        let _ = s.set_item(key, value);
    }
}

/// The page URL's query parameters (`?airport=EGLC&aircraft=kestrel`).
pub fn query() -> Vec<(String, String)> {
    let Some(search) = web_sys::window().and_then(|w| w.location().search().ok()) else { return Vec::new() };
    search
        .trim_start_matches('?')
        .split('&')
        .filter(|s| !s.is_empty())
        .map(|kv| {
            let (k, v) = kv.split_once('=').unwrap_or((kv, ""));
            let dec = |s: &str| js_sys::decode_uri_component(&s.replace('+', " ")).ok().and_then(|s| s.as_string()).unwrap_or_else(|| s.to_string());
            (dec(k), dec(v))
        })
        .collect()
}

/// Open a page of the site in a new tab.
pub fn open_url(url: &str) {
    if let Some(w) = web_sys::window() {
        let _ = w.open_with_url_and_target(url, "_blank");
    }
}

pub fn device_pixel_ratio() -> f64 {
    web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0)
}

// ---------------------------------------------------------------- assets

/// Assets are files under `assets/` next to the page, listed in
/// `assets/manifest.json` ({"files": {"path": bytes, ...}}, written by
/// scripts/web-build.sh). Fonts, UI sounds and engine samples are compiled
/// into the module; aircraft are downloaded when chosen.
#[derive(Default)]
struct AssetState {
    manifest: Option<Vec<(String, u64)>>,
    /// Other files the build shipped (e.g. `data/ourairports.tsv.gz`).
    extras: Vec<String>,
    /// Asset directories fully loaded / being loaded.
    loaded: std::collections::HashSet<String>,
    loading: std::collections::HashMap<String, (u64, u64)>,
    failed: std::collections::HashMap<String, String>,
}

thread_local! {
    static ASSETS: RefCell<AssetState> = RefCell::new(AssetState::default());
}

/// Fetch the asset manifest (once).
pub async fn load_manifest() -> Result<(), String> {
    if ASSETS.with(|a| a.borrow().manifest.is_some()) {
        return Ok(());
    }
    let b = get("assets/manifest.json", Duration::from_secs(30)).await?;
    let v: serde_json::Value = serde_json::from_slice(&b).map_err(|e| e.to_string())?;
    let files = v.get("files").and_then(|f| f.as_object()).ok_or("manifest has no files")?;
    let list = files.iter().map(|(k, v)| (k.clone(), v.as_u64().unwrap_or(0))).collect();
    let extras = v.get("extras").and_then(|e| e.as_array()).map(|e| e.iter().filter_map(|x| x.as_str().map(String::from)).collect()).unwrap_or_default();
    ASSETS.with(|a| {
        let mut a = a.borrow_mut();
        a.manifest = Some(list);
        a.extras = extras;
    });
    Ok(())
}

/// Did the build ship this file (manifest "extras")?
pub fn has_extra(path: &str) -> bool {
    ASSETS.with(|a| a.borrow().extras.iter().any(|e| e == path))
}

/// State of an asset directory (e.g. `aircraft/kestrel/`).
pub enum DirState {
    Ready,
    /// (bytes done, bytes total)
    Loading(u64, u64),
    Failed(String),
    Absent,
}

pub fn dir_state(dir: &str) -> DirState {
    ASSETS.with(|a| {
        let a = a.borrow();
        if a.loaded.contains(dir) {
            DirState::Ready
        } else if let Some(e) = a.failed.get(dir) {
            DirState::Failed(e.clone())
        } else if let Some(&(d, t)) = a.loading.get(dir) {
            DirState::Loading(d, t)
        } else {
            DirState::Absent
        }
    })
}

/// Start downloading every file of an asset directory into the asset store
/// (no-op if loaded or loading). Poll with [`dir_state`].
pub fn preload_dir(dir: &str) {
    let dir = dir.to_string();
    let start = ASSETS.with(|a| {
        let mut a = a.borrow_mut();
        if a.loaded.contains(&dir) || a.loading.contains_key(&dir) {
            return false;
        }
        a.failed.remove(&dir);
        a.loading.insert(dir.clone(), (0, 0));
        true
    });
    if !start {
        return;
    }
    spawn_local(async move {
        let r = preload(&dir).await;
        ASSETS.with(|a| {
            let mut a = a.borrow_mut();
            a.loading.remove(&dir);
            match r {
                Ok(()) => {
                    a.loaded.insert(dir.clone());
                }
                Err(e) => {
                    log(&format!("assets: {dir}: {e}"));
                    a.failed.insert(dir.clone(), e);
                }
            }
        });
    });
}

async fn preload(dir: &str) -> Result<(), String> {
    load_manifest().await?;
    let files: Vec<(String, u64)> = ASSETS.with(|a| a.borrow().manifest.as_ref().map(|m| m.iter().filter(|(p, _)| p.starts_with(dir)).cloned().collect()).unwrap_or_default());
    if files.is_empty() {
        return Err(format!("no files under {dir}"));
    }
    let total: u64 = files.iter().map(|f| f.1).sum();
    ASSETS.with(|a| a.borrow_mut().loading.insert(dir.to_string(), (0, total)));
    // A few requests at a time.
    let mut done = 0u64;
    for chunk in files.chunks(6) {
        // Started together (the browser runs them in parallel), awaited in turn.
        let mut started = Vec::new();
        for (p, _) in chunk {
            let url = format!("assets/{p}");
            let f = begin(&url, Duration::from_secs(120))?;
            started.push((p.clone(), url, f));
        }
        for (p, url, f) in started {
            let r = finish(&url, f).await?;
            if !(200..300).contains(&r.status) {
                return Err(format!("HTTP {} for {url}", r.status));
            }
            done += r.body.len() as u64;
            crate::assets::insert(&p, r.body);
            ASSETS.with(|a| a.borrow_mut().loading.insert(dir.to_string(), (done, total)));
        }
    }
    Ok(())
}

/// Install the panic hook (panics → console.error + the page's error box).
pub fn init() {
    std::panic::set_hook(Box::new(|info| {
        console_error_panic_hook::hook(info);
        let _ = page_fatal(&format!("Skyward crashed: {info}"));
    }));
}

/// Publish a line of diagnostics as `window.skywardStats` (read by test
/// harnesses and the console).
pub fn publish_stats(s: &str) {
    if let Some(w) = web_sys::window() {
        let _ = js_sys::Reflect::set(&w, &JsValue::from_str("skywardStats"), &JsValue::from_str(s));
    }
}

/// Size the 3D scene renders at for an output of `w`×`h` (the tonemap pass
/// scales it up): at most ~1.6 Mpixel by default, or `?scale=0.5..1` from
/// the page URL.
pub fn scene_size(w: u32, h: u32) -> (u32, u32) {
    thread_local! {
        static FIXED: Option<f32> = query().iter().find(|(k, _)| k == "scale").and_then(|(_, v)| v.parse::<f32>().ok()).map(|s| s.clamp(0.25, 1.0));
    }
    let s = FIXED.with(|f| *f).unwrap_or_else(|| (1.6e6 / (w as f32 * h as f32).max(1.0)).sqrt().min(1.0));
    (((w as f32 * s).round() as u32).max(1), ((h as f32 * s).round() as u32).max(1))
}
