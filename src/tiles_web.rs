//! Browser tile pool (wasm32). No threads share memory on GitHub Pages (no
//! COOP/COEP headers, so no SharedArrayBuffer), so tiles are built in Web
//! Workers instead: each runs its own instance of this same module
//! (web/worker.js), fetches the vector tile and the DEM tiles its build
//! needs, builds it ([`skyward_worker_build`]) and posts the finished tile
//! back as bytes (platform/wire.rs); the page only decodes them.
//!
//! Without workers (no module workers, or the page didn't hand over the
//! compiled module) the same work runs on the page: async downloads, builds
//! from `try_recv` as many per frame as the frame budget allows
//! (platform::web::may_work) — at least one. A build that read a DEM tile
//! still in flight is redone once it has arrived. Downloads are cached by
//! the browser's HTTP cache.

use super::{TILEJSON_URL, TileResult, gunzip_if_needed};
use crate::osm::TileId;
use crate::platform::web;
use crate::platform::wire;
use crate::scene::TileBuild;
use crate::ui::mapdata::MapVectors;
use std::cell::RefCell;
use std::collections::{HashSet, VecDeque};
use std::rc::Rc;
use std::sync::Arc;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use web_time::{Duration, Instant};

/// Concurrent tile downloads (page builds).
const FETCHES: usize = 6;
/// Downloaded tiles waiting to be built, at most (beyond the fetches).
const READY_MAX: usize = 4;
/// Jobs in flight per worker (downloads overlap the builds).
const PER_WORKER: usize = 3;

/// What a worker posts back: the tile, its map vectors, fetch / build ms, bytes downloaded.
type Posted = (TileId, (Result<TileBuild, String>, (Option<Arc<MapVectors>>, (f32, (f32, u64)))));

struct Fetched {
    id: TileId,
    bytes: Result<Vec<u8>, String>,
    net_bytes: usize,
    fetch_ms: f32,
    attempts: u32,
}

struct Worker {
    w: web_sys::Worker,
    ready: bool,
    /// Has its own airport database (runway flattening).
    db: bool,
    jobs: usize,
    _on_msg: Closure<dyn FnMut(web_sys::MessageEvent)>,
}

#[derive(Default)]
struct St {
    /// Pending jobs, best LAST (popped from the end).
    jobs: Vec<TileId>,
    /// Fetching, building, or waiting for DEM tiles.
    active: HashSet<TileId>,
    fetching: usize,
    ready: VecDeque<Fetched>,
    /// Finished tiles from the workers (encoded).
    done: VecDeque<Vec<u8>>,
    workers: Vec<Worker>,
    /// Offloaded jobs (platform::web::offload) waiting for their worker.
    calls: std::collections::HashMap<u32, web::Done>,
    next_call: u32,
    template: Option<String>,
    template_at: Option<Instant>,
    template_busy: bool,
    downloaded: u64,
    quit: bool,
}

pub struct Pool {
    st: Rc<RefCell<St>>,
}

impl Pool {
    pub fn new(workers: usize) -> Pool {
        let st = Rc::new(RefCell::new(St::default()));
        start_workers(&st, workers);
        if !st.borrow().workers.is_empty() {
            let weak = Rc::downgrade(&st);
            web::set_offload(Some(Box::new(move |kind, data, done| weak.upgrade().is_some_and(|st| offload(&st, kind, data, done)))));
        }
        Pool { st }
    }

    /// Replace the job queue (ordered best-first).
    pub fn set_jobs(&self, mut jobs: Vec<TileId>) {
        jobs.reverse();
        {
            let mut st = self.st.borrow_mut();
            jobs.retain(|j| !st.active.contains(j));
            st.jobs = jobs;
        }
        kick(&self.st);
    }

    pub fn in_flight(&self) -> HashSet<TileId> {
        self.st.borrow().active.clone()
    }

    pub fn queued(&self) -> usize {
        self.st.borrow().jobs.len()
    }

    pub fn downloaded_bytes(&self) -> u64 {
        self.st.borrow().downloaded
    }

    /// The next finished tile (decoded from a worker, or built here if the
    /// frame budget allows).
    pub fn try_recv(&self) -> Option<TileResult> {
        loop {
            if !web::may_work() {
                return None;
            }
            let posted = self.st.borrow_mut().done.pop_front();
            if let Some(b) = posted {
                web::note_work();
                let r = match wire::decode::<Posted>(&b) {
                    Ok((id, (build, (map, (fetch_ms, (build_ms, bytes)))))) => {
                        let mut st = self.st.borrow_mut();
                        st.active.remove(&id);
                        st.downloaded += bytes;
                        TileResult { id, build, map, fetch_ms, build_ms, from_cache: false, bytes: bytes as usize }
                    }
                    Err(e) => {
                        web::log(&format!("tiles: bad tile from a worker: {e}"));
                        continue;
                    }
                };
                kick(&self.st);
                return Some(r);
            }
            let f = self.st.borrow_mut().ready.pop_front()?;
            kick(&self.st);
            let t0 = Instant::now();
            let _ = crate::terrain::web::take_misses();
            let (build, map) = build_fetched(f.id, &f.bytes);
            web::note_work();
            let misses = crate::terrain::web::take_misses();
            if !misses.is_empty() && f.attempts < 3 && f.bytes.is_ok() {
                // Built over a DEM tile still loading: again once it is in.
                let st = self.st.clone();
                web::spawn_local(async move {
                    crate::terrain::web::ensure(&misses).await;
                    let mut st = st.borrow_mut();
                    if !st.quit {
                        st.ready.push_front(Fetched { attempts: f.attempts + 1, ..f });
                    }
                });
                continue;
            }
            self.st.borrow_mut().active.remove(&f.id);
            return Some(TileResult { id: f.id, build, map, fetch_ms: f.fetch_ms, build_ms: t0.elapsed().as_secs_f32() * 1000.0, from_cache: false, bytes: f.net_bytes });
        }
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        let mut st = self.st.borrow_mut();
        st.quit = true;
        st.jobs.clear();
        st.ready.clear();
        st.done.clear();
        for w in st.workers.drain(..) {
            w.w.terminate();
        }
    }
}

fn build_fetched(id: TileId, bytes: &Result<Vec<u8>, String>) -> (Result<TileBuild, String>, Option<Arc<MapVectors>>) {
    match bytes {
        Ok(_) if id.z == crate::terrain::HORIZON_ZOOM => (Ok(crate::terrain::build_horizon_tile(id)), None),
        Ok(b) => super::build(id, b),
        Err(e) => (Err(e.clone()), None),
    }
}

// ---------------------------------------------------------------- workers

/// Start `n` tile workers (at most the CPUs less one, at most 3) from the
/// compiled module the page left in `window.skywardModule`. None started:
/// the page builds the tiles itself.
fn start_workers(st: &Rc<RefCell<St>>, n: usize) {
    let Some(window) = web_sys::window() else { return };
    if web::query().iter().any(|(k, _)| k == "noworkers") {
        web::log("tiles: building on the page (?noworkers)");
        return;
    }
    let Ok(module) = js_sys::Reflect::get(&window, &JsValue::from_str("skywardModule")) else { return };
    if module.is_undefined() || module.is_null() {
        return;
    }
    let cpus = window.navigator().hardware_concurrency() as usize;
    let n = n.min(cpus.saturating_sub(1).max(1)).min(3);
    let opts = web_sys::WorkerOptions::new();
    opts.set_type(web_sys::WorkerType::Module);
    for i in 0..n {
        let w = match web_sys::Worker::new_with_options("worker.js", &opts) {
            Ok(w) => w,
            Err(e) => {
                web::log(&format!("tiles: no workers ({e:?}); building on the page"));
                return;
            }
        };
        let st2 = Rc::downgrade(st);
        let on_msg = Closure::<dyn FnMut(web_sys::MessageEvent)>::new(move |e: web_sys::MessageEvent| {
            let Some(st) = st2.upgrade() else { return };
            on_message(&st, i, e.data());
        });
        w.set_onmessage(Some(on_msg.as_ref().unchecked_ref()));
        let msg = js_sys::Object::new();
        let _ = js_sys::Reflect::set(&msg, &"type".into(), &"init".into());
        let _ = js_sys::Reflect::set(&msg, &"module".into(), &module);
        if w.post_message(&msg).is_err() {
            return;
        }
        st.borrow_mut().workers.push(Worker { w, ready: false, db: false, jobs: 0, _on_msg: on_msg });
    }
    web::log(&format!("tiles: {n} build workers"));
}

fn on_message(st: &Rc<RefCell<St>>, i: usize, data: JsValue) {
    let get = |k: &str| js_sys::Reflect::get(&data, &JsValue::from_str(k)).unwrap_or(JsValue::UNDEFINED);
    {
        let mut s = st.borrow_mut();
        if s.quit {
            return;
        }
        let Some(w) = s.workers.get_mut(i) else { return };
        match get("type").as_string().as_deref() {
            Some("ready") => w.ready = true,
            Some("db") => w.db = get("ok").as_bool().unwrap_or(false),
            Some("tile") => {
                w.jobs = w.jobs.saturating_sub(1);
                let b = js_sys::Uint8Array::new(&get("data")).to_vec();
                s.done.push_back(b);
            }
            Some("job") => {
                w.jobs = w.jobs.saturating_sub(1);
                let call = get("call").as_f64().unwrap_or(-1.0) as u32;
                let data = get("data");
                let r = (!data.is_undefined()).then(|| js_sys::Uint8Array::new(&data).to_vec());
                if let Some(done) = s.calls.remove(&call) {
                    drop(s);
                    done(r);
                    kick(st);
                    return;
                }
            }
            Some("error") => {
                web::log(&format!("tiles: worker {i}: {}", get("message").as_string().unwrap_or_default()));
                w.jobs = w.jobs.saturating_sub(1);
            }
            _ => {}
        }
    }
    kick(st);
}

/// Hand a job to the least busy ready worker (platform::web::offload).
fn offload(st: &Rc<RefCell<St>>, kind: &str, data: Vec<u8>, done: web::Done) -> bool {
    let mut s = st.borrow_mut();
    let s = &mut *s;
    if s.quit {
        return false;
    }
    let Some(w) = s.workers.iter_mut().filter(|w| w.ready).min_by_key(|w| w.jobs) else { return false };
    let call = s.next_call;
    s.next_call = s.next_call.wrapping_add(1);
    let msg = js_sys::Object::new();
    let set = |k: &str, v: JsValue| {
        let _ = js_sys::Reflect::set(&msg, &JsValue::from_str(k), &v);
    };
    set("type", "job".into());
    set("kind", kind.into());
    set("call", call.into());
    set("data", js_sys::Uint8Array::from(&data[..]).into());
    if w.w.post_message(&msg).is_err() {
        return false;
    }
    w.jobs += 1;
    s.calls.insert(call, done);
    true
}

/// Worker side: a small job from the page (platform::web::offload).
#[wasm_bindgen]
pub fn skyward_worker_job(kind: String, data: &[u8]) -> Option<js_sys::Uint8Array> {
    let out = match kind.as_str() {
        // Sky glow over cities (lights/gpu.rs): (sources, (eye, haze)) → table.
        "skyglow" => {
            let (src, (eye, haze)): (Vec<crate::lights::sky::GlowSource>, (glam::Vec3, f32)) = wire::decode(data).ok()?;
            wire::encode(&crate::lights::sky::skyglow(&src, eye, haze))
        }
        _ => return None,
    };
    Some(js_sys::Uint8Array::from(&out[..]))
}

/// Hand jobs to idle workers. Detailed tiles flatten runways from the
/// airport database: once the page has one, they go only to workers that
/// have loaded theirs.
fn dispatch(st: &Rc<RefCell<St>>) {
    let page_db = crate::terrain::has_airport_db();
    let mut need_template = false;
    {
        let mut s = st.borrow_mut();
        let s = &mut *s;
        for w in s.workers.iter_mut().filter(|w| w.ready) {
            while w.jobs < PER_WORKER {
                let pick = s.jobs.iter().rposition(|id| (id.is_far() || w.db || !page_db) && (id.z == crate::terrain::HORIZON_ZOOM || s.template.is_some()));
                let Some(k) = pick else {
                    need_template |= s.template.is_none() && !s.jobs.is_empty();
                    break;
                };
                let id = s.jobs.remove(k);
                let msg = js_sys::Object::new();
                let set = |k: &str, v: JsValue| {
                    let _ = js_sys::Reflect::set(&msg, &JsValue::from_str(k), &v);
                };
                set("type", "build".into());
                set("z", id.z.into());
                set("x", id.x.into());
                set("y", id.y.into());
                set("template", s.template.clone().unwrap_or_default().into());
                if w.w.post_message(&msg).is_err() {
                    s.jobs.push(id);
                    break;
                }
                s.active.insert(id);
                w.jobs += 1;
            }
        }
    }
    if need_template {
        let st2 = st.clone();
        web::spawn_local(async move {
            if template(&st2).await.is_some() {
                kick(&st2);
            }
        });
    }
}

/// Worker side, once: load the airport database (runway flattening).
#[wasm_bindgen]
pub async fn skyward_worker_init() -> bool {
    match crate::airports::web_download().await {
        Ok((list, bytes)) => {
            crate::terrain::set_airport_db(Arc::new(crate::airports::Db::new(list, bytes)));
            true
        }
        Err(e) => {
            web::log(&format!("tile worker: no airport database ({e})"));
            false
        }
    }
}

/// Worker side: download and build one tile, encoded for the page.
#[wasm_bindgen]
pub async fn skyward_worker_build(z: u32, x: u32, y: u32, template: String) -> js_sys::Uint8Array {
    let id = TileId { z, x, y };
    let t0 = Instant::now();
    let keys = crate::terrain::web::keys_for(id);
    for k in &keys {
        crate::terrain::web::start_load(*k);
    }
    let (bytes, net) = if id.z == crate::terrain::HORIZON_ZOOM {
        (Ok(Vec::new()), 0)
    } else if template.is_empty() {
        (Err("offline: no tile URL".to_string()), 0)
    } else {
        match fetch_url(&template, id).await {
            Ok((b, n)) => (Ok(b), n),
            Err(e) => (Err(e), 0),
        }
    };
    if bytes.is_ok() {
        crate::terrain::web::ensure(&keys).await;
    }
    let fetch_ms = t0.elapsed().as_secs_f32() * 1000.0;
    let mut build_ms = 0.0;
    let mut attempt = 0;
    let (build, map) = loop {
        let t = Instant::now();
        let _ = crate::terrain::web::take_misses();
        let r = build_fetched(id, &bytes);
        build_ms += t.elapsed().as_secs_f32() * 1000.0;
        let misses = crate::terrain::web::take_misses();
        if misses.is_empty() || attempt >= 3 || bytes.is_err() {
            break r;
        }
        attempt += 1;
        crate::terrain::web::ensure(&misses).await;
    };
    let posted: Posted = (id, (build, (map, (fetch_ms, (build_ms, net as u64)))));
    js_sys::Uint8Array::from(&wire::encode(&posted)[..])
}

// ---------------------------------------------------------------- page builds

/// Start work while there is room: worker jobs, or page downloads.
fn kick(st: &Rc<RefCell<St>>) {
    if !st.borrow().workers.is_empty() {
        dispatch(st);
        return;
    }
    loop {
        let id = {
            let mut s = st.borrow_mut();
            if s.quit || s.fetching >= FETCHES || s.fetching + s.ready.len() >= FETCHES + READY_MAX {
                return;
            }
            let Some(id) = s.jobs.pop() else { return };
            s.active.insert(id);
            s.fetching += 1;
            id
        };
        let st2 = st.clone();
        web::spawn_local(async move {
            let f = fetch_job(&st2, id).await;
            {
                let mut s = st2.borrow_mut();
                s.fetching -= 1;
                s.downloaded += f.net_bytes as u64;
                if s.quit {
                    return;
                }
                s.ready.push_back(f);
            }
            kick(&st2);
        });
    }
}

/// The tile URL template from OpenFreeMap's TileJSON (one request at a time;
/// retried every 30 s while offline).
async fn template(st: &Rc<RefCell<St>>) -> Option<String> {
    loop {
        {
            let mut s = st.borrow_mut();
            if let Some(t) = &s.template {
                return Some(t.clone());
            }
            if !s.template_busy {
                if s.template_at.is_some_and(|t| t.elapsed() < Duration::from_secs(30)) {
                    return None;
                }
                s.template_busy = true;
                break;
            }
        }
        web::sleep(Duration::from_millis(50)).await;
    }
    let r = fetch_template().await;
    let mut s = st.borrow_mut();
    s.template_busy = false;
    s.template_at = Some(Instant::now());
    match r {
        Ok(t) => s.template = Some(t.clone()),
        Err(e) => web::log(&format!("tiles: {e}")),
    }
    s.template.clone()
}

async fn fetch_template() -> Result<String, String> {
    let b = web::get(TILEJSON_URL, Duration::from_secs(20)).await?;
    let v: serde_json::Value = serde_json::from_slice(&b).map_err(|e| e.to_string())?;
    v.get("tiles").and_then(|t| t.get(0)).and_then(|t| t.as_str()).map(String::from).ok_or_else(|| "TileJSON has no tile URL".into())
}

async fn fetch_url(template: &str, id: TileId) -> Result<(Vec<u8>, usize), String> {
    let url = template.replace("{z}", &id.z.to_string()).replace("{x}", &id.x.to_string()).replace("{y}", &id.y.to_string());
    let r = web::fetch(&url, Duration::from_secs(25)).await?;
    match r.status {
        // No data here (open ocean etc.): an empty tile is a valid answer.
        204 | 404 => Ok((Vec::new(), 0)),
        200..=299 => {
            let n = r.body.len();
            Ok((gunzip_if_needed(r.body)?, n))
        }
        s => Err(format!("HTTP {s}")),
    }
}

async fn fetch_job(st: &Rc<RefCell<St>>, id: TileId) -> Fetched {
    let t0 = Instant::now();
    // The DEM tiles load alongside the vector tile.
    let keys = crate::terrain::web::keys_for(id);
    for k in &keys {
        crate::terrain::web::start_load(*k);
    }
    let (bytes, net_bytes) = if id.z == crate::terrain::HORIZON_ZOOM {
        (Ok(Vec::new()), 0)
    } else {
        match template(st).await {
            Some(t) => match fetch_url(&t, id).await {
                Ok((b, n)) => (Ok(b), n),
                Err(e) => (Err(e), 0),
            },
            None => (Err("offline: no tile URL".to_string()), 0),
        }
    };
    if bytes.is_ok() {
        crate::terrain::web::ensure(&keys).await;
    }
    Fetched { id, bytes, net_bytes, fetch_ms: t0.elapsed().as_secs_f32() * 1000.0, attempts: 0 }
}

thread_local! {
    static ANY: Rc<RefCell<St>> = Rc::new(RefCell::new(St::default()));
}

/// One tile at any zoom, decompressed (the menus' overview map).
pub async fn fetch_any_async(id: TileId) -> Result<Vec<u8>, String> {
    let st = ANY.with(|a| a.clone());
    let t = template(&st).await.ok_or("offline: no tile URL")?;
    fetch_url(&t, id).await.map(|r| r.0)
}
