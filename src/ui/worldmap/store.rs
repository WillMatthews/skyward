//! World-map tile store: decoded tiles with their arrival time (cross-fade)
//! and last use (LRU under a memory cap), and the loader — a prioritised
//! request list rebuilt every frame (centre first; tiles that left the view
//! before a worker took them are dropped), served by a small thread pool
//! natively and by a few concurrent fetches + the tile Web Workers in the
//! browser. Fetches go through the tile disk cache / the Cache API.

use super::decode::MapTile;
use crate::osm::TileId;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};
use web_time::Instant;

/// Cross-fade time of a newly arrived tile (s).
pub const FADE: f64 = 0.22;
/// Retry a failed tile after this long (s).
const RETRY: f64 = 20.0;
/// Concurrent tile loads.
#[cfg(not(target_arch = "wasm32"))]
const WORKERS: usize = 4;
#[cfg(target_arch = "wasm32")]
const WEB_CONCURRENT: usize = 6;

struct Entry {
    tile: Arc<MapTile>,
    bytes: usize,
    born: f64,
    used: u64,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Default)]
struct QState {
    /// Waiting, most urgent first.
    jobs: Vec<TileId>,
    active: HashSet<TileId>,
}

#[cfg(not(target_arch = "wasm32"))]
struct Queue {
    st: std::sync::Mutex<QState>,
    cv: std::sync::Condvar,
}

type Msg = (TileId, Option<MapTile>);

pub struct Store {
    tiles: HashMap<TileId, Entry>,
    bytes: usize,
    /// Memory budget (bytes) for decoded tiles.
    pub cap: usize,
    frame: u64,
    epoch: Instant,
    failed: HashMap<TileId, f64>,
    /// Queued or loading (as of the last request round).
    pending: HashSet<TileId>,
    /// Bumped when the set of tiles changes (redraw).
    pub generation: u64,
    rx: Receiver<Msg>,
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    tx: Sender<Msg>,
    #[cfg(not(target_arch = "wasm32"))]
    q: Arc<Queue>,
    #[cfg(target_arch = "wasm32")]
    active: HashSet<TileId>,
}

impl Default for Store {
    fn default() -> Self {
        Self::new()
    }
}

impl Store {
    pub fn new() -> Store {
        let (tx, rx) = channel::<Msg>();
        #[cfg(not(target_arch = "wasm32"))]
        let q = {
            let q = Arc::new(Queue { st: std::sync::Mutex::new(QState::default()), cv: std::sync::Condvar::new() });
            for i in 0..WORKERS {
                let (q, tx) = (q.clone(), tx.clone());
                let _ = std::thread::Builder::new().name(format!("worldmap-{i}")).spawn(move || {
                    loop {
                        let id = {
                            let mut s = q.st.lock().unwrap();
                            loop {
                                if !s.jobs.is_empty() {
                                    let id = s.jobs.remove(0);
                                    s.active.insert(id);
                                    break id;
                                }
                                s = q.cv.wait(s).unwrap();
                            }
                        };
                        let t = crate::tiles::fetch_any(id).ok().map(|b| super::decode::decode(id, &b));
                        q.st.lock().unwrap().active.remove(&id);
                        if tx.send((id, t)).is_err() {
                            return;
                        }
                    }
                });
            }
            q
        };
        let cap = if crate::platform::WEB { 96 << 20 } else { 224 << 20 };
        Store {
            tiles: HashMap::new(),
            bytes: 0,
            cap,
            frame: 0,
            epoch: Instant::now(),
            failed: HashMap::new(),
            pending: HashSet::new(),
            generation: 0,
            rx,
            tx,
            #[cfg(not(target_arch = "wasm32"))]
            q,
            #[cfg(target_arch = "wasm32")]
            active: HashSet::new(),
        }
    }

    pub fn now(&self) -> f64 {
        self.epoch.elapsed().as_secs_f64()
    }

    /// Collect finished loads.
    pub fn poll(&mut self) {
        let now = self.now();
        while let Ok((id, t)) = self.rx.try_recv() {
            self.pending.remove(&id);
            #[cfg(target_arch = "wasm32")]
            self.active.remove(&id);
            match t {
                Some(t) => self.insert(id, t, now),
                None => {
                    self.failed.insert(id, now + RETRY);
                }
            }
        }
    }

    pub fn insert(&mut self, id: TileId, t: MapTile, now: f64) {
        let bytes = t.bytes();
        if let Some(old) = self.tiles.insert(id, Entry { tile: Arc::new(t), bytes, born: now, used: self.frame }) {
            self.bytes -= old.bytes;
        }
        self.bytes += bytes;
        self.generation += 1;
    }

    pub fn get(&self, id: TileId) -> Option<&Arc<MapTile>> {
        self.tiles.get(&id).map(|e| &e.tile)
    }

    /// Loaded tile's opacity (fading in), None if not loaded.
    pub fn alpha(&self, id: TileId, now: f64) -> Option<f32> {
        self.tiles.get(&id).map(|e| (((now - e.born) / FADE).clamp(0.0, 1.0)) as f32)
    }

    /// Mark a tile used this frame (kept by the LRU).
    pub fn touch(&mut self, id: TileId) {
        if let Some(e) = self.tiles.get_mut(&id) {
            e.used = self.frame;
        }
    }

    /// Loads in progress or queued.
    pub fn busy(&self) -> usize {
        self.pending.len()
    }

    pub fn bytes(&self) -> usize {
        self.bytes
    }

    pub fn len(&self) -> usize {
        self.tiles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }

    /// This frame's wanted tiles, most urgent first: replaces the queue
    /// (anything no longer wanted and not yet started is dropped).
    pub fn request(&mut self, want: &[TileId]) {
        let now = self.now();
        self.failed.retain(|_, t| *t > now);
        let mut seen = HashSet::new();
        let list: Vec<TileId> = want.iter().copied().filter(|id| !self.tiles.contains_key(id) && !self.failed.contains_key(id) && seen.insert(*id)).collect();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let mut s = self.q.st.lock().unwrap();
            s.jobs = list.into_iter().filter(|id| !s.active.contains(id)).collect();
            self.pending = s.jobs.iter().chain(s.active.iter()).copied().collect();
            if !s.jobs.is_empty() {
                self.q.cv.notify_all();
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            for id in list {
                if self.active.len() >= WEB_CONCURRENT {
                    break;
                }
                if self.active.insert(id) {
                    let tx = self.tx.clone();
                    crate::platform::web::spawn_local(async move {
                        let t = match crate::tiles::fetch_any_async(id).await {
                            Ok(b) => Some(super::web::decode_off_page(id, b).await),
                            Err(_) => None,
                        };
                        let _ = tx.send((id, t));
                    });
                }
            }
            self.pending = self.active.clone();
        }
    }

    /// End of frame: evict least-recently-used tiles over the budget (never
    /// one drawn this frame).
    pub fn end_frame(&mut self) {
        if self.bytes > self.cap {
            let mut old: Vec<(u64, TileId, usize)> = self.tiles.iter().filter(|(_, e)| e.used < self.frame).map(|(k, e)| (e.used, *k, e.bytes)).collect();
            old.sort_by_key(|o| (o.0, o.1));
            for (_, k, b) in old {
                if self.bytes <= self.cap {
                    break;
                }
                // Keep the world backdrop.
                if k.z == 0 {
                    continue;
                }
                self.tiles.remove(&k);
                self.bytes -= b;
                self.generation += 1;
            }
        }
        self.frame += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tile(n: usize) -> MapTile {
        MapTile { fills: vec![(0, vec![[0.0, 0.0]; n], vec![0; n])], ..Default::default() }
    }

    #[test]
    fn lru_evicts_unused_oldest_first() {
        let mut s = Store::new();
        let id = |x| TileId { z: 10, x, y: 1 };
        let one = tile(1000).bytes();
        s.cap = one * 3;
        for x in 0..3 {
            s.insert(id(x), tile(1000), 0.0);
            s.end_frame();
        }
        // tile 0 used again: tile 1 is now the oldest
        s.touch(id(0));
        s.end_frame();
        s.insert(id(3), tile(1000), 0.0);
        s.touch(id(3));
        s.end_frame();
        assert!(s.get(id(1)).is_none(), "LRU tile kept");
        assert!(s.get(id(0)).is_some() && s.get(id(2)).is_some() && s.get(id(3)).is_some());
        assert!(s.bytes() <= s.cap);
    }

    #[test]
    fn tiles_in_use_survive_over_budget() {
        let mut s = Store::new();
        s.cap = 1;
        let a = TileId { z: 12, x: 1, y: 1 };
        s.insert(a, tile(500), 0.0);
        s.touch(a);
        s.end_frame();
        // drawn last frame but not this one: now evictable
        s.end_frame();
        assert!(s.get(a).is_none());
        // the z0 backdrop is never evicted
        let w = TileId { z: 0, x: 0, y: 0 };
        s.insert(w, tile(500), 0.0);
        s.end_frame();
        s.end_frame();
        assert!(s.get(w).is_some());
    }

    #[test]
    fn fade_in() {
        let mut s = Store::new();
        let a = TileId { z: 5, x: 1, y: 1 };
        assert_eq!(s.alpha(a, 0.0), None);
        s.insert(a, tile(1), 10.0);
        assert_eq!(s.alpha(a, 10.0), Some(0.0));
        assert!((s.alpha(a, 10.0 + FADE * 0.5).unwrap() - 0.5).abs() < 1e-5);
        assert_eq!(s.alpha(a, 11.0), Some(1.0));
    }
}
