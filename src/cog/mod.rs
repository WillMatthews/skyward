//! Cloud-Optimised GeoTIFF plumbing shared by the raster layers (land cover,
//! canopy height, Copernicus DEM): the in-house TIFF reader ([`tiff`]), HTTP
//! range reads with a polite back-off and an atomic disk cache ([`net`],
//! native), and a keyed load-once LRU ([`Store`]).

pub mod tiff;

/// Range reads + disk cache (native).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) mod net {
    use std::io::Read;
    use std::path::{Path, PathBuf};
    use std::sync::{LazyLock, Mutex};
    use web_time::{Duration, Instant};

    static FAILED_AT: Mutex<Option<Instant>> = Mutex::new(None);
    const BACKOFF: Duration = Duration::from_secs(60);

    const USER_AGENT: &str = "skyward-flightsim/0.2";

    static AGENT: LazyLock<ureq::Agent> =
        LazyLock::new(|| ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(40)).user_agent(USER_AGENT).build());

    /// Bytes `start .. start + len` of `url`; `Ok(None)` if the object does
    /// not exist (403 / 404). Fails fast in offline mode and for a minute
    /// after a network failure.
    pub fn range(url: &str, start: u64, len: u64) -> Result<Option<Vec<u8>>, String> {
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        // After a network failure, fail fast for a while: every tile build
        // would otherwise wait out its own connect timeout.
        if FAILED_AT.lock().unwrap().is_some_and(|t| t.elapsed() < BACKOFF) {
            return Err("network unavailable (backing off)".into());
        }
        if len == 0 {
            return Ok(Some(Vec::new()));
        }
        let r = AGENT.get(url).set("Range", &format!("bytes={}-{}", start, start + len - 1)).call();
        if let Err(ureq::Error::Transport(_)) = &r {
            *FAILED_AT.lock().unwrap() = Some(Instant::now());
        }
        match r {
            Ok(r) => {
                let status = r.status();
                let mut b = Vec::with_capacity(len.min(64 << 20) as usize);
                r.into_reader().take(len.min(256 << 20)).read_to_end(&mut b).map_err(|e| e.to_string())?;
                // A server ignoring the range sends the whole object from 0.
                if status == 200 && start > 0 {
                    return Err("server ignored the range request".into());
                }
                b.truncate(len as usize);
                Ok(Some(b))
            }
            // No such object (S3 answers 404 in listable buckets, 403 otherwise).
            Err(ureq::Error::Status(403 | 404, _)) => Ok(None),
            // Range past the end (short files): nothing there.
            Err(ureq::Error::Status(416, _)) => Ok(Some(Vec::new())),
            // Throttled / server trouble: back off like a network failure.
            Err(ureq::Error::Status(s @ (429 | 500..=599), _)) => {
                *FAILED_AT.lock().unwrap() = Some(Instant::now());
                Err(format!("HTTP {s}"))
            }
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn read(path: &Path) -> Option<Vec<u8>> {
        std::fs::read(path).ok()
    }

    /// Write atomically (temp file + rename): readers never see half a file.
    pub fn write(path: &Path, bytes: &[u8]) {
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let tmp = path.with_extension(format!("tmp{}", std::process::id()));
        if std::fs::write(&tmp, bytes).is_ok() {
            let _ = std::fs::rename(&tmp, path);
        }
    }

    /// Delete the oldest files under `root` until it is within 80 % of `limit` bytes.
    pub fn prune(root: PathBuf, limit: u64) {
        let mut files = Vec::new();
        let mut total = 0u64;
        let mut stack = vec![root];
        while let Some(d) = stack.pop() {
            let Ok(rd) = std::fs::read_dir(&d) else { continue };
            for e in rd.flatten() {
                let Ok(md) = e.metadata() else { continue };
                if md.is_dir() {
                    stack.push(e.path());
                } else {
                    total += md.len();
                    files.push((md.modified().unwrap_or(std::time::SystemTime::UNIX_EPOCH), md.len(), e.path()));
                }
            }
        }
        if total <= limit {
            return;
        }
        files.sort();
        for (_, len, p) in files {
            if total <= limit * 8 / 10 {
                break;
            }
            if std::fs::remove_file(&p).is_ok() {
                total -= len;
            }
        }
    }

    /// Parse a TIFF header cached at `path`, fetching (more of) the file's
    /// first bytes from `url` as needed. `Ok(None)`: the file does not exist
    /// (remembered on disk as `none`).
    pub fn header(url: &str, path: &Path, none: &Path, what: &str) -> Result<Option<(bool, Vec<super::tiff::Ifd>)>, String> {
        use super::tiff;
        if none.exists() {
            return Ok(None);
        }
        let mut bytes = read(path).unwrap_or_default();
        let mut fetched = false;
        loop {
            match tiff::parse(&bytes) {
                Ok(tiff::Parse::Ok { big_endian, ifds }) => {
                    if fetched {
                        write(path, &bytes);
                    }
                    return Ok(Some((big_endian, ifds)));
                }
                Ok(tiff::Parse::Need(n)) => {
                    // COG headers are a few kB; never more than a few MB.
                    if n > 8 << 20 {
                        return Err(format!("{what}: header too large"));
                    }
                    let want = n.max(64 << 10).max(bytes.len() as u64 * 2);
                    match range(url, 0, want)? {
                        Some(b) if b.len() as u64 > bytes.len() as u64 => bytes = b,
                        Some(_) => return Err(format!("{what}: truncated header")),
                        None => {
                            write(none, b"");
                            return Ok(None);
                        }
                    }
                    fetched = true;
                }
                Err(e) => return Err(format!("{what}: {e}")),
            }
        }
    }
}

/// A keyed, shared, LRU-bounded cache of loaded things: the first caller
/// loads, concurrent callers for the same key wait for it, failures are
/// remembered for a while (no hammering a dead server).
pub(crate) struct Store<K, V> {
    map: std::sync::Mutex<(std::collections::HashMap<K, Slot<V>>, u64)>,
    cv: std::sync::Condvar,
    cap: usize,
    /// Printed (once) with the first load failure.
    what: &'static str,
    warned: std::sync::atomic::AtomicBool,
}

enum Slot<V> {
    Loading,
    Ready(Option<std::sync::Arc<V>>, u64),
    Failed(web_time::Instant),
}

/// Retry a failed load after this long.
const RETRY: web_time::Duration = web_time::Duration::from_secs(60);

impl<K: std::hash::Hash + Eq + Clone, V> Store<K, V> {
    /// `what`: the first failure is logged as "`what`: error".
    pub fn new(cap: usize, what: &'static str) -> Self {
        Store { map: std::sync::Mutex::new((std::collections::HashMap::new(), 0)), cv: std::sync::Condvar::new(), cap, what, warned: std::sync::atomic::AtomicBool::new(false) }
    }

    /// The value for `k`, loading it with `load` if needed. `Ok(None)` from the
    /// loader means "known absent" (kept like a value); `Err` is retried later.
    pub fn get(&self, k: &K, load: impl FnOnce() -> Result<Option<V>, String>) -> Option<std::sync::Arc<V>> {
        let mut g = self.map.lock().unwrap();
        loop {
            g.1 += 1;
            let tick = g.1;
            match g.0.get_mut(k) {
                Some(Slot::Ready(v, used)) => {
                    *used = tick;
                    return v.clone();
                }
                Some(Slot::Loading) => {
                    g = self.cv.wait(g).unwrap();
                    continue;
                }
                Some(Slot::Failed(t)) if t.elapsed() < RETRY => return None,
                _ => {}
            }
            break;
        }
        g.0.insert(k.clone(), Slot::Loading);
        drop(g);
        let r = load();
        let mut g = self.map.lock().unwrap();
        g.1 += 1;
        let tick = g.1;
        let out = match r {
            Ok(v) => {
                let v = v.map(std::sync::Arc::new);
                g.0.insert(k.clone(), Slot::Ready(v.clone(), tick));
                v
            }
            Err(e) => {
                if !self.warned.swap(true, std::sync::atomic::Ordering::Relaxed) {
                    eprintln!("{}: {e}", self.what);
                }
                g.0.insert(k.clone(), Slot::Failed(web_time::Instant::now()));
                None
            }
        };
        // Evict the least recently used in batches (amortised: the sort is
        // over the whole map).
        if g.0.len() > self.cap + (self.cap / 4).max(1) {
            let mut ages: Vec<(u64, K)> = g.0.iter().filter_map(|(k, s)| if let Slot::Ready(_, u) = s { Some((*u, k.clone())) } else { None }).collect();
            let excess = ages.len().saturating_sub(self.cap);
            if excess > 0 && excess < ages.len() {
                ages.select_nth_unstable_by_key(excess, |a| a.0);
            }
            ages.truncate(excess);
            for (_, k) in ages {
                g.0.remove(&k);
            }
        }
        self.cv.notify_all();
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn store_loads_once_and_remembers_absence() {
        let s: Store<u32, u32> = Store::new(2, "test");
        let mut calls = 0;
        assert_eq!(*s.get(&1, || Ok(Some(5))).unwrap(), 5);
        assert_eq!(*s.get(&1, || panic!("cached")).unwrap(), 5);
        assert!(s.get(&2, || Ok(None)).is_none());
        assert!(s.get(&2, || panic!("absence is cached")).is_none());
        assert!(s.get(&3, || Err("down".into())).is_none());
        // Failures are not retried at once.
        assert!(
            s.get(&3, || {
                calls += 1;
                Ok(Some(1))
            })
            .is_none()
        );
        assert_eq!(calls, 0);
        // LRU: three ready entries with a cap of two evicts the oldest.
        s.get(&4, || Ok(Some(4)));
        assert_eq!(*s.get(&1, || Ok(Some(9))).unwrap(), 9);
    }
}
