//! Fetching the feed: a polite poller (one request in flight, adaptive
//! interval, exponential back-off on errors, `Retry-After` on 429) and, on
//! the desktop, a worker thread that does the HTTP and the JSON parsing so
//! the frame never waits on the network.
//!
//! The browser build cannot use the API: adsb.lol does not send CORS headers
//! (checked: no `Access-Control-Allow-Origin` on GET, 405 on preflight), so
//! live traffic is disabled there with a message ([`WEB_MESSAGE`]).

#[cfg(not(target_arch = "wasm32"))]
use super::feed::{self, Snapshot};

/// Shown instead of live traffic in the browser build.
pub const WEB_MESSAGE: &str = "Live traffic needs the desktop version — the adsb.lol API does not allow requests from web pages (no CORS).";

/// Seconds between polls with traffic around / with none.
pub const POLL_BUSY: f64 = 5.0;
pub const POLL_QUIET: f64 = 10.0;
/// Longest back-off after repeated failures, s.
pub const BACKOFF_MAX: f64 = 120.0;

/// What went wrong with a fetch.
#[derive(Clone, Debug, PartialEq)]
pub enum Failure {
    /// HTTP 429 (or 503) with the server's Retry-After (s), if any.
    RateLimited(Option<f64>),
    /// Network unreachable, timeout, DNS.
    Offline(String),
    /// HTTP error or unreadable response.
    Bad(String),
}

/// When to ask next.
#[derive(Clone, Debug)]
pub struct Poller {
    pub in_flight: bool,
    pub next_at: f64,
    pub failures: u32,
    pub last_ok: Option<f64>,
    /// Seconds between polls when all is well (adapts to the traffic).
    pub interval: f64,
}

impl Default for Poller {
    fn default() -> Self {
        Poller { in_flight: false, next_at: 0.0, failures: 0, last_ok: None, interval: POLL_BUSY }
    }
}

impl Poller {
    /// Time for a request (and none in flight)?
    pub fn due(&self, now: f64) -> bool {
        !self.in_flight && now >= self.next_at
    }

    pub fn sent(&mut self) {
        self.in_flight = true;
    }

    /// A response arrived: `Ok(aircraft near the player)` or a failure.
    pub fn done(&mut self, now: f64, r: Result<usize, Failure>) {
        self.in_flight = false;
        match r {
            Ok(near) => {
                self.failures = 0;
                self.last_ok = Some(now);
                // Busy skies: every 5 s; empty: every 10 s.
                self.interval = if near > 0 { POLL_BUSY } else { POLL_QUIET };
                self.next_at = now + self.interval;
            }
            Err(f) => {
                self.failures += 1;
                let backoff = (POLL_QUIET * 2f64.powi(self.failures as i32 - 1)).min(BACKOFF_MAX);
                let wait = match f {
                    Failure::RateLimited(Some(s)) => s.clamp(5.0, 600.0).max(backoff),
                    Failure::RateLimited(None) => backoff.max(60.0),
                    _ => backoff,
                };
                self.next_at = now + wait;
            }
        }
    }
}

/// Where to ask: the player's position moved ahead along the track (so the
/// circle covers more of where the aircraft is going), and the radius (NM).
pub fn query_centre(lat: f64, lon: f64, track_deg: f32, gs_ms: f32, radius_nm: f32) -> (f64, f64) {
    let ahead_m = (gs_ms * 120.0).min(radius_nm * 1852.0 * 0.35) as f64;
    if ahead_m < 500.0 {
        return (lat, lon);
    }
    let t = (track_deg as f64).to_radians();
    let dlat = ahead_m * t.cos() / 111_320.0;
    let dlon = ahead_m * t.sin() / (111_320.0 * lat.to_radians().cos().max(0.05));
    (lat + dlat, (lon + dlon + 540.0).rem_euclid(360.0) - 180.0)
}

/// The HTTP side (desktop): requests in, parsed snapshots out.
#[cfg(not(target_arch = "wasm32"))]
pub struct Worker {
    tx: std::sync::mpsc::Sender<String>,
    rx: std::sync::mpsc::Receiver<Result<Snapshot, Failure>>,
}

#[cfg(not(target_arch = "wasm32"))]
impl Worker {
    pub fn start() -> Option<Worker> {
        use std::sync::mpsc;
        let (tx, jobs) = mpsc::channel::<String>();
        let (out, rx) = mpsc::channel();
        std::thread::Builder::new()
            .name("adsb".into())
            .spawn(move || {
                let agent = ureq::AgentBuilder::new()
                    .timeout_connect(std::time::Duration::from_secs(6))
                    .timeout(std::time::Duration::from_secs(12))
                    .user_agent(&feed::user_agent())
                    // No cookies, no identifying headers.
                    .build();
                while let Ok(url) = jobs.recv() {
                    let r = fetch(&agent, &url);
                    if out.send(r).is_err() {
                        break;
                    }
                }
            })
            .ok()?;
        Some(Worker { tx, rx })
    }

    pub fn request(&self, url: String) -> bool {
        self.tx.send(url).is_ok()
    }

    pub fn poll(&self) -> Option<Result<Snapshot, Failure>> {
        self.rx.try_recv().ok()
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch(agent: &ureq::Agent, url: &str) -> Result<Snapshot, Failure> {
    match agent.get(url).call() {
        Ok(resp) => {
            let text = resp.into_string().map_err(|e| Failure::Bad(e.to_string()))?;
            feed::parse(&text).map_err(Failure::Bad)
        }
        Err(ureq::Error::Status(code @ (429 | 503), resp)) => {
            let after = resp.header("Retry-After").and_then(|s| s.trim().parse::<f64>().ok());
            let _ = code;
            Err(Failure::RateLimited(after))
        }
        Err(ureq::Error::Status(code, _)) => Err(Failure::Bad(format!("HTTP {code}"))),
        Err(e) => Err(Failure::Offline(e.to_string())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polite_polling() {
        let mut p = Poller::default();
        assert!(p.due(0.0));
        p.sent();
        // One request in flight at a time.
        assert!(!p.due(100.0));
        p.done(1.0, Ok(12));
        assert!(!p.due(5.9));
        assert!(p.due(6.0));
        p.sent();
        p.done(6.0, Ok(0));
        assert!((p.next_at - 16.0).abs() < 1e-9, "quiet skies poll less often");
        // Failures back off exponentially up to two minutes.
        let mut waits = Vec::new();
        let mut now = 20.0;
        for _ in 0..8 {
            p.sent();
            p.done(now, Err(Failure::Offline("no route".into())));
            waits.push(p.next_at - now);
            now = p.next_at;
        }
        assert_eq!(waits[..4], [10.0, 20.0, 40.0, 80.0]);
        assert!(waits.iter().all(|w| *w <= BACKOFF_MAX));
        // Rate limited: at least a minute, or what the server asks.
        let mut q = Poller::default();
        q.sent();
        q.done(0.0, Err(Failure::RateLimited(None)));
        assert!(q.next_at >= 60.0);
        let mut q = Poller::default();
        q.sent();
        q.done(0.0, Err(Failure::RateLimited(Some(300.0))));
        assert!((q.next_at - 300.0).abs() < 1e-9);
        // Recovery resets the back-off.
        q.sent();
        q.done(400.0, Ok(3));
        assert_eq!(q.failures, 0);
        assert!((q.next_at - 405.0).abs() < 1e-9);
    }

    #[test]
    fn query_looks_ahead() {
        // Parked: centred on the player.
        assert_eq!(query_centre(51.47, -0.45, 90.0, 0.0, 50.0), (51.47, -0.45));
        // 300 m/s eastbound: 120 s ahead (36 km) capped at 35 % of 50 NM (32.4 km).
        let (lat, lon) = query_centre(51.47, -0.45, 90.0, 300.0, 50.0);
        assert!((lat - 51.47).abs() < 1e-9);
        let east = (lon + 0.45) * 111_320.0 * 51.47f64.to_radians().cos();
        assert!((east - 0.35 * 50.0 * 1852.0).abs() < 50.0, "{east}");
    }
}
