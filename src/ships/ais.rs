//! Live AIS from Fintraffic's Digitraffic marine API (CC BY 4.0, no key,
//! CORS `*`: the browser build can use it too).
//!
//! * `GET /api/ais/v1/locations?latitude=&longitude=&radius=` (km): GeoJSON
//!   positions (SOG kn, COG °, true heading ° or 511, nav status, report
//!   time `timestampExternal` in Unix ms). Cached 60 s by their CDN, so the
//!   poller asks at most once a minute.
//! * `GET /api/ais/v1/vessels`: static data of the vessels heard recently
//!   (name, AIS ship type, antenna offsets A/B/C/D → length and beam,
//!   draught in 0.1 m). Asked once a session, then every
//!   [`META_REFRESH_S`] if positions arrive for unknown vessels.
//!
//! Coverage is the Baltic north of ~57.5° N (Gulf of Finland, Gulf of
//! Bothnia, Åland); outside [`COVERAGE`] the API is never called.
//! Requests carry a generic `Digitraffic-User` header natively (the browser
//! sends no custom headers: that would need a CORS preflight).

use std::collections::HashMap;

/// Attribution (CC BY 4.0).
pub const ATTRIBUTION: &str = "Live ships: Fintraffic / digitraffic.fi, CC BY 4.0";

pub const BASE: &str = "https://meri.digitraffic.fi/api/ais/v1";
/// Generic client id (no personal information).
pub const USER: &str = "Skyward/0.2";

/// Where Digitraffic has data: (west, south, east, north).
pub const COVERAGE: (f64, f64, f64, f64) = (9.0, 57.3, 32.5, 66.5);
/// Seconds between position polls (the CDN caches answers for 60 s).
pub const POLL_S: f64 = 60.0;
/// Seconds between metadata refreshes when unknown vessels are around.
pub const META_REFRESH_S: f64 = 900.0;
/// Query radius around the player (km).
pub const RADIUS_KM: f64 = 60.0;

/// True if (`lat`, `lon`) is within reach of the feed (with the query radius).
pub fn covers(lat: f64, lon: f64) -> bool {
    let (w, s, e, n) = COVERAGE;
    lat > s && lat < n && lon > w && lon < e
}

pub fn locations_url(lat: f64, lon: f64) -> String {
    format!("{BASE}/locations?latitude={lat:.3}&longitude={lon:.3}&radius={RADIUS_KM:.0}")
}

pub fn vessels_url() -> String {
    format!("{BASE}/vessels")
}

/// One position report.
#[derive(Clone, Debug, PartialEq)]
pub struct Fix {
    pub mmsi: u32,
    pub lat: f64,
    pub lon: f64,
    /// Speed over ground, knots.
    pub sog: f32,
    /// Course over ground, degrees (360 = unknown).
    pub cog: f32,
    /// True heading, degrees (None = not available).
    pub heading: Option<f32>,
    /// Navigational status (0 under way, 1 at anchor, 5 moored, 7 fishing, 8 sailing, 15 undefined).
    pub nav_stat: u8,
    /// Report time, Unix seconds.
    pub t: f64,
}

impl Fix {
    /// At anchor / moored / aground, or not moving.
    pub fn stopped(&self) -> bool {
        matches!(self.nav_stat, 1 | 5 | 6) || self.sog < 0.3
    }
    /// Best heading (degrees): true heading, else course over ground.
    pub fn best_heading(&self) -> Option<f32> {
        self.heading.or(if self.cog < 360.0 && self.sog >= 0.5 { Some(self.cog) } else { None })
    }
}

/// Static and voyage data of a vessel.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Static {
    pub mmsi: u32,
    pub name: String,
    pub ship_type: u8,
    pub a: u16,
    pub b: u16,
    pub c: u8,
    pub d: u8,
    /// Draught, m (0 = unknown).
    pub draught: f32,
    pub destination: String,
}

fn num(v: &serde_json::Value, k: &str) -> Option<f64> {
    v.get(k).and_then(|x| x.as_f64()).filter(|x| x.is_finite())
}

/// Parse a `/locations` answer. Malformed features are skipped; positions
/// outside the world or AIS "not available" values (91° / 181°) dropped.
pub fn parse_locations(text: &str) -> Result<Vec<Fix>, String> {
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("locations: {e}"))?;
    let feats = v.get("features").and_then(|f| f.as_array()).ok_or("locations: no features")?;
    let mut out = Vec::with_capacity(feats.len());
    for f in feats {
        let Some(mmsi) = num(f, "mmsi").or_else(|| f.get("properties").and_then(|p| num(p, "mmsi"))) else { continue };
        let Some(c) = f.get("geometry").and_then(|g| g.get("coordinates")).and_then(|c| c.as_array()) else { continue };
        let (Some(lon), Some(lat)) = (c.first().and_then(|x| x.as_f64()), c.get(1).and_then(|x| x.as_f64())) else { continue };
        if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
            continue;
        }
        let Some(p) = f.get("properties") else { continue };
        let sog = num(p, "sog").unwrap_or(0.0) as f32;
        let cog = num(p, "cog").unwrap_or(360.0) as f32;
        let heading = num(p, "heading").map(|h| h as f32).filter(|h| (0.0..360.0).contains(h));
        let t = num(p, "timestampExternal").map(|ms| ms / 1000.0).unwrap_or(0.0);
        out.push(Fix {
            mmsi: mmsi as u32,
            lat,
            lon,
            // 102.3 = not available.
            sog: if (0.0..102.0).contains(&sog) { sog } else { 0.0 },
            cog: if (0.0..360.0).contains(&cog) { cog } else { 360.0 },
            heading,
            nav_stat: num(p, "navStat").unwrap_or(15.0).clamp(0.0, 15.0) as u8,
            t,
        });
    }
    Ok(out)
}

/// Parse a `/vessels` answer.
pub fn parse_vessels(text: &str) -> Result<Vec<Static>, String> {
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("vessels: {e}"))?;
    let arr = v.as_array().ok_or("vessels: not an array")?;
    let mut out = Vec::with_capacity(arr.len());
    for x in arr {
        let Some(mmsi) = num(x, "mmsi") else { continue };
        let u = |k: &str, max: f64| num(x, k).unwrap_or(0.0).clamp(0.0, max);
        let s = |k: &str| x.get(k).and_then(|v| v.as_str()).unwrap_or("").trim().chars().filter(|c| !c.is_control()).take(40).collect::<String>();
        out.push(Static {
            mmsi: mmsi as u32,
            name: s("name"),
            ship_type: u("shipType", 255.0) as u8,
            a: u("referencePointA", 511.0) as u16,
            b: u("referencePointB", 511.0) as u16,
            c: u("referencePointC", 63.0) as u8,
            d: u("referencePointD", 63.0) as u8,
            draught: u("draught", 255.0) as f32 * 0.1,
            destination: s("destination"),
        });
    }
    Ok(out)
}

/// What the worker brings back.
pub enum Answer {
    Fixes(Vec<Fix>),
    Vessels(Vec<Static>),
    Failed(String),
}

/// Polite scheduling: one request at a time, positions every [`POLL_S`],
/// exponential back-off on failure (to 10 min).
#[derive(Clone, Debug, Default)]
pub struct Poller {
    pub in_flight: bool,
    pub next_fix: f64,
    pub next_meta: f64,
    pub have_meta: bool,
    pub failures: u32,
    pub last_ok: Option<f64>,
}

/// The next request due at `now` (Unix seconds), if any.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Due {
    Fixes,
    Vessels,
}

impl Poller {
    pub fn due(&self, now: f64, unknown: usize) -> Option<Due> {
        if self.in_flight {
            return None;
        }
        if now >= self.next_meta && (!self.have_meta || unknown > 0) {
            return Some(Due::Vessels);
        }
        (now >= self.next_fix).then_some(Due::Fixes)
    }
    pub fn sent(&mut self) {
        self.in_flight = true;
    }
    pub fn done(&mut self, now: f64, what: Due, ok: bool) {
        self.in_flight = false;
        if ok {
            self.failures = 0;
            self.last_ok = Some(now);
            match what {
                Due::Fixes => self.next_fix = now + POLL_S,
                Due::Vessels => {
                    self.have_meta = true;
                    self.next_meta = now + META_REFRESH_S;
                }
            }
        } else {
            self.failures += 1;
            let wait = (30.0 * 2f64.powi(self.failures as i32 - 1)).min(600.0);
            self.next_fix = self.next_fix.max(now + wait);
            self.next_meta = self.next_meta.max(now + wait);
            if what == Due::Vessels && !self.have_meta {
                // Positions alone still work (default dimensions).
                self.have_meta = true;
            }
        }
    }
}

/// Native HTTP worker: requests in, parsed answers out.
#[cfg(not(target_arch = "wasm32"))]
pub struct Worker {
    tx: std::sync::mpsc::Sender<(Due, String)>,
    rx: std::sync::mpsc::Receiver<(Due, Answer)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl Worker {
    pub fn start() -> Option<Worker> {
        use std::io::Read;
        use std::sync::mpsc;
        let (tx, jobs) = mpsc::channel::<(Due, String)>();
        let (out, rx) = mpsc::channel();
        std::thread::Builder::new()
            .name("ais".into())
            .spawn(move || {
                let agent = ureq::AgentBuilder::new().timeout_connect(std::time::Duration::from_secs(8)).timeout(std::time::Duration::from_secs(20)).user_agent(USER).build();
                while let Ok((what, url)) = jobs.recv() {
                    let r = agent.get(&url).set("Digitraffic-User", USER).set("Accept-Encoding", "gzip").call();
                    let ans = match r {
                        Ok(r) => {
                            let mut raw = Vec::new();
                            match r.into_reader().take(16 << 20).read_to_end(&mut raw) {
                                Ok(_) => {
                                    // (Still gzipped unless the HTTP client unpacked it.)
                                    let gz = raw.starts_with(&[0x1f, 0x8b]);
                                    let text = if gz {
                                        let mut s = String::new();
                                        flate2::read::GzDecoder::new(&raw[..]).take(64 << 20).read_to_string(&mut s).map(|_| s).map_err(|e| e.to_string())
                                    } else {
                                        String::from_utf8(raw).map_err(|e| e.to_string())
                                    };
                                    match (text, what) {
                                        (Ok(t), Due::Fixes) => parse_locations(&t).map(Answer::Fixes).unwrap_or_else(Answer::Failed),
                                        (Ok(t), Due::Vessels) => parse_vessels(&t).map(Answer::Vessels).unwrap_or_else(Answer::Failed),
                                        (Err(e), _) => Answer::Failed(e),
                                    }
                                }
                                Err(e) => Answer::Failed(e.to_string()),
                            }
                        }
                        Err(e) => Answer::Failed(e.to_string()),
                    };
                    if out.send((what, ans)).is_err() {
                        break;
                    }
                }
            })
            .ok()?;
        Some(Worker { tx, rx })
    }

    pub fn request(&self, what: Due, url: String) -> bool {
        self.tx.send((what, url)).is_ok()
    }

    pub fn poll(&self) -> Option<(Due, Answer)> {
        self.rx.try_recv().ok()
    }
}

/// Browser: the same requests through `fetch` (the answers land in a
/// shared slot the session polls).
#[cfg(target_arch = "wasm32")]
pub struct Worker {
    slot: std::rc::Rc<std::cell::RefCell<Vec<(Due, Answer)>>>,
}

#[cfg(target_arch = "wasm32")]
impl Worker {
    pub fn start() -> Option<Worker> {
        Some(Worker { slot: Default::default() })
    }

    pub fn request(&self, what: Due, url: String) -> bool {
        let slot = self.slot.clone();
        crate::platform::web::spawn_local(async move {
            let ans = match crate::platform::web::get(&url, web_time::Duration::from_secs(20)).await {
                Ok(body) => match (String::from_utf8(body), what) {
                    (Ok(t), Due::Fixes) => parse_locations(&t).map(Answer::Fixes).unwrap_or_else(Answer::Failed),
                    (Ok(t), Due::Vessels) => parse_vessels(&t).map(Answer::Vessels).unwrap_or_else(Answer::Failed),
                    (Err(e), _) => Answer::Failed(e.to_string()),
                },
                Err(e) => Answer::Failed(e),
            };
            slot.borrow_mut().push((what, ans));
        });
        true
    }

    pub fn poll(&self) -> Option<(Due, Answer)> {
        let mut s = self.slot.borrow_mut();
        if s.is_empty() { None } else { Some(s.remove(0)) }
    }
}

/// One live vessel's track: the last report and a smoothed display state
/// (dead reckoning from the report, corrections blended out over a few seconds).
#[derive(Clone, Debug)]
pub struct Track {
    pub fix: Fix,
    /// Display position (lat, lon) and heading (deg) at `shown_t`.
    pub shown: Option<(f64, f64, f32)>,
    /// Correction still being blended out: Δ(lat, lon).
    pub corr: (f64, f64),
}

/// Dead-reckoned position of a report at time `t` (Unix s): along the
/// course at the reported speed, at most 3 minutes ahead (a stale report
/// of a fast ferry must not sail it onto the shore).
pub fn dead_reckon(f: &Fix, t: f64) -> (f64, f64) {
    if f.stopped() || f.cog >= 360.0 {
        return (f.lat, f.lon);
    }
    let dt = (t - f.t).clamp(0.0, 180.0);
    let d = f.sog as f64 * 0.514_444 * dt;
    let c = (f.cog as f64).to_radians();
    let dlat = d * c.cos() / crate::osm::M_PER_DEG_LAT;
    let dlon = d * c.sin() / (crate::osm::M_PER_DEG_LAT * f.lat.to_radians().cos().max(0.05));
    (f.lat + dlat, f.lon + dlon)
}

/// Merge a batch of reports into the tracks (newer reports replace older).
/// Returns how many MMSIs have no static data.
pub fn merge(tracks: &mut HashMap<u32, Track>, fixes: Vec<Fix>, statics: &HashMap<u32, Static>, now: f64) -> usize {
    for f in fixes {
        match tracks.get_mut(&f.mmsi) {
            Some(tr) => {
                if f.t > tr.fix.t {
                    // Keep the shown position continuous: the jump becomes a correction.
                    if let Some((la, lo, _)) = tr.shown {
                        let (nla, nlo) = dead_reckon(&f, now);
                        tr.corr = (la - nla, lo - nlo);
                        // Big jumps (bad report, long gap): snap.
                        if tr.corr.0.abs() > 0.02 || tr.corr.1.abs() > 0.04 {
                            tr.corr = (0.0, 0.0);
                        }
                    }
                    tr.fix = f;
                }
            }
            None => {
                tracks.insert(f.mmsi, Track { fix: f, shown: None, corr: (0.0, 0.0) });
            }
        }
    }
    // Reports older than 20 minutes: gone.
    tracks.retain(|_, t| now - t.fix.t < 1200.0);
    tracks.keys().filter(|m| !statics.contains_key(m)).count()
}
