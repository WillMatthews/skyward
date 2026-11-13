//! The server browser's model: LAN discovery (broadcast on every local
//! subnet + mDNS `_skyward._udp`), favourites and recent servers, manual
//! addresses, pings refreshed every few seconds, details of the selected
//! server. The screen is `ui/browser.rs`.

use super::prefs::Prefs;
use skyward_net::proto::{PROTOCOL, ServerDetails, ServerInfo};
use std::net::SocketAddr;

/// Rescan / re-ping interval, s.
const SCAN_EVERY: f64 = 3.0;
/// A LAN server not heard from for this long is dropped from the list.
const LAN_FORGET: f64 = 12.0;
/// Unanswered pings before a known server shows as unreachable.
const UNREACHABLE_AFTER: u32 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    All,
    Lan,
    Favourites,
    Recent,
}

impl Tab {
    pub const ALL: [Tab; 4] = [Tab::All, Tab::Lan, Tab::Favourites, Tab::Recent];
    pub fn label(self) -> &'static str {
        match self {
            Tab::All => "All",
            Tab::Lan => "LAN",
            Tab::Favourites => "Favourites",
            Tab::Recent => "Recent",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SortKey {
    Name,
    Players,
    Airport,
    Ping,
}

#[derive(Clone, Debug)]
pub struct Entry {
    /// Address as shown / stored ("192.168.1.20:47820", "lan-box.local").
    pub key: String,
    pub sock: Option<SocketAddr>,
    pub info: Option<ServerInfo>,
    pub proto: u16,
    pub version: String,
    pub ping_ms: Option<f32>,
    pub lan: bool,
    pub mdns: bool,
    pub manual: bool,
    pub last_seen: f64,
    pub details: Option<ServerDetails>,
    /// Lookup / reachability problem to show instead of the info.
    pub error: Option<String>,
    missed: u32,
    resolving: bool,
}

impl Entry {
    fn new(key: &str) -> Entry {
        Entry {
            key: key.to_string(),
            sock: None,
            info: None,
            proto: 0,
            version: String::new(),
            ping_ms: None,
            lan: false,
            mdns: false,
            manual: false,
            last_seen: f64::NEG_INFINITY,
            details: None,
            error: None,
            missed: 0,
            resolving: false,
        }
    }
    /// The server runs a protocol this build cannot join.
    pub fn incompatible(&self) -> bool {
        self.proto != 0 && self.proto != PROTOCOL
    }
    pub fn full(&self) -> bool {
        self.info.as_ref().is_some_and(|i| i.players >= i.max_players)
    }
    pub fn locked(&self) -> bool {
        self.info.as_ref().is_some_and(|i| i.password)
    }
    pub fn name(&self) -> String {
        self.info.as_ref().map(|i| i.name.clone()).unwrap_or_else(|| self.key.clone())
    }
    pub fn online(&self) -> bool {
        self.ping_ms.is_some() && self.error.is_none()
    }
}

pub struct Browser {
    #[cfg(not(target_arch = "wasm32"))]
    scanner: Option<skyward_net::discovery::Scanner>,
    #[cfg(not(target_arch = "wasm32"))]
    resolves: Vec<(String, std::sync::mpsc::Receiver<std::io::Result<SocketAddr>>)>,
    pub entries: Vec<Entry>,
    pub tab: Tab,
    pub sort: SortKey,
    pub ascending: bool,
    pub search: String,
    pub hide_full: bool,
    pub hide_locked: bool,
    pub selected: Option<String>,
    /// "Add by address" field.
    pub add_text: String,
    /// Password prompt for this entry (key, typed password).
    pub password: Option<(String, String)>,
    /// Problem with discovery itself (no socket).
    pub problem: Option<String>,
    next_scan: f64,
    pub scans: u32,
    next_details: f64,
    open: bool,
}

impl Default for Browser {
    fn default() -> Self {
        Browser {
            #[cfg(not(target_arch = "wasm32"))]
            scanner: None,
            #[cfg(not(target_arch = "wasm32"))]
            resolves: Vec::new(),
            entries: Vec::new(),
            tab: Tab::All,
            sort: SortKey::Ping,
            ascending: true,
            search: String::new(),
            hide_full: false,
            hide_locked: false,
            selected: None,
            add_text: String::new(),
            password: None,
            problem: None,
            next_scan: 0.0,
            scans: 0,
            next_details: 0.0,
            open: false,
        }
    }
}

impl Browser {
    /// The screen opened: start discovery, list favourites and recents.
    pub fn open(&mut self, prefs: &Prefs, now: f64) {
        if crate::platform::WEB {
            self.problem = Some(super::WEB_MESSAGE.into());
            return;
        }
        if !self.open {
            self.open = true;
            self.next_scan = now;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.scanner.is_none() {
            let seed = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(7);
            match skyward_net::discovery::Scanner::new(seed) {
                Ok(s) => {
                    self.scanner = Some(s);
                    self.problem = None;
                }
                Err(e) => self.problem = Some(format!("LAN discovery unavailable: {e}")),
            }
        }
        for r in prefs.favourites.iter().chain(prefs.recent.iter()) {
            self.ensure(&r.addr, now);
        }
    }

    /// The screen closed: stop pinging.
    pub fn close(&mut self) {
        self.open = false;
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    /// Rescan now (the Refresh button).
    pub fn refresh(&mut self, now: f64) {
        self.next_scan = now;
        for e in &mut self.entries {
            e.missed = 0;
        }
    }

    /// Make sure an address is listed (and being resolved / pinged).
    pub fn ensure(&mut self, addr: &str, now: f64) -> String {
        let key = addr.trim().to_string();
        if key.is_empty() {
            return key;
        }
        if let Some(e) = self.entries.iter().find(|e| e.key.eq_ignore_ascii_case(&key) || e.sock.is_some_and(|s| s.to_string() == key)) {
            return e.key.clone();
        }
        let mut e = Entry::new(&key);
        e.manual = true;
        e.resolving = true;
        self.entries.push(e);
        self.resolve(&key, now);
        key
    }

    /// "Add by address".
    pub fn add_manual(&mut self, now: f64) -> Option<String> {
        let t = self.add_text.trim().to_string();
        if t.is_empty() {
            return None;
        }
        self.add_text.clear();
        let k = self.ensure(&t, now);
        self.selected = Some(k.clone());
        Some(k)
    }

    #[allow(unused_variables)]
    fn resolve(&mut self, key: &str, now: f64) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (tx, rx) = std::sync::mpsc::channel();
            let k = key.to_string();
            if std::thread::Builder::new()
                .name("mp-lookup".into())
                .spawn(move || {
                    let _ = tx.send(skyward_net::discovery::resolve(&k, skyward_net::DEFAULT_PORT));
                })
                .is_ok()
            {
                self.resolves.push((key.to_string(), rx));
            }
        }
    }

    pub fn entry(&self, key: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.key == key)
    }

    /// Discovery traffic, pings, lookups. Call every frame while the screen is open.
    pub fn tick(&mut self, now: f64) {
        #[cfg(not(target_arch = "wasm32"))]
        if self.open {
            // Lookups that finished.
            let mut done = Vec::new();
            self.resolves.retain(|(k, rx)| match rx.try_recv() {
                Ok(r) => {
                    done.push((k.clone(), r));
                    false
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => true,
                Err(_) => false,
            });
            for (k, r) in done {
                if let Some(e) = self.entries.iter_mut().find(|e| e.key == k) {
                    e.resolving = false;
                    match r {
                        Ok(a) => {
                            e.sock = Some(a);
                            e.error = None;
                            if let Some(s) = self.scanner.as_mut() {
                                s.probe(a, now, skyward_net::discovery::Via::Direct);
                            }
                        }
                        Err(err) => e.error = Some(format!("Unknown host: {err}")),
                    }
                }
            }
            let Some(sc) = self.scanner.as_mut() else { return };
            if now >= self.next_scan {
                self.next_scan = now + SCAN_EVERY;
                self.scans += 1;
                sc.scan(now);
                for e in &mut self.entries {
                    if let Some(a) = e.sock {
                        if !e.lan || now - e.last_seen > SCAN_EVERY * 0.9 {
                            sc.probe(a, now, skyward_net::discovery::Via::Direct);
                            e.missed += 1;
                            if e.missed > UNREACHABLE_AFTER && e.manual {
                                e.error = Some("No answer (offline, or UDP blocked by a firewall)".into());
                                e.ping_ms = None;
                            }
                        }
                    }
                }
                // LAN servers that went away.
                self.entries.retain(|e| e.manual || !e.lan || now - e.last_seen < LAN_FORGET);
            }
            // Details of the selected server.
            if now >= self.next_details {
                self.next_details = now + 2.0;
                if let Some(a) = self.selected.as_ref().and_then(|k| self.entries.iter().find(|e| &e.key == k)).and_then(|e| e.sock) {
                    sc.request_details(a);
                }
            }
            let events = sc.poll(now);
            for ev in events {
                self.on_event(ev, now);
            }
        }
        #[cfg(target_arch = "wasm32")]
        let _ = now;
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn on_event(&mut self, ev: skyward_net::discovery::ScanEvent, now: f64) {
        use skyward_net::discovery::{ScanEvent, Via};
        match ev {
            ScanEvent::Server { addr, proto, version, info, rtt_ms, via } => {
                // Same server found another way (localhost and the LAN address): one entry.
                if let Some(inst) = info.as_ref().map(|i| i.instance) {
                    if let Some(other) = self.entries.iter_mut().find(|e| e.sock != Some(addr) && e.info.as_ref().is_some_and(|i| i.instance == inst)) {
                        let other_lo = other.sock.is_some_and(|s| s.ip().is_loopback());
                        if addr.ip().is_loopback() || !other_lo {
                            // Keep the existing (LAN) entry; just refresh it.
                            other.last_seen = now;
                            other.missed = 0;
                            if via == Via::Direct || other.ping_ms.is_none() {
                                other.ping_ms = Some(rtt_ms);
                            }
                            return;
                        }
                        // The existing one is the loopback alias: switch it to the LAN address.
                        other.sock = Some(addr);
                        if !other.manual {
                            other.key = addr.to_string();
                        }
                    }
                }
                let idx = match self.entries.iter().position(|e| e.sock == Some(addr)) {
                    Some(i) => i,
                    None => {
                        let mut e = Entry::new(&addr.to_string());
                        e.sock = Some(addr);
                        self.entries.push(e);
                        self.entries.len() - 1
                    }
                };
                let e = &mut self.entries[idx];
                e.proto = proto;
                e.version = version;
                if info.is_some() {
                    e.info = info;
                }
                e.ping_ms = Some(rtt_ms);
                e.last_seen = now;
                e.missed = 0;
                e.error = None;
                match via {
                    Via::Broadcast => e.lan = true,
                    Via::Mdns => {
                        e.lan = true;
                        e.mdns = true;
                    }
                    Via::Direct => {}
                }
            }
            ScanEvent::Details { addr, details } => {
                if let Some(e) = self.entries.iter_mut().find(|e| e.sock == Some(addr)) {
                    e.details = Some(details);
                }
            }
        }
    }

    /// The entries to show, filtered and sorted.
    pub fn visible(&self, prefs: &Prefs) -> Vec<&Entry> {
        let q = self.search.trim().to_lowercase();
        let mut v: Vec<&Entry> = self
            .entries
            .iter()
            .filter(|e| match self.tab {
                Tab::All => true,
                Tab::Lan => e.lan,
                Tab::Favourites => prefs.is_favourite(&e.key),
                Tab::Recent => prefs.recent.iter().any(|r| r.addr.eq_ignore_ascii_case(&e.key)),
            })
            .filter(|e| !(self.hide_full && e.full()) && !(self.hide_locked && e.locked()))
            .filter(|e| {
                q.is_empty()
                    || e.name().to_lowercase().contains(&q)
                    || e.key.to_lowercase().contains(&q)
                    || e.info.as_ref().is_some_and(|i| i.airport.to_lowercase().contains(&q) || i.region.to_lowercase().contains(&q))
            })
            .collect();
        let key = |e: &Entry| -> (u8, f32, String) {
            // Reachable, compatible servers first whatever the column.
            let rank = if e.online() && !e.incompatible() { 0 } else { 1 };
            match self.sort {
                SortKey::Name => (rank, 0.0, e.name().to_lowercase()),
                SortKey::Players => (rank, -(e.info.as_ref().map(|i| i.players as f32).unwrap_or(-1.0)), e.name().to_lowercase()),
                SortKey::Airport => (rank, 0.0, e.info.as_ref().map(|i| i.airport.clone()).unwrap_or_default()),
                SortKey::Ping => (rank, e.ping_ms.unwrap_or(f32::MAX), e.name().to_lowercase()),
            }
        };
        v.sort_by(|a, b| {
            let (ka, kb) = (key(a), key(b));
            let o = ka.0.cmp(&kb.0).then(ka.1.total_cmp(&kb.1)).then(ka.2.cmp(&kb.2));
            if self.ascending || ka.0 != kb.0 { o } else { o.reverse() }
        });
        v
    }

    /// Click on a column header: sort by it, or flip the order.
    pub fn sort_by(&mut self, k: SortKey) {
        if self.sort == k {
            self.ascending = !self.ascending;
        } else {
            self.sort = k;
            self.ascending = true;
        }
    }

    /// For tests and screenshots: a server as if discovered.
    pub fn insert_for_test(&mut self, e: Entry) {
        self.entries.push(e);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(key: &str, name: &str, players: u16, ping: f32) -> Entry {
        let mut e = Entry::new(key);
        e.info = Some(ServerInfo { name: name.into(), players, max_players: 8, airport: "EGLC".into(), ..Default::default() });
        e.proto = PROTOCOL;
        e.ping_ms = Some(ping);
        e.lan = true;
        e
    }

    #[test]
    fn filter_and_sort() {
        let mut b = Browser::default();
        b.insert_for_test(entry("a:1", "Bravo", 2, 30.0));
        b.insert_for_test(entry("b:1", "alpha", 8, 5.0));
        let mut old = entry("c:1", "Charlie", 1, 1.0);
        old.proto = PROTOCOL + 1;
        b.insert_for_test(old);
        let prefs = Prefs::default();
        let names = |b: &Browser| b.visible(&prefs).iter().map(|e| e.name()).collect::<Vec<_>>();
        assert_eq!(names(&b), ["alpha", "Bravo", "Charlie"], "by ping; incompatible last");
        b.sort_by(SortKey::Name);
        assert_eq!(names(&b), ["alpha", "Bravo", "Charlie"]);
        b.sort_by(SortKey::Name);
        assert_eq!(names(&b), ["Bravo", "alpha", "Charlie"]);
        b.hide_full = true;
        assert_eq!(names(&b), ["Bravo", "Charlie"]);
        b.search = "char".into();
        assert_eq!(names(&b), ["Charlie"]);
    }
}
