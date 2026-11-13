//! Multiplayer preferences: player name, favourite and recent servers —
//! `multiplayer.json` next to `settings.json` (ui/settings.rs). Passwords are
//! never stored.

use serde_json::{Value, json};

#[derive(Clone, Debug, PartialEq)]
pub struct Recent {
    /// Address as typed / found ("192.168.1.20:47820", "lan-box.local").
    pub addr: String,
    /// Server name when last seen.
    pub name: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Prefs {
    pub name: String,
    pub favourites: Vec<Recent>,
    pub recent: Vec<Recent>,
    /// Headless captures and tests never write the player's file.
    pub read_only: bool,
}

const MAX_RECENT: usize = 12;

#[cfg(not(target_arch = "wasm32"))]
fn path() -> std::path::PathBuf {
    crate::ui::settings::path().with_file_name("multiplayer.json")
}

fn list(v: &Value) -> Vec<Recent> {
    v.as_array()
        .map(|a| {
            a.iter()
                .filter_map(|e| {
                    Some(Recent {
                        addr: e.get("addr")?.as_str()?.chars().take(255).collect(),
                        name: e.get("name").and_then(Value::as_str).unwrap_or("").chars().take(64).collect(),
                    })
                })
                .filter(|r| !r.addr.trim().is_empty())
                .collect()
        })
        .unwrap_or_default()
}

fn to_json(v: &[Recent]) -> Value {
    Value::Array(v.iter().map(|r| json!({ "addr": r.addr, "name": r.name })).collect())
}

/// A default player name: the login name, else "Pilot".
fn default_name() -> String {
    let n = std::env::var("USER").or_else(|_| std::env::var("USERNAME")).unwrap_or_default();
    let n = skyward_net::codec::sanitize(&n, skyward_net::proto::MAX_NAME);
    if n.is_empty() {
        "Pilot".into()
    } else {
        let mut c = n.chars();
        c.next().map(|f| f.to_uppercase().collect::<String>() + c.as_str()).unwrap_or(n)
    }
}

impl Prefs {
    pub fn from_json(v: &Value) -> Prefs {
        let name =
            v.get("name").and_then(Value::as_str).map(|s| skyward_net::codec::sanitize(s, skyward_net::proto::MAX_NAME)).filter(|s| !s.is_empty()).unwrap_or_else(default_name);
        Prefs { name, favourites: list(&v["favourites"]), recent: list(&v["recent"]), read_only: false }
    }

    pub fn to_json(&self) -> Value {
        json!({ "name": self.name, "favourites": to_json(&self.favourites), "recent": to_json(&self.recent) })
    }

    pub fn load() -> Prefs {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(v) = std::fs::read_to_string(path()).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()) {
            return Prefs::from_json(&v);
        }
        Prefs::from_json(&Value::Null)
    }

    pub fn save(&self) {
        #[cfg(not(target_arch = "wasm32"))]
        if !self.read_only {
            let p = path();
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let tmp = p.with_extension("tmp");
            if std::fs::write(&tmp, serde_json::to_string_pretty(&self.to_json()).unwrap_or_default()).is_ok() {
                let _ = std::fs::rename(&tmp, &p);
            }
        }
    }

    pub fn is_favourite(&self, addr: &str) -> bool {
        self.favourites.iter().any(|f| f.addr.eq_ignore_ascii_case(addr))
    }

    pub fn toggle_favourite(&mut self, addr: &str, name: &str) {
        if self.is_favourite(addr) {
            self.favourites.retain(|f| !f.addr.eq_ignore_ascii_case(addr));
        } else {
            self.favourites.push(Recent { addr: addr.into(), name: name.into() });
        }
    }

    /// Joined a server: most recent first.
    pub fn add_recent(&mut self, addr: &str, name: &str) {
        self.recent.retain(|r| !r.addr.eq_ignore_ascii_case(addr));
        self.recent.insert(0, Recent { addr: addr.into(), name: name.into() });
        self.recent.truncate(MAX_RECENT);
        for f in &mut self.favourites {
            if f.addr.eq_ignore_ascii_case(addr) && !name.is_empty() {
                f.name = name.into();
            }
        }
    }

    pub fn forget(&mut self, addr: &str) {
        self.recent.retain(|r| !r.addr.eq_ignore_ascii_case(addr));
        self.favourites.retain(|r| !r.addr.eq_ignore_ascii_case(addr));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_recent_order() {
        let mut p = Prefs { name: "Will".into(), ..Default::default() };
        p.add_recent("10.0.0.2:47820", "A");
        p.add_recent("10.0.0.3:47820", "B");
        p.add_recent("10.0.0.2:47820", "A2");
        assert_eq!(p.recent.iter().map(|r| r.addr.as_str()).collect::<Vec<_>>(), ["10.0.0.2:47820", "10.0.0.3:47820"]);
        p.toggle_favourite("lan-box.local", "Box");
        assert!(p.is_favourite("LAN-BOX.local"));
        let q = Prefs::from_json(&p.to_json());
        assert_eq!(p, q);
        p.toggle_favourite("lan-box.local", "");
        assert!(!p.is_favourite("lan-box.local"));
        // Junk in the file never panics and gets a default name.
        let r = Prefs::from_json(&serde_json::json!({ "name": "\u{202e}  ", "recent": [1, {"addr": ""}, {"addr": "x"}] }));
        assert!(!r.name.is_empty());
        assert_eq!(r.recent.len(), 1);
    }
}
