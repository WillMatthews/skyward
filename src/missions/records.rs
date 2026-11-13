//! Best mission results, kept per player on this machine: next to the
//! settings (`~/.config/skyward/missions.json`) or in the browser's
//! localStorage. Nothing leaves the machine.

use super::Debrief;
use super::score::Medal;
use serde_json::{Map, Value, json};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Best {
    pub score: f32,
    pub grade: String,
    pub medal: Medal,
    pub time_s: f32,
    pub completed: bool,
    pub attempts: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Records {
    pub map: BTreeMap<String, Best>,
}

#[cfg(not(target_arch = "wasm32"))]
fn path() -> std::path::PathBuf {
    crate::ui::settings::path().with_file_name("missions.json")
}

impl Records {
    pub fn from_json(v: &Value) -> Records {
        let mut map = BTreeMap::new();
        if let Some(o) = v.get("best").and_then(|x| x.as_object()) {
            for (k, b) in o {
                let f = |n: &str| b.get(n).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32;
                map.insert(
                    k.clone(),
                    Best {
                        score: f("score"),
                        grade: b.get("grade").and_then(|x| x.as_str()).unwrap_or("").to_string(),
                        medal: Medal::from_name(b.get("medal").and_then(|x| x.as_str()).unwrap_or("")),
                        time_s: f("time_s"),
                        completed: b.get("completed").and_then(|x| x.as_bool()).unwrap_or(false),
                        attempts: f("attempts") as u32,
                    },
                );
            }
        }
        Records { map }
    }

    pub fn to_json(&self) -> Value {
        let mut o = Map::new();
        for (k, b) in &self.map {
            o.insert(k.clone(), json!({ "score": b.score, "grade": b.grade, "medal": b.medal.name(), "time_s": b.time_s, "completed": b.completed, "attempts": b.attempts }));
        }
        json!({ "version": 1, "best": Value::Object(o) })
    }

    pub fn load() -> Records {
        #[cfg(target_arch = "wasm32")]
        let text = crate::platform::web::storage_get("skyward.missions");
        #[cfg(not(target_arch = "wasm32"))]
        let text = std::fs::read_to_string(path()).ok();
        text.and_then(|t| serde_json::from_str::<Value>(&t).ok()).map(|v| Records::from_json(&v)).unwrap_or_default()
    }

    pub fn save(&self) {
        let text = serde_json::to_string_pretty(&self.to_json()).unwrap_or_default();
        #[cfg(target_arch = "wasm32")]
        crate::platform::web::storage_set("skyward.missions", &text);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let p = path();
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let tmp = p.with_extension("tmp");
            if std::fs::write(&tmp, text).is_ok() {
                let _ = std::fs::rename(&tmp, &p);
            }
        }
    }

    /// Count an attempt and keep the result if it is the best. True = new best.
    pub fn record(&mut self, id: &str, d: &Debrief) -> bool {
        let b = self.map.entry(id.to_string()).or_default();
        b.attempts += 1;
        let better = (d.success && !b.completed) || (d.success == b.completed && d.score.total > b.score) || b.attempts == 1;
        if better {
            b.score = d.score.total;
            b.grade = d.score.grade.to_string();
            b.medal = d.score.medal;
            b.time_s = d.time_s;
            b.completed = d.success;
        }
        better
    }

    pub fn get(&self, id: &str) -> Option<&Best> {
        self.map.get(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_round_trip_and_keep_the_best() {
        let mut r = Records::default();
        let mut d = Debrief { success: false, ..Default::default() };
        d.score.total = 300.0;
        assert!(r.record("river", &d));
        d.success = true;
        d.score.total = 640.0;
        d.score.grade = "C";
        d.score.medal = Medal::Bronze;
        assert!(r.record("river", &d));
        d.score.total = 500.0;
        assert!(!r.record("river", &d));
        let b = r.get("river").unwrap();
        assert_eq!((b.score, b.completed, b.attempts, b.medal), (640.0, true, 3, Medal::Bronze));
        let back = Records::from_json(&r.to_json());
        assert_eq!(back, r);
    }
}
