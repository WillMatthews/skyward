//! Structured logging to stderr: `2026-11-13T12:00:00Z INFO  join #1 ...`
//! or JSON lines (`--log-json`) for log collectors.

use skyward_net::server::Level;

pub struct Logger {
    pub min: Level,
    pub json: bool,
}

/// Days since 1970-01-01 → (year, month, day) (proleptic Gregorian).
pub fn civil(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + if m <= 2 { 1 } else { 0 };
    (y, m, d)
}

pub fn timestamp() -> String {
    let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0) as i64;
    let (y, mo, d) = civil(t.div_euclid(86400));
    let s = t.rem_euclid(86400);
    format!("{y:04}-{mo:02}-{d:02}T{:02}:{:02}:{:02}Z", s / 3600, s / 60 % 60, s % 60)
}

fn json_escape(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o
}

impl Logger {
    pub fn new(level: &str, json: bool) -> Logger {
        let min = match level {
            "debug" => Level::Debug,
            "warn" => Level::Warn,
            _ => Level::Info,
        };
        Logger { min, json }
    }

    pub fn log(&self, level: Level, msg: &str) {
        if level < self.min {
            return;
        }
        let name = match level {
            Level::Debug => "DEBUG",
            Level::Info => "INFO",
            Level::Warn => "WARN",
        };
        if self.json {
            eprintln!("{{\"ts\":\"{}\",\"level\":\"{}\",\"msg\":\"{}\"}}", timestamp(), name.to_lowercase(), json_escape(msg));
        } else {
            eprintln!("{} {name:<5} {msg}", timestamp());
        }
    }

    pub fn info(&self, msg: &str) {
        self.log(Level::Info, msg);
    }
    pub fn warn(&self, msg: &str) {
        self.log(Level::Warn, msg);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn civil_dates() {
        assert_eq!(super::civil(0), (1970, 1, 1));
        assert_eq!(super::civil(19_723), (2024, 1, 1));
        assert_eq!(super::civil(20_405), (2025, 11, 13));
        assert_eq!(super::json_escape("a\"b\\c\n"), "a\\\"b\\\\c\\n");
    }
}
