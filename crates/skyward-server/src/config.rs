//! Command-line flags and the optional TOML config file.

use skyward_net::proto::{DEFAULT_PORT, TimeSync, WorldSync};
use std::net::IpAddr;

pub const HELP: &str = "\
skyward-server — Skyward dedicated multiplayer server

USAGE:
    skyward-server [OPTIONS]

OPTIONS:
    -c, --config <FILE>      TOML config file (flags override it)
    -n, --name <NAME>        server name shown in the browser   [Skyward server]
    -p, --port <PORT>        UDP game port                        [47820]
    -b, --bind <ADDR>        address to listen on                 [0.0.0.0]
    -m, --max-players <N>    player slots (1-64)                  [16]
        --password <PW>      require a password to join           [none]
        --motd <TEXT>        message of the day (browser + on join)
    -a, --airport <ICAO>     airport everyone starts at           [EGLC]
        --region <TEXT>      region label for the browser (\"London\")
        --time <MODE>        `real` (UTC now) or a local time `HH:MM`   [real]
        --day <N>            day of year with a fixed time (1-366)       [today]
        --weather <MODE>     `live` (METAR of the airport, refreshed every 10 min),
                             `own` (each client keeps its own), a preset
                             (clear, few, scattered, broken, overcast, rain, fog)
                             or a full METAR string                       [live]
        --tick-hz <N>        snapshot rate, 10-60 Hz              [30]
        --no-discovery       don't answer LAN discovery broadcasts (UDP 47821)
        --no-mdns            don't announce over mDNS (_skyward._udp)
        --ban-file <FILE>    banned addresses, one per line (ban / unban keep it updated)
        --log-level <LVL>    debug, info, warn                    [info]
        --log-json           log as JSON lines
        --no-console         don't read commands from stdin
    -h, --help               this help
    -V, --version            version

PORTS (UDP; open them in a host firewall for LAN play):
    47820   game traffic (or --port)
    47821   LAN discovery broadcasts (shared by all servers on a host)
    5353    mDNS (shared with the system's responder)

CONSOLE (type `help` while running):
    list · kick <player> [reason] · ban <player|ip> [reason] · unban <ip> · bans
    say <text> · time real|HH:MM · weather live|own|<preset>|<METAR> · motd <text>
    status · quit

EXAMPLE:
    skyward-server --name \"Friday night\" --airport EGKK --max-players 12 --time 21:30
";

#[derive(Clone, Debug)]
pub struct Config {
    pub name: String,
    pub port: u16,
    pub bind: IpAddr,
    pub max_players: u16,
    pub password: String,
    pub motd: String,
    pub airport: String,
    pub region: String,
    pub time: TimeSync,
    pub weather: Weather,
    pub tick_hz: u8,
    pub discovery: bool,
    pub mdns: bool,
    pub ban_file: Option<String>,
    pub log_level: String,
    pub log_json: bool,
    pub console: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Weather {
    Live,
    Own,
    Metar(String),
}

impl Default for Config {
    fn default() -> Self {
        Config {
            name: "Skyward server".into(),
            port: DEFAULT_PORT,
            bind: IpAddr::from([0, 0, 0, 0]),
            max_players: 16,
            password: String::new(),
            motd: String::new(),
            airport: "EGLC".into(),
            region: String::new(),
            time: TimeSync::Real,
            weather: Weather::Live,
            tick_hz: 30,
            discovery: true,
            mdns: true,
            ban_file: None,
            log_level: "info".into(),
            log_json: false,
            console: true,
        }
    }
}

/// "14:30" → hours.
pub fn parse_hhmm(s: &str) -> Option<f32> {
    let (h, m) = s.trim().split_once(':')?;
    let (h, m): (u32, u32) = (h.parse().ok()?, m.parse().ok()?);
    (h < 24 && m < 60).then(|| h as f32 + m as f32 / 60.0)
}

/// Today's day of year (UTC).
pub fn today() -> u16 {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let (y, m, d) = super::log::civil(secs as i64 / 86400);
    let cum = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    (cum[(m - 1) as usize] + d + if leap && m > 2 { 1 } else { 0 }) as u16
}

/// A preset → a representative METAR (the game derives clouds / wind / visibility from it).
pub fn preset_metar(airport: &str, p: &str) -> Option<String> {
    let body = match p.to_ascii_lowercase().as_str() {
        "clear" => "24006KT 9999 SKC 18/08 Q1020",
        "few" => "24008KT 9999 FEW040 17/08 Q1018",
        "scattered" => "24010KT 9999 SCT035 16/08 Q1015",
        "broken" => "23012KT 9999 BKN025 14/09 Q1011",
        "overcast" => "22014KT 8000 OVC012 12/10 Q1006",
        "rain" => "21016G26KT 4000 RA BKN010 OVC020 11/10 Q1001",
        "fog" => "00000KT 0300 FG VV001 08/08 Q1022",
        _ => return None,
    };
    Some(format!("{} 121200Z {body}", if airport.is_empty() { "XXXX" } else { airport }))
}

impl Config {
    pub fn world(&self) -> WorldSync {
        let (metar, live) = match &self.weather {
            Weather::Live => (String::new(), true),
            Weather::Own => (String::new(), false),
            Weather::Metar(m) => (m.clone(), false),
        };
        WorldSync { time: self.time.clone(), utc: 0.0, metar, live_weather: live }
    }

    pub fn parse_weather(&self, v: &str) -> Result<Weather, String> {
        let v = v.trim();
        Ok(match v.to_ascii_lowercase().as_str() {
            "live" => Weather::Live,
            "own" | "none" | "client" => Weather::Own,
            _ => match preset_metar(&self.airport, v) {
                Some(m) => Weather::Metar(m),
                None if v.split_whitespace().count() >= 3 => Weather::Metar(v.to_string()),
                None => return Err(format!("weather \"{v}\": use live, own, clear, few, scattered, broken, overcast, rain, fog or a METAR")),
            },
        })
    }

    pub fn parse_time(&self, v: &str, day: Option<u16>) -> Result<TimeSync, String> {
        if v.eq_ignore_ascii_case("real") {
            return Ok(TimeSync::Real);
        }
        let hours = parse_hhmm(v).ok_or_else(|| format!("time \"{v}\": use real or HH:MM"))?;
        let day = match (day, &self.time) {
            (Some(d), _) => d,
            (None, TimeSync::Fixed { day, .. }) => *day,
            _ => today(),
        };
        Ok(TimeSync::Fixed { hours, day })
    }

    fn set(&mut self, key: &str, v: &str, day: &mut Option<u16>, time: &mut Option<String>, weather: &mut Option<String>) -> Result<(), String> {
        let num = |v: &str| v.trim().parse::<u64>().map_err(|_| format!("{key}: \"{v}\" is not a number"));
        match key {
            "name" => self.name = v.to_string(),
            "port" => self.port = u16::try_from(num(v)?).map_err(|_| "port out of range".to_string())?,
            "bind" => self.bind = v.trim().parse().map_err(|_| format!("bind: \"{v}\" is not an IP address"))?,
            "max_players" | "max-players" => self.max_players = num(v)?.clamp(1, 64) as u16,
            "password" => self.password = v.to_string(),
            "motd" => self.motd = v.to_string(),
            "airport" => self.airport = v.trim().to_ascii_uppercase(),
            "region" => self.region = v.to_string(),
            "time" => *time = Some(v.to_string()),
            "day" => *day = Some(num(v)?.clamp(1, 366) as u16),
            "weather" => *weather = Some(v.to_string()),
            "tick_hz" | "tick-hz" => self.tick_hz = num(v)?.clamp(10, 60) as u8,
            "ban_file" | "ban-file" => self.ban_file = Some(v.to_string()),
            "log_level" | "log-level" => {
                if !["debug", "info", "warn"].contains(&v) {
                    return Err(format!("log level \"{v}\": use debug, info or warn"));
                }
                self.log_level = v.to_string()
            }
            _ => return Err(format!("unknown setting \"{key}\"")),
        }
        Ok(())
    }

    /// Parse flags (and the config file they name). Err = message to print (exit 2);
    /// Ok(None) = help / version printed.
    pub fn from_args(args: &[String]) -> Result<Option<Config>, String> {
        let mut c = Config::default();
        let (mut day, mut time, mut weather) = (None, None, None);
        // The config file first, so flags override it.
        if let Some(i) = args.iter().position(|a| a == "-c" || a == "--config") {
            let path = args.get(i + 1).ok_or("--config needs a file")?;
            let text = std::fs::read_to_string(path).map_err(|e| format!("config {path}: {e}"))?;
            let table: toml::Table = text.parse().map_err(|e| format!("config {path}: {e}"))?;
            for (k, v) in table {
                let s = match &v {
                    toml::Value::String(s) => s.clone(),
                    toml::Value::Integer(n) => n.to_string(),
                    toml::Value::Float(f) => f.to_string(),
                    toml::Value::Boolean(b) => {
                        match k.as_str() {
                            "discovery" => c.discovery = *b,
                            "mdns" => c.mdns = *b,
                            "log_json" => c.log_json = *b,
                            "console" => c.console = *b,
                            _ => return Err(format!("config {path}: \"{k}\" is not a switch")),
                        }
                        continue;
                    }
                    _ => return Err(format!("config {path}: \"{k}\" has an unsupported type")),
                };
                c.set(&k, &s, &mut day, &mut time, &mut weather).map_err(|e| format!("config {path}: {e}"))?;
            }
        }
        let mut i = 1;
        while i < args.len() {
            let a = args[i].as_str();
            let mut val = || -> Result<String, String> {
                i += 1;
                args.get(i).cloned().ok_or_else(|| format!("{a} needs a value"))
            };
            match a {
                "-h" | "--help" => {
                    print!("{HELP}");
                    return Ok(None);
                }
                "-V" | "--version" => {
                    println!("skyward-server {} (protocol {})", skyward_net::VERSION, skyward_net::PROTOCOL);
                    return Ok(None);
                }
                "-c" | "--config" => {
                    val()?;
                }
                "-n" | "--name" => c.set("name", &val()?, &mut day, &mut time, &mut weather)?,
                "-p" | "--port" => c.set("port", &val()?, &mut day, &mut time, &mut weather)?,
                "-b" | "--bind" => c.set("bind", &val()?, &mut day, &mut time, &mut weather)?,
                "-m" | "--max-players" => c.set("max_players", &val()?, &mut day, &mut time, &mut weather)?,
                "-a" | "--airport" => c.set("airport", &val()?, &mut day, &mut time, &mut weather)?,
                "--password" | "--motd" | "--region" | "--time" | "--day" | "--weather" | "--tick-hz" | "--ban-file" | "--log-level" => {
                    let v = val()?;
                    c.set(&a[2..], &v, &mut day, &mut time, &mut weather)?
                }
                "--no-discovery" => c.discovery = false,
                "--no-mdns" => c.mdns = false,
                "--log-json" => c.log_json = true,
                "--no-console" => c.console = false,
                other => return Err(format!("unknown option {other} (see --help)")),
            }
            i += 1;
        }
        if let Some(t) = time {
            c.time = c.parse_time(&t, day)?;
        } else if let (Some(d), TimeSync::Fixed { hours, .. }) = (day, &c.time) {
            c.time = TimeSync::Fixed { hours: *hours, day: d };
        }
        if let Some(w) = weather {
            c.weather = c.parse_weather(&w)?;
        }
        if c.name.trim().is_empty() {
            return Err("the server name is empty".into());
        }
        Ok(Some(c))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &[&str]) -> Vec<String> {
        std::iter::once("skyward-server").chain(s.iter().copied()).map(String::from).collect()
    }

    #[test]
    fn flags() {
        let c = Config::from_args(&args(&["--name", "Friday", "-p", "5000", "--time", "21:30", "--day", "40", "--weather", "overcast", "--max-players", "99"])).unwrap().unwrap();
        assert_eq!(c.name, "Friday");
        assert_eq!(c.port, 5000);
        assert_eq!(c.time, TimeSync::Fixed { hours: 21.5, day: 40 });
        assert!(matches!(c.weather, Weather::Metar(ref m) if m.contains("OVC012")));
        assert_eq!(c.max_players, 64);
        assert!(Config::from_args(&args(&["--bogus"])).is_err());
        assert!(Config::from_args(&args(&["--time", "25:00"])).is_err());
        assert!(Config::from_args(&args(&["--port"])).is_err());
    }

    #[test]
    fn toml_file() {
        let dir = std::env::temp_dir().join(format!("skyward-server-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("server.toml");
        std::fs::write(&p, "name = \"LAN party\"\nport = 47900\nmax_players = 8\nairport = \"egkk\"\nweather = \"EGKK 121150Z 25015KT 9999 SCT045 22/10 Q1015\"\nmdns = false\n")
            .unwrap();
        let c = Config::from_args(&args(&["-c", p.to_str().unwrap(), "--port", "48000"])).unwrap().unwrap();
        assert_eq!(c.name, "LAN party");
        assert_eq!(c.port, 48000, "flags override the file");
        assert_eq!(c.airport, "EGKK");
        assert!(!c.mdns);
        assert!(matches!(c.weather, Weather::Metar(_)));
        std::fs::write(&p, "name = [1, 2]\n").unwrap();
        assert!(Config::from_args(&args(&["-c", p.to_str().unwrap()])).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
