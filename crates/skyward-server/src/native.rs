//! The server process: sockets, the main loop, the console, live METAR,
//! signals.

#[path = "config.rs"]
mod config;
#[path = "log.rs"]
mod log;

use config::{Config, Weather};
use log::Logger;
use skyward_net::discovery::{MdnsResponder, bind_shared};
use skyward_net::proto::{DISCOVERY_PORT, MAX_RECV, TimeSync, WorldSync};
use skyward_net::server::{Server, ServerConfig};
use std::io::BufRead;
use std::net::{IpAddr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

const METAR_EVERY: Duration = Duration::from_secs(600);
const STATUS_EVERY: f64 = 300.0;

fn seed() -> [u8; 32] {
    // Process-unique and unpredictable enough for cookies / tokens: OS
    // randomness where there is some, plus time, pid and an address, mixed
    // through SHA-256.
    use std::io::Read;
    let mut s = Vec::new();
    let mut r = [0u8; 32];
    if std::fs::File::open("/dev/urandom").and_then(|mut f| f.read_exact(&mut r)).is_ok() {
        s.extend_from_slice(&r);
    }
    s.extend_from_slice(&std::process::id().to_le_bytes());
    s.extend_from_slice(format!("{:?}{:?}", std::time::SystemTime::now(), Instant::now()).as_bytes());
    let x = Box::new(0u8);
    s.extend_from_slice(&(&*x as *const u8 as usize).to_le_bytes());
    skyward_net::sha256::digest(&s)
}

/// Fetch the airport's METAR (blocking; run off the main loop).
fn fetch_metar(icao: &str) -> Result<String, String> {
    let url = format!("https://aviationweather.gov/api/data/metar?ids={icao}&format=raw");
    let body = ureq::get(&url).timeout(Duration::from_secs(15)).call().map_err(|e| e.to_string())?.into_string().map_err(|e| e.to_string())?;
    let line = body.lines().map(str::trim).find(|l| !l.is_empty()).ok_or_else(|| format!("no METAR for {icao}"))?;
    let line = line.trim_start_matches("METAR ").trim_start_matches("SPECI ").trim();
    if line.split_whitespace().count() < 3 {
        return Err(format!("unexpected answer for {icao}: {line}"));
    }
    Ok(line.chars().take(250).collect())
}

fn load_bans(path: &str, log: &Logger) -> Vec<IpAddr> {
    let Ok(text) = std::fs::read_to_string(path) else { return Vec::new() };
    let mut v = Vec::new();
    for (i, l) in text.lines().enumerate() {
        let l = l.split('#').next().unwrap_or("").trim();
        if l.is_empty() {
            continue;
        }
        match l.parse::<IpAddr>() {
            Ok(ip) => v.push(ip),
            Err(_) => log.warn(&format!("{path}:{}: not an IP address: {l}", i + 1)),
        }
    }
    v
}

fn save_bans(path: &str, bans: &std::collections::HashSet<IpAddr>, log: &Logger) {
    let mut v: Vec<String> = bans.iter().map(|b| b.to_string()).collect();
    v.sort();
    let text = format!("# skyward-server bans (one address per line)\n{}\n", v.join("\n"));
    if let Err(e) = std::fs::write(path, text) {
        log.warn(&format!("could not save bans to {path}: {e}"));
    }
}

fn txt(server: &Server) -> Vec<String> {
    let i = server.server_info();
    vec![
        format!("proto={}", skyward_net::PROTOCOL),
        format!("v={}", skyward_net::VERSION),
        format!("name={}", i.name),
        format!("players={}/{}", i.players, i.max_players),
        format!("airport={}", i.airport),
        format!("pw={}", i.password as u8),
    ]
}

fn uptime(s: f64) -> String {
    let s = s as u64;
    format!("{}h{:02}m", s / 3600, s / 60 % 60)
}

pub fn run() -> i32 {
    let args: Vec<String> = std::env::args().collect();
    let cfg = match Config::from_args(&args) {
        Ok(Some(c)) => c,
        Ok(None) => return 0,
        Err(e) => {
            eprintln!("skyward-server: {e}");
            return 2;
        }
    };
    let log = Logger::new(&cfg.log_level, cfg.log_json);
    let started = Instant::now();
    let now = || started.elapsed().as_secs_f64();

    // Game socket (blocking with a short timeout: the loop's clock).
    let game_addr = SocketAddr::new(cfg.bind, cfg.port);
    let game = match UdpSocket::bind(game_addr) {
        Ok(s) => s,
        Err(e) => {
            log.warn(&format!("cannot listen on UDP {game_addr}: {e} (another server on this port? try --port)"));
            return 1;
        }
    };
    let _ = game.set_read_timeout(Some(Duration::from_millis(4)));
    let port = game.local_addr().map(|a| a.port()).unwrap_or(cfg.port);

    let scfg = ServerConfig {
        name: cfg.name.clone(),
        max_players: cfg.max_players,
        password: cfg.password.clone(),
        motd: cfg.motd.clone(),
        airport: cfg.airport.clone(),
        region: cfg.region.clone(),
        tick_hz: cfg.tick_hz,
        world: cfg.world(),
        version: skyward_net::VERSION.into(),
        game_port: port,
    };
    let mut server = Server::new(scfg, seed(), now());
    if let Some(p) = &cfg.ban_file {
        for ip in load_bans(p, &log) {
            server.bans.insert(ip);
        }
    }

    // LAN discovery (shared port) on the same address family / interface as the game.
    let discovery = match (cfg.discovery, cfg.bind) {
        (true, IpAddr::V4(ip)) => match bind_shared(SocketAddrV4::new(ip, DISCOVERY_PORT)) {
            Ok(s) => Some(s),
            Err(e) => {
                log.warn(&format!("LAN discovery off: cannot listen on UDP {ip}:{DISCOVERY_PORT}: {e}"));
                None
            }
        },
        _ => None,
    };
    let loopback_only = cfg.bind.is_loopback();
    let mut mdns = if cfg.mdns && !loopback_only {
        let host = format!("skyward-{:08x}", server.instance as u32);
        match MdnsResponder::new(&cfg.name, &host, port) {
            Ok(m) => Some(m),
            Err(e) => {
                log.warn(&format!("mDNS off: {e}"));
                None
            }
        }
    } else {
        None
    };

    // Ctrl-C / SIGTERM → graceful shutdown.
    let stop = Arc::new(AtomicBool::new(false));
    for sig in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        if let Err(e) = signal_hook::flag::register(sig, stop.clone()) {
            log.warn(&format!("signal handler: {e}"));
        }
    }

    // Console.
    let (cmd_tx, cmd_rx) = mpsc::channel::<String>();
    if cfg.console {
        std::thread::Builder::new()
            .name("console".into())
            .spawn(move || {
                let stdin = std::io::stdin();
                for line in stdin.lock().lines() {
                    let Ok(l) = line else { break };
                    if cmd_tx.send(l).is_err() {
                        break;
                    }
                }
            })
            .ok();
    }

    // Live METAR.
    let (wx_tx, wx_rx) = mpsc::channel::<Result<String, String>>();
    let live_weather = Arc::new(AtomicBool::new(cfg.weather == Weather::Live));
    {
        let icao = cfg.airport.clone();
        let live = live_weather.clone();
        let stop = stop.clone();
        std::thread::Builder::new()
            .name("metar".into())
            .spawn(move || {
                let mut next = Instant::now();
                while !stop.load(Ordering::Relaxed) {
                    if live.load(Ordering::Relaxed) && Instant::now() >= next {
                        next = Instant::now() + METAR_EVERY;
                        if wx_tx.send(fetch_metar(&icao)).is_err() {
                            break;
                        }
                    }
                    std::thread::sleep(Duration::from_millis(250));
                }
            })
            .ok();
    }

    let ips: Vec<String> =
        if cfg.bind.is_unspecified() { skyward_net::ifaces::ipv4().into_iter().filter(|i| !i.loopback).map(|i| i.addr.to_string()).collect() } else { vec![cfg.bind.to_string()] };
    log.info(&format!(
        "\"{}\" listening on UDP {}:{port} (LAN address{} {}); discovery {}, mDNS {}; {} slots, airport {}, {}{}",
        cfg.name,
        cfg.bind,
        if ips.len() == 1 { "" } else { "es" },
        if ips.is_empty() { "none".to_string() } else { ips.join(", ") },
        if discovery.is_some() { format!("on UDP {DISCOVERY_PORT}") } else { "off".into() },
        if mdns.is_some() { "on" } else { "off" },
        cfg.max_players,
        cfg.airport,
        server.cfg.world.summary(),
        if cfg.password.is_empty() { "" } else { ", password set" },
    ));
    if cfg.console {
        log.info("type `help` for console commands");
    }

    let mut buf = [0u8; 2048];
    let mut next_status = STATUS_EVERY;
    let mut last_metar = String::new();
    loop {
        if stop.load(Ordering::Relaxed) {
            log.info("shutting down (signal)");
            break;
        }
        let t = now();
        // Game traffic (the read timeout paces the loop when idle).
        for k in 0..512 {
            if k > 0 {
                let _ = game.set_nonblocking(true);
            }
            match game.recv_from(&mut buf) {
                Ok((n, from)) if n <= MAX_RECV => server.handle(from, &buf[..n], t),
                Ok(_) => {}
                Err(_) => break,
            }
        }
        let _ = game.set_nonblocking(false);
        let t = now();
        // Discovery broadcasts: answer from the discovery socket (the reply carries the game port).
        if let Some(d) = &discovery {
            for _ in 0..64 {
                let Ok((n, from)) = d.recv_from(&mut buf) else { break };
                if n > MAX_RECV {
                    continue;
                }
                let before = server.out.len();
                server.handle(from, &buf[..n], t);
                for (to, b) in server.out.drain(before..).collect::<Vec<_>>() {
                    let _ = d.send_to(&b, to);
                }
            }
        }
        if let Some(m) = mdns.as_mut() {
            let tx = txt(&server);
            m.poll(t, &tx);
        }
        server.tick(t);
        for (to, b) in server.out.drain(..) {
            let _ = game.send_to(&b, to);
        }
        for e in server.log.drain(..) {
            log.log(e.level, &e.msg);
        }
        // Live weather.
        while let Ok(r) = wx_rx.try_recv() {
            match r {
                Ok(m) if live_weather.load(Ordering::Relaxed) => {
                    if m != last_metar {
                        log.info(&format!("weather: {m}"));
                        last_metar = m.clone();
                        let mut w = server.cfg.world.clone();
                        w.metar = m;
                        w.live_weather = true;
                        server.set_world(w, t);
                    }
                }
                Ok(_) => {}
                Err(e) => log.warn(&format!("live weather: {e} (keeping the last one)")),
            }
        }
        // Console commands.
        while let Ok(line) = cmd_rx.try_recv() {
            match command(&mut server, &line, t, &log, &cfg, &live_weather) {
                Cmd::Quit => {
                    stop.store(true, Ordering::Relaxed);
                }
                Cmd::Reply(r) => {
                    for l in r.lines() {
                        println!("{l}");
                    }
                }
                Cmd::BansChanged(r) => {
                    println!("{r}");
                    if let Some(p) = &cfg.ban_file {
                        save_bans(p, &server.bans, &log);
                    }
                }
            }
        }
        if t >= next_status {
            next_status = t + STATUS_EVERY;
            let s = server.stats;
            log.info(&format!(
                "status: {} / {} players, up {}, in {:.1} kB/s, out {:.1} kB/s, {} malformed, {} rate-limited",
                server.peers.len(),
                cfg.max_players,
                uptime(t),
                s.bytes_in as f64 / 1e3 / t.max(1.0),
                s.bytes_out as f64 / 1e3 / t.max(1.0),
                s.malformed,
                s.rate_limited
            ));
        }
        if stop.load(Ordering::Relaxed) {
            break;
        }
    }
    // Graceful: tell the players and the LAN.
    server.shutdown("The server shut down.");
    for (to, b) in server.out.drain(..) {
        let _ = game.send_to(&b, to);
    }
    if let Some(m) = &mdns {
        m.goodbye(&txt(&server));
    }
    for e in server.log.drain(..) {
        log.log(e.level, &e.msg);
    }
    log.info(&format!("stopped after {}", uptime(now())));
    0
}

enum Cmd {
    Reply(String),
    BansChanged(String),
    Quit,
}

fn command(server: &mut Server, line: &str, now: f64, log: &Logger, cfg: &Config, live: &AtomicBool) -> Cmd {
    let line = line.trim();
    let (cmd, rest) = line.split_once(char::is_whitespace).map(|(a, b)| (a, b.trim())).unwrap_or((line, ""));
    let reply = |s: String| Cmd::Reply(s);
    match cmd.to_ascii_lowercase().as_str() {
        "" => reply(String::new()),
        "help" | "?" => reply(
            "list                      players (id, name, aircraft, address, ping, time)\n\
             kick <player> [reason]    disconnect a player (by #id or name)\n\
             ban <player|ip> [reason]  kick and ban the address\n\
             unban <ip> · bans         lift a ban · list bans\n\
             say <text>                chat to everyone as the server\n\
             time real|HH:MM [day]     set the time of day for everyone\n\
             weather live|own|<preset>|<METAR>   presets: clear few scattered broken overcast rain fog\n\
             motd <text>               message of the day\n\
             status                    players, traffic, uptime\n\
             quit                      shut down (players are told)"
                .into(),
        ),
        "list" | "players" | "who" => {
            let rows = server.list(now);
            if rows.is_empty() {
                return reply("no players".into());
            }
            let mut s = format!("{} / {} players:", rows.len(), server.cfg.max_players);
            for (id, name, ac, addr, ping, secs) in rows {
                s.push_str(&format!("\n  #{id:<4} {name:<24} {ac:<14} {addr:<22} {ping:>4} ms  {}", uptime(secs as f64)));
            }
            reply(s)
        }
        "kick" | "ban" => {
            if rest.is_empty() {
                return reply(format!("usage: {cmd} <player> [reason]"));
            }
            // "kick Bob the Builder too fast": try the longest player-name prefix first.
            let words: Vec<&str> = rest.split_whitespace().collect();
            let mut target = None;
            for k in (1..=words.len()).rev() {
                let who = words[..k].join(" ");
                if server.find(&who).is_some() || (cmd == "ban" && k == 1 && who.parse::<IpAddr>().is_ok()) {
                    target = Some((who, words[k..].join(" ")));
                    break;
                }
            }
            let Some((who, reason)) = target else { return reply(format!("no player \"{rest}\" (see list)")) };
            match server.kick(&who, &reason, cmd == "ban") {
                Ok(m) if cmd == "ban" => Cmd::BansChanged(m),
                Ok(m) => reply(m),
                Err(e) => reply(e),
            }
        }
        "unban" => match rest.parse::<IpAddr>() {
            Ok(ip) if server.bans.remove(&ip) => {
                log.info(&format!("unbanned {ip}"));
                Cmd::BansChanged(format!("unbanned {ip}"))
            }
            Ok(ip) => reply(format!("{ip} is not banned")),
            Err(_) => reply("usage: unban <ip>".into()),
        },
        "bans" => {
            let mut v: Vec<String> = server.bans.iter().map(|b| b.to_string()).collect();
            v.sort();
            reply(if v.is_empty() { "no bans".into() } else { v.join("\n") })
        }
        "say" => {
            server.say(rest);
            log.info(&format!("chat <server> {rest}"));
            reply(String::new())
        }
        "motd" => {
            server.cfg.motd = rest.chars().take(skyward_net::proto::MAX_MOTD).collect();
            reply(format!("motd: {}", server.cfg.motd))
        }
        "time" => {
            let mut it = rest.split_whitespace();
            let t = it.next().unwrap_or("");
            let day = it.next().and_then(|d| d.parse::<u16>().ok()).map(|d| d.clamp(1, 366));
            let mut c = cfg.clone();
            c.time = server.cfg.world.time.clone();
            match c.parse_time(t, day) {
                Ok(ts) => {
                    let mut w = server.cfg.world.clone();
                    w.time = ts;
                    let s = describe_time(&w.time);
                    server.set_world(w, now);
                    log.info(&format!("time set: {s}"));
                    reply(format!("time: {s}"))
                }
                Err(e) => reply(e),
            }
        }
        "weather" => match cfg.parse_weather(rest) {
            Ok(wx) => {
                let mut w: WorldSync = server.cfg.world.clone();
                live.store(wx == Weather::Live, Ordering::Relaxed);
                match &wx {
                    Weather::Live => {
                        w.live_weather = true;
                        // (The next fetch fills the METAR; until then keep the current one.)
                    }
                    Weather::Own => {
                        w.metar.clear();
                        w.live_weather = false;
                    }
                    Weather::Metar(m) => {
                        w.metar = m.clone();
                        w.live_weather = false;
                    }
                }
                let s = w.summary();
                server.set_world(w, now);
                log.info(&format!("weather set: {s}"));
                reply(format!("weather: {s}"))
            }
            Err(e) => reply(e),
        },
        "status" => {
            let s = server.stats;
            reply(format!(
                "{} / {} players · up {} · {} · in {} kB / out {} kB · {} malformed · {} rate-limited",
                server.peers.len(),
                server.cfg.max_players,
                uptime(now),
                server.cfg.world.summary(),
                s.bytes_in / 1000,
                s.bytes_out / 1000,
                s.malformed,
                s.rate_limited
            ))
        }
        "quit" | "exit" | "stop" | "shutdown" => {
            log.info("shutting down (console)");
            Cmd::Quit
        }
        other => reply(format!("unknown command \"{other}\" — try help")),
    }
}

fn describe_time(t: &TimeSync) -> String {
    match t {
        TimeSync::Real => "real time".into(),
        TimeSync::Fixed { hours, day } => format!("{:02}:{:02} local, day {day}", *hours as u32, (hours.fract() * 60.0).round() as u32 % 60),
    }
}
