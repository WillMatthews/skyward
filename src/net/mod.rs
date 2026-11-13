//! Multiplayer, game side. The wire protocol, reliability layer, clock sync,
//! interpolation and discovery live in `crates/skyward-net` (shared with the
//! dedicated server, `crates/skyward-server`); this module connects them to
//! the simulation, the renderer, the audio and the interface:
//!
//! * [`Mp`] — the session: connecting, the server's world sync (time and
//!   weather), the player's state out at the tick rate, other aircraft in
//!   ([`remote`]), chat, events, spectating.
//! * [`browse`] — the server browser's model (LAN discovery, favourites,
//!   recents, pings); the screen is `ui/browser.rs`, the in-flight overlay
//!   (name tags, chat, players, quick-chat wheel) `ui/mp.rs`.
//!
//! The browser build has no UDP: multiplayer is disabled there with a clear
//! message ([`WEB_MESSAGE`]).

pub mod browse;
pub mod capture;
pub mod combat;
pub mod prefs;
pub mod remote;

use crate::flight::AircraftDef;
use crate::sim::Sim;
use crate::world::World;
use glam::{Quat, Vec3, vec3};
use remote::Remote;
#[cfg(not(target_arch = "wasm32"))]
use skyward_net::client::ClientEvent;
use skyward_net::client::JoinRequest;
use skyward_net::proto::{GameEvent, Msg, SERVER_ID, TimeSync, WorldSync};
use std::collections::BTreeMap;
use web_time::Instant;

/// Shown instead of the server browser in the browser build.
pub const WEB_MESSAGE: &str = "Multiplayer needs the desktop version of Skyward — web browsers cannot send the UDP packets the game and the LAN discovery use.";

/// Canned messages of the quick-chat wheel (gamepad / Backspace), clockwise from the top.
pub const QUICK_CHAT: [&str; 8] = ["Hello!", "Ready for departure", "Follow me", "Join up on my wing", "Nice landing!", "Going around", "Taking off", "Good flight — see you!"];

#[derive(Clone, Debug, PartialEq)]
pub enum Status {
    Idle,
    /// Looking up the host name.
    Resolving,
    Connecting,
    Connected,
    /// Could not join (shown in the browser).
    Failed(String),
    /// The session ended (kicked, server shut down, timed out).
    Closed(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChatKind {
    Player,
    Me,
    Server,
    /// Joins, leaves, crashes (from the game).
    System,
}

#[derive(Clone, Debug)]
pub struct ChatLine {
    pub name: String,
    pub text: String,
    pub kind: ChatKind,
    /// Mp clock (s) when it arrived.
    pub at: f64,
}

/// The server we are on.
#[derive(Clone, Debug, Default)]
pub struct ServerView {
    /// As typed / found (the favourites / recents key).
    pub addr: String,
    pub name: String,
    pub motd: String,
    pub airport: String,
    pub region: String,
}

/// Quick-chat wheel state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wheel {
    pub sel: Option<usize>,
    /// Opened by holding a controller button (released = send).
    pub held: bool,
}

/// What the map, the ND and ATC need of another aircraft.
#[derive(Clone, Debug)]
pub struct Traffic {
    pub id: u16,
    pub name: String,
    pub aircraft: String,
    pub pos: Vec3,
    pub vel: Vec3,
    pub heading: f32,
    pub on_ground: bool,
}

/// The traffic picture of this frame for instruments drawn deep inside the
/// cockpit renderer (the ND's TCAS, `ui/mp.rs`), which only see the sim.
static PICTURE: std::sync::Mutex<Vec<Traffic>> = std::sync::Mutex::new(Vec::new());

/// Other aircraft this frame (empty when not connected).
pub fn traffic_picture() -> Vec<Traffic> {
    PICTURE.lock().map(|v| v.clone()).unwrap_or_default()
}

fn set_picture(v: Vec<Traffic>) {
    if let Ok(mut p) = PICTURE.lock() {
        *p = v;
    }
}

pub struct Mp {
    pub status: Status,
    /// Port for addresses typed without one (Settings › Multiplayer).
    pub default_port: u16,
    #[cfg(not(target_arch = "wasm32"))]
    conn: Option<skyward_net::udp::UdpClient>,
    #[cfg(not(target_arch = "wasm32"))]
    resolve: Option<std::sync::mpsc::Receiver<std::io::Result<std::net::SocketAddr>>>,
    req: Option<JoinRequest>,
    pub server: Option<ServerView>,
    pub me: u16,
    pub remotes: BTreeMap<u16, Remote>,
    pub chat: Vec<ChatLine>,
    /// Toasts for the app to show: (kind, title, body).
    pub notices: Vec<(crate::ui::toast::Kind, String, String)>,
    world: Option<(WorldSync, f64)>,
    applied_metar: String,
    pub spectate: Option<u16>,
    spec_yaw: Option<f32>,
    /// Name tags over other aircraft (Num *).
    pub tags: bool,
    /// Player list (F11).
    pub show_players: bool,
    /// Chat input open (Enter).
    pub chat_open: bool,
    /// UI frame the chat opened on (the Enter that opened it must not send).
    pub chat_open_frame: u64,
    pub chat_draft: String,
    pub wheel: Option<Wheel>,
    pub prefs: prefs::Prefs,
    /// This frame's camera (eye, view, vertical fov) for the name tags.
    pub cam: Option<(Vec3, glam::Mat4, f32)>,
    epoch: Instant,
    last_t: f64,
    // own-aircraft tracking (events)
    was_crashed: bool,
    last_pos: Option<Vec3>,
    last_aircraft: Option<(String, bool)>,
    /// Set when a join succeeds: the app starts the flight at the server's airport.
    pub start_flight: bool,
    /// Set when spectating stops: the app restarts the player at the start.
    pub want_restart: bool,
    defs: Vec<AircraftDef>,
    nonce: u64,
    /// Air combat (weapons/ ↔ the network, combat.rs) and its incoming messages.
    pub combat: combat::Link,
    combat_in: Vec<(u16, skyward_net::proto::CombatMsg)>,
}

impl Default for Mp {
    fn default() -> Self {
        Mp::new()
    }
}

impl Drop for Mp {
    /// Leaving the game leaves the server politely (the slot frees at once).
    fn drop(&mut self) {
        self.leave();
    }
}

impl Mp {
    pub fn new() -> Mp {
        let seed = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1);
        Mp {
            status: Status::Idle,
            #[cfg(not(target_arch = "wasm32"))]
            conn: None,
            #[cfg(not(target_arch = "wasm32"))]
            resolve: None,
            req: None,
            server: None,
            me: 0,
            remotes: BTreeMap::new(),
            chat: Vec::new(),
            notices: Vec::new(),
            world: None,
            applied_metar: String::new(),
            spectate: None,
            spec_yaw: None,
            tags: true,
            default_port: skyward_net::DEFAULT_PORT,
            show_players: false,
            chat_open: false,
            chat_open_frame: 0,
            chat_draft: String::new(),
            wheel: None,
            prefs: prefs::Prefs::load(),
            cam: None,
            epoch: Instant::now(),
            last_t: 0.0,
            was_crashed: false,
            last_pos: None,
            last_aircraft: None,
            start_flight: false,
            want_restart: false,
            defs: Vec::new(),
            nonce: seed ^ (std::process::id() as u64).rotate_left(32),
            combat: Default::default(),
            combat_in: Vec::new(),
        }
    }

    /// Seconds since this session object was made (the local clock).
    pub fn now(&self) -> f64 {
        self.epoch.elapsed().as_secs_f64()
    }

    pub fn connected(&self) -> bool {
        self.status == Status::Connected
    }

    /// Connecting or connected.
    pub fn active(&self) -> bool {
        matches!(self.status, Status::Resolving | Status::Connecting | Status::Connected)
    }

    fn defs(&mut self) -> &[AircraftDef] {
        if self.defs.is_empty() {
            self.defs = crate::flight::catalogue();
        }
        &self.defs
    }

    /// Server clock now (s).
    pub fn server_time(&self) -> f64 {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(c) = &self.conn {
            return c.client.server_time(self.now());
        }
        self.now()
    }

    pub fn ping_ms(&self) -> f32 {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(c) = &self.conn {
            return c.client.ping_ms();
        }
        0.0
    }

    /// Packet loss to the server (recent, 0..1).
    pub fn loss(&self) -> f32 {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(c) = &self.conn {
            return c.client.chan.loss(self.now());
        }
        0.0
    }

    /// Join `addr` ("host[:port]") as `name` flying `aircraft` (catalogue id).
    pub fn join(&mut self, addr: &str, password: &str, name: &str, aircraft: &str) {
        self.leave();
        let name = skyward_net::codec::sanitize(name, skyward_net::proto::MAX_NAME);
        self.req = Some(JoinRequest {
            name: if name.is_empty() { "Pilot".into() } else { name },
            aircraft: aircraft.into(),
            password: password.into(),
            version: skyward_net::VERSION.into(),
        });
        self.server = Some(ServerView { addr: addr.trim().to_string(), ..Default::default() });
        #[cfg(target_arch = "wasm32")]
        {
            self.status = Status::Failed(WEB_MESSAGE.into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (tx, rx) = std::sync::mpsc::channel();
            let a = addr.trim().to_string();
            let port = self.default_port;
            // (Name lookups block: off the frame.)
            let ok = std::thread::Builder::new()
                .name("mp-resolve".into())
                .spawn(move || {
                    let _ = tx.send(skyward_net::discovery::resolve(&a, port));
                })
                .is_ok();
            if ok {
                self.resolve = Some(rx);
                self.status = Status::Resolving;
            } else {
                self.status = Status::Failed("Could not start the connection.".into());
            }
        }
    }

    /// Leave the server (polite disconnect) and forget its aircraft.
    pub fn leave(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let now = self.now();
            if let Some(c) = self.conn.as_mut() {
                c.disconnect(now);
            }
            self.conn = None;
            self.resolve = None;
        }
        if self.active() {
            self.status = Status::Idle;
        }
        self.remotes.clear();
        set_picture(Vec::new());
        self.spectate = None;
        self.world = None;
        self.applied_metar.clear();
        self.chat_open = false;
        self.wheel = None;
        self.last_aircraft = None;
    }

    /// Clear a failure (the browser's error banner was dismissed).
    pub fn clear_error(&mut self) {
        if matches!(self.status, Status::Failed(_) | Status::Closed(_)) {
            self.status = Status::Idle;
        }
    }

    fn sys(&mut self, text: String, kind: ChatKind) {
        let at = self.now();
        let name = match kind {
            ChatKind::Server => "Server".to_string(),
            _ => String::new(),
        };
        self.chat.push(ChatLine { name, text, kind, at });
        if self.chat.len() > 200 {
            self.chat.drain(..50);
        }
    }

    /// Send a chat line.
    pub fn say(&mut self, text: &str) {
        let text = skyward_net::codec::sanitize(text, skyward_net::proto::MAX_CHAT);
        // (The server echoes it to everyone, us included: shown on arrival.)
        #[cfg(not(target_arch = "wasm32"))]
        if let (false, Some(c)) = (text.is_empty(), self.conn.as_mut()) {
            c.client.send(Msg::Chat { from: 0, text });
        }
        #[cfg(target_arch = "wasm32")]
        let _ = text;
    }

    pub fn name_of(&self, id: u16) -> String {
        if id == SERVER_ID {
            return "Server".into();
        }
        if id == self.me {
            return self.req.as_ref().map(|r| r.name.clone()).unwrap_or_default();
        }
        self.remotes.get(&id).map(|r| r.name.clone()).unwrap_or_else(|| format!("#{id}"))
    }

    /// Network work every frame (menus too): resolve, receive, keep-alive.
    pub fn poll(&mut self) {
        let now = self.now();
        self.last_t = now;
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(rx) = &self.resolve {
                match rx.try_recv() {
                    Ok(Ok(addr)) => {
                        self.resolve = None;
                        self.nonce = self.nonce.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                        match self.req.clone().map(|r| skyward_net::udp::UdpClient::connect(addr, r, now, self.nonce)) {
                            Some(Ok(c)) => {
                                eprintln!("multiplayer: connecting to {addr}");
                                self.conn = Some(c);
                                self.status = Status::Connecting;
                            }
                            Some(Err(e)) => self.status = Status::Failed(format!("Could not open a UDP socket: {e}")),
                            None => self.status = Status::Idle,
                        }
                    }
                    Ok(Err(e)) => {
                        self.resolve = None;
                        let a = self.server.as_ref().map(|s| s.addr.clone()).unwrap_or_default();
                        self.status = Status::Failed(format!("Could not find \"{a}\": {e}"));
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => {}
                    Err(_) => {
                        self.resolve = None;
                        self.status = Status::Failed("Address lookup failed.".into());
                    }
                }
            }
            let events = match self.conn.as_mut() {
                Some(c) => c.poll(now),
                None => Vec::new(),
            };
            for e in events {
                self.on_event(e);
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn on_event(&mut self, e: ClientEvent) {
        match e {
            ClientEvent::Connected { player, welcome } => {
                self.me = player;
                self.status = Status::Connected;
                self.start_flight = true;
                let addr = self.server.as_ref().map(|s| s.addr.clone()).unwrap_or_default();
                let i = &welcome.info;
                self.server = Some(ServerView { addr: addr.clone(), name: i.name.clone(), motd: welcome.motd.clone(), airport: i.airport.clone(), region: i.region.clone() });
                self.prefs.add_recent(&addr, &i.name);
                self.prefs.save();
                self.world = Some((welcome.world.clone(), self.now()));
                eprintln!("multiplayer: joined \"{}\" as #{player} ({} / {} players, airport {})", i.name, i.players, i.max_players, i.airport);
                self.sys(format!("Connected to {}", i.name), ChatKind::System);
                if !welcome.motd.is_empty() {
                    let m = welcome.motd.clone();
                    self.sys(m, ChatKind::Server);
                }
            }
            ClientEvent::Failed(f) => {
                self.conn = None;
                let msg = f.describe();
                eprintln!("multiplayer: {msg}");
                self.status = Status::Failed(msg);
            }
            ClientEvent::Closed { reason, text } => {
                self.conn = None;
                let mut msg = reason.describe().to_string();
                if !text.is_empty() && text != "left" {
                    msg = format!("{msg} {text}");
                }
                eprintln!("multiplayer: {msg}");
                self.notices.push((crate::ui::toast::Kind::Bad, "Disconnected".into(), msg.clone()));
                self.status = Status::Closed(msg);
                self.remotes.clear();
                set_picture(Vec::new());
                self.spectate = None;
            }
            ClientEvent::Msg(m) => self.on_msg(m),
        }
    }

    fn on_msg(&mut self, m: Msg) {
        match m {
            Msg::Snapshot { entries, .. } => {
                let t = self.server_time();
                for (id, s) in entries {
                    if let Some(r) = self.remotes.get_mut(&id) {
                        r.interp.push(s, t);
                    }
                }
            }
            Msg::Player(p) => {
                if p.id == self.me {
                    return;
                }
                let defs = self.defs().to_vec();
                match self.remotes.get_mut(&p.id) {
                    Some(r) => {
                        r.name = p.name;
                        r.spectating = p.spectating;
                        if r.aircraft != p.aircraft {
                            r.aircraft = p.aircraft;
                            r.load(&defs);
                        }
                    }
                    None => {
                        let mut r = Remote::new(p.id, p.name, p.aircraft, &defs);
                        r.spectating = p.spectating;
                        self.remotes.insert(p.id, r);
                    }
                }
            }
            Msg::Left { id, .. } => {
                self.remotes.remove(&id);
                if self.spectate == Some(id) {
                    self.stop_spectating();
                }
            }
            Msg::Chat { from, text } => {
                let kind = if from == SERVER_ID {
                    ChatKind::Server
                } else if from == self.me {
                    ChatKind::Me
                } else {
                    ChatKind::Player
                };
                let name = self.name_of(from);
                let at = self.now();
                self.chat.push(ChatLine { name, text, kind, at });
                if self.chat.len() > 200 {
                    self.chat.drain(..50);
                }
            }
            Msg::World(w) => self.world = Some((w, self.now())),
            Msg::Combat { from, msg } => {
                if from != self.me {
                    self.combat_in.push((from, msg));
                    if self.combat_in.len() > 4096 {
                        self.combat_in.drain(..1024);
                    }
                }
            }
            Msg::Event { from, event } => {
                let name = self.name_of(from);
                match event {
                    GameEvent::Crash { severity } => {
                        let what = match severity {
                            0 => "had an accident",
                            1 => "crashed",
                            _ => "crashed — aircraft destroyed",
                        };
                        self.sys(format!("{name} {what}"), ChatKind::System);
                        self.notices.push((crate::ui::toast::Kind::Warn, format!("{name} {what}"), String::new()));
                    }
                    GameEvent::Respawn => {
                        if let Some(r) = self.remotes.get_mut(&from) {
                            r.interp.reset();
                        }
                    }
                    GameEvent::Eject => self.sys(format!("{name} ejected"), ChatKind::System),
                    GameEvent::Separation { .. } => {}
                }
            }
            _ => {}
        }
    }

    /// The server's time and weather → the world and the sim.
    fn apply_world(&mut self, sim: &mut Sim, world: &mut World) {
        let now = self.now();
        let Some((w, at)) = self.world.clone() else { return };
        let elapsed = (now - at).max(0.0);
        match w.time {
            TimeSync::Real => {
                world.live_time = true;
                world.live_date = true;
                let local = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
                if w.utc > 0.0 {
                    world.clock_skew_s = (w.utc + elapsed - local).round() as i64;
                }
            }
            TimeSync::Fixed { hours, day } => {
                world.live_time = false;
                world.live_date = false;
                let h = hours as f64 + elapsed / 3600.0;
                sim.hours = h.rem_euclid(24.0) as f32;
                sim.day_of_year = ((day as u32 - 1 + (h / 24.0).floor() as u32) % 365) + 1;
            }
        }
        if !w.metar.is_empty() && w.metar != self.applied_metar {
            self.applied_metar = w.metar.clone();
            match crate::weather::Wx::from_metar_str(&w.metar) {
                Some(wx) => {
                    eprintln!("multiplayer: server weather {}", w.metar);
                    world.set_wx(wx);
                }
                None => eprintln!("multiplayer: could not decode the server's METAR: {}", w.metar),
            }
        } else if !w.metar.is_empty() {
            // The server's weather is authoritative: never the client's own live fetch.
            world.live_weather = false;
        }
    }

    /// While flying: world sync, the player's state out, events, remote poses.
    /// `paused`: the sim is paused / the menu is open (others see us frozen).
    pub fn fly(&mut self, sim: &mut Sim, world: &mut World, paused: bool, dt: f32) {
        if !self.connected() {
            // Left the server: the fight goes back to single player.
            if !self.combat.units.is_empty() {
                if let Some(c) = sim.combat.as_deref_mut() {
                    self.combat.clear(c);
                }
                self.combat.units.clear();
            }
            return;
        }
        // One shared timeline: no time warp in multiplayer.
        sim.warp = 1;
        self.apply_world(sim, world);
        let spectating = self.spectate.is_some();
        // Spectating: our (paused, hidden) aircraft rides along with the target,
        // so the scenery streams and the floating origin follow it.
        if let Some(id) = self.spectate {
            match self.remotes.get(&id).filter(|r| r.cur.is_some()) {
                Some(r) => {
                    sim.paused = true;
                    sim.ac.pos = r.pos;
                    sim.ac.vel = r.vel;
                    sim.ac.rot = r.rot;
                }
                None if !self.remotes.contains_key(&id) => self.stop_spectating(),
                None => {}
            }
        }
        // Events from our own aircraft.
        let crashed = sim.ac.crashed.is_some();
        let pos = sim.ac.pos;
        let def_id = sim.def().id.to_string();
        let mut out: Vec<Msg> = Vec::new();
        if crashed && !self.was_crashed {
            let severity = match sim.severity {
                Some(crate::damage::Severity::Destroyed) => 2,
                Some(crate::damage::Severity::Substantial) => 1,
                _ => 0,
            };
            out.push(Msg::Event { from: 0, event: GameEvent::Crash { severity } });
        }
        let jumped = self.last_pos.is_some_and(|p| capture::teleported(p, pos, dt));
        if !spectating && ((self.was_crashed && !crashed) || jumped) {
            out.push(Msg::Event { from: 0, event: GameEvent::Respawn });
        }
        if self.last_aircraft.as_ref() != Some(&(def_id.clone(), spectating)) {
            out.push(Msg::SetAircraft { aircraft: def_id.clone(), spectating });
            self.last_aircraft = Some((def_id, spectating));
        }
        self.was_crashed = crashed;
        self.last_pos = Some(pos);
        let now = self.now();
        let state = capture::own_state(sim, world, paused && !spectating, spectating);
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(c) = self.conn.as_mut() {
            for m in out {
                c.client.send(m);
            }
            c.client.set_state(state, now);
        }
        #[cfg(target_arch = "wasm32")]
        let _ = (out, state, now);
        self.sample(world, dt);
        self.fight(sim);
    }

    /// Air combat: the other players as units of the local fight, their
    /// weapons / hits / damage in, ours out (net/combat.rs).
    fn fight(&mut self, sim: &mut Sim) {
        let on = self.world.as_ref().map(|w| w.0.combat).unwrap_or(true);
        sim.ensure_combat();
        let me = self.me;
        let remotes: Vec<(u16, usize, Vec3, Quat, Vec3, bool)> =
            self.remotes.values().map(|r| (r.id, r.def, r.pos, r.rot, r.vel, r.cur.is_some() && !r.spectating && !r.crashed)).collect();
        let names: BTreeMap<u16, String> = self.remotes.values().map(|r| (r.id, r.name.clone())).collect();
        let incoming = std::mem::take(&mut self.combat_in);
        let def = sim.def().clone();
        let Sim { combat, ac, .. } = sim;
        let Some(c) = combat.as_deref_mut() else { return };
        self.combat.sync(c, on, &remotes);
        self.combat.name(c, &names);
        for (from, m) in incoming {
            self.combat.incoming(c, me, from, m, Some((&mut *ac, &def)));
        }
        let out = self.combat.outgoing(c, me);
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(conn) = self.conn.as_mut() {
            for msg in out {
                conn.client.send(Msg::Combat { from: 0, msg });
            }
        }
        #[cfg(target_arch = "wasm32")]
        let _ = out;
    }

    /// Interpolate every remote aircraft to the playout time.
    fn sample(&mut self, world: &World, dt: f32) {
        let t = self.server_time();
        let defs = self.defs().to_vec();
        for r in self.remotes.values_mut() {
            r.poll_model(&defs);
            r.sample(t, dt as f64, &world.proj);
        }
        set_picture(self.traffic());
    }

    /// Other aircraft for the renderer (eye-relative): model draw lists, light
    /// sprites and afterburner flames.
    pub fn render_traffic(&mut self, eye: Vec3) -> (Vec<crate::gfx_aircraft::AircraftFrame>, Vec<crate::lights::LightInst>, crate::scene::MeshData) {
        let mut frames = Vec::new();
        let mut lights = Vec::new();
        let mut flames = crate::scene::MeshData::default();
        if !self.connected() {
            return (frames, lights, flames);
        }
        // One flashing / animation clock for everyone (the server's).
        let time = (self.server_time() % 3600.0) as f32;
        let defs = self.defs().to_vec();
        for r in self.remotes.values_mut() {
            if let Some(f) = r.frame(&defs, eye, time, &mut lights, &mut flames) {
                frames.push(f);
            }
        }
        (frames, lights, flames)
    }

    /// Other aircraft (current world frame) for the map, ND, ATC and tags.
    pub fn traffic(&self) -> Vec<Traffic> {
        self.remotes
            .values()
            .filter(|r| r.visible())
            .map(|r| Traffic {
                id: r.id,
                name: r.name.clone(),
                aircraft: r.aircraft.clone(),
                pos: r.pos,
                vel: r.vel,
                heading: r.heading(),
                on_ground: r.state().is_some_and(|s| s.has(skyward_net::proto::flags::ON_GROUND)),
            })
            .collect()
    }

    // ------------------------------------------------------------ spectating

    pub fn start_spectating(&mut self, id: u16) {
        if self.remotes.contains_key(&id) {
            self.spectate = Some(id);
            self.spec_yaw = None;
            self.show_players = false;
        }
    }

    /// Back to flying: the player restarts at the start position.
    pub fn stop_spectating(&mut self) {
        if self.spectate.take().is_some() {
            self.want_restart = true;
        }
    }

    /// Next / previous aircraft to watch.
    pub fn cycle_spectate(&mut self, dir: i32) {
        let ids: Vec<u16> = self.remotes.values().filter(|r| r.visible()).map(|r| r.id).collect();
        if ids.is_empty() {
            return;
        }
        let i = self.spectate.and_then(|s| ids.iter().position(|&x| x == s)).map(|i| (i as i32 + dir).rem_euclid(ids.len() as i32) as usize).unwrap_or(0);
        self.spectate = Some(ids[i]);
        self.spec_yaw = None;
    }

    /// The spectator camera: a chase view behind the watched aircraft.
    pub fn spectate_view(&mut self, dt: f32) -> Option<(Vec3, Vec3)> {
        let r = self.remotes.get(&self.spectate?)?;
        r.cur.as_ref()?;
        let fwd = r.rot * Vec3::Z;
        let yaw_t = fwd.x.atan2(-fwd.z);
        let yaw = match self.spec_yaw {
            Some(y) => {
                let d = crate::ui::wrap_pi(yaw_t - y);
                y + d * (1.0 - (-dt / 0.35).exp())
            }
            None => yaw_t,
        };
        self.spec_yaw = Some(yaw);
        let back = vec3(-yaw.sin(), 0.0, yaw.cos());
        let dist = r.size * 2.6 + 6.0;
        let p = 12f32.to_radians();
        let mut eye = r.pos + (back * p.cos() + Vec3::Y * p.sin()) * dist;
        eye.y = eye.y.max(r.pos.y.min(eye.y) + 0.5);
        Some((eye, r.pos + Vec3::Y * (0.5 + r.size * 0.08)))
    }

    pub fn spectating_name(&self) -> Option<String> {
        self.spectate.map(|id| self.name_of(id))
    }

    // ------------------------------------------------------------ audio

    /// The nearest other aircraft's engines for the mixer.
    pub fn sounds(&self, ear: Vec3) -> [crate::audio::params::TrafficFrame; crate::audio::params::MAX_TRAFFIC] {
        use crate::audio::params::{MAX_TRAFFIC, TrafficFrame};
        let mut out = [TrafficFrame::default(); MAX_TRAFFIC];
        if !self.connected() {
            return out;
        }
        let mut near: Vec<&Remote> = self.remotes.values().filter(|r| r.visible() && r.pos.distance(ear) < 6000.0).collect();
        near.sort_by(|a, b| a.pos.distance_squared(ear).total_cmp(&b.pos.distance_squared(ear)));
        for (slot, r) in out.iter_mut().zip(near) {
            let Some(st) = r.state() else { continue };
            let Some(def) = self.defs.get(r.def) else { continue };
            let mut profile = crate::audio::capture::profile_for(def, r.size);
            // One voice per aircraft (its engines together).
            profile.engines = 1;
            let n = (st.n_engines as usize).max(1);
            let e = &st.engines[0];
            let eng = crate::audio::params::EngineFrame {
                pos: r.pos,
                rpm: e.rpm,
                spool: e.spool,
                thrust: e.thrust,
                ab: st.afterburner,
                running: e.running && !r.crashed,
                throttle: st.throttle,
            };
            let rotor = crate::audio::params::RotorFrame {
                nr_rpm: st.rotor.nr_rpm,
                nr: st.rotor.nr_pct / 100.0,
                ng: e.spool,
                torque: e.spool,
                power: e.spool,
                tail_rpm: st.rotor.tail_rpm,
                ..Default::default()
            };
            *slot =
                TrafficFrame { id: r.id as u32 + 1, profile, n_engines: n as u8, eng, pos: r.pos, vel: r.vel, axis: r.rot * Vec3::Z, tas: st.tas, reverser: st.reverser, rotor };
        }
        out
    }

    /// For the HUD's rotation of the watched aircraft (spectator instruments).
    pub fn spectated(&self) -> Option<(&Remote, Quat)> {
        let r = self.remotes.get(&self.spectate?)?;
        Some((r, r.rot))
    }
}
