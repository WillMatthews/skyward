//! Live real-world air traffic from ADS-B (adsb.lol, ODbL 1.0).
//!
//! The aircraft around the player, as reported by the volunteer receivers
//! of adsb.lol, flown by stand-in models from the catalogue:
//!
//! * [`feed`] — the API and its JSON; [`fetch`] — the polite poller and the
//!   worker thread (desktop; the browser build cannot call the API: no CORS).
//! * [`playout`] — smooth motion: buffered playout ~10–15 s behind the feed,
//!   Hermite interpolation between reports, dead reckoning when late,
//!   decaying corrections (the reasoning is in that module).
//! * [`synth`] — phase, attitude, gear / flaps / spoilers / reversers,
//!   engines and lights from the motion.
//! * [`types`] — ICAO type → stand-in model, scale, wake category.
//! * [`ground`] — taxiway / runway / stand snapping on the ground.
//! * [`ui`] — moving-map layer and name tags.
//!
//! Drawing goes through the multiplayer remote-aircraft path
//! (`net::remote::Remote`: shared model cache, per-aircraft renderer
//! slots, light sprites); the ND's TCAS, the map, ATC traffic information
//! and the traffic engine sounds read the same picture as multiplayer
//! traffic. Multiplayer players take precedence for render and sound slots.
//! Everything is client-side: each player fetches their own.
//!
//! **Wake turbulence hook** (for the weather track): [`LiveTraffic::wake_sources`]
//! lists every live aircraft's position, velocity, span and wake category.

pub mod feed;
pub mod fetch;
pub mod ground;
pub mod playout;
pub mod synth;
pub mod types;
pub mod ui;

use crate::flight::{AircraftDef, Environment};
use crate::net::Traffic;
use crate::net::remote::Remote;
use crate::world::World;
use feed::Snapshot;
use glam::{Quat, Vec2, Vec3, vec2, vec3};
use skyward_net::proto::AcState;
use std::collections::HashMap;

/// Attribution (ODbL requires it wherever the data is shown).
pub const ATTRIBUTION: &str = "Live traffic: adsb.lol contributors, ODbL 1.0";

/// Aircraft drawn with full models (nearest first); the rest get light sprites.
pub const MODEL_BUDGET: usize = 32;
/// Model draw distance, m; lights alone out to [`LIGHTS_DIST`].
pub const MODEL_DIST: f32 = 25_000.0;
pub const LIGHTS_DIST: f32 = 40_000.0;
/// Reports this old (feed clock vs. playout) drop the aircraft, s.
const EXPIRE: f64 = playout::MAX_EXTRAPOLATE + 15.0;

/// Player-facing options (persisted in settings.json, `ui/settings.rs`).
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub enabled: bool,
    /// Query radius, NM.
    pub radius_nm: f32,
    /// Most aircraft shown at once (nearest first).
    pub max_aircraft: usize,
    pub ground: bool,
    /// Callsign tags over the aircraft.
    pub tags: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { enabled: !crate::platform::WEB, radius_nm: 50.0, max_aircraft: 150, ground: true, tags: true }
    }
}

pub const RADII_NM: [f32; 4] = [25.0, 50.0, 100.0, 150.0];
pub const BUDGETS: [usize; 4] = [50, 100, 150, 250];

/// One aircraft's wake for the weather / wake-turbulence model.
#[derive(Clone, Copy, Debug)]
pub struct WakeSource {
    /// Current world frame, m.
    pub pos: Vec3,
    pub vel: Vec3,
    pub span: f32,
    pub wake: types::Wake,
    pub on_ground: bool,
}

/// The world as the traffic needs it (the game's `World`, or a test stand-in).
pub struct Env<'a> {
    pub proj: crate::osm::Proj,
    pub qnh_hpa: f32,
    pub runways: &'a [crate::scene::Runway],
    pub env: &'a dyn Environment,
    pub layouts: Vec<std::sync::Arc<crate::airport::layout::Layout>>,
}

impl<'a> Env<'a> {
    pub fn of(world: &'a World) -> Env<'a> {
        Env { proj: world.proj, qnh_hpa: world.wx.qnh_hpa, runways: &world.runways, env: world, layouts: world.airport_detail.layouts() }
    }
}

/// The player, for the query and the distances.
#[derive(Clone, Copy, Debug, Default)]
pub struct Me {
    pub pos: Vec3,
    pub vel: Vec3,
    /// Simulation clock (replays run on it), s.
    pub sim_time: f64,
}

/// Per-type data (from the catalogue def of the stand-in).
#[derive(Clone)]
struct TypeInfo {
    def: usize,
    contacts: Vec<Vec3>,
    rest_pitch: f32,
    /// Reference point above the wheels at rest, m.
    gear_h: f32,
    /// Distance from the reference point forward to the nose wheel, m.
    nose: f32,
    length: f32,
}

/// One live aircraft.
pub struct Live {
    pub hex: u32,
    pub callsign: String,
    pub icao_type: String,
    pub category: String,
    pub stand: types::Stand,
    pub squawk: Option<u16>,
    pub emergency: Option<String>,
    key: String,
    track: playout::Track,
    pub anim: synth::Anim,
    snap_off: Vec2,
    /// This frame (current world frame).
    pub pos: Vec3,
    pub rot: Quat,
    pub vel: Vec3,
    pub on_ground: bool,
    pub state: AcState,
    pub shown: bool,
    pub dist: f32,
    /// Stand it is parked on.
    pub parked_at: Option<String>,
    /// Feed time of the newest report.
    newest: f64,
}

impl Live {
    /// The label shown on tags, the map and the ND ("BAW123", else the type / address).
    pub fn label(&self) -> String {
        if !self.callsign.is_empty() {
            self.callsign.clone()
        } else if !self.icao_type.is_empty() {
            format!("{} {:06X}", self.icao_type, self.hex & 0xff_ffff)
        } else {
            format!("{:06X}", self.hex & 0xff_ffff)
        }
    }
    pub fn type_label(&self) -> String {
        types::label(&self.icao_type, &self.stand)
    }
    /// Stable 16-bit id for the picture / audio (hashed address, high range).
    pub fn id(&self) -> u16 {
        0x8000 | ((self.hex ^ (self.hex >> 15)) & 0x7fff) as u16
    }
}

enum Source {
    Off,
    #[cfg(not(target_arch = "wasm32"))]
    Live {
        worker: Option<fetch::Worker>,
        poller: fetch::Poller,
    },
    /// A recorded session replayed on the simulation clock (deterministic).
    Replay {
        frames: Vec<Snapshot>,
        next: usize,
    },
}

/// The live traffic session.
pub struct LiveTraffic {
    pub settings: Settings,
    /// Session override of the enabled setting (command line, captures).
    pub force: Option<bool>,
    source: Source,
    pub status: String,
    pub aircraft: HashMap<u32, Live>,
    /// Feed clock offset: feed time = local Unix time + offset (live).
    offset: Option<f64>,
    /// Replay: feed time of the replay start and the sim time it maps to (0).
    replay_t0: Option<(f64, f64)>,
    /// Playout delay in use, s.
    pub delay: f64,
    ages: Vec<f64>,
    /// Playout time of the last update.
    last_t: Option<f64>,
    pub ground: ground::GroundNet,
    defs: Vec<AircraftDef>,
    infos: HashMap<String, TypeInfo>,
    visuals: HashMap<u32, (Remote, f64)>,
    /// This frame's camera (eye, view, vertical fov) for the tags.
    pub cam: Option<(Vec3, glam::Mat4, f32)>,
    /// Stop points (world) of stands a live aircraft is parked on: the
    /// procedural parked aircraft there are hidden.
    pub hidden: Vec<Vec2>,
    /// Timing of the last update / render (ms), for the performance budget.
    pub cpu_ms: f32,
    pub render_ms: f32,
    pub fetched: u64,
    /// Publish the picture for the ND / ATC (off in unit tests, which run in parallel).
    pub publish: bool,
}

impl Default for LiveTraffic {
    fn default() -> Self {
        LiveTraffic::new()
    }
}

/// The picture of this frame for code that only sees the sim (ND, ATC).
static PICTURE: std::sync::Mutex<Vec<Traffic>> = std::sync::Mutex::new(Vec::new());

/// Live aircraft this frame (empty when off).
pub fn picture() -> Vec<Traffic> {
    PICTURE.lock().map(|v| v.clone()).unwrap_or_default()
}

fn set_picture(v: Vec<Traffic>) {
    if let Ok(mut p) = PICTURE.lock() {
        *p = v;
    }
}

fn unix_now() -> f64 {
    web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

impl LiveTraffic {
    pub fn new() -> LiveTraffic {
        LiveTraffic {
            settings: Settings::default(),
            force: None,
            source: Source::Off,
            status: "live traffic off".into(),
            aircraft: HashMap::new(),
            offset: None,
            replay_t0: None,
            delay: 12.0,
            ages: Vec::new(),
            last_t: None,
            ground: ground::GroundNet::default(),
            defs: crate::flight::catalogue(),
            infos: HashMap::new(),
            visuals: HashMap::new(),
            cam: None,
            hidden: Vec::new(),
            cpu_ms: 0.0,
            render_ms: 0.0,
            fetched: 0,
            publish: true,
        }
    }

    /// Replay a recording (`--adsb-fixture`); always on, never fetches.
    pub fn replay(&mut self, frames: Vec<Snapshot>) {
        self.source = Source::Replay { frames, next: 0 };
        self.settings.enabled = true;
        self.delay = 10.0;
        self.status = "live traffic: recorded".into();
    }

    pub fn replaying(&self) -> bool {
        matches!(self.source, Source::Replay { .. })
    }

    pub fn active(&self) -> bool {
        self.settings.enabled && !matches!(self.source, Source::Off) && !self.aircraft.is_empty()
    }

    /// Turn on / off (settings changed). Clears everything when off.
    fn set_enabled(&mut self, on: bool) {
        if self.replaying() {
            return;
        }
        let running = !matches!(self.source, Source::Off);
        if on && !running {
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.source = Source::Live { worker: fetch::Worker::start(), poller: fetch::Poller::default() };
                self.status = "live traffic: connecting…".into();
            }
            #[cfg(target_arch = "wasm32")]
            {
                self.status = fetch::WEB_MESSAGE.into();
            }
        } else if !on && running {
            self.source = Source::Off;
            self.clear();
            self.status = "live traffic off".into();
        }
    }

    pub fn clear(&mut self) {
        self.aircraft.clear();
        self.visuals.clear();
        self.hidden.clear();
        self.last_t = None;
        if self.publish {
            set_picture(Vec::new());
        }
    }

    fn info(&mut self, model: &str, scale: f32) -> Option<TypeInfo> {
        let key = format!("{model}@{scale:.2}");
        if let Some(i) = self.infos.get(&key) {
            return Some(i.clone());
        }
        let def = self.defs.iter().position(|d| d.id == model)?;
        let d = &self.defs[def];
        let contacts = synth::contacts(d, scale);
        let rest_pitch = synth::rest_pitch(d);
        let gear_h = synth::rest_height(&contacts, synth::attitude_quat(0.0, rest_pitch, 0.0));
        let nose = d.gear.iter().filter(|g| matches!(g.kind, crate::flight::ContactKind::Wheel { .. })).map(|g| g.pos.z).fold(0.0f32, f32::max) * scale;
        let length = d.gear.iter().map(|g| g.pos.z).fold(0.0f32, |a, z| a.max(z.abs())) * 2.2 * scale + d.wingspan * 0.3 * scale;
        let i = TypeInfo { def, contacts, rest_pitch, gear_h, nose: nose.max(1.0), length: length.max(4.0) };
        self.infos.insert(key, i.clone());
        Some(i)
    }

    /// The current feed clock (Unix s): wall clock (live) or the replay
    /// clock driven by the simulation time.
    fn feed_now(&mut self, sim_time: f64) -> Option<f64> {
        match &self.source {
            Source::Off => None,
            #[cfg(not(target_arch = "wasm32"))]
            Source::Live { .. } => Some(unix_now() + self.offset.unwrap_or(0.0)),
            Source::Replay { frames, .. } => {
                let f0 = frames.first()?.now;
                // The recording starts with the flight (sim time 0), however late the first frame is drawn.
                let (t0, s0) = *self.replay_t0.get_or_insert((f0, 0.0));
                Some(t0 + (sim_time - s0).max(0.0))
            }
        }
    }

    fn ingest(&mut self, snap: &Snapshot, arrived: f64) {
        for r in &snap.reports {
            let a = self.aircraft.entry(r.hex).or_insert_with(|| {
                let stand = types::stand_in(&r.icao_type, &r.category, r.gs_kt.unwrap_or(0.0), r.alt_baro_ft.unwrap_or(0.0));
                Live {
                    hex: r.hex,
                    callsign: String::new(),
                    icao_type: r.icao_type.clone(),
                    category: r.category.clone(),
                    stand,
                    squawk: None,
                    emergency: None,
                    key: String::new(),
                    track: playout::Track::default(),
                    anim: synth::Anim::default(),
                    snap_off: Vec2::ZERO,
                    pos: Vec3::ZERO,
                    rot: Quat::IDENTITY,
                    vel: Vec3::ZERO,
                    on_ground: false,
                    state: AcState::default(),
                    shown: false,
                    dist: f32::MAX,
                    parked_at: None,
                    newest: r.t,
                }
            });
            if !r.callsign.is_empty() {
                a.callsign = r.callsign.clone();
            }
            if a.icao_type.is_empty() && !r.icao_type.is_empty() {
                a.icao_type = r.icao_type.clone();
                a.stand = types::stand_in(&r.icao_type, &r.category, 0.0, 0.0);
            }
            a.squawk = r.squawk;
            a.emergency = r.emergency.clone();
            if a.track.push(r) {
                a.newest = a.newest.max(r.t);
                self.ages.push((arrived - r.t).max(0.0));
            }
        }
        if self.ages.len() > 400 {
            let n = self.ages.len();
            self.ages.drain(..n - 400);
        }
    }

    /// Playout delay wanted: the poll interval plus most report ages plus a margin.
    fn target_delay(&self, interval: f64) -> f64 {
        if self.ages.is_empty() {
            return 12.0;
        }
        let mut a = self.ages.clone();
        a.sort_by(|x, y| x.total_cmp(y));
        let p80 = a[(a.len() * 4 / 5).min(a.len() - 1)];
        (interval + p80 + 2.0).clamp(7.0, 20.0)
    }

    /// Per frame: fetch / replay, play out, place, animate. `enabled` is
    /// the setting; `sim_time` the simulation clock (replays run on it).
    pub fn update(&mut self, sim: &crate::sim::Sim, world: &World, dt: f32, settings: &Settings) {
        let mut settings = settings.clone();
        if let Some(on) = self.force {
            settings.enabled = on;
        }
        let settings = &settings;
        let me = Me { pos: sim.ac.pos, vel: sim.ac.vel, sim_time: sim.time as f64 };
        let env = if settings.enabled || self.replaying() { Some(Env::of(world)) } else { None };
        match env {
            Some(env) => self.step(me, &env, dt, settings),
            None => self.step_off(settings),
        }
    }

    fn step_off(&mut self, settings: &Settings) {
        if !self.replaying() && (settings.enabled != self.settings.enabled || !matches!(self.source, Source::Off)) {
            self.set_enabled(false);
        }
        self.settings = settings.clone();
    }

    /// [`update`](Self::update) with the world abstracted (tests).
    pub fn step(&mut self, me: Me, env: &Env, dt: f32, settings: &Settings) {
        let t_cpu = web_time::Instant::now();
        let mut s = settings.clone();
        if self.replaying() {
            s.enabled = true;
        }
        if s.enabled != self.settings.enabled || matches!(self.source, Source::Off) == s.enabled {
            self.set_enabled(s.enabled);
        }
        self.settings = s;
        if !self.settings.enabled {
            return;
        }
        let Some(now) = self.feed_now(me.sim_time) else { return };
        // ---- data in
        let mut snaps: Vec<(Snapshot, f64)> = Vec::new();
        let mut interval = 10.0;
        match &mut self.source {
            Source::Off => {}
            #[cfg(not(target_arch = "wasm32"))]
            Source::Live { worker, poller } => {
                let wall = unix_now();
                interval = poller.interval;
                if let Some(w) = worker.as_ref() {
                    while let Some(r) = w.poll() {
                        self.fetched += 1;
                        match r {
                            Ok(snap) => {
                                // Feed clock ≈ local clock + offset (smoothed; the minimum
                                // of recent offsets excludes the network delay).
                                let off = snap.now - wall;
                                self.offset = Some(match self.offset {
                                    Some(o) if (o - off).abs() < 30.0 => o + (off - o) * 0.2,
                                    _ => off,
                                });
                                let near = snap.reports.len();
                                poller.done(wall, Ok(near));
                                self.status = format!("live traffic: {near} aircraft · adsb.lol");
                                snaps.push((snap, wall + self.offset.unwrap_or(0.0)));
                            }
                            Err(f) => {
                                poller.done(wall, Err(f.clone()));
                                let wait = (poller.next_at - wall).max(0.0);
                                self.status = match f {
                                    fetch::Failure::RateLimited(_) => format!("live traffic: rate limited, retrying in {wait:.0} s"),
                                    fetch::Failure::Offline(_) => format!("live traffic: offline, retrying in {wait:.0} s"),
                                    fetch::Failure::Bad(e) => format!("live traffic: {e}, retrying in {wait:.0} s"),
                                };
                            }
                        }
                    }
                    if poller.due(wall) {
                        let v = me.vel;
                        let trk = v.x.atan2(-v.z).to_degrees();
                        let gs = vec2(v.x, v.z).length();
                        let (lat, lon) = env.proj.to_latlon(vec2(me.pos.x, me.pos.z));
                        let (qlat, qlon) = fetch::query_centre(lat, lon, trk, gs, self.settings.radius_nm);
                        if w.request(feed::url(qlat, qlon, self.settings.radius_nm)) {
                            poller.sent();
                        }
                    }
                } else {
                    self.status = "live traffic: could not start the network thread".into();
                }
            }
            Source::Replay { frames, next } => {
                while *next < frames.len() && frames[*next].now <= now {
                    snaps.push((frames[*next].clone(), frames[*next].now));
                    *next += 1;
                }
                interval = 5.0;
            }
        }
        for (s, arrived) in &snaps {
            self.ingest(s, *arrived);
        }
        if !self.replaying() {
            let want = self.target_delay(interval);
            let k = 0.1 * dt.clamp(0.0, 0.1) as f64;
            self.delay += (want - self.delay).clamp(-k, k);
        }
        let t = now - self.delay;
        // Catch up after a jump (headless fast-forward, first frame, hitch):
        // settle the attitude filters over the last few seconds at 10 Hz.
        let steps: Vec<(f64, f32)> = match self.last_t {
            Some(lt) if t - lt > 0.5 && t - lt < 600.0 => {
                let from = (t - 4.0).max(lt);
                let n = ((t - from) / 0.1).ceil().max(1.0) as usize;
                (1..=n).map(|k| (from + (t - from) * k as f64 / n as f64, ((t - from) / n as f64) as f32)).collect()
            }
            None => (0..40).map(|k| (t - 4.0 + 0.1 * (k + 1) as f64, 0.1)).collect(),
            Some(lt) => vec![(t, (t - lt).max(0.0) as f32)],
        };
        self.last_t = Some(t);
        // Airport networks for ground snapping.
        if self.settings.ground {
            self.ground.sync(env.layouts.clone());
        }
        for (k, (ts, dts)) in steps.iter().enumerate() {
            self.place_all(env, *ts, *dts, k + 1 == steps.len());
        }
        // Drop old aircraft.
        self.aircraft.retain(|_, a| a.newest > t - EXPIRE);
        // Budget: the nearest N are shown.
        let mut order: Vec<(f32, u32)> = self.aircraft.values().filter(|a| a.shown).map(|a| (a.pos.distance(me.pos), a.hex)).collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (i, (_, hex)) in order.iter().enumerate() {
            if i >= self.settings.max_aircraft {
                if let Some(a) = self.aircraft.get_mut(hex) {
                    a.shown = false;
                }
            }
        }
        for a in self.aircraft.values_mut() {
            a.dist = a.pos.distance(me.pos);
        }
        self.hidden = self.aircraft.values().filter(|a| a.shown && a.parked_at.is_some()).map(|a| vec2(a.pos.x, a.pos.z)).collect();
        if self.publish {
            set_picture(self.traffic());
        }
        if self.replaying() {
            let n = self.aircraft.values().filter(|a| a.shown).count();
            self.status = format!("live traffic: {n} aircraft (recorded)");
        }
        self.cpu_ms = t_cpu.elapsed().as_secs_f32() * 1000.0;
    }

    /// Play out every aircraft at `t` and place it in the world.
    fn place_all(&mut self, env: &Env, t: f64, dt: f32, last: bool) {
        let proj = env.proj;
        let world = env.env;
        let qnh_corr = ((env.qnh_hpa as f64) - 1013.25) * 8.23;
        let ground_y = |lat: f64, lon: f64| -> f64 {
            let (x, z) = proj.to_world64(lat, lon);
            world.ground_height(vec2(x as f32, z as f32)) as f64
        };
        let hexes: Vec<u32> = self.aircraft.keys().copied().collect();
        let show_ground = self.settings.ground;
        for hex in hexes {
            let (model, scale) = {
                let a = &self.aircraft[&hex];
                (a.stand.model, a.stand.scale)
            };
            let Some(info) = self.info(model, scale) else { continue };
            let def = &self.defs[info.def];
            let a = self.aircraft.get_mut(&hex).unwrap();
            a.key = format!("{model}@{scale:.2}");
            let hs = playout::Heights { ground: &ground_y, qnh_corr, gear: info.gear_h as f64 };
            let Some(p) = a.track.sample(t, dt as f64, &hs) else {
                a.shown = false;
                continue;
            };
            if last {
                a.track.trim(t);
            }
            let (x, z) = proj.to_world64(p.lat, p.lon);
            let mut xz = vec2(x as f32, z as f32);
            let mut vel = vec3(p.vel[0] as f32, p.vel[1] as f32, p.vel[2] as f32);
            let gy = world.ground_height(xz);
            let agl = p.y as f32 - info.gear_h - gy;
            let on_ground = p.on_ground || agl < 0.5;
            let gs = vec2(vel.x, vel.z).length();
            // ---- ground: stands, runways, taxiways
            let mut ground_heading = None;
            a.parked_at = None;
            if on_ground && show_ground {
                let track = if gs > 1.0 { vel.x.atan2(-vel.z).to_degrees().rem_euclid(360.0) } else { a.anim.heading };
                let target = if gs < 0.8 {
                    match self.ground.stand(&proj, xz) {
                        Some((stop, h, name)) => {
                            a.parked_at = Some(name);
                            ground_heading = Some(h);
                            Some(ground::cg_from_stop(stop, h, info.nose) - xz)
                        }
                        None => None,
                    }
                } else if let Some(s) = ground::GroundNet::runway(env.runways, xz, track).filter(|_| gs > 15.0) {
                    ground_heading = Some(s.heading);
                    Some(s.pos - xz)
                } else if let Some(s) = self.ground.taxiway(&proj, xz, track) {
                    if gs > 2.0 {
                        ground_heading = Some(s.heading);
                    }
                    Some(s.pos - xz)
                } else {
                    None
                };
                a.snap_off = ground::smooth_offset(a.snap_off, target.unwrap_or(Vec2::ZERO), dt.max(1e-3));
                if a.anim.init && a.parked_at.is_some() && dt > 0.0 && a.snap_off.distance(target.unwrap_or(a.snap_off)) > 0.5 {
                    // Parked: settle straight onto the stand.
                    a.snap_off = target.unwrap_or(a.snap_off);
                }
                xz += a.snap_off;
                if gs < 0.5 {
                    vel = Vec3::ZERO;
                }
            } else {
                a.snap_off = ground::smooth_offset(a.snap_off, Vec2::ZERO, dt.max(1e-3));
                xz += a.snap_off;
            }
            // ---- attitude and configuration
            let pos0 = vec3(xz.x, p.y as f32, xz.y);
            let wind = if on_ground { Vec3::ZERO } else { world.wind(pos0) };
            let liftoff_in = if on_ground && gs > 30.0 {
                // When does the curve leave the ground? (the playout knows the future)
                (1..=8).map(|k| k as f64 * 0.5).find(|dtf| a.track.raw(t + dtf, &hs).is_some_and(|q| q.y - (ground_y(q.lat, q.lon) + info.gear_h as f64) > 0.6)).map(|s| s as f32)
            } else {
                None
            };
            let rot = a.anim.step(&synth::Frame {
                def,
                contacts: &info.contacts,
                rest_pitch: info.rest_pitch,
                dt,
                vel,
                wind,
                on_ground,
                agl: agl.max(0.0),
                alt_m: p.y as f32,
                heading: p.heading,
                roll: p.roll,
                liftoff_in,
                ground_heading,
                at_stand: a.parked_at.is_some(),
            });
            // Wheels on the ground (rotation pivots on the mains).
            let rest = synth::rest_height(&info.contacts, rot);
            let y = if on_ground { gy + rest } else { (p.y as f32).max(gy + rest) };
            a.pos = vec3(xz.x, y, xz.y);
            a.rot = rot;
            a.vel = vel;
            a.on_ground = on_ground;
            a.state = synth::state(&a.anim, def, rot, vel, wind, y, on_ground, t);
            let (lat, lon) = proj.to_latlon(xz);
            a.state.lat = lat;
            a.state.lon = lon;
            a.shown = !(on_ground && !show_ground);
        }
    }

    /// The traffic picture (current world frame) for the map, ND and ATC.
    pub fn traffic(&self) -> Vec<Traffic> {
        self.aircraft
            .values()
            .filter(|a| a.shown)
            .map(|a| Traffic {
                id: a.id(),
                name: a.label(),
                aircraft: a.stand.model.to_string(),
                pos: a.pos,
                vel: a.vel,
                heading: a.anim.heading.to_radians(),
                on_ground: a.on_ground,
                kind: a.type_label(),
                live: true,
            })
            .collect()
    }

    /// Wake-turbulence generators (hook for the weather track's wake model).
    pub fn wake_sources(&self) -> Vec<WakeSource> {
        self.aircraft.values().filter(|a| a.shown).map(|a| WakeSource { pos: a.pos, vel: a.vel, span: a.stand.span, wake: a.stand.wake, on_ground: a.on_ground }).collect()
    }

    /// Draw lists for the renderer (eye-relative): models for the nearest
    /// [`MODEL_BUDGET`] within [`MODEL_DIST`], light sprites for the rest
    /// out to [`LIGHTS_DIST`]. `slots_left`: renderer slots after the
    /// multiplayer players.
    pub fn render(
        &mut self,
        eye: Vec3,
        time: f32,
        slots_left: usize,
        lights: &mut Vec<crate::lights::LightInst>,
        flames: &mut crate::scene::MeshData,
    ) -> Vec<crate::gfx_aircraft::AircraftFrame> {
        let t_cpu = web_time::Instant::now();
        let mut frames = Vec::new();
        if !self.settings.enabled || self.aircraft.is_empty() {
            self.visuals.clear();
            return frames;
        }
        let mut near: Vec<(f32, u32)> = self.aircraft.values().filter(|a| a.shown).map(|a| (a.pos.distance(eye), a.hex)).filter(|(d, _)| *d < LIGHTS_DIST).collect();
        near.sort_by(|a, b| a.0.total_cmp(&b.0));
        let budget = MODEL_BUDGET.min(slots_left);
        let now = self.last_t.unwrap_or(0.0);
        for (i, (d, hex)) in near.iter().enumerate() {
            let a = &self.aircraft[hex];
            if i < budget && *d < MODEL_DIST {
                let v = self.visuals.entry(*hex).or_insert_with(|| {
                    let mut r = Remote::new_live(a.id(), a.label(), a.stand.model.to_string(), &self.defs, a.stand.scale);
                    r.poll_model(&self.defs);
                    (r, now)
                });
                if v.0.aircraft != a.stand.model || (v.0.scale - a.stand.scale).abs() > 1e-3 {
                    v.0 = Remote::new_live(a.id(), a.label(), a.stand.model.to_string(), &self.defs, a.stand.scale);
                }
                v.1 = now;
                v.0.poll_model(&self.defs);
                v.0.place(a.state, a.pos, a.rot, a.vel);
                let before = lights.len();
                match v.0.frame(&self.defs, eye, time, lights, flames) {
                    Some(f) => frames.push(f),
                    None => {
                        // Model still loading: lights only.
                        lights.truncate(before);
                        generic_lights(a, self.infos.get(&a.key), eye, time, lights);
                    }
                }
            } else {
                generic_lights(a, self.infos.get(&a.key), eye, time, lights);
            }
        }
        // Models not drawn for a while are released.
        self.visuals.retain(|_, (_, t)| now - *t < 10.0);
        self.render_ms = t_cpu.elapsed().as_secs_f32() * 1000.0;
        frames
    }

    /// The nearest live aircraft's engines into the free audio slots.
    pub fn sounds(&self, ear: Vec3, out: &mut [crate::audio::params::TrafficFrame; crate::audio::params::MAX_TRAFFIC]) {
        let mut near: Vec<&Live> = self.aircraft.values().filter(|a| a.shown && a.pos.distance(ear) < 6000.0 && a.anim.phase != Some(synth::Phase::Parked)).collect();
        near.sort_by(|a, b| a.pos.distance_squared(ear).total_cmp(&b.pos.distance_squared(ear)));
        let mut it = near.into_iter();
        for slot in out.iter_mut().filter(|s| s.id == 0) {
            let Some(a) = it.next() else { break };
            let Some(info) = self.infos.get(&a.key) else { continue };
            let def = &self.defs[info.def];
            let st = &a.state;
            let mut profile = crate::audio::capture::profile_for(def, a.stand.span * 0.5);
            profile.engines = 1;
            let e = &st.engines[0];
            let eng = crate::audio::params::EngineFrame { pos: a.pos, rpm: e.rpm, spool: e.spool, thrust: e.thrust, ab: 0.0, running: e.running, throttle: st.throttle };
            let rotor = crate::audio::params::RotorFrame {
                nr_rpm: st.rotor.nr_rpm,
                nr: st.rotor.nr_pct / 100.0,
                ng: e.spool,
                torque: e.spool,
                power: e.spool,
                tail_rpm: st.rotor.tail_rpm,
                ..Default::default()
            };
            *slot = crate::audio::params::TrafficFrame {
                id: 0x10000 + a.id() as u32,
                profile,
                n_engines: st.n_engines.max(1),
                eng,
                pos: a.pos,
                vel: a.vel,
                axis: a.rot * Vec3::Z,
                tas: st.tas,
                reverser: st.reverser,
                rotor,
            };
        }
    }

    /// One line per aircraft (diagnostics: `SKYWARD_ADSB_DEBUG=1` with `--shot`).
    pub fn debug_lines(&self, proj: &crate::osm::Proj) -> Vec<String> {
        let mut v: Vec<&Live> = self.aircraft.values().collect();
        v.sort_by(|a, b| a.dist.total_cmp(&b.dist));
        v.iter()
            .map(|a| {
                let (lat, lon) = proj.to_latlon(vec2(a.pos.x, a.pos.z));
                format!(
                    "{:<8} {:<4} {:<11} {} {:.5},{:.5} y {:.0} m hdg {:.0} pitch {:.1} bank {:.1} gear {:.1} flaps {:.2} {} snap {:.1} m{}",
                    a.label(),
                    a.icao_type,
                    a.stand.model,
                    a.anim.phase.map(|p| p.name()).unwrap_or("-"),
                    lat,
                    lon,
                    a.pos.y,
                    a.anim.heading,
                    a.anim.pitch.to_degrees(),
                    a.anim.bank.to_degrees(),
                    a.state.gear,
                    a.state.flaps,
                    if a.shown { "shown" } else { "hidden" },
                    a.snap_off.length(),
                    a.parked_at.as_ref().map(|s| format!(" stand {s}")).unwrap_or_default()
                )
            })
            .collect()
    }

    /// Number of aircraft shown.
    pub fn shown(&self) -> usize {
        self.aircraft.values().filter(|a| a.shown).count()
    }
}

/// Light sprites for an aircraft drawn without its model (far away / loading):
/// navigation lights at the wing tips and tail, beacon, strobes, landing light.
fn generic_lights(a: &Live, info: Option<&TypeInfo>, eye: Vec3, time: f32, out: &mut Vec<crate::lights::LightInst>) {
    use skyward_net::proto::lights as lb;
    let half = a.stand.span * 0.5;
    let len = info.map(|i| i.length).unwrap_or(a.stand.span);
    let mut rig = crate::lights::AircraftLightRig::default();
    let bits = a.state.lights;
    if bits & lb::NAV != 0 {
        rig.nav.push((vec3(half, 0.0, -len * 0.05), [1.0, 0.04, 0.02]));
        rig.nav.push((vec3(-half, 0.0, -len * 0.05), [0.05, 1.0, 0.3]));
        rig.nav.push((vec3(0.0, len * 0.08, -len * 0.5), [1.0, 1.0, 1.0]));
    }
    if bits & lb::BEACON != 0 {
        rig.beacons.push(vec3(0.0, len * 0.07, 0.0));
    }
    if bits & lb::STROBE != 0 {
        rig.strobes.push(vec3(half, 0.0, -len * 0.05));
        rig.strobes.push(vec3(-half, 0.0, -len * 0.05));
    }
    if bits & lb::LANDING != 0 {
        rig.landing.push((vec3(0.0, -len * 0.03, len * 0.35), Vec3::Z));
    }
    let l = crate::lights::AircraftLights::build(&rig, a.pos - eye, a.rot, time, bits & lb::LANDING != 0, bits & lb::TAXI != 0);
    out.extend_from_slice(&l.sprites[..l.n_sprites]);
}

#[cfg(test)]
mod tests;
