//! Ships (Track SH): live where AIS is open, plausible everywhere else.
//!
//! * [`ais`] — live positions and static data from Fintraffic Digitraffic
//!   (Baltic north of ~57.5° N; CC BY 4.0, CORS, no key), dead-reckoned
//!   and smoothed between the once-a-minute polls.
//! * [`density`] — per-class vessel density (EMODnet Human Activities,
//!   CC BY 4.0) fetched per region on demand and cached under
//!   `cache/ships/`; a thin generic mix where there is no data.
//! * [`sim`] — procedural ships spawned at that density around the eye,
//!   class by the per-layer density, on water deep enough for their
//!   draught, steered along the lanes the density draws and around land;
//!   anchored at the hot spots (ports, anchorages).
//! * [`types`] — AIS ship type × length → class → hull model.
//! * [`models`] — the FlightGear AI ship models (SAVAGE, BSD-3; fgdata,
//!   GPL-2.0+) and a procedural workboat hull, drawn by gfx_aircraft.rs.
//! * [`render`] — the hull on the sea state (heave, pitch, roll), the wake
//!   (foam on the water shader) and the COLREGs lights at night.
//!
//! Switches: `SKYWARD_NO_SHIPS=1` turns ships off (golden scenes set it),
//! `SKYWARD_NO_AIS=1` only the live data, `SKYWARD_NO_SHIP_DENSITY=1` the
//! EMODnet fetches (generic density). `SKYWARD_SHIPS_DEBUG=1` prints the
//! census and the ships.

pub mod ais;
pub mod density;
pub mod models;
pub mod render;
pub mod sim;
pub mod types;

#[cfg(test)]
mod tests;

use crate::gfx_aircraft::AircraftFrame;
use crate::lights::LightInst;
use crate::osm::Proj;
use crate::scene::MeshData;
use crate::water::SeaState;
use crate::world::World;
use density::{N, Sample};
use glam::{Mat4, Vec2, Vec3, vec2};
use sim::{Census, Rng, Sea, Ship};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};

/// Off with `SKYWARD_NO_SHIPS`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| std::env::var_os("SKYWARD_NO_SHIPS").is_none());
    *ON
}

/// Live AIS off with `SKYWARD_NO_AIS` (procedural ships stay).
pub fn live_enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| std::env::var_os("SKYWARD_NO_AIS").is_none());
    *ON
}

static INLINE: AtomicBool = AtomicBool::new(false);

/// Headless captures load ship models inline (the shot must show them).
pub fn set_inline_loads(on: bool) {
    INLINE.store(on, Ordering::Relaxed);
}

pub fn inline_loads() -> bool {
    INLINE.load(Ordering::Relaxed)
}

/// Ships are simulated within this radius of the eye (m).
pub const SIM_RADIUS: f32 = if cfg!(target_arch = "wasm32") { 14_000.0 } else { 22_000.0 };
/// Most procedural ships at once.
pub const MAX_SHIPS: usize = if cfg!(target_arch = "wasm32") { 40 } else { 90 };
/// Hulls drawn (nearest first) and how far.
pub const MODEL_BUDGET: usize = if cfg!(target_arch = "wasm32") { 20 } else { 48 };
pub const MODEL_DIST: f32 = 18_000.0;
/// Navigation lights drawn this far (m).
pub const LIGHTS_DIST: f32 = 22_000.0;

/// The world as the ships see it.
pub struct WorldSea<'a> {
    pub world: &'a World,
    pub month: u32,
}

impl Sea for WorldSea<'_> {
    fn water(&self, xz: Vec2) -> Option<bool> {
        self.world.water_known(xz)
    }
    fn depth(&self, xz: Vec2) -> Option<f32> {
        let (lat, lon) = self.world.proj.to_latlon(xz);
        crate::bathy::depth_at(lat, lon)
    }
    fn density(&self, xz: Vec2) -> Sample {
        let (lat, lon) = self.world.proj.to_latlon(xz);
        density::sample(lat, lon)
    }
    fn month(&self) -> u32 {
        self.month
    }
}

/// Counters for the HUD / perf report.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub procedural: usize,
    pub live: usize,
    pub drawn: usize,
    pub lights: usize,
    pub wake_verts: usize,
    /// CPU time of the last update / render (ms).
    pub update_ms: f32,
    pub render_ms: f32,
}

/// The ships session.
pub struct Ships {
    pub on: bool,
    pub live_on: bool,
    pub ships: Vec<Ship>,
    proj: Option<Proj>,
    /// Session time (s).
    t: f64,
    rng: Rng,
    census: Option<Census>,
    census_t: f64,
    next_id: u64,
    filled: bool,
    /// Hand placements done (SKYWARD_SHIPS_PLACE, SKYWARD_SHIP_GALLERY).
    placed: bool,
    // Live AIS.
    tracks: HashMap<u32, ais::Track>,
    statics: HashMap<u32, ais::Static>,
    poller: ais::Poller,
    worker: Option<ais::Worker>,
    unknown: usize,
    /// A position poll has been answered (shots wait for the first).
    fixes_answered: bool,
    pub status: String,
    /// The sea state where the ships are (refreshed every few seconds).
    sea: SeaState,
    sea_t: f64,
    trail_t: f64,
    pub stats: Stats,
    frames: u64,
}

impl Default for Ships {
    fn default() -> Self {
        Self::new()
    }
}

/// Unix time now (s).
fn unix_now() -> f64 {
    web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

impl Ships {
    pub fn new() -> Ships {
        Ships {
            on: enabled(),
            live_on: live_enabled(),
            ships: Vec::new(),
            proj: None,
            t: 0.0,
            rng: Rng(0x5eed_5b1f),
            census: None,
            census_t: -1e9,
            next_id: 1,
            filled: false,
            placed: false,
            tracks: HashMap::new(),
            statics: HashMap::new(),
            poller: ais::Poller::default(),
            worker: None,
            unknown: 0,
            fixes_answered: false,
            status: String::new(),
            sea: SeaState::default(),
            sea_t: -1e9,
            trail_t: 0.0,
            stats: Stats::default(),
            frames: 0,
        }
    }

    /// Nothing still loading for the view (headless shots wait for this).
    pub fn idle(&self) -> bool {
        if !self.on {
            return true;
        }
        let live_wait = self.worker.is_some() && !self.fixes_answered && self.poller.failures == 0;
        density::idle() && models::idle() && !live_wait
    }

    /// Is the live feed answering here?
    pub fn live_here(&self) -> bool {
        self.poller.last_ok.is_some_and(|t| unix_now() - t < 600.0) && self.worker.is_some()
    }

    /// Start the downloads for a view at `eye` (density, live data) without
    /// simulating: headless captures call it while the scenery streams.
    pub fn prefetch(&mut self, world: &World, eye: Vec3) {
        if !self.on {
            return;
        }
        let e2 = vec2(eye.x, eye.z);
        let (lat, lon) = world.proj.to_latlon(e2);
        density::set_focus(lat, lon, SIM_RADIUS as f64 + 5000.0);
        self.update_live(world, e2, lat, lon);
    }

    /// Advance by `dt` (s): rebase, live data, procedural spawns, steering.
    pub fn update(&mut self, world: &World, eye: Vec3, dt: f32, month: u32) {
        if !self.on {
            if !self.ships.is_empty() {
                self.ships.clear();
            }
            return;
        }
        let t0 = web_time::Instant::now();
        // Floating origin moved: re-express positions in the new frame.
        if let Some(old) = self.proj.filter(|p| *p != world.proj) {
            let conv = |p: Vec2| {
                let (la, lo) = old.to_latlon(p);
                world.proj.to_world(la, lo)
            };
            for s in &mut self.ships {
                s.pos = conv(s.pos);
                for q in &mut s.trail {
                    q.0 = conv(q.0);
                }
            }
            if let Some(c) = self.census.as_mut() {
                c.centre = conv(c.centre);
                for cell in &mut c.cells {
                    cell.0 = conv(cell.0);
                }
            }
        }
        self.proj = Some(world.proj);
        let dt = dt.clamp(0.0, 5.0);
        self.t += dt as f64;
        let e2 = vec2(eye.x, eye.z);
        let (lat, lon) = world.proj.to_latlon(e2);
        density::set_focus(lat, lon, SIM_RADIUS as f64 + 5000.0);
        let sea = WorldSea { world, month };
        self.update_live(world, e2, lat, lon);
        self.update_procedural(&sea, e2, dt);
        // Water levels (lakes above the sea), wake trails, sea state.
        for s in &mut self.ships {
            s.level = if world.water_known(s.pos) == Some(true) { crate::flight::Environment::ground_height(world, s.pos) } else { 0.0 };
        }
        if self.t >= self.trail_t {
            self.trail_t = self.t + 1.5;
            let t = self.t;
            for s in self.ships.iter_mut().filter(|s| s.speed > 0.5 && s.pos.distance(e2) < render::WAKE_DIST * 1.5) {
                s.trail.push((s.pos, t));
                if s.trail.len() > 64 {
                    s.trail.remove(0);
                }
            }
        }
        if self.t - self.sea_t > 5.0 {
            self.sea_t = self.t;
            let at = self.ships.iter().map(|s| s.pos).min_by(|a, b| a.distance(e2).total_cmp(&b.distance(e2)));
            if let Some(p) = at {
                self.sea = crate::water::sea_at(world, p);
            }
        }
        self.stats.procedural = self.ships.iter().filter(|s| s.mmsi.is_none()).count();
        self.stats.live = self.ships.len() - self.stats.procedural;
        self.stats.update_ms = t0.elapsed().as_secs_f32() * 1000.0;
    }

    fn update_live(&mut self, world: &World, e2: Vec2, lat: f64, lon: f64) {
        let now = unix_now();
        if !self.live_on || crate::platform::offline() || !ais::covers(lat, lon) {
            self.ships.retain(|s| s.mmsi.is_none());
            self.tracks.clear();
            return;
        }
        if self.worker.is_none() {
            self.worker = ais::Worker::start();
            self.status = "Live ships: connecting".into();
        }
        let Some(w) = self.worker.as_ref() else { return };
        while let Some((what, ans)) = w.poll() {
            match ans {
                ais::Answer::Fixes(f) => {
                    self.poller.done(now, what, true);
                    self.fixes_answered = true;
                    let n = f.len();
                    self.unknown = ais::merge(&mut self.tracks, f, &self.statics, now);
                    self.status = format!("Live ships: {n} reports (Digitraffic)");
                }
                ais::Answer::Vessels(v) => {
                    self.poller.done(now, what, true);
                    for s in v {
                        self.statics.insert(s.mmsi, s);
                    }
                    self.unknown = self.tracks.keys().filter(|m| !self.statics.contains_key(m)).count();
                }
                ais::Answer::Failed(e) => {
                    self.poller.done(now, what, false);
                    self.status = format!("Live ships: {e}");
                    eprintln!("ships: AIS: {e}");
                }
            }
        }
        if let Some(what) = self.poller.due(now, self.unknown) {
            let url = match what {
                ais::Due::Fixes => ais::locations_url(lat, lon),
                ais::Due::Vessels => ais::vessels_url(),
            };
            if w.request(what, url) {
                self.poller.sent();
            }
        }
        // Tracks → ships (dead-reckoned, corrections blended out).
        let proj = world.proj;
        let mut seen = Vec::with_capacity(self.tracks.len());
        for (mmsi, tr) in self.tracks.iter_mut() {
            let (la, lo) = ais::dead_reckon(&tr.fix, now);
            tr.corr = (tr.corr.0 * 0.97, tr.corr.1 * 0.97);
            let (la, lo) = (la + tr.corr.0, lo + tr.corr.1);
            let pos = proj.to_world(la, lo);
            if pos.distance(e2) > SIM_RADIUS * 1.5 {
                continue;
            }
            seen.push(*mmsi);
            let hdg = tr.fix.best_heading().map(|h| h.to_radians());
            tr.shown = Some((la, lo, hdg.unwrap_or(0.0).to_degrees()));
            let id = (1u64 << 40) | *mmsi as u64;
            let st = self.statics.get(mmsi);
            let class = st.map(|s| types::class_of(s.ship_type, s.a as f32 + s.b as f32, &s.name)).unwrap_or(types::Class::Service);
            match self.ships.iter_mut().find(|s| s.id == id) {
                Some(s) => {
                    s.pos = pos;
                    if let Some(h) = hdg {
                        s.heading = (s.heading + sim::ang_diff(s.heading, h) * 0.1).rem_euclid(std::f32::consts::TAU);
                    }
                    s.speed = if tr.fix.stopped() { 0.0 } else { tr.fix.sog * 0.514_444 };
                    s.anchored = matches!(tr.fix.nav_stat, 1 | 5);
                    if s.class != class {
                        let (len, beam) = st.map(|x| types::dims(class, x.a, x.b, x.c, x.d, s.length)).unwrap_or((s.length, s.beam));
                        s.class = class;
                        s.model = types::model_for(class, len);
                        (s.length, s.beam) = (len, beam);
                    }
                }
                None => {
                    let (lo_l, hi_l) = class.length_range();
                    let (len, beam) = match st {
                        Some(x) => types::dims(class, x.a, x.b, x.c, x.d, (lo_l + hi_l) * 0.5),
                        None => (30.0, 8.0),
                    };
                    let draught = st.map(|x| x.draught).filter(|d| *d > 0.3).unwrap_or(class.draught(len));
                    let mut s = Ship::new(id, class, len, beam, draught, pos, hdg.unwrap_or(0.0));
                    s.mmsi = Some(*mmsi);
                    s.name = st.map(|x| x.name.clone()).filter(|n| !n.is_empty()).unwrap_or_else(|| format!("MMSI {mmsi}"));
                    s.speed = if tr.fix.stopped() { 0.0 } else { tr.fix.sog * 0.514_444 };
                    s.anchored = matches!(tr.fix.nav_stat, 1 | 5);
                    self.ships.push(s);
                }
            }
        }
        self.ships.retain(|s| s.mmsi.is_none_or(|m| seen.contains(&m)));
    }

    /// Ships placed by hand (captures, model checks): `SKYWARD_SHIPS_PLACE` =
    /// `lat,lon,class,length,heading,knots` entries separated by `;`, and
    /// `SKYWARD_SHIP_GALLERY=1`: every hull in a row 600 m north of the eye.
    fn place_fixed(&mut self, world: &World, e2: Vec2) {
        if let Ok(spec) = std::env::var("SKYWARD_SHIPS_PLACE") {
            for e in spec.split(';').filter(|e| !e.trim().is_empty()) {
                let f: Vec<&str> = e.split(',').map(|x| x.trim()).collect();
                let num = |i: usize| f.get(i).and_then(|x| x.parse::<f64>().ok());
                let (Some(lat), Some(lon), Some(class), Some(len)) = (num(0), num(1), f.get(2).and_then(|c| types::Class::parse(c)), num(3)) else {
                    eprintln!("ships: bad SKYWARD_SHIPS_PLACE entry {e:?}");
                    continue;
                };
                let len = len as f32;
                let mut s =
                    Ship::new(self.next_id, class, len, len * class.beam_ratio(), class.draught(len), world.proj.to_world(lat, lon), (num(4).unwrap_or(0.0) as f32).to_radians());
                s.cruise = num(5).unwrap_or(0.0) as f32 * 0.514_444;
                s.speed = s.cruise;
                s.anchored = s.cruise <= 0.0;
                s.name = format!("{} (placed)", class.name());
                self.next_id += 1;
                self.ships.push(s);
            }
        }
        if std::env::var_os("SKYWARD_SHIP_GALLERY").is_some() {
            let all = [
                (types::Class::Container, 300.0),
                (types::Class::Cargo, 160.0),
                (types::Class::Tanker, 220.0),
                (types::Class::Cruise, 290.0),
                (types::Class::Passenger, 180.0),
                (types::Class::Passenger, 60.0),
                (types::Class::Fishing, 25.0),
                (types::Class::Fishing, 45.0),
                (types::Class::Sailing, 14.0),
                (types::Class::Pleasure, 15.0),
                (types::Class::Barge, 80.0),
                (types::Class::Tug, 30.0),
                (types::Class::Military, 110.0),
            ];
            // Bow to stern heading east, broadside to a camera looking north.
            let total: f32 = all.iter().map(|a| a.1 + 60.0).sum();
            // (The variable's value, if a number, shifts the row east, m.)
            let shift = std::env::var("SKYWARD_SHIP_GALLERY").ok().and_then(|v| v.parse::<f32>().ok()).unwrap_or(0.0);
            let mut x = -0.5 * total + shift;
            for (class, len) in all {
                let p = e2 + vec2(x + len * 0.5, -500.0);
                x += len + 60.0;
                let mut s = Ship::new(self.next_id, class, len, len * class.beam_ratio(), class.draught(len), p, std::f32::consts::FRAC_PI_2);
                s.name = format!("{} {len:.0} m", class.name());
                self.next_id += 1;
                self.ships.push(s);
            }
        }
    }

    fn update_procedural(&mut self, sea: &WorldSea, e2: Vec2, dt: f32) {
        if !self.placed {
            self.placed = true;
            self.place_fixed(sea.world, e2);
        }
        // Census of the density around the eye (as the scenery and the data arrive).
        let moved = self.census.as_ref().is_none_or(|c| c.centre.distance(e2) > 2000.0);
        if moved || self.t - self.census_t > if self.filled { 20.0 } else { 1.0 } {
            self.census = Some(sim::census(sea, e2, SIM_RADIUS, 1000.0));
            self.census_t = self.t;
        }
        let Some(cen) = self.census.as_ref() else { return };
        // Where the live feed answers, real ships replace the commercial traffic
        // (pleasure craft without AIS stay, thinner).
        let live = self.live_here();
        let mut want = cen.expected;
        for (l, w) in want.iter_mut().enumerate() {
            if live {
                *w *= match density::LAYERS[l].0 {
                    density::Layer::Pleasure | density::Layer::Sailing => 0.5,
                    _ => 0.0,
                };
            }
        }
        // Within the budget the commercial traffic comes first (at most 60 %
        // of it); leisure craft fill the rest (most of their AIS hours are
        // spent moored in marinas: a third of them are out on the water).
        let leisure = |l: usize| matches!(density::LAYERS[l].0, density::Layer::Pleasure | density::Layer::Sailing);
        let cap = MAX_SHIPS as f32;
        let commercial: f32 = (0..N).filter(|&l| !leisure(l)).map(|l| want[l]).sum();
        let kc = if commercial > cap * 0.6 { cap * 0.6 / commercial } else { 1.0 };
        let fun: f32 = (0..N).filter(|&l| leisure(l)).map(|l| want[l] * 0.35).sum();
        let room = cap - commercial * kc;
        // (The density is an annual mean: yachts are out in summer, laid up in
        // winter, the more so the further from the equator.)
        let (lat, _) = sea.world.proj.to_latlon(e2);
        let season = sea.month.clamp(1, 12) as f32;
        let summer = (((season - 1.0) / 12.0 * std::f32::consts::TAU).cos() * -0.5 + 0.5) * if lat >= 0.0 { 1.0 } else { -1.0 } + if lat >= 0.0 { 0.0 } else { 1.0 };
        let season_k = 1.0 - ((lat.abs() as f32 - 30.0) / 30.0).clamp(0.0, 0.85) * (1.0 - summer);
        let kl = if fun > room { room.max(0.0) / fun } else { 1.0 } * 0.35 * season_k;
        for (l, w) in want.iter_mut().enumerate() {
            *w *= if leisure(l) { kl } else { kc };
        }
        let mut have = [0usize; N];
        for s in self.ships.iter().filter(|s| s.mmsi.is_none()) {
            if let Some(l) = s.layer {
                have[l.index()] += 1;
            }
        }
        // First fill: everything at once, anywhere; later: a few per update,
        // far out (no pop-in near the player).
        let first = !self.filled && !cen.cells.is_empty();
        let attempts = if first { 600 } else { 4 };
        let min_dist = if first { 0.0 } else { SIM_RADIUS * 0.6 };
        let mut spawned = 0;
        for _ in 0..attempts {
            // The layer furthest below its share.
            let Some(l) = (0..N).filter(|&l| (have[l] as f32) < want[l] - 0.5).max_by(|&a, &b| (want[a] - have[a] as f32).total_cmp(&(want[b] - have[b] as f32))) else { break };
            let Some(p) = sim::pick_cell(cen, l, e2, min_dist, &mut self.rng) else {
                want[l] = 0.0;
                continue;
            };
            let id = self.next_id;
            if let Some(s) = sim::spawn(sea, density::LAYERS[l].0, p, &self.ships, id, &mut self.rng) {
                self.next_id += 1;
                self.ships.push(s);
                have[l] += 1;
                spawned += 1;
            }
        }
        if first {
            self.filled = true;
            if std::env::var_os("SKYWARD_SHIPS_DEBUG").is_some() {
                let names: Vec<String> = (0..N).map(|l| format!("{} {:.1}/{}", sim::layer_name(l), cen.expected[l], have[l])).collect();
                eprintln!("ships: census {} cells, expected/spawned: {} ({} spawned)", cen.cells.len(), names.join(", "), spawned);
                for d in [vec2(0.0, 0.0), vec2(0.0, 2000.0), vec2(2000.0, 0.0), vec2(0.0, -2000.0), vec2(-2000.0, 0.0)] {
                    let p = e2 + d;
                    eprintln!("  eye{d:?}: water {:?} depth {:?} density {:?}", sea.water(p), sea.depth(p), sim::ships_per_km2(sea, p).map(|v| v.iter().sum::<f32>()));
                }
                // Why layers came up short: what the world says at a few of their spawn points.
                for l in (0..N).filter(|&l| (have[l] as f32) < cen.expected[l] * 0.5) {
                    for _ in 0..4 {
                        if let Some(p) = sim::pick_cell(cen, l, e2, 0.0, &mut self.rng) {
                            let (la, lo) = sea.world.proj.to_latlon(p);
                            eprintln!(
                                "  {} at {la:.4},{lo:.4}: water {:?} depth {:?} density {:.3}",
                                sim::layer_name(l),
                                sea.water(p),
                                sea.depth(p),
                                sim::ships_per_km2(sea, p).map(|v| v[l]).unwrap_or(-1.0)
                            );
                        }
                    }
                }
            }
        }
        // Move (sub-stepped: steering checks every few seconds), despawn far / stranded ones.
        let t = self.t;
        let rng = &mut self.rng;
        let steps = (dt / 0.5).ceil().max(1.0) as usize;
        for s in self.ships.iter_mut().filter(|s| s.mmsi.is_none()) {
            for k in 0..steps {
                sim::step(sea, s, t - (dt * (steps - 1 - k) as f32 / steps as f32) as f64, dt / steps as f32, rng);
            }
        }
        self.ships.retain(|s| {
            if s.mmsi.is_some() {
                return true;
            }
            let d = s.pos.distance(e2);
            d < SIM_RADIUS * 1.25 && !(s.stuck && d > 3000.0) && sea.world.water_known(s.pos) != Some(false)
        });
    }

    /// This frame's hulls (eye-relative frames for the aircraft renderer),
    /// their lights and wakes. `view` = the camera's view matrix, `sun_y` =
    /// the sun's elevation sine.
    pub fn render(&mut self, eye: Vec3, view: Mat4, time: f32, sun_y: f32, lights: &mut Vec<LightInst>, wakes: &mut MeshData) -> Vec<AircraftFrame> {
        let t0 = web_time::Instant::now();
        let mut frames = Vec::new();
        if !self.on || self.ships.is_empty() {
            self.stats.drawn = 0;
            return frames;
        }
        let night = ((0.06 - sun_y) / 0.1).clamp(0.0, 1.0);
        let e2 = vec2(eye.x, eye.z);
        let mut near: Vec<(f32, usize)> = self.ships.iter().enumerate().map(|(i, s)| (s.pos.distance(e2), i)).filter(|(d, _)| *d < LIGHTS_DIST).collect();
        near.sort_by(|a, b| a.0.total_cmp(&b.0));
        let (n_lights, n_wake) = (lights.len(), wakes.verts.len());
        let mut drawn = 0;
        for (d, i) in near {
            let s = &self.ships[i];
            let (rot, heave) = render::attitude(s, &self.sea, time);
            let p = Vec3::new(s.pos.x, s.level + heave, s.pos.y);
            // In front of the camera (a generous margin: tall ships, wakes)?
            let vz = view.transform_vector3(p - eye).z;
            let ahead = vz < s.length.max(60.0);
            let m = models::get(s.model).or_else(|| models::get(types::Model::Workboat));
            let k = m.as_ref().map(|m| (s.length / m.length()).clamp(0.2, 4.0)).unwrap_or(1.0);
            let top = m.as_ref().map(|m| m.hi.y * k).unwrap_or(s.length * 0.2).max(2.5);
            let hb = m.as_ref().map(|m| m.lo.x.abs().max(m.hi.x) * k).unwrap_or(s.beam * 0.5);
            if ahead && drawn < MODEL_BUDGET && d < MODEL_DIST && s.length > d * 0.0012 {
                if let Some(m) = &m {
                    frames.push(m.frame(render::body(m, s, p, rot, eye)));
                    drawn += 1;
                }
            }
            render::lights(s, p - eye, rot, top, hb, night, time, lights);
            if ahead && d < render::WAKE_DIST {
                render::wake(s, eye, s.level - eye.y, wakes);
            }
        }
        self.stats.drawn = drawn;
        self.stats.lights = lights.len() - n_lights;
        self.stats.wake_verts = wakes.verts.len() - n_wake;
        self.stats.render_ms = t0.elapsed().as_secs_f32() * 1000.0;
        self.frames += 1;
        if self.frames == 3 && std::env::var_os("SKYWARD_SHIPS_DEBUG").is_some() {
            eprintln!("ships: {:?}", self.stats);
            for l in lights[n_lights..].iter().take(6) {
                eprintln!("  light {:?} {:.1} cd kind {}", l.pos, l.cd, l.kind());
            }
            if let Some(proj) = self.proj {
                for l in self.debug_lines(&proj, eye).iter().take(12) {
                    eprintln!("  {l}");
                }
            }
        }
        frames
    }

    /// One line per ship (diagnostics).
    pub fn debug_lines(&self, proj: &Proj, eye: Vec3) -> Vec<String> {
        let mut v: Vec<&Ship> = self.ships.iter().collect();
        let e2 = vec2(eye.x, eye.z);
        v.sort_by(|a, b| a.pos.distance(e2).total_cmp(&b.pos.distance(e2)));
        v.iter()
            .map(|s| {
                let (la, lo) = proj.to_latlon(s.pos);
                format!(
                    "{:>6.0} m  {:<16} {:<14} {:>5.0} m  T {:>4.1}  {:>3.0}°  {:>4.1} kn  {}  {:.4},{:.4}  {:?}",
                    s.pos.distance(e2),
                    s.name.chars().take(16).collect::<String>(),
                    s.class.name(),
                    s.length,
                    s.draught,
                    s.heading.to_degrees(),
                    s.speed / 0.514_444,
                    if s.mmsi.is_some() {
                        "AIS"
                    } else if s.anchored {
                        "anchored"
                    } else {
                        "proc"
                    },
                    la,
                    lo,
                    s.model
                )
            })
            .collect()
    }
}

/// Map positions for the moving map / minimap: (world xz, heading rad, length m, live).
pub fn map_marks(ships: &Ships) -> Vec<(Vec2, f32, f32, bool)> {
    ships.ships.iter().map(|s| (s.pos, s.heading, s.length, s.mmsi.is_some())).collect()
}
