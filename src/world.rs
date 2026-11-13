//! The endless streaming world: decides which map tiles should be resident
//! around the aircraft, feeds the worker pool, integrates finished tiles
//! (collision now, GPU meshes via a budgeted upload queue), merges runway
//! pieces across tiles and answers the flight model's environment queries.

use crate::clouds::{CloudField, Weather};
use crate::weather::{self, Live, Season, WindField, Wx, WxRender};
use crate::flight::Environment;
use crate::osm::{Airport, Proj, RunwayPiece, TileId, Xform2, FAR_ZOOM, ZOOM};
use crate::scene::{self, Runway, TileCollision, TreeInst};
use crate::ui::mapdata::MapVectors;
use std::sync::Arc;
use crate::tiles::Pool;
use glam::{vec2, vec3, Vec2, Vec3};
use std::collections::{HashMap, HashSet, VecDeque};
use web_time::{Duration, Instant};

/// Default load radius (m). A z14 tile is ≈1.5 km wide at London's latitude.
pub const LOAD_RADIUS: f32 = 7500.0;
/// Far-field (low-zoom) tiles are kept within this distance (m).
pub const FAR_RADIUS: f32 = 30_000.0;
/// Terrain-only horizon tiles (terrain.rs) out to this distance (m).
pub const HORIZON_RADIUS: f32 = 95_000.0;
/// The frame is re-centred on the aircraft once it is this far (m) from the
/// origin: keeps f32 positions precise and the local projection accurate.
pub const REBASE_DIST: f32 = 20_000.0;
/// ... or this far north / south of it: a new origin latitude rescales the
/// east-west axis (cos lat), so those shifts are kept small (≈0.04 % per
/// shift in the UK — well under a pixel) and frequent.
pub const REBASE_NS: f32 = 2_000.0;
/// Period (m) of the renderer's world-space procedural patterns (scene.wgsl
/// `NP`): the pattern origin is passed modulo this, so it stays precise.
pub const NOISE_PERIOD: f64 = 16_384.0;
/// Extra distance before a loaded tile is evicted (hysteresis).
const UNLOAD_MARGIN: f32 = 1800.0;
/// How far ahead along the velocity vector we prefetch (seconds, capped).
const LOOKAHEAD_S: f32 = 30.0;
/// Default GPU budget for tile meshes (bytes).
pub const GPU_BUDGET: u64 = 1_400 << 20;

/// GPU-bound part of a finished tile, waiting in the upload queue.
pub struct TileMeshes {
    pub id: TileId,
    /// Geometry is in the tile's own frame (`Proj::tile_frame`).
    pub max_h: f32,
    pub min_y: f32,
    pub flat: scene::MeshData,
    /// Base / draped index ranges of `flat` (terrain.rs).
    pub ranges: crate::terrain::FlatRanges,
    /// Water bodies (tile-frame bounds, surface height): the reflection plane.
    pub water: Vec<(Vec2, Vec2, f32)>,
    /// Terrain surface (tile frame).
    pub grid: Arc<crate::terrain::Grid>,
    pub buildings: scene::MeshData,
    pub trees: Vec<TreeInst>,
    pub conifers: usize,
    /// Light sources (lights.rs), tile frame.
    pub lights: crate::lights::TileLights,
}

struct Loaded {
    /// Bounds in the current frame.
    rect: (Vec2, Vec2),
    /// Tile frame → current frame (collision data is in the tile frame).
    xf: Xform2,
    far: bool,
    collision: TileCollision,
    /// Terrain surface (tile frame).
    grid: Arc<crate::terrain::Grid>,
    buildings: usize,
    trees: usize,
}

#[derive(Default, Debug, Clone)]
pub struct Stats {
    pub tiles_built: usize,
    pub from_network: usize,
    pub from_cache: usize,
    pub failures: usize,
    pub fetch_ms: f32,
    pub build_ms: f32,
    pub max_build_ms: f32,
    pub tile_bytes: usize,
}

/// One origin shift: ground-fixed things move with `map` (exact between the
/// two local projections), things carried by the aircraft / the air (the
/// aircraft, cameras, clouds) by the rigid `delta` (= `map` at the aircraft).
#[derive(Clone, Copy, Debug)]
pub struct Rebase {
    pub map: Xform2,
    pub delta: Vec3,
}

impl Rebase {
    pub fn ground(&self, p: Vec2) -> Vec2 {
        self.map.apply(p)
    }
    pub fn shift(&self, p: Vec3) -> Vec3 {
        p + self.delta
    }
    pub fn shift2(&self, p: Vec2) -> Vec2 {
        p + vec2(self.delta.x, self.delta.z)
    }
}

pub struct World {
    /// The current local frame (floating origin, see `rebase`).
    pub proj: Proj,
    /// The frame at the start (restart point, reference for shader patterns).
    pub home: Proj,
    /// Number of origin shifts so far.
    pub rebases: u32,
    /// World-fixed offset of the current frame (m, f64): field coordinates
    /// (wind turbulence, cloud cells) are `p + field_off`, so they don't jump
    /// when the origin moves.
    field_off: (f64, f64),
    pool: Pool,
    tiles: HashMap<TileId, Loaded>,
    failed: HashMap<TileId, (Instant, u32)>,
    /// Finished tiles waiting for GPU upload (nearest first is not required:
    /// the renderer drains this with a per-frame byte budget).
    pub uploads: VecDeque<TileMeshes>,
    /// Tiles the GPU / HUD must drop.
    evicted: Vec<TileId>,
    /// Runway pieces per tile, in the tile's frame.
    pieces: HashMap<TileId, Vec<RunwayPiece>>,
    /// All runways in the current frame: OurAirports runways (exact ends,
    /// widths, thresholds) around the focus, plus tile-merged runways where
    /// OurAirports has none. Longest first.
    pub runways: Vec<Runway>,
    /// Airport / runway database (airports.rs), once loaded.
    pub airport_db: Option<Arc<crate::airports::Db>>,
    /// Where (lat, lon) the database runways were last gathered around.
    db_centre: Option<(f64, f64)>,
    /// Vector moving-map data per tile (tile frame), new ones for the HUD.
    map_vectors: Vec<(TileId, Arc<MapVectors>)>,
    /// Bumped whenever `runways` changes (the GPU runway mesh is rebuilt).
    pub runway_version: u64,
    /// Some runway's elevation is still unknown (waiting for DEM data).
    runway_elev_pending: bool,
    elev_retry: Option<Instant>,
    /// Database runways around `db_centre`, in the current frame.
    db_runways: Vec<Runway>,
    pub airports: Vec<Airport>,
    pub radius: f32,
    radius_eff: f32,
    focus: Vec2,
    vel: Vec2,
    last_schedule: Option<(Instant, Vec2)>,
    dirty: bool,
    /// Reported by the renderer each frame.
    pub gpu_bytes: u64,
    /// Tiles the renderer is still copying to the GPU.
    pub gpu_pending: HashSet<TileId>,
    pub gpu_budget: u64,
    pub stats: Stats,
    /// Weather: volumetric cumulus + the thermals under them.
    pub clouds: CloudField,
    /// Live weather fetcher (METAR + Open-Meteo, see weather.rs).
    pub live: Live,
    /// The weather in effect (live, a fixed METAR or a preset).
    pub wx: Wx,
    /// Follow the live weather / the real local time.
    pub live_weather: bool,
    pub live_time: bool,
    /// Use today's real date (season, sun) even when the clock time is set by hand.
    pub live_date: bool,
    /// Seconds added to this machine's clock for the live time (multiplayer:
    /// the server's UTC is authoritative, net/).
    pub clock_skew_s: i64,
    live_seen: u64,
    /// Bumped whenever `clouds` is replaced.
    pub cloud_version: u64,
    pub wind: WindField,
    pub season: Season,
    /// Sim time (s) and clock (local hours, day of year) of the last tick.
    sim_time: f32,
    clock: (f32, u32),
    /// Real local time text when live time is on.
    pub real_time: Option<weather::RealTime>,
    /// Ground elevation (m) the cloud layers / ground fog are referred to
    /// (reported heights are above the ground): the terrain around the focus.
    pub terrain_base: f32,
    base_check: Option<Instant>,
    /// Resident tiles at the last check (a new tile re-checks at once, so the
    /// result doesn't depend on wall-clock timing: deterministic headless shots).
    base_check_tiles: usize,
    /// Airport detail (airport/): layouts, bakes, stands, vehicles.
    pub airport_detail: crate::airport::AirportScene,
    /// Road traffic over the resident tiles' lane graphs (traffic/).
    pub traffic: crate::traffic::Traffic,
    /// Thunderstorms, lightning, mountain waves, wakes, turbulence setting (wx/).
    pub wxs: crate::wx::WxSim,
}

fn rect_dist(r: (Vec2, Vec2), p: Vec2) -> f32 {
    let d = (r.0 - p).max(p - r.1).max(Vec2::ZERO);
    d.length()
}

impl World {
    pub fn new(lat: f64, lon: f64, workers: usize) -> World {
        let proj = Proj::new(lat, lon);
        World {
            proj,
            home: proj,
            rebases: 0,
            field_off: (0.0, 0.0),
            pool: Pool::new(workers),
            tiles: HashMap::new(),
            failed: HashMap::new(),
            uploads: VecDeque::new(),
            evicted: Vec::new(),
            pieces: HashMap::new(),
            runways: Vec::new(),
            airport_db: None,
            db_centre: None,
            map_vectors: Vec::new(),
            runway_version: 0,
            runway_elev_pending: false,
            elev_retry: None,
            db_runways: Vec::new(),
            airports: Vec::new(),
            radius: LOAD_RADIUS,
            radius_eff: LOAD_RADIUS,
            focus: Vec2::ZERO,
            vel: Vec2::ZERO,
            last_schedule: None,
            dirty: true,
            gpu_bytes: 0,
            gpu_pending: HashSet::new(),
            gpu_budget: GPU_BUDGET,
            stats: Stats::default(),
            clouds: CloudField::new(7, Weather::Scattered),
            live: Live::new(lat, lon, false),
            wx: Wx::preset(Weather::Scattered),
            live_weather: false,
            live_time: false,
            live_date: false,
            clock_skew_s: 0,
            live_seen: 0,
            cloud_version: 1,
            wind: WindField::from_wx(&Wx::preset(Weather::Scattered)),
            season: Season::at(crate::sim::DEFAULT_DAY, lat),
            sim_time: 0.0,
            clock: (crate::sim::DEFAULT_HOURS, crate::sim::DEFAULT_DAY),
            real_time: None,
            terrain_base: 0.0,
            base_check: None,
            base_check_tiles: 0,
            airport_detail: crate::airport::AirportScene::new(),
            traffic: crate::traffic::Traffic::new(proj),
            wxs: crate::wx::WxSim::with_env(),
        }
    }

    // ------------------------------------------------------------ weather

    /// Follow the real weather and time (starts fetching), or stop following.
    pub fn set_live(&mut self, on: bool) {
        self.live_weather = on;
        self.live_time = on;
        self.live_date = on;
        if on {
            self.live.start();
            self.live_seen = 0; // re-apply whatever is known
        }
    }

    /// A cloud preset (menu / --weather): live weather off.
    pub fn set_weather_preset(&mut self, w: Weather) {
        self.live_weather = false;
        self.wx = Wx::preset(w);
        self.wind = WindField { scale: self.wind.scale, ..WindField::from_wx(&self.wx) };
        let drift = self.clouds.drift();
        self.clouds = CloudField::new(self.clouds.seed(), w);
        self.clouds.raise(self.terrain_base);
        self.clouds.set_drift(drift);
        self.cloud_version += 1;
        self.wxs.set_weather(&self.wx, self.clouds.seed(), self.terrain_base);
    }

    /// Fixed weather (e.g. a METAR string): live weather off.
    pub fn set_wx(&mut self, wx: Wx) {
        self.live_weather = false;
        self.wx = wx;
        self.apply_wx();
    }

    fn sun_up(&self) -> f32 {
        crate::atmosphere::sun_direction(self.proj.lat0, self.proj.lon0, self.clock.1, self.clock.0).y.max(0.0)
    }

    fn apply_wx(&mut self) {
        let spec = self.wx.cloud_spec(self.sun_up(), &self.season);
        let drift = self.clouds.drift();
        self.clouds = CloudField::from_spec(self.clouds.seed(), &spec);
        self.clouds.raise(self.terrain_base);
        self.clouds.set_drift(drift);
        self.wind = WindField { scale: self.wind.scale, ..WindField::from_wx(&self.wx) };
        self.cloud_version += 1;
        self.wxs.set_weather(&self.wx, self.clouds.seed(), self.terrain_base);
    }

    /// The player's turbulence setting (Settings): scales gusts / turbulence.
    pub fn set_turbulence(&mut self, mode: crate::wx::turbulence::Mode) {
        self.wxs.mode = mode;
        self.wind.scale = mode.scale();
    }

    /// Current-frame xz of a field-coordinate point (see [`World::to_field`]).
    pub fn from_field(&self, p: Vec2) -> Vec2 {
        vec2((p.x as f64 - self.field_off.0) as f32, (p.y as f64 - self.field_off.1) as f32)
    }

    /// Field coordinates (fixed to the ground, unaffected by the floating origin) of a current-frame point.
    pub fn to_field(&self, p: Vec3) -> Vec3 {
        vec3((p.x as f64 + self.field_off.0) as f32, p.y, (p.z as f64 + self.field_off.1) as f32)
    }

    /// Call once per frame: polls the live data, moves the clouds with the
    /// wind and updates the season. `clock` = the sim's (local hours, day of
    /// year). Returns the real clock to use when live time is on.
    pub fn tick_weather(&mut self, sim_time: f32, clock: (f32, u32)) -> Option<(f32, u32)> {
        self.live.poll();
        let mut out = None;
        self.clock = clock;
        if self.live_time || self.live_date {
            let off = self.live.wx.as_ref().and_then(|w| w.utc_offset_s);
            // The clock is the start's (its zone): the sun then moves smoothly with the aircraft.
            let rt = weather::real_time(self.home.lat0, self.home.lon0, weather::unix_now() + self.clock_skew_s, off);
            self.clock = if self.live_time { (rt.sun_hours, rt.sun_doy) } else { (clock.0, rt.day_of_year) };
            out = Some(self.clock);
            self.real_time = self.live_time.then_some(rt);
        } else {
            self.real_time = None;
        }
        let season = Season::at(self.clock.1, self.proj.lat0);
        let season_changed = (season.leaf - self.season.leaf).abs() > 0.02 || (season.summer - self.season.summer).abs() > 0.05;
        self.season = season;
        if self.live_weather && (self.live.version != self.live_seen) {
            self.live_seen = self.live.version;
            if let Some(w) = self.live.wx.clone() {
                self.wx = w;
                self.apply_wx();
            }
        } else if season_changed && !self.wx.source.starts_with("preset") {
            self.apply_wx();
        }
        self.sim_time = if sim_time.is_finite() { sim_time } else { 0.0 };
        // Weather heights follow the terrain: re-place the layers when the
        // ground around the focus is well above / below what they refer to.
        if self.base_check.is_none_or(|t| t.elapsed() > Duration::from_secs(2)) || self.tiles.len() != self.base_check_tiles {
            self.base_check = Some(Instant::now());
            self.base_check_tiles = self.tiles.len();
            let e = self.regional_elevation();
            if (e - self.terrain_base).abs() > 250.0 {
                let preset = self.wx.source.starts_with("preset");
                self.terrain_base = e;
                if preset && !self.live_weather {
                    let w = self.clouds.weather();
                    let drift = self.clouds.drift();
                    self.clouds = CloudField::new(self.clouds.seed(), w);
                    self.clouds.raise(e);
                    self.clouds.set_drift(drift);
                    self.cloud_version += 1;
                    self.wxs.set_ground(e);
                } else {
                    self.apply_wx();
                }
            }
        }
        self.update_drift();
        // Storms drift with the clouds; lightning; the lee-wave grid (wx/).
        let off = vec2(self.field_off.0 as f32, self.field_off.1 as f32);
        let mut wxs = std::mem::take(&mut self.wxs);
        let fo = self.field_off;
        wxs.tick(self.sim_time, self.focus, off, self.clouds.drift(), self.tiles.len(), &|p: Vec2| self.terrain_height(vec2((p.x as f64 - fo.0) as f32, (p.y as f64 - fo.1) as f32)));
        self.wxs = wxs;
        out
    }

    /// Mean ground height around the focus (±3 km).
    fn regional_elevation(&self) -> f32 {
        let f = self.focus;
        let pts = [vec2(0.0, 0.0), vec2(3000.0, 0.0), vec2(-3000.0, 0.0), vec2(0.0, 3000.0), vec2(0.0, -3000.0)];
        pts.iter().map(|o| self.terrain_height(f + *o)).sum::<f32>() / pts.len() as f32
    }

    /// Clouds drift with the wind aloft; the field also stays put in the world when the origin moves.
    fn update_drift(&mut self) {
        let wind = self.wind.aloft * (self.sim_time % 1.0e6);
        self.clouds.set_drift(vec2((wind.x as f64 - self.field_off.0) as f32, (wind.y as f64 - self.field_off.1) as f32));
    }

    // ------------------------------------------------------------ floating origin

    /// The start point (origin of the home frame) in current coordinates.
    pub fn home_pos(&self) -> Vec2 {
        self.home.map_to(&self.proj).apply(Vec2::ZERO)
    }

    /// Shader pattern frame: (x scale, x offset, z offset, 0) mapping
    /// current-frame xz to a fixed world "pattern space" (the home frame's
    /// metres), offsets modulo [`NOISE_PERIOD`].
    pub fn pattern_frame(&self) -> [f32; 4] {
        let m = self.proj.map_to(&self.home);
        let wrap = |v: f64| (v - NOISE_PERIOD * (v / NOISE_PERIOD).round()) as f32;
        [m.sx as f32, wrap(m.tx), wrap(m.tz), 0.0]
    }

    /// Distance (m) of a current-frame point from a lat/lon (local flat
    /// approximation around the point; fine for reports).
    pub fn distance_to_geo(&self, p: Vec2, lat: f64, lon: f64) -> f32 {
        let (la, lo) = self.proj.to_latlon(p);
        let k = crate::osm::M_PER_DEG_LAT;
        let dx = (lo - lon) * k * ((la + lat) * 0.5).to_radians().cos();
        let dz = (la - lat) * k;
        (dx * dx + dz * dz).sqrt() as f32
    }

    /// True when `p` (the aircraft) is far enough from the origin to re-centre.
    pub fn wants_rebase(&self, p: Vec2) -> bool {
        p.length() > REBASE_DIST || p.y.abs() > REBASE_NS
    }

    /// Re-centre the frame on `anchor` (current-frame xz, normally the
    /// aircraft). Moves everything the world owns; returns the shift the
    /// caller applies to its own state (aircraft, cameras, bot, renderer).
    pub fn rebase(&mut self, anchor: Vec2) -> Rebase {
        let (lat, lon) = self.proj.to_latlon(anchor);
        self.rebase_to(Proj::new(lat, lon), anchor)
    }

    /// Switch to the frame `new` (`anchor` decides the rigid shift).
    pub fn rebase_to(&mut self, new: Proj, anchor: Vec2) -> Rebase {
        let map = self.proj.map_to(&new);
        let a2 = map.apply(anchor);
        let delta = vec3(a2.x - anchor.x, 0.0, a2.y - anchor.y);
        // Field coordinates (p + field_off) must not change: p' = p + delta.
        // Accumulate the exact (f64) shift, not the f32 one.
        let dx = map.sx * anchor.x as f64 + map.tx - anchor.x as f64;
        let dz = map.tz;
        self.field_off = (self.field_off.0 - dx, self.field_off.1 - dz);
        self.proj = new;
        self.rebases += 1;
        self.traffic.rebase(new);
        for (id, t) in self.tiles.iter_mut() {
            t.xf = Proj::tile_frame(*id).map_to(&new);
            t.rect = id.rect(&new);
        }
        for a in &mut self.airports {
            a.pos = map.apply(a.pos);
        }
        for r in &mut self.db_runways {
            r.a = map.apply(r.a);
            r.b = map.apply(r.b);
        }
        self.merge_runways();
        self.runway_version += 1;
        self.focus = map.apply(self.focus);
        self.last_schedule = None;
        self.dirty = true;
        self.update_drift();
        self.wxs.storms.set_drift(self.clouds.drift());
        // Live weather follows the flight (re-fetched every 50 km).
        self.live.relocate(new.lat0, new.lon0);
        Rebase { map, delta }
    }

    /// Weather parameters for the renderer.
    pub fn render_weather(&self) -> WxRender {
        let cirrus_drift = self.wind.aloft * (self.sim_time % 1.0e6) * 1.5;
        let mut r = WxRender { drift: self.clouds.drift(), cirrus_drift, season: self.season, cloud_version: self.cloud_version, ..self.wx.render(self.sun_up()) };
        if r.fog_top > 0.0 {
            // Ground fog lies on the terrain (top above the local ground).
            r.fog_top += self.terrain_base;
        }
        // Under a storm's rain shaft: heavy rain, wet ground, murk.
        if self.wxs.storms.active() {
            let p = vec3(self.focus.x, self.terrain_height(self.focus) + 50.0, self.focus.y);
            let (mmh, _) = self.wxs.rain_at(p);
            if mmh > 0.5 {
                let k = (mmh / 60.0).clamp(0.0, 1.0);
                if r.snow > 0.0 {
                    r.snow = r.snow.max(0.3 + 0.7 * k);
                } else {
                    r.rain = r.rain.max(0.3 + 0.7 * k);
                }
                r.wet = r.wet.max(0.55 + 0.45 * k);
                // Visibility in heavy rain: ~1-3 km.
                r.fog_ext = r.fog_ext.max(0.3 + 1.6 * k);
                r.fog_top = r.fog_top.max(self.terrain_base + self.wxs.storms.spec.base * 0.9);
            }
        }
        r
    }

    /// Ground roughness 0 (open) .. 1 (dense city) from the tile's buildings.
    pub fn roughness(&self, xz: Vec2) -> f32 {
        self.tile_at(xz).map_or(0.0, |t| (t.buildings as f32 / 3000.0).clamp(0.0, 1.0))
    }

    /// Where the streaming is centred (aircraft / camera) and how it moves.
    pub fn set_focus(&mut self, p: Vec2, vel: Vec2) {
        self.focus = p;
        self.vel = vel;
    }

    fn ahead(&self) -> Vec2 {
        let v = self.vel * LOOKAHEAD_S;
        let max = self.radius_eff * 0.6;
        self.focus + if v.length() > max { v.normalize() * max } else { v }
    }

    /// Integrate finished tiles, evict far ones, reschedule work. Cheap; call every frame.
    pub fn update(&mut self) {
        self.update_db_area(false);
        if self.runway_elev_pending && self.elev_retry.is_none_or(|t| t.elapsed() > Duration::from_millis(500)) {
            self.elev_retry = Some(Instant::now());
            self.update_runway_elevations();
            // Make sure the DEM data of runways nobody streams arrives.
            for r in self.runways.iter().filter(|r| !r.elev_known) {
                for p in [r.a, r.b, (r.a + r.b) * 0.5] {
                    let (lat, lon) = self.proj.to_latlon(p);
                    crate::terrain::prefetch(lat, lon);
                }
            }
        }
        while let Some(r) = self.pool.try_recv() {
            self.dirty = true;
            if let Some(m) = r.map {
                self.map_vectors.push((r.id, m));
            }
            match r.build {
                Ok(b) => {
                    self.failed.remove(&r.id);
                    let st = &mut self.stats;
                    st.tiles_built += 1;
                    if r.from_cache {
                        st.from_cache += 1
                    } else {
                        st.from_network += 1
                    }
                    st.fetch_ms += r.fetch_ms;
                    st.build_ms += r.build_ms;
                    st.max_build_ms = st.max_build_ms.max(r.build_ms);
                    st.tile_bytes += r.bytes;
                    // A tile that drifted out of range while building is dropped.
                    if self.keep(b.id, b.id.rect(&self.proj)) {
                        self.integrate(b);
                    }
                }
                Err(e) => {
                    self.stats.failures += 1;
                    let attempts = self.failed.get(&r.id).map_or(0, |f| f.1) + 1;
                    let wait = Duration::from_secs((2u64 << attempts.min(5)).min(60));
                    if attempts <= 2 {
                        eprintln!("tile {}/{}/{}: {e} (retry in {}s)", r.id.z, r.id.x, r.id.y, wait.as_secs());
                    }
                    self.failed.insert(r.id, (Instant::now() + wait, attempts));
                }
            }
        }

        // GPU budget: shrink the radius when over, regrow slowly when well under.
        if self.gpu_bytes > self.gpu_budget {
            self.radius_eff = (self.radius_eff - 300.0).max(3000.0);
            self.dirty = true;
        } else if self.gpu_bytes < self.gpu_budget * 3 / 4 && self.radius_eff < self.radius {
            self.radius_eff = (self.radius_eff + 50.0).min(self.radius);
        }

        self.airport_detail.update();
        let due = match self.last_schedule {
            None => true,
            Some((t, p)) => t.elapsed() > Duration::from_millis(250) || p.distance(self.focus) > 150.0,
        };
        if self.dirty || due {
            self.schedule();
        }
    }

    /// Add a finished tile: collision now, GPU meshes via the upload queue.
    fn integrate(&mut self, b: scene::TileBuild) {
        let xf = Proj::tile_frame(b.id).map_to(&self.proj);
        let rect = b.id.rect(&self.proj);
        if !b.runways.is_empty() && !self.pieces.contains_key(&b.id) {
            self.pieces.insert(b.id, b.runways.clone());
            self.merge_runways();
        }
        for a in &b.airports {
            let a = &Airport { pos: xf.apply(a.pos), ..a.clone() };
            let dup = self.airports.iter().any(|o| {
                (!a.icao.is_empty() && o.icao == a.icao) || (a.icao.is_empty() && o.name == a.name && o.pos.distance(a.pos) < 3000.0)
            });
            if !dup {
                self.airports.push(a.clone());
            }
        }
        if let Some(a) = &b.aero {
            self.airport_detail.add_tile(b.id, a.clone(), b.grid.clone(), self.airport_db.as_ref());
        }
        if let Some(r) = &b.roads {
            self.traffic.add_tile(r.clone());
        }
        let trees = b.trees.len();
        let water = b.collision.water_levels(&b.grid);
        self.tiles.insert(b.id, Loaded { rect, xf, far: b.id.is_far(), collision: b.collision, grid: b.grid.clone(), buildings: b.building_count, trees });
        self.uploads.push_back(TileMeshes { id: b.id, max_h: b.max_h, min_y: b.min_y, flat: b.flat, ranges: b.ranges, water, grid: b.grid, buildings: b.buildings, trees: b.trees, conifers: b.conifers, lights: b.lights });
        if self.runway_elev_pending {
            self.update_runway_elevations();
        }
    }

    fn keep(&self, id: TileId, rect: (Vec2, Vec2)) -> bool {
        if id.z == crate::terrain::HORIZON_ZOOM {
            return rect_dist(rect, self.focus) <= HORIZON_RADIUS + 20_000.0;
        }
        if id.is_far() {
            return rect_dist(rect, self.focus) <= FAR_RADIUS + 8000.0;
        }
        let lim = self.radius_eff + UNLOAD_MARGIN;
        rect_dist(rect, self.focus) <= lim || rect_dist(rect, self.ahead()) <= lim
    }

    /// Tiles that should be resident, best first.
    fn desired(&self) -> Vec<(TileId, f32)> {
        let r = self.radius_eff;
        let ahead = self.ahead();
        let lo = self.focus.min(ahead) - Vec2::splat(r);
        let hi = self.focus.max(ahead) + Vec2::splat(r);
        let (lat_n, lon_w) = self.proj.to_latlon(lo);
        let (lat_s, lon_e) = self.proj.to_latlon(hi);
        let (x0, y0) = TileId::frac(lat_n, lon_w, ZOOM);
        let (x1, y1) = TileId::frac(lat_s, lon_e, ZOOM);
        let vdir = self.vel.normalize_or_zero();
        let mut out = Vec::new();
        for y in y0.floor() as u32..=y1.floor() as u32 {
            for x in x0.floor() as u32..=x1.floor() as u32 {
                let id = TileId::new(x, y);
                let rect = id.rect(&self.proj);
                let d = rect_dist(rect, self.focus);
                let da = rect_dist(rect, ahead);
                if d > r && da > r * 0.8 {
                    continue;
                }
                // Prefer near tiles, then those along the flight path.
                let c = (rect.0 + rect.1) * 0.5 - self.focus;
                let along = if c.length() > 1.0 { c.normalize().dot(vdir).max(0.0) } else { 1.0 };
                let score = d.min(da + 600.0) - 0.4 * along * d.min(3000.0);
                out.push((id, score));
            }
        }
        out.sort_by(|a, b| a.1.total_cmp(&b.1));
        out
    }

    /// Terrain-only horizon tiles that should be resident, nearest first.
    fn desired_horizon(&self) -> Vec<TileId> {
        self.desired_ring(HORIZON_RADIUS, crate::terrain::HORIZON_ZOOM)
    }

    /// Far-field tiles that should be resident, nearest first.
    fn desired_far(&self) -> Vec<TileId> {
        self.desired_ring(FAR_RADIUS, FAR_ZOOM)
    }

    fn desired_ring(&self, r: f32, zoom: u32) -> Vec<TileId> {
        let (lat_n, lon_w) = self.proj.to_latlon(self.focus - Vec2::splat(r));
        let (lat_s, lon_e) = self.proj.to_latlon(self.focus + Vec2::splat(r));
        let (x0, y0) = TileId::frac(lat_n, lon_w, zoom);
        let (x1, y1) = TileId::frac(lat_s, lon_e, zoom);
        let mut out = Vec::new();
        for y in y0.floor() as u32..=y1.floor() as u32 {
            for x in x0.floor() as u32..=x1.floor() as u32 {
                let id = TileId { z: zoom, x, y };
                let d = rect_dist(id.rect(&self.proj), self.focus);
                if d <= r {
                    out.push((id, d));
                }
            }
        }
        out.sort_by(|a, b| a.1.total_cmp(&b.1));
        out.into_iter().map(|(id, _)| id).collect()
    }

    fn schedule(&mut self) {
        self.dirty = false;
        self.last_schedule = Some((Instant::now(), self.focus));
        // Evict far tiles.
        let gone: Vec<TileId> = self.tiles.iter().filter(|(id, t)| !self.keep(**id, t.rect)).map(|(id, _)| *id).collect();
        for id in gone {
            self.tiles.remove(&id);
            self.uploads.retain(|u| u.id != id);
            self.evicted.push(id);
            self.airport_detail.remove_tile(id);
            self.traffic.remove_tile(id);
        }
        let now = Instant::now();
        let busy = self.pool.in_flight();
        let wanted = |id: &TileId| !self.tiles.contains_key(id) && !busy.contains(id) && self.failed.get(id).is_none_or(|(t, _)| now >= *t);
        let mut jobs: Vec<TileId> = self.desired().into_iter().map(|(id, _)| id).filter(wanted).collect();
        // Far-field tiles go in after the most urgent detailed tiles.
        let far: Vec<TileId> = self.desired_far().into_iter().filter(wanted).collect();
        let at = jobs.len().min(6);
        jobs.splice(at..at, far);
        // Horizon terrain: after the nearest detailed tiles too, before the rest.
        let horizon: Vec<TileId> = self.desired_horizon().into_iter().filter(wanted).collect();
        let at = jobs.len().min(16);
        jobs.splice(at..at, horizon);
        self.pool.set_jobs(jobs);
    }

    fn merge_runways(&mut self) {
        // Pieces are stored in their tile's frame; merge in the current one.
        let owned: Vec<RunwayPiece> = self
            .pieces
            .iter()
            .flat_map(|(id, v)| {
                let xf = Proj::tile_frame(*id).map_to(&self.proj);
                v.iter().map(move |p| RunwayPiece { a: xf.apply(p.a), b: xf.apply(p.b), name: p.name.clone() })
            })
            .collect();
        let all: Vec<&RunwayPiece> = owned.iter().filter(|p| p.a.distance(p.b) > 20.0).collect();
        let n = all.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut [usize], i: usize) -> usize {
            let mut i = i;
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        for i in 0..n {
            for j in i + 1..n {
                if same_runway(all[i], all[j]) {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                }
            }
        }
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..n {
            let r = find(&mut parent, i);
            groups.entry(r).or_default().push(i);
        }
        let mut runways = Vec::new();
        for (_, g) in groups {
            let longest = *g.iter().max_by(|&&a, &&b| all[a].a.distance(all[a].b).total_cmp(&all[b].a.distance(all[b].b))).unwrap();
            let (p0, d) = (all[longest].a, (all[longest].b - all[longest].a).normalize());
            let (mut tmin, mut tmax) = (f32::MAX, f32::MIN);
            let mut name = String::new();
            for &i in &g {
                for q in [all[i].a, all[i].b] {
                    let t = (q - p0).dot(d);
                    tmin = tmin.min(t);
                    tmax = tmax.max(t);
                }
                if name.is_empty() {
                    name = all[i].name.clone();
                }
            }
            let len = tmax - tmin;
            if len < 250.0 {
                continue;
            }
            runways.push(Runway { a: p0 + d * tmin, b: p0 + d * tmax, width: Runway::width_for(len), name, paved: true, ..Default::default() });
        }
        // OurAirports first; tile runways only where the database has nothing.
        // (Estimated database runways give way to a tile runway at the same field.)
        let db = &self.db_runways;
        let covered = |r: &Runway| db.iter().any(|o| o.source == scene::RunwaySource::OurAirports && same_strip(o, r));
        runways.retain(|r| !covered(r));
        let tile_has = |o: &Runway| runways.iter().any(|r| same_strip(o, r) || (r.a + r.b).distance(o.a + o.b) < 2.0 * 1500.0 && o.source == scene::RunwaySource::Estimated && angle_close(o, r));
        let mut all: Vec<Runway> = db.iter().filter(|o| o.source == scene::RunwaySource::OurAirports || !tile_has(o)).cloned().collect();
        all.extend(runways);
        let mut runways = all;
        runways.sort_by(|a, b| b.length().total_cmp(&a.length()));
        let changed = runways.len() != self.runways.len()
            || runways.iter().zip(&self.runways).any(|(a, b)| a.a.distance(b.a) > 0.5 || a.b.distance(b.b) > 0.5);
        if changed {
            // Keep known elevations (re-resolved below).
            for r in &mut runways {
                if let Some(o) = self.runways.iter().find(|o| o.name == r.name && o.airport == r.airport && o.a.distance(r.a) < 0.5) {
                    r.elev = o.elev;
                }
            }
            self.runways = runways;
            self.runway_version += 1;
            self.runway_elev_pending = true;
            self.update_runway_elevations();
        }
    }

    /// Runway plane heights (terrain.rs: the same profiles the tiles are
    /// flattened to). Non-blocking: retried as DEM data arrives.
    fn update_runway_elevations(&mut self) {
        let mut pending = false;
        let mut changed = false;
        for r in &mut self.runways {
            let e = match r.source {
                scene::RunwaySource::Tiles => crate::terrain::fit_elevation(r, &self.proj),
                _ => crate::terrain::runway_elevation(r, &self.proj).or_else(|| if crate::terrain::has_airport_db() { None } else { crate::terrain::fit_elevation(r, &self.proj) }),
            };
            match e {
                Some((a, b)) => {
                    if (r.elev[0] - a).abs() > 0.01 || (r.elev[1] - b).abs() > 0.01 || !r.elev_known {
                        r.elev = [a, b];
                        r.elev_known = true;
                        changed = true;
                    }
                }
                None => pending |= !r.elev_known,
            }
        }
        self.runway_elev_pending = pending;
        if changed && std::env::var_os("SKYWARD_TERRAIN_DEBUG").is_some() {
            for r in self.runways.iter().take(12) {
                eprintln!("runway {} {} {:?} elev {:?} known {}", r.airport, r.name, r.source, r.elev, r.elev_known);
            }
        }
        if changed {
            self.runway_version += 1;
        }
    }

    /// Main runway of the airfield nearest to `p`: the longest runway within
    /// 3 km of the nearest one (so a short strip next to the airport
    /// reference point doesn't win over the real main runway).
    pub fn spawn_runway(&self, p: Vec2) -> Option<&Runway> {
        let d = |r: &Runway| seg_dist(p, r.a, r.b);
        let dmin = self.runways.iter().map(d).fold(f32::MAX, f32::min);
        self.runways.iter().filter(|r| d(r) <= dmin + 3000.0).max_by(|a, b| a.length().total_cmp(&b.length()))
    }

    /// (ready, wanted) tiles whose rect lies within `r` of `p` — "ready" means
    /// built and uploaded.
    pub fn progress(&self, p: Vec2, r: f32) -> (usize, usize) {
        let (lat_n, lon_w) = self.proj.to_latlon(p - Vec2::splat(r));
        let (lat_s, lon_e) = self.proj.to_latlon(p + Vec2::splat(r));
        let (x0, y0) = TileId::frac(lat_n, lon_w, ZOOM);
        let (x1, y1) = TileId::frac(lat_s, lon_e, ZOOM);
        let mut pending: HashSet<TileId> = self.uploads.iter().map(|u| u.id).collect();
        pending.extend(self.gpu_pending.iter().copied());
        let (mut ready, mut total) = (0, 0);
        for y in y0.floor() as u32..=y1.floor() as u32 {
            for x in x0.floor() as u32..=x1.floor() as u32 {
                let id = TileId::new(x, y);
                if rect_dist(id.rect(&self.proj), p) > r {
                    continue;
                }
                total += 1;
                // Failed tiles count as done for progress so offline gaps don't hang loading.
                let failed_hard = self.failed.get(&id).is_some_and(|f| f.1 >= 2);
                if (self.tiles.contains_key(&id) && !pending.contains(&id)) || failed_hard {
                    ready += 1;
                }
            }
        }
        (ready, total)
    }

    /// Diagnostics: tiles within `r` of `p` that are not ready, with why.
    pub fn missing(&self, p: Vec2, r: f32) -> Vec<String> {
        let (lat_n, lon_w) = self.proj.to_latlon(p - Vec2::splat(r));
        let (lat_s, lon_e) = self.proj.to_latlon(p + Vec2::splat(r));
        let (x0, y0) = TileId::frac(lat_n, lon_w, ZOOM);
        let (x1, y1) = TileId::frac(lat_s, lon_e, ZOOM);
        let busy = self.pool.in_flight();
        let mut out = Vec::new();
        for y in y0.floor() as u32..=y1.floor() as u32 {
            for x in x0.floor() as u32..=x1.floor() as u32 {
                let id = TileId::new(x, y);
                if rect_dist(id.rect(&self.proj), p) > r {
                    continue;
                }
                let st = if self.tiles.contains_key(&id) {
                    if self.uploads.iter().any(|u| u.id == id) || self.gpu_pending.contains(&id) { "uploading" } else { continue }
                } else if busy.contains(&id) {
                    "in flight"
                } else if let Some(f) = self.failed.get(&id) {
                    if f.1 >= 2 { continue } else { "failed once" }
                } else {
                    "not requested"
                };
                out.push(format!("{}/{}/{} {st}", id.z, id.x, id.y));
            }
        }
        out
    }

    /// True when every far-field tile in range is resident (or has failed).
    pub fn far_ready(&self) -> bool {
        self.desired_far().iter().all(|id| self.tiles.contains_key(id) || self.failed.contains_key(id))
    }

    /// Ground height from the finest resident surface, else the DEM tiles in
    /// memory, else sea level.
    fn terrain_height(&self, xz: Vec2) -> f32 {
        for z in [ZOOM, FAR_ZOOM, crate::terrain::HORIZON_ZOOM] {
            if let Some(t) = self.tiles.get(&TileId::at_world_z(&self.proj, xz, z)) {
                return t.grid.height(t.xf.inverse(xz));
            }
        }
        let (lat, lon) = self.proj.to_latlon(xz);
        crate::terrain::cached_height(lat, lon).unwrap_or(0.0)
    }

    /// Ground height where terrain data is resident, None while it streams
    /// (the bot's procedure planner re-plans once it is known).
    pub fn ground_height_known(&self, xz: Vec2) -> Option<f32> {
        for z in [ZOOM, FAR_ZOOM, crate::terrain::HORIZON_ZOOM] {
            if let Some(t) = self.tiles.get(&TileId::at_world_z(&self.proj, xz, z)) {
                return Some(t.grid.height(t.xf.inverse(xz)));
            }
        }
        let (lat, lon) = self.proj.to_latlon(xz);
        crate::terrain::cached_height(lat, lon)
    }

    /// Water-surface height of the water body nearest `xz` within `r` (m).
    pub fn water_level_near(&self, xz: Vec2, r: f32) -> Option<f32> {
        let t = self.tiles.get(&TileId::at_world(&self.proj, xz))?;
        t.collision.nearest_water(t.xf.inverse(xz), r).map(|p| t.grid.height(p))
    }

    pub fn take_evicted(&mut self) -> Vec<TileId> {
        std::mem::take(&mut self.evicted)
    }


    /// New vector moving-map tiles (tile frame).
    pub fn take_map_vectors(&mut self) -> Vec<(TileId, Arc<MapVectors>)> {
        std::mem::take(&mut self.map_vectors)
    }

    /// Every resident tile (e.g. to clear the renderer when the world is replaced).
    pub fn tile_ids(&self) -> Vec<TileId> {
        self.tiles.keys().copied().collect()
    }

    // ------------------------------------------------------------ runway source

    /// Use the OurAirports database for runways (and airport names).
    pub fn set_airport_db(&mut self, db: Arc<crate::airports::Db>) {
        let late = !crate::terrain::has_airport_db();
        crate::terrain::set_airport_db(db.clone());
        if late {
            // Detailed tiles built before the database have no runway flattening: rebuild them.
            let near: Vec<TileId> = self.tiles.keys().filter(|id| !id.is_far()).copied().collect();
            for id in near {
                self.tiles.remove(&id);
                self.uploads.retain(|u| u.id != id);
                self.evicted.push(id);
                self.airport_detail.remove_tile(id);
                self.traffic.remove_tile(id);
            }
        }
        self.airport_db = Some(db);
        self.db_centre = None;
        self.update_db_area(true);
    }

    /// Gather the database's runways / airports around the focus (again
    /// after the frame moved, or once the focus is 40 km from the last area).
    fn update_db_area(&mut self, force: bool) {
        let Some(db) = self.airport_db.clone() else { return };
        let (lat, lon) = self.proj.to_latlon(self.focus);
        if !force && self.db_centre.is_some_and(|c| crate::airports::distance_km(c, (lat, lon)) < DB_REFRESH_KM) {
            return;
        }
        self.db_centre = Some((lat, lon));
        self.db_runways = db.runways_near(&self.proj, lat, lon, DB_RADIUS_KM);
        for (i, _) in db.near(lat, lon, DB_RADIUS_KM) {
            let a = &db.airports[i];
            let code = a.code().to_string();
            if self.airports.iter().any(|o| o.icao == code || (!a.icao.is_empty() && o.icao == a.icao)) {
                continue;
            }
            self.airports.push(Airport { pos: self.proj.to_world(a.lat, a.lon), icao: code, iata: a.iata.clone(), name: a.name.clone() });
        }
        self.merge_runways();
    }

    /// Detailed (z14) tiles resident.
    pub fn tile_count(&self) -> usize {
        self.tiles.values().filter(|t| !t.far).count()
    }
    pub fn far_count(&self) -> usize {
        self.tiles.values().filter(|t| t.far).count()
    }
    pub fn building_count(&self) -> usize {
        self.tiles.values().map(|t| t.buildings).sum()
    }
    pub fn tree_count(&self) -> usize {
        self.tiles.values().map(|t| t.trees).sum()
    }
    pub fn queued(&self) -> usize {
        self.pool.queued() + self.pool.in_flight().len()
    }
    pub fn downloaded_bytes(&self) -> u64 {
        self.pool.downloaded_bytes()
    }
    pub fn effective_radius(&self) -> f32 {
        self.radius_eff
    }
    pub fn focus(&self) -> Vec2 {
        self.focus
    }

    fn tile_at(&self, p: Vec2) -> Option<&Loaded> {
        self.tiles.get(&TileId::at_world(&self.proj, p))
    }
}

/// Runways gathered from the database within this radius of the focus (km)...
const DB_RADIUS_KM: f64 = 90.0;
/// ... again once the focus has moved this far (km).
const DB_REFRESH_KM: f64 = 40.0;

/// Two runways (any source) describing the same strip: parallel within 6°,
/// centrelines within 60 m, overlapping.
fn same_strip(p: &Runway, q: &Runway) -> bool {
    let dp = (p.b - p.a).normalize_or_zero();
    let dq = (q.b - q.a).normalize_or_zero();
    if dp.perp_dot(dq).abs() > 6f32.to_radians().sin() {
        return false;
    }
    let nrm = vec2(-dp.y, dp.x);
    if (q.a - p.a).dot(nrm).abs() > 60.0 || (q.b - p.a).dot(nrm).abs() > 60.0 {
        return false;
    }
    let (ta, tb) = ((q.a - p.a).dot(dp), (q.b - p.a).dot(dp));
    ta.min(tb) <= p.length() && ta.max(tb) >= 0.0
}

fn angle_close(p: &Runway, q: &Runway) -> bool {
    let dp = (p.b - p.a).normalize_or_zero();
    let dq = (q.b - q.a).normalize_or_zero();
    dp.perp_dot(dq).abs() < 12f32.to_radians().sin()
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Collinear, overlapping (or nearly touching) runway centreline pieces.
fn same_runway(p: &RunwayPiece, q: &RunwayPiece) -> bool {
    let dp = (p.b - p.a).normalize();
    let dq = (q.b - q.a).normalize();
    if dp.perp_dot(dq).abs() > 3f32.to_radians().sin() {
        return false;
    }
    let nrm = vec2(-dp.y, dp.x);
    if (q.a - p.a).dot(nrm).abs() > 25.0 || (q.b - p.a).dot(nrm).abs() > 25.0 {
        return false;
    }
    let (ta, tb) = ((q.a - p.a).dot(dp), (q.b - p.a).dot(dp));
    let (lo, hi) = (ta.min(tb), ta.max(tb));
    let len = p.a.distance(p.b);
    lo <= len + 60.0 && hi >= -60.0
}

impl Environment for World {
    fn ground_height(&self, xz: Vec2) -> f32 {
        self.terrain_height(xz)
    }
    fn obstacle_top(&self, xz: Vec2) -> Option<f32> {
        self.tile_at(xz).and_then(|t| t.collision.obstacle_top(t.xf.inverse(xz)))
    }
    fn is_water(&self, xz: Vec2) -> bool {
        self.tile_at(xz).is_some_and(|t| t.collision.is_water(t.xf.inverse(xz)))
    }
    fn updraft(&self, p: Vec3) -> f32 {
        // Thermals, storm drafts and mountain waves (wx/), plus slope lift where
        // the wind meets rising ground (ridge.rs).
        let xz = vec2(p.x, p.z);
        let w = self.wind.mean(p.y - self.terrain_height(xz), 0.3);
        self.clouds.updraft(p) + self.wxs.updraft(p, self.to_field(p)) + crate::ridge::slope_lift(|q| self.terrain_height(q), p, w)
    }
    fn wind(&self, p: Vec3) -> Vec3 {
        let xz = vec2(p.x, p.z);
        // Convective turbulence: in / under the cumulus and around thermals
        // (light bumps under fair-weather cumulus, moderate in towering ones).
        let lift = self.clouds.updraft(p).abs();
        let in_cloud = if p.y > self.clouds.cu_base() - 50.0 && p.y < self.clouds.top() { self.clouds.coverage(xz) } else { 0.0 };
        let conv = (lift / 3.0).max(in_cloud).clamp(0.0, 1.0);
        let deep = (self.clouds.top() - self.clouds.cu_base()) / 3000.0;
        let pfield = self.to_field(p);
        // Storm outflow / mountain waves / wakes and their turbulence.
        let (extra, _, sev) = self.wxs.air(p, pfield);
        let severity = (conv * (1.1 + deep.clamp(0.0, 1.4))).max(sev);
        // (The boundary layer follows the terrain: height above the ground.)
        let pf = vec3(pfield.x, p.y - self.terrain_height(xz), pfield.z);
        self.wind.at(pf, self.sim_time, self.roughness(xz), severity) + extra
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn piece(a: Vec2, b: Vec2) -> RunwayPiece {
        RunwayPiece { a, b, name: String::new() }
    }

    #[test]
    fn runway_pieces_merge() {
        let p = piece(vec2(0.0, 0.0), vec2(800.0, 10.0));
        let q = piece(vec2(1500.0, 19.0), vec2(780.0, 9.8));
        assert!(same_runway(&p, &q));
        let r = piece(vec2(0.0, 300.0), vec2(800.0, 310.0));
        assert!(!same_runway(&p, &r));
    }

    // ------------------------------------------------------------ floating origin

    use crate::flight::{self, Aircraft, Controls};
    use crate::osm::{Area, Kind, TileFeatures};

    const LAT: f64 = 51.5045;
    const LON: f64 = 0.03;

    fn area(kind: Kind, c: Vec2, h: Vec2) -> Area {
        let outer = vec![c + vec2(-h.x, -h.y), c + vec2(h.x, -h.y), c + vec2(h.x, h.y), c + vec2(-h.x, h.y)];
        Area { kind, border: vec![false; outer.len()], outer, holes: vec![], hole_border: vec![], key: 1 }
    }

    /// A z14 tile near the origin with a tower, a pond and a 1 km runway
    /// (tile-local feature positions returned for the checks).
    fn test_tile(id: TileId) -> (scene::TileBuild, Vec2, Vec2, (Vec2, Vec2)) {
        let tp = Proj::tile_frame(id);
        let (lo, hi) = id.rect(&tp);
        let c = (lo + hi) * 0.5;
        let tower = c + vec2(-250.0, -200.0);
        let pond = c + vec2(250.0, -200.0);
        let rw = (c + vec2(-500.0, 300.0), c + vec2(500.0, 330.0));
        let f = TileFeatures {
            areas: vec![
                area(Kind::Building { height: 60.0, min_height: 0.0, colour: None }, tower, vec2(20.0, 25.0)),
                area(Kind::Water, pond, vec2(90.0, 60.0)),
            ],
            lines: vec![],
            runways: vec![RunwayPiece { a: rw.0, b: rw.1, name: "09/27".into() }],
            airports: vec![],
            aero: Default::default(),
            roads: vec![],
        };
        (scene::build_tile(id, &tp, f), tower, pond, rw)
    }

    fn test_world() -> (World, Xform2, Vec2, Vec2, (Vec2, Vec2)) {
        let mut w = World::new(LAT, LON, 0);
        // Wind with gusts and turbulence, so the air is position-dependent.
        w.set_wx(weather::Wx::from_metar_str("EGLC 121150Z 24018G30KT 9999 SCT030 15/08 Q1015").unwrap());
        let (b, tower, pond, rw) = test_tile(TileId::at_world(&w.proj, vec2(1200.0, -800.0)));
        let xf = Proj::tile_frame(b.id).map_to(&w.proj);
        w.integrate(b);
        (w, xf, tower, pond, rw)
    }

    #[test]
    fn frame_maps_are_exact() {
        let a = Proj::new(51.5, 0.03);
        let b = Proj::new(49.2, 2.1);
        let m = a.map_to(&b);
        for (lat, lon) in [(51.51, 0.02), (50.0, 1.0), (48.86, 2.35), (51.3, -0.4)] {
            let (ax, az) = a.to_world64(lat, lon);
            let (bx, bz) = b.to_world64(lat, lon);
            assert!((m.sx * ax + m.tx - bx).abs() < 1e-6 && (az + m.tz - bz).abs() < 1e-6);
        }
        // Tile frames compose: tile → home and tile → there → home agree.
        let t = Proj::tile_frame(TileId::new(8193, 5447));
        let (d, e) = (t.map_to(&a), t.map_to(&b));
        let p = vec2(700.0, 400.0);
        assert!(b.map_to(&a).apply(e.apply(p)).distance(d.apply(p)) < 0.01);
    }

    #[test]
    fn rebase_keeps_environment_and_runways() {
        let (mut w, xf, tower, pond, rw) = test_world();
        w.tick_weather(37.0, (12.0, 200));
        let (tw, pw) = (xf.apply(tower), xf.apply(pond));
        assert_eq!(w.obstacle_top(tw), Some(60.0));
        assert!(w.is_water(pw) && !w.is_water(tw));
        assert_eq!(w.runways.len(), 1);
        let (ra, rb) = (w.runways[0].a, w.runways[0].b);
        let air: Vec<Vec3> = (0..12).map(|i| vec3(tw.x + 37.0 * i as f32, 40.0 + 90.0 * i as f32, tw.y - 23.0 * i as f32)).collect();
        let before: Vec<(Vec3, f32)> = air.iter().map(|p| (w.wind(*p), w.updraft(*p))).collect();

        // Move the origin 18 km east and 1.9 km north (rescales x), anchored at the tower.
        let (lat, lon) = w.proj.to_latlon(tw + vec2(18_000.0, -1_900.0));
        let r = w.rebase_to(Proj::new(lat, lon), tw);
        assert!((r.map.sx - 1.0).abs() > 1e-5, "the test shift should rescale x");
        let (tw2, pw2) = (r.ground(tw), r.ground(pw));
        assert_eq!(w.obstacle_top(tw2), Some(60.0));
        assert!(w.is_water(pw2) && !w.is_water(tw2));
        assert!(w.obstacle_top(tw2 + vec2(0.0, 40.0)).is_none());
        // Runway: the exact map of the old one (and where the tile itself says it is).
        assert!(w.runways[0].a.distance(r.ground(ra)) < 0.02 && w.runways[0].b.distance(r.ground(rb)) < 0.02);
        let xf2 = Proj::tile_frame(TileId::at_world(&w.proj, tw2)).map_to(&w.proj);
        let ends = [xf2.apply(rw.0), xf2.apply(rw.1)];
        assert!(ends.iter().all(|e| w.runways[0].a.distance(*e) < 0.02 || w.runways[0].b.distance(*e) < 0.02));
        // Air (wind, gusts, turbulence, thermals): unchanged at the shifted points.
        for (p, (wind, up)) in air.iter().zip(&before) {
            let q = r.shift(*p);
            assert!((w.wind(q) - *wind).length() < 0.02, "wind {:?} vs {:?}", w.wind(q), wind);
            assert!((w.updraft(q) - up).abs() < 0.01);
        }
        // Same answers as a world started at the new origin.
        let mut fresh = World::new(lat, lon, 0);
        let (b, ..) = test_tile(TileId::at_world(&fresh.proj, tw2));
        fresh.integrate(b);
        assert!(fresh.runways[0].a.distance(w.runways[0].a) < 0.05 || fresh.runways[0].a.distance(w.runways[0].b) < 0.05);
        assert_eq!(fresh.obstacle_top(tw2), Some(60.0));
    }

    #[test]
    fn rebase_mid_flight_is_invisible_to_physics() {
        let def = flight::catalogue().into_iter().find(|d| d.id == "kestrel").unwrap();
        let (mut w1, xf, tower, ..) = test_world();
        let (mut w2, ..) = test_world();
        let p = xf.apply(tower);
        let mut a1 = Aircraft::airborne(&def, vec3(p.x, 120.0, p.y + 600.0), 0.3, 50.0);
        let mut a2 = a1.clone();
        let ctl = Controls { throttle: 0.8, ..flight::trimmed_controls_at(&def, 50.0, 120.0) };
        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        let mut shift = Vec3::ZERO;
        for i in 0..960 {
            if i == 480 {
                let (lat, lon) = w1.proj.to_latlon(vec2(a1.pos.x, a1.pos.z) + vec2(-15_000.0, 1_800.0));
                let r = w1.rebase_to(Proj::new(lat, lon), vec2(a1.pos.x, a1.pos.z));
                a1.rebase(r.delta);
                shift = r.delta;
            }
            t += dt;
            w1.tick_weather(t, (12.0, 200));
            w2.tick_weather(t, (12.0, 200));
            a1.step(&def, &ctl, &w1, dt);
            a2.step(&def, &ctl, &w2, dt);
        }
        assert!(a1.crashed.is_none() && a2.crashed.is_none());
        let err = (a1.pos - (a2.pos + shift)).length();
        assert!(err < 0.05, "position differs by {err} m after the shift");
        assert!((a1.vel - a2.vel).length() < 0.01);
        assert!(a1.rot.angle_between(a2.rot) < 1e-3);
    }
}
