//! Airport detail (Track A): every airport in the world gets taxiway and
//! apron markings, airside buildings (glass terminals, hangars with doors,
//! control towers), stands with parked aircraft and jet bridges, ground
//! vehicles (parked and driving), signs, windsocks and the airfield lights.
//!
//! Pipeline:
//! 1. **Tile workers** (`scene.rs` → [`tile`]): airside building styles,
//!    concrete aprons, taxiway widths by airport class, and a per-tile
//!    summary of the airport's features ([`tile::AeroTile`]: aprons,
//!    taxiways with refs, gates, service roads, classified buildings).
//! 2. **Layout** ([`layout::build`], background thread): the airport's
//!    tiles + OurAirports runways + optional Overpass data ([`overpass`]) →
//!    markings, stands, aircraft, vehicles, signs, lights, windsocks.
//! 3. **Bake** ([`layout::bake`], background thread): the layout placed on
//!    each tile's terrain (markings draped exactly like the tile's layers).
//! 4. **Render** ([`gfx`]): static meshes per tile, instanced parked
//!    aircraft (textured / clustered LODs, [`fleet`]) and vehicles, the
//!    animated windsocks.
//!
//! Sites, layouts and bakes live here ([`AirportScene`], owned by the
//! world); everything is kept in airport / tile frames (lat/lon-anchored),
//! so the floating origin never touches it.

pub mod fleet;
pub mod font;
pub mod gfx;
pub mod layout;
pub mod mesh;
pub mod overpass;
pub mod tile;
pub mod types;

use crate::osm::{Proj, TileId};
use glam::{Mat4, Vec2, Vec3, vec2};
use layout::{Bake, Layout};
use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::{Arc, mpsc};
use tile::AeroTile;
use web_time::{Duration, Instant};

/// Results from the background jobs.
enum Msg {
    Layout(String, u64, Arc<Layout>),
    Bake(String, u64, TileId, Arc<Bake>),
    Osm(String, Option<Arc<overpass::OsmAirport>>),
}

enum Osm {
    Fetching(Instant),
    Done(Option<Arc<overpass::OsmAirport>>),
}

struct Site {
    airport: Arc<crate::airports::Airport>,
    tiles: BTreeSet<TileId>,
    /// Layout generation submitted last / of the current layout (stale results are dropped).
    generation: u64,
    layout_gen: u64,
    layout: Option<Arc<Layout>>,
    building: bool,
    dirty: bool,
    last_build: Option<Instant>,
    osm: Osm,
    /// Bakes and the layout generation they were made from.
    bakes: HashMap<TileId, (u64, Arc<Bake>)>,
    baking: HashSet<TileId>,
}

/// A job for the background worker.
enum Job {
    Layout(Arc<crate::airports::Airport>, Vec<(TileId, Arc<AeroTile>)>, Option<Arc<overpass::OsmAirport>>, u64),
    Bake(String, u64, Arc<Layout>, TileId, Arc<crate::terrain::Grid>),
}

fn run(job: Job, tx: &mpsc::Sender<Msg>) {
    match job {
        Job::Layout(ap, tiles, osm, generation) => {
            let t0 = Instant::now();
            let l = layout::build(&ap, &tiles, osm.as_deref());
            if std::env::var_os("SKYWARD_AIRPORT_DEBUG").is_some() {
                eprintln!(
                    "airport {}: layout from {} tiles{} in {:.1} ms: {} stands, {} aircraft, {} vehicles, {} routes, {} objects, {} lights, {} holds, {} socks, {} mark tris",
                    ap.ident,
                    tiles.len(),
                    if osm.is_some() { " + overpass" } else { "" },
                    t0.elapsed().as_secs_f32() * 1000.0,
                    l.stands.len(),
                    l.parked.len(),
                    l.vehicles.len(),
                    l.routes.len(),
                    l.objects.len(),
                    l.lights.len(),
                    l.holds.len(),
                    l.socks.len(),
                    l.marks.idx.len() / 3
                );
            }
            if std::env::var("SKYWARD_AIRPORT_DEBUG").is_ok_and(|v| v == "2") {
                let ll = |p: Vec2| l.frame.to_latlon(p);
                for h in l.holds.iter().take(6) {
                    eprintln!("  hold {} / {} at {:?} dir {:?}", h.taxiway, h.runway, ll(h.at), h.dir);
                }
                for s in &l.socks {
                    eprintln!("  windsock at {:?}", ll(*s));
                }
                for s in l.stands.iter().filter(|s| s.contact.is_some()).take(12) {
                    eprintln!("  stand {} at {:?} hdg {:.0}", s.name, ll(s.stop), s.hdg.to_degrees());
                }
            }
            let _ = tx.send(Msg::Layout(ap.ident.clone(), generation, Arc::new(l)));
        }
        Job::Bake(ident, generation, l, id, grid) => {
            let b = layout::bake(&l, id, &grid);
            let _ = tx.send(Msg::Bake(ident, generation, id, Arc::new(b)));
        }
    }
}

/// Every airport near the aircraft: layouts, bakes, per-frame instances.
pub struct AirportScene {
    tiles: HashMap<TileId, (Arc<AeroTile>, Arc<crate::terrain::Grid>)>,
    sites: HashMap<String, Site>,
    tx: mpsc::Sender<Msg>,
    rx: mpsc::Receiver<Msg>,
    #[cfg(not(target_arch = "wasm32"))]
    jobs: Option<mpsc::Sender<Job>>,
    in_flight: usize,
    /// Bumped whenever the set of bakes changes (renderer resync).
    pub version: u64,
    /// Bumped whenever the airport lights change.
    pub light_version: u64,
    pub enabled: bool,
}

impl Default for AirportScene {
    fn default() -> Self {
        Self::new()
    }
}

/// A start position picked on a stand (lat / lon of the stop point): the
/// parked aircraft there is left out (startpos.rs sets it at spawn).
static RESERVED: std::sync::Mutex<Option<(f64, f64)>> = std::sync::Mutex::new(None);

/// Keep the stand at (lat, lon) free of parked aircraft (None: no reservation).
pub fn reserve_spot(at: Option<(f64, f64)>) {
    if let Ok(mut g) = RESERVED.lock() {
        *g = at;
    }
}

/// The stand reserved for the player (lat / lon), if any.
pub fn reserved_spot() -> Option<(f64, f64)> {
    RESERVED.lock().ok().and_then(|g| *g)
}

/// Rebuild a layout at most this often while its tiles keep arriving.
const REBUILD_EVERY: Duration = Duration::from_millis(2500);
/// Give up waiting for Overpass (headless shots wait for it) after this.
const OSM_WAIT: Duration = Duration::from_secs(75);

impl AirportScene {
    pub fn new() -> Self {
        let (tx, rx) = mpsc::channel();
        AirportScene {
            tiles: HashMap::new(),
            sites: HashMap::new(),
            tx,
            rx,
            #[cfg(not(target_arch = "wasm32"))]
            jobs: None,
            in_flight: 0,
            version: 0,
            light_version: 0,
            enabled: std::env::var_os("SKYWARD_NO_AIRPORTS").is_none(),
        }
    }

    fn submit(&mut self, job: Job) {
        self.in_flight += 1;
        let tx = self.tx.clone();
        #[cfg(not(target_arch = "wasm32"))]
        {
            if self.jobs.is_none() {
                let (jtx, jrx) = mpsc::channel::<Job>();
                let tx2 = tx.clone();
                let ok = std::thread::Builder::new()
                    .name("airports".into())
                    .spawn(move || {
                        while let Ok(j) = jrx.recv() {
                            run(j, &tx2);
                        }
                    })
                    .is_ok();
                if ok {
                    self.jobs = Some(jtx);
                }
            }
            if let Some(j) = &self.jobs {
                if let Err(mpsc::SendError(job)) = j.send(job) {
                    run(job, &tx);
                }
            } else {
                run(job, &tx);
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            crate::platform::spawn("airport", move || run(job, &tx));
        }
    }

    /// A detailed tile with airport features arrived (tile frame data + its terrain).
    pub fn add_tile(&mut self, id: TileId, t: Arc<AeroTile>, grid: Arc<crate::terrain::Grid>, db: Option<&Arc<crate::airports::Db>>) {
        if !self.enabled {
            return;
        }
        let owners: BTreeSet<String> = t.pieces.iter().filter(|p| !p.owner.is_empty()).map(|p| p.owner.clone()).collect();
        self.tiles.insert(id, (t, grid));
        for o in owners {
            if !self.sites.contains_key(&o) {
                let Some(ap) = db.and_then(|d| d.get(&o)).cloned() else { continue };
                let osm = if overpass::disabled() || overpass::failed_recently(&o) {
                    Osm::Done(None)
                } else if let Some(c) = overpass::cached(&o) {
                    Osm::Done(Some(c))
                } else {
                    self.fetch_osm(&ap);
                    Osm::Fetching(Instant::now())
                };
                self.sites.insert(
                    o.clone(),
                    Site {
                        airport: Arc::new(ap),
                        tiles: BTreeSet::new(),
                        generation: 0,
                        layout_gen: 0,
                        layout: None,
                        building: false,
                        dirty: true,
                        last_build: None,
                        osm,
                        bakes: HashMap::new(),
                        baking: HashSet::new(),
                    },
                );
            }
            let s = self.sites.get_mut(&o).unwrap();
            s.tiles.insert(id);
            s.dirty = true;
            // A re-delivered tile: its old bake is stale.
            if s.bakes.remove(&id).is_some() {
                self.version += 1;
                self.light_version += 1;
            }
        }
    }

    pub fn remove_tile(&mut self, id: TileId) {
        if self.tiles.remove(&id).is_none() {
            return;
        }
        let mut gone = Vec::new();
        for (k, s) in self.sites.iter_mut() {
            if s.tiles.remove(&id) {
                if s.bakes.remove(&id).is_some() {
                    self.version += 1;
                    self.light_version += 1;
                }
                s.baking.remove(&id);
                if s.tiles.is_empty() {
                    gone.push(k.clone());
                }
            }
        }
        for k in gone {
            self.sites.remove(&k);
            self.version += 1;
            self.light_version += 1;
        }
    }

    fn fetch_osm(&self, ap: &crate::airports::Airport) {
        let bbox = osm_bbox(ap);
        let ident = ap.ident.clone();
        let tx = self.tx.clone();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let _ = std::thread::Builder::new().name("overpass".into()).spawn(move || {
                let r = overpass::fetch_blocking(&ident, bbox, OSM_WAIT - Duration::from_secs(5));
                if let Err(e) = &r {
                    eprintln!("airport {ident}: {e} (procedural layout)");
                }
                let _ = tx.send(Msg::Osm(ident, r.ok()));
            });
        }
        #[cfg(target_arch = "wasm32")]
        {
            crate::platform::web::spawn_local(async move {
                let r = overpass::fetch_async(bbox).await;
                if let Err(e) = &r {
                    crate::platform::web::log(&format!("airport {ident}: {e} (procedural layout)"));
                }
                let _ = tx.send(Msg::Osm(ident, r.ok()));
            });
        }
    }

    /// Collect finished jobs, start layouts / bakes. Call once per frame.
    pub fn update(&mut self) {
        while let Ok(m) = self.rx.try_recv() {
            match m {
                Msg::Layout(id, generation, l) => {
                    self.in_flight = self.in_flight.saturating_sub(1);
                    if let Some(s) = self.sites.get_mut(&id) {
                        s.building = false;
                        if s.generation == generation {
                            // Start loading the parked aircraft types it uses.
                            let mut tys: Vec<u16> = l.parked.iter().map(|p| p.model).collect();
                            tys.sort_unstable();
                            tys.dedup();
                            for t in tys {
                                let _ = fleet::get(t);
                            }
                            s.layout = Some(l);
                            s.layout_gen = generation;
                            // Keep the old bakes on screen until the new ones arrive.
                            s.baking.clear();
                        }
                    }
                }
                Msg::Bake(id, generation, tile, b) => {
                    self.in_flight = self.in_flight.saturating_sub(1);
                    if let Some(s) = self.sites.get_mut(&id) {
                        s.baking.remove(&tile);
                        if s.layout_gen == generation && s.tiles.contains(&tile) {
                            s.bakes.insert(tile, (generation, b));
                            self.version += 1;
                            self.light_version += 1;
                        }
                    }
                }
                Msg::Osm(id, o) => {
                    if let Some(s) = self.sites.get_mut(&id) {
                        s.osm = Osm::Done(o.clone());
                        if o.is_some() {
                            s.dirty = true;
                        }
                    }
                }
            }
        }
        let mut jobs = Vec::new();
        for (ident, s) in self.sites.iter_mut() {
            if s.dirty && !s.building && s.last_build.is_none_or(|t| t.elapsed() >= REBUILD_EVERY) {
                s.dirty = false;
                s.building = true;
                s.last_build = Some(Instant::now());
                s.generation += 1;
                let tiles: Vec<(TileId, Arc<AeroTile>)> = s.tiles.iter().filter_map(|id| self.tiles.get(id).map(|t| (*id, t.0.clone()))).collect();
                let osm = match &s.osm {
                    Osm::Done(o) => o.clone(),
                    Osm::Fetching(_) => None,
                };
                jobs.push(Job::Layout(s.airport.clone(), tiles, osm, s.generation));
                continue;
            }
            // Bake the resident tiles the current layout hasn't been placed on.
            if let Some(l) = &s.layout {
                for id in &s.tiles {
                    let stale = s.bakes.get(id).is_none_or(|b| b.0 != s.layout_gen);
                    if stale && !s.baking.contains(id) {
                        if let Some((_, g)) = self.tiles.get(id) {
                            s.baking.insert(*id);
                            jobs.push(Job::Bake(ident.clone(), s.layout_gen, l.clone(), *id, g.clone()));
                        }
                    }
                }
            }
        }
        for j in jobs {
            self.submit(j);
        }
    }

    /// Nothing pending: layouts built and baked for every resident tile,
    /// Overpass answered (or given up), parked aircraft models loaded.
    pub fn idle(&self) -> bool {
        if !self.enabled {
            return true;
        }
        self.in_flight == 0
            && fleet::settled()
            && self.sites.values().all(|s| {
                !s.dirty
                    && !s.building
                    && s.baking.is_empty()
                    && s.layout.is_some()
                    && s.tiles.iter().all(|t| s.bakes.get(t).is_some_and(|b| b.0 == s.layout_gen))
                    && !matches!(s.osm, Osm::Fetching(t) if t.elapsed() < OSM_WAIT)
            })
    }

    /// Every bake (for the renderer): (site, tile, bake).
    pub fn bakes(&self) -> impl Iterator<Item = (&str, &Arc<Bake>)> {
        self.sites.iter().flat_map(|(k, s)| s.bakes.values().map(move |b| (k.as_str(), &b.1)))
    }

    /// Airport lights (taxiway, stop bars, floods, windsocks) in the current frame.
    pub fn lights(&self, proj: &Proj) -> Vec<crate::lights::LightInst> {
        let mut out = Vec::new();
        for (_, b) in self.bakes() {
            let m = Proj::tile_frame(b.id).map_to(proj);
            for l in &b.lights {
                let mut l = *l;
                let p = m.apply(vec2(l.pos[0], l.pos[2]));
                l.pos = [p.x, l.pos[1], p.y];
                out.push(l);
            }
        }
        out
    }

    /// This frame's instances (eye-relative): parked aircraft by LOD,
    /// vehicles (parked and driving), windsocks. `wind(p)`: the wind at a
    /// current-frame point; `ground(xz)`: terrain height.
    pub fn frame(&self, proj: &Proj, eye: Vec3, time: f32, wind: &dyn Fn(Vec3) -> Vec3, ground: &dyn Fn(Vec2) -> f32) -> Frame {
        let mut f = Frame::default();
        let (ex, ey, ez) = (eye.x as f64, eye.y as f64, eye.z as f64);
        let rel = |m: &crate::osm::Xform2, p: Vec3| -> Vec3 { Vec3::new((m.sx * p.x as f64 + m.tx - ex) as f32, (p.y as f64 - ey) as f32, (p.z as f64 + m.tz - ez) as f32) };
        // The parked aircraft nearest a reserved stand (the player's) stays out.
        let reserved = reserved_spot().map(|(la, lo)| proj.to_world(la, lo));
        let skip = reserved.and_then(|q| {
            let mut best: Option<(f32, usize, usize)> = None;
            for (bi, (_, b)) in self.bakes().enumerate() {
                let m = Proj::tile_frame(b.id).map_to(proj);
                for (pi, &(_, p, _, _)) in b.parked.iter().enumerate() {
                    let w = m.apply(vec2(p.x, p.z));
                    let d = w.distance(q);
                    if d < 45.0 && best.is_none_or(|bb| d < bb.0) {
                        best = Some((d, bi, pi));
                    }
                }
            }
            best.map(|b| (b.1, b.2))
        });
        for (bi, (_, b)) in self.bakes().enumerate() {
            let m = Proj::tile_frame(b.id).map_to(proj);
            for (pi, &(ty, p, hdg, livery)) in b.parked.iter().enumerate() {
                if skip == Some((bi, pi)) {
                    continue;
                }
                let r = rel(&m, p);
                let d = r.length();
                if d > 12_000.0 {
                    continue;
                }
                let lod = if d < 260.0 {
                    0
                } else if d < 2600.0 {
                    1
                } else {
                    2
                };
                f.parked.push(Inst { model: ty, lod, xf: mesh::place(r, hdg), livery });
            }
            for &(k, p, hdg, livery) in &b.vehicles {
                let r = rel(&m, p);
                if r.length() < 2800.0 {
                    f.vehicles.push(Inst { model: k, lod: 0, xf: mesh::place(r, hdg), livery });
                }
            }
            for (i, s) in b.socks.iter().enumerate() {
                let r = rel(&m, *s);
                if r.length() > 6000.0 {
                    continue;
                }
                let wp = r + eye;
                let w = wind(wp);
                let wxz = vec2(w.x, w.z);
                let kt = wxz.length() * 1.944;
                let toward = if kt > 0.5 { wxz.normalize() } else { vec2(0.3, 0.95).normalize() };
                // A sock stands straight out at ~15 kt.
                let fill = (kt / 15.0).clamp(0.05, 1.0);
                mesh::windsock_sock(&mut f.dynamic, r, toward, fill, time, i as f32 * 0.37);
            }
        }
        // Vehicles on the service roads.
        for s in self.sites.values() {
            let Some(l) = &s.layout else { continue };
            if s.bakes.is_empty() {
                continue;
            }
            let m = l.frame.map_to(proj);
            for rt in &l.routes {
                let c = m.apply(rt.pts[0]);
                if (vec2(c.x, c.y) - vec2(eye.x, eye.z)).length() > 3500.0 + rt.len {
                    continue;
                }
                for &(k, phase, speed) in &rt.vehicles {
                    let (p, hdg) = rt.at(phase, speed, time);
                    let w = m.apply(p);
                    let r = Vec3::new(w.x - eye.x, ground(w) - eye.y, w.y - eye.z);
                    if r.length() < 2800.0 {
                        f.vehicles.push(Inst { model: k, lod: 0, xf: mesh::place(r, hdg), livery: (phase as u16) & 0xff });
                    }
                }
            }
        }
        f
    }

    /// The current layout of an airport (ATC taxi routing), if built.
    pub fn layout(&self, ident: &str) -> Option<Arc<Layout>> {
        self.sites.get(ident).and_then(|s| s.layout.clone())
    }

    /// Every airport layout built so far (live-traffic taxiway snapping,
    /// `adsb/`; the tyres' pavement map, `ground/`).
    pub fn layouts(&self) -> Vec<Arc<Layout>> {
        self.sites.values().filter_map(|s| s.layout.clone()).collect()
    }

    /// Stands of the airports in range (startpos / debugging): (airport, name, stop point, heading) in the current frame.
    pub fn stands(&self, proj: &Proj) -> Vec<(String, String, Vec2, f32)> {
        let mut out = Vec::new();
        for (k, s) in &self.sites {
            if let Some(l) = &s.layout {
                let m = l.frame.map_to(proj);
                out.extend(l.stands.iter().map(|st| (k.clone(), st.name.clone(), m.apply(st.stop), st.hdg)));
            }
        }
        out
    }
}

/// The Overpass query box of an airport (south, west, north, east): its
/// runways plus an apron margin by class.
pub fn osm_bbox(ap: &crate::airports::Airport) -> (f64, f64, f64, f64) {
    let (mut s, mut w, mut n, mut e) = (ap.lat, ap.lon, ap.lat, ap.lon);
    for r in &ap.runways {
        for end in &r.ends {
            if let (Some(la), Some(lo)) = (end.lat, end.lon) {
                s = s.min(la);
                n = n.max(la);
                w = w.min(lo);
                e = e.max(lo);
            }
        }
    }
    let m = match ap.kind {
        crate::airports::Kind::Large => 1800.0,
        crate::airports::Kind::Medium => 1000.0,
        _ => 500.0,
    };
    let dlat = m / crate::osm::M_PER_DEG_LAT;
    let dlon = dlat / ap.lat.to_radians().cos().max(0.1);
    (s - dlat, w - dlon, n + dlat, e + dlon)
}

/// Hide the parked aircraft standing within 25 m of `at` (world points: stands
/// taken by live traffic, `adsb/`). `eye`: the frame's eye (instances are eye-relative).
pub fn hide_parked(f: &mut Frame, at: &[Vec2], eye: Vec3) {
    if at.is_empty() {
        return;
    }
    f.parked.retain(|p| {
        let w = p.xf.w_axis;
        let xz = Vec2::new(w.x + eye.x, w.z + eye.z);
        !at.iter().any(|h| h.distance(xz) < 25.0)
    });
}

/// One instance this frame (eye-relative placement, body → world).
#[derive(Clone, Copy, Debug)]
pub struct Inst {
    pub model: u16,
    pub lod: u8,
    pub xf: Mat4,
    pub livery: u16,
}

#[derive(Default)]
pub struct Frame {
    pub parked: Vec<Inst>,
    pub vehicles: Vec<Inst>,
    /// Windsocks (eye-relative, scene vertex format).
    pub dynamic: crate::scene::MeshData,
}
