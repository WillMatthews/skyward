//! Road traffic (Track RT): cars, vans, buses, lorries and motorbikes driving
//! the real road network, by day and by night.
//!
//! * [`net`]: the lane graph of each map tile (built in the tile workers
//!   from the OpenMapTiles `transportation` layer: lanes, junction
//!   connectors, conflicts, portals to the neighbouring tiles).
//! * [`sim`]: the microscopic simulation (IDM car following, junctions,
//!   sources / sinks) around the eye, deterministic per tile seed.
//! * [`Traffic`]: runs the simulation on a worker thread (natively; inline
//!   in the browser and for headless captures) and hands the renderer one
//!   instance snapshot per frame; the GPU extrapolates the last few ms.
//! * [`gpu`] / [`mesh`]: instanced vehicle meshes (detailed near, boxes
//!   further out, light points only beyond), lit by the scene shader; the
//!   head / tail / brake lights are light sprites (lights.wgsl).
//! * [`Level`]: the Settings choice Off / Low / Medium / High (Off costs
//!   nothing: no simulation, no draws).

pub mod gpu;
pub mod mesh;
pub mod net;
pub mod sim;
pub mod vehicle;

#[cfg(test)]
mod tests;

use crate::osm::{Proj, TileId};
use glam::Vec3;
use std::sync::Arc;

/// Traffic density setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Level {
    Off,
    Low,
    #[default]
    Medium,
    High,
}

impl Level {
    pub const ALL: [Level; 4] = [Level::Off, Level::Low, Level::Medium, Level::High];
    pub const NAMES: [&'static str; 4] = ["Off", "Low", "Medium", "High"];
    pub fn from_u8(v: u8) -> Level {
        Level::ALL[(v as usize).min(3)]
    }
    pub fn as_u8(self) -> u8 {
        self as u8
    }
    pub fn parse(s: &str) -> Option<Level> {
        Level::NAMES.iter().position(|n| n.eq_ignore_ascii_case(s)).map(|i| Level::ALL[i])
    }
    /// Default: Medium natively, Low in the browser.
    pub fn default_for_platform() -> Level {
        if crate::platform::WEB { Level::Low } else { Level::Medium }
    }
    /// Simulated radius around the eye (m).
    pub fn radius(self) -> f32 {
        match self {
            Level::Off => 0.0,
            Level::Low => 1300.0,
            Level::Medium => 2200.0,
            Level::High => 3200.0,
        }
    }
    /// Share of the full traffic density.
    pub fn density(self) -> f32 {
        match self {
            Level::Off => 0.0,
            Level::Low => 0.4,
            Level::Medium => 0.7,
            Level::High => 1.0,
        }
    }
    /// Vehicles at most.
    pub fn cap(self) -> usize {
        match self {
            Level::Off => 0,
            Level::Low => 2500,
            Level::Medium => 8000,
            Level::High => 18000,
        }
    }
    /// Vehicle meshes are drawn to this distance (m); lights to the radius.
    pub fn draw_dist(self) -> f32 {
        match self {
            Level::Off => 0.0,
            Level::Low => 1200.0,
            Level::Medium => 2000.0,
            Level::High => 2800.0,
        }
    }
}

/// One vehicle on the GPU (32 bytes): position relative to the eye (this
/// frame, extrapolated), heading (unit x, z), road grade (dy / ds), body
/// colour (sRGB rgba8), kind | flags << 8 | seed << 16.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VehInst {
    pub pos: [f32; 3],
    pub dx: f32,
    pub dz: f32,
    pub grade: f32,
    pub col: u32,
    pub info: u32,
}

/// Instance flags (`info` bits 8..16).
pub const BRAKE: u32 = 1;

/// How the snapshot is cut for the view.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub eye: Vec3,
    pub fwd: Vec3,
    /// Vehicles with dot(dir, fwd) below this (and further than 60 m) are skipped.
    pub cos_half: f32,
    pub lod0_dist: f32,
    pub draw_dist: f32,
    /// Light points only out to here (0 by day).
    pub light_dist: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub vehicles: u32,
    pub drawn: u32,
    pub active_lanes: u32,
    /// Simulation + snapshot time of the last update (ms, on whichever thread ran it).
    pub sim_ms: f32,
}

/// Instances for one frame, grouped (kind, lod, first, count): lod 0
/// detailed, 1 simple (to 3x the lod 0 distance), 2 boxes, 3 lights only. Positions are in the current frame (world) at `time`;
/// `speed` (m/s, per instance) extrapolates them to the frame's time.
#[derive(Clone, Debug, Default)]
pub struct Snapshot {
    pub epoch: u32,
    pub time: f64,
    pub inst: Vec<VehInst>,
    pub speed: Vec<f32>,
    pub ranges: Vec<(u8, u8, u32, u32)>,
    pub stats: Stats,
}

/// Per-frame input.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub time: f64,
    pub hours: f32,
    pub view: View,
}

enum Msg {
    Add(Arc<net::RoadNet>),
    Remove(TileId),
    Rebase(Proj),
    Level(Level),
    Reset(Frame),
    Step(Frame, u32),
    Recycle(Snapshot),
}

struct Worker {
    tx: std::sync::mpsc::Sender<Msg>,
    rx: std::sync::mpsc::Receiver<Snapshot>,
}

/// The traffic system: owns the simulation (inline or on a worker thread).
pub struct Traffic {
    level: Level,
    sim: Option<Box<sim::Sim>>,
    worker: Option<Worker>,
    /// Run in the caller's thread (browser, headless captures).
    inline: bool,
    /// Origin-shift generation (snapshots from an older frame are dropped).
    epoch: u32,
    snap: Snapshot,
    have: bool,
    /// Main-thread time of the last `frame` call (ms).
    pub main_ms: f32,
    /// Refill and settle at the next frame.
    reset_pending: bool,
}

fn handle(sim: &mut sim::Sim, m: Msg) {
    match m {
        Msg::Add(n) => sim.add_tile(n),
        Msg::Remove(id) => sim.remove_tile(id),
        Msg::Rebase(p) => sim.rebase(p),
        Msg::Level(l) => sim.set_level(l),
        Msg::Reset(f) => sim.reset(f.time, 20.0, f.view.eye, f.hours),
        Msg::Step(..) | Msg::Recycle(_) => {}
    }
}

impl Traffic {
    pub fn new(proj: Proj) -> Traffic {
        let level = Level::default_for_platform();
        let mut sim = Box::new(sim::Sim::new(proj));
        sim.set_level(level);
        Traffic {
            level,
            sim: Some(sim),
            worker: None,
            inline: crate::platform::WEB || std::env::var_os("SKYWARD_TRAFFIC_INLINE").is_some(),
            epoch: 0,
            snap: Snapshot::default(),
            have: false,
            main_ms: 0.0,
            reset_pending: false,
        }
    }

    pub fn level(&self) -> Level {
        self.level
    }

    fn send(&mut self, m: Msg) {
        if let Some(w) = &self.worker {
            let _ = w.tx.send(m);
        } else if let Some(s) = self.sim.as_mut() {
            handle(s, m);
        }
    }

    pub fn set_level(&mut self, l: Level) {
        // (SKYWARD_NO_TRAFFIC=1 keeps it off: golden captures.)
        let l = if std::env::var_os("SKYWARD_NO_TRAFFIC").is_some() { Level::Off } else { l };
        if l != self.level {
            self.level = l;
            self.have = false;
            self.send(Msg::Level(l));
        }
    }

    /// Run the simulation in the caller's thread (deterministic captures).
    pub fn set_inline(&mut self) {
        self.inline = true;
    }

    pub fn add_tile(&mut self, n: Arc<net::RoadNet>) {
        self.send(Msg::Add(n));
    }

    pub fn remove_tile(&mut self, id: TileId) {
        self.send(Msg::Remove(id));
    }

    pub fn rebase(&mut self, proj: Proj) {
        self.epoch = self.epoch.wrapping_add(1);
        self.have = false;
        self.send(Msg::Rebase(proj));
    }

    /// Refill every lane and let the traffic settle up to `time` (headless
    /// captures: the picture then doesn't depend on the streaming order).
    pub fn reset(&mut self) {
        self.have = false;
        self.reset_pending = true;
    }

    fn start_worker(&mut self) {
        let Some(mut sim) = self.sim.take() else { return };
        let (tx, jobs) = std::sync::mpsc::channel::<Msg>();
        let (done, rx) = std::sync::mpsc::channel::<Snapshot>();
        let ok = std::thread::Builder::new().name("traffic".into()).spawn(move || {
            let mut spare: Vec<Snapshot> = Vec::new();
            while let Ok(m) = jobs.recv() {
                let mut step = None;
                let take = |m: Msg, sim: &mut sim::Sim, step: &mut Option<(Frame, u32)>, spare: &mut Vec<Snapshot>| match m {
                    Msg::Step(f, g) => *step = Some((f, g)),
                    Msg::Recycle(s) => spare.push(s),
                    m => handle(sim, m),
                };
                take(m, &mut sim, &mut step, &mut spare);
                while let Ok(m) = jobs.try_recv() {
                    take(m, &mut sim, &mut step, &mut spare);
                }
                if let Some((f, g)) = step {
                    let t0 = web_time::Instant::now();
                    sim.advance(f.time, f.view.eye, f.hours);
                    let mut out = spare.pop().unwrap_or_default();
                    sim.snapshot(f.time, &f.view, &mut out);
                    out.epoch = g;
                    out.stats.sim_ms = t0.elapsed().as_secs_f32() * 1000.0;
                    if done.send(out).is_err() {
                        break;
                    }
                }
            }
        });
        match ok {
            Ok(_) => self.worker = Some(Worker { tx, rx }),
            Err(_) => {
                // No thread: stay inline (the closure, and the sim, are gone: start over).
                self.inline = true;
            }
        }
    }

    /// This frame's vehicles (None when off / nothing yet).
    pub fn frame(&mut self, f: Frame) -> Option<&Snapshot> {
        if self.level == Level::Off {
            return None;
        }
        let t0 = web_time::Instant::now();
        if std::mem::take(&mut self.reset_pending) {
            self.send(Msg::Reset(f));
        }
        if self.inline || cfg!(target_arch = "wasm32") {
            if self.worker.is_none() {
                if let Some(sim) = self.sim.as_mut() {
                    sim.advance(f.time, f.view.eye, f.hours);
                    sim.snapshot(f.time, &f.view, &mut self.snap);
                    self.snap.epoch = self.epoch;
                    self.snap.stats.sim_ms = t0.elapsed().as_secs_f32() * 1000.0;
                    self.have = true;
                }
            }
        } else {
            if self.worker.is_none() {
                self.start_worker();
            }
            if let Some(w) = &self.worker {
                let _ = w.tx.send(Msg::Step(f, self.epoch));
                let mut newest: Option<Snapshot> = None;
                while let Ok(s) = w.rx.try_recv() {
                    if let Some(old) = newest.take() {
                        let _ = w.tx.send(Msg::Recycle(old));
                    }
                    newest = Some(s);
                }
                if let Some(s) = newest {
                    if s.epoch == self.epoch {
                        let old = std::mem::replace(&mut self.snap, s);
                        let _ = w.tx.send(Msg::Recycle(old));
                        self.have = true;
                    } else {
                        let _ = w.tx.send(Msg::Recycle(s));
                    }
                }
            }
        }
        self.main_ms = t0.elapsed().as_secs_f32() * 1000.0;
        self.have.then_some(&self.snap)
    }

    /// Statistics of the latest snapshot.
    pub fn stats(&self) -> Stats {
        if self.have { self.snap.stats } else { Stats::default() }
    }
}
