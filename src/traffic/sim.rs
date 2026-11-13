//! Microscopic traffic simulation over the tiles' lane graphs.
//!
//! * Only lanes within the level's radius of the eye are *active*; a lane
//!   that becomes active is filled at its equilibrium density (seeded by the
//!   tile, the lane and the time, so it is reproducible), one that drops out
//!   is emptied. Lanes nobody can drive into from an active lane are
//!   *sources* (vehicles arrive at the traffic flow rate); dead ends and the
//!   edge of the active area are sinks.
//! * Car following is the Intelligent Driver Model per vehicle type, with
//!   anticipation of the curve speed of the next connector, a hard no-overlap
//!   constraint and stop lines at junctions: a vehicle may enter a connector
//!   only while no conflicting connector is in use and no approaching
//!   vehicle with right of way is due first (major road, straight on, then
//!   whoever is nearer; long waits raise a vehicle's priority).
//! * Fixed 0.1 s steps; between steps the snapshot extrapolates along the
//!   lane (positions follow the curves exactly).
//! * Traffic crosses tile edges through the portals (matched by position and
//!   direction to the neighbour's lanes).

use super::net::{RoadNet, class, flag};
use super::vehicle::{self, Spec};
use super::{Level, Snapshot, Stats, VehInst, View};
use crate::osm::{Proj, TileId, Xform2};
use glam::{Vec2, Vec3, vec2};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

/// Simulation step (s).
pub const DT: f64 = 0.1;
/// Steps per call at most (a long stall skips time instead of catching up).
const MAX_STEPS: usize = 8;
/// Gap kept between vehicles in any case (m).
const MIN_GAP: f32 = 0.4;
/// How far ahead a front vehicle looks for a leader across lanes (m).
const LOOKAHEAD: f32 = 160.0;
/// Lanes further than this from the eye are held near their target density (m).
const REGULATE_DIST: f32 = 900.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Link {
    slot: u16,
    lane: u32,
}

#[derive(Clone, Default)]
struct LaneRt {
    /// Vehicles on the lane, front (largest s) first.
    q: VecDeque<u32>,
    active: bool,
    source: bool,
    /// Next arrival time at a source.
    spawn_t: f64,
    /// Connectors: the soonest approaching vehicle this step (eta s, priority).
    claim_step: u32,
    claim_eta: f32,
    claim_prio: f32,
    claim_veh: u32,
    /// Times this lane was filled / fed (seeds).
    n: u32,
    /// Step stamp: listed in this step's occupied lanes.
    mark: u32,
}

struct TileRt {
    net: Arc<RoadNet>,
    /// Tile frame → current frame.
    xf: Xform2,
    lanes: Vec<LaneRt>,
    active: Vec<u32>,
    /// Edge portals: next lane in the neighbour (resolved lazily).
    edge_next: HashMap<u32, Option<Link>>,
    edge_prev: HashMap<u32, Option<Link>>,
    seed: u64,
}

#[derive(Clone, Copy, Debug)]
struct Veh {
    slot: u16,
    kind: u8,
    brake: bool,
    lane: u32,
    next: Option<Link>,
    s: f32,
    v: f32,
    a: f32,
    /// Desired-speed factor of the driver.
    v0f: f32,
    col: u32,
    seed: u32,
    /// Time stopped at a junction (s).
    wait: f32,
    /// Time standing still (s): the head of a gridlock is removed.
    stuck: f32,
    hint: u32,
    hops: u32,
    alive: bool,
}

pub struct Sim {
    slots: Vec<Option<TileRt>>,
    /// Occupied slots in tile order (the update order: reproducible
    /// whatever order the tiles streamed in).
    order: Vec<usize>,
    by_id: HashMap<TileId, u16>,
    vehs: Vec<Veh>,
    free: Vec<u32>,
    alive: usize,
    proj: Proj,
    /// Time of the last step (s).
    pub t: f64,
    started: bool,
    pub level: Level,
    pub hours: f32,
    eye: Vec3,
    /// Active lanes with vehicles (the update loops visit only these) and
    /// the active sources, rebuilt at each activation.
    occupied: Vec<Link>,
    sources: Vec<Link>,
    act_t: f64,
    act_eye: Vec2,
    stamp: u32,
    /// Per (kind, lod) instance buckets (instance, speed), reused.
    buckets: Vec<Vec<(VehInst, f32)>>,
    pub stats: Stats,
}

fn hash(x: u64) -> u64 {
    crate::scene::hash(x)
}

fn h01(x: u64) -> f32 {
    (hash(x) >> 40) as f32 / (1u64 << 24) as f32
}

/// IDM acceleration toward a leader `gap` m ahead closing at `dv` m/s.
pub fn idm(sp: &Spec, v: f32, v0: f32, gap: f32, dv: f32) -> f32 {
    let free = 1.0 - (v / v0.max(0.1)).powi(4);
    let s_star = sp.s0 + (v * sp.t + v * dv / (2.0 * (sp.a * sp.b).sqrt())).max(0.0);
    let g = gap.max(0.1);
    (sp.a * (free - (s_star / g).powi(2))).clamp(-9.0, sp.a)
}

impl Sim {
    pub fn new(proj: Proj) -> Sim {
        Sim {
            slots: Vec::new(),
            order: Vec::new(),
            by_id: HashMap::new(),
            vehs: Vec::new(),
            free: Vec::new(),
            alive: 0,
            proj,
            t: 0.0,
            started: false,
            level: Level::Medium,
            hours: 12.0,
            eye: Vec3::ZERO,
            occupied: Vec::new(),
            sources: Vec::new(),
            act_t: f64::MIN,
            act_eye: Vec2::splat(f32::MAX),
            stamp: 0,
            buckets: Vec::new(),
            stats: Stats::default(),
        }
    }

    pub fn vehicle_count(&self) -> usize {
        self.alive
    }

    pub fn add_tile(&mut self, net: Arc<RoadNet>) {
        let id = net.id;
        self.remove_tile(id);
        let xf = Proj::tile_frame(id).map_to(&self.proj);
        let n = net.lanes.len();
        let rt =
            TileRt { net, xf, lanes: vec![LaneRt::default(); n], active: Vec::new(), edge_next: HashMap::new(), edge_prev: HashMap::new(), seed: hash(id.seed() ^ 0x007a_111c) };
        let slot = match self.slots.iter().position(|s| s.is_none()) {
            Some(i) => i,
            None => {
                self.slots.push(None);
                self.slots.len() - 1
            }
        };
        self.slots[slot] = Some(rt);
        self.by_id.insert(id, slot as u16);
        self.relink();
    }

    pub fn remove_tile(&mut self, id: TileId) {
        let Some(slot) = self.by_id.remove(&id) else { return };
        if let Some(t) = self.slots[slot as usize].take() {
            for l in &t.lanes {
                for &v in &l.q {
                    self.vehs[v as usize].alive = false;
                    self.free.push(v);
                    self.alive -= 1;
                }
            }
        }
        // Vehicles elsewhere heading into it: they now stop at their lane's end.
        for v in self.vehs.iter_mut().filter(|v| v.alive) {
            if v.next.is_some_and(|n| n.slot == slot) {
                v.next = None;
            }
        }
        self.relink();
    }

    /// Edge links changed: re-resolve lazily.
    fn relink(&mut self) {
        let mut order: Vec<(TileId, usize)> = self.by_id.iter().map(|(id, &s)| (*id, s as usize)).collect();
        order.sort();
        self.order = order.into_iter().map(|x| x.1).collect();
        // (Lists may name a freed / reused slot: rebuilt at the next activation.)
        self.occupied.clear();
        self.sources.clear();
        for t in self.slots.iter_mut().flatten() {
            t.edge_next.clear();
            t.edge_prev.clear();
        }
        self.act_t = f64::MIN;
    }

    pub fn rebase(&mut self, proj: Proj) {
        self.proj = proj;
        for (id, &slot) in &self.by_id {
            if let Some(t) = self.slots[slot as usize].as_mut() {
                t.xf = Proj::tile_frame(*id).map_to(&proj);
            }
        }
        self.act_t = f64::MIN;
    }

    /// Drop every vehicle (they are refilled at the next step).
    pub fn clear(&mut self) {
        for t in self.slots.iter_mut().flatten() {
            for l in &mut t.lanes {
                l.q.clear();
                l.active = false;
                l.source = false;
            }
            t.active.clear();
        }
        self.occupied.clear();
        self.sources.clear();
        self.vehs.clear();
        self.free.clear();
        self.alive = 0;
        self.act_t = f64::MIN;
    }

    /// Restart at time `t`: refill and settle for `settle` seconds before it.
    pub fn reset(&mut self, t: f64, settle: f64, eye: Vec3, hours: f32) {
        self.clear();
        self.eye = eye;
        self.hours = hours;
        self.t = t - settle;
        self.started = true;
        let steps = (settle / DT).round() as usize;
        for _ in 0..steps {
            self.step();
        }
        self.t = t;
    }

    pub fn set_level(&mut self, l: Level) {
        if l != self.level {
            self.level = l;
            self.clear();
        }
    }

    /// Advance to time `t` (s) around `eye` (current frame).
    pub fn advance(&mut self, t: f64, eye: Vec3, hours: f32) {
        self.eye = eye;
        self.hours = hours;
        if self.level == Level::Off {
            self.t = t;
            return;
        }
        if !self.started || t < self.t - 1.0 {
            self.started = true;
            self.t = t;
        }
        let mut n = 0;
        while self.t + DT <= t {
            if n == MAX_STEPS {
                self.t = t;
                break;
            }
            self.step();
            n += 1;
        }
        if self.act_t == f64::MIN {
            self.activate();
        }
    }

    fn time_factor(&self) -> f32 {
        crate::lights::sky::traffic_level(self.hours)
    }

    // ------------------------------------------------------------ links

    fn lane(&self, l: Link) -> &super::net::Lane {
        &self.slots[l.slot as usize].as_ref().unwrap().net.lanes[l.lane as usize]
    }

    fn rt(&self, l: Link) -> &LaneRt {
        &self.slots[l.slot as usize].as_ref().unwrap().lanes[l.lane as usize]
    }

    /// The neighbour's lane continuing an edge lane (`out`) or feeding it.
    fn resolve_edge(&mut self, slot: u16, lane: u32, out: bool) -> Option<Link> {
        let t = self.slots[slot as usize].as_ref()?;
        let cache = if out { &t.edge_next } else { &t.edge_prev };
        if let Some(r) = cache.get(&lane) {
            return *r;
        }
        let (p, d) = if out { t.net.end(lane) } else { t.net.start(lane) };
        let (wp, wd) = (t.xf.apply(p), vec2(d.x * t.xf.sx as f32, d.y).normalize_or(d));
        // The neighbour is across the edge: a step along (out) or back (in).
        let probe = wp + wd * if out { 1.0 } else { -1.0 };
        let id = TileId::at_world(&self.proj, probe);
        let mut found = None;
        if let Some(&s2) = self.by_id.get(&id) {
            if s2 != slot {
                let t2 = self.slots[s2 as usize].as_ref().unwrap();
                let lp = t2.xf.inverse(wp);
                let list = if out { &t2.net.edge_in } else { &t2.net.edge_out };
                let mut best = 4.0f32;
                for &l2 in list {
                    let (q, e) = if out { t2.net.start(l2) } else { t2.net.end(l2) };
                    let dist = q.distance(lp);
                    if dist < best && e.dot(d) > 0.7 {
                        best = dist;
                        found = Some(Link { slot: s2, lane: l2 });
                    }
                }
            }
        }
        let t = self.slots[slot as usize].as_mut().unwrap();
        if out {
            t.edge_next.insert(lane, found);
        } else {
            t.edge_prev.insert(lane, found);
        }
        found
    }

    /// Where a vehicle on `l` goes next (weighted pick, deterministic).
    fn choose_next(&mut self, l: Link, seed: u32, hops: u32) -> Option<Link> {
        let t = self.slots[l.slot as usize].as_ref()?;
        let succ = t.net.succs(l.lane);
        if succ.is_empty() {
            if t.net.lanes[l.lane as usize].has(flag::EDGE_OUT) {
                return self.resolve_edge(l.slot, l.lane, true);
            }
            return None;
        }
        if succ.len() == 1 {
            return Some(Link { slot: l.slot, lane: succ[0].lane });
        }
        let total: f32 = succ.iter().map(|s| s.weight).sum();
        let mut x = h01((seed as u64) << 24 ^ (hops as u64) << 8 ^ l.lane as u64 ^ t.seed) * total;
        for s in succ {
            if x < s.weight {
                return Some(Link { slot: l.slot, lane: s.lane });
            }
            x -= s.weight;
        }
        Some(Link { slot: l.slot, lane: succ[succ.len() - 1].lane })
    }

    // ------------------------------------------------------------ vehicles

    fn alloc(&mut self, v: Veh) -> u32 {
        self.alive += 1;
        if let Some(i) = self.free.pop() {
            self.vehs[i as usize] = v;
            i
        } else {
            self.vehs.push(v);
            (self.vehs.len() - 1) as u32
        }
    }

    fn kill(&mut self, i: u32) {
        self.vehs[i as usize].alive = false;
        self.free.push(i);
        self.alive -= 1;
    }

    /// A new vehicle for lane `l` (kind, colour, driver) from `seed`.
    fn make(&self, l: Link, s: f32, seed: u64) -> Veh {
        let t = self.slots[l.slot as usize].as_ref().unwrap();
        let ln = &t.net.lanes[l.lane as usize];
        let urban = t.net.urban > 0.4;
        let mut k = vehicle::pick(ln.class(), t.net.region, urban, h01(seed ^ 1));
        // Wide vehicles keep off narrow lanes.
        if ln.class() >= class::MINOR && matches!(k, vehicle::kind::ARTIC | vehicle::kind::LORRY) {
            k = vehicle::kind::VAN;
        }
        let sp = vehicle::spec(k);
        let v0f = 0.88 + 0.22 * h01(seed ^ 2);
        Veh {
            slot: l.slot,
            kind: k,
            brake: false,
            lane: l.lane,
            next: None,
            s,
            v: (ln.speed * v0f).min(sp.vmax) * 0.85,
            a: 0.0,
            v0f,
            col: vehicle::colour(k, t.net.region, h01(seed ^ 3), h01(seed ^ 4)),
            seed: (hash(seed ^ 5) >> 32) as u32,
            wait: 0.0,
            stuck: 0.0,
            hint: 0,
            hops: 0,
            alive: true,
        }
    }

    fn cap(&self) -> usize {
        self.level.cap()
    }

    /// Fill a lane that just became active at its equilibrium density.
    fn populate(&mut self, l: Link) {
        let (len, k, seed) = {
            let t = self.slots[l.slot as usize].as_ref().unwrap();
            let ln = &t.net.lanes[l.lane as usize];
            if ln.has(flag::CONNECTOR) || ln.density <= 0.0 {
                return;
            }
            let k = ln.density * self.time_factor() * self.level.density();
            (ln.len, k, t.seed ^ (l.lane as u64) << 20 ^ (self.t / 30.0).floor() as u64)
        };
        let want = len * k;
        let n = (want + h01(seed)).floor() as usize;
        if n == 0 || self.alive >= self.cap() {
            return;
        }
        // Even spacing with jitter, front first.
        let mut s_next = f32::MAX;
        let mut vs = Vec::with_capacity(n);
        for j in (0..n).rev() {
            let sj = ((j as f32 + 0.5 + (h01(seed ^ (j as u64 + 7) << 3) - 0.5) * 0.7) / n as f32 * len).clamp(0.0, len - 0.1);
            let mut v = self.make(l, sj, seed ^ (j as u64) << 40 ^ 0x5eed);
            let sp = vehicle::spec(v.kind);
            if s_next < f32::MAX && sj > s_next - sp.s0 - MIN_GAP {
                continue;
            }
            v.next = None;
            s_next = sj - sp.len;
            vs.push(v);
        }
        for v in vs {
            let hops = v.hops;
            let seed = v.seed;
            let i = self.alloc(v);
            let nx = self.choose_next(l, seed, hops);
            self.vehs[i as usize].next = nx;
            self.slots[l.slot as usize].as_mut().unwrap().lanes[l.lane as usize].q.push_back(i);
        }
    }

    /// Lanes within the radius of the eye are active; sources found.
    fn activate(&mut self) {
        let r = self.level.radius();
        let eye2 = vec2(self.eye.x, self.eye.z);
        let mut fill: Vec<Link> = Vec::new();
        let mut kills: Vec<u32> = Vec::new();
        // Far from the eye the lanes are held near their target density
        // (jams there would never clear and the excess is invisible).
        let k_t = self.time_factor() * self.level.density();
        let vehs = &self.vehs;
        for &si in &self.order {
            let Some(t) = self.slots[si].as_mut() else { continue };
            let e = t.xf.inverse(eye2);
            let (lo, hi) = t.net.id.rect(&Proj::tile_frame(t.net.id));
            let td = (lo - e).max(e - hi).max(Vec2::ZERO).length();
            let mut act = Vec::new();
            for (li, ln) in t.net.lanes.iter().enumerate() {
                let rt = &mut t.lanes[li];
                let d = if td > r + 400.0 { f32::MAX } else { (Vec2::from(ln.lo) - e).max(e - Vec2::from(ln.hi)).max(Vec2::ZERO).length() };
                if rt.active {
                    if d > r + 250.0 {
                        rt.active = false;
                        kills.extend(rt.q.drain(..));
                    } else {
                        act.push(li as u32);
                        // The head of a gridlock (rare: roundabouts full, deadlocked
                        // junctions) leaves; sooner where it can't be seen well.
                        if let Some(&f) = rt.q.front() {
                            if vehs[f as usize].stuck > if d > 400.0 { 40.0 } else { 120.0 } {
                                kills.extend(rt.q.pop_front());
                            }
                        }
                        if d > REGULATE_DIST && ln.density > 0.0 {
                            let max_n = (ln.len * ln.density * k_t * 1.6 + 1.0) as usize;
                            while rt.q.len() > max_n {
                                kills.extend(rt.q.pop_back());
                            }
                        }
                    }
                } else if d < r {
                    rt.active = true;
                    rt.n = rt.n.wrapping_add(1);
                    act.push(li as u32);
                    fill.push(Link { slot: si as u16, lane: li as u32 });
                }
            }
            t.active = act;
        }
        for v in kills {
            self.kill(v);
        }
        for l in fill {
            self.populate(l);
        }
        // Sources: active lanes no active lane feeds.
        for oi in 0..self.order.len() {
            let si = self.order[oi];
            let Some(t) = self.slots[si].as_ref() else { continue };
            let act = t.active.clone();
            for li in act {
                let (edge_in, npred) = {
                    let t = self.slots[si].as_ref().unwrap();
                    let ln = &t.net.lanes[li as usize];
                    (ln.has(flag::EDGE_IN), ln.npred)
                };
                let fed = if edge_in {
                    self.resolve_edge(si as u16, li, false).is_some_and(|p| self.rt(p).active)
                } else {
                    let t = self.slots[si].as_ref().unwrap();
                    npred > 0 && t.net.preds(li).iter().any(|&p| t.lanes[p as usize].active)
                };
                let t = self.slots[si].as_mut().unwrap();
                let rt = &mut t.lanes[li as usize];
                if !fed && !rt.source {
                    rt.spawn_t = self.t;
                }
                rt.source = !fed;
            }
        }
        self.occupied.clear();
        self.sources.clear();
        let mut n_active = 0;
        for &si in &self.order {
            let Some(t) = self.slots[si].as_ref() else { continue };
            n_active += t.active.len();
            for &li in &t.active {
                let (rt, ln) = (&t.lanes[li as usize], &t.net.lanes[li as usize]);
                let l = Link { slot: si as u16, lane: li };
                if !rt.q.is_empty() {
                    self.occupied.push(l);
                }
                if rt.source && ln.density > 0.0 && !ln.has(flag::CONNECTOR) {
                    self.sources.push(l);
                }
            }
        }
        self.stats.active_lanes = n_active as u32;
        self.act_t = self.t;
        self.act_eye = eye2;
    }

    /// Push the vehicles of a lane back so none overlaps the one ahead.
    fn fix_gaps(&mut self, l: Link) {
        let q = &self.slots[l.slot as usize].as_ref().unwrap().lanes[l.lane as usize].q;
        let mut lead: Option<(f32, f32)> = None;
        for &i in q {
            let v = &mut self.vehs[i as usize];
            if let Some((ls, lv)) = lead {
                if v.s > ls - MIN_GAP {
                    v.s = ls - MIN_GAP;
                    v.v = v.v.min(lv);
                }
            }
            lead = Some((v.s - vehicle::spec(v.kind).len, v.v));
        }
    }

    /// The vehicle at the back of a lane: (s, length, speed).
    fn tail(&self, l: Link) -> Option<(f32, f32, f32)> {
        let rt = self.rt(l);
        let &i = rt.q.back()?;
        let v = &self.vehs[i as usize];
        Some((v.s, vehicle::spec(v.kind).len, v.v))
    }

    /// Nearest leader beyond the end of a vehicle's lane: (gap from the lane end, speed).
    fn leader_beyond(&self, next: Option<Link>) -> Option<(f32, f32)> {
        let mut at = next?;
        let mut dist = 0.0;
        for _ in 0..3 {
            if !self.rt(at).active {
                return None;
            }
            if let Some((s, len, v)) = self.tail(at) {
                return Some((dist + s - len, v));
            }
            let ln = self.lane(at);
            dist += ln.len;
            if dist > LOOKAHEAD {
                return None;
            }
            // Beyond: only a connector's single successor is certain.
            if ln.has(flag::CONNECTOR) {
                let t = self.slots[at.slot as usize].as_ref().unwrap();
                at = Link { slot: at.slot, lane: t.net.succ[ln.succ0 as usize].lane };
            } else {
                return None;
            }
        }
        None
    }

    // ------------------------------------------------------------ step

    fn step(&mut self) {
        self.t += DT;
        let dt = DT as f32;
        if self.t - self.act_t > 0.5 || vec2(self.eye.x, self.eye.z).distance(self.act_eye) > 60.0 {
            self.activate();
        }
        self.stamp = self.stamp.wrapping_add(1);
        let stamp = self.stamp;
        // Only lanes with vehicles are visited; lanes that gain one this step join the list.
        let active = std::mem::take(&mut self.occupied);
        let mut added: Vec<Link> = Vec::new();
        for &l in &active {
            self.slots[l.slot as usize].as_mut().unwrap().lanes[l.lane as usize].mark = stamp;
        }

        // 1. Junction claims: front vehicles approaching a conflicting connector.
        for &l in &active {
            let Some(&fi) = self.rt(l).q.front() else { continue };
            let v = self.vehs[fi as usize];
            let Some(c) = v.next else { continue };
            let cl = self.lane(c);
            if cl.nconf == 0 || c.slot != l.slot {
                continue;
            }
            let rem = self.lane(l).len - v.s;
            if rem > 45.0 + v.v * v.v / 3.0 || !self.room_after(c, vehicle::spec(v.kind).len) {
                continue;
            }
            let eta = rem / v.v.max(2.0);
            let prio = cl.prio - v.wait / 8.0;
            let rt = &mut self.slots[c.slot as usize].as_mut().unwrap().lanes[c.lane as usize];
            if rt.claim_step != stamp || eta < rt.claim_eta {
                rt.claim_step = stamp;
                rt.claim_eta = eta;
                rt.claim_prio = prio;
                rt.claim_veh = fi;
            }
        }

        // 2. Accelerations.
        for &l in &active {
            let (len, speed, is_conn) = {
                let ln = self.lane(l);
                (ln.len, ln.speed, ln.has(flag::CONNECTOR))
            };
            let n = self.rt(l).q.len();
            for qi in 0..n {
                let i = self.rt(l).q[qi];
                let v = self.vehs[i as usize];
                let sp = vehicle::spec(v.kind);
                let rem = len - v.s;
                // Desired speed, slowing for the curve of the next connector.
                let mut v0 = (speed * v.v0f).min(sp.vmax);
                if let Some(nx) = v.next {
                    if !is_conn {
                        let vc = self.lane(nx).speed * v.v0f.min(1.0);
                        if vc < v0 {
                            v0 = v0.min((vc * vc + 2.0 * 1.4 * (rem - 2.0).max(0.0)).sqrt());
                        }
                    }
                }
                let mut a;
                if qi > 0 {
                    let lead = self.vehs[self.rt(l).q[qi - 1] as usize];
                    let gap = lead.s - vehicle::spec(lead.kind).len - v.s;
                    a = idm(sp, v.v, v0, gap, v.v - lead.v);
                } else {
                    a = match self.leader_beyond(v.next) {
                        Some((g, lv)) => idm(sp, v.v, v0, rem + g, v.v - lv),
                        None => idm(sp, v.v, v0, 1e4, 0.0),
                    };
                    // Stop line: the way through the junction is not clear.
                    let mut stop = false;
                    if let Some(c) = v.next {
                        if c.slot == l.slot && !is_conn {
                            stop = self.blocked(c, i, rem, v.v);
                        }
                        // The road ahead is not simulated (outside the active area / unloaded tile): free.
                    }
                    if stop {
                        a = a.min(idm(sp, v.v, v0, rem - 1.0, v.v));
                    }
                    let vm = &mut self.vehs[i as usize];
                    vm.wait = if stop && v.v < 0.5 { v.wait + dt } else { 0.0 };
                }
                let vm = &mut self.vehs[i as usize];
                vm.a = a;
                vm.brake = a < -1.0 || (vm.v < 0.3 && a <= 0.05);
            }
        }

        // 3. Integrate, no overlaps within a lane.
        for &l in &active {
            let n = self.rt(l).q.len();
            let mut lead: Option<(f32, f32)> = None;
            for qi in 0..n {
                let i = self.rt(l).q[qi] as usize;
                let v = &mut self.vehs[i];
                let nv = (v.v + v.a * dt).max(0.0);
                let ds = if nv == 0.0 && v.a < 0.0 { (-0.5 * v.v * v.v / v.a).min(v.v * dt) } else { 0.5 * (v.v + nv) * dt };
                v.s += ds.max(0.0);
                v.v = nv;
                v.stuck = if nv < 0.1 { v.stuck + dt } else { 0.0 };
                if let Some((ls, lv)) = lead {
                    if v.s > ls - MIN_GAP {
                        v.s = (ls - MIN_GAP).max(v.s - ds);
                        v.v = v.v.min(lv);
                    }
                }
                lead = Some((v.s - vehicle::spec(v.kind).len, v.v));
            }
        }

        // 4. Lane changes at the ends (sinks: dead ends, the edge of the area).
        for &l in &active {
            while let Some(&fi) = self.rt(l).q.front() {
                let v = self.vehs[fi as usize];
                let len = self.lane(l).len;
                if v.s <= len {
                    break;
                }
                let target = v.next.filter(|n| self.slots[n.slot as usize].is_some() && self.rt(*n).active);
                let mut s = v.s - len;
                let mut vv = v.v;
                if let Some(nx) = target {
                    // Never onto the back of the vehicle ahead: no room, wait at the end.
                    if let Some((ts, tl, tv)) = self.tail(nx) {
                        let max_s = ts - tl - MIN_GAP;
                        if max_s < -0.2 {
                            let vm = &mut self.vehs[fi as usize];
                            vm.s = len;
                            vm.v = 0.0;
                            self.fix_gaps(l);
                            break;
                        }
                        if s > max_s {
                            s = max_s;
                            vv = vv.min(tv);
                        }
                    }
                }
                self.slots[l.slot as usize].as_mut().unwrap().lanes[l.lane as usize].q.pop_front();
                let Some(nx) = target else {
                    self.kill(fi);
                    continue;
                };
                let hops = v.hops + 1;
                let next2 = self.choose_next(nx, v.seed, hops);
                let vm = &mut self.vehs[fi as usize];
                vm.slot = nx.slot;
                vm.lane = nx.lane;
                vm.s = s;
                vm.v = vv;
                vm.hops = hops;
                vm.hint = 0;
                vm.next = next2;
                // Insert keeping the order (entries from several connectors).
                let vehs = &self.vehs;
                let rt = &mut self.slots[nx.slot as usize].as_mut().unwrap().lanes[nx.lane as usize];
                if rt.mark != stamp {
                    rt.mark = stamp;
                    added.push(nx);
                }
                let q = &mut rt.q;
                let mut at = q.len();
                while at > 0 && vehs[q[at - 1] as usize].s < s {
                    at -= 1;
                }
                q.insert(at, fi);
            }
        }

        // 5. Arrivals at the sources.
        if self.alive < self.cap() {
            let k_t = self.time_factor() * self.level.density();
            let sources = std::mem::take(&mut self.sources);
            for &l in &sources {
                let (src, due) = {
                    let rt = self.rt(l);
                    (rt.source, rt.spawn_t <= self.t)
                };
                let (density, speed, conn) = {
                    let ln = self.lane(l);
                    (ln.density, ln.speed, ln.has(flag::CONNECTOR))
                };
                if !src || !due || conn || density <= 0.0 {
                    continue;
                }
                let tail = self.tail(l);
                let rate = (density * k_t * speed * 0.8).max(1e-4);
                let t = self.slots[l.slot as usize].as_mut().unwrap();
                let rt = &mut t.lanes[l.lane as usize];
                rt.n = rt.n.wrapping_add(1);
                let seed = t.seed ^ (l.lane as u64) << 22 ^ (rt.n as u64) << 44 ^ 0xa441;
                // Poisson arrivals (exponential gaps).
                rt.spawn_t = self.t + (-(1.0 - h01(seed).min(0.999)).ln() / rate) as f64;
                let mut v = self.make(l, 0.0, seed);
                let sp = vehicle::spec(v.kind);
                if let Some((ts, tl, tv)) = tail {
                    let gap = ts - tl;
                    if gap < sp.s0 + v.v.min(tv) * sp.t * 0.6 + 1.0 {
                        // Queue backed up to the source: try again soon.
                        self.slots[l.slot as usize].as_mut().unwrap().lanes[l.lane as usize].spawn_t = self.t + 0.5;
                        continue;
                    }
                    v.v = v.v.min(tv + 2.0);
                }
                let i = self.alloc(v);
                let nx = self.choose_next(l, self.vehs[i as usize].seed, 0);
                self.vehs[i as usize].next = nx;
                let rt = &mut self.slots[l.slot as usize].as_mut().unwrap().lanes[l.lane as usize];
                rt.q.push_back(i);
                if rt.mark != stamp {
                    rt.mark = stamp;
                    added.push(l);
                }
            }
            self.sources = sources;
        }
        let mut occ = active;
        occ.extend(added);
        occ.retain(|l| !self.rt(*l).q.is_empty());
        self.occupied = occ;
    }

    /// Is there room beyond connector `c` for a vehicle `len` long (don't
    /// enter a junction you can't leave)?
    fn room_after(&self, c: Link, len: f32) -> bool {
        let t = self.slots[c.slot as usize].as_ref().unwrap();
        let cl = &t.net.lanes[c.lane as usize];
        if let Some((cs, ctl, cv)) = self.tail(c) {
            if cs - ctl < 1.0 && cv < 1.0 {
                return false;
            }
        }
        let out = Link { slot: c.slot, lane: t.net.succ[cl.succ0 as usize].lane };
        match self.tail(out) {
            Some((ts, tl, tv)) => ts - tl > len + 2.5 || tv > 3.0,
            None => true,
        }
    }

    /// May vehicle `me` (on its lane, `rem` m from the end at `v` m/s) not
    /// enter connector `c` yet?
    fn blocked(&self, c: Link, me: u32, rem: f32, v: f32) -> bool {
        let t = self.slots[c.slot as usize].as_ref().unwrap();
        let cl = &t.net.lanes[c.lane as usize];
        if cl.nconf == 0 {
            return false;
        }
        // Too close to stop comfortably: committed.
        if rem < v * v / (2.0 * 3.5) + 0.5 && v > 1.0 {
            return false;
        }
        // Only decide near the line.
        if rem > 40.0 + v * v / 3.0 {
            return false;
        }
        let me_v = &self.vehs[me as usize];
        if !self.room_after(c, vehicle::spec(me_v.kind).len) {
            return true;
        }
        let my_prio = cl.prio - me_v.wait / 8.0;
        let my_eta = rem / v.max(2.0);
        for &k in t.net.conflicts(c.lane) {
            let rt = &t.lanes[k as usize];
            if !rt.q.is_empty() {
                return true;
            }
            if rt.claim_step == self.stamp && rt.claim_veh != me {
                let first = rt.claim_prio < my_prio - 0.25
                    || ((rt.claim_prio - my_prio).abs() <= 0.25 && (rt.claim_eta < my_eta - 0.1 || ((rt.claim_eta - my_eta).abs() <= 0.1 && k < c.lane)));
                if first && rt.claim_eta < 5.0 {
                    return true;
                }
            }
        }
        false
    }

    // ------------------------------------------------------------ output

    /// Instances for time `t` (≥ the last step), as seen from `view`.
    pub fn snapshot(&mut self, t: f64, view: &View, out: &mut Snapshot) {
        let nb = vehicle::KINDS * 4;
        let mut buckets = std::mem::take(&mut self.buckets);
        if buckets.len() != nb {
            buckets = vec![Vec::new(); nb];
        }
        for b in &mut buckets {
            b.clear();
        }
        let dt = (t - self.t).clamp(0.0, 0.3) as f32;
        let eye = view.eye;
        let draw2 = view.draw_dist * view.draw_dist;
        let light2 = view.light_dist * view.light_dist;
        if self.act_t == f64::MIN {
            self.activate();
        }
        for &Link { slot, lane: li } in &self.occupied {
            let Some(t) = self.slots[slot as usize].as_ref() else { continue };
            let xf = t.xf;
            let sx = xf.sx as f32;
            {
                let rt = &t.lanes[li as usize];
                if rt.q.is_empty() {
                    continue;
                }
                let ln = &t.net.lanes[li as usize];
                if ln.has(flag::TUNNEL) {
                    continue;
                }
                // Lane box against the view (cheap reject).
                let (blo, bhi) = (xf.apply(Vec2::from(ln.lo)), xf.apply(Vec2::from(ln.hi)));
                let bd = (blo - vec2(eye.x, eye.z)).max(vec2(eye.x, eye.z) - bhi).max(Vec2::ZERO).length_squared();
                if bd > light2.max(draw2) {
                    continue;
                }
                for &vi in &rt.q {
                    let v = &self.vehs[vi as usize];
                    // Extrapolate along the lane.
                    let mut s = v.s + v.v * dt + 0.5 * v.a * dt * dt;
                    if v.v + v.a * dt < 0.0 {
                        s = v.s + 0.5 * v.v * v.v / (-v.a).max(1e-3);
                    }
                    let (mut lane, mut net_t) = (li, t);
                    if s > ln.len {
                        if let Some(nx) = v.next {
                            if let Some(t2) = self.slots[nx.slot as usize].as_ref() {
                                s -= ln.len;
                                lane = nx.lane;
                                net_t = t2;
                                if t2.net.lanes[lane as usize].has(flag::TUNNEL) {
                                    continue;
                                }
                            }
                        }
                    }
                    let mut hint = if lane == li { v.hint } else { 0 };
                    let (p, d, grade) = net_t.net.eval(lane, s, &mut hint);
                    let xf2 = net_t.xf;
                    let w = xf2.apply(vec2(p[0], p[2]));
                    let pos = Vec3::new(w.x, p[1], w.y);
                    let rel = pos - eye;
                    let d2 = rel.length_squared();
                    let lod = if d2 < view.lod0_dist * view.lod0_dist {
                        0
                    } else if d2 < view.lod0_dist * view.lod0_dist * 9.0 {
                        1
                    } else if d2 < draw2 {
                        2
                    } else if d2 < light2 {
                        3
                    } else {
                        continue;
                    };
                    // Behind the camera (with a margin for long vehicles / shadows).
                    if d2 > 3600.0 && rel.dot(view.fwd) < view.cos_half * d2.sqrt() {
                        continue;
                    }
                    let dir = vec2(d.x * sx, d.y).normalize_or(d);
                    let info = v.kind as u32 | (v.brake as u32) << 8 | (v.seed & 0xffff) << 16;
                    buckets[lod * vehicle::KINDS + v.kind as usize].push((VehInst { pos: pos.into(), dx: dir.x, dz: dir.y, grade, col: v.col, info }, (v.v + v.a * dt).max(0.0)));
                }
            }
        }
        out.inst.clear();
        out.speed.clear();
        out.ranges.clear();
        for lod in 0..4u8 {
            for k in 0..vehicle::KINDS {
                let b = &buckets[lod as usize * vehicle::KINDS + k];
                if b.is_empty() {
                    continue;
                }
                out.ranges.push((k as u8, lod, out.inst.len() as u32, b.len() as u32));
                out.inst.extend(b.iter().map(|x| x.0));
                out.speed.extend(b.iter().map(|x| x.1));
            }
        }
        self.buckets = buckets;
        out.time = t;
        self.stats.vehicles = self.alive as u32;
        self.stats.drawn = out.inst.len() as u32;
        out.stats = self.stats;
    }

    /// Check invariants (tests): queues ordered, no overlaps, vehicles alive and where they say.
    #[cfg(test)]
    pub fn check(&self) -> Result<(), String> {
        let mut seen = 0;
        for (si, t) in self.slots.iter().enumerate() {
            let Some(t) = t else { continue };
            for (li, rt) in t.lanes.iter().enumerate() {
                let mut prev: Option<&Veh> = None;
                for &i in &rt.q {
                    let v = &self.vehs[i as usize];
                    seen += 1;
                    if !v.alive || v.slot as usize != si || v.lane as usize != li {
                        return Err(format!("vehicle {i} misplaced"));
                    }
                    if !v.s.is_finite() || !v.v.is_finite() || v.v < 0.0 {
                        return Err(format!("vehicle {i} bad state s {} v {}", v.s, v.v));
                    }
                    if let Some(p) = prev {
                        let gap = p.s - vehicle::spec(p.kind).len - v.s;
                        if gap < -0.01 {
                            return Err(format!("overlap {gap:.2} m on lane {li}"));
                        }
                    }
                    prev = Some(v);
                }
            }
        }
        if seen != self.alive {
            return Err(format!("{seen} queued vs {} alive", self.alive));
        }
        Ok(())
    }

    /// Smallest gap between consecutive vehicles on any lane (tests).
    #[cfg(test)]
    pub fn min_gap(&self) -> f32 {
        let mut m = f32::MAX;
        for t in self.slots.iter().flatten() {
            for rt in &t.lanes {
                for w in rt.q.iter().collect::<Vec<_>>().windows(2) {
                    let (a, b) = (&self.vehs[*w[0] as usize], &self.vehs[*w[1] as usize]);
                    m = m.min(a.s - vehicle::spec(a.kind).len - b.s);
                }
            }
        }
        m
    }

    /// Queues (tests): per class (vehicles, stopped), and the worst lanes.
    #[cfg(test)]
    pub fn diagnose(&self) -> String {
        let mut by = [(0usize, 0usize); 9];
        let mut worst: Vec<(usize, String)> = Vec::new();
        for (si, t) in self.slots.iter().enumerate() {
            let Some(t) = t else { continue };
            for &li in &t.active {
                let rt = &t.lanes[li as usize];
                let ln = &t.net.lanes[li as usize];
                let c = ln.class().min(8) as usize;
                let stopped = rt.q.iter().filter(|&&i| self.vehs[i as usize].v < 0.5).count();
                by[c].0 += rt.q.len();
                by[c].1 += stopped;
                if stopped >= 3 {
                    let f = &self.vehs[rt.q[0] as usize];
                    let nx = f.next.map(|n| {
                        let nl = self.lane(n);
                        let tl = self.rt(n).q.back().map(|&i| {
                            let v = &self.vehs[i as usize];
                            format!(
                                "tail s {:.1} v {:.1} a {:.1} next {:?} lane active {} len {:.0} nsucc {} flags {:x}",
                                v.s,
                                v.v,
                                v.a,
                                v.next,
                                self.rt(n).active,
                                nl.len,
                                nl.nsucc,
                                nl.flags()
                            )
                        });
                        format!(
                            "next {:?} c{} conf {} q {} room {} {:?}",
                            n,
                            nl.class(),
                            nl.nconf,
                            self.rt(n).q.len(),
                            if nl.has(flag::CONNECTOR) { self.room_after(n, 4.0) } else { true },
                            tl
                        )
                    });
                    worst.push((
                        stopped,
                        format!(
                            "slot {si} lane {li} c{} len {:.0} q {} stopped {stopped} src {} front s {:.1} v {:.1} wait {:.0} {:?}",
                            ln.class(),
                            ln.len,
                            rt.q.len(),
                            rt.source,
                            f.s,
                            f.v,
                            f.wait,
                            nx
                        ),
                    ));
                }
            }
        }
        worst.sort_by_key(|a| std::cmp::Reverse(a.0));
        let w: Vec<String> = worst.iter().take(12).map(|w| w.1.clone()).collect();
        format!("by class {:?}\n{}", by, w.join("\n"))
    }

    #[cfg(test)]
    pub fn slot_of(&self, id: TileId) -> Option<u16> {
        self.by_id.get(&id).copied()
    }

    /// (slot, lane, s) of every vehicle (tests).
    #[cfg(test)]
    pub fn positions(&self) -> Vec<(u16, u32, f32, u32)> {
        self.vehs.iter().enumerate().filter(|(_, v)| v.alive).map(|(i, v)| (v.slot, v.lane, v.s, i as u32)).collect()
    }

    #[cfg(test)]
    pub fn edge_link(&mut self, id: TileId, lane: u32) -> Option<(TileId, u32)> {
        let s = *self.by_id.get(&id)?;
        let l = self.resolve_edge(s, lane, true)?;
        let t = self.slots[l.slot as usize].as_ref()?;
        Some((t.net.id, l.lane))
    }
}
