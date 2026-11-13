//! Taxiing a route like a crew: follow a taxiway-centreline polyline (the
//! ATC taxi graph, `atc/taxi.rs`) with the main gear on the centreline
//! (pure pursuit from the main-gear midpoint, so the nose swings wide in the
//! turns and the mains stay on the pavement — "judgemental oversteer"), at a
//! sensible speed: ~20 kt on straights for jets, slowing for the corners
//! (lateral acceleration ≤ ~0.1 g) and to stop at the end (the holding point
//! or the stand), braking gently. Tiller / pedals / differential braking by
//! type, as a pilot would steer it.

use super::{GroundSpec, Steer, pushback::geometry};
use crate::flight::{Aircraft, AircraftDef, Category, ContactKind, Controls, EngineKind, KT};
use glam::{Vec2, Vec3};

/// Taxi state along one route.
#[derive(Clone, Debug)]
pub struct Taxi {
    /// Route (world XZ), densified to ≤ 4 m spacing.
    pub pts: Vec<Vec2>,
    /// Cumulative distance along the route at each point.
    cum: Vec<f32>,
    /// Arc length of the main gear's projection (progress).
    pub s: f32,
    /// Speed limit at each point (m/s), with the deceleration preview applied.
    vmax: Vec<f32>,
    /// Stop with the nose this far short of the last point (m).
    pub stop_short: f32,
    thr_i: f32,
    pub done: bool,
    /// Cross-track error of the main gear (m), for tests / the HUD.
    pub xtrack: f32,
    pub v_target: f32,
}

/// Straight-line taxi speed (m/s) for a type.
pub fn straight_speed(def: &AircraftDef) -> f32 {
    KT * match def.category {
        Category::Airliner => 20.0,
        Category::FastJet => 20.0,
        Category::Warbird => 12.0,
        _ => 14.0,
    }
}

/// Lateral acceleration the crew accepts in a turn (m/s²).
fn lat_accel(def: &AircraftDef) -> f32 {
    match def.category {
        Category::Airliner => 0.9,
        _ => 1.2,
    }
}

fn densify(route: &[Vec2], step: f32) -> Vec<Vec2> {
    let mut out: Vec<Vec2> = Vec::new();
    for w in route.windows(2) {
        let n = (w[0].distance(w[1]) / step).ceil().max(1.0) as usize;
        for k in 0..n {
            out.push(w[0].lerp(w[1], k as f32 / n as f32));
        }
    }
    if let Some(l) = route.last() {
        out.push(*l);
    }
    out.dedup_by(|a, b| a.distance(*b) < 0.2);
    out
}

impl Taxi {
    pub fn new(def: &AircraftDef, route: &[Vec2], stop_short: f32) -> Taxi {
        // Smooth the graph's corners a little (the network is resampled at 12 m).
        let mut pts = densify(route, 4.0);
        for _ in 0..3 {
            let src = pts.clone();
            for i in 1..pts.len().saturating_sub(1) {
                pts[i] = src[i] * 0.5 + (src[i - 1] + src[i + 1]) * 0.25;
            }
        }
        let mut cum = vec![0.0; pts.len()];
        for i in 1..pts.len() {
            cum[i] = cum[i - 1] + pts[i - 1].distance(pts[i]);
        }
        // Curvature speed limits (radius from three points ~12 m apart).
        let n = pts.len();
        let vs = straight_speed(def);
        let a_lat = lat_accel(def);
        let mut vmax = vec![vs; n];
        for i in 0..n {
            let (a, b) = (i.saturating_sub(3), (i + 3).min(n - 1));
            if b > a + 1 {
                let (p, q, r) = (pts[a], pts[i], pts[b]);
                let area2 = (q - p).perp_dot(r - p).abs();
                let abc = p.distance(q) * q.distance(r) * p.distance(r);
                let radius = if area2 > 1e-3 { abc / (2.0 * area2) } else { 1e4 };
                vmax[i] = vs.min((a_lat * radius).sqrt().max(2.2));
            }
        }
        // Stop at the end.
        let end = (cum.last().copied().unwrap_or(0.0) - stop_short).max(0.0);
        let a_dec = 0.6;
        for i in 0..n {
            let d = (end - cum[i]).max(0.0);
            vmax[i] = vmax[i].min((2.0 * a_dec * d).sqrt());
        }
        // Preview: slow down in time for the corners.
        for i in (0..n.saturating_sub(1)).rev() {
            let d = cum[i + 1] - cum[i];
            vmax[i] = vmax[i].min((vmax[i + 1] * vmax[i + 1] + 2.0 * a_dec * d).sqrt());
        }
        Taxi { pts, cum, s: 0.0, vmax, stop_short, thr_i: 0.0, done: false, xtrack: 0.0, v_target: 0.0 }
    }

    pub fn length(&self) -> f32 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    /// Distance left to the stopping point (m).
    pub fn remaining(&self) -> f32 {
        (self.length() - self.stop_short - self.s).max(0.0)
    }

    fn point_at(&self, s: f32) -> Vec2 {
        let s = s.clamp(0.0, self.length());
        let i = self.cum.partition_point(|c| *c < s).clamp(1, self.pts.len().max(2) - 1);
        let (c0, c1) = (self.cum[i - 1], self.cum[i]);
        let k = if c1 > c0 { (s - c0) / (c1 - c0) } else { 0.0 };
        self.pts[i - 1].lerp(self.pts[i], k)
    }

    fn vmax_at(&self, s: f32) -> f32 {
        let i = self.cum.partition_point(|c| *c < s).min(self.vmax.len().saturating_sub(1));
        self.vmax.get(i).copied().unwrap_or(0.0)
    }

    /// Project `p` on the route near the current progress.
    fn project(&mut self, p: Vec2) {
        let mut best = (f32::INFINITY, self.s, 0.0);
        let i0 = self.cum.partition_point(|c| *c < self.s - 30.0).saturating_sub(1);
        for i in i0..self.pts.len().saturating_sub(1) {
            if self.cum[i] > self.s + 80.0 {
                break;
            }
            let (a, b) = (self.pts[i], self.pts[i + 1]);
            let ab = b - a;
            let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            let q = a + ab * t;
            let d = p.distance(q);
            if d < best.0 {
                best = (d, self.cum[i] + t * ab.length(), ab.normalize_or_zero().perp_dot(p - a).signum() * d);
            }
        }
        if best.0.is_finite() {
            self.s = best.1.max(self.s - 1.0);
            self.xtrack = best.2;
        }
    }

    /// Drive the controls for one step. Returns true when stopped at the end.
    pub fn update(&mut self, def: &AircraftDef, sp: &GroundSpec, ac: &Aircraft, ctl: &mut Controls, dt: f32) -> bool {
        let (nose, mid) = match geometry(def) {
            Some(g) => g,
            None => {
                // Taildragger: the tail wheel is aft; steer about the mains.
                let mains: Vec<Vec3> = def.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { braked: true, .. })).map(|g| g.pos).collect();
                let mid = if mains.is_empty() { Vec3::ZERO } else { mains.iter().copied().sum::<Vec3>() / mains.len() as f32 };
                (mid + Vec3::Z * 6.0, mid)
            }
        };
        let wheelbase = (nose.z - mid.z).abs().max(2.0);
        let pm = ac.pos + ac.rot * mid;
        let p = Vec2::new(pm.x, pm.z);
        self.project(p);
        let gs = Vec2::new(ac.vel.x, ac.vel.z).length();
        let hdg = ac.heading();
        let fwd = Vec2::new(hdg.sin(), -hdg.cos());
        // Pure pursuit from the main gear.
        let look = (wheelbase * 1.3 + gs * 1.2).clamp(8.0, 40.0);
        let target = self.point_at(self.s + look);
        let to = target - p;
        let eta = fwd.perp_dot(to.normalize_or(fwd)).asin_safe();
        let dist = to.length().max(1.0);
        // Steering angle of a bicycle with this wheelbase onto the pursuit arc (+ = right).
        let delta = (2.0 * wheelbase * eta.sin() / dist).atan();
        let remaining = self.remaining();
        let v_t = self.vmax_at(self.s + gs * 1.0).min(self.vmax_at(self.s));
        self.v_target = v_t;
        // Steering per type.
        ctl.tiller = 0.0;
        ctl.rudder = 0.0;
        ctl.brake_left = 0.0;
        ctl.brake_right = 0.0;
        let deg = delta.to_degrees();
        match sp.steer {
            Steer::Tiller { tiller_deg, .. } => ctl.tiller = (deg / tiller_deg).clamp(-1.0, 1.0),
            Steer::Pedal { deg: d, fade_ms } | Steer::Caster { link_deg: d, fade_ms, .. } | Steer::TailSteer { deg: d, fade_ms, .. } => {
                let eff = d / (1.0 + (gs / fade_ms).powi(2));
                ctl.rudder = (deg / eff.max(1.0)).clamp(-1.0, 1.0);
                // Past the steering's reach: differential braking on the inside wheel.
                let over = (deg.abs() - eff) / 15.0;
                if over > 0.0 {
                    let b = over.clamp(0.0, 0.6);
                    if deg > 0.0 { ctl.brake_right = b } else { ctl.brake_left = b }
                }
            }
            Steer::TailCaster { .. } | Steer::Fixed => {
                ctl.tail_lock = false;
                ctl.rudder = (deg / 10.0).clamp(-1.0, 1.0);
                let b = (deg.abs() / 25.0).clamp(0.0, 0.7);
                if deg > 0.0 { ctl.brake_right = b } else { ctl.brake_left = b }
            }
        }
        // Speed: throttle (breakaway thrust when stopped), gentle brakes.
        let jet = def.engine_kind == EngineKind::Jet;
        let (idle_thr, max_thr) = match def.category {
            Category::Airliner if jet => (0.0, 0.42),
            Category::Airliner => (0.04, 0.35),
            Category::FastJet => (0.0, 0.25),
            _ => (0.05, 0.45),
        };
        let e = v_t - gs;
        self.thr_i = (self.thr_i + e * 0.03 * dt).clamp(0.0, max_thr);
        let breakaway = if gs < 0.5 && v_t > 1.0 { if jet { 0.28 } else { 0.3 } } else { 0.0 };
        ctl.throttle = (idle_thr + 0.12 * e + self.thr_i).max(breakaway).clamp(0.0, max_thr);
        ctl.reverse = false;
        ctl.parking_brake = false;
        let over = gs - v_t;
        ctl.brakes = if over > 0.4 { ((over - 0.4) * 0.25).clamp(0.0, 0.6) } else { 0.0 };
        if over > 0.0 {
            ctl.throttle = idle_thr;
            self.thr_i *= 1.0 - dt;
        }
        if remaining < 0.5 || (remaining < 3.0 && gs < 0.5) {
            ctl.throttle = 0.0;
            ctl.brakes = 1.0;
            ctl.tiller = 0.0;
            self.done = gs < 0.3;
        }
        self.done
    }
}

trait AsinSafe {
    fn asin_safe(self) -> f32;
}
impl AsinSafe for f32 {
    fn asin_safe(self) -> f32 {
        self.clamp(-1.0, 1.0).asin()
    }
}

// ---------------------------------------------------------------- the bot on the ground

use super::guide;
use super::pushback::{Phase as PushPhase, PushDir, Pushback};
use crate::world::World;

/// What the bot is doing on the ground.
#[derive(Clone, Debug)]
pub enum Stage {
    /// Pushed back by the tug.
    Push(Box<Pushback>),
    /// Taxiing a route; `to_hold`: to the departure holding point (else a stand).
    Taxi(Box<Taxi>, bool),
    /// Parked on a stand (s so far): turnaround, then push and depart again.
    Parked(f32),
}

/// Result of a step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Running,
    /// At the holding point, ready to line up.
    AtHold,
    /// No route (no layout): the caller falls back to its runway taxi.
    Failed,
}

/// The auto-flight bot's ground phases: pushback from a stand, taxi to the
/// departure holding point, and after landing taxi to a free stand, park,
/// then push back and go again.
#[derive(Clone, Debug)]
pub struct BotGround {
    pub stage: Stage,
    /// The departure: runway and the take-off end's threshold / direction.
    pub runway: crate::scene::Runway,
    pub thr: Vec2,
    pub dir: Vec2,
    pub status: String,
    /// The tug's tow this step (applied by the simulation).
    pub tow: Option<super::Tow>,
    /// Taxi route for the map (world XZ).
    pub route: Vec<Vec2>,
}

/// Taxiway centreline segments near the aircraft (for the pushback).
fn lanes(world: &World, ac: &Aircraft) -> Vec<(Vec2, Vec2)> {
    let p = Vec2::new(ac.pos.x, ac.pos.z);
    world.pavement.taxi.iter().filter(|t| super::surface::seg_dist(p, t.0, t.1) < 200.0).map(|t| (t.0, t.1)).collect()
}

/// Seconds parked on the stand between flights.
const TURNAROUND_S: f32 = 30.0;

impl BotGround {
    /// Leave from here: pushed back first if parked nose-in on a stand.
    pub fn depart(def: &AircraftDef, ac: &Aircraft, world: &World, runway: &crate::scene::Runway, thr: Vec2, dir: Vec2) -> Option<BotGround> {
        let nose = guide::nose_xz(def, ac);
        let mut g = BotGround { stage: Stage::Parked(TURNAROUND_S), runway: runway.clone(), thr, dir, status: String::new(), tow: None, route: Vec::new() };
        if guide::on_stand(world, nose, ac.heading()).is_some() && super::pushback::can_push(def) {
            g.stage = Stage::Push(Box::new(Pushback::onto_lane(def, ac, g.push_dir(def, ac, world), &lanes(world, ac))?));
            return Some(g);
        }
        g.taxi_to_hold(def, ac, world).then_some(g)
    }

    /// After landing: taxi to a free stand (and later depart again from `runway`).
    pub fn arrive(def: &AircraftDef, ac: &Aircraft, world: &World, runway: &crate::scene::Runway, thr: Vec2, dir: Vec2) -> Option<BotGround> {
        let nose = guide::nose_xz(def, ac);
        let gd = guide::plan_to_stand(world, nose, def.wingspan)?;
        let t = Taxi::new(def, &gd.route, 1.0);
        Some(BotGround { stage: Stage::Taxi(Box::new(t), false), runway: runway.clone(), thr, dir, status: format!("Taxiing to {}", gd.dest), tow: None, route: gd.route })
    }

    /// Pushback direction: whichever way leaves the nose pointing nearer the taxi route.
    fn push_dir(&self, def: &AircraftDef, ac: &Aircraft, world: &World) -> PushDir {
        let nose = guide::nose_xz(def, ac);
        let toward = guide::plan_departure(world, nose, Some((&self.runway, self.thr, self.dir))).map(|g| g.point_at(g.length().min(250.0)) - nose).unwrap_or(-self.dir);
        let h = ac.heading();
        let after = |d: f32| Vec2::new((h + d).sin(), -(h + d).cos());
        if after(std::f32::consts::FRAC_PI_2).dot(toward) >= after(-std::f32::consts::FRAC_PI_2).dot(toward) { PushDir::TailLeft } else { PushDir::TailRight }
    }

    fn taxi_to_hold(&mut self, def: &AircraftDef, ac: &Aircraft, world: &World) -> bool {
        let nose = guide::nose_xz(def, ac);
        match guide::plan_departure(world, nose, Some((&self.runway, self.thr, self.dir))) {
            Some(gd) => {
                self.status = format!("Taxiing to {}", gd.dest);
                self.route = gd.route.clone();
                self.stage = Stage::Taxi(Box::new(Taxi::new(def, &gd.route, 8.0)), true);
                true
            }
            None => false,
        }
    }

    pub fn update(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, world: &World, dt: f32) -> Step {
        let sp = super::spec::spec(def);
        self.tow = None;
        ctl.reverse = false;
        ctl.spoilers = 0.0;
        ctl.aileron = 0.0;
        ctl.elevator = if def.taildragger { 1.0 } else { 0.0 };
        ctl.autobrake = super::Autobrake::Off;
        match &mut self.stage {
            Stage::Push(p) => {
                ctl.throttle = 0.0;
                ctl.brakes = 0.0;
                ctl.tiller = 0.0;
                ctl.rudder = 0.0;
                if matches!(p.phase, PushPhase::WaitBrake) {
                    ctl.parking_brake = true;
                } else if matches!(p.phase, PushPhase::Connecting(_)) {
                    ctl.parking_brake = false;
                }
                self.tow = p.update(def, ac, ctl, dt);
                self.status = format!("Pushback {}", p.dir.label());
                if matches!(p.phase, PushPhase::Leaving(_) | PushPhase::Done) {
                    ctl.parking_brake = false;
                    if !self.taxi_to_hold(def, ac, world) {
                        return Step::Failed;
                    }
                }
                Step::Running
            }
            Stage::Taxi(t, to_hold) => {
                let to_hold = *to_hold;
                ctl.parking_brake = false;
                if t.update(def, &sp, ac, ctl, dt) {
                    if to_hold {
                        return Step::AtHold;
                    }
                    self.stage = Stage::Parked(0.0);
                    self.route.clear();
                }
                Step::Running
            }
            Stage::Parked(s) => {
                *s += dt;
                ctl.throttle = 0.0;
                ctl.parking_brake = true;
                ctl.brakes = 0.0;
                self.status = format!("Parked on the stand — turnaround {:.0} s", (TURNAROUND_S - *s).max(0.0));
                if *s > TURNAROUND_S {
                    ctl.parking_brake = false;
                    match Pushback::onto_lane(def, ac, self.push_dir(def, ac, world), &lanes(world, ac)) {
                        Some(p) if super::pushback::can_push(def) => self.stage = Stage::Push(Box::new(p)),
                        _ => {
                            if !self.taxi_to_hold(def, ac, world) {
                                return Step::Failed;
                            }
                        }
                    }
                }
                Step::Running
            }
        }
    }

    /// The world origin moved: `f` maps ground positions to the new frame.
    pub fn rebase(&mut self, f: &dyn Fn(Vec2) -> Vec2) {
        for p in &mut self.route {
            *p = f(*p);
        }
        self.thr = f(self.thr);
        match &mut self.stage {
            Stage::Push(p) => {
                p.start = f(p.start);
                p.tug.pos = f(p.tug.pos);
            }
            Stage::Taxi(t, _) => {
                for p in &mut t.pts {
                    *p = f(*p);
                }
            }
            Stage::Parked(_) => {}
        }
    }

    /// The tug (for drawing), while pushing.
    pub fn pushback(&self) -> Option<&Pushback> {
        match &self.stage {
            Stage::Push(p) => Some(p),
            _ => None,
        }
    }
}
