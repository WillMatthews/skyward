//! Taxi guidance: a taxi route over the airport's taxiway graph (the ATC
//! clearance's, or planned here to the departure holding point / the nearest
//! stand), progress along it, the next turn, a speed advisory, and
//! "follow the greens": the taxiway centreline lights along the route light
//! up green ahead of the aircraft (A-SMGCS style), ending at a red stop bar
//! at the holding point.

use crate::atc::taxi;
use crate::lights::{LightInst, colour, kind};
use crate::world::World;
use glam::{Vec2, Vec3, vec2, vec3};

/// Centreline light spacing (m) and how far ahead they are lit.
const SPACING: f32 = 15.0;
const LIT_AHEAD: f32 = 420.0;

#[derive(Clone, Debug, Default)]
pub struct Guide {
    /// Route (world XZ) and the cumulative distance at each point.
    pub route: Vec<Vec2>,
    cum: Vec<f32>,
    /// Taxiway refs along it, and where it ends ("holding point A4, RWY 27R").
    pub via: Vec<String>,
    pub dest: String,
    /// Ends at a runway-holding point (red stop bar) rather than a stand.
    pub to_hold: bool,
    /// Progress (m) of the nose along the route, and its distance from it.
    pub s: f32,
    pub off: f32,
    /// Next turn: distance (m) and angle (rad, + = right).
    pub next_turn: Option<(f32, f32)>,
    /// Advisory speed (m/s).
    pub advisory: f32,
    /// Follow-the-greens lights on.
    pub greens: bool,
    /// Bumped when the lit lights change (the renderer re-uploads).
    pub version: u64,
    lit_k: i64,
}

fn wrap(a: f32) -> f32 {
    let mut d = a.rem_euclid(std::f32::consts::TAU);
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    d
}

impl Guide {
    pub fn new(route: Vec<Vec2>, via: Vec<String>, dest: String, to_hold: bool) -> Guide {
        let mut cum = vec![0.0; route.len()];
        for i in 1..route.len() {
            cum[i] = cum[i - 1] + route[i - 1].distance(route[i]);
        }
        Guide { route, cum, via, dest, to_hold, greens: true, version: 1, lit_k: i64::MIN, ..Default::default() }
    }

    pub fn active(&self) -> bool {
        self.route.len() >= 2
    }

    pub fn length(&self) -> f32 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    pub fn remaining(&self) -> f32 {
        (self.length() - self.s).max(0.0)
    }

    pub fn point_at(&self, s: f32) -> Vec2 {
        if self.route.is_empty() {
            return Vec2::ZERO;
        }
        let s = s.clamp(0.0, self.length());
        let i = self.cum.partition_point(|c| *c < s).clamp(1, self.route.len().max(2) - 1).min(self.route.len() - 1);
        let (c0, c1) = (self.cum[i - 1], self.cum[i]);
        let k = if c1 > c0 { (s - c0) / (c1 - c0) } else { 0.0 };
        self.route[i - 1].lerp(self.route[i], k)
    }

    fn dir_at(&self, s: f32) -> Vec2 {
        (self.point_at(s + 6.0) - self.point_at(s - 6.0)).normalize_or(Vec2::Y)
    }

    pub fn rebase(&mut self, delta: Vec2) {
        for p in &mut self.route {
            *p += delta;
        }
        self.version += 1;
    }

    /// Track the aircraft (nose position) and refresh the turn / speed advice.
    pub fn update(&mut self, nose: Vec2, straight: f32) {
        if !self.active() {
            return;
        }
        // Project the nose near the last progress.
        let mut best = (f32::INFINITY, self.s);
        for i in 0..self.route.len() - 1 {
            if self.cum[i + 1] < self.s - 60.0 || self.cum[i] > self.s + 150.0 {
                continue;
            }
            let (a, b) = (self.route[i], self.route[i + 1]);
            let ab = b - a;
            let t = ((nose - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            let d = nose.distance(a + ab * t);
            if d < best.0 {
                best = (d, self.cum[i] + t * ab.length());
            }
        }
        if best.0.is_finite() {
            self.s = best.1.max(self.s - 2.0);
            self.off = best.0;
        }
        // Next turn: where the route's direction changes by more than 25° over 40 m.
        self.next_turn = None;
        let mut d = 10.0;
        while self.s + d < self.length() - 5.0 && d < 900.0 {
            let a = self.dir_at(self.s + d);
            let b = self.dir_at(self.s + d + 40.0);
            let ang = wrap(b.x.atan2(-b.y) - a.x.atan2(-a.y));
            if ang.abs() > 25f32.to_radians() {
                self.next_turn = Some((d, ang));
                break;
            }
            d += 8.0;
        }
        // Speed advice: straight-line speed; ≤ 10 kt into a turn; slowing to stop at the end.
        let kt = crate::flight::KT;
        let mut v = straight;
        if let Some((dist, ang)) = self.next_turn {
            let turn_v = if ang.abs() > 60f32.to_radians() { 10.0 * kt } else { 15.0 * kt };
            v = v.min((turn_v * turn_v + 2.0 * 0.6 * (dist - 10.0).max(0.0)).sqrt());
        }
        v = v.min((2.0 * 0.6 * self.remaining()).sqrt());
        self.advisory = v;
        let k = (self.s / SPACING).floor() as i64;
        if k != self.lit_k {
            self.lit_k = k;
            self.version += 1;
        }
    }

    /// Follow-the-greens: green centreline lights from just behind the nose
    /// to ~400 m ahead, and the red stop bar across the end at a holding point.
    pub fn lights(&self, ground: &dyn Fn(Vec2) -> f32) -> Vec<LightInst> {
        let mut out = Vec::new();
        if !self.greens || !self.active() {
            return out;
        }
        let start = ((self.s - 20.0) / SPACING).ceil().max(0.0) * SPACING;
        let end = (self.s + LIT_AHEAD).min(self.length());
        let mut s = start;
        while s <= end {
            let p = self.point_at(s);
            out.push(LightInst::new(vec3(p.x, ground(p) + 0.12, p.y), 140.0, colour::TAXI_GREEN, kind::TAXI));
            // Closer spacing in the curves (7.5 m), as on real centrelines.
            let curve = wrap(self.dir_at(s + 8.0).x.atan2(-self.dir_at(s + 8.0).y) - self.dir_at(s).x.atan2(-self.dir_at(s).y)).abs();
            s += if curve > 0.08 { SPACING * 0.5 } else { SPACING };
        }
        if self.to_hold && end >= self.length() - 1.0 {
            let p = *self.route.last().unwrap_or(&Vec2::ZERO);
            let d = self.dir_at(self.length() - 3.0);
            let side = vec2(-d.y, d.x);
            for k in -4..=4 {
                let q = p + side * (k as f32 * 2.6);
                out.push(LightInst::new(vec3(q.x, ground(q) + 0.15, q.y), 260.0, colour::RWY_RED, kind::TAXI));
            }
        }
        out
    }

    /// A short line for the HUD: "TAXI via B A A4 · LEFT in 240 m · 10 KT".
    pub fn status(&self) -> String {
        let via = if self.via.is_empty() { String::new() } else { format!(" via {}", self.via.join(" ")) };
        let turn = match self.next_turn {
            Some((d, a)) if d < 600.0 => format!(" · {} in {:.0} m", if a > 0.0 { "RIGHT" } else { "LEFT" }, (d / 10.0).round() * 10.0),
            _ => String::new(),
        };
        format!("TAXI to {}{via}{turn} · {:.0} KT · {:.0} m", self.dest, self.advisory / crate::flight::KT, self.remaining())
    }
}

/// The airport and its taxi graph around `p` (world frame), if its layout is built.
pub fn graph_near(world: &World, p: Vec2) -> Option<(String, taxi::Graph, Vec<taxi::RwyBox>, Vec<(Vec2, String, String)>)> {
    let rw = world
        .runways
        .iter()
        .filter(|r| !r.airport.is_empty())
        .min_by(|a, b| crate::ground::surface::seg_dist(p, a.a, a.b).total_cmp(&crate::ground::surface::seg_dist(p, b.a, b.b)))?;
    if crate::ground::surface::seg_dist(p, rw.a, rw.b) > 6000.0 {
        return None;
    }
    let ap = rw.airport.clone();
    let l = world.airport_detail.layout(&ap)?;
    let m = l.frame.map_to(&world.proj);
    let tw: Vec<(Vec<Vec2>, String)> = l.taxiways.iter().map(|(pl, n)| (pl.iter().map(|q| m.apply(*q)).collect(), n.clone())).collect();
    if tw.is_empty() {
        return None;
    }
    let boxes = world.runways.iter().filter(|r| r.airport == ap).map(|r| taxi::RwyBox { a: r.a, b: r.b, width: r.width, name: format!("{}/{}", r.ident_a, r.ident_b) }).collect();
    let holds = l.holds.iter().map(|h| (m.apply(h.at), h.taxiway.clone(), h.runway.clone())).collect();
    Some((ap, taxi::Graph::build(&tw), boxes, holds))
}

/// A route from `p` to `to`: from the nearest graph node, or — if that one is
/// on a piece of the network that does not connect (a stub of apron
/// taxilane) — from the next nearest within reach.
fn route_from(g: &taxi::Graph, p: Vec2, to: Vec2, boxes: &[taxi::RwyBox]) -> Option<taxi::Route> {
    // (The target snaps to its nearest node: a stand / hold may sit off the centreline.)
    let to = g.pts[g.nearest(to, 400.0)? as usize];
    let mut near: Vec<(f32, Vec2)> = g.pts.iter().map(|q| (q.distance(p), *q)).filter(|x| x.0 < 250.0).collect();
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut tried: Vec<Vec2> = Vec::new();
    for (_, q) in near {
        if tried.iter().any(|t| t.distance(q) < 25.0) {
            continue;
        }
        tried.push(q);
        if let Some(mut r) = g.route(q, to, 1.0, boxes) {
            r.pts.insert(0, p);
            return Some(r);
        }
        if tried.len() >= 8 {
            break;
        }
    }
    None
}

/// Plan a route from `p` to the departure holding point of the runway end
/// best into wind (or `end`: (runway, threshold, direction)). Returns the guide.
pub fn plan_departure(world: &World, p: Vec2, end: Option<(&crate::scene::Runway, Vec2, Vec2)>) -> Option<Guide> {
    let (ap, g, boxes, holds) = graph_near(world, p)?;
    let wind = world.wx.wind();
    let (rw, thr, dir) = match end {
        Some(e) => (e.0.clone(), e.1, e.2),
        None => {
            // Into wind: the end whose direction best opposes the wind's motion.
            let mut best: Option<(f32, crate::scene::Runway, Vec2, Vec2)> = None;
            for r in world.runways.iter().filter(|r| r.airport == ap && r.paved) {
                for b_end in [false, true] {
                    let (thr, dir) = crate::airports::threshold(r, b_end);
                    let score = -wind.dot(dir) + r.length() * 1e-3;
                    if best.as_ref().is_none_or(|x| score > x.0) {
                        best = Some((score, r.clone(), thr, dir));
                    }
                }
            }
            let b = best?;
            (b.1, b.2, b.3)
        }
    };
    let name = format!("{}/{}", rw.ident_a, rw.ident_b);
    let rname = if (rw.b - thr).length() < (rw.a - thr).length() { rw.ident_b.clone() } else { rw.ident_a.clone() };
    let mut best: Option<(f32, taxi::HoldPt, taxi::Route)> = None;
    for (h, along) in taxi::departure_holds(&holds, &name, thr, dir) {
        let Some(r) = route_from(&g, p, h.at, &boxes) else { continue };
        let score = r.length + 1.5 * along + if r.crosses.contains(&name) { 5000.0 } else { 0.0 };
        if best.as_ref().is_none_or(|b| score < b.0) {
            best = Some((score, h, r));
        }
    }
    if best.is_none() {
        // Holds near the start of the run not reachable on the graph: any of
        // that runway's holds, nearest the threshold first (an intersection departure).
        for h in holds.iter().filter(|h| h.2 == name) {
            let Some(r) = route_from(&g, p, h.0, &boxes) else { continue };
            let score = r.length + 1.5 * (h.0 - thr).dot(dir).abs() + if r.crosses.contains(&name) { 5000.0 } else { 0.0 };
            if best.as_ref().is_none_or(|b| score < b.0) {
                best = Some((score, taxi::HoldPt { at: h.0, taxiway: taxi::clean_name(&h.1) }, r));
            }
        }
    }
    if std::env::var_os("SKYWARD_GROUND_DEBUG").is_some() {
        eprintln!(
            "ground: plan departure {ap} {name} ({rname}): {} graph nodes, {} holds, {} for this runway, best {:?}",
            g.len(),
            holds.len(),
            holds.iter().filter(|h| h.2 == name).count(),
            best.as_ref().map(|b| (b.1.taxiway.clone(), b.2.length))
        );
    }
    let (_, h, r) = best?;
    Some(Guide::new(r.pts, r.via, format!("holding point {} RWY {}", h.taxiway, rname), true))
}

/// Plan a route from `p` to the nearest free stand big enough for `span` m
/// (after landing). The route ends at the stand's nose-wheel stop point.
pub fn plan_to_stand(world: &World, p: Vec2, span: f32) -> Option<Guide> {
    let (ap, g, boxes, _) = graph_near(world, p)?;
    let l = world.airport_detail.layout(&ap)?;
    let m = l.frame.map_to(&world.proj);
    let parked: Vec<Vec2> = l.parked.iter().map(|x| m.apply(x.at)).collect();
    let (name, at) = l
        .stands
        .iter()
        .filter(|s| s.span + 0.5 >= span)
        .map(|s| (s.name.clone(), m.apply(s.stop)))
        .filter(|(_, at)| !parked.iter().any(|q| q.distance(*at) < 30.0))
        .min_by(|a, b| a.1.distance(p).total_cmp(&b.1.distance(p)))?;
    let r = route_from(&g, p, at, &boxes)?;
    let mut pts = r.pts;
    pts.push(at);
    Some(Guide::new(pts, r.via, format!("stand {name}"), false))
}

/// The stand the aircraft is parked on (nose within 25 m of its stop point,
/// heading within 30°): its stop point and heading.
pub fn on_stand(world: &World, nose: Vec2, hdg: f32) -> Option<(Vec2, f32)> {
    world
        .airport_detail
        .stands(&world.proj)
        .into_iter()
        .filter(|s| s.2.distance(nose) < 25.0 && wrap(s.3 - hdg).abs() < 0.5)
        .min_by(|a, b| a.2.distance(nose).total_cmp(&b.2.distance(nose)))
        .map(|s| (s.2, s.3))
}

/// World position of the nose wheel (or the CG).
pub fn nose_xz(def: &crate::flight::AircraftDef, ac: &crate::flight::Aircraft) -> Vec2 {
    let n = super::pushback::geometry(def).map(|g| g.0).unwrap_or(Vec3::ZERO);
    let p = ac.pos + ac.rot * n;
    vec2(p.x, p.z)
}
