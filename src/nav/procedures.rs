//! Terrain-aware synthetic procedures: departures, climb / descent holds and
//! approaches built from the terrain grid around a field, for any airport.
//!
//! Real instrument procedures (SIDs, STARs, RNP-AR approaches) are not
//! licence-clean worldwide, so the bot flies **synthetic** ones built here —
//! every one marked synthetic — in the spirit of PANS-OPS design:
//!
//! * a [`Grid`] samples the terrain (and buildings) around the field once
//!   (200 m cells, ±40 km) and keeps max-filtered copies so a lateral
//!   protection area is a handful of lookups;
//! * an **arrival** is searched *backwards* from the landing threshold: a
//!   straight final of at least `straight_min`, then fly-by turns no tighter
//!   than the type's approach turn radius, rising at the glidepath angle (3°
//!   up to the type's steepest) until a **holding pattern** fits at that
//!   height (the descent hold / initial approach fix). Blocked straight-ins
//!   give **curved (RNP-AR-like) valley approaches**; a path that has to come
//!   round more than ~100° is the synthetic equivalent of a **circling**
//!   approach to that runway (e.g. Innsbruck 08 flown down the valley from
//!   the east and round);
//! * a **departure** is searched *forwards* from the end of the runway at the
//!   type's planning climb gradient (the one-engine-out gradient for
//!   multi-engine types), valley-following, until it reaches the safe
//!   altitude — or, when no valley leads out low enough, until a **climb
//!   hold** over a safe fix fits;
//! * every segment is checked against the terrain with a required obstacle
//!   clearance (ROC) that grows from the runway out: approach 15 → 75 m on
//!   final (an obstacle-assessment surface near the threshold), 150 m before
//!   the FAF, holds 300 m over a 700 m buffer, departures 35 m + 0.8 % of
//!   the distance flown (PANS-OPS's MOC) up to 300 m.
//!
//! The planner is pure (no world access): the bot hands it a [`Grid`] built
//! from `World::ground_height` / `obstacle_top`, the unit tests a synthetic
//! one.

use glam::{Vec2, vec2};

/// Clearance (m) every point of a hold keeps over the (buffered) terrain.
pub const ROC_HOLD: f32 = 300.0;
/// Buffer (m) around the nominal holding track the terrain is taken over.
pub const HOLD_BUFFER: f32 = 700.0;
/// Buffer (m) the path corridors' lookups cover (a lookup takes the highest
/// of the four nodes around the point: about a cell either way).
const PATH_BUFFER: f32 = 150.0;
/// Path search step (m).
const STEP: f32 = 250.0;
/// Paths kept per search depth.
const BEAM: usize = 48;

// ------------------------------------------------------------------ terrain grid

/// Terrain (incl. obstacles) around a field: `n`×`n` nodes, `cell` m apart,
/// plus max-filtered copies for corridor / hold checks.
#[derive(Clone, Debug)]
pub struct Grid {
    pub lo: Vec2,
    pub cell: f32,
    pub n: usize,
    h: Vec<f32>,
    /// Max over ±PATH_BUFFER.
    hp: Vec<f32>,
    /// Max over ±HOLD_BUFFER.
    hh: Vec<f32>,
    /// Nodes whose height was not known when sampled (streaming).
    pub unknown: usize,
    /// Highest node (used for anything outside the grid).
    pub top: f32,
}

impl Grid {
    /// Sample `height` (None = not known yet) over a square of half-size
    /// `half` around `centre`. Unknown nodes take the highest known height
    /// within 2 km (conservative) — check [`Grid::complete`] and re-plan later.
    pub fn build(mut height: impl FnMut(Vec2) -> Option<f32>, centre: Vec2, half: f32, cell: f32) -> Grid {
        let n = ((2.0 * half / cell).ceil() as usize + 1).max(2);
        let lo = centre - Vec2::splat(half);
        let mut h = vec![f32::NAN; n * n];
        let mut unknown = 0;
        for j in 0..n {
            for i in 0..n {
                let p = lo + vec2(i as f32, j as f32) * cell;
                match height(p) {
                    Some(v) if v.is_finite() => h[j * n + i] = v,
                    _ => unknown += 1,
                }
            }
        }
        let known_top = h.iter().copied().filter(|v| v.is_finite()).fold(f32::MIN, f32::max);
        if unknown > 0 {
            let fill = max_filter(&h, n, (2000.0 / cell).ceil() as usize);
            for k in 0..h.len() {
                if !h[k].is_finite() {
                    h[k] = if fill[k].is_finite() { fill[k] } else { known_top.max(0.0) };
                }
            }
        }
        let top = h.iter().copied().fold(f32::MIN, f32::max);
        let hp = max_filter(&h, n, (PATH_BUFFER / cell).floor() as usize);
        let hh = max_filter(&h, n, (HOLD_BUFFER / cell).ceil() as usize);
        Grid { lo, cell, n, h, hp, hh, unknown, top }
    }

    /// Every node was known when sampled.
    pub fn complete(&self) -> bool {
        self.unknown == 0
    }

    pub fn contains(&self, p: Vec2) -> bool {
        let q = (p - self.lo) / self.cell;
        q.x >= 0.0 && q.y >= 0.0 && q.x <= (self.n - 1) as f32 && q.y <= (self.n - 1) as f32
    }

    /// Conservative lookup: the highest of the four nodes around `p`
    /// (outside the grid: the highest node anywhere).
    fn look(&self, v: &[f32], p: Vec2) -> f32 {
        let q = (p - self.lo) / self.cell;
        if !(q.x >= 0.0 && q.y >= 0.0) {
            return self.top;
        }
        let (i, j) = (q.x.floor() as usize, q.y.floor() as usize);
        if i + 1 >= self.n || j + 1 >= self.n {
            return self.top;
        }
        let k = j * self.n + i;
        v[k].max(v[k + 1]).max(v[k + self.n]).max(v[k + self.n + 1])
    }

    /// Terrain height at `p` (conservative: highest surrounding node).
    pub fn at(&self, p: Vec2) -> f32 {
        self.look(&self.h, p)
    }

    /// Highest terrain within ~PATH_BUFFER of `p`.
    pub fn near(&self, p: Vec2) -> f32 {
        self.look(&self.hp, p)
    }

    /// Highest terrain within ~HOLD_BUFFER of `p`.
    pub fn wide(&self, p: Vec2) -> f32 {
        self.look(&self.hh, p)
    }

    /// Highest terrain within `r` of `c` (a sector / minimum safe altitude).
    pub fn max_within(&self, c: Vec2, r: f32) -> f32 {
        let mut top = self.at(c);
        let k = (r / self.cell).ceil() as i32;
        let q = ((c - self.lo) / self.cell).round();
        for dj in -k..=k {
            for di in -k..=k {
                let (i, j) = (q.x as i32 + di, q.y as i32 + dj);
                if i < 0 || j < 0 || i >= self.n as i32 || j >= self.n as i32 {
                    continue;
                }
                if ((di * di + dj * dj) as f32).sqrt() * self.cell <= r {
                    top = top.max(self.h[j as usize * self.n + i as usize]);
                }
            }
        }
        top
    }

    /// Combine with another grid over the same nodes (the higher of the two).
    pub fn max_with(&mut self, o: &Grid) {
        if o.n != self.n || o.lo.distance(self.lo) > 1.0 {
            return;
        }
        for (a, b) in [(&mut self.h, &o.h), (&mut self.hp, &o.hp), (&mut self.hh, &o.hh)] {
            for (x, y) in a.iter_mut().zip(b.iter()) {
                *x = x.max(*y);
            }
        }
        self.top = self.top.max(o.top);
    }

    /// Re-express the grid after a floating-origin shift (rigid: the grid is
    /// re-sampled whenever a plan is made, this keeps lookups valid in between).
    pub fn shift(&mut self, d: Vec2) {
        self.lo += d;
    }
}

/// Square max filter of radius `r` nodes (separable).
fn max_filter(h: &[f32], n: usize, r: usize) -> Vec<f32> {
    let pass = |src: &[f32], horiz: bool| -> Vec<f32> {
        let mut out = vec![f32::NAN; src.len()];
        for a in 0..n {
            for b in 0..n {
                let mut m = f32::NAN;
                for k in b.saturating_sub(r)..=(b + r).min(n - 1) {
                    let v = if horiz { src[a * n + k] } else { src[k * n + a] };
                    if v.is_finite() && (m.is_nan() || v > m) {
                        m = v;
                    }
                }
                if horiz {
                    out[a * n + b] = m;
                } else {
                    out[b * n + a] = m;
                }
            }
        }
        out
    };
    pass(&pass(h, true), false)
}

// ------------------------------------------------------------------ inputs / outputs

/// What the planner needs to know about the aircraft.
#[derive(Clone, Debug)]
pub struct Perf {
    /// Turn radius (m) on the approach (fly-by / RF turns) and climbing out.
    pub app_turn_r: f32,
    pub dep_turn_r: f32,
    /// Holding pattern: turn radius and inbound leg length (m).
    pub hold_r: f32,
    pub hold_leg: f32,
    /// Glidepath angles to try (rad), shallowest first.
    pub min_gp: f32,
    pub max_gp: f32,
    /// Minimum straight final (m from the aim point) and preferred FAF distance.
    pub straight_min: f32,
    pub final_dist: f32,
    /// Planning climb gradient (rise / run; the engine-out one for multi-engine types).
    pub climb_grad: f32,
    /// All-engines gradient (reported, and the check of the planning one).
    pub climb_grad_aeo: f32,
    /// Planned on one engine out.
    pub oei: bool,
    /// Longest path searched (m).
    pub max_len: f32,
}

/// A racetrack holding pattern: inbound along `inbound` to `fix`, turns to
/// the `side` (+1 right, −1 left), turn radius `r`, straight legs `leg`.
#[derive(Clone, Debug)]
pub struct Hold {
    pub fix: Vec2,
    pub inbound: Vec2,
    pub side: f32,
    pub r: f32,
    pub leg: f32,
    /// Lowest altitude (m) the hold clears the terrain at.
    pub min_alt: f32,
}

impl Hold {
    fn right(&self) -> Vec2 {
        right_of(self.inbound)
    }

    /// Points along the nominal holding track (`step` m apart).
    pub fn track(&self, step: f32) -> Vec<Vec2> {
        let n = (self.lap() / step).ceil().max(8.0) as usize;
        (0..n).map(|k| self.point(self.lap() * k as f32 / n as f32)).collect()
    }

    /// Lowest altitude at which this hold clears the terrain by [`ROC_HOLD`].
    pub fn need(&self, g: &Grid) -> f32 {
        self.track(150.0).iter().map(|p| g.wide(*p)).fold(f32::MIN, f32::max) + ROC_HOLD
    }

    /// Length of one lap of the nominal track.
    pub fn lap(&self) -> f32 {
        2.0 * self.leg + 2.0 * std::f32::consts::PI * self.r
    }

    /// Point `s` m round the lap from the fix (turn at the fix, outbound,
    /// turn at the far end, inbound to the fix).
    pub fn point(&self, s: f32) -> Vec2 {
        let pi = std::f32::consts::PI;
        let (u, r, side) = (self.inbound, self.r, self.side);
        let right = self.right();
        let c1 = self.fix + right * side * r;
        let c2 = c1 - u * self.leg;
        let h0 = heading(u);
        let on = |c: Vec2, h: f32| c - vec2(h.cos(), h.sin()) * side * r;
        let s = s.rem_euclid(self.lap());
        let arc = pi * r;
        if s < arc {
            on(c1, h0 + side * s / r)
        } else if s < arc + self.leg {
            self.fix + right * side * 2.0 * r - u * (s - arc)
        } else if s < 2.0 * arc + self.leg {
            on(c2, h0 + side * (pi + (s - arc - self.leg) / r))
        } else {
            self.fix - u * (self.leg - (s - 2.0 * arc - self.leg))
        }
    }

    /// Where `p` is round the lap: (s, distance from the nominal track).
    pub fn closest(&self, p: Vec2) -> (f32, f32) {
        let pi = std::f32::consts::PI;
        let (u, r, side) = (self.inbound, self.r, self.side);
        let right = self.right();
        let c1 = self.fix + right * side * r;
        let c2 = c1 - u * self.leg;
        let h0 = heading(u);
        let arc = pi * r;
        let mut best = (0.0, f32::MAX);
        let mut take = |s: f32, d: f32| {
            if d < best.1 {
                best = (s, d);
            }
        };
        // Arcs: the heading at which the arc passes nearest p.
        for (c, hs, s0) in [(c1, h0, 0.0), (c2, h0 + side * pi, arc + self.leg)] {
            let v = p - c;
            let n = v.normalize_or(-right * side);
            let h = (-side * n.y).atan2(-side * n.x);
            let a = (side * wrap(h - hs)).rem_euclid(2.0 * pi);
            if a <= pi {
                take(s0 + a * r, (v.length() - r).abs());
            }
        }
        // Legs (and the arc ends).
        for (a, dir, s0) in [(self.fix + right * side * 2.0 * r, -u, arc), (self.fix - u * self.leg, u, 2.0 * arc + self.leg)] {
            let t = (p - a).dot(dir).clamp(0.0, self.leg);
            take(s0 + t, p.distance(a + dir * t));
        }
        best
    }

    /// Guidance round the racetrack (pure pursuit): the track (heading, rad)
    /// to fly from `p`, and the distance off the nominal track (m).
    pub fn steer(&self, p: Vec2) -> (f32, f32) {
        let (s, d) = self.closest(p);
        let look = 0.8 * self.r + 300.0 + 0.5 * d;
        let c = self.point(s + look);
        (heading(c - p), d)
    }

    /// On the inbound leg, heading for the fix (within `tol` m of the leg).
    pub fn inbound_at(&self, p: Vec2, hdg: f32, tol: f32) -> bool {
        let along = (p - self.fix).dot(self.inbound);
        let cross = (p - self.fix).dot(self.right());
        along < 0.0 && along > -self.leg - 300.0 && cross.abs() < tol && wrap(hdg - heading(self.inbound)).abs() < 0.5
    }
}

/// The kind of approach the arrival search found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArrKind {
    /// Straight in from the hold fix.
    Straight,
    /// Curved valley approach (RNP-AR-like fly-by / RF turns).
    Curved,
    /// Comes round more than ~100° on the way down: the synthetic equivalent
    /// of a circling / visual manoeuvre to this runway.
    Circling,
}

/// A planned approach: from the hold fix (`path[0]`) to the aim point.
#[derive(Clone, Debug)]
pub struct Arrival {
    /// Aim point (m past the runway end) of the glidepath.
    pub aim: f32,
    pub path: Vec<Vec2>,
    /// Path altitude (m) at each point: the glidepath from the aim point.
    pub alt: Vec<f32>,
    /// Distance (m) to the aim point along the path at each point.
    pub dist: Vec<f32>,
    pub gp: f32,
    pub hold: Hold,
    pub kind: ArrKind,
    /// Length (m) of the straight final (to the aim point).
    pub straight: f32,
    /// Total heading change along the path (rad).
    pub turn: f32,
    /// Smallest margin (m) over the required clearance anywhere on the path.
    pub margin: f32,
    /// The terrain was fully known when planned.
    pub complete: bool,
    /// Always true: no published procedure is used.
    pub synthetic: bool,
}

/// A planned departure: from the runway end along `path`, climbing.
#[derive(Clone, Debug)]
pub struct Departure {
    pub path: Vec<Vec2>,
    /// Minimum altitude (m) at each point (the planning gradient).
    pub alt: Vec<f32>,
    /// Climb in this hold (at the end of the path) up to `target` first.
    pub hold: Option<Hold>,
    pub target: f32,
    pub grad: f32,
    pub oei: bool,
    pub turn: f32,
    pub margin: f32,
    pub complete: bool,
    pub synthetic: bool,
}

/// Where along a path an aircraft is.
#[derive(Clone, Copy, Debug)]
pub struct PathPos {
    /// Segment index (from `path[i]` to `path[i+1]`).
    pub i: usize,
    /// Cross-track error (m, + = right of the path).
    pub cross: f32,
    /// Track of the segment (rad).
    pub track: f32,
    /// Distance flown along the path from its start (m).
    pub along: f32,
    /// Fraction along segment `i`.
    pub t: f32,
}

/// Locate `p` on polyline `path`, searching segments from `hint` on (the
/// path is flown in order; it never jumps back).
pub fn locate(path: &[Vec2], p: Vec2, hint: usize) -> PathPos {
    let mut best: Option<(f32, PathPos)> = None;
    let mut acc = 0.0;
    for i in 0..path.len().saturating_sub(1) {
        let (a, b) = (path[i], path[i + 1]);
        let ab = b - a;
        let len = ab.length().max(1e-3);
        let u = ab / len;
        if i >= hint && i <= hint + 12 {
            let t = ((p - a).dot(u) / len).clamp(0.0, 1.0);
            let q = a + ab * t;
            let d = p.distance(q);
            // (Beyond the end of the last segment: keep projecting along it.)
            let last = i + 2 == path.len();
            let t_raw = (p - a).dot(u) / len;
            let tt = if last { t_raw.max(0.0) } else { t };
            let cross = (p - a).dot(right_of(u));
            if best.is_none_or(|bb| d < bb.0 - 1.0) {
                best = Some((d, PathPos { i, cross, track: heading(u), along: acc + len * tt, t: tt }));
            }
        }
        acc += len;
    }
    best.map(|b| b.1).unwrap_or(PathPos { i: 0, cross: 0.0, track: 0.0, along: 0.0, t: 0.0 })
}

/// Length of a polyline.
pub fn length(path: &[Vec2]) -> f32 {
    path.windows(2).map(|w| w[0].distance(w[1])).sum()
}

// ------------------------------------------------------------------ geometry helpers

/// Heading (0 = north, +π/2 = east) of a world XZ direction (−z = north).
pub fn heading(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

pub fn dir(h: f32) -> Vec2 {
    vec2(h.sin(), -h.cos())
}

/// Right of a direction (looking along it, seen from above).
pub fn right_of(d: Vec2) -> Vec2 {
    vec2(-d.y, d.x)
}

pub fn wrap(a: f32) -> f32 {
    let mut d = a.rem_euclid(std::f32::consts::TAU);
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    d
}

// ------------------------------------------------------------------ required clearance

/// Approach ROC (m) at `d` m from the aim point along the path:
/// an obstacle-assessment surface near the runway, 75 m on final, 150 m
/// beyond the FAF.
pub fn roc_approach(d: f32, faf: f32) -> f32 {
    if d < faf { (0.025 * d).clamp(5.0, 75.0) } else { 75.0 + ((d - faf) / 3000.0).clamp(0.0, 1.0) * 75.0 }
}

/// Departure ROC (m) `d` m past the lift-off point: 5 m + 0.8 % (PANS-OPS
/// minimum obstacle clearance), at least 90 m once 10 km out (turns), at
/// most 300 m.
pub fn roc_departure(d: f32) -> f32 {
    (5.0 + 0.008 * d).max(if d > 10_000.0 { 90.0 } else { 0.0 }).min(300.0)
}

/// Corridor half-width (m) `d` m out (the grid lookups cover ~PATH_BUFFER of
/// it): from the runway width out to ~1 km (the bot tracks to tens of
/// metres; a splay of 8 % stands in for RNP 0.3 containment).
fn half_width(d: f32) -> f32 {
    (75.0 + 0.08 * d).min(1000.0)
}

/// Highest terrain across the corridor at `p` (track `u`, `d` m out).
fn corridor(g: &Grid, p: Vec2, u: Vec2, d: f32) -> f32 {
    let w = (half_width(d) - PATH_BUFFER).max(0.0);
    let r = right_of(u);
    let mut top = g.near(p);
    if w > 1.0 {
        let n = (w / 200.0).ceil() as i32;
        for k in 1..=n {
            let o = r * (w * k as f32 / n as f32);
            top = top.max(g.near(p + o)).max(g.near(p - o));
        }
    }
    top
}

// ------------------------------------------------------------------ beam search

#[derive(Clone, Copy, Debug)]
struct Node {
    p: Vec2,
    /// Direction of travel of the search (away from the runway for arrivals).
    u: Vec2,
    /// Distance from the search origin (m).
    d: f32,
    /// Remaining straight distance before a turn is allowed.
    straight: f32,
    turn: f32,
    cost: f32,
    /// `cost` plus the look-ahead penalty (beam ranking).
    rank: f32,
    margin: f32,
    parent: usize,
}

struct Search<'a> {
    g: &'a Grid,
    turn_r: f32,
    /// Altitude of the path at distance d.
    alt: &'a dyn Fn(f32) -> f32,
    roc: &'a dyn Fn(f32) -> f32,
    max_len: f32,
    /// Heading changes (rad) beyond which a path is dropped.
    max_turn: f32,
}

/// Look-ahead distance (m) for the beam ranking.
const LOOK: f32 = 4000.0;

impl Search<'_> {
    /// Free distance (m, up to [`LOOK`]) ahead of a node: the best of three
    /// rays (straight, ±30°), the path still climbing / descending as planned
    /// — so the beam turns into the open valley before the walls close in.
    fn room(&self, p: Vec2, u: Vec2, d: f32) -> f32 {
        let mut best: f32 = 0.0;
        for a in [0.0f32, 0.52, -0.52] {
            let (s, c) = a.sin_cos();
            let v = vec2(u.x * c - u.y * s, u.x * s + u.y * c);
            let mut free = LOOK;
            let mut k = 500.0;
            while k <= LOOK {
                let q = p + v * k;
                if (self.alt)(d + k) - self.g.near(q) - (self.roc)(d + k) < 0.0 {
                    free = k - 500.0;
                    break;
                }
                k += 500.0;
            }
            best = best.max(free);
            if best >= LOOK {
                break;
            }
        }
        best
    }

    /// Beam search from `start` along `u0` (straight for `straight0` m);
    /// `done(node, depth)` returns Some(score) when the node ends a path.
    fn run(&self, start: Vec2, u0: Vec2, straight0: f32, done: &mut dyn FnMut(&Node) -> Option<f32>) -> Option<(Vec<Node>, usize)> {
        let mut arena: Vec<Node> = vec![Node { p: start, u: u0, d: 0.0, straight: straight0, turn: 0.0, cost: 0.0, rank: 0.0, margin: f32::MAX, parent: usize::MAX }];
        let mut front = vec![0usize];
        let dpsi = STEP / self.turn_r;
        let mut best: Option<(f32, usize)> = None;
        let mut depth = 0;
        while !front.is_empty() && (depth as f32) * STEP < self.max_len {
            depth += 1;
            let mut next: Vec<Node> = Vec::new();
            for &fi in &front {
                let f = arena[fi];
                let opts: &[f32] = if f.straight > 0.0 { &[0.0] } else { &[0.0, 0.5, -0.5, 1.0, -1.0] };
                for &k in opts {
                    let a = dpsi * k;
                    let (s, c) = a.sin_cos();
                    let u = vec2(f.u.x * c - f.u.y * s, f.u.x * s + f.u.y * c).normalize();
                    // (Chord of the turn: the mid heading.)
                    let um = (f.u + u).normalize_or(u);
                    let p = f.p + um * STEP;
                    let d = f.d + STEP;
                    let turn = f.turn + a.abs();
                    if turn > self.max_turn {
                        continue;
                    }
                    let top = corridor(self.g, p, um, d);
                    let m = (self.alt)(d) - top - (self.roc)(d);
                    if m < 0.0 {
                        continue;
                    }
                    // Cost: distance, turns, and a little for hugging the terrain
                    // (stay mid-valley when there is room).
                    let hug = (200.0 - m).max(0.0) * 1.5;
                    let cost = f.cost + STEP + a.abs() * 2500.0 + hug;
                    let rank = cost + (LOOK - self.room(p, u, d)) * 1.2;
                    next.push(Node { p, u, d, straight: f.straight - STEP, turn, cost, rank, margin: f.margin.min(m), parent: fi });
                }
            }
            // Keep the best few, spread out (no two within 60 m and 5°).
            next.sort_by(|a, b| a.rank.total_cmp(&b.rank));
            let mut kept: Vec<Node> = Vec::new();
            for nd in next {
                if kept.len() >= BEAM {
                    break;
                }
                if kept.iter().any(|k| k.p.distance(nd.p) < 60.0 && k.u.dot(nd.u) > 0.996) {
                    continue;
                }
                kept.push(nd);
            }
            if std::env::var("SKYWARD_PROC_DEBUG").is_ok() {
                if let Some(k) = kept.first() {
                    eprintln!(
                        "depth {depth} kept {} best p {:?} d {} turn {:.0} alt {:.0} top {:.0}",
                        kept.len(),
                        k.p,
                        k.d,
                        k.turn.to_degrees(),
                        (self.alt)(k.d),
                        corridor(self.g, k.p, k.u, k.d)
                    );
                } else {
                    eprintln!("depth {depth}: nothing left");
                }
            }
            front.clear();
            for nd in kept {
                arena.push(nd);
                let id = arena.len() - 1;
                if let Some(score) = done(&nd) {
                    if best.is_none_or(|b| score < b.0) {
                        best = Some((score, id));
                    }
                } else {
                    front.push(id);
                }
            }
            // The first depth that ends a path wins (shortest), unless a
            // cheaper one comes within a few more steps.
            if let Some((_, id)) = best {
                if arena[id].d + 4.0 * STEP <= depth as f32 * STEP {
                    break;
                }
            }
        }
        let (_, id) = best?;
        let mut chain = Vec::new();
        let mut k = id;
        while k != usize::MAX {
            chain.push(arena[k]);
            k = arena[k].parent;
        }
        chain.reverse();
        Some((chain, id))
    }
}

/// Best hold at node `n` (inbound towards the runway, i.e. opposite `n.u`):
/// the side needing the lower altitude.
fn hold_at(g: &Grid, p: Vec2, inbound: Vec2, perf: &Perf) -> Hold {
    let mk = |side: f32, off: f32| {
        let fix = p - right_of(inbound) * side * off;
        let mut h = Hold { fix, inbound, side, r: perf.hold_r, leg: perf.hold_leg, min_alt: 0.0 };
        h.min_alt = h.need(g);
        h
    };
    let (a, b) = (mk(1.0, 0.0), mk(-1.0, 0.0));
    if b.min_alt + 1.0 < a.min_alt { b } else { a }
}

/// A climb hold at `p`: as [`hold_at`], or centred on the track (its
/// racetrack straddling the valley axis the departure follows), whichever
/// clears lower.
fn climb_hold_at(g: &Grid, p: Vec2, inbound: Vec2, perf: &Perf) -> Hold {
    let mut best = hold_at(g, p, inbound, perf);
    for side in [1.0, -1.0] {
        let fix = p - right_of(inbound) * side * perf.hold_r;
        let mut h = Hold { fix, inbound, side, r: perf.hold_r, leg: perf.hold_leg, min_alt: 0.0 };
        h.min_alt = h.need(g);
        if h.min_alt + 1.0 < best.min_alt {
            best = h;
        }
    }
    best
}

// ------------------------------------------------------------------ arrival

/// Plan an approach to the runway end `thr` (landing direction `rdir`,
/// elevation `elev`): the aim point lies `disp` (displaced threshold) plus
/// the distance that crosses the threshold at `tch` m (at most `aim_max`)
/// beyond it.
pub fn plan_arrival(g: &Grid, thr: Vec2, rdir: Vec2, elev: f32, disp: f32, aim_max: f32, tch: f32, perf: &Perf) -> Option<Arrival> {
    let mut best: Option<(f32, Arrival)> = None;
    let steps = ((perf.max_gp - perf.min_gp) / 0.25f32.to_radians()).round().max(0.0) as usize;
    for k in 0..=steps {
        let gp = perf.min_gp + k as f32 * 0.25f32.to_radians();
        let tan = gp.tan();
        let aim = disp + (tch / tan).min(aim_max);
        // Search from the threshold outwards; d measured from the threshold.
        let alt = move |d: f32| elev + (d + aim) * tan;
        let faf = perf.final_dist;
        let roc = move |d: f32| roc_approach(d + aim, faf);
        // Straight final: at least `straight_min` from the aim point.
        let straight0 = (perf.straight_min - aim).max(STEP);
        let s = Search { g, turn_r: perf.app_turn_r, alt: &alt, roc: &roc, max_len: perf.max_len, max_turn: 200f32.to_radians() };
        let min_len = (perf.final_dist - aim).max(straight0);
        let mut done = |n: &Node| -> Option<f32> {
            if n.d < min_len {
                return None;
            }
            // (Check the hold every other step: it's the costly bit.)
            if ((n.d / STEP).round() as i32) % 2 != 0 {
                return None;
            }
            let h = hold_at(g, n.p, -n.u, perf);
            (h.min_alt <= alt(n.d) + 1.0).then_some(n.cost)
        };
        let Some((chain, _)) = s.run(thr, -rdir, straight0, &mut done) else { continue };
        let last = chain.last().unwrap();
        let hold = hold_at(g, last.p, -last.u, perf);
        // From the fix to the threshold.
        let path: Vec<Vec2> = chain.iter().rev().map(|n| n.p).collect();
        let dist: Vec<f32> = chain.iter().rev().map(|n| n.d + aim).collect();
        let alt_v: Vec<f32> = dist.iter().map(|d| elev + d * tan).collect();
        let turn = last.turn;
        // Straight part next to the runway.
        let straight = chain.iter().take_while(|n| n.u.dot(-rdir) > 0.999).last().map_or(0.0, |n| n.d + aim);
        let kind = if turn < 12f32.to_radians() {
            ArrKind::Straight
        } else if turn < 100f32.to_radians() {
            ArrKind::Curved
        } else {
            ArrKind::Circling
        };
        // Preference: straight-in over curved, shallow over steep, short over long.
        let score = last.d + 2500.0 * turn + 3000.0 * (gp.to_degrees() - 3.0).max(0.0) + if kind == ArrKind::Straight { 0.0 } else { 3000.0 };
        let arr = Arrival {
            aim,
            path,
            alt: alt_v,
            dist,
            gp,
            hold,
            kind,
            straight,
            turn,
            margin: chain.iter().skip(1).map(|n| n.margin).fold(f32::MAX, f32::min),
            complete: g.complete(),
            synthetic: true,
        };
        if best.as_ref().is_none_or(|b| score < b.0) {
            best = Some((score, arr));
        }
    }
    best.map(|b| b.1)
}

// ------------------------------------------------------------------ departure

/// Plan a departure from runway end `der` (direction of take-off `rdir`,
/// end elevation `elev`; lift-off `lift_back` m before the end), climbing
/// to `target` (m).
pub fn plan_departure(g: &Grid, der: Vec2, rdir: Vec2, elev: f32, lift_back: f32, target: f32, perf: &Perf) -> Option<Departure> {
    let grad = perf.climb_grad;
    // Screen height at the runway end, then the planning gradient from lift-off.
    let alt = move |d: f32| elev + 10.0 + (d + lift_back) * grad;
    let roc = move |d: f32| roc_departure(d + lift_back);
    // Straight ahead to 400 ft above the runway first (no turns below it).
    let straight0 = ((120.0 / grad.max(0.01)) - lift_back).max(STEP);
    let s = Search { g, turn_r: perf.dep_turn_r, alt: &alt, roc: &roc, max_len: perf.max_len, max_turn: 270f32.to_radians() };
    let mut hold_cand: Option<(f32, Node)> = None;
    let mut done = |n: &Node| -> Option<f32> {
        if alt(n.d) >= target {
            return Some(n.cost);
        }
        // A climb hold here? (Every other step, past the straight segment.)
        if n.straight <= 0.0 && ((n.d / STEP).round() as i32) % 2 == 0 && hold_cand.is_none() {
            let h = climb_hold_at(g, n.p + n.u * perf.hold_leg * 0.5, n.u, perf);
            if h.min_alt <= alt(n.d) + 1.0 {
                hold_cand = Some((n.cost, *n));
            }
        }
        None
    };
    let found = s.run(der, rdir, straight0, &mut done);
    let (chain, hold) = match found {
        Some((chain, _)) => (chain, None),
        None => {
            // No valley out at the planning gradient: climb in the earliest hold that fits.
            let (_, n) = hold_cand?;
            // Rebuild the chain to that node by searching again up to its distance.
            let max_d = n.d;
            let target_p = n.p;
            let mut done2 = |m: &Node| -> Option<f32> { (m.d >= max_d && m.p.distance(target_p) < 1.0).then_some(m.cost) };
            let s2 = Search { max_len: max_d + STEP, ..s };
            let (chain, _) = s2.run(der, rdir, straight0, &mut done2)?;
            let last = *chain.last().unwrap();
            let h = climb_hold_at(g, last.p + last.u * perf.hold_leg * 0.5, last.u, perf);
            (chain, Some(h))
        }
    };
    let last = *chain.last().unwrap();
    Some(Departure {
        path: chain.iter().map(|n| n.p).collect(),
        alt: chain.iter().map(|n| alt(n.d)).collect(),
        hold,
        target,
        grad,
        oei: perf.oei,
        turn: last.turn,
        margin: chain.iter().skip(1).map(|n| n.margin).fold(f32::MAX, f32::min),
        complete: g.complete(),
        synthetic: true,
    })
}

/// Highest terrain within ~HOLD_BUFFER of the straight line `a` → `b`
/// (inside the grid; the part outside counts as the grid's highest node).
pub fn line_floor(g: &Grid, a: Vec2, b: Vec2) -> f32 {
    let n = (a.distance(b) / 150.0).ceil().max(1.0) as usize;
    (0..=n).map(|k| g.wide(a.lerp(b, k as f32 / n as f32))).fold(f32::MIN, f32::max)
}

/// Lowest safe altitude within `r` of `c`: highest terrain + 300 m.
pub fn msa(g: &Grid, c: Vec2, r: f32) -> f32 {
    g.max_within(c, r) + 300.0
}

/// Smallest margin (m) of a path over terrain + `roc(d)` (d = distance
/// along from its start), sampled every 50 m on the centreline. For tests
/// and the soak report.
pub fn path_margin(g: &Grid, path: &[Vec2], alt: &[f32], roc: impl Fn(f32) -> f32) -> f32 {
    let mut m = f32::MAX;
    let mut acc = 0.0;
    for i in 0..path.len().saturating_sub(1) {
        let (a, b) = (path[i], path[i + 1]);
        let len = a.distance(b);
        let n = (len / 50.0).ceil().max(1.0) as usize;
        for k in 0..=n {
            let t = k as f32 / n as f32;
            let p = a.lerp(b, t);
            let h = alt[i] + (alt[i + 1] - alt[i]) * t;
            m = m.min(h - g.at(p) - roc(acc + len * t));
        }
        acc += len;
    }
    m
}

// ------------------------------------------------------------------ valley routes

/// A low route from `a` to `b` over the grid (A* on a coarse lattice,
/// cost = distance + climbing above the lower end), for helicopter tours.
/// Returns waypoints (a … b), simplified.
pub fn valley_route(g: &Grid, a: Vec2, b: Vec2, cell: f32) -> Vec<Vec2> {
    use std::cmp::Ordering;
    use std::collections::{BinaryHeap, HashMap};
    #[derive(PartialEq)]
    struct Q(f32, (i32, i32));
    impl Eq for Q {}
    impl PartialOrd for Q {
        fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
            Some(self.cmp(o))
        }
    }
    impl Ord for Q {
        fn cmp(&self, o: &Self) -> Ordering {
            o.0.total_cmp(&self.0)
        }
    }
    let base = g.at(a).min(g.at(b));
    let key = |p: Vec2| (((p.x - a.x) / cell).round() as i32, ((p.y - a.y) / cell).round() as i32);
    let pos = |k: (i32, i32)| a + vec2(k.0 as f32, k.1 as f32) * cell;
    let goal = key(b);
    let lim = (a.distance(b) / cell).ceil() as i32 + 12;
    let mut open = BinaryHeap::new();
    let mut gcost: HashMap<(i32, i32), f32> = HashMap::new();
    let mut from: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
    gcost.insert((0, 0), 0.0);
    open.push(Q(0.0, (0, 0)));
    let mut n = 0;
    while let Some(Q(_, k)) = open.pop() {
        n += 1;
        if k == goal || n > 60_000 {
            break;
        }
        let gk = gcost[&k];
        for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
            let nk = (k.0 + di, k.1 + dj);
            if nk.0.abs() > lim || nk.1.abs() > lim {
                continue;
            }
            let p = pos(nk);
            if !g.contains(p) {
                continue;
            }
            let step = cell * ((di * di + dj * dj) as f32).sqrt();
            let rise = (g.near(p) - base).max(0.0);
            let c = gk + step * (1.0 + rise / 150.0);
            if gcost.get(&nk).is_none_or(|&o| c < o) {
                gcost.insert(nk, c);
                from.insert(nk, k);
                let h = p.distance(b);
                open.push(Q(c + h, nk));
            }
        }
    }
    if !from.contains_key(&goal) {
        return vec![a, b];
    }
    let mut pts = vec![b];
    let mut k = goal;
    while let Some(&pk) = from.get(&k) {
        pts.push(pos(pk));
        k = pk;
    }
    pts.reverse();
    pts[0] = a;
    // Simplify: drop points that sit (nearly) on the line between neighbours.
    let mut out = vec![pts[0]];
    for i in 1..pts.len() - 1 {
        let prev = *out.last().unwrap();
        let (u, v) = ((pts[i] - prev).normalize_or_zero(), (pts[i + 1] - pts[i]).normalize_or_zero());
        if u.dot(v) < 0.995 || pts[i].distance(prev) > 3000.0 {
            out.push(pts[i]);
        }
    }
    out.push(b);
    out
}

/// Relief (m) around `c`: highest minus lowest terrain within `r`.
pub fn relief(g: &Grid, c: Vec2, r: f32) -> f32 {
    let k = (r / g.cell).ceil() as i32;
    let q = ((c - g.lo) / g.cell).round();
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for dj in -k..=k {
        for di in -k..=k {
            let (i, j) = (q.x as i32 + di, q.y as i32 + dj);
            if i < 0 || j < 0 || i >= g.n as i32 || j >= g.n as i32 || ((di * di + dj * dj) as f32).sqrt() * g.cell > r {
                continue;
            }
            let h = g.h[j as usize * g.n + i as usize];
            lo = lo.min(h);
            hi = hi.max(h);
        }
    }
    (hi - lo).max(0.0)
}

/// A sightseeing loop through the valleys around `home`: in each of `n`
/// sectors the lowest ground `r_min`..`r_max` away (a valley, not a peak),
/// joined by low routes ([`valley_route`]) and back home. Waypoints in order
/// (home excluded), the sector points flagged true.
pub fn valley_tour(g: &Grid, home: Vec2, n: usize, r_min: f32, r_max: f32, seed: u32) -> Vec<(Vec2, bool)> {
    let tau = std::f32::consts::TAU;
    let off = (seed % 7) as f32 * 0.37;
    let mut pts = Vec::new();
    for k in 0..n {
        let mut best: Option<(f32, Vec2)> = None;
        for a in 0..8 {
            let ang = off + (k as f32 + a as f32 / 8.0) * tau / n as f32;
            let mut r = r_min;
            while r <= r_max {
                let q = home + dir(ang) * r;
                if g.contains(q) {
                    // Low ground, a little bonus for distance (a real outing).
                    let score = g.at(q) - r * 0.01;
                    if best.is_none_or(|b| score < b.0) {
                        best = Some((score, q));
                    }
                }
                r += 500.0;
            }
        }
        if let Some((_, q)) = best {
            pts.push(q);
        }
    }
    let mut out = Vec::new();
    let mut prev = home;
    for q in pts.iter().copied().chain(std::iter::once(home)) {
        let route = valley_route(g, prev, q, 300.0);
        for (i, w) in route.iter().enumerate().skip(1) {
            out.push((*w, i + 1 == route.len() && q != home));
        }
        prev = q;
    }
    // (The final point is home itself: the caller lands there.)
    out.pop();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLAT: f32 = 600.0;

    /// A valley: floor at 600 m along the x axis for x < 0, 2,600 m walls
    /// either side (floor half-width 3 km at the bottom, 30° slopes), which
    /// bends north (−z) at x = 0; open beyond 20 km east.
    fn valley(p: Vec2) -> f32 {
        // Distance from the axis {(x, 0): x <= 0} ∪ {(0, z): z <= 0}.
        let d1 = if p.x <= 0.0 { p.y.abs() } else { p.length() };
        let d2 = if p.y <= 0.0 { p.x.abs() } else { p.length() };
        let wall = ((d1.min(d2) - 3000.0).max(0.0) * 0.58).min(2000.0);
        FLAT + wall
    }

    fn perf() -> Perf {
        Perf {
            app_turn_r: 1600.0,
            dep_turn_r: 2000.0,
            hold_r: 1800.0,
            hold_leg: 5000.0,
            min_gp: 3f32.to_radians(),
            max_gp: 4.5f32.to_radians(),
            straight_min: 3500.0,
            final_dist: 9000.0,
            climb_grad: 0.033,
            climb_grad_aeo: 0.08,
            oei: true,
            max_len: 45_000.0,
        }
    }

    #[test]
    fn flat_field_is_a_straight_in() {
        let g = Grid::build(|_| Some(FLAT), Vec2::ZERO, 25_000.0, 200.0);
        let a = plan_arrival(&g, vec2(0.0, 0.0), vec2(1.0, 0.0), FLAT, 0.0, 300.0, 15.5, &perf()).expect("plan");
        assert_eq!(a.kind, ArrKind::Straight);
        assert!((a.gp.to_degrees() - 3.0).abs() < 0.01, "gp {}", a.gp.to_degrees());
        // The hold fix is about the FAF distance out, on the extended centreline.
        assert!(a.path[0].y.abs() < 1.0 && a.path[0].x < -8000.0 && a.path[0].x > -10_500.0, "fix {:?}", a.path[0]);
        let d = plan_departure(&g, vec2(2000.0, 0.0), vec2(1.0, 0.0), FLAT, 800.0, FLAT + 900.0, &perf()).expect("departure");
        assert!(d.hold.is_none() && d.turn < 0.01);
    }

    #[test]
    fn valley_paths_clear_the_terrain() {
        let g = Grid::build(|p| Some(valley(p)), vec2(0.0, -10_000.0), 40_000.0, 200.0);
        let pf = perf();
        // Runway on the valley floor at x ∈ [-6000, -4000], landing east (towards the bend).
        let thr = vec2(-6000.0, 0.0);
        let a = plan_arrival(&g, thr, vec2(1.0, 0.0), FLAT, 0.0, 300.0, 15.5, &pf).expect("arrival");
        let roc = |d: f32| roc_approach(a.dist[0] - d, pf.final_dist).min(75.0);
        let m = path_margin(&g, &a.path, &a.alt, roc);
        assert!(m > -5.0, "arrival margin {m}");
        // Its hold clears the terrain where it is flown.
        assert!(a.hold.min_alt <= a.alt[0] + 1.0 && a.hold.need(&g) <= a.alt[0] + 1.0);
        // Landing west (from the bend): must come round the corner of the valley.
        let thr_w = vec2(-4000.0, 0.0);
        let b = plan_arrival(&g, thr_w, vec2(-1.0, 0.0), FLAT, 0.0, 300.0, 15.5, &pf).expect("arrival west");
        let m = path_margin(&g, &b.path, &b.alt, |d| roc_approach(b.dist[0] - d, pf.final_dist).min(75.0));
        assert!(m > -5.0, "arrival west margin {m}");
        // Departures both ways: never below terrain + ROC, gradient as planned.
        for (der, dir) in [(vec2(-4000.0, 0.0), vec2(1.0, 0.0)), (vec2(-6000.0, 0.0), vec2(-1.0, 0.0))] {
            let d = plan_departure(&g, der, dir, FLAT, 800.0, 3300.0, &pf).expect("departure");
            let m = path_margin(&g, &d.path, &d.alt, |x| roc_departure(x + 800.0) * 0.9);
            assert!(m > -5.0, "departure margin {m}");
            // The planned gradient is what the type can do on one engine.
            assert!(d.grad <= pf.climb_grad_aeo && d.grad <= pf.climb_grad + 1e-6);
            for w in d.alt.windows(2) {
                assert!(w[1] - w[0] <= STEP * pf.climb_grad + 0.01);
            }
            // Ends at the target or in a hold that clears the terrain at the end altitude.
            match &d.hold {
                Some(h) => assert!(h.need(&g) <= d.alt.last().unwrap() + 1.0),
                None => assert!(*d.alt.last().unwrap() >= d.target - 1.0),
            }
        }
    }

    #[test]
    fn blocked_straight_in_curves() {
        // A ridge across the extended centreline 10..13 km out, 1,400 m high,
        // with a gap 2.5 km to the south: the approach comes through the gap.
        let ridge = |p: Vec2| -> Option<f32> {
            let across = p.x < -10_000.0 && p.x > -13_000.0;
            let gap = (p.y - 2500.0).abs() < 2000.0;
            Some(if across && !gap { 2000.0 } else { FLAT })
        };
        let g = Grid::build(ridge, Vec2::ZERO, 30_000.0, 200.0);
        let a = plan_arrival(&g, Vec2::ZERO, vec2(1.0, 0.0), FLAT, 0.0, 300.0, 15.5, &perf()).expect("arrival");
        let m = path_margin(&g, &a.path, &a.alt, |d| roc_approach(a.dist[0] - d, 9000.0).min(75.0));
        assert!(m > -5.0, "margin {m}");
        assert_ne!(a.kind, ArrKind::Straight);
    }

    #[test]
    fn holds_and_locate() {
        let h = Hold { fix: Vec2::ZERO, inbound: vec2(0.0, -1.0), side: 1.0, r: 1000.0, leg: 3000.0, min_alt: 0.0 };
        // Inbound north, right turns: the outbound leg lies to the east (+x).
        let t = h.track(200.0);
        assert!(t.iter().all(|p| p.x > -10.0 && p.x < 2010.0));
        // The lap is continuous and closest() inverts point().
        for k in 0..100 {
            let s = h.lap() * k as f32 / 100.0;
            let p = h.point(s);
            assert!(p.distance(h.point(s + 1.0)) < 1.5);
            let (s2, d) = h.closest(p);
            assert!(d < 1.0 && (s2 - s).abs() < 2.0, "{s} {s2} {d}");
        }
        // On the inbound leg, heading (about) north; on the outbound leg, south.
        let (trk, e) = h.steer(vec2(0.0, 2900.0));
        assert!(wrap(trk - 0.0).abs() < 0.05 && e.abs() < 1.0, "{trk} {e}");
        let (trk, _) = h.steer(vec2(2000.0, 100.0));
        assert!(wrap(trk - std::f32::consts::PI).abs() < 0.05, "{trk}");
        // Off to the west of the inbound leg: back towards it, still northbound.
        let (trk, _) = h.steer(vec2(-1500.0, 2900.0));
        assert!(wrap(trk).abs() < 1.2 && wrap(trk) > 0.1, "{trk}");
        let path = [vec2(0.0, 0.0), vec2(1000.0, 0.0), vec2(1000.0, 1000.0)];
        let l = locate(&path, vec2(1010.0, 500.0), 0);
        assert_eq!(l.i, 1);
        assert!((l.along - 1500.0).abs() < 1.0 && (l.cross + 10.0).abs() < 0.5, "{l:?}");
    }

    #[test]
    fn valley_tour_stays_low() {
        // A cross of valleys (floor 500 m) through high ground (2,000 m).
        let m = |p: Vec2| Some(if p.x.abs() < 1500.0 || p.y.abs() < 1500.0 { 500.0 } else { 2000.0 });
        let g = Grid::build(m, Vec2::ZERO, 15_000.0, 200.0);
        assert!(relief(&g, Vec2::ZERO, 8000.0) > 1000.0);
        let t = valley_tour(&g, Vec2::ZERO, 3, 3000.0, 9000.0, 0);
        assert!(t.iter().filter(|w| w.1).count() == 3);
        let mut prev = Vec2::ZERO;
        for (w, _) in t.iter().chain(std::iter::once(&(Vec2::ZERO, false))) {
            for k in 0..=20 {
                let p = prev.lerp(*w, k as f32 / 20.0);
                assert!(g.at(p) < 1000.0, "over the high ground at {p:?}");
            }
            prev = *w;
        }
    }

    #[test]
    fn valley_route_goes_round_the_mountain() {
        // A mountain between a and b: the route goes round it, not over.
        let m = |p: Vec2| Some(if p.length() < 3000.0 { 2500.0 } else { 500.0 });
        let g = Grid::build(m, Vec2::ZERO, 12_000.0, 200.0);
        let r = valley_route(&g, vec2(-8000.0, 0.0), vec2(8000.0, 0.0), 400.0);
        assert!(r.len() > 2);
        for w in r.windows(2) {
            for k in 0..=20 {
                let p = w[0].lerp(w[1], k as f32 / 20.0);
                assert!(g.at(p) < 1000.0, "over the mountain at {p:?}");
            }
        }
    }
}
