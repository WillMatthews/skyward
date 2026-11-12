//! Auto-flight bot: an AI pilot that flies the real flight model by moving
//! the controls (and by driving the autopilot), never by editing the state.
//!
//! * **Glider soaring** ([`Bot`] on a glider): reads the total-energy vario,
//!   picks cumulus to glide to (cloud size ≈ expected climb), cruises at the
//!   MacCready speed-to-fly, centres thermals with a lift-weighted circle fit
//!   and an orbit-following guidance law (bank ~38°), leaves below cloud base,
//!   wanders along a sightseeing route, keeps a runway within safe glide and
//!   lands on it (airbrakes on a steep final) when the lift runs out.
//! * **Powered auto tour**: take-off roll with centreline tracking, rotation
//!   at Vr, gear and flaps up, autopilot climb, a tour of landmarks (named
//!   London sights when in range, else tall-building clusters / water derived
//!   from the loaded map), then an obstacle-checked approach on a computed
//!   glidepath, stabilised-approach gate (go-around if not), flare, rollout
//!   with spoilers / reversers / brakes, taxi to the runway end, turn round
//!   and do it again. The Lancer adds an afterburner climb and a fast low pass.
//!
//! Wind: the bot reads the wind like an FMS (ground velocity − air velocity,
//! smoothed) and the reported surface wind / gust (ATIS). It flies tracks,
//! not headings (wind-correction angle on every leg and on final), lands and
//! takes off into wind (backtracking the runway when needed), adds half the
//! headwind plus the gust to its approach speed, de-crabs in the flare and
//! holds the centreline wing-low. The glider centres thermals in the frame
//! that drifts with the wind aloft, flies MacCready speed-to-fly corrected for
//! head / tailwind and works out glide range with the wind.
//!
//! The bot is called once per physics step (before `Aircraft::step`).

use crate::autopilot::Autopilot;
use crate::clouds::CloudField;
use crate::flight::{Aircraft, AircraftDef, Category, ControlLaw, Controls, EngineKind, Environment, GRAVITY, KT};
use crate::scene::Runway;
use crate::world::World;
use glam::{vec2, Vec2, Vec3};
use std::collections::VecDeque;
use std::f32::consts::{PI, TAU};

const FT: f32 = 0.3048;
const NM: f32 = 1852.0;

// ------------------------------------------------------------------ geometry helpers

pub fn wrap_pi(a: f32) -> f32 {
    let mut d = a.rem_euclid(TAU);
    if d > PI {
        d -= TAU;
    }
    d
}

/// Heading (0 = north, π/2 = east) of a world XZ direction.
pub fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// World XZ unit direction of a heading.
pub fn dir_of(h: f32) -> Vec2 {
    vec2(h.sin(), -h.cos())
}

fn xz(p: Vec3) -> Vec2 {
    vec2(p.x, p.z)
}

/// Heading that makes good `track` (rad) at true airspeed `tas` in wind `w`
/// (m/s, the direction the air moves): the wind-correction ("crab") angle.
pub fn crab(track: f32, tas: f32, w: Vec2) -> f32 {
    let d = dir_of(track);
    let right = vec2(-d.y, d.x);
    let x = w.dot(right);
    wrap_pi(track - (x / tas.max(15.0)).clamp(-0.6, 0.6).asin())
}

/// Headwind component (m/s, + = on the nose) of wind `w` for travel along `dir`.
pub fn headwind(w: Vec2, dir: Vec2) -> f32 {
    -w.dot(dir.normalize_or_zero())
}

/// Landing / take-off preference for a runway direction: obstacle-clear
/// glidepath (deg) plus a penalty for tailwind (≈1° per 1.6 kt) — so it
/// lands into wind unless the obstacles make that end much worse.
fn wind_penalty(w: Vec2, dir: Vec2) -> f32 {
    let tail_kt = (-headwind(w, dir)).max(0.0) / KT;
    tail_kt * 0.6 + if tail_kt > 10.0 { 10.0 } else { 0.0 }
}

/// Runway designator ("09", "27") for a course.
pub fn rwy_number(course: f32) -> String {
    let d = (course.to_degrees().rem_euclid(360.0) / 10.0).round() as i32;
    format!("{:02}", if d == 0 { 36 } else { d })
}

/// One end of a runway: threshold, direction of travel (landing / take-off).
#[derive(Clone, Debug)]
pub struct RunwayEnd {
    pub thr: Vec2,
    pub dir: Vec2,
    pub len: f32,
    pub width: f32,
    pub name: String,
    /// Runway surface height (m) at the threshold and at the far end (terrain.rs).
    pub elev: f32,
    pub elev_far: f32,
}

impl RunwayEnd {
    /// `reverse = false` → from `rw.a` towards `rw.b`.
    pub fn of(rw: &Runway, reverse: bool) -> RunwayEnd {
        let (thr, far) = if reverse { (rw.b, rw.a) } else { (rw.a, rw.b) };
        let dir = (far - thr).normalize_or(vec2(0.0, -1.0));
        let (elev, elev_far) = (rw.height_at(thr), rw.height_at(far));
        RunwayEnd { thr, dir, len: rw.length(), width: rw.width, name: rwy_number(heading_of(dir)), elev, elev_far }
    }
    /// Runway surface height at `p` (level beyond the ends).
    pub fn elev_at(&self, p: Vec2) -> f32 {
        let t = (self.along(p) / self.len.max(1.0)).clamp(0.0, 1.0);
        self.elev + (self.elev_far - self.elev) * t
    }
    pub fn course(&self) -> f32 {
        heading_of(self.dir)
    }
    pub fn right(&self) -> Vec2 {
        vec2(-self.dir.y, self.dir.x)
    }
    /// Distance down the runway from the threshold (negative = before it, on the approach side).
    pub fn along(&self, p: Vec2) -> f32 {
        (p - self.thr).dot(self.dir)
    }
    /// Lateral offset, + = right of the centreline (looking along `dir`).
    pub fn cross(&self, p: Vec2) -> f32 {
        (p - self.thr).dot(self.right())
    }
    pub fn far(&self) -> Vec2 {
        self.thr + self.dir * self.len
    }
    pub fn reversed(&self) -> RunwayEnd {
        let dir = -self.dir;
        RunwayEnd { thr: self.far(), dir, len: self.len, width: self.width, name: rwy_number(heading_of(dir)), elev: self.elev_far, elev_far: self.elev }
    }
    pub fn same_runway(&self, rw: &Runway) -> bool {
        let m = (rw.a + rw.b) * 0.5;
        m.distance(self.thr + self.dir * self.len * 0.5) < 60.0
    }
}

/// Glidepath angle (rad) needed to clear obstacles by 60 m on the approach to
/// `end` (aiming `aim` m past the threshold), searching 8 km out in a
/// widening funnel. At least 3°.
pub fn approach_slope(world: &World, end: &RunwayEnd, aim: f32) -> f32 {
    let mut need = 3f32.to_radians();
    let mut s = 150.0;
    while s <= 8000.0 {
        let half = 90.0 + 0.1 * s;
        let mut l = -half;
        while l <= half {
            let p = end.thr - end.dir * s + end.right() * l;
            // Buildings and the terrain itself, above the threshold.
            let top = world.obstacle_top(p).unwrap_or(f32::MIN).max(world.ground_height(p)) - end.elev;
            if top > -50.0 {
                need = need.max(((top + 60.0) / (s + aim)).atan());
            }
            l += 45.0;
        }
        s += 40.0;
    }
    need
}

/// Climb gradient (rise / run) needed to clear obstacles by 60 m departing
/// along `end` (from lift-off ~60% down the runway), 7 km out.
pub fn departure_gradient(world: &World, end: &RunwayEnd) -> f32 {
    let lift = end.len * 0.6;
    let mut need: f32 = 0.0;
    let mut s = end.len;
    while s <= end.len + 7000.0 {
        let half = 90.0 + 0.1 * (s - end.len);
        let mut l = -half;
        while l <= half {
            let q = end.thr + end.dir * s + end.right() * l;
            let top = world.obstacle_top(q).unwrap_or(f32::MIN).max(world.ground_height(q)) - end.elev;
            if top > -50.0 {
                need = need.max((top + 60.0) / (s - lift).max(50.0));
            }
            l += 45.0;
        }
        s += 40.0;
    }
    need
}

/// Which way to take off from `rw`: into wind, else the end with the easier climb-out.
pub fn best_takeoff_end(world: &World, rw: &Runway) -> RunwayEnd {
    let a = RunwayEnd::of(rw, false);
    let b = RunwayEnd::of(rw, true);
    let w = world.wind.surface;
    // Climb gradient in % ≈ "degrees" of effort; tailwind dominates when real.
    let cost = |e: &RunwayEnd| departure_gradient(world, e) * 100.0 * 0.5 + wind_penalty(w, e.dir);
    if cost(&b) + 0.5 < cost(&a) { b } else { a }
}

/// Which way to land on `rw`: into wind, unless the other end's obstacle-clear
/// glidepath is much shallower. Returns the end and its glidepath (rad).
pub fn best_landing_end(world: &World, rw: &Runway, aim: f32) -> (RunwayEnd, f32) {
    let a = RunwayEnd::of(rw, false);
    let b = RunwayEnd::of(rw, true);
    let (sa, sb) = (approach_slope(world, &a, aim), approach_slope(world, &b, aim));
    let w = world.wind.surface;
    let ca = sa.to_degrees() + wind_penalty(w, a.dir);
    let cb = sb.to_degrees() + wind_penalty(w, b.dir);
    if cb + 0.1 < ca { (b, sb) } else { (a, sa) }
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Name of the airfield a runway belongs to (nearest airport label within 4 km).
pub fn field_name(world: &World, rw: &Runway) -> String {
    let m = (rw.a + rw.b) * 0.5;
    world
        .airports
        .iter()
        .filter(|a| a.pos.distance(m) < 4000.0)
        .min_by(|a, b| a.pos.distance(m).total_cmp(&b.pos.distance(m)))
        .map(|a| if !a.name.is_empty() { a.name.clone() } else if !a.icao.is_empty() { a.icao.clone() } else { a.iata.clone() })
        .unwrap_or_else(|| "airfield".into())
}

// ------------------------------------------------------------------ landmarks

/// A sightseeing waypoint.
#[derive(Clone, Debug)]
pub struct Landmark {
    pub name: String,
    pub pos: Vec2,
}

/// Named sights (lat, lon) used when they are in range of the start.
const SIGHTS: &[(&str, f64, f64)] = &[
    ("Canary Wharf", 51.5050, -0.0195),
    ("Tower Bridge", 51.5055, -0.0754),
    ("St Paul's", 51.5138, -0.0984),
    ("Westminster", 51.4995, -0.1248),
    ("Battersea", 51.4818, -0.1446),
    ("Hyde Park", 51.5073, -0.1657),
    ("Greenwich", 51.4826, -0.0077),
    ("The O2", 51.5030, 0.0032),
    ("Thames Barrier", 51.4966, 0.0368),
    ("Olympic Park", 51.5430, -0.0166),
    ("Wembley", 51.5560, -0.2796),
    ("Richmond Park", 51.4428, -0.2760),
    ("Crystal Palace", 51.4200, -0.0700),
    ("Dartford Crossing", 51.4645, 0.2600),
    ("Epping Forest", 51.6500, 0.0500),
    ("Windsor Castle", 51.4839, -0.6044),
    ("Kew Gardens", 51.4787, -0.2956),
];

/// A loop of up to `max` landmarks between `min_r` and `max_r` of `home`,
/// ordered around it. Falls back to features found in the loaded map (tallest
/// buildings, water) and then to a ring of points.
pub fn plan_tour(world: &World, home: Vec2, min_r: f32, max_r: f32, max: usize) -> Vec<Landmark> {
    let mut c: Vec<Landmark> = SIGHTS
        .iter()
        .map(|&(n, lat, lon)| Landmark { name: n.into(), pos: world.proj.to_world(lat, lon) })
        .filter(|l| (min_r..=max_r).contains(&l.pos.distance(home)))
        .collect();
    if c.len() < 2 {
        c.extend(derived_landmarks(world, home, max_r.min(7000.0)));
        c.retain(|l| l.pos.distance(home) >= min_r * 0.5);
    }
    if c.len() < 2 {
        for k in 0..3 {
            let a = k as f32 * TAU / 3.0 + 0.4;
            c.push(Landmark { name: format!("Waypoint {}", k + 1), pos: home + dir_of(a) * (min_r.max(3000.0)) });
        }
    }
    // Thin out to `max`, evenly around the circle, then order by bearing so the route is a loop.
    c.sort_by(|a, b| heading_of(a.pos - home).total_cmp(&heading_of(b.pos - home)));
    if c.len() > max {
        let n = c.len();
        c = (0..max).map(|i| c[i * n / max].clone()).collect();
    }
    c
}

/// The glider's sightseeing route, variant `n`: different radius, number of
/// sights, starting point and direction each time, so successive launches
/// (and laps) don't fly the same loop.
fn glider_tour(world: &World, home: Vec2, n: u32) -> Vec<Landmark> {
    let n = n as usize;
    let max_r = [8_000.0, 11_000.0, 6_500.0, 9_500.0, 13_000.0][n % 5];
    let count = 3 + n % 3;
    let mut t = plan_tour(world, home, 2500.0, max_r, count + 2);
    // Drop one or two sights (a different one each time) to vary the shape.
    while t.len() > count {
        let k = (n * 7 + t.len()) % t.len();
        t.remove(k);
    }
    if !t.is_empty() {
        let k = (n / 2) % t.len();
        t.rotate_left(k);
    }
    if n % 2 == 1 {
        t.reverse();
    }
    t
}

/// Tallest-building cluster and the biggest nearby water, from the loaded map.
fn derived_landmarks(world: &World, home: Vec2, r: f32) -> Vec<Landmark> {
    let step = 150.0;
    let n = (r / step) as i32;
    let mut tall: Option<(f32, Vec2)> = None;
    let mut water = Vec::new();
    for iz in -n..=n {
        for ix in -n..=n {
            let p = home + vec2(ix as f32, iz as f32) * step;
            if let Some(t) = world.obstacle_top(p) {
                if tall.is_none_or(|(h, _)| t > h) {
                    tall = Some((t, p));
                }
            } else if (ix + iz) % 2 == 0 && world.is_water(p) {
                water.push(p);
            }
        }
    }
    let mut out = Vec::new();
    if let Some((h, p)) = tall {
        if h > 40.0 {
            out.push(Landmark { name: format!("Tall buildings ({h:.0} m)"), pos: p });
        }
    }
    if !water.is_empty() {
        // The water point with the most water around it, away from home.
        let best = water
            .iter()
            .max_by(|a, b| {
                let sa = water.iter().filter(|q| q.distance(**a) < 600.0).count() as f32 + a.distance(home) * 1e-3;
                let sb = water.iter().filter(|q| q.distance(**b) < 600.0).count() as f32 + b.distance(home) * 1e-3;
                sa.total_cmp(&sb)
            })
            .copied()
            .unwrap();
        out.push(Landmark { name: "The river".into(), pos: best });
    }
    out
}

// ------------------------------------------------------------------ inner loops ("hands")

#[derive(Default, Clone, Debug)]
struct Hands {
    pitch_i: f32,
    spd_i: f32,
    spd_init: bool,
    prev_ias: f32,
    accel: f32,
    bank: f32,
    g_el: Option<f32>,
    roll_i: f32,
}

impl Hands {
    fn fbw(def: &AircraftDef, ac: &Aircraft) -> bool {
        def.control_law != ControlLaw::Direct && ac.fbw_active() > 0.5
    }

    /// Elevator for a pitch-attitude demand (rad).
    fn elevator(&mut self, def: &AircraftDef, ac: &Aircraft, cmd: f32, dt: f32) -> f32 {
        let e = cmd - ac.pitch();
        let q = ac.rates().y;
        if Self::fbw(def, ac) {
            let tas = ac.tas().max(20.0);
            let phi = ac.roll();
            let gamma = (ac.vertical_speed() / ac.vel.length().max(1.0)).clamp(-1.0, 1.0).asin();
            let nz_neutral = gamma.cos() / phi.cos().max(0.5);
            let dnz = 0.8 * e * tas / GRAVITY - 0.3 * q * tas / GRAVITY;
            self.pitch_i = 0.0;
            Self::nz_stick(def, nz_neutral, dnz)
        } else {
            let (kp, ki, kd) = match def.category {
                Category::Airliner => (2.4, 0.5, 2.0),
                Category::FastJet => (2.0, 0.5, 1.0),
                Category::Warbird => (2.2, 0.6, 0.9),
                _ => (2.2, 0.8, 0.7),
            };
            let turn = (1.0 / ac.roll().cos().max(0.6) - 1.0) * 0.5;
            self.pitch_i = (self.pitch_i + ki * (e + 0.1 * turn) * dt).clamp(-0.8, 0.8);
            (kp * e - kd * q + self.pitch_i).clamp(-1.0, 1.0)
        }
    }

    /// FBW stick for a load-factor change `dnz` (g) about `nz_neutral`
    /// (the control law commands load factor; fast jets have a shaped stick).
    fn nz_stick(def: &AircraftDef, nz_neutral: f32, dnz: f32) -> f32 {
        {
            let range = if dnz >= 0.0 { (def.nz_max - nz_neutral).max(0.3) } else { (nz_neutral - def.nz_min).max(0.3) };
            let mut x = (dnz / range).clamp(-1.0, 1.0);
            if def.control_law == ControlLaw::FastJet {
                let target = x;
                for _ in 0..4 {
                    let f = x * (0.35 + 0.65 * x * x) - target;
                    x -= f / (0.35 + 1.95 * x * x);
                }
                x = x.clamp(-1.0, 1.0);
            }
            x
        }
    }

    /// Pitch demand that flies indicated airspeed `v` (m/s) on the elevator.
    fn speed_pitch(&mut self, ac: &Aircraft, v: f32, lo: f32, hi: f32, dt: f32) -> f32 {
        let ias = ac.airspeed();
        if !self.spd_init {
            self.spd_i = ac.pitch();
            self.spd_init = true;
            self.prev_ias = ias;
        }
        if dt > 0.0 {
            let a = (ias - self.prev_ias) / dt;
            self.accel += (a - self.accel) * (dt / 0.8).min(1.0);
        }
        self.prev_ias = ias;
        let e = ias - v; // fast → pitch up
        self.spd_i = (self.spd_i + 0.004 * e * dt).clamp(lo, hi);
        // Speed trend damps the phugoid: pitch up while still accelerating.
        (self.spd_i + 0.015 * e + 0.05 * self.accel).clamp(lo, hi)
    }

    /// Glider stick: fly airspeed `v` through the load factor (stick ≈ angle of attack), as a
    /// glider pilot does; works the same wings level or in a 45° thermalling turn.
    fn elevator_for_speed(&mut self, ac: &Aircraft, v: f32, dt: f32) -> f32 {
        let ias = ac.airspeed();
        if dt > 0.0 {
            let a = (ias - self.prev_ias) / dt;
            self.accel += (a - self.accel) * (dt / 0.8).min(1.0);
        }
        self.prev_ias = ias;
        let phi = ac.roll().abs().min(1.1);
        let nz_cmd = (1.0 / phi.cos() + 0.03 * (ias - v) + 0.10 * self.accel).clamp(0.4, 2.2);
        let err = nz_cmd - ac.g_load();
        let mut i = self.g_el.unwrap_or(0.0);
        i = (i + 0.35 * err * dt).clamp(-1.0, 1.0);
        self.g_el = Some(i);
        (i + 0.25 * err - 0.25 * ac.rates().y).clamp(-1.0, 1.0)
    }

    /// Aileron with an integrator (holds a steep bank against the overbanking tendency).
    fn aileron_i(&mut self, def: &AircraftDef, ac: &Aircraft, bank: f32, dt: f32) -> f32 {
        let e = bank - ac.roll();
        self.roll_i = (self.roll_i + 0.5 * e * dt).clamp(-0.4, 0.4);
        (Self::aileron(def, ac, bank) + 0.6 * e + self.roll_i).clamp(-1.0, 1.0)
    }

    fn aileron(def: &AircraftDef, ac: &Aircraft, bank: f32) -> f32 {
        let p = ac.rates().x;
        if Self::fbw(def, ac) {
            (1.5 * (bank - ac.roll()) / def.roll_rate_max.max(0.05)).clamp(-1.0, 1.0)
        } else {
            let (ka, kr) = if def.category == Category::Airliner { (2.5, 1.5) } else { (2.2, 0.5) };
            (ka * (bank - ac.roll()) - kr * p).clamp(-1.0, 1.0)
        }
    }

    fn coord_rudder(ac: &Aircraft) -> f32 {
        (3.0 * ac.beta()).clamp(-1.0, 1.0)
    }
}

/// Bank (rad) that turns onto heading `h`, limited to `max`.
fn bank_for_heading(ac: &Aircraft, h: f32, max: f32) -> f32 {
    let e = wrap_pi(h - ac.heading());
    (e * 1.6).clamp(-max, max)
}

/// Ground steering: rudder that holds the centreline of `end` (also used for taxiing to a point).
fn steer(def: &AircraftDef, ac: &Aircraft, course: f32, cross: f32) -> f32 {
    let herr = wrap_pi(course - ac.heading());
    // Cross-track correction, gentler at speed and none at take-off speeds (no swerving).
    let k = 0.04 * (6.0 / gs(ac).max(6.0)) * (1.0 - (gs(ac) - 12.0) / 8.0).clamp(0.0, 1.0);
    let corr = (-cross * k).clamp(-0.3, 0.3);
    let e = wrap_pi(herr + corr);
    let (k, kd) = if def.taildragger { (5.0, 2.0) } else { (2.5, 1.2) };
    (k * e - kd * ac.rates().z).clamp(-1.0, 1.0)
}

/// Flare / touchdown in a crosswind: (rudder, aileron) that line the nose up
/// with the runway and hold the centreline with the upwind wing down
/// (at most `max_bank_deg`).
fn decrab(def: &AircraftDef, ac: &Aircraft, end: &RunwayEnd, p: Vec2, max_bank_deg: f32) -> (f32, f32) {
    let x = end.cross(p);
    let v_lat = vec2(ac.vel.x, ac.vel.z).dot(end.right());
    let herr = wrap_pi(end.course() - ac.heading());
    let rudder = (3.0 * herr - 1.2 * ac.rates().z).clamp(-1.0, 1.0);
    let lim = max_bank_deg.to_radians();
    let bank = (-(0.10 * v_lat + 0.012 * x)).clamp(-lim, lim);
    (rudder, Hands::aileron(def, ac, bank))
}

/// Pitch attitude to touch down in (main wheels first; three-point for taildraggers).
fn touchdown_pitch(def: &AircraftDef) -> f32 {
    let d = match def.category {
        Category::Trainer => 5.0,
        Category::Warbird => def.rest_pitch().to_degrees() - 1.0,
        Category::FastJet => 9.0,
        Category::Airliner => {
            if def.engine_kind == EngineKind::Jet {
                4.5
            } else {
                3.5
            }
        }
        Category::Glider => 2.0,
    };
    d.to_radians()
}

fn gs(ac: &Aircraft) -> f32 {
    vec2(ac.vel.x, ac.vel.z).length()
}

fn flap_lever(def: &AircraftDef, idx: usize) -> f32 {
    let full = def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
    def.flap_detents.get(idx.min(def.flap_detents.len().saturating_sub(1))).copied().unwrap_or(0.0) / full
}

fn approach_to(cur: f32, target: f32, rate: f32, dt: f32) -> f32 {
    cur + (target - cur).clamp(-rate * dt, rate * dt)
}

// ------------------------------------------------------------------ stats / log

#[derive(Default, Clone, Debug)]
pub struct BotStats {
    pub takeoffs: u32,
    pub landings: u32,
    pub go_arounds: u32,
    pub cycles: u32,
    pub thermals: u32,
    pub thermal_time: f32,
    pub thermal_gain: f32,
    pub airborne_time: f32,
    pub max_alt: f32,
    pub min_alt_airborne: f32,
    pub relaunches: u32,
    pub worst_touchdown: f32,
}

// ------------------------------------------------------------------ the bot

pub struct Bot {
    /// One-line HUD status ("Thermalling +2.4 m/s").
    pub status: String,
    pub stats: BotStats,
    /// Timestamped events (take-offs, landings, thermals, …) for logs.
    pub events: Vec<(f32, String)>,
    /// Set by the glider bot after landing in watch mode: the app may relaunch.
    pub wants_relaunch: bool,
    t: f32,
    hands: Hands,
    kind: Kind,
    was_on_ground: bool,
    last_vs: f32,
    /// Smoothed wind estimate (m/s, ground plane).
    wind: Option<Vec2>,
}

enum Kind {
    Glider(Box<Glider>),
    Powered(Box<Powered>),
}

impl Bot {
    pub fn new(def: &AircraftDef, ac: &Aircraft, world: &World, home: Option<&Runway>) -> Bot {
        let kind = if def.has_engine {
            Kind::Powered(Box::new(Powered::new(def, ac, world, home)))
        } else {
            Kind::Glider(Box::new(Glider::new(def, ac, world, home)))
        };
        let mut b = Bot {
            status: "Auto flight".into(),
            stats: BotStats { min_alt_airborne: f32::MAX, ..Default::default() },
            events: Vec::new(),
            wants_relaunch: false,
            t: 0.0,
            hands: Hands::default(),
            kind,
            was_on_ground: ac.on_ground,
            last_vs: 0.0,
            wind: None,
        };
        b.log(format!("auto flight engaged ({})", def.name));
        b
    }

    pub fn is_glider(&self) -> bool {
        matches!(self.kind, Kind::Glider(_))
    }

    /// Short machine-readable phase name (logs).
    pub fn phase(&self) -> String {
        match &self.kind {
            Kind::Glider(g) => format!("{:?}", g.mode_name()),
            Kind::Powered(p) => format!("{:?}", p.phase),
        }
    }

    /// The current sightseeing target, if any (name, position).
    pub fn waypoint(&self) -> Option<(String, Vec2)> {
        match &self.kind {
            Kind::Glider(g) => g.tour.get(g.wp).map(|l| (l.name.clone(), l.pos)),
            Kind::Powered(p) => match p.phase {
                PPhase::Enroute => p.tour.get(p.wp).map(|l| (l.name.clone(), l.pos)),
                _ => None,
            },
        }
    }

    /// The remaining sightseeing route (next waypoint first), for the map.
    pub fn route(&self) -> Vec<(String, Vec2)> {
        let (tour, wp) = match &self.kind {
            Kind::Glider(g) => (&g.tour, g.wp),
            Kind::Powered(p) => match p.phase {
                PPhase::Enroute => (&p.tour, p.wp),
                _ => return Vec::new(),
            },
        };
        tour.iter().skip(wp).map(|l| (l.name.clone(), l.pos)).collect()
    }

    /// Something the cinematic camera may want to frame (runway threshold when landing).
    pub fn focus_runway(&self) -> Option<RunwayEnd> {
        match &self.kind {
            Kind::Glider(g) => g.land.as_ref().map(|l| l.end.clone()),
            Kind::Powered(p) => p.rw.clone(),
        }
    }

    fn log(&mut self, s: String) {
        self.events.push((self.t, s));
    }

    /// The world origin moved: re-express every remembered position.
    /// Ground-fixed things (runways, sights, gates) use the exact map; things
    /// carried by the air (thermals, cloud targets) the rigid shift.
    pub fn rebase(&mut self, r: &crate::world::Rebase) {
        let rw_end = |e: &mut RunwayEnd| {
            let far = r.ground(e.far());
            e.thr = r.ground(e.thr);
            e.dir = (far - e.thr).normalize_or(e.dir);
            e.len = far.distance(e.thr);
        };
        match &mut self.kind {
            Kind::Glider(g) => {
                if let Some(t) = &mut g.target {
                    t.pos = r.shift2(t.pos);
                }
                if let Some(th) = &mut g.thermal {
                    th.c = r.shift2(th.c);
                    for smp in th.samples.iter_mut() {
                        smp.1 = r.shift2(smp.1);
                    }
                }
                g.search_c = r.shift2(g.search_c);
                for l in &mut g.tour {
                    l.pos = r.ground(l.pos);
                }
                g.home = r.ground(g.home);
                if let Some(l) = &mut g.land {
                    rw_end(&mut l.end);
                }
            }
            Kind::Powered(p) => {
                if let Some(f) = &mut p.field {
                    f.a = r.ground(f.a);
                    f.b = r.ground(f.b);
                }
                if let Some(e) = &mut p.rw {
                    rw_end(e);
                }
                for l in &mut p.tour {
                    l.pos = r.ground(l.pos);
                }
                p.gate = r.ground(p.gate);
            }
        }
    }

    /// One physics step: read the aircraft, write `ctl` (and drive `ap`).
    pub fn update(&mut self, def: &AircraftDef, ac: &Aircraft, ctl: &mut Controls, ap: &mut Autopilot, world: &World, dt: f32) {
        self.t += dt;
        if ac.crashed.is_some() {
            ap.disengage();
            self.status = "Crashed".into();
            return;
        }
        // Bookkeeping.
        if !ac.on_ground {
            self.stats.airborne_time += dt;
            self.stats.max_alt = self.stats.max_alt.max(ac.pos.y);
            self.stats.min_alt_airborne = self.stats.min_alt_airborne.min(ac.pos.y);
            self.last_vs = ac.vertical_speed();
        }
        if ac.on_ground && !self.was_on_ground {
            self.stats.worst_touchdown = self.stats.worst_touchdown.max(-self.last_vs);
        }
        self.was_on_ground = ac.on_ground;
        // Wind as the air-data computer sees it (gusts smoothed out), seeded
        // with the reported wind at the aircraft's height.
        let raw = xz(ac.wind_estimate());
        let wind = match self.wind {
            Some(w) if ac.tas() > 15.0 => w + (raw - w) * (dt / 6.0).min(1.0),
            Some(w) => w,
            None => world.wind.mean(ac.pos.y.max(0.0), 0.3),
        };
        self.wind = Some(wind);
        let mut ev = Vec::new();
        match &mut self.kind {
            Kind::Glider(g) => {
                g.wind = wind;
                g.wind_sfc = world.wind.surface;
            }
            Kind::Powered(p) => p.wind = wind,
        }
        match &mut self.kind {
            Kind::Glider(g) => g.update(def, ac, ctl, ap, world, &mut self.hands, &mut self.stats, &mut ev, &mut self.status, &mut self.wants_relaunch, self.t, dt),
            Kind::Powered(p) => p.update(def, ac, ctl, ap, world, &mut self.hands, &mut self.stats, &mut ev, &mut self.status, self.t, dt),
        }
        for e in ev {
            self.log(e);
        }
    }
}

// ------------------------------------------------------------------ glider

#[derive(Clone, Copy, Debug, PartialEq)]
enum GMode {
    Cruise,
    Search,
    Thermal,
    FinalGlide,
    Landing,
    Landed,
}

#[derive(Clone, Debug)]
struct Target {
    pos: Vec2,
    /// Expected climb (m/s) — judged from the cloud's size.
    est: f32,
    radius: f32,
    key: (i32, i32),
}

struct ThermalState {
    c: Vec2,
    dir: f32,
    t0: f32,
    alt0: f32,
    /// (time, position, vario) every 0.25 s.
    samples: VecDeque<(f32, Vec2, f32)>,
    turned: f32,
    last_hdg: f32,
    lap_alt: f32,
    lap_t: f32,
    laps: u32,
    last_lap_climb: f32,
    key: Option<(i32, i32)>,
    next_fit: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum LPhase {
    ToGate,
    LoseHeight,
    Final,
    Flare,
    Rollout,
}

struct GliderLanding {
    end: RunwayEnd,
    slope: f32,
    phase: LPhase,
    field: String,
}

struct Glider {
    mode: GMode,
    polar: Vec<(f32, f32)>,
    vs1: f32,
    mc: f32,
    vario: f32,
    vario_fast: f32,
    prev_e: Option<f32>,
    target: Option<Target>,
    thermal: Option<ThermalState>,
    used: Vec<((i32, i32), f32)>,
    next_plan: f32,
    sample_t: f32,
    lift_t: f32,
    lift_peak: f32,
    search_t0: f32,
    search_c: Vec2,
    tour: Vec<Landmark>,
    wp: usize,
    home: Vec2,
    land: Option<GliderLanding>,
    landed_t: f32,
    airbrake: f32,
    avoid: Option<(f32, f32)>,
    flare_pitch: f32,
    sink_i: f32,
    /// Smoothed wind at the glider (set by `Bot::update` every step).
    wind: Vec2,
    /// Reported surface wind (for glide range: the wind weakens lower down).
    wind_sfc: Vec2,
    /// Which tour variant this is (each launch / lap picks another route).
    tour_n: u32,
}

/// Tours flown so far (all gliders): each new one is planned differently.
static GLIDER_TOURS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Cloud-base and cumulus information as a pilot would see it.
fn sky(world: &World) -> &CloudField {
    &world.clouds
}

impl Glider {
    fn new(def: &AircraftDef, ac: &Aircraft, world: &World, home: Option<&Runway>) -> Glider {
        // Speed polar (IAS m/s → sink m/s) from the flight model's own trim solution.
        let vs1 = def.vs1_kt * KT;
        let mut polar = Vec::new();
        let mut v = vs1 * 1.05;
        while v < def.vne_kt * KT * 0.8 {
            let tp = def.trim_point_at(v, 0.0, 800.0, true);
            polar.push((v, -tp.gamma.sin() * crate::flight::tas_from_cas(v, 800.0)));
            v += 0.5;
        }
        let home = home.map(|r| (r.a + r.b) * 0.5).unwrap_or(xz(ac.pos));
        let tour_n = GLIDER_TOURS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let tour = glider_tour(world, home, tour_n);
        Glider {
            mode: if ac.on_ground { GMode::Landed } else { GMode::Cruise },
            polar,
            vs1,
            mc: 1.0,
            vario: 0.0,
            vario_fast: 0.0,
            prev_e: None,
            target: None,
            thermal: None,
            used: Vec::new(),
            next_plan: 0.0,
            sample_t: 0.0,
            lift_t: 0.0,
            lift_peak: 0.0,
            search_t0: 0.0,
            search_c: Vec2::ZERO,
            tour,
            wp: 0,
            home,
            land: None,
            landed_t: 0.0,
            airbrake: 0.0,
            avoid: None,
            flare_pitch: 0.0,
            sink_i: 0.0,
            wind: world.wind.mean(ac.pos.y, 0.3),
            wind_sfc: world.wind.surface,
            tour_n,
        }
    }

    fn mode_name(&self) -> GMode {
        self.mode
    }

    fn sink_at(&self, v: f32) -> f32 {
        let i = self.polar.partition_point(|p| p.0 < v).min(self.polar.len() - 1);
        self.polar[i].1
    }

    fn best_ld(&self) -> f32 {
        self.polar.iter().map(|(v, s)| v / s.max(0.1)).fold(0.0, f32::max)
    }

    /// Best glide ratio over the ground with headwind `hw` (m/s; < 0 tailwind),
    /// flying the speed that maximises it.
    fn best_ld_wind(&self, hw: f32) -> f32 {
        self.polar.iter().map(|(v, s)| (v - hw) / s.max(0.1)).fold(0.5, f32::max)
    }

    /// MacCready speed to fly (IAS) for climb expectation `m` in air moving vertically at `netto`.
    fn stf(&self, m: f32, netto: f32) -> f32 {
        self.stf_wind(m, netto, 0.0)
    }

    /// ... into a headwind `hw` (m/s, < 0 = tailwind): maximise the
    /// cross-country speed over the ground, (v − hw) / (sink + m − netto) —
    /// faster into wind, slower downwind.
    fn stf_wind(&self, m: f32, netto: f32, hw: f32) -> f32 {
        let m = m - netto;
        self.polar
            .iter()
            .map(|&(v, s)| (v, (v - hw) / (s + m).max(0.05)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
            .unwrap_or(self.vs1 * 1.4)
    }

    fn thermal_speed(&self) -> f32 {
        self.vs1 * 1.30
    }

    fn is_used(&self, key: (i32, i32), t: f32) -> bool {
        self.used.iter().any(|(k, until)| *k == key && *until > t)
    }

    /// Nearest known runway long enough for a glider, and the point to aim for.
    fn nearest_field(&self, world: &World, p: Vec2) -> Option<Runway> {
        world
            .runways
            .iter()
            .filter(|r| r.length() > 350.0)
            .min_by(|a, b| seg_dist(p, a.a, a.b).total_cmp(&seg_dist(p, b.a, b.b)))
            .cloned()
    }

    /// Height (m) needed to glide from `p` to `field` with a safety margin and
    /// arrive at circuit height — glide ratio over the ground in the wind.
    fn needed_height(&self, p: Vec2, field: &Runway) -> f32 {
        let d = seg_dist(p, field.a, field.b);
        let to = (field.a + field.b) * 0.5 - p;
        // The wind weakens on the way down: use the mean of here and the surface.
        let w = (self.wind + self.wind_sfc) * 0.5;
        let ld = self.best_ld_wind(headwind(w, to)) * 0.65;
        // (Absolute: above the field's runway.)
        d / ld.max(6.0) + 250.0 + field.height_at(p)
    }

    #[allow(clippy::too_many_arguments)]
    fn update(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        ap: &mut Autopilot,
        world: &World,
        hands: &mut Hands,
        stats: &mut BotStats,
        ev: &mut Vec<String>,
        status: &mut String,
        relaunch: &mut bool,
        t: f32,
        dt: f32,
    ) {
        ap.disengage();
        let p = xz(ac.pos);
        // Total-energy vario (compensated for speed changes), two time constants.
        let e = ac.pos.y + ac.tas() * ac.tas() / (2.0 * GRAVITY);
        let raw = match self.prev_e {
            Some(pe) => (e - pe) / dt,
            None => 0.0,
        };
        self.prev_e = Some(e);
        self.vario += (raw - self.vario) * (dt / 1.6).min(1.0);
        self.vario_fast += (raw - self.vario_fast) * (dt / 0.5).min(1.0);
        let netto = self.vario + self.sink_at(ac.airspeed());
        // Thermals (and their clouds) drift with the wind aloft; the glider with the local wind.
        let aloft = world.wind.aloft;
        let wind = self.wind;
        let tas = ac.tas().max(15.0);

        if ac.on_ground {
            self.ground(def, ac, ctl, hands, stats, ev, status, relaunch, t, dt);
            return;
        }
        let base = sky(world).base();

        // ---- planning, once a second
        if t >= self.next_plan {
            self.next_plan = t + 1.0;
            self.plan(def, ac, world, stats, ev, t);
        }

        // ---- thermal detection while cruising / searching
        // (Not in the first seconds: the vario settles after the release.)
        if matches!(self.mode, GMode::Cruise | GMode::Search | GMode::FinalGlide) && ac.pos.y < base - 180.0 && t > 4.0 {
            let thresh = if self.mode == GMode::FinalGlide { 1.2 } else { (0.45 * self.mc).max(0.6) };
            if self.vario > thresh {
                self.lift_t += dt;
                if self.lift_t < dt * 1.5 {
                    self.lift_peak = self.vario;
                }
                self.lift_peak = self.lift_peak.max(self.vario);
                // Push on into the core while the lift still builds, then turn.
                let fading = self.vario < self.lift_peak - 0.15;
                if (self.lift_t > 1.0 && fading) || self.lift_t > 4.0 {
                    self.start_thermal(ac, world, ev, stats, t, aloft);
                }
            } else {
                self.lift_t = 0.0;
            }
        }

        // ---- fly
        let mut bank = 0.0;
        let mut speed = self.stf(self.mc, netto.clamp(-3.0, 1.0));
        let mut brake = 0.0;
        let v_thermal = self.thermal_speed();
        match self.mode {
            GMode::Thermal => {
                let th = self.thermal.as_mut().unwrap();
                // Samples for the centre fit.
                if t >= self.sample_t {
                    self.sample_t = t + 0.25;
                    // In the frame drifting with the thermal.
                    th.samples.push_back((t, p - aloft * t, self.vario_fast));
                    while th.samples.front().is_some_and(|s| t - s.0 > 26.0) {
                        th.samples.pop_front();
                    }
                }
                let h = ac.heading();
                th.turned += wrap_pi(h - th.last_hdg).abs();
                th.last_hdg = h;
                if th.turned >= TAU {
                    th.turned -= TAU;
                    th.laps += 1;
                    th.last_lap_climb = (ac.pos.y - th.lap_alt) / (t - th.lap_t).max(1.0);
                    th.lap_alt = ac.pos.y;
                    th.lap_t = t;
                }
                if t >= th.next_fit && t - th.t0 > 8.0 {
                    th.next_fit = t + 1.5;
                    Self::fit_centre(th);
                }
                // Orbit guidance around the estimated centre, in the frame
                // drifting with the thermal (centre and samples live there).
                let v = ac.tas().max(15.0);
                let nominal = 38f32.to_radians();
                let r_nom = v * v / (GRAVITY * nominal.tan());
                let d = p - aloft * t - th.c;
                let dist = d.length().max(1.0);
                let u = d / dist;
                // Tangent in the turn direction (right turn = clockwise seen from above).
                let tang = vec2(-u.y, u.x) * th.dir;
                let beta = ((dist - r_nom) / 120.0).atan().clamp(-0.8, 0.8);
                let want = tang * beta.cos() - u * beta.sin();
                // Air velocity that gives that velocity relative to the thermal
                // (the local wind and the thermal's drift differ with height).
                let herr = wrap_pi(heading_of(want * v - (wind - aloft)) - ac.heading());
                // Gentle corrections around the nominal bank (a steep turn is slow to adjust).
                let corr = (0.35 * herr).clamp(-12f32.to_radians(), 12f32.to_radians());
                bank = (th.dir * nominal + corr).clamp(-48f32.to_radians(), 48f32.to_radians());
                // Never roll out of (or reverse) the turn while centring.
                if th.dir > 0.0 {
                    bank = bank.max(18f32.to_radians());
                } else {
                    bank = bank.min(-18f32.to_radians());
                }
                speed = v_thermal;
                *status = format!("Thermalling {:+.1} m/s  (avg {:+.1})", self.vario, (ac.pos.y - th.alt0) / (t - th.t0).max(1.0));
            }
            GMode::Search => {
                // Circle under the cloud where the thermal should be (it drifts with the wind aloft).
                self.search_c += aloft * dt;
                let v = ac.tas().max(15.0);
                let r = (v * v / (GRAVITY * 30f32.to_radians().tan())).max(120.0);
                let d = p - self.search_c;
                let dist = d.length().max(1.0);
                let u = d / dist;
                let tang = vec2(u.y, -u.x);
                let beta = ((dist - r) / 80.0).atan();
                let want = tang * beta.cos() - u * beta.sin();
                bank = bank_for_heading(ac, heading_of(want * v - (wind - aloft)), 32f32.to_radians());
                speed = self.thermal_speed() * 1.08;
                *status = format!("Searching for lift  ({:+.1} m/s)", self.vario);
            }
            GMode::Cruise => {
                let (goal, label) = match &self.target {
                    // Aim off for the cloud's drift while we get there.
                    Some(tg) => (
                        tg.pos + aloft * (p.distance(tg.pos) / (tas + headwind(-aloft, tg.pos - p)).max(10.0)).min(400.0),
                        format!("Gliding to {} {:.1} km (expect {:+.1})", if sky(world).visible() { "cloud" } else { "thermal" }, p.distance(tg.pos) / 1000.0, tg.est),
                    ),
                    None => {
                        let wp = self.tour.get(self.wp).map(|l| (l.pos, l.name.clone())).unwrap_or((self.home, "home".into()));
                        (wp.0, format!("Searching for lift, heading for {} {:.1} km", wp.1, p.distance(wp.0) / 1000.0))
                    }
                };
                let mut track = heading_of(goal - p);
                if let Some((h, until)) = self.avoid {
                    if t < until {
                        track = h;
                    }
                }
                bank = bank_for_heading(ac, crab(track, tas, wind), 30f32.to_radians());
                // MacCready speed to fly for the head / tailwind on this leg.
                speed = self.stf_wind(self.mc, netto.clamp(-3.0, 1.0), headwind(wind, dir_of(track)));
                *status = label;
            }
            GMode::FinalGlide | GMode::Landing => {
                let (b, s, br, label) = self.landing_guidance(def, ac, world, hands, ctl, ev, stats, t, dt);
                bank = b;
                speed = s;
                brake = br;
                *status = label;
                if self.mode == GMode::Landing && matches!(self.land.as_ref().map(|l| l.phase), Some(LPhase::Flare)) {
                    // Flare: elevator handled in landing_guidance; rudder de-crabs.
                    ctl.aileron = Hands::aileron(def, ac, bank);
                    ctl.rudder = match &self.land {
                        Some(l) => decrab(def, ac, &l.end, p, 8.0).0,
                        None => Hands::coord_rudder(ac),
                    };
                    ctl.flaps = approach_to(ctl.flaps, brake, 1.5, dt);
                    return;
                }
            }
            GMode::Landed => {}
        }
        // Never fly into cloud: open the airbrakes and push on near the base.
        if ac.pos.y > base - 60.0 && self.mode != GMode::Landing {
            brake = brake.max(((ac.pos.y - (base - 60.0)) / 40.0).clamp(0.0, 1.0));
            speed = speed.max(self.vs1 * 2.0);
        }
        self.airbrake = approach_to(self.airbrake, brake, 1.5, dt);
        ctl.elevator = hands.elevator_for_speed(ac, speed, dt);
        hands.bank = approach_to(hands.bank, bank, 12f32.to_radians(), dt);
        ctl.aileron = hands.aileron_i(def, ac, hands.bank, dt);
        ctl.rudder = Hands::coord_rudder(ac);
        ctl.flaps = self.airbrake;
        ctl.brakes = 0.0;
    }

    /// Lift-weighted circle fit: shift the estimated centre toward where the lift was strongest.
    fn fit_centre(th: &mut ThermalState) {
        let lag = 1.0; // vario lag (s): pair each reading with where we were slightly earlier
        let s: Vec<(Vec2, f32)> = th
            .samples
            .iter()
            .map(|&(ts, _, w)| {
                let p = th.samples.iter().min_by(|a, b| (a.0 - (ts - lag)).abs().total_cmp(&(b.0 - (ts - lag)).abs())).map(|x| x.1).unwrap();
                (p, w)
            })
            .collect();
        if s.len() < 20 {
            return;
        }
        let lo = s.iter().map(|x| x.1).fold(f32::MAX, f32::min);
        let hi = s.iter().map(|x| x.1).fold(f32::MIN, f32::max);
        let mean = s.iter().fold(Vec2::ZERO, |a, x| a + x.0) / s.len() as f32;
        if hi - lo < 0.25 {
            th.c = th.c.lerp(mean, 0.3);
            return;
        }
        let cut = lo + 0.45 * (hi - lo);
        let (mut wc, mut ws) = (Vec2::ZERO, 0.0);
        for (p, w) in &s {
            let k = (w - cut).max(0.0).powi(2);
            wc += *p * k;
            ws += k;
        }
        if ws < 1e-6 {
            return;
        }
        let wc = wc / ws;
        let shift = ((wc - mean) * 1.6).clamp_length_max(55.0);
        th.c = th.c.lerp(mean + shift, 0.55);
    }

    fn start_thermal(&mut self, ac: &Aircraft, world: &World, ev: &mut Vec<String>, stats: &mut BotStats, t: f32, aloft: Vec2) {
        let p = xz(ac.pos);
        let fwd = dir_of(ac.heading());
        let right = vec2(-fwd.y, fwd.x);
        // Turn toward the cloud's centre if we can see one nearby, else to the right.
        let cloud = sky(world).thermals_near(p, 900.0).into_iter().min_by(|a, b| a.0.distance(p).total_cmp(&b.0.distance(p)));
        let dir = match cloud {
            Some((c, _, _)) if (c - p).dot(right).abs() > 30.0 => (c - p).dot(right).signum(),
            _ => 1.0,
        };
        let v = ac.tas().max(15.0);
        let r = v * v / (GRAVITY * 38f32.to_radians().tan());
        let key = self.target.as_ref().filter(|tg| tg.pos.distance(p) < tg.radius + 900.0).map(|tg| tg.key);
        self.thermal = Some(ThermalState {
            c: p + right * dir * r - aloft * t,
            dir,
            t0: t,
            alt0: ac.pos.y,
            samples: VecDeque::new(),
            turned: 0.0,
            last_hdg: ac.heading(),
            lap_alt: ac.pos.y,
            lap_t: t,
            laps: 0,
            last_lap_climb: 0.0,
            key,
            next_fit: t + 4.0,
        });
        self.mode = GMode::Thermal;
        self.lift_t = 0.0;
        stats.thermals += 1;
        ev.push(format!("lift {:+.1} m/s at {:.0} m — circling {}", self.vario, ac.pos.y, if dir > 0.0 { "right" } else { "left" }));
    }

    fn leave_thermal(&mut self, ac: &Aircraft, stats: &mut BotStats, ev: &mut Vec<String>, t: f32, why: &str) {
        if let Some(th) = self.thermal.take() {
            let dt = t - th.t0;
            let gain = ac.pos.y - th.alt0;
            stats.thermal_time += dt;
            stats.thermal_gain += gain;
            let avg = gain / dt.max(1.0);
            ev.push(format!("left thermal ({why}): {:+.0} m in {:.0} s, avg {:+.2} m/s, top {:.0} m", gain, dt, avg, ac.pos.y));
            // MacCready setting tracks the climbs we actually get.
            if dt > 40.0 {
                self.mc = (0.6 * self.mc + 0.4 * avg.max(0.3)).clamp(0.3, 3.0);
            }
            let key = th.key.or_else(|| self.target.as_ref().filter(|tg| tg.pos.distance(xz(ac.pos)) < tg.radius + 900.0).map(|tg| tg.key));
            if let Some(k) = key {
                self.used.push((k, t + if avg < 0.5 { 1500.0 } else { 900.0 }));
            }
        }
        self.target = None;
        self.mode = GMode::Cruise;
        self.next_plan = 0.0;
    }

    fn plan(&mut self, def: &AircraftDef, ac: &Aircraft, world: &World, stats: &mut BotStats, ev: &mut Vec<String>, t: f32) {
        let p = xz(ac.pos);
        let field = self.nearest_field(world, p);
        let base = sky(world).base();
        let _ = def;

        // Tour progress.
        if let Some(l) = self.tour.get(self.wp) {
            if p.distance(l.pos) < 1500.0 {
                ev.push(format!("tour: over {}", l.name));
                self.wp += 1;
                if self.wp >= self.tour.len() {
                    // Lap done: another route next time.
                    self.wp = 0;
                    self.tour_n = GLIDER_TOURS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    self.tour = glider_tour(world, self.home, self.tour_n);
                    if let Some(f) = self.tour.first() {
                        ev.push(format!("new tour: {} first", f.name));
                    }
                }
            }
        }
        // Obstacles ahead when low (tall buildings).
        if !matches!(self.mode, GMode::Landing | GMode::Thermal) {
            self.avoid = avoid_obstacles(world, ac, 30.0, 110.0).map(|h| (h, t + 8.0)).or(self.avoid.filter(|a| a.1 > t));
        }

        match self.mode {
            GMode::Thermal => {
                let th = self.thermal.as_ref().unwrap();
                let elapsed = t - th.t0;
                let avg = (ac.pos.y - th.alt0) / elapsed.max(1.0);
                if ac.pos.y > base - 100.0 {
                    self.leave_thermal(ac, stats, ev, t, "cloud base");
                } else if th.laps >= 2 && th.last_lap_climb < (0.35 * self.mc).max(0.25) && elapsed > 45.0 {
                    self.leave_thermal(ac, stats, ev, t, "weak");
                } else if elapsed > 30.0 && avg < -0.3 {
                    self.leave_thermal(ac, stats, ev, t, "lost it");
                } else if let Some(f) = &field {
                    // Drifted out of glide range of the field while circling in weak lift.
                    if ac.pos.y < self.needed_height(p, f) && th.last_lap_climb < 0.5 && th.laps >= 2 {
                        self.leave_thermal(ac, stats, ev, t, "too low");
                    }
                }
                return;
            }
            GMode::Landing => return,
            GMode::Landed => return,
            _ => {}
        }

        // Final glide decision.
        if let Some(f) = &field {
            let need = self.needed_height(p, f);
            if ac.pos.y < need && self.mode != GMode::FinalGlide {
                self.mode = GMode::FinalGlide;
                self.land = None;
                ev.push(format!("low ({:.0} m, need {:.0}) — final glide to {}", ac.pos.y, need, field_name(world, f)));
                return;
            }
            if self.mode == GMode::FinalGlide && ac.pos.y > need + 250.0 {
                self.mode = GMode::Cruise;
                self.land = None;
            }
        }
        if self.mode == GMode::FinalGlide {
            return;
        }

        // Search pattern timeout.
        if self.mode == GMode::Search {
            if t - self.search_t0 > 50.0 {
                if let Some(tg) = self.target.take() {
                    self.used.push((tg.key, t + 1200.0));
                }
                ev.push(if sky(world).visible() { "no lift under the cloud — moving on" } else { "no lift there — moving on" }.into());
                self.mode = GMode::Cruise;
            } else {
                return;
            }
        }

        // Choose (or keep) a cloud to glide to.
        let ld_plan = self.best_ld() * 0.7;
        let reach = ((ac.pos.y - world.ground_height(p) - 250.0) * ld_plan).max(0.0);
        let wp = self.tour.get(self.wp).map(|l| l.pos).unwrap_or(self.home);
        let visible = sky(world).visible();
        // Keys in field coordinates: a cloud keeps its key while it drifts.
        let drift = sky(world).drift();
        let fkey = |c: Vec2| (((c.x - drift.x) / 100.0).floor() as i32, ((c.y - drift.y) / 100.0).floor() as i32);
        let cands: Vec<Target> = if visible {
            sky(world)
                .cells_near(p, reach.min(9000.0))
                .into_iter()
                .map(|c| {
                    let key = fkey(c.center);
                    // Pilot's judgement of the cloud: size → climb, with some error.
                    let judge = 0.8 + 0.4 * ((key.0.wrapping_mul(73856093) ^ key.1.wrapping_mul(19349663)) as u32 % 1000) as f32 / 1000.0;
                    Target { pos: c.center, est: (c.strength - 0.9) * judge, radius: c.thermal_radius, key }
                })
                .collect()
        } else {
            // Blue thermals: no clouds to read, only the occasional sign (birds, dust) close by.
            sky(world)
                .thermals_near(p, 1600.0)
                .into_iter()
                .map(|(c, r, s)| Target { pos: c, est: (s - 0.9) * 0.8, radius: r, key: fkey(c) })
                .collect()
        };
        // The current target has drifted with the wind: refresh where it is.
        if let Some(cur) = &mut self.target {
            if let Some(c) = cands.iter().find(|c| c.key == cur.key) {
                cur.pos = c.pos;
            } else {
                // Out of sight: dead-reckon its drift (plan() runs once a second).
                cur.pos += world.wind.aloft;
            }
        }
        let mut best: Option<(f32, Target)> = None;
        let ld_still = self.best_ld();
        let margin = field.as_ref().map_or(1000.0, |f| ac.pos.y - self.needed_height(p, f));
        for c in cands {
            if self.is_used(c.key, t) || c.est < 0.4 {
                continue;
            }
            // Glide over the ground there in this wind: upwind clouds cost more height.
            let ld_w = self.best_ld_wind(headwind(self.wind, c.pos - p)) * 0.7;
            let d = p.distance(c.pos);
            if d * ld_plan / ld_w.max(1.0) > reach || d < 80.0 {
                continue;
            }
            let arrive = ac.pos.y - d / ld_w.max(1.0);
            if arrive < 300.0 + world.ground_height(c.pos) {
                continue;
            }
            if let Some(f) = &field {
                if arrive < self.needed_height(c.pos, f) - 60.0 {
                    continue;
                }
            }
            let progress = (p.distance(wp) - c.pos.distance(wp)) / 1000.0;
            let d_eff = d * ld_still * 0.7 / ld_w.max(1.0);
            // Stay upwind of home: clouds that cost glide range back are worth less.
            // (Weighs more as the height margin over the final glide shrinks.)
            let range = field.as_ref().map_or(0.0, |f| (self.needed_height(p, f) - self.needed_height(c.pos, f)) / 300.0);
            let k_up = 0.4 + 1.2 * (1.0 - margin / 600.0).clamp(0.0, 1.0);
            // Sightseeing only while there is height to spare over the glide home.
            let k_tour = 0.25 * (margin / 600.0).clamp(0.0, 1.0);
            let score = c.est - 0.30 * d_eff / 1000.0 + k_tour * progress + k_up * range.clamp(-2.0, 1.0);
            if best.as_ref().is_none_or(|b| score > b.0) {
                best = Some((score, c));
            }
        }
        // Hysteresis: keep the current target unless the new one is clearly better.
        let keep = match (&self.target, &best) {
            (Some(cur), Some((s, nb))) if cur.key != nb.key => {
                let d = p.distance(cur.pos) * ld_still * 0.7 / (self.best_ld_wind(headwind(self.wind, cur.pos - p)) * 0.7).max(1.0);
                let cur_score = cur.est - 0.30 * d / 1000.0 + 0.25 * (p.distance(wp) - cur.pos.distance(wp)) / 1000.0;
                cur_score > *s - 0.4 && !self.is_used(cur.key, t)
            }
            (Some(cur), None) => !self.is_used(cur.key, t),
            _ => false,
        };
        if !keep {
            if let Some((_, nb)) = best {
                if self.target.as_ref().is_none_or(|c| c.key != nb.key) {
                    ev.push(format!(
                        "heading for {} {:.1} km away (expect {:+.1} m/s)",
                        if visible { "cloud" } else { "a likely thermal" },
                        p.distance(nb.pos) / 1000.0,
                        nb.est
                    ));
                }
                self.target = Some(nb);
            } else {
                self.target = None;
            }
        }
        // Arrived under the cloud without finding lift: search.
        if let Some(tg) = &self.target {
            if p.distance(tg.pos) < tg.radius * 0.9 {
                self.mode = GMode::Search;
                self.search_t0 = t;
                self.search_c = tg.pos;
            }
        }
    }

    /// Approach and landing guidance. Returns (bank, speed, airbrake, status).
    #[allow(clippy::too_many_arguments)]
    fn landing_guidance(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        world: &World,
        hands: &mut Hands,
        ctl: &mut Controls,
        ev: &mut Vec<String>,
        stats: &mut BotStats,
        _t: f32,
        dt: f32,
    ) -> (f32, f32, f32, String) {
        let _ = stats;
        let p = xz(ac.pos);
        if self.land.is_none() {
            let Some(f) = self.nearest_field(world, p) else {
                return (0.0, self.stf(0.0, 0.0), 0.0, "Final glide — no airfield known".into());
            };
            let (end, slope) = best_landing_end(world, &f, 120.0);
            let field = field_name(world, &f);
            ev.push(format!("landing RWY {} at {} (glidepath {:.1}°)", end.name, field, slope.to_degrees()));
            self.land = Some(GliderLanding { end, slope: slope.max(5f32.to_radians()), phase: LPhase::ToGate, field });
            self.mode = GMode::Landing;
        }
        let wind = self.wind;
        let tas = ac.tas().max(15.0);
        let to_gate_hw = self.land.as_ref().map(|l| headwind(wind, l.end.thr - l.end.dir * 1800.0 - p)).unwrap_or(0.0);
        let ld_glide = self.best_ld_wind(to_gate_hw) * 0.6;
        let v_glide = self.stf_wind(0.5, 0.0, to_gate_hw);
        let l = self.land.as_mut().unwrap();
        let end = l.end.clone();
        let s = -end.along(p); // distance out from the threshold
        let x = end.cross(p);
        let aim = 120.0;
        let gp = (s + aim) * l.slope.tan() + def.gear_height;
        // Heights above the threshold; the wheels above the ground below.
        let h = ac.pos.y - end.elev;
        let agl = ac.pos.y - world.ground_height(p);
        // Approach speed plus half the headwind on final.
        let vapp = self.vs1 * 1.38 + (0.5 * headwind(wind, end.dir)).clamp(0.0, 5.0);
        let gate = end.thr - end.dir * 1800.0;
        let gate_h = (1800.0 + aim) * l.slope.tan();
        match l.phase {
            LPhase::ToGate => {
                let d = p.distance(gate);
                // Close enough and lined up roughly: go straight onto final.
                if s > 600.0 && x.abs() < s * 0.5 && h < gp + 120.0 && h > gp - 60.0 {
                    l.phase = LPhase::Final;
                } else if d < 450.0 {
                    l.phase = if h > gate_h + 80.0 { LPhase::LoseHeight } else { LPhase::Final };
                }
                let surplus = h - gate_h - d / ld_glide;
                let brake = if surplus > 150.0 { 0.6 } else { 0.0 };
                let b = bank_for_heading(ac, crab(heading_of(gate - p), tas, wind), 30f32.to_radians());
                (b, v_glide.max(vapp), brake, format!("Final glide to {} RWY {} — {:.1} km, {:+.0} m spare", l.field, end.name, d / 1000.0, surplus))
            }
            LPhase::LoseHeight => {
                // Orbit near the gate with airbrakes until at gate height, then roll out toward the runway.
                let d = p - (gate - end.dir * 400.0);
                let dist = d.length().max(1.0);
                let u = d / dist;
                let tang = vec2(-u.y, u.x);
                let r = 260.0;
                let beta = ((dist - r) / 80.0).atan();
                let want = tang * beta.cos() - u * beta.sin();
                let b = bank_for_heading(ac, heading_of(want * tas - wind), 35f32.to_radians());
                let toward = wrap_pi(end.course() - ac.heading()).abs() < 50f32.to_radians();
                // Height to lose for the glide from here to the threshold (in this wind).
                let here_h = (p.distance(end.thr) + aim) * l.slope.tan();
                if (h < here_h + 40.0 && toward) || h < here_h - 20.0 {
                    l.phase = LPhase::Final;
                }
                let brake = ((h - here_h) / 60.0).clamp(0.0, 1.0);
                (b, vapp, brake, format!("Losing height over {} ({:.0} m)", l.field, h))
            }
            LPhase::Final => {
                let hdg = crab(end.course() + (-x / 120.0).atan().clamp(-0.6, 0.6), tas, wind);
                let b = bank_for_heading(ac, hdg, 25f32.to_radians());
                // Airbrakes fly the glidepath; nominal half brakes.
                let brake = (0.5 + (h - gp) * 0.03).clamp(0.0, 1.0);
                let wheels = agl - def.gear_height;
                if wheels < 4.5 {
                    l.phase = LPhase::Flare;
                    self.flare_pitch = ac.pitch();
                    self.sink_i = 0.0;
                }
                (b, vapp, brake, format!("Final approach RWY {} ({}) — {:.0} m", end.name, l.field, s.max(0.0)))
            }
            LPhase::Flare => {
                let wheels = (agl - def.gear_height).max(0.0);
                let target_vs = -(0.35 + 0.22 * wheels);
                let err = target_vs - ac.vertical_speed();
                self.sink_i = (self.sink_i + 0.06 * err * dt).clamp(-0.1, 0.25);
                let pitch_cmd = (self.flare_pitch + 0.08 * err + self.sink_i).clamp(-0.05, 0.2);
                ctl.elevator = hands.elevator(def, ac, pitch_cmd, dt);
                // Bank into wind to hold the centreline (rudder lines up the nose, see update()).
                let v_lat = vec2(ac.vel.x, ac.vel.z).dot(end.right());
                let b = (-(0.10 * v_lat + 0.012 * x)).clamp(-0.14, 0.14);
                (b, vapp, 0.6, "Flare".into())
            }
            LPhase::Rollout => (0.0, vapp, 1.0, "Rollout".into()),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn ground(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        hands: &mut Hands,
        stats: &mut BotStats,
        ev: &mut Vec<String>,
        status: &mut String,
        relaunch: &mut bool,
        t: f32,
        dt: f32,
    ) {
        let _ = hands;
        if self.mode != GMode::Landed {
            self.mode = GMode::Landed;
            self.landed_t = t;
            stats.landings += 1;
            let where_ = self.land.as_ref().map(|l| format!("RWY {} at {}", l.end.name, l.field)).unwrap_or_else(|| "off-field".into());
            ev.push(format!("landed {where_}"));
            if let Some(l) = &mut self.land {
                l.phase = LPhase::Rollout;
            }
        }
        let course = self.land.as_ref().map(|l| l.end.course()).unwrap_or(ac.heading());
        let x = self.land.as_ref().map(|l| l.end.cross(xz(ac.pos))).unwrap_or(0.0);
        ctl.elevator = 0.3;
        ctl.aileron = (-2.0 * ac.roll()).clamp(-1.0, 1.0);
        ctl.rudder = steer(def, ac, course, x);
        ctl.brakes = if t - self.landed_t > 1.0 { 0.8 } else { 0.0 };
        ctl.flaps = approach_to(ctl.flaps, 1.0, 1.0, dt);
        *status = if gs(ac) > 1.0 { format!("Rollout — {:.0} kt", ac.airspeed() / KT) } else { "Landed".into() };
        if gs(ac) < 0.5 && t - self.landed_t > 20.0 {
            *relaunch = true;
        }
    }
}

/// Highest ground (or building) within `dist` ahead along `vel` (and a
/// little to either side): the floor a cruise altitude must clear.
fn terrain_floor(world: &World, p: Vec2, vel: Vec2, dist: f32) -> f32 {
    let d = vel.normalize_or(vec2(0.0, -1.0));
    let side = vec2(-d.y, d.x);
    let mut top = world.ground_height(p);
    let mut s = 0.0;
    while s <= dist {
        for l in [-600.0, 0.0, 600.0] {
            let q = p + d * s + side * l;
            top = top.max(world.obstacle_top(q).unwrap_or(f32::MIN).max(world.ground_height(q)));
        }
        s += 300.0;
    }
    top
}

/// If the path ahead (next `secs` seconds) passes within `margin` m above a
/// building, a heading that clears, else None.
fn avoid_obstacles(world: &World, ac: &Aircraft, secs: f32, margin: f32) -> Option<f32> {
    let p = xz(ac.pos);
    let v = gs(ac).max(20.0);
    let range = (v * secs).max(1200.0);
    let clear = |h: f32| -> bool {
        let d = dir_of(h);
        let side = vec2(-d.y, d.x);
        let mut s = 60.0;
        while s < range {
            let t = s / v;
            let alt = ac.pos.y + ac.vertical_speed().min(0.0) * t;
            for l in [-90.0, 0.0, 90.0] {
                let q = p + d * s + side * l;
                // Buildings and rising ground.
                let top = world.obstacle_top(q).unwrap_or(f32::MIN).max(world.ground_height(q));
                if top + margin > alt {
                    return false;
                }
            }
            s += 60.0;
        }
        true
    };
    let h0 = heading_of(vec2(ac.vel.x, ac.vel.z));
    if clear(h0) {
        return None;
    }
    for k in 1..=7 {
        for sgn in [1.0, -1.0] {
            let h = h0 + sgn * k as f32 * 20f32.to_radians();
            if clear(h) {
                return Some(wrap_pi(h));
            }
        }
    }
    Some(wrap_pi(h0 + PI))
}

// ------------------------------------------------------------------ powered

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PPhase {
    Hold,
    TakeoffRoll,
    Rotate,
    InitialClimb,
    Enroute,
    LowPass,
    Position,
    Final,
    Flare,
    Rollout,
    Taxi,
    Turnaround,
    LineUp,
    GoAround,
    Stuck,
}

/// Per-type numbers for the powered bot.
#[derive(Clone, Debug)]
struct Profile {
    to_flap: usize,
    app_flap: usize,
    land_flap: usize,
    vr: f32,
    climb_ias: f32,
    cruise_ias: f32,
    vapp: f32,
    flap_up_ias: f32,
    rot_pitch: f32,
    climb_pitch: f32,
    tour_alt: f32,
    final_dist: f32,
    aim: f32,
    flare_h: f32,
    tour_r: f32,
    taxi: f32,
    hand_off: f32,
    to_throttle: f32,
    max_gs: f32,
    turn_r: f32,
    stab_x: f32,
    brakes: f32,
}

impl Profile {
    fn of(def: &AircraftDef) -> Profile {
        let n = def.flap_detents.len();
        let last = n.saturating_sub(1);
        let kt = KT;
        let deg = |d: f32| d.to_radians();
        match def.category {
            Category::Airliner => {
                let jet = def.engine_kind == EngineKind::Jet;
                let to = match def.id {
                    "meridian190" => 2,
                    _ => 1,
                }
                .min(last);
                Profile {
                    to_flap: to,
                    app_flap: (last.saturating_sub(1)).max(1).min(last),
                    land_flap: last,
                    vr: def.vr_kt * kt,
                    climb_ias: if jet { 220.0 } else { 170.0 } * kt,
                    cruise_ias: if jet { 250.0 } else { 210.0 } * kt,
                    vapp: (def.vref_kt + 5.0) * kt,
                    flap_up_ias: (def.vref_kt + 40.0) * kt,
                    rot_pitch: deg(if jet { 9.0 } else { 8.0 }),
                    climb_pitch: deg(if jet { 12.0 } else { 9.0 }),
                    tour_alt: if jet { 1000.0 } else { 900.0 },
                    final_dist: if jet { 11_000.0 } else { 9_000.0 },
                    aim: 300.0,
                    flare_h: if jet { 11.0 } else { 8.0 },
                    tour_r: 26_000.0,
                    taxi: 12.0,
                    hand_off: 120.0,
                    to_throttle: 1.0,
                    max_gs: deg(4.5),
                    turn_r: 2200.0,
                    stab_x: 45.0,
                    brakes: 0.5,
                }
            }
            Category::FastJet => Profile {
                to_flap: 0,
                app_flap: last.min(1),
                land_flap: last,
                vr: def.vr_kt * kt,
                climb_ias: 320.0 * kt,
                cruise_ias: 380.0 * kt,
                vapp: (def.vref_kt + 8.0) * kt,
                flap_up_ias: 200.0 * kt,
                rot_pitch: deg(11.0),
                climb_pitch: deg(18.0),
                tour_alt: 1200.0,
                final_dist: 9_000.0,
                aim: 250.0,
                flare_h: 7.0,
                tour_r: 22_000.0,
                taxi: 12.0,
                hand_off: 150.0,
                to_throttle: 1.0,
                max_gs: deg(5.0),
                turn_r: 2500.0,
                stab_x: 40.0,
                brakes: 0.7,
            },
            Category::Warbird => Profile {
                to_flap: 0,
                app_flap: 0,
                land_flap: last,
                vr: def.vr_kt * kt,
                climb_ias: 160.0 * kt,
                cruise_ias: 230.0 * kt,
                vapp: (def.vref_kt + 6.0) * kt,
                flap_up_ias: 0.0,
                rot_pitch: deg(9.0),
                climb_pitch: deg(12.0),
                tour_alt: 650.0,
                final_dist: 5_000.0,
                aim: 150.0,
                flare_h: 4.0,
                tour_r: 14_000.0,
                taxi: 7.0,
                hand_off: 100.0,
                to_throttle: 1.0,
                max_gs: deg(5.0),
                turn_r: 900.0,
                stab_x: 35.0,
                brakes: 0.35,
            },
            _ => Profile {
                to_flap: 1.min(last),
                app_flap: (last / 2).max(1).min(last),
                land_flap: last,
                vr: def.vr_kt * kt,
                climb_ias: 75.0 * kt,
                cruise_ias: 100.0 * kt,
                vapp: (def.vs0_kt * 1.3 + 4.0).min(def.vref_kt) * kt,
                flap_up_ias: 68.0 * kt,
                rot_pitch: deg(7.0),
                climb_pitch: deg(8.0),
                tour_alt: 600.0,
                final_dist: 4_000.0,
                aim: 140.0,
                flare_h: 3.5,
                tour_r: 12_000.0,
                taxi: 9.0,
                hand_off: 90.0,
                to_throttle: 1.0,
                max_gs: deg(6.0),
                turn_r: 500.0,
                stab_x: 30.0,
                brakes: 0.6,
            },
        }
    }
}

struct Powered {
    phase: PPhase,
    pf: Profile,
    field: Option<Runway>,
    field_name: String,
    rw: Option<RunwayEnd>,
    gs_angle: f32,
    tour: Vec<Landmark>,
    wp: usize,
    phase_t: f32,
    rot_pitch: f32,
    flap_idx: usize,
    next_flap_t: f32,
    next_plan: f32,
    avoid: Option<(f32, f32)>,
    alt_floor: f32,
    flare_pitch: f32,
    sink_i: f32,
    touch_t: f32,
    turn_dir: f32,
    gate: Vec2,
    low_pass_done: bool,
    /// Flying the approach as a fast low pass (no gear, no flaps, ~100 m, then pull up).
    low_pass: bool,
    lp_inbound: bool,
    ab_climb: bool,
    leg_t: f32,
    /// Smoothed wind (set by `Bot::update` every step).
    wind: Vec2,
    /// Approach speed for this approach: Vapp + ½ headwind + gust.
    vapp: f32,
    /// Smoothed IAS for the stabilised-approach gate (gusts).
    ias_f: f32,
    /// Landed into wind: turn round and backtrack to take off the same way.
    backtrack: bool,
    /// Runway remaining (m) at which the taxi turns round (None = near the end).
    turn_at: Option<f32>,
    /// Height (m) and sink rate (m/s) at the start of the flare.
    flare_h0: f32,
    flare_vs0: f32,
}

impl Powered {
    fn new(def: &AircraftDef, ac: &Aircraft, world: &World, home: Option<&Runway>) -> Powered {
        let pf = Profile::of(def);
        let p = xz(ac.pos);
        let need = crate::sim::min_runway(def);
        let field = home
            .filter(|r| r.length() >= need * 0.8)
            .cloned()
            .or_else(|| {
                world
                    .runways
                    .iter()
                    .filter(|r| r.length() >= need * 0.8)
                    .min_by(|a, b| seg_dist(p, a.a, a.b).total_cmp(&seg_dist(p, b.a, b.b)))
                    .cloned()
            })
            .or_else(|| world.spawn_runway(p).cloned());
        let field_name = field.as_ref().map(|f| field_name(world, f)).unwrap_or_else(|| "airfield".into());
        let home_p = field.as_ref().map(|f| (f.a + f.b) * 0.5).unwrap_or(p);
        let tour = plan_tour(world, home_p, if def.category == Category::Trainer { 3000.0 } else { 4000.0 }, pf.tour_r, 4);
        let vapp0 = pf.vapp;
        let mut s = Powered {
            phase: PPhase::Enroute,
            pf,
            field,
            field_name,
            rw: None,
            gs_angle: 3f32.to_radians(),
            tour,
            wp: 0,
            phase_t: 0.0,
            rot_pitch: 0.0,
            flap_idx: 0,
            next_flap_t: 0.0,
            next_plan: 0.0,
            avoid: None,
            alt_floor: 0.0,
            flare_pitch: 0.0,
            sink_i: 0.0,
            touch_t: 0.0,
            turn_dir: -1.0,
            gate: Vec2::ZERO,
            low_pass_done: false,
            low_pass: false,
            lp_inbound: false,
            ab_climb: def.has_afterburner,
            leg_t: 0.0,
            wind: world.wind.surface,
            vapp: vapp0,
            ias_f: ac.airspeed(),
            backtrack: false,
            turn_at: None,
            flare_h0: 0.0,
            flare_vs0: 0.0,
        };
        // Where are we?
        if ac.on_ground {
            if let Some(f) = s.field.clone() {
                // Take off in the direction we face if aligned, else pick the better end.
                let a = RunwayEnd::of(&f, false);
                let end = if wrap_pi(a.course() - ac.heading()).abs() < 0.5 {
                    a
                } else if wrap_pi(a.reversed().course() - ac.heading()).abs() < 0.5 {
                    a.reversed()
                } else {
                    best_takeoff_end(world, &f)
                };
                let remain = end.len - end.along(p);
                s.rw = Some(end);
                s.phase = if gs(ac) > 15.0 {
                    PPhase::Rollout
                } else if remain > crate::sim::min_runway(def) * 0.75 && wrap_pi(s.rw.as_ref().unwrap().course() - ac.heading()).abs() < 0.2 {
                    PPhase::Hold
                } else {
                    PPhase::Taxi
                };
            } else {
                s.phase = PPhase::Stuck;
            }
        } else if s.field.as_ref().is_some_and(|f| {
            // Already set up on a final approach (or close in with the gear down): land.
            let (end, _) = best_landing_end(world, f, s.pf.aim);
            let d = -end.along(p);
            let on_final = d > 0.0 && d < 30_000.0 && end.cross(p).abs() < 3000.0 && wrap_pi(end.course() - ac.heading()).abs() < 0.8;
            (on_final || (ac.gear_pos() > 0.5 && seg_dist(p, f.a, f.b) < 15_000.0)) && ac.pos.y - f.height_at(p) < 1200.0
        }) {
            s.start_approach(world, ac);
        } else {
            s.phase = PPhase::Enroute;
        }
        s
    }

    fn set_phase(&mut self, ph: PPhase, t: f32) {
        self.phase = ph;
        self.phase_t = t;
    }

    fn start_approach(&mut self, world: &World, ac: &Aircraft) {
        let Some(f) = self.field.clone() else {
            self.phase = PPhase::Enroute;
            return;
        };
        let (end, slope) = best_landing_end(world, &f, self.pf.aim);
        self.gs_angle = slope.clamp(3f32.to_radians(), self.pf.max_gs);
        // Wind additive: half the steady headwind plus the full gust, at most 15 kt.
        let hw = headwind(world.wind.surface, end.dir).max(0.0);
        self.vapp = self.pf.vapp + (0.5 * hw + world.wind.gust).min(15.0 * KT);
        // Gate on the side we are on, beyond the final approach fix.
        let p = xz(ac.pos);
        let side = if end.cross(p) >= 0.0 { 1.0 } else { -1.0 };
        self.gate = end.thr - end.dir * (self.pf.final_dist + self.pf.turn_r * 1.5) + end.right() * side * self.pf.turn_r * 1.2;
        self.rw = Some(end);
        self.phase = PPhase::Position;
    }

    fn set_flaps(&mut self, def: &AircraftDef, ctl: &mut Controls, idx: usize, t: f32) {
        // One detent at a time, a few seconds apart (as a crew would).
        if idx != self.flap_idx && t >= self.next_flap_t {
            self.flap_idx = if idx > self.flap_idx { self.flap_idx + 1 } else { self.flap_idx - 1 };
            self.next_flap_t = t + 4.0;
        }
        ctl.flaps = flap_lever(def, self.flap_idx);
    }

    #[allow(clippy::too_many_arguments)]
    fn update(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        ap: &mut Autopilot,
        world: &World,
        hands: &mut Hands,
        stats: &mut BotStats,
        ev: &mut Vec<String>,
        status: &mut String,
        t: f32,
        dt: f32,
    ) {
        let p = xz(ac.pos);
        let pf = self.pf.clone();
        let ias = ac.airspeed();
        self.ias_f += (ias - self.ias_f) * (dt / 2.0).min(1.0);
        let wind = self.wind;
        let tas = ac.tas().max(20.0);
        // Heights above the runway in use (terrain.rs) and of the wheels above the ground.
        let fe = self.rw.as_ref().map(|e| e.elev).or_else(|| self.field.as_ref().map(|f| f.height_at(p))).unwrap_or(0.0);
        let ay = ac.pos.y - fe;
        let wheels = ac.pos.y - def.gear_height - world.ground_height(p);
        // Cruise high enough over the terrain ahead (and the field).
        let terrain_alt = terrain_floor(world, p, vec2(ac.vel.x, ac.vel.z), 9000.0) + pf.tour_alt.min(450.0);
        let rwname = self.rw.as_ref().map(|r| r.name.clone()).unwrap_or_default();
        let fbw = def.control_law != ControlLaw::Direct;
        // Throttle never touched by the AP on the ground.
        match self.phase {
            PPhase::Stuck => {
                ap.disengage();
                ctl.throttle = 0.0;
                ctl.brakes = 1.0;
                *status = "No runway known — auto flight idle".into();
            }
            PPhase::Hold => {
                ap.disengage();
                ctl.throttle = 0.0;
                ctl.brakes = 1.0;
                ctl.reverse = false;
                ctl.spoilers = 0.0;
                ctl.gear_down = true;
                self.set_flaps(def, ctl, pf.to_flap, t);
                ctl.elevator = 0.0;
                ctl.aileron = 0.0;
                ctl.rudder = 0.0;
                *status = format!("Ready for departure RWY {rwname}");
                if t - self.phase_t > 5.0 && self.flap_idx == pf.to_flap {
                    self.set_phase(PPhase::TakeoffRoll, t);
                    ev.push(format!("take-off roll RWY {rwname} ({})", self.field_name));
                }
            }
            PPhase::TakeoffRoll | PPhase::Rotate => {
                ap.disengage();
                let end = self.rw.clone().unwrap();
                ctl.brakes = 0.0;
                ctl.reverse = false;
                ctl.throttle = approach_to(ctl.throttle, if def.has_afterburner { 1.0 } else { pf.to_throttle }, 0.5, dt);
                self.set_flaps(def, ctl, pf.to_flap, t);
                // Propeller swing (torque, P-factor, gyroscopic precession as the tail comes up) wants
                // rudder against the rotation before it starts.
                let swing = if def.engine_kind == EngineKind::Piston && def.engine_count == 1 {
                    def.prop_rotation * (ias / pf.vr).clamp(0.0, 1.0) * if def.taildragger { 0.35 } else { 0.1 }
                } else {
                    0.0
                };
                ctl.rudder = (steer(def, ac, end.course(), end.cross(p)) + swing).clamp(-1.0, 1.0);
                ctl.aileron = (-2.0 * ac.roll() - 0.5 * ac.rates().x).clamp(-1.0, 1.0);
                if self.phase == PPhase::TakeoffRoll {
                    ctl.elevator = if def.taildragger && ias > 0.45 * pf.vr {
                        // Tail up, then hold it level.
                        (3.0 * (1f32.to_radians() - ac.pitch()) - 0.8 * ac.rates().y).clamp(-1.0, 0.6)
                    } else if def.taildragger {
                        0.0
                    } else {
                        0.0
                    };
                    *status = format!("Take-off roll RWY {rwname} — {:.0} kt", ias / KT);
                    if ias >= pf.vr {
                        self.set_phase(PPhase::Rotate, t);
                        self.rot_pitch = ac.pitch();
                        hands.pitch_i = 0.0;
                    }
                    // Abort if we run out of runway long before Vr.
                    if end.len - end.along(p) < 150.0 && ias < pf.vr * 0.8 {
                        ev.push("rejected take-off: out of runway".into());
                        self.set_phase(PPhase::Rollout, t);
                    }
                } else {
                    let rate: f32 = if def.category == Category::Airliner { 2.5 } else { 4.0 };
                    self.rot_pitch = approach_to(self.rot_pitch, pf.rot_pitch, rate.to_radians(), dt);
                    ctl.elevator = if ac.on_ground && !fbw {
                        hands.elevator(def, ac, self.rot_pitch, dt).max(0.0)
                    } else if ac.on_ground {
                        ((self.rot_pitch - ac.pitch()) * 6.0 - ac.rates().y * 2.0).clamp(-0.3, 1.0)
                    } else {
                        hands.elevator(def, ac, self.rot_pitch, dt)
                    };
                    *status = "Rotate".into();
                    if !ac.on_ground && t - self.phase_t > 1.0 && wheels > 3.0 {
                        stats.takeoffs += 1;
                        ev.push(format!("airborne at {:.0} kt", ias / KT));
                        self.set_phase(PPhase::InitialClimb, t);
                    }
                }
            }
            PPhase::InitialClimb | PPhase::GoAround => {
                ap.disengage();
                let end_course = self.rw.as_ref().map(|r| r.course()).unwrap_or(ac.heading());
                let thr = if def.has_afterburner && self.ab_climb { 1.0 } else if def.has_afterburner { 0.9 } else { 1.0 };
                ctl.throttle = approach_to(ctl.throttle, thr, 0.5, dt);
                ctl.reverse = false;
                ctl.spoilers = 0.0;
                if ac.vertical_speed() > 1.5 && wheels > 12.0 {
                    ctl.gear_down = false;
                }
                // Pitch for the climb, but keep the speed above V2-ish.
                let v2 = pf.vr + 10.0 * KT;
                let mut pitch = pf.climb_pitch;
                if ias < v2 {
                    pitch -= (v2 - ias) * 0.02;
                }
                let pitch = pitch.clamp(3f32.to_radians(), pf.climb_pitch);
                self.rot_pitch = approach_to(self.rot_pitch, pitch, 3f32.to_radians(), dt);
                ctl.elevator = hands.elevator(def, ac, self.rot_pitch, dt);
                // Track the runway course out (crabbed into the wind).
                let hdg = crab(self.avoid.filter(|a| a.1 > t).map(|a| a.0).unwrap_or(end_course), tas, wind);
                ctl.aileron = Hands::aileron(def, ac, bank_for_heading(ac, hdg, 15f32.to_radians()));
                ctl.rudder = Hands::coord_rudder(ac);
                if self.phase == PPhase::GoAround {
                    self.set_flaps(def, ctl, pf.to_flap.max(pf.app_flap.min(1)), t);
                    *status = "Go-around".into();
                } else {
                    if ias > pf.flap_up_ias {
                        self.set_flaps(def, ctl, 0, t);
                    } else {
                        ctl.flaps = flap_lever(def, self.flap_idx);
                    }
                    *status = format!("Climbing out — {:.0} ft", ac.pos.y / FT);
                }
                if wheels > pf.hand_off {
                    ap.engage_hold_current(ac);
                    ap.hdg = Some(hdg);
                    ap.alt = Some((fe + pf.tour_alt).max(terrain_alt).max(ac.pos.y));
                    ap.ias = Some(pf.climb_ias);
                    ap.vs = None;
                    if self.phase == PPhase::GoAround {
                        ap.alt = Some(fe + (pf.final_dist * self.gs_angle.tan()).max(450.0));
                        ap.ias = Some(pf.vapp + 25.0 * KT);
                        let w = world;
                        self.start_approach(w, ac);
                    } else {
                        // A fresh tour each circuit (rotate the starting landmark for variety).
                        self.wp = 0;
                        if self.tour.len() > 1 && stats.takeoffs > 1 {
                            self.tour.reverse();
                        }
                        self.low_pass_done = false;
                        self.set_phase(PPhase::Enroute, t);
                    }
                    self.phase_t = t;
                }
                if t >= self.next_plan {
                    self.next_plan = t + 1.0;
                    self.avoid = avoid_obstacles(world, ac, 30.0, 90.0).map(|h| (h, t + 10.0)).or(self.avoid.filter(|a| a.1 > t));
                }
            }
            PPhase::Enroute | PPhase::LowPass | PPhase::Position => {
                if !ap.engaged {
                    ap.engage_hold_current(ac);
                }
                ctl.reverse = false;
                ctl.spoilers = 0.0;
                ctl.brakes = 0.0;
                if t >= self.next_plan {
                    self.next_plan = t + 1.0;
                    self.avoid = avoid_obstacles(world, ac, 35.0, 120.0).map(|h| (h, t + 12.0)).or(self.avoid.filter(|a| a.1 > t));
                }
                let mut hdg;
                let mut alt;
                let mut spd;
                match self.phase {
                    PPhase::Enroute => {
                        if ac.vertical_speed() > 1.0 {
                            ctl.gear_down = false;
                        }
                        if ias > pf.flap_up_ias || self.flap_idx > 0 && ias > pf.climb_ias * 0.8 {
                            self.set_flaps(def, ctl, 0, t);
                        } else {
                            ctl.flaps = flap_lever(def, self.flap_idx);
                        }
                        let wp = self.tour.get(self.wp).cloned();
                        match wp {
                            Some(l) => {
                                hdg = crab(heading_of(l.pos - p), tas, wind);
                                let d = p.distance(l.pos);
                                *status = format!("Tour: {} {:.1} km", l.name, d / 1000.0);
                                // Overhead, or abeam inside our turn circle (it can't be overflown from here).
                                let r_turn = ac.tas().powi(2) / (GRAVITY * 25f32.to_radians().tan());
                                let off = wrap_pi(hdg - ac.heading()).abs();
                                if t - self.leg_t > 8.0 && (d < 900.0 || (d < 1.3 * r_turn && off > 70f32.to_radians())) {
                                    self.leg_t = t;
                                    ev.push(format!("tour: over {} at {:.0} ft", l.name, ac.pos.y / FT));
                                    self.wp += 1;
                                }
                            }
                            None => {
                                // Tour done: home.
                                if def.category == Category::FastJet && !self.low_pass_done {
                                    // A fast low pass flown down the approach path, then come round and land.
                                    self.low_pass = true;
                                    self.start_approach(world, ac);
                                    ev.push(format!("low pass RWY {}", self.rw.as_ref().map(|r| r.name.clone()).unwrap_or_default()));
                                } else {
                                    ev.push(format!("tour complete — returning to {}", self.field_name));
                                    self.start_approach(world, ac);
                                }
                                hdg = ac.heading();
                            }
                        }
                        alt = (fe + pf.tour_alt).max(terrain_alt);
                        // Afterburner climb for the fighter, then cruise power.
                        spd = if ac.pos.y < alt - 60.0 { pf.climb_ias } else { pf.cruise_ias };
                        if self.ab_climb && ac.pos.y > alt - 100.0 {
                            self.ab_climb = false;
                        }
                    }
                    PPhase::LowPass => {
                        let end = self.rw.clone().unwrap();
                        let s = -end.along(p);
                        let x = end.cross(p);
                        let r_turn = ac.tas().powi(2) / (GRAVITY * 28f32.to_radians().tan());
                        if !self.lp_inbound && (p.distance(self.gate) < r_turn * 1.2 || (s > 5000.0 && x.abs() < 1500.0 && wrap_pi(end.course() - ac.heading()).abs() < 0.8)) {
                            self.lp_inbound = true;
                        }
                        if !self.lp_inbound {
                            hdg = crab(heading_of(self.gate - p), tas, wind);
                            alt = fe + 450.0;
                            spd = 300.0 * KT;
                            *status = format!("Setting up a low pass RWY {} — {:.1} km", end.name, p.distance(self.gate) / 1000.0);
                        } else {
                            // Intercept gain matched to the (large) turn radius, or it orbits.
                            hdg = crab(end.course() + (-x / r_turn.max(1200.0)).atan().clamp(-0.6, 0.6), tas, wind);
                            let lined = x.abs() < 500.0 && wrap_pi(end.course() - ac.heading()).abs() < 0.35;
                            alt = fe + if lined && s < 6000.0 && s > -end.len - 800.0 { 120.0 } else { 350.0 };
                            spd = if lined { 450.0 } else { 300.0 } * KT;
                            *status = format!("Fast low pass RWY {} — {:.0} kt", end.name, ias / KT);
                            if s < -end.len - 1500.0 || (s < -end.len && x.abs() > 3000.0) {
                                // Pull up with reheat, then come round to land.
                                self.low_pass_done = true;
                                self.lp_inbound = false;
                                self.ab_climb = true;
                                ev.push(format!("low pass complete at {:.0} kt, {:.0} ft", ias / KT, ac.pos.y / FT));
                                self.start_approach(world, ac);
                            }
                        }
                    }
                    _ => {
                        // Position: to the gate, then intercept the final course.
                        let end = self.rw.clone().unwrap();
                        let s = -end.along(p);
                        let x = end.cross(p);
                        let faf_alt = (pf.final_dist + pf.aim) * self.gs_angle.tan();
                        let to_gate = p.distance(self.gate);
                        let inbound = wrap_pi(end.course() - ac.heading()).abs() < 100f32.to_radians();
                        let capture = to_gate < pf.turn_r.max(600.0) * 1.2
                            || (s > pf.final_dist * 0.5 && x.abs() < s * 0.8 && inbound && s < pf.final_dist + pf.turn_r * 1.6);
                        if capture || self.phase_t < 0.0 {
                            self.phase_t = -1.0; // latched: intercepting
                            self.ab_climb = false; // (after a low pass: no more reheat)
                            let l = (pf.turn_r * 0.35).max(250.0);
                            hdg = crab(end.course() + (-x / l).atan().clamp(-0.8, 0.8), tas, wind);
                            *status = format!("Intercepting final RWY {} — {:.1} nm", end.name, s / NM);
                        } else {
                            hdg = crab(heading_of(self.gate - p), tas, wind);
                            *status = format!("Returning to {} — RWY {} {:.1} km", self.field_name, end.name, to_gate / 1000.0);
                        }
                        alt = fe + faf_alt.max(300.0);
                        if self.ab_climb && ay < 900.0 {
                            alt = fe + 1000.0;
                        } else {
                            self.ab_climb = false;
                        }
                        spd = (self.vapp + if def.category == Category::Airliner { 35.0 } else { 20.0 } * KT).max(1.3 * def.stall_speed(ac.surfaces().flaps));
                        if self.low_pass {
                            spd = 300.0 * KT;
                            ctl.gear_down = false;
                            self.set_flaps(def, ctl, 0, t);
                        } else if s < pf.final_dist + 6000.0 && s > 0.0 && x.abs() < 6000.0 {
                            self.set_flaps(def, ctl, pf.app_flap, t);
                        } else {
                            ctl.flaps = flap_lever(def, self.flap_idx);
                        }
                        if s < pf.final_dist + 2500.0 && self.phase_t < 0.0 && !self.low_pass && ias < def.vle_kt * KT * 0.95 {
                            ctl.gear_down = true;
                        }
                        // Glidepath capture from below.
                        let gp = (s + pf.aim) * self.gs_angle.tan();
                        if self.phase_t < 0.0 && x.abs() < 400.0 && ay >= gp - 15.0 && s < pf.final_dist + 3000.0 {
                            ev.push(format!("established RWY {} at {:.1} nm, {:.0} ft (glidepath {:.1}°)", end.name, s / NM, ac.pos.y / FT, self.gs_angle.to_degrees()));
                            self.set_phase(PPhase::Final, t);
                        }
                    }
                }
                if let Some((h, until)) = self.avoid {
                    if until > t && self.phase != PPhase::Position {
                        hdg = crab(h, tas, wind);
                        alt = alt.max(ac.pos.y);
                    }
                }
                ap.hdg = Some(wrap_pi(hdg));
                ap.alt = Some(alt.max(self.alt_floor));
                ap.vs = None;
                ap.ias = Some(spd);
                if !ap.update(def, ac, ctl, dt) {
                    ap.engage_hold_current(ac);
                }
                if self.ab_climb && self.phase != PPhase::LowPass && ay < 900.0 && ac.vertical_speed() > -1.0 {
                    ctl.throttle = 1.0;
                }
            }
            PPhase::Final => {
                let end = self.rw.clone().unwrap();
                let s = -end.along(p);
                let x = end.cross(p);
                let gp = (s + pf.aim) * self.gs_angle.tan() + def.gear_height;
                if self.low_pass {
                    // Down the approach path to 100 m, fast and clean, then reheat climb away.
                    if !ap.engaged {
                        ap.engage_hold_current(ac);
                    }
                    let l = 1500.0;
                    ap.hdg = Some(crab(end.course() + (-x / l).atan().clamp(-0.4, 0.4), tas, wind));
                    ap.alt = Some(fe + gp.max(100.0).min(ay.max(100.0)));
                    ap.vs = None;
                    ap.ias = Some(420.0 * KT);
                    ctl.gear_down = false;
                    self.set_flaps(def, ctl, 0, t);
                    if !ap.update(def, ac, ctl, dt) {
                        ap.engage_hold_current(ac);
                    }
                    *status = format!("Fast low pass RWY {} — {:.0} kt, {:.0} ft", end.name, ias / KT, wheels / FT);
                    if s < -end.len - 500.0 {
                        ev.push(format!("low pass RWY {} at {:.0} kt, {:.0} ft — pulling up", end.name, ias / KT, wheels / FT));
                        self.low_pass = false;
                        self.low_pass_done = true;
                        self.ab_climb = true;
                        self.start_approach(world, ac);
                    }
                    return;
                }
                // Gear below its limit speed (the stabilised gate catches it if still up).
                if ias < def.vle_kt * KT * 0.95 {
                    ctl.gear_down = true;
                }
                self.set_flaps(def, ctl, if s < pf.final_dist * 0.8 { pf.land_flap } else { pf.app_flap }, t);
                // Descent rate for the glidepath from the ground speed along the runway (headwind → less).
                let g_along = vec2(ac.vel.x, ac.vel.z).dot(end.dir).max(20.0);
                let vs_gp = -g_along * self.gs_angle.tan();
                let l = (pf.turn_r * 0.25).max(200.0).min(s * 0.3 + 120.0);
                // Track the centreline: the cross-track law gives a track, the crab angle the heading.
                let hdg = crab(end.course() + (-x / l).atan().clamp(-0.5, 0.5), tas, wind);
                if !ap.engaged {
                    ap.engage_hold_current(ac);
                }
                ap.hdg = Some(hdg);
                ap.alt = None;
                ap.vs = Some((vs_gp + 0.12 * (gp - ay)).clamp(vs_gp * 1.8 - 1.0, 1.5));
                ap.ias = Some(self.vapp);
                if !ap.update(def, ac, ctl, dt) {
                    ap.engage_hold_current(ac);
                }
                *status = format!("Final approach RWY {} — {:.1} nm, {:.0} ft", end.name, s / NM, wheels / FT);
                // Stabilised-approach gate below 500 ft.
                if wheels < 150.0 && wheels > pf.flare_h + 5.0 {
                    let unstable = x.abs() > pf.stab_x
                        || (ay - gp).abs() > 35.0
                        || self.ias_f < self.vapp - 10.0 * KT
                        || self.ias_f > self.vapp + 25.0 * KT
                        || ac.gear_pos() < 0.99;
                    if unstable {
                        ev.push(format!(
                            "go-around: unstable at {:.0} ft (x {:.0} m, gp {:+.0} m, {:.0} kt)",
                            wheels / FT,
                            x,
                            ay - gp,
                            ias / KT
                        ));
                        stats.go_arounds += 1;
                        ap.disengage();
                        self.rot_pitch = ac.pitch();
                        self.set_phase(PPhase::GoAround, t);
                        return;
                    }
                }
                // Flare earlier on a steep / fast approach (fast jets: ~4 s of sink).
                let lead = 3.0;
                if wheels < pf.flare_h.max(-ac.vertical_speed() * lead) {
                    ap.disengage();
                    self.flare_h0 = wheels;
                    self.flare_vs0 = -ac.vertical_speed();
                    self.flare_pitch = ac.pitch();
                    self.sink_i = 0.0;
                    hands.pitch_i = 0.0;
                    self.set_phase(PPhase::Flare, t);
                }
                if s < -end.len * 0.5 {
                    ev.push("go-around: long".into());
                    stats.go_arounds += 1;
                    self.rot_pitch = ac.pitch();
                    self.set_phase(PPhase::GoAround, t);
                }
            }
            PPhase::Flare => {
                ap.disengage();
                let end = self.rw.clone().unwrap();
                let h = wheels.max(0.0);
                let fast_jet = def.category == Category::FastJet;
                // Retard: jets at the flare, props a little later; the fast jet
                // keeps approach power to the ground (a power-on touchdown,
                // no speed — and so no lift — bled off in the flare).
                if fast_jet {
                    if h < 1.0 {
                        ctl.throttle = approach_to(ctl.throttle, 0.0, 0.6, dt);
                    }
                } else if def.engine_kind == EngineKind::Jet || h < pf.flare_h * 0.5 {
                    ctl.throttle = approach_to(ctl.throttle, 0.0, 0.6, dt);
                }
                let target_vs = -(0.45 + 0.28 * h).min(4.0);
                let err = target_vs - ac.vertical_speed();
                // Pitch per m/s of sink error: gentler for faster aircraft (vs responds faster to pitch).
                let k_vs = (1.8 / ac.tas().max(20.0)).clamp(0.015, 0.06);
                self.sink_i = (self.sink_i + 0.8 * k_vs * err * dt).clamp(-0.05, 0.12);
                let max_pitch = if def.category == Category::Airliner { 8f32.to_radians() } else { 13f32.to_radians() };
                // Ease the nose up to the touchdown attitude as the wheels near the runway.
                let td = touchdown_pitch(def);
                let frac = (1.0 - h / pf.flare_h).clamp(0.0, 1.0);
                let min_pitch = self.flare_pitch + (td - self.flare_pitch).max(0.0) * frac;
                let pitch_cmd = (self.flare_pitch + k_vs * err + self.sink_i).max(min_pitch).clamp(-0.05, max_pitch);
                ctl.elevator = if def.category == Category::FastJet && Hands::fbw(def, ac) {
                    // Fast FBW jet: fly the sink-rate profile straight through the
                    // load-factor law (pitch attitude alone lags far too much at
                    // 150 kt off a steep final), with a tail-strike attitude limit.
                    let nz_neutral = 1.0 / ac.roll().cos().max(0.5);
                    // Sink shrinking linearly with height from the flare entry
                    // (h0, vs0) to a firm 1.2 m/s at the runway (no long float on
                    // a short runway), with feed-forward.
                    let (h0, s0) = (self.flare_h0.max(1.0), self.flare_vs0.max(1.3));
                    let k = (s0 - 1.2) / h0;
                    let tgt = -(1.2 + k * h);
                    let a_cmd = 2.0 * (tgt - ac.vertical_speed()) + k * (-ac.vertical_speed()).max(0.0);
                    let mut dnz = (a_cmd / GRAVITY).clamp(-0.25, 0.9);
                    if ac.pitch() > td + 3f32.to_radians() {
                        dnz = dnz.min(-0.6 * (ac.pitch() - td - 3f32.to_radians()) * ac.tas() / GRAVITY);
                    }
                    Hands::nz_stick(def, nz_neutral, dnz)
                } else {
                    hands.elevator(def, ac, pitch_cmd, dt)
                };
                // De-crab: rudder lines the nose up with the runway, the wing
                // goes down into the wind to stop the drift (sideslip).
                // Less bank in the last couple of metres (wingtip / nacelle clearance).
                let bank_max = if def.category == Category::Airliner { 6.0 } else { 9.0 } * (h / 2.5).clamp(0.4, 1.0);
                let (rud, ail) = decrab(def, ac, &end, p, bank_max);
                ctl.rudder = rud;
                ctl.aileron = ail;
                *status = format!("Flare — {:.0} ft", h / FT);
                if ac.on_ground {
                    stats.landings += 1;
                    let d = end.along(p);
                    ev.push(format!(
                        "touchdown RWY {} at {}: {:.0} m past threshold, {:.0} kt, sink {:.1} m/s",
                        end.name,
                        self.field_name,
                        d,
                        ias / KT,
                        stats.worst_touchdown
                    ));
                    self.touch_t = t;
                    self.set_phase(PPhase::Rollout, t);
                }
                if t - self.phase_t > 25.0 {
                    ev.push("go-around: floated".into());
                    stats.go_arounds += 1;
                    self.rot_pitch = ac.pitch();
                    self.set_phase(PPhase::GoAround, t);
                }
            }
            PPhase::Rollout => {
                ap.disengage();
                let end = self.rw.clone().unwrap();
                let g = gs(ac);
                ctl.throttle = 0.0;
                ctl.spoilers = if def.has_spoilers { 1.0 } else { 0.0 };
                let rev = def.has_reverser && g > 60.0 * KT && ac.on_ground;
                ctl.reverse = def.has_reverser && g > 35.0 * KT && ac.on_ground;
                if rev {
                    ctl.throttle = if def.engine_kind == EngineKind::Jet { 0.8 } else { 0.6 };
                }
                let since = t - self.touch_t;
                let tail = def.taildragger;
                ctl.elevator = if tail {
                    if g < 45.0 * KT { 1.0 } else if ias < def.vs0_kt * KT * 1.05 { 0.4 } else { 0.0 }
                } else if def.category == Category::Airliner {
                    0.0
                } else {
                    -0.1
                };
                if def.category != Category::Airliner && since > 1.0 {
                    // Flaps up for braking.
                    self.set_flaps(def, ctl, 0, t);
                }
                ctl.brakes = if since < 1.0 {
                    0.0
                } else if tail {
                    if g < 45.0 * KT { 0.7 } else { 0.25 }
                } else {
                    pf.brakes
                };
                // Upwind aileron (crosswind from the left → left aileron), more as we slow.
                let xw = wind.dot(end.right());
                let into = -(xw / 10.0).clamp(-1.0, 1.0) * (1.0 - g / (pf.vr * 1.2)).clamp(0.3, 1.0);
                ctl.aileron = (into - 2.0 * ac.roll()).clamp(-1.0, 1.0);
                ctl.rudder = steer(def, ac, end.course(), end.cross(p));
                *status = format!("Rollout{} — {:.0} kt", if rev { ", reverse" } else { "" }, g / KT);
                if !ac.on_ground && since > 1.0 && wheels > 5.0 {
                    // Bounced badly: go around.
                    self.rot_pitch = ac.pitch();
                    self.set_phase(PPhase::GoAround, t);
                }
                if g < pf.taxi * 1.3 {
                    ctl.reverse = false;
                    stats.cycles += 1;
                    ev.push(format!("rollout complete, {:.0} m remaining", end.len - end.along(p)));
                    // Take off into wind next time: if that is the way we landed,
                    // turn round here and backtrack to the threshold.
                    let next = self.field.as_ref().map(|f| best_takeoff_end(world, f));
                    self.backtrack = next.is_some_and(|n| n.dir.dot(end.dir) > 0.0);
                    // Turn round a little way on (room to move over to the side first).
                    self.turn_at = self.backtrack.then_some(end.len - end.along(p) - 140.0);
                    if self.backtrack {
                        ev.push(format!("backtracking RWY {} to depart into wind", end.name));
                    }
                    self.set_phase(PPhase::Taxi, t);
                }
            }
            PPhase::Taxi | PPhase::Turnaround | PPhase::LineUp => {
                ap.disengage();
                let end = self.rw.clone().unwrap();
                ctl.spoilers = 0.0;
                ctl.reverse = false;
                self.set_flaps(def, ctl, 0, t);
                let g = gs(ac);
                let remain = end.len - end.along(p);
                let margin = if def.category == Category::Airliner { 90.0 } else { 45.0 };
                // Pure pursuit of a point on the centreline (or offset from it) a little ahead.
                let look = if def.category == Category::Airliner { 45.0 } else { 25.0 };
                let along = end.along(p);
                let pursue = |x_t: f32| heading_of(end.thr + end.dir * (along + look) + end.right() * x_t - p);
                let (course, v_t) = match self.phase {
                    PPhase::Taxi => {
                        *status = format!("Taxiing to the end of RWY {} — {:.0} m", end.name, remain.max(0.0));
                        // Move over to the right-hand side before the left turn round.
                        self.turn_dir = -1.0;
                        // Backtracking: turn round a little way on from where we stopped.
                        let stop = self.turn_at.unwrap_or(margin).max(margin);
                        let side = if remain < stop + 200.0 { (end.width * 0.5 - 2.0 - def.wingspan * 0.1).max(0.0).min(12.0) } else { 0.0 };
                        if remain < stop {
                            self.set_phase(PPhase::Turnaround, t);
                        }
                        // Fast backtrack, slowing for the turn at the end.
                        (pursue(side), pf.taxi.min(2.5 + (remain - margin).max(0.0) * 0.04))
                    }
                    PPhase::Turnaround => {
                        *status = "Turning around".into();
                        let back = end.reversed();
                        if wrap_pi(back.course() - ac.heading()).abs() < 0.5 {
                            self.rw = Some(back);
                            self.turn_at = None;
                            if self.backtrack {
                                // Now taxi back down the runway to its far end, and turn round again.
                                self.backtrack = false;
                                self.set_phase(PPhase::Taxi, t);
                            } else {
                                self.set_phase(PPhase::LineUp, t);
                            }
                        }
                        (ac.heading() + self.turn_dir * 1.2, 2.5)
                    }
                    _ => {
                        let herr = wrap_pi(end.course() - ac.heading());
                        let x = end.cross(p);
                        *status = format!("Lining up RWY {}", end.name);
                        if herr.abs() < 0.04 && x.abs() < 3.0 && t - self.phase_t > 3.0 || t - self.phase_t > 120.0 {
                            self.set_phase(PPhase::Hold, t);
                        }
                        (pursue(0.0), if x.abs() > 8.0 { 4.0 } else { 3.0 })
                    }
                };
                let x = 0.0;
                ctl.rudder = if self.phase == PPhase::Turnaround { self.turn_dir } else { steer(def, ac, course, x) };
                let thr_max = if def.engine_kind == EngineKind::Jet { 0.45 } else { 0.35 };
                ctl.throttle = ((v_t - g) * 0.15 + if def.engine_kind == EngineKind::Jet { 0.12 } else { 0.08 }).clamp(0.0, thr_max);
                if g > v_t + 1.0 {
                    ctl.throttle = 0.0;
                }
                ctl.brakes = ((g - v_t - 0.5) * 0.3).clamp(0.0, 0.8);
                ctl.elevator = if def.taildragger { 1.0 } else { 0.0 };
                // Taxiing in a crosswind: aileron into the wind.
                let xw = wind.dot(dir_of(ac.heading()).perp());
                ctl.aileron = (-(xw / 10.0).clamp(-1.0, 1.0) - 2.0 * ac.roll()).clamp(-1.0, 1.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runway_numbers_and_geometry() {
        assert_eq!(rwy_number(0.0), "36");
        assert_eq!(rwy_number(90f32.to_radians()), "09");
        assert_eq!(rwy_number(-93f32.to_radians()), "27");
        let rw = Runway { a: vec2(0.0, 0.0), b: vec2(1000.0, 0.0), width: 30.0, name: String::new(), paved: true, ..Default::default() };
        let e = RunwayEnd::of(&rw, false);
        assert_eq!(e.name, "09");
        assert!((e.along(vec2(250.0, 0.0)) - 250.0).abs() < 1e-3);
        // East-bound: right of the centreline is south (+z).
        assert!(e.cross(vec2(100.0, 20.0)) > 19.0);
        let r = e.reversed();
        assert_eq!(r.name, "27");
        assert!((r.thr - rw.b).length() < 1e-3);
        assert!((wrap_pi(heading_of(dir_of(1.0)) - 1.0)).abs() < 1e-5);
    }

    #[test]
    fn glider_polar_and_speed_to_fly() {
        let def = crate::flight::find("swift").unwrap();
        let mut polar = Vec::new();
        let mut v = def.vs1_kt * KT * 1.05;
        while v < def.vne_kt * KT * 0.8 {
            let tp = def.trim_point_at(v, 0.0, 800.0, true);
            polar.push((v, -tp.gamma.sin() * crate::flight::tas_from_cas(v, 800.0)));
            v += 0.5;
        }
        let ld = polar.iter().map(|(v, s)| v / s).fold(0.0, f32::max);
        assert!((30.0..45.0).contains(&ld), "L/D {ld}");
        // MacCready: a stronger expected climb means flying faster between thermals.
        let stf = |m: f32| polar.iter().map(|&(v, s)| (v, v / (s + m))).max_by(|a, b| a.1.total_cmp(&b.1)).unwrap().0;
        assert!(stf(3.0) > stf(1.0) + 2.0 && stf(1.0) > stf(0.0));
    }
}
