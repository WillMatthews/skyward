//! FMS-lite: a flight plan (origin → waypoints → destination + approach),
//! LNAV leg guidance with fly-by sequencing, a VNAV descent path to the
//! runway, and distance / ETA / fuel predictions.
//!
//! Legs are track-to-fix (TF) great-circle legs flown in the local frame.
//! An approach appends its fixes — the intermediate fix (IF, 10 NM), the
//! final approach fix (FAF, 5 NM) and the runway threshold — on the final
//! course; an ILS approach auto-tunes NAV 1, an RNAV approach flies the
//! same geometry on a synthetic LNAV / VNAV path (3°, 50 ft over the
//! threshold).

use super::db::{Ils, Navaid};
use super::geo::{self, LL, NM};
use super::radio::RadioEnv;
use glam::Vec2;

const FT: f32 = 0.3048;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WptKind {
    Navaid,
    Airport,
    User,
    /// Approach fix (IF / FAF).
    Fix,
    /// Runway threshold (the last approach leg).
    Runway,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Wpt {
    pub ident: String,
    pub ll: LL,
    pub kind: WptKind,
    /// Altitude constraint (m): "at" for approach fixes, "at or above" en route.
    pub alt: Option<f32>,
}

impl Wpt {
    pub fn new(ident: impl Into<String>, ll: LL, kind: WptKind) -> Wpt {
        Wpt { ident: ident.into(), ll, kind, alt: None }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApprKind {
    Ils,
    Rnav,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Approach {
    pub kind: ApprKind,
    pub airport: String,
    pub runway: String,
    pub thr: LL,
    /// Final course (true, rad).
    pub course: f32,
    /// Path angle (rad).
    pub angle: f32,
    pub thr_elev: f32,
    /// The localizer to tune (ILS approaches).
    pub ils: Option<Navaid>,
}

impl Approach {
    pub fn name(&self) -> String {
        format!("{} {}", if self.kind == ApprKind::Ils { "ILS" } else { "RNAV" }, self.runway)
    }
    /// An approach on an ILS (or the same geometry as RNAV).
    pub fn from_ils(n: &Navaid, kind: ApprKind) -> Option<Approach> {
        let i: &Ils = n.ils.as_ref()?;
        Some(Approach {
            kind,
            airport: i.airport.clone(),
            runway: i.runway.clone(),
            thr: i.thr,
            course: i.course,
            angle: if kind == ApprKind::Rnav { 3f32.to_radians() } else { i.gs_angle },
            thr_elev: i.thr_elev,
            ils: (kind == ApprKind::Ils).then(|| n.clone()),
        })
    }
}

/// One line of the flight-plan page.
#[derive(Clone, Debug, PartialEq)]
pub struct Pred {
    pub ident: String,
    /// Leg course (true, rad) and length (m).
    pub course: f32,
    pub leg: f32,
    /// Distance from the aircraft along the route (m), time (s), fuel left (kg).
    pub dist: f32,
    pub eta: f32,
    pub efob: Option<f32>,
    pub alt: Option<f32>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Plan {
    pub origin: Option<Wpt>,
    pub dest: Option<Wpt>,
    /// En-route waypoints followed by the approach fixes.
    pub legs: Vec<Wpt>,
    pub approach: Option<Approach>,
    /// Index into `legs` of the waypoint flown to.
    pub active: usize,
    /// Start of the active leg after a direct-to (else the previous waypoint).
    pub from: Option<LL>,
    /// Cruise level (m).
    pub cruise: f32,
    /// Bumped on every edit (displays cache on it).
    pub version: u64,
}

/// Lateral guidance on the active leg.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Lnav {
    pub wpt: String,
    /// Desired track (true, rad).
    pub track: f32,
    /// Cross-track (m, + = the track is to the right).
    pub xtk: f32,
    /// Along-track distance to the waypoint (m).
    pub to_go: f32,
    /// The next leg's track.
    pub next: Option<f32>,
}

/// Vertical guidance.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Vnav {
    /// Path altitude here (m), capped at cruise.
    pub alt: f32,
    /// Path angle (rad, + = descending).
    pub angle: f32,
    /// Distance to the threshold along the route (m).
    pub to_thr: f32,
    /// Distance to the top of descent (m, ≤ 0 past it).
    pub to_tod: f32,
    pub cruise: f32,
}

impl Plan {
    fn edited(&mut self) {
        self.version = self.version.wrapping_add(1);
        self.active = self.active.min(self.legs.len());
    }

    pub fn is_empty(&self) -> bool {
        self.legs.is_empty()
    }

    /// Number of en-route waypoints (before the approach fixes).
    pub fn enroute_len(&self) -> usize {
        self.legs.iter().position(|w| matches!(w.kind, WptKind::Fix | WptKind::Runway)).unwrap_or(self.legs.len())
    }

    pub fn set_origin(&mut self, w: Option<Wpt>) {
        self.origin = w;
        self.edited();
    }

    pub fn set_dest(&mut self, w: Option<Wpt>) {
        if self.dest.as_ref().map(|d| &d.ident) != w.as_ref().map(|d| &d.ident) {
            self.set_approach(None);
        }
        self.dest = w;
        self.edited();
    }

    /// Insert an en-route waypoint at `i` (clamped before the approach).
    pub fn insert(&mut self, i: usize, w: Wpt) {
        let i = i.min(self.enroute_len());
        self.legs.insert(i, w);
        if i < self.active || (i == self.active && self.from.is_some() && self.active > 0) {
            self.active += 1;
        }
        self.edited();
    }

    pub fn remove(&mut self, i: usize) {
        if i < self.legs.len() {
            let w = self.legs.remove(i);
            if matches!(w.kind, WptKind::Fix | WptKind::Runway) {
                // Removing an approach fix removes the approach.
                self.set_approach(None);
            }
            if i < self.active {
                self.active -= 1;
            }
            self.edited();
        }
    }

    /// Move waypoint `i` (map drag). Approach fixes stay where they are.
    pub fn move_to(&mut self, i: usize, ll: LL) {
        if let Some(w) = self.legs.get_mut(i).filter(|w| !matches!(w.kind, WptKind::Fix | WptKind::Runway)) {
            w.ll = ll;
            if w.kind != WptKind::User {
                w.kind = WptKind::User;
                w.ident = format!("WP{:02}", i + 1);
            }
            self.edited();
        }
    }

    /// Direct to waypoint `i` from `here`.
    pub fn direct_to(&mut self, i: usize, here: LL) {
        if i < self.legs.len() {
            self.active = i;
            self.from = Some(here);
            self.edited();
        }
    }

    /// Replace the approach (and its fixes).
    pub fn set_approach(&mut self, a: Option<Approach>) {
        let n = self.enroute_len();
        self.legs.truncate(n);
        if let Some(a) = &a {
            let back = geo::wrap_2pi(a.course + std::f32::consts::PI);
            let fix = |d_nm: f32, name: &str| {
                let ll = geo::gc_dest(a.thr, back, (d_nm * NM) as f64);
                let alt = a.thr_elev + 50.0 * FT + d_nm * NM * a.angle.tan();
                Wpt { ident: name.to_string(), ll, kind: WptKind::Fix, alt: Some(((alt / FT) / 100.0).round() * 100.0 * FT) }
            };
            let rw = a.runway.clone();
            self.legs.push(fix(10.0, &format!("CI{rw}")));
            self.legs.push(fix(5.0, &format!("FF{rw}")));
            self.legs.push(Wpt { ident: format!("RW{rw}"), ll: a.thr, kind: WptKind::Runway, alt: Some(a.thr_elev + 50.0 * FT) });
        }
        self.approach = a;
        self.edited();
    }

    fn leg_from(&self, i: usize) -> Option<LL> {
        if i == self.active {
            if let Some(f) = self.from {
                return Some(f);
            }
        }
        if i == 0 { self.origin.as_ref().map(|o| o.ll) } else { self.legs.get(i - 1).map(|w| w.ll) }
    }

    /// Lateral guidance at `p` (world), or None without an active leg.
    pub fn lnav(&self, env: &dyn RadioEnv, p: Vec2) -> Option<Lnav> {
        let to = self.legs.get(self.active)?;
        let b = env.to_world(to.ll);
        let a = self.leg_from(self.active).map(|ll| env.to_world(ll)).unwrap_or(p);
        let d = b - a;
        let dir = if d.length() > 1.0 { d / d.length() } else { (b - p).normalize_or(glam::vec2(0.0, -1.0)) };
        let xtk = -(p - a).dot(geo::right_of(dir));
        let next = self.legs.get(self.active + 1).map(|n| geo::heading_of(env.to_world(n.ll) - b));
        Some(Lnav { wpt: to.ident.clone(), track: geo::heading_of(dir), xtk, to_go: (b - p).dot(dir), next })
    }

    /// Sequence the active waypoint: passed it (or inside the fly-by turn
    /// anticipation for ground speed `gs` m/s and bank `bank` rad).
    pub fn sequence(&mut self, env: &dyn RadioEnv, p: Vec2, gs: f32, bank: f32) {
        let Some(g) = self.lnav(env, p) else { return };
        // The runway is the last fix: no sequencing past it.
        if self.legs.get(self.active).is_some_and(|w| w.kind == WptKind::Runway) {
            return;
        }
        let lead = match g.next {
            Some(n) => {
                let r = gs * gs / (9.81 * bank.tan().max(0.1));
                r * (geo::wrap_pi(n - g.track).abs().min(2.4) / 2.0).tan()
            }
            None => 0.0,
        };
        if g.to_go <= lead.max(gs * 1.0) && self.active + 1 < self.legs.len() {
            self.active += 1;
            self.from = None;
            self.version = self.version.wrapping_add(1);
        } else if g.to_go < 0.0 && self.active + 1 >= self.legs.len() {
            // Past the last waypoint: stays active (the map shows the leg).
        }
    }

    /// Distance from `p` along the remaining route to waypoint `i` (m).
    pub fn dist_to(&self, env: &dyn RadioEnv, p: Vec2, i: usize) -> f32 {
        if i < self.active || self.legs.is_empty() {
            return 0.0;
        }
        let mut d = 0.0;
        let mut at = p;
        for w in &self.legs[self.active..=i.min(self.legs.len() - 1)] {
            let q = env.to_world(w.ll);
            d += at.distance(q);
            at = q;
        }
        d
    }

    /// The VNAV profile: a constant-angle path up from the threshold, capped
    /// at the cruise level.
    pub fn vnav(&self, env: &dyn RadioEnv, p: Vec2) -> Option<Vnav> {
        let a = self.approach.as_ref()?;
        let last = self.legs.iter().rposition(|w| w.kind == WptKind::Runway)?;
        if self.active > last {
            return None;
        }
        let to_thr = self.dist_to(env, p, last);
        let raw = a.thr_elev + 50.0 * FT + to_thr * a.angle.tan();
        let cruise = self.cruise.max(a.thr_elev + 300.0);
        let tod = (cruise - a.thr_elev - 50.0 * FT) / a.angle.tan().max(0.01);
        Some(Vnav { alt: raw.min(cruise), angle: a.angle, to_thr, to_tod: to_thr - tod, cruise })
    }

    /// Predictions per remaining waypoint at ground speed `gs` (m/s), fuel
    /// on board `fob` (kg) burning `flow` kg/h.
    pub fn predictions(&self, env: &dyn RadioEnv, p: Vec2, gs: f32, fob: Option<f32>, flow: f32) -> Vec<Pred> {
        let mut out = Vec::new();
        let mut at = p;
        let mut dist = 0.0;
        let gs = gs.max(30.0);
        for (i, w) in self.legs.iter().enumerate().skip(self.active) {
            let q = env.to_world(w.ll);
            let prev = self.leg_from(i).map(|ll| env.to_world(ll)).unwrap_or(at);
            dist += at.distance(q);
            let eta = dist / gs;
            out.push(Pred {
                ident: w.ident.clone(),
                course: geo::heading_of(q - prev),
                leg: prev.distance(q),
                dist,
                eta,
                efob: fob.map(|f| (f - flow * eta / 3600.0).max(0.0)),
                alt: w.alt,
            });
            at = q;
        }
        out
    }

    /// The route as (ident, world position) from the active waypoint on.
    pub fn route_world(&self, env: &dyn RadioEnv) -> Vec<(String, Vec2)> {
        self.legs.iter().skip(self.active).map(|w| (w.ident.clone(), env.to_world(w.ll))).collect()
    }

    /// All legs as world points (for the map: flown legs too).
    pub fn all_world(&self, env: &dyn RadioEnv) -> Vec<(String, Vec2)> {
        let mut v = Vec::new();
        if let Some(o) = &self.origin {
            v.push((o.ident.clone(), env.to_world(o.ll)));
        }
        v.extend(self.legs.iter().map(|w| (w.ident.clone(), env.to_world(w.ll))));
        v
    }
}
