//! Radar vectoring to an ILS final, the way a UK approach director does it:
//! a heading towards a downwind / base point when needed, a base leg to a
//! point 3 NM before the intercept, then a 30° closing heading onto the
//! localiser about 11 NM from touchdown, at an altitude that is below the
//! glidepath there (the glideslope is then captured from below, after the
//! localiser), cleared for the approach with it.
//!
//! Geometry in the world frame (+X east, −Z north); headings true, degrees.

use glam::Vec2;

pub const NM: f32 = 1852.0;
const FT: f32 = 0.3048;

/// Intercept point on the extended centreline (distance from the threshold).
pub const D_INTERCEPT: f32 = 11.0 * NM;
/// Closing angle onto the localiser.
pub const CUT_DEG: f32 = 30.0;
/// Length of the closing leg.
const LEG: f32 = 3.0 * NM;

/// The final approach being vectored for.
#[derive(Clone, Debug, PartialEq)]
pub struct Final {
    /// Landing threshold (world) and landing direction (unit).
    pub thr: Vec2,
    pub dir: Vec2,
    /// Threshold elevation (ft) and glidepath angle (deg).
    pub elev_ft: f32,
    pub gs_deg: f32,
}

pub fn heading_of(v: Vec2) -> f32 {
    v.x.atan2(-v.y).to_degrees().rem_euclid(360.0)
}

pub fn dir_of(deg: f32) -> Vec2 {
    let r = deg.to_radians();
    Vec2::new(r.sin(), -r.cos())
}

/// Signed smallest difference a − b in degrees (−180..180).
pub fn hdg_diff(a: f32, b: f32) -> f32 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

impl Final {
    pub fn course(&self) -> f32 {
        heading_of(self.dir)
    }
    fn right(&self) -> Vec2 {
        Vec2::new(-self.dir.y, self.dir.x)
    }
    /// Distance before the threshold along the approach (+ = on the approach side).
    pub fn along(&self, p: Vec2) -> f32 {
        (self.thr - p).dot(self.dir)
    }
    /// Lateral offset (+ = right of the course, looking along the landing direction).
    pub fn lateral(&self, p: Vec2) -> f32 {
        (p - self.thr).dot(self.right())
    }
    /// Intercept altitude (ft, rounded down to 500 ft): ≥ 100 ft below the
    /// glidepath half a mile inside the intercept point, so the glideslope
    /// is joined from below a couple of miles after the localiser.
    pub fn intercept_alt_ft(&self) -> f32 {
        let d = D_INTERCEPT - 0.5 * NM;
        let gp = self.elev_ft + 50.0 + d * self.gs_deg.to_radians().tan() / FT;
        (((gp - 100.0) / 500.0).floor() * 500.0).max(self.elev_ft + 1500.0).max(1500.0)
    }
}

/// The plan for one aircraft: waypoints to fly headings to, then the intercept.
#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub wps: Vec<Vec2>,
    /// Which side of the centreline the aircraft joins from (+1 right, −1 left).
    pub side: f32,
    /// Closing heading (deg true).
    pub intercept_hdg: f32,
    pub alt_ft: f32,
}

pub fn plan(f: &Final, pos: Vec2) -> Plan {
    let c = f.lateral(pos);
    let a = f.along(pos);
    let side = if c.abs() < 0.5 * NM { if a > 0.0 { 1.0 } else { c.signum().max(0.0) * 2.0 - 1.0 } } else { c.signum() };
    let r = f.right();
    let (s, co) = CUT_DEG.to_radians().sin_cos();
    // Closing leg direction: along the landing direction, towards the centreline.
    let u = f.dir * co - r * side * s;
    let i = f.thr - f.dir * D_INTERCEPT;
    let j = i - u * LEG;
    let a_j = f.along(j);
    let mut wps = Vec::new();
    if a < a_j + 2.0 * NM {
        // Too close in (or beyond the field): downwind on our side, then base.
        let lat = c.abs().max(5.0 * NM) * side;
        let w = f.thr - f.dir * (a_j + 2.5 * NM) + r * lat;
        if a < 0.0 {
            // Beyond the field: first abeam the threshold.
            wps.push(f.thr + f.dir * 2.0 * NM + r * lat);
        }
        wps.push(w);
    }
    wps.push(j);
    Plan { wps, side, intercept_hdg: heading_of(u), alt_ft: f.intercept_alt_ft() }
}

/// A heading that tracks from `p` to `q` with the wind (m/s, world) at true airspeed `tas`.
pub fn heading_to(p: Vec2, q: Vec2, wind: Vec2, tas: f32) -> f32 {
    let d = (q - p).normalize_or_zero();
    if d == Vec2::ZERO {
        return 0.0;
    }
    let cross = d.perp_dot(wind);
    let wca = (cross / tas.max(30.0)).clamp(-0.5, 0.5).asin();
    // Turning the nose into the wind: rotate the track by −wca (perp_dot sign).
    (heading_of(d) - wca.to_degrees()).rem_euclid(360.0)
}

/// Progress through a plan.
#[derive(Clone, Debug, PartialEq)]
pub struct Director {
    pub fin: Final,
    pub plan: Plan,
    /// Next waypoint (index into `plan.wps`; == len → intercept heading given).
    pub wp: usize,
    /// Last heading issued.
    pub issued: Option<f32>,
    pub intercept_given: bool,
}

/// What the director wants to say now.
#[derive(Clone, Debug, PartialEq)]
pub enum Call {
    /// A new heading (and a turn direction, None = "fly heading").
    Heading { hdg: f32, turn: Option<bool> },
    /// The closing heading with the approach clearance.
    Intercept { hdg: f32, turn: Option<bool> },
}

impl Director {
    pub fn new(fin: Final, pos: Vec2) -> Director {
        let plan = plan(&fin, pos);
        Director { fin, plan, wp: 0, issued: None, intercept_given: false }
    }

    /// Distance from the threshold (NM) along the approach.
    pub fn track_miles(&self, pos: Vec2) -> f32 {
        let mut d = 0.0;
        let mut p = pos;
        for w in &self.plan.wps[self.wp.min(self.plan.wps.len())..] {
            d += p.distance(*w);
            p = *w;
        }
        (d + p.distance(self.fin.thr)) / NM
    }

    /// Advance with the aircraft's position, heading (deg true), TAS and the wind.
    pub fn update(&mut self, pos: Vec2, hdg: f32, tas: f32, wind: Vec2) -> Option<Call> {
        if self.intercept_given {
            return None;
        }
        let turn_of = |from: f32, to: f32| -> Option<bool> {
            let d = hdg_diff(to, from);
            if d.abs() < 15.0 { None } else { Some(d > 0.0) }
        };
        // Sequence the waypoints: within the turn anticipation, or passed abeam.
        while self.wp < self.plan.wps.len() {
            let w = self.plan.wps[self.wp];
            let next_dir = if self.wp + 1 < self.plan.wps.len() { (self.plan.wps[self.wp + 1] - w).normalize_or_zero() } else { dir_of(self.plan.intercept_hdg) };
            let in_dir = (w - pos).normalize_or_zero();
            let turn = in_dir.angle_to(next_dir).abs();
            let r_turn = tas.max(60.0).powi(2) / (9.81 * 25f32.to_radians().tan());
            let lead = (r_turn * (turn * 0.5).tan()).clamp(0.3 * NM, 2.0 * NM);
            let passed = (w - pos).dot(dir_of(hdg)) < 0.0 && pos.distance(w) < 3.0 * NM;
            if pos.distance(w) < lead || passed {
                self.wp += 1;
            } else {
                break;
            }
        }
        if self.wp >= self.plan.wps.len() {
            self.intercept_given = true;
            let h = self.plan.intercept_hdg;
            return Some(Call::Intercept { hdg: h, turn: turn_of(hdg, h) });
        }
        // The first leg: straight at the point; later legs: the planned leg
        // (downwind → base), corrected back onto it when well off.
        // The first leg: straight at the point; later legs (downwind → base):
        // the planned leg, corrected back onto it when well off — and left
        // alone in between (no see-sawing).
        let want = if self.wp == 0 {
            Some(heading_to(pos, self.plan.wps[0], wind, tas))
        } else {
            let (a, b) = (self.plan.wps[self.wp - 1], self.plan.wps[self.wp]);
            let leg = (b - a).normalize_or_zero();
            let xtk = (pos - a).perp_dot(leg);
            let along = (pos - a).dot(leg);
            let base = heading_to(a, b, wind, tas);
            if along < 2.0 * NM || xtk.abs() < 0.3 * NM {
                // Still turning onto it (the turn starts early), or on it.
                Some(base)
            } else if xtk.abs() > 1.2 * NM {
                Some((base + (xtk / NM * 10.0).clamp(-25.0, 25.0)).rem_euclid(360.0))
            } else {
                None
            }
        };
        // Corrections only when they matter (not close to the turn point).
        let near = pos.distance(self.plan.wps[self.wp]) < 2.5 * NM;
        let want = want?;
        let need = match self.issued {
            None => true,
            Some(h) => hdg_diff(want, h).abs() > if near { 35.0 } else { 18.0 },
        };
        if need {
            let h = (want / 5.0).round() * 5.0;
            self.issued = Some(h);
            return Some(Call::Heading { hdg: h, turn: turn_of(hdg, h) });
        }
        None
    }
}
