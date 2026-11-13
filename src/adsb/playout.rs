//! Smooth motion from an irregular, delayed position feed.
//!
//! **Approach: buffered playout.** Each aircraft keeps its recent reports
//! and is drawn at `feed_now − delay` (the delay, ~10–15 s, is chosen by
//! [`super::LiveTraffic`] from the poll interval and the report ages, and
//! slews slowly). At that time there is almost always a report on each
//! side, so the position is *interpolated*: a cubic Hermite curve through
//! both positions and both reported velocities (ground speed + track,
//! vertical rate). It passes exactly through every report, follows turns
//! and level-offs, and — because the future is known — touch-downs flare
//! (the vertical rate at the first on-ground report is zero) and lift-offs
//! can be anticipated by the attitude synthesis (`synth.rs`).
//!
//! Why not extrapolate to "now"? Reports arrive every 5–10 s and are
//! already 0–5 s old: predicting 10 s ahead along the track misses every
//! turn, level-off and touch-down by hundreds of metres, and each new report
//! then has to drag the aircraft back. Nobody in the game can see the real
//! aircraft, so ten seconds of latency are invisible, while corrections are
//! not.
//!
//! When the feed is late (no report past the playout time) the track is
//! dead-reckoned along its track rate and vertical rate for up to
//! [`MAX_EXTRAPOLATE`] s; the first report after that may disagree with the
//! guess, and any jump between frames — from a late report, a re-ordered one
//! or an extrapolation — is absorbed into a correction offset that decays
//! over ~2 s at a rate bounded by a fraction of the aircraft's speed (it
//! never runs backwards). Velocities are limited so the cubic cannot
//! overshoot (Fritsch–Carlson style): stationary aircraft with GPS jitter
//! stay put, and implausible jumps (bad MLAT fixes) are dropped.
//!
//! Positions are geodetic; the curve is evaluated in a local east-up-south
//! tangent frame at the earlier report, as the multiplayer interpolator
//! does, so the floating origin never touches the stored data.

use super::feed::{Report, Source};
use skyward_net::interp::Tangent;
use std::collections::VecDeque;

const KT: f64 = 0.514_444;
const FT: f64 = 0.3048;
const FPM: f64 = 0.3048 / 60.0;

/// Longest dead reckoning past the newest report, s.
pub const MAX_EXTRAPOLATE: f64 = 30.0;
/// Longest backwards extension before the oldest report (new aircraft show
/// at once instead of waiting a playout delay), s.
pub const MAX_BACKFILL: f64 = 30.0;
/// Reports kept behind the playout time, s.
const KEEP_BEHIND: f64 = 20.0;
/// Faster than this between two reports is a bad fix, m/s.
const MAX_SPEED: f64 = 750.0;
/// Interpolate across gaps up to this long; beyond, start over.
const MAX_GAP: f64 = 150.0;
/// Correction decay time constant, s.
const CORR_TAU: f64 = 2.0;

/// One stored report (geodetic, SI units).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub lat: f64,
    pub lon: f64,
    /// Pressure altitude, m (None: on the ground or unknown).
    pub alt: Option<f64>,
    pub on_ground: bool,
    /// Velocity over the ground: east, south (m/s).
    pub vel: [f64; 2],
    /// Vertical speed, m/s.
    pub vs: f64,
    pub has_vel: bool,
    /// True heading, deg (when sent).
    pub heading: Option<f32>,
    /// Bank, deg (when sent).
    pub roll: Option<f32>,
    /// Track rate, rad/s (+ right).
    pub turn: Option<f64>,
}

impl Sample {
    pub fn from_report(r: &Report) -> Sample {
        let gs = r.gs_kt.unwrap_or(0.0) as f64 * KT;
        let trk = r.track_deg.or(r.true_heading_deg).unwrap_or(0.0) as f64;
        let (s, c) = trk.to_radians().sin_cos();
        let alt = if r.on_ground { None } else { r.alt_baro_ft.map(|a| a as f64 * FT).or(r.alt_geom_ft.map(|a| a as f64 * FT)) };
        Sample {
            t: r.t,
            lat: r.lat,
            lon: r.lon,
            alt,
            on_ground: r.on_ground,
            vel: [gs * s, -gs * c],
            vs: if r.on_ground { 0.0 } else { r.vrate_fpm.unwrap_or(0.0) as f64 * FPM },
            has_vel: r.gs_kt.is_some() && (r.track_deg.is_some() || r.true_heading_deg.is_some()),
            heading: r.true_heading_deg,
            roll: r.roll_deg,
            turn: r.track_rate.map(|x| (x as f64).to_radians()),
        }
    }

    pub fn speed(&self) -> f64 {
        (self.vel[0] * self.vel[0] + self.vel[1] * self.vel[1]).sqrt()
    }
}

/// How heights are resolved when sampling: ground height (m, world y) at a
/// geodetic point, the QNH correction of pressure altitudes (m), and the
/// height of the reference point above the wheels (m).
pub struct Heights<'a> {
    pub ground: &'a dyn Fn(f64, f64) -> f64,
    pub qnh_corr: f64,
    pub gear: f64,
}

impl Heights<'_> {
    /// World height of a sample's reference point.
    fn of(&self, s: &Sample) -> f64 {
        match s.alt {
            Some(a) if !s.on_ground => (a + self.qnh_corr).max((self.ground)(s.lat, s.lon) + self.gear),
            _ => (self.ground)(s.lat, s.lon) + self.gear,
        }
    }
}

/// The playout output for one time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub lat: f64,
    pub lon: f64,
    /// World height of the reference point (m), before ground contact is enforced.
    pub y: f64,
    /// Velocity: east, up, south (m/s).
    pub vel: [f64; 3],
    pub on_ground: bool,
    /// Interpolated heading / bank when the aircraft sends them.
    pub heading: Option<f32>,
    pub roll: Option<f32>,
    /// Seconds past the newest report (0 when interpolating), negative before the oldest.
    pub extrapolated: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Track {
    pub buf: VecDeque<Sample>,
    /// Last raw (uncorrected) output: time, geodetic, height, velocity.
    raw_prev: Option<(f64, f64, f64, f64, [f64; 3])>,
    /// Continuity offset (east, up, south), m.
    corr: [f64; 3],
    rejects: u32,
}

fn hermite(p0: f64, v0: f64, p1: f64, v1: f64, h: f64, s: f64) -> (f64, f64) {
    let s2 = s * s;
    let s3 = s2 * s;
    let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    let h10 = s3 - 2.0 * s2 + s;
    let h01 = -2.0 * s3 + 3.0 * s2;
    let h11 = s3 - s2;
    let p = h00 * p0 + h10 * h * v0 + h01 * p1 + h11 * h * v1;
    let d00 = 6.0 * s2 - 6.0 * s;
    let d10 = 3.0 * s2 - 4.0 * s + 1.0;
    let d01 = -6.0 * s2 + 6.0 * s;
    let d11 = 3.0 * s2 - 2.0 * s;
    let v = (d00 * p0 + d01 * p1) / h + d10 * v0 + d11 * v1;
    (p, v)
}

/// Limit end velocities so the cubic through a segment of `chord` metres in
/// `h` s neither overshoots nor loops: each end at most 3 × the chord speed
/// (+ a small allowance), and velocity against the chord removed for short
/// segments (stationary jitter).
fn limit(v: [f64; 2], chord: [f64; 2], h: f64) -> [f64; 2] {
    let cl = (chord[0] * chord[0] + chord[1] * chord[1]).sqrt();
    let cap = 3.0 * cl / h + 0.5;
    let n = (v[0] * v[0] + v[1] * v[1]).sqrt();
    let mut v = if n > cap { [v[0] * cap / n, v[1] * cap / n] } else { v };
    if cl > 1e-6 && cl < 30.0 {
        let u = [chord[0] / cl, chord[1] / cl];
        let along = v[0] * u[0] + v[1] * u[1];
        if along < 0.0 {
            v = [v[0] - along * u[0], v[1] - along * u[1]];
        }
    }
    v
}

/// The 1-D monotone limit (vertical): no overshoot past either end level.
fn limit1(v: f64, dy: f64, h: f64) -> f64 {
    if v * dy <= 0.0 {
        return 0.0;
    }
    let cap = 3.0 * dy.abs() / h;
    v.clamp(-cap, cap)
}

fn lerp_angle(a: f32, b: f32, s: f32) -> f32 {
    let d = (b - a + 540.0).rem_euclid(360.0) - 180.0;
    (a + d * s).rem_euclid(360.0)
}

impl Track {
    pub fn newest(&self) -> Option<&Sample> {
        self.buf.back()
    }

    pub fn oldest_t(&self) -> Option<f64> {
        self.buf.front().map(|s| s.t)
    }

    /// Add a report. Returns false if it was dropped (duplicate, bad fix).
    pub fn push(&mut self, r: &Report) -> bool {
        let mut s = Sample::from_report(r);
        if let Some(b) = self.buf.back() {
            if (s.t - b.t).abs() < 0.25 {
                return false;
            }
            // Implausible jump between consecutive fixes (bad MLAT / decode):
            // dropped, unless they keep coming (then the old ones were wrong).
            let near = if s.t > b.t { *b } else { *self.buf.iter().rev().find(|x| x.t < s.t).unwrap_or(b) };
            let tan = Tangent::new(near.lat, near.lon);
            let d = tan.to_local(s.lat, s.lon, 0.0);
            let dist = (d[0] * d[0] + d[2] * d[2]).sqrt();
            let dt = (s.t - near.t).abs().max(0.5);
            if dist / dt > MAX_SPEED && dist > 1000.0 {
                self.rejects += 1;
                if self.rejects < 3 {
                    return false;
                }
                self.buf.clear();
            } else if s.t > b.t && s.t - b.t > MAX_GAP {
                // Out of coverage for minutes: start over (no curve across it).
                self.buf.clear();
            } else if r.source == Source::Mlat && s.t > b.t && b.has_vel {
                // MLAT fixes are noisy: pull halfway towards the dead-reckoned position.
                let dt = s.t - b.t;
                let pred = [b.vel[0] * dt, b.vel[1] * dt];
                let k = 0.5;
                let p = [d[0] * (1.0 - k) + pred[0] * k, d[2] * (1.0 - k) + pred[1] * k];
                let (lat, lon, _) = tan.to_geo([p[0], 0.0, p[1]]);
                s.lat = lat;
                s.lon = lon;
            }
        }
        self.rejects = 0;
        match self.buf.iter().position(|x| x.t > s.t) {
            Some(i) => self.buf.insert(i, s),
            None => self.buf.push_back(s),
        }
        while self.buf.len() > 64 {
            self.buf.pop_front();
        }
        true
    }

    /// Forget reports nobody will need again (keep one at or before `t`).
    pub fn trim(&mut self, t: f64) {
        while self.buf.len() > 2 && self.buf[1].t <= t - KEEP_BEHIND {
            self.buf.pop_front();
        }
    }

    /// The raw (uncorrected) curve at `t`.
    pub fn raw(&self, t: f64, hs: &Heights) -> Option<Pose> {
        let n = self.buf.len();
        if n == 0 {
            return None;
        }
        let i = self.buf.iter().position(|s| s.t > t);
        match i {
            // Before the oldest report: run it backwards (straight).
            Some(0) => {
                let s = &self.buf[0];
                let back = (t - s.t).max(-MAX_BACKFILL);
                Some(self.extrapolate(s, back, back, hs))
            }
            Some(i) => Some(self.interpolate(&self.buf[i - 1], &self.buf[i], t, hs)),
            None => {
                let s = &self.buf[n - 1];
                let ahead = t - s.t;
                Some(self.extrapolate(s, ahead.min(MAX_EXTRAPOLATE), ahead, hs))
            }
        }
    }

    fn turn_rate(&self, s: &Sample) -> f64 {
        if let Some(w) = s.turn {
            return w.clamp(-0.06, 0.06);
        }
        // From the last two reports' tracks.
        let n = self.buf.len();
        if n >= 2 {
            let (a, b) = (&self.buf[n - 2], &self.buf[n - 1]);
            if a.has_vel && b.has_vel && a.speed() > 20.0 && b.speed() > 20.0 && b.t - a.t > 0.5 {
                let ha = a.vel[0].atan2(-a.vel[1]);
                let hb = b.vel[0].atan2(-b.vel[1]);
                let d = (hb - ha + 3.0 * std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
                return (d / (b.t - a.t)).clamp(-0.06, 0.06);
            }
        }
        0.0
    }

    /// Dead reckoning `dt` s from a report (negative = backwards): along the
    /// track, turning at the track rate (forwards only), at the vertical rate.
    fn extrapolate(&self, s: &Sample, dt: f64, really: f64, hs: &Heights) -> Pose {
        let tan = Tangent::new(s.lat, s.lon);
        let w = if dt > 0.0 { self.turn_rate(s) } else { 0.0 };
        let v = if s.on_ground && s.speed() < 1.5 { [0.0, 0.0] } else { s.vel };
        let (x, z, vel) = if w.abs() > 1e-4 {
            // Arc: rotate the velocity by w·t (clockwise seen from above = right turn).
            let th = w * dt;
            let (sn, cs) = th.sin_cos();
            // Right turn: heading increases; in (east, south) coords the
            // velocity rotates by +th about up (east → south).
            let rot = |a: [f64; 2], c: f64, sn: f64| [a[0] * c - a[1] * sn, a[0] * sn + a[1] * c];
            let vend = rot(v, cs, sn);
            // ∫ R(wτ) v dτ = (1/w) [ [sin, cos-1] ... ]
            let ix = (v[0] * sn - v[1] * (1.0 - cs)) / w;
            let iz = (v[0] * (1.0 - cs) + v[1] * sn) / w;
            (ix, iz, vend)
        } else {
            (v[0] * dt, v[1] * dt, v)
        };
        let y0 = hs.of(s);
        let vs = if s.on_ground { 0.0 } else { s.vs };
        let y = y0 + vs * dt;
        let (lat, lon, _) = tan.to_geo([x, 0.0, z]);
        Pose { lat, lon, y, vel: [vel[0], vs, vel[1]], on_ground: s.on_ground, heading: s.heading, roll: s.roll, extrapolated: really }
    }

    fn interpolate(&self, a: &Sample, b: &Sample, t: f64, hs: &Heights) -> Pose {
        let h = (b.t - a.t).max(1e-3);
        let s = ((t - a.t) / h).clamp(0.0, 1.0);
        let tan = Tangent::new(a.lat, a.lon);
        let pb = tan.to_local(b.lat, b.lon, 0.0);
        let chord = [pb[0], pb[2]];
        // Stationary on the ground: no creeping between jittery fixes.
        let parked = a.on_ground && b.on_ground && a.speed() < 1.0 && b.speed() < 1.0;
        let va = if a.has_vel { limit(a.vel, chord, h) } else { [chord[0] / h, chord[1] / h] };
        let vb = if b.has_vel { limit(b.vel, chord, h) } else { [chord[0] / h, chord[1] / h] };
        let (x, vx, z, vz) = if parked {
            (chord[0] * s, 0.0, chord[1] * s, 0.0)
        } else {
            let (x, vx) = hermite(0.0, va[0], chord[0], vb[0], h, s);
            let (z, vz) = hermite(0.0, va[1], chord[1], vb[1], h, s);
            (x, vx, z, vz)
        };
        let (ya, yb) = (hs.of(a), hs.of(b));
        let dy = yb - ya;
        let vsa = limit1(if a.on_ground { 0.0 } else { a.vs }, dy, h);
        let vsb = limit1(if b.on_ground { 0.0 } else { b.vs }, dy, h);
        let (y, vy) = hermite(ya, vsa, yb, vsb, h, s);
        let (lat, lon, _) = tan.to_geo([x, 0.0, z]);
        let heading = match (a.heading, b.heading) {
            (Some(p), Some(q)) => Some(lerp_angle(p, q, s as f32)),
            (p, q) => p.or(q),
        };
        let roll = match (a.roll, b.roll) {
            (Some(p), Some(q)) => Some(p + (q - p) * s as f32),
            _ => None,
        };
        // On the ground while both ends are (lift-off / touch-down emerge from the heights).
        Pose { lat, lon, y, vel: [vx, vy, vz], on_ground: a.on_ground && b.on_ground, heading, roll, extrapolated: 0.0 }
    }

    /// The pose at playout time `t`, continuous from frame to frame (any
    /// jump of the raw curve goes into a decaying correction). `dt`: time
    /// since the previous call.
    pub fn sample(&mut self, t: f64, dt: f64, hs: &Heights) -> Option<Pose> {
        let raw = self.raw(t, hs)?;
        if let Some((tp, lat, lon, y, v)) = self.raw_prev {
            let step = t - tp;
            if step > 0.0 && step < 1.0 {
                // Where the previous raw output said we would be now.
                let tan = Tangent::new(lat, lon);
                let now = tan.to_local(raw.lat, raw.lon, raw.y);
                let pred = [v[0] * step, y + v[1] * step, v[2] * step];
                let jump = [now[0] - pred[0], now[1] - pred[1], now[2] - pred[2]];
                let speed = (v[0] * v[0] + v[2] * v[2]).sqrt();
                let jh = (jump[0] * jump[0] + jump[2] * jump[2]).sqrt();
                let tol = 0.25 + 0.01 * speed * step * 60.0;
                if jh > tol && jh < 5000.0 {
                    self.corr[0] -= jump[0];
                    self.corr[2] -= jump[2];
                }
                if jump[1].abs() > 0.25 + 0.02 * v[1].abs() && jump[1].abs() < 1000.0 {
                    self.corr[1] -= jump[1];
                }
            } else if step >= 1.0 || step < 0.0 {
                self.corr = [0.0; 3];
            }
        }
        self.raw_prev = Some((t, raw.lat, raw.lon, raw.y, raw.vel));
        // Decay the correction, never faster than a fraction of the speed
        // (so the aircraft is never seen moving backwards).
        let dt = dt.clamp(0.0, 1.0);
        let speed = (raw.vel[0] * raw.vel[0] + raw.vel[2] * raw.vel[2]).sqrt();
        let k = 1.0 - (-dt / CORR_TAU).exp();
        let mut dec = [self.corr[0] * k, self.corr[1] * k, self.corr[2] * k];
        let hmax = (0.4 * speed + 0.3) * dt;
        let hl = (dec[0] * dec[0] + dec[2] * dec[2]).sqrt();
        if hl > hmax {
            dec[0] *= hmax / hl;
            dec[2] *= hmax / hl;
        }
        let vmax = (0.4 * raw.vel[1].abs() + 1.0) * dt;
        dec[1] = dec[1].clamp(-vmax, vmax);
        let mut corr_rate = [0.0; 3];
        for k in 0..3 {
            self.corr[k] -= dec[k];
            if dt > 0.0 {
                corr_rate[k] = -dec[k] / dt;
            }
        }
        let mut out = raw;
        if self.corr != [0.0; 3] {
            let tan = Tangent::new(raw.lat, raw.lon);
            let (lat, lon, _) = tan.to_geo([self.corr[0], 0.0, self.corr[2]]);
            out.lat = lat;
            out.lon = lon;
            out.y += self.corr[1];
            for k in 0..3 {
                out.vel[k] += corr_rate[k];
            }
        }
        Some(out)
    }

    /// The pending correction (m), for tests and diagnostics.
    pub fn correction(&self) -> [f64; 3] {
        self.corr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT0: f64 = 51.47;
    const LON0: f64 = -0.45;

    /// A deterministic Gaussian-ish noise source.
    struct Rng(u64);
    impl Rng {
        fn u(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }
        fn n(&mut self) -> f64 {
            (0..6).map(|_| self.u()).sum::<f64>() - 3.0
        }
    }

    /// Synthetic truth: 140 m/s heading 090, straight for 60 s, a 3°/s right
    /// turn for 30 s (90°), then straight; descending at 4 m/s from 1500 m.
    fn truth(t: f64) -> ([f64; 2], [f64; 2], f64, f64) {
        let v = 140.0;
        let w = 3f64.to_radians();
        let alt = 1500.0 - 4.0 * t;
        if t < 60.0 {
            ([v * t, 0.0], [v, 0.0], alt, 90.0)
        } else if t < 90.0 {
            let th = w * (t - 60.0);
            let r = v / w;
            // Right turn from east towards south (+z).
            ([v * 60.0 + r * th.sin(), r * (1.0 - th.cos())], [v * th.cos(), v * th.sin()], alt, 90.0 + th.to_degrees())
        } else {
            let r = v / w;
            let p0 = [v * 60.0 + r, r];
            ([p0[0], p0[1] + v * (t - 90.0)], [0.0, v], alt, 180.0)
        }
    }

    fn report(t: f64, rng: &mut Rng, noise: f64) -> Report {
        let (p, v, alt, trk) = truth(t);
        let tan = Tangent::new(LAT0, LON0);
        let (lat, lon, _) = tan.to_geo([p[0] + rng.n() * noise, 0.0, p[1] + rng.n() * noise]);
        let gs = (v[0] * v[0] + v[1] * v[1]).sqrt() / KT + rng.n() * 1.0;
        Report {
            hex: 1,
            t,
            lat,
            lon,
            alt_baro_ft: Some(((alt / FT) / 25.0).round() as f32 * 25.0),
            gs_kt: Some(gs as f32),
            track_deg: Some((trk + rng.n() * 0.7) as f32),
            vrate_fpm: Some((-4.0 / FPM + rng.n() * 64.0) as f32),
            ..Default::default()
        }
    }

    fn flat() -> impl Fn(f64, f64) -> f64 {
        |_, _| 0.0
    }

    /// Feed reports every 4–9 s (each arriving 0–3 s late), play out 12 s
    /// behind, compare with the truth.
    #[test]
    fn playout_follows_a_noisy_turning_track() {
        let mut rng = Rng(7);
        let mut tr = Track::default();
        let g = flat();
        let hs = Heights { ground: &g, qnh_corr: 0.0, gear: 2.0 };
        // Report times and their arrival times.
        let mut reports = Vec::new();
        let mut t = 0.0;
        while t < 160.0 {
            let r = report(t, &mut rng, 7.0);
            let arrive = t + rng.u() * 3.0;
            reports.push((arrive, r));
            t += 4.0 + rng.u() * 5.0;
        }
        let delay = 12.0;
        let dt = 1.0 / 60.0;
        let (mut max_err, mut sum2, mut n) = (0.0f64, 0.0, 0);
        let mut max_alt_err = 0.0f64;
        let mut prev: Option<[f64; 3]> = None;
        let mut prev_v: Option<[f64; 2]> = None;
        let mut max_acc = 0.0f64;
        let mut now = 0.0;
        let tan = Tangent::new(LAT0, LON0);
        while now < 150.0 {
            for (arr, r) in &reports {
                if *arr > now - dt && *arr <= now {
                    tr.push(r);
                }
            }
            let tp = now - delay;
            if tp > 5.0 {
                let p = tr.sample(tp, dt, &hs).unwrap();
                let l = tan.to_local(p.lat, p.lon, p.y);
                let (tp_true, tv, talt, _) = truth(tp);
                let e = ((l[0] - tp_true[0]).powi(2) + (l[2] - tp_true[1]).powi(2)).sqrt();
                max_err = max_err.max(e);
                sum2 += e * e;
                n += 1;
                max_alt_err = max_alt_err.max((l[1] - talt).abs());
                // Never backwards along the true direction of motion.
                if let Some(q) = prev {
                    let step = [l[0] - q[0], l[2] - q[2]];
                    assert!(step[0] * tv[0] + step[1] * tv[1] > 0.0, "moved backwards at {tp:.2}");
                    let v = [step[0] / dt, step[1] / dt];
                    if let Some(pv) = prev_v {
                        let a = ((v[0] - pv[0]).powi(2) + (v[1] - pv[1]).powi(2)).sqrt() / dt;
                        max_acc = max_acc.max(a);
                    }
                    prev_v = Some(v);
                }
                prev = Some(l);
            }
            tr.trim(now - delay);
            now += dt;
        }
        let rms = (sum2 / n as f64).sqrt();
        eprintln!("playout: max error {max_err:.1} m, rms {rms:.1} m, alt max {max_alt_err:.1} m, max accel {max_acc:.2} m/s²");
        assert!(max_err < 30.0, "max error {max_err:.1} m");
        assert!(rms < 12.0, "rms error {rms:.1} m");
        assert!(max_alt_err < 15.0, "altitude error {max_alt_err:.1} m");
        // Smooth: no jerks (a 3°/s turn at 140 m/s is 7.3 m/s²).
        assert!(max_acc < 25.0, "max acceleration {max_acc:.1} m/s²");
    }

    /// The feed stops: dead reckoning keeps turning; when reports resume the
    /// correction blends out without a jump and without running backwards.
    #[test]
    fn extrapolation_and_recovery() {
        let mut rng = Rng(3);
        let mut tr = Track::default();
        let g = flat();
        let hs = Heights { ground: &g, qnh_corr: 0.0, gear: 2.0 };
        for t in [50.0, 56.0, 62.0, 68.0] {
            let mut r = report(t, &mut rng, 0.0);
            r.track_rate = if t >= 60.0 { Some(3.0) } else { Some(0.0) };
            tr.push(&r);
        }
        let tan = Tangent::new(LAT0, LON0);
        let dt: f64 = 1.0 / 60.0;
        let mut t: f64 = 66.0;
        let mut prev: Option<[f64; 3]> = None;
        let mut worst = 0.0f64;
        let mut max_step = 0.0f64;
        while t < 95.0 {
            if (t - 80.0).abs() < dt * 0.5 {
                // Reports resume (the 74 s one arrives late, then 80 s).
                tr.push(&report(74.0, &mut rng, 0.0));
                tr.push(&report(80.0, &mut rng, 0.0));
            }
            let p = tr.sample(t, dt, &hs).unwrap();
            let l = tan.to_local(p.lat, p.lon, p.y);
            let (tt, tv, _, _) = truth(t);
            if t < 78.0 {
                // 10 s of dead reckoning in the turn: within a few tens of metres.
                worst = worst.max(((l[0] - tt[0]).powi(2) + (l[2] - tt[1]).powi(2)).sqrt());
            }
            if let Some(q) = prev {
                let step = [l[0] - q[0], l[2] - q[2]];
                assert!(step[0] * tv[0] + step[1] * tv[1] > 0.0, "backwards at {t:.2}");
                max_step = max_step.max((step[0] * step[0] + step[1] * step[1]).sqrt());
            }
            prev = Some(l);
            t += dt;
        }
        eprintln!("extrapolation: worst {worst:.1} m over 10 s, largest frame step {max_step:.2} m");
        assert!(worst < 60.0, "dead reckoning error {worst:.1} m");
        // 140 m/s at 60 Hz = 2.33 m per frame: corrections add at most 40 %.
        assert!(max_step < 2.33 * 1.45, "frame step {max_step:.2} m");
    }

    /// A parked aircraft with GPS jitter stays put; a bad fix is ignored.
    #[test]
    fn parked_and_outliers() {
        let mut tr = Track::default();
        let g = flat();
        let hs = Heights { ground: &g, qnh_corr: 0.0, gear: 2.0 };
        let tan = Tangent::new(LAT0, LON0);
        let mut rng = Rng(11);
        for k in 0..6 {
            let (lat, lon, _) = tan.to_geo([rng.n() * 2.0, 0.0, rng.n() * 2.0]);
            tr.push(&Report { hex: 1, t: k as f64 * 5.0, lat, lon, on_ground: true, gs_kt: Some(0.0), track_deg: Some(270.0), ..Default::default() });
        }
        // 50 km away a second later: dropped.
        let (lat, lon, _) = tan.to_geo([50_000.0, 0.0, 0.0]);
        assert!(!tr.push(&Report { hex: 1, t: 26.0, lat, lon, on_ground: true, gs_kt: Some(0.0), ..Default::default() }));
        for k in 0..240 {
            let p = tr.sample(5.0 + k as f64 / 10.0, 0.1, &hs).unwrap();
            let l = tan.to_local(p.lat, p.lon, p.y);
            assert!(l[0].abs() < 8.0 && l[2].abs() < 8.0, "{l:?}");
            assert!((l[1] - 2.0).abs() < 1e-6);
            assert!(p.on_ground);
        }
    }

    /// Landing: the curve flares into the first on-ground report (vertical
    /// speed → 0 at touch-down) and never goes below the wheels.
    #[test]
    fn touchdown_flares() {
        let mut tr = Track::default();
        let g = flat();
        let hs = Heights { ground: &g, qnh_corr: 0.0, gear: 3.0 };
        let tan = Tangent::new(LAT0, LON0);
        let v = 70.0;
        let mk = |t: f64, alt_ft: Option<f32>, vs: f32| {
            let (lat, lon, _) = tan.to_geo([v * t, 0.0, 0.0]);
            Report {
                hex: 1,
                t,
                lat,
                lon,
                on_ground: alt_ft.is_none(),
                alt_baro_ft: alt_ft,
                gs_kt: Some((v / KT) as f32),
                track_deg: Some(90.0),
                vrate_fpm: Some(vs),
                ..Default::default()
            }
        };
        tr.push(&mk(0.0, Some(400.0), -700.0));
        tr.push(&mk(8.0, Some(150.0), -700.0));
        tr.push(&mk(20.0, None, 0.0));
        let mut last_y = f64::MAX;
        for k in 0..=240 {
            let t = 8.0 + k as f64 * 0.05;
            let p = tr.raw(t, &hs).unwrap();
            assert!(p.y >= 3.0 - 1e-6, "{t}: {}", p.y);
            assert!(p.y <= last_y + 1e-6, "climbs at {t}");
            last_y = p.y;
            if t > 19.5 {
                assert!(p.vel[1] > -1.0, "sink {:.2} m/s at {t}", p.vel[1]);
            }
        }
    }
}
