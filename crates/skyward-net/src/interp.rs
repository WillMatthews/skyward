//! Snapshot interpolation for remote aircraft.
//!
//! Each remote aircraft keeps a small buffer of timestamped states (server
//! clock). It is drawn at `server_now − delay`: between two samples the
//! position follows a cubic Hermite curve through both positions *and*
//! velocities (so 20 Hz samples of a 250 m/s jet in a 7 g turn stay on the
//! arc) and the attitude a slerp; past the newest sample (packets lost or
//! late) it dead-reckons along the velocity and angular velocity for up to
//! [`MAX_EXTRAPOLATE`] s, then holds. When fresh samples end an
//! extrapolation, the difference is blended out over ~0.2 s instead of
//! snapping. The delay adapts to the measured send interval and arrival
//! jitter (the jitter buffer), and changes slowly.
//!
//! The maths runs in a local east-up-south tangent frame around the newest
//! sample (metres, f64), the same axes as the game's world frame.

use crate::proto::{AcState, M_PER_DEG_LAT, flags};
use std::collections::VecDeque;

/// Longest dead-reckoning past the newest sample, s.
pub const MAX_EXTRAPOLATE: f64 = 0.6;
/// No sample for this long: the aircraft is shown as stale (frozen, faded tag).
pub const STALE_AFTER: f64 = 3.0;
const BUFFER: usize = 48;
/// Jumps larger than this between consecutive samples are respawns / teleports.
const TELEPORT_M: f64 = 800.0;

// ---------------------------------------------------------------- small vector maths

pub type V3 = [f64; 3];
/// Quaternion xyzw.
pub type Q = [f64; 4];

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn len(a: V3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}
fn v3(a: [f32; 3]) -> V3 {
    [a[0] as f64, a[1] as f64, a[2] as f64]
}

pub fn q_mul(a: Q, b: Q) -> Q {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}
pub fn q_conj(a: Q) -> Q {
    [-a[0], -a[1], -a[2], a[3]]
}
fn q_dot(a: Q, b: Q) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}
pub fn q_norm(a: Q) -> Q {
    let n = q_dot(a, a).sqrt().max(1e-12);
    [a[0] / n, a[1] / n, a[2] / n, a[3] / n]
}
/// Rotation of `angle * axis` (a rotation vector).
pub fn q_from_rotvec(r: V3) -> Q {
    let a = len(r);
    if a < 1e-12 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    let s = (a * 0.5).sin() / a;
    [r[0] * s, r[1] * s, r[2] * s, (a * 0.5).cos()]
}
/// Angle between two attitudes, radians.
pub fn q_angle(a: Q, b: Q) -> f64 {
    2.0 * q_dot(a, b).abs().min(1.0).acos()
}
pub fn q_slerp(a: Q, b: Q, t: f64) -> Q {
    let mut b = b;
    let mut d = q_dot(a, b);
    if d < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
        d = -d;
    }
    if d > 0.9995 {
        return q_norm([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, a[3] + (b[3] - a[3]) * t]);
    }
    let th = d.acos();
    let s = th.sin();
    let (ka, kb) = (((1.0 - t) * th).sin() / s, (t * th).sin() / s);
    [a[0] * ka + b[0] * kb, a[1] * ka + b[1] * kb, a[2] * ka + b[2] * kb, a[3] * ka + b[3] * kb]
}
fn q64(q: [f32; 4]) -> Q {
    q_norm([q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64])
}
fn q32(q: Q) -> [f32; 4] {
    [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32]
}

/// Local east-up-south tangent frame around a reference point (equirectangular,
/// like the game's frames).
#[derive(Clone, Copy, Debug)]
pub struct Tangent {
    pub lat0: f64,
    pub lon0: f64,
    m_lon: f64,
}

impl Tangent {
    pub fn new(lat0: f64, lon0: f64) -> Tangent {
        Tangent { lat0, lon0, m_lon: M_PER_DEG_LAT * lat0.to_radians().cos().max(1e-6) }
    }
    /// (lat, lon, alt) → [east, up, south] metres.
    pub fn to_local(&self, lat: f64, lon: f64, alt: f64) -> V3 {
        let dlon = (lon - self.lon0 + 540.0).rem_euclid(360.0) - 180.0;
        [dlon * self.m_lon, alt, -(lat - self.lat0) * M_PER_DEG_LAT]
    }
    pub fn to_geo(&self, p: V3) -> (f64, f64, f64) {
        let lon = (self.lon0 + p[0] / self.m_lon + 540.0).rem_euclid(360.0) - 180.0;
        (self.lat0 - p[2] / M_PER_DEG_LAT, lon, p[1])
    }
}

// ---------------------------------------------------------------- interpolator

/// One remote aircraft as drawn this frame.
#[derive(Clone, Copy, Debug)]
pub struct Sampled {
    /// Interpolated state (pose, velocities, surfaces, lights...).
    pub state: AcState,
    /// Seconds past the newest sample (0 when interpolating).
    pub extrapolated: f64,
    /// No sample for [`STALE_AFTER`] s.
    pub stale: bool,
}

#[derive(Clone, Debug)]
pub struct Interpolator {
    buf: VecDeque<AcState>,
    /// Playout delay in use, s.
    pub delay: f64,
    /// Measured send interval (median gap between samples), s.
    pub interval: f64,
    /// Arrival jitter (mean absolute deviation of arrival − sample time), s.
    pub jitter: f64,
    lag_mean: f64,
    gaps: VecDeque<f64>,
    last_arrival: f64,
    // continuity
    last_out: Option<(f64, V3, Q, V3, (f64, f64))>, // (time, pos, rot, vel, tangent ref)
    corr_p: V3,
    corr_q: Q,
    was_extrapolating: bool,
}

impl Default for Interpolator {
    fn default() -> Self {
        Interpolator {
            buf: VecDeque::new(),
            delay: 0.1,
            interval: 0.05,
            jitter: 0.0,
            lag_mean: 0.0,
            gaps: VecDeque::new(),
            last_arrival: f64::NEG_INFINITY,
            last_out: None,
            corr_p: [0.0; 3],
            corr_q: [0.0, 0.0, 0.0, 1.0],
            was_extrapolating: false,
        }
    }
}

/// Playout delay (s): the mean sample → arrival lag (sender's tick, the
/// network both ways through the server, the server's tick) plus 1.25 send
/// intervals, so one lost packet is bridged by interpolation, plus a jitter
/// margin.
fn delay_for(lag: f64, interval: f64, jitter: f64) -> f64 {
    (lag.max(0.0) + interval * 1.25 + jitter * 2.5 + 0.01).clamp(0.05, 1.0)
}

impl Interpolator {
    pub fn newest(&self) -> Option<&AcState> {
        self.buf.back()
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.last_out = None;
        self.corr_p = [0.0; 3];
        self.corr_q = [0.0, 0.0, 0.0, 1.0];
    }

    /// A sample arrived at server time `arrival` (the local clock mapped to the server's).
    pub fn push(&mut self, s: AcState, arrival: f64) {
        if let Some(b) = self.buf.back() {
            if s.t <= b.t {
                // Older than what we have (reordered): insert if it fills a gap.
                if s.t > self.buf.front().map(|f| f.t).unwrap_or(f64::MAX) && !self.buf.iter().any(|x| (x.t - s.t).abs() < 1e-6) {
                    let i = self.buf.iter().position(|x| x.t > s.t).unwrap_or(self.buf.len());
                    self.buf.insert(i, s);
                }
                return;
            }
            // Teleport (respawn, reset): don't interpolate across it.
            let tan = Tangent::new(b.lat, b.lon);
            let dt = (s.t - b.t).min(2.0);
            let pred = add(tan.to_local(b.lat, b.lon, b.alt as f64), scale(v3(b.vel), dt));
            if len(sub(tan.to_local(s.lat, s.lon, s.alt as f64), pred)) > TELEPORT_M {
                self.reset();
            } else {
                self.gaps.push_back(s.t - b.t);
                if self.gaps.len() > 21 {
                    self.gaps.pop_front();
                }
                let mut g: Vec<f64> = self.gaps.iter().copied().collect();
                g.sort_by(|a, b| a.total_cmp(b));
                self.interval = g[g.len() / 2].clamp(0.01, 0.5);
            }
        }
        // Jitter of the one-way lag (sample → arrival).
        let lag = arrival - s.t;
        if self.last_arrival.is_finite() {
            self.jitter = 0.95 * self.jitter + 0.05 * (lag - self.lag_mean).abs();
            self.lag_mean = 0.95 * self.lag_mean + 0.05 * lag;
        } else {
            self.lag_mean = lag;
        }
        self.last_arrival = arrival;
        if self.buf.is_empty() && self.last_out.is_none() {
            // First sample: start at the right delay (no slow slew from the default).
            self.delay = delay_for(self.lag_mean, self.interval, self.jitter);
        }
        self.buf.push_back(s);
        while self.buf.len() > BUFFER {
            self.buf.pop_front();
        }
    }

    /// The pose at server time `now − delay`. `dt`: frame time (slews the
    /// delay and decays corrections).
    pub fn sample(&mut self, now: f64, dt: f64) -> Option<Sampled> {
        let newest = *self.buf.back()?;
        // Adapt the playout delay slowly (≤ 50 ms per second) — no time warps.
        let want = delay_for(self.lag_mean, self.interval, self.jitter);
        let k = 0.05 * dt.clamp(0.0, 0.1);
        self.delay += (want - self.delay).clamp(-k, k);
        let t = now - self.delay;
        let tan = Tangent::new(newest.lat, newest.lon);
        let stale = now - self.last_arrival > STALE_AFTER;
        // Drop samples nobody will need again (keep one before t).
        while self.buf.len() > 2 && self.buf[1].t <= t {
            self.buf.pop_front();
        }
        let (mut st, extrap) = match self.buf.iter().position(|s| s.t > t) {
            Some(0) => (self.buf[0], 0.0), // before the oldest: hold it
            Some(i) => (interpolate(&tan, &self.buf[i - 1], &self.buf[i], t), 0.0),
            None => {
                let e = (t - newest.t).max(0.0);
                if newest.has(flags::PAUSED) || newest.has(flags::CRASHED) || stale { (newest, e) } else { (extrapolate(&tan, &newest, e.min(MAX_EXTRAPOLATE)), e) }
            }
        };
        // Continuity: when fresh samples end an extrapolation (or the buffer
        // changes under us), blend the jump out instead of snapping.
        let p = tan.to_local(st.lat, st.lon, st.alt as f64);
        let q = q64(st.rot);
        let extrapolating = extrap > 0.0;
        if let Some((t0, p0, q0, v0, (la, lo))) = self.last_out {
            let fdt = (t - t0).clamp(0.0, 0.25);
            let tan0 = Tangent::new(la, lo);
            let (gla, glo, gal) = tan0.to_geo(add(p0, scale(v0, fdt)));
            let cont = tan.to_local(gla, glo, gal);
            if self.was_extrapolating && !extrapolating {
                let e = sub(cont, p);
                if len(e) < 60.0 {
                    self.corr_p = e;
                    self.corr_q = q_mul(q0, q_conj(q));
                } else {
                    self.corr_p = [0.0; 3];
                    self.corr_q = [0.0, 0.0, 0.0, 1.0];
                }
            }
        }
        let decay = (-dt.max(0.0) / 0.2).exp();
        self.corr_p = scale(self.corr_p, decay);
        self.corr_q = q_slerp([0.0, 0.0, 0.0, 1.0], self.corr_q, decay);
        let p_out = add(p, self.corr_p);
        let q_out = q_norm(q_mul(self.corr_q, q));
        let (la, lo, al) = tan.to_geo(p_out);
        st.lat = la;
        st.lon = lo;
        st.alt = al as f32;
        st.rot = q32(q_out);
        st.t = t;
        self.last_out = Some((t, p, q, v3(st.vel), (tan.lat0, tan.lon0)));
        self.was_extrapolating = extrapolating;
        Some(Sampled { state: st, extrapolated: extrap, stale })
    }
}

fn lerp(a: f32, b: f32, s: f32) -> f32 {
    a + (b - a) * s
}

/// Hermite position, slerped attitude, blended controls between `a` and `b` at `t`.
pub fn interpolate(tan: &Tangent, a: &AcState, b: &AcState, t: f64) -> AcState {
    let h = (b.t - a.t).max(1e-6);
    let s = ((t - a.t) / h).clamp(0.0, 1.0);
    let pa = tan.to_local(a.lat, a.lon, a.alt as f64);
    let pb = tan.to_local(b.lat, b.lon, b.alt as f64);
    let (va, vb) = (v3(a.vel), v3(b.vel));
    let (s2, s3) = (s * s, s * s * s);
    let (h00, h10, h01, h11) = (2.0 * s3 - 3.0 * s2 + 1.0, s3 - 2.0 * s2 + s, -2.0 * s3 + 3.0 * s2, s3 - s2);
    let p = add(add(scale(pa, h00), scale(va, h10 * h)), add(scale(pb, h01), scale(vb, h11 * h)));
    // Velocity: derivative of the curve (consistent with the motion drawn).
    let (d00, d10, d01, d11) = (6.0 * s2 - 6.0 * s, 3.0 * s2 - 4.0 * s + 1.0, -6.0 * s2 + 6.0 * s, 3.0 * s2 - 2.0 * s);
    let v = add(add(scale(pa, d00 / h), scale(va, d10)), add(scale(pb, d01 / h), scale(vb, d11)));
    let q = q_slerp(q64(a.rot), q64(b.rot), s);
    let (lat, lon, alt) = tan.to_geo(p);
    let sf = s as f32;
    let near = if s < 0.5 { a } else { b };
    let mut out = *near;
    out.t = t;
    out.lat = lat;
    out.lon = lon;
    out.alt = alt as f32;
    out.rot = q32(q);
    out.vel = [v[0] as f32, v[1] as f32, v[2] as f32];
    out.omega = [lerp(a.omega[0], b.omega[0], sf), lerp(a.omega[1], b.omega[1], sf), lerp(a.omega[2], b.omega[2], sf)];
    out.aileron = lerp(a.aileron, b.aileron, sf);
    out.elevator = lerp(a.elevator, b.elevator, sf);
    out.rudder = lerp(a.rudder, b.rudder, sf);
    out.flaps = lerp(a.flaps, b.flaps, sf);
    out.spoilers = lerp(a.spoilers, b.spoilers, sf);
    out.gear = lerp(a.gear, b.gear, sf);
    out.reverser = lerp(a.reverser, b.reverser, sf);
    out.afterburner = lerp(a.afterburner, b.afterburner, sf);
    out.throttle = lerp(a.throttle, b.throttle, sf);
    out.brakes = lerp(a.brakes, b.brakes, sf);
    out.tas = lerp(a.tas, b.tas, sf);
    if a.n_engines == b.n_engines {
        for i in 0..a.n_engines as usize {
            out.engines[i].rpm = lerp(a.engines[i].rpm, b.engines[i].rpm, sf);
            out.engines[i].spool = lerp(a.engines[i].spool, b.engines[i].spool, sf);
            out.engines[i].thrust = lerp(a.engines[i].thrust, b.engines[i].thrust, sf);
        }
    }
    out.rotor.nr_rpm = lerp(a.rotor.nr_rpm, b.rotor.nr_rpm, sf);
    out.rotor.collective_deg = lerp(a.rotor.collective_deg, b.rotor.collective_deg, sf);
    out.rotor.tilt_aft_deg = lerp(a.rotor.tilt_aft_deg, b.rotor.tilt_aft_deg, sf);
    out.rotor.tilt_right_deg = lerp(a.rotor.tilt_right_deg, b.rotor.tilt_right_deg, sf);
    out
}

/// Dead reckoning: constant velocity and angular velocity for `e` seconds.
pub fn extrapolate(tan: &Tangent, s: &AcState, e: f64) -> AcState {
    let p = add(tan.to_local(s.lat, s.lon, s.alt as f64), scale(v3(s.vel), e));
    let q = q_norm(q_mul(q_from_rotvec(scale(v3(s.omega), e)), q64(s.rot)));
    let (lat, lon, alt) = tan.to_geo(p);
    let mut out = *s;
    out.t = s.t + e;
    out.lat = lat;
    out.lon = lon;
    out.alt = alt as f32;
    out.rot = q32(q);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 60° banked, level turn at 120 m/s around a point near London.
    pub fn turn(t: f64) -> AcState {
        let tan = Tangent::new(51.5, -0.1);
        let v = 120.0;
        let w = 9.81 * 60f64.to_radians().tan() / v; // rad/s
        let r = v / w;
        let a = w * t;
        // Circle: start heading north (−z), turning right (towards +x).
        let p = [r - r * a.cos(), 800.0, -r * a.sin()];
        let vel = [v * a.sin(), 0.0, -v * a.cos()];
        let (lat, lon, alt) = tan.to_geo(p);
        // Attitude: yaw (heading a, clockwise from north) then bank 60° right.
        let yaw = q_from_rotvec([0.0, -(a + std::f64::consts::PI), 0.0]);
        let bank = q_from_rotvec([0.0, 0.0, 60f64.to_radians()]);
        let rot = q_norm(q_mul(yaw, bank));
        AcState { t, lat, lon, alt: alt as f32, rot: q32(rot), vel: [vel[0] as f32, vel[1] as f32, vel[2] as f32], omega: [0.0, -w as f32, 0.0], ..Default::default() }
    }

    fn err(a: &AcState, b: &AcState) -> f64 {
        let tan = Tangent::new(a.lat, a.lon);
        len(sub(tan.to_local(a.lat, a.lon, a.alt as f64), tan.to_local(b.lat, b.lon, b.alt as f64)))
    }

    #[test]
    fn tangent_round_trip() {
        let tan = Tangent::new(51.5, -0.1);
        let p = [1234.5, 300.0, -4321.0];
        let (la, lo, al) = tan.to_geo(p);
        let q = tan.to_local(la, lo, al);
        assert!(len(sub(p, q)) < 1e-6);
        // Across the antimeridian.
        let tan = Tangent::new(0.0, 179.99);
        let p = tan.to_local(0.0, -179.99, 0.0);
        assert!((p[0] - 0.02 * M_PER_DEG_LAT).abs() < 1e-3, "{p:?}");
    }

    #[test]
    fn hermite_follows_the_turn() {
        // 20 Hz samples of a 7 m/s² (2 g) turn: Hermite error ≪ linear.
        let tan = Tangent::new(51.5, -0.1);
        let (a, b) = (turn(1.0), turn(1.05));
        let mut worst: f64 = 0.0;
        for k in 0..=10 {
            let t = 1.0 + 0.005 * k as f64;
            worst = worst.max(err(&interpolate(&tan, &a, &b, t), &turn(t)));
        }
        assert!(worst < 0.01, "hermite error {worst} m");
        // Attitude halfway is the slerp.
        let mid = interpolate(&tan, &a, &b, 1.025);
        assert!(q_angle(q64(mid.rot), q64(turn(1.025).rot)) < 0.002);
    }

    #[test]
    fn extrapolation_bounded() {
        let tan = Tangent::new(51.5, -0.1);
        let s = turn(0.0);
        // 0.3 s of dead reckoning in a hard turn: chord error ≈ ½ a t² ≈ 0.77 m.
        let e = err(&extrapolate(&tan, &s, 0.3), &turn(0.3));
        assert!(e < 1.0, "extrapolation error {e} m");
        let q = q_angle(q64(extrapolate(&tan, &s, 0.3).rot), q64(turn(0.3).rot));
        assert!(q < 0.01, "attitude error {q} rad");
    }

    #[test]
    fn buffer_plays_out_smoothly() {
        let mut ip = Interpolator::default();
        let mut worst: f64 = 0.0;
        for i in 0..200 {
            let t = i as f64 * 0.05;
            ip.push(turn(t), t + 0.04);
            for k in 0..3 {
                let now = t + 0.04 + k as f64 * 0.0167;
                if let Some(s) = ip.sample(now, 0.0167) {
                    if i > 20 {
                        worst = worst.max(err(&s.state, &turn(s.state.t)));
                        assert_eq!(s.extrapolated, 0.0);
                    }
                }
            }
        }
        assert!(worst < 0.05, "interpolation error {worst} m");
        assert!((ip.interval - 0.05).abs() < 1e-6);
    }

    #[test]
    fn teleport_resets() {
        let mut ip = Interpolator::default();
        ip.push(turn(0.0), 0.0);
        let mut far = turn(0.05);
        far.lat += 0.1; // 11 km
        ip.push(far, 0.05);
        assert_eq!(ip.buf.len(), 1);
    }
}
