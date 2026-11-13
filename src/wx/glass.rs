//! On the glass: rain on the windscreen, wipers, misting / frost, ice.
//!
//! * **Windscreen panels** ([`panels`]): the front windscreen of each
//!   imported aircraft is found from its glass (translucent geometry ahead of
//!   the pilot's eye whose normals face forward), split into left / right
//!   panes and fitted with planes. Rain drops live *on those planes* (the
//!   shader intersects each view ray with them), so they stay on the glass
//!   when the pilot looks around and the wipers clear exactly what they sweep.
//! * **Wipers** ([`Wipers`]): off / slow / fast / auto (rain-rate driven),
//!   always finishing a stroke back to park; the blades are drawn as 3D arms
//!   on the panes (none of the imported models carries animated wipers).
//! * **Glass state** ([`GlassState`]): rain arriving on the glass (rain rate
//!   and airspeed), how long since each point was wiped (analytic from the
//!   wiper motion), misting (cold glass in humid air, cleared by demisting
//!   over a minute) and frost / ice creeping in from the frame as airframe
//!   icing builds (systems/).

use glam::{Mat4, Quat, Vec2, Vec3, vec3};
use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

/// A fitted windscreen pane (body frame: +Z nose, +Y up, +X left).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Panel {
    /// A point on the plane (centroid) and the unit normal facing the cabin.
    pub origin: Vec3,
    pub normal: Vec3,
    /// In-plane axes: `u` across (toward the left wing), `v` up the glass.
    pub u: Vec3,
    pub v: Vec3,
    /// Extent of the glass in (u, v) around `origin` (m).
    pub lo: Vec2,
    pub hi: Vec2,
}

impl Panel {
    pub fn point(&self, uv: Vec2) -> Vec3 {
        self.origin + self.u * uv.x + self.v * uv.y
    }
    /// Wiper pivot (bottom, toward the outboard side of the pane) and arm length.
    pub fn wiper(&self) -> (Vec2, f32) {
        let w = self.hi.x - self.lo.x;
        let h = self.hi.y - self.lo.y;
        // Outboard = away from the aircraft centreline.
        let outboard = if self.origin.x >= 0.0 { 1.0 } else { -1.0 };
        let cx = (self.lo.x + self.hi.x) * 0.5 + outboard * self.u.x.signum() * w * 0.18;
        (Vec2::new(cx, self.lo.y + h * 0.04), (h * 0.9).min(w * 0.85))
    }
}

/// Fit the windscreen panes of an aircraft asset (cached per id).
pub fn panels(asset: &crate::aircraft_asset::AircraftAsset) -> Vec<Panel> {
    static CACHE: OnceLock<Mutex<HashMap<String, Vec<Panel>>>> = OnceLock::new();
    let cache = CACHE.get_or_init(Default::default);
    if let Some(p) = cache.lock().ok().and_then(|c| c.get(&asset.id).cloned()) {
        return p;
    }
    let p = fit_panels(asset);
    if let Ok(mut c) = cache.lock() {
        c.insert(asset.id.clone(), p.clone());
    }
    p
}

fn fit_panels(a: &crate::aircraft_asset::AircraftAsset) -> Vec<Panel> {
    let Some(eye) = a.eye else { return Vec::new() };
    let mut st = crate::aircraft_asset::AnimState::default();
    let mut pose = crate::aircraft_asset::Pose::default();
    let ch = vec![0.0f32; a.channels.len()];
    a.pose(&ch, &mut st, 0.0, &mut pose);
    // Triangles of glass ahead of the eye facing forward: (points, area-weighted normal).
    let mut sides: [(Vec<Vec3>, Vec3, f32); 2] = [(Vec::new(), Vec3::ZERO, 0.0), (Vec::new(), Vec3::ZERO, 0.0)];
    for c in &a.chunks {
        let m = &a.materials[c.material];
        if !m.translucent || m.col.w > 0.6 || c.display.is_some() || !pose.visible[c.node] {
            continue;
        }
        let xf = pose.world[c.node];
        for t in (c.first..c.first + c.count).step_by(3) {
            let p: [Vec3; 3] = std::array::from_fn(|k| {
                let v = a.vertices[(a.indices[(t + k as u32) as usize] + c.base_vertex) as usize];
                xf.transform_point3(Vec3::from(v.pos))
            });
            let ctr = (p[0] + p[1] + p[2]) / 3.0;
            let d = ctr - eye;
            if d.z < 0.25 || d.length() > 3.0 || d.y < -0.45 {
                continue;
            }
            let mut n = (p[1] - p[0]).cross(p[2] - p[0]);
            let area = n.length() * 0.5;
            if area < 1e-6 {
                continue;
            }
            n /= 2.0 * area;
            if n.dot(d) > 0.0 {
                n = -n; // face the cabin
            }
            if -n.z < 0.45 {
                continue; // side / roof glass
            }
            let s = if ctr.x >= 0.0 { 0 } else { 1 };
            sides[s].0.extend_from_slice(&p);
            sides[s].1 += n * area;
            sides[s].2 += area;
        }
    }
    let mut out = Vec::new();
    for (pts, nsum, area) in sides {
        if area < 0.05 || pts.is_empty() {
            continue;
        }
        let n = nsum.normalize();
        let origin = pts.iter().copied().sum::<Vec3>() / pts.len() as f32;
        let u = Vec3::Y.cross(n).normalize_or_zero();
        let u = if u.x < 0.0 { -u } else { u };
        let v = n.cross(u).normalize_or_zero();
        let v = if v.y < 0.0 { -v } else { v };
        let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
        for p in &pts {
            let q = Vec2::new((*p - origin).dot(u), (*p - origin).dot(v));
            lo = lo.min(q);
            hi = hi.max(q);
        }
        // Keep clear of the frame.
        lo += Vec2::splat(0.02);
        hi -= Vec2::splat(0.02);
        if hi.x - lo.x > 0.2 && hi.y - lo.y > 0.15 {
            out.push(Panel { origin, normal: n, u, v, lo, hi });
        }
    }
    out
}

/// Wiper switch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WiperMode {
    #[default]
    Off,
    Slow,
    Fast,
    /// Speed from the rain rate (off in the dry).
    Auto,
}

impl WiperMode {
    pub fn next(self) -> WiperMode {
        match self {
            WiperMode::Off => WiperMode::Slow,
            WiperMode::Slow => WiperMode::Fast,
            WiperMode::Fast => WiperMode::Auto,
            WiperMode::Auto => WiperMode::Off,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            WiperMode::Off => "off",
            WiperMode::Slow => "slow",
            WiperMode::Fast => "fast",
            WiperMode::Auto => "auto",
        }
    }
}

/// Sweep angle of the wipers (rad, from park).
pub const SWEEP: f32 = 1.75;

/// The wipers: mode and the motion of the blades.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Wipers {
    pub mode: WiperMode,
    /// Cycle phase 0..1 (0 = parked, 0.5 = far end) and whether a stroke is under way.
    pub phase: f32,
    pub running: bool,
    /// Time (s) since the wipers last stopped at park (for the drops that built up since).
    pub since_stop: f32,
    /// Duration of the stroke in progress (s).
    pub stroke: f32,
}

impl Wipers {
    /// Advance by `dt`; `rain` 0..1 at the glass (auto mode).
    pub fn update(&mut self, dt: f32, rain: f32) {
        // (stroke duration, rest at park between strokes)
        let want = match self.mode {
            WiperMode::Off => None,
            WiperMode::Slow => Some((2.2, 0.0)),
            WiperMode::Fast => Some((1.25, 0.0)),
            WiperMode::Auto => {
                if rain > 0.55 {
                    Some((1.25, 0.0))
                } else if rain > 0.15 {
                    Some((2.2, 0.0))
                } else if rain > 0.02 {
                    Some((1.6, 3.5)) // intermittent
                } else {
                    None
                }
            }
        };
        if self.running {
            self.phase += dt / self.stroke.max(0.3);
            if self.phase >= 1.0 {
                match want {
                    Some((s, r)) if r <= 0.0 => {
                        self.stroke = s;
                        self.phase -= 1.0;
                    }
                    _ => {
                        self.running = false;
                        self.phase = 0.0;
                        self.since_stop = 0.0;
                    }
                }
            }
        } else {
            self.since_stop += dt;
            if let Some((s, rest)) = want
                && self.since_stop >= rest
            {
                self.running = true;
                self.stroke = s;
                self.phase = 0.0;
            }
        }
    }

    /// Blade angle from park (rad): smooth out-and-back.
    pub fn angle(&self) -> f32 {
        if !self.running {
            return 0.0;
        }
        SWEEP * (0.5 - 0.5 * (std::f32::consts::TAU * self.phase).cos())
    }

    /// Seconds since the blade last passed angle `a` (from park): drops at
    /// `a` have built up for this long. Analytic from the cycle.
    pub fn age_at(&self, a: f32) -> f32 {
        let stroke = self.stroke.max(0.3);
        let x = (1.0 - 2.0 * (a / SWEEP).clamp(0.0, 1.0)).clamp(-1.0, 1.0);
        // Phases where the blade is at angle a: φ1 (outbound) and 1 - φ1 (return).
        let p1 = x.acos() / std::f32::consts::TAU;
        let p2 = 1.0 - p1;
        if !self.running {
            // Parked: the last pass was on the final return stroke.
            return self.since_stop.max(0.0) + (1.0 - p2) * stroke;
        }
        let ph = self.phase;
        let back = |p: f32| if ph >= p { ph - p } else { ph - p + 1.0 };
        back(p1).min(back(p2)) * stroke
    }
}

/// What the windscreen shader needs about the glass this frame.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct GlassState {
    /// Rain arriving on the glass 0..1 (rain rate and airspeed).
    pub rain: f32,
    /// Airspeed (m/s): drops run up and outboard with the airflow.
    pub airspeed: f32,
    /// Misting 0..1, frost (from the frame inward) 0..1, airframe ice 0..1.
    pub fog: f32,
    pub frost: f32,
    pub ice: f32,
    /// Seconds of rain on the glass (drop build-up clock for unwiped glass).
    pub wet_time: f32,
    pub wipers: Wipers,
    /// Demister on (it is, automatically, on airliners).
    pub demist: bool,
}

impl GlassState {
    /// Advance the glass. `rain_rate` 0..1 (falling rain at the aircraft),
    /// `snow` 0..1, `oat` °C, `dew` °C, `ice` 0..1 (airframe ice), `in_cloud`.
    #[allow(clippy::too_many_arguments)]
    pub fn update(&mut self, dt: f32, rain_rate: f32, snow: f32, airspeed: f32, oat: f32, dew: f32, ice: f32, in_cloud: f32) {
        if !dt.is_finite() || dt <= 0.0 {
            return;
        }
        // At speed the windscreen sweeps up far more drops; at a standstill it's the rain rate.
        let catch = (0.5 + airspeed / 60.0).min(2.5);
        let target = ((rain_rate + 0.6 * in_cloud * (oat > -2.0) as u8 as f32) * catch).clamp(0.0, 1.0);
        self.rain += (target - self.rain) * (1.0 - (-dt / 1.5).exp());
        self.airspeed = airspeed;
        self.wet_time = if self.rain > 0.01 { (self.wet_time + dt).min(600.0) } else { (self.wet_time - dt * 0.2).max(0.0) };
        self.wipers.update(dt, self.rain);
        // Misting: glass colder than the dew point (descending into humid air) — the demister clears it.
        let fog_target = if (oat - dew) < 2.0 && oat < 12.0 { 0.55 } else { 0.0 } * if self.demist { 0.3 } else { 1.0 };
        let k = if fog_target > self.fog { 1.0 / 40.0 } else { 1.0 / 25.0 };
        self.fog += (fog_target - self.fog) * (1.0 - (-dt * k).exp());
        // Frost at the frame edges: sub-zero, moisture (cloud / snow / icing).
        let frost_target = if oat < 0.0 { (0.25 * in_cloud + 0.35 * snow + ice).min(1.0) * smooth(0.0, -6.0, oat).max(0.4) } else { 0.0 };
        let k = if frost_target > self.frost { 1.0 / 60.0 } else { 1.0 / 90.0 };
        self.frost += (frost_target - self.frost) * (1.0 - (-dt * k).exp());
        self.ice = ice.clamp(0.0, 1.0);
    }

    /// Anything to draw?
    pub fn visible(&self) -> bool {
        self.rain > 0.005 || self.wet_time > 0.5 || self.fog > 0.01 || self.frost > 0.01 || self.ice > 0.01
    }
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The wiper arms as geometry (body frame, appended to the aircraft's
/// dynamic mesh): a dark arm and rubber blade on each pane, at `angle`.
pub fn wiper_mesh(panels: &[Panel], angle: f32, out: &mut crate::scene::MeshData, xf: Mat4) {
    for p in panels {
        let (pivot, len) = p.wiper();
        // Park: lying along the bottom of the pane, pointing inboard; sweep up and over.
        let inboard = if p.origin.x >= 0.0 { -1.0 } else { 1.0 } * p.u.x.signum();
        let a0 = if inboard > 0.0 { 0.06 } else { std::f32::consts::PI - 0.06 };
        let a = if inboard > 0.0 { a0 + angle } else { a0 - angle };
        let d = Vec2::new(a.cos(), a.sin());
        let lift = -p.normal * 0.012; // just outside the glass
        let base = p.point(pivot) + lift;
        let tip = p.point(pivot + d * len) + lift;
        let side = (p.u * -d.y + p.v * d.x) * 0.009;
        let outward = -p.normal * 0.012;
        let dark = [0.03, 0.03, 0.035];
        // Arm: a thin flat bar (two faces) and the blade a little longer.
        quad(out, xf, base - side, tip - side, tip + side, base + side, -p.normal, dark);
        let bside = side * 0.6;
        let b0 = p.point(pivot + d * len * 0.12) + lift + outward * 0.5;
        let b1 = tip + outward * 0.5;
        quad(out, xf, b0 - bside, b1 - bside, b1 + bside, b0 + bside, -p.normal, [0.015, 0.015, 0.015]);
    }
}

fn quad(m: &mut crate::scene::MeshData, xf: Mat4, a: Vec3, b: Vec3, c: Vec3, d: Vec3, n: Vec3, col: [f32; 3]) {
    let n = xf.transform_vector3(n).normalize_or_zero();
    let i0 = m.verts.len() as u32;
    for p in [a, b, c, d] {
        m.verts.push(crate::scene::Vert { pos: xf.transform_point3(p).into(), nrm: n.into(), col, uv: [0.0, 0.0], mat: crate::scene::mat::METAL });
    }
    // Both windings: visible from the cabin and from outside.
    m.idx.extend_from_slice(&[i0, i0 + 1, i0 + 2, i0, i0 + 2, i0 + 3, i0, i0 + 2, i0 + 1, i0, i0 + 3, i0 + 2]);
}

/// Body → eye-relative world transform for the wiper mesh.
pub fn body_xf(pos: Vec3, rot: Quat, eye: Vec3) -> Mat4 {
    Mat4::from_rotation_translation(rot, pos - eye)
}

/// The panes in eye-relative world space for the shader: (origin, u, v,
/// lo, hi) per pane, max 2.
pub fn world_panels(panels: &[Panel], pos: Vec3, rot: Quat, eye: Vec3) -> Vec<Panel> {
    panels.iter().take(2).map(|p| Panel { origin: rot * p.origin + pos - eye, normal: rot * p.normal, u: rot * p.u, v: rot * p.v, ..*p }).collect()
}

/// A flat pane in front of the eye for aircraft without fitted glass (test
/// rig / debugging).
pub fn default_panel(eye: Vec3) -> Panel {
    Panel { origin: eye + vec3(0.0, 0.1, 0.8), normal: vec3(0.0, -0.5, -0.866), u: Vec3::X, v: vec3(0.0, 0.866, -0.5), lo: Vec2::new(-0.6, -0.3), hi: Vec2::new(0.6, 0.3) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wipers_sweep_and_park() {
        let mut w = Wipers { mode: WiperMode::Fast, ..Default::default() };
        let mut max: f32 = 0.0;
        for _ in 0..300 {
            w.update(1.0 / 60.0, 0.8);
            max = max.max(w.angle());
        }
        assert!((max - SWEEP).abs() < 0.05, "full sweep {max}");
        // Just-wiped glass is young, the far end of the sweep has waited a while.
        let a_now = w.angle();
        assert!(w.age_at(a_now) < 0.05);
        // Off: finish the stroke and park, then drops build up everywhere.
        w.mode = WiperMode::Off;
        for _ in 0..200 {
            w.update(1.0 / 60.0, 0.8);
        }
        assert_eq!(w.angle(), 0.0);
        assert!(w.age_at(0.8) > 2.0);
        // Auto in the dry: stays parked.
        let mut a = Wipers { mode: WiperMode::Auto, ..Default::default() };
        for _ in 0..120 {
            a.update(1.0 / 60.0, 0.0);
        }
        assert_eq!(a.angle(), 0.0);
    }

    #[test]
    fn glass_wets_mists_and_frosts() {
        let mut g = GlassState::default();
        for _ in 0..600 {
            g.update(0.1, 0.7, 0.0, 70.0, 12.0, 11.0, 0.0, 0.0);
        }
        assert!(g.rain > 0.8 && g.visible());
        let mut f = GlassState::default();
        for _ in 0..1200 {
            f.update(0.1, 0.0, 0.3, 80.0, -8.0, -9.0, 0.6, 1.0);
        }
        assert!(f.frost > 0.5 && f.fog > 0.2, "{f:?}");
    }

    #[test]
    fn airliner_windscreen_is_found() {
        let a = crate::aircraft_asset::AircraftAsset::load("meridian320").expect("A320 asset");
        let p = panels(&a);
        assert_eq!(p.len(), 2, "{p:?}");
        let eye = a.eye.unwrap();
        for q in &p {
            assert!(q.origin.z > eye.z + 0.3 && q.origin.z < eye.z + 1.6, "pane ahead of the eye {q:?}");
            assert!(q.normal.z < -0.4, "faces the cabin {q:?}");
            assert!(q.hi.y - q.lo.y > 0.25 && q.hi.x - q.lo.x > 0.3);
        }
        assert!(p[0].origin.x * p[1].origin.x < 0.0, "left and right panes");
    }
}
