//! Slope (orographic) lift: wind meeting rising ground is deflected upwards
//! along it — the lift gliders soar along ridges — and sinks again in the
//! lee. A thin potential-flow-like model: the vertical air velocity is the
//! wind times the terrain slope along it (taken a little upwind: the flow
//! rises before the face), fading with height above the ground on the scale
//! of the hill (~350 m). Added to the thermal updraft by `World::updraft`.
//!
//! Mountain *waves* (lee waves high above the ridges) belong to the weather
//! model; when it provides them they add to this the same way.

use glam::{Vec2, Vec3, vec2};

/// Height scale (m) over which the slope lift fades above the ground.
pub const DECAY: f32 = 350.0;

/// Vertical air velocity (m/s, + up) at `p` from the wind `w` (m/s, world
/// xz, the direction the air moves, at the point's height) over the terrain
/// `h`.
pub fn slope_lift(h: impl Fn(Vec2) -> f32, p: Vec3, w: Vec2) -> f32 {
    let speed = w.length();
    if speed < 0.5 || !p.is_finite() {
        return 0.0;
    }
    let xz = vec2(p.x, p.z);
    let agl = p.y - h(xz);
    if agl <= 0.0 {
        return 0.0;
    }
    let u = w / speed;
    // Rise per metre along the wind over the ~500 m upwind of the point (and
    // a little beyond it): the face the air is climbing.
    let d = 250.0;
    let up = h(xz - u * (2.0 * d));
    let mid = h(xz - u * d);
    let at = h(xz + u * (0.4 * d));
    let slope = (0.5 * (mid - up) / d + 0.5 * (at - mid) / (1.4 * d)).clamp(-1.0, 1.0);
    // Lee side: separated flow, weaker (and patchier) sink.
    let k = if slope > 0.0 { 1.0 } else { 0.6 };
    (speed * slope * k * (-agl / DECAY).exp()).clamp(-6.0, 8.0)
}

/// Highest-lift direction of travel along a slope at `p`: the contour
/// direction (perpendicular to the terrain gradient) and the uphill unit
/// vector, from the terrain `h` (None on flat ground).
pub fn contour(h: impl Fn(Vec2) -> f32, p: Vec2) -> Option<(Vec2, Vec2)> {
    let d = 120.0;
    let g = vec2(h(p + vec2(d, 0.0)) - h(p - vec2(d, 0.0)), h(p + vec2(0.0, d)) - h(p - vec2(0.0, d))) / (2.0 * d);
    let l = g.length();
    if l < 0.03 {
        return None;
    }
    let n = g / l;
    Some((vec2(-n.y, n.x), n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec3;

    /// A ridge along z: rises from 500 m at x = -2000 to 1500 m at x = 0, then down.
    fn ridge(p: Vec2) -> f32 {
        500.0 + (1000.0 - p.x.abs() * 0.5).max(0.0)
    }

    #[test]
    fn lift_on_the_windward_face_sink_in_the_lee() {
        let w = vec2(10.0, 0.0); // blowing towards +x: the face at x < 0 is windward
        let windward = slope_lift(ridge, vec3(-800.0, 1200.0, 0.0), w);
        let lee = slope_lift(ridge, vec3(800.0, 1200.0, 0.0), w);
        assert!(windward > 2.0, "windward {windward}");
        assert!(lee < -1.0, "lee {lee}");
        // Fades with height; nothing in still air or over flat ground.
        assert!(slope_lift(ridge, vec3(-800.0, 2400.0, 0.0), w) < windward * 0.3);
        assert_eq!(slope_lift(ridge, vec3(-800.0, 1200.0, 0.0), Vec2::ZERO), 0.0);
        assert_eq!(slope_lift(|_| 300.0, vec3(0.0, 500.0, 0.0), w), 0.0);
        // The contour runs along the ridge (z), uphill towards x = 0.
        let (c, n) = contour(ridge, vec2(-800.0, 0.0)).unwrap();
        assert!(c.x.abs() < 1e-3 && n.x > 0.99);
    }
}
