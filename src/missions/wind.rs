//! A microburst on the final approach: a downburst whose core sinks air at
//! `down_ms` and spreads out near the ground at up to `out_ms` (radially
//! outward), so an aircraft flying through it meets a headwind gain, then
//! the downdraft, then a tailwind — the classic energy trap. Model: radial
//! outflow ∝ (r/R)·e^(1−(r/R)²) (peaks at the core edge), shaped with height
//! (strongest within ~300 m of the ground); downdraft Gaussian in r, fading
//! towards the ground (it turns into the outflow).

use super::defs::Burst;
use glam::{Vec2, Vec3};

/// Air velocity added by the burst at `p` (world), centred at `c` (xz) over
/// ground at `ground` (m), at strength `k` (0..1 during build-up):
/// (horizontal wind vector (x, z), vertical air speed m/s, + = up).
pub fn burst_air(b: &Burst, c: Vec2, ground: f32, p: Vec3, k: f32) -> (Vec2, f32) {
    let d = Vec2::new(p.x, p.z) - c;
    let r = d.length();
    let rr = r / b.radius_m.max(50.0);
    let h = (p.y - ground).max(0.0);
    // Outflow: radial, peaks at r ≈ R/√2 .. R, strongest low down.
    let shape_h = (h / 60.0).min(1.0) * (-(h / 450.0).powi(2)).exp();
    let radial = rr * (1.0 - rr * rr).exp() / 1.1658;
    let out = b.out_ms * radial * shape_h * k;
    let dir = if r > 1.0 { d / r } else { Vec2::ZERO };
    // Downdraft: core, fading near the ground.
    let w = -b.down_ms * (-rr * rr).exp() * (h / 150.0).clamp(0.15, 1.0) * k;
    (dir * out, w)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn microburst_headwind_then_sink_then_tailwind() {
        let b = Burst { at_nm: 2.0, radius_m: 900.0, down_ms: 10.0, out_ms: 15.0, ramp_s: 1.0 };
        // Flying north (−z) through a burst at the origin, 250 m up.
        let along = |z: f32| burst_air(&b, Vec2::ZERO, 0.0, Vec3::new(0.0, 250.0, z), 1.0);
        let (w_in, _) = along(700.0); // before the core: outflow towards +z = headwind
        let (_, down) = along(0.0);
        let (w_out, _) = along(-700.0); // past it: outflow towards −z = tailwind
        assert!(w_in.y > 8.0, "headwind {:?}", w_in);
        assert!(down < -7.0, "downdraft {down}");
        assert!(w_out.y < -8.0, "tailwind {:?}", w_out);
        // Far away: nothing.
        let (w, v) = along(6000.0);
        assert!(w.length() < 0.5 && v.abs() < 0.1);
    }
}
