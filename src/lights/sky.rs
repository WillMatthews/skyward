//! Night-sky state for a frame: moon, stars, light-pollution skyglow and the
//! rhythm of the city (street lights on, windows lit, traffic).

use crate::atmosphere::{self, SkyClock, GLOW_AZ, GLOW_EL, LUX};
use glam::{vec3, Mat3, Vec2, Vec3};

/// Everything the renderer needs about the night for one frame.
#[derive(Clone, Copy, Debug)]
pub struct NightState {
    /// Toward the moon (world), its top-of-atmosphere irradiance (scene
    /// units), illuminated fraction and phase angle (rad).
    pub moon_dir: Vec3,
    pub moon_e: f32,
    pub moon_frac: f32,
    pub moon_alpha: f32,
    /// Equatorial → world rotation (stars).
    pub star_rot: Mat3,
    /// Street lights on 0..1 (photocells).
    pub street: f32,
    /// Share of office / home windows lit, traffic density (0..1).
    pub office: f32,
    pub homes: f32,
    pub traffic: f32,
    /// Local clock (hours).
    pub hours: f32,
}

/// Natural night-sky radiance (airglow + unresolved stars, ~22 mag/arcsec²
/// ≈ 0.2 mcd/m²), scene units.
pub const AIRGLOW: f32 = 2.2e-4 * LUX;

/// Colour of the light-pollution glow: a mix of LED and sodium.
pub const GLOW_TINT: Vec3 = Vec3::new(1.0, 0.66, 0.42);

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Piecewise-linear daily curve through (hour, value) points (wraps at 24).
fn daily(h: f32, pts: &[(f32, f32)]) -> f32 {
    let h = h.rem_euclid(24.0);
    let n = pts.len();
    for i in 0..n {
        let (h0, v0) = pts[i];
        let (h1, v1) = if i + 1 < n { pts[i + 1] } else { (pts[0].0 + 24.0, pts[0].1) };
        let hh = if h < h0 { h + 24.0 } else { h };
        if hh >= h0 && hh <= h1 {
            return v0 + (v1 - v0) * (hh - h0) / (h1 - h0).max(1e-3);
        }
    }
    pts[0].1
}

/// Road traffic over the day relative to the rush hours (0..1): quiet in
/// the small hours, peaks around 8:00 and 18:00 (also drives traffic/).
pub fn traffic_level(h: f32) -> f32 {
    daily(h, &[(0.0, 0.3), (3.0, 0.12), (6.0, 0.5), (8.0, 1.0), (10.0, 0.75), (16.5, 0.85), (18.0, 1.0), (20.0, 0.7), (22.5, 0.5)])
}

impl NightState {
    pub fn new(clock: &SkyClock, sun: Vec3, cover: f32) -> NightState {
        let (moon_dir, dist) = atmosphere::moon_direction(clock);
        let (moon_frac, moon_alpha) = atmosphere::moon_phase(moon_dir, sun);
        let moon_e = atmosphere::moon_lux(moon_alpha, dist) * LUX;
        let h = clock.local_hours;
        // Photocells switch at ~35 lux: around sunset, earlier under thick cloud.
        let street = smooth(0.03 + 0.05 * cover, -0.015, sun.y);
        let office = daily(h, &[(0.0, 0.10), (5.5, 0.08), (7.5, 0.55), (9.0, 0.8), (17.0, 0.8), (18.5, 0.55), (20.0, 0.32), (22.0, 0.18)]);
        let homes = daily(h, &[(0.0, 0.13), (1.5, 0.06), (4.5, 0.04), (6.5, 0.22), (8.5, 0.1), (16.0, 0.16), (18.5, 0.36), (21.0, 0.4), (23.0, 0.24)]);
        let traffic = traffic_level(h);
        NightState { moon_dir, moon_e, moon_frac, moon_alpha, star_rot: atmosphere::star_rotation(clock), street, office, homes, traffic, hours: h }
    }
}

/// Solid angle of the moon's disc (sr), at its mean distance.
pub const MOON_SOLID_ANGLE: f32 = std::f32::consts::PI * 0.004_52 * 0.004_52 * 1.8 * 1.8;

/// Mean over the disc of the moon's Lommel–Seeliger shading `2 μ0 / (μ0 + μ)`
/// (1 at full moon) for a phase angle (rad): the disc radiance scale that
/// makes the shaded disc deliver the moon's measured illuminance.
pub fn moon_disc_norm(alpha: f32) -> f32 {
    let (sa, ca) = alpha.sin_cos();
    let n = 24;
    let (mut s, mut c) = (0.0, 0.0);
    for j in 0..n {
        for i in 0..n {
            let x = (i as f32 + 0.5) / n as f32 * 2.0 - 1.0;
            let y = (j as f32 + 0.5) / n as f32 * 2.0 - 1.0;
            let r2 = x * x + y * y;
            if r2 > 1.0 {
                continue;
            }
            let mu = (1.0 - r2).sqrt();
            let mu0 = x * sa + mu * ca;
            if mu0 > 0.0 {
                s += 2.0 * mu0 / (mu0 + mu);
            }
            c += 1.0;
        }
    }
    (s / c).max(1e-3)
}

/// A skyglow source: position (current frame, ground) and upward flux (lm).
pub type GlowSource = (Vec2, f32);

/// Sample distances (m) and lengths along each skyglow ray.
const STEPS: [(f32, f32); 9] = [(80.0, 160.0), (300.0, 280.0), (700.0, 520.0), (1400.0, 900.0), (2600.0, 1500.0), (4500.0, 2300.0), (7500.0, 3700.0), (12000.0, 5300.0), (19000.0, 8700.0)];

/// Light-pollution skyglow: sky radiance (scene units) in `GLOW_EL` x
/// `GLOW_AZ` directions (elevation 90°·(i/7)², azimuth clockwise from north)
/// from single scattering by air and haze of the cities' upward light, plus
/// the upward radiance of the city around the eye (scene units).
pub fn skyglow(sources: &[GlowSource], eye: Vec3, haze: f32) -> ([f32; GLOW_AZ * GLOW_EL], f32) {
    let mut out = [0.0f32; GLOW_AZ * GLOW_EL];
    let eye2 = Vec2::new(eye.x, eye.z);
    // Drop negligible sources (far and faint).
    let src: Vec<(Vec2, f32)> = sources.iter().copied().filter(|(p, lm)| *lm > 0.0 && (*lm / (p.distance(eye2) + 2000.0).powi(2)) > 1e-4).collect();
    // City around the eye: flux density within ~2.5 km → upward radiance.
    let mut up = 0.0;
    {
        let (mut wsum, mut fsum) = (0.0, 0.0);
        for (p, lm) in &src {
            let d2 = p.distance_squared(eye2);
            let w = (-d2 / (2.0 * 2500.0f32.powi(2))).exp();
            wsum += w;
            fsum += w * lm;
        }
        // Each source stands for one GLOW cell (~3 km square in London).
        let cell_area = 3000.0f32 * 3000.0;
        let wnorm = (2.0 * std::f32::consts::PI * 2500.0f32.powi(2)) / cell_area;
        if wsum > 0.0 {
            up = fsum / wnorm.max(1.0) / cell_area / std::f32::consts::PI;
        }
    }
    if src.is_empty() {
        return (out, up * LUX);
    }
    let rayleigh = |h: f32| 13.6e-6 * (-h / 8000.0).exp();
    let mie = |h: f32| {
        let hk = h * 0.001;
        let e = (-hk / 1.2).exp();
        let bl = haze * e * (1.0 + 2.0 * (1.0 - smooth(0.6, 2.2, hk)));
        4.0e-6 * e.max(bl)
    };
    let g = 0.72f32;
    let hg = |c: f32| (1.0 - g * g) / (4.0 * std::f32::consts::PI * (1.0 + g * g - 2.0 * g * c).max(1e-4).powf(1.5));
    let ray_ph = |c: f32| 3.0 / (16.0 * std::f32::consts::PI) * (1.0 + c * c);
    // Ground-level extinction for the light's path (m⁻¹).
    let ext0 = rayleigh(0.0) + mie(0.0) * 1.1;
    for ei in 0..GLOW_EL {
        let el = 0.5 * std::f32::consts::PI * (ei as f32 / 7.0).powi(2);
        let el = el.max(0.012);
        for ai in 0..GLOW_AZ {
            let az = (ai as f32 + 0.5) / GLOW_AZ as f32 * std::f32::consts::TAU;
            let d = vec3(az.sin() * el.cos(), el.sin(), -az.cos() * el.cos());
            let mut l = 0.0;
            for (t, dt) in STEPS {
                let p = eye + d * t;
                let h = p.y.max(1.0);
                let (sr, sm) = (rayleigh(h), mie(h));
                if sr + sm < 1e-9 {
                    continue;
                }
                let t_eye = (-(ext0 * 0.8) * t * (-(h - eye.y).max(0.0) / 2500.0).exp().max(0.3)).exp();
                let mut e = 0.0;
                for (s, lm) in &src {
                    let dx = p.x - s.x;
                    let dz = p.z - s.y;
                    let r2 = dx * dx + dz * dz + h * h;
                    let r = r2.sqrt();
                    // Lambertian upward emission: I(θ) = lm/π · cosθ.
                    let i = lm / std::f32::consts::PI * (h / r);
                    let c = -(dx * d.x + h * d.y + dz * d.z) / r;
                    let ph = (sr * ray_ph(c) + sm * hg(c)) / (sr + sm);
                    e += i / r2 * ph * (-ext0 * r * 0.7).exp();
                }
                l += (sr + sm) * e * dt * t_eye;
            }
            // Calibration: central London's clear-sky zenith ≈ 17 mag/arcsec²
            // (~15 mcd/m²); this single-scattering model with the flux above
            // gives ~1/6 of that (multiple scattering, uplight from signs,
            // floodlit facades and sports grounds are not modelled).
            out[ei * GLOW_AZ + ai] = l * LUX * 6.0;
        }
    }
    (out, up * LUX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn moon_phase_nov_2026() {
        // New moon 2026-11-09 07:02 UT, full moon 2026-11-24 14:53 UT.
        let at = |doy: u32, h: f64| {
            let c = SkyClock { lat: 51.5, lon: 0.0, year: 2026, day_of_year: doy, utc_hours: h, local_hours: h as f32 };
            let (m, _) = atmosphere::moon_direction(&c);
            let sun = atmosphere::sun_direction_zoned(51.5, 0.0, 0.0, doy, h as f32);
            atmosphere::moon_phase(m, sun).0
        };
        let new = at(313, 7.0);
        let full = at(328, 15.0);
        assert!(new < 0.02, "new moon fraction {new}");
        assert!(full > 0.98, "full moon fraction {full}");
    }

    #[test]
    fn moon_rises_in_the_east_on_full_moon() {
        // Full moon 24 Nov 2026: rises around sunset (~16:00 UT) in the north-east, high at midnight in the south.
        let c = SkyClock { lat: 51.5, lon: 0.0, year: 2026, day_of_year: 328, utc_hours: 24.0, local_hours: 0.0 };
        let (m, d) = atmosphere::moon_direction(&c);
        assert!(m.y > 0.6, "high at midnight {m:?}");
        assert!(m.z > 0.3, "in the south {m:?}");
        assert!((356_000.0..407_000.0).contains(&d), "{d}");
    }

    #[test]
    fn polaris_is_north_at_latitude_height() {
        // Polaris: RA 2h31m49s, Dec +89°15'51".
        let c = SkyClock::default();
        let r = atmosphere::star_rotation(&c);
        let (ra, dec) = ((2.0f32 + 31.0 / 60.0 + 49.0 / 3600.0) * 15.0, 89.264f32);
        let (ra, dec) = (ra.to_radians(), dec.to_radians());
        let p = r * vec3(dec.cos() * ra.cos(), dec.cos() * ra.sin(), dec.sin());
        let alt = p.y.asin().to_degrees();
        assert!((alt - 51.5).abs() < 1.0, "alt {alt}");
        assert!(p.z < -0.5, "north {p:?}");
    }

    #[test]
    fn skyglow_brighter_toward_city() {
        let src: Vec<GlowSource> = (0..9).map(|i| (Vec2::new(-15000.0 + (i % 3) as f32 * 3000.0, (i / 3) as f32 * 3000.0 - 3000.0), 9.0e6)).collect();
        let (t, _) = skyglow(&src, vec3(0.0, 300.0, 0.0), 2.0);
        // Toward the west (az 270°: bin 12) at the horizon vs east (bin 4).
        let west = t[12];
        let east = t[4];
        assert!(west > east * 3.0, "west {west} east {east}");
        assert!(t[7 * GLOW_AZ] > 0.0);
    }
}
