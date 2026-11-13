//! Mountain (lee) waves, rotors and lenticular clouds.
//!
//! Stable air crossing a ridge oscillates downstream at the buoyancy
//! frequency N: trapped lee waves of wavelength λ = 2πU/N (5–20 km), crests
//! capped by smooth lenticular clouds where the air is moist, and under the
//! first crest, below ridge-top height, the rotor — reversed, violently
//! turbulent flow. Linear theory gives the streamline displacement downstream
//! of terrain h as the terrain slope convolved with the lee-wave response:
//!
//!   η(x) = −∫₀^S h′(x − s) · cos(k s) · e^(−s/D) ds,   k = N/U,
//!
//! integrated *upwind* only, so nothing happens upwind of a ridge, a narrow
//! ridge (width ~ λ/2) resonates and broad, gentle terrain hardly makes waves.
//! The vertical wind is w = U ∂η/∂x, ramped in from the ground to the ridge
//! tops and fading out in the upper troposphere.
//!
//! [`WaveField::build`] samples the terrain on a grid around the aircraft
//! (fixed to the ground: *field* coordinates, unlike the drifting clouds) and
//! is rebuilt when the aircraft has moved far or the wind changed.

use glam::{Vec2, vec2};

/// Brunt–Väisälä frequency of the stable lower troposphere (1/s).
pub const N_BV: f32 = 0.011;
/// Grid resolution and size.
pub const RES: usize = 96;
pub const CELL: f32 = 600.0;

/// Wave conditions: the wind across the ridges and how stable / moist the air is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaveSpec {
    /// Wind near ridge-top level (m/s, world xz direction of motion).
    pub wind: Vec2,
    /// 0..1: stability (convection breaks the waves up).
    pub stable: f32,
    /// Lenticulars form (moist layer aloft / ACSL reported).
    pub lenticular: bool,
    /// Lenticular level above the regional ground (m).
    pub lent_alt: f32,
}

impl WaveSpec {
    pub fn from_wx(wx: &crate::weather::Wx) -> WaveSpec {
        let wind = wx.wind_aloft() * 1.15;
        let stable = 1.0 - ((wx.cape - 300.0) / 1200.0).clamp(0.0, 0.7);
        let spread = wx.temp_c - wx.dew_c;
        let acsl = wx.metar.as_ref().is_some_and(|m| m.raw.contains("ACSL") || m.raw.contains("LENTICULAR"));
        WaveSpec { wind, stable, lenticular: acsl || (spread < 16.0 && wx.precip == crate::weather::Precip::None), lent_alt: 2600.0 + 100.0 * spread.clamp(0.0, 20.0) }
    }
    /// Strength factor 0..1 from the wind speed (waves need ≳ 20 kt across the ridge).
    pub fn drive(&self) -> f32 {
        let u = self.wind.length();
        smooth(8.0, 17.0, u) * self.stable
    }
}

fn smooth(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The lee-wave field on a grid (field coordinates: fixed to the ground).
#[derive(Clone, Debug)]
pub struct WaveField {
    pub spec: WaveSpec,
    /// Grid centre (field xz).
    pub center: Vec2,
    /// Streamline displacement amplitude (m, + = crest) at ridge-top level.
    eta: Vec<f32>,
    /// Rotor intensity 0..1 (below the crest, under the first wave).
    rotor: Vec<f32>,
    /// Terrain height and the upwind ridge-top height (m, world y).
    ground: Vec<f32>,
    crest: Vec<f32>,
    /// Lenticular cover 0..1.
    lent: Vec<f32>,
    pub max_eta: f32,
    pub wavelength: f32,
}

impl WaveField {
    pub fn empty() -> WaveField {
        WaveField {
            spec: WaveSpec { wind: Vec2::ZERO, stable: 0.0, lenticular: false, lent_alt: 4000.0 },
            center: Vec2::ZERO,
            eta: Vec::new(),
            rotor: Vec::new(),
            ground: Vec::new(),
            crest: Vec::new(),
            lent: Vec::new(),
            max_eta: 0.0,
            wavelength: 0.0,
        }
    }

    pub fn active(&self) -> bool {
        !self.eta.is_empty() && self.max_eta > 20.0
    }

    /// Build the field around `center` (field xz) from a terrain sampler
    /// (field xz → ground height, m).
    pub fn build(spec: WaveSpec, center: Vec2, terrain: &dyn Fn(Vec2) -> f32) -> WaveField {
        let mut f = WaveField::empty();
        f.spec = spec;
        f.center = center;
        let drive = spec.drive();
        let u = spec.wind.length();
        if drive <= 0.0 || u < 1.0 {
            return f;
        }
        let dir = spec.wind / u;
        let k = N_BV / u;
        let lambda = std::f32::consts::TAU / k;
        f.wavelength = lambda;
        let reach = (lambda * 3.0).clamp(10_000.0, 42_000.0);
        let decay = lambda * 2.5;
        // Heights on a padded grid (the upwind fetch included), sampled once.
        let pad = (reach / CELL).ceil() as usize + 2;
        let m = RES + 2 * pad;
        let o = center - Vec2::splat((m as f32 - 1.0) * 0.5 * CELL);
        let mut h = vec![0.0f32; m * m];
        for j in 0..m {
            for i in 0..m {
                let v = terrain(o + vec2(i as f32, j as f32) * CELL);
                h[j * m + i] = if v.is_finite() { v } else { 0.0 };
            }
        }
        let hat = |p: Vec2| -> f32 {
            let q = (p - o) / CELL;
            let (x, y) = (q.x.clamp(0.0, m as f32 - 1.001), q.y.clamp(0.0, m as f32 - 1.001));
            let (i, j) = (x.floor() as usize, y.floor() as usize);
            let (fx, fy) = (x - i as f32, y - j as f32);
            let a = h[j * m + i] * (1.0 - fx) + h[j * m + i + 1] * fx;
            let b = h[(j + 1) * m + i] * (1.0 - fx) + h[(j + 1) * m + i + 1] * fx;
            a * (1.0 - fy) + b * fy
        };
        let g0 = center - Vec2::splat((RES as f32 - 1.0) * 0.5 * CELL);
        let ds = (CELL * 0.5).min(lambda / 16.0);
        let n_s = (reach / ds) as usize;
        f.eta = vec![0.0; RES * RES];
        f.ground = vec![0.0; RES * RES];
        f.crest = vec![0.0; RES * RES];
        f.rotor = vec![0.0; RES * RES];
        f.lent = vec![0.0; RES * RES];
        for j in 0..RES {
            for i in 0..RES {
                let p = g0 + vec2(i as f32, j as f32) * CELL;
                let mut acc = 0.0;
                let mut top: f32 = hat(p);
                let mut low: f32 = top;
                for s in 0..n_s {
                    let sd = (s as f32 + 0.5) * ds;
                    let q = p - dir * sd;
                    // Slope along the wind at the upwind point.
                    let slope = (hat(q + dir * ds * 0.5) - hat(q - dir * ds * 0.5)) / ds;
                    acc += slope * (k * sd).cos() * (-sd / decay).exp() * ds;
                    if sd < lambda * 1.2 {
                        let hq = hat(q);
                        top = top.max(hq);
                        low = low.min(hq);
                    }
                }
                let relief = (top - low).max(0.0);
                // Linear amplitude, capped: waves rarely displace the air by more than ~relief.
                let eta = (acc * 0.9).clamp(-0.85 * relief, 0.85 * relief) * drive;
                let idx = j * RES + i;
                f.eta[idx] = eta;
                f.ground[idx] = hat(p);
                f.crest[idx] = top;
                // Rotor: under a wave crest in the lee (well below the ridge
                // upwind — not the air riding up over a ridge itself).
                let lee = smooth(0.25, 0.5, (top - hat(p)) / relief.max(1.0));
                f.rotor[idx] = (smooth(0.12, 0.45, eta / relief.max(150.0)) * smooth(300.0, 900.0, relief) * lee * drive).clamp(0.0, 1.0);
                f.max_eta = f.max_eta.max(eta.abs());
            }
        }
        if spec.lenticular {
            // Lenses only on the strongest crests (the moist air reaching saturation there).
            let th = f.max_eta * 0.55;
            for (l, e) in f.lent.iter_mut().zip(&f.eta) {
                *l = smooth(th, f.max_eta.max(th + 1.0) * 0.92, *e);
            }
        }
        f
    }

    fn sample(&self, v: &[f32], p: Vec2) -> f32 {
        if v.is_empty() {
            return 0.0;
        }
        let g0 = self.center - Vec2::splat((RES as f32 - 1.0) * 0.5 * CELL);
        let q = (p - g0) / CELL;
        if q.x < 0.0 || q.y < 0.0 || q.x > RES as f32 - 1.001 || q.y > RES as f32 - 1.001 {
            return 0.0;
        }
        let (i, j) = (q.x.floor() as usize, q.y.floor() as usize);
        let (fx, fy) = (q.x - i as f32, q.y - j as f32);
        let a = v[j * RES + i] * (1.0 - fx) + v[j * RES + i + 1] * fx;
        let b = v[(j + 1) * RES + i] * (1.0 - fx) + v[(j + 1) * RES + i + 1] * fx;
        // Fade out toward the grid edge.
        let e = (q.x.min(q.y).min(RES as f32 - 1.0 - q.x).min(RES as f32 - 1.0 - q.y) / 6.0).clamp(0.0, 1.0);
        (a * (1.0 - fy) + b * fy) * e
    }

    /// Streamline displacement (m) at field xz.
    pub fn eta(&self, p: Vec2) -> f32 {
        self.sample(&self.eta, p)
    }
    pub fn lenticular(&self, p: Vec2) -> f32 {
        self.sample(&self.lent, p)
    }

    /// Vertical profile 0..1: from the ground up to the ridge tops, fading
    /// out above ~9 km.
    fn profile(&self, p: Vec2, y: f32) -> f32 {
        let g = self.sample(&self.ground, p);
        let c = self.sample(&self.crest, p).max(g + 100.0);
        smooth(g, g + (c - g) * 0.8 + 100.0, y) * (1.0 - smooth(c + 5000.0, c + 9000.0, y))
    }

    /// The waves' air motion at field point (xz, y): (vertical m/s,
    /// horizontal m/s world xz, turbulence severity 0..4).
    pub fn air(&self, p: Vec2, y: f32) -> (f32, Vec2, f32) {
        if !self.active() {
            return (0.0, Vec2::ZERO, 0.0);
        }
        let u = self.spec.wind.length();
        let dir = self.spec.wind / u.max(1e-3);
        let d = 150.0;
        let slope = (self.eta(p + dir * d) - self.eta(p - dir * d)) / (2.0 * d);
        let prof = self.profile(p, y);
        let w = u * slope * prof;
        // Rotor: below the crest, flow reversed near the ground, severe turbulence.
        let rot = self.sample(&self.rotor, p);
        let crest = self.sample(&self.crest, p);
        let g = self.sample(&self.ground, p);
        let below = 1.0 - smooth(crest * 0.95 + g * 0.05, crest + 400.0, y);
        let r = rot * below * smooth(g - 1.0, g + 80.0, y);
        // Downslope wind: air plunging behind the ridge (negative displacement) speeds up near the ground.
        let eta = self.eta(p);
        let low = 1.0 - smooth(g + 200.0, crest + 300.0, y);
        let downslope = (-eta / 400.0).clamp(0.0, 1.0) * low * 0.5 * u;
        let horiz = dir * (downslope - 0.9 * u * r);
        // Waves are smooth (light chop in strong ones); rotors severe to extreme.
        let sev = (r * 3.6).max(smooth(0.3, 1.0, slope.abs() * 30.0) * prof * 1.2);
        (w, horiz, sev)
    }

    /// Lenticular cover (0..1) on the grid, row-major RES x RES, and the grid
    /// origin (field xz of texel 0) + size (m): the cloud renderer's texture.
    pub fn lenticular_grid(&self) -> Option<(&[f32], Vec2, f32)> {
        (self.spec.lenticular && self.active()).then(|| (self.lent.as_slice(), self.center - Vec2::splat(RES as f32 * 0.5 * CELL), RES as f32 * CELL))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ridge(p: Vec2) -> f32 {
        // A 1500 m north–south ridge at x = 0, ~5 km wide.
        200.0 + 1500.0 * (-(p.x / 2500.0).powi(2)).exp()
    }

    #[test]
    fn waves_form_downwind_of_a_ridge() {
        // 30 kt westerly (air moving east, +x) across the ridge.
        let spec = WaveSpec { wind: vec2(16.0, 0.0), stable: 1.0, lenticular: true, lent_alt: 4000.0 };
        let f = WaveField::build(spec, vec2(10_000.0, 0.0), &ridge);
        assert!(f.active(), "max eta {}", f.max_eta);
        let lambda = f.wavelength;
        assert!(lambda > 7000.0 && lambda < 11_000.0, "λ = 2πU/N: {lambda}");
        // Nothing upwind of the ridge.
        let up = f.air(vec2(-12_000.0, 0.0), 2500.0);
        assert!(up.0.abs() < 0.3, "upwind w {}", up.0);
        // Downwind: oscillating vertical motion, several m/s, alternating along x.
        let ws: Vec<f32> = (0..60).map(|i| f.air(vec2(2000.0 + i as f32 * 400.0, 0.0), 3000.0).0).collect();
        let max = ws.iter().cloned().fold(0.0f32, f32::max);
        let min = ws.iter().cloned().fold(0.0f32, f32::min);
        assert!(max > 1.5 && min < -1.5, "lee waves w range {min}..{max}");
        // The first crest (max displacement) and its rotor and lenticular are downwind, within ~1.2 λ.
        let (mut best, mut at) = (0.0, 0.0);
        for i in 0..200 {
            let x = 3000.0 + i as f32 * 100.0;
            let e = f.eta(vec2(x, 0.0));
            if e > best {
                best = e;
                at = x;
            }
            if e < best * 0.5 && best > 50.0 {
                break;
            }
        }
        assert!(at > 0.4 * lambda && at < 1.3 * lambda, "first crest at {at} m (λ {lambda})");
        assert!(f.lenticular(vec2(at, 0.0)) > 0.3);
        let rotor = f.air(vec2(at, 0.0), ridge(vec2(at, 0.0)) + 150.0);
        assert!(rotor.2 > 2.0, "rotor severity {}", rotor.2);
        assert!(rotor.1.x < 0.0, "rotor flow reversed near the ground: {:?}", rotor.1);
        // Light wind: no waves.
        let calm = WaveField::build(WaveSpec { wind: vec2(4.0, 0.0), ..spec }, vec2(10_000.0, 0.0), &ridge);
        assert!(!calm.active());
        // Flat ground: no waves.
        let flat = WaveField::build(spec, vec2(10_000.0, 0.0), &|_| 100.0);
        assert!(!flat.active());
    }
}
