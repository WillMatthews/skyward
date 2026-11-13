//! Atmospheric turbulence: the Dryden model of MIL-F-8785C / MIL-HDBK-1797
//! as a *frozen spatial field* the aircraft flies through.
//!
//! * **Intensities and scale lengths** ([`dryden`]): the low-altitude model
//!   (below 1000 ft AGL: σw = 0.1·W20, σu = σv = σw / (0.177 + 0.000823 h)^0.4,
//!   Lw = h, Lu = Lv = h / (0.177 + 0.000823 h)^1.2), the medium / high
//!   altitude model (above 2000 ft: L = 1750 ft, σ from the probability-of-
//!   exceedance table for light / moderate / severe / extreme turbulence) and
//!   a linear blend between 1000 and 2000 ft.
//! * **Field** ([`TurbField`]): three independent isotropic, divergence-free
//!   random-Fourier-mode fields whose energy spectrum is the one the Dryden
//!   spectra follow from (E(k) = 8σ²L (kL)⁴ / (π (1 + (kL)²)³)). Along any
//!   straight line the component along the line therefore has the Dryden
//!   *longitudinal* spectrum and the two across it the *transverse* one —
//!   exactly MIL-F-8785C's Φu, Φv, Φw for flight along the mean wind. Each
//!   component field has its own intensity and scale length (the low-altitude
//!   anisotropy), the field is carried with the mean wind (Taylor's frozen
//!   turbulence) so any number of aircraft see the same air, and spatial
//!   differences of it give the rotational gusts (flight.rs samples it across
//!   the span and along the fuselage).
//!
//! Units: metres, m/s. Heights above the ground.

use glam::{DVec3, Vec3};
use std::f64::consts::TAU;
use std::sync::OnceLock;

const FT: f32 = 0.3048;
const KT: f32 = 0.514_444;

/// Turbulence categories (PIREP / MIL-F-8785C wording). As an `f32`
/// "severity" 0..4 the sim interpolates between them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    None,
    Light,
    Moderate,
    Severe,
    Extreme,
}

impl Category {
    pub fn severity(self) -> f32 {
        self as u8 as f32
    }
    pub fn of(severity: f32) -> Category {
        match severity {
            s if s < 0.5 => Category::None,
            s if s < 1.5 => Category::Light,
            s if s < 2.5 => Category::Moderate,
            s if s < 3.5 => Category::Severe,
            _ => Category::Extreme,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Category::None => "none",
            Category::Light => "light",
            Category::Moderate => "moderate",
            Category::Severe => "severe",
            Category::Extreme => "extreme",
        }
    }
}

/// The player's turbulence setting (Settings › Flight).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Mode {
    /// As the weather makes it.
    #[default]
    Realistic,
    /// Half the gust velocities (and no rotational gusts beyond that).
    Reduced,
    /// Smooth air: no turbulence and no gusts (mean wind and shear only).
    Off,
}

impl Mode {
    pub const ALL: [Mode; 3] = [Mode::Realistic, Mode::Reduced, Mode::Off];
    /// Multiplier on the gust / turbulence velocities.
    pub fn scale(self) -> f32 {
        match self {
            Mode::Realistic => 1.0,
            Mode::Reduced => 0.5,
            Mode::Off => 0.0,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Mode::Realistic => "Realistic",
            Mode::Reduced => "Reduced",
            Mode::Off => "Off",
        }
    }
    pub fn parse(s: &str) -> Option<Mode> {
        Mode::ALL.into_iter().find(|m| m.name().eq_ignore_ascii_case(s))
    }
}

/// Dryden parameters: rms gust velocities and scale lengths for the
/// longitudinal (along the mean wind), lateral and vertical components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dryden {
    pub sigma: Vec3,
    pub length: Vec3,
}

impl Dryden {
    pub const CALM: Dryden = Dryden { sigma: Vec3::ZERO, length: Vec3::splat(533.0) };
}

/// MIL-HDBK-1797 medium / high altitude rms intensity (ft/s) by altitude
/// (ft) for probabilities of exceedance 1e-2 (light), 1e-3 (moderate),
/// 1e-5 (severe), 1e-6 (extreme).
const HI_ALT_FT: [f32; 12] = [500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0, 65000.0, 75000.0, 80000.0];
const HI_SIGMA_FTS: [[f32; 12]; 4] = [
    [6.6, 6.9, 7.4, 6.7, 4.6, 2.7, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0],
    [8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7, 0.0, 0.0, 0.0],
    [15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1],
    [18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2],
];
/// Wind speed at 20 ft (kt) MIL-F-8785C associates with each category.
const W20_KT: [f32; 5] = [0.0, 15.0, 30.0, 45.0, 60.0];

fn table(xs: &[f32], ys: &[f32], x: f32) -> f32 {
    if x <= xs[0] {
        return ys[0];
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            let t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + (ys[i] - ys[i - 1]) * t;
        }
    }
    ys[ys.len() - 1]
}

/// Interpolate the 0..4 severity scale over a per-category table (index 0 = none).
fn by_severity(sev: f32, f: impl Fn(usize) -> f32) -> f32 {
    let s = sev.clamp(0.0, 4.0);
    let i = (s.floor() as usize).min(3);
    let t = s - i as f32;
    f(i) * (1.0 - t) + f(i + 1) * t
}

/// Medium / high altitude rms gust (m/s) at `h_m` for a severity 0..4.
pub fn high_sigma(h_m: f32, severity: f32) -> f32 {
    let h_ft = h_m / FT;
    by_severity(severity, |c| if c == 0 { 0.0 } else { table(&HI_ALT_FT, &HI_SIGMA_FTS[c - 1], h_ft) }) * FT
}

/// The wind at 20 ft (m/s) the low-altitude model uses for a severity.
pub fn w20_for(severity: f32) -> f32 {
    by_severity(severity, |c| W20_KT[c]) * KT
}

/// Dryden parameters at `h` m above the ground. `w20` = wind at 20 ft (m/s,
/// gusts included) drives the low-altitude model; `severity` (0..4) the
/// medium / high altitude one (and raises the low-altitude W20 to the
/// category's when that is stronger, e.g. near a thunderstorm).
pub fn dryden(h: f32, w20: f32, severity: f32) -> Dryden {
    let h = h.max(3.0);
    let h_ft = h / FT;
    let low = {
        let hf = h_ft.min(1000.0);
        let w = w20.max(w20_for(severity)).max(0.0);
        let sw = 0.1 * w;
        let d = 0.177 + 0.000823 * hf;
        let su = sw / d.powf(0.4);
        let lw = hf * FT;
        let lu = hf / d.powf(1.2) * FT;
        Dryden { sigma: Vec3::new(su, su, sw), length: Vec3::new(lu, lu, lw) }
    };
    let high = {
        let s = high_sigma(h, severity);
        Dryden { sigma: Vec3::splat(s), length: Vec3::splat(1750.0 * FT) }
    };
    if h_ft <= 1000.0 {
        low
    } else if h_ft >= 2000.0 {
        high
    } else {
        let t = (h_ft - 1000.0) / 1000.0;
        Dryden { sigma: low.sigma.lerp(high.sigma, t), length: low.length.lerp(high.length, t) }
    }
}

/// Dryden longitudinal one-sided spatial PSD (m²/s² per rad/m): σ² 2L/π / (1 + (LΩ)²).
pub fn psd_long(sigma: f32, l: f32, omega: f32) -> f32 {
    sigma * sigma * 2.0 * l / std::f32::consts::PI / (1.0 + (l * omega).powi(2))
}

/// Dryden transverse one-sided spatial PSD: σ² L/π (1 + 3(LΩ)²) / (1 + (LΩ)²)².
pub fn psd_trans(sigma: f32, l: f32, omega: f32) -> f32 {
    let x2 = (l * omega).powi(2);
    sigma * sigma * l / std::f32::consts::PI * (1.0 + 3.0 * x2) / (1.0 + x2).powi(2)
}

// ------------------------------------------------------------------ field

/// Fourier modes per component field.
const NM: usize = 160;
const K_MIN: f64 = 1.0e-4;
const K_MAX: f64 = 2.5;

#[derive(Clone, Debug)]
struct Comp {
    /// Wavenumber (rad/m), its bin width, unit wave vector, phase, and the
    /// projection of the mode's (solenoidal) amplitude direction on this
    /// component's axis.
    k: Vec<f64>,
    dk: Vec<f64>,
    dir: Vec<DVec3>,
    phase: Vec<f64>,
    proj: Vec<f64>,
}

/// Three frozen random-Fourier-mode turbulence fields (along-wind, cross-wind,
/// vertical), defined in the wind-aligned frame (x along the mean wind,
/// y up, z across). Deterministic for a seed.
#[derive(Clone, Debug)]
pub struct TurbField {
    comps: [Comp; 3],
}

fn hash(mut x: u64) -> u64 {
    x ^= x >> 33;
    x = x.wrapping_mul(0xff51_afd7_ed55_8ccd);
    x ^= x >> 33;
    x = x.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    x ^= x >> 33;
    x
}

fn u01(seed: u64, i: u64) -> f64 {
    (hash(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ i) >> 11) as f64 / (1u64 << 53) as f64
}

/// Isotropic Dryden energy spectrum for unit σ (∫E dk = 3/2).
fn energy(k: f64, l: f64) -> f64 {
    let x = k * l;
    8.0 / std::f64::consts::PI * l * x.powi(4) / (1.0 + x * x).powi(3)
}

impl TurbField {
    pub fn new(seed: u64) -> TurbField {
        let r = (K_MAX / K_MIN).ln() / NM as f64;
        let axes = [DVec3::X, DVec3::Z, DVec3::Y];
        let comps = std::array::from_fn(|c| {
            let s = seed.wrapping_add(c as u64 * 0x1000_0000);
            let mut out = Comp { k: Vec::with_capacity(NM), dk: Vec::with_capacity(NM), dir: Vec::new(), phase: Vec::new(), proj: Vec::new() };
            // R2 low-discrepancy sequence (independent directions and azimuths).
            let (a1, a2) = (0.754_877_666_246_693, 0.569_840_290_998_053);
            for n in 0..NM {
                // Log-spaced, jittered within the bin.
                let k = K_MIN * ((n as f64 + 0.15 + 0.7 * u01(s, 4 * n as u64)) * r).exp();
                out.k.push(k);
                out.dk.push(k * r);
                // Low-discrepancy wave directions (consecutive wavenumbers get
                // well-spread directions: smooth 1D spectra along any line).
                let mu = 2.0 * ((n as f64 * a1 + u01(s, 999)).fract()) - 1.0;
                let az = TAU * ((n as f64 * a2 + u01(s, 998) + u01(s, 4 * n as u64 + 1) * 0.05).fract());
                let st = (1.0 - mu * mu).max(0.0).sqrt();
                let d = DVec3::new(st * az.cos(), mu, st * az.sin());
                // Amplitude direction: in the plane normal to d (divergence free),
                // the projection of this component's axis onto it — the
                // largest share this mode can give the component (the
                // isotropic average over amplitude directions is half of it,
                // which the per-sample normalisation absorbs).
                let e = axes[c];
                let amp = (e - d * d.dot(e)).normalize_or_zero();
                out.dir.push(d);
                out.phase.push(TAU * u01(s, 4 * n as u64 + 3));
                out.proj.push(amp.dot(e));
            }
            out
        });
        TurbField { comps }
    }

    /// The shared field every aircraft flies through.
    pub fn global() -> &'static TurbField {
        static F: OnceLock<TurbField> = OnceLock::new();
        F.get_or_init(|| TurbField::new(0x5eed_7ab1e))
    }

    /// Gust velocity (wind frame: x along the wind, y up, z across) at the
    /// field point `q` (wind-frame metres, already advected with the wind).
    /// Each component is normalised so its variance is exactly σ² for this
    /// mode set (the spectrum's truncation at the band edges included).
    pub fn sample(&self, q: DVec3, d: &Dryden) -> Vec3 {
        let mut out = [0.0f32; 3];
        let sig = [d.sigma.x, d.sigma.z, d.sigma.y];
        let len = [d.length.x, d.length.z, d.length.y];
        for (c, comp) in self.comps.iter().enumerate() {
            if sig[c] <= 0.0 {
                continue;
            }
            let l = len[c].max(1.0) as f64;
            let (mut acc, mut var) = (0.0f64, 0.0f64);
            for n in 0..NM {
                let a2 = 4.0 * energy(comp.k[n], l) * comp.dk[n] * comp.proj[n] * comp.proj[n];
                var += a2 * 0.5;
                let ph = (comp.dir[n].dot(q) * comp.k[n] + comp.phase[n]).rem_euclid(TAU);
                acc += a2.sqrt() * comp.proj[n].signum() * (ph as f32).cos() as f64;
            }
            out[c] = if var > 1e-12 { (acc / var.sqrt()) as f32 * sig[c] } else { 0.0 };
        }
        Vec3::new(out[0], out[2], out[1])
    }
}

/// Rotate a world-frame point (x east, z south) into the wind frame of a
/// mean wind blowing toward `dir` (unit, world xz), and gust velocities back.
pub fn to_wind_frame(p: DVec3, dir: glam::Vec2) -> DVec3 {
    let (c, s) = (dir.x as f64, dir.y as f64);
    DVec3::new(p.x * c + p.z * s, p.y, -p.x * s + p.z * c)
}

pub fn from_wind_frame(v: Vec3, dir: glam::Vec2) -> Vec3 {
    let (c, s) = (dir.x, dir.y);
    Vec3::new(v.x * c - v.z * s, v.y, v.x * s + v.z * c)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal radix-2 FFT (in place, complex as (re, im)).
    fn fft(re: &mut [f64], im: &mut [f64]) {
        let n = re.len();
        let mut j = 0;
        for i in 1..n {
            let mut bit = n >> 1;
            while j & bit != 0 {
                j ^= bit;
                bit >>= 1;
            }
            j |= bit;
            if i < j {
                re.swap(i, j);
                im.swap(i, j);
            }
        }
        let mut len = 2;
        while len <= n {
            let ang = -TAU / len as f64;
            for i in (0..n).step_by(len) {
                for k in 0..len / 2 {
                    let (s, c) = (ang * k as f64).sin_cos();
                    let (a, b) = (i + k, i + k + len / 2);
                    let (xr, xi) = (re[b] * c - im[b] * s, re[b] * s + im[b] * c);
                    re[b] = re[a] - xr;
                    im[b] = im[a] - xi;
                    re[a] += xr;
                    im[a] += xi;
                }
            }
            len <<= 1;
        }
    }

    /// Welch one-sided PSD of `x` sampled every `dx` metres: (Ω rad/m, PSD).
    fn welch(x: &[f64], dx: f64, seg: usize) -> Vec<(f64, f64)> {
        let mut acc = vec![0.0; seg / 2];
        let mut nseg = 0;
        let win: Vec<f64> = (0..seg).map(|i| 0.5 - 0.5 * (TAU * i as f64 / seg as f64).cos()).collect();
        let wpow: f64 = win.iter().map(|w| w * w).sum();
        let mut s = 0;
        while s + seg <= x.len() {
            let mean = x[s..s + seg].iter().sum::<f64>() / seg as f64;
            let mut re: Vec<f64> = (0..seg).map(|i| (x[s + i] - mean) * win[i]).collect();
            let mut im = vec![0.0; seg];
            fft(&mut re, &mut im);
            for k in 1..seg / 2 {
                acc[k] += re[k] * re[k] + im[k] * im[k];
            }
            nseg += 1;
            s += seg / 2;
        }
        // One-sided density per (rad/m): 2 |X|² dx / (2π W).
        (1..seg / 2).map(|k| (TAU * k as f64 / (seg as f64 * dx), 2.0 * acc[k] * dx / (TAU * wpow * nseg as f64))).collect()
    }

    /// Fly a straight line along the mean wind through the field at
    /// medium altitude (L = 533 m): the u component follows the Dryden
    /// longitudinal spectrum, v and w the transverse one, and the
    /// variances match σ².
    #[test]
    fn spectra_match_dryden() {
        let f = TurbField::new(42);
        let d = Dryden { sigma: Vec3::new(2.0, 2.0, 2.0), length: Vec3::splat(533.0) };
        let dx = 8.0;
        let n = 1 << 18;
        let mut u = Vec::with_capacity(n);
        let mut v = Vec::with_capacity(n);
        let mut w = Vec::with_capacity(n);
        for i in 0..n {
            let g = f.sample(DVec3::new(i as f64 * dx, 1200.0, 37.0), &d);
            u.push(g.x as f64);
            v.push(g.z as f64);
            w.push(g.y as f64);
        }
        let var = |x: &[f64]| {
            let m = x.iter().sum::<f64>() / x.len() as f64;
            x.iter().map(|a| (a - m) * (a - m)).sum::<f64>() / x.len() as f64
        };
        for (name, x) in [("u", &u), ("v", &v), ("w", &w)] {
            let s2 = var(x);
            assert!((s2.sqrt() - 2.0).abs() < 0.3, "{name}: sigma {:.2} vs 2.0", s2.sqrt());
        }
        // Log-binned band averages between 1/(10 L) and 1/(0.1 L)... (Ω from 2e-4 to 0.2 rad/m).
        let seg = 8192;
        let check = |x: &[f64], long: bool, name: &str| {
            let p = welch(x, dx, seg);
            let (mut worst, mut mean_err): (f64, f64) = (0.0, 0.0);
            let mut report = String::new();
            let bands: [(f64, f64); 6] = [(1.5e-3, 3e-3), (3e-3, 6e-3), (6e-3, 1.2e-2), (1.2e-2, 2.5e-2), (2.5e-2, 5e-2), (5e-2, 0.1)];
            for (lo, hi) in bands {
                let sel: Vec<&(f64, f64)> = p.iter().filter(|(o, _)| *o >= lo && *o < hi).collect();
                let meas = sel.iter().map(|(_, s)| s).sum::<f64>() / sel.len() as f64;
                let ana = sel.iter().map(|(o, _)| if long { psd_long(2.0, 533.0, *o as f32) } else { psd_trans(2.0, 533.0, *o as f32) } as f64).sum::<f64>() / sel.len() as f64;
                let r = meas / ana;
                report += &format!(" [{lo:.0e}..{hi:.0e}] {r:.2}");
                worst = worst.max(r.ln().abs());
                mean_err += r.ln().abs() / bands.len() as f64;
            }
            eprintln!("{name}: measured/Dryden per band:{report} (mean |ln| {mean_err:.2})");
            // Every band within a factor 2 (a finite set of Fourier modes), the shape on average within ~25%.
            assert!(worst < 2f64.ln(), "{name}: PSD off Dryden by a factor {:.2} ({report})", worst.exp());
            assert!(mean_err < 0.25, "{name}: PSD shape off Dryden: mean |ln ratio| {mean_err:.2} ({report})");
        };
        check(&u, true, "u");
        check(&v, false, "v");
        check(&w, false, "w");
    }

    #[test]
    fn intensity_scaling() {
        // Low altitude: σw = 0.1 W20, σu/σw = 1/(0.177 + 0.000823 h)^0.4, Lw = h.
        let d = dryden(50.0 * FT, 30.0 * KT, 0.0);
        assert!((d.sigma.z - 3.0 * KT).abs() < 1e-3);
        let k = 1.0 / (0.177 + 0.000823 * 50.0f32).powf(0.4);
        assert!((d.sigma.x / d.sigma.z - k).abs() < 1e-3);
        assert!((d.length.z - 50.0 * FT).abs() < 1e-3);
        // Medium / high altitude: L = 1750 ft and the table's σ (moderate at 15 000 ft: 8 ft/s).
        let m = dryden(15000.0 * FT, 0.0, 2.0);
        assert!((m.length.x - 1750.0 * FT).abs() < 1e-2 && m.length == Vec3::splat(m.length.x));
        assert!((m.sigma.x - 8.0 * FT).abs() < 1e-3);
        // Monotone in severity, everywhere.
        for h in [30.0, 200.0, 450.0, 2000.0, 6000.0, 11000.0] {
            let mut last = -1.0;
            for s in 0..=8 {
                let sig = dryden(h, 5.0, s as f32 * 0.5).sigma.length();
                assert!(sig >= last - 1e-4, "h {h}: not monotone at severity {}", s as f32 * 0.5);
                last = sig;
            }
        }
        // Severe ≈ 2.5–3x moderate aloft; calm high air is calm.
        assert!(high_sigma(3000.0, 3.0) > 2.0 * high_sigma(3000.0, 2.0));
        assert_eq!(high_sigma(9000.0, 0.0), 0.0);
        // The blend between 1000 and 2000 ft is continuous.
        let a = dryden(999.0 * FT, 10.0, 1.0);
        let b = dryden(1001.0 * FT, 10.0, 1.0);
        assert!((a.sigma - b.sigma).length() < 0.05 && (a.length - b.length).length() < 2.0);
        let a = dryden(1999.0 * FT, 10.0, 1.0);
        let b = dryden(2001.0 * FT, 10.0, 1.0);
        assert!((a.sigma - b.sigma).length() < 0.05 && (a.length - b.length).length() < 2.0);
        // The field scales linearly with σ.
        let f = TurbField::new(3);
        let q = DVec3::new(123.0, 400.0, -77.0);
        let g1 = f.sample(q, &Dryden { sigma: Vec3::splat(1.0), length: Vec3::splat(300.0) });
        let g3 = f.sample(q, &Dryden { sigma: Vec3::splat(3.0), length: Vec3::splat(300.0) });
        assert!((g3 - 3.0 * g1).length() < 1e-4);
        assert!(Category::of(3.2) == Category::Severe && Category::of(0.2) == Category::None);
    }

    #[test]
    fn far_from_origin_stays_finite_and_smooth() {
        let f = TurbField::global();
        let d = dryden(800.0, 8.0, 2.0);
        let q = DVec3::new(3.0e6, 800.0, -2.0e6);
        let a = f.sample(q, &d);
        let b = f.sample(q + DVec3::new(0.05, 0.0, 0.0), &d);
        assert!(a.is_finite() && (a - b).length() < 0.05, "{a:?} {b:?}");
    }
}
