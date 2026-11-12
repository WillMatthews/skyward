//! Moisture aloft and the condensation criteria behind the visual effects:
//! contrails (Schmidt–Appleman), and condensation in low-pressure regions
//! (wingtip / flap-tip vortex cores, the suction peak over a wing at high
//! lift, the Prandtl–Glauert expansion around Mach 1).

use crate::world::World;
use glam::Vec3;

/// The air at a point.
#[derive(Clone, Copy, Debug, Default)]
pub struct Air {
    /// Temperature and dew point (°C).
    pub t_c: f32,
    pub td_c: f32,
    /// Relative humidity over water and over ice (0..1+).
    pub rh_w: f32,
    pub rh_i: f32,
    /// Static pressure (Pa) and density (kg/m³).
    pub p: f32,
    pub rho: f32,
}

/// Saturation vapour pressure over water / ice (Pa), Magnus.
pub fn e_sat_w(t_c: f32) -> f32 {
    611.2 * (17.62 * t_c / (243.12 + t_c)).exp()
}
pub fn e_sat_i(t_c: f32) -> f32 {
    611.2 * (22.46 * t_c / (272.62 + t_c)).exp()
}

/// Dew point (°C) of vapour pressure `e` (Pa).
fn dew_of(e: f32) -> f32 {
    let l = (e.max(1e-3) / 611.2).ln();
    243.12 * l / (17.62 - l)
}

fn hash3(p: Vec3) -> f32 {
    let n = (p.x.floor() as i32).wrapping_mul(73856093) ^ (p.y.floor() as i32).wrapping_mul(19349663) ^ (p.z.floor() as i32).wrapping_mul(83492791);
    let mut x = n as u32;
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    (x >> 8) as f32 / 16_777_216.0
}

/// Smooth value noise 0..1 (for patchy upper-air humidity).
fn vnoise(p: Vec3) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = f * f * (Vec3::splat(3.0) - 2.0 * f);
    let mut acc = 0.0;
    for k in 0..8 {
        let o = Vec3::new((k & 1) as f32, ((k >> 1) & 1) as f32, ((k >> 2) & 1) as f32);
        let w = (if o.x > 0.5 { u.x } else { 1.0 - u.x }) * (if o.y > 0.5 { u.y } else { 1.0 - u.y }) * (if o.z > 0.5 { u.z } else { 1.0 - u.z });
        acc += w * hash3(i + o);
    }
    acc
}

/// Temperature, humidity and pressure at a world point, from the surface
/// observation (temperature / dew point), the cloud layers and, aloft, the
/// high-cloud cover (ice supersaturation is patchy: contrails persist in some
/// regions and not others).
pub fn air_at(world: &World, pos: Vec3, isa_only: bool) -> Air {
    let h = pos.y.max(0.0);
    let atm = crate::flight::isa(h);
    let (t0, td0) = if isa_only { (15.0, 8.0) } else { (world.wx.temp_c, world.wx.dew_c.min(world.wx.temp_c)) };
    let hk = h / 1000.0;
    let t = if hk < 11.0 { t0 - 6.5 * hk } else { t0 - 6.5 * 11.0 };
    // Mixed layer: the dew point falls slowly with height (≈ 1.8 K/km).
    let mut td = (td0 - 1.8 * hk).min(t);
    let mut rh_w = e_sat_w(td) / e_sat_w(t);
    // In / at the base of reported cloud layers the air is saturated.
    if !isa_only {
        for l in &world.wx.layers {
            let base = l.base_ft * 0.3048;
            let cov = l.cover.fraction();
            let near = 1.0 - ((h - base - 150.0).abs() / 450.0).min(1.0);
            rh_w = rh_w.max(near * (0.75 + 0.27 * cov));
        }
    }
    // Upper troposphere: ice-supersaturated regions (cirrus country).
    let mut rh_i = rh_w * e_sat_w(t) / e_sat_i(t).max(1e-6);
    if hk > 6.0 {
        let high = if isa_only { 0.3 } else { world.wx.om.as_ref().map(|o| o.cloud_high).unwrap_or(0.3) };
        let patch = vnoise(Vec3::new(pos.x / 40_000.0, hk / 1.5, pos.z / 40_000.0));
        let aloft = 0.62 + 0.35 * high + 0.42 * (patch - 0.5) * 2.0;
        let blend = ((hk - 6.0) / 2.0).clamp(0.0, 1.0);
        rh_i = rh_i * (1.0 - blend) + aloft.clamp(0.1, 1.35) * blend;
        rh_w = rh_i * e_sat_i(t) / e_sat_w(t);
        td = dew_of(rh_w * e_sat_w(t));
    }
    Air { t_c: t, td_c: td.min(t), rh_w: rh_w.clamp(0.0, 1.0), rh_i: rh_i.max(0.0), p: atm.pressure, rho: atm.density }
}

/// Schmidt–Appleman criterion (Schumann 1996): does a jet exhaust plume
/// saturate as it mixes into this air? Returns the threshold temperature
/// (°C) for the ambient humidity; contrails form below it.
pub fn contrail_threshold(p: f32, rh_w: f32) -> f32 {
    // Mixing-line slope G (Pa/K): EI_H2O cp p / (eps Q (1 - eta)).
    let g = 1.25 * 1004.0 * p / (0.622 * 43.2e6 * (1.0 - 0.3));
    let x = (g - 0.053).max(1e-3).ln();
    let t_lm = -46.46 + 9.43 * x + 0.72 * x * x;
    // Liquid-saturation threshold for humid air (first-order correction).
    t_lm - (1.0 - rh_w.clamp(0.0, 1.0)) * e_sat_w(t_lm) / g
}

/// Contrail state behind a jet: (forms 0..1, lifetime s, spreading m/s).
pub fn contrail(air: &Air) -> (f32, f32, f32) {
    let thr = contrail_threshold(air.p, air.rh_w);
    let forms = ((thr - air.t_c) / 1.5).clamp(0.0, 1.0);
    if forms <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    // Persistent (and spreading) in ice-supersaturated air, otherwise the ice
    // sublimates within seconds to a minute.
    if air.rh_i >= 1.0 {
        let s = (air.rh_i - 1.0).min(0.3);
        (forms, 240.0, 0.25 + 1.5 * s)
    } else {
        let life = 1.5 + 60.0 * (air.rh_i / 1.0).powi(6);
        (forms, life, 0.15)
    }
}

/// Adiabatic cooling (K) of air whose pressure drops by `dp` (Pa).
pub fn cooling(air: &Air, dp: f32) -> f32 {
    let tk = air.t_c + 273.15;
    tk * (1.0 - (1.0 - (dp / air.p).clamp(0.0, 0.9)).powf(0.2857))
}

/// Visible condensation (0..1) when air is cooled by `dt_cool` K; scaled by
/// the water content (cold, dry air at height makes thin vapour).
pub fn condensation(air: &Air, dt_cool: f32) -> f32 {
    let spread = (air.t_c - air.td_c).max(0.0);
    let x = ((dt_cool - spread) / 1.2).clamp(0.0, 1.0);
    let water = (e_sat_w(air.td_c) / e_sat_w(10.0)).clamp(0.15, 1.5);
    x * x * (3.0 - 2.0 * x) * water.min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schmidt_appleman_threshold() {
        // At 250 hPa: ≈ −42 °C at liquid saturation, ≈ −50 °C in dry air.
        let dry = contrail_threshold(25_000.0, 0.0);
        let wet = contrail_threshold(25_000.0, 1.0);
        eprintln!("T_crit 250 hPa: dry {dry:.1} °C, saturated {wet:.1} °C");
        assert!((-55.0..-45.0).contains(&dry), "{dry}");
        assert!((-44.0..-38.0).contains(&wet), "{wet}");
        // FL350 ISA (−54 °C): contrails form; at 4 km (−11 °C) they don't.
        let fl350 = Air { t_c: -54.3, td_c: -60.0, rh_w: 0.5, rh_i: 0.9, p: 23_840.0, rho: 0.38 };
        assert!(contrail(&fl350).0 > 0.9);
        let low = Air { t_c: -11.0, td_c: -15.0, rh_w: 0.7, rh_i: 0.8, p: 61_640.0, rho: 0.82 };
        assert_eq!(contrail(&low).0, 0.0);
        // Ice-supersaturated: persistent.
        let ss = Air { rh_i: 1.15, ..fl350 };
        assert!(contrail(&ss).1 > 100.0 && contrail(&fl350).1 < 60.0);
    }

    #[test]
    fn vortex_condensation_needs_humid_air() {
        let humid = Air { t_c: 12.0, td_c: 11.8, rh_w: 0.99, rh_i: 1.0, p: 101_000.0, rho: 1.22 };
        let dry = Air { td_c: 2.0, ..humid };
        // A strong vortex core (≈ 1 kPa drop).
        let c = cooling(&humid, 1000.0);
        assert!(c > 0.6 && c < 1.2, "{c}");
        assert!(condensation(&humid, c) > 0.3);
        assert_eq!(condensation(&dry, c), 0.0);
    }
}
