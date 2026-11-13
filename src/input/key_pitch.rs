//! Keyboard pitch: the pilot's hand on the stick.
//!
//! A key is a switch, but a pilot's push or pull is a force: on a real
//! aeroplane the stick force per g (≈ 3–10 lbf/g on light aircraft, 20–50
//! lbf/g on transports) and the seat of the pants stop a firm pull well short
//! of the structure's limit. Without that feel a held key slammed the elevator
//! to its stop in under half a second, which at cruise speed is several g
//! either way (Citation X at 315 kt: +6.7 / −3.0 g; ultimate is +4.35 / −1.5).
//!
//! So a held pitch key moves the stick progressively (≈ 0.7 s to the stop at
//! manoeuvring speed, proportionally slower as (V / V_A)² grows: constant g
//! onset), and on stick-to-surface (Direct law) aircraft it eases off when the felt
//! load factor (with a short look-ahead) nears a firm-manoeuvre band:
//! 80 % of the positive limit and 60 % of the negative limit — a push-over to
//! 0 g or a firm push to −0.9 g in a light aircraft, never into permanent set.
//! Releasing the key re-centres quickly. Analog sticks / yokes are not
//! limited: a pilot can still break the aeroplane with real stick inputs.

/// Stick travel rates (full deflection per second).
pub const RATE_OUT: f32 = 1.5;
pub const RATE_BACK: f32 = 3.5;
/// The firm-manoeuvre band as fractions of the limit load factors.
pub const BAND_POS: f32 = 0.8;
pub const BAND_NEG: f32 = 0.6;
/// Look-ahead on the load factor (s) and the back-off gain (stick / g / s).
const LEAD: f32 = 0.5;
const GAIN: f32 = 10.0;

/// What the pilot feels: the firm-manoeuvre band (g) and how much faster
/// than at manoeuvring speed the aeroplane answers the stick.
#[derive(Clone, Copy, Debug)]
pub struct Feel {
    pub hi: f32,
    pub lo: f32,
    /// (V / V_A)², at least 1: the dynamic pressure relative to the speed at
    /// which full back stick just reaches the band (V_A = Vs √n).
    pub q_ratio: f32,
}

impl Feel {
    /// From the limit load factors (+, −), the clean stall speed and the
    /// indicated airspeed (kt).
    pub fn new(n_lim: f32, n_lim_neg: f32, vs1_kt: f32, ias_kt: f32) -> Feel {
        let hi = 1.0 + BAND_POS * (n_lim - 1.0).max(0.0);
        let va = vs1_kt.max(10.0) * hi.sqrt();
        Feel { hi, lo: BAND_NEG * n_lim_neg.min(0.0), q_ratio: (ias_kt / va).powi(2).max(1.0) }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct KeyPitch {
    g_prev: Option<f32>,
    g_rate: f32,
}

impl KeyPitch {
    /// One step. `stick`: current stick (−1..1, + = back); `key`: +1 pull,
    /// −1 push, 0 none; `g`: load factor; `feel`: None for no g feel (FBW
    /// aircraft, on the ground).
    pub fn step(&mut self, stick: f32, key: f32, g: f32, feel: Option<Feel>, dt: f32) -> f32 {
        if dt <= 0.0 {
            return stick;
        }
        let g = if g.is_finite() { g } else { 1.0 };
        let rate = (g - self.g_prev.unwrap_or(g)) / dt;
        self.g_prev = Some(g);
        self.g_rate += (rate.clamp(-60.0, 60.0) - self.g_rate) * (1.0 - (-dt / 0.05f32).exp());
        let toward = |cur: f32, target: f32, r: f32| cur + (target - cur).clamp(-r * dt, r * dt);
        if key.abs() < 0.5 {
            return toward(stick, 0.0, RATE_BACK);
        }
        // Above manoeuvring speed a given stick movement is worth more g: the
        // hand moves the stick proportionally slower (constant g onset).
        let out = RATE_OUT / feel.map_or(1.0, |f| f.q_ratio);
        let mut s = toward(stick, key.signum(), if stick * key < 0.0 { RATE_BACK } else { out });
        if let Some(f) = feel {
            let g_pred = g + LEAD * self.g_rate;
            let over = if key > 0.0 { g_pred - f.hi } else { f.lo - g_pred };
            if over > 0.0 {
                // Ease off: no further travel, and back towards neutral in
                // proportion to the excess (never past it).
                let held = if key > 0.0 { stick.min(s) } else { stick.max(s) };
                let back = (GAIN * over * dt).min(held.abs());
                s = held - back * key.signum();
            }
        }
        s.clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn progressive_travel_and_recentre() {
        let mut k = KeyPitch::default();
        let mut s = 0.0;
        for _ in 0..30 {
            s = k.step(s, -1.0, 1.0, None, 1.0 / 120.0);
        }
        // 0.25 s held: ~0.38 of the travel, not the stop.
        assert!((s + 0.375).abs() < 0.01, "{s}");
        for _ in 0..120 {
            s = k.step(s, -1.0, 1.0, None, 1.0 / 120.0);
        }
        assert_eq!(s, -1.0);
        for _ in 0..40 {
            s = k.step(s, 0.0, 1.0, None, 1.0 / 120.0);
        }
        assert_eq!(s, 0.0);
    }

    #[test]
    fn eases_off_beyond_the_band() {
        let band = Feel::new(2.5, -1.0, 100.0, 100.0);
        assert!((band.hi - 2.2).abs() < 1e-5 && (band.lo + 0.6).abs() < 1e-5 && band.q_ratio == 1.0);
        let mut k = KeyPitch::default();
        // Pushing at −0.8 g (beyond −0.6): the stick goes back towards neutral.
        let mut s = -0.5;
        for _ in 0..12 {
            s = k.step(s, -1.0, -0.8, Some(band), 1.0 / 120.0);
        }
        assert!(s > -0.5 && s <= 0.0, "{s}");
        // Inside the band it keeps going.
        let mut s2 = -0.5;
        for _ in 0..12 {
            s2 = k.step(s2, -1.0, 0.2, Some(band), 1.0 / 120.0);
        }
        assert!(s2 < -0.6, "{s2}");
    }
}
