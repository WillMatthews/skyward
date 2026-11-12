//! One-shot sound effects (touchdowns, thunks, chimes, crash, thunder...).

use super::dsp::*;
use super::samples::{self, Pcm, Voice};
use glam::Vec3;

/// Where a one-shot is heard from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route {
    /// From the airframe: positional outside, felt inside.
    Airframe,
    /// Inside the cockpit (systems bus).
    Cockpit,
    /// Around the listener (ambience), with a pan.
    Listener,
    Ui,
    /// At a fixed world position (`Shot::wpos`): propagation delay at the
    /// start, distance / air absorption / pan updated every block.
    World,
}

#[derive(Clone, Copy, Debug)]
pub enum Kind {
    /// Low thud: sine with a pitch drop + low-passed noise.
    Thud { f0: f32, f1: f32, ph: f32, decay: f32, noise: f32 },
    /// Tyre chirp: noise through a resonant band-pass sweeping f0 → f1.
    Squeal { f0: f32, f1: f32, ph: f32 },
    /// Metallic clank: an impulse into three long-ringing resonators.
    Clank { kicked: bool },
    /// Long low-passed noise swell (thunder, crash tail, doors).
    Rumble { attack: f32, crackle: f32 },
    /// Note sequence (chimes / warnings): `notes` in Hz, 0 = rest.
    Chime { notes: [f32; 8], n: usize, note: f32, ph: [f32; 3], bright: f32 },
    /// Short tick.
    Click,
    /// A songbird phrase: FM chirps.
    Bird { f: f32, ph: f32, syll: f32, n: u32 },
    /// A recorded sample (one-shot, or a loop held for `dur`).
    Sample { v: Voice, p: &'static Pcm },
    /// Sonic-boom N-wave: pressure jumps up, falls linearly to the
    /// negative peak over `len` s, jumps back; shocks rise in `rise` s.
    NWave { len: f32, rise: f32 },
    /// Ringing ears after a violent impact: a high whistle fading out.
    Ring { f: f32, ph: f32 },
}

#[derive(Clone, Copy, Debug)]
pub struct Shot {
    pub kind: Kind,
    pub route: Route,
    pub amp: f32,
    pub pan: f32,
    /// Samples to wait before starting.
    pub delay: u32,
    /// Seconds since start / total duration.
    pub t: f32,
    pub dur: f32,
    pub f: [Svf; 3],
    pub lp: OnePole,
    pub env: f32,
    /// World position (Route::World) and its block-ramped stereo gains.
    pub wpos: Vec3,
    pub wl: Ramp,
    pub wr: Ramp,
    pub air: OnePole,
    /// Low-pass the output (muffled / cockpit versions), 0 = off.
    pub muffle: f32,
    pub mlp: [OnePole; 2],
    /// Crackle burst envelope.
    pub cenv: f32,
}

impl Shot {
    pub fn new(kind: Kind, route: Route, amp: f32, dur: f32) -> Shot {
        Shot {
            kind,
            route,
            amp,
            pan: 0.0,
            delay: 0,
            t: 0.0,
            dur,
            f: [Svf::default(); 3],
            lp: OnePole::default(),
            env: 0.0,
            wpos: Vec3::ZERO,
            wl: Ramp::new(0.0),
            wr: Ramp::new(0.0),
            air: OnePole::default(),
            muffle: 0.0,
            mlp: [OnePole::default(); 2],
            cenv: 0.0,
        }
    }
    /// A recorded sample at `pitch` (None if not decoded yet). `dur` 0 =
    /// the whole sample (one-shots); for loops, how long to hold it.
    pub fn sample(name: &str, pitch: f32, route: Route, amp: f32, dur: f32, sr: f32) -> Option<Shot> {
        let id = samples::id(name);
        let p = samples::pcm(id)?;
        let mut v = Voice::new(id, 0.0);
        v.inc = Ramp::new(p.rate / sr * pitch.max(0.05));
        v.gain = Ramp::new(1.0);
        let d = if dur > 0.0 { dur } else { p.len as f32 / (p.rate * pitch.max(0.05)) + 0.01 };
        Some(Shot::new(Kind::Sample { v, p }, route, amp, d))
    }
    /// Place at a world position (Route::World).
    pub fn at(mut self, pos: Vec3) -> Shot {
        self.route = Route::World;
        self.wpos = pos;
        self
    }
    /// Low-pass the shot at `fc` Hz (2 poles).
    pub fn muffled(mut self, fc: f32, sr: f32) -> Shot {
        self.muffle = fc;
        self.mlp = [OnePole::new(fc, sr); 2];
        self
    }
    pub fn delayed(mut self, secs: f32, sr: f32) -> Shot {
        self.delay = (secs.max(0.0) * sr) as u32;
        self
    }
    pub fn panned(mut self, p: f32) -> Shot {
        self.pan = p;
        self
    }
    /// Prepare filters for this kind.
    pub fn init(mut self, sr: f32, rng: &mut Rng) -> Shot {
        match self.kind {
            Kind::Thud { .. } => {
                self.lp = OnePole::new(180.0, sr);
            }
            Kind::Squeal { f0, .. } => {
                self.f[0] = Svf::new(f0, 9.0, sr);
                self.f[1] = Svf::new(f0 * 2.1, 6.0, sr);
            }
            Kind::Clank { .. } => {
                let base = 380.0 + 900.0 * rng.uni();
                self.f[0] = Svf::new(base, 60.0, sr);
                self.f[1] = Svf::new(base * (2.3 + 0.4 * rng.uni()), 70.0, sr);
                self.f[2] = Svf::new(base * (3.9 + 0.8 * rng.uni()), 50.0, sr);
            }
            Kind::Rumble { .. } => {
                self.f[0] = Svf::new(90.0, 0.7, sr);
                self.f[1] = Svf::new(400.0, 0.7, sr);
                self.f[2] = Svf::new(1400.0 + 1200.0 * rng.uni(), 0.8, sr);
                self.lp = OnePole::new(25.0, sr);
            }
            Kind::Chime { .. } => {
                self.lp = OnePole::new(5000.0, sr);
            }
            Kind::Click => {
                self.f[0] = Svf::new(2600.0, 3.0, sr);
            }
            Kind::Bird { .. } | Kind::Sample { .. } | Kind::Ring { .. } => {}
            Kind::NWave { .. } => {
                // DC guard (the N-wave is zero-mean, but keep the woofer safe).
                self.lp = OnePole::new(12.0, sr);
            }
        }
        self
    }

    /// Next sample; None when finished.
    #[inline]
    pub fn tick(&mut self, sr: f32, rng: &mut Rng) -> Option<f32> {
        if self.delay > 0 {
            self.delay -= 1;
            return Some(0.0);
        }
        let dt = 1.0 / sr;
        self.t += dt;
        if self.t > self.dur {
            return None;
        }
        let t = self.t;
        let u = t / self.dur;
        let a = self.amp;
        let y = match &mut self.kind {
            Kind::Thud { f0, f1, ph, decay, noise } => {
                let f = *f1 + (*f0 - *f1) * (-t * 18.0).exp();
                *ph += f * dt;
                let env = (-t / *decay).exp() * smoothstep(0.0, 0.002, t);
                let n = self.lp.lp(rng.bi()) * 3.0 * *noise;
                (sin_turns(*ph) + n) * env * a
            }
            Kind::Squeal { f0, f1, ph } => {
                let f = *f0 + (*f1 - *f0) * u;
                if ((t * sr) as u32).is_multiple_of(32) {
                    self.f[0].set(f, 9.0, sr);
                    self.f[1].set(f * 2.1, 6.0, sr);
                }
                *ph += f * dt;
                let w = rng.bi();
                let env = smoothstep(0.0, 0.01, t) * (1.0 - u).powf(1.5);
                (self.f[0].bpn(w) * 0.8 + self.f[1].bpn(w) * 0.35 + sin_turns(*ph) * 0.12) * env * a
            }
            Kind::Clank { kicked } => {
                let x = if !*kicked {
                    *kicked = true;
                    1.0
                } else if t < 0.002 {
                    rng.bi() * 0.3
                } else {
                    0.0
                };
                let env = (1.0 - u).max(0.0);
                (self.f[0].bp(x) + self.f[1].bp(x) * 0.7 + self.f[2].bp(x) * 0.5) * 6.0 * env * a
            }
            Kind::Rumble { attack, crackle } => {
                let env = smoothstep(0.0, *attack, t) * (1.0 - u).powf(1.6);
                // Slowly rolling amplitude: thunder's grumble.
                let m = 0.6 + 0.8 * self.lp.lp(rng.bi() * 4.0).abs().min(1.0);
                let w = rng.bi();
                let mut y = self.f[0].lp(w) * 2.2 + self.f[1].bp(w) * 0.6;
                // Crackle: short (~2 ms) noise bursts, not single-sample spikes.
                if *crackle > 0.0 && rng.uni() < *crackle * (1.0 - u) * 0.002 {
                    self.cenv = 0.5 + rng.uni();
                }
                if self.cenv > 1e-3 {
                    self.cenv *= 1.0 - 500.0 * dt;
                    y += self.f[2].bp(rng.bi() * self.cenv * 6.0);
                }
                y * env * m * a
            }
            Kind::Chime { notes, n, note, ph, bright } => {
                let k = ((t / *note) as usize).min(*n - 1);
                let f = notes[k];
                let tn = t - k as f32 * *note;
                if f <= 0.0 {
                    0.0
                } else {
                    for (i, m) in [1.0f32, 2.0, 3.0].iter().enumerate() {
                        ph[i] += f * m * dt;
                    }
                    let env = smoothstep(0.0, 0.006, tn) * (-tn * 5.0 / *note).exp();
                    let tone = sin_turns(ph[0]) + *bright * (0.5 * sin_turns(ph[1]) + 0.3 * sin_turns(ph[2]));
                    self.lp.lp(tone) * env * a
                }
            }
            Kind::Click => {
                let x = if t < 0.0004 { rng.bi() } else { 0.0 };
                self.f[0].bpn(x) * a * (1.0 - u)
            }
            Kind::Bird { f, ph, syll, n } => {
                // Syllables of ~70 ms with gaps; each a fast downward sweep
                // with vibrato.
                let k = (t / *syll) as u32;
                let ts = t - k as f32 * *syll;
                if k >= *n || ts > *syll * 0.7 {
                    0.0
                } else {
                    let s = ts / (*syll * 0.7);
                    let fr = *f * (1.25 - 0.45 * s) * (1.0 + 0.04 * sin_turns(ts * 38.0));
                    *ph += fr * dt;
                    let env = (s * std::f32::consts::PI).sin();
                    sin_turns(*ph) * env * env * a
                }
            }
            Kind::Sample { v, p } => {
                // Loops held for `dur` fade over their last 150 ms.
                let fade = if p.looped { ((self.dur - t) / 0.15).clamp(0.0, 1.0) } else { 1.0 };
                if v.done {
                    return None;
                }
                v.tick(p, samples::kern()) * a * fade
            }
            Kind::NWave { len, rise } => {
                let (len, rise) = (*len, (*rise).max(1e-4));
                // Front shock up, linear expansion, rear shock back to 0.
                let up = smoothstep(0.0, rise, t);
                let back = smoothstep(len, len + rise, t);
                let ramp = 1.0 - 2.0 * ((t - 0.5 * rise) / len).clamp(0.0, 1.0);
                let p = up * ramp * (1.0 - back);
                (p - self.lp.lp(p)) * a
            }
            Kind::Ring { f, ph } => {
                *ph += *f * (1.0 + 0.002 * sin_turns(t * 5.3)) * dt;
                let env = smoothstep(0.0, 0.08, t) * (1.0 - u).powf(2.2);
                (sin_turns(*ph) + 0.08 * sin_turns(*ph * 2.003)) * env * a
            }
        };
        let y = if self.muffle > 0.0 {
            let z = self.mlp[0].lp(y);
            self.mlp[1].lp(z)
        } else {
            y
        };
        self.env = y;
        Some(y)
    }
}

/// Fixed pool of one-shots (no allocation on the audio thread).
pub struct Pool {
    pub shots: [Option<Shot>; 40],
}

impl Default for Pool {
    fn default() -> Self {
        Pool { shots: [None; 40] }
    }
}

impl Pool {
    pub fn add(&mut self, s: Shot) {
        if let Some(slot) = self.shots.iter_mut().find(|s| s.is_none()) {
            *slot = Some(s);
        } else {
            // Steal the oldest.
            if let Some(slot) = self.shots.iter_mut().max_by(|a, b| a.map_or(0.0, |s| s.t).total_cmp(&b.map_or(0.0, |s| s.t))) {
                *slot = Some(s);
            }
        }
    }
}
