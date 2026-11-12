//! One-shot sound effects (touchdowns, thunks, chimes, crash, thunder...).

use super::dsp::*;

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
}

impl Shot {
    pub fn new(kind: Kind, route: Route, amp: f32, dur: f32) -> Shot {
        Shot { kind, route, amp, pan: 0.0, delay: 0, t: 0.0, dur, f: [Svf::default(); 3], lp: OnePole::default(), env: 0.0 }
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
                self.lp = OnePole::new(25.0, sr);
            }
            Kind::Chime { .. } => {
                self.lp = OnePole::new(5000.0, sr);
            }
            Kind::Click => {
                self.f[0] = Svf::new(2600.0, 3.0, sr);
            }
            Kind::Bird { .. } => {}
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
                if *crackle > 0.0 && rng.uni() < *crackle * (1.0 - u) * 0.002 {
                    y += rng.bi() * 3.0;
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
