//! Small, allocation-free DSP building blocks for the procedural sound
//! engine. Everything here runs on the audio thread (or the offline
//! renderer): no locks, no allocation after construction.

use std::f32::consts::{PI, TAU};

/// Fast deterministic white noise (xorshift32).
#[derive(Clone, Copy, Debug)]
pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Rng {
        Rng(seed.wrapping_mul(0x9e37_79b9) | 1)
    }
    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    /// Uniform in [0, 1).
    #[inline]
    pub fn uni(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }
    /// Uniform in [-1, 1).
    #[inline]
    pub fn bi(&mut self) -> f32 {
        self.uni() * 2.0 - 1.0
    }
    /// Approximately Gaussian (sum of three uniforms), unit variance.
    #[inline]
    pub fn gauss(&mut self) -> f32 {
        (self.bi() + self.bi() + self.bi()) * 1.0
    }
}

/// Pink noise (Paul Kellet's economy filter), roughly unit RMS.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pink {
    b: [f32; 3],
}

impl Pink {
    #[inline]
    pub fn tick(&mut self, white: f32) -> f32 {
        self.b[0] = 0.99765 * self.b[0] + white * 0.0990460;
        self.b[1] = 0.96300 * self.b[1] + white * 0.2965164;
        self.b[2] = 0.57000 * self.b[2] + white * 1.0526913;
        (self.b[0] + self.b[1] + self.b[2] + white * 0.1848) * 0.45
    }
}

/// One-pole low-pass; `hp()` gives the matching high-pass.
#[derive(Clone, Copy, Debug, Default)]
pub struct OnePole {
    pub z: f32,
    a: f32,
}

impl OnePole {
    pub fn new(fc: f32, sr: f32) -> OnePole {
        let mut f = OnePole::default();
        f.set(fc, sr);
        f
    }
    #[inline]
    pub fn set(&mut self, fc: f32, sr: f32) {
        self.a = 1.0 - (-TAU * fc.clamp(1.0, sr * 0.49) / sr).exp();
    }
    #[inline]
    pub fn lp(&mut self, x: f32) -> f32 {
        self.z += self.a * (x - self.z);
        self.z
    }
    #[inline]
    pub fn hp(&mut self, x: f32) -> f32 {
        x - self.lp(x)
    }
}

/// Topology-preserving-transform state-variable filter (Simper). Stable
/// under fast modulation; gives LP / BP / HP from one state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Svf {
    ic1: f32,
    ic2: f32,
    a1: f32,
    a2: f32,
    a3: f32,
    k: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SvfOut {
    pub lp: f32,
    pub bp: f32,
    pub hp: f32,
}

impl Svf {
    pub fn new(fc: f32, q: f32, sr: f32) -> Svf {
        let mut s = Svf::default();
        s.set(fc, q, sr);
        s
    }
    #[inline]
    pub fn set(&mut self, fc: f32, q: f32, sr: f32) {
        let g = (PI * fc.clamp(5.0, sr * 0.45) / sr).tan();
        let k = 1.0 / q.max(0.05);
        self.k = k;
        self.a1 = 1.0 / (1.0 + g * (g + k));
        self.a2 = g * self.a1;
        self.a3 = g * self.a2;
    }
    #[inline]
    pub fn tick(&mut self, x: f32) -> SvfOut {
        let v3 = x - self.ic2;
        let v1 = self.a1 * self.ic1 + self.a2 * v3;
        let v2 = self.ic2 + self.a2 * self.ic1 + self.a3 * v3;
        self.ic1 = 2.0 * v1 - self.ic1;
        self.ic2 = 2.0 * v2 - self.ic2;
        SvfOut { lp: v2, bp: v1, hp: x - self.k * v1 - v2 }
    }
    #[inline]
    pub fn lp(&mut self, x: f32) -> f32 {
        self.tick(x).lp
    }
    #[inline]
    pub fn bp(&mut self, x: f32) -> f32 {
        self.tick(x).bp
    }
    #[inline]
    pub fn hp(&mut self, x: f32) -> f32 {
        self.tick(x).hp
    }
    /// Band-pass normalised to unity gain at the centre.
    #[inline]
    pub fn bpn(&mut self, x: f32) -> f32 {
        self.tick(x).bp * self.k
    }
}

/// Per-sample linear ramp towards a target set once per control block —
/// parameter changes never step (no zipper noise).
#[derive(Clone, Copy, Debug, Default)]
pub struct Ramp {
    pub v: f32,
    step: f32,
    left: u32,
}

impl Ramp {
    pub fn new(v: f32) -> Ramp {
        Ramp { v, step: 0.0, left: 0 }
    }
    #[inline]
    pub fn to(&mut self, target: f32, n: u32) {
        let t = if target.is_finite() { target } else { 0.0 };
        if n == 0 {
            self.v = t;
            self.left = 0;
        } else {
            self.step = (t - self.v) / n as f32;
            self.left = n;
        }
    }
    #[inline]
    pub fn tick(&mut self) -> f32 {
        if self.left > 0 {
            self.v += self.step;
            self.left -= 1;
        }
        self.v
    }
}

/// Exponential smoother with separate rise / fall time constants (seconds),
/// stepped once per control block.
#[derive(Clone, Copy, Debug, Default)]
pub struct Follow {
    pub v: f32,
}

impl Follow {
    #[inline]
    pub fn go(&mut self, target: f32, dt: f32, rise: f32, fall: f32) -> f32 {
        let tau = if target > self.v { rise } else { fall };
        let a = if tau <= 0.0 { 1.0 } else { 1.0 - (-dt / tau).exp() };
        if target.is_finite() {
            self.v += (target - self.v) * a;
        }
        self.v
    }
}

/// Phase accumulator (0..1).
#[derive(Clone, Copy, Debug, Default)]
pub struct Phasor {
    pub p: f32,
}

impl Phasor {
    /// Advance by `inc` (= f / sr); returns the phase before advancing and
    /// whether it wrapped.
    #[inline]
    pub fn tick(&mut self, inc: f32) -> f32 {
        let p = self.p;
        self.p += inc;
        if self.p >= 1.0 {
            self.p -= self.p.floor();
        }
        p
    }
}

/// Fast sine of a phase in turns (0..1), parabolic approximation with one
/// correction step (~0.1 % error) — plenty for audio partials.
#[inline]
pub fn sin_turns(p: f32) -> f32 {
    let x = p - p.floor(); // 0..1
    let x = x * 2.0 - 1.0; // -1..1 maps to -π..π
    let y = 4.0 * x * (1.0 - x.abs());
    let y = 0.225 * (y * y.abs() - y) + y;
    -y
}

/// Band-limited periodic table (one period), read with linear interpolation.
#[derive(Clone, Debug)]
pub struct Table {
    pub data: Vec<f32>,
}

pub const TABLE_N: usize = 4096;

impl Table {
    /// Band-limit an arbitrary one-period waveform (`TABLE_N` points) to its
    /// first `harmonics` partials (FFT, zero the rest, inverse FFT; DC
    /// removed) and normalise to peak 1. ~1 ms, so it is cheap enough to run
    /// when the aircraft changes.
    pub fn from_wave(wave: &[f32], harmonics: usize) -> Table {
        let n = TABLE_N;
        let mut re: Vec<f32> = (0..n).map(|i| wave.get(i).copied().unwrap_or(0.0)).collect();
        let mut im = vec![0.0f32; n];
        fft(&mut re, &mut im);
        for k in 0..n {
            let h = k.min(n - k);
            if h == 0 || h > harmonics {
                re[k] = 0.0;
                im[k] = 0.0;
            }
        }
        // Inverse FFT via conjugation.
        for v in im.iter_mut() {
            *v = -*v;
        }
        fft(&mut re, &mut im);
        let mut data: Vec<f32> = re.iter().map(|v| v / n as f32).collect();
        let peak = data.iter().fold(1e-9f32, |m, v| m.max(v.abs()));
        for d in &mut data {
            *d /= peak;
        }
        data.push(data[0]);
        Table { data }
    }
    #[inline]
    pub fn at(&self, phase: f32) -> f32 {
        let x = (phase - phase.floor()) * TABLE_N as f32;
        let i = x as usize;
        let f = x - i as f32;
        let i = i.min(TABLE_N - 1);
        self.data[i] + (self.data[i + 1] - self.data[i]) * f
    }
}

/// In-place iterative radix-2 complex FFT (forward; length a power of two).
pub fn fft(re: &mut [f32], im: &mut [f32]) {
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
    // Exact twiddles for the largest stage; smaller stages stride through them.
    let tw: Vec<(f32, f32)> = (0..n / 2).map(|k| (-TAU * k as f32 / n as f32).sin_cos()).collect();
    let mut len = 2;
    while len <= n {
        let stride = n / len;
        for i in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wi, wr) = tw[k * stride];
                let (ar, ai) = (re[i + k], im[i + k]);
                let (br, bi) = (re[i + k + len / 2], im[i + k + len / 2]);
                let (tr, ti) = (br * wr - bi * wi, br * wi + bi * wr);
                re[i + k] = ar + tr;
                im[i + k] = ai + ti;
                re[i + k + len / 2] = ar - tr;
                im[i + k + len / 2] = ai - ti;
            }
        }
        len <<= 1;
    }
}

/// Mono delay line with fractional (cubic Hermite) reads — used for the
/// speed-of-sound propagation delay, which also produces the Doppler shift.
#[derive(Clone, Debug)]
pub struct DelayLine {
    buf: Vec<f32>,
    w: usize,
    mask: usize,
}

impl DelayLine {
    pub fn new(max_samples: usize) -> DelayLine {
        let n = (max_samples + 4).next_power_of_two();
        DelayLine { buf: vec![0.0; n], w: 0, mask: n - 1 }
    }
    pub fn max_delay(&self) -> f32 {
        (self.buf.len() - 4) as f32
    }
    #[inline]
    pub fn push(&mut self, x: f32) {
        self.w = (self.w + 1) & self.mask;
        self.buf[self.w] = x;
    }
    /// Read `d` samples back (d >= 1 recommended).
    #[inline]
    pub fn read(&self, d: f32) -> f32 {
        let d = d.clamp(1.0, self.max_delay());
        let di = d as usize;
        let f = d - di as f32;
        let m = self.mask;
        let i0 = self.w.wrapping_sub(di) & m;
        let xm1 = self.buf[(i0 + 1) & m];
        let x0 = self.buf[i0];
        let x1 = self.buf[i0.wrapping_sub(1) & m];
        let x2 = self.buf[i0.wrapping_sub(2) & m];
        // Hermite between x0 (f=0) and x1 (f=1).
        let c0 = x0;
        let c1 = 0.5 * (x1 - xm1);
        let c2 = xm1 - 2.5 * x0 + 2.0 * x1 - 0.5 * x2;
        let c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);
        ((c3 * f + c2) * f + c1) * f + c0
    }
}

/// Simple feedback delay network reverb (4 lines, Householder mixing) for a
/// sense of open-air space on the exterior cameras.
#[derive(Clone, Debug)]
pub struct Fdn {
    lines: [Vec<f32>; 4],
    idx: [usize; 4],
    damp: [OnePole; 4],
    pub fb: f32,
}

impl Fdn {
    pub fn new(sr: f32) -> Fdn {
        let lens_ms = [37.1f32, 53.7, 71.3, 89.9];
        let lines = lens_ms.map(|ms| vec![0.0f32; ((ms * 0.001 * sr) as usize).max(8)]);
        Fdn { lines, idx: [0; 4], damp: [OnePole::new(3500.0, sr); 4], fb: 0.72 }
    }
    /// Mono in, stereo out.
    #[inline]
    pub fn tick(&mut self, x: f32) -> (f32, f32) {
        let mut o = [0.0f32; 4];
        for i in 0..4 {
            o[i] = self.lines[i][self.idx[i]];
        }
        let s = (o[0] + o[1] + o[2] + o[3]) * 0.5;
        for i in 0..4 {
            let v = self.damp[i].lp((o[i] - s) * self.fb) + x;
            self.lines[i][self.idx[i]] = v;
            self.idx[i] += 1;
            if self.idx[i] >= self.lines[i].len() {
                self.idx[i] = 0;
            }
        }
        (o[0] + o[2] * 0.7 - o[3] * 0.3, o[1] + o[3] * 0.7 - o[2] * 0.3)
    }
}

/// Look-ahead peak limiter (stereo-linked) with a final safety clamp.
#[derive(Clone, Debug)]
pub struct Limiter {
    ceiling: f32,
    gain: f32,
    look: usize,
    buf_l: Vec<f32>,
    buf_r: Vec<f32>,
    peak_q: Vec<f32>,
    pos: usize,
    release: f32,
    attack: f32,
    /// Samples the safety knee had to touch (should stay ~0).
    pub clamped: u64,
    /// Smallest gain applied so far (for reports).
    pub min_gain: f32,
}

impl Limiter {
    pub fn new(sr: f32, ceiling_db: f32) -> Limiter {
        let look = ((sr * 0.003) as usize).max(8);
        Limiter {
            ceiling: 10f32.powf(ceiling_db / 20.0),
            gain: 1.0,
            look,
            buf_l: vec![0.0; look],
            buf_r: vec![0.0; look],
            peak_q: vec![0.0; look],
            pos: 0,
            release: 1.0 - (-1.0 / (0.25 * sr)).exp(),
            attack: 1.0 - (-1.0 / (look as f32 * 0.2)).exp(),
            clamped: 0,
            min_gain: 1.0,
        }
    }
    #[inline]
    pub fn tick(&mut self, l: f32, r: f32) -> (f32, f32) {
        let l = if l.is_finite() { l } else { 0.0 };
        let r = if r.is_finite() { r } else { 0.0 };
        let pk = l.abs().max(r.abs());
        self.peak_q[self.pos] = pk;
        // Max over the look-ahead window (~3 ms).
        let m = self.peak_q.iter().fold(0.0f32, |a, &b| a.max(b));
        let target = (self.ceiling * 0.97 / m.max(1e-9)).min(1.0);
        if target < self.gain {
            self.gain += (target - self.gain) * self.attack;
        } else {
            self.gain += (target - self.gain) * self.release;
        }
        self.min_gain = self.min_gain.min(self.gain);
        let dl = self.buf_l[self.pos];
        let dr = self.buf_r[self.pos];
        self.buf_l[self.pos] = l;
        self.buf_r[self.pos] = r;
        self.pos = (self.pos + 1) % self.look;
        let mut ol = dl * self.gain;
        let mut or = dr * self.gain;
        // Safety knee for anything the look-ahead missed (the limiter aims at
        // 97 %): above 98 % of the ceiling, saturate smoothly so the output
        // can never exceed it.
        let c = self.ceiling;
        let knee = |x: f32| {
            let a = x.abs();
            if a <= 0.98 * c { x } else { x.signum() * (0.98 * c + 0.02 * c * ((a - 0.98 * c) / (0.02 * c)).tanh()) }
        };
        if ol.abs() > 0.98 * c || or.abs() > 0.98 * c {
            self.clamped += 1;
            ol = knee(ol);
            or = knee(or);
        }
        (ol, or)
    }
}

/// Equal-power stereo pan, `p` in -1 (left) .. 1 (right).
#[inline]
pub fn pan(p: f32) -> (f32, f32) {
    let a = (p.clamp(-1.0, 1.0) + 1.0) * 0.25 * PI;
    (a.cos(), a.sin())
}

#[inline]
pub fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Soft saturation (unity slope at 0).
#[inline]
pub fn sat(x: f32) -> f32 {
    x / (1.0 + x.abs())
}
