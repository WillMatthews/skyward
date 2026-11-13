//! Recorded samples: engine loops, start / shutdown, airframe, impacts —
//! FlightGear sound sets (see `assets/audio/CREDITS.md`), embedded as Ogg
//! Vorbis (`assets/audio/samples`, built by `assets/audio/tools/build_samples.py`).
//!
//! * Decoding happens off the audio thread: [`prepare`] (game thread: a
//!   worker thread natively, inline on wasm32) or [`prepare_blocking`]
//!   (offline renderer, tests). Each sample is decoded once into a global
//!   slot; the audio thread only ever does a lock-free [`pcm`] lookup and
//!   simply hears nothing (or the procedural fallback) until it is ready.
//! * Loops are baked seamless at build time (correlation-matched end point,
//!   tail cross-faded into the head); the decoded buffer is padded with
//!   wrapped samples so the interpolator never needs a modulo.
//! * [`Voice`] plays a sample at any rate with a windowed-sinc interpolator
//!   (12 taps, 256 phases): the build script stores every sample at a rate
//!   low enough that the highest pitch it is played at stays below Nyquist,
//!   so no run-time anti-aliasing is needed.

use super::dsp::Ramp;
use std::sync::OnceLock;

macro_rules! embed {
    ($($n:literal),* $(,)?) => {
        /// Sample names, index = sample id.
        pub const NAMES: &[&str] = &[$($n),*];
        /// The Vorbis files (natively compiled in; the browser build
        /// downloads them when an aircraft needs them, see [`prepare`]).
        #[cfg(not(target_arch = "wasm32"))]
        static FILES: &[&[u8]] = &[$(include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/audio/samples/", $n, ".ogg"))),*];
    };
}

embed!(
    "c172_idle",
    "c172_run",
    "c172_crank",
    "c172_start",
    "c172_cut",
    "c172_stall",
    "tyre_roll",
    "tyre_gravel",
    "flap_motor_light",
    "merlin_1",
    "merlin_2",
    "merlin_3",
    "merlin_start",
    "merlin_stop",
    "f16_x_idle_f",
    "f16_x_idle_a",
    "f16_x_mil_f",
    "f16_x_mil_a",
    "f16_x_ab",
    "f16_x_abx",
    "f16_x_whine",
    "f16_x_roar",
    "f16_i_idle_lo",
    "f16_i_idle_hi",
    "f16_i_88_lo",
    "f16_i_88_hi",
    "f16_i_100_lo",
    "f16_i_100_hi",
    "f16_i_ab",
    "atr_out",
    "atr_int",
    "atr_rev",
    "atr_prop",
    "atr_start",
    "atr_stop",
    "cfm_x_idle",
    "cfm_x_idle_r",
    "cfm_x_idle_c",
    "cfm_x_idle_d",
    "cfm_x_spool",
    "cfm_x_lowspool",
    "cfm_x_spool_c",
    "cfm_x_spool_cr",
    "cfm_x_rumble",
    "cfm_x_rumble_d",
    "cfm_x_buzz",
    "cfm_x_buzz_d",
    "cfm_x_rev",
    "cfm_x_start",
    "cfm_x_stop",
    "cfm_c_idle",
    "cfm_c_idle2",
    "cfm_c_spool",
    "cfm_c_hum",
    "cfm_c_comb",
    "cfm_c_comb2",
    "cfm_c_buzz",
    "cfm_c_rev",
    "cfm_c_start",
    "cfm_c_stop",
    "v25_x_idle",
    "v25_x_idle_r",
    "v25_x_spool",
    "v25_x_lowspool",
    "v25_x_buzz",
    "v25_x_buzz_d",
    "v25_x_start",
    "v25_c_idle",
    "v25_c_lowspool",
    "v25_c_comb",
    "v25_c_buzz",
    "v25_c_rev",
    "v25_c_start",
    "heavy_roll",
    "heavy_roll_int",
    "heavy_touch",
    "fighter_touch",
    "gear_heavy",
    "gear_fighter",
    "flap_motor_heavy",
    "wind_airliner",
    "wind_light",
    "glider_wind",
    "glider_brake",
    "glider_roll",
    "thunder_1",
    "thunder_2",
    "thunder_3",
    "crash_c172",
    "crash_heavy",
    "crash_light",
    "crash_scrape",
    "crash_f16",
    "crash_explode",
    "crash_water",
    "vul_x_turb",
    "vul_x_jet",
    "vul_x_blast",
    "vul_x_howl",
    "vul_i_eng",
    "vul_start",
    "vul_stop",
);

const N: usize = NAMES.len();
/// Build index: name, rate, frames, loop, extra attenuation (dB).
const INDEX: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/assets/audio/samples/index.tsv"
));

/// Guard samples on each side of the data (wrapped for loops, zero for one-shots).
pub const PAD: usize = 8;

/// A decoded sample (mono).
#[derive(Debug)]
pub struct Pcm {
    pub rate: f32,
    /// PAD + len + PAD samples, as decoded (i16: half the memory of f32).
    pub data: Vec<i16>,
    /// Sample value scale (1/32768 and the build attenuation undone).
    pub scale: f32,
    pub len: usize,
    pub looped: bool,
}

#[allow(clippy::declare_interior_mutable_const)]
const EMPTY: OnceLock<Pcm> = OnceLock::new();
static PCM: [OnceLock<Pcm>; N] = [EMPTY; N];

/// Sample id by name (setup time only; panics on a typo in debug builds).
pub fn id(name: &str) -> usize {
    let i = NAMES.iter().position(|n| *n == name);
    debug_assert!(i.is_some(), "unknown sample {name}");
    i.unwrap_or(0)
}

/// The decoded sample, if [`prepare`] has finished it. Lock-free.
#[inline]
pub fn pcm(i: usize) -> Option<&'static Pcm> {
    PCM.get(i).and_then(|c| c.get())
}

fn index_row(name: &str) -> Option<(f32, usize, bool, f32)> {
    INDEX.lines().find_map(|l| {
        let mut it = l.split('\t');
        if it.next()? != name {
            return None;
        }
        let rate = it.next()?.parse().ok()?;
        let frames = it.next()?.parse().ok()?;
        let looped = it.next()? == "1";
        let att: f32 = it.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
        Some((rate, frames, looped, att))
    })
}

#[cfg(not(target_arch = "wasm32"))]
fn decode(i: usize) -> Pcm {
    decode_bytes(i, FILES[i])
}

fn decode_bytes(i: usize, file: &[u8]) -> Pcm {
    let name = NAMES[i];
    let (rate_i, frames, looped, att) = index_row(name).unwrap_or((22050.0, 0, false, 0.0));
    let mut raw: Vec<i16> = Vec::with_capacity(frames + 1024);
    let mut rate = rate_i;
    // Undo the build script's extra attenuation (headroom for the i16
    // decode); the common -1.5 dB is part of the calibration.
    let scale = 10f32.powf(-att / 20.0) / 32768.0;
    match lewton::inside_ogg::OggStreamReader::new(std::io::Cursor::new(file)) {
        Ok(mut r) => {
            rate = r.ident_hdr.audio_sample_rate as f32;
            let ch = r.ident_hdr.audio_channels.max(1) as usize;
            while let Ok(Some(p)) = r.read_dec_packet_itl() {
                raw.extend(p.iter().step_by(ch).copied());
            }
        }
        Err(e) => eprintln!("audio: sample {name}: {e}"),
    }
    // The build index has the exact length (Vorbis may pad the last page).
    let len = if frames > 0 {
        frames.min(raw.len()).max(1)
    } else {
        raw.len().max(1)
    };
    let short = raw.len() < frames;
    raw.resize(len, 0);
    if looped && short && len > 64 {
        // Shorter than expected: hide the seam with a short cross-fade.
        let x = (len / 8).min(256);
        for k in 0..x {
            let t = (k as f32 + 0.5) / x as f32;
            raw[k] = (raw[k] as f32 * t + raw[len - x + k] as f32 * (1.0 - t)) as i16;
        }
    }
    let mut data = Vec::with_capacity(len + 2 * PAD);
    for k in 0..PAD {
        data.push(if looped {
            raw[(len * PAD - PAD + k) % len]
        } else {
            0
        });
    }
    data.extend_from_slice(&raw);
    for k in 0..PAD {
        data.push(if looped { raw[k % len] } else { 0 });
    }
    Pcm {
        rate,
        data,
        scale,
        len,
        looped,
    }
}

/// Decode `names` now (blocking). Already-decoded samples are skipped.
/// (Browser: only the samples already downloaded, see [`prepare`].)
pub fn prepare_blocking(names: &[&'static str]) {
    let _ = kernel();
    for n in names {
        let i = id(n);
        #[cfg(not(target_arch = "wasm32"))]
        PCM[i].get_or_init(|| decode(i));
        #[cfg(target_arch = "wasm32")]
        if let Ok(b) = crate::assets::read(&web::path(i)) {
            PCM[i].get_or_init(|| decode_bytes(i, &b));
        }
    }
}

/// Decode `names` in the background: natively on a thread; in the browser
/// each file is downloaded (`assets/audio/samples/`, not compiled in: it
/// keeps the first download small) and decoded in a tile worker.
pub fn prepare(names: &'static [&'static str]) {
    if names.iter().all(|n| pcm(id(n)).is_some()) {
        return;
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = kernel();
        for n in names {
            web::start(id(n));
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        if std::thread::Builder::new()
            .name("audio-decode".into())
            .spawn(move || prepare_blocking(names))
            .is_ok()
        {
            return;
        }
        prepare_blocking(names);
    }
}

/// Worker job "sample" (tiles_web.rs): sample index (u32) + its Vorbis file
/// → the decoded sample ([`web::decode_wire`]).
#[allow(dead_code)]
pub fn decode_job(data: &[u8]) -> Option<Vec<u8>> {
    let i = u32::from_le_bytes(data.get(..4)?.try_into().ok()?) as usize;
    if i >= N {
        return None;
    }
    let p = decode_bytes(i, &data[4..]);
    let mut out = Vec::with_capacity(17 + p.data.len() * 2);
    out.extend_from_slice(&p.rate.to_le_bytes());
    out.extend_from_slice(&p.scale.to_le_bytes());
    out.extend_from_slice(&(p.len as u64).to_le_bytes());
    out.push(p.looped as u8);
    out.extend_from_slice(bytemuck::cast_slice(&p.data));
    Some(out)
}

#[cfg(target_arch = "wasm32")]
mod web {
    use super::*;
    use crate::platform::web as w;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[allow(clippy::declare_interior_mutable_const)]
    const IDLE: AtomicBool = AtomicBool::new(false);
    static STARTED: [AtomicBool; N] = [IDLE; N];

    pub fn path(i: usize) -> String {
        format!("audio/samples/{}.ogg", NAMES[i])
    }

    /// The worker's answer → a sample.
    fn decode_wire(b: &[u8]) -> Option<Pcm> {
        let rate = f32::from_le_bytes(b.get(0..4)?.try_into().ok()?);
        let scale = f32::from_le_bytes(b.get(4..8)?.try_into().ok()?);
        let len = u64::from_le_bytes(b.get(8..16)?.try_into().ok()?) as usize;
        let looped = *b.get(16)? != 0;
        let raw = b.get(17..)?;
        let mut data = vec![0i16; raw.len() / 2];
        let n = data.len() * 2;
        bytemuck::cast_slice_mut::<i16, u8>(&mut data).copy_from_slice(&raw[..n]);
        (data.len() == len + 2 * PAD).then_some(Pcm { rate, data, scale, len, looped })
    }

    /// Download sample `i` and decode it (a worker, else the page between
    /// frames). Until then it is silent.
    pub fn start(i: usize) {
        if pcm(i).is_some() || STARTED[i].swap(true, Ordering::Relaxed) {
            return;
        }
        w::spawn_local(async move {
            let bytes = match w::get(&format!("assets/{}", path(i)), web_time::Duration::from_secs(60)).await {
                Ok(b) => b,
                Err(e) => {
                    w::log(&format!("audio: sample {}: {e}", NAMES[i]));
                    STARTED[i].store(false, Ordering::Relaxed);
                    return;
                }
            };
            let mut job = (i as u32).to_le_bytes().to_vec();
            job.extend_from_slice(&bytes);
            if let Some(f) = w::offload_begin("sample", job) {
                if let Some(p) = f.await.as_deref().and_then(decode_wire) {
                    let _ = PCM[i].set(p);
                    return;
                }
            }
            w::defer(Box::new(move || {
                PCM[i].get_or_init(|| decode_bytes(i, &bytes));
            }));
        });
    }
}

/// True once every sample of `names` is decoded.
pub fn ready(names: &[&str]) -> bool {
    names.iter().all(|n| pcm(id(n)).is_some())
}

// ------------------------------------------------------------------ sampler

const TAPS: usize = 12;
const PHASES: usize = 256;

/// Kaiser-windowed sinc, `PHASES + 1` rows of `TAPS` (row p = fraction p/PHASES).
fn kernel() -> &'static [f32] {
    static K: OnceLock<Vec<f32>> = OnceLock::new();
    K.get_or_init(|| {
        let beta = 7.0f64;
        let i0 = |x: f64| {
            let (mut s, mut t) = (1.0f64, 1.0f64);
            for k in 1..30 {
                t *= (x / 2.0) / k as f64;
                s += t * t;
            }
            s
        };
        let cutoff = 0.94f64;
        let half = TAPS as f64 / 2.0;
        let mut k = Vec::with_capacity((PHASES + 1) * TAPS);
        for p in 0..=PHASES {
            let f = p as f64 / PHASES as f64;
            let mut row = [0.0f64; TAPS];
            for (t, r) in row.iter_mut().enumerate() {
                // Tap t sits at offset (t - (TAPS/2 - 1)) from the base sample.
                let x = t as f64 - (half - 1.0) - f;
                let s = if x.abs() < 1e-9 {
                    cutoff
                } else {
                    (std::f64::consts::PI * cutoff * x).sin() / (std::f64::consts::PI * x)
                };
                let w = (x / half).clamp(-1.0, 1.0);
                *r = s * i0(beta * (1.0 - w * w).max(0.0).sqrt()) / i0(beta);
            }
            let sum: f64 = row.iter().sum();
            k.extend(row.iter().map(|v| (v / sum) as f32));
        }
        k
    })
}

/// One playing sample.
#[derive(Clone, Copy, Debug)]
pub struct Voice {
    /// Position in source frames.
    pub pos: f64,
    /// Source frames per output sample (block-ramped).
    pub inc: Ramp,
    pub gain: Ramp,
    pub done: bool,
}

impl Voice {
    pub fn new(_sample: usize, start: f64) -> Voice {
        Voice {
            pos: start,
            inc: Ramp::new(0.0),
            gain: Ramp::new(0.0),
            done: false,
        }
    }
    /// Next output sample.
    #[inline]
    pub fn tick(&mut self, p: &Pcm, k: &[f32]) -> f32 {
        let g = self.gain.tick();
        let inc = self.inc.tick() as f64;
        if self.done {
            return 0.0;
        }
        let i = self.pos.floor();
        let f = self.pos - i;
        let base = i as usize + PAD - (TAPS / 2 - 1);
        let y = if g.abs() > 1e-6 {
            let ph = f * PHASES as f64;
            let pi = (ph as usize).min(PHASES - 1);
            let pf = (ph - pi as f64) as f32;
            let r0 = &k[pi * TAPS..pi * TAPS + TAPS];
            let r1 = &k[(pi + 1) * TAPS..(pi + 1) * TAPS + TAPS];
            let d = &p.data[base..base + TAPS];
            let (mut a, mut b) = (0.0f32, 0.0f32);
            for t in 0..TAPS {
                let x = d[t] as f32;
                a += x * r0[t];
                b += x * r1[t];
            }
            (a + (b - a) * pf) * g * p.scale
        } else {
            0.0
        };
        self.pos += inc;
        let len = p.len as f64;
        if self.pos >= len {
            if p.looped {
                self.pos -= len * (self.pos / len).floor();
            } else {
                self.done = true;
                self.pos = len - 1.0;
            }
        }
        y
    }
}

/// The interpolation kernel (built by `prepare*`; built here if needed).
#[inline]
pub fn kern() -> &'static [f32] {
    kernel()
}

/// A continuously controlled loop of one sample (wind, rolling, motors):
/// attaches itself once the sample is decoded, starts at `start` (0..1)
/// into the loop, and stays silent until then.
#[derive(Clone, Copy, Debug)]
pub struct Looper {
    name: &'static str,
    start: f32,
    v: Option<(Voice, &'static Pcm)>,
}

impl Looper {
    pub const fn new(name: &'static str, start: f32) -> Looper {
        Looper {
            name,
            start,
            v: None,
        }
    }
    /// Switch to another sample (keeps playing position if it is the same).
    pub fn set_sample(&mut self, name: &'static str) {
        if self.name != name {
            self.name = name;
            self.v = None;
        }
    }
    /// Once per block: target gain and pitch.
    pub fn control(&mut self, gain: f32, pitch: f32, sr: f32, n: u32) {
        if self.v.is_none() {
            if gain <= 1e-5 {
                return;
            }
            let id = id(self.name);
            if let Some(p) = pcm(id) {
                let v = Voice::new(id, (self.start.fract() * p.len as f32) as f64);
                self.v = Some((v, p));
            } else {
                return;
            }
        }
        if let Some((v, p)) = &mut self.v {
            v.gain.to(gain.max(0.0), n);
            v.inc.to(p.rate / sr * pitch.max(0.05), n);
        }
    }
    #[inline]
    pub fn tick(&mut self) -> f32 {
        match &mut self.v {
            Some((v, p)) => {
                if v.gain.v <= 1e-6 && v.gain.tick() <= 1e-6 {
                    // Silent: keep the phase moving cheaply.
                    v.pos += v.inc.tick() as f64;
                    if v.pos >= p.len as f64 {
                        v.pos -= p.len as f64 * (v.pos / p.len as f64).floor();
                    }
                    return 0.0;
                }
                v.tick(p, kern())
            }
            None => 0.0,
        }
    }
}
