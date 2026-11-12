//! Pre-rendered speech clips (embedded WAVs, decoded once) and a sampler
//! voice that plays them through a band-limited "cockpit speaker".

use super::dsp::{OnePole, Svf, sat};
use super::params::Clip;
use std::sync::OnceLock;

macro_rules! clip {
    ($f:literal) => {
        include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/audio/voice/", $f, ".wav")).as_slice()
    };
}

/// Embedded WAV bytes, in `Clip::ALL` order.
fn wav_bytes(c: Clip) -> &'static [u8] {
    match c {
        Clip::Ft2500 => clip!("2500"),
        Clip::Ft1000 => clip!("1000"),
        Clip::Ft500 => clip!("500"),
        Clip::Ft400 => clip!("400"),
        Clip::Ft300 => clip!("300"),
        Clip::Ft200 => clip!("200"),
        Clip::Ft100 => clip!("100"),
        Clip::Ft50 => clip!("50"),
        Clip::Ft40 => clip!("40"),
        Clip::Ft30 => clip!("30"),
        Clip::Ft20 => clip!("20"),
        Clip::Ft10 => clip!("10"),
        Clip::Retard => clip!("retard"),
        Clip::Minimums => clip!("minimums"),
        Clip::ApproachingMinimums => clip!("app_minimums"),
        Clip::HundredAbove => clip!("hundred_above"),
        Clip::Stall => clip!("stall"),
        Clip::PullUp => clip!("pull_up"),
        Clip::SinkRate => clip!("sink_rate"),
        Clip::TooLowGear => clip!("too_low_gear"),
        Clip::TooLowFlaps => clip!("too_low_flaps"),
        Clip::Terrain => clip!("terrain"),
        Clip::BankAngle => clip!("bank_angle"),
        Clip::V1 => clip!("v1"),
        Clip::Overspeed => clip!("overspeed"),
        Clip::EngineFailure => clip!("engine_fail"),
    }
}

pub struct Pcm {
    pub rate: f32,
    pub data: Vec<f32>,
}

fn decode(bytes: &[u8]) -> Pcm {
    match hound::WavReader::new(std::io::Cursor::new(bytes)) {
        Ok(r) => {
            let spec = r.spec();
            let ch = spec.channels.max(1) as usize;
            let data: Vec<f32> = match spec.sample_format {
                hound::SampleFormat::Int => {
                    let scale = 1.0 / (1i64 << (spec.bits_per_sample.max(1) - 1)) as f32;
                    r.into_samples::<i32>().step_by(ch).map(|s| s.unwrap_or(0) as f32 * scale).collect()
                }
                hound::SampleFormat::Float => r.into_samples::<f32>().step_by(ch).map(|s| s.unwrap_or(0.0)).collect(),
            };
            Pcm { rate: spec.sample_rate as f32, data }
        }
        Err(_) => Pcm { rate: 16000.0, data: Vec::new() },
    }
}

/// All clips, decoded on first use.
pub fn bank() -> &'static [Pcm] {
    static BANK: OnceLock<Vec<Pcm>> = OnceLock::new();
    BANK.get_or_init(|| Clip::ALL.iter().map(|c| decode(wav_bytes(*c))).collect())
}

/// Clip length in seconds (for the speech queue on the game thread).
pub fn clip_seconds(c: Clip) -> f32 {
    let p = &bank()[c.index()];
    p.data.len() as f32 / p.rate.max(1.0)
}

/// Plays one clip at a time (a newer clip cuts the old one with a short fade).
pub struct Speaker {
    clip: Option<usize>,
    pos: f64,
    fade: f32,
    next: Option<usize>,
    hp: Svf,
    lp: Svf,
    body: Svf,
    dc: OnePole,
    sr: f32,
    /// 0..1 envelope of "someone is talking" for ducking.
    pub active: f32,
}

impl Speaker {
    pub fn new(sr: f32) -> Speaker {
        Speaker {
            clip: None,
            pos: 0.0,
            fade: 1.0,
            next: None,
            hp: Svf::new(320.0, 0.7, sr),
            lp: Svf::new(4200.0, 0.8, sr),
            body: Svf::new(1800.0, 1.4, sr),
            dc: OnePole::new(20.0, sr),
            sr,
            active: 0.0,
        }
    }
    pub fn play(&mut self, c: Clip) {
        if self.clip.is_some() {
            self.next = Some(c.index());
        } else {
            self.clip = Some(c.index());
            self.pos = 0.0;
            self.fade = 1.0;
        }
    }
    #[inline]
    pub fn tick(&mut self) -> f32 {
        let bank = bank();
        let mut out = 0.0;
        if let Some(i) = self.clip {
            let p = &bank[i];
            let ip = self.pos as usize;
            if ip + 1 >= p.data.len() {
                self.clip = self.next.take();
                self.pos = 0.0;
                self.fade = 1.0;
            } else {
                let f = (self.pos - ip as f64) as f32;
                out = p.data[ip] + (p.data[ip + 1] - p.data[ip]) * f;
                self.pos += p.rate as f64 / self.sr as f64;
                if self.next.is_some() {
                    self.fade -= 1.0 / (0.02 * self.sr);
                    if self.fade <= 0.0 {
                        self.clip = self.next.take();
                        self.pos = 0.0;
                        self.fade = 1.0;
                    }
                }
                out *= self.fade.max(0.0);
            }
        }
        let target = if self.clip.is_some() { 1.0 } else { 0.0 };
        let rate = if target > self.active { 0.004 } else { 0.00008 };
        self.active += (target - self.active) * rate * 48_000.0 / self.sr;
        // Cockpit loudspeaker: band-limited, a presence bump, a little drive.
        let x = self.hp.hp(out);
        let x = self.lp.lp(x);
        let x = x + 0.35 * self.body.bp(x);
        let y = sat(x * 1.6) * 0.8;
        y - self.dc.lp(y)
    }
}
