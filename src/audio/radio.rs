//! The VHF radio in the headset: ATC / ATIS / pilot transmissions assembled
//! from the pre-rendered phrase vocabulary (`assets/audio/atc/<voice>.ogg`,
//! one packed stream per voice + an index; see `src/atc/phrase.rs`) and
//! played through an AM airband chain: squelch break, carrier hiss by signal
//! strength, 300–3,000 Hz voice band, compression and a little drive, the
//! squelch tail when the carrier drops. Two receivers (COM 1 / COM 2) can be
//! heard at once. The pilot's own transmissions come back as side-tone
//! (band-limited, no squelch noise).
//!
//! The game thread [`submit`]s a [`Tx`] and raises `Event::Radio(id)`; the
//! audio thread takes it with a `try_lock` (never blocking) and plays it.

use super::dsp::{OnePole, Rng, Svf, sat};
use crate::atc::phrase::Tok;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

/// Voices of the pack (Kokoro voice names), index = `Voice as usize`.
pub const VOICE_FILES: [&str; 6] = ["am_liam", "bf_emma", "am_onyx", "bm_fable", "af_river", "af_aoede"];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Voice {
    /// The player's read-backs and requests.
    Pilot = 0,
    Emma = 1,
    Onyx = 2,
    Fable = 3,
    River = 4,
    Aoede = 5,
}

impl Voice {
    pub const CONTROLLERS: [Voice; 5] = [Voice::Emma, Voice::Onyx, Voice::Fable, Voice::River, Voice::Aoede];
}

macro_rules! pack {
    ($($v:literal),*) => {
        // (The browser build downloads the voices instead: `prepare`.)
        #[cfg(not(target_arch = "wasm32"))]
        const OGG: [&[u8]; 6] = [$(include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/audio/atc/", $v, ".ogg"))),*];
        const IDX: [&str; 6] = [$(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/audio/atc/", $v, ".idx"))),*];
    };
}
pack!("am_liam", "bf_emma", "am_onyx", "bm_fable", "af_river", "af_aoede");

/// One voice: decoded PCM and the clip of each vocabulary entry.
pub struct Pack {
    pub rate: f32,
    pub pcm: Vec<f32>,
    /// Vocabulary index → (start, len) in samples.
    pub clips: HashMap<u16, (u32, u32)>,
}

fn parse_index(i: usize) -> (f32, usize, HashMap<u16, (u32, u32)>) {
    let v = crate::atc::phrase::vocab();
    let mut lines = IDX[i].lines();
    let head: Vec<&str> = lines.next().unwrap_or("12000\t0").split('\t').collect();
    let rate = head.first().and_then(|s| s.parse().ok()).unwrap_or(12_000.0);
    let total = head.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let mut clips = HashMap::new();
    for l in lines {
        let f: Vec<&str> = l.split('\t').collect();
        if f.len() < 3 {
            continue;
        }
        if let (Some(ix), Ok(a), Ok(n)) = (v.get(f[0]), f[1].parse::<u32>(), f[2].parse::<u32>()) {
            clips.insert(ix, (a, n));
        }
    }
    (rate, total, clips)
}

#[cfg(not(target_arch = "wasm32"))]
fn decode(i: usize) -> Pack {
    decode_bytes(i, OGG[i])
}

fn decode_bytes(i: usize, ogg: &[u8]) -> Pack {
    let (rate_i, total, clips) = parse_index(i);
    let mut pcm: Vec<f32> = Vec::with_capacity(total + 4096);
    let mut rate = rate_i;
    match lewton::inside_ogg::OggStreamReader::new(std::io::Cursor::new(ogg)) {
        Ok(mut r) => {
            rate = r.ident_hdr.audio_sample_rate as f32;
            let ch = r.ident_hdr.audio_channels.max(1) as usize;
            while let Ok(Some(p)) = r.read_dec_packet_itl() {
                pcm.extend(p.iter().step_by(ch).map(|s| *s as f32 / 32768.0));
            }
        }
        Err(e) => eprintln!("audio: radio voice {}: {e}", VOICE_FILES[i]),
    }
    if total > 0 {
        pcm.resize(total, 0.0);
    }
    Pack { rate, pcm, clips }
}

static PACKS: [OnceLock<Pack>; 6] = [const { OnceLock::new() }; 6];

/// A voice once decoded (None while decoding).
pub fn pack(v: Voice) -> Option<&'static Pack> {
    PACKS[v as usize].get()
}

/// Decode every voice now (offline renders, tests).
#[cfg(not(target_arch = "wasm32"))]
pub fn prepare_blocking() {
    for (i, p) in PACKS.iter().enumerate() {
        p.get_or_init(|| decode(i));
    }
}

/// Worker job "radio" (tiles_web.rs): voice index (u32) + its Vorbis file →
/// rate (f32) + PCM (f32s).
#[allow(dead_code)]
pub fn decode_job(data: &[u8]) -> Option<Vec<u8>> {
    let i = u32::from_le_bytes(data.get(..4)?.try_into().ok()?) as usize;
    if i >= PACKS.len() {
        return None;
    }
    let p = decode_bytes(i, &data[4..]);
    let mut out = p.rate.to_le_bytes().to_vec();
    out.extend_from_slice(bytemuck::cast_slice(&p.pcm));
    Some(out)
}

/// Decode the voices off the audio thread: a worker natively; in the
/// browser each voice is downloaded (`assets/audio/atc/`, not compiled in)
/// and decoded in a tile worker (else on the page between frames).
pub fn prepare() {
    #[cfg(target_arch = "wasm32")]
    {
        use crate::platform::web;
        static STARTED: OnceLock<()> = OnceLock::new();
        if STARTED.set(()).is_err() {
            return;
        }
        for i in 0..PACKS.len() {
            web::spawn_local(async move {
                let url = format!("assets/audio/atc/{}.ogg", VOICE_FILES[i]);
                let bytes = match web::get(&url, web_time::Duration::from_secs(90)).await {
                    Ok(b) => b,
                    Err(e) => return web::log(&format!("audio: radio voice {}: {e}", VOICE_FILES[i])),
                };
                let mut job = (i as u32).to_le_bytes().to_vec();
                job.extend_from_slice(&bytes);
                if let Some(f) = web::offload_begin("radio", job) {
                    if let Some(b) = f.await.filter(|b| b.len() >= 4) {
                        let rate = f32::from_le_bytes(b[..4].try_into().unwrap());
                        let mut pcm = vec![0f32; (b.len() - 4) / 4];
                        let n = pcm.len() * 4;
                        bytemuck::cast_slice_mut::<f32, u8>(&mut pcm).copy_from_slice(&b[4..4 + n]);
                        let (_, _, clips) = parse_index(i);
                        let _ = PACKS[i].set(Pack { rate, pcm, clips });
                        return;
                    }
                }
                web::defer(Box::new(move || {
                    PACKS[i].get_or_init(|| decode_bytes(i, &bytes));
                }));
            });
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        static STARTED: OnceLock<()> = OnceLock::new();
        if STARTED.set(()).is_err() {
            return;
        }
        if std::thread::Builder::new().name("radio-decode".into()).spawn(prepare_blocking).is_err() {
            prepare_blocking();
        }
    }
}

/// Length of one vocabulary entry for a voice (s); None if not rendered for it.
pub fn clip_seconds(v: Voice, word: u16) -> Option<f32> {
    let (_, n) = parse_cached(v).get(&word).copied()?;
    Some(n as f32 / 12_000.0)
}

/// The index alone (cheap, available before the PCM is decoded).
fn parse_cached(v: Voice) -> &'static HashMap<u16, (u32, u32)> {
    static IX: [OnceLock<HashMap<u16, (u32, u32)>>; 6] = [const { OnceLock::new() }; 6];
    IX[v as usize].get_or_init(|| parse_index(v as usize).2)
}

/// Does the voice have this vocabulary entry?
pub fn has(v: Voice, word: u16) -> bool {
    parse_cached(v).contains_key(&word)
}

/// Gap between consecutive words (s): the clips carry their own short lead-in / tail.
pub const WORD_GAP: f32 = 0.012;
/// Squelch break before / tail after a controller transmission (s).
pub const SQUELCH_OPEN: f32 = 0.09;
pub const SQUELCH_TAIL: f32 = 0.2;

/// How long a transmission lasts (s), squelch included.
pub fn duration(v: Voice, toks: &[Tok]) -> f32 {
    let mut t = SQUELCH_OPEN + SQUELCH_TAIL;
    for k in toks {
        t += match *k {
            Tok::Word(w) => clip_seconds(v, w).unwrap_or(0.0) + WORD_GAP,
            Tok::Gap(g) => g,
        };
    }
    t
}

/// One transmission on a receiver.
#[derive(Clone, Debug, PartialEq)]
pub struct Tx {
    pub voice: Voice,
    pub toks: Vec<Tok>,
    /// Receiver 0 = COM 1, 1 = COM 2.
    pub ch: u8,
    /// The pilot's own transmission (side-tone).
    pub own: bool,
    /// Signal strength 0..1 (hiss and fading when weak).
    pub strength: f32,
    /// Volume of that receiver 0..1.
    pub volume: f32,
}

static PENDING: Mutex<Vec<(u32, Arc<Tx>)>> = Mutex::new(Vec::new());
static NEXT: Mutex<u32> = Mutex::new(1);

/// Hand a transmission to the audio thread; raise `Event::Radio(id)` with the id.
pub fn submit(tx: Tx) -> u32 {
    let id = {
        let mut n = NEXT.lock().unwrap_or_else(|e| e.into_inner());
        *n = n.wrapping_add(1).max(1);
        *n
    };
    let mut p = PENDING.lock().unwrap_or_else(|e| e.into_inner());
    p.push((id, Arc::new(tx)));
    // A stalled audio thread must not grow this without bound.
    let n = p.len();
    if n > 16 {
        p.drain(..n - 16);
    }
    id
}

fn take(id: u32) -> Option<Arc<Tx>> {
    let mut p = PENDING.try_lock().ok()?;
    let i = p.iter().position(|x| x.0 == id)?;
    Some(p.remove(i).1)
}

/// One receiver playing a transmission.
struct Slot {
    tx: Option<Arc<Tx>>,
    /// Token index, sample position within the current word (in pack samples, fractional).
    tok: usize,
    pos: f64,
    /// Seconds left of the current gap.
    gap: f32,
    /// Squelch phases (s remaining).
    open: f32,
    tail: f32,
    /// Waiting for the audio thread to take it.
    want: Option<u32>,
    // DSP.
    hp1: Svf,
    hp2: Svf,
    lp1: Svf,
    lp2: Svf,
    pres: Svf,
    nlp: Svf,
    nhp: Svf,
    env: f32,
    gain: f32,
    fade: OnePole,
    level: f32,
}

impl Slot {
    fn new(sr: f32) -> Slot {
        Slot {
            tx: None,
            tok: 0,
            pos: 0.0,
            gap: 0.0,
            open: 0.0,
            tail: 0.0,
            want: None,
            hp1: Svf::new(330.0, 0.71, sr),
            hp2: Svf::new(330.0, 0.71, sr),
            lp1: Svf::new(2900.0, 0.71, sr),
            lp2: Svf::new(2900.0, 0.71, sr),
            pres: Svf::new(1900.0, 1.6, sr),
            nlp: Svf::new(3200.0, 0.6, sr),
            nhp: Svf::new(500.0, 0.6, sr),
            env: 0.0,
            gain: 1.0,
            fade: OnePole::new(30.0, sr),
            level: 0.0,
        }
    }
    fn busy(&self) -> bool {
        self.tx.is_some() || self.tail > 0.0
    }
}

/// The receivers (audio thread).
pub struct Radio {
    sr: f32,
    slots: [Slot; 2],
    rng: Rng,
    /// 0..1 "someone is talking" for ducking the world.
    pub active: f32,
}

impl Radio {
    pub fn new(sr: f32) -> Radio {
        Radio { sr, slots: [Slot::new(sr), Slot::new(sr)], rng: Rng::new(0x5ad10), active: 0.0 }
    }

    /// A transmission was submitted (id from [`submit`]).
    pub fn start(&mut self, id: u32) {
        // Which slot: the submission says; until it is taken, park the id on slot 0's queue.
        match take(id) {
            Some(tx) => self.begin(tx),
            None => self.slots[0].want = Some(id),
        }
    }

    fn begin(&mut self, tx: Arc<Tx>) {
        let s = &mut self.slots[(tx.ch as usize).min(1)];
        s.open = if tx.own { 0.02 } else { SQUELCH_OPEN };
        s.tail = 0.0;
        s.tok = 0;
        s.pos = 0.0;
        s.gap = 0.0;
        s.tx = Some(tx);
    }

    /// Cut a receiver (detuned / power lost): a short tail.
    pub fn cut(&mut self, ch: u8) {
        let s = &mut self.slots[(ch as usize).min(1)];
        if s.tx.take().is_some() {
            s.tail = 0.06;
        }
    }

    #[inline]
    pub fn tick(&mut self) -> f32 {
        if let Some(id) = self.slots[0].want {
            if let Some(tx) = take(id) {
                self.slots[0].want = None;
                self.begin(tx);
            }
        }
        let dt = 1.0 / self.sr;
        let mut out = 0.0;
        let mut any = false;
        for s in self.slots.iter_mut() {
            if !s.busy() {
                continue;
            }
            any = true;
            let (own, strength, vol, voice) = match &s.tx {
                Some(t) => (t.own, t.strength.clamp(0.0, 1.0), t.volume, Some(t.voice)),
                None => (false, 0.5, 1.0, None),
            };
            // --- speech sample
            let mut x = 0.0;
            let mut end = false;
            if let (Some(t), Some(v)) = (s.tx.as_deref(), voice) {
                if s.open > 0.0 {
                    s.open -= dt;
                } else if s.gap > 0.0 {
                    s.gap -= dt;
                } else if s.tok >= t.toks.len() {
                    end = true;
                } else {
                    match t.toks[s.tok] {
                        Tok::Gap(g) => {
                            s.gap = g;
                            s.tok += 1;
                        }
                        Tok::Word(w) => match (pack(v), parse_cached(v).get(&w)) {
                            (Some(p), Some(&(a, n))) => {
                                let ip = s.pos as usize;
                                if ip + 1 >= n as usize {
                                    s.tok += 1;
                                    s.pos = 0.0;
                                    s.gap = WORD_GAP;
                                } else {
                                    let i = (a as usize + ip).min(p.pcm.len().saturating_sub(2));
                                    let f = (s.pos - ip as f64) as f32;
                                    x = p.pcm[i] + (p.pcm[i + 1] - p.pcm[i]) * f;
                                    s.pos += p.rate as f64 / self.sr as f64;
                                }
                            }
                            // Not decoded yet / not in this voice: skip the word.
                            _ => {
                                s.tok += 1;
                                s.pos = 0.0;
                            }
                        },
                    }
                }
            }
            if end {
                // End of the message: the carrier drops → squelch tail.
                s.tx = None;
                s.tail = if own { 0.0 } else { SQUELCH_TAIL };
            }
            // --- noise: carrier hiss while keyed, the squelch burst at open / close
            let keyed = s.tx.is_some();
            let white = self.rng.bi();
            let mut noise = 0.0;
            if !own {
                let hiss = if keyed { 0.012 + 0.06 * (1.0 - strength).powi(2) } else { 0.0 };
                let burst = if keyed && s.open > 0.0 {
                    0.22 * (s.open / SQUELCH_OPEN).clamp(0.0, 1.0)
                } else if !keyed && s.tail > 0.0 {
                    let k = s.tail / SQUELCH_TAIL;
                    0.3 * k * k
                } else {
                    0.0
                };
                noise = (hiss + burst) * white;
            }
            if !keyed && s.tail > 0.0 {
                s.tail -= dt;
            }
            // Weak signal: slow fading of the speech.
            let fadeg = if own { 1.0 } else { 1.0 - 0.35 * (1.0 - strength) * (0.5 + 0.5 * (self.rng.uni() - 0.5)) };
            let fadeg = s.fade.lp(fadeg);
            // --- voice band, compressor, drive
            let y = x * fadeg + noise;
            let y = s.hp2.hp(s.hp1.hp(y));
            let y = s.lp2.lp(s.lp1.lp(y));
            let y = y + 0.45 * s.pres.bp(y);
            let a = y.abs();
            let k = if a > s.env { 0.02 } else { 0.0012 };
            s.env += (a - s.env) * k;
            let thr = 0.12;
            let want = if s.env > thr { (thr / s.env).powf(0.7) } else { 1.0 };
            s.gain += (want - s.gain) * 0.002;
            let drive = if own { 1.6 } else { 2.6 };
            let y = sat(y * s.gain * drive) * 0.7;
            // A touch of extra band-limited hiss outside the voice chain.
            let n2 = if keyed && !own { s.nhp.hp(s.nlp.lp(white)) * 0.006 } else { 0.0 };
            let lvl = vol * if own { 0.75 } else { 1.0 };
            s.level += (lvl - s.level) * 0.001;
            out += (y + n2) * s.level;
        }
        let target = if any { 1.0 } else { 0.0 };
        let rate = if target > self.active { 0.004 } else { 0.0001 };
        self.active += (target - self.active) * rate * 48_000.0 / self.sr;
        out
    }
}
