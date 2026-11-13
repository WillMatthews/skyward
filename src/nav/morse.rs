//! Morse identification of navaids: the keying of an ident ("IRR" →
//! `.. .-. .-.`) at the ICAO rate of about 7 words per minute, repeated with
//! a pause, as a function of time — so the audio thread keys its 1020 Hz
//! (VOR / ILS) or 400 Hz-modulated (NDB) tone sample-accurately — and an
//! offline WAV renderer.

/// Dot length (s): 7 words per minute (PARIS standard: 1.2 s / wpm).
pub const DOT: f32 = 1.2 / 7.0;
/// Silence between repetitions of the ident (s).
pub const REPEAT_GAP: f32 = 7.0;
/// Ident tone (Hz): VOR / ILS / DME 1020 Hz.
pub const TONE_HZ: f32 = 1020.0;

pub fn code(c: char) -> Option<&'static str> {
    Some(match c.to_ascii_uppercase() {
        'A' => ".-",
        'B' => "-...",
        'C' => "-.-.",
        'D' => "-..",
        'E' => ".",
        'F' => "..-.",
        'G' => "--.",
        'H' => "....",
        'I' => "..",
        'J' => ".---",
        'K' => "-.-",
        'L' => ".-..",
        'M' => "--",
        'N' => "-.",
        'O' => "---",
        'P' => ".--.",
        'Q' => "--.-",
        'R' => ".-.",
        'S' => "...",
        'T' => "-",
        'U' => "..-",
        'V' => "...-",
        'W' => ".--",
        'X' => "-..-",
        'Y' => "-.--",
        'Z' => "--..",
        '0' => "-----",
        '1' => ".----",
        '2' => "..---",
        '3' => "...--",
        '4' => "....-",
        '5' => ".....",
        '6' => "-....",
        '7' => "--...",
        '8' => "---..",
        '9' => "----.",
        _ => return None,
    })
}

/// The key-down intervals of one ident (seconds from its start) and its
/// length including the inter-letter gaps (no trailing gap).
pub fn elements(ident: &str) -> (Vec<(f32, f32)>, f32) {
    let mut t = 0.0;
    let mut out = Vec::new();
    let letters: Vec<&str> = ident.chars().filter_map(code).collect();
    for (li, l) in letters.iter().enumerate() {
        for (ei, e) in l.chars().enumerate() {
            let len = if e == '-' { 3.0 * DOT } else { DOT };
            out.push((t, t + len));
            t += len;
            if ei + 1 < l.len() {
                t += DOT; // between elements
            }
        }
        if li + 1 < letters.len() {
            t += 3.0 * DOT; // between letters
        }
    }
    (out, t)
}

/// A compiled ident: keying as a function of time.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Keyer {
    on: Vec<(f32, f32)>,
    period: f32,
}

impl Keyer {
    pub fn new(ident: &str) -> Keyer {
        let (on, len) = elements(ident);
        Keyer { period: if on.is_empty() { 0.0 } else { len + REPEAT_GAP }, on }
    }
    /// Is the key down at time `t` (s; the ident starts at t = 0 and repeats)?
    pub fn key(&self, t: f32) -> bool {
        if self.period <= 0.0 {
            return false;
        }
        let x = t.rem_euclid(self.period);
        self.on.iter().any(|&(a, b)| x >= a && x < b)
    }
    pub fn period(&self) -> f32 {
        self.period
    }
}

/// Decode a key-down list back into letters (for tests): elements longer than
/// 2 dots are dashes, gaps longer than 2 dots separate letters.
pub fn decode(on: &[(f32, f32)], dot: f32) -> String {
    let mut out = String::new();
    let mut cur = String::new();
    let mut prev_end: Option<f32> = None;
    for &(a, b) in on {
        if let Some(pe) = prev_end {
            if a - pe > 2.0 * dot {
                out.push(letter(&cur));
                cur.clear();
            }
        }
        cur.push(if b - a > 2.0 * dot { '-' } else { '.' });
        prev_end = Some(b);
    }
    if !cur.is_empty() {
        out.push(letter(&cur));
    }
    out
}

fn letter(c: &str) -> char {
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789".chars().find(|&ch| code(ch) == Some(c)).unwrap_or('?')
}

/// Render `secs` seconds of an ident (16-bit mono PCM samples at `sr`):
/// the tone keyed with 5 ms raised-cosine edges (no clicks).
pub fn render(ident: &str, sr: u32, secs: f32) -> Vec<i16> {
    let k = Keyer::new(ident);
    let n = (secs * sr as f32) as usize;
    let mut env = 0.0f32;
    let ramp = 1.0 / (0.005 * sr as f32);
    (0..n)
        .map(|i| {
            let t = i as f32 / sr as f32;
            let target = if k.key(t) { 1.0 } else { 0.0 };
            env = if target > env { (env + ramp).min(1.0) } else { (env - ramp).max(0.0) };
            let shaped = 0.5 - 0.5 * (env * std::f32::consts::PI).cos();
            let s = (t * TONE_HZ * std::f32::consts::TAU).sin() * shaped * 0.5;
            (s * 32767.0) as i16
        })
        .collect()
}

/// Write a WAV of an ident (offline verification; never played).
#[cfg(not(target_arch = "wasm32"))]
pub fn write_wav(path: &str, ident: &str, secs: f32) -> Result<(), String> {
    let sr = 22_050;
    let spec = hound::WavSpec { channels: 1, sample_rate: sr, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec).map_err(|e| e.to_string())?;
    for s in render(ident, sr, secs) {
        w.write_sample(s).map_err(|e| e.to_string())?;
    }
    w.finalize().map_err(|e| e.to_string())
}

/// Recover the key-down intervals from PCM by envelope detection (tests).
pub fn detect(samples: &[i16], sr: u32) -> Vec<(f32, f32)> {
    let win = (sr as f32 * 0.004) as usize;
    let mut out = Vec::new();
    let mut start = None;
    for (k, chunk) in samples.chunks(win.max(1)).enumerate() {
        let peak = chunk.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0) as f32 / 32767.0;
        let t = (k * win) as f32 / sr as f32;
        match (peak > 0.2, start) {
            (true, None) => start = Some(t),
            (false, Some(s)) => {
                out.push((s, t));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        out.push((s, samples.len() as f32 / sr as f32));
    }
    out
}
