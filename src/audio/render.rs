//! Offline audio rendering for verification without speakers:
//! `skyward --audio-render out.wav --seconds N [--aircraft ..] [--bot] [--cam ..]
//!  [--audio-script "2:cam=cockpit,10:gear,..."]` runs the sim headlessly
//! (no GPU, no audio device) and writes the mixed audio, a spectrogram PNG
//! (with the expected engine frequencies overlaid and event marks) and a
//! report of objective checks (peak, clipping, NaNs, clicks, CPU cost).

use super::capture::Director;
use super::dsp::fft;
use super::params::{Event, Family, Frame};
use super::synth::{Settings, Synth};
use crate::sim::{CamMode, Input, Key, Sim};
use crate::world::World;
use std::time::{Duration, Instant};

/// When a scripted cue fires.
#[derive(Clone, Copy, Debug)]
enum Trigger {
    /// Seconds into the recording.
    Time(f32),
    /// Radio altitude (ft above the wheels) at or below this, airborne.
    Ra(f32),
    /// Seconds after the next touchdown.
    Touchdown(f32),
}

/// Scripted cues: `"T:cue"` (seconds), `"ra200:cue"` (radio altitude, ft),
/// `"td:cue"` / `"td+5:cue"` (after touchdown). Cues: `cam=chase|cockpit|...`,
/// `throttle=X`, `gear`, `flaps+`/`flaps-`, `spoilers+`/`spoilers-`,
/// `reverse`, `ap`, `ap_off`, `ap_alt=M`, `ap_vs=MS`, `ap_ias=KT`,
/// `athr_off`, `fail`, `pause`, `warp=N`, `hold=KEY`/`release=KEY`, `nobot`.
pub struct Cues {
    list: Vec<(Trigger, String, bool)>,
    td_at: Option<f32>,
    was_air: bool,
}

pub fn parse_script(s: &str) -> Cues {
    let list = s
        .split(',')
        .filter_map(|c| {
            let (t, w) = c.trim().split_once(':')?;
            let t = t.trim();
            let trig = if let Some(ft) = t.strip_prefix("ra") {
                Trigger::Ra(ft.parse().ok()?)
            } else if let Some(rest) = t.strip_prefix("td") {
                Trigger::Touchdown(rest.trim_start_matches('+').parse().unwrap_or(0.0))
            } else {
                Trigger::Time(t.parse().ok()?)
            };
            Some((trig, w.trim().to_string(), false))
        })
        .collect();
    Cues { list, td_at: None, was_air: false }
}

impl Cues {
    /// Cues due now (each fires once).
    pub fn due(&mut self, t: f32, sim: &Sim, world: &World) -> Vec<String> {
        use crate::flight::Environment;
        let ac = &sim.ac;
        let ra_ft = (ac.pos.y - world.ground_height(glam::vec2(ac.pos.x, ac.pos.z)) - sim.def().gear_height) / crate::flight::FT;
        if !ac.on_ground {
            self.was_air = true;
        } else if self.was_air && self.td_at.is_none() {
            self.td_at = Some(t);
        }
        let mut out = Vec::new();
        for (trig, what, done) in &mut self.list {
            if *done {
                continue;
            }
            let fire = match *trig {
                Trigger::Time(x) => t >= x,
                Trigger::Ra(ft) => !ac.on_ground && ra_ft <= ft,
                Trigger::Touchdown(d) => self.td_at.is_some_and(|t0| t >= t0 + d),
            };
            if fire {
                *done = true;
                out.push(what.clone());
            }
        }
        out
    }
}

fn key_named(s: &str) -> Option<Key> {
    Some(match s {
        "pitchup" => Key::PitchUp,
        "pitchdown" => Key::PitchDown,
        "rollleft" => Key::RollLeft,
        "rollright" => Key::RollRight,
        "brakes" => Key::Brakes,
        "throttleup" => Key::ThrottleUp,
        "throttledown" => Key::ThrottleDown,
        _ => return None,
    })
}

/// Apply a cue to the sim.
pub fn apply(what: &str, sim: &mut Sim, input: &mut Input) {
    let (k, v) = what.split_once('=').unwrap_or((what, ""));
    let num = |d: f32| v.parse::<f32>().unwrap_or(d);
    match k {
        "cam" => {
            // update() handles every queued press, so cycle there.
            if let Some(m) = CamMode::parse(v) {
                let mut c = sim.cam;
                while c != m {
                    input.pressed.push(Key::Camera);
                    c = next_cam(c);
                }
            }
        }
        "throttle" => sim.ctl.throttle = num(sim.ctl.throttle),
        "gear" => input.pressed.push(Key::Gear),
        "flaps+" => input.pressed.push(Key::FlapsDown),
        "flaps-" => input.pressed.push(Key::FlapsUp),
        "spoilers+" => input.pressed.push(Key::SpoilerOut),
        "spoilers-" => input.pressed.push(Key::SpoilerIn),
        "reverse" => input.pressed.push(Key::Reverse),
        "ap" => input.pressed.push(Key::ApToggle),
        "ap_off" => {
            sim.ap.disengage();
            sim.ap_warn = 4.0;
        }
        "ap_alt" => sim.ap.alt = Some(num(0.0)),
        "ap_vs" => sim.ap.vs = Some(num(-3.5)),
        "ap_ias" => sim.ap.ias = Some(num(140.0) * crate::flight::KT),
        "athr_off" => sim.ap.ias = None,
        "fail" => input.pressed.push(Key::EngineFail),
        "pause" => input.pressed.push(Key::Pause),
        "warp" => sim.warp = num(1.0) as u32,
        "hold" => {
            if let Some(k) = key_named(v) {
                input.held.insert(k);
            }
        }
        "release" => {
            if let Some(k) = key_named(v) {
                input.held.remove(&k);
            }
        }
        "nobot" => sim.disable_bot("script"),
        _ => eprintln!("audio-script: unknown cue {what}"),
    }
}

fn next_cam(c: CamMode) -> CamMode {
    match c {
        CamMode::Chase => CamMode::Cockpit,
        CamMode::Cockpit => CamMode::Tower,
        CamMode::Tower => CamMode::Flyby,
        CamMode::Flyby => CamMode::Cinematic,
        CamMode::Cinematic => CamMode::Chase,
    }
}

/// Records the mixed output of a sim run.
pub struct Recorder {
    pub sr: u32,
    synth: Synth,
    director: Director,
    events: Vec<Event>,
    pub buf: Vec<f32>,
    carry: f64,
    cpu: Duration,
    /// (time, label) for event marks.
    marks: Vec<(f32, String)>,
    /// (time, expected engine-0 frequencies) for the overlay.
    lines: Vec<(f32, [f32; 2])>,
    t: f32,
    last_frame: Frame,
}

impl Recorder {
    pub fn new(sr: u32) -> Recorder {
        let mut synth = Synth::new(sr as f32);
        synth.set_settings(Settings::default());
        Recorder {
            sr,
            synth,
            director: Director::default(),
            events: Vec::new(),
            buf: Vec::new(),
            carry: 0.0,
            cpu: Duration::ZERO,
            marks: Vec::new(),
            lines: Vec::new(),
            t: 0.0,
            last_frame: Frame::default(),
        }
    }

    pub fn mark(&mut self, label: impl Into<String>) {
        self.marks.push((self.t, label.into()));
    }

    /// Capture this frame's state and render `dt` seconds of audio.
    pub fn step(&mut self, sim: &Sim, world: &World, dt: f32, menu: bool) {
        self.events.clear();
        let f = self.director.capture(sim, world, dt, menu, &mut self.events);
        if f.profile.type_id != self.last_frame.profile.type_id {
            // Offline: decode now so the render is deterministic.
            if let Some(set) = super::soundsets::set(f.profile.set) {
                super::samples::prepare_blocking(set.samples);
            }
            super::samples::prepare_blocking(super::soundsets::COMMON);
        }
        let t0 = Instant::now();
        self.synth.push_frame(&f);
        for e in &self.events {
            self.synth.event(e);
        }
        self.carry += dt as f64 * self.sr as f64;
        let n = self.carry.floor() as usize;
        self.carry -= n as f64;
        let start = self.buf.len();
        self.buf.resize(start + n * 2, 0.0);
        let booms = self.synth.booms;
        self.synth.render(&mut self.buf[start..]);
        self.cpu += t0.elapsed();
        if self.synth.booms != booms {
            let (len, dp) = self.synth.last_boom;
            self.marks.push((self.t, format!("sonic boom: N-wave {:.0} ms, {:.0} Pa", len * 1000.0, dp)));
        }
        for e in &self.events {
            let label = match e {
                Event::Speak(c) => format!("say {c:?}"),
                e => format!("{e:?}").split(['{', '(']).next().unwrap_or("").trim().to_string(),
            };
            self.marks.push((self.t, label));
        }
        if f.cut {
            self.marks.push((self.t, format!("cam {:?}", sim.cam)));
        }
        let p = f.profile;
        let e = f.eng[0];
        let lines = match p.family {
            Family::LightPiston | Family::Warbird => [e.rpm / 60.0 * p.cylinders as f32 * 0.5, e.rpm * p.prop_gear / 60.0 * p.blades as f32],
            Family::Turboprop => [e.rpm * p.prop_gear / 60.0 * p.blades as f32, 0.0],
            Family::Airliner | Family::Fighter => [e.rpm / 60.0 * p.fan_blades as f32, e.rpm / 60.0],
            Family::Glider => [0.0, 0.0],
        };
        self.lines.push((self.t, lines));
        if std::env::var_os("SKYWARD_AUDIO_DEBUG").is_some() && (self.t * 2.0).floor() != ((self.t + dt) * 2.0).floor() {
            eprintln!(
                "t {:6.1} {} cam {:?} ear-ac {:6.0} m  tas {:5.1} mach {:.2} rpm {:6.0} spool {:.2} thr {:+.2} ab {:.2} rev {:.2} gear {:.2} flaps {:.2} gnd {} gs {:4.1} buffet {:.2} shake {:.2} vario {:?}",
                self.t, sim.bot.as_ref().map(|b| b.phase()).unwrap_or_default(), sim.cam, f.ear_pos.distance(f.ac_pos), f.tas, f.mach, e.rpm, e.spool, e.thrust, e.ab, f.reverser, f.gear, f.flaps, f.on_ground, f.ground_speed, f.buffet, f.shake, f.vario
            );
        }
        self.last_frame = f;
        self.t += dt;
    }

    /// Write the WAV, spectrogram PNG and report. Returns the report text.
    pub fn finish(&self, path: &str) -> Result<String, String> {
        // 32-bit float: no quantisation artefacts in the analysis.
        let spec = hound::WavSpec { channels: 2, sample_rate: self.sr, bits_per_sample: 32, sample_format: hound::SampleFormat::Float };
        let mut w = hound::WavWriter::create(path, spec).map_err(|e| e.to_string())?;
        for &x in &self.buf {
            w.write_sample(x).map_err(|e| e.to_string())?;
        }
        w.finalize().map_err(|e| e.to_string())?;
        let png = path.trim_end_matches(".wav").to_string() + ".png";
        spectrogram(&self.buf, self.sr, &self.marks, &self.lines, &png)?;
        let rep = self.report();
        let txt = path.trim_end_matches(".wav").to_string() + ".txt";
        std::fs::write(&txt, &rep).map_err(|e| e.to_string())?;
        Ok(rep)
    }

    fn report(&self) -> String {
        let sr = self.sr as f32;
        let n = self.buf.len() / 2;
        let secs = n as f32 / sr;
        let mono: Vec<f32> = (0..n).map(|i| 0.5 * (self.buf[2 * i] + self.buf[2 * i + 1])).collect();
        let peak = self.buf.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let clips = self.buf.iter().filter(|x| x.abs() >= 0.999).count();
        let nans = self.buf.iter().filter(|x| !x.is_finite()).count() as u64 + self.synth.nan_count;
        let rms_all = (mono.iter().map(|x| (x * x) as f64).sum::<f64>() / n.max(1) as f64).sqrt() as f32;
        let win = sr as usize;
        let mut per_s: Vec<f32> = mono.chunks(win).map(|c| (c.iter().map(|x| x * x).sum::<f32>() / c.len() as f32).sqrt()).collect();
        let rms_line: String = per_s.iter().map(|r| format!("{:.0}", dbf(*r))).collect::<Vec<_>>().join(" ");
        per_s.sort_by(|a, b| a.total_cmp(b));
        // Click detector: isolated spikes in the second difference relative
        // to its local level.
        let mut clicks = Vec::new();
        let d2: Vec<f32> = (2..n).map(|i| mono[i] - 2.0 * mono[i - 1] + mono[i - 2]).collect();
        let w2 = (0.02 * sr) as usize;
        let mut acc = 0.0f64;
        for i in 0..d2.len() {
            acc += (d2[i] * d2[i]) as f64;
            if i >= w2 {
                acc -= (d2[i - w2] * d2[i - w2]) as f64;
            }
            if i >= w2 {
                let rms = (acc.max(0.0) / w2 as f64).sqrt() as f32;
                if d2[i].abs() > 0.03 && d2[i].abs() > 14.0 * rms.max(1e-6) {
                    clicks.push(i as f32 / sr);
                }
            }
        }
        let cpu_ms_per_10ms = self.cpu.as_secs_f64() * 1000.0 / (secs as f64 * 100.0).max(1e-9);
        let mut s = String::new();
        s += &format!("duration {:.1} s @ {} Hz\n", secs, self.sr);
        s += &format!("peak {:.2} dBFS, overall RMS {:.1} dBFS, loudest second {:.1} dBFS, median second {:.1} dBFS\n", dbf(peak), dbf(rms_all), dbf(*per_s.last().unwrap_or(&0.0)), dbf(per_s.get(per_s.len() / 2).copied().unwrap_or(0.0)));
        s += &format!("samples at full scale: {clips}; limiter safety-knee samples: {}; limiter min gain {:.1} dB; NaN/inf: {nans}\n", self.synth.limiter_clamped(), dbf(self.synth.limiter_min_gain()));
        s += &format!("isolated-spike detector: {} hits{}\n", clicks.len(), if clicks.is_empty() { String::new() } else { format!(" (first: {})", clicks.iter().take(8).map(|t| format!("{t:.3}s")).collect::<Vec<_>>().join(", ")) });
        s += &format!("synth CPU: {:.3} ms per 10 ms block ({:.2} % of one core)\n", cpu_ms_per_10ms, cpu_ms_per_10ms * 10.0);
        s += &format!("RMS per second (dBFS): {rms_line}\n");
        s += "events:\n";
        for (t, l) in &self.marks {
            s += &format!("  {t:7.2}s  {l}\n");
        }
        s
    }
}

fn dbf(x: f32) -> f32 {
    20.0 * x.max(1e-7).log10()
}

// ------------------------------------------------------------------ spectrogram

fn colormap(v: f32) -> [u8; 3] {
    // Dark blue → purple → orange → pale yellow.
    let stops: [(f32, [f32; 3]); 5] = [(0.0, [0.0, 0.0, 0.02]), (0.3, [0.25, 0.04, 0.45]), (0.55, [0.75, 0.15, 0.35]), (0.8, [0.98, 0.55, 0.1]), (1.0, [1.0, 1.0, 0.75])];
    let v = v.clamp(0.0, 1.0);
    for w in stops.windows(2) {
        if v <= w[1].0 {
            let t = (v - w[0].0) / (w[1].0 - w[0].0);
            let c = [0, 1, 2].map(|i| ((w[0].1[i] + (w[1].1[i] - w[0].1[i]) * t) * 255.0) as u8);
            return c;
        }
    }
    [255, 255, 190]
}

fn spectrogram(buf: &[f32], sr: u32, marks: &[(f32, String)], lines: &[(f32, [f32; 2])], path: &str) -> Result<(), String> {
    let n = buf.len() / 2;
    let mono: Vec<f32> = (0..n).map(|i| 0.5 * (buf[2 * i] + buf[2 * i + 1])).collect();
    let fft_n = 4096usize;
    let cols = 1800usize.min(n / 256).max(10);
    let hop = (n.saturating_sub(fft_n) / cols).max(1);
    let (spec_h, wave_h) = (520usize, 90usize);
    let h = spec_h + wave_h + 4;
    let mut img = image::RgbImage::new(cols as u32, h as u32);
    let (fmin, fmax) = (25.0f32, 16_000.0f32);
    let win: Vec<f32> = (0..fft_n).map(|i| 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / fft_n as f32).cos()).collect();
    let mut re = vec![0.0f32; fft_n];
    let mut im = vec![0.0f32; fft_n];
    let secs = n as f32 / sr as f32;
    for c in 0..cols {
        let s0 = c * hop;
        for i in 0..fft_n {
            re[i] = mono.get(s0 + i).copied().unwrap_or(0.0) * win[i];
            im[i] = 0.0;
        }
        fft(&mut re, &mut im);
        for y in 0..spec_h {
            // Log frequency axis, top = high.
            let frac = 1.0 - y as f32 / spec_h as f32;
            let f = fmin * (fmax / fmin).powf(frac);
            let bin = (f / sr as f32 * fft_n as f32).round() as usize;
            let bin2 = ((f * 1.012) / sr as f32 * fft_n as f32).round() as usize;
            let mut p = 0.0f32;
            for b in bin..=bin2.max(bin) {
                if b < fft_n / 2 {
                    p = p.max(re[b] * re[b] + im[b] * im[b]);
                }
            }
            // 0 dB = a full-scale sine (Hann window: |X| = A·N/4).
            let db = 10.0 * (p.max(1e-20) / (fft_n as f32 * fft_n as f32 / 16.0)).log10();
            let v = (db + 115.0) / 95.0;
            img.put_pixel(c as u32, y as u32, image::Rgb(colormap(v)));
        }
        // Waveform strip: peak and RMS of this column's hop.
        let seg = &mono[s0.min(n)..(s0 + hop).min(n)];
        let pk = seg.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let rms = (seg.iter().map(|x| x * x).sum::<f32>() / seg.len().max(1) as f32).sqrt();
        let y0 = spec_h + 4;
        for y in 0..wave_h {
            let a = 1.0 - y as f32 / wave_h as f32;
            let col = if a <= rms { [120, 200, 255] } else if a <= pk { [40, 90, 140] } else { [12, 12, 16] };
            img.put_pixel(c as u32, (y0 + y) as u32, image::Rgb(col));
        }
        // 0.9 FS reference line.
        let yr = y0 + (wave_h as f32 * 0.1) as usize;
        img.put_pixel(c as u32, yr as u32, image::Rgb([200, 60, 60]));
    }
    // Expected engine frequencies (cyan / green dots) — pitch tracking.
    let y_of = |f: f32| -> Option<u32> {
        if f < fmin || f > fmax {
            return None;
        }
        let frac = (f / fmin).ln() / (fmax / fmin).ln();
        Some(((1.0 - frac) * spec_h as f32) as u32)
    };
    for (t, fr) in lines {
        let c = ((t * sr as f32) as usize / hop).min(cols - 1);
        if !c.is_multiple_of(3) {
            continue;
        }
        for (k, f) in fr.iter().enumerate() {
            if let Some(y) = y_of(*f) {
                let col = if k == 0 { [0, 255, 255] } else { [120, 255, 120] };
                img.put_pixel(c as u32, y.min(spec_h as u32 - 1), image::Rgb(col));
            }
        }
    }
    // Event marks: white ticks along the top; camera cuts full-height dashed.
    for (t, l) in marks {
        let c = ((t * sr as f32) as usize / hop).min(cols - 1) as u32;
        let cam = l.starts_with("cam");
        let len = if cam { spec_h as u32 } else { 14 };
        for y in 0..len {
            if !cam || (y / 6) % 2 == 0 {
                img.put_pixel(c, y, image::Rgb(if cam { [255, 255, 255] } else { [80, 255, 80] }));
            }
        }
    }
    // Second ticks along the bottom of the spectrogram, octave lines left.
    for s in 0..secs as usize {
        let c = ((s as f32 * sr as f32) as usize / hop).min(cols - 1) as u32;
        let len = if s % 10 == 0 { 10 } else { 4 };
        for y in 0..len {
            img.put_pixel(c, (spec_h - 1 - y) as u32, image::Rgb([255, 255, 255]));
        }
    }
    for f in [50.0f32, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0] {
        if let Some(y) = y_of(f) {
            for x in 0..8 {
                img.put_pixel(x, y, image::Rgb([255, 255, 255]));
            }
        }
    }
    img.save(path).map_err(|e| e.to_string())
}
