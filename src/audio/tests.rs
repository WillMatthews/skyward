//! Offline checks of the synthesis engine with synthetic frames (no sim,
//! no device). `SKYWARD_AUDIO_TEST_OUT=/dir cargo test audio` also writes
//! the WAVs for inspection.

use super::capture::profile_for;
use super::params::*;
use super::synth::Synth;
use glam::{Quat, Vec3};

const SR: f32 = 48_000.0;

fn frame_for(id: &str) -> Frame {
    let def = crate::flight::find(id).expect("aircraft");
    let p = profile_for(&def, 18.0);
    let mut f = Frame { active: true, warp: 1.0, profile: p, interior: 0.0, day: 1.0, ..Frame::default() };
    f.ear_right = Vec3::X;
    f.ear_up = Vec3::Y;
    f.ear_fwd = Vec3::NEG_Z;
    f.ear_agl = 2.0;
    f.gear = 1.0;
    for i in 0..p.engines {
        f.eng[i] = EngineFrame { rpm: p.n1_rpm * 0.85, spool: 0.85, thrust: 0.8, running: true, throttle: 0.85, ..Default::default() };
    }
    f
}

/// Fly `f` along +X past the listener at the origin; returns mono output.
fn flyby(mut f: Frame, speed: f32, offset: f32, secs: f32) -> Vec<f32> {
    let mut s = Synth::new(SR);
    let frames = (secs * 60.0) as usize;
    let mut out = Vec::new();
    let mut buf = vec![0.0f32; 1600];
    let x0 = -speed * secs * 0.5;
    for k in 0..frames {
        let t = k as f32 / 60.0;
        let pos = Vec3::new(x0 + speed * t, offset, 0.0);
        f.ac_pos = pos;
        f.ac_vel = Vec3::new(speed, 0.0, 0.0);
        f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2); // nose along +X
        f.tas = speed;
        for i in 0..f.profile.engines {
            f.eng[i].pos = pos;
        }
        f.cut = k == 0;
        s.push_frame(&f);
        s.render(&mut buf);
        out.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
    }
    out
}

fn save(name: &str, x: &[f32]) {
    if let Some(dir) = std::env::var_os("SKYWARD_AUDIO_TEST_OUT") {
        let p = std::path::Path::new(&dir).join(name);
        let spec = hound::WavSpec { channels: 1, sample_rate: SR as u32, bits_per_sample: 32, sample_format: hound::SampleFormat::Float };
        let mut w = hound::WavWriter::create(p, spec).unwrap();
        for v in x {
            w.write_sample(*v).unwrap();
        }
        w.finalize().unwrap();
    }
}

/// Dominant frequency in [lo, hi] Hz of a segment (zero-padded DFT peak).
fn peak_freq(x: &[f32], lo: f32, hi: f32) -> f32 {
    let n = x.len();
    let mut best = (0.0f32, lo);
    let mut f = lo;
    while f < hi {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (i, v) in x.iter().enumerate() {
            let w = 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / n as f32).cos();
            let a = std::f64::consts::TAU * f as f64 * i as f64 / SR as f64;
            re += (v * w) as f64 * a.cos();
            im += (v * w) as f64 * a.sin();
        }
        let m = (re * re + im * im) as f32;
        if m > best.0 {
            best = (m, f);
        }
        f += 1.0;
    }
    best.1
}

#[test]
fn doppler_matches_moving_source_formula() {
    // A Meridian 320 at 90 m/s passes 60 m from the listener. Its fan tone
    // must be shifted by 1/(1 − v/c) on approach and 1/(1 + v/c) going away.
    let mut f = frame_for("meridian320");
    let p = f.profile;
    for i in 0..p.engines {
        f.eng[i].rpm = p.n1_rpm * 0.6; // subsonic tips: clean blade-pass tone
        f.eng[i].spool = 0.6;
    }
    let v = 90.0;
    let x = flyby(f, v, 60.0, 16.0);
    save("doppler.wav", &x);
    let bpf = p.n1_rpm * 0.6 / 60.0 * p.fan_blades as f32;
    let c = 343.0;
    let seg = |t: f32| &x[(t * SR) as usize..((t + 0.5) * SR) as usize];
    let up = peak_freq(seg(2.0), bpf * 1.1, bpf * 1.5);
    let down = peak_freq(seg(13.0), bpf * 0.6, bpf * 0.95);
    let want_up = bpf / (1.0 - v / c * 0.99);
    let want_down = bpf / (1.0 + v / c * 0.99);
    assert!((up / want_up - 1.0).abs() < 0.03, "approach {up} Hz vs {want_up}");
    assert!((down / want_down - 1.0).abs() < 0.03, "recede {down} Hz vs {want_down}");
}

#[test]
fn every_family_is_finite_and_limited() {
    for id in ["kestrel", "swift", "peregrine", "lancer", "aerolink", "meridian190", "meridian320"] {
        let mut f = frame_for(id);
        f.rain = 1.0;
        f.buffet = 1.0;
        f.overspeed = 1.0;
        f.stall_warn = true;
        f.gear_warn = true;
        f.shake = 1.5;
        f.reverser = 1.0;
        f.vario = Some(4.0);
        for e in f.eng.iter_mut() {
            e.ab = 1.0;
            e.spool = 1.05;
            e.thrust = 1.5;
        }
        let mut s = Synth::new(SR);
        let mut buf = vec![0.0f32; 1600];
        let mut peak = 0.0f32;
        for k in 0..240 {
            f.interior = if (k / 60) % 2 == 0 { 0.0 } else { 1.0 };
            f.ac_pos = Vec3::new(0.0, 2.0, -20.0);
            f.cut = k % 60 == 0;
            s.push_frame(&f);
            if k == 30 {
                for e in [Event::Crash { speed: 80.0 }, Event::Boom(1.0), Event::Thunder(1.0), Event::AbLight(0), Event::Touchdown { sink: 4.0, speed: 70.0 }] {
                    s.event(&e);
                }
            }
            s.render(&mut buf);
            for v in &buf {
                assert!(v.is_finite(), "{id}: non-finite sample");
                peak = peak.max(v.abs());
            }
        }
        assert!(peak <= 0.892, "{id}: peak {peak} above the -1 dBFS ceiling");
        assert!(s.limiter_clamped() < 50, "{id}: limiter safety knee engaged {} times", s.limiter_clamped());
    }
}

#[test]
fn camera_cut_has_no_click() {
    // Outside → far tower position → cockpit: the output must not jump.
    let f0 = frame_for("aerolink");
    let mut s = Synth::new(SR);
    let mut buf = vec![0.0f32; 1600];
    let mut x = Vec::new();
    for k in 0..300 {
        let mut f = f0;
        f.ac_pos = Vec3::new(0.0, 100.0, 0.0);
        f.ear_pos = match k {
            0..=99 => Vec3::new(0.0, 104.0, 40.0),
            100..=199 => Vec3::new(600.0, 30.0, 0.0),
            _ => Vec3::new(0.0, 101.0, 4.0),
        };
        f.interior = if k >= 200 { 1.0 } else { 0.0 };
        f.cut = k == 0 || k == 100 || k == 200;
        s.push_frame(&f);
        s.render(&mut buf);
        x.extend(buf.chunks(2).map(|c| c[0]));
    }
    save("cuts.wav", &x);
    // Largest sample-to-sample step around each cut vs. the steady signal.
    let steady = x[(0.8 * SR) as usize..(1.6 * SR) as usize].windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
    for cut in [100usize, 200] {
        let i = (cut as f32 / 60.0 * SR) as usize;
        let around = x[i - 2000..i + 6000].windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
        assert!(around < steady * 2.5 + 0.02, "cut at frame {cut}: step {around} vs steady {steady}");
    }
}

/// Opens a real cpal stream on the ALSA "null" sink (no sound reaches the
/// speakers) and checks the callback runs and keeps up. Linux only;
/// `cargo test audio -- --ignored`.
#[test]
#[ignore]
fn null_device_stream_runs() {
    use super::output::{Output, Shared};
    use std::sync::{Arc, Mutex};
    // SAFETY: single-threaded test setup before any audio thread exists.
    unsafe { std::env::set_var("SKYWARD_AUDIO_DEVICE", std::env::var("SKYWARD_TEST_DEVICE").unwrap_or_else(|_| "null".into())) };
    let mut f = frame_for("meridian320");
    f.ear_pos = Vec3::new(0.0, 2.0, 30.0);
    let shared = Arc::new(Mutex::new(Shared { frame: f, seq: 1, events: Vec::new(), settings: super::Settings::default(), cpu_ns: 0, cpu_audio_s: 0.0 }));
    let out = Output::open(shared.clone()).expect("open null device");
    eprintln!("opened {} @ {} Hz", out.name, out.sample_rate);
    for i in 0..20 {
        std::thread::sleep(std::time::Duration::from_millis(100));
        let mut sh = shared.lock().unwrap();
        sh.seq += 1;
        sh.frame.eng[0].spool = 0.3 + 0.03 * i as f32;
        if i == 10 {
            sh.events.push(Event::Speak(Clip::Ft100));
        }
    }
    let sh = shared.lock().unwrap();
    eprintln!("rendered {:.2} s of audio, {:.3} ms CPU per 10 ms", sh.cpu_audio_s, sh.cpu_ns as f64 / 1e6 / (sh.cpu_audio_s * 100.0));
    assert!(sh.cpu_audio_s > 1.0, "callback did not run");
    assert!(!out.failed());
}

#[test]
fn switching_aircraft_is_cheap_on_the_audio_thread() {
    // Voices (incl. their band-limited wavetables) are built inside the
    // callback when the aircraft changes: keep that well under a buffer.
    let mut s = Synth::new(SR);
    let mut buf = vec![0.0f32; 512];
    for id in ["kestrel", "peregrine", "aerolink", "lancer", "meridian190", "meridian320"] {
        let f = frame_for(id);
        let t = std::time::Instant::now();
        s.push_frame(&f);
        s.render(&mut buf);
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        eprintln!("{id}: voice set-up + first block {ms:.2} ms");
        assert!(ms < if cfg!(debug_assertions) { 80.0 } else { 8.0 }, "{id}: {ms} ms");
    }
}
