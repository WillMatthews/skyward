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
    // The procedural voice (its fan tone is exact); the recorded set has its
    // own Doppler test below.
    f.profile.set = SoundSet::None;
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
    for id in ["kestrel", "swift", "peregrine", "lancer", "aerolink", "meridian190", "meridian320", "hercules", "galaxy", "vulcan", "ec130", "fpv"] {
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
                for e in [Event::Crash { speed: 80.0, vel: Vec3::new(70.0, -30.0, 0.0), mass: 20_000.0, surface: Surface::Ground, in_air: false }, Event::Boom(1.0), Event::Thunder(1.0), Event::AbLight(0), Event::Touchdown { sink: 4.0, speed: 70.0 }] {
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
    for id in ["kestrel", "peregrine", "aerolink", "lancer", "meridian190", "meridian320", "hercules", "galaxy", "vulcan"] {
        let f = frame_for(id);
        let t = std::time::Instant::now();
        s.push_frame(&f);
        s.render(&mut buf);
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        eprintln!("{id}: voice set-up + first block {ms:.2} ms");
        assert!(ms < if cfg!(debug_assertions) { 80.0 } else { 8.0 }, "{id}: {ms} ms");
    }
}

// ------------------------------------------------------------ recorded sets

fn prepare_all() {
    let all: Vec<&'static str> = super::samples::NAMES.to_vec();
    super::samples::prepare_blocking(&all);
}

/// Isolated-spike count (second difference vs its local RMS), as in render.rs.
fn spikes(x: &[f32]) -> usize {
    let n = x.len();
    if n < 3000 {
        return 0;
    }
    let d2: Vec<f32> = (2..n).map(|i| x[i] - 2.0 * x[i - 1] + x[i - 2]).collect();
    let w = (0.02 * SR) as usize;
    let mut acc = 0.0f64;
    let mut hits = 0;
    for i in 0..d2.len() {
        acc += (d2[i] * d2[i]) as f64;
        if i >= w {
            acc -= (d2[i - w] * d2[i - w]) as f64;
            let rms = (acc.max(0.0) / w as f64).sqrt() as f32;
            if d2[i].abs() > 0.03 && d2[i].abs() > 14.0 * rms.max(1e-6) {
                hits += 1;
            }
        }
    }
    hits
}

#[test]
fn every_sample_decodes_with_seamless_loops() {
    let t = std::time::Instant::now();
    prepare_all();
    let mb: usize = (0..super::samples::NAMES.len()).filter_map(super::samples::pcm).map(|p| p.data.len() * 2).sum();
    eprintln!("decoded all samples in {:.0} ms, {:.1} MB", t.elapsed().as_secs_f64() * 1000.0, mb as f64 / 1e6);
    for (i, name) in super::samples::NAMES.iter().enumerate() {
        let p = super::samples::pcm(i).unwrap_or_else(|| panic!("{name} not decoded"));
        assert!(p.len > 1000, "{name}: {} frames", p.len);
        let d: Vec<f32> = p.data[super::samples::PAD..super::samples::PAD + p.len].iter().map(|v| *v as f32 * p.scale).collect();
        assert!(d.iter().all(|v| v.is_finite() && v.abs() < 1.5), "{name}: bad samples");
        let rms = (d.iter().map(|v| v * v).sum::<f32>() / d.len() as f32).sqrt();
        assert!(rms > 1e-3, "{name}: silent");
        if p.looped {
            // The wrap (last → first) must look like any other step.
            let step = (d[0] - d[p.len - 1]).abs();
            let typical = d.windows(2).map(|w| (w[1] - w[0]).abs()).sum::<f32>() / p.len as f32;
            assert!(step < 8.0 * typical + 0.02, "{name}: loop seam step {step} vs typical {typical}");
        }
    }
}

/// Run one engine type through idle → max → idle (exterior, then cockpit);
/// returns mono output.
fn runup(id: &str, interior: f32) -> Vec<f32> {
    prepare_all();
    let mut f = frame_for(id);
    f.on_ground = true;
    f.day = 0.0;
    f.ear_agl = 300.0; // no ambience birds / city
    f.interior = interior;
    let def = crate::flight::find(id).unwrap();
    let mut s = Synth::new(SR);
    let mut buf = vec![0.0f32; 1600];
    let mut out = Vec::new();
    for k in 0..(14.0 * 60.0) as usize {
        let t = k as f32 / 60.0;
        let pw = if t < 2.0 { 0.0 } else if t < 7.0 { (t - 2.0) / 5.0 } else if t < 9.0 { 1.0 } else if t < 12.0 { 1.0 - (t - 9.0) / 3.0 } else { 0.0 };
        let p = f.profile;
        for i in 0..p.engines {
            let e = &mut f.eng[i];
            match p.family {
                Family::Airliner | Family::Fighter => {
                    let n1 = def.n1_idle + (1.0 - def.n1_idle) * pw;
                    e.spool = n1;
                    e.rpm = n1 * p.n1_rpm;
                }
                _ => {
                    e.spool = pw;
                    e.rpm = 800.0 + 1900.0 * pw;
                }
            }
            e.pos = Vec3::new(0.0, 2.0, -20.0);
        }
        f.ac_pos = Vec3::new(0.0, 2.0, -20.0);
        f.cut = k == 0;
        s.push_frame(&f);
        s.render(&mut buf);
        out.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
    }
    out
}

#[test]
fn recorded_sets_have_no_clicks_through_the_power_range() {
    for id in ["kestrel", "peregrine", "lancer", "aerolink", "meridian190", "meridian320", "vulcan"] {
        for interior in [0.0, 1.0] {
            let x = runup(id, interior);
            save(&format!("runup_{id}_{interior}.wav"), &x);
            let peak = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert!(peak < 0.9 && peak > 0.01, "{id}/{interior}: peak {peak}");
            // Skip the first 0.3 s (fade-in).
            let n = spikes(&x[(0.3 * SR) as usize..]);
            assert!(n == 0, "{id}/{interior}: {n} isolated spikes");
        }
    }
}

#[test]
fn recorded_piston_pitch_tracks_rpm() {
    // C172 run loop: firing fundamental 81.1 Hz at 2433 rpm. At 2000 and
    // 2600 rpm the dominant line in 50..120 Hz must sit at rpm / 30.
    prepare_all();
    for rpm in [2100.0f32, 2600.0] {
        let mut f = frame_for("kestrel");
        f.day = 0.0;
        f.ear_agl = 300.0;
        f.eng[0] = EngineFrame { rpm, spool: 0.9, thrust: 0.8, running: true, throttle: 0.9, pos: Vec3::new(0.0, 2.0, -20.0), ..Default::default() };
        f.ac_pos = Vec3::new(0.0, 2.0, -20.0);
        // Procedural detail off: the samples alone must carry the pitch.
        let mut s = Synth::new(SR);
        let mut buf = vec![0.0f32; 1600];
        let mut out = Vec::new();
        for k in 0..180 {
            f.cut = k == 0;
            s.push_frame(&f);
            s.render(&mut buf);
            out.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
        }
        let seg = &out[(1.0 * SR) as usize..(2.9 * SR) as usize];
        let want = rpm / 30.0;
        let got = peak_freq(seg, want * 0.8, want * 1.2);
        assert!((got / want - 1.0).abs() < 0.03, "{rpm} rpm: {got} Hz vs {want}");
    }
}

/// F-16 passes over a static listener at Mach `m`, height `h`; returns
/// (mono output, synth).
fn supersonic_pass(m: f32, h: f32, interior: f32) -> (Vec<f32>, Synth) {
    prepare_all();
    let mut f = frame_for("lancer");
    f.day = 0.0;
    f.ear_agl = 1.7;
    f.on_ground = false;
    f.interior = interior;
    let v = m * 340.0;
    let secs = 13.0;
    let mut s = Synth::new(SR);
    let mut buf = vec![0.0f32; 1600];
    let mut out = Vec::new();
    for k in 0..(secs * 60.0) as usize {
        let t = k as f32 / 60.0;
        let pos = Vec3::new(v * (t - 5.0), h, 0.0);
        f.ac_pos = pos;
        f.ac_vel = Vec3::new(v, 0.0, 0.0);
        f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        f.tas = v;
        f.mach = m;
        for e in f.eng.iter_mut() {
            e.pos = pos;
            e.ab = 1.0;
        }
        f.ear_pos = if interior > 0.5 { pos } else { Vec3::new(0.0, 1.7, 0.0) };
        f.ear_vel = if interior > 0.5 { f.ac_vel } else { Vec3::ZERO };
        f.cut = k == 0;
        s.push_frame(&f);
        s.render(&mut buf);
        out.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
    }
    (out, s)
}

#[test]
fn sonic_boom_arrives_with_the_mach_cone() {
    for (m, h) in [(1.3f32, 300.0f32), (1.6, 2000.0)] {
        let (x, s) = supersonic_pass(m, h, 0.0);
        save(&format!("boom_{m}_{h}.wav"), &x);
        assert_eq!(s.booms, 1, "M{m} {h} m: {} booms", s.booms);
        // Cone geometry: overhead at t = 5 s; the cone reaches the ground
        // h / tan(μ) later, μ = asin(1/M).
        // (The synth's speed of sound is 343 m/s; the pass is at m·340.)
        let mu = (343.0 / (m * 340.0)).asin();
        let t_cone = 5.0 + (h - 1.7) / mu.tan() / (m * 340.0);
        let i_cone = (t_cone * SR) as usize;
        // Silence before (only the pre-boom block may hold the fade).
        let pre = &x[(1.0 * SR) as usize..i_cone - (0.15 * SR) as usize];
        let pre_rms = (pre.iter().map(|v| v * v).sum::<f32>() / pre.len() as f32).sqrt();
        // (Only the listener's own ambience, ~2e-3, before the cone.)
        assert!(pre_rms < 3.5e-3, "M{m}: heard {pre_rms} before the cone");
        // The first shock within ~2 blocks + 1 frame of the geometric time.
        let first = x.iter().position(|v| v.abs() > 0.02).unwrap();
        let dt = first as f32 / SR - t_cone;
        eprintln!("M{m} {h} m: boom at {:.3} s, cone {t_cone:.3} s, N-wave {:.3} s, {:.0} Pa", first as f32 / SR, s.last_boom.0, s.last_boom.1);
        assert!(dt.abs() < 0.012, "M{m} {h} m: boom at {:.3} s, cone at {t_cone:.3} s", first as f32 / SR);
        // N-wave length as modelled, and the engine roar afterwards.
        let (len, dp) = s.last_boom;
        assert!(len > 0.012 && len < 0.45 && dp > 1.0, "len {len} dp {dp}");
        let post = &x[i_cone + (0.6 * SR) as usize..i_cone + (1.6 * SR) as usize];
        let post_rms = (post.iter().map(|v| v * v).sum::<f32>() / post.len() as f32).sqrt();
        let k = if h < 1000.0 { 5.0 } else { 1.3 };
        assert!(post_rms > k * pre_rms.max(1e-4), "M{m}: no roar after the boom ({post_rms} vs {pre_rms})");
        assert!(s.limiter_clamped() < 50);
    }
}

#[test]
fn no_boom_in_the_cockpit_or_subsonic() {
    let (_, s) = supersonic_pass(1.4, 500.0, 1.0);
    assert_eq!(s.booms, 0, "cockpit");
    let (_, s) = supersonic_pass(0.95, 300.0, 0.0);
    assert_eq!(s.booms, 0, "subsonic");
}

#[test]
fn crashes_and_debris_events_are_finite_and_scaled() {
    prepare_all();
    let mut levels = Vec::new();
    for (id, speed) in [("kestrel", 35.0f32), ("lancer", 150.0), ("meridian320", 75.0)] {
        for surface in [Surface::Ground, Surface::Paved, Surface::Water, Surface::Building] {
            for interior in [0.0, 1.0] {
                let mut f = frame_for(id);
                f.day = 0.0;
                f.ear_agl = 300.0;
                f.interior = interior;
                f.ac_pos = Vec3::new(0.0, 0.0, -40.0);
                f.ear_pos = if interior > 0.5 { f.ac_pos } else { Vec3::ZERO };
                let mut s = Synth::new(SR);
                let mut buf = vec![0.0f32; 1600];
                let mut x = Vec::new();
                let def = crate::flight::find(id).unwrap();
                for k in 0..(6.0 * 60.0) as usize {
                    f.crashed = k >= 30;
                    for e in f.eng.iter_mut() {
                        e.running = !f.crashed;
                    }
                    f.cut = k == 0;
                    s.push_frame(&f);
                    if k == 30 {
                        s.event(&Event::Crash { speed, vel: Vec3::new(speed * 0.95, -speed * 0.3, 0.0), mass: def.mass, surface, in_air: false });
                    }
                    s.render(&mut buf);
                    x.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
                }
                assert!(x.iter().all(|v| v.is_finite()), "{id} {surface:?}");
                let peak = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
                assert!(peak <= 0.892, "{id}: peak {peak}");
                let seg = &x[(0.5 * SR) as usize..(2.5 * SR) as usize];
                let rms = (seg.iter().map(|v| v * v).sum::<f32>() / seg.len() as f32).sqrt();
                if interior > 0.5 {
                    // Fatal impacts cut to (ringing) near-silence.
                    let tail = &x[(4.0 * SR) as usize..];
                    let tr = (tail.iter().map(|v| v * v).sum::<f32>() / tail.len() as f32).sqrt();
                    assert!(tr < 0.05 * rms + 0.02, "{id} cockpit tail {tr} vs impact {rms}");
                } else if surface == Surface::Ground {
                    levels.push((id, rms));
                }
            }
        }
    }
    // Bigger, faster aircraft crash louder.
    assert!(levels[0].1 < levels[2].1, "{levels:?}");
}

#[test]
fn debris_event_api_is_positional() {
    prepare_all();
    let level = |dist: f32| {
        let mut f = frame_for("kestrel");
        f.day = 0.0;
        f.ear_agl = 300.0;
        f.eng = [EngineFrame::default(); MAX_ENG];
        let mut s = Synth::new(SR);
        let mut buf = vec![0.0f32; 1600];
        let mut x = Vec::new();
        let p = Vec3::new(dist, 0.0, 0.0);
        for k in 0..(4.0 * 60.0) as usize {
            f.cut = k == 0;
            s.push_frame(&f);
            if k == 10 {
                s.event(&Event::Impact { pos: p, speed: 20.0, mass: 300.0, surface: Surface::Ground });
                s.event(&Event::Separation { pos: p, mass: 200.0 });
                s.event(&Event::Explosion { pos: p, energy: 0.5 });
                s.event(&Event::Fire { id: 1, pos: p, size: 0.6 });
            }
            if (10..120).contains(&k) {
                s.event(&Event::Scrape { id: 2, pos: p, speed: 15.0, mass: 500.0, surface: Surface::Paved });
            }
            s.render(&mut buf);
            x.extend(buf.chunks(2).map(|c| 0.5 * (c[0] + c[1])));
        }
        assert!(x.iter().all(|v| v.is_finite()));
        // Sound arrives after the propagation delay.
        let first = x.iter().position(|v| v.abs() > 1e-3).unwrap_or(0) as f32 / SR;
        let rms = (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt();
        (first, rms)
    };
    let (t_near, near) = level(20.0);
    let (t_far, far) = level(600.0);
    assert!(near > 3.0 * far, "near {near} far {far}");
    assert!(t_far - t_near > 1.5, "delay near {t_near} far {t_far}");
}

// ------------------------------------------------------------------ rotorcraft

/// Stereo → mono.
fn mono(x: &[f32]) -> Vec<f32> {
    x.chunks(2).map(|c| 0.5 * (c[0] + c[1])).collect()
}

fn seg(x: &[f32], t0: f32, t1: f32) -> &[f32] {
    &x[(t0 * SR) as usize..((t1 * SR) as usize).min(x.len())]
}

fn rms(x: &[f32]) -> f32 {
    (x.iter().map(|v| v * v).sum::<f32>() / x.len().max(1) as f32).sqrt()
}

/// Energy in [lo, hi] Hz (Hann-windowed DFT, 2 Hz steps).
fn band(x: &[f32], lo: f32, hi: f32) -> f32 {
    let n = x.len();
    let mut e = 0.0f64;
    let mut f = lo;
    while f < hi {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (i, v) in x.iter().enumerate().step_by(2) {
            let w = 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / n as f32).cos();
            let a = std::f64::consts::TAU * f as f64 * i as f64 / SR as f64;
            re += (v * w) as f64 * a.cos();
            im += (v * w) as f64 * a.sin();
        }
        e += re * re + im * im;
        f += 2.0;
    }
    e as f32
}

/// Hovering EC130 heard from 30 m: optional slap, engine lit or not.
fn heli_hover(slap: f32, lit: bool, nr: f32, secs: f32) -> Vec<f32> {
    let mut f = frame_for("ec130");
    let mut s = Synth::new(SR);
    let mut out = Vec::new();
    let mut buf = vec![0.0f32; 1600];
    f.ac_pos = Vec3::new(0.0, 15.0, -30.0);
    f.eng[0] = EngineFrame { pos: f.ac_pos + Vec3::new(0.0, 1.0, 1.4), rpm: 386.0 * nr, spool: if lit { 0.9 } else { 0.0 }, running: lit, ..Default::default() };
    f.rotor = RotorFrame { nr_rpm: 386.0 * nr, nr, ng: if lit { 0.9 } else { 0.0 }, torque: if lit { 0.75 } else { 0.0 }, power: if lit { 0.75 } else { 0.0 }, tail_rpm: 3568.0 * nr, tail_pitch: 16.0, slap, load: 1.0, ..Default::default() };
    for k in 0..(secs * 60.0) as usize {
        f.cut = k == 0;
        s.push_frame(&f);
        s.render(&mut buf);
        out.extend_from_slice(&buf);
    }
    mono(&out)
}

#[test]
fn heli_rotor_and_fenestron_tones() {
    let x = heli_hover(0.0, true, 1.0, 3.0);
    save("heli_hover.wav", &x);
    let y = seg(&x, 1.0, 3.0);
    // Main-rotor blade passage: 3 × 386 rpm = 19.3 Hz.
    let bp = peak_freq(y, 12.0, 30.0);
    assert!((bp - 19.3).abs() < 1.5, "blade passage {bp} Hz");
    // Fenestron: 10 (unevenly spaced) blades at 3,568 rpm → energy at shaft
    // orders around the 595 Hz blade passage.
    let ft = peak_freq(seg(&x, 1.0, 1.5), 480.0, 720.0);
    let order = ft / (3568.0 / 60.0);
    assert!((order - order.round()).abs() < 0.08 && (8.0..=12.0).contains(&order.round()), "fenestron {ft} Hz (order {order})");
}

#[test]
fn heli_blade_slap_is_impulsive() {
    let calm = heli_hover(0.0, true, 1.0, 3.0);
    let slap = heli_hover(1.0, true, 1.0, 3.0);
    save("heli_slap.wav", &slap);
    let crest = |x: &[f32]| x.iter().fold(0.0f32, |m, v| m.max(v.abs())) / rms(x).max(1e-9);
    let (c0, c1) = (crest(seg(&calm, 1.0, 3.0)), crest(seg(&slap, 1.0, 3.0)));
    // BVI: sharp pulses with energy up into the kHz range.
    let (h0, h1) = (band(seg(&calm, 1.0, 1.5), 800.0, 2400.0), band(seg(&slap, 1.0, 1.5), 800.0, 2400.0));
    assert!(c1 > 1.3 * c0, "crest {c0} → {c1}");
    assert!(h1 > 3.0 * h0, "0.8–2.4 kHz energy {h0} → {h1}");
}

#[test]
fn heli_rotor_runs_on_after_engine_failure() {
    let lit = heli_hover(0.0, true, 1.0, 3.0);
    let dead = heli_hover(0.0, false, 0.95, 3.0);
    let (a, b) = (seg(&lit, 1.0, 1.5), seg(&dead, 1.0, 1.5));
    // Rotor thump still there; the gas generator / combustion gone.
    let (r0, r1) = (band(a, 14.0, 70.0), band(b, 14.0, 70.0));
    // Compressor tone: 4.6 × the gas-generator shaft (52,110 rpm at 100 %): ≈ 3.6 kHz at 90 % Ng.
    let (w0, w1) = (band(a, 3500.0, 3700.0), band(b, 3500.0, 3700.0));
    assert!(r1 > 0.5 * r0, "rotor {r0} → {r1}");
    assert!(w1 < 0.25 * w0, "turbine whine {w0} → {w1}");
    for v in &dead {
        assert!(v.is_finite());
    }
}

#[test]
fn heli_scenarios_are_clean() {
    for (name, x) in [("approach_slap", super::scenarios::heli_approach_slap()), ("autorotation", super::scenarios::heli_autorotation(true)), ("startup", super::scenarios::heli_startup())] {
        let m = mono(&x);
        save(&format!("heli_{name}.wav"), &m);
        assert!(m.iter().all(|v| v.is_finite() && v.abs() <= 1.0), "{name}");
        assert!(rms(&m) > 1e-3, "{name} silent");
    }
}

#[test]
fn drone_motor_tones_follow_speed() {
    let x = mono(&super::scenarios::drone_flight());
    save("drone_flight.wav", &x);
    // Hover (0.34 of 3,300 rad/s ≈ 10.7k rpm): tri-blade passage ≈ 535 Hz;
    // punch-out (1.0): ≈ 1.58 kHz.
    let hover = peak_freq(seg(&x, 2.5, 3.0), 450.0, 650.0);
    let punch = peak_freq(seg(&x, 5.0, 5.5), 1300.0, 1800.0);
    let bpf = |m: f32| m * 3300.0 / std::f32::consts::TAU * 3.0;
    assert!((hover / bpf(0.355) - 1.0).abs() < 0.06, "hover {hover} Hz vs {}", bpf(0.355));
    assert!((punch / bpf(1.0) - 1.0).abs() < 0.04, "punch {punch} Hz vs {}", bpf(1.0));
    // Punch-out is much louder than the hover.
    assert!(rms(seg(&x, 5.0, 5.5)) > 2.0 * rms(seg(&x, 2.5, 3.0)));
    assert!(x.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
}
