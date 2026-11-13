//! Scripted offline scenarios (synthetic frames, no sim): engine run-ups in
//! chase and cockpit views, tower fly-bys, crashes, supersonic passes.
//! `SKYWARD_AUDIO_SCENARIOS=/dir cargo test --release audio::scenarios -- --ignored --nocapture`
//! writes `<dir>/<aircraft>_<scenario>.wav` (stereo float) for listening and
//! analysis — the same script runs against any version of the synth (A/B).

use super::capture::profile_for;
use super::params::*;
use super::dsp::smoothstep;
use super::synth::Synth;
use glam::{Quat, Vec3};

const SR: f32 = 48_000.0;
const FPS: f32 = 60.0;

struct Plane {
    id: &'static str,
    /// (idle, max) gauge rpm (props) — jets use N1.
    rpm: (f32, f32),
    cruise: f32,
    crash_speed: f32,
}

const PLANES: &[Plane] = &[
    Plane {
        id: "kestrel",
        rpm: (750.0, 2650.0),
        cruise: 55.0,
        crash_speed: 35.0,
    },
    Plane {
        id: "peregrine",
        rpm: (800.0, 2900.0),
        cruise: 130.0,
        crash_speed: 75.0,
    },
    Plane {
        id: "lancer",
        rpm: (0.0, 0.0),
        cruise: 250.0,
        crash_speed: 150.0,
    },
    Plane {
        id: "aerolink",
        rpm: (1000.0, 1200.0),
        cruise: 120.0,
        crash_speed: 60.0,
    },
    Plane {
        id: "meridian190",
        rpm: (0.0, 0.0),
        cruise: 140.0,
        crash_speed: 75.0,
    },
    Plane {
        id: "meridian320",
        rpm: (0.0, 0.0),
        cruise: 140.0,
        crash_speed: 75.0,
    },
    Plane {
        id: "hercules",
        rpm: (730.0, 1020.0),
        cruise: 110.0,
        crash_speed: 60.0,
    },
    Plane {
        id: "galaxy",
        rpm: (0.0, 0.0),
        cruise: 140.0,
        crash_speed: 75.0,
    },
    Plane {
        id: "vulcan",
        rpm: (0.0, 0.0),
        cruise: 150.0,
        crash_speed: 80.0,
    },
];

fn base_frame(id: &str) -> Frame {
    let def = crate::flight::find(id).expect("aircraft");
    let p = profile_for(&def, 18.0);
    let mut f = Frame {
        active: true,
        warp: 1.0,
        profile: p,
        interior: 0.0,
        day: 1.0,
        ..Frame::default()
    };
    f.ear_right = Vec3::X;
    f.ear_up = Vec3::Y;
    f.ear_fwd = Vec3::NEG_Z;
    f.ear_agl = 1.7;
    f.gear = 1.0;
    f.paved = true;
    f
}

/// Set every engine to power `pw` (0..1), afterburner `ab`.
fn set_power(f: &mut Frame, pl: &Plane, pw: f32, ab: f32, running: bool) {
    let def = crate::flight::find(pl.id).unwrap();
    let p = f.profile;
    for i in 0..p.engines {
        let e = &mut f.eng[i];
        e.running = running;
        e.throttle = pw;
        e.ab = ab;
        match p.family {
            Family::Airliner | Family::Fighter => {
                let n1 = def.n1_idle + (1.0 - def.n1_idle) * pw.powf(0.8);
                e.spool = n1;
                e.rpm = n1 * p.n1_rpm;
                e.thrust = pw;
            }
            _ => {
                e.spool = pw;
                e.rpm = pl.rpm.0 + (pl.rpm.1 - pl.rpm.0) * pw.powf(0.8);
                e.thrust = pw;
            }
        }
    }
}

fn place_engines(f: &mut Frame, id: &str) {
    let def = crate::flight::find(id).unwrap();
    for i in 0..f.profile.engines {
        let off = def.engine_pos.get(i).copied().unwrap_or(Vec3::ZERO);
        f.eng[i].pos = f.ac_pos + f.ac_rot * off;
    }
}

struct Rec {
    s: Synth,
    out: Vec<f32>,
    buf: Vec<f32>,
}

impl Rec {
    fn new() -> Rec {
        Rec {
            s: Synth::new(SR),
            out: Vec::new(),
            buf: vec![0.0; (SR / FPS) as usize * 2],
        }
    }
    fn step(&mut self, f: &Frame) {
        self.s.push_frame(f);
        self.s.render(&mut self.buf);
        self.out.extend_from_slice(&self.buf);
    }
    fn save(&self, dir: &std::path::Path, name: &str) {
        let spec = hound::WavSpec {
            channels: 2,
            sample_rate: SR as u32,
            bits_per_sample: 32,
            sample_format: hound::SampleFormat::Float,
        };
        let mut w = hound::WavWriter::create(dir.join(format!("{name}.wav")), spec).unwrap();
        for v in &self.out {
            w.write_sample(*v).unwrap();
        }
        w.finalize().unwrap();
    }
}

/// Engine run-up on the ground: idle → max → idle, chase view then cockpit.
fn runup(pl: &Plane) -> Rec {
    let mut r = Rec::new();
    let mut f = base_frame(pl.id);
    compat::prepare(&f);
    f.on_ground = true;
    f.ac_pos = Vec3::new(0.0, 2.0, 0.0);
    f.ac_rot = Quat::IDENTITY; // nose along +Z
    let fighter = f.profile.family == Family::Fighter;
    let total = 40.0;
    for k in 0..(total * FPS) as usize {
        let t = k as f32 / FPS;
        let tt = t % 20.0;
        // 0-4 idle, 4-10 ramp up, 10-15 max (AB 12-15 for fighters), 15-17 ramp down, 17-20 idle
        let pw = if tt < 4.0 {
            0.0
        } else if tt < 10.0 {
            (tt - 4.0) / 6.0
        } else if tt < 15.0 {
            1.0
        } else if tt < 17.0 {
            1.0 - (tt - 15.0) / 2.0
        } else {
            0.0
        };
        let ab = if fighter && (12.0..15.0).contains(&tt) {
            1.0
        } else {
            0.0
        };
        set_power(&mut f, pl, pw, ab, true);
        let inside = t >= 20.0;
        f.interior = if inside { 1.0 } else { 0.0 };
        let refd = f.profile.size * 2.6 + 6.0;
        // Chase camera: behind and a little to the side and above.
        f.ear_pos = if inside {
            f.ac_pos + Vec3::new(0.0, 1.0, 2.0)
        } else {
            f.ac_pos + Vec3::new(refd * 0.35, refd * 0.25, -refd * 0.9)
        };
        f.ear_fwd = (f.ac_pos - f.ear_pos).normalize_or(Vec3::Z);
        f.ear_right = f.ear_fwd.cross(Vec3::Y).normalize_or(Vec3::X);
        f.cut = k == 0 || k == (20.0 * FPS) as usize;
        place_engines(&mut f, pl.id);
        r.step(&f);
    }
    r
}

/// A fly-by past a tower listener: level at `alt` m, `speed` m/s, passing
/// `offset` m to the side at t = secs/2.
#[allow(clippy::too_many_arguments)]
fn flyby(
    pl: &Plane,
    speed: f32,
    alt: f32,
    offset: f32,
    secs: f32,
    pw: f32,
    ab: f32,
    mach: f32,
) -> Rec {
    let mut r = Rec::new();
    let mut f = base_frame(pl.id);
    compat::prepare(&f);
    f.on_ground = false;
    f.gear = 0.0;
    f.ear_pos = Vec3::new(offset, 1.7, 0.0);
    f.ear_fwd = Vec3::NEG_X;
    f.ear_right = Vec3::NEG_Z;
    f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2); // nose along +X
    let x0 = -speed * secs * 0.5;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        f.ac_pos = Vec3::new(x0 + speed * t, alt, 0.0);
        f.ac_vel = Vec3::new(speed, 0.0, 0.0);
        f.tas = speed;
        f.mach = mach;
        set_power(&mut f, pl, pw, ab, true);
        f.cut = k == 0;
        place_engines(&mut f, pl.id);
        r.step(&f);
    }
    r
}

/// Cockpit (or chase) while accelerating through Mach 1.
fn transonic(pl: &Plane, chase: bool) -> Rec {
    let mut r = Rec::new();
    let mut f = base_frame(pl.id);
    compat::prepare(&f);
    f.on_ground = false;
    f.gear = 0.0;
    f.interior = if chase { 0.0 } else { 1.0 };
    f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let secs = 30.0;
    let mut x = 0.0f32;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        // M 0.8 → 1.4 over 24 s, hold.
        let m = 0.8 + 0.6 * (t / 24.0).min(1.0);
        let v = m * 300.0; // a ≈ 300 m/s at altitude
        x += v / FPS;
        f.ac_pos = Vec3::new(x, 9000.0, 0.0);
        f.ac_vel = Vec3::new(v, 0.0, 0.0);
        f.tas = v;
        f.mach = m;
        set_power(&mut f, pl, 1.0, 1.0, true);
        let refd = f.profile.size * 2.6 + 6.0;
        f.ear_pos = if chase {
            f.ac_pos + Vec3::new(-refd, refd * 0.2, refd * 0.3)
        } else {
            f.ac_pos + Vec3::new(2.0, 1.0, 0.0)
        };
        f.ear_vel = f.ac_vel;
        f.ear_fwd = Vec3::X;
        f.ear_right = Vec3::Z;
        f.cut = k == 0;
        place_engines(&mut f, pl.id);
        r.step(&f);
    }
    r
}

/// Fly into the ground at `crash_speed` (shallow, 15°), chase or cockpit.
fn crash(pl: &Plane, cockpit: bool) -> Rec {
    let mut r = Rec::new();
    let mut f = base_frame(pl.id);
    compat::prepare(&f);
    f.on_ground = false;
    f.gear = 0.0;
    f.interior = if cockpit { 1.0 } else { 0.0 };
    let v = pl.crash_speed;
    let dir = Vec3::new(0.97, -0.26, 0.0);
    f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let secs = 12.0;
    let t_hit = 3.0;
    let p_hit = Vec3::new(0.0, 0.0, 0.0);
    let refd = f.profile.size * 2.6 + 6.0;
    let mut ear = p_hit - dir * (v * t_hit) + Vec3::new(-refd * 0.9, refd * 0.3, refd * 0.3);
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        let crashed = t >= t_hit;
        if !crashed {
            f.ac_pos = p_hit + dir * v * (t - t_hit);
            f.ac_vel = dir * v;
            ear += dir * v / FPS;
        } else {
            f.ac_vel = Vec3::ZERO;
        }
        f.tas = if crashed { 0.0 } else { v };
        set_power(&mut f, pl, if crashed { 0.0 } else { 0.7 }, 0.0, !crashed);
        f.crashed = crashed;
        f.ear_pos = if cockpit {
            f.ac_pos + Vec3::new(1.0, 1.0, 0.0)
        } else {
            ear
        };
        f.ear_vel = if crashed { Vec3::ZERO } else { f.ac_vel };
        f.ear_fwd = Vec3::X;
        f.ear_right = Vec3::Z;
        f.cut = k == 0;
        place_engines(&mut f, pl.id);
        r.s.push_frame(&f);
        if k == (t_hit * FPS) as usize {
            let def = crate::flight::find(pl.id).unwrap();
            r.s.event(&compat::crash_event(v, dir * v, def.mass));
        }
        r.s.render(&mut r.buf);
        r.out.extend_from_slice(&r.buf);
    }
    r
}

// ------------------------------------------------------------------ rotorcraft

/// Helicopter state for the scenarios: rotor speed fraction, Ng fraction,
/// engine lit, torque / power fraction, blade slap, rotor load.
#[allow(clippy::too_many_arguments)]
fn heli_state(f: &mut Frame, nr: f32, ng: f32, lit: bool, tq: f32, slap: f32, load: f32) {
    f.rotor = RotorFrame {
        nr_rpm: 386.0 * nr,
        nr,
        ng,
        torque: tq,
        power: tq,
        tail_rpm: 3568.0 * nr,
        tail_pitch: 4.0 + 14.0 * tq,
        slap,
        load,
        ..RotorFrame::default()
    };
    f.eng[0].running = lit;
    f.eng[0].rpm = 386.0 * nr;
    f.eng[0].spool = ng;
    f.stall_warn = nr < 0.93 && nr > 0.2 && (lit || tq == 0.0) && f.rotor.load > 0.1;
}

fn chase_ear(f: &mut Frame, inside: bool) {
    let refd = f.profile.size * 2.6 + 6.0;
    f.interior = if inside { 1.0 } else { 0.0 };
    f.ear_pos = if inside {
        f.ac_pos + Vec3::new(0.67, 0.16, 2.15)
    } else {
        f.ac_pos + Vec3::new(refd * 0.35, refd * 0.2, -refd * 0.9)
    };
    f.ear_fwd = (f.ac_pos - f.ear_pos).normalize_or(Vec3::Z);
    f.ear_right = f.ear_fwd.cross(Vec3::Y).normalize_or(Vec3::X);
}

/// EC130: battery start on the ground (starter, light-off, rotor run-up,
/// flight idle), lift into a hover. Chase view.
pub(super) fn heli_startup() -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame("ec130");
    f.on_ground = true;
    f.ac_pos = Vec3::new(0.0, 1.6, 0.0);
    f.ac_rot = Quat::IDENTITY;
    let total = 45.0;
    for k in 0..(total * FPS) as usize {
        let t = k as f32 / FPS;
        // Starter to 15 % Ng, light-off at 5 s, idle 68 % by 18 s; the rotor
        // follows once the free turbine has torque (8–30 s); collective at 36 s.
        let ng = if t < 5.0 { 0.15 * t / 5.0 } else { (0.15 + 0.53 * ((t - 5.0) / 13.0).min(1.0)).min(0.68) + 0.3 * smoothstep(30.0, 34.0, t) + 0.05 * smoothstep(36.0, 40.0, t) };
        let nr = smoothstep(7.0, 30.0, t) * 0.75 + 0.25 * smoothstep(26.0, 34.0, t);
        let tq = 0.08 + 0.1 * smoothstep(30.0, 34.0, t) + 0.6 * smoothstep(36.0, 40.0, t);
        let load = smoothstep(36.0, 40.0, t);
        heli_state(&mut f, nr, ng, t >= 5.0, tq, 0.0, load);
        f.stall_warn = false;
        f.on_ground = t < 39.0;
        f.ac_pos.y = 1.6 + 3.0 * smoothstep(39.0, 44.0, t);
        chase_ear(&mut f, false);
        f.cut = k == 0;
        place_engines(&mut f, "ec130");
        r.step(&f);
    }
    r.out
}

/// EC130 shallow approach descent past a listener on the ground ahead of it:
/// no slap (0–6 s), blade slap building (6–14 s), slap fading as it
/// arrives in the hover (14–20 s).
pub(super) fn heli_approach_slap() -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame("ec130");
    f.on_ground = false;
    f.ear_pos = Vec3::new(30.0, 1.7, 0.0);
    f.ear_fwd = Vec3::NEG_Z;
    f.ear_right = Vec3::X;
    let secs = 20.0;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        // Coming towards the listener along −Z at 30 m/s, then slowing to a hover 80 m out.
        let v = 30.0 * (1.0 - smoothstep(12.0, 19.0, t));
        let z = 80.0 + 30.0 * (19.0 - t).max(0.0) * 0.55;
        f.ac_pos = Vec3::new(0.0, 15.0 + 4.0 * (19.0 - t).max(0.0), z);
        f.ac_vel = Vec3::new(0.0, -2.0, -v);
        f.ac_rot = Quat::from_rotation_y(std::f32::consts::PI); // nose along −Z
        f.tas = v;
        let slap = smoothstep(6.0, 9.0, t) * (1.0 - smoothstep(14.0, 17.0, t));
        heli_state(&mut f, 1.0, 0.86, true, 0.45 + 0.3 * smoothstep(14.0, 18.0, t), slap, 1.0);
        f.cut = k == 0;
        place_engines(&mut f, "ec130");
        r.step(&f);
    }
    r.out
}

/// EC130 cockpit: cruise, engine failure at 5 s (turbine winds down, the
/// rotor sound runs on, Nr dips and the low-Nr horn sounds until the
/// collective is lowered), steady autorotation, flare with slap, cushion.
pub(super) fn heli_autorotation(inside: bool) -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame("ec130");
    f.on_ground = false;
    f.ac_pos = Vec3::new(0.0, 300.0, 0.0);
    f.ac_rot = Quat::IDENTITY;
    let secs = 30.0;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        f.tas = 33.0;
        f.ac_vel = Vec3::new(0.0, if t > 5.0 { -10.0 } else { 0.0 }, 33.0);
        let dead = t >= 5.0;
        let ng = if dead { 0.88 * (-(t - 5.0) / 4.0).exp() } else { 0.88 };
        // Nr: dips to 88 % in the ~1.3 s before the collective goes down,
        // recovers to ~97 %, rises in the flare (22–25 s), bleeds in the cushion.
        let nr = if t < 5.0 {
            1.0
        } else if t < 6.3 {
            1.0 - 0.12 * (t - 5.0) / 1.3
        } else if t < 22.0 {
            0.88 + 0.09 * smoothstep(6.3, 9.0, t)
        } else if t < 25.0 {
            0.97 + 0.08 * smoothstep(22.0, 24.0, t)
        } else {
            1.05 - 0.35 * smoothstep(25.0, 27.5, t)
        };
        let slap = 0.25 * smoothstep(6.0, 8.0, t) * (1.0 - smoothstep(20.0, 21.0, t)) + 0.9 * smoothstep(22.0, 23.0, t) * (1.0 - smoothstep(25.0, 26.0, t));
        let load = if t < 22.0 { 0.95 } else if t < 25.0 { 1.5 } else { 1.2 };
        heli_state(&mut f, nr, ng, !dead, if dead { 0.0 } else { 0.55 }, slap, load);
        f.stall_warn = nr < 0.93;
        f.on_ground = t > 27.5;
        if f.on_ground {
            f.ac_vel = Vec3::ZERO;
            f.tas = 0.0;
        }
        chase_ear(&mut f, inside);
        f.cut = k == 0;
        place_engines(&mut f, "ec130");
        r.step(&f);
    }
    r.out
}

/// FPV quad: armed idle, hover, punch-out, prop-wash descent; chase view.
pub(super) fn drone_flight() -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame("fpv");
    f.on_ground = false;
    f.ac_pos = Vec3::new(0.0, 20.0, 0.0);
    f.ac_rot = Quat::IDENTITY;
    let secs = 12.0;
    let wmax = 3300.0;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        // Motor speed fraction: idle 0.12, hover 0.34, punch 1.0, wash 0.3 ± chop.
        let base = if t < 1.5 {
            0.12
        } else if t < 4.0 {
            0.34
        } else if t < 6.0 {
            1.0
        } else if t < 7.0 {
            0.25
        } else {
            0.33
        };
        let wash = smoothstep(7.0, 7.5, t) * (1.0 - smoothstep(10.5, 11.0, t));
        f.rotor.vrs = wash;
        for i in 0..4 {
            let chop = wash * 0.12 * ((t * (13.0 + i as f32 * 3.1)).sin());
            let m = (base + 0.01 * i as f32 + chop).clamp(0.05, 1.0);
            let w = m * wmax;
            f.eng[i].rpm = w * 60.0 / std::f32::consts::TAU;
            f.eng[i].spool = m;
            f.eng[i].thrust = 1.36e-6 * w * w;
            f.eng[i].running = true;
        }
        f.ac_vel = Vec3::new(0.0, if (4.0..6.0).contains(&t) { 25.0 } else if wash > 0.0 { -4.0 } else { 0.0 }, 0.0);
        f.ac_pos += f.ac_vel / FPS;
        chase_ear(&mut f, false);
        f.cut = k == 0;
        place_engines(&mut f, "fpv");
        r.step(&f);
    }
    r.out
}

/// FPV quad passing 15 m from a static listener at 35 m/s, hard on the throttle.
pub(super) fn drone_flyby() -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame("fpv");
    f.on_ground = false;
    f.ear_pos = Vec3::new(0.0, 1.7, 15.0);
    f.ear_fwd = Vec3::NEG_Z;
    f.ear_right = Vec3::X;
    f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let secs = 8.0;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        f.ac_pos = Vec3::new(-140.0 + 35.0 * t, 5.0, 0.0);
        f.ac_vel = Vec3::new(35.0, 0.0, 0.0);
        f.tas = 35.0;
        for i in 0..4 {
            let m = 0.72 + 0.02 * i as f32;
            let w = m * 3300.0;
            f.eng[i].rpm = w * 60.0 / std::f32::consts::TAU;
            f.eng[i].spool = m;
            f.eng[i].thrust = 1.36e-6 * w * w;
            f.eng[i].running = true;
        }
        f.cut = k == 0;
        place_engines(&mut f, "fpv");
        r.step(&f);
    }
    r.out
}

/// `SKYWARD_AUDIO_SCENARIOS=/dir cargo test --release rotorcraft_scenarios -- --ignored`
#[test]
#[ignore]
fn rotorcraft_scenarios() {
    let Some(dir) = std::env::var_os("SKYWARD_AUDIO_SCENARIOS") else {
        eprintln!("set SKYWARD_AUDIO_SCENARIOS=/dir");
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).unwrap();
    let save = |name: &str, x: &[f32]| {
        let spec = hound::WavSpec { channels: 2, sample_rate: SR as u32, bits_per_sample: 32, sample_format: hound::SampleFormat::Float };
        let mut w = hound::WavWriter::create(dir.join(format!("{name}.wav")), spec).unwrap();
        for v in x {
            w.write_sample(*v).unwrap();
        }
        w.finalize().unwrap();
    };
    save("ec130_startup", &heli_startup());
    save("ec130_approach_slap", &heli_approach_slap());
    save("ec130_autorotation_cockpit", &heli_autorotation(true));
    save("ec130_autorotation_chase", &heli_autorotation(false));
    let pl = Plane { id: "ec130", rpm: (0.0, 0.0), cruise: 57.0, crash_speed: 30.0 };
    save("ec130_flyby", &heli_flyby(&pl));
    save("fpv_flight", &drone_flight());
    save("fpv_flyby", &drone_flyby());
}

/// EC130 cruise fly-by (110 kt, 60 m up, 40 m to the side) of a tower listener.
fn heli_flyby(pl: &Plane) -> Vec<f32> {
    let mut r = Rec::new();
    let mut f = base_frame(pl.id);
    f.on_ground = false;
    f.ear_pos = Vec3::new(40.0, 1.7, 0.0);
    f.ear_fwd = Vec3::NEG_X;
    f.ear_right = Vec3::NEG_Z;
    f.ac_rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let secs = 16.0;
    let x0 = -pl.cruise * secs * 0.5;
    for k in 0..(secs * FPS) as usize {
        let t = k as f32 / FPS;
        f.ac_pos = Vec3::new(x0 + pl.cruise * t, 60.0, 0.0);
        f.ac_vel = Vec3::new(pl.cruise, 0.0, 0.0);
        f.tas = pl.cruise;
        heli_state(&mut f, 1.0, 0.93, true, 0.7, 0.08, 1.0);
        f.cut = k == 0;
        place_engines(&mut f, pl.id);
        r.step(&f);
    }
    r.out
}

#[test]
#[ignore]
fn render_scenarios() {
    let Some(dir) = std::env::var_os("SKYWARD_AUDIO_SCENARIOS") else {
        eprintln!("set SKYWARD_AUDIO_SCENARIOS=/dir");
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).unwrap();
    let only = std::env::var("SKYWARD_AUDIO_ONLY").unwrap_or_default();
    for pl in PLANES {
        if !only.is_empty() && !only.split(',').any(|o| o == pl.id) {
            continue;
        }
        let t0 = web_time::Instant::now();
        runup(pl).save(&dir, &format!("{}_runup", pl.id));
        flyby(pl, pl.cruise, 60.0, 40.0, 16.0, 0.8, 0.0, pl.cruise / 340.0)
            .save(&dir, &format!("{}_flyby", pl.id));
        crash(pl, false).save(&dir, &format!("{}_crash_chase", pl.id));
        crash(pl, true).save(&dir, &format!("{}_crash_cockpit", pl.id));
        if pl.id == "lancer" {
            // Supersonic: low pass (300 m, M 1.3), high pass (3 km, M 1.4).
            flyby(pl, 1.3 * 340.0, 300.0, 0.0, 16.0, 1.0, 1.0, 1.3)
                .save(&dir, "lancer_supersonic_low");
            flyby(pl, 1.4 * 330.0, 3000.0, 0.0, 30.0, 1.0, 1.0, 1.4)
                .save(&dir, "lancer_supersonic_high");
            flyby(pl, 0.95 * 340.0, 100.0, 50.0, 16.0, 1.0, 1.0, 0.95)
                .save(&dir, "lancer_transonic_flyby");
            transonic(pl, false).save(&dir, "lancer_mach_cockpit");
            transonic(pl, true).save(&dir, "lancer_mach_chase");
        }
        eprintln!("{}: {:.1} s", pl.id, t0.elapsed().as_secs_f32());
    }
}

mod compat {
    use super::*;
    /// Decode the recorded set before rendering (deterministic).
    pub fn prepare(f: &Frame) {
        if let Some(set) = super::super::soundsets::set(f.profile.set) {
            super::super::samples::prepare_blocking(set.samples);
        }
        super::super::samples::prepare_blocking(super::super::soundsets::COMMON);
    }
    pub fn crash_event(speed: f32, vel: Vec3, mass: f32) -> Event {
        Event::Crash {
            speed,
            vel,
            mass,
            surface: Surface::Ground,
            in_air: false,
        }
    }
}

#[test]
#[ignore]
fn crash_cpu_profile() {
    let pl = &PLANES[0];
    let mut f = base_frame(pl.id);
    compat::prepare(&f);
    let mut s = Synth::new(SR);
    let mut buf = vec![0.0f32; 1600];
    let def = crate::flight::find(pl.id).unwrap();
    set_power(&mut f, pl, 0.7, 0.0, true);
    for sec in 0..20 {
        let t0 = web_time::Instant::now();
        for k in 0..60 {
            f.crashed = sec >= 3;
            for e in f.eng.iter_mut() {
                e.running = !f.crashed;
            }
            f.cut = sec == 0 && k == 0;
            s.push_frame(&f);
            if sec == 3 && k == 0 {
                s.event(&compat::crash_event(
                    35.0,
                    Vec3::new(34.0, -8.0, 0.0),
                    def.mass,
                ));
            }
            s.render(&mut buf);
        }
        eprintln!(
            "second {sec}: {:.2} % of a core",
            t0.elapsed().as_secs_f64() * 100.0
        );
    }
}
