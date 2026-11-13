//! Weapon sounds (Track W): guns as trains of individual reports at the
//! real cyclic rate (each a muzzle-blast crack + a low "body" thump through
//! the airframe + mechanism clatter), so the Hispano's slow pounding, the
//! .303s' rattle and the M61's 100 Hz "brrrt" come out of the same model;
//! the AIM-9 seeker growl / lock tone and the RWR tones in the headset.
//! One-shots (cracks of passing rounds, hits, launches, flares, ejection)
//! go to the shared one-shot pool.

use super::dsp::*;
use super::fx::{Kind, Route, Shot};
use super::params::Event;
use glam::Vec3;

/// Per gun type: (body thump Hz, thump decay s, crack level, crack decay s, mechanism bp Hz, level).
const GUNS: [(f32, f32, f32, f32, f32, f32); 4] = [
    // 20 mm Hispano: heavy, slow pounding.
    (52.0, 0.075, 0.9, 0.006, 900.0, 1.0),
    // .303 Browning: light, sharp, fast.
    (130.0, 0.03, 0.55, 0.003, 2200.0, 0.32),
    // M61A1: 100 rounds / s — the individual reports fuse into a buzz-saw tone.
    (75.0, 0.012, 0.7, 0.0025, 1400.0, 0.55),
    // .50 Browning.
    (95.0, 0.04, 0.7, 0.004, 1600.0, 0.5),
];

/// Overlapping reports per voice.
const SLOTS: usize = 12;

#[derive(Clone, Copy, Default)]
struct Report {
    t: f32,
    on: bool,
    ph: f32,
    amp: f32,
}

#[derive(Clone)]
struct GunVoice {
    /// Rounds still to schedule and the interval between them (samples).
    pending: f32,
    interval: f32,
    until_next: f32,
    reps: [Report; SLOTS],
    body: Svf,
    mech: Svf,
    crack: OnePole,
    /// Where (own aircraft = inside / airframe; else world position) and level.
    own: bool,
    pos: Vec3,
    /// Gun type (index into `GUNS`).
    kind: usize,
}

impl GunVoice {
    fn new(sr: f32, k: usize) -> GunVoice {
        GunVoice {
            pending: 0.0,
            interval: 1.0,
            until_next: 0.0,
            reps: [Report::default(); SLOTS],
            body: Svf::new(GUNS[k].0 * 1.6, 0.9, sr),
            mech: Svf::new(GUNS[k].4, 2.0, sr),
            crack: OnePole::new(6000.0, sr),
            own: true,
            pos: Vec3::ZERO,
            kind: k,
        }
    }
    fn fire(&mut self, rounds: u16, rate: f32, sr: f32) {
        self.pending += rounds as f32;
        self.interval = (sr / rate.max(1.0)).max(8.0);
        // Never schedule further ahead than ~2 frames (stopping stays crisp).
        self.pending = self.pending.min(rate * 0.05 + 2.0);
    }
    /// (crack / blast, body thump, mechanism) this sample.
    fn tick(&mut self, sr: f32, rng: &mut Rng) -> (f32, f32, f32) {
        let (fb, db, cl, dc, _, lvl) = GUNS[self.kind];
        if self.pending >= 1.0 {
            self.until_next -= 1.0;
            if self.until_next <= 0.0 {
                self.pending -= 1.0;
                self.until_next += self.interval * (1.0 + 0.04 * rng.bi());
                if let Some(r) = self.reps.iter_mut().find(|r| !r.on) {
                    *r = Report { t: 0.0, on: true, ph: 0.0, amp: lvl * (0.9 + 0.2 * rng.uni()) };
                }
            }
        } else {
            self.until_next = self.until_next.max(0.0).min(self.interval);
        }
        let dt = 1.0 / sr;
        let (mut c, mut b, mut m) = (0.0, 0.0, 0.0);
        for r in self.reps.iter_mut().filter(|r| r.on) {
            r.t += dt;
            if r.t > 0.25 {
                r.on = false;
                continue;
            }
            // Blast: a sharp, bright noise burst; body: a decaying low sine with a pitch drop.
            let n = rng.bi();
            c += n * (-r.t / dc).exp() * cl * r.amp;
            r.ph += fb * (1.0 + 0.6 * (-r.t * 40.0).exp()) * dt;
            b += sin_turns(r.ph) * (-r.t / db).exp() * smoothstep(0.0, 0.0008, r.t) * r.amp;
            m += n * (-r.t / 0.02).exp() * 0.25 * r.amp;
        }
        (self.crack.hp(c), self.body.lp(b * 1.4), self.mech.bp(m))
    }
}

/// Headset tones: AIM-9 growl / lock, RWR.
#[derive(Clone)]
struct Tones {
    growl: f32,
    locked: bool,
    rwr: u8,
    rwr_new: f32,
    age: f32,
    ph: [f32; 3],
    lvl: Follow,
    lock_lvl: Follow,
    rwr_lvl: Follow,
    noise: OnePole,
    t: f32,
}

pub struct Weapons {
    sr: f32,
    guns: Vec<GunVoice>,
    tones: Tones,
    ear: Vec3,
    right: Vec3,
    interior: f32,
}

impl Weapons {
    pub fn new(sr: f32) -> Weapons {
        // Own guns (one voice per type) + up to 6 other aircraft's guns.
        let guns = (0..10).map(|i| GunVoice::new(sr, i % 4)).collect();
        Weapons {
            sr,
            guns,
            tones: Tones {
                growl: 0.0,
                locked: false,
                rwr: 0,
                rwr_new: 9.0,
                age: 9.0,
                ph: [0.0; 3],
                lvl: Follow::default(),
                lock_lvl: Follow::default(),
                rwr_lvl: Follow::default(),
                noise: OnePole::new(500.0, sr),
                t: 0.0,
            },
            ear: Vec3::ZERO,
            right: Vec3::X,
            interior: 0.0,
        }
    }

    /// Per block: the listener.
    pub fn control(&mut self, ear: Vec3, right: Vec3, interior: f32, shift: Vec3) {
        self.ear = ear;
        self.right = right.normalize_or(Vec3::X);
        self.interior = interior;
        for g in &mut self.guns {
            g.pos += shift;
        }
    }

    /// Handle a weapons event; one-shots go into `add`. Returns false if not ours.
    pub fn event(&mut self, e: &Event, add: &mut dyn FnMut(Shot)) -> bool {
        let sr = self.sr;
        match *e {
            Event::Gun { gun, rounds, rate, pos, own } => {
                let k = (gun as usize).min(3);
                // Own guns: voices 0..4 by type; others: 4.. picked by position.
                let vi = if own {
                    k
                } else {
                    let free = (4..self.guns.len()).find(|&i| (self.guns[i].pos.distance(pos) < 30.0 && self.guns[i].pending > 0.0) || self.guns[i].pending < 1.0).unwrap_or(4);
                    if self.guns[free].kind != k && self.guns[free].pending < 1.0 {
                        let mut v = GunVoice::new(sr, k);
                        v.reps = self.guns[free].reps;
                        self.guns[free] = v;
                    }
                    free
                };
                let g = &mut self.guns[vi];
                g.own = own;
                g.pos = pos;
                g.fire(rounds, rate, sr);
            }
            Event::Crack { pos, miss, cal, supersonic } => {
                let side = (pos - self.ear).dot(self.right).clamp(-1.0, 1.0);
                let k = (6.0 / miss.max(1.0)).min(1.0) * (cal / 0.02).sqrt().clamp(0.4, 1.2);
                if supersonic {
                    // Bow-shock N-wave: very short (≈ 0.3–0.6 ms for small arms).
                    let len = 0.00025 + 0.012 * cal * (miss.max(1.0)).powf(0.25);
                    add(Shot::new(Kind::NWave { len, rise: 0.00004 }, Route::Listener, 0.9 * k, len + 0.02).panned(side * 0.8));
                    add(Shot::new(Kind::Click, Route::Listener, 0.35 * k, 0.012).panned(side * 0.8));
                } else {
                    add(Shot::new(Kind::Squeal { f0: 2600.0, f1: 1300.0, ph: 0.0 }, Route::Listener, 0.25 * k, 0.22).panned(side * 0.7));
                }
            }
            Event::Hit { pos, energy, own } => {
                let e = energy.clamp(0.05, 1.0);
                if own {
                    add(Shot::new(Kind::Clank { kicked: false }, Route::Airframe, 0.35 * e.sqrt(), 0.18));
                    add(Shot::new(Kind::Thud { f0: 180.0, f1: 60.0, ph: 0.0, decay: 0.03 + 0.05 * e, noise: 2.5 }, Route::Airframe, 0.4 + 0.5 * e, 0.25));
                } else {
                    add(Shot::new(Kind::Clank { kicked: false }, Route::World, 0.5 * e.sqrt(), 0.18).at(pos));
                    add(Shot::new(Kind::Thud { f0: 140.0, f1: 50.0, ph: 0.0, decay: 0.04, noise: 2.0 }, Route::World, 0.6 * e, 0.3).at(pos));
                }
            }
            Event::Missile { pos, own } => {
                // Rail launch: the motor lights with a roar that tears away.
                if own {
                    add(Shot::new(Kind::Thud { f0: 90.0, f1: 40.0, ph: 0.0, decay: 0.2, noise: 2.5 }, Route::Airframe, 0.7, 0.6));
                    add(Shot::new(Kind::Rumble { attack: 0.01, crackle: 0.9 }, Route::Airframe, 0.55, 1.6));
                }
                add(Shot::new(Kind::Rumble { attack: 0.02, crackle: 1.0 }, Route::World, 0.9, 3.0).at(pos));
            }
            Event::Countermeasure { pos, own } => {
                if own {
                    add(Shot::new(Kind::Thud { f0: 220.0, f1: 90.0, ph: 0.0, decay: 0.02, noise: 3.0 }, Route::Airframe, 0.25, 0.12));
                    add(Shot::new(Kind::Click, Route::Cockpit, 0.12, 0.02));
                } else {
                    add(Shot::new(Kind::Thud { f0: 200.0, f1: 90.0, ph: 0.0, decay: 0.02, noise: 3.0 }, Route::World, 0.3, 0.12).at(pos));
                }
            }
            Event::Eject { pos, own, seat } => {
                if own {
                    // Canopy jettison bang, the catapult / rocket, then the wind.
                    add(Shot::new(Kind::Thud { f0: 70.0, f1: 30.0, ph: 0.0, decay: 0.25, noise: 3.0 }, Route::Listener, 1.0, 0.8));
                    add(Shot::new(Kind::Clank { kicked: false }, Route::Listener, 0.4, 0.3));
                    if seat {
                        add(Shot::new(Kind::Rumble { attack: 0.01, crackle: 1.0 }, Route::Listener, 0.9, 0.7).delayed(0.15, sr));
                    }
                    add(Shot::new(Kind::Rumble { attack: 0.3, crackle: 0.0 }, Route::Listener, 0.35, 4.0).delayed(0.4, sr));
                } else {
                    add(Shot::new(Kind::Thud { f0: 80.0, f1: 35.0, ph: 0.0, decay: 0.2, noise: 2.0 }, Route::World, 0.6, 0.6).at(pos));
                }
            }
            Event::Tones { growl, locked, rwr, rwr_new } => {
                let t = &mut self.tones;
                t.growl = growl;
                t.locked = locked;
                t.rwr = rwr;
                if rwr_new {
                    t.rwr_new = 0.0;
                }
                t.age = 0.0;
            }
            _ => return false,
        }
        true
    }

    /// One sample: (own guns outside → airframe emitter, own guns inside,
    /// other aircraft's guns (stereo), headset tones).
    pub fn tick(&mut self, rng: &mut Rng) -> (f32, f32, (f32, f32), f32) {
        let sr = self.sr;
        let (mut ext, mut int, mut wl, mut wr) = (0.0, 0.0, 0.0, 0.0);
        for g in self.guns.iter_mut() {
            if g.pending < 1.0 && !g.reps.iter().any(|r| r.on) {
                continue;
            }
            let (c, b, m) = g.tick(sr, rng);
            if g.own {
                // Outside: the blast dominates; inside: thumps through the
                // structure, the mechanism, a muffled blast.
                ext += c * 1.1 + b * 0.5;
                int += b * 1.3 + m * 0.6 + c * 0.12;
            } else {
                let d = g.pos.distance(self.ear).max(5.0);
                let gain = (160.0 / d).min(1.0) * 1.2;
                let p = ((g.pos - self.ear).dot(self.right) / d).clamp(-1.0, 1.0);
                let (l, r) = pan(p * 0.8);
                // Distance dulls the crack more than the boom.
                let far = (d / 1500.0).min(1.0);
                let y = (c * (1.0 - 0.8 * far) + b * 0.8) * gain;
                wl += y * l;
                wr += y * r;
            }
        }
        (ext, int, (wl, wr), self.tones(rng))
    }

    fn tones(&mut self, rng: &mut Rng) -> f32 {
        let dt = 1.0 / self.sr;
        let t = &mut self.tones;
        t.age += dt;
        t.rwr_new += dt;
        t.t += dt;
        let fresh = t.age < 0.3;
        // AIM-9: a growl whose pitch / loudness rise with the signal; locked:
        // a clear, higher steady tone.
        let g = if fresh && !t.locked { t.growl } else { 0.0 };
        let lv = t.lvl.go(g, dt, 0.05, 0.08);
        let ll = t.lock_lvl.go(if fresh && t.locked { 1.0 } else { 0.0 }, dt, 0.03, 0.06);
        let mut y = 0.0;
        if lv > 1e-4 {
            let f = 380.0 + 520.0 * lv;
            t.ph[0] = (t.ph[0] + f * dt).fract();
            // Amplitude-modulated by low-passed noise: the "growl".
            let am = 0.55 + 0.45 * t.noise.lp(rng.bi() * 3.0).clamp(-1.0, 1.0);
            y += (sin_turns(t.ph[0]) + 0.3 * sin_turns(t.ph[0] * 2.0)) * am * lv * 0.12;
        }
        if ll > 1e-4 {
            t.ph[1] = (t.ph[1] + 1580.0 * dt).fract();
            y += (sin_turns(t.ph[1]) + 0.2 * sin_turns(t.ph[1] * 3.0)) * ll * 0.1;
        }
        // RWR: new emitter → three short beeps; track → steady pulsed tone; launch → fast warble.
        let rw = if fresh { t.rwr } else { 0 };
        let (f, gate) = match rw {
            3 => (1000.0 + 450.0 * (t.t * 14.0 * std::f32::consts::TAU).sin().signum(), 1.0),
            2 => (1300.0, if (t.t * 3.0).fract() < 0.75 { 1.0 } else { 0.0 }),
            _ if t.rwr_new < 0.45 => (2100.0, if (t.rwr_new * 6.7).fract() < 0.6 { 1.0 } else { 0.0 }),
            _ => (0.0, 0.0),
        };
        let rl = t.rwr_lvl.go(gate, dt, 0.004, 0.01);
        if rl > 1e-4 && f > 0.0 {
            t.ph[2] = (t.ph[2] + f * dt).fract();
            let sq = if t.ph[2] < 0.5 { 1.0 } else { -1.0 };
            y += (sin_turns(t.ph[2]) * 0.7 + sq * 0.15) * rl * 0.1;
        }
        y
    }

    /// Any gun sounding (offline renders / tests).
    pub fn busy(&self) -> bool {
        self.guns.iter().any(|g| g.pending >= 1.0 || g.reps.iter().any(|r| r.on))
    }
}

#[cfg(test)]
mod tests {
    //! Offline renders (`SKYWARD_AUDIO_TEST_OUT=/dir cargo test --release audio::weapons -- --nocapture`
    //! writes the WAVs): guns at their cyclic rates, cracks, tones, launch, ejection.
    use super::super::capture::profile_for;
    use super::super::params::*;
    use super::super::synth::Synth;
    use glam::{Quat, Vec3};

    const SR: f32 = 48_000.0;

    fn frame(id: &str, interior: f32) -> Frame {
        let def = crate::flight::find(id).expect("aircraft");
        let p = profile_for(&def, 10.0);
        let mut f = Frame { active: true, warp: 1.0, profile: p, interior, day: 1.0, ..Frame::default() };
        f.ear_right = Vec3::X;
        f.ear_up = Vec3::Y;
        f.ear_fwd = Vec3::NEG_Z;
        f.ear_agl = 3000.0;
        f.ac_rot = Quat::IDENTITY;
        for i in 0..p.engines {
            f.eng[i] = EngineFrame { rpm: p.n1_rpm * 0.9, spool: 0.9, thrust: 0.85, running: true, throttle: 0.9, ..Default::default() };
        }
        f
    }

    /// Run `secs` of frames; `ev(frame_index)` gives the events of each frame.
    fn render(mut f: Frame, secs: f32, mut ev: impl FnMut(usize, &mut Frame) -> Vec<Event>) -> Vec<f32> {
        let mut s = Synth::new(SR);
        let mut buf = vec![0.0f32; 1600];
        let mut out = Vec::new();
        for k in 0..(secs * 60.0) as usize {
            let evs = ev(k, &mut f);
            f.cut = k == 0;
            s.push_frame(&f);
            for e in &evs {
                s.event(e);
            }
            s.render(&mut buf);
            out.extend_from_slice(&buf);
        }
        out
    }

    fn save(name: &str, x: &[f32]) {
        if let Some(dir) = std::env::var_os("SKYWARD_AUDIO_TEST_OUT") {
            let p = std::path::Path::new(&dir).join(name);
            let spec = hound::WavSpec { channels: 2, sample_rate: SR as u32, bits_per_sample: 32, sample_format: hound::SampleFormat::Float };
            let mut w = hound::WavWriter::create(p, spec).unwrap();
            for v in x {
                w.write_sample(*v).unwrap();
            }
            w.finalize().unwrap();
        }
    }

    fn mono(x: &[f32]) -> Vec<f32> {
        x.chunks(2).map(|c| 0.5 * (c[0] + c[1])).collect()
    }

    /// Strength of the envelope's periodicity at `hz` (normalised autocorrelation of |x| at that lag).
    fn periodicity(x: &[f32], hz: f32) -> f32 {
        let env: Vec<f32> = x.iter().map(|v| v.abs()).collect();
        let m = env.iter().sum::<f32>() / env.len() as f32;
        let e: Vec<f32> = env.iter().map(|v| v - m).collect();
        let lag = (SR / hz) as usize;
        let n = e.len() - lag;
        let num: f32 = (0..n).map(|i| e[i] * e[i + lag]).sum();
        let den: f32 = (0..n).map(|i| e[i] * e[i]).sum::<f32>().max(1e-9);
        num / den
    }

    fn check(x: &[f32], what: &str) {
        assert!(x.iter().all(|v| v.is_finite()), "{what}: non-finite");
        let peak = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(peak <= 0.892, "{what}: peak {peak}");
    }

    #[test]
    fn guns_fire_at_their_cyclic_rates() {
        // F-16 cockpit: M61 at 6,000 rpm (100 rounds / s) for 1.5 s.
        let x = render(frame("lancer", 1.0), 3.0, |k, _| {
            if (30..120).contains(&k) { vec![Event::Gun { gun: 2, rounds: if k % 3 == 0 { 2 } else { 1 }, rate: 100.0, pos: Vec3::ZERO, own: true }] } else { vec![] }
        });
        check(&x, "M61");
        save("w_f16_m61_cockpit.wav", &x);
        let m = mono(&x);
        let seg = &m[(0.8 * SR) as usize..(1.8 * SR) as usize];
        let p100 = periodicity(seg, 100.0);
        let p73 = periodicity(seg, 73.0);
        eprintln!("M61 envelope periodicity: 100 Hz {p100:.2}, 73 Hz {p73:.2}");
        assert!(p100 > p73 + 0.1, "{p100} vs {p73}");
        // Spitfire cockpit: 2 × Hispano (≈ 20 / s together) then all guns
        // (engine idle so the guns' rhythm is measurable).
        let mut quiet = frame("peregrine", 1.0);
        for e in quiet.eng.iter_mut() {
            e.spool = 0.1;
            e.thrust = 0.05;
            e.throttle = 0.0;
        }
        let x = render(quiet, 4.0, |k, _| {
            let mut v = Vec::new();
            if (30..150).contains(&k) {
                v.push(Event::Gun { gun: 0, rounds: if k % 3 == 0 { 1 } else { 0 }, rate: 20.0, pos: Vec3::ZERO, own: true });
            }
            if (90..150).contains(&k) {
                v.push(Event::Gun { gun: 1, rounds: if k % 4 == 0 { 5 } else { 1 }, rate: 76.7, pos: Vec3::ZERO, own: true });
            }
            v
        });
        check(&x, "Spitfire guns");
        save("w_spitfire_guns_cockpit.wav", &x);
        let m = mono(&x);
        let cannon = &m[(0.6 * SR) as usize..(1.4 * SR) as usize];
        let pc = periodicity(cannon, 20.0);
        eprintln!("Hispano pair envelope periodicity at 20 Hz: {pc:.2}");
        assert!(pc > 0.2, "{pc}");
        // Outside: a bandit's M61 burst 300 m away, then 3 km away (quieter, duller).
        let mut level = Vec::new();
        for d in [300.0f32, 3000.0, 1.0e7] {
            let mut fr = frame("lancer", 0.0);
            for e in fr.eng.iter_mut() {
                e.running = false;
                e.spool = 0.0;
                e.thrust = 0.0;
            }
            let x = render(fr, 2.0, |k, f| {
                f.ac_pos = Vec3::new(0.0, 0.0, 2000.0);
                if (20..80).contains(&k) { vec![Event::Gun { gun: 2, rounds: 2, rate: 100.0, pos: Vec3::new(d, 0.0, 0.0), own: false }] } else { vec![] }
            });
            check(&x, "M61 outside");
            if d < 1.0e6 {
                save(&format!("w_m61_outside_{d:.0}m.wav"), &x);
            }
            let m = mono(&x);
            let seg = &m[(0.4 * SR) as usize..(1.3 * SR) as usize];
            level.push((seg.iter().map(|v| v * v).sum::<f32>() / seg.len() as f32).sqrt());
        }
        // (The last run is the background without the gun: subtract it.)
        let gun = |l: f32| (l * l - level[2] * level[2]).max(0.0).sqrt();
        eprintln!("bandit M61 RMS at 300 m {:.4}, 3 km {:.4} (over the background)", gun(level[0]), gun(level[1]));
        assert!(gun(level[0]) > gun(level[1]) * 3.0, "{level:?}");
    }

    #[test]
    fn cracks_tones_launch_and_ejection_render_cleanly() {
        let x = render(frame("lancer", 1.0), 3.0, |k, _| match k {
            20 => vec![Event::Crack { pos: Vec3::new(4.0, 0.0, 0.0), miss: 4.0, cal: 0.02, supersonic: true }],
            40 => vec![Event::Crack { pos: Vec3::new(-10.0, 0.0, 0.0), miss: 10.0, cal: 0.0079, supersonic: true }],
            60 => vec![Event::Crack { pos: Vec3::new(3.0, 0.0, 0.0), miss: 3.0, cal: 0.0079, supersonic: false }],
            90 => vec![Event::Hit { pos: Vec3::ZERO, energy: 0.8, own: true }],
            120 => vec![Event::Hit { pos: Vec3::new(0.0, 0.0, -200.0), energy: 0.8, own: false }],
            _ => vec![],
        });
        check(&x, "cracks");
        save("w_cracks_and_hits.wav", &x);
        // AIM-9: growl rising with the signal, then the lock tone; RWR: new contact, track, launch.
        let x = render(frame("lancer", 1.0), 8.0, |k, _| {
            let t = k as f32 / 60.0;
            let growl = (t / 2.5).min(1.0);
            let rwr = if t < 4.0 {
                0
            } else if t < 5.5 {
                1
            } else if t < 6.8 {
                2
            } else {
                3
            };
            vec![Event::Tones { growl, locked: (2.8..4.0).contains(&t), rwr, rwr_new: k == 240 }]
        });
        check(&x, "tones");
        save("w_aim9_and_rwr_tones.wav", &x);
        let m = mono(&x);
        let rms = |a: f32, b: f32| {
            let s = &m[(a * SR) as usize..(b * SR) as usize];
            (s.iter().map(|v| v * v).sum::<f32>() / s.len() as f32).sqrt()
        };
        assert!(rms(1.8, 2.4) > 0.005 && rms(3.0, 3.8) > 0.005 && rms(7.0, 7.8) > 0.005);
        let x = render(frame("lancer", 1.0), 5.0, |k, _| match k {
            20 => vec![Event::Missile { pos: Vec3::new(4.0, -0.5, 0.0), own: true }],
            80 => vec![Event::Countermeasure { pos: Vec3::ZERO, own: true }],
            95 => vec![Event::Countermeasure { pos: Vec3::ZERO, own: true }],
            150 => vec![Event::Eject { pos: Vec3::ZERO, own: true, seat: true }],
            _ => vec![],
        });
        check(&x, "launch / flares / eject");
        save("w_launch_flares_eject.wav", &x);
    }
}
