//! The audio-thread engine: positional mixing of the engine and airframe
//! emitters (distance, air absorption, speed-of-sound delay → Doppler,
//! directivity), cockpit sounds, ambience, one-shots, buses, ducking and the
//! output limiter. `Synth::render` is deterministic for a given sequence of
//! frames/events, so the offline renderer hears exactly what the app plays.

use super::crash::{self, Wreck};
use super::dsp::*;
use super::engines::{EngCtl, EngineVoice};
use super::fx::{Kind, Pool, Route, Shot};
use super::params::*;
use super::samples::Looper;
use super::voice::Speaker;
use glam::Vec3;

const BLOCK: usize = 64;
const SPEED_OF_SOUND: f32 = 343.0;
/// Longest propagation delay modelled (s) — ~3.4 km.
const MAX_DELAY_S: f32 = 10.0;
const N_EMIT: usize = MAX_ENG + 1;
const AIRFRAME: usize = MAX_ENG;

/// Mixer settings (menu sliders).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    pub master: f32,
    pub engines: f32,
    pub environment: f32,
    pub systems: f32,
    pub voice: f32,
    pub ui: f32,
    /// ATC / ATIS radio in the headset.
    pub radio: f32,
    pub muted: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { master: 0.7, engines: 1.0, environment: 1.0, systems: 1.0, voice: 1.0, ui: 0.8, radio: 1.0, muted: false }
    }
}

/// Per-family mix tuning.
#[derive(Clone, Copy, Debug)]
struct Tune {
    ext: f32,
    int_eng: f32,
    int_lp: f32,
    int_mix: [f32; 3],
    wind: f32,
    wind_lp: f32,
    whistle: f32,
}

fn tune(f: Family) -> Tune {
    match f {
        Family::LightPiston => Tune { ext: 1.0, int_eng: 1.35, int_lp: 2600.0, int_mix: [0.9, 1.0, 0.6], wind: 0.9, wind_lp: 4000.0, whistle: 0.6 },
        Family::Warbird => Tune { ext: 1.0, int_eng: 1.45, int_lp: 2400.0, int_mix: [0.9, 1.1, 0.9], wind: 0.9, wind_lp: 4200.0, whistle: 0.5 },
        Family::Turboprop => Tune { ext: 1.0, int_eng: 0.8, int_lp: 1300.0, int_mix: [1.4, 0.35, 0.3], wind: 0.55, wind_lp: 2200.0, whistle: 0.0 },
        Family::Fighter => Tune { ext: 1.0, int_eng: 0.7, int_lp: 1100.0, int_mix: [0.35, 0.9, 0.7], wind: 0.8, wind_lp: 3200.0, whistle: 0.4 },
        Family::Airliner => Tune { ext: 1.0, int_eng: 0.5, int_lp: 650.0, int_mix: [0.6, 0.5, 0.5], wind: 0.65, wind_lp: 1500.0, whistle: 0.0 },
        Family::Glider => Tune { ext: 1.0, int_eng: 0.0, int_lp: 1000.0, int_mix: [0.0; 3], wind: 1.25, wind_lp: 3200.0, whistle: 0.9 },
        // Helicopter cabin: rotor thump and gearbox through the roof, the
        // turbine whine behind the bulkhead, slap muffled.
        Family::Helicopter => Tune { ext: 1.0, int_eng: 1.2, int_lp: 3200.0, int_mix: [0.45, 0.35, 1.1], wind: 0.6, wind_lp: 2600.0, whistle: 0.2 },
        // FPV "cockpit" = the camera on the frame: the motors right there.
        Family::Drone => Tune { ext: 1.0, int_eng: 1.0, int_lp: 9000.0, int_mix: [1.0, 0.6, 1.0], wind: 0.35, wind_lp: 5000.0, whistle: 0.0 },
    }
}

/// A positional sound source: mono signal → propagation delay line →
/// air absorption → distance gain → pan.
struct Emitter {
    line: DelayLine,
    est: Vec3,
    delay: f32,
    d_step: f32,
    dist: f32,
    air: Svf,
    gain: Ramp,
    gl: Ramp,
    gr: Ramp,
    send: Ramp,
    // directivity weights for (front, rear, case)
    dir: [Ramp; 3],
    // interior path
    int_l: Ramp,
    int_r: Ramp,
    int_lp: Svf,
    int_lp2: OnePole,
    /// Interior gain of the filtered (exterior-derived) path.
    int_g: f32,
    /// Can sound from the source reach the ear now (outside a supersonic
    /// source's Mach cone it cannot)? Ramped gain of that.
    heard: bool,
    vis: Ramp,
}

impl Emitter {
    fn new(sr: f32) -> Emitter {
        Emitter {
            line: DelayLine::new((MAX_DELAY_S * sr) as usize + BLOCK * 2),
            est: Vec3::ZERO,
            delay: 1.0,
            d_step: 0.0,
            dist: 30.0,
            air: Svf::new(18_000.0, 0.6, sr),
            gain: Ramp::new(0.0),
            gl: Ramp::new(0.7),
            gr: Ramp::new(0.7),
            send: Ramp::new(0.0),
            dir: [Ramp::new(1.0); 3],
            int_l: Ramp::new(0.0),
            int_r: Ramp::new(0.0),
            int_lp: Svf::new(1000.0, 0.7, sr),
            int_lp2: OnePole::new(1500.0, sr),
            int_g: 1.0,
            heard: true,
            vis: Ramp::new(1.0),
        }
    }
}

/// Continuous airframe / ground / aero noise (positional) plus its
/// cockpit-side rendering.
struct Airframe {
    pink: Pink,
    aero_lp: Svf,
    aero_amp: Ramp,
    gear_bp: Svf,
    gear_amp: Ramp,
    flap_bp: Svf,
    flap_amp: Ramp,
    spoil_lp: Svf,
    spoil_amp: Ramp,
    buff_lp: Svf,
    buff_amp: Ramp,
    buff_mod: OnePole,
    roll_lp: Svf,
    roll_amp: Ramp,
    grass_bp: Svf,
    grass_amp: Ramp,
    hum_bp: Svf,
    hum_amp: Ramp,
    joint_ph: f32,
    joint_inc: Ramp,
    joint_env: f32,
    joint_amp: f32,
    joint_lp: OnePole,
    bump_rate: Ramp,
    bump_env: f32,
    squeal_ph: [Phasor; 2],
    squeal_amp: Ramp,
    squeal_f: Ramp,
    scrub_bp: Svf,
    scrub_amp: Ramp,
    vib_ph: Phasor,
    isr: f32,
    // Recorded tyre / gear rumble (exterior; heavies also a cockpit one).
    roll_rec: Looper,
    roll_rec_int: Looper,
    gravel_rec: Looper,
    /// Glider air-brake (ASK21 recording).
    brake_rec: Looper,
}

/// Interior-only sounds: cockpit wind, rain on the canopy, systems.
struct Cockpit {
    pink: Pink,
    wind_lp: Svf,
    wind_hp: OnePole,
    wind_amp: Ramp,
    whistle: Svf,
    whistle_amp: Ramp,
    whistle_f: f32,
    gust_lfo: OnePole,
    rain_rate: Ramp,
    rain_res: [Svf; 3],
    rain_hiss: Svf,
    rain_hiss_amp: Ramp,
    shake_lp: Svf,
    shake_amp: Ramp,
    rattle_bp: Svf,
    // warnings
    horn_ph: [Phasor; 3],
    horn_amp: Ramp,
    horn_kind: u8,
    beep_ph: Phasor,
    gate_ph: Phasor,
    shaker_ph: Phasor,
    shaker_env: f32,
    shaker_amp: Ramp,
    shaker_bp: Svf,
    shaker_lp: OnePole,
    gwarn_amp: Ramp,
    gwarn_ph: Phasor,
    clack_ph: Phasor,
    clack_amp: Ramp,
    clack_env: f32,
    clack_bp: Svf,
    // vario
    vario_ph: Phasor,
    vario_gate: Phasor,
    vario_f: Ramp,
    vario_rate: f32,
    vario_amp: Ramp,
    vario_beep: bool,
    vario_env: f32,
    // navaid Morse ident (nav/morse.rs), keyed sample-accurately
    ident: [u8; 4],
    ident_keyer: crate::nav::morse::Keyer,
    ident_t: f32,
    ident_ph: Phasor,
    ident_amp: Ramp,
    ident_env: f32,
    // motors
    gear_m_ph: [Phasor; 2],
    gear_m_amp: Ramp,
    gear_m_f: Ramp,
    flap_m_ph: [Phasor; 2],
    flap_m_amp: Ramp,
    flap_m_f: Ramp,
    motor_hiss: Svf,
    motor_lp: Svf,
    flap_lp: Svf,
    pack_bp: Svf,
    pack_amp: Ramp,
    fan_ph: Phasor,
    isr: f32,
    // Recorded: cockpit wind, the C172 stall horn, flap / gear motors.
    wind_rec: Looper,
    horn_rec: Looper,
    flap_rec: Looper,
    gear_rec: Looper,
}

/// Around the listener: city, birds, wind, rain, thunder.
struct Ambience {
    pink: Pink,
    city_lp: Svf,
    city_amp: Ramp,
    traffic_bp: Svf,
    traffic_mod: OnePole,
    wind_lp: Svf,
    wind_amp: Ramp,
    wind_mod: OnePole,
    rain_bp: Svf,
    rain_amp: Ramp,
    drop_rate: Ramp,
    drop_hp: OnePole,
    drop_env: f32,
    drop_amp: f32,
    drop_pan: f32,
    bird_rate: f32,
    isr: f32,
    move_lp: Svf,
    move_amp: Ramp,
}

pub struct Synth {
    pub sr: f32,
    rng: Rng,
    frame: Frame,
    have_frame: bool,
    since_frame: f32,
    profile: Profile,
    tune: Tune,
    voices: [Option<EngineVoice>; MAX_ENG],
    emit: [Emitter; N_EMIT],
    ear: Vec3,
    inside: Follow,
    inside_r: Ramp,
    world: Follow,
    world_r: Ramp,
    muffle: Follow,
    muffle_l: OnePole,
    muffle_r: OnePole,
    muffle_fc: f32,
    cut_gain: Ramp,
    cut_wait: i64,
    air: Airframe,
    cock: Cockpit,
    amb: Ambience,
    pool: Pool,
    wreck: Wreck,
    /// Everything but the UI bus (cut to silence after a fatal impact in the cockpit).
    blackout: Ramp,
    blackout_wait: i64,
    was_crashed: bool,
    /// Mach number last block (shock passing over the canopy at M 1).
    prev_mach: f32,
    /// Engine voices' share of exterior mix in the interior path.
    ife: [f32; MAX_ENG],
    speaker: Speaker,
    /// The cabin PA (missions): its own channel, under the cockpit voices.
    pa: Speaker,
    /// COM 1 / COM 2 receivers (ATC, ATIS).
    radio: super::radio::Radio,
    fdn: Fdn,
    world_lp: OnePole,
    world_lp2: OnePole,
    lim: Limiter,
    settings: Settings,
    g_eng: Ramp,
    g_env: Ramp,
    g_sys: Ramp,
    g_voice: Ramp,
    g_radio: Ramp,
    g_ui: Ramp,
    g_master: Ramp,
    ext_sys: Ramp,
    // stats
    pub booms: u32,
    /// (N-wave length s, overpressure Pa) of the last boom.
    pub last_boom: (f32, f32),
    pub samples: u64,
    pub peak: f32,
    pub nan_count: u64,
}

impl Synth {
    pub fn new(sr: f32) -> Synth {
        let sr = sr.clamp(8000.0, 192_000.0);
        let mut s = Synth {
            sr,
            rng: Rng::new(0x5eed),
            frame: Frame::default(),
            have_frame: false,
            since_frame: 0.0,
            profile: Profile::default(),
            tune: tune(Family::Glider),
            voices: [None, None, None, None],
            emit: [0, 1, 2, 3, 4].map(|_| Emitter::new(sr)),
            ear: Vec3::ZERO,
            inside: Follow::default(),
            inside_r: Ramp::new(0.0),
            world: Follow::default(),
            world_r: Ramp::new(0.0),
            muffle: Follow::default(),
            muffle_l: OnePole::new(20_000.0, sr),
            muffle_r: OnePole::new(20_000.0, sr),
            muffle_fc: 20_000.0,
            cut_gain: Ramp::new(0.0),
            cut_wait: 0,
            air: Airframe::new(sr),
            cock: Cockpit::new(sr),
            amb: Ambience::new(sr),
            pool: Pool::default(),
            wreck: Wreck::new(sr),
            blackout: Ramp::new(1.0),
            blackout_wait: 0,
            was_crashed: false,
            prev_mach: 0.0,
            ife: [1.0; MAX_ENG],
            speaker: Speaker::new(sr),
            pa: Speaker::new(sr),
            radio: super::radio::Radio::new(sr),
            fdn: Fdn::new(sr),
            world_lp: OnePole::new(700.0, sr),
            world_lp2: OnePole::new(1100.0, sr),
            lim: Limiter::new(sr, -1.0),
            settings: Settings::default(),
            g_eng: Ramp::new(1.0),
            g_env: Ramp::new(1.0),
            g_sys: Ramp::new(1.0),
            g_voice: Ramp::new(1.0),
            g_radio: Ramp::new(1.0),
            g_ui: Ramp::new(1.0),
            g_master: Ramp::new(0.0),
            ext_sys: Ramp::new(1.0),
            booms: 0,
            last_boom: (0.0, 0.0),
            samples: 0,
            peak: 0.0,
            nan_count: 0,
        };
        // Decode the speech bank now, not on the first callout.
        let _ = super::voice::bank();
        s.frame.warp = 1.0;
        s
    }

    pub fn set_settings(&mut self, st: Settings) {
        self.settings = st;
    }

    pub fn limiter_clamped(&self) -> u64 {
        self.lim.clamped
    }
    pub fn limiter_min_gain(&self) -> f32 {
        self.lim.min_gain
    }

    /// New game-thread frame.
    pub fn push_frame(&mut self, f: &Frame) {
        if f.profile.type_id != self.profile.type_id || f.profile.family != self.profile.family || f.profile.engines != self.profile.engines {
            self.profile = f.profile;
            self.tune = tune(f.profile.family);
            for i in 0..MAX_ENG {
                self.voices[i] = if i < f.profile.engines { EngineVoice::new(&f.profile, i, self.sr) } else { None };
            }
        }
        if self.was_crashed && !f.crashed {
            // Respawn: the wreck and the ringing go away.
            self.wreck.clear();
            self.blackout.to(1.0, (0.3 * self.sr) as u32);
            self.blackout_wait = 0;
        }
        self.was_crashed = f.crashed;
        let cut = f.cut || !self.have_frame;
        if f.shift != Vec3::ZERO && !cut {
            self.ear += f.shift;
            for e in &mut self.emit {
                e.est += f.shift;
            }
        }
        self.frame = *f;
        self.frame.cut = cut;
        self.have_frame = true;
        self.since_frame = 0.0;
        if cut && self.cut_wait <= 0 {
            // Fade the outside mix out, then snap the delays, then fade in.
            let n = (0.012 * self.sr) as u32;
            self.cut_gain.to(0.0, n);
            self.cut_wait = n as i64;
        }
    }

    pub fn event(&mut self, e: &Event) {
        let sr = self.sr;
        // Crashes, impacts, wreckage (crash.rs): positional, by energy.
        {
            let ear = self.ear;
            let cockpit = self.frame.interior > 0.5;
            let fam = self.profile.family;
            let mut cx = crash::Ctx { pool: &mut self.pool, rng: &mut self.rng, sr, ear };
            match *e {
                Event::Crash { speed, vel, mass, surface, in_air } => {
                    let vel = if vel.length_squared() > 0.01 { vel } else { Vec3::new(0.0, -speed, 0.0) };
                    let pos = self.frame.ac_pos;
                    let _ = fam;
                    if crash::crash(&mut cx, &mut self.wreck, pos, vel, mass.max(100.0), surface, in_air, cockpit) {
                        // Fatal, seen from the cockpit: the world cuts out
                        // just after the impact (ringing ears stay, UI bus).
                        self.blackout_wait = (0.2 * sr) as i64;
                    }
                    return;
                }
                Event::Impact { pos, speed, mass, surface } => {
                    crash::impact(&mut cx, pos, speed, mass, surface);
                    return;
                }
                Event::Separation { pos, mass } => {
                    crash::separation(&mut cx, pos, mass);
                    return;
                }
                Event::Explosion { pos, energy } => {
                    crash::explosion(&mut cx, pos, energy);
                    return;
                }
                Event::Scrape { id, pos, speed, mass, surface } => {
                    self.wreck.scrape(id, pos, speed, mass, surface, ear);
                    return;
                }
                Event::Fire { id, pos, size } => {
                    self.wreck.fire(id.min(7), pos, size, false, ear);
                    return;
                }
                _ => {}
            }
        }
        let rng = &mut self.rng;
        let pool = &mut self.pool;
        let speaker = &mut self.speaker;
        let fam = self.profile.family;
        let heavy = matches!(fam, Family::Airliner | Family::Turboprop);
        let mut add = |s: Shot, rng: &mut Rng| pool.add(s.init(sr, rng));
        match *e {
            Event::GearStart { down } => {
                // Uplock release / doors: a thump and a whoosh.
                let a = if heavy { 0.35 } else { 0.25 };
                add(Shot::new(Kind::Thud { f0: 120.0, f1: 70.0, ph: 0.0, decay: 0.07, noise: 0.6 }, Route::Airframe, a, 0.35), rng);
                add(Shot::new(Kind::Clank { kicked: false }, Route::Airframe, 0.06, 0.25), rng);
                if down {
                    add(Shot::new(Kind::Rumble { attack: 0.3, crackle: 0.0 }, Route::Airframe, 0.12, 1.4), rng);
                }
                // Airliners: the recorded A320 gear sequence (doors, motors, thumps).
                if heavy && let Some(s) = Shot::sample("gear_heavy", 1.0, Route::Airframe, 0.35, 0.0, sr) {
                    add(s, rng);
                }
            }
            Event::GearLock { down } => {
                let a = if down { 0.6 } else { 0.4 };
                add(Shot::new(Kind::Thud { f0: 160.0, f1: 55.0, ph: 0.0, decay: 0.09, noise: 1.0 }, Route::Airframe, a, 0.45), rng);
                add(Shot::new(Kind::Clank { kicked: false }, Route::Airframe, 0.12, 0.35), rng);
            }
            Event::Touchdown { sink, speed } => {
                let k = (sink / 2.5).clamp(0.08, 1.5);
                add(Shot::new(Kind::Thud { f0: 110.0, f1: 45.0, ph: 0.0, decay: 0.12 + 0.1 * k, noise: 1.2 }, Route::Airframe, 0.25 + 0.5 * k, 0.8), rng);
                let rec = match fam {
                    Family::Airliner | Family::Turboprop => Some(("heavy_touch", 0.55)),
                    Family::Fighter => Some(("fighter_touch", 0.45)),
                    _ => None,
                };
                if let Some((name, g)) = rec
                    && let Some(s) = Shot::sample(name, 0.95 + 0.1 * rng.uni(), Route::Airframe, g * (0.4 + 0.6 * k.min(1.2)), 0.0, sr)
                {
                    add(s, rng);
                }
                if speed > 8.0 && fam != Family::Glider {
                    let sq = (speed / 70.0).clamp(0.2, 1.0) * (0.4 + 0.6 * k.min(1.0));
                    let f0 = 1500.0 + 500.0 * rng.uni();
                    let n = if heavy { 2 } else { 1 };
                    for i in 0..n {
                        let s = Shot::new(Kind::Squeal { f0: f0 * (1.0 + 0.07 * i as f32), f1: f0 * 0.62, ph: 0.0 }, Route::Airframe, 0.32 * sq, 0.16 + 0.12 * sq);
                        add(s.delayed(0.03 * i as f32 + 0.01 * rng.uni(), sr), rng);
                    }
                }
            }
            Event::Thump(a) => {
                add(Shot::new(Kind::Thud { f0: 70.0, f1: 40.0, ph: 0.0, decay: 0.08, noise: 1.6 }, Route::Airframe, 0.3 * a, 0.4), rng);
                if a > 0.4 {
                    add(Shot::new(Kind::Clank { kicked: false }, Route::Cockpit, 0.04 * a, 0.2), rng);
                }
            }
            Event::Crash { .. } | Event::Impact { .. } | Event::Separation { .. } | Event::Explosion { .. } | Event::Scrape { .. } | Event::Fire { .. } => {}
            Event::Speak(c) if c.is_pa() => self.pa.play(c),
            Event::Speak(c) => speaker.play(c),
            Event::Radio(id) => self.radio.start(id),
            Event::RadioCut(ch) => self.radio.cut(ch),
            Event::ApDisconnect => {
                // Original three-note bugle figure, played twice.
                let (c, e, g, hi) = (1046.5, 1318.5, 1568.0, 2093.0);
                let notes = [c, e, g, hi, c, e, g, hi];
                add(Shot::new(Kind::Chime { notes, n: 8, note: 0.11, ph: [0.0; 3], bright: 1.0 }, Route::Cockpit, 0.16, 0.11 * 8.0 + 0.1), rng);
            }
            Event::MasterCaution => {
                let notes = [1180.0, 0.0, 1180.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                add(Shot::new(Kind::Chime { notes, n: 3, note: 0.32, ph: [0.0; 3], bright: 0.4 }, Route::Cockpit, 0.2, 0.96), rng);
            }
            Event::Crc => {
                // Continuous repetitive chime: four bright dings a second.
                let f = 1046.5;
                let notes = [f, 0.0, f, 0.0, f, 0.0, f, 0.0];
                add(Shot::new(Kind::Chime { notes, n: 8, note: 0.125, ph: [0.0; 3], bright: 0.7 }, Route::Cockpit, 0.22, 1.0), rng);
            }
            Event::FireBell => {
                // Electric fire bell: a clapper striking ~16 times a second.
                let f = 1320.0;
                let notes = [f; 8];
                for k in 0..2 {
                    add(Shot::new(Kind::Chime { notes, n: 8, note: 0.0625, ph: [0.0; 3], bright: 1.0 }, Route::Cockpit, 0.16, 0.5).delayed(0.5 * k as f32, sr), rng);
                }
            }
            Event::AbLight(_) => {
                add(Shot::new(Kind::Thud { f0: 70.0, f1: 32.0, ph: 0.0, decay: 0.25, noise: 2.2 }, Route::Airframe, 0.9, 1.0), rng);
                add(Shot::new(Kind::Rumble { attack: 0.02, crackle: 0.6 }, Route::Airframe, 0.4, 0.8), rng);
            }
            Event::Boom(k) => {
                // An externally signalled boom (strength 0..1): a generic
                // N-wave; the synth makes its own from the geometry.
                let len = 0.12;
                add(Shot::new(Kind::NWave { len, rise: 0.002 }, Route::Listener, 0.6 * k, len + 0.1), rng);
                add(Shot::new(Kind::NWave { len, rise: 0.002 }, Route::Listener, 0.45 * k, len + 0.1).delayed(0.006, sr), rng);
                add(Shot::new(Kind::Rumble { attack: 0.03, crackle: 0.2 }, Route::Listener, 0.25 * k, 2.0), rng);
            }
            Event::Thunder(km) => {
                let k = (2.0 / km.max(0.5)).clamp(0.15, 1.0);
                let pan = rng.bi() * 0.7;
                let delay = km * 1000.0 / SPEED_OF_SOUND;
                let names = ["thunder_1", "thunder_2", "thunder_3"];
                let name = names[(rng.next_u32() % 3) as usize];
                // Recorded thunder (fgdata), duller and softer with distance,
                // over a procedural low grumble.
                if let Some(s) = Shot::sample(name, 0.85 + 0.25 * rng.uni(), Route::Listener, 0.55 * k.powf(0.8), 0.0, sr) {
                    add(s.muffled(900.0 + 5000.0 * k, sr).delayed(delay.min(8.0), sr).panned(pan), rng);
                }
                add(
                    Shot::new(Kind::Rumble { attack: 0.08 + 0.3 * (1.0 - k), crackle: 0.3 * k }, Route::Listener, 0.4 * k.powf(0.7), 3.5 + 4.0 * rng.uni()).delayed(delay.min(8.0) + 0.1, sr).panned(pan),
                    rng,
                );
            }
            Event::ThunderRoll { km, rumble, energy, pan, ground } => {
                // Close strikes: the crack (an N-wave from the nearest channel
                // segment, then the tearing crackle of the rest of it); further
                // away the air absorbs the highs and only the rolling rumble
                // is left, lasting as long as the channel's spread of distances.
                let k = (3.0 / km.max(0.3)).clamp(0.12, 1.2) * energy.clamp(0.4, 1.8).sqrt();
                let lp = (14_000.0 * (-km / 2.5).exp() + 450.0).min(16_000.0);
                if km < 2.5 && ground {
                    let len = 0.006 + 0.004 * km;
                    add(Shot::new(Kind::NWave { len, rise: 0.0006 + 0.001 * km }, Route::Listener, 0.9 * k, len + 0.05).panned(pan), rng);
                    add(Shot::new(Kind::Rumble { attack: 0.004, crackle: 1.0 }, Route::Listener, 0.55 * k, 0.6 + 0.3 * km).muffled(lp, sr).panned(pan), rng);
                }
                let names = ["thunder_1", "thunder_2", "thunder_3"];
                let name = names[(rng.next_u32() % 3) as usize];
                if let Some(s) = Shot::sample(name, 0.8 + 0.25 * rng.uni() - 0.04 * km.min(5.0), Route::Listener, 1.1 * k.powf(0.7), 0.0, sr) {
                    add(s.muffled(lp.min(900.0 + 5000.0 * k), sr).delayed(0.02 + 0.05 * km.min(4.0), sr).panned(pan * 0.8), rng);
                }
                add(
                    Shot::new(Kind::Rumble { attack: 0.05 + 0.25 * (1.0 - k), crackle: 0.35 * k }, Route::Listener, 0.8 * k.powf(0.6), rumble.clamp(2.0, 20.0) + 1.5)
                        .muffled(lp.min(2500.0), sr)
                        .delayed(0.08, sr)
                        .panned(pan * 0.6),
                    rng,
                );
            }
            Event::UiClick => add(Shot::new(Kind::Click, Route::Ui, 0.35, 0.03), rng),
        }
    }

    /// Render interleaved stereo into `out` (len = 2 × frames).
    pub fn render(&mut self, out: &mut [f32]) {
        no_denormals();
        let frames = out.len() / 2;
        let mut done = 0;
        while done < frames {
            let n = (frames - done).min(BLOCK);
            self.control(n as u32);
            for i in 0..n {
                let (l, r) = self.tick();
                out[(done + i) * 2] = l;
                out[(done + i) * 2 + 1] = r;
            }
            done += n;
        }
    }

    /// The listener crossed into the Mach cone: an N-wave whose length and
    /// strength follow the far-field (Whitham / Carlson) scaling with Mach
    /// number, airframe length and miss distance —
    ///   Δp ∝ (M²−1)^⅛ · l^¾ · r^−¾,   T ≈ 3.42/a · M/(M²−1)^⅜ · r^¼ · l^¾ · K
    /// (K ≈ 0.08: an F-16 at 10 km gives ~0.1 s / ~50 Pa, at 300 m ~40 ms
    /// and several hundred Pa) — a ground reflection a few ms behind it
    /// (doubling for a listener near the ground), and a rolling tail.
    fn sonic_boom(&mut self, miss: f32, from: Vec3, mach: f32, ear_agl: f32) {
        let sr = self.sr;
        let m = mach.max(1.005);
        let mm = m * m - 1.0;
        let l = self.profile.size.clamp(5.0, 70.0);
        let r = miss.max(20.0);
        let dp = 50.0 * (mm / 0.96).powf(0.125) * (l / 15.0).powf(0.75) * (10_000.0 / r).powf(0.75);
        let len = (3.42 / 340.0 * m / mm.powf(0.375) * r.powf(0.25) * l.powf(0.75) * 0.08).clamp(0.012, 0.45);
        // Shock rise time: ~1 ms close, several ms after kilometres of turbulence.
        let rise = 0.0007 + 0.004 * (r / 10_000.0).min(1.0);
        let amp = 0.9 * dp / (dp + 180.0);
        let d = from.length().max(1.0);
        let x = from.dot(self.frame.ear_right) / d;
        let elev = (from.y / d).clamp(0.15, 1.0);
        let refl = 2.0 * ear_agl.clamp(0.0, 60.0) * elev / SPEED_OF_SOUND;
        let p = (x * 0.7).clamp(-0.8, 0.8);
        let rng = &mut self.rng;
        let mut add = |s: Shot| self.pool.add(s.init(sr, rng));
        add(Shot::new(Kind::NWave { len, rise }, Route::Listener, amp, len + 0.05).panned(p));
        add(Shot::new(Kind::NWave { len, rise: rise * 1.6 }, Route::Listener, amp * 0.8, len + 0.05).panned(-0.3 * p).delayed(refl + 0.0015, sr));
        add(Shot::new(Kind::Rumble { attack: 0.02, crackle: 0.15 }, Route::Listener, amp * 0.3, 1.2 + (r / 3000.0).min(2.5)).panned(0.5 * p).delayed(0.01, sr));
        self.booms += 1;
        self.last_boom = (len, dp);
    }

    fn control(&mut self, n: u32) {
        let sr = self.sr;
        let dt = n as f32 / sr;
        let f = self.frame;
        let st = self.settings;
        self.since_frame = (self.since_frame + dt).min(0.5);
        let sf = self.since_frame;
        let fam = self.profile.family;
        let tn = self.tune;

        // Bus gains.
        let m = if st.muted { 0.0 } else { st.master.clamp(0.0, 1.5) };
        self.g_master.to(m * m, n);
        self.g_eng.to(st.engines.clamp(0.0, 2.0), n);
        self.g_env.to(st.environment.clamp(0.0, 2.0), n);
        self.g_sys.to(st.systems.clamp(0.0, 2.0), n);
        self.g_voice.to(st.voice.clamp(0.0, 2.0), n);
        self.g_radio.to(st.radio.clamp(0.0, 2.0), n);
        self.g_ui.to(st.ui.clamp(0.0, 2.0), n);

        let world_target = if !self.have_frame || !f.active || f.paused { 0.0 } else if f.menu { 0.45 } else { 1.0 };
        let w = self.world.go(world_target, dt, 0.35, 0.18);
        self.world_r.to(w, n);
        let mf = self.muffle.go(if f.menu || f.paused { 1.0 } else { 0.0 }, dt, 0.2, 0.3);
        self.muffle_fc = 20_000.0 * (1.0 - mf) + 700.0 * mf;
        self.muffle_l.set(self.muffle_fc, sr);
        self.muffle_r.set(self.muffle_fc, sr);
        let ins = self.inside.go(f.interior, dt, 0.18, 0.18);
        self.inside_r.to(ins, n);
        self.ext_sys.to(0.22 + 0.78 * ins, n);

        if self.blackout_wait > 0 {
            self.blackout_wait -= n as i64;
            if self.blackout_wait <= 0 {
                self.blackout.to(0.0, (0.09 * sr) as u32);
            }
        }

        // ---- listener / emitter kinematics (dead-reckoned between frames)
        let snap = if self.cut_wait > 0 {
            self.cut_wait -= n as i64;
            if self.cut_wait <= 0 {
                self.cut_gain.to(1.0, (0.06 * sr) as u32);
                true
            } else {
                false
            }
        } else {
            false
        };
        let first = self.samples == 0;
        let k = 1.0 - (-dt / 0.06).exp();
        let ear_t = f.ear_pos + f.ear_vel * sf;
        if snap || first {
            self.ear = ear_t;
        } else {
            self.ear += f.ear_vel * dt;
            self.ear += (ear_t - self.ear) * k;
        }
        let ear = self.ear;
        let (er, eu, ef) = (f.ear_right, f.ear_up, f.ear_fwd);
        let axis = f.ac_rot * Vec3::Z;
        // Supersonic source: exact retarded-time solution with the full
        // speed (silence ahead of the Mach cone, boom on entering it). At
        // time-warp the velocity is scaled, so stay with the subsonic model.
        let sup = f.mach >= 1.0 && f.warp <= 1.01 && !f.paused && !f.crashed;
        let max_slew = if f.warp > 1.5 { 0.02 } else if sup { 5.0 } else { 0.6 };
        let static_ear = f.ear_vel.length() < 0.3 * f.ac_vel.length().max(1.0);
        let mut boom: Option<(f32, Vec3)> = None;
        let refd = self.profile.size * 2.6 + 6.0; // = the chase-camera distance
        let dmin = self.profile.size * 0.45;
        let n_eng = self.profile.engines;
        for e in 0..N_EMIT {
            let active = e == AIRFRAME || e < n_eng;
            if !active {
                continue;
            }
            let src_t = if e == AIRFRAME { f.ac_pos } else { f.eng[e].pos } + f.ac_vel * sf;
            let em = &mut self.emit[e];
            if snap || first {
                em.est = src_t;
            } else {
                em.est += f.ac_vel * dt;
                em.est += (src_t - em.est) * k;
            }
            let dv = em.est - ear;
            let d = dv.length().max(0.01);
            em.dist = d;
            let dir = dv / d;
            // Retarded time: the sound heard now left the source when it was
            // at src − v·D, so solve |a − v·D| = c·D (constant velocity):
            // the exact moving-source Doppler, not just the distance rate.
            // Rationalised root τ = d² / (a·v + √((a·v)² − (v² − c²)d²)),
            // valid at any speed: no real root, or a non-positive
            // denominator (supersonic source not yet past), means nothing
            // emitted can have reached the ear yet.
            let c = SPEED_OF_SOUND;
            let v = f.ac_vel;
            let v2 = if sup { v.length_squared() } else { v.length_squared().min((0.8 * c) * (0.8 * c)) };
            let av = dv.dot(v) * if v.length_squared() > 1e-6 { (v2 / v.length_squared()).sqrt() } else { 0.0 };
            let disc = av * av - (v2 - c * c) * d * d;
            let den = av + disc.max(0.0).sqrt();
            let heard = !sup || (disc >= 0.0 && den > 1e-3);
            let delay_s = if heard { d * d / den.max(1e-3) } else { em.delay / sr };
            let target = (delay_s * sr).clamp(1.0, em.line.max_delay() - BLOCK as f32);
            let entering = heard && !em.heard;
            if snap || first || entering {
                em.delay = target;
                em.d_step = 0.0;
            } else if heard {
                em.d_step = ((target - em.delay) / n as f32).clamp(-max_slew, max_slew);
            } else {
                em.d_step = 0.0;
            }
            if entering && !snap && !first && e == AIRFRAME && sup && f.interior < 0.5 && static_ear {
                // The listener just crossed into the Mach cone. Miss
                // distance (normal to the track) and where the sound came from.
                let vh = v.normalize_or_zero();
                let miss = dv.cross(vh).length();
                let from = dv - v * delay_s;
                boom = Some((miss, from));
            }
            em.heard = heard;
            em.vis.to(if heard { 1.0 } else { 0.0 }, if entering { (0.04 * sr) as u32 } else { n.min((0.006 * sr) as u32).max(1) });
            // Exterior: distance law, air absorption, pan, reverb send.
            // Spherical spreading (a touch under 1/r: ground reflection),
            // eased beyond ~400 m so distant aircraft stay audible over the
            // listener's own ambience (the mix can't hold 90 dB of range).
            let dd = d.max(dmin);
            let g = if dd < 400.0 { (refd / dd).powf(0.92) } else { (refd / 400.0).powf(0.92) * (400.0 / dd).powf(0.75) };
            let g = g.min(1.4) * tn.ext;
            em.gain.to(g, n);
            let fc = 19_000.0 * (160.0 / (d + 160.0)).powf(1.05);
            // Reading the delay line fast (steep Doppler at supersonic
            // speed) would alias: lower the absorption filter with the rate.
            let rate = (1.0 - em.d_step).max(1.0);
            em.air.set((fc / rate).max(300.0), 0.6, sr);
            let x = dir.dot(er);
            let front = dir.dot(ef);
            // Sources behind are slightly narrower.
            let (pl, pr) = pan(x * if front < 0.0 { 0.8 } else { 0.95 });
            em.gl.to(pl, n);
            em.gr.to(pr, n);
            let _ = eu;
            em.send.to(0.05 + 0.22 * smoothstep(40.0, 1500.0, d) + 0.1 * f.urban, n);
            // Directivity: c = cos(angle between engine axis and the ear).
            let c = -dir.dot(axis);
            let (gf, gr, gc) = match fam {
                Family::Airliner | Family::Fighter => (0.2 + 0.8 * smoothstep(-0.4, 0.75, c), 0.15 + 0.85 * smoothstep(0.2, -0.8, c), 0.6 + 0.4 * smoothstep(-0.5, 0.6, c)),
                Family::Turboprop => (0.6 + 0.4 * (1.0 - c * c), 0.35 + 0.65 * smoothstep(0.1, -0.8, c), 0.8),
                // Rotor loading / BVI noise radiates ahead and below the disc;
                // the exhaust aft; the thump, whine and Fenestron all round.
                Family::Helicopter => {
                    let below = smoothstep(-0.05, 0.7, dir.y);
                    (0.35 + 0.45 * smoothstep(-0.4, 0.8, c) + 0.45 * below, 0.4 + 0.6 * smoothstep(0.2, -0.8, c), 1.0)
                }
                _ => (0.65 + 0.35 * (1.0 - c * c) + 0.1 * c, 0.85 + 0.15 * (1.0 - c.abs()), 1.0),
            };
            em.dir[0].to(gf, n);
            em.dir[1].to(gr, n);
            em.dir[2].to(gc, n);
            // Interior path: engine heard through the airframe, panned by
            // where it sits relative to the head.
            let ip = (x * 0.8).clamp(-0.85, 0.85);
            let (il, ir) = pan(ip);
            em.int_g = if e == AIRFRAME { 1.0 } else { tn.int_eng };
            em.int_l.to(il, n);
            em.int_r.to(ir, n);
            // Past Mach 1 nothing radiates forward from the engines to the
            // cockpit through the air: only the structure-borne low end.
            let supc = smoothstep(0.95, 1.25, f.mach);
            let lp = if e == AIRFRAME { 900.0 } else { tn.int_lp * (1.0 - 0.5 * supc) };
            em.int_lp.set(lp, 0.65, sr);
            em.int_lp2.set(lp * 1.8, sr);
        }
        if let Some((miss, from)) = boom {
            self.sonic_boom(miss.max(20.0), from, f.mach, f.ear_agl);
        }
        // Through Mach 1 the shock system sweeps over the canopy: a soft
        // jolt and a flutter of the airframe (felt more than heard).
        let pm = self.prev_mach;
        self.prev_mach = f.mach;
        if !first && !f.crashed && !f.on_ground && f.warp <= 1.01 && ((pm < 1.0 && f.mach >= 1.0) || (pm >= 1.0 && f.mach < 1.0)) {
            let up = f.mach >= pm;
            let s = Shot::new(Kind::Thud { f0: 70.0, f1: 30.0, ph: 0.0, decay: 0.18, noise: 3.0 }, Route::Airframe, if up { 0.45 } else { 0.3 }, 0.9);
            self.pool.add(s.init(sr, &mut self.rng));
            let s = Shot::new(Kind::Rumble { attack: 0.05, crackle: 0.0 }, Route::Airframe, 0.25, 1.2);
            self.pool.add(s.init(sr, &mut self.rng));
        }

        // ---- engines
        let eng_warp = if f.paused || f.menu { 0.0 } else { 1.0 };
        for i in 0..n_eng {
            if let Some(v) = &mut self.voices[i] {
                let c = EngCtl { e: f.eng[i], tas: f.tas, reverser: f.reverser, warp: eng_warp, dist: self.emit[i].dist, crashed: f.crashed, rotor: f.rotor };
                v.control(&c, &self.profile, n);
                self.ife[i] = v.int_from_ext();
            }
        }

        // World-placed one-shots and the wreck follow the listener.
        for s in self.pool.shots.iter_mut().flatten() {
            if s.route == Route::World {
                let (g, (l, r), fc) = crash::place(s.wpos, ear, er);
                s.wl.to(g * l, n);
                s.wr.to(g * r, n);
                s.air.set(fc, sr);
            }
        }
        self.wreck.control(ear, er, n, dt);
        self.air.control(&f, fam, sr, n);
        self.cock.control(&f, fam, &tn, sr, n, dt);
        self.amb.control(&f, sr, n, dt);
        // Birds: occasional phrases while near the ground in daylight.
        if self.rng.uni() < self.amb.bird_rate * dt {
            let fb = 2600.0 + 2600.0 * self.rng.uni();
            let s = Shot::new(Kind::Bird { f: fb, ph: 0.0, syll: 0.06 + 0.06 * self.rng.uni(), n: 2 + (self.rng.uni() * 6.0) as u32 }, Route::Listener, 0.012 + 0.02 * self.rng.uni(), 0.9)
                .panned(self.rng.bi() * 0.9);
            self.pool.add(s.init(sr, &mut self.rng));
        }
    }

    #[inline]
    fn tick(&mut self) -> (f32, f32) {
        let sr = self.sr;
        let ins = self.inside_r.tick();
        // The outside mix leaves faster than the inside one arrives (no
        // loudness bump when the two are correlated).
        let outs = (1.0 - ins) * (1.0 - ins);
        let cut = self.cut_gain.tick();
        let n_eng = self.profile.engines;
        let mut ext_eng = (0.0f32, 0.0f32);
        let mut int_eng = (0.0f32, 0.0f32);
        let mut send = 0.0f32;
        // Engines → their emitters.
        for i in 0..n_eng {
            let o = match &mut self.voices[i] {
                Some(v) => v.tick(),
                None => continue,
            };
            let em = &mut self.emit[i];
            let ext = o.front * em.dir[0].tick() + o.rear * em.dir[1].tick() + o.case * em.dir[2].tick();
            em.line.push(ext);
            let im = self.tune.int_mix;
            // Interior: the exterior components through the airframe
            // (filtered), plus any recorded cockpit layers as they are.
            let int = (o.front * im[0] + o.rear * im[1] + o.case * im[2]) * self.ife[i] * em.int_g;
            let int = em.int_lp2.lp(em.int_lp.lp(int)) + o.int;
            int_eng.0 += int * em.int_l.tick();
            int_eng.1 += int * em.int_r.tick();
            em.delay = (em.delay + em.d_step).max(1.0);
            let s = em.air.lp(em.line.read(em.delay)) * em.gain.tick() * cut * em.vis.tick();
            ext_eng.0 += s * em.gl.tick();
            ext_eng.1 += s * em.gr.tick();
            send += s * em.send.tick();
        }
        // Airframe emitter (aero / ground / airframe one-shots).
        let (mut a_ext, mut a_int) = self.air.tick(&mut self.rng);
        let mut cock_shots = 0.0f32;
        let mut lis = (0.0f32, 0.0f32);
        let lis_g = 1.0 - 0.45 * ins;
        let mut ui = 0.0f32;
        let mut wext = (0.0f32, 0.0f32);
        for slot in self.pool.shots.iter_mut() {
            if let Some(s) = slot {
                match s.tick(sr, &mut self.rng) {
                    Some(y) => match s.route {
                        Route::Airframe => {
                            a_ext += y;
                            a_int += y * 0.8;
                        }
                        Route::Cockpit => cock_shots += y,
                        Route::Listener => {
                            let (l, r) = pan(s.pan);
                            lis.0 += y * l * lis_g;
                            lis.1 += y * r * lis_g;
                        }
                        Route::Ui => ui += y,
                        Route::World => {
                            let y = s.air.lp(y);
                            wext.0 += y * s.wl.tick();
                            wext.1 += y * s.wr.tick();
                        }
                    },
                    None => *slot = None,
                }
            }
        }
        let mut ext_env = (0.0f32, 0.0f32);
        let mut int_env = (0.0f32, 0.0f32);
        {
            let em = &mut self.emit[AIRFRAME];
            em.line.push(a_ext);
            em.delay = (em.delay + em.d_step).max(1.0);
            let s = em.air.lp(em.line.read(em.delay)) * em.gain.tick() * cut * em.vis.tick();
            ext_env.0 += s * em.gl.tick();
            ext_env.1 += s * em.gr.tick();
            send += s * em.send.tick();
            let ai = em.int_lp.lp(a_int);
            int_env.0 += ai * 0.7;
            int_env.1 += ai * 0.7;
        }
        // Wreckage loops (fires, scrapes) join the world-placed one-shots.
        let (fl, fr) = self.wreck.tick(&mut self.rng);
        wext.0 += fl;
        wext.1 += fr;
        send += (wext.0 + wext.1) * 0.15;
        // Heard from the cockpit: through the airframe, muffled.
        let wi = self.world_lp.lp(self.world_lp2.lp((wext.0 + wext.1) * 0.5)) * 0.35;
        let (rl, rr) = self.fdn.tick(send * 0.5);
        // Cockpit.
        let (c_env, c_sys) = self.cock.tick(&mut self.rng);
        let c_sys = (c_sys.0 + cock_shots, c_sys.1 + cock_shots);
        int_env.0 += c_env.0;
        int_env.1 += c_env.1;
        let (al, ar) = self.amb.tick(&mut self.rng);
        let amb_g = 1.0 - 0.8 * ins;
        lis.0 += al * amb_g;
        lis.1 += ar * amb_g;
        let voice = self.speaker.tick() + 0.7 * self.pa.tick();
        let com = self.radio.tick();
        let duck = (1.0 - 0.42 * self.speaker.active) * (1.0 - 0.3 * self.radio.active);

        let w = self.world_r.tick();
        let ge = self.g_eng.tick() * w * duck;
        let gv = self.g_env.tick() * w * duck;
        let gs = self.g_sys.tick() * w * duck * self.ext_sys.tick();
        let gvo = self.g_voice.tick() * w * (0.5 + 0.5 * ins);
        // The headset radio: the same inside and out (not muffled by the airframe).
        let gr = self.g_radio.tick() * w;
        let gu = self.g_ui.tick();
        let bo = self.blackout.tick();
        let wl = wext.0 * outs + wi * ins;
        let wr = wext.1 * outs + wi * ins;
        let mut l = ((ext_eng.0 * outs + int_eng.0 * ins) * ge + (ext_env.0 * outs + int_env.0 * ins + lis.0 + rl * outs + wl) * gv + c_sys.0 * gs + voice * gvo + com * gr) * bo;
        let mut r = ((ext_eng.1 * outs + int_eng.1 * ins) * ge + (ext_env.1 * outs + int_env.1 * ins + lis.1 + rr * outs + wr) * gv + c_sys.1 * gs + voice * gvo + com * gr) * bo;
        if self.muffle_fc < 19_000.0 {
            l = self.muffle_l.lp(l);
            r = self.muffle_r.lp(r);
        } else {
            self.muffle_l.z = l;
            self.muffle_r.z = r;
        }
        l += ui * gu;
        r += ui * gu;
        let gm = self.g_master.tick();
        let (mut l, mut r) = self.lim.tick(l * gm, r * gm);
        if !l.is_finite() || !r.is_finite() {
            self.nan_count += 1;
            l = 0.0;
            r = 0.0;
        }
        self.peak = self.peak.max(l.abs()).max(r.abs());
        self.samples += 1;
        (l, r)
    }
}

impl Airframe {
    fn new(sr: f32) -> Airframe {
        Airframe {
            pink: Pink::default(),
            aero_lp: Svf::new(800.0, 0.6, sr),
            aero_amp: Ramp::new(0.0),
            gear_bp: Svf::new(170.0, 0.8, sr),
            gear_amp: Ramp::new(0.0),
            flap_bp: Svf::new(520.0, 0.8, sr),
            flap_amp: Ramp::new(0.0),
            spoil_lp: Svf::new(110.0, 0.8, sr),
            spoil_amp: Ramp::new(0.0),
            buff_lp: Svf::new(38.0, 1.2, sr),
            buff_amp: Ramp::new(0.0),
            buff_mod: OnePole::new(11.0, sr),
            roll_lp: Svf::new(85.0, 0.8, sr),
            roll_amp: Ramp::new(0.0),
            grass_bp: Svf::new(260.0, 0.5, sr),
            grass_amp: Ramp::new(0.0),
            hum_bp: Svf::new(650.0, 2.0, sr),
            hum_amp: Ramp::new(0.0),
            joint_ph: 0.0,
            joint_inc: Ramp::new(0.0),
            joint_env: 0.0,
            joint_amp: 0.0,
            joint_lp: OnePole::new(70.0, sr),
            bump_rate: Ramp::new(0.0),
            bump_env: 0.0,
            squeal_ph: [Phasor::default(); 2],
            squeal_amp: Ramp::new(0.0),
            squeal_f: Ramp::new(2900.0 / sr),
            scrub_bp: Svf::new(1300.0, 1.0, sr),
            scrub_amp: Ramp::new(0.0),
            vib_ph: Phasor::default(),
            isr: 1.0 / sr,
            roll_rec: Looper::new("tyre_roll", 0.3),
            roll_rec_int: Looper::new("heavy_roll_int", 0.6),
            gravel_rec: Looper::new("tyre_gravel", 0.1),
            brake_rec: Looper::new("glider_brake", 0.2),
        }
    }

    fn control(&mut self, f: &Frame, fam: Family, sr: f32, n: u32) {
        let v = f.tas;
        let live = f.active && !f.crashed;
        let q = (v / 80.0).powi(2);
        let light = matches!(fam, Family::LightPiston | Family::Glider);
        let aero = if live { (0.05 * (v / 60.0).powi(2)).min(0.3) } else { 0.0 };
        self.aero_amp.to(aero, n);
        self.aero_lp.set((250.0 + 22.0 * v).min(7000.0), 0.6, sr);
        let gear_k = if f.profile.retract_gear { 0.11 } else { 0.03 };
        self.gear_amp.to(if live { gear_k * f.gear * q.min(4.0) } else { 0.0 }, n);
        self.flap_amp.to(if live { 0.05 * f.flaps * q.min(4.0) } else { 0.0 }, n);
        self.spoil_amp.to(if live { 0.22 * f.spoilers * q.min(4.0) } else { 0.0 }, n);
        // Transonic: shocks dancing on the wing (M ≈ 0.88..1.1) shake the
        // airframe — a low, irregular rumble that dies away once supersonic.
        let trans = smoothstep(0.86, 0.97, f.mach) * (1.0 - smoothstep(1.03, 1.15, f.mach));
        let trans = if f.on_ground { 0.0 } else { trans * if fam == Family::Fighter { 0.45 } else { 0.25 } };
        self.buff_amp.to(if live { 0.75 * f.buffet + 0.25 * f.overspeed + trans } else { 0.0 }, n);
        let gs = f.ground_speed;
        let on = f.on_ground && live;
        let roll = if on { (gs / 40.0).powf(0.8).min(1.6) } else { 0.0 };
        let heavy = if light { 0.6 } else { 1.0 };
        self.roll_amp.to(if f.paved { 0.22 * roll * heavy } else { 0.12 * roll }, n);
        self.grass_amp.to(if !f.paved && on { 0.16 * roll } else { 0.0 }, n);
        self.hum_amp.to(if on { 0.02 * (gs / 30.0).min(2.0) } else { 0.0 }, n);
        // Runway joints / centre-line lights every ~15 m (paved).
        self.joint_inc.to(if on && f.paved { gs / 15.0 / sr } else { 0.0 }, n);
        self.joint_amp = (gs / 40.0).min(1.2) * heavy;
        self.bump_rate.to(if on && !f.paved { gs * 0.9 } else { 0.0 }, n);
        // Brakes: squeal at low speed, scrub at high.
        let br = if on { f.brakes } else { 0.0 };
        let sq = br * smoothstep(0.3, 2.5, gs) * (1.0 - smoothstep(8.0, 22.0, gs));
        self.squeal_amp.to(0.03 * sq * if light { 0.6 } else { 1.0 }, n);
        self.squeal_f.to(if light { 3400.0 } else { 2600.0 } / sr, n);
        self.scrub_amp.to(0.05 * br * smoothstep(1.0, 30.0, gs), n);
        // Recorded rolling (c172p tyres on asphalt / gravel; A320 SASA gear
        // rumble for the heavies), level ~ log of ground speed like FG,
        // pitch rising gently with speed.
        let heavy_ac = matches!(fam, Family::Airliner | Family::Turboprop);
        self.roll_rec.set_sample(match fam {
            Family::Airliner | Family::Turboprop => "heavy_roll",
            Family::Glider => "glider_roll",
            _ => "tyre_roll",
        });
        // Glider air-brakes: the paddles' buffeting roar grows with speed.
        let bg = if live && fam == Family::Glider { 0.12 * f.spoilers * (v / 30.0).powi(2).min(3.0) } else { 0.0 };
        self.brake_rec.control(bg, (0.8 + v / 120.0).min(1.3), sr, n);
        let kspd = smoothstep(0.5, 6.0, gs) * (1.0 + (gs / 10.0).max(1.0).ln() * 0.6);
        let rg = if on && f.paved { kspd * if heavy_ac { 0.16 } else { 0.22 } } else { 0.0 };
        let rp = if heavy_ac { 0.8 + gs / 120.0 } else { 0.75 + gs / 70.0 };
        self.roll_rec.control(rg, rp.min(1.4), sr, n);
        let gg = if on && !f.paved { kspd * 0.3 } else { 0.0 };
        self.gravel_rec.control(gg, (0.8 + gs / 60.0).min(1.5), sr, n);
        self.roll_rec_int.control(if on && heavy_ac { kspd * 0.18 } else { 0.0 }, (0.85 + gs / 100.0).min(1.3), sr, n);
    }

    /// (exterior mono, interior mono)
    #[inline]
    fn tick(&mut self, rng: &mut Rng) -> (f32, f32) {
        let w = rng.bi();
        let pk = self.pink.tick(w);
        let aero = self.aero_lp.lp(pk) * self.aero_amp.tick();
        let gear = self.gear_bp.bpn(pk) * self.gear_amp.tick();
        let flap = self.flap_bp.bpn(pk) * self.flap_amp.tick();
        let sm = 0.5 + self.buff_mod.lp(rng.bi() * 3.0).abs().min(1.0);
        let spoil = self.spoil_lp.lp(pk) * self.spoil_amp.tick() * sm;
        let buff = self.buff_lp.bpn(pk) * self.buff_amp.tick() * (0.3 + sm) * 2.0;
        let roll = self.roll_lp.lp(pk) * self.roll_amp.tick() * 1.6;
        let ga = self.grass_amp.tick();
        let grass = self.grass_bp.bpn(pk) * ga * (0.6 + 0.4 * w.abs());
        let hum = self.hum_bp.bpn(w) * self.hum_amp.tick();
        self.joint_ph += self.joint_inc.tick();
        if self.joint_ph >= 1.0 {
            self.joint_ph -= 1.0;
            self.joint_env = 1.0;
        }
        let br = self.bump_rate.tick();
        if br > 0.0 && rng.uni() < br * self.isr {
            self.bump_env = 0.4 + rng.uni();
        }
        self.joint_env *= 0.9975;
        self.bump_env *= 0.9985;
        let thumps = self.joint_lp.lp(w * (self.joint_env * self.joint_amp + self.bump_env * ga * 3.0)) * 2.5;
        let vib = sin_turns(self.vib_ph.tick(7.0 * self.isr));
        let fi = self.squeal_f.tick() * (1.0 + 0.012 * vib);
        let p0 = self.squeal_ph[0].tick(fi);
        let p1 = self.squeal_ph[1].tick(fi * 2.01);
        let squeal = (sin_turns(p0) + 0.3 * sin_turns(p1)) * self.squeal_amp.tick() * (0.7 + 0.3 * w);
        let scrub = self.scrub_bp.bpn(w) * self.scrub_amp.tick();
        let rr = self.roll_rec.tick() + self.gravel_rec.tick() + self.brake_rec.tick();
        let ri = self.roll_rec_int.tick();
        let ext = aero + gear + flap + spoil + buff * 0.4 + roll * 0.7 + grass * 0.7 + hum + thumps + squeal + scrub + rr;
        let int = gear * 0.7 + flap * 0.3 + spoil * 1.0 + buff * 1.3 + roll * 1.2 + grass * 1.0 + hum * 0.4 + thumps * 1.5 + squeal * 0.4 + scrub * 0.4 + rr * 0.5 + ri;
        (ext, int)
    }
}

impl Cockpit {
    fn new(sr: f32) -> Cockpit {
        Cockpit {
            pink: Pink::default(),
            wind_lp: Svf::new(600.0, 0.6, sr),
            wind_hp: OnePole::new(60.0, sr),
            wind_amp: Ramp::new(0.0),
            whistle: Svf::new(1500.0, 9.0, sr),
            whistle_amp: Ramp::new(0.0),
            whistle_f: 1500.0,
            gust_lfo: OnePole::new(0.7, sr),
            rain_rate: Ramp::new(0.0),
            rain_res: [Svf::new(2500.0, 6.0, sr), Svf::new(4100.0, 6.0, sr), Svf::new(6200.0, 5.0, sr)],
            rain_hiss: Svf::new(5000.0, 0.7, sr),
            rain_hiss_amp: Ramp::new(0.0),
            shake_lp: Svf::new(28.0, 1.0, sr),
            shake_amp: Ramp::new(0.0),
            rattle_bp: Svf::new(420.0, 3.0, sr),
            horn_ph: [Phasor::default(); 3],
            horn_amp: Ramp::new(0.0),
            horn_kind: 0,
            beep_ph: Phasor::default(),
            gate_ph: Phasor::default(),
            shaker_ph: Phasor::default(),
            shaker_env: 0.0,
            shaker_amp: Ramp::new(0.0),
            shaker_bp: Svf::new(190.0, 2.0, sr),
            shaker_lp: OnePole::new(70.0, sr),
            gwarn_amp: Ramp::new(0.0),
            gwarn_ph: Phasor::default(),
            clack_ph: Phasor::default(),
            clack_amp: Ramp::new(0.0),
            clack_env: 0.0,
            clack_bp: Svf::new(2300.0, 5.0, sr),
            vario_ph: Phasor::default(),
            vario_gate: Phasor::default(),
            vario_f: Ramp::new(600.0 / sr),
            vario_rate: 2.0,
            vario_amp: Ramp::new(0.0),
            vario_beep: true,
            vario_env: 0.0,
            ident: [0; 4],
            ident_keyer: crate::nav::morse::Keyer::default(),
            ident_t: 0.0,
            ident_ph: Phasor::default(),
            ident_amp: Ramp::new(0.0),
            ident_env: 0.0,
            gear_m_ph: [Phasor::default(); 2],
            gear_m_amp: Ramp::new(0.0),
            gear_m_f: Ramp::new(400.0 / sr),
            flap_m_ph: [Phasor::default(); 2],
            flap_m_amp: Ramp::new(0.0),
            flap_m_f: Ramp::new(230.0 / sr),
            motor_hiss: Svf::new(3000.0, 0.8, sr),
            motor_lp: Svf::new(1800.0, 0.7, sr),
            flap_lp: Svf::new(1500.0, 0.7, sr),
            pack_bp: Svf::new(1300.0, 0.45, sr),
            pack_amp: Ramp::new(0.0),
            fan_ph: Phasor::default(),
            isr: 1.0 / sr,
            wind_rec: Looper::new("wind_light", 0.0),
            horn_rec: Looper::new("c172_stall", 0.0),
            flap_rec: Looper::new("flap_motor_light", 0.0),
            gear_rec: Looper::new("gear_fighter", 0.0),
        }
    }

    fn control(&mut self, f: &Frame, fam: Family, tn: &Tune, sr: f32, n: u32, _dt: f32) {
        let live = f.active && !f.crashed;
        // Navaid ident: a new station restarts the Morse sequence.
        if f.ident != self.ident {
            self.ident = f.ident;
            let s: String = f.ident.iter().filter(|b| **b != 0).map(|b| *b as char).collect();
            self.ident_keyer = crate::nav::morse::Keyer::new(&s);
            // (Half a second of silence first: the station "comes in".)
            self.ident_t = -0.5;
        }
        self.ident_amp.to(if f.active && !f.paused && f.ident[0] != 0 { 0.05 * f.ident_gain } else { 0.0 }, n);
        let v = if live { f.tas } else { 0.0 };
        // Wind over the cockpit: level and brightness rise with speed;
        // sideslip makes it louder.
        let slip = 1.0 + 1.5 * f.beta.abs().min(0.5);
        let vref = match fam {
            Family::Glider => 35.0,
            Family::LightPiston | Family::Warbird => 55.0,
            Family::Turboprop => 90.0,
            Family::Fighter => 150.0,
            Family::Airliner => 120.0,
            Family::Helicopter => 60.0,
            Family::Drone => 25.0,
        };
        let wa = 0.05 * tn.wind * (v / vref).powf(1.7).min(5.0) * slip;
        self.wind_amp.to(wa, n);
        // Supersonic: the canopy sits in a hissing, turbulent boundary
        // layer behind the bow shock — broader and brighter, no cavity
        // whistle (the airflow tones need subsonic flow over the gaps).
        let sup = smoothstep(0.98, 1.25, f.mach);
        self.wind_lp.set((150.0 + 20.0 * v).min(tn.wind_lp * (1.0 + 0.7 * sup)), 0.6, sr);
        self.whistle_f = 900.0 + 9.0 * v + 3000.0 * f.overspeed;
        self.whistle.set(self.whistle_f.min(9000.0), 10.0, sr);
        self.whistle_amp.to(tn.whistle * 0.02 * (v / vref).powi(2).min(4.0) * (1.0 + 4.0 * f.overspeed) * (1.0 - 0.85 * sup), n);
        // Airliner flight deck: air-conditioning packs and avionics fans.
        let pack = match fam {
            Family::Airliner => 0.02,
            Family::Turboprop => 0.012,
            Family::Fighter => 0.012,
            _ => 0.0,
        };
        self.pack_amp.to(if f.active && !f.crashed { pack } else { 0.0 }, n);
        // Rain on the canopy: drop rate rises with airspeed.
        let rr = if live || f.active { f.rain * (250.0 + 22.0 * v).min(3500.0) } else { 0.0 };
        self.rain_rate.to(rr, n);
        self.rain_hiss_amp.to(0.012 * f.rain * (1.0 + v / 60.0).min(3.0) + 0.006 * f.snow * (1.0 + v / 60.0).min(3.0), n);
        self.shake_amp.to(if live { 0.45 * f.shake.min(1.2) } else { 0.0 }, n);
        // Stall warning by type.
        let sw = f.stall_warn && live;
        self.horn_kind = match fam {
            Family::LightPiston => 1,
            Family::Fighter => 2,
            Family::Airliner | Family::Turboprop => 3,
            // Helicopter: low rotor-RPM horn (continuous).
            Family::Helicopter => 4,
            _ => 0,
        };
        let horn_on = sw && matches!(self.horn_kind, 2 | 4);
        self.horn_amp.to(if horn_on { 0.09 } else { 0.0 }, n);
        self.shaker_amp.to(if sw && self.horn_kind == 3 { 0.32 } else { 0.0 }, n);
        self.gwarn_amp.to(if f.gear_warn && live { 0.07 } else { 0.0 }, n);
        let clacker = matches!(fam, Family::Airliner | Family::Turboprop) && f.overspeed > 0.0 && live;
        self.clack_amp.to(if clacker { 0.18 } else { 0.0 }, n);
        // Glider vario: beeps rising in pitch and rate in lift, a low
        // continuous drone in strong sink, quiet in between.
        match f.vario {
            Some(vz) if live => {
                if vz > 0.2 {
                    self.vario_beep = true;
                    self.vario_f.to((560.0 + 170.0 * vz.min(6.0)) / sr, n);
                    self.vario_rate = 1.6 + 1.25 * vz.min(6.0);
                    self.vario_amp.to(0.085, n);
                } else if vz < -1.6 {
                    self.vario_beep = false;
                    self.vario_f.to((330.0 + 22.0 * (vz + 1.6).max(-6.0)) / sr, n);
                    self.vario_amp.to(0.05 * smoothstep(-1.6, -2.4, vz), n);
                } else {
                    self.vario_amp.to(0.0, n);
                }
            }
            _ => self.vario_amp.to(0.0, n),
        }
        // Actuator motors.
        let gm = (f.gear_rate.abs() > 0.01 && live) as u8 as f32;
        let (gf, ga) = match fam {
            Family::Warbird => (150.0, 0.05),
            Family::Fighter => (520.0, 0.035),
            _ => (410.0, 0.04),
        };
        self.gear_m_amp.to(gm * ga, n);
        self.gear_m_f.to(gf / sr, n);
        let fm = (f.flap_rate.abs() > 0.004 && live) as u8 as f32;
        let (ff, fa) = match fam {
            Family::LightPiston | Family::Glider => (235.0, 0.03),
            Family::Airliner | Family::Turboprop => (640.0, 0.016),
            _ => (330.0, 0.02),
        };
        self.flap_m_amp.to(fm * fa, n);
        self.flap_m_f.to(ff / sr, n);

        // ---- recorded layers
        // Wind: fgdata wind1 (light / warbird), A320 SASA cockpit wind
        // (airliners, turboprop, fighter), ASK21 whoosh (glider); level
        // follows the procedural model, pitch rises with airspeed.
        let wname = match fam {
            Family::Glider => "glider_wind",
            Family::LightPiston | Family::Warbird => "wind_light",
            _ => "wind_airliner",
        };
        self.wind_rec.set_sample(wname);
        let wpitch = match fam {
            Family::Glider => 0.55 + v / 45.0,
            Family::LightPiston | Family::Warbird => 0.6 + v / 110.0,
            _ => 0.55 + v / 260.0,
        };
        let wk = match fam {
            Family::Glider => 1.5,
            Family::LightPiston | Family::Warbird => 2.2,
            _ => 1.6,
        };
        self.wind_rec.control(wa * wk * (1.0 - 0.3 * sup), wpitch.clamp(0.4, 2.0), sr, n);
        // C172 stall horn (recorded reed horn) for the light single.
        let hr = if sw && fam == Family::LightPiston { 0.22 } else { 0.0 };
        self.horn_rec.control(hr, 1.0, sr, n);
        // Motors: recorded loops while moving (the synth tones stay, quieter).
        let (fname, fgain) = match fam {
            Family::Airliner | Family::Turboprop => ("flap_motor_heavy", 0.09),
            _ => ("flap_motor_light", 0.1),
        };
        self.flap_rec.set_sample(fname);
        self.flap_rec.control(fm * fgain, 1.0, sr, n);
        self.gear_rec.control(if fam == Family::Fighter { gm * 0.12 } else { 0.0 }, 1.0, sr, n);
    }

    /// ((env stereo), (systems stereo))
    #[inline]
    fn tick(&mut self, rng: &mut Rng) -> ((f32, f32), (f32, f32)) {
        let sr_k = self.isr;
        let w = rng.bi();
        let w2 = rng.bi();
        let pk = self.pink.tick(w);
        let g = 1.0 + 0.35 * self.gust_lfo.lp(w2 * 6.0).clamp(-1.0, 1.0);
        let wind = self.wind_hp.hp(self.wind_lp.lp(pk)) * self.wind_amp.tick() * g * 1.6;
        let whistle = self.whistle.bpn(w) * self.whistle_amp.tick() * g;
        // Rain drops: each a click ringing one of three canopy modes.
        let rate = self.rain_rate.tick();
        let mut drop = 0.0;
        let mut k = 3;
        if rate > 0.0 && rng.uni() < rate * sr_k {
            drop = rng.uni().powi(3) * 1.2 * if rng.uni() < 0.5 { 1.0 } else { -1.0 };
            // Each drop rings the canopy at its own pitch.
            k = (rng.next_u32() % 3) as usize;
            let f = 1400.0 * (1.0 + 4.0 * rng.uni() * rng.uni());
            self.rain_res[k].set(f, 2.5 + 3.0 * rng.uni(), 1.0 / sr_k);
        }
        let mut rain = 0.0;
        for (i, r) in self.rain_res.iter_mut().enumerate() {
            rain += r.bp(if i == k { drop } else { 0.0 });
        }
        let rain = rain * 0.9 + self.rain_hiss.bpn(w2) * self.rain_hiss_amp.tick();
        let sh = self.shake_amp.tick();
        let shake = self.shake_lp.bpn(pk) * sh * 1.5 + self.rattle_bp.bpn(w) * sh * 0.05 * (w2 > 0.6) as u8 as f32;
        let pa = self.pack_amp.tick();
        let pack = if pa > 1e-6 { (self.pack_bp.bpn(pk) + 0.12 * sin_turns(self.fan_ph.tick(398.0 * sr_k))) * pa } else { 0.0 };
        // Recorded wind under the modelled one (decorrelated a little by the
        // modelled part, which differs left / right).
        let wr = self.wind_rec.tick();
        let env_l = wind * 0.55 + wr * 0.8 + whistle * 0.7 + rain * 0.8 + shake + pack;
        let env_r = wind * 0.55 + wr * 0.8 + whistle * 0.5 + rain * 0.6 + shake + pack * 0.9;

        // ---- systems
        let mut sys = 0.0f32;
        let ha = self.horn_amp.tick();
        if ha > 1e-5 {
            match self.horn_kind {
                1 => {
                    // Reed horn: slightly unstable pitch, rich harmonics.
                    let f = (2050.0 + 60.0 * self.gust_lfo.z) * sr_k;
                    let a = self.horn_ph[0].tick(f);
                    let b = self.horn_ph[1].tick(f * 2.0);
                    let c = self.horn_ph[2].tick(f * 3.0);
                    sys += (sin_turns(a) + 0.5 * sin_turns(b) + 0.25 * sin_turns(c)) * ha;
                }
                4 => {
                    // Low rotor RPM: a steady two-tone horn.
                    let a = self.horn_ph[0].tick(600.0 * sr_k);
                    let b = self.horn_ph[1].tick(1200.0 * sr_k);
                    let c = self.horn_ph[2].tick(1800.0 * sr_k);
                    sys += (sin_turns(a) + 0.45 * sin_turns(b) + 0.2 * sin_turns(c)) * ha * 0.8;
                }
                _ => {
                    // Fighter AoA warning: fast interrupted tone.
                    let gate = self.gate_ph.tick(9.0 * sr_k);
                    let p = self.beep_ph.tick(1150.0 * sr_k);
                    let on = smoothstep(0.0, 0.05, gate) * smoothstep(0.55, 0.45, gate);
                    sys += (sin_turns(p) + 0.3 * sin_turns(p * 3.0)) * ha * on;
                }
            }
        }
        let sa = self.shaker_amp.tick();
        if sa > 1e-5 {
            // Stick shaker: an eccentric motor rattling the column at ~26 Hz.
            if self.shaker_ph.tick(26.0 * sr_k) > self.shaker_ph.p {
                self.shaker_env = 1.0;
            }
            self.shaker_env *= 0.996;
            let x = rng.bi() * self.shaker_env;
            sys += (self.shaker_bp.bpn(x) * 0.8 + self.shaker_lp.lp(x) * 4.0) * sa;
        }
        let gwa = self.gwarn_amp.tick();
        if gwa > 1e-5 {
            let gate = self.gate_ph.tick(2.0 * sr_k);
            let p = self.gwarn_ph.tick(360.0 * sr_k);
            let saw = 2.0 * p - 1.0;
            let on = smoothstep(0.0, 0.03, gate) * smoothstep(0.6, 0.55, gate);
            sys += self.motor_hiss.lp(saw) * gwa * on * 2.0;
        }
        let ca = self.clack_amp.tick();
        if ca > 1e-5 {
            if self.clack_ph.tick(9.0 * sr_k) > self.clack_ph.p {
                self.clack_env = 1.0;
            }
            self.clack_env *= 0.985;
            sys += self.clack_bp.bpn(rng.bi() * self.clack_env) * ca * 3.0;
        }
        let va = self.vario_amp.tick();
        let vf = self.vario_f.tick();
        if va > 1e-5 {
            let p = self.vario_ph.tick(vf);
            let tone = sin_turns(p) + 0.28 * sin_turns(p * 3.0) + 0.12 * sin_turns(p * 5.0);
            let gate = if self.vario_beep {
                let gp = self.vario_gate.tick(self.vario_rate * sr_k);
                if gp < 0.55 { 1.0 } else { 0.0 }
            } else {
                1.0
            };
            self.vario_env += (gate - self.vario_env) * 0.006;
            sys += tone * va * self.vario_env;
        } else {
            self.vario_env *= 0.99;
        }
        let ia = self.ident_amp.tick();
        if ia > 1e-5 {
            self.ident_t += sr_k;
            let key = if self.ident_keyer.key(self.ident_t) { 1.0 } else { 0.0 };
            // ~4 ms attack / release: no key clicks.
            self.ident_env += (key - self.ident_env) * (sr_k / 0.004).min(1.0);
            let p = self.ident_ph.tick(crate::nav::morse::TONE_HZ * sr_k);
            sys += sin_turns(p) * ia * self.ident_env;
        } else {
            self.ident_env = 0.0;
        }
        let gma = self.gear_m_amp.tick();
        let gmf = self.gear_m_f.tick();
        if gma > 1e-5 {
            let wob = 1.0 + 0.02 * self.gust_lfo.z;
            let a = self.gear_m_ph[0].tick(gmf * wob);
            let b = self.gear_m_ph[1].tick(gmf * 2.98 * wob);
            sys += (self.motor_lp.lp(2.0 * a - 1.0) * 0.5 + sin_turns(b) * 0.4 + self.motor_hiss.bpn(w) * 0.6) * gma;
        }
        let fma = self.flap_m_amp.tick();
        let fmf = self.flap_m_f.tick();
        if fma > 1e-5 {
            let a = self.flap_m_ph[0].tick(fmf);
            let b = self.flap_m_ph[1].tick(fmf * 4.0);
            sys += (self.flap_lp.lp(2.0 * a - 1.0) * 0.6 + sin_turns(b) * 0.3) * fma;
        }
        sys += self.horn_rec.tick() + self.flap_rec.tick() + self.gear_rec.tick();
        ((env_l, env_r), (sys, sys))
    }
}

impl Ambience {
    fn new(sr: f32) -> Ambience {
        Ambience {
            pink: Pink::default(),
            city_lp: Svf::new(220.0, 0.6, sr),
            city_amp: Ramp::new(0.0),
            traffic_bp: Svf::new(900.0, 0.7, sr),
            traffic_mod: OnePole::new(0.3, sr),
            wind_lp: Svf::new(350.0, 0.6, sr),
            wind_amp: Ramp::new(0.0),
            wind_mod: OnePole::new(0.5, sr),
            rain_bp: Svf::new(3500.0, 0.6, sr),
            rain_amp: Ramp::new(0.0),
            drop_rate: Ramp::new(0.0),
            drop_hp: OnePole::new(1500.0, sr),
            drop_env: 0.0,
            drop_amp: 0.0,
            drop_pan: 0.0,
            bird_rate: 0.0,
            isr: 1.0 / sr,
            move_lp: Svf::new(500.0, 0.6, sr),
            move_amp: Ramp::new(0.0),
        }
    }
    fn control(&mut self, f: &Frame, _sr: f32, n: u32, _dt: f32) {
        let near = if f.active { smoothstep(350.0, 15.0, f.ear_agl) } else { 0.0 };
        self.city_amp.to(0.028 * f.urban.max(0.15) * near, n);
        self.wind_amp.to(0.03 * (f.wind_surface / 8.0 + f.gust / 10.0).min(2.0) * near, n);
        self.rain_amp.to(0.05 * f.rain * smoothstep(1500.0, 50.0, f.ear_agl), n);
        self.drop_rate.to(f.rain * 900.0 * near, n);
        // A camera flying with the aircraft hears the wind of its motion.
        let es = f.ear_vel.length();
        self.move_amp.to(if f.active && f.interior < 0.5 { (0.018 * (es / 60.0).powf(1.6)).min(0.08) } else { 0.0 }, n);
        self.move_lp.set((200.0 + 6.0 * es).min(3000.0), 0.6, _sr);
        self.bird_rate = if f.rain < 0.1 { 0.35 * f.day * near * (1.0 - 0.6 * f.urban) } else { 0.0 };
    }
    #[inline]
    fn tick(&mut self, rng: &mut Rng) -> (f32, f32) {
        let w = rng.bi();
        let pk = self.pink.tick(w);
        let city = self.city_lp.lp(pk) * self.city_amp.tick() * 2.0;
        let tm = 0.5 + self.traffic_mod.lp(rng.bi() * 12.0).abs().min(1.0);
        let traffic = self.traffic_bp.bpn(pk) * self.city_amp.v * 0.3 * tm;
        let wm = 0.4 + self.wind_mod.lp(rng.bi() * 8.0).abs().min(1.2);
        let wind = self.wind_lp.lp(pk) * self.wind_amp.tick() * wm * 1.5;
        let rain = self.rain_bp.bpn(w) * self.rain_amp.tick();
        let rate = self.drop_rate.tick();
        if rate > 0.0 && rng.uni() < rate * self.isr {
            self.drop_env = 1.0;
            self.drop_amp = rng.uni().powi(2) * 0.12;
            self.drop_pan = rng.bi();
        }
        self.drop_env *= 0.97;
        let d = self.drop_hp.hp(rng.bi() * self.drop_env * self.drop_amp);
        let (dl, dr) = pan(self.drop_pan);
        let mv = self.move_lp.lp(pk) * self.move_amp.tick() * wm;
        let c = city + traffic + wind + rain + mv;
        (c + d * dl, c * 0.97 + d * dr)
    }
}
