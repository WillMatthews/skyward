//! Rotorcraft synthesis: the turboshaft helicopter and the electric
//! multirotor. Same three-component contract as `engines.rs`:
//! `front` (rotor loading / blade slap — radiated ahead and below),
//! `rear` (exhaust), `case` (everything heard all round: rotor thump,
//! gearbox, gas generator, tail rotor).
//!
//! Helicopter (EC130 B4 class):
//! * main rotor — a one-revolution wavetable with one loading pulse per
//!   blade (3 blades at Nr: 19.3 Hz blade passage at 386 rpm), slightly
//!   different blade to blade (tracking / balance), giving the low "thump"
//!   and its harmonics; blade-synchronous broadband "swish";
//! * blade slap (blade-vortex interaction) — a sharp, asymmetric pressure
//!   doublet per blade passage (a second, band-limited table up to ~2 kHz)
//!   plus a crack of filtered noise at each passage, its strength from the
//!   rotor model (shallow descents, decelerations, tight turns);
//! * Arriel turboshaft — gas-generator whine (shaft order + compressor
//!   tones) on Ng, combustion / exhaust roar on power, starter whine, and
//!   silence after a flameout while the rotor (and its sound) runs on;
//! * main gearbox — gear-mesh whine on Nr, louder with torque;
//! * Fenestron — 10 unevenly spaced blades at 3,568 rpm: the blade-pass tone
//!   (~595 Hz) spread into shaft-order sidebands, louder with blade pitch.
//!
//! Drone: per motor a tri-blade prop table at its own speed (the four motors
//! beat against each other), the brushless motor's electrical whine
//! (7 pole pairs), broadband prop noise rising steeply with thrust, and
//! prop-wash roughness.

use super::dsp::*;
use super::engines::{EngCtl, EngOut};
use super::params::Profile;

/// Gas-generator shaft speed at 100 % Ng (Arriel 2 class), rpm.
const NG_RPM_100: f32 = 52_110.0;

/// One-revolution pressure tables for a rotor of `blades` blades:
/// (loading "thump", impulsive "slap" doublet). Blade-to-blade scatter in
/// amplitude and spacing (`spacing` adds the deliberately uneven spacing of
/// a Fenestron).
fn rotor_tables(blades: u32, harm_soft: usize, harm_hard: usize, spacing: f32, seed: u32) -> (Table, Table) {
    let mut rng = Rng::new(seed);
    let n = TABLE_N;
    let b = blades.max(1) as usize;
    let mut soft = vec![0.0f32; n];
    let mut hard = vec![0.0f32; n];
    let var: Vec<f32> = (0..b).map(|_| 1.0 + 0.05 * rng.bi()).collect();
    let off: Vec<f32> = (0..b).map(|k| 0.003 * rng.bi() + spacing * ((k as f32 * 2.399).sin()) / b as f32).collect();
    for (i, (s, h)) in soft.iter_mut().zip(hard.iter_mut()).enumerate() {
        let t = i as f32 / n as f32;
        for k in 0..b {
            let c = (k as f32 + 0.5) / b as f32 + off[k];
            let mut d = t - c;
            d -= d.round();
            // Loading / thickness pulse: a broad dip per blade.
            let ws = 0.11 / b as f32;
            *s += -var[k] * (-(d * d) / (2.0 * ws * ws)).exp();
            // BVI: a sharp positive spike then a negative lobe (a few ms).
            let wh = 0.0013;
            *h += var[k] * ((-(d * d) / (2.0 * wh * wh)).exp() - 0.7 * (-((d - 2.2 * wh) * (d - 2.2 * wh)) / (2.0 * (1.6 * wh) * (1.6 * wh))).exp());
        }
    }
    (Table::from_wave(&soft, harm_soft), Table::from_wave(&hard, harm_hard))
}

// ------------------------------------------------------------------ helicopter

pub struct Heli {
    sr: f32,
    rng: Rng,
    pink: Pink,
    blades: f32,
    // main rotor
    thump: Table,
    slap: Table,
    ph: Phasor,
    inc: Ramp,
    thump_amp: Ramp,
    slap_amp: Ramp,
    swish_amp: Ramp,
    swish_bp: Svf,
    crack_bp: Svf,
    crack_env: f32,
    crack_amp: f32,
    last_q: f32,
    sub_lp: OnePole,
    // turboshaft
    gg_ph: [Phasor; 3],
    gg_inc: Ramp,
    gg_amp: Ramp,
    comp_amp: Ramp,
    hiss: Svf,
    hiss_amp: Ramp,
    roar: Svf,
    roar_lp: OnePole,
    roar_amp: Ramp,
    roar_lfo: OnePole,
    start_ph: Phasor,
    start_amp: Ramp,
    // gearbox
    mgb_ph: [Phasor; 2],
    mgb_inc: Ramp,
    mgb_amp: Ramp,
    // fenestron
    fen: Table,
    fen_ph: Phasor,
    fen_inc: Ramp,
    fen_amp: Ramp,
    fen_bb: Svf,
    fen_bb_amp: Ramp,
    rev_hz: f32,
}

impl Heli {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Heli {
        let seed = p.type_id ^ (idx as u32 * 2_654_435_761);
        let (thump, slap) = rotor_tables(p.blades.max(2), 48, 420, 0.0, seed);
        let (fen, _) = rotor_tables(p.fan_blades.max(2), 40, 8, 0.6, seed ^ 0x55aa);
        Heli {
            sr,
            rng: Rng::new(seed),
            pink: Pink::default(),
            blades: p.blades.max(2) as f32,
            thump,
            slap,
            ph: Phasor::default(),
            inc: Ramp::new(0.0),
            thump_amp: Ramp::new(0.0),
            slap_amp: Ramp::new(0.0),
            swish_amp: Ramp::new(0.0),
            swish_bp: Svf::new(520.0, 0.7, sr),
            crack_bp: Svf::new(1500.0, 0.9, sr),
            crack_env: 0.0,
            crack_amp: 0.0,
            last_q: 0.0,
            sub_lp: OnePole::new(60.0, sr),
            gg_ph: [Phasor::default(); 3],
            gg_inc: Ramp::new(0.0),
            gg_amp: Ramp::new(0.0),
            comp_amp: Ramp::new(0.0),
            hiss: Svf::new(4500.0, 0.8, sr),
            hiss_amp: Ramp::new(0.0),
            roar: Svf::new(300.0, 0.6, sr),
            roar_lp: OnePole::new(80.0, sr),
            roar_amp: Ramp::new(0.0),
            roar_lfo: OnePole::new(1.5, sr),
            start_ph: Phasor::default(),
            start_amp: Ramp::new(0.0),
            mgb_ph: [Phasor::default(); 2],
            mgb_inc: Ramp::new(0.0),
            mgb_amp: Ramp::new(0.0),
            fen,
            fen_ph: Phasor::default(),
            fen_inc: Ramp::new(0.0),
            fen_amp: Ramp::new(0.0),
            fen_bb: Svf::new(2500.0, 0.6, sr),
            fen_bb_amp: Ramp::new(0.0),
            rev_hz: 0.0,
        }
    }

    pub fn control(&mut self, c: &EngCtl, _p: &Profile, n: u32) {
        let r = &c.rotor;
        let sr = self.sr;
        if c.warp > 0.0 {
            self.rev_hz = r.nr_rpm.max(0.0) / 60.0;
        }
        let dead = c.crashed;
        let rev = if dead { 0.0 } else { self.rev_hz };
        let spin = smoothstep(0.3, 4.0, rev);
        self.inc.to(rev / sr, n);
        // Loading noise grows with blade loading and tip speed.
        let tip = (rev / 6.43).clamp(0.0, 1.2);
        let load = r.load.clamp(0.0, 2.0);
        self.thump_amp.to(spin * tip * tip * (0.1 + 0.2 * load), n);
        self.swish_amp.to(spin * tip * tip * (0.03 + 0.05 * load) * (1.0 + 0.6 * c.tas.min(80.0) / 80.0), n);
        self.swish_bp.set((rev * self.blades * 28.0).clamp(250.0, 900.0), 0.7, sr);
        let slap = if dead { 0.0 } else { r.slap.clamp(0.0, 1.3) };
        self.slap_amp.to(spin * tip * tip * 1.1 * slap, n);
        self.crack_amp = spin * 0.45 * slap * slap;
        // Turboshaft: gas generator on Ng, combustion on power.
        let ng = if dead { 0.0 } else { r.ng.clamp(0.0, 1.1) };
        let lit = c.e.running && !dead;
        let gg_hz = ng * NG_RPM_100 / 60.0;
        self.gg_inc.to(gg_hz / sr, n);
        let turn = smoothstep(0.08, 0.6, ng);
        self.gg_amp.to(0.035 * turn * (0.5 + 0.5 * ng), n);
        self.comp_amp.to(0.022 * turn * ng * ng, n);
        self.hiss.set((3000.0 + 3500.0 * ng).min(sr * 0.45), 0.7, sr);
        self.hiss_amp.to(0.035 * turn * ng * ng, n);
        let pw = r.power.clamp(0.0, 1.2);
        self.roar_amp.to(if lit { 0.03 + 0.12 * pw.powf(1.2) } else { 0.0 }, n);
        // Starter whine while the core spools without fire (start / relight).
        let starting = !lit && ng > 0.05 && ng < 0.6 && !dead;
        self.start_amp.to(if starting { 0.03 } else { 0.0 }, n);
        // Main gearbox: bevel / epicyclic mesh orders of the rotor speed.
        self.mgb_inc.to(rev * 160.0 / sr, n);
        self.mgb_amp.to(spin * tip * (0.006 + 0.022 * r.torque.clamp(0.0, 1.2)), n);
        // Fenestron.
        let trev = if dead { 0.0 } else { r.tail_rpm / 60.0 };
        self.fen_inc.to(trev / sr, n);
        let tt = (trev / 59.5).clamp(0.0, 1.2);
        let pitch = (r.tail_pitch.abs() / 20.0).clamp(0.0, 1.5);
        self.fen_amp.to(tt * tt * (0.02 + 0.05 * pitch), n);
        self.fen_bb_amp.to(tt * tt * (0.01 + 0.03 * pitch), n);
    }

    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let w = self.rng.bi();
        let pk = self.pink.tick(w);
        let p = self.ph.tick(self.inc.tick());
        // Main rotor.
        let thump = self.thump.at(p) * self.thump_amp.tick();
        let sub = self.sub_lp.lp(thump) * 1.2;
        let bl = sin_turns(p * self.blades);
        let swish = self.swish_bp.bpn(pk) * (0.45 + 0.55 * bl * bl) * self.swish_amp.tick();
        let slap = self.slap.at(p) * self.slap_amp.tick();
        // A crack of broadband noise at each blade passage when slapping.
        let q = (p * self.blades + 0.5).fract();
        if q < self.last_q && self.crack_amp > 1e-4 {
            self.crack_env = self.crack_amp * (0.8 + 0.4 * self.rng.uni());
        }
        self.last_q = q;
        let crack = self.crack_bp.bpn(w) * self.crack_env;
        self.crack_env *= 0.9965;
        // Turboshaft.
        let inc = self.gg_inc.tick();
        let a = self.gg_ph[0].tick(inc);
        let b = self.gg_ph[1].tick(inc * 4.6);
        let d = self.gg_ph[2].tick(inc * 6.17);
        let gg = sin_turns(a) * self.gg_amp.tick() + (0.7 * sin_turns(b) + 0.5 * sin_turns(d)) * self.comp_amp.tick();
        let hiss = self.hiss.bpn(w) * self.hiss_amp.tick();
        let lfo = 1.0 + 0.25 * self.roar_lfo.lp(w * 20.0).clamp(-1.0, 1.0);
        let roar = (self.roar.lp(pk) * 1.3 + self.roar_lp.lp(pk) * 1.6) * self.roar_amp.tick() * lfo;
        let st = sin_turns(self.start_ph.tick(inc * 0.11)) * self.start_amp.tick();
        // Gearbox and Fenestron.
        let mi = self.mgb_inc.tick();
        let mgb = (sin_turns(self.mgb_ph[0].tick(mi)) + 0.5 * sin_turns(self.mgb_ph[1].tick(mi * 2.63))) * self.mgb_amp.tick();
        let fp = self.fen_ph.tick(self.fen_inc.tick());
        let fen = self.fen.at(fp) * self.fen_amp.tick() + self.fen_bb.bpn(w) * self.fen_bb_amp.tick() * (0.7 + 0.3 * sin_turns(fp * 10.0));
        EngOut { front: slap + crack + swish, rear: roar + hiss * 0.4, case: thump + sub + gg + hiss * 0.6 + st + mgb + fen, int: 0.0 }
    }
}

// ------------------------------------------------------------------ drone

pub struct Drone {
    sr: f32,
    rng: Rng,
    soft: Table,
    hard: Table,
    ph: Phasor,
    inc: Ramp,
    tone_amp: Ramp,
    mach_mix: Ramp,
    elec_ph: [Phasor; 2],
    elec_amp: Ramp,
    poles: f32,
    bb: Svf,
    bb_amp: Ramp,
    wash_lp: OnePole,
    wash_amp: Ramp,
    wash_bp: Svf,
    jit: OnePole,
    rev_hz: f32,
}

impl Drone {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Drone {
        let seed = p.type_id ^ (idx as u32 * 40_503);
        let (soft, hard) = rotor_tables(p.blades.max(2), 24, 30, 0.0, seed);
        Drone {
            sr,
            rng: Rng::new(seed),
            soft,
            hard,
            ph: Phasor { p: (idx as f32 * 0.37).fract() },
            inc: Ramp::new(0.0),
            tone_amp: Ramp::new(0.0),
            mach_mix: Ramp::new(0.0),
            elec_ph: [Phasor::default(); 2],
            elec_amp: Ramp::new(0.0),
            poles: p.cylinders.max(1) as f32,
            bb: Svf::new(3000.0, 0.6, sr),
            bb_amp: Ramp::new(0.0),
            wash_lp: OnePole::new(9.0, sr),
            wash_amp: Ramp::new(0.0),
            wash_bp: Svf::new(700.0, 0.8, sr),
            jit: OnePole::new(3.0, sr),
            rev_hz: 0.0,
        }
    }

    pub fn control(&mut self, c: &EngCtl, p: &Profile, n: u32) {
        let sr = self.sr;
        if c.warp > 0.0 {
            self.rev_hz = c.e.rpm.max(0.0) / 60.0;
        }
        let on = c.e.running && !c.crashed;
        let rev = if on { self.rev_hz } else { 0.0 };
        self.inc.to(rev / sr, n);
        let tip = std::f32::consts::TAU * rev * p.prop_radius / 340.0;
        let load = (c.e.thrust / p.thrust_max.max(1.0)).clamp(0.0, 1.3);
        let spin = smoothstep(5.0, 40.0, rev);
        self.tone_amp.to(spin * (0.05 + 0.5 * load) * (0.3 + tip), n);
        self.mach_mix.to(smoothstep(0.35, 0.65, tip), n);
        self.elec_amp.to(spin * (0.012 + 0.03 * load), n);
        self.bb.set((1800.0 + 3500.0 * tip).min(sr * 0.45), 0.6, sr);
        self.bb_amp.to(spin * load.powf(1.5) * 0.12, n);
        let wash = c.rotor.vrs.clamp(0.0, 1.0);
        self.wash_amp.to(spin * wash * (0.1 + 0.3 * load), n);
    }

    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let w = self.rng.bi();
        // Motor speed wobbles a touch (ESC / gyro noise): the arms beat.
        let j = 1.0 + 0.004 * self.jit.lp(w * 30.0).clamp(-1.0, 1.0);
        let inc = self.inc.tick() * j;
        let p = self.ph.tick(inc);
        let m = self.mach_mix.tick();
        let tone = (self.soft.at(p) * (1.0 - m) + self.hard.at(p) * m) * self.tone_amp.tick();
        // Electrical whine: pole-pair order and its 2nd harmonic.
        let e0 = self.elec_ph[0].tick(inc * self.poles);
        let e1 = self.elec_ph[1].tick(inc * self.poles * 2.0);
        let elec = (sin_turns(e0) + 0.45 * sin_turns(e1)) * self.elec_amp.tick();
        let bb = self.bb.bpn(w) * self.bb_amp.tick();
        // Prop wash: the props chop through their own turbulent wake.
        let gust = self.wash_lp.lp(w * 40.0).clamp(-1.0, 1.0);
        let wa = self.wash_amp.tick();
        let wash = self.wash_bp.bpn(w) * wa * (0.5 + gust.abs() * 2.0);
        let rough = 1.0 + 2.5 * wa * gust;
        EngOut { front: tone * rough + bb + wash, rear: bb * 0.5, case: elec + tone * 0.3, int: 0.0 }
    }
}
