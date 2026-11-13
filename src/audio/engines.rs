//! Physically-informed engine synthesis. Each voice produces three mono
//! components that the mixer weights by directivity:
//! `front` (propeller plane / fan inlet), `rear` (exhaust / jet) and `case`
//! (mechanical whine, heard all round).
//!
//! * Piston: every cylinder firing is an event (even firing order from crank
//!   RPM and cylinder count, per-cylinder character, cycle-to-cycle jitter
//!   that makes the idle lope) exciting an exhaust-stack resonator bank plus
//!   a noise "crack"; overrun pops after a throttle chop.
//! * Propellers: a band-limited one-revolution wavetable with one pressure
//!   pulse per blade (blade-pass harmonics), sharpening with tip Mach, plus
//!   blade-modulated broadband "whup".
//! * Turbofan: one-revolution fan tables — regular blade-pass tones and, once
//!   the fan tips go supersonic, the irregular shock pattern that gives the
//!   buzz-saw multiple pure tones at shaft orders; N2 whine; broadband jet
//!   mixing noise rising steeply with jet velocity; reverser roar;
//!   afterburner rumble + crackle.

use super::dsp::*;
use super::params::{EngineFrame, Family, Profile, RotorFrame};
use super::rotorcraft::{Drone, Heli};
use super::sampled::{Drivers, SampleEngine as SampledEngine, n2_from_n1};
use super::soundsets::{self, Set};

#[derive(Clone, Copy, Debug, Default)]
pub struct EngOut {
    pub front: f32,
    pub rear: f32,
    pub case: f32,
    /// Recorded cockpit sound (already "inside": not filtered again).
    pub int: f32,
}

/// Per-block control inputs for one engine.
#[derive(Clone, Copy, Debug, Default)]
pub struct EngCtl {
    pub e: EngineFrame,
    pub tas: f32,
    pub reverser: f32,
    /// World sounds frozen (pause): hold.
    pub warp: f32,
    /// Listener distance (m) — picks close / distant recordings.
    pub dist: f32,
    /// The aircraft was destroyed: engines stop dead (no shutdown sequence).
    pub crashed: bool,
    /// Rotorcraft state (zero for fixed wing).
    pub rotor: RotorFrame,
}

fn prop_tables(blades: u32, max_rev_hz: f32, seed: u32) -> (Table, Table) {
    let mut rng = Rng::new(seed);
    let n = TABLE_N;
    let b = blades.max(1) as usize;
    let mut soft = vec![0.0f32; n];
    let mut hard = vec![0.0f32; n];
    let var: Vec<f32> = (0..b).map(|_| 1.0 + 0.06 * rng.bi()).collect();
    let off: Vec<f32> = (0..b).map(|_| 0.004 * rng.bi()).collect();
    for i in 0..n {
        let t = i as f32 / n as f32;
        for k in 0..b {
            let c = (k as f32 + 0.5) / b as f32 + off[k];
            let mut d = t - c;
            d -= d.round();
            // Loading pulse: negative pressure dip as each blade passes,
            // wider (gentler) at low tip Mach, sharp at high Mach.
            let ws = 0.09 / b as f32;
            let wh = 0.028 / b as f32;
            soft[i] += -var[k] * (-(d * d) / (2.0 * ws * ws)).exp();
            hard[i] += -var[k] * (-(d * d) / (2.0 * wh * wh)).exp() + 0.35 * var[k] * (-(d - 1.6 * wh).powi(2) / (2.0 * wh * wh)).exp();
        }
    }
    let h = ((16_000.0 / max_rev_hz.max(5.0)) as usize).clamp(8, 220);
    (Table::from_wave(&soft, h), Table::from_wave(&hard, h))
}

fn fan_tables(blades: u32, max_rev_hz: f32, seed: u32) -> (Table, Table) {
    let mut rng = Rng::new(seed);
    let n = TABLE_N;
    let b = blades.max(1) as usize;
    let mut reg = vec![0.0f32; n];
    let mut irr = vec![0.0f32; n];
    // Regular blade wakes: narrow pulses (each only touches nearby samples).
    let w = 0.18 / b as f32;
    let span = (4.0 * w * n as f32) as i64 + 1;
    for k in 0..b {
        let c = (k as f32 / b as f32 * n as f32) as i64;
        for j in -span..=span {
            let i = (c + j).rem_euclid(n as i64) as usize;
            let mut d = i as f32 / n as f32 - k as f32 / b as f32;
            d -= d.round();
            reg[i] += (-(d * d) / (2.0 * w * w)).exp();
        }
    }
    // Rotor-locked shocks: one sawtooth tooth per blade, heights and spacing
    // differ blade to blade (manufacturing scatter) — energy at every shaft
    // order, the "buzz-saw".
    let h: Vec<f32> = (0..b).map(|_| 0.45 + 0.9 * rng.uni()).collect();
    let pos: Vec<f32> = {
        let mut p: Vec<f32> = (0..b).map(|k| (k as f32 + 0.25 * rng.bi()) / b as f32).collect();
        p.sort_by(|a, b| a.total_cmp(b));
        p
    };
    for i in 0..n {
        let t = i as f32 / n as f32;
        // Find the tooth we're in.
        let j = pos.partition_point(|&p| p <= t);
        let k = if j == 0 { b - 1 } else { j - 1 };
        let start = pos[k];
        let end = if k + 1 < b { pos[k + 1] } else { pos[0] + 1.0 };
        let mut u = t - start;
        if u < 0.0 {
            u += 1.0;
        }
        let frac = u / (end - start).max(1e-4);
        irr[i] = h[k] * (1.0 - 2.0 * frac);
    }
    let hh = ((17_000.0 / max_rev_hz.max(5.0)) as usize).clamp(8, 240);
    (Table::from_wave(&reg, hh), Table::from_wave(&irr, hh))
}

// ------------------------------------------------------------------ piston

pub struct Piston {
    sr: f32,
    rng: Rng,
    cyl: u32,
    warbird: bool,
    // crank
    ev_phase: f32,
    inc: Ramp,
    jitter: f32,
    cur_jit: f32,
    count: u32,
    // blow-down pressure pulse (difference of two decays)
    p1: f32,
    p2: f32,
    d1: f32,
    d2: f32,
    cyl_gain: [f32; 12],
    // excitation
    res: [Svf; 4],
    res_gain: [f32; 4],
    crack_env: f32,
    crack_decay: f32,
    crack_amp: f32,
    crack_bp: Svf,
    body: OnePole,
    load: Ramp,
    misfire: f32,
    // overrun pops
    pop_rate: Ramp,
    pop_env: f32,
    pop_amp: f32,
    pop_lp: Svf,
    prev_throttle: f32,
    overrun: f32,
    // mechanical / supercharger
    whine_ph: [Phasor; 2],
    whine_inc: Ramp,
    whine_amp: Ramp,
    tick_hp: OnePole,
    // prop
    prop: Prop,
    pub rpm: f32,
    // restart after a failure: starter motor + catching cylinders
    was_running: bool,
    start_t: f32,
    die_t: f32,
    starter_ph: Phasor,
    starter_amp: Ramp,
    starter_lp: OnePole,
}

pub struct Prop {
    soft: Table,
    hard: Table,
    ph: Phasor,
    inc: Ramp,
    amp: Ramp,
    mach_mix: Ramp,
    whup_amp: Ramp,
    bp: Svf,
    blades: f32,
    radius: f32,
    sr: f32,
    rng: Rng,
    flutter: OnePole,
}

impl Prop {
    pub fn new(blades: u32, radius: f32, max_rev_hz: f32, seed: u32, sr: f32) -> Prop {
        let (soft, hard) = prop_tables(blades, max_rev_hz, seed);
        Prop {
            soft,
            hard,
            ph: Phasor { p: (seed % 97) as f32 / 97.0 },
            inc: Ramp::new(0.0),
            amp: Ramp::new(0.0),
            mach_mix: Ramp::new(0.0),
            whup_amp: Ramp::new(0.0),
            bp: Svf::new(900.0, 0.9, sr),
            blades: blades as f32,
            radius,
            sr,
            rng: Rng::new(seed ^ 0xabcd),
            flutter: OnePole::new(4.0, sr),
        }
    }
    /// `rev_hz`: propeller revolutions per second; `load` 0..1 blade loading.
    pub fn control(&mut self, rev_hz: f32, load: f32, tas: f32, n: u32, gain: f32) {
        let tip = (std::f32::consts::TAU * rev_hz * self.radius).hypot(tas);
        let mtip = tip / 340.0;
        self.inc.to(rev_hz / self.sr, n);
        let spin = smoothstep(1.0, 8.0, rev_hz);
        let a = spin * (0.12 * smoothstep(0.15, 0.85, mtip) + 0.55 * load.clamp(0.0, 1.2) * smoothstep(0.1, 0.6, mtip)) * gain;
        self.amp.to(a, n);
        self.mach_mix.to(smoothstep(0.55, 0.9, mtip), n);
        self.whup_amp.to(spin * (0.05 + 0.25 * load.clamp(0.0, 1.0)) * smoothstep(0.1, 0.7, mtip) * gain, n);
        let bpf = rev_hz * self.blades;
        self.bp.set((bpf * 6.0).clamp(300.0, 3000.0), 0.8, self.sr);
    }
    #[inline]
    pub fn tick(&mut self) -> f32 {
        let p = self.ph.tick(self.inc.tick());
        let m = self.mach_mix.tick();
        let tone = self.soft.at(p) * (1.0 - m) + self.hard.at(p) * m;
        // Blade-synchronous broadband "whup".
        let bl = sin_turns(p * self.blades);
        let w = self.bp.bpn(self.rng.bi()) * (0.55 + 0.45 * bl);
        // Blade loading breathes with the inflow turbulence.
        let fl = 1.0 + self.flutter.lp(self.rng.bi() * 9.0).clamp(-0.4, 0.4);
        tone * self.amp.tick() * fl + w * self.whup_amp.tick()
    }
}

impl Piston {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Piston {
        let seed = p.type_id ^ (idx as u32 * 7919);
        let mut rng = Rng::new(seed);
        let warbird = p.family == Family::Warbird;
        let mut cyl_gain = [1.0f32; 12];
        for (i, g) in cyl_gain.iter_mut().enumerate() {
            // A V12's two banks breathe through different stacks: alternate
            // firings differ, which puts energy at half orders (the growl).
            let bank = if warbird && i % 2 == 1 { 0.62 } else { 1.0 };
            *g = bank * (1.0 + 0.2 * rng.bi());
        }
        let (freqs, qs, gains): ([f32; 4], [f32; 4], [f32; 4]) = if warbird {
            ([72.0, 235.0, 690.0, 1850.0], [2.2, 2.6, 2.2, 1.6], [1.0, 0.75, 0.45, 0.3])
        } else {
            ([98.0, 360.0, 1050.0, 2500.0], [2.8, 2.4, 2.0, 1.5], [1.0, 0.6, 0.3, 0.14])
        };
        let res = [0, 1, 2, 3].map(|i| Svf::new(freqs[i], qs[i], sr));
        let max_rev = 3000.0 * p.prop_gear / 60.0;
        Piston {
            sr,
            rng,
            cyl: p.cylinders.clamp(1, 12),
            warbird,
            ev_phase: 0.0,
            inc: Ramp::new(0.0),
            jitter: 0.0,
            cur_jit: 1.0,
            count: 0,
            p1: 0.0,
            p2: 0.0,
            d1: (-1.0 / (0.0016 * sr)).exp(),
            d2: (-1.0 / (0.0004 * sr)).exp(),
            cyl_gain,
            res,
            res_gain: gains,
            crack_env: 0.0,
            crack_decay: 0.99,
            crack_amp: 0.0,
            crack_bp: Svf::new(if warbird { 1600.0 } else { 2200.0 }, 0.9, sr),
            body: OnePole::new(30.0, sr),
            load: Ramp::new(0.0),
            misfire: 0.0,
            pop_rate: Ramp::new(0.0),
            pop_env: 0.0,
            pop_amp: 0.0,
            pop_lp: Svf::new(900.0, 0.7, sr),
            prev_throttle: 0.0,
            overrun: 0.0,
            whine_ph: [Phasor::default(); 2],
            whine_inc: Ramp::new(0.0),
            whine_amp: Ramp::new(0.0),
            tick_hp: OnePole::new(3000.0, sr),
            prop: Prop::new(p.blades, p.prop_radius, max_rev, seed ^ 0x55, sr),
            rpm: 0.0,
            was_running: true,
            start_t: 0.0,
            die_t: 0.0,
            starter_ph: Phasor::default(),
            starter_amp: Ramp::new(0.0),
            starter_lp: OnePole::new(900.0, sr),
        }
    }

    pub fn control(&mut self, c: &EngCtl, p: &Profile, n: u32) {
        let dt = n as f32 / self.sr;
        let e = &c.e;
        let rpm = if c.warp > 0.0 { e.rpm.max(0.0) } else { self.rpm };
        self.rpm = rpm;
        // Fuel cut: the engine stumbles for a moment rather than stopping dead.
        if !e.running && self.was_running {
            self.die_t = 1.0;
        }
        self.die_t = (self.die_t - dt).max(0.0);
        let running = e.running || self.die_t > 0.0;
        // 4-stroke: every cylinder fires once per two revolutions.
        let ev_hz = rpm / 60.0 * 0.5 * self.cyl as f32;
        self.inc.to(ev_hz / self.sr, n);
        let pw = if e.running { e.spool.clamp(0.0, 1.1) } else if running { 0.5 * self.die_t } else { 0.0 };
        self.load.to(if running { 0.42 + 0.58 * pw.powf(0.7) } else { 0.04 }, n);
        let idle = 1.0 - smoothstep(0.1, 0.4, pw);
        self.jitter = if running { 0.012 + 0.06 * idle } else { 0.08 };
        // Restart: the starter grinds and the cylinders catch one by one.
        if e.running && !self.was_running {
            self.start_t = 2.2;
        }
        self.was_running = e.running;
        self.start_t = (self.start_t - dt).max(0.0);
        let starting = smoothstep(0.0, 1.2, self.start_t);
        let dying = if e.running { 0.0 } else { 1.0 - self.die_t };
        self.misfire = if running { (0.004 * idle).max(0.6 * starting).max(dying) } else { 1.0 };
        self.starter_amp.to(0.06 * smoothstep(0.6, 1.4, self.start_t), n);
        // Crack decay: shorter at high rpm.
        let tau = (0.0045 - 0.0025 * smoothstep(800.0, 2800.0, rpm)).max(0.0015);
        self.crack_decay = (-1.0 / (tau * self.sr)).exp();
        // Overrun pops after a throttle chop at high rpm.
        let chop = (self.prev_throttle - e.throttle).max(0.0);
        self.prev_throttle = e.throttle;
        if chop > 0.02 && rpm > 1400.0 {
            self.overrun = (self.overrun + chop * 4.0).min(1.0);
        }
        self.overrun *= (-dt / if self.warbird { 1.6 } else { 0.9 }).exp();
        let windmill = smoothstep(0.35, 0.05, pw) * smoothstep(900.0, 2200.0, rpm);
        let pr = if running { (self.overrun * 22.0 + windmill * 2.5) * if self.warbird { 1.0 } else { 0.35 } } else { 0.0 };
        self.pop_rate.to(pr, n);
        // Supercharger / gear whine (warbird) or valve-train (light).
        let wf = if self.warbird { rpm / 60.0 * 48.0 } else { rpm / 60.0 * 30.0 };
        self.whine_inc.to(wf / self.sr, n);
        self.whine_amp.to(
            if self.warbird { 0.035 * pw.powf(1.5) * smoothstep(600.0, 1800.0, rpm) } else { 0.006 * smoothstep(500.0, 2000.0, rpm) },
            n,
        );
        // Propeller.
        let rev = rpm * p.prop_gear / 60.0;
        self.prop.control(rev, if running { pw } else { 0.0 }, c.tas, n, if self.warbird { 1.2 } else { 1.3 });
    }

    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let inc = self.inc.tick() * self.cur_jit;
        let load = self.load.tick();
        self.ev_phase += inc;
        if self.ev_phase >= 1.0 {
            self.ev_phase -= 1.0;
            let c = (self.count % self.cyl) as usize;
            self.count = self.count.wrapping_add(1);
            let scatter = if self.warbird { 0.22 } else { 0.14 };
            let mut a = load * self.cyl_gain[c] * (1.0 + scatter * self.rng.bi());
            if self.rng.uni() < self.misfire {
                a *= 0.08;
            }
            // Cycle-to-cycle scatter of the next interval (the idle lope).
            self.cur_jit = 1.0 + self.jitter * self.rng.gauss() * 0.6;
            self.p1 += a;
            self.p2 += a;
            self.crack_amp = a * a;
            self.crack_env = 1.0;
        }
        self.p1 *= self.d1;
        self.p2 *= self.d2;
        let pulse = self.p1 - self.p2;
        let mut ex = 0.0;
        for i in 0..4 {
            ex += self.res[i].bpn(pulse) * self.res_gain[i];
        }
        // Remove the DC of the unipolar pulse train.
        let body = pulse - self.body.lp(pulse);
        self.crack_env *= self.crack_decay;
        let crack = self.crack_bp.bpn(self.rng.bi()) * self.crack_env * self.crack_amp;
        // Overrun pops: random bangs of filtered noise.
        let rate = self.pop_rate.tick();
        if rate > 0.0 && self.rng.uni() < rate / self.sr {
            self.pop_amp = 0.4 + 0.9 * self.rng.uni().powi(2);
            self.pop_env = 1.0;
        }
        self.pop_env *= 0.9965;
        let pop = self.pop_lp.lp(self.rng.bi() * self.pop_env * self.pop_amp) * if self.warbird { 7.0 } else { 3.5 };
        let (g, cg) = if self.warbird { (4.8, 1.4) } else { (3.6, 0.75) };
        let rear = (ex * 2.2 + body * 0.5) * g / 2.2 + crack * cg + pop;
        // Case: supercharger whine + valve tick.
        let wi = self.whine_inc.tick();
        let p0 = self.whine_ph[0].tick(wi);
        let p1 = self.whine_ph[1].tick(wi * 1.5);
        let whine = (sin_turns(p0) + 0.4 * sin_turns(p1)) * self.whine_amp.tick();
        let sa = self.starter_amp.tick();
        let starter = if sa > 1e-5 {
            let p = self.starter_ph.tick(95.0 / self.sr);
            self.starter_lp.lp((2.0 * p - 1.0) + 0.3 * self.rng.bi()) * sa
        } else {
            0.0
        };
        let tick = self.tick_hp.hp(self.rng.bi() * self.crack_env) * 0.05;
        let front = self.prop.tick();
        EngOut { front, rear, case: whine + tick + starter, int: 0.0 }
    }
}

// ------------------------------------------------------------------ turboprop

pub struct Turboprop {
    sr: f32,
    rng: Rng,
    prop: Prop,
    detune: f32,
    drift: OnePole,
    gg_ph: [Phasor; 3],
    gg_inc: Ramp,
    gg_amp: Ramp,
    wobble: f32,
    hiss: Svf,
    hiss_amp: Ramp,
    roar: Svf,
    roar_amp: Ramp,
    pink: Pink,
    beta: Svf,
    beta_amp: Ramp,
    rumble: OnePole,
    rpm: f32,
}

impl Turboprop {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Turboprop {
        let seed = p.type_id ^ (idx as u32 * 104_729);
        Turboprop {
            sr,
            rng: Rng::new(seed),
            prop: Prop::new(p.blades, p.prop_radius, 1300.0 * p.prop_gear / 60.0, seed, sr),
            detune: 1.0 + 0.0011 * idx as f32,
            drift: OnePole::new(0.05, sr / 64.0),
            gg_ph: [Phasor::default(); 3],
            gg_inc: Ramp::new(0.0),
            gg_amp: Ramp::new(0.0),
            wobble: 0.0,
            hiss: Svf::new(5000.0, 0.8, sr),
            hiss_amp: Ramp::new(0.0),
            roar: Svf::new(350.0, 0.6, sr),
            roar_amp: Ramp::new(0.0),
            pink: Pink::default(),
            beta: Svf::new(900.0, 0.5, sr),
            beta_amp: Ramp::new(0.0),
            rumble: OnePole::new(90.0, sr),
            rpm: 0.0,
        }
    }
    pub fn control(&mut self, c: &EngCtl, p: &Profile, n: u32) {
        let e = &c.e;
        if c.warp > 0.0 {
            self.rpm = e.rpm.max(0.0);
        }
        let running = e.running;
        let pw = e.spool.clamp(0.0, 1.1);
        // Gas generator speed: ~60 % at ground idle, follows the power lever.
        let ng = if running { 0.62 + 0.38 * pw } else { 0.62 * pw };
        self.wobble = 0.002;
        self.gg_inc.to(ng * 6400.0 / self.sr, n);
        self.gg_amp.to(0.05 * smoothstep(0.1, 0.6, ng) * (0.6 + 0.4 * ng), n);
        self.hiss_amp.to(0.05 * ng * ng, n);
        self.hiss.set(3500.0 + 3000.0 * ng, 0.7, self.sr);
        self.roar_amp.to(0.12 * pw.powf(1.3) + 0.01 * ng, n);
        // Synchrophaser keeps the props within a whisker: a slow, wandering beat.
        let d = self.drift.lp(self.rng.bi() * 3.0) * 0.02;
        let rev = self.rpm * p.prop_gear / 60.0 * (self.detune + d);
        let load = if running { pw } else { 0.0 };
        self.beta_amp.to(c.reverser * (0.15 + 0.6 * pw) * smoothstep(5.0, 15.0, rev), n);
        self.prop.control(rev, load * (1.0 + 0.6 * c.reverser), c.tas, n, 1.0);
    }
    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let w = self.rng.bi();
        let inc = self.gg_inc.tick() * (1.0 + self.wobble * self.rng.bi());
        let a = self.gg_ph[0].tick(inc);
        let b = self.gg_ph[1].tick(inc * 0.71);
        let d = self.gg_ph[2].tick(inc * 1.93);
        let gg = (sin_turns(a) + 0.6 * sin_turns(b) + 0.25 * sin_turns(d)) * self.gg_amp.tick();
        let hiss = self.hiss.bpn(w) * self.hiss_amp.tick();
        let pk = self.pink.tick(w);
        let roar = (self.roar.lp(pk) * 1.3 + self.rumble.lp(pk) * 1.4) * self.roar_amp.tick();
        let beta = self.beta.lp(pk) * self.beta_amp.tick() * 1.5;
        let prop = self.prop.tick();
        EngOut { front: prop + beta + hiss * 0.5, rear: roar, case: gg + hiss * 0.5, int: 0.0 }
    }
}

// ------------------------------------------------------------------ turbofan

pub struct Jet {
    sr: f32,
    rng: Rng,
    fighter: bool,
    reg: Table,
    irr: Table,
    ph: Phasor,
    inc: Ramp,
    tone_amp: Ramp,
    buzz_amp: Ramp,
    fan_bb: Svf,
    fan_bb_amp: Ramp,
    n2_ph: [Phasor; 2],
    n2_inc: Ramp,
    n2_amp: Ramp,
    fm: f32,
    fm_lp: OnePole,
    pink: Pink,
    roar_lp: Svf,
    roar_bp: Svf,
    roar_amp: Ramp,
    roar_lfo: OnePole,
    rumble: Svf,
    rev_amp: Ramp,
    rev_f: Svf,
    // afterburner
    ab_amp: Ramp,
    ab_lp: Svf,
    crackle_rate: Ramp,
    crackle_hp: OnePole,
    crackle_env: f32,
    crackle_amp: f32,
    blades: f32,
    rpm: f32,
    wob: OnePole,
}

impl Jet {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Jet {
        let seed = p.type_id ^ (idx as u32 * 15_485_863);
        let fighter = p.family == Family::Fighter;
        let max_rev = p.n1_rpm * 1.06 / 60.0;
        let (reg, irr) = fan_tables(p.fan_blades, max_rev, seed);
        Jet {
            sr,
            rng: Rng::new(seed),
            fighter,
            reg,
            irr,
            ph: Phasor { p: 0.37 * idx as f32 },
            inc: Ramp::new(0.0),
            tone_amp: Ramp::new(0.0),
            buzz_amp: Ramp::new(0.0),
            fan_bb: Svf::new(2000.0, 1.2, sr),
            fan_bb_amp: Ramp::new(0.0),
            n2_ph: [Phasor::default(); 2],
            n2_inc: Ramp::new(0.0),
            n2_amp: Ramp::new(0.0),
            fm: 0.0,
            fm_lp: OnePole::new(3.0, sr),
            pink: Pink::default(),
            roar_lp: Svf::new(600.0, 0.6, sr),
            roar_bp: Svf::new(200.0, 0.8, sr),
            roar_amp: Ramp::new(0.0),
            roar_lfo: OnePole::new(6.0, sr),
            rumble: Svf::new(70.0, 0.7, sr),
            rev_amp: Ramp::new(0.0),
            rev_f: Svf::new(1400.0, 0.5, sr),
            ab_amp: Ramp::new(0.0),
            ab_lp: Svf::new(160.0, 0.7, sr),
            crackle_rate: Ramp::new(0.0),
            crackle_hp: OnePole::new(700.0, sr),
            crackle_env: 0.0,
            crackle_amp: 0.0,
            blades: p.fan_blades as f32,
            rpm: 0.0,
            wob: OnePole::new(3.0, sr),
        }
    }
    pub fn control(&mut self, c: &EngCtl, p: &Profile, n: u32) {
        let e = &c.e;
        if c.warp > 0.0 {
            self.rpm = e.rpm.max(0.0);
        }
        let n1 = (self.rpm / p.n1_rpm).clamp(0.0, 1.1);
        let rev = self.rpm / 60.0;
        self.inc.to(rev / self.sr, n);
        // Relative tip Mach rises with N1 (and inflow): supersonic above ~85 %.
        let mrel = n1 * 1.18 + c.tas / 340.0 * 0.25;
        let buzz = smoothstep(0.98, 1.12, mrel);
        let (tone, bz) = if self.fighter { (0.05 + 0.10 * n1 * n1, 0.10) } else { (0.04 + 0.14 * n1 * n1, 0.30) };
        self.tone_amp.to(tone * smoothstep(0.03, 0.2, n1), n);
        self.buzz_amp.to(bz * buzz, n);
        let bpf = rev * self.blades;
        self.fan_bb.set(bpf.clamp(400.0, 9000.0), 1.0, self.sr);
        self.fan_bb_amp.to(0.06 * n1 * n1, n);
        // N2 (core) whine — dominant at idle on the ground.
        let n2 = if e.running { 0.58 + 0.42 * n1 } else { n1 * 0.9 };
        let k = if self.fighter { 7600.0 } else { 6100.0 };
        self.n2_inc.to(n2 * k / self.sr, n);
        self.n2_amp.to(0.05 * smoothstep(0.15, 0.55, n2) * (1.2 - 0.5 * n1), n);
        // Jet mixing noise: grows very steeply with jet velocity.
        let idle = p.n1_rpm.max(1.0);
        let _ = idle;
        let jet = smoothstep(0.15, 1.0, n1);
        let ab = e.ab.clamp(0.0, 1.0);
        self.roar_amp.to((0.04 + 0.32 * jet.powf(2.2)) * if self.fighter { 1.05 } else { 1.0 }, n);
        self.roar_lp.set(500.0 + 2600.0 * jet, 0.6, self.sr);
        self.roar_bp.set(120.0 + 230.0 * jet, 0.9, self.sr);
        // Reverser: the fan stream is turned forward and out.
        self.rev_amp.to(c.reverser * (0.05 + 0.6 * jet), n);
        // Afterburner.
        self.ab_amp.to(ab * 0.32, n);
        self.crackle_rate.to(ab * 380.0 + if self.fighter { 60.0 * smoothstep(0.6, 1.0, jet) } else { 0.0 }, n);
    }
    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let w = self.rng.bi();
        let p = self.ph.tick(self.inc.tick());
        // Fan tones warble with the turbulence the inlet swallows.
        let wob = 1.0 + self.wob.lp(self.rng.bi() * 10.0).clamp(-0.5, 0.5);
        let tones = (self.reg.at(p) * self.tone_amp.tick() + self.irr.at(p) * self.buzz_amp.tick()) * wob;
        let fan_bb = self.fan_bb.bpn(w) * self.fan_bb_amp.tick();
        self.fm = self.fm_lp.lp(self.rng.bi()) * 0.004;
        let ni = self.n2_inc.tick() * (1.0 + self.fm * 8.0);
        let a = self.n2_ph[0].tick(ni);
        let b = self.n2_ph[1].tick(ni * 1.47);
        let n2 = (sin_turns(a) + 0.35 * sin_turns(b)) * self.n2_amp.tick();
        let pk = self.pink.tick(w);
        let lfo = 0.75 + 0.5 * self.roar_lfo.lp(self.rng.bi() * 6.0).clamp(-0.5, 0.5);
        let ra = self.roar_amp.tick();
        let roar = (self.roar_lp.lp(pk) + self.roar_bp.bpn(pk) * 0.9 + self.rumble.lp(pk) * 1.2) * ra * lfo;
        let rev = self.rev_f.lp(pk) * self.rev_amp.tick() * 1.4;
        // Afterburner: deep rumble + impulsive crackle (skewed spikes).
        let aba = self.ab_amp.tick();
        let ab = self.ab_lp.lp(pk) * aba * 2.2;
        let cr = self.crackle_rate.tick();
        if self.rng.uni() < cr / self.sr {
            self.crackle_amp = self.rng.uni().powi(3) * (0.5 + aba);
            self.crackle_env = 1.0;
        }
        self.crackle_env *= 0.93;
        let crackle = self.crackle_hp.hp(self.crackle_env * self.crackle_amp * (1.0 + 0.5 * w)) * 1.4;
        EngOut { front: tones + fan_bb + rev, rear: roar + ab + crackle, case: n2, int: 0.0 }
    }
}

enum Proc {
    Piston(Box<Piston>),
    Turboprop(Box<Turboprop>),
    Jet(Box<Jet>),
    Heli(Box<Heli>),
    Drone(Box<Drone>),
}

/// One engine's sound: the recorded set (when the aircraft has one and its
/// samples are decoded) with the procedural voice underneath as detail
/// (blade-pass tones, crackle, buzz-saw...), or the procedural voice alone.
pub struct EngineVoice {
    proc_: Proc,
    samp: Option<Box<SampledEngine>>,
    set: Option<&'static Set>,
    seed: u32,
    sr: f32,
    /// Procedural component gains (ramped when the samples arrive).
    pg: [Ramp; 3],
    /// Sample-bank fade-in.
    sg: Ramp,
    n1_idle: f32,
}

impl EngineVoice {
    pub fn new(p: &Profile, idx: usize, sr: f32) -> Option<EngineVoice> {
        let proc_ = match p.family {
            Family::Glider => return None,
            Family::LightPiston | Family::Warbird => Proc::Piston(Box::new(Piston::new(p, idx, sr))),
            Family::Turboprop => Proc::Turboprop(Box::new(Turboprop::new(p, idx, sr))),
            Family::Fighter | Family::Airliner => Proc::Jet(Box::new(Jet::new(p, idx, sr))),
            Family::Helicopter => Proc::Heli(Box::new(Heli::new(p, idx, sr))),
            Family::Drone => Proc::Drone(Box::new(Drone::new(p, idx, sr))),
        };
        let seed = p.type_id ^ (idx as u32).wrapping_mul(0x9e37_79b9);
        let mut v = EngineVoice { proc_, samp: None, set: soundsets::set(p.set), seed, sr, pg: [Ramp::new(1.0); 3], sg: Ramp::new(0.0), n1_idle: p.n1_idle };
        v.try_samples(0);
        Some(v)
    }

    fn try_samples(&mut self, n: u32) {
        if self.samp.is_some() {
            return;
        }
        if let Some(set) = self.set
            && let Some(s) = SampledEngine::new(set, self.seed, self.sr)
        {
            self.samp = Some(Box::new(s));
            let d = set.detail;
            // At aircraft load (n = 0) switch at once; later, cross-fade.
            let m = if n == 0 { 0 } else { (0.4 * self.sr) as u32 };
            self.pg[0].to(d.front, m);
            self.pg[1].to(d.rear, m);
            self.pg[2].to(d.case, m);
            self.sg.to(1.0, m);
        }
    }

    /// Share of the exterior mix that still feeds the filtered interior path.
    pub fn int_from_ext(&self) -> f32 {
        match (&self.samp, self.set) {
            (Some(_), Some(s)) => s.int_from_ext,
            _ => 1.0,
        }
    }

    pub fn control(&mut self, c: &EngCtl, p: &Profile, n: u32) {
        self.try_samples(n);
        match &mut self.proc_ {
            Proc::Piston(v) => v.control(c, p, n),
            Proc::Turboprop(v) => v.control(c, p, n),
            Proc::Jet(v) => v.control(c, p, n),
            Proc::Heli(v) => v.control(c, p, n),
            Proc::Drone(v) => v.control(c, p, n),
        }
        if let Some(s) = &mut self.samp {
            let e = &c.e;
            let n1 = if p.n1_rpm > 0.0 { e.rpm / p.n1_rpm } else { e.spool };
            let d = match p.family {
                Family::Turboprop => {
                    // Gas generator: ~62 % at ground idle, follows the power lever.
                    let pw = e.spool.clamp(0.0, 1.1);
                    let ng = if e.running { 0.62 + 0.38 * pw } else { 0.62 * pw.min(1.0) * 1.6 }.min(1.05);
                    Drivers { n1: ng * 100.0, n2: ng * 100.0, rpm: e.rpm, pow: pw, ab: 0.0, rev: c.reverser }
                }
                Family::Fighter | Family::Airliner => Drivers { n1: n1 * 100.0, n2: n2_from_n1(n1, self.n1_idle), rpm: e.rpm, pow: e.spool, ab: e.ab, rev: c.reverser },
                _ => Drivers { n1: e.spool * 100.0, n2: e.spool * 100.0, rpm: e.rpm, pow: e.spool.clamp(0.0, 1.1), ab: 0.0, rev: 0.0 },
            };
            s.control(e, &d, c.dist, c.crashed, n);
        }
    }

    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let mut o = match &mut self.proc_ {
            Proc::Piston(v) => v.tick(),
            Proc::Turboprop(v) => v.tick(),
            Proc::Jet(v) => v.tick(),
            Proc::Heli(v) => v.tick(),
            Proc::Drone(v) => v.tick(),
        };
        let (a, b, c) = (self.pg[0].tick(), self.pg[1].tick(), self.pg[2].tick());
        o.front *= a;
        o.rear *= b;
        o.case *= c;
        if let Some(s) = &mut self.samp {
            let g = self.sg.tick();
            let so = s.tick();
            o.front += so.front * g;
            o.rear += so.rear * g;
            o.case += so.case * g;
            o.int += so.int * g;
        }
        o
    }
}
