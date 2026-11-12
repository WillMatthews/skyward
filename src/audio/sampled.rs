//! Sample-based engine voice: plays a [`soundsets::Set`] for one engine —
//! every layer a looping [`samples::Voice`] whose volume and pitch follow the
//! engine through the set's tables and equal-power cross-fade groups, faded
//! between close / normal / distant recordings by listener distance, with
//! slow random gain / pitch drift and random loop start points so the loops
//! never line up the same way twice; start-up and shutdown one-shots on the
//! running-state transitions.

use super::dsp::*;
use super::engines::EngOut;
use super::params::EngineFrame;
use super::samples::{self, Voice};
use super::soundsets::{Band, Bus, Drv, Pitch, Set, table, xfade_weight};

/// Engine state in the units the sets use.
#[derive(Clone, Copy, Debug, Default)]
pub struct Drivers {
    pub n1: f32,
    pub n2: f32,
    pub rpm: f32,
    pub pow: f32,
    pub ab: f32,
    pub rev: f32,
}

impl Drivers {
    #[inline]
    fn get(&self, d: Drv) -> f32 {
        match d {
            Drv::N1 => self.n1,
            Drv::N2 => self.n2,
            Drv::Rpm => self.rpm,
            Drv::Pow => self.pow,
            Drv::Ab => self.ab,
            Drv::Rev => self.rev,
        }
    }
}

/// Core speed (%) from fan speed: the sim models one spool; the sound sets
/// need both. Ground idle ≈ 58 % N2, take-off ≈ 99 %; below idle (start /
/// spool-down) N2 falls in proportion.
pub fn n2_from_n1(n1: f32, idle: f32) -> f32 {
    let idle = idle.clamp(0.05, 0.8);
    if n1 >= idle {
        58.0 + 41.0 * ((n1 - idle) / (1.0 - idle)).clamp(0.0, 1.1).powf(0.75)
    } else {
        58.0 * (n1 / idle).max(0.0)
    }
}

struct LayerVoice {
    v: Voice,
    pcm: &'static samples::Pcm,
    bus: Bus,
    rate: f32,
    drift_g: OnePole,
    drift_p: OnePole,
}

struct ShotVoice {
    v: Voice,
    pcm: &'static samples::Pcm,
    rate: f32,
    bus: Bus,
    gain: f32,
    wait: u32,
    hold: i64,
}

pub struct SampleEngine {
    set: &'static Set,
    sr: f32,
    rng: Rng,
    layers: Vec<LayerVoice>,
    shots: Vec<ShotVoice>,
    fire: Follow,
    was_running: Option<bool>,
    kern: &'static [f32],
    /// Sum of the start one-shots' remaining level (ducks nothing; for tests).
    pub shots_active: usize,
}

impl SampleEngine {
    /// None until every sample of the set is decoded.
    pub fn new(set: &'static Set, seed: u32, sr: f32) -> Option<SampleEngine> {
        if !samples::ready(set.samples) {
            return None;
        }
        let mut rng = Rng::new(seed ^ 0x51a7);
        let ctl_sr = sr / 64.0;
        let layers = set
            .layers
            .iter()
            .map(|l| {
                let id = samples::id(l.s);
                let p = samples::pcm(id).expect("ready");
                // Random start point: two engines (or two sessions) never
                // play their loops in step.
                let start = rng.uni() as f64 * p.len as f64;
                LayerVoice {
                    v: Voice::new(id, start),
                    pcm: p,
                    bus: l.bus,
                    rate: p.rate,
                    drift_g: OnePole::new(0.25, ctl_sr),
                    drift_p: OnePole::new(0.15, ctl_sr),
                }
            })
            .collect();
        Some(SampleEngine {
            set,
            sr,
            rng,
            layers,
            shots: Vec::with_capacity(8),
            fire: Follow { v: 1.0 },
            was_running: None,
            kern: samples::kern(),
            shots_active: 0,
        })
    }

    /// Once per block. `dist` = listener distance (m), `warp` 0 = paused.
    pub fn control(&mut self, e: &EngineFrame, d: &Drivers, dist: f32, crashed: bool, n: u32) {
        let dt = n as f32 / self.sr;
        let set = self.set;
        // Start-up / shutdown one-shots on running-state changes.
        if crashed {
            for lv in &mut self.layers {
                lv.v.gain.to(0.0, n);
            }
            for s in &mut self.shots {
                s.v.gain.to(0.0, n);
                s.gain = 0.0;
            }
            self.shots.retain(|s| s.v.gain.v > 1e-4);
            self.was_running = Some(false);
            self.fire.v = 0.0;
            return;
        }
        match self.was_running {
            Some(w) if w != e.running => {
                let list = if e.running { set.start } else { set.stop };
                // A new transition cuts the previous sequence short.
                for s in &mut self.shots {
                    s.gain = 0.0;
                }
                for o in list {
                    let id = samples::id(o.s);
                    if let Some(p) = samples::pcm(id) {
                        let mut v = Voice::new(id, 0.0);
                        v.gain = Ramp::new(o.gain * set.gain);
                        self.shots.push(ShotVoice {
                            v,
                            pcm: p,
                            rate: p.rate,
                            bus: o.bus,
                            gain: o.gain * set.gain,
                            wait: (o.delay * self.sr) as u32,
                            hold: if o.hold > 0.0 {
                                (o.hold * self.sr) as i64
                            } else {
                                i64::MAX
                            },
                        });
                    }
                }
            }
            _ => {}
        }
        self.was_running = Some(e.running);
        let fire = self
            .fire
            .go(if e.running { 1.0 } else { 0.0 }, dt, 0.05, 0.12);
        // Distance bands: close recordings fade out beyond ~30 m, distant
        // ones take over beyond a few hundred metres.
        let near = 1.0 - smoothstep(25.0, 110.0, dist);
        let far = smoothstep(140.0, 800.0, dist);
        let base = 1.0 - 0.55 * far;
        let inc_k = 1.0 / self.sr;
        let lvl_x = db_to_gain(table(set.level.1, d.get(set.level.0)));
        let lvl_i = db_to_gain(table(set.level_int.1, d.get(set.level_int.0)));
        for (i, l) in set.layers.iter().enumerate() {
            let lv = &mut self.layers[i];
            let mut g = table(l.vol.1, d.get(l.vol.0)) * l.gain * set.gain;
            if let Some((dv, t)) = l.vmul {
                g *= table(t, d.get(dv));
            }
            if let Some((gi, k)) = l.xf {
                let grp = &set.groups[gi];
                g *= xfade_weight(grp.centres, k, d.get(grp.drv));
            }
            if l.fire {
                g *= fire;
            }
            if l.bus == Bus::Int {
                g *= lvl_i * set.gain_int;
            } else {
                g *= lvl_x;
                g *= match l.band {
                    Band::All => base,
                    Band::Near => near,
                    Band::Far => far,
                };
            }
            let pitch = match l.pitch {
                Pitch::Tab(dv, t) => table(t, d.get(dv)),
                Pitch::Ratio(dv, r, ex, lo, hi) => (d.get(dv).max(0.0) / r).powf(ex).clamp(lo, hi),
                Pitch::Lin(dv, a, b, lo, hi) => (a + b * d.get(dv)).clamp(lo, hi),
            };
            // Slow random drift: ±~1 dB, ±~0.3 % — a live engine never
            // holds perfectly still, and it hides loop recognition.
            let dg = 1.0 + 0.11 * lv.drift_g.lp(self.rng.bi() * 3.0).clamp(-1.0, 1.0);
            let dp = 1.0 + 0.003 * lv.drift_p.lp(self.rng.bi() * 3.0).clamp(-1.0, 1.0);
            lv.v.gain.to(g.max(0.0) * dg, n);
            lv.v.inc.to(lv.rate * inc_k * pitch.max(0.05) * dp, n);
        }
        for s in &mut self.shots {
            s.v.inc.to(s.rate * inc_k, n);
            if s.gain == 0.0 {
                s.v.gain.to(0.0, (0.15 * self.sr) as u32);
            }
        }
        self.shots
            .retain(|s| !s.v.done && !(s.gain == 0.0 && s.v.gain.v < 1e-4));
        self.shots_active = self.shots.len();
    }

    #[inline]
    pub fn tick(&mut self) -> EngOut {
        let mut o = EngOut::default();
        let k = self.kern;
        for lv in &mut self.layers {
            let y = lv.v.tick(lv.pcm, k);
            o.add(lv.bus, y);
        }
        for s in &mut self.shots {
            if s.wait > 0 {
                s.wait -= 1;
                continue;
            }
            s.hold -= 1;
            if s.hold == 0 {
                s.gain = 0.0;
                s.v.gain.to(0.0, (0.08 * self.sr) as u32);
            }
            let y = s.v.tick(s.pcm, k);
            o.add(s.bus, y);
        }
        o
    }
}

#[inline]
fn db_to_gain(db: f32) -> f32 {
    10f32.powf(db / 20.0)
}

impl EngOut {
    #[inline]
    pub fn add(&mut self, bus: Bus, y: f32) {
        match bus {
            Bus::Front => self.front += y,
            Bus::Rear => self.rear += y,
            Bus::Case => self.case += y,
            Bus::Int => self.int += y,
        }
    }
}
