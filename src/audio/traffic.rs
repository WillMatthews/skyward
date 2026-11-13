//! Other aircraft (multiplayer traffic): one engine voice per aircraft (the
//! same recorded / procedural voices as the player's, `engines.rs`) placed
//! in the world — propagation delay (so Doppler comes out of the moving
//! source), air absorption, distance law, jet / prop directivity, pan.
//! Heard muffled through the airframe from the cockpit.

use super::dsp::*;
use super::engines::{EngCtl, EngineVoice};
use super::params::{Family, MAX_TRAFFIC, TrafficFrame};
use glam::Vec3;

const SPEED_OF_SOUND: f32 = 343.0;
/// ~2.6 km of propagation delay.
const MAX_DELAY_S: f32 = 7.5;

struct Voice {
    id: u32,
    voice: Option<EngineVoice>,
    line: DelayLine,
    est: Vec3,
    delay: f32,
    d_step: f32,
    dist: f32,
    air: Svf,
    gain: Ramp,
    gl: Ramp,
    gr: Ramp,
    dir: [Ramp; 3],
    fade: Ramp,
    fresh: bool,
}

impl Voice {
    fn new(t: &TrafficFrame, sr: f32) -> Voice {
        Voice {
            id: t.id,
            voice: EngineVoice::new(&t.profile, 0, sr),
            line: DelayLine::new((MAX_DELAY_S * sr) as usize + 256),
            est: t.pos,
            delay: 1.0,
            d_step: 0.0,
            dist: 100.0,
            air: Svf::new(18_000.0, 0.6, sr),
            gain: Ramp::new(0.0),
            gl: Ramp::new(0.7),
            gr: Ramp::new(0.7),
            dir: [Ramp::new(1.0); 3],
            fade: Ramp::new(0.0),
            fresh: true,
        }
    }
}

#[derive(Default)]
pub struct Traffic {
    voices: [Option<Voice>; MAX_TRAFFIC],
}

impl Traffic {
    /// Shift every position by the floating-origin move.
    pub fn shift(&mut self, d: Vec3) {
        for v in self.voices.iter_mut().flatten() {
            v.est += d;
        }
    }

    /// Per block: match voices to the frame's aircraft, kinematics, controls.
    /// `since`: seconds since the frame (dead reckoning); `warp` 0 = frozen.
    pub fn control(&mut self, frames: &[TrafficFrame; MAX_TRAFFIC], ear: Vec3, ear_right: Vec3, since: f32, warp: f32, n: u32, sr: f32) {
        let dt = n as f32 / sr;
        // Voices whose aircraft left the frame fade out, then free the slot.
        for slot in self.voices.iter_mut() {
            if let Some(v) = slot {
                if !frames.iter().any(|t| t.id == v.id) {
                    v.fade.to(0.0, (0.25 * sr) as u32);
                    if v.fade.v <= 1e-4 {
                        *slot = None;
                    }
                }
            }
        }
        for t in frames.iter().filter(|t| t.id != 0) {
            let i = match self.voices.iter().position(|v| v.as_ref().is_some_and(|v| v.id == t.id)) {
                Some(i) => i,
                None => match self.voices.iter().position(|v| v.is_none()) {
                    Some(i) => {
                        self.voices[i] = Some(Voice::new(t, sr));
                        i
                    }
                    None => continue,
                },
            };
            let v = self.voices[i].as_mut().unwrap();
            v.fade.to(1.0, (0.3 * sr) as u32);
            let src = t.pos + t.vel * since;
            let k = 1.0 - (-dt / 0.06).exp();
            if v.fresh {
                v.est = src;
            } else {
                v.est += t.vel * dt;
                v.est += (src - v.est) * k;
            }
            let dv = v.est - ear;
            let d = dv.length().max(0.5);
            v.dist = d;
            let dir = dv / d;
            // Retarded time (subsonic model: speeds capped at 0.8 c).
            let c = SPEED_OF_SOUND;
            let vel = t.vel.clamp_length_max(0.8 * c);
            let av = dv.dot(vel);
            let disc = av * av - (vel.length_squared() - c * c) * d * d;
            let delay_s = d * d / (av + disc.max(0.0).sqrt()).max(1e-3);
            let target = (delay_s * sr).clamp(1.0, v.line.max_delay() - 128.0);
            if v.fresh {
                v.delay = target;
                v.d_step = 0.0;
                v.fresh = false;
            } else {
                v.d_step = ((target - v.delay) / n as f32).clamp(-0.6, 0.6);
            }
            let size = t.profile.size.max(1.0);
            let refd = size * 2.6 + 6.0;
            let g = if d < 400.0 { (refd / d.max(size * 0.5)).powf(0.92) } else { (refd / 400.0).powf(0.92) * (400.0 / d).powf(0.75) };
            // Several engines: louder (incoherent sum).
            let g = g.min(1.4) * (t.n_engines.max(1) as f32).sqrt() * 0.85;
            v.gain.to(g, n);
            let fc = 19_000.0 * (160.0 / (d + 160.0)).powf(1.05);
            v.air.set(fc.max(300.0), 0.6, sr);
            let (pl, pr) = pan(dir.dot(ear_right) * 0.9);
            v.gl.to(pl, n);
            v.gr.to(pr, n);
            let cth = -dir.dot(t.axis.normalize_or_zero());
            let (gf, gb, gc) = match t.profile.family {
                Family::Airliner | Family::Fighter => (0.2 + 0.8 * smoothstep(-0.4, 0.75, cth), 0.15 + 0.85 * smoothstep(0.2, -0.8, cth), 0.6 + 0.4 * smoothstep(-0.5, 0.6, cth)),
                Family::Turboprop => (0.6 + 0.4 * (1.0 - cth * cth), 0.35 + 0.65 * smoothstep(0.1, -0.8, cth), 0.8),
                _ => (0.65 + 0.35 * (1.0 - cth * cth) + 0.1 * cth, 0.85 + 0.15 * (1.0 - cth.abs()), 1.0),
            };
            v.dir[0].to(gf, n);
            v.dir[1].to(gb, n);
            v.dir[2].to(gc, n);
            if let Some(ev) = v.voice.as_mut() {
                let ctl = EngCtl { e: t.eng, tas: t.tas, reverser: t.reverser, warp, dist: d, crashed: !t.eng.running && t.eng.rpm < 1.0, rotor: t.rotor };
                ev.control(&ctl, &t.profile, n);
            }
        }
    }

    /// One sample: (left, right) outside, and a mono feed for the cockpit path.
    #[inline]
    pub fn tick(&mut self) -> (f32, f32, f32) {
        let (mut l, mut r) = (0.0, 0.0);
        for v in self.voices.iter_mut().flatten() {
            let o = match v.voice.as_mut() {
                Some(ev) => ev.tick(),
                None => continue,
            };
            let x = o.front * v.dir[0].tick() + o.rear * v.dir[1].tick() + o.case * v.dir[2].tick();
            v.line.push(x);
            v.delay = (v.delay + v.d_step).max(1.0);
            let s = v.air.lp(v.line.read(v.delay)) * v.gain.tick() * v.fade.tick();
            l += s * v.gl.tick();
            r += s * v.gr.tick();
        }
        (l, r, (l + r) * 0.5)
    }

    pub fn active(&self) -> usize {
        self.voices.iter().flatten().count()
    }
}

#[cfg(test)]
mod tests {
    use super::super::params::{EngineFrame, Frame, MAX_TRAFFIC, TrafficFrame};
    use super::super::synth::Synth;
    use glam::Vec3;

    /// RMS of the last second of 2 s heard by a spectator (no own aircraft)
    /// with one jet `dist` m ahead, moving at `speed` towards the ear.
    fn heard(dist: f32, speed: f32) -> f32 {
        let def = crate::flight::find("lancer").expect("lancer");
        let mut p = crate::audio::capture::profile_for(&def, 8.0);
        p.engines = 1;
        let pos = Vec3::new(0.0, 0.0, -dist);
        let mut f = Frame { active: false, spectator: true, warp: 1.0, ear_right: Vec3::X, ear_up: Vec3::Y, ear_fwd: Vec3::NEG_Z, day: 1.0, ..Default::default() };
        let eng = EngineFrame { pos, rpm: p.n1_rpm * 0.9, spool: 0.9, thrust: 0.8, ab: 0.0, running: true, throttle: 0.9 };
        f.traffic[0] = TrafficFrame { id: 7, profile: p, n_engines: 1, eng, pos, vel: Vec3::new(0.0, 0.0, speed), axis: Vec3::Z, tas: speed, ..Default::default() };
        let mut s = Synth::new(48_000.0);
        let mut out = vec![0f32; 48_000 * 2 / 10];
        let mut acc = 0.0f64;
        let mut n = 0usize;
        for k in 0..20 {
            // Fresh frames every 0.1 s, as the game sends them.
            let mut g = f;
            g.traffic[0].pos = pos + g.traffic[0].vel * (k as f32 * 0.1);
            s.push_frame(&g);
            s.render(&mut out);
            if k >= 10 {
                acc += out.iter().map(|x| (*x as f64) * (*x as f64)).sum::<f64>();
                n += out.len();
            }
        }
        (acc / n as f64).sqrt() as f32
    }

    #[test]
    fn traffic_is_heard_by_distance() {
        let near = heard(120.0, 0.0);
        let far = heard(2400.0, 0.0);
        assert!(near > 1e-3, "near jet audible: {near}");
        assert!(near > far * 3.0, "distance law: near {near} far {far}");
        // No traffic: silence (spectator, nothing else in the world).
        let mut s = Synth::new(48_000.0);
        let f = Frame { active: false, spectator: true, warp: 1.0, ..Default::default() };
        assert_eq!(f.traffic, [TrafficFrame::default(); MAX_TRAFFIC]);
        s.push_frame(&f);
        let mut out = vec![0f32; 9600];
        s.render(&mut out);
        let quiet = out.iter().map(|x| x.abs()).fold(0.0f32, f32::max);
        assert!(quiet < near, "no traffic is quieter: {quiet}");
    }
}
