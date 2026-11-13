//! The "Dogfight" quick start: 1v1 / 2v2 against AI (Spitfire vs Spitfire,
//! F-16 vs F-16, or mixed eras), a head-on merge at fighting altitude,
//! respawns after a kill and a running score.

use super::ai::{Brain, Difficulty};
use super::{AiCraft, Combat, Life};
use crate::flight::{Aircraft, Controls, Environment, KT};
use glam::{Vec2, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    OneVOne,
    TwoVTwo,
}

impl Format {
    pub fn parse(s: &str) -> Option<Format> {
        match s {
            "1v1" => Some(Format::OneVOne),
            "2v2" => Some(Format::TwoVTwo),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Format::OneVOne => "1v1",
            Format::TwoVTwo => "2v2",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub format: Format,
    /// Aircraft ids: the player's side and the opponents'.
    pub player: String,
    pub opponent: String,
    pub difficulty: Difficulty,
    /// The local aircraft is flown by the AI too (watch mode / captures).
    pub autopilot: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config { format: Format::OneVOne, player: "peregrine".into(), opponent: "peregrine".into(), difficulty: Difficulty::Average, autopilot: false }
    }
}

/// Merge altitude (m) and separation (m) by type.
fn merge_for(jet: bool) -> (f32, f32, f32) {
    if jet { (5500.0, 9000.0, 420.0 * KT) } else { (2600.0, 3800.0, 270.0 * KT) }
}

#[derive(Clone, Debug)]
pub struct Dogfight {
    pub cfg: Config,
    /// Where the fight is (world): the merge point.
    pub centre: Vec3,
    pub heading: f32,
    /// Team kills: (player's side, opponents).
    pub score: (u32, u32),
    /// Seconds until each dead AI unit respawns.
    respawn: Vec<(u32, f32)>,
    /// The local player is due a respawn (the sim does it: it owns the aircraft).
    pub player_respawn: bool,
    player_dead_t: f32,
    pub round: u32,
}

const CALLS: [&str; 6] = ["Viper 1", "Viper 2", "Bandit 1", "Bandit 2", "Bandit 3", "Bandit 4"];

impl Dogfight {
    pub fn rebase(&mut self, d: Vec3) {
        self.centre += d;
    }

    /// Set up the fight around `centre` (the local player is already flying
    /// there in the sim, heading `heading`). Returns the player's unit id.
    pub fn start(c: &mut Combat, cfg: Config, player: Option<&mut Aircraft>, centre: Vec2, ground: f32, heading: f32) -> Option<u32> {
        let pdef = c.def_index(&cfg.player).or(c.def_index("peregrine"))?;
        let odef = c.def_index(&cfg.opponent).unwrap_or(pdef);
        let jet = c.defs[pdef].engine_kind == crate::flight::EngineKind::Jet || c.defs[odef].engine_kind == crate::flight::EngineKind::Jet;
        let (alt, sep, _) = merge_for(jet);
        let y = ground + alt;
        let c3 = Vec3::new(centre.x, y, centre.y);
        let mut df = Dogfight { cfg: cfg.clone(), centre: c3, heading, score: (0, 0), respawn: Vec::new(), player_respawn: false, player_dead_t: 0.0, round: 1 };
        let fwd = Vec3::new(heading.sin(), 0.0, -heading.cos());
        let right = Vec3::new(-fwd.z, 0.0, fwd.x);
        let mut local = None;
        // The local player.
        if let Some(ac) = player {
            let d = &c.defs[pdef];
            let (_, _, spd) = merge_for(d.engine_kind == crate::flight::EngineKind::Jet);
            *ac = Aircraft::airborne(d, c3 - fwd * sep * 0.5, heading, spd);
            ac.sys = crate::systems::Systems::new(d, false);
            let id = c.add_unit(pdef, 0, CALLS[0], None);
            c.local = Some(id);
            local = Some(id);
            if cfg.autopilot {
                // Flown by the AI: the sim copies its controls (see sim.rs).
            }
        }
        let n = if cfg.format == Format::TwoVTwo { 2 } else { 1 };
        // Wingman.
        if n == 2 {
            let pos = c3 - fwd * sep * 0.5 + right * 700.0 - fwd * 300.0;
            df.spawn(c, pdef, 0, CALLS[1], pos, heading);
        }
        for k in 0..n {
            let pos = c3 + fwd * sep * 0.5 + right * (380.0 + 700.0 * k as f32) + Vec3::Y * 60.0 * k as f32;
            df.spawn(c, odef, 1, CALLS[2 + k], pos, heading + std::f32::consts::PI);
        }
        c.dogfight = Some(df);
        local
    }

    /// Create an AI aircraft.
    pub fn spawn(&mut self, c: &mut Combat, def: usize, team: u8, call: &str, pos: Vec3, heading: f32) -> u32 {
        let d = c.defs[def].clone();
        let (_, _, spd) = merge_for(d.engine_kind == crate::flight::EngineKind::Jet);
        let mut ac = Aircraft::airborne(&d, pos, heading, spd);
        ac.sys = crate::systems::Systems::new(&d, false);
        let ctl = Controls { throttle: 0.9, gear_down: false, ..Default::default() };
        let seed = (c.units.len() as u32 + 1).wrapping_mul(0x9E37_79B9) ^ self.round;
        let brain = Brain::new(self.cfg.difficulty, seed);
        let ai = AiCraft { prev_pos: ac.pos, prev_rot: ac.rot, ac, ctl, brain, debris: None, known_lost: 0, passive: false, external: false };
        c.add_unit(def, team, call, Some(Box::new(ai)))
    }

    /// Respawn AI after a kill; flag the player's respawn; keep the score.
    pub fn step(&mut self, c: &mut Combat, env: &dyn Environment, dt: f32) {
        // Score from the kill feed's Kill events is kept by `credit`; teams here.
        let mut a = 0;
        let mut b = 0;
        for u in &c.units {
            if u.team == 0 {
                a += u.kills;
            } else {
                b += u.kills;
            }
        }
        self.score = (a, b);
        for u in c.units.iter() {
            let gone = match u.life {
                Life::Dead(s) => s > 6.0,
                Life::Ejected(s) | Life::Doomed(s) => s > 18.0,
                Life::Alive => false,
            };
            if !gone {
                continue;
            }
            if u.ai.is_some() {
                if !self.respawn.iter().any(|r| r.0 == u.id) {
                    self.respawn.push((u.id, 2.0));
                }
            } else if Some(u.id) == c.local {
                self.player_dead_t += dt;
                if !self.player_respawn {
                    self.player_respawn = true;
                }
            }
        }
        // Where the fight is now: around the local player (or the centre).
        let focus = c.local_unit().and_then(|u| u.ai.as_ref().map(|a| a.ac.pos)).unwrap_or(self.centre);
        let mut done = Vec::new();
        for r in &mut self.respawn {
            r.1 -= dt;
            if r.1 <= 0.0 {
                done.push(r.0);
            }
        }
        self.respawn.retain(|r| r.1 > 0.0);
        for id in done {
            let Some(i) = c.units.iter().position(|u| u.id == id) else { continue };
            let def = c.units[i].def;
            let d = c.defs[def].clone();
            let jet = d.engine_kind == crate::flight::EngineKind::Jet;
            let (alt, sep, spd) = merge_for(jet);
            self.round += 1;
            let ang = self.round as f32 * 2.399;
            let off = Vec3::new(ang.cos(), 0.0, ang.sin()) * sep * 0.6;
            let mut pos = focus_pos(self, focus) + off;
            let g = env.ground_height(Vec2::new(pos.x, pos.z));
            pos.y = pos.y.max(g + alt * 0.8);
            let to = (focus - pos).normalize_or(Vec3::Z);
            let heading = to.x.atan2(-to.z);
            let mut ac = Aircraft::airborne(&d, pos, heading, spd);
            ac.sys = crate::systems::Systems::new(&d, false);
            let seed = id.wrapping_mul(0x85EB_CA6B) ^ self.round;
            let brain = Brain::new(self.cfg.difficulty, seed);
            c.units[i].ai = Some(Box::new(AiCraft {
                prev_pos: ac.pos,
                prev_rot: ac.rot,
                ac,
                ctl: Controls { throttle: 0.9, gear_down: false, ..Default::default() },
                brain,
                debris: None,
                known_lost: 0,
                passive: false,
                external: false,
            }));
            c.units[i].prev_pos = pos;
            c.rearm(id);
            let name = c.units[i].callsign.clone();
            c.notes.push(format!("{name} joins the fight"));
        }
    }
}

fn focus_pos(df: &Dogfight, focus: Vec3) -> Vec3 {
    if focus.is_finite() { focus } else { df.centre }
}
