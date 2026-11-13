//! Weapons and air combat (Track W): guns with real exterior ballistics,
//! IR and radar-guided missiles, countermeasures, radar / RWR, per-part
//! hit volumes driving the structural and systems damage models, ejection
//! and bail-out, and AI opponents flying the same flight model.
//!
//! [`Combat`] owns everything that shoots or can be shot. The local
//! player's aircraft stays in `sim::Sim` (passed in each step); AI and
//! (later) network aircraft live in [`Unit::ai`] / are fed by `src/net`.
//! Everything that happens is also an explicit [`events::CombatEvent`] in
//! [`Combat::outbox`] for the network layer (see `docs/WEAPONS.md`).
//!
//! Frames: world positions in the sim's floating-origin frame (rebased
//! with [`Combat::rebase`]); body frame as `flight.rs` (+X left wing,
//! +Y up, +Z nose, origin = CG).

pub mod ai;
pub mod ballistics;
pub mod bdmg;
pub mod eject;
pub mod events;
pub mod guns;
pub mod hitbox;
pub mod missiles;
pub mod render;
pub mod sensors;
pub mod setup;
#[cfg(test)]
mod tests;

use crate::damage::Part;
use crate::flight::{Aircraft, AircraftDef, Controls, Environment};
use events::{CombatEvent, Emission, Id, MissileEndKind, WeaponKind};
use glam::{Quat, Vec2, Vec3};
use hitbox::{HitModel, HitPart};
use missiles::{Contact, Missile, MissileKind, Source};
use std::sync::Arc;

/// Who decides hits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Authority {
    /// Single player: this simulation resolves everything.
    Local,
    /// Multiplayer server: resolves everything and broadcasts the results.
    Server,
    /// Multiplayer client: shoots visually, applies the server's results only.
    Client,
}

/// Server / session rules.
#[derive(Clone, Copy, Debug)]
pub struct Rules {
    /// Combat on: weapons fire and do damage. Off: triggers do nothing.
    pub enabled: bool,
    pub friendly_fire: bool,
    pub authority: Authority,
}

impl Default for Rules {
    fn default() -> Self {
        Rules { enabled: true, friendly_fire: false, authority: Authority::Local }
    }
}

/// The pilot's weapon controls this step (edges are consumed by `step`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Trigger {
    /// Main trigger (Spitfire: the selected guns; F-16: the gun in GUN mode, else ignored).
    pub guns: bool,
    /// Second trigger (Spitfire: cannon only); held.
    pub cannon: bool,
    /// Weapon release (missile): edge.
    pub release: bool,
    /// Radar lock / seeker uncage: edge.
    pub lock: bool,
    /// Countermeasures (flare + chaff program): edge.
    pub cm: bool,
    /// Eject / bail out: edge (the sim asks for it twice).
    pub eject: bool,
    /// Master-mode / gun-group select: edge.
    pub select: bool,
    /// Sight setting (convergence): edge.
    pub sight: bool,
}

/// Master arm mode (F-16) / gun group (Spitfire).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Master {
    Gun,
    Srm,
    Mrm,
}

impl Master {
    pub fn label(self) -> &'static str {
        match self {
            Master::Gun => "GUN",
            Master::Srm => "SRM",
            Master::Mrm => "MRM",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GunGroup {
    Both,
    Cannon,
    Mg,
}

impl GunGroup {
    pub fn label(self) -> &'static str {
        match self {
            GunGroup::Both => "CANNON + MG",
            GunGroup::Cannon => "CANNON",
            GunGroup::Mg => "MG",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Life {
    Alive,
    /// Going down (shot / broken): since (s), why.
    Doomed(f32),
    /// Pilot out (ejected / bailed out).
    Ejected(f32),
    /// Wreck on the ground / gone.
    Dead(f32),
}

/// An AI-flown aircraft.
pub struct AiCraft {
    pub ac: Aircraft,
    pub ctl: Controls,
    pub brain: ai::Brain,
    pub prev_pos: Vec3,
    pub prev_rot: Quat,
    /// Loose parts and the wreck.
    pub debris: Option<crate::debris::Debris>,
    pub known_lost: u16,
    /// Brain off (tests / scripted): controls and triggers are left as set.
    pub passive: bool,
    /// A network player's aircraft: its pose is written from the network
    /// each frame (no physics, no brain; drawn by the multiplayer renderer).
    pub external: bool,
}

/// One combatant.
pub struct Unit {
    pub id: Id,
    pub team: u8,
    pub callsign: String,
    /// Index into [`Combat::defs`].
    pub def: usize,
    pub ai: Option<Box<AiCraft>>,
    pub load: guns::Loadout,
    pub guns: Vec<guns::GunState>,
    /// Gun lines (body frame) for the current convergence.
    pub gun_dirs: Vec<Vec3>,
    pub conv: usize,
    pub missiles: Vec<(MissileKind, u8)>,
    pub flares: u16,
    pub chaff: u16,
    pub master: Master,
    pub group: GunGroup,
    pub trig: Trigger,
    pub hm: Arc<HitModel>,
    pub bd: bdmg::BattleDamage,
    pub radar: sensors::Radar,
    pub rwr: sensors::Rwr,
    pub seeker: sensors::RailSeeker,
    pub life: Life,
    /// (attacker, sim time) of the last hit taken.
    pub last_hit: Option<(Id, WeaponKind, f64)>,
    pub kills: u32,
    pub deaths: u32,
    /// Seconds since this unit was hit / since a round passed close (HUD flash).
    pub hit_flash: f32,
    /// Recoil shake this frame (cockpit camera), 0..1.
    pub shake: f32,
    /// Previous position (fuzing, interpolation).
    pub prev_pos: Vec3,
    /// Countermeasure program (salvo left, interval timer).
    cm_left: u8,
    cm_t: f32,
    /// Rendering state (models.rs), built lazily by the renderer glue.
    pub model: Option<crate::models::AircraftModel>,
}

impl Unit {
    pub fn alive(&self) -> bool {
        self.life == Life::Alive
    }
    pub fn flying(&self) -> bool {
        matches!(self.life, Life::Alive | Life::Doomed(_) | Life::Ejected(_))
    }
    pub fn ammo(&self, group: Option<u8>) -> u32 {
        self.guns.iter().zip(&self.load.guns).filter(|(_, g)| group.is_none_or(|k| g.group == k)).map(|(s, _)| s.ammo).sum()
    }
    pub fn missiles_left(&self, k: MissileKind) -> u8 {
        self.missiles.iter().find(|m| m.0 == k).map(|m| m.1).unwrap_or(0)
    }
}

/// A gun round in flight.
#[derive(Clone, Copy, Debug)]
pub struct Projectile {
    pub shooter: Id,
    pub weapon: WeaponKind,
    pub round: ballistics::Round,
    pub pos: Vec3,
    pub vel: Vec3,
    pub prev: Vec3,
    pub t: f32,
    pub tracer: bool,
    /// HE with a delay fuze (bursts inside, not on the skin).
    pub delay: bool,
    pub cracked: bool,
    /// Visual only (client-side regeneration, or the server's own copy).
    pub visual: bool,
}

/// A flare or chaff bundle.
#[derive(Clone, Copy, Debug)]
pub struct Decoy {
    pub id: u32,
    pub owner: Id,
    pub flare: bool,
    pub pos: Vec3,
    pub vel: Vec3,
    pub t: f32,
}

/// Visual / audio effects raised by the simulation (drained per frame).
#[derive(Clone, Copy, Debug)]
pub enum Fx {
    Muzzle { unit: Id, kind: WeaponKind, rate: f32, pos: Vec3, dir: Vec3, vel: Vec3, cal: f32, rounds: u32 },
    Case { pos: Vec3, vel: Vec3, big: bool },
    Hit { pos: Vec3, normal: Vec3, vel: Vec3, he: bool, cal: f32 },
    Ground { pos: Vec3, he: bool },
    Burst { pos: Vec3, vel: Vec3, energy: f32 },
    Launch { pos: Vec3, vel: Vec3 },
    Dispense { pos: Vec3, vel: Vec3, flare: bool },
}

/// A kill-feed line.
#[derive(Clone, Debug)]
pub struct FeedLine {
    pub t: f64,
    pub text: String,
    /// The local player was involved (highlight).
    pub mine: bool,
}

pub struct Combat {
    pub defs: Vec<AircraftDef>,
    pub rules: Rules,
    pub units: Vec<Unit>,
    pub rounds: Vec<Projectile>,
    pub missiles: Vec<Missile>,
    pub decoys: Vec<Decoy>,
    pub chutes: Vec<eject::Chute>,
    pub canopies: Vec<eject::Canopy>,
    /// Sim time (s).
    pub time: f64,
    /// Sun direction (IR seekers), set by the app each frame.
    pub sun: Vec3,
    /// Network / replay outbox (drained by `src/net` when wired; capped).
    pub outbox: Vec<CombatEvent>,
    /// Effects this frame (drained by the renderer glue).
    pub fx: Vec<Fx>,
    /// Bandit wrecks' particle effects (debris::Fx), drained like `fx`.
    pub debris_fx: Vec<crate::debris::Fx>,
    /// Sounds (drained into `sim.audio_events`).
    pub audio: Vec<crate::audio::params::Event>,
    pub feed: Vec<FeedLine>,
    /// HUD messages for the local player.
    pub notes: Vec<String>,
    pub dogfight: Option<setup::Dogfight>,
    /// The local player's unit id (None: spectating / headless AI fights).
    pub local: Option<Id>,
    /// The local aircraft flown by the dogfight AI (watch mode / captures),
    /// and the controls it wants (the sim copies them).
    pub autopilot: Option<ai::Brain>,
    pub autopilot_ctl: Option<Controls>,
    rng: u32,
    next_id: u32,
    fire_batch: Vec<(Id, u8, u16, u32, f64)>,
    batch_t: f32,
    state_t: f32,
    emits: Vec<(Id, Id, Emission)>,
}

/// RCS (m²) by type class.
fn rcs_of(def: &AircraftDef) -> f32 {
    match def.category {
        crate::flight::Category::FastJet => 4.0,
        crate::flight::Category::Warbird => 6.0,
        _ => 20.0,
    }
}

impl Combat {
    pub fn new(rules: Rules) -> Combat {
        Combat {
            defs: crate::flight::catalogue(),
            rules,
            units: Vec::new(),
            rounds: Vec::new(),
            missiles: Vec::new(),
            decoys: Vec::new(),
            chutes: Vec::new(),
            canopies: Vec::new(),
            time: 0.0,
            sun: Vec3::new(0.3, 0.8, 0.2).normalize(),
            outbox: Vec::new(),
            fx: Vec::new(),
            debris_fx: Vec::new(),
            audio: Vec::new(),
            feed: Vec::new(),
            notes: Vec::new(),
            dogfight: None,
            local: None,
            autopilot: None,
            autopilot_ctl: None,
            rng: 0x2545_F491,
            next_id: 1,
            fire_batch: Vec::new(),
            batch_t: 0.0,
            state_t: 0.0,
            emits: Vec::new(),
        }
    }

    pub fn def_index(&self, id: &str) -> Option<usize> {
        self.defs.iter().position(|d| d.id == id)
    }

    fn rnd(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng >> 8) as f32 / 16_777_216.0
    }

    /// Hit volumes from the flight geometry (the renderer replaces them with
    /// the model's own part boxes and planform once it is loaded).
    pub fn estimate_hits(def: &AircraftDef) -> Arc<HitModel> {
        let sd = crate::damage::struct_def(def.id);
        Arc::new(HitModel::build(def, sd, &crate::debris::PartGeom::estimate(def, sd), hitbox::planform_for(def, sd, &[])))
    }

    /// Add a combatant (unarmed types get no weapons but can still be shot).
    pub fn add_unit(&mut self, def: usize, team: u8, callsign: &str, ai: Option<Box<AiCraft>>) -> Id {
        let d = &self.defs[def];
        let load = guns::loadout(d).unwrap_or(guns::Loadout {
            guns: vec![],
            missiles: vec![],
            flares: 0,
            chaff: 0,
            radar: false,
            rwr: false,
            sight: guns::Sight::Gyro,
            convergence: &[],
            conv_default: 0,
            seat: false,
        });
        let hm = Self::estimate_hits(d);
        let id = self.units.iter().map(|u| u.id + 1).max().unwrap_or(0);
        let gst =
            load.guns.iter().enumerate().map(|(k, g)| guns::GunState { rng: 0x9E37 ^ (id * 31 + k as u32 + 1).wrapping_mul(0x85EB_CA6B), ..guns::GunState::new(g) }).collect();
        let mut u = Unit {
            id,
            team,
            callsign: callsign.to_string(),
            def,
            ai,
            guns: gst,
            gun_dirs: Vec::new(),
            conv: load.conv_default,
            missiles: load.missiles.clone(),
            flares: load.flares,
            chaff: load.chaff,
            master: Master::Gun,
            group: GunGroup::Both,
            trig: Trigger::default(),
            bd: bdmg::BattleDamage::new(&hm),
            hm,
            radar: sensors::Radar { mode: if load.radar { sensors::RadarMode::Search } else { sensors::RadarMode::Off }, ..Default::default() },
            rwr: Default::default(),
            seeker: Default::default(),
            life: Life::Alive,
            last_hit: None,
            kills: 0,
            deaths: 0,
            hit_flash: 99.0,
            shake: 0.0,
            prev_pos: Vec3::ZERO,
            cm_left: 0,
            cm_t: 0.0,
            model: None,
            load,
        };
        self.harmonise(&mut u, 150.0);
        if let Some(a) = &u.ai {
            u.prev_pos = a.ac.pos;
        }
        self.units.push(u);
        id
    }

    fn harmonise(&self, u: &mut Unit, speed: f32) {
        let d = &self.defs[u.def];
        let conv = u.load.convergence.get(u.conv).copied().unwrap_or(0.0);
        u.gun_dirs = u.load.guns.iter().map(|g| guns::gun_line(g, d.eye, conv, speed)).collect();
    }

    pub fn unit(&self, id: Id) -> Option<&Unit> {
        self.units.iter().find(|u| u.id == id)
    }
    pub fn unit_mut(&mut self, id: Id) -> Option<&mut Unit> {
        self.units.iter_mut().find(|u| u.id == id)
    }
    pub fn local_unit(&self) -> Option<&Unit> {
        self.local.and_then(|id| self.unit(id))
    }
    pub fn local_unit_mut(&mut self) -> Option<&mut Unit> {
        let id = self.local?;
        self.unit_mut(id)
    }

    /// Re-arm and repair a unit (respawn): new battle damage, full ammunition.
    pub fn rearm(&mut self, id: Id) {
        let Some(i) = self.units.iter().position(|u| u.id == id) else { return };
        let u = &mut self.units[i];
        u.guns =
            u.load.guns.iter().enumerate().map(|(k, g)| guns::GunState { rng: 0x9E37 ^ (id * 31 + k as u32 + 1).wrapping_mul(0x85EB_CA6B), ..guns::GunState::new(g) }).collect();
        u.missiles = u.load.missiles.clone();
        u.flares = u.load.flares;
        u.chaff = u.load.chaff;
        u.bd = bdmg::BattleDamage::new(&u.hm);
        u.life = Life::Alive;
        u.last_hit = None;
        u.radar.unlock();
        u.seeker.locked = None;
        u.trig = Trigger::default();
        u.cm_left = 0;
    }

    /// The floating origin moved.
    pub fn rebase(&mut self, d: Vec3) {
        for u in &mut self.units {
            u.prev_pos += d;
            if let Some(a) = &mut u.ai {
                a.ac.rebase(d);
                a.prev_pos += d;
                if let Some(db) = &mut a.debris {
                    db.rebase(d);
                }
            }
            for t in &mut u.radar.tracks {
                t.pos += d;
            }
        }
        for r in &mut self.rounds {
            r.pos += d;
            r.prev += d;
        }
        for m in &mut self.missiles {
            m.pos += d;
            m.prev_pos += d;
            m.est_pos += d;
        }
        for x in &mut self.decoys {
            x.pos += d;
        }
        for c in &mut self.chutes {
            c.rebase(d);
        }
        for c in &mut self.canopies {
            c.pos += d;
        }
        if let Some(df) = &mut self.dogfight {
            df.rebase(d);
        }
    }

    /// Effective sight-line convergence for the local unit (m), 0 = none.
    pub fn convergence(&self, u: &Unit) -> f32 {
        u.load.convergence.get(u.conv).copied().unwrap_or(0.0)
    }

    fn views(&self, player: Option<&Aircraft>) -> Vec<sensors::View> {
        self.units
            .iter()
            .filter_map(|u| {
                let ac = match &u.ai {
                    Some(a) => &a.ac,
                    None => player?,
                };
                let d = &self.defs[u.def];
                Some(sensors::View { id: u.id, team: u.team, pos: ac.pos, vel: ac.vel, rot: ac.rot, rcs: rcs_of(d), alive: u.alive() && ac.crashed.is_none(), has_rwr: u.load.rwr })
            })
            .collect()
    }

    /// IR contacts as seen from `from` (units, flares).
    fn ir_contacts(&self, views: &[sensors::View], from: Vec3, skip: Id) -> Vec<Contact> {
        let mut c = Vec::new();
        for (u, v) in self.units.iter().zip(views.iter()) {
            if u.id == skip || !u.flying() {
                continue;
            }
            let d = &self.defs[u.def];
            let jet = d.engine_kind == crate::flight::EngineKind::Jet;
            let (power, ab) = match &u.ai {
                Some(a) => (a.ac.engine_power(), a.ac.afterburner()),
                None => (0.8, 0.0),
            };
            let fire = u.bd.fire * 800.0;
            c.push(Contact { src: Source::Unit(u.id), pos: v.pos, vel: v.vel, strength: missiles::ir_intensity(v.pos, v.rot * Vec3::Z, from, jet, power, ab) + fire, age: 99.0 });
        }
        for x in &self.decoys {
            if x.flare {
                c.push(Contact { src: Source::Flare(x.id), pos: x.pos, vel: x.vel, strength: missiles::flare_intensity(x.t), age: x.t });
            }
        }
        c
    }

    fn radar_contacts(&self, views: &[sensors::View]) -> Vec<Contact> {
        let mut c: Vec<Contact> = views.iter().filter(|v| v.alive).map(|v| Contact { src: Source::Unit(v.id), pos: v.pos, vel: v.vel, strength: v.rcs, age: 99.0 }).collect();
        for x in &self.decoys {
            if !x.flare && x.t < 6.0 {
                // A bloomed bundle: ~40 m², fading as it disperses.
                let s = 40.0 * (x.t / 0.3).min(1.0) * (1.0 - x.t / 6.0);
                c.push(Contact { src: Source::Chaff(x.id), pos: x.pos, vel: x.vel, strength: s, age: x.t });
            }
        }
        c
    }

    /// The local player's controls (the window layer sets these each frame).
    pub fn set_trigger(&mut self, t: Trigger) {
        if let Some(u) = self.local_unit_mut() {
            let keep = u.trig;
            u.trig = Trigger {
                guns: t.guns,
                cannon: t.cannon,
                release: keep.release || t.release,
                lock: keep.lock || t.lock,
                cm: keep.cm || t.cm,
                eject: keep.eject || t.eject,
                select: keep.select || t.select,
                sight: keep.sight || t.sight,
            };
        }
    }

    /// One fixed physics step for everything in the fight. `player`: the
    /// local aircraft (owned by the sim) for the unit without `ai`.
    pub fn step(&mut self, mut player: Option<(&mut Aircraft, &AircraftDef)>, env: &dyn Environment, dt: f32) {
        self.time += dt as f64;
        let pv = player.as_ref().map(|p| &*p.0);
        let views = self.views(pv);
        let amb = crate::systems::Ambient::default();
        // ---- AI pilots + flight models.
        let n = self.units.len();
        for i in 0..n {
            if self.units[i].ai.is_none() {
                continue;
            }
            let def = self.defs[self.units[i].def].clone();
            let mut a = self.units[i].ai.take().unwrap();
            if a.external {
                self.units[i].ai = Some(a);
                continue;
            }
            a.prev_pos = a.ac.pos;
            a.prev_rot = a.ac.rot;
            if !a.passive {
                let mut ctl = a.ctl;
                if a.ac.crashed.is_none() {
                    let o = self.think(i, &views, &a.ac, &def, &mut a.brain, &mut ctl, env, dt);
                    self.orders(i, o, &views);
                }
                a.ctl = ctl;
            }
            if a.ac.crashed.is_none() {
                let ctl = a.ctl;
                crate::systems::step(&mut a.ac, &def, &ctl, &amb, dt);
                a.ac.step(&def, &ctl, env, dt);
            }
            self.units[i].ai = Some(a);
        }
        // The local aircraft on autopilot (watch mode / captures): same brain, the sim applies the controls.
        if let (Some(mut brain), Some(p), Some(i)) = (self.autopilot.take(), player.as_ref(), self.local.and_then(|id| self.units.iter().position(|u| u.id == id))) {
            let mut ctl = self.autopilot_ctl.unwrap_or(Controls { throttle: 0.9, gear_down: false, ..Default::default() });
            if p.0.crashed.is_none() {
                let o = self.think(i, &views, p.0, p.1, &mut brain, &mut ctl, env, dt);
                self.orders(i, o, &views);
            }
            self.autopilot_ctl = Some(ctl);
            self.autopilot = Some(brain);
        }

        // ---- player-side bookkeeping (its flight model is stepped by the sim).
        // ---- sensors: radar, RWR, AIM-9 seeker on the rail.
        let views = self.views(player.as_ref().map(|p| &*p.0));
        self.emits.clear();
        let mut paints: Vec<(Id, Id, Emission)> = Vec::new();
        for i in 0..n {
            let Some(me) = views.iter().copied().find(|v| v.id == self.units[i].id) else { continue };
            if !self.units[i].flying() || !self.units[i].load.radar {
                continue;
            }
            let others: Vec<sensors::View> = views.iter().copied().filter(|v| v.id != me.id).collect();
            let em = self.units[i].radar.update(&me, &others, dt);
            for (to, k) in em {
                paints.push((me.id, to, k));
            }
        }
        for m in &self.missiles {
            if let Some(tg) = m.target {
                if m.active && matches!(m.def.seeker, missiles::SeekerKind::Radar { .. }) {
                    paints.push((u32::MAX - m.id, tg, Emission::Active));
                } else if m.datalink && !m.active {
                    paints.push((m.owner, tg, Emission::Launch));
                }
            }
        }
        for i in 0..n {
            if !self.units[i].load.rwr {
                continue;
            }
            let id = self.units[i].id;
            let Some(me) = views.iter().copied().find(|v| v.id == id) else { continue };
            let hits: Vec<(Id, Vec3, Emission)> = paints
                .iter()
                .filter(|p| p.1 == id)
                .filter_map(|p| {
                    let pos = views.iter().find(|v| v.id == p.0).map(|v| v.pos).or_else(|| self.missiles.iter().find(|m| u32::MAX - m.id == p.0).map(|m| m.pos))?;
                    Some((p.0, pos, p.2))
                })
                .collect();
            self.units[i].rwr.update(me.pos, me.rot, &hits, dt);
        }
        self.emits = paints;
        // AIM-9 seekers (slaved to the radar lock, else boresight).
        for i in 0..n {
            if self.units[i].missiles_left(MissileKind::Aim9) == 0 || !self.units[i].alive() {
                self.units[i].seeker.locked = None;
                self.units[i].seeker.signal = 0.0;
                continue;
            }
            let id = self.units[i].id;
            let Some(me) = views.iter().copied().find(|v| v.id == id) else { continue };
            let contacts = self.ir_contacts(&views, me.pos, id);
            let slave = self.units[i].radar.stt.and_then(|s| views.iter().find(|v| v.id == s)).map(|v| (v.pos - me.pos).normalize_or_zero());
            let sun = self.sun;
            let missiles::SeekerKind::Ir { gimbal, fov, r_ref, .. } = missiles::AIM9M.seeker else { unreachable!() };
            let ai_flown = self.units[i].ai.is_some() || (self.local == Some(id) && self.autopilot.is_some());
            let u = &mut self.units[i];
            let srm = u.master == Master::Srm || ai_flown;
            if !srm {
                u.seeker.signal = 0.0;
                u.seeker.locked = None;
                continue;
            }
            u.seeker.update(me.pos, me.rot, slave, &contacts, sun, gimbal, fov, r_ref, dt);
            // AI: uncage when it hears the growl on its target.
            if ai_flown && u.seeker.locked.is_none() && u.seeker.signal > 0.3 && u.seeker.uncage(me.pos, &contacts, fov, r_ref) {
                // Tone: the AI goes to SRM.
                u.master = Master::Srm;
            }
        }
        // ---- the pilots' switches.
        for i in 0..n {
            self.switches(i, &views);
        }
        // ---- guns
        for i in 0..n {
            self.fire_guns(i, &mut player, dt);
        }
        // ---- missiles: release
        for i in 0..n {
            self.release(i, &views);
        }
        // ---- countermeasures
        for i in 0..n {
            self.dispense(i, &views, dt);
        }
        // ---- rounds in flight
        self.fly_rounds(&mut player, env, dt);
        // ---- missiles in flight
        self.fly_missiles(&mut player, env, dt);
        // ---- decoys
        for x in &mut self.decoys {
            x.t += dt;
            let k = if x.flare { 0.9 } else { 3.5 };
            x.vel += (Vec3::new(0.0, if x.flare { -9.81 } else { -0.6 }, 0.0) - x.vel * k) * dt;
            x.pos += x.vel * dt;
        }
        self.decoys.retain(|x| x.t < if x.flare { 5.0 } else { 7.0 });
        // ---- damage processes, deaths, ejections
        self.life_cycle(&mut player, env, dt);
        // ---- chutes / canopies
        for c in &mut self.chutes {
            let g = env.ground_height(Vec2::new(c.pos.x, c.pos.z));
            c.step(g, env.wind(c.pos), dt);
        }
        self.chutes.retain(|c| c.landed_t < 40.0);
        for c in &mut self.canopies {
            let g = env.ground_height(Vec2::new(c.pos.x, c.pos.z));
            c.step(g, dt);
        }
        self.canopies.retain(|c| c.t < 40.0);
        // ---- events: batch the gun fire, missile states.
        self.batch_t += dt;
        if self.batch_t >= 0.1 {
            self.batch_t = 0.0;
            for (s, g, r, seed, t0) in self.fire_batch.drain(..) {
                self.outbox.push(CombatEvent::Fire { shooter: s, gun: g, rounds: r, seed, t: t0 });
            }
        }
        self.state_t += dt;
        if self.state_t >= 0.1 && self.rules.authority != Authority::Client {
            self.state_t = 0.0;
            for m in &self.missiles {
                self.outbox.push(CombatEvent::MissileState { id: m.id, pos: m.pos, vel: m.vel, target: m.target, active: m.active });
            }
        }
        if self.outbox.len() > 8192 {
            // Nobody is draining it (single player): keep the newest.
            self.outbox.drain(..4096);
        }
        for u in &mut self.units {
            u.hit_flash += dt;
            u.shake = (u.shake - dt * 6.0).max(0.0);
            if let Some(a) = &u.ai {
                u.prev_pos = a.ac.pos;
            } else if let Some(p) = player.as_ref() {
                u.prev_pos = p.0.pos;
            }
        }
        // ---- dogfight: respawns, score (setup.rs).
        if let Some(mut df) = self.dogfight.take() {
            df.step(self, env, dt);
            self.dogfight = Some(df);
        }
    }

    /// One AI decision / control step for unit `i` flying `ac`.
    #[allow(clippy::too_many_arguments)]
    fn think(&self, i: usize, views: &[sensors::View], ac: &Aircraft, def: &AircraftDef, brain: &mut ai::Brain, ctl: &mut Controls, env: &dyn Environment, dt: f32) -> ai::Orders {
        let u = &self.units[i];
        let threats: Vec<ai::Threat> = self
            .missiles
            .iter()
            .filter(|m| m.target == Some(u.id) && m.phase != missiles::Phase::Ballistic)
            .map(|m| ai::Threat { pos: m.pos, vel: m.vel, radar: matches!(m.def.seeker, missiles::SeekerKind::Radar { .. }) })
            .collect();
        let enemies: Vec<sensors::View> = views.iter().copied().filter(|v| v.team != u.team).collect();
        let in_flight = self.missiles.iter().filter(|m| m.owner == u.id).count();
        let seeker_lock = match u.seeker.locked {
            Some(Source::Unit(id)) => Some(id),
            _ => None,
        };
        let doomed = !u.alive();
        let pilot_dead = u.bd.pilot == bdmg::Pilot::Dead || matches!(u.life, Life::Ejected(_));
        let me = sensors::View { id: u.id, team: u.team, pos: ac.pos, vel: ac.vel, rot: ac.rot, rcs: rcs_of(def), alive: u.alive(), has_rwr: u.load.rwr };
        let gh = |x: Vec3| env.ground_height(Vec2::new(x.x, x.z));
        let (p, v) = (ac.pos, ac.vel);
        let s = ai::Situation {
            me,
            def,
            ac,
            enemies: &enemies,
            threats: &threats,
            ground: [gh(p), gh(p + v * 3.0), gh(p + v * 8.0)],
            muzzle_v: u.load.guns.first().map(|g| g.round.muzzle_v).unwrap_or(800.0),
            gun_range: if def.category == crate::flight::Category::FastJet { 900.0 } else { 420.0 },
            has_gun: u.guns.iter().any(|g| g.ammo > 0) && !u.bd.guns_jammed,
            aim9: u.missiles_left(MissileKind::Aim9) > 0,
            aim120: u.missiles_left(MissileKind::Aim120) > 0,
            seeker_lock,
            radar_lock: u.radar.stt,
            missiles_in_flight: in_flight,
            time: self.time as f32,
        };
        brain.update(&s, ctl, doomed, pilot_dead, dt);
        brain.orders
    }

    /// Turn the AI's orders into the unit's switches and triggers.
    fn orders(&mut self, i: usize, o: ai::Orders, views: &[sensors::View]) {
        let u = &mut self.units[i];
        let alive = u.alive();
        u.trig.guns = o.guns && alive;
        if alive {
            if o.aim9 {
                u.master = Master::Srm;
                u.trig.release = true;
            } else if o.aim120 {
                u.master = Master::Mrm;
                u.trig.release = true;
            } else if o.guns {
                u.master = Master::Gun;
            }
            if o.countermeasures {
                u.trig.cm = true;
            }
            if let Some(id) = o.lock {
                if u.radar.stt != Some(id) {
                    if let Some(me) = views.iter().copied().find(|v| v.id == u.id) {
                        let enemies: Vec<sensors::View> = views.iter().copied().filter(|v| v.team != u.team).collect();
                        u.radar.lock_next(&me, &enemies);
                    }
                }
            }
        }
        if o.eject && matches!(u.life, Life::Doomed(_)) {
            u.trig.eject = true;
        }
    }

    /// Master mode / gun group / convergence / lock switches.
    fn switches(&mut self, i: usize, views: &[sensors::View]) {
        let u = &mut self.units[i];
        let tr = &mut u.trig;
        let (select, sight, lock) = (std::mem::take(&mut tr.select), std::mem::take(&mut tr.sight), std::mem::take(&mut tr.lock));
        if select {
            if u.load.missiles.is_empty() {
                u.group = match u.group {
                    GunGroup::Both => GunGroup::Cannon,
                    GunGroup::Cannon => GunGroup::Mg,
                    GunGroup::Mg => GunGroup::Both,
                };
                if u.ai.is_none() {
                    self.notes.push(format!("Guns: {}", u.group.label()));
                }
            } else {
                u.master = match u.master {
                    Master::Gun => Master::Srm,
                    Master::Srm => Master::Mrm,
                    Master::Mrm => Master::Gun,
                };
                u.seeker.locked = None;
                if u.ai.is_none() {
                    self.notes.push(format!("Master mode {}", u.master.label()));
                }
            }
        }
        if sight && !u.load.convergence.is_empty() {
            u.conv = (u.conv + 1) % u.load.convergence.len();
            let yd = u.load.convergence[u.conv] / 0.9144;
            if u.ai.is_none() {
                self.notes.push(format!("Guns harmonised at {yd:.0} yd"));
            }
            let u = &mut self.units[i];
            let d = &self.defs[u.def];
            let conv = u.load.convergence[u.conv];
            u.gun_dirs = u.load.guns.iter().map(|g| guns::gun_line(g, d.eye, conv, 150.0)).collect();
        }
        if lock {
            let u = &mut self.units[i];
            let Some(me) = views.iter().copied().find(|v| v.id == u.id) else { return };
            let others: Vec<sensors::View> = views.iter().copied().filter(|v| v.id != me.id).collect();
            match u.master {
                Master::Srm => {
                    if u.seeker.locked.is_some() {
                        u.seeker.locked = None;
                    } else {
                        let contacts = {
                            let id = u.id;
                            let pos = me.pos;
                            self.ir_contacts(views, pos, id)
                        };
                        let missiles::SeekerKind::Ir { fov, r_ref, .. } = missiles::AIM9M.seeker else { unreachable!() };
                        let u = &mut self.units[i];
                        let ok = u.seeker.uncage(me.pos, &contacts, fov, r_ref);
                        if u.ai.is_none() && !ok {
                            self.notes.push("No IR target in the seeker".into());
                        }
                    }
                }
                _ if u.load.radar => {
                    let got = u.radar.lock_next(&me, &others);
                    if u.ai.is_none() {
                        let msg = match got {
                            Some(id) => format!("Radar lock: {}", self.units.iter().find(|x| x.id == id).map(|x| x.callsign.clone()).unwrap_or_default()),
                            None => "Radar: no target".into(),
                        };
                        self.notes.push(msg);
                    }
                }
                _ => {}
            }
        }
    }

    fn fire_guns(&mut self, i: usize, player: &mut Option<(&mut Aircraft, &AircraftDef)>, dt: f32) {
        let enabled = self.rules.enabled;
        let visual = !self.resolves(self.units[i].id);
        let u = &mut self.units[i];
        let alive = u.alive() && enabled;
        let jam = u.bd.guns_jammed;
        let (pos, vel, rot, mass) = match (&u.ai, player.as_ref()) {
            (Some(a), _) => (a.ac.pos, a.ac.vel, a.ac.rot, a.ac.dmg.mass(&self.defs[u.def])),
            (None, Some(p)) => (p.0.pos, p.0.vel, p.0.rot, p.0.dmg.mass(p.1)),
            _ => return,
        };
        let gun_mode = u.master == Master::Gun || u.load.missiles.is_empty();
        let mut recoil = Vec3::ZERO;
        let mut new_rounds = Vec::new();
        let mut fx = Vec::new();
        for (k, g) in u.load.guns.iter().enumerate() {
            let on = alive
                && !jam
                && gun_mode
                && if g.group == 0 && u.load.convergence.is_empty() {
                    u.trig.guns
                } else {
                    (u.trig.guns && (u.group == GunGroup::Both || (u.group == GunGroup::Cannon && g.group == 0) || (u.group == GunGroup::Mg && g.group == 1)))
                        || (u.trig.cannon && g.group == 0)
                };
            let st = &mut u.guns[k];
            let seed0 = st.rng;
            let (nr, rate) = guns::tick(st, g, on, dt);
            if nr == 0 {
                continue;
            }
            let dir0 = rot * u.gun_dirs.get(k).copied().unwrap_or(Vec3::Z);
            let muzzle = pos + rot * g.muzzle;
            for j in 0..nr {
                let (u1, u2) = (xorshift(&mut st.rng), xorshift(&mut st.rng));
                let dir = guns::disperse(dir0, g.sigma, u1, u2);
                let v = vel + dir * g.round.muzzle_v * (1.0 + (xorshift(&mut st.rng) - 0.5) * 0.01);
                let frac = (j as f32 + 0.5) / nr as f32;
                let p = muzzle + vel * (dt * frac);
                let tracer = g.tracer_every > 0 && (st.fired - nr + j + 1).is_multiple_of(g.tracer_every);
                new_rounds.push(Projectile {
                    shooter: u.id,
                    weapon: g.kind,
                    round: g.round,
                    pos: p,
                    vel: v,
                    prev: p,
                    t: 0.0,
                    tracer,
                    delay: g.kind == WeaponKind::M61,
                    cracked: false,
                    visual,
                });
                recoil -= dir * (g.round.mass * g.round.muzzle_v);
            }
            fx.push(Fx::Muzzle { unit: u.id, kind: g.kind, rate, pos: muzzle, dir: dir0, vel, cal: g.round.calibre, rounds: nr });
            if let Some(e) = g.eject {
                fx.push(Fx::Case { pos: pos + rot * (g.muzzle - Vec3::Z * 0.8), vel: vel + rot * e * 6.0, big: g.round.calibre > 0.015 });
            }
            // Batch the fire events per gun.
            match self.fire_batch.iter_mut().find(|b| b.0 == u.id && b.1 == k as u8) {
                Some(b) => b.2 = b.2.saturating_add(nr as u16),
                None => self.fire_batch.push((u.id, k as u8, nr as u16, seed0, self.time)),
            }
            u.shake = (u.shake + 0.08 * nr as f32 * (g.round.calibre / 0.02)).min(1.0);
        }
        if recoil != Vec3::ZERO {
            let dv = recoil / mass.max(100.0);
            match (&mut u.ai, player.as_mut()) {
                (Some(a), _) => a.ac.vel += dv,
                (None, Some(p)) => p.0.vel += dv,
                _ => {}
            }
        }
        self.rounds.extend(new_rounds);
        self.fx.extend(fx);
    }

    fn release(&mut self, i: usize, views: &[sensors::View]) {
        let rel = std::mem::take(&mut self.units[i].trig.release);
        if !rel || !self.rules.enabled || !self.units[i].alive() {
            return;
        }
        let u = &self.units[i];
        let Some(me) = views.iter().copied().find(|v| v.id == u.id) else { return };
        let kind = match u.master {
            Master::Srm => MissileKind::Aim9,
            Master::Mrm => MissileKind::Aim120,
            Master::Gun => return,
        };
        if u.missiles_left(kind) == 0 {
            if u.ai.is_none() {
                self.notes.push(format!("No {} left", missiles::def_of(kind).name));
            }
            return;
        }
        let (target, seek, est) = match kind {
            MissileKind::Aim9 => {
                let tgt = match u.seeker.locked {
                    Some(Source::Unit(id)) => Some(id),
                    _ => None,
                };
                (tgt, u.seeker.dir, tgt.and_then(|id| views.iter().find(|v| v.id == id)).map(|v| (v.pos, v.vel)))
            }
            MissileKind::Aim120 => {
                let Some(id) = u.radar.stt.or(u.radar.tracks.first().map(|t| t.id)) else {
                    if u.ai.is_none() {
                        self.notes.push("AIM-120: no radar target".into());
                    }
                    return;
                };
                let tr = u.radar.track(id).map(|t| (t.pos, t.vel));
                (Some(id), tr.map(|t| (t.0 - me.pos).normalize()).unwrap_or(me.rot * Vec3::Z), tr)
            }
        };
        if kind == MissileKind::Aim9 && target.is_none() && u.ai.is_some() {
            return;
        }
        let side = if u.missiles_left(kind) % 2 == 0 { 1.0 } else { -1.0 };
        let span = self.defs[u.def].wingspan * 0.5;
        let station = match kind {
            MissileKind::Aim9 => Vec3::new(side * span, -0.1, -0.5),
            MissileKind::Aim120 => Vec3::new(side * (span - 1.2), -0.45, -0.2),
        };
        let pos = me.pos + me.rot * station;
        let id = self.next_id;
        self.next_id += 1;
        let owner = u.id;
        let m = Missile::new(id, missiles::def_of(kind), owner, target, pos, me.vel + me.rot * Vec3::new(0.0, -2.0, 0.0), seek, est);
        let u = &mut self.units[i];
        if let Some(e) = u.missiles.iter_mut().find(|m| m.0 == kind) {
            e.1 -= 1;
        }
        if kind == MissileKind::Aim9 {
            u.seeker.locked = None;
        }
        self.fx.push(Fx::Launch { pos, vel: me.vel });
        self.audio.push(crate::audio::params::Event::Missile { pos, own: Some(owner) == self.local });
        self.outbox.push(CombatEvent::MissileLaunch { id, shooter: owner, kind: kind.weapon(), target, pos, vel: m.vel });
        let tname = target.and_then(|t| self.unit(t)).map(|u| u.callsign.clone());
        let me_name = self.units[i].callsign.clone();
        let call = match kind {
            MissileKind::Aim9 => "Fox Two",
            MissileKind::Aim120 => "Fox Three",
        };
        if Some(owner) == self.local || target == self.local {
            self.notes.push(match tname {
                Some(t) => format!("{me_name}: {call} on {t}"),
                None => format!("{me_name}: {call}"),
            });
        }
        self.missiles.push(m);
    }

    fn dispense(&mut self, i: usize, views: &[sensors::View], dt: f32) {
        let start = std::mem::take(&mut self.units[i].trig.cm);
        let u = &mut self.units[i];
        if start && u.cm_left == 0 && (u.flares > 0 || u.chaff > 0) {
            // Program: 4 × (flare + chaff), 0.25 s apart.
            u.cm_left = 4;
            u.cm_t = 0.0;
        }
        if u.cm_left == 0 || !u.flying() {
            return;
        }
        u.cm_t -= dt;
        if u.cm_t > 0.0 {
            return;
        }
        u.cm_t = 0.25;
        u.cm_left -= 1;
        let Some(me) = views.iter().copied().find(|v| v.id == u.id) else { return };
        let owner = u.id;
        let mut out = Vec::new();
        if u.flares > 0 {
            u.flares -= 1;
            out.push(true);
        }
        if u.chaff > 0 {
            u.chaff -= 1;
            out.push(false);
        }
        for flare in out {
            let side = if self.rnd() < 0.5 { 1.0 } else { -1.0 };
            let pos = me.pos + me.rot * Vec3::new(side * 0.6, -0.6, -3.5);
            let vel = me.vel + me.rot * Vec3::new(side * 6.0, -22.0, -4.0);
            let id = self.next_id;
            self.next_id += 1;
            self.decoys.push(Decoy { id, owner, flare, pos, vel, t: 0.0 });
            self.fx.push(Fx::Dispense { pos, vel, flare });
            self.outbox.push(CombatEvent::Countermeasure { owner, flare, count: 1, pos, vel });
            if flare {
                self.audio.push(crate::audio::params::Event::Countermeasure { pos, own: Some(owner) == self.local });
            }
        }
    }

    fn fly_rounds(&mut self, player: &mut Option<(&mut Aircraft, &AircraftDef)>, env: &dyn Environment, dt: f32) {
        let mut rounds = std::mem::take(&mut self.rounds);
        let listener = self.local.and_then(|id| self.unit(id)).and_then(|u| match &u.ai {
            Some(a) => Some(a.ac.pos),
            None => player.as_ref().map(|p| p.0.pos),
        });
        rounds.retain_mut(|r| {
            r.prev = r.pos;
            let (np, nv) = ballistics::step(&r.round, r.pos, r.vel, Vec3::ZERO, dt);
            r.pos = np;
            r.vel = nv;
            r.t += dt;
            // A crack close by the listener (supersonic) or a whiz.
            if let Some(l) = listener {
                if !r.cracked {
                    let (d, _) = missiles::closest_approach(r.prev, r.pos, l, l);
                    let own = self.unit(r.shooter).is_some_and(|u| Some(u.id) == self.local);
                    if d < 18.0 && !own {
                        r.cracked = true;
                        self.audio.push(crate::audio::params::Event::Crack { pos: r.pos, miss: d, cal: r.round.calibre, supersonic: r.vel.length() > 345.0 });
                    }
                }
            }
            // Ground.
            let g = env.ground_height(Vec2::new(r.pos.x, r.pos.z));
            if r.pos.y <= g {
                self.fx.push(Fx::Ground { pos: Vec3::new(r.pos.x, g, r.pos.z), he: r.round.he > 0.0 });
                return false;
            }
            if r.t > 6.0 || r.vel.length() < 120.0 {
                return false;
            }
            if r.visual || !self.rules.enabled {
                return true;
            }
            // Aircraft.
            !self.resolve_round(r, player)
        });
        self.rounds = rounds;
    }

    /// Hit test one round against everyone; apply damage. True if it was stopped.
    fn resolve_round(&mut self, r: &Projectile, player: &mut Option<(&mut Aircraft, &AircraftDef)>) -> bool {
        let shooter_team = self.unit(r.shooter).map(|u| u.team);
        for i in 0..self.units.len() {
            let u = &self.units[i];
            if u.id == r.shooter || !u.flying() || (!self.rules.friendly_fire && Some(u.team) == shooter_team) {
                continue;
            }
            let (pos, rot, vel, prev) = match (&u.ai, player.as_ref()) {
                (Some(a), _) => (a.ac.pos, a.ac.rot, a.ac.vel, a.prev_pos),
                (None, Some(p)) => (p.0.pos, p.0.rot, p.0.vel, u.prev_pos),
                _ => continue,
            };
            let prev = if prev.distance(pos) > 200.0 { pos - vel * (1.0 / 120.0) } else { prev };
            // Broad phase.
            let (d, _) = missiles::closest_approach(r.prev, r.pos, prev, pos);
            if d > u.hm.radius {
                continue;
            }
            let inv = rot.inverse();
            let a = inv * (r.prev - prev);
            let b = inv * (r.pos - pos);
            let hits = u.hm.intersect(a, b);
            if hits.is_empty() {
                continue;
            }
            let v_rel = (r.vel - vel).length();
            let mut ke = r.round.energy(v_rel) / 1000.0;
            let mut stopped = false;
            let he = r.round.he > 0.0;
            let mut strikes: Vec<bdmg::Strike> = Vec::new();
            for (k, h) in hits.iter().enumerate() {
                if h.part == HitPart::Armour {
                    // ¼ in plate: stops rifle-calibre ball; HE bursts on it.
                    if he {
                        strikes.push(bdmg::Strike { part: HitPart::Armour, dmg: 0.0, he, incendiary: r.round.incendiary, pos: h.pos, normal: h.normal });
                        stopped = true;
                        break;
                    }
                    ke -= 3.5;
                    if ke <= 0.0 {
                        stopped = true;
                        break;
                    }
                    continue;
                }
                let detonate = he && (!r.delay || k >= 1 || hits.len() == 1);
                if detonate {
                    let blast = r.round.he * 4000.0;
                    strikes.push(bdmg::Strike { part: h.part, dmg: blast * vuln(h.part) + ke * 0.5, he: true, incendiary: r.round.incendiary, pos: h.pos, normal: h.normal });
                    for (p, dist) in u.hm.near(h.pos, 0.9) {
                        if p != h.part && p != HitPart::Armour {
                            strikes.push(bdmg::Strike { part: p, dmg: blast * 0.35 * (1.0 - dist / 0.9) * vuln(p), he: true, incendiary: false, pos: h.pos, normal: h.normal });
                        }
                    }
                    stopped = true;
                    break;
                }
                strikes.push(bdmg::Strike { part: h.part, dmg: ke * 0.5 * vuln(h.part), he: false, incendiary: r.round.incendiary, pos: h.pos, normal: h.normal });
                ke -= absorb(h.part, h.depth);
                if ke <= 0.2 {
                    stopped = true;
                    break;
                }
            }
            let world_pos = pos + rot * hits[0].pos;
            let world_n = rot * hits[0].normal;
            self.fx.push(Fx::Hit { pos: world_pos, normal: world_n, vel, he, cal: r.round.calibre });
            let total: f32 = strikes.iter().map(|s| s.dmg).sum();
            self.audio.push(crate::audio::params::Event::Hit { pos: world_pos, energy: (total / 60.0).min(1.0), own: Some(u.id) == self.local });
            let target = u.id;
            self.report_hits(i, target, r.shooter, r.weapon, &strikes, player);
            return stopped;
        }
        false
    }

    /// Does this simulation resolve `shooter`'s hits? Local / server: always;
    /// a multiplayer client: only its own (the others' are visual).
    pub fn resolves(&self, shooter: Id) -> bool {
        self.rules.authority != Authority::Client || Some(shooter) == self.local
    }

    /// Hits found on unit `i`: applied here when this simulation owns the
    /// target's damage, else sent as one `Hit` claim per strike (a
    /// multiplayer client hitting another player: the server validates the
    /// claim and the target's client applies it).
    fn report_hits(&mut self, i: usize, target: Id, shooter: Id, weapon: WeaponKind, strikes: &[bdmg::Strike], player: &mut Option<(&mut Aircraft, &AircraftDef)>) {
        let owns = self.rules.authority != Authority::Client || Some(target) == self.local;
        if owns {
            let total: f32 = strikes.iter().map(|s| s.dmg).sum();
            if let Some(s0) = strikes.first() {
                self.outbox.push(CombatEvent::Hit { target, shooter, weapon, part: s0.part as u8, pos: s0.pos, normal: s0.normal, damage: total });
            }
            self.apply_strikes(i, strikes, shooter, weapon, player);
        } else {
            for s in strikes.iter().filter(|s| s.dmg > 0.0) {
                self.outbox.push(CombatEvent::Hit { target, shooter, weapon, part: s.part as u8, pos: s.pos, normal: s.normal, damage: s.dmg.min(4000.0) });
            }
            if let Some(u) = self.units.get_mut(i) {
                u.hit_flash = 0.0;
                u.last_hit = Some((shooter, weapon, self.time));
            }
        }
    }

    /// Remove a combatant (a network player left).
    pub fn remove_unit(&mut self, id: Id) {
        self.units.retain(|u| u.id != id);
        self.rounds.retain(|r| r.shooter != id);
    }

    /// Apply strikes to unit `i` (authoritative path).
    fn apply_strikes(&mut self, i: usize, strikes: &[bdmg::Strike], shooter: Id, weapon: WeaponKind, player: &mut Option<(&mut Aircraft, &AircraftDef)>) {
        let def = self.defs[self.units[i].def].clone();
        let mut rolls: Vec<f32> = (0..strikes.len() * 6 + 4).map(|_| self.rnd()).collect();
        let mut roll = move || rolls.pop().unwrap_or(0.5);
        let time = self.time;
        let u = &mut self.units[i];
        let ac: &mut Aircraft = match (&mut u.ai, player.as_mut()) {
            (Some(a), _) => &mut a.ac,
            (None, Some(p)) => p.0,
            _ => return,
        };
        let mut outcomes = Vec::new();
        for s in strikes {
            if s.dmg <= 0.0 && s.part != HitPart::Armour {
                continue;
            }
            outcomes.extend(u.bd.strike(&def, ac, s, &mut roll));
        }
        u.last_hit = Some((shooter, weapon, time));
        u.hit_flash = 0.0;
        let lost = ac.dmg.lost;
        let failures: Vec<(u8, u8)> = u.bd.failures.iter().map(|f| (crate::systems::failures::Kind::ALL.iter().position(|k| *k == f.kind).unwrap_or(0) as u8, f.idx)).collect();
        let hp: Vec<u8> = u.bd.hp.iter().zip(u.bd.max.iter()).map(|(h, m)| (h / m.max(1e-3) * 255.0).round().clamp(0.0, 255.0) as u8).collect();
        let (id, fire, leak) = (u.id, u.bd.fire, u.bd.leak);
        let local = Some(id) == self.local;
        let callsign = u.callsign.clone();
        self.outbox.push(CombatEvent::Damage { target: id, hp, lost, failures, fire, leak });
        for o in outcomes {
            let msg = match &o {
                bdmg::Outcome::Failure(f) => Some(crate::systems::failures::Kind::label(f.kind, &crate::systems::defs::sys_def(&def), &def, f.idx as usize)),
                bdmg::Outcome::Separated(p) => Some(format!("{} shot away", cap(p.name()))),
                bdmg::Outcome::Fire(p) => Some(format!("Fire — {}", p.name())),
                bdmg::Outcome::PilotWounded => Some(if local { "You're hit — wounded".into() } else { format!("{callsign}'s pilot is hit") }),
                bdmg::Outcome::PilotKilled => Some(if local { "Pilot killed".into() } else { format!("{callsign}'s pilot is dead") }),
                bdmg::Outcome::Explosion => Some("Explosion!".into()),
                bdmg::Outcome::GunsJammed => Some("Guns jammed".into()),
                bdmg::Outcome::Glycol => Some("Glycol leak — engine will overheat".into()),
            };
            if let Some(m) = msg {
                if local {
                    self.notes.push(m);
                }
            }
            if let bdmg::Outcome::Explosion = o {
                let p = match (&self.units[i].ai, player.as_ref()) {
                    (Some(a), _) => a.ac.pos,
                    (None, Some(p)) => p.0.pos,
                    _ => Vec3::ZERO,
                };
                self.fx.push(Fx::Burst { pos: p, vel: Vec3::ZERO, energy: 0.6 });
                self.audio.push(crate::audio::params::Event::Explosion { pos: p, energy: 0.5 });
            }
        }
    }

    fn fly_missiles(&mut self, player: &mut Option<(&mut Aircraft, &AircraftDef)>, env: &dyn Environment, dt: f32) {
        let views = self.views(player.as_ref().map(|p| &*p.0));
        let mut ms = std::mem::take(&mut self.missiles);
        let mut ended: Vec<(u32, MissileEndKind, Vec3, Option<(Id, Vec3)>)> = Vec::new();
        let sun = self.sun;
        let mut rolls: Vec<f32> = (0..64).map(|_| self.rnd()).collect();
        for m in &mut ms {
            // Datalink from the launcher's radar.
            if m.datalink && !m.active {
                let link = m.target.and_then(|tg| self.unit(m.owner).filter(|o| o.alive()).and_then(|o| o.radar.track(tg)).filter(|t| t.age < 2.5)).map(|t| (t.pos, t.vel));
                match link {
                    Some((p, v)) => m.uplink(p, v),
                    None => m.datalink = false,
                }
            }
            let contacts = match m.def.seeker {
                missiles::SeekerKind::Ir { .. } => self.ir_contacts(&views, m.pos, m.owner),
                missiles::SeekerKind::Radar { .. } => self.radar_contacts(&views),
            };
            let mut r = || rolls.pop().unwrap_or(0.5);
            m.step(&contacts, sun, &mut r, dt);
            // Fuze: anyone (but the launcher just after launch) within the fuze radius.
            if m.t > m.def.arm {
                let mut best: Option<(Id, f32, f32)> = None;
                for (u, v) in self.units.iter().zip(views.iter()) {
                    if !u.flying() || (u.id == m.owner && m.t < 4.0) {
                        continue;
                    }
                    let prev = if u.prev_pos.distance(v.pos) > 200.0 { v.pos } else { u.prev_pos };
                    let (d, f) = missiles::closest_approach(m.prev_pos, m.pos, prev, v.pos);
                    if d < m.def.fuze && best.is_none_or(|b| d < b.1) {
                        best = Some((u.id, d, f));
                    }
                }
                if let Some((id, _, f)) = best {
                    let burst = m.prev_pos.lerp(m.pos, f);
                    ended.push((m.id, MissileEndKind::Detonated, burst, Some((id, m.vel))));
                    continue;
                }
                // Tracking a flare / chaff at close range: bursts on it.
                if let Some(Source::Flare(_) | Source::Chaff(_)) = m.tracking {
                    if m.est_pos.distance(m.pos) < m.def.fuze {
                        ended.push((m.id, MissileEndKind::Decoyed, m.pos, None));
                        continue;
                    }
                }
            }
            if m.pos.y <= env.ground_height(Vec2::new(m.pos.x, m.pos.z)) {
                ended.push((m.id, MissileEndKind::Ground, m.pos, None));
                continue;
            }
            if let Some(tg) = m.target.and_then(|t| views.iter().find(|v| v.id == t)) {
                let r = tg.pos.distance(m.pos);
                m.min_range = m.min_range.min(r);
                if m.min_range < 400.0 && r > m.min_range + 600.0 {
                    ended.push((m.id, MissileEndKind::Missed, m.pos, None));
                    continue;
                }
            }
            if m.spent() {
                ended.push((m.id, MissileEndKind::Missed, m.pos, None));
            }
        }
        for (id, kind, pos, victim) in &ended {
            let Some(k) = ms.iter().position(|m| m.id == *id) else { continue };
            let m = ms.remove(k);
            self.outbox.push(CombatEvent::MissileEnd { id: *id, kind: *kind, pos: *pos });
            // Warhead (self-destruct too: the fuze fires at the end of flight).
            self.fx.push(Fx::Burst { pos: *pos, vel: m.vel * 0.3, energy: m.def.warhead / 25.0 });
            let local_l = listener_dist(self, player, *pos);
            if local_l < 20_000.0 {
                self.audio.push(crate::audio::params::Event::Explosion { pos: *pos, energy: (m.def.warhead / 30.0).min(0.8) });
            }
            if *kind == MissileEndKind::Decoyed && Some(m.owner) == self.local {
                self.notes.push(format!("{} decoyed", m.def.name));
            }
            if *kind == MissileEndKind::Detonated && self.resolves(m.owner) && self.rules.enabled {
                let _ = victim;
                self.blast(&m, *pos, player);
            }
        }
        self.missiles = ms;
    }

    /// Fragments and blast on everyone near a burst.
    fn blast(&mut self, m: &Missile, burst: Vec3, player: &mut Option<(&mut Aircraft, &AircraftDef)>) {
        let axis = m.dir();
        for i in 0..self.units.len() {
            let u = &self.units[i];
            if !u.flying() {
                continue;
            }
            let (pos, rot, vel) = match (&u.ai, player.as_ref()) {
                (Some(a), _) => (a.ac.pos, a.ac.rot, a.ac.vel),
                (None, Some(p)) => (p.0.pos, p.0.rot, p.0.vel),
                _ => continue,
            };
            if pos.distance(burst) > 60.0 {
                continue;
            }
            let hm = u.hm.clone();
            let id = u.id;
            let fr = missiles::fragment_hits(m.def, burst, axis, m.vel, &hm, pos, rot, vel);
            let mut strikes = Vec::new();
            for (part, n, per, at) in fr {
                // Poisson-ish: the expected count, randomised ±.
                let k = (n * (0.6 + 0.8 * self.rnd())).round();
                if k < 1.0 && self.rnd() > n {
                    continue;
                }
                let k = k.max(1.0);
                strikes.push(bdmg::Strike {
                    part,
                    dmg: k * per * vuln(part).sqrt(),
                    he: true,
                    incendiary: true,
                    pos: at,
                    normal: (at - (rot.inverse() * (burst - pos))).normalize_or(Vec3::Y),
                });
            }
            if strikes.is_empty() {
                continue;
            }
            self.report_hits(i, id, m.owner, m.def.kind.weapon(), &strikes, player);
        }
    }

    /// Doomed / dead / ejected bookkeeping, AI wrecks, kill credit.
    fn life_cycle(&mut self, player: &mut Option<(&mut Aircraft, &AircraftDef)>, env: &dyn Environment, dt: f32) {
        let t = self.time;
        let mut rolls: Vec<f32> = (0..32).map(|_| self.rnd()).collect();
        let mut kills: Vec<(Id, Option<(Id, WeaponKind)>, String)> = Vec::new();
        let mut ejects: Vec<(Id, Vec3, Vec3, Quat, bool, Vec3)> = Vec::new();
        for i in 0..self.units.len() {
            if self.units[i].ai.as_ref().is_some_and(|a| a.external) {
                // (Its own client runs its damage and reports it.)
                continue;
            }
            let def = self.defs[self.units[i].def].clone();
            let u = &mut self.units[i];
            let local = Some(u.id) == self.local;
            let ac: &mut Aircraft = match (&mut u.ai, player.as_mut()) {
                (Some(a), _) => &mut a.ac,
                (None, Some(p)) => p.0,
                _ => continue,
            };
            let mut roll = || rolls.pop().unwrap_or(0.5);
            let outs = u.bd.update(&def, ac, dt, &mut roll);
            for o in outs {
                match o {
                    bdmg::Outcome::Explosion => {
                        self.fx.push(Fx::Burst { pos: ac.pos, vel: ac.vel, energy: 1.0 });
                        self.audio.push(crate::audio::params::Event::Explosion { pos: ac.pos, energy: 0.9 });
                    }
                    bdmg::Outcome::Failure(f) if local => {
                        self.notes.push(crate::systems::failures::Kind::label(f.kind, &crate::systems::defs::sys_def(&def), &def, f.idx as usize))
                    }
                    bdmg::Outcome::Separated(p) if local => self.notes.push(format!("{} gone", cap(p.name()))),
                    _ => {}
                }
            }
            let recent = u.last_hit.filter(|h| t - h.2 < 25.0).map(|h| (h.0, h.1));
            let broken = ac.dmg.failure.clone().or_else(|| u.bd.doomed.clone());
            match u.life {
                Life::Alive => {
                    if let Some(why) = broken {
                        u.life = Life::Doomed(0.0);
                        let by = recent;
                        kills.push((u.id, by, why));
                    } else if let Some(why) = ac.crashed.clone() {
                        u.life = Life::Dead(0.0);
                        let by = recent.or(Some((u.id, WeaponKind::Terrain))).filter(|b| b.0 != u.id || recent.is_none());
                        kills.push((u.id, by.filter(|b| b.0 != u.id), why));
                    }
                }
                Life::Doomed(s) => {
                    u.life = if ac.crashed.is_some() { Life::Dead(0.0) } else { Life::Doomed(s + dt) };
                }
                Life::Ejected(s) => {
                    u.life = Life::Ejected(s + dt);
                    if ac.crashed.is_some() && s > 0.0 {
                        u.life = Life::Dead(0.0);
                    }
                }
                Life::Dead(s) => u.life = Life::Dead(s + dt),
            }
            // Ejection / bail out (the pilot's choice; the sim asks for it twice).
            if std::mem::take(&mut u.trig.eject) && u.bd.pilot != bdmg::Pilot::Dead && !matches!(u.life, Life::Ejected(_) | Life::Dead(_)) && ac.crashed.is_none() {
                let was_alive = u.life == Life::Alive;
                u.life = Life::Ejected(0.0);
                ejects.push((u.id, ac.pos, ac.vel, ac.rot, u.load.seat, def.eye));
                if was_alive {
                    kills.push((u.id, u.last_hit.filter(|h| t - h.2 < 25.0).map(|h| (h.0, h.1)), if u.load.seat { "ejected".into() } else { "bailed out".into() }));
                }
                // Hands off: the aircraft flies on alone.
                if let Some(a) = &mut u.ai {
                    a.ctl.throttle = 0.0;
                    a.ctl.aileron = 0.15;
                }
            }
            // AI wreck: loose parts and the crash become debris.
            if let Some(a) = &mut u.ai {
                let lost = a.ac.dmg.lost;
                let new = lost & !a.known_lost;
                if new != 0 && a.ac.crashed.is_none() {
                    let geom = crate::debris::PartGeom::estimate(&def, crate::damage::struct_def(def.id));
                    let d = a.debris.get_or_insert_with(|| crate::debris::Debris::new(&def, geom));
                    d.separate_in_flight(new, a.ac.pos, a.ac.rot, a.ac.vel, a.ac.omega);
                    a.known_lost = lost;
                }
                if let (Some(reason), Some(imp)) = (a.ac.crashed.clone(), a.ac.impact) {
                    let geom = crate::debris::PartGeom::estimate(&def, crate::damage::struct_def(def.id));
                    let d = a.debris.get_or_insert_with(|| crate::debris::Debris::new(&def, geom));
                    if d.main.is_none() {
                        let sev = crate::damage::classify(&def, &reason, imp.vel, true);
                        let all = d.geom_has() | Part::Fuselage.bit();
                        d.crash(&imp, &reason, sev, all & !a.known_lost, a.ac.gear_pos(), env);
                    }
                }
                if let Some(d) = &mut a.debris {
                    d.update(dt, env, &|_| false);
                    self.audio.append(&mut d.audio);
                    self.debris_fx.append(&mut d.fx);
                    if let Some(b) = d.main.and_then(|m| d.bodies.get(m)) {
                        let (o, r) = b.origin();
                        a.ac.pos = o;
                        a.ac.rot = r;
                        a.ac.vel = b.vel;
                    }
                }
            }
        }
        for (victim, by, why) in kills {
            self.credit(victim, by, &why);
        }
        for (who, pos, vel, rot, seat, eye) in ejects {
            let (c, can) = eject::Chute::new(who, seat, pos, vel, rot, eye);
            self.chutes.push(c);
            self.canopies.push(can);
            self.audio.push(crate::audio::params::Event::Eject { pos, own: Some(who) == self.local, seat });
            self.outbox.push(CombatEvent::Eject { who, pos, vel, rot });
            // (Ejecting from a flyable aircraft is already in the feed as a loss.)
            if !self.feed.last().is_some_and(|f| f.t == t && (f.text.contains("ejected") || f.text.contains("bailed out"))) {
                let name = self.unit(who).map(|u| u.callsign.clone()).unwrap_or_default();
                self.feed.push(FeedLine { t, text: format!("{name} {}", if seat { "ejected" } else { "bailed out" }), mine: Some(who) == self.local });
            }
        }
    }

    fn credit(&mut self, victim: Id, by: Option<(Id, WeaponKind)>, why: &str) {
        let t = self.time;
        let vname = self.unit(victim).map(|u| u.callsign.clone()).unwrap_or_default();
        if let Some(u) = self.unit_mut(victim) {
            u.deaths += 1;
        }
        let (text, killer, weapon) = match by {
            Some((k, w)) if k != victim => {
                let kname = self.unit(k).map(|u| u.callsign.clone()).unwrap_or_default();
                if let Some(u) = self.unit_mut(k) {
                    u.kills += 1;
                }
                (format!("{kname} ▸ {vname}  ({})", w.label()), Some(k), w)
            }
            _ => (format!("{vname} — {}", why.trim_start_matches("Structural failure: ").trim_start_matches("Shot down: ")), None, WeaponKind::Terrain),
        };
        let mine = Some(victim) == self.local || killer.is_some() && killer == self.local;
        if killer.is_some() && killer == self.local {
            self.notes.push(format!("Splash! {vname} destroyed"));
        }
        self.feed.push(FeedLine { t, text, mine });
        if self.feed.len() > 30 {
            self.feed.remove(0);
        }
        self.outbox.push(CombatEvent::Kill { victim, killer, weapon });
    }

    /// Apply an authoritative event from the server (clients), or replay one.
    pub fn apply(&mut self, e: &CombatEvent, player: Option<(&mut Aircraft, &AircraftDef)>) {
        let mut player = player;
        match e {
            CombatEvent::Damage { target, hp, lost, failures, fire, leak } => {
                let Some(i) = self.units.iter().position(|u| u.id == *target) else { return };
                let def = self.defs[self.units[i].def].clone();
                let u = &mut self.units[i];
                let ac: &mut Aircraft = match (&mut u.ai, player.as_mut()) {
                    (Some(a), _) => &mut a.ac,
                    (None, Some(p)) => p.0,
                    _ => return,
                };
                for (k, q) in hp.iter().enumerate().take(u.bd.hp.len()) {
                    u.bd.hp[k] = u.bd.max[k] * *q as f32 / 255.0;
                }
                let sd = crate::damage::struct_def(def.id);
                for p in Part::ALL {
                    if lost & p.bit() != 0 && !ac.dmg.is_lost(p) {
                        ac.dmg.separate(sd, &def, p, "shot away");
                    }
                }
                for &(k, idx) in failures {
                    if let Some(kind) = crate::systems::failures::Kind::ALL.get(k as usize) {
                        let f = crate::systems::failures::Failure::new(*kind, idx as usize);
                        if !u.bd.failures.contains(&f) {
                            crate::systems::set_failure(ac, &def, f, true);
                            u.bd.failures.push(f);
                        }
                    }
                }
                u.bd.fire = *fire;
                u.bd.leak = *leak;
                u.hit_flash = 0.0;
            }
            CombatEvent::Kill { victim, killer, weapon } => {
                if let Some(u) = self.unit_mut(*victim) {
                    if u.alive() {
                        u.life = Life::Doomed(0.0);
                    }
                }
                self.credit(*victim, killer.map(|k| (k, *weapon)), "shot down");
            }
            CombatEvent::Rules { combat, friendly_fire } => {
                self.rules.enabled = *combat;
                self.rules.friendly_fire = *friendly_fire;
            }
            CombatEvent::MissileEnd { id, pos, .. } => {
                if let Some(k) = self.missiles.iter().position(|m| m.id == *id) {
                    let m = self.missiles.remove(k);
                    self.fx.push(Fx::Burst { pos: *pos, vel: m.vel * 0.3, energy: m.def.warhead / 25.0 });
                }
            }
            CombatEvent::MissileState { id, pos, vel, target, active } => {
                if let Some(m) = self.missiles.iter_mut().find(|m| m.id == *id) {
                    m.pos = *pos;
                    m.vel = *vel;
                    m.target = *target;
                    m.active = *active;
                }
            }
            CombatEvent::MissileLaunch { id, shooter, kind, target, pos, vel } => {
                if self.missiles.iter().any(|m| m.id == *id) {
                    return;
                }
                let k = if *kind == WeaponKind::Aim120 { MissileKind::Aim120 } else { MissileKind::Aim9 };
                self.missiles.push(Missile::new(*id, missiles::def_of(k), *shooter, *target, *pos, *vel, vel.normalize_or(Vec3::Z), None));
            }
            CombatEvent::Fire { shooter, gun, rounds, seed, .. } => {
                // Regenerate the rounds for the tracers (visual only).
                let Some(i) = self.units.iter().position(|u| u.id == *shooter) else { return };
                let u = &self.units[i];
                let Some(g) = u.load.guns.get(*gun as usize).cloned() else { return };
                let (pos, vel, rot) = match (&u.ai, player.as_ref()) {
                    (Some(a), _) => (a.ac.pos, a.ac.vel, a.ac.rot),
                    (None, Some(p)) => (p.0.pos, p.0.vel, p.0.rot),
                    _ => return,
                };
                let mut rng = *seed;
                let dir0 = rot * u.gun_dirs.get(*gun as usize).copied().unwrap_or(Vec3::Z);
                for j in 0..*rounds {
                    let (u1, u2) = (xorshift(&mut rng), xorshift(&mut rng));
                    let dir = guns::disperse(dir0, g.sigma, u1, u2);
                    let _ = xorshift(&mut rng);
                    let p = pos + rot * g.muzzle;
                    self.rounds.push(Projectile {
                        shooter: *shooter,
                        weapon: g.kind,
                        round: g.round,
                        pos: p,
                        vel: vel + dir * g.round.muzzle_v,
                        prev: p,
                        t: 0.0,
                        tracer: g.tracer_every > 0 && (j as u32).is_multiple_of(g.tracer_every),
                        delay: false,
                        cracked: false,
                        visual: true,
                    });
                }
            }
            CombatEvent::Countermeasure { owner, flare, pos, vel, .. } => {
                let id = self.next_id;
                self.next_id += 1;
                self.decoys.push(Decoy { id, owner: *owner, flare: *flare, pos: *pos, vel: *vel, t: 0.0 });
            }
            CombatEvent::Eject { who, pos, vel, rot } => {
                let (seat, eye) = self.unit(*who).map(|u| (u.load.seat, self.defs[u.def].eye)).unwrap_or((true, Vec3::ZERO));
                let (c, can) = eject::Chute::new(*who, seat, *pos, *vel, *rot, eye);
                self.chutes.push(c);
                self.canopies.push(can);
                if let Some(u) = self.unit_mut(*who) {
                    u.life = Life::Ejected(0.0);
                }
            }
            CombatEvent::Hit { target, shooter, weapon, part, pos, normal, damage } => {
                // A validated hit from another player: applied when it is ours,
                // shown when it is someone else's.
                let Some(i) = self.units.iter().position(|u| u.id == *target) else { return };
                let pose = match (&self.units[i].ai, player.as_ref()) {
                    (Some(a), _) => Some((a.ac.pos, a.ac.rot, a.ac.vel)),
                    (None, Some(p)) => Some((p.0.pos, p.0.rot, p.0.vel)),
                    _ => None,
                };
                let he = matches!(weapon, WeaponKind::Hispano | WeaponKind::M61 | WeaponKind::Aim9 | WeaponKind::Aim120);
                if let Some((p, r, v)) = pose {
                    self.fx.push(Fx::Hit { pos: p + r * *pos, normal: r * *normal, vel: v, he, cal: 0.02 });
                    self.audio.push(crate::audio::params::Event::Hit { pos: p + r * *pos, energy: (*damage / 60.0).min(1.0), own: Some(*target) == self.local });
                }
                if Some(*target) == self.local && self.rules.enabled {
                    let part = hitbox::HitPart::from_idx(*part as usize);
                    let s = bdmg::Strike { part, dmg: damage.clamp(0.0, 4000.0), he, incendiary: true, pos: *pos, normal: *normal };
                    self.apply_strikes(i, &[s], *shooter, *weapon, &mut player);
                }
            }
            CombatEvent::Emit { .. } => {}
        }
    }

    /// The local pilot can no longer fly (dead / out of the aircraft).
    pub fn local_incapacitated(&self) -> bool {
        self.local_unit().is_some_and(|u| u.bd.pilot == bdmg::Pilot::Dead || matches!(u.life, Life::Ejected(_)))
    }

    /// The local pilot's parachute (camera target after ejecting).
    pub fn local_chute(&self) -> Option<&eject::Chute> {
        let id = self.local?;
        self.chutes.iter().rev().find(|c| c.owner == id)
    }

    /// Current emissions (for the net layer's RWR replication / debugging).
    pub fn emissions(&self) -> &[(Id, Id, Emission)] {
        &self.emits
    }
}

/// Damage multipliers: how vulnerable a part is to a given amount of energy.
fn vuln(p: HitPart) -> f32 {
    match p {
        HitPart::Pilot => 20.0,
        HitPart::Radiator => 5.0,
        HitPart::Engine => 2.0,
        HitPart::FuelFuselage | HitPart::FuelWingL | HitPart::FuelWingR => 2.0,
        HitPart::Hydraulics | HitPart::Electrics => 3.0,
        HitPart::Ammo => 2.0,
        HitPart::AileronL | HitPart::AileronR | HitPart::Elevator | HitPart::Rudder => 1.5,
        _ => 1.0,
    }
}

/// Energy (kJ) a round loses passing through a part.
fn absorb(p: HitPart, depth: f32) -> f32 {
    match p {
        HitPart::Engine => 5.0 + depth * 2.0,
        HitPart::Armour => 3.5,
        HitPart::Pilot => 1.5,
        _ if p.is_structure() => 0.35 + depth * 0.25,
        _ => 0.8 + depth * 0.6,
    }
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

pub fn xorshift(s: &mut u32) -> f32 {
    let mut x = (*s).max(1);
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    (x >> 8) as f32 / 16_777_216.0
}

fn listener_dist(c: &Combat, player: &Option<(&mut Aircraft, &AircraftDef)>, p: Vec3) -> f32 {
    let l = c.local.and_then(|id| c.unit(id)).and_then(|u| match &u.ai {
        Some(a) => Some(a.ac.pos),
        None => player.as_ref().map(|q| q.0.pos),
    });
    l.map(|l| l.distance(p)).unwrap_or(0.0)
}
