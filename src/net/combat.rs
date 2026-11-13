//! Air combat over the network (weapons/ ↔ `proto::CombatMsg`).
//!
//! Every other player is an `external` unit in the local [`Combat`] (its
//! pose written from the interpolated network state each frame), so rounds,
//! missile seekers and fuzes see them. This client is authoritative for
//! nothing but its own aircraft: it detects its own hits and sends them as
//! claims (the server validates them, the target applies them), applies the
//! hits others claim on it, and reports its own damage, kills and ejection.
//! Other players' rounds, missiles and countermeasures are regenerated
//! locally for the tracers, trails and seekers.

use crate::weapons::events::{CombatEvent, Id, WeaponKind};
use crate::weapons::{AiCraft, Authority, Combat, Life};
use glam::{Quat, Vec3};
use skyward_net::proto::{CombatMsg, SERVER_ID};
use std::collections::BTreeMap;

/// Network id ↔ combat unit id, and this client's missiles.
#[derive(Default)]
pub struct Link {
    pub units: BTreeMap<u16, Id>,
    own_missiles: Vec<u32>,
}

/// A remote missile id, kept apart from this client's own (each client numbers its own).
fn remote_missile(from: u16, id: u32) -> u32 {
    (id & 0x00FF_FFFF) | ((from as u32 % 255 + 1) << 24)
}

impl Link {
    fn net_of(&self, unit: Id) -> Option<u16> {
        self.units.iter().find(|(_, u)| **u == unit).map(|(n, _)| *n)
    }

    /// Forget every remote unit (left the server).
    pub fn clear(&mut self, c: &mut Combat) {
        for (_, u) in std::mem::take(&mut self.units) {
            c.remove_unit(u);
        }
        c.rules.authority = Authority::Local;
        c.rules.enabled = true;
    }

    /// Make the local fight match the server: rules, one external unit per
    /// remote player, their poses. `remotes`: (net id, catalogue index, pos, rot, vel, flying).
    pub fn sync(&mut self, c: &mut Combat, combat_on: bool, remotes: &[(u16, usize, Vec3, Quat, Vec3, bool)]) {
        c.rules.authority = Authority::Client;
        c.rules.enabled = combat_on;
        c.dogfight = None;
        // Departed / spectating players.
        let live: Vec<u16> = remotes.iter().filter(|r| r.5).map(|r| r.0).collect();
        let gone: Vec<u16> = self.units.keys().copied().filter(|n| !live.contains(n)).collect();
        for n in gone {
            if let Some(u) = self.units.remove(&n) {
                c.remove_unit(u);
            }
        }
        for &(n, def, pos, rot, vel, flying) in remotes {
            if !flying {
                continue;
            }
            // A changed type is a new unit (other hit volumes and loadout).
            if let Some(&u) = self.units.get(&n) {
                if c.unit(u).is_some_and(|x| x.def != def) {
                    c.remove_unit(u);
                    self.units.remove(&n);
                }
            }
            let id = match self.units.get(&n) {
                Some(&u) => u,
                None => {
                    let d = c.defs[def.min(c.defs.len() - 1)].clone();
                    let mut ac = crate::flight::Aircraft::airborne(&d, pos, 0.0, 60.0);
                    ac.rot = rot;
                    ac.vel = vel;
                    let ai = AiCraft {
                        prev_pos: pos,
                        prev_rot: rot,
                        ac,
                        ctl: Default::default(),
                        brain: crate::weapons::ai::Brain::new(crate::weapons::ai::Difficulty::Average, n as u32 + 1),
                        debris: None,
                        known_lost: 0,
                        passive: true,
                        external: true,
                    };
                    // Free for all: every player is their own side.
                    let u = c.add_unit(def.min(c.defs.len() - 1), 1 + (n % 250) as u8, "", Some(Box::new(ai)));
                    self.units.insert(n, u);
                    u
                }
            };
            if let Some(u) = c.unit_mut(id) {
                if let Some(a) = u.ai.as_mut() {
                    a.prev_pos = a.ac.pos;
                    a.prev_rot = a.ac.rot;
                    if a.ac.pos.distance(pos) > 500.0 {
                        a.prev_pos = pos;
                    }
                    a.ac.pos = pos;
                    a.ac.rot = rot;
                    a.ac.vel = vel;
                }
                u.prev_pos = u.ai.as_ref().map(|a| a.prev_pos).unwrap_or(pos);
                // A respawned player flies again.
                if !u.alive() && matches!(u.life, Life::Dead(s) if s > 3.0) {
                    c.rearm(id);
                }
            }
        }
    }

    /// Name the units (callsigns from the player list).
    pub fn name(&self, c: &mut Combat, names: &BTreeMap<u16, String>) {
        for (n, u) in &self.units {
            if let (Some(unit), Some(name)) = (c.unit_mut(*u), names.get(n)) {
                if unit.callsign != *name {
                    unit.callsign = name.clone();
                }
            }
        }
    }

    /// What this client tells the others: its own fire, launches, flares,
    /// hit claims, damage, kill and ejection (drains the outbox).
    pub fn outgoing(&mut self, c: &mut Combat, me: u16) -> Vec<CombatMsg> {
        let local = c.local;
        let mut out = Vec::new();
        for e in std::mem::take(&mut c.outbox) {
            match e {
                CombatEvent::Fire { shooter, gun, rounds, seed, .. } if Some(shooter) == local => out.push(CombatMsg::Fire { gun, rounds, seed }),
                CombatEvent::MissileLaunch { id, shooter, kind, target, .. } if Some(shooter) == local => {
                    self.own_missiles.push(id);
                    if self.own_missiles.len() > 64 {
                        self.own_missiles.remove(0);
                    }
                    let target = target.and_then(|t| self.net_of(t)).unwrap_or(SERVER_ID);
                    out.push(CombatMsg::Launch { id, kind: u8::from(kind == WeaponKind::Aim120), target });
                }
                CombatEvent::MissileEnd { id, .. } if self.own_missiles.contains(&id) => out.push(CombatMsg::MissileEnd { id }),
                CombatEvent::Countermeasure { owner, flare, .. } if Some(owner) == local => out.push(CombatMsg::Countermeasure { flare }),
                CombatEvent::Hit { target, shooter, weapon, part, pos, normal, damage } if Some(shooter) == local && Some(target) != local => {
                    if let Some(t) = self.net_of(target) {
                        let he = matches!(weapon, WeaponKind::Hispano | WeaponKind::M61 | WeaponKind::Aim9 | WeaponKind::Aim120);
                        out.push(CombatMsg::Hit { target: t, weapon: weapon as u8, part, pos: pos.to_array(), normal: normal.to_array(), damage, he });
                    }
                }
                CombatEvent::Damage { target, hp, lost, failures, fire, leak } if Some(target) == local => out.push(CombatMsg::Damage { hp, lost, failures, fire, leak }),
                CombatEvent::Kill { victim, killer, weapon } if Some(victim) == local => {
                    let killer = killer.and_then(|k| self.net_of(k)).unwrap_or(SERVER_ID);
                    out.push(CombatMsg::Kill { killer, weapon: weapon as u8 });
                }
                CombatEvent::Eject { who, .. } if Some(who) == local => out.push(CombatMsg::Eject),
                _ => {}
            }
        }
        let _ = me;
        // One claim per target / part / weapon per frame (bursts hit the same part many times).
        let mut merged: Vec<CombatMsg> = Vec::with_capacity(out.len());
        for m in out {
            if let CombatMsg::Hit { target, weapon, part, damage, .. } = m {
                if let Some(CombatMsg::Hit { damage: d, .. }) =
                    merged.iter_mut().find(|x| matches!(x, CombatMsg::Hit { target: t, weapon: w, part: p, .. } if *t == target && *w == weapon && *p == part))
                {
                    *d = (*d + damage).min(skyward_net::proto::MAX_HIT_DAMAGE);
                    continue;
                }
            }
            merged.push(m);
        }
        merged
    }

    /// Apply a combat message from player `from` (stamped by the server).
    pub fn incoming(&mut self, c: &mut Combat, me: u16, from: u16, m: CombatMsg, player: Option<(&mut crate::flight::Aircraft, &crate::flight::AircraftDef)>) {
        let Some(&unit) = self.units.get(&from) else { return };
        let pose = c.unit(unit).and_then(|u| u.ai.as_ref()).map(|a| (a.ac.pos, a.ac.vel, a.ac.rot));
        let (pos, vel, rot) = pose.unwrap_or((Vec3::ZERO, Vec3::ZERO, Quat::IDENTITY));
        let unit_of = |n: u16, s: &Link| if n == me { c.local } else { s.units.get(&n).copied() };
        let ev = match m {
            CombatMsg::Fire { gun, rounds, seed } => CombatEvent::Fire { shooter: unit, gun, rounds, seed, t: c.time },
            CombatMsg::Launch { id, kind, target } => CombatEvent::MissileLaunch {
                id: remote_missile(from, id),
                shooter: unit,
                kind: if kind == 1 { WeaponKind::Aim120 } else { WeaponKind::Aim9 },
                target: unit_of(target, self),
                pos: pos + rot * Vec3::new(0.0, -0.4, 0.0),
                vel,
            },
            CombatMsg::MissileEnd { id } => CombatEvent::MissileEnd { id: remote_missile(from, id), kind: crate::weapons::events::MissileEndKind::Detonated, pos },
            CombatMsg::Countermeasure { flare } => {
                CombatEvent::Countermeasure { owner: unit, flare, count: 1, pos: pos + rot * Vec3::new(0.0, -0.6, -3.5), vel: vel + rot * Vec3::new(0.0, -22.0, -4.0) }
            }
            CombatMsg::Hit { target, weapon, part, pos, normal, damage, .. } => {
                let Some(t) = unit_of(target, self) else { return };
                CombatEvent::Hit {
                    target: t,
                    shooter: unit,
                    weapon: WeaponKind::from_u8(weapon).unwrap_or(WeaponKind::M61),
                    part,
                    pos: Vec3::from(pos),
                    normal: Vec3::from(normal),
                    damage,
                }
            }
            CombatMsg::Damage { hp, lost, failures, fire, leak } => CombatEvent::Damage { target: unit, hp, lost, failures, fire, leak },
            CombatMsg::Kill { killer, weapon } => {
                CombatEvent::Kill { victim: unit, killer: unit_of(killer, self), weapon: WeaponKind::from_u8(weapon).unwrap_or(WeaponKind::Terrain) }
            }
            CombatMsg::Eject => CombatEvent::Eject { who: unit, pos, vel, rot },
        };
        c.apply(&ev, player);
        // (Applying a remote event must not echo it back.)
        c.outbox.retain(|e| match e {
            CombatEvent::Kill { victim, .. } => Some(*victim) == c.local,
            CombatEvent::Damage { target, .. } => Some(*target) == c.local,
            _ => true,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::weapons::Rules;

    /// Two clients and a hit: A's claim, relayed, damages B's own aircraft on
    /// B, and B's damage report reaches A's copy of B.
    #[test]
    fn hit_claims_round_trip_between_clients() {
        let defs = crate::flight::catalogue();
        let spit = defs.iter().position(|d| d.id == "peregrine").unwrap();
        let mk = || {
            let mut c = Combat::new(Rules::default());
            let id = c.add_unit(spit, 0, "me", None);
            c.local = Some(id);
            c
        };
        let (mut a, mut b) = (mk(), mk());
        let (mut la, mut lb) = (Link::default(), Link::default());
        let p = Vec3::new(0.0, 2000.0, 0.0);
        la.sync(&mut a, true, &[(2, spit, p, Quat::IDENTITY, Vec3::Z * 100.0, true)]);
        lb.sync(&mut b, true, &[(1, spit, p, Quat::IDENTITY, Vec3::Z * 100.0, true)]);
        // A "detects" a 20 mm hit on B's left wing.
        let tb = la.units[&2];
        a.outbox.push(CombatEvent::Hit {
            target: tb,
            shooter: a.local.unwrap(),
            weapon: WeaponKind::Hispano,
            part: crate::weapons::hitbox::HitPart::WingL as u8,
            pos: Vec3::X,
            normal: Vec3::Y,
            damage: 60.0,
        });
        let msgs = la.outgoing(&mut a, 1);
        assert_eq!(msgs.len(), 1);
        // Through the wire format.
        let wire: Vec<CombatMsg> = msgs
            .into_iter()
            .map(|m| {
                let e = skyward_net::proto::Msg::Combat { from: 1, msg: m };
                match skyward_net::proto::Msg::decode_body(e.type_id(), &e.encode_body()).unwrap() {
                    skyward_net::proto::Msg::Combat { msg, .. } => msg,
                    _ => unreachable!(),
                }
            })
            .collect();
        let def = defs[spit].clone();
        let mut ac = crate::flight::Aircraft::airborne(&def, p, 0.0, 100.0);
        ac.sys = crate::systems::Systems::new(&def, false);
        for m in wire {
            lb.incoming(&mut b, 2, 1, m, Some((&mut ac, &def)));
        }
        let wl = crate::weapons::hitbox::HitPart::WingL.idx();
        let bu = b.local_unit().unwrap();
        assert!(bu.bd.hp[wl] < bu.bd.max[wl], "B took the hit");
        assert!(ac.dmg.weaken[0] > 0.0);
        // B reports its damage; A applies it to its copy of B.
        let back = lb.outgoing(&mut b, 2);
        assert!(back.iter().any(|m| matches!(m, CombatMsg::Damage { .. })));
        for m in back {
            la.incoming(&mut a, 1, 2, m, None);
        }
        let ab = a.unit(tb).unwrap();
        assert!(ab.bd.hp[wl] < ab.bd.max[wl], "A sees B damaged");
        // Combat off: no rounds.
        la.sync(&mut a, false, &[(2, spit, p, Quat::IDENTITY, Vec3::Z * 100.0, true)]);
        assert!(!a.rules.enabled);
        // A leaves: units gone, back to single-player rules.
        la.clear(&mut a);
        assert_eq!(a.units.len(), 1);
        assert_eq!(a.rules.authority, Authority::Local);
    }
}
