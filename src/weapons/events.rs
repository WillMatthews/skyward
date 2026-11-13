//! Combat events: everything that happens in a fight as explicit, plain
//! data with a compact, versioned binary encoding (little endian, one tag
//! byte per event) for the network layer (`src/net`, Track MP) and replays.
//!
//! Authority (see `docs/WEAPONS.md`): clients send what their pilot does
//! (`Fire`, `MissileLaunch`, `Countermeasure`, `Eject`); the server runs the
//! same [`super::Combat`] simulation for every aircraft and is the only one
//! that resolves hits — it broadcasts `Hit`, `Damage`, `MissileEnd` and
//! `Kill`, which clients apply with [`super::Combat::apply`]. Visual-only
//! rounds on clients are regenerated from `Fire { seed }` so tracers match
//! without sending every bullet.

use glam::{Quat, Vec3};

/// Combatant id (0 = the local player in single-player; network ids in multiplayer).
pub type Id = u32;

/// Wire format version (bump on any layout change).
pub const VERSION: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WeaponKind {
    Hispano = 0,
    Browning303 = 1,
    M61 = 2,
    Aim9 = 3,
    Aim120 = 4,
    Collision = 5,
    Terrain = 6,
    Fire = 7,
    Mg50 = 8,
}

impl WeaponKind {
    pub fn from_u8(v: u8) -> Option<WeaponKind> {
        Some(match v {
            0 => WeaponKind::Hispano,
            1 => WeaponKind::Browning303,
            2 => WeaponKind::M61,
            3 => WeaponKind::Aim9,
            4 => WeaponKind::Aim120,
            5 => WeaponKind::Collision,
            6 => WeaponKind::Terrain,
            7 => WeaponKind::Fire,
            8 => WeaponKind::Mg50,
            _ => return None,
        })
    }
    pub fn label(self) -> &'static str {
        match self {
            WeaponKind::Hispano => "20 mm",
            WeaponKind::Browning303 => ".303",
            WeaponKind::M61 => "M61 20 mm",
            WeaponKind::Aim9 => "AIM-9",
            WeaponKind::Aim120 => "AIM-120",
            WeaponKind::Collision => "collision",
            WeaponKind::Terrain => "terrain",
            WeaponKind::Fire => "fire",
            WeaponKind::Mg50 => ".50",
        }
    }
}

/// Radar / missile emissions as an RWR sees them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Emission {
    Search = 0,
    Track = 1,
    Launch = 2,
    /// A missile's own seeker went active on the target.
    Active = 3,
}

/// Missile phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MissileEndKind {
    Detonated = 0,
    Missed = 1,
    Ground = 2,
    Decoyed = 3,
}

#[derive(Clone, Debug, PartialEq)]
pub enum CombatEvent {
    /// `rounds` fired by gun `gun` (index in the shooter's loadout) since the
    /// last event, from `t` (sim seconds); the dispersion RNG is `seed`.
    Fire {
        shooter: Id,
        gun: u8,
        rounds: u16,
        seed: u32,
        t: f64,
    },
    MissileLaunch {
        id: u32,
        shooter: Id,
        kind: WeaponKind,
        target: Option<Id>,
        pos: Vec3,
        vel: Vec3,
    },
    /// Periodic missile state (server → clients, ~10 Hz).
    MissileState {
        id: u32,
        pos: Vec3,
        vel: Vec3,
        target: Option<Id>,
        active: bool,
    },
    MissileEnd {
        id: u32,
        kind: MissileEndKind,
        pos: Vec3,
    },
    /// Flares (`flare` true) or chaff released.
    Countermeasure {
        owner: Id,
        flare: bool,
        count: u8,
        pos: Vec3,
        vel: Vec3,
    },
    /// A projectile / fragment hit (server-authoritative): body-frame point and normal.
    Hit {
        target: Id,
        shooter: Id,
        weapon: WeaponKind,
        part: u8,
        pos: Vec3,
        normal: Vec3,
        damage: f32,
    },
    /// Authoritative damage state after hits: hit points left per part
    /// (quantised 0..255 of max), separated structural parts, failures set
    /// (systems `Kind` index, idx), fire / leak levels.
    Damage {
        target: Id,
        hp: Vec<u8>,
        lost: u16,
        failures: Vec<(u8, u8)>,
        fire: f32,
        leak: f32,
    },
    Kill {
        victim: Id,
        killer: Option<Id>,
        weapon: WeaponKind,
    },
    Eject {
        who: Id,
        pos: Vec3,
        vel: Vec3,
        rot: Quat,
    },
    /// RWR-visible emission from `from` onto `to` (state changes only).
    Emit {
        from: Id,
        to: Id,
        kind: Emission,
        on: bool,
    },
    /// Server rule change: combat on / off.
    Rules {
        combat: bool,
        friendly_fire: bool,
    },
}

struct W(Vec<u8>);

impl W {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn v3(&mut self, v: Vec3) {
        self.f32(v.x);
        self.f32(v.y);
        self.f32(v.z);
    }
    fn opt(&mut self, v: Option<Id>) {
        self.u32(v.unwrap_or(u32::MAX));
    }
}

struct R<'a>(&'a [u8], usize);

impl R<'_> {
    fn take(&mut self, n: usize) -> Option<&[u8]> {
        let s = self.0.get(self.1..self.1 + n)?;
        self.1 += n;
        Some(s)
    }
    fn u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }
    fn u16(&mut self) -> Option<u16> {
        Some(u16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn f64(&mut self) -> Option<f64> {
        Some(f64::from_le_bytes(self.take(8)?.try_into().ok()?))
    }
    fn v3(&mut self) -> Option<Vec3> {
        Some(Vec3::new(self.f32()?, self.f32()?, self.f32()?))
    }
    fn opt(&mut self) -> Option<Option<Id>> {
        let v = self.u32()?;
        Some((v != u32::MAX).then_some(v))
    }
}

impl CombatEvent {
    /// Append the encoding of this event to `out`.
    pub fn encode(&self, out: &mut Vec<u8>) {
        let mut w = W(std::mem::take(out));
        match self {
            CombatEvent::Fire { shooter, gun, rounds, seed, t } => {
                w.u8(1);
                w.u32(*shooter);
                w.u8(*gun);
                w.u16(*rounds);
                w.u32(*seed);
                w.f64(*t);
            }
            CombatEvent::MissileLaunch { id, shooter, kind, target, pos, vel } => {
                w.u8(2);
                w.u32(*id);
                w.u32(*shooter);
                w.u8(*kind as u8);
                w.opt(*target);
                w.v3(*pos);
                w.v3(*vel);
            }
            CombatEvent::MissileState { id, pos, vel, target, active } => {
                w.u8(3);
                w.u32(*id);
                w.v3(*pos);
                w.v3(*vel);
                w.opt(*target);
                w.u8(*active as u8);
            }
            CombatEvent::MissileEnd { id, kind, pos } => {
                w.u8(4);
                w.u32(*id);
                w.u8(*kind as u8);
                w.v3(*pos);
            }
            CombatEvent::Countermeasure { owner, flare, count, pos, vel } => {
                w.u8(5);
                w.u32(*owner);
                w.u8(*flare as u8);
                w.u8(*count);
                w.v3(*pos);
                w.v3(*vel);
            }
            CombatEvent::Hit { target, shooter, weapon, part, pos, normal, damage } => {
                w.u8(6);
                w.u32(*target);
                w.u32(*shooter);
                w.u8(*weapon as u8);
                w.u8(*part);
                w.v3(*pos);
                w.v3(*normal);
                w.f32(*damage);
            }
            CombatEvent::Damage { target, hp, lost, failures, fire, leak } => {
                w.u8(7);
                w.u32(*target);
                w.u8(hp.len().min(255) as u8);
                for v in hp.iter().take(255) {
                    w.u8(*v);
                }
                w.u16(*lost);
                w.u8(failures.len().min(255) as u8);
                for (k, i) in failures.iter().take(255) {
                    w.u8(*k);
                    w.u8(*i);
                }
                w.f32(*fire);
                w.f32(*leak);
            }
            CombatEvent::Kill { victim, killer, weapon } => {
                w.u8(8);
                w.u32(*victim);
                w.opt(*killer);
                w.u8(*weapon as u8);
            }
            CombatEvent::Eject { who, pos, vel, rot } => {
                w.u8(9);
                w.u32(*who);
                w.v3(*pos);
                w.v3(*vel);
                for c in rot.to_array() {
                    w.f32(c);
                }
            }
            CombatEvent::Emit { from, to, kind, on } => {
                w.u8(10);
                w.u32(*from);
                w.u32(*to);
                w.u8(*kind as u8);
                w.u8(*on as u8);
            }
            CombatEvent::Rules { combat, friendly_fire } => {
                w.u8(11);
                w.u8(*combat as u8);
                w.u8(*friendly_fire as u8);
            }
        }
        *out = w.0;
    }

    /// Decode one event from the front of `buf`: (event, bytes used).
    pub fn decode(buf: &[u8]) -> Option<(CombatEvent, usize)> {
        let mut r = R(buf, 0);
        let e = match r.u8()? {
            1 => CombatEvent::Fire { shooter: r.u32()?, gun: r.u8()?, rounds: r.u16()?, seed: r.u32()?, t: r.f64()? },
            2 => CombatEvent::MissileLaunch { id: r.u32()?, shooter: r.u32()?, kind: WeaponKind::from_u8(r.u8()?)?, target: r.opt()?, pos: r.v3()?, vel: r.v3()? },
            3 => CombatEvent::MissileState { id: r.u32()?, pos: r.v3()?, vel: r.v3()?, target: r.opt()?, active: r.u8()? != 0 },
            4 => {
                let id = r.u32()?;
                let kind = match r.u8()? {
                    0 => MissileEndKind::Detonated,
                    1 => MissileEndKind::Missed,
                    2 => MissileEndKind::Ground,
                    3 => MissileEndKind::Decoyed,
                    _ => return None,
                };
                CombatEvent::MissileEnd { id, kind, pos: r.v3()? }
            }
            5 => CombatEvent::Countermeasure { owner: r.u32()?, flare: r.u8()? != 0, count: r.u8()?, pos: r.v3()?, vel: r.v3()? },
            6 => CombatEvent::Hit { target: r.u32()?, shooter: r.u32()?, weapon: WeaponKind::from_u8(r.u8()?)?, part: r.u8()?, pos: r.v3()?, normal: r.v3()?, damage: r.f32()? },
            7 => {
                let target = r.u32()?;
                let n = r.u8()? as usize;
                let mut hp = Vec::with_capacity(n);
                for _ in 0..n {
                    hp.push(r.u8()?);
                }
                let lost = r.u16()?;
                let m = r.u8()? as usize;
                let mut failures = Vec::with_capacity(m);
                for _ in 0..m {
                    failures.push((r.u8()?, r.u8()?));
                }
                CombatEvent::Damage { target, hp, lost, failures, fire: r.f32()?, leak: r.f32()? }
            }
            8 => CombatEvent::Kill { victim: r.u32()?, killer: r.opt()?, weapon: WeaponKind::from_u8(r.u8()?)? },
            9 => CombatEvent::Eject { who: r.u32()?, pos: r.v3()?, vel: r.v3()?, rot: Quat::from_array([r.f32()?, r.f32()?, r.f32()?, r.f32()?]) },
            10 => {
                let (from, to) = (r.u32()?, r.u32()?);
                let kind = match r.u8()? {
                    0 => Emission::Search,
                    1 => Emission::Track,
                    2 => Emission::Launch,
                    3 => Emission::Active,
                    _ => return None,
                };
                CombatEvent::Emit { from, to, kind, on: r.u8()? != 0 }
            }
            11 => CombatEvent::Rules { combat: r.u8()? != 0, friendly_fire: r.u8()? != 0 },
            _ => return None,
        };
        Some((e, r.1))
    }

    /// A batch: version byte, then events back to back.
    pub fn encode_batch(events: &[CombatEvent]) -> Vec<u8> {
        let mut out = vec![VERSION];
        for e in events {
            e.encode(&mut out);
        }
        out
    }
    pub fn decode_batch(buf: &[u8]) -> Option<Vec<CombatEvent>> {
        if buf.first() != Some(&VERSION) {
            return None;
        }
        let mut out = Vec::new();
        let mut i = 1;
        while i < buf.len() {
            let (e, n) = CombatEvent::decode(&buf[i..])?;
            out.push(e);
            i += n;
        }
        Some(out)
    }

    /// Events only the authority may originate (clients must not trust their own).
    pub fn authoritative(&self) -> bool {
        matches!(
            self,
            CombatEvent::Hit { .. }
                | CombatEvent::Damage { .. }
                | CombatEvent::Kill { .. }
                | CombatEvent::MissileEnd { .. }
                | CombatEvent::MissileState { .. }
                | CombatEvent::Rules { .. }
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_event_round_trips() {
        let v = Vec3::new(1.5, -2.25, 1000.125);
        let all = vec![
            CombatEvent::Fire { shooter: 3, gun: 1, rounds: 12, seed: 0xDEAD_BEEF, t: 123.456 },
            CombatEvent::MissileLaunch { id: 7, shooter: 0, kind: WeaponKind::Aim120, target: Some(4), pos: v, vel: -v },
            CombatEvent::MissileLaunch { id: 8, shooter: 0, kind: WeaponKind::Aim9, target: None, pos: v, vel: v },
            CombatEvent::MissileState { id: 7, pos: v, vel: v, target: None, active: true },
            CombatEvent::MissileEnd { id: 7, kind: MissileEndKind::Decoyed, pos: v },
            CombatEvent::Countermeasure { owner: 2, flare: true, count: 2, pos: v, vel: v },
            CombatEvent::Hit { target: 1, shooter: 0, weapon: WeaponKind::Hispano, part: 7, pos: v, normal: Vec3::X, damage: 52.5 },
            CombatEvent::Damage { target: 1, hp: vec![255, 0, 128], lost: 0b100, failures: vec![(0, 0), (5, 2)], fire: 0.5, leak: 0.25 },
            CombatEvent::Kill { victim: 1, killer: Some(0), weapon: WeaponKind::M61 },
            CombatEvent::Kill { victim: 1, killer: None, weapon: WeaponKind::Terrain },
            CombatEvent::Eject { who: 1, pos: v, vel: v, rot: Quat::from_rotation_y(0.3) },
            CombatEvent::Emit { from: 1, to: 0, kind: Emission::Track, on: true },
            CombatEvent::Rules { combat: false, friendly_fire: true },
        ];
        let buf = CombatEvent::encode_batch(&all);
        let back = CombatEvent::decode_batch(&buf).unwrap();
        assert_eq!(all, back);
        // Truncated input fails cleanly, never panics.
        for n in 0..buf.len() {
            let _ = CombatEvent::decode_batch(&buf[..n]);
        }
        assert!(CombatEvent::decode_batch(&[VERSION + 1]).is_none());
        assert!(all[6].authoritative() && !all[0].authoritative());
    }
}
