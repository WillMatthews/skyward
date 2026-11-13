//! Sensors: a simple fire-control radar (search with track-while-scan,
//! single-target track), the radar warning receiver, and the AIM-9's
//! seeker on the rail (caged / slaved to the radar / uncaged and tracking,
//! with the seeker growl and lock tone).

use super::events::{Emission, Id};
use super::missiles::{Contact, Source};
use glam::{Quat, Vec3};

/// What sensors see of another aircraft.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub id: Id,
    pub team: u8,
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    /// Radar cross-section (m²).
    pub rcs: f32,
    pub alive: bool,
    pub has_rwr: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Track {
    pub id: Id,
    pub pos: Vec3,
    pub vel: Vec3,
    pub age: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadarMode {
    Off,
    /// Range-while-search / track-while-scan: periodic updates of everything in the volume.
    Search,
    /// Single-target track: continuous, the best datalink for the AIM-120.
    Stt,
}

#[derive(Clone, Debug)]
pub struct Radar {
    pub mode: RadarMode,
    pub tracks: Vec<Track>,
    pub stt: Option<Id>,
    pub scan: f32,
    pub notched: f32,
    /// Search volume half-angle (rad) and detection range for 5 m² (m).
    pub cone: f32,
    pub range: f32,
}

impl Default for Radar {
    fn default() -> Self {
        Radar { mode: RadarMode::Search, tracks: Vec::new(), stt: None, scan: 0.0, notched: 0.0, cone: 60f32.to_radians(), range: 70_000.0 }
    }
}

/// Ground-clutter notch: a target moving across the beam (radial speed
/// over the ground ≈ 0) while the radar looks down is filtered out.
pub fn notched(from: Vec3, to: Vec3, vel: Vec3) -> bool {
    let d = (to - from).normalize_or_zero();
    d.y < -0.03 && vel.dot(d).abs() < 25.0
}

impl Radar {
    fn sees(&self, me: &View, o: &View) -> bool {
        let d = o.pos - me.pos;
        let r = d.length();
        o.alive && o.id != me.id && r > 1.0 && r < self.range * (o.rcs / 5.0).powf(0.25) && (me.rot * Vec3::Z).angle_between(d) < self.cone && !notched(me.pos, o.pos, o.vel)
    }

    /// Update; returns the aircraft illuminated this step and how (for their RWRs).
    pub fn update(&mut self, me: &View, others: &[View], dt: f32) -> Vec<(Id, Emission)> {
        let mut em = Vec::new();
        for t in &mut self.tracks {
            t.age += dt;
            t.pos += t.vel * dt;
        }
        self.tracks.retain(|t| t.age < 8.0);
        if self.mode == RadarMode::Off {
            self.stt = None;
            return em;
        }
        if let Some(id) = self.stt {
            match others.iter().find(|o| o.id == id) {
                Some(o) if o.alive && (me.rot * Vec3::Z).angle_between(o.pos - me.pos) < self.cone && o.pos.distance(me.pos) < self.range * 1.3 => {
                    if notched(me.pos, o.pos, o.vel) {
                        self.notched += dt;
                    } else {
                        self.notched = 0.0;
                    }
                    if self.notched > 1.2 {
                        self.stt = None;
                        self.mode = RadarMode::Search;
                    } else {
                        self.upsert(o, true);
                        em.push((o.id, Emission::Track));
                    }
                }
                _ => {
                    self.stt = None;
                    self.mode = RadarMode::Search;
                }
            }
        }
        // Search bars: a frame every ~2 s (each target painted briefly).
        self.scan += dt;
        if self.scan >= 2.0 || self.mode == RadarMode::Search && self.tracks.is_empty() && self.scan > 0.5 {
            self.scan = 0.0;
            for o in others {
                if Some(o.id) != self.stt && self.sees(me, o) {
                    self.upsert(o, false);
                    em.push((o.id, Emission::Search));
                }
            }
        }
        em
    }

    fn upsert(&mut self, o: &View, _stt: bool) {
        match self.tracks.iter_mut().find(|t| t.id == o.id) {
            Some(t) => {
                t.pos = o.pos;
                t.vel = o.vel;
                t.age = 0.0;
            }
            None => self.tracks.push(Track { id: o.id, pos: o.pos, vel: o.vel, age: 0.0 }),
        }
    }

    /// Lock the target nearest the boresight (or the next one): STT.
    pub fn lock_next(&mut self, me: &View, others: &[View]) -> Option<Id> {
        let fwd = me.rot * Vec3::Z;
        let mut c: Vec<(&View, f32)> = others.iter().filter(|o| o.team != me.team && self.sees(me, o)).map(|o| (o, fwd.angle_between(o.pos - me.pos))).collect();
        c.sort_by(|a, b| a.1.total_cmp(&b.1));
        let next = match self.stt {
            Some(cur) => c.iter().position(|x| x.0.id == cur).map(|i| c[(i + 1) % c.len()].0.id).or(c.first().map(|x| x.0.id)),
            None => c.first().map(|x| x.0.id),
        };
        self.stt = next;
        self.notched = 0.0;
        self.mode = if next.is_some() { RadarMode::Stt } else { RadarMode::Search };
        if let Some(o) = next.and_then(|id| others.iter().find(|o| o.id == id)) {
            self.upsert(o, true);
        }
        next
    }
    pub fn unlock(&mut self) {
        self.stt = None;
        if self.mode == RadarMode::Stt {
            self.mode = RadarMode::Search;
        }
    }
    pub fn track(&self, id: Id) -> Option<&Track> {
        self.tracks.iter().find(|t| t.id == id)
    }
}

/// One RWR contact.
#[derive(Clone, Copy, Debug)]
pub struct RwrContact {
    pub from: Id,
    /// Azimuth relative to the nose (rad, + = right) and range (for the display ring).
    pub az: f32,
    pub range: f32,
    pub kind: Emission,
    /// Seconds since last painted, and since it first appeared.
    pub since: f32,
    pub age: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Rwr {
    pub contacts: Vec<RwrContact>,
    /// A new contact appeared (tone), seconds ago.
    pub new_t: f32,
}

impl Rwr {
    /// Update with this step's illuminations of `me`: (emitter id, position, kind).
    pub fn update(&mut self, me_pos: Vec3, me_rot: Quat, hits: &[(Id, Vec3, Emission)], dt: f32) {
        self.new_t += dt;
        for c in &mut self.contacts {
            c.since += dt;
            c.age += dt;
        }
        let inv = me_rot.inverse();
        for &(from, pos, kind) in hits {
            let d = inv * (pos - me_pos);
            // Body +X is the left wing: azimuth positive to the right.
            let az = (-d.x).atan2(d.z);
            let range = (pos - me_pos).length();
            match self.contacts.iter_mut().find(|c| c.from == from && (c.kind == kind || (kind != Emission::Active && c.kind != Emission::Active))) {
                Some(c) => {
                    if kind as u8 > c.kind as u8 || c.since > 2.5 {
                        c.kind = kind;
                    }
                    c.az = az;
                    c.range = range;
                    c.since = 0.0;
                }
                None => {
                    self.contacts.push(RwrContact { from, az, range, kind, since: 0.0, age: 0.0 });
                    self.new_t = 0.0;
                }
            }
        }
        // Search contacts persist a few scans; track / launch drop quickly when the paint stops.
        self.contacts.retain(|c| c.since < if c.kind == Emission::Search { 6.0 } else { 1.0 });
    }
    pub fn worst(&self) -> Option<Emission> {
        self.contacts.iter().map(|c| c.kind).max_by_key(|k| *k as u8)
    }
}

/// The AIM-9 seeker on the rail.
#[derive(Clone, Debug)]
pub struct RailSeeker {
    /// Line of sight (world unit vector).
    pub dir: Vec3,
    /// Tracking (uncaged) what.
    pub locked: Option<Source>,
    /// Signal strength 0..1 (the growl's loudness / pitch).
    pub signal: f32,
}

impl Default for RailSeeker {
    fn default() -> Self {
        RailSeeker { dir: Vec3::Z, locked: None, signal: 0.0 }
    }
}

impl RailSeeker {
    /// Caged: looks along the boresight (or the radar's STT line of sight).
    /// Uncaged: follows the source it tracks, up to the gimbal limit.
    pub fn update(&mut self, pos: Vec3, rot: Quat, slave: Option<Vec3>, contacts: &[Contact], sun: Vec3, gimbal: f32, fov: f32, r_ref: f32, dt: f32) {
        let fwd = rot * Vec3::Z;
        let strength = |c: &Contact| -> f32 {
            let r = c.pos.distance(pos).max(1.0);
            c.strength / (1500.0 * (r / r_ref).powi(2))
        };
        match self.locked {
            Some(src) => {
                let hit = contacts.iter().find(|c| c.src == src);
                match hit {
                    Some(c) if strength(c) >= 1.0 && (c.pos - pos).angle_between(fwd) < gimbal && !(sun.y > -0.05 && (c.pos - pos).angle_between(sun) < fov * 1.5) => {
                        self.dir = (c.pos - pos).normalize();
                        self.signal = (strength(c).log10() * 0.4 + 0.6).clamp(0.3, 1.0);
                    }
                    _ => {
                        self.locked = None;
                    }
                }
            }
            None => {
                let target = slave.unwrap_or(fwd);
                self.dir = (self.dir + (target - self.dir) * (dt * 12.0).min(1.0)).normalize_or(fwd);
                let best = contacts
                    .iter()
                    .filter(|c| !matches!(c.src, Source::Chaff(_)) && (c.pos - pos).angle_between(self.dir) < fov && strength(c) >= 1.0)
                    .map(strength)
                    .fold(0.0f32, f32::max);
                let sun_in = sun.y > -0.05 && self.dir.angle_between(sun) < fov * 1.5;
                self.signal = if sun_in {
                    1.0
                } else if best > 0.0 {
                    (best.log10() * 0.4 + 0.5).clamp(0.2, 1.0)
                } else {
                    0.0
                };
            }
        }
    }

    /// Uncage onto the strongest source in the field of view.
    pub fn uncage(&mut self, pos: Vec3, contacts: &[Contact], fov: f32, r_ref: f32) -> bool {
        let best = contacts
            .iter()
            .filter(|c| matches!(c.src, Source::Unit(_)) && (c.pos - pos).angle_between(self.dir) < fov * 1.4)
            .map(|c| (c.src, c.strength / (1500.0 * (c.pos.distance(pos).max(1.0) / r_ref).powi(2))))
            .filter(|x| x.1 >= 1.0)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        self.locked = best.map(|b| b.0);
        self.locked.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view(id: Id, team: u8, pos: Vec3, vel: Vec3) -> View {
        View { id, team, pos, vel, rot: Quat::IDENTITY, rcs: 5.0, alive: true, has_rwr: true }
    }

    #[test]
    fn radar_locks_paints_and_loses_in_the_notch() {
        let me = view(0, 0, Vec3::new(0.0, 6000.0, 0.0), Vec3::new(0.0, 0.0, 250.0));
        let mut tgt = view(1, 1, Vec3::new(0.0, 3000.0, 20_000.0), Vec3::new(0.0, 0.0, -250.0));
        let mut r = Radar::default();
        assert_eq!(r.lock_next(&me, &[tgt]), Some(1));
        let em = r.update(&me, &[tgt], 0.1);
        assert!(em.contains(&(1, Emission::Track)));
        // RWR on the target sees the lock, at its 12 o'clock.
        let mut w = Rwr::default();
        let rot = Quat::from_rotation_y(std::f32::consts::PI);
        w.update(tgt.pos, rot, &[(0, me.pos, Emission::Track)], 0.1);
        assert_eq!(w.worst(), Some(Emission::Track));
        assert!(w.contacts[0].az.abs() < 0.2);
        // The target turns to beam the radar (look-down): the lock breaks.
        tgt.vel = Vec3::new(250.0, 0.0, 0.0);
        for _ in 0..20 {
            r.update(&me, &[tgt], 0.1);
        }
        assert!(r.stt.is_none());
    }

    #[test]
    fn rail_seeker_growls_and_tracks_off_boresight() {
        let pos = Vec3::new(0.0, 5000.0, 0.0);
        let mut s = RailSeeker::default();
        let tp = Vec3::new(0.0, 5000.0, 3000.0);
        let c = [Contact { src: Source::Unit(1), pos: tp, vel: Vec3::ZERO, strength: 1500.0, age: 9.0 }];
        s.update(pos, Quat::IDENTITY, None, &c, Vec3::NEG_Y, 0.7, 0.045, 10_000.0, 0.5);
        assert!(s.signal > 0.2);
        assert!(s.uncage(pos, &c, 0.045, 10_000.0));
        // The target moves 30° off the nose: still tracked; 50°: lost (gimbal).
        let c2 = [Contact { pos: pos + Vec3::new(1500.0, 0.0, 2600.0), ..c[0] }];
        s.update(pos, Quat::IDENTITY, None, &c2, Vec3::NEG_Y, 0.7, 0.045, 10_000.0, 0.1);
        assert!(s.locked.is_some());
        let c3 = [Contact { pos: pos + Vec3::new(2600.0, 0.0, 2000.0), ..c[0] }];
        s.update(pos, Quat::IDENTITY, None, &c3, Vec3::NEG_Y, 0.7, 0.045, 10_000.0, 0.1);
        assert!(s.locked.is_none());
    }
}
