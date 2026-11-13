//! Battle damage: what hits do to the airframe, its systems and the pilot.
//!
//! Each hit volume ([`HitPart`]) has hit points; a strike removes some and
//! crossing thresholds drives the existing simulations rather than a
//! separate "health bar":
//! * structure → `damage::Damage` (shot-through spars weaken the wing's
//!   limit / ultimate load; a part shot away separates, with the debris,
//!   asymmetric aerodynamics and CG shift that already exist);
//! * systems → `systems` failures (engine partial power / fire / seizure,
//!   fuel leaks, hydraulic systems, generator / electrical fire, jammed
//!   controls, cracked windscreen);
//! * the Spitfire's glycol radiators leak coolant: the Merlin overheats and
//!   seizes a minute or two later (white vapour trailing meanwhile);
//! * burning fuel spreads and eats the structure it is in, and a fire in the
//!   fuselage tanks ends in an explosion if it isn't blown out;
//! * the pilot can be wounded (slower, weaker) or killed.

use super::hitbox::{HitModel, HitPart, N_HIT};
use crate::damage::Part;
use crate::flight::{Aircraft, AircraftDef};
use crate::systems::failures::{Failure, Kind};
use glam::Vec3;

/// A hole / scorch for rendering (body frame).
#[derive(Clone, Copy, Debug)]
pub struct Hole {
    pub pos: Vec3,
    pub normal: Vec3,
    pub size: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Pilot {
    #[default]
    Ok,
    Wounded,
    Dead,
}

/// One strike on a part.
#[derive(Clone, Copy, Debug)]
pub struct Strike {
    pub part: HitPart,
    pub dmg: f32,
    pub he: bool,
    pub incendiary: bool,
    pub pos: Vec3,
    pub normal: Vec3,
}

/// What a strike (or the damage processes) caused, for messages / events.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Failure(Failure),
    Separated(Part),
    Fire(HitPart),
    PilotWounded,
    PilotKilled,
    Explosion,
    GunsJammed,
    Glycol,
}

#[derive(Clone, Debug)]
pub struct BattleDamage {
    pub hp: [f32; N_HIT],
    pub max: [f32; N_HIT],
    pub holes: Vec<Hole>,
    /// Burning fuel (0 = none .. 1 = fully involved) and where.
    pub fire: f32,
    pub fire_at: HitPart,
    pub fire_t: f32,
    /// Fuel vapour leaking (trail intensity 0..1).
    pub leak: f32,
    /// Coolant lost (0..1) and engine temperature excess (1 = seizure).
    pub glycol: f32,
    pub heat: f32,
    /// Engine damage smoke (oil) 0..1.
    pub oil: f32,
    pub pilot: Pilot,
    /// Why this aircraft is going down (None = still flyable).
    pub doomed: Option<String>,
    pub failures: Vec<Failure>,
    pub guns_jammed: bool,
    /// Blew up (fuel / ammunition).
    pub exploded: bool,
    /// Total damage taken (scoring, AI morale).
    pub total: f32,
}

impl BattleDamage {
    pub fn new(hm: &HitModel) -> BattleDamage {
        BattleDamage {
            hp: hm.hp,
            max: hm.hp,
            holes: Vec::new(),
            fire: 0.0,
            fire_at: HitPart::Fuselage,
            fire_t: 0.0,
            leak: 0.0,
            glycol: 0.0,
            heat: 0.0,
            oil: 0.0,
            pilot: Pilot::Ok,
            doomed: None,
            failures: Vec::new(),
            guns_jammed: false,
            exploded: false,
            total: 0.0,
        }
    }
    /// Fraction of a part's hit points lost (0..1).
    pub fn lost(&self, p: HitPart) -> f32 {
        let i = p.idx();
        (1.0 - self.hp[i] / self.max[i].max(1e-3)).clamp(0.0, 1.0)
    }

    fn fail(&mut self, def: &AircraftDef, ac: &mut Aircraft, f: Failure, out: &mut Vec<Outcome>) {
        if self.failures.contains(&f) {
            return;
        }
        crate::systems::set_failure(ac, def, f, true);
        if ac.sys.fail.has(f) {
            self.failures.push(f);
            out.push(Outcome::Failure(f));
        }
    }

    fn separate(&mut self, def: &AircraftDef, ac: &mut Aircraft, p: Part, why: &str, out: &mut Vec<Outcome>) {
        if ac.dmg.is_lost(p) {
            return;
        }
        let sd = crate::damage::struct_def(def.id);
        ac.dmg.separate(sd, def, p, why);
        if ac.dmg.failure.is_none() && matches!(p, Part::WingL | Part::WingR | Part::Aft) {
            ac.dmg.failure = Some(format!("Shot down: {why}"));
        }
        out.push(Outcome::Separated(p));
        if matches!(p, Part::WingL | Part::WingR | Part::Aft | Part::Nose) || (ac.dmg.is_lost(Part::HTailL) && ac.dmg.is_lost(Part::HTailR)) {
            self.doom(format!("{} shot away", p.name()));
        }
    }

    pub fn doom(&mut self, why: String) {
        if self.doomed.is_none() {
            self.doomed = Some(why);
        }
    }

    /// Apply one strike. `roll` is a uniform random number source.
    pub fn strike(&mut self, def: &AircraftDef, ac: &mut Aircraft, s: &Strike, roll: &mut dyn FnMut() -> f32) -> Vec<Outcome> {
        let mut out = Vec::new();
        let i = s.part.idx();
        let before = self.lost(s.part);
        self.hp[i] = (self.hp[i] - s.dmg).max(0.0);
        self.total += s.dmg;
        let after = self.lost(s.part);
        if s.part.is_structure() || s.part == HitPart::Cockpit {
            let size = if s.he { 0.18 + 0.002 * s.dmg.min(80.0) } else { 0.035 + 0.004 * s.dmg.min(20.0) };
            if self.holes.len() >= 160 {
                self.holes.remove(0);
            }
            self.holes.push(Hole { pos: s.pos, normal: s.normal, size });
        }
        let jet = def.engine_kind == crate::flight::EngineKind::Jet;
        let fire_p = |base: f32| base * if s.incendiary { 1.6 } else { 1.0 } * if s.he { 1.5 } else { 1.0 };
        let crossed = |x: f32| before < x && after >= x;
        match s.part {
            HitPart::Pilot => {
                // A hit through the armour / seat: wounded, or worse.
                if after >= 1.0 || (s.dmg > 25.0 && roll() < 0.5) {
                    self.pilot = Pilot::Dead;
                    self.doom("pilot killed".into());
                    out.push(Outcome::PilotKilled);
                } else if self.pilot == Pilot::Ok {
                    self.pilot = Pilot::Wounded;
                    out.push(Outcome::PilotWounded);
                }
            }
            HitPart::Cockpit => {
                if !ac.sys.windscreen_cracked && roll() < 0.6 {
                    ac.sys.windscreen_cracked = true;
                }
                if roll() < 0.3 {
                    self.fail(def, ac, Failure::new(Kind::AttitudeInd, 0), &mut out);
                }
            }
            HitPart::Engine => {
                self.oil = self.oil.max(after * 0.9);
                if crossed(0.3) || (after > 0.3 && roll() < 0.2) {
                    self.fail(def, ac, Failure::new(Kind::PartialPower, 0), &mut out);
                }
                if roll() < fire_p(if jet { 0.10 } else { 0.08 }) {
                    self.fail(def, ac, Failure::new(Kind::EngineFire, 0), &mut out);
                    out.push(Outcome::Fire(HitPart::Engine));
                }
                if after >= 0.85 {
                    self.fail(def, ac, Failure::new(Kind::Flameout, 0), &mut out);
                }
            }
            HitPart::Radiator => {
                if self.glycol == 0.0 {
                    out.push(Outcome::Glycol);
                }
                self.glycol = (self.glycol + 0.2 + 0.01 * s.dmg).min(1.0);
            }
            HitPart::FuelFuselage | HitPart::FuelWingL | HitPart::FuelWingR => {
                let tank = match s.part {
                    // Spitfire: upper tank above the lower; jet: forward / aft reservoirs.
                    HitPart::FuelFuselage if jet => usize::from(s.pos.z < def.eye.z - 4.0),
                    HitPart::FuelFuselage => usize::from(s.pos.y <= 0.1),
                    _ => 1,
                };
                self.fail(def, ac, Failure::new(Kind::FuelLeak, tank), &mut out);
                self.leak = (self.leak + 0.25 + 0.02 * s.dmg).min(1.0);
                if roll() < fire_p(0.12) {
                    self.ignite(s.part, 0.25, &mut out);
                }
            }
            HitPart::WingL | HitPart::WingR | HitPart::AileronL | HitPart::AileronR => {
                let (wing, ail, side) =
                    if matches!(s.part, HitPart::WingL | HitPart::AileronL) { (HitPart::WingL, HitPart::AileronL, 0) } else { (HitPart::WingR, HitPart::AileronR, 1) };
                if matches!(s.part, HitPart::AileronL | HitPart::AileronR) {
                    // The aileron hit also damages the wing behind it.
                    self.hp[wing.idx()] = (self.hp[wing.idx()] - s.dmg * 0.4).max(0.0);
                    if after > 0.6 && roll() < 0.35 {
                        self.fail(def, ac, Failure::new(Kind::ControlJam, 1), &mut out);
                    }
                    if after >= 1.0 {
                        self.fail(def, ac, Failure::new(Kind::ControlJam, 1), &mut out);
                    }
                }
                let wl = self.lost(wing);
                ac.dmg.weaken[side] = wl;
                let _ = ail;
                if wl >= 1.0 {
                    let p = if side == 0 { Part::WingL } else { Part::WingR };
                    self.separate(def, ac, p, &format!("{} shot away", wing.name()), &mut out);
                }
                // Cannon ammunition in the wing (warbird): cook-off.
            }
            HitPart::Ammo => {
                if s.he && roll() < 0.12 {
                    out.push(Outcome::Explosion);
                    // The ammunition bay blows: heavy damage around it.
                    let w = if s.pos.x > 0.3 {
                        HitPart::WingL
                    } else if s.pos.x < -0.3 {
                        HitPart::WingR
                    } else {
                        HitPart::Fuselage
                    };
                    let wi = w.idx();
                    self.hp[wi] = (self.hp[wi] - self.max[wi] * 0.6).max(0.0);
                    if w != HitPart::Fuselage {
                        let side = if w == HitPart::WingL { 0 } else { 1 };
                        ac.dmg.weaken[side] = self.lost(w);
                    }
                }
                if !self.guns_jammed && (after > 0.5 || roll() < 0.15) {
                    self.guns_jammed = true;
                    out.push(Outcome::GunsJammed);
                }
            }
            HitPart::HTailL | HitPart::HTailR => {
                if after > 0.5 && roll() < 0.25 {
                    self.fail(def, ac, Failure::new(Kind::ControlJam, 0), &mut out);
                }
                if after >= 1.0 {
                    let p = if s.part == HitPart::HTailL { Part::HTailL } else { Part::HTailR };
                    self.separate(def, ac, p, &format!("{} shot away", s.part.name()), &mut out);
                }
            }
            HitPart::Elevator => {
                if after > 0.5 && roll() < 0.4 || after >= 1.0 {
                    self.fail(def, ac, Failure::new(Kind::ControlJam, 0), &mut out);
                }
            }
            HitPart::Rudder => {
                if after > 0.5 && roll() < 0.4 || after >= 1.0 {
                    self.fail(def, ac, Failure::new(Kind::ControlJam, 2), &mut out);
                }
            }
            HitPart::Fin => {
                if after >= 1.0 {
                    self.separate(def, ac, Part::Fin, "fin shot away", &mut out);
                }
            }
            HitPart::Aft => {
                // Control runs to the tail.
                if roll() < 0.08 {
                    let axis = if roll() < 0.6 { 0 } else { 2 };
                    self.fail(def, ac, Failure::new(Kind::ControlJam, axis), &mut out);
                }
                if after >= 1.0 {
                    self.separate(def, ac, Part::Aft, "tail shot off", &mut out);
                }
            }
            HitPart::Fuselage => {
                if after >= 1.0 {
                    self.separate(def, ac, Part::Aft, "fuselage broken", &mut out);
                }
            }
            HitPart::Nose => {
                if after >= 1.0 && crate::damage::struct_def(def.id).m_nose > 0.0 {
                    self.separate(def, ac, Part::Nose, "nose shot away", &mut out);
                }
            }
            HitPart::Hydraulics => {
                let n = crate::systems::failures::Kind::Hydraulic.count(&crate::systems::defs::sys_def(def), def);
                self.fail(def, ac, Failure::new(Kind::Hydraulic, 0), &mut out);
                if after > 0.6 && n > 1 {
                    self.fail(def, ac, Failure::new(Kind::Hydraulic, 1), &mut out);
                }
            }
            HitPart::Electrics => {
                self.fail(def, ac, Failure::new(Kind::Generator, 0), &mut out);
                if roll() < fire_p(0.1) {
                    self.fail(def, ac, Failure::new(Kind::ElecFire, 0), &mut out);
                }
            }
            HitPart::Armour => {}
        }
        out
    }

    fn ignite(&mut self, at: HitPart, k: f32, out: &mut Vec<Outcome>) {
        if self.fire <= 0.0 {
            self.fire_at = at;
            self.fire_t = 0.0;
            out.push(Outcome::Fire(at));
        }
        self.fire = (self.fire + k).min(1.0);
    }

    /// Damage processes over time: fire, coolant loss, leaks.
    pub fn update(&mut self, def: &AircraftDef, ac: &mut Aircraft, dt: f32, roll: &mut dyn FnMut() -> f32) -> Vec<Outcome> {
        let mut out = Vec::new();
        if ac.crashed.is_some() {
            return out;
        }
        // ---- glycol: the Merlin overheats once the coolant is gone.
        if self.glycol > 0.0 {
            self.glycol = (self.glycol + dt * 0.02).min(1.0);
            let power = ac.engine_power().clamp(0.2, 1.0);
            self.heat += dt * self.glycol * power / 70.0;
            if self.heat > 0.6 {
                self.fail(def, ac, Failure::new(Kind::PartialPower, 0), &mut out);
            }
            if self.heat >= 1.0 && !self.failures.contains(&Failure::new(Kind::Flameout, 0)) {
                self.fail(def, ac, Failure::new(Kind::Flameout, 0), &mut out);
                self.oil = 1.0;
                if roll() < 0.5 {
                    self.fail(def, ac, Failure::new(Kind::EngineFire, 0), &mut out);
                }
            }
        }
        // ---- fire: grows with fuel and airflow, eats the structure it sits in;
        // speed can blow it out early on.
        if self.fire > 0.0 {
            self.fire_t += dt;
            let v = ac.airspeed();
            let blow_out = if self.fire < 0.4 && v > 120.0 { (v - 120.0) / 400.0 } else { 0.0 };
            self.fire = (self.fire + dt * (0.035 - blow_out)).clamp(0.0, 1.0);
            if self.fire <= 0.0 {
                self.fire_t = 0.0;
            }
            let burn = self.fire * dt * 6.0;
            let at = self.fire_at;
            let structural = match at {
                HitPart::FuelWingL => HitPart::WingL,
                HitPart::FuelWingR => HitPart::WingR,
                _ => HitPart::Fuselage,
            };
            let si = structural.idx();
            self.hp[si] = (self.hp[si] - burn).max(0.0);
            if matches!(structural, HitPart::WingL | HitPart::WingR) {
                let side = if structural == HitPart::WingL { 0 } else { 1 };
                ac.dmg.weaken[side] = self.lost(structural);
                if self.hp[si] <= 0.0 {
                    let p = if side == 0 { Part::WingL } else { Part::WingR };
                    self.separate(def, ac, p, "wing burnt through", &mut out);
                }
            } else if self.fire > 0.85 && self.fire_t > 12.0 && !self.exploded && roll() < dt * 0.15 {
                // The fuselage tanks go up.
                self.exploded = true;
                out.push(Outcome::Explosion);
                self.separate(def, ac, Part::Aft, "fuel tanks exploded", &mut out);
                self.doom("exploded".into());
            }
        }
        // ---- leaks fade as the tank drains; any engine fire is also visible.
        self.leak = (self.leak - dt * 0.004).max(0.0);
        out
    }

    /// Doomed, or a burning wreck / dead pilot.
    pub fn is_doomed(&self) -> bool {
        self.doomed.is_some()
    }
    /// Smoke trailing from damage: (oil / engine smoke, fuel / coolant vapour, fire) 0..1.
    pub fn trail(&self, ac: &Aircraft) -> (f32, f32, f32) {
        let eng_fire = ac.sys.eng.first().map(|e| e.fire).unwrap_or(0.0);
        (self.oil.max(eng_fire * 0.8), self.leak.max(self.glycol * (1.0 - self.heat * 0.3)), self.fire.max(eng_fire))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debris::PartGeom;
    use crate::flight::catalogue;
    use crate::weapons::hitbox::planform_for;

    fn setup(id: &str) -> (AircraftDef, Aircraft, BattleDamage) {
        let def = catalogue().into_iter().find(|d| d.id == id).unwrap();
        let sd = crate::damage::struct_def(id);
        let hm = HitModel::build(&def, sd, &PartGeom::estimate(&def, sd), planform_for(&def, sd, &[]));
        let mut ac = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 120.0);
        ac.sys = crate::systems::Systems::new(&def, false);
        let bd = BattleDamage::new(&hm);
        (def, ac, bd)
    }

    fn hit(part: HitPart, dmg: f32, he: bool) -> Strike {
        Strike { part, dmg, he, incendiary: true, pos: Vec3::ZERO, normal: Vec3::Y }
    }

    /// Damage → failure mapping on the real systems / structure models.
    #[test]
    fn damage_maps_to_systems_failures_and_separations() {
        let mut never = || 0.99f32;
        let mut always = || 0.0f32;
        // Spitfire: engine hits → partial power, then seized; radiator → glycol → overheat → flameout.
        let (def, mut ac, mut bd) = setup("peregrine");
        bd.strike(&def, &mut ac, &hit(HitPart::Engine, 30.0, false), &mut never);
        assert!(ac.sys.fail.has(Failure::new(Kind::PartialPower, 0)));
        assert!(!ac.sys.fail.has(Failure::new(Kind::Flameout, 0)));
        bd.strike(&def, &mut ac, &hit(HitPart::Engine, 60.0, false), &mut never);
        assert!(ac.sys.fail.has(Failure::new(Kind::Flameout, 0)));
        let (def, mut ac, mut bd) = setup("peregrine");
        let o = bd.strike(&def, &mut ac, &hit(HitPart::Radiator, 3.0, false), &mut never);
        assert!(o.contains(&Outcome::Glycol));
        let mut t = 0.0;
        while !ac.sys.fail.has(Failure::new(Kind::Flameout, 0)) && t < 400.0 {
            bd.update(&def, &mut ac, 0.1, &mut never);
            t += 0.1;
        }
        eprintln!("glycol hit → engine seized after {t:.0} s");
        assert!((40.0..300.0).contains(&t), "{t}");
        // Wing shot away → separation + doomed; partial damage weakens it first.
        let (def, mut ac, mut bd) = setup("peregrine");
        bd.strike(&def, &mut ac, &hit(HitPart::WingL, 80.0, true), &mut never);
        assert!(ac.dmg.weaken[0] > 0.4 && !ac.dmg.is_lost(Part::WingL));
        bd.strike(&def, &mut ac, &hit(HitPart::WingL, 100.0, true), &mut never);
        assert!(ac.dmg.is_lost(Part::WingL) && bd.is_doomed());
        // Pilot killed → doomed.
        let (def, mut ac, mut bd) = setup("peregrine");
        bd.strike(&def, &mut ac, &hit(HitPart::Pilot, 10.0, false), &mut never);
        assert_eq!(bd.pilot, Pilot::Wounded);
        bd.strike(&def, &mut ac, &hit(HitPart::Pilot, 40.0, true), &mut never);
        assert_eq!(bd.pilot, Pilot::Dead);
        assert!(bd.is_doomed());
        // F-16: tanks → leak (+ fire with bad luck), hydraulics → both systems, electrics → generator,
        // controls → jams.
        let (def, mut ac, mut bd) = setup("lancer");
        let o = bd.strike(&def, &mut ac, &hit(HitPart::FuelFuselage, 50.0, true), &mut always);
        assert!(ac.sys.fail.has(Failure::new(Kind::FuelLeak, 1)), "aft tank: {o:?}");
        assert!(bd.fire > 0.0);
        bd.strike(&def, &mut ac, &hit(HitPart::Hydraulics, 30.0, false), &mut never);
        assert!(ac.sys.fail.has(Failure::new(Kind::Hydraulic, 0)) && ac.sys.fail.has(Failure::new(Kind::Hydraulic, 1)));
        bd.strike(&def, &mut ac, &hit(HitPart::Electrics, 10.0, false), &mut never);
        assert!(ac.sys.fail.has(Failure::new(Kind::Generator, 0)));
        bd.strike(&def, &mut ac, &hit(HitPart::Rudder, 30.0, false), &mut always);
        assert!(ac.sys.fail.has(Failure::new(Kind::ControlJam, 2)));
        // The fuselage fire grows (not blown out at 120 m/s once established) and ends in an explosion.
        let mut t = 0.0;
        let mut r = 0u32;
        let mut rnd = move || {
            r = r.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (r >> 8) as f32 / 16_777_216.0
        };
        bd.fire = 0.5;
        while !bd.exploded && t < 200.0 {
            bd.update(&def, &mut ac, 0.1, &mut rnd);
            t += 0.1;
        }
        eprintln!("F-16 fuselage fire → explosion after {t:.0} s");
        assert!(bd.exploded && ac.dmg.is_lost(Part::Aft));
    }
}
