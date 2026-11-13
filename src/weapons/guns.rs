//! Gun installations and loadouts per type, and the firing mechanism
//! (rate, spin-up, ammunition, harmonisation, dispersion, recoil).
//!
//! * Spitfire IX (C wing): 2 × 20 mm Hispano Mk II, 120 rpg, ~600 rpm,
//!   inboard; 4 × .303 Browning Mk II, 350 rpg, ~1,150 rpm, outboard. The
//!   spade grip's rocker fires machine guns (top), cannon (bottom) or both.
//!   Harmonised to converge at 250 yd (selectable 200 / 250 / 300 / 400 yd).
//! * F-16C: M61A1 Vulcan, 511 rounds of PGU-28/B, 6,000 rpm after a
//!   ~0.5 s spin-up (the first rounds leave at a lower rate), left LEX.

use super::ballistics::{self, Round};
use super::events::WeaponKind;
use super::missiles::MissileKind;
use crate::flight::{AircraftDef, Category};
use glam::{Quat, Vec3};

/// One gun.
#[derive(Clone, Debug)]
pub struct GunDef {
    pub kind: WeaponKind,
    pub round: Round,
    /// Rounds per minute (sustained).
    pub rpm: f32,
    /// Seconds from trigger to full rate (rotary guns); 0 = recoil-operated.
    pub spin_up: f32,
    /// Muzzle, body frame.
    pub muzzle: Vec3,
    /// Trigger group: 0 = cannon, 1 = machine guns (rotary / single gun: 0).
    pub group: u8,
    pub ammo: u32,
    /// Every n-th round is a tracer (0 = none).
    pub tracer_every: u32,
    /// 1-σ angular dispersion, rad.
    pub sigma: f32,
    /// Spent-case / link ejection direction (body frame), if overboard.
    pub eject: Option<Vec3>,
}

/// Sight fitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sight {
    /// Mk II gyro gunsight (Spitfire).
    Gyro,
    /// HUD with LCOS / EEGS gun funnel (F-16).
    Hud,
}

#[derive(Clone, Debug)]
pub struct Loadout {
    pub guns: Vec<GunDef>,
    pub missiles: Vec<(MissileKind, u8)>,
    pub flares: u16,
    pub chaff: u16,
    pub radar: bool,
    pub rwr: bool,
    pub sight: Sight,
    /// Harmonisation range choices (m) and the default index.
    pub convergence: &'static [f32],
    pub conv_default: usize,
    /// Ejection seat (true) or bail out over the side.
    pub seat: bool,
}

const YD: f32 = 0.9144;
const SPIT_CONV: &[f32] = &[200.0 * YD, 250.0 * YD, 300.0 * YD, 400.0 * YD];
const F16_CONV: &[f32] = &[];

/// The loadout of a type: only fighters / warbirds are armed.
pub fn loadout(def: &AircraftDef) -> Option<Loadout> {
    match def.id {
        "peregrine" => {
            let y = def.wing_height - 0.04;
            let mut guns = Vec::new();
            for s in [1.0f32, -1.0] {
                guns.push(GunDef {
                    kind: WeaponKind::Hispano,
                    round: ballistics::HISPANO,
                    rpm: 600.0,
                    spin_up: 0.0,
                    muzzle: Vec3::new(s * 1.98, y - 0.06, 2.55),
                    group: 0,
                    ammo: 120,
                    tracer_every: 0,
                    sigma: 1.1e-3,
                    eject: Some(Vec3::new(s * 0.1, -1.0, -0.2)),
                });
            }
            for (x, z) in [(2.75f32, 1.82f32), (3.15, 1.66)] {
                for s in [1.0f32, -1.0] {
                    guns.push(GunDef {
                        kind: WeaponKind::Browning303,
                        round: ballistics::BROWNING_303,
                        rpm: 1150.0,
                        spin_up: 0.0,
                        muzzle: Vec3::new(s * x, y, z),
                        group: 1,
                        ammo: 350,
                        tracer_every: 5,
                        sigma: 1.5e-3,
                        eject: Some(Vec3::new(0.0, -1.0, -0.1)),
                    });
                }
            }
            Some(Loadout { guns, missiles: vec![], flares: 0, chaff: 0, radar: false, rwr: false, sight: Sight::Gyro, convergence: SPIT_CONV, conv_default: 1, seat: false })
        }
        "lancer" => Some(Loadout {
            guns: vec![GunDef {
                kind: WeaponKind::M61,
                round: ballistics::PGU28,
                rpm: 6000.0,
                spin_up: 0.5,
                muzzle: Vec3::new(0.92, 0.52, 2.75),
                group: 0,
                ammo: 511,
                tracer_every: 5,
                sigma: 1.7e-3,
                // Cases are retained in the drum (linkless feed).
                eject: None,
            }],
            // Two AIM-9 on the wing tips, two AIM-120 on the outer pylons.
            missiles: vec![(MissileKind::Aim9, 2), (MissileKind::Aim120, 2)],
            flares: 30,
            chaff: 30,
            radar: true,
            rwr: true,
            sight: Sight::Hud,
            convergence: F16_CONV,
            conv_default: 0,
            seat: true,
        }),
        // Other fighters / warbirds that join the roster: generic fits.
        _ if def.category == Category::FastJet => Some(Loadout {
            guns: vec![GunDef {
                kind: WeaponKind::M61,
                round: ballistics::PGU28,
                rpm: 6000.0,
                spin_up: 0.5,
                muzzle: def.eye + Vec3::new(0.8, -0.5, -1.4),
                group: 0,
                ammo: 500,
                tracer_every: 5,
                sigma: 1.7e-3,
                eject: None,
            }],
            missiles: vec![(MissileKind::Aim9, 2)],
            flares: 30,
            chaff: 30,
            radar: true,
            rwr: true,
            sight: Sight::Hud,
            convergence: F16_CONV,
            conv_default: 0,
            seat: true,
        }),
        _ if def.category == Category::Warbird => {
            let y = def.wing_height;
            let mut guns = Vec::new();
            for x in [2.2f32, 2.6, 3.0] {
                for s in [1.0f32, -1.0] {
                    guns.push(GunDef {
                        kind: WeaponKind::Mg50,
                        round: ballistics::BMG50,
                        rpm: 800.0,
                        spin_up: 0.0,
                        muzzle: Vec3::new(s * x, y, 1.6),
                        group: 1,
                        ammo: 400,
                        tracer_every: 5,
                        sigma: 1.4e-3,
                        eject: Some(Vec3::new(0.0, -1.0, 0.0)),
                    });
                }
            }
            Some(Loadout { guns, missiles: vec![], flares: 0, chaff: 0, radar: false, rwr: false, sight: Sight::Gyro, convergence: SPIT_CONV, conv_default: 1, seat: false })
        }
        _ => None,
    }
}

/// Live state of one gun.
#[derive(Clone, Debug, Default)]
pub struct GunState {
    pub ammo: u32,
    /// Rate fraction 0..1 (rotary spin).
    pub spin: f32,
    /// Fractional rounds owed.
    pub acc: f32,
    /// Rounds fired in total (tracer sequence).
    pub fired: u32,
    /// Seconds since the last round (muzzle flash / smoke).
    pub since: f32,
    /// Jammed (battle damage).
    pub jammed: bool,
    /// Barrel heat 0..1 (smoke).
    pub heat: f32,
    /// Dispersion random state (per gun, so a `Fire` event's seed regenerates its rounds).
    pub rng: u32,
}

impl GunState {
    pub fn new(g: &GunDef) -> GunState {
        GunState { ammo: g.ammo, since: 99.0, acc: if g.spin_up > 0.0 { 0.0 } else { 0.999 }, ..Default::default() }
    }
}

/// Gun line (body frame unit vector) harmonised so the round from `muzzle`
/// crosses the sight line (from `eye` along +Z) at range `conv`, the drop at
/// that range compensated. `conv` = 0: parallel to the boresight (rotary
/// gun on the jet, whose HUD computes the lead instead).
pub fn gun_line(g: &GunDef, eye: Vec3, conv: f32, launch_speed: f32) -> Vec3 {
    if conv <= 0.0 {
        // The M61 is boresighted ~2 mrad up so its rounds cross the waterline near 1,500 ft.
        return Vec3::new(0.0, 0.002, 1.0).normalize();
    }
    let (_, drop) = ballistics::Table::new(&g.round, launch_speed, 1000.0).at(conv);
    let aim = eye + Vec3::Z * conv + Vec3::Y * drop;
    (aim - g.muzzle).normalize()
}

/// Rounds to fire this step from one gun (trigger held `on`) and the rate now (rounds / s).
pub fn tick(st: &mut GunState, g: &GunDef, on: bool, dt: f32) -> (u32, f32) {
    st.since += dt;
    st.heat = (st.heat - dt * 0.08).max(0.0);
    let on = on && st.ammo > 0 && !st.jammed;
    if g.spin_up > 0.0 {
        // Hydraulic drive: ~0.5 s to full rate, spins down a little slower.
        let target = if on { 1.0 } else { 0.0 };
        let rate = if on { 1.0 / g.spin_up } else { 1.0 / (g.spin_up * 1.4) };
        st.spin += (target - st.spin).clamp(-rate * dt, rate * dt);
    } else {
        st.spin = if on { 1.0 } else { 0.0 };
    }
    if !on {
        st.acc = st.acc.min(0.999);
        if g.spin_up <= 0.0 {
            // Recoil-operated: the next trigger pull fires at once.
            st.acc = 0.999;
        }
        return (0, 0.0);
    }
    // Rotary guns fire from ~20 % of full rate during spin-up.
    let r = g.rpm / 60.0 * if g.spin_up > 0.0 { (st.spin * 1.25 - 0.25).max(0.0) } else { 1.0 };
    st.acc += r * dt;
    let mut n = 0;
    while st.acc >= 1.0 && st.ammo > 0 {
        st.acc -= 1.0;
        st.ammo -= 1;
        st.fired += 1;
        n += 1;
    }
    if n > 0 {
        st.since = 0.0;
        st.heat = (st.heat + n as f32 * if g.spin_up > 0.0 { 0.0012 } else { 0.006 }).min(1.0);
    }
    (n, r)
}

/// Dispersion: a Gaussian cone about `dir` (Box–Muller from two uniforms).
pub fn disperse(dir: Vec3, sigma: f32, u1: f32, u2: f32) -> Vec3 {
    let r = sigma * (-2.0 * u1.max(1e-7).ln()).sqrt();
    let a = std::f32::consts::TAU * u2;
    let side = dir.any_orthonormal_vector();
    let up = dir.cross(side);
    let q = Quat::from_axis_angle(up, r * a.cos()) * Quat::from_axis_angle(side, r * a.sin());
    (q * dir).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::catalogue;

    #[test]
    fn spitfire_and_f16_loadouts_and_rates() {
        let cat = catalogue();
        let spit = loadout(cat.iter().find(|d| d.id == "peregrine").unwrap()).unwrap();
        assert_eq!(spit.guns.iter().filter(|g| g.kind == WeaponKind::Hispano).count(), 2);
        assert_eq!(spit.guns.iter().filter(|g| g.kind == WeaponKind::Browning303).count(), 4);
        assert!(spit.guns.iter().all(|g| g.ammo == if g.group == 0 { 120 } else { 350 }));
        let f16 = loadout(cat.iter().find(|d| d.id == "lancer").unwrap()).unwrap();
        assert_eq!(f16.guns[0].ammo, 511);
        // Airliners, helicopters, drones, gliders: unarmed.
        for d in cat.iter().filter(|d| !matches!(d.category, Category::FastJet | Category::Warbird)) {
            assert!(loadout(d).is_none(), "{}", d.id);
        }
        // M61: ~0.5 s spin-up, then 100 rounds / s; the Hispano fires at once at 10 / s.
        let g = &f16.guns[0];
        let mut st = GunState::new(g);
        let mut n = 0;
        let dt = 1.0 / 120.0;
        for _ in 0..60 {
            n += tick(&mut st, g, true, dt).0;
        }
        assert!((15..45).contains(&n), "spin-up rounds {n}");
        let mut m = 0;
        for _ in 0..120 {
            m += tick(&mut st, g, true, dt).0;
        }
        assert!((98..=101).contains(&m), "full rate {m}");
        let h = &spit.guns[0];
        let mut st = GunState::new(h);
        assert_eq!(tick(&mut st, h, true, dt).0, 1);
        // Out of ammunition after 120 rounds = 12 s.
        let mut total = 1;
        for _ in 0..(14 * 120) {
            total += tick(&mut st, h, true, dt).0;
        }
        assert_eq!(total, 120);
    }

    #[test]
    fn harmonised_guns_converge_on_the_sight_line() {
        let def = catalogue().into_iter().find(|d| d.id == "peregrine").unwrap();
        let l = loadout(&def).unwrap();
        let conv = l.convergence[l.conv_default];
        for g in &l.guns {
            let dir = gun_line(g, def.eye, conv, 150.0);
            // Fly the round and check where it crosses z = conv.
            let (t, _, _) = ballistics::time_to_range(&g.round, Vec3::new(0.0, 1000.0, 0.0), dir * (g.round.muzzle_v + 150.0), conv).unwrap();
            let _ = t;
            let mut p = g.muzzle + Vec3::new(0.0, 1000.0, 0.0);
            let mut v = dir * (g.round.muzzle_v + 150.0);
            while p.z < def.eye.z + conv {
                (p, v) = ballistics::step(&g.round, p, v, Vec3::ZERO, 1.0 / 2000.0);
            }
            let miss = Vec3::new(p.x, p.y - 1000.0, 0.0) - Vec3::new(def.eye.x, def.eye.y, 0.0);
            assert!(miss.length() < 0.35, "{} at {:?}: miss {:?}", g.kind.label(), g.muzzle, miss);
        }
        let d = disperse(Vec3::Z, 1e-3, 0.5, 0.25);
        assert!((d.angle_between(Vec3::Z) - 1e-3 * (2.0 * 2f32.ln()).sqrt()).abs() < 1e-4);
    }
}
