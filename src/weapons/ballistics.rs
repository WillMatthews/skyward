//! Exterior ballistics of gun rounds: a point mass under gravity and
//! Mach-dependent drag in the ISA atmosphere (or a supplied density).
//!
//! Drag deceleration `a = ½ ρ v² Cd(M) A / m`, with `Cd(M)` a standard
//! spin-stabilised spitzer projectile curve (subsonic plateau, transonic
//! rise, slow supersonic decay — the shape of the G7 reference) scaled by
//! the round's form factor. Rounds are fired with the aircraft's velocity
//! added (the shooter's velocity is the launch frame), so a fighter at
//! 200 m/s firing ahead gives its rounds ≈ 1,080 m/s ground speed.

use crate::flight::GRAVITY;
use glam::Vec3;

/// A kind of gun round.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Round {
    pub name: &'static str,
    /// Projectile mass, kg.
    pub mass: f32,
    /// Calibre, m.
    pub calibre: f32,
    /// Muzzle velocity, m/s.
    pub muzzle_v: f32,
    /// Form factor × the reference drag curve.
    pub form: f32,
    /// High-explosive filler, kg TNT-equivalent (0 = ball / AP).
    pub he: f32,
    /// Incendiary (sets fuel alight more readily).
    pub incendiary: bool,
    /// Self-destruct (tracer burn-out / fuze) after this many seconds; 0 = never.
    pub self_destruct: f32,
}

/// 20 mm Hispano Mk II (British HE/I): 130 g at 880 m/s (long barrel).
pub const HISPANO: Round = Round { name: "20 mm Hispano", mass: 0.130, calibre: 0.020, muzzle_v: 880.0, form: 1.75, he: 0.011, incendiary: true, self_destruct: 0.0 };
/// .303 Browning Mk II (Mk VII ball / B Mk VI incendiary): 11.3 g at 744 m/s.
pub const BROWNING_303: Round = Round { name: ".303 Browning", mass: 0.0113, calibre: 0.0079, muzzle_v: 744.0, form: 0.95, he: 0.0, incendiary: true, self_destruct: 0.0 };
/// 20 × 102 mm PGU-28/B SAPHEI (M61A1): 102 g at 1,050 m/s.
pub const PGU28: Round = Round { name: "20 mm PGU-28/B", mass: 0.102, calibre: 0.020, muzzle_v: 1050.0, form: 0.95, he: 0.0104, incendiary: true, self_destruct: 0.0 };
/// .50 BMG M2 (generic warbird fallback): 46 g at 870 m/s.
pub const BMG50: Round = Round { name: ".50 Browning", mass: 0.046, calibre: 0.0127, muzzle_v: 870.0, form: 0.9, he: 0.0, incendiary: true, self_destruct: 0.0 };

/// Reference drag coefficient vs Mach (spin-stabilised spitzer, G7 shape).
pub fn cd_ref(m: f32) -> f32 {
    const T: [(f32, f32); 11] =
        [(0.0, 0.157), (0.7, 0.157), (0.85, 0.172), (0.95, 0.245), (1.05, 0.38), (1.2, 0.39), (1.5, 0.355), (2.0, 0.305), (2.5, 0.275), (3.0, 0.255), (4.0, 0.235)];
    let m = m.max(0.0);
    for w in T.windows(2) {
        let (a, b) = (w[0], w[1]);
        if m <= b.0 {
            let t = ((m - a.0) / (b.0 - a.0)).clamp(0.0, 1.0);
            return a.1 + (b.1 - a.1) * t;
        }
    }
    T[T.len() - 1].1
}

impl Round {
    /// Cross-section, m².
    pub fn area(&self) -> f32 {
        std::f32::consts::PI * 0.25 * self.calibre * self.calibre
    }
    /// Drag deceleration vector for air-relative velocity `v_air` at density `rho` and speed of sound `a`.
    pub fn drag(&self, v_air: Vec3, rho: f32, a: f32) -> Vec3 {
        let s = v_air.length();
        if s < 1e-3 {
            return Vec3::ZERO;
        }
        let cd = cd_ref(s / a.max(1.0)) * self.form;
        -v_air * (0.5 * rho * s * cd * self.area() / self.mass)
    }
    /// Kinetic energy (J) at a closing speed.
    pub fn energy(&self, v: f32) -> f32 {
        0.5 * self.mass * v * v
    }
}

/// One integration step (semi-implicit midpoint): returns the new (pos, vel).
pub fn step(r: &Round, pos: Vec3, vel: Vec3, wind: Vec3, dt: f32) -> (Vec3, Vec3) {
    let atm = crate::flight::isa(pos.y);
    let g = Vec3::new(0.0, -GRAVITY, 0.0);
    let a0 = r.drag(vel - wind, atm.density, atm.speed_of_sound) + g;
    let vm = vel + a0 * (0.5 * dt);
    let a1 = r.drag(vm - wind, atm.density, atm.speed_of_sound) + g;
    let v1 = vel + a1 * dt;
    (pos + vm * dt, v1)
}

/// Fly a round from `pos` with `vel` until it has travelled `range` m
/// downrange along the initial direction; returns (time of flight, drop below
/// the bore line, remaining speed). For gunsights, tests and the AI's lead.
pub fn time_to_range(r: &Round, pos: Vec3, vel: Vec3, range: f32) -> Option<(f32, f32, f32)> {
    let dir = vel.normalize_or_zero();
    if dir == Vec3::ZERO {
        return None;
    }
    let (mut p, mut v) = (pos, vel);
    let mut t = 0.0;
    let dt = 1.0 / 480.0;
    while t < 8.0 {
        let (np, nv) = step(r, p, v, Vec3::ZERO, dt);
        let d0 = (p - pos).dot(dir);
        let d1 = (np - pos).dot(dir);
        if d1 >= range {
            let f = ((range - d0) / (d1 - d0).max(1e-6)).clamp(0.0, 1.0);
            let hit = p.lerp(np, f);
            let along = pos + dir * range;
            // Drop: below the bore line, measured in the vertical plane of fire.
            let drop = (along - hit).length() * if hit.y < along.y { 1.0 } else { -1.0 };
            return Some((t + f * dt, drop, v.lerp(nv, f).length()));
        }
        p = np;
        v = nv;
        t += dt;
    }
    None
}

/// Precomputed fly-out table for a round fired at a given launch speed and
/// altitude: time and mean speed to each range (cheap lookups for sights / AI).
#[derive(Clone, Debug)]
pub struct Table {
    /// (range m, time s, drop m) at 50 m steps from 0.
    pub rows: Vec<(f32, f32, f32)>,
}

impl Table {
    pub fn new(r: &Round, launch_speed: f32, alt: f32) -> Table {
        let mut rows = vec![(0.0, 0.0, 0.0)];
        let v = Vec3::new(0.0, 0.0, r.muzzle_v + launch_speed);
        let pos = Vec3::new(0.0, alt, 0.0);
        let mut range = 50.0;
        while range <= 2500.0 {
            match time_to_range(r, pos, v, range) {
                Some((t, d, _)) => rows.push((range, t, d)),
                None => break,
            }
            range += 50.0;
        }
        Table { rows }
    }
    /// (time of flight, drop) to `range` (linear interpolation; extrapolated past the end).
    pub fn at(&self, range: f32) -> (f32, f32) {
        let r = range.max(0.0);
        let n = self.rows.len();
        if n < 2 {
            return (0.0, 0.0);
        }
        let i = ((r / 50.0) as usize).min(n - 2);
        let (a, b) = (self.rows[i], self.rows[i + 1]);
        let f = (r - a.0) / (b.0 - a.0);
        (a.1 + (b.1 - a.1) * f, a.2 + (b.2 - a.2) * f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 20 mm Hispano Mk II, fired from the ground: 880 m/s, 130 g. Published:
    /// ≈ 1.66 s to 1,000 yd (914 m) for the 131 g / 850 m/s Mk V round
    /// (ww2aircraft.net gunnery data); the Mk II's 400 m time of flight
    /// derived from the same drag is ≈ 0.52 s with ≈ 1.3–1.4 m of drop
    /// (gravity alone over 0.52 s: 1.33 m).
    #[test]
    fn hispano_drop_and_time_of_flight_at_400_m() {
        let (t, drop, v) = time_to_range(&HISPANO, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, HISPANO.muzzle_v), 400.0).unwrap();
        eprintln!("Hispano 400 m: tof {t:.3} s, drop {drop:.2} m, v {v:.0} m/s");
        assert!((0.48..0.57).contains(&t), "tof {t}");
        assert!((1.1..1.7).contains(&drop), "drop {drop}");
        assert!((640.0..790.0).contains(&v), "v {v}");
        // The Mk V reference round to 1,000 yd.
        let mk5 = Round { muzzle_v: 850.0, mass: 0.131, ..HISPANO };
        let (t, _, _) = time_to_range(&mk5, Vec3::ZERO, Vec3::new(0.0, 0.0, 850.0), 914.4).unwrap();
        eprintln!("Hispano Mk V 1,000 yd: tof {t:.2} s (published ≈ 1.66 s)");
        assert!((1.5..1.85).contains(&t), "tof {t}");
    }

    #[test]
    fn launch_speed_adds_and_altitude_reduces_drag() {
        let (t0, _, _) = time_to_range(&PGU28, Vec3::ZERO, Vec3::new(0.0, 0.0, PGU28.muzzle_v), 600.0).unwrap();
        let (t1, _, _) = time_to_range(&PGU28, Vec3::ZERO, Vec3::new(0.0, 0.0, PGU28.muzzle_v + 250.0), 600.0).unwrap();
        let (t2, _, _) = time_to_range(&PGU28, Vec3::new(0.0, 9000.0, 0.0), Vec3::new(0.0, 0.0, PGU28.muzzle_v), 600.0).unwrap();
        assert!(t1 < t0 * 0.85, "{t0} {t1}");
        assert!(t2 < t0, "{t0} {t2}");
        // The light .303 slows much faster than the 20 mm.
        let (a, _, va) = time_to_range(&BROWNING_303, Vec3::ZERO, Vec3::new(0.0, 0.0, 744.0), 600.0).unwrap();
        let (b, _, vb) = time_to_range(&HISPANO, Vec3::ZERO, Vec3::new(0.0, 0.0, 880.0), 600.0).unwrap();
        assert!(a > b && va < vb);
    }

    #[test]
    fn table_matches_direct_flyout() {
        let tb = Table::new(&HISPANO, 0.0, 0.0);
        let (t, d) = tb.at(400.0);
        let (t2, d2, _) = time_to_range(&HISPANO, Vec3::ZERO, Vec3::new(0.0, 0.0, 880.0), 400.0).unwrap();
        assert!((t - t2).abs() < 0.01 && (d - d2).abs() < 0.05);
    }
}
