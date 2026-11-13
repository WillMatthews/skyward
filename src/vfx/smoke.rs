//! Airshow smoke (Track H2b): the aerobat's smoke system — oil injected into
//! the hot exhaust — leaves a dense white trail that hangs in the air, drifts
//! with the wind and slowly spreads. Toggled with the thrust-reverse key (T)
//! on types with `AircraftDef::smoke_system` ([`Vfx::smoke`]).

use super::{Ctx, Kind, Vfx, mk};
use glam::Vec3;

impl Vfx {
    pub(super) fn airshow_smoke(&mut self, c: &Ctx, dt: f32) {
        let ac = c.ac;
        if !self.smoke || !c.def.smoke_system || ac.crashed.is_some() || !ac.engine_running(0) {
            return;
        }
        // The exhaust stacks under the cowling, a little behind the firewall.
        let e = c.def.engine_pos.first().copied().unwrap_or(Vec3::ZERO);
        let exhaust = e - Vec3::Z * 1.0 - Vec3::Y * 0.35;
        let p = c.pos + c.rot * exhaust;
        // A thick tube that stays ~40 s, spreading; plus billows for body.
        self.trail(400, mk::TRAIL, p, 1.1, 1.0, 40.0, 0.35, 0.05, 0.02, 2.5);
        let vel = ac.vel;
        let n = self.count(0x5E0E, 70.0, dt);
        for _ in 0..n {
            let d = self.rsph();
            let q = p - vel * dt * self.rnd() + d * 0.25;
            let v = vel * 0.15 + d * 1.2 + self.wind;
            let s = 0.5 + 0.4 * self.rnd();
            self.emit(Kind::Steam, q, v, 14.0, s, s * 9.0, Vec3::new(0.93, 0.93, 0.92), 0.5, 0.35, 0.0, 0.08);
        }
    }
}
