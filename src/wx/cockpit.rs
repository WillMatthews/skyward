//! The weather as the crew meets it, kept with the sim (`Sim::wxc`): the
//! windscreen and wipers, the pilot's head bouncing on its neck in
//! turbulence (camera shake), and the weather radar sweep for the ND.

use super::glass::{GlassState, WiperMode};
use super::radar::{self, RadarCtl, RadarImage};
use crate::flight::{Aircraft, AircraftDef, Category};
use glam::{Vec2, Vec3, vec2, vec3};

/// The pilot's head on its neck: a damped spring driven by the changes in
/// the specific force the seat transmits (turbulence jolts, hard landings,
/// the bump of a gust) — not by the steady g of a turn.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadShake {
    /// Head offset from the design eye (body frame, m) and its velocity.
    pub pos: Vec3,
    vel: Vec3,
    /// Slow average of the specific force (body m/s²).
    f_slow: Vec3,
    prev_vel: Option<Vec3>,
}

impl HeadShake {
    /// `vel` world velocity (m/s), `rot` attitude, `dt` frame time.
    pub fn update(&mut self, vel: Vec3, rot: glam::Quat, on: bool, dt: f32) {
        if !on || (dt.is_nan() || dt <= 0.0) || !vel.is_finite() {
            *self = HeadShake { prev_vel: Some(vel), ..Default::default() };
            return;
        }
        let dt = dt.min(0.05);
        let acc = match self.prev_vel {
            Some(p) => (vel - p) / dt,
            None => Vec3::ZERO,
        };
        self.prev_vel = Some(vel);
        // Specific force in the body frame (what the seat pushes with).
        let f = rot.conjugate() * (acc + Vec3::Y * 9.81);
        let f = if f.is_finite() { f.clamp_length_max(80.0) } else { Vec3::ZERO };
        self.f_slow += (f - self.f_slow) * (dt / 0.5).min(1.0);
        let jolt = f - self.f_slow;
        // Neck: ~2 Hz, under-damped; ~1.5 cm per g of jolt (vertical), less sideways / fore-aft.
        let w = std::f32::consts::TAU * 2.0;
        let z = 0.42;
        let gain = vec3(0.0009, 0.0016, 0.0007);
        let a = -jolt * gain * w * w - self.pos * w * w - self.vel * 2.0 * z * w;
        self.vel += a * dt;
        self.pos += self.vel * dt;
        self.pos = self.pos.clamp(Vec3::splat(-0.06), Vec3::splat(0.06));
        if !self.pos.is_finite() {
            *self = HeadShake::default();
        }
    }
    /// Head nod (rad, pitch) that goes with the bounce.
    pub fn nod(&self) -> f32 {
        -self.pos.y * 1.6
    }
}

/// The weather radar: controls and the sweep in progress.
#[derive(Clone, Debug)]
pub struct Radar {
    pub ctl: RadarCtl,
    pub img: RadarImage,
    /// Next azimuth column to sweep, and the storm cells for this sweep.
    next_az: usize,
    cells: Vec<super::storm::Cell>,
}

impl Default for Radar {
    fn default() -> Self {
        Radar { ctl: RadarCtl::default(), img: RadarImage::empty(1.0), next_az: 0, cells: Vec::new() }
    }
}

/// The ND tilt presets cycled by the radar-tilt key (None = automatic).
pub const TILTS: [Option<f32>; 6] = [None, Some(2.0), Some(0.0), Some(-1.5), Some(-3.0), Some(-5.0)];

impl Radar {
    /// Sweep a few azimuth columns (a full sweep every ~1.5 s): `range` m.
    pub fn update(&mut self, world: &crate::world::World, ac: &Aircraft, range: f32, dt: f32) {
        if (self.img.range - range).abs() > 1.0 {
            self.img = RadarImage::empty(range);
            self.next_az = 0;
        }
        if self.next_az == 0 {
            self.cells = if world.wxs.storms.active() { world.wxs.storms.cells_near(vec2(ac.pos.x, ac.pos.z), range) } else { Vec::new() };
        }
        let cols = ((radar::AZ_BINS as f32 * dt / 1.5).ceil() as usize).clamp(1, radar::AZ_BINS);
        let storms = &world.wxs.storms;
        let wx = &world.wx;
        // Showers in the cumulus / steady rain under a deck also return.
        let rain_dbz = match wx.precip {
            crate::weather::Precip::None => -99.0,
            crate::weather::Precip::Drizzle => 10.0,
            crate::weather::Precip::Snow => 12.0 + 10.0 * wx.intensity,
            _ => 14.0 + 20.0 * wx.intensity,
        };
        let base = world.clouds.base();
        // Rain bands: patchy over tens of km (cells / bands in the deck).
        let patch = |xz: Vec2| {
            let a = (xz.x * 1.3e-4).sin() * (xz.y * 1.7e-4).cos() + (xz.x * 3.1e-4 + xz.y * 2.3e-4).sin() * 0.6;
            a / 1.6
        };
        let dbz = |p: Vec3| {
            let mut out = super::storm::StormAir::default();
            if !self.cells.is_empty() {
                storms.air_cells(&self.cells, p, &mut out);
            }
            let mut d = out.dbz();
            if rain_dbz > 0.0 && p.y < base + 1500.0 {
                let xz = vec2(p.x, p.z);
                let cov = if wx.showers { world.clouds.coverage(xz) } else { 0.5 + 0.5 * patch(xz) };
                if cov > 0.3 {
                    d = d.max(rain_dbz - 12.0 * (1.0 - cov) + if wx.showers { 8.0 * cov } else { 6.0 * patch(xz) });
                }
            }
            d
        };
        let ground = |xz: Vec2| crate::flight::Environment::ground_height(world, xz);
        let sweep = radar::scan_columns(ac.pos, ac.heading(), &self.ctl, range, &dbz, &ground, self.next_az, cols, &mut self.img);
        self.next_az = sweep % radar::AZ_BINS;
    }
}

/// Weather state the sim keeps for the crew.
#[derive(Clone, Debug, Default)]
pub struct WxCockpit {
    pub glass: GlassState,
    pub head: HeadShake,
    pub radar: Radar,
    /// Camera shake on / off (Settings).
    pub shake: bool,
}

impl WxCockpit {
    /// Per frame. `range_m` = the ND range (radar).
    pub fn update(&mut self, world: &crate::world::World, ac: &Aircraft, def: &AircraftDef, range_m: f32, dt: f32, paused: bool) {
        if paused || (dt.is_nan() || dt <= 0.0) {
            return;
        }
        // Rain at the aircraft: the weather's, or heavier under a storm.
        let r = world.render_weather();
        let (mmh, cloud) = world.wxs.rain_at(ac.pos);
        let storm_rain = (mmh / 50.0).clamp(0.0, 1.0);
        let in_cloud = cloud.max(if ac.pos.y > world.clouds.cu_base() && ac.pos.y < world.clouds.top() { world.clouds.coverage(vec2(ac.pos.x, ac.pos.z)) } else { 0.0 });
        // Above the deck it doesn't rain.
        let below = if ac.pos.y < world.clouds.base() + 300.0 { 1.0 } else { 0.0 };
        let rain = (r.rain * below).max(storm_rain);
        let ice = ac.sys.fx.ice.clamp(0.0, 1.0);
        self.glass.demist = def.category == Category::Airliner;
        let oat = ac.oat();
        let dew = world.wx.dew_c - (world.wx.temp_c - oat);
        self.glass.update(dt, rain, r.snow * below, ac.airspeed(), oat, dew, ice, in_cloud);
        if self.glass.wipers.mode == WiperMode::Off && def.category != Category::Airliner {
            self.glass.wipers = Default::default();
        }
        self.head.update(ac.vel, ac.rot, self.shake && ac.crashed.is_none(), dt);
        if def.category == Category::Airliner {
            self.radar.update(world, ac, range_m, dt);
        }
    }

    pub fn cycle_wipers(&mut self) -> &'static str {
        self.glass.wipers.mode = self.glass.wipers.mode.next();
        self.glass.wipers.mode.name()
    }

    pub fn cycle_tilt(&mut self) -> String {
        let i = TILTS.iter().position(|t| *t == self.radar.ctl.tilt).unwrap_or(0);
        self.radar.ctl.tilt = TILTS[(i + 1) % TILTS.len()];
        match self.radar.ctl.tilt {
            None => "Radar tilt AUTO".into(),
            Some(t) => format!("Radar tilt {t:+.1}°"),
        }
    }

    pub fn cycle_gain(&mut self) -> String {
        let g = [0.0, 4.0, 8.0, -6.0];
        let i = g.iter().position(|x| *x == self.radar.ctl.gain).unwrap_or(0);
        self.radar.ctl.gain = g[(i + 1) % g.len()];
        if self.radar.ctl.gain == 0.0 { "Radar gain CAL".into() } else { format!("Radar gain {:+.0} dB", self.radar.ctl.gain) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn head_bounces_with_jolts_not_steady_g() {
        let mut h = HeadShake::default();
        let rot = glam::Quat::IDENTITY;
        // Steady climb / turn-free cruise: no shake.
        for i in 0..240 {
            h.update(vec3(0.0, 2.0, -70.0) + Vec3::Y * 0.0 * i as f32, rot, true, 1.0 / 60.0);
        }
        assert!(h.pos.length() < 1e-3, "{:?}", h.pos);
        // A sharp 1 g upward jolt: the head drops a centimetre or two, then settles.
        let mut v = vec3(0.0, 0.0, -70.0);
        let mut peak: f32 = 0.0;
        for i in 0..240 {
            if (10..16).contains(&i) {
                v.y += 9.81 / 60.0 * 1.0;
            }
            h.update(v, rot, true, 1.0 / 60.0);
            peak = peak.max(h.pos.y.abs());
        }
        assert!(peak > 0.002 && peak < 0.06, "peak {peak}");
        assert!(h.pos.length() < peak * 0.5);
        // Off: nothing.
        let mut o = HeadShake::default();
        o.update(v, rot, false, 1.0 / 60.0);
        assert_eq!(o.pos, Vec3::ZERO);
    }
}
