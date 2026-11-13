//! The weather as the crew meets it, kept with the sim (`Sim::wxc`): the
//! windscreen and wipers, and the weather radar sweep for the ND. (The
//! cockpit head motion in turbulence is the sim's `head_motion`, driven by
//! the same accelerations: Settings › Camera.)

use super::glass::{GlassState, WiperMode};
use super::radar::{self, RadarCtl, RadarImage};
use crate::flight::{Aircraft, AircraftDef, Category};
use glam::{Vec2, Vec3, vec2};

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
    pub radar: Radar,
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
