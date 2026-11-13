//! Weather drama and turbulence (Track WX).
//!
//! * [`turbulence`] — Dryden / MIL-F-8785C turbulence as a frozen spatial
//!   field: intensities and scale lengths by height and severity, rotational
//!   gusts from its spatial differences (flight.rs).
//! * [`storm`] — thunderstorm cells (cumulonimbus, anvils, overshooting
//!   tops, rain shafts / virga, updrafts, downdrafts, gust fronts) from the
//!   METAR / Open-Meteo, lightning flashes and their channels, thunder delay.
//! * [`mountain`] — lee waves, rotors and lenticular clouds behind ridges.
//! * [`wake`] — wake vortices behind other aircraft (and your own, after a 360).
//! * [`radar`] — the airliners' weather radar for the ND.
//! * [`glass`] — rain on the windscreen, wipers, misting, frost.
//! * [`gpu`] — lightning channels, the scene flash and the windscreen pass.
//!
//! [`WxSim`] ties them together for the world (`World::wxs`): rebuilt when
//! the weather changes, ticked every frame, and sampled by `World::wind` /
//! `World::updraft` (so the flight model, rotorcraft, floats and windsocks
//! all feel the same air).

pub mod cockpit;
pub mod glass;
pub mod gpu;
pub mod mountain;
pub mod radar;
pub mod storm;
pub mod turbulence;
pub mod wake;

use glam::{Vec2, Vec3, vec2};
use storm::{Cell, Flash, FlashKind, StormField, StormSpec};

/// Weather phenomena around the aircraft (owned by the world).
#[derive(Clone, Debug)]
pub struct WxSim {
    pub storms: StormField,
    pub waves: mountain::WaveField,
    pub wakes: wake::WakeField,
    /// The player's turbulence setting.
    pub mode: turbulence::Mode,
    /// Cells within reach of the focus (field space), refreshed each tick.
    local: Vec<Cell>,
    /// Wave field: what it was built for (field centre, wind), when.
    wave_key: Option<(Vec2, Vec2, usize)>,
    wave_spec: mountain::WaveSpec,
    /// Lightning flashes started in the last ~1.5 s (world space, now).
    pub flashes: Vec<Flash>,
    /// Flash forced at the next tick (screenshots: `SKYWARD_FLASH=cg|ic`).
    pub force: Option<FlashKind>,
    forced: Vec<Flash>,
    pub time: f32,
    /// Bumped when the storm field changes (renderer re-bakes its map).
    pub version: u64,
    /// Bumped when the lenticular grid changes.
    pub wave_version: u64,
}

impl Default for WxSim {
    fn default() -> Self {
        WxSim {
            storms: StormField::none(),
            waves: mountain::WaveField::empty(),
            wakes: wake::WakeField::default(),
            mode: turbulence::Mode::Realistic,
            local: Vec::new(),
            wave_key: None,
            wave_spec: mountain::WaveSpec { wind: Vec2::ZERO, stable: 0.0, lenticular: false, lent_alt: 4000.0 },
            flashes: Vec::new(),
            force: None,
            forced: Vec::new(),
            time: 0.0,
            version: 1,
            wave_version: 1,
        }
    }
}

/// Example weather for the Conditions screen (after the station code): a
/// thunderstorm preset and other drama, decoded like any METAR.
pub const EXAMPLE_METARS: [(&str, &str); 6] = [
    ("Thunderstorm", "151650Z 23018G35KT 4000 +TSRA BKN025CB SCT080 26/21 Q1004"),
    ("Squall line + hail", "151650Z 27025G45KT 2000 +TSRAGR SQ BKN015CB OVC060 22/19 Q1001"),
    ("Storms nearby", "151650Z 20010KT 9999 VCTS FEW040CB SCT100 28/18 Q1009"),
    ("Mountain wave", "151650Z 18035G50KT 9999 FEW080 SCT120 19/04 Q1002 RMK ACSL"),
    ("Icing drizzle", "151650Z 03012KT 3000 -FZDZ BR OVC006 M03/M04 Q1021"),
    ("Gusty crosswind", "151650Z 32028G42KT 9999 SCT030 12/04 Q0998"),
];

/// `SKYWARD_FLASH=cg|ic`: force a lightning flash as the first frame
/// renders (storm screenshots / thunder renders).
pub fn forced_flash_from_env() -> Option<FlashKind> {
    std::env::var("SKYWARD_FLASH").ok().map(|v| if v.eq_ignore_ascii_case("ic") { FlashKind::Cloud } else { FlashKind::Ground })
}

impl WxSim {
    /// A fresh state honouring `SKYWARD_FLASH`.
    pub fn with_env() -> WxSim {
        WxSim { force: forced_flash_from_env(), ..Default::default() }
    }

    /// New weather: rebuild the storms; the waves rebuild on the next tick.
    pub fn set_weather(&mut self, wx: &crate::weather::Wx, seed: u32, ground: f32) {
        let spec = StormSpec::from_wx(wx);
        let drift = self.storms.drift();
        self.storms = StormField::new(seed ^ 0x5707, spec);
        self.storms.set_ground(ground);
        self.storms.set_drift(drift);
        self.wave_spec = mountain::WaveSpec::from_wx(wx);
        self.wave_key = None;
        self.local.clear();
        self.version += 1;
    }

    /// Per frame: storm drift (= the cloud drift), the cells near the focus,
    /// lightning, the lee-wave grid. `focus_field` = focus in field
    /// coordinates (fixed to the ground), `terrain(field xz)` = ground height,
    /// `tiles` = resident tile count (new terrain → rebuild the waves).
    pub fn tick(&mut self, t: f32, focus: Vec2, field_off: Vec2, drift: Vec2, tiles: usize, terrain: &dyn Fn(Vec2) -> f32) {
        self.time = t;
        self.storms.set_drift(drift);
        self.local = self.storms.cells_near(focus, 30_000.0);
        // Lightning: everything within 60 km that flashed in the last 1.5 s.
        self.flashes = self.storms.flashes(t, focus, 60_000.0, 1.5);
        if let Some(k) = self.force.take()
            && let Some(f) = self.storms.forced_flash(t, focus, k)
        {
            self.forced.push(f);
        }
        self.forced.retain(|f| t - f.t0 < f.duration() + 2.0);
        self.flashes.extend(self.forced.iter().copied());
        // Lee waves: rebuild when the aircraft moved > 12 km from the grid
        // centre, the wind changed, or terrain streamed in (debounced).
        let fc = focus + field_off;
        let spec = self.wave_spec;
        let rebuild = match self.wave_key {
            None => true,
            Some((c, w, n)) => c.distance(fc) > 12_000.0 || (w - spec.wind).length() > 3.0 || (tiles > n + 12 && t > 0.0),
        };
        if rebuild && spec.drive() > 0.0 {
            self.waves = mountain::WaveField::build(spec, fc, terrain);
            self.wave_key = Some((fc, spec.wind, tiles));
            self.wave_version += 1;
        } else if rebuild {
            self.waves = mountain::WaveField::empty();
            self.wave_key = Some((fc, spec.wind, tiles));
        }
    }

    pub fn set_ground(&mut self, g: f32) {
        self.storms.set_ground(g);
        self.version += 1;
    }

    /// The storms' air at world `p` (from the cells cached by `tick`).
    pub fn storm_air(&self, p: Vec3) -> storm::StormAir {
        let mut out = storm::StormAir::default();
        if self.local.is_empty() || !p.is_finite() {
            return out;
        }
        self.storms.air_cells(&self.local, p, &mut out);
        out
    }

    /// Extra air motion at world `p` (field coordinates `pf` for the
    /// ground-fixed waves / wakes): (wind m/s world, updraft m/s, turbulence severity 0..4).
    pub fn air(&self, p: Vec3, pf: Vec3) -> (Vec3, f32, f32) {
        let s = self.storm_air(p);
        let (ww, wh, wsev) = self.waves.air(vec2(pf.x, pf.z), p.y);
        let wake = self.wakes.velocity(pf, self.time, 8.0);
        let k = self.mode.scale();
        // Storm drafts and waves are the mean flow (felt in every mode);
        // wakes are turbulence (scaled by the setting).
        let wind = Vec3::new(s.outflow.x + wh.x, 0.0, s.outflow.y + wh.y) + wake * k;
        (wind, s.updraft + ww, s.severity.max(wsev))
    }

    /// Vertical air motion only (storm drafts + mountain waves), m/s.
    pub fn updraft(&self, p: Vec3, pf: Vec3) -> f32 {
        let s = self.storm_air(p);
        s.updraft + self.waves.air(vec2(pf.x, pf.z), p.y).0
    }

    /// Rain at a world point from the storms (mm/h) and whether it is in cloud.
    pub fn rain_at(&self, p: Vec3) -> (f32, f32) {
        let s = self.storm_air(p);
        (s.rain_mmh, s.cloud)
    }

    /// Radar reflectivity (dBZ) at a world point from the storms.
    pub fn dbz(&self, p: Vec3) -> f32 {
        self.storm_air(p).dbz()
    }

    /// The cells for the GPU storm map (nearest the eye).
    pub fn gpu_cells(&self, eye: Vec2) -> Vec<Cell> {
        self.storms.gpu_cells(eye)
    }

    /// Scene light from the flashes now: (world position of the brightest,
    /// intensity 0..~2, total flash light scaled by distance 0..1 for the
    /// ambient lift).
    pub fn flash_light(&self, t: f32, eye: Vec3) -> Option<(Vec3, f32, f32)> {
        let mut best: Option<(Vec3, f32, f32)> = None;
        let mut total = 0.0;
        for f in &self.flashes {
            let b = f.brightness(t);
            if b <= 0.0 {
                continue;
            }
            let pos = if f.kind == FlashKind::Ground { (f.origin + f.strike) * 0.5 } else { f.origin };
            let d = pos.distance(eye).max(500.0);
            let lit = b * (3000.0 / d).powi(2).min(4.0);
            total += lit;
            if best.is_none_or(|x| x.1 < b) {
                best = Some((pos, b, 0.0));
            }
        }
        best.map(|(p, b, _)| (p, b, total.min(4.0)))
    }
}

impl StormField {
    /// [`StormField::air`] over a pre-collected cell list.
    pub fn air_cells(&self, cells: &[Cell], p: Vec3, out: &mut storm::StormAir) {
        let f = vec2(p.x, p.z) - self.drift();
        for c in cells {
            // Skip cells far beyond their reach (outflow ~4 rain radii, anvil).
            if c.center.distance(f) > c.radius * 5.0 + c.anvil_len + 4000.0 {
                continue;
            }
            self.cell_air_pub(c, f, p.y, out);
        }
    }
}

/// Thunder for the audio: when each flash's sound reaches the listener.
#[derive(Clone, Debug, Default)]
pub struct ThunderQueue {
    /// (flash key, arrival time, distance km, rumble s, energy, world direction of the nearest channel point).
    pending: Vec<(u64, f32, f32, f32, f32, Vec3)>,
    seen: Vec<(u64, f32)>,
}

/// A thunder sound arriving now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Thunder {
    /// Distance to the nearest part of the channel (km).
    pub km: f32,
    /// How long the rumble rolls on (s): the spread of channel distances / c.
    pub rumble: f32,
    pub energy: f32,
    /// Direction from the listener (world, unit).
    pub dir: Vec3,
    pub ground: bool,
}

impl ThunderQueue {
    /// Feed this frame's flashes; returns the thunder that arrives at the
    /// listener `ear` by time `t`. Flashes beyond 25 km are not heard.
    pub fn update(&mut self, flashes: &[Flash], ear: Vec3, t: f32) -> Vec<Thunder> {
        for f in flashes {
            if self.seen.iter().any(|(k, _)| *k == f.key) {
                continue;
            }
            self.seen.push((f.key, f.t0));
            let segs = storm::channel(f);
            let (t1, t2) = storm::thunder_times(f, &segs, ear);
            let km = (t1 - f.t0) * storm::SOUND_SPEED / 1000.0;
            if km > 25.0 {
                continue;
            }
            let near = segs.iter().flat_map(|s| [s.a, s.b]).min_by(|a, b| a.distance(ear).total_cmp(&b.distance(ear))).unwrap_or(f.origin);
            self.pending.push((f.key | if f.kind == FlashKind::Ground { 1 << 63 } else { 0 }, t1, km, (t2 - t1).clamp(1.5, 25.0), f.energy, (near - ear).normalize_or_zero()));
        }
        self.seen.retain(|(_, t0)| t - t0 < 120.0);
        let mut out = Vec::new();
        self.pending.retain(|&(key, at, km, rumble, energy, dir)| {
            if at <= t {
                out.push(Thunder { km, rumble, energy, dir, ground: key >> 63 == 1 });
                false
            } else {
                at - t < 120.0
            }
        });
        out
    }
}

#[cfg(test)]
mod thunder_tests {
    use super::*;
    use glam::vec3;

    #[test]
    fn thunder_arrives_after_the_flash() {
        let wx = crate::weather::Wx::from_metar_str("EGLL 151350Z 24015G30KT 4000 +TSRA BKN012CB 22/18 Q1006").unwrap();
        let mut s = WxSim::default();
        s.set_weather(&wx, 3, 0.0);
        s.force = Some(FlashKind::Ground);
        s.tick(10.0, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, 0, &|_| 0.0);
        let f = *s.flashes.iter().find(|f| f.key >> 40 == 0xf1a5).expect("forced flash");
        let mut q = ThunderQueue::default();
        let ear = f.strike + vec3(3000.0, 2.0, 0.0);
        let mut heard = None;
        let mut t = f.t0;
        while t < f.t0 + 30.0 {
            let fl = if t < f.t0 + 1.0 { vec![f] } else { vec![] };
            if let Some(th) = q.update(&fl, ear, t).first() {
                heard = Some((t, *th));
                break;
            }
            t += 1.0 / 60.0;
        }
        let (t_heard, th) = heard.expect("thunder heard");
        let segs = storm::channel(&f);
        let d = segs.iter().flat_map(|s| [s.a, s.b]).map(|p| p.distance(ear)).fold(f32::MAX, f32::min);
        assert!((t_heard - f.t0 - d / storm::SOUND_SPEED).abs() < 0.03, "delay {} vs {}", t_heard - f.t0, d / storm::SOUND_SPEED);
        assert!(th.km > 2.0 && th.km < 3.2 && th.rumble > 1.5 && th.ground, "{th:?}");
        // The flash lights the scene while it lasts.
        assert!(s.flash_light(f.t0 + 0.005, ear).is_some_and(|l| l.1 > 0.3));
    }

    #[test]
    fn air_is_finite_and_calm_without_storms() {
        let s = WxSim::default();
        let (w, u, sev) = s.air(vec3(10.0, 500.0, 10.0), vec3(10.0, 500.0, 10.0));
        assert_eq!((w, u, sev), (Vec3::ZERO, 0.0, 0.0));
    }
}

#[cfg(test)]
mod tests;
