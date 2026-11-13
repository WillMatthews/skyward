//! Radios: two NAV receivers (VOR / localizer + glideslope, with their DME),
//! an ADF, two COMs and a transponder, each with active / standby
//! frequencies.
//!
//! Reception is modelled: a station is heard only inside its service volume
//! (VOR / DME by class, NDB by power, localizer ±10° to 18 NM and ±35° to
//! 10 NM, glideslope ±8° to 10 NM and below 1.75 θ), above the radio horizon
//! and — for VHF — with a clear line of sight over the terrain. Receivers
//! need bus power and can fail (systems/failures.rs). Outputs are signed
//! "fly towards" deviations: + = the course / path is right / above.

use super::db::{Ils, Navaid};
use super::geo::{self, LL, NM};
use glam::{Vec2, Vec3, vec3};

/// What the receivers need from the world.
pub trait RadioEnv {
    fn to_world(&self, ll: LL) -> Vec2;
    fn to_ll(&self, p: Vec2) -> LL;
    /// Terrain height (None where unknown).
    fn ground(&self, p: Vec2) -> Option<f32>;
    /// Surface height (m) of runway end `rwy` at `airport`, if loaded.
    fn runway_elev(&self, airport: &str, rwy: &str) -> Option<f32>;
}

/// Scan interval (s): station selection, range and line of sight.
const SCAN: f32 = 0.5;
/// VOR full-scale deflection.
pub const VOR_FULL: f32 = 10.0 * std::f32::consts::PI / 180.0;

/// Outputs of a NAV receiver.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NavOut {
    /// A usable bearing signal (NAV flag clear).
    pub valid: bool,
    /// Tuned to a localizer.
    pub loc: bool,
    pub ident: String,
    /// LOC: front course; VOR: the selected course (OBS). True, radians.
    pub course: f32,
    /// Angular deviation (rad, + = fly right).
    pub dev: f32,
    /// Needle, −1..1 = full scale (+ = right).
    pub cdi: f32,
    /// VOR TO / FROM (None = OFF).
    pub to: Option<bool>,
    /// VOR radial the aircraft is on (true, rad, from the station).
    pub radial: f32,
    /// Bearing to the station (true, rad).
    pub bearing: f32,
    pub gs_valid: bool,
    /// Glideslope angular deviation (rad, + = fly up) and needle −1..1.
    pub gs_dev: f32,
    pub gs: f32,
    /// Glidepath angle (rad).
    pub gs_angle: f32,
    /// DME slant range (m), if the station has DME and it is received.
    pub dme: Option<f32>,
    /// LOC: lateral distance from the course (m, + = course to the right) and
    /// distance to the threshold along the course (m, + = before it).
    pub xtk: f32,
    pub to_thr: f32,
    /// Threshold elevation (m) and the runway end (LOC).
    pub thr_elev: f32,
    pub runway: String,
    pub airport: String,
    /// Height of the aircraft above the glidepath (m, + = above).
    pub gp_err: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NavRx {
    pub active: u32,
    pub standby: u32,
    /// Selected course (true, radians).
    pub obs: f32,
    pub ident_audio: bool,
    pub out: NavOut,
    station: Option<Navaid>,
    gs_ok: bool,
    dme_ok: bool,
    scan_t: f32,
}

impl NavRx {
    fn new(active: u32, standby: u32) -> NavRx {
        NavRx { active, standby, obs: 0.0, ident_audio: false, out: NavOut::default(), station: None, gs_ok: false, dme_ok: false, scan_t: 0.0 }
    }
    pub fn station(&self) -> Option<&Navaid> {
        self.station.as_ref()
    }
    pub fn swap(&mut self) {
        std::mem::swap(&mut self.active, &mut self.standby);
        self.scan_t = 0.0;
    }
    pub fn tune(&mut self, khz: u32) {
        if khz != self.active {
            self.active = khz;
            self.scan_t = 0.0;
            self.station = None;
            self.out = NavOut::default();
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AdfOut {
    pub valid: bool,
    pub ident: String,
    /// Bearing to the station (true, rad).
    pub bearing: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Adf {
    pub active: u32,
    pub standby: u32,
    pub ident_audio: bool,
    pub out: AdfOut,
    station: Option<Navaid>,
    scan_t: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum XpdrMode {
    Off,
    #[default]
    Stby,
    On,
    Alt,
}

impl XpdrMode {
    pub fn label(self) -> &'static str {
        match self {
            XpdrMode::Off => "OFF",
            XpdrMode::Stby => "STBY",
            XpdrMode::On => "ON",
            XpdrMode::Alt => "ALT",
        }
    }
    pub fn next(self) -> XpdrMode {
        match self {
            XpdrMode::Off => XpdrMode::Stby,
            XpdrMode::Stby => XpdrMode::On,
            XpdrMode::On => XpdrMode::Alt,
            XpdrMode::Alt => XpdrMode::Off,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Xpdr {
    /// Four octal digits.
    pub code: [u8; 4],
    pub mode: XpdrMode,
    /// Seconds of IDENT left.
    pub ident: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Com {
    pub active: u32,
    pub standby: u32,
}

/// All the radios of one aircraft.
#[derive(Clone, Debug, PartialEq)]
pub struct Radios {
    pub nav: [NavRx; 2],
    pub adf: Adf,
    pub com: [Com; 2],
    pub xpdr: Xpdr,
    /// Bus power and failures (set by the owner each step).
    pub powered: bool,
    pub nav_failed: [bool; 2],
    pub gs_failed: [bool; 2],
    pub adf_failed: bool,
    pub time: f32,
}

impl Default for Radios {
    fn default() -> Self {
        Radios {
            nav: [NavRx::new(110_300, 113_750), NavRx::new(115_100, 115_600)],
            adf: Adf { active: 362, standby: 389, ident_audio: false, out: AdfOut::default(), station: None, scan_t: 0.0 },
            com: [Com { active: 118_505, standby: 121_500 }, Com { active: 121_500, standby: 122_800 }],
            xpdr: Xpdr { code: [7, 0, 0, 0], mode: XpdrMode::Alt, ident: 0.0 },
            powered: true,
            nav_failed: [false; 2],
            gs_failed: [false; 2],
            adf_failed: false,
            time: 0.0,
        }
    }
}

/// Frequency steps (kHz) and limits.
pub const NAV_MIN: u32 = 108_000;
pub const NAV_MAX: u32 = 117_950;
pub const COM_MIN: u32 = 118_000;
pub const COM_MAX: u32 = 136_975;
pub const ADF_MIN: u32 = 190;
pub const ADF_MAX: u32 = 1_799;

/// Step a frequency by whole MHz (`mhz`) and channels (`ch`), wrapping
/// inside the band like a radio's outer / inner knob.
pub fn step_freq(f: u32, mhz: i32, ch: i32, lo: u32, hi: u32, spacing: u32) -> u32 {
    let (m, k) = (f / 1000, f % 1000);
    let (m_lo, m_hi) = ((lo / 1000) as i32, (hi / 1000) as i32);
    let m = (m as i32 - m_lo + mhz).rem_euclid(m_hi - m_lo + 1) + m_lo;
    let n = (1000 / spacing) as i32;
    let k = ((k / spacing) as i32 + ch).rem_euclid(n) as u32 * spacing;
    (m as u32 * 1000 + k).clamp(lo, hi)
}

/// Height of the station's antenna (m) above its site.
const ANT_H: f32 = 8.0;

fn site(env: &dyn RadioEnv, ll: LL, elev: f32) -> Vec3 {
    let p = env.to_world(ll);
    let h = env.ground(p).unwrap_or(elev).max(elev - 30.0);
    vec3(p.x, h + ANT_H, p.y)
}

/// Station reception check: range, radio horizon, line of sight.
fn heard(env: &dyn RadioEnv, ant: Vec3, ac: Vec3, range_m: f32, los: bool) -> bool {
    let d = ant.distance(ac);
    if d > range_m {
        return false;
    }
    let base = ant.y.min(env.ground(glam::vec2(ac.x, ac.z)).unwrap_or(ant.y - ANT_H));
    if glam::vec2(ant.x - ac.x, ant.z - ac.z).length() > geo::radio_horizon(ac.y - base, ant.y - base) {
        return false;
    }
    !los || geo::line_of_sight(&|p| env.ground(p), ant, ac, 40)
}

impl Radios {
    /// Advance by `dt`: rescan stations every half second, recompute the
    /// deviations every call. `stations` = navaids around the aircraft.
    pub fn step(&mut self, env: &dyn RadioEnv, stations: &[Navaid], ac: Vec3, dt: f32) {
        self.time += dt;
        self.xpdr.ident = (self.xpdr.ident - dt).max(0.0);
        for k in 0..2 {
            let ok = self.powered && !self.nav_failed[k];
            let gs_fail = self.gs_failed[k];
            let rx = &mut self.nav[k];
            rx.scan_t -= dt;
            if !ok {
                rx.station = None;
                rx.out = NavOut::default();
                rx.scan_t = 0.0;
                continue;
            }
            if rx.scan_t <= 0.0 {
                rx.scan_t = SCAN;
                scan_nav(rx, env, stations, ac);
            }
            nav_outputs(rx, env, ac, gs_fail);
        }
        // ADF
        let ok = self.powered && !self.adf_failed;
        let adf = &mut self.adf;
        adf.scan_t -= dt;
        if !ok {
            adf.station = None;
            adf.out = AdfOut::default();
        } else {
            if adf.scan_t <= 0.0 {
                adf.scan_t = SCAN;
                let best = stations
                    .iter()
                    .filter(|s| s.kind.is_ndb() && s.freq_khz == adf.active)
                    .map(|s| (s, site(env, s.ll(), s.elev_m)))
                    .filter(|(s, a)| heard(env, *a, ac, s.range_nm * NM, false))
                    .min_by(|a, b| a.1.distance(ac).total_cmp(&b.1.distance(ac)));
                adf.station = best.map(|b| b.0.clone());
            }
            adf.out = match &adf.station {
                Some(s) => {
                    let p = env.to_world(s.ll());
                    AdfOut { valid: true, ident: s.ident.clone(), bearing: geo::heading_of(p - glam::vec2(ac.x, ac.z)) }
                }
                None => AdfOut::default(),
            };
        }
    }

    /// The ident to key in the headset: (ident, volume) of the first
    /// receiver with ident audio on and a station.
    pub fn ident_audio(&self) -> Option<(&str, f32)> {
        for rx in &self.nav {
            if rx.ident_audio && (rx.out.valid || rx.out.dme.is_some()) {
                return rx.station.as_ref().map(|s| (s.ident.as_str(), 1.0));
            }
        }
        if self.adf.ident_audio && self.adf.out.valid {
            return self.adf.station.as_ref().map(|s| (s.ident.as_str(), 0.8));
        }
        None
    }

    /// Squawk as a 4-digit number (e.g. 7000).
    pub fn squawk(&self) -> u32 {
        self.xpdr.code.iter().fold(0, |a, d| a * 10 + *d as u32)
    }
}

fn scan_nav(rx: &mut NavRx, env: &dyn RadioEnv, stations: &[Navaid], ac: Vec3) {
    let p = glam::vec2(ac.x, ac.z);
    let mut best: Option<(f32, &Navaid, bool, bool)> = None;
    for s in stations.iter().filter(|s| s.kind.vhf() && s.freq_khz == rx.active) {
        let (ok, gs, dme) = match &s.ils {
            Some(ils) => {
                let (ant, dir) = loc_frame(env, ils);
                let (dev, d, front) = geo::loc_deviation(ant, dir, p);
                let az = dev.abs();
                let loc_ok = front && ((az <= 10f32.to_radians() && d <= 18.0 * NM) || (az <= 35f32.to_radians() && d <= 10.0 * NM));
                let elev = env.runway_elev(&ils.airport, &ils.runway).unwrap_or(ils.thr_elev);
                let a = vec3(ant.x, elev + ANT_H, ant.y);
                let loc_ok = loc_ok && heard(env, a, ac, 25.0 * NM, true);
                let gs_ok = loc_ok
                    && ils.gs.is_some_and(|g| {
                        let gp = env.to_world(g);
                        let gd = gp.distance(p);
                        let ga = geo::elevation_angle(vec3(gp.x, elev, gp.y), ac);
                        az <= 8f32.to_radians() && gd <= 10.5 * NM && ga <= 1.75 * ils.gs_angle
                    });
                (loc_ok, gs_ok, loc_ok && s.kind.has_dme())
            }
            None => {
                let ant = site(env, s.ll(), s.elev_m);
                let ok = heard(env, ant, ac, s.range_nm * NM, true);
                (ok && s.kind.bearing(), false, ok && s.kind.has_dme())
            }
        };
        if !(ok || dme) {
            continue;
        }
        let d = env.to_world(s.ll()).distance(p) - if ok { 1e6 } else { 0.0 };
        if best.as_ref().is_none_or(|b| d < b.0) {
            best = Some((d, s, gs, dme));
        }
    }
    match best.map(|b| (b.1.clone(), b.2, b.3)) {
        Some((s, gs, dme)) => {
            rx.gs_ok = gs;
            rx.dme_ok = dme;
            rx.station = Some(s);
        }
        None => {
            rx.station = None;
            rx.gs_ok = false;
            rx.dme_ok = false;
        }
    }
}

/// Localizer antenna and front-course direction in the world frame: along
/// the runway as the world draws it (threshold → antenna), so the beam lies
/// exactly on the painted centreline.
pub fn loc_frame(env: &dyn RadioEnv, ils: &Ils) -> (Vec2, Vec2) {
    let ant = env.to_world(ils.loc);
    let thr = env.to_world(ils.thr);
    let d = ant - thr;
    (ant, if d.length() > 100.0 { d / d.length() } else { geo::dir_of(ils.course) })
}

fn nav_outputs(rx: &mut NavRx, env: &dyn RadioEnv, ac: Vec3, gs_failed: bool) {
    let p = glam::vec2(ac.x, ac.z);
    let Some(s) = &rx.station else {
        rx.out = NavOut { course: rx.obs, ..NavOut::default() };
        return;
    };
    let mut o = NavOut { ident: s.ident.clone(), airport: s.airport.clone(), ..NavOut::default() };
    let sp = env.to_world(s.ll());
    o.bearing = geo::heading_of(sp - p);
    o.radial = geo::wrap_2pi(o.bearing + std::f32::consts::PI);
    if rx.dme_ok {
        let dp = env.to_world(s.dme_ll());
        let dh = env.ground(dp).unwrap_or(s.elev_m);
        o.dme = Some(vec3(dp.x, dh + ANT_H, dp.y).distance(ac));
    }
    match &s.ils {
        Some(ils) => {
            let (ant, dir) = loc_frame(env, ils);
            let (dev, d, front) = geo::loc_deviation(ant, dir, p);
            let thr = env.to_world(ils.thr);
            o.loc = true;
            o.valid = front;
            o.course = geo::heading_of(dir);
            o.dev = dev;
            o.cdi = (dev / ils.half_width).clamp(-1.0, 1.0);
            o.xtk = d * dev.sin();
            o.to_thr = (thr - p).dot(dir);
            o.runway = ils.runway.clone();
            o.thr_elev = env.runway_elev(&ils.airport, &ils.runway).unwrap_or(ils.thr_elev);
            o.gs_angle = ils.gs_angle;
            if let Some(g) = ils.gs {
                let gp = env.to_world(g);
                let site = vec3(gp.x, o.thr_elev, gp.y);
                let ga = geo::elevation_angle(site, ac);
                o.gs_dev = ils.gs_angle - ga;
                o.gs = (o.gs_dev / (0.24 * ils.gs_angle)).clamp(-1.0, 1.0);
                // Along the course from the glideslope site.
                let along = (gp - p).dot(dir).max(1.0);
                o.gp_err = ac.y - (o.thr_elev + along * ils.gs_angle.tan());
                o.gs_valid = rx.gs_ok && !gs_failed;
            }
        }
        None => {
            o.course = rx.obs;
            if s.kind.bearing() {
                let (dev, to) = geo::vor_deviation(rx.obs, o.radial);
                o.valid = true;
                o.dev = dev;
                o.cdi = (dev / VOR_FULL).clamp(-1.0, 1.0);
                // Near the station (cone of confusion) the TO / FROM flag drops.
                let agl = (ac.y - env.ground(sp).unwrap_or(s.elev_m)).max(0.0);
                let over = sp.distance(p) < (agl * 0.6).max(300.0);
                o.to = if over { None } else { Some(to) };
                o.xtk = sp.distance(p) * dev.sin();
            }
        }
    }
    rx.out = o;
}
