//! Night lighting (Track J): city, airport and aircraft lights, the night sky
//! (moon, stars, light-pollution skyglow) and the GPU passes that draw them.
//!
//! * Light *sources* are compact instances ([`LightInst`]): a position, an
//!   intensity in candela and a kind (street lamp, runway edge light, PAPI,
//!   sequenced flasher, ...). They are generated in the tile workers
//!   ([`place::CityGen`], called from `scene.rs`), from the runways
//!   ([`place::airport_lights`]) and per frame from the aircraft model
//!   ([`place::AircraftLightRig`]).
//! * Every light is drawn as an energy-conserving sprite (its irradiance at
//!   the eye spread over a footprint of at least ~1.5 px, so distant lights
//!   neither alias nor sparkle), dimmed by the aerial perspective, with a haze
//!   / fog halo; also in the mirrored water-reflection pass.
//! * Lamps that light the ground (street lights, apron floods) are splatted
//!   into a ground light map around the eye that every lit surface samples.
//! * The aircraft's landing / taxi lights, strobes and beacon are a few
//!   analytic spot / point lights in the scene shader, with visible beams in
//!   haze and fog ([`gpu::LightRenderer::beams`]).

pub mod place;
pub mod gpu;
pub mod sky;

pub use place::{AircraftLightRig, AircraftLights, TileLights};
pub use gpu::LightRenderer;

use glam::Vec3;

/// One light source on the GPU (32 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LightInst {
    /// Position in its frame (tile frame, world frame or eye-relative).
    pub pos: [f32; 3],
    /// Luminous intensity toward the viewer at its peak (cd).
    pub cd: f32,
    /// Linear rgb (unorm8, brightest channel = 1) + kind in the top byte.
    pub col: u32,
    /// Main direction (snorm8 x3) + a byte parameter.
    pub dir: u32,
    /// Kind-specific parameters (second colour, PAPI angle, phase, ...).
    pub a: f32,
    pub b: f32,
}

/// Light kinds (top byte of [`LightInst::col`]); == `lights.wgsl`.
pub mod kind {
    /// Steady omnidirectional point (aircraft nav lights, flashes driven per frame).
    pub const OMNI: u32 = 0;
    /// Street lamp (on at dusk); b = mounting height (m), a = ground intensity (cd, for the light map).
    pub const STREET: u32 = 1;
    /// Runway light seen in two directions: `col` toward viewers on the -dir
    /// side, `a` (packed colour bits) toward the +dir side. dir = runway axis.
    pub const BIDIR: u32 = 2;
    /// Directional light shining toward `dir` (approach / TDZ lights).
    pub const UNIDIR: u32 = 3;
    /// Sequenced flasher ("rabbit") toward `dir`; a = sequence phase 0..1.
    pub const FLASHER: u32 = 4;
    /// PAPI unit toward `dir`: white above elevation angle `a` (rad), red below.
    pub const PAPI: u32 = 5;
    /// Rotating airport beacon: white / green (a = packed second colour).
    pub const BEACON: u32 = 6;
    /// Apron / pitch floodlight mast: b = height, a = ground intensity (cd).
    pub const FLOOD: u32 = 8;
    /// Aircraft-warning light on a tall structure (steady red at night).
    pub const OBSTACLE: u32 = 9;
    /// Aircraft landing / taxi light beam toward `dir`; a = cos(half angle).
    pub const SPOT: u32 = 10;
    /// Taxiway edge / centreline light (small, omnidirectional).
    pub const TAXI: u32 = 11;
    /// Aggregated far-city lights (low-zoom tiles): b = z14 cell code, hidden
    /// where detailed tiles are loaded.
    pub const FAR: u32 = 12;
    /// Flashing white obstacle strobe; a = phase.
    pub const STROBE: u32 = 13;
}

/// Pack a linear colour (normalised to its brightest channel) and a kind.
pub fn pack_col(c: [f32; 3], kind: u32) -> u32 {
    let m = c[0].max(c[1]).max(c[2]).max(1e-6);
    let q = |v: f32| ((v / m).clamp(0.0, 1.0) * 255.0 + 0.5) as u32;
    q(c[0]) | (q(c[1]) << 8) | (q(c[2]) << 16) | (kind << 24)
}

/// Pack a direction (snorm8 x3) and a byte parameter.
pub fn pack_dir(d: Vec3, param: u8) -> u32 {
    let d = d.normalize_or_zero();
    let q = |v: f32| ((v.clamp(-1.0, 1.0) * 127.0).round() as i32 as u32) & 0xff;
    q(d.x) | (q(d.y) << 8) | (q(d.z) << 16) | ((param as u32) << 24)
}

/// A colour packed into an f32 parameter slot (bit pattern; never a NaN
/// since the top byte is < 0x7f).
pub fn col_param(c: [f32; 3]) -> f32 {
    f32::from_bits(pack_col(c, 0) & 0x00ff_ffff)
}

impl LightInst {
    pub fn new(pos: Vec3, cd: f32, col: [f32; 3], kind: u32) -> Self {
        LightInst { pos: pos.into(), cd, col: pack_col(col, kind), dir: pack_dir(Vec3::Y, 0), a: 0.0, b: 0.0 }
    }
    pub fn with_dir(mut self, d: Vec3, param: u8) -> Self {
        self.dir = pack_dir(d, param);
        self
    }
    pub fn with_ab(mut self, a: f32, b: f32) -> Self {
        self.a = a;
        self.b = b;
        self
    }
    pub fn kind(&self) -> u32 {
        self.col >> 24
    }
}

/// Light colours (linear rgb).
pub mod colour {
    /// Cool-white LED street lighting (~4000 K).
    pub const LED_COOL: [f32; 3] = [0.86, 0.92, 1.0];
    /// Warm-white LED (~3000 K).
    pub const LED_WARM: [f32; 3] = [1.0, 0.78, 0.52];
    /// High-pressure sodium (SON).
    pub const SODIUM: [f32; 3] = [1.0, 0.52, 0.16];
    /// Runway / approach white (incandescent-ish halogen).
    pub const RWY_WHITE: [f32; 3] = [1.0, 0.9, 0.75];
    pub const RWY_YELLOW: [f32; 3] = [1.0, 0.72, 0.12];
    pub const RWY_RED: [f32; 3] = [1.0, 0.06, 0.03];
    pub const RWY_GREEN: [f32; 3] = [0.15, 1.0, 0.4];
    pub const TAXI_BLUE: [f32; 3] = [0.12, 0.3, 1.0];
    pub const TAXI_GREEN: [f32; 3] = [0.2, 1.0, 0.45];
    pub const OBST_RED: [f32; 3] = [1.0, 0.05, 0.02];
    pub const FLOOD: [f32; 3] = [1.0, 0.86, 0.62];
}

/// Known steep-approach aerodromes: (lat, lon, PAPI angle in degrees).
const STEEP: &[(f64, f64, f32)] = &[
    (51.5048, 0.0553, 5.5),  // London City
    (46.0040, 8.9106, 6.65), // Lugano
    (36.1512, -5.3497, 3.5), // Gibraltar
    (43.7710, 7.0193, 3.5),  // Cannes-Mandelieu
];

/// Airport lighting for the world's runways and airports (world frame). PAPI
/// angles: 3°, a known steep approach (London City 5.5°), or — on shorter
/// runways — whatever clears the obstacles under the approach (≤ 4.5°).
pub fn airport_lights_for(world: &crate::world::World) -> Vec<LightInst> {
    let sites = place::beacon_sites(&world.runways, &world.airports.iter().map(|a| a.pos).collect::<Vec<_>>());
    place::airport_lights(&world.runways, &sites, |rw, reverse| {
        let len = rw.length();
        let mid = world.proj.to_latlon((rw.a + rw.b) * 0.5);
        let known = STEEP.iter().find(|(la, lo, _)| (mid.0 - la).abs() < 0.02 && (mid.1 - lo).abs() < 0.03).map(|s| s.2);
        let g = match known {
            Some(g) => g,
            None if len >= 2400.0 => 3.0,
            None => {
                // Obstacle-clear angle with a 30 m margin in a narrow funnel, + 0.5°.
                let end = crate::bot::RunwayEnd::of(rw, reverse);
                let aim = if len > 1500.0 { 300.0 } else { (len * 0.17).max(150.0) };
                let mut need = 0.0f32;
                let mut s = 200.0;
                while s <= 6000.0 {
                    let half = 60.0 + 0.12 * s;
                    let mut l = -half;
                    while l <= half {
                        if let Some(top) = crate::flight::Environment::obstacle_top(world, end.thr - end.dir * s + end.right() * l) {
                            need = need.max(((top + 30.0) / (s + aim)).atan().to_degrees());
                        }
                        l += 40.0;
                    }
                    s += 50.0;
                }
                (((need + 0.5) * 2.0).ceil() / 2.0).clamp(3.0, 4.5)
            }
        };
        if std::env::var_os("SKYWARD_NIGHT_DEBUG").is_some() {
            eprintln!("runway {} {:?} -> {:?} w {} len {:.0}: PAPI {:.2} deg (end {})", rw.name, rw.a, rw.b, rw.width, len, g, if reverse { "b" } else { "a" });
        }
        g.to_radians()
    })
}
