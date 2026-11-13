//! What the game thread tells the audio thread: one [`Frame`] of continuous
//! state per video frame plus discrete [`Event`]s. Plain `Copy` data.

use glam::{Quat, Vec3};

pub const MAX_ENG: usize = 4;

/// Sound family of the current aircraft (picked from the aircraft id, with a
/// fallback on the engine kind).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Family {
    #[default]
    Glider,
    /// Light single (flat-four, two-blade prop, reed stall horn).
    LightPiston,
    /// V12 warbird (four-blade prop, supercharger whine, exhaust crackle).
    Warbird,
    /// Twin turboprop regional (six-blade props, beta / reverse).
    Turboprop,
    /// Afterburning low-bypass fighter.
    Fighter,
    /// High-bypass twin-jet airliner.
    Airliner,
    /// Turboshaft helicopter: main rotor (thump / blade slap), shrouded
    /// tail rotor, gas-generator whine, main gearbox (see `rotorcraft.rs`).
    Helicopter,
    /// Electric multirotor: one motor + prop whine per arm (`rotorcraft.rs`).
    Drone,
}

/// Rotorcraft state (from `Aircraft::rotor.out`), zero for fixed wing.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotorFrame {
    /// Main rotor speed, rpm, and as a fraction of nominal.
    pub nr_rpm: f32,
    pub nr: f32,
    /// Gas generator Ng (fraction of 100 %), engine torque (fraction), shaft
    /// power / power available.
    pub ng: f32,
    pub torque: f32,
    pub power: f32,
    /// Tail rotor rpm and blade pitch (deg).
    pub tail_rpm: f32,
    pub tail_pitch: f32,
    /// Blade-vortex-interaction "slap" strength 0..~1.2 (shallow descents,
    /// decelerations, tight turns — derived in `capture.rs`).
    pub slap: f32,
    /// Main-rotor thrust / weight.
    pub load: f32,
    /// Vortex-ring / prop-wash roughness 0..1.
    pub vrs: f32,
    /// Retreating-blade stall 0..1.
    pub stall: f32,
    /// Main rotor azimuth (turns, 0..1) for blade-locked phase.
    pub azimuth: f32,
}

/// Recorded engine sound set (see `soundsets.rs`); `None` = procedural only.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SoundSet {
    #[default]
    None,
    /// Lycoming flat-four (Cessna 172P recordings).
    C172,
    /// Rolls-Royce Merlin V12 (Spitfire IX recordings).
    Merlin,
    /// Afterburning F110 / F100 (F-16 recordings).
    F16,
    /// PW100-series turboprop (DHC-8 recordings).
    Atr,
    /// CFM56-5B (A320 SASA recordings).
    Cfm56,
    /// IAE V2500 (A320 SASA recordings).
    V2500,
    /// Bristol Siddeley Olympus 301 turbojets (FGUK Vulcan recordings): the howl.
    Olympus,
}

/// Static per-type sound parameters (sent with every frame; tiny).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Profile {
    pub family: Family,
    /// Distinguishes aircraft of the same family (voice re-init on change).
    pub type_id: u32,
    pub engines: usize,
    pub cylinders: u32,
    pub blades: u32,
    /// Fan blades (jets).
    pub fan_blades: u32,
    /// Prop RPM per gauge RPM.
    pub prop_gear: f32,
    pub prop_radius: f32,
    /// Gauge RPM at 100 % N1 (jets).
    pub n1_rpm: f32,
    /// Max thrust per engine (N, dry) for normalising.
    pub thrust_max: f32,
    /// Size of the airframe (m): reference distance for the exterior mix.
    pub size: f32,
    pub retract_gear: bool,
    /// Recorded engine sound set.
    pub set: SoundSet,
    /// Ground-idle N1 fraction (jets) — to derive N2.
    pub n1_idle: f32,
    /// Gross mass, kg (impact energy).
    pub mass: f32,
}

/// Live state of one engine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EngineFrame {
    /// World position of the engine (prop hub / fan / nozzle).
    pub pos: Vec3,
    /// Gauge RPM (piston crank, turboprop Np, jet N1 rpm).
    pub rpm: f32,
    /// Power fraction (props) or N1 fraction (jets).
    pub spool: f32,
    /// Thrust / max dry thrust (can be negative in reverse).
    pub thrust: f32,
    /// Afterburner 0..1.
    pub ab: f32,
    pub running: bool,
    /// Throttle lever 0..1 (for crackle on chop).
    pub throttle: f32,
}

/// Continuous state, sampled once per video frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Frame {
    /// False while loading / no aircraft: the world sounds fade out.
    pub active: bool,
    /// Sim paused (P) — world sounds fade out.
    pub paused: bool,
    /// Menu open over a running sim — world sounds duck and muffle.
    pub menu: bool,
    pub warp: f32,
    pub profile: Profile,

    // --- listener (camera) ---
    pub ear_pos: Vec3,
    pub ear_vel: Vec3,
    pub ear_right: Vec3,
    pub ear_up: Vec3,
    pub ear_fwd: Vec3,
    /// 1 = cockpit view, 0 = outside.
    pub interior: f32,
    /// The camera jumped (cut): snap the propagation delays.
    pub cut: bool,
    /// Floating-origin shift applied to every world position since the last
    /// frame (listener and sources moved together — not a cut).
    pub shift: Vec3,
    /// Listener height above the ground (ambience), m.
    pub ear_agl: f32,

    // --- aircraft ---
    pub ac_pos: Vec3,
    pub ac_vel: Vec3,
    pub ac_rot: Quat,
    pub eng: [EngineFrame; MAX_ENG],
    /// Rotorcraft: rotor / turboshaft / motor state.
    pub rotor: RotorFrame,
    pub reverser: f32,
    /// True airspeed m/s, Mach, dynamic pressure (Pa).
    pub tas: f32,
    pub mach: f32,
    pub qbar: f32,
    /// Sideslip, rad.
    pub beta: f32,
    /// 0..1 aerodynamic buffet (stall approach, Mach, high g).
    pub buffet: f32,
    /// 0..1 how far past the never-exceed speed.
    pub overspeed: f32,
    /// Gear 0 = up .. 1 = down; flaps / spoilers 0..1.
    pub gear: f32,
    pub flaps: f32,
    pub spoilers: f32,
    /// Rates of change (per second) for the actuator motor sounds.
    pub gear_rate: f32,
    pub flap_rate: f32,
    pub on_ground: bool,
    pub ground_speed: f32,
    /// On a paved runway (else grass).
    pub paved: bool,
    pub brakes: f32,
    /// Smoothed airframe jolt (turbulence / rough ground), 0..~1.
    pub shake: f32,
    pub crashed: bool,

    // --- warnings (cockpit) ---
    pub stall_warn: bool,
    pub gear_warn: bool,
    /// Glider total-energy vario (m/s), None for powered aircraft.
    pub vario: Option<f32>,
    /// Navaid Morse ident in the headset (nav/): up to 4 letters (0 = none) and its level.
    pub ident: [u8; 4],
    pub ident_gain: f32,

    // --- environment ---
    pub rain: f32,
    pub snow: f32,
    pub wind_surface: f32,
    pub gust: f32,
    pub thunder: bool,
    /// Daylight 0..1 (birds).
    pub day: f32,
    /// City-ness under the listener 0..1.
    pub urban: f32,
}

/// Recorded speech clips (pre-rendered, see assets/audio/CREDITS.md).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Clip {
    Ft2500,
    Ft1000,
    Ft500,
    Ft400,
    Ft300,
    Ft200,
    Ft100,
    Ft50,
    Ft40,
    Ft30,
    Ft20,
    Ft10,
    Retard,
    Minimums,
    ApproachingMinimums,
    HundredAbove,
    Stall,
    PullUp,
    SinkRate,
    TooLowGear,
    TooLowFlaps,
    Terrain,
    BankAngle,
    V1,
    Overspeed,
    EngineFailure,
    EngineFire,
    Warning,
    Caution,
    Smoke,
}

impl Clip {
    pub const ALL: [Clip; 30] = [
        Clip::Ft2500,
        Clip::Ft1000,
        Clip::Ft500,
        Clip::Ft400,
        Clip::Ft300,
        Clip::Ft200,
        Clip::Ft100,
        Clip::Ft50,
        Clip::Ft40,
        Clip::Ft30,
        Clip::Ft20,
        Clip::Ft10,
        Clip::Retard,
        Clip::Minimums,
        Clip::ApproachingMinimums,
        Clip::HundredAbove,
        Clip::Stall,
        Clip::PullUp,
        Clip::SinkRate,
        Clip::TooLowGear,
        Clip::TooLowFlaps,
        Clip::Terrain,
        Clip::BankAngle,
        Clip::V1,
        Clip::Overspeed,
        Clip::EngineFailure,
        Clip::EngineFire,
        Clip::Warning,
        Clip::Caution,
        Clip::Smoke,
    ];
    pub fn index(self) -> usize {
        Clip::ALL.iter().position(|c| *c == self).unwrap_or(0)
    }
}

/// What something hit or slides on (impact / scrape timbre).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Surface {
    #[default]
    Ground,
    Paved,
    Water,
    Building,
    Metal,
}

/// One-shot sounds. (`Impact` / `Scrape` / `Separation` / `Fire` /
/// `Explosion` / `Boom` are for debris physics and external effects; the
/// sim does not raise them yet.)
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    /// Gear began moving (door / uplock release).
    GearStart { down: bool },
    /// Gear locked (down and locked, or stowed).
    GearLock { down: bool },
    /// Wheels touched: sink rate (m/s, positive down) and ground speed (m/s).
    Touchdown { sink: f32, speed: f32 },
    /// Airframe jolt from turbulence / a gust (0..1).
    Thump(f32),
    /// The aircraft was destroyed: layered impact by type and energy
    /// (see `crash.rs`). `vel` = velocity just before (m/s, world),
    /// `mass` kg, `in_air` = broke up in flight (overstress / overspeed).
    Crash { speed: f32, vel: Vec3, mass: f32, surface: Surface, in_air: bool },
    /// Positional impact of the airframe or a piece of it (for debris
    /// physics): world position, closing speed (m/s), mass (kg).
    Impact { pos: Vec3, speed: f32, mass: f32, surface: Surface },
    /// A body sliding / grinding on a surface. Send every frame while it
    /// slides (same `id`); the sound fades when updates stop. `id` < 16.
    Scrape { id: u8, pos: Vec3, speed: f32, mass: f32, surface: Surface },
    /// A part tore off (wing, gear leg, engine...): metal tearing + bang.
    Separation { pos: Vec3, mass: f32 },
    /// A fire at `pos`: `size` 0..1 (0 = out). Re-send to move / resize;
    /// `id` < 8 (fires are few and long-lived).
    Fire { id: u8, pos: Vec3, size: f32 },
    /// Fuel / ordnance explosion, `energy` 0..1.
    Explosion { pos: Vec3, energy: f32 },
    Speak(Clip),
    /// A radio transmission was submitted (`radio::submit` id): ATC / ATIS / pilot.
    Radio(u32),
    /// A receiver was detuned / lost power (0 = COM 1, 1 = COM 2).
    RadioCut(u8),
    /// Autopilot disconnect warning.
    ApDisconnect,
    MasterCaution,
    /// One second of the continuous repetitive chime (master warning, Airbus / ATR).
    Crc,
    /// One second of the fire bell (Embraer).
    FireBell,
    /// Afterburner light-off on engine i.
    AbLight(usize),
    /// Sonic boom reached the listener (strength 0..1).
    Boom(f32),
    /// Thunder clap at a distance (km).
    Thunder(f32),
    UiClick,
}
