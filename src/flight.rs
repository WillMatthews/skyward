//! Skyward flight dynamics (v2).
//!
//! One data-driven, six-degree-of-freedom fixed-wing model; every aircraft is a
//! parameter set ([`AircraftDef`]).
//!
//! Frames and conventions
//! ----------------------
//! * World: metres, +X east, -Z north, +Y up. Heading 0 = north, 90° = east,
//!   `heading = atan2(f.x, -f.z)` for forward vector `f`.
//! * Body: +Z nose, +Y up, -X right wing. `rot` maps body -> world.
//!   Body yaw for heading `h` is `Quat::from_rotation_y(PI - h)`; nose-up pitch is
//!   a rotation about body -X; right-wing-down roll is a rotation about body +Z.
//! * Internally the angular velocity is kept as a body-frame vector `omega`.
//!   The classic aerodynamic rates are `p = omega.z` (roll, right wing down +),
//!   `q = -omega.x` (pitch, nose up +), `r = -omega.y` (yaw, nose right +).
//!
//! Model summary
//! -------------
//! * Atmosphere: ISA (troposphere + lower stratosphere). Aerodynamic forces use
//!   the local density; [`Aircraft::airspeed`] is the *indicated* (calibrated)
//!   airspeed from the pitot-static impact pressure, [`Aircraft::tas`] the true
//!   airspeed. All V-speeds in [`AircraftDef`] are indicated.
//! * Lift: linear lift curve blended (logistic, both signs of alpha) into a
//!   flat-plate `sin 2α` curve past the stall, so the full ±180° range is defined.
//!   Flaps shift the curve up and move the stall; on gliders the "flap" lever
//!   drives the airbrakes. Spoilers dump lift and add drag.
//! * Drag: parasitic + induced (with ground effect) + post-stall flat-plate +
//!   sideslip + flap/airbrake + landing gear + spoilers + transonic wave drag
//!   (Korn-style rise from the drag-divergence Mach, supersonic decay).
//! * Moments: static stability in pitch (sin α) and yaw (weathercock, sin β),
//!   dihedral effect, roll/pitch/yaw rate damping (integrated implicitly, so it
//!   is unconditionally stable), roll-damping reversal near the stall (wing
//!   drop / autorotation), adverse yaw, pitch break at the stall, quadratic
//!   tumbling damping for post-stall motion. Elevator and rudder see the
//!   propeller slipstream.
//! * Engines (per engine, up to [`MAX_ENGINES`], each at its own body position so
//!   failures give asymmetric thrust in yaw and roll):
//!   - fixed-pitch piston (the trainer, v1 model) and constant-speed piston /
//!     turboprop (momentum-theory static thrust blended into η·P/V), altitude
//!     power lapse with supercharger / flat-rating critical altitude, torque
//!     limit, reaction torque, P-factor, slipstream swirl on the fin and
//!     gyroscopic precession; reverse (beta) pitch on the ground; failed
//!     turboprops autofeather, failed pistons windmill;
//!   - turbofan / turbojet with nonlinear N1 spool dynamics, thrust lapse with
//!     density and Mach, afterburner (throttle > 0.9), thrust reversers
//!     (ground only, spool-dependent).
//! * Control laws: direct (stick = surface) with optional yaw damper; the
//!   fast-jet law (g command, AoA limiter, roll-rate command, turn
//!   coordination) and the airliner law (C*-like g command with flight-path
//!   hold, alpha / bank / load-factor protection) are nonlinear dynamic
//!   inversions of this same aerodynamic model.
//! * Ground: spring/damper contact points with rolling, (differential) braking
//!   and saturating lateral tyre friction, steering that fades with speed
//!   (nose or tail wheel). Retractable gear has a transit time and a
//!   weight-on-wheels interlock; with the gear not locked down the belly / pods
//!   touch first ("Gear-up landing"). Hard points (prop tips, wingtips, canopy,
//!   fin, nacelles) crash on contact; tail bumpers scrape.
//! * Integration: semi-implicit (symplectic) Euler at the caller's fixed step,
//!   quaternion integrated in the body frame and renormalised.

use glam::{Quat, Vec2, Vec3};
use std::f32::consts::{PI, TAU};

/// Rotorcraft step / spawn (inside `flight` for the private airframe state).
#[path = "rotor/glue.rs"]
mod rotor_glue;

/// Track H2a jets: 777-300ER, CRJ700, Citation X, Concorde (src/flight/jets.rs).
mod jets;
/// Track H2b aircraft (Extra 500, Super Cub, Zlin, Otter, DC-3).
mod ga;

/// Gravitational acceleration, m/s².
pub const GRAVITY: f32 = 9.81;
/// Sea-level ISA air density, kg/m³.
pub const AIR_DENSITY: f32 = 1.225;
/// Metres per second in one knot.
pub const KT: f32 = 0.514_444;
/// Metres per second in one foot per minute.
pub const FPM: f32 = 0.00508;
/// Metres in one foot.
pub const FT: f32 = 0.3048;
/// Maximum number of engines on one aircraft.
pub const MAX_ENGINES: usize = 4;

/// Sea-level ISA pressure, Pa.
pub const SEA_LEVEL_PRESSURE: f32 = 101_325.0;
/// Sea-level ISA temperature, K.
pub const SEA_LEVEL_TEMPERATURE: f32 = 288.15;
/// Sea-level ISA speed of sound, m/s.
pub const SEA_LEVEL_SOUND: f32 = 340.294;
const R_AIR: f32 = 287.053;

// ---------------------------------------------------------------------------
// Atmosphere
// ---------------------------------------------------------------------------

/// ISA conditions at one altitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Atmosphere {
    /// K.
    pub temperature: f32,
    /// Pa.
    pub pressure: f32,
    /// kg/m³.
    pub density: f32,
    /// m/s.
    pub speed_of_sound: f32,
}

impl Atmosphere {
    /// Density ratio ρ/ρ₀.
    pub fn sigma(&self) -> f32 {
        self.density / AIR_DENSITY
    }
    /// Outside air temperature, °C.
    pub fn temperature_c(&self) -> f32 {
        self.temperature - 273.15
    }
}

/// International Standard Atmosphere at geometric altitude `alt_m` (metres
/// above mean sea level; clamped to -2 km .. 32 km).
pub fn isa(alt_m: f32) -> Atmosphere {
    let h = finite_or(alt_m, 0.0).clamp(-2_000.0, 32_000.0);
    let (t, p) = if h <= 11_000.0 {
        let t = SEA_LEVEL_TEMPERATURE - 0.0065 * h;
        (t, SEA_LEVEL_PRESSURE * (t / SEA_LEVEL_TEMPERATURE).powf(5.255_88))
    } else {
        (216.65, 22_632.06 * (-(h - 11_000.0) / 6_341.62).exp())
    };
    Atmosphere { temperature: t, pressure: p, density: p / (R_AIR * t), speed_of_sound: (1.4 * R_AIR * t).sqrt() }
}

/// Pitot impact pressure (total − static), Pa, at Mach `m` and static pressure
/// `p` (isentropic below Mach 1, Rayleigh pitot formula above).
fn impact_pressure(p: f32, m: f32) -> f32 {
    let m = m.max(0.0);
    if m <= 1.0 {
        p * ((1.0 + 0.2 * m * m).powf(3.5) - 1.0)
    } else {
        p * (166.921_6 * m.powi(7) / (7.0 * m * m - 1.0).powf(2.5) - 1.0)
    }
}

/// Calibrated airspeed (m/s) that produces impact pressure `qc` at sea level.
fn cas_from_qc(qc: f32) -> f32 {
    let r = qc.max(0.0) / SEA_LEVEL_PRESSURE;
    let sub = SEA_LEVEL_SOUND * (5.0 * ((r + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt();
    if sub <= SEA_LEVEL_SOUND {
        return sub;
    }
    // Supersonic calibrated airspeed: invert the Rayleigh formula.
    let (mut lo, mut hi) = (1.0f32, 6.0f32);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if impact_pressure(SEA_LEVEL_PRESSURE, mid) > qc {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    0.5 * (lo + hi) * SEA_LEVEL_SOUND
}

/// Calibrated (indicated) airspeed for a true airspeed at an altitude, m/s.
pub fn cas_from_tas(tas: f32, alt_m: f32) -> f32 {
    let atm = isa(alt_m);
    cas_from_qc(impact_pressure(atm.pressure, finite_or(tas, 0.0) / atm.speed_of_sound))
}

/// True airspeed for a calibrated (indicated) airspeed at an altitude, m/s.
pub fn tas_from_cas(cas: f32, alt_m: f32) -> f32 {
    let cas = finite_or(cas, 0.0).max(0.0);
    let (mut lo, mut hi) = (0.0f32, 3_000.0f32);
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        if cas_from_tas(mid, alt_m) > cas {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    0.5 * (lo + hi)
}

// ---------------------------------------------------------------------------
// Environment and controls
// ---------------------------------------------------------------------------

/// What the world looks like to the flight model.
pub trait Environment {
    /// Terrain height (world Y) at a horizontal position.
    fn ground_height(&self, xz: Vec2) -> f32;
    /// Roof height (world Y) if `xz` lies inside a building footprint.
    fn obstacle_top(&self, xz: Vec2) -> Option<f32>;
    /// True if the terrain at `xz` is water.
    fn is_water(&self, xz: Vec2) -> bool;
    /// Vertical air velocity at a point, m/s (thermals; + = rising air).
    fn updraft(&self, p: Vec3) -> f32;
    /// Wind at a point (m/s, world frame, the direction the air moves):
    /// steady wind + gusts + turbulence. Thermal lift is `updraft`, not
    /// included here. Default: still air.
    fn wind(&self, _p: Vec3) -> Vec3 {
        Vec3::ZERO
    }
}

/// Pilot inputs. All values are clamped (and NaN-scrubbed) by the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    /// -1..1, +1 = stick back = nose up. (FBW aircraft: load-factor command.)
    pub elevator: f32,
    /// -1..1, +1 = roll right. (FBW aircraft: roll-rate command.)
    pub aileron: f32,
    /// -1..1, +1 = yaw right (also nose/tail-wheel steering and differential
    /// braking on the ground).
    pub rudder: f32,
    /// 0..1. With an afterburner, > 0.9 is the reheat zone. With `reverse`
    /// engaged on the ground it sets the reverse thrust level.
    pub throttle: f32,
    /// 0..1 (on gliders this is the airbrake / spoiler lever).
    pub flaps: f32,
    /// -1..1 elevator trim, +1 = nose up (ignored by the airliner FBW law,
    /// which trims itself; a small g bias on the fast-jet law).
    pub trim: f32,
    /// 0..1 wheel brakes.
    pub brakes: f32,
    /// Landing-gear lever (default down). Ignored for fixed gear; the gear
    /// cannot retract while there is weight on the wheels.
    pub gear_down: bool,
    /// 0..1 speedbrake / spoiler lever. On the ground any setting deploys the
    /// ground spoilers fully (lift dumpers), as does selecting reverse.
    pub spoilers: f32,
    /// Thrust reversers / reverse pitch. Only deploy on the ground; forward
    /// thrust is held at idle while selected.
    pub reverse: bool,
    /// Bit `i` set = engine `i` failed (fuel cut; props windmill or feather).
    pub failed_engines: u8,
    /// Split throttles (Track M): per-engine lever positions 0..1, used only
    /// when `Some` (differential thrust); `None` = every engine on `throttle`.
    pub engine_throttle: Option<[f32; MAX_ENGINES]>,
}

impl Default for Controls {
    fn default() -> Self {
        Controls {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.0,
            flaps: 0.0,
            trim: 0.0,
            brakes: 0.0,
            gear_down: true,
            spoilers: 0.0,
            reverse: false,
            failed_engines: 0,
            engine_throttle: None,
        }
    }
}

fn finite_or(x: f32, d: f32) -> f32 {
    if x.is_finite() { x } else { d }
}

impl Controls {
    fn sanitized(&self) -> Controls {
        Controls {
            elevator: finite_or(self.elevator, 0.0).clamp(-1.0, 1.0),
            aileron: finite_or(self.aileron, 0.0).clamp(-1.0, 1.0),
            rudder: finite_or(self.rudder, 0.0).clamp(-1.0, 1.0),
            throttle: finite_or(self.throttle, 0.0).clamp(0.0, 1.0),
            flaps: finite_or(self.flaps, 0.0).clamp(0.0, 1.0),
            trim: finite_or(self.trim, 0.0).clamp(-1.0, 1.0),
            brakes: finite_or(self.brakes, 0.0).clamp(0.0, 1.0),
            gear_down: self.gear_down,
            spoilers: finite_or(self.spoilers, 0.0).clamp(0.0, 1.0),
            reverse: self.reverse,
            failed_engines: self.failed_engines,
            engine_throttle: self.engine_throttle.map(|t| t.map(|x| finite_or(x, 0.0).clamp(0.0, 1.0))),
        }
    }

    /// Thrust lever of engine `i` (its split lever, else the common one).
    pub fn lever(&self, i: usize) -> f32 {
        match self.engine_throttle {
            Some(t) => t.get(i).copied().unwrap_or(self.throttle),
            None => self.throttle,
        }
    }

    /// Move engine `i`'s lever by `d` (splitting the throttles if they are not yet).
    pub fn nudge_lever(&mut self, i: usize, d: f32) {
        let mut t = self.engine_throttle.unwrap_or([self.throttle; MAX_ENGINES]);
        if let Some(x) = t.get_mut(i) {
            *x = (*x + d).clamp(0.0, 1.0);
        }
        self.engine_throttle = Some(t);
    }
}

/// Smoothed, actual control-surface deflections (for animation / gauges).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Surfaces {
    /// -1..1, same sign as `Controls::aileron` (+ = right aileron up).
    pub aileron: f32,
    /// -1..1, same sign as `Controls::elevator` (+ = trailing edge up), includes trim.
    pub elevator: f32,
    /// -1..1, same sign as `Controls::rudder` (+ = trailing edge right).
    pub rudder: f32,
    /// 0..1 flap (or airbrake) extension.
    pub flaps: f32,
    /// Accumulated propeller angle of engine 0, radians, wrapped to 0..2π.
    pub prop_angle: f32,
    /// Gear *retraction*: 0 = down and locked .. 1 = stowed (inverse of
    /// [`Aircraft::gear_pos`]); always 0 for fixed gear.
    pub gear: f32,
    /// 0..1 spoiler / speedbrake deflection.
    pub spoilers: f32,
    /// 0..1 thrust-reverser deployment.
    pub reverser: f32,
    /// 0..1 afterburner (max over engines).
    pub afterburner: f32,
    /// Propeller angles per engine, radians 0..2π.
    pub prop_angles: [f32; 4],
}

// ---------------------------------------------------------------------------
// Aircraft definition
// ---------------------------------------------------------------------------

/// Broad aircraft class (UI grouping, autopilot gains).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Category {
    #[default]
    Trainer,
    Glider,
    Warbird,
    FastJet,
    Airliner,
    /// Rotorcraft with a main rotor (src/rotor/heli.rs).
    Helicopter,
    /// Multirotor drone (src/rotor/quad.rs).
    Drone,
}

/// Engine type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum EngineKind {
    #[default]
    None,
    Piston,
    Turboprop,
    Jet,
    /// Free-turbine turboshaft driving a rotor (helicopters).
    Turboshaft,
    /// Battery-electric motors (drones).
    Electric,
}

/// Flight-control law.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum ControlLaw {
    /// Stick moves the surfaces (plus an optional yaw damper).
    #[default]
    Direct,
    /// Airliner FBW: stick = load-factor demand, neutral stick holds the
    /// flight path (auto-trim), alpha / bank / g protections.
    Airliner,
    /// Fighter FBW: stick = g demand (pitch-rate demand with the gear down),
    /// AoA and g limiter, roll-rate command, automatic turn coordination.
    FastJet,
}

/// How a ground contact point behaves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ContactKind {
    /// A tyre: rolling resistance along its heading, strong lateral grip.
    Wheel { steerable: bool, braked: bool },
    /// A skid / tail bumper / outrigger: isotropic sliding friction.
    Skid { friction: f32 },
}

/// A sprung ground contact in body coordinates (uncompressed position).
#[derive(Clone, Copy, Debug)]
pub struct GearPoint {
    pub pos: Vec3,
    /// N/m.
    pub spring: f32,
    /// N·s/m.
    pub damping: f32,
    pub kind: ContactKind,
    /// Part of the retractable undercarriage (absent unless down and locked).
    pub retracts: bool,
}

/// A structural point that must not touch the ground (or a building).
#[derive(Clone, Copy, Debug)]
pub struct HardPoint {
    pub pos: Vec3,
    /// Crash reason when it touches the ground.
    pub reason: &'static str,
    /// Touching the ground below this ground speed (m/s) is tolerated as a
    /// scrape (e.g. a glider wingtip at the end of the landing roll).
    pub tolerate_below: f32,
}

/// Parameter set for one aircraft type.
#[derive(Clone, Debug, Default)]
pub struct AircraftDef {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub has_engine: bool,
    /// "Flaps" or "Airbrakes".
    pub flap_label: &'static str,
    /// Lever detents in display units (degrees of flap for powered aircraft,
    /// an arbitrary 0..1 scale for airbrakes). The `Controls::flaps` lever
    /// value for a detent is `detent / last_detent`.
    pub flap_detents: &'static [f32],
    /// Flap deflection in degrees at lever = 1 (0 for airbrakes); for display.
    pub flap_max_deg: f32,
    /// V-speeds for the ASI arcs (knots indicated).
    pub vs0_kt: f32,
    pub vs1_kt: f32,
    pub vno_kt: f32,
    pub vne_kt: f32,
    /// CG height above the ground when at rest on the wheels, m.
    pub gear_height: f32,
    pub wingspan: f32,

    // --- mass ---
    /// kg.
    pub mass: f32,
    /// Moments of inertia, kg·m²: roll (about body Z), pitch (X), yaw (Y).
    pub inertia_roll: f32,
    pub inertia_pitch: f32,
    pub inertia_yaw: f32,

    // --- geometry ---
    /// m².
    pub wing_area: f32,
    /// Mean aerodynamic chord, m.
    pub chord: f32,
    /// Height of the wing above the CG (for ground effect), m.
    pub wing_height: f32,

    // --- lift ---
    /// Lift coefficient at zero body alpha, clean.
    pub cl0: f32,
    /// Lift-curve slope, 1/rad.
    pub cl_alpha: f32,
    /// Stall break centre (positive alpha), rad, clean.
    pub alpha_stall: f32,
    /// Stall break centre for negative alpha (positive number), rad.
    pub alpha_stall_neg: f32,
    /// Sharpness of the stall blend, 1/rad (bigger = more abrupt).
    pub stall_sharpness: f32,
    /// Post-stall flat-plate lift amplitude: CL = cl_flat · sin 2α.
    pub cl_flat: f32,
    /// Lift just past the break as a fraction of the linear CL there.
    pub stall_drop: f32,
    /// Alpha (rad) over which post-stall lift decays onto the flat-plate curve.
    pub stall_decay: f32,

    // --- drag ---
    pub cd0: f32,
    /// Oswald efficiency.
    pub oswald: f32,
    /// Profile-drag bucket: + cd_profile_k · (CL − cl_min_profile)².
    pub cd_profile_k: f32,
    pub cl_min_profile: f32,
    /// Flat-plate drag at 90° alpha.
    pub cd_flat: f32,
    /// Sideslip drag: + cd_beta · sin²β.
    pub cd_beta: f32,

    // --- flaps / airbrakes (all at lever = 1) ---
    /// Lift increment (negative for spoilers).
    pub flap_cl: f32,
    /// Shape of the lift increment vs. lever: frac = f·(1 + curve·(1−f)).
    pub flap_curve: f32,
    /// Change of stall alpha, rad.
    pub flap_alpha_stall: f32,
    /// Drag increment, scaled by lever^flap_cd_exp.
    pub flap_cd: f32,
    pub flap_cd_exp: f32,
    /// Pitching-moment increment.
    pub flap_cm: f32,

    // --- pitch ---
    pub cm0: f32,
    /// Static stability, per rad (negative = stable).
    pub cm_alpha: f32,
    /// Pitch damping, per (q·c / 2V).
    pub cm_q: f32,
    /// Pitching moment at full elevator.
    pub cm_elevator: f32,
    /// Extra nose-down moment once stalled.
    pub cm_stall: f32,
    /// Elevator travel commanded by full trim (fraction of full elevator).
    pub trim_authority: f32,

    // --- lateral / directional ---
    /// Side force per rad of sideslip (negative).
    pub cy_beta: f32,
    /// Dihedral effect (negative = stable).
    pub cl_beta: f32,
    /// Roll damping per (p·b / 2V) (negative).
    pub cl_p: f32,
    /// Fully stalled roll damping as a fraction of `-cl_p` (autorotation strength).
    pub stall_autorotation: f32,
    /// Roll due to yaw rate per (r·b/2V), multiplied by CL.
    pub cl_r_per_cl: f32,
    /// Rolling moment at full aileron.
    pub cl_aileron: f32,
    /// Weathercock stability (positive).
    pub cn_beta: f32,
    /// Yaw damping per (r·b / 2V) (negative).
    pub cn_r: f32,
    /// Yaw due to roll rate per (p·b/2V) (negative = adverse).
    pub cn_p: f32,
    /// Yawing moment at full rudder.
    pub cn_rudder: f32,
    /// Adverse yaw from full aileron (negative).
    pub cn_aileron: f32,

    // --- control actuation ---
    /// Control surface slew rate, full travel per second.
    pub surface_rate: f32,
    /// Flap lever slew rate, 0..1 per second.
    pub flap_rate: f32,

    // --- engine / propeller (v1 fields; totals over all engines) ---
    /// Static thrust at full power (sea level), N, all engines. For jets the
    /// dry (military) rating.
    pub static_thrust: f32,
    /// Fixed-pitch propeller: full-power thrust loss per m/s and per (m/s)².
    pub thrust_v1: f32,
    pub thrust_v2: f32,
    /// Windmilling drag coefficient (on the prop disk / fan face) at idle.
    pub windmill_cd: f32,
    /// Propeller disk (or fan face) area, m², all engines.
    pub prop_disk_area: f32,
    /// Fraction of the slipstream dynamic pressure rise felt by the tail.
    pub prop_wash: f32,
    /// Engine power fraction at idle throttle (props).
    pub idle_power: f32,
    /// Engine spool time constant, s (props); idle → max time, s (jets).
    pub spool_time: f32,
    /// Gauge RPM at idle and at full static power (governed RPM for
    /// constant-speed props).
    pub rpm_idle: f32,
    pub rpm_static: f32,
    /// Extra RPM per m/s of forward airspeed (fixed-pitch prop unloading;
    /// windmilling).
    pub rpm_per_ms: f32,

    // --- ground ---
    pub gear: &'static [GearPoint],
    pub hard_points: &'static [HardPoint],
    /// Rolling resistance coefficient.
    pub roll_mu: f32,
    /// Braking friction coefficient at full brakes.
    pub brake_mu: f32,
    /// Lateral tyre friction coefficient.
    pub tire_mu: f32,
    /// Nose (or tail) wheel steering angle at full rudder, low speed, degrees.
    pub nose_steer_deg: f32,

    // --- limits ---
    /// Max survivable sink rate at touchdown, m/s.
    pub max_touchdown_sink: f32,
    /// Max bank at touchdown, rad.
    pub max_touchdown_bank: f32,
    /// Minimum pitch attitude at touchdown, rad (below = nose-wheel first).
    pub min_touchdown_pitch: f32,
    /// Structural load-factor limits (failure beyond these).
    pub g_limit_pos: f32,
    pub g_limit_neg: f32,

    // ================= v2 =================
    pub category: Category,
    pub engine_kind: EngineKind,
    /// Number of engines (= `engine_pos.len()`, at most [`MAX_ENGINES`]).
    pub engine_count: usize,
    pub retractable_gear: bool,
    pub has_spoilers: bool,
    pub has_reverser: bool,
    pub has_afterburner: bool,
    pub taildragger: bool,
    /// Cockpit eye point (body frame, m).
    pub eye: Vec3,
    /// Rotation speed, approach reference speed, max gear-extended speed (kt IAS).
    pub vr_kt: f32,
    pub vref_kt: f32,
    pub vle_kt: f32,
    /// Published service ceiling, m.
    pub ceiling_m: f32,
    /// Maximum operating Mach (0 = not applicable).
    pub mmo: f32,
    pub control_law: ControlLaw,

    // --- gear / drag devices ---
    /// Gear transit time, s.
    pub gear_time: f32,
    /// Drag coefficient of the extended undercarriage (retractable only).
    pub cd_gear: f32,
    /// Lift fraction dumped by fully deflected spoilers.
    pub spoiler_cl: f32,
    /// Drag increment of fully deflected spoilers / speedbrake.
    pub spoiler_cd: f32,
    /// In-flight spoiler deflection at full lever (ground spoilers go to 1).
    pub spoiler_flight_max: f32,
    /// Drag-divergence Mach (0 = no compressibility drag).
    pub mach_dd: f32,
    /// Peak transonic wave-drag increment (≈ Mach 1.08).
    pub cd_wave: f32,

    // --- engines ---
    /// Thrust application point of each engine (prop hub / fan / nozzle).
    pub engine_pos: &'static [Vec3],
    /// Rated shaft power per engine, W (props).
    pub engine_power_w: f32,
    /// Constant-speed propeller (otherwise the v1 fixed-pitch model).
    pub const_speed_prop: bool,
    /// Propeller efficiency in cruise and static figure of merit.
    pub prop_eta: f32,
    pub prop_fm: f32,
    pub prop_radius: f32,
    /// +1 = clockwise seen from behind (swings left), -1 = anticlockwise.
    pub prop_rotation: f32,
    /// Propeller RPM per gauge RPM (reduction gearbox).
    pub prop_gear: f32,
    /// Propeller polar moment of inertia, kg·m² (gyroscopic precession).
    pub prop_inertia: f32,
    /// Power is held to this altitude (supercharger / flat rating), m.
    pub crit_alt_m: f32,
    /// Above it power falls as (σ/σ_crit)^lapse_exp.
    pub lapse_exp: f32,
    /// Multipliers for reaction torque (1 = physical), slipstream swirl on the
    /// fin, and P-factor (thrust-centre shift per rad of inflow angle, as a
    /// fraction of the prop radius).
    pub torque_k: f32,
    pub swirl_k: f32,
    pub pfactor_k: f32,
    /// Built-in rigging (offset fin / aileron trim strip) as rolling and
    /// yawing moment coefficients, computed by [`catalogue`] to cancel the
    /// propeller effects in level flight at `rig_kt` (kt IAS, 1,500 m).
    pub cl_rig: f32,
    pub rig_kt: f32,
    pub cn_rig: f32,
    /// Jets: idle N1 (0..1) and thrust fraction at idle.
    pub n1_idle: f32,
    pub idle_thrust_frac: f32,
    /// Jets: dry thrust Mach factor `1 + a·M + b·M²`.
    pub thrust_mach: [f32; 2],
    /// Extra afterburner thrust, sea-level static, all engines, N.
    pub ab_static_thrust: f32,
    /// Afterburner thrust Mach factor `1 + a·M + b·M²` (ram effect).
    pub ab_mach: [f32; 2],
    /// Engine/inlet total-temperature limit, K (0 = none): thrust fades
    /// away as the ram total temperature passes it, which caps the fast jet
    /// near Mach 1.3 at sea level and Mach 2 in the stratosphere.
    pub t_total_max: f32,
    /// Gauge RPM at 100 % N1 (jets).
    pub n1_rpm: f32,
    /// Specific fuel consumption: kg/(kW·h) for props, kg/(N·h) for jets (dry).
    pub sfc: f32,
    /// Afterburner specific fuel consumption, kg/(N·h).
    pub sfc_ab: f32,
    /// EGT / ITT at idle and at maximum power, °C (ISA sea level).
    pub egt_idle: f32,
    pub egt_max: f32,
    /// Reverse thrust as a fraction of the forward gross thrust.
    pub reverse_eff: f32,

    // --- FBW / augmentation ---
    /// FBW angle-of-attack limit, rad.
    pub alpha_max: f32,
    /// FBW load-factor limits (clean).
    pub nz_max: f32,
    pub nz_min: f32,
    /// FBW maximum commanded roll rate, rad/s.
    pub roll_rate_max: f32,
    /// Yaw-damper gain (rudder per rad/s of non-turning yaw rate), 0 = none.
    pub yaw_damper: f32,

    // ================= real aircraft (v3) =================
    /// JSBSim aerodynamics (FlightGear FDM) replacing the parametric
    /// lift/drag/moment model at run time. The parametric fields above are
    /// then fitted to it ([`JsbDef::fit`]) for the analytic helpers (trim,
    /// stall speed, FBW inversion feed-forward).
    pub jsb: Option<std::sync::Arc<JsbDef>>,

    // ================= v3 heavies =================
    /// Display class for menus ("" = the category's name).
    pub role: &'static str,
    /// Runway length the type needs, m (0 = the category default, see
    /// `sim::min_runway`).
    pub min_runway_m: f32,
    /// Can use unpaved / grass strips (STOL / rough-field gear).
    pub rough_field: bool,

    // ================= v3 rotorcraft =================
    /// Rotorcraft flight model (src/rotor): replaces the fixed-wing engines,
    /// control law and aerodynamics in [`Aircraft::step`].
    pub rotor: Option<std::sync::Arc<crate::rotor::RotorDef>>,

    // ================= v3 H2a jets =================
    /// Automatic trim-fuel transfer (Concorde): CG travel aft of the loaded
    /// CG scheduled on Mach, (Mach, metres aft); empty = none.
    pub cg_schedule: &'static [(f32, f32)],
    /// Fuel-transfer CG travel rate, m/s.
    pub cg_rate: f32,
    // ================= v3 GA / props (Track H2b) =================
    /// Floats / hull (src/water.rs): non-empty = a floatplane that operates
    /// from water (its `gear` are the float keels as skids on land).
    pub hulls: &'static [crate::water::Hull],
    /// Airshow smoke system (the T key toggles the smoke trail, src/vfx/smoke.rs).
    pub smoke_system: bool,
}

impl AircraftDef {
    /// A floatplane (operates from water lanes, src/water.rs).
    pub fn on_water(&self) -> bool {
        !self.hulls.is_empty()
    }
}

/// A JSBSim aerodynamic model plus how Skyward's normalised surfaces map onto
/// its deflection properties.
pub struct JsbDef {
    pub aero: crate::jsbsim::JsbAero,
    /// Elevator travel, rad: trailing edge up (stick back) / down.
    pub elev_up: f32,
    pub elev_down: f32,
    /// Aileron travel, rad: up / down.
    pub ail_up: f32,
    pub ail_down: f32,
    pub rudder_max: f32,
    /// Flap deflection at lever 1, degrees.
    pub flap_max_deg: f32,
    /// Slat schedule vs. flap lever (lever, slat 0..1), and slat degrees at 1.
    pub slats: &'static [(f32, f32)],
    pub slat_max_deg: f32,
    /// Spoiler / speedbrake degrees at 1.
    pub spoiler_max_deg: f32,
    /// F-16 leading-edge flap schedule.
    pub lef: bool,
    /// Gliders: the flap lever drives the airbrakes (speedbrake property).
    pub flaps_are_airbrakes: bool,
    /// Sign corrections found at load time so that +1 elevator pitches up,
    /// +1 aileron rolls right and +1 rudder yaws right.
    pub sign_e: f32,
    pub sign_a: f32,
    pub sign_r: f32,
    /// Lift peak vs. flap lever: (lever, alpha rad, CLmax), filled by [`JsbDef::fit`].
    pub clmax: Vec<(f32, f32, f32)>,
}

impl std::fmt::Debug for JsbDef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "JsbDef {{ S {:.1} m², b {:.2} m, {} properties }}", self.aero.sw_m2, self.aero.bw_m, self.aero.n_slots)
    }
}

impl AircraftDef {
    /// Trim-fuel CG travel aft (m) the transfer schedule sets at Mach `m`
    /// (0 without one).
    pub fn cg_scheduled(&self, m: f32) -> f32 {
        if self.cg_schedule.is_empty() { 0.0 } else { interp_table(self.cg_schedule, m) }
    }

    pub fn aspect_ratio(&self) -> f32 {
        self.wingspan * self.wingspan / self.wing_area
    }

    fn induced_k(&self) -> f32 {
        1.0 / (PI * self.oswald * self.aspect_ratio())
    }

    fn flap_lift_frac(&self, f: f32) -> f32 {
        f * (1.0 + self.flap_curve * (1.0 - f))
    }

    fn engines(&self) -> usize {
        self.engine_count.min(self.engine_pos.len()).min(MAX_ENGINES)
    }

    /// Lift and drag coefficients at body angle of attack `alpha` (rad) with the
    /// flap/airbrake lever at `flap` (0..1). `ge` scales induced drag
    /// (1 = out of ground effect). Clean: no gear, spoiler or Mach terms.
    pub fn lift_drag(&self, alpha: f32, flap: f32, ge: f32) -> LiftDrag {
        let fl = self.flap_lift_frac(flap);
        let a0p = self.alpha_stall + self.flap_alpha_stall * fl;
        let a0n = self.alpha_stall_neg;
        let m = self.stall_sharpness;
        let sp = logistic(m * (alpha - a0p));
        let sn = logistic(m * (-alpha - a0n));
        let s = (sp + sn).min(1.0);
        let lin = |a: f32| self.cl0 + self.flap_cl * fl + self.cl_alpha * a;
        let flat = |a: f32| self.cl_flat * (2.0 * a).sin();
        // Post-stall: starts at a fraction of the stall CL and decays onto the
        // flat-plate curve as alpha grows past the break.
        let post = |a: f32, a0: f32, beyond: f32| {
            flat(a) + (self.stall_drop * lin(a0) - flat(a0)) * (-beyond.max(0.0) / self.stall_decay).exp()
        };
        let cl_pos = post(alpha, a0p, alpha - a0p);
        let cl_neg = post(alpha, -a0n, -alpha - a0n);
        let (wp, wn) = if sp + sn > 1.0 { (sp / (sp + sn), sn / (sp + sn)) } else { (sp, sn) };
        let cl = (1.0 - s) * lin(alpha) + wp * cl_pos + wn * cl_neg;
        let sa = alpha.sin();
        let cd = self.cd0
            + self.flap_cd * flap.max(0.0).powf(self.flap_cd_exp)
            + ge * self.induced_k() * cl * cl
            + (1.0 - s) * self.cd_profile_k * (cl - self.cl_min_profile).powi(2)
            + s * self.cd_flat * sa * sa;
        LiftDrag { cl, cd, stall_pos: sp, stall: s }
    }

    /// Transonic / supersonic wave-drag increment at Mach `m`.
    pub fn wave_drag(&self, m: f32) -> f32 {
        if self.mach_dd <= 0.0 || self.cd_wave <= 0.0 {
            return 0.0;
        }
        let mdd = self.mach_dd;
        let mc = mdd - 0.108; // Korn: 20 (M - Mcrit)^4 = 0.002 at Mdd
        const M_PEAK: f32 = 1.08;
        let at_dd = 0.002f32.min(self.cd_wave);
        if m <= mc {
            0.0
        } else if m <= mdd {
            (20.0 * (m - mc).powi(4)).min(self.cd_wave)
        } else if m <= M_PEAK {
            let t = ((m - mdd) / (M_PEAK - mdd).max(0.01)).clamp(0.0, 1.0);
            at_dd + (self.cd_wave - at_dd) * (t * PI * 0.5).sin()
        } else {
            self.cd_wave * ((M_PEAK * M_PEAK - 1.0) / (m * m - 1.0)).powf(0.12)
        }
    }

    /// Angle of attack (rad) at which CL peaks for the given flap setting.
    pub fn alpha_peak(&self, flap: f32) -> f32 {
        if let Some(j) = &self.jsb {
            if !j.clmax.is_empty() {
                return interp_table(&j.clmax.iter().map(|c| (c.0, c.1)).collect::<Vec<_>>(), flap);
            }
        }
        let fl = self.flap_lift_frac(flap);
        let centre = self.alpha_stall + self.flap_alpha_stall * fl;
        let mut best = (f32::MIN, centre);
        let mut a = centre - 0.2;
        while a < centre + 0.05 {
            let cl = self.lift_drag(a, flap, 1.0).cl;
            if cl > best.0 {
                best = (cl, a);
            }
            a += 0.001;
        }
        best.1
    }

    /// Maximum lift coefficient for the given flap setting.
    pub fn cl_max(&self, flap: f32) -> f32 {
        self.lift_drag(self.alpha_peak(flap), flap, 1.0).cl
    }

    /// Highest usable angle of attack: the CL peak, or the FBW AoA limit.
    pub fn alpha_usable(&self, flap: f32) -> f32 {
        let a = self.alpha_peak(flap);
        if self.control_law == ControlLaw::FastJet { a.min(self.alpha_max) } else { a }
    }

    /// 1-g stall speed (m/s, indicated) from CLmax, ignoring thrust. For the
    /// fast jet: the 1-g speed at the FBW AoA limit.
    pub fn stall_speed(&self, flap: f32) -> f32 {
        let cl = match &self.jsb {
            Some(j) if !j.clmax.is_empty() => {
                let cl = interp_table(&j.clmax.iter().map(|c| (c.0, c.2)).collect::<Vec<_>>(), flap);
                if self.control_law == ControlLaw::FastJet {
                    // CL at the AoA limit (the lift curve is ~linear up there).
                    let ap = interp_table(&j.clmax.iter().map(|c| (c.0, c.1)).collect::<Vec<_>>(), flap);
                    cl * (self.alpha_max / ap.max(0.1)).min(1.0)
                } else {
                    cl
                }
            }
            _ => self.lift_drag(self.alpha_usable(flap), flap, 1.0).cl,
        };
        (2.0 * self.mass * GRAVITY / (AIR_DENSITY * self.wing_area * cl)).sqrt()
    }

    /// v1 helper: total propulsive thrust (N) along the nose at sea level, true
    /// airspeed `v`, with every engine at power / spool fraction `power`.
    pub fn thrust(&self, v: f32, power: f32) -> f32 {
        if !self.has_engine {
            return 0.0;
        }
        let atm = isa(0.0);
        let n = self.engines().max(1) as f32;
        let spool = match self.engine_kind {
            EngineKind::Jet => self.n1_idle + (1.0 - self.n1_idle) * power.clamp(0.0, 1.0),
            _ => power,
        };
        n * self.engine_output(spool, 0.0, 0.0, v, &atm, false).thrust
    }

    fn power_for_throttle(&self, throttle: f32) -> f32 {
        if self.has_engine {
            self.idle_power + (1.0 - self.idle_power) * throttle.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    /// Steady-state engine spool (power fraction for props, N1 for jets) and
    /// afterburner level for a forward-thrust lever position.
    fn spool_for_throttle(&self, throttle: f32) -> (f32, f32) {
        let t = throttle.clamp(0.0, 1.0);
        match self.engine_kind {
            EngineKind::None | EngineKind::Electric => (0.0, 0.0),
            EngineKind::Piston | EngineKind::Turboprop | EngineKind::Turboshaft => (self.power_for_throttle(t), 0.0),
            EngineKind::Jet => {
                let (dry, ab) = if self.has_afterburner { ((t / 0.9).min(1.0), ((t - 0.9) / 0.1).clamp(0.0, 1.0)) } else { (t, 0.0) };
                (self.n1_idle + (1.0 - self.n1_idle) * dry.powf(1.0 / 1.8), ab)
            }
        }
    }

    /// Shaft-power lapse with altitude (piston / turboprop).
    fn power_lapse(&self, sigma: f32) -> f32 {
        let sc = isa(self.crit_alt_m.max(0.0)).sigma();
        if sigma >= sc { 1.0 } else { (sigma / sc).max(0.0).powf(self.lapse_exp.max(0.1)) }
    }

    /// Jet thrust density/Mach lapse, with the total-temperature limit.
    fn jet_lapse(&self, atm: &Atmosphere, mach: f32, coeffs: [f32; 2]) -> f32 {
        let m = mach.max(0.0);
        let limit = if self.t_total_max > 0.0 {
            let tt = atm.temperature * (1.0 + 0.2 * m * m);
            let x = ((tt - (self.t_total_max - 30.0)) / 45.0).clamp(0.0, 1.0);
            1.0 - x * x * (3.0 - 2.0 * x)
        } else {
            1.0
        };
        atm.sigma().max(0.0).powf(0.75) * (1.0 + coeffs[0] * m + coeffs[1] * m * m).max(0.05) * limit
    }

    /// Output of one engine at spool state `spool` (power fraction / N1),
    /// afterburner `ab`, reverser deployment `rev`, forward true airspeed
    /// `v`, in the given atmosphere.
    fn engine_output(&self, spool: f32, ab: f32, rev: f32, v: f32, atm: &Atmosphere, feathered: bool) -> EngineOut {
        let n = self.engines().max(1) as f32;
        let v = v.max(0.0);
        let rho = atm.density;
        let sigma = atm.sigma();
        let qbar = 0.5 * rho * v * v;
        let disk = self.prop_disk_area / n;
        match self.engine_kind {
            EngineKind::None | EngineKind::Turboshaft | EngineKind::Electric => EngineOut::default(),
            EngineKind::Piston | EngineKind::Turboprop => {
                let lapse = self.power_lapse(sigma);
                let p = spool.clamp(0.0, 1.0);
                let shaft = p * self.engine_power_w * lapse;
                let wind_k = if feathered { 0.08 } else { 1.0 };
                let windmill = (1.0 - p) * (1.0 - p) * wind_k * self.windmill_cd * qbar * disk;
                let (forward, gross_static) = if !self.const_speed_prop {
                    let st = self.static_thrust / n;
                    let full = (st - self.thrust_v1 / n * v - self.thrust_v2 / n * v * v).max(-0.5 * st) * lapse;
                    (p * full, p * st * lapse)
                } else {
                    let a = PI * self.prop_radius * self.prop_radius;
                    let ts = self.prop_fm * (2.0 * rho * a * shaft * shaft).cbrt();
                    let td = self.prop_eta * shaft / v.max(1.0);
                    let t = if shaft > 1.0 { (ts.powi(-3) + td.powi(-3)).powf(-1.0 / 3.0) } else { 0.0 };
                    (t, ts)
                };
                let fwd = forward - windmill;
                let reverse = -self.reverse_eff * gross_static;
                EngineOut { thrust: fwd * (1.0 - rev) + reverse * rev, gross: forward.max(0.0), shaft_power: shaft, ab_thrust: 0.0 }
            }
            EngineKind::Jet => {
                let mach = v / atm.speed_of_sound;
                let idle = self.n1_idle.max(0.05);
                let n1 = spool.max(0.0);
                let frac = if n1 >= idle {
                    let x = ((n1 - idle) / (1.0 - idle)).max(0.0);
                    self.idle_thrust_frac + (1.0 - self.idle_thrust_frac) * x.powf(1.8)
                } else {
                    self.idle_thrust_frac * (n1 / idle).powi(2)
                };
                let dry = self.static_thrust / n * self.jet_lapse(atm, mach, self.thrust_mach) * frac;
                let wet = ab.clamp(0.0, 1.0) * self.ab_static_thrust / n * self.jet_lapse(atm, mach, self.ab_mach);
                let gross = dry + wet;
                // Ram drag of a dead (windmilling) engine.
                let windmill = (1.0 - (n1 / idle).min(1.0)) * self.windmill_cd * qbar * disk;
                let fwd = gross - windmill;
                let reverse = -self.reverse_eff * dry;
                EngineOut { thrust: fwd * (1.0 - rev) + reverse * rev, gross, shaft_power: 0.0, ab_thrust: wet }
            }
        }
    }

    /// Gauge RPM an engine settles at.
    fn target_rpm(&self, spool: f32, v_fwd: f32, running: bool) -> f32 {
        if !self.has_engine {
            return 0.0;
        }
        let v = v_fwd.max(0.0);
        match self.engine_kind {
            EngineKind::None | EngineKind::Turboshaft | EngineKind::Electric => 0.0,
            EngineKind::Jet => spool * self.n1_rpm,
            _ => {
                if !running {
                    // Windmilling (or feathered: turboprops autofeather).
                    return if self.engine_kind == EngineKind::Turboprop { 0.0 } else { v * self.rpm_per_ms.max(4.0) };
                }
                let pn = ((spool - self.idle_power) / (1.0 - self.idle_power)).clamp(0.0, 1.0);
                if self.const_speed_prop {
                    // The governor holds RPM once there is enough power or airflow.
                    let g = (2.5 * pn + v / 45.0).min(1.0);
                    self.rpm_idle + (self.rpm_static - self.rpm_idle) * g
                } else {
                    self.rpm_idle + (self.rpm_static - self.rpm_idle) * pn + v * self.rpm_per_ms
                }
            }
        }
    }

    fn slipstream_q(&self, thrust: f32) -> f32 {
        if self.has_engine && self.prop_disk_area > 0.0 && self.prop_wash > 0.0 {
            self.prop_wash * thrust.max(0.0) / self.prop_disk_area
        } else {
            0.0
        }
    }

    /// Reaction-torque roll and swirl/P-factor yaw (N·m, roll right +, yaw
    /// right +) of one propeller.
    fn prop_moments(&self, out: &EngineOut, prop_rpm: f32, v_fwd: f32, vb: Vec3, rho: f32) -> (f32, f32) {
        if !matches!(self.engine_kind, EngineKind::Piston | EngineKind::Turboprop) || self.prop_radius <= 0.0 {
            return (0.0, 0.0);
        }
        let w = (prop_rpm * TAU / 60.0).max(20.0);
        let q_shaft = out.shaft_power / w;
        let rot = self.prop_rotation;
        let roll = -rot * self.torque_k * q_shaft;
        // Swirl: strongest at high power and low speed.
        let a = PI * self.prop_radius * self.prop_radius;
        let q_slip = out.thrust.max(0.0) / a;
        let qd = 0.5 * rho * v_fwd.max(0.0).powi(2);
        let swirl = -rot * self.swirl_k * q_shaft * q_slip / (q_slip + qd + 1.0);
        // P-factor: inflow angle at the disc (induced velocity keeps it zero at rest).
        let vi = (out.thrust.max(0.0) / (2.0 * rho * a)).sqrt();
        let inflow = (-vb.y).atan2(vb.z.max(0.0) + vi + 0.1);
        let pf = -rot * self.pfactor_k * out.thrust.max(0.0) * inflow * self.prop_radius;
        (roll, swirl + pf)
    }

    /// Approximate pitch attitude at rest on the wheels (three-point attitude
    /// for a taildragger, else ~0).
    pub fn rest_pitch(&self) -> f32 {
        let wheels = self.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { .. }));
        let fwd = wheels.clone().max_by(|a, b| a.pos.z.total_cmp(&b.pos.z));
        let aft = wheels.min_by(|a, b| a.pos.z.total_cmp(&b.pos.z));
        match (fwd, aft) {
            (Some(f), Some(r)) if self.taildragger && f.pos.z > r.pos.z => (r.pos.y - f.pos.y).atan2(f.pos.z - r.pos.z),
            _ => 0.0,
        }
    }

    /// Steady straight flight at indicated `speed` (m/s) with the lever at
    /// `flap`, at sea level, clean (gear up if retractable): level flight for
    /// powered aircraft, a steady glide for gliders. If the engines cannot
    /// hold level flight the throttle is clamped to 1 (not a true equilibrium).
    pub fn trim_point(&self, speed: f32, flap: f32) -> TrimPoint {
        self.trim_point_at(speed, flap, 0.0, false)
    }

    /// [`trim_point`](Self::trim_point) at altitude `alt` (m) with the gear
    /// down or up (`gear_down` is ignored for fixed gear).
    pub fn trim_point_at(&self, speed: f32, flap: f32, alt: f32, gear_down: bool) -> TrimPoint {
        if let Some(j) = &self.jsb {
            return self.trim_point_jsb(j, speed, flap, alt, gear_down);
        }
        let w = self.mass * GRAVITY;
        let atm = isa(alt);
        let ias = finite_or(speed, 30.0).max(1.0);
        let tas = tas_from_cas(ias, alt);
        let mach = tas / atm.speed_of_sound;
        let q = 0.5 * atm.density * tas * tas;
        let s = self.wing_area;
        let gear = if self.retractable_gear && gear_down { self.cd_gear } else { 0.0 };
        let extra_cd = gear + self.wave_drag(mach);
        let a_hi = self.alpha_usable(flap);
        let mut gamma = 0.0f32;
        let mut alpha = 0.0f32;
        let mut thrust = 0.0f32;
        for _ in 0..30 {
            // Solve for alpha such that the forces normal to the path balance.
            let need = |a: f32| -> f32 {
                let ld = self.lift_drag(a, flap, 1.0);
                let t = if self.has_engine { q * s * (ld.cd + extra_cd) / a.cos() } else { 0.0 };
                q * s * ld.cl + t * a.sin() - w * gamma.cos()
            };
            let (mut lo, mut hi) = (-0.25f32, a_hi);
            if need(hi) < 0.0 {
                alpha = hi;
            } else {
                for _ in 0..50 {
                    let mid = 0.5 * (lo + hi);
                    if need(mid) > 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                alpha = 0.5 * (lo + hi);
            }
            let ld = self.lift_drag(alpha, flap, 1.0);
            if self.has_engine {
                thrust = q * s * (ld.cd + extra_cd) / alpha.cos();
                break;
            }
            gamma = -((ld.cd + extra_cd) / ld.cl.max(0.05)).atan();
        }
        // Throttle for the required thrust.
        let mut throttle = 0.0;
        let mut power = 0.0;
        if self.has_engine {
            let n = self.engines().max(1) as f32;
            let v_fwd = tas * alpha.cos();
            let total = |thr: f32| {
                let (sp, ab) = self.spool_for_throttle(thr);
                n * self.engine_output(sp, ab, 0.0, v_fwd, &atm, false).thrust
            };
            let (mut lo, mut hi) = (0.0f32, 1.0f32);
            if total(1.0) < thrust {
                lo = 1.0;
            } else if total(0.0) > thrust {
                hi = 0.0;
            } else {
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    if total(mid) > thrust {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
            }
            throttle = 0.5 * (lo + hi);
            power = self.spool_for_throttle(throttle).0;
            thrust = total(throttle);
        }
        // Elevator for zero pitching moment (aero + thrust-line offsets).
        let ld = self.lift_drag(alpha, flap, 1.0);
        let fl = self.flap_lift_frac(flap);
        let cm = self.cm0 + self.cm_alpha * alpha.sin() + self.flap_cm * fl + self.cm_stall * ld.stall_pos;
        let n = self.engines().max(1) as f32;
        let m_thrust: f32 = self.engine_pos.iter().take(self.engines()).map(|p| -p.y * thrust / n).sum();
        let q_tail = q + self.slipstream_q(thrust);
        let elev = -(cm * q * s * self.chord + m_thrust) / (self.cm_elevator * q_tail * s * self.chord);
        let trim = if self.control_law == ControlLaw::Direct { (elev / self.trim_authority).clamp(-1.0, 1.0) } else { 0.0 };
        TrimPoint { alpha, gamma, pitch: alpha + gamma, throttle, power, trim, elevator: elev }
    }
}

impl AircraftDef {
    /// Throttle (and resulting thrust, power) for a total thrust need.
    fn throttle_for(&self, thrust: f32, v_fwd: f32, atm: &Atmosphere) -> (f32, f32, f32) {
        let n = self.engines().max(1) as f32;
        let total = |thr: f32| {
            let (sp, ab) = self.spool_for_throttle(thr);
            n * self.engine_output(sp, ab, 0.0, v_fwd, atm, false).thrust
        };
        let (mut lo, mut hi) = (0.0f32, 1.0f32);
        if total(1.0) < thrust {
            lo = 1.0;
        } else if total(0.0) > thrust {
            hi = 0.0;
        } else {
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if total(mid) > thrust {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
        }
        let throttle = 0.5 * (lo + hi);
        (throttle, total(throttle), self.spool_for_throttle(throttle).0)
    }

    /// Exact trim against the JSBSim aerodynamics: Newton on (alpha,
    /// elevator) for lift = weight and zero pitching moment, thrust balancing
    /// drag (or, for a glider, the glide path balancing drag).
    fn trim_point_jsb(&self, j: &JsbDef, speed: f32, flap: f32, alt: f32, gear_down: bool) -> TrimPoint {
        let w = self.mass * GRAVITY;
        let atm = isa(alt);
        let ias = finite_or(speed, 30.0).max(1.0);
        let tas = tas_from_cas(ias, alt);
        let mach = tas / atm.speed_of_sound;
        let gear = if self.retractable_gear { if gear_down { 1.0 } else { 0.0 } } else { 1.0 };
        let n = self.engines().max(1) as f32;
        // Residuals for (alpha, elevator) at path angle gamma and thrust T.
        let eval = |a: f32, e: f32, thrust: f32| {
            let vb = Vec3::new(0.0, -tas * a.sin(), tas * a.cos());
            let s = JsbSurf { elevator: e, flaps: flap, gear, ..Default::default() };
            let q_slip = self.slipstream_q(thrust);
            let wash = if q_slip > 0.0 { ((tas * tas + 2.0 * q_slip / atm.density).sqrt() - tas).max(0.0) } else { 0.0 };
            let mut i = j.input(vb, Vec3::ZERO, atm.density, mach, &s, 1000.0, 1000.0, wash, atm.pressure);
            // Trim with the trim fuel where its schedule puts it (Concorde).
            i.cg_aft_m = self.cg_scheduled(mach);
            j.aero.eval_steady(&i)
        };
        let (mut a, mut e, mut thrust, mut gamma) = (deg(3.0), 0.0f32, 0.0f32, 0.0f32);
        for _ in 0..25 {
            let o = eval(a, e, thrust);
            let lift_dir = Vec3::new(0.0, a.cos(), a.sin());
            let drag_dir = Vec3::new(0.0, a.sin(), -a.cos());
            let lift = o.force.dot(lift_dir);
            let drag = o.force.dot(drag_dir);
            if self.has_engine {
                thrust = ((drag + w * gamma.sin()) / a.cos()).max(0.0);
            } else {
                gamma = -(drag / lift.max(1.0)).atan();
            }
            let m_thrust: f32 = self.engine_pos.iter().take(self.engines()).map(|p| -p.y * thrust / n).sum();
            let r = |a: f32, e: f32| {
                let o = eval(a, e, thrust);
                let l = o.force.dot(Vec3::new(0.0, a.cos(), a.sin())) + thrust * a.sin() - w * gamma.cos();
                let m = -o.moment.x + m_thrust;
                (l, m)
            };
            let (r1, r2) = r(a, e);
            let (da, de) = (1e-3, 1e-2);
            let (a1, a2) = r(a + da, e);
            let (b1, b2) = r(a, e + de);
            let (j11, j21, j12, j22) = ((a1 - r1) / da, (a2 - r2) / da, (b1 - r1) / de, (b2 - r2) / de);
            let det = j11 * j22 - j12 * j21;
            if det.abs() < 1e-9 {
                break;
            }
            let step_a = (r1 * j22 - r2 * j12) / det;
            let step_e = (j11 * r2 - j21 * r1) / det;
            a = (a - step_a.clamp(-0.05, 0.05)).clamp(-0.2, self.alpha_usable(flap));
            e = (e - step_e.clamp(-0.3, 0.3)).clamp(-1.5, 1.5);
            if step_a.abs() < 1e-5 && step_e.abs() < 1e-4 {
                break;
            }
        }
        let (mut throttle, mut power) = (0.0, 0.0);
        if self.has_engine {
            let (t, tt, p) = self.throttle_for(thrust, tas * a.cos(), &atm);
            throttle = t;
            power = p;
            thrust = tt;
        }
        let _ = thrust;
        let trim = if self.control_law == ControlLaw::Direct { (e / self.trim_authority).clamp(-1.0, 1.0) } else { 0.0 };
        TrimPoint { alpha: a, gamma, pitch: a + gamma, throttle, power, trim, elevator: e }
    }
}

/// Output of one engine.
#[derive(Clone, Copy, Debug, Default)]
struct EngineOut {
    /// Net thrust along the body nose, N (negative = drag / reverse).
    thrust: f32,
    /// Gross forward thrust before windmill/reverse, N.
    gross: f32,
    /// Shaft power, W (props).
    shaft_power: f32,
    /// Afterburner part of `gross`, N.
    ab_thrust: f32,
}

/// Aerodynamic coefficients at a point.
#[derive(Clone, Copy, Debug)]
pub struct LiftDrag {
    pub cl: f32,
    pub cd: f32,
    /// Stall blend factor on the positive-alpha side (0 attached .. 1 stalled).
    pub stall_pos: f32,
    /// Combined stall blend factor (both sides).
    pub stall: f32,
}

/// A steady straight-flight condition, see [`AircraftDef::trim_point`].
#[derive(Clone, Copy, Debug)]
pub struct TrimPoint {
    /// Angle of attack, rad.
    pub alpha: f32,
    /// Flight-path angle, rad (0 = level, negative = descending).
    pub gamma: f32,
    /// Pitch attitude, rad.
    pub pitch: f32,
    /// Throttle lever 0..1 (0 for gliders).
    pub throttle: f32,
    /// Engine power fraction (props) / N1 (jets).
    pub power: f32,
    /// Trim lever needed with the stick neutral (-1..1, clamped; 0 for FBW).
    pub trim: f32,
    /// Total elevator needed (unclamped; > 1 means out of trim range).
    pub elevator: f32,
}

fn logistic(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

fn move_toward(cur: f32, target: f32, max_step: f32) -> f32 {
    let d = target - cur;
    if d.abs() <= max_step { target } else { cur + max_step * d.signum() }
}

fn deg(d: f32) -> f32 {
    d.to_radians()
}

const fn wheel(pos: Vec3, spring: f32, damping: f32, steerable: bool, braked: bool, retracts: bool) -> GearPoint {
    GearPoint { pos, spring, damping, kind: ContactKind::Wheel { steerable, braked }, retracts }
}

const fn skid(pos: Vec3, spring: f32, damping: f32, friction: f32) -> GearPoint {
    GearPoint { pos, spring, damping, kind: ContactKind::Skid { friction }, retracts: false }
}

const fn hard(x: f32, y: f32, z: f32, reason: &'static str) -> HardPoint {
    HardPoint { pos: Vec3::new(x, y, z), reason, tolerate_below: 0.0 }
}

const GEAR_UP: &str = "Gear-up landing";

// --- Kestrel C1 geometry -----------------------------------------------------

const KESTREL_GEAR: &[GearPoint] = &[
    // c172p JSBSim contacts (CG-relative): steerable nose wheel, braked mains.
    wheel(Vec3::new(0.0, -1.15, 1.17), 52_538.0, 5_000.0, true, false, false),
    wheel(Vec3::new(1.09, -1.02, -0.47), 78_807.0, 8_000.0, false, true, false),
    wheel(Vec3::new(-1.09, -1.02, -0.47), 78_807.0, 8_000.0, false, true, false),
    // Tail skid.
    skid(Vec3::new(0.0, -0.20, -4.77), 29_188.0, 6_000.0, 0.5),
];

const KESTREL_HARD: &[HardPoint] = &[
    hard(0.0, -0.84, 1.94, "Prop strike"),
    hard(0.0, -0.20, 2.25, "Flew into the ground"),
    hard(5.66, 0.81, 0.0, "Wingtip strike"),
    hard(-5.66, 0.81, 0.0, "Wingtip strike"),
    hard(0.0, 0.95, 0.3, "Crashed inverted"),
    hard(0.0, 1.56, -5.5, "Tail struck the ground"),
    hard(1.7, 0.25, -5.4, "Tail struck the ground"),
    hard(-1.7, 0.25, -5.4, "Tail struck the ground"),
];

const KESTREL_ENGINES: &[Vec3] = &[Vec3::new(0.0, 0.11, 1.94)];

// --- Swift S2 geometry -------------------------------------------------------

const SWIFT_GEAR: &[GearPoint] = &[
    // ASK 21 JSBSim contacts: braked main wheel, nose wheel, tail wheel.
    wheel(Vec3::new(0.0, -0.70, -0.32), 92_277.0, 7_500.0, false, true, false),
    wheel(Vec3::new(0.0, -0.62, 1.97), 136_059.0, 6_000.0, false, false, false),
    wheel(Vec3::new(0.0, -0.23, -4.93), 132_759.0, 2_900.0, false, false, false),
    // Invisible wing-runner outriggers so it doesn't topple at rest.
    skid(Vec3::new(0.9, -0.67, -0.32), 20_000.0, 2_500.0, 0.05),
    skid(Vec3::new(-0.9, -0.67, -0.32), 20_000.0, 2_500.0, 0.05),
];

const SWIFT_HARD: &[HardPoint] = &[
    hard(0.0, -0.20, 2.75, "Flew into the ground"),
    HardPoint { pos: Vec3::new(8.44, 0.52, -0.08), reason: "Wingtip strike", tolerate_below: 12.0 },
    HardPoint { pos: Vec3::new(-8.44, 0.52, -0.08), reason: "Wingtip strike", tolerate_below: 12.0 },
    hard(0.0, 0.62, 1.0, "Crashed inverted"),
    hard(0.0, 1.90, -5.2, "Tail struck the ground"),
];

// --- Peregrine Mk IX (Spitfire IX class) ---------------------------------------

const PEREGRINE_GEAR: &[GearPoint] = &[
    // Spitfire IX (FlightGear model, gear down): retractable braked mains ahead
    // of the CG, steerable tail wheel; rests ~13.5° nose-up.
    wheel(Vec3::new(0.76, -1.92, 0.58), 120_000.0, 45_000.0, false, true, true),
    wheel(Vec3::new(-0.76, -1.92, 0.58), 120_000.0, 45_000.0, false, true, true),
    wheel(Vec3::new(0.00, -0.43, -5.53), 80_000.0, 6_000.0, true, false, false),
];

const PEREGRINE_HARD: &[HardPoint] = &[
    hard(0.00, -1.28, 2.62, "Prop strike"),
    hard(0.00, 0.13, 3.09, "Flew into the ground"),
    hard(5.62, -0.25, -0.20, "Wingtip strike"),
    hard(-5.62, -0.25, -0.20, "Wingtip strike"),
    hard(0.00, 0.90, -0.80, "Crashed inverted"),
    hard(0.00, 1.50, -5.70, "Crashed inverted"),
    // Underside: radiators and belly (first to touch with the gear up).
    hard(1.30, -0.90, -0.40, GEAR_UP),
    hard(-1.30, -0.90, -0.40, GEAR_UP),
    hard(0.00, -0.75, 0.80, GEAR_UP),
    hard(0.00, -0.60, -2.80, GEAR_UP),
];

// Prop pivot as built by the models track (9.55 m airframe; the table said z = 4.60).
const PEREGRINE_ENGINES: &[Vec3] = &[Vec3::new(0.00, 0.13, 2.62)];

// --- Lancer F2 (F-16 / Gripen class) -------------------------------------------

const LANCER_GEAR: &[GearPoint] = &[
    // F-16 JSBSim contacts.
    wheel(Vec3::new(0.0, -1.72, 3.20), 251_745.0, 30_000.0, true, false, true),
    wheel(Vec3::new(1.17, -1.60, -0.38), 547_271.0, 65_000.0, false, true, true),
    wheel(Vec3::new(-1.17, -1.60, -0.38), 547_271.0, 65_000.0, false, true, true),
    // Nozzle / ventral fin scrape (over-rotation).
    skid(Vec3::new(0.0, -0.34, -4.70), 300_000.0, 30_000.0, 0.5),
];

const LANCER_HARD: &[HardPoint] = &[
    hard(0.0, 0.04, 7.82, "Flew into the ground"),
    hard(4.61, 0.21, -1.41, "Wingtip strike"),
    hard(-4.61, 0.21, -1.41, "Wingtip strike"),
    hard(0.0, 1.33, 3.52, "Crashed inverted"),
    hard(0.0, 3.39, -4.66, "Crashed inverted"),
    hard(0.0, -1.10, 3.30, GEAR_UP),
    hard(0.0, -1.05, 0.50, GEAR_UP),
    hard(0.0, -0.70, -3.00, GEAR_UP),
];

const LANCER_ENGINES: &[Vec3] = &[Vec3::new(0.0, 0.27, -5.20)];

// --- Aerolink 72 (ATR 72 class) --------------------------------------------------

const AEROLINK_GEAR: &[GearPoint] = &[
    // ATR 72-500 (FlightGear model, gear down).
    wheel(Vec3::new(0.0, -2.35, 10.08), 300_000.0, 25_000.0, true, false, true),
    wheel(Vec3::new(1.92, -2.75, -0.72), 790_000.0, 87_000.0, false, true, true),
    wheel(Vec3::new(-1.92, -2.75, -0.72), 790_000.0, 87_000.0, false, true, true),
    // Tail bumper.
    skid(Vec3::new(0.0, -0.20, -11.0), 600_000.0, 60_000.0, 0.5),
];

const AEROLINK_HARD: &[HardPoint] = &[
    hard(0.0, -1.50, 11.70, "Flew into the ground"),
    hard(13.50, 1.20, -1.00, "Wingtip strike"),
    hard(-13.50, 1.20, -1.00, "Wingtip strike"),
    hard(3.86, -1.17, 2.90, "Prop strike"),
    hard(-3.86, -1.17, 2.90, "Prop strike"),
    hard(0.0, 1.30, 8.00, "Crashed inverted"),
    hard(0.0, 5.20, -13.50, "Crashed inverted"),
    hard(0.0, -1.70, 6.00, GEAR_UP),
    hard(0.0, -1.60, -4.00, GEAR_UP),
    hard(1.60, -2.00, -0.70, GEAR_UP),
    hard(-1.60, -2.00, -0.70, GEAR_UP),
];

const AEROLINK_ENGINES: &[Vec3] = &[Vec3::new(3.86, 0.79, 2.90), Vec3::new(-3.86, 0.79, 2.90)];

// --- Meridian 190 (E190 class) ---------------------------------------------------

const MERIDIAN190_GEAR: &[GearPoint] = &[
    // E190 JSBSim contacts (main-gear stiffness raised to keep the static
    // compression near the model's).
    wheel(Vec3::new(0.00, -2.24, 10.78), 364_848.0, 45_000.0, true, false, true),
    wheel(Vec3::new(2.57, -2.34, -2.07), 1_200_000.0, 170_000.0, false, true, true),
    wheel(Vec3::new(-2.57, -2.34, -2.07), 1_200_000.0, 170_000.0, false, true, true),
    skid(Vec3::new(0.00, 0.32, -15.30), 1_000_000.0, 100_000.0, 0.5),
];

const MERIDIAN190_HARD: &[HardPoint] = &[
    hard(0.00, -0.69, 15.10, "Flew into the ground"),
    hard(14.10, 0.42, -6.00, "Wingtip strike"),
    hard(-14.10, 0.42, -6.00, "Wingtip strike"),
    hard(4.04, -1.73, 3.10, "Engine pod strike"),
    hard(-4.04, -1.73, 3.10, "Engine pod strike"),
    hard(4.04, -1.68, 0.70, "Engine pod strike"),
    hard(-4.04, -1.68, 0.70, "Engine pod strike"),
    hard(0.00, 2.22, 12.20, "Crashed inverted"),
    hard(0.00, 8.32, -19.30, "Crashed inverted"),
    hard(0.00, -1.68, 6.20, GEAR_UP),
    hard(0.00, -1.43, -5.80, GEAR_UP),
];

const MERIDIAN190_ENGINES: &[Vec3] = &[Vec3::new(4.04, -1.03, 3.12), Vec3::new(-4.04, -1.03, 3.12)];

// --- Meridian 320 (A320 class) ---------------------------------------------------

const MERIDIAN320_GEAR: &[GearPoint] = &[
    // A320 JSBSim contacts (track from the model; main-gear stiffness raised
    // to keep the static compression near the model's).
    wheel(Vec3::new(0.0, -3.63, 11.22), 488_823.0, 60_000.0, true, false, true),
    wheel(Vec3::new(3.86, -3.64, -1.29), 2_000_000.0, 280_000.0, false, true, true),
    wheel(Vec3::new(-3.86, -3.64, -1.29), 2_000_000.0, 280_000.0, false, true, true),
    skid(Vec3::new(0.0, 1.60, -21.19), 1_500_000.0, 150_000.0, 0.5),
];

const MERIDIAN320_HARD: &[HardPoint] = &[
    hard(0.0, -0.15, 16.37, "Flew into the ground"),
    hard(17.00, 1.07, -4.92, "Wingtip strike"),
    hard(-17.00, 1.07, -4.92, "Wingtip strike"),
    hard(5.63, -2.55, 2.92, "Engine pod strike"),
    hard(-5.63, -2.55, 2.92, "Engine pod strike"),
    hard(5.63, -2.45, 0.60, "Engine pod strike"),
    hard(-5.63, -2.45, 0.60, "Engine pod strike"),
    hard(0.0, 2.14, 13.43, "Crashed inverted"),
    hard(0.0, 8.47, -19.46, "Crashed inverted"),
    hard(0.0, -1.52, 11.62, GEAR_UP),
    hard(0.0, -1.47, -9.22, GEAR_UP),
];

const MERIDIAN320_ENGINES: &[Vec3] = &[Vec3::new(5.63, -1.55, 4.03), Vec3::new(-5.63, -1.55, 4.03)];

/// The aircraft roster, in contract order: kestrel, swift, peregrine, lancer,
/// aerolink, meridian190, meridian320, then the v3 heavies hercules, galaxy,
/// vulcan, the rotorcraft ec130 (helicopter) and fpv (quadcopter), the H2b
/// props extra500, cub, zlin, otter, dc3 and the H2a jets triple7, crj700,
/// citationx, concorde.
pub fn catalogue() -> Vec<AircraftDef> {
    // Built once (the JSBSim FDMs are parsed and fitted here).
    static CAT: std::sync::OnceLock<Vec<AircraftDef>> = std::sync::OnceLock::new();
    CAT.get_or_init(|| {
        let mut v = vec![
            kestrel_def(),
            swift_def(),
            peregrine_def(),
            lancer_def(),
            aerolink_def(),
            meridian190_def(),
            meridian320_def(),
            hercules_def(),
            galaxy_def(),
            vulcan_def(),
            crate::rotor::ec130_def(),
            crate::rotor::quad_def(),
            // Track H2b: more propeller types (src/flight/ga.rs).
            ga::extra500_def(),
            ga::cub_def(),
            ga::zlin_def(),
            ga::otter_def(),
            ga::dc3_def(),
            jets::triple7_def(),
            jets::crj700_def(),
            jets::citationx_def(),
            jets::concorde_def(),
        ];
        for d in &mut v {
            rig_for_cruise(d);
        }
        v
    })
    .clone()
}

/// Real-type names accepted for the aircraft ids (CLI `--aircraft`, tests).
/// The ids themselves are the stable contract used across the app.
pub const ALIASES: &[(&str, &str)] = &[
    ("c172", "kestrel"),
    ("c172p", "kestrel"),
    ("cessna172", "kestrel"),
    ("ask21", "swift"),
    ("spitfire", "peregrine"),
    ("spitfire9", "peregrine"),
    ("spitfireix", "peregrine"),
    ("f16", "lancer"),
    ("f-16", "lancer"),
    ("atr72", "aerolink"),
    ("atr-72", "aerolink"),
    ("atr", "aerolink"),
    ("e190", "meridian190"),
    ("embraer190", "meridian190"),
    ("a320", "meridian320"),
    ("c130", "hercules"),
    ("c-130", "hercules"),
    ("c130h", "hercules"),
    ("c5", "galaxy"),
    ("c-5", "galaxy"),
    ("c5m", "galaxy"),
    ("vulcanb2", "vulcan"),
    ("avro-vulcan", "vulcan"),
    ("heli", "ec130"),
    ("helicopter", "ec130"),
    ("h130", "ec130"),
    ("ec130b4", "ec130"),
    ("quad", "fpv"),
    ("drone", "fpv"),
    ("quadcopter", "fpv"),
    ("777", "triple7"),
    ("b777", "triple7"),
    ("777-300er", "triple7"),
    ("777300er", "triple7"),
    ("boeing777", "triple7"),
    ("crj", "crj700"),
    ("crj-700", "crj700"),
    ("crj700er", "crj700"),
    ("cl600", "crj700"),
    ("citation", "citationx"),
    ("citation-x", "citationx"),
    ("c750", "citationx"),
    ("cessna750", "citationx"),
    ("sst", "concorde"),
    ("ba-concorde", "concorde"),
    ("ea500", "extra500"),
    ("extra-500", "extra500"),
    ("pa18", "cub"),
    ("pa-18", "cub"),
    ("supercub", "cub"),
    ("super-cub", "cub"),
    ("z50", "zlin"),
    ("z-50", "zlin"),
    ("z50lx", "zlin"),
    ("zlin50", "zlin"),
    ("aerobat", "zlin"),
    ("dhc3", "otter"),
    ("dhc-3", "otter"),
    ("floatplane", "otter"),
    ("seaplane", "otter"),
    ("dc-3", "dc3"),
    ("c47", "dc3"),
    ("c-47", "dc3"),
    ("dakota", "dc3"),
];

/// Canonical id for an id or alias (case-insensitive).
pub fn resolve_id(s: &str) -> String {
    let l = s.trim().to_ascii_lowercase();
    ALIASES.iter().find(|(a, _)| *a == l).map(|(_, id)| id.to_string()).unwrap_or(l)
}

/// Roster index of an aircraft id or alias.
pub fn index_of(s: &str) -> Option<usize> {
    let id = resolve_id(s);
    catalogue().iter().position(|d| d.id == id)
}

/// Look up an aircraft definition by id (or alias, see [`ALIASES`]).
pub fn find(id: &str) -> Option<AircraftDef> {
    let id = resolve_id(id);
    catalogue().into_iter().find(|d| d.id == id)
}

/// Controls (trim + throttle, everything else neutral, gear up if
/// retractable) that hold steady straight flight at indicated `speed_ms`,
/// clean, at sea level. Handy for air starts; see [`trimmed_controls_at`].
pub fn trimmed_controls(def: &AircraftDef, speed_ms: f32) -> Controls {
    trimmed_controls_at(def, speed_ms, 0.0)
}

/// [`trimmed_controls`] at altitude `alt` (m).
pub fn trimmed_controls_at(def: &AircraftDef, speed_ms: f32, alt: f32) -> Controls {
    if let Some(rd) = &def.rotor {
        return crate::rotor::defs::trimmed_controls(def, rd, speed_ms, alt);
    }
    let tp = def.trim_point_at(speed_ms, 0.0, alt, false);
    Controls { throttle: tp.throttle, trim: tp.trim, gear_down: !def.retractable_gear, ..Default::default() }
}

/// Built-in rigging for propeller aircraft: an offset fin / aileron strip
/// that cancels the torque, swirl and P-factor at the cruise condition, as on
/// the real thing. Away from cruise the pilot still needs rudder and aileron.
fn rig_for_cruise(def: &mut AircraftDef) {
    if !matches!(def.engine_kind, EngineKind::Piston | EngineKind::Turboprop)
        || (def.torque_k == 0.0 && def.swirl_k == 0.0 && def.pfactor_k == 0.0)
        || def.rig_kt <= 0.0
    {
        return;
    }
    let ias = def.rig_kt * KT;
    let alt = 1500.0;
    let tp = def.trim_point_at(ias, 0.0, alt, false);
    let atm = isa(alt);
    let tas = tas_from_cas(ias, alt);
    let vb = Vec3::new(0.0, -tas * tp.alpha.sin(), tas * tp.alpha.cos());
    let out = def.engine_output(tp.power, 0.0, 0.0, vb.z, &atm, false);
    let rpm = def.target_rpm(tp.power, vb.z, true) * def.prop_gear;
    let (roll, yaw) = def.prop_moments(&out, rpm, vb.z, vb, atm.density);
    let n = def.engines() as f32;
    let qsb = 0.5 * atm.density * tas * tas * def.wing_area * def.wingspan;
    def.cl_rig = -roll * n / qsb;
    def.cn_rig = -yaw * n / qsb;
}

// --- JSBSim aerodynamics per aircraft ---------------------------------------

fn with_jsb(mut def: AircraftDef, xml: &str, aliases: &[(&str, &str)], consts: &[(&str, f64)], fit_kt: f32, setup: impl FnOnce(&mut JsbDef)) -> AircraftDef {
    let mut j = JsbDef::new(xml, aliases, consts);
    j.flap_max_deg = def.flap_max_deg;
    setup(&mut j);
    j.calibrate_signs();
    // Reference geometry follows the FDM so fitted derivatives stay consistent.
    def.wing_area = j.aero.sw_m2 as f32;
    def.wingspan = j.aero.bw_m as f32;
    def.chord = j.aero.cbar_m as f32;
    j.fit(&mut def, fit_kt * KT);
    def.jsb = Some(std::sync::Arc::new(j));
    def
}

fn kestrel_def() -> AircraftDef {
    with_jsb(
        kestrel_base(),
        include_str!("../assets/aircraft/kestrel/fdm.xml"),
        &[],
        &[
            ("gear/drag-coef[1]", 1.0),
            ("gear/drag-coef[2]", 1.0),
            ("gear/drag-coef[3]", 1.0),
            ("gear/drag-coef[4]", 1.0),
            ("gear/drag-coef[5]", 1.0),
            ("gear/drag-coef[6]", 1.0),
            ("controls/flight/adv-yaw-fact1", 0.2),
            ("controls/flight/adv-yaw-fact2", 0.2),
        ],
        90.0,
        |j| {
            j.elev_up = deg(28.0);
            j.elev_down = deg(23.0);
            j.ail_up = deg(20.0);
            j.ail_down = deg(15.0);
            j.rudder_max = deg(16.0);
        },
    )
}

fn swift_def() -> AircraftDef {
    with_jsb(swift_base(), include_str!("../assets/aircraft/swift/fdm.xml"), &[], &[], 55.0, |j| {
        j.elev_up = 0.35;
        j.elev_down = 0.35;
        j.ail_up = 0.35;
        j.ail_down = 0.35;
        // The ASK 21 FDM uses a ±1 "rad" rudder as a normalised value.
        j.rudder_max = 1.0;
        j.flaps_are_airbrakes = true;
    })
}

fn lancer_def() -> AircraftDef {
    with_jsb(
        lancer_base(),
        include_str!("../assets/aircraft/lancer/fdm.xml"),
        &[("fcs/fly-by-wire/roll/aileron-pos-norm", "fcs/aileron-pos-norm"), ("fcs/fly-by-wire/yaw/rudder-pos-norm", "fcs/rudder-pos-norm")],
        &[],
        300.0,
        |j| {
            // All-moving stabilators, flaperons, rudder.
            j.elev_up = deg(25.0);
            j.elev_down = deg(25.0);
            j.ail_up = deg(21.5);
            j.ail_down = deg(21.5);
            j.rudder_max = deg(30.0);
            j.lef = true;
        },
    )
}

fn meridian190_def() -> AircraftDef {
    with_jsb(
        meridian190_base(),
        include_str!("../assets/aircraft/meridian190/fdm.xml"),
        &[],
        &[
            ("aero-config/coefficients/bank-elevator", 0.05),
            ("aero-config/coefficients/bank-rudder", 0.01),
            ("aero-config/coefficients/drag-alpha", 2.0),
            ("aero-config/coefficients/drag-beta", 1.0),
            ("aero-config/coefficients/drag-elevator", 0.02),
            ("aero-config/coefficients/drag-flap", 0.04),
            ("aero-config/coefficients/drag-gear", 0.04),
            ("aero-config/coefficients/drag-hstab", 0.01),
            ("aero-config/coefficients/drag-induced", 0.03),
            ("aero-config/coefficients/drag-mach", 0.021),
            ("aero-config/coefficients/drag-mfs", 0.01),
            ("aero-config/coefficients/drag-parasitic", 0.01),
            ("aero-config/coefficients/drag-slat", 0.003),
            ("aero-config/coefficients/lift-elevator", 0.2),
            ("aero-config/coefficients/lift-hstab", 0.4),
            ("aero-config/coefficients/yaw-elevator", 0.005),
        ],
        200.0,
        |j| {
            j.elev_up = deg(27.0);
            j.elev_down = deg(17.0);
            j.ail_up = deg(25.0);
            j.ail_down = deg(20.0);
            j.rudder_max = deg(25.0);
            j.slats = &[(0.0, 0.0), (0.15, 0.6), (0.4, 0.6), (0.6, 1.0), (1.0, 1.0)];
            j.slat_max_deg = 25.0;
            j.spoiler_max_deg = 40.0;
        },
    )
}

fn meridian320_def() -> AircraftDef {
    with_jsb(meridian320_base(), include_str!("../assets/aircraft/meridian320/fdm.xml"), &[], &[], 200.0, |j| {
        j.elev_up = deg(30.0);
        j.elev_down = deg(17.0);
        j.ail_up = deg(25.0);
        j.ail_down = deg(25.0);
        j.rudder_max = deg(30.0);
        // Slats 18° / 22° / 22° / 27° with CONF 1 / 2 / 3 / FULL.
        j.slats = &[(0.0, 0.0), (0.1, 0.667), (0.25, 0.667), (0.375, 0.815), (0.5, 0.815), (1.0, 1.0)];
        j.slat_max_deg = 27.0;
        j.spoiler_max_deg = 45.0;
    })
}

fn kestrel_base() -> AircraftDef {
    AircraftDef {
        id: "kestrel",
        name: "Cessna 172P Skyhawk",
        description: "Four-seat high-wing trainer (FlightGear c172p, JSBSim aerodynamics). \
                      160 hp Lycoming O-320, fixed-pitch prop, flaps 10/20/30°. Docile and forgiving.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 10.0, 20.0, 30.0],
        flap_max_deg: 30.0,
        vs0_kt: 44.0,
        vs1_kt: 49.0,
        vno_kt: 128.0,
        vne_kt: 160.0,
        gear_height: 0.99,
        wingspan: 10.91,
        mass: 932.0,
        inertia_roll: 1500.0,
        inertia_pitch: 1806.0,
        inertia_yaw: 2748.0,
        wing_area: 16.17,
        chord: 1.49,
        wing_height: 0.85,
        cl0: 0.35,
        cl_alpha: 4.9,
        alpha_stall: deg(16.8),
        alpha_stall_neg: deg(15.0),
        stall_sharpness: 50.0,
        cl_flat: 1.0,
        stall_drop: 0.85,
        stall_decay: 0.15,
        cd0: 0.031,
        oswald: 0.72,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.15,
        flap_cl: 0.70,
        flap_curve: 0.5,
        flap_alpha_stall: deg(-2.0),
        flap_cd: 0.040,
        flap_cd_exp: 1.6,
        flap_cm: -0.03,
        cm0: 0.07,
        cm_alpha: -1.1,
        cm_q: -18.0,
        cm_elevator: 0.55,
        cm_stall: -0.12,
        trim_authority: 0.35,
        cy_beta: -0.45,
        cl_beta: -0.09,
        cl_p: -0.47,
        stall_autorotation: 0.2,
        cl_r_per_cl: 0.25,
        cl_aileron: 0.05,
        cn_beta: 0.07,
        cn_r: -0.10,
        cn_p: -0.03,
        cn_rudder: 0.022,
        cn_aileron: -0.004,
        surface_rate: 5.0,
        flap_rate: 0.2,
        static_thrust: 2400.0,
        thrust_v1: 4.84,
        thrust_v2: 0.236,
        windmill_cd: 0.19,
        prop_disk_area: 2.84,
        prop_wash: 0.35,
        idle_power: 0.05,
        spool_time: 0.5,
        rpm_idle: 650.0,
        rpm_static: 2350.0,
        rpm_per_ms: 6.0,
        gear: KESTREL_GEAR,
        hard_points: KESTREL_HARD,
        roll_mu: 0.02,
        brake_mu: 0.7,
        tire_mu: 0.85,
        nose_steer_deg: 12.0,
        max_touchdown_sink: 3.5,
        max_touchdown_bank: deg(12.0),
        min_touchdown_pitch: deg(-2.5),
        g_limit_pos: 5.7,
        g_limit_neg: -3.0,

        category: Category::Trainer,
        engine_kind: EngineKind::Piston,
        engine_count: 1,
        eye: Vec3::new(0.21, 0.46, -0.22),
        vr_kt: 55.0,
        vref_kt: 65.0,
        vle_kt: 160.0,
        ceiling_m: 4_100.0,
        engine_pos: KESTREL_ENGINES,
        engine_power_w: 119_000.0,
        prop_radius: 0.95,
        prop_rotation: 1.0,
        prop_gear: 1.0,
        prop_inertia: 0.0,
        lapse_exp: 1.13,
        sfc: 0.30,
        egt_idle: 560.0,
        egt_max: 720.0,
        ..Default::default()
    }
}

fn swift_base() -> AircraftDef {
    AircraftDef {
        id: "swift",
        name: "Schleicher ASK 21",
        description: "Two-seat glass-fibre training sailplane (FlightGear ASK21, JSBSim \
                      aerodynamics). 17 m span, L/D 34, top-surface airbrakes. Find thermals to stay up.",
        has_engine: false,
        flap_label: "Airbrakes",
        flap_detents: &[0.0, 0.25, 0.5, 0.75, 1.0],
        flap_max_deg: 0.0,
        vs0_kt: 36.0,
        vs1_kt: 36.0,
        vno_kt: 108.0,
        vne_kt: 151.0,
        gear_height: 0.67,
        wingspan: 17.0,
        mass: 520.0,
        inertia_roll: 1361.0,
        inertia_pitch: 669.0,
        inertia_yaw: 3450.0,
        wing_area: 17.95,
        chord: 1.05,
        wing_height: 0.1,
        cl0: 0.40,
        cl_alpha: 5.6,
        alpha_stall: deg(12.3),
        alpha_stall_neg: deg(10.0),
        stall_sharpness: 50.0,
        cl_flat: 1.0,
        stall_drop: 0.8,
        stall_decay: 0.12,
        cd0: 0.0102,
        oswald: 0.92,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.12,
        flap_cl: -0.22,
        flap_curve: 0.0,
        flap_alpha_stall: 0.0,
        flap_cd: 0.040,
        flap_cd_exp: 1.0,
        flap_cm: 0.0,
        cm0: 0.05,
        cm_alpha: -0.9,
        cm_q: -35.0,
        cm_elevator: 0.5,
        cm_stall: -0.10,
        trim_authority: 0.35,
        cy_beta: -0.35,
        cl_beta: -0.08,
        cl_p: -0.55,
        stall_autorotation: 0.3,
        cl_r_per_cl: 0.25,
        cl_aileron: 0.065,
        cn_beta: 0.05,
        cn_r: -0.09,
        cn_p: -0.04,
        cn_rudder: 0.022,
        cn_aileron: -0.008,
        surface_rate: 4.0,
        flap_rate: 1.0,
        static_thrust: 0.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.0,
        prop_disk_area: 0.0,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 1.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: SWIFT_GEAR,
        hard_points: SWIFT_HARD,
        roll_mu: 0.025,
        brake_mu: 0.5,
        tire_mu: 0.8,
        nose_steer_deg: 0.0,
        max_touchdown_sink: 3.5,
        max_touchdown_bank: deg(12.0),
        min_touchdown_pitch: deg(-3.0),
        g_limit_pos: 7.0,
        g_limit_neg: -4.5,

        category: Category::Glider,
        engine_kind: EngineKind::None,
        engine_count: 0,
        eye: Vec3::new(0.0, 0.30, 1.27),
        vr_kt: 45.0,
        vref_kt: 50.0,
        vle_kt: 150.0,
        ceiling_m: 8_000.0,
        ..Default::default()
    }
}

fn peregrine_def() -> AircraftDef {
    AircraftDef {
        id: "peregrine",
        name: "Supermarine Spitfire Mk IX",
        description: "Merlin-engined fighter (FlightGear spitfireIX model; flight model calibrated to \
                      its YASim data and published figures). 1,720 hp, four-blade prop, taildragger: \
                      swings left on take-off, so lead with right rudder.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 85.0],
        flap_max_deg: 85.0,
        vs0_kt: 70.0,
        vs1_kt: 82.0,
        vno_kt: 300.0,
        vne_kt: 390.0,
        gear_height: 1.67,
        wingspan: 11.23,
        mass: 3_400.0,
        inertia_roll: 6_100.0,
        inertia_pitch: 7_700.0,
        inertia_yaw: 13_000.0,
        wing_area: 22.48,
        chord: 2.0,
        wing_height: -0.4,
        cl0: 0.18,
        cl_alpha: 4.6,
        alpha_stall: deg(17.0),
        alpha_stall_neg: deg(14.0),
        stall_sharpness: 40.0,
        cl_flat: 1.0,
        stall_drop: 0.8,
        stall_decay: 0.15,
        cd0: 0.0215,
        oswald: 0.82,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.62,
        flap_curve: 0.0,
        flap_alpha_stall: deg(-2.0),
        flap_cd: 0.075,
        flap_cd_exp: 1.0,
        flap_cm: -0.05,
        cm0: 0.04,
        cm_alpha: -0.75,
        cm_q: -15.0,
        cm_elevator: 0.60,
        cm_stall: -0.10,
        trim_authority: 0.3,
        cy_beta: -0.5,
        cl_beta: -0.12,
        cl_p: -0.45,
        stall_autorotation: 0.35,
        cl_r_per_cl: 0.25,
        cl_aileron: 0.040,
        cn_beta: 0.065,
        cn_r: -0.11,
        cn_p: -0.03,
        cn_rudder: 0.032,
        cn_aileron: -0.004,
        surface_rate: 5.0,
        flap_rate: 0.6,
        static_thrust: 22_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.12,
        prop_disk_area: 8.04,
        prop_wash: 0.45,
        idle_power: 0.02,
        spool_time: 0.8,
        rpm_idle: 700.0,
        rpm_static: 2_850.0,
        rpm_per_ms: 14.0,
        gear: PEREGRINE_GEAR,
        hard_points: PEREGRINE_HARD,
        roll_mu: 0.03,
        brake_mu: 0.6,
        tire_mu: 0.8,
        nose_steer_deg: 12.0,
        max_touchdown_sink: 3.5,
        max_touchdown_bank: deg(10.0),
        min_touchdown_pitch: deg(-6.0),
        g_limit_pos: 9.5,
        g_limit_neg: -4.5,

        category: Category::Warbird,
        engine_kind: EngineKind::Piston,
        engine_count: 1,
        retractable_gear: true,
        taildragger: true,
        eye: Vec3::new(0.0, 0.70, -1.15),
        vr_kt: 90.0,
        vref_kt: 90.0,
        vle_kt: 139.0,
        ceiling_m: 13_100.0,
        mmo: 0.0,
        gear_time: 6.0,
        cd_gear: 0.012,
        mach_dd: 0.76,
        cd_wave: 0.05,
        engine_pos: PEREGRINE_ENGINES,
        engine_power_w: 1_250_000.0,
        const_speed_prop: true,
        prop_eta: 0.82,
        prop_fm: 0.75,
        prop_radius: 1.64,
        prop_rotation: 1.0,
        prop_gear: 0.477,
        prop_inertia: 85.0,
        crit_alt_m: 5_800.0,
        lapse_exp: 1.0,
        // Net of the slipstream swirl over the wing roots, which opposes
        // most of the engine reaction torque.
        torque_k: 0.4,
        swirl_k: 0.35,
        pfactor_k: 0.30,
        rig_kt: 200.0,
        sfc: 0.30,
        egt_idle: 560.0,
        egt_max: 780.0,
        ..Default::default()
    }
}

fn lancer_base() -> AircraftDef {
    AircraftDef {
        id: "lancer",
        name: "General Dynamics F-16C",
        description: "Single-engine FBW fighter (FlightGear f16, JSBSim aerodynamics). F100-PW-229: \
                      79 kN dry, 129 kN in reheat above the 90 % detent; 9 g / AoA limiter.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 20.0],
        flap_max_deg: 20.0,
        vs0_kt: 115.0,
        vs1_kt: 122.0,
        vno_kt: 600.0,
        vne_kt: 800.0,
        gear_height: 1.53,
        wingspan: 9.14,
        mass: 10_973.0,
        inertia_roll: 17_216.0,
        inertia_pitch: 80_179.0,
        inertia_yaw: 92_344.0,
        wing_area: 27.87,
        chord: 3.45,
        wing_height: -0.2,
        cl0: 0.03,
        cl_alpha: 3.6,
        alpha_stall: deg(32.0),
        alpha_stall_neg: deg(20.0),
        stall_sharpness: 15.0,
        cl_flat: 1.2,
        stall_drop: 0.9,
        stall_decay: 0.4,
        cd0: 0.0180,
        oswald: 0.72,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.4,
        cd_beta: 0.3,
        flap_cl: 0.25,
        flap_curve: 0.0,
        flap_alpha_stall: 0.0,
        flap_cd: 0.02,
        flap_cd_exp: 1.0,
        flap_cm: -0.02,
        cm0: 0.0,
        cm_alpha: -0.15,
        cm_q: -6.0,
        cm_elevator: 0.35,
        cm_stall: -0.05,
        trim_authority: 0.3,
        cy_beta: -0.8,
        cl_beta: -0.05,
        cl_p: -0.35,
        stall_autorotation: 0.05,
        cl_r_per_cl: 0.15,
        cl_aileron: 0.05,
        cn_beta: 0.10,
        cn_r: -0.25,
        cn_p: -0.02,
        cn_rudder: 0.04,
        cn_aileron: -0.002,
        surface_rate: 6.0,
        flap_rate: 0.5,
        static_thrust: 76_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.15,
        prop_disk_area: 0.75,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 4.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: LANCER_GEAR,
        hard_points: LANCER_HARD,
        roll_mu: 0.02,
        brake_mu: 0.6,
        tire_mu: 0.8,
        nose_steer_deg: 22.0,
        max_touchdown_sink: 4.0,
        max_touchdown_bank: deg(10.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 13.5,
        g_limit_neg: -4.5,

        category: Category::FastJet,
        engine_kind: EngineKind::Jet,
        engine_count: 1,
        retractable_gear: true,
        has_spoilers: true,
        has_afterburner: true,
        eye: Vec3::new(0.0, 1.07, 4.16),
        vr_kt: 140.0,
        vref_kt: 145.0,
        vle_kt: 300.0,
        ceiling_m: 15_240.0,
        mmo: 2.05,
        control_law: ControlLaw::FastJet,
        gear_time: 5.0,
        cd_gear: 0.020,
        spoiler_cl: 0.0,
        spoiler_cd: 0.06,
        spoiler_flight_max: 1.0,
        mach_dd: 0.93,
        cd_wave: 0.028,
        engine_pos: LANCER_ENGINES,
        n1_idle: 0.35,
        idle_thrust_frac: 0.05,
        thrust_mach: [-0.20, 0.20],
        ab_static_thrust: 53_000.0,
        ab_mach: [0.0, 0.25],
        t_total_max: 410.0,
        n1_rpm: 14_000.0,
        sfc: 0.077,
        sfc_ab: 0.20,
        egt_idle: 420.0,
        egt_max: 900.0,
        alpha_max: deg(25.0),
        nz_max: 9.0,
        nz_min: -3.0,
        roll_rate_max: deg(240.0),
        yaw_damper: 0.0,
        ..Default::default()
    }
}

fn aerolink_def() -> AircraftDef {
    AircraftDef {
        id: "aerolink",
        name: "ATR 72-500",
        description: "Twin-turboprop regional airliner (FlightGear ATR-72-500 model; flight model \
                      calibrated to its YASim data and published figures). 2 × PW127F, six-blade props, \
                      reverse pitch. Needs ~1,400 m.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 15.0, 30.0],
        flap_max_deg: 30.0,
        vs0_kt: 92.0,
        vs1_kt: 109.0,
        vno_kt: 250.0,
        vne_kt: 250.0,
        gear_height: 2.64,
        wingspan: 27.05,
        mass: 21_000.0,
        inertia_roll: 220_000.0,
        inertia_pitch: 500_000.0,
        inertia_yaw: 680_000.0,
        wing_area: 61.0,
        chord: 2.3,
        wing_height: 1.16,
        cl0: 0.40,
        cl_alpha: 5.2,
        alpha_stall: deg(17.0),
        alpha_stall_neg: deg(14.0),
        stall_sharpness: 40.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.027,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.85,
        flap_curve: 0.6,
        flap_alpha_stall: deg(1.0),
        flap_cd: 0.060,
        flap_cd_exp: 1.5,
        flap_cm: -0.05,
        cm0: 0.05,
        cm_alpha: -1.4,
        cm_q: -25.0,
        cm_elevator: 0.6,
        cm_stall: -0.10,
        trim_authority: 0.4,
        cy_beta: -0.6,
        cl_beta: -0.11,
        cl_p: -0.5,
        stall_autorotation: 0.15,
        cl_r_per_cl: 0.25,
        cl_aileron: 0.028,
        cn_beta: 0.08,
        cn_r: -0.12,
        cn_p: -0.03,
        cn_rudder: 0.05,
        cn_aileron: -0.003,
        surface_rate: 2.5,
        flap_rate: 0.08,
        static_thrust: 75_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.15,
        prop_disk_area: 23.9,
        prop_wash: 0.0,
        idle_power: 0.06,
        spool_time: 1.5,
        rpm_idle: 800.0,
        rpm_static: 1_200.0,
        rpm_per_ms: 8.0,
        gear: AEROLINK_GEAR,
        hard_points: AEROLINK_HARD,
        roll_mu: 0.02,
        brake_mu: 0.45,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(10.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 3.75,
        g_limit_neg: -1.5,

        category: Category::Airliner,
        engine_kind: EngineKind::Turboprop,
        engine_count: 2,
        retractable_gear: true,
        has_reverser: true,
        eye: Vec3::new(0.6, 0.25, 9.25),
        vr_kt: 108.0,
        vref_kt: 113.0,
        vle_kt: 185.0,
        ceiling_m: 7_620.0,
        mmo: 0.55,
        gear_time: 8.0,
        cd_gear: 0.018,
        mach_dd: 0.62,
        cd_wave: 0.03,
        engine_pos: AEROLINK_ENGINES,
        engine_power_w: 2_051_000.0,
        const_speed_prop: true,
        prop_eta: 0.85,
        prop_fm: 0.72,
        prop_radius: 1.965,
        prop_rotation: 1.0,
        prop_gear: 1.0,
        prop_inertia: 150.0,
        crit_alt_m: 1_500.0,
        lapse_exp: 1.0,
        torque_k: 1.0,
        swirl_k: 0.0,
        pfactor_k: 0.05,
        rig_kt: 180.0,
        sfc: 0.29,
        egt_idle: 450.0,
        egt_max: 780.0,
        reverse_eff: 0.35,
        yaw_damper: 2.0,
        // ATR 72-500: TOFL ~1,300 m at MTOW, less at the weights flown here.
        min_runway_m: 1_100.0,
        ..Default::default()
    }
}

fn meridian190_base() -> AircraftDef {
    AircraftDef {
        id: "meridian190",
        name: "Embraer E190",
        description: "Regional twin-jet (E-jet family by the FlightGear community, JSBSim \
                      aerodynamics). 2 × GE CF34-10E, spoilers, reversers, slats / flaps.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 7.0, 10.0, 20.0, 37.0],
        flap_max_deg: 37.0,
        vs0_kt: 103.0,
        vs1_kt: 148.0,
        vno_kt: 320.0,
        vne_kt: 320.0,
        gear_height: 2.23,
        wingspan: 28.7,
        mass: 40_000.0,
        inertia_roll: 1_036_672.0,
        inertia_pitch: 713_656.0,
        inertia_yaw: 1_072_040.0,
        wing_area: 92.5,
        chord: 3.6,
        wing_height: -1.2,
        cl0: 0.30,
        cl_alpha: 5.0,
        alpha_stall: deg(15.5),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.021,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.95,
        flap_curve: 1.0,
        flap_alpha_stall: deg(4.0),
        flap_cd: 0.075,
        flap_cd_exp: 1.6,
        flap_cm: -0.06,
        cm0: 0.04,
        cm_alpha: -1.3,
        cm_q: -28.0,
        cm_elevator: 0.7,
        cm_stall: -0.12,
        trim_authority: 0.5,
        cy_beta: -0.7,
        cl_beta: -0.08,
        cl_p: -0.5,
        stall_autorotation: 0.1,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.03,
        cn_beta: 0.10,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.06,
        cn_aileron: -0.002,
        surface_rate: 2.5,
        flap_rate: 0.06,
        static_thrust: 181_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 2.74,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 6.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: MERIDIAN190_GEAR,
        hard_points: MERIDIAN190_HARD,
        roll_mu: 0.02,
        brake_mu: 0.5,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(8.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 3.75,
        g_limit_neg: -1.5,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 2,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.53, 1.45, 11.75),
        vr_kt: 135.0,
        vref_kt: 128.0,
        vle_kt: 250.0,
        ceiling_m: 12_497.0,
        mmo: 0.82,
        gear_time: 9.0,
        cd_gear: 0.018,
        spoiler_cl: 0.45,
        spoiler_cd: 0.08,
        spoiler_flight_max: 0.6,
        mach_dd: 0.80,
        cd_wave: 0.05,
        engine_pos: MERIDIAN190_ENGINES,
        n1_idle: 0.22,
        idle_thrust_frac: 0.04,
        thrust_mach: [-0.85, 0.35],
        n1_rpm: 7_400.0,
        sfc: 0.037,
        egt_idle: 420.0,
        egt_max: 870.0,
        reverse_eff: 0.40,
        yaw_damper: 2.5,
        // E190s operate London City (1,508 m) with the steep-approach option.
        min_runway_m: 1_450.0,
        ..Default::default()
    }
}

fn meridian320_base() -> AircraftDef {
    AircraftDef {
        id: "meridian320",
        name: "Airbus A320-200",
        description: "Narrow-body twin-jet (FlightGear A320-family, JSBSim aerodynamics). \
                      2 × CFM56-5B4, normal-law FBW with protections, spoilers, reversers.",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 10.0, 15.0, 20.0, 40.0],
        flap_max_deg: 40.0,
        vs0_kt: 118.0,
        vs1_kt: 157.0,
        vno_kt: 350.0,
        vne_kt: 350.0,
        gear_height: 3.54,
        wingspan: 35.8,
        mass: 60_443.0,
        inertia_roll: 1_112_113.0,
        inertia_pitch: 2_941_751.0,
        inertia_yaw: 2_461_842.0,
        wing_area: 122.6,
        chord: 4.19,
        wing_height: -1.4,
        cl0: 0.30,
        cl_alpha: 5.2,
        alpha_stall: deg(14.5),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.0195,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.95,
        flap_curve: 1.0,
        flap_alpha_stall: deg(4.0),
        flap_cd: 0.075,
        flap_cd_exp: 1.6,
        flap_cm: -0.06,
        cm0: 0.03,
        cm_alpha: -1.2,
        cm_q: -30.0,
        cm_elevator: 0.7,
        cm_stall: -0.12,
        trim_authority: 0.5,
        cy_beta: -0.7,
        cl_beta: -0.08,
        cl_p: -0.5,
        stall_autorotation: 0.1,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.03,
        cn_beta: 0.10,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.06,
        cn_aileron: -0.002,
        surface_rate: 2.5,
        flap_rate: 0.06,
        static_thrust: 240_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 4.70,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 6.5,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: MERIDIAN320_GEAR,
        hard_points: MERIDIAN320_HARD,
        roll_mu: 0.02,
        brake_mu: 0.5,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(8.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 3.75,
        g_limit_neg: -1.5,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 2,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.55, 1.15, 13.55),
        vr_kt: 145.0,
        vref_kt: 142.0,
        vle_kt: 280.0,
        ceiling_m: 12_131.0,
        mmo: 0.82,
        control_law: ControlLaw::Airliner,
        gear_time: 10.0,
        cd_gear: 0.016,
        spoiler_cl: 0.45,
        spoiler_cd: 0.08,
        spoiler_flight_max: 0.5,
        mach_dd: 0.80,
        cd_wave: 0.06,
        engine_pos: MERIDIAN320_ENGINES,
        n1_idle: 0.21,
        idle_thrust_frac: 0.04,
        thrust_mach: [-0.85, 0.35],
        n1_rpm: 5_000.0,
        sfc: 0.036,
        egt_idle: 400.0,
        egt_max: 900.0,
        reverse_eff: 0.40,
        alpha_max: deg(14.0),
        nz_max: 2.5,
        nz_min: -1.0,
        roll_rate_max: deg(15.0),
        yaw_damper: 2.5,
        min_runway_m: 1_800.0,
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// v3 heavies: C-130H Hercules, C-5M Super Galaxy, Avro Vulcan B.2
// ---------------------------------------------------------------------------

// --- Hercules (C-130H) ---------------------------------------------------------

const HERCULES_GEAR: &[GearPoint] = &[
    // FGAddon C130 model, gear down (tandem main wheels in the fuselage pods).
    wheel(Vec3::new(0.0, -3.36, 9.10), 450_000.0, 45_000.0, true, false, true),
    wheel(Vec3::new(2.125, -3.36, 0.42), 1_400_000.0, 140_000.0, false, true, true),
    wheel(Vec3::new(-2.125, -3.36, 0.42), 1_400_000.0, 140_000.0, false, true, true),
    wheel(Vec3::new(2.125, -3.36, -1.11), 1_400_000.0, 140_000.0, false, true, true),
    wheel(Vec3::new(-2.125, -3.36, -1.11), 1_400_000.0, 140_000.0, false, true, true),
    // Ramp hinge / aft belly (over-rotation ~12°).
    skid(Vec3::new(0.0, -2.62, -4.45), 1_500_000.0, 150_000.0, 0.5),
];

const HERCULES_HARD: &[HardPoint] = &[
    hard(0.0, -1.25, 12.80, "Flew into the ground"),
    hard(20.15, 1.74, -0.23, "Wingtip strike"),
    hard(-20.15, 1.74, -0.23, "Wingtip strike"),
    hard(9.82, -0.89, 3.35, "Prop strike"),
    hard(-9.82, -0.89, 3.35, "Prop strike"),
    hard(4.93, -0.89, 3.35, "Prop strike"),
    hard(-4.93, -0.89, 3.35, "Prop strike"),
    hard(0.0, 1.95, 8.00, "Crashed inverted"),
    hard(0.0, 8.70, -13.30, "Crashed inverted"),
    hard(0.0, -2.55, 6.00, GEAR_UP),
    hard(0.0, -2.55, -2.50, GEAR_UP),
    hard(1.80, -2.50, -0.30, GEAR_UP),
    hard(-1.80, -2.50, -0.30, GEAR_UP),
];

/// Propeller hubs, engine 0 = left outboard (#1) .. 3 = right outboard (#4).
const HERCULES_ENGINES: &[Vec3] = &[Vec3::new(9.82, 1.17, 3.35), Vec3::new(4.93, 1.17, 3.35), Vec3::new(-4.93, 1.17, 3.35), Vec3::new(-9.82, 1.17, 3.35)];

fn hercules_def() -> AircraftDef {
    AircraftDef {
        id: "hercules",
        name: "Lockheed C-130H Hercules",
        description: "Four-turboprop tactical airlifter (FlightGear C130 model; flight model calibrated \
                      to its YASim data and published figures). 4 × T56-A-15, 4,590 shp, four-blade props, \
                      high wing, rough-field gear: short fields from ~700 m.",
        role: "Military transport",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 18.0, 36.0],
        flap_max_deg: 36.0,
        vs0_kt: 96.0,
        vs1_kt: 119.0,
        vno_kt: 250.0,
        vne_kt: 290.0,
        gear_height: 3.282,
        wingspan: 40.41,
        mass: 60_000.0,
        inertia_roll: 1_600_000.0,
        inertia_pitch: 2_300_000.0,
        inertia_yaw: 3_600_000.0,
        wing_area: 162.1,
        chord: 4.16,
        wing_height: 1.2,
        cl0: 0.45,
        cl_alpha: 5.0,
        alpha_stall: deg(15.4),
        alpha_stall_neg: deg(14.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.022,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.95,
        flap_curve: 0.6,
        flap_alpha_stall: deg(-1.0),
        flap_cd: 0.070,
        flap_cd_exp: 1.5,
        flap_cm: -0.05,
        cm0: 0.08,
        cm_alpha: -1.4,
        cm_q: -25.0,
        cm_elevator: 0.9,
        cm_stall: -0.10,
        trim_authority: 0.5,
        cy_beta: -0.6,
        cl_beta: -0.10,
        cl_p: -0.5,
        stall_autorotation: 0.15,
        cl_r_per_cl: 0.25,
        cl_aileron: 0.026,
        cn_beta: 0.08,
        cn_r: -0.12,
        cn_p: -0.03,
        cn_rudder: 0.055,
        cn_aileron: -0.003,
        surface_rate: 2.0,
        flap_rate: 0.12,
        static_thrust: 200_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.15,
        prop_disk_area: 53.1,
        prop_wash: 0.0,
        idle_power: 0.06,
        spool_time: 1.8,
        rpm_idle: 730.0,
        rpm_static: 1_020.0,
        rpm_per_ms: 6.0,
        gear: HERCULES_GEAR,
        hard_points: HERCULES_HARD,
        roll_mu: 0.02,
        brake_mu: 0.6,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.6,
        max_touchdown_bank: deg(10.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 3.0,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Turboprop,
        engine_count: 4,
        retractable_gear: true,
        has_reverser: true,
        eye: Vec3::new(0.6, 0.53, 9.96),
        vr_kt: 112.0,
        vref_kt: 125.0,
        vle_kt: 168.0,
        ceiling_m: 10_060.0,
        mmo: 0.55,
        gear_time: 10.0,
        cd_gear: 0.020,
        mach_dd: 0.62,
        cd_wave: 0.03,
        engine_pos: HERCULES_ENGINES,
        engine_power_w: 3_420_000.0,
        const_speed_prop: true,
        prop_eta: 0.84,
        prop_fm: 0.80,
        prop_radius: 2.055,
        prop_rotation: 1.0,
        prop_gear: 1.0,
        prop_inertia: 300.0,
        crit_alt_m: 0.0,
        lapse_exp: 0.75,
        torque_k: 1.0,
        swirl_k: 0.0,
        pfactor_k: 0.05,
        rig_kt: 200.0,
        sfc: 0.30,
        egt_idle: 520.0,
        egt_max: 1_070.0,
        reverse_eff: 0.60,
        yaw_damper: 2.0,
        min_runway_m: 900.0,
        rough_field: true,
        ..Default::default()
    }
}

// --- Galaxy (C-5M) --------------------------------------------------------------

const GALAXY_GEAR: &[GearPoint] = &[
    // FGMEMBERS C-5 model, gear down: four-wheel nose, four six-wheel main bogies.
    wheel(Vec3::new(0.0, -4.97, 19.15), 2_000_000.0, 800_000.0, true, false, true),
    wheel(Vec3::new(3.94, -4.97, -0.54), 3_500_000.0, 3_000_000.0, false, true, true),
    wheel(Vec3::new(-3.94, -4.97, -0.54), 3_500_000.0, 3_000_000.0, false, true, true),
    wheel(Vec3::new(3.94, -4.97, -5.60), 3_500_000.0, 3_000_000.0, false, true, true),
    wheel(Vec3::new(-3.94, -4.97, -5.60), 3_500_000.0, 3_000_000.0, false, true, true),
    // Aft fuselage / ramp (over-rotation ~11°).
    skid(Vec3::new(0.0, -1.41, -20.50), 6_000_000.0, 900_000.0, 0.5),
];

const GALAXY_HARD: &[HardPoint] = &[
    hard(0.0, -0.8, 29.50, "Flew into the ground"),
    hard(33.90, 1.70, -11.20, "Wingtip strike"),
    hard(-33.90, 1.70, -11.20, "Wingtip strike"),
    hard(19.15, -0.45, 2.40, "Engine pod strike"),
    hard(-19.15, -0.45, 2.40, "Engine pod strike"),
    hard(12.20, -0.15, 5.40, "Engine pod strike"),
    hard(-12.20, -0.15, 5.40, "Engine pod strike"),
    hard(0.0, 7.5, 16.50, "Crashed inverted"),
    hard(0.0, 16.4, -40.50, "Crashed inverted"),
    hard(0.0, -3.3, 13.50, GEAR_UP),
    hard(0.0, -3.0, -6.50, GEAR_UP),
];

/// Fan faces, engine 0 = left outboard .. 3 = right outboard.
const GALAXY_ENGINES: &[Vec3] = &[Vec3::new(19.15, 0.70, 2.42), Vec3::new(12.21, 1.39, 5.41), Vec3::new(-12.21, 1.39, 5.41), Vec3::new(-19.15, 0.70, 2.42)];

fn galaxy_def() -> AircraftDef {
    with_jsb(
        galaxy_base(),
        include_str!("../assets/aircraft/galaxy/fdm.xml"),
        &[],
        &[],
        200.0,
        |j| {
            j.elev_up = deg(25.0);
            j.elev_down = deg(20.0);
            j.ail_up = deg(20.0);
            j.ail_down = deg(20.0);
            j.rudder_max = deg(25.0);
            j.spoiler_max_deg = 45.0;
        },
    )
}

fn galaxy_base() -> AircraftDef {
    AircraftDef {
        id: "galaxy",
        name: "Lockheed C-5M Super Galaxy",
        description: "Strategic airlifter (FGMEMBERS C-5 model, JSBSim aerodynamics calibrated to \
                      published figures). 4 × GE CF6-80C2, 228 kN each, 280 t as flown: slow to spool, \
                      huge inertia, needs a long runway.",
        role: "Strategic airlifter",
        has_engine: true,
        flap_label: "Flaps",
        flap_detents: &[0.0, 8.0, 16.0, 25.0, 32.0, 40.0],
        flap_max_deg: 40.0,
        vs0_kt: 106.0,
        vs1_kt: 145.0,
        vno_kt: 350.0,
        vne_kt: 350.0,
        gear_height: 4.859,
        wingspan: 67.89,
        mass: 280_000.0,
        inertia_roll: 42_000_000.0,
        inertia_pitch: 30_900_000.0,
        inertia_yaw: 71_000_000.0,
        wing_area: 576.0,
        chord: 10.88,
        wing_height: 7.5,
        cl0: 0.40,
        cl_alpha: 4.4,
        alpha_stall: deg(13.5),
        alpha_stall_neg: deg(12.0),
        stall_sharpness: 35.0,
        cl_flat: 1.0,
        stall_drop: 0.75,
        stall_decay: 0.15,
        cd0: 0.020,
        oswald: 0.80,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 1.2,
        flap_curve: 0.5,
        flap_alpha_stall: 0.0,
        flap_cd: 0.08,
        flap_cd_exp: 1.0,
        flap_cm: -0.05,
        cm0: 0.0,
        cm_alpha: -0.6,
        cm_q: -17.0,
        cm_elevator: 0.5,
        cm_stall: -0.10,
        trim_authority: 0.9,
        cy_beta: -1.0,
        cl_beta: -0.10,
        cl_p: -0.4,
        stall_autorotation: 0.1,
        cl_r_per_cl: 0.2,
        cl_aileron: 0.035,
        cn_beta: 0.12,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.035,
        cn_aileron: 0.0,
        surface_rate: 2.0,
        flap_rate: 0.025,
        static_thrust: 912_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 17.5,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 8.5,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: GALAXY_GEAR,
        hard_points: GALAXY_HARD,
        roll_mu: 0.02,
        brake_mu: 0.5,
        tire_mu: 0.8,
        nose_steer_deg: 60.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(8.0),
        min_touchdown_pitch: deg(-1.5),
        g_limit_pos: 2.5,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 4,
        retractable_gear: true,
        has_spoilers: true,
        has_reverser: true,
        eye: Vec3::new(0.62, 4.15, 24.35),
        vr_kt: 145.0,
        vref_kt: 138.0,
        vle_kt: 250.0,
        ceiling_m: 12_500.0,
        mmo: 0.825,
        gear_time: 10.0,
        cd_gear: 0.025,
        spoiler_cl: 0.3,
        spoiler_cd: 0.05,
        spoiler_flight_max: 0.5,
        mach_dd: 0.80,
        cd_wave: 0.05,
        engine_pos: GALAXY_ENGINES,
        n1_idle: 0.22,
        idle_thrust_frac: 0.04,
        thrust_mach: [-0.85, 0.35],
        n1_rpm: 3_433.0,
        sfc: 0.034,
        egt_idle: 420.0,
        egt_max: 900.0,
        reverse_eff: 0.35,
        yaw_damper: 2.5,
        min_runway_m: 2_400.0,
        ..Default::default()
    }
}

// --- Vulcan (B.2) ---------------------------------------------------------------

const VULCAN_GEAR: &[GearPoint] = &[
    // FGMEMBERS AvroVulcanB2 model / FDM contacts, gear down.
    wheel(Vec3::new(0.0, -2.13, 6.92), 1_400_000.0, 140_000.0, true, false, true),
    wheel(Vec3::new(4.81, -2.13, -2.05), 2_200_000.0, 700_000.0, false, true, true),
    wheel(Vec3::new(-4.81, -2.13, -2.05), 2_200_000.0, 700_000.0, false, true, true),
    // Jet-pipe fairings / tail (tail-down limit ~10°).
    skid(Vec3::new(0.0, 0.05, -13.6), 2_000_000.0, 200_000.0, 0.5),
];

const VULCAN_HARD: &[HardPoint] = &[
    hard(0.0, -0.2, 16.5, "Flew into the ground"),
    hard(16.80, 0.28, -8.9, "Wingtip strike"),
    hard(-16.80, 0.28, -8.9, "Wingtip strike"),
    hard(9.0, -0.40, -6.0, "Wingtip strike"),
    hard(-9.0, -0.40, -6.0, "Wingtip strike"),
    hard(0.0, 1.9, 11.0, "Crashed inverted"),
    hard(0.0, 6.12, -12.98, "Crashed inverted"),
    hard(0.0, -1.0, 5.0, GEAR_UP),
    hard(0.0, -1.0, -3.0, GEAR_UP),
    hard(3.5, -0.9, -2.0, GEAR_UP),
    hard(-3.5, -0.9, -2.0, GEAR_UP),
];

/// Jet-pipe exits, engine 0 = left outer .. 3 = right outer (buried in the wing roots).
const VULCAN_ENGINES: &[Vec3] = &[Vec3::new(3.56, -0.71, -10.44), Vec3::new(2.34, -0.71, -10.44), Vec3::new(-2.34, -0.71, -10.44), Vec3::new(-3.56, -0.71, -10.44)];

fn vulcan_def() -> AircraftDef {
    with_jsb(vulcan_base(), include_str!("../assets/aircraft/vulcan/fdm.xml"), &[], &[], 250.0, |j| {
        // Elevons: ±20° pitch, ±20° roll; the flap lever drives the airbrakes.
        j.elev_up = 0.35;
        j.elev_down = 0.30;
        j.ail_up = 0.35;
        j.ail_down = 0.35;
        j.rudder_max = 0.35;
        j.flaps_are_airbrakes = true;
    })
}

fn vulcan_base() -> AircraftDef {
    AircraftDef {
        id: "vulcan",
        name: "Avro Vulcan B.2",
        description: "Delta-wing V-bomber (FGUK / FGMEMBERS model, JSBSim with Skyward delta-wing \
                      aerodynamics). 4 × Olympus 301, 89 kN each. No stall break — just rising drag \
                      and sink at high alpha. Flap lever = airbrakes.",
        role: "Delta-wing bomber",
        has_engine: true,
        flap_label: "Airbrakes",
        flap_detents: &[0.0, 0.5, 1.0],
        flap_max_deg: 0.0,
        vs0_kt: 88.0,
        vs1_kt: 88.0,
        vno_kt: 350.0,
        vne_kt: 415.0,
        gear_height: 2.051,
        wingspan: 33.83,
        mass: 60_000.0,
        inertia_roll: 1_690_000.0,
        inertia_pitch: 1_640_000.0,
        inertia_yaw: 3_380_000.0,
        wing_area: 368.4,
        chord: 10.89,
        wing_height: 0.3,
        cl0: 0.04,
        cl_alpha: 3.1,
        alpha_stall: deg(28.0),
        alpha_stall_neg: deg(20.0),
        stall_sharpness: 15.0,
        cl_flat: 1.0,
        stall_drop: 0.95,
        stall_decay: 0.3,
        cd0: 0.0125,
        oswald: 0.85,
        cd_profile_k: 0.0,
        cl_min_profile: 0.0,
        cd_flat: 1.3,
        cd_beta: 0.2,
        flap_cl: 0.0,
        flap_curve: 0.0,
        flap_alpha_stall: 0.0,
        flap_cd: 0.045,
        flap_cd_exp: 1.0,
        flap_cm: 0.0,
        cm0: 0.0,
        cm_alpha: -0.7,
        cm_q: -21.0,
        cm_elevator: 0.6,
        cm_stall: 0.0,
        trim_authority: 0.6,
        cy_beta: -1.0,
        cl_beta: -0.10,
        cl_p: -0.4,
        stall_autorotation: 0.05,
        cl_r_per_cl: 0.15,
        cl_aileron: 0.05,
        cn_beta: 0.12,
        cn_r: -0.15,
        cn_p: -0.02,
        cn_rudder: 0.05,
        cn_aileron: 0.0,
        surface_rate: 2.0,
        flap_rate: 0.4,
        static_thrust: 356_000.0,
        thrust_v1: 0.0,
        thrust_v2: 0.0,
        windmill_cd: 0.05,
        prop_disk_area: 4.0,
        prop_wash: 0.0,
        idle_power: 0.0,
        spool_time: 7.0,
        rpm_idle: 0.0,
        rpm_static: 0.0,
        rpm_per_ms: 0.0,
        gear: VULCAN_GEAR,
        hard_points: VULCAN_HARD,
        roll_mu: 0.02,
        brake_mu: 0.45,
        tire_mu: 0.8,
        nose_steer_deg: 50.0,
        max_touchdown_sink: 3.3,
        max_touchdown_bank: deg(5.0),
        min_touchdown_pitch: deg(-1.0),
        g_limit_pos: 3.0,
        g_limit_neg: -1.0,

        category: Category::Airliner,
        engine_kind: EngineKind::Jet,
        engine_count: 4,
        retractable_gear: true,
        eye: Vec3::new(0.34, 1.95, 11.85),
        vr_kt: 145.0,
        vref_kt: 150.0,
        vle_kt: 230.0,
        ceiling_m: 17_000.0,
        mmo: 0.93,
        gear_time: 10.0,
        cd_gear: 0.015,
        mach_dd: 0.88,
        cd_wave: 0.045,
        engine_pos: VULCAN_ENGINES,
        n1_idle: 0.28,
        idle_thrust_frac: 0.05,
        thrust_mach: [-0.55, 0.45],
        n1_rpm: 6_600.0,
        sfc: 0.084,
        egt_idle: 380.0,
        egt_max: 690.0,
        yaw_damper: 2.0,
        min_runway_m: 2_000.0,
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// Aircraft state
// ---------------------------------------------------------------------------

/// Body-frame attitude quaternion for heading / pitch / roll (radians).
pub fn attitude(heading: f32, pitch: f32, roll: f32) -> Quat {
    Quat::from_rotation_y(PI - heading) * Quat::from_rotation_x(-pitch) * Quat::from_rotation_z(roll)
}

fn lower_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_lowercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Per-aircraft seed for the structural strength scatter.
fn next_seed() -> u32 {
    static SEED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0x5EED);
    SEED.fetch_add(0x9E37_79B9, std::sync::atomic::Ordering::Relaxed)
}

/// Live state of one engine.
#[derive(Clone, Copy, Debug, Default)]
struct EngState {
    /// Power fraction (props) / N1 (jets).
    spool: f32,
    /// Gauge RPM (piston: crankshaft; turboprop: prop Np; jet: N1 RPM).
    rpm: f32,
    prop_angle: f32,
    /// Last net thrust, N.
    thrust: f32,
    shaft_power: f32,
    ab: f32,
    egt: f32,
    fuel_flow: f32,
    running: bool,
}

/// Live aircraft state.
#[derive(Clone, Debug)]
pub struct Aircraft {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    pub on_ground: bool,
    /// Set (and the state frozen) when the aircraft is destroyed.
    pub crashed: Option<String>,
    /// Body-frame angular velocity (rad/s). See module docs for p/q/r.
    pub omega: Vec3,
    surf: Surfaces,
    stick_elevator: f32,
    trim: f32,
    air_vel: Vec3,
    alpha: f32,
    beta: f32,
    g_load: f32,
    g_filtered: f32,
    side_g: f32,
    alpha_peak: f32,
    peak_flap: f32,
    // v2
    eng: [EngState; MAX_ENGINES],
    gear_pos: f32,
    ias: f32,
    mach: f32,
    oat: f32,
    /// 0 = direct (ground) .. 1 = FBW flight law.
    law_blend: f32,
    nz_int: f32,
    q_int: f32,
    /// Parking-brake stiction anchor (world XZ) while held stationary.
    brake_anchor: Option<Vec2>,
    /// Wind as the airframe feels it (span-averaged turbulence).
    wind_f: Option<Vec3>,
    /// Tyre / strut compression per gear point (m), for the model's struts.
    gear_comp: [f32; 8],
    /// JSBSim aerodynamics state (stall hysteresis).
    jsb: crate::jsbsim::JsbState,
    /// Airliner FBW: bank attitude held with the stick neutral.
    bank_hold: Option<f32>,
    /// Structural state: wing bending, damage, separated parts (damage.rs).
    pub dmg: crate::damage::Damage,
    /// The state at the moment of a crash (before it was frozen): the
    /// wreck / debris simulation continues from here.
    pub impact: Option<Impact>,
    /// Aircraft systems and failures (src/systems): electrics, fuel,
    /// hydraulics, engines' accessories, instruments. Stepped by
    /// [`crate::systems::step`]; only its `fx` is read here (neutral
    /// unless something has failed or been switched off).
    pub sys: crate::systems::Systems,
    /// Rotorcraft state (rotor speed, inflow, engine, assist; src/rotor).
    pub rotor: crate::rotor::RotorState,
    /// Trim-fuel CG travel aft of the loaded CG, m (`AircraftDef::cg_schedule`).
    cg_aft: f32,
    /// Floats / hull on the water: sea state and the last contact (src/water.rs).
    pub water: crate::water::WaterState,
    /// Damage / crash assists (settings; the sim sets them each step).
    pub assists: crate::damage::Assists,
    /// A crash the crash assists may let the flight survive: decided at the
    /// start of the next step (which knows the aircraft and the terrain).
    pending_crash: Option<String>,
}

/// Kinematic state captured when the aircraft crashed.
#[derive(Clone, Copy, Debug)]
pub struct Impact {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    /// Body-frame angular velocity.
    pub omega: Vec3,
    pub on_ground: bool,
}

impl Aircraft {
    /// The world origin moved (floating origin): every world position shifts
    /// by `delta`. Velocities, attitude and forces are unchanged.
    pub fn rebase(&mut self, delta: Vec3) {
        self.pos += delta;
        self.rotor.quad.home += delta;
        if let Some(a) = &mut self.brake_anchor {
            *a += Vec2::new(delta.x, delta.z);
        }
    }

    fn blank(def: &AircraftDef) -> Self {
        let mut a = Aircraft {
            pos: Vec3::ZERO,
            vel: Vec3::ZERO,
            rot: Quat::IDENTITY,
            on_ground: false,
            crashed: None,
            omega: Vec3::ZERO,
            surf: Surfaces::default(),
            stick_elevator: 0.0,
            trim: 0.0,
            air_vel: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            g_load: 1.0,
            g_filtered: 1.0,
            side_g: 0.0,
            alpha_peak: def.alpha_usable(0.0),
            peak_flap: 0.0,
            eng: [EngState::default(); MAX_ENGINES],
            gear_pos: 1.0,
            ias: 0.0,
            mach: 0.0,
            oat: 15.0,
            law_blend: 0.0,
            nz_int: 0.0,
            q_int: 0.0,
            brake_anchor: None,
            wind_f: None,
            gear_comp: [0.0; 8],
            jsb: Default::default(),
            bank_hold: None,
            dmg: crate::damage::Damage::new(next_seed()),
            impact: None,
            sys: Default::default(),
            rotor: Default::default(),
            cg_aft: 0.0,
            water: Default::default(),
            assists: Default::default(),
            pending_crash: None,
        };
        a.settle_engines(def, 0.0, 0.0, &isa(0.0));
        a
    }

    /// Put every running engine in the steady state for `throttle`.
    fn settle_engines(&mut self, def: &AircraftDef, throttle: f32, v_fwd: f32, atm: &Atmosphere) {
        let (spool, ab) = def.spool_for_throttle(throttle);
        let oat = atm.temperature_c();
        for i in 0..def.engines() {
            let out = def.engine_output(spool, ab, 0.0, v_fwd, atm, false);
            let e = &mut self.eng[i];
            e.running = true;
            e.spool = spool;
            e.ab = ab;
            e.rpm = def.target_rpm(spool, v_fwd, true);
            e.thrust = out.thrust;
            e.shaft_power = out.shaft_power;
            e.egt = Self::egt_target(def, spool, oat, true);
            e.fuel_flow = Self::fuel_flow_for(def, &out, v_fwd / atm.speed_of_sound, true);
        }
        self.surf.afterburner = if def.engines() > 0 { ab } else { 0.0 };
    }

    fn egt_target(def: &AircraftDef, spool: f32, oat: f32, running: bool) -> f32 {
        if !running || !def.has_engine {
            return oat;
        }
        let load = match def.engine_kind {
            EngineKind::Jet => ((spool - def.n1_idle) / (1.0 - def.n1_idle)).clamp(0.0, 1.05),
            _ => ((spool - def.idle_power) / (1.0 - def.idle_power)).clamp(0.0, 1.0),
        };
        oat - 15.0 + def.egt_idle + (def.egt_max - def.egt_idle) * load.powf(1.5)
    }

    fn fuel_flow_for(def: &AircraftDef, out: &EngineOut, mach: f32, running: bool) -> f32 {
        if !running {
            return 0.0;
        }
        match def.engine_kind {
            EngineKind::None | EngineKind::Electric => 0.0,
            EngineKind::Piston | EngineKind::Turboprop | EngineKind::Turboshaft => def.sfc * out.shaft_power / 1000.0,
            EngineKind::Jet => {
                let dry = (out.gross - out.ab_thrust).max(0.0);
                def.sfc * (1.0 + 0.55 * mach.clamp(0.0, 2.5)) * dry + def.sfc_ab * out.ab_thrust
            }
        }
    }

    /// At rest on the ground at `xz`, engines idling, brakes released, gear down.
    pub fn on_runway(def: &AircraftDef, xz: Vec2, heading_rad: f32, env: &dyn Environment) -> Self {
        let mut a = Self::blank(def);
        let gh = finite_or(env.ground_height(xz), 0.0);
        // A floatplane on land (a ramp) rests on its keels, not at its waterline.
        let h0 = if def.on_water() && !env.is_water(xz) { def.gear.iter().map(|g| -g.pos.y).fold(def.gear_height, f32::max) } else { def.gear_height };
        a.pos = Vec3::new(xz.x, gh + h0, xz.y);
        a.rot = attitude(heading_rad, def.rest_pitch(), 0.0);
        a.on_ground = true;
        // Let the gear settle onto its springs, then zero any residual motion.
        let hold = Controls { brakes: 1.0, ..Default::default() };
        for _ in 0..480 {
            a.step(def, &hold, env, 1.0 / 120.0);
        }
        let (pitch, roll) = (a.pitch(), a.roll());
        let y = a.pos.y;
        a.crashed = None;
        a.pos = Vec3::new(xz.x, y, xz.y);
        a.rot = attitude(heading_rad, pitch, roll);
        a.vel = Vec3::ZERO;
        a.omega = Vec3::ZERO;
        a.air_vel = Vec3::ZERO;
        a.on_ground = true;
        a.surf = Surfaces::default();
        a.settle_engines(def, 0.0, 0.0, &isa(a.pos.y));
        a.stick_elevator = 0.0;
        a.g_load = 1.0;
        a.g_filtered = 1.0;
        a.ias = 0.0;
        a.mach = 0.0;
        a.law_blend = 0.0;
        a.impact = None;
        a.dmg.events.clear();
        a
    }

    /// In steady straight flight at indicated airspeed `speed_ms`: level for
    /// powered aircraft (engine power set to hold it, see
    /// [`trimmed_controls_at`]), on the steady glide path for gliders.
    /// Retractable gear starts up (clean) unless `speed_ms` is below
    /// 1.15 × Vref, where it starts down; check [`Aircraft::gear_pos`] and
    /// set `Controls::gear_down` to match.
    pub fn airborne(def: &AircraftDef, pos: Vec3, heading_rad: f32, speed_ms: f32) -> Self {
        if let Some(rd) = def.rotor.clone() {
            return Self::airborne_rotor(&rd, def, pos, heading_rad, speed_ms);
        }
        let mut a = Self::blank(def);
        let speed = finite_or(speed_ms, 30.0).max(1.0);
        let gear_down = !def.retractable_gear || speed < 1.15 * def.vref_kt * KT;
        let alt = finite_or(pos.y, 0.0);
        let tp = def.trim_point_at(speed, 0.0, alt, gear_down);
        let atm = isa(alt);
        let tas = tas_from_cas(speed, alt);
        a.pos = pos;
        a.rot = attitude(heading_rad, tp.pitch, 0.0);
        let dir = attitude(heading_rad, tp.gamma, 0.0) * Vec3::Z;
        a.vel = dir * tas;
        a.air_vel = a.vel;
        a.alpha = tp.alpha;
        a.gear_pos = if gear_down { 1.0 } else { 0.0 };
        a.surf.gear = 1.0 - a.gear_pos;
        a.settle_engines(def, tp.throttle, tas * tp.alpha.cos(), &atm);
        a.trim = tp.trim;
        a.surf.elevator = if def.control_law == ControlLaw::Direct {
            (tp.trim * def.trim_authority).clamp(-1.0, 1.0)
        } else {
            tp.elevator.clamp(-1.0, 1.0)
        };
        a.ias = speed;
        a.mach = tas / atm.speed_of_sound;
        a.cg_aft = def.cg_scheduled(a.mach);
        a.oat = atm.temperature_c();
        a.law_blend = if def.control_law == ControlLaw::Direct { 0.0 } else { 1.0 };
        // Wings already bent by the 1 g flight load.
        let sd = crate::damage::struct_def(def.id);
        a.dmg.flex = [sd.tip_per_g; 2];
        a
    }

    pub fn forward(&self) -> Vec3 {
        self.rot * Vec3::Z
    }
    pub fn up(&self) -> Vec3 {
        self.rot * Vec3::Y
    }
    pub fn right(&self) -> Vec3 {
        self.rot * Vec3::NEG_X
    }
    /// Radians, 0 = north, π/2 = east, in -π..π.
    pub fn heading(&self) -> f32 {
        let f = self.forward();
        if f.x.abs() + f.z.abs() < 1e-6 {
            // Pointing straight up/down: use the up vector's tail instead.
            let u = self.up();
            let s = if f.y > 0.0 { -1.0 } else { 1.0 };
            return (s * u.x).atan2(-s * u.z);
        }
        f.x.atan2(-f.z)
    }
    /// Radians, + = nose up.
    pub fn pitch(&self) -> f32 {
        self.forward().y.clamp(-1.0, 1.0).asin()
    }
    /// Radians, + = right wing down.
    pub fn roll(&self) -> f32 {
        let r = self.right();
        let u = self.up();
        (-r.y).atan2(u.y)
    }
    /// Indicated (calibrated) airspeed, m/s — what the ASI shows. Equal to the
    /// true airspeed at sea level, lower at altitude.
    pub fn airspeed(&self) -> f32 {
        self.ias
    }
    /// Indicated airspeed in knots.
    pub fn airspeed_kt(&self) -> f32 {
        self.airspeed() / KT
    }
    /// Wind (incl. updraft) as an air-data / GPS computer sees it: ground
    /// velocity minus air velocity (m/s, the direction the air moves).
    pub fn wind_estimate(&self) -> Vec3 {
        self.vel - self.air_vel
    }
    /// True airspeed (m/s), relative to the air mass including updraft.
    pub fn tas(&self) -> f32 {
        self.air_vel.length()
    }
    /// Mach number.
    pub fn mach(&self) -> f32 {
        self.mach
    }
    /// Outside air temperature, °C.
    pub fn oat(&self) -> f32 {
        self.oat
    }
    /// Vertical speed over the ground, m/s.
    pub fn vertical_speed(&self) -> f32 {
        self.vel.y
    }
    /// Angle of attack, rad.
    pub fn alpha(&self) -> f32 {
        self.alpha
    }
    /// Sideslip angle, rad (+ = air coming from the right).
    pub fn beta(&self) -> f32 {
        self.beta
    }
    /// Load factor along body up (1.0 in level flight / at rest).
    pub fn g_load(&self) -> f32 {
        self.g_load
    }
    /// Lateral specific force in g (slip-ball deflection; + = ball to the right).
    pub fn slip(&self) -> f32 {
        self.side_g
    }
    /// Body rates (p roll right, q pitch up, r yaw right), rad/s.
    pub fn rates(&self) -> Vec3 {
        Vec3::new(self.omega.z, -self.omega.x, -self.omega.y)
    }
    /// Rate of change of heading, rad/s (+ = turning right).
    pub fn turn_rate(&self) -> f32 {
        // World angular velocity about the vertical axis; heading grows clockwise.
        let w_world = self.rot * self.omega;
        -w_world.y
    }
    /// Trim-fuel CG travel aft of the loaded CG, m (Concorde; 0 for others).
    pub fn cg_aft(&self) -> f32 {
        self.cg_aft
    }
    /// Stall-warning horn: within ~3° of the stall angle (or 2° of the FBW AoA
    /// limit) while airborne.
    pub fn stall_warning(&self) -> bool {
        !self.on_ground && self.crashed.is_none() && self.tas() > 5.0 && self.alpha > self.alpha_peak - deg(3.0)
    }
    /// Engine 0 gauge RPM (0 for gliders).
    pub fn engine_rpm(&self) -> f32 {
        self.eng[0].rpm
    }
    /// Engine 0 power fraction 0..1 (props) or N1 (jets); lags the throttle.
    pub fn engine_power(&self) -> f32 {
        self.eng[0].spool
    }
    /// Smoothed actual control-surface deflections.
    pub fn surfaces(&self) -> Surfaces {
        self.surf
    }
    /// Height of the CG above the terrain.
    pub fn agl(&self, env: &dyn Environment) -> f32 {
        self.pos.y - env.ground_height(Vec2::new(self.pos.x, self.pos.z))
    }

    // --- v2 instruments ---

    /// Number of engines (the `def` is needed because a glider has none).
    pub fn engine_count(&self, def: &AircraftDef) -> usize {
        def.engines()
    }
    /// Gauge RPM of engine `i` (piston: crankshaft, turboprop: propeller Np,
    /// jet: N1 RPM). 0 for a missing engine.
    pub fn engine_rpm_i(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(0.0, |e| e.rpm)
    }
    /// Jet fan speed N1 as a fraction (0 .. ~1.05); props: power fraction.
    pub fn n1(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(0.0, |e| e.spool)
    }
    /// Exhaust gas (turboprop: inter-turbine) temperature of engine `i`, °C.
    pub fn egt(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(self.oat, |e| e.egt)
    }
    /// Fuel flow of engine `i`, kg/h.
    pub fn fuel_flow(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(0.0, |e| e.fuel_flow)
    }
    /// Net thrust of engine `i`, N (negative in reverse / windmilling).
    pub fn engine_thrust(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(0.0, |e| e.thrust)
    }
    /// Shaft power of engine `i`, W (props).
    pub fn shaft_power(&self, i: usize) -> f32 {
        self.eng.get(i).map_or(0.0, |e| e.shaft_power)
    }
    /// True if engine `i` is running (not failed / shut down).
    pub fn engine_running(&self, i: usize) -> bool {
        self.eng.get(i).is_some_and(|e| e.running)
    }
    /// Landing gear position: 0 = up and stowed .. 1 = down and locked.
    /// Compression of gear point `i`: (metres, 0..1 of a nominal 0.25 m stroke).
    pub fn gear_compression(&self, i: usize) -> (f32, f32) {
        let m = self.gear_comp.get(i).copied().unwrap_or(0.0).max(0.0);
        (m, (m / 0.25).min(1.0))
    }

    pub fn gear_pos(&self) -> f32 {
        self.gear_pos
    }
    /// Test / scenario helpers: gear and flaps set instantly.
    pub fn retract_gear_now(&mut self) {
        self.gear_pos = 0.0;
        self.surf.gear = 1.0;
    }
    pub fn extend_gear_now(&mut self) {
        self.gear_pos = 1.0;
        self.surf.gear = 0.0;
    }
    pub fn set_flaps_now(&mut self, f: f32) {
        self.surf.flaps = f.clamp(0.0, 1.0);
    }
    /// Afterburner level 0..1 (max over engines).
    pub fn afterburner(&self) -> f32 {
        self.surf.afterburner
    }
    /// 0 = direct law .. 1 = full FBW flight law (blends at lift-off / touchdown).
    pub fn fbw_active(&self) -> f32 {
        self.law_blend
    }

    fn crash(&mut self, reason: String) {
        // Crash assists: decide next step whether the flight goes on.
        if self.crashed.is_none() && self.assists.crashes != crate::damage::Realism::Realistic && !reason.starts_with("Simulation fault") {
            if self.pending_crash.is_none() {
                self.pending_crash = Some(reason);
            }
            return;
        }
        self.crash_now(reason);
    }

    fn crash_now(&mut self, reason: String) {
        if self.crashed.is_none() {
            self.impact = Some(Impact { pos: self.pos, vel: self.vel, rot: self.rot, omega: self.omega, on_ground: self.on_ground });
            // After a structural failure in the air, that is the cause.
            let reason = match &self.dmg.failure {
                Some(f) if !reason.starts_with("Structural") => format!("{f}; {}", lower_first(&reason)),
                _ => reason,
            };
            self.crashed = Some(reason);
        }
        self.vel = Vec3::ZERO;
        self.omega = Vec3::ZERO;
        self.air_vel = Vec3::ZERO;
        for e in &mut self.eng {
            e.spool = 0.0;
            e.rpm = 0.0;
            e.thrust = 0.0;
            e.ab = 0.0;
            e.fuel_flow = 0.0;
            e.shaft_power = 0.0;
        }
        self.surf.afterburner = 0.0;
        self.ias = 0.0;
    }

    /// Crash assists (`assists.crashes`): Forgiving — anything short of a
    /// high-energy impact bounces / scrapes on; Off — nothing ends the
    /// flight: minor contacts bounce, the rest resets the aircraft to a safe
    /// state (level, 300 m above the ground, repaired). Returns false when the
    /// crash stands (the aircraft is now crashed).
    fn survive_crash(&mut self, def: &AircraftDef, env: &dyn Environment, reason: String) -> bool {
        use crate::damage::{Realism, Severity};
        let mode = self.assists.crashes;
        let structural = self.dmg.failure.is_some();
        // Judged by the impact energy (a hard landing that "broke up" the
        // airframe in Realistic is just a hard landing here).
        let kind = if reason.starts_with("Hard landing") { "hard landing" } else { reason.as_str() };
        let sev = crate::damage::classify(def, kind, self.vel, structural);
        let xz = Vec2::new(self.pos.x, self.pos.z);
        let water = env.is_water(xz) && !def.on_water() && (reason.contains("water") || reason.contains("Ditch"));
        let note = |a: &mut Self, msg: String| a.dmg.events.push(crate::damage::DamageEvent { part: None, msg, caution: true });
        match mode {
            Realism::Realistic => {}
            Realism::Forgiving if sev == Severity::Destroyed || water => {}
            Realism::Off if sev == Severity::Destroyed || water || structural => {
                // Reset: level flight 300 m up at a safe speed, airframe repaired.
                let ground = finite_or(env.ground_height(xz), 0.0).max(finite_or(env.obstacle_top(xz).unwrap_or(f32::MIN), f32::MIN));
                let speed = self.vel.length().max(1.3 * def.vref_kt.max(def.vs1_kt) * KT).min(def.vno_kt.max(60.0) * KT);
                let (assists, sys, heading) = (self.assists, self.sys.clone(), self.heading());
                *self = Aircraft::airborne(def, Vec3::new(self.pos.x, ground + 300.0, self.pos.z), heading, speed);
                self.assists = assists;
                self.sys = sys;
                self.dmg.mode = assists.structural;
                note(self, format!("{reason} — reset (crashes off)"));
                return true;
            }
            _ => {
                // Bounce / scrape: wings level, nose no lower than level, on the
                // gear (or over the obstacle), the sink and some speed gone.
                let ground = finite_or(env.ground_height(xz), 0.0);
                let top = env.obstacle_top(xz).filter(|t| t.is_finite() && *t > ground + 1.0);
                let rest = def.gear.iter().map(|g| -g.pos.y).fold(0.5f32, f32::max) + 0.05;
                let (heading, pitch) = (self.heading(), self.pitch().clamp(0.0, deg(10.0)));
                self.rot = attitude(heading, pitch, 0.0);
                self.omega = Vec3::ZERO;
                self.dmg.side_f = 0.0;
                let h = Vec3::new(self.vel.x, 0.0, self.vel.z);
                self.vel = h * if top.is_some() { 0.6 } else { 0.85 };
                self.pos.y = match top {
                    Some(t) => t + rest + 2.0,
                    None => ground + rest,
                };
                self.on_ground = top.is_none();
                note(self, format!("{reason} — {} (crash assists)", if top.is_some() { "scraped over" } else { "bounced" }));
                return true;
            }
        }
        self.crash_now(reason);
        false
    }

    /// Advance the simulation by `dt` seconds (designed for 1/120 s).
    pub fn step(&mut self, def: &AircraftDef, ctl: &Controls, env: &dyn Environment, dt: f32) {
        if self.crashed.is_some() || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        self.dmg.mode = self.assists.structural;
        if let Some(reason) = self.pending_crash.take() {
            if !self.survive_crash(def, env, reason) {
                return;
            }
        }
        if let Some(rd) = def.rotor.clone() {
            // Rotorcraft: own forces / drive train (src/rotor), shared contact and integration.
            return self.step_rotor(&rd, def, ctl, env, dt);
        }
        let dt = dt.min(1.0 / 30.0);
        let mut c = ctl.sanitized();
        // Separated engines are gone for good.
        c.failed_engines |= self.dmg.failed_engine_bits();
        // Systems (fuel starvation, shut-down / failed engines, brakes; systems/).
        let fx = self.sys.fx;
        c.failed_engines |= fx.engine_cut;
        c.brakes *= fx.brakes;
        let sd = crate::damage::struct_def(def.id);
        let gear_before = self.gear_pos;
        let saved = self.clone();

        // ---- air data -----------------------------------------------------
        let updraft = finite_or(env.updraft(self.pos), 0.0);
        let wind = env.wind(self.pos);
        let wind = if wind.is_finite() { wind.clamp_length_max(150.0) } else { Vec3::ZERO };
        // The airframe averages the air over its own size: eddies smaller than
        // the wingspan push on one part of it and pull on another, so the
        // point-sampled turbulence is smoothed over the time the aircraft
        // takes to fly through its span (no high-frequency shake).
        let tau = (def.wingspan * 0.8 / self.vel.length().max(15.0)).clamp(0.04, 0.6);
        let wind = match self.wind_f {
            Some(w) => w + (wind - w) * (dt / tau).min(1.0),
            None => wind,
        };
        self.wind_f = Some(wind);
        // Air-relative velocity: airspeed differs from groundspeed by the wind.
        let air_world = self.vel - wind - Vec3::Y * updraft;
        let inv = self.rot.conjugate();
        let vb = inv * air_world;
        let ground_h = finite_or(env.ground_height(Vec2::new(self.pos.x, self.pos.z)), 0.0);
        let agl = self.pos.y - ground_h;
        let atm = isa(self.pos.y);
        let tas = vb.length();
        self.mach = tas / atm.speed_of_sound;
        self.ias = cas_from_qc(impact_pressure(atm.pressure, self.mach));
        self.oat = atm.temperature_c();
        // Trim-fuel transfer follows its Mach schedule at the pumps' rate.
        if !def.cg_schedule.is_empty() {
            self.cg_aft = move_toward(self.cg_aft, def.cg_scheduled(self.mach), def.cg_rate * dt);
        }

        // ---- landing gear, spoilers, reversers ------------------------------
        if def.retractable_gear {
            // Weight-on-wheels interlock: no retraction on the ground.
            let target = if c.gear_down || self.on_ground { 1.0 } else { 0.0 };
            if !self.dmg.gear_jam {
                // Hydraulics / uplocks / alternate extension (systems/).
                let k = if target > self.gear_pos { fx.gear_down } else { fx.gear_up };
                self.gear_pos = move_toward(self.gear_pos, target, dt / def.gear_time.max(0.5) * k);
            }
        } else {
            self.gear_pos = 1.0;
        }
        self.surf.gear = 1.0 - self.gear_pos;
        let rev_req = def.has_reverser && c.reverse;
        self.surf.reverser = move_toward(self.surf.reverser, if rev_req && self.on_ground { 1.0 } else { 0.0 }, dt / 1.5);
        let sp_target = if !def.has_spoilers {
            0.0
        } else if self.on_ground && def.spoiler_cl > 0.0 && (c.spoilers > 0.05 || rev_req) {
            1.0
        } else {
            c.spoilers * def.spoiler_flight_max
        } * fx.spoilers;
        self.surf.spoilers = move_toward(self.surf.spoilers, sp_target, 1.5 * dt);

        // ---- engines ------------------------------------------------------
        self.update_engines(def, &c, &atm, vb, dt);
        let n_eng = def.engines();
        let mut f_eng = Vec3::ZERO;
        let mut m_eng = Vec3::ZERO;
        let mut thrust_total = 0.0;
        let mut h_prop = 0.0;
        for i in 0..n_eng {
            let e = self.eng[i];
            let r = def.engine_pos[i];
            let f = Vec3::Z * e.thrust;
            f_eng += f;
            m_eng += r.cross(f);
            thrust_total += e.thrust;
            if matches!(def.engine_kind, EngineKind::Piston | EngineKind::Turboprop) {
                let prop_rpm = e.rpm * def.prop_gear;
                let out = EngineOut { thrust: e.thrust, gross: e.thrust.max(0.0), shaft_power: e.shaft_power, ab_thrust: 0.0 };
                let (roll, yaw) = def.prop_moments(&out, prop_rpm, vb.z, vb, atm.density);
                m_eng += Vec3::new(0.0, -yaw, roll);
                h_prop += def.prop_rotation * def.prop_inertia * prop_rpm * TAU / 60.0;
            }
        }
        let q_slip = def.slipstream_q(thrust_total);

        // ---- control law & actuators --------------------------------------
        let surf_before = self.surf;
        self.actuate(def, &c, vb, agl, &atm, q_slip, m_eng, dt);
        crate::systems::apply_actuators(&fx, &surf_before, &mut self.surf);
        self.dmg.apply_jams(&mut self.surf);

        // ---- aerodynamics -------------------------------------------------
        let mut aero = self.aero(def, vb, agl, &atm, q_slip);
        if fx.ice > 0.0 || fx.flap_asym != 0.0 {
            // Airframe ice (lift loss growing towards the stall, drag) and split flaps (systems/).
            let q_s = 0.5 * atm.density * vb.length_squared() * def.wing_area;
            let (f, m) = crate::systems::airframe_forces(&fx, def, vb, aero.force, aero.alpha, self.surf.flaps, q_s);
            aero.force += f;
            aero.moment += m;
        }
        self.jsb = aero.jsb;
        self.alpha = aero.alpha;
        self.beta = aero.beta;
        self.air_vel = air_world;
        // Structural loads: lift and the rolling moment (less its rate damping)
        // decide how each half-wing is loaded.
        let lift_dir = {
            let vs = (vb.y * vb.y + vb.z * vb.z).sqrt();
            if vs > 1e-2 { Vec3::new(0.0, vb.z, -vb.y) / vs } else { Vec3::Y }
        };
        // Lost / damaged surfaces.
        let corr = crate::damage::aero_correction(sd, def, &self.dmg, vb, atm.density, aero.force + f_eng, &self.surf);
        aero.force += corr.force;
        aero.moment += corr.moment;
        aero.damping *= corr.damp_scale;
        let load_lift = aero.force.dot(lift_dir);
        let load_roll = aero.moment.z - aero.damping.z * self.omega.z;
        let f_body = aero.force + f_eng;
        let mut m_body = aero.moment + m_eng;
        if fx.cg != Vec3::ZERO {
            // Fuel imbalance: the weight acts off the centre of gravity (systems/).
            let w_body = inv * Vec3::new(0.0, -(self.dmg.mass(def) + fx.mass_delta) * GRAVITY, 0.0);
            m_body += fx.cg.cross(w_body);
        }

        // ---- ground -------------------------------------------------------
        if def.on_water() {
            self.water.update_sea(env, self.pos, dt);
        }
        let gnd = self.ground(def, env, &c, dt);
        self.water.last = gnd.water_contact;
        // Only the floats on the water: no brakes, no tyres.
        let water_only = gnd.water && !gnd.land;
        if let Some(reason) = gnd.crash {
            self.crash(reason);
            return;
        }
        m_body += gnd.moment_body;
        self.gear_comp = gnd.comp;
        let mut f_ground = gnd.force_world;

        // Static friction of held brakes (the velocity-level tyre model alone
        // lets a stationary aircraft creep under idle thrust).
        let gs = Vec2::new(self.vel.x, self.vel.z);
        if gnd.contact && !water_only && c.brakes > 0.3 && gs.length() < 0.3 {
            let here = Vec2::new(self.pos.x, self.pos.z);
            let anchor = *self.brake_anchor.get_or_insert(here);
            let m = def.mass;
            let f = -(here - anchor) * (16.0 * m) - gs * (5.6 * m);
            let cap = def.brake_mu * c.brakes * gnd.force_world.y.max(0.0) * 0.8;
            if f.length() > cap {
                self.brake_anchor = None;
            } else {
                f_ground += Vec3::new(f.x, 0.0, f.y);
            }
        } else {
            self.brake_anchor = None;
        }

        // Touchdown assessment.
        if gnd.contact && !self.on_ground {
            let sink = -self.vel.y;
            let bank = self.roll();
            let pitch = self.pitch();
            if sink > 3.0 * def.max_touchdown_sink {
                self.crash(format!("Flew into the ground at {:.0} fpm", sink / FPM));
                return;
            }
            if sink > def.max_touchdown_sink {
                // Up to ~1.7x the design sink rate the gear legs fail (the
                // airframe survives and slides on its belly); beyond, the
                // fuselage breaks.
                let how = if sink > 1.7 * def.max_touchdown_sink || def.category == Category::Glider {
                    "airframe broke up"
                } else if water_only {
                    "float struts collapsed"
                } else {
                    // (With crash assists the gear survives a survivable landing.)
                    let realistic = self.assists.crashes == crate::damage::Realism::Realistic;
                    for p in [crate::damage::Part::GearL, crate::damage::Part::GearR, crate::damage::Part::GearNose].into_iter().filter(|_| realistic) {
                        if def.gear.iter().any(|g| crate::damage::gear_part(def, g) == Some(p)) {
                            self.dmg.separate(sd, def, p, "hard landing");
                        }
                    }
                    "landing gear collapsed"
                };
                self.crash(format!("Hard landing: touched down at {:.0} fpm — {how}", sink / FPM));
                return;
            }
            if bank.abs() > def.max_touchdown_bank {
                let what = if water_only { "Caught a float" } else { "Wingtip strike" };
                self.crash(format!("{what}: touched down with {:.0}° of bank", bank.abs().to_degrees()));
                return;
            }
            if water_only && pitch < def.min_touchdown_pitch {
                self.crash(format!("Dug the float bows in: touched down nose-low ({:.0}° nose down)", -pitch.to_degrees()));
                return;
            }
            if pitch < def.min_touchdown_pitch {
                self.crash(format!(
                    "Nose-gear collapse: touched down nose-first ({:.0}° nose down)",
                    -pitch.to_degrees()
                ));
                return;
            }
        }
        self.on_ground = gnd.contact;

        // ---- load factor ----------------------------------------------------
        // (Fuel burnt / airframe ice change the mass: systems/.)
        let mass = self.dmg.mass(def) + fx.mass_delta;
        let weight = mass * GRAVITY;
        // Forces now act about the shifted centre of gravity.
        let f_world = self.rot * f_body + f_ground;
        let up = self.up();
        self.g_load = f_world.dot(up) / weight;
        self.side_g = -f_world.dot(self.right()) / weight;
        let g_aero = f_body.y / weight;
        self.g_filtered += (g_aero - self.g_filtered) * (1.0 - (-dt / 0.1f32).exp());

        // ---- integrate translation (symplectic Euler) -----------------------
        let acc = f_world / mass - Vec3::Y * GRAVITY;
        self.vel += acc * dt;
        self.pos += self.vel * dt;

        // ---- integrate rotation (damping implicit per axis) ------------------
        let inertia = self.dmg.inertia(def);
        let w = self.omega;
        // Rigid body + spinning propellers (gyroscopic precession).
        let gyro = w.cross(inertia * w + Vec3::Z * h_prop);
        // Negative (autorotative) damping is explicit in effect; bound it so the
        // implicit denominator stays well away from zero.
        let damping = aero.damping.max(-0.5 * inertia / dt);
        let mut w_new = (w + dt * (m_body - gyro) / inertia) / (Vec3::ONE + dt * damping / inertia);
        let wl = w_new.length();
        const MAX_RATE: f32 = 15.0;
        if wl > MAX_RATE {
            w_new *= MAX_RATE / wl;
        }
        self.omega = w_new;
        self.rot = (self.rot * Quat::from_scaled_axis(self.omega * dt)).normalize();

        // ---- structural loads, wing bending, damage (damage.rs) ---------------
        let loads = crate::damage::Loads {
            lift: load_lift,
            roll_moment: load_roll,
            n_total: self.g_load,
            side_g: self.side_g,
            ias: self.ias,
            mach: self.mach,
            flaps: self.surf.flaps,
            spoilers: self.surf.spoilers,
            gear_pos: self.gear_pos,
            gear_moving: (self.gear_pos - gear_before).abs() > 1e-6,
            on_ground: self.on_ground,
            ground_speed: Vec2::new(self.vel.x, self.vel.z).length(),
            q_s: 0.5 * atm.density * vb.length_squared() * def.wing_area,
            cl_max: def.cl_max(self.surf.flaps),
            dt,
        };
        self.dmg.update(sd, def, &loads);
        // A ground loop / hard sideways skid folds the main gear.
        if self.on_ground && !water_only && self.dmg.side_f.abs() > sd.gear_side_g && loads.ground_speed > 8.0 && self.dmg.lost & crate::damage::GEAR_BITS == 0 && def.category != Category::Glider {
            let realistic = self.assists.crashes == crate::damage::Realism::Realistic;
            for p in [crate::damage::Part::GearL, crate::damage::Part::GearR].into_iter().filter(|_| realistic) {
                if def.gear.iter().any(|g| crate::damage::gear_part(def, g) == Some(p)) {
                    self.dmg.separate(sd, def, p, "side load");
                }
            }
            self.crash(format!("Ground loop: main gear collapsed under a {:.1} g side load", self.dmg.side_f.abs()));
            return;
        }

        // ---- obstacles --------------------------------------------------------
        if let Some(reason) = self.obstacle_check(def, env) {
            self.crash(reason);
            return;
        }
        let ground_now = finite_or(env.ground_height(Vec2::new(self.pos.x, self.pos.z)), 0.0);
        if self.pos.y < ground_now + 0.15 {
            let r = if env.is_water(Vec2::new(self.pos.x, self.pos.z)) {
                "Ditched in the water".to_string()
            } else {
                "Flew into the ground".to_string()
            };
            self.crash(r);
            return;
        }

        // ---- numerical guard ----------------------------------------------------
        let finite = self.pos.is_finite()
            && self.vel.is_finite()
            && self.omega.is_finite()
            && self.rot.is_finite()
            && self.eng.iter().all(|e| e.rpm.is_finite() && e.spool.is_finite() && e.thrust.is_finite())
            && self.alpha.is_finite()
            && self.ias.is_finite();
        if !finite {
            *self = saved;
            self.crash("Simulation fault (state became non-finite)".to_string());
        }
    }

    fn update_engines(&mut self, def: &AircraftDef, c: &Controls, atm: &Atmosphere, vb: Vec3, dt: f32) {
        let v_fwd = vb.z;
        let oat = atm.temperature_c();
        let rev = self.surf.reverser;
        // Reverse selected: forward thrust held at idle until deployed, then the
        // lever sets reverse power.
        let mut ab_max = 0.0f32;
        let fx = self.sys.fx;
        for i in 0..def.engines() {
            // Each engine on its own lever when the throttles are split.
            let lever = c.lever(i);
            let thr = if def.has_reverser && c.reverse { if rev > 0.9 { lever } else { 0.0 } } else { lever };
            let (spool_t, ab_t) = def.spool_for_throttle(thr);
            let failed = c.failed_engines & (1 << i) != 0;
            let e = &mut self.eng[i];
            e.running = !failed;
            // Systems: partial power, starter cranking, governor overspeed (systems/).
            let pw = fx.power[i];
            let crank = fx.crank[i];
            match def.engine_kind {
                EngineKind::None | EngineKind::Turboshaft | EngineKind::Electric => {}
                EngineKind::Piston | EngineKind::Turboprop => {
                    let target = if failed { 0.0 } else { spool_t * pw };
                    e.spool += (target - e.spool) * (1.0 - (-dt / def.spool_time.max(0.05)).exp());
                    let mut rpm_t = def.target_rpm(e.spool, v_fwd, !failed) * (1.0 + fx.rpm_over[i]);
                    if failed && crank > 0.0 {
                        rpm_t = rpm_t.max(crank * def.rpm_idle);
                    }
                    e.rpm += (rpm_t - e.rpm) * (1.0 - (-dt / 0.3f32).exp());
                }
                EngineKind::Jet => {
                    let idle = def.n1_idle;
                    let n1_t = if failed {
                        (0.04 + 0.12 * self.mach.min(1.0)).min(idle).max(crank * idle)
                    } else if pw < 1.0 {
                        idle + (spool_t - idle).max(0.0) * pw
                    } else {
                        spool_t
                    };
                    let x = ((e.spool - idle) / (1.0 - idle)).clamp(0.0, 1.0);
                    // Acceleration is slow at low N1 (≈ spool_time from idle to max).
                    let r0 = 1.848 / def.spool_time.max(0.3) * (1.0 - idle);
                    let step = if n1_t > e.spool {
                        ((n1_t - e.spool) / 0.6).min(r0 * (0.25 + 0.75 * x))
                    } else {
                        ((n1_t - e.spool) / 0.8).max(-1.5 * r0)
                    };
                    e.spool += step * dt;
                    let ab_target = if !failed && e.spool > 0.97 * spool_t.max(idle) && spool_t > 0.99 { ab_t } else { 0.0 };
                    e.ab += (ab_target - e.ab) * (1.0 - (-dt / 0.35f32).exp());
                    if e.ab < 1e-3 && ab_target == 0.0 {
                        e.ab = 0.0;
                    }
                    e.rpm = e.spool * def.n1_rpm;
                }
            }
            let feathered = failed && def.engine_kind == EngineKind::Turboprop;
            let mut out = def.engine_output(e.spool, e.ab, rev, v_fwd, atm, feathered);
            if def.engine_kind == EngineKind::Turboprop {
                // Torque limit: shaft power cannot exceed max torque × Np.
                let np = e.rpm * def.prop_gear;
                let p_max = def.engine_power_w * e.rpm / def.rpm_static.max(1.0);
                if out.shaft_power > p_max && np > 1.0 {
                    let k = p_max / out.shaft_power;
                    out = def.engine_output(e.spool * k, e.ab, rev, v_fwd, atm, feathered);
                }
            }
            e.thrust = out.thrust;
            e.shaft_power = out.shaft_power;
            let lit = fx.lit & (1 << i) != 0;
            let egt_spool = if lit { e.spool.max(def.n1_idle.max(def.idle_power)) } else { e.spool };
            let egt_t = Self::egt_target(def, egt_spool, oat, !failed || lit) + fx.egt_add[i];
            e.egt += (egt_t - e.egt) * (1.0 - (-dt / 2.5f32).exp());
            e.fuel_flow = Self::fuel_flow_for(def, &out, self.mach, !failed);
            e.prop_angle = (e.prop_angle + e.rpm * def.prop_gear * (TAU / 60.0) * dt).rem_euclid(TAU);
            ab_max = ab_max.max(e.ab);
        }
        self.surf.afterburner = ab_max;
        for i in 0..MAX_ENGINES {
            self.surf.prop_angles[i] = self.eng[i].prop_angle;
        }
        self.surf.prop_angle = self.eng[0].prop_angle;
    }

    /// Pilot inputs (and the flight-control law) → surface deflections.
    #[allow(clippy::too_many_arguments)]
    fn actuate(&mut self, def: &AircraftDef, c: &Controls, vb: Vec3, agl: f32, atm: &Atmosphere, q_slip: f32, m_eng: Vec3, dt: f32) {
        let rate = def.surface_rate * dt;
        self.stick_elevator = move_toward(self.stick_elevator, c.elevator, rate);
        self.trim = move_toward(self.trim, c.trim, 0.5 * dt);
        self.surf.flaps = move_toward(self.surf.flaps, c.flaps, def.flap_rate * dt);
        if (self.surf.flaps - self.peak_flap).abs() > 1e-3 {
            self.peak_flap = self.surf.flaps;
            self.alpha_peak = def.alpha_usable(self.peak_flap);
        }
        let tas = vb.length();
        let direct_e = (self.stick_elevator + self.trim * def.trim_authority).clamp(-1.0, 1.0);

        if def.control_law == ControlLaw::Direct {
            self.surf.aileron = move_toward(self.surf.aileron, c.aileron, rate);
            let mut rud = c.rudder;
            if def.yaw_damper > 0.0 && !self.on_ground && tas > 30.0 {
                // Yaw damper: oppose yaw rate that is not part of a coordinated turn.
                let r = -self.omega.y;
                let r_turn = GRAVITY * self.roll().sin() * self.pitch().cos() / tas;
                rud = (rud - def.yaw_damper * (r - r_turn)).clamp(-1.0, 1.0);
            }
            self.surf.rudder = move_toward(self.surf.rudder, rud, rate);
            self.surf.elevator = direct_e;
            return;
        }

        // FBW: blend from direct (on the ground) to the flight law after lift-off.
        let airborne = !self.on_ground && tas > 25.0;
        self.law_blend = if airborne { move_toward(self.law_blend, 1.0, dt / 1.5) } else { move_toward(self.law_blend, 0.0, dt / 0.3) };
        if self.law_blend <= 0.0 {
            self.nz_int = 0.0;
            self.q_int = 0.0;
            self.surf.aileron = move_toward(self.surf.aileron, c.aileron, rate);
            self.surf.rudder = move_toward(self.surf.rudder, c.rudder, rate);
            self.surf.elevator = move_toward(self.surf.elevator, direct_e, rate);
            return;
        }
        let (e, a, r) = self.fbw(def, c, vb, agl, atm, q_slip, m_eng, dt);
        let b = self.law_blend;
        let e_cmd = b * e + (1.0 - b) * direct_e;
        let a_cmd = b * a + (1.0 - b) * c.aileron;
        let r_cmd = b * r + (1.0 - b) * c.rudder;
        self.surf.elevator = move_toward(self.surf.elevator, e_cmd.clamp(-1.0, 1.0), rate);
        self.surf.aileron = move_toward(self.surf.aileron, a_cmd.clamp(-1.0, 1.0), rate);
        self.surf.rudder = move_toward(self.surf.rudder, r_cmd.clamp(-1.0, 1.0), rate);
    }

    /// FBW flight laws as nonlinear dynamic inversion of this model: outer
    /// loops make body-rate demands, the inner loop solves for the surface
    /// deflection that gives the wanted angular acceleration.
    #[allow(clippy::too_many_arguments)]
    fn fbw(&mut self, def: &AircraftDef, c: &Controls, vb: Vec3, agl: f32, atm: &Atmosphere, q_slip: f32, m_eng: Vec3, dt: f32) -> (f32, f32, f32) {
        let v = vb.length().max(30.0);
        let p = self.omega.z;
        let q = -self.omega.x;
        let r = -self.omega.y;
        let phi = self.roll();
        let theta = self.pitch();
        let vel = self.vel;
        let gamma = if vel.length() > 1.0 { (vel.y / vel.length()).clamp(-1.0, 1.0).asin() } else { theta };
        let (cg, cphi) = (gamma.cos(), phi.cos());
        let jet = def.control_law == ControlLaw::FastJet;
        let gear_dn = self.gear_pos > 0.5;
        // (Concorde's lever droops the nose: no high-lift devices, so no relaxed limits.)
        let flaps_out = self.surf.flaps > 0.05 && def.flap_max_deg > 0.0;

        // ---- limits ----
        let (nz_max, nz_min, a_max) = if jet {
            if gear_dn { (4.0, -1.0, deg(20.0).min(def.alpha_max)) } else { (def.nz_max, def.nz_min, def.alpha_max) }
        } else if flaps_out {
            (2.0, 0.0, self.alpha_peak - deg(2.5))
        } else {
            (def.nz_max, def.nz_min, (self.alpha_peak - deg(2.5)).min(def.alpha_max.max(deg(5.0))))
        };
        let a_min = -deg(12.0);

        // ---- pitch: load-factor demand ----
        // Neutral stick holds the flight path (with bank compensation; the
        // airliner only up to 33° of bank). The fighter law blends towards a
        // 1 g demand in a dive, so a released stick pulls out of it.
        let comp = if cphi >= 0.5 { 1.0 / cphi } else { (cphi * 4.0).max(0.0) };
        let nz_neutral = if jet {
            comp * (cg + (1.0 - cg) * (-2.0 * gamma.sin()).clamp(0.0, 1.0))
        } else {
            // Pitch compensation fades out between 33° and 45° of bank.
            let w = ((phi.abs() - deg(33.0)) / deg(12.0)).clamp(0.0, 1.0);
            cg * (comp * (1.0 - w) + cphi.max(0.0) * w)
        };
        // Fighter: progressive stick (fine control near centre for keyboard
        // pilots, full 9 g at the stop) plus a small g bias from the trim.
        let se = self.stick_elevator;
        let shaped = if jet { se * (0.35 + 0.65 * se * se) } else { se };
        let stick = (shaped + if jet { 0.15 * self.trim } else { 0.0 }).clamp(-1.0, 1.0);
        let nz_cmd = if stick >= 0.0 {
            nz_neutral + stick * (nz_max - nz_neutral).max(0.0)
        } else {
            nz_neutral + (-stick) * (nz_min - nz_neutral).min(0.0)
        }
        .clamp(nz_min, nz_max);
        let nz = self.g_load;
        let k_i = if stick.abs() < 0.02 { 0.6 } else { 0.25 };
        self.nz_int = (self.nz_int + (nz_cmd - nz) * dt * k_i).clamp(-0.6, 0.6);
        let g_over_v = GRAVITY / v;
        let q_path = g_over_v * (nz - cg * cphi);
        let mut q_cmd = g_over_v * (nz_cmd - cg * cphi) + g_over_v * (1.5 * (nz_cmd - nz) + 1.5 * self.nz_int);
        if !jet {
            // High-speed protection: nose-up demand beyond Vmo / Mmo.
            let over = (self.ias / (def.vne_kt * KT) - 1.03).max(0.0) * 20.0
                + if def.mmo > 0.0 { (self.mach / def.mmo - 1.01).max(0.0) * 40.0 } else { 0.0 };
            q_cmd += g_over_v * over.min(1.0);
            // Pitch-attitude protection: +30° / -15°.
            q_cmd = q_cmd.min(0.5 * (deg(30.0) - theta)).max(0.5 * (deg(-15.0) - theta));
        }
        // Load-factor protection: the alpha that would give the limit g is
        // approached at the same rate as the AoA limit, so the short-period
        // lag can't carry nz past nz_max / nz_min (F-16 +9 / −3 g, airliner
        // normal law +2.5 / −1 g).
        let n_alpha = (def.cl_alpha * 0.5 * atm.density * vb.length_squared() * def.wing_area / (def.mass * GRAVITY)).max(1.0);
        // (The airliner law's slower inner loop settles a little past a
        // negative target: aim 10 % short so −1 g is not exceeded.)
        let q_hi = q_path + 2.5 * (if jet { 1.0 } else { 0.97 } * nz_max - nz) / n_alpha;
        let q_lo = q_path + 2.5 * (if jet { 0.97 } else { 0.9 } * nz_min - nz) / n_alpha;
        if q_cmd > q_hi || q_cmd < q_lo || nz > nz_max || nz < nz_min {
            // Protection active: unwind the integrators (their wind-up from
            // the entry held Concorde at −1.17 g for seconds).
            self.nz_int *= 0.95;
            self.q_int *= 0.95;
        }
        q_cmd = q_cmd.clamp(q_lo.min(q_hi), q_hi.max(q_lo));
        // AoA protection.
        q_cmd = q_cmd.min(q_path + 2.5 * (a_max - self.alpha));
        q_cmd = q_cmd.max(q_path + 2.5 * (a_min - self.alpha));

        // ---- roll: rate command, bank hold / protection ----
        let p_max = def.roll_rate_max.max(0.05);
        let mut p_cmd = c.aileron * p_max;
        if !jet {
            // Neutral stick holds the bank attitude (latched on release).
            if c.aileron.abs() < 0.05 {
                let hold = *self.bank_hold.get_or_insert(phi.clamp(deg(-33.0), deg(33.0)));
                p_cmd = 0.8 * (hold - phi);
            } else {
                self.bank_hold = None;
            }
            if c.aileron.abs() < 0.05 && phi.abs() > deg(33.0) {
                p_cmd = -0.5 * (phi.abs() - deg(33.0)) * phi.signum();
            }
            let lim = deg(67.0);
            p_cmd = p_cmd.min(1.0 * (lim - phi)).max(1.0 * (-lim - phi));
        }

        // ---- yaw: coordination + sideslip suppression + pedal ----
        let r_turn = GRAVITY * phi.sin() * theta.cos() / v;
        let r_cmd = r_turn + 1.5 * self.beta + c.rudder * if jet { 0.25 } else { 0.08 };

        // ---- inner loop: dynamic inversion ----
        let aero = self.aero(def, vb, agl, atm, q_slip);
        let iv = Vec3::new(def.inertia_pitch, def.inertia_yaw, def.inertia_roll);
        // Moments (pitch nose-up, yaw nose-right, roll right) without the controls.
        let m_pitch_tot = -(aero.moment.x + m_eng.x) - aero.damping.x * q;
        let n_yaw_tot = -(aero.moment.y + m_eng.y) - aero.damping.y * r;
        let l_roll_tot = (aero.moment.z + m_eng.z) - aero.damping.z * p;
        let m_other = m_pitch_tot - aero.elev_power * self.surf.elevator;
        let n_other = n_yaw_tot - aero.rudder_power * self.surf.rudder;
        let l_other = l_roll_tot - aero.ail_power * self.surf.aileron;

        self.q_int = (self.q_int + (q_cmd - q) * dt).clamp(-0.2, 0.2);
        let qdot = 4.0 * (q_cmd - q) + 3.0 * self.q_int;
        let pdot = 5.0 * (p_cmd - p);
        let rdot = 2.5 * (r_cmd - r);
        let solve = |want: f32, other: f32, power: f32| if power.abs() > 1.0 { (want - other) / power } else { 0.0 };
        let e = solve(iv.x * qdot, m_other, aero.elev_power);
        let a = solve(iv.z * pdot, l_other, aero.ail_power);
        let rd = solve(iv.y * rdot, n_other, aero.rudder_power);
        // Avoid integrator wind-up while saturated.
        if e.abs() > 1.0 {
            self.q_int *= 0.98;
        }
        (e, a, rd)
    }

    fn aero(&self, def: &AircraftDef, vb: Vec3, agl: f32, atm: &Atmosphere, q_slip: f32) -> AeroOut {
        if let Some(j) = &def.jsb {
            return self.aero_jsb(def, j, vb, agl, atm, q_slip);
        }
        let rho = atm.density;
        let v2 = vb.length_squared();
        let v = v2.sqrt();
        let qd = 0.5 * rho * v2;
        let vs2 = vb.y * vb.y + vb.z * vb.z;
        let alpha = if vs2 > 1e-4 { (-vb.y).atan2(vb.z) } else { 0.0 };
        let beta = if v > 1e-2 { (-vb.x / v).clamp(-1.0, 1.0).asin() } else { 0.0 };
        let (sb, b, c, s) = (beta.sin(), def.wingspan, def.chord, def.wing_area);
        let flap = self.surf.flaps;
        let mach = v / atm.speed_of_sound;

        // Ground effect on induced drag (McCormick).
        let h = (agl + def.wing_height).max(0.05);
        let x = 16.0 * h / b;
        let ge = (x * x / (1.0 + x * x)).clamp(0.05, 1.0);

        let ld = def.lift_drag(alpha, flap, ge);
        let sp = self.surf.spoilers;
        let cl = if ld.cl > 0.0 { ld.cl * (1.0 - def.spoiler_cl * sp) } else { ld.cl };
        let gear_cd = if def.retractable_gear { def.cd_gear * self.gear_pos } else { 0.0 };
        let qs = 0.5 * rho * vs2;
        let lift_dir = if vs2 > 1e-4 { Vec3::new(0.0, vb.z, -vb.y) / vs2.sqrt() } else { Vec3::Y };
        let drag_dir = if v > 1e-3 { -vb / v } else { Vec3::ZERO };
        let cd = ld.cd + def.cd_beta * sb * sb + gear_cd + def.spoiler_cd * sp + def.wave_drag(mach);
        let force = lift_dir * (qs * s * cl) + drag_dir * (qd * s * cd) + Vec3::NEG_X * (qd * s * def.cy_beta * sb);

        // Rates.
        let p = self.omega.z;
        let r = -self.omega.y;
        let q_rate = -self.omega.x;
        let q_tail = qd + q_slip;
        let rate_k_b = 0.25 * rho * v * s * b * b; // × coeff × rate → moment
        let rate_k_c = 0.25 * rho * v * s * c * c;

        let ail_eff = 1.0 - 0.6 * ld.stall;
        let ail = self.surf.aileron * ail_eff;
        let fl = def.flap_lift_frac(flap);

        // Pitch (nose up +).
        let cm = def.cm0 + def.cm_alpha * alpha.sin() + def.flap_cm * fl + def.cm_stall * ld.stall_pos;
        let elev_power = q_tail * s * c * def.cm_elevator;
        let m_pitch = qd * s * c * cm + elev_power * self.surf.elevator;

        // Roll (right wing down +).
        let cl_r = def.cl_r_per_cl * ld.cl.clamp(-1.0, 1.6);
        let ail_power = qd * s * b * def.cl_aileron * ail_eff;
        let l_roll = qd * s * b * (def.cl_beta * sb + def.cl_rig) + ail_power * self.surf.aileron + rate_k_b * cl_r * r;

        // Yaw (nose right +).
        let rudder_power = q_tail * s * b * def.cn_rudder;
        let n_yaw = qd * s * b * (def.cn_beta * sb + def.cn_aileron * ail + def.cn_rig)
            + rudder_power * self.surf.rudder
            + rate_k_b * def.cn_p * p;

        // Roll damping collapses and reverses once the wing is stalled (the
        // down-going wing sees more alpha and *loses* lift): wing drop and
        // spin autorotation. Deep in the post-stall regime (|alpha| > ~45°)
        // the wing is a flat plate again and damps.
        let deep = ((alpha.abs() - 0.6) / 0.3).clamp(0.0, 1.0);
        let slope_ratio = (1.0 - (1.0 + def.stall_autorotation) * ld.stall) * (1.0 - deep) + 0.5 * deep;
        let yaw_damp_ratio = 1.0 - 0.6 * ld.stall * (1.0 - deep);
        let base = 0.03;
        let tumble = |l: f32| 0.15 * rho * s * l * l * l;
        let d_roll = -def.cl_p * slope_ratio * rate_k_b + tumble(0.5 * b) * p.abs() + base * def.inertia_roll;
        let d_pitch = -def.cm_q * rate_k_c + tumble(0.4 * b) * q_rate.abs() + base * def.inertia_pitch;
        let d_yaw = -def.cn_r * yaw_damp_ratio * rate_k_b + tumble(0.45 * b) * r.abs() + base * def.inertia_yaw;

        AeroOut {
            force,
            moment: Vec3::new(-m_pitch, -n_yaw, l_roll),
            damping: Vec3::new(d_pitch, d_yaw, d_roll),
            alpha,
            beta,
            elev_power,
            ail_power,
            rudder_power,
            jsb: Default::default(),
        }
    }

    /// Aerodynamics from the aircraft's JSBSim FDM. Rate damping is
    /// linearised by finite differences and returned separately, so the
    /// integrator keeps treating it implicitly; control powers (for the FBW
    /// inversion) likewise.
    fn aero_jsb(&self, def: &AircraftDef, j: &JsbDef, vb: Vec3, agl: f32, atm: &Atmosphere, q_slip: f32) -> AeroOut {
        let rho = atm.density;
        let v = vb.length();
        let mach = v / atm.speed_of_sound;
        let wash = if q_slip > 0.0 && rho > 0.0 { ((v * v + 2.0 * q_slip / rho).sqrt() - v).max(0.0) } else { 0.0 };
        let s = JsbSurf {
            elevator: self.surf.elevator,
            aileron: self.surf.aileron,
            rudder: self.surf.rudder,
            flaps: self.surf.flaps,
            spoilers: self.surf.spoilers,
            gear: if def.retractable_gear { self.gear_pos } else { 1.0 },
        };
        let h_cg = agl.max(0.0);
        let h_wing = (agl + def.wing_height).max(0.0);
        let ev = |w: Vec3, s: &JsbSurf| {
            let mut i = j.input(vb, w, rho, mach, s, h_cg, h_wing, wash, atm.pressure);
            i.cg_aft_m = self.cg_aft;
            j.aero.eval(&i, &self.jsb)
        };
        let (o, st) = ev(self.omega, &s);
        // Rate damping per axis: -dM_i/dω_i.
        let h = 0.05;
        let mut damping = Vec3::ZERO;
        for k in 0..3 {
            let mut w = self.omega;
            w[k] += h;
            let m = ev(w, &s).0.moment;
            damping[k] = -(m[k] - o.moment[k]) / h;
        }
        // Keep a little structural / unmodelled damping, like the parametric model.
        let base = 0.03 * Vec3::new(def.inertia_pitch, def.inertia_yaw, def.inertia_roll);
        let damping = damping + base;
        let moment = o.moment + (damping - base) * self.omega;
        let (mut elev_power, mut ail_power, mut rudder_power) = (0.0, 0.0, 0.0);
        if def.control_law != ControlLaw::Direct {
            let d = 0.1;
            let p = |f: &dyn Fn(&mut JsbSurf)| {
                let mut a = s;
                f(&mut a);
                ev(self.omega, &a).0.moment
            };
            let e0 = (s.elevator - d).max(-1.0);
            let e1 = (s.elevator + d).min(1.0);
            let (m0, m1) = (p(&|x| x.elevator = e0), p(&|x| x.elevator = e1));
            elev_power = -(m1.x - m0.x) / (e1 - e0);
            let a0 = (s.aileron - d).max(-1.0);
            let a1 = (s.aileron + d).min(1.0);
            let (m0, m1) = (p(&|x| x.aileron = a0), p(&|x| x.aileron = a1));
            ail_power = (m1.z - m0.z) / (a1 - a0);
            let r0 = (s.rudder - d).max(-1.0);
            let r1 = (s.rudder + d).min(1.0);
            let (m0, m1) = (p(&|x| x.rudder = r0), p(&|x| x.rudder = r1));
            rudder_power = -(m1.y - m0.y) / (r1 - r0);
        }
        AeroOut { force: o.force, moment, damping, alpha: o.alpha, beta: o.beta, elev_power, ail_power, rudder_power, jsb: st }
    }

    fn gear_locked(&self, def: &AircraftDef) -> bool {
        !def.retractable_gear || self.gear_pos >= 0.98
    }

    fn ground(&self, def: &AircraftDef, env: &dyn Environment, c: &Controls, dt: f32) -> GroundOut {
        let mut out = GroundOut::default();
        let inv = self.rot.conjugate();
        let ground_speed = Vec2::new(self.vel.x, self.vel.z).length();
        let steer_max = def.nose_steer_deg.to_radians() / (1.0 + (ground_speed / 10.0).powi(2));
        let steer = self.surf.rudder * steer_max;
        let locked = self.gear_locked(def);

        let mut torque_world = Vec3::ZERO;
        let gear_gone = self.dmg.lost & crate::damage::GEAR_BITS != 0;
        for gp in def.gear {
            if (gp.retracts && !locked) || self.dmg.gear_lost(def, gp) {
                continue;
            }
            let r = self.rot * gp.pos;
            let cp = self.pos + r;
            let cxz = Vec2::new(cp.x, cp.z);
            let h = finite_or(env.ground_height(cxz), 0.0);
            let pen = h - cp.y;
            if pen <= 0.0 {
                continue;
            }
            if env.is_water(cxz) {
                if def.on_water() {
                    // Floats: the hull model below carries it on the water.
                    continue;
                }
                out.crash = Some("Ditched in the water".to_string());
                return out;
            }
            out.contact = true;
            out.land = true;
            if let Some(i) = def.gear.iter().position(|g| std::ptr::eq(g, gp)) {
                if i < 8 {
                    out.comp[i] = pen;
                }
            }
            // Differential braking with the rudder (as on the warbirds' brake
            // lever + rudder bar, or toe brakes): the inside wheel brakes harder.
            let brake = if gp.pos.x > 0.1 {
                c.brakes * (1.0 - c.rudder).clamp(0.0, 1.0)
            } else if gp.pos.x < -0.1 {
                c.brakes * (1.0 + c.rudder).clamp(0.0, 1.0)
            } else {
                c.brakes
            };
            // A tail wheel steers the opposite way to a nose wheel (nose-wheel
            // steering needs hydraulics: systems/).
            let s = if gp.pos.z < 0.0 { -steer } else { steer * self.sys.fx.steering };
            // A burst tyre runs on its rim: high drag, little grip, no braking.
            let burst = self.sys.fx.tyre_burst(gp.pos.x);
            let (kind, brake) = if burst { (ContactKind::Skid { friction: 0.35 }, 0.0) } else { (gp.kind, brake) };
            let f = contact_force(self, def, gp.pos, pen, gp.spring, gp.damping, kind, s, brake, dt);
            out.force_world += f;
            torque_world += r.cross(f);
        }
        for hp in def.hard_points {
            let r = self.rot * hp.pos;
            let cp = self.pos + r;
            let cxz = Vec2::new(cp.x, cp.z);
            let h = finite_or(env.ground_height(cxz), 0.0);
            let pen = h - cp.y;
            if pen <= 0.0 {
                continue;
            }
            if env.is_water(cxz) {
                out.crash = Some(if def.on_water() {
                    match hp.reason {
                        "Wingtip strike" => "A wingtip caught the water — cartwheeled".to_string(),
                        "Prop strike" => "The propeller hit the water".to_string(),
                        "Crashed inverted" => "Capsized".to_string(),
                        _ => "Flipped over on the water".to_string(),
                    }
                } else {
                    "Ditched in the water".to_string()
                });
                return out;
            }
            out.land = true;
            if ground_speed >= hp.tolerate_below {
                let belly_hit = ((def.retractable_gear && !locked) || gear_gone) && hp.pos.y < -0.5 && hp.reason != "Wingtip strike";
                let reason = if belly_hit {
                    GEAR_UP
                } else if hp.reason == GEAR_UP {
                    "Belly struck the ground"
                } else {
                    hp.reason
                };
                out.crash = Some(reason.to_string());
                return out;
            }
            // Tolerated scrape.
            let kind = ContactKind::Skid { friction: 0.6 };
            let f = contact_force(self, def, hp.pos, pen, 30_000.0, 4_000.0, kind, 0.0, 0.0, dt);
            out.force_world += f;
            torque_world += r.cross(f);
        }
        out.moment_body = inv * torque_world;
        // Floats on the water (src/water.rs).
        if def.on_water() {
            let body = crate::water::BodyState { pos: self.pos, vel: self.vel, rot: self.rot, omega: self.omega };
            let wc = crate::water::water_contact(def.hulls, &body, env, &self.water.sea, self.water.time, self.surf.rudder, def.mass * GRAVITY);
            if wc.wet {
                out.contact = true;
                out.water = true;
                out.force_world += wc.force;
                out.moment_body += wc.moment;
                if wc.bow_buried {
                    out.crash = Some("Dug the float bows in — flipped over".to_string());
                }
            }
            out.water_contact = wc;
        }
        out
    }

    fn obstacle_check(&self, def: &AircraftDef, env: &dyn Environment) -> Option<String> {
        let hit = |p: Vec3| -> bool {
            match env.obstacle_top(Vec2::new(p.x, p.z)) {
                Some(top) if top.is_finite() => p.y < top,
                _ => false,
            }
        };
        if hit(self.pos) {
            return Some("Flew into a building".to_string());
        }
        for hp in def.hard_points {
            if hit(self.pos + self.rot * hp.pos) {
                return Some(if hp.reason == "Wingtip strike" {
                    "Wingtip struck a building".to_string()
                } else {
                    "Flew into a building".to_string()
                });
            }
        }
        let locked = self.gear_locked(def);
        for gp in def.gear {
            if (gp.retracts && !locked) || self.dmg.gear_lost(def, gp) {
                continue;
            }
            if hit(self.pos + self.rot * gp.pos) {
                return Some("Undercarriage struck a building".to_string());
            }
        }
        None
    }
}

#[allow(clippy::too_many_arguments)]
fn contact_force(
    a: &Aircraft,
    def: &AircraftDef,
    pos_b: Vec3,
    pen: f32,
    spring: f32,
    damping: f32,
    kind: ContactKind,
    steer: f32,
    brakes: f32,
    dt: f32,
) -> Vec3 {
    let v_cp = a.vel + a.rot * a.omega.cross(pos_b);
    // Progressive spring (bottoming out) + damper (oleo: heavier on
    // rebound, so a firm arrival does not bounce back); never pulls.
    let c = if v_cp.y > 0.0 { 2.5 * damping } else { damping };
    let fz = (spring * pen * (1.0 + (pen / 0.15).powi(2)) - c * v_cp.y).clamp(0.0, 40.0 * def.mass * GRAVITY);
    let vt = Vec3::new(v_cp.x, 0.0, v_cp.z);

    let (fwd_b, mu_long, mu_lat) = match kind {
        ContactKind::Wheel { steerable, braked } => {
            let fwd = if steerable { Vec3::new(-steer.sin(), 0.0, steer.cos()) } else { Vec3::Z };
            let mu = def.roll_mu + if braked { def.brake_mu * brakes } else { 0.0 };
            (fwd, mu, def.tire_mu)
        }
        ContactKind::Skid { friction } => (Vec3::Z, friction, friction),
    };
    let mut fwd = a.rot * fwd_b;
    fwd.y = 0.0;
    let fwd = fwd.try_normalize().unwrap_or(Vec3::Z);
    let side = Vec3::Y.cross(fwd);
    let v_long = vt.dot(fwd);
    let v_lat = vt.dot(side);
    // Velocity-level (implicit) friction: the force that would cancel this
    // contact's share of the slip within a few steps, capped by Coulomb
    // friction. Gives stiction without stiff explicit gains.
    let share = fz / GRAVITY; // supported mass, kg
    let k_long = 0.6 * share / dt;
    let k_lat = 0.35 * share / dt;
    // A rolling tyre's side force builds with slip angle (cornering
    // stiffness) and saturates at ~8°; skids and slow tyres grip fully.
    let lat_cap = match kind {
        ContactKind::Wheel { .. } => {
            let slip = v_lat.abs().atan2(v_long.abs() + 1.0);
            mu_lat * fz * (slip / 0.14).clamp(0.05, 1.0)
        }
        ContactKind::Skid { .. } => mu_lat * fz,
    };
    let f_long = -(v_long * k_long).clamp(-mu_long * fz, mu_long * fz);
    let f_lat = -(v_lat * k_lat).clamp(-lat_cap, lat_cap);
    Vec3::Y * fz + fwd * f_long + side * f_lat
}

struct AeroOut {
    force: Vec3,
    moment: Vec3,
    damping: Vec3,
    alpha: f32,
    beta: f32,
    /// Moment per unit surface deflection (pitch up, roll right, yaw right).
    elev_power: f32,
    ail_power: f32,
    rudder_power: f32,
    /// JSBSim evaluator state after this evaluation.
    jsb: crate::jsbsim::JsbState,
}

/// Surface / configuration state for one JSBSim evaluation.
#[derive(Clone, Copy, Debug, Default)]
pub struct JsbSurf {
    pub elevator: f32,
    pub aileron: f32,
    pub rudder: f32,
    pub flaps: f32,
    pub spoilers: f32,
    pub gear: f32,
}

impl JsbDef {
    /// Build from FDM XML with property aliases and constants.
    pub fn new(xml: &str, aliases: &[(&str, &str)], consts: &[(&str, f64)]) -> JsbDef {
        let aero = crate::jsbsim::JsbAero::parse(xml, aliases, consts).expect("JSBSim FDM");
        JsbDef {
            aero,
            elev_up: deg(25.0),
            elev_down: deg(20.0),
            ail_up: deg(20.0),
            ail_down: deg(15.0),
            rudder_max: deg(25.0),
            flap_max_deg: 30.0,
            slats: &[],
            slat_max_deg: 0.0,
            spoiler_max_deg: 45.0,
            lef: false,
            flaps_are_airbrakes: false,
            sign_e: 1.0,
            sign_a: 1.0,
            sign_r: 1.0,
            clmax: Vec::new(),
        }
    }

    /// Evaluator inputs for a state.
    #[allow(clippy::too_many_arguments)]
    pub fn input(&self, vb: Vec3, omega: Vec3, rho: f32, mach: f32, s: &JsbSurf, h_cg: f32, h_wing: f32, prop_wash: f32, pressure: f32) -> crate::jsbsim::AeroInput {
        let e = s.elevator * self.sign_e;
        let a = s.aileron * self.sign_a;
        let r = s.rudder * self.sign_r;
        // JSBSim: elevator positive = trailing edge down (nose down); ailerons
        // positive = trailing edge down; Skyward +e = stick back (TE up).
        let elev = if e >= 0.0 { -e * self.elev_up } else { -e * self.elev_down };
        let ail = |x: f32| if x >= 0.0 { x * self.ail_down } else { x * self.ail_up };
        let slat = if self.slats.is_empty() { 0.0 } else { interp_table(self.slats, s.flaps) };
        let v = vb.length();
        let vs2 = vb.y * vb.y + vb.z * vb.z;
        let alpha = if vs2 > 1e-4 { (-vb.y).atan2(vb.z) } else { 0.0 };
        let lef = if self.lef && v > 1.0 {
            let qbar = 0.5 * rho * v * v;
            ((1.38 * alpha.to_degrees() - 9.05 * qbar / pressure.max(1.0) + 1.45).clamp(0.0, 25.0)) / 25.0
        } else {
            0.0
        };
        crate::jsbsim::AeroInput {
            vb,
            p: omega.z,
            q: -omega.x,
            r: -omega.y,
            alpha_dot: 0.0,
            rho,
            mach,
            elevator: elev,
            aileron_l: ail(a),
            aileron_r: ail(-a),
            rudder: r * self.rudder_max,
            elevator_norm: -e,
            aileron_norm: a,
            rudder_norm: r,
            flap_deg: if self.flaps_are_airbrakes { 0.0 } else { s.flaps * self.flap_max_deg },
            flap_norm: if self.flaps_are_airbrakes { 0.0 } else { s.flaps },
            slat_norm: slat,
            slat_deg: slat * self.slat_max_deg,
            speedbrake: if self.flaps_are_airbrakes { s.flaps } else { s.spoilers },
            spoiler: s.spoilers,
            spoiler_deg: s.spoilers * self.spoiler_max_deg,
            gear: s.gear,
            h_mac: h_wing,
            h_cg,
            prop_wash,
            stab_deg: 0.0,
            stab_norm: 0.0,
            lef_norm: lef,
            cg_aft_m: 0.0,
        }
    }

    /// Fix the control sign conventions (positive elevator pitches up etc.).
    fn calibrate_signs(&mut self) {
        let v = 60.0;
        let al = deg(2.0);
        let vb = Vec3::new(0.0, -v * al.sin(), v * al.cos());
        let m = |d: &JsbDef, s: JsbSurf| {
            let i = d.input(vb, Vec3::ZERO, 1.225, v / 340.0, &s, 100.0, 100.0, 0.0, 101_325.0);
            d.aero.eval_steady(&i).moment
        };
        let base = JsbSurf { gear: 0.0, ..Default::default() };
        // Pitch up = negative moment about +X; roll right = +Z; yaw right = -Y.
        let pe = m(self, JsbSurf { elevator: 0.5, ..base }).x - m(self, JsbSurf { elevator: -0.5, ..base }).x;
        if pe > 0.0 {
            self.sign_e = -1.0;
        }
        let pa = m(self, JsbSurf { aileron: 0.5, ..base }).z - m(self, JsbSurf { aileron: -0.5, ..base }).z;
        if pa < 0.0 {
            self.sign_a = -1.0;
        }
        let pr = m(self, JsbSurf { rudder: 0.5, ..base }).y - m(self, JsbSurf { rudder: -0.5, ..base }).y;
        if pr > 0.0 {
            self.sign_r = -1.0;
        }
    }
}

impl JsbDef {
    /// Fit the parametric model fields of `def` (lift curve, polar, stability
    /// and control derivatives) to this JSBSim model, so trim / stall-speed
    /// estimates and the FBW feed-forward agree with the flown aerodynamics.
    pub fn fit(&mut self, def: &mut AircraftDef, v: f32) {
        // Lift peaks per flap setting (gear down, as in the circuit).
        self.clmax = (0..=4)
            .map(|k| {
                let lever = k as f32 / 4.0;
                let mut best = (0.0f32, -9.0f32);
                let mut a = 0.0f32;
                while a <= 35.0 {
                    let al = deg(a);
                    let vb = Vec3::new(0.0, -60.0 * al.sin(), 60.0 * al.cos());
                    let s = JsbSurf { flaps: lever, gear: 1.0, ..Default::default() };
                    let c = self.aero.eval_steady(&self.input(vb, Vec3::ZERO, 1.225, 0.18, &s, 300.0, 300.0, 0.0, 101_325.0)).cl;
                    if c > best.1 {
                        best = (al, c);
                    } else if c < best.1 - 0.25 && al > best.0 + deg(4.0) {
                        break;
                    }
                    a += 0.5;
                }
                (lever, best.0, best.1)
            })
            .collect();
        let rho = 1.225;
        let gear0 = if def.retractable_gear { 0.0 } else { 1.0 };
        let (s_ref, b, c) = (self.aero.sw_m2 as f32, self.aero.bw_m as f32, self.aero.cbar_m as f32);
        let qd = 0.5 * rho * v * v;
        let ev = |al: f32, be: f32, w: Vec3, s: JsbSurf| {
            let vb = Vec3::new(-v * be.sin(), -v * al.sin() * be.cos(), v * al.cos() * be.cos());
            let i = self.input(vb, w, rho, v / 340.3, &s, 200.0, 200.0, 0.0, 101_325.0);
            self.aero.eval_steady(&i)
        };
        let base = JsbSurf { gear: gear0, ..Default::default() };
        let cm = |o: &crate::jsbsim::AeroOutput| -o.moment.x / (qd * s_ref * c);
        let cl_roll = |o: &crate::jsbsim::AeroOutput| o.moment.z / (qd * s_ref * b);
        let cn = |o: &crate::jsbsim::AeroOutput| -o.moment.y / (qd * s_ref * b);
        // Lift curve and stall.
        let cl_at = |a: f32, s: JsbSurf| ev(deg(a), 0.0, Vec3::ZERO, s).cl;
        // First lift peak (the attached-flow branch; stop once CL falls clearly).
        let a_max = |s: JsbSurf| {
            let mut best = (0.0, -9.0);
            let mut a = 0.0;
            while a <= 30.0 {
                let c = cl_at(a, s);
                if c > best.1 {
                    best = (a, c);
                } else if c < best.1 - 0.25 && a > best.0 + 4.0 {
                    break;
                }
                a += 0.5;
            }
            best
        };
        def.cl0 = cl_at(0.0, base);
        def.cl_alpha = (cl_at(6.0, base) - cl_at(-2.0, base)) / deg(8.0);
        let (amax, clmax) = a_max(base);
        def.alpha_stall = deg(amax + 0.5);
        let past = cl_at(amax + 5.0, base);
        let lin = def.cl0 + def.cl_alpha * deg(amax + 5.0);
        def.stall_drop = (past / lin.max(0.1)).clamp(0.4, 0.98);
        def.alpha_stall_neg = def.alpha_stall.min(deg(15.0));
        // Polar CD = cd0 + k CL² over the attached range (least squares).
        let (mut sx, mut sy, mut sxx, mut sxy, mut n) = (0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let mut a = -2.0;
        while a <= 8.0 {
            let o = ev(deg(a), 0.0, Vec3::ZERO, base);
            let x = o.cl * o.cl;
            sx += x;
            sy += o.cd;
            sxx += x * x;
            sxy += x * o.cd;
            n += 1.0;
            a += 1.0;
        }
        let k = ((n * sxy - sx * sy) / (n * sxx - sx * sx)).max(1e-4);
        def.cd0 = ((sy - k * sx) / n).max(0.004);
        def.oswald = (1.0 / (PI * def.aspect_ratio() * k)).clamp(0.4, 1.1);
        def.cd_profile_k = 0.0;
        // Flaps (lever 1).
        if def.flap_max_deg > 0.0 && self.flap_max_deg > 0.0 {
            let full = JsbSurf { flaps: 1.0, ..base };
            let o0 = ev(deg(2.0), 0.0, Vec3::ZERO, base);
            let o1 = ev(deg(2.0), 0.0, Vec3::ZERO, full);
            def.flap_cl = o1.cl - o0.cl;
            def.flap_cd = (o1.cd - o0.cd).max(0.0);
            def.flap_cm = cm(&o1) - cm(&o0);
            def.flap_alpha_stall = deg(a_max(full).0 + 0.5) - def.alpha_stall;
        }
        // Pitch.
        let o0 = ev(0.0, 0.0, Vec3::ZERO, base);
        let o6 = ev(deg(6.0), 0.0, Vec3::ZERO, base);
        def.cm0 = cm(&o0);
        def.cm_alpha = (cm(&o6) - cm(&o0)) / deg(6.0).sin();
        let a2 = deg(2.0);
        let ep = ev(a2, 0.0, Vec3::ZERO, JsbSurf { elevator: 0.5, ..base });
        let em = ev(a2, 0.0, Vec3::ZERO, JsbSurf { elevator: -0.5, ..base });
        def.cm_elevator = cm(&ep) - cm(&em);
        let q = 0.1;
        let oq = ev(a2, 0.0, Vec3::new(-q, 0.0, 0.0), base);
        let oa = ev(a2, 0.0, Vec3::ZERO, base);
        def.cm_q = (cm(&oq) - cm(&oa)) / (q * c / (2.0 * v));
        // Lateral / directional.
        let bt = deg(5.0);
        let ob = ev(a2, bt, Vec3::ZERO, base);
        def.cy_beta = (-ob.force.x / (qd * s_ref)) / bt.sin();
        def.cl_beta = (cl_roll(&ob) - cl_roll(&oa)) / bt.sin();
        def.cn_beta = (cn(&ob) - cn(&oa)) / bt.sin();
        let p = 0.2;
        let op = ev(a2, 0.0, Vec3::new(0.0, 0.0, p), base);
        def.cl_p = (cl_roll(&op) - cl_roll(&oa)) / (p * b / (2.0 * v));
        def.cn_p = (cn(&op) - cn(&oa)) / (p * b / (2.0 * v));
        let r = 0.1;
        let or = ev(a2, 0.0, Vec3::new(0.0, -r, 0.0), base);
        def.cn_r = (cn(&or) - cn(&oa)) / (r * b / (2.0 * v));
        def.cl_r_per_cl = ((cl_roll(&or) - cl_roll(&oa)) / (r * b / (2.0 * v))) / oa.cl.max(0.1);
        let ap = ev(a2, 0.0, Vec3::ZERO, JsbSurf { aileron: 0.5, ..base });
        let am = ev(a2, 0.0, Vec3::ZERO, JsbSurf { aileron: -0.5, ..base });
        def.cl_aileron = cl_roll(&ap) - cl_roll(&am);
        def.cn_aileron = cn(&ap) - cn(&am);
        let rp = ev(a2, 0.0, Vec3::ZERO, JsbSurf { rudder: 0.5, ..base });
        let rm = ev(a2, 0.0, Vec3::ZERO, JsbSurf { rudder: -0.5, ..base });
        def.cn_rudder = cn(&rp) - cn(&rm);
        // Spoilers / gear drag.
        if def.has_spoilers {
            let a4 = deg(4.0);
            let s0 = ev(a4, 0.0, Vec3::ZERO, base);
            let s1 = ev(a4, 0.0, Vec3::ZERO, JsbSurf { spoilers: 1.0, ..base });
            def.spoiler_cl = (1.0 - s1.cl / s0.cl.max(0.05)).clamp(0.0, 1.0);
            def.spoiler_cd = (s1.cd - s0.cd).max(0.0);
        }
        if def.retractable_gear {
            let g1 = ev(a2, 0.0, Vec3::ZERO, JsbSurf { gear: 1.0, ..base });
            def.cd_gear = (g1.cd - oa.cd).max(0.0);
        }
        let _ = clmax;
    }
}

fn interp_table(t: &[(f32, f32)], x: f32) -> f32 {
    if t.is_empty() {
        return 0.0;
    }
    if x <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        if x <= w[1].0 {
            let d = w[1].0 - w[0].0;
            return w[0].1 + if d > 1e-9 { (x - w[0].0) / d } else { 1.0 } * (w[1].1 - w[0].1);
        }
    }
    t[t.len() - 1].1
}

#[derive(Default)]
struct GroundOut {
    force_world: Vec3,
    moment_body: Vec3,
    contact: bool,
    crash: Option<String>,
    comp: [f32; 8],
    /// Some contact was on land (gear, skids, scrapes) — not just the water.
    land: bool,
    /// The floats / hull touched the water (src/water.rs).
    water: bool,
    water_contact: crate::water::Contact,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Track H2b types against published numbers (src/flight/tests/ga_types.rs).
    mod ga_types;

    const DT: f32 = 1.0 / 120.0;

    /// The fixed-wing roster (rotorcraft have their own tests in src/rotor).
    fn fixed_wing() -> Vec<AircraftDef> {
        catalogue().into_iter().filter(|d| d.rotor.is_none()).collect()
    }

    /// Flat ground at y = 0 with optional water rectangles, buildings and a
    /// uniform updraft.
    #[derive(Default)]
    struct TestEnv {
        water: Vec<(Vec2, Vec2)>,
        buildings: Vec<(Vec2, Vec2, f32)>,
        updraft: f32,
        wind: Vec3,
    }

    fn inside(p: Vec2, lo: Vec2, hi: Vec2) -> bool {
        p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y
    }

    impl Environment for TestEnv {
        fn ground_height(&self, _xz: Vec2) -> f32 {
            0.0
        }
        fn obstacle_top(&self, xz: Vec2) -> Option<f32> {
            self.buildings.iter().find(|(lo, hi, _)| inside(xz, *lo, *hi)).map(|b| b.2)
        }
        fn is_water(&self, xz: Vec2) -> bool {
            self.water.iter().any(|(lo, hi)| inside(xz, *lo, *hi))
        }
        fn updraft(&self, _p: Vec3) -> f32 {
            self.updraft
        }
        fn wind(&self, _p: Vec3) -> Vec3 {
            self.wind
        }
    }

    /// `cargo test --release jsb_diag -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn jsb_diag() {
        let only = std::env::var("DIAG").ok();
        for def in fixed_wing() {
            if only.as_deref().is_some_and(|o| o != def.id) {
                continue;
            }
            if let Some(j) = &def.jsb {
                println!("== {} signs e{} a{} r{}", def.id, j.sign_e, j.sign_a, j.sign_r);
            } else {
                println!("== {} (parametric)", def.id);
            }
            println!("   cl0 {:.3} cla {:.2} a_st {:.1} drop {:.2} cd0 {:.4} e {:.2} flap_cl {:.2} flap_cd {:.3} flap_cm {:.3} flap_as {:.1}",
                def.cl0, def.cl_alpha, def.alpha_stall.to_degrees(), def.stall_drop, def.cd0, def.oswald, def.flap_cl, def.flap_cd, def.flap_cm, def.flap_alpha_stall.to_degrees());
            println!("   cm0 {:.3} cma {:.3} cmq {:.1} cme {:.3} | cyb {:.3} clb {:.3} clp {:.3} cnb {:.3} cnr {:.3} cnp {:.3} clr/cl {:.3} cla {:.3} cna {:.4} cnr_ {:.3} spl {:.2}/{:.3} cdg {:.3}",
                def.cm0, def.cm_alpha, def.cm_q, def.cm_elevator, def.cy_beta, def.cl_beta, def.cl_p, def.cn_beta, def.cn_r, def.cn_p, def.cl_r_per_cl, def.cl_aileron, def.cn_aileron, def.cn_rudder, def.spoiler_cl, def.spoiler_cd, def.cd_gear);
            println!("   vs1 est {:.0} kt (flaps full {:.0} kt)", def.stall_speed(0.0) / KT, def.stall_speed(1.0) / KT);
            let env = TestEnv::default();
            let v = ref_kt(&def) * KT;
            let (mut a, c) = flying(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, v, 0.0);
            let tp = def.trim_point_at(v, 0.0, 1500.0, false);
            println!("   trim @{:.0} kt: alpha {:.1} pitch {:.1} thr {:.2} trim {:.2} elev {:.2}", v / KT, tp.alpha.to_degrees(), tp.pitch.to_degrees(), tp.throttle, tp.trim, tp.elevator);
            for k in 0..(30 * 120) {
                a.step(&def, &c, &env, DT);
                if k % (5 * 120) == 0 || a.crashed.is_some() {
                    println!("     t {:4.1} ias {:5.1} alt {:6.0} pitch {:5.1} roll {:5.1} alpha {:5.1} vs {:5.1} q {:6.3}", k as f32 * DT, a.airspeed_kt(), a.pos.y, a.pitch().to_degrees(), a.roll().to_degrees(), a.alpha.to_degrees(), a.vel.y, -a.omega.x);
                }
                if a.crashed.is_some() {
                    println!("     CRASH {:?}", a.crashed);
                    break;
                }
            }
        }
    }

    #[test]
    #[ignore]
    fn jsb_probe() {
        let id = std::env::var("DIAG").unwrap_or("kestrel".into());
        let def = find(&id).unwrap();
        let j = def.jsb.as_ref().unwrap();
        let v = ref_kt(&def) * KT;
        for adeg in [-4.0f32, 0.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0] {
            let al = adeg.to_radians();
            let vb = Vec3::new(0.0, -v * al.sin(), v * al.cos());
            let s = JsbSurf { gear: if def.retractable_gear { 0.0 } else { 1.0 }, ..Default::default() };
            let i = j.input(vb, Vec3::ZERO, 1.225, v / 340.0, &s, 300.0, 300.0, 0.0, 101_325.0);
            let o = j.aero.eval_steady(&i);
            println!("a {adeg:5.1} CL {:6.3} CD {:7.4} F {:?} M {:?}", o.cl, o.cd, o.force, o.moment);
        }
        for flaps in [0.0f32, 0.5, 1.0] {
            let mut best = (0.0f32, -9.0f32);
            for k in 0..60 {
                let adeg = k as f32 * 0.5;
                let al = adeg.to_radians();
                let vb = Vec3::new(0.0, -60.0 * al.sin(), 60.0 * al.cos());
                let s = JsbSurf { flaps, gear: 1.0, ..Default::default() };
                let i = j.input(vb, Vec3::ZERO, 1.225, 0.18, &s, 300.0, 300.0, 0.0, 101_325.0);
                let c = j.aero.eval_steady(&i).cl;
                if c > best.1 { best = (adeg, c); } else if c < best.1 - 0.12 { break; }
            }
            let vs = (2.0 * def.mass * GRAVITY / (1.225 * j.aero.sw_m2 as f32 * best.1)).sqrt();
            println!("flaps {flaps}: CLmax {:.2} at {:.1}°, Vs {:.0} kt (def vs1 {} vs0 {} vref {})", best.1, best.0, vs / KT, def.vs1_kt, def.vs0_kt, def.vref_kt);
        }
        // Damping derivatives at 2°.
        let al = 2f32.to_radians();
        let vb = Vec3::new(0.0, -v * al.sin(), v * al.cos());
        let s = JsbSurf { gear: if def.retractable_gear { 0.0 } else { 1.0 }, ..Default::default() };
        for w in [Vec3::ZERO, Vec3::new(0.05, 0.0, 0.0), Vec3::new(0.0, 0.05, 0.0), Vec3::new(0.0, 0.0, 0.05)] {
            let i = j.input(vb, w, 1.225, v / 340.0, &s, 300.0, 300.0, 0.0, 101_325.0);
            let o = j.aero.eval_steady(&i);
            println!("w {w:?} M {:?}", o.moment);
        }
    }

    #[test]
    #[ignore]
    fn ap_probe() {
        let id = std::env::var("DIAG").unwrap_or("meridian320".into());
        let kt: f32 = std::env::var("KT").ok().and_then(|t| t.parse().ok()).unwrap_or(220.0);
        let def = find(&id).unwrap();
        let env = TestEnv::default();
        let (mut a, mut c) = flying(&def, Vec3::new(0.0, 600.0, 0.0), 0.0, kt * KT, 0.0);
        let mut ap = crate::autopilot::Autopilot::new();
        ap.engage_hold_current(&a);
        ap.alt = Some(1000.0);
        ap.ias = Some(kt * KT);
        for k in 0..(240 * 120) {
            ap.update(&def, &a, &mut c, DT);
            a.step(&def, &c, &env, DT);
            if k % (20 * 120) == 0 {
                println!("t {:4.0} alt {:6.0} ias {:5.1} vs {:5.1} pitch {:5.1} alpha {:5.1} thr {:.2} elev_cmd {:.2} surf_e {:.2} n1 {:.2}", k as f32 * DT, a.pos.y, a.airspeed_kt(), a.vertical_speed(), a.pitch().to_degrees(), a.alpha.to_degrees(), c.throttle, c.elevator, a.surfaces().elevator, a.n1(0));
            }
        }
    }

    #[test]
    #[ignore]
    fn pull_probe() {
        let def = find("lancer").unwrap();
        let env = TestEnv::default();
        let (mut a, c) = flying(&def, Vec3::new(0.0, 140.0, 0.0), 0.0, 430.0 * KT, 0.0);
        let st: f32 = std::env::var("STICK").ok().and_then(|t| t.parse().ok()).unwrap_or(1.0);
        for k in 0..(8 * 120) {
            a.step(&def, &Controls { elevator: st, throttle: 0.9, ..c }, &env, DT);
            if k % 30 == 0 {
                println!("t {:4.2} alt {:6.0} g {:5.2} alpha {:5.1} pitch {:5.1} ias {:5.0} e {:.2} q {:.2}", k as f32 * DT, a.pos.y, a.g_load(), a.alpha.to_degrees(), a.pitch().to_degrees(), a.airspeed_kt(), a.surfaces().elevator, -a.omega.x);
            }
        }
    }

    #[test]
    #[ignore]
    fn power_probe() {
        let id = std::env::var("DIAG").unwrap_or("meridian320".into());
        let def = find(&id).unwrap();
        let (a, _) = flying(&def, Vec3::new(0.0, 1000.0, 0.0), 0.0, 165.0 * KT, 0.25);
        let vb = a.rot.conjugate() * a.vel;
        let o = a.aero(&def, vb, 1000.0, &isa(1000.0), 0.0);
        println!("powers e {:.0} a {:.0} r {:.0} damping {:?}", o.elev_power, o.ail_power, o.rudder_power, o.damping);
    }

    #[test]
    #[ignore]
    fn ground_probe() {
        let id = std::env::var("DIAG").unwrap_or("meridian320".into());
        let def = find(&id).unwrap();
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        let thr: f32 = std::env::var("THR").ok().and_then(|t| t.parse().ok()).unwrap_or(0.0);
        let brk: f32 = std::env::var("BRK").ok().and_then(|t| t.parse().ok()).unwrap_or(if thr > 0.0 { 0.0 } else { 1.0 });
        let el: f32 = std::env::var("ELEV").ok().and_then(|t| t.parse().ok()).unwrap_or(0.0);
        let c = Controls { throttle: thr, brakes: brk, elevator: el, ..Default::default() };
        for k in 0..(40 * 120) {
            a.step(&def, &c, &env, DT);
            if k % 120 == 0 || a.crashed.is_some() {
                println!("t {:4.1} y {:6.3} pitch {:6.2} roll {:6.2} gs {:5.1} w {:?} gnd {} alpha {:5.1}", k as f32 * DT, a.pos.y, a.pitch().to_degrees(), a.roll().to_degrees(), a.vel.length() / KT, a.omega, a.on_ground, a.alpha.to_degrees());
            }
            if a.crashed.is_some() { println!("CRASH {:?}", a.crashed); break; }
        }
    }

    fn kestrel() -> AircraftDef {
        fixed_wing().remove(0)
    }
    fn swift() -> AircraftDef {
        fixed_wing().remove(1)
    }
    fn ac(id: &str) -> AircraftDef {
        find(id).unwrap()
    }

    /// A comfortable reference cruise speed for the generic handling tests (kt IAS).
    fn ref_kt(def: &AircraftDef) -> f32 {
        match def.id {
            "kestrel" => 90.0,
            "swift" => 55.0,
            "peregrine" => 200.0,
            "lancer" => 350.0,
            "aerolink" => 180.0,
            "hercules" => 200.0,
            // Below ~300 kt the slender delta is on the back of the drag curve
            // (speed-unstable at a fixed throttle, as the real one: autothrottle).
            "concorde" => 380.0,
            "extra500" => 160.0,
            "cub" => 80.0,
            "zlin" => 120.0,
            "otter" => 95.0,
            "dc3" => 140.0,
            _ => 250.0,
        }
    }

    fn assert_sane(a: &Aircraft) {
        assert!(a.pos.is_finite() && a.vel.is_finite() && a.rot.is_finite() && a.omega.is_finite());
        assert!((a.rot.length() - 1.0).abs() < 1e-3);
        assert!(a.alpha().is_finite() && a.g_load().is_finite() && a.engine_rpm().is_finite());
    }

    /// Steady straight flight with the flap lever already at `flap` (gear down
    /// if flaps are out). `speed` is indicated.
    fn flying(def: &AircraftDef, pos: Vec3, heading: f32, speed: f32, flap: f32) -> (Aircraft, Controls) {
        let gear_down = !def.retractable_gear || flap > 0.0;
        let mut a = Aircraft::airborne(def, pos, heading, speed);
        let tp = def.trim_point_at(speed, flap, pos.y, gear_down);
        let tas = tas_from_cas(speed, pos.y);
        a.rot = attitude(heading, tp.pitch, 0.0);
        a.vel = attitude(heading, tp.gamma, 0.0) * Vec3::Z * tas;
        a.air_vel = a.vel;
        a.surf.flaps = flap;
        a.peak_flap = flap;
        a.alpha_peak = def.alpha_usable(flap);
        a.gear_pos = if gear_down { 1.0 } else { 0.0 };
        a.surf.gear = 1.0 - a.gear_pos;
        a.settle_engines(def, tp.throttle, tas * tp.alpha.cos(), &isa(pos.y));
        a.trim = tp.trim;
        a.surf.elevator = if def.control_law == ControlLaw::Direct { (tp.trim * def.trim_authority).clamp(-1.0, 1.0) } else { tp.elevator };
        let c = Controls { throttle: tp.throttle, trim: tp.trim, flaps: flap, gear_down, ..Default::default() };
        (a, c)
    }

    /// Simple test-pilot autopilot.
    #[derive(Default)]
    struct Pilot {
        pitch_i: f32,
        outer_i: f32,
        init: bool,
    }

    enum Vert {
        Speed(f32),
        Alt(f32),
        Pitch(f32),
    }

    impl Pilot {
        /// Returns controls given the target vertical mode and bank (rad).
        fn fly(&mut self, def: &AircraftDef, a: &Aircraft, base: Controls, vert: Vert, bank: f32, dt: f32) -> Controls {
            if !self.init {
                self.outer_i = a.pitch();
                self.init = true;
            }
            let pitch_cmd = match vert {
                Vert::Pitch(p) => p,
                Vert::Speed(v) => {
                    let e = a.airspeed() - v; // fast -> pitch up
                    self.outer_i = (self.outer_i + 0.004 * e * dt).clamp(-0.5, 0.5);
                    (self.outer_i + 0.012 * e).clamp(-0.5, 0.5)
                }
                Vert::Alt(h) => {
                    let vs_cmd = (0.25 * (h - a.pos.y)).clamp(-4.0, 4.0);
                    let e = vs_cmd - a.vertical_speed();
                    self.outer_i = (self.outer_i + 0.01 * e * dt).clamp(-0.5, 0.5);
                    (self.outer_i + 0.03 * e).clamp(-0.5, 0.5)
                }
            };
            let e = pitch_cmd - a.pitch();
            let rates = a.rates();
            let fbw = def.control_law != ControlLaw::Direct && a.fbw_active() > 0.5;
            let (elevator, aileron) = if fbw {
                // Stick = load-factor demand: ask for a pitch rate proportional to the error.
                let dnz = 6.0 * e;
                let el = (dnz / (def.nz_max - 1.0)).clamp(-1.0, 1.0);
                let ai = (1.5 * (bank - a.roll()) / def.roll_rate_max).clamp(-1.0, 1.0);
                (el, ai)
            } else {
                let heavy = def.category == Category::Airliner;
                let (kp, ki, kd) = if heavy { (2.5, 1.0, 2.0) } else { (3.0, 1.5, 0.6) };
                self.pitch_i = (self.pitch_i + ki * e * dt).clamp(-1.0, 1.0);
                let el = (kp * e + self.pitch_i - kd * rates.y).clamp(-1.0, 1.0);
                let (ka, kr) = if heavy { (2.5, 1.5) } else { (2.0, 0.4) };
                let ai = (ka * (bank - a.roll()) - kr * rates.x).clamp(-1.0, 1.0);
                (el, ai)
            };
            let rudder = (3.0 * a.beta()).clamp(-1.0, 1.0);
            Controls { elevator, aileron, rudder, ..base }
        }
    }

    fn run<F: FnMut(&Aircraft) -> Controls>(a: &mut Aircraft, def: &AircraftDef, env: &TestEnv, secs: f32, mut f: F) {
        let n = (secs / DT).round() as usize;
        for _ in 0..n {
            let c = f(a);
            a.step(def, &c, env, DT);
            assert_sane(a);
            if a.crashed.is_some() {
                break;
            }
        }
    }

    fn wrap(a: f32) -> f32 {
        let mut d = a.rem_euclid(TAU);
        if d > PI {
            d -= TAU;
        }
        d
    }

    // ------------------------------------------------------------------
    // Conventions
    // ------------------------------------------------------------------

    #[test]
    fn attitude_conventions() {
        for &h in &[0.0f32, 0.5, 1.5, -2.0, 3.0] {
            for &p in &[0.0f32, 0.3, -0.4] {
                for &r in &[0.0f32, 0.5, -1.0] {
                    let a = Aircraft { rot: attitude(h, p, r), ..Aircraft::blank(&kestrel()) };
                    assert!((a.heading() - h).abs() < 1e-4, "heading {h} {p} {r}: {}", a.heading());
                    assert!((a.pitch() - p).abs() < 1e-4);
                    assert!((a.roll() - r).abs() < 1e-4);
                    assert!((a.right() - a.forward().cross(a.up())).length() < 1e-5);
                }
            }
        }
        let east = Aircraft { rot: attitude(PI / 2.0, 0.0, 0.0), ..Aircraft::blank(&kestrel()) };
        assert!((east.forward() - Vec3::X).length() < 1e-5);
        assert!((east.right() - Vec3::Z).length() < 1e-5); // facing east, right wing points south
        let north = Aircraft { rot: Quat::from_rotation_y(PI), ..Aircraft::blank(&kestrel()) };
        assert!((north.forward() - Vec3::NEG_Z).length() < 1e-5);
        // Pitch-up is a rotation about local -X.
        let up = Aircraft { rot: Quat::from_rotation_y(PI) * Quat::from_axis_angle(Vec3::NEG_X, 0.2), ..Aircraft::blank(&kestrel()) };
        assert!((up.pitch() - 0.2).abs() < 1e-5);
    }

    #[test]
    fn catalogue_order() {
        let c = fixed_wing();
        let ids: Vec<_> = c.iter().map(|d| d.id).collect();
        assert_eq!(ids, ["kestrel", "swift", "peregrine", "lancer", "aerolink", "meridian190", "meridian320", "hercules", "galaxy", "vulcan", "extra500", "cub", "zlin", "otter", "dc3", "triple7", "crj700", "citationx", "concorde"]);
        assert!(c[0].has_engine && !c[1].has_engine);
        assert_eq!(c[1].flap_label, "Airbrakes");
        for d in &c {
            assert_eq!(d.engine_count, d.engine_pos.len(), "{}", d.id);
            assert!(d.engine_count <= MAX_ENGINES);
            assert_eq!(d.has_engine, d.engine_count > 0);
            assert_eq!(d.has_engine, d.engine_kind != EngineKind::None);
            assert!(d.vr_kt > 0.0 && d.vref_kt > 0.0 && d.vle_kt > 0.0 && d.ceiling_m > 0.0);
            assert!(d.vs0_kt <= d.vs1_kt && d.vs1_kt < d.vno_kt && d.vno_kt <= d.vne_kt);
            // (A flapless type, e.g. the Zlin aerobat, has the single detent 0.)
            assert!(*d.flap_detents.last().unwrap() > 0.0 || (d.flap_detents.len() == 1 && d.flap_max_deg == 0.0));
            if d.retractable_gear {
                assert!(d.gear.iter().any(|g| g.retracts) && d.gear_time > 0.0);
            }
        }
    }

    /// The shared geometry table in docs/V2_INTERFACES.md (wheel contacts, engine
    /// pivots and eye points must match the 3D models exactly).
    #[test]
    fn model_and_flight_geometry_agree() {
        // The flight geometry (FDM contacts, eye, engines) and the imported
        // visual model must agree: wheels touch the runway, the cockpit eye is
        // the model's.
        for def in fixed_wing() {
            let Ok(asset) = crate::aircraft_asset::AircraftAsset::load(def.id) else { continue };
            let eye = asset.eye.expect("model eye");
            assert!(eye.distance(def.eye) < 0.05, "{} eye {:?} vs model {:?}", def.id, def.eye, eye);
            // Pose with the gear down; lowest exterior vertex near each wheel.
            let mut ch = vec![0.0f32; asset.channels.len()];
            for (i, c) in asset.channels.iter().enumerate() {
                if c == "gear_pos" {
                    ch[i] = 1.0;
                }
            }
            let mut st = crate::aircraft_asset::AnimState::default();
            let mut pose = crate::aircraft_asset::Pose::default();
            asset.pose(&ch, &mut st, 0.0, &mut pose);
            for g in def.gear.iter().filter(|g| matches!(g.kind, ContactKind::Wheel { .. })) {
                let mut low = f32::MAX;
                for c in asset.chunks.iter().filter(|c| !c.interior && pose.visible[c.node]) {
                    let m = pose.world[c.node];
                    for k in c.first..c.first + c.count {
                        let v = m.transform_point3(Vec3::from(asset.vertices[(asset.indices[k as usize] + c.base_vertex) as usize].pos));
                        if Vec2::new(v.x - g.pos.x, v.z - g.pos.z).length() < 0.4 {
                            low = low.min(v.y);
                        }
                    }
                }
                assert!((low - g.pos.y).abs() < 0.15, "{} wheel {:?}: model tyre bottom {low}", def.id, g.pos);
            }
            assert_eq!(def.engine_count, def.engine_pos.len());
        }
    }

    // ------------------------------------------------------------------
    // Atmosphere
    // ------------------------------------------------------------------

    #[test]
    fn isa_atmosphere() {
        let sl = isa(0.0);
        assert!((sl.density - 1.225).abs() < 1e-3);
        assert!((sl.temperature - 288.15).abs() < 1e-3);
        assert!((sl.speed_of_sound - 340.29).abs() < 0.1);
        let t = isa(11_000.0);
        assert!((t.temperature - 216.65).abs() < 0.01);
        assert!((t.density - 0.3639).abs() < 2e-3, "{}", t.density);
        assert!((t.speed_of_sound - 295.07).abs() < 0.1);
        assert!((t.pressure - 22_632.0).abs() < 30.0);
        let h = isa(5_000.0);
        assert!((h.density - 0.7364).abs() < 2e-3);
        let s = isa(15_000.0);
        assert!((s.density - 0.1948).abs() < 2e-3, "{}", s.density);
        // IAS / TAS.
        assert!((cas_from_tas(100.0, 0.0) - 100.0).abs() < 0.05);
        let tas = tas_from_cas(250.0 * KT, 10_668.0); // FL350
        eprintln!("250 KIAS at FL350 = {:.0} KTAS, M{:.3}", tas / KT, tas / isa(10_668.0).speed_of_sound);
        assert!((tas / KT - 427.0).abs() < 5.0);
        // Round trip incl. supersonic.
        for &(v, h) in &[(50.0f32, 0.0f32), (150.0, 3000.0), (240.0, 11_000.0), (600.0, 12_000.0), (420.0, 0.0)] {
            let c = cas_from_tas(v, h);
            assert!((tas_from_cas(c, h) - v).abs() < 0.05, "{v} {h}");
        }
        // M2 at 11 km indicates about 710 kt.
        let c = cas_from_tas(2.0 * t.speed_of_sound, 11_000.0) / KT;
        eprintln!("M2 at 11 km = {c:.0} KIAS");
        assert!((690.0..730.0).contains(&c));
    }

    #[test]
    fn airspeed_is_indicated_at_altitude() {
        for def in fixed_wing() {
            let kt = ref_kt(&def);
            let alt = if def.category == Category::Airliner || def.category == Category::FastJet { 9000.0 } else { 3000.0 };
            let a = Aircraft::airborne(&def, Vec3::new(0.0, alt, 0.0), 0.0, kt * KT);
            assert!((a.airspeed_kt() - kt).abs() < 0.2, "{} ias {}", def.id, a.airspeed_kt());
            assert!(a.tas() > a.airspeed() * 1.1);
            assert!((a.mach() - a.tas() / isa(alt).speed_of_sound).abs() < 1e-3);
        }
    }

    // ------------------------------------------------------------------
    // Report: prints the performance table (run with --nocapture).
    // ------------------------------------------------------------------

    #[test]
    fn static_polar_report() {
        for def in fixed_wing() {
            eprintln!("== {} ==", def.name);
            let full = *def.flap_detents.last().unwrap();
            for &d in def.flap_detents {
                let f = d / full;
                eprintln!(
                    "  flap {:.2}: CLmax {:.3} at {:.1}°, Vs {:.1} kt",
                    f,
                    def.cl_max(f),
                    def.alpha_peak(f).to_degrees(),
                    def.stall_speed(f) / KT
                );
            }
            let v0 = (def.vs1_kt * 0.85).round();
            for i in 0..12 {
                let kt = v0 + i as f32 * def.vs1_kt * 0.15;
                let tp = def.trim_point(kt * KT, 0.0);
                let ld = def.lift_drag(tp.alpha, 0.0, 1.0);
                eprintln!(
                    "  {:5.0} kt: alpha {:5.1}° gamma {:5.2}° L/D {:5.1} sink {:5.2} thr {:.2} trim {:+.2}",
                    kt,
                    tp.alpha.to_degrees(),
                    tp.gamma.to_degrees(),
                    ld.cl / ld.cd,
                    -kt * KT * tp.gamma.sin(),
                    tp.throttle,
                    tp.trim
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Performance (v1 aircraft)
    // ------------------------------------------------------------------

    /// Slowly decelerate holding altitude at idle until the wing stalls;
    /// returns the minimum airspeed held (kt).
    fn sim_stall_speed(def: &AircraftDef, flap: f32) -> f32 {
        let env = TestEnv::default();
        let v0 = def.stall_speed(flap) * 1.35;
        let (mut a, base) = flying(def, Vec3::new(0.0, 1000.0, 0.0), 0.0, v0, flap);
        let base = Controls { throttle: 0.0, ..base };
        let mut pilot = Pilot::default();
        let mut min_v = f32::MAX;
        let mut t = 0.0;
        let a_lim = def.alpha_usable(flap);
        while t < 150.0 {
            let c = pilot.fly(def, &a, base, Vert::Alt(1000.0), 0.0, DT);
            a.step(def, &c, &env, DT);
            assert_sane(&a);
            t += DT;
            if a.pos.y < 1000.0 - 15.0 || a.alpha() > a_lim - deg(0.3) {
                break;
            }
            min_v = min_v.min(a.airspeed());
        }
        assert!(a.crashed.is_none());
        min_v / KT
    }

    #[test]
    fn stall_speeds() {
        let k = kestrel();
        let clean = sim_stall_speed(&k, 0.0);
        let full = sim_stall_speed(&k, 1.0);
        let g = sim_stall_speed(&swift(), 0.0);
        eprintln!("stall: kestrel clean {clean:.1} kt, flap30 {full:.1} kt; swift {g:.1} kt");
        assert!((45.0..=54.0).contains(&clean), "kestrel clean stall {clean}");
        assert!((39.0..=49.0).contains(&full), "kestrel full-flap stall {full}");
        assert!((33.0..=40.0).contains(&g), "swift stall {g}");
    }

    #[test]
    fn stall_warning_and_break() {
        let k = kestrel();
        let env = TestEnv::default();
        let (mut a, base) = flying(&k, Vec3::new(0.0, 1000.0, 0.0), 0.0, 60.0 * KT, 0.0);
        let base = Controls { throttle: 0.0, ..base };
        let mut pilot = Pilot::default();
        let mut warned_at = None;
        for i in 0..(60.0 / DT) as usize {
            let c = pilot.fly(&k, &a, base, Vert::Alt(1000.0), 0.0, DT);
            a.step(&k, &c, &env, DT);
            if warned_at.is_none() && a.stall_warning() {
                warned_at = Some((i, a.airspeed() / KT));
            }
        }
        let (_, v) = warned_at.expect("stall warning never sounded");
        eprintln!("stall warning at {v:.1} kt");
        assert!(v > 48.0 && v < 58.0);
        // Cruise: no warning.
        let (a, _) = flying(&k, Vec3::new(0.0, 1000.0, 0.0), 0.0, 100.0 * KT, 0.0);
        assert!(!a.stall_warning());
    }

    #[test]
    fn kestrel_cruise_full_throttle() {
        let k = kestrel();
        let env = TestEnv::default();
        let (mut a, base) = flying(&k, Vec3::new(0.0, 500.0, 0.0), 0.0, 100.0 * KT, 0.0);
        let base = Controls { throttle: 1.0, ..base };
        let mut pilot = Pilot::default();
        run(&mut a, &k, &env, 240.0, |a| pilot.fly(&k, a, base, Vert::Alt(500.0), 0.0, DT));
        let v = a.airspeed() / KT;
        eprintln!("cruise full throttle at 500 m: {v:.1} KIAS ({:.1} KTAS), {:.0} rpm", a.tas() / KT, a.engine_rpm());
        assert!((108.0..=121.0).contains(&v), "cruise {v}");
        assert!((500.0 - a.pos.y).abs() < 5.0);
        assert!((2550.0..2800.0).contains(&a.engine_rpm()));
    }

    #[test]
    fn kestrel_best_rate_climb() {
        let k = kestrel();
        let env = TestEnv::default();
        let (mut a, base) = flying(&k, Vec3::new(0.0, 300.0, 0.0), 0.0, 74.0 * KT, 0.0);
        let base = Controls { throttle: 1.0, ..base };
        let mut pilot = Pilot::default();
        run(&mut a, &k, &env, 40.0, |a| pilot.fly(&k, a, base, Vert::Speed(74.0 * KT), 0.0, DT));
        let h0 = a.pos.y;
        run(&mut a, &k, &env, 30.0, |a| pilot.fly(&k, a, base, Vert::Speed(74.0 * KT), 0.0, DT));
        let roc = (a.pos.y - h0) / 30.0 / FPM;
        let v = a.airspeed() / KT;
        eprintln!("climb at {v:.1} kt: {roc:.0} fpm");
        assert!((v - 74.0).abs() < 2.0);
        assert!((600.0..=780.0).contains(&roc), "climb {roc}");
        let mut best = (0.0, 0.0);
        for kt in [60.0f32, 66.0, 70.0, 74.0, 78.0, 82.0, 90.0] {
            let (mut a, base) = flying(&k, Vec3::new(0.0, 300.0, 0.0), 0.0, kt * KT, 0.0);
            let base = Controls { throttle: 1.0, ..base };
            let mut pilot = Pilot::default();
            run(&mut a, &k, &env, 40.0, |a| pilot.fly(&k, a, base, Vert::Speed(kt * KT), 0.0, DT));
            let h0 = a.pos.y;
            run(&mut a, &k, &env, 20.0, |a| pilot.fly(&k, a, base, Vert::Speed(kt * KT), 0.0, DT));
            let r = (a.pos.y - h0) / 20.0 / FPM;
            eprintln!("   climb {kt} kt: {r:.0} fpm");
            if r > best.1 {
                best = (kt, r);
            }
        }
        assert!((66.0..=82.0).contains(&best.0), "best rate speed {}", best.0);
    }

    #[test]
    fn kestrel_takeoff_roll() {
        let k = kestrel();
        let r = takeoff(&k, 1.0 / 3.0, deg(8.0), false, true);
        eprintln!("kestrel takeoff: ground roll {:.0} m in {:.1} s at {:.0} kt, drift {:.2} m", r.roll_m, r.time_s, r.liftoff_kt, r.drift_m);
        // C172P POH: ~220 m (720 ft) ground roll at this weight, flaps 10°.
        assert!((170.0..=300.0).contains(&r.roll_m), "ground roll {}", r.roll_m);
        assert!(r.drift_m < 5.0);
    }

    /// Steady glide at `kt` with the lever at `flap`: returns (L/D, sink m/s).
    fn glide(def: &AircraftDef, kt: f32, flap: f32) -> (f32, f32) {
        let env = TestEnv::default();
        let (mut a, base) = flying(def, Vec3::new(0.0, 2000.0, 0.0), 0.0, kt * KT, flap);
        let base = Controls { throttle: 0.0, ..base };
        let mut pilot = Pilot::default();
        run(&mut a, def, &env, 40.0, |a| pilot.fly(def, a, base, Vert::Speed(kt * KT), 0.0, DT));
        let p0 = a.pos;
        run(&mut a, def, &env, 40.0, |a| pilot.fly(def, a, base, Vert::Speed(kt * KT), 0.0, DT));
        assert!(a.crashed.is_none());
        let d = a.pos - p0;
        let horiz = Vec2::new(d.x, d.z).length();
        (horiz / -d.y, -d.y / 40.0)
    }

    #[test]
    fn kestrel_power_off_glide() {
        let k = kestrel();
        let mut best = (0.0, 0.0);
        for kt in [60.0f32, 65.0, 70.0, 75.0] {
            let (ld, sink) = glide(&k, kt, 0.0);
            eprintln!("kestrel glide {kt} kt: L/D {ld:.1}, sink {:.0} fpm", sink / FPM);
            if ld > best.1 {
                best = (kt, ld);
            }
        }
        // POH ~9:1 with the prop stopped; the c172p FDM with a windmilling prop ~8.
        assert!((7.0..=10.0).contains(&best.1), "kestrel best glide {}", best.1);
    }

    #[test]
    fn swift_polar() {
        let g = swift();
        let mut best = (0.0, 0.0);
        let mut min_sink = (0.0, f32::MAX);
        for kt in [42.0f32, 45.0, 48.0, 51.0, 54.0, 57.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0] {
            let (ld, sink) = glide(&g, kt, 0.0);
            eprintln!("swift glide {kt} kt: L/D {ld:.1}, sink {sink:.2} m/s");
            if ld > best.1 {
                best = (kt, ld);
            }
            if sink < min_sink.1 {
                min_sink = (kt, sink);
            }
        }
        eprintln!("swift best L/D {:.1} at {} kt, min sink {:.2} at {} kt", best.1, best.0, min_sink.1, min_sink.0);
        // The FlightGear ASK 21 JSBSim polar is a little better than the
        // published 34:1 (it peaks near 42:1).
        assert!((33.0..=45.0).contains(&best.1));
        assert!((48.0..=65.0).contains(&best.0));
        // Sink is true vertical speed at 2 km, so ~10 % above the sea-level polar.
        assert!((0.55..=0.85).contains(&min_sink.1));
        assert!((42.0..=48.0).contains(&min_sink.0));
        // Airbrakes at least halve the glide ratio.
        let (ld_b, sink_b) = glide(&g, 55.0, 1.0);
        eprintln!("swift full airbrakes at 55 kt: L/D {ld_b:.1}, sink {sink_b:.2} m/s");
        assert!(ld_b < 0.5 * best.1);
        assert!(ld_b > 3.5);
    }

    #[test]
    fn thermal_updraft_lifts_glider() {
        let g = swift();
        let env = TestEnv { updraft: 2.0, ..Default::default() };
        let (mut a, base) = flying(&g, Vec3::new(0.0, 1000.0, 0.0), 0.0, 50.0 * KT, 0.0);
        let mut pilot = Pilot::default();
        run(&mut a, &g, &env, 30.0, |a| pilot.fly(&g, a, base, Vert::Speed(50.0 * KT), 0.0, DT));
        assert!(a.vertical_speed() > 1.0, "vs in thermal {}", a.vertical_speed());
        assert!((a.airspeed() / KT - 50.0).abs() < 3.0);
    }

    // ------------------------------------------------------------------
    // Wind
    // ------------------------------------------------------------------

    /// Start in steady flight inside an air mass moving with `wind`.
    fn flying_in_wind(def: &AircraftDef, heading: f32, kt: f32, wind: Vec3) -> (Aircraft, Controls) {
        let (mut a, c) = flying(def, Vec3::new(0.0, 900.0, 0.0), heading, kt * KT, 0.0);
        a.vel += wind;
        (a, c)
    }

    #[test]
    fn groundspeed_is_airspeed_plus_wind() {
        let def = kestrel();
        // 12 m/s tailwind (from the south, heading north = -Z).
        let wind = Vec3::new(0.0, 0.0, -12.0);
        let env = TestEnv { wind, ..Default::default() };
        let (mut a, base) = flying_in_wind(&def, 0.0, 90.0, wind);
        let mut pilot = Pilot::default();
        run(&mut a, &def, &env, 40.0, |a| pilot.fly(&def, a, base, Vert::Alt(900.0), 0.0, DT));
        let gs = Vec2::new(a.vel.x, a.vel.z).length();
        let tas = a.tas();
        assert!((a.airspeed_kt() - 90.0).abs() < 4.0, "ias {}", a.airspeed_kt());
        assert!((gs - (tas + 12.0)).abs() < 1.0, "gs {gs} tas {tas}");
        // And into the same wind: groundspeed = TAS - wind.
        let (mut a, base) = flying_in_wind(&def, std::f32::consts::PI, 90.0, wind);
        let mut pilot = Pilot::default();
        run(&mut a, &def, &env, 40.0, |a| pilot.fly(&def, a, base, Vert::Alt(900.0), 0.0, DT));
        let gs = Vec2::new(a.vel.x, a.vel.z).length();
        assert!((gs - (a.tas() - 12.0)).abs() < 1.0, "gs {gs} tas {}", a.tas());
    }

    #[test]
    fn crosswind_drifts_and_needs_a_crab() {
        let def = kestrel();
        // 10 m/s wind blowing east while heading north.
        let wind = Vec3::new(10.0, 0.0, 0.0);
        let env = TestEnv { wind, ..Default::default() };
        let (mut a, base) = flying_in_wind(&def, 0.0, 90.0, wind);
        let mut pilot = Pilot::default();
        let x0 = a.pos.x;
        run(&mut a, &def, &env, 60.0, |a| pilot.fly(&def, a, base, Vert::Alt(900.0), 0.0, DT));
        let drift = a.pos.x - x0;
        assert!((drift - 600.0).abs() < 60.0, "drift {drift} m");
        // Heading stays north (the nose points into the airflow) but the track is right of it.
        let track = a.vel.x.atan2(-a.vel.z).to_degrees();
        assert!(a.heading().to_degrees().abs() < 4.0 || (a.heading().to_degrees() - 360.0).abs() < 4.0, "hdg {}", a.heading().to_degrees());
        assert!((track - (10.0f32).atan2(46.3).to_degrees()).abs() < 3.0, "track {track}");
        // The airflow stays straight down the nose: no sideslip from steady wind.
        assert!(a.beta().abs() < 2f32.to_radians(), "beta {}", a.beta().to_degrees());
    }

    #[test]
    fn glider_drifts_with_the_air_mass() {
        let g = swift();
        let wind = Vec3::new(0.0, 0.0, 8.0); // blowing south
        let env = TestEnv { wind, updraft: 1.5, ..Default::default() };
        let (mut a, base) = flying_in_wind(&g, std::f32::consts::FRAC_PI_2, 50.0, wind);
        let mut pilot = Pilot::default();
        let z0 = a.pos.z;
        // Circling in a thermal: the circles drift downwind.
        run(&mut a, &g, &env, 120.0, |a| pilot.fly(&g, a, base, Vert::Speed(50.0 * KT), 30f32.to_radians(), DT));
        assert!(a.crashed.is_none());
        let dz = a.pos.z - z0;
        assert!((dz - 960.0).abs() < 200.0, "drift south {dz}");
    }

    #[test]
    fn landing_roll_in_a_crosswind_stays_sane() {
        // Parked into a 25 kt wind gusting about: no NaNs, no lift-off.
        let def = kestrel();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &TestEnv::default());
        for i in 0..(30.0 / DT) as usize {
            let t = i as f32 * DT;
            let gust = 13.0 + 4.0 * (t * 1.3).sin() * (t * 0.37).cos();
            let env = TestEnv { wind: Vec3::new(gust * 0.6, 0.5 * (t * 2.1).sin(), gust * 0.8), ..Default::default() };
            a.step(&def, &Controls { brakes: 1.0, ..Default::default() }, &env, DT);
            assert_sane(&a);
        }
        assert!(a.crashed.is_none() && a.on_ground, "{:?}", a.crashed);
        assert!(Vec2::new(a.vel.x, a.vel.z).length() < 1.0);
    }

    // ------------------------------------------------------------------
    // Handling and stability (all aircraft)
    // ------------------------------------------------------------------

    #[test]
    fn hands_off_trimmed_stability() {
        for def in fixed_wing() {
            let env = TestEnv::default();
            let speed = ref_kt(&def) * KT;
            let ctl = trimmed_controls_at(&def, speed, 1500.0);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 1.0, speed);
            let mut max_dv = 0.0f32;
            let mut max_bank = 0.0f32;
            let mut max_dvs = 0.0f32;
            let vs0 = a.vertical_speed();
            run(&mut a, &def, &env, 60.0, |a| {
                max_dv = max_dv.max((a.airspeed() - speed).abs());
                max_bank = max_bank.max(a.roll().abs());
                max_dvs = max_dvs.max((a.vertical_speed() - vs0).abs());
                ctl
            });
            eprintln!(
                "{} hands-off 60 s: max dV {:.1} kt, max bank {:.2}°, max dVS {:.2} m/s, heading drift {:.2}°",
                def.name,
                max_dv / KT,
                max_bank.to_degrees(),
                max_dvs,
                (a.heading() - 1.0).to_degrees()
            );
            assert!(a.crashed.is_none());
            assert!(max_dv < 4.0 * KT, "{} speed wandered {}", def.name, max_dv / KT);
            assert!(max_bank < deg(1.0), "{} bank {}", def.name, max_bank.to_degrees());
            assert!(max_dvs < 1.0, "{} dvs {}", def.name, max_dvs);
        }
    }

    #[test]
    fn hands_off_disturbed_recovers() {
        // A pitch-up kick and a roll disturbance: phugoid and spiral must not diverge.
        for def in fixed_wing() {
            let env = TestEnv::default();
            let speed = ref_kt(&def) * KT;
            let ctl = trimmed_controls_at(&def, speed, 1500.0);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, speed);
            a.omega = Vec3::new(-0.15, 0.0, 0.1); // pitch up + roll right
            let mut max_bank = 0.0f32;
            let mut late_dv = 0.0f32;
            let mut t = 0.0;
            run(&mut a, &def, &env, 90.0, |a| {
                max_bank = max_bank.max(a.roll().abs());
                t += DT;
                if t > 60.0 {
                    late_dv = late_dv.max((a.airspeed() - speed).abs());
                }
                ctl
            });
            eprintln!(
                "{} disturbed: max bank {:.1}°, final bank {:.1}°, late dV {:.1} kt",
                def.name,
                max_bank.to_degrees(),
                a.roll().to_degrees(),
                late_dv / KT
            );
            assert!(a.crashed.is_none(), "{} {:?}", def.name, a.crashed);
            assert!(max_bank < deg(45.0));
            assert!(late_dv < 12.0 * KT, "{} late dv {}", def.name, late_dv / KT);
        }
    }

    #[test]
    fn release_controls_in_gentle_bank() {
        for def in fixed_wing() {
            let env = TestEnv::default();
            let speed = ref_kt(&def) * KT;
            let ctl = trimmed_controls_at(&def, speed, 1500.0);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, speed);
            let mut pilot = Pilot::default();
            run(&mut a, &def, &env, 10.0, |a| pilot.fly(&def, a, ctl, Vert::Pitch(a.pitch()), deg(15.0), DT));
            let bank0 = a.roll();
            let h0 = a.heading();
            let mut max_bank = 0.0f32;
            let mut max_rate = 0.0f32;
            run(&mut a, &def, &env, 30.0, |a| {
                max_bank = max_bank.max(a.roll().abs());
                max_rate = max_rate.max(a.rates().length());
                ctl
            });
            eprintln!(
                "{} released at {:.1}° bank: after 30 s bank {:.1}°, max {:.1}°, turned {:.0}°, max rate {:.2}",
                def.name,
                bank0.to_degrees(),
                a.roll().to_degrees(),
                max_bank.to_degrees(),
                (a.heading() - h0).rem_euclid(TAU).to_degrees(),
                max_rate
            );
            assert!(a.crashed.is_none());
            assert!(max_bank < deg(35.0), "{} spiralled to {}", def.name, max_bank.to_degrees());
            assert!(max_rate < 0.5);
            // It keeps turning the way it was banked.
            // (30° for the v1 aircraft; half the ideal turn at 15° of bank for fast ones.)
            let ideal = 30.0 * GRAVITY * deg(15.0).tan() / a.tas();
            assert!((a.heading() - h0).rem_euclid(TAU) > deg(30.0).min(0.5 * ideal), "{} turned {}", def.name, (a.heading() - h0).to_degrees());
        }
    }

    #[test]
    fn bank_turn_360() {
        for def in fixed_wing() {
            let env = TestEnv::default();
            let speed = ref_kt(&def) * KT;
            let ctl = trimmed_controls_at(&def, speed, 1500.0);
            let ctl = if def.has_engine { Controls { throttle: (ctl.throttle + 0.15).min(1.0), ..ctl } } else { ctl };
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, speed);
            let mut pilot = Pilot::default();
            let mut turned = 0.0f32;
            let mut prev = a.heading();
            let mut t = 0.0;
            let mut max_slip = 0.0f32;
            let mut max_bank_err = 0.0f32;
            let h0 = a.pos.y;
            let vert = |h0: f32| if def.has_engine { Vert::Alt(h0) } else { Vert::Speed(55.0 * KT) };
            let tas = a.tas();
            let expect = TAU * tas / (GRAVITY * deg(30.0).tan());
            while turned < TAU && t < expect * 2.0 + 30.0 {
                let mut c = pilot.fly(&def, &a, ctl, vert(h0), deg(30.0), DT);
                c.rudder = 0.0; // feet on the floor: adverse yaw and weathercocking only
                a.step(&def, &c, &env, DT);
                assert_sane(&a);
                assert!(a.crashed.is_none());
                let h = a.heading();
                turned += wrap(h - prev);
                prev = h;
                t += DT;
                if std::env::var("TRACE").is_ok() && (t * 120.0).round() as i32 % 600 == 0 {
                    eprintln!("  t {t:.0} bank {:.1} pitch {:.1} nz {:.2} el {:.2} ias {:.0} alt {:.0} turned {:.0}", a.roll().to_degrees(), a.pitch().to_degrees(), a.g_load(), c.elevator, a.airspeed_kt(), a.pos.y, turned.to_degrees());
                }
                if t > 15.0 {
                    max_slip = max_slip.max(a.beta().abs());
                    max_bank_err = max_bank_err.max((a.roll() - deg(30.0)).abs());
                }
            }
            eprintln!(
                "{} 30° bank 360°: {:.1} s (ideal {:.1} s), alt change {:.1} m, max slip {:.1}°, g {:.2}, bank err {:.1}°",
                def.name,
                t,
                expect,
                a.pos.y - h0,
                max_slip.to_degrees(),
                a.g_load(),
                max_bank_err.to_degrees()
            );
            assert!(turned >= TAU, "{} did not complete the turn", def.name);
            assert!(t < expect * 1.4 + 10.0);
            assert!(max_slip < deg(5.0));
            assert!(max_bank_err < deg(5.0));
            if def.has_engine {
                assert!((a.pos.y - h0).abs() < 20.0, "{} alt {}", def.name, a.pos.y - h0);
            }
            assert!((a.g_load() - 1.155).abs() < 0.15);
        }
    }

    #[test]
    fn roll_rates() {
        for def in fixed_wing() {
            let env = TestEnv::default();
            let speed = ref_kt(&def) * KT;
            let ctl = trimmed_controls_at(&def, speed, 1500.0);
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, speed);
            let mut t45 = None;
            let mut t = 0.0;
            let mut pmax = 0.0f32;
            run(&mut a, &def, &env, 6.0, |a| {
                t += DT;
                pmax = pmax.max(a.rates().x);
                if t45.is_none() && a.roll() > deg(45.0) {
                    t45 = Some(t);
                }
                Controls { aileron: 1.0, ..ctl }
            });
            eprintln!("{} full aileron: 0→45° in {:.2} s, peak roll rate {:.0}°/s", def.name, t45.unwrap_or(99.0), pmax.to_degrees());
            let t45 = t45.unwrap();
            let range = match def.category {
                // Unlimited aerobat: ~200°/s+.
                _ if def.id == "zlin" => 0.12..0.6,
                // (The Super Cub's big fabric wing rolls lazily.)
                _ if def.id == "cub" => 0.6..2.8,
                Category::Trainer | Category::Warbird => 0.6..2.0,
                Category::Glider => 1.5..4.5,
                Category::FastJet => 0.15..0.8,
                Category::Airliner => 1.8..6.0,
                Category::Helicopter | Category::Drone => unreachable!(),
            };
            assert!(range.contains(&t45), "{} t45 {t45}", def.name);
            if def.id == "lancer" {
                assert!(pmax < deg(260.0) && pmax > deg(180.0), "lancer roll rate {}", pmax.to_degrees());
            }
            if def.id == "meridian320" {
                // Normal law: roll-rate demand 15°/s.
                assert!(pmax < deg(17.0), "A320-class roll rate {}", pmax.to_degrees());
            }
        }
    }

    #[test]
    fn rudder_and_sideslip() {
        let k = kestrel();
        let env = TestEnv::default();
        let ctl = trimmed_controls(&k, 80.0 * KT);
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 1000.0, 0.0), 0.0, 80.0 * KT);
        let mut pilot = Pilot::default();
        // Steady-heading sideslip: full right rudder, wings held with aileron.
        run(&mut a, &k, &env, 8.0, |a| {
            let mut c = pilot.fly(&k, a, ctl, Vert::Pitch(0.05), deg(-8.0), DT);
            c.rudder = 1.0;
            c
        });
        eprintln!("full rudder sideslip: beta {:.1}°", a.beta().to_degrees());
        // Right rudder yaws the nose right -> air comes from the left.
        assert!(a.beta() < deg(-6.0) && a.beta() > deg(-25.0));
    }

    // ------------------------------------------------------------------
    // Ground handling
    // ------------------------------------------------------------------

    #[test]
    fn rest_on_runway() {
        // (Floatplanes rest afloat: src/flight/tests/ga_types.rs.)
        for def in fixed_wing().into_iter().filter(|d| !d.on_water()) {
            let env = TestEnv::default();
            let mut a = Aircraft::on_runway(&def, Vec2::new(10.0, -20.0), 1.2, &env);
            let y0 = a.pos.y;
            eprintln!(
                "{} at rest: CG height {:.3} m (def {:.3}), pitch {:.2}°, rpm {:.0}, n1 {:.2}",
                def.name,
                a.pos.y,
                def.gear_height,
                a.pitch().to_degrees(),
                a.engine_rpm(),
                a.n1(0)
            );
            assert!((a.pos.y - def.gear_height).abs() < 0.03, "{} gear height {}", def.id, a.pos.y);
            assert!(a.on_ground && a.crashed.is_none());
            assert!((a.heading() - 1.2).abs() < 1e-3);
            assert_eq!(a.gear_pos(), 1.0);
            run(&mut a, &def, &env, 60.0, |_| Controls { brakes: 1.0, ..Default::default() });
            eprintln!("  after 60 s: moved {:.2} m, heading {:.1}°", (Vec2::new(a.pos.x, a.pos.z) - Vec2::new(10.0, -20.0)).length(), a.heading().to_degrees());
            assert!(a.crashed.is_none());
            assert!(a.vel.length() < 0.05, "{} creeping {}", def.name, a.vel.length());
            assert!((a.pos.y - y0).abs() < 0.01);
            assert!((Vec2::new(a.pos.x, a.pos.z) - Vec2::new(10.0, -20.0)).length() < 0.5);
            assert!(a.on_ground);
            if def.taildragger {
                // Spitfire IX 13.6°; the others' three-point attitude from their gear geometry.
                let three = if def.id == "peregrine" { deg(13.6) } else { def.rest_pitch() };
                assert!((a.pitch() - three).abs() < deg(1.0), "{} three-point attitude {}", def.id, a.pitch().to_degrees());
            } else if def.has_engine {
                // The c172p FDM's long nose strut sits ~4° nose-up.
                assert!(a.pitch().abs() < deg(5.0), "{} pitch {}", def.id, a.pitch().to_degrees());
            }
            assert!((a.g_load() - 1.0).abs() < 0.05, "{} g {}", def.id, a.g_load());
            // Weight on wheels: the gear will not retract.
            run(&mut a, &def, &env, 15.0, |_| Controls { brakes: 1.0, gear_down: false, ..Default::default() });
            assert_eq!(a.gear_pos(), 1.0);
            assert!(a.crashed.is_none());
        }
    }

    #[test]
    fn taxi_steer_and_brake() {
        let k = kestrel();
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&k, Vec2::ZERO, 0.0, &env);
        // Taxi with a bit of power, full right rudder: should turn right on the spot-ish.
        run(&mut a, &k, &env, 20.0, |_| Controls { throttle: 0.35, rudder: 1.0, ..Default::default() });
        let gs = Vec2::new(a.vel.x, a.vel.z).length();
        let turn = a.turn_rate();
        eprintln!("taxi: {:.1} m/s, turn rate {:.1}°/s, heading {:.0}°", gs, turn.to_degrees(), a.heading().to_degrees());
        assert!(gs > 2.0 && gs < 12.0);
        assert!(turn > deg(5.0));
        assert!(a.roll().abs() < deg(3.0));
        // Brake to a stop.
        run(&mut a, &k, &env, 10.0, |_| Controls { brakes: 1.0, ..Default::default() });
        assert!(a.vel.length() < 0.1);
        assert!(a.crashed.is_none());
    }

    #[test]
    fn high_speed_rudder_on_ground_is_gentle() {
        let k = kestrel();
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&k, Vec2::ZERO, 0.0, &env);
        run(&mut a, &k, &env, 12.0, |_| Controls { throttle: 1.0, ..Default::default() });
        run(&mut a, &k, &env, 3.0, |_| Controls { throttle: 1.0, rudder: 1.0, ..Default::default() });
        assert!(a.crashed.is_none(), "{:?}", a.crashed);
        assert!(a.roll().abs() < deg(8.0));
    }

    #[test]
    fn airliner_taxi_turns() {
        for id in ["aerolink", "meridian190", "meridian320", "lancer", "hercules", "galaxy", "vulcan"] {
            let def = ac(id);
            let env = TestEnv::default();
            let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
            // (The 280 t C-5 needs more than a quarter throttle to break away.)
            let thr = match id { "lancer" => 0.15, "galaxy" => 0.4, _ => 0.25 };
            let mut t = 0.0;
            run(&mut a, &def, &env, 40.0, |a| {
                t += DT;
                let gs = Vec2::new(a.vel.x, a.vel.z).length();
                let rud = if t > 10.0 { 1.0 } else { 0.0 };
                Controls { throttle: if gs < 8.0 { thr } else { 0.0 }, brakes: if gs > 10.0 { 0.3 } else { 0.0 }, rudder: rud, ..Default::default() }
            });
            let gs = Vec2::new(a.vel.x, a.vel.z).length();
            eprintln!("{id} taxi: {gs:.1} m/s, turn rate {:.1}°/s", a.turn_rate().to_degrees());
            assert!(a.crashed.is_none(), "{id} {:?}", a.crashed);
            assert!(gs > 2.0 && gs < 12.0, "{id} taxi speed {gs}");
            assert!(a.turn_rate() > deg(4.0), "{id} turn {}", a.turn_rate().to_degrees());
            run(&mut a, &def, &env, 15.0, |_| Controls { brakes: 1.0, ..Default::default() });
            assert!(a.vel.length() < 0.1);
        }
    }

    // ------------------------------------------------------------------
    // Take-off
    // ------------------------------------------------------------------

    struct TakeoffResult {
        roll_m: f32,
        liftoff_kt: f32,
        time_s: f32,
        drift_m: f32,
        /// Height above the runway 15 s after lift-off with the gear coming up.
        climb_15s: f32,
        max_heading_dev: f32,
    }

    /// Full-power take-off: hold the centreline with rudder, rotate at Vr to
    /// `pitch`; `tail_up`: raise the tail early (taildragger). Lift-off counts
    /// once airborne for 2 s; then gear up and a 15 s climb at the same pitch.
    fn takeoff(def: &AircraftDef, flap: f32, pitch: f32, tail_up: bool, rudder_pilot: bool) -> TakeoffResult {
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(def, Vec2::ZERO, 0.0, &env);
        let start = a.pos;
        a.surf.flaps = flap;
        let mut pilot = Pilot::default();
        let base = Controls { throttle: 1.0, flaps: flap, ..Default::default() };
        let mut t = 0.0f32;
        let mut airborne_since: Option<(f32, f32, f32)> = None;
        let mut liftoff = None;
        let mut max_dev = 0.0f32;
        while t < 120.0 {
            let ias = a.airspeed() / KT;
            let mut c = if ias < def.vr_kt {
                let mut c = base;
                if tail_up && ias > 0.45 * def.vr_kt {
                    // Forward stick to lift the tail, then hold about level.
                    c.elevator = (3.0 * (deg(1.0) - a.pitch()) - 0.8 * a.rates().y).clamp(-1.0, 0.6);
                } else if tail_up {
                    c.elevator = 0.0;
                }
                c
            } else {
                pilot.fly(def, &a, base, Vert::Pitch(pitch), 0.0, DT)
            };
            if rudder_pilot {
                let (k, kd) = if def.taildragger { (5.0, 2.0) } else { (2.0, 1.2) };
                let pd = (-k * a.heading() - kd * a.rates().z - 0.05 * a.pos.x).clamp(-1.0, 1.0);
                c.rudder = if a.on_ground { pd } else { c.rudder + pd * 0.5 };
            } else {
                c.rudder = 0.0;
            }
            a.step(def, &c, &env, DT);
            assert_sane(&a);
            if std::env::var("TRACE").is_ok() && ((t * 120.0).round() as i32 % 30 == 0 || a.crashed.is_some()) {
                eprintln!("  t {t:.2} ias {:.0} hdg {:.1} r {:.1} roll {:.1} pitch {:.1} x {:.1} rud {:.2} el {:.2} gnd {}", a.airspeed_kt(), a.heading().to_degrees(), a.rates().z.to_degrees(), a.roll().to_degrees(), a.pitch().to_degrees(), a.pos.x, c.rudder, c.elevator, a.on_ground);
            }
            assert!(a.crashed.is_none(), "{} take-off: {:?}", def.id, a.crashed);
            t += DT;
            max_dev = max_dev.max(wrap(a.heading()).abs());
            if !a.on_ground {
                if airborne_since.is_none() {
                    airborne_since = Some(((a.pos - start).length(), t, ias));
                }
                if t - airborne_since.unwrap().1 > 2.0 {
                    liftoff = airborne_since;
                    break;
                }
            } else {
                airborne_since = None;
            }
        }
        let (d, tl, v) = liftoff.unwrap_or_else(|| panic!("{} never lifted off", def.id));
        let drift = a.pos.x.abs();
        // Clean up and climb.
        let base = Controls { throttle: 1.0, flaps: flap, gear_down: false, ..Default::default() };
        run(&mut a, def, &env, 13.0, |a| pilot.fly(def, a, base, Vert::Pitch(pitch), 0.0, DT));
        assert!(a.crashed.is_none(), "{} climb-out: {:?}", def.id, a.crashed);
        TakeoffResult { roll_m: d, liftoff_kt: v, time_s: tl, drift_m: drift, climb_15s: a.pos.y - def.gear_height, max_heading_dev: max_dev }
    }

    #[test]
    fn v2_takeoffs() {
        // (id, flap lever, rotation pitch, ground-roll range m, lift-off speed range kt)
        let cases: [(&str, f32, f32, std::ops::Range<f32>, std::ops::Range<f32>); 5] = [
            ("peregrine", 0.0, 10.0, 230.0..420.0, 85.0..110.0),
            ("lancer", 0.0, 12.0, 300.0..800.0, 135.0..185.0),
            ("aerolink", 0.5, 8.0, 700.0..1150.0, 105.0..130.0),
            ("meridian190", 0.27, 9.0, 800.0..1400.0, 125.0..155.0),
            ("meridian320", 0.25, 9.0, 1000.0..1600.0, 135.0..165.0),
        ];
        for (id, flap, pitch, roll, vlof) in cases {
            let def = ac(id);
            let r = takeoff(&def, flap, deg(pitch), def.taildragger, true);
            eprintln!(
                "{id} take-off: roll {:.0} m in {:.1} s, lift-off {:.0} KIAS, drift {:.1} m, max hdg dev {:.1}°, gear {:.2}, +15 s height {:.0} m",
                r.roll_m,
                r.time_s,
                r.liftoff_kt,
                r.drift_m,
                r.max_heading_dev.to_degrees(),
                0.0,
                r.climb_15s
            );
            assert!(roll.contains(&r.roll_m), "{id} ground roll {}", r.roll_m);
            assert!(vlof.contains(&r.liftoff_kt), "{id} lift-off speed {}", r.liftoff_kt);
            assert!(r.drift_m < if def.taildragger { 15.0 } else { 6.0 }, "{id} drift {}", r.drift_m);
            assert!(r.climb_15s > 40.0, "{id} climb {}", r.climb_15s);
        }
    }

    // ------------------------------------------------------------------
    // Landings and crashes
    // ------------------------------------------------------------------

    /// Touch down from 0.25 m with the given sink rate and bank, then idle and brakes.
    fn land(def: &AircraftDef, sink_fpm: f32, bank: f32, pitch_override: Option<f32>) -> Aircraft {
        let env = TestEnv::default();
        let (kt, flap) = match def.category {
            // Three-point landing just above the stall.
            _ if def.taildragger => (def.vs0_kt * 1.08, 1.0),
            Category::Trainer => (60.0, 1.0),
            Category::Glider => (48.0, 0.5),
            _ => (def.vref_kt - 5.0, 1.0),
        };
        let (mut a, base) = flying(def, Vec3::new(0.0, def.gear_height + 0.5, 0.0), 0.0, kt * KT, flap);
        let tp = def.trim_point_at(kt * KT, flap, 0.0, true);
        // Concorde: ~14° alpha on the approach, but she touches down at ~10.5°
        // (ground effect carries her; the nozzles reach the runway at ~13.5°).
        let pitch = pitch_override.unwrap_or(if def.taildragger {
            def.rest_pitch() - deg(0.5)
        } else if def.id == "concorde" {
            deg(10.5)
        } else {
            tp.pitch + deg(2.0)
        });
        a.rot = attitude(0.0, pitch, bank);
        a.vel = Vec3::new(0.0, -sink_fpm * FPM, -kt * KT);
        // Start with the low-side wheels (not the CG) 0.25 m above the runway:
        // wide-track gear in a bank would otherwise begin inside the ground.
        let track = def.gear.iter().map(|g| g.pos.x.abs()).fold(0.0f32, f32::max);
        a.pos.y = def.gear_height + 0.25 + track * bank.sin().abs();
        let mut on_ground_time = 0.0;
        let tail = def.taildragger;
        let mut tt = 0.0f32;
        run(&mut a, def, &env, 60.0, |a| {
            tt += DT;
            if std::env::var("TRACE").map_or(false, |v| v == def.id) && (tt * 120.0).round() as i32 % 15 == 0 {
                eprintln!("  t {tt:.2} ias {:.0} hdg {:.1} r {:.1} roll {:.1} pitch {:.1} x {:.1} gnd {} vs {:.1}", a.airspeed_kt(), a.heading().to_degrees(), a.rates().z.to_degrees(), a.roll().to_degrees(), a.pitch().to_degrees(), a.pos.x, a.on_ground, a.vertical_speed());
            }
            if a.on_ground {
                on_ground_time += DT;
            }
            if on_ground_time > 0.5 {
                // Taildragger: stick fully back, moderate braking (no nose-over).
                let slow = a.airspeed() < 45.0 * KT;
                let (brakes, elevator) = if tail { (if slow { 0.8 } else { 0.3 }, if slow { 1.0 } else { 0.4 }) } else { (1.0, 0.0) };
                let rudder = if tail { -5.0 * a.heading() - 2.0 * a.rates().z } else { -2.0 * a.heading() };
                let aileron = (-2.0 * a.roll()).clamp(-1.0, 1.0);
                Controls { throttle: 0.0, brakes, elevator, aileron, rudder: rudder.clamp(-1.0, 1.0), ..base }
            } else if on_ground_time > 0.0 {
                // Wheels have touched: keep the wings level.
                let k = if def.control_law == ControlLaw::Direct { 2.0 } else { 2.0 / def.roll_rate_max };
                let kp = if def.control_law == ControlLaw::Direct { 1.0 } else { 1.0 / def.roll_rate_max };
                Controls { throttle: 0.0, aileron: (-k * a.roll() - kp * a.rates().x).clamp(-1.0, 1.0), ..base }
            } else {
                Controls { throttle: 0.0, ..base }
            }
        });
        a
    }

    #[test]
    fn landings() {
        // (Floatplanes land on water: src/flight/tests/ga_types.rs.)
        for def in fixed_wing().into_iter().filter(|d| !d.on_water()) {
            let a = land(&def, 500.0, 0.0, None);
            eprintln!("{} 500 fpm: crashed {:?}, final speed {:.2}, x {:.1}", def.name, a.crashed, a.vel.length(), a.pos.x);
            assert!(a.crashed.is_none());
            assert!(a.on_ground && a.vel.length() < 0.2, "{} still moving {}", def.id, a.vel.length());
            // The Vulcan's low, far-aft delta tips (nose-high touchdown) leave
            // only ~5° of bank at touchdown, as on the real aircraft.
            // (Concorde's slender delta likewise: tips far aft and low.)
            let bank = if def.id == "vulcan" || def.id == "concorde" { 3.5 } else { 6.0 };
            let a = land(&def, 300.0, deg(bank), None);
            assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
            let a = land(&def, 1000.0, 0.0, None);
            eprintln!("{} 1000 fpm: {:?}", def.name, a.crashed);
            assert!(a.crashed.as_deref().unwrap_or("").starts_with("Hard landing"));
            let a = land(&def, 200.0, deg(18.0), None);
            eprintln!("{} 18° bank: {:?}", def.name, a.crashed);
            let r = a.crashed.as_deref().unwrap_or("");
            assert!(r.contains("Wingtip") || r.contains("pod strike") || r.contains("Prop strike"), "{} {r}", def.id);
        }
        let k = kestrel();
        let a = land(&k, 300.0, 0.0, Some(deg(-5.0)));
        eprintln!("kestrel nose-first: {:?}", a.crashed);
        assert!(a.crashed.is_some());
    }

    #[test]
    fn gear_up_landing_and_transit() {
        for def in fixed_wing().into_iter().filter(|d| d.retractable_gear) {
            let env = TestEnv::default();
            // Transit time.
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, def.vref_kt * 1.3 * KT);
            let ctl = trimmed_controls_at(&def, def.vref_kt * 1.3 * KT, 1500.0);
            assert_eq!(a.gear_pos(), 0.0);
            assert_eq!(a.surfaces().gear, 1.0);
            let mut t_down = None;
            let mut t = 0.0;
            run(&mut a, &def, &env, 15.0, |a| {
                t += DT;
                if t_down.is_none() && a.gear_pos() >= 1.0 {
                    t_down = Some(t);
                }
                Controls { gear_down: true, ..ctl }
            });
            let t_down = t_down.expect("gear never came down");
            eprintln!("{} gear transit {:.1} s", def.id, t_down);
            assert!((5.0..=10.5).contains(&t_down));
            assert_eq!(a.surfaces().gear, 0.0);
            // Belly landing.
            let (kt, flap) = (def.vref_kt, 1.0);
            let (mut a, base) = flying(&def, Vec3::new(0.0, 3.0, 0.0), 0.0, kt * KT, flap);
            a.gear_pos = 0.0;
            a.vel.y = -1.0;
            let base = Controls { throttle: 0.0, gear_down: false, ..base };
            let mut pilot = Pilot::default();
            run(&mut a, &def, &env, 30.0, |a| pilot.fly(&def, a, base, Vert::Pitch(deg(if def.taildragger { 6.0 } else { 3.0 })), 0.0, DT));
            eprintln!("{} gear-up: {:?}", def.id, a.crashed);
            assert_eq!(a.crashed.as_deref(), Some("Gear-up landing"), "{}", def.id);
        }
    }

    #[test]
    fn gear_overspeed_breaks_it() {
        let def = ac("meridian320");
        let env = TestEnv::default();
        let speed = 385.0 * KT;
        let ctl = trimmed_controls_at(&def, speed, 3000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, speed);
        run(&mut a, &def, &env, 20.0, |_| Controls { gear_down: true, throttle: 1.0, ..ctl });
        eprintln!("A320-class gear at 385 kt: {:?}", a.dmg.events.iter().map(|e| &e.msg).collect::<Vec<_>>());
        // The gear is torn off; the aircraft flies on without it.
        assert_eq!(a.dmg.lost & crate::damage::GEAR_BITS, crate::damage::GEAR_BITS);
        assert!(a.dmg.events.iter().any(|e| e.msg.contains("landing gear")));
        assert!(a.crashed.is_none());
        // Within Vle the drag rises but nothing breaks.
        let speed = 250.0 * KT;
        let ctl = trimmed_controls_at(&def, speed, 3000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, speed);
        let mut pilot = Pilot::default();
        run(&mut a, &def, &env, 30.0, |a| pilot.fly(&def, a, Controls { gear_down: true, ..ctl }, Vert::Alt(3000.0), 0.0, DT));
        assert!(a.crashed.is_none());
        assert!(a.airspeed() < speed - 10.0 * KT, "gear drag {}", a.airspeed_kt());
    }

    #[test]
    fn ditching() {
        let k = kestrel();
        let env = TestEnv { water: vec![(Vec2::new(-500.0, -2000.0), Vec2::new(500.0, -200.0))], ..Default::default() };
        // Gentle "landing" on the water.
        let (mut a, base) = flying(&k, Vec3::new(0.0, 3.0, -300.0), 0.0, 60.0 * KT, 1.0);
        a.vel.y = -1.0;
        run(&mut a, &k, &env, 20.0, |_| Controls { throttle: 0.0, ..base });
        assert_eq!(a.crashed.as_deref(), Some("Ditched in the water"));
        // Taxiing into the water.
        let mut a = Aircraft::on_runway(&k, Vec2::new(0.0, -150.0), 0.0, &env);
        run(&mut a, &k, &env, 60.0, |_| Controls { throttle: 0.4, ..Default::default() });
        assert_eq!(a.crashed.as_deref(), Some("Ditched in the water"));
        // Flying over water is fine.
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 100.0, -300.0), 0.0, 90.0 * KT);
        let ctl = trimmed_controls(&k, 90.0 * KT);
        run(&mut a, &k, &env, 5.0, |_| ctl);
        assert!(a.crashed.is_none());
    }

    #[test]
    fn building_collisions() {
        let k = kestrel();
        // Building north of the start: x -20..20, z -420..-400, roof 40 m.
        let env = TestEnv {
            buildings: vec![(Vec2::new(-20.0, -420.0), Vec2::new(20.0, -400.0), 40.0)],
            ..Default::default()
        };
        let ctl = trimmed_controls(&k, 90.0 * KT);
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 30.0, 0.0), 0.0, 90.0 * KT);
        run(&mut a, &k, &env, 20.0, |_| ctl);
        assert_eq!(a.crashed.as_deref(), Some("Flew into a building"));
        // Above the roof: fine.
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 60.0, 0.0), 0.0, 90.0 * KT);
        run(&mut a, &k, &env, 20.0, |_| ctl);
        assert!(a.crashed.is_none());
        // CG passes 4 m beside the wall, the wingtip does not.
        let mut a = Aircraft::airborne(&k, Vec3::new(24.0, 30.0, 0.0), 0.0, 90.0 * KT);
        run(&mut a, &k, &env, 20.0, |_| ctl);
        assert_eq!(a.crashed.as_deref(), Some("Wingtip struck a building"));
        // ...and 7 m away it clears.
        let mut a = Aircraft::airborne(&k, Vec3::new(27.0, 30.0, 0.0), 0.0, 90.0 * KT);
        run(&mut a, &k, &env, 20.0, |_| ctl);
        assert!(a.crashed.is_none());
    }

    #[test]
    fn flying_into_ground() {
        let k = kestrel();
        let env = TestEnv::default();
        let ctl = trimmed_controls(&k, 100.0 * KT);
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 100.0, 0.0), 0.0, 100.0 * KT);
        a.rot = attitude(0.0, deg(-30.0), 0.0);
        a.vel = a.forward() * 100.0 * KT;
        run(&mut a, &k, &env, 30.0, |_| ctl);
        eprintln!("dive into ground: {:?}", a.crashed);
        assert!(a.crashed.is_some());
        let p = a.pos;
        // Frozen after a crash.
        a.step(&k, &ctl, &env, DT);
        assert_eq!(a.pos, p);
    }

    #[test]
    fn structural_limits() {
        let k = kestrel();
        let env = TestEnv::default();
        // Full back stick near Vne -> over-g (stall-limited below ~150 kt).
        let ctl = trimmed_controls(&k, 155.0 * KT);
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 2000.0, 0.0), 0.0, 155.0 * KT);
        run(&mut a, &k, &env, 5.0, |_| Controls { elevator: 1.0, ..ctl });
        eprintln!("yank at 155 kt: {:?} / {:?}", a.dmg.failure, a.crashed);
        // Beyond ultimate load a wing comes off (the aircraft then falls out of control).
        assert!(a.dmg.failure.as_deref().unwrap_or("").contains("overstressed"));
        assert!(a.dmg.is_lost(crate::damage::Part::WingL) || a.dmg.is_lost(crate::damage::Part::WingR));
        // A firm 3 g pull at 110 kt is fine.
        let ctl = trimmed_controls(&k, 110.0 * KT);
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 2000.0, 0.0), 0.0, 110.0 * KT);
        let mut gmax = 0.0f32;
        run(&mut a, &k, &env, 3.0, |a| {
            gmax = gmax.max(a.g_load());
            Controls { elevator: 0.35, ..ctl }
        });
        eprintln!("0.35 elevator pull at 110 kt: max {gmax:.1} g");
        assert!(a.crashed.is_none());
        assert!(gmax > 2.0);
        // Overspeed in a vertical dive.
        let mut a = Aircraft::airborne(&k, Vec3::new(0.0, 5000.0, 0.0), 0.0, 120.0 * KT);
        a.rot = attitude(0.0, deg(-80.0), 0.0);
        a.vel = a.forward() * a.tas();
        run(&mut a, &k, &env, 60.0, |_| Controls { throttle: 1.0, trim: -0.5, ..Default::default() });
        eprintln!("vertical dive: {:?} / {:?}", a.dmg.failure, a.crashed);
        assert!(a.dmg.failure.as_deref().unwrap_or("").contains("overspeed"));
        // The crash that follows names the structural failure as its cause.
        if let Some(c) = &a.crashed {
            assert!(c.starts_with("Structural failure"), "{c}");
        }
    }

    // ------------------------------------------------------------------
    // Robustness
    // ------------------------------------------------------------------

    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
        }
    }

    #[test]
    fn random_stick_stirring() {
        for def in fixed_wing() {
            for seed in 1..=6u64 {
                let env = TestEnv { updraft: 0.5, ..Default::default() };
                let mut rng = Lcg(seed * 7919);
                let v0 = (def.vs1_kt * 1.6).max(80.0) * KT;
                let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, v0);
                let mut c = Controls::default();
                let mut crashes = 0;
                for i in 0..(120.0 / DT) as usize {
                    if i % 30 == 0 {
                        c = Controls {
                            elevator: rng.next(),
                            aileron: rng.next(),
                            rudder: rng.next(),
                            throttle: rng.next() * 0.5 + 0.5,
                            flaps: rng.next() * 0.5 + 0.5,
                            trim: rng.next(),
                            brakes: 0.0,
                            gear_down: rng.next() > 0.0,
                            spoilers: rng.next().max(0.0),
                            reverse: rng.next() > 0.8,
                            failed_engines: if rng.next() > 0.7 { 1 } else { 0 },
                            engine_throttle: None,
                        };
                        if seed % 2 == 0 {
                            // Bang-bang extremes.
                            c.elevator = c.elevator.signum();
                            c.aileron = c.aileron.signum();
                            c.rudder = c.rudder.signum();
                        }
                    }
                    a.step(&def, &c, &env, DT);
                    assert_sane(&a);
                    assert!(a.tas() < 900.0);
                    if a.crashed.is_some() {
                        crashes += 1;
                        a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, v0);
                    }
                }
                eprintln!("{} stirring seed {seed}: {crashes} structural failures, finished OK", def.name);
            }
        }
    }

    #[test]
    fn ground_stirring_and_pathological_inputs() {
        for def in fixed_wing() {
            let env = TestEnv::default();
            let mut rng = Lcg(42);
            let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.3, &env);
            let mut c = Controls::default();
            for i in 0..(120.0 / DT) as usize {
                if i % 60 == 0 {
                    c = Controls {
                        elevator: rng.next(),
                        aileron: rng.next(),
                        rudder: rng.next(),
                        throttle: rng.next() * 0.5 + 0.5,
                        flaps: 0.0,
                        trim: 0.0,
                        brakes: (rng.next() * 0.5 + 0.5).max(0.0),
                        gear_down: rng.next() > -0.5,
                        spoilers: rng.next().max(0.0),
                        reverse: rng.next() > 0.5,
                        failed_engines: (rng.next().abs() * 4.0) as u8,
                        engine_throttle: None,
                    };
                }
                a.step(&def, &c, &env, DT);
                assert_sane(&a);
                if a.crashed.is_some() {
                    a = Aircraft::on_runway(&def, Vec2::ZERO, 0.3, &env);
                }
            }
            // NaN / infinite inputs and odd dt values are ignored safely.
            let bad = Controls {
                elevator: f32::NAN,
                aileron: f32::INFINITY,
                rudder: -f32::INFINITY,
                throttle: f32::NAN,
                flaps: 7.0,
                trim: -9.0,
                brakes: f32::NAN,
                spoilers: f32::NAN,
                failed_engines: 0xff,
                ..Default::default()
            };
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1000.0, 0.0), 0.0, def.vs1_kt * 1.5 * KT);
            for _ in 0..600 {
                a.step(&def, &bad, &env, DT);
                a.step(&def, &bad, &env, 0.0);
                a.step(&def, &bad, &env, f32::NAN);
                assert_sane(&a);
            }
            // Zero airspeed, inverted, falling.
            // (Gear up: dropping out of the sky with it down would overspeed it.)
            let clean = Controls { gear_down: !def.retractable_gear, ..Default::default() };
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 1.0);
            a.gear_pos = if def.retractable_gear { 0.0 } else { 1.0 };
            a.vel = Vec3::ZERO;
            a.rot = attitude(0.0, 0.3, PI);
            run(&mut a, &def, &env, 30.0, |_| clean);
            // The light aircraft must fly themselves out of it; a heavy jet
            // dropped like this may legitimately overspeed, but stays finite.
            if matches!(def.category, Category::Trainer | Category::Glider | Category::Warbird) {
                assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
            }
            eprintln!("{} dropped inverted at zero speed: {:?}", def.id, a.crashed);
            // Tail-first.
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 30.0);
            a.vel = -a.vel;
            run(&mut a, &def, &env, 30.0, |_| clean);
        }
    }

    #[test]
    fn spin_entry_and_recovery() {
        // Aircraft with a conventional (unprotected) stall.
        for def in fixed_wing().into_iter().filter(|d| matches!(d.category, Category::Trainer | Category::Glider | Category::Warbird)) {
            let env = TestEnv::default();
            let speed = def.stall_speed(0.0) * 1.2;
            // (v1 ran this at sea-level density; 1,500 m ISA is close to that.)
            // (The aerobat's spins are fast and its hands-off pull-out from the
            // vertical slow: it starts where an aerobatic pilot would spin, high.)
            let alt0 = if def.category == Category::Warbird || def.id == "zlin" { 5000.0 } else { 1500.0 };
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt0, 0.0), 0.0, speed);
            // Clean (gear up) for the retractable.
            if def.retractable_gear {
                a.gear_pos = 0.0;
            }
            let gear = !def.retractable_gear;
            // Full back stick, full left rudder at idle.
            let mut max_rate = 0.0f32;
            run(&mut a, &def, &env, 15.0, |a| {
                max_rate = max_rate.max(a.rates().length());
                Controls { elevator: 1.0, rudder: -1.0, throttle: 0.0, gear_down: gear, ..Default::default() }
            });
            let yaw = a.turn_rate();
            eprintln!(
                "{} pro-spin 15 s: turn rate {:.0}°/s, max body rate {:.1} rad/s, alpha {:.0}°, alt lost {:.0} m",
                def.name,
                yaw.to_degrees(),
                max_rate,
                a.alpha().to_degrees(),
                alt0 - a.pos.y
            );
            assert!(a.crashed.is_none());
            assert!(max_rate < 15.0);
            run(&mut a, &def, &env, 4.0, |_| Controls { elevator: -0.3, rudder: 1.0, gear_down: gear, ..Default::default() });
            // Then hands off (a stable aircraft flies itself out of the dive). The
            // warbird's fixed aileron rigging is set for cruise power, so at idle
            // in the dive the pilot keeps its wings level.
            let mut gmax = 0.0f32;
            let level = def.category == Category::Warbird;
            run(&mut a, &def, &env, 20.0, |a| {
                gmax = gmax.max(a.g_load());
                let aileron = if level { (-2.0 * a.roll() - 0.5 * a.rates().x).clamp(-1.0, 1.0) } else { 0.0 };
                Controls { gear_down: gear, aileron, ..Default::default() }
            });
            eprintln!(
                "{} after recovery: alpha {:.0}°, turn rate {:.0}°/s, airspeed {:.0} kt, pull-out peak {:.1} g",
                def.name,
                a.alpha().to_degrees(),
                a.turn_rate().to_degrees(),
                a.airspeed() / KT,
                gmax
            );
            assert!(a.crashed.is_none(), "{:?}", a.crashed);
            assert!(a.turn_rate().abs() < deg(10.0));
            assert!(a.alpha() < def.alpha_peak(0.0));
        }
    }

    #[test]
    fn engine_and_surfaces() {
        let k = kestrel();
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&k, Vec2::ZERO, 0.0, &env);
        let idle = a.engine_rpm();
        run(&mut a, &k, &env, 3.0, |_| Controls { throttle: 1.0, brakes: 1.0, flaps: 1.0, aileron: 1.0, ..Default::default() });
        let s = a.surfaces();
        eprintln!("rpm idle {idle:.0}, static full {:.0}", a.engine_rpm());
        assert!((550.0..800.0).contains(&idle));
        assert!((2200.0..2450.0).contains(&a.engine_rpm()));
        assert!((s.aileron - 1.0).abs() < 1e-6);
        assert!(s.flaps > 0.5 && s.flaps < 0.7); // flaps take ~5 s
        assert!(s.prop_angle >= 0.0 && s.prop_angle < TAU);
        assert_eq!(s.prop_angle, s.prop_angles[0]);
        assert_eq!(s.gear, 0.0);
        assert!(a.egt(0) > 600.0 && a.fuel_flow(0) > 20.0 && a.fuel_flow(0) < 50.0, "egt {} ff {}", a.egt(0), a.fuel_flow(0));
        assert_eq!(a.engine_count(&k), 1);
        let g = swift();
        let a = Aircraft::on_runway(&g, Vec2::ZERO, 0.0, &env);
        assert_eq!(a.engine_rpm(), 0.0);
        assert_eq!(a.engine_count(&g), 0);
    }

    #[test]
    fn untrimmed_air_starts_are_benign() {
        // Air starts with default controls (trim 0, gear lever down) and a fixed
        // throttle, as the app does: should settle into flight, not dive.
        for def in fixed_wing() {
            for (kt, throttle, flap) in [(def.vs1_kt * 1.65, 0.65, 0.0), (def.vs1_kt * 1.35, 0.45, 1.0 / 3.0), (def.vs1_kt * 1.4, 0.0, 0.0)] {
                if def.has_engine && throttle == 0.0 {
                    continue; // that's an engine cut, not a benign start
                }
                let env = TestEnv::default();
                let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 600.0, 0.0), 0.7, kt * KT);
                // (The app must set the gear lever to match the spawned gear.)
                let gear_down = a.gear_pos() > 0.5;
                // Clean-config pitch trim for the start speed, as the app sets it
                // (Sim::spawn uses trimmed_controls_at); throttle / flaps left off-trim.
                let trim = def.trim_point_at(kt * KT, if def.has_engine { flap } else { 0.0 }, 600.0, gear_down).trim;
                let ctl = Controls { throttle: if def.has_engine { throttle } else { 0.0 }, flaps: if def.has_engine { flap } else { 0.0 }, gear_down, trim, ..Default::default() };
                let (mut vmin, mut vmax, mut vs_ext, mut bank) = (f32::MAX, 0.0f32, 0.0f32, 0.0f32);
                run(&mut a, &def, &env, 60.0, |a| {
                    vmin = vmin.min(a.airspeed() / KT);
                    vmax = vmax.max(a.airspeed() / KT);
                    vs_ext = vs_ext.max(a.vertical_speed().abs());
                    bank = bank.max(a.roll().abs());
                    ctl
                });
                eprintln!(
                    "{} start {kt:.0} kt thr {throttle} flap {flap:.2}: speed {vmin:.0}..{vmax:.0} kt, |vs| max {vs_ext:.1} m/s, final vs {:.1}, bank {:.1}°",
                    def.name,
                    a.vertical_speed(),
                    bank.to_degrees()
                );
                assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
                if def.category == Category::Warbird || def.id == "zlin" {
                    // 1,250 kW through one propeller with no trim: torque rolls it
                    // and the speed change pitches it. Use trimmed_controls_at().
                    // (The aerobat is built spirally neutral: it stays where it is put.)
                    continue;
                }
                // Turboprops: prop torque away from the rigging speed needs aileron trim.
                // Pistons with modelled torque / swirl (the H2b types) likewise drift
                // a little off their rigging speed.
                let bank_lim = if def.engine_kind == EngineKind::Turboprop {
                    35.0
                } else if def.torque_k > 0.0 || def.swirl_k > 0.0 {
                    12.0
                } else {
                    3.0
                };
                assert!(bank < deg(bank_lim), "{} bank {}", def.id, bank.to_degrees());
                assert!(vmin > def.vs1_kt * if flap > 0.0 { 0.9 } else { 1.0 }, "{} vmin {vmin}", def.id);
                // A fixed 65 % throttle is a big thrust excess for the jets (they
                // zoom-climb): the app should start them with trimmed_controls_at().
                let vs_lim = match def.category {
                    // The lightly loaded Vulcan has a fighter-like T/W (0.6), the
                    // Citation X 0.44.
                    _ if def.id == "vulcan" || def.id == "citationx" => 45.0,
                    Category::Airliner | Category::FastJet => 35.0,
                    _ => 9.0,
                };
                assert!(vs_ext < vs_lim, "{} vs {vs_ext}", def.id);
            }
        }
    }

    /// Start 10 % fast from trim, stick free (only a wing leveller on the
    /// ailerons so the slow spiral mode does not pollute the measurement);
    /// geometric-mean ratio of successive airspeed overshoots.
    fn phugoid_growth(def: &AircraftDef, kt: f32) -> f32 {
        let env = TestEnv::default();
        let ctl = trimmed_controls_at(def, kt * KT, 1500.0);
        let mut a = Aircraft::airborne(def, Vec3::new(0.0, 1500.0, 0.0), 0.7, kt * KT);
        let v_trim = a.tas();
        a.vel *= 1.1;
        let mut peaks: Vec<f32> = vec![];
        let mut last_peak = f32::NEG_INFINITY;
        let mut t = 0.0f32;
        let mut prev = (0.0f32, 0.0f32);
        let secs = if matches!(def.category, Category::Trainer | Category::Glider) { 120.0 } else { 200.0 };
        run(&mut a, def, &env, secs, |a| {
            t += DT;
            let v = a.tas();
            if prev.1 > prev.0 && prev.1 > v {
                // Phugoid periods are tens of seconds: ripples on one crest
                // (short-period / numerical) are the same peak — keep its maximum.
                let p = prev.1 - v_trim;
                match peaks.last_mut() {
                    Some(last) if t - last_peak < 8.0 => *last = last.max(p),
                    _ => peaks.push(p),
                }
                last_peak = t;
            }
            prev = (prev.1, v);
            let fbw = def.control_law != ControlLaw::Direct;
            let k = if fbw { 1.0 / def.roll_rate_max } else { 1.0 };
            Controls { aileron: (k * (-1.5 * a.roll() - 0.5 * a.rates().x)).clamp(-1.0, 1.0), ..ctl }
        });
        if peaks.len() < 3 {
            return f32::NAN;
        }
        (peaks[peaks.len() - 1] / peaks[1]).powf(1.0 / (peaks.len() - 2) as f32)
    }

    #[test]
    fn phugoid_is_damped() {
        let cases = [
            ("kestrel", vec![60.0f32, 90.0, 110.0]),
            ("swift", vec![45.0, 55.0, 80.0]),
            ("peregrine", vec![130.0, 200.0, 280.0]),
            ("aerolink", vec![150.0, 200.0, 240.0]),
            ("meridian190", vec![180.0, 250.0, 300.0]),
            ("hercules", vec![160.0, 200.0, 240.0]),
            ("galaxy", vec![200.0, 250.0, 300.0]),
            ("vulcan", vec![200.0, 250.0, 300.0]),
        ];
        for (id, speeds) in cases {
            let def = ac(id);
            for kt in speeds {
                let g = phugoid_growth(&def, kt);
                eprintln!("{} phugoid at {kt} kt: amplitude ratio per cycle {:.3}", def.name, if g.is_nan() { 0.0 } else { g });
                // NaN = fewer than three overshoots (heavily damped).
                // The E190 FDM's phugoid is about neutral near Vmo.
                let lim = if def.id == "meridian190" && kt > 280.0 { 1.02 } else { 0.95 };
                assert!(g.is_nan() || g < lim, "{} phugoid not damped at {kt} kt: {g}", def.name);
            }
        }
    }

    // ------------------------------------------------------------------
    // v2 aircraft: performance, systems and handling
    // ------------------------------------------------------------------

    /// Full-throttle level flight at `alt` holding altitude: settled TAS (kt).
    fn max_level_tas(def: &AircraftDef, alt: f32, start_kt: f32, secs: f32) -> (f32, f32) {
        let env = TestEnv::default();
        let mut a = Aircraft::airborne(def, Vec3::new(0.0, alt, 0.0), 0.0, start_kt * KT);
        let base = Controls { throttle: 1.0, gear_down: !def.retractable_gear, ..trimmed_controls_at(def, start_kt * KT, alt) };
        let mut pilot = Pilot::default();
        run(&mut a, def, &env, secs, |a| pilot.fly(def, a, base, Vert::Alt(alt), 0.0, DT));
        assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
        (a.tas() / KT, a.mach())
    }

    /// Steady full-throttle climb at indicated `kt` from `alt`: rate (fpm) over 20 s.
    fn climb_rate(def: &AircraftDef, alt: f32, kt: f32) -> f32 {
        let env = TestEnv::default();
        let (mut a, base) = flying(def, Vec3::new(0.0, alt, 0.0), 0.0, kt * KT, 0.0);
        let base = Controls { throttle: 1.0, ..base };
        let mut pilot = Pilot::default();
        run(&mut a, def, &env, 15.0, |a| pilot.fly(def, a, base, Vert::Speed(kt * KT), 0.0, DT));
        // Energy height, so any speed change during the measurement counts.
        let e = |a: &Aircraft| a.pos.y + a.tas() * a.tas() / (2.0 * GRAVITY);
        let h0 = e(&a);
        run(&mut a, def, &env, 10.0, |a| pilot.fly(def, a, base, Vert::Speed(kt * KT), 0.0, DT));
        assert!(a.crashed.is_none());
        (e(&a) - h0) / 10.0 / FPM
    }

    /// The performance table: targets are published-ish figures for the
    /// real-world class analogue at the simulated weight (see the report).
    #[test]
    fn v2_performance_table() {
        struct Row {
            id: &'static str,
            stall: (f32, f32),        // clean / landing config, kt IAS
            climb: (f32, f32, f32),   // (alt m, speed kt, target fpm)
            cruise: (f32, f32, f32),  // (alt m, target max level TAS kt, tolerance)
        }
        let rows = [
            Row { id: "peregrine", stall: (82.0, 70.0), climb: (0.0, 160.0, 4500.0), cruise: (6400.0, 350.0, 15.0) },
            Row { id: "lancer", stall: (122.0, 115.0), climb: (0.0, 450.0, 40000.0), cruise: (0.0, 800.0, 60.0) },
            Row { id: "aerolink", stall: (109.0, 92.0), climb: (0.0, 140.0, 1900.0), cruise: (5200.0, 285.0, 15.0) },
            // E190 / A320: the FlightGear JSBSim FDMs (E190 at 40 t, A320 at 60 t).
            Row { id: "meridian190", stall: (150.0, 107.0), climb: (0.0, 250.0, 6500.0), cruise: (10_670.0, 470.0, 15.0) },
            // (Full-thrust level speed: the A320 FDM's Mach drag rise is gentle, so it
            // passes Mmo with the thrust levers at the stop; the app's autothrottle
            // and Mmo protection keep it at M0.82.)
            Row { id: "meridian320", stall: (170.0, 118.0), climb: (0.0, 250.0, 5800.0), cruise: (10_670.0, 525.0, 20.0) },
        ];
        for r in rows {
            let def = ac(r.id);
            let s_clean = sim_stall_speed(&def, 0.0);
            let s_full = sim_stall_speed(&def, 1.0);
            let roc = climb_rate(&def, r.climb.0 + 300.0, r.climb.1);
            let (vmax, m) = max_level_tas(&def, r.cruise.0.max(300.0), cas_from_tas(r.cruise.1 * 0.9 * KT, r.cruise.0.max(300.0)) / KT, 300.0);
            eprintln!(
                "PERF {:12} stall {:5.1}/{:5.1} kt (target {}/{}), climb {:6.0} fpm at {} KIAS (target {}), max level {:5.0} KTAS M{:.2} at {} m (target {})",
                r.id, s_clean, s_full, r.stall.0, r.stall.1, roc, r.climb.1, r.climb.2, vmax, m, r.cruise.0, r.cruise.1
            );
            assert!((s_clean / r.stall.0 - 1.0).abs() < 0.06, "{} clean stall {s_clean}", r.id);
            assert!((s_full / r.stall.1 - 1.0).abs() < 0.08, "{} landing stall {s_full}", r.id);
            assert!((roc / r.climb.2 - 1.0).abs() < 0.3, "{} climb {roc}", r.id);
            if r.id != "lancer" {
                assert!((vmax - r.cruise.1).abs() < r.cruise.2 + 10.0, "{} max speed {vmax}", r.id);
            }
        }
    }

    #[test]
    fn lancer_supersonic_and_limits() {
        let def = ac("lancer");
        let env = TestEnv::default();
        // Full reheat at 11 km from Mach 1.2: accelerates past Mach 1.8.
        let a11 = isa(11_000.0);
        let v0 = cas_from_tas(1.2 * a11.speed_of_sound, 11_000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 11_000.0, 0.0), 0.0, v0);
        let base = Controls { throttle: 1.0, gear_down: false, ..Default::default() };
        let mut pilot = Pilot::default();
        let mut ab = 0.0f32;
        run(&mut a, &def, &env, 240.0, |a| {
            ab = ab.max(a.afterburner());
            pilot.fly(&def, a, base, Vert::Alt(11_000.0), 0.0, DT)
        });
        eprintln!("lancer full reheat at 11 km: M{:.2} ({:.0} KIAS), n1 {:.2}, ff {:.0} kg/h", a.mach(), a.airspeed_kt(), a.n1(0), a.fuel_flow(0));
        assert!(a.mach() > 1.8 && a.mach() < 2.2, "M{}", a.mach());
        assert!(ab > 0.95 && a.surfaces().afterburner > 0.95);
        // Dry power at 11 km stays subsonic-ish.
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 11_000.0, 0.0), 0.0, cas_from_tas(0.9 * a11.speed_of_sound, 11_000.0));
        let base = Controls { throttle: 0.9, gear_down: false, ..Default::default() };
        let mut pilot = Pilot::default();
        run(&mut a, &def, &env, 240.0, |a| pilot.fly(&def, a, base, Vert::Alt(11_000.0), 0.0, DT));
        eprintln!("lancer military power at 11 km: M{:.2}", a.mach());
        assert!(a.mach() > 0.9 && a.mach() < 1.25 && a.afterburner() == 0.0);
        // Sea level, reheat: supersonic but limited (~Mach 1.3).
        let (v, m) = max_level_tas(&def, 300.0, 600.0, 200.0);
        eprintln!("lancer reheat at 300 m: {v:.0} KTAS, M{m:.2}");
        assert!(m > 1.1 && m < 1.45);

        // g limiter: full aft stick at 450 KIAS.
        let ctl = trimmed_controls_at(&def, 450.0 * KT, 3000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 450.0 * KT);
        let mut gmax = 0.0f32;
        run(&mut a, &def, &env, 4.0, |a| {
            gmax = gmax.max(a.g_load());
            Controls { elevator: 1.0, ..ctl }
        });
        eprintln!("lancer full aft stick at 450 kt: peak {gmax:.2} g");
        assert!(a.crashed.is_none());
        assert!((8.5..9.8).contains(&gmax), "g limit {gmax}");
        // Full forward stick: about -3 g.
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 450.0 * KT);
        let mut gmin = 9.0f32;
        run(&mut a, &def, &env, 2.0, |a| {
            gmin = gmin.min(a.g_load());
            Controls { elevator: -1.0, ..ctl }
        });
        eprintln!("lancer full forward stick: {gmin:.2} g");
        assert!((-3.6..-2.5).contains(&gmin) && a.crashed.is_none());
        // AoA limiter at low speed: full aft stick, idle.
        let ctl = trimmed_controls_at(&def, 220.0 * KT, 3000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 220.0 * KT);
        let mut amax = 0.0f32;
        run(&mut a, &def, &env, 20.0, |a| {
            amax = amax.max(a.alpha());
            Controls { elevator: 1.0, throttle: 0.0, ..ctl }
        });
        eprintln!("lancer full aft stick at idle from 220 kt: max AoA {:.1}°, now {:.0} kt", amax.to_degrees(), a.airspeed_kt());
        assert!(amax < deg(27.5) && amax > deg(22.0), "AoA limit {}", amax.to_degrees());
        assert!(a.crashed.is_none());
        // Roll: full stick gives ~240°/s.
        let ctl = trimmed_controls_at(&def, 400.0 * KT, 3000.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 400.0 * KT);
        let mut pmax = 0.0f32;
        run(&mut a, &def, &env, 1.5, |a| {
            pmax = pmax.max(a.rates().x);
            Controls { aileron: 1.0, ..ctl }
        });
        eprintln!("lancer roll rate {:.0}°/s", pmax.to_degrees());
        assert!(pmax > deg(200.0));
        // Released: it stops rolling and holds the bank.
        let bank = a.roll();
        run(&mut a, &def, &env, 3.0, |_| ctl);
        assert!(a.rates().x.abs() < deg(3.0));
        eprintln!("lancer released at {:.0}°, holds {:.0}°", bank.to_degrees(), a.roll().to_degrees());
    }

    #[test]
    fn airliner_fbw_protections() {
        let def = ac("meridian320");
        let env = TestEnv::default();
        let speed = 250.0 * KT;
        let ctl = trimmed_controls_at(&def, speed, 3000.0);
        // Neutral stick in a 30° bank: flight path held (no trim needed).
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, speed);
        run(&mut a, &def, &env, 6.0, |a| Controls { aileron: if a.roll() < deg(28.0) { 0.5 } else { 0.0 }, ..ctl });
        let h0 = a.pos.y;
        run(&mut a, &def, &env, 20.0, |_| ctl);
        eprintln!("A320-class 30° bank hands-off: bank {:.1}°, alt change {:.1} m", a.roll().to_degrees(), a.pos.y - h0);
        assert!((a.roll() - deg(30.0)).abs() < deg(3.0));
        assert!((a.pos.y - h0).abs() < 25.0);
        // Bank protection: full roll input holds ~67°, released returns to 33°.
        run(&mut a, &def, &env, 10.0, |_| Controls { aileron: 1.0, ..ctl });
        let held = a.roll();
        run(&mut a, &def, &env, 15.0, |_| ctl);
        eprintln!("A320-class bank protection: held {:.0}° at full stick, {:.0}° after release", held.to_degrees(), a.roll().to_degrees());
        assert!(held < deg(70.0) && held > deg(60.0));
        assert!(a.roll() < deg(36.0));
        // Load-factor protection: full aft stick gives 2.5 g, not more.
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 300.0 * KT);
        let ctl = trimmed_controls_at(&def, 300.0 * KT, 3000.0);
        let mut gmax = 0.0f32;
        run(&mut a, &def, &env, 4.0, |a| {
            gmax = gmax.max(a.g_load());
            Controls { elevator: 1.0, ..ctl }
        });
        eprintln!("A320-class full aft stick at 300 kt: {gmax:.2} g");
        assert!((2.3..2.8).contains(&gmax));
        // Alpha protection: full aft stick at idle never stalls.
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 200.0 * KT);
        let ctl = trimmed_controls_at(&def, 200.0 * KT, 3000.0);
        let mut amax = 0.0f32;
        run(&mut a, &def, &env, 40.0, |a| {
            amax = amax.max(a.alpha());
            Controls { elevator: 1.0, throttle: 0.0, ..ctl }
        });
        eprintln!("A320-class alpha protection: max AoA {:.1}° (CL peak {:.1}°), {:.0} kt", amax.to_degrees(), def.alpha_peak(0.0).to_degrees(), a.airspeed_kt());
        assert!(amax < def.alpha_peak(0.0) - deg(1.0));
        assert!(a.crashed.is_none());
    }

    #[test]
    fn spool_up_times() {
        for (id, lo, hi) in [
            ("meridian320", 5.0, 8.5),
            ("meridian190", 5.0, 8.0),
            ("lancer", 2.5, 5.0),
            ("aerolink", 1.5, 5.0),
            ("peregrine", 0.8, 3.0),
            // C-130 T56: constant-speed props, power responds in ~2-3 s.
            ("hercules", 1.5, 5.0),
            // C-5 CF6-80C2: big fans, slow from idle (~8-10 s to 90 %).
            ("galaxy", 7.0, 11.0),
            // Vulcan Olympus 301 turbojets: ~6-8 s idle to full.
            ("vulcan", 5.0, 9.0),
        ] {
            let def = ac(id);
            let env = TestEnv::default();
            let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
            // Taildraggers run up at reduced boost (full power drags them through the brakes).
            let thr = if def.has_afterburner { 0.9 } else if def.taildragger { 0.7 } else { 1.0 };
            // Steady full thrust (brakes on).
            let mut b = a.clone();
            // Taildraggers: stick fully back during the run-up, as in the real thing.
            let stick = if def.taildragger { 1.0 } else { 0.0 };
            run(&mut b, &def, &env, 30.0, |_| Controls { throttle: thr, brakes: 1.0, elevator: stick, ..Default::default() });
            let full: f32 = (0..def.engine_count).map(|i| b.engine_thrust(i)).sum();
            let mut t = 0.0f32;
            let mut t90 = None;
            run(&mut a, &def, &env, 20.0, |a| {
                t += DT;
                let th: f32 = (0..def.engine_count).map(|i| a.engine_thrust(i)).sum();
                if t90.is_none() && th > 0.9 * full {
                    t90 = Some(t);
                }
                Controls { throttle: thr, brakes: 1.0, elevator: stick, ..Default::default() }
            });
            let t90 = t90.unwrap_or(99.0);
            eprintln!("{id}: idle → 90 % static thrust in {t90:.1} s (full {:.1} kN, n1/power {:.2})", full / 1000.0, b.n1(0));
            assert!((lo..hi).contains(&t90), "{id} spool {t90}");
        }
    }

    #[test]
    fn afterburner_and_engine_gauges() {
        let def = ac("lancer");
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        run(&mut a, &def, &env, 10.0, |_| Controls { throttle: 0.9, brakes: 1.0, ..Default::default() });
        let dry = a.engine_thrust(0);
        assert_eq!(a.afterburner(), 0.0);
        run(&mut a, &def, &env, 3.0, |_| Controls { throttle: 1.0, brakes: 1.0, ..Default::default() });
        let wet = a.engine_thrust(0);
        eprintln!("lancer static: dry {:.1} kN, wet {:.1} kN, AB {:.2}, ff {:.0} kg/h, egt {:.0} °C", dry / 1e3, wet / 1e3, a.afterburner(), a.fuel_flow(0), a.egt(0));
        assert!((dry - 76_000.0).abs() < 2_000.0);
        assert!((wet - 129_000.0).abs() < 4_000.0);
        assert!(a.afterburner() > 0.95 && a.surfaces().afterburner > 0.95);
        assert!(a.fuel_flow(0) > 15_000.0); // reheat is thirsty
        // Airliner cruise gauges at FL350.
        let def = ac("meridian320");
        let alt = 10_668.0;
        let ias = cas_from_tas(0.78 * isa(alt).speed_of_sound, alt);
        let a = Aircraft::airborne(&def, Vec3::new(0.0, alt, 0.0), 0.0, ias);
        eprintln!(
            "A320-class M0.78 FL350: N1 {:.2}, EGT {:.0} °C, fuel flow {:.0} kg/h/engine, thrust {:.1} kN/engine, {:.0} KIAS",
            a.n1(0),
            a.egt(0),
            a.fuel_flow(0),
            a.engine_thrust(0) / 1e3,
            a.airspeed_kt()
        );
        assert!((0.75..1.0).contains(&a.n1(0)));
        assert!((900.0..1600.0).contains(&a.fuel_flow(0)), "ff {}", a.fuel_flow(0));
        // Turboprop cruise.
        let def = ac("aerolink");
        let a = Aircraft::airborne(&def, Vec3::new(0.0, 5200.0, 0.0), 0.0, 200.0 * KT);
        eprintln!(
            "ATR-class 200 KIAS FL170: Np {:.0} rpm, power {:.0} kW/engine, ITT {:.0} °C, fuel flow {:.0} kg/h/engine",
            a.engine_rpm_i(1),
            a.shaft_power(1) / 1e3,
            a.egt(1),
            a.fuel_flow(1)
        );
        assert!((1150.0..1250.0).contains(&a.engine_rpm_i(1)));
        assert!((200.0..600.0).contains(&a.fuel_flow(1)));
    }

    #[test]
    fn mach_drag_rise() {
        let def = ac("meridian320");
        for (m, lo, hi) in [(0.70f32, 0.0, 0.0005), (0.78, 0.0002, 0.003), (0.82, 0.0015, 0.010), (0.90, 0.02, 0.06)] {
            let w = def.wave_drag(m);
            eprintln!("A320-class wave drag at M{m}: {w:.4}");
            assert!((lo..=hi).contains(&w), "M{m} {w}");
        }
        let l = ac("lancer");
        assert!(l.wave_drag(1.1) > 0.025 && l.wave_drag(2.0) > 0.015 && l.wave_drag(2.0) < l.wave_drag(1.1));
        assert_eq!(kestrel().wave_drag(0.5), 0.0);
    }

    /// Ground roll from touchdown to a stop at Vref, flaps full, idle, full brakes.
    fn landing_roll(def: &AircraftDef, spoilers: bool, reverse: bool) -> f32 {
        let env = TestEnv::default();
        let kt = def.vref_kt - 5.0;
        let (mut a, base) = flying(def, Vec3::new(0.0, def.gear_height + 0.5, 0.0), 0.0, kt * KT, 1.0);
        let tp = def.trim_point_at(kt * KT, 1.0, 0.0, true);
        a.rot = attitude(0.0, tp.pitch + deg(1.0), 0.0);
        a.vel = Vec3::new(0.0, -2.0, -tas_from_cas(kt * KT, 0.0));
        a.pos.y = def.gear_height + 0.2;
        let mut touch: Option<Vec3> = None;
        let mut t_ground = 0.0;
        run(&mut a, def, &env, 90.0, |a| {
            if a.on_ground {
                touch.get_or_insert(a.pos);
                t_ground += DT;
            }
            let rolling = t_ground > 0.3;
            let gs = Vec2::new(a.vel.x, a.vel.z).length();
            if std::env::var("LR_DEBUG").is_ok() && (t_ground * 20.0).fract() < 0.01 + DT * 20.0 * 0.0 {
                eprintln!("lr {} tg {:5.2} y {:5.2} pitch {:5.1} roll {:7.1} gs {:5.1} w {:?} sp {:.2} rev {:.2} alpha {:.1}", def.id, t_ground, a.pos.y, a.pitch().to_degrees(), a.roll().to_degrees(), gs / KT, a.omega, a.surfaces().spoilers, a.surfaces().reverser, a.alpha.to_degrees());
            }
            let rev = reverse && rolling && gs > 30.0 * KT;
            Controls {
                throttle: if rev { 1.0 } else { 0.0 },
                reverse: rev,
                spoilers: if spoilers { 1.0 } else { 0.0 },
                brakes: if rolling { 1.0 } else { 0.0 },
                elevator: if rolling { 0.0 } else { 0.2 },
                rudder: (-2.0 * a.heading()).clamp(-1.0, 1.0),
                ..base
            }
        });
        assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
        assert!(a.vel.length() < 0.3);
        (a.pos - touch.unwrap()).length()
    }

    #[test]
    fn spoilers_and_reversers() {
        for (id, lo, hi) in [("meridian190", 400.0, 1100.0), ("meridian320", 450.0, 1400.0), ("aerolink", 350.0, 900.0)] {
            let def = ac(id);
            let plain = landing_roll(&def, false, false);
            let sp = if def.has_spoilers { landing_roll(&def, true, false) } else { plain };
            let all = landing_roll(&def, def.has_spoilers, true);
            eprintln!("{id} landing roll: brakes only {plain:.0} m, + ground spoilers {sp:.0} m, + reverse {all:.0} m");
            assert!((lo..hi).contains(&sp), "{id} roll {sp}");
            if def.has_spoilers {
                assert!(sp < plain * 0.95, "{id} spoilers {sp} vs {plain}");
            }
            assert!(all < sp * 0.95, "{id} reversers {all} vs {sp}");
        }
        // In flight: spoilers steepen the idle descent at a given speed; the
        // reversers refuse to deploy.
        let def = ac("meridian320");
        let mut sinks = vec![];
        for sp in [0.0f32, 1.0] {
            let env = TestEnv::default();
            let (mut a, base) = flying(&def, Vec3::new(0.0, 3000.0, 0.0), 0.0, 250.0 * KT, 0.0);
            let base = Controls { throttle: 0.0, spoilers: sp, reverse: true, ..base };
            let mut pilot = Pilot::default();
            run(&mut a, &def, &env, 30.0, |a| pilot.fly(&def, a, base, Vert::Speed(250.0 * KT), 0.0, DT));
            let h0 = a.pos.y;
            run(&mut a, &def, &env, 20.0, |a| pilot.fly(&def, a, base, Vert::Speed(250.0 * KT), 0.0, DT));
            assert_eq!(a.surfaces().reverser, 0.0);
            assert!(a.engine_thrust(0) > -1000.0);
            assert!((a.surfaces().spoilers - sp * def.spoiler_flight_max).abs() < 0.01);
            sinks.push((h0 - a.pos.y) / 20.0 / FPM);
        }
        eprintln!("A320-class idle descent at 250 kt: {:.0} fpm clean, {:.0} fpm speedbrake", sinks[0], sinks[1]);
        assert!(sinks[1] > sinks[0] * 1.4);
    }

    #[test]
    fn engine_failure_controllability() {
        for (id, kt, flap) in [("aerolink", 130.0, 0.5), ("meridian190", 160.0, 0.27), ("meridian320", 165.0, 0.25)] {
            let def = ac(id);
            let env = TestEnv::default();
            // Climb-out at full power, then engine 0 (left wing, +X) fails.
            let (mut a, base) = flying(&def, Vec3::new(0.0, 300.0, 0.0), 0.0, kt * KT, flap);
            let base = Controls { throttle: 1.0, gear_down: false, ..base };
            a.gear_pos = 0.0;
            let mut pilot = Pilot::default();
            run(&mut a, &def, &env, 10.0, |a| pilot.fly(&def, a, base, Vert::Speed(kt * KT), 0.0, DT));
            // Feet off: it yaws and rolls towards the dead engine.
            let mut b = a.clone();
            let h0 = b.heading();
            run(&mut b, &def, &env, 4.0, |_| Controls { failed_engines: 1, ..base });
            let yaw_free = wrap(b.heading() - h0);
            // Proper handling: rudder against the yaw, ~3° bank towards the live engine.
            let failed = Controls { failed_engines: 1, ..base };
            let mut pilot = Pilot::default();
            let mut max_rud = 0.0f32;
            let mut max_dev = 0.0f32;
            let h0 = a.heading();
            let z0 = a.pos.y;
            let mut rud_i = 0.0f32;
            run(&mut a, &def, &env, 40.0, |a| {
                // Bank ~3° towards the live engine and rudder against the yaw
                // (on the A320 too: normal law has no engine-out rudder compensation).
                let bank = deg(3.0);
                let mut c = pilot.fly(&def, a, failed, Vert::Speed(kt * KT), bank, DT);
                let e = wrap(a.heading() - h0);
                rud_i = (rud_i - 1.5 * e * DT).clamp(-1.0, 1.0);
                c.rudder = (rud_i - 4.0 * e - 2.0 * a.rates().z).clamp(-1.0, 1.0);
                max_rud = max_rud.max(c.rudder.abs());
                max_dev = max_dev.max(wrap(a.heading() - h0).abs());
                c
            });
            let roc = a.vertical_speed() / FPM;
            eprintln!(
                "{id} engine 0 failed at {kt} kt: free yaw {:.1}° in 4 s; handled: max rudder {:.2}, max hdg dev {:.1}°, climbing {:.0} fpm, live/dead thrust {:.1}/{:.1} kN, windmill rpm {:.0}",
                yaw_free.to_degrees(),
                max_rud,
                max_dev.to_degrees(),
                roc,
                a.engine_thrust(1) / 1e3,
                a.engine_thrust(0) / 1e3,
                a.engine_rpm_i(0)
            );
            // Left engine dead → nose swings left (heading decreases). The
            // A320-class normal law coordinates and holds it by itself.
            if def.control_law == ControlLaw::Direct {
                assert!(yaw_free < deg(-3.0), "{id} free yaw {}", yaw_free.to_degrees());
                assert!(max_rud < 0.95, "{id} rudder authority {max_rud}");
            } else {
                // Normal law holds the bank; the pilot still owns the rudder.
                assert!(yaw_free.abs() < deg(6.0), "{id} FBW free yaw {}", yaw_free.to_degrees());
            }
            assert!(a.crashed.is_none());
            // The A320 FDM's rudder (Cn_dr 0.042/rad) saturates against full
            // asymmetric thrust at 165 kt: it holds the heading only roughly.
            let dev_lim = if id == "meridian320" { 25.0 } else { 12.0 };
            assert!(max_dev < deg(dev_lim), "{id} heading dev {}", max_dev.to_degrees());
            assert!(wrap(a.heading() - h0).abs() < deg(if id == "meridian320" { 20.0 } else { 3.0 }), "{id} final heading {}", wrap(a.heading() - h0).to_degrees());
            assert!(roc > 200.0, "{id} one-engine climb {roc}");
            assert!(!a.engine_running(0) && a.engine_running(1));
            assert!(a.pos.y > z0);
        }
    }

    #[test]
    fn peregrine_swings_left_without_rudder() {
        let def = ac("peregrine");
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        let mut min_hdg = 0.0f32;
        let mut t = 0.0;
        run(&mut a, &def, &env, 12.0, |a| {
            t += DT;
            min_hdg = min_hdg.min(wrap(a.heading()));
            Controls { throttle: (t * 0.4).min(1.0), ..Default::default() }
        });
        eprintln!("peregrine feet-off take-off roll: heading went to {:.1}° ({:?})", min_hdg.to_degrees(), a.crashed);
        assert!(min_hdg < deg(-15.0), "swing {}", min_hdg.to_degrees());
        // With rudder it tracks straight (see v2_takeoffs) — and needs a lot of right rudder.
        let r = takeoff(&def, 0.0, deg(10.0), true, true);
        assert!(r.max_heading_dev < deg(12.0));
    }

    #[test]
    fn peregrine_tail_rises_and_noses_over() {
        let def = ac("peregrine");
        let env = TestEnv::default();
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        let p0 = a.pitch();
        // Full power, forward stick from 40 kt: the tail comes up.
        let mut t = 0.0f32;
        let mut tail_up_kt = None;
        while t < 20.0 {
            t += DT;
            if tail_up_kt.is_none() && a.pitch() < deg(8.0) {
                tail_up_kt = Some(a.airspeed_kt());
            }
            if tail_up_kt.is_some() && a.airspeed_kt() > 75.0 {
                break;
            }
            let el = if a.airspeed_kt() > 40.0 { (3.0 * (deg(1.0) - a.pitch()) - 0.8 * a.rates().y).clamp(-1.0, 0.3) } else { 0.0 };
            let rud = (-5.0 * a.heading() - 2.0 * a.rates().z - 0.05 * a.pos.x).clamp(-1.0, 1.0);
            let c = Controls { throttle: 1.0, elevator: el, rudder: rud, ..Default::default() };
            a.step(&def, &c, &env, DT);
            assert!(a.crashed.is_none(), "{:?}", a.crashed);
        }
        let v = tail_up_kt.expect("tail never came up");
        eprintln!("peregrine: three-point {:.1}°, tail off the ground (<8°) at {v:.0} kt, pitch now {:.1}° at {:.0} kt", p0.to_degrees(), a.pitch().to_degrees(), a.airspeed_kt());
        assert!((40.0..80.0).contains(&v));
        // Now stamp on the brakes with the tail up at speed: nose-over.
        run(&mut a, &def, &env, 5.0, |_| Controls { throttle: 0.0, brakes: 1.0, elevator: -0.3, ..Default::default() });
        eprintln!("peregrine heavy braking tail-up: {:?}", a.crashed);
        assert_eq!(a.crashed.as_deref(), Some("Prop strike"));
        // Gentle braking in the three-point attitude with the stick back is fine.
        let mut a = Aircraft::on_runway(&def, Vec2::ZERO, 0.0, &env);
        run(&mut a, &def, &env, 5.0, |a| Controls { throttle: 0.5, rudder: (-5.0 * a.heading() - 2.0 * a.rates().z).clamp(-1.0, 1.0), elevator: 1.0, ..Default::default() });
        let v = a.airspeed_kt();
        eprintln!("  before braking: {:?} pitch {:.1}", a.crashed, a.pitch().to_degrees());
        run(&mut a, &def, &env, 15.0, |_| Controls { throttle: 0.0, brakes: 0.5, elevator: 1.0, ..Default::default() });
        eprintln!("peregrine braking from {v:.0} kt three-point: {:?}, {:.1} m/s", a.crashed, a.vel.length());
        assert!(a.crashed.is_none());
    }

    #[test]
    fn torque_and_p_factor_in_flight() {
        // Full power at low speed in the warbird needs right rudder and left... er,
        // right aileron to fly straight: release them and it yaws/rolls left.
        let def = ac("peregrine");
        let env = TestEnv::default();
        let ctl = trimmed_controls_at(&def, 120.0 * KT, 1500.0);
        let mut a = Aircraft::airborne(&def, Vec3::new(0.0, 1500.0, 0.0), 0.0, 120.0 * KT);
        run(&mut a, &def, &env, 5.0, |_| Controls { throttle: 1.0, gear_down: false, ..ctl });
        eprintln!("peregrine full power at 120 kt, hands off 5 s: heading {:.1}°, bank {:.1}°", a.heading().to_degrees(), a.roll().to_degrees());
        assert!(a.heading() < deg(-2.0) && a.roll() < deg(-3.0));
        // The rigging cancels it at the cruise point (see hands_off_trimmed_stability).
    }

    // ------------------------------------------------------------------
    // v3 heavies: C-130H, C-5M, Vulcan B.2 against published figures
    // ------------------------------------------------------------------

    /// One-engine-out (engine 0, left outboard) climb-out handled with rudder
    /// and ~3° of bank: (free yaw in 4 s, max rudder, max heading deviation, climb fpm).
    fn engine_out(def: &AircraftDef, kt: f32, flap: f32) -> (f32, f32, f32, f32) {
        let env = TestEnv::default();
        let (mut a, base) = flying(def, Vec3::new(0.0, 300.0, 0.0), 0.0, kt * KT, flap);
        let base = Controls { throttle: 1.0, gear_down: false, ..base };
        a.gear_pos = 0.0;
        let mut pilot = Pilot::default();
        run(&mut a, def, &env, 10.0, |a| pilot.fly(def, a, base, Vert::Speed(kt * KT), 0.0, DT));
        if std::env::var("HEAVY_REPORT").is_ok() {
            eprintln!("  {} all engines at {kt} kt: {:.0} fpm, thrust {:.0} kN, pitch {:.1} alpha {:.1}", def.id, a.vertical_speed() / FPM, (0..def.engine_count).map(|i| a.engine_thrust(i)).sum::<f32>() / 1e3, a.pitch().to_degrees(), a.alpha().to_degrees());
        }
        let mut b = a.clone();
        let h0 = b.heading();
        run(&mut b, def, &env, 4.0, |_| Controls { failed_engines: 1, ..base });
        let yaw_free = wrap(b.heading() - h0);
        let failed = Controls { failed_engines: 1, ..base };
        let mut pilot = Pilot::default();
        let (mut max_rud, mut max_dev, mut rud_i) = (0.0f32, 0.0f32, 0.0f32);
        let h0 = a.heading();
        let mut tt = 0.0f32;
        let mut y20 = 0.0f32;
        run(&mut a, def, &env, 40.0, |a| {
            tt += DT;
            if (tt - 20.0).abs() < DT * 0.5 {
                y20 = a.pos.y;
            }
            let mut c = pilot.fly(def, a, failed, Vert::Speed(kt * KT), deg(3.0), DT);
            let e = wrap(a.heading() - h0);
            rud_i = (rud_i - 1.5 * e * DT).clamp(-1.0, 1.0);
            c.rudder = (rud_i - 4.0 * e - 2.0 * a.rates().z).clamp(-1.0, 1.0);
            max_rud = max_rud.max(c.rudder.abs());
            max_dev = max_dev.max(wrap(a.heading() - h0).abs());
            if std::env::var("EO_TRACE").is_ok_and(|v| v == def.id) && (tt * 0.5).fract() < DT * 0.5 {
                eprintln!("   eo t {:.1} vs {:.0} ias {:.0} pitch {:.1} roll {:.1} el {:.2} ail {:.2} rud {:.2} g {:.2}", tt, a.vertical_speed() / FPM, a.airspeed_kt(), a.pitch().to_degrees(), a.roll().to_degrees(), c.elevator, c.aileron, c.rudder, a.g_load());
            }
            c
        });
        assert!(a.crashed.is_none(), "{} engine out {:?}", def.id, a.crashed);
        if std::env::var("HEAVY_REPORT").is_ok() {
            eprintln!(
                "  {} engine out: thrust {:?} kN, ias {:.0}, pitch {:.1}, alpha {:.1}, beta {:.1}, gear {:.1}, flaps {:.2}",
                def.id,
                (0..def.engine_count).map(|i| (a.engine_thrust(i) / 1e3).round()).collect::<Vec<_>>(),
                a.airspeed_kt(),
                a.pitch().to_degrees(),
                a.alpha().to_degrees(),
                a.beta().to_degrees(),
                a.gear_pos(),
                a.surfaces().flaps
            );
        }
        // Mean climb over the last 20 s (the test pilot's speed hold is slow on the heavies).
        (yaw_free, max_rud, max_dev, (a.pos.y - y20) / 20.0 / FPM)
    }

    /// Published figures (sources: type fact sheets / flight manuals as summarised
    /// in each `assets/aircraft/<id>/CREDITS.md`), at the simulated weights:
    /// C-130H 60 t, C-5M 280 t, Vulcan B.2 60 t. Stall / minimum speeds are
    /// power-off, 1 g, scaled to the simulated weight.
    #[test]
    fn v3_heavies_published_numbers() {
        struct Row {
            id: &'static str,
            /// (clean, landing config) 1-g stall / minimum speed, kt IAS.
            stall: (f32, f32),
            /// Take-off: flap lever, rotation pitch°, ground-roll range m, lift-off kt range.
            to: (f32, f32, std::ops::Range<f32>, std::ops::Range<f32>),
            /// (alt m, KIAS, target fpm): full-power climb.
            climb: (f32, f32, f32),
            /// (alt m, target max level KTAS, tolerance kt).
            vmax: (f32, f32, f32),
            /// Landing roll (brakes + spoilers if fitted + reverse if fitted), m.
            roll: std::ops::Range<f32>,
            /// Engine-out check: (KIAS, flap lever).
            eo: (f32, f32),
        }
        let rows = [
            // C-130H: stall ~129 / 104 KIAS at 70 t (flaps up / 100 %) → 119 / 96 at 60 t;
            // take-off 1,093 m over 50 ft at 70 t (≈ 600 m ground roll at 60 t, flaps 50 %);
            // climb 1,830 fpm at MTOW (≈ 2,300 at 60 t); 320 KTAS at 20,000 ft;
            // landing roll ~ 550 m with reverse pitch.
            Row { id: "hercules", stall: (119.0, 96.0), to: (0.5, 9.0, 450.0..800.0, 105.0..128.0), climb: (0.0, 150.0, 2300.0), vmax: (6100.0, 320.0, 15.0), roll: 400.0..850.0, eo: (130.0, 0.5) },
            // C-5M: clean stall ~145 KIAS / full flap ~106 at 280 t; take-off run 1,600 m
            // (5,400 ft, published) — at 280 t, flaps 40 % (16°), Vr 145, lift-off ~155;
            // climb 2,100 fpm is the published average at MTOW: initial sea-level climb
            // at 280 t with 4 × 228 kN is ~4,000 fpm; max M0.79 (462 KTAS) at FL350;
            // landing run 1,100 m (3,600 ft) with reversers.
            Row { id: "galaxy", stall: (145.0, 106.0), to: (0.4, 9.0, 1000.0..1700.0, 140.0..170.0), climb: (0.0, 250.0, 4000.0), vmax: (10_670.0, 462.0, 20.0), roll: 600.0..1300.0, eo: (160.0, 0.4) },
            // Vulcan B.2 at 60 t (light, display-like weight): no stall break — the
            // minimum 1-g speed is at ~28° alpha (~88 kt); rotate 145 kt, unstick
            // ~160-170 kt after ~800 m; T/W 0.6 gives ~10,000 fpm at 300 KIAS (no
            // published figure — physics estimate); 561 KTAS / M0.96 max at altitude
            // (545 at 12 km); landing roll without the brake chute ~1,000 m.
            Row { id: "vulcan", stall: (88.0, 88.0), to: (0.0, 9.0, 550.0..1000.0, 145.0..178.0), climb: (0.0, 300.0, 10_000.0), vmax: (12_000.0, 545.0, 25.0), roll: 700.0..1600.0, eo: (170.0, 0.0) },
        ];
        for r in rows {
            let def = ac(r.id);
            let s_clean = sim_stall_speed(&def, 0.0);
            let s_full = sim_stall_speed(&def, 1.0);
            let t = takeoff(&def, r.to.0, deg(r.to.1), false, true);
            let roc = climb_rate(&def, r.climb.0 + 300.0, r.climb.1);
            let (vmax, m) = max_level_tas(&def, r.vmax.0, cas_from_tas(r.vmax.1 * 0.9 * KT, r.vmax.0) / KT, 300.0);
            let roll = landing_roll(&def, def.has_spoilers, def.has_reverser);
            let (yaw_free, max_rud, max_dev, eo_roc) = engine_out(&def, r.eo.0, r.eo.1);
            eprintln!(
                "HEAVY {:9} stall {:5.1}/{:5.1} kt (target {}/{}) | T/O roll {:5.0} m lift-off {:4.0} kt | climb {:5.0} fpm (target {}) | max {:4.0} KTAS M{:.2} (target {}) | landing roll {:4.0} m | eng-out yaw {:5.1}° rud {:.2} dev {:.1}° climb {:5.0} fpm",
                r.id, s_clean, s_full, r.stall.0, r.stall.1, t.roll_m, t.liftoff_kt, roc, r.climb.2, vmax, m, r.vmax.1, roll, yaw_free.to_degrees(), max_rud, max_dev.to_degrees(), eo_roc
            );
            if std::env::var("HEAVY_REPORT").is_ok() {
                continue;
            }
            assert!((s_clean / r.stall.0 - 1.0).abs() < 0.07, "{} clean stall {s_clean}", r.id);
            assert!((s_full / r.stall.1 - 1.0).abs() < 0.08, "{} landing stall {s_full}", r.id);
            assert!(r.to.2.contains(&t.roll_m), "{} ground roll {}", r.id, t.roll_m);
            assert!(r.to.3.contains(&t.liftoff_kt), "{} lift-off {}", r.id, t.liftoff_kt);
            assert!(t.climb_15s > 40.0, "{} climb-out {}", r.id, t.climb_15s);
            assert!((roc / r.climb.2 - 1.0).abs() < 0.3, "{} climb {roc}", r.id);
            assert!((vmax - r.vmax.1).abs() < r.vmax.2 + 10.0, "{} max speed {vmax}", r.id);
            assert!(r.roll.contains(&roll), "{} landing roll {roll}", r.id);
            // Engine 0 dead: nose swings towards it; rudder holds it with margin; still climbs.
            assert!(yaw_free < deg(-1.0), "{} free yaw {}", r.id, yaw_free.to_degrees());
            assert!(max_rud < 0.95, "{} rudder {max_rud}", r.id);
            assert!(max_dev < deg(10.0), "{} heading dev {}", r.id, max_dev.to_degrees());
            assert!(eo_roc > 300.0, "{} 3-engine climb {eo_roc}", r.id);
        }
    }

    /// `cargo test --release heavy_probe -- --ignored --nocapture`: trim table
    /// of the v3 heavies (approach, climb, cruise Mach sweep).
    #[test]
    #[ignore]
    fn heavy_probe() {
        if let Ok(id) = std::env::var("PROBE_LEVEL") {
            let def = ac(&id);
            let alt: f32 = std::env::var("PROBE_ALT").ok().and_then(|v| v.parse().ok()).unwrap_or(10_670.0);
            let kt: f32 = std::env::var("PROBE_KT").ok().and_then(|v| v.parse().ok()).unwrap_or(400.0);
            let env = TestEnv::default();
            let start = cas_from_tas(kt * KT, alt) / KT;
            let mut a = Aircraft::airborne(&def, Vec3::new(0.0, alt, 0.0), 0.0, start * KT);
            let base = Controls { throttle: 1.0, gear_down: false, ..trimmed_controls_at(&def, start * KT, alt) };
            let mut pilot = Pilot::default();
            for k in 0..(120 * 120) {
                let c = pilot.fly(&def, &a, base, Vert::Alt(alt), 0.0, DT);
                if k % 600 == 0 {
                    eprintln!("t {:4.0} ktas {:5.1} M{:.3} alt {:6.0} pitch {:5.2} alpha {:5.2} el {:5.2} n1 {:.2} thrust {:.0} kN", k as f32 * DT, a.tas() / KT, a.mach(), a.pos.y, a.pitch().to_degrees(), a.alpha().to_degrees(), c.elevator, a.n1(0), (0..def.engine_count).map(|i| a.engine_thrust(i)).sum::<f32>() / 1e3);
                }
                a.step(&def, &c, &env, DT);
            }
            return;
        }
        for id in ["hercules", "galaxy", "vulcan"] {
            let def = ac(id);
            for (kt, flap, alt, gd) in [(def.vref_kt - 5.0, 1.0, 0.0, true), (def.vref_kt + 10.0, 0.5, 0.0, true), (def.vr_kt, 0.4, 0.0, true)] {
                let tp = def.trim_point_at(kt * KT, flap, alt, gd);
                eprintln!("{id} {kt:.0} kt flap {flap}: alpha {:.1} pitch {:.1} thr {:.2} elev {:.2} trim {:.2}", tp.alpha.to_degrees(), tp.pitch.to_degrees(), tp.throttle, tp.elevator, tp.trim);
            }
            let alt = if id == "hercules" { 6100.0 } else { 10_670.0 };
            for m in [0.5f32, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9] {
                let tas = m * isa(alt).speed_of_sound;
                let tp = def.trim_point_at(cas_from_tas(tas, alt), 0.0, alt, false);
                eprintln!("{id} M{m} ({:.0} KTAS) at {alt} m: alpha {:.1} thr {:.2} elev {:.2}", tas / KT, tp.alpha.to_degrees(), tp.throttle, tp.elevator);
            }
        }
    }

    /// Per-type runway minima (start-position / airport-filter rules): the
    /// STOL-ish turboprops are not held to the jet airliners' 1,800 m.
    #[test]
    fn runway_minima_per_type() {
        let need = |id: &str| crate::sim::min_runway(&ac(id));
        assert!(need("hercules") <= 1000.0 && ac("hercules").rough_field);
        assert!(need("aerolink") <= 1508.0, "the ATR fits London City"); // EGLC 09/27
        assert!(need("meridian190") <= 1508.0, "E190s operate London City");
        assert!(need("meridian320") >= 1800.0);
        assert!(need("galaxy") >= 2400.0 && need("vulcan") >= 2000.0);
        assert!(need("kestrel") < 600.0 && need("lancer") >= 1400.0);
    }

    /// Full-throttle level acceleration of an FBW type from `start_ktas` at
    /// `alt`: the law's flight-path hold with a small altitude trim on the
    /// stick. Settled (TAS kt, Mach).
    fn fbw_max_level(def: &AircraftDef, alt: f32, start_ktas: f32, secs: f32) -> (f32, f32) {
        let env = TestEnv::default();
        let ias = cas_from_tas(start_ktas * KT, alt);
        let mut a = Aircraft::airborne(def, Vec3::new(0.0, alt, 0.0), 0.0, ias);
        let base = Controls { throttle: 1.0, gear_down: false, ..trimmed_controls_at(def, ias, alt) };
        run(&mut a, def, &env, secs, |a| Controls { elevator: (0.004 * (alt - a.pos.y) - 0.03 * a.vertical_speed()).clamp(-0.3, 0.3), ..base });
        assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
        (a.tas() / KT, a.mach())
    }

    /// Landing roll of a slender delta: touchdown from a held 10.5° attitude
    /// a few knots below Vref (ground effect cushions it), nose lowered,
    /// maximum braking and reverse. Distance from touchdown to stop, m.
    fn delta_landing_roll(def: &AircraftDef) -> f32 {
        let env = TestEnv::default();
        let kt = def.vref_kt - 8.0;
        let (mut a, base) = flying(def, Vec3::new(0.0, 50.0, 0.0), 0.0, kt * KT, 1.0);
        let pitch = deg(10.5);
        a.rot = attitude(0.0, pitch, 0.0);
        a.vel = Vec3::new(0.0, -1.0, -tas_from_cas(kt * KT, 0.0));
        let low = def.gear.iter().map(|g| (a.rot * g.pos).y).fold(f32::MAX, f32::min);
        a.pos.y = -low + 0.4;
        let mut pilot = Pilot::default();
        let mut touch: Option<Vec3> = None;
        let mut t_ground = 0.0;
        run(&mut a, def, &env, 120.0, |a| {
            if a.on_ground {
                touch.get_or_insert(a.pos);
                t_ground += DT;
            }
            let gs = Vec2::new(a.vel.x, a.vel.z).length();
            let rev = t_ground > 1.0 && gs > 30.0 * KT;
            let mut c = if t_ground > 0.0 { Controls { elevator: (2.0 * (deg(-1.0) - a.pitch())).clamp(-0.3, 0.3), ..base } } else { pilot.fly(def, a, base, Vert::Pitch(pitch), 0.0, DT) };
            c.throttle = if rev { 1.0 } else { 0.0 };
            c.reverse = rev;
            c.brakes = if t_ground > 2.0 { 1.0 } else { 0.0 };
            c.rudder = (-2.0 * a.heading()).clamp(-1.0, 1.0);
            c
        });
        assert!(a.crashed.is_none(), "{} {:?}", def.id, a.crashed);
        assert!(a.vel.length() < 0.3);
        (a.pos - touch.unwrap()).length()
    }

    /// Track H2a jets against published figures (sources in each
    /// `assets/aircraft/<id>/CREDITS.md`), at the simulated weights: 777-300ER
    /// 250 t, CRJ700 29.7 t, Citation X 14 t, Concorde 118 t. `H2A_REPORT=1`
    /// prints the table without asserting.
    #[test]
    fn h2a_jets_published_numbers() {
        struct Row {
            id: &'static str,
            /// (clean, landing config) 1-g stall / minimum speed, kt IAS.
            stall: (f32, f32),
            /// Take-off: flap lever, rotation pitch°, ground-roll range m, lift-off kt range.
            to: (f32, f32, std::ops::Range<f32>, std::ops::Range<f32>),
            /// (alt m, KIAS, target fpm): full-power climb.
            climb: (f32, f32, f32),
            /// (alt m, target max level KTAS, tolerance kt).
            vmax: (f32, f32, f32),
            /// Landing roll (brakes + spoilers if fitted + reverse), m.
            roll: std::ops::Range<f32>,
            /// Engine-out check: (KIAS, flap lever).
            eo: (f32, f32),
        }
        let rows = [
            // 777-300ER at 250 t: Vref30 151 kt (FCOM table at 551,000 lb) → Vs0 ~123,
            // clean 1-g stall ~148; flaps 15, Vr ~150, lift-off ~160 after ~1,300 m;
            // T/W 0.42 gives ~5,000 fpm at 250 KIAS (physics estimate, no published
            // figure at this weight); Mmo 0.89 (512 KTAS) at FL350, max level ~M0.89;
            // landing roll with autobrake max + reverse ~1,200 m.
            Row { id: "triple7", stall: (148.0, 123.0), to: (0.5, 9.0, 1000.0..1750.0, 150.0..175.0), climb: (0.0, 250.0, 5000.0), vmax: (10_670.0, 512.0, 20.0), roll: 650.0..1300.0, eo: (170.0, 0.5) },
            // CRJ700 at 29.7 t: Vref45 ~136 kt → Vs0 ~108, clean ~139; flaps 8, Vr 135,
            // ~1,100 m ground roll (TOFL 1,564 m at MTOW); full-power climb ~4,500 fpm;
            // Mmo 0.825 → max level ~465 KTAS at FL350; landing roll ~800 m (LFL 1,536 m).
            Row { id: "crj700", stall: (139.0, 108.0), to: (0.44, 9.0, 700.0..1300.0, 128.0..155.0), climb: (0.0, 250.0, 5800.0), vmax: (10_670.0, 470.0, 20.0), roll: 550.0..1100.0, eo: (150.0, 0.18) },
            // Citation X at 14 t: Vref ~121 kt → Vs0 ~93, clean ~113; flaps 15, Vr 120,
            // ~800 m ground roll (TOFL 1,567 m at MTOW 16.2 t); climb 3,650 fpm at MTOW on
            // climb thrust → ~6,200 fpm on take-off thrust at 14 t (T/W 0.44, physics
            // estimate); Mmo 0.92 → 527 KTAS at FL370;
            // landing roll ~600 m (LFL 1,067 m).
            Row { id: "citationx", stall: (113.0, 93.0), to: (0.43, 10.0, 550.0..1100.0, 115.0..145.0), climb: (0.0, 250.0, 6200.0), vmax: (11_280.0, 527.0, 20.0), roll: 400.0..900.0, eo: (140.0, 0.43) },
            // CRJ700 take-off on flaps 20 (lever 0.44).
            // Concorde at 118 t: no stall break; the FBW's 17° alpha limit sets
            // the minimum speed (~128 kt; the wing itself flies to ~30°); reheat take-off, rotation 165 kt to 12°, unstick ~185 kt
            // (published Vr 195 / V2 220 at 185 t), ~1,100 m at this weight;
            // climb at 400 KIAS with reheat ~16,000 fpm (physics estimate: T/W
            // 0.53); max level with full reheat at FL550 ~M2.2 (1,250 KTAS; the
            // record is M2.23, Mmo 2.04 in service); landing roll with reverse ~1,300 m
            // (LFL 2,220 m at MLW),
            // flown the Concorde way (10.5° attitude through the flare, see below).
            Row { id: "concorde", stall: (128.0, 128.0), to: (0.68, 12.0, 800.0..1500.0, 170.0..205.0), climb: (0.0, 400.0, 16_000.0), vmax: (16_760.0, 1_250.0, 60.0), roll: 1000.0..2000.0, eo: (230.0, 0.0) },
        ];
        let only = std::env::var("H2A_ONLY").ok();
        for r in rows {
            if only.as_deref().is_some_and(|o| o != r.id) {
                continue;
            }
            let def = ac(r.id);
            if std::env::var("H2A_REPORT").is_ok() {
                eprintln!(
                    "  {}: analytic stall {:.0}/{:.0} kt, alpha usable {:.1}/{:.1}°, CLmax {:.2}/{:.2}",
                    r.id,
                    def.stall_speed(0.0) / KT,
                    def.stall_speed(1.0) / KT,
                    def.alpha_usable(0.0).to_degrees(),
                    def.alpha_usable(1.0).to_degrees(),
                    def.cl_max(0.0),
                    def.cl_max(1.0)
                );
            }
            let s_clean = sim_stall_speed(&def, 0.0);
            let s_full = sim_stall_speed(&def, 1.0);
            let t = takeoff(&def, r.to.0, deg(r.to.1), false, true);
            let roc = climb_rate(&def, r.climb.0 + 300.0, r.climb.1);
            // At Mach 2 the test pilot's attitude loop is far too slow through a
            // g-command law (0.1 g is 0.1°/s at 600 m/s): Concorde's FBW holds
            // the flight path, with a gentle altitude trim.
            let (vmax, m) = if r.id == "concorde" { fbw_max_level(&def, r.vmax.0, r.vmax.1 * 0.9, 600.0) } else { max_level_tas(&def, r.vmax.0, cas_from_tas(r.vmax.1 * 0.9 * KT, r.vmax.0) / KT, 300.0) };
            // The slender delta approaches at ~14° alpha: the generic roll test
            // (trimmed attitude + 1°, just above the runway) would start it with
            // the nozzles in the concrete. Concorde holds ~10.5° through the flare.
            let roll = if r.id == "concorde" { delta_landing_roll(&def) } else { landing_roll(&def, def.has_spoilers, def.has_reverser) };
            let (yaw_free, max_rud, max_dev, eo_roc) = engine_out(&def, r.eo.0, r.eo.1);
            eprintln!(
                "H2A {:9} stall {:5.1}/{:5.1} kt (target {}/{}) | T/O roll {:5.0} m lift-off {:4.0} kt | climb {:5.0} fpm (target {}) | max {:4.0} KTAS M{:.2} (target {}) | landing roll {:4.0} m | eng-out yaw {:5.1}° rud {:.2} dev {:.1}° climb {:5.0} fpm",
                r.id, s_clean, s_full, r.stall.0, r.stall.1, t.roll_m, t.liftoff_kt, roc, r.climb.2, vmax, m, r.vmax.1, roll, yaw_free.to_degrees(), max_rud, max_dev.to_degrees(), eo_roc
            );
            if std::env::var("H2A_REPORT").is_ok() {
                continue;
            }
            assert!((s_clean / r.stall.0 - 1.0).abs() < 0.07, "{} clean stall {s_clean}", r.id);
            assert!((s_full / r.stall.1 - 1.0).abs() < 0.08, "{} landing stall {s_full}", r.id);
            assert!(r.to.2.contains(&t.roll_m), "{} ground roll {}", r.id, t.roll_m);
            assert!(r.to.3.contains(&t.liftoff_kt), "{} lift-off {}", r.id, t.liftoff_kt);
            assert!(t.climb_15s > 40.0, "{} climb-out {}", r.id, t.climb_15s);
            assert!((roc / r.climb.2 - 1.0).abs() < 0.3, "{} climb {roc}", r.id);
            assert!((vmax - r.vmax.1).abs() < r.vmax.2 + 10.0, "{} max speed {vmax}", r.id);
            assert!(r.roll.contains(&roll), "{} landing roll {roll}", r.id);
            // FBW types (777) compensate the thrust asymmetry themselves (TAC).
            if def.control_law != ControlLaw::Airliner {
                assert!(yaw_free < deg(-1.0), "{} free yaw {}", r.id, yaw_free.to_degrees());
                assert!(max_rud < 0.95, "{} rudder {max_rud}", r.id);
            }
            assert!(max_dev < deg(10.0), "{} heading dev {}", r.id, max_dev.to_degrees());
            assert!(eo_roc > 300.0, "{} engine-out climb {eo_roc}", r.id);
        }
    }

    /// Concorde's supersonic envelope: she holds Mach 2 at FL550 on dry thrust
    /// (supercruise, throttle at the reheat gate), needs reheat for the
    /// transonic push (M0.95 → M1.7 at FL400 in about 10 min — the FDM's
    /// "Mach 0.95 to 1.7 in 10 minutes" in the climb), the trim fuel moves
    /// the CG aft on the way, and the nose-temperature limit caps her near
    /// M2.2 even with full reheat.
    #[test]
    fn concorde_supercruise_and_transonic_push() {
        let def = ac("concorde");
        let env = TestEnv::default();
        // FBW flight-path hold, small altitude trim on the stick.
        let fly = |a: &mut Aircraft, throttle: f32, alt: f32, secs: f32| {
            let base = Controls { throttle, gear_down: false, ..Default::default() };
            run(a, &def, &env, secs, |a| Controls { elevator: (0.004 * (alt - a.pos.y) - 0.03 * a.vertical_speed()).clamp(-0.3, 0.3), ..base });
            assert!(a.crashed.is_none(), "{:?}", a.crashed);
        };
        let at_mach = |m: f32, alt: f32| Aircraft::airborne(&def, Vec3::new(0.0, alt, 0.0), 0.0, cas_from_tas(m * isa(alt).speed_of_sound, alt));
        // Supercruise.
        let mut a = at_mach(2.0, 16_760.0);
        fly(&mut a, 0.9, 16_760.0, 240.0);
        eprintln!("concorde dry at FL550: M{:.3}, cg aft {:.2} m, afterburner {:.2}", a.mach(), a.cg_aft(), a.surfaces().afterburner);
        assert!(a.mach() > 1.97, "supercruise M{}", a.mach());
        assert!(a.surfaces().afterburner < 0.01);
        assert!((a.cg_aft() - 1.65).abs() < 0.05, "trim fuel {}", a.cg_aft());
        // Transonic push with reheat.
        let mut a = at_mach(0.95, 12_190.0);
        let cg0 = a.cg_aft();
        let mut t = 0.0;
        while a.mach() < 1.7 && t < 900.0 {
            fly(&mut a, 1.0, 12_190.0, 10.0);
            t += 10.0;
        }
        eprintln!("concorde M0.95 → M{:.2} at FL400 with reheat in {t:.0} s, cg aft {:.2} m", a.mach(), a.cg_aft());
        assert!(a.mach() >= 1.7 && t <= 600.0, "transonic push: M{} after {t} s", a.mach());
        assert!(a.cg_aft() > cg0 + 0.4, "trim fuel moving aft: {cg0} → {}", a.cg_aft());
        // Dry thrust alone stalls in the transonic drag hump.
        let mut a = at_mach(0.98, 13_700.0);
        fly(&mut a, 0.9, 13_700.0, 300.0);
        eprintln!("concorde dry from M0.98 at FL450: M{:.2} after 300 s", a.mach());
        assert!(a.mach() < 1.3, "dry through the transonic: M{}", a.mach());
        // Full reheat at FL550: the nose-temperature limit holds her near M2.2.
        let mut a = at_mach(2.1, 16_760.0);
        fly(&mut a, 1.0, 16_760.0, 300.0);
        eprintln!("concorde full reheat at FL550: M{:.2}", a.mach());
        assert!(a.mach() < 2.3, "temperature limit: M{}", a.mach());
    }
}
