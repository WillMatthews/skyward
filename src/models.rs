//! Procedural 3D aircraft models (original designs, no real-world branding).
//!
//! | id | aircraft |
//! |---|---|
//! | `kestrel` | Kestrel C1, two-seat high-wing light trainer |
//! | `swift` | Swift S2, single-seat 15 m sailplane |
//! | `peregrine` | Peregrine Mk IX, elliptical-wing V12 warbird (taildragger) |
//! | `lancer` | Lancer F2, single-engine afterburning fighter |
//! | `aerolink` | Aerolink 72, high-wing twin-turboprop regional airliner |
//! | `meridian190` | Meridian 190, ~100-seat twin-jet regional airliner |
//! | `meridian320` | Meridian 320, ~180-seat twin-jet narrow-body |
//!
//! Body frame: metres, +Z = nose, +Y = up, -X = right wing, +X = left wing,
//! origin = centre of gravity. Gear contact points, engine pivots and the
//! cockpit eye follow the shared table in `docs/V2_INTERFACES.md`.
//!
//! Moving surfaces are separate [`Part`]s whose meshes are in part-local
//! coordinates (origin at the hinge pivot). Body transform of a part:
//! `translate(pivot) * rotation(axis, deflection * max_angle)`, see
//! [`Part::transform`]. Axis signs are chosen so that:
//! * elevator +1 (stick back): trailing edge up;
//! * aileron +1 (roll right): right aileron TE up, left aileron TE down;
//! * rudder +1 (yaw right): TE to the right (-X);
//! * flaps 0..1: TE down (airliner flaps also move aft: they hinge about a
//!   low pivot in the flap-track fairing); airbrakes / spoilers 0..1: panels rise;
//! * propeller `Propeller(i)`: rotation about the thrust axis (+Z) by the
//!   accumulated prop angle of engine `i` (`max_angle` = 1). Prop vertices carry
//!   `uv = [kind, r/R]`: `kind` 0 = blade (draw at low rpm), 1 = disc (fade in
//!   at high rpm);
//! * gear `GearNose` / `GearMain` / `GearDoors` (and `GearTail` if retractable):
//!   deflection = `surfaces.gear` (0 = down, wheel contact points on the table;
//!   1 = stowed, legs and wheels inside the wells and doors closed flush);
//! * `Reverser`: 0 = stowed .. 1 = deployed; `Afterburner`: EMISSIVE flame
//!   cones (uv.y = 1), show only when lit and scale uv.x by the level;
//! * `Strobe` / `Beacon`: EMISSIVE lenses (deflection unused, `max_angle` 0)
//!   for the app to blink. Steady nav lights (red +X tip, green -X tip,
//!   white tail) are EMISSIVE in the body mesh.
//!
//! Colours are linear RGB. EMISSIVE verts carry the intensity in `uv.x`.
//!
//! Engine indices: `Propeller(0)` / reverser doors with `pivot.x > 0` belong to
//! the left engine (+X), index 1 to the right engine (-X). Deviation from the
//! shared table: the Peregrine prop pivot is at z = 3.25 (not 4.60) so the
//! airframe keeps its 9.5 m length with the tabled tail-wheel / main-wheel /
//! eye positions; y and radius match the table.

mod aerolink;
mod airkit;
mod geom;
mod lancer;
mod light;
mod meridian;
mod peregrine;
#[cfg(test)]
mod tests;

use crate::scene::MeshData;
use glam::{Mat4, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartKind {
    AileronLeft,
    AileronRight,
    Elevator,
    Rudder,
    FlapLeft,
    FlapRight,
    Airbrakes,
    /// Deflection 0..1 = `surfaces.spoilers` (several parts).
    Spoilers,
    /// Engine index; axis = thrust axis; deflection = `prop_angles[i]`.
    Propeller(u8),
    GearNose,
    GearMain,
    /// Retractable tail wheel (no model uses it yet: the Peregrine's is fixed).
    #[allow(dead_code)]
    GearTail,
    GearDoors,
    Reverser,
    Afterburner,
    Strobe,
    Beacon,
}

pub struct Part {
    pub kind: PartKind,
    /// Part-local mesh, origin at the hinge pivot.
    pub mesh: MeshData,
    /// Hinge point in body coordinates.
    pub pivot: Vec3,
    /// Unit hinge axis in body coordinates (sign encodes the deflection sense).
    pub axis: Vec3,
    /// Rotation in radians at full (±1) deflection.
    pub max_angle: f32,
}

impl Part {
    /// Body-space transform for a normalised deflection (or prop angle).
    #[allow(dead_code)]
    pub fn transform(&self, deflection: f32) -> Mat4 {
        Mat4::from_translation(self.pivot) * Mat4::from_axis_angle(self.axis, deflection * self.max_angle)
    }
}

pub struct AircraftModel {
    pub body: MeshData,
    pub parts: Vec<Part>,
    /// Cockpit eye point (body frame; x > 0 = left seat).
    pub eye: Vec3,
}

/// All model ids, in roster order.
#[allow(dead_code)]
pub const IDS: [&str; 7] = ["kestrel", "swift", "peregrine", "lancer", "aerolink", "meridian190", "meridian320"];

/// Build an aircraft model by id (see [`IDS`]); unknown ids give the Kestrel.
pub fn build(id: &str) -> AircraftModel {
    match id {
        "swift" => light::swift(),
        "peregrine" => peregrine::peregrine(),
        "lancer" => lancer::lancer(),
        "aerolink" => aerolink::aerolink(),
        "meridian190" => meridian::meridian190(),
        "meridian320" => meridian::meridian320(),
        _ => light::kestrel(),
    }
}
