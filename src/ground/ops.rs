//! Ground operations state kept by the simulation (`Sim::ground`): the tug
//! and pushback, chocks and ground power, taxi guidance / follow-the-greens,
//! and the pilot aids (rudder–tiller link, brake skid release). The per-frame
//! and per-step glue is `ground/sim_glue.rs` (a child module of `sim`).

use super::guide::Guide;
use super::pushback::{PushDir, Pushback};
use crate::scene::MeshData;
use glam::{Mat4, Quat, Vec2, Vec3};

#[derive(Default)]
pub struct GroundOps {
    pub push: Option<Pushback>,
    /// Pushback requested: direction, seconds until the tug starts (press again to change direction).
    pub pending: Option<(PushDir, f32)>,
    /// Chocks in / ground power connected (the ground crew).
    pub chocks: bool,
    pub gpu: bool,
    /// Seconds parked with the engines off and the parking brake set (the crew then chocks it).
    pub parked_t: f32,
    /// Taxi guidance (route on the map, follow-the-greens).
    pub guide: Option<Guide>,
    /// The rudder keys / pedals also work the tiller at taxi speed (no tiller axis bound).
    pub tiller_link: bool,
    /// Ease the brakes off a locking wheel on types without anti-skid.
    pub brake_assist: bool,
    /// A tiller axis or the tiller keys moved recently.
    pub tiller_input: f32,
    /// Advisory sequence numbers already shown.
    pub seen_note: u32,
    pub seen_push: u32,
    /// Ground height under the tug (for drawing).
    pub tug_y: f32,
    tug_mesh: Option<MeshData>,
    /// Follow-the-greens lights version last built.
    pub greens_built: u64,
    /// Airborne since the (re)start: taxi guidance goes to a stand.
    pub flown: bool,
    /// Seconds a stand start has waited for the airport layout.
    pub respawn_t: f32,
    /// The settings' default autobrake was applied to this flight.
    pub prefs_applied: bool,
    /// The ATC taxi route the taxi ribbon was last started for.
    pub ribbon_route: Option<u64>,
}

impl GroundOps {
    pub fn new() -> GroundOps {
        GroundOps { tiller_link: true, brake_assist: true, ..Default::default() }
    }

    /// Everything resets with the aircraft (restart / new spawn).
    pub fn reset(&mut self) {
        let (link, assist) = (self.tiller_link, self.brake_assist);
        *self = GroundOps { tiller_link: link, brake_assist: assist, ..Default::default() };
    }

    pub fn rebase(&mut self, delta: Vec3) {
        let d = Vec2::new(delta.x, delta.z);
        if let Some(p) = &mut self.push {
            p.start += d;
            p.tug.pos += d;
        }
        if let Some(g) = &mut self.guide {
            g.rebase(d);
        }
    }

    /// The pushback tug `p` (attached or driving away), eye-relative, into `out`.
    pub fn tug_mesh(&mut self, out: &mut MeshData, eye: Vec3, p: Option<&Pushback>) {
        let Some(p) = p else { return };
        let mesh = self.tug_mesh.get_or_insert_with(|| crate::airport::mesh::vehicle(crate::airport::mesh::Vehicle::Tug, 0));
        let pos = Vec3::new(p.tug.pos.x, self.tug_y, p.tug.pos.y);
        let m = Mat4::from_translation(pos - eye) * Mat4::from_quat(Quat::from_rotation_y(std::f32::consts::PI - p.tug.hdg));
        let mat = out.mat;
        out.append(mesh, m);
        out.mat = mat;
    }
}

/// A ground-menu request (the comms panel's Ground page, ui/comms.rs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GroundReq {
    Push(PushDir),
    PushStop,
    ParkingBrake,
    /// Chocks + ground power in / out.
    Services,
    /// Autobrake selector: next position.
    Autobrake,
    TailLock,
    TaxiGuide,
}
