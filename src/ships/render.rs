//! Drawing ships: the hull on the water moving with the sea state, the
//! wake (foam drawn by the water shader on an overlay of the sea), and the
//! COLREGs navigation lights at night.

use super::models::ShipModel;
use super::sim::{Ship, dir};
use super::types::Class;
use crate::lights::LightInst;
use crate::scene::{MeshData, mat};
use crate::water::SeaState;
use glam::{Mat3, Mat4, Quat, Vec2, Vec3, vec3};

/// Hull attitude and heave from the sea state: the surface sampled at the
/// bow, stern and both sides, the response cut down for hulls long against
/// the waves (a 300 m ship barely notices a 60 m swell). Returns the
/// rotation (body → world) and the heave (m).
pub fn attitude(s: &Ship, sea: &SeaState, t: f32) -> (Quat, f32) {
    let f = dir(s.heading);
    let r = Vec2::new(-f.y, f.x);
    let (hl, hb) = (s.length * 0.4, s.beam * 0.5);
    let h = |o: Vec2| sea.height(s.pos + o, t);
    let (bow, stern, stbd, port) = (h(f * hl), h(-f * hl), h(r * hb), h(-r * hb));
    let lam = sea.wavelength.max(sea.swell_len).max(1.0);
    let resp = 1.0 / (1.0 + (s.length / lam).powi(2) * 0.6);
    let pitch = ((bow - stern) / (2.0 * hl)).clamp(-0.07, 0.07) * resp;
    // Roll: the slope across the beam, plus a slow rolling of its own on a
    // seaway (small craft more).
    let own = (t * (0.9 / s.beam.max(2.0).sqrt()) + s.seed as f32).sin() * (sea.wave_h + sea.swell_h * 0.5).min(3.0) * 0.012 * (30.0 / s.length.max(8.0)).min(3.0);
    let roll = ((stbd - port) / (2.0 * hb) * resp + own).clamp(-0.12, 0.12);
    let heave = ((bow + stern + stbd + port) * 0.25 * resp).clamp(-1.2, 1.2);
    let fwd = Vec3::new(f.x, 0.0, f.y);
    let right = Vec3::new(r.x, 0.0, r.y);
    let fwd = (fwd + Vec3::Y * pitch).normalize();
    let right = (right - Vec3::Y * roll).normalize();
    let up = right.cross(fwd).normalize();
    let right = fwd.cross(up);
    // Body: +X port, +Y up, +Z bow.
    (Quat::from_mat3(&Mat3::from_cols(-right, up, fwd)), heave)
}

/// The hull's transform (body → eye-relative world) at `p` (world, on the
/// water): the model scaled to the ship's length, centred midships.
pub fn body(m: &ShipModel, s: &Ship, p: Vec3, rot: Quat, eye: Vec3) -> Mat4 {
    let k = (s.length / m.length()).clamp(0.2, 4.0);
    Mat4::from_rotation_translation(rot, p - eye) * Mat4::from_scale(Vec3::splat(k)) * Mat4::from_translation(vec3(0.0, 0.0, -m.centre_z()))
}

// ---------------------------------------------------------------- lights

/// Luminous intensities (cd): masthead (6 NM) brighter than side / stern /
/// all-round (3 NM) as in COLREGs, a third for small craft. Scaled up from
/// the legal minimum (94 / 12 cd) to read at the renderer's exposure the way
/// the street lamps (~650 cd) do; the night glare cap keeps them tight.
const MASTHEAD_CD: f32 = 1100.0;
const SIDE_CD: f32 = 300.0;

/// Visible share (0..1) of a sectored light toward `to` (unit, world):
/// within `half` (rad) either side of `axis`, with a 2° soft cut-off.
fn sector(axis: Vec3, to: Vec3, half: f32) -> f32 {
    let a = Vec2::new(axis.x, axis.z).normalize_or_zero();
    let b = Vec2::new(to.x, to.z).normalize_or_zero();
    let ang = a.dot(b).clamp(-1.0, 1.0).acos();
    ((half - ang) / 0.035 + 0.5).clamp(0.0, 1.0)
}

/// Navigation, anchor and deck lights of a ship at night (`night` 0..1),
/// eye-relative. `p` = waterline midships (eye-relative), `rot` = attitude,
/// `top` = mast height above the waterline (m), `hb` = the hull's half beam
/// as drawn (m). Lights sit just outside the drawn hull (or above it), not
/// inside: the superstructure would hide them.
pub fn lights(s: &Ship, p: Vec3, rot: Quat, top: f32, hb: f32, night: f32, t: f32, out: &mut Vec<LightInst>) {
    if night <= 0.0 {
        return;
    }
    let big = s.length >= 50.0;
    let k = night * if big { 1.0 } else { 0.35 };
    let fwd = rot * Vec3::Z;
    let port_v = rot * Vec3::X;
    // Sidelight sectors: dead ahead to 22.5° abaft the beam (centred 56.25° off the bow).
    let half_side = 56.25f32.to_radians();
    let (sh, ch) = half_side.sin_cos();
    let port_ax = fwd * ch + port_v * sh;
    let stbd_ax = fwd * ch - port_v * sh;
    let at = |x: f32, y: f32, z: f32| p + rot * vec3(x, y, z);
    let to_eye = |q: Vec3| (-q).normalize_or_zero();
    let hl = s.length * 0.5;
    let white = [1.0, 0.95, 0.85];
    let red = [1.0, 0.04, 0.02];
    let green = [0.1, 1.0, 0.45];
    let mut put = |pos: Vec3, cd: f32, col: [f32; 3]| {
        if cd > 0.01 {
            out.push(LightInst::new(pos, cd, col, crate::lights::kind::OMNI));
        }
    };
    if s.anchored {
        // All-round white forward (higher) and aft; deck lights on big ships.
        put(at(0.0, (top * 0.4).max(3.0), hl * 0.92), MASTHEAD_CD * 0.5 * k, white);
        if big {
            put(at(0.0, top * 0.3, -hl - 0.4), MASTHEAD_CD * 0.5 * k, white);
        }
    } else if s.class == Class::Sailing {
        // Tricolour at the masthead.
        let m = at(0.0, top + 0.3, 0.0);
        let v = to_eye(m);
        put(m, SIDE_CD * k * sector(port_ax, v, half_side), red);
        put(m + rot * vec3(-0.05, 0.0, 0.0), SIDE_CD * k * sector(stbd_ax, v, half_side), green);
        put(m - rot * vec3(0.0, 0.05, 0.0), SIDE_CD * k * sector(-fwd, v, 67.5f32.to_radians()), white);
    } else {
        // Masthead light(s): 225° forward; a second, higher one aft on ships ≥ 50 m.
        let m1 = at(0.0, if big { top * 0.75 } else { top + 0.3 }, hl * 0.45);
        put(m1, MASTHEAD_CD * k * sector(fwd, to_eye(m1), 112.5f32.to_radians()), white);
        if big {
            let m2 = at(0.0, top + 0.5, -hl * 0.35);
            put(m2, MASTHEAD_CD * k * sector(fwd, to_eye(m2), 112.5f32.to_radians()), white);
        }
        // Sidelights: 112.5° from ahead to 22.5° abaft the beam.
        let (side_y, side_z) = if big { (top * 0.5, -hl * 0.3) } else { (top * 0.4, hl * 0.1) };
        let pl = at(hb + 0.4, side_y, side_z);
        let sl = at(-hb - 0.4, side_y, side_z);
        put(pl, SIDE_CD * k * sector(port_ax, to_eye(pl), half_side), red);
        put(sl, SIDE_CD * k * sector(stbd_ax, to_eye(sl), half_side), green);
        // Stern light: 135° aft.
        let st = at(0.0, top * 0.2, -hl - 0.4);
        put(st, SIDE_CD * k * sector(-fwd, to_eye(st), 67.5f32.to_radians()), white);
        if s.class == Class::Fishing {
            // Trawling: all-round green over white.
            put(at(0.0, top + 2.2, hl * 0.1), SIDE_CD * k, green);
            put(at(0.0, top + 0.6, hl * 0.1), SIDE_CD * k, white);
        }
    }
    // Deck and accommodation lighting.
    let warm = [1.0, 0.8, 0.55];
    match s.class {
        Class::Cruise | Class::Passenger => {
            // Rows of lit windows along both sides.
            let n = ((s.length / 12.0) as usize).clamp(4, 24);
            for i in 0..n {
                let z = -hl * 0.8 + s.length * 0.8 * (i as f32 + 0.5) / n as f32;
                for (x, deck) in [(hb + 0.25, 0.3), (-hb - 0.25, 0.3), (hb + 0.25, 0.42), (-hb - 0.25, 0.42)] {
                    let flick = 0.6 + 0.4 * ((i as f32 * 7.3 + s.seed as f32 + x).sin() * 0.5 + 0.5);
                    put(at(x, top * deck, z), 70.0 * night * flick, warm);
                }
            }
        }
        Class::Cargo | Class::Container | Class::Tanker | Class::Barge if big => {
            put(at(0.0, top + 1.0, -hl * 0.35), 3000.0 * night, [1.0, 0.86, 0.62]);
            put(at(hb * 0.5, top * 0.6, hl * 0.5), 1500.0 * night, [1.0, 0.86, 0.62]);
        }
        _ => {}
    }
    let _ = t;
}

// ---------------------------------------------------------------- wake

/// Wake drawn this far (m) from the eye at most.
pub const WAKE_DIST: f32 = 6000.0;

/// Push a vertex of the wake overlay: water material, foam in uv.x, the
/// overlay marker uv.y = 2 (scene.wgsl `shade_water`).
fn wv(m: &mut MeshData, p: Vec3, foam: f32) -> u32 {
    m.v_uv(p, Vec3::Y, [0.0, 0.0, 0.0], [foam.clamp(0.0, 1.0), 2.0])
}

/// A ribbon along `pts` (eye-relative, on the water) with half widths and
/// centre foam per point: three vertices across (edges without foam).
fn ribbon(m: &mut MeshData, pts: &[(Vec3, f32, f32)]) {
    if pts.len() < 2 {
        return;
    }
    let mut prev: Option<[u32; 3]> = None;
    for i in 0..pts.len() {
        let (p, w, f) = pts[i];
        let a = if i + 1 < pts.len() { pts[i + 1].0 - p } else { p - pts[i - 1].0 };
        let side = Vec3::new(-a.z, 0.0, a.x).normalize_or_zero() * w;
        let row = [wv(m, p + side, 0.0), wv(m, p, f), wv(m, p - side, 0.0)];
        if let Some(q) = prev {
            for k in 0..2 {
                m.idx.extend([q[k], row[k], row[k + 1], q[k], row[k + 1], q[k + 1]]);
            }
        }
        prev = Some(row);
    }
}

/// The wake of a ship under way: the turbulent trail astern (along its
/// recent track), the Kelvin arms from the bow (19.47° half angle) and a
/// little bow wave. `y` = the water level (eye-relative).
pub fn wake(s: &Ship, eye: Vec3, y: f32, out: &mut MeshData) {
    if s.speed < 0.7 {
        return;
    }
    let mat0 = out.mat;
    out.mat = mat::WATER;
    let y = y + 0.12;
    let f = dir(s.heading);
    let r = Vec2::new(-f.y, f.x);
    let e = Vec2::new(eye.x, eye.z);
    let at = |q: Vec2| Vec3::new(q.x - e.x, y, q.y - e.y);
    let strength = (s.speed / 7.0).clamp(0.2, 1.0);
    let hl = s.length * 0.5;
    // Trail: from the stern back along the track.
    let stern = s.pos - f * hl * 0.95;
    let max_len = (s.length * 6.0 + s.speed * 40.0).min(2500.0);
    let mut pts = vec![(at(stern), s.beam * 0.45, strength)];
    let mut acc = 0.0;
    let mut last = stern;
    for &(q, _) in s.trail.iter().rev() {
        let q = q - f * hl * 0.95;
        let d = q.distance(last);
        if d < s.length * 0.15 {
            continue;
        }
        acc += d;
        last = q;
        let k = (acc / max_len).min(1.0);
        pts.push((at(q), s.beam * (0.5 + 1.6 * k.sqrt()), strength * (1.0 - k).powf(1.6)));
        if acc >= max_len {
            break;
        }
    }
    if pts.len() < 3 {
        // No history yet: a straight trail.
        for i in 1..8 {
            let k = i as f32 / 7.0;
            pts.push((at(stern - f * max_len * 0.4 * k), s.beam * (0.5 + 1.2 * k), strength * (1.0 - k)));
        }
    }
    ribbon(out, &pts);
    // Kelvin arms and the bow wave.
    let bow = s.pos + f * hl * 0.92;
    let arm = (s.length * 2.2).clamp(25.0, 700.0);
    let (sa, ca) = (19.47f32.to_radians().sin(), 19.47f32.to_radians().cos());
    for side in [-1.0f32, 1.0] {
        let d = -f * ca + r * side * sa;
        let mut pts = Vec::new();
        for i in 0..6 {
            let k = i as f32 / 5.0;
            pts.push((at(bow + d * arm * k + r * side * s.beam * 0.4), s.beam * 0.12 + arm * 0.03 * k, strength * 0.75 * (1.0 - k).powf(1.3)));
        }
        ribbon(out, &pts);
    }
    out.mat = mat0;
}
