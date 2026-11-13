//! Aircraft on the ground follow the airport: taxiing aircraft are pulled
//! onto the nearest taxiway (or runway) centreline that runs their way,
//! parked ones onto the stand they stopped at (and the procedural aircraft
//! parked there is hidden).
//!
//! The taxiway centrelines come from the airport layouts (`airport/`), one
//! grid-indexed network per airport in its own frame; queries map the
//! world point into that frame, so the floating origin needs no rebuild.

use crate::airport::layout::Layout;
use crate::osm::Proj;
use glam::{Vec2, vec2};
use std::collections::HashMap;
use std::sync::Arc;

/// Taxiway centreline snapping distance, m.
pub const SNAP_TAXI: f32 = 30.0;
/// A stopped aircraft this close to a stand's stop point is parked on it, m.
pub const SNAP_STAND: f32 = 45.0;
const CELL: f32 = 80.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Taxiway,
    Runway,
    Stand,
}

/// Where a ground aircraft is drawn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snap {
    /// World point (reference point for taxiways / runways; the nose-wheel
    /// stop point for stands).
    pub pos: Vec2,
    /// Heading to follow, degrees true.
    pub heading: f32,
    pub kind: Kind,
    /// Distance moved, m.
    pub moved: f32,
}

struct Net {
    layout: Arc<Layout>,
    segs: Vec<(Vec2, Vec2)>,
    grid: HashMap<(i32, i32), Vec<u32>>,
}

impl Net {
    fn new(layout: Arc<Layout>) -> Net {
        let mut segs = Vec::new();
        for (line, _) in &layout.taxiways {
            for w in line.windows(2) {
                if w[0].distance(w[1]) > 0.5 {
                    segs.push((w[0], w[1]));
                }
            }
        }
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, (a, b)) in segs.iter().enumerate() {
            let lo = a.min(*b) - Vec2::splat(SNAP_TAXI);
            let hi = a.max(*b) + Vec2::splat(SNAP_TAXI);
            for gx in (lo.x / CELL).floor() as i32..=(hi.x / CELL).floor() as i32 {
                for gz in (lo.y / CELL).floor() as i32..=(hi.y / CELL).floor() as i32 {
                    grid.entry((gx, gz)).or_default().push(i as u32);
                }
            }
        }
        Net { layout, segs, grid }
    }
}

/// The taxiway networks of the airports around.
#[derive(Default)]
pub struct GroundNet {
    nets: Vec<Net>,
}

fn closest(a: Vec2, b: Vec2, p: Vec2) -> (Vec2, f32) {
    let d = b - a;
    let t = ((p - a).dot(d) / d.length_squared().max(1e-6)).clamp(0.0, 1.0);
    (a + d * t, t)
}

fn hdg_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y).to_degrees().rem_euclid(360.0)
}

fn diff(a: f32, b: f32) -> f32 {
    ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs()
}

impl GroundNet {
    /// Keep one network per current airport layout (rebuilt when a layout changes).
    pub fn sync(&mut self, layouts: Vec<Arc<Layout>>) {
        self.nets.retain(|n| layouts.iter().any(|l| Arc::ptr_eq(l, &n.layout)));
        for l in layouts {
            if !self.nets.iter().any(|n| Arc::ptr_eq(&n.layout, &l)) {
                self.nets.push(Net::new(l));
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.nets.is_empty()
    }

    /// The stand an aircraft stopped near `p` (world) is on: (stop point, heading deg, name).
    pub fn stand(&self, proj: &Proj, p: Vec2) -> Option<(Vec2, f32, String)> {
        let mut best: Option<(f32, Vec2, f32, String)> = None;
        for n in &self.nets {
            let m = n.layout.frame.map_to(proj);
            let q = m.inverse(p);
            if q.x < n.layout.lo.x - 100.0 || q.y < n.layout.lo.y - 100.0 || q.x > n.layout.hi.x + 100.0 || q.y > n.layout.hi.y + 100.0 {
                continue;
            }
            for s in &n.layout.stands {
                let d = s.stop.distance(q);
                if d < SNAP_STAND && best.as_ref().is_none_or(|b| d < b.0) {
                    best = Some((d, m.apply(s.stop), s.hdg.to_degrees().rem_euclid(360.0), s.name.clone()));
                }
            }
        }
        best.map(|(_, p, h, n)| (p, h, n))
    }

    /// The nearest taxiway centreline point to `p` (world) running along
    /// `track` (deg; either direction), when within [`SNAP_TAXI`].
    pub fn taxiway(&self, proj: &Proj, p: Vec2, track: f32) -> Option<Snap> {
        let mut best: Option<(f32, Vec2, f32)> = None;
        for n in &self.nets {
            let m = n.layout.frame.map_to(proj);
            let q = m.inverse(p);
            let Some(list) = n.grid.get(&((q.x / CELL).floor() as i32, (q.y / CELL).floor() as i32)) else { continue };
            for &i in list {
                let (a, b) = n.segs[i as usize];
                let (c, _) = closest(a, b, q);
                let d = c.distance(q);
                if d > SNAP_TAXI {
                    continue;
                }
                let h = hdg_of(b - a);
                // Along the segment, either way (penalise misaligned ones).
                let mis = diff(h, track).min(diff(h + 180.0, track));
                if mis > 50.0 {
                    continue;
                }
                let score = d + mis * 0.4;
                if best.is_none_or(|b| score < b.0) {
                    let heading = if diff(h, track) <= 90.0 { h } else { (h + 180.0).rem_euclid(360.0) };
                    best = Some((score, m.apply(c), heading));
                }
            }
        }
        best.map(|(_, c, h)| Snap { pos: c, heading: h, kind: Kind::Taxiway, moved: c.distance(p) })
    }

    /// The nearest runway centreline point, when on the runway and aligned with it.
    pub fn runway(runways: &[crate::scene::Runway], p: Vec2, track: f32) -> Option<Snap> {
        let mut best: Option<Snap> = None;
        for r in runways {
            let (c, t) = closest(r.a, r.b, p);
            let d = c.distance(p);
            if d > r.width * 0.5 + 5.0 || t <= 0.0 || t >= 1.0 {
                continue;
            }
            let h = hdg_of(r.b - r.a);
            let heading = if diff(h, track) <= 90.0 { h } else { (h + 180.0).rem_euclid(360.0) };
            if diff(heading, track) > 25.0 {
                continue;
            }
            if best.is_none_or(|b| d < b.moved) {
                best = Some(Snap { pos: c, heading, kind: Kind::Runway, moved: d });
            }
        }
        best
    }
}

/// Snapping is applied as a smoothed offset (no lateral jumps when the
/// nearest centreline changes): returns the new offset for this frame.
pub fn smooth_offset(prev: Vec2, target: Vec2, dt: f32) -> Vec2 {
    let k = 1.0 - (-dt / 1.5).exp();
    let step = (target - prev) * k;
    // At most 4 m/s sideways.
    let max = 4.0 * dt;
    prev + if step.length() > max { step * (max / step.length()) } else { step }
}

/// The reference (CG) point for a nose-wheel stop point and heading (deg).
pub fn cg_from_stop(stop: Vec2, heading_deg: f32, nose_dist: f32) -> Vec2 {
    let h = heading_deg.to_radians();
    stop - vec2(h.sin(), -h.cos()) * nose_dist
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airport::layout::Stand;

    fn layout() -> Arc<Layout> {
        let mut l = Layout {
            ident: "TEST".into(),
            frame: Proj::new(51.47, -0.45),
            kind: crate::airports::Kind::Large,
            marks: Default::default(),
            objects: Vec::new(),
            lights: Vec::new(),
            parked: Vec::new(),
            vehicles: Vec::new(),
            socks: Vec::new(),
            routes: Vec::new(),
            stands: Vec::new(),
            holds: Vec::new(),
            taxiways: Vec::new(),
            lo: Vec2::ZERO,
            hi: Vec2::ZERO,
            tiles: Vec::new(),
        };
        // An east-west taxiway and a north-south one crossing at the origin.
        l.taxiways.push((vec![vec2(-500.0, 0.0), vec2(0.0, 0.0), vec2(500.0, 0.0)], "A".into()));
        l.taxiways.push((vec![vec2(0.0, -400.0), vec2(0.0, 400.0)], "B".into()));
        l.stands.push(Stand { name: "12".into(), stop: vec2(200.0, -80.0), hdg: 0.0, span: 65.0, contact: None });
        l.lo = vec2(-600.0, -500.0);
        l.hi = vec2(600.0, 500.0);
        Arc::new(l)
    }

    #[test]
    fn taxiway_and_stand_snapping() {
        let mut g = GroundNet::default();
        g.sync(vec![layout()]);
        // The game frame is centred elsewhere: snapping maps between frames.
        let proj = Proj::new(51.48, -0.40);
        let m = layout().frame.map_to(&proj);
        let w = |p: Vec2| m.apply(p);
        // 9 m north of taxiway A, rolling east: pulled onto the centreline.
        let s = g.taxiway(&proj, w(vec2(120.0, -9.0)), 92.0).unwrap();
        assert!((s.pos - w(vec2(120.0, 0.0))).length() < 0.5, "{:?}", s);
        assert!((s.heading - 90.0).abs() < 1.0);
        // Rolling west: same line, heading reversed.
        assert!((g.taxiway(&proj, w(vec2(120.0, -9.0)), 268.0).unwrap().heading - 270.0).abs() < 1.0);
        // Near the crossing heading north: taxiway B, not A.
        let s = g.taxiway(&proj, w(vec2(6.0, -20.0)), 2.0).unwrap();
        assert!((s.pos - w(vec2(0.0, -20.0))).length() < 0.5, "{:?}", s);
        // Too far: nothing.
        assert!(g.taxiway(&proj, w(vec2(120.0, -60.0)), 90.0).is_none());
        // Stopped 20 m from stand 12: parked on it.
        let (p, h, name) = g.stand(&proj, w(vec2(210.0, -95.0))).unwrap();
        assert_eq!(name, "12");
        assert!((p - w(vec2(200.0, -80.0))).length() < 0.5);
        assert!(h.abs() < 1e-3);
        // The CG sits behind the nose wheel.
        let cg = cg_from_stop(vec2(0.0, 0.0), 90.0, 12.0);
        assert!((cg - vec2(-12.0, 0.0)).length() < 1e-4);
    }

    #[test]
    fn runway_centreline() {
        let rw = crate::scene::Runway { a: vec2(0.0, 0.0), b: vec2(3000.0, 0.0), width: 45.0, ..Default::default() };
        let s = GroundNet::runway(std::slice::from_ref(&rw), vec2(1000.0, 8.0), 268.0).unwrap();
        assert!((s.pos - vec2(1000.0, 0.0)).length() < 1e-3);
        assert!((s.heading - 270.0).abs() < 1e-3);
        // Crossing the runway: not snapped along it.
        assert!(GroundNet::runway(std::slice::from_ref(&rw), vec2(1000.0, 8.0), 0.0).is_none());
        // The offset moves smoothly (≤ 4 m/s).
        let o = smooth_offset(Vec2::ZERO, vec2(10.0, 0.0), 0.1);
        assert!(o.x > 0.0 && o.x <= 0.4 + 1e-6);
    }
}
