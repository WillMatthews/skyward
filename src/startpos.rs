//! Start positions chosen in the departure screen: a runway threshold, a
//! final approach at 3 / 5 / 10 NM, the apron, a thermal near the field, or
//! anywhere in the air. Stored geographically (airport ident, runway end,
//! lat / lon), so they survive the floating origin and restarts.

use crate::airports;
use crate::flight::{self, Aircraft, AircraftDef, Category, Controls, EngineKind, KT};
use crate::scene::Runway;
use crate::flight::Environment;
use crate::world::World;
use glam::{vec2, vec3, Vec2};

#[derive(Clone, Debug, PartialEq)]
pub enum Spec {
    /// Lined up on a runway end ("27L").
    Runway(String),
    /// Established on final to a runway end, `nm` out on a 3° path.
    Final(String, f32),
    /// Parked on the apron near the airport reference point.
    Apron,
    /// In the air: lat, lon, altitude (m), heading (deg true).
    Air { lat: f64, lon: f64, alt_m: f32, hdg: f32 },
    /// Gliders: released near the best thermal around the field.
    Thermal,
}

impl Spec {
    pub fn encode(&self) -> String {
        match self {
            Spec::Runway(e) => format!("runway:{e}"),
            Spec::Final(e, nm) => format!("final:{e}:{nm}"),
            Spec::Apron => "apron".into(),
            Spec::Air { lat, lon, alt_m, hdg } => format!("air:{lat:.5}:{lon:.5}:{alt_m:.0}:{hdg:.0}"),
            Spec::Thermal => "thermal".into(),
        }
    }
    pub fn decode(s: &str) -> Option<Spec> {
        let f: Vec<&str> = s.split(':').collect();
        Some(match f.first()? {
            &"runway" => Spec::Runway(f.get(1)?.to_string()),
            &"final" => Spec::Final(f.get(1)?.to_string(), f.get(2)?.parse().ok()?),
            &"apron" => Spec::Apron,
            &"thermal" => Spec::Thermal,
            &"air" => Spec::Air { lat: f.get(1)?.parse().ok()?, lon: f.get(2)?.parse().ok()?, alt_m: f.get(3)?.parse().ok()?, hdg: f.get(4)?.parse().ok()? },
            _ => return None,
        })
    }
    /// Short description for the UI.
    pub fn label(&self) -> String {
        match self {
            Spec::Runway(e) if e.is_empty() => "Runway into wind".into(),
            Spec::Runway(e) => format!("Runway {e}"),
            Spec::Final(e, nm) => format!("{nm:.0} NM final, runway {e}"),
            Spec::Apron => "Apron".into(),
            Spec::Air { alt_m, hdg, .. } => format!("Airborne, {:.0} ft, heading {:03.0}°", alt_m * 3.280_84, hdg),
            Spec::Thermal => "Thermal near the field".into(),
        }
    }
    /// The matching generic start (labels, bot behaviour).
    pub fn as_start(&self) -> crate::sim::Start {
        use crate::sim::Start;
        match self {
            Spec::Runway(_) | Spec::Apron => Start::Runway,
            Spec::Final(..) => Start::Approach,
            Spec::Air { .. } => Start::City,
            Spec::Thermal => Start::Thermal,
        }
    }
    pub fn valid_for(&self, def: &AircraftDef) -> bool {
        match self {
            Spec::Runway(_) | Spec::Apron => def.has_engine,
            Spec::Thermal => !def.has_engine,
            _ => true,
        }
    }
}

/// A start at a given airport.
#[derive(Clone, Debug, PartialEq)]
pub struct CustomStart {
    pub airport: String,
    pub spec: Spec,
}

/// The runway (current frame) with end `end` at `airport`, and whether `end` is its b end.
pub fn find_runway(world: &World, airport: &str, end: &str) -> Option<(Runway, bool)> {
    let from_world = world.runways.iter().find(|r| r.airport == airport && (r.ident_a == end || r.ident_b == end));
    if let Some(r) = from_world {
        return Some((r.clone(), r.ident_b == end));
    }
    let db = world.airport_db.as_ref()?;
    let a = db.get(airport)?;
    for rw in &a.runways {
        if rw.ends[0].ident == end || rw.ends[1].ident == end {
            let r = airports::to_world(a, rw, &world.proj)?;
            let b = r.ident_b == end;
            return Some((r, b));
        }
    }
    None
}

/// Typical cruise IAS (m/s) for air starts.
pub fn cruise_ias(def: &AircraftDef) -> f32 {
    KT * match def.category {
        Category::Trainer => 100.0,
        Category::Glider => 55.0,
        Category::Warbird => 230.0,
        Category::FastJet => 350.0,
        Category::Airliner => {
            if def.engine_kind == EngineKind::Jet {
                250.0
            } else {
                200.0
            }
        }
        Category::Helicopter => 110.0,
        Category::Drone => 25.0,
    }
}

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// Approach speed (m/s) at `d` metres from the threshold.
fn approach_ias(def: &AircraftDef, d: f32) -> f32 {
    let far = d > 8000.0;
    KT * match def.category {
        Category::Airliner => def.vref_kt + if far { 35.0 } else { 8.0 },
        Category::FastJet => def.vref_kt + if far { 25.0 } else { 10.0 },
        Category::Warbird => def.vref_kt * 1.15,
        Category::Trainer | Category::Glider => def.vs1_kt * 1.45,
        Category::Helicopter | Category::Drone => def.vref_kt,
    }
}

/// Where the start puts the aircraft (current frame), before anything is loaded.
pub fn point(world: &World, cs: &CustomStart) -> Option<Vec2> {
    match &cs.spec {
        Spec::Runway(e) => find_runway(world, &cs.airport, e).map(|(r, b)| airports::threshold(&r, b).0),
        Spec::Final(e, nm) => find_runway(world, &cs.airport, e).map(|(r, b)| {
            let (thr, d) = airports::threshold(&r, b);
            thr - d * nm * 1852.0
        }),
        Spec::Air { lat, lon, .. } => Some(world.proj.to_world(*lat, *lon)),
        Spec::Apron | Spec::Thermal => {
            let a = world.airport_db.as_ref()?.get(&cs.airport)?;
            Some(world.proj.to_world(a.lat, a.lon))
        }
    }
}

/// Aircraft, controls, flap detent and runway for a custom start (None if
/// the airport / runway is unknown: the caller falls back to the generic start).
pub fn spawn(def: &AircraftDef, world: &World, cs: &CustomStart) -> Option<(Aircraft, Controls, usize, Option<Runway>)> {
    let air = |pos: glam::Vec3, hdg: f32, v: f32| {
        let ac = Aircraft::airborne(def, pos, hdg, v);
        let mut ctl = flight::trimmed_controls_at(def, v, pos.y);
        ctl.gear_down = ac.gear_pos() > 0.5;
        (ac, ctl)
    };
    let nearest_runway = |p: Vec2| -> Option<Runway> {
        world.runways.iter().filter(|r| r.airport == cs.airport).max_by(|a, b| a.length().total_cmp(&b.length())).cloned().or_else(|| world.spawn_runway(p).cloned())
    };
    match &cs.spec {
        Spec::Runway(e) => {
            let (rw, b) = find_runway(world, &cs.airport, e)?;
            let (thr, d) = airports::threshold(&rw, b);
            // Line up just past the start of the pavement (not the displaced threshold: use the full length).
            let start = if b { rw.b } else { rw.a };
            let back = def.gear.iter().map(|g| g.pos.z.abs()).fold(0.0, f32::max);
            let p = start + d * (25.0 + back);
            let _ = thr;
            Some((Aircraft::on_runway(def, p, heading_of(d), world), Controls::default(), 0, Some(rw)))
        }
        Spec::Final(e, nm) => {
            let (rw, b) = find_runway(world, &cs.airport, e)?;
            let (thr, d) = airports::threshold(&rw, b);
            let dist = nm * 1852.0;
            let p = thr - d * dist;
            // (Above the threshold: terrain.rs runway elevation.)
            let alt = dist * 3f32.to_radians().tan() + 15.0 + rw.height_at(thr);
            let v = approach_ias(def, dist);
            let (mut ac, mut ctl) = air(vec3(p.x, alt, p.y), heading_of(d), v);
            let last = def.flap_detents.len().saturating_sub(1);
            let flap = match def.category {
                Category::Airliner => {
                    if *nm <= 3.5 {
                        last
                    } else if *nm <= 6.0 {
                        (last * 2 / 3).max(1).min(last)
                    } else {
                        1.min(last)
                    }
                }
                Category::Trainer => 1.min(last),
                _ => 0,
            };
            if def.has_engine {
                ctl.throttle *= 0.75;
                ctl.flaps = def.flap_detents.get(flap).copied().unwrap_or(0.0) / def.flap_detents.last().copied().unwrap_or(1.0).max(1e-3);
                if def.retractable_gear && *nm <= 6.0 {
                    ctl.gear_down = true;
                }
            }
            // Established: the flaps and gear already where the levers are (not still travelling).
            ac.set_flaps_now(ctl.flaps);
            if ctl.gear_down {
                ac.extend_gear_now();
            }
            Some((ac, ctl, flap, Some(rw)))
        }
        Spec::Apron => {
            let db = world.airport_db.as_ref()?;
            let a = db.get(&cs.airport)?;
            let arp = world.proj.to_world(a.lat, a.lon);
            let rws: Vec<&Runway> = world.runways.iter().filter(|r| r.airport == cs.airport || seg_dist(arp, r.a, r.b) < 4000.0).collect();
            let clear = |q: Vec2| {
                rws.iter().all(|r| seg_dist(q, r.a, r.b) > r.width * 0.5 + 70.0)
                    && (0..8).all(|k| {
                        let o = vec2((k as f32 * 0.785).cos(), (k as f32 * 0.785).sin()) * (def.wingspan * 0.6 + 6.0);
                        world.obstacle_top(q + o).is_none()
                    })
                    && world.obstacle_top(q).is_none()
            };
            let mut best = None;
            'search: for ring in 0..14 {
                let rad = 60.0 + ring as f32 * 45.0;
                let n = 12 + ring * 4;
                for k in 0..n {
                    let ang = k as f32 / n as f32 * std::f32::consts::TAU;
                    let q = arp + vec2(ang.cos(), ang.sin()) * rad;
                    if clear(q) {
                        best = Some(q);
                        break 'search;
                    }
                }
            }
            let q = best?;
            let rw = nearest_runway(q);
            // Face the runway (taxi out from here).
            let hdg = match &rw {
                Some(r) => {
                    let m = (r.a + r.b) * 0.5;
                    let dd = r.b - r.a;
                    let t = ((q - r.a).dot(dd) / dd.length_squared().max(1.0)).clamp(0.0, 1.0);
                    let near = r.a + dd * t;
                    let _ = m;
                    heading_of((near - q).normalize_or(vec2(0.0, -1.0)))
                }
                None => 0.0,
            };
            Some((Aircraft::on_runway(def, q, hdg, world), Controls { brakes: 1.0, ..Controls::default() }, 0, rw))
        }
        Spec::Air { lat, lon, alt_m, hdg } => {
            let p = world.proj.to_world(*lat, *lon);
            let v = cruise_ias(def);
            let (ac, ctl) = air(vec3(p.x, alt_m.max(world.ground_height(p) + 60.0), p.y), hdg.to_radians(), v);
            Some((ac, ctl, 0, nearest_runway(p)))
        }
        Spec::Thermal => None,
    }
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn specs_round_trip() {
        for s in [Spec::Runway("27L".into()), Spec::Final("09".into(), 5.0), Spec::Apron, Spec::Thermal, Spec::Air { lat: 51.5, lon: -0.1, alt_m: 900.0, hdg: 270.0 }] {
            assert_eq!(Spec::decode(&s.encode()), Some(s.clone()));
        }
    }
}
