//! Traffic tests: lane graph extraction, car following, density, tile-edge
//! continuity, the Off setting, and a long run over real London tiles
//! (skipped without the tile cache).

use super::net::{self, RoadLine, RoadNet, class, flag};
use super::sim::Sim;
use super::{Level, Snapshot, Traffic, View};
use crate::osm::{Proj, TileId};
use glam::{Vec2, Vec3, vec2};
use std::sync::Arc;

/// A z14 tile in London (left-hand traffic) and one in Paris (right-hand).
const LONDON: TileId = TileId { z: 14, x: 8186, y: 5448 };
const PARIS: TileId = TileId { z: 14, x: 8299, y: 5636 };

fn road(c: u8, flags: u8, pts: &[Vec2]) -> RoadLine {
    RoadLine { class: c, flags, layer: 0, pts: pts.to_vec(), cut: [false; 2] }
}

fn build(id: TileId, roads: &[RoadLine]) -> RoadNet {
    let proj = Proj::tile_frame(id);
    let height = |_: Vec2| 0.0;
    let built = |_: Vec2| true;
    net::build(&net::Ctx { id, proj: &proj, height: &height, built: &built }, roads).expect("a lane graph")
}

fn view_at(eye: Vec3) -> View {
    View { eye, fwd: Vec3::NEG_Z, cos_half: -1.0, lod0_dist: 150.0, draw_dist: 2500.0, light_dist: 3000.0 }
}

/// Lanes that are not junction connectors.
fn lanes(n: &RoadNet) -> Vec<u32> {
    (0..n.lanes.len() as u32).filter(|&l| !n.lanes[l as usize].has(flag::CONNECTOR)).collect()
}

#[test]
fn lane_directions_and_one_way() {
    // An east-west two-way primary road and a north-south one-way secondary.
    let ew = road(class::PRIMARY, 0, &[vec2(100.0, 500.0), vec2(900.0, 500.0)]);
    let ns = road(class::SECONDARY, flag::ONEWAY, &[vec2(1100.0, 100.0), vec2(1100.0, 900.0)]);
    let n = build(LONDON, &[ew.clone(), ns.clone()]);
    let ls = lanes(&n);
    // Two-way: one lane each way; one-way 9 m: two lanes, same direction.
    assert_eq!(ls.len(), 4, "{} lanes", ls.len());
    let mut east = 0;
    let mut west = 0;
    let mut south = 0;
    for &l in &ls {
        let (p, d) = n.start(l);
        if d.x > 0.9 {
            east += 1;
            // Keep left: travelling east (+x), the lane is north of the centre line (z < 500).
            assert!(p.y < 500.0 - 1.0, "eastbound lane at z {}", p.y);
        } else if d.x < -0.9 {
            west += 1;
            assert!(p.y > 500.0 + 1.0, "westbound lane at z {}", p.y);
        } else {
            assert!(d.y > 0.9, "one-way lane runs against its direction: {d:?}");
            south += 1;
        }
    }
    assert_eq!((east, west, south), (1, 1, 2));
    // Right-hand traffic in Paris: eastbound south of the centre line.
    let n = build(PARIS, &[ew]);
    for l in lanes(&n) {
        let (p, d) = n.start(l);
        if d.x > 0.9 {
            assert!(p.y > 501.0);
        } else {
            assert!(p.y < 499.0);
        }
    }
    // A motorway carriageway (one-way, 18 m) has the four painted lanes.
    let n = build(LONDON, &[road(class::MOTORWAY, flag::ONEWAY, &[vec2(100.0, 300.0), vec2(1000.0, 300.0)])]);
    assert_eq!(lanes(&n).len(), 4);
    for l in lanes(&n) {
        assert!(n.start(l).1.x > 0.99);
    }
}

#[test]
fn one_way_reversed_in_the_data() {
    // OSM oneway=-1: extract_road stores it in travel order.
    let f = crate::mvt::Feature {
        id: 1,
        geom_type: crate::mvt::GeomType::Line,
        tags: [("oneway".to_string(), crate::mvt::Value::Num(-1.0))].into_iter().collect(),
        parts: vec![vec![[0.0, 0.0], [100.0, 0.0]]],
    };
    let mut out = Vec::new();
    net::extract_road(&mut out, &f, "tertiary", &|p: &[[f32; 2]]| vec![(p.iter().map(|q| vec2(q[0], q[1])).collect(), [false; 2])]);
    assert_eq!(out.len(), 1);
    assert!(out[0].flags & flag::ONEWAY != 0);
    assert_eq!(out[0].pts[0], vec2(100.0, 0.0));
    // Footways and parking aisles are not driven on.
    net::extract_road(&mut out, &f, "path", &|_| vec![(vec![Vec2::ZERO, Vec2::X], [false; 2])]);
    assert_eq!(out.len(), 1);
}

#[test]
fn junction_connectors_and_conflicts() {
    // A crossroads: two two-way roads through a shared node.
    let a = road(class::PRIMARY, 0, &[vec2(100.0, 500.0), vec2(500.0, 500.0), vec2(900.0, 500.0)]);
    let b = road(class::TERTIARY, 0, &[vec2(500.0, 100.0), vec2(500.0, 500.0), vec2(500.0, 900.0)]);
    let n = build(LONDON, &[a, b]);
    // Split into four arms of two lanes each.
    assert_eq!(lanes(&n).len(), 8);
    let conns: Vec<u32> = (0..n.lanes.len() as u32).filter(|&l| n.lanes[l as usize].has(flag::CONNECTOR) && n.start(l).0.distance(vec2(500.0, 500.0)) < 30.0).collect();
    // The far ends are dead ends: a U-turn each.
    assert_eq!((0..n.lanes.len() as u32).filter(|&l| n.lanes[l as usize].has(flag::CONNECTOR)).count(), 16);
    // Each approach: straight, left and right (no U-turns at a crossroads).
    assert_eq!(conns.len(), 12, "{} connectors", conns.len());
    // Every connector leads to exactly one lane, every approach lane has exits.
    for &c in &conns {
        assert_eq!(n.succs(c).len(), 1);
        assert!(n.lanes[c as usize].speed > 2.0 && n.lanes[c as usize].speed < 15.0);
    }
    for l in lanes(&n) {
        let (p, _) = n.end(l);
        if p.distance(vec2(500.0, 500.0)) < 30.0 {
            assert!(!n.succs(l).is_empty());
        }
    }
    // Straight-on across traffic conflicts with the crossing road's straight-on.
    assert!(conns.iter().any(|&c| n.lanes[c as usize].nconf > 0));
    // The major road goes first.
    let prio = |cl: u8| conns.iter().filter(|&&c| n.lanes[n.preds(c)[0] as usize].class() == cl).map(|&c| n.lanes[c as usize].prio).fold(f32::MAX, f32::min);
    assert!(prio(class::PRIMARY) < prio(class::TERTIARY));
}

#[test]
fn car_following_never_overlaps() {
    // A long road with a T-junction and a dead end (U-turn), busy.
    let a = road(class::TRUNK, 0, &[vec2(50.0, 400.0), vec2(700.0, 400.0), vec2(1400.0, 420.0)]);
    let b = road(class::SECONDARY, 0, &[vec2(700.0, 400.0), vec2(700.0, 900.0)]);
    let id = LONDON;
    let mut sim = Sim::new(Proj::tile_frame(id));
    sim.set_level(Level::High);
    sim.add_tile(Arc::new(build(id, &[a, b])));
    let eye = Vec3::new(700.0, 100.0, 450.0);
    let mut min_gap = f32::MAX;
    let mut moved = 0.0;
    for i in 0..3000 {
        sim.advance(i as f64 * 0.1, eye, 8.0);
        sim.check().unwrap();
        min_gap = min_gap.min(sim.min_gap());
        if i % 50 == 0 {
            let mut snap = Snapshot::default();
            sim.snapshot(i as f64 * 0.1, &view_at(eye), &mut snap);
            assert!(snap.inst.iter().all(|v| v.pos.iter().all(|x| x.is_finite())));
            moved += snap.speed.iter().sum::<f32>() / snap.speed.len().max(1) as f32;
        }
    }
    assert!(sim.vehicle_count() > 20, "{} vehicles", sim.vehicle_count());
    assert!(min_gap >= -0.01, "overlap {min_gap}");
    // Traffic keeps moving (no gridlock at the junction).
    assert!(moved / 60.0 > 3.0, "mean speed {}", moved / 60.0);
}

#[test]
fn idm_follows_and_stops() {
    let sp = super::vehicle::spec(super::vehicle::kind::SALOON);
    // Free road: accelerate; at the desired speed: ~0; closing fast on a close car: brake hard.
    assert!(super::sim::idm(sp, 5.0, 20.0, 1e4, 0.0) > 1.0);
    assert!(super::sim::idm(sp, 20.0, 20.0, 1e4, 0.0).abs() < 0.01);
    assert!(super::sim::idm(sp, 15.0, 20.0, 10.0, 10.0) < -5.0);
    // A follower settles behind a stopped car at about the standstill gap.
    let (mut s, mut v) = (0.0f32, 15.0f32);
    let stop = 200.0;
    for _ in 0..2000 {
        let a = super::sim::idm(sp, v, 20.0, stop - s, v);
        let nv = (v + a * 0.1).max(0.0);
        s += 0.5 * (v + nv) * 0.1;
        v = nv;
    }
    assert!(v < 0.05 && stop - s > 1.0 && stop - s < sp.s0 + 1.0, "stopped {} m short at {v}", stop - s);
}

#[test]
fn density_scales_with_the_setting() {
    let a = road(class::PRIMARY, 0, &[vec2(50.0, 400.0), vec2(1400.0, 400.0)]);
    let b = road(class::MOTORWAY, flag::ONEWAY, &[vec2(50.0, 700.0), vec2(1400.0, 700.0)]);
    let net = Arc::new(build(LONDON, &[a, b]));
    let count = |l: Level, hours: f32| {
        let mut sim = Sim::new(Proj::tile_frame(LONDON));
        sim.set_level(l);
        sim.add_tile(net.clone());
        sim.reset(100.0, 0.2, Vec3::new(700.0, 50.0, 500.0), hours);
        sim.vehicle_count() as f32
    };
    let (low, med, high) = (count(Level::Low, 8.0), count(Level::Medium, 8.0), count(Level::High, 8.0));
    assert!(high > 60.0, "{high} vehicles at High");
    assert!((low / high - 0.4).abs() < 0.12 && (med / high - 0.7).abs() < 0.12, "{low} / {med} / {high}");
    // Rush hour against the small hours.
    let night = count(Level::High, 3.0);
    assert!(night < high * 0.25, "{night} at 3 am vs {high} at 8 am");
    assert_eq!(count(Level::Off, 8.0), 0.0);
}

#[test]
fn traffic_crosses_tile_edges() {
    // One east-west road over two neighbouring tiles, clipped at the shared edge.
    let a = TileId { x: LONDON.x, ..LONDON };
    let b = TileId { x: LONDON.x + 1, ..LONDON };
    let pa = Proj::tile_frame(a);
    let pb = Proj::tile_frame(b);
    let (_, hi_a) = a.rect(&pa);
    let z = 600.0;
    // The edge in a's frame is x = hi_a.x; in b's frame x = 0.
    let ra = RoadLine { class: class::PRIMARY, flags: 0, layer: 0, pts: vec![vec2(200.0, z), vec2(hi_a.x, z)], cut: [false, true] };
    let to_b = pa.map_to(&pb);
    let zb = to_b.apply(vec2(hi_a.x, z)).y;
    let rb = RoadLine { class: class::PRIMARY, flags: 0, layer: 0, pts: vec![vec2(0.0, zb), vec2(900.0, zb)], cut: [true, false] };
    let na = build(a, &[ra]);
    let nb = build(b, &[rb]);
    assert_eq!(na.edge_out.len(), 1);
    assert_eq!(na.edge_in.len(), 1);
    assert_eq!(nb.edge_in.len(), 1);
    let mut sim = Sim::new(pa);
    sim.set_level(Level::High);
    sim.add_tile(Arc::new(na.clone()));
    sim.add_tile(Arc::new(nb.clone()));
    // The eastbound lane of a continues into b's eastbound lane; and back.
    let out = na.edge_out[0];
    let (tb, lb) = sim.edge_link(a, out).expect("edge link");
    assert_eq!(tb, b);
    assert!(nb.start(lb).1.x > 0.9);
    let back = nb.edge_out[0];
    assert_eq!(sim.edge_link(b, back).map(|x| x.0), Some(a));
    // Vehicles flow across: the same vehicle is seen in a, then in b.
    let eye = Vec3::new(hi_a.x, 100.0, z);
    let mut crossed = 0;
    let mut last: std::collections::HashMap<u32, u16> = Default::default();
    let sb = sim.slot_of(b).unwrap();
    for i in 0..1800 {
        let t = i as f64 * 0.1;
        sim.advance(t, eye, 17.0);
        sim.check().unwrap();
        let now: std::collections::HashMap<u32, u16> = sim.positions().into_iter().map(|(s, _, _, id)| (id, s)).collect();
        crossed += now.iter().filter(|(id, s)| **s == sb && last.get(id).is_some_and(|s0| *s0 != sb)).count();
        last = now;
    }
    assert!(crossed >= 5, "{crossed} vehicles crossed the tile edge");
    // Positions are continuous across the edge: the eastbound lane ends where b's begins.
    let xf = pb.map_to(&pa);
    let (pe, _) = na.end(out);
    let (ps, _) = nb.start(lb);
    assert!(xf.apply(ps).distance(pe) < 0.5, "{:?} vs {:?}", xf.apply(ps), pe);
}

#[test]
fn off_means_nothing() {
    let mut t = Traffic::new(Proj::tile_frame(LONDON));
    t.set_inline();
    t.set_level(Level::Off);
    t.add_tile(Arc::new(build(LONDON, &[road(class::PRIMARY, 0, &[vec2(50.0, 400.0), vec2(1400.0, 400.0)])])));
    let f = super::Frame { time: 10.0, hours: 8.0, view: view_at(Vec3::new(700.0, 50.0, 400.0)) };
    assert!(t.frame(f).is_none());
    assert_eq!(t.stats().vehicles, 0);
    t.set_level(Level::Medium);
    let n = t.frame(f).map(|s| s.inst.len()).unwrap_or(0);
    assert!(n > 0);
    t.set_level(Level::Off);
    assert!(t.frame(f).is_none());
}

/// Real London tiles from the cache (if present): the lane graph builds,
/// and an hour-long rush-hour run keeps flowing with no overlaps.
#[test]
fn london_tiles_flow() {
    let ids: Vec<TileId> = (8185..=8187).flat_map(|x| (5447..=5448).map(move |y| TileId { z: 14, x, y })).collect();
    let mut nets = Vec::new();
    for id in &ids {
        let path = std::path::Path::new("cache/tiles/14").join(id.x.to_string()).join(format!("{}.pbf", id.y));
        let Ok(bytes) = std::fs::read(&path) else {
            eprintln!("london_tiles_flow: no tile cache, skipped");
            return;
        };
        let bytes = if bytes.starts_with(&[0x1f, 0x8b]) {
            use std::io::Read;
            let mut out = Vec::new();
            flate2::read::GzDecoder::new(&bytes[..]).read_to_end(&mut out).unwrap();
            out
        } else {
            bytes
        };
        let layers = crate::mvt::decode(&bytes).unwrap();
        let proj = Proj::tile_frame(*id);
        let f = crate::osm::extract(&layers, *id, &proj);
        let t0 = std::time::Instant::now();
        let n = build(*id, &f.roads);
        eprintln!(
            "tile {}/{}: {} roads -> {} lanes ({} edge in / {} out), {:.1} ms, {} KB",
            id.x,
            id.y,
            f.roads.len(),
            n.lanes.len(),
            n.edge_in.len(),
            n.edge_out.len(),
            t0.elapsed().as_secs_f64() * 1e3,
            n.bytes() / 1024
        );
        nets.push(Arc::new(n));
    }
    let proj = Proj::tile_frame(ids[2]);
    let mut sim = Sim::new(proj);
    sim.set_level(Level::High);
    for n in &nets {
        sim.add_tile(n.clone());
    }
    let (lo, hi) = ids[2].rect(&proj);
    let eye = ((lo + hi) * 0.5).extend(300.0);
    let eye = Vec3::new(eye.x, 300.0, eye.y);
    let mut speeds = Vec::new();
    let mut t_step = 0.0;
    for i in 0..6000 {
        let t = i as f64 * 0.1;
        let t0 = std::time::Instant::now();
        sim.advance(t, eye, 8.0);
        t_step += t0.elapsed().as_secs_f64();
        if i % 100 == 0 {
            sim.check().unwrap();
            let mut snap = Snapshot::default();
            sim.snapshot(t, &view_at(eye), &mut snap);
            let mean = snap.speed.iter().sum::<f32>() / snap.speed.len().max(1) as f32;
            let stopped = snap.speed.iter().filter(|&&v| v < 0.5).count() as f32 / snap.speed.len().max(1) as f32;
            speeds.push((mean, stopped, sim.vehicle_count()));
        }
    }
    eprintln!("{}", sim.diagnose());
    eprintln!("london: {:?}", &speeds[speeds.len() - 5..]);
    eprintln!("london: {:.3} ms per step, {} vehicles", t_step * 1e3 / 6000.0, sim.vehicle_count());
    let late = &speeds[speeds.len() / 2..];
    let mean = late.iter().map(|s| s.0).sum::<f32>() / late.len() as f32;
    let stopped = late.iter().map(|s| s.1).sum::<f32>() / late.len() as f32;
    assert!(mean > 3.0, "mean speed {mean:.1} m/s");
    assert!(stopped < 0.6, "{:.0} % stopped", stopped * 100.0);
    // The population holds up (sources keep feeding, nothing drains or explodes).
    let (first, last) = (speeds[5].2 as f32, speeds[speeds.len() - 1].2 as f32);
    assert!(last > first * 0.6 && last < first * 1.6, "{first} -> {last} vehicles");
}
