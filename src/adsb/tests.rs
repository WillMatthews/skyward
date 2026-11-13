//! The whole pipeline on the recorded Heathrow fixture, with a flat world.

use super::*;
use crate::flight::Environment;
use glam::Vec2;

struct Flat;
impl Environment for Flat {
    fn ground_height(&self, _xz: Vec2) -> f32 {
        25.0
    }
    fn obstacle_top(&self, _xz: Vec2) -> Option<f32> {
        None
    }
    fn is_water(&self, _xz: Vec2) -> bool {
        false
    }
    fn updraft(&self, _p: Vec3) -> f32 {
        0.0
    }
    fn wind(&self, _p: Vec3) -> Vec3 {
        Vec3::new(4.0, 0.0, 1.0)
    }
}

fn frames() -> Vec<Snapshot> {
    feed::parse_recording(include_str!("../../tests/adsb/egll_arrivals.json")).unwrap()
}

fn env() -> Env<'static> {
    Env { proj: crate::osm::Proj::new(51.4706, -0.4619), qnh_hpa: 1013.25, runways: &[], env: &Flat, layouts: Vec::new() }
}

#[test]
fn replays_the_heathrow_recording() {
    let mut lt = LiveTraffic::new();
    lt.publish = false;
    lt.replay(frames());
    let env = env();
    let settings = Settings { max_aircraft: 500, ..Default::default() };
    let me = |t: f64| Me { pos: Vec3::new(0.0, 600.0, 0.0), vel: Vec3::ZERO, sim_time: t };
    let mut t = 0.0;
    let mut prev: HashMap<u32, Vec3> = HashMap::new();
    let mut max_step = 0.0f32;
    while t < 30.0 {
        lt.step(me(t), &env, 1.0 / 30.0, &settings);
        for a in lt.aircraft.values().filter(|a| a.shown) {
            if let Some(p) = prev.get(&a.hex) {
                // No teleports frame to frame: ≤ 1.3 × the fastest real speed (Mach 0.9).
                let step = a.pos.distance(*p);
                max_step = max_step.max(step);
            }
        }
        prev = lt.aircraft.values().map(|a| (a.hex, a.pos)).collect();
        t += 1.0 / 30.0;
    }
    eprintln!("replay: {} aircraft, largest frame step {max_step:.1} m, {:.3} ms / frame", lt.shown(), lt.cpu_ms);
    assert!(lt.shown() > 50, "{} shown", lt.shown());
    assert!(max_step < 320.0 * 1.3 / 30.0, "frame step {max_step:.1} m");
    // The picture carries callsigns, types and the live flag.
    let pic = lt.traffic();
    assert!(pic.iter().all(|t| t.live && !t.name.is_empty()));
    assert!(pic.iter().any(|t| t.kind == "A20N"));
    // Ground traffic sits on the ground, airborne traffic above it.
    let g = lt.aircraft.values().filter(|a| a.shown && a.on_ground).count();
    assert!(g > 10, "{g} on the ground");
    for a in lt.aircraft.values().filter(|a| a.shown) {
        assert!(a.pos.y >= 25.0 - 0.01, "{} below ground: {}", a.label(), a.pos.y);
        if a.on_ground {
            assert!(a.pos.y < 25.0 + 8.0, "{} floats: {}", a.label(), a.pos.y);
        }
    }
    // Arrivals on final (low, descending) have the gear down and flaps out.
    let fin: Vec<&Live> = lt.aircraft.values().filter(|a| a.shown && !a.on_ground && a.pos.y < 600.0 && a.vel.y < -1.5 && a.stand.model == "meridian320").collect();
    assert!(!fin.is_empty());
    for a in fin {
        assert!(a.state.gear > 0.9, "{} gear {}", a.label(), a.state.gear);
        assert!(a.state.flaps > 0.3, "{} flaps {}", a.label(), a.state.flaps);
    }
    // Budget: only the nearest N are shown.
    let small = Settings { max_aircraft: 10, ..Default::default() };
    lt.step(me(t), &env, 1.0 / 30.0, &small);
    assert_eq!(lt.shown(), 10);
    let far = lt.aircraft.values().filter(|a| a.shown).map(|a| a.dist).fold(0.0, f32::max);
    assert!(lt.aircraft.values().filter(|a| !a.shown).all(|a| a.dist >= far - 1.0));
    // Ground traffic can be hidden.
    let no_ground = Settings { ground: false, max_aircraft: 500, ..Default::default() };
    lt.step(me(t + 0.1), &env, 1.0 / 30.0, &no_ground);
    assert!(lt.aircraft.values().filter(|a| a.shown).all(|a| !a.on_ground));
}

/// ≤ 0.5 ms of CPU per frame for 200 aircraft (release builds).
#[test]
fn two_hundred_aircraft_fit_the_budget() {
    let mut fr = frames();
    // Triple the recording with shifted copies (new addresses).
    for f in fr.iter_mut() {
        let base = f.reports.clone();
        for k in 1..3u32 {
            for r in &base {
                let mut c = r.clone();
                c.hex = r.hex ^ (k << 20);
                c.lat += 0.05 * k as f64;
                f.reports.push(c);
            }
        }
    }
    let n = fr[0].reports.len();
    assert!(n >= 200, "{n}");
    let mut lt = LiveTraffic::new();
    lt.publish = false;
    lt.replay(fr);
    let env = env();
    let settings = Settings { max_aircraft: 500, ..Default::default() };
    let mut worst = 0.0f32;
    let mut sum = 0.0f32;
    let mut frames_n = 0;
    for k in 0..400 {
        let t = 12.0 + k as f64 / 60.0;
        lt.step(Me { pos: Vec3::new(0.0, 600.0, 0.0), vel: Vec3::ZERO, sim_time: t }, &env, 1.0 / 60.0, &settings);
        if k > 20 {
            worst = worst.max(lt.cpu_ms);
            sum += lt.cpu_ms;
            frames_n += 1;
        }
    }
    let mean = sum / frames_n as f32;
    eprintln!("{} aircraft: mean {mean:.3} ms, worst {worst:.3} ms per frame", lt.shown());
    assert!(lt.shown() >= 200);
    if !cfg!(debug_assertions) {
        assert!(mean < 0.5, "mean {mean:.3} ms");
    }
}
