//! Scripted test flights for structures, crashes and effects (Track L):
//! `SKYWARD_SCENARIO=<name>` with the usual headless `--shot` / `--audio-render`
//! flags. Each scenario positions the aircraft once (relative to the start
//! runway where it matters) and then flies a simple control law.
//!
//! | name | what |
//! |---|---|
//! | `pullup` | hard pull for 4 s from t = 2 s (wing flex) |
//! | `overg` | re-launch at 150 kt, full back stick (C172: wing separation) |
//! | `bellyland` | gear-up landing on the start runway |
//! | `hardland` | ~1.4x design sink rate: the gear collapses |
//! | `crash` | 25° dive into the ground beside the runway |
//! | `transonic` | fast low pass at Mach 0.98 with reheat (vapour cone) |
//! | `cruise` | hold heading and altitude (contrails) |
//! | `inverted` | aerobatics: roll inverted at 130 kt / 300 m over the field and hold it (smoke on with the aerobat's smoke system) |
//!
//! Forced impacts (Track CR): `SKYWARD_CRASH_TEST=<kind>` puts the aircraft a
//! few metres above the start runway (or water / a building) already on the
//! impact path, so the contact happens about a second in. Kinds:
//! `hardland` (2× the design sink), `nosein` (20° nose-down dive), `belly`
//! (gear up), `wingtip` (30° of bank), `ditch` (gear up onto the nearest
//! water), `building` (a wingtip through the nearest building at flying
//! speed), `tailstrike`, `fly` (no impact: level at 400 m over the field,
//! a wings-level / height-hold "pilot" — for flying on with damage). Override the impact with `SKYWARD_CRASH_KT`,
//! `SKYWARD_CRASH_SINK` (m/s), `SKYWARD_CRASH_PITCH` / `SKYWARD_CRASH_BANK`
//! (degrees). Capture frames after it with `SKYWARD_SHOT_SEQ=1,2,4,8`.
//!
//! Damage injection: `SKYWARD_DAMAGE=lwing:0.5,eng1:fire,...` damages the
//! airframe at the start (after any staging); see [`parse_damage`].

use crate::flight::{Aircraft, KT};
use crate::sim::Sim;
use crate::world::World;
use glam::{vec2, Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: String,
    pub t: f32,
    started: bool,
}

impl Scenario {
    pub fn from_env() -> Option<Scenario> {
        let name = match std::env::var("SKYWARD_CRASH_TEST") {
            Ok(k) if !k.trim().is_empty() => format!("ct:{}", k.trim().to_ascii_lowercase()),
            _ => std::env::var("SKYWARD_SCENARIO").ok()?.trim().to_ascii_lowercase(),
        };
        if name.is_empty() {
            return None;
        }
        Some(Scenario { name, t: 0.0, started: false })
    }
    pub fn restart(&mut self) {
        self.t = 0.0;
        self.started = false;
    }
}

/// Runway frame: threshold, direction, length.
fn runway(sim: &Sim) -> (Vec2, Vec2, f32) {
    match &sim.runway {
        Some(r) => {
            let d = (r.b - r.a).normalize_or(vec2(0.0, -1.0));
            (r.a, d, r.a.distance(r.b))
        }
        None => (vec2(sim.ac.pos.x, sim.ac.pos.z), vec2(0.0, -1.0), 2000.0),
    }
}

/// Height above the ground under the aircraft.
fn sc_alt(world: &World, ac: &Aircraft) -> f32 {
    ac.pos.y - crate::flight::Environment::ground_height(world, vec2(ac.pos.x, ac.pos.z))
}

fn heading_of(d: Vec2) -> f32 {
    d.x.atan2(-d.y)
}

/// Air start on the extended centreline `back` m before the threshold at `agl` m.
fn approach(sim: &mut Sim, world: &World, back: f32, agl: f32, kt: f32) {
    let (a, d, _) = runway(sim);
    let p = a - d * back;
    let gh = crate::flight::Environment::ground_height(world, p);
    let def = sim.def().clone();
    sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + agl, p.y), heading_of(d), kt * KT);
    sim.ctl = crate::flight::trimmed_controls_at(&def, kt * KT, gh + agl);
    sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
}

fn env_f(k: &str) -> Option<f32> {
    std::env::var(k).ok().and_then(|v| v.trim().parse::<f32>().ok()).filter(|v| v.is_finite())
}

/// A forced impact (`SKYWARD_CRASH_TEST`): where, how fast, which attitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactSpec {
    /// World XZ of the point under the CG at the start, and the heading.
    pub at: Vec2,
    pub heading: f32,
    /// Lowest point above the ground at the start (m).
    pub clear: f32,
    pub kt: f32,
    /// Sink rate (m/s, + down), pitch and bank (degrees).
    pub sink: f32,
    pub pitch: f32,
    pub bank: f32,
    pub gear_down: bool,
}

/// Default impact parameters per kind (before the env overrides): (kt, sink m/s, pitch°, bank°, gear down).
pub fn impact_defaults(def: &crate::flight::AircraftDef, kind: &str) -> (f32, f32, f32, f32, bool) {
    let vref = if def.vref_kt > 0.0 { def.vref_kt } else { def.vs1_kt * 1.3 }.max(if def.rotor.is_some() { 0.0 } else { 40.0 });
    let ms = def.max_touchdown_sink.max(0.5);
    let heli = def.rotor.is_some();
    match kind {
        "hardland" if heli => (5.0, 3.0 * ms, 2.0, 0.0, true),
        "hardland" => (vref, 2.0 * ms, 4.0, 0.0, true),
        "nosein" => {
            let kt = vref * 1.4;
            (kt, kt * KT * 20f32.to_radians().sin(), -20.0, 0.0, true)
        }
        "belly" => (vref, 1.0, 3.0, 0.0, false),
        "wingtip" => (vref * 1.05, 0.8, 2.0, 30.0, true),
        "ditch" => (vref, 1.5, 6.0, 0.0, false),
        "building" => (vref * 1.3, 0.0, 2.0, 0.0, false),
        "tailstrike" => (vref * 0.95, 1.2, 14.0, 0.0, true),
        "fly" => (vref * 1.5, 0.0, 2.0, 0.0, false),
        _ => (vref, ms, 3.0, 0.0, true),
    }
}

/// Find where to stage the impact. None = the scenery isn't there yet.
fn impact_site(sim: &Sim, world: &World, kind: &str) -> Option<(Vec2, f32, f32)> {
    use crate::flight::Environment;
    let (a, d, len) = runway(sim);
    let side = Vec2::new(-d.y, d.x);
    let hdg = heading_of(d);
    match kind {
        "ditch" => {
            // The nearest water to the runway (rings outward).
            for r in (1..=80).map(|k| k as f32 * 50.0) {
                let n = ((r * 0.08) as usize).max(8);
                for k in 0..n {
                    let ang = k as f32 / n as f32 * std::f32::consts::TAU;
                    let p = a + d * (len * 0.5) + Vec2::new(ang.cos(), ang.sin()) * r;
                    // Water 300 m ahead too, so the slide stays on it.
                    if (0..=6).all(|k| world.is_water(p + d * (k as f32 * 100.0))) {
                        return Some((p - d * 20.0, hdg, 0.0));
                    }
                }
            }
            None
        }
        "building" => {
            // A building 6..40 m tall near the field, passed alongside so that
            // only the outer quarter of the left wing goes through its wall:
            // the fuselage track (and the inner wing) must be clear.
            let semi = sim.def().wingspan * 0.5;
            let c = a + d * (len * 0.5);
            let mut cands: Vec<(f32, Vec2, f32)> = Vec::new();
            for i in -50..=50 {
                for j in -50..=50 {
                    let p = c + Vec2::new(i as f32, j as f32) * 40.0;
                    let gh = world.ground_height(p);
                    if let Some(t) = world.obstacle_top(p).map(|t| t - gh).filter(|h| (6.0..40.0).contains(h)) {
                        cands.push((p.distance(c), p, t));
                    }
                }
            }
            cands.sort_by(|x, y| x.0.total_cmp(&y.0));
            let clear = |from: Vec2, dir: Vec2, t0: f32, t1: f32| (0..=((t1 - t0) / 4.0) as i32).all(|k| world.obstacle_top(from + dir * (t0 + k as f32 * 4.0)).is_none());
            for (_, p, h) in cands.into_iter().take(200) {
                for dir in [d, -d, side, -side] {
                    let lat = Vec2::new(-dir.y, dir.x);
                    let Some(edge) = (0..120).map(|k| p + lat * k as f32).find(|q| world.obstacle_top(*q).is_none()) else { continue };
                    let cg = edge + lat * (semi * 0.75);
                    if clear(cg, dir, -250.0, 60.0) && clear(cg - lat * (semi * 0.6), dir, -250.0, 60.0) && clear(cg + lat * semi, dir, -250.0, 60.0) {
                        return Some((cg - dir * 250.0, heading_of(dir), (h * 0.5).max(3.0)));
                    }
                }
            }
            None
        }
        "fly" => Some((a - d * 1500.0, hdg, 400.0)),
        _ => Some((a + d * (len * 0.35) + side * 0.0, hdg, 0.0)),
    }
}

/// One item of a `SKYWARD_DAMAGE` spec.
#[derive(Clone, Debug, PartialEq)]
pub enum DamageItem {
    /// Strike damage 0..1 (1 = torn off) to these parts.
    Hurt(Vec<crate::damage::Part>, f32),
    /// Engine `i` (0-based) on fire.
    Fire(usize),
    /// A wing tank leaking (0 left, 1 right).
    Leak(usize),
}

/// Parse `SKYWARD_DAMAGE`: comma-separated `part:amount` (amount 0..1, 1 =
/// torn off; `off` = 1). Parts: `lwing` `rwing` `htail` / `elev` (both
/// tailplane halves) `lhtail` `rhtail` `fin` / `rudder` `tail` (the rear
/// fuselage) `nose` `fuselage` / `body` `gear` (both mains) `ngear` `eng1`..`eng4`.
/// Also `eng<N>:fire`, `leak` / `lleak` / `rleak`, `gear:collapse`.
pub fn parse_damage(spec: &str) -> Vec<DamageItem> {
    use crate::damage::Part;
    let mut out = Vec::new();
    for item in spec.split(',').map(|x| x.trim().to_ascii_lowercase()).filter(|x| !x.is_empty()) {
        let (k, v) = item.split_once(':').unwrap_or((item.as_str(), "1"));
        let eng = k.strip_prefix("eng").and_then(|n| n.parse::<usize>().ok()).filter(|n| (1..=4).contains(n)).map(|n| n - 1);
        if let (Some(i), "fire") = (eng, v) {
            out.push(DamageItem::Fire(i));
            continue;
        }
        match k {
            "leak" | "lleak" => {
                out.push(DamageItem::Leak(0));
                continue;
            }
            "rleak" => {
                out.push(DamageItem::Leak(1));
                continue;
            }
            _ => {}
        }
        let amount = match v {
            "off" | "collapse" | "lost" => 1.0,
            x => match x.parse::<f32>() {
                Ok(a) if a.is_finite() => a.clamp(0.0, 1.0),
                _ => continue,
            },
        };
        let parts = match (k, eng) {
            (_, Some(i)) => vec![Part::engine(i)],
            ("lwing", _) => vec![Part::WingL],
            ("rwing", _) => vec![Part::WingR],
            ("htail" | "elev" | "elevator", _) => vec![Part::HTailL, Part::HTailR],
            ("lhtail", _) => vec![Part::HTailL],
            ("rhtail", _) => vec![Part::HTailR],
            ("fin" | "rudder", _) => vec![Part::Fin],
            ("tail", _) => vec![Part::Aft],
            ("nose", _) => vec![Part::Nose],
            ("fuselage" | "body", _) => vec![Part::Fuselage],
            ("gear", _) => vec![Part::GearL, Part::GearR],
            ("ngear", _) => vec![Part::GearNose],
            _ => continue,
        };
        out.push(DamageItem::Hurt(parts, amount));
    }
    out
}

/// Apply a `SKYWARD_DAMAGE` spec to the aircraft now.
pub fn inject_damage(sim: &mut Sim, spec: &str) {
    use crate::systems::failures::{Failure, Kind};
    let def = sim.def().clone();
    let sd = crate::damage::struct_def(def.id);
    for item in parse_damage(spec) {
        match item {
            DamageItem::Hurt(parts, a) => {
                for p in parts {
                    let have = sim.ac.dmg.hurt[p.idx()];
                    sim.ac.dmg.hit(sd, &def, p, (a - have).max(0.0), "injected");
                }
            }
            DamageItem::Fire(i) => {
                sim.ac.dmg.hit(sd, &def, crate::damage::Part::engine(i), 0.6, "injected");
                let ac = sim.ac.clone();
                sim.ac.sys.set_failure(&def, &ac, Failure::new(Kind::EngineFire, i), true);
            }
            DamageItem::Leak(i) => {
                let ac = sim.ac.clone();
                sim.ac.sys.set_failure(&def, &ac, Failure::new(Kind::FuelLeak, i), true);
            }
        }
    }
    eprintln!("damage injected: {spec} -> {:?}", sim.ac.dmg.annunciations());
}

/// Stage a forced impact: the aircraft placed on its impact path.
fn stage_impact(sim: &mut Sim, world: &World, kind: &str) -> bool {
    let def = sim.def().clone();
    let Some((at, heading, height)) = impact_site(sim, world, kind) else { return false };
    let (kt, sink, pitch, bank, gear) = impact_defaults(&def, kind);
    let kt = env_f("SKYWARD_CRASH_KT").unwrap_or(kt);
    let sink = env_f("SKYWARD_CRASH_SINK").unwrap_or(sink);
    let pitch = env_f("SKYWARD_CRASH_PITCH").unwrap_or(pitch).to_radians();
    let bank = env_f("SKYWARD_CRASH_BANK").unwrap_or(bank).to_radians();
    let gh = crate::flight::Environment::ground_height(world, at);
    let speed = (kt * KT).max(0.1);
    let mut ac = Aircraft::airborne(&def, Vec3::new(at.x, gh + 50.0, at.y), heading, speed);
    ac.rot = crate::flight::attitude(heading, pitch, bank);
    if gear && def.retractable_gear {
        ac.extend_gear_now();
    } else if !gear && def.retractable_gear {
        ac.retract_gear_now();
    }
    // Lowest point of the airframe below the CG at this attitude.
    let low = def
        .gear
        .iter()
        .filter(|g| gear || !g.retracts)
        .map(|g| g.pos)
        .chain(def.hard_points.iter().map(|h| h.pos))
        .map(|p| (ac.rot * p).y)
        .fold(0.0f32, f32::min);
    // About 0.6 s (at least 1.5 m) of fall before the contact.
    let clear = height.max((sink * 0.6).max(1.5));
    ac.pos.y = gh + clear - low;
    let fwd = Vec2::new(heading.sin(), -heading.cos());
    let h = (speed * speed - sink * sink).max(0.0).sqrt();
    ac.vel = Vec3::new(fwd.x * h, -sink, fwd.y * h);
    if def.rotor.is_some() {
        ac.vel = Vec3::new(fwd.x * speed, -sink, fwd.y * speed);
    }
    sim.ac = ac;
    sim.ctl = crate::flight::trimmed_controls_at(&def, speed, gh + clear);
    sim.ctl.gear_down = gear;
    // (Rotorcraft keep the trimmed collective: it rides on the throttle.)
    if def.rotor.is_none() {
        sim.ctl.throttle = if kind == "building" || kind == "fly" { 0.8 } else { 0.0 };
    }
    sim.level_hold = None;
    eprintln!("crash test {kind}: {kt:.0} kt, sink {sink:.1} m/s, pitch {:.0}°, bank {:.0}°, gear {}, clear {clear:.1} m", pitch.to_degrees(), bank.to_degrees(), if gear { "down" } else { "up" });
    true
}

pub fn control(sim: &mut Sim, world: &World, dt: f32) {
    let Some(mut sc) = sim.scenario.take() else { return };
    if let Some(kind) = sc.name.strip_prefix("ct:").map(str::to_string) {
        if !sc.started {
            sc.started = stage_impact(sim, world, &kind);
            sim.bot = None;
            sim.ap.engaged = false;
        } else if sim.ac.crashed.is_none() {
            sc.t += dt * sim.warp as f32;
            // Hold the staged attitude until the first contact, then hands off
            // (the player's controls stay as they were).
            if kind == "fly" {
                // A pilot holding the wings level and the height (as far as
                // the damaged airframe allows).
                let ac = &sim.ac;
                sim.ctl.aileron = (-2.5 * ac.roll() - 0.4 * ac.rates().x).clamp(-1.0, 1.0);
                sim.ctl.elevator = (-0.08 * ac.vertical_speed() + 1.5 * (0.05 - ac.pitch())).clamp(-1.0, 1.0);
                sim.ctl.rudder = (-0.5 * ac.rates().y).clamp(-1.0, 1.0);
                sim.ctl.throttle = 0.85;
            } else if !sim.ac.on_ground && sc.t < 3.0 && kind != "building" {
                let (_, _, pitch, bank, _) = impact_defaults(sim.def(), &kind);
                let pitch = env_f("SKYWARD_CRASH_PITCH").unwrap_or(pitch).to_radians();
                let bank = env_f("SKYWARD_CRASH_BANK").unwrap_or(bank).to_radians();
                sim.ctl.elevator = (3.0 * (pitch - sim.ac.pitch()) - 0.5 * sim.ac.rates().y).clamp(-1.0, 1.0);
                sim.ctl.aileron = (3.0 * (sim.ac.roll() - bank) * -1.0 - 0.3 * sim.ac.rates().x).clamp(-1.0, 1.0);
            } else if kind == "building" {
                // Hold the height and the wings level through the building.
                let ac = &sim.ac;
                sim.ctl.aileron = (-2.0 * ac.roll() - 0.3 * ac.rates().x).clamp(-1.0, 1.0);
                sim.ctl.elevator = (-0.15 * ac.vertical_speed() + 2.0 * (0.04 - ac.pitch())).clamp(-1.0, 1.0);
            } else {
                sim.ctl.elevator = 0.0;
                sim.ctl.aileron = 0.0;
                sim.ctl.brakes = 0.0;
            }
        }
        sim.scenario = Some(sc);
        return;
    }
    if sim.ac.crashed.is_none() {
        sc.t += dt * sim.warp as f32;
    }
    let def = sim.def().clone();
    let first = !sc.started;
    sc.started = true;
    sim.bot = None;
    sim.ap.engaged = false;
    let ac = &sim.ac;
    let vs = ac.vertical_speed();
    let agl = ac.agl(world);
    match sc.name.as_str() {
        "pullup" => {
            if first {
                sim.level_hold = None;
            }
            sim.ctl.elevator = if (2.0..6.0).contains(&sc.t) { 1.0 } else if sc.t >= 6.0 { -0.2 * (sim.ac.pitch() - 0.05) * 5.0 } else { 0.0 };
            sim.ctl.aileron = (-2.0 * sim.ac.roll()).clamp(-1.0, 1.0);
        }
        "overg" => {
            if first {
                let p = sim.ac.pos;
                let h = sim.ac.heading();
                sim.ac = Aircraft::airborne(&def, p, h, 150.0 * KT);
                sim.ctl = crate::flight::trimmed_controls_at(&def, 150.0 * KT, p.y);
                sim.level_hold = None;
            }
            sim.ctl.elevator = if sc.t > 1.5 { 1.0 } else { 0.0 };
            sim.ctl.aileron = if sc.t < 1.5 { (-2.0 * sim.ac.roll()).clamp(-1.0, 1.0) } else { 0.0 };
        }
        "bellyland" | "hardland" => {
            let belly = sc.name == "bellyland";
            let vref = def.vref_kt;
            if first {
                sim.level_hold = None;
                approach(sim, world, 1200.0, 45.0, vref * 1.05);
                sim.set_flap_detent(def.flap_detents.len().saturating_sub(1));
                sim.ac.set_flaps_now(sim.ctl.flaps);
                if belly && def.retractable_gear {
                    sim.ctl.gear_down = false;
                    sim.ac.retract_gear_now();
                } else {
                    sim.ctl.gear_down = true;
                    sim.ac.extend_gear_now();
                }
            }
            let ac = &sim.ac;
            // Descent: a glide path, then a firm (or not) arrival.
            let target = if belly {
                (-0.06 * agl).clamp(-3.5, -0.9)
            } else {
                -def.max_touchdown_sink * 1.4
            };
            let ias = ac.airspeed() / KT;
            let fbw = def.control_law != crate::flight::ControlLaw::Direct;
            let k = if fbw { 0.35 } else { 0.12 };
            sim.ctl.elevator = (k * (target - vs) - if fbw { 0.0 } else { 0.6 * ac.rates().y }).clamp(-1.0, 1.0);
            sim.ctl.throttle = if ac.on_ground || agl < 3.0 { 0.0 } else { (0.35 + 0.05 * (vref - ias)).clamp(0.0, 1.0) };
            let (_, d, _) = runway(sim);
            let he = crate::bot::wrap_pi(heading_of(d) - ac.heading());
            sim.ctl.aileron = (-2.5 * ac.roll() - 1.0 * he).clamp(-1.0, 1.0);
            sim.ctl.rudder = 0.0;
        }
        "crash" => {
            if first {
                let (a, d, _) = runway(sim);
                // Off to the side of the runway, diving in.
                let side = Vec2::new(-d.y, d.x) * 220.0;
                let p = a + d * 300.0 + side;
                let gh = crate::flight::Environment::ground_height(world, p);
                let v = (def.vref_kt * 1.5).max(def.vs1_kt * 2.2);
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + 260.0, p.y), heading_of(d), v * KT);
                sim.ac.rot = crate::flight::attitude(heading_of(d), (-24f32).to_radians(), 0.15);
                sim.ac.vel = sim.ac.forward() * sim.ac.vel.length();
                sim.ctl = crate::flight::trimmed_controls_at(&def, v * KT, gh + 260.0);
                sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
                sim.level_hold = None;
            }
            sim.ctl.elevator = -0.05;
            sim.ctl.aileron = 0.0;
        }
        "transonic" => {
            if first {
                let p = sim.ac.pos;
                let h = sim.ac.heading();
                let gh = crate::flight::Environment::ground_height(world, vec2(p.x, p.z));
                let alt = gh + std::env::var("SKYWARD_SCENARIO_ALT").ok().and_then(|s| s.parse().ok()).unwrap_or(250.0);
                let a = crate::flight::isa(alt);
                let m = std::env::var("SKYWARD_SCENARIO_MACH").ok().and_then(|s| s.parse().ok()).unwrap_or(0.98f32);
                let ias = crate::flight::cas_from_tas(m * a.speed_of_sound, alt);
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, alt, p.z), h, ias);
                sim.ctl = crate::flight::trimmed_controls_at(&def, ias, alt);
                sim.level_hold = Some((h, alt));
            }
            sim.ctl.throttle = 1.0;
        }
        "inverted" => {
            if first {
                let (a, d, _) = runway(sim);
                let p = a + d * 400.0;
                let gh = crate::flight::Environment::ground_height(world, p);
                let kt = 130.0;
                sim.ac = Aircraft::airborne(&def, Vec3::new(p.x, gh + 300.0, p.y), heading_of(d), kt * KT);
                sim.ctl = crate::flight::trimmed_controls_at(&def, kt * KT, gh + 300.0);
                sim.ctl.gear_down = sim.ac.gear_pos() > 0.5;
                sim.level_hold = None;
                sim.vfx.smoke = def.smoke_system;
            }
            // Half roll to inverted, then hold the height with forward stick.
            let ac = &sim.ac;
            let target_roll = if sc.t > 1.0 { std::f32::consts::PI } else { 0.0 };
            let err = crate::bot::wrap_pi(ac.roll() - target_roll);
            sim.ctl.aileron = (-2.0 * err - 0.3 * ac.rates().x).clamp(-1.0, 1.0);
            let h_err = sc_alt(world, ac) - 300.0;
            let inv = ac.up().y < 0.0;
            let pitch = (-0.02 * h_err - 0.08 * vs).clamp(-0.6, 0.6);
            sim.ctl.elevator = if inv { -(0.3 + pitch) } else { 0.1 + pitch }.clamp(-1.0, 1.0);
            sim.ctl.throttle = 0.85;
        }
        "cruise" if first => {
            sim.level_hold = Some((sim.ac.heading(), sim.ac.pos.y));
        }
        _ => {}
    }
    sim.scenario = Some(sc);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damage::Part;

    #[test]
    fn damage_spec_parses() {
        let v = parse_damage("lwing:0.5, eng1:fire,htail:off ,gear:collapse,leak,bogus:1,rwing:x");
        assert_eq!(
            v,
            vec![
                DamageItem::Hurt(vec![Part::WingL], 0.5),
                DamageItem::Fire(0),
                DamageItem::Hurt(vec![Part::HTailL, Part::HTailR], 1.0),
                DamageItem::Hurt(vec![Part::GearL, Part::GearR], 1.0),
                DamageItem::Leak(0),
            ]
        );
        assert_eq!(parse_damage("eng2:2"), vec![DamageItem::Hurt(vec![Part::Eng1], 1.0)]);
        assert!(parse_damage("").is_empty());
    }
}
