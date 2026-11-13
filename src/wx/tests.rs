//! Flight-model tests in the WX air: the aircraft stay numerically sound in
//! extreme turbulence, rotational gusts reach the 6-DoF model, turbulence
//! scales with the setting, and mountain-wave / storm air reaches it.

use crate::flight::{self, Aircraft, Controls, Environment};
use crate::weather::{WindField, Wx};
use glam::{Vec2, Vec3, vec3};
use std::cell::Cell;

struct TurbEnv {
    wf: WindField,
    severity: f32,
    t: Cell<f32>,
}

impl Environment for TurbEnv {
    fn ground_height(&self, _: Vec2) -> f32 {
        0.0
    }
    fn obstacle_top(&self, _: Vec2) -> Option<f32> {
        None
    }
    fn is_water(&self, _: Vec2) -> bool {
        false
    }
    fn updraft(&self, _: Vec3) -> f32 {
        0.0
    }
    fn wind(&self, p: Vec3) -> Vec3 {
        self.wf.at(p, self.t.get(), 0.0, self.severity)
    }
}

/// Fly `secs` with a simple attitude holder; returns (aircraft, rms vertical
/// load-factor deviation, peak rotational gust rad/s).
fn fly(id: &str, env: &TurbEnv, secs: f32) -> (Aircraft, f32, f32) {
    let def = flight::find(id).expect("type");
    let v = def.vref_kt.max(def.vr_kt) * 1.45 * flight::KT;
    let alt = 1800.0;
    let mut a = Aircraft::airborne(&def, vec3(0.0, alt, 0.0), 0.3, v);
    let tp = def.trim_point_at(v, 0.0, alt, false);
    let mut c = Controls { throttle: tp.throttle, trim: tp.trim, elevator: tp.elevator, gear_down: false, ..Default::default() };
    let dt = 1.0 / 120.0;
    let (mut s2, mut n, mut peak) = (0.0f32, 0usize, 0.0f32);
    for k in 0..(secs / dt) as usize {
        env.t.set(k as f32 * dt);
        // Wings level, hold the trim pitch.
        c.aileron = (-a.roll() * 2.0 - a.omega.z * 0.4).clamp(-1.0, 1.0);
        c.elevator = (tp.elevator + (tp.pitch - a.pitch()) * 2.5 + a.omega.x * 0.3).clamp(-1.0, 1.0);
        a.step(&def, &c, env, dt);
        assert!(a.pos.is_finite() && a.vel.is_finite() && a.omega.is_finite() && a.rot.is_finite(), "{id}: non-finite state at t {}", k as f32 * dt);
        if let Some(r) = &a.crashed {
            assert!(!r.contains("Simulation fault"), "{id}: {r}");
            break;
        }
        if k > 240 {
            s2 += (a.g_load() - 1.0).powi(2);
            n += 1;
            peak = peak.max(a.gust_rates.length());
        }
    }
    (a, (s2 / n.max(1) as f32).sqrt(), peak)
}

fn env(metar: &str, severity: f32, scale: f32) -> TurbEnv {
    let wx = Wx::from_metar_str(metar).unwrap();
    TurbEnv { wf: WindField { scale, ..WindField::from_wx(&wx) }, severity, t: Cell::new(0.0) }
}

#[test]
fn extreme_turbulence_stays_finite() {
    let e = env("EGLL 151350Z 27035G55KT 9999 SCT040 18/09 Q1018", 4.0, 1.0);
    for id in ["kestrel", "swift", "peregrine", "lancer", "aerolink", "meridian320", "galaxy", "zlin"] {
        let (a, g_rms, peak) = fly(id, &e, 60.0);
        eprintln!("{id}: extreme turbulence: g rms {g_rms:.2}, peak rotational gust {peak:.3} rad/s, crashed {:?}", a.crashed);
        assert!(peak > 0.005, "{id}: rotational gusts reach the model ({peak})");
    }
}

#[test]
fn turbulence_scales_with_severity_and_setting() {
    let base = "EGLL 151350Z 27012KT 9999 SCT040 18/09 Q1018";
    let (_, light, _) = fly("meridian320", &env(base, 1.0, 1.0), 40.0);
    let (_, severe, _) = fly("meridian320", &env(base, 3.0, 1.0), 40.0);
    let (_, reduced, _) = fly("meridian320", &env(base, 3.0, 0.5), 40.0);
    let (_, off, peak_off) = fly("meridian320", &env(base, 3.0, 0.0), 40.0);
    eprintln!("A320 g rms: light {light:.3} severe {severe:.3} severe-reduced {reduced:.3} off {off:.3} (rot. gust {peak_off:.4})");
    // What the turbulence adds over the (smooth-air) attitude-hold wobble.
    let ex = |x: f32| (x * x - off * off).max(0.0).sqrt();
    assert!(ex(severe) > 2.0 * ex(light), "severe {severe} vs light {light}");
    assert!(ex(reduced) < 0.75 * ex(severe) && ex(reduced) > 0.2 * ex(severe), "reduced {reduced} vs {severe}");
    assert!(peak_off < 0.003, "no rotational gusts with turbulence off: {peak_off}");
    // In the Dryden sense: severe at medium altitude bumps an airliner by ±0.3 g or more (rms ≳ 0.1).
    assert!(ex(severe) > 0.08, "severe rms {severe}");
}

#[test]
fn example_metars_make_their_weather() {
    use super::storm::StormSpec;
    let wx = |i: usize| Wx::from_metar_str(&format!("EGLL {}", super::EXAMPLE_METARS[i].1)).expect("decodes");
    let storm = StormSpec::from_wx(&wx(0));
    assert!(storm.active() && storm.flash_rate > 0.0 && storm.strength > 0.9, "{storm:?}");
    let squall = StormSpec::from_wx(&wx(1));
    assert!(squall.hail && squall.active());
    let near = StormSpec::from_wx(&wx(2));
    assert!(near.active() && near.home_km > 10.0, "VCTS: storms in the vicinity, not overhead {near:?}");
    let wave = super::mountain::WaveSpec::from_wx(&wx(3));
    assert!(wave.drive() > 0.5 && wave.lenticular, "{wave:?}");
    assert!(!StormSpec::from_wx(&wx(4)).active() && wx(4).temp_c < 0.0);
    assert!(wx(5).gust_kt > 40.0);
}

#[test]
fn mountain_wave_and_storm_air_reach_the_wind() {
    // A WxSim over a synthetic ridge: the world's wind / updraft see the waves.
    let w = Wx::from_metar_str(&format!("LOWI {}", super::EXAMPLE_METARS[3].1)).unwrap();
    let mut s = super::WxSim::default();
    s.set_weather(&w, 5, 0.0);
    let ridge = |p: Vec2| 600.0 + 1800.0 * (-(p.y / 3000.0).powi(2)).exp(); // east-west ridge at z = 0
    s.tick(1.0, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, 10, &ridge);
    assert!(s.waves.active(), "southerly gale over an east-west ridge makes waves");
    // Downwind (the wind blows from the south: toward -z) the air rises and sinks.
    let ws: Vec<f32> = (1..40).map(|i| s.air(vec3(0.0, 3500.0, -(i as f32) * 500.0), vec3(0.0, 3500.0, -(i as f32) * 500.0)).1).collect();
    assert!(ws.iter().cloned().fold(0.0, f32::max) > 1.0 && ws.iter().cloned().fold(0.0, f32::min) < -1.0, "{ws:?}");
    // Upwind: nothing.
    let up = s.air(vec3(0.0, 3500.0, 15_000.0), vec3(0.0, 3500.0, 15_000.0)).1;
    assert!(up.abs() < 0.5, "{up}");
}
