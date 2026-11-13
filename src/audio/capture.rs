//! Game-thread side: turns the simulation state into an audio [`Frame`]
//! plus one-shot [`Event`]s (state transitions, cockpit callouts, GPWS).

use super::dsp::smoothstep;
use super::params::*;
use super::voice;
use crate::flight::{AircraftDef, Category, EngineKind, Environment, FT, GRAVITY};
use crate::sim::{CamMode, Sim};
use crate::world::World;
use glam::{Vec2, Vec3};

/// Sound profile for an aircraft type.
pub fn profile_for(def: &AircraftDef, size: f32) -> Profile {
    let n = def.engine_count.min(MAX_ENG);
    let hash = def.id.bytes().fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619));
    let mut p = Profile {
        family: Family::Glider,
        type_id: hash,
        engines: if def.has_engine { n } else { 0 },
        cylinders: 4,
        blades: 2,
        fan_blades: 24,
        prop_gear: if def.prop_gear > 0.0 { def.prop_gear } else { 1.0 },
        prop_radius: def.prop_radius.max(0.5),
        n1_rpm: def.n1_rpm.max(1000.0),
        thrust_max: (def.static_thrust / n.max(1) as f32).max(1.0),
        size: size.max(4.0),
        retract_gear: def.retractable_gear,
        set: SoundSet::None,
        n1_idle: def.n1_idle,
        mass: def.mass,
    };
    p.family = match def.engine_kind {
        EngineKind::None => Family::Glider,
        EngineKind::Piston => {
            if def.category == Category::Warbird || def.engine_power_w > 400_000.0 {
                Family::Warbird
            } else {
                Family::LightPiston
            }
        }
        EngineKind::Turboprop => Family::Turboprop,
        EngineKind::Turboshaft => Family::Helicopter,
        EngineKind::Electric => Family::Drone,
        EngineKind::Jet => {
            // (Concorde's reheat does not make her a fighter.)
            if (def.has_afterburner && def.category != Category::Airliner) || def.category == Category::FastJet {
                Family::Fighter
            } else {
                Family::Airliner
            }
        }
    };
    // Recorded engine set: by type (current ids and the FlightGear-derived
    // ones), else by family.
    let id = def.id.to_ascii_lowercase();
    let has = |k: &[&str]| k.iter().any(|s| id.contains(s));
    p.set = if p.engines == 0 {
        SoundSet::None
    } else if id == "kestrel" || has(&["c172", "cessna"]) {
        SoundSet::C172
    } else if id == "peregrine" || has(&["spitfire", "spit"]) {
        SoundSet::Merlin
    } else if id == "lancer" || has(&["f16", "f-16"]) {
        SoundSet::F16
    } else if id == "aerolink" || has(&["atr"]) {
        SoundSet::Atr
    } else if id == "meridian190" || has(&["e190", "ejet", "e-jet", "embraer"]) {
        SoundSet::V2500
    } else if id == "meridian320" || has(&["a320", "a32"]) {
        SoundSet::Cfm56
    } else if id == "vulcan" || has(&["vulcan"]) {
        SoundSet::Olympus
    } else if id == "galaxy" || has(&["c5", "c-5", "galaxy"]) {
        // CF6-80C2 (big-fan turbofan): nearest recorded set.
        SoundSet::Cfm56
    } else if id == "concorde" {
        // Olympus 593: the Vulcan's Olympus 301 grown up — same recordings.
        SoundSet::Olympus
    } else if id == "triple7" || has(&["777", "b777"]) {
        // GE90-115B: the big-fan CFM56 set, lower blade-pass tone.
        SoundSet::Cfm56
    } else if id == "crj700" || id == "citationx" || has(&["crj", "citation"]) {
        // CF34-8C / AE3007C: small high-bypass fans, the E-Jet's (CF34) set.
        SoundSet::V2500
    } else if id == "hercules" || has(&["c130", "c-130", "hercules"]) {
        // Allison T56: nearest recorded turboprop set (PW100), 4-blade props.
        SoundSet::Atr
    } else if matches!(id.as_str(), "dc3" | "otter") || has(&["dc-3", "dakota", "dhc3", "otter"]) {
        // P&W R-1830 Twin Wasp (DC-3 recordings); the Otter's R-1340 Wasp is the same family.
        SoundSet::Radial
    } else if id == "extra500" {
        // Rolls-Royce 250 turboprop: nearest recorded set (PW100), 5-blade prop.
        SoundSet::Atr
    } else {
        match p.family {
            Family::LightPiston => SoundSet::C172,
            Family::Warbird => SoundSet::Merlin,
            Family::Fighter => SoundSet::F16,
            Family::Turboprop => SoundSet::Atr,
            Family::Airliner => SoundSet::Cfm56,
            // Rotorcraft: procedural (rotorcraft.rs).
            Family::Glider | Family::Helicopter | Family::Drone => SoundSet::None,
        }
    };
    match def.category {
        Category::Helicopter => p.family = Family::Helicopter,
        Category::Drone => p.family = Family::Drone,
        _ => {}
    }
    if matches!(p.family, Family::Helicopter | Family::Drone) {
        p.set = SoundSet::None;
    }
    match def.id {
        "kestrel" => {
            p.cylinders = 4;
            p.blades = 2;
        }
        "peregrine" => {
            p.cylinders = 12;
            p.blades = 4;
        }
        "aerolink" => p.blades = 6,
        "lancer" => p.fan_blades = 23,
        "meridian190" => p.fan_blades = 28,
        "meridian320" => p.fan_blades = 36,
        "hercules" => p.blades = 4,
        "galaxy" => p.fan_blades = 38,
        "vulcan" => p.fan_blades = 22,
        "triple7" => p.fan_blades = 22,
        "crj700" => p.fan_blades = 28,
        "citationx" => p.fan_blades = 24,
        "concorde" => p.fan_blades = 22,
        // Track H2b.
        "extra500" => p.blades = 5,
        "cub" => {
            p.cylinders = 4;
            p.blades = 2;
        }
        "zlin" => {
            p.cylinders = 6;
            p.blades = 3;
        }
        "otter" => {
            p.cylinders = 9;
            p.blades = 3;
        }
        "dc3" => {
            p.cylinders = 14;
            p.blades = 3;
        }
        // EC130 B4: 3-blade Starflex main rotor (Ø 10.69 m), 10-blade Fenestron.
        "ec130" => {
            p.blades = 3;
            p.fan_blades = 10;
            p.prop_radius = 5.345;
        }
        _ => match p.family {
            Family::Helicopter => {
                p.blades = 3;
                p.fan_blades = 10;
                p.prop_radius = (def.wingspan * 0.5).max(2.0);
            }
            Family::Drone => {
                // 5-inch tri-blade props on 2207 motors (7 pole pairs).
                p.blades = 3;
                p.cylinders = 7;
                p.prop_radius = 0.0635;
                p.thrust_max = 12.0;
            }
            Family::Warbird => {
                p.cylinders = 12;
                p.blades = 4;
            }
            Family::Turboprop => p.blades = 6,
            _ => {}
        },
    }
    p
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Prio {
    Callout = 1,
    Advisory = 2,
    Warning = 3,
}

/// Altitude callouts (ft, clip) for the airliners / turboprop.
const CALLOUTS: [(f32, Clip); 12] = [
    (2500.0, Clip::Ft2500),
    (1000.0, Clip::Ft1000),
    (500.0, Clip::Ft500),
    (400.0, Clip::Ft400),
    (300.0, Clip::HundredAbove),
    (200.0, Clip::Minimums),
    (100.0, Clip::Ft100),
    (50.0, Clip::Ft50),
    (40.0, Clip::Ft40),
    (30.0, Clip::Ft30),
    (20.0, Clip::Ft20),
    (10.0, Clip::Ft10),
];

/// Watches the simulation from frame to frame.
pub struct Director {
    now: f64,
    /// Last ATC radio command passed on (atc/ sequence numbers).
    radio_seq: u64,
    started: bool,
    type_id: u32,
    prev_eye: Vec3,
    prev_ac: Vec3,
    ear_vel: Vec3,
    prev_cam: Option<CamMode>,
    prev_on_ground: bool,
    air_time: f32,
    prev_vs: f32,
    prev_speed: f32,
    prev_vel: Vec3,
    prev_gear: f32,
    prev_flaps: f32,
    gear_moving: bool,
    prev_running: [bool; MAX_ENG],
    ab_lit: bool,
    prev_crashed: bool,
    prev_ap: bool,
    prev_ap_warn: f32,
    /// Systems alerts (systems/): last one-shot seen, last continuous chime.
    prev_aural: u32,
    last_loop: f64,
    /// Mission cabin PA / windshear warning sequence numbers (missions/).
    prev_pa: u32,
    prev_shear: u32,
    g_slow: f32,
    shake: f32,
    last_thump: f64,
    // speech
    queue: Vec<(Clip, Prio, f64)>,
    busy_until: f64,
    armed: [bool; CALLOUTS.len()],
    retard_said: bool,
    v1_said: bool,
    last_warn: [f64; 8],
    // glider vario
    te_prev: Option<(f32, f32)>,
    vario: f32,
    // thunder
    next_thunder: f64,
    /// Thunder from the storms' actual flashes (wx/), arriving after distance / c.
    thunder_q: crate::wx::ThunderQueue,
    rng: u32,
}

impl Default for Director {
    fn default() -> Self {
        Director {
            radio_seq: 0,
            now: 0.0,
            started: false,
            type_id: 0,
            prev_eye: Vec3::ZERO,
            prev_ac: Vec3::ZERO,
            ear_vel: Vec3::ZERO,
            prev_cam: None,
            prev_on_ground: true,
            air_time: 0.0,
            prev_vs: 0.0,
            prev_speed: 0.0,
            prev_vel: Vec3::ZERO,
            prev_gear: 1.0,
            prev_flaps: 0.0,
            gear_moving: false,
            prev_running: [true; MAX_ENG],
            ab_lit: false,
            prev_crashed: false,
            prev_ap: false,
            prev_ap_warn: 0.0,
            prev_aural: 0,
            last_loop: -10.0,
            prev_pa: 0,
            prev_shear: 0,
            g_slow: 1.0,
            shake: 0.0,
            last_thump: -10.0,
            queue: Vec::new(),
            busy_until: 0.0,
            armed: [false; CALLOUTS.len()],
            retard_said: false,
            v1_said: false,
            last_warn: [-100.0; 8],
            te_prev: None,
            vario: 0.0,
            next_thunder: 20.0,
            thunder_q: Default::default(),
            rng: 0x1234_5678,
        }
    }
}

/// Rotor sound drivers from the rotorcraft model (`src/rotor`).
///
/// Blade slap is blade-vortex interaction: each blade meets the tip vortex
/// shed by the one ahead when the wake stays in the tip-path plane — the
/// net flow through the disc (climb component + induced velocity) near zero
/// at moderate advance ratio: shallow approach descents and decelerations.
/// Strength grows with the vortex strength (blade loading). High-g turns
/// (advancing-tip loading) and the vortex ring add their own impulsive
/// noise.
pub fn rotor_frame(def: &AircraftDef, ac: &crate::flight::Aircraft, p: &Profile) -> RotorFrame {
    let o = &ac.rotor.out;
    if def.rotor.is_none() {
        return RotorFrame::default();
    }
    let (nr_nom, power_ref) = match def.rotor.as_deref().and_then(|r| r.heli()) {
        Some(h) => (h.omega * 60.0 / std::f32::consts::TAU, h.p_mgb),
        None => (o.nr_rpm.max(1.0) / (o.nr_pct.max(1.0) / 100.0), 1.0),
    };
    let nr = o.nr_rpm / nr_nom.max(1.0);
    let vt = (o.nr_rpm / 60.0 * std::f32::consts::TAU * p.prop_radius).max(1.0);
    let tas = ac.tas();
    // Disc tilt forward (rad): shaft tilt + flapping, less the nose-up attitude.
    let tilt = 0.035 - ac.pitch() - o.tilt_deg[0].to_radians();
    let w = ac.vertical_speed() * tilt.cos() + tas * tilt.sin();
    let net = w + o.v_induced;
    let load = (o.thrust / (def.mass * GRAVITY)).clamp(0.0, 2.5);
    let mu = o.mu;
    let bvi = (-(net / 3.0).powi(2)).exp() * smoothstep(0.05, 0.12, mu) * (1.0 - smoothstep(0.3, 0.42, mu)) * load.powf(1.5);
    let turn = smoothstep(1.3, 1.9, ac.g_load()) * smoothstep(0.1, 0.22, mu);
    let spin = smoothstep(0.45, 0.8, nr);
    let slap = if ac.on_ground || ac.crashed.is_some() { 0.0 } else { ((1.15 * bvi + 0.6 * turn + 0.45 * o.vrs) * spin).min(1.3) };
    let _ = vt;
    RotorFrame {
        nr_rpm: o.nr_rpm,
        nr,
        ng: o.ng_pct / 100.0,
        torque: o.torque_pct / 100.0,
        power: o.power_w / power_ref.max(1.0),
        tail_rpm: o.tail_rpm,
        tail_pitch: o.tail_pitch_deg,
        slap,
        load,
        vrs: o.vrs,
        stall: o.stall,
        azimuth: o.azimuth / std::f32::consts::TAU,
    }
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

impl Director {
    fn rand(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng >> 8) as f32 / 16_777_216.0
    }

    fn say(&mut self, c: Clip, p: Prio) {
        if !self.queue.iter().any(|(q, _, _)| *q == c) {
            self.queue.push((c, p, self.now));
        }
    }

    /// Repeatable warning `slot`, at most every `every` seconds.
    fn warn(&mut self, slot: usize, every: f64, c: Clip) {
        if self.now - self.last_warn[slot] >= every {
            self.last_warn[slot] = self.now;
            self.say(c, Prio::Warning);
        }
    }

    /// Build this frame's audio state. `dt` is the real frame time (s);
    /// `menu` = the menu is open over the running sim.
    pub fn capture(&mut self, sim: &Sim, world: &World, dt: f32, menu: bool, events: &mut Vec<Event>) -> Frame {
        let dt = dt.clamp(1e-4, 0.25);
        self.now += dt as f64;
        let def = sim.def();
        let ac = &sim.ac;
        let profile = profile_for(def, sim.size);
        let frozen = sim.paused || menu;
        let warp = if frozen { 0.0 } else { sim.warp.max(1) as f32 };

        // ---- listener
        let (pos, rot) = sim.render_pose();
        let (eye, right, up, fwd) = match &sim.last_view {
            Some(v) => {
                let inv = v.view.inverse();
                (v.eye, inv.x_axis.truncate(), inv.y_axis.truncate(), -inv.z_axis.truncate())
            }
            None => (pos, rot * Vec3::NEG_X, rot * Vec3::Y, rot * Vec3::Z),
        };
        let new_type = profile.type_id != self.type_id;
        // Floating origin: aircraft and camera jump together by the same
        // large offset — shift the audio geometry, don't treat it as a cut.
        let dac = pos - self.prev_ac;
        let shift = if self.started && !new_type && dac.length() > 300.0 && ((eye - self.prev_eye) - dac).length() < 30.0 { dac } else { Vec3::ZERO };
        self.prev_eye += shift;
        self.prev_ac = pos;
        let predicted = self.prev_eye + self.ear_vel * dt;
        let jump = eye.distance(predicted);
        let cut = !self.started || new_type || self.prev_cam != Some(sim.cam) || jump > 12.0 + ac.vel.length() * dt * 0.5;
        if cut {
            self.ear_vel = if sim.cam == CamMode::Cockpit || sim.cam == CamMode::Chase { ac.vel * warp } else { Vec3::ZERO };
        } else {
            let v = (eye - self.prev_eye) / dt;
            self.ear_vel += (v - self.ear_vel) * (dt / 0.06).min(1.0);
        }
        self.prev_eye = eye;
        self.prev_cam = Some(sim.cam);
        let ear_xz = Vec2::new(eye.x, eye.z);
        let ear_agl = eye.y - world.ground_height(ear_xz);

        if new_type {
            // Decode this aircraft's recorded engine set (and the shared
            // airframe / impact samples) off the audio thread.
            if let Some(set) = super::soundsets::set(profile.set) {
                super::samples::prepare(set.samples);
            }
            super::samples::prepare(super::soundsets::COMMON);
            // New aircraft / respawn of another type: reset all edge detectors.
            *self = Director { now: self.now, prev_eye: eye, prev_ac: pos, prev_cam: Some(sim.cam), next_thunder: self.now + 15.0, radio_seq: self.radio_seq, ..Default::default() };
            self.type_id = profile.type_id;
            self.prev_on_ground = ac.on_ground;
            self.prev_gear = ac.gear_pos();
            self.prev_flaps = ac.surfaces().flaps;
            for i in 0..MAX_ENG {
                self.prev_running[i] = ac.engine_running(i);
            }
            self.prev_ap = sim.ap.engaged;
            self.prev_crashed = ac.crashed.is_some();
        }
        self.started = true;

        // ---- aircraft state
        let surf = ac.surfaces();
        let n = profile.engines;
        let mut eng = [EngineFrame::default(); MAX_ENG];
        for (i, e) in eng.iter_mut().enumerate().take(n) {
            let off = def.engine_pos.get(i).copied().unwrap_or(Vec3::ZERO);
            *e = EngineFrame {
                pos: pos + rot * off,
                rpm: ac.engine_rpm_i(i),
                spool: ac.n1(i),
                thrust: ac.engine_thrust(i) / profile.thrust_max,
                ab: if def.has_afterburner { ac.afterburner() } else { 0.0 },
                running: ac.engine_running(i) && ac.crashed.is_none(),
                throttle: sim.ctl.throttle,
            };
        }
        let crashed = ac.crashed.is_some();
        let atm = crate::flight::isa(ac.pos.y);
        let tas = ac.tas();
        let qbar = 0.5 * atm.density * tas * tas;
        let gs = Vec2::new(ac.vel.x, ac.vel.z).length();
        let vs = ac.vertical_speed();
        let ias_kt = ac.airspeed_kt();
        let agl = ac.agl(world);
        let ra_ft = (agl - def.gear_height).max(0.0) / FT;

        // Buffet: stall approach, Mach, gear / flaps over their limits.
        let peak = def.alpha_peak(surf.flaps);
        let stall_buf = smoothstep(peak - 0.06, peak + 0.05, ac.alpha()) * smoothstep(0.3, 0.8, ias_kt / def.vs1_kt.max(10.0));
        let mach_buf = if def.mmo > 0.0 && profile.family != Family::Fighter { smoothstep(def.mmo, def.mmo + 0.07, ac.mach()) } else { 0.0 };
        let g_buf = smoothstep(def.g_limit_pos * 0.7, def.g_limit_pos, ac.g_load());
        let buffet = if ac.on_ground || crashed { 0.0 } else { stall_buf.max(mach_buf).max(g_buf * 0.6) };
        let mut overspeed = ((ias_kt - def.vne_kt) / (0.08 * def.vne_kt.max(50.0))).clamp(0.0, 1.0);
        if def.mmo > 0.0 {
            overspeed = overspeed.max(((ac.mach() - def.mmo) / 0.05).clamp(0.0, 1.0));
        }
        let gear = ac.gear_pos();
        let gear_rate = (gear - self.prev_gear) / dt;
        let flap_rate = (surf.flaps - self.prev_flaps) / dt;

        // Jolt from the load factor: fast deviation from a slow average.
        let g = ac.g_load();
        if !frozen {
            self.g_slow += (g - self.g_slow) * (dt / 0.35).min(1.0);
            let dev = (g - self.g_slow).abs();
            self.shake += (dev * 2.2 - self.shake) * (dt / 0.12).min(1.0);
            if !ac.on_ground && self.air_time > 3.0 && dev > 0.14 && self.now - self.last_thump > 0.35 && !crashed {
                self.last_thump = self.now;
                events.push(Event::Thump(((dev - 0.08) * 1.8).clamp(0.1, 1.0)));
            }
        }

        // Paved or grass under the wheels.
        let xz = Vec2::new(ac.pos.x, ac.pos.z);
        let paved = world.runways.iter().any(|r| seg_dist(xz, r.a, r.b) < r.width * 0.5 + 4.0);

        // ---- discrete events
        if !frozen {
            if ac.on_ground {
                if !self.prev_on_ground && self.air_time > 0.6 && !crashed {
                    events.push(Event::Touchdown { sink: (-self.prev_vs).max(0.0), speed: gs });
                }
                self.air_time = 0.0;
            } else {
                self.air_time += dt * warp;
            }
            if profile.retract_gear {
                let moving = (gear - self.prev_gear).abs() > 1e-5;
                if moving && !self.gear_moving {
                    events.push(Event::GearStart { down: gear > self.prev_gear });
                }
                if !moving && self.gear_moving && (gear <= 0.001 || gear >= 0.999) {
                    events.push(Event::GearLock { down: gear >= 0.999 });
                }
                self.gear_moving = moving;
            }
            if crashed && !self.prev_crashed {
                let reason = ac.crashed.as_deref().unwrap_or("");
                let r = reason.to_ascii_lowercase();
                let xz = Vec2::new(ac.pos.x, ac.pos.z);
                let in_air = r.contains("structural") || r.contains("overstress");
                let surface = if r.contains("water") || r.contains("ditch") || world.is_water(xz) {
                    Surface::Water
                } else if r.contains("building") {
                    Surface::Building
                } else if world.runways.iter().any(|w| seg_dist(xz, w.a, w.b) < w.width * 0.5 + 4.0) {
                    Surface::Paved
                } else {
                    Surface::Ground
                };
                events.push(Event::Crash { speed: self.prev_speed, vel: self.prev_vel, mass: def.mass, surface, in_air });
            }
            // Wreckage: impacts, scrapes, separations, fires, explosions (debris.rs).
            events.extend(sim.audio_events.iter().copied());
            // ATC / ATIS radio (atc/, audio/radio.rs).
            for (seq, o) in sim.atc.out.iter() {
                if *seq > self.radio_seq {
                    events.push(match *o {
                        crate::atc::RadioOut::Start(id) => Event::Radio(id),
                        crate::atc::RadioOut::Cut(ch) => Event::RadioCut(ch),
                    });
                }
            }
            self.radio_seq = sim.atc.out_seq;
            // (With the systems running, their alerts own the chimes: below.)
            let sys_alerts = ac.sys.init && ac.sys.sd.alerting != crate::systems::defs::Alerting::Annunciators;
            for i in 0..n {
                let r = eng[i].running;
                let practice = sim.ctl.failed_engines & (1 << i) != 0;
                let commanded = ac.sys.init && (!ac.sys.sw.eng_master[i] || ac.sys.eng[i].dead);
                if !r && self.prev_running[i] && !crashed && (practice || !(sys_alerts || commanded)) {
                    events.push(Event::MasterCaution);
                    if n > 1 {
                        self.say(Clip::EngineFailure, Prio::Warning);
                    }
                }
                self.prev_running[i] = r;
            }
            let ab = eng[0].ab;
            if def.has_afterburner && ab > 0.02 && !self.ab_lit {
                events.push(Event::AbLight(0));
                self.ab_lit = true;
            } else if ab < 0.005 {
                self.ab_lit = false;
            }
            let ap_dropped = self.prev_ap && !sim.ap.engaged && sim.bot.is_none();
            if (ap_dropped || sim.ap_warn > self.prev_ap_warn + 0.5) && def.has_engine {
                events.push(Event::ApDisconnect);
            }
            self.prev_ap = sim.ap.engaged;
            self.prev_ap_warn = sim.ap_warn;
            // Systems alerts: one-shot chimes / voice, continuous CRC or fire bell.
            let sys = &ac.sys;
            if sys.aural_seq != self.prev_aural {
                self.prev_aural = sys.aural_seq;
                match sys.aural {
                    crate::systems::Aural::Chime => events.push(Event::MasterCaution),
                    crate::systems::Aural::EngineFire => self.say(Clip::EngineFire, Prio::Warning),
                    crate::systems::Aural::Warning => self.say(Clip::Warning, Prio::Warning),
                    crate::systems::Aural::Caution => self.say(Clip::Caution, Prio::Warning),
                    crate::systems::Aural::Smoke => self.say(Clip::Smoke, Prio::Warning),
                    _ => {}
                }
            }
            // Missions: the cabin PA (its own channel) and the windshear warning.
            if let Some(m) = sim.mission.as_ref() {
                if m.pa_seq != self.prev_pa {
                    self.prev_pa = m.pa_seq;
                    if let Some(c) = Clip::pa(&m.pa) {
                        events.push(Event::Speak(c));
                    }
                }
                if m.shear_seq != self.prev_shear {
                    self.prev_shear = m.shear_seq;
                    self.say(Clip::Windshear, Prio::Warning);
                }
                if m.shear && !crashed {
                    self.warn(7, 4.0, Clip::Windshear);
                }
            }
            if !crashed && self.now - self.last_loop >= 1.0 {
                match sys.aural_loop {
                    crate::systems::Aural::Crc => {
                        events.push(Event::Crc);
                        self.last_loop = self.now;
                    }
                    crate::systems::Aural::Bell => {
                        events.push(Event::FireBell);
                        self.last_loop = self.now;
                    }
                    _ => {}
                }
            }
        }
        self.prev_on_ground = ac.on_ground;
        self.prev_vs = vs;
        self.prev_speed = ac.vel.length();
        self.prev_vel = ac.vel;
        self.prev_gear = gear;
        self.prev_flaps = surf.flaps;
        self.prev_crashed = crashed;

        // ---- callouts / GPWS (airliners and the turboprop)
        let fam = profile.family;
        let airliner = matches!(fam, Family::Airliner | Family::Turboprop);
        if airliner && !frozen && !crashed {
            if ac.on_ground {
                self.armed = [false; CALLOUTS.len()];
                self.retard_said = false;
                if !self.v1_said && sim.ctl.throttle > 0.5 && !sim.ctl.reverse && ias_kt > def.vr_kt - 6.0 && ias_kt < def.vr_kt + 10.0 {
                    self.v1_said = true;
                    self.say(Clip::V1, Prio::Advisory);
                }
                if ias_kt < 30.0 {
                    self.v1_said = false;
                }
            } else {
                for (i, (thr, clip)) in CALLOUTS.iter().enumerate() {
                    if ra_ft > thr + 40.0f32.max(thr * 0.1) {
                        self.armed[i] = true;
                    } else if self.armed[i] && ra_ft <= *thr && vs < -0.3 {
                        self.armed[i] = false;
                        let thrust_up = sim.ctl.throttle > 0.12 && !sim.ctl.reverse;
                        let c = if *thr <= 20.0 && thrust_up && !self.retard_said {
                            self.retard_said = true;
                            Clip::Retard
                        } else {
                            *clip
                        };
                        // A newer altitude callout supersedes stale ones.
                        self.queue.retain(|(_, p, _)| *p != Prio::Callout);
                        self.say(c, Prio::Callout);
                    }
                }
                let descent_fpm = -vs / FT * 60.0;
                if ra_ft > 50.0 && ra_ft < 2450.0 {
                    if descent_fpm > 1800.0 + 1.8 * ra_ft {
                        self.warn(0, 1.6, Clip::PullUp);
                    } else if descent_fpm > 1200.0 + 1.4 * ra_ft {
                        self.warn(1, 2.6, Clip::SinkRate);
                    }
                }
                if ra_ft < 500.0 && ra_ft > 30.0 && gear < 0.99 && vs < -1.0 && ias_kt < 190.0 {
                    self.warn(2, 3.5, Clip::TooLowGear);
                } else if ra_ft < 245.0 && ra_ft > 30.0 && gear >= 0.99 && surf.flaps < 0.12 && vs < -1.0 && ias_kt < 160.0 {
                    self.warn(3, 3.5, Clip::TooLowFlaps);
                }
                if ac.roll().abs() > 35f32.to_radians() && ra_ft > 30.0 {
                    self.warn(4, 3.0, Clip::BankAngle);
                }
                if ac.stall_warning() {
                    self.warn(5, 1.6, Clip::Stall);
                }
                if overspeed > 0.0 {
                    self.warn(6, 4.0, Clip::Overspeed);
                }
            }
        }
        if crashed {
            self.queue.clear();
        }
        // Speak the most important pending line once the previous one is done.
        if self.now >= self.busy_until && !self.queue.is_empty() {
            let now = self.now;
            self.queue.retain(|(_, p, t)| *p != Prio::Callout || now - t < 1.2);
            if let Some(i) = (0..self.queue.len()).max_by(|&a, &b| self.queue[a].1.cmp(&self.queue[b].1).then(self.queue[b].2.total_cmp(&self.queue[a].2))) {
                let (c, _, _) = self.queue.remove(i);
                events.push(Event::Speak(c));
                self.busy_until = self.now + voice::clip_seconds(c) as f64 + 0.12;
            }
        }

        // ---- gear warning horn (warbird / fighter; airliners get the voice)
        let gear_warn = profile.retract_gear
            && !airliner
            && !ac.on_ground
            && !crashed
            && gear < 0.99
            && agl < 160.0
            && vs < -0.5
            && (sim.ctl.throttle < 0.25 || surf.flaps > 0.6);

        // ---- glider vario (total energy)
        let mut vario = None;
        if !def.has_engine {
            let e = ac.pos.y + tas * tas / (2.0 * GRAVITY);
            if let Some((pe, pt)) = self.te_prev {
                let dts = sim.time - pt;
                if dts > 1e-4 {
                    let raw = ((e - pe) / dts).clamp(-15.0, 15.0);
                    self.vario += (raw - self.vario) * (dts / 0.9).min(1.0);
                }
            }
            self.te_prev = Some((e, sim.time));
            if !ac.on_ground && !crashed {
                vario = Some(self.vario);
            }
        }

        // (Sonic booms: the synth derives them from the source / listener
        // geometry — see Synth::sonic_boom.)
        let interior = if sim.cam == CamMode::Cockpit { 1.0 } else { 0.0 };
        let m = ac.mach();
        let rotor = rotor_frame(def, ac, &profile);
        let rotorcraft = matches!(fam, Family::Helicopter | Family::Drone);

        // ---- weather
        let wr = world.render_weather();
        if world.wxs.storms.active() && world.wxs.storms.spec.flash_rate > 0.0 {
            // Each flash's thunder reaches the listener after distance / c:
            // a sharp crack close by, a long low rumble from far away.
            for th in self.thunder_q.update(&world.wxs.flashes, eye, world.wxs.time) {
                if !frozen {
                    let pan = th.dir.dot(right).clamp(-1.0, 1.0);
                    events.push(Event::ThunderRoll { km: th.km, rumble: th.rumble, energy: th.energy, pan, ground: th.ground });
                }
            }
        } else if world.wx.thunder && !frozen {
            if self.now > self.next_thunder {
                let km = 1.0 + self.rand() * 9.0;
                events.push(Event::Thunder(km));
                self.next_thunder = self.now + 12.0 + self.rand() as f64 * 45.0;
            }
        } else {
            self.next_thunder = self.now + 8.0;
        }
        let h = sim.hours;
        let day = smoothstep(5.0, 6.5, h) * (1.0 - smoothstep(19.5, 21.0, h));

        Frame {
            active: true,
            paused: sim.paused,
            menu,
            warp: sim.warp.max(1) as f32,
            profile,
            ear_pos: eye,
            ear_vel: if frozen { Vec3::ZERO } else { self.ear_vel },
            ear_right: right.normalize_or(Vec3::X),
            ear_up: up.normalize_or(Vec3::Y),
            ear_fwd: fwd.normalize_or(Vec3::NEG_Z),
            interior,
            cut,
            shift,
            ear_agl,
            ac_pos: pos,
            ac_vel: ac.vel * warp,
            ac_rot: rot,
            eng,
            rotor,
            reverser: surf.reverser,
            tas,
            mach: m,
            qbar,
            beta: ac.beta(),
            buffet: if rotorcraft { (rotor.stall + 0.6 * rotor.vrs).min(1.0) } else { buffet },
            overspeed,
            gear: if profile.retract_gear { gear } else { 1.0 },
            flaps: surf.flaps,
            spoilers: surf.spoilers,
            gear_rate: if frozen { 0.0 } else { gear_rate },
            flap_rate: if frozen { 0.0 } else { flap_rate },
            on_ground: ac.on_ground,
            ground_speed: if ac.on_ground { gs } else { 0.0 },
            paved,
            brakes: sim.ctl.brakes,
            shake: self.shake.min(1.5),
            crashed,
            // Helicopters: the low rotor-RPM horn instead of a stall warning.
            stall_warn: !crashed && if fam == Family::Helicopter { ac.rotor.out.low_nr } else { !rotorcraft && ac.stall_warning() },
            gear_warn,
            vario,
            ident: sim.nav.radios.ident_audio().map(|(id, _)| ident_bytes(id)).unwrap_or_default(),
            ident_gain: sim.nav.radios.ident_audio().map_or(0.0, |(_, g)| g),
            rain: wr.rain,
            snow: wr.snow,
            wind_surface: world.wind.surface.length(),
            gust: world.wind.gust,
            thunder: world.wx.thunder,
            day,
            urban: world.roughness(ear_xz),
            traffic: Default::default(),
            spectator: false,
        }
    }
}

/// A navaid ident as the audio frame carries it (up to 4 letters).
fn ident_bytes(id: &str) -> [u8; 4] {
    let mut b = [0u8; 4];
    for (k, c) in id.bytes().filter(|c| c.is_ascii_alphanumeric()).take(4).enumerate() {
        b[k] = c.to_ascii_uppercase();
    }
    b
}
