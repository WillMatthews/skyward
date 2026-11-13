//! Air traffic control on the radio: ATIS and the delivery / ground /
//! tower / approach (director, radar) / departure units of the airports
//! around, on their real frequencies (OurAirports, [`freqs`]). Tune COM 1 to
//! talk; the comms menu (`ui/comms.rs`) offers the requests that make sense
//! for where you are and who you are talking to. Phraseology follows the UK
//! CAP 413 / ICAO Doc 4444 style ([`phrase`]); transmissions are assembled
//! from pre-rendered voice clips and played through the radio chain
//! (`audio/radio.rs`) with subtitles.
//!
//! * **ATIS** ([`atis`]): a looping broadcast from the live METAR, its letter
//!   advancing when the weather or the runway in use changes.
//! * **Departure**: IFR / VFR clearance with a squawk, start-up, taxi to the
//!   runway-holding point over the airport's taxiway graph ([`taxi`]),
//!   line-up and take-off clearance with the surface wind, the hand-off to
//!   departure / radar once airborne, "identified", climb, own navigation.
//! * **Arrival**: radar vectors to a 30° intercept of the ILS
//!   ([`vectors`]) with descent and the approach clearance, the hand-off to
//!   tower when established, landing clearance, vacate, taxi in.
//! * **Deviations**: altitude busts, runway incursions, take-off or landing
//!   without a clearance (stop immediately / go around), squawk 7700.
//! * **Emergencies**: MAYDAY / PAN PAN with the failure from the systems, 7700,
//!   vectors to the nearest ILS.
//! * The pilot's read-backs (automatic, or from the menu) in a voice of their
//!   own; optional assists tune the next frequency and set the squawk. The
//!   auto-flight bot does its own radio work in watch mode.

pub mod atis;
pub mod freqs;
pub mod phrase;
pub mod taxi;
#[cfg(test)]
mod tests;
pub mod traffic;
pub mod vectors;

use crate::audio::radio::{self, Voice};
use crate::flight::{Aircraft, AircraftDef, Category};
use crate::nav::NavState;
use crate::world::World;
use freqs::Fac;
use glam::{Vec2, vec2};
use phrase::{Callsign, Utt};
use std::collections::{HashMap, HashSet, VecDeque};

const FT: f32 = 0.3048;
const KT: f32 = 0.514_444;
const NM: f32 = 1852.0;

/// "alpha" → "Alpha".
pub fn cap(s: &str) -> String {
    s.split(' ')
        .map(|w| {
            let mut c = w.chars();
            match c.next() {
                Some(f) => f.to_uppercase().chain(c).collect::<String>(),
                None => String::new(),
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn hash(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3))
}

/// A unit on a frequency.
#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    /// Airport ident (OurAirports) and its ICAO / display code.
    pub airport: String,
    pub code: String,
    pub fac: Fac,
    pub khz: u32,
    /// Spoken / shown callsign ("heathrow tower" / "Heathrow Tower").
    pub spoken: String,
    pub text: String,
    pub ll: (f64, f64),
    pub elev_m: f32,
    pub voice: Voice,
}

impl Station {
    /// The station name without the unit ("Heathrow"), for "contact Heathrow Tower".
    pub fn short_text(&self) -> String {
        self.text.clone()
    }
    /// One unit may serve several airports (Thames Radar): it is known by
    /// its callsign and frequency.
    pub fn key(&self) -> (String, u32) {
        (self.spoken.clone(), self.khz)
    }
}

/// Where the flight is, as ATC sees it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// On the ground, nothing asked for yet.
    Parked,
    /// Has its clearance (IFR / VFR).
    Cleared,
    /// Taxiing to the holding point.
    Taxi,
    /// Lined up, waiting.
    LineUp,
    /// Cleared for take-off.
    Takeoff,
    /// Airborne, handed over / with departure radar.
    Departure,
    /// Own navigation.
    Enroute,
    /// Being vectored for an approach.
    Vectors,
    /// Cleared for the approach (or established).
    Approach,
    /// Cleared to land.
    Landing,
    /// On the ground after landing, vacating.
    Landed,
    /// Taxiing in.
    TaxiIn,
}

/// A pilot request (comms menu items).
#[derive(Clone, Debug, PartialEq)]
pub enum Req {
    /// IFR (or, for light aircraft without a destination, VFR) clearance.
    Clearance,
    StartUp,
    Taxi,
    Ready,
    /// First call after a hand-off (airborne).
    CheckIn,
    RequestIls,
    RequestVisual,
    Established,
    FieldInSight,
    Final,
    GoingAround,
    Vacated,
    Mayday,
    PanPan,
    SayAgain,
    /// Read back the last instruction (manual read-backs).
    Readback,
    /// Tune a station: index into [`Atc::stations`], on COM 2 if true.
    Tune(usize, bool),
}

/// Pilot-side effects applied after the read-back (assists).
#[derive(Clone, Debug, PartialEq)]
enum Fx {
    /// Tune COM 1 active (the old frequency goes to standby).
    Tune(u32),
    Squawk(u32),
    /// The bot may go (take-off clearance read back).
    ReleaseBot,
    /// Autopilot-follows-ATC assist: heading (deg true), altitude (ft), arm APPR.
    Hdg(f32),
    Alt(f32),
    Appr,
}

/// A command for the autopilot (the "autopilot follows ATC" assist).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ApCmd {
    Hdg(f32),
    Alt(f32),
    Appr,
}

#[derive(Clone, Debug)]
enum After {
    None,
    /// The pilot's call finished: the controller answers.
    Reply(Req),
    /// The controller finished: the pilot reads back, then the effects and
    /// an optional follow-up from the controller ("readback correct").
    Readback(Utt, Vec<Fx>, Option<Box<Msg>>),
    /// Effects, then an optional follow-up.
    Done(Vec<Fx>, Option<Box<Msg>>),
}

#[derive(Clone, Debug)]
struct Msg {
    pilot: bool,
    station: Station,
    utt: Utt,
    delay: f32,
    after: After,
}

/// A transmission on the air (subtitles).
#[derive(Clone, Debug, PartialEq)]
pub struct OnAir {
    /// Receiver: 0 = COM 1, 1 = COM 2.
    pub ch: u8,
    /// Who is talking ("Heathrow Tower", "Skyward 247").
    pub who: String,
    pub text: String,
    pub own: bool,
    pub atis: bool,
    pub left: f32,
    pub total: f32,
}

/// Something for the audio director.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RadioOut {
    Start(u32),
    Cut(u8),
}

/// One line of the comms log.
#[derive(Clone, Debug, PartialEq)]
pub struct LogLine {
    pub who: String,
    pub text: String,
    pub own: bool,
}

#[derive(Clone, Debug)]
struct AtisState {
    letter: char,
    digest: String,
    rwy: atis::InUse,
    utt: Utt,
    /// Seconds until the loop restarts (per receiver).
    next: [f32; 2],
    playing: [bool; 2],
}

/// Departure plan: runway, holding point, taxi route (world frame).
#[derive(Clone, Debug, Default)]
pub struct DepPlan {
    pub airport: String,
    pub runway: String,
    pub hold: Option<taxi::HoldPt>,
    pub route: Vec<Vec2>,
    pub crosses: Vec<String>,
    /// Taxiway refs along the route.
    pub via: Vec<String>,
}

/// Arrival: the approach being flown.
#[derive(Clone, Debug)]
pub struct ArrPlan {
    pub airport: String,
    pub runway: String,
    pub ils: bool,
    pub director: Option<vectors::Director>,
}

/// The ATC session of one flight.
pub struct Atc {
    pub callsign: Callsign,
    /// The type has a radio (and ATC is on).
    pub enabled: bool,
    /// Read back automatically (else from the menu).
    pub auto_readback: bool,
    /// Tune the next frequency and set the squawk on read-back.
    pub assist: bool,
    /// The bot does the radio work when it flies.
    pub bot_comms: bool,
    /// The autopilot flies the read-back headings / altitudes and arms the approach.
    pub ap_follow: bool,
    /// Commands for the autopilot (taken by `step`).
    pub ap_cmds: Vec<ApCmd>,
    /// COM 2 receiver selected: off at the start (COM 2 is pre-tuned to the
    /// ATIS), on when you tune COM 2; deselected once you talk to ATC (assist).
    pub com2_rx: bool,
    /// Requests made while the frequency was busy: sent when it is free.
    deferred: VecDeque<Req>,
    pub phase: Phase,
    /// Stations in range of the area (refreshed as the aircraft moves).
    pub stations: Vec<Station>,
    stations_at: Option<(f64, f64)>,
    stations_t: f32,
    pub squawk: Option<u32>,
    pub cleared_alt_ft: Option<f32>,
    pub dep: Option<DepPlan>,
    pub arr: Option<ArrPlan>,
    pub emergency: Option<&'static str>,
    contacted: HashSet<(String, u32)>,
    /// Units that have said "identified" / their name to us.
    replied: HashSet<(String, u32)>,
    queue: VecDeque<Msg>,
    /// What is on the air now (both receivers), for subtitles.
    pub on_air: Vec<OnAir>,
    /// Seconds until COM 1 is free (a transmission in progress).
    busy: f32,
    /// Seconds since COM 1 went quiet.
    quiet: f32,
    current: Option<Msg>,
    pub log: VecDeque<LogLine>,
    /// Audio commands with sequence numbers (the audio director tracks the last one it saw).
    pub out: VecDeque<(u64, RadioOut)>,
    pub out_seq: u64,
    atis: HashMap<String, AtisState>,
    last_ctl: HashMap<String, Utt>,
    /// Manual read-back waiting.
    pub pending_readback: Option<(Utt, Station)>,
    pending_fx: Vec<Fx>,
    pending_readback_fx: Vec<Fx>,
    pending_then: Option<Box<Msg>>,
    /// Deviation call cooldowns (s).
    warn_t: [f32; 4],
    bust_t: f32,
    /// The cleared level the aircraft has reached (busts count from there).
    level_reached: Option<f32>,
    pub bot_hold: bool,
    bot_phase: String,
    tuned_once: bool,
    pub time: f32,
    rng: u32,
    /// The taxi graph of the airport we're at (built lazily).
    graph: Option<(String, u64, taxi::Graph)>,
    lineup_t: f32,
    land_t: f32,
    /// COM 1 / 2 frequencies last frame (detuning cuts the receiver).
    last_khz: [u32; 2],
    /// UTC now (hours, minutes).
    utc: (u32, u32),
    /// Traffic information (`traffic.rs`): who was called (seconds ago, negative), next check, sequencing done.
    traffic_seen: HashMap<u16, f32>,
    traffic_check: f32,
    seq_given: bool,
}

impl Default for Atc {
    fn default() -> Self {
        Atc::new_for("meridian320", Category::Airliner, true)
    }
}

/// What the session needs from the sim each frame.
pub struct Ctx<'a> {
    pub world: &'a World,
    pub ac: &'a Aircraft,
    pub def: &'a AircraftDef,
    pub ground_y: f32,
    /// Bot phase name and its runway (when the bot flies).
    pub bot: Option<(String, Option<crate::bot::RunwayEnd>)>,
    /// UTC (hours, minutes).
    pub utc: (u32, u32),
    /// Destination ICAO from the flight plan.
    pub dest: Option<String>,
    pub paused: bool,
}

impl Atc {
    pub fn new(def: &AircraftDef) -> Atc {
        Atc::new_for(def.id, def.category, crate::nav::state::has_radios(def))
    }

    pub fn new_for(id: &str, cat: Category, radios: bool) -> Atc {
        Atc {
            callsign: Callsign::for_type(id, cat),
            enabled: radios,
            auto_readback: true,
            assist: true,
            bot_comms: true,
            ap_follow: false,
            ap_cmds: Vec::new(),
            com2_rx: false,
            deferred: VecDeque::new(),
            phase: Phase::Parked,
            stations: Vec::new(),
            stations_at: None,
            stations_t: 0.0,
            squawk: None,
            cleared_alt_ft: None,
            dep: None,
            arr: None,
            emergency: None,
            contacted: HashSet::new(),
            replied: HashSet::new(),
            queue: VecDeque::new(),
            on_air: Vec::new(),
            busy: 0.0,
            quiet: 10.0,
            current: None,
            log: VecDeque::new(),
            out: VecDeque::new(),
            out_seq: 0,
            atis: HashMap::new(),
            last_ctl: HashMap::new(),
            pending_readback: None,
            pending_fx: Vec::new(),
            pending_readback_fx: Vec::new(),
            pending_then: None,
            warn_t: [0.0; 4],
            bust_t: 0.0,
            level_reached: None,
            bot_hold: false,
            bot_phase: String::new(),
            tuned_once: false,
            time: 0.0,
            rng: 0x1234_5678,
            graph: None,
            lineup_t: 0.0,
            land_t: 0.0,
            last_khz: [0; 2],
            utc: (12, 0),
            traffic_seen: HashMap::new(),
            traffic_check: 0.0,
            seq_given: false,
        }
    }

    fn rand(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng >> 8) as f32 / 16_777_216.0
    }

    /// The world origin moved: re-express stored positions.
    pub fn rebase(&mut self, r: &crate::world::Rebase) {
        if let Some(d) = &mut self.dep {
            for p in d.route.iter_mut() {
                *p = r.ground(*p);
            }
            if let Some(h) = &mut d.hold {
                h.at = r.ground(h.at);
            }
        }
        if let Some(d) = self.arr.as_mut().and_then(|a| a.director.as_mut()) {
            d.fin.thr = r.ground(d.fin.thr);
            for w in d.plan.wps.iter_mut() {
                *w = r.ground(*w);
            }
        }
        self.graph = None;
    }

    fn emit(&mut self, o: RadioOut) {
        self.out_seq += 1;
        self.out.push_back((self.out_seq, o));
        while self.out.len() > 32 {
            self.out.pop_front();
        }
    }

    fn log_line(&mut self, who: String, text: String, own: bool) {
        self.log.push_back(LogLine { who, text, own });
        while self.log.len() > 40 {
            self.log.pop_front();
        }
    }

    // ------------------------------------------------------------ stations

    fn refresh_stations(&mut self, world: &World, ll: (f64, f64)) {
        let (Some(adb), Some(fdb)) = (world.airport_db.as_ref(), freqs::db()) else { return };
        let mut v = Vec::new();
        for (i, _) in adb.near(ll.0, ll.1, 150.0) {
            let ap = &adb.airports[i];
            let fs = fdb.at(&ap.ident);
            if fs.is_empty() && !ap.icao.is_empty() {
                v.extend(stations_of(ap, fdb.at(&ap.icao)));
            } else {
                v.extend(stations_of(ap, fs));
            }
        }
        self.stations = v;
        self.stations_at = Some(ll);
    }

    /// Install stations directly (tests).
    pub fn set_stations(&mut self, s: Vec<Station>) {
        self.stations = s;
        self.stations_at = Some((0.0, 0.0));
    }

    /// Distance (m) and signal strength (0..1) of a station from the aircraft; None = out of range.
    fn reception(&self, s: &Station, world: &World, ac_pos: glam::Vec3, ground_y: f32) -> Option<(f32, f32)> {
        let p = world.proj.to_world(s.ll.0, s.ll.1);
        let d = p.distance(vec2(ac_pos.x, ac_pos.z));
        let range = s.fac.range_nm() * NM;
        // Radio horizon: aircraft height above the station + a 15 m antenna.
        let h = (ac_pos.y - s.elev_m).max((ac_pos.y - ground_y).max(2.0)).max(2.0);
        let horizon = 4120.0 * (h.sqrt() + 15f32.sqrt());
        if d > range || d > horizon {
            return None;
        }
        Some((d, (1.0 - (d / range.min(horizon)).powi(2)).clamp(0.05, 1.0)))
    }

    /// The station on a frequency (in range), nearest first.
    pub fn station_on(&self, khz: u32, world: &World, ac: &Aircraft, ground_y: f32) -> Option<(Station, f32)> {
        self.stations
            .iter()
            .filter(|s| s.khz.abs_diff(khz) <= 3)
            .filter_map(|s| self.reception(s, world, ac.pos, ground_y).map(|(d, q)| (s, d, q)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(s, _, q)| (s.clone(), q))
    }

    /// Stations to offer in the "tune" list: in range, nearest airports first.
    pub fn tune_list(&self, world: &World, ac: &Aircraft, ground_y: f32) -> Vec<(usize, f32)> {
        let mut v: Vec<(usize, f32)> = self.stations.iter().enumerate().filter_map(|(i, s)| self.reception(s, world, ac.pos, ground_y).map(|(d, _)| (i, d))).collect();
        v.sort_by(|a, b| {
            let (sa, sb) = (&self.stations[a.0], &self.stations[b.0]);
            let da = world.proj.to_world(sa.ll.0, sa.ll.1).distance(vec2(ac.pos.x, ac.pos.z));
            let db = world.proj.to_world(sb.ll.0, sb.ll.1).distance(vec2(ac.pos.x, ac.pos.z));
            if sa.airport != sb.airport { da.total_cmp(&db) } else { sa.fac.cmp(&sb.fac) }
        });
        v
    }

    /// The station of `airport` with unit `fac` (first of the preferred list).
    pub fn find(&self, airport: &str, facs: &[Fac]) -> Option<Station> {
        facs.iter().find_map(|f| self.stations.iter().find(|s| s.airport == airport && s.fac == *f).cloned())
    }

    // ------------------------------------------------------------ queue

    fn say_ctl(&mut self, st: &Station, utt: Utt, delay: f32, after: After) {
        self.queue.push_back(Msg { pilot: false, station: st.clone(), utt, delay, after });
    }

    fn say_pilot(&mut self, st: &Station, utt: Utt, delay: f32, after: After) {
        self.queue.push_back(Msg { pilot: true, station: st.clone(), utt, delay, after });
    }

    /// Controller message addressed to us: "<callsign>, ..." with a read-back.
    fn instruct(&mut self, st: &Station, body: Utt, readback: Option<Utt>, fx: Vec<Fx>) {
        let m = self.instruct_msg(st, body, readback, fx);
        self.queue.push_back(m);
    }

    fn instruct_msg(&mut self, st: &Station, body: Utt, readback: Option<Utt>, fx: Vec<Fx>) -> Msg {
        let mut u = Utt::new();
        let cs = self.callsign.clone();
        u.say_as(&cs.short_spoken, &cs.short_text).comma();
        merge(&mut u, &body);
        let u = u.finish();
        self.last_ctl.insert(st.text.clone(), u.clone());
        let d = 1.2 + 1.2 * self.rand();
        let after = match readback {
            Some(rb) => {
                let mut r = rb;
                r.comma().say_as(&cs.short_spoken, &cs.short_text);
                After::Readback(r.finish(), fx, None)
            }
            None => After::Done(fx, None),
        };
        Msg { pilot: false, station: st.clone(), utt: u, delay: d, after }
    }

    /// The pilot's call: "<station>, <callsign>, ..." on first contact, else "<callsign>, ...".
    fn call(&mut self, st: &Station, body: Utt, req: Req) {
        if self.assist {
            self.com2_rx = false;
        }
        let first = !self.contacted.contains(&st.key());
        let cs = self.callsign.clone();
        let mut u = Utt::new();
        if first {
            u.say_as(&st.spoken, &st.text).comma().say_as(&cs.full_spoken, &cs.full_text);
            self.contacted.insert(st.key());
        } else {
            u.say_as(&cs.short_spoken, &cs.short_text);
        }
        if !body.is_empty() {
            u.comma();
            merge(&mut u, &body);
        }
        self.say_pilot(st, u.finish(), 0.3, After::Reply(req));
    }

    // ------------------------------------------------------------ menu

    /// The requests that make sense now on COM 1: (request, label).
    pub fn menu(&self, world: &World, ac: &Aircraft, ground_y: f32, khz: u32) -> Vec<(Req, String)> {
        let mut v: Vec<(Req, String)> = Vec::new();
        if !self.enabled {
            return v;
        }
        if let Some((rb, _)) = &self.pending_readback {
            v.push((Req::Readback, format!("Read back: “{}”", short(&rb.text, 48))));
        }
        let Some((st, _)) = self.station_on(khz, world, ac, ground_y) else {
            return v;
        };
        let ground = ac.on_ground;
        let p = self.phase;
        let first = !self.contacted.contains(&st.key());
        match st.fac {
            Fac::Atis => {}
            Fac::Delivery | Fac::Ground | Fac::Tower if ground => {
                let deliv = st.fac == Fac::Delivery || self.find(&st.airport, &[Fac::Delivery]).is_none();
                if deliv && p == Phase::Parked && st.fac != Fac::Tower
                    || (st.fac == Fac::Tower && p == Phase::Parked && self.find(&st.airport, &[Fac::Ground, Fac::Delivery]).is_none())
                {
                    let vfr = self.vfr_default();
                    v.push((Req::Clearance, if vfr { "Request VFR departure".into() } else { "Request IFR clearance".into() }));
                }
                if st.fac != Fac::Tower && matches!(p, Phase::Parked | Phase::Cleared) && self.callsign_is_airline() {
                    v.push((Req::StartUp, "Request start-up".into()));
                }
                if (st.fac == Fac::Ground || (st.fac == Fac::Tower && self.find(&st.airport, &[Fac::Ground]).is_none())) && matches!(p, Phase::Parked | Phase::Cleared) {
                    v.push((Req::Taxi, "Request taxi".into()));
                }
                if matches!(p, Phase::Taxi | Phase::Parked | Phase::Cleared | Phase::LineUp) && st.fac != Fac::Delivery {
                    v.push((Req::Ready, "Ready for departure".into()));
                }
                if matches!(p, Phase::Landed | Phase::Landing) {
                    v.push((Req::Vacated, "Runway vacated".into()));
                }
            }
            _ if ground => {}
            f => {
                if first && matches!(p, Phase::Departure | Phase::Takeoff | Phase::Enroute) && f.is_radar() {
                    v.push((Req::CheckIn, format!("Check in with {}", st.text)));
                }
                if f.is_radar() && !matches!(p, Phase::Vectors | Phase::Approach | Phase::Landing) {
                    v.push((Req::RequestIls, "Request ILS approach".into()));
                    v.push((Req::RequestVisual, "Request visual approach".into()));
                }
                if matches!(p, Phase::Vectors | Phase::Approach) {
                    v.push((Req::Established, "Established on the localiser".into()));
                    v.push((Req::FieldInSight, "Field in sight".into()));
                }
                if f == Fac::Tower && !matches!(p, Phase::Landing) {
                    v.push((Req::Final, "Final — request landing".into()));
                }
                if matches!(p, Phase::Approach | Phase::Landing | Phase::Vectors) {
                    v.push((Req::GoingAround, "Going around".into()));
                }
            }
        }
        if st.fac != Fac::Atis {
            if self.emergency.is_none() {
                v.push((Req::PanPan, "PAN PAN (urgency)".into()));
                v.push((Req::Mayday, "MAYDAY (distress)".into()));
            }
            if self.last_ctl.contains_key(&st.text) {
                v.push((Req::SayAgain, "Say again".into()));
            }
        }
        v
    }

    fn callsign_is_airline(&self) -> bool {
        !self.callsign.full_text.contains('-')
    }

    fn vfr_default(&self) -> bool {
        self.callsign.full_text.contains('-')
    }

    /// Ask for something (from the menu or the bot).
    pub fn request(&mut self, r: Req, nav: &mut NavState, ctx: &Ctx) {
        if let Req::Tune(i, com2) = r {
            if com2 {
                self.com2_rx = true;
            }
            if let Some(s) = self.stations.get(i) {
                let c = &mut nav.radios.com[com2 as usize];
                if c.active != s.khz {
                    c.standby = c.active;
                    c.active = s.khz;
                }
            }
            return;
        }
        if r == Req::Readback {
            if let Some((u, st)) = self.pending_readback.take() {
                let fx = std::mem::take(&mut self.pending_readback_fx);
                let then = self.pending_then.take();
                self.say_pilot(&st, u, 0.2, After::Done(fx, then));
            }
            return;
        }
        // Wait for a gap on the frequency (and for any read-back / tuning in progress).
        if !self.queue.is_empty() || self.current.is_some() {
            if !self.deferred.contains(&r) {
                self.deferred.push_back(r);
            }
            return;
        }
        let khz = nav.radios.com[0].active;
        let Some((st, _)) = self.station_on(khz, ctx.world, ctx.ac, ctx.ground_y) else {
            // Nobody there: the call goes out unanswered.
            let st = Station {
                airport: String::new(),
                code: String::new(),
                fac: Fac::Tower,
                khz,
                spoken: String::new(),
                text: String::new(),
                ll: (0.0, 0.0),
                elev_m: 0.0,
                voice: Voice::Fable,
            };
            let body = self.pilot_body(&r, &st, ctx);
            let cs = self.callsign.clone();
            let mut u = Utt::new();
            u.say_as(&cs.full_spoken, &cs.full_text).comma();
            merge(&mut u, &body);
            self.say_pilot(&st, u.finish(), 0.3, After::None);
            return;
        };
        let body = self.pilot_body(&r, &st, ctx);
        match r {
            Req::Mayday | Req::PanPan => {
                let mut u = Utt::new();
                if r == Req::Mayday {
                    u.say_as("mayday mayday mayday", "MAYDAY MAYDAY MAYDAY").comma();
                } else {
                    u.say_as("pan pan, pan pan, pan pan", "PAN PAN, PAN PAN, PAN PAN").comma();
                }
                let cs = self.callsign.clone();
                u.say_as(&st.spoken, &st.text).comma().say_as(&cs.full_spoken, &cs.full_text).comma();
                merge(&mut u, &body);
                self.contacted.insert(st.key());
                self.say_pilot(&st, u.finish(), 0.2, After::Reply(r));
            }
            Req::SayAgain => {
                let mut u = Utt::new();
                u.say("say again");
                self.call(&st, u, Req::SayAgain);
            }
            _ => self.call(&st, body, r),
        }
    }

    /// What the pilot says after the callsign for a request.
    fn pilot_body(&self, r: &Req, st: &Station, ctx: &Ctx) -> Utt {
        let mut u = Utt::new();
        let first = !self.contacted.contains(&st.key());
        let info = self.atis.get(&st.airport).map(|a| a.letter);
        let ac = ctx.ac;
        let alt_ft = ac.pos.y / FT;
        let tl = self.tl(ctx.world, &st.airport);
        let with_info = |u: &mut Utt| {
            if let Some(l) = info {
                let w = phrase::letter_word(l).unwrap_or("alpha");
                u.comma().say_as(&format!("information {w}"), &format!("information {}", cap(w)));
            }
        };
        match r {
            Req::Clearance => {
                let (ps, pt) = self.position_words(ctx);
                u.say_as(&ps, &pt);
                if first {
                    with_info(&mut u);
                }
                u.comma();
                if self.vfr_default() && ctx.dest.is_none() {
                    u.say_as("request vfr departure", "request VFR departure");
                } else {
                    match ctx.dest.as_deref().and_then(phrase::airport_name) {
                        Some((s, t)) => u.say_as(&format!("request ifr clearance to {s}"), &format!("request IFR clearance to {t}")),
                        None => match &ctx.dest {
                            Some(d) => u.say_as(&format!("request ifr clearance to {}", phrase::spell(d)), &format!("request IFR clearance to {d}")),
                            None => u.say_as("request ifr clearance", "request IFR clearance"),
                        },
                    };
                }
            }
            Req::StartUp => {
                let (ps, pt) = self.position_words(ctx);
                u.say_as(&ps, &pt);
                if first {
                    with_info(&mut u);
                }
                u.comma().say("request start-up");
            }
            Req::Taxi => {
                let (ps, pt) = self.position_words(ctx);
                u.say_as(&ps, &pt);
                if first {
                    with_info(&mut u);
                }
                u.comma().say("request taxi");
            }
            Req::Ready => {
                let on_rwy = self.on_runway(ctx).is_some();
                if let Some(h) = self.dep.as_ref().and_then(|d| d.hold.as_ref()).filter(|h| !on_rwy && h.at.distance(vec2(ac.pos.x, ac.pos.z)) < 150.0 && !h.taxiway.is_empty()) {
                    u.say_as(&format!("holding point {}", phrase::spell(&h.taxiway)), &format!("holding point {}", h.taxiway)).comma();
                }
                u.say("ready for departure");
            }
            Req::CheckIn => {
                let vs = ac.vertical_speed() / FT * 60.0;
                u.say("passing").feet(alt_ft);
                if vs > 400.0 {
                    if let Some(c) = self.cleared_alt_ft {
                        u.say("climbing").level(c, tl);
                    } else {
                        u.say("climbing");
                    }
                } else if vs < -400.0 {
                    u.say("descending");
                }
            }
            Req::RequestIls | Req::RequestVisual => {
                u.level(alt_ft, tl);
                let vs = ac.vertical_speed() / FT * 60.0;
                if vs < -400.0 {
                    u.say("descending");
                }
                if first {
                    with_info(&mut u);
                }
                u.comma();
                if *r == Req::RequestIls {
                    u.say_as("request ils approach", "request ILS approach");
                } else {
                    u.say("request visual approach");
                }
            }
            Req::Established => {
                u.say("established on the localiser");
                if let Some(a) = &self.arr {
                    u.comma().runway(&a.runway);
                }
            }
            Req::FieldInSight => {
                u.say("field in sight");
            }
            Req::Final => {
                u.say("final");
                if let Some(rw) = self.landing_runway(st, ctx) {
                    u.runway(&rw);
                }
            }
            Req::GoingAround => {
                u.say("going around");
            }
            Req::Vacated => {
                u.say("runway vacated");
            }
            Req::Mayday | Req::PanPan => {
                let what = emergency_words(ac);
                u.say(what).comma();
                if ac.on_ground {
                    u.say("hold position");
                } else {
                    u.say("request vectors for an immediate landing");
                }
                u.comma().num(persons(ctx.def)).say("persons on board");
            }
            _ => {}
        }
        u
    }

    fn position_words(&self, ctx: &Ctx) -> (String, String) {
        // On a stand (from the airport layout), else on the apron.
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        if let Some((_, name, at, _)) =
            ctx.world.airport_detail.stands(&ctx.world.proj).into_iter().filter(|s| s.2.distance(p) < 45.0).min_by(|a, b| a.2.distance(p).total_cmp(&b.2.distance(p)))
        {
            let n: String = name.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
            if !n.is_empty() && n.len() <= 4 {
                let _ = at;
                return (format!("stand {}", phrase::spell(&n)), format!("stand {n}"));
            }
        }
        if self.on_runway(ctx).is_some() {
            return (String::new(), String::new());
        }
        ("on the apron".into(), "on the apron".into())
    }

    /// Transition level at an airport (flight level number).
    fn tl(&self, world: &World, airport: &str) -> u32 {
        let qnh = world.wx.qnh_hpa;
        let ta = world.airport_db.as_ref().and_then(|d| d.get(airport)).map_or(5000, atis::transition_altitude);
        atis::transition_level(ta, qnh)
    }

    /// The airport an arrival is for: the bot's, else the unit's.
    fn arrival_airport(&self, st: &Station, ctx: &Ctx) -> String {
        if let Some((_, Some(end))) = &ctx.bot {
            if let Some(r) = ctx.world.runways.iter().find(|r| end.same_runway(r) && !r.airport.is_empty()) {
                return r.airport.clone();
            }
        }
        st.airport.clone()
    }

    fn landing_runway(&self, st: &Station, ctx: &Ctx) -> Option<String> {
        if let Some(a) = &self.arr {
            return Some(a.runway.clone());
        }
        if let Some((_, Some(end))) = &ctx.bot {
            return self.runway_ident(ctx.world, "", end);
        }
        self.atis.get(&st.airport).map(|a| a.rwy.arr.clone())
    }

    /// The ident of a bot runway end at an airport.
    fn runway_ident(&self, world: &World, airport: &str, end: &crate::bot::RunwayEnd) -> Option<String> {
        let rw = world.runways.iter().find(|r| end.same_runway(r) && (r.airport == airport || airport.is_empty()))?;
        let id = if rw.a.distance(end.thr) < rw.b.distance(end.thr) { &rw.ident_a } else { &rw.ident_b };
        (!id.is_empty()).then(|| id.clone())
    }

    /// The runway we're on (airport, end ident facing our heading).
    fn on_runway(&self, ctx: &Ctx) -> Option<(String, String)> {
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        let h = ctx.ac.heading();
        let fwd = vec2(h.sin(), -h.cos());
        for rw in &ctx.world.runways {
            let d = rw.b - rw.a;
            let len = d.length().max(1.0);
            let u = d / len;
            let s = (p - rw.a).dot(u);
            let l = (p - rw.a).perp_dot(u);
            if s > -30.0 && s < len + 30.0 && l.abs() < rw.width * 0.5 + 2.0 && !rw.airport.is_empty() {
                let id = if fwd.dot(u) >= 0.0 { &rw.ident_a } else { &rw.ident_b };
                return Some((rw.airport.clone(), id.clone()));
            }
        }
        None
    }

    // ------------------------------------------------------------ responses

    /// The controller answers a request (the pilot's call just finished).
    fn respond(&mut self, r: Req, st: &Station, ctx: &Ctx) {
        let ac = ctx.ac;
        let tl = self.tl(ctx.world, &st.airport);
        let qnh = ctx.world.wx.qnh_hpa;
        match r {
            Req::SayAgain => {
                if let Some(u) = self.last_ctl.get(&st.text).cloned() {
                    self.say_ctl(st, u, 1.0, After::None);
                }
            }
            Req::Clearance => {
                let code = self.squawk.unwrap_or_else(|| new_squawk(&mut self.rng));
                self.squawk = Some(code);
                let ta = ctx.world.airport_db.as_ref().and_then(|d| d.get(&st.airport)).map_or(5000, atis::transition_altitude);
                let mut b = Utt::new();
                let mut rb = Utt::new();
                if self.vfr_default() && ctx.dest.is_none() {
                    let not_above = 2000.0;
                    for u in [&mut b, &mut rb] {
                        u.say("cleared to leave the control zone").comma().say_as("vfr", "VFR").comma().say("not above").level(not_above, tl).comma().qnh(qnh).comma().squawk(code);
                    }
                    self.cleared_alt_ft = Some(not_above);
                } else {
                    let init = (ta as f32).min(6000.0);
                    for u in [&mut b, &mut rb] {
                        match ctx.dest.as_deref() {
                            Some(d) => match phrase::airport_name(d) {
                                Some((s, t)) => {
                                    u.say_as(&format!("cleared to {s}"), &format!("cleared to {t}"));
                                }
                                None => {
                                    u.say_as(&format!("cleared to {}", phrase::spell(d)), &format!("cleared to {d}"));
                                }
                            },
                            None => {
                                u.say("cleared");
                            }
                        }
                        u.say("via flight planned route").comma().say("after departure climb straight ahead").comma().level(init, tl).comma().squawk(code);
                    }
                    self.cleared_alt_ft = Some(init);
                }
                self.phase = Phase::Cleared;
                self.plan_departure(ctx, &st.airport);
                let fx = vec![Fx::Squawk(code)];
                self.instruct(st, b, Some(rb), fx);
                // "Readback correct" (+ contact ground) once the read-back is in.
                let mut rc = Utt::new();
                rc.say("readback correct");
                let mut rb2 = None;
                let mut fx2 = Vec::new();
                if st.fac == Fac::Delivery {
                    if let Some(g) = self.find(&st.airport, &[Fac::Ground, Fac::Tower]) {
                        rc.comma().say("contact").say_as(&g.spoken, &g.text).freq(g.khz);
                        let mut r = Utt::new();
                        r.say_as(&g.spoken, &g.text).freq(g.khz);
                        rb2 = Some(r);
                        fx2.push(Fx::Tune(g.khz));
                        self.contacted.remove(&g.key());
                    }
                }
                let m = self.instruct_msg(st, rc, rb2, fx2);
                self.attach_followup(m);
            }
            Req::StartUp => {
                let mut b = Utt::new();
                b.say("start-up approved").comma().qnh(qnh);
                let mut rb = Utt::new();
                rb.say("start-up approved").comma().qnh(qnh);
                self.instruct(st, b, Some(rb), vec![]);
            }
            Req::Taxi => {
                if st.fac == Fac::Delivery {
                    return self.contact(st, &[Fac::Ground, Fac::Tower]);
                }
                self.plan_departure(ctx, &st.airport);
                let dep = self.dep.clone().unwrap_or_default();
                let mut b = Utt::new();
                let mut hold = String::new();
                if let Some(h) = &dep.hold {
                    hold = h.taxiway.clone();
                }
                for u in [&mut b] {
                    if hold.is_empty() {
                        u.say("taxi to holding point");
                    } else {
                        u.say_as(&format!("taxi to holding point {}", phrase::spell(&hold)), &format!("taxi to holding point {hold}"));
                    }
                    u.runway(&dep.runway);
                    let via = route_via(&dep);
                    if !via.is_empty() {
                        u.say("via");
                        for (i, t) in via.iter().enumerate() {
                            if i > 0 {
                                u.comma();
                            }
                            u.spell(t);
                        }
                    }
                    for c in &dep.crosses {
                        u.comma().say_as(&format!("cross runway {}", phrase::runway_words(c)), &format!("cross runway {c}"));
                    }
                    u.comma().qnh(qnh);
                }
                let rb = b.clone();
                self.phase = Phase::Taxi;
                self.instruct(st, b, Some(rb), vec![]);
            }
            Req::Ready => {
                if st.fac != Fac::Tower {
                    if let Some(t) = self.find(&st.airport, &[Fac::Tower]) {
                        return self.contact_station(st, &t);
                    }
                }
                if self.dep.is_none() {
                    self.plan_departure(ctx, &st.airport);
                }
                let rwy = self.on_runway(ctx).map(|r| r.1).or_else(|| self.dep.as_ref().map(|d| d.runway.clone())).unwrap_or_default();
                if let Some(d) = self.dep.as_mut() {
                    d.runway = rwy.clone();
                }
                let lined_up = self.on_runway(ctx).is_some();
                if lined_up || ctx.bot.is_some() {
                    self.takeoff_clearance(st, ctx, &rwy);
                } else {
                    let mut b = Utt::new();
                    b.runway(&rwy).comma().say("line up and wait");
                    let mut rb = Utt::new();
                    rb.say("line up and wait").comma().runway(&rwy);
                    self.phase = Phase::LineUp;
                    self.lineup_t = 0.0;
                    self.instruct(st, b, Some(rb), vec![]);
                }
            }
            Req::CheckIn => {
                let mut b = Utt::new();
                b.say_as(&st.spoken, &st.text).comma().say("identified");
                self.mark_replied(st);
                let mut rb = Utt::new();
                let cruise = ctx.def.ceiling_m.min(crate::nav::state::default_cruise(ctx.def)) / FT;
                let target = self.cleared_alt_ft.unwrap_or(cruise).max(cruise.min((tl * 100) as f32 + 1000.0));
                if self.emergency.is_none() {
                    b.comma().say("climb").level(target, tl).comma().say("resume own navigation");
                    rb.say("climb").level(target, tl).comma().say("resume own navigation");
                    self.cleared_alt_ft = Some(target);
                    self.phase = Phase::Enroute;
                    self.instruct(st, b, Some(rb), vec![Fx::Alt(target)]);
                } else {
                    self.instruct(st, b, None, vec![]);
                }
            }
            Req::RequestIls | Req::RequestVisual => {
                if st.fac == Fac::Tower && !ac.on_ground {
                    // Tower can only clear a visual approach close in.
                }
                let ils = r == Req::RequestIls;
                let p = vec2(ac.pos.x, ac.pos.z);
                let near = ctx.world.proj.to_world(st.ll.0, st.ll.1).distance(p) < 12.0 * NM;
                if !ils && (ctx.bot.is_some() || near) {
                    // A visual approach: cleared straight away, over to tower.
                    let rw = self.landing_runway(st, ctx).unwrap_or_default();
                    let apt = self.arrival_airport(st, ctx);
                    self.arr = Some(ArrPlan { airport: apt.clone(), runway: rw.clone(), ils: false, director: None });
                    self.phase = Phase::Approach;
                    let mut b = Utt::new();
                    if !self.contacted_reply(st) {
                        b.say_as(&st.spoken, &st.text).comma().say("identified").comma();
                        self.mark_replied(st);
                    }
                    b.say("cleared visual approach").runway(&rw);
                    let mut rb = Utt::new();
                    rb.say("cleared visual approach").runway(&rw);
                    let mut fx = vec![];
                    if st.fac != Fac::Tower {
                        if let Some(t) = self.find(&apt, &[Fac::Tower]) {
                            b.comma().say("contact").say_as(&t.spoken, &t.text).freq(t.khz);
                            rb.comma().say_as(&t.spoken, &t.text).freq(t.khz);
                            fx.push(Fx::Tune(t.khz));
                            self.contacted.remove(&t.key());
                        }
                    }
                    return self.instruct(st, b, Some(rb), fx);
                }
                self.begin_vectors(st, ctx, ils, None);
            }
            Req::Established => {
                if st.fac.is_radar() {
                    self.phase = Phase::Approach;
                    if let Some(t) = self.find(&st.airport, &[Fac::Tower]) {
                        return self.contact_station(st, &t);
                    }
                }
                self.landing_or_continue(st, ctx);
            }
            Req::FieldInSight => {
                let rw = self.landing_runway(st, ctx).unwrap_or_default();
                let mut b = Utt::new();
                b.say("cleared visual approach").runway(&rw);
                let rb = b.clone();
                self.phase = Phase::Approach;
                if let Some(a) = self.arr.as_mut() {
                    a.director = None;
                }
                if st.fac.is_radar() {
                    if let Some(t) = self.find(&st.airport, &[Fac::Tower]) {
                        b.comma().say("contact").say_as(&t.spoken, &t.text).freq(t.khz);
                        let mut rb2 = rb.clone();
                        rb2.comma().say_as(&t.spoken, &t.text).freq(t.khz);
                        return self.instruct(st, b, Some(rb2), vec![Fx::Tune(t.khz)]);
                    }
                }
                self.instruct(st, b, Some(rb), vec![]);
            }
            Req::Final => self.landing_or_continue(st, ctx),
            Req::GoingAround => {
                let alt = self.missed_alt(ctx, &st.airport);
                let mut b = Utt::new();
                b.say("roger").comma().say("climb straight ahead").comma().level(alt, tl);
                let mut rb = Utt::new();
                rb.say("climb straight ahead").comma().level(alt, tl);
                self.cleared_alt_ft = Some(alt);
                self.phase = Phase::Departure;
                let mut fx = vec![];
                if let Some(a) = self.find(&st.airport, &[Fac::Director, Fac::Approach, Fac::Radar]) {
                    if st.fac == Fac::Tower && ctx.bot.is_none() {
                        b.comma().say("contact").say_as(&a.spoken, &a.text).freq(a.khz);
                        rb.comma().say_as(&a.spoken, &a.text).freq(a.khz);
                        fx.push(Fx::Tune(a.khz));
                        self.contacted.remove(&a.key());
                    }
                }
                self.instruct(st, b, Some(rb), fx);
            }
            Req::Vacated => {
                if st.fac == Fac::Tower {
                    if let Some(g) = self.find(&st.airport, &[Fac::Ground]) {
                        return self.contact_station(st, &g);
                    }
                }
                self.phase = Phase::TaxiIn;
                let mut b = Utt::new();
                let via = self.taxi_in_route(ctx, &st.airport);
                b.say("taxi to the apron");
                if !via.is_empty() {
                    b.say("via");
                    for (i, t) in via.iter().enumerate() {
                        if i > 0 {
                            b.comma();
                        }
                        b.spell(t);
                    }
                }
                let rb = b.clone();
                self.instruct(st, b, Some(rb), vec![]);
            }
            Req::Mayday | Req::PanPan => {
                let mayday = r == Req::Mayday;
                self.emergency = Some(if mayday { "MAYDAY" } else { "PAN PAN" });
                let mut b = Utt::new();
                b.say(if mayday { "roger mayday" } else { "roger pan pan" });
                let mut fx = Vec::new();
                if !ac.on_ground && mayday {
                    b.comma().squawk(7700);
                    fx.push(Fx::Squawk(7700));
                }
                if ac.on_ground {
                    b.comma().say("emergency services are standing by");
                    self.instruct(st, b, Some(Utt::new().say("wilco").clone()), fx);
                } else {
                    // Vectors to the nearest ILS (this unit's airport).
                    b.comma().say("emergency services are standing by");
                    self.instruct(st, b, None, fx);
                    self.mark_replied(st);
                    self.begin_vectors(st, ctx, true, Some(()));
                }
            }
            Req::Readback | Req::Tune(..) => {}
        }
    }

    /// Queue a controller follow-up after the read-back of the message just queued.
    fn attach_followup(&mut self, m: Msg) {
        if let Some(last) = self.queue.back_mut() {
            match &mut last.after {
                After::Readback(_, _, then) | After::Done(_, then) => *then = Some(Box::new(m)),
                a => *a = After::Done(Vec::new(), Some(Box::new(m))),
            }
        }
    }

    fn contact(&mut self, st: &Station, facs: &[Fac]) {
        if let Some(t) = self.find(&st.airport, facs) {
            self.contact_station(st, &t);
        }
    }

    /// "Contact <unit> <frequency>" (read back, then tune).
    fn contact_station(&mut self, st: &Station, to: &Station) {
        let mut b = Utt::new();
        b.say("contact").say_as(&to.spoken, &to.text).freq(to.khz);
        let mut rb = Utt::new();
        rb.say_as(&to.spoken, &to.text).freq(to.khz);
        self.contacted.remove(&to.key());
        self.instruct(st, b, Some(rb), vec![Fx::Tune(to.khz)]);
    }

    fn takeoff_clearance(&mut self, st: &Station, ctx: &Ctx, rwy: &str) {
        let wx = &ctx.world.wx;
        let mut b = Utt::new();
        b.say("surface wind").wind(Some(wx.wind_from), wx.wind_kt, (wx.gust_kt > 0.0).then_some(wx.gust_kt)).comma().runway(rwy).comma().say("cleared for take-off");
        let mut rb = Utt::new();
        rb.say("cleared for take-off").comma().runway(rwy);
        self.phase = Phase::Takeoff;
        self.instruct(st, b, Some(rb), vec![Fx::ReleaseBot]);
    }

    fn landing_or_continue(&mut self, st: &Station, ctx: &Ctx) {
        let rw = self.landing_runway(st, ctx).unwrap_or_default();
        let wx = &ctx.world.wx;
        let mut b = Utt::new();
        b.say("surface wind").wind(Some(wx.wind_from), wx.wind_kt, (wx.gust_kt > 0.0).then_some(wx.gust_kt)).comma().runway(&rw).comma().say("cleared to land");
        let mut rb = Utt::new();
        rb.say("cleared to land").comma().runway(&rw);
        self.phase = Phase::Landing;
        if self.arr.is_none() {
            self.arr = Some(ArrPlan { airport: st.airport.clone(), runway: rw, ils: false, director: None });
        }
        self.instruct(st, b, Some(rb), vec![]);
    }

    fn missed_alt(&self, ctx: &Ctx, airport: &str) -> f32 {
        let elev = ctx.world.airport_db.as_ref().and_then(|d| d.get(airport)).map_or(0.0, |a| a.elev_ft);
        (((elev + 2000.0) / 1000.0).ceil() * 1000.0).max(2000.0).min(4000.0)
    }

    /// Start radar vectors for the arrival runway of the station's airport.
    fn begin_vectors(&mut self, st: &Station, ctx: &Ctx, ils: bool, emergency: Option<()>) {
        let world = ctx.world;
        let tl = self.tl(world, &st.airport);
        let qnh = world.wx.qnh_hpa;
        let Some(ap) = world.airport_db.as_ref().and_then(|d| d.get(&st.airport)).cloned() else { return };
        let rwy = self.ensure_atis(&ap, world).rwy.arr.clone();
        let ilsrec = crate::nav::db::ils_at(&ap).into_iter().find(|n| n.ils.as_ref().is_some_and(|i| i.runway == rwy));
        let fin = match &ilsrec {
            Some(n) => {
                let i = n.ils.as_ref().unwrap();
                let thr = world.proj.to_world(i.thr.0, i.thr.1);
                let loc = world.proj.to_world(i.loc.0, i.loc.1);
                let dir = (loc - thr).normalize_or(vectors::dir_of(i.course.to_degrees()));
                vectors::Final { thr, dir, elev_ft: ap.elev_ft, gs_deg: i.gs_angle.to_degrees() }
            }
            None => {
                let Some(rw) = world.runways.iter().find(|r| r.airport == ap.ident && (r.ident_a == rwy || r.ident_b == rwy)) else { return };
                let (thr, dir) = crate::airports::threshold(rw, rw.ident_b == rwy);
                vectors::Final { thr, dir, elev_ft: ap.elev_ft, gs_deg: 3.0 }
            }
        };
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        let dir = vectors::Director::new(fin, p);
        let alt_now = ctx.ac.pos.y / FT;
        let int_alt = dir.plan.alt_ft;
        let first_alt = if dir.track_miles(p) > 25.0 { int_alt.max((int_alt + 2000.0).min(alt_now)).max(int_alt) } else { int_alt };
        let first_alt = ((first_alt / 1000.0).floor() * 1000.0).max(int_alt);
        let mut b = Utt::new();
        if !self.contacted_reply(st) {
            b.say_as(&st.spoken, &st.text).comma().say("identified").comma();
        }
        b.say_as(&format!("vectors for ils approach runway {}", phrase::runway_words(&rwy)), &format!("vectors for ILS approach runway {rwy}"));
        let mut rb = Utt::new();
        let mut fx = Vec::new();
        if alt_now > first_alt + 200.0 {
            b.comma().say("descend").level(first_alt, tl).comma().qnh(qnh);
            rb.say("descend").level(first_alt, tl).comma().qnh(qnh);
            fx.push(Fx::Alt(first_alt));
        } else {
            b.comma().qnh(qnh);
            rb.qnh(qnh);
        }
        if !self.contacted_reply(st) {
            self.mark_replied(st);
        }
        self.cleared_alt_ft = Some(first_alt);
        self.arr = Some(ArrPlan { airport: ap.ident.clone(), runway: rwy, ils: ilsrec.is_some() && ils, director: Some(dir) });
        self.phase = Phase::Vectors;
        self.seq_given = false;
        let _ = emergency;
        self.instruct(st, b, Some(rb), fx);
    }

    fn contacted_reply(&self, st: &Station) -> bool {
        self.replied.contains(&st.key())
    }
    fn mark_replied(&mut self, st: &Station) {
        self.replied.insert(st.key());
    }
}

// ---------------------------------------------------------------- the frame

impl Atc {
    /// Advance by `dt` real seconds: stations, ATIS, the frequency, the
    /// controllers' own calls, deviations, the bot's radio work.
    pub fn update(&mut self, nav: &mut NavState, ctx: &Ctx, dt: f32) {
        if !self.enabled || ctx.paused {
            return;
        }
        let dt = dt.clamp(0.0, 0.25);
        self.time += dt;
        self.utc = ctx.utc;
        for w in self.warn_t.iter_mut() {
            *w = (*w - dt).max(0.0);
        }
        let ac = ctx.ac;
        // No power, no radio.
        if !nav.radios.powered || ac.crashed.is_some() {
            if !self.on_air.is_empty() {
                self.emit(RadioOut::Cut(0));
                self.emit(RadioOut::Cut(1));
                self.on_air.clear();
            }
            self.queue.clear();
            self.current = None;
            self.busy = 0.0;
            for a in self.atis.values_mut() {
                a.playing = [false; 2];
            }
            return;
        }
        radio::prepare();
        // Stations around.
        self.stations_t -= dt;
        let ll = ctx.world.proj.to_latlon(vec2(ac.pos.x, ac.pos.z));
        if self.stations_t <= 0.0 {
            self.stations_t = 3.0;
            let moved = self.stations_at.is_none_or(|p| crate::airports::distance_km(p, ll) > 15.0);
            if moved || self.stations.is_empty() {
                self.refresh_stations(ctx.world, ll);
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(dir) = std::env::var_os("SKYWARD_ATC_DUMP") {
            dump_layouts(ctx.world, std::path::Path::new(&dir));
        }
        if !self.tuned_once && !self.stations.is_empty() {
            self.tuned_once = true;
            self.initial_tune(nav, ctx);
            self.last_khz = [nav.radios.com[0].active, nav.radios.com[1].active];
        }
        // Detuning cuts what was playing on that receiver.
        for k in 0..2 {
            let f = nav.radios.com[k].active;
            if f != self.last_khz[k] {
                if k == 1 && self.tuned_once {
                    self.com2_rx = true;
                }
                if self.on_air.iter().any(|o| o.ch == k as u8) {
                    self.emit(RadioOut::Cut(k as u8));
                    self.on_air.retain(|o| o.ch != k as u8);
                }
                for a in self.atis.values_mut() {
                    a.playing[k] = false;
                    a.next[k] = 0.6;
                }
                self.last_khz[k] = f;
            }
        }
        for o in self.on_air.iter_mut() {
            o.left -= dt;
        }
        self.on_air.retain(|o| o.left > -0.4);
        // The ATIS of the airport we're talking to (its letter for our calls).
        if self.stations_t > 2.9 {
            if let Some(ap) = self.tuned(nav, ctx).and_then(|s| ctx.world.airport_db.as_ref().and_then(|d| d.get(&s.airport)).cloned()) {
                self.ensure_atis(&ap, ctx.world);
            }
        }
        self.atis_frame(nav, ctx, dt);
        self.frequency(nav, ctx, dt);
        if self.queue.is_empty() && self.current.is_none() && self.quiet > 0.8 {
            if let Some(r) = self.deferred.pop_front() {
                self.request(r, nav, ctx);
            }
        }
        // Controllers call us only once we're talking to them (a flight that
        // never calls ATC is left alone — except a 7700 squawk).
        let talking = !self.contacted.is_empty() || nav.radios.squawk() == 7700;
        if self.queue.is_empty() && self.current.is_none() && self.quiet > 2.5 && talking {
            self.monitor(nav, ctx, dt);
        }
        if let Some((phase, _)) = &ctx.bot {
            if self.bot_comms {
                let phase = phase.clone();
                self.bot_radio(nav, ctx, &phase);
            }
        } else {
            self.bot_hold = false;
        }
    }

    /// First radio setup at the start: COM 1 on the unit to call, COM 2 on the ATIS.
    fn initial_tune(&mut self, nav: &mut NavState, ctx: &Ctx) {
        let ac = ctx.ac;
        let here = self.nearest_airport(ctx);
        let Some(ap) = here else { return };
        if let Some(a) = self.find(&ap, &[Fac::Atis]) {
            nav.radios.com[1].active = a.khz;
        }
        let first = if ac.on_ground {
            if self.on_runway(ctx).is_some() { self.find(&ap, &[Fac::Tower, Fac::Ground]) } else { self.find(&ap, &[Fac::Delivery, Fac::Ground, Fac::Tower]) }
        } else {
            self.find(&ap, &[Fac::Director, Fac::Approach, Fac::Radar, Fac::Tower])
        };
        if std::env::var_os("SKYWARD_ATC_DEBUG").is_some() {
            eprintln!(
                "atc: start at {:?} ({:?}), on runway {:?}, first unit {:?}",
                ac.pos,
                ctx.world.proj.to_latlon(vec2(ac.pos.x, ac.pos.z)),
                self.on_runway(ctx),
                first.as_ref().map(|s| &s.text)
            );
        }
        if let Some(s) = first {
            nav.radios.com[0].active = s.khz;
        }
        let second = if ac.on_ground { self.find(&ap, &[Fac::Ground, Fac::Tower]) } else { self.find(&ap, &[Fac::Tower]) };
        if let Some(s) = second.filter(|s| s.khz != nav.radios.com[0].active) {
            nav.radios.com[0].standby = s.khz;
        }
        if !ac.on_ground {
            self.phase = Phase::Enroute;
        } else if self.on_runway(ctx).is_some() {
            self.phase = Phase::Taxi;
        }
        self.last_khz = [nav.radios.com[0].active, nav.radios.com[1].active];
    }

    /// The nearest airport with stations (ident).
    fn nearest_airport(&self, ctx: &Ctx) -> Option<String> {
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        self.stations
            .iter()
            .map(|s| (s, ctx.world.proj.to_world(s.ll.0, s.ll.1).distance(p)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .filter(|x| x.1 < 40_000.0)
            .map(|x| x.0.airport.clone())
    }

    /// The ATIS state of an airport (runway in use, letter, broadcast), updated with the weather.
    fn ensure_atis(&mut self, ap: &crate::airports::Airport, world: &World) -> &AtisState {
        let wx = &world.wx;
        let rwy = atis::runway_in_use(ap, wx.wind_from, wx.wind_kt).unwrap_or(atis::InUse { arr: String::new(), dep: String::new(), hdg: 0.0 });
        let dg = atis::digest(wx, &rwy);
        let utc = self.utc;
        let entry = self.atis.entry(ap.ident.clone()).or_insert_with(|| {
            let letter = (b'A' + (hash(&ap.ident) % 26) as u8) as char;
            AtisState { letter, digest: String::new(), rwy: rwy.clone(), utt: Utt::new(), next: [0.0; 2], playing: [false; 2] }
        });
        if entry.digest != dg {
            if !entry.digest.is_empty() {
                entry.letter = if entry.letter == 'Z' { 'A' } else { (entry.letter as u8 + 1) as char };
            }
            entry.digest = dg;
            entry.rwy = rwy;
            let t = wx.metar.as_ref().and_then(|m| m.time).map(|(_, h, m)| (h, m)).unwrap_or((utc.0, utc.1 / 20 * 20));
            entry.utt = atis::broadcast(ap, wx, entry.letter, t, &entry.rwy);
        }
        entry
    }

    /// ATIS on either receiver: loop the broadcast.
    fn atis_frame(&mut self, nav: &NavState, ctx: &Ctx, dt: f32) {
        for k in 0..2 {
            if k == 1 && !self.com2_rx {
                if self.on_air.iter().any(|o| o.ch == 1) {
                    self.emit(RadioOut::Cut(1));
                    self.on_air.retain(|o| o.ch != 1);
                }
                continue;
            }
            let khz = nav.radios.com[k].active;
            let st = self.station_on(khz, ctx.world, ctx.ac, ctx.ground_y).filter(|s| s.0.fac == Fac::Atis);
            let Some((st, q)) = st else { continue };
            let Some(ap) = ctx.world.airport_db.as_ref().and_then(|d| d.get(&st.airport)).cloned() else { continue };
            let letter = self.ensure_atis(&ap, ctx.world).letter;
            let playing = self.on_air.iter().any(|o| o.ch == k as u8 && o.atis);
            let a = self.atis.get_mut(&ap.ident).unwrap();
            if playing {
                continue;
            }
            a.next[k] -= dt;
            if a.next[k] > 0.0 {
                continue;
            }
            a.next[k] = 1.6;
            let utt = a.utt.clone();
            let dur = radio::duration(st.voice, &utt.toks);
            let id = radio::submit(radio::Tx { voice: st.voice, toks: utt.toks.clone(), ch: k as u8, own: false, strength: q, volume: 0.85 });
            self.emit(RadioOut::Start(id));
            let lw = cap(phrase::letter_word(letter).unwrap_or("alpha"));
            self.on_air.push(OnAir {
                ch: k as u8,
                who: format!("{} ATIS · information {lw}", st.text.trim_end_matches(" Information")),
                text: utt.text.clone(),
                own: false,
                atis: true,
                left: dur,
                total: dur,
            });
        }
    }

    /// The ATC frequency (COM 1): play the queue, half-duplex.
    fn frequency(&mut self, nav: &mut NavState, ctx: &Ctx, dt: f32) {
        self.busy -= dt;
        if self.current.is_none() {
            self.quiet += dt;
        }
        // A transmission finished.
        if self.busy <= 0.0 {
            if let Some(m) = self.current.take() {
                self.quiet = 0.0;
                self.finished(m, nav, ctx);
            }
        }
        if self.busy > 0.0 || self.current.is_some() {
            return;
        }
        let Some(front) = self.queue.front_mut() else { return };
        front.delay -= dt;
        if front.delay > 0.0 {
            return;
        }
        let m = self.queue.pop_front().unwrap();
        let khz = nav.radios.com[0].active;
        let tuned = m.station.khz.abs_diff(khz) <= 3;
        let rx = self.reception(&m.station, ctx.world, ctx.ac.pos, ctx.ground_y).map(|x| x.1);
        let voice = if m.pilot { Voice::Pilot } else { m.station.voice };
        let dur = radio::duration(voice, &m.utt.toks).max(0.8);
        // The controller isn't heard off frequency (and the message is lost).
        if !m.pilot && (!tuned || rx.is_none()) {
            return;
        }
        let who = if m.pilot { self.callsign.short_text.clone() } else { m.station.text.clone() };
        let id = radio::submit(radio::Tx { voice, toks: m.utt.toks.clone(), ch: 0, own: m.pilot, strength: rx.unwrap_or(1.0), volume: 1.0 });
        self.emit(RadioOut::Start(id));
        self.on_air.retain(|o| o.ch != 0 || o.atis);
        self.on_air.push(OnAir { ch: 0, who: who.clone(), text: m.utt.text.clone(), own: m.pilot, atis: false, left: dur, total: dur });
        self.log_line(who, m.utt.text.clone(), m.pilot);
        self.busy = dur;
        // A pilot call off frequency (or out of range) is not answered.
        let mut m = m;
        if m.pilot && (!tuned || rx.is_none() || m.station.airport.is_empty()) {
            m.after = match m.after {
                After::Done(fx, _) => After::Done(fx, None),
                _ => After::None,
            };
        }
        self.current = Some(m);
    }

    fn finished(&mut self, m: Msg, nav: &mut NavState, ctx: &Ctx) {
        match m.after {
            After::None => {}
            After::Reply(r) => self.respond(r, &m.station, ctx),
            After::Readback(rb, fx, then) => {
                if self.auto_readback || ctx.bot.is_some() {
                    let d = 0.7 + 0.8 * self.rand();
                    self.say_pilot(&m.station, rb, d, After::Done(fx, then));
                } else {
                    self.pending_readback = Some((rb, m.station.clone()));
                    self.pending_readback_fx = fx;
                    self.pending_then = then;
                }
            }
            After::Done(fx, then) => {
                for f in fx {
                    self.apply(f, nav, ctx);
                }
                if let Some(t) = then {
                    self.queue.push_back(*t);
                }
            }
        }
    }

    fn apply(&mut self, f: Fx, nav: &mut NavState, ctx: &Ctx) {
        let assist = self.assist || ctx.bot.is_some();
        match f {
            Fx::Tune(khz) if assist => {
                let c = &mut nav.radios.com[0];
                if c.active != khz {
                    c.standby = c.active;
                    c.active = khz;
                }
            }
            Fx::Squawk(code) if assist => {
                let s = format!("{code:04}");
                for (i, ch) in s.chars().enumerate().take(4) {
                    nav.radios.xpdr.code[i] = ch.to_digit(10).unwrap_or(0) as u8;
                }
                if nav.radios.xpdr.mode == crate::nav::radio::XpdrMode::Stby {
                    nav.radios.xpdr.mode = crate::nav::radio::XpdrMode::Alt;
                }
            }
            Fx::ReleaseBot => self.bot_hold = false,
            Fx::Hdg(h) if self.ap_follow => self.ap_cmds.push(ApCmd::Hdg(h)),
            Fx::Alt(a) if self.ap_follow => self.ap_cmds.push(ApCmd::Alt(a)),
            Fx::Appr if self.ap_follow => self.ap_cmds.push(ApCmd::Appr),
            _ => {}
        }
    }

    /// The station COM 1 is on (in range).
    fn tuned(&self, nav: &NavState, ctx: &Ctx) -> Option<Station> {
        self.station_on(nav.radios.com[0].active, ctx.world, ctx.ac, ctx.ground_y).map(|x| x.0)
    }

    /// Controller-initiated calls: hand-offs, vectors, landing / vacate, deviations.
    fn monitor(&mut self, nav: &mut NavState, ctx: &Ctx, dt: f32) {
        let ac = ctx.ac;
        let Some(st) = self.tuned(nav, ctx) else { return };
        if st.fac == Fac::Atis {
            return;
        }
        let agl = (ac.pos.y - ctx.ground_y) / FT;
        let tl = self.tl(ctx.world, &st.airport);
        let gs = vec2(ac.vel.x, ac.vel.z).length();
        let bot = ctx.bot.is_some();
        // Squawk 7700 without a call: treat it as a distress call.
        if self.emergency.is_none() && nav.radios.squawk() == 7700 && !ac.on_ground && st.fac != Fac::Atis {
            self.emergency = Some("MAYDAY");
            let mut b = Utt::new();
            b.say("roger mayday").comma().say("emergency services are standing by");
            self.instruct(&st, b, None, vec![]);
            self.begin_vectors(&st, ctx, true, Some(()));
            return;
        }
        // Traffic information and sequencing against the traffic picture (traffic.rs).
        if self.traffic_calls(&st, ctx, dt) {
            return;
        }
        match self.phase {
            // Line-up: the take-off clearance once lined up (or after a while).
            Phase::LineUp if st.fac == Fac::Tower => {
                self.lineup_t += dt;
                let aligned = self.on_runway(ctx).is_some() && gs < 3.0;
                if (aligned && self.lineup_t > 6.0) || self.lineup_t > 45.0 {
                    let rwy = self.dep.as_ref().map(|d| d.runway.clone()).unwrap_or_default();
                    self.takeoff_clearance(&st, ctx, &rwy);
                }
            }
            // Airborne: tower hands over to departure / radar.
            Phase::Takeoff if !ac.on_ground && agl > 1200.0 && st.fac == Fac::Tower => {
                self.phase = Phase::Departure;
                match self.find(&st.airport, &[Fac::Departure, Fac::Radar, Fac::Approach, Fac::Director, Fac::Control]) {
                    Some(d) => self.contact_station(&st, &d),
                    None => {
                        self.phase = Phase::Enroute;
                        let mut b = Utt::new();
                        b.say("frequency change approved");
                        let rb = Utt::new().say("roger").clone();
                        self.instruct(&st, b, Some(rb), vec![]);
                    }
                }
            }
            // Radar vectors.
            Phase::Vectors if st.fac.is_radar() && self.arr.as_ref().is_some_and(|a| a.airport == st.airport) && !bot => {
                let pos = vec2(ac.pos.x, ac.pos.z);
                let hdg = ac.heading().to_degrees().rem_euclid(360.0);
                let wind = ctx.world.wind.mean(ac.pos.y.max(0.0), 0.3);
                let tas = ac.tas().max(50.0);
                let arr = self.arr.clone().unwrap();
                let Some(mut d) = arr.director.clone() else { return };
                let call = d.update(pos, hdg, tas, wind);
                let int_alt = d.plan.alt_ft;
                let miles = d.track_miles(pos);
                if let Some(a) = self.arr.as_mut() {
                    a.director = Some(d.clone());
                }
                let need_descent = self.cleared_alt_ft.is_some_and(|c| c > int_alt + 50.0);
                match call {
                    Some(vectors::Call::Heading { hdg: h, turn }) => {
                        let (mut b, mut rb) = (Utt::new(), Utt::new());
                        turn_words(&mut b, h, turn);
                        turn_words(&mut rb, h, turn);
                        let mut fx = vec![Fx::Hdg(h)];
                        if need_descent && miles < 24.0 {
                            b.comma().say("descend").level(int_alt, tl);
                            rb.comma().say("descend").level(int_alt, tl);
                            self.cleared_alt_ft = Some(int_alt);
                            fx.push(Fx::Alt(int_alt));
                        }
                        self.instruct(&st, b, Some(rb), fx);
                    }
                    Some(vectors::Call::Intercept { hdg: h, turn }) => {
                        let (mut b, mut rb) = (Utt::new(), Utt::new());
                        let m = miles.round() as u32;
                        if (5..=30).contains(&m) {
                            b.say_as(&format!("{} miles from touchdown", phrase::digits(m)), &format!("{m} miles from touchdown")).comma();
                        }
                        turn_words(&mut b, h, turn);
                        turn_words(&mut rb, h, turn);
                        let mut fx = vec![Fx::Hdg(h)];
                        if need_descent {
                            b.comma().say("descend").level(int_alt, tl);
                            rb.comma().say("descend").level(int_alt, tl);
                            self.cleared_alt_ft = Some(int_alt);
                            fx.push(Fx::Alt(int_alt));
                        }
                        if arr.ils {
                            fx.push(Fx::Appr);
                        }
                        let rw = arr.runway.clone();
                        if arr.ils {
                            b.comma()
                                .say_as(&format!("cleared ils approach runway {}", phrase::runway_words(&rw)), &format!("cleared ILS approach runway {rw}"))
                                .comma()
                                .say("report established");
                            rb.comma()
                                .say_as(&format!("cleared ils approach runway {}", phrase::runway_words(&rw)), &format!("cleared ILS approach runway {rw}"))
                                .comma()
                                .say("wilco");
                        } else {
                            b.comma().say("report field in sight");
                            rb.comma().say("wilco");
                        }
                        self.phase = if arr.ils { Phase::Approach } else { Phase::Vectors };
                        self.instruct(&st, b, Some(rb), fx);
                    }
                    None => {
                        if need_descent && miles < 24.0 {
                            let (mut b, mut rb) = (Utt::new(), Utt::new());
                            b.say("descend").level(int_alt, tl);
                            rb.say("descend").level(int_alt, tl);
                            self.cleared_alt_ft = Some(int_alt);
                            self.instruct(&st, b, Some(rb), vec![Fx::Alt(int_alt)]);
                        }
                    }
                }
            }
            // Established on the localiser: report it (assist) — the director hands over to tower.
            Phase::Approach if st.fac.is_radar() && self.assist && !bot => {
                let o = &nav.radios.nav[0].out;
                if o.loc && o.valid && o.cdi.abs() < 0.3 && o.to_thr > 0.0 && o.to_thr < 20.0 * NM {
                    self.request(Req::Established, nav, ctx);
                }
            }
            // Handed to tower on the approach: check in (assist).
            Phase::Approach if st.fac == Fac::Tower && self.assist && !bot && !self.contacted.contains(&st.key()) => {
                self.request(Req::Established, nav, ctx);
            }
            // Landed: vacate (or, for the bot, backtrack).
            Phase::Landing if ac.on_ground && gs < 35.0 * KT && st.fac == Fac::Tower => {
                self.phase = Phase::Landed;
                let rwy = self.arr.as_ref().map(|a| a.runway.clone()).unwrap_or_default();
                if bot {
                    let mut b = Utt::new();
                    b.say_as(&format!("backtrack runway {}", phrase::runway_words(&rwy)), &format!("backtrack runway {rwy}")).comma().say("report when ready for departure");
                    let mut rb = Utt::new();
                    rb.say_as(&format!("backtrack runway {}", phrase::runway_words(&rwy)), &format!("backtrack runway {rwy}")).comma().say("wilco");
                    self.arr = None;
                    self.dep = Some(DepPlan { airport: st.airport.clone(), runway: String::new(), ..Default::default() });
                    self.phase = Phase::Taxi;
                    self.instruct(&st, b, Some(rb), vec![]);
                } else {
                    let left = self.exit_side(ctx, &st.airport);
                    let mut b = Utt::new();
                    b.say(if left { "vacate left" } else { "vacate right" });
                    let mut rb = Utt::new();
                    rb.say(if left { "vacate left" } else { "vacate right" });
                    let mut fx = vec![];
                    if let Some(g) = self.find(&st.airport, &[Fac::Ground]) {
                        b.comma().say("contact").say_as(&g.spoken, &g.text).freq(g.khz);
                        rb.comma().say_as(&g.spoken, &g.text).freq(g.khz);
                        fx.push(Fx::Tune(g.khz));
                        self.contacted.remove(&g.key());
                    }
                    self.instruct(&st, b, Some(rb), fx);
                }
            }
            _ => {}
        }
        if !self.queue.is_empty() || bot {
            return;
        }
        self.deviations(nav, ctx, &st, agl, gs, dt);
    }

    /// Altitude busts, runway incursions, take-off / landing without a clearance.
    fn deviations(&mut self, _nav: &mut NavState, ctx: &Ctx, st: &Station, agl: f32, gs: f32, dt: f32) {
        let ac = ctx.ac;
        let tl = self.tl(ctx.world, &st.airport);
        let alt_ft = ac.pos.y / FT;
        // Altitude bust (radar units, airborne, cleared level known).
        let level_phase = matches!(self.phase, Phase::Departure | Phase::Enroute | Phase::Vectors);
        if let Some(c) = self.cleared_alt_ft.filter(|_| st.fac.is_radar() && !ac.on_ground && agl > 800.0 && level_phase) {
            let off = alt_ft - c;
            if off.abs() < 200.0 {
                self.level_reached = Some(c);
            }
            // Climbing / descending through the cleared level counts too.
            let through = self.level_reached != Some(c) && off > 300.0 && ac.vertical_speed() > 0.0 && self.phase == Phase::Departure;
            let away = (self.level_reached == Some(c) && off.abs() > 300.0) || through;
            self.bust_t = if away { self.bust_t + dt } else { 0.0 };
            if self.bust_t > 5.0 && self.warn_t[0] <= 0.0 {
                self.warn_t[0] = 40.0;
                let mut b = Utt::new();
                b.say("check altitude").comma().say("cleared").level(c, tl);
                let mut rb = Utt::new();
                rb.say("cleared").level(c, tl);
                self.instruct(st, b, Some(rb), vec![]);
                return;
            }
        }
        let on_rwy = self.on_runway(ctx);
        // Runway incursion / take-off without a clearance.
        if ac.on_ground && matches!(st.fac, Fac::Tower | Fac::Ground) && self.warn_t[1] <= 0.0 {
            if let Some((ap, id)) = &on_rwy {
                let allowed = matches!(self.phase, Phase::LineUp | Phase::Takeoff | Phase::Landing | Phase::Landed)
                    || (self.phase == Phase::Taxi && self.dep.as_ref().is_some_and(|d| d.crosses.iter().any(|c| c.split('/').any(|e| e == id) || c.contains(id.as_str()))));
                if !allowed && *ap == st.airport && gs > 2.0 {
                    self.warn_t[1] = 25.0;
                    let mut b = Utt::new();
                    b.say("stop immediately").comma().say_as("i say again", "I say again").comma().say("stop immediately");
                    if gs < 25.0 * KT {
                        b.comma().say("you are not cleared to enter the runway");
                    }
                    let rb = Utt::new().say("stop immediately").clone();
                    self.instruct(st, b, Some(rb), vec![]);
                    return;
                }
            }
        }
        // Landing without a clearance: go around.
        if !ac.on_ground && st.fac == Fac::Tower && self.phase != Phase::Landing && agl < 400.0 && agl > 40.0 && self.warn_t[2] <= 0.0 {
            let p = vec2(ac.pos.x, ac.pos.z);
            let h = ac.heading();
            let fwd = vec2(h.sin(), -h.cos());
            let near_thr = ctx.world.runways.iter().filter(|r| r.airport == st.airport).any(|r| {
                [(r.a, r.b), (r.b, r.a)].iter().any(|(t, f)| {
                    let d = (*f - *t).normalize_or_zero();
                    let along = (*t - p).dot(d);
                    along > -200.0 && along < 3.0 * NM && (p - *t).perp_dot(d).abs() < 300.0 && fwd.dot(d) > 0.9
                })
            });
            if near_thr {
                self.warn_t[2] = 30.0;
                let mut b = Utt::new();
                b.say("go around").comma().say_as("i say again", "I say again").comma().say("go around");
                let rb = Utt::new().say("going around").clone();
                self.phase = Phase::Departure;
                self.instruct(st, b, Some(rb), vec![]);
            }
        }
    }

    /// Which side the nearest taxiway exit is on (looking along the landing direction).
    fn exit_side(&mut self, ctx: &Ctx, airport: &str) -> bool {
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        let h = ctx.ac.heading();
        let fwd = vec2(h.sin(), -h.cos());
        let right = vec2(-fwd.y, fwd.x);
        if let Some(g) = self.graph_for(ctx, airport) {
            let mut best: Option<(f32, f32)> = None;
            for q in &g.pts {
                let d = *q - p;
                let ahead = d.dot(fwd);
                let side = d.dot(right);
                if ahead > -50.0 && ahead < 1500.0 && side.abs() > 40.0 && side.abs() < 200.0 && best.is_none_or(|b| ahead < b.0) {
                    best = Some((ahead, side));
                }
            }
            if let Some((_, s)) = best {
                return s < 0.0;
            }
        }
        true
    }

    /// The taxi graph of an airport, from its layout (world frame).
    fn graph_for(&mut self, ctx: &Ctx, airport: &str) -> Option<&taxi::Graph> {
        let l = ctx.world.airport_detail.layout(airport)?;
        let key = l.taxiways.len() as u64 ^ (l.taxiways.iter().map(|t| t.0.len()).sum::<usize>() as u64) << 20;
        let stale = !self.graph.as_ref().is_some_and(|g| g.0 == airport && g.1 == key);
        if stale {
            let m = l.frame.map_to(&ctx.world.proj);
            let tw: Vec<(Vec<Vec2>, String)> = l.taxiways.iter().map(|(pl, n)| (pl.iter().map(|p| m.apply(*p)).collect(), n.clone())).collect();
            self.graph = Some((airport.to_string(), key, taxi::Graph::build(&tw)));
        }
        self.graph.as_ref().map(|g| &g.2)
    }

    /// Runways of an airport as boxes for the router.
    fn runway_boxes(world: &World, airport: &str) -> Vec<taxi::RwyBox> {
        world.runways.iter().filter(|r| r.airport == airport).map(|r| taxi::RwyBox { a: r.a, b: r.b, width: r.width, name: format!("{}/{}", r.ident_a, r.ident_b) }).collect()
    }

    /// Departure runway, holding point and taxi route from where the aircraft is.
    fn plan_departure(&mut self, ctx: &Ctx, airport: &str) {
        let world = ctx.world;
        let Some(ap) = world.airport_db.as_ref().and_then(|d| d.get(airport)).cloned() else { return };
        let mut rwy = self.ensure_atis(&ap, world).rwy.dep.clone();
        if let Some((_, Some(end))) = &ctx.bot {
            if let Some(id) = self.runway_ident(world, airport, end) {
                rwy = id;
            }
        }
        let mut plan = DepPlan { airport: airport.to_string(), runway: rwy.clone(), ..Default::default() };
        let Some(rw) = world.runways.iter().find(|r| r.airport == ap.ident && (r.ident_a == rwy || r.ident_b == rwy)).cloned() else {
            self.dep = Some(plan);
            return;
        };
        let (thr, dir) = crate::airports::threshold(&rw, rw.ident_b == rwy);
        let boxes = Self::runway_boxes(world, airport);
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        let layout = world.airport_detail.layout(airport);
        if std::env::var_os("SKYWARD_ATC_DEBUG").is_some() {
            eprintln!(
                "atc: departure {airport} {rwy}: layout {}",
                layout.as_ref().map_or("none".into(), |l| format!(
                    "{} taxiways, {} holds ({:?})",
                    l.taxiways.len(),
                    l.holds.len(),
                    l.holds.iter().map(|h| h.runway.clone()).collect::<std::collections::BTreeSet<_>>()
                ))
            );
        }
        if let (Some(l), Some(g)) = (layout.clone(), self.graph_for(ctx, airport).cloned()) {
            let m = l.frame.map_to(&world.proj);
            let holds: Vec<(Vec2, String, String)> = l.holds.iter().map(|h| (m.apply(h.at), h.taxiway.clone(), h.runway.clone())).collect();
            let rname = rw.name.clone();
            // Of the holds near the start of the take-off run, the one with the
            // best route: not across the departure runway, short.
            let mut best: Option<(f32, taxi::HoldPt, taxi::Route)> = None;
            for h in taxi::departure_holds(&holds, &rname, thr, dir) {
                let Some(r) = g.route(p, h.0.at, 400.0, &boxes) else { continue };
                let score = r.length + if r.crosses.contains(&rname) { 5000.0 } else { 0.0 } + 1.5 * h.1;
                if std::env::var_os("SKYWARD_ATC_DEBUG").is_some() {
                    eprintln!(
                        "atc:   hold {} at {:?} ({:.0} m from the start): route {:.0} m via {:?} crossing {:?} → {score:.0}",
                        h.0.taxiway, h.0.at, h.1, r.length, r.via, r.crosses
                    );
                }
                if best.as_ref().is_none_or(|b| score < b.0) {
                    best = Some((score, h.0, r));
                }
            }
            if let Some((_, h, r)) = best {
                plan.route = r.pts;
                plan.crosses = r.crosses.iter().map(|c| pick_end(c, &rw)).collect();
                let mut via = r.via;
                if via.last().is_some_and(|l| *l == h.taxiway) && via.len() > 1 {
                    via.pop();
                }
                plan.via = via;
                plan.hold = Some(h);
            }
        }
        self.dep = Some(plan);
    }

    /// Route from the runway exit to the nearest stand / apron: the "via" list.
    fn taxi_in_route(&mut self, ctx: &Ctx, airport: &str) -> Vec<String> {
        let world = ctx.world;
        let p = vec2(ctx.ac.pos.x, ctx.ac.pos.z);
        let stands: Vec<Vec2> = world.airport_detail.stands(&world.proj).into_iter().filter(|s| s.0 == airport).map(|s| s.2).collect();
        let boxes = Self::runway_boxes(world, airport);
        let Some(g) = self.graph_for(ctx, airport).cloned() else { return Vec::new() };
        let Some(target) = stands.iter().min_by(|a, b| a.distance(p).total_cmp(&b.distance(p))).copied() else { return Vec::new() };
        g.route(p, target, 400.0, &boxes).map(|r| r.via).unwrap_or_default()
    }

    /// The auto-flight bot's radio work (watch mode).
    fn bot_radio(&mut self, nav: &mut NavState, ctx: &Ctx, phase: &str) {
        let changed = phase != self.bot_phase;
        self.bot_phase = phase.to_string();
        let idle = self.queue.is_empty() && self.current.is_none() && self.quiet > 1.5 && self.pending_readback.is_none();
        let airport =
            ctx.bot.as_ref().and_then(|b| b.1.as_ref()).and_then(|end| ctx.world.runways.iter().find(|r| end.same_runway(r)).map(|r| r.airport.clone())).filter(|a| !a.is_empty());
        let Some(airport) = airport.or_else(|| self.nearest_airport(ctx)) else {
            self.bot_hold = false;
            return;
        };
        let tower = self.find(&airport, &[Fac::Tower]);
        let tune = |nav: &mut NavState, s: &Station| {
            let c = &mut nav.radios.com[0];
            if c.active != s.khz {
                c.standby = c.active;
                c.active = s.khz;
            }
        };
        match phase {
            "Hold" => {
                let Some(t) = tower else {
                    self.bot_hold = false;
                    return;
                };
                if changed {
                    self.bot_hold = self.phase != Phase::Takeoff;
                    if !matches!(self.phase, Phase::LineUp | Phase::Takeoff) {
                        self.phase = Phase::Taxi;
                        self.dep = None;
                    }
                }
                if !matches!(self.phase, Phase::Takeoff | Phase::LineUp) && idle && self.bot_hold {
                    tune(nav, &t);
                    if self.dep.is_none() {
                        self.plan_departure(ctx, &airport);
                    }
                    self.request(Req::Ready, nav, ctx);
                }
            }
            "TakeoffRoll" | "Rotate" => {
                self.bot_hold = false;
                if self.phase == Phase::Taxi {
                    self.phase = Phase::Takeoff;
                }
            }
            "Enroute" | "LowPass" | "InitialClimb" => {
                if self.phase == Phase::Departure && idle {
                    if let Some(st) = self.tuned(nav, ctx).filter(|s| s.fac.is_radar() && !self.contacted.contains(&s.key())) {
                        let _ = st;
                        self.request(Req::CheckIn, nav, ctx);
                    }
                }
            }
            "Position" => {
                if idle && matches!(self.phase, Phase::Enroute | Phase::Departure) {
                    if let Some(a) = self.find(&airport, &[Fac::Director, Fac::Approach, Fac::Radar]) {
                        tune(nav, &a);
                        self.request(Req::RequestVisual, nav, ctx);
                    } else if let Some(t) = &tower {
                        tune(nav, t);
                        self.phase = Phase::Approach;
                    }
                }
            }
            "Final" => {
                if idle && !matches!(self.phase, Phase::Landing) {
                    if let Some(t) = &tower {
                        tune(nav, t);
                        self.request(Req::Final, nav, ctx);
                    }
                }
            }
            "GoAround" if changed && self.phase == Phase::Landing => self.request(Req::GoingAround, nav, ctx),
            _ => {}
        }
    }
}

// ---------------------------------------------------------------- helpers

/// Append `b` to `a` (tokens and text).
fn merge(a: &mut Utt, b: &Utt) {
    a.toks.extend(b.toks.iter().copied());
    a.missing.extend(b.missing.iter().cloned());
    let t = b.text.trim();
    if t.is_empty() {
        return;
    }
    if !a.text.is_empty() && !a.text.ends_with(' ') {
        a.text.push(' ');
    }
    // Lower-case the first letter unless it's a name / code.
    let mut c = t.chars();
    let f = c.next().unwrap();
    let keep =
        t.split(' ').next().is_some_and(|w| w.chars().skip(1).any(|c| c.is_ascii_uppercase() || c.is_ascii_digit()) || w.len() == 1 || w.chars().all(|c| !c.is_ascii_lowercase()));
    if a.text.trim_end().ends_with('.') || a.text.is_empty() || keep || is_proper(t) {
        a.text.push_str(t);
    } else {
        a.text.extend(f.to_lowercase());
        a.text.push_str(c.as_str());
    }
}

fn is_proper(t: &str) -> bool {
    let w = t.split([' ', ',']).next().unwrap_or("");
    phrase::NAMES.iter().chain(phrase::UNITS.iter()).any(|n| n.2.split(' ').next() == Some(w))
        || ["Skyward", "Ascot", "Reach", "Viper", "Golf", "MAYDAY", "PAN", "QNH", "ILS", "VFR", "IFR", "I"].contains(&w)
}

fn short(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { format!("{}…", s.chars().take(n).collect::<String>()) }
}

/// "Turn left heading 090" / "fly heading 090".
fn turn_words(u: &mut Utt, hdg: f32, turn: Option<bool>) {
    let h = phrase::heading_value(hdg);
    let digits = phrase::spell(&format!("{h:03}"));
    match turn {
        Some(true) => u.say_as(&format!("turn right heading {digits}"), &format!("turn right heading {h:03}")),
        Some(false) => u.say_as(&format!("turn left heading {digits}"), &format!("turn left heading {h:03}")),
        None => u.say_as(&format!("fly heading {digits}"), &format!("fly heading {h:03}")),
    };
}

/// A crossing named "09/27" → the end facing us is not known: say the lower ident.
fn pick_end(name: &str, _dep: &crate::scene::Runway) -> String {
    name.split('/').next().unwrap_or(name).to_string()
}

/// The "via" list of a departure plan.
fn route_via(d: &DepPlan) -> Vec<String> {
    d.via.clone()
}

/// A squawk from the UK conspicuity-free ranges (octal digits, not 7xxx / 0000 / 2000).
fn new_squawk(rng: &mut u32) -> u32 {
    *rng ^= *rng << 13;
    *rng ^= *rng >> 17;
    *rng ^= *rng << 5;
    let mut r = *rng;
    let mut code = 0;
    for i in 0..4 {
        let d = if i == 0 { 1 + r % 6 } else { r % 8 };
        code = code * 10 + d;
        r /= 8;
    }
    code
}

/// What's wrong, from the active failures (systems/).
fn emergency_words(ac: &Aircraft) -> &'static str {
    use crate::systems::failures::Kind as K;
    let s = &ac.sys;
    if !s.init {
        return "engine failure";
    }
    let f = &s.fail;
    if f.any_of(K::EngineFire) {
        "engine fire"
    } else if f.any_of(K::ElecFire) {
        "smoke in the cockpit"
    } else if f.any_of(K::Flameout) || f.any_of(K::PartialPower) || f.any_of(K::PropOverspeed) || f.any_of(K::CarbIce) {
        "engine failure"
    } else if f.any_of(K::FuelLeak) || f.any_of(K::FuelPump) {
        "fuel leak"
    } else if f.any_of(K::Generator) {
        "electrical failure"
    } else if f.any_of(K::Hydraulic) {
        "hydraulic failure"
    } else if f.any_of(K::GearExtend) {
        "gear failure"
    } else if f.any_of(K::BirdStrike) {
        "bird strike"
    } else {
        "engine failure"
    }
}

/// Persons on board by type.
fn persons(def: &AircraftDef) -> u32 {
    match def.id {
        "kestrel" => 2,
        "lancer" => 1,
        "aerolink" => 52,
        "meridian190" => 88,
        "meridian320" => 156,
        "hercules" => 6,
        "galaxy" => 9,
        "vulcan" => 5,
        "triple7" => 380,
        "crj700" => 70,
        "citationx" => 10,
        "concorde" => 100,
        "extra500" => 4,
        "zlin" => 1,
        "otter" => 6,
        "dc3" => 24,
        _ => 2,
    }
}

/// Voice for a unit: varies by airport so neighbouring units sound different.
fn voice_for(airport: &str, fac: Fac) -> Voice {
    use Voice::*;
    let pool: &[Voice] = match fac {
        Fac::Atis => &[Emma, Aoede],
        Fac::Delivery => &[Emma, Aoede, River],
        Fac::Ground => &[Onyx, Aoede, Emma],
        Fac::Tower => &[Fable, Onyx, Emma],
        _ => &[River, Aoede, Fable, Onyx],
    };
    pool[(hash(airport) as usize / 7) % pool.len()]
}

/// The stations of one airport from its frequency records.
pub fn stations_of(ap: &crate::airports::Airport, fs: &[freqs::Freq]) -> Vec<Station> {
    let (an_s, an_t) = atis::station_name(ap);
    let mut out: Vec<Station> = Vec::new();
    for f in fs {
        if out.iter().any(|s| s.fac == f.fac) && f.fac != Fac::Atis {
            // One frequency per unit (the first published).
            continue;
        }
        if out.iter().any(|s| s.fac == Fac::Atis) && f.fac == Fac::Atis {
            continue;
        }
        let mut d = f.desc.to_ascii_uppercase();
        if let Some(u) = phrase::APPROACH_UNITS.iter().find(|u| u.0 == ap.code()) {
            if f.fac.is_radar() && !d.contains(u.1) {
                d = format!("{} {d}", u.1);
            }
        }
        let (ns, nt) = match phrase::UNITS.iter().find(|u| d.split([' ', '/']).any(|w| w == u.0)) {
            Some(u) if f.fac.is_radar() => (u.1.to_string(), u.2.to_string()),
            _ => (an_s.clone(), an_t.clone()),
        };
        let mut voice = voice_for(&ap.ident, f.fac);
        // Ground and tower of one airport never share a voice.
        if out.iter().any(|s| s.voice == voice && s.fac != Fac::Atis) {
            let i = Voice::CONTROLLERS.iter().position(|v| *v == voice).unwrap_or(0);
            voice = Voice::CONTROLLERS[(i + 1) % Voice::CONTROLLERS.len()];
        }
        // "Thames Radar" (London City's approach is Thames Radar).
        let fac = if ns != an_s && f.fac == Fac::Approach { Fac::Radar } else { f.fac };
        let word = fac.word();
        out.push(Station {
            airport: ap.ident.clone(),
            code: ap.code().to_string(),
            fac: f.fac,
            khz: f.khz,
            spoken: format!("{ns} {word}"),
            text: format!("{nt} {}", if f.fac == Fac::Atis { "Information".into() } else { cap(word) }),
            ll: (ap.lat, ap.lon),
            elev_m: ap.elev_ft * FT,
            voice,
        });
    }
    out
}

// ---------------------------------------------------------------- sim glue

/// UTC (h, m) from the sim's local clock.
fn utc_of(sim: &crate::sim::Sim, world: &World) -> (u32, u32) {
    let off = world.wx.utc_offset_s.unwrap_or_else(|| crate::weather::guess_offset_s(sim.home_ll.0, sim.home_ll.1, crate::weather::unix_now())) as f32 / 3600.0;
    let h = (sim.hours - off).rem_euclid(24.0);
    (h.floor() as u32 % 24, ((h.fract() * 60.0).floor() as u32).min(59))
}

macro_rules! with_ctx {
    ($sim:expr, $world:expr, |$atc:ident, $nav:ident, $ctx:ident| $body:expr) => {{
        let utc = utc_of($sim, $world);
        let crate::sim::Sim { atc: $atc, nav: $nav, ac, defs, sel, ground_y, bot, paused, .. } = $sim;
        let bot = bot.as_ref().map(|b| (b.phase(), b.focus_runway()));
        let dest = $nav.plan.dest.as_ref().map(|d| d.ident.clone());
        let $ctx = Ctx { world: $world, ac, def: &defs[*sel], ground_y: *ground_y, bot, utc, dest, paused: *paused };
        $body
    }};
}

/// One frame of ATC for the sim (after the physics). `dt` = real seconds.
pub fn step(sim: &mut crate::sim::Sim, world: &World, dt: f32) {
    with_ctx!(sim, world, |atc, nav, ctx| atc.update(nav, &ctx, dt));
    let hold = sim.atc.bot_hold;
    if let Some(b) = sim.bot.as_mut() {
        b.set_atc_wait(hold);
    }
    // The autopilot follows the read-back instructions (assist).
    for c in std::mem::take(&mut sim.atc.ap_cmds) {
        if sim.bot.is_some() || !sim.ap.engaged {
            continue;
        }
        match c {
            ApCmd::Hdg(h) => {
                if sim.ap.lat != crate::autopilot::Lat::Loc {
                    sim.ap.lat = crate::autopilot::Lat::Hdg;
                }
                sim.ap.hdg = Some(crate::nav::geo::wrap_2pi(h.to_radians()) - if h > 180.0 { std::f32::consts::TAU } else { 0.0 });
            }
            ApCmd::Alt(a) => {
                if sim.ap.vert != crate::autopilot::Vert::Gs {
                    sim.ap.alt = Some(a * FT);
                    sim.ap.vs = None;
                }
            }
            ApCmd::Appr => {
                if !sim.ap.appr {
                    sim.ap.toggle_appr();
                }
            }
        }
    }
}

/// A request from the comms menu.
pub fn request(sim: &mut crate::sim::Sim, world: &World, r: Req) {
    with_ctx!(sim, world, |atc, nav, ctx| atc.request(r, nav, &ctx));
}

/// The comms menu items now.
pub fn menu(sim: &crate::sim::Sim, world: &World) -> Vec<(Req, String)> {
    sim.atc.menu(world, &sim.ac, sim.ground_y, sim.nav.radios.com[0].active)
}

/// Debugging / test fixtures: write each loaded airport layout's taxiways,
/// holds and runways (airport frame) to `<dir>/<ident>.taxi` (`SKYWARD_ATC_DUMP=dir`).
#[cfg(not(target_arch = "wasm32"))]
fn dump_layouts(world: &World, dir: &std::path::Path) {
    let Some(adb) = world.airport_db.as_ref() else { return };
    for r in &world.runways {
        if r.airport.is_empty() {
            continue;
        }
        let path = dir.join(format!("{}.taxi", r.airport));
        let Some(l) = world.airport_detail.layout(&r.airport) else { continue };
        let Some(ap) = adb.get(&r.airport) else { continue };
        let mut s = format!("F\t{}\t{}\t{}\n", ap.ident, ap.lat, ap.lon);
        let frame = crate::osm::Proj::new(ap.lat, ap.lon);
        for rw in ap.runways.iter().filter_map(|x| crate::airports::to_world(ap, x, &frame)) {
            s += &format!("R\t{}\t{}\t{:.1},{:.1}\t{:.1},{:.1}\t{:.0}\n", rw.ident_a, rw.ident_b, rw.a.x, rw.a.y, rw.b.x, rw.b.y, rw.width);
        }
        for (pl, n) in &l.taxiways {
            s += &format!("T\t{}\t{}\n", n, pl.iter().map(|p| format!("{:.1},{:.1}", p.x, p.y)).collect::<Vec<_>>().join(";"));
        }
        for h in &l.holds {
            s += &format!("H\t{}\t{}\t{:.1},{:.1}\n", h.taxiway, h.runway, h.at.x, h.at.y);
        }
        let old = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
        if (s.len() as u64) > old {
            let _ = std::fs::write(&path, s);
        }
    }
}

/// Demo / test helper (`--audio-script "jump=lineup"`): put the aircraft on
/// the departure runway, lined up (the taxi done in fast-forward).
pub fn jump_to_runway(sim: &mut crate::sim::Sim, world: &World) {
    let rwy = sim.atc.dep.as_ref().map(|d| (d.airport.clone(), d.runway.clone()));
    let Some((ap, id)) = rwy else { return };
    let Some(rw) = world.runways.iter().find(|r| r.airport == ap && (r.ident_a == id || r.ident_b == id)) else { return };
    let (thr, dir) = crate::airports::threshold(rw, rw.ident_b == id);
    let def = sim.def().clone();
    let p = thr + dir * (25.0 + def.gear.iter().map(|g| g.pos.z.abs()).fold(0.0, f32::max));
    let sys = sim.ac.sys.clone();
    sim.ac = Aircraft::on_runway(&def, p, dir.x.atan2(-dir.y), world);
    sim.ac.sys = sys;
    sim.ctl.throttle = 0.0;
    sim.ctl.brakes = 1.0;
}
