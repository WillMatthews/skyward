//! Navigation tests: ILS geometry and deviations, receiver range / line of
//! sight, Morse timing, flight-plan sequencing, autopilot LOC / VOR / LNAV
//! captures, and a coupled ILS approach and autoland at EGLL 27R.

use super::db::{self, Kind, Navaid};
use super::fms::{ApprKind, Wpt, WptKind};
use super::geo::{self, LL, NM};
use super::morse;
use super::radio::RadioEnv;
use super::state::NavState;
use crate::autopilot::{Autopilot, Lat, Vert};
use crate::flight::{Aircraft, AircraftDef, Environment, KT, find, trimmed_controls_at};
use crate::osm::Proj;
use glam::{Vec2, Vec3, vec2, vec3};

const DT: f32 = 1.0 / 120.0;
const FT: f32 = 0.3048;

const AIRPORTS: &str = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\",\"continent\",\"iso_country\",\"iso_region\",\"municipality\",\"scheduled_service\",\"icao_code\",\"iata_code\",\"gps_code\",\"local_code\",\"home_link\",\"wikipedia_link\",\"keywords\"
2430,\"EGLC\",\"medium_airport\",\"London City Airport\",51.505299,0.055278,19,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLC\",\"LCY\",\"EGLC\",,,,
2434,\"EGLL\",\"large_airport\",\"London Heathrow Airport\",51.4706,-0.461941,83,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLL\",\"LHR\",\"EGLL\",,,,
9,\"EGKB\",\"medium_airport\",\"London Biggin Hill Airport\",51.3308,0.0325,598,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"no\",\"EGKB\",\"BQH\",\"EGKB\",,,,
";
const RUNWAYS: &str = "\"id\",\"airport_ref\",\"airport_ident\",\"length_ft\",\"width_ft\",\"surface\",\"lighted\",\"closed\",\"le_ident\",\"le_latitude_deg\",\"le_longitude_deg\",\"le_elevation_ft\",\"le_heading_degT\",\"le_displaced_threshold_ft\",\"he_ident\",\"he_latitude_deg\",\"he_longitude_deg\",\"he_elevation_ft\",\"he_heading_degT\",\"he_displaced_threshold_ft\"
239448,2430,\"EGLC\",4948,98,\"CON\",1,0,\"09\",51.505576,0.044333,16,93,,\"27\",51.504894,0.066026,19,273,
239399,2434,\"EGLL\",12799,164,\"ASP\",1,0,\"09L\",51.47749,-0.489439,79,90,1007,\"27R\",51.477681,-0.433227,78,270,
239398,2434,\"EGLL\",12001,164,\"ASP\",1,0,\"09R\",51.46478,-0.486808,75,90,1013,\"27L\",51.464957,-0.434048,77,270,
5,9,\"EGKB\",5928,148,\"ASP\",1,0,\"03\",,,,,,\"21\",,,,,
";
const NAVAIDS: &str = "\"id\",\"filename\",\"ident\",\"name\",\"type\",\"frequency_khz\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\",\"iso_country\",\"dme_frequency_khz\",\"dme_channel\",\"dme_latitude_deg\",\"dme_longitude_deg\",\"dme_elevation_ft\",\"slaved_variation_deg\",\"magnetic_variation_deg\",\"usageType\",\"power\",\"associated_airport\"
86050,\"Biggin_VOR-DME_GB\",\"BIG\",\"Biggin\",\"VOR-DME\",115100,51.33089828491211,0.034811001271009445,590,\"GB\",115100,\"098X\",,,,-3.001,-2.117,\"BOTH\",\"HIGH\",\"EGKB\"
86222,\"Bovingdon_VOR-DME_GB\",\"BNN\",\"Bovingdon\",\"VOR-DME\",113750,51.72610092163086,-0.5497220158576965,500,\"GB\",113750,\"084Y\",,,,-4.001,-2.284,\"HI\",\"HIGH\",
90156,\"Lambourne_VOR-DME_GB\",\"LAM\",\"Lambourne\",\"VOR-DME\",115600,51.64609909057617,0.15166699886322021,200,\"GB\",115600,\"103X\",,,,-3.001,-2.119,\"BOTH\",\"HIGH\",
1,\"Test_NDB_GB\",\"LCY\",\"London City\",\"NDB\",322,51.5048,0.0600,20,\"GB\",,,,,,,,\"LO\",\"LOW\",\"EGLC\"
";

fn adb() -> crate::airports::Db {
    crate::airports::Db::new(crate::airports::parse_csv(AIRPORTS, RUNWAYS), 0)
}

/// Test world: a local frame, flat ground at `ground` (or a ridge), runways at 0 m.
struct Env {
    proj: Proj,
    ground: f32,
    ridge: Option<(Vec2, Vec2, f32)>,
}

impl Env {
    fn at(ll: LL) -> Env {
        Env { proj: Proj::new(ll.0, ll.1), ground: 0.0, ridge: None }
    }
}

impl RadioEnv for Env {
    fn to_world(&self, ll: LL) -> Vec2 {
        self.proj.to_world(ll.0, ll.1)
    }
    fn to_ll(&self, p: Vec2) -> LL {
        self.proj.to_latlon(p)
    }
    fn ground(&self, p: Vec2) -> Option<f32> {
        if let Some((a, b, h)) = self.ridge {
            let d = b - a;
            let t = ((p - a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
            if (a + d * t).distance(p) < 800.0 {
                return Some(h);
            }
        }
        Some(self.ground)
    }
    fn runway_elev(&self, _: &str, _: &str) -> Option<f32> {
        Some(self.ground)
    }
}

struct Flat;
impl Environment for Flat {
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
}

fn egll_27r() -> Navaid {
    let a = adb();
    db::ils_at(a.get("EGLL").unwrap()).into_iter().find(|n| n.ils.as_ref().unwrap().runway == "27R").unwrap()
}

#[test]
fn navaid_csv_parse_and_cache_roundtrip() {
    let list = db::parse_csv(NAVAIDS);
    assert_eq!(list.len(), 4);
    let big = &list[0];
    assert_eq!((big.ident.as_str(), big.kind, big.freq_khz, big.freq_label()), ("BIG", Kind::VorDme, 115_100, "115.10".to_string()));
    assert!((big.elev_m - 590.0 * FT).abs() < 0.1 && big.range_nm == 130.0 && big.airport == "EGKB");
    let ndb = &list[3];
    assert!(ndb.kind.is_ndb() && ndb.freq_label() == "322" && ndb.range_nm == 25.0);
    let back = db::decode(&db::encode(&list)).unwrap();
    assert_eq!(back.len(), 4);
    assert_eq!(back[0].ident, "BIG");
    assert!((back[0].lat - big.lat).abs() < 1e-5);
    let d = db::NavDb::new(list);
    assert_eq!(d.find("lam", (51.5, 0.0)).map(|n| n.freq_khz), Some(115_600));
    assert_eq!(d.near((51.33, 0.03), 5.0).len(), 1);
}

#[test]
fn ils_frequencies_and_synthetic_records() {
    let ch = db::ils_channels();
    assert_eq!(ch.len(), 40);
    assert!(ch.iter().all(|f| db::is_ils_freq(*f)));
    assert!(!db::is_ils_freq(115_100) && !db::is_ils_freq(108_200));
    // EGLL 27R: the published facts.
    let i = egll_27r();
    let ils = i.ils.as_ref().unwrap();
    assert_eq!((i.ident.as_str(), i.freq_khz, i.kind, ils.synthetic), ("IRR", 110_300, Kind::Ils, false));
    assert!((ils.course.to_degrees() - 269.7).abs() < 0.5, "{}", ils.course.to_degrees());
    assert!((ils.gs_angle.to_degrees() - 3.0).abs() < 1e-4);
    // Displaced threshold 1007 ft from the 09L end... no: 27R has none; length ≈ 3.9 km.
    assert!((ils.length - 3901.0).abs() < 30.0, "{}", ils.length);
    // EGLC 27: 5.5° glidepath.
    let a = adb();
    let lc = db::ils_at(a.get("EGLC").unwrap());
    assert_eq!(lc.len(), 2);
    assert!(lc.iter().all(|n| (n.ils.as_ref().unwrap().gs_angle.to_degrees() - 5.5).abs() < 1e-3));
    // EGKB has no surveyed ends → no ILS; both ends of a synthetic runway share a channel.
    assert!(db::ils_at(a.get("EGKB").unwrap()).is_empty());
    let mut ap = a.get("EGLC").unwrap().clone();
    ap.icao = "XXXX".into();
    ap.ident = "XXXX".into();
    let s = db::ils_at(&ap);
    assert!(s.iter().all(|n| n.ils.as_ref().unwrap().synthetic));
    assert_eq!(s[0].freq_khz, s[1].freq_khz);
    assert!(s[0].ident.starts_with('I') && s[0].ident != s[1].ident, "{} {}", s[0].ident, s[1].ident);
}

/// Localizer / glideslope deviations of the EGLL 27R ILS at known offsets.
#[test]
fn loc_gs_deviation_maths() {
    let i = egll_27r();
    let ils = i.ils.clone().unwrap();
    let env = Env::at(ils.thr);
    let mut nav = NavState::default();
    nav.add_stations([i.clone()]);
    nav.radios.nav[0].tune(110_300);
    let (_, dir) = super::radio::loc_frame(&env, &ils);
    let right = geo::right_of(dir);
    let thr = env.to_world(ils.thr);
    let gs_site = env.to_world(ils.gs.unwrap());
    let along_gs = (gs_site - thr).dot(dir);
    // 6 NM out, 100 m right of the centreline, 20 m above the glidepath.
    let d = 6.0 * NM;
    let p = thr - dir * d + right * 100.0;
    let h = (d + along_gs) * ils.gs_angle.tan() + 20.0;
    let mut ac = Aircraft::airborne(&find("meridian320").unwrap(), vec3(p.x, h, p.y), ils.course, 70.0);
    ac.vel = Vec3::ZERO;
    let def = find("meridian320").unwrap();
    nav.step(&env, &ac, &def, None, None, 0.6);
    let o = &nav.radios.nav[0].out;
    assert!(o.valid && o.loc && o.gs_valid, "{o:?}");
    assert_eq!(o.ident, "IRR");
    // Right of course → the course is left: negative; |angle| = atan(100 / distance to the antenna).
    let d_ant = env.to_world(ils.loc).distance(p);
    let want = -(100.0 / d_ant).atan();
    assert!((o.dev - want).abs() < 2e-4, "{} vs {}", o.dev, want);
    assert!((o.xtk + 100.0).abs() < 1.0, "xtk {}", o.xtk);
    assert!((o.cdi - want / ils.half_width).abs() < 1e-3);
    // Above the path → fly down; 20 m at ~11.4 km ≈ 0.1°.
    assert!(o.gs < 0.0 && (o.gp_err - 20.0).abs() < 1.0, "{} {}", o.gs, o.gp_err);
    let want_gs = -(20.0 / (d + along_gs)).atan() * ils.gs_angle.cos().powi(2);
    assert!((o.gs_dev - want_gs).abs() < 3e-4, "{} {}", o.gs_dev, want_gs);
    // DME at the glideslope: ~ 6 NM + the touchdown zone.
    let dme = o.dme.unwrap() / NM;
    assert!((dme - (d + along_gs) / NM).abs() < 0.1, "{dme}");
    assert!((o.to_thr - d).abs() < 5.0);
    // Full-scale beyond the half width; out of the ±35° sector: no signal.
    let p2 = thr - dir * d + right * 5000.0;
    let ac2 = Aircraft::airborne(&def, vec3(p2.x, h, p2.y), ils.course, 70.0);
    nav.step(&env, &ac2, &def, None, None, 0.6);
    let o = &nav.radios.nav[0].out;
    assert!(o.valid && o.cdi == -1.0 && !o.gs_valid, "{o:?}");
    let p3 = thr - right * 9000.0;
    let ac3 = Aircraft::airborne(&def, vec3(p3.x, 600.0, p3.y), ils.course, 70.0);
    nav.step(&env, &ac3, &def, None, None, 0.6);
    assert!(!nav.radios.nav[0].out.valid);
    // Unpowered or failed: flags.
    nav.step(&env, &ac, &def, None, None, 0.6);
    assert!(nav.radios.nav[0].out.valid);
    nav.equipped = false;
    nav.step(&env, &ac, &def, None, None, 0.6);
    assert!(!nav.radios.nav[0].out.valid && nav.radios.nav[0].out.dme.is_none());
}

/// VOR / DME / NDB reception: service range, radio horizon, terrain shadow.
#[test]
fn receiver_range_and_line_of_sight() {
    let list = db::parse_csv(NAVAIDS);
    let big = list[0].clone();
    // Ground at the station's elevation; altitudes below are above it.
    let mut env = Env::at(big.ll());
    env.ground = big.elev_m;
    let g0 = big.elev_m;
    let def = find("kestrel").unwrap();
    let mut nav = NavState::default();
    nav.add_stations(list.clone());
    nav.radios.nav[0].tune(115_100);
    nav.radios.nav[0].obs = 0.0;
    let site = env.to_world(big.ll());
    let mut at = |d_nm: f32, alt: f32, env: &Env| {
        let p = site + vec2(0.0, d_nm * NM); // south of the station
        let ac = Aircraft::airborne(&def, vec3(p.x, alt + g0, p.y), 0.0, 50.0);
        nav.step(env, &ac, &def, None, None, 0.6);
        nav.radios.nav[0].out.clone()
    };
    // 40 NM south at 3000 ft: in range and above the horizon; radial 180, TO for OBS 360.
    let o = at(40.0, 914.0, &env);
    assert!(o.valid && o.ident == "BIG", "{o:?}");
    assert!((o.radial.to_degrees() - 180.0).abs() < 0.5 && o.to == Some(true));
    assert!((o.dme.unwrap() / NM - 40.0).abs() < 0.3);
    // 40 NM at 300 ft: below the radio horizon (≈ 34 NM for 300 ft + the antenna).
    assert!(!at(40.0, 91.0, &env).valid);
    // 140 NM at FL250: beyond the service volume (130 NM high-altitude class).
    assert!(!at(140.0, 7620.0, &env).valid);
    // A ridge between: shadowed at 2000 ft, clear at 6000 ft.
    let mut env2 = Env::at(big.ll());
    env2.ground = g0;
    env2.ridge = Some((site + vec2(-5000.0, 15.0 * NM), site + vec2(5000.0, 15.0 * NM), g0 + 720.0));
    assert!(!at(30.0, 610.0, &env2).valid);
    assert!(at(30.0, 1830.0, &env2).valid);
    // ADF on the NDB: bearing to the station.
    let ndb = list[3].clone();
    let env3 = Env::at(ndb.ll());
    nav.radios.adf.active = 322;
    let p = env3.to_world(ndb.ll()) + vec2(10_000.0, 0.0); // east of it
    let ac = Aircraft::airborne(&def, vec3(p.x, 600.0, p.y), 0.0, 50.0);
    nav.step(&env3, &ac, &def, None, None, 0.6);
    let a = &nav.radios.adf.out;
    assert!(a.valid && a.ident == "LCY" && (a.bearing.to_degrees() - 270.0).abs() < 0.5, "{a:?}");
    // 30 NM: beyond a low-power NDB's 25 NM.
    let p = env3.to_world(ndb.ll()) + vec2(30.0 * NM, 0.0);
    let ac = Aircraft::airborne(&def, vec3(p.x, 600.0, p.y), 0.0, 50.0);
    nav.step(&env3, &ac, &def, None, None, 0.6);
    assert!(!nav.radios.adf.out.valid);
}

#[test]
fn morse_timing_and_wav() {
    // I = "..", R = ".-.": IRR = .. / .-. / .-.
    let (on, len) = morse::elements("IRR");
    assert_eq!(on.len(), 2 + 3 + 3);
    let d = morse::DOT;
    // Dot 1, gap 1, dot 1, letter gap 3, dot, gap, dash 3 ...
    assert!((on[0].1 - on[0].0 - d).abs() < 1e-6 && (on[1].0 - on[0].1 - d).abs() < 1e-6);
    assert!((on[2].0 - on[1].1 - 3.0 * d).abs() < 1e-6);
    assert!((on[3].1 - on[3].0 - 3.0 * d).abs() < 1e-6);
    // Total: I(3) + 3 + R(7) + 3 + R(7) = 23 dots.
    assert!((len - 23.0 * d).abs() < 1e-5, "{len}");
    assert!((d - 0.1714).abs() < 1e-3, "7 wpm");
    assert_eq!(morse::decode(&on, d), "IRR");
    let k = morse::Keyer::new("IRR");
    assert!(k.key(0.01) && !k.key(d * 1.5) && k.key(d * 2.5));
    assert!(k.key(k.period() + 0.01), "repeats");
    // Rendered audio decodes back (envelope detection on the PCM).
    let sr = 22_050;
    let pcm = morse::render("BIG", sr, 6.0);
    let det = morse::detect(&pcm, sr);
    assert_eq!(morse::decode(&det, d), "BIG", "{det:?}");
    // Tone frequency: zero crossings in the first dash of B (−...).
    let n = (d * 2.0 * sr as f32) as usize;
    let s = &pcm[(0.02 * sr as f32) as usize..n];
    let zc = s.windows(2).filter(|w| (w[0] >= 0) != (w[1] >= 0)).count() as f32;
    let hz = zc / 2.0 / (s.len() as f32 / sr as f32);
    assert!((hz - morse::TONE_HZ).abs() < 15.0, "{hz}");
}

fn env_egll() -> (Env, Navaid) {
    let i = egll_27r();
    (Env::at(i.ils.as_ref().unwrap().thr), i)
}

/// A route with a 90° turn: sequencing (fly-by), distances and predictions.
#[test]
fn route_sequencing_and_predictions() {
    let (env, _) = env_egll();
    let def = find("meridian320").unwrap();
    let mut nav = NavState::new(&def);
    let ll = |x: f32, z: f32| env.to_ll(vec2(x, z));
    nav.plan.set_origin(Some(Wpt::new("ORIG", ll(0.0, 20_000.0), WptKind::Airport)));
    nav.plan.insert(0, Wpt::new("WP1", ll(0.0, 0.0), WptKind::User));
    nav.plan.insert(1, Wpt::new("WP2", ll(20_000.0, 0.0), WptKind::User));
    nav.plan.insert(2, Wpt::new("WP3", ll(20_000.0, -20_000.0), WptKind::User));
    assert_eq!(nav.plan.legs.len(), 3);
    // Predictions from the origin at 100 m/s with 5,000 kg burning 2,400 kg/h.
    let p0 = vec2(0.0, 20_000.0);
    let pr = nav.plan.predictions(&env, p0, 100.0, Some(5000.0), 2400.0);
    assert_eq!(pr.len(), 3);
    assert!((pr[2].dist - 60_000.0).abs() < 50.0 && (pr[2].eta - 600.0).abs() < 1.0);
    assert!((pr[2].efob.unwrap() - 4600.0).abs() < 2.0);
    assert!((pr[1].course.to_degrees() - 90.0).abs() < 0.2);
    // Fly it with LNAV.
    let mut ac = Aircraft::airborne(&def, vec3(p0.x, 1500.0, p0.y), 0.0, 230.0 * KT);
    let mut ctl = trimmed_controls_at(&def, 230.0 * KT, 1500.0);
    ctl.gear_down = false;
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    ap.toggle_lat(Lat::Lnav);
    let mut max_x: f32 = 0.0;
    let mut seq = Vec::new();
    for i in 0..(400.0 / DT) as usize {
        nav.step(&env, &ac, &def, None, None, DT);
        ap.nav = nav.ap_input(&env, &ac, &def, 0.0);
        assert!(ap.update(&def, &ac, &mut ctl, DT));
        ac.step(&def, &ctl, &Flat, DT);
        if seq.last() != Some(&nav.plan.active) {
            seq.push(nav.plan.active);
        }
        // Cross-track from the polyline (turns cut the corner by design: measure on the legs' middles).
        let p = vec2(ac.pos.x, ac.pos.z);
        if i > (10.0 / DT) as usize {
            let on_leg = |a: Vec2, b: Vec2| {
                let d = b - a;
                let t = (p - a).dot(d) / d.length_squared();
                (0.2..0.8).contains(&t).then(|| (p - (a + d * t)).length())
            };
            for (a, b) in [(vec2(0.0, 20_000.0), vec2(0.0, 0.0)), (vec2(0.0, 0.0), vec2(20_000.0, 0.0)), (vec2(20_000.0, 0.0), vec2(20_000.0, -20_000.0))] {
                if let Some(x) = on_leg(a, b) {
                    max_x = max_x.max(x);
                }
            }
        }
    }
    eprintln!("LNAV: sequence {seq:?}, max cross-track mid-leg {max_x:.0} m, lat {:?}", ap.lat);
    assert_eq!(seq, vec![0, 1, 2]);
    assert_eq!(ap.lat, Lat::Lnav);
    assert!(max_x < 60.0, "{max_x}");
    // Direct-to, delete, move.
    let here = env.to_ll(vec2(ac.pos.x, ac.pos.z));
    nav.plan.direct_to(0, here);
    assert_eq!((nav.plan.active, nav.plan.from.is_some()), (0, true));
    nav.plan.remove(0);
    assert_eq!(nav.plan.legs[0].ident, "WP2");
    nav.plan.move_to(0, ll(25_000.0, 0.0));
    assert_eq!(nav.plan.legs[0].kind, WptKind::User);
    assert!(env.to_world(nav.plan.legs[0].ll).distance(vec2(25_000.0, 0.0)) < 1.0);
}

/// Approach fixes and the VNAV path.
#[test]
fn approach_legs_and_vnav_path() {
    let (env, i) = env_egll();
    let def = find("meridian320").unwrap();
    let mut nav = NavState::new(&def);
    nav.plan.cruise = 3000.0;
    nav.plan_ils(&i, ApprKind::Ils);
    let ids: Vec<&str> = nav.plan.legs.iter().map(|w| w.ident.as_str()).collect();
    assert_eq!(ids, ["CI27R", "FF27R", "RW27R"]);
    // FAF at 5 NM: 1600 ft above the threshold (3°), rounded to 100 ft.
    // (MSL: the threshold at the airport's 83 ft.)
    let faf = nav.plan.legs[1].alt.unwrap() / FT;
    assert!((faf - 83.0 - 1600.0).abs() < 60.0, "{faf}");
    let thr = env.to_world(i.ils.as_ref().unwrap().thr);
    let dir = geo::dir_of(i.ils.as_ref().unwrap().course);
    let p = thr - dir * 15.0 * NM;
    nav.plan.active = 0;
    let v = nav.plan.vnav(&env, p).unwrap();
    assert!((v.to_thr / NM - 15.0).abs() < 0.05, "{}", v.to_thr / NM);
    let want = 83.0 * FT + 50.0 * FT + 15.0 * NM * 3f32.to_radians().tan();
    assert!((v.alt - want).abs() < 2.0, "{} {}", v.alt, want);
    // Far out: capped at the cruise level; T/D ≈ 3000 m / tan 3° from the threshold.
    let v = nav.plan.vnav(&env, thr - dir * 40.0 * NM).unwrap();
    assert_eq!(v.alt, 3000.0);
    assert!((v.to_tod / NM - (40.0 - (3000.0 - 83.0 * FT - 15.24) / 3f32.to_radians().tan() / NM)).abs() < 0.05);
    // Auto-tune: NAV 1 on the localizer, course set.
    let ac = Aircraft::airborne(&def, vec3(p.x, 900.0, p.y), i.ils.as_ref().unwrap().course, 80.0);
    nav.step(&env, &ac, &def, None, None, 0.6);
    assert_eq!(nav.radios.nav[0].active, 110_300);
    assert!((nav.radios.nav[0].obs - i.ils.as_ref().unwrap().course).abs() < 1e-5);
}

/// Fly a localizer intercept at `cut` degrees from `d_nm` out; return
/// (max overshoot m past the centreline, final |xtk| m, captured).
fn intercept(id: &str, cut: f32, d_nm: f32, kt: f32, alt: f32) -> (f32, f32, f32, bool) {
    let (env, i) = env_egll();
    let ils = i.ils.clone().unwrap();
    let def: AircraftDef = find(id).unwrap();
    let mut nav = NavState::new(&def);
    nav.plan_ils(&i, ApprKind::Ils);
    let dir = geo::dir_of(ils.course);
    let right = geo::right_of(dir);
    let thr = env.to_world(ils.thr);
    // Start 3.5 km left of the course, heading `cut` degrees towards it.
    let start = thr - dir * d_nm * NM - right * 3500.0;
    let hdg = geo::wrap_2pi(ils.course + cut.to_radians());
    let mut ac = Aircraft::airborne(&def, vec3(start.x, alt, start.y), hdg, kt * KT);
    let mut ctl = trimmed_controls_at(&def, kt * KT, alt);
    ctl.gear_down = ac.gear_pos() > 0.5;
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    ap.toggle_lat(Lat::Loc);
    let (mut over, mut crossed, mut captured_at) = (0.0f32, false, f32::NAN);
    let half = ils.half_width;
    for k in 0..(240.0 / DT) as usize {
        nav.step(&env, &ac, &def, None, None, DT);
        ap.nav = nav.ap_input(&env, &ac, &def, 0.0);
        assert!(ap.update(&def, &ac, &mut ctl, DT));
        ac.step(&def, &ctl, &Flat, DT);
        let o = &nav.radios.nav[0].out;
        if ap.lat == Lat::Loc && captured_at.is_nan() {
            captured_at = k as f32 * DT;
        }
        // xtk > 0: course to the right (we start left of it).
        if o.xtk < 0.0 {
            crossed = true;
        }
        if crossed {
            over = over.max(-o.xtk);
        }
    }
    let o = &nav.radios.nav[0].out;
    let d_ant = env.to_world(ils.loc).distance(vec2(ac.pos.x, ac.pos.z));
    let dot_m = d_ant * (half / 2.0).tan();
    eprintln!(
        "{id} LOC intercept {cut}° at {d_nm} NM: captured after {captured_at:.0} s, overshoot {over:.1} m ({:.2} dot), final xtk {:.1} m, modes {:?}/{}",
        over / dot_m,
        o.xtk,
        ap.lat,
        ap.capturing
    );
    (over, over / dot_m, o.xtk.abs(), ap.lat == Lat::Loc)
}

#[test]
fn localizer_capture_30_and_45_degrees() {
    for (id, kt, alt) in [("meridian320", 180.0, 900.0), ("aerolink", 160.0, 900.0), ("kestrel", 95.0, 600.0)] {
        for cut in [30.0, 45.0] {
            let (over, dots, xtk, captured) = intercept(id, cut, 16.0, kt, alt);
            assert!(captured, "{id} {cut}°");
            assert!(dots < 0.2 && over < 60.0, "{id} {cut}°: overshoot {over:.1} m ({dots:.2} dot)");
            assert!(xtk < 8.0, "{id} {cut}°: settled {xtk}");
        }
    }
}

/// VOR radial tracking: intercept the BIG 360° radial outbound.
#[test]
fn vor_radial_capture() {
    let list = db::parse_csv(NAVAIDS);
    let big = list[0].clone();
    let env = Env::at(big.ll());
    let def = find("kestrel").unwrap();
    let mut nav = NavState::new(&def);
    nav.add_stations(list);
    nav.radios.nav[0].tune(115_100);
    nav.radios.nav[0].obs = 0.0;
    // 8 km south-east of the station, heading 330 (30° cut onto the 360 radial outbound... from the east).
    let start = env.to_world(big.ll()) + vec2(4000.0, -6000.0);
    let mut ac = Aircraft::airborne(&def, vec3(start.x, 900.0, start.y), 330f32.to_radians(), 95.0 * KT);
    let mut ctl = trimmed_controls_at(&def, 95.0 * KT, 900.0);
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    ap.toggle_lat(Lat::Vor);
    for _ in 0..(300.0 / DT) as usize {
        nav.step(&env, &ac, &def, None, None, DT);
        ap.nav = nav.ap_input(&env, &ac, &def, 0.0);
        ap.update(&def, &ac, &mut ctl, DT);
        ac.step(&def, &ctl, &Flat, DT);
    }
    let o = &nav.radios.nav[0].out;
    eprintln!("VOR 360 radial: radial {:.2}°, xtk {:.0} m, to {:?}, mode {:?}", o.radial.to_degrees(), o.xtk, o.to, ap.lat);
    assert_eq!(ap.lat, Lat::Vor);
    assert_eq!(o.to, Some(false));
    assert!(o.cdi.abs() < 0.05 && o.xtk.abs() < 60.0);
}

/// The headline test: an A320 coupled to the EGLL 27R ILS from a 30°
/// intercept at 2,500 ft, gear and flaps landing, A/THR at Vapp: LOC then G/S
/// capture from below, threshold crossed within ±1 dot and 50 ± 15 ft, then
/// autoland — flare, retard, touchdown in the zone, rollout on the centreline.
#[test]
fn coupled_ils_approach_egll_27r() {
    let (env, i) = env_egll();
    let ils = i.ils.clone().unwrap();
    let def = find("meridian320").unwrap();
    let mut nav = NavState::new(&def);
    nav.plan_ils(&i, ApprKind::Ils);
    let dir = geo::dir_of(ils.course);
    let right = geo::right_of(dir);
    let thr = env.to_world(ils.thr);
    let start = thr - dir * 13.0 * NM + right * 3000.0;
    let alt = 2500.0 * FT;
    let vapp = (def.vref_kt + 5.0) * KT;
    let mut ac = Aircraft::airborne(&def, vec3(start.x, alt, start.y), geo::wrap_2pi(ils.course - 30f32.to_radians()), vapp);
    let mut ctl = trimmed_controls_at(&def, vapp, alt);
    let full = *def.flap_detents.last().unwrap();
    ctl.flaps = 1.0;
    ac.set_flaps_now(ctl.flaps);
    ctl.gear_down = true;
    let _ = full;
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    ap.ias = Some(vapp);
    ap.toggle_appr();
    assert_eq!((ap.arm_lat, ap.arm_gs), (Some(Lat::Loc), true));
    let mut thr_cross = None;
    let mut modes = Vec::new();
    let mut touchdown = None;
    let mut max_sink_td = 0.0f32;
    for k in 0..(400.0 / DT) as usize {
        nav.step(&env, &ac, &def, None, None, DT);
        ap.nav = nav.ap_input(&env, &ac, &def, 0.0);
        let was_air = !ac.on_ground;
        let vs = ac.vertical_speed();
        let engaged = ap.update(&def, &ac, &mut ctl, DT);
        ac.step(&def, &ctl, &Flat, DT);
        assert!(ac.crashed.is_none(), "crashed: {:?}", ac.crashed);
        let tag = format!("{:?}/{:?}", ap.lat, ap.vert);
        if modes.last() != Some(&tag) {
            eprintln!("t {:6.1} s: {tag} (to thr {:.2} NM, {:.0} ft)", k as f32 * DT, nav.radios.nav[0].out.to_thr / NM, ac.pos.y / FT);
            modes.push(tag);
        }
        let o = &nav.radios.nav[0].out;
        if thr_cross.is_none() && o.to_thr < 0.0 {
            let dots = o.dev / (ils.half_width / 2.0);
            thr_cross = Some((dots, ac.pos.y / FT, (ac.pos.y - def.gear_height) / FT));
        }
        if was_air && ac.on_ground && touchdown.is_none() {
            max_sink_td = -vs;
            touchdown = Some((-o.to_thr, o.xtk));
        }
        if !engaged {
            break;
        }
    }
    let (dots, h_ft, wheels_ft) = thr_cross.expect("never reached the threshold");
    eprintln!("threshold: LOC {dots:+.2} dot, glidepath reference {h_ft:.1} ft, wheels {wheels_ft:.1} ft");
    assert!(dots.abs() <= 1.0, "LOC {dots} dots");
    assert!((h_ft - 50.0).abs() <= 15.0, "threshold height {h_ft} ft");
    assert!(modes.iter().any(|m| m == "Loc/Gs"), "{modes:?}");
    assert!(modes.iter().any(|m| m == "Loc/Flare"), "{modes:?}");
    let (td_dist, td_x) = touchdown.expect("no touchdown");
    eprintln!(
        "touchdown {td_dist:.0} m past the threshold, {td_x:+.1} m off the centreline, sink {:.2} m/s ({:.0} fpm); stopped at {:.1} kt",
        max_sink_td,
        max_sink_td / 0.00508,
        vec2(ac.vel.x, ac.vel.z).length() / KT
    );
    assert!(td_dist > 150.0 && td_dist < 900.0, "touchdown at {td_dist} m");
    assert!(td_x.abs() < 6.0);
    assert!(max_sink_td < 2.4, "hard: {max_sink_td} m/s");
    assert!(ac.on_ground && vec2(ac.vel.x, ac.vel.z).length() < 30.0 * KT);
    assert_eq!(ap.vert, Vert::Basic, "rollout ends with the AP off");
}

/// A synthetic ILS into rising ground loses its glideslope (localizer only).
#[test]
fn synthetic_glidepath_terrain_check() {
    let a = adb();
    let mut ap = a.get("EGLC").unwrap().clone();
    ap.icao = "XXXX".into();
    ap.ident = "XXXX".into();
    let n = db::ils_at(&ap).into_iter().next().unwrap();
    let ils = n.ils.clone().unwrap();
    let mut env = Env::at(ils.thr);
    assert!(super::state::glidepath_clear(&env, &n));
    // A 600 m ridge across the approach 6 NM out.
    let (_, dir) = super::radio::loc_frame(&env, &ils);
    let c = env.to_world(ils.thr) - dir * 6.0 * NM;
    let side = geo::right_of(dir) * 3000.0;
    env.ridge = Some((c - side, c + side, 600.0));
    assert!(!super::state::glidepath_clear(&env, &n));
}

// ---------------------------------------------------------------- nav.dat

/// Rows from FlightGear's nav.dat (810, AIRAC 2013.10; Robin A. Peel,
/// GPL-2.0-or-later), and a malformed row.
const NAV_DAT: &str = "I
810 Version - data cycle 2013.10, build 20131334, metadata NavXP810.  Copyright (c) 2013, Robin A. Peel (robin@x-plane.com).   GPL.

2  51.50435000  000.06750300      0   322  10    0.0 LCY  CITY NDB
3  51.33089800  000.03481100    590 11510 130   -3.0 BIG  BIGGIN VOR-DME
12  51.33087500  000.03481100    590 11510 125    0.0 BIG  BIGGIN VOR-DME
4  51.47769700 -000.42708100     83 11030  18      89.691 IAA  EGLL 09L ILS-cat-I
4  51.46496100 -000.42837500     83 10950  18      89.729 IBB  EGLL 09R ILS-cat-I
4  51.46476100 -000.49111900     83 10950  18     269.670 ILL  EGLL 27L ILS-cat-I
4  51.47746700 -000.49373900     83 11030  18     269.660 IRR  EGLL 27R ILS-cat-I
6  51.47873900 -000.48065800     51 11030  10  300089.666 IAA  EGLL 09L GS
6  51.46361100 -000.47791100     52 10950  10  300089.690 IBB  EGLL 09R GS
6  51.46363300 -000.43876700     56 10950  10  300269.690 ILL  EGLL 27L GS
6  51.47883100 -000.43820600     60 11030  10  300269.666 IRR  EGLL 27R GS
12  51.47882800 -000.45912800     89 11030  18       1.000 IAA  EGLL 09L DME-ILS
12  51.46376700 -000.45854700     84 10950  18       0.900 IBB  EGLL 09R DME-ILS
12  51.46376700 -000.45854700     84 10950  18       0.900 ILL  EGLL 27L DME-ILS
12  51.47882800 -000.45912800     89 11030  18       1.000 IRR  EGLL 27R DME-ILS
4  51.50564200  000.04212500     19 11115  18     272.892 ILSR EGLC 27  ILS-cat-I
6  51.50551400  000.06335000     35 11115  10  550272.903 ILSR EGLC 27  GS
12  51.50579400  000.05442500     48 11115  18       0.400 ILSR EGLC 27  DME-ILS
5  65.63144444 -018.01119444    200 11190  18      24.000 IEY  BIAR 01  LDA
7  54.69264700 -006.08535800    268     0   0     245.030 ---- EGAA 25  OM
8  47.47223300 -122.31102500    433     0   0     180.343 ---- KSEA 16L MM
4  47.42939200 -122.30805600    338 11030  18     180.343 ISNQ KSEA 16L ILS-cat-I
6  47.46081700 -122.30939400    425 11030  10  300180.343 ISNQ KSEA 16L GS
13  49.20933100  007.39577800   1201 10910  25    0.0 ZND  ZWEIBRUCKEN DME
4  bad row
99
";

fn ils_fixture() -> super::navdat::IlsDb {
    super::navdat::IlsDb::new(super::navdat::ils_records(&super::navdat::parse(NAV_DAT)))
}

#[test]
fn navdat_parse_rows() {
    use super::navdat;
    let recs = navdat::parse(NAV_DAT);
    assert_eq!(recs.len(), 24, "the malformed row and the 99 end are skipped");
    let ndb = &recs[0];
    assert_eq!((ndb.code, ndb.ident.as_str(), ndb.freq_khz(), ndb.name.as_str()), (2, "LCY", 322, "CITY NDB"));
    let gs = recs.iter().find(|r| r.code == 6 && r.ident == "ILSR").unwrap();
    let (a, c) = gs.gs_angle_course();
    assert!((a - 5.5).abs() < 1e-4 && (c - 272.903).abs() < 1e-3, "{a} {c}");
    let dme = recs.iter().find(|r| r.code == 12 && r.ident == "IRR").unwrap();
    assert_eq!((dme.airport.as_str(), dme.runway.as_str(), dme.freq_khz()), ("EGLL", "27R", 110_300));
    let vor_dme = recs.iter().find(|r| r.code == 12 && r.ident == "BIG").unwrap();
    assert!(vor_dme.airport.is_empty() && vor_dme.name == "BIGGIN VOR-DME");
    // VOR / NDB / DME rows as navaids (the VOR-DME's DME row folds into it).
    let n = navdat::navaids(&recs);
    let kinds: Vec<(&str, Kind, u32)> = n.iter().map(|n| (n.ident.as_str(), n.kind, n.freq_khz)).collect();
    assert_eq!(kinds, vec![("LCY", Kind::Ndb, 322), ("BIG", Kind::VorDme, 115_100), ("ZND", Kind::Dme, 109_100)]);
    // The 1100 layout: a region column after the airport.
    let xp11 = "I\n1100 Version - data cycle 1802\n\n4  47.42939200 -122.30805600    338 11030  18     180.343 ISNQ KSEA K1 16L ILS-cat-I\n\
                3  47.43538889 -122.30961111    356 11680 130   19.000 SEA  ENRT K1 SEATTLE VORTAC\n99\n";
    let r = navdat::parse(xp11);
    assert_eq!(r.len(), 2);
    assert_eq!((r[0].airport.as_str(), r[0].runway.as_str(), r[0].name.as_str()), ("KSEA", "16L", "ILS-cat-I"));
    assert_eq!(r[1].name, "SEATTLE VORTAC");
}

#[test]
fn navdat_ils_records_and_compact_roundtrip() {
    use super::navdat::{self, MarkerKind};
    let list = navdat::ils_records(&navdat::parse(NAV_DAT));
    assert_eq!(list.len(), 7);
    let irr = list.iter().find(|r| r.ident == "IRR").unwrap();
    assert_eq!((irr.airport.as_str(), irr.runway.as_str(), irr.freq_khz), ("EGLL", "27R", 110_300));
    assert!((irr.course_deg - 269.66).abs() < 1e-3 && irr.gs.is_some_and(|g| (g.1 - 3.0).abs() < 1e-4));
    assert_eq!(irr.dme, Some((51.478828, -0.459128)));
    // A code-5 row has no glideslope; markers join their runway end.
    let lda = list.iter().find(|r| r.ident == "IEY").unwrap();
    assert!(lda.gs.is_none() && lda.facility == "LDA");
    let sea = list.iter().find(|r| r.ident == "ISNQ").unwrap();
    assert_eq!(sea.markers.len(), 1);
    assert_eq!(sea.markers[0].kind, MarkerKind::Middle);
    let back = navdat::decode(&navdat::encode(&list)).unwrap();
    assert_eq!(back.len(), list.len());
    for (a, b) in back.iter().zip(&list) {
        assert_eq!((&a.airport, &a.runway, &a.ident, a.freq_khz, &a.facility, a.markers.len()), (&b.airport, &b.runway, &b.ident, b.freq_khz, &b.facility, b.markers.len()));
        assert!((a.loc.0 - b.loc.0).abs() < 1e-6 && (a.course_deg - b.course_deg).abs() < 1e-3);
        assert_eq!(a.gs.is_some(), b.gs.is_some());
    }
    assert!(navdat::decode("skyward-ils-0\n").is_none());
    assert!(navdat::rwy_eq("09L", "9L") && !navdat::rwy_eq("09L", "09R"));
}

/// nav.dat's EGLL ILS carry the published facts the hand table had, on the
/// real antenna sites; the threshold stays the OurAirports one.
#[test]
fn real_ils_matches_published_egll() {
    let a = adb();
    let real = ils_fixture();
    let egll = a.get("EGLL").unwrap();
    let list = db::ils_at_with(egll, Some(&real));
    assert_eq!(list.len(), 4);
    let synth = db::ils_at_with(egll, None);
    for (rwy, ident, freq) in [("27R", "IRR", 110_300), ("09L", "IAA", 110_300), ("27L", "ILL", 109_500), ("09R", "IBB", 109_500)] {
        let n = list.iter().find(|n| n.ils.as_ref().unwrap().runway == rwy).unwrap();
        let s = synth.iter().find(|n| n.ils.as_ref().unwrap().runway == rwy).unwrap();
        let (i, si) = (n.ils.as_ref().unwrap(), s.ils.as_ref().unwrap());
        assert_eq!((n.ident.as_str(), n.freq_khz, n.kind, i.synthetic), (ident, freq, Kind::Ils, false), "{rwy}");
        assert_eq!((n.ident.as_str(), n.freq_khz), (s.ident.as_str(), s.freq_khz), "{rwy}: as the hand table");
        assert!((i.gs_angle.to_degrees() - 3.0).abs() < 1e-3);
        // Same threshold; the course within 0.1° of the runway's; the
        // localizer within 200 m of where the synthetic one put it.
        assert!(geo::gc_dist(i.thr, si.thr) < 0.5);
        assert!(geo::wrap_pi(i.course - si.course).abs() < 0.1f32.to_radians(), "{rwy}: {} vs {}", i.course.to_degrees(), si.course.to_degrees());
        assert!(geo::gc_dist(i.loc, si.loc) < 200.0, "{rwy}: {}", geo::gc_dist(i.loc, si.loc));
        assert!(n.dme.is_some() && i.gs.is_some());
    }
    // The radios fly the real 27R as the synthetic one: beam along the runway.
    let n = list.iter().find(|n| n.ident == "IRR").unwrap();
    let ils = n.ils.as_ref().unwrap();
    let env = Env::at(ils.thr);
    let (_, dir) = super::radio::loc_frame(&env, ils);
    assert!(geo::wrap_pi(geo::heading_of(dir) - ils.course).abs() < 0.1f32.to_radians());
}

/// The chart table beats nav.dat's stale facts; at an airport nav.dat covers,
/// runway ends it lists none for get none (unless charted); an ILS whose runway
/// OurAirports does not know still works (threshold from the localizer).
#[test]
fn real_ils_overrides_and_fallback() {
    let a = adb();
    let real = ils_fixture();
    let lc = db::ils_at_with(a.get("EGLC").unwrap(), Some(&real));
    assert_eq!(lc.len(), 2);
    let r27 = lc.iter().find(|n| n.ils.as_ref().unwrap().runway == "27").unwrap();
    assert!(!r27.ils.as_ref().unwrap().synthetic && r27.ident == "LST" && r27.freq_khz == 111_150);
    // The fixture has no EGLC 09: the chart table makes it on the runway geometry.
    let r09 = lc.iter().find(|n| n.ils.as_ref().unwrap().runway == "09").unwrap();
    assert!(r09.ident == "LSR" && (r09.ils.as_ref().unwrap().gs_angle.to_degrees() - 5.5).abs() < 1e-3);
    // An airport whose one runway has a real ILS at one end only.
    let mut ap = a.get("EGLC").unwrap().clone();
    ap.icao = "KSEA".into();
    ap.ident = "KSEA".into();
    ap.runways[0].ends[0].ident = "34R".into();
    ap.runways[0].ends[1].ident = "16L".into();
    let s = db::ils_at_with(&ap, Some(&real));
    // nav.dat covers the airport: the end it lists no ILS for gets none.
    assert_eq!(s.len(), 1);
    let r = &s[0];
    assert_eq!((r.ident.as_str(), r.ils.as_ref().unwrap().synthetic, r.ils.as_ref().unwrap().markers.len()), ("ISNQ", false, 1));
    // An airport nav.dat does not cover: synthetic on both ends.
    let mut un = ap.clone();
    un.icao = "KXYZ".into();
    un.ident = "KXYZ".into();
    let u = db::ils_at_with(&un, Some(&real));
    assert!(u.len() == 2 && u.iter().all(|n| n.ils.as_ref().unwrap().synthetic));
    // No OurAirports runway at BIAR: the LDA still works (localizer only).
    let mut biar = ap.clone();
    biar.icao = "BIAR".into();
    biar.ident = "BIAR".into();
    biar.runways.clear();
    let l = db::ils_at_with(&biar, Some(&real));
    assert_eq!(l.len(), 1);
    assert_eq!((l[0].kind, l[0].ident.as_str()), (Kind::Loc, "IEY"));
    let i = l[0].ils.as_ref().unwrap();
    assert!((geo::gc_dist(i.thr, i.loc) - 2800.0).abs() < 1.0 && i.gs.is_none());
}

/// The whole nav.dat (not in the repo): `SKYWARD_NAVDAT=/path/nav.dat.gz
/// cargo test navdat_full_file -- --ignored --nocapture`.
#[test]
#[ignore]
fn navdat_full_file() {
    use super::navdat;
    let raw = std::fs::read(std::env::var("SKYWARD_NAVDAT").expect("SKYWARD_NAVDAT")).unwrap();
    let recs = navdat::parse(&navdat::text_of(&raw));
    let list = navdat::ils_records(&recs);
    let (gz, n) = navdat::compact_from_navdat(&raw).unwrap();
    let gs = list.iter().filter(|r| r.gs.is_some()).count();
    let dme = list.iter().filter(|r| r.dme.is_some()).count();
    let mk: usize = list.iter().map(|r| r.markers.len()).sum();
    eprintln!(
        "{} rows, {} ILS / LOC ({gs} with GS, {dme} with DME, {mk} markers, {n} compact), {} navaids, compact {:.0} kB",
        recs.len(),
        list.len(),
        navdat::navaids(&recs).len(),
        gz.len() as f64 / 1e3
    );
    assert!(n > 3000);
    let real = navdat::IlsDb::new(list);
    let egll = db::ils_at_with(adb().get("EGLL").unwrap(), Some(&real));
    assert!(egll.iter().any(|n| n.ident == "IRR" && n.freq_khz == 110_300 && !n.ils.as_ref().unwrap().synthetic));
    // With SKYWARD_OURAIRPORTS=cache/ourairports.tsv.gz: how the real ILS sit
    // on the OurAirports runways worldwide.
    let Ok(p) = std::env::var("SKYWARD_OURAIRPORTS") else { return };
    let aps = crate::airports::decode(&navdat::text_of(&std::fs::read(p).unwrap())).unwrap();
    let (mut real_n, mut synth_n, mut on_rwy, mut offset) = (0, 0, 0, 0);
    let mut errs = Vec::new();
    for ap in &aps {
        for n in db::ils_at_with(ap, Some(&real)) {
            let i = n.ils.as_ref().unwrap();
            if i.synthetic {
                synth_n += 1;
                continue;
            }
            real_n += 1;
            if ap.runways.iter().any(|r| r.surveyed() && r.ends.iter().any(|e| e.ident == i.runway)) {
                on_rwy += 1;
                let d = geo::wrap_pi(geo::gc_bearing(i.thr, i.loc) - i.course).abs().to_degrees();
                if d > 2.0 {
                    offset += 1;
                }
                errs.push(d);
            }
        }
    }
    errs.sort_by(|a, b| a.total_cmp(b));
    let q = |f: f64| errs.get((errs.len() as f64 * f) as usize).copied().unwrap_or(0.0);
    eprintln!(
        "{real_n} real ILS at OurAirports airports ({on_rwy} on a surveyed runway end; threshold → localizer vs course: median {:.2}°, 95 % {:.2}°, {offset} offset > 2°), {synth_n} synthetic",
        q(0.5),
        q(0.95)
    );
}
