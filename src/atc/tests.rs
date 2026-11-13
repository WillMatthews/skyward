//! ATC tests: phraseology (numbers, runways, frequencies, levels), the
//! vocabulary covering every template and every voice, ATIS from sample
//! METARs, runway selection by wind, transition levels, frequency parsing,
//! taxi routes over the real EGLL / EGLC taxiway networks, radar vectors
//! that lead an A320 on the autopilot to a localiser (and glideslope)
//! capture, and the radio chain's output.

use super::phrase::{self, Utt};
use super::taxi::{self, Graph, RwyBox};
use super::{atis, freqs, vectors};
use crate::audio::radio::{self, Voice};
use crate::weather::Wx;
use glam::{Vec2, vec2};

fn spoken(u: &Utt) -> String {
    let v = phrase::vocab();
    u.toks
        .iter()
        .filter_map(|t| match t {
            phrase::Tok::Word(w) => Some(v.keys[*w as usize]),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn numbers_runways_frequencies() {
    assert_eq!(phrase::digits(1018), "one zero one eight");
    assert_eq!(phrase::digits(1019), "one zero one niner");
    assert_eq!(phrase::runway_words("27R"), "two seven right");
    assert_eq!(phrase::runway_words("09L"), "zero niner left");
    assert_eq!(phrase::runway_words("36C"), "three six centre");
    assert_eq!(phrase::freq_words(118_500), "one one eight decimal five");
    assert_eq!(phrase::freq_text(118_500), "118.5");
    assert_eq!(phrase::freq_words(121_975), "one two one decimal niner seven five");
    assert_eq!(phrase::freq_words(132_700), "one three two decimal seven");
    assert_eq!(phrase::freq_text(118_075), "118.075");
    assert_eq!(phrase::alt_words(3500), "three thousand five hundred");
    assert_eq!(phrase::alt_words(11_000), "one one thousand");
    assert_eq!(phrase::alt_words(2000), "two thousand");
    assert_eq!(phrase::fl_words(80), "eight zero");
    assert_eq!(phrase::fl_words(100), "one hundred");
    assert_eq!(phrase::fl_words(245), "two four five");
    assert_eq!(phrase::spell("A10"), "alpha one zero");
    let mut u = Utt::new();
    u.runway("27R").comma().say("cleared for take-off");
    let u = u.finish();
    assert_eq!(u.text, "Runway 27R, cleared for take-off.");
    assert_eq!(spoken(&u), "runway two seven right cleared for take-off");
    assert!(u.missing.is_empty(), "{:?}", u.missing);
    let mut u = Utt::new();
    u.say("contact").say_as("heathrow tower", "Heathrow Tower").freq(118_500);
    assert_eq!(spoken(&u), "contact heathrow tower one one eight decimal five");
    assert_eq!(u.text, "Contact Heathrow Tower 118.5");
    let mut u = Utt::new();
    u.level(6000.0, 70).comma().level(8000.0, 70).comma().qnh(1018.0).comma().squawk(4621);
    assert_eq!(u.text, "Altitude 6,000 feet, flight level 80, QNH 1018, squawk 4621");
    assert_eq!(spoken(&u), "altitude six thousand feet flight level eight zero qnh one zero one eight squawk four six two one");
    let mut u = Utt::new();
    u.wind(Some(243.0), 12.4, Some(25.0));
    assert_eq!(u.text, "240 degrees 12 knots gusting 25 knots");
    assert_eq!(spoken(&u), "two four zero degrees one two knots gusting two five knots");
    assert_eq!(phrase::heading_value(2.0), 360);
    let c = phrase::Callsign::registration("G-SKYW");
    assert_eq!((c.full_spoken.as_str(), c.short_text.as_str(), c.short_spoken.as_str()), ("golf sierra kilo yankee whiskey", "G-YW", "golf yankee whiskey"));
}

/// Every phrase literal in the templates (atc/*.rs) is in the vocabulary.
#[test]
fn vocabulary_covers_the_templates() {
    let v = phrase::vocab();
    let mut missing = Vec::new();
    for src in [include_str!("mod.rs"), include_str!("atis.rs")] {
        for pat in ["say(\"", "say_as(\"", "say_as(&format!(\""] {
            let mut rest = src;
            while let Some(i) = rest.find(pat) {
                rest = &rest[i + pat.len()..];
                let end = rest.find('"').unwrap_or(0);
                let lit = &rest[..end];
                // Literal pieces between format placeholders.
                for piece in lit.split(|c| c == '{' || c == '}').enumerate().filter(|(k, _)| k % 2 == 0).map(|x| x.1) {
                    let mut u = Utt::new();
                    u.say(piece.trim().trim_matches(','));
                    for m in u.missing {
                        missing.push(format!("{m} (in \"{lit}\")"));
                    }
                }
            }
        }
    }
    for (_, s, _) in phrase::NAMES {
        if v.get(s).is_none() {
            missing.push(s.to_string());
        }
    }
    for w in phrase::DIGITS.iter().chain(phrase::LETTERS.iter()) {
        if v.get(w).is_none() {
            missing.push(w.to_string());
        }
    }
    assert!(missing.is_empty(), "not in vocab.tsv: {missing:?}");
}

/// The voice packs contain what each voice needs (the assets match vocab.tsv).
#[test]
fn voice_packs_match_the_vocabulary() {
    let v = phrase::vocab();
    let mut gaps = Vec::new();
    for (i, k) in v.keys.iter().enumerate() {
        let tags = v.tags[i];
        let need: Vec<Voice> =
            [(Voice::Pilot, 'P'), (Voice::Emma, 'C'), (Voice::Onyx, 'C'), (Voice::Fable, 'C'), (Voice::River, 'C'), (Voice::Aoede, 'C'), (Voice::Emma, 'A'), (Voice::Aoede, 'A')]
                .into_iter()
                .filter(|(_, t)| tags.contains(*t))
                .map(|x| x.0)
                .collect();
        for vo in need {
            if !radio::has(vo, i as u16) {
                gaps.push(format!("{k} / {vo:?}"));
            }
        }
    }
    assert!(gaps.is_empty(), "re-run assets/audio/tools/gen_atc.py: {gaps:?}");
    radio::prepare_blocking();
    for vo in [Voice::Pilot, Voice::Emma, Voice::Fable] {
        let p = radio::pack(vo).unwrap();
        assert!(p.pcm.len() > 12_000 * 60, "{vo:?} decoded {} samples", p.pcm.len());
        let (a, n) = p.clips[&v.get("runway").unwrap()];
        let clip = &p.pcm[a as usize..(a + n) as usize];
        let rms = (clip.iter().map(|x| x * x).sum::<f32>() / clip.len() as f32).sqrt();
        assert!(rms > 0.03 && rms < 0.4, "{vo:?} 'runway' rms {rms}");
    }
}

const AIRPORTS: &str = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\",\"continent\",\"iso_country\",\"iso_region\",\"municipality\",\"scheduled_service\",\"icao_code\",\"iata_code\",\"gps_code\",\"local_code\",\"home_link\",\"wikipedia_link\",\"keywords\"
2430,\"EGLC\",\"medium_airport\",\"London City Airport\",51.505299,0.055278,19,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLC\",\"LCY\",\"EGLC\",,,,
2434,\"EGLL\",\"large_airport\",\"London Heathrow Airport\",51.470748,-0.459909,83,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGLL\",\"LHR\",\"EGLL\",,,,
2429,\"EGKK\",\"large_airport\",\"London Gatwick Airport\",51.148102,-0.190278,202,\"EU\",\"GB\",\"GB-ENG\",\"London\",\"yes\",\"EGKK\",\"LGW\",\"EGKK\",,,,
";
const RUNWAYS: &str = "\"id\",\"airport_ref\",\"airport_ident\",\"length_ft\",\"width_ft\",\"surface\",\"lighted\",\"closed\",\"le_ident\",\"le_latitude_deg\",\"le_longitude_deg\",\"le_elevation_ft\",\"le_heading_degT\",\"le_displaced_threshold_ft\",\"he_ident\",\"he_latitude_deg\",\"he_longitude_deg\",\"he_elevation_ft\",\"he_heading_degT\",\"he_displaced_threshold_ft\"
239448,2430,\"EGLC\",4948,98,\"CON\",1,0,\"09\",51.505576,0.044333,16,93,,\"27\",51.504894,0.066026,19,273,
239399,2434,\"EGLL\",12799,164,\"ASP\",1,0,\"09L\",51.47749,-0.489439,79,90,1007,\"27R\",51.477681,-0.433227,78,270,
239398,2434,\"EGLL\",12001,164,\"ASP\",1,0,\"09R\",51.46478,-0.486808,75,90,1013,\"27L\",51.464957,-0.434048,77,270,
239397,2429,\"EGKK\",10879,148,\"ASP\",1,0,\"08R\",51.14365,-0.21086,196,78,,\"26L\",51.15475,-0.16588,202,258,
239396,2429,\"EGKK\",8415,148,\"ASP\",1,0,\"08L\",51.14552,-0.20878,196,78,,\"26R\",51.15394,-0.17414,202,258,
";

fn adb() -> crate::airports::Db {
    crate::airports::Db::new(crate::airports::parse_csv(AIRPORTS, RUNWAYS), 0)
}

#[test]
fn runway_in_use_by_wind() {
    let db = adb();
    let ll = db.get("EGLL").unwrap();
    let w = |d: f32, kt: f32| atis::runway_in_use(ll, d, kt).unwrap();
    let r = w(240.0, 12.0);
    assert_eq!((r.arr.as_str(), r.dep.as_str()), ("27R", "27L"));
    let r = w(80.0, 15.0);
    assert_eq!((r.arr.as_str(), r.dep.as_str()), ("09L", "09R"));
    // Calm and light tailwind: the westerly preference.
    assert_eq!(w(90.0, 3.0).arr, "27R");
    assert_eq!(w(0.0, 0.0).arr, "27R");
    // A northerly crosswind: the end with the (small) headwind.
    assert_eq!(w(350.0, 20.0).arr, "27R");
    assert_eq!(w(10.0, 20.0).arr, "09L");
    let lc = db.get("EGLC").unwrap();
    assert_eq!(atis::runway_in_use(lc, 100.0, 10.0).unwrap().arr, "09");
    assert_eq!(atis::runway_in_use(lc, 260.0, 10.0).unwrap().dep, "27");
    // Gatwick: 26L is the main runway (26R the standby): both "26" ends, landing on the right one heading west.
    let kk = db.get("EGKK").unwrap();
    assert!(atis::runway_in_use(kk, 250.0, 10.0).unwrap().arr.starts_with("26"));
}

#[test]
fn transition_levels() {
    let db = adb();
    assert_eq!(atis::transition_altitude(db.get("EGLL").unwrap()), 6000);
    assert_eq!(atis::transition_level(6000, 1013.0), 70);
    assert_eq!(atis::transition_level(6000, 1030.0), 70);
    assert_eq!(atis::transition_level(6000, 995.0), 80);
    assert_eq!(atis::transition_level(3000, 1013.0), 40);
}

fn atis_for(icao: &str, metar: &str, letter: char) -> Utt {
    let db = adb();
    let ap = db.get(icao).unwrap();
    let wx = Wx::from_metar_str(metar).unwrap();
    let rwy = atis::runway_in_use(ap, wx.wind_from, wx.wind_kt).unwrap();
    atis::broadcast(ap, &wx, letter, (12, 50), &rwy)
}

#[test]
fn atis_from_sample_metars() {
    let u = atis_for("EGLL", "EGLL 131250Z 24012KT 9999 FEW020 BKN035 15/08 Q1013", 'B');
    eprintln!("{}", u.text);
    assert!(u.missing.is_empty(), "{:?}", u.missing);
    for want in [
        "Heathrow information Bravo, time 1250.",
        "Landing runway 27R, departing runway 27L.",
        "Surface wind 240 degrees 12 knots.",
        "Visibility 10 kilometres or more.",
        "Cloud few 2,000 feet, broken 3,500 feet.",
        "Temperature 15, dew point 8.",
        "QNH 1013 hectopascals.",
        "Transition level flight level 70.",
        "Expect ILS approach.",
        "Advise on initial contact you have information Bravo.",
    ] {
        assert!(u.text.contains(want), "missing “{want}” in: {}", u.text);
    }
    let s = spoken(&u);
    assert!(s.starts_with("heathrow information bravo time one two five zero"), "{s}");
    assert!(s.contains("surface wind two four zero degrees one two knots"), "{s}");
    assert!(s.contains("qnh one zero one three hectopascals"), "{s}");
    // Rain, mist, gusts, low cloud, low pressure (transition level 80), easterly.
    let u = atis_for("EGLL", "EGLL 131250Z 08018G30KT 4000 -RA BR SCT006 OVC012 09/08 Q0996", 'K');
    eprintln!("{}", u.text);
    assert!(u.missing.is_empty(), "{:?}", u.missing);
    for want in [
        "Kilo",
        "Landing runway 09L, departing runway 09R.",
        "080 degrees 18 knots gusting 30 knots",
        "Visibility 4,000 metres.",
        "Light rain, mist.",
        "Cloud scattered 600 feet, overcast 1,200 feet.",
        "QNH 996",
        "flight level 80",
        "Runway surface wet.",
    ] {
        assert!(u.text.contains(want), "missing “{want}” in: {}", u.text);
    }
    // CAVOK, variable wind, sub-zero temperatures, London City.
    let u = atis_for("EGLC", "EGLC 131250Z VRB03KT CAVOK M02/M05 Q1031", 'D');
    eprintln!("{}", u.text);
    assert!(u.missing.is_empty(), "{:?}", u.missing);
    for want in ["London City information Delta", "Runway in use 27.", "Surface wind variable 3 knots.", "CAVOK.", "Temperature -2, dew point -5.", "QNH 1031"] {
        assert!(u.text.contains(want), "missing “{want}” in: {}", u.text);
    }
    assert!(spoken(&u).contains("temperature minus two dew point minus five"));
    // Thunderstorms in the vicinity, CB.
    let u = atis_for("EGLC", "EGLC 131250Z 27015KT 9999 VCTS FEW030CB 22/16 Q1008", 'E');
    assert!(u.missing.is_empty(), "{:?}", u.missing);
    assert!(u.text.contains("Thunderstorm in the vicinity.") && u.text.contains("few 3,000 feet cumulonimbus"), "{}", u.text);
}

#[test]
fn frequencies_csv() {
    let csv = "\"id\",\"airport_ref\",\"airport_ident\",\"type\",\"description\",\"frequency_mhz\"
323434,2434,\"EGLL\",\"ATIS\",\"Heathrow ATIS\",113.75
319777,2434,\"EGLL\",\"ATIS\",\"Heathrow ATIS\",128.075
58109,2434,\"EGLL\",\"CLD\",\"Heathrow Delivery\",121.975
58107,2434,\"EGLL\",\"DIR\",\"Heathrow Director\",119.725
58110,2434,\"EGLL\",\"GND\",\"Heathrow Ground\",121.7
58111,2434,\"EGLL\",\"TWR\",\"Heathrow Tower\",118.5
58212,2430,\"EGLC\",\"APP\",\"THAMES RDR\",132.7
58215,2430,\"EGLC\",\"TWR\",\"TWR\",118.075
";
    let list = freqs::parse_csv(csv);
    // The VOR-borne ATIS (113.75) is not a COM frequency.
    assert_eq!(list.len(), 7);
    let rt = freqs::decode(&freqs::encode(&list)).unwrap();
    assert_eq!(rt, list);
    let db = freqs::FreqDb::new(list);
    let a = adb();
    let st = super::stations_of(a.get("EGLL").unwrap(), db.at("EGLL"));
    let t: Vec<(String, u32)> = st.iter().map(|s| (s.text.clone(), s.khz)).collect();
    assert!(t.contains(&("Heathrow Information".into(), 128_075)), "{t:?}");
    assert!(t.contains(&("Heathrow Director".into(), 119_725)));
    assert!(t.contains(&("Heathrow Delivery".into(), 121_975)));
    let tower = st.iter().find(|s| s.text == "Heathrow Tower").unwrap();
    let ground = st.iter().find(|s| s.text == "Heathrow Ground").unwrap();
    assert_ne!(tower.voice, ground.voice);
    let lc = super::stations_of(a.get("EGLC").unwrap(), db.at("EGLC"));
    assert!(lc.iter().any(|s| s.text == "Thames Radar" && s.khz == 132_700), "{:?}", lc.iter().map(|s| &s.text).collect::<Vec<_>>());
    assert!(lc.iter().any(|s| s.text == "London City Tower" && s.spoken == "london city tower"));
}

// ---------------------------------------------------------------- taxi

struct Fixture {
    graph: Graph,
    runways: Vec<(String, String, Vec2, Vec2, f32)>,
    holds: Vec<(Vec2, String, String)>,
}

fn fixture(text: &str) -> Fixture {
    let p = |s: &str| {
        let (a, b) = s.split_once(',').unwrap();
        vec2(a.parse().unwrap(), b.parse().unwrap())
    };
    let mut tw = Vec::new();
    let mut runways = Vec::new();
    let mut holds = Vec::new();
    for l in text.lines() {
        let f: Vec<&str> = l.split('\t').collect();
        match f[0] {
            "R" => runways.push((f[1].to_string(), f[2].to_string(), p(f[3]), p(f[4]), f[5].parse().unwrap())),
            "T" => tw.push((f[2].split(';').map(p).collect::<Vec<_>>(), f[1].to_string())),
            "H" => holds.push((p(f[3]), f[1].to_string(), f[2].to_string())),
            _ => {}
        }
    }
    Fixture { graph: Graph::build(&tw), runways, holds }
}

fn boxes(f: &Fixture) -> Vec<RwyBox> {
    f.runways.iter().map(|r| RwyBox { a: r.2, b: r.3, width: r.4, name: format!("{}/{}", r.0, r.1) }).collect()
}

/// Route to the departure hold of `rwy` from `from`; returns the route and hold.
fn taxi_to(f: &Fixture, rwy: &str, from: Vec2) -> (taxi::Route, taxi::HoldPt) {
    let r = f.runways.iter().find(|r| r.0 == rwy || r.1 == rwy).unwrap();
    let (thr, far) = if r.0 == rwy { (r.2, r.3) } else { (r.3, r.2) };
    let dir = (far - thr).normalize();
    let name = format!("{}/{}", r.0, r.1);
    let h = taxi::departure_hold(&f.holds, &name, thr, dir).expect("no holding point");
    let route = f.graph.route(from, h.at, 400.0, &boxes(f)).expect("no taxi route");
    (route, h)
}

#[test]
fn taxi_routes_egll_and_eglc() {
    let f = fixture(include_str!("fixtures/egll.taxi"));
    eprintln!("EGLL taxi graph: {} nodes", f.graph.len());
    assert!(f.graph.len() > 1500);
    // From the Terminal 5 apron (west, between the runways) to 27L and 27R; from the central area to 09R.
    for (rwy, from) in [("27L", vec2(-1500.0, 0.0)), ("27R", vec2(-1500.0, 0.0)), ("09R", vec2(0.0, 0.0)), ("09L", vec2(500.0, 100.0))] {
        let (r, h) = taxi_to(&f, rwy, from);
        eprintln!("EGLL {rwy}: hold {} via {:?}, crosses {:?}, {:.0} m", h.taxiway, r.via, r.crosses, r.length);
        assert!(r.length > 200.0 && r.length < 9000.0, "{rwy}: {:.0} m", r.length);
        assert!(!r.via.is_empty(), "{rwy}: no taxiway names");
        assert!(r.pts.last().unwrap().distance(h.at) < 60.0);
        // The route stays on the taxiways: no long jumps.
        assert!(r.pts.windows(2).all(|w| w[0].distance(w[1]) < 40.0));
    }
    let f = fixture(include_str!("fixtures/eglc.taxi"));
    eprintln!("EGLC taxi graph: {} nodes", f.graph.len());
    // From the apron (south of the runway, centre) to the 27 and 09 holds.
    for rwy in ["27", "09"] {
        let (r, h) = taxi_to(&f, rwy, vec2(0.0, 150.0));
        eprintln!("EGLC {rwy}: hold {:?} via {:?}, {:.0} m", h.taxiway, r.via, r.length);
        assert!(r.length > 100.0 && r.length < 3000.0);
        assert!(r.crosses.is_empty(), "EGLC taxiing must not cross the runway: {:?}", r.crosses);
    }
}

// ---------------------------------------------------------------- vectors

struct Env(crate::osm::Proj);
impl crate::nav::radio::RadioEnv for Env {
    fn to_world(&self, ll: crate::nav::geo::LL) -> Vec2 {
        self.0.to_world(ll.0, ll.1)
    }
    fn to_ll(&self, p: Vec2) -> crate::nav::geo::LL {
        self.0.to_latlon(p)
    }
    fn ground(&self, _: Vec2) -> Option<f32> {
        Some(0.0)
    }
    fn runway_elev(&self, _: &str, _: &str) -> Option<f32> {
        Some(0.0)
    }
}

struct Flat;
impl crate::flight::Environment for Flat {
    fn ground_height(&self, _: Vec2) -> f32 {
        0.0
    }
    fn obstacle_top(&self, _: Vec2) -> Option<f32> {
        None
    }
    fn is_water(&self, _: Vec2) -> bool {
        false
    }
    fn updraft(&self, _: glam::Vec3) -> f32 {
        0.0
    }
}

/// Fly an A320 on the autopilot under radar vectors from `start` (NM east /
/// north of the 27R threshold) and return (LOC captured, G/S captured, capture
/// distances NM, below the glidepath at G/S capture, headings issued).
fn vectored(east_nm: f32, north_nm: f32, hdg: f32, alt_ft: f32) -> (bool, bool, f32, f32, Vec<f32>) {
    use crate::autopilot::{Autopilot, Lat, Vert};
    use crate::flight::{Aircraft, KT, find, trimmed_controls_at};
    use crate::nav::{NavState, fms::ApprKind};
    const DT: f32 = 1.0 / 120.0;
    let db = adb();
    let i = crate::nav::db::ils_at(db.get("EGLL").unwrap()).into_iter().find(|n| n.ils.as_ref().unwrap().runway == "27R").unwrap();
    let ils = i.ils.clone().unwrap();
    let env = Env(crate::osm::Proj::new(ils.thr.0, ils.thr.1));
    let def = find("meridian320").unwrap();
    let mut nav = NavState::new(&def);
    nav.plan_ils(&i, ApprKind::Ils);
    let thr = env.0.to_world(ils.thr.0, ils.thr.1);
    let loc = env.0.to_world(ils.loc.0, ils.loc.1);
    let fin = vectors::Final { thr, dir: (loc - thr).normalize(), elev_ft: 0.0, gs_deg: ils.gs_angle.to_degrees() };
    let start = thr + vec2(east_nm, -north_nm) * vectors::NM;
    let ias = 210.0 * KT;
    let alt = alt_ft * 0.3048;
    let mut ac = Aircraft::airborne(&def, glam::vec3(start.x, alt, start.y), hdg.to_radians(), ias);
    let mut ctl = trimmed_controls_at(&def, ias, alt);
    let mut ap = Autopilot::new();
    ap.engage_hold_current(&ac);
    ap.ias = Some(ias);
    let mut dirc = vectors::Director::new(fin.clone(), start);
    ap.alt = Some(dirc.plan.alt_ft * 0.3048);
    let mut issued = Vec::new();
    let (mut loc_at, mut gs_at) = (f32::NAN, f32::NAN);
    let mut t = 0.0f32;
    let mut slowed = false;
    while t < 1500.0 {
        // The controller looks every 4 s (and the pilot reacts at once).
        if (t / 4.0).fract() < DT / 4.0 * 1.01 {
            let p = vec2(ac.pos.x, ac.pos.z);
            match dirc.update(p, ac.heading().to_degrees().rem_euclid(360.0), ac.tas(), Vec2::ZERO) {
                Some(vectors::Call::Heading { hdg, .. }) => {
                    issued.push(hdg);
                    ap.hdg = Some(hdg.to_radians());
                }
                Some(vectors::Call::Intercept { hdg, .. }) => {
                    issued.push(hdg);
                    ap.hdg = Some(hdg.to_radians());
                    ap.toggle_appr();
                }
                None => {}
            }
        }
        // Approach flaps and 180 kt from 16 NM.
        if !slowed && fin.along(vec2(ac.pos.x, ac.pos.z)) < 16.0 * vectors::NM {
            ap.ias = Some(180.0 * KT);
            ctl.flaps = 0.5;
            slowed = true;
        }
        nav.step(&env, &ac, &def, None, None, DT);
        ap.nav = nav.ap_input(&env, &ac, &def, 0.0);
        let engaged = ap.update(&def, &ac, &mut ctl, DT);
        ac.step(&def, &ctl, &Flat, DT);
        let o = &nav.radios.nav[0].out;
        if std::env::var_os("ATC_TRACE").is_some() && (t / 10.0).fract() < DT / 10.0 * 1.01 {
            eprintln!(
                "t {t:5.0} pos {:7.0},{:7.0} alt {:5.0} ft hdg {:5.1} ias {:4.0} engaged {engaged} {:?}/{:?} arm {:?} loc {} valid {} xtk {:.0} to_thr {:.1}",
                ac.pos.x,
                ac.pos.z,
                ac.pos.y / 0.3048,
                ac.heading().to_degrees().rem_euclid(360.0),
                ac.airspeed() / KT,
                ap.lat,
                ap.vert,
                ap.arm_lat,
                o.loc,
                o.valid,
                o.xtk,
                o.to_thr / vectors::NM
            );
        }
        if ap.lat == Lat::Loc && loc_at.is_nan() {
            loc_at = o.to_thr / vectors::NM;
        }
        if ap.vert == Vert::Gs && gs_at.is_nan() {
            gs_at = o.to_thr / vectors::NM;
            // Captured from below, with the localiser already held.
            assert_eq!(ap.lat, Lat::Loc);
            break;
        }
        assert!(ac.crashed.is_none(), "crashed {:?} at {:?}, t {t:.0}, issued {issued:?}, ap {:?}/{:?}", ac.crashed, ac.pos, ap.lat, ap.vert);
        t += DT;
    }
    eprintln!(
        "vectors from {east_nm} E {north_nm} N hdg {hdg}: wps {}, headings {issued:?}, LOC at {loc_at:.1} NM, G/S at {gs_at:.1} NM (int alt {} ft)",
        dirc.plan.wps.len(),
        dirc.plan.alt_ft
    );
    (!loc_at.is_nan(), !gs_at.is_nan(), loc_at, gs_at, issued)
}

#[test]
fn vectors_lead_to_localiser_capture() {
    // Straight in from the east, from the north abeam, from the south-west beyond the field.
    for (e, n, h, alt) in [(28.0, 3.0, 270.0, 6000.0), (8.0, 14.0, 180.0, 5000.0), (-12.0, -9.0, 60.0, 5000.0)] {
        let (loc, gs, loc_nm, gs_nm, issued) = vectored(e, n, h, alt);
        assert!(loc, "no LOC capture from {e},{n}");
        assert!(gs, "no G/S capture from {e},{n}");
        assert!(loc_nm > 8.0 && loc_nm < 15.0, "LOC captured at {loc_nm} NM");
        assert!(loc_nm > gs_nm, "G/S before LOC");
        assert!(gs_nm > 6.0, "G/S captured late ({gs_nm} NM)");
        assert!(!issued.is_empty() && issued.len() < 8, "{issued:?}");
    }
}

#[test]
fn intercept_geometry() {
    let fin = vectors::Final { thr: Vec2::ZERO, dir: vec2(-1.0, 0.0), elev_ft: 83.0, gs_deg: 3.0 };
    // 27R-like: landing west. Intercept heading 30° off the course, from the side we're on.
    let p = vectors::plan(&fin, vec2(20.0 * vectors::NM, -5.0 * vectors::NM));
    assert!((vectors::hdg_diff(p.intercept_hdg, 270.0).abs() - 30.0).abs() < 0.5, "{}", p.intercept_hdg);
    assert_eq!(p.alt_ft, 3000.0);
    // Below the glidepath at the intercept point.
    let gp = 83.0 + 50.0 + vectors::D_INTERCEPT * 3f32.to_radians().tan() / 0.3048;
    assert!(p.alt_ft < gp - 100.0);
    // Beyond the field: a downwind first.
    let p = vectors::plan(&fin, vec2(-10.0 * vectors::NM, 2.0 * vectors::NM));
    assert!(p.wps.len() >= 2);
}

// ---------------------------------------------------------------- radio

#[test]
fn radio_chain_renders_a_transmission() {
    radio::prepare_blocking();
    let mut u = Utt::new();
    u.say_as("skyward two four seven", "Skyward 247").comma().runway("27R").comma().say("cleared to land");
    let toks = u.finish().toks;
    let dur = radio::duration(Voice::Fable, &toks);
    assert!(dur > 2.0 && dur < 6.0, "{dur}");
    let id = radio::submit(radio::Tx { voice: Voice::Fable, toks, ch: 0, own: false, strength: 0.8, volume: 1.0 });
    let sr = 48_000.0;
    let mut r = radio::Radio::new(sr);
    r.start(id);
    let n = ((dur + 0.3) * sr) as usize;
    let out: Vec<f32> = (0..n).map(|_| r.tick()).collect();
    let peak = out.iter().fold(0.0f32, |m, x| m.max(x.abs()));
    let rms = (out.iter().map(|x| x * x).sum::<f32>() / n as f32).sqrt();
    assert!(peak < 0.95 && rms > 0.03, "peak {peak} rms {rms}");
    // Band-limited: little energy above 4.5 kHz (a crude high-pass difference test).
    let hf = (out.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f32>() / n as f32).sqrt();
    assert!(hf < rms * 0.6, "hf {hf} vs rms {rms}");
    // Silent at the end (squelch closed).
    let tail = &out[n - 2000..];
    assert!(tail.iter().all(|x| x.abs() < 1e-3));
}
