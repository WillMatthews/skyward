//! Where the aircraft crashed, in words (Track CA): "at 12 Bath Road,
//! Hounslow TW6 2AA, United Kingdom", "3 km NW of Lukla, Nepal", "in the
//! English Channel, 18 km S of Dover".
//!
//! One reverse-geocoding request per crash to OpenStreetMap's Nominatim
//! (`/reverse`, data © OpenStreetMap contributors, ODbL — the crash panel
//! shows the attribution next to the address). Its usage policy is kept: a
//! single request per crash and never more than one a second, a generic
//! User-Agent naming the project (no user or machine details; the browser
//! build sends the browser's own), answers cached (in memory, and natively
//! under `cache/geocode/`). Nominatim sends CORS headers
//! (`Access-Control-Allow-Origin: *`, checked), so the browser build asks it
//! too.
//!
//! The request never blocks a frame: the crash panel shows "Locating…" until
//! the answer arrives. Offline, on failure or with `SKYWARD_NO_GEOCODE=1`
//! (headless captures unless `--geocode`), the place is described from what
//! is already loaded: the nearest airport (airport database) or named place
//! (the map tiles' place labels), e.g. "4 km SE of EGLL London Heathrow"; at
//! sea "in the water, 18 km S of Dover".

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

/// Shown next to an address that came from OpenStreetMap data.
pub const ATTRIBUTION: &str = "© OpenStreetMap contributors";

const ENDPOINT: &str = "https://nominatim.openstreetmap.org/reverse";
/// Give up on the network after this long (the fallback text stays).
const PATIENCE: Duration = Duration::from_secs(12);

/// The User-Agent (generic: the project, not the user or the machine).
pub fn user_agent() -> String {
    format!("Skyward-flight-sim/{} (+https://github.com/WillMatthews/skyward)", env!("CARGO_PKG_VERSION"))
}

static DISABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Turn the network lookup off (headless captures: goldens never hit the network).
pub fn set_disabled(on: bool) {
    DISABLED.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// Switched off (`SKYWARD_NO_GEOCODE=1`, headless captures): the offline
/// description only, not even cached answers, so captures are reproducible.
pub fn forced_off() -> bool {
    DISABLED.load(std::sync::atomic::Ordering::Relaxed) || std::env::var_os("SKYWARD_NO_GEOCODE").is_some()
}

/// No request: switched off, or offline mode (cached answers still count).
pub fn disabled() -> bool {
    forced_off() || crate::platform::offline()
}

pub fn url(lat: f64, lon: f64) -> String {
    format!("{ENDPOINT}?format=jsonv2&lat={lat:.5}&lon={lon:.5}&zoom=18&addressdetails=1&accept-language=en")
}

// ---------------------------------------------------------------- Nominatim's answer

/// What Nominatim says about a point.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Reply {
    /// A street-level address ("12 Bath Road, Hounslow TW6 2AA, United Kingdom").
    pub address: Option<String>,
    /// A named sea / bay / lake at the spot.
    pub water: Option<String>,
    /// The settlement it names, when there is no street (hamlet, village, town).
    pub near: Option<String>,
    /// County / state.
    pub region: Option<String>,
    pub country: Option<String>,
}

impl Reply {
    /// Nothing below the state / country: open water, or nowhere in particular.
    pub fn nothing_local(&self) -> bool {
        self.address.is_none() && self.water.is_none() && self.near.is_none()
    }
}

/// Read a `format=jsonv2` reverse answer. `{"error":"Unable to geocode"}`
/// (open sea) is an empty reply, not an error.
pub fn parse(text: &str) -> Result<Reply, String> {
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("bad JSON: {e}"))?;
    if let Some(e) = v.get("error") {
        let e = e.as_str().unwrap_or("error");
        return if e.contains("Unable to geocode") { Ok(Reply::default()) } else { Err(e.to_string()) };
    }
    let a = v.get("address").ok_or("no address")?;
    let s = |o: &serde_json::Value, k: &str| o.get(k).and_then(|x| x.as_str()).map(str::trim).filter(|x| !x.is_empty()).map(str::to_string);
    let first = |keys: &[&str]| keys.iter().find_map(|k| s(a, k));
    let cat = s(&v, "category").unwrap_or_default();
    let typ = s(&v, "type").unwrap_or_default();
    let addresstype = s(&v, "addresstype").unwrap_or_default();
    let name = s(&v, "name");

    let road = first(&["road", "pedestrian", "square", "footway", "cycleway"]);
    let small = first(&["suburb", "quarter", "neighbourhood", "village", "hamlet", "isolated_dwelling"]);
    let big = first(&["town", "city", "municipality"]).filter(|b| Some(b) != small.as_ref());
    let postcode = s(a, "postcode");
    let country = s(a, "country");
    let region = first(&["county", "state_district", "state"]);

    const WATER_TYPES: [&str; 9] = ["sea", "ocean", "bay", "strait", "water", "lake", "reservoir", "river", "lagoon"];
    let water = name
        .clone()
        .filter(|_| (matches!(cat.as_str(), "natural" | "water" | "waterway" | "place") && WATER_TYPES.contains(&typ.as_str())) || WATER_TYPES.contains(&addresstype.as_str()));

    let address = match (&road, small.is_some() || big.is_some()) {
        (Some(road), true) => {
            let mut parts: Vec<String> = Vec::new();
            // A named place on the street (a terminal, a farm), not the street itself.
            if let Some(n) = name.as_ref().filter(|n| *n != road && !matches!(cat.as_str(), "highway" | "boundary" | "place") && water.is_none()) {
                parts.push(n.clone());
            }
            parts.push(match s(a, "house_number") {
                Some(h) => format!("{h} {road}"),
                None => road.clone(),
            });
            parts.extend(small.iter().cloned());
            parts.extend(big.iter().cloned());
            if let Some(pc) = &postcode {
                let last = parts.last_mut().unwrap();
                last.push(' ');
                last.push_str(pc);
            }
            parts.extend(country.iter().cloned());
            Some(parts.join(", "))
        }
        _ => None,
    };
    Ok(Reply { address, water, near: small.or(big), region, country })
}

// ---------------------------------------------------------------- from what is loaded

/// A named reference point: an airport or a place label.
#[derive(Clone, Debug, PartialEq)]
pub struct Place {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    /// Bigger = preferred at a distance (a city beats a nearer hamlet).
    pub weight: f64,
    pub airport: bool,
}

impl Place {
    /// "EGLL London Heathrow" from the airport database.
    pub fn airport(a: &crate::airports::Airport) -> Place {
        Place::airport_named(&a.ident, &a.name, a.lat, a.lon)
    }

    pub fn airport_named(ident: &str, name: &str, lat: f64, lon: f64) -> Place {
        let mut n = name.to_string();
        for cut in [" International Airport", " Airport", " Aerodrome", " Airfield", " Air Base", " Airstrip"] {
            if let Some(s) = n.strip_suffix(cut) {
                n = s.to_string();
                break;
            }
        }
        let name = if n.is_empty() || n == ident { ident.to_string() } else { format!("{ident} {n}") };
        Place { name, lat, lon, weight: 1.6, airport: true }
    }

    /// A map place label (cities count for more than villages).
    pub fn label(name: &str, lat: f64, lon: f64, kind: crate::ui::mapdata::LabelKind) -> Option<Place> {
        use crate::ui::mapdata::LabelKind as K;
        let weight = match kind {
            K::City => 4.0,
            K::Town => 2.5,
            K::Village => 1.5,
            K::Suburb => 1.0,
            K::Water => return None,
        };
        Some(Place { name: name.to_string(), lat, lon, weight, airport: false })
    }
}

/// Compass point from `from` to `to` (8 points).
pub fn compass(from: (f64, f64), to: (f64, f64)) -> &'static str {
    let dn = to.0 - from.0;
    let de = (to.1 - from.1) * from.0.to_radians().cos();
    let brg = de.atan2(dn).to_degrees().rem_euclid(360.0);
    ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][((brg + 22.5) / 45.0) as usize % 8]
}

/// "3 km NW of Lukla", "near Dover", "at EGLL London Heathrow", and the
/// distance (km), from the best reference point.
pub fn relative(lat: f64, lon: f64, refs: &[Place]) -> Option<(String, f64)> {
    let best = refs.iter().map(|p| (p, crate::airports::distance_km((p.lat, p.lon), (lat, lon)))).min_by(|a, b| (a.1 / a.0.weight).total_cmp(&(b.1 / b.0.weight)))?;
    let (p, d) = best;
    let text = if p.airport && d < 2.5 {
        format!("at {}", p.name)
    } else if d < 1.5 {
        format!("near {}", p.name)
    } else {
        format!("{:.0} km {} of {}", d, compass((p.lat, p.lon), (lat, lon)), p.name)
    };
    Some((text, d))
}

fn the(water: &str) -> String {
    let bare = ["Lake ", "Loch ", "Lac ", "Lago ", "The ", "the "].iter().any(|p| water.starts_with(p));
    if bare { water.to_string() } else { format!("the {water}") }
}

/// Coordinates as a last resort ("51.4700° N, 0.4543° W").
pub fn coords(lat: f64, lon: f64) -> String {
    format!("{:.4}° {}, {:.4}° {}", lat.abs(), if lat >= 0.0 { "N" } else { "S" }, lon.abs(), if lon >= 0.0 { "E" } else { "W" })
}

/// The phrase after "Crashed": from Nominatim's reply if there is one, else
/// from the reference points. `water`: the wreck is in water (the sim knows).
pub fn describe(lat: f64, lon: f64, water: bool, reply: Option<&Reply>, refs: &[Place]) -> String {
    let rel = relative(lat, lon, refs);
    // A reference farther than this says little ("212 km SW of …").
    let rel_near = rel.as_ref().filter(|r| r.1 <= 40.0).map(|r| r.0.clone());
    let rel_any = rel.as_ref().filter(|r| r.1 <= 400.0).map(|r| r.0.clone());
    let joined = |head: String, tail: Option<String>| match tail {
        Some(t) if t.starts_with("at ") => format!("{head}, off {}", &t[3..]),
        Some(t) => format!("{head}, {t}"),
        None => head,
    };
    if let Some(r) = reply {
        if let Some(a) = &r.address {
            return format!("at {a}");
        }
        if let Some(w) = &r.water {
            return joined(format!("in {}", the(w)), rel_any);
        }
        if water {
            let head = if r.nothing_local() { "in the sea" } else { "in the water" };
            return joined(head.into(), rel_any.or_else(|| r.country.as_ref().map(|c| format!("off {c}"))));
        }
        let country = r.country.clone();
        if let Some(t) = rel_near {
            return [Some(t), country].into_iter().flatten().collect::<Vec<_>>().join(", ");
        }
        if let Some(n) = &r.near {
            return format!("near {}", [Some(n.clone()), r.region.clone(), country].into_iter().flatten().collect::<Vec<_>>().join(", "));
        }
        if r.region.is_some() || country.is_some() {
            return format!("in {}", [r.region.clone(), country].into_iter().flatten().collect::<Vec<_>>().join(", "));
        }
    }
    if water {
        return joined("in the water".into(), rel_any);
    }
    rel_any.unwrap_or_else(|| format!("at {}", coords(lat, lon)))
}

// ---------------------------------------------------------------- the lookup

type Slot = Arc<Mutex<Option<Result<Reply, String>>>>;

enum Net {
    /// Not asked (disabled, offline).
    Off,
    Pending(Slot, Instant),
    Done(Option<Reply>),
}

static CACHE: Mutex<Option<HashMap<(i64, i64), Reply>>> = Mutex::new(None);
/// The last request (at most one a second, however many crashes).
static LAST: Mutex<Option<Instant>> = Mutex::new(None);

fn key(lat: f64, lon: f64) -> (i64, i64) {
    ((lat * 1e4).round() as i64, (lon * 1e4).round() as i64)
}

#[cfg(not(target_arch = "wasm32"))]
fn disk_path(k: (i64, i64)) -> std::path::PathBuf {
    std::path::PathBuf::from("cache").join("geocode").join(format!("{}_{}.json", k.0, k.1))
}

/// Wait until a second has passed since the last request; claim the slot.
fn turn() -> Duration {
    let mut l = LAST.lock().unwrap();
    let now = Instant::now();
    let wait = l.map(|t| Duration::from_millis(1100).saturating_sub(now.duration_since(t))).unwrap_or_default();
    *l = Some(now + wait);
    wait
}

fn remember(k: (i64, i64), r: &Reply) {
    CACHE.lock().unwrap().get_or_insert_with(HashMap::new).insert(k, r.clone());
}

fn start(lat: f64, lon: f64) -> Net {
    let k = key(lat, lon);
    if forced_off() {
        return Net::Off;
    }
    if let Some(r) = CACHE.lock().unwrap().as_ref().and_then(|c| c.get(&k)) {
        return Net::Done(Some(r.clone()));
    }
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(r) = std::fs::read_to_string(disk_path(k)).ok().and_then(|t| parse(&t).ok()) {
        remember(k, &r);
        return Net::Done(Some(r));
    }
    if disabled() {
        return Net::Off;
    }
    let slot: Slot = Arc::new(Mutex::new(None));
    let out = slot.clone();
    let u = url(lat, lon);
    #[cfg(not(target_arch = "wasm32"))]
    {
        let ok = crate::platform::spawn("geocode", move || {
            std::thread::sleep(turn());
            let agent = ureq::AgentBuilder::new().timeout_connect(std::time::Duration::from_secs(5)).timeout(std::time::Duration::from_secs(10)).user_agent(&user_agent()).build();
            let r = match agent.get(&u).call() {
                Ok(resp) => resp.into_string().map_err(|e| e.to_string()).and_then(|t| {
                    let r = parse(&t)?;
                    let p = disk_path(k);
                    if let Some(d) = p.parent() {
                        let _ = std::fs::create_dir_all(d);
                    }
                    let _ = std::fs::write(&p, &t);
                    Ok(r)
                }),
                Err(e) => Err(e.to_string()),
            };
            if let Ok(r) = &r {
                remember(k, r);
            }
            *out.lock().unwrap() = Some(r);
        });
        if !ok {
            return Net::Off;
        }
    }
    #[cfg(target_arch = "wasm32")]
    crate::platform::web::spawn_local(async move {
        use crate::platform::web;
        let wait = turn();
        if !wait.is_zero() {
            web::sleep(wait).await;
        }
        let r = match web::fetch(&u, Duration::from_secs(10)).await {
            Ok(resp) if (200..300).contains(&resp.status) => parse(&String::from_utf8_lossy(&resp.body)),
            Ok(resp) => Err(format!("HTTP {}", resp.status)),
            Err(e) => Err(e),
        };
        if let Ok(r) = &r {
            remember(k, r);
        }
        *out.lock().unwrap() = Some(r);
    });
    Net::Pending(slot, Instant::now())
}

/// One crash's place: the lookup in flight, the reference points, the text.
pub struct CrashSite {
    pub lat: f64,
    pub lon: f64,
    /// The wreck is in water.
    pub water: bool,
    airport: Option<Place>,
    /// Map place labels near the site (looked up once the UI has them).
    places: Option<Vec<Place>>,
    net: Net,
    logged: bool,
}

impl CrashSite {
    /// Start the lookup (unless disabled) and note the nearest airport.
    pub fn new(lat: f64, lon: f64, water: bool) -> CrashSite {
        let airport = crate::terrain::airport_db().and_then(|db| {
            let near = db.near(lat, lon, 150.0);
            // Prefer a real airport over a heliport / closed strip at a similar distance.
            let score = |i: usize, d: f64| {
                use crate::airports::Kind;
                let a = &db.airports[i];
                d / match a.kind {
                    Kind::Large => 3.0,
                    Kind::Medium => 2.0,
                    Kind::Small => 1.0,
                    Kind::Seaplane => 0.5,
                }
            };
            near.into_iter().min_by(|a, b| score(a.0, a.1).total_cmp(&score(b.0, b.1))).map(|(i, _)| Place::airport(&db.airports[i]))
        });
        CrashSite { lat, lon, water, airport, places: None, net: start(lat, lon), logged: false }
    }

    /// Pick up the answer if it has arrived (or stop waiting for it).
    pub fn poll(&mut self) {
        if let Net::Pending(slot, t0) = &self.net {
            let got = slot.lock().unwrap().take();
            if let Some(r) = got {
                if let Err(e) = &r {
                    eprintln!("geocode: {e}");
                }
                self.net = Net::Done(r.ok());
            } else if t0.elapsed() > PATIENCE {
                eprintln!("geocode: no answer after {} s", PATIENCE.as_secs());
                self.net = Net::Done(None);
            }
        }
    }

    /// Still waiting for Nominatim.
    pub fn locating(&self) -> bool {
        matches!(self.net, Net::Pending(..))
    }

    /// The map's place labels have not been looked at yet.
    pub fn needs_places(&self) -> bool {
        self.places.is_none()
    }

    pub fn set_places(&mut self, places: Vec<Place>) {
        self.places = Some(places);
    }

    fn reply(&self) -> Option<&Reply> {
        match &self.net {
            Net::Done(r) => r.as_ref(),
            _ => None,
        }
    }

    fn refs(&self) -> Vec<Place> {
        self.airport.iter().chain(self.places.iter().flatten()).cloned().collect()
    }

    /// The phrase after "Crashed" ("at 12 Bath Road, …").
    pub fn text(&self) -> String {
        describe(self.lat, self.lon, self.water, self.reply(), &self.refs())
    }

    /// Show "© OpenStreetMap contributors": the text uses OSM data.
    pub fn osm(&self) -> bool {
        self.reply().is_some() || self.places.as_ref().is_some_and(|p| !p.is_empty())
    }

    /// Once settled, the line for the log (only the first time).
    pub fn log_line(&mut self) -> Option<String> {
        if self.logged || self.locating() {
            return None;
        }
        self.logged = true;
        Some(format!("Crashed {} ({})", self.text(), coords(self.lat, self.lon)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const URBAN: &str = r#"{"place_id":278929166,"licence":"Data © OpenStreetMap contributors, ODbL 1.0. http://osm.org/copyright","osm_type":"node","osm_id":10052045215,"lat":"51.4704303","lon":"-0.4535904","category":"emergency","type":"designated","place_rank":30,"addresstype":"emergency","name":"","display_name":"Cosmopolitan Way, Harmondsworth, London Borough of Hillingdon, Greater London, England, TW6 1SL, United Kingdom","address":{"road":"Cosmopolitan Way","suburb":"Harmondsworth","city_district":"London Borough of Hillingdon","ISO3166-2-lvl8":"GB-HIL","city":"Greater London","state":"England","ISO3166-2-lvl4":"GB-ENG","postcode":"TW6 1SL","country":"United Kingdom","country_code":"gb"}}"#;
    const HOUSE: &str = r#"{"category":"building","type":"house","addresstype":"building","name":"","address":{"house_number":"14","road":"Helmsdale Walk","suburb":"Park South","town":"Swindon","county":"Swindon","state":"England","postcode":"SN3 2DG","country":"United Kingdom","country_code":"gb"}}"#;
    const RURAL: &str = r#"{"place_id":243070127,"category":"place","type":"hamlet","place_rank":20,"addresstype":"hamlet","name":"Syanjma","display_name":"Syanjma, Khumbupasanglahmu-02, Khumbu Pasang Lhamu, Solukhumbu, Koshi Province, Nepal","address":{"hamlet":"Syanjma","city_district":"Khumbupasanglahmu-02","town":"Khumbu Pasang Lhamu","municipality":"Khumbu Pasang Lhamu","county":"Solukhumbu","state":"Koshi Province","ISO3166-2-lvl4":"NP-P1","country":"Nepal","country_code":"np"}}"#;
    /// In territorial waters Nominatim answers with the state only.
    const COASTAL: &str = r#"{"category":"boundary","type":"administrative","place_rank":8,"addresstype":"state","name":"England","display_name":"England, United Kingdom","address":{"state":"England","ISO3166-2-lvl4":"GB-ENG","country":"United Kingdom","country_code":"gb"}}"#;
    const OPEN_SEA: &str = r#"{"error":"Unable to geocode"}"#;
    const NAMED_SEA: &str = r#"{"category":"natural","type":"strait","addresstype":"strait","name":"English Channel","address":{"strait":"English Channel"}}"#;

    #[test]
    fn urban_addresses() {
        let r = parse(URBAN).unwrap();
        assert_eq!(r.address.as_deref(), Some("Cosmopolitan Way, Harmondsworth, Greater London TW6 1SL, United Kingdom"));
        assert_eq!(describe(51.47, -0.45, false, Some(&r), &[]), "at Cosmopolitan Way, Harmondsworth, Greater London TW6 1SL, United Kingdom");
        let h = parse(HOUSE).unwrap();
        assert_eq!(h.address.as_deref(), Some("14 Helmsdale Walk, Park South, Swindon SN3 2DG, United Kingdom"));
    }

    #[test]
    fn rural_is_relative() {
        let r = parse(RURAL).unwrap();
        assert_eq!(r.address, None);
        assert_eq!(r.near.as_deref(), Some("Syanjma"));
        assert_eq!(r.country.as_deref(), Some("Nepal"));
        let lukla = Place { name: "Lukla".into(), lat: 27.6869, lon: 86.7314, weight: 2.5, airport: false };
        // ~4 km NW of Lukla.
        assert_eq!(describe(27.7100, 86.7000, false, Some(&r), std::slice::from_ref(&lukla)), "4 km NW of Lukla, Nepal");
        // No reference point close by: Nominatim's own names.
        assert_eq!(describe(27.7100, 86.7000, false, Some(&r), &[]), "near Syanjma, Solukhumbu, Nepal");
    }

    #[test]
    fn at_sea() {
        let dover = Place { name: "Dover".into(), lat: 51.1279, lon: 1.3134, weight: 2.5, airport: false };
        let r = parse(OPEN_SEA).unwrap();
        assert!(r.nothing_local());
        assert_eq!(describe(50.97, 1.31, true, Some(&r), std::slice::from_ref(&dover)), "in the sea, 18 km S of Dover");
        let c = parse(COASTAL).unwrap();
        assert!(c.nothing_local() && c.country.as_deref() == Some("United Kingdom"));
        assert_eq!(describe(50.97, 1.31, true, Some(&c), &[]), "in the sea, off United Kingdom");
        let n = parse(NAMED_SEA).unwrap();
        assert_eq!(n.water.as_deref(), Some("English Channel"));
        assert_eq!(describe(50.97, 1.31, true, Some(&n), std::slice::from_ref(&dover)), "in the English Channel, 18 km S of Dover");
    }

    #[test]
    fn errors() {
        assert!(parse(r#"{"error":"Invalid parameters"}"#).is_err());
        assert!(parse("<html>Too many requests</html>").is_err());
        assert!(parse(r#"{"place_id":1}"#).is_err());
    }

    #[test]
    fn fallback_text() {
        let egll = Place { name: "EGLL London Heathrow".into(), lat: 51.4706, lon: -0.4619, weight: 1.6, airport: true };
        assert_eq!(describe(51.4700, -0.4543, false, None, std::slice::from_ref(&egll)), "at EGLL London Heathrow");
        // 4 km south-east of the field.
        assert_eq!(describe(51.4450, -0.4220, false, None, std::slice::from_ref(&egll)), "4 km SE of EGLL London Heathrow");
        // A town beats a farther airport; a near one is "near".
        let hounslow = Place { name: "Hounslow".into(), lat: 51.4673, lon: -0.3613, weight: 2.5, airport: false };
        assert_eq!(describe(51.4700, -0.3700, false, None, &[egll.clone(), hounslow]), "near Hounslow");
        // Water, offline.
        let dover = Place { name: "Dover".into(), lat: 51.1279, lon: 1.3134, weight: 2.5, airport: false };
        assert_eq!(describe(50.97, 1.31, true, None, &[dover]), "in the water, 18 km S of Dover");
        assert_eq!(describe(51.4706, -0.40, true, None, &[egll]), "in the water, 4 km E of EGLL London Heathrow");
        // Nothing known: coordinates.
        assert_eq!(describe(-12.5, 140.25, false, None, &[]), "at 12.5000° S, 140.2500° E");
        assert_eq!(compass((0.0, 0.0), (1.0, 0.0)), "N");
        assert_eq!(compass((0.0, 0.0), (-1.0, -1.0)), "SW");
    }

    #[test]
    fn airport_names_shortened() {
        assert_eq!(Place::airport_named("EGLL", "London Heathrow Airport", 51.47, -0.46).name, "EGLL London Heathrow");
        assert_eq!(Place::airport_named("VNLK", "Tenzing-Hillary Airport", 27.69, 86.73).name, "VNLK Tenzing-Hillary");
        assert!(user_agent().starts_with("Skyward-flight-sim/") && user_agent().contains("(+https://"));
        assert!(url(51.47, -0.4543).contains("lat=51.47000&lon=-0.45430&zoom=18&addressdetails=1"));
    }
}
