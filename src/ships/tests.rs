use super::density::{self, Layer, N, Sample};
use super::sim::{self, Rng, Sea, Ship, navigable};
use super::types::{Class, Model, class_of, dims, model_for};
use super::{ais, models};
use glam::{Vec2, vec2};

// ---------------------------------------------------------------- AIS

const LOCATIONS: &str = include_str!("../../tests/ships/locations.json");
const VESSELS: &str = include_str!("../../tests/ships/vessels.json");

#[test]
fn digitraffic_locations_parse() {
    let f = ais::parse_locations(LOCATIONS).unwrap();
    assert_eq!(f.len(), 6);
    let pilot = f.iter().find(|x| x.mmsi == 230033880).unwrap();
    assert!((pilot.lat - 60.165878).abs() < 1e-9 && (pilot.lon - 24.98325).abs() < 1e-9);
    assert!((pilot.sog - 12.2).abs() < 1e-4);
    assert_eq!(pilot.heading, Some(156.0));
    assert_eq!(pilot.best_heading(), Some(156.0));
    assert!(pilot.t > 1.7e9 && pilot.t < 2.0e9, "report time in Unix seconds: {}", pilot.t);
    assert!(!pilot.stopped());
    // 511 = no heading; moored (5) and slow ones are stopped.
    let moored = f.iter().find(|x| x.mmsi == 230055040).unwrap();
    assert_eq!(moored.heading, None);
    assert_eq!(moored.nav_stat, 5);
    assert!(moored.stopped());
    assert_eq!(moored.best_heading(), None);
}

#[test]
fn digitraffic_vessels_parse_and_classify() {
    let v = ais::parse_vessels(VESSELS).unwrap();
    assert_eq!(v.len(), 6);
    let ella = v.iter().find(|x| x.mmsi == 273290820).unwrap();
    assert_eq!(ella.name, "ELLA");
    assert_eq!(ella.ship_type, 70);
    assert_eq!(ella.a as u32 + ella.b as u32, 141);
    assert!((ella.draught - 4.7).abs() < 1e-4);
    assert_eq!(class_of(ella.ship_type, 141.0, &ella.name), Class::Cargo);
    let pilot = v.iter().find(|x| x.mmsi == 230033880).unwrap();
    assert_eq!(class_of(pilot.ship_type, 15.0, &pilot.name), Class::Service);
}

#[test]
fn bad_ais_input_is_skipped_not_fatal() {
    assert!(ais::parse_locations("not json").is_err());
    let odd = r#"{"features":[{"mmsi":1,"geometry":{"coordinates":[200.0,95.0]},"properties":{}},
        {"mmsi":2,"geometry":{"coordinates":[24.0,60.0]},"properties":{"sog":102.3,"cog":360.0,"heading":511,"navStat":99}},
        {"geometry":{"coordinates":[24.0,60.0]},"properties":{}}]}"#;
    let f = ais::parse_locations(odd).unwrap();
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].sog, 0.0);
    assert_eq!(f[0].heading, None);
    assert_eq!(f[0].nav_stat, 15);
    let v = ais::parse_vessels(r#"[{"mmsi":5,"name":"A\u0007B","referencePointA":9999,"draught":-4}]"#).unwrap();
    assert_eq!(v[0].name, "AB");
    assert_eq!(v[0].a, 511);
    assert_eq!(v[0].draught, 0.0);
}

#[test]
fn dead_reckoning_follows_the_course_and_stops_after_three_minutes() {
    let f = ais::Fix { mmsi: 1, lat: 60.0, lon: 25.0, sog: 10.0, cog: 90.0, heading: Some(90.0), nav_stat: 0, t: 1000.0 };
    let (la, lo) = ais::dead_reckon(&f, 1060.0);
    // 10 kn east for a minute ≈ 308.7 m.
    let dx = (lo - 25.0) * crate::osm::M_PER_DEG_LAT * 60f64.to_radians().cos();
    assert!((dx - 308.67).abs() < 1.0, "{dx}");
    assert!((la - 60.0).abs() < 1e-9);
    let far = ais::dead_reckon(&f, 1000.0 + 3600.0);
    let capped = ais::dead_reckon(&f, 1000.0 + 180.0);
    assert_eq!(far, capped);
    let moored = ais::Fix { nav_stat: 5, ..f };
    assert_eq!(ais::dead_reckon(&moored, 1060.0), (60.0, 25.0));
}

#[test]
fn poller_is_polite() {
    let mut p = ais::Poller::default();
    // Metadata first, then positions once a minute.
    assert_eq!(p.due(0.0, 0), Some(ais::Due::Vessels));
    p.sent();
    assert_eq!(p.due(0.0, 0), None, "one request at a time");
    p.done(0.0, ais::Due::Vessels, true);
    assert_eq!(p.due(0.0, 0), Some(ais::Due::Fixes));
    p.sent();
    p.done(1.0, ais::Due::Fixes, true);
    assert_eq!(p.due(30.0, 0), None);
    assert_eq!(p.due(61.0, 0), Some(ais::Due::Fixes));
    // Failures back off.
    p.sent();
    p.done(61.0, ais::Due::Fixes, false);
    p.sent();
    p.done(61.0, ais::Due::Fixes, false);
    assert_eq!(p.due(100.0, 0), None);
    assert!(p.next_fix >= 61.0 + 60.0);
}

#[test]
fn merging_reports_keeps_the_newest() {
    let mut tracks = std::collections::HashMap::new();
    let statics = std::collections::HashMap::new();
    let f = ais::Fix { mmsi: 7, lat: 60.0, lon: 25.0, sog: 0.0, cog: 360.0, heading: None, nav_stat: 1, t: 1000.0 };
    assert_eq!(ais::merge(&mut tracks, vec![f.clone()], &statics, 1001.0), 1);
    ais::merge(&mut tracks, vec![ais::Fix { t: 900.0, lat: 61.0, ..f.clone() }], &statics, 1002.0);
    assert_eq!(tracks[&7].fix.lat, 60.0, "an older report does not replace a newer one");
    ais::merge(&mut tracks, vec![], &statics, 1000.0 + 1300.0);
    assert!(tracks.is_empty(), "stale tracks expire");
}

// ---------------------------------------------------------------- types

#[test]
fn ais_types_map_to_sensible_models() {
    let cases: [(u8, f32, &str, Class, Model); 14] = [
        (70, 120.0, "ELLA", Class::Cargo, Model::Freighter),
        (79, 366.0, "EVER X", Class::Container, Model::Container),
        (80, 250.0, "TANKER", Class::Tanker, Model::Tanker),
        (60, 212.0, "VIKING GRACE", Class::Passenger, Model::Ferry),
        (69, 290.0, "AIDA", Class::Cruise, Model::Cruise),
        (60, 60.0, "SUOMENLINNA II", Class::Passenger, Model::FerrySmall),
        (40, 40.0, "", Class::HighSpeed, Model::FerrySmall),
        (30, 20.0, "", Class::Fishing, Model::Trawler),
        (30, 60.0, "", Class::Fishing, Model::TrawlerLarge),
        (36, 12.0, "", Class::Sailing, Model::Sailboat),
        (37, 15.0, "", Class::Pleasure, Model::Motorsailer),
        (52, 30.0, "", Class::Tug, Model::Workboat),
        (35, 120.0, "", Class::Military, Model::Patrol),
        (0, 85.0, "", Class::Barge, Model::Barge),
    ];
    for (ty, len, name, class, model) in cases {
        let c = class_of(ty, len, name);
        assert_eq!(c, class, "type {ty} length {len}");
        assert_eq!(model_for(c, len), model, "type {ty} length {len}");
    }
    // Every class has a model and a sensible size / speed / draught.
    for c in Class::ALL {
        let (lo, hi) = c.length_range();
        assert!(lo > 0.0 && hi > lo);
        let (a, b) = c.speed_kn();
        assert!(a > 0.0 && b > a && b < 40.0);
        assert!(c.draught(hi) > 0.3 && c.draught(hi) < 23.0);
        let _ = model_for(c, lo);
    }
    // Dimensions: AIS A+B / C+D when given, the class's shape otherwise.
    assert_eq!(dims(Class::Cargo, 120, 21, 10, 12, 50.0), (141.0, 22.0));
    let (l, b) = dims(Class::Cargo, 0, 0, 0, 0, 100.0);
    assert_eq!(l, 100.0);
    assert!((b - 16.0).abs() < 1e-3);
}

#[test]
fn every_imported_hull_loads_and_is_ship_shaped() {
    for m in Model::IMPORTED {
        let s = models::load(m).unwrap_or_else(|e| panic!("{m:?}: {e}"));
        let (lo, hi) = (s.lo, s.hi);
        let len = s.length();
        let beam = hi.x - lo.x;
        // Long along +Z (bow), waterline near y = 0 (hull below, superstructure above).
        assert!(len > beam * 1.8, "{m:?}: length {len} beam {beam}");
        assert!(lo.y < 0.5 && hi.y > 1.0, "{m:?}: y {} .. {}", lo.y, hi.y);
        assert!(!s.frame(glam::Mat4::IDENTITY).draws.is_empty());
    }
    for m in [Model::Workboat, Model::Patrol] {
        let s = models::procedural(m);
        assert!((s.length() - 30.0).abs() < 1.0);
        assert!(s.lo.y < 0.0 && s.hi.y > 2.0);
    }
}

// ---------------------------------------------------------------- density

#[test]
fn density_tiles_decode_and_cache() {
    use crate::cog::tiff::fixture::tiff_f32;
    let px = density::PX;
    // A lane across the middle; GeoServer's no-data on the "land" quarter.
    let tif = tiff_f32(
        px,
        px,
        64,
        64,
        |x, y| {
            if x < px / 4 {
                -3.4028235e38
            } else if (y as i32 - 64).abs() < 4 {
                730.0
            } else {
                7.3
            }
        },
        [1.0, 1.0, 0.0],
        [0.0; 6],
        false,
    );
    let v = density::decode(&tif).unwrap();
    assert!(v[0].is_nan());
    assert!((v[64 * px as usize + 100] - 1.0).abs() < 1e-6, "730 h/km²/month = one ship per km²");
    assert!((v[10 * px as usize + 100] - 0.01).abs() < 1e-6);
    let t = density::Tile { layers: (0..N).map(|_| v.clone()).collect() };
    let back = density::decode_cache(&density::encode_cache(&t)).unwrap();
    assert!(back.layers[3][0].is_nan());
    assert!((back.layers[3][64 * px as usize + 100] - 1.0).abs() < 1e-3);
    assert!(density::decode(b"<?xml version").is_err());
    match density::sample_tile(&t, 0.01, 0.5) {
        Sample::Land => {}
        s => panic!("{s:?}"),
    }
    match density::sample_tile(&t, 0.8, 0.5) {
        Sample::Ships(x) => assert!((x[0] - 1.0).abs() < 1e-6),
        s => panic!("{s:?}"),
    }
}

#[test]
fn density_tiles_are_web_mercator_z8() {
    // The Solent.
    let (k, fx, fy) = density::Key::at(50.8, -1.3);
    assert_eq!((k.x, k.y), (127, 85));
    assert!((0.0..1.0).contains(&fx) && (0.0..1.0).contains(&fy));
    let (x0, y0, x1, y1) = k.bounds();
    let (mx, my) = density::merc(50.8, -1.3);
    assert!(mx > x0 && mx < x1 && my > y0 && my < y1);
    assert!(density::url("09", k).contains("vesseldensity_09avg") && density::url("09", k).contains("scalesize=i(128),j(128)"));
    // A 27 km disc never needs more than the 2×2 tiles around it.
    assert!(density::keys_around(50.8, -1.3, 27_000.0).len() <= 4);
}

// ---------------------------------------------------------------- spawning

/// A synthetic sea: a 4 km wide east-west channel (z in -2000..2000) with
/// a lane of traffic along its middle, a 300 m strip of shoal water (2 m)
/// along its north edge, land elsewhere. An island sits in the channel at x = 3 km.
struct Channel;

impl Channel {
    fn island(xz: Vec2) -> bool {
        xz.distance(vec2(3000.0, 0.0)) < 500.0
    }
}

impl Sea for Channel {
    fn water(&self, xz: Vec2) -> Option<bool> {
        Some(xz.y.abs() < 2000.0 && !Self::island(xz))
    }
    fn depth(&self, xz: Vec2) -> Option<f32> {
        if self.water(xz) != Some(true) {
            return Some(-1.0);
        }
        Some(if xz.y < -1700.0 { 2.0 } else { 25.0 })
    }
    fn density(&self, xz: Vec2) -> Sample {
        if self.water(xz) != Some(true) {
            return Sample::Land;
        }
        let mut v = [0.0f32; N];
        // Cargo in a lane along z = 500, leisure everywhere in the channel.
        v[Layer::Cargo.index()] = 0.3 * (-((xz.y - 500.0) / 400.0).powi(2)).exp();
        v[Layer::Sailing.index()] = 0.02;
        v[Layer::Fishing.index()] = 0.01;
        Sample::Ships(v)
    }
}

#[test]
fn spawns_follow_the_density_and_stay_on_deep_enough_water() {
    let sea = Channel;
    let cen = sim::census(&sea, Vec2::ZERO, 10_000.0, 250.0);
    let exp_cargo = cen.expected[Layer::Cargo.index()];
    // Lane: 0.3 / km² × ∫ exp(-(z/400)²) dz ≈ 0.3 × 0.709 km × 20 km of channel.
    assert!((exp_cargo - 0.3 * 0.709 * 20.0).abs() < 0.6, "{exp_cargo}");
    assert!(cen.cells.iter().all(|c| sea.water(c.0) == Some(true)));
    let mut rng = Rng(42);
    let mut ships: Vec<Ship> = Vec::new();
    let mut lane = 0;
    let (mut moving, mut along) = (0, 0);
    for i in 0..400 {
        let l = if i % 2 == 0 { Layer::Cargo } else { Layer::Sailing };
        let Some(p) = sim::pick_cell(&cen, l.index(), Vec2::ZERO, 0.0, &mut rng) else { continue };
        if let Some(s) = sim::spawn(&sea, l, p, &ships, i, &mut rng) {
            if l == Layer::Cargo {
                if (s.pos.y - 500.0).abs() < 1000.0 {
                    lane += 1;
                }
                // Along the lane (east-west), not across it (away from the island).
                if !s.anchored && s.pos.distance(vec2(3000.0, 0.0)) > 2500.0 {
                    let h = s.heading.to_degrees();
                    moving += 1;
                    if (h - 90.0).abs() < 35.0 || (h - 270.0).abs() < 35.0 {
                        along += 1;
                    }
                }
            }
            ships.push(s);
        }
    }
    let cargo = ships.iter().filter(|s| s.layer == Some(Layer::Cargo)).count();
    assert!(cargo > 20, "{cargo}");
    assert!(along as f32 >= moving as f32 * 0.85, "cargo along the lane: {along}/{moving}");
    assert!(lane as f32 > cargo as f32 * 0.9, "cargo in the lane: {lane}/{cargo}");
    for s in &ships {
        assert!(sim::fits(&sea, s.pos, s.heading, s.length, s.beam, s.draught), "{} at {:?} does not fit", s.name, s.pos);
        // Nothing drawing more than the shoal's 2 m on it.
        if s.pos.y < -1700.0 {
            assert!(s.draught * 1.2 + 1.0 <= 2.0, "{} (T {}) on the shoal", s.name, s.draught);
        }
        assert!(sea.water(s.pos) == Some(true));
    }
    // No two hulls on top of each other.
    for (i, a) in ships.iter().enumerate() {
        for b in &ships[i + 1..] {
            assert!(a.pos.distance(b.pos) > (a.length + b.length) * 0.5, "{} / {}", a.name, b.name);
        }
    }
}

#[test]
fn ships_under_way_never_reach_land_or_shoals() {
    let sea = Channel;
    let mut rng = Rng(7);
    let mut ships = Vec::new();
    // Cargo heading straight for the island, and others along the channel.
    for (i, (p, h)) in [(vec2(0.0, 0.0), 90.0f32), (vec2(-4000.0, 500.0), 90.0), (vec2(6000.0, -800.0), 270.0), (vec2(0.0, 1500.0), 60.0)].iter().enumerate() {
        let mut s = Ship::new(i as u64, Class::Cargo, 180.0, 28.0, 11.0, *p, h.to_radians());
        s.layer = Some(Layer::Cargo);
        s.cruise = 7.0;
        s.speed = 7.0;
        ships.push(s);
    }
    let mut t = 0.0f64;
    for _ in 0..(3600 * 2) {
        t += 0.5;
        for s in &mut ships {
            sim::step(&sea, s, t, 0.5, &mut rng);
            assert!(navigable(&sea, s.pos, s.draught), "{} at {:?} (t {t})", s.id, s.pos);
            assert!(!Channel::island(s.pos));
        }
    }
    // They kept moving (not all stuck).
    assert!(ships.iter().filter(|s| s.speed > 3.0).count() >= 3);
}

#[test]
fn lane_axis_finds_the_lane_direction() {
    let (axis, strength) = sim::lane_axis(&Channel, vec2(-5000.0, 500.0), Layer::Cargo.index(), 300.0);
    assert!(strength > 0.5, "{strength}");
    // East-west (90° or 270° folded into 0..180).
    assert!((axis.to_degrees() - 90.0).abs() < 10.0, "{}", axis.to_degrees());
}

#[test]
fn generic_density_is_thin_and_seasonal() {
    let deep = density::generic(Some(60.0), 7);
    let coast = density::generic(Some(8.0), 7);
    let winter = density::generic(Some(8.0), 1);
    assert!(deep[Layer::Cargo.index()] > coast[Layer::Cargo.index()]);
    assert!(coast[Layer::Pleasure.index()] > winter[Layer::Pleasure.index()]);
    // At most ~1 ship per 30 km² anywhere.
    for v in [deep, coast, winter] {
        assert!(v.iter().sum::<f32>() < 0.035);
    }
}

#[test]
fn attitude_is_level_on_a_glassy_sea_and_follows_the_heading() {
    let s = Ship::new(1, Class::Cargo, 150.0, 24.0, 9.0, Vec2::ZERO, 90f32.to_radians());
    let (rot, heave) = super::render::attitude(&s, &crate::water::SeaState::default(), 3.0);
    assert_eq!(heave, 0.0);
    let bow = rot * glam::Vec3::Z;
    assert!((bow - glam::Vec3::X).length() < 1e-4, "heading 090 = east: {bow:?}");
    let up = rot * glam::Vec3::Y;
    assert!((up - glam::Vec3::Y).length() < 1e-4);
    // Port is to the north (-z) when heading east.
    assert!((rot * glam::Vec3::X - glam::Vec3::NEG_Z).length() < 1e-4);
}

/// `cargo test --release dump_hull_bounds -- --ignored --nocapture`: the hulls' body-frame bounds.
#[test]
#[ignore]
fn dump_hull_bounds() {
    for m in Model::IMPORTED {
        let s = models::load(m).unwrap();
        eprintln!("{m:?}: lo {:?} hi {:?}", s.lo, s.hi);
    }
}
