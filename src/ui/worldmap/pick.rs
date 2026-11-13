//! Start-position picking on the airport diagram: a click (lat / lon) and
//! the map scale → the runway end, stand or helipad under it. Distances are
//! measured on the ground (local tangent plane around the airport, metres)
//! so the targets keep their real size at any latitude; small targets get a
//! minimum on-screen radius.

use super::diagram::Diagram;
use crate::airports::{Airport, Rwy};

#[derive(Clone, Debug, PartialEq)]
pub enum Pick {
    RunwayEnd(String),
    Stand { name: String, lat: f64, lon: f64, hdg: f32 },
    Pad { name: String, lat: f64, lon: f64, hdg: f32 },
}

/// WGS-84 metres per degree of latitude / longitude at latitude `lat`.
pub fn m_per_deg(lat: f64) -> (f64, f64) {
    let p = lat.to_radians();
    (111_132.92 - 559.82 * (2.0 * p).cos() + 1.175 * (4.0 * p).cos(), 111_412.84 * p.cos() - 93.5 * (3.0 * p).cos() + 0.118 * (5.0 * p).cos())
}

/// (east, north) metres of `p` from `o` (lat, lon).
pub fn enu(o: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let (mlat, mlon) = m_per_deg((o.0 + p.0) * 0.5);
    let mut dlon = p.1 - o.1;
    dlon -= (dlon / 360.0).round() * 360.0;
    (dlon * mlon, (p.0 - o.0) * mlat)
}

/// The point `e` m east and `n` m north of `o`.
pub fn offset(o: (f64, f64), e: f64, n: f64) -> (f64, f64) {
    let (mlat, mlon) = m_per_deg(o.0);
    (o.0 + n / mlat, o.1 + e / mlon.max(1.0))
}

fn dist(o: (f64, f64), p: (f64, f64)) -> f64 {
    let (e, n) = enu(o, p);
    (e * e + n * n).sqrt()
}

/// Both ends of a runway (lat, lon): surveyed, else laid out from the
/// reference point along its heading.
pub fn runway_ends(a: &Airport, rw: &Rwy) -> Option<[(f64, f64); 2]> {
    if let (Some(la), Some(lo), Some(lb), Some(lob)) = (rw.ends[0].lat, rw.ends[0].lon, rw.ends[1].lat, rw.ends[1].lon) {
        return Some([(la, lo), (lb, lob)]);
    }
    let h = rw.heading(0)?.to_radians() as f64;
    let half = rw.length_m as f64 * 0.5;
    let (dn, de) = (h.cos() * half, h.sin() * half);
    Some([offset((a.lat, a.lon), -de, -dn), offset((a.lat, a.lon), de, dn)])
}

/// What a click at (lat, lon) picks, with the map at `m_per_pt` metres per point.
pub fn pick(a: &Airport, diag: Option<&Diagram>, at: (f64, f64), m_per_pt: f64) -> Option<Pick> {
    let mut best: Option<(f64, Pick)> = None;
    let mut offer = |score: f64, p: Pick| {
        if score <= 1.0 && best.as_ref().is_none_or(|b| score < b.0) {
            best = Some((score, p));
        }
    };
    let pad_hdg = a.runways.iter().filter(|r| !r.ends[0].ident.starts_with('H')).max_by(|x, y| x.length_m.total_cmp(&y.length_m)).and_then(|r| r.heading(0)).unwrap_or(0.0);
    if let Some(d) = diag {
        for s in &d.stands {
            let r = (s.span as f64 * 0.4).max(11.0 * m_per_pt);
            offer(dist((s.lat, s.lon), at) / r, Pick::Stand { name: s.name.clone(), lat: s.lat, lon: s.lon, hdg: s.hdg });
        }
        for p in &d.pads {
            let r = (p.size as f64 * 0.7).max(12.0 * m_per_pt);
            offer(dist((p.lat, p.lon), at) / r, Pick::Pad { name: p.name.clone(), lat: p.lat, lon: p.lon, hdg: pad_hdg });
        }
    }
    // Runways: anywhere on the pavement (or just beyond an end) picks the nearer end.
    for rw in a.runways.iter().filter(|r| !r.ends[0].ident.starts_with('H') && !r.ends[0].ident.is_empty()) {
        let Some([e0, e1]) = runway_ends(a, rw) else { continue };
        let (bx, by) = enu(e0, e1);
        let len = (bx * bx + by * by).sqrt().max(1.0);
        let (ux, uy) = (bx / len, by / len);
        let (px, py) = enu(e0, at);
        let along = px * ux + py * uy;
        let lat_off = (px * uy - py * ux).abs();
        let half_w = rw.width_m.max(18.0) as f64 * 0.5 + 6.0 * m_per_pt;
        let beyond = 40.0 * m_per_pt;
        if along < -beyond || along > len + beyond {
            continue;
        }
        let end = if along < len * 0.5 { 0 } else { 1 };
        // Stands / pads beside a runway win over the runway.
        offer(0.5 + 0.5 * lat_off / half_w, Pick::RunwayEnd(rw.ends[end].ident.clone()));
    }
    best.map(|b| b.1)
}

#[cfg(test)]
mod tests {
    use super::super::diagram::{DPad, DStand};
    use super::*;
    use crate::airports::{End, Kind, Surface};

    fn airport(lat: f64) -> Airport {
        let end = |id: &str, la: f64, lo: f64| End { ident: id.into(), lat: Some(la), lon: Some(lo), hdg_true: None, displaced_m: 0.0 };
        Airport {
            ident: "XTST".into(),
            kind: Kind::Large,
            name: "Test".into(),
            lat,
            lon: 10.0,
            elev_ft: 0.0,
            country: String::new(),
            region: String::new(),
            city: String::new(),
            scheduled: true,
            icao: "XTST".into(),
            iata: String::new(),
            // 3 km east-west runway, centred on the reference point
            runways: vec![Rwy {
                length_m: 3000.0,
                width_m: 45.0,
                surface: Surface::parse("ASP"),
                lighted: true,
                ends: [end("09", lat, offset((lat, 10.0), -1500.0, 0.0).1), end("27", lat, offset((lat, 10.0), 1500.0, 0.0).1)],
            }],
        }
    }

    /// (lat, lon) `e` m east and `n` m north of the reference point.
    fn at(a: &Airport, e: f64, n: f64) -> (f64, f64) {
        offset((a.lat, a.lon), e, n)
    }

    fn diag(a: &Airport) -> Diagram {
        let (sl, so) = at(a, 200.0, 150.0);
        let (pl, po) = at(a, -600.0, 120.0);
        Diagram {
            stands: vec![DStand { name: "23".into(), lat: sl, lon: so, hdg: 180.0, span: 36.0, contact: true }],
            pads: vec![DPad { name: "H1".into(), lat: pl, lon: po, size: 20.0 }],
            ..Default::default()
        }
    }

    #[test]
    fn picks_stand_pad_and_runway_ends() {
        for lat in [51.47, 0.5, -33.9, 69.7] {
            let a = airport(lat);
            let d = diag(&a);
            let mpp = 0.5; // zoomed in: stands are the click targets
            match pick(&a, Some(&d), at(&a, 205.0, 148.0), mpp) {
                Some(Pick::Stand { name, hdg, .. }) => assert!(name == "23" && hdg == 180.0),
                p => panic!("lat {lat}: {p:?}"),
            }
            assert!(matches!(pick(&a, Some(&d), at(&a, -598.0, 125.0), mpp), Some(Pick::Pad { name, .. }) if name == "H1"));
            // on the runway, west half → 09, east half → 27, just off the end still counts
            assert_eq!(pick(&a, Some(&d), at(&a, -1200.0, 10.0), mpp), Some(Pick::RunwayEnd("09".into())));
            assert_eq!(pick(&a, Some(&d), at(&a, 900.0, -15.0), mpp), Some(Pick::RunwayEnd("27".into())));
            assert_eq!(pick(&a, Some(&d), at(&a, 1510.0, 0.0), mpp), Some(Pick::RunwayEnd("27".into())));
            // grass beside the runway, open apron: nothing
            assert_eq!(pick(&a, Some(&d), at(&a, 0.0, 80.0), mpp), None, "lat {lat}");
            assert_eq!(pick(&a, Some(&d), at(&a, 400.0, 300.0), mpp), None);
        }
    }

    #[test]
    fn small_targets_grow_when_zoomed_out() {
        let a = airport(51.47);
        let d = diag(&a);
        // 30 m from the stand: a miss up close, a hit when one point is 4 m
        assert_eq!(pick(&a, Some(&d), at(&a, 230.0, 150.0), 0.3), None);
        assert!(matches!(pick(&a, Some(&d), at(&a, 230.0, 150.0), 4.0), Some(Pick::Stand { .. })));
    }

    #[test]
    fn unsurveyed_runway_laid_out_on_its_heading() {
        let mut a = airport(51.0);
        for e in &mut a.runways[0].ends {
            e.lat = None;
            e.lon = None;
        }
        let [e0, e1] = runway_ends(&a, &a.runways[0]).unwrap();
        let (x, y) = enu(e0, e1);
        assert!((x - 3000.0).abs() < 5.0 && y.abs() < 5.0, "{x} {y}");
        assert_eq!(pick(&a, None, at(&a, -1000.0, 0.0), 1.0), Some(Pick::RunwayEnd("09".into())));
    }
}
