//! Airport diagrams for the world map: the same layout the 3D airport is
//! built from (airport/layout.rs: stands with their numbers and headings,
//! runway-holding points, windsocks; from the airport's z14 map tiles,
//! OurAirports and cached Overpass data) — built headless on a background
//! thread, without the scene — plus helipads (OurAirports "H" runways, the
//! tiles' and Overpass's helipads) and the airside buildings by class.
//! Everything in lat / lon.

use crate::airport::{layout, tile};
use crate::airports::{Airport, Db, Kind};
use crate::osm::{Kind as OKind, Proj, TileId};
use glam::Vec2;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};

#[derive(Clone, Debug, PartialEq)]
pub struct DStand {
    pub name: String,
    /// Nose-wheel stop point.
    pub lat: f64,
    pub lon: f64,
    /// Aircraft heading parked (deg true).
    pub hdg: f32,
    /// Largest wingspan (m).
    pub span: f32,
    /// Contact stand (jet bridge).
    pub contact: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DHold {
    pub lat: f64,
    pub lon: f64,
    /// Toward the runway (deg true).
    pub dir: f32,
    pub width: f32,
    pub taxiway: String,
    pub runway: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DPad {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    /// Side / diameter (m).
    pub size: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Diagram {
    pub ident: String,
    pub stands: Vec<DStand>,
    pub holds: Vec<DHold>,
    pub socks: Vec<(f64, f64)>,
    pub pads: Vec<DPad>,
    /// Airside buildings: class (airport::tile::BClass code), outline, triangles.
    pub buildings: Vec<(u8, Vec<(f64, f64)>, Vec<u32>)>,
    /// Built with Overpass data (real stand numbers) rather than procedurally.
    pub osm: bool,
}

/// Compass heading (deg) of a direction in a world frame (x east, y south).
fn heading(d: Vec2) -> f32 {
    d.x.atan2(-d.y).to_degrees().rem_euclid(360.0)
}

/// The airport's z14 tiles: its runways plus an apron margin by class.
pub fn tiles_for(ap: &Airport) -> Vec<TileId> {
    let (mut s, mut w, mut n, mut e) = (ap.lat, ap.lon, ap.lat, ap.lon);
    for r in &ap.runways {
        for end in &r.ends {
            if let (Some(la), Some(lo)) = (end.lat, end.lon) {
                s = s.min(la);
                n = n.max(la);
                w = w.min(lo);
                e = e.max(lo);
            }
        }
    }
    let m = match ap.kind {
        Kind::Large => 1400.0,
        Kind::Medium => 900.0,
        _ => 500.0,
    };
    let dlat = m / crate::osm::M_PER_DEG_LAT;
    let dlon = dlat / ap.lat.to_radians().cos().max(0.1);
    let z = super::pyramid::MAX_TILE_Z;
    let (x0, y0) = TileId::frac(n + dlat, w - dlon, z);
    let (x1, y1) = TileId::frac(s - dlat, e + dlon, z);
    let mut out = Vec::new();
    for y in y0.floor() as u32..=y1.floor() as u32 {
        for x in x0.floor() as u32..=x1.floor() as u32 {
            out.push(TileId { z, x, y });
        }
    }
    // Keep the nearest 80 (a huge field's far corners can go).
    let (cx, cy) = TileId::frac(ap.lat, ap.lon, z);
    out.sort_by(|a, b| {
        let d = |t: &TileId| (t.x as f64 + 0.5 - cx).powi(2) + (t.y as f64 + 0.5 - cy).powi(2);
        d(a).total_cmp(&d(b))
    });
    out.truncate(80);
    out.sort();
    out
}

/// Tile units (0..1) of a z-tile → lat / lon.
fn tile_ll(id: TileId, p: [f32; 2]) -> (f64, f64) {
    let n = (1u64 << id.z) as f64;
    super::super::overview::unmerc((id.x as f64 + p[0] as f64) / n, (id.y as f64 + p[1] as f64) / n)
}

/// One tile's airport summary (as the scene's tile workers make it) and its helipads.
fn tile_summary(id: TileId, bytes: &[u8]) -> (Option<Arc<tile::AeroTile>>, Vec<DPad>) {
    let Ok(layers) = crate::mvt::decode(bytes) else { return (None, Vec::new()) };
    let mut pads = Vec::new();
    for l in layers.iter().filter(|l| l.name == "aeroway") {
        let e = l.extent as f32;
        for f in &l.features {
            if !matches!(f.str("class"), Some("helipad")) {
                continue;
            }
            let pts: Vec<[f32; 2]> = f.parts.iter().flatten().map(|p| [p[0] / e, p[1] / e]).collect();
            if pts.is_empty() {
                continue;
            }
            let n = pts.len() as f32;
            let c = [pts.iter().map(|p| p[0]).sum::<f32>() / n, pts.iter().map(|p| p[1]).sum::<f32>() / n];
            if !(0.0..1.0).contains(&c[0]) || !(0.0..1.0).contains(&c[1]) {
                continue;
            }
            let (lat, lon) = tile_ll(id, c);
            pads.push(DPad { name: f.str("ref").unwrap_or("").to_string(), lat, lon, size: 20.0 });
        }
    }
    let proj = Proj::tile_frame(id);
    let f = crate::osm::extract(&layers, id, &proj);
    let ctx = tile::Ctx::new(id, &proj, &f);
    if ctx.is_empty() {
        return (None, pads);
    }
    let blds = f
        .areas
        .iter()
        .filter_map(|a| {
            let OKind::Building { height, .. } = a.kind else { return None };
            let c = ctx.classify(a, height)?;
            Some(tile::AeroBuilding { outer: a.outer.clone(), class: c.code(), height, base: 0.0 })
        })
        .collect();
    (ctx.summary(id, &f, blds), pads)
}

/// Build the diagram from the airport's tiles (`fetch(id)`: the decompressed tile).
pub fn build(ap: &Airport, osm: Option<&crate::airport::overpass::OsmAirport>, fetch: &dyn Fn(TileId) -> Option<Vec<u8>>) -> Diagram {
    let mut tiles = Vec::new();
    let mut pads: Vec<DPad> = Vec::new();
    let mut blds = Vec::new();
    for id in tiles_for(ap) {
        let Some(b) = fetch(id) else { continue };
        let (s, p) = tile_summary(id, &b);
        pads.extend(p);
        if let Some(s) = s {
            let pj = Proj::tile_frame(id);
            for b in &s.buildings {
                if s.pieces.iter().any(|pc| pc.owner == ap.ident) && b.outer.len() >= 3 {
                    let data: Vec<f64> = b.outer.iter().flat_map(|p| [p.x as f64, p.y as f64]).collect();
                    let tris = earcutr::earcut(&data, &[], 2).unwrap_or_default();
                    blds.push((b.class, b.outer.iter().map(|p| pj.to_latlon(*p)).collect::<Vec<_>>(), tris.into_iter().map(|i| i as u32).collect()));
                }
            }
            tiles.push((id, s));
        }
    }
    let mut d = Diagram { ident: ap.ident.clone(), osm: osm.is_some(), buildings: blds, ..Default::default() };
    if !tiles.is_empty() {
        let l = layout::build(ap, &tiles, osm);
        let ll = |p: Vec2| l.frame.to_latlon(p);
        d.stands = l
            .stands
            .iter()
            .map(|s| {
                let (lat, lon) = ll(s.stop);
                let d = Vec2::new(s.hdg.sin(), -s.hdg.cos());
                DStand { name: s.name.clone(), lat, lon, hdg: heading(d), span: s.span, contact: s.contact.is_some() }
            })
            .collect();
        d.holds = l
            .holds
            .iter()
            .map(|h| {
                let (lat, lon) = ll(h.at);
                DHold { lat, lon, dir: heading(h.dir), width: h.width, taxiway: h.taxiway.clone(), runway: h.runway.clone() }
            })
            .collect();
        d.socks = l.socks.iter().map(|s| ll(*s)).collect();
    }
    // Helipads: OurAirports "H" runways, Overpass, the tiles (deduplicated).
    for r in &ap.runways {
        let e = &r.ends[0];
        if e.ident.starts_with('H') {
            if let (Some(lat), Some(lon)) = (e.lat, e.lon) {
                pads.insert(0, DPad { name: e.ident.clone(), lat, lon, size: r.length_m.clamp(12.0, 60.0) });
            }
        }
    }
    if let Some(o) = osm {
        for (p, r) in &o.helipads {
            pads.push(DPad { name: r.clone(), lat: p.0, lon: p.1, size: 20.0 });
        }
    }
    let mut out: Vec<DPad> = Vec::new();
    for p in pads {
        if !out.iter().any(|q| crate::airports::distance_km((q.lat, q.lon), (p.lat, p.lon)) < 0.025) {
            out.push(p);
        }
    }
    for (i, p) in out.iter_mut().enumerate() {
        if p.name.is_empty() {
            p.name = format!("H{}", i + 1);
        }
    }
    d.pads = out;
    d
}

enum DState {
    Building,
    Ready(Arc<Diagram>),
}

/// Diagrams by airport, built on demand in the background.
pub struct Diagrams {
    map: HashMap<String, DState>,
    rx: Receiver<(String, Diagram, bool)>,
    tx: Sender<(String, Diagram, bool)>,
}

impl Default for Diagrams {
    fn default() -> Self {
        let (tx, rx) = channel();
        Diagrams { map: HashMap::new(), rx, tx }
    }
}

impl Diagrams {
    fn poll(&mut self) {
        while let Ok((id, d, last)) = self.rx.try_recv() {
            let _ = last;
            self.map.insert(id, DState::Ready(Arc::new(d)));
        }
    }

    /// Diagrams still building.
    pub fn busy(&self) -> usize {
        self.map.values().filter(|s| matches!(s, DState::Building)).count()
    }

    /// The diagram of `ap` once built (starts building it).
    pub fn get(&mut self, ap: &Airport, db: &Arc<Db>) -> Option<Arc<Diagram>> {
        self.poll();
        match self.map.get(&ap.ident) {
            Some(DState::Ready(d)) => return Some(d.clone()),
            Some(DState::Building) => return None,
            None => {}
        }
        self.map.insert(ap.ident.clone(), DState::Building);
        // Airport tiles find their owners through the global airport list.
        if crate::terrain::airport_db().is_none() {
            crate::terrain::set_airport_db(db.clone());
        }
        let ap = Arc::new(ap.clone());
        let tx = self.tx.clone();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let _ = std::thread::Builder::new().name("worldmap-diagram".into()).spawn(move || {
                use crate::airport::overpass;
                let fetch = |id: TileId| crate::tiles::fetch_any(id).ok();
                let cached = if overpass::disabled() { None } else { overpass::cached(&ap.ident) };
                let d = build(&ap, cached.as_deref(), &fetch);
                let refetch = cached.is_none() && !overpass::disabled() && !overpass::failed_recently(&ap.ident) && !matches!(ap.kind, Kind::Seaplane);
                if tx.send((ap.ident.clone(), d, !refetch)).is_err() || !refetch {
                    return;
                }
                // Real stand numbers once Overpass answers (cached for the 3D airport too).
                let bbox = crate::airport::osm_bbox(&ap);
                if let Ok(o) = overpass::fetch_blocking(&ap.ident, bbox, std::time::Duration::from_secs(60)) {
                    let d = build(&ap, Some(&o), &fetch);
                    let _ = tx.send((ap.ident.clone(), d, true));
                }
            });
        }
        #[cfg(target_arch = "wasm32")]
        {
            crate::platform::web::spawn_local(async move {
                let mut got: HashMap<TileId, Vec<u8>> = HashMap::new();
                for id in tiles_for(&ap) {
                    if let Ok(b) = crate::tiles::fetch_any_async(id).await {
                        got.insert(id, b);
                    }
                }
                let fetch = move |id: TileId| got.get(&id).cloned();
                let d = build(&ap, None, &fetch);
                let _ = tx.send((ap.ident.clone(), d, true));
            });
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airports::{End, Rwy, Surface};

    fn field(kind: Kind) -> Airport {
        let end = |id: &str, lat: f64, lon: f64| End { ident: id.into(), lat: Some(lat), lon: Some(lon), hdg_true: None, displaced_m: 0.0 };
        Airport {
            ident: "XTST".into(),
            kind,
            name: "Test".into(),
            lat: 51.47,
            lon: -0.46,
            elev_ft: 80.0,
            country: "GB".into(),
            region: String::new(),
            city: String::new(),
            scheduled: false,
            icao: "XTST".into(),
            iata: String::new(),
            runways: vec![
                Rwy { length_m: 3900.0, width_m: 50.0, surface: Surface::parse("ASP"), lighted: true, ends: [end("09", 51.47, -0.49), end("27", 51.47, -0.43)] },
                Rwy { length_m: 30.0, width_m: 30.0, surface: Surface::parse("ASP"), lighted: false, ends: [end("H1", 51.468, -0.45), End::default()] },
            ],
        }
    }

    #[test]
    fn tiles_cover_runways_with_margin() {
        let t = tiles_for(&field(Kind::Large));
        let (x0, _) = TileId::frac(51.47, -0.49, 14);
        let (x1, _) = TileId::frac(51.47, -0.43, 14);
        assert!(t.iter().any(|t| t.x == x0 as u32) && t.iter().any(|t| t.x == x1 as u32));
        assert!(t.len() >= 6 && t.len() <= 80, "{}", t.len());
        assert!(t.len() > tiles_for(&field(Kind::Small)).len());
    }

    #[test]
    fn helipads_from_ourairports_without_tiles() {
        let d = build(&field(Kind::Medium), None, &|_| None);
        assert_eq!(d.pads.len(), 1);
        assert_eq!(d.pads[0].name, "H1");
        assert!(d.stands.is_empty());
    }

    #[test]
    fn headings_are_compass() {
        assert!((heading(Vec2::new(0.0, -1.0)) - 0.0).abs() < 1e-4);
        assert!((heading(Vec2::new(1.0, 0.0)) - 90.0).abs() < 1e-4);
        assert!((heading(Vec2::new(0.0, 1.0)) - 180.0).abs() < 1e-4);
    }
}
