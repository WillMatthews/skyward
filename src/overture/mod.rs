//! Overture Maps buildings where OpenStreetMap has none: machine-learned
//! footprints (Microsoft, Google), Esri Community Maps and the other sources
//! Overture conflates, with heights / floor counts where given.
//!
//! **Data**: the buildings PMTiles archive Overture publishes with each
//! monthly release (`overturemaps-extras-us-west-2/tiles/<release>/buildings.pmtiles`,
//! z14 MVT, layer `building`, ≈ 180 GB). Releases are kept for 60 days, so the
//! current one is looked up once per run in Overture's STAC catalog. Every
//! source in the theme is ODbL or CC BY 4.0 (see [`CREDIT`]); none is filtered
//! out. Native builds only for now (the bucket sends CORS headers, so the
//! browser build could follow).
//!
//! **Flow**: a detail-tile worker ([`crate::tiles`]) calls [`add_to_tile`]
//! after decoding the OpenFreeMap tile. The Overture tile is read with HTTP
//! range requests (header + root directory once per run, a leaf directory per
//! ~4000 tiles, then the tile), reduced to [`Footprint`]s and cached on disk
//! under `cache/overture/` (the reduced form: a few percent of the raw tile).
//! Overture's OSM-sourced buildings are dropped there (OpenFreeMap has them,
//! from a newer planet); [`merge`] then adds the footprints that overlap no
//! OSM building: OSM always wins.
//!
//! **Degrades**: offline with nothing cached, no Overture tile (ocean),
//! network errors (one-minute back-off), the browser build, unit tests, or
//! `SKYWARD_NO_OVERTURE=1` → the tile builds exactly as before.

pub mod pmtiles;

use crate::mvt::{GeomType, Layer};
use crate::osm::{Kind, Proj, TileFeatures, TileId};
use glam::Vec2;

/// Off in tests, the browser, and with `SKYWARD_NO_OVERTURE`.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| !cfg!(test) && !cfg!(target_arch = "wasm32") && std::env::var_os("SKYWARD_NO_OVERTURE").is_none());
    *ON
}

/// Attribution (Credits screen; also README, NOTICE, web/credits.html).
pub const CREDIT: &str = "Overture Maps Foundation (overturemaps.org) — buildings: © OpenStreetMap contributors (ODbL); Global ML Building Footprints, licensed by Microsoft under ODbL; Google Open Buildings (CC BY 4.0); Esri Community Maps contributors (CC BY 4.0); and the other sources Overture lists (desktop version)";

/// At most this many Overture buildings per tile (largest kept): dense
/// informal settlements reach 8000 footprints a tile.
pub const MAX_PER_TILE: usize = 6000;

/// Where a footprint's geometry came from (Overture `@geometry_source`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Osm,
    Microsoft,
    Google,
    Esri,
    Other,
}

impl Source {
    pub const ALL: [Source; 5] = [Source::Osm, Source::Microsoft, Source::Google, Source::Esri, Source::Other];

    pub fn parse(s: &str) -> Source {
        match s {
            "OpenStreetMap" => Source::Osm,
            s if s.starts_with("Microsoft") => Source::Microsoft,
            s if s.starts_with("Google") => Source::Google,
            s if s.starts_with("Esri") => Source::Esri,
            _ => Source::Other,
        }
    }
}

crate::wire_enum!(Source, Source::ALL);

/// One building footprint in tile units (`extent` per side).
#[derive(Clone, Debug, PartialEq)]
pub struct Footprint {
    pub source: Source,
    /// Metres; 0 = not given.
    pub height: f32,
    pub min_height: f32,
    /// Floors above ground; 0 = not given.
    pub floors: f32,
    pub outer: Vec<[f32; 2]>,
    pub holes: Vec<Vec<[f32; 2]>>,
}

crate::wire_struct!(Footprint { source, height, min_height, floors, outer, holes });

/// A tile's footprints.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TileFootprints {
    pub extent: u32,
    pub items: Vec<Footprint>,
}

crate::wire_struct!(TileFootprints { extent, items });

/// The footprints in a decoded Overture buildings tile: above-ground
/// buildings from every source but OSM (OpenFreeMap has those, and the
/// `building_part`s, which are all OSM's).
pub fn footprints(layers: &[Layer]) -> TileFootprints {
    let Some(l) = layers.iter().find(|l| l.name == "building") else { return TileFootprints { extent: 4096, items: Vec::new() } };
    let mut items = Vec::new();
    for f in &l.features {
        if f.geom_type != GeomType::Polygon || f.num("is_underground").unwrap_or(0.0) > 0.5 {
            continue;
        }
        let source = Source::parse(f.str("@geometry_source").unwrap_or(""));
        if source == Source::Osm {
            continue;
        }
        let pos = |k: &str| f.num(k).filter(|v| v.is_finite() && *v > 0.0).unwrap_or(0.0) as f32;
        let (height, min_height, floors) = (pos("height"), pos("min_height"), pos("num_floors"));
        for (outer, holes) in crate::mvt::polygons(&f.parts) {
            items.push(Footprint { source, height, min_height, floors, outer, holes });
        }
    }
    TileFootprints { extent: l.extent, items }
}

/// Building height: Overture's height, else floors × 3 m, else the
/// procedural guess from the footprint area (as for OSM buildings without one).
pub fn height(fp: &Footprint, area_m2: f32, key: u64) -> f32 {
    if fp.height > 1.0 {
        fp.height.min(500.0)
    } else if fp.floors >= 1.0 {
        (fp.floors * 3.0).min(500.0)
    } else {
        crate::osm::synth_height(area_m2, key)
    }
}

fn bbox(r: &[Vec2]) -> (Vec2, Vec2) {
    r.iter().fold((Vec2::splat(f32::MAX), Vec2::splat(f32::MIN)), |(a, b), p| (a.min(*p), b.max(*p)))
}

fn area(r: &[Vec2]) -> f32 {
    let n = r.len();
    (0..n).map(|i| r[i].perp_dot(r[(i + 1) % n])).sum::<f32>().abs() * 0.5
}

fn segments_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (r, s) = (b - a, d - c);
    let den = r.perp_dot(s);
    if den.abs() < 1e-9 {
        return false;
    }
    let t = (c - a).perp_dot(s) / den;
    let u = (c - a).perp_dot(r) / den;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Do two footprints (rings, world space) overlap?
pub fn overlaps(a: &[Vec2], b: &[Vec2]) -> bool {
    use crate::osm::point_in_ring;
    let (na, nb) = (a.len(), b.len());
    if na < 3 || nb < 3 {
        return false;
    }
    let ca = a.iter().copied().sum::<Vec2>() / na as f32;
    let cb = b.iter().copied().sum::<Vec2>() / nb as f32;
    point_in_ring(ca, b)
        || point_in_ring(cb, a)
        || a.iter().any(|p| point_in_ring(*p, b))
        || b.iter().any(|p| point_in_ring(*p, a))
        || (0..na).any(|i| (0..nb).any(|j| segments_cross(a[i], a[(i + 1) % na], b[j], b[(j + 1) % nb])))
}

/// What [`merge`] did with a tile.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// OSM (OpenFreeMap) buildings already in the tile.
    pub osm: usize,
    /// Overture footprints whose box touches the tile.
    pub candidates: usize,
    /// Dropped: overlapping an OSM building.
    pub duplicates: usize,
    /// Dropped: over [`MAX_PER_TILE`].
    pub capped: usize,
    /// Added, per [`Source::ALL`].
    pub added: [usize; 5],
}

impl Stats {
    pub fn added_total(&self) -> usize {
        self.added.iter().sum()
    }
}

/// Add the footprints that overlap no OSM building to the tile's features
/// (built in `proj`, the tile frame).
pub fn merge(out: &mut TileFeatures, id: TileId, proj: &Proj, t: &TileFootprints) -> Stats {
    let mut st = Stats::default();
    let e = t.extent.max(1) as f32;
    let (lo, hi) = id.rect(proj);
    // OSM buildings in a bucket grid.
    const N: usize = 32;
    let cell = (hi - lo) / N as f32;
    let cell_of = |p: Vec2| {
        let c = ((p - lo) / cell).floor();
        (c.x.clamp(0.0, N as f32 - 1.0) as usize, c.y.clamp(0.0, N as f32 - 1.0) as usize)
    };
    let mut grid: Vec<Vec<u32>> = vec![Vec::new(); N * N];
    let osm: Vec<(&[Vec2], (Vec2, Vec2))> =
        out.areas.iter().filter(|a| matches!(a.kind, Kind::Building { .. }) && a.outer.len() >= 3).map(|a| (&a.outer[..], bbox(&a.outer))).collect();
    st.osm = osm.len();
    for (i, (_, (a, b))) in osm.iter().enumerate() {
        let ((x0, z0), (x1, z1)) = (cell_of(*a), cell_of(*b));
        for z in z0..=z1 {
            for x in x0..=x1 {
                grid[z * N + x].push(i as u32);
            }
        }
    }
    // Candidates: (area m², index).
    let mut keep: Vec<(f32, usize)> = Vec::new();
    for (k, fp) in t.items.iter().enumerate() {
        if fp.outer.len() < 3 {
            continue;
        }
        let ring: Vec<Vec2> = fp.outer.iter().map(|p| crate::osm::tile_point(id, proj, e, *p)).collect();
        let (a, b) = bbox(&ring);
        if b.x < lo.x || b.y < lo.y || a.x > hi.x || a.y > hi.y {
            continue; // only in the tile's buffer
        }
        st.candidates += 1;
        let ((x0, z0), (x1, z1)) = (cell_of(a), cell_of(b));
        let mut dup = false;
        'cells: for z in z0..=z1 {
            for x in x0..=x1 {
                for &i in &grid[z * N + x] {
                    let (r, (oa, ob)) = osm[i as usize];
                    if oa.x <= b.x && ob.x >= a.x && oa.y <= b.y && ob.y >= a.y && overlaps(&ring, r) {
                        dup = true;
                        break 'cells;
                    }
                }
            }
        }
        if dup {
            st.duplicates += 1;
        } else {
            keep.push((area(&ring), k));
        }
    }
    if keep.len() > MAX_PER_TILE {
        keep.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        st.capped = keep.len() - MAX_PER_TILE;
        keep.truncate(MAX_PER_TILE);
        keep.sort_by_key(|k| k.1);
    }
    for (area_m2, k) in keep {
        let fp = &t.items[k];
        // Stable key: bbox centre in global tile units (as osm.rs), salted.
        let (a, b) = fp.outer.iter().fold(([f32::MAX; 2], [f32::MIN; 2]), |(a, b), p| ([a[0].min(p[0]), a[1].min(p[1])], [b[0].max(p[0]), b[1].max(p[1])]));
        let gx = id.x as i64 * t.extent as i64 + ((a[0] + b[0]) * 0.5) as i64;
        let gy = id.y as i64 * t.extent as i64 + ((a[1] + b[1]) * 0.5) as i64;
        let key = crate::osm::hash(((gx as u64) << 32 ^ gy as u64) ^ 0x6f76_6572_7475_7265);
        let h = height(fp, area_m2, key);
        let min_h = fp.min_height.clamp(0.0, (h - 1.0).max(0.0));
        let before = out.areas.len();
        crate::osm::push_building(out, id, proj, e, &fp.outer, &fp.holes, h, min_h, key);
        if out.areas.len() > before {
            let s = Source::ALL.iter().position(|s| *s == fp.source).unwrap_or(4);
            st.added[s] += 1;
        }
    }
    st
}

/// Fill a detail tile's OSM building gaps from Overture (no-op when
/// disabled, offline with nothing cached, or on any failure).
pub fn add_to_tile(out: &mut TileFeatures, id: TileId, proj: &Proj) {
    if !enabled() || id.z != crate::osm::ZOOM {
        return;
    }
    let t0 = web_time::Instant::now();
    #[cfg(not(target_arch = "wasm32"))]
    let t = net::for_tile(id);
    #[cfg(target_arch = "wasm32")]
    let t: Option<TileFootprints> = None;
    let Some(t) = t else { return };
    let st = merge(out, id, proj, &t);
    if std::env::var_os("SKYWARD_OVERTURE_DEBUG").is_some() {
        let a = st.added;
        let ms = t0.elapsed().as_secs_f32() * 1e3;
        eprintln!(
            "overture {}/{}/{}: osm {}, candidates {}, dup {}, capped {}, added {} (osm {}, microsoft {}, google {}, esri {}, other {}), {ms:.0} ms",
            id.z,
            id.x,
            id.y,
            st.osm,
            st.candidates,
            st.duplicates,
            st.capped,
            st.added_total(),
            a[0],
            a[1],
            a[2],
            a[3],
            a[4]
        );
    }
}

// ---------------------------------------------------------------- downloads

/// Release lookup, range reads, directory + disk caches (native).
#[cfg(not(target_arch = "wasm32"))]
mod net {
    use super::pmtiles::{self, Entry};
    use super::{TileFootprints, footprints};
    use crate::osm::TileId;
    use std::io::{Read, Write};
    use std::path::PathBuf;
    use std::sync::{Arc, LazyLock, Mutex};
    use web_time::{Duration, Instant};

    const CATALOG: &str = "https://stac.overturemaps.org/catalog.json";
    const ARCHIVE_URL: &str = "https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com/tiles/{release}/buildings.pmtiles";
    const USER_AGENT: &str = "skyward-flightsim/0.2";
    /// Disk budget for `cache/overture/` (oldest pruned at startup).
    const CACHE_LIMIT: u64 = 600_000_000;
    /// Reduced-tile format version (in the cache path).
    const FORMAT: u32 = 1;
    const BACKOFF: Duration = Duration::from_secs(60);

    static FAILED_AT: Mutex<Option<Instant>> = Mutex::new(None);
    static AGENT: LazyLock<ureq::Agent> = LazyLock::new(|| {
        std::thread::spawn(|| crate::landcover::net::prune(&root(), CACHE_LIMIT));
        ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(8)).timeout(Duration::from_secs(40)).user_agent(USER_AGENT).build()
    });

    pub fn root() -> PathBuf {
        PathBuf::from("cache").join("overture")
    }

    fn tile_path(id: TileId) -> PathBuf {
        root().join(format!("tiles-v{FORMAT}")).join(id.z.to_string()).join(id.x.to_string()).join(format!("{}.bin", id.y))
    }

    /// The archive moved on (release expired): look the release up again.
    const GONE: &str = "overture: archive gone";

    /// GET `url` (whole, or `range` = (start, len)), at most `cap` bytes.
    fn get(url: &str, range: Option<(u64, u64)>, cap: u64) -> Result<Vec<u8>, String> {
        if crate::platform::offline() {
            return Err("offline mode".into());
        }
        if FAILED_AT.lock().unwrap().is_some_and(|t| t.elapsed() < BACKOFF) {
            return Err("network unavailable (backing off)".into());
        }
        let mut req = AGENT.get(url);
        if let Some((start, len)) = range {
            if len == 0 {
                return Ok(Vec::new());
            }
            req = req.set("Range", &format!("bytes={}-{}", start, start + len - 1));
        }
        match req.call() {
            Ok(r) => {
                if range.is_some_and(|r| r.0 > 0) && r.status() == 200 {
                    return Err("server ignored the range request".into());
                }
                let mut b = Vec::new();
                r.into_reader().take(cap).read_to_end(&mut b).map_err(|e| e.to_string())?;
                Ok(b)
            }
            Err(ureq::Error::Status(403 | 404, _)) => Err(GONE.into()),
            Err(e @ ureq::Error::Transport(_)) => {
                *FAILED_AT.lock().unwrap() = Some(Instant::now());
                Err(e.to_string())
            }
            Err(e) => Err(e.to_string()),
        }
    }

    pub struct Archive {
        release: String,
        url: String,
        header: pmtiles::Header,
        root: Vec<Entry>,
    }

    /// The current release's archive (header + root directory), loaded once;
    /// a failure is retried after a minute.
    static ARCHIVE: Mutex<Option<(Instant, Result<Arc<Archive>, String>)>> = Mutex::new(None);

    fn archive() -> Result<Arc<Archive>, String> {
        let mut g = ARCHIVE.lock().unwrap();
        if let Some((t, r)) = &*g {
            if r.is_ok() || t.elapsed() < BACKOFF {
                return r.clone();
            }
        }
        let r = load_archive().map(Arc::new);
        *g = Some((Instant::now(), r.clone()));
        r
    }

    fn forget_archive() {
        *ARCHIVE.lock().unwrap() = None;
    }

    /// A release name as the catalog gives it (`2026-10-21.0`): only digits,
    /// dots and dashes go into the URL.
    pub fn valid_release(s: &str) -> bool {
        !s.is_empty() && s.len() <= 32 && s.bytes().all(|c| c.is_ascii_digit() || c == b'.' || c == b'-')
    }

    fn load_archive() -> Result<Archive, String> {
        let cat = get(CATALOG, None, 4 << 20)?;
        let v: serde_json::Value = serde_json::from_slice(&cat).map_err(|e| format!("overture catalog: {e}"))?;
        let release = v.get("latest").and_then(|x| x.as_str()).filter(|s| valid_release(s)).ok_or("overture catalog: no latest release")?.to_string();
        let url = ARCHIVE_URL.replace("{release}", &release);
        let head = get(&url, Some((0, pmtiles::HEAD_LEN)), pmtiles::HEAD_LEN)?;
        let header = pmtiles::header(&head)?;
        if header.tile_type != 1 {
            return Err("overture: archive is not MVT".into());
        }
        let (ro, rl) = header.root;
        let raw = match head.get(ro as usize..(ro + rl) as usize) {
            Some(b) => b.to_vec(),
            None => get(&url, Some((ro, rl)), 1 << 24)?,
        };
        let root = pmtiles::directory(&pmtiles::inflate(&raw, header.internal)?)?;
        Ok(Archive { release, url, header, root })
    }

    /// Leaf directories, keyed by (release, offset, length).
    static LEAVES: LazyLock<crate::landcover::Store<(String, u64, u32), Vec<Entry>>> = LazyLock::new(|| crate::landcover::Store::named(256, "overture"));

    fn leaf(a: &Archive, e: Entry) -> Result<Arc<Vec<Entry>>, String> {
        let mut err = None;
        let v = LEAVES.get(&(a.release.clone(), e.offset, e.length), || {
            let path = root().join("leaves").join(&a.release).join(format!("{}-{}.bin", e.offset, e.length));
            let raw = match std::fs::read(&path) {
                Ok(b) if b.len() == e.length as usize => b,
                _ => {
                    let b = get(&a.url, Some((a.header.leaves_offset + e.offset, e.length as u64)), e.length as u64).inspect_err(|x| err = Some(x.clone()))?;
                    crate::landcover::net::write(&path, &b);
                    b
                }
            };
            pmtiles::directory(&pmtiles::inflate(&raw, a.header.internal)?).map(Some)
        });
        v.ok_or_else(|| err.unwrap_or_else(|| "overture: leaf directory unavailable".into()))
    }

    /// The raw (decompressed) MVT of a tile; `Ok(None)` = no tile there.
    fn fetch_tile(id: TileId) -> Result<Option<Vec<u8>>, String> {
        let a = archive()?;
        let tid = pmtiles::tile_id(id.z as u8, id.x, id.y);
        let mut dir: Arc<Vec<Entry>> = Arc::new(a.root.clone());
        for _ in 0..4 {
            let Some(e) = pmtiles::find(&dir, tid) else { return Ok(None) };
            if e.run_length == 0 {
                dir = leaf(&a, e)?;
                continue;
            }
            let b = get(&a.url, Some((a.header.data_offset + e.offset, e.length as u64)), e.length as u64)?;
            return pmtiles::inflate(&b, a.header.tile).map(Some);
        }
        Err("overture: directory too deep".into())
    }

    fn encode(t: &TileFootprints) -> Vec<u8> {
        let mut g = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        let _ = g.write_all(&crate::platform::wire::encode(t));
        g.finish().unwrap_or_default()
    }

    fn decode(b: &[u8]) -> Option<TileFootprints> {
        let raw = pmtiles::inflate(b, pmtiles::Compression::Gzip).ok()?;
        crate::platform::wire::decode(&raw).ok()
    }

    /// A tile's footprints: the disk cache, else the archive (blocking: tile
    /// workers only). `None` when unavailable (see the module docs).
    pub fn for_tile(id: TileId) -> Option<TileFootprints> {
        let path = tile_path(id);
        if let Some(t) = std::fs::read(&path).ok().and_then(|b| decode(&b)) {
            return Some(t);
        }
        if crate::platform::offline() {
            return None;
        }
        let r = fetch_tile(id).and_then(|b| match b {
            Some(b) => crate::mvt::decode(&b).map(|l| footprints(&l)),
            None => Ok(TileFootprints { extent: 4096, items: Vec::new() }),
        });
        match r {
            Ok(t) => {
                crate::landcover::net::write(&path, &encode(&t));
                Some(t)
            }
            Err(e) => {
                if e == GONE {
                    forget_archive();
                }
                static WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                if !WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                    eprintln!("overture: {e} (buildings from OSM only)");
                }
                None
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn release_names_are_checked() {
            assert!(valid_release("2026-10-21.0"));
            assert!(!valid_release(""));
            assert!(!valid_release("../../x"));
            assert!(!valid_release("2026-10-21.0/../a"));
        }

        #[test]
        fn reduced_tiles_round_trip() {
            let t = TileFootprints {
                extent: 4096,
                items: vec![super::super::Footprint {
                    source: super::super::Source::Google,
                    height: 0.0,
                    min_height: 0.0,
                    floors: 2.0,
                    outer: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
                    holes: vec![],
                }],
            };
            assert_eq!(decode(&encode(&t)), Some(t));
            assert_eq!(decode(b"junk"), None);
        }

        /// Live data (network; `cargo test --release -- --ignored live_`): a
        /// rural Kenyan tile (Kakamega) has thousands of footprints, mostly ML.
        #[test]
        #[ignore]
        fn live_kenya_tile() {
            let b = fetch_tile(TileId { z: 14, x: 9773, y: 8179 }).unwrap().unwrap();
            let t = footprints(&crate::mvt::decode(&b).unwrap());
            assert!(t.items.len() > 3000, "{}", t.items.len());
            assert!(t.items.iter().any(|f| f.source == super::super::Source::Google));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec2;

    /// Minimal MVT writer for fixtures: one layer of polygons with tags.
    pub fn mvt(layer: &str, feats: &[(Vec<(&str, MvtVal)>, Vec<Vec<[i32; 2]>>)]) -> Vec<u8> {
        fn varint(o: &mut Vec<u8>, mut v: u64) {
            while v >= 0x80 {
                o.push((v as u8 & 0x7f) | 0x80);
                v >>= 7;
            }
            o.push(v as u8);
        }
        fn field(o: &mut Vec<u8>, f: u64, b: &[u8]) {
            varint(o, f << 3 | 2);
            varint(o, b.len() as u64);
            o.extend_from_slice(b);
        }
        let zz = |v: i32| ((v << 1) ^ (v >> 31)) as u32 as u64;
        let mut keys: Vec<&str> = Vec::new();
        let mut vals: Vec<Vec<u8>> = Vec::new();
        let mut l = Vec::new();
        field(&mut l, 1, layer.as_bytes());
        for (tags, rings) in feats {
            let mut f = Vec::new();
            varint(&mut f, 3 << 3);
            varint(&mut f, 3); // polygon
            let mut t = Vec::new();
            for (k, v) in tags {
                let ki = keys.iter().position(|x| x == k).unwrap_or_else(|| {
                    keys.push(k);
                    keys.len() - 1
                });
                let mut vb = Vec::new();
                match v {
                    MvtVal::S(s) => field(&mut vb, 1, s.as_bytes()),
                    MvtVal::D(d) => {
                        varint(&mut vb, 3 << 3 | 1);
                        vb.extend_from_slice(&d.to_le_bytes());
                    }
                    MvtVal::B(b) => {
                        varint(&mut vb, 7 << 3);
                        varint(&mut vb, *b as u64);
                    }
                }
                vals.push(vb);
                varint(&mut t, ki as u64);
                varint(&mut t, vals.len() as u64 - 1);
            }
            field(&mut f, 2, &t);
            let mut g = Vec::new();
            let (mut cx, mut cy) = (0, 0);
            for r in rings {
                varint(&mut g, 1 | 1 << 3);
                varint(&mut g, zz(r[0][0] - cx));
                varint(&mut g, zz(r[0][1] - cy));
                (cx, cy) = (r[0][0], r[0][1]);
                varint(&mut g, 2 | ((r.len() as u64 - 1) << 3));
                for p in &r[1..] {
                    varint(&mut g, zz(p[0] - cx));
                    varint(&mut g, zz(p[1] - cy));
                    (cx, cy) = (p[0], p[1]);
                }
                varint(&mut g, 7 | 1 << 3);
            }
            field(&mut f, 4, &g);
            field(&mut l, 2, &f);
        }
        for k in &keys {
            field(&mut l, 3, k.as_bytes());
        }
        for v in &vals {
            field(&mut l, 4, v);
        }
        varint(&mut l, 5 << 3);
        varint(&mut l, 4096);
        let mut tile = Vec::new();
        field(&mut tile, 3, &l);
        tile
    }

    pub enum MvtVal {
        S(&'static str),
        D(f64),
        B(bool),
    }
    use MvtVal::*;

    /// Clockwise in tile units (y down) = exterior per the MVT spec.
    fn square(x: i32, y: i32, s: i32) -> Vec<[i32; 2]> {
        vec![[x, y], [x + s, y], [x + s, y + s], [x, y + s]]
    }

    fn fixture() -> Vec<u8> {
        mvt(
            "building",
            &[
                (vec![("@geometry_source", S("OpenStreetMap")), ("is_underground", B(false))], vec![square(100, 100, 40)]),
                (vec![("@geometry_source", S("Microsoft ML Buildings")), ("height", D(12.5))], vec![square(1000, 1000, 30)]),
                (vec![("@geometry_source", S("Google Open Buildings")), ("num_floors", D(4.0))], vec![square(2000, 2000, 20)]),
                (vec![("@geometry_source", S("Esri Community Maps")), ("is_underground", B(true))], vec![square(3000, 3000, 20)]),
                // A courtyard block: a hole (anticlockwise).
                (vec![("@geometry_source", S("Google Open Buildings"))], vec![square(500, 2500, 100), vec![[520, 2520], [520, 2580], [580, 2580], [580, 2520]]]),
            ],
        )
    }

    #[test]
    fn decodes_fixture_from_a_pmtiles_archive() {
        let a = pmtiles::tests::archive(&[((14, 8186, 5448), fixture())]);
        let h = pmtiles::header(&a).unwrap();
        let root = pmtiles::directory(&pmtiles::inflate(&a[h.root.0 as usize..(h.root.0 + h.root.1) as usize], h.internal).unwrap()).unwrap();
        let e = pmtiles::find(&root, pmtiles::tile_id(14, 8186, 5448)).unwrap();
        let o = (h.data_offset + e.offset) as usize;
        let t = footprints(&crate::mvt::decode(&pmtiles::inflate(&a[o..o + e.length as usize], h.tile).unwrap()).unwrap());
        assert_eq!(t.extent, 4096);
        // The OSM-sourced and the underground ones are dropped.
        assert_eq!(t.items.len(), 3);
        assert_eq!(t.items.iter().map(|f| f.source).collect::<Vec<_>>(), [Source::Microsoft, Source::Google, Source::Google]);
        assert_eq!(t.items[0].height, 12.5);
        assert_eq!(t.items[1].floors, 4.0);
        assert_eq!(t.items[0].outer.len(), 4);
        assert_eq!(t.items[2].holes.len(), 1);
    }

    #[test]
    fn height_heuristic() {
        let mut fp = Footprint { source: Source::Google, height: 0.0, min_height: 0.0, floors: 0.0, outer: vec![], holes: vec![] };
        // Nothing given: the OSM procedural guess (small hut → 2.6–3.6 m).
        let h = height(&fp, 25.0, 7);
        assert!((2.6..=3.6).contains(&h), "{h}");
        assert_eq!(h, crate::osm::synth_height(25.0, 7));
        fp.floors = 5.0;
        assert_eq!(height(&fp, 25.0, 7), 15.0);
        fp.height = 31.0;
        assert_eq!(height(&fp, 25.0, 7), 31.0);
        fp.height = 9000.0;
        assert_eq!(height(&fp, 25.0, 7), 500.0);
    }

    #[test]
    fn overlap_tests() {
        let sq = |x: f32, y: f32, s: f32| vec![vec2(x, y), vec2(x + s, y), vec2(x + s, y + s), vec2(x, y + s)];
        assert!(overlaps(&sq(0.0, 0.0, 10.0), &sq(5.0, 5.0, 10.0)));
        assert!(overlaps(&sq(0.0, 0.0, 10.0), &sq(2.0, 2.0, 3.0))); // contained
        assert!(overlaps(&sq(2.0, 2.0, 3.0), &sq(0.0, 0.0, 10.0))); // contains
        assert!(overlaps(&sq(0.0, 0.0, 10.0), &sq(0.0, 0.0, 10.0))); // identical
        // A plus: no vertex inside the other, edges cross.
        let wide = vec![vec2(-10.0, -1.0), vec2(10.0, -1.0), vec2(10.0, 1.0), vec2(-10.0, 1.0)];
        let tall = vec![vec2(-1.0, -10.0), vec2(1.0, -10.0), vec2(1.0, 10.0), vec2(-1.0, 10.0)];
        assert!(overlaps(&wide, &tall));
        assert!(!overlaps(&sq(0.0, 0.0, 10.0), &sq(11.0, 0.0, 10.0)));
    }

    #[test]
    fn merge_keeps_osm_and_fills_gaps() {
        let id = TileId { z: 14, x: 8186, y: 5448 };
        let proj = Proj::tile_frame(id);
        let t = footprints(&crate::mvt::decode(&fixture()).unwrap());
        // OSM has one building, overlapping the 4-floor footprint.
        let osm =
            |f: &mut TileFeatures| crate::osm::push_building(f, id, &proj, 4096.0, &[[2010.0, 2010.0], [2050.0, 2010.0], [2050.0, 2050.0], [2010.0, 2050.0]], &[], 9.0, 0.0, 1);
        let mut f = TileFeatures::default();
        osm(&mut f);
        let st = merge(&mut f, id, &proj, &t);
        assert_eq!((st.osm, st.candidates, st.duplicates), (1, 3, 1));
        assert_eq!(st.added, [0, 1, 1, 0, 0]);
        let blds: Vec<_> = f.areas.iter().filter_map(|a| if let Kind::Building { height, .. } = a.kind { Some(height) } else { None }).collect();
        // OSM's own building is untouched; the ML one carries its height.
        assert_eq!(blds.len(), 3);
        assert_eq!(blds[..2], [9.0, 12.5]);
        assert_eq!(f.areas[2].holes.len(), 1);
        // Deterministic: same input, same keys.
        let mut g = TileFeatures::default();
        osm(&mut g);
        merge(&mut g, id, &proj, &t);
        assert_eq!(f.areas.iter().map(|a| a.key).collect::<Vec<_>>(), g.areas.iter().map(|a| a.key).collect::<Vec<_>>());
    }

    #[test]
    fn merge_caps_per_tile_keeping_the_largest() {
        let id = TileId { z: 14, x: 8186, y: 5448 };
        let proj = Proj::tile_frame(id);
        let n = MAX_PER_TILE + 10;
        let items = (0..n)
            .map(|i| {
                let (x, y) = ((i % 90) as f32 * 45.0, (i / 90) as f32 * 45.0);
                let s = if i < 10 { 2.0 } else { 20.0 };
                Footprint { source: Source::Microsoft, height: 0.0, min_height: 0.0, floors: 0.0, outer: vec![[x, y], [x + s, y], [x + s, y + s], [x, y + s]], holes: vec![] }
            })
            .collect();
        let mut f = TileFeatures::default();
        let st = merge(&mut f, id, &proj, &TileFootprints { extent: 4096, items });
        assert_eq!(st.capped, 10);
        assert_eq!(st.added_total(), MAX_PER_TILE);
        // The ten tiny ones went.
        assert!(f.areas.iter().all(|a| {
            let (lo, hi) = bbox(&a.outer);
            (hi - lo).x > 5.0
        }));
    }

    #[test]
    fn source_names() {
        assert_eq!(Source::parse("OpenStreetMap"), Source::Osm);
        assert_eq!(Source::parse("Microsoft ML Buildings"), Source::Microsoft);
        assert_eq!(Source::parse("Google Open Buildings"), Source::Google);
        assert_eq!(Source::parse("Esri Community Maps"), Source::Esri);
        assert_eq!(Source::parse("Instituto Geográfico Nacional (España)"), Source::Other);
    }
}
