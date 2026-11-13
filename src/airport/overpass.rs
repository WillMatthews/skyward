//! Airport detail from OpenStreetMap via the Overpass API (stands /
//! parking positions, windsocks, holding positions, control towers) — what
//! the OpenMapTiles vector tiles leave out. Optional: the layout falls back
//! to procedural placement when Overpass is slow, down or the airport is
//! sparsely mapped.
//!
//! Overpass is flaky: requests go to several public instances in turn with
//! exponential backoff, a generic User-Agent and no personal data; answers
//! are cached for good under `cache/airports/` (natively; the browser build
//! asks every session — the instances send CORS headers).
//! Data © OpenStreetMap contributors (ODbL).

use std::sync::Arc;

/// Overpass instances, tried in turn.
const MIRRORS: [&str; 3] =
    ["https://overpass-api.de/api/interpreter", "https://overpass.private.coffee/api/interpreter", "https://maps.mail.ru/osm/tools/overpass/api/interpreter"];
const USER_AGENT: &str = "skyward-flightsim/0.3";

/// The airport features we use (lat / lon).
#[derive(Clone, Debug, Default)]
pub struct OsmAirport {
    /// Parking positions: a point, or a line (stop point at one end), and the ref.
    pub stands: Vec<(Vec<(f64, f64)>, String)>,
    pub windsocks: Vec<(f64, f64)>,
    pub holds: Vec<(f64, f64)>,
    /// Control towers: position and height (m) if tagged.
    pub towers: Vec<((f64, f64), Option<f32>)>,
}

/// The Overpass QL query for a lat / lon box (south, west, north, east).
pub fn query(b: (f64, f64, f64, f64)) -> String {
    let bb = format!("({:.5},{:.5},{:.5},{:.5})", b.0, b.1, b.2, b.3);
    format!(
        "[out:json][timeout:60];(nwr[\"aeroway\"~\"^(parking_position|windsock|holding_position|control_tower|tower)$\"]{bb};nwr[\"man_made\"=\"tower\"][\"tower:type\"~\"^(observation|aircraft_control|communication)$\"]{bb};nwr[\"building\"=\"control_tower\"]{bb};nwr[\"service\"=\"aircraft_control\"]{bb};);out geom;"
    )
}

/// Percent-encode a query-string value.
pub fn encode(s: &str) -> String {
    let mut o = String::with_capacity(s.len() * 3);
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || b"-_.~".contains(&b) {
            o.push(b as char);
        } else {
            o.push_str(&format!("%{b:02X}"));
        }
    }
    o
}

fn centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len().max(1) as f64;
    (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n)
}

/// Parse an Overpass JSON answer.
pub fn parse(text: &str) -> Result<OsmAirport, String> {
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("overpass json: {e}"))?;
    let els = v.get("elements").and_then(|e| e.as_array()).ok_or("overpass: no elements")?;
    let mut out = OsmAirport::default();
    for e in els {
        let tags = e.get("tags");
        let tag = |k: &str| tags.and_then(|t| t.get(k)).and_then(|x| x.as_str()).unwrap_or("");
        let pts: Vec<(f64, f64)> = match e.get("type").and_then(|t| t.as_str()) {
            Some("node") => match (e.get("lat").and_then(|x| x.as_f64()), e.get("lon").and_then(|x| x.as_f64())) {
                (Some(la), Some(lo)) => vec![(la, lo)],
                _ => continue,
            },
            Some("way") => e
                .get("geometry")
                .and_then(|g| g.as_array())
                .map(|g| g.iter().filter_map(|p| Some((p.get("lat")?.as_f64()?, p.get("lon")?.as_f64()?))).collect())
                .unwrap_or_default(),
            Some("relation") => {
                let c = e
                    .get("bounds")
                    .and_then(|b| Some(((b.get("minlat")?.as_f64()? + b.get("maxlat")?.as_f64()?) * 0.5, (b.get("minlon")?.as_f64()? + b.get("maxlon")?.as_f64()?) * 0.5)));
                match c {
                    Some(c) => vec![c],
                    None => continue,
                }
            }
            _ => continue,
        };
        if pts.is_empty() {
            continue;
        }
        let height = tag("height").trim_end_matches(" m").trim_end_matches('m').trim().parse::<f32>().ok();
        match tag("aeroway") {
            "parking_position" => out.stands.push((pts, tag("ref").to_string())),
            "windsock" => out.windsocks.push(centroid(&pts)),
            "holding_position" => out.holds.push(centroid(&pts)),
            "control_tower" | "tower" => out.towers.push((centroid(&pts), height)),
            _ => {
                if tag("man_made") == "tower" || tag("building") == "control_tower" || tag("service") == "aircraft_control" {
                    out.towers.push((centroid(&pts), height));
                }
            }
        }
    }
    Ok(out)
}

fn cache_path(ident: &str) -> std::path::PathBuf {
    std::path::PathBuf::from("cache").join("airports").join(format!("{}.overpass.json", ident.replace(['/', '\\', '.'], "_")))
}

/// Overpass is skipped entirely (procedural fallback only) with
/// `SKYWARD_NO_OVERPASS=1` (offline tests).
pub fn disabled() -> bool {
    std::env::var_os("SKYWARD_NO_OVERPASS").is_some()
}

/// The cached answer for an airport, if any (native).
pub fn cached(ident: &str) -> Option<Arc<OsmAirport>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let text = std::fs::read_to_string(cache_path(ident)).ok()?;
        parse(&text).ok().map(Arc::new)
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = ident;
        None
    }
}

#[cfg(not(target_arch = "wasm32"))]
/// A recent failure for this airport (don't hammer the servers: retry after an hour).
pub fn failed_recently(ident: &str) -> bool {
    std::fs::metadata(cache_path(ident).with_extension("failed")).ok().and_then(|m| m.modified().ok()).and_then(|t| t.elapsed().ok()).is_some_and(|e| e.as_secs() < 3600)
}

#[cfg(target_arch = "wasm32")]
pub fn failed_recently(_ident: &str) -> bool {
    false
}

/// Shorten an HTTP error (ureq repeats the whole URL).
fn short(e: &str) -> String {
    let e = match (e.find("?data="), e.rfind(": ")) {
        (Some(a), Some(b)) if b > a => format!("{}{}", &e[..a], &e[b..]),
        _ => e.to_string(),
    };
    e.chars().take(160).collect()
}

#[cfg(not(target_arch = "wasm32"))]
/// Fetch (blocking, with retries over the mirrors) and cache. Gives up
/// after `budget` (and remembers the failure for an hour).
pub fn fetch_blocking(ident: &str, bbox: (f64, f64, f64, f64), budget: std::time::Duration) -> Result<Arc<OsmAirport>, String> {
    let r = fetch_inner(ident, bbox, budget);
    let fail = cache_path(ident).with_extension("failed");
    match &r {
        Ok(_) => {
            let _ = std::fs::remove_file(&fail);
        }
        Err(_) => {
            if let Some(d) = fail.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let _ = std::fs::write(&fail, b"");
        }
    }
    r
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_inner(ident: &str, bbox: (f64, f64, f64, f64), budget: std::time::Duration) -> Result<Arc<OsmAirport>, String> {
    if crate::platform::offline() {
        return Err("offline mode".into());
    }
    use std::io::Read;
    let t0 = web_time::Instant::now();
    let agent = ureq::AgentBuilder::new().timeout_connect(std::time::Duration::from_secs(8)).timeout(std::time::Duration::from_secs(70)).user_agent(USER_AGENT).build();
    let q = query(bbox);
    let mut wait = 2.0f32;
    let mut last = String::from("no attempt");
    for round in 0..4 {
        for m in MIRRORS {
            if t0.elapsed() > budget {
                return Err(format!("overpass: gave up after {:.0} s ({last})", t0.elapsed().as_secs_f32()));
            }
            match agent.get(m).query("data", &q).call() {
                Ok(r) => {
                    let mut b = Vec::new();
                    if r.into_reader().take(64 << 20).read_to_end(&mut b).is_err() {
                        last = format!("{m}: read failed");
                        continue;
                    }
                    let text = String::from_utf8_lossy(&b).to_string();
                    match parse(&text) {
                        Ok(a) => {
                            let p = cache_path(ident);
                            if let Some(d) = p.parent() {
                                let _ = std::fs::create_dir_all(d);
                            }
                            let tmp = p.with_extension(format!("tmp{}", std::process::id()));
                            if std::fs::write(&tmp, &text).is_ok() {
                                let _ = std::fs::rename(&tmp, &p);
                            }
                            return Ok(Arc::new(a));
                        }
                        Err(e) => last = format!("{m}: {e}"),
                    }
                }
                Err(e) => last = short(&e.to_string()),
            }
        }
        let _ = round;
        std::thread::sleep(std::time::Duration::from_secs_f32(wait));
        wait *= 2.0;
    }
    Err(format!("overpass: {last}"))
}

#[cfg(target_arch = "wasm32")]
/// Fetch through the browser (CORS), with retries over the mirrors.
pub async fn fetch_async(bbox: (f64, f64, f64, f64)) -> Result<Arc<OsmAirport>, String> {
    use crate::platform::web;
    use web_time::Duration;
    let q = encode(&query(bbox));
    let mut wait = 2.0f32;
    let mut last = String::new();
    for _ in 0..3 {
        for m in MIRRORS {
            match web::fetch(&format!("{m}?data={q}"), Duration::from_secs(70)).await {
                Ok(r) if (200..300).contains(&r.status) => match parse(&String::from_utf8_lossy(&r.body)) {
                    Ok(a) => return Ok(Arc::new(a)),
                    Err(e) => last = e,
                },
                Ok(r) => last = format!("HTTP {}", r.status),
                Err(e) => last = e,
            }
        }
        web::sleep(Duration::from_secs_f32(wait)).await;
        wait *= 2.0;
    }
    Err(format!("overpass: {last}"))
}

#[cfg(test)]
mod tests {
    #[test]
    fn parses_overpass_json() {
        let j = r#"{"elements":[
            {"type":"node","id":1,"lat":51.47,"lon":-0.45,"tags":{"aeroway":"windsock"}},
            {"type":"way","id":2,"geometry":[{"lat":51.471,"lon":-0.451},{"lat":51.4712,"lon":-0.4515}],"tags":{"aeroway":"parking_position","ref":"512"}},
            {"type":"node","id":3,"lat":51.47,"lon":-0.46,"tags":{"man_made":"tower","tower:type":"observation","height":"87"}}
        ]}"#;
        let a = super::parse(j).unwrap();
        assert_eq!(a.windsocks.len(), 1);
        assert_eq!(a.stands.len(), 1);
        assert_eq!(a.stands[0].1, "512");
        assert_eq!(a.towers[0].1, Some(87.0));
        assert!(super::encode("a b\"").contains("%20"));
    }
}
