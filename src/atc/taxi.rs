//! Taxi routing over an airport's taxiway network (the map's taxiway
//! centrelines with their refs, `airport/layout.rs`): a graph of resampled
//! centreline nodes joined where taxiways meet or cross, Dijkstra from the
//! aircraft to a runway-holding point, and the clearance's "via" list (the
//! taxiway refs along the route) plus any runways it crosses.

use glam::Vec2;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Resampling step along a centreline (m).
const STEP: f32 = 12.0;
/// Nodes of different taxiways closer than this are joined (m).
const JOIN: f32 = 9.0;
/// A taxiway end within this of another taxiway is joined to it (m).
const SNAP: f32 = 30.0;

#[derive(Clone, Debug)]
pub struct Graph {
    pub pts: Vec<Vec2>,
    /// Taxiway ref of each node ("" = unnamed).
    pub name: Vec<u16>,
    pub names: Vec<String>,
    adj: Vec<Vec<(u32, f32)>>,
}

/// A runway as the router sees it (for crossings): centreline ends, width, ident pair.
#[derive(Clone, Debug)]
pub struct RwyBox {
    pub a: Vec2,
    pub b: Vec2,
    pub width: f32,
    pub name: String,
}

impl RwyBox {
    pub fn contains(&self, p: Vec2) -> bool {
        let d = self.b - self.a;
        let len = d.length().max(1.0);
        let u = d / len;
        let s = (p - self.a).dot(u);
        let l = (p - self.a).perp_dot(u);
        s > 0.0 && s < len && l.abs() < self.width * 0.5
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Route {
    pub pts: Vec<Vec2>,
    /// Taxiway refs in order (consecutive duplicates and unnamed removed).
    pub via: Vec<String>,
    /// Runways crossed on the way (by name, in order).
    pub crosses: Vec<String>,
    pub length: f32,
}

#[derive(PartialEq)]
struct Item(f32, u32);
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}

fn key(p: Vec2, cell: f32) -> (i32, i32) {
    ((p.x / cell).floor() as i32, (p.y / cell).floor() as i32)
}

impl Graph {
    /// Build from taxiway centrelines (polyline, ref).
    pub fn build(taxiways: &[(Vec<Vec2>, String)]) -> Graph {
        let mut g = Graph { pts: Vec::new(), name: Vec::new(), names: vec![String::new()], adj: Vec::new() };
        let mut name_ix: HashMap<String, u16> = HashMap::new();
        let mut ends: Vec<u32> = Vec::new();
        let mut owner: Vec<u32> = Vec::new();
        for (wi, (pl, nm)) in taxiways.iter().enumerate() {
            if pl.len() < 2 {
                continue;
            }
            let n = clean_name(nm);
            let ni = if n.is_empty() {
                0
            } else {
                *name_ix.entry(n.clone()).or_insert_with(|| {
                    g.names.push(n.clone());
                    (g.names.len() - 1) as u16
                })
            };
            let mut prev: Option<u32> = None;
            let first = g.pts.len() as u32;
            for w in pl.windows(2) {
                let (a, b) = (w[0], w[1]);
                let len = a.distance(b);
                let k = (len / STEP).ceil().max(1.0) as usize;
                let start = if prev.is_none() { 0 } else { 1 };
                for i in start..=k {
                    let p = a + (b - a) * (i as f32 / k as f32);
                    let id = g.pts.len() as u32;
                    g.pts.push(p);
                    g.name.push(ni);
                    g.adj.push(Vec::new());
                    owner.push(wi as u32);
                    if let Some(q) = prev {
                        let d = g.pts[q as usize].distance(p);
                        g.adj[q as usize].push((id, d));
                        g.adj[id as usize].push((q, d));
                    }
                    prev = Some(id);
                }
            }
            ends.push(first);
            if let Some(l) = prev {
                ends.push(l);
            }
        }
        // Spatial grid of nodes.
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, p) in g.pts.iter().enumerate() {
            grid.entry(key(*p, SNAP)).or_default().push(i as u32);
        }
        let near = |p: Vec2, r: f32| -> Vec<u32> {
            let (k0, k1) = (key(p - Vec2::splat(r), SNAP), key(p + Vec2::splat(r), SNAP));
            let mut v = Vec::new();
            for x in k0.0..=k1.0 {
                for y in k0.1..=k1.1 {
                    if let Some(c) = grid.get(&(x, y)) {
                        v.extend(c.iter().copied().filter(|&j| g.pts[j as usize].distance(p) <= r));
                    }
                }
            }
            v
        };
        let mut extra: Vec<(u32, u32, f32)> = Vec::new();
        // Crossings / junctions: nodes of different taxiways close together.
        for i in 0..g.pts.len() {
            for j in near(g.pts[i], JOIN) {
                if (j as usize) > i && owner[j as usize] != owner[i] {
                    extra.push((i as u32, j, g.pts[i].distance(g.pts[j as usize])));
                }
            }
        }
        // Dangling ends: join to the nearest node of another taxiway.
        for &e in &ends {
            let p = g.pts[e as usize];
            if let Some(j) = near(p, SNAP)
                .into_iter()
                .filter(|&j| owner[j as usize] != owner[e as usize])
                .min_by(|a, b| g.pts[*a as usize].distance(p).total_cmp(&g.pts[*b as usize].distance(p)))
            {
                extra.push((e, j, g.pts[j as usize].distance(p)));
            }
        }
        for (a, b, d) in extra {
            if !g.adj[a as usize].iter().any(|x| x.0 == b) {
                g.adj[a as usize].push((b, d));
                g.adj[b as usize].push((a, d));
            }
        }
        g
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    /// Nearest node to `p` within `r`.
    pub fn nearest(&self, p: Vec2, r: f32) -> Option<u32> {
        self.pts.iter().enumerate().map(|(i, q)| (i, q.distance(p))).filter(|x| x.1 <= r).min_by(|a, b| a.1.total_cmp(&b.1)).map(|x| x.0 as u32)
    }

    /// Shortest route between two points (each snapped to the network within
    /// `snap` m). Runway pavement costs extra (taxi on taxiways, cross runways
    /// only where needed).
    pub fn route(&self, from: Vec2, to: Vec2, snap: f32, runways: &[RwyBox]) -> Option<Route> {
        let s = self.nearest(from, snap)?;
        let t = self.nearest(to, snap)?;
        let n = self.pts.len();
        let on_rwy: Vec<bool> = self.pts.iter().map(|p| runways.iter().any(|r| r.contains(*p))).collect();
        let mut dist = vec![f32::INFINITY; n];
        let mut prev = vec![u32::MAX; n];
        let mut heap = BinaryHeap::new();
        dist[s as usize] = 0.0;
        heap.push(Item(0.0, s));
        while let Some(Item(d, u)) = heap.pop() {
            if u == t {
                break;
            }
            if d > dist[u as usize] {
                continue;
            }
            for &(v, w) in &self.adj[u as usize] {
                let w = if on_rwy[v as usize] { w * 4.0 } else { w };
                let nd = d + w;
                if nd < dist[v as usize] {
                    dist[v as usize] = nd;
                    prev[v as usize] = u;
                    heap.push(Item(nd, v));
                }
            }
        }
        if !dist[t as usize].is_finite() {
            return None;
        }
        let mut path = vec![t];
        let mut c = t;
        while c != s {
            c = prev[c as usize];
            path.push(c);
        }
        path.reverse();
        let pts: Vec<Vec2> = path.iter().map(|&i| self.pts[i as usize]).collect();
        let length = pts.windows(2).map(|w| w[0].distance(w[1])).sum();
        // Taxiway refs along the way: ignore short stretches (< 40 m) of a
        // taxiway (junction nodes) so "via" lists the taxiways actually followed.
        let mut runs: Vec<(u16, f32)> = Vec::new();
        for (k, &i) in path.iter().enumerate() {
            let nm = self.name[i as usize];
            let seg = if k > 0 { self.pts[path[k - 1] as usize].distance(self.pts[i as usize]) } else { 0.0 };
            match runs.last_mut() {
                Some(r) if r.0 == nm => r.1 += seg,
                _ => runs.push((nm, seg)),
            }
        }
        let mut via: Vec<String> = Vec::new();
        for (k, (nm, len)) in runs.iter().enumerate() {
            let last = k + 1 == runs.len();
            if *nm == 0 || (*len < 150.0 && !last) {
                continue;
            }
            let s = &self.names[*nm as usize];
            if via.last() != Some(s) {
                via.push(s.clone());
            }
        }
        // A clearance names a handful: the first ones and the last.
        if via.len() > 6 {
            let last = via.pop().unwrap_or_default();
            via.truncate(5);
            via.push(last);
        }
        // Runways crossed (entered and left again).
        let mut crosses = Vec::new();
        for r in runways {
            let inside: Vec<bool> = pts.iter().map(|p| r.contains(*p)).collect();
            let entered = inside.iter().position(|x| *x);
            if let Some(e) = entered {
                if e > 0 && inside[e..].iter().any(|x| !*x) && !inside[inside.len() - 1] && !crosses.contains(&r.name) {
                    crosses.push(r.name.clone());
                }
            }
        }
        Some(Route { pts, via, crosses, length })
    }
}

/// A taxiway ref as used on the radio ("A", "S4E", "Link 21" → "L21"); long
/// descriptive names are dropped.
pub fn clean_name(s: &str) -> String {
    let t = s.trim();
    if t.is_empty() {
        return String::new();
    }
    let up = t.to_ascii_uppercase();
    let up = up.strip_prefix("TAXIWAY ").or_else(|| up.strip_prefix("TAXILANE ")).unwrap_or(&up).trim().to_string();
    if let Some(r) = up.strip_prefix("LINK ") {
        return format!("L{}", r.trim());
    }
    let compact: String = up.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
    if compact.len() <= 4 && compact.chars().next().is_some_and(|c| c.is_ascii_alphabetic()) { compact } else { String::new() }
}

/// A runway-holding point to taxi to: (position, taxiway ref, runway end it serves).
#[derive(Clone, Debug, PartialEq)]
pub struct HoldPt {
    pub at: Vec2,
    pub taxiway: String,
}

/// Candidate holding points for a departure (hold, distance from the start of
/// the take-off run): beside that runway, within 700 m of the threshold.
pub fn departure_holds(holds: &[(Vec2, String, String)], runway_name: &str, thr: Vec2, dir: Vec2) -> Vec<(HoldPt, f32)> {
    let mut v: Vec<(HoldPt, f32)> = holds
        .iter()
        .filter(|h| h.2 == runway_name)
        .map(|h| (HoldPt { at: h.0, taxiway: clean_name(&h.1) }, (h.0 - thr).dot(dir)))
        .filter(|x| x.1 > -250.0 && x.1 < 700.0)
        .map(|(h, a)| (h, a.abs()))
        .collect();
    v.sort_by(|a, b| a.1.total_cmp(&b.1));
    v
}

/// The holding point for a departure from the threshold `thr` along `dir`
/// (unit): of the holds beside that runway, the one nearest the start of the
/// take-off run (and not far down the runway).
pub fn departure_hold(holds: &[(Vec2, String, String)], runway_name: &str, thr: Vec2, dir: Vec2) -> Option<HoldPt> {
    holds
        .iter()
        .filter(|h| h.2 == runway_name)
        .map(|h| (h, (h.0 - thr).dot(dir)))
        .filter(|x| x.1 > -250.0)
        .min_by(|a, b| (a.1.abs()).total_cmp(&b.1.abs()))
        .map(|(h, _)| HoldPt { at: h.0, taxiway: clean_name(&h.1) })
}
