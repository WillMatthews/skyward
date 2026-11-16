//! apt.dat geometry (Track AD2): the pavement polygons (row 110 with their
//! 111–116 Bezier rings and holes), painted lines and taxiway lights (line
//! and light codes on pavement edges and row 120 linear features) and the
//! taxiway sign legends (row 20), in an airport's frame.
//!
//! * Bezier paths: a node's control point (112 / 114 / 116) shapes the
//!   segment leaving it; the segment arriving at it uses its mirror image
//!   through the node. Cubic when both ends have one, quadratic when one
//!   does; subdivided until the chord error is under a tolerance.
//! * Surfaces → the scene's paving materials (asphalt shades, apron
//!   concrete, grass, dirt, gravel); transparent / water pavements are not
//!   drawn.
//! * Line codes → painted strokes (solid / dashed / double / hold-short
//!   patterns, colours, black borders on concrete); light codes → the
//!   airfield light instances (`lights.rs`: taxi, unidirectional, flashers).
//! * Sign text (`{@Y}`, `{@L}`, `{@R}`, `{@B}`, `{@@}` back side, `{^l}`
//!   arrows, `{r2}` numerals, `|` dividers) → sign panels for `mesh::sign_faces`.

use super::aptdat::{AptAirport, Path, Pavement};
use super::mesh::Panel;
use crate::lights::{LightInst, colour, kind};
use crate::osm::Proj;
use crate::scene::{MeshData, mat};
use glam::{Vec2, vec2, vec3};

/// `SKYWARD_APTDAT_GEO=0`: keep the taxi network / stands but draw the
/// procedural pavement, markings, lights and signs (comparisons).
pub fn disabled() -> bool {
    std::env::var("SKYWARD_APTDAT_GEO").is_ok_and(|v| v == "0")
}

/// Chord tolerance (m) of the Bezier subdivision: pavement edges, painted lines.
const TOL_PAVE: f32 = 0.2;
const TOL_LINE: f32 = 0.1;
/// Most pieces per Bezier segment.
const MAX_SUB: usize = 24;

/// A tessellated path: points, and per point the line / light type of the
/// segment starting there. Closed paths repeat their first point at the end.
#[derive(Clone, Debug, Default)]
pub struct Poly {
    pub pts: Vec<Vec2>,
    pub line: Vec<u8>,
    pub light: Vec<u8>,
    pub closed: bool,
}

/// Pieces for a Bezier segment so the chord error stays under `tol`
/// (bound: the second derivative's largest norm M, error ≤ M / (8 n²)).
fn pieces(m2: f32, tol: f32) -> usize {
    ((m2 / (8.0 * tol)).sqrt().ceil() as usize).clamp(1, MAX_SUB)
}

fn cubic(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f32) -> Vec2 {
    let u = 1.0 - t;
    p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t)
}

fn quad(p0: Vec2, p1: Vec2, p2: Vec2, t: f32) -> Vec2 {
    let u = 1.0 - t;
    p0 * (u * u) + p1 * (2.0 * u * t) + p2 * (t * t)
}

/// Tessellate a path (lat / lon → the frame by `xy`), chord tolerance `tol` (m).
pub fn tessellate(p: &Path, xy: &dyn Fn(f64, f64) -> Vec2, tol: f32) -> Poly {
    let n = p.nodes.len();
    let mut out = Poly { closed: p.closed, ..Default::default() };
    if n == 0 {
        return out;
    }
    let pos: Vec<Vec2> = p.nodes.iter().map(|k| xy(k.lat, k.lon)).collect();
    let ctl: Vec<Option<Vec2>> = p.nodes.iter().zip(&pos).map(|(k, q)| k.ctrl.map(|c| xy(c.0, c.1)).filter(|c| c.distance(*q) > 0.01)).collect();
    let segs = if p.closed { n } else { n - 1 };
    for i in 0..segs {
        let j = (i + 1) % n;
        let (p0, p3) = (pos[i], pos[j]);
        let h0 = ctl[i];
        let h1 = ctl[j].map(|c| p3 * 2.0 - c);
        let (line, light) = (p.nodes[i].line, p.nodes[i].light);
        let mut push = |q: Vec2| {
            if out.pts.last().is_none_or(|l| l.distance(q) > 0.005) {
                out.pts.push(q);
                out.line.push(line);
                out.light.push(light);
            }
        };
        match (h0, h1) {
            (Some(a), Some(b)) => {
                let m2 = 6.0 * (p0 - a * 2.0 + b).length().max((a - b * 2.0 + p3).length());
                let k = pieces(m2, tol);
                for s in 0..k {
                    push(cubic(p0, a, b, p3, s as f32 / k as f32));
                }
            }
            (Some(h), None) | (None, Some(h)) => {
                let k = pieces(2.0 * (p0 - h * 2.0 + p3).length(), tol);
                for s in 0..k {
                    push(quad(p0, h, p3, s as f32 / k as f32));
                }
            }
            (None, None) => push(p0),
        }
    }
    // The end point (closed: back at the start).
    let last = if p.closed { pos[0] } else { pos[n - 1] };
    let (line, light) = if p.closed { (p.nodes[0].line, p.nodes[0].light) } else { (0, 0) };
    if out.pts.last().is_none_or(|l| l.distance(last) > 0.005) || out.pts.len() == 1 {
        out.pts.push(last);
        out.line.push(line);
        out.light.push(light);
    }
    out
}

// ---------------------------------------------------------------- surfaces

/// Material and colour of a pavement surface code (None: not drawn).
pub fn surface(code: u8) -> Option<(f32, [f32; 3])> {
    Some(match code {
        // Asphalt: XP11 shade, then light / dark / very dark / new (20–38).
        1 | 24..=26 => (mat::ASPHALT, [0.40, 0.40, 0.41]),
        20..=23 => (mat::ASPHALT, [0.48, 0.48, 0.47]),
        27..=30 => (mat::ASPHALT, [0.34, 0.34, 0.35]),
        31..=34 => (mat::ASPHALT, [0.29, 0.29, 0.30]),
        35..=38 => (mat::ASPHALT, [0.24, 0.24, 0.25]),
        // Concrete: XP11 shade, light new, dark (50–57).
        2 | 53 | 54 => (mat::CONCRETE, [0.66, 0.66, 0.63]),
        50..=52 => (mat::CONCRETE, [0.72, 0.72, 0.69]),
        55..=57 => (mat::CONCRETE, [0.57, 0.57, 0.55]),
        3 => (mat::GRASS, [0.35, 0.44, 0.25]),
        4 => (mat::DIFFUSE, [0.46, 0.38, 0.28]),
        5 => (mat::ASPHALT, [0.55, 0.53, 0.50]),
        12 => (mat::DIFFUSE, [0.72, 0.66, 0.56]),
        14 => (mat::DIFFUSE, [0.88, 0.89, 0.91]),
        _ => return None,
    })
}

/// A hard surface (tyres roll on pavement: `ground/surface.rs`).
pub fn hard(code: u8) -> bool {
    matches!(code, 1 | 2 | 5 | 20..=38 | 50..=57)
}

// ---------------------------------------------------------------- painted lines

const YELLOW: [f32; 3] = [0.86, 0.66, 0.10];
const WHITE: [f32; 3] = [0.84, 0.84, 0.82];
const RED: [f32; 3] = [0.72, 0.10, 0.08];
const BLACK: [f32; 3] = [0.06, 0.06, 0.06];
const ORANGE: [f32; 3] = [0.88, 0.42, 0.08];
const BLUE: [f32; 3] = [0.10, 0.25, 0.70];
const GREEN: [f32; 3] = [0.10, 0.50, 0.20];

/// One stroke of a line style: lateral offset (left positive), width,
/// colour, dash (length, gap, phase) or solid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stroke {
    pub off: f32,
    pub w: f32,
    pub col: [f32; 3],
    pub dash: Option<(f32, f32, f32)>,
}

const fn s(off: f32, w: f32, col: [f32; 3]) -> Stroke {
    Stroke { off, w, col, dash: None }
}

const fn d(off: f32, w: f32, col: [f32; 3], dash: f32, gap: f32) -> Stroke {
    Stroke { off, w, col, dash: Some((dash, gap, 0.0)) }
}

/// The strokes of a line type code and whether it has a black border
/// (codes 51–99: types 1–49 for concrete). Hold-short lines (4, 5, 6):
/// the runway is on the left of the string (the broken lines).
pub fn line_style(code: u8) -> (Vec<Stroke>, bool) {
    let (base, border) = if code > 50 && code < 100 { (code - 50, true) } else { (code, false) };
    let v = match base {
        0 => vec![],
        1 => vec![s(0.0, 0.3, YELLOW)],
        2 => vec![d(0.0, 0.3, YELLOW, 1.0, 1.0)],
        3 | 12 => vec![s(0.2, 0.15, YELLOW), s(-0.2, 0.15, YELLOW)],
        4 | 13 => vec![d(0.45, 0.15, YELLOW, 0.9, 0.9), d(0.15, 0.15, YELLOW, 0.9, 0.9), s(-0.15, 0.15, YELLOW), s(-0.45, 0.15, YELLOW)],
        5 | 14 => vec![d(0.2, 0.15, YELLOW, 0.9, 0.9), s(-0.1, 0.15, YELLOW)],
        // ILS hold: a ladder.
        6 => vec![s(0.55, 0.15, YELLOW), s(-0.55, 0.15, YELLOW), d(0.0, 1.1, YELLOW, 0.15, 1.0)],
        7 => vec![s(0.0, 0.15, YELLOW), d(0.35, 0.15, YELLOW, 0.9, 0.9), d(-0.35, 0.15, YELLOW, 0.9, 0.9)],
        8 => vec![d(0.0, 0.3, YELLOW, 1.0, 4.0)],
        9 => vec![d(0.2, 0.15, YELLOW, 1.0, 4.0), d(-0.2, 0.15, YELLOW, 1.0, 4.0)],
        10 => vec![s(0.0, 0.6, YELLOW)],
        11 => vec![d(0.0, 0.6, YELLOW, 1.0, 1.0)],
        // Hatched yellow (bars across).
        19 => vec![d(0.0, 1.0, YELLOW, 0.3, 0.6)],
        20 => vec![s(0.0, 0.2, WHITE)],
        // Zipper / chequerboard: two dashed rows out of phase.
        21 => vec![Stroke { off: 0.15, w: 0.3, col: WHITE, dash: Some((0.3, 0.3, 0.0)) }, Stroke { off: -0.15, w: 0.3, col: WHITE, dash: Some((0.3, 0.3, 0.3)) }],
        22 => vec![d(0.0, 0.2, WHITE, 1.0, 1.0)],
        23 => vec![d(0.0, 0.15, WHITE, 3.0, 3.0)],
        24 => vec![s(0.0, 0.4, WHITE)],
        25 => vec![d(0.0, 0.4, WHITE, 3.0, 3.0)],
        30 => vec![s(0.0, 0.2, RED)],
        31 => vec![d(0.0, 0.2, RED, 1.0, 1.0)],
        32 => vec![s(0.0, 0.45, RED)],
        40 => vec![s(0.0, 0.2, ORANGE)],
        41 => vec![s(0.0, 0.2, BLUE)],
        42 => vec![s(0.0, 0.2, GREEN)],
        43 => vec![d(0.0, 0.2, GREEN, 1.0, 1.0)],
        _ => vec![s(0.0, 0.3, YELLOW)],
    };
    (v, border && base != 0)
}

/// Offset a polyline sideways (left positive), mitred.
fn offset(pts: &[Vec2], off: f32) -> Vec<Vec2> {
    if off == 0.0 {
        return pts.to_vec();
    }
    let n = pts.len();
    (0..n)
        .map(|i| {
            let d0 = if i > 0 { (pts[i] - pts[i - 1]).normalize_or_zero() } else { Vec2::ZERO };
            let d1 = if i + 1 < n { (pts[i + 1] - pts[i]).normalize_or_zero() } else { Vec2::ZERO };
            let dd = (d0 + d1).normalize_or(d0.max(d1));
            // Mitre: keep the offset lines parallel through bends (capped).
            let k = if d0 != Vec2::ZERO && d1 != Vec2::ZERO { 1.0 / dd.dot(d1).max(0.5) } else { 1.0 };
            pts[i] + vec2(dd.y, -dd.x) * (off * k)
        })
        .collect()
}

/// Draw a stroke along a polyline (flat, y = 0). Solid strokes are joined
/// quads (overlapping a little at the joints); dashes run continuously
/// along the whole polyline.
pub fn stroke(m: &mut MeshData, pts: &[Vec2], st: &Stroke) {
    let line = offset(pts, st.off);
    match st.dash {
        None => {
            for w in line.windows(2) {
                let dd = (w[1] - w[0]).normalize_or_zero();
                if dd == Vec2::ZERO {
                    continue;
                }
                let ext = dd * (st.w * 0.25).min(0.1);
                m.ground_rect(w[0] - ext, w[1] + ext, st.w, 0.0, st.col);
            }
        }
        Some((dash, gap, phase)) => {
            let period = dash + gap;
            let mut acc = phase;
            for w in line.windows(2) {
                let len = w[0].distance(w[1]);
                if len < 1e-3 {
                    continue;
                }
                let dd = (w[1] - w[0]) / len;
                let mut k = (acc / period).floor();
                while k * period - acc < len {
                    let a = (k * period - acc).max(0.0);
                    let b = (k * period + dash - acc).min(len);
                    if b > a + 1e-3 {
                        m.ground_rect(w[0] + dd * a, w[0] + dd * b, st.w, 0.0, st.col);
                    }
                    k += 1.0;
                }
                acc += len;
            }
        }
    }
}

/// Runs of equal codes along a tessellated path: (code, points).
fn runs(p: &Poly, codes: &[u8]) -> Vec<(u8, Vec<Vec2>)> {
    let mut out: Vec<(u8, Vec<Vec2>)> = Vec::new();
    for i in 0..p.pts.len().saturating_sub(1) {
        let c = codes[i];
        if c == 0 {
            continue;
        }
        match out.last_mut() {
            Some((lc, pts)) if *lc == c && pts.last() == Some(&p.pts[i]) => pts.push(p.pts[i + 1]),
            _ => out.push((c, vec![p.pts[i], p.pts[i + 1]])),
        }
    }
    out
}

/// Paint the line runs of a path.
fn paint(m: &mut MeshData, p: &Poly) -> usize {
    let mut n = 0;
    for (code, pts) in runs(p, &p.line) {
        let (strokes, border) = line_style(code);
        if strokes.is_empty() {
            continue;
        }
        if border {
            let half = strokes.iter().map(|s| s.off.abs() + s.w * 0.5).fold(0.0f32, f32::max);
            stroke(m, &pts, &s(0.0, half * 2.0 + 0.3, BLACK));
        }
        for st in &strokes {
            stroke(m, &pts, st);
        }
        n += 1;
    }
    n
}

// ---------------------------------------------------------------- lights

/// Points every `step` m along a polyline (from `start` m), with the direction there.
fn along(pts: &[Vec2], step: f32, start: f32) -> Vec<(Vec2, Vec2)> {
    let mut out = Vec::new();
    let mut next = start;
    let mut acc = 0.0;
    for w in pts.windows(2) {
        let len = w[0].distance(w[1]);
        if len < 1e-4 {
            continue;
        }
        let dd = (w[1] - w[0]) / len;
        while next <= acc + len {
            out.push((w[0] + dd * (next - acc), dd));
            next += step;
        }
        acc += len;
    }
    out
}

/// Lights of a path's light runs. `out_side`: for pavement edges, the
/// side (+1 left, −1 right) away from the pavement (edge lights stand ~1 m off it).
fn lights(out: &mut Vec<LightInst>, p: &Poly, out_side: Option<f32>) {
    for (code, pts) in runs(p, &p.light) {
        let len: f32 = pts.windows(2).map(|w| w[0].distance(w[1])).sum();
        let edge = matches!(code, 2 | 6);
        let pts = match out_side {
            Some(sd) if edge => offset(&pts, sd),
            _ => pts,
        };
        let (step, start) = match code {
            // Hold lines: closely spaced across the taxiway.
            3 | 4 => (1.5, 0.75),
            2 | 6 => (if len < 60.0 { (len / 2.0).max(7.5) } else { 30.0 }, 0.0),
            _ => (15.0, 7.5f32.min(len * 0.5)),
        };
        for (k, (q, dd)) in along(&pts, step, start).into_iter().enumerate() {
            let right = vec2(-dd.y, dd.x);
            let p3 = |h: f32| vec3(q.x, h, q.y);
            let alt = if k % 2 == 0 { colour::TAXI_GREEN } else { colour::RWY_YELLOW };
            let l = match code {
                1 | 7 => LightInst::new(p3(0.12), 60.0, colour::TAXI_GREEN, kind::TAXI),
                2 => LightInst::new(p3(0.35), 40.0, colour::TAXI_BLUE, kind::TAXI),
                3 => LightInst::new(p3(0.12), 80.0, colour::RWY_YELLOW, kind::UNIDIR).with_dir(vec3(right.x, 0.05, right.y), 0),
                4 => LightInst::new(p3(0.12), 150.0, colour::RWY_YELLOW, kind::FLASHER).with_dir(vec3(right.x, 0.1, right.y), 0).with_ab(if k % 2 == 0 { 0.0 } else { 0.5 }, 0.0),
                5 | 8 => LightInst::new(p3(0.12), 60.0, alt, kind::TAXI),
                6 => LightInst::new(p3(0.35), 40.0, colour::RWY_RED, kind::TAXI),
                _ => continue,
            };
            out.push(l);
        }
    }
}

// ---------------------------------------------------------------- the whole airport

/// Everything drawn from an airport's apt.dat pavement and linear features
/// (airport frame, flat: `layout::bake` drapes it).
#[derive(Default)]
pub struct Geo {
    /// Pavement polygons (painter's order).
    pub pave: MeshData,
    /// Painted lines (over the pavement).
    pub paint: MeshData,
    pub lights: Vec<LightInst>,
    /// Hard-surface outlines (the tyres' pavement map).
    pub hard: Vec<Vec<Vec2>>,
    /// Painted line runs (replace the procedural markings when > 0).
    pub painted: usize,
}

/// A pavement polygon: tessellated boundary and holes (open rings).
fn rings(pv: &Pavement, xy: &dyn Fn(f64, f64) -> Vec2) -> Vec<Poly> {
    pv.rings.iter().map(|r| tessellate(r, xy, TOL_PAVE)).filter(|p| p.pts.len() >= 4).collect()
}

pub fn build(a: &AptAirport, frame: &Proj) -> Geo {
    let xy = |lat: f64, lon: f64| frame.to_world(lat, lon);
    let mut g = Geo::default();
    let mut edges: Vec<(Poly, Option<f32>)> = Vec::new();
    for pv in &a.pavements {
        let rs = rings(pv, &xy);
        let Some(outer) = rs.first() else { continue };
        let ring = |p: &Poly| p.pts[..p.pts.len() - 1].to_vec();
        if let Some((m, col)) = surface(pv.surface) {
            let holes: Vec<Vec<Vec2>> = rs[1..].iter().map(ring).collect();
            g.pave.mat = m;
            g.pave.flat_poly(&ring(outer), &holes, 0.0, glam::Vec3::Y, col);
            if hard(pv.surface) {
                g.hard.push(ring(outer));
            }
        }
        // Lines / lights on the pavement edges: the outward side from the
        // ring's winding (boundary: away from the inside; holes: into the hole).
        for (k, r) in pv.rings.iter().enumerate() {
            if !r.nodes.iter().any(|n| n.line != 0 || n.light != 0) {
                continue;
            }
            let p = tessellate(r, &xy, TOL_LINE);
            let area = super::mesh::ring_area(&p.pts);
            let side = if k == 0 { area.signum() } else { -area.signum() };
            edges.push((p, Some(side)));
        }
    }
    for r in &a.lines {
        if r.nodes.iter().any(|n| n.line != 0 || n.light != 0) {
            edges.push((tessellate(r, &xy, TOL_LINE), None));
        }
    }
    g.paint.mat = mat::DIFFUSE;
    for (p, side) in &edges {
        g.painted += paint(&mut g.paint, p);
        lights(&mut g.lights, p, *side);
    }
    g
}

// ---------------------------------------------------------------- signs

/// A sign legend: front and back panels (style, text). Empty back: plain.
pub fn sign_text(text: &str) -> (Vec<(Panel, String)>, Vec<(Panel, String)>) {
    let mut sides: [Vec<(Panel, String)>; 2] = [Vec::new(), Vec::new()];
    let mut side = 0;
    let mut style = Panel::Direction;
    let mut cur = String::new();
    fn flush(sides: &mut [Vec<(Panel, String)>; 2], side: usize, style: Panel, cur: &mut String) {
        let t = cur.trim().to_string();
        cur.clear();
        if !t.is_empty() {
            sides[side].push((style, t));
        }
    }
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '{' => {
                let mut inner = String::new();
                for c2 in chars.by_ref() {
                    if c2 == '}' {
                        break;
                    }
                    inner.push(c2);
                }
                for item in inner.split(',').map(str::trim) {
                    let next_style = match item {
                        "@Y" => Some(Panel::Direction),
                        "@L" => Some(Panel::Location),
                        "@R" => Some(Panel::Mandatory),
                        "@B" => Some(Panel::Distance),
                        _ => None,
                    };
                    if let Some(st) = next_style {
                        flush(&mut sides, side, style, &mut cur);
                        style = st;
                        continue;
                    }
                    match item {
                        "@@" => {
                            flush(&mut sides, side, style, &mut cur);
                            side = 1;
                            style = Panel::Direction;
                        }
                        "|" => flush(&mut sides, side, style, &mut cur),
                        "^l" => cur.push('\u{2190}'),
                        "^r" => cur.push('\u{2192}'),
                        "^u" => cur.push('\u{2191}'),
                        "^d" => cur.push('\u{2193}'),
                        "^lu" => cur.push('\u{2196}'),
                        "^ru" => cur.push('\u{2197}'),
                        "^ld" => cur.push('\u{2199}'),
                        "^rd" => cur.push('\u{2198}'),
                        "r1" => cur.push('I'),
                        "r2" => cur.push_str("II"),
                        "r3" => cur.push_str("III"),
                        // No-entry roundel: a bar on red.
                        "no-entry" => {
                            flush(&mut sides, side, style, &mut cur);
                            sides[side].push((Panel::Mandatory, "-".into()));
                        }
                        x if x.chars().count() == 1 => cur.push_str(x),
                        _ => {}
                    }
                }
            }
            '|' => flush(&mut sides, side, style, &mut cur),
            '_' => cur.push(' '),
            c if c.is_ascii_graphic() => cur.push(c.to_ascii_uppercase()),
            _ => {}
        }
    }
    flush(&mut sides, side, style, &mut cur);
    let [front, back] = sides;
    (front, back)
}

/// Legend height (m) of a sign size code (1 small … 3 large; 4 / 5 distance remaining).
pub fn sign_size(code: u8) -> f32 {
    match code {
        1 => 0.45,
        3 | 4 => 0.85,
        _ => 0.65,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::airport::aptdat::PathNode;

    fn node(lat: f64, lon: f64, ctrl: Option<(f64, f64)>, line: u8) -> PathNode {
        PathNode { lat, lon, ctrl, line, light: 0 }
    }

    /// Metres in a local frame (lat / lon read as y / x metres).
    fn flat(lat: f64, lon: f64) -> Vec2 {
        vec2(lon as f32, -lat as f32)
    }

    #[test]
    fn straight_paths_keep_their_nodes() {
        let p = Path { nodes: vec![node(0.0, 0.0, None, 1), node(0.0, 10.0, None, 1), node(10.0, 10.0, None, 0)], closed: false };
        let t = tessellate(&p, &flat, 0.1);
        assert_eq!(t.pts, vec![vec2(0.0, 0.0), vec2(10.0, 0.0), vec2(10.0, -10.0)]);
        assert_eq!(t.line, vec![1, 1, 0]);
        // Closed: back at the start, the closing segment styled by the last node.
        let c = Path { closed: true, ..p };
        let t = tessellate(&c, &flat, 0.1);
        assert_eq!(t.pts.len(), 4);
        assert_eq!(t.pts[3], t.pts[0]);
    }

    #[test]
    fn bezier_curves_stay_within_tolerance() {
        // A quarter circle of radius 100 m from (100, 0) to (0, 100) as a cubic
        // (handles k·r along the tangents), the second handle mirrored at its node.
        let k = 0.5523;
        let p = Path {
            nodes: vec![
                node(0.0, 100.0, Some((100.0 * k, 100.0)), 0),
                // The control point stored at the end node is the one *after*
                // it: its mirror through the node shapes the arriving segment.
                node(100.0, 0.0, Some((100.0, -100.0 * k)), 0),
            ],
            closed: false,
        };
        let t = tessellate(&p, &flat, 0.1);
        assert!(t.pts.len() > 8 && t.pts.len() <= MAX_SUB + 1, "{}", t.pts.len());
        // Every chord midpoint lies within tolerance (+ the cubic's own 0.03 % error) of the circle.
        for w in t.pts.windows(2) {
            let m = (w[0] + w[1]) * 0.5;
            assert!((m.length() - 100.0).abs() < 0.15, "{m:?} {}", m.length());
            assert!((w[0].length() - 100.0).abs() < 0.05);
        }
        // Quadratic (one handle): ends on the end node.
        let q = Path { nodes: vec![node(0.0, 0.0, Some((10.0, 10.0)), 0), node(0.0, 20.0, None, 0)], closed: false };
        let t = tessellate(&q, &flat, 0.05);
        assert!(t.pts.len() > 3);
        assert_eq!(*t.pts.last().unwrap(), vec2(20.0, 0.0));
        assert!(t.pts.iter().all(|p| p.y <= 0.0 && p.y >= -5.01), "{:?}", t.pts);
    }

    #[test]
    fn pavement_with_a_hole_is_triangulated_around_it() {
        let sq = |c: f64, h: f64, ccw: bool| {
            let mut v = vec![node(c - h, c - h, None, 0), node(c - h, c + h, None, 0), node(c + h, c + h, None, 0), node(c + h, c - h, None, 0)];
            if !ccw {
                v.reverse();
            }
            Path { nodes: v, closed: true }
        };
        let mut a = AptAirport::default();
        a.pavements.push(Pavement { surface: 2, hdg: 0.0, rings: vec![sq(0.0, 50.0, true), sq(0.0, 10.0, false)] });
        a.pavements.push(Pavement { surface: 15, hdg: 0.0, rings: vec![sq(200.0, 10.0, true)] });
        let frame = Proj::new(0.0, 0.0);
        let g = build(&a, &frame);
        // Only the concrete one (transparent is not drawn), with the hole left out.
        assert!(g.pave.verts.iter().all(|v| v.mat == mat::CONCRETE));
        let area: f32 = g
            .pave
            .idx
            .chunks(3)
            .map(|t| {
                let p = |i: u32| vec2(g.pave.verts[i as usize].pos[0], g.pave.verts[i as usize].pos[2]);
                (p(t[1]) - p(t[0])).perp_dot(p(t[2]) - p(t[0])).abs() * 0.5
            })
            .sum();
        let side = |deg: f64| (frame.to_world(deg, 0.0) - frame.to_world(0.0, 0.0)).length();
        let want = (2.0 * side(50.0)).powi(2) - (2.0 * side(10.0)).powi(2);
        assert!((area / want - 1.0).abs() < 0.01, "{area} vs {want}");
        assert_eq!(g.hard.len(), 1);
    }

    #[test]
    fn line_codes_map_to_strokes() {
        assert_eq!(line_style(1).0, vec![s(0.0, 0.3, YELLOW)]);
        let (b, border) = line_style(51);
        assert!(border && b == line_style(1).0);
        // Runway hold: two dashed on the runway (left) side, two solid on the right.
        let (h, _) = line_style(4);
        assert_eq!(h.len(), 4);
        assert!(h.iter().filter(|s| s.dash.is_some()).all(|s| s.off > 0.0));
        assert!(h.iter().filter(|s| s.dash.is_none()).all(|s| s.off < 0.0));
        assert_eq!(line_style(3).0.len(), 2);
        assert!(line_style(20).0[0].col == WHITE && line_style(80).1 && line_style(80).0[0].col == RED);
        assert!(line_style(0).0.is_empty() && !line_style(0).1);
        // Dashes: 20 m of 1 m dashes every 2 m.
        let mut m = MeshData::default();
        stroke(&mut m, &[vec2(0.0, 0.0), vec2(10.0, 0.0), vec2(10.0, 10.0)], &d(0.0, 0.2, YELLOW, 1.0, 1.0));
        assert!((10..=11).contains(&(m.idx.len() / 6)), "{}", m.idx.len() / 6);
    }

    #[test]
    fn edge_lights_stand_off_the_pavement() {
        // A 100 m square apron with blue edge lights all round, a green centreline through it.
        let mut ring = Path { nodes: vec![], closed: true };
        for (la, lo) in [(0.0, 0.0), (0.0, 0.0009), (0.0009, 0.0009), (0.0009, 0.0)] {
            ring.nodes.push(PathNode { lat: la, lon: lo, ctrl: None, line: 3, light: 2 });
        }
        let mut a = AptAirport::default();
        a.pavements.push(Pavement { surface: 1, hdg: 0.0, rings: vec![ring.clone()] });
        a.lines.push(Path {
            nodes: vec![PathNode { lat: 0.00045, lon: 0.0, ctrl: None, line: 51, light: 1 }, PathNode { lat: 0.00045, lon: 0.0009, ctrl: None, line: 0, light: 0 }],
            closed: false,
        });
        let frame = Proj::new(0.0, 0.0);
        let g = build(&a, &frame);
        let outer: Vec<Vec2> = ring.nodes.iter().map(|n| frame.to_world(n.lat, n.lon)).collect();
        let blue: Vec<&LightInst> = g.lights.iter().filter(|l| l.col == LightInst::new(glam::Vec3::ZERO, 0.0, colour::TAXI_BLUE, kind::TAXI).col).collect();
        assert!(blue.len() >= 8, "{}", blue.len());
        for l in &blue {
            assert!(!crate::osm::point_in_ring(vec2(l.pos[0], l.pos[2]), &outer), "edge light inside the pavement: {:?}", l.pos);
        }
        let green = g.lights.len() - blue.len();
        assert!((5..=8).contains(&green), "{green}");
        assert_eq!(g.painted, 2);
        assert!(!g.paint.idx.is_empty());
    }

    #[test]
    fn sign_text_parses_into_panels() {
        use Panel::*;
        let (f, b) = sign_text("{@L}A{@R}09-27{@@,@L}A");
        assert_eq!(f, vec![(Location, "A".to_string()), (Mandatory, "09-27".to_string())]);
        assert_eq!(b, vec![(Location, "A".to_string())]);
        let (f, b) = sign_text("{@Y,^l}C{@@}D{^r}");
        assert_eq!(f, vec![(Direction, "\u{2190}C".to_string())]);
        assert_eq!(b, vec![(Direction, "D\u{2192}".to_string())]);
        let (f, _) = sign_text("{@L}N11{@R}09R_CAT_{r2}/{r3}");
        assert_eq!(f[1], (Mandatory, "09R CAT II/III".to_string()));
        let (f, _) = sign_text("{@L}LINK43{@Y,^ld}S{^ru}{|}S4W{^u}");
        assert_eq!(f.len(), 3);
        assert_eq!(f[2], (Direction, "S4W\u{2191}".to_string()));
        let (f, _) = sign_text("{@Y}Y|LINK_52{^r}");
        assert_eq!(f, vec![(Direction, "Y".to_string()), (Direction, "LINK 52\u{2192}".to_string())]);
        // Junk is harmless.
        let (f, b) = sign_text("{{{@Q}}}");
        assert!(f.iter().chain(&b).all(|p| !p.1.is_empty()));
    }

    #[test]
    fn london_city_pavement_lines_lights_and_signs() {
        let text = {
            use std::io::Read;
            let mut s = String::new();
            flate2::read::GzDecoder::new(&include_bytes!("../../tests/data/aptdat_eglc_full.dat.gz")[..]).read_to_string(&mut s).unwrap();
            s
        };
        let a = crate::airport::aptdat::parse(&text).remove(0);
        assert!(a.pavements.len() > 20, "{}", a.pavements.len());
        assert!(a.pavements.iter().any(|p| p.rings.len() > 1), "holes");
        assert!(a.lines.len() > 50, "{}", a.lines.len());
        let frame = Proj::new(51.505278, 0.055278);
        let t0 = std::time::Instant::now();
        let g = build(&a, &frame);
        eprintln!(
            "EGLC: {} pavements, {} lines → {} pave tris, {} paint tris, {} runs, {} lights in {:.1} ms",
            a.pavements.len(),
            a.lines.len(),
            g.pave.idx.len() / 3,
            g.paint.idx.len() / 3,
            g.painted,
            g.lights.len(),
            t0.elapsed().as_secs_f32() * 1000.0
        );
        assert!(g.pave.idx.len() / 3 > 500);
        assert!(g.painted > 50);
        assert!(!g.lights.is_empty());
        // The pavement spans the apron / taxiways along the runway (≈1.5 km).
        let (lo, hi) = super::super::mesh::bbox(&g.pave.verts.iter().map(|v| vec2(v.pos[0], v.pos[2])).collect::<Vec<_>>());
        assert!(hi.x - lo.x > 1000.0 && hi.x - lo.x < 3000.0, "{lo:?} {hi:?}");
        for s in &a.signs {
            let (f, b) = sign_text(&s.text);
            assert!(!f.is_empty() || !b.is_empty(), "{}", s.text);
        }
    }
}
