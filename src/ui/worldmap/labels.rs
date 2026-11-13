//! Label placement: candidates in priority order, each kept only if its box
//! clears every box already placed (and the caller's symbols). A uniform
//! grid keeps it linear-ish for the few hundred candidates a frame has.
//! Ties break on a stable key so labels don't flicker while panning.

use egui::Rect;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Cand {
    /// Lower = placed first.
    pub pri: f32,
    /// Screen box (axis-aligned; rotated labels pass their bounds).
    pub rect: Rect,
    /// Stable tie-breaker / duplicate key (e.g. the text's hash).
    pub key: u64,
}

const CELL: f32 = 64.0;

struct Grid {
    cells: HashMap<(i32, i32), Vec<Rect>>,
}

impl Grid {
    fn range(r: &Rect) -> (i32, i32, i32, i32) {
        ((r.min.x / CELL).floor() as i32, (r.min.y / CELL).floor() as i32, (r.max.x / CELL).floor() as i32, (r.max.y / CELL).floor() as i32)
    }
    fn hits(&self, r: &Rect) -> bool {
        let (x0, y0, x1, y1) = Grid::range(r);
        for x in x0..=x1 {
            for y in y0..=y1 {
                if self.cells.get(&(x, y)).is_some_and(|v| v.iter().any(|o| o.intersects(*r))) {
                    return true;
                }
            }
        }
        false
    }
    fn add(&mut self, r: Rect) {
        let (x0, y0, x1, y1) = Grid::range(&r);
        for x in x0..=x1 {
            for y in y0..=y1 {
                self.cells.entry((x, y)).or_default().push(r);
            }
        }
    }
}

/// Place labels: returns the indices of the accepted candidates (in
/// placement order), at most `max`. `blocked`: boxes already taken.
pub fn place(cands: &[Cand], blocked: &[Rect], max: usize) -> Vec<usize> {
    let mut order: Vec<usize> = (0..cands.len()).collect();
    order.sort_by(|&a, &b| cands[a].pri.total_cmp(&cands[b].pri).then(cands[a].key.cmp(&cands[b].key)));
    let mut g = Grid { cells: HashMap::new() };
    for r in blocked {
        g.add(*r);
    }
    let mut out = Vec::new();
    for i in order {
        if out.len() >= max {
            break;
        }
        let r = cands[i].rect;
        if !r.is_positive() || g.hits(&r) {
            continue;
        }
        g.add(r);
        out.push(i);
    }
    out
}

/// Stable hash of a label text (FNV-1a).
pub fn key(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
}

/// Axis-aligned bounds of a `w` × `h` box centred at `c`, rotated by `a`.
pub fn rotated_bounds(c: egui::Pos2, w: f32, h: f32, a: f32) -> Rect {
    let (s, co) = a.sin_cos();
    let hw = (w * co.abs() + h * s.abs()) * 0.5;
    let hh = (w * s.abs() + h * co.abs()) * 0.5;
    Rect::from_center_size(c, egui::vec2(hw * 2.0, hh * 2.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{pos2, vec2};

    fn c(pri: f32, x: f32, y: f32, k: u64) -> Cand {
        Cand { pri, rect: Rect::from_center_size(pos2(x, y), vec2(60.0, 16.0)), key: k }
    }

    #[test]
    fn higher_priority_wins_overlap() {
        let v = vec![c(5.0, 100.0, 100.0, 1), c(1.0, 110.0, 104.0, 2), c(3.0, 400.0, 100.0, 3)];
        let p = place(&v, &[], 100);
        assert_eq!(p, vec![1, 2]);
    }

    #[test]
    fn disjoint_all_placed_and_capped() {
        let v: Vec<Cand> = (0..20).map(|i| c(i as f32, 100.0 * i as f32, 50.0, i)).collect();
        assert_eq!(place(&v, &[], 100).len(), 20);
        assert_eq!(place(&v, &[], 5), vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn blocked_boxes_respected() {
        let v = vec![c(0.0, 100.0, 100.0, 1), c(1.0, 300.0, 100.0, 2)];
        let p = place(&v, &[Rect::from_center_size(pos2(100.0, 100.0), vec2(8.0, 8.0))], 10);
        assert_eq!(p, vec![1]);
    }

    #[test]
    fn ties_are_stable() {
        let a = vec![c(1.0, 100.0, 100.0, 9), c(1.0, 105.0, 100.0, 4)];
        let b = vec![a[1].clone(), a[0].clone()];
        // the same label wins whatever order the tiles delivered them in
        assert_eq!(a[place(&a, &[], 10)[0]].key, 4);
        assert_eq!(b[place(&b, &[], 10)[0]].key, 4);
    }

    #[test]
    fn grid_catches_large_boxes_across_cells() {
        let big = Cand { pri: 0.0, rect: Rect::from_min_max(pos2(-10.0, -10.0), pos2(500.0, 30.0)), key: 1 };
        let small = c(1.0, 450.0, 20.0, 2);
        assert_eq!(place(&[big, small], &[], 10), vec![0]);
    }

    #[test]
    fn rotation_bounds() {
        let r = rotated_bounds(pos2(0.0, 0.0), 100.0, 10.0, std::f32::consts::FRAC_PI_2);
        assert!((r.width() - 10.0).abs() < 1e-3 && (r.height() - 100.0).abs() < 1e-3);
    }
}
