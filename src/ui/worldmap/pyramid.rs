//! Tile pyramid selection: which tiles cover a view at which level, in what
//! order to fetch them (centre first), and what to draw while some are
//! still loading (the nearest loaded ancestor under a missing or fading
//! tile; loaded children when zooming out) — so the map never flashes blank.

use crate::osm::TileId;

/// Deepest level of the vector tiles (OpenFreeMap); views beyond it overzoom.
pub const MAX_TILE_Z: u32 = 14;

/// A tile in a view, with its unwrapped column (views can cross the antimeridian).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Slot {
    pub id: TileId,
    pub tx: i64,
}

impl Slot {
    pub fn new(z: u32, tx: i64, y: u32) -> Slot {
        let n = 1i64 << z;
        Slot { id: TileId { z, x: tx.rem_euclid(n) as u32, y }, tx }
    }
    /// Mercator origin (x, y) and size of the tile (0..1 world units).
    pub fn rect(&self) -> (f64, f64, f64) {
        let s = 1.0 / (1u64 << self.id.z) as f64;
        (self.tx as f64 * s, self.id.y as f64 * s, s)
    }
    pub fn parent(&self) -> Slot {
        Slot::new(self.id.z - 1, self.tx.div_euclid(2), self.id.y / 2)
    }
    pub fn children(&self) -> [Slot; 4] {
        let (z, x, y) = (self.id.z + 1, self.tx * 2, self.id.y * 2);
        [Slot::new(z, x, y), Slot::new(z, x + 1, y), Slot::new(z, x, y + 1), Slot::new(z, x + 1, y + 1)]
    }
}

/// Tile level for a view zoom (world = 256·2^zoom points): tiles are shown
/// 256–512 points wide, overzoomed past `max_z`.
pub fn tile_zoom(view_zoom: f64, max_z: u32) -> u32 {
    view_zoom.floor().clamp(0.0, max_z as f64) as u32
}

/// Tiles at level `z` covering the Mercator box [x0, x1] × [y0, y1], nearest
/// to (cx, cy) first (fetch priority) with its squared distance in tiles.
pub fn cover(x0: f64, y0: f64, x1: f64, y1: f64, cx: f64, cy: f64, z: u32) -> Vec<(Slot, f64)> {
    let n = (1u64 << z) as f64;
    let (tx0, tx1) = ((x0 * n).floor() as i64, (x1 * n).floor() as i64);
    let tx1 = tx1.min(tx0 + n as i64 - 1);
    let ty0 = (y0.max(0.0) * n).floor() as i64;
    let ty1 = ((y1.min(1.0) * n).ceil() as i64 - 1).min(n as i64 - 1);
    let mut out = Vec::new();
    for ty in ty0.max(0)..=ty1 {
        for tx in tx0..=tx1 {
            let d = (tx as f64 + 0.5 - cx * n).powi(2) + (ty as f64 + 0.5 - cy * n).powi(2);
            out.push((Slot::new(z, tx, ty as u32), d));
        }
    }
    out.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.tx.cmp(&b.0.tx)).then(a.0.id.y.cmp(&b.0.id.y)));
    out
}

/// One tile to draw and its opacity (fading in).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Draw {
    pub slot: Slot,
    pub alpha: f32,
}

/// What to draw for the wanted tiles (coarse first: each tile paints its
/// own background, so finer tiles cover their fallbacks), and which wanted
/// tiles are missing. `have(id)`: None = not loaded, Some(opacity).
pub fn plan(want: &[Slot], have: impl Fn(TileId) -> Option<f32>) -> (Vec<Draw>, Vec<TileId>) {
    let mut out: Vec<Draw> = Vec::new();
    let mut missing = Vec::new();
    let push = |out: &mut Vec<Draw>, s: Slot, a: f32| {
        if !out.iter().any(|d| d.slot == s) {
            out.push(Draw { slot: s, alpha: a });
        }
    };
    for &s in want {
        let a = have(s.id);
        if a.is_some_and(|a| a >= 1.0) {
            push(&mut out, s, 1.0);
            continue;
        }
        match a {
            Some(a) => push(&mut out, s, a),
            None => missing.push(s.id),
        }
        // Zooming out: the finer tiles already here stand in.
        if a.is_none() && s.id.z < MAX_TILE_Z {
            let ch = s.children();
            let opaque: Vec<Slot> = ch.iter().copied().filter(|c| have(c.id).is_some_and(|a| a >= 1.0)).collect();
            for c in &opaque {
                push(&mut out, *c, 1.0);
            }
            if opaque.len() == 4 {
                continue;
            }
        }
        // Underneath: loaded ancestors up to the first opaque one.
        let mut p = s;
        while p.id.z > 0 {
            p = p.parent();
            if let Some(pa) = have(p.id) {
                push(&mut out, p, pa.min(1.0));
                if pa >= 1.0 {
                    break;
                }
            }
        }
    }
    // Coarse to fine; stable, so the wanted order holds within a level.
    out.sort_by_key(|d| d.slot.id.z);
    (out, missing)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn ids(v: &[Draw]) -> Vec<(u32, i64, u32)> {
        v.iter().map(|d| (d.slot.id.z, d.slot.tx, d.slot.id.y)).collect()
    }

    #[test]
    fn level_by_screen_size() {
        assert_eq!(tile_zoom(0.3, 14), 0);
        assert_eq!(tile_zoom(5.99, 14), 5);
        assert_eq!(tile_zoom(6.0, 14), 6);
        // overzoom: stand-level views keep the deepest tiles
        assert_eq!(tile_zoom(18.4, 14), 14);
        assert_eq!(tile_zoom(11.0, 8), 8);
        assert_eq!(tile_zoom(-1.0, 14), 0);
    }

    #[test]
    fn cover_view_centre_first() {
        // A 1000 x 600 point view at zoom 10 over London: tiles are 256 pt wide.
        let (cx, cy) = crate::ui::overview::merc(51.47, -0.46);
        let s = 256.0 * 2f64.powi(10);
        let (x0, x1) = (cx - 500.0 / s, cx + 500.0 / s);
        let (y0, y1) = (cy - 300.0 / s, cy + 300.0 / s);
        let c = cover(x0, y0, x1, y1, cx, cy, 10);
        assert!(c.len() >= 12 && c.len() <= 6 * 4, "{}", c.len());
        // every tile overlaps the view, the first one holds the centre
        let first = c[0].0;
        let (ox, oy, sz) = first.rect();
        assert!(cx >= ox && cx < ox + sz && cy >= oy && cy < oy + sz);
        for (sl, _) in &c {
            let (ox, oy, sz) = sl.rect();
            assert!(ox < x1 && ox + sz > x0 && oy < y1 && oy + sz > y0);
            assert_eq!(sl.id.z, 10);
        }
        // sorted by distance
        assert!(c.windows(2).all(|w| w[0].1 <= w[1].1));
    }

    #[test]
    fn cover_wraps_antimeridian() {
        // centred on 180°: columns -1 and 0 of the next world
        let c = cover(0.98, 0.4, 1.02, 0.45, 1.0, 0.42, 4);
        let xs: Vec<(i64, u32)> = c.iter().map(|s| (s.0.tx, s.0.id.x)).collect();
        assert!(xs.contains(&(15, 15)) && xs.contains(&(16, 0)), "{xs:?}");
        // a view wider than the world holds each column once
        let c = cover(-0.5, 0.0, 1.5, 1.0, 0.5, 0.5, 1);
        assert_eq!(c.len(), 4);
        // poles clamp
        let c = cover(0.1, -0.2, 0.2, 0.01, 0.15, 0.0, 3);
        assert!(c.iter().all(|s| s.0.id.y == 0));
    }

    #[test]
    fn plan_all_loaded_draws_only_wanted() {
        let want: Vec<Slot> = cover(0.5, 0.3, 0.52, 0.32, 0.51, 0.31, 8).into_iter().map(|s| s.0).collect();
        let (d, m) = plan(&want, |_| Some(1.0));
        assert!(m.is_empty());
        assert_eq!(d.len(), want.len());
        assert!(d.iter().all(|x| x.alpha == 1.0 && x.slot.id.z == 8));
    }

    #[test]
    fn plan_keeps_parent_until_children_arrive() {
        let parent = Slot::new(9, 100, 200);
        let ch = parent.children();
        let mut loaded: HashMap<TileId, f32> = HashMap::new();
        loaded.insert(parent.id, 1.0);
        loaded.insert(ch[0].id, 1.0);
        loaded.insert(ch[1].id, 0.4); // fading in
        let (d, m) = plan(&ch, |id| loaded.get(&id).copied());
        // the two missing children are reported, the parent drawn first (under)
        assert_eq!(m, vec![ch[2].id, ch[3].id]);
        assert_eq!(d[0].slot, parent);
        assert_eq!(ids(&d[1..]), vec![(10, 200, 400), (10, 201, 400)]);
        assert_eq!(d[2].alpha, 0.4);
        // once all four are in, the parent is no longer drawn
        for c in &ch {
            loaded.insert(c.id, 1.0);
        }
        let (d, m) = plan(&ch, |id| loaded.get(&id).copied());
        assert!(m.is_empty() && d.len() == 4 && d.iter().all(|x| x.slot.id.z == 10));
    }

    #[test]
    fn plan_falls_back_to_nearest_loaded_ancestor() {
        // only z3 is here: a z12 tile draws its z3 ancestor (skipping the empty levels)
        let s = Slot::new(12, 2047, 1362);
        let mut a = s;
        while a.id.z > 3 {
            a = a.parent();
        }
        let (d, m) = plan(&[s], |id| (id == a.id).then_some(1.0));
        assert_eq!(m, vec![s.id]);
        assert_eq!(ids(&d), vec![(3, a.tx, a.id.y)]);
        // nothing loaded at all: nothing to draw (the canvas background shows)
        let (d, _) = plan(&[s], |_| None);
        assert!(d.is_empty());
    }

    #[test]
    fn plan_zoom_out_uses_children() {
        let s = Slot::new(6, 31, 21);
        let ch = s.children();
        let (d, m) = plan(&[s], |id| ch.iter().any(|c| c.id == id).then_some(1.0));
        assert_eq!(m, vec![s.id]);
        assert_eq!(d.len(), 4);
        assert!(d.iter().all(|x| x.slot.id.z == 7));
        // partly loaded children over the grandparent
        let gp = s.parent();
        let (d, _) = plan(&[s], |id| (id == ch[0].id || id == gp.id).then_some(1.0));
        assert_eq!(ids(&d), vec![(5, gp.tx, gp.id.y), (7, ch[0].tx, ch[0].id.y)]);
    }

    #[test]
    fn plan_wraps_parents_across_the_antimeridian() {
        // column -1 at z2 is column 3; its parent is column -1 at z1 (= 1)
        let s = Slot::new(2, -1, 1);
        assert_eq!(s.id.x, 3);
        let p = s.parent();
        assert_eq!((p.tx, p.id.x, p.id.y), (-1, 1, 0));
        let (d, _) = plan(&[s], |id| (id == p.id).then_some(1.0));
        assert_eq!(d[0].slot.tx, -1);
    }
}
