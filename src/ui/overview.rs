//! The map view (Web Mercator) shared by the menus' maps, and the menus'
//! world map: the departure screen's airport picker (continuous level of
//! detail down to the stands, ui/worldmap/) and the coarse backdrops of the
//! other menu screens.

use super::canvas;
use super::worldmap::{self, style::MapStyle, Opts, WorldMap};
use egui::{Pos2, Rect};
use std::f64::consts::PI;

/// Deepest tile level of the menus' backdrop maps (the picker goes to z14).
pub const MAX_Z: u32 = 8;
/// Zoom limits of the picker (world = 256·2^zoom points).
pub const MIN_ZOOM: f64 = 1.2;
pub const MAX_ZOOM: f64 = 19.0;

/// Mercator x / y in 0..1 for a lat / lon.
pub fn merc(lat: f64, lon: f64) -> (f64, f64) {
    let lat = lat.clamp(-85.05, 85.05).to_radians();
    ((lon + 180.0) / 360.0, (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0)
}

pub fn unmerc(x: f64, y: f64) -> (f64, f64) {
    let lon = x * 360.0 - 180.0;
    let n = PI - 2.0 * PI * y;
    (n.sinh().atan().to_degrees(), lon)
}

/// Map view: centre in Mercator units, zoom (world width = 256·2^zoom points).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    pub cx: f64,
    pub cy: f64,
    pub zoom: f64,
    pub rect: Rect,
}

impl View {
    pub fn scale(&self) -> f64 {
        256.0 * 2f64.powf(self.zoom)
    }
    pub fn to_screen(&self, x: f64, y: f64) -> Pos2 {
        let s = self.scale();
        let mut dx = x - self.cx;
        dx -= dx.round(); // wrap around the antimeridian
        Pos2::new(self.rect.center().x + (dx * s) as f32, self.rect.center().y + ((y - self.cy) * s) as f32)
    }
    pub fn ll_to_screen(&self, lat: f64, lon: f64) -> Pos2 {
        let (x, y) = merc(lat, lon);
        self.to_screen(x, y)
    }
    pub fn to_merc(&self, p: Pos2) -> (f64, f64) {
        let s = self.scale();
        (self.cx + (p.x - self.rect.center().x) as f64 / s, self.cy + (p.y - self.rect.center().y) as f64 / s)
    }
    pub fn to_ll(&self, p: Pos2) -> (f64, f64) {
        let (x, y) = self.to_merc(p);
        unmerc(x.rem_euclid(1.0), y.clamp(0.0, 1.0))
    }
    /// Zoom by `f` keeping the point under `at` fixed.
    pub fn zoom_at(&mut self, at: Pos2, dz: f64) {
        let before = self.to_merc(at);
        self.zoom = (self.zoom + dz).clamp(MIN_ZOOM, MAX_ZOOM);
        let after = self.to_merc(at);
        self.cx += before.0 - after.0;
        self.cy = (self.cy + before.1 - after.1).clamp(0.0, 1.0);
    }
    /// Metres per point at the view centre.
    pub fn m_per_pt(&self) -> f64 {
        worldmap::style::m_per_pt(unmerc(0.5, self.cy.clamp(0.0, 1.0)).0, self.zoom)
    }
    /// Lat / lon bounds of the view.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let (lat0, lon0) = self.to_ll(self.rect.left_bottom());
        let (lat1, lon1) = self.to_ll(self.rect.right_top());
        let w = self.rect.width() as f64 / self.scale();
        if w >= 1.0 { (lat0, lat1, -180.0, 180.0) } else { (lat0, lat1, lon0, lon1) }
    }
}

/// The menus' world map (kept by the departure picker; the other screens
/// borrow it for their backdrops).
#[derive(Default)]
pub struct Overview {
    pub map: WorldMap,
}

impl Overview {
    pub fn new() -> Overview {
        Overview { map: WorldMap::new() }
    }

    /// Tiles (and airport diagrams) still loading.
    pub fn busy(&self) -> usize {
        self.map.busy()
    }

    /// A coarse backdrop map (z0–z8, night style); returns its label boxes.
    pub fn draw(&mut self, p: &egui::Painter, cv: &mut canvas::Frame, key: &'static str, ppp: f32, v: &View, dark: bool) -> Vec<Rect> {
        let o = Opts { key, max_z: MAX_Z, style: if dark { MapStyle::Night } else { MapStyle::Day }, labels: true, gates: false };
        self.map.draw(p, cv, ppp, v, &o, &[], &mut |_| {})
    }

    /// The picker map with all its detail; labels keep clear of `blocked`,
    /// `under` draws between the map and its labels.
    #[allow(clippy::too_many_arguments)]
    pub fn draw_with(&mut self, p: &egui::Painter, cv: &mut canvas::Frame, ppp: f32, v: &View, o: &Opts, blocked: &[Rect], under: &mut dyn FnMut(&egui::Painter)) -> Vec<Rect> {
        self.map.draw(p, cv, ppp, v, o, blocked, under)
    }
}

/// Browser: the decode job of the tile workers (tiles_web.rs dispatches "overview" here).
#[cfg(target_arch = "wasm32")]
pub mod web {
    pub use crate::ui::worldmap::web::decode_job;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merc_round_trip_and_zoom_anchor() {
        let (x, y) = merc(51.47, -0.4543);
        let (la, lo) = unmerc(x, y);
        assert!((la - 51.47).abs() < 1e-9 && (lo + 0.4543).abs() < 1e-9);
        let mut v = View { cx: x, cy: y, zoom: 10.0, rect: Rect::from_min_size(Pos2::ZERO, egui::vec2(1000.0, 800.0)) };
        let at = Pos2::new(800.0, 200.0);
        let before = v.to_ll(at);
        v.zoom_at(at, 3.5);
        let after = v.to_ll(at);
        assert!((before.0 - after.0).abs() < 1e-9 && (before.1 - after.1).abs() < 1e-9);
        v.zoom_at(at, 40.0);
        assert_eq!(v.zoom, MAX_ZOOM);
        // ~0.2 m per point at the deepest zoom over London
        assert!(v.m_per_pt() < 0.3);
    }
}
