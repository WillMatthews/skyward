//! Smooth "fly to" between two map views (van Wijk & Nuij, "Smooth and
//! efficient zooming and panning", 2003 — the path d3 / MapLibre use): far
//! jumps zoom out, travel and zoom back in at a perceptually even speed.

const RHO: f64 = std::f64::consts::SQRT_2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlyTo {
    x0: f64,
    y0: f64,
    w0: f64,
    dx: f64,
    dy: f64,
    w1: f64,
    r0: f64,
    /// Path length (in zoom-equivalent units).
    s: f64,
    view_w: f64,
    pub t: f64,
    pub dur: f64,
}

fn width(view_w: f64, zoom: f64) -> f64 {
    view_w / (256.0 * 2f64.powf(zoom))
}

impl FlyTo {
    /// From (x, y, zoom) to (x, y, zoom) in Mercator units; `view_w`: the map width in points.
    pub fn new(from: (f64, f64, f64), to: (f64, f64, f64), view_w: f64) -> FlyTo {
        let view_w = view_w.max(1.0);
        let (w0, w1) = (width(view_w, from.2), width(view_w, to.2));
        let mut dx = to.0 - from.0;
        dx -= dx.round(); // the short way round the antimeridian
        let dy = to.1 - from.1;
        let d2 = dx * dx + dy * dy;
        let (r0, s) = if d2 < 1e-18 {
            (0.0, (w1 / w0).ln() / RHO)
        } else {
            let d1 = d2.sqrt();
            let b0 = (w1 * w1 - w0 * w0 + RHO.powi(4) * d2) / (2.0 * w0 * RHO * RHO * d1);
            let b1 = (w1 * w1 - w0 * w0 - RHO.powi(4) * d2) / (2.0 * w1 * RHO * RHO * d1);
            let r0 = ((b0 * b0 + 1.0).sqrt() - b0).ln();
            let r1 = ((b1 * b1 + 1.0).sqrt() - b1).ln();
            (r0, (r1 - r0) / RHO)
        };
        let dur = (0.35 + s.abs() * 0.22).clamp(0.35, 2.4);
        FlyTo { x0: from.0, y0: from.1, w0, dx, dy, w1, r0, s, view_w, t: 0.0, dur }
    }

    /// The view at path fraction `u` (0..1): (x, y, zoom).
    pub fn at(&self, u: f64) -> (f64, f64, f64) {
        let d2 = self.dx * self.dx + self.dy * self.dy;
        let s = u * self.s;
        let (k, w) = if d2 < 1e-18 {
            (u, self.w0 * (RHO * s).exp())
        } else {
            let d1 = d2.sqrt();
            let c0 = self.r0.cosh();
            let k = self.w0 / (RHO * RHO * d1) * (c0 * (RHO * s + self.r0).tanh() - self.r0.sinh());
            (k, self.w0 * c0 / (RHO * s + self.r0).cosh())
        };
        let w = if u >= 1.0 { self.w1 } else { w };
        let zoom = (self.view_w / (256.0 * w)).log2();
        (self.x0 + k * self.dx, self.y0 + k * self.dy, zoom)
    }

    /// Advance by `dt` seconds (eased); the view now, and whether it arrived.
    pub fn step(&mut self, dt: f64) -> ((f64, f64, f64), bool) {
        self.t = (self.t + dt / self.dur).min(1.0);
        let u = self.t;
        let e = if u < 0.5 { 4.0 * u * u * u } else { 1.0 - (-2.0 * u + 2.0).powi(3) * 0.5 };
        (self.at(e), self.t >= 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ends_where_asked() {
        let f = FlyTo::new((0.5, 0.33, 5.0), (0.49, 0.34, 13.0), 1200.0);
        let a = f.at(0.0);
        let b = f.at(1.0);
        assert!((a.0 - 0.5).abs() < 1e-12 && (a.1 - 0.33).abs() < 1e-12 && (a.2 - 5.0).abs() < 1e-9, "{a:?}");
        assert!((b.0 - 0.49).abs() < 1e-9 && (b.1 - 0.34).abs() < 1e-9 && (b.2 - 13.0).abs() < 1e-6, "{b:?}");
    }

    #[test]
    fn long_jumps_zoom_out_in_between() {
        // London → Sydney at street level: the middle of the flight is far out
        let f = FlyTo::new((0.4995, 0.3326, 13.0), (0.9195, 0.6015, 13.0), 1500.0);
        let m = f.at(0.5);
        assert!(m.2 < 5.0, "{m:?}");
        // zoom changes monotonically out then in
        let zs: Vec<f64> = (0..=20).map(|i| f.at(i as f64 / 20.0).2).collect();
        let lo = zs.iter().cloned().fold(f64::MAX, f64::min);
        let imin = zs.iter().position(|z| *z == lo).unwrap();
        assert!(zs[..=imin].windows(2).all(|w| w[1] <= w[0] + 1e-9));
        assert!(zs[imin..].windows(2).all(|w| w[1] >= w[0] - 1e-9));
    }

    #[test]
    fn pure_zoom_keeps_centre() {
        let f = FlyTo::new((0.25, 0.4, 3.0), (0.25, 0.4, 9.0), 800.0);
        let m = f.at(0.5);
        assert!((m.0 - 0.25).abs() < 1e-12 && (m.2 - 6.0).abs() < 1e-9, "{m:?}");
    }

    #[test]
    fn crosses_the_antimeridian_the_short_way() {
        let f = FlyTo::new((0.99, 0.4, 6.0), (0.01, 0.4, 6.0), 1000.0);
        let m = f.at(0.5);
        assert!((m.0.rem_euclid(1.0) - 1.0).abs() < 0.02 || m.0.rem_euclid(1.0) < 0.02, "{m:?}");
        let mut g = f;
        let mut n = 0;
        while !g.step(1.0 / 60.0).1 {
            n += 1;
        }
        assert!(n > 10 && n < 200, "{n}");
    }
}
