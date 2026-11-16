//! WGS84 → Ordnance Survey National Grid (OSGB36, EPSG:27700) eastings and
//! northings, after the Ordnance Survey's "A guide to coordinate systems in
//! Great Britain": the 7-parameter Helmert transformation from ETRS89
//! (taken as WGS84 — the two differ by well under a metre) to OSGB36 on the
//! Airy 1830 ellipsoid, then the National Grid's Transverse Mercator
//! projection.
//!
//! Accuracy: the Helmert step is good to a few metres across Great Britain
//! (OSTN15, the definitive transformation, is a 1 km grid of corrections
//! too large to ship for a 24 m terrain grid). Heights are not transformed.

/// GRS80 (≈ WGS84) and Airy 1830 semi-axes (m).
const GRS80: (f64, f64) = (6_378_137.000, 6_356_752.314_1);
const AIRY: (f64, f64) = (6_377_563.396, 6_356_256.909);

/// ETRS89 → OSGB36 Helmert: translations (m), scale (ppm), rotations (″).
const T: [f64; 3] = [-446.448, 125.157, -542.060];
const S_PPM: f64 = 20.4894;
const R_SEC: [f64; 3] = [-0.1502, -0.2470, -0.8421];

/// National Grid: scale on the central meridian, true origin (49° N, 2° W),
/// false origin offsets (m).
const F0: f64 = 0.999_601_271_7;
const LAT0: f64 = 49.0;
const LON0: f64 = -2.0;
const E0: f64 = 400_000.0;
const N0: f64 = -100_000.0;

/// Geodetic (lat, lon in degrees, height 0) → Earth-centred cartesian on an ellipsoid.
fn cartesian(lat: f64, lon: f64, (a, b): (f64, f64)) -> [f64; 3] {
    let (p, l) = (lat.to_radians(), lon.to_radians());
    let e2 = 1.0 - (b * b) / (a * a);
    let nu = a / (1.0 - e2 * p.sin().powi(2)).sqrt();
    [nu * p.cos() * l.cos(), nu * p.cos() * l.sin(), (1.0 - e2) * nu * p.sin()]
}

/// Cartesian → geodetic (lat, lon in degrees) on an ellipsoid (iterated to < 1e-12 rad).
fn geodetic([x, y, z]: [f64; 3], (a, b): (f64, f64)) -> (f64, f64) {
    let e2 = 1.0 - (b * b) / (a * a);
    let p = x.hypot(y);
    let mut lat = z.atan2(p * (1.0 - e2));
    for _ in 0..10 {
        let nu = a / (1.0 - e2 * lat.sin().powi(2)).sqrt();
        let next = (z + e2 * nu * lat.sin()).atan2(p);
        let done = (next - lat).abs() < 1e-12;
        lat = next;
        if done {
            break;
        }
    }
    (lat.to_degrees(), y.atan2(x).to_degrees())
}

/// ETRS89 (≈ WGS84) → OSGB36 geodetic (lat, lon in degrees).
pub fn helmert(lat: f64, lon: f64) -> (f64, f64) {
    let [x, y, z] = cartesian(lat, lon, GRS80);
    let s = 1.0 + S_PPM * 1e-6;
    let r = R_SEC.map(|v| (v / 3600.0).to_radians());
    let out = [T[0] + s * x - r[2] * y + r[1] * z, T[1] + r[2] * x + s * y - r[0] * z, T[2] - r[1] * x + r[0] * y + s * z];
    geodetic(out, AIRY)
}

/// OSGB36 geodetic (lat, lon in degrees) → National Grid (easting, northing) in metres.
pub fn grid(lat: f64, lon: f64) -> (f64, f64) {
    let (a, b) = AIRY;
    let e2 = 1.0 - (b * b) / (a * a);
    let n = (a - b) / (a + b);
    let (p, p0) = (lat.to_radians(), LAT0.to_radians());
    let dl = (lon - LON0).to_radians();
    let (sp, cp) = p.sin_cos();
    let tp2 = (sp / cp).powi(2);
    let nu = a * F0 / (1.0 - e2 * sp * sp).sqrt();
    let rho = a * F0 * (1.0 - e2) / (1.0 - e2 * sp * sp).powf(1.5);
    let eta2 = nu / rho - 1.0;
    let (n2, n3) = (n * n, n * n * n);
    let (dp, sp_) = (p - p0, p + p0);
    let m = b
        * F0
        * ((1.0 + n + 1.25 * n2 + 1.25 * n3) * dp - (3.0 * n + 3.0 * n2 + 21.0 / 8.0 * n3) * dp.sin() * sp_.cos()
            + (15.0 / 8.0 * n2 + 15.0 / 8.0 * n3) * (2.0 * dp).sin() * (2.0 * sp_).cos()
            - 35.0 / 24.0 * n3 * (3.0 * dp).sin() * (3.0 * sp_).cos());
    let i = m + N0;
    let ii = nu / 2.0 * sp * cp;
    let iii = nu / 24.0 * sp * cp.powi(3) * (5.0 - tp2 + 9.0 * eta2);
    let iiia = nu / 720.0 * sp * cp.powi(5) * (61.0 - 58.0 * tp2 + tp2 * tp2);
    let iv = nu * cp;
    let v = nu / 6.0 * cp.powi(3) * (nu / rho - tp2);
    let vi = nu / 120.0 * cp.powi(5) * (5.0 - 18.0 * tp2 + tp2 * tp2 + 14.0 * eta2 - 58.0 * tp2 * eta2);
    let northing = i + ii * dl.powi(2) + iii * dl.powi(4) + iiia * dl.powi(6);
    let easting = E0 + iv * dl + v * dl.powi(3) + vi * dl.powi(5);
    (easting, northing)
}

/// WGS84 (lat, lon in degrees) → National Grid (easting, northing) in metres.
pub fn from_wgs84(lat: f64, lon: f64) -> (f64, f64) {
    let (la, lo) = helmert(lat, lon);
    grid(la, lo)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dms(d: f64, m: f64, s: f64) -> f64 {
        d + m / 60.0 + s / 3600.0
    }

    /// The OS guide's projection example (Annex C): OSGB36 52°39′27.2531″ N,
    /// 1°43′4.5177″ E → E 651 409.903, N 313 177.270.
    #[test]
    fn projection_matches_the_os_worked_example() {
        let (e, n) = grid(dms(52.0, 39.0, 27.2531), dms(1.0, 43.0, 4.5177));
        assert!((e - 651_409.903).abs() < 0.01 && (n - 313_177.270).abs() < 0.01, "{e} {n}");
        // The true origin maps to the false origin offsets.
        let (e, n) = grid(LAT0, LON0);
        assert!((e - E0).abs() < 1e-6 && (n - N0).abs() < 1e-6, "{e} {n}");
    }

    /// The same point (Caister water tower) from its ETRS89 position
    /// (52°39′28.8282″ N, 1°42′57.8663″ E): the Helmert step lands within a
    /// few metres of the OSGB36 grid position.
    #[test]
    fn helmert_lands_within_metres() {
        let (e, n) = from_wgs84(dms(52.0, 39.0, 28.8282), dms(1.0, 42.0, 57.8663));
        assert!((e - 651_409.9).abs() < 5.0 && (n - 313_177.3).abs() < 5.0, "{e} {n}");
        // Geodetic ↔ cartesian round trip.
        let (la, lo) = geodetic(cartesian(51.5, -0.12, AIRY), AIRY);
        assert!((la - 51.5).abs() < 1e-10 && (lo + 0.12).abs() < 1e-10);
    }

    /// Landmarks against their National Grid references (to ~50 m).
    #[test]
    fn landmarks() {
        // Big Ben, TQ 30254 79641 (530254, 179641).
        let (e, n) = from_wgs84(51.500_729, -0.124_625);
        assert!((e - 530_254.0).abs() < 50.0 && (n - 179_641.0).abs() < 50.0, "Big Ben {e} {n}");
        // Ben Nevis summit, NN 16664 71250 (216664, 771250).
        let (e, n) = from_wgs84(56.796_891, -5.003_675);
        assert!((e - 216_664.0).abs() < 50.0 && (n - 771_250.0).abs() < 50.0, "Ben Nevis {e} {n}");
    }
}
