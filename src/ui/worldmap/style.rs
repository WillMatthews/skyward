//! World-map cartography: day and night palettes (the moving map's colours,
//! ui/map.rs, extended with the classes the world map draws) and the
//! zoom-dependent widths / visibility of every line class.

use super::decode::*;
use egui::Color32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MapStyle {
    #[default]
    Day,
    Night,
}

pub struct Palette {
    pub bg: Color32,
    pub fills: [Color32; N_FILLS],
    /// (fill, casing) per line class.
    pub lines: [(Color32, Color32); N_LINES],
    pub label: Color32,
    pub label2: Color32,
    pub halo: Color32,
    pub water_label: Color32,
    pub shield_bg: Color32,
    pub shield_fg: Color32,
    pub taxi_centre: Color32,
    pub runway: Color32,
    pub runway_edge: Color32,
    pub stand: Color32,
    pub stand_text: Color32,
    pub hold: Color32,
}

fn rgb(h: u32) -> Color32 {
    Color32::from_rgb((h >> 16) as u8, (h >> 8) as u8, h as u8)
}

pub fn palette(s: MapStyle) -> Palette {
    match s {
        MapStyle::Day => Palette {
            bg: rgb(0xEEEDE6),
            fills: [
                rgb(0xE6E2DC), // residential
                rgb(0xEADDDA), // commercial
                rgb(0xE1DDE4), // industrial
                rgb(0xEAEDDE), // farm
                rgb(0xD9E8C6), // grass
                rgb(0xD3E3BF), // scrub
                rgb(0xB5D6A3), // wood
                rgb(0xCDE2D6), // wetland
                rgb(0xF0E6C6), // sand
                rgb(0xF8FBFC), // ice
                rgb(0xC8E5B6), // park
                rgb(0xE3E3EA), // aerodrome
                rgb(0xA4CCEC), // water
                rgb(0xCBCDD6), // apron
                rgb(0xBFC2CC), // helipad
                rgb(0xD9D2CB), // building
            ],
            lines: [
                (rgb(0xA4CCEC), Color32::TRANSPARENT), // stream
                (rgb(0xA4CCEC), Color32::TRANSPARENT), // river
                (rgb(0xD2C7AE), Color32::TRANSPARENT), // path
                (rgb(0xFFFFFF), rgb(0xDCD6CE)),        // service
                (rgb(0xFFFFFF), rgb(0xD5CEC4)),        // minor
                (rgb(0xFFFFFF), rgb(0xCFC7BB)),        // tertiary
                (rgb(0xFFF4CC), rgb(0xD9C68E)),        // secondary
                (rgb(0xFBD68C), rgb(0xD9A85A)),        // primary
                (rgb(0xF8BE70), rgb(0xCE9346)),        // trunk
                (rgb(0xF5A85B), rgb(0xC37A33)),        // motorway
                (rgb(0x9298A4), Color32::TRANSPARENT), // rail
                (rgb(0xC3C6D0), Color32::TRANSPARENT), // taxiway
                (rgb(0x5A5F6C), Color32::TRANSPARENT), // runway
                (rgb(0xB4A9C0), Color32::TRANSPARENT), // state
                (rgb(0x8E809C), Color32::TRANSPARENT), // country
            ],
            label: rgb(0x2B3038),
            label2: rgb(0x5D6470),
            halo: Color32::from_rgba_premultiplied(242, 242, 236, 225),
            water_label: rgb(0x3D6E9C),
            shield_bg: rgb(0x3A6F4A),
            shield_fg: Color32::WHITE,
            taxi_centre: rgb(0xE2B42C),
            runway: rgb(0x4A4F5C),
            runway_edge: rgb(0x2B2F38),
            stand: rgb(0x3E7BC4),
            stand_text: Color32::WHITE,
            hold: rgb(0xD8392B),
        },
        MapStyle::Night => Palette {
            bg: rgb(0x161B22),
            fills: [
                rgb(0x1E232C), // residential
                rgb(0x252833), // commercial
                rgb(0x222531), // industrial
                rgb(0x181E24), // farm
                rgb(0x19271F), // grass
                rgb(0x1B2822), // scrub
                rgb(0x15281E), // wood
                rgb(0x182827), // wetland
                rgb(0x29281F), // sand
                rgb(0x2C3640), // ice
                rgb(0x182A21), // park
                rgb(0x20242D), // aerodrome
                rgb(0x0E2740), // water
                rgb(0x2F3442), // apron
                rgb(0x394052), // helipad
                rgb(0x2E3441), // building
            ],
            lines: [
                (rgb(0x173A5A), Color32::TRANSPARENT),
                (rgb(0x173A5A), Color32::TRANSPARENT),
                (rgb(0x3A3830), Color32::TRANSPARENT),
                (rgb(0x343A46), Color32::TRANSPARENT),
                (rgb(0x3C4350), Color32::TRANSPARENT),
                (rgb(0x46505E), Color32::TRANSPARENT),
                (rgb(0x5A5A4C), Color32::TRANSPARENT),
                (rgb(0x8C7444), rgb(0x3B3324)),
                (rgb(0x9A7742), rgb(0x40321F)),
                (rgb(0xA87A3E), rgb(0x45331F)),
                (rgb(0x4A505C), Color32::TRANSPARENT),
                (rgb(0x434958), Color32::TRANSPARENT),
                (rgb(0x8C95A8), Color32::TRANSPARENT),
                (rgb(0x3A4250), Color32::TRANSPARENT),
                (rgb(0x6C7686), Color32::TRANSPARENT),
            ],
            label: rgb(0xD8DEE6),
            label2: rgb(0x8E98A8),
            halo: Color32::from_rgba_premultiplied(14, 17, 22, 220),
            water_label: rgb(0x6FA6D8),
            shield_bg: rgb(0x2D5A3B),
            shield_fg: rgb(0xE8F0EA),
            taxi_centre: rgb(0xC9A12E),
            runway: rgb(0x8C95A8),
            runway_edge: rgb(0x4B5263),
            stand: rgb(0x4A90E2),
            stand_text: Color32::WHITE,
            hold: rgb(0xE5483A),
        },
    }
}

/// Metres per point at Mercator latitude `lat` (deg) and view zoom.
pub fn m_per_pt(lat: f64, zoom: f64) -> f64 {
    40_075_016.686 * lat.to_radians().cos() / (256.0 * 2f64.powf(zoom))
}

/// Line class drawing at a view zoom: (half-width pt, casing half-width pt, opacity);
/// None = hidden at this zoom.
pub fn line_width(class: u8, zoom: f64, mpp: f64) -> Option<(f32, f32, f32)> {
    // (real width m, min width pt at z10, zoom it appears, casing from zoom)
    let (m, base, from, casing_from) = match class {
        L_STREAM => (4.0, 0.55, 11.5, 99.0),
        L_RIVER => (24.0, 1.1, 5.0, 99.0),
        L_PATH => (2.0, 0.45, 14.5, 99.0),
        L_SERVICE => (5.0, 0.55, 13.0, 15.5),
        L_MINOR => (8.0, 0.8, 11.5, 14.0),
        L_TERTIARY => (11.0, 1.05, 10.0, 13.0),
        L_SECONDARY => (13.0, 1.25, 8.5, 11.5),
        L_PRIMARY => (16.0, 1.45, 7.0, 10.0),
        L_TRUNK => (20.0, 1.7, 5.5, 8.5),
        L_MOTORWAY => (26.0, 1.9, 4.5, 7.0),
        L_RAIL => (3.0, 0.8, 9.0, 99.0),
        L_TAXIWAY => (23.0, 0.8, 11.0, 99.0),
        L_RUNWAY => (45.0, 1.6, 9.0, 99.0),
        L_STATE => (0.0, 0.8, 3.5, 99.0),
        L_COUNTRY => (0.0, 1.3, 0.0, 99.0),
        _ => return None,
    };
    if zoom < from - 0.5 {
        return None;
    }
    // Fade in over half a level.
    let a = ((zoom - (from - 0.5)) / 0.5).clamp(0.0, 1.0) as f32;
    let min_pt = (base * 1.17f64.powf(zoom - 10.0)).clamp(base * 0.45, base * 2.6);
    let w = if m > 0.0 { (m / mpp).max(min_pt) } else { base };
    let w = w.min(400.0) as f32;
    let cas = if zoom >= casing_from { (0.6 + (zoom - casing_from).clamp(0.0, 4.0) * 0.18) as f32 } else { 0.0 };
    Some((w * 0.5, cas, a))
}

/// Fill class visibility (opacity) at a view zoom.
pub fn fill_alpha(class: u8, zoom: f64) -> f32 {
    let from = match class {
        F_BUILDING => 14.5,
        F_HELIPAD => 13.0,
        F_APRON => 10.5,
        _ => 0.0,
    };
    ((zoom - (from - 0.5)) / 0.5).clamp(0.0, 1.0) as f32
}

/// Label visibility at a view zoom: Some((priority, lower placed first; size pt)) when shown.
pub fn label_rule(kind: u8, rank: u8, zoom: f64) -> Option<(f32, f32)> {
    let r = rank as f64;
    let (show, pri, size) = match kind {
        K_COUNTRY => (zoom < 7.0 && zoom > 1.5 + r * 0.5, r, 13.0),
        K_STATE => (zoom > 4.0 && zoom < 9.0, 20.0 + r, 11.0),
        K_CITY => (zoom > 3.0 + r * 0.38 && zoom < 15.5, 8.0 + r, if zoom > 9.0 { 14.0 } else { 12.5 }),
        K_TOWN => (zoom > 7.0 && zoom < 16.0, 22.0 + r, 12.0),
        K_VILLAGE => (zoom > 10.0 && zoom < 17.0, 30.0 + r, 11.0),
        K_SUBURB => (zoom > 11.5 && zoom < 16.5, 34.0 + r, 11.0),
        K_SEA => (zoom < 7.0, 6.0, 12.0),
        K_LAKE => (zoom > 11.0, 40.0, 11.0),
        K_RIVER => (zoom > 12.0, 42.0, 10.5),
        K_SHIELD => (zoom > 8.0 && zoom < 15.0, 26.0 + r, 10.0),
        K_ROAD => (zoom > 14.5 + (r - 3.0).max(0.0) * 0.25, 44.0 + r, 10.5),
        K_TAXI => (zoom > 14.2, 2.0, 10.5),
        K_GATE => (zoom > 15.3, 4.0, 10.0),
        K_PAD => (zoom > 14.5, 3.0, 10.0),
        _ => (false, 99.0, 10.0),
    };
    show.then_some((pri as f32, size))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn widths_scale_with_zoom() {
        let lat = 51.5;
        let w = |c, z: f64| line_width(c, z, m_per_pt(lat, z)).map(|x| x.0).unwrap_or(0.0);
        // motorways get wider as you zoom in, and reach their real width
        assert!(w(L_MOTORWAY, 8.0) < w(L_MOTORWAY, 12.0));
        assert!(w(L_MOTORWAY, 12.0) < w(L_MOTORWAY, 17.0));
        let z = 18.0;
        assert!((w(L_MOTORWAY, z) as f64 - 13.0 / m_per_pt(lat, z)).abs() < 0.5);
        // ordering by class at a city zoom
        assert!(w(L_MOTORWAY, 13.0) > w(L_PRIMARY, 13.0) && w(L_PRIMARY, 13.0) > w(L_MINOR, 13.0));
        // small things hidden far out
        assert!(line_width(L_PATH, 9.0, m_per_pt(lat, 9.0)).is_none());
        assert!(line_width(L_MINOR, 6.0, m_per_pt(lat, 6.0)).is_none());
        assert!(line_width(L_COUNTRY, 2.0, m_per_pt(lat, 2.0)).is_some());
    }

    #[test]
    fn labels_by_zoom() {
        assert!(label_rule(K_COUNTRY, 1, 3.0).is_some());
        assert!(label_rule(K_COUNTRY, 1, 9.0).is_none());
        assert!(label_rule(K_TAXI, 0, 12.0).is_none());
        assert!(label_rule(K_TAXI, 0, 16.0).is_some());
        assert!(label_rule(K_CITY, 1, 6.0).unwrap().0 < label_rule(K_TOWN, 1, 8.0).unwrap().0);
    }
}
