//! Live 3D cockpits: the instrument channels the imported FlightGear cockpit
//! animations read ([`channels`]) and the glass displays rendered into
//! offscreen textures mapped onto the cockpit screens ([`displays`]).

pub mod channels;
pub mod displays;

/// Ambient daylight 0 (night) .. 1 (day) from the sun's elevation (sine):
/// civil twilight fades the panel lighting in and the displays down.
pub fn daylight(sun_y: f32) -> f32 {
    let t = ((sun_y - (-0.08)) / (0.14 - (-0.08))).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod daylight_tests {
    #[test]
    fn daylight_ramps_through_twilight() {
        assert_eq!(super::daylight(-0.3), 0.0);
        assert_eq!(super::daylight(0.5), 1.0);
        let d = super::daylight(0.03);
        assert!(d > 0.3 && d < 0.7, "{d}");
    }
}
