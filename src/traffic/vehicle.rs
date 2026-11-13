//! Vehicle types: dimensions, driving behaviour (IDM parameters), the
//! traffic mix by road class and region, and body colours.

use super::net::{Region, class};

/// Vehicle kinds (instance `info` low byte; == traffic.wgsl).
pub mod kind {
    pub const HATCH: u8 = 0;
    pub const SALOON: u8 = 1;
    pub const ESTATE: u8 = 2;
    pub const SUV: u8 = 3;
    pub const CAB: u8 = 4;
    pub const VAN: u8 = 5;
    pub const BUS: u8 = 6;
    pub const DECKER: u8 = 7;
    pub const LORRY: u8 = 8;
    pub const ARTIC: u8 = 9;
    pub const BIKE: u8 = 10;
}
pub const KINDS: usize = 11;

/// Dimensions and driver model of a kind.
#[derive(Clone, Copy, Debug)]
pub struct Spec {
    /// Length, width, height (m).
    pub len: f32,
    pub width: f32,
    pub height: f32,
    /// IDM: max acceleration, comfortable braking (m/s²), time gap (s), standstill gap (m).
    pub a: f32,
    pub b: f32,
    pub t: f32,
    pub s0: f32,
    /// Top speed it keeps to (m/s): lorries and buses are limited.
    pub vmax: f32,
    /// Headlamps: height, half spacing (m); tail lamps: height, half spacing.
    pub head: (f32, f32),
    pub tail: (f32, f32),
}

pub const SPECS: [Spec; KINDS] = [
    Spec { len: 4.05, width: 1.76, height: 1.48, a: 1.7, b: 2.3, t: 1.2, s0: 2.0, vmax: 50.0, head: (0.68, 0.62), tail: (0.85, 0.66) },
    Spec { len: 4.75, width: 1.82, height: 1.45, a: 1.6, b: 2.3, t: 1.15, s0: 2.0, vmax: 50.0, head: (0.66, 0.64), tail: (0.86, 0.68) },
    Spec { len: 4.85, width: 1.82, height: 1.5, a: 1.5, b: 2.2, t: 1.25, s0: 2.0, vmax: 50.0, head: (0.68, 0.64), tail: (0.9, 0.7) },
    Spec { len: 4.65, width: 1.92, height: 1.72, a: 1.5, b: 2.1, t: 1.3, s0: 2.0, vmax: 50.0, head: (0.82, 0.66), tail: (1.0, 0.72) },
    Spec { len: 4.58, width: 1.84, height: 1.83, a: 1.4, b: 2.2, t: 1.1, s0: 1.8, vmax: 40.0, head: (0.72, 0.6), tail: (0.9, 0.7) },
    Spec { len: 5.4, width: 2.0, height: 2.45, a: 1.3, b: 2.0, t: 1.4, s0: 2.2, vmax: 31.0, head: (0.85, 0.72), tail: (0.95, 0.88) },
    Spec { len: 12.0, width: 2.55, height: 3.1, a: 0.9, b: 1.6, t: 1.6, s0: 2.6, vmax: 24.0, head: (0.75, 1.0), tail: (1.0, 1.05) },
    Spec { len: 11.2, width: 2.55, height: 4.4, a: 0.9, b: 1.6, t: 1.6, s0: 2.6, vmax: 20.0, head: (0.75, 1.0), tail: (1.0, 1.05) },
    Spec { len: 8.8, width: 2.5, height: 3.6, a: 0.8, b: 1.6, t: 1.7, s0: 2.6, vmax: 25.0, head: (0.9, 0.95), tail: (0.8, 1.05) },
    Spec { len: 16.5, width: 2.55, height: 4.0, a: 0.6, b: 1.5, t: 1.8, s0: 2.8, vmax: 25.0, head: (0.9, 0.95), tail: (0.8, 1.05) },
    Spec { len: 2.1, width: 0.8, height: 1.4, a: 2.6, b: 3.0, t: 1.0, s0: 1.5, vmax: 45.0, head: (0.95, 0.0), tail: (0.85, 0.0) },
];

pub fn spec(k: u8) -> &'static Spec {
    &SPECS[(k as usize).min(KINDS - 1)]
}

/// Pick a kind for a lane of class `c` (`r` uniform 0..1).
pub fn pick(c: u8, region: Region, urban: bool, r: f32) -> u8 {
    use kind::*;
    let cab = if region.london && urban { 9.0 } else { 0.0 };
    let bus = if region.london { (0.8, 5.5) } else { (4.5, 0.0) };
    let mix: &[(u8, f32)] = match c {
        class::MOTORWAY => &[(HATCH, 18.0), (SALOON, 22.0), (ESTATE, 10.0), (SUV, 16.0), (VAN, 12.0), (LORRY, 4.0), (ARTIC, 7.0), (BUS, 0.8), (BIKE, 1.0)],
        class::TRUNK if urban => &[
            (HATCH, 22.0),
            (SALOON, 20.0),
            (ESTATE, 9.0),
            (SUV, 16.0),
            (CAB, cab * 0.5),
            (VAN, 12.0),
            (LORRY, 3.0),
            (ARTIC, 2.5),
            (BUS, bus.0 * 0.5),
            (DECKER, bus.1 * 0.5),
            (BIKE, 2.5),
        ],
        class::TRUNK => &[(HATCH, 18.0), (SALOON, 22.0), (ESTATE, 10.0), (SUV, 16.0), (VAN, 12.0), (LORRY, 4.0), (ARTIC, 5.0), (BUS, 0.8), (BIKE, 1.5)],
        class::PRIMARY | class::SECONDARY => {
            &[(HATCH, 22.0), (SALOON, 18.0), (ESTATE, 8.0), (SUV, 15.0), (CAB, cab), (VAN, 12.0), (BUS, bus.0), (DECKER, bus.1), (LORRY, 2.5), (ARTIC, 0.6), (BIKE, 4.0)]
        }
        class::TERTIARY | class::MINOR => &[(HATCH, 30.0), (SALOON, 20.0), (ESTATE, 10.0), (SUV, 18.0), (CAB, cab * 0.3), (VAN, 12.0), (LORRY, 1.0), (BIKE, 3.0)],
        class::BUSWAY => &[(BUS, bus.0 + 0.1), (DECKER, bus.1)],
        _ => &[(HATCH, 30.0), (SALOON, 20.0), (ESTATE, 10.0), (SUV, 15.0), (VAN, 20.0)],
    };
    let total: f32 = mix.iter().map(|m| m.1).sum();
    let mut x = r * total;
    for &(k, w) in mix {
        if x < w {
            return k;
        }
        x -= w;
    }
    mix[0].0
}

fn pack(c: [f32; 3]) -> u32 {
    let b = |v: f32| (v.clamp(0.0, 1.0) * 255.0 + 0.5) as u32;
    b(c[0]) | b(c[1]) << 8 | b(c[2]) << 16 | 255 << 24
}

/// Body colour (sRGB, packed rgba8) for a kind; `r`, `r2` uniform 0..1.
pub fn colour(k: u8, region: Region, r: f32, r2: f32) -> u32 {
    use kind::*;
    // Car colour shares (white, black, greys and silver dominate).
    const CARS: [([f32; 3], f32); 13] = [
        ([0.88, 0.88, 0.87], 19.0),
        ([0.035, 0.035, 0.04], 19.0),
        ([0.30, 0.31, 0.33], 16.0),
        ([0.58, 0.59, 0.60], 13.0),
        ([0.14, 0.27, 0.55], 8.0),
        ([0.05, 0.09, 0.22], 5.0),
        ([0.55, 0.04, 0.04], 7.0),
        ([0.30, 0.03, 0.05], 3.0),
        ([0.08, 0.20, 0.12], 3.0),
        ([0.58, 0.52, 0.42], 2.5),
        ([0.80, 0.33, 0.05], 1.5),
        ([0.85, 0.68, 0.10], 1.0),
        ([0.42, 0.30, 0.20], 2.0),
    ];
    let pick_from = |list: &[([f32; 3], f32)], x: f32| {
        let total: f32 = list.iter().map(|c| c.1).sum();
        let mut x = x * total;
        for (c, w) in list {
            if x < *w {
                return *c;
            }
            x -= w;
        }
        list[0].0
    };
    let c = match k {
        CAB => {
            if r < 0.78 {
                [0.02, 0.02, 0.025]
            } else {
                pick_from(&CARS, r2)
            }
        }
        VAN => {
            if r < 0.62 {
                [0.90, 0.90, 0.89]
            } else {
                pick_from(&CARS, r2)
            }
        }
        BUS | DECKER => {
            if region.london {
                [0.72, 0.03, 0.02]
            } else {
                const LIVERIES: [[f32; 3]; 5] = [[0.88, 0.88, 0.86], [0.10, 0.28, 0.62], [0.10, 0.42, 0.25], [0.85, 0.75, 0.25], [0.65, 0.08, 0.10]];
                LIVERIES[(r * 5.0) as usize % 5]
            }
        }
        // Lorries: the cab colour (the box / trailer is mostly white or grey).
        LORRY | ARTIC => {
            const CABS: [([f32; 3], f32); 6] =
                [([0.88, 0.88, 0.87], 6.0), ([0.10, 0.20, 0.50], 2.0), ([0.55, 0.05, 0.04], 2.0), ([0.06, 0.06, 0.07], 2.0), ([0.15, 0.35, 0.20], 1.0), ([0.85, 0.60, 0.08], 1.0)];
            pick_from(&CABS, r)
        }
        _ => pick_from(&CARS, r),
    };
    pack(c)
}
