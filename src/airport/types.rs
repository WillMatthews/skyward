//! Parked-aircraft types: the sim's own aircraft assets reused as static
//! instances, with the dimensions the stand planner needs (body frame:
//! +Z nose, +X left, origin at the CG; from `aircraft_asset` geometry dumps).

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Jet,
    Turboprop,
    Light,
    Glider,
    Warbird,
}

#[derive(Clone, Copy, Debug)]
pub struct Type {
    /// Asset directory (assets/aircraft/<id>).
    pub id: &'static str,
    pub role: Role,
    pub span: f32,
    /// Nose / tail z (body frame).
    pub nose: f32,
    pub tail: f32,
    /// Forward left passenger door: z, x (half fuselage width), sill height above the ground.
    pub door: Option<(f32, f32, f32)>,
}

impl Type {
    pub fn length(&self) -> f32 {
        self.nose - self.tail
    }
}

pub const TYPES: [Type; 11] = [
    Type { id: "meridian320", role: Role::Jet, span: 34.1, nose: 16.37, tail: -21.24, door: Some((10.6, 1.98, 3.4)) },
    Type { id: "meridian190", role: Role::Jet, span: 28.2, nose: 15.03, tail: -20.80, door: Some((11.2, 1.50, 2.3)) },
    Type { id: "aerolink", role: Role::Turboprop, span: 27.0, nose: 11.83, tail: -15.33, door: None },
    Type { id: "kestrel", role: Role::Light, span: 11.3, nose: 2.28, tail: -5.89, door: None },
    Type { id: "swift", role: Role::Glider, span: 16.9, nose: 2.86, tail: -5.48, door: None },
    Type { id: "peregrine", role: Role::Warbird, span: 11.25, nose: 3.09, tail: -6.14, door: None },
    // Military fields only (see `layout::pick_type`); parked with the cargo
    // ramp / crew door open.
    Type { id: "hercules", role: Role::Turboprop, span: 40.3, nose: 12.82, tail: -16.82, door: None },
    Type { id: "vulcan", role: Role::Warbird, span: 33.6, nose: 18.36, tail: -13.73, door: None },
    // Track H2a: the 777 on the widebody stands at hubs, the CRJ with the
    // regional jets, the Citation X on business / GA aprons (airstair down).
    Type { id: "triple7", role: Role::Jet, span: 64.8, nose: 35.49, tail: -38.88, door: Some((28.2, 3.05, 5.3)) },
    Type { id: "crj700", role: Role::Jet, span: 23.3, nose: 15.59, tail: -16.88, door: Some((12.0, 1.35, 2.0)) },
    Type { id: "citationx", role: Role::Light, span: 19.6, nose: 9.70, tail: -12.24, door: None },
];

pub const A320: u16 = 0;
pub const E190: u16 = 1;
pub const ATR: u16 = 2;
pub const C172: u16 = 3;
pub const GLIDER: u16 = 4;
pub const SPITFIRE: u16 = 5;
pub const C130: u16 = 6;
pub const VULCAN: u16 = 7;
pub const TRIPLE7: u16 = 8;
pub const CRJ700: u16 = 9;
pub const CITATIONX: u16 = 10;

pub fn get(i: u16) -> &'static Type {
    &TYPES[(i as usize).min(TYPES.len() - 1)]
}
