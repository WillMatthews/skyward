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

pub const TYPES: [Type; 6] = [
    Type { id: "meridian320", role: Role::Jet, span: 34.1, nose: 16.37, tail: -21.24, door: Some((10.6, 1.98, 3.4)) },
    Type { id: "meridian190", role: Role::Jet, span: 28.2, nose: 15.03, tail: -20.80, door: Some((11.2, 1.50, 2.3)) },
    Type { id: "aerolink", role: Role::Turboprop, span: 27.0, nose: 11.83, tail: -15.33, door: None },
    Type { id: "kestrel", role: Role::Light, span: 11.3, nose: 2.28, tail: -5.89, door: None },
    Type { id: "swift", role: Role::Glider, span: 16.9, nose: 2.86, tail: -5.48, door: None },
    Type { id: "peregrine", role: Role::Warbird, span: 11.25, nose: 3.09, tail: -6.14, door: None },
];

pub const A320: u16 = 0;
pub const E190: u16 = 1;
pub const ATR: u16 = 2;
pub const C172: u16 = 3;
pub const GLIDER: u16 = 4;
pub const SPITFIRE: u16 = 5;

pub fn get(i: u16) -> &'static Type {
    &TYPES[(i as usize).min(TYPES.len() - 1)]
}
