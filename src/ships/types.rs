//! What a ship is: the AIS ship-type code (ITU-R M.1371) and its length →
//! a [`Class`] (behaviour, speed, lights, density layer) → a [`Model`]
//! (which hull is drawn, scaled to the length).
//!
//! No canonical type → model table exists; this one buckets by type code ×
//! length band, as the ADS-B stand-ins do by ICAO type.

/// Behaviour class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Class {
    /// General cargo, bulk carriers, ro-ro.
    Cargo,
    /// Big box ships (cargo ≥ 250 m).
    Container,
    Tanker,
    /// Ferries (passenger / ro-pax).
    Passenger,
    /// Passenger ≥ 200 m that is not a known ferry line.
    Cruise,
    HighSpeed,
    Fishing,
    /// Tugs, towing, pilot boats.
    Tug,
    /// Dredgers, SAR, law enforcement, other service craft.
    Service,
    Military,
    /// Motor yachts and pleasure craft.
    Pleasure,
    Sailing,
    /// Inland barges, pontoons.
    Barge,
}

impl Class {
    pub const ALL: [Class; 13] = [
        Class::Cargo,
        Class::Container,
        Class::Tanker,
        Class::Passenger,
        Class::Cruise,
        Class::HighSpeed,
        Class::Fishing,
        Class::Tug,
        Class::Service,
        Class::Military,
        Class::Pleasure,
        Class::Sailing,
        Class::Barge,
    ];

    /// By a short key (`cargo`, `container`, `tanker`, `passenger`, `cruise`,
    /// `highspeed`, `fishing`, `tug`, `service`, `military`, `pleasure`,
    /// `sailing`, `barge`), for placements on the command line.
    pub fn parse(s: &str) -> Option<Class> {
        Class::ALL.into_iter().find(|c| format!("{c:?}").eq_ignore_ascii_case(s))
    }

    pub fn name(self) -> &'static str {
        match self {
            Class::Cargo => "cargo",
            Class::Container => "container ship",
            Class::Tanker => "tanker",
            Class::Passenger => "ferry",
            Class::Cruise => "cruise ship",
            Class::HighSpeed => "fast ferry",
            Class::Fishing => "fishing vessel",
            Class::Tug => "tug",
            Class::Service => "service vessel",
            Class::Military => "naval vessel",
            Class::Pleasure => "motor yacht",
            Class::Sailing => "sailing yacht",
            Class::Barge => "barge",
        }
    }

    /// Typical length range (m) of a procedural ship of the class.
    pub fn length_range(self) -> (f32, f32) {
        match self {
            Class::Cargo => (80.0, 200.0),
            Class::Container => (200.0, 366.0),
            Class::Tanker => (90.0, 250.0),
            Class::Passenger => (60.0, 200.0),
            Class::Cruise => (200.0, 330.0),
            Class::HighSpeed => (30.0, 70.0),
            Class::Fishing => (15.0, 55.0),
            Class::Tug => (22.0, 35.0),
            Class::Service => (20.0, 70.0),
            Class::Military => (60.0, 150.0),
            Class::Pleasure => (8.0, 25.0),
            Class::Sailing => (8.0, 18.0),
            Class::Barge => (40.0, 110.0),
        }
    }

    /// Cruising speed range (knots) of a procedural ship under way.
    pub fn speed_kn(self) -> (f32, f32) {
        match self {
            Class::Cargo => (10.0, 14.0),
            Class::Container => (14.0, 19.0),
            Class::Tanker => (10.0, 13.5),
            Class::Passenger => (14.0, 21.0),
            Class::Cruise => (13.0, 18.0),
            Class::HighSpeed => (25.0, 34.0),
            Class::Fishing => (3.0, 9.0),
            Class::Tug => (6.0, 11.0),
            Class::Service => (6.0, 12.0),
            Class::Military => (12.0, 18.0),
            Class::Pleasure => (6.0, 18.0),
            Class::Sailing => (4.0, 7.0),
            Class::Barge => (5.0, 8.0),
        }
    }

    /// Beam / length.
    pub fn beam_ratio(self) -> f32 {
        match self {
            Class::Cargo | Class::Tanker => 0.16,
            Class::Container => 0.14,
            Class::Passenger | Class::Cruise => 0.15,
            Class::HighSpeed => 0.25,
            Class::Fishing => 0.24,
            Class::Tug => 0.34,
            Class::Service => 0.22,
            Class::Military => 0.12,
            Class::Pleasure | Class::Sailing => 0.30,
            Class::Barge => 0.14,
        }
    }

    /// Typical draught (m) at a length, when AIS does not say.
    pub fn draught(self, length: f32) -> f32 {
        let d = match self {
            Class::Cargo | Class::Tanker => 0.058 * length + 1.0,
            Class::Container => 0.045 * length + 1.5,
            Class::Passenger | Class::Cruise => 0.032 * length + 1.5,
            Class::HighSpeed => 2.0,
            Class::Fishing => 0.11 * length + 1.0,
            Class::Tug => 4.5,
            Class::Service => 0.07 * length + 1.0,
            Class::Military => 0.04 * length + 2.0,
            Class::Pleasure => 0.08 * length + 0.4,
            Class::Sailing => 0.13 * length + 0.5,
            Class::Barge => 2.8,
        };
        d.min(22.0)
    }

    /// Anchored / moored share of procedural ships of the class at a hot
    /// spot (density well above the lane level: ports, anchorages).
    pub fn anchor_share(self) -> f32 {
        match self {
            Class::Cargo | Class::Tanker | Class::Container => 0.65,
            Class::Fishing | Class::Pleasure | Class::Sailing => 0.4,
            Class::Tug | Class::Service | Class::Barge => 0.5,
            _ => 0.15,
        }
    }

    /// Loiters (random slow turns) instead of following a lane.
    pub fn loiters(self) -> bool {
        matches!(self, Class::Fishing | Class::Pleasure | Class::Sailing | Class::Service)
    }
}

/// Class from the AIS ship type (0..99), overall length (m, 0 = unknown)
/// and name.
pub fn class_of(ship_type: u8, length: f32, name: &str) -> Class {
    let ferry_name = {
        let n = name.to_ascii_uppercase();
        ["LINE", "VIKING", "SILJA", "ECKER", "FINN", "TALLINK", "STENA", "FERRY", "EXPRESS", "STAR", "MEGASTAR", "MYSTAR"].iter().any(|k| n.contains(k))
    };
    match ship_type {
        30 => Class::Fishing,
        31 | 32 | 52 => Class::Tug,
        33 | 34 | 50 | 51 | 53 | 54 | 55 | 58 | 59 => Class::Service,
        35 => Class::Military,
        36 => Class::Sailing,
        37 => Class::Pleasure,
        40..=49 => {
            if length > 90.0 {
                Class::Passenger
            } else {
                Class::HighSpeed
            }
        }
        60..=69 => {
            if length >= 200.0 && !ferry_name {
                Class::Cruise
            } else {
                Class::Passenger
            }
        }
        70..=79 => {
            if length >= 250.0 {
                Class::Container
            } else {
                Class::Cargo
            }
        }
        80..=89 => Class::Tanker,
        _ => match length {
            l if l <= 0.0 => Class::Service,
            l if l < 25.0 => Class::Pleasure,
            l if l < 60.0 => Class::Service,
            l if l < 120.0 => Class::Barge,
            l if l < 250.0 => Class::Cargo,
            _ => Class::Container,
        },
    }
}

/// The hull drawn for a ship (`assets/ships/<id>/`, src/ships/models.rs),
/// or a procedural one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Model {
    Container,
    Cruise,
    Freighter,
    Tanker,
    Ferry,
    FerrySmall,
    Trawler,
    TrawlerLarge,
    Sailboat,
    Motorsailer,
    Barge,
    /// Procedural workboat hull (tugs, service, military; and while an
    /// imported model loads).
    Workboat,
    /// Procedural grey patrol hull (naval vessels).
    Patrol,
}

impl Model {
    pub const IMPORTED: [Model; 11] = [
        Model::Container,
        Model::Cruise,
        Model::Freighter,
        Model::Tanker,
        Model::Ferry,
        Model::FerrySmall,
        Model::Trawler,
        Model::TrawlerLarge,
        Model::Sailboat,
        Model::Motorsailer,
        Model::Barge,
    ];

    /// Turn (degrees about the vertical) that puts the bow of the imported
    /// model on +Z (some SAVAGE conversions lie along another axis).
    pub fn yaw_fix(self) -> f32 {
        match self {
            Model::Ferry | Model::Barge => 90.0,
            _ => 0.0,
        }
    }

    /// Asset directory under `assets/ships/` (None: procedural).
    pub fn dir(self) -> Option<&'static str> {
        Some(match self {
            Model::Container => "container",
            Model::Cruise => "cruise",
            Model::Freighter => "freighter",
            Model::Tanker => "tanker",
            Model::Ferry => "ferry",
            Model::FerrySmall => "ferry_small",
            Model::Trawler => "trawler",
            Model::TrawlerLarge => "trawler_large",
            Model::Sailboat => "sailboat",
            Model::Motorsailer => "motorsailer",
            Model::Barge => "barge",
            Model::Workboat | Model::Patrol => return None,
        })
    }
}

/// Which hull a ship of a class and length is drawn with.
pub fn model_for(class: Class, length: f32) -> Model {
    match class {
        Class::Container => Model::Container,
        Class::Cargo => Model::Freighter,
        Class::Tanker => Model::Tanker,
        Class::Passenger => {
            if length >= 100.0 {
                Model::Ferry
            } else {
                Model::FerrySmall
            }
        }
        Class::Cruise => Model::Cruise,
        Class::HighSpeed => Model::FerrySmall,
        Class::Fishing => {
            if length >= 35.0 {
                Model::TrawlerLarge
            } else {
                Model::Trawler
            }
        }
        Class::Sailing => Model::Sailboat,
        Class::Pleasure => Model::Motorsailer,
        Class::Barge => Model::Barge,
        Class::Tug | Class::Service => Model::Workboat,
        Class::Military => Model::Patrol,
    }
}

/// Dimensions: AIS length / beam when known (antenna offsets A+B, C+D),
/// else the class's typical figures at `fallback_len`.
pub fn dims(class: Class, a: u16, b: u16, c: u8, d: u8, fallback_len: f32) -> (f32, f32) {
    let len = (a as f32 + b as f32).clamp(0.0, 460.0);
    let beam = (c as f32 + d as f32).clamp(0.0, 70.0);
    let len = if len >= 3.0 { len } else { fallback_len };
    let beam = if beam >= 1.0 && beam < len { beam } else { len * class.beam_ratio() };
    (len, beam)
}
