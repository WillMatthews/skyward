//! ICAO type designator → the closest model in the catalogue, the uniform
//! scale that brings its wingspan near the real one, the wake category and
//! how ATC names it.
//!
//! The table lists designators (exact) and families (prefix, `*`). Anything
//! unknown falls back on the emitter category (A1 light … A5 heavy, A7
//! rotorcraft, B1 glider …), then on speed / altitude.

/// Wake turbulence category (ICAO Doc 8643): light, medium, heavy, super.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Wake {
    L,
    M,
    H,
    J,
}

/// How a type is drawn and named.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stand {
    /// Catalogue id of the model drawn.
    pub model: &'static str,
    /// Uniform scale applied to the model (real span / model span, rounded).
    pub scale: f32,
    /// Real wingspan (rotor diameter for helicopters), m.
    pub span: f32,
    pub wake: Wake,
    /// What ATC calls it in traffic information (vocabulary words).
    pub spoken: &'static str,
}

const fn st(model: &'static str, scale: f32, span: f32, wake: Wake, spoken: &'static str) -> Stand {
    Stand { model, scale, span, wake, spoken }
}

use Wake::*;

/// (designator or `PREFIX*`, stand-in). First match wins: exact entries
/// before the families they belong to.
pub const TABLE: &[(&str, Stand)] = &[
    // ---- Airbus narrowbodies → A320
    ("A318", st("meridian320", 0.95, 34.1, M, "airbus")),
    ("A319", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("A320", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("A321", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("A19N", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("A20N", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("A21N", st("meridian320", 1.0, 35.8, M, "airbus")),
    ("BCS1", st("meridian320", 0.98, 35.1, M, "airbus")),
    ("BCS3", st("meridian320", 0.98, 35.1, M, "airbus")),
    // ---- Boeing 737 / 757 → A320 (until a 737 exists)
    ("B37M", st("meridian320", 1.0, 35.9, M, "boeing")),
    ("B38M", st("meridian320", 1.0, 35.9, M, "boeing")),
    ("B39M", st("meridian320", 1.0, 35.9, M, "boeing")),
    ("B3XM", st("meridian320", 1.0, 35.9, M, "boeing")),
    ("B73*", st("meridian320", 0.97, 34.3, M, "boeing")),
    ("B75*", st("meridian320", 1.06, 38.0, M, "boeing")),
    // ---- Widebodies → 777 (scaled)
    ("B77W", st("triple7", 1.0, 64.8, H, "boeing")),
    ("B77L", st("triple7", 1.0, 64.8, H, "boeing")),
    ("B778", st("triple7", 1.1, 71.8, H, "boeing")),
    ("B779", st("triple7", 1.1, 71.8, H, "boeing")),
    ("B77*", st("triple7", 0.94, 60.9, H, "boeing")),
    ("B78*", st("triple7", 0.93, 60.1, H, "boeing")),
    ("B76*", st("triple7", 0.76, 48.5, H, "boeing")),
    ("B74*", st("triple7", 1.03, 66.4, H, "boeing")),
    ("A388", st("triple7", 1.23, 79.8, J, "airbus")),
    ("A35*", st("triple7", 1.0, 64.8, H, "airbus")),
    ("A34*", st("triple7", 0.98, 63.5, H, "airbus")),
    ("A33*", st("triple7", 0.96, 62.0, H, "airbus")),
    ("A30*", st("triple7", 0.69, 44.8, H, "airbus")),
    ("A310", st("triple7", 0.68, 43.9, H, "airbus")),
    ("MD11", st("triple7", 0.8, 51.7, H, "airliner")),
    ("DC10", st("triple7", 0.77, 50.4, H, "airliner")),
    ("IL96", st("triple7", 0.93, 60.1, H, "airliner")),
    // ---- Regional jets
    ("E17*", st("meridian190", 0.91, 26.0, M, "embraer")),
    ("E75*", st("meridian190", 0.98, 28.0, M, "embraer")),
    ("E19*", st("meridian190", 1.0, 28.7, M, "embraer")),
    ("E290", st("meridian190", 1.17, 33.7, M, "embraer")),
    ("E295", st("meridian190", 1.22, 35.1, M, "embraer")),
    ("E135", st("crj700", 0.86, 20.0, M, "regional jet")),
    ("E145", st("crj700", 0.86, 20.0, M, "regional jet")),
    ("E35L", st("crj700", 0.86, 20.0, M, "business jet")),
    ("CRJ1", st("crj700", 0.91, 21.2, M, "regional jet")),
    ("CRJ2", st("crj700", 0.91, 21.2, M, "regional jet")),
    ("CRJ7", st("crj700", 1.0, 23.2, M, "regional jet")),
    ("CRJ9", st("crj700", 1.07, 24.9, M, "regional jet")),
    ("CRJX", st("crj700", 1.13, 26.2, M, "regional jet")),
    ("F70", st("crj700", 1.21, 28.1, M, "regional jet")),
    ("F100", st("crj700", 1.21, 28.1, M, "regional jet")),
    ("RJ*", st("crj700", 1.13, 26.3, M, "regional jet")),
    ("B46*", st("crj700", 1.13, 26.3, M, "regional jet")),
    ("D328", st("crj700", 0.9, 21.0, M, "regional jet")),
    ("J328", st("crj700", 0.9, 21.0, M, "regional jet")),
    // ---- Turboprop airliners → ATR 72
    ("AT4*", st("aerolink", 0.91, 24.6, M, "turboprop")),
    ("AT7*", st("aerolink", 1.0, 27.1, M, "turboprop")),
    ("DH8D", st("aerolink", 1.05, 28.4, M, "turboprop")),
    ("DH8*", st("aerolink", 0.95, 25.9, M, "turboprop")),
    ("SB20", st("aerolink", 0.91, 24.8, M, "turboprop")),
    ("SF34", st("aerolink", 0.79, 21.4, M, "turboprop")),
    ("JS41", st("aerolink", 0.68, 18.4, M, "turboprop")),
    ("E120", st("aerolink", 0.73, 19.8, M, "turboprop")),
    ("D228", st("aerolink", 0.62, 17.0, L, "turboprop")),
    ("DHC6", st("aerolink", 0.73, 19.8, L, "turboprop")),
    ("B190", st("aerolink", 0.65, 17.7, M, "turboprop")),
    ("BE20", st("aerolink", 0.61, 16.6, L, "turboprop")),
    ("BE30", st("aerolink", 0.65, 17.7, L, "turboprop")),
    ("BE35", st("extra500", 0.88, 10.2, L, "light aircraft")),
    ("BE9*", st("aerolink", 0.56, 15.3, L, "turboprop")),
    ("PAY*", st("aerolink", 0.55, 14.9, L, "turboprop")),
    ("SW4", st("aerolink", 0.63, 17.4, L, "turboprop")),
    ("BN2*", st("aerolink", 0.55, 14.9, L, "light aircraft")),
    ("L410", st("aerolink", 0.73, 19.9, L, "turboprop")),
    // ---- Military transports
    ("C130", st("hercules", 1.0, 40.4, M, "military transport")),
    ("C30J", st("hercules", 1.0, 40.4, M, "military transport")),
    ("L100", st("hercules", 1.0, 40.4, M, "military transport")),
    ("A400", st("hercules", 1.05, 42.4, H, "military transport")),
    ("C5", st("galaxy", 1.0, 67.9, H, "military transport")),
    ("C5M", st("galaxy", 1.0, 67.9, H, "military transport")),
    ("C17", st("galaxy", 0.76, 51.8, H, "military transport")),
    ("K35R", st("triple7", 0.61, 39.9, H, "military transport")),
    ("P8", st("meridian320", 1.05, 37.6, M, "military transport")),
    ("E3TF", st("triple7", 0.68, 44.4, H, "military transport")),
    ("VULC", st("vulcan", 1.0, 30.3, H, "military jet")),
    ("CONC", st("concorde", 1.0, 25.6, H, "concorde")),
    // ---- Fighters / fast jets → F-16
    ("F16", st("lancer", 1.0, 9.96, M, "military jet")),
    ("F15", st("lancer", 1.31, 13.1, M, "military jet")),
    ("F18*", st("lancer", 1.2, 12.3, M, "military jet")),
    ("F35", st("lancer", 1.07, 10.7, M, "military jet")),
    ("F22", st("lancer", 1.36, 13.6, M, "military jet")),
    ("EUFI", st("lancer", 1.1, 11.0, M, "military jet")),
    ("TOR", st("lancer", 1.39, 13.9, M, "military jet")),
    ("RFAL", st("lancer", 1.09, 10.9, M, "military jet")),
    ("M2000", st("lancer", 0.91, 9.1, M, "military jet")),
    ("GRIF", st("lancer", 0.84, 8.4, M, "military jet")),
    ("HAWK", st("lancer", 0.94, 9.4, L, "military jet")),
    ("A10", st("lancer", 1.75, 17.5, M, "military jet")),
    ("T38", st("lancer", 0.77, 7.7, L, "military jet")),
    ("PC21", st("extra500", 0.79, 9.1, L, "military jet")),
    // ---- Business jets → Citation X
    ("C750", st("citationx", 1.0, 19.4, M, "business jet")),
    ("C68A", st("citationx", 1.14, 22.0, M, "business jet")),
    ("C680", st("citationx", 0.82, 19.4, M, "business jet")),
    ("C700", st("citationx", 1.14, 22.0, M, "business jet")),
    ("C510", st("citationx", 0.68, 13.2, L, "business jet")),
    ("C25*", st("citationx", 0.73, 14.4, L, "business jet")),
    ("C5*", st("citationx", 0.82, 15.9, L, "business jet")),
    ("C6*", st("citationx", 0.85, 16.3, M, "business jet")),
    ("E50P", st("citationx", 0.63, 12.3, L, "business jet")),
    ("E55P", st("citationx", 0.82, 16.0, L, "business jet")),
    ("E545", st("citationx", 1.07, 20.7, M, "business jet")),
    ("E550", st("citationx", 1.07, 20.7, M, "business jet")),
    ("LJ*", st("citationx", 0.78, 15.5, L, "business jet")),
    ("GLF6", st("citationx", 1.57, 30.4, M, "business jet")),
    ("GLF5", st("citationx", 1.47, 28.5, M, "business jet")),
    ("GLF*", st("citationx", 1.21, 23.7, M, "business jet")),
    ("GA7C", st("citationx", 1.4, 27.4, M, "business jet")),
    ("GA8C", st("citationx", 1.47, 28.6, M, "business jet")),
    ("GLEX", st("citationx", 1.5, 28.7, M, "business jet")),
    ("GL5T", st("citationx", 1.5, 28.7, M, "business jet")),
    ("GL7T", st("citationx", 1.6, 31.7, M, "business jet")),
    ("CL30", st("citationx", 1.1, 21.0, M, "business jet")),
    ("CL35", st("citationx", 1.1, 21.0, M, "business jet")),
    ("CL60", st("citationx", 1.1, 21.0, M, "business jet")),
    ("F2TH", st("citationx", 1.0, 19.3, M, "business jet")),
    ("F900", st("citationx", 1.0, 19.3, M, "business jet")),
    ("FA7X", st("citationx", 1.35, 26.2, M, "business jet")),
    ("FA8X", st("citationx", 1.35, 26.3, M, "business jet")),
    ("FA6X", st("citationx", 1.33, 25.9, M, "business jet")),
    ("FA50", st("citationx", 0.97, 18.9, M, "business jet")),
    ("H25*", st("citationx", 0.85, 16.5, M, "business jet")),
    ("BE40", st("citationx", 0.69, 13.3, L, "business jet")),
    ("PRM1", st("citationx", 0.71, 13.6, L, "business jet")),
    ("HDJT", st("citationx", 0.62, 12.1, L, "business jet")),
    ("G280", st("citationx", 1.0, 19.2, M, "business jet")),
    ("ASTR", st("citationx", 0.85, 16.6, M, "business jet")),
    ("PC24", st("citationx", 0.88, 17.0, L, "business jet")),
    ("SF50", st("citationx", 0.6, 11.8, L, "business jet")),
    // ---- Helicopters → EC130 (rotor diameter)
    ("EC30", st("ec130", 1.0, 10.7, L, "helicopter")),
    ("EC20", st("ec130", 0.96, 10.2, L, "helicopter")),
    ("AS50", st("ec130", 1.0, 10.7, L, "helicopter")),
    ("AS55", st("ec130", 1.0, 10.7, L, "helicopter")),
    ("AS65", st("ec130", 1.1, 11.9, L, "helicopter")),
    ("EC35", st("ec130", 0.95, 10.2, L, "helicopter")),
    ("EC45", st("ec130", 1.03, 11.0, L, "helicopter")),
    ("EC55", st("ec130", 1.13, 12.0, L, "helicopter")),
    ("EC75", st("ec130", 1.38, 14.8, M, "helicopter")),
    ("EC25", st("ec130", 1.53, 16.2, M, "helicopter")),
    ("H160", st("ec130", 1.25, 13.4, L, "helicopter")),
    ("A109", st("ec130", 1.03, 11.0, L, "helicopter")),
    ("A119", st("ec130", 1.03, 10.8, L, "helicopter")),
    ("A139", st("ec130", 1.29, 13.8, M, "helicopter")),
    ("A169", st("ec130", 1.13, 12.1, L, "helicopter")),
    ("A189", st("ec130", 1.36, 14.6, M, "helicopter")),
    ("AW*", st("ec130", 1.29, 13.8, M, "helicopter")),
    ("R22", st("ec130", 0.72, 7.7, L, "helicopter")),
    ("R44", st("ec130", 0.94, 10.1, L, "helicopter")),
    ("R66", st("ec130", 0.94, 10.1, L, "helicopter")),
    ("B06", st("ec130", 1.04, 10.2, L, "helicopter")),
    ("B407", st("ec130", 1.0, 10.7, L, "helicopter")),
    ("B429", st("ec130", 1.0, 11.0, L, "helicopter")),
    ("S76", st("ec130", 1.25, 13.4, M, "helicopter")),
    ("S92", st("ec130", 1.6, 17.2, M, "helicopter")),
    ("H60", st("ec130", 1.53, 16.4, M, "helicopter")),
    ("H47", st("ec130", 1.7, 18.3, H, "helicopter")),
    ("EH10", st("ec130", 1.7, 18.6, M, "helicopter")),
    ("PUMA", st("ec130", 1.4, 15.0, M, "helicopter")),
    ("LYNX", st("ec130", 1.2, 12.8, L, "helicopter")),
    ("GAZL", st("ec130", 0.98, 10.5, L, "helicopter")),
    ("MI8", st("ec130", 2.0, 21.3, M, "helicopter")),
    // ---- Warbirds and classics
    ("SPIT", st("peregrine", 1.0, 11.2, L, "light aircraft")),
    ("HURI", st("peregrine", 1.09, 12.2, L, "light aircraft")),
    ("P51", st("peregrine", 1.0, 11.3, L, "light aircraft")),
    ("DC3", st("dc3", 1.0, 29.0, M, "light aircraft")),
    ("C47", st("dc3", 1.0, 29.0, M, "light aircraft")),
    ("DHC3", st("otter", 1.0, 17.7, L, "light aircraft")),
    ("DHC2", st("otter", 0.83, 14.6, L, "light aircraft")),
    // ---- Gliders
    ("GLID", st("swift", 1.0, 17.0, L, "glider")),
    ("AS21", st("swift", 1.0, 17.0, L, "glider")),
    ("ASK*", st("swift", 1.0, 17.0, L, "glider")),
    ("DG*", st("swift", 1.0, 17.0, L, "glider")),
    ("DUOD", st("swift", 1.18, 20.0, L, "glider")),
    ("VENT", st("swift", 0.88, 15.0, L, "glider")),
    // ---- Aerobatic singles → Zlin
    ("Z50", st("zlin", 1.0, 8.6, L, "light aircraft")),
    ("CAP*", st("zlin", 0.93, 8.0, L, "light aircraft")),
    ("PITS", st("zlin", 0.71, 6.1, L, "light aircraft")),
    ("EA30", st("zlin", 0.93, 8.0, L, "light aircraft")),
    ("EA50", st("extra500", 1.0, 11.6, L, "light aircraft")),
    ("YK52", st("zlin", 1.08, 9.3, L, "light aircraft")),
    // ---- High-wing Pipers / bush planes → Cub
    ("PA18", st("cub", 1.0, 10.7, L, "light aircraft")),
    ("PA11", st("cub", 1.0, 10.8, L, "light aircraft")),
    ("J3", st("cub", 1.0, 10.7, L, "light aircraft")),
    ("CH7*", st("cub", 0.95, 10.2, L, "light aircraft")),
    ("BL8", st("cub", 1.0, 10.5, L, "light aircraft")),
    ("A1", st("cub", 1.0, 10.7, L, "light aircraft")),
    ("M7", st("cub", 1.0, 10.3, L, "light aircraft")),
    ("PA22", st("cub", 0.83, 8.9, L, "light aircraft")),
    ("AUS*", st("cub", 1.0, 10.9, L, "light aircraft")),
    // ---- High-wing Cessnas → C172
    ("C150", st("kestrel", 0.9, 10.1, L, "light aircraft")),
    ("C152", st("kestrel", 0.9, 10.1, L, "light aircraft")),
    ("C172", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C72R", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("P172", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C177", st("kestrel", 0.98, 10.8, L, "light aircraft")),
    ("C182", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C206", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C207", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C210", st("kestrel", 1.02, 11.2, L, "light aircraft")),
    ("C208", st("kestrel", 1.44, 15.9, L, "light aircraft")),
    ("C17*", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C18*", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C20*", st("kestrel", 1.0, 11.0, L, "light aircraft")),
    ("C337", st("kestrel", 1.08, 11.6, L, "light aircraft")),
    ("GA8", st("kestrel", 1.1, 12.0, L, "light aircraft")),
    ("KODI", st("kestrel", 1.33, 14.6, L, "light aircraft")),
    ("PC6T", st("kestrel", 1.45, 15.9, L, "light aircraft")),
    // ---- Low-wing singles / turboprop singles → Extra 500
    ("PC12", st("extra500", 1.4, 16.3, L, "light aircraft")),
    ("TBM*", st("extra500", 1.1, 12.8, L, "light aircraft")),
    ("P46T", st("extra500", 1.13, 13.1, L, "light aircraft")),
    ("PA46", st("extra500", 1.13, 13.1, L, "light aircraft")),
    ("M600", st("extra500", 1.13, 13.1, L, "light aircraft")),
    ("SR2*", st("extra500", 1.0, 11.7, L, "light aircraft")),
    ("S22T", st("extra500", 1.0, 11.7, L, "light aircraft")),
    ("DA4*", st("extra500", 1.02, 11.9, L, "light aircraft")),
    ("DA62", st("extra500", 1.2, 14.6, L, "light aircraft")),
    ("DA20", st("extra500", 0.93, 10.9, L, "light aircraft")),
    ("P28*", st("extra500", 0.92, 10.7, L, "light aircraft")),
    ("PA28", st("extra500", 0.92, 10.7, L, "light aircraft")),
    ("P32*", st("extra500", 0.94, 11.0, L, "light aircraft")),
    ("PA32", st("extra500", 0.94, 11.0, L, "light aircraft")),
    ("PA34", st("extra500", 1.02, 11.9, L, "light aircraft")),
    ("PA44", st("extra500", 0.98, 11.6, L, "light aircraft")),
    ("PA31", st("extra500", 1.1, 12.4, L, "light aircraft")),
    ("BE33", st("extra500", 0.88, 10.2, L, "light aircraft")),
    ("BE36", st("extra500", 0.88, 10.2, L, "light aircraft")),
    ("BE55", st("extra500", 0.98, 11.5, L, "light aircraft")),
    ("BE58", st("extra500", 1.0, 11.5, L, "light aircraft")),
    ("BE76", st("extra500", 0.98, 11.6, L, "light aircraft")),
    ("M20*", st("extra500", 0.94, 11.0, L, "light aircraft")),
    ("TOBA", st("extra500", 0.84, 9.8, L, "light aircraft")),
    ("TB*", st("extra500", 0.84, 9.8, L, "light aircraft")),
    ("RV*", st("extra500", 0.66, 7.7, L, "light aircraft")),
    ("AA5", st("extra500", 0.81, 9.4, L, "light aircraft")),
    ("SIRA", st("extra500", 0.88, 10.2, L, "light aircraft")),
    ("DR40", st("extra500", 0.74, 8.7, L, "light aircraft")),
    ("C310", st("extra500", 0.98, 11.3, L, "light aircraft")),
    ("C340", st("extra500", 1.04, 12.2, L, "light aircraft")),
    ("C402", st("extra500", 1.15, 13.4, L, "light aircraft")),
    ("C414", st("extra500", 1.15, 13.4, L, "light aircraft")),
    ("C421", st("extra500", 1.06, 12.5, L, "light aircraft")),
    ("P68", st("kestrel", 1.09, 12.0, L, "light aircraft")),
    ("EVOT", st("extra500", 0.76, 9.0, L, "light aircraft")),
    ("G115", st("extra500", 0.86, 10.0, L, "light aircraft")),
    ("TUTR", st("extra500", 0.86, 10.0, L, "light aircraft")),
];

/// Matches a table key: exact, or a `PREFIX*` family.
fn matches(key: &str, ty: &str) -> bool {
    match key.strip_suffix('*') {
        Some(prefix) => ty.starts_with(prefix),
        None => key == ty,
    }
}

/// The table entry for a designator.
pub fn lookup(ty: &str) -> Option<Stand> {
    let ty = ty.trim();
    if ty.is_empty() {
        return None;
    }
    TABLE.iter().find(|(k, _)| matches(k, ty)).map(|(_, s)| *s)
}

/// From the emitter category alone (ADS-B DO-260 set A / B).
pub fn by_category(cat: &str) -> Option<Stand> {
    Some(match cat {
        "A1" => st("kestrel", 1.0, 11.0, L, "light aircraft"),
        "A2" => st("citationx", 1.0, 19.4, M, "business jet"),
        "A3" => st("meridian320", 1.0, 35.8, M, "airliner"),
        "A4" => st("meridian320", 1.06, 38.0, M, "airliner"),
        "A5" => st("triple7", 1.0, 64.8, H, "airliner"),
        "A6" => st("lancer", 1.0, 9.96, M, "military jet"),
        "A7" => st("ec130", 1.0, 10.7, L, "helicopter"),
        "B1" => st("swift", 1.0, 17.0, L, "glider"),
        "B4" => st("cub", 0.8, 8.5, L, "light aircraft"),
        // Unmanned: the quadcopter.
        "B6" => st("fpv", 1.0, 0.5, L, "drone"),
        _ => return None,
    })
}

/// The stand-in for a report: designator, then category, then a guess from
/// how it flies (fast or high → airliner, else a light aircraft).
pub fn stand_in(icao_type: &str, category: &str, gs_kt: f32, alt_ft: f32) -> Stand {
    if let Some(s) = lookup(icao_type) {
        return s;
    }
    // Rotorcraft designators we don't list: ICAO "H…" types are helicopters
    // in the type list (H25B is a business jet, matched above).
    if let Some(s) = by_category(category) {
        return s;
    }
    if gs_kt > 250.0 || alt_ft > 20_000.0 {
        st("meridian320", 1.0, 35.8, M, "airliner")
    } else if gs_kt > 160.0 {
        st("citationx", 1.0, 19.4, M, "business jet")
    } else {
        st("kestrel", 1.0, 11.0, L, "light aircraft")
    }
}

/// A short human name of the type for labels ("A320", or the stand-in's
/// class when the designator is unknown).
pub fn label(icao_type: &str, s: &Stand) -> String {
    if !icao_type.is_empty() {
        icao_type.to_string()
    } else {
        match s.spoken {
            "airliner" => "JET".into(),
            "light aircraft" => "LIGHT".into(),
            "helicopter" => "HELI".into(),
            other => other.to_ascii_uppercase(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_stand_in_is_in_the_catalogue() {
        let defs = crate::flight::catalogue();
        for (k, s) in TABLE {
            let d = defs.iter().find(|d| d.id == s.model).unwrap_or_else(|| panic!("{k}: no model {}", s.model));
            assert!(s.scale > 0.5 && s.scale < 2.1, "{k}: scale {}", s.scale);
            // The scale brings the model's span near the real one (helicopters: rotor).
            if d.rotor.is_none() && d.wingspan > 1.0 {
                let err = (d.wingspan * s.scale - s.span).abs() / s.span;
                assert!(err < 0.2, "{k}: {} × {} = {:.1} m vs {:.1} m", d.id, s.scale, d.wingspan * s.scale, s.span);
            }
        }
        for c in ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "B1", "B4", "B6"] {
            let s = by_category(c).unwrap();
            assert!(defs.iter().any(|d| d.id == s.model), "{c}");
        }
    }

    #[test]
    fn families_map_to_the_right_models() {
        let m = |t: &str| lookup(t).map(|s| s.model);
        for t in ["A318", "A319", "A320", "A321", "A20N", "A21N", "B738", "B38M", "B737", "B752"] {
            assert_eq!(m(t), Some("meridian320"), "{t}");
        }
        for t in ["B77W", "B772", "B789", "B788", "B78X", "A359", "A35K", "A333", "A339", "B744", "B748", "A388", "B763"] {
            assert_eq!(m(t), Some("triple7"), "{t}");
        }
        for t in ["E170", "E75L", "E190", "E195", "E295"] {
            assert_eq!(m(t), Some("meridian190"), "{t}");
        }
        for t in ["CRJ2", "CRJ7", "CRJ9", "CRJX"] {
            assert_eq!(m(t), Some("crj700"), "{t}");
        }
        for t in ["AT72", "AT76", "AT45", "DH8D", "DH8C"] {
            assert_eq!(m(t), Some("aerolink"), "{t}");
        }
        for t in ["C130", "C30J"] {
            assert_eq!(m(t), Some("hercules"), "{t}");
        }
        for t in ["C750", "C56X", "C25A", "GLF6", "GLEX", "CL60", "FA7X", "LJ45", "E55P"] {
            assert_eq!(m(t), Some("citationx"), "{t}");
        }
        for t in ["C172", "C152", "C182", "C208"] {
            assert_eq!(m(t), Some("kestrel"), "{t}");
        }
        for t in ["PA18", "J3"] {
            assert_eq!(m(t), Some("cub"), "{t}");
        }
        for t in ["SR22", "DA40", "PA28", "P28A", "TOBA", "PC12", "TBM9"] {
            assert_eq!(m(t), Some("extra500"), "{t}");
        }
        for t in ["EC35", "EC45", "AS50", "R44", "A139", "S92"] {
            assert_eq!(m(t), Some("ec130"), "{t}");
        }
        for t in ["F16", "EUFI", "F35", "HAWK"] {
            assert_eq!(m(t), Some("lancer"), "{t}");
        }
        // Heavies are heavy, the A380 super.
        assert_eq!(lookup("B77W").unwrap().wake, Wake::H);
        assert_eq!(lookup("A388").unwrap().wake, Wake::J);
        assert_eq!(lookup("A320").unwrap().wake, Wake::M);
        assert_eq!(lookup("C172").unwrap().wake, Wake::L);
        // Bigger stand-ins are scaled up, smaller ones down.
        assert!(lookup("A388").unwrap().scale > 1.15);
        assert!(lookup("B763").unwrap().scale < 0.85);
    }

    #[test]
    fn the_recorded_traffic_is_covered() {
        // Every designator seen in the Heathrow recording has a real entry.
        let frames = super::super::feed::parse_recording(include_str!("../../tests/adsb/egll_arrivals.json")).unwrap();
        let mut unknown = Vec::new();
        for r in &frames[0].reports {
            if !r.icao_type.is_empty() && lookup(&r.icao_type).is_none() {
                unknown.push(r.icao_type.clone());
            }
        }
        assert!(unknown.is_empty(), "unmapped: {unknown:?}");
    }

    #[test]
    fn fallbacks() {
        assert_eq!(stand_in("", "A5", 0.0, 0.0).model, "triple7");
        assert_eq!(stand_in("ZZZZ", "A7", 0.0, 0.0).model, "ec130");
        assert_eq!(stand_in("", "", 450.0, 35_000.0).model, "meridian320");
        assert_eq!(stand_in("", "", 90.0, 2_000.0).model, "kestrel");
        assert_eq!(stand_in("", "B6", 20.0, 300.0).model, "fpv");
        assert_eq!(label("A20N", &lookup("A20N").unwrap()), "A20N");
        assert_eq!(label("", &by_category("A7").unwrap()), "HELI");
    }
}
