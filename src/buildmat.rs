//! Building materials: which facade / roof texture a building gets.
//!
//! The textures are CC0 photo materials from ambientCG, packed by
//! `tools/buildtex/pack.py` into `assets/buildings/` (a 512 px albedo and a
//! 256 px normal + roughness map per layer) and bound as two texture arrays
//! (gfx.rs); scene.wgsl draws windows, floor bands and night lights over them
//! procedurally.
//!
//! **Selection**, most specific first:
//! 1. tags: Overture `facade_material` / `roof_material` / `roof_shape`,
//!    `class` / `subtype`; the OpenMapTiles `colour`, which is derived from
//!    OSM `building:material` when no `building:colour` is set (its fixed
//!    palette maps back to the material);
//! 2. use: the tags, else the landuse the building stands in (residential,
//!    commercial, industrial, institutional), else size and height (small low
//!    footprints are houses, big low ones sheds, tall ones offices / flats);
//! 3. region (latitude / longitude): red and yellow brick around the North
//!    Sea, white render and clay tiles around the Mediterranean, flat
//!    concrete roofs in South Asia, timber in Scandinavia and North America;
//! 4. a stable per-building hash picks within the weighted choices and
//!    varies the tint.
//!
//! The choice travels in the vertex material id (`mat + 64 * code`, see
//! [`Style::code_wall`]); code 0 is the old untextured look, which
//! `SKYWARD_BUILDING_TEX=0` restores everywhere.

/// Texture layers: (file stem in `assets/buildings/`, tile size in metres
/// (width, height)). Keep in step with `tools/buildtex/pack.py` and
/// `BT_SIZE` in scene.wgsl.
pub const LAYERS: [(&str, [f32; 2]); 16] = [
    ("00_brick_red", [1.05, 1.05]),
    ("01_brick_orange", [1.10, 1.10]),
    ("02_brick_yellow", [2.40, 1.20]),
    ("03_brick_brown", [2.40, 1.20]),
    ("04_brick_white", [1.05, 1.05]),
    ("05_stone", [4.00, 2.00]),
    ("06_concrete", [2.40, 2.40]),
    ("07_render", [2.00, 2.00]),
    ("08_timber", [3.00, 1.50]),
    ("09_metal", [2.00, 2.00]),
    ("10_clay_tile", [2.90, 2.90]),
    ("11_clay_pantile", [2.90, 2.90]),
    ("12_slate", [2.50, 2.50]),
    ("13_dark_tile", [2.90, 2.90]),
    ("14_gravel", [1.50, 1.50]),
    ("15_membrane", [2.50, 2.50]),
];

/// Albedo / normal map sizes (px).
pub const ALBEDO_PX: u32 = 512;
pub const NORMAL_PX: u32 = 256;

/// Layer indices.
pub mod layer {
    pub const BRICK_RED: u8 = 0;
    pub const BRICK_ORANGE: u8 = 1;
    pub const BRICK_YELLOW: u8 = 2;
    pub const BRICK_BROWN: u8 = 3;
    pub const BRICK_WHITE: u8 = 4;
    pub const STONE: u8 = 5;
    pub const CONCRETE: u8 = 6;
    pub const RENDER: u8 = 7;
    pub const TIMBER: u8 = 8;
    pub const METAL: u8 = 9;
    pub const CLAY: u8 = 10;
    pub const PANTILE: u8 = 11;
    pub const SLATE: u8 = 12;
    pub const DARK_TILE: u8 = 13;
    pub const GRAVEL: u8 = 14;
    pub const MEMBRANE: u8 = 15;
}

/// Textured buildings on (default) or the old flat-colour look
/// (`SKYWARD_BUILDING_TEX=0`). Always on in the browser.
pub fn enabled() -> bool {
    static ON: std::sync::LazyLock<bool> = std::sync::LazyLock::new(|| std::env::var("SKYWARD_BUILDING_TEX").map_or(true, |v| v.trim() != "0"));
    *ON
}

// ---------------------------------------------------------------- tags

/// What a building is used for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Use {
    #[default]
    Unknown,
    House,
    /// Flats / apartments.
    Residential,
    /// Offices, shops, hotels.
    Commercial,
    /// Factories, warehouses, sheds, farm buildings.
    Industrial,
    /// Schools, hospitals, churches, town halls, stations.
    Civic,
}

impl Use {
    pub const ALL: [Use; 6] = [Use::Unknown, Use::House, Use::Residential, Use::Commercial, Use::Industrial, Use::Civic];
}

/// Wall material named by the data.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Facade {
    #[default]
    Unknown,
    Brick,
    Stone,
    Concrete,
    Render,
    Wood,
    Metal,
    Glass,
}

impl Facade {
    pub const ALL: [Facade; 8] = [Facade::Unknown, Facade::Brick, Facade::Stone, Facade::Concrete, Facade::Render, Facade::Wood, Facade::Metal, Facade::Glass];
}

/// Roof material named by the data.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RoofMat {
    #[default]
    Unknown,
    Tiles,
    Slate,
    Metal,
    Concrete,
    Gravel,
    Membrane,
}

impl RoofMat {
    pub const ALL: [RoofMat; 7] = [RoofMat::Unknown, RoofMat::Tiles, RoofMat::Slate, RoofMat::Metal, RoofMat::Concrete, RoofMat::Gravel, RoofMat::Membrane];
}

/// Roof shape named by the data.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Shape {
    #[default]
    Unknown,
    Flat,
    Pitched,
}

impl Shape {
    pub const ALL: [Shape; 3] = [Shape::Unknown, Shape::Flat, Shape::Pitched];
}

crate::wire_enum!(Use, Use::ALL);
crate::wire_enum!(Facade, Facade::ALL);
crate::wire_enum!(RoofMat, RoofMat::ALL);
crate::wire_enum!(Shape, Shape::ALL);

/// What the map data says about a building (all optional).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tags {
    pub usage: Use,
    pub facade: Facade,
    pub roof: RoofMat,
    pub shape: Shape,
}

crate::wire_struct!(Tags { usage, facade, roof, shape });

impl Tags {
    /// From Overture's building properties (`subtype`, `class`,
    /// `facade_material`, `roof_material`, `roof_shape`; "" when absent).
    pub fn overture(subtype: &str, class: &str, facade: &str, roof: &str, shape: &str) -> Tags {
        let usage = match class {
            "house" | "detached" | "semidetached_house" | "terrace" | "bungalow" | "cabin" | "farm" | "hut" | "static_caravan" => Use::House,
            "apartments" | "dormitory" | "residential" => Use::Residential,
            "office" | "commercial" | "retail" | "supermarket" | "kiosk" | "hotel" => Use::Commercial,
            "industrial" | "warehouse" | "factory" | "manufacture" | "barn" | "cowshed" | "farm_auxiliary" | "greenhouse" | "shed" | "hangar" | "storage_tank" | "garage"
            | "garages" => Use::Industrial,
            "school" | "university" | "college" | "kindergarten" | "hospital" | "church" | "cathedral" | "chapel" | "mosque" | "temple" | "synagogue" | "civic" | "government"
            | "public" | "train_station" | "transportation" | "fire_station" | "museum" => Use::Civic,
            _ => match subtype {
                "residential" => Use::Residential,
                "commercial" | "entertainment" => Use::Commercial,
                "industrial" | "agricultural" | "outbuilding" | "service" => Use::Industrial,
                "civic" | "religious" | "education" | "medical" | "transportation" | "military" => Use::Civic,
                _ => Use::Unknown,
            },
        };
        let facade = match facade {
            "brick" => Facade::Brick,
            "stone" | "sandstone" | "limestone" => Facade::Stone,
            "concrete" | "cement_block" => Facade::Concrete,
            "plaster" | "clay" | "mud" | "timber_framing" => Facade::Render,
            "wood" => Facade::Wood,
            "metal" | "steel" | "plastic" => Facade::Metal,
            "glass" => Facade::Glass,
            _ => Facade::Unknown,
        };
        let roof = match roof {
            "roof_tiles" | "tile" | "clay" | "eternit" => RoofMat::Tiles,
            "slate" | "stone" => RoofMat::Slate,
            "metal" | "copper" | "tin" | "plastic" => RoofMat::Metal,
            "concrete" => RoofMat::Concrete,
            "gravel" | "grass" => RoofMat::Gravel,
            "tar_paper" | "asphalt" | "solar_panels" => RoofMat::Membrane,
            _ => RoofMat::Unknown,
        };
        let shape = match shape {
            "flat" => Shape::Flat,
            "" => Shape::Unknown,
            "gabled" | "hipped" | "half_hipped" | "pyramidal" | "gambrel" | "mansard" | "skillion" | "saltbox" | "side_hipped" => Shape::Pitched,
            _ => Shape::Unknown,
        };
        Tags { usage, facade, roof, shape }
    }
}

/// The wall material behind an OpenMapTiles `colour`: OpenMapTiles fills it
/// from `building:material` with a fixed palette when the building has no
/// `building:colour` (planetiler-openmaptiles Building.java).
pub fn facade_from_colour(c: [f32; 3]) -> Facade {
    const PALETTE: [(u32, Facade); 10] = [
        (0xbd8161, Facade::Brick),    // brick, masonry, traditional
        (0xdadbdb, Facade::Render),   // plaster
        (0xd48741, Facade::Wood),     // wood
        (0xd3c2b0, Facade::Concrete), // concrete
        (0xb7b1a6, Facade::Metal),    // metal, steel, tin
        (0xb4a995, Facade::Stone),    // stone, sandstone
        (0x9d8b75, Facade::Render),   // mud, clay
        (0x5a81a0, Facade::Glass),    // glass
        (0x6a7880, Facade::Concrete), // cement_block
        (0xb3b0a9, Facade::Render),   // timber_framing
    ];
    let q = |v: f32| (v * 255.0).round().clamp(0.0, 255.0) as u32;
    let hex = q(c[0]) << 16 | q(c[1]) << 8 | q(c[2]);
    PALETTE.iter().find(|(h, _)| *h == hex).map_or(Facade::Unknown, |(_, f)| *f)
}

// ---------------------------------------------------------------- region

/// Building traditions, roughly by latitude / longitude.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Region {
    /// British Isles, Low Countries, northern Germany, Denmark: brick.
    NorthSea,
    /// Norway, Sweden, Finland: painted timber, render.
    Nordic,
    /// Iberia, southern France, Italy, Greece, Turkey, the Levant: white /
    /// ochre render, clay tiles, flat roofs.
    Mediterranean,
    /// North Africa, the Middle East: sand-coloured render, flat roofs.
    Arid,
    /// USA, Canada: timber siding houses, brick and concrete downtowns.
    NorthAmerica,
    /// Indian subcontinent: rendered / bare concrete, flat roofs.
    SouthAsia,
    /// East and South-East Asia: concrete, tiles.
    EastAsia,
    /// Everywhere else (central / eastern Europe, ...): render, brick, tiles.
    Other,
}

impl Region {
    pub fn at(lat: f64, lon: f64) -> Region {
        let (la, lo) = (lat, lon);
        if (-170.0..-50.0).contains(&lo) && (15.0..72.0).contains(&la) {
            Region::NorthAmerica
        } else if (57.5..72.0).contains(&la) && (4.0..32.0).contains(&lo) {
            Region::Nordic
        } else if ((49.8..61.0).contains(&la) && (-11.0..2.0).contains(&lo)) || ((50.5..57.5).contains(&la) && (2.0..14.5).contains(&lo)) {
            Region::NorthSea
        } else if (35.0..45.5).contains(&la) && (-10.0..37.0).contains(&lo) {
            Region::Mediterranean
        } else if (12.0..37.0).contains(&la) && (-18.0..62.0).contains(&lo) {
            Region::Arid
        } else if (5.0..36.0).contains(&la) && (62.0..98.0).contains(&lo) {
            Region::SouthAsia
        } else if (-11.0..50.0).contains(&la) && (98.0..150.0).contains(&lo) {
            Region::EastAsia
        } else {
            Region::Other
        }
    }
}

// ---------------------------------------------------------------- style

/// Facade layout drawn by the shader over the material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Windows {
    /// Flats / mixed: punched windows on a ~3 m storey grid (shopfronts on
    /// some ground floors).
    Punched = 0,
    /// Houses: fewer, smaller windows, low storeys.
    House = 1,
    /// Offices: ribbon windows with mullions.
    Ribbon = 2,
    /// Sheds / factories: a clerestory strip, doors, few lights at night.
    Industrial = 3,
    /// Schools, churches, town halls: tall windows, high storeys.
    Civic = 4,
}

/// Everything the mesher needs to draw one building.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Style {
    /// Wall layer, or `None` for a curtain-wall glass tower (mat::GLASS).
    pub wall: Option<u8>,
    pub windows: Windows,
    /// Tint applied to the wall texture (sRGB-encoded multiplier, 1 = as is).
    pub wall_tint: [f32; 3],
    pub roof: u8,
    pub roof_tint: [f32; 3],
    /// Roof shape: pitched where the footprint allows one, else flat.
    pub pitched: bool,
    /// Ground-floor shopfronts allowed.
    pub shops: bool,
}

impl Style {
    /// Material code for the wall vertices (0 = legacy).
    pub fn code_wall(&self) -> u32 {
        self.wall.map_or(0, |l| code(l, self.windows))
    }
    /// Material code for the roof vertices.
    pub fn code_roof(&self) -> u32 {
        code(self.roof, Windows::Punched)
    }
}

/// `layer + 1` in bits 0..5, the window layout in bits 5..8.
pub fn code(layer: u8, w: Windows) -> u32 {
    (layer as u32 + 1) | (w as u32) << 5
}

/// Vertex material id carrying a code: `base + 64 * code` (scene.wgsl
/// splits it again; exact in f32).
pub fn mat_with(base: f32, code: u32) -> f32 {
    base + 64.0 * code as f32
}

/// What is known about one building.
#[derive(Clone, Copy, Debug)]
pub struct Input {
    pub height: f32,
    /// Footprint area (m²).
    pub area: f32,
    /// OpenMapTiles `colour` (sRGB 0..1).
    pub colour: Option<[f32; 3]>,
    pub tags: Tags,
    /// Landuse class under the building (osm::Kind::Urban).
    pub landuse: Option<u8>,
    pub region: Region,
    pub key: u64,
    /// The footprint suits a pitched roof (scene::pitched_plan).
    pub can_pitch: bool,
}

/// Deterministic 0..1 values from the building key.
struct Dice(u64);

impl Dice {
    fn f(&mut self) -> f32 {
        self.0 = crate::scene::hash(self.0.wrapping_add(0x9e37_79b9_7f4a_7c15));
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
    /// Weighted pick.
    fn pick<T: Copy>(&mut self, opts: &[(f32, T)]) -> T {
        let total: f32 = opts.iter().map(|o| o.0).sum();
        let mut x = self.f() * total;
        for o in opts {
            if x < o.0 {
                return o.1;
            }
            x -= o.0;
        }
        opts[opts.len() - 1].1
    }
}

/// The building's use: tags, else landuse, else its size.
pub fn usage(i: &Input) -> Use {
    let u = match i.tags.usage {
        Use::Unknown => match i.landuse {
            Some(0) => Use::Residential,
            Some(1) => Use::Commercial,
            Some(2) => Use::Industrial,
            Some(3) => Use::Civic,
            _ => Use::Unknown,
        },
        u => u,
    };
    match u {
        Use::Unknown | Use::Residential if i.height <= 12.5 && i.area <= 320.0 => Use::House,
        Use::Unknown if i.height <= 16.0 && i.area >= 2500.0 => Use::Industrial,
        Use::Unknown if i.height > 30.0 => Use::Commercial,
        Use::Unknown => Use::Residential,
        u => u,
    }
}

fn scale(c: [f32; 3], k: f32) -> [f32; 3] {
    [c[0] * k, c[1] * k, c[2] * k]
}

const WHITE: [f32; 3] = [1.0, 1.0, 1.0];

/// Paint colours for render (multipliers on the near-white plaster).
fn render_tint(d: &mut Dice, r: Region) -> [f32; 3] {
    let opts: &[(f32, [f32; 3])] = match r {
        Region::Mediterranean => &[(5.0, [1.04, 1.03, 1.0]), (2.0, [1.02, 0.93, 0.78]), (1.5, [1.0, 0.82, 0.6]), (0.7, [1.0, 0.78, 0.7]), (0.5, [0.86, 0.92, 0.98])],
        Region::Arid => &[(3.0, [0.96, 0.88, 0.74]), (2.0, [0.9, 0.8, 0.66]), (1.5, [1.03, 1.0, 0.95])],
        Region::SouthAsia => &[(3.0, [1.0, 0.98, 0.94]), (1.5, [0.97, 0.92, 0.8]), (1.0, [0.92, 0.95, 0.88]), (1.0, [1.0, 0.86, 0.8]), (0.8, [0.86, 0.9, 0.98])],
        Region::NorthSea => &[(5.0, [1.03, 1.02, 0.98]), (2.0, [1.0, 0.96, 0.86]), (0.6, [0.9, 0.93, 0.95])],
        _ => &[(4.0, [1.03, 1.02, 0.98]), (2.0, [1.0, 0.94, 0.8]), (1.0, [0.95, 0.9, 0.82]), (0.7, [0.98, 0.86, 0.76]), (0.5, [0.88, 0.92, 0.9])],
    };
    scale(d.pick(opts), 0.94 + 0.08 * d.f())
}

/// Paint colours for timber siding (on light, yellowish raw boards).
fn timber_tint(d: &mut Dice, r: Region) -> [f32; 3] {
    let opts: &[(f32, [f32; 3])] = match r {
        // Falu red, ochre, white, grey-green.
        Region::Nordic => &[(4.0, [0.68, 0.32, 0.26]), (2.0, [1.0, 0.86, 0.6]), (2.0, [1.08, 1.08, 1.14]), (1.0, [0.72, 0.8, 0.74])],
        _ => &[(3.0, [1.08, 1.08, 1.14]), (2.0, [0.86, 0.88, 0.92]), (1.0, [0.74, 0.84, 0.98]), (1.0, [0.92, 0.9, 0.78]), (1.0, [0.78, 0.86, 0.8])],
    };
    scale(d.pick(opts), 0.92 + 0.1 * d.f())
}

/// Cladding colours (on grey galvanised sheet).
fn metal_tint(d: &mut Dice) -> [f32; 3] {
    let c = d.pick(&[(4.0, [1.05, 1.05, 1.05]), (2.0, [1.25, 1.25, 1.22]), (1.5, [0.8, 0.9, 1.05]), (1.0, [0.78, 0.92, 0.8]), (1.0, [0.6, 0.6, 0.62]), (0.6, [0.9, 0.6, 0.55])]);
    scale(c, 0.92 + 0.12 * d.f())
}

/// Slight per-building variation of a natural material.
fn natural(d: &mut Dice) -> [f32; 3] {
    let k = 0.9 + 0.16 * d.f();
    [k * (0.98 + 0.04 * d.f()), k, k * (0.98 + 0.04 * d.f())]
}

/// Brick of the region (and era: yellow London stock, red, brown engineering
/// brick, painted white).
fn brick(d: &mut Dice, r: Region) -> u8 {
    use layer::*;
    match r {
        Region::NorthSea => d.pick(&[(5.0, BRICK_RED), (2.5, BRICK_YELLOW), (2.0, BRICK_BROWN), (1.5, BRICK_ORANGE), (0.6, BRICK_WHITE)]),
        Region::NorthAmerica => d.pick(&[(5.0, BRICK_RED), (2.0, BRICK_BROWN), (1.5, BRICK_ORANGE), (1.0, BRICK_YELLOW), (0.5, BRICK_WHITE)]),
        Region::Mediterranean | Region::Arid => d.pick(&[(3.0, BRICK_ORANGE), (2.0, BRICK_YELLOW), (1.0, BRICK_RED)]),
        _ => d.pick(&[(3.0, BRICK_RED), (2.0, BRICK_ORANGE), (1.0, BRICK_BROWN), (1.0, BRICK_YELLOW)]),
    }
}

/// A wall layer from a named facade material.
fn wall_for(f: Facade, d: &mut Dice, r: Region) -> Option<u8> {
    use layer::*;
    Some(match f {
        Facade::Brick => brick(d, r),
        Facade::Stone => STONE,
        Facade::Concrete => CONCRETE,
        Facade::Render => RENDER,
        Facade::Wood => TIMBER,
        Facade::Metal => METAL,
        Facade::Glass | Facade::Unknown => return None,
    })
}

/// Choose the style of one building.
pub fn select(i: &Input) -> Style {
    use layer::*;
    let mut d = Dice(i.key ^ 0xb7e1_5163_a5a5_0f0f);
    let r = i.region;
    let u = usage(i);
    let h = i.height;
    // Named material (Overture tag, else the OpenMapTiles colour palette).
    let named = match i.tags.facade {
        Facade::Unknown => i.colour.map_or(Facade::Unknown, facade_from_colour),
        f => f,
    };
    let mut glass = named == Facade::Glass;
    let mut wall = wall_for(named, &mut d, r);
    if wall.is_none() && !glass {
        // Tall towers: mostly curtain walls; some precast concrete.
        if h > 45.0 {
            let p_glass = match u {
                Use::Residential | Use::House => 0.4,
                Use::Industrial => 0.2,
                _ => 0.75,
            };
            glass = d.f() < p_glass;
            wall = Some(d.pick(&[(4.0, CONCRETE), (1.0, RENDER), (if r == Region::NorthSea || r == Region::NorthAmerica { 1.5 } else { 0.3 }, BRICK_BROWN)]));
        } else {
            let br = brick(&mut d, r);
            let opts: Vec<(f32, u8)> = match (u, r) {
                (Use::Industrial, Region::NorthSea) => vec![(5.0, METAL), (2.0, CONCRETE), (2.0, br)],
                (Use::Industrial, Region::SouthAsia | Region::Arid) => vec![(3.0, CONCRETE), (2.0, METAL), (1.0, RENDER)],
                (Use::Industrial, _) => vec![(6.0, METAL), (3.0, CONCRETE), (1.0, br)],
                (Use::House, Region::NorthSea) => vec![(6.0, br), (2.5, RENDER), (0.4, TIMBER), (0.3, STONE)],
                (Use::House, Region::Nordic) => vec![(6.0, TIMBER), (3.0, RENDER), (1.0, br)],
                (Use::House, Region::NorthAmerica) => vec![(5.0, TIMBER), (3.0, br), (2.0, RENDER), (0.3, STONE)],
                (Use::House, Region::Mediterranean) => vec![(8.0, RENDER), (1.5, STONE), (0.5, br)],
                (Use::House, Region::Arid) => vec![(8.0, RENDER), (1.5, CONCRETE), (0.5, STONE)],
                (Use::House, Region::SouthAsia) => vec![(5.0, RENDER), (3.0, CONCRETE), (2.0, br)],
                (Use::House, Region::EastAsia) => vec![(4.0, RENDER), (4.0, CONCRETE), (1.0, br), (1.0, TIMBER)],
                (Use::House, Region::Other) => vec![(6.0, RENDER), (2.5, br), (0.8, TIMBER), (0.7, STONE)],
                (Use::Civic, Region::NorthSea) => vec![(4.0, br), (3.0, STONE), (2.0, CONCRETE), (1.0, RENDER)],
                (Use::Civic, _) => vec![(3.0, RENDER), (3.0, STONE), (2.0, CONCRETE), (1.5, br)],
                (Use::Commercial, Region::NorthSea) => vec![(4.0, br), (3.0, CONCRETE), (1.5, STONE), (1.5, RENDER)],
                (Use::Commercial, Region::NorthAmerica) => vec![(4.0, br), (4.0, CONCRETE), (1.0, STONE), (1.0, RENDER)],
                (Use::Commercial, Region::Mediterranean | Region::Arid) => vec![(5.0, RENDER), (3.0, CONCRETE), (1.0, STONE)],
                (Use::Commercial, _) => vec![(4.0, CONCRETE), (4.0, RENDER), (1.5, br), (0.5, STONE)],
                // Flats.
                (_, Region::NorthSea) => vec![(5.0, br), (2.5, CONCRETE), (2.0, RENDER), (0.3, STONE)],
                (_, Region::NorthAmerica) => vec![(5.0, br), (2.0, CONCRETE), (2.0, RENDER), (1.0, TIMBER)],
                (_, Region::Mediterranean | Region::Arid) => vec![(7.0, RENDER), (2.0, CONCRETE), (0.5, STONE)],
                (_, Region::SouthAsia | Region::EastAsia) => vec![(5.0, RENDER), (4.0, CONCRETE), (0.5, br)],
                (_, Region::Nordic) => vec![(4.0, RENDER), (3.0, br), (2.0, CONCRETE)],
                (_, Region::Other) => vec![(5.0, RENDER), (3.0, CONCRETE), (2.0, br)],
            };
            wall = Some(d.pick(&opts));
        }
    }
    // A colour that is not a palette material: a painted / tinted facade.
    let own = i.colour.filter(|c| facade_from_colour(*c) == Facade::Unknown);
    let wall_tint = match (wall, own) {
        (Some(RENDER | TIMBER | METAL | CONCRETE), Some(c)) => {
            let m = (c[0] + c[1] + c[2]) / 3.0;
            let k = 0.5;
            [(m + (c[0] - m) * k) / m.max(0.2), (m + (c[1] - m) * k) / m.max(0.2), (m + (c[2] - m) * k) / m.max(0.2)]
        }
        (Some(RENDER), None) => render_tint(&mut d, r),
        (Some(TIMBER), None) => timber_tint(&mut d, r),
        (Some(METAL), None) => metal_tint(&mut d),
        // Precast / cast concrete: from pale limestone-like to weathered dark.
        (Some(CONCRETE), None) => scale(d.pick(&[(3.0, [0.95, 0.95, 0.97]), (2.0, [1.0, 0.94, 0.84]), (2.0, [0.72, 0.72, 0.74]), (1.0, [0.55, 0.56, 0.58])]), 0.9 + 0.15 * d.f()),
        _ => natural(&mut d),
    };
    let windows = match u {
        Use::House => Windows::House,
        Use::Industrial => Windows::Industrial,
        Use::Civic => Windows::Civic,
        Use::Commercial if h > 15.0 => Windows::Ribbon,
        Use::Residential if h > 45.0 && wall == Some(CONCRETE) => Windows::Ribbon,
        _ => Windows::Punched,
    };
    // Roof shape: data, else what the footprint allows, less often where
    // flat roofs are the custom.
    let flat_share = match (r, u) {
        (_, Use::Industrial) => 0.5,
        (Region::SouthAsia | Region::Arid, _) => 0.92,
        (Region::Mediterranean, _) => 0.45,
        (Region::EastAsia, _) => 0.55,
        _ => 0.0,
    };
    let pitched = i.can_pitch
        && match i.tags.shape {
            Shape::Flat => false,
            Shape::Pitched => true,
            Shape::Unknown => !matches!(i.tags.roof, RoofMat::Gravel | RoofMat::Membrane) && d.f() >= flat_share,
        };
    let roof = match i.tags.roof {
        RoofMat::Tiles if pitched => d.pick(&[(2.0, CLAY), (1.0, PANTILE), (1.0, DARK_TILE)]),
        RoofMat::Slate if pitched => SLATE,
        RoofMat::Metal => METAL,
        RoofMat::Concrete => CONCRETE,
        RoofMat::Gravel => GRAVEL,
        RoofMat::Membrane => MEMBRANE,
        _ if pitched => match r {
            Region::NorthSea => d.pick(&[(4.0, SLATE), (4.0, CLAY), (2.0, DARK_TILE), (1.0, PANTILE)]),
            Region::Nordic => d.pick(&[(3.0, CLAY), (3.0, DARK_TILE), (3.0, METAL)]),
            Region::NorthAmerica => d.pick(&[(5.0, DARK_TILE), (2.0, SLATE), (2.0, METAL), (0.5, CLAY)]),
            Region::Mediterranean | Region::Arid => d.pick(&[(5.0, PANTILE), (4.0, CLAY)]),
            Region::SouthAsia | Region::EastAsia => d.pick(&[(3.0, METAL), (3.0, CLAY), (2.0, DARK_TILE)]),
            Region::Other => d.pick(&[(5.0, CLAY), (2.0, DARK_TILE), (1.0, SLATE), (1.0, PANTILE), (1.0, METAL)]),
        },
        _ => match (u, r) {
            (Use::Industrial, _) => d.pick(&[(5.0, METAL), (3.0, MEMBRANE), (1.0, GRAVEL)]),
            (_, Region::SouthAsia | Region::Arid) => d.pick(&[(7.0, CONCRETE), (2.0, MEMBRANE)]),
            (_, Region::Mediterranean) => d.pick(&[(4.0, CONCRETE), (3.0, MEMBRANE), (2.0, GRAVEL)]),
            _ => d.pick(&[(5.0, MEMBRANE), (4.0, GRAVEL), (1.0, CONCRETE)]),
        },
    };
    let roof_tint = match roof {
        METAL if pitched => scale(d.pick(&[(3.0, [0.9, 0.9, 0.92]), (2.0, [0.75, 0.42, 0.36]), (1.5, [0.5, 0.62, 0.52]), (1.5, [0.42, 0.42, 0.46])]), 0.9 + 0.15 * d.f()),
        METAL => metal_tint(&mut d),
        // Bitumen felt to pale grey single-ply / reflective coatings.
        MEMBRANE => scale(WHITE, 1.2 + 1.0 * d.f()),
        CONCRETE => scale(d.pick(&[(3.0, [0.95, 0.95, 0.95]), (1.0, [1.0, 0.95, 0.86])]), 0.85 + 0.2 * d.f()),
        _ => natural(&mut d),
    };
    Style { wall: if glass { None } else { wall }, windows, wall_tint, roof, roof_tint, pitched, shops: matches!(u, Use::Residential | Use::Commercial | Use::Unknown) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(height: f32, area: f32, region: Region) -> Input {
        Input { height, area, colour: None, tags: Tags::default(), landuse: None, region, key: 1, can_pitch: true }
    }

    /// Share of 400 keys whose style satisfies `f`.
    fn share(base: Input, f: impl Fn(&Style) -> bool) -> f32 {
        (0..400u64).filter(|k| f(&select(&Input { key: crate::scene::hash(*k), ..base }))).count() as f32 / 400.0
    }

    #[test]
    fn openmaptiles_material_colours_map_back() {
        let c = |hex: u32| [((hex >> 16) & 255) as f32 / 255.0, ((hex >> 8) & 255) as f32 / 255.0, (hex & 255) as f32 / 255.0];
        assert_eq!(facade_from_colour(c(0xbd8161)), Facade::Brick);
        assert_eq!(facade_from_colour(c(0xdadbdb)), Facade::Render);
        assert_eq!(facade_from_colour(c(0xd48741)), Facade::Wood);
        assert_eq!(facade_from_colour(c(0xb7b1a6)), Facade::Metal);
        assert_eq!(facade_from_colour(c(0x5a81a0)), Facade::Glass);
        assert_eq!(facade_from_colour(c(0x6a7880)), Facade::Concrete);
        assert_eq!(facade_from_colour(c(0x123456)), Facade::Unknown);
    }

    #[test]
    fn tagged_materials_win() {
        let mut i = input(10.0, 150.0, Region::Mediterranean);
        i.tags.facade = Facade::Metal;
        assert_eq!(select(&i).wall, Some(layer::METAL));
        i.tags.facade = Facade::Glass;
        assert_eq!(select(&i).wall, None);
        // An OSM brick building in the Mediterranean is still brick.
        let mut j = input(10.0, 150.0, Region::Mediterranean);
        j.colour = Some([0xbd as f32 / 255.0, 0x81 as f32 / 255.0, 0x61 as f32 / 255.0]);
        assert!(share(j, |s| matches!(s.wall, Some(0..=4))) == 1.0);
        // Roof tags.
        let mut k = input(8.0, 120.0, Region::NorthSea);
        k.tags.roof = RoofMat::Slate;
        k.tags.shape = Shape::Pitched;
        let s = select(&k);
        assert!(s.pitched);
        assert_eq!(s.roof, layer::SLATE);
        k.tags.shape = Shape::Flat;
        assert!(!select(&k).pitched);
    }

    #[test]
    fn overture_tags_parse() {
        let t = Tags::overture("residential", "house", "brick", "roof_tiles", "gabled");
        assert_eq!(t, Tags { usage: Use::House, facade: Facade::Brick, roof: RoofMat::Tiles, shape: Shape::Pitched });
        let t = Tags::overture("industrial", "", "", "", "flat");
        assert_eq!((t.usage, t.shape), (Use::Industrial, Shape::Flat));
        assert_eq!(Tags::overture("", "", "", "", ""), Tags::default());
    }

    #[test]
    fn use_from_landuse_and_size() {
        let mut i = input(8.0, 150.0, Region::Other);
        assert_eq!(usage(&i), Use::House);
        i.area = 5000.0;
        assert_eq!(usage(&i), Use::Industrial);
        i.landuse = Some(1);
        assert_eq!(usage(&i), Use::Commercial);
        i.landuse = Some(2);
        i.area = 150.0;
        assert_eq!(usage(&i), Use::Industrial);
        let tall = input(80.0, 900.0, Region::Other);
        assert_eq!(usage(&tall), Use::Commercial);
    }

    #[test]
    fn regional_flavour() {
        // UK houses: mostly brick; Mediterranean houses: mostly render with
        // clay roofs or flat; South Asia: flat roofs.
        let uk = input(8.0, 120.0, Region::NorthSea);
        assert!(share(uk, |s| matches!(s.wall, Some(0..=4))) > 0.55);
        assert!(share(uk, |s| s.pitched) > 0.95);
        let med = input(8.0, 120.0, Region::Mediterranean);
        assert!(share(med, |s| s.wall == Some(layer::RENDER)) > 0.65);
        assert!(share(med, |s| !s.pitched || matches!(s.roof, layer::CLAY | layer::PANTILE)) == 1.0);
        let bihar = input(9.0, 120.0, Region::SouthAsia);
        assert!(share(bihar, |s| !s.pitched) > 0.8);
        let nordic = input(7.0, 120.0, Region::Nordic);
        assert!(share(nordic, |s| s.wall == Some(layer::TIMBER)) > 0.45);
        // Industrial estates: cladding; towers: mostly glass.
        let mut shed = input(9.0, 6000.0, Region::NorthSea);
        shed.landuse = Some(2);
        assert!(share(shed, |s| s.wall == Some(layer::METAL)) > 0.4);
        assert!(share(shed, |s| s.windows == Windows::Industrial) == 1.0);
        let tower = input(120.0, 1500.0, Region::NorthSea);
        let g = share(tower, |s| s.wall.is_none());
        assert!(g > 0.6 && g < 0.9, "{g}");
    }

    #[test]
    fn regions() {
        assert_eq!(Region::at(51.5, -0.1), Region::NorthSea);
        assert_eq!(Region::at(52.37, 4.9), Region::NorthSea);
        assert_eq!(Region::at(38.9, 1.43), Region::Mediterranean);
        assert_eq!(Region::at(40.75, -73.99), Region::NorthAmerica);
        assert_eq!(Region::at(25.6, 85.1), Region::SouthAsia);
        assert_eq!(Region::at(59.33, 18.07), Region::Nordic);
        assert_eq!(Region::at(48.14, 11.58), Region::Other);
        assert_eq!(Region::at(25.2, 55.3), Region::Arid);
    }

    #[test]
    fn stable_and_encodable() {
        let i = input(14.0, 400.0, Region::Other);
        assert_eq!(select(&i), select(&i));
        for k in 0..200u64 {
            let s = select(&Input { key: k, ..i });
            for c in [s.code_wall(), s.code_roof()] {
                assert!(c < 256);
                let m = mat_with(2.0, c);
                assert_eq!(((m + 0.5) as i32) & 63, 2);
                assert_eq!(((m + 0.5) as i32) >> 6, c as i32);
            }
        }
    }

    #[test]
    fn layer_sizes_match_the_shader() {
        let wgsl = include_str!("shaders/scene.wgsl");
        for (i, (_, s)) in LAYERS.iter().enumerate() {
            let want = format!("vec2<f32>({:.2}, {:.2}), // {i}", s[0], s[1]);
            assert!(wgsl.contains(&want), "scene.wgsl BT_SIZE lacks {want}");
        }
    }
}
