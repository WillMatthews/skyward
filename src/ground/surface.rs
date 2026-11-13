//! What a tyre rolls on: the pavement or field type ([`Ground`]), its
//! contamination from the weather ([`Contam`]: damp, wet with a water depth,
//! slush, snow, ice) and the friction that follows — the tyre's peak and
//! locked-wheel friction coefficients, rolling resistance, contaminant drag,
//! and dynamic hydroplaning above Horne's speed (9·√p kt spinning, 7.7·√p kt
//! for a wheel that has stopped turning).
//!
//! Figures: ESDU 71026 / FAA AC 25-7 / AC 91-79 (wet / contaminated runway
//! braking), NASA TN D-2056 (Horne, hydroplaning), Boeing / Airbus runway
//! condition codes (RCAM: dry 6 .. wet ice 0).

use glam::Vec2;

/// Base surface.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Ground {
    #[default]
    Asphalt,
    Concrete,
    /// Mown grass / turf (strips, the infield).
    Grass,
    Gravel,
    /// Dirt, sand, clay.
    Dirt,
    /// Prepared snow runway / frozen lake.
    Snow,
    /// Open water (floats only; wheels sink: handled by the flight model).
    Water,
}

impl Ground {
    pub fn paved(self) -> bool {
        matches!(self, Ground::Asphalt | Ground::Concrete)
    }
    /// From an OurAirports surface name (`airports::Surface::name`) or free text.
    pub fn from_name(s: &str) -> Ground {
        match crate::airports::Surface::parse(s) {
            crate::airports::Surface::Concrete => Ground::Concrete,
            crate::airports::Surface::Grass => Ground::Grass,
            crate::airports::Surface::Gravel => Ground::Gravel,
            crate::airports::Surface::Dirt => Ground::Dirt,
            crate::airports::Surface::Snow => Ground::Snow,
            crate::airports::Surface::Water => Ground::Water,
            _ => Ground::Asphalt,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Ground::Asphalt => "asphalt",
            Ground::Concrete => "concrete",
            Ground::Grass => "grass",
            Ground::Gravel => "gravel",
            Ground::Dirt => "dirt",
            Ground::Snow => "snow",
            Ground::Water => "water",
        }
    }
}

/// Contamination on top of the surface.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Contam {
    #[default]
    Dry,
    /// Damp (visible moisture, no standing water).
    Damp,
    /// Wet: water depth in mm (≤ 3 mm "wet", more is standing water).
    Wet(f32),
    /// Slush, depth mm.
    Slush(f32),
    /// Dry or compacted snow, depth mm.
    Snow(f32),
    /// Ice (`wet`: at or above 0 °C / with water on it).
    Ice { wet: bool },
}

impl Contam {
    /// Standing water / slush depth (mm) that a tyre has to displace.
    pub fn fluid_mm(self) -> f32 {
        match self {
            Contam::Wet(d) | Contam::Slush(d) => d,
            _ => 0.0,
        }
    }
    /// Fluid density (kg/m³) for displacement drag and hydroplaning.
    pub fn fluid_density(self) -> f32 {
        match self {
            Contam::Slush(_) => 850.0,
            _ => 1000.0,
        }
    }
    pub fn label(self) -> String {
        match self {
            Contam::Dry => "dry".into(),
            Contam::Damp => "damp".into(),
            Contam::Wet(d) if d > 3.0 => format!("standing water {d:.0} mm"),
            Contam::Wet(_) => "wet".into(),
            Contam::Slush(d) => format!("slush {d:.0} mm"),
            Contam::Snow(d) => format!("snow {d:.0} mm"),
            Contam::Ice { wet: true } => "wet ice".into(),
            Contam::Ice { wet: false } => "ice".into(),
        }
    }
    /// ICAO runway condition code (RCAM): 6 dry .. 0 wet ice.
    pub fn rwycc(self) -> u8 {
        match self {
            Contam::Dry => 6,
            Contam::Damp => 5,
            Contam::Wet(d) if d <= 3.0 => 5,
            Contam::Wet(_) | Contam::Slush(_) => 2,
            Contam::Snow(d) if d < 3.0 => 5,
            Contam::Snow(_) => 3,
            Contam::Ice { wet: false } => 1,
            Contam::Ice { wet: true } => 0,
        }
    }
    /// Parse "dry", "damp", "wet", "wet:6", "flooded", "slush:10", "snow:30", "ice", "wet-ice".
    pub fn parse(s: &str) -> Option<Contam> {
        let (k, d) = s.split_once(':').map(|(a, b)| (a, b.parse::<f32>().ok())).unwrap_or((s, None));
        Some(match k.trim().to_ascii_lowercase().as_str() {
            "dry" => Contam::Dry,
            "damp" => Contam::Damp,
            "wet" => Contam::Wet(d.unwrap_or(1.5)),
            "flooded" | "standing" | "water" => Contam::Wet(d.unwrap_or(8.0)),
            "slush" => Contam::Slush(d.unwrap_or(10.0)),
            "snow" => Contam::Snow(d.unwrap_or(30.0)),
            "ice" => Contam::Ice { wet: false },
            "wet-ice" | "wetice" => Contam::Ice { wet: true },
            _ => return None,
        })
    }
}

/// The surface under one wheel.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Surface {
    pub ground: Ground,
    pub contam: Contam,
}

impl Surface {
    /// Dry asphalt (what the flight model assumes without a world).
    pub const DRY: Surface = Surface { ground: Ground::Asphalt, contam: Contam::Dry };

    pub fn new(ground: Ground, contam: Contam) -> Surface {
        Surface { ground, contam }
    }

    pub fn paved(&self) -> bool {
        self.ground.paved()
    }

    pub fn label(&self) -> String {
        match self.contam {
            Contam::Dry => self.ground.name().to_string(),
            c => format!("{} ({})", self.ground.name(), c.label()),
        }
    }

    /// Dry peak friction of the bare surface (a good aircraft tyre).
    fn mu_dry(&self) -> f32 {
        match self.ground {
            Ground::Asphalt => 0.85,
            Ground::Concrete => 0.80,
            Ground::Grass => 0.45,
            Ground::Gravel => 0.55,
            Ground::Dirt => 0.55,
            Ground::Snow => 0.28,
            Ground::Water => 0.05,
        }
    }

    /// Rolling resistance multiplier on the tyre's own (paved) figure, plus an
    /// additive part for soft ground (sinkage).
    pub fn rolling(&self) -> (f32, f32) {
        let (m, add) = match self.ground {
            Ground::Asphalt | Ground::Concrete => (1.0, 0.0),
            Ground::Grass => (1.0, 0.035),
            Ground::Gravel => (1.0, 0.02),
            Ground::Dirt => (1.0, 0.03),
            Ground::Snow => (1.0, 0.05),
            Ground::Water => (1.0, 0.2),
        };
        let c = match self.contam {
            Contam::Snow(d) => (d * 0.0015).min(0.12),
            _ => 0.0,
        };
        (m, add + c)
    }

    /// Peak (best slip) friction coefficient at ground speed `v` (m/s) for a
    /// tyre inflated to `psi`; `spinning`: the wheel is turning (a wheel that
    /// has stopped hydroplanes at a lower speed and stays planing).
    /// Returns (peak μ, locked-wheel μ, hydroplaning 0..1).
    pub fn friction(&self, v: f32, psi: f32, spinning: bool) -> (f32, f32, f32) {
        let v = v.abs();
        let kt = v / crate::flight::KT;
        let dry = self.mu_dry();
        // Locked wheels slide at 70–80 % of the peak on dry pavement.
        let (mut peak, mut slide_k) = (dry, 0.75);
        let soft = !self.ground.paved();
        let vhp = hydroplane_kt(psi, spinning);
        let mut hp = 0.0;
        match self.contam {
            Contam::Dry => {
                // A slight fall with speed (rubber heating).
                peak *= 1.0 - 0.10 * (kt / 150.0).min(1.2);
            }
            Contam::Damp => {
                peak *= 0.85 - 0.12 * (kt / 150.0).min(1.2);
                slide_k = 0.6;
            }
            Contam::Wet(d) => {
                // Viscous hydroplaning: the thin film is squeezed out ever less
                // with speed (ESDU 71026: ~0.5 μdry at low speed falling to
                // ~0.2–0.3 at 120 kt on a wet grooved runway; worse ungrooved).
                let x = (kt / vhp.max(20.0)).min(1.4);
                let base = if soft { 0.55 } else { 0.72 };
                peak *= base * (1.0 - 0.55 * x).max(0.18);
                // Locked wheels on a wet runway: reverted-rubber skids, very low.
                slide_k = 0.45;
                // Dynamic hydroplaning once the water is deeper than the tread
                // can drain (~2.5 mm), above Horne's speed.
                let depth = smoothstep(1.5, 3.0, d);
                hp = depth * smoothstep(0.9 * vhp, 1.02 * vhp, kt);
            }
            Contam::Slush(d) => {
                peak = 0.25 * (1.0 - 0.3 * (kt / 120.0).min(1.0));
                slide_k = 0.6;
                let depth = smoothstep(2.0, 6.0, d);
                // Slush is denser: it plans the tyre a little earlier.
                hp = depth * smoothstep(0.85 * vhp, 0.98 * vhp, kt);
            }
            Contam::Snow(d) => {
                peak = if d < 3.0 { 0.35 } else { 0.28 } * (1.0 - 0.15 * (kt / 120.0).min(1.0));
                slide_k = 0.75;
            }
            Contam::Ice { wet } => {
                peak = if wet { 0.05 } else { 0.10 };
                slide_k = 0.8;
            }
        }
        // Planing tyre: only the air / water shear is left.
        let planed = 0.05;
        let peak = peak * (1.0 - hp) + planed * hp;
        (peak, (peak * slide_k).min(peak), hp)
    }
}

/// Horne's dynamic hydroplaning speed (kt) for a tyre at `psi`: 9·√p spinning
/// down from a rolling tyre (take-off / rollout), 7.7·√p for a tyre that is
/// not turning (touchdown on a locked / spun-down wheel).
pub fn hydroplane_kt(psi: f32, spinning: bool) -> f32 {
    (if spinning { 9.0 } else { 7.7 }) * psi.max(1.0).sqrt()
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The runway contamination the weather leaves (from the METAR / weather state).
///
/// `precip`: falling precipitation kind, `intensity` 0..1, `recent`: recent
/// rain (RERA), `temp_c`, `snow_depth_m` on the ground. Runways are swept /
/// cleared: lying snow becomes compacted snow or slush on the runway.
pub fn from_weather(wx: &crate::weather::Wx) -> Contam {
    use crate::weather::Precip;
    let t = wx.temp_c;
    let i = wx.intensity.clamp(0.0, 1.0);
    let freezing = t <= 0.5;
    match wx.precip {
        Precip::Rain | Precip::Drizzle | Precip::Hail if freezing => Contam::Ice { wet: true },
        Precip::Snow | Precip::Sleet if !freezing && t < 3.0 => Contam::Slush(3.0 + 9.0 * i),
        Precip::Snow | Precip::Sleet if freezing => Contam::Snow(3.0 + 20.0 * i + wx.snow_depth_m.min(0.05) * 1000.0 * 0.3),
        Precip::Sleet | Precip::Snow => Contam::Wet(1.0 + 2.0 * i),
        Precip::Drizzle => Contam::Wet(0.5 + 0.5 * i),
        // Light rain ~1 mm, heavy rain / showers / thunderstorms > 3 mm (standing water in the ruts).
        Precip::Rain | Precip::Hail => Contam::Wet(0.8 + if wx.showers || wx.thunder { 6.5 } else { 4.5 } * i * i),
        Precip::None => {
            if wx.snow_depth_m > 0.02 && freezing {
                Contam::Snow((wx.snow_depth_m * 1000.0 * 0.15).clamp(2.0, 25.0))
            } else if wx.recent_rain {
                Contam::Wet(0.4)
            } else if freezing && wx.dew_c <= 0.0 && (t - wx.dew_c) < 2.0 {
                // Hoar frost / black ice.
                Contam::Ice { wet: false }
            } else if wx.fog || (t - wx.dew_c) < 0.8 {
                Contam::Damp
            } else {
                Contam::Dry
            }
        }
    }
}

/// Paved areas around the aircraft (current world frame): runways with their
/// surfaces, taxiway centrelines with a width, apron polygons.
#[derive(Clone, Debug, Default)]
pub struct Pavement {
    /// Runway: ends, half width, base surface.
    pub runways: Vec<(Vec2, Vec2, f32, Ground)>,
    /// Taxiway segments (a, b, half width).
    pub taxi: Vec<(Vec2, Vec2, f32)>,
    /// Apron outlines.
    pub aprons: Vec<(Vec<Vec2>, Vec2, Vec2)>,
    /// Bounds of the airports whose layouts are known (off their pavement: `off`).
    pub known: Vec<(Vec2, Vec2)>,
    /// What the ground is off the pavement.
    pub off: Ground,
    /// Contamination of the paved areas (and the grass gets the water too).
    pub contam: Contam,
    /// Key of what it was built from (layouts / frame / runways).
    pub key: u64,
}

impl Pavement {
    /// The surface at `p`.
    pub fn at(&self, p: Vec2) -> Surface {
        let c = self.contam;
        for (a, b, hw, g) in &self.runways {
            if seg_dist(p, *a, *b) <= *hw + 1.5 {
                return Surface::new(*g, if g.paved() || *g == Ground::Snow { c } else { soft_contam(c) });
            }
        }
        let paved = self.taxi.iter().any(|(a, b, hw)| seg_dist(p, *a, *b) <= *hw)
            || self.aprons.iter().any(|(r, lo, hi)| p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && point_in_ring(p, r));
        // Off the known pavement: the airfield's grass / the fields — unless
        // nothing is known here yet (near a runway without a layout, or no
        // scenery at all): pavement, so an apron start is not "on grass".
        let known = self.known.iter().any(|(lo, hi)| p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y);
        let near_runway = self.runways.iter().any(|(a, b, _, _)| seg_dist(p, *a, *b) < 400.0);
        if paved || (!known && (near_runway || self.runways.is_empty())) { Surface::new(Ground::Asphalt, c) } else { Surface::new(self.off, soft_contam(c)) }
    }

    /// Is `p` on a runway, taxiway or apron?
    pub fn on_pavement(&self, p: Vec2) -> bool {
        self.at(p).paved() || self.runways.iter().any(|(a, b, hw, _)| seg_dist(p, *a, *b) <= *hw + 1.5)
    }
}

/// Grass / dirt holds the water as mud rather than standing water.
fn soft_contam(c: Contam) -> Contam {
    match c {
        Contam::Wet(d) => Contam::Wet(d.min(2.0)),
        Contam::Ice { .. } => Contam::Snow(2.0),
        c => c,
    }
}

pub fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

pub fn point_in_ring(p: Vec2, r: &[Vec2]) -> bool {
    let mut inside = false;
    let n = r.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (r[i], r[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}
