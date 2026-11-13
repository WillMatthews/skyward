//! Per-part hit volumes of an airframe (body frame: +X left wing, +Y up,
//! +Z nose, origin = CG) and segment intersection.
//!
//! The structure (wings, fuselage sections, tail surfaces) comes from the
//! part boxes of the real model (`debris::PartGeom`, cut from the imported
//! mesh — or estimated from the flight geometry when no model is loaded),
//! with the wings trimmed to the model's measured planform (leading /
//! trailing edge per span station) so a round through the empty corner of a
//! swept or elliptical wing's bounding box misses. Systems sit inside the
//! structure where they are on the real type: the engine, fuel tanks,
//! radiators (Spitfire glycol), hydraulics, electrics, ammunition, the pilot
//! (with the Spitfire's armour plate behind the seat) and the control
//! surfaces along the trailing edges.

use crate::damage::{Part, StructDef};
use crate::debris::PartGeom;
use crate::flight::AircraftDef;
use glam::Vec3;

/// What a round can hit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum HitPart {
    Pilot = 0,
    Cockpit,
    Engine,
    /// Liquid-cooled engines: glycol / oil radiators.
    Radiator,
    FuelFuselage,
    FuelWingL,
    FuelWingR,
    WingL,
    WingR,
    AileronL,
    AileronR,
    Fuselage,
    Nose,
    Aft,
    HTailL,
    HTailR,
    Elevator,
    Fin,
    Rudder,
    Hydraulics,
    Electrics,
    Ammo,
    Armour,
}

pub const N_HIT: usize = 23;

impl HitPart {
    pub const ALL: [HitPart; N_HIT] = [
        HitPart::Pilot,
        HitPart::Cockpit,
        HitPart::Engine,
        HitPart::Radiator,
        HitPart::FuelFuselage,
        HitPart::FuelWingL,
        HitPart::FuelWingR,
        HitPart::WingL,
        HitPart::WingR,
        HitPart::AileronL,
        HitPart::AileronR,
        HitPart::Fuselage,
        HitPart::Nose,
        HitPart::Aft,
        HitPart::HTailL,
        HitPart::HTailR,
        HitPart::Elevator,
        HitPart::Fin,
        HitPart::Rudder,
        HitPart::Hydraulics,
        HitPart::Electrics,
        HitPart::Ammo,
        HitPart::Armour,
    ];
    pub fn idx(self) -> usize {
        self as usize
    }
    pub fn from_idx(i: usize) -> HitPart {
        HitPart::ALL[i.min(N_HIT - 1)]
    }
    pub fn name(self) -> &'static str {
        match self {
            HitPart::Pilot => "pilot",
            HitPart::Cockpit => "cockpit",
            HitPart::Engine => "engine",
            HitPart::Radiator => "radiator",
            HitPart::FuelFuselage => "fuselage tank",
            HitPart::FuelWingL => "left wing tank",
            HitPart::FuelWingR => "right wing tank",
            HitPart::WingL => "left wing",
            HitPart::WingR => "right wing",
            HitPart::AileronL => "left aileron",
            HitPart::AileronR => "right aileron",
            HitPart::Fuselage => "fuselage",
            HitPart::Nose => "nose",
            HitPart::Aft => "rear fuselage",
            HitPart::HTailL => "left tailplane",
            HitPart::HTailR => "right tailplane",
            HitPart::Elevator => "elevator",
            HitPart::Fin => "fin",
            HitPart::Rudder => "rudder",
            HitPart::Hydraulics => "hydraulics",
            HitPart::Electrics => "electrics",
            HitPart::Ammo => "ammunition bay",
            HitPart::Armour => "armour plate",
        }
    }
    /// The separable structural part this belongs to (what tears off when it fails).
    pub fn structure(self) -> Option<Part> {
        Some(match self {
            HitPart::WingL | HitPart::AileronL | HitPart::FuelWingL => Part::WingL,
            HitPart::WingR | HitPart::AileronR | HitPart::FuelWingR => Part::WingR,
            HitPart::HTailL => Part::HTailL,
            HitPart::HTailR => Part::HTailR,
            HitPart::Fin | HitPart::Rudder => Part::Fin,
            HitPart::Aft | HitPart::Elevator => Part::Aft,
            HitPart::Nose => Part::Nose,
            _ => return None,
        })
    }
    /// Structure (skin, spars) rather than a system inside it.
    pub fn is_structure(self) -> bool {
        matches!(
            self,
            HitPart::WingL
                | HitPart::WingR
                | HitPart::Fuselage
                | HitPart::Nose
                | HitPart::Aft
                | HitPart::HTailL
                | HitPart::HTailR
                | HitPart::Fin
                | HitPart::AileronL
                | HitPart::AileronR
                | HitPart::Elevator
                | HitPart::Rudder
        )
    }
}

/// A wing planform trim: at span station |x|, the chord between `le` and `te` (z).
#[derive(Clone, Debug, Default)]
pub struct Planform {
    /// (station |x|, leading-edge z, trailing-edge z), ascending stations.
    pub bins: Vec<(f32, f32, f32)>,
}

impl Planform {
    pub fn contains(&self, p: Vec3) -> bool {
        if self.bins.len() < 2 {
            return true;
        }
        let x = p.x.abs();
        let (first, last) = (self.bins[0], self.bins[self.bins.len() - 1]);
        if x < first.0 - 0.05 {
            return true;
        }
        if x > last.0 + 0.15 {
            return false;
        }
        let mut le = last.1;
        let mut te = last.2;
        for w in self.bins.windows(2) {
            if x <= w[1].0 {
                let f = ((x - w[0].0) / (w[1].0 - w[0].0).max(1e-4)).clamp(0.0, 1.0);
                le = w[0].1 + (w[1].1 - w[0].1) * f;
                te = w[0].2 + (w[1].2 - w[0].2) * f;
                break;
            }
        }
        p.z <= le + 0.05 && p.z >= te - 0.05
    }
}

/// One axis-aligned (body frame) hit volume.
#[derive(Clone, Debug)]
pub struct HitVol {
    pub part: HitPart,
    pub lo: Vec3,
    pub hi: Vec3,
    /// Trimmed to the wing planform.
    pub wing: bool,
}

/// One intersection of a segment with a volume.
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub part: HitPart,
    /// Distance along the segment (0..1) where it enters.
    pub t: f32,
    /// Entry point and outward surface normal (body frame).
    pub pos: Vec3,
    pub normal: Vec3,
    /// Path length inside the volume (m).
    pub depth: f32,
}

/// Hit volumes of one airframe.
#[derive(Clone, Debug)]
pub struct HitModel {
    pub vols: Vec<HitVol>,
    pub planform: Planform,
    /// Bounding sphere radius about the CG (broad phase).
    pub radius: f32,
    /// Hit points per part (structure strength / system robustness, damage units).
    pub hp: [f32; N_HIT],
}

fn bx(part: HitPart, lo: Vec3, hi: Vec3) -> HitVol {
    HitVol { part, lo: lo.min(hi), hi: lo.max(hi), wing: false }
}

/// The planform measured from the model, else a synthetic one (elliptical for
/// the Spitfire-class warbird, straight-tapered and swept for the jet).
pub fn planform_for(def: &AircraftDef, sd: &StructDef, measured: &[(f32, f32, f32, f32)]) -> Planform {
    let mut bins: Vec<(f32, f32, f32)> = measured.iter().filter(|b| b.0 >= 0.0 && b.1 > b.2).map(|b| (b.0, b.1, b.2)).collect();
    bins.sort_by(|a, b| a.0.total_cmp(&b.0));
    if bins.len() >= 3 {
        return Planform { bins };
    }
    let semi = def.wingspan * 0.5;
    let (z0, z1) = sd.wing_z;
    let root = z1 - z0;
    let mid = 0.5 * (z0 + z1);
    let mut bins = Vec::new();
    for i in 0..=12 {
        let x = sd.fus_r + (semi - sd.fus_r) * i as f32 / 12.0;
        let s = ((x - sd.fus_r) / (semi - sd.fus_r).max(0.1)).clamp(0.0, 1.0);
        let (le, te) = if def.category == crate::flight::Category::Warbird {
            // Elliptical: chord ∝ √(1 − η²), quarter-chord line straight.
            let c = root * (1.0 - s * s).max(0.0).sqrt().max(0.12);
            let qc = z1 - 0.25 * root;
            (qc + 0.25 * c, qc - 0.75 * c)
        } else {
            // Trapezoidal, taper 0.25, LE swept back by the chord loss.
            let c = root * (1.0 - 0.75 * s);
            (z1 - (root - c) * 0.9, z1 - (root - c) * 0.9 - c)
        };
        let _ = mid;
        bins.push((x, le, te));
    }
    Planform { bins }
}

impl HitModel {
    /// Hit volumes for a type from its part boxes and structure.
    pub fn build(def: &AircraftDef, sd: &StructDef, geom: &PartGeom, planform: Planform) -> HitModel {
        let mut v: Vec<HitVol> = Vec::new();
        let g = |p: Part| -> Option<(Vec3, Vec3)> { (geom.has & p.bit() != 0).then(|| (geom.lo[p.idx()], geom.hi[p.idx()])) };
        let fus = g(Part::Fuselage).unwrap_or((Vec3::new(-sd.fus_r, -sd.fus_r, -3.0), Vec3::new(sd.fus_r, sd.fus_r, 3.0)));
        // Fuselage: only its central tube (the part box includes the wing roots).
        let r = sd.fus_r;
        let (flo, fhi) = (Vec3::new(-r * 0.95, fus.0.y.max(-r * 1.3), fus.0.z), Vec3::new(r * 0.95, fus.1.y.min(r * 1.5), fus.1.z));
        let jet = def.engine_kind == crate::flight::EngineKind::Jet;
        let eye = def.eye;
        // ---- systems first (inside the structure; the entry order sorts them).
        // Pilot: torso + head around the eye, armour behind (warbirds).
        v.push(bx(HitPart::Pilot, eye + Vec3::new(-0.22, -0.75, -0.35), eye + Vec3::new(0.22, 0.14, 0.12)));
        v.push(bx(HitPart::Cockpit, eye + Vec3::new(-0.45, -0.85, -0.6), eye + Vec3::new(0.45, 0.3, 1.0)));
        if def.category == crate::flight::Category::Warbird {
            // 6.35 mm (¼ in) plate behind the seat, 4 in headrest plate.
            v.push(bx(HitPart::Armour, eye + Vec3::new(-0.3, -0.8, -0.5), eye + Vec3::new(0.3, 0.2, -0.43)));
        }
        let e = def.engine_pos.first().copied().unwrap_or(Vec3::ZERO);
        if jet {
            // Turbofan in the rear fuselage: from the compressor face to the nozzle.
            v.push(bx(HitPart::Engine, Vec3::new(-0.55, e.y - 0.55, e.z), Vec3::new(0.55, e.y + 0.55, e.z + 4.4)));
            // Fuselage tanks (F / A reservoirs) between cockpit and engine, wing tanks.
            v.push(bx(HitPart::FuelFuselage, Vec3::new(-0.6, -0.2, e.z + 4.4), Vec3::new(0.6, 0.75, eye.z - 1.4)));
            v.push(bx(HitPart::Hydraulics, Vec3::new(-0.7, -0.6, -1.4), Vec3::new(0.7, -0.1, 0.4)));
            v.push(bx(HitPart::Electrics, Vec3::new(-0.45, -0.3, eye.z + 0.9), Vec3::new(0.45, 0.4, eye.z + 2.4)));
            // M61 drum behind the cockpit.
            v.push(bx(HitPart::Ammo, Vec3::new(-0.4, -0.25, eye.z - 2.6), Vec3::new(0.4, 0.4, eye.z - 1.5)));
        } else {
            // Engine forward of the firewall; glycol radiators under the wings.
            let fw = sd.firewall_z.unwrap_or(e.z - 1.2);
            v.push(bx(HitPart::Engine, Vec3::new(-0.42, e.y - 0.5, fw), Vec3::new(0.42, e.y + 0.45, e.z + 1.1)));
            let wy = def.wing_height;
            if def.category == crate::flight::Category::Warbird {
                for s in [1.0f32, -1.0] {
                    v.push(bx(HitPart::Radiator, Vec3::new(s * 1.3, wy - 0.55, -0.6), Vec3::new(s * 2.3, wy - 0.1, 0.6)));
                }
            }
            // Main tanks ahead of the cockpit (Spitfire: 85 gal upper + lower).
            v.push(bx(HitPart::FuelFuselage, Vec3::new(-0.4, -0.35, eye.z + 0.6), Vec3::new(0.4, 0.55, fw)));
            v.push(bx(HitPart::Electrics, Vec3::new(-0.3, -0.3, eye.z - 1.6), Vec3::new(0.3, 0.3, eye.z - 0.7)));
            // Cannon ammunition boxes in the wings.
            for s in [1.0f32, -1.0] {
                v.push(bx(HitPart::Ammo, Vec3::new(s * 2.0, wy - 0.12, -0.6), Vec3::new(s * 2.9, wy + 0.12, 0.4)));
            }
        }
        // ---- structure
        let semi = def.wingspan * 0.5;
        for (s, part, wpart, tank, ail) in
            [(1.0f32, Part::WingL, HitPart::WingL, HitPart::FuelWingL, HitPart::AileronL), (-1.0, Part::WingR, HitPart::WingR, HitPart::FuelWingR, HitPart::AileronR)]
        {
            let (lo, hi) = g(part).unwrap_or((Vec3::new(s * r, def.wing_height - 0.15, sd.wing_z.0), Vec3::new(s * semi, def.wing_height + 0.25, sd.wing_z.1)));
            let (xl, xh) = (lo.x.abs().min(hi.x.abs()), lo.x.abs().max(hi.x.abs()));
            let span = xh - xl;
            // Wing tank (inboard leading-edge box on the warbird, most of the inner wing on the jet).
            let (t0, t1) = if jet { (xl, xl + span * 0.55) } else { (xl, xl + span * 0.25) };
            // (The Spitfire IX carries its fuel in the fuselage only.)
            if jet {
                let mut tv = bx(tank, Vec3::new(s * t0, lo.y, (lo.z + hi.z) * 0.5), Vec3::new(s * t1, hi.y, hi.z));
                tv.wing = true;
                v.push(tv);
            }
            // Aileron: the outer trailing-edge strip.
            let mut av = bx(ail, Vec3::new(s * (xl + span * 0.55), lo.y, lo.z - 0.2), Vec3::new(s * (xl + span * 0.95), hi.y, lo.z + (hi.z - lo.z) * 0.28));
            av.wing = true;
            v.push(av);
            let mut wv = bx(wpart, Vec3::new(s * xl, lo.y, lo.z), Vec3::new(s * xh, hi.y, hi.z));
            wv.wing = true;
            v.push(wv);
        }
        if let Some((lo, hi)) = g(Part::Nose) {
            v.push(bx(HitPart::Nose, lo.max(Vec3::new(-r, -r * 1.2, lo.z)), hi.min(Vec3::new(r, r * 1.2, hi.z))));
        }
        v.push(bx(HitPart::Fuselage, flo, fhi));
        let aft = g(Part::Aft).unwrap_or((Vec3::new(-r * 0.7, -r * 0.5, sd.tail_z - 1.5), Vec3::new(r * 0.7, r, sd.aft_z)));
        v.push(bx(HitPart::Aft, Vec3::new(aft.0.x.max(-r * 0.8), aft.0.y.max(-r), aft.0.z), Vec3::new(aft.1.x.min(r * 0.8), aft.1.y.min(r * 1.1), aft.1.z)));
        let ht = |p: Part, s: f32| {
            g(p).unwrap_or((Vec3::new(s * sd.fin_r, sd.ht_pos.y - 0.1, sd.ht_pos.z - 1.0), Vec3::new(s * sd.ht_pos.x * 2.0, sd.ht_pos.y + 0.1, sd.ht_pos.z + 0.8)))
        };
        for (s, p, hp) in [(1.0f32, Part::HTailL, HitPart::HTailL), (-1.0, Part::HTailR, HitPart::HTailR)] {
            let (lo, hi) = ht(p, s);
            if !jet {
                // Elevator: the trailing third of the tailplane.
                v.push(bx(HitPart::Elevator, lo, Vec3::new(hi.x, hi.y, lo.z + (hi.z - lo.z) * 0.35)));
            }
            v.push(bx(hp, lo, hi));
        }
        let (flo, fhi) = g(Part::Fin).unwrap_or((Vec3::new(-0.1, sd.fin_y, sd.vt_pos.z - 1.2), Vec3::new(0.1, sd.vt_pos.y * 2.0 - sd.fin_y, sd.vt_pos.z + 1.0)));
        v.push(bx(HitPart::Rudder, Vec3::new(flo.x, flo.y, flo.z), Vec3::new(fhi.x, fhi.y, flo.z + (fhi.z - flo.z) * 0.32)));
        v.push(bx(HitPart::Fin, flo, fhi));
        let radius = v.iter().map(|h| h.lo.length().max(h.hi.length())).fold(0.0, f32::max) + 0.5;
        // Hit points (damage units; a 20 mm HE hit ≈ 50, a .303 ball ≈ 2).
        let heavy = (def.mass / 3400.0).clamp(0.6, 6.0).sqrt();
        let mut hp = [100.0f32; N_HIT];
        let set = |hp: &mut [f32; N_HIT], p: HitPart, x: f32| hp[p.idx()] = x;
        set(&mut hp, HitPart::Pilot, 30.0);
        set(&mut hp, HitPart::Cockpit, 90.0);
        set(&mut hp, HitPart::Engine, if jet { 110.0 } else { 80.0 } * heavy);
        set(&mut hp, HitPart::Radiator, 14.0);
        set(&mut hp, HitPart::FuelFuselage, 70.0);
        set(&mut hp, HitPart::FuelWingL, 50.0);
        set(&mut hp, HitPart::FuelWingR, 50.0);
        set(&mut hp, HitPart::WingL, 160.0 * heavy);
        set(&mut hp, HitPart::WingR, 160.0 * heavy);
        set(&mut hp, HitPart::AileronL, 30.0);
        set(&mut hp, HitPart::AileronR, 30.0);
        set(&mut hp, HitPart::Fuselage, 320.0 * heavy);
        set(&mut hp, HitPart::Nose, 140.0 * heavy);
        set(&mut hp, HitPart::Aft, 150.0 * heavy);
        set(&mut hp, HitPart::HTailL, 60.0 * heavy);
        set(&mut hp, HitPart::HTailR, 60.0 * heavy);
        set(&mut hp, HitPart::Elevator, 30.0);
        set(&mut hp, HitPart::Fin, 70.0 * heavy);
        set(&mut hp, HitPart::Rudder, 25.0);
        set(&mut hp, HitPart::Hydraulics, 40.0);
        set(&mut hp, HitPart::Electrics, 35.0);
        set(&mut hp, HitPart::Ammo, 45.0);
        set(&mut hp, HitPart::Armour, 1.0e9);
        HitModel { vols: v, planform, radius, hp }
    }

    /// Every volume the body-frame segment a→b passes through, by entry order.
    pub fn intersect(&self, a: Vec3, b: Vec3) -> Vec<Hit> {
        let mut out = Vec::new();
        let d = b - a;
        let len = d.length();
        if len < 1e-6 {
            return out;
        }
        // Broad phase: the segment passes within the bounding sphere.
        let t_c = (-a.dot(d) / d.length_squared()).clamp(0.0, 1.0);
        if (a + d * t_c).length() > self.radius {
            return out;
        }
        for v in &self.vols {
            let Some((t0, t1, n)) = slab(a, d, v.lo, v.hi) else { continue };
            let (mut t0, t1) = (t0, t1);
            if v.wing {
                // First point inside the planform along the path through the box.
                let steps = 8;
                let mut found = None;
                for k in 0..=steps {
                    let t = t0 + (t1 - t0) * k as f32 / steps as f32;
                    if self.planform.contains(a + d * t) {
                        found = Some(t);
                        break;
                    }
                }
                match found {
                    Some(t) => t0 = t,
                    None => continue,
                }
            }
            out.push(Hit { part: v.part, t: t0, pos: a + d * t0, normal: n, depth: (t1 - t0) * len });
        }
        out.sort_by(|x, y| x.t.total_cmp(&y.t));
        out
    }

    /// Volumes within `r` of a point (blast), with the distance.
    pub fn near(&self, p: Vec3, r: f32) -> Vec<(HitPart, f32)> {
        self.vols
            .iter()
            .filter_map(|v| {
                let q = p.clamp(v.lo, v.hi);
                let d = q.distance(p);
                (d <= r && (!v.wing || self.planform.contains(q))).then_some((v.part, d))
            })
            .collect()
    }

    /// Projected area (m²) seen along `dir` (body frame), for the fragment / AI geometry.
    pub fn area_along(&self, dir: Vec3) -> f32 {
        let d = dir.normalize_or_zero().abs();
        self.vols
            .iter()
            .filter(|v| v.part.is_structure())
            .map(|v| {
                let s = v.hi - v.lo;
                s.y * s.z * d.x + s.x * s.z * d.y + s.x * s.y * d.z
            })
            .sum::<f32>()
            * 0.7
    }
}

/// Ray / box slab test: entry and exit parameters within 0..1 and the entry normal.
fn slab(a: Vec3, d: Vec3, lo: Vec3, hi: Vec3) -> Option<(f32, f32, Vec3)> {
    let mut t0 = 0.0f32;
    let mut t1 = 1.0f32;
    let mut n = Vec3::ZERO;
    for k in 0..3 {
        let (o, dd, l, h) = (a[k], d[k], lo[k], hi[k]);
        if dd.abs() < 1e-9 {
            if o < l || o > h {
                return None;
            }
            continue;
        }
        let (mut ta, mut tb) = ((l - o) / dd, (h - o) / dd);
        let mut nn = Vec3::ZERO;
        nn[k] = -dd.signum();
        if ta > tb {
            std::mem::swap(&mut ta, &mut tb);
        }
        if ta > t0 {
            t0 = ta;
            n = nn;
        }
        t1 = t1.min(tb);
        if t0 > t1 {
            return None;
        }
    }
    if n == Vec3::ZERO {
        // Started inside: the normal faces back along the path.
        n = -d.normalize_or_zero();
    }
    Some((t0, t1, n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::catalogue;

    fn model(id: &str) -> (AircraftDef, HitModel) {
        let def = catalogue().into_iter().find(|d| d.id == id).unwrap();
        let sd = crate::damage::struct_def(id);
        let geom = PartGeom::estimate(&def, sd);
        let pf = planform_for(&def, sd, &[]);
        let m = HitModel::build(&def, sd, &geom, pf);
        (def, m)
    }

    #[test]
    fn rounds_from_six_oclock_hit_tail_then_pilot_armour() {
        let (def, m) = model("peregrine");
        // A round along the centreline from behind, at the pilot's height.
        let y = def.eye.y - 0.3;
        let hits = m.intersect(Vec3::new(0.0, y, -30.0), Vec3::new(0.0, y, 30.0));
        let parts: Vec<HitPart> = hits.iter().map(|h| h.part).collect();
        let ia = parts.iter().position(|p| *p == HitPart::Armour).expect("armour");
        let ip = parts.iter().position(|p| *p == HitPart::Pilot).expect("pilot");
        assert!(ia < ip, "armour before pilot: {parts:?}");
        assert!(parts.contains(&HitPart::Engine));
    }

    #[test]
    fn wing_planform_trims_the_bounding_box() {
        let (def, m) = model("peregrine");
        let sd = crate::damage::struct_def("peregrine");
        let semi = def.wingspan * 0.5;
        let y = def.wing_height;
        // Vertical shots through the wing near the root hit; through the
        // bounding-box corner at the elliptical tip's trailing edge, miss.
        let root = m.intersect(Vec3::new(1.5, y + 5.0, 0.2), Vec3::new(1.5, y - 5.0, 0.2));
        assert!(root.iter().any(|h| h.part == HitPart::WingL), "{root:?}");
        let corner = Vec3::new(semi * 0.95, y, sd.wing_z.0 + 0.1);
        let tip = m.intersect(corner + Vec3::Y * 5.0, corner - Vec3::Y * 5.0);
        assert!(!tip.iter().any(|h| h.part == HitPart::WingL), "{tip:?}");
        // Far off: nothing (broad phase).
        assert!(m.intersect(Vec3::new(40.0, 0.0, 0.0), Vec3::new(40.0, 0.0, 10.0)).is_empty());
    }

    #[test]
    fn jet_hits_engine_fuel_and_tail() {
        let (def, m) = model("lancer");
        let e = def.engine_pos[0];
        let hits = m.intersect(Vec3::new(-20.0, e.y, e.z + 2.0), Vec3::new(20.0, e.y, e.z + 2.0));
        assert!(hits.iter().any(|h| h.part == HitPart::Engine), "{hits:?}");
        let s = crate::damage::struct_def("lancer");
        let fin = m.intersect(Vec3::new(10.0, s.vt_pos.y, s.vt_pos.z), Vec3::new(-10.0, s.vt_pos.y, s.vt_pos.z));
        assert!(fin.iter().any(|h| h.part == HitPart::Fin || h.part == HitPart::Rudder), "{fin:?}");
        // Entry normals face the shooter.
        let h = hits.first().unwrap();
        assert!(h.normal.x < -0.5, "{:?}", h.normal);
        assert!(m.near(e, 1.0).iter().any(|(p, _)| *p == HitPart::Engine));
        assert!(m.area_along(Vec3::Y) > m.area_along(Vec3::Z));
    }
}
