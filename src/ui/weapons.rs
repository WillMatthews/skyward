//! Weapons symbology and combat UI (Track W): the F-16 HUD's gun funnel
//! (EEGS-style), LCOS pipper with range bar, target box, AIM-9 seeker
//! circle and the AIM-120 ASE circle / steering dot, launch zones and
//! cues; the Spitfire's Mk II gyro gunsight (stadiametric ranging ring with
//! the computed lead); the RWR scope; weapon / ammunition status; bandit
//! markers in outside views; damage report; kill feed and dogfight score.

use super::hudsym::Cam;
use super::{f_digits_b, f_semi, f_ui, pal, sp};
use crate::sim::{CamMode, Sim};
use crate::weapons::events::Emission;
use crate::weapons::missiles::MissileKind;
use crate::weapons::{Combat, GunGroup, Life, Master, Unit};
use egui::{Align2, Color32, Pos2, Rect, Stroke, Vec2 as EVec2};
use glam::{Quat, Vec3};

const FT: f32 = 3.280_84;

fn line(p: &egui::Painter, a: Pos2, b: Pos2, w: f32, col: Color32) {
    p.line_segment([a, b], Stroke::new(w + 2.2, Color32::from_black_alpha(60)));
    p.line_segment([a, b], Stroke::new(w, col));
}
fn circle(p: &egui::Painter, c: Pos2, r: f32, w: f32, col: Color32) {
    p.circle_stroke(c, r, Stroke::new(w + 2.2, Color32::from_black_alpha(60)));
    p.circle_stroke(c, r, Stroke::new(w, col));
}
fn text(p: &egui::Painter, at: Pos2, align: Align2, s: impl ToString, size: f32, col: Color32) {
    let s = s.to_string();
    let size = size.max(12.0);
    p.text(at + EVec2::new(1.0, 1.0), align, &s, f_digits_b(size), Color32::from_black_alpha(90));
    p.text(at, align, s, f_digits_b(size), col);
}
fn dashed_circle(p: &egui::Painter, c: Pos2, r: f32, w: f32, col: Color32, n: usize) {
    for i in 0..n {
        let a0 = std::f32::consts::TAU * i as f32 / n as f32;
        let a1 = a0 + std::f32::consts::TAU / n as f32 * 0.55;
        line(p, super::polar(c, r, a0), super::polar(c, r, a1), w, col);
    }
}

/// The local unit and its aircraft pose (pos, rot, vel).
fn local(sim: &Sim) -> Option<(&Combat, &Unit)> {
    let c = sim.combat.as_deref()?;
    Some((c, c.local_unit()?))
}

/// Position / velocity of a unit (the local one is the sim's aircraft).
fn pose_of(sim: &Sim, c: &Combat, id: u32) -> Option<(Vec3, Vec3, Quat)> {
    let u = c.unit(id)?;
    match &u.ai {
        Some(a) => Some((a.ac.pos, a.ac.vel, a.ac.rot)),
        None if Some(id) == c.local => Some((sim.ac.pos, sim.ac.vel, sim.ac.rot)),
        None => None,
    }
}

/// The enemy the sights work on: the radar lock, else the nearest within 15° of the nose.
fn sight_target(sim: &Sim, c: &Combat, u: &Unit) -> Option<(u32, Vec3, Vec3)> {
    if let Some(id) = u.radar.stt {
        if let Some((p, v, _)) = pose_of(sim, c, id) {
            return Some((id, p, v));
        }
    }
    let fwd = sim.ac.rot * Vec3::Z;
    c.units
        .iter()
        .filter(|o| o.team != u.team && o.flying())
        .filter_map(|o| pose_of(sim, c, o.id).map(|(p, v, _)| (o.id, p, v)))
        .filter(|(_, p, _)| fwd.angle_between(*p - sim.ac.pos) < 15f32.to_radians() && p.distance(sim.ac.pos) < 3000.0)
        .min_by(|a, b| a.1.distance(sim.ac.pos).total_cmp(&b.1.distance(sim.ac.pos)))
}

/// Where rounds fired over the last seconds are now (relative to the
/// aircraft, world axes), assuming the present turn rate: the gun funnel /
/// LCOS stream. Returns (time of flight, offset).
fn stream(sim: &Sim, u: &Unit) -> Vec<(f32, Vec3)> {
    let Some(g) = u.load.guns.first() else { return Vec::new() };
    let gl = u.gun_dirs.first().copied().unwrap_or(Vec3::Z);
    let ac = &sim.ac;
    let w_world = ac.rot * ac.omega;
    let mv = g.round.muzzle_v;
    let mut out = Vec::new();
    let atm = crate::flight::isa(ac.pos.y);
    for k in 1..=24 {
        let t = k as f32 * 0.075;
        // Attitude and velocity t seconds ago.
        let back = Quat::from_scaled_axis(-w_world * t);
        let dir = back * (ac.rot * gl);
        let v0 = back * ac.vel;
        // Mean speed with drag (density-scaled decay ≈ 18 % per second at sea level).
        let decay = (1.0 - 0.18 * atm.density / 1.225 * t * 0.5).max(0.5);
        let rel = dir * (mv * t * decay) + (v0 - ac.vel) * t + Vec3::new(0.0, -0.5 * crate::flight::GRAVITY * t * t, 0.0);
        out.push((t, rel));
    }
    out
}

/// F-16 HUD weapon symbology (called from `hudsym::draw`).
pub fn hud(p: &egui::Painter, cam: &Cam, field: Rect, sim: &Sim, k: f32, col: Color32) {
    let Some((c, u)) = local(sim) else { return };
    let w = 1.6 * k;
    let (fw, fh) = (field.width(), field.height());
    let ac = &sim.ac;
    let fwd = sim.render_pose().1 * Vec3::Z;
    // Master mode / stores (bottom-left, above α).
    let bl = Pos2::new(field.min.x + fw * 0.03, field.max.y - fh * 0.17 - 52.0 * k);
    let mode = match u.master {
        Master::Gun => format!("GUN {}", u.ammo(None)),
        Master::Srm => format!("SRM {}  {}", u.missiles_left(MissileKind::Aim9), if u.seeker.locked.is_some() { "LOCK" } else { "" }),
        Master::Mrm => format!("MRM {}", u.missiles_left(MissileKind::Aim120)),
    };
    text(p, bl, Align2::LEFT_CENTER, mode, 12.0 * k, col);
    text(p, bl + EVec2::new(0.0, -16.0 * k), Align2::LEFT_CENTER, format!("F{} C{}", u.flares, u.chaff), 11.0 * k, col);
    let tgt = sight_target(sim, c, u);
    // Target box + range / closure (right side).
    if let Some((_, tp, tv)) = tgt {
        if let Some(b) = cam.project((tp - eye(sim)).normalize()) {
            let s = 9.0 * k;
            p.rect_stroke(Rect::from_center_size(b, EVec2::splat(s * 2.0)), 0, Stroke::new(w, col), egui::StrokeKind::Middle);
        }
        let r = tp.distance(ac.pos);
        let vc = -(tv - ac.vel).dot((tp - ac.pos).normalize_or_zero());
        let rx = field.max.x - fw * 0.03;
        let ry = field.max.y - fh * 0.30;
        text(p, Pos2::new(rx, ry), Align2::RIGHT_CENTER, format!("{:.1}", r / 1852.0), 12.0 * k, col);
        text(p, Pos2::new(rx, ry + 16.0 * k), Align2::RIGHT_CENTER, format!("{:+.0}", vc / crate::flight::KT), 11.0 * k, col);
    }
    match u.master {
        Master::Gun => {
            // EEGS funnel: the stream's walls a 35 ft wingspan apart at each range.
            let st = stream(sim, u);
            let span = 10.7;
            let mut left = Vec::new();
            let mut right = Vec::new();
            let mut pipper = None;
            let trange = tgt.map(|t| t.1.distance(ac.pos));
            for (i, (t, rel)) in st.iter().enumerate() {
                let r = rel.length().max(10.0);
                let Some(cpt) = cam.project(rel.normalize()) else { continue };
                let half = (span * 0.5 / r).atan() * cam.ppd() * 57.3;
                // Walls perpendicular to the stream on screen.
                let next = st.get(i + 1).and_then(|n| cam.project(n.1.normalize())).unwrap_or(cpt + EVec2::new(0.0, 1.0));
                // (A straight stream projects almost to a point: the funnel droops straight down.)
                let along = if (next - cpt).length() > 0.5 { (next - cpt).normalized() } else { EVec2::new(0.0, 1.0) };
                let perp = EVec2::new(-along.y, along.x);
                if *t > 0.2 && r < 1400.0 {
                    left.push(cpt - perp * half);
                    right.push(cpt + perp * half);
                }
                if let Some(tr) = trange {
                    if pipper.is_none() && r >= tr {
                        pipper = Some((cpt, tr));
                    }
                }
            }
            for side in [left, right] {
                if side.len() > 1 {
                    p.add(egui::Shape::line(side.clone(), Stroke::new(w + 2.2, Color32::from_black_alpha(60))));
                    p.add(egui::Shape::line(side, Stroke::new(w, col)));
                }
            }
            // LCOS pipper on the target's range with a range bar (1,000 ft per clock hour).
            let (pc, range) = pipper.unwrap_or_else(|| {
                let r = 2000.0 / FT;
                let pt = st.iter().find(|s| s.1.length() >= r).and_then(|s| cam.project(s.1.normalize())).or(cam.project(fwd)).unwrap_or(field.center());
                (pt, r)
            });
            let rr = 22.0 * k;
            circle(p, pc, rr, w, col);
            p.circle_filled(pc, 1.8 * k, col);
            let frac = (range * FT / 12_000.0).clamp(0.0, 1.0);
            let n = 24;
            let mut arc = Vec::new();
            for i in 0..=((n as f32 * frac) as usize) {
                arc.push(super::polar(pc, rr - 4.0 * k, std::f32::consts::TAU * i as f32 / n as f32));
            }
            if arc.len() > 1 {
                p.add(egui::Shape::line(arc, Stroke::new(w * 1.8, col)));
            }
            if tgt.is_some_and(|t| t.1.distance(ac.pos) < 900.0) && (sim.time * 3.0).fract() < 0.7 {
                text(p, pc + EVec2::new(0.0, rr + 14.0 * k), Align2::CENTER_CENTER, "IN RNG", 11.0 * k, col);
            }
        }
        Master::Srm => {
            // Seeker circle: dashed caged, solid when tracking.
            let dir = u.seeker.dir;
            if let Some(sc) = cam.project(dir) {
                let r = 2.5 * cam.ppd() * 1.0;
                if u.seeker.locked.is_some() {
                    circle(p, sc, r, w, col);
                } else {
                    dashed_circle(p, sc, r, w, col, 12);
                }
            }
            dlz(p, field, sim, tgt, MissileKind::Aim9, u, k, col);
        }
        Master::Mrm => {
            // ASE circle and steering dot (lead pursuit for the missile).
            let bore = cam.project(fwd).unwrap_or(field.center());
            let r = 30.0 * k;
            circle(p, bore, r, w, col);
            if let Some((_, tp, tv)) = tgt {
                let tof = tp.distance(ac.pos) / 900.0;
                let lead = tp + tv * tof * 0.5;
                if let Some(dot) = cam.project((lead - eye(sim)).normalize()) {
                    let d = dot - bore;
                    let d = if d.length() > r * 1.3 { d.normalized() * r * 1.3 } else { d };
                    p.circle_filled(bore + d, 4.0 * k, col);
                }
            }
            dlz(p, field, sim, tgt, MissileKind::Aim120, u, k, col);
            // Missiles in flight: time to active / impact.
            if let Some(m) = c.missiles.iter().find(|m| Some(m.owner) == c.local && m.def.kind == MissileKind::Aim120) {
                let r = m.est_pos.distance(m.pos);
                let s = if m.active { format!("M{:.0}", r / m.speed().max(100.0)) } else { format!("A{:.0}", ((r - 15_000.0) / m.speed().max(100.0)).max(0.0)) };
                text(p, Pos2::new(field.center().x, field.max.y - fh * 0.24), Align2::CENTER_CENTER, s, 12.0 * k, col);
            }
        }
    }
    let _ = c;
}

/// Launch zone scale on the right of the HUD (Rmax / Rmin, the target's range caret).
#[allow(clippy::too_many_arguments)]
fn dlz(p: &egui::Painter, field: Rect, sim: &Sim, tgt: Option<(u32, Vec3, Vec3)>, kind: MissileKind, u: &Unit, k: f32, col: Color32) {
    let Some((_, tp, tv)) = tgt else { return };
    let ac = &sim.ac;
    let r = tp.distance(ac.pos);
    let los = (tp - ac.pos).normalize_or_zero();
    // Simple kinematic zone: closure and altitude stretch Rmax; aspect matters for the AIM-9.
    let vc = -(tv - ac.vel).dot(los);
    let alt_k = (ac.pos.y / 6000.0).clamp(0.4, 1.6);
    let (rmax, rmin) = match kind {
        MissileKind::Aim9 => ((7000.0 + vc * 18.0).clamp(2500.0, 17_000.0) * alt_k.sqrt(), 500.0),
        MissileKind::Aim120 => ((28_000.0 + vc * 60.0).clamp(9000.0, 70_000.0) * alt_k, 1500.0),
    };
    let scale_max = rmax * 1.4;
    let x = field.max.x - field.width() * 0.07;
    let y0 = field.max.y - field.height() * 0.42;
    let y1 = field.min.y + field.height() * 0.28;
    let yr = |d: f32| y0 + (y1 - y0) * (d / scale_max).clamp(0.0, 1.0);
    let w = 1.6 * k;
    line(p, Pos2::new(x, yr(rmin)), Pos2::new(x, yr(rmax)), w, col);
    line(p, Pos2::new(x - 7.0 * k, yr(rmax)), Pos2::new(x, yr(rmax)), w, col);
    line(p, Pos2::new(x - 7.0 * k, yr(rmin)), Pos2::new(x, yr(rmin)), w, col);
    let cy = yr(r);
    p.add(egui::Shape::convex_polygon(vec![Pos2::new(x + 2.0 * k, cy), Pos2::new(x + 11.0 * k, cy - 5.0 * k), Pos2::new(x + 11.0 * k, cy + 5.0 * k)], col, Stroke::NONE));
    let ready = match kind {
        MissileKind::Aim9 => u.seeker.locked.is_some(),
        MissileKind::Aim120 => u.radar.stt.is_some(),
    };
    if ready && r < rmax && r > rmin && (sim.time * 2.5).fract() < 0.7 {
        text(p, Pos2::new(field.center().x, field.max.y - field.height() * 0.30), Align2::CENTER_CENTER, "SHOOT", 15.0 * k, col);
    }
}

fn eye(sim: &Sim) -> Vec3 {
    sim.last_view.as_ref().map(|v| v.eye).unwrap_or(sim.ac.pos)
}

/// Everything else drawn while flying with weapons (all views).
pub fn draw(p: &egui::Painter, screen: Rect, sim: &Sim) {
    let Some((c, u)) = local(sim) else { return };
    let Some(v) = sim.last_view.as_ref() else { return };
    let cam = Cam::from_view(v.view, v.fov_y, screen);
    let cockpit = sim.cam == CamMode::Cockpit;
    let jet = u.load.sight == crate::weapons::guns::Sight::Hud;
    // ---- the gyro gunsight on its reflector glass (cockpit view, warbird).
    if cockpit && !jet && u.alive() {
        gunsight(p, &cam, sim, c, u);
    }
    // ---- bandit markers (outside views; in the cockpit only beyond 1.5 km).
    for o in c.units.iter().filter(|o| Some(o.id) != c.local && Some(o.id) != sim.view_unit && o.flying()) {
        let Some((tp, _, _)) = pose_of(sim, c, o.id) else { continue };
        let d = tp - v.eye;
        let r = d.length();
        if cockpit && r < 1500.0 {
            continue;
        }
        let col = if o.team == u.team { pal::CYAN } else { pal::RED };
        let col = if o.alive() { col } else { Color32::from_rgb(140, 140, 140) };
        match cam.project(d.normalize()) {
            Some(sp) if screen.contains(sp) => {
                let s = (14.0 + 4000.0 / r.max(200.0)).min(30.0);
                p.rect_stroke(Rect::from_center_size(sp, EVec2::splat(s)), 2, Stroke::new(1.4, col), egui::StrokeKind::Middle);
                p.text(sp + EVec2::new(s * 0.5 + 4.0, -s * 0.5), Align2::LEFT_TOP, format!("{}  {:.1} km", o.callsign, r / 1000.0), f_semi(12.0), col);
            }
            _ => {
                // Off-screen: an arrow on the edge pointing toward it.
                let fwd_c = v.view.transform_vector3(d);
                let dir2 = EVec2::new(fwd_c.x, -fwd_c.y).normalized();
                let cen = screen.center();
                let edge = cen + dir2 * (screen.height() * 0.42);
                let n = dir2.rot90();
                p.add(egui::Shape::convex_polygon(vec![edge + dir2 * 12.0, edge - dir2 * 4.0 + n * 7.0, edge - dir2 * 4.0 - n * 7.0], col, Stroke::NONE));
            }
        }
    }
    // Outside views: where the guns are pointing (harmonised range).
    if !cockpit && u.alive() {
        let conv = c.convergence(u).max(400.0);
        let aim = sim.ac.pos + sim.render_pose().1 * Vec3::Z * conv;
        if let Some(a) = cam.project((aim - v.eye).normalize()) {
            line(p, a - EVec2::new(8.0, 0.0), a - EVec2::new(3.0, 0.0), 1.4, pal::HUD);
            line(p, a + EVec2::new(3.0, 0.0), a + EVec2::new(8.0, 0.0), 1.4, pal::HUD);
            line(p, a - EVec2::new(0.0, 8.0), a - EVec2::new(0.0, 3.0), 1.4, pal::HUD);
            line(p, a + EVec2::new(0.0, 3.0), a + EVec2::new(0.0, 8.0), 1.4, pal::HUD);
        }
    }
    // ---- stores / ammunition panel (bottom-left).
    stores(p, screen, sim, u, c);
    // ---- RWR scope.
    if u.load.rwr {
        rwr(p, screen, sim, u, c);
    }
    // ---- kill feed and score (top-right).
    feed(p, screen, sim, c);
    // ---- red flash when hit; status when out.
    if u.hit_flash < 0.25 {
        let a = ((0.25 - u.hit_flash) / 0.25 * 70.0) as u8;
        p.rect_filled(screen, 0, Color32::from_rgba_unmultiplied(160, 0, 0, a));
    }
    let big = |s: &str, sub: &str| {
        let r = Rect::from_center_size(Pos2::new(screen.center().x, screen.min.y + 64.0), EVec2::new(420.0, 64.0));
        super::panel(p, r, sp::R_PANEL);
        p.text(r.center() - EVec2::new(0.0, 10.0), Align2::CENTER_CENTER, s, f_semi(22.0), pal::TEXT);
        p.text(r.center() + EVec2::new(0.0, 18.0), Align2::CENTER_CENTER, sub, f_ui(13.0), pal::TEXT2);
    };
    let df = c.dogfight.is_some();
    match u.life {
        Life::Ejected(_) => big(if u.load.seat { "Ejected" } else { "Bailed out" }, if df { "Back in the fight shortly — R to respawn now" } else { "R to restart" }),
        _ if u.bd.pilot == crate::weapons::bdmg::Pilot::Dead => big("Pilot killed", if df { "Respawning…" } else { "R to restart" }),
        Life::Dead(_) => big("Shot down", if df { "Respawning…" } else { "R to restart" }),
        _ => {}
    }
}

/// Mk II gyro gunsight: fixed cross, and the gyro ring of six diamonds whose
/// diameter frames the target's wingspan at the set range, displaced by the
/// computed lead (own turn rate × time of flight) and the drop.
fn gunsight(p: &egui::Painter, cam: &Cam, sim: &Sim, c: &Combat, u: &Unit) {
    let rot = sim.render_pose().1;
    let fwd = rot * Vec3::Z;
    let Some(centre) = cam.project(fwd) else { return };
    let amber = Color32::from_rgba_unmultiplied(255, 150, 60, 220);
    let ppd = cam.ppd();
    // Fixed graticule: a cross with a 100 mil ring (dim).
    let fixed = Color32::from_rgba_unmultiplied(255, 140, 60, 120);
    let r_fixed = 5.73 * ppd;
    p.circle_stroke(centre, r_fixed, Stroke::new(1.0, fixed));
    for (a, b) in [(EVec2::new(-1.0, 0.0), 0.6), (EVec2::new(1.0, 0.0), 0.6), (EVec2::new(0.0, 1.0), 0.6), (EVec2::new(0.0, -1.0), 0.6)] {
        p.line_segment([centre + a * r_fixed * 0.25, centre + a * r_fixed * (0.25 + b)], Stroke::new(1.0, fixed));
    }
    // Twist-grip range: framed on the nearest enemy near the nose (else 300 yd).
    let tgt = sight_target(sim, c, u);
    // (Beyond the twist grip's 800 yd a pilot leaves it at the harmonisation range.)
    let conv = c.convergence(u).max(150.0);
    let range = tgt.map(|t| t.1.distance(sim.ac.pos)).filter(|r| *r < 730.0).unwrap_or(conv).clamp(140.0, 730.0);
    let span = tgt.and_then(|t| c.unit(t.0)).map(|o| c.defs[o.def].wingspan).unwrap_or(11.2);
    let g = u.load.guns.first();
    let mv = g.map(|g| g.round.muzzle_v).unwrap_or(800.0) + sim.ac.vel.length();
    let tof = range / (mv * 0.85);
    // Lead: the aircraft's angular rate (body pitch / yaw) × time of flight, opposite to the turn.
    let rates = sim.ac.rates();
    // (The gyro's deflection is mechanically limited to ≈ 10°.)
    let lim = 10f32.to_radians();
    let lead_up = (-rates.y * tof).clamp(-lim, lim);
    let lead_right = (-rates.z * tof).clamp(-lim, lim);
    let drop = 0.5 * crate::flight::GRAVITY * tof * tof / range;
    let up = rot * Vec3::Y;
    let right = rot * Vec3::NEG_X;
    if std::env::var_os("SKYWARD_DEBUG_SIGHT").is_some() {
        eprintln!("gyro sight: range {range:.0} tof {tof:.3} rates {rates:?} lead up {lead_up:.4} right {lead_right:.4} drop {drop:.4}");
    }
    let d = (fwd + up * (lead_up - drop) + right * lead_right).normalize();
    let Some(ring_c) = cam.project(d) else { return };
    let ring_r = (span * 0.5 / range).atan().to_degrees() * ppd;
    for i in 0..6 {
        let a = std::f32::consts::TAU * i as f32 / 6.0;
        let q = super::polar(ring_c, ring_r, a);
        let s = 3.2;
        p.add(egui::Shape::convex_polygon(vec![q + EVec2::new(0.0, -s), q + EVec2::new(s, 0.0), q + EVec2::new(0.0, s), q + EVec2::new(-s, 0.0)], amber, Stroke::NONE));
    }
    p.circle_filled(ring_c, 2.0, amber);
    let _ = c;
}

fn stores(p: &egui::Painter, screen: Rect, sim: &Sim, u: &Unit, c: &Combat) {
    let w = 230.0;
    let rows = if u.load.missiles.is_empty() { 3 } else { 4 };
    let h = 26.0 + rows as f32 * 19.0 + if u.bd.total > 0.0 { 60.0 } else { 0.0 };
    let r = Rect::from_min_size(Pos2::new(sp::L, screen.max.y - h - 210.0), EVec2::new(w, h));
    super::panel(p, r, sp::R_PANEL);
    let mut y = r.min.y + 14.0;
    let x = r.min.x + 12.0;
    let title = if u.load.missiles.is_empty() { format!("GUNS · {}", u.group.label()) } else { format!("STORES · {}", u.master.label()) };
    p.text(Pos2::new(x, y), Align2::LEFT_CENTER, title, f_semi(12.0), pal::ACCENT);
    y += 19.0;
    let bar = |p: &egui::Painter, y: f32, label: &str, n: u32, max: u32, active: bool| {
        p.text(Pos2::new(x, y), Align2::LEFT_CENTER, label, f_ui(12.0), if active { pal::TEXT } else { pal::TEXT3 });
        let br = Rect::from_min_size(Pos2::new(x + 86.0, y - 4.0), EVec2::new(90.0, 8.0));
        p.rect_filled(br, 2, Color32::from_white_alpha(20));
        let f = n as f32 / max.max(1) as f32;
        p.rect_filled(Rect::from_min_size(br.min, EVec2::new(90.0 * f, 8.0)), 2, if f < 0.2 { pal::AMBER } else { pal::GREEN });
        p.text(Pos2::new(r.max.x - 12.0, y), Align2::RIGHT_CENTER, n.to_string(), f_digits_b(13.0), pal::TEXT);
    };
    let groups: Vec<u8> = if u.load.guns.iter().any(|g| g.group == 1) { vec![0, 1] } else { vec![0] };
    for gi in groups {
        let guns: Vec<_> = u.load.guns.iter().zip(&u.guns).filter(|(g, _)| g.group == gi).collect();
        if guns.is_empty() {
            continue;
        }
        let label = format!("{} × {}", guns.len(), guns[0].0.kind.label());
        let n: u32 = guns.iter().map(|x| x.1.ammo).sum();
        let max: u32 = guns.iter().map(|x| x.0.ammo).sum();
        let active =
            !u.load.missiles.is_empty() && u.master == Master::Gun || u.load.missiles.is_empty() && (u.group == GunGroup::Both || (u.group == GunGroup::Cannon) == (gi == 0));
        bar(p, y, &label, n, max, active);
        y += 19.0;
    }
    for (kind, max) in &u.load.missiles {
        let n = u.missiles_left(*kind) as u32;
        let active = matches!((kind, u.master), (MissileKind::Aim9, Master::Srm) | (MissileKind::Aim120, Master::Mrm));
        bar(p, y, crate::weapons::missiles::def_of(*kind).name, n, *max as u32, active);
        y += 19.0;
    }
    if !u.load.convergence.is_empty() {
        p.text(Pos2::new(x, y), Align2::LEFT_CENTER, format!("Harmonised {:.0} yd · F9", c.convergence(u) / 0.9144), f_ui(11.5), pal::TEXT2);
        y += 19.0;
    } else {
        p.text(Pos2::new(x, y), Align2::LEFT_CENTER, format!("Flares {} · chaff {}", u.flares, u.chaff), f_ui(11.5), pal::TEXT2);
        y += 19.0;
    }
    if u.bd.total > 0.0 {
        // Damage report: the worst-hit parts.
        let mut parts: Vec<(crate::weapons::hitbox::HitPart, f32)> =
            crate::weapons::hitbox::HitPart::ALL.iter().map(|hp| (*hp, u.bd.lost(*hp))).filter(|x| x.1 > 0.05 && x.0 != crate::weapons::hitbox::HitPart::Armour).collect();
        parts.sort_by(|a, b| b.1.total_cmp(&a.1));
        p.text(Pos2::new(x, y), Align2::LEFT_CENTER, "DAMAGE", f_semi(11.5), pal::RED);
        y += 15.0;
        let s: Vec<String> = parts.iter().take(4).map(|(hp, f)| format!("{} {:.0}%", hp.name(), f * 100.0)).collect();
        let mut line_s = s.join(" · ");
        if u.bd.fire > 0.0 {
            line_s = format!("FIRE · {line_s}");
        }
        p.text(Pos2::new(x, y), Align2::LEFT_CENTER, line_s, f_ui(11.0), pal::AMBER);
        let _ = sim;
    }
}

/// RWR azimuth scope: emitters by bearing, threat level by ring.
fn rwr(p: &egui::Painter, screen: Rect, sim: &Sim, u: &Unit, c: &Combat) {
    let rad = 62.0;
    let cen = Pos2::new(screen.max.x - rad - sp::L, screen.max.y - rad - 230.0);
    super::panel(p, Rect::from_center_size(cen, EVec2::splat(rad * 2.0 + 16.0)), 64);
    let dim = Color32::from_rgba_unmultiplied(110, 255, 140, 90);
    for r in [rad * 0.33, rad * 0.66, rad] {
        p.circle_stroke(cen, r, Stroke::new(1.0, dim));
    }
    p.line_segment([cen - EVec2::new(4.0, 0.0), cen + EVec2::new(4.0, 0.0)], Stroke::new(1.0, dim));
    p.line_segment([cen - EVec2::new(0.0, 4.0), cen + EVec2::new(0.0, 4.0)], Stroke::new(1.0, dim));
    for e in &u.rwr.contacts {
        // Inner ring = most lethal (tracking / launch), outer = search.
        let rr = match e.kind {
            Emission::Search => rad * 0.82,
            Emission::Track => rad * 0.55,
            Emission::Launch | Emission::Active => rad * 0.3,
        };
        let at = super::polar(cen, rr, e.az);
        let label = if e.kind == Emission::Active {
            "M"
        } else {
            c.unit(e.from).map(|o| if c.defs[o.def].category == crate::flight::Category::FastJet { "16" } else { "U" }).unwrap_or("U")
        };
        let flash = matches!(e.kind, Emission::Launch | Emission::Active) && (sim.time * 4.0).fract() > 0.5;
        if flash {
            continue;
        }
        let col = if matches!(e.kind, Emission::Search) { pal::HUD } else { pal::AMBER };
        p.text(at, Align2::CENTER_CENTER, label, f_digits_b(13.0), col);
        if e.kind != Emission::Search {
            p.add(egui::Shape::closed_line(
                vec![at + EVec2::new(0.0, -11.0), at + EVec2::new(11.0, 0.0), at + EVec2::new(0.0, 11.0), at + EVec2::new(-11.0, 0.0)],
                Stroke::new(1.3, col),
            ));
        }
    }
    p.text(cen + EVec2::new(0.0, rad + 14.0), Align2::CENTER_CENTER, "RWR", f_semi(10.0), pal::TEXT3);
}

fn feed(p: &egui::Painter, screen: Rect, sim: &Sim, c: &Combat) {
    // Below the corner map (300 pt at the top right).
    let x = screen.max.x - sp::L;
    let mut y = sp::M + 300.0 + sp::M;
    if let Some(df) = &c.dogfight {
        let r = Rect::from_min_size(Pos2::new(x - 200.0, y), EVec2::new(200.0, 30.0));
        super::panel(p, r, sp::R_PANEL);
        p.text(r.center(), Align2::CENTER_CENTER, format!("{}  ·  {}  {} — {}", df.cfg.format.label(), df.cfg.difficulty.name(), df.score.0, df.score.1), f_semi(13.0), pal::TEXT);
        y += 38.0;
    }
    let now = c.time;
    for l in c.feed.iter().rev().filter(|l| now - l.t < 12.0).take(5) {
        let a = (1.0 - ((now - l.t) as f32 - 9.0).max(0.0) / 3.0).clamp(0.0, 1.0);
        let col = if l.mine { pal::AMBER } else { pal::TEXT };
        let col = Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), (255.0 * a) as u8);
        p.text(Pos2::new(x + 1.0, y + 1.0), Align2::RIGHT_TOP, &l.text, f_semi(13.0), Color32::from_black_alpha((120.0 * a) as u8));
        p.text(Pos2::new(x, y), Align2::RIGHT_TOP, &l.text, f_semi(13.0), col);
        y += 18.0;
    }
    let _ = sim;
}

/// Controls help rows for armed types.
pub fn control_rows(def: &crate::flight::AircraftDef) -> Vec<(&'static str, &'static str, String)> {
    let Some(l) = crate::weapons::guns::loadout(def) else { return Vec::new() };
    let mut v = vec![("Weapons", "Space (in the air)", "Fire: guns · missile in SRM / MRM".to_string())];
    if l.missiles.is_empty() {
        v.push(("Weapons", "Right Alt", "Cannon only (second button)".into()));
        v.push(("Weapons", "F5", "Guns: cannon + MG · cannon · MG".into()));
        v.push(("Weapons", "F9", "Harmonisation range".into()));
    } else {
        v.push(("Weapons", "F5", "Master mode: GUN · SRM · MRM".into()));
        v.push(("Weapons", "F7", "Radar lock · AIM-9 uncage".into()));
        v.push(("Weapons", "Right Alt", "Weapon release (pickle)".into()));
        v.push(("Weapons", "F8", "Flares + chaff".into()));
    }
    v.push(("Weapons", "F10 F10", if l.seat { "Eject".into() } else { "Bail out".into() }));
    v
}
