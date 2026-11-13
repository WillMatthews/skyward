//! Mountain fields for the powered bot: terrain-aware synthetic departures
//! (valley-following, engine-out gradient, climb-in-hold), descent holds and
//! straight / curved / circling-style approaches, planned by
//! `nav::procedures` over a terrain grid sampled around the field.
//!
//! Flat fields keep the standard visual pattern (bot.rs); a field counts as
//! "mountain" when the terrain within 15 km rises more than 450 m above it.

use super::*;
use crate::nav::procedures::{self as pr, ArrKind, Arrival, Departure, Grid, Hold, Perf};

/// Half-size (m) and cell (m) of the terrain grid sampled around a field.
const GRID_HALF: f32 = 40_000.0;
const GRID_CELL: f32 = 200.0;

/// Where the bot is in a planned arrival.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Stage {
    /// Flying to the hold / initial approach fix.
    ToFix,
    /// In the holding pattern (descending to the path's start).
    Hold,
    /// Flying the approach path (PPhase::Final).
    Path,
}

/// Planned procedures at a mountain field.
pub(super) struct Mtn {
    /// Terrain around the field: first from the world's resident tiles (the
    /// far / horizon levels are coarse: 380-760 m cells smooth the ridges),
    /// then — native builds — from the DEM at ~75 m in the background.
    pub grid: Grid,
    /// The fine DEM grid is in (or never coming: browser).
    pub fine: bool,
    fine_rx: Option<std::sync::mpsc::Receiver<Grid>>,
    /// World rebases when the fine grid was requested (a shift voids it).
    fine_rebases: u32,
    /// When the grid was sampled (s, bot time) — re-sampled while incomplete.
    pub grid_t: f32,
    pub dep: Option<Departure>,
    pub dep_i: usize,
    /// Climbing in the departure hold.
    pub dep_hold: bool,
    pub arr: Option<Arrival>,
    pub arr_i: usize,
    pub stage: Stage,
    /// The departure is a missed approach: approach again at its end.
    pub then_approach: bool,
    pub msa: f32,
    /// En-route climb hold over the field (slow climbers below a leg's floor).
    pub climb_hold: Option<Hold>,
}

impl Mtn {
    pub fn rebase(&mut self, r: &crate::world::Rebase) {
        let lo = self.grid.lo;
        self.grid.shift(r.ground(lo) - lo);
        let hold = |h: &mut Hold| {
            let f = r.ground(h.fix + h.inbound * 1000.0);
            h.fix = r.ground(h.fix);
            h.inbound = (f - h.fix).normalize_or(h.inbound);
        };
        if let Some(d) = &mut self.dep {
            for p in &mut d.path {
                *p = r.ground(*p);
            }
            if let Some(h) = &mut d.hold {
                hold(h);
            }
        }
        if let Some(a) = &mut self.arr {
            for p in &mut a.path {
                *p = r.ground(*p);
            }
            hold(&mut a.hold);
        }
        if let Some(h) = &mut self.climb_hold {
            hold(h);
        }
    }
}

/// Is `field` among mountains? (Terrain within 15 km > 450 m above it.)
pub(super) fn is_mountain(world: &World, field: &Runway) -> bool {
    let c = (field.a + field.b) * 0.5;
    let e = field.height_at(c);
    let mut top = f32::MIN;
    for k in 0..36 {
        for r in [2500.0, 5000.0, 8000.0, 11_000.0, 15_000.0] {
            let p = c + dir_of(k as f32 * TAU / 36.0) * r;
            top = top.max(world.ground_height(p));
        }
    }
    top - e > 450.0
}

/// DEM zoom of the background planning grid (Terrarium z10: ~100 m at 47°,
/// a dozen or so tiles for the whole survey — quick to fetch on a first visit).
#[cfg(not(target_arch = "wasm32"))]
const DEM_PLAN_Z: u32 = 10;

/// Build the fine planning grid from the DEM in a background thread
/// (downloads the tiles it needs; the bot waits for it at the holding point).
#[cfg(not(target_arch = "wasm32"))]
fn spawn_fine(world: &World, c: Vec2) -> Option<std::sync::mpsc::Receiver<Grid>> {
    let proj = world.proj;
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::Builder::new()
        .name("bot-terrain".into())
        .spawn(move || {
            let mut s = crate::terrain::Sampler::new(DEM_PLAN_Z, true);
            let g = Grid::build(
                |p| {
                    let (lat, lon) = proj.to_latlon(p);
                    s.missing = false;
                    let h = s.at(lat, lon);
                    (!s.missing).then_some(h)
                },
                c,
                GRID_HALF,
                GRID_CELL,
            );
            let _ = tx.send(g);
        })
        .ok()?;
    Some(rx)
}

#[cfg(target_arch = "wasm32")]
fn spawn_fine(_: &World, _: Vec2) -> Option<std::sync::mpsc::Receiver<Grid>> {
    None
}

/// A valley sightseeing loop for light types at a mountain field (None on
/// flat ground).
pub(super) fn valley_tour(world: &World, home: Vec2) -> Option<Vec<Landmark>> {
    let g = Grid::build(|q| world.ground_height_known(q), home, 14_000.0, 150.0);
    if pr::relief(&g, home, 8000.0) < 600.0 {
        return None;
    }
    let mut k = 0;
    let t: Vec<Landmark> = pr::valley_tour(&g, home, 3, 3500.0, 10_000.0, 0)
        .into_iter()
        .map(|(pos, main)| {
            if main {
                k += 1;
            }
            Landmark { name: if main { format!("Valley {k}") } else { "along the valley".into() }, pos }
        })
        .collect();
    (!t.is_empty()).then_some(t)
}

/// The terrain around `c`, as the planner reads it. (Buildings are left to
/// the bot's obstacle avoidance and the standard approach-slope check: at
/// 200 m sampling they would only be hit or missed by chance.)
pub(super) fn grid(world: &World, c: Vec2) -> Grid {
    Grid::build(|p| world.ground_height_known(p), c, GRID_HALF, GRID_CELL)
}

/// Climb gradients (all engines, one engine out) near the field at the
/// climb speed — from the published / calibrated climb rates in flight.rs's
/// tests (a light-to-mid weight), conservatively rounded down. Single-engine
/// types return the same number twice.
pub(super) fn climb_gradients(def: &AircraftDef) -> (f32, f32) {
    match def.id {
        "meridian320" => (0.075, 0.028),
        "meridian190" => (0.080, 0.030),
        "aerolink" => (0.065, 0.025),
        "hercules" => (0.060, 0.038),
        "galaxy" => (0.055, 0.032),
        "vulcan" => (0.120, 0.070),
        "dc3" => (0.055, 0.015),
        "otter" => (0.070, 0.070),
        "cub" => (0.090, 0.090),
        "extra500" => (0.090, 0.090),
        "zlin" => (0.110, 0.110),
        _ => match def.category {
            Category::FastJet => (0.20, 0.20),
            Category::Warbird => (0.12, 0.12),
            Category::Airliner => (0.07, 0.025),
            _ => (0.075, 0.075),
        },
    }
}

/// Holding / positioning speed (IAS m/s): the bot's Position speed.
pub(super) fn hold_ias(def: &AircraftDef, vapp: f32) -> f32 {
    (vapp + if def.category == Category::Airliner { 35.0 } else { 20.0 } * KT).max(1.3 * def.stall_speed(flap_lever(def, def.flap_detents.len().saturating_sub(2))))
}

/// Climb speed on a valley departure: V2 + 15 kt or so with the take-off
/// flaps kept out (tight turns in the valley, as the engine-out escape
/// routes are flown); light types at their climb speed.
pub(super) fn dep_ias(def: &AircraftDef, pf: &Profile) -> f32 {
    match def.category {
        Category::Airliner | Category::FastJet | Category::Warbird => (pf.vr + 25.0 * KT).min(pf.climb_ias),
        _ => pf.climb_ias,
    }
}

/// Highest IAS (m/s) at altitude `alt` that still flies a circle of radius
/// `r` over the ground at 20° of bank (the autopilot's 25° in reserve) with
/// the wind `w` (m/s) behind.
pub(super) fn ias_for_radius(r: f32, alt: f32, w: f32) -> f32 {
    crate::flight::cas_from_tas(((r * GRAVITY * 20f32.to_radians().tan()).sqrt() - w).max(30.0), alt)
}

/// The planner's view of the type. Turn radii are planned at less bank than
/// the autopilot's 25° (margin for wind and lag); holds at their TAS
/// `hold_h` m above the field (the bot slows down above that to fly them).
pub(super) fn perf(def: &AircraftDef, pf: &Profile, vapp: f32, elev: f32) -> Perf {
    let tas = |ias: f32, h: f32| crate::flight::tas_from_cas(ias, elev + h);
    let r = |v: f32, bank: f32| v * v / (GRAVITY * bank.to_radians().tan());
    let airliner = def.category == Category::Airliner;
    let (aeo, oei) = climb_gradients(def);
    let multi = def.engine_count > 1 && def.category != Category::FastJet;
    let v_hold = tas(hold_ias(def, vapp), 1800.0);
    let straight = match def.category {
        _ if heavy(def) => 4500.0,
        Category::Airliner if def.engine_kind == EngineKind::Jet => 3500.0,
        Category::Airliner => 3000.0,
        Category::FastJet => 3000.0,
        Category::Warbird => 2000.0,
        _ => 1500.0,
    };
    Perf {
        app_turn_r: r(tas(vapp + 15.0 * KT, 800.0), if airliner { 18.0 } else { 22.0 }),
        dep_turn_r: r(tas(dep_ias(def, pf), 1200.0), 20.0),
        hold_r: r(v_hold, 22.0),
        hold_leg: (v_hold * 60.0).max(1500.0),
        min_gp: 3f32.to_radians(),
        max_gp: pf.max_gs,
        straight_min: straight,
        final_dist: (pf.final_dist * 0.8).max(straight + 1000.0),
        climb_grad: if multi { oei } else { aeo * 0.6 },
        climb_grad_aeo: aeo,
        oei: multi,
        max_len: 45_000.0,
    }
}

impl Powered {
    /// The field's mountain state, (re)sampling the terrain grid when there
    /// is none yet or the last one was sampled while terrain still streamed.
    pub(super) fn mtn_ensure(&mut self, world: &World, t: f32) -> bool {
        let Some(f) = self.field.clone() else { return false };
        let stale = self.mtn.as_ref().is_none_or(|m| !m.grid.complete() && t - m.grid_t > 20.0);
        if !stale {
            return true;
        }
        if self.mtn.is_none() && (self.not_mountain || !is_mountain(world, &f)) {
            // (Decided once the terrain under the field is known.)
            self.not_mountain = world.ground_height_known((f.a + f.b) * 0.5).is_some();
            return false;
        }
        let c = (f.a + f.b) * 0.5;
        let g = grid(world, c);
        let msa = pr::msa(&g, c, 25_000.0);
        match &mut self.mtn {
            Some(m) => {
                m.grid = g;
                m.grid_t = t;
                m.msa = msa;
            }
            None => {
                let fine_rx = spawn_fine(world, c);
                self.mtn = Some(Mtn {
                    grid: g,
                    fine: fine_rx.is_none(),
                    fine_rx,
                    fine_rebases: world.rebases,
                    grid_t: t,
                    dep: None,
                    dep_i: 0,
                    dep_hold: false,
                    arr: None,
                    arr_i: 0,
                    stage: Stage::ToFix,
                    then_approach: false,
                    msa,
                    climb_hold: None,
                })
            }
        }
        true
    }

    /// Take in the fine terrain grid when it is ready; re-plan what is not
    /// yet being flown (the departure at the holding point, an arrival still
    /// on its way to the fix). True when the fine grid is (or will never be) in.
    pub(super) fn mtn_poll(&mut self, def: &AircraftDef, ac: &Aircraft, world: &World, t: f32, ev: &mut Vec<String>) -> bool {
        let Some(m) = &mut self.mtn else { return true };
        if m.fine {
            return true;
        }
        let got = match &m.fine_rx {
            Some(rx) => match rx.try_recv() {
                Ok(g) => Some(g),
                Err(std::sync::mpsc::TryRecvError::Empty) => return false,
                Err(_) => None,
            },
            None => None,
        };
        m.fine_rx = None;
        m.fine = true;
        let Some(mut g) = got else { return true };
        if world.rebases != m.fine_rebases {
            // (The origin moved meanwhile: the grid is in the old frame — ask again.)
            if let Some(f) = &self.field {
                let c = (f.a + f.b) * 0.5;
                m.fine = false;
                m.fine_rx = spawn_fine(world, c);
                m.fine_rebases = world.rebases;
            }
            return false;
        }
        g.max_with(&m.grid);
        let c = self.field.as_ref().map(|f| (f.a + f.b) * 0.5).unwrap_or_default();
        m.msa = pr::msa(&g, c, 25_000.0);
        m.grid = g;
        m.grid_t = t;
        ev.push(format!("terrain survey of {} complete (MSA {:.0} ft)", self.field_name, m.msa / FT));
        match self.phase {
            PPhase::Hold | PPhase::LineUp | PPhase::Taxi | PPhase::Turnaround => self.dep_planned = false,
            PPhase::Position if self.mtn.as_ref().is_some_and(|m| (m.arr.is_some() && m.stage == Stage::ToFix) || m.arr.is_none()) && self.phase_t >= 0.0 => {
                self.start_approach_any(def, world, ac, t, ev);
            }
            _ => {}
        }
        true
    }

    /// Altitude a departure climbs to before the tour: airliners, jets and
    /// heavies (IFR) the 25 km minimum safe altitude; light types (VFR, valley
    /// flying) high enough for every leg of the tour and the way home, at most
    /// the MSA.
    fn mtn_target(&self, def: &AircraftDef, world: &World) -> f32 {
        let Some(m) = &self.mtn else { return 0.0 };
        if matches!(def.category, Category::Airliner | Category::FastJet) {
            return m.msa.min(def.ceiling_m * 0.8);
        }
        let home = self.field.as_ref().map(|f| (f.a + f.b) * 0.5).unwrap_or_default();
        let fe = self.field.as_ref().map(|f| f.height_at(home)).unwrap_or(0.0);
        let mut pts = vec![home];
        pts.extend(self.tour.iter().map(|l| l.pos));
        pts.push(home);
        let mut need = fe + self.pf.tour_alt.min(600.0);
        for w in pts.windows(2) {
            need = need.max(route_floor(world, w[0], w[1], 40_000.0) + 300.0).max(pr::line_floor(&m.grid, w[0], w[1]) + 300.0);
        }
        need.min(m.msa).min(def.ceiling_m * 0.8)
    }

    /// Plan the departure from `end` (take-off direction) at a mountain field.
    pub(super) fn mtn_plan_departure(&mut self, def: &AircraftDef, world: &World, end: &RunwayEnd, t: f32, ev: &mut Vec<String>) {
        if !self.mtn_ensure(world, t) {
            return;
        }
        let target = self.mtn_target(def, world);
        let pf = perf(def, &self.pf, self.pf.vapp, end.elev);
        // (A climb hold is flown at the departure speed, up to the target.)
        let v = crate::flight::tas_from_cas(dep_ias(def, &self.pf), (end.elev + target) * 0.5);
        let r = v * v / (GRAVITY * 22f32.to_radians().tan());
        let pf = Perf { hold_r: r, hold_leg: (r * 1.2).max(1200.0), ..pf };
        let m = self.mtn.as_mut().unwrap();
        let mut dep = pr::plan_departure(&m.grid, end.far(), end.dir, end.elev_far, end.len * 0.4, target, &pf);
        if dep.is_none() && pf.oei {
            // No engine-out path: plan on the all-engines gradient (with a margin);
            // an engine failure would mean returning to the field.
            let pf2 = Perf { climb_grad: pf.climb_grad_aeo * 0.7, oei: false, ..pf.clone() };
            dep = pr::plan_departure(&m.grid, end.far(), end.dir, end.elev_far, end.len * 0.4, target, &pf2);
        }
        // The first leg of the tour starts where the departure ends: climb
        // high enough for it there (a climb hold if need be).
        let mut target = target;
        for _ in 0..2 {
            let (Some(d), Some(first)) = (&dep, self.tour.first()) else { break };
            let end_p = *d.path.last().unwrap();
            let need = (pr::line_floor(&m.grid, end_p, first.pos) + 300.0).min(m.msa);
            if need <= target + 50.0 {
                break;
            }
            target = need;
            let pf2 = Perf { climb_grad: d.grad, oei: d.oei, ..pf.clone() };
            if let Some(d2) = pr::plan_departure(&m.grid, end.far(), end.dir, end.elev_far, end.len * 0.4, target, &pf2) {
                dep = Some(d2);
            }
        }
        match &dep {
            Some(d) => ev.push(format!(
                "departure RWY {} (synthetic): {:.1} km, turns {:.0}°, {} {:.1} % gradient, {}{}",
                end.name,
                pr::length(&d.path) / 1000.0,
                d.turn.to_degrees(),
                if d.oei { "engine-out" } else { "all-engines" },
                d.grad * 100.0,
                match &d.hold {
                    Some(h) => format!("climb in the hold to {:.0} ft (hold clear from {:.0} ft)", d.target / FT, h.min_alt / FT),
                    None => format!("to {:.0} ft", d.target / FT),
                },
                if d.complete { "" } else { " (terrain still loading)" }
            )),
            None => ev.push(format!("departure RWY {}: no terrain-clear path found — straight out with terrain avoidance", end.name)),
        }
        m.dep = dep;
        m.dep_i = 0;
        m.dep_hold = false;
    }

    /// Choose a runway end and plan the approach at a mountain field.
    /// False: no plan (fly the standard pattern).
    pub(super) fn mtn_start_approach(&mut self, def: &AircraftDef, world: &World, t: f32, ev: &mut Vec<String>) -> bool {
        let Some(f) = self.field.clone() else { return false };
        if !self.mtn_ensure(world, t) {
            return false;
        }
        let w = world.wind.surface;
        let mut best: Option<(f32, RunwayEnd, Arrival, f32)> = None;
        for end in [RunwayEnd::of(&f, false), RunwayEnd::of(&f, true)] {
            let tail = (-headwind(w, end.dir)).max(0.0) / KT;
            if tail > 10.0 {
                continue;
            }
            let hw = headwind(w, end.dir).max(0.0);
            let vapp = self.pf.vapp + (0.5 * hw + world.wind.gust).min(15.0 * KT);
            let pf = perf(def, &self.pf, vapp, end.elev);
            let m = self.mtn.as_ref().unwrap();
            let Some(a) = pr::plan_arrival(&m.grid, end.thr, end.dir, end.elev, end.disp, self.pf.aim, threshold_height(def), &pf) else { continue };
            let cost = pr::length(&a.path) + 2500.0 * a.turn + 3000.0 * (a.gp.to_degrees() - 3.0) + tail * 1500.0 - hw * 300.0;
            if best.as_ref().is_none_or(|b| cost < b.0) {
                best = Some((cost, end, a, vapp));
            }
        }
        let Some((_, end, a, vapp)) = best else {
            ev.push("no terrain-clear approach found — standard pattern".into());
            return false;
        };
        ev.push(format!(
            "approach RWY {} (synthetic {}): {:.1} km from the hold, glidepath {:.1}°, straight final {:.1} km, turns {:.0}°, hold {} turns from {:.0} ft{}",
            end.name,
            match a.kind {
                ArrKind::Straight => "straight-in",
                ArrKind::Curved => "curved valley approach",
                ArrKind::Circling => "circling-type valley approach",
            },
            pr::length(&a.path) / 1000.0,
            a.gp.to_degrees(),
            a.straight / 1000.0,
            a.turn.to_degrees(),
            if a.hold.side > 0.0 { "right" } else { "left" },
            a.hold.min_alt / FT,
            if a.complete { "" } else { " (terrain still loading)" }
        ));
        self.gs_angle = a.gp;
        self.vapp = vapp;
        self.aim = a.aim;
        self.rw = Some(end);
        let m = self.mtn.as_mut().unwrap();
        m.arr = Some(a);
        m.arr_i = 0;
        m.stage = Stage::ToFix;
        self.orbit = false;
        self.phase = PPhase::Position;
        self.phase_t = t;
        true
    }

    /// Mountain field: the lowest altitude to fly from `p` (towards `to`, and
    /// 6 km along `vel`) over the surveyed terrain, 300 m clear (MIN when the
    /// point is outside the survey).
    pub(super) fn mtn_floor(&self, p: Vec2, to: Option<Vec2>, vel: Vec2) -> f32 {
        let Some(m) = &self.mtn else { return f32::MIN };
        if !m.grid.contains(p) {
            return f32::MIN;
        }
        // IFR types: never below the minimum safe altitude near the field
        // (until in the hold / on the procedure).
        if self.ifr && self.field.as_ref().is_some_and(|f| f.a.distance(p) < 30_000.0) {
            return m.msa;
        }
        let ahead = p + vel.normalize_or_zero() * 4000.0;
        let mut f = pr::line_floor(&m.grid, p, ahead);
        if let Some(to) = to {
            f = f.max(pr::line_floor(&m.grid, p, to));
        }
        f + 300.0
    }

    /// The surveyed floor (m) of the straight leg from `p` to `to` (+300 m).
    pub(super) fn mtn_leg_floor(&self, p: Vec2, to: Vec2) -> f32 {
        let Some(m) = &self.mtn else { return f32::MIN };
        if !m.grid.contains(p) {
            return f32::MIN;
        }
        pr::line_floor(&m.grid, p, to) + 300.0
    }

    /// En route below the floor a leg needs by more than 150 m: climb in a
    /// hold over the field first. (heading, altitude) while doing so.
    pub(super) fn mtn_climb_hold(&mut self, def: &AircraftDef, ac: &Aircraft, floor: f32, t: f32) -> Option<(f32, f32)> {
        let field = self.field.clone()?;
        let wind = self.wind;
        let climb = self.pf.climb_ias;
        let m = self.mtn.as_mut()?;
        let p = xz(ac.pos);
        let active = m.climb_hold.is_some();
        if ac.pos.y > floor - if active { 30.0 } else { 150.0 } {
            if m.climb_hold.take().is_some() {
                let _ = t;
            }
            return None;
        }
        if m.climb_hold.is_none() {
            // Here (along the way we are heading) if it is clear at this
            // height, else over the field (the valley the departure left).
            let v = crate::flight::tas_from_cas(climb, floor);
            let r = v * v / (GRAVITY * 22f32.to_radians().tan());
            let leg = (r * 1.5).max(1500.0);
            let c = (field.a + field.b) * 0.5;
            let u_rw = (field.b - field.a).normalize_or(vec2(0.0, -1.0));
            let u_here = dir_of(ac.heading());
            let mk = |fix: Vec2, u: Vec2, side: f32| {
                let mut h = Hold { fix, inbound: u, side, r, leg, min_alt: 0.0 };
                h.min_alt = h.need(&m.grid);
                h
            };
            let cands = [mk(p + u_here * r, u_here, 1.0), mk(p + u_here * r, u_here, -1.0), mk(c + u_rw * r, u_rw, 1.0), mk(c + u_rw * r, u_rw, -1.0)];
            let here = cands[..2].iter().filter(|h| h.min_alt <= ac.pos.y + 30.0).min_by(|a, b| a.min_alt.total_cmp(&b.min_alt));
            let pick = here.cloned().unwrap_or_else(|| if cands[3].min_alt < cands[2].min_alt { cands[3].clone() } else { cands[2].clone() });
            if std::env::var("SKYWARD_PROC_DEBUG").is_ok() {
                eprintln!(
                    "climb hold at {:?} r {:.0} leg {:.0} side {} min_alt {:.0} (alt {:.0}, floor {:.0}) here {:?}",
                    pick.fix,
                    pick.r,
                    pick.leg,
                    pick.side,
                    pick.min_alt,
                    ac.pos.y,
                    floor,
                    cands.iter().map(|h| h.min_alt as i32).collect::<Vec<_>>()
                );
            }
            m.climb_hold = Some(pick);
        }
        let h = m.climb_hold.as_ref().unwrap();
        let (trk, _) = h.steer(p);
        let _ = def;
        // (The surveyed hold is trusted: no building / terrain dodging inside it.)
        self.avoid = None;
        Some((crab(trk, ac.tas().max(20.0), wind), floor))
    }

    /// A planned arrival is being flown.
    pub(super) fn mtn_arriving(&self) -> bool {
        self.mtn.as_ref().is_some_and(|m| m.arr.is_some())
    }

    /// Position phase at a mountain field: to the fix, hold down, join the
    /// path. Returns (heading, altitude, speed, steep descent).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn mtn_position(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        world: &World,
        ev: &mut Vec<String>,
        status: &mut String,
        t: f32,
    ) -> (f32, f32, f32, bool) {
        let p = xz(ac.pos);
        let tas = ac.tas().max(20.0);
        let wind = self.wind;
        let vel = vec2(ac.vel.x, ac.vel.z);
        let rwname = self.rw.as_ref().map(|r| r.name.clone()).unwrap_or_default();
        let m = self.mtn.as_mut().unwrap();
        let a = m.arr.as_ref().unwrap();
        let fix = a.path[0];
        let start_alt = a.alt[0];
        let hold = a.hold.clone();
        let to_fix = p.distance(fix);
        let mut steep = false;
        let (hdg, alt);
        // Joining straight from the inbound side: lined up, at the path's height.
        let joinable = hold.inbound_at(p, ac.heading(), 500.0) && to_fix < 3000.0 && ac.pos.y < start_alt + 60.0;
        match m.stage {
            Stage::ToFix => {
                hdg = crab(heading_of(fix - p), tas, wind);
                // (Route floor to the fix, 600 m either side; the terrain ahead.)
                alt = start_alt
                    .max(if self.ifr { m.msa } else { f32::MIN })
                    .max(route_floor(world, p, fix, 40_000.0) + 300.0)
                    .max(terrain_floor(world, p, vel, 5000.0) + 300.0)
                    .max(pr::line_floor(&m.grid, p, fix) + 300.0)
                    .max(pr::line_floor(&m.grid, p, p + vel.normalize_or_zero() * 6000.0) + 300.0);
                *status = format!("Inbound to the hold for RWY {rwname} — {:.1} km", to_fix / 1000.0);
                if ac.pos.y < alt - 150.0 || m.climb_hold.is_some() {
                    // Too low for the way to the fix (a slow climber): climb in a hold first.
                    let need = alt;
                    if let Some((h, a)) = self.mtn_climb_hold(def, ac, need, t) {
                        *status = format!("Climbing in the hold to {:.0} ft before the approach", a / FT);
                        let floor = 1.3 * def.stall_speed(ac.surfaces().flaps);
                        return (h, a, self.pf.climb_ias.max(floor), false);
                    }
                    return (hdg, alt, hold_ias(def, self.vapp), false);
                }
                if to_fix < hold.r * 2.0 {
                    m.stage = Stage::Hold;
                    ev.push(format!("entering the hold for RWY {rwname} at {:.0} ft (path starts at {:.0} ft)", ac.pos.y / FT, start_alt / FT));
                }
            }
            Stage::Hold | Stage::Path => {
                let (trk, e) = hold.steer(p);
                if std::env::var("SKYWARD_PROC_DEBUG").is_ok() && (t * 1.0).fract() < 0.02 {
                    eprintln!(
                        "hold t {t:.0} p {:?} e {e:.0} alt {:.0} grid {:.0} wide {:.0} world {:.0} min_alt {:.0} hdg {:.0} trk {:.0} ias {:.0} gs {:.0} roll {:.0} r {:.0} wind {:?}",
                        p,
                        ac.pos.y,
                        m.grid.at(p),
                        m.grid.wide(p),
                        world.ground_height(p),
                        hold.min_alt,
                        ac.heading().to_degrees(),
                        trk.to_degrees(),
                        ac.airspeed() / KT,
                        gs(ac),
                        ac.roll().to_degrees(),
                        hold.r,
                        wind
                    );
                }
                hdg = crab(trk, tas, wind);
                alt = start_alt.max(hold.min_alt);
                steep = ac.pos.y > alt + 300.0;
                *status = format!("Holding for RWY {rwname} — {:.0} ft to lose", ((ac.pos.y - start_alt) / FT).max(0.0));
                // Leave on the inbound leg once down to the path's start.
                if hold.inbound_at(p, ac.heading(), 600.0) && ac.pos.y < start_alt + 80.0 {
                    m.stage = Stage::Path;
                }
            }
        }
        if joinable || m.stage == Stage::Path {
            m.stage = Stage::Path;
            m.arr_i = 0;
            ev.push(format!("leaving the hold: approach RWY {rwname} at {:.0} ft", ac.pos.y / FT));
            self.set_phase(PPhase::Final, t);
        }
        // Approach flaps from the hold on (the hold is flown at approach speed).
        if self.mtn.as_ref().is_some_and(|m| m.stage != Stage::ToFix) || to_fix < 15_000.0 {
            self.set_flaps(def, ctl, self.pf.app_flap, t);
        } else {
            ctl.flaps = flap_lever(def, self.flap_idx);
        }
        self.avoid = self.avoid.filter(|_| self.mtn.as_ref().is_some_and(|m| m.stage == Stage::ToFix));
        // In the hold: slow enough to fly its turns (planned at a lower
        // altitude's TAS), never below 1.3 Vs of the flaps actually out.
        let floor = 1.3 * def.stall_speed(ac.surfaces().flaps);
        let in_hold = self.mtn.as_ref().is_some_and(|m| m.stage != Stage::ToFix);
        let cap = if in_hold { ias_for_radius(hold.r, ac.pos.y, wind.length()) } else { f32::MAX };
        let spd = hold_ias(def, self.vapp).min(cap).max(floor);
        (hdg, alt, spd, steep)
    }

    /// On a planned arrival path: (distance to the threshold, cross-track,
    /// course to fly, glidepath height above `fe`, on the straight final).
    pub(super) fn mtn_final_geom(&mut self, p: Vec2, gs: f32, fe: f32) -> Option<(f32, f32, f32, f32, bool)> {
        let aim = self.aim;
        let m = self.mtn.as_mut()?;
        if m.stage != Stage::Path {
            return None;
        }
        let a = m.arr.as_ref()?;
        let pos = pr::locate(&a.path, p, m.arr_i);
        m.arr_i = pos.i;
        let total = pr::length(&a.path);
        let remain = total - pos.along + aim;
        let s = remain - aim;
        let elev = a.alt.last().map(|h| h - aim * a.gp.tan()).unwrap_or(fe);
        let gp = elev + remain.max(0.0) * a.gp.tan() - fe;
        // Lead the turns: the course a few seconds ahead along the path.
        let lead = (gs * 5.0).max(200.0);
        let target = pos.along + lead;
        let mut crs = pos.track;
        let mut seg_start = 0.0;
        for i in 0..a.path.len().saturating_sub(1) {
            let len = a.path[i].distance(a.path[i + 1]);
            if seg_start + len >= target && i >= pos.i {
                crs = heading_of(a.path[i + 1] - a.path[i]);
                break;
            }
            seg_start += len;
        }
        // (Blend: half the current segment, half the one ahead.)
        crs = pos.track + wrap_pi(crs - pos.track) * 0.5;
        Some((s, pos.cross, crs, gp, remain <= a.straight + 50.0))
    }

    /// Departure / missed approach at a mountain field: along the planned
    /// path, then (if planned) climbing in the hold to the safe altitude.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn mtn_departure(
        &mut self,
        def: &AircraftDef,
        ac: &Aircraft,
        ctl: &mut Controls,
        ap: &mut Autopilot,
        world: &World,
        ev: &mut Vec<String>,
        status: &mut String,
        t: f32,
        dt: f32,
    ) {
        let p = xz(ac.pos);
        let tas = ac.tas().max(20.0);
        let wind = self.wind;
        if !ap.engaged {
            ap.engage_hold_current(ac);
        }
        ctl.reverse = false;
        ctl.spoilers = 0.0;
        ctl.brakes = 0.0;
        if ac.vertical_speed() > 1.0 {
            ctl.gear_down = false;
        }
        // Take-off flaps kept for the whole procedure (flown at V2 + 25 kt);
        // retracted en route.
        let to_flap = if self.pf.flap_up_ias > 0.0 { self.pf.to_flap } else { 0 };
        self.set_flaps(def, ctl, to_flap, t);
        let mut spd = dep_ias(def, &self.pf);
        let Some(m) = self.mtn.as_mut() else {
            self.set_phase(PPhase::Enroute, t);
            return;
        };
        let Some(d) = m.dep.as_ref() else {
            self.set_phase(PPhase::Enroute, t);
            return;
        };
        let target = d.target;
        let hdg;
        if !m.dep_hold {
            let pos = pr::locate(&d.path, p, m.dep_i);
            m.dep_i = pos.i;
            let ahead = (pos.i + 1).min(d.path.len().saturating_sub(2));
            let crs_next = heading_of(d.path[ahead + 1] - d.path[ahead]);
            let crs = pos.track + wrap_pi(crs_next - pos.track) * 0.5;
            hdg = crab(crs + (-pos.cross / 600.0).atan().clamp(-0.6, 0.6), tas, wind);
            let left = pr::length(&d.path) - pos.along;
            *status = format!("{} — {:.0} ft, {:.1} km to go", if m.then_approach { "Missed approach" } else { "Valley departure" }, ac.pos.y / FT, left / 1000.0);
            if left < 300.0 {
                if d.hold.is_some() && ac.pos.y < target - 60.0 {
                    m.dep_hold = true;
                    ev.push(format!("climbing in the hold to {:.0} ft", target / FT));
                } else if ac.pos.y < target - 60.0 {
                    // Past the end below the target: hold here (a racetrack along the last leg).
                    let u = (d.path[d.path.len() - 1] - d.path[d.path.len() - 2]).normalize_or(dir_of(ac.heading()));
                    let pf = perf(def, &self.pf, self.vapp, 0.0);
                    let mut h = Hold { fix: p + u * pf.hold_leg * 0.5, inbound: u, side: 1.0, r: pf.hold_r, leg: pf.hold_leg, min_alt: 0.0 };
                    h.min_alt = h.need(&m.grid);
                    let dep = m.dep.as_mut().unwrap();
                    dep.hold = Some(h);
                    m.dep_hold = true;
                    ev.push(format!("end of the departure path below {:.0} ft: climbing in a hold", target / FT));
                }
            }
        } else {
            let h = d.hold.as_ref().unwrap();
            let (trk, _) = h.steer(p);
            hdg = crab(trk, tas, wind);
            *status = format!("Climbing in the hold — {:.0} ft to {:.0} ft", ac.pos.y / FT, target / FT);
        }
        if let (true, Some(h)) = (m.dep_hold, m.dep.as_ref().and_then(|d| d.hold.as_ref())) {
            spd = spd.min(ias_for_radius(h.r, ac.pos.y, wind.length()));
        }
        let spd = spd.max(1.25 * def.stall_speed(ac.surfaces().flaps));
        let done = ac.pos.y >= target - 60.0;
        ap.hdg = Some(wrap_pi(hdg));
        ap.alt = Some(target.max(ac.pos.y.min(target)));
        ap.vs = None;
        ap.ias = Some(spd);
        if !ap.update(def, ac, ctl, dt) {
            ap.engage_hold_current(ac);
        }
        if done {
            let again = m.then_approach;
            m.then_approach = false;
            m.dep = None;
            m.dep_hold = false;
            ev.push(format!("departure complete at {:.0} ft", ac.pos.y / FT));
            if again {
                self.start_approach_any(def, world, ac, t, ev);
            } else {
                self.set_phase(PPhase::Enroute, t);
            }
        }
    }
}

/// Smallest clearance (m) of the aircraft over the terrain / buildings now.
pub(super) fn clearance(world: &World, p: Vec3) -> f32 {
    let q = xz(p);
    p.y - world.obstacle_top(q).unwrap_or(f32::MIN).max(world.ground_height(q))
}
