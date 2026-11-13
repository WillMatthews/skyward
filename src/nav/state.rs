//! Per-flight navigation state owned by the sim: the radios, the flight
//! plan, the stations around the aircraft, and the inputs the autopilot /
//! flight director couple to.

use super::db::{self, NavDb, Navaid};
use super::fms::{ApprKind, Approach, Plan, Wpt, WptKind};
use super::geo::{self, LL};
use super::radio::{RadioEnv, Radios};
use crate::autopilot::{Beam, NavInput, Path};
use crate::flight::{Aircraft, AircraftDef, Category};
use glam::{Vec2, vec2};

/// The world as the radios see it.
pub struct WorldEnv<'a>(pub &'a crate::world::World);

impl RadioEnv for WorldEnv<'_> {
    fn to_world(&self, ll: LL) -> Vec2 {
        self.0.proj.to_world(ll.0, ll.1)
    }
    fn to_ll(&self, p: Vec2) -> LL {
        self.0.proj.to_latlon(p)
    }
    fn ground(&self, p: Vec2) -> Option<f32> {
        Some(crate::flight::Environment::ground_height(self.0, p))
    }
    fn runway_elev(&self, airport: &str, rwy: &str) -> Option<f32> {
        let r = self.0.runways.iter().find(|r| r.airport == airport && (r.ident_a == rwy || r.ident_b == rwy) && r.elev_known)?;
        let end = if r.ident_a == rwy { r.a } else { r.b };
        Some(r.height_at(end))
    }
}

/// Navigation display mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NdMode {
    /// Compass rose with the ILS deviation (NAV 1 localizer / glideslope).
    RoseIls,
    /// Compass rose with the VOR course and deviation bar.
    RoseVor,
    /// Compass rose with the flight plan.
    RoseNav,
    /// Forward arc map with the flight plan.
    #[default]
    Arc,
    /// North-up plan view of the flight plan around the active waypoint.
    Plan,
}

impl NdMode {
    pub fn next(self) -> NdMode {
        match self {
            NdMode::RoseIls => NdMode::RoseVor,
            NdMode::RoseVor => NdMode::RoseNav,
            NdMode::RoseNav => NdMode::Arc,
            NdMode::Arc => NdMode::Plan,
            NdMode::Plan => NdMode::RoseIls,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            NdMode::RoseIls => "ROSE ILS",
            NdMode::RoseVor => "ROSE VOR",
            NdMode::RoseNav => "ROSE NAV",
            NdMode::Arc => "ARC",
            NdMode::Plan => "PLAN",
        }
    }
}

/// Radios, flight plan and display modes of one flight.
#[derive(Clone, Debug)]
pub struct NavState {
    pub nd_mode: NdMode,
    pub radios: Radios,
    pub plan: Plan,
    /// Stations around the aircraft (VOR / DME / NDB + ILS), refreshed as it moves.
    local: Vec<Navaid>,
    local_at: Option<LL>,
    local_t: f32,
    local_db: bool,
    /// Tune NAV 1 to the approach's ILS automatically.
    pub auto_tune: bool,
    /// Has the radio type (no radios in a glider / the Spitfire).
    pub equipped: bool,
    /// ND / map: show the navaids around.
    pub show_navaids: bool,
    tuned_for: Option<String>,
}

impl Default for NavState {
    fn default() -> Self {
        NavState {
            nd_mode: NdMode::Arc,
            radios: Radios::default(),
            plan: Plan::default(),
            local: Vec::new(),
            local_at: None,
            local_t: 0.0,
            local_db: false,
            auto_tune: true,
            equipped: true,
            show_navaids: true,
            tuned_for: None,
        }
    }
}

/// Does the type carry nav radios?
pub fn has_radios(def: &AircraftDef) -> bool {
    !matches!(def.category, Category::Glider | Category::Warbird)
}

impl NavState {
    pub fn new(def: &AircraftDef) -> NavState {
        let mut s = NavState { equipped: has_radios(def), ..Default::default() };
        s.plan.cruise = default_cruise(def);
        s
    }

    pub fn local(&self) -> &[Navaid] {
        &self.local
    }

    /// Navaids near a point (for the map / ND), nearest first.
    pub fn navaids_near(&self, env: &dyn RadioEnv, p: Vec2, r: f32) -> Vec<&Navaid> {
        let mut v: Vec<(&Navaid, f32)> = self.local.iter().map(|n| (n, env.to_world(n.ll()).distance(p))).filter(|x| x.1 < r).collect();
        v.sort_by(|a, b| a.1.total_cmp(&b.1));
        v.into_iter().map(|x| x.0).collect()
    }

    /// Refresh the station list: navaids within 300 km, ILS within 80 km
    /// and the planned approach's ILS.
    fn refresh(&mut self, env: &dyn RadioEnv, p: Vec2, ndb: Option<&NavDb>, adb: Option<&crate::airports::Db>) {
        let ll = env.to_ll(p);
        let mut v: Vec<Navaid> = Vec::new();
        if let Some(d) = ndb {
            v.extend(d.near(ll, 300.0).into_iter().map(|(i, _)| d.navaids[i].clone()));
        }
        if let Some(a) = adb {
            v.extend(db::ils_near(a, ll, 80.0));
        }
        if let Some(i) = self.plan.approach.as_ref().and_then(|a| a.ils.clone()) {
            if !v.iter().any(|n| n.ident == i.ident && n.freq_khz == i.freq_khz) {
                v.push(i);
            }
        }
        // A synthetic glidepath must clear the terrain under its sector: else
        // the facility is a localizer only (a published one would be offset or steeper).
        for n in v.iter_mut() {
            if n.ils.as_ref().is_some_and(|i| i.synthetic && i.gs.is_some()) && !glidepath_clear(env, n) {
                if let Some(i) = n.ils.as_mut() {
                    i.gs = None;
                }
                n.kind = db::Kind::Loc;
            }
        }
        self.local = v;
        self.local_at = Some(ll);
        self.local_db = ndb.is_some();
    }

    /// Put stations in directly (tests / no database).
    pub fn add_stations(&mut self, s: impl IntoIterator<Item = Navaid>) {
        self.local.extend(s);
    }

    /// One physics step: refresh the stations now and then, run the radios,
    /// tune the approach's ILS, sequence the flight plan.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, env: &dyn RadioEnv, ac: &Aircraft, def: &AircraftDef, ndb: Option<&NavDb>, adb: Option<&crate::airports::Db>, dt: f32) {
        let p = vec2(ac.pos.x, ac.pos.z);
        self.local_t -= dt;
        if self.local_t <= 0.0 {
            self.local_t = 5.0;
            let moved = self.local_at.is_none_or(|ll| geo::gc_dist(ll, env.to_ll(p)) > 20_000.0);
            if moved || (ndb.is_some() && !self.local_db) || self.approach_missing() {
                if ndb.is_some() || adb.is_some() {
                    self.refresh(env, p, ndb, adb);
                }
            }
        }
        // Power and failures (systems/).
        let sys = &ac.sys;
        self.radios.powered = self.equipped && (!sys.init || sys.bus_ok());
        for k in 0..2 {
            self.radios.nav_failed[k] = sys.init && sys.fail.has(crate::systems::failures::Failure::new(crate::systems::failures::Kind::NavRadio, k));
            self.radios.gs_failed[k] = sys.init && sys.fail.has(crate::systems::failures::Failure::new(crate::systems::failures::Kind::GlideSlope, k));
        }
        self.radios.adf_failed = sys.init && sys.fail.any_of(crate::systems::failures::Kind::Adf);
        // Auto-tune the approach's ILS on NAV 1.
        if self.auto_tune {
            let want =
                self.plan.approach.as_ref().and_then(|a| a.ils.as_ref()).map(|n| (n.freq_khz, n.ils.as_ref().map_or(0.0, |i| i.course), format!("{}{}", n.airport, n.ident)));
            if let Some((f, crs, key)) = want {
                if self.tuned_for.as_ref() != Some(&key) {
                    self.radios.nav[0].tune(f);
                    self.radios.nav[0].obs = crs;
                    self.tuned_for = Some(key);
                }
            }
        }
        self.radios.step(env, &self.local, ac.pos, dt);
        let gs = vec2(ac.vel.x, ac.vel.z).length();
        let bank = if def.category == Category::Airliner { 25f32 } else { 30f32 }.to_radians() * 0.75;
        self.plan.sequence(env, p, gs, bank);
        let _ = def;
    }

    fn approach_missing(&self) -> bool {
        self.plan.approach.as_ref().and_then(|a| a.ils.as_ref()).is_some_and(|i| !self.local.iter().any(|n| n.ident == i.ident && n.freq_khz == i.freq_khz))
    }

    /// The autopilot's navigation inputs at this instant.
    pub fn ap_input(&self, env: &dyn RadioEnv, ac: &Aircraft, def: &AircraftDef, ground_y: f32) -> NavInput {
        let p = vec2(ac.pos.x, ac.pos.z);
        let o = &self.radios.nav[0].out;
        let beam = |valid: bool| Beam { course: o.course, xtk: o.xtk, valid, to_thr: o.to_thr, thr_elev: o.thr_elev, next: None };
        let loc = (o.loc && o.valid).then(|| beam(true));
        let vor = (!o.loc && o.valid).then(|| beam(true));
        let gs = (o.loc && o.gs_valid).then_some(Path { err: o.gp_err, angle: o.gs_angle, valid: true, cruise: 0.0, alt: 0.0 });
        let lnav = self.plan.lnav(env, p).map(|g| Beam { course: g.track, xtk: g.xtk, valid: true, to_thr: 0.0, thr_elev: 0.0, next: g.next.map(|n| (n, g.to_go)) });
        let vnav = self.plan.vnav(env, p).map(|v| Path { err: ac.pos.y - v.alt, angle: v.angle, valid: true, cruise: v.cruise, alt: v.alt });
        NavInput { loc, vor, gs, lnav, vnav, radio_alt: ac.pos.y - def.gear_height - ground_y }
    }

    /// Plan an approach to runway end `rwy` of `airport` (ILS if it has one,
    /// else RNAV on the same geometry) as the destination.
    pub fn plan_approach(&mut self, adb: &crate::airports::Db, airport: &str, rwy: &str, kind: ApprKind) -> Result<String, String> {
        let ap = adb.get(airport).ok_or_else(|| format!("{airport}: unknown airport"))?;
        let ils = db::ils_at(ap).into_iter().find(|n| n.ils.as_ref().is_some_and(|i| i.runway.eq_ignore_ascii_case(rwy)));
        let ils = match ils {
            Some(i) => i,
            None => {
                // A non-instrument runway: an RNAV approach on its geometry.
                let rw = ap.runways.iter().find(|r| r.ends.iter().any(|e| e.ident.eq_ignore_ascii_case(rwy))).ok_or_else(|| format!("{} has no runway {rwy}", ap.code()))?;
                let e = rw.ends.iter().position(|e| e.ident.eq_ignore_ascii_case(rwy)).unwrap_or(0);
                rnav_geometry(ap, rw, e).ok_or_else(|| format!("{} {rwy}: runway ends not surveyed", ap.code()))?
            }
        };
        let kind = if ils.ils.as_ref().is_some_and(|i| i.synthetic && ils.kind == db::Kind::Loc) { ApprKind::Rnav } else { kind };
        let appr = Approach::from_ils(&ils, kind).ok_or("no approach")?;
        let name = appr.name();
        if self.plan.dest.as_ref().is_none_or(|d| d.ident != ap.ident) {
            self.plan.set_dest(Some(Wpt::new(ap.code(), (ap.lat, ap.lon), WptKind::Airport)));
        }
        self.plan.set_approach(Some(appr));
        if self.plan.active >= self.plan.legs.len() {
            self.plan.active = 0;
        }
        self.tuned_for = None;
        if kind == ApprKind::Ils && !self.local.iter().any(|n| n.ident == ils.ident && n.freq_khz == ils.freq_khz) {
            self.local.push(ils);
        }
        Ok(format!("{} {}", ap.code(), name))
    }

    /// Build a flight plan from a spec: `WPT,WPT,...,ICAO/RWY[/RNAV]` —
    /// navaid idents, airport codes or `lat:lon`, the last item an approach.
    pub fn route_from_spec(&mut self, spec: &str, here: LL, ndb: Option<&NavDb>, adb: Option<&crate::airports::Db>) -> Result<String, String> {
        let mut out = Vec::new();
        for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let parts: Vec<&str> = item.split('/').collect();
            if parts.len() >= 2 {
                let adb = adb.ok_or("airport database not loaded")?;
                let kind = if parts.get(2).is_some_and(|k| k.eq_ignore_ascii_case("rnav")) { ApprKind::Rnav } else { ApprKind::Ils };
                out.push(self.plan_approach(adb, &parts[0].to_ascii_uppercase(), &parts[1].to_ascii_uppercase(), kind)?);
                continue;
            }
            let w = if let Some((a, b)) = item.split_once(':').and_then(|(a, b)| Some((a.parse::<f64>().ok()?, b.parse::<f64>().ok()?))) {
                Wpt::new(format!("WP{:02}", self.plan.legs.len() + 1), (a, b), WptKind::User)
            } else if let Some(n) = ndb.and_then(|d| d.find(item, here)) {
                Wpt::new(n.ident.clone(), n.ll(), WptKind::Navaid)
            } else if let Some(a) = adb.and_then(|d| d.get(item)) {
                Wpt::new(a.code(), (a.lat, a.lon), WptKind::Airport)
            } else {
                return Err(format!("{item}: not found"));
            };
            out.push(w.ident.clone());
            let n = self.plan.enroute_len();
            self.plan.insert(n, w);
        }
        self.plan.active = 0;
        self.plan.from = Some(here);
        Ok(out.join(" "))
    }

    /// Tune NAV 1 to the ILS of a runway end, if one is in range of the list.
    pub fn tune_runway(&mut self, airport: &str, rwy: &str) {
        let Some(n) = self.local.iter().find(|n| n.ils.as_ref().is_some_and(|i| i.airport == airport && i.runway == rwy)) else { return };
        let (f, crs) = (n.freq_khz, n.ils.as_ref().map_or(0.0, |i| i.course));
        if self.radios.nav[0].active != f {
            self.radios.nav[0].tune(f);
            self.radios.nav[0].obs = crs;
        }
    }

    /// Plan an approach from an ILS record directly (tests).
    pub fn plan_ils(&mut self, ils: &Navaid, kind: ApprKind) {
        if let Some(a) = Approach::from_ils(ils, kind) {
            self.plan.set_approach(Some(a));
            self.tuned_for = None;
            if !self.local.iter().any(|n| n.ident == ils.ident && n.freq_khz == ils.freq_khz) {
                self.local.push(ils.clone());
            }
        }
    }

    /// Distance to go and time / fuel at the destination (m, s, kg).
    pub fn dest_prediction(&self, env: &dyn RadioEnv, ac: &Aircraft) -> Option<(f32, f32, Option<f32>)> {
        let p = vec2(ac.pos.x, ac.pos.z);
        let gs = vec2(ac.vel.x, ac.vel.z).length();
        let (fob, flow) = fuel(ac);
        let preds = self.plan.predictions(env, p, gs, fob, flow);
        preds.last().map(|l| (l.dist, l.eta, l.efob))
    }
}

/// Does the glidepath of `n` clear the terrain (with a margin growing from
/// 30 m at the threshold to ~400 m at 10 NM) across the localizer sector?
pub fn glidepath_clear(env: &dyn RadioEnv, n: &Navaid) -> bool {
    let Some(ils) = &n.ils else { return true };
    let (_, dir) = super::radio::loc_frame(env, ils);
    let thr = env.to_world(ils.thr);
    let elev = env.runway_elev(&ils.airport, &ils.runway).unwrap_or(ils.thr_elev);
    let side = geo::right_of(dir);
    let tan = ils.gs_angle.tan();
    let mut d = 900.0;
    while d <= 10.0 * geo::NM {
        let path = elev + 15.0 + d * tan;
        let margin = 30.0 + 0.02 * d;
        let half = 150.0 + 0.1 * d;
        for k in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
            let p = thr - dir * d + side * half * k;
            if env.ground(p).is_some_and(|g| g + margin > path) {
                return false;
            }
        }
        d += 400.0;
    }
    true
}

/// Fuel on board (kg, when the systems model it) and total flow (kg/h).
pub fn fuel(ac: &Aircraft) -> (Option<f32>, f32) {
    let sys = &ac.sys;
    let fob = (sys.init && !sys.sd.tanks.is_empty()).then(|| sys.fuel_total());
    let flow = (0..crate::flight::MAX_ENGINES).map(|k| ac.fuel_flow(k)).sum::<f32>();
    (fob, flow)
}

/// A typical cruise level (m) for the type.
pub fn default_cruise(def: &AircraftDef) -> f32 {
    let ft = 0.3048;
    ft * match def.category {
        Category::Trainer => 4500.0,
        Category::Glider => 3000.0,
        Category::Warbird => 8000.0,
        Category::FastJet => 25_000.0,
        Category::Airliner => {
            if def.engine_kind == crate::flight::EngineKind::Jet {
                12_000.0
            } else {
                9_000.0
            }
        }
    }
}

/// An approach on a runway without an ILS: the same geometry, flagged as a localizer-less RNAV.
fn rnav_geometry(ap: &crate::airports::Airport, rw: &crate::airports::Rwy, e: usize) -> Option<Navaid> {
    let (a, b) = (&rw.ends[e], &rw.ends[1 - e]);
    let pa = (a.lat?, a.lon?);
    let pb = (b.lat?, b.lon?);
    let course = geo::gc_bearing(pa, pb);
    let thr = geo::gc_dest(pa, course, a.displaced_m as f64);
    let elev = ap.elev_ft * 0.3048;
    Some(Navaid {
        kind: db::Kind::Loc,
        ident: format!("R{}", a.ident),
        name: format!("{} RWY {}", ap.name, a.ident),
        freq_khz: 0,
        lat: pb.0,
        lon: pb.1,
        elev_m: elev,
        dme: None,
        range_nm: 0.0,
        airport: ap.ident.clone(),
        ils: Some(db::Ils {
            airport: ap.ident.clone(),
            runway: a.ident.clone(),
            thr,
            thr_elev: elev,
            course,
            loc: pb,
            gs: None,
            gs_angle: 3f32.to_radians(),
            half_width: 2f32.to_radians(),
            length: rw.length_m - a.displaced_m,
            synthetic: true,
        }),
    })
}
