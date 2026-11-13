//! Warnings, cautions and advisories: ECAM / EICAS messages with their
//! procedures (lines tick off as the crew does them), the F-16 caution
//! lights, light-aircraft annunciators. Computed from the systems state on
//! demand (displays, the master lights and the aural alerts).

use super::defs::Alerting;
use super::failures::Kind;
use super::{Power, Selector, Systems};
use crate::flight::{Aircraft, AircraftDef, Controls, EngineKind};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Warning,
    Caution,
    Advisory,
}

/// One procedure line: "- THR LEVER 1 .......... IDLE".
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub action: String,
    pub value: String,
    /// The crew has done it (or it no longer applies).
    pub done: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Alert {
    pub id: u32,
    pub level: Level,
    pub title: String,
    pub lines: Vec<Line>,
    pub fire: bool,
    pub smoke: bool,
}

fn fnv(s: &str) -> u32 {
    let mut h: u32 = 0x811C_9DC5;
    for b in s.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    h.max(1)
}

struct Out(Vec<Alert>);

impl Out {
    fn add(&mut self, level: Level, title: impl Into<String>, lines: Vec<Line>) -> &mut Alert {
        let title = title.into();
        self.0.push(Alert { id: fnv(&title), level, title, lines, fire: false, smoke: false });
        self.0.last_mut().unwrap()
    }
}

fn l(action: &str, value: &str, done: bool) -> Line {
    Line { action: action.to_string(), value: value.to_string(), done }
}

/// Every alert active now, warnings first.
pub fn alerts(sys: &Systems, def: &AircraftDef, ac: &Aircraft, ctl: &Controls) -> Vec<Alert> {
    let mut o = Out(Vec::new());
    if !sys.init || ac.crashed.is_some() {
        return o.0;
    }
    let sd = &sys.sd;
    let style = sd.alerting;
    let n = sys.n_eng;
    let airborne = !ac.on_ground;
    let multi = n > 1;
    let tag = |i: usize| if multi { format!(" {}", i + 1) } else { String::new() };
    let any_running = (0..n).any(|i| sys.eng[i].run);

    // ---------------- engines
    for i in 0..n {
        let e = &sys.eng[i];
        let k = tag(i);
        let master = sys.sw.eng_master[i];
        let handle = sys.sw.fire_handle[i];
        // (Split throttles: this engine's own lever.)
        let idle = ctl.lever(i) < 0.05;
        if e.fire > 0.0 && sd.fire_detect {
            let lines = match style {
                Alerting::Ecam => {
                    let mut v = vec![
                        l(&format!("THR LEVER{k}"), "IDLE", idle || !master),
                        l(&format!("ENG MASTER{k}"), "OFF", !master),
                        l(&format!("ENG{k} FIRE P/B"), "PUSH", handle),
                        l("AGENT 1 AFTER 10 S", "DISCH", e.agent >= 1),
                    ];
                    if sd.bottles >= 2 {
                        v.push(l("IF FIRE AFTER 30 S: AGENT 2", "DISCH", e.agent >= 2));
                    }
                    v
                }
                Alerting::Atr => vec![
                    l(&format!("PL{k}"), "FI", idle || !master),
                    l(&format!("CL{k}"), "FTR THEN FUEL SO", !master),
                    l(&format!("FIRE HANDLE{k}"), "PULL", handle),
                    l("AGENT 1 AFTER 10 S", "DISCH", e.agent >= 1),
                    l("IF FIRE AFTER 30 S: AGENT 2", "DISCH", e.agent >= 2),
                ],
                Alerting::Eicas => vec![
                    l(&format!("THRUST LEVER{k}"), "IDLE", idle || !master),
                    l(&format!("START/STOP SEL{k}"), "STOP", !master),
                    l(&format!("FIRE HANDLE{k}"), "PULL", handle),
                    l(&format!("FIRE HANDLE{k}"), "ROTATE (BOTTLE 1)", e.agent >= 1),
                    l("IF FIRE PERSISTS AFTER 30 S", "BOTTLE 2", e.agent >= 2),
                ],
                _ => vec![l("THROTTLE", "MIN PRACTICAL", idle), l("IF FIRE PERSISTS", "EJECT", false), l("ENGINE", "OFF IF FIRE CONFIRMED", !master)],
            };
            let title = match style {
                Alerting::Vms => "ENG FIRE".to_string(),
                _ => format!("ENG{k} FIRE"),
            };
            o.add(Level::Warning, title, lines).fire = true;
            continue;
        }
        let starting = e.start > 0.0 || e.crank > 0.0;
        if !e.run && master && !starting && (airborne || any_running || e.dead) && def.has_engine {
            match style {
                Alerting::Ecam | Alerting::Atr => {
                    let lines = if e.dead {
                        vec![l(&format!("ENG MASTER{k}"), "OFF", !master), l("LAND ASAP", "", false)]
                    } else {
                        vec![
                            l("ENG MODE SEL", "IGN", sys.sw.ign_start),
                            l(&format!("THR LEVER{k}"), "IDLE", idle),
                            l("IF NO RELIGHT AFTER 30 S", "", false),
                            l(&format!("ENG MASTER{k}"), "OFF 30 S, THEN ON", false),
                        ]
                    };
                    let t = if e.dead { format!("ENG{k} SHUT DOWN") } else { format!("ENG{k} FAIL") };
                    o.add(Level::Caution, t, lines);
                }
                Alerting::Eicas => {
                    o.add(Level::Caution, format!("ENG{k} FAIL"), vec![l(&format!("THRUST LEVER{k}"), "IDLE", idle), l("IGNITION", "OVRD", sys.sw.ign_start)]);
                }
                Alerting::Vms => {
                    o.add(
                        Level::Warning,
                        "ENGINE",
                        vec![l("THROTTLE", "OFF, THEN MIDRANGE", false), l("JFS (BELOW 400 KT)", "START 2", e.start > 0.0), l("IF NO AIRSTART", "EJECT / FORCED LANDING", false)],
                    );
                }
                Alerting::Annunciators => {
                    if any_running || airborne || sys.power != Power::Off {
                        o.add(Level::Warning, "OIL PRESS", vec![]);
                    }
                }
            }
        }
        if e.overspeed > 0.3 {
            let lines = match style {
                Alerting::Atr => {
                    vec![l(&format!("PL{k}"), "REDUCE", ctl.throttle < 0.6), l("IAS", "REDUCE", ac.airspeed_kt() < 180.0), l(&format!("IF NP > 101 %: CL{k}"), "FTR", !master)]
                }
                _ => vec![l("THROTTLE", "REDUCE", ctl.throttle < 0.6), l("AIRSPEED", "REDUCE", ac.airspeed_kt() < 160.0)],
            };
            if style == Alerting::Annunciators {
                // No warning system: the pilot reads the RPM gauge.
            } else {
                o.add(Level::Caution, format!("PROP{k} OVERSPEED"), lines);
            }
        }
        if e.bird > 0.0 && e.run && style != Alerting::Annunciators {
            o.add(Level::Advisory, format!("ENG{k} VIB HIGH"), vec![]);
        }
        if sys.has(Kind::FuelPump, i) && style != Alerting::Annunciators {
            let t = match style {
                Alerting::Ecam => format!("FUEL{k} PUMP LO PR"),
                Alerting::Eicas => format!("FUEL PUMP{k} FAIL"),
                Alerting::Vms => "FUEL LOW PRESS".to_string(),
                _ => format!("FUEL PUMP{k} LO PR"),
            };
            o.add(Level::Caution, t, vec![l("AVOID HIGH ALTITUDE", "BELOW FL 150", ac.pos.y < 4_500.0)]);
        }
    }

    // ---------------- electrics
    let gens_lost = sd.gens_per_engine > 0 && any_running && !(0..n).any(|i| sys.gen_on[i]) && !(sd.apu && sys.apu > 0.95);
    for i in 0..n {
        if sys.has(Kind::Generator, i) && sys.eng[i].run {
            let k = tag(i);
            match style {
                Alerting::Annunciators => {}
                Alerting::Vms => {
                    o.add(Level::Caution, "MAIN GEN", vec![l("MAIN PWR", "RESET, THEN BATT → MAIN", false)]);
                }
                Alerting::Eicas => {
                    o.add(Level::Caution, format!("IDG{k} OFF BUS"), vec![l("APU", "START", sys.apu > 0.0 || !sd.apu)]);
                }
                _ => {
                    o.add(
                        Level::Caution,
                        format!("ELEC GEN{k} FAULT"),
                        vec![l(&format!("GEN{k}"), "OFF THEN ON", false), l(&format!("IF UNSUCCESSFUL: GEN{k}"), "OFF", !sys.sw.gens[i])],
                    );
                }
            }
        }
    }
    if style == Alerting::Annunciators && sd.gens_per_engine > 0 && sys.power != Power::Off && sys.power < Power::Normal && sys.sw.battery {
        // Low-voltage light (and the POH procedure).
        o.add(
            Level::Warning,
            "VOLTS",
            vec![
                l("AMMETER", "CHECK (DISCHARGE)", true),
                l("MASTER SWITCH (ALT)", "OFF, THEN ON", false),
                l("ELECTRICAL LOAD", "REDUCE (AVIONICS OFF)", !sys.sw.avionics),
                l("LAND", "AS SOON AS PRACTICAL", false),
            ],
        );
    } else if gens_lost && airborne {
        match style {
            Alerting::Ecam => {
                o.add(Level::Warning, "ELEC EMER CONFIG", vec![l("LAND ASAP", "", false), l("APU (IF AVAIL)", "START", sys.apu > 0.0), l("GEN 1+2", "OFF THEN ON", false)]);
            }
            Alerting::Eicas => {
                o.add(Level::Warning, "ELEC EMERGENCY", vec![l("RAT", "DEPLOYED", sys.rat_out), l("APU", "START", sys.apu > 0.0), l("LAND ASAP", "", false)]);
            }
            Alerting::Atr => {
                o.add(Level::Warning, "DC EMER CONFIG", vec![l("BAT", "CHECK", sys.sw.battery), l("LAND ASAP", "", false)]);
            }
            Alerting::Vms => {
                o.add(Level::Warning, "EPU RUN", vec![l("LAND AS SOON AS POSSIBLE", "", false)]);
            }
            Alerting::Annunciators => {}
        }
    }

    // ---------------- hydraulics
    if any_running && !sd.hyd.is_empty() {
        let lost: Vec<usize> = (0..sd.hyd.len().min(3)).filter(|k| sys.hyd[*k] < 0.5).collect();
        if !lost.is_empty() && style != Alerting::Annunciators {
            let names: Vec<&str> = lost.iter().map(|k| sd.hyd[*k].name).collect();
            let level = if lost.len() >= 2 { Level::Warning } else { Level::Caution };
            let title = match style {
                Alerting::Ecam => format!("HYD {} SYS LO PR", names.join("+")),
                Alerting::Eicas => format!("HYD {} LO PRESS", names.join("+")),
                Alerting::Atr => format!("{} HYD LO PR", names.join("+")),
                _ => {
                    if lost.len() >= 2 {
                        "HYD/OIL PRESS".to_string()
                    } else {
                        format!("HYD {} PRESS", names[0])
                    }
                }
            };
            let mut lines = vec![];
            if sd.gear_hyd != 0 && lost.iter().any(|k| sd.gear_hyd & (1 << k) != 0) {
                lines.push(l("FOR LDG: L/G", "GRVTY EXTN", sys.sw.alt_gear));
            }
            if sd.brake_hyd != 0 && lost.iter().any(|k| sd.brake_hyd & (1 << k) != 0) {
                lines.push(l("BRAKING", "ALTN / ACCU ONLY", true));
            }
            if level == Level::Warning {
                lines.push(l("LAND ASAP", "", false));
            }
            o.add(level, title, lines);
        }
    }

    // ---------------- fuel
    let cap = sys.fuel_capacity();
    if cap > 0.0 && n > 0 {
        let total = sys.fuel_total();
        if total < 0.12 * cap && style != Alerting::Annunciators {
            let t = match style {
                Alerting::Vms => "FUEL LOW",
                Alerting::Ecam => "FUEL L+R WING TK LO LVL",
                _ => "FUEL LOW",
            };
            o.add(Level::Caution, t, vec![l("FUEL X FEED", "AS RQRD", true), l("LAND", "ASAP", false)]);
        }
        if sd.tanks.len() >= 2 && sd.tanks[0].2 != 0.0 {
            let d = (sys.tank[0] - sys.tank[1]).abs();
            if d > 0.15 * sys.tank_cap[0].max(1.0) && style != Alerting::Annunciators {
                o.add(Level::Advisory, "FUEL IMBALANCE", vec![l("FUEL X FEED", "OPEN", sys.sw.crossfeed), l("FUEL LEAK", "SUSPECT IF RAPID", false)]);
            }
        }
        if sd.feed == super::defs::Feed::Selector && sys.sw.selector == Selector::Off && any_running {
            o.add(Level::Caution, "FUEL SELECTOR OFF", vec![l("FUEL SELECTOR", "BOTH", false)]);
        }
    }

    // ---------------- instruments / anti-ice
    if style == Alerting::Annunciators && sd.vacuum && !sys.vacuum_ok() && any_running {
        o.add(Level::Caution, "VAC", vec![l("ATTITUDE / HEADING IND", "DISREGARD", true), l("USE", "TURN COORD, ALT, ASI, COMPASS", true)]);
    }
    if sd.glass && sys.fail.any_of(Kind::AttitudeInd) {
        let t = match style {
            Alerting::Ecam => "NAV IR 1 FAULT",
            Alerting::Eicas => "AHRS 1 FAIL",
            Alerting::Vms => "NAV",
            _ => "AHRS 1 FAULT",
        };
        o.add(Level::Caution, t, vec![l("ATT HDG SWTG", "CAPT ON 3", false), l("STANDBY ATTITUDE", "USE", true)]);
    } else if sd.glass && sys.fail.any_of(Kind::HeadingInd) {
        o.add(Level::Caution, if style == Alerting::Eicas { "AHRS 1 HDG FAIL" } else { "NAV HDG 1 FAULT" }, vec![]);
    }
    if sd.glass && sys.pfd_red_x() {
        o.add(Level::Caution, if style == Alerting::Eicas { "DU 1 FAIL" } else { "EIS DMC 1 FAULT" }, vec![l("PFD / ND XFR", "TO ND", false)]);
    }
    if sd.pitot_heat && airborne && style != Alerting::Annunciators && !(sys.sw.pitot_heat && sys.bus_ok()) {
        o.add(Level::Caution, if style == Alerting::Vms { "PROBE HEAT" } else { "ANTI ICE CAPT PITOT" }, vec![l("PROBE / WINDOW HEAT", "ON", sys.sw.pitot_heat)]);
    }
    if sys.ice > 0.05 && style != Alerting::Annunciators && style != Alerting::Vms {
        let prot = sd.wing_anti_ice && sys.sw.wing_anti_ice;
        let t = if style == Alerting::Atr { "ICING" } else { "ANTI ICE ICE DETECTED" };
        o.add(
            if prot { Level::Advisory } else { Level::Caution },
            t,
            vec![
                l(if style == Alerting::Atr { "AIRFRAME DE ICING" } else { "WING ANTI ICE" }, "ON", prot),
                l("ENG ANTI ICE", "ON", sys.sw.eng_anti_ice),
                l("MIN SPEED", "+10 KT", false),
            ],
        );
    }

    // ---------------- flight controls / gear / brakes
    if sys.fail.any_of(Kind::FlapAsym) || sys.fail.any_of(Kind::FlapJam) {
        let t = match style {
            Alerting::Ecam | Alerting::Atr => "F/CTL FLAPS LOCKED",
            Alerting::Eicas => "FLAP FAIL",
            Alerting::Vms => "FLCS FAULT",
            Alerting::Annunciators => "",
        };
        if !t.is_empty() {
            o.add(Level::Caution, t, vec![l("FOR LDG", "USE CURRENT FLAPS", true), l("APPR SPD / LDG DIST", "INCREASE", false)]);
        }
    }
    if sys.jam.iter().any(|j| j.is_some()) && style != Alerting::Annunciators {
        let axis = ["ELEV", "AIL", "RUD"][sys.jam.iter().position(|j| j.is_some()).unwrap_or(0)];
        o.add(
            Level::Warning,
            if style == Alerting::Vms { "FLCS FAULT".to_string() } else { format!("F/CTL {axis} JAM") },
            vec![l("USE TRIM / OTHER AXES", "", false), l("LAND ASAP", "", false)],
        );
    }
    if def.retractable_gear && ctl.gear_down && ac.gear_pos() < 0.99 && sys.fx.gear_down == 0.0 && !ac.on_ground {
        let title = match style {
            Alerting::Annunciators => "GEAR UNSAFE",
            Alerting::Vms => "GEAR",
            _ => "L/G GEAR NOT DOWNLOCKED",
        };
        o.add(Level::Warning, title, vec![l("L/G LEVER", "RECYCLE", false), l("IF UNSUCCESSFUL: L/G", "GRVTY EXTN", sys.sw.alt_gear)]);
    }
    if sys.fail.any_of(Kind::Brakes) && style != Alerting::Annunciators {
        o.add(
            Level::Caution,
            if style == Alerting::Eicas { "BRAKE FAIL" } else { "BRAKES SYS 1+2 FAULT" },
            vec![l("REVERSERS / SPOILERS", "MAX", false), l("LDG DIST", "×2", false)],
        );
    }
    if (sys.fx.tyre[0] || sys.fx.tyre[1]) && matches!(style, Alerting::Ecam | Alerting::Eicas) {
        o.add(Level::Advisory, "WHEEL TYRE LO PR", vec![]);
    }
    if sys.cockpit_smoke() > 0.05 && sys.fail.any_of(Kind::ElecFire) {
        let lines = vec![
            l("LAND ASAP", "", false),
            l("CREW OXY MASKS", "ON / 100 %", false),
            l(if style == Alerting::Annunciators { "MASTER / AVIONICS" } else { "AVIONICS (SMOKE SOURCE)" }, "OFF", !sys.sw.avionics),
        ];
        let t = match style {
            Alerting::Ecam => "AVIONICS SMOKE",
            Alerting::Eicas => "SMOKE AVIONICS",
            Alerting::Atr => "ELEC SMOKE",
            _ => "SMOKE",
        };
        if style != Alerting::Vms {
            o.add(Level::Warning, t, lines).smoke = true;
        }
    }
    // Cabin altitude above 9,550 ft (cabin.rs).
    if sys.cabin.warning() && style != Alerting::Annunciators {
        let fpm = ac.vertical_speed() * 196.85;
        let lines = vec![
            l("CREW OXY MASKS", "USE", sys.cabin.crew_masks),
            l("SIGNS", "ON", sys.cabin.crew_masks),
            l("EMER DESCENT", "INITIATE", fpm < -1500.0 || ac.pos.y < 10_000.0 * 0.3048),
            l("THR LEVERS", "IDLE", ctl.throttle < 0.05 || ac.pos.y < 10_000.0 * 0.3048),
            l("SPD BRK", "FULL", ctl.spoilers > 0.9 || ac.pos.y < 10_000.0 * 0.3048),
            l("MAX FL", "100 / MEA", ac.pos.y < 10_200.0 * 0.3048),
            l("PAX OXY MASKS", "MAN ON", sys.cabin.pax_masks),
        ];
        let t = match style {
            Alerting::Ecam => "CAB PR EXCESS CAB ALT",
            Alerting::Eicas => "CABIN ALTITUDE",
            Alerting::Atr => "EXCESS CAB ALT",
            _ => "CABIN PRESS",
        };
        o.add(Level::Warning, t, lines);
    }
    if sys.windscreen_cracked && style != Alerting::Annunciators {
        o.add(Level::Advisory, "WINDSHIELD CRACKED", vec![l("SPEED", "250 KT MAX", ac.airspeed_kt() < 250.0)]);
    }
    let _ = EngineKind::None;
    let mut v = o.0;
    v.sort_by_key(|a| a.level);
    v
}

/// Short labels for an annunciator strip (lit lights), most important first.
pub fn annunciator_labels(sys: &Systems, def: &AircraftDef, ac: &Aircraft, ctl: &Controls) -> Vec<(String, Level)> {
    alerts(sys, def, ac, ctl).into_iter().filter(|a| a.level != Level::Advisory).map(|a| (a.title, a.level)).collect()
}
