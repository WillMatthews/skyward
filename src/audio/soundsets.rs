//! Per-aircraft recorded engine sound sets: which samples play, on which
//! bus, and how their volume and pitch follow the engine — translated from
//! the FlightGear `*-sound.xml` / sound-filter configs of the aircraft the
//! recordings come from (tables are the FG ones unless noted), plus
//! equal-power cross-fade groups for RPM-indexed loop banks.
//!
//! Drivers: `N1` / `N2` in percent (N2 is derived from N1 — the sim has no
//! core spool), `Rpm` crank / prop RPM, `Pow` power or spool fraction,
//! `Ab` afterburner 0..1, `Rev` reverser 0..1.

use super::params::SoundSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drv {
    N1,
    N2,
    Rpm,
    Pow,
    Ab,
    Rev,
}

/// Exterior directivity bus (as in `engines::EngOut`) or the recorded
/// interior (cockpit) bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bus {
    Front,
    Rear,
    Case,
    Int,
}

/// Listener-distance band of an exterior layer (FG ships close / normal /
/// distant recordings of the same state).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Band {
    All,
    Near,
    Far,
}

#[derive(Clone, Copy, Debug)]
pub enum Pitch {
    /// Piecewise-linear table of the driver (clamped at the ends).
    Tab(Drv, &'static [(f32, f32)]),
    /// `clamp((x / ref)^exp, lo, hi)` — a loop recorded at `ref`.
    Ratio(Drv, f32, f32, f32, f32),
    /// `clamp(a + b·x, lo, hi)` (FG `offset` / `factor`).
    Lin(Drv, f32, f32, f32, f32),
}

#[derive(Clone, Copy, Debug)]
pub struct Layer {
    pub s: &'static str,
    pub bus: Bus,
    pub band: Band,
    pub vol: (Drv, &'static [(f32, f32)]),
    /// Optional second volume factor (e.g. reverser position).
    pub vmul: Option<(Drv, &'static [(f32, f32)])>,
    pub pitch: Pitch,
    pub gain: f32,
    /// Member of an equal-power cross-fade group: (group, index in its centres).
    pub xf: Option<(usize, usize)>,
    /// Combustion layer: silenced quickly when the engine is not running
    /// (piston firing sounds), instead of following the spool-down.
    pub fire: bool,
}

/// A one-shot played on a start / shutdown.
#[derive(Clone, Copy, Debug)]
pub struct OneShot {
    pub s: &'static str,
    pub bus: Bus,
    pub gain: f32,
    pub delay: f32,
    /// For a looped sample: play it this long (s), then fade.
    pub hold: f32,
}

/// Cross-fade group: driver and the centre value of each member.
#[derive(Clone, Copy, Debug)]
pub struct XFade {
    pub drv: Drv,
    pub centres: &'static [f32],
}

/// Gains for the procedural engine voice when it plays under the samples
/// as detail (front / rear / case components).
#[derive(Clone, Copy, Debug)]
pub struct Detail {
    pub front: f32,
    pub rear: f32,
    pub case: f32,
}

pub struct Set {
    pub layers: &'static [Layer],
    pub groups: &'static [XFade],
    pub start: &'static [OneShot],
    pub stop: &'static [OneShot],
    pub detail: Detail,
    /// How much of the exterior mix still feeds the (filtered) interior
    /// path: 1 without interior recordings, small with them.
    pub int_from_ext: f32,
    /// Overall calibration of this set against the rest of the mix.
    pub gain: f32,
    /// Loudness against power (dB table), exterior and interior. The FG
    /// layer tables cross-fade recordings that were each normalised when
    /// recorded, so on their own an idling engine is nearly as loud as one
    /// at take-off power; real ones differ by 15-25 dB outside (jet noise
    /// grows ~ V_jet⁸) and ~10 dB in the cockpit.
    pub level: (Drv, &'static [(f32, f32)]),
    pub level_int: (Drv, &'static [(f32, f32)]),
    /// Cockpit-recording calibration (relative to `gain`).
    pub gain_int: f32,
    /// Every sample the set uses (for decoding).
    pub samples: &'static [&'static str],
}

const fn l(s: &'static str, bus: Bus, vol: (Drv, &'static [(f32, f32)]), pitch: Pitch) -> Layer {
    Layer {
        s,
        bus,
        band: Band::All,
        vol,
        vmul: None,
        pitch,
        gain: 1.0,
        xf: None,
        fire: false,
    }
}

impl Layer {
    const fn g(mut self, g: f32) -> Layer {
        self.gain = g;
        self
    }
    const fn xf(mut self, g: usize, i: usize) -> Layer {
        self.xf = Some((g, i));
        self
    }
    const fn near(mut self) -> Layer {
        self.band = Band::Near;
        self
    }
    const fn far(mut self) -> Layer {
        self.band = Band::Far;
        self
    }
    const fn mul(mut self, d: Drv, t: &'static [(f32, f32)]) -> Layer {
        self.vmul = Some((d, t));
        self
    }
    const fn fire(mut self) -> Layer {
        self.fire = true;
        self
    }
}

const fn shot(s: &'static str, bus: Bus, gain: f32, delay: f32) -> OneShot {
    OneShot {
        s,
        bus,
        gain,
        delay,
        hold: 0.0,
    }
}

const ONE: &[(f32, f32)] = &[(0.0, 1.0)];

use Bus::*;
use Drv::*;
use Pitch::*;

// ------------------------------------------------------------ Cessna 172
// Two recordings: idle (firing fundamental 29.7 Hz → ~890 rpm) and run
// (81.1 Hz → 2433 rpm; FG pitches it as 2300). Equal-power cross-fade
// between them over 950..2100 rpm (FG: idle fader 1 → 0 over 1200..2500).
const C172_LAYERS: &[Layer] = &[
    l(
        "c172_idle",
        Case,
        (Rpm, &[(0.0, 0.0), (250.0, 0.0), (520.0, 1.0)]),
        Ratio(Rpm, 890.0, 1.0, 0.45, 1.6),
    )
    .xf(0, 0)
    .fire(),
    l(
        "c172_run",
        Case,
        (Pow, &[(0.0, 0.85), (1.0, 1.12)]),
        Ratio(Rpm, 2433.0, 1.0, 0.35, 1.25),
    )
    .xf(0, 1)
    .fire(),
];
pub static C172: Set = Set {
    layers: C172_LAYERS,
    groups: &[XFade {
        drv: Rpm,
        centres: &[950.0, 2100.0],
    }],
    start: &[
        OneShot {
            s: "c172_crank",
            bus: Case,
            gain: 0.9,
            delay: 0.0,
            hold: 1.1,
        },
        shot("c172_start", Case, 1.0, 1.0),
    ],
    stop: &[shot("c172_cut", Case, 0.9, 0.0)],
    // The 2-blade prop tone and the exhaust pulses are in the recording, at
    // the same frequency (blade pass = firing rate on a direct-drive
    // flat-four): the modelled ones would only beat against them.
    detail: Detail {
        front: 0.0,
        rear: 0.0,
        case: 0.35,
    },
    int_from_ext: 1.0,
    gain: 1.6,
    level: (Pow, &[(0.0, 0.0)]),
    level_int: (Pow, &[(0.0, 0.0)]),
    gain_int: 1.0,
    samples: &[
        "c172_idle",
        "c172_run",
        "c172_crank",
        "c172_start",
        "c172_cut",
    ],
};

// ------------------------------------------------------------ Spitfire IX
// Merlin loops at 1070 / 1508 / 2155 rpm (measured firing frequency rpm/10;
// FG refs 1053 / 1493 / 2165, hard-switched there — cross-faded here).
pub static MERLIN: Set = Set {
    layers: &[
        l(
            "merlin_1",
            Case,
            (Rpm, &[(0.0, 0.0), (300.0, 0.0), (550.0, 1.0)]),
            Ratio(Rpm, 1070.0, 1.0, 0.45, 1.5),
        )
        .xf(0, 0)
        .fire(),
        l(
            "merlin_2",
            Case,
            (Rpm, ONE),
            Ratio(Rpm, 1508.0, 1.0, 0.5, 1.6),
        )
        .xf(0, 1)
        .fire(),
        l(
            "merlin_3",
            Case,
            (Pow, &[(0.0, 0.85), (1.0, 1.1)]),
            Ratio(Rpm, 2155.0, 1.0, 0.5, 1.5),
        )
        .xf(0, 2)
        .fire(),
    ],
    groups: &[XFade {
        drv: Rpm,
        centres: &[1070.0, 1508.0, 2155.0],
    }],
    start: &[shot("merlin_start", Case, 1.0, 0.0)],
    stop: &[shot("merlin_stop", Case, 1.0, 0.0)],
    detail: Detail {
        front: 0.45,
        rear: 0.3,
        case: 0.5,
    },
    int_from_ext: 1.0,
    gain: 1.0,
    level: (Pow, &[(0.0, -11.0), (0.3, -6.0), (0.7, -2.0), (1.0, 0.0)]),
    level_int: (Pow, &[(0.0, -8.0), (0.5, -3.0), (1.0, 0.0)]),
    gain_int: 1.0,
    samples: &[
        "merlin_1",
        "merlin_2",
        "merlin_3",
        "merlin_start",
        "merlin_stop",
    ],
};

// ------------------------------------------------------------ F-16
// Exterior: GEED ground-idle / MIL recordings, front (inlet) and aft
// (exhaust), cross-faded on N1; afterburner blast + crackle; FGUK whine
// (FG: pitch 0.8 + 0.00666·N1); the crackling "roar" FG plays for still
// cameras, here for mid / far listeners. Interior: Timi's cockpit set at
// three N2 points, low (rumble, pitch on N1) and high (whine, pitch on N2)
// halves, with the FG pitch laws.
pub static F16: Set = Set {
    layers: &[
        l(
            "f16_x_idle_f",
            Front,
            (N1, &[(0.0, 0.0), (8.0, 0.0), (25.0, 0.85), (35.0, 1.0)]),
            Ratio(N1, 35.0, 1.0, 0.4, 1.7),
        )
        .xf(0, 0),
        l(
            "f16_x_mil_f",
            Front,
            (N1, ONE),
            Ratio(N1, 100.0, 1.0, 0.6, 1.08),
        )
        .xf(0, 1),
        l(
            "f16_x_idle_a",
            Rear,
            (N1, &[(0.0, 0.0), (8.0, 0.0), (25.0, 0.85), (35.0, 1.0)]),
            Ratio(N1, 35.0, 0.8, 0.5, 1.6),
        )
        .xf(1, 0),
        l(
            "f16_x_mil_a",
            Rear,
            (N1, &[(35.0, 0.55), (100.0, 1.0)]),
            Ratio(N1, 100.0, 0.6, 0.7, 1.05),
        )
        .xf(1, 1)
        .g(1.3),
        l(
            "f16_x_ab",
            Rear,
            (Ab, &[(0.0, 0.0), (0.04, 0.45), (1.0, 1.0)]),
            Lin(Ab, 0.94, 0.08, 0.9, 1.05),
        )
        .g(1.0),
        l(
            "f16_x_abx",
            Rear,
            (Ab, &[(0.0, 0.0), (0.1, 0.35), (1.0, 0.8)]),
            Lin(Ab, 1.0, 0.0, 1.0, 1.0),
        )
        .g(0.9),
        l(
            "f16_x_whine",
            Front,
            (N1, &[(20.0, 0.0), (40.0, 0.25), (80.0, 0.35), (100.0, 0.4)]),
            Lin(N1, 0.8, 0.00666, 0.5, 1.6),
        )
        .g(0.45),
        l(
            "f16_x_roar",
            Rear,
            (N1, &[(55.0, 0.0), (85.0, 0.5), (100.0, 0.8)]),
            Lin(N1, 0.8, 0.00666, 0.5, 1.6),
        )
        .far()
        .g(0.7),
        l(
            "f16_i_idle_lo",
            Int,
            (N2, &[(0.0, 0.0), (20.0, 0.0), (45.0, 1.0)]),
            Lin(N1, 0.8635, 0.00455, 0.5, 1.4),
        )
        .xf(2, 0),
        l(
            "f16_i_idle_hi",
            Int,
            (N2, &[(0.0, 0.0), (20.0, 0.0), (45.0, 1.0)]),
            Lin(N2, -0.8468, 0.03078, 0.3, 1.6),
        )
        .xf(2, 0),
        l(
            "f16_i_88_lo",
            Int,
            (N2, ONE),
            Lin(N1, 0.9193, 0.00102, 0.5, 1.3),
        )
        .xf(2, 1),
        l(
            "f16_i_88_hi",
            Int,
            (N2, ONE),
            Lin(N2, -2.1529, 0.03583, 0.3, 1.5),
        )
        .xf(2, 1),
        l(
            "f16_i_100_lo",
            Int,
            (N2, ONE),
            Lin(N1, 0.9, 0.0009999, 0.5, 1.3),
        )
        .xf(2, 2),
        l(
            "f16_i_100_hi",
            Int,
            (N2, ONE),
            Lin(N2, -1.5056, 0.025056, 0.3, 1.5),
        )
        .xf(2, 2),
        l(
            "f16_i_ab",
            Int,
            (Ab, &[(0.0, 0.0), (0.05, 0.5), (1.0, 1.0)]),
            Lin(Ab, 1.0, 0.0, 1.0, 1.0),
        ),
    ],
    groups: &[
        XFade {
            drv: N1,
            centres: &[40.0, 92.0],
        },
        XFade {
            drv: N1,
            centres: &[40.0, 92.0],
        },
        XFade {
            drv: N2,
            centres: &[60.0, 88.0, 100.0],
        },
    ],
    start: &[],
    stop: &[],
    detail: Detail {
        front: 0.3,
        rear: 0.3,
        case: 0.35,
    },
    int_from_ext: 0.22,
    gain: 1.0,
    level: (
        N1,
        &[
            (20.0, -16.0),
            (35.0, -13.0),
            (60.0, -8.0),
            (85.0, -2.5),
            (100.0, 0.0),
        ],
    ),
    level_int: (N1, &[(35.0, -6.0), (70.0, -3.0), (100.0, 0.0)]),
    gain_int: 1.8,
    samples: &[
        "f16_x_idle_f",
        "f16_x_mil_f",
        "f16_x_idle_a",
        "f16_x_mil_a",
        "f16_x_ab",
        "f16_x_abx",
        "f16_x_whine",
        "f16_x_roar",
        "f16_i_idle_lo",
        "f16_i_idle_hi",
        "f16_i_88_lo",
        "f16_i_88_hi",
        "f16_i_100_lo",
        "f16_i_100_hi",
        "f16_i_ab",
    ],
};

// ------------------------------------------------------------ ATR 72
// DHC-8 (PW120-family) exterior / interior turbine recordings driven by the
// gas-generator speed Ng (FG: volume ∝ N2, pitch 0.015·N2 clamped 0.2..1.2),
// the fgdata turboprop loop as a quiet propeller layer, reverse roar.
const NG: &[(f32, f32)] = &[(0.0, 0.0), (100.0, 1.0)];
pub static ATR: Set = Set {
    layers: &[
        l("atr_out", Case, (N2, NG), Lin(N2, 0.0, 0.015, 0.2, 1.25)),
        l("atr_prop", Front, (N2, NG), Lin(N2, 0.0, 0.015, 0.2, 1.25)).g(0.15),
        l("atr_rev", Case, (N2, NG), Lin(N2, 0.0, 0.015, 0.2, 1.25))
            .mul(Rev, &[(0.0, 0.0), (1.0, 1.0)])
            .g(0.55),
        l("atr_int", Int, (N2, NG), Lin(N2, 0.0, 0.015, 0.2, 1.25)),
    ],
    groups: &[],
    start: &[
        shot("atr_start", Case, 0.7, 0.0),
        shot("atr_start", Int, 0.35, 0.0),
    ],
    stop: &[
        shot("atr_stop", Case, 0.6, 0.0),
        shot("atr_stop", Int, 0.3, 0.0),
    ],
    detail: Detail {
        front: 0.8,
        rear: 0.3,
        case: 0.25,
    },
    int_from_ext: 0.35,
    gain: 1.0,
    level: (
        N2,
        &[(55.0, -11.0), (62.0, -9.0), (80.0, -4.0), (100.0, 0.0)],
    ),
    level_int: (N2, &[(62.0, -6.0), (100.0, 0.0)]),
    gain_int: 0.63,
    samples: &[
        "atr_out",
        "atr_prop",
        "atr_rev",
        "atr_int",
        "atr_start",
        "atr_stop",
    ],
};

// ------------------------------------------------------------ Vulcan B.2 Olympus 301
// FGUK AvroVulcanB2 Sounds/vulcanb2-sound.xml layers (turbine whine, exhaust,
// distorted jet blast, the intake howl). The FG volume factors are tied to its
// JSBSim N2 (which runs past 100 %); the tables are re-fitted to Skyward's
// N2 so the howl comes in at climb power (~85 %) as on the real aircraft.
const VUL_TURB_V: &[(f32, f32)] = &[(5.0, 0.0), (30.0, 0.5), (55.0, 0.9), (100.0, 1.0)];
const VUL_JET_V: &[(f32, f32)] = &[(45.0, 0.0), (70.0, 0.3), (100.0, 0.75)];
const VUL_BLAST_V: &[(f32, f32)] = &[(60.0, 0.0), (80.0, 0.3), (100.0, 0.65)];
const VUL_HOWL_V: &[(f32, f32)] = &[(78.0, 0.0), (88.0, 0.35), (100.0, 0.6)];
const VUL_INT_V: &[(f32, f32)] = &[(10.0, 0.0), (40.0, 0.45), (100.0, 1.0)];

pub static OLYMPUS: Set = Set {
    layers: &[
        l("vul_x_turb", Front, (N2, VUL_TURB_V), Lin(N2, 0.75, 0.0075, 0.75, 1.5)).g(0.55),
        l("vul_x_howl", Front, (N2, VUL_HOWL_V), Lin(N2, 0.7, 0.003, 0.9, 1.0)),
        l("vul_x_jet", Rear, (N2, VUL_JET_V), Lin(N2, 0.7, 0.003, 0.85, 1.0)),
        l("vul_x_blast", Rear, (N2, VUL_BLAST_V), Lin(N2, 0.75, 0.0025, 0.9, 1.0)),
        l("vul_i_eng", Int, (N2, VUL_INT_V), Lin(N2, 0.7, 0.003, 0.8, 1.0)),
    ],
    groups: &[],
    start: &[
        shot("vul_start", Case, 0.7, 0.0),
        shot("vul_start", Int, 0.3, 0.0),
    ],
    stop: &[
        shot("vul_stop", Case, 0.6, 0.0),
        shot("vul_stop", Int, 0.3, 0.0),
    ],
    detail: Detail {
        front: 0.4,
        rear: 0.5,
        case: 0.3,
    },
    int_from_ext: 0.3,
    gain: 0.8,
    // Turbojets: idle → full ~22 dB outside (the exhaust dominates).
    level: (N2, &[(30.0, -22.0), (60.0, -13.0), (85.0, -5.0), (100.0, 0.0)]),
    level_int: (N2, &[(30.0, -10.0), (100.0, 0.0)]),
    gain_int: 0.6,
    samples: &["vul_x_turb", "vul_x_howl", "vul_x_jet", "vul_x_blast", "vul_i_eng", "vul_start", "vul_stop"],
};

// ------------------------------------------------------------ A320 CFM56-5B
// FG A320-family Systems/cfm56-sound.xml tables, verbatim.
const CFM_IDLE_V: &[(f32, f32)] = &[
    (12.0, 0.0),
    (14.0, 0.01),
    (18.9, 0.9),
    (41.7, 0.31),
    (70.0, 0.018),
    (99.8, 0.0),
];
const CFM_IDLE_R: &[(f32, f32)] = &[
    (12.0, 0.0),
    (14.0, 0.01),
    (18.9, 1.0),
    (41.7, 0.41),
    (55.0, 0.24),
    (57.2, 0.1),
    (70.0, 0.018),
    (99.8, 0.0),
];
const CFM_IDLE_C: &[(f32, f32)] = &[
    (12.0, 0.0),
    (14.0, 0.01),
    (18.9, 1.0),
    (41.7, 0.41),
    (70.0, 0.018),
    (99.8, 0.0),
];
const CFM_IDLE_P: Pitch = Tab(N1, &[(18.9, 1.0), (42.5, 1.5)]);
const CFM_SPOOL_V: &[(f32, f32)] = &[
    (31.0, 0.0),
    (60.0, 0.6),
    (76.0, 0.63),
    (91.0, 0.55),
    (95.0, 0.2),
];
const CFM_SPOOL_P: Pitch = Tab(N1, &[(31.0, 0.63), (95.0, 2.0)]);
const CFM_RUMBLE_V: &[(f32, f32)] = &[(60.0, 0.0), (90.0, 0.25), (100.0, 0.83)];
const CFM_RUMBLE_P: Pitch = Tab(N2, &[(60.0, 0.5), (100.0, 1.0)]);
const CFM_RUMBLE_D: &[(f32, f32)] = &[
    (62.0, 0.0),
    (74.0, 0.05),
    (88.0, 0.55),
    (90.0, 0.8),
    (94.0, 0.83),
    (99.0, 0.85),
];
const CFM_BUZZ_V: &[(f32, f32)] = &[
    (75.0, 0.0),
    (82.0, 0.6),
    (87.0, 0.8),
    (90.0, 0.83),
    (95.0, 0.85),
];
const CFM_BUZZ_P: Pitch = Tab(N1, &[(31.0, 0.315), (95.0, 1.0)]);
const CFM_REV_V: &[(f32, f32)] = &[(30.0, 0.0), (70.0, 0.8), (100.0, 0.995)];
const CFM_REV_P: Pitch = Tab(N2, &[(60.0, 0.3), (90.0, 1.0)]);
const REV: &[(f32, f32)] = &[(0.0, 0.0), (1.0, 1.0)];
pub static CFM56: Set = Set {
    layers: &[
        l("cfm_x_idle", Front, (N1, CFM_IDLE_V), CFM_IDLE_P),
        l("cfm_x_idle_r", Rear, (N1, CFM_IDLE_R), CFM_IDLE_P),
        l("cfm_x_idle_c", Front, (N1, CFM_IDLE_C), CFM_IDLE_P).near(),
        l("cfm_x_idle_d", Case, (N1, CFM_IDLE_V), CFM_IDLE_P).far(),
        l("cfm_x_spool", Front, (N1, CFM_SPOOL_V), CFM_SPOOL_P),
        l("cfm_x_lowspool", Front, (N1, CFM_SPOOL_V), CFM_SPOOL_P),
        l("cfm_x_spool_c", Front, (N1, CFM_SPOOL_V), CFM_SPOOL_P).near(),
        l("cfm_x_spool_cr", Rear, (N1, CFM_SPOOL_V), CFM_SPOOL_P).near(),
        l("cfm_x_rumble", Rear, (N2, CFM_RUMBLE_V), CFM_RUMBLE_P),
        l("cfm_x_rumble_d", Rear, (N1, CFM_RUMBLE_D), CFM_RUMBLE_P).far(),
        l("cfm_x_buzz", Front, (N1, CFM_BUZZ_V), CFM_BUZZ_P),
        l("cfm_x_buzz_d", Front, (N1, CFM_BUZZ_V), CFM_BUZZ_P).far(),
        l("cfm_x_rev", Case, (N2, CFM_REV_V), CFM_REV_P).mul(Rev, REV),
        // Cockpit.
        l(
            "cfm_c_idle",
            Int,
            (
                N1,
                &[
                    (12.0, 0.0),
                    (18.9, 0.9),
                    (41.7, 0.673),
                    (55.0, 0.297),
                    (57.2, 0.042),
                    (70.0, 0.0),
                ],
            ),
            Tab(N1, &[(18.9, 1.1), (42.5, 1.65)]),
        ),
        l(
            "cfm_c_idle2",
            Int,
            (
                N2,
                &[
                    (12.0, 0.0),
                    (18.9, 0.0),
                    (41.7, 0.62),
                    (55.0, 0.964),
                    (57.2, 1.0),
                    (83.0, 0.18),
                ],
            ),
            Tab(N2, &[(58.0, 1.0), (82.5, 1.5)]),
        ),
        // FG plays this file twice ("spool" + "lowspool") with one pitch law: summed tables.
        l(
            "cfm_c_spool",
            Int,
            (
                N1,
                &[
                    (31.0, 0.0),
                    (50.0, 0.534),
                    (69.0, 1.22),
                    (76.4, 1.091),
                    (88.0, 0.215),
                ],
            ),
            Tab(N1, &[(31.0, 0.63), (95.0, 2.0)]),
        ),
        l(
            "cfm_c_hum",
            Int,
            (
                N2,
                &[
                    (20.0, 0.0),
                    (41.7, 0.005),
                    (55.0, 0.02),
                    (57.2, 0.09),
                    (65.0, 0.8),
                    (75.0, 0.0),
                ],
            ),
            Tab(N2, &[(58.0, 1.0), (82.0, 1.5)]),
        ),
        l(
            "cfm_c_comb",
            Int,
            (
                N2,
                &[
                    (59.0, 0.0),
                    (62.0, 0.4),
                    (74.0, 0.128),
                    (78.0, 0.056),
                    (84.0, 0.056),
                    (94.4, 0.04),
                ],
            ),
            Tab(N2, &[(60.0, 1.0), (100.0, 2.0)]),
        ),
        l(
            "cfm_c_comb2",
            Int,
            (
                N2,
                &[
                    (62.0, 0.0),
                    (74.0, 0.013),
                    (78.0, 0.4025),
                    (84.0, 0.98),
                    (94.4, 1.0),
                ],
            ),
            Tab(N2, &[(60.0, 0.5), (100.0, 1.0)]),
        ),
        l(
            "cfm_c_buzz",
            Int,
            (
                N1,
                &[
                    (75.0, 0.0),
                    (80.0, 0.894),
                    (87.0, 0.995),
                    (89.0, 1.0),
                    (95.0, 0.936),
                ],
            ),
            Tab(N1, &[(31.0, 0.315), (95.0, 1.0)]),
        ),
        l("cfm_c_rev", Int, (N2, CFM_REV_V), CFM_REV_P).mul(Rev, REV),
    ],
    groups: &[],
    start: &[
        shot("cfm_x_start", Case, 0.8, 0.0),
        shot("cfm_c_start", Int, 0.8, 0.0),
    ],
    stop: &[
        shot("cfm_x_stop", Case, 0.8, 0.0),
        shot("cfm_c_stop", Int, 0.8, 0.0),
    ],
    detail: Detail {
        front: 0.15,
        rear: 0.12,
        case: 0.15,
    },
    int_from_ext: 0.15,
    gain: 1.0,
    level: (
        N1,
        &[
            (15.0, -17.0),
            (21.0, -15.0),
            (40.0, -10.0),
            (60.0, -6.0),
            (80.0, -2.5),
            (100.0, 0.0),
        ],
    ),
    level_int: (
        N1,
        &[(21.0, -8.0), (50.0, -5.0), (80.0, -1.5), (100.0, 0.0)],
    ),
    gain_int: 0.4,
    samples: &[
        "cfm_x_idle",
        "cfm_x_idle_r",
        "cfm_x_idle_c",
        "cfm_x_idle_d",
        "cfm_x_spool",
        "cfm_x_lowspool",
        "cfm_x_spool_c",
        "cfm_x_spool_cr",
        "cfm_x_rumble",
        "cfm_x_rumble_d",
        "cfm_x_buzz",
        "cfm_x_buzz_d",
        "cfm_x_rev",
        "cfm_c_idle",
        "cfm_c_idle2",
        "cfm_c_spool",
        "cfm_c_hum",
        "cfm_c_comb",
        "cfm_c_comb2",
        "cfm_c_buzz",
        "cfm_c_rev",
        "cfm_x_start",
        "cfm_c_start",
        "cfm_x_stop",
        "cfm_c_stop",
    ],
};

// ------------------------------------------------------------ IAE V2500 (E190)
// FG A320-family Systems/v2500-sound.xml tables. (No CF34 recordings with
// a clear licence exist; the V2500 is the closest real twin-spool high-
// bypass set and gives the E190 a different voice from the A320.)
const V25_IDLE_V: &[(f32, f32)] = &[
    (19.0, 0.0),
    (41.7, 0.31),
    (53.0, 0.47),
    (56.0, 0.9),
    (81.8, 0.05),
    (97.0, 0.01),
];
const V25_IDLE_P: Pitch = Tab(N2, &[(56.0, 1.0), (99.34, 1.851)]);
const V25_SPOOL_V: &[(f32, f32)] = &[
    (12.8, 0.0),
    (18.2, 0.5),
    (51.4, 0.55),
    (69.5, 0.6),
    (78.4, 0.33),
];
const V25_SPOOL_P: Pitch = Tab(N1, &[(21.0, 0.319), (110.0, 1.541)]);
const V25_BUZZ_V: &[(f32, f32)] = &[
    (70.2, 0.0),
    (74.6, 0.2),
    (89.8, 0.95),
    (95.0, 0.9),
    (110.0, 0.91),
];
const V25_BUZZ_P: Pitch = Tab(N1, &[(20.0, 0.245), (110.0, 1.094)]);
const V25_RUMBLE_V: &[(f32, f32)] = &[(60.0, 0.0), (90.0, 0.3), (110.0, 0.9)];
pub static V2500: Set = Set {
    layers: &[
        l("v25_x_idle", Front, (N2, V25_IDLE_V), V25_IDLE_P),
        l("v25_x_idle_r", Rear, (N2, V25_IDLE_V), V25_IDLE_P),
        l("v25_x_spool", Front, (N1, V25_SPOOL_V), V25_SPOOL_P),
        l("v25_x_lowspool", Front, (N1, V25_SPOOL_V), V25_SPOOL_P),
        l("v25_x_buzz", Front, (N1, V25_BUZZ_V), V25_BUZZ_P),
        l("v25_x_buzz_d", Front, (N1, V25_BUZZ_V), V25_BUZZ_P).far(),
        l(
            "cfm_x_rumble",
            Rear,
            (N2, V25_RUMBLE_V),
            Tab(N2, &[(60.0, 0.5), (100.0, 1.0)]),
        ),
        l(
            "cfm_x_rumble_d",
            Rear,
            (N2, V25_RUMBLE_V),
            Tab(N2, &[(60.0, 0.5), (100.0, 1.0)]),
        )
        .far(),
        l(
            "cfm_x_rev",
            Case,
            (N2, &[(30.0, 0.0), (60.0, 0.8), (90.0, 0.9)]),
            Tab(N2, &[(55.0, 0.5), (90.0, 1.0)]),
        )
        .mul(Rev, REV),
        l(
            "v25_c_idle",
            Int,
            (
                N2,
                &[
                    (19.0, 0.0),
                    (41.7, 0.086),
                    (53.642, 0.128),
                    (56.291, 0.495),
                    (81.898, 0.426),
                    (91.17, 0.309),
                    (99.779, 0.17),
                ],
            ),
            V25_IDLE_P,
        ),
        l(
            "v25_c_lowspool",
            Int,
            (
                N1,
                &[
                    (10.1, 0.0),
                    (21.4, 0.29),
                    (51.4, 0.531),
                    (69.5, 0.545),
                    (78.4, 0.414),
                ],
            ),
            V25_SPOOL_P,
        ),
        l(
            "v25_c_comb",
            Int,
            (
                N2,
                &[
                    (59.0, 0.0),
                    (62.0, 0.05),
                    (74.0, 0.35),
                    (78.0, 0.55),
                    (84.0, 0.98),
                    (94.4, 0.95),
                ],
            ),
            Tab(N2, &[(60.0, 0.5), (110.0, 1.125)]),
        ),
        l(
            "v25_c_buzz",
            Int,
            (
                N1,
                &[
                    (70.0, 0.0),
                    (74.0, 0.894),
                    (87.0, 0.995),
                    (89.0, 1.0),
                    (95.0, 0.936),
                    (110.0, 0.94),
                ],
            ),
            Tab(N1, &[(20.0, 0.245), (110.0, 1.151)]),
        ),
        l(
            "v25_c_rev",
            Int,
            (N2, &[(30.0, 0.0), (60.0, 0.6), (100.0, 0.895)]),
            Tab(N2, &[(60.0, 0.3), (90.0, 1.0)]),
        )
        .mul(Rev, REV),
    ],
    groups: &[],
    start: &[
        shot("v25_x_start", Case, 0.8, 0.0),
        shot("v25_c_start", Int, 0.8, 0.0),
    ],
    stop: &[
        shot("cfm_x_stop", Case, 0.8, 0.0),
        shot("cfm_c_stop", Int, 0.8, 0.0),
    ],
    detail: Detail {
        front: 0.15,
        rear: 0.12,
        case: 0.15,
    },
    int_from_ext: 0.15,
    gain: 1.0,
    level: (
        N1,
        &[
            (15.0, -17.0),
            (22.0, -15.0),
            (40.0, -10.0),
            (60.0, -6.0),
            (80.0, -2.5),
            (100.0, 0.0),
        ],
    ),
    level_int: (
        N1,
        &[(22.0, -8.0), (50.0, -5.0), (80.0, -1.5), (100.0, 0.0)],
    ),
    gain_int: 0.5,
    samples: &[
        "v25_x_idle",
        "v25_x_idle_r",
        "v25_x_spool",
        "v25_x_lowspool",
        "v25_x_buzz",
        "v25_x_buzz_d",
        "cfm_x_rumble",
        "cfm_x_rumble_d",
        "cfm_x_rev",
        "v25_c_idle",
        "v25_c_lowspool",
        "v25_c_comb",
        "v25_c_buzz",
        "v25_c_rev",
        "v25_x_start",
        "v25_c_start",
        "cfm_x_stop",
        "cfm_c_stop",
    ],
};

/// Airframe / environment / impact samples every aircraft can use.
pub const COMMON: &[&str] = &[
    "tyre_roll",
    "tyre_gravel",
    "flap_motor_light",
    "c172_stall",
    "heavy_roll",
    "heavy_roll_int",
    "heavy_touch",
    "fighter_touch",
    "gear_heavy",
    "gear_fighter",
    "flap_motor_heavy",
    "wind_airliner",
    "wind_light",
    "glider_wind",
    "glider_brake",
    "glider_roll",
    "thunder_1",
    "thunder_2",
    "thunder_3",
    "crash_c172",
    "crash_heavy",
    "crash_light",
    "crash_scrape",
    "crash_f16",
    "crash_explode",
    "crash_water",
];

pub fn set(s: SoundSet) -> Option<&'static Set> {
    Some(match s {
        SoundSet::None => return None,
        SoundSet::C172 => &C172,
        SoundSet::Merlin => &MERLIN,
        SoundSet::F16 => &F16,
        SoundSet::Atr => &ATR,
        SoundSet::Cfm56 => &CFM56,
        SoundSet::V2500 => &V2500,
        SoundSet::Olympus => &OLYMPUS,
    })
}

/// Piecewise-linear table lookup, clamped at the ends.
#[inline]
pub fn table(t: &[(f32, f32)], x: f32) -> f32 {
    match t.len() {
        0 => 1.0,
        1 => t[0].1,
        _ => {
            if x <= t[0].0 {
                return t[0].1;
            }
            for w in t.windows(2) {
                if x <= w[1].0 {
                    let u = (x - w[0].0) / (w[1].0 - w[0].0).max(1e-6);
                    return w[0].1 + (w[1].1 - w[0].1) * u;
                }
            }
            t[t.len() - 1].1
        }
    }
}

/// Equal-power weight of member `i` of a cross-fade over `centres` at `x`.
pub fn xfade_weight(centres: &[f32], i: usize, x: f32) -> f32 {
    let n = centres.len();
    if n <= 1 {
        return 1.0;
    }
    if x <= centres[0] {
        return if i == 0 { 1.0 } else { 0.0 };
    }
    if x >= centres[n - 1] {
        return if i == n - 1 { 1.0 } else { 0.0 };
    }
    let k = centres.windows(2).position(|w| x < w[1]).unwrap_or(n - 2);
    let u = (x - centres[k]) / (centres[k + 1] - centres[k]);
    let a = u * std::f32::consts::FRAC_PI_2;
    if i == k {
        a.cos()
    } else if i == k + 1 {
        a.sin()
    } else {
        0.0
    }
}
