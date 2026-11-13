# Boeing 777-300ER (`triple7`)

**Source:** FGAddon `trunk/Aircraft/777` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/777/>

**Licence:** GPL-2.0 (package `LICENSE` = GNU GPL version 2; no "or later" statement, so treated as GPL v2 — the combined work is distributed under GPL v2, see `/NOTICE`).

**Authors** (package `AUTHORS`): Syd Adams (3D models, YASim, research), Justin Smithies (Nasal, YASim), Karla (model additions), Thorsten Brehm (brakes, GPWS, warnings, cockpit), Joshua Davidson (flight controls, autopilot), Isaak Dieleman (ground services, systems, MFD), Sidi Liang, Yongfan Li, Valkmjolnir (Nasal, effects, liveries), Lars Van Hattum (liveries, UV mapping, model updates).

**Imported:** Exterior `Models/777-300ER.xml` (`777-300ER.ac`, gear with the six-wheel bogies, flaps / slats / spoilers / ailerons / flaperons / elevators / rudder, reversers, lights), the 300 flight deck (`flightdeck-300.xml`: shell, overhead, glareshield / MCP, pedestal with the three CDUs, standby instruments, clocks, gear lever), textures. The flight deck's pedestal and overhead (shown in FlightGear only from the internal view) are loaded unconditionally. Glass screens mapped to Skyward's displays: PFD × 2, ND × 2, upper EICAS (`ewd`), lower MFD (`sd`), CDUs (`mcdu`); the FlightGear 3D-PFD overlay objects are not imported. FDM is YASim (`777-300ER.xml`): reference data only. Ground services, pushback, chocks, EFBs, cabin crew, effects and the landing-light ground spots are not imported.

**Livery:** The package's plain white `Liveries-300ER/white.png` (no airline titles, logos or registration) — used as is.

**Flight model:** Skyward parametric model calibrated to the YASim data (2 × GE90-115B, 513 kN; 436.8 m² wing, 64.8 m span; FCOM Vref table) and published 777-300ER figures (Vref 151 kt flaps 30 at 250 t, Mmo 0.89, ceiling 43,100 ft), flown at 250 t with airliner FBW (C* law, envelope protection). CG at 25 % MAC from the YASim MAC note. See `src/flight/jets.rs` and `src/flight.rs` `h2a_jets_published_numbers`.

**Sound:** the CFM56 recordings (`/assets/audio/CREDITS.md`) with the GE90's 22-blade fan tone.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, effect / ground-service submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels; the PFD screens, which carry no texture coordinates, get a planar mapping). Files here: `model.skac`, `t*.jpg|png`, `import-report.txt`, `fdm.json`. Rebuild: see `tools/fgimport/README.md` (`aircraft/b777.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
