# Lockheed C-130H Hercules (`hercules`)

**Source:** FGAddon `trunk/Aircraft/C130` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/C130/>

**Licence:** GPL-2.0 (package `COPYING` = GNU GPL version 2; no "or later" statement, so treated as GPL v2 — the combined work is distributed under GPL v2, see `/NOTICE`).

**Authors:** Emmanuel Baranger (3D model, FDM), Didier Fabert (textures), Jettoo (FDM); propellers, lights and flight deck from the same package.

**Imported:** Exterior `Models/c130.xml` (`c130.ac`, four 4-blade propellers, gear, lights), flight deck / interior (`Interior/`), textures. FDM is YASim (`Systems/FDM/c130-yasim.xml`): reference data only. The tanker / gunship / refuelling-probe variants are not imported. The rear ramp (`baieB` / `baieH`, a multiplayer property in FlightGear) is driven by Skyward's `doors` channel: it opens when the aircraft stands parked.

**Livery:** The package's default grey scheme with the French Air Force roundels, unit codes (`G4-GE`) and fin serial painted out (`tools/fgimport/liveries/make_liveries.py`, `c130()`). No national insignia or registrations. The propeller sheet gets an alpha channel on its disc / blur area (`propeller-disc.png`).

**Flight model:** Skyward parametric model calibrated to the YASim data (4 × T56-A-15, 4,590 shp, 4.11 m Hamilton Standard props, 162.1 m² wing) and published C-130H figures (stall, take-off distance, climb, 320 KTAS at 20,000 ft) at 60 t; CG estimated from the model geometry (between the tandem main wheels). See `src/flight.rs` `v3_heavies_published_numbers`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, effect / weapon / pilot submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `import-report.txt`. Rebuild: see `tools/fgimport/README.md` (`aircraft/c130.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
