# Lockheed C-5M Super Galaxy (`galaxy`)

**Source:** github.com/FGMEMBERS/C-5 (`master` 33ec1e31, 2022-07-07) — <https://github.com/FGMEMBERS/C-5>

**Licence:** GPL-2.0 (repository `COPYING` = GNU GPL version 2; no "or later" statement, so treated as GPL v2 — see `/NOTICE`).

**Authors:** Jon Bourgeois (FDM, 3D model, animation; engines and pylons from the FlightGear Beluga model, wheels from the FlightGear 747-400, both GPL), Gary Scott (liveries), Peter Brendt "Jabberwocky" (V03 flight tuning).

**Imported:** Exterior `Models/C-5.xml` (`C-5.ac`, CF6-80C2 engines, landing gear, cargo bay, rear cabin, lights), flight deck (`Models/Cockpit`, PFD), textures; JSBSim FDM `C-5M-JSB.xml`.

**Livery:** The package's grey "436th" scheme with the US national insignia, "U.S. AIR FORCE" / command titles, unit badges, fin flag, flash band and serials painted out (`make_liveries.py`, `c5()`). No national insignia or registrations.

**Flight model:** JSBSim aerodynamics at run time, with Skyward calibration patches (`tools/fgimport/patches/aero.py`: body-alpha lift curve without the FDM's incidence offset, flap lift 0.030/deg, drag rise past M0.79, trim / elevator drag, pitch damping, rudder power, dihedral effect) checked against published C-5M figures at 280 t (stall, take-off run, climb, M0.79 max, landing run). The FDM's visual reference point does not match the model; the importer places the model with `vrp_m` so the loaded CG sits 1 m ahead of the front main bogies, ~5 m above the runway. Skyward CF6-80C2 engine model (4 × 228 kN), gear, reversers, spoilers.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, effect / weapon / pilot submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `import-report.txt`, `fdm.xml` (the JSBSim sections Skyward reads, patched as above) and `fdm.json` (derived mass / contact report). Rebuild: see `tools/fgimport/README.md` (`aircraft/c5.json`, then `patches/aero.py galaxy`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
