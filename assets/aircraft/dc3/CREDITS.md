# Douglas DC-3 / C-47 (`dc3`)

**Source:** FGAddon `trunk/Aircraft/Douglas-Dc3` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/Douglas-Dc3/>

**Licence:** GPL-2.0 (package `COPYING` = GNU GPL version 2; no "or later" statement in the package, so treated as GPL v2 — see `/NOTICE`).

**Authors:** Emmanuel Baranger "Helijah" (3D model, textures, FDM), Gary Neely "Buckaroo" (YASim FDM, 2012), the PAF team (`contributors.txt`).

**Imported:** Exterior `Models/dc-3.xml` (wheeled variant: airframe, two R-1830 radials with cowlings and three-blade propellers, retractable main gear, tail wheel) and the cockpit (`Models/Interior`: panel, overhead, pedestal). Pilot figures, registration (`Immat`), the float / AI variants and the liveries directory are not imported.

**Livery:** Derived from the package's default texture (a 1950s airline scheme): the airline titles and the nose / fin emblems are painted out and the blue cheatline and fin sweep repainted in an original oxblood with a gold pin stripe (`tools/fgimport/liveries/make_liveries.py`, `dc3()`). No registration, no operator markings.

**Flight model:** Skyward parametric model calibrated to the YASim data (Buckaroo's notes: Vs0 64 KIAS at gross, 2 × 1,200 hp take-off / 1,050 METO, 3.48 m props, 91.7 m², 29 m) and published figures at 11 t: stall 73 kt clean / 63 kt flaps 45°, ≈ 1,130 fpm climb, ≈ 200 KTAS max at 8,500 ft, ≈ 160 KTAS cruise. See `src/flight/tests/ga_types.rs`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized and re-encoded; LODs, effects and cockpit pick logic dropped; animations mapped to Skyward channels). Rebuild: see `tools/fgimport/README.md` (`aircraft/dc3.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
