# Piper PA-18 Super Cub (`cub`)

**Source:** FGAddon `trunk/Aircraft/Piper-PA-18` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/Piper-PA-18/>

**Licence:** GPL-2.0 (package `COPYING` = GNU GPL version 2; no "or later" statement in the package, so treated as GPL v2 — see `/NOTICE`).

**Authors:** Emmanuel Baranger (3D model, YASim FDM, textures).

**Why not the J-3 Cub:** FGAddon's `J3Cub` package ships the GNU GPL **version 3** (`copying.txt`; README "License: GPL"). GPL v3-only content cannot be combined with Skyward's GPL v2-only content (A320 family, heavies), so the closely related, clearly GPL v2 PA-18-150 Super Cub stands in for it.

**Imported:** Exterior `Models/pa18.xml` (fuselage, wing, struts, tundra-tyre gear, propeller) and the cockpit / panel (`Models/Interior`). The pilot figure and the registration submodel (`Immat`) are not imported.

**Livery:** The package's default Cub-yellow fabric (no registration, no markings).

**Flight model:** Skyward parametric model calibrated to the YASim data (PA-18-150: 794 kg MTOW, 16.58 m², 10.73 m, Lycoming O-320 150 hp, fixed-pitch prop) and published Piper figures: stall 41 kt clean / 37 kt flaps, 960 fpm climb, 113 kt max, ≈100 m take-off run and landing roll. See `src/flight/tests/ga_types.rs`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized and re-encoded; LODs, effects and cockpit pick logic dropped; animations mapped to Skyward channels). Rebuild: see `tools/fgimport/README.md` (`aircraft/pa18.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
