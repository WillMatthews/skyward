# Supermarine Spitfire Mk IX (`peregrine`)

**Source:** FGAddon `trunk/Aircraft/spitfireIX` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/spitfireIX/>

**Licence:** GPL-2.0 (README: "released under the terms of the GPLv2"; FGAddon content is GPL v2+).

**Authors:** Detlef Faber; cockpit instruments by AJ MacLeod (photos courtesy spitfirespares.co.uk), Vivian Meazza, Anders Gidenstam.

**Imported:** Exterior model (`Models/SpitfireIX.xml`, `spitfire9.ac`), cockpit, propeller / prop disc, textures. FDM is YASim (`spit9-yasim.xml`): used as reference data only.

**Livery:** Stock FlightGear RAF camouflage, including RAF roundels and squadron codes (kept: removing national insignia from the texture atlas was not practical).

**Flight model:** Skyward parametric model calibrated to the YASim data (mass, geometry, Merlin power) and published performance; CG and gear geometry measured from the visual model.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
