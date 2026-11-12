# ATR 72-500 (`aerolink`)

**Source:** FGAddon `trunk/Aircraft/ATR-72-500` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/ATR-72-500/>

**Licence:** No licence file or per-file notice in the package; distributed through FGAddon, whose contribution policy requires GPL v2+ compatible content (FlightGear is GPL v2). Treated as GPL-2.0-or-later — the least certain provenance of the set.

**Authors:** Ryan Miller (3D model, FDM).

**Imported:** Exterior `Models/ATR-72-500.xml` (+ engines, gear, flight deck, props, textures). FDM is YASim (`ATR-72-500.xml`): reference data only.

**Livery:** Neutral Skyward livery made from the package's own paint kit (`paintkit.xcf` base layers) with an original slate cheat-line (`tools/fgimport/liveries/make_liveries.py`). The real-airline liveries (American Eagle, FedEx, Air New Zealand) are not used.

**Flight model:** Skyward parametric model calibrated to the YASim data (2 × PW127F 2,750 shp, 1.965 m props) and published performance; CG estimated from the YASim geometry.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
