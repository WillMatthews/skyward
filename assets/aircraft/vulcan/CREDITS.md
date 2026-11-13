# Avro Vulcan B.2 (`vulcan`)

**Source:** github.com/FGMEMBERS/AvroVulcanB2 (FGUK; `master` 3064c65f, 2016-05-25) — <https://github.com/FGMEMBERS/AvroVulcanB2>

**Licence:** GPL-2.0 (repository `COPYING` = GNU GPL version 2; no "or later" statement, so treated as GPL v2 — see `/NOTICE`). The model is based on Jonathan Harris's X-Plane Vulcan, which he released under the GPL (package `readme.txt`).

**Authors:** Stuart Buchanan (model, JSBSim FDM), "Stuart C" and FGUK, Jonathan Harris (original model), "Algernon" (sound), Jon Stockill (repaints), Lee Elliott; dials and FDM code borrowed from the FGUK Lightning.

**Imported:** Exterior `Models/AvroVulcanB2.xml` (`vulcanb2.ac`, gear, airbrakes, lights), cockpit (`Models/cockpit.xml`), textures; JSBSim FDM `vulcanb2.xml`. Weapons (bombs, Blue Steel, Red Beard, WE177, Shrike, ECM pod), exhaust heat-haze / flame effects and light cones are not imported.

**Livery:** The XM607 wrap-around camouflage with its roundels, fin flashes, serials and badges painted out (`make_liveries.py`, `vulcan()`, flat-colour fill). No national insignia or serials.

**Flight model:** JSBSim at run time with Skyward delta-wing aerodynamics (`tools/fgimport/patches/aero.py`): the FDM is an Aeromatic first cut with a conventional transport lift curve (sharp stall at 13°), replaced by a low-slope delta lift curve rising to ~28° with no stall break, high-alpha drag rise (lost leading-edge suction), induced drag for AR 3.1, drag rise past M0.88 and airbrake coefficients. The flap lever drives the airbrakes (the Vulcan has no flaps). Skyward Olympus 301 turbojet model (4 × 89 kN), gear; flown at 60 t.

**Sound:** the Olympus set, including the howl, from this package's `Sounds/` (see `/assets/audio/CREDITS.md`).

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, effect / weapon / pilot submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `import-report.txt`, `fdm.xml` (the JSBSim sections Skyward reads, patched as above) and `fdm.json` (derived mass / contact report). Rebuild: see `tools/fgimport/README.md` (`aircraft/vulcan.json`, then `patches/aero.py vulcan`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
