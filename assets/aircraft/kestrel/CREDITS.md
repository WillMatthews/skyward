# Cessna 172P Skyhawk (`kestrel`)

**Source:** c172p-team/c172p (GitHub, the FlightGear default Cessna 172P), commit 63ba852 — <https://github.com/c172p-team/c172p>

**Licence:** GPL-2.0-or-later (`LICENSE` = GNU GPL v2; FlightGear aircraft are distributed as GPL v2 or later).

**Authors:** David Megginson (original), Gilberto Agostinho, Wayne Bragg, Juan Vera del Campo, onox, thevirtualfer, Daniel Dubreuil, Jonathan Redpath, Jonathan Schellhase, Tuomas Kuosmanen, Anders Gidenstam, Waldo Kitty, algefaen, Horacio, D-ECHO, Jarl Arntzen, Benedikt Hallinger, Sascha Reißner, Josh Davidson (from `c172p-set.xml`).

**Imported:** Exterior model (`Models/c172p.xml`, `c172-common.ac`), the c172p panel / interior, default textures; JSBSim FDM `c172p.xml` aerodynamics, metrics, mass & ground reactions (`fdm.xml`).

**Livery:** Default (house) white/blue scheme; the wing underside carries the default texture's registration (a real civil registration, no airline branding).

**Flight model:** JSBSim aerodynamics evaluated at run time (src/jsbsim.rs); engine (160 hp O-320, fixed-pitch prop) and gear from Skyward's models, calibrated to the FDM / POH; loaded: pilot + passenger + 110 kg fuel = 932 kg.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.xml` (the JSBSim sections Skyward reads, includes resolved) and `fdm.json` (derived mass / contact report), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
