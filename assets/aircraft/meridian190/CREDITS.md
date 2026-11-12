# Embraer E190 (`meridian190`)

**Source:** E-jet-family-YV (GitHub YV3399/E-jet-family-YV, maintained by tdammers), commit 1338d8f — <https://github.com/YV3399/E-jet-family-YV>

**Licence:** GPL-2.0-or-later (`COPYING.md`: "released under the terms of the GNU General Public License (GPL) version 2.0 or later"; `License.txt`). Files with other terms (sounds, a font, an EFB icon) are not imported.

**Authors:** Gabriel Hernandez (YV3399), D-ECHO, Tobias Dammers, Semir Gebran (3D), Stefan Frank (3D, liveries), Jonathan Redpath; original version by Narendran Muraleedharan with Skyop, HanselVB, AndersG, Brandano, Geir and others (README).

**Imported:** Exterior `Models/E190.xml` (`erj190.ac`) with the flight deck, gear, engines, textures; JSBSim FDM `FDM/E190.xml` + `aerodynamics.xml`.

**Livery:** The model's default white house scheme with a dark radome (no airline titles).

**Flight model:** JSBSim aerodynamics at run time (aero-config constants from the package's `common.xml`); Skyward engines (2 × CF34-10E, 91 kN) and gear; 40 t.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.xml` (the JSBSim sections Skyward reads, includes resolved) and `fdm.json` (derived mass / contact report), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
