# Schleicher ASK 21 (`swift`)

**Source:** FGAddon `trunk/Aircraft/ASK21` (SourceForge SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/ASK21/>

**Licence:** GPL-2.0-or-later (README: "GNU General Public License (GPL), version 2 or, at your option, any later version"; `COPYING`).

**Authors:** Heiko Schulz, Detlef Faber (original, based on the Bocian by AJ MacLeod); Thomas Polzer, Viktor Radnai, Bea Wolf, Gilberto Agostinho (2016 update, parts from the ASK13 by Patrice Poly); Herbert Wagner; Anders Gidenstam (dual control).

**Imported:** Exterior model (`Models/ask21.xml`) with cockpit, textures; JSBSim FDM `ask21-jsb.xml` (aerodynamics, metrics, mass, contacts).

**Livery:** Default white glider scheme (competition number / registration as in the default texture).

**Flight model:** JSBSim aerodynamics at run time; airbrakes = the FDM's speed-brake property. Two pilots, 520 kg.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.xml` (the JSBSim sections Skyward reads, includes resolved) and `fdm.json` (derived mass / contact report), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
