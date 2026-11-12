# General Dynamics F-16C (Block 52) (`lancer`)

**Source:** FGAddon `trunk/Aircraft/f16` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/f16/>

**Licence:** GPL-2.0-or-later (`copyright.txt`: "version 2 of the License, or (at your option) any later version"; `LICENSE`).

**Authors:** Erik Hofman (main), Nikolai V. Chr. (FBW, aero blocks), J Maverick 16, Richard Harrison, Josh Davidson, Jonathan Redpath, Martin "Pegasus" Schmitt, Prohm "Rama" Snitwong, Pensacola, Justin Nicholson, Martien van der Plas, Petar Jedvaj, Fabien Barbier, NC-687, Adam Barszczak, Gary Brown, Thorsten Renk and others (see FGAddon `authors.txt`).

**Imported:** Exterior `Models/F-16.xml` with the cockpit, nozzle, gear, textures; JSBSim FDM `f16b52.xml` (block-15 aerodynamics data set), metrics, mass, contacts.

**Livery:** Stock grey scheme of the default model (small national / unit markings from the default texture may be visible).

**Flight model:** JSBSim aerodynamics at run time (stabilator / flaperon / rudder, leading-edge-flap schedule); Skyward's FBW law, F100-PW-229 engine model (79 kN / 129 kN) and gear. 10.97 t.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.xml` (the JSBSim sections Skyward reads, includes resolved) and `fdm.json` (derived mass / contact report), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
