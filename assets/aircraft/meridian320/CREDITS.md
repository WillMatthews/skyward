# Airbus A320-200 (CFM56) (`meridian320`)

**Source:** FGAddon `trunk/Aircraft/A320-family` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/A320-family/>

**Licence:** GPL-2.0 **only** (`README.copyright.md`: "under the terms of version 2 of the GNU General Public License"; `LICENSE`). This is why the combined work must stay distributable under GPL v2 (see the top-level LICENSE note).

**Authors:** Josh Davidson / Octal450 (FDM, systems, instruments), Jonathan Redpath / legoboyvdlp, merspieler, Matthew Maring, Thorsten Herrmann / TH-555, Semir Gebran / CaptB (3D, textures).

**Imported:** Exterior `Models/A320-200-CFM.xml` (`A320-216.ac`, `CFM56.ac`), flight deck, gear, textures; JSBSim FDM `FDE/A320-200-CFM.xml` + `Config/*`.

**Livery:** Neutral white scheme derived from the package's unbranded factory-primer texture (`make_liveries.py`: primer → white / light-grey wings, registration and fin masking bands painted out) and the package's plain white CFM56 nacelle texture. The default Swiss / American / Delta / Austrian liveries are not used.

**Flight model:** JSBSim aerodynamics at run time; Skyward normal-law FBW, CFM56-5B4 engine model (120 kN) and gear; 60.4 t (the FDM's default payload).

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG; LODs, damage/variant/pilot/effect submodels, procedural-light billboards and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.xml` (the JSBSim sections Skyward reads, includes resolved) and `fdm.json` (derived mass / contact report), `import-report.txt`. Rebuild: see `tools/fgimport/README.md`.

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`).
