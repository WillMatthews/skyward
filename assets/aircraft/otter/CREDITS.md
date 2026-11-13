# de Havilland Canada DHC-3 Otter on floats (`otter`)

**Source:** FGAddon `trunk/Aircraft/dhc3` (SVN r21799), float variant `dhc3F` — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/dhc3/>

**Licence:** GPL-2.0 (package `COPYING` = GNU GPL version 2; no "or later" statement in the package, so treated as GPL v2 — see `/NOTICE`).

**Authors:** Emmanuel Baranger and Pierre Geoffroy (3D model, YASim FDM; based on Syd Adams's DHC-2 Beaver).

**Why not the Beaver:** FGAddon's `dhc2` (DHC-2 Beaver, `dhc2F` floats) has no licence statement anywhere in the package (no COPYING, no headers — policy-only GPL, like the ATR flagged in PLAN.md), so it is not imported. The Otter — the Beaver's bigger sister, the same family of bush floatplane — is clearly GPL v2.

**Imported:** Exterior `Models/dhc3.xml` with `sim/model/variant = 2` (straight floats with water rudders, struts and paddle) and the cockpit / panel (`Models/Interior`). Pilot figure, registration (`Immat`) and the FG wake / spray particle effects are not imported (Skyward draws its own spray, `src/vfx/spray.rs`).

**Livery:** The package's default yellow scheme with black / orange / red cheatlines and the "Otter" type script (no registration, no operator markings).

**Flight model:** Skyward parametric model calibrated to the YASim data (3,500 kg on floats, 34.84 m², 17.69 m, P&W R-1340 600 hp, three-blade constant-speed) and published DHC-3 figures (stall 50–55 kt flaps down, ≈ 750 fpm climb on floats, ≈ 120 kt max). Float hydrodynamics are Skyward's (`src/water.rs`: buoyancy, planing, hump, step, water rudders, waves). See `src/flight/tests/ga_types.rs`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized and re-encoded; LODs, effects and cockpit pick logic dropped; animations mapped to Skyward channels). Rebuild: see `tools/fgimport/README.md` (`aircraft/dhc3.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
