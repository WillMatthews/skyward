# Zlin Z-50 LX (`zlin`)

**Source:** FGAddon `trunk/Aircraft/Zlin-50lx` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/Zlin-50lx/>

**Licence:** GPL-2.0 (package `COPYING` = GNU GPL version 2; no "or later" statement in the package, so treated as GPL v2 — see `/NOTICE`).

**Authors:** Emmanuel Baranger (3D model, FDM), Pierre Geoffroy (FDM), Sébastien Marque (panel), Laurent Hayvel (texture); FDM updates by "OO ZVY".

**Why the Zlin:** of the aerobats asked for, FGAddon's `pittss1c` has no licence statement at all (no COPYING, no headers — policy-only GPL, not clearly GPL), the community Pitts S-2B (flightgearbrasil) uses files from the non-GPL "DavePack", and there is no GPL Extra 300. The Zlin Z-50 LX — a world-champion unlimited aerobat — is the best-quality clearly-GPL alternative.

**Imported:** Exterior `Models/z50lx.xml` and the cockpit / panel (`Models/Interior`). The pilot figure, registration (`Immat`) and the FG smoke particle system are not imported (Skyward draws its own smoke trail, `src/vfx/smoke.rs`).

**Livery:** The package's default sunburst scheme (no registration, no national or sponsor markings).

**Flight model:** Skyward parametric model calibrated to the YASim data (720 kg aerobatic, 12.5 m², 8.58 m, Lycoming AEIO-540 300 hp, MT three-blade constant-speed) and published Z-50 L figures: stall ≈ 58 kt, ≈ 3,100 fpm climb, 158 kt max level, +9 / −6 g; symmetric wing (flies inverted), strong autorotation (snap rolls, spins with a standard recovery); inverted fuel and oil systems (`src/systems`). See `src/flight/tests/ga_types.rs`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized and re-encoded; LODs, effects and cockpit pick logic dropped; animations mapped to Skyward channels). Rebuild: see `tools/fgimport/README.md` (`aircraft/z50lx.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
