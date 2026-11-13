# Cessna Citation X (`citationx`)

**Source:** github.com/FGMEMBERS/CitationX (`master` e497ad98, 2019-09-23; the FGAddon `CitationX` with later fixes) — <https://github.com/FGMEMBERS/CitationX>

**Licence:** GPL-2.0 (repository `COPYING` = GNU GPL version 2, added in 2016 by a FlightGear contributor; several of the package's Nasal files carry GPL headers; the model files themselves have no per-file notice; FGAddon admits GPL content only). Treated as GPL v2 — see `/NOTICE`.

**Authors:** Syd Adams (model, YASim FDM, instruments); later fixes by Ludovic Brenta, Jonathan Redpath and other FlightGear contributors (repository history).

**Imported:** Exterior `Models/Citation-X.xml` (`CitationX.ac`: gear, flaps, slats, speedbrakes, ailerons, elevators, rudder, reversers, airstair door, lights), the flight deck (panel with its live Primus 2000 PFD / MFD / EICAS instruments, pedestal, glareshield, standby gauges) and the cabin, textures. The registration text and shadow models are not imported. The cockpit walls are the fuselage skin seen from inside: Skyward adds an inward-facing grey copy of it (`interior_shell`). FDM is YASim (`CitationX.xml`): reference data only.

**Livery:** The package's default `livery1.png` (white with purple / maroon / grey swooshes, no titles, logos or registration) — used as is.

**Flight model:** Skyward parametric model calibrated to the YASim data (2 × AE3007C1, 30 kN each; 48.96 m² wing, 37° sweep) and published Citation X figures (Mmo 0.92 / 527 KTAS, Vref ~121 kt at 14 t, TOFL 1,567 m at MTOW), flown at 14 t. CG estimated from the model (7 % of the weight on the nose wheel).

**Sound:** the E-Jet's CF34-class recordings (`/assets/audio/CREDITS.md`) with a 24-blade fan tone.

**Modifications:** Converted with `tools/fgimport` (as for the other types). Files here: `model.skac`, `t*.jpg|png`, `import-report.txt`. Rebuild: see `tools/fgimport/README.md` (`aircraft/citationx.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
