# Aerospatiale / BAC Concorde (`concorde`)

**Source:** FGAddon `trunk/Aircraft/Concorde` (SVN r21799, aircraft version 2.16) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/Concorde/>; mirror github.com/FGMEMBERS/Concorde (`af22b701`).

**Licence:** GPL. The JSBSim FDM (`Concorde.xml`) declares "GPL (General Public License)" in its file header; the FGMEMBERS mirror carries the GNU GPL version 2 as `COPYING`; FGAddon admits GPL content only. No version is named in the package files (GPL §9: any version may be chosen); distributed here under GPL v2 with the rest of Skyward — see `/NOTICE`.

**Authors:** the FlightGear Concorde developers; the package `Changelog.txt` credits contributions by C. Schmitt (pilot seats, tyre smoke, exhausts), T. Renk (2D / 3D cockpit textures) and V. Meazza (contrails).

**Imported:** Exterior `Models/Concorde_ba.xml` (`concorde.ac`: droop nose and visor, gear with the tail bumper wheel, elevons, rudders, secondary-nozzle buckets / reversers, reheat flames, lights), the flight deck (`Concorde_flightdeck.xml`, `Concorde_instruments.xml`: pedestal, INS, yokes, seats) and **its 2D instrument panels placed in 3D** (captain / co-pilot / centre, overhead, engineer's station; ~1,100 instrument placements) — imported by `tools/fgimport`'s panel support as live layers (needles, flags, switches). Crew figures, chocks, pushback, effects and flood-light cones are not imported. FDM: `Concorde.xml` (JSBSim).

**Livery:** Derived from the package's `concorde.png` (the G-BOAC scheme): airline titles, speedmarques, tail flag, registration and fleet letters painted out — a plain white Concorde with its door outlines (`tools/fgimport/liveries/make_liveries.py`, `concorde()`).

**Flight model:** JSBSim at run time with Skyward slender-delta aerodynamics (`tools/fgimport/patches/aero.py`): a Polhamus leading-edge-vortex lift curve (CL 0.72 at the 13.5° approach alpha, rising to ~1.3 near 30° with no stall break) scaled with Mach (supersonic lift slope ~2.2/rad at M2), drag due to lift with leading-edge suction at cruise alpha and none in the vortex regime (approach L/D ~4, M2 L/D ~7.5), the transonic wave-drag hump, reduced residual pitch stability (the centre of pressure moves aft with Mach through `aero_ref_pt_shift_x`, now evaluated by `src/jsbsim.rs`), and the trim-fuel transfer moving the CG aft by ~1.65 m through the transonic acceleration (`cg_schedule`, `src/flight/jets.rs`). Skyward Olympus 593 Mk 610 model (4 × 140 kN dry, 169 kN with reheat; intake ram recovery; 127 °C nose-temperature limit). Analogue fly-by-wire with autostabilisation (Skyward's airliner law, 17° alpha limit). Flap lever = droop nose / visor (UP, visor down, 5°, 12.5°). Flown at 118 t.

**Sound:** the Olympus recordings of the Vulcan (FGUK, `/assets/audio/CREDITS.md`) — the 593 is the Olympus 301's descendant; supersonic boom / Mach cone from Skyward's synthesis.

**Modifications:** Converted with `tools/fgimport`. Files here: `model.skac`, `t*.jpg|png`, `import-report.txt`, `fdm.xml` (the JSBSim sections Skyward reads, patched as above), `fdm.json`. Rebuild: see `tools/fgimport/README.md` (`aircraft/concorde.json`, then `patches/aero.py concorde`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
