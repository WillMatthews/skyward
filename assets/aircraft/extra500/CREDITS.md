# Extra EA-500 (`extra500`)

**Source:** FGAddon `trunk/Aircraft/extra500` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/extra500/>

**Licence:** GPL-2.0-or-later (package `README` and every file header: "either version 2 of the License, or (at your option) any later version"; `extra500-set.xml` `<license-description>`: GNU GPL v2). Distributed here under GPL v2 as part of the combined work (see `/NOTICE`).

**Authors:** Dirk Dittmann (JSBSim, Nasal), Eric van den Berg (JSBSim, Nasal, autopilot), Thomas Grossberger (3D model) — package `AUTHORS`.

**Imported:** Exterior `Models/extra500.xml` (airframe, wing, five-blade propeller, retractable gear, lights) and the cockpit (`Models/interior`, `panels/`, `instruments/`): the two Avidyne-style IFD screens (`IFD.Screen`) show Skyward's PFD (left) and moving map (right); standby ASI / altimeter / attitude, engine (torque, TOT, N1 / N2, oil) and fuel gauges are bound to live values. Passengers, luggage, jacks, ground-power cart, shadow and ice-overlay submodels and the text-rendered registration are not imported.

**Livery:** The package's default pearl-white scheme (no registration: the FG text registration is not imported). No operator markings.

**Flight model:** The package's JSBSim aerodynamics (`extra500-aerodynamics.xml`) is a per-surface model (local wing / tailplane / fin velocities, free-floating ailerons) built on custom system properties that Skyward's JSBSim evaluator does not run; the type flies on Skyward's parametric model using that FDM's geometry, mass and inertia (1,986 kg loaded, 14.03 m², 11.0 m, Rolls-Royce 250-B17F/2 450 shp, MT five-blade 2.1 m) calibrated to published figures: stall 61 kt flaps down, ≈1,600 fpm climb, ≈ 200–210 KTAS cruise high up. See `src/flight/tests/ga_types.rs`. `fdm.json` (mass / CG / contacts report) is kept for reference.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded; LODs, effects and cockpit pick/knob logic dropped; animations mapped to Skyward channels). Rebuild: see `tools/fgimport/README.md` (`aircraft/extra500.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
