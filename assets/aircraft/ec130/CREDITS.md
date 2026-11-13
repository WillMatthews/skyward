# Airbus Helicopters EC130 B4 (`ec130`)

**Source:** FGAddon `trunk/Aircraft/ec130` (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/ec130/>

**Licence:** GPL-2.0-or-later. The package ships the GNU GPL version 2 (`COPYING`); its model / FDM / set files carry the standard notice "either version 2 of the License, or (at your option) any later version" (`ec130b4-set.xml`, `ec130-base.xml`, `Models/ec130b4_fdm.xml`), and `<license>GPL v2</license>`.

**Authors:** Heiko Schulz (FDM, model, systems), Melchior Franz (Nasal), Michael Habarta (model details, animations, liveries, instruments). "EUROCOPTER" / "Airbus Helicopters" are trade marks of their owner; the type name is used descriptively only.

**Imported:** Exterior and cabin `Models/ec130b4.xml` (`ec130b4_t2.ac`: fuselage, Starflex main rotor with blades and blur disc, Fenestron, skids, lights), the B4 instrument panel (`Models/Instruments/`: airspeed, attitude, altimeter, VSI, turn, clock, compass, Nr / N2 tachometer tape, caution panel, levers, radios) with live needles, and the two VEMD screens (drawn by Skyward, `src/rotor/instruments.rs`). Not imported: people, luggage, floats, searchlights, ground equipment, rotor wakes (Skyward draws its own rotor wash), the T2 variant. The YASim FDM (`Models/ec130b4_fdm.xml`) is reference data only (geometry: hub, Fenestron, skids, CG).

**Livery:** The package's default FlightGear demonstrator scheme with the "EC 130" lettering and the www.FlightGear.org web address removed (`tools/fgimport/liveries/make_liveries.py`, `ec130()`). No operator markings or registrations.

**Flight model:** Skyward's own rotorcraft model (`src/rotor/heli.rs`): blade-element / momentum main rotor with rotor RPM, turboshaft and governor, Fenestron, autorotation, ground effect, vortex ring state, retreating-blade stall; calibrated to published EC130 B4 data (MTOW 2,427 kg, Arriel 2B1 632 kW take-off, hover ceilings IGE 3,098 m / OGE 2,537 m, Vne 155 kt, fast cruise 130 kt). See `src/rotor/tests.rs`.

**Modifications:** Converted with `tools/fgimport` (AC3D + model XML → `model.skac`; textures downsized to ≤2048 px (cockpit ≤1024) and re-encoded as JPEG/PNG, SGI `.rgb` textures converted with `scripts/texconv.py`; effect / pick / tooltip logic dropped; rotor `rpm` / blade pitch / flapping and the instruments mapped to Skyward channels; the blur disc's rpm-driven transparency becomes a visibility switch). Files here: `model.skac` (geometry, node tree, animations), `t*.jpg|png` (textures), `fdm.json`, `import-report.txt`. Rebuild: see `tools/fgimport/README.md` (`aircraft/ec130.json`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
