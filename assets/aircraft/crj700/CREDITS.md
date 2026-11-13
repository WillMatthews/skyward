# Bombardier CRJ700 (`crj700`)

**Sources:**
- Model, textures, cockpit: github.com/FGMEMBERS/CRJ700-family (`master` 4862db2f, 2016-02-18) — <https://github.com/FGMEMBERS/CRJ700-family>
- JSBSim FDM: FGAddon `trunk/Aircraft/CRJ700-family/CRJ700jsb.xml` and its engine / system files (SVN r21799) — <https://sourceforge.net/p/flightgear/fgaddon/HEAD/tree/trunk/Aircraft/CRJ700-family/>

**Licence:** GPL-2.0-or-later. The 2016 snapshot's `LICENSE` reads "All models, textures, markup and scripts licensed under the GNU General Public License, version 2 or above"; the FGAddon `LICENSE` puts "all markup (XML)" — the FDM — under GPL v2 or above. (Later FGAddon revisions relicensed most models and textures under CC-BY-4.0: those are **not** used; the import is the GPL snapshot.)

**Authors:** Ryan "Skyop" Miller (initial model and FDM), D-JSB and Daniel M. (Daniel Meissner; 2015 overhaul, JSBSim FDM), Henning S. (JSBSim variant); liveries / models by "Zbyszek", "SP-LEC", "07-BUT", Roland Hader ("Quix0r"), Robert Maheu, Thomas Ito-haigh; thanks to Emilian Huminiuc, Vivian Meazza, Syd Adams (package `README.md`).

**Imported:** Exterior `Models/CRJ700.xml` (`CRJ700.ac`, gear, engines with reversers, surfaces, slats, spoilers, lights), the flight deck (`CRJ700-flightdeck.xml`: PFD / MFD / EICAS, CDUs, MCP, overhead, side panels, standby instrument, clock) and the cabin, textures. The layered PFD / MFD / EICAS screens are mapped to Skyward's `pfd` / `nd` / `ewd` displays (their FlightGear overlay objects are not imported); the CDUs to `mcdu`. Pushback, pilots and effects are not imported. FDM: `CRJ700jsb.xml` (JSBSim).

**Livery:** The package's `CRJ700-Blank.png` (white, no titles, logos or registration) — used as is.

**Flight model:** JSBSim at run time with Skyward calibration patches (`tools/fgimport/patches/aero.py`): the FDM is an Aeromatic first cut (linear lift to CL 2.3 at 21°, Mcrit 0.53), replaced by a CRJ lift curve (CLmax ~1.35 at 13°), flap lift for 45°, induced drag for AR 7.6, drag rise past M0.80 (Mmo 0.825), T-tail elevator power (Cm_de ~2.4/rad) and more pitch stability / damping. Skyward CF34-8C5B1 model (2 × 61.3 kN). Flown at 29.7 t. The VRP is set (`vrp_m`) to place the 2016 model on the FDM frame.

**Sound:** the E-Jet's CF34-class recordings (`/assets/audio/CREDITS.md`).

**Modifications:** Converted with `tools/fgimport` (as for the other types). Files here: `model.skac`, `t*.jpg|png`, `import-report.txt`, `fdm.xml` (the JSBSim sections Skyward reads, patched as above), `fdm.json`. Rebuild: see `tools/fgimport/README.md` (`aircraft/crj700.json`, then `patches/aero.py crj700`).

These files are covered by the licence above; Skyward as a whole is GPL-2.0-or-later (see `/LICENSE`, `/NOTICE`).
