# Ship hulls (`assets/ships/`)

**Source:** FlightGear fgdata, branch `next`, `Models/Maritime/Civilian/` — <https://gitlab.com/flightgear/fgdata/-/tree/next/Models/Maritime/Civilian>

| id | FlightGear model | Licence |
|---|---|---|
| `container` | ContainerShip.xml / .ac (+ ContainerShip.png) | SAVAGE BSD-3 (hull); texture fgdata GPL v2 |
| `cruise` | CruiseShip.xml / .ac | SAVAGE BSD-3 |
| `freighter` | Freighter.xml / .ac | SAVAGE BSD-3 |
| `ferry` | LargeFerry.xml / .ac | SAVAGE BSD-3 |
| `ferry_small` | MediumFerry.xml / .ac | SAVAGE BSD-3 |
| `trawler` | Trawler.xml / .ac | SAVAGE BSD-3 |
| `trawler_large` | LargeTrawler.xml / .ac | SAVAGE BSD-3 |
| `sailboat` | SailBoatUnderSail.xml / .ac | SAVAGE BSD-3 |
| `barge` | barge.xml / .ac | SAVAGE BSD-3 |
| `tanker` | Tanker_sailing.xml / Tanker.ac | fgdata GPL v2 |
| `motorsailer` | SailBoatUnderPower11m.xml / .ac | fgdata GPL v2 |

**SAVAGE:** converted by FlightGear from the X3D originals of the Naval
Postgraduate School SAVAGE project (<https://savage.nps.edu/Savage/>),
Copyright (c) 1995-2007 the authors, BSD-3-Clause — the full notice is in
`LICENSE-SAVAGE.txt` (from fgdata's `readme.savage`). Neither NPS nor the
MOVES Institute endorses Skyward.

**Modifications:** converted with `tools/fgimport` (configs and the rebuild
script in `tools/fgimport/ships/`): wakes, light billboards and ground
planes dropped (Skyward draws its own wakes and COLREGs lights), textures
re-encoded at most 1024 px. At run time the LargeFerry and barge are turned
90° (they were built along another axis) and every hull is centred
midships and scaled to the ship's length.
