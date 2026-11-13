# fgimport — FlightGear aircraft → Skyward assets

Offline tool (not part of the game build). It turns a FlightGear aircraft
(AC3D models + model XML + textures, and the JSBSim FDM) into
`assets/aircraft/<id>/`: `model.skac` (zlib: JSON header with node tree,
animations, materials, chunks + vertex / index blobs), `t*.jpg|png`
textures, `fdm.xml` (the JSBSim sections the flight model reads, includes
resolved, with the loaded CG as `<skyward_cg>`), `fdm.json` (mass / CG /
contact / engine report) and `import-report.txt` (counts, unmapped
properties).

## Rebuild

```sh
export FG_SRC=/some/scratch/fgsrc          # downloads go here
S=tools/fgimport/scripts
SKIP="Liveries/,AI-,Armament,stores/,Ramp,chocks,Autopush,Operations/,Cabin/,EFB,paintkit,Primus-Epic,MCDU,Nasal/,Sounds/,gui/,Tutorials,Dialogs,Splash,Previews,walker,Human/,submodels" \
python3 -I $S/fetch.py c172p gh:c172p-team/c172p@master c172p Models/c172p.xml c172p.xml
python3 -I $S/fetch.py ask21 svn:ASK21 ASK21 Models/ask21.xml ask21-jsb.xml
python3 -I $S/fetch.py spitfire svn:spitfireIX spitfireIX Models/SpitfireIX.xml spit9-yasim.xml
python3 -I $S/fetch.py f16 svn:f16 f16 Models/F-16.xml f16b52.xml
python3 -I $S/fetch.py atr72 svn:ATR-72-500 ATR-72-500 Models/ATR-72-500.xml ATR-72-500.xml
python3 -I $S/fetch.py e190 gh:YV3399/E-jet-family-YV@master E-jet-family Models/E190.xml FDM/E190.xml
python3 -I $S/fetch.py a320 svn:A320-family A320-family Models/A320-200-CFM.xml FDE/A320-200-CFM.xml
# v3 heavies (Track H): C130 from FGAddon; C-5 and the Vulcan are git clones of
# github.com/FGMEMBERS/C-5 and /AvroVulcanB2 placed at $FG_SRC/c5 and $FG_SRC/AvroVulcanB2
python3 -I $S/fetch.py c130 svn:C130 C130 Models/c130.xml Systems/FDM/c130-yasim.xml
# extra inputs: ATR Models/paintkit.xcf, A320 Models/Liveries/CFM/2k/CFM56-white.png
python3 -I $S/texconv.py $FG_SRC                       # SGI .rgb / DDS → PNG
python3 -I tools/fgimport/liveries/make_liveries.py $FG_SRC   # neutral A320 / ATR liveries
python3 -I tools/fgimport/liveries/make_liveries.py $FG_SRC c130 c5 vulcan   # heavies: insignia painted out
cargo run --release --manifest-path tools/fgimport/Cargo.toml -- \
    tools/fgimport/aircraft/c172p.json $FG_SRC/c172p assets/aircraft/kestrel   # etc.
# The C-5 and Vulcan JSBSim aero get Skyward calibration patches after import:
python3 -I tools/fgimport/patches/aero.py galaxy assets/aircraft/galaxy/fdm.xml
python3 -I tools/fgimport/patches/aero.py vulcan assets/aircraft/vulcan/fdm.xml
```

`FGIMPORT_DUMP=<regex>` prints model-frame bounding boxes of matching objects.
In the game: `SKAC_DUMP=<id> cargo test --release geometry_dump -- --ignored
--nocapture` prints the posed (gear-down) geometry in the body frame.

## Config (`aircraft/<name>.json`)

| key | meaning |
|---|---|
| `id`, `name`, `fg_name` | Skyward id (output dir), display name, `Aircraft/<fg_name>/` prefix used in the FG files |
| `model` | entry model XML |
| `fdm` | `{"jsbsim": "file.xml"}` (CG / VRP / contacts from the FDM) or `{"cg_model": [x,y,z]}` (YASim types: CG in model metres) |
| `fuel_kg`, `pointmass_kg` | loaded condition for the CG / inertia |
| `vrp_m` | JSBSim types: override the FDM's VRP (structural metres) when it does not match the visual model (C-5) |
| `eye_view` / `eye_model` / `eye_body` | cockpit eye (FG view offsets / model frame / body frame) |
| `props` | static property values (variants, damage flags …) |
| `channels` | `[regex, channel, scale, offset]` property → Skyward channel rules (before the built-ins) |
| `exclude_models`, `exclude_objects`, `interior_models`, `interior_objects`, `light_models` | regexes on submodel paths / object names |
| `textures` | texture overrides (basename → path in the source tree), e.g. the neutral liveries |
| `max_tex`, `interior_max_tex`, `jpeg_quality`, `range_m` | texture limits; LOD distance for `range` animations |

Supported FG animations: rotate / knob, translate / slider, spin, select,
scale, range (static LOD pick), material (emission → animated emissive,
static texture swap), named animation groups, `alpha-test` / `blend` /
transparent effects (glass), light-cone / procedural lights (hidden; their
positions become light markers). Expressions: factor / offset / min / max,
interpolation tables, SGExpression subset, `<condition>` trees. Static inputs
are folded at import; FlightGear's rotation sense is right-handed about the
axis (verified by `aircraft_asset::sign_tests`).

Model frame (FG): x aft, y right, z up. Body frame (Skyward): +Z nose,
+Y up, +X left, origin = loaded CG.
