# Skyward

A native Rust flight simulator flown over **real OpenStreetMap scenery** —
anywhere on Earth, for as long as you like. wgpu (Vulkan), winit, egui.

## Features

* **Endless streamed world** — OpenMapTiles vector tiles (OpenFreeMap, no key)
  stream in around the aircraft: detailed z14 tiles (extruded buildings with
  facades and windows, instanced trees, roads, water, parks, airfields with
  merged runways) out to ~7.5 km, generalised z10 tiles to ~30 km, with
  collision and a moving map. Tiles are cached in `cache/tiles/`.
* **Long flights** — a floating origin re-centres the local frame on the
  aircraft as it flies (see *Floating origin* below), so London → Paris (or
  further) stays precise, undistorted and hitch-free.
* **Rendering** — physically based sky and aerial perspective (Hillaire-style
  atmosphere LUTs + froxels), volumetric raymarched cumulus / stratus / cirrus
  with temporal reprojection and cloud shadows, cascaded shadow maps, planar
  water reflections, SSAO, MSAA HDR, bloom and **AgX** tonemapping; seasons
  (leaf fall, autumn colours, crop calendar), rain, snow, wet ground and fog.
* **Live weather & time** — the nearest METAR (aviationweather.gov) and
  Open-Meteo (wind aloft, cloud layers, snow depth) drive clouds, wind,
  gusts, turbulence, visibility and precipitation; the sun follows the real
  local time and date. The weather is re-fetched as a long flight moves on.
* **Wind** — boundary-layer gradient, gusts, mechanical and convective
  turbulence, thermals under the cumulus that drift with the wind aloft.
* **Seven aircraft** with one data-driven flight model (lift / drag polars,
  stall, ISA atmosphere, engines and propellers, gear, flaps, spoilers,
  reversers, asymmetric thrust, FBW for the fighter) and procedural models.
* **Autopilot** (heading / altitude / V/S / speed with autothrottle).
* **Auto flight bot** — an AI pilot that flies the real flight model through
  the controls: glider soaring and powered take-off → landmark tour →
  approach → landing cycles, wind-aware (see below).
* **Cameras** — chase, cockpit, tower, fly-by and an auto-cycling cinematic
  camera; headless capture / soak-test modes for testing without a window.

Default start: London City Airport (EGLC). Any lat/lon or airport code works.

## Run

```sh
cargo run --release
cargo run --release -- --lat 51.3308 --lon 0.0325
cargo run --release -- --airport EGKB        # resolved via Nominatim, cached
cargo run --release -- --watch --aircraft aerolink   # sit back: auto flight + cinematic camera
```

You spawn on the runway nearest the start point. On hybrid-graphics laptops it
asks wgpu for the high-performance adapter (e.g. the discrete NVIDIA GPU).

### Aircraft
Pick from the Tab menu (grouped by class). Types that need a long runway are
moved to the nearest suitable airfield (e.g. the Meridians start at Heathrow
when London City is too short) — the HUD says why.

| id | | |
|---|---|---|
| `kestrel` | **Kestrel C1** | 2-seat high-wing trainer. Stall 48 kt, cruise ~115 kt. |
| `swift` | **Swift S2** | 15 m sailplane, L/D 38:1. Released near a thermal; airbrakes on F / V. |
| `peregrine` | **Peregrine Mk IX** | V12 warbird, taildragger — swings left on take-off (right rudder), retractable gear. |
| `lancer` | **Lancer F2** | Single-engine FBW fighter, afterburner above 90 % throttle, speed brake, supersonic. |
| `aerolink` | **Aerolink 72** | Twin turboprop regional airliner, reverse pitch. Needs 1,800 m. |
| `meridian190` / `meridian320` | **Meridian 190 / 320** | Twin-jet airliners: spoilers, reversers, slats/flaps, autopilot. Need 1,800 m. |

Starts per type: runway, final approach (airliners: 10 nm at 3,000 ft), over
the city (fast jets: 3,000 m), glider thermal release.

### Controls
| Key | Action |
|---|---|
| ↑ / W, ↓ / S | pitch down / up |
| ← / A, → / D | roll |
| Q / E | rudder (steers on the ground) |
| Shift / Ctrl, = / -, PgUp / PgDn | throttle (Lancer: above the 90 % detent = afterburner) |
| F / V | flaps (glider: airbrakes) out / in |
| G | landing gear up / down (3 green = down and locked, red = in transit) |
| / and \ (or ') | spoilers / speed brake out / in one step |
| T | thrust reverse on / off (ground only; throttle then sets reverse power) |
| B or Space (hold) | brakes |
| [ / ] | trim nose down / up |
| Z | autopilot on (holds heading, altitude, speed) / off; a mouse AP panel appears |
| Ins / Del, Home / End, 9 / 0 | autopilot heading ∓5°, altitude ±100 ft, speed ∓5 kt |
| K | engine-failure practice (fails engine 1, then 2, then restores) |
| J | **auto flight**: the bot flies; any stick / throttle / lever input takes back |
| C | camera: chase → cockpit → tower → fly-by → cinematic |
| Right-drag, wheel | orbit / zoom chase camera |
| M, `,` / `.` | moving map on / off, zoom out / in |
| X | time warp 1 / 2 / 4 / 8× |
| P | pause |
| R | restart (back at the start point) |
| Tab | menu: aircraft, start, auto flight (watch), time of day, weather, live |
| H / F1 | help |
| F12 | screenshot |
| Esc | quit |

Stick inputs disconnect the autopilot (flashing AP DISCONNECT); throttle keys
only drop the autothrottle.

### Auto flight (watch mode)
Tick **Auto flight (watch)** in the menu, `--watch`, or press **J** in flight.
The bot flies the real flight model through the controls and autopilot —
nothing is scripted or teleported:

* **Glider** — reads the total-energy vario, judges cumulus by size, glides at
  the MacCready speed-to-fly (faster into a headwind, slower downwind),
  centres thermals in the frame that drifts with the wind aloft (lift-weighted
  circle fit + orbit guidance, ~38° bank), aims off for a cloud's drift,
  leaves near cloud base, wanders along a sightseeing route (a different
  route each launch / lap), always keeps a runway within safe glide *in the
  wind*, and lands into wind (airbrakes on a steep final) when the lift runs
  out. In watch mode it is relaunched after landing.
* **Powered** — take-off into wind with centreline tracking, rotation at Vr,
  gear / flap retraction, autopilot climb, a tour of landmarks (London sights
  when in range, else tall buildings / water from the map), obstacle-checked
  approach on a computed glidepath (London City needs ~5.5° for some types),
  stabilised gate (else go-around), flare, rollout with spoilers / reversers /
  brakes, taxi (backtracking when it landed into wind), turn round, repeat.
  The Lancer adds an afterburner climb, a fast low pass and a power-on flare.
* **Wind** — every leg is flown as a track (wind-correction angle from the
  air data), the runway end is chosen by headwind component, the approach
  speed gets half the headwind plus the gust, the glidepath descent rate
  follows the ground speed, and in the flare the bot de-crabs with rudder and
  holds the centreline wing-low.

The cinematic camera cycles chase angles, fly-bys, tower, nose and wingtip shots.

## Command line

| Flag | Meaning |
|---|---|
| `--lat L --lon L` | start point (default EGLC 51.5045, 0.0300) |
| `--airport CODE` | start at an airport (ICAO / name, resolved via Nominatim, cached) |
| `--aircraft ID` | `kestrel` `swift` `peregrine` `lancer` `aerolink` `meridian190` `meridian320` |
| `--start S` | `runway` `approach` `city` `thermal` |
| `--bot` | hand the aircraft to the auto-flight bot |
| `--watch` | auto flight + cinematic camera (+ glider relaunch) |
| `--live` / `--no-live` | follow the real weather and time (default on) / don't |
| `--metar "RAW"` | fixed weather from a METAR, e.g. `"EGLC 121450Z 18015G25KT 9999 SCT040 15/08 Q1015"` |
| `--weather W` | cloud preset `clear` `few` `scattered` `broken` `overcast` |
| `--time HH.MM` | local clock time (turns live time off) |
| `--date MM-DD` | date (season, sun) |
| `--radius-km R` | detailed-tile load radius |
| `--map-zoom N` | moving-map zoom step 0–5 |
| `--mute` | *(audio — Track K, in progress: engine / wind sound; flags and keys to be documented here)* |

Headless (no window) — renders offscreen and writes a PNG:

| Flag | Meaning |
|---|---|
| `--shot out.png` | headless capture |
| `--width W --height H` | image size (default 1920×1080) |
| `--cam C` | `chase` `cockpit` `tower` `flyby` `cinematic` |
| `--seconds S` | fly S seconds first (fast-forward; with `--realtime`: wall-clock) |
| `--warp N` | time warp 1–64 for the flight before the shot / the soak |
| `--throttle T`, `--heading DEG`, `--ap` | throttle, initial heading, engage the autopilot |
| `--gear up\|down`, `--flaps N`, `--spoilers F`, `--reverse`, `--fail BITS` | aircraft setup |
| `--eye x,y,z --look x,y,z` | fixed camera |
| `--menu`, `--no-hud`, `--stats` | show the menu, hide the HUD, print the final state |
| `--realtime [--fps HZ]` | stream + render in real time flying straight and level; reports frame times, streaming gaps, camera smoothness and origin shifts (`--fps`: fixed sim step) |
| `--then-reset` | after `--realtime`: press R and time the scenery catch-up at the start |
| `--soak [--shot-every S]` | fly the bot for `--seconds` of sim time as fast as streaming allows; minute log, events, PNGs |
| `--rebase-check` | floating-origin test: render, move the origin (20 km east, then 2 km north), render again, report the image difference |

```sh
# London → Paris (≈340 km) at warp 8, then a picture
skyward --shot paris.png --aircraft lancer --start city --heading 151 --ap --seconds 205 --warp 8
# streaming / frame-time soak over a long distance
skyward --shot out.png --aircraft lancer --start city --heading 150 --throttle 0.85 --realtime --seconds 360 --warp 4
# glider in a 15 kt wind, airliner in a 15 kt crosswind
skyward --shot g.png --aircraft swift --start thermal --bot --soak --seconds 2400 --warp 8 --no-live --metar "EGLC 121150Z 25015KT 9999 SCT045 22/10 Q1015"
skyward --shot m.png --aircraft meridian320 --bot --soak --seconds 4200 --warp 8 --no-live --metar "EGLC 121150Z 18015KT 9999 SCT040 18/08 Q1015"
```
`BOT_TRACE=Final,Flare` prints the controls every frame in those bot phases;
`SKYWARD_REBASE_LOG=1` logs every origin shift; `SKYWARD_GPU_TIMING=1` prints
a per-pass GPU timing breakdown.

## Floating origin

Positions are f32 metres in a local frame (+X east, −Z north, +Y up), an
equirectangular projection around a movable origin. When the aircraft is
20 km from the origin — or 2 km north / south of it, since a new origin
latitude rescales the east–west axis by cos(lat) — the frame is re-centred on
the aircraft (`World::rebase`):

* every map tile is built in **its own frame** (anchored at its north-west
  corner, small precise coordinates) and placed with an **exact** affine map
  `x' = sx·x + tx, z' = z + tz` (both frames are linear in lon / lat), so
  tiles built before and after a shift line up perfectly; the GPU gets that
  map per tile as immediate data, collision queries apply its inverse;
* ground-fixed things (runways, airports, sights, the bot's gates) move by the
  same exact map; the aircraft, cameras, cloud field, wind turbulence and
  thermals by the rigid shift at the aircraft — nothing near it moves;
* procedural ground patterns live in a fixed "pattern space" passed modulo a
  16 384 m period, and every pattern is exactly periodic, so they never jump
  and stay precise anywhere on Earth; cloud and AO temporal history is
  re-expressed in the new frame instead of being thrown away;
* rendering is **eye-relative** in every pass: each tile's placement relative
  to the camera is computed in f64 on the CPU, the aircraft mesh is built
  relative to the eye, and the view, shadow-cascade, reflection and AO
  matrices are rotation-only / eye-relative — no large numbers cancel in
  f32, so nothing shakes however far you fly;
* the sun is computed for the aircraft's own position (in the start's time
  zone), the live weather is re-fetched every 50 km.

A shift costs well under 0.1 ms.

## Layout
| File | What |
|---|---|
| `src/tiles.rs` | TileJSON + tile download, disk cache, background worker pool, airport lookup |
| `src/mvt.rs` | Mapbox Vector Tile decoder |
| `src/osm.rs` | projections / frames, tile math, MVT → clipped tile-frame features |
| `src/scene.rs` | per-tile meshes (ground, buildings, instanced trees), collision, minimap tiles, runways |
| `src/world.rs` | streaming, floating origin, runway merging, environment queries (ground, wind, lift) |
| `src/gfx.rs`, `src/shaders/` | wgpu renderer (per-tile culling and transforms, chunked uploads) |
| `src/atmosphere.rs` | sun position, sky / aerial-perspective LUTs |
| `src/clouds.rs` | volumetric cloud field + renderer, thermals |
| `src/weather.rs` | METAR / Open-Meteo, weather → clouds / wind / precipitation, seasons |
| `src/flight.rs` | flight dynamics + aircraft definitions (unit-tested) |
| `src/models.rs`, `src/models/` | procedural aircraft meshes with animated surfaces |
| `src/sim.rs` | session, controls, autopilot glue, runway choice, cameras (incl. cinematic) |
| `src/bot.rs` | auto-flight bot: glider soaring + powered take-off / tour / landing |
| `src/autopilot.rs` | heading / altitude / V/S / speed autopilot |
| `src/hud.rs` | instruments, moving map, menus (egui) |

See `ROADMAP.md` for what's next.

## Credits

Map data © [OpenMapTiles](https://openmaptiles.org/) ©
[OpenStreetMap contributors](https://www.openstreetmap.org/copyright) (ODbL);
vector tiles served by [OpenFreeMap](https://openfreemap.org/). Weather:
METARs from the [aviationweather.gov](https://aviationweather.gov/) data API
(NOAA), forecasts from [Open-Meteo](https://open-meteo.com/) (CC BY 4.0).
Airport lookup via [Nominatim](https://nominatim.org/).
