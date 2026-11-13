# Skyward

A native Rust flight simulator flown over **real OpenStreetMap scenery** —
anywhere on Earth, for as long as you like. wgpu (Vulkan), winit, egui.

## Features

* **Endless streamed world** — OpenMapTiles vector tiles (OpenFreeMap, no key)
  stream in around the aircraft: detailed z14 tiles (extruded buildings with
  facades and windows, instanced trees, roads, water, parks, airfields with
  merged runways) out to ~7.5 km, generalised z10 tiles to ~30 km, with
  collision and a moving map. Tiles are cached in `cache/tiles/`.
* **Terrain** — real elevation from DEM tiles (AWS Terrain Tiles, Terrarium
  encoding; cached in `cache/terrain/`) under every level of detail: ~24 m
  grids under the detailed tiles, ~380 m under the far tiles and terrain-only
  horizon tiles to ~95 km. The map layers are clipped to the terrain's
  triangles so roads, rivers and land use lie exactly on the hills; buildings
  stand on foundations, lakes are level, runways are flattened to their
  surveyed line and blended into the hills around them; ridges cast shadows;
  rock, alpine meadow and a seasonal snow line on the mountains. The flight
  model, bot (glidepaths above the threshold, terrain clearance), clouds,
  fog, wind and radio altitude all follow the ground.
* **Landmarks** — procedural models of famous buildings from their real
  dimensions replace the OSM extrusions (London: the Shard, Tower Bridge,
  the London Eye with its turning wheel, St Paul's, Big Ben and the Palace of
  Westminster, BT Tower, the Gherkin, the Walkie-Talkie, One Canada Square,
  The O2, Wembley, Battersea Power Station); floodlit at night. A registry
  by location (`src/landmarks/`): more cities are more table rows.
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
* **Ten real aircraft** from FlightGear's open hangar — Cessna 172P,
  ASK 21, Spitfire Mk IX, F-16C, ATR 72-500, Embraer E190, Airbus A320 and
  the heavies: Lockheed C-130H Hercules, C-5M Super Galaxy, Avro Vulcan B.2 — with
  their textured 3D models, cockpits and animations (surfaces, gear, props,
  lights, instruments), and their **JSBSim aerodynamics** evaluated at run
  time (C172, ASK 21, F-16, E190, A320, C-5, Vulcan — the last two with
  Skyward calibration patches, the Vulcan with delta-wing aerodynamics; the
  YASim-based Spitfire, ATR and C-130 use a calibrated model) inside one 6-DoF flight model (ISA atmosphere, engines and
  propellers, gear, spoilers, reversers, asymmetric thrust, FBW laws).
* **Rotorcraft** — an Airbus Helicopters **EC130 B4** (FlightGear model with
  its working B4 panel and VEMD screens) on Skyward's own rotor flight
  model: blade-element / momentum main rotor with collective, cyclic and
  rotor RPM, free-turbine engine with FADEC governor (Nr / N2 / Ng /
  torque / T4 / first-limit indicator), Fenestron and pedals, translational
  lift, ground effect, vortex ring state, retreating-blade stall, dynamic
  rollover on the skids and **autorotation** (engine failure → entry, Nr
  on the collective, flare, cushion); tail-rotor and hydraulic failures;
  rotor wash (dust / spray); a stability assist (attitude hold, SAS, off).
  And an original **5-inch FPV quadcopter**: four motors and props, X
  mixer with airmode, rate (acro) and angle modes with Betaflight-style
  rates, a sagging 6S LiPo, prop wash, a tilted wide-angle FPV camera and
  an OSD.
* **Structure and crashes** — wings bend with the load on each half (first
  bending mode; an A320's tips rise ≈ 1.2 m in 1 g flight, ≈ 2.5 m at 2.5 g),
  limit / ultimate load factors per type, permanent set and flutter, flap and
  gear overspeed; parts tear off with the aerodynamics responding (asymmetric
  lift, lost tail stability). Survivable accidents (gear-up, gear collapse)
  slide to a stop in one piece; fatal ones break up into rigid-body debris
  that tumbles, slides, burns and sinks.
* **Systems and failures** — per type, data-driven: electrics (battery,
  alternator / generators, APU, RAT / EPU, buses, ammeter), fuel (tanks,
  selector / crossfeed, burn per engine feeding mass and lateral CG back into
  the flight model, leaks, starvation), hydraulics (engine / electric / RAT
  pumps, PTU; gear, flaps, spoilers, brakes, nose-wheel steering and powered
  controls depend on them), vacuum and pitot-static systems behind the
  instruments, engine starts (magnetos / primer / starter; APU bleed,
  starter-generator, JFS) from cold and dark or ready to go, oil,
  carburettor icing, fire detection and extinguishers, anti-ice and airframe
  icing. Failures that behave like the real ones — engine fire, flameout,
  partial power, prop overspeed, carb ice, fuel leak / pump, generator
  (the battery runs down, then the avionics die), vacuum (gyros topple),
  pitot / static blockage (the ASI becomes an altimeter), AHRS / display
  failures (red flags, red X), gear that won't extend (alternate extension),
  flap asymmetry / jam, hydraulic loss, control jams, brakes, tyre burst,
  electrical smoke, bird strike — with master warning / caution, ECAM / EICAS
  messages whose procedure lines tick off as you fly them, and the aural
  alerts (CRC, fire bell, voice). Fail anything now or armed for a time,
  altitude, speed or random window, or random failures by MTBF (**O**).
* **Effects** — fireballs, burning fuel with wind-blown smoke columns, dust,
  spray, sparks, tyre and engine smoke; contrails by the Schmidt–Appleman
  criterion (persisting in ice-supersaturated air), vortex and wing-surface
  condensation in humid air, the transonic vapour cone, afterburner shock
  diamonds and heat haze.
* **Autopilot** (heading / altitude / V/S / speed with autothrottle).
* **Auto flight bot** — an AI pilot that flies the real flight model through
  the controls: glider soaring and powered take-off → landmark tour →
  approach → landing cycles, wind-aware (see below).
* **Cameras** — chase, cockpit, tower, fly-by and an auto-cycling cinematic
  camera; headless capture / soak-test modes for testing without a window.
* **Instruments per class** — airliners: glass PFD (speed / altitude tapes,
  VLS and stall bands, speed trend, flight-director bars, FMA), navigation
  display (track-up map arc, wind, route) and engine / warning display; fast
  jet: a conformal head-up display in the cockpit (flight-path marker, pitch
  ladder on the real horizon) and a HUD repeater outside; light aircraft,
  warbird and glider: analogue panels (six-pack + tach; Spitfire-style dials
  in mph with boost; glider ASI in km/h, TE vario with MacCready ring).
  `U` cycles full / minimal / off.
* **Vector moving map** — drawn from the tiles' vector data (water, land use,
  roads by class, rail, buildings when zoomed in, runways, places) with 4×
  MSAA, day / night / navigation-display styles, north- or track-up, range
  rings, trend vector, wind, auto-flight route and airports with collision-free
  labels. `M` cycles corner / large / off; mouse wheel zooms, drag pans.
* **Airports** — ~49 000 airfields and their runways from
  [OurAirports](https://ourairports.com/data/): exact runway ends, headings,
  widths, displaced thresholds, surface and lighting drive the runways in the
  world, spawns and the auto-flight bot (tile runways fill gaps). Every
  airport is detailed from the map data (aprons, taxiways with their refs,
  gates, airside buildings, apron roads), OpenStreetMap via Overpass where it
  answers (parking positions, windsocks, towers) and procedural placement
  where the data is thin: graded airfields, concrete aprons, taxiway centre
  and edge lines, runway-holding positions, stand lead-in lines and numbers;
  glass terminals, hangars with their doors, control towers, a fire station
  and fuel farm; stands with jet bridges and parked aircraft (the sim's own
  models, textured close up, clustered LODs far away), ground vehicles at
  the stands and driving the apron roads, windsocks blowing in the real wind,
  readable lit signs (holding position, location, direction), taxiway
  centre / edge lights, stop bars and floodlit aprons at night
  (`src/airport/`).
* **Menus** — main menu → aircraft (rendered thumbnails, specs) → departure
  (world map, fuzzy search by name / city / ICAO / IATA, suitability filter,
  airport diagram with clickable runway ends, live METAR with head / crosswind
  per runway end, local time and daylight, recents / favourites, random) →
  conditions (live, or time / season / weather preset / METAR) → fly. Settings
  (display, units, sound, controls), pause menu, loading screen. Settings, the
  last flight and favourites persist in `~/.config/skyward/settings.json`.

Default start: the last flight (first run: London City, EGLC).

## Play in the browser

The same code builds for the web (wasm32 + **WebGPU**): menus, all seven
aircraft, streamed scenery, terrain, live weather and sound in a browser tab.

* **Browsers**: WebGPU is required — Chrome / Edge 113+ (Windows, macOS,
  ChromeOS, Android; Linux behind `chrome://flags/#enable-unsafe-webgpu`),
  Safari 26+, Firefox 141+ on Windows. Without it the page says so.
* **Download**: ≈15 MB of wasm (≈9 MB gzipped) before the first flight,
  plus the chosen aircraft (2–15 MB) and the airport list from OurAirports
  (≈17 MB of CSV, ≈4 MB compressed; once per browser cache). Map, terrain and
  weather stream as natively; the browser's HTTP cache stands in for `cache/`.
* **Differences**: no live METARs (aviationweather.gov sends no CORS headers,
  so the weather comes from Open-Meteo alone); settings live in the browser's
  `localStorage`; lighter defaults (Low scenery detail, 2048² shadow cascades,
  the 3D view rendered at ≤ 1.6 Mpixel and scaled up); sound starts after the
  click on Fly (browser autoplay rules); no aircraft-card thumbnails; no
  Quit button. Tiles are built in Web Workers — each runs its own copy of the
  module and posts finished tiles back (`src/tiles_web.rs`); GitHub Pages
  can't send the COOP / COEP headers shared-memory threads need.
* **URL options**: the query string is the command line, e.g.
  `?airport=EGKK&aircraft=meridian320&no-menu`, `?watch`, `?time=12.00`;
  `?scale=0.5` renders the 3D view at half size; `?noworkers` builds tiles
  on the page.

Local build and server (127.0.0.1 only):

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.130 --locked   # = Cargo.lock's wasm-bindgen
cargo install wasm-opt --locked                              # optional (smaller, faster wasm)
scripts/web-serve.sh            # build dist/ and serve http://127.0.0.1:8098/
scripts/web-build.sh            # build dist/ only
```

**GitHub Pages** (`.github/workflows/pages.yml`): every push to `main`
builds `dist/` and uploads it as the Pages artifact; the deploy job runs only
once the repository owner turns Pages on:

1. Settings › Pages › Build and deployment › Source: **GitHub Actions**.
2. Settings › Secrets and variables › Actions › Variables › New repository
   variable: `PAGES_ENABLED` = `true`.
3. Actions › Pages › Run workflow (or push to `main`). The site is
   `https://<owner>.github.io/skyward/`.

(Pages for a private repository needs a paid plan; until the repository is
public and the variable is set, the workflow builds but doesn't deploy.)

## Run

```sh
cargo run --release
cargo run --release -- --lat 51.3308 --lon 0.0325
cargo run --release -- --airport EGKB        # OurAirports ident / ICAO / IATA / name (Nominatim fallback)
cargo run --release -- --airport EGKK --aircraft meridian320 --start-at final:26L:5 --no-menu
cargo run --release -- --watch --aircraft aerolink   # sit back: auto flight + cinematic camera
```

With `--no-menu` (or after Fly) you spawn at the chosen start: a runway end,
a 3 / 5 / 10 NM final, the apron, a thermal (gliders) or anywhere in the air. On hybrid-graphics laptops it
asks wgpu for the high-performance adapter (e.g. the discrete NVIDIA GPU).

### Aircraft
Pick in the aircraft screen. The departure screen filters airports by the
runway length / surface the type needs ("All" shows every field). Without a
chosen runway end, types that need a long runway are moved to the nearest
suitable airfield.

| id (alias) | | |
|---|---|---|
| `kestrel` (`c172`) | **Cessna 172P Skyhawk** | 4-seat high-wing trainer, JSBSim aero. Stall 49 kt clean / 44 kt flaps 30, cruise ~105 kt. |
| `swift` (`ask21`) | **Schleicher ASK 21** | 17 m two-seat sailplane, JSBSim aero. Released near a thermal; airbrakes on F / V. |
| `peregrine` (`spitfire`) | **Supermarine Spitfire Mk IX** | Merlin warbird, taildragger — swings left on take-off (right rudder), retractable gear. |
| `lancer` (`f16`) | **General Dynamics F-16C** | FBW fighter with JSBSim aero, afterburner above 90 % throttle, speed brake, supersonic. |
| `aerolink` (`atr72`) | **ATR 72-500** | Twin turboprop regional airliner, reverse pitch. |
| `meridian190` (`e190`) / `meridian320` (`a320`) | **Embraer E190 / Airbus A320** | Twin-jet airliners with JSBSim aero: spoilers, reversers, slats/flaps, autopilot (A320: normal-law FBW). Need 1,450 / 1,800 m. |
| `hercules` (`c130`) | **Lockheed C-130H Hercules** | Four T56 turboprops, high wing, short and rough fields (from ~900 m, grass OK), reverse pitch. |
| `galaxy` (`c5`) | **Lockheed C-5M Super Galaxy** | 280 t strategic airlifter: slow-spooling CF6s, huge inertia, spoilers and reversers. Needs 2,400 m. |
| `vulcan` | **Avro Vulcan B.2** | Delta-wing V-bomber: no stall break but huge drag at high alpha, T/W 0.6, the Olympus howl. Flap lever = airbrakes. Needs 2,000 m. |
| `ec130` (`heli`, `h130`) | **Airbus Helicopters EC130 B4** | Single-turboshaft light helicopter: collective on the throttle, cyclic on the stick, pedals on the rudder; G or ; cycles the stability assist (attitude hold → off → SAS); K fails the engine (autorotation). Starts beside the runway, rotors turning. |
| `fpv` (`quad`, `drone`) | **5" FPV quadcopter** | Angle (self-levelling) or rate (acro) mode on G / ;, throttle on the throttle; C → cockpit = the FPV camera with OSD. On a gamepad both sticks fly "mode 2" (left: throttle / yaw, right: pitch / roll). |

The models, textures and FDM data come from FlightGear (GPL); sources,
authors and licences per aircraft are in `assets/aircraft/<id>/CREDITS.md`,
and `tools/fgimport/` rebuilds the assets from the FlightGear packages.

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
| O / F2 | **systems & failures** panel: fail / arm / random failures, repair; systems (battery, generators, fuel, engine start, APU, hydraulics, anti-ice, alternate gear, fire handles) |
| ` | master warning / caution reset (silences the chime) |
| J | **auto flight**: the bot flies; any stick / throttle / lever input takes back |
| C | camera: chase → cockpit → tower → fly-by → cinematic |
| Right-drag, wheel | orbit / zoom chase camera; in the cockpit: look around |
| Num 4 / 6 / 8 / 2, Num 5 | glance left / right / up / down (hold), centre the view |
| Y | mouse yoke on / off (enable it in Settings › Controls › Mouse & view) |
| M, `,` / `.` | moving map: corner / large / off, range out / in |
| U | instruments: full / minimal / off |
| I | weather panel: expand / collapse |
| N | mute sound |
| X | time warp 1 / 2 / 4 / 8× |
| P | pause |
| R | restart (back at the start point) |
| Esc / Tab | pause menu: resume, restart, change aircraft / airport, settings |
| H / F1 | help |
| F12 | screenshot |
| | quit from the pause or main menu |

Stick inputs disconnect the autopilot (flashing AP DISCONNECT); throttle keys
only drop the autothrottle. Every key is rebindable in **Settings › Controls**.

### Game controllers
Gamepads, joysticks / HOTAS, throttle quadrants and rudder pedals work out of
the box and can be used together (gilrs: Linux evdev, Windows, macOS, browser
Gamepad API; on Linux button-less pedals are read straight from evdev).
Hot-plug is supported; each device model gets a default layout on first sight —
Xbox / PlayStation pads (left stick flies, triggers = rudder, right stick
looks, bumpers = throttle), sticks (T.16000M, X52, Extreme 3D...: twist =
rudder, slider / Z = throttle, hat = glance), throttles (TCA quadrant, X56:
per-engine levers), pedals (rudder + toe brakes) — saved per device in the
settings file. **Settings › Controls**: bind any axis or button by moving /
pressing it, deadzone, response curve, sensitivity, invert and calibration per
axis with a live response plot, conflict warnings, per-device enable / reset,
mouse yoke and head-look options. Analog axes drive the controls directly
(keys keep their smoothing); levers apply when moved; automation is only
disconnected when an input moves clearly away from where it was when the
autopilot / bot took over. `skyward --controllers` lists devices and prints
their inputs; `SKYWARD_FAKE_INPUT=xbox,x52,t16000m,tca,pedals` adds virtual
devices (tests, screenshots).

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
| `--airport CODE` | start at an airport (OurAirports ident / ICAO / IATA / name; Nominatim fallback) |
| `--start-at SPEC` | `runway:27L`, `final:27L:5` (NM), `apron`, `air:LAT:LON:ALT_M:HDG`, `thermal` |
| `--no-menu` | fly straight away (skip the main menu) |
| `--vsync on\|off` | present with vsync (default on: smooth); off = mailbox / immediate |
| `--hud full\|minimal\|off`, `--map corner\|large\|off` | instrument / map mode |
| `--aircraft ID` | `kestrel` `swift` `peregrine` `lancer` `aerolink` `meridian190` `meridian320` `hercules` `galaxy` `vulcan` `ec130` `fpv` (or `c172` `ask21` `spitfire` `f16` `atr72` `e190` `a320` `c130` `c5` `heli` `quad` `drone`) |
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
| `--mute` | start muted (N toggles; volumes in Settings › Sound) |

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
| `--fail pitot,vacuum,fire2@30` | systems failures by id (`fire` `flameout` `partial` `overspeed` `carbice` `leak` `pump` `gen` `vacuum` `pitot` `static` `ai` `dg` `pfd` `gear` `flapasym` `flapjam` `hyd` `jam` `brakes` `tyre` `smoke` `bird` `icing`; a digit = engine / tank / system, `jam-rudder`, `tyre-right`; `@S` arms it S seconds in) |
| `--cold-dark`, `--panel failures\|armed\|systems` | start cold and dark; show the systems & failures panel |
| `--eye x,y,z --look x,y,z` | fixed camera |
| `--eye-ll lat,lon,h --look-ll lat,lon,h` | fixed camera by geography (heights above the ground) |
| `--menu-screen S` | capture a menu: `main` `aircraft` `airport` `conditions` `settings` `pause` `loading` `credits` |
| `--menu`, `--no-hud`, `--stats` | show the aircraft menu, hide the HUD, print the final state |
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
`SKYWARD_SCENARIO=pullup|overg|bellyland|hardland|crash|transonic|cruise`
flies a scripted structures / crash / effects test (src/scenario.rs; with
`--shot` or `--audio-render`), `SKYWARD_NO_VFX=1` turns the effects off,
`SKYWARD_VFX_STATS=1` prints their counts, `SKYWARD_DEBUG_DEBRIS=1` the wreck;
`BOT_TRACE=Final,Flare` prints the controls every frame in those bot phases;
`SKYWARD_SOAK_LOG=10` logs a soak every 10 s instead of every minute (with height above the ground);
`SKYWARD_REBASE_LOG=1` logs every origin shift; `SKYWARD_NO_AIRPORTS=1` turns
the airport detail layer off, `SKYWARD_NO_OVERPASS=1` builds it from the map
tiles alone (no Overpass requests), `SKYWARD_AIRPORT_DEBUG=1` (or `2`) logs
each airport's layout (holds, stands, windsocks); `SKYWARD_FLAT_TERRAIN=1`
flattens the world (sea level everywhere), `SKYWARD_NO_LANDMARKS=1` uses the
plain OSM buildings, `SKYWARD_TERRAIN_DEBUG=1` logs per-tile terrain grids and
runway elevations; `SKYWARD_GPU_TIMING=1` prints
a per-pass GPU timing breakdown; `SKYWARD_UI_TIMING=1` the interface's CPU
cost and map vertex counts; `SKYWARD_FRAME_LOG=file` (windowed) logs each
frame's raw time and the smoothed simulation step; `SKYWARD_UI_SCRIPT=
"4:aircraft,7:departure,10:airport=EGKK,11:start=final:26L:5,12:fly"` drives
the menus for tests.

### Frame pacing
The window presents with vsync (FIFO) by default and the simulation advances
by the median of recent frame times (hitches pass through; a slow correction
keeps sim time on wall time), with the physics pose interpolated between
120 Hz steps — even motion at any refresh rate. `--vsync off` or Settings ›
Display switches to mailbox / immediate for the lowest latency.

## Tests and CI

`cargo test` runs the unit tests (flight model, audio, input mapping...).
GitHub Actions (`.github/workflows/ci.yml`) runs rustfmt on the formatted paths
(`scripts/fmt-paths.txt`), `clippy -D warnings`, the tests and a release build
on Linux, plus a wasm32 check; `render.yml` is a nightly software-Vulkan
(lavapipe) smoke render; `pages.yml` builds the browser version (and deploys
it to GitHub Pages once enabled, see *Play in the browser*).

Golden regression (needs a GPU and the tile cache, so it runs locally):
`scripts/regress.sh` renders the scenes in `tests/goldens/scenes.txt`
(fixed time / date / METAR, no live data) and compares them with the committed
PNGs perceptually (CIELAB ΔE + SSIM) and the audio renders by spectral distance
(`tools/regress`); diffs and heat-maps land in `target/regress/`. After an
intended visual or sound change: `scripts/regress.sh --update [scene...]`.

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
| `src/terrain.rs` | DEM tiles + cache, per-tile terrain grids (adaptive), drape of the flat layers, skirts / LOD buckets, runway flattening |
| `src/landmarks.rs`, `src/landmarks/` | landmark registry (by location) and the procedural London models |
| `src/world.rs` | streaming, floating origin, runway merging, environment queries (ground, wind, lift) |
| `src/gfx.rs`, `src/shaders/` | wgpu renderer (per-tile culling and transforms, chunked uploads) |
| `src/atmosphere.rs` | sun position, sky / aerial-perspective LUTs |
| `src/clouds.rs` | volumetric cloud field + renderer, thermals |
| `src/weather.rs` | METAR / Open-Meteo, weather → clouds / wind / precipitation, seasons |
| `src/flight.rs` | flight dynamics + aircraft definitions (unit-tested) |
| `src/rotor/` | rotorcraft: helicopter rotor / engine / Fenestron model and stability assist (`heli.rs`), FPV quad motors / flight controller / battery (`quad.rs`), definitions (`defs.rs`), the step inside the airframe (`glue.rs`), bot + autopilot (`bot.rs`), VEMD / OSD / panel (`instruments.rs`), the quad's procedural model (`quad_model.rs`), published-numbers tests (`tests.rs`); rotor wash in `src/vfx/rotorwash.rs`, sound in `src/audio/rotorcraft.rs` |
| `src/systems/` | aircraft systems per type (`defs.rs`), failures + schedule (`failures.rs`), warnings / ECAM / EICAS (`alerts.rs`), session glue; `src/ui/failures.rs` the panel |
| `src/damage.rs` | structural data per type, loads, wing bending mode, damage / failures, aero of lost surfaces |
| `src/debris.rs` | wreckage: rigid-body debris (terrain, buildings, water), crash break-up, fires, sound / effect events |
| `src/vfx/`, `src/shaders/vfx*.wgsl` | particles, trails, vapour, shock diamonds, heat haze; condensation physics (`atmo.rs`) |
| `src/scenario.rs` | scripted structure / crash / effects test flights (`SKYWARD_SCENARIO`) |
| `src/models.rs`, `src/aircraft_asset.rs` | imported aircraft models (`assets/aircraft/<id>/model.skac`): node tree, FlightGear animations → Skyward channels |
| `src/gfx_aircraft.rs`, `src/shaders/aircraft*.wgsl` | textured aircraft renderer (main / reflection / shadow passes) |
| `src/jsbsim.rs` | JSBSim aerodynamics evaluator (functions, tables, axes, stall hysteresis) |
| `src/assets.rs` | asset loading (in-memory store first, then `assets/` on disk) |
| `src/platform/`, `src/tiles_web.rs`, `web/` | browser build: fetch, frame budget, localStorage, lazy assets (`web.rs`), tile wire format (`wire.rs`), Web Worker tile pool, page shell / worker / credits page; `scripts/web-build.sh`, `scripts/web-serve.sh` |
| `tools/fgimport/` | offline converter FlightGear aircraft → Skyward assets (+ neutral liveries) |
| `src/sim.rs` | session, controls, autopilot glue, runway choice, cameras (incl. cinematic) |
| `src/bot.rs` | auto-flight bot: glider soaring + powered take-off / tour / landing |
| `src/autopilot.rs` | heading / altitude / V/S / speed autopilot |
| `src/ui/` | interface: design system (`mod.rs`), instruments (`gauges.rs`, `glass.rs`, `hudsym.rs`), moving map (`mapdata.rs` worker side, `map.rs`, MSAA `canvas.rs`), menus (`menus.rs`, `overview.rs` world map), in-flight UI (`flight.rs`), settings (`controls.rs`: bindings / devices), toasts |
| `src/input/` | game controllers (gilrs + Linux evdev), bindings, default layouts, curves, head-look / analog glue (`sim_glue.rs`) |
| `src/airports.rs` | OurAirports download / cache / search, runways in the world frame, METAR lookup |
| `src/airport/`, `src/shaders/airport*.wgsl` | airport detail: tile-side airside styles + summaries (`tile.rs`), layout (`layout.rs`: markings, stands, signs, lights, vehicles), Overpass client (`overpass.rs`), procedural models + sign font (`mesh.rs`, `font.rs`), parked-aircraft LODs (`fleet.rs`), renderer (`gfx.rs`) |
| `src/startpos.rs` | departure-screen starts (runway end, final, apron, air) |

See `ROADMAP.md` for what's next.

## Credits

Elevation: [Terrain Tiles](https://registry.opendata.aws/terrain-tiles/) on
AWS (Mapzen / Tilezen) — ArcticDEM terrain data DEM(s) were created from
DigitalGlobe, Inc., imagery and funded under National Science Foundation
awards 1043681, 1559691, and 1542736; Australia terrain data © Commonwealth
of Australia (Geoscience Australia) 2017; Austria terrain data © offene Daten
Österreichs – Digitales Geländemodell (DGM) Österreich; Canada terrain data
contains information licensed under the Open Government Licence – Canada;
Europe terrain data produced using Copernicus data and information funded by
the European Union - EU-DEM layers; Global ETOPO1 terrain data U.S. National
Oceanic and Atmospheric Administration; Mexico terrain data source: INEGI,
Continental relief, 2016; New Zealand terrain data Copyright 2011 Crown
copyright (c) Land Information New Zealand and the New Zealand Government
(All rights reserved); Norway terrain data © Kartverket; United Kingdom
terrain data © Environment Agency copyright and/or database right 2015. All
rights reserved; United States 3DEP (formerly NED) and global GMTED2010 and
SRTM terrain data courtesy of the U.S. Geological Survey. Landmark positions
and footprints from OpenStreetMap (ODbL); the models are procedural.

Map data © [OpenMapTiles](https://openmaptiles.org/) ©
[OpenStreetMap contributors](https://www.openstreetmap.org/copyright) (ODbL);
vector tiles served by [OpenFreeMap](https://openfreemap.org/). Weather:
METARs from the [aviationweather.gov](https://aviationweather.gov/) data API
(NOAA), forecasts from [Open-Meteo](https://open-meteo.com/) (CC BY 4.0).
Airports and runways: [OurAirports](https://ourairports.com/data/) (public
domain; cached in `cache/ourairports.tsv.gz`); airport detail from
OpenStreetMap through the [Overpass API](https://overpass-api.de/) (©
OpenStreetMap contributors, ODbL; cached in `cache/airports/`); fallback lookup via
[Nominatim](https://nominatim.org/). Fonts (SIL Open Font License 1.1, see
`assets/fonts/`): [Inter](https://github.com/rsms/inter),
[JetBrains Mono](https://github.com/JetBrains/JetBrainsMono) and
[B612](https://github.com/polarsys/b612) (Airbus / Intactile / ENAC, designed
for cockpit displays).

Aircraft (GPL, from [FlightGear](https://www.flightgear.org/)): the
[c172p team](https://github.com/c172p-team/c172p) (Cessna 172P); FGAddon
ASK 21 (Heiko Schulz, Detlef Faber, Thomas Polzer, Viktor Radnai, Bea Wolf,
Gilberto Agostinho …), Spitfire Mk IX (Detlef Faber), F-16 (Erik Hofman,
Nikolai V. Chr. and the F-16 team), ATR 72-500 (Ryan Miller), A320-family
(Josh Davidson, Jonathan Redpath, merspieler, Matthew Maring, Thorsten
Herrmann, Semir Gebran); [E-jet family](https://github.com/YV3399/E-jet-family-YV)
(Gabriel Hernandez, D-ECHO, Tobias Dammers, Semir Gebran, Stefan Frank,
Narendran Muraleedharan …). Details: `assets/aircraft/<id>/CREDITS.md`.

## Licence

GPL-2.0-or-later — see `LICENSE` (it explains why: the imported FlightGear
aircraft are GPL, the A320 package GPL v2 only).
