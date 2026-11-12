# Skyward — roadmap

Native Rust flight sim (wgpu / Vulkan, winit, egui) over real-world
OpenStreetMap scenery, streamed endlessly from vector tiles. Default area:
London City Airport (EGLC) + Canary Wharf; any lat/lon works.

Architecture principle: **one data-driven fixed-wing flight model**. Each
aircraft is a parameter set (mass, wing, lift/drag curves, engine, control
authority, gear) plus a procedural mesh. Rotorcraft/multirotors get their own
models later.

## v1 — foundations ✅
- Real OSM scenery (buildings, water, parks, roads, runways), lit GPU scene
  with fog, fixed-wing flight model, crashes, trainer + glider, menu, chase /
  cockpit / tower cameras, instruments, minimap.

## v2 — more fixed wing, a living world ✅ (2026-11-12)
- **Endless streamed world** — OpenFreeMap / OpenMapTiles vector tiles: z14
  detail (facaded buildings, instanced trees, roads, airfields, collision) to
  ~7.5 km, z10 far field to ~30 km, disk cache, worker pool, budgeted GPU
  uploads, per-tile culling, merged runways, moving map, `--airport`.
- **Floating origin** — f32 local frame re-centred on the aircraft (20 km /
  2 km north-south), tiles built in their own frames and placed with exact
  affine maps, eye-relative rendering in every pass, periodic pattern space
  for procedural textures, history kept across shifts; London → Paris and
  beyond stays precise and hitch-free.
- **Renderer** — Hillaire-style atmosphere (LUTs + aerial-perspective
  froxels), volumetric cumulus / stratus / cirrus with temporal
  reprojection and cloud shadows, cascaded shadows, planar water
  reflections, SSAO, MSAA HDR, bloom, AgX tonemapping; seasons, rain, snow,
  wet ground, fog.
- **Live weather, wind and time** — METAR (aviationweather.gov) + Open-Meteo:
  clouds, visibility, precipitation, wind gradient, gusts, turbulence,
  thermals drifting with the wind aloft; real local time and date; re-fetched
  along long flights.
- **Aircraft** — Kestrel trainer, Swift glider, Peregrine warbird
  (taildragger swing, torque, P-factor), Lancer FBW fighter (afterburner,
  supersonic, g / AoA limits), Aerolink turboprop, Meridian 190 / 320 jets;
  multi-engine asymmetry and failures, spoilers, reversers, retractable gear.
- **Autopilot** — heading / altitude / V/S / speed with autothrottle.
- **Auto flight bot** — glider soaring (thermal centring in the drifting
  frame, MacCready with wind, varied tours, final glide, landing into wind)
  and powered take-off → tour → obstacle-checked approach → landing cycles
  for every type, flying tracks with wind correction, landing / departing
  into wind (backtracking), gust additives, de-crab / wing-low crosswind
  landings; cinematic camera; headless soak tests.

## v3 — heavies, rotorcraft and polish
- **Military cargo**: C-130-class (4 turboprops, STOL-ish, short fields) and
  C-5 Galaxy-class (very heavy jet, long runways).
- **Avro Vulcan-class delta bomber**: delta-wing aero (high-alpha lift,
  no stall break, big induced drag), 4 buried jets.
- **Heavy airliner** handling, **ILS approaches** (approach charts per runway).
- **Helicopter**: collective/cyclic/pedals, ground effect, autorotation.
- **Quadcopter**: rate/angle modes, FPV camera.
- Polish carried over from v2:
  - Moon and stars at night.
  - Glass facades reflecting their neighbours (not just the sky).
  - Rain shafts under showers; droplets on the cockpit glass.
  - Field landings (land-outs) for the glider when no runway is in reach.
  - WASM / browser build.
  - Proper taxiway routing; SRTM terrain elevation for hilly areas.

## Ideas parking lot
- Gamepad / joystick, multiplayer, more regions' landmark sets.
