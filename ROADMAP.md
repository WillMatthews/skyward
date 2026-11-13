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
- ✅ **Real aircraft** — the seven types are now FlightGear aircraft (textured
  models with cockpits and animations, JSBSim aerodynamics for C172 / ASK 21 /
  F-16 / E190 / A320), GPL-2.0-or-later; the heavies below (C130 FGAddon,
  FGMEMBERS C-5 and AvroVulcanB2) are GPL v2.
- ✅ **Military cargo** (2026-11-13): Lockheed C-130H Hercules (4 × T56
  turboprops, high wing, short / rough fields from ~900 m) and C-5M Super
  Galaxy (280 t, 4 × CF6, slow spool, huge inertia, 2,400 m runways).
- ✅ **Avro Vulcan B.2**: delta-wing aero (lift to ~28° alpha with no stall
  break, big induced / high-alpha drag), 4 buried Olympus jets, the howl;
  airbrakes on the flap lever.
- Runway minima are per type now (ATR 1,100 m, E190 1,450 m, C-130 900 m on
  grass too, C-5 2,400 m, Vulcan 2,000 m).
- ✅ **More jets** (2026-11-13, Track H2a): **Boeing 777-300ER** (513 kN
  GE90s, FBW, glass cockpit, parked on the hubs' widebody stands),
  **Bombardier CRJ700** (rear engines, T-tail), **Cessna Citation X**
  (Mach 0.92 bizjet, parked on business aprons) and **Concorde** — Mach 2 at
  FL550-600, reheat, slender-delta vortex lift with no stall break, the
  centre of pressure moving aft with Mach balanced by trim-fuel transfer,
  droop nose and visor, and her analogue cockpit live (FlightGear 2D panels
  imported into the 3D cockpit).
- **Heavy airliner** handling, **ILS approaches** (approach charts per runway).
- ✅ **Helicopter** (2026-11-13): Airbus Helicopters EC130 B4 — FGAddon
  model (GPL v2+) with its B4 panel and VEMD, on Skyward's own rotor
  model: collective / cyclic / pedals, rotor RPM and FADEC turboshaft,
  translational lift, ground effect, vortex ring, retreating-blade stall,
  dynamic rollover, **autorotation** to a flared landing; tail-rotor and
  hydraulic failures; attitude-hold / SAS assist for keyboard pilots;
  rotor wash; blade slap. Within ~2–5 % of the published hover ceilings
  and fast cruise.
- ✅ **Quadcopter**: an original 5" FPV quad — acro / angle modes,
  Betaflight-style rates, airmode mixer, 6S battery sag, prop wash, tilted
  wide FPV camera with OSD, gamepad mode 2, a scenic-orbit bot.
- ✅ **More propeller types** (H2b): Extra EA-500 turboprop single, Piper
  PA-18 Super Cub, Zlin Z-50 LX aerobat (snap rolls, spins, inverted
  flight, smoke), DHC-3 Otter on floats and the DC-3 / C-47 — FlightGear
  GPL models, flight models calibrated to published figures.
- ✅ **Water operations**: float hydrodynamics (buoyancy, the hump, planing
  on the step, spray, water rudders, weathervaning, wind-driven chop),
  floatplane starts and bot circuits on rivers / docks / lakes; the same
  hull model is the ditching API for the missions track.
- Polish carried over from v2:
  - Moon and stars at night.
  - Glass facades reflecting their neighbours (not just the sky).
  - Rain shafts under showers; droplets on the cockpit glass.
  - Field landings (land-outs) for the glider when no runway is in reach.
  - WASM / browser build.
  - Proper taxiway routing; SRTM terrain elevation for hilly areas.

## Ideas parking lot
- Gamepad / joystick, multiplayer, more regions' landmark sets.
