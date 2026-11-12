# Skyward — AAA plan

Single source of truth for work items. Goal: **AAA quality** — a sim that looks,
sounds and flies like a modern commercial flight simulator, over the real world.

Legend: `[x]` done · `[~]` in progress (track) · `[ ]` todo · **P0** blocks AAA
feel · **P1** expected of AAA · **P2** polish / delight.
Tracks are parallel agent workstreams (see bottom). Update this file when an
item lands; keep entries one line.

## The bar (acceptance criteria)
- **Look**: screenshots indistinguishable in quality from current commercial
  sims at the same viewpoint, day, dusk, night, rain, fog. No placeholder art.
- **Sound**: blind listeners rate engines/callouts/impacts as "real recording".
- **Flight**: each type within published numbers (stall, Vr, climb, cruise,
  landing roll); type-rated pilots would recognise the handling.
- **Performance**: 1440p ≥ 120 fps on RTX 5090 laptop over London; no frame
  > 33 ms while streaming; browser build ≥ 60 fps on a mid-range GPU.
- **UX**: a new player can pick an airport and be flying in < 30 s; nothing
  looks dev-made; every screen designed.
- **Robustness**: no crash/hang in 8 h soak; offline-tolerant; graceful GPU
  /audio/network failures.

---

## 1. World & scenery
- [x] Endless OSM vector-tile streaming, far-field tiles, floating origin (D, I)
- [x] Buildings with procedural facades, pitched roofs, trees, seasons (E, H)
- [ ] **P0** Terrain elevation (DEM tiles, e.g. Terrarium/Copernicus) + normal/detail shading; runways/airports flattened correctly
- [ ] **P0** Landmark-quality buildings: use OSM `building:part` / `roof:shape` / heights fully; hand-authored or procedural landmarks (Shard, Tower Bridge, London Eye, St Paul's, O2, Canary Wharf tops)
- [ ] **P1** Satellite/aerial imagery option for ground texture (licence-clean source) blended with vector landuse
- [ ] **P1** Airports: terminals, hangars, jet bridges, apron markings, stands, parked aircraft, ground vehicles, windsocks, signage
- [ ] **P1** Bridges with decks/towers, tunnels hidden, rail with trains, motorways with moving traffic (day + night)
- [ ] **P1** Water: coastline/ocean waves, tides on the Thames, boats/ships moving, shoreline foam
- [ ] **P2** Vegetation: species by region, hedgerows as geometry, forest LOD canopies, grass blades near ground
- [ ] **P2** Wildlife/birds, people at airports, construction cranes with lights

## 2. Rendering & VFX
- [x] Hillaire atmosphere, time of day, AgX, cascaded shadows, AO, bloom (E)
- [x] Volumetric clouds + shadows + thermals, live weather (C, H)
- [x] Rain/snow/fog, wet surfaces, sky-shake + precision fixes (H, I)
- [~] **P0** Night: moon/stars/skyglow, street lights, lit windows, traffic, airport lighting, PAPI, landing lights (J)
- [ ] **P0** Supersonic + atmospheric VFX: vapour cones, shock diamonds, heat haze, contrails, wingtip vortices/condensation (L)
- [ ] **P0** Crash VFX: fire, smoke, dust, debris (L)
- [ ] **P1** TAA/upscaling (DLSS-class quality via own TAA + sharpening) with MSAA fallback
- [ ] **P1** Screen-space or ray-traced reflections for glass towers/wet streets
- [ ] **P1** Rain on canopy/windscreen droplets + wipers; icing on airframe
- [ ] **P1** Lightning/thunderstorms (CB clouds), rain shafts, rainbows
- [ ] **P2** Photo mode (free cam, DOF, exposure, filters)
- [ ] **P2** Quality presets (Low → Ultra) and dynamic resolution

## 3. Aircraft
- [~] **P0** Real aircraft from FlightGear (GPL): C172, ASK 21, Spitfire IX, F-16, ATR 72, E190, A320 — textured models, animations, liveries (N)
- [~] **P0** JSBSim-data flight dynamics per type, re-validated tests + bot soaks (N)
- [ ] **P0** Structural loads, wing flex, failures, damage model, survivable vs fatal crashes, physics debris (L)
- [ ] **P1** Full 3D cockpits with working instruments (render gauges into cockpit textures), head look (mouse/TrackIR-style)
- [ ] **P1** Systems depth per type: electrics, fuel (burn + CG), hydraulics, engine start procedures, failures menu
- [ ] **P1** Ground handling: taxiing with tillers, pushback, chocks, wheel friction by surface
- [ ] **P2** v3 types: C-130, C-5, Avro Vulcan; helicopter; quadcopter
- [ ] **P2** Livery selector (house liveries; user-supplied)

## 4. Audio
- [x] Procedural engine/airframe/systems audio, 3D positional + Doppler, callouts (K)
- [~] **P0** Sample-based engines from real recordings, supersonic Mach cone + N-wave boom (K2)
- [~] **P0** Crash/impact sound redesign, event API for debris/fire (K2)
- [ ] **P1** ATC radio (tower/approach phraseology, AI voice) and ATIS from live METAR
- [ ] **P1** Environment: airport ambience, rain on fuselage by surface, wind howl in gear wells
- [ ] **P2** Cockpit switch/knob foley, seat rattles, headset/no-headset mix

## 5. Gameplay & modes
- [x] Auto-fly bot (glider soaring, powered tours, wind-aware) + watch mode (F, I)
- [ ] **P1** Missions/challenges: landing challenges (crosswind, short field), glider task racing, tours with scoring
- [ ] **P1** Flight planning: route on map, waypoints, FMS-lite, ND route
- [ ] **P1** ILS/VOR/GPS approaches + autopilot APPR/NAV/LNAV/VNAV modes
- [ ] **P1** Replay / flight recorder with cinematic cameras
- [ ] **P2** Live real traffic (ADS-B feed) as AI aircraft
- [ ] **P2** Multiplayer (shared sky)
- [ ] **P2** Tutorial / onboarding flights

## 6. UI / UX
- [x] Design system, fonts, redesigned HUD, per-class instruments (PFD/ND, F-16 HUD, analogue) (M)
- [x] Vector moving map + world-map airport picker (OurAirports, search, filters, runway/start picker, live METAR) (M)
- [x] Menus: aircraft cards, conditions, settings, pause, loading, toasts; frame pacing fix (M)
- [ ] **P1** UI follow-ups: ND label overlapping heading scale; airport-diagram backdrop detail (higher-zoom tiles); apron/parking data; UI CPU cost ~4 ms in dense ND
- [ ] **P1** Bot uses displaced thresholds; A320 bot occasional go-around on stable final
- [ ] **P1** Input: gamepad, joystick/HOTAS, rudder pedals, key rebinding, sensitivity/curves
- [ ] **P1** Accessibility: UI scale, colour-blind-safe palettes, subtitles for callouts/ATC
- [ ] **P2** Localisation-ready strings

## 7. Platform, quality & release
- [x] Private repo `WillMatthews/skyward`, checkpoint commits pushed as tracks land
- [ ] **P0** WASM/WebGPU build + GitHub Pages workflow, GPL licence + credits (G) — goes live when the owner makes the repo public (owner's call only)
- [ ] **P1** CI: build + tests + clippy on push; screenshot regression tests (headless `--shot` golden images); audio render regression
- [ ] **P1** Native release packages (Linux AppImage/Flatpak, Windows, macOS)
- [ ] **P1** Performance budgets enforced in CI (GPU timing on soak runs)
- [ ] **P1** Licence/attribution audit for every asset and data source (OSM/OpenMapTiles, FlightGear, OurAirports, fonts, audio)
- [ ] **P1** 8-hour soak (bot) with memory/VRAM leak checks; offline mode test
- [ ] **P2** Settings/save persistence, crash logs, opt-in diagnostics (no tracking by default)
- [ ] **P2** Website/landing page with trailer + screenshots

---

## Tracks
| Track | Scope | Status |
|---|---|---|
| A–I | v1 → v2 foundations (flight, models, clouds, streaming, graphics, bot, weather, floating origin) | ✅ |
| K | Procedural audio | ✅ |
| J | Night & lighting | 🟡 running |
| M | UI/HUD, maps, airport picker, menus, frame pacing | ✅ |
| N | Real FlightGear aircraft + JSBSim FDM | 🟡 running |
| K2 | Sample-based audio, supersonic, crash sounds | 🟡 running |
| L | Structure/flex/crashes + supersonic/atmospheric VFX | ⏳ after N |
| G | WASM + GitHub Pages workflow (repo stays private until owner flips it) | ⏳ last of current batch |
| next | Terrain elevation, landmarks, airports detail, 3D cockpits, input devices, CI | ⏳ to schedule |
