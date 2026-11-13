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
- [x] **P0** Terrain elevation (Terrarium DEM, 3 LODs to ~95 km, draped layers, flattened runways, terrain shadows, alpine shading) (T)
- [x] **P0** Procedural landmarks registry: 12 London landmarks incl. turning London Eye, floodlit at night (T)
- [x] **P1** Aprons / taxiways graded like runways: the airfield around each runway is levelled to its plane (by airport class; hills and water left alone) (A)
- [ ] **P1** Terrain follow-ups: detail normals near the ground; per-water-body reflection planes; far-field popping (geomorph); coastal sea in horizon tiles; OSM `building:part` / `roof:shape` via Overpass for non-landmark buildings; more cities' landmarks
- [ ] **P1** Satellite/aerial imagery option for ground texture (licence-clean source) blended with vector landuse
- [x] **P1** Airports, worldwide and data-driven (map tiles + OurAirports + Overpass, procedural fallback): concrete aprons, taxiway centre / edge lines, holding positions, stand lead-ins / numbers / restraint lines; glass terminals, hangars with doors, control towers, fire station, fuel farm; stands with jet bridges and parked aircraft (real assets, textured + clustered LODs); parked and driving ground vehicles; wind-driven windsocks; lit runway-holding / location / direction signs; taxiway / stop-bar / guard lights and floodlit aprons (A)
- [ ] **P1** Airport follow-ups: browser builds layouts / parked-type LODs on the page (one-off hitches; parked types limited to downloaded aircraft); no widebody / bizjet assets (A320 stands in on big stands); terminal-piece canopies on multi-part buildings; taxiway centrelines stop at the runway edge (no lead-on curves); vehicles drive the service roads straight through (no junction logic); fire station / fuel farm / tower are procedural where OSM has none; Overpass flaky (1 h back-off, cached forever)
- [ ] **P1** Bridges with decks/towers, tunnels hidden, rail with trains, motorways with moving traffic (day + night)
- [ ] **P1** Water: coastline/ocean waves, tides on the Thames, boats/ships moving, shoreline foam
- [ ] **P2** Vegetation: species by region, hedgerows as geometry, forest LOD canopies, grass blades near ground
- [ ] **P2** Wildlife/birds, people at airports, construction cranes with lights

## 2. Rendering & VFX
- [x] Hillaire atmosphere, time of day, AgX, cascaded shadows, AO, bloom (E)
- [x] Volumetric clouds + shadows + thermals, live weather (C, H)
- [x] Rain/snow/fog, wet surfaces, sky-shake + precision fixes (H, I)
- [x] Night: moon/stars/skyglow, street lights, lit windows, traffic, airport lighting, PAPI, landing lights (J)
- [ ] **P2** Night follow-ups: light pools beyond 4 km, halos behind clouds, README/help for L key
- [x] Supersonic + atmospheric VFX: vapour cone, shock diamonds, heat haze, Schmidt–Appleman contrails, vortex / wing condensation (L)
- [x] Crash VFX: fireball, fuel fire + smoke column, dust, spray, sparks, tyre / engine smoke; GPU particles lit by sun/moon/fires (L)
- [ ] **P1** VFX follow-ups: vapour cone ray-march costs ~1.2 ms at 1440p when it fills the screen (half-res option); wing-surface fog low-contrast on white airliners; smoke at night only lit by fire ambient (no volumetric glow); fire streaks on still-sliding wrecks; particles drawn over clouds (no cloud depth); haze strength untuned for props
- [ ] **P1** TAA/upscaling (DLSS-class quality via own TAA + sharpening) with MSAA fallback
- [ ] **P1** Screen-space or ray-traced reflections for glass towers/wet streets
- [ ] **P1** Rain on canopy/windscreen droplets + wipers; icing on airframe
- [ ] **P1** Lightning/thunderstorms (CB clouds), rain shafts, rainbows
- [ ] **P2** Photo mode (free cam, DOF, exposure, filters)
- [ ] **P2** Quality presets (Low → Ultra) and dynamic resolution

## 3. Aircraft
- [x] Real aircraft from FlightGear (GPL): C172, ASK 21, Spitfire IX, F-16, ATR 72, E190, A320 — textured models, cockpits, animations, neutral liveries (N)
- [x] JSBSim aero evaluator (C172, ASK21, F-16, E190, A320); Spitfire/ATR calibrated to YASim data (N)
- [ ] **P1** FDM follow-ups: A320 engine-out rudder authority, A320 Mmo overshoot, E190 clean stall high, ASK21 L/D 42 vs 34, F-16 climb low; Spitfire prop-disc dome; thumbnails load ~4 s
- [ ] **P1** Decide ATR 72 asset licence (FGAddon policy-only GPL notice)
- [x] Structural loads, wing flex (modal), limit/ultimate damage, flutter, flap/gear overspeed, separations with asymmetric aero, survivable vs fatal crashes, rigid-body debris (L)
- [ ] **P1** Structure follow-ups: body-body debris collisions; fuselage crush/deform; cracks along split seams; fuel model (mass/CG/fire size from actual fuel)
- [x] **P1** Full 3D cockpits with working instruments: FG instrument animations (needles, drums/`textranslate`, `texrotate`/`texmultiple` attitude balls, flags) bound to live sim values with real instrument lag (src/cockpit), glass PFD/ND/EWD/SD/ISIS/MCDU rendered offscreen onto the A320/E190/F-16 screens (mip-mapped, auto-dimmed), night backlighting, auto-dimmed displays; mouse head look (C)
- [ ] **P2** Cockpit follow-ups: hide the 2D glass/gauge overlay in cockpit view by default; radio/transponder frequency drums (no nav radios yet); clickable knobs; F-16 combiner HUD as a collimated texture; ATR glass EFIS pages
- [x] **P1** Systems depth per type (src/systems, data-driven with category defaults): electrics (battery / gens / APU / RAT / EPU, ammeter), fuel (tanks, selector / crossfeed, burn → mass + lateral CG, leaks, starvation), hydraulics (PTU; gear, flaps, spoilers, brakes, NWS, powered controls), vacuum + pitot-static instruments, engine starts (cold & dark / auto-start / ready), oil, carb ice, fire detection + bottles, anti-ice / airframe icing; 24 failures with flags / red X, master warning / caution, ECAM / EICAS procedures, CRC / fire bell / voice; failures panel (now / armed by time-alt-speed / random window / MTBF), `--fail`, `--cold-dark` (S)
- [ ] **P2** Systems follow-ups: per-engine throttles (single lever today, so "THR LEVER 1 IDLE" means all), cockpit switch / fire-handle animations, F-16 HUD + ND ignore AHRS flags, engine-fire VFX (flames / smoke trail), captain vs F/O display units (red X shows on both PFDs), settings persistence of cold & dark / random failures, `--fail` in windowed mode
- [ ] **P1** Ground handling: taxiing with tillers, pushback, chocks, wheel friction by surface
- [x] **P2** v3 heavies: C-130H Hercules (FGAddon C130, calibrated to YASim + published), C-5M Super Galaxy (FGMEMBERS C-5, patched JSBSim), Avro Vulcan B.2 (FGUK, JSBSim + delta-wing aero, Olympus howl); neutral liveries, bot / audio / per-type runway minima (H)
- [x] **P2** Heavies polish: re-imported with live cockpits (C-130 82 / Vulcan 63 / C-5 23 live instruments), Vulcan repainted Dark Green / Dark Sea Grey camo (no white blotches), C-130 prop-disc alpha, wing-fog planform from the upper skin (no fog band under high wings), cargo ramp / visor / crew door open when parked, Hercules + Vulcan parked at air bases (HF)
- [ ] **P1** Heavies follow-ups: C-130 bot floats ~850 m before touchdown at EGLC; C-130 / C-5 use the PW100 / CFM56 recordings (no T56 / CF6 set); C-5 cockpit is a crude FG shell with one 3D PFD (no glass mapping, no overhead / engine gauges); C-5 kneels only when parked in the fleet (flight model gear can't kneel); C-5 too big for stands (not in the parked fleet); doors open only after 8 s stopped at idle (no parking brake / door key); Vulcan brake chute; fuel burn (all flown at one weight); Vulcan paint reads glossy (AC material shininess)
- [ ] **P2** v3 types: helicopter; quadcopter
- [ ] **P2** Livery selector (house liveries; user-supplied)

## 4. Audio
- [x] Procedural engine/airframe/systems audio, 3D positional + Doppler, callouts (K)
- [x] Sample-based engines from real FlightGear recordings, supersonic Mach cone + N-wave boom (K2)
- [x] Crash/impact sound redesign, event API for debris/fire (K2)
- [ ] **P1** Audio follow-ups: CF34-class recording for E190, engine start sequences once starters exist, remove unused menu_window, human listening pass on levels
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
- [x] **P1** QA: goldens re-blessed (+ terrain_alps, landmarks_city; flight scenes VFX-free), bot terrain fixes (approach/departure terrain margins, route MSA, terrain-aware gate + hold, go-around climb-out, low-pass obstacle floor), spawn fixes; soak clean at EGLC/EGKB/LOWI except below (QA)
- [ ] **P1** Bot in mountain terrain: airliners at LOWI land on 26 but not 08 (straight-in from the hold is too high → safe go-arounds); ATR/E190 can climb into rising side valleys with a westerly (no terrain-aware departure / climb-in-hold)
- [x] **P1** Input: gamepad, joystick/HOTAS, rudder pedals, quadrants (gilrs + evdev, hot-plug), rebinding UI, curves/deadzone/calibration, mouse yoke, head-look (P)
- [ ] **P2** Input follow-ups: differential toe brakes + per-engine throttles need `Controls` fields; help overlay shows default keys not bindings; menu navigation by gamepad; mouse-yoke reticle; verify defaults on real X52/X56/TCA/Windows
- [ ] **P1** Accessibility: UI scale, colour-blind-safe palettes, subtitles for callouts/ATC
- [ ] **P2** Localisation-ready strings

## 7. Platform, quality & release
- [x] Private repo `WillMatthews/skyward`, checkpoint commits pushed as tracks land
- [x] **P0** WASM/WebGPU build (Web Worker tile builds, fetch, localStorage, lazy aircraft assets, lighter web defaults) + GitHub Pages workflow (deploys once `PAGES_ENABLED`), GPL licence + credits page (G) — goes live when the owner makes the repo public (owner's call only)
- [ ] **P1** Web follow-ups: no live METAR (aviationweather.gov has no CORS) — a CORS-friendly METAR source; aircraft-card thumbnails; Cache API for tiles beyond the HTTP cache; ship the compact airport list from CI; audio decode / aircraft texture decode still on the page's thread (one-off stalls at flight start); test Safari / Firefox / mid-range GPUs
- [x] **P1** CI: fmt (formatted paths) + clippy -D warnings + tests + release build + wasm32 check on push; lavapipe smoke render nightly; local golden image/audio regression `scripts/regress.sh` (P)
- [ ] **P2** CI follow-ups: one-shot `cargo fmt` of the tree when no track is active → `cargo fmt --check`; burn down the crate-wide clippy allows in Cargo.toml
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
| J | Night & lighting | ✅ |
| M | UI/HUD, maps, airport picker, menus, frame pacing | ✅ |
| N | Real FlightGear aircraft + JSBSim FDM | ✅ |
| K2 | Sample-based audio, supersonic, crash sounds | ✅ |
| G | WASM + GitHub Pages workflow (repo stays private until owner flips it) | ✅ |
| P | Input devices (gilrs/evdev, rebinding, curves, head-look) + CI/goldens | ✅ |
| T | Terrain elevation + landmarks | ✅ |
| L | Structure/flex/crashes + supersonic/atmospheric VFX (damage, debris, vfx) | ✅ |
| A (v3) | Airport detail: markings, buildings, stands, jet bridges, parked aircraft, vehicles, signs, windsocks, airfield lights, airfield grading (`src/airport/`) | ✅ |
| C (v3) | Live 3D cockpit instruments (steam gauges + glass displays) | ✅ |
| H | v3 heavies: C-130H, C-5M, Vulcan B.2 | ✅ |
| HF | Heavies polish: live cockpits, liveries, doors, air-base fleet | ✅ |
| S | Aircraft systems + realistic failures (systems/, failures panel, ECAM / EICAS, aural) | ✅ |
| next | ILS/FMS (F), rotorcraft (Q), ATC/ATIS (R) | ⏳ queued |
