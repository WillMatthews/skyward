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
- [ ] **P1** Airport follow-ups: browser builds layouts / parked-type LODs on the page (one-off hitches; parked types limited to downloaded aircraft); widebody stands only where a hub's stand spacing allows (≥ 62 m; 777, H2a) — OSM stand sizes not read; terminal-piece canopies on multi-part buildings; taxiway centrelines stop at the runway edge (no lead-on curves); vehicles drive the service roads straight through (no junction logic); fire station / fuel farm / tower are procedural where OSM has none; Overpass flaky (1 h back-off, cached forever)
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
- [x] **P0** Negative-g / keyboard loads fix (DMG): held pitch keys now move the stick progressively (slower above V_A) with a pilot's g feel (eases off at 80 % / 60 % of the +/− limit, `input/key_pitch.rs`) — a held key no longer slams full elevator (Citation X +6.7 / −3.0 g → +2.1 / −0.3 g); FBW load-factor protection with integrator unwind (Concorde −1.17 → −0.98 g, F-16 −3.5 → −2.96 g); C172 FDM negative stall (CL table stopped at −5°); wing-bending aero damping + implicit modal step; Spitfire −4 g, Citation X +2.62 g (25.337); damage / crash assists `structural_damage` / `crashes` Realistic · Forgiving · Off (missions Realistic); per-type push-over / limit / ultimate / keyboard / FBW / flex tests (DMG)
- [ ] **P1** Loads follow-ups: Aeromatic FDMs (CRJ700, E190, C-5) have ~2× too much nose-down elevator power vs Cmα (full forward at cruise → α −35°); analog full-forward at 0.9 Vno still separates a wing on 9 transports (beyond V_A: realistic but abrupt); no per-type down-elevator travel limit; FBW airliner law reaches only +2.45 g (protection margin); settings UI for the assists (SET)
- [ ] **P1** Structure follow-ups: body-body debris collisions; fuselage crush/deform; cracks along split seams; fuel model (mass/CG/fire size from actual fuel)
- [x] **P1** Full 3D cockpits with working instruments: FG instrument animations (needles, drums/`textranslate`, `texrotate`/`texmultiple` attitude balls, flags) bound to live sim values with real instrument lag (src/cockpit), glass PFD/ND/EWD/SD/ISIS/MCDU rendered offscreen onto the A320/E190/F-16 screens (mip-mapped, auto-dimmed), night backlighting, auto-dimmed displays; mouse head look (C)
- [ ] **P2** Cockpit follow-ups: hide the 2D glass/gauge overlay in cockpit view by default; clickable knobs; F-16 combiner HUD as a collimated texture; ATR glass EFIS pages
- [x] **P1** Systems depth per type (src/systems, data-driven with category defaults): electrics (battery / gens / APU / RAT / EPU, ammeter), fuel (tanks, selector / crossfeed, burn → mass + lateral CG, leaks, starvation), hydraulics (PTU; gear, flaps, spoilers, brakes, NWS, powered controls), vacuum + pitot-static instruments, engine starts (cold & dark / auto-start / ready), oil, carb ice, fire detection + bottles, anti-ice / airframe icing; 24 failures with flags / red X, master warning / caution, ECAM / EICAS procedures, CRC / fire bell / voice; failures panel (now / armed by time-alt-speed / random window / MTBF), `--fail`, `--cold-dark` (S)
- [ ] **P2** Systems follow-ups: cockpit switch / fire-handle animations, F-16 HUD + ND ignore AHRS flags, engine-fire VFX (flames / smoke trail), captain vs F/O display units (red X shows on both PFDs), settings persistence of cold & dark / random failures, `--fail` in windowed mode
- [ ] **P1** Ground handling: taxiing with tillers, pushback, chocks, wheel friction by surface
- [x] **P2** v3 heavies: C-130H Hercules (FGAddon C130, calibrated to YASim + published), C-5M Super Galaxy (FGMEMBERS C-5, patched JSBSim), Avro Vulcan B.2 (FGUK, JSBSim + delta-wing aero, Olympus howl); neutral liveries, bot / audio / per-type runway minima (H)
- [x] **P2** Heavies polish: re-imported with live cockpits (C-130 82 / Vulcan 63 / C-5 23 live instruments), Vulcan repainted Dark Green / Dark Sea Grey camo (no white blotches), C-130 prop-disc alpha, wing-fog planform from the upper skin (no fog band under high wings), cargo ramp / visor / crew door open when parked, Hercules + Vulcan parked at air bases (HF)
- [ ] **P1** Heavies follow-ups: C-130 bot floats ~850 m before touchdown at EGLC; C-130 / C-5 use the PW100 / CFM56 recordings (no T56 / CF6 set); C-5 cockpit is a crude FG shell with one 3D PFD (no glass mapping, no overhead / engine gauges); C-5 kneels only when parked in the fleet (flight model gear can't kneel); C-5 too big for stands (not in the parked fleet); doors open only after 8 s stopped at idle (no parking brake / door key); Vulcan brake chute; fuel burn (all flown at one weight); Vulcan paint reads glossy (AC material shininess)
- [x] **P2** v3 rotorcraft: Airbus Helicopters EC130 B4 (FGAddon ec130, GPL v2+: model, B4 panel, VEMD) on an own rotor FDM (blade element + momentum, Nr state, FADEC turboshaft, Fenestron, ETL, ground effect, VRS, retreating-blade stall, dynamic rollover, autorotation + flare), stability assist, tail-rotor / hydraulic failures, rotor wash, turbine + blade-slap audio, bot (lift, tour, approach to hover, landing) and autopilot; original 5" FPV quad (motors, mixer + airmode, acro / angle, LiPo sag, prop wash, FPV camera + OSD, gamepad mode 2, orbit bot) (Q)
- [ ] **P1** Rotorcraft follow-ups: helipads (OSM `aeroway=helipad` not yet carried through the tiles — rotorcraft start beside the runway); FPV camera is a rectilinear 120° view with a vignette (no barrel-distortion pass); autorotation descent ≈ 2,000 fpm vs ≈ 1,700 published; cruise needs ~5° of bank against the Fenestron side force (no sideslip trim); cyclic / collective / pedal cockpit animations follow the effective (assisted) inputs; no rotor brake / start sequence on the rotor (cold & dark starts the engine with the rotor at flight idle); VEMD lower page shows derived values (no oil system); quad turtle mode / crash recovery; water landings (floats) not modelled
- [x] **P2** H2a jets: Boeing 777-300ER (FGAddon 777, GPL v2; calibrated to YASim + FCOM; glass PFD / ND / EICAS / MFD / CDUs mapped; widebody stands at hubs), Bombardier CRJ700 (FGMEMBERS 2016 GPL snapshot + FGAddon JSBSim FDM, patched), Cessna Citation X (FGMEMBERS, live Primus 2000 panel, business / GA aprons), Concorde (FGAddon; slender-delta JSBSim aero with vortex lift, Mach-dependent CoP + trim-fuel CG transfer, reheat, droop nose / visor on the flap lever, analogue cockpit live via the new 2D-panel importer); neutral liveries, bot / audio / systems / per-type runway minima (H2a)
- [ ] **P1** H2a follow-ups: 777 interior is heavy (900 k triangles, 20 MB model — decimate the flight-deck shell / overhead); 777 bogie tilt not animated (rests on the aft axle, ~1.2° nose-down); 777 bot's first approach after a tour often joins high (5.5 nm, 3,000 ft) → safe go-around; Citation X cockpit walls are a grey inward copy of the skin (no trim); Concorde: panel text layers (digital readouts) not imported, FBW-only pitch at Mach 2 (the test pilot / autopilot alt-hold is slow through a g law at 600 m/s), engineer's panels on a few placeholder properties, no intake ramps / spill-door model, nose / visor only on the flap lever (no automatic schedule), tail bumper raised to allow the 12-13° rotation (the visual wheel dips at full rotation); CRJ / Citation X fan tones on the E-Jet recordings (no CF34-8 / AE3007 sets)
- [x] **P2** More propeller types (FlightGear GPL): Extra EA-500 (glass IFDs on the PFD / map), PA-18 Super Cub (stands in for the GPL-v3 J-3), Zlin Z-50 LX aerobat (snap rolls, spins, inverted fuel / oil, airshow smoke; stands in for the unlicensed Pitts S-1C), DHC-3 Otter on floats (stands in for the unlicensed DHC-2 Beaver), DC-3 / C-47 (radial sound set); calibrated to published figures; bot, systems, cards, parked GA (H2b)
- [x] **P1** Water operations: float / hull hydrodynamics (`src/water.rs`: buoyancy, planing, hump, step, spray, water rudders, weathervaning, wind sea), water-lane starts from the OSM water, bot water take-off / landing; `water_contact()` API for ditching (H2b)
- [ ] **P1** H2b follow-ups: Extra 500's own JSBSim aero (per-surface model the evaluator can't run) — flown on the calibrated model; Avidyne IFD own pages (PFD / moving map stand in); J-3 Cub / Beaver / Pitts if their licences are clarified; Zlin spins are a fast, steep autorotation (~270°/s at ~16° alpha) rather than a settled 40° spin; the DC-3 model is heavy (≈600 k triangles); no water wake / rendered bow wave (spray particles only); floats on land are skids (no beaching dolly / amphibian wheels); no seaplane-base routing in the airport picker (any airport → nearest water)
- [ ] **P2** Livery selector (house liveries; user-supplied)

## 4. Audio
- [x] Procedural engine/airframe/systems audio, 3D positional + Doppler, callouts (K)
- [x] Sample-based engines from real FlightGear recordings, supersonic Mach cone + N-wave boom (K2)
- [x] Crash/impact sound redesign, event API for debris/fire (K2)
- [ ] **P1** Audio follow-ups: CF34-class recording for E190, engine start sequences once starters exist, remove unused menu_window, human listening pass on levels
- [x] **P1** ATC radio + ATIS (src/atc): real frequencies (OurAirports), UK CAP 413 phraseology from pre-rendered Kokoro voice clips (6 voices) through an airband chain + subtitles; ATIS from the live METAR (letter, runway by wind, TL); clearance / start-up / taxi over the taxiway graph / line-up / take-off / hand-offs; radar vectors to ILS intercepts; landing / vacate; deviations (bust, incursion, no clearance), MAYDAY / PAN / 7700; auto read-backs + assists; comms menu (F4); bot radio in watch mode (R)
- [ ] **P1** ATC follow-ups: some isolated-word clips have Kokoro end artefacts ("knots", "feet", "six" in the US voices) — cut words from carrier sentences with a forced aligner; no AI traffic (no traffic information / sequencing / "number two"); no SIDs / STARs (clearances "via flight planned route", initial altitude by TA); taxi routes need the airport layout loaded (and use the map's hold names); no ground "push and start"; one frequency per unit (no sectorisation); the bot flies its own visual pattern (not the vectors); voices / airport names English only (others are spelled ICAO)
- [ ] **P1** Environment: airport ambience, rain on fuselage by surface, wind howl in gear wells
- [ ] **P2** Cockpit switch/knob foley, seat rattles, headset/no-headset mix

## 5. Gameplay & modes
- [x] Auto-fly bot (glider soaring, powered tours, wind-aware) + watch mode (F, I)
- [x] **P1** Airline-emergency missions (M): 12 data-driven missions (`assets/missions/*.json`; triggers by time / altitude / speed / distance / phase / player action → failures, engine destruction, fuel, decompression, microburst, ATC / crew lines, cabin PA, checkpoints), briefing, objective HUD, debrief with scoring (survival, landing quality, procedures, time), grades / medals, best results per player, retry from checkpoint; ditching survivability, cabin pressure + oxygen masks, split thrust levers (differential thrust), microburst wind, fire tenders after an emergency landing, Kokoro PA lines; 11 of 12 won headlessly by the scripted pilot in tests (M)
- [ ] **P1** Mission follow-ups: `hydraulics4` (C-5, thrust only) not yet won by the scripted pilot (lands too fast / hard); ATC doesn't script its own calls (the crew / ATC lines are subtitles, MAYDAY via F4 is live ATC); no cabin / evacuation visuals (slides, people); tenders are procedural boxes driving straight to the aircraft (no taxiway routing, no foam); pitot mission uses the existing pitot block (ASI as altimeter), not AF447-style low reads; no mission editor / user missions folder; briefing map of the area
- [ ] **P1** Missions/challenges: landing challenges (crosswind, short field), glider task racing, tours with scoring
- [x] **P1** Flight planning: FMS-lite (origin → waypoints → destination + ILS / RNAV approach with IF / FAF, VNAV path), edited in the F3 panel and on the map (click / drag / right-click), ND / map route, distance / ETA / fuel predictions, MCDU F-PLN (F)
- [x] **P1** ILS/VOR/NDB navigation (OurAirports navaids, synthetic ILS per instrument runway), radios with range / horizon / terrain LOS, power and failures, Morse idents; autopilot LOC / VOR / LNAV / APPR (G/S) / VNAV with no-overshoot captures, FD, Airbus FMA, KAP 140 annunciators, airliner autoland; PFD LS diamonds, ND ROSE / ARC / PLAN; Cessna 3D radios / VOR heads live; bot couples to the ILS (F)
- [ ] **P1** Nav follow-ups: no magnetic variation (all courses true); no airways / SIDs / STARs / published procedures (RNAV = ILS geometry); no marker beacons / back courses / GS false lobes; ILS synthetic outside London (frequency clashes possible); MCDU is display-only (editing in the F3 panel); KAP 140 buttons not clickable; ADF needle (KI-227) / E190 / ATR cockpit radios not bound; LOWI 08 still not landed by the airliners (synthetic ILS there is LOC-only by terrain; bot uses its own path)
- [ ] **P1** Replay / flight recorder with cinematic cameras
- [ ] **P2** Live real traffic (ADS-B feed) as AI aircraft
- [x] **P2** Multiplayer (shared sky): dedicated `skyward-server` (headless, any OS; flags + TOML; console list / kick / ban / say / time / weather / motd; logs; live METAR; clean shutdown), own UDP reliability layer (`crates/skyward-net`: acks + ordered reliable channel, clock sync, Hermite snapshot interpolation + dead reckoning, protocol version handshake, cookies, password proof, rate limits), LAN server browser (broadcast + mDNS discovery, pings, favourites / recents, add by address, details, password prompt, clear refusals), other players with real models / lights / engine sound / name tags / map + ND TCAS, chat + quick-chat wheel, spectating, server time + weather sync; web build: disabled with a message (MP)
- [x] **P2** Multiplayer × missions / H2b merge: MP keys moved off the split-throttle F-keys (players F11, name tags Num *, quick chat Backspace / Num 0), Missions disabled while connected, `ON_WATER` state flag for floatplanes (MP)
- [ ] **P1** Multiplayer follow-ups: other players' contrails / smoke / crash fire and debris are not drawn (flags and lost-part bits are sent; lost parts are hidden); ATC does not know about other players (no traffic information / sequencing); no collisions between players; no voice chat; IPv4 discovery only (IPv6 by address works); server-side cheat checks are limited to sanity (states are client-authoritative); no internet listing (by design)
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
- [ ] **P2** Input follow-ups: differential toe brakes need a `Controls` field (per-engine throttles done: `Controls::engine_throttle`, M); help overlay shows default keys not bindings; menu navigation by gamepad; mouse-yoke reticle; verify defaults on real X52/X56/TCA/Windows
- [ ] **P1** Accessibility: UI scale, colour-blind-safe palettes, subtitles for callouts (ATC / ATIS subtitles done, R)
- [ ] **P2** Localisation-ready strings

## 7. Platform, quality & release
- [x] Private repo `WillMatthews/skyward`, checkpoint commits pushed as tracks land
- [x] **P0** WASM/WebGPU build (Web Worker tile builds, fetch, localStorage, lazy aircraft assets, lighter web defaults) + GitHub Pages workflow (deploys once `PAGES_ENABLED`), GPL licence + credits page (G) — goes live when the owner makes the repo public (owner's call only)
- [x] **P1** Web polish (WEB): live on GitHub Pages; loading screen with real progress (streaming compile), friendly no-WebGPU / no-adapter / network / memory errors; content-hashed module, glue and worker; sound samples + ATC voices downloaded on demand (wasm 19.9 → 12.4 MB, 12.7 → 5.4 MB gzipped; first load ≈ 18.8 → 7.9 MB over the wire); live METARs in the browser via VATSIM's CORS-enabled METAR service; pre-rendered aircraft-card thumbnails (`web/thumbs/`); tiles kept in Cache Storage across visits; compact airport / frequency / navaid lists built in CI (`scripts/web-data.mjs`); aircraft textures, samples and voices decoded in the tile workers
- [ ] **P1** Web follow-ups: airport layouts still build on the page (≈ 0.8–1 s hitch behind the main menu: needs a wire codec for `airport::Job` → worker); first-frame pipeline compiles stall ≈ 3 s on Intel iGPUs (as native; wgpu has no async pipeline creation); the Intel Arc iGPU here runs ≈ 6–10 fps at 1600×900 (native: 158 ms/frame on it: scene cost, not the web build) — needs GPU-side LOD work for mid-range targets; VATSIM's METAR terms are unstated (owner: confirm, or swap to a NOAA proxy); untested: Safari 26, Firefox with WebGPU (Windows), Chrome on a discrete GPU with a real display, mobile
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
| F | Navigation: navaids, radios, FMS-lite, ILS / VOR / LNAV / VNAV autopilot, autoland, nav displays (`src/nav/`) | ✅ |
| Q | Rotorcraft: EC130 B4 helicopter + FPV quad (src/rotor, rotor wash, audio, bot) | ✅ |
| R | ATC radio + ATIS: frequencies, phraseology, voices, taxi routing, vectors, comms menu, bot radio (`src/atc/`) | ✅ |
| M (v3) | Airline-emergency missions: framework, 12 missions, briefing / HUD / debrief, scoring, ditching, cabin pressure, split throttles, microburst, fire tenders, PA voices (`src/missions/`) | ✅ |
| UI | Menu scrolling / responsive layouts | ⏳ running |
| MP | Multiplayer: dedicated server, UDP netcode, LAN server browser, remote aircraft, chat, spectate (`crates/`, `src/net/`) | ✅ |
| H2a | Jets: 777-300ER, CRJ700, Citation X, Concorde (+ importer: 2D panels, path-scoped excludes, forced submodels, planar screen UVs, interior shells; JSBSim `aero_ref_pt_shift_x`) | ✅ |
| H2b | Five more propeller types (Extra 500, Super Cub, Zlin Z-50 LX, Otter floats, DC-3) + water operations (`src/water.rs`) | ✅ |
| next | H2 extra aircraft (proposed) | ⏳ awaiting decision |
