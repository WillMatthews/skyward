# Audio assets

Skyward's sound is a mix of **recorded samples** from the FlightGear
project (engines, start-ups, shutdowns, rolling, wind, thunder, impacts) and
**our own synthesis** (`src/audio/`): propeller blade-pass and buzz-saw
tones, piston firing detail, afterburner crackle, the positional /
Doppler / Mach-cone mixing, sonic-boom N-waves, buffet, rain, birds, chimes,
warnings, fires, scrapes and everything else. The cockpit voice callouts are
our own (Kokoro TTS, below).

## Licence summary

All recorded engine / effect samples come from FlightGear aircraft and data
released under the **GNU GPL**:

| Source | Licence | Notes |
|---|---|---|
| FGAddon `Aircraft/f16` | GPL-2.0-or-later (`copyright.txt`: "version 2 of the License, or (at your option) any later version", applies to all files in the package) | Sounds by Carsten Brueggmann (GEED), Timi "MrChillStorm", FGUK — credited in the package `authors.txt` |
| FGAddon `Aircraft/spitfireIX` | GPL-2.0 (`README`: "released under the terms of the GPLv2", package `COPYING` = GPL-2) | Detlef Faber |
| FGAddon `Aircraft/DHC-8` | GPL-2.0-or-later (`README.md` licence section) | `out.wav`, `intern.wav`, `reverse.wav` recorded by Andreas Spiekermann (README "SOUNDS SOURCES") |
| FGAddon `Aircraft/A320-family` | **GPL-2.0 only** (`README.copyright.md`: "version 2 of the GNU General Public License") | SASA CFM56-5B / IAE V2500 / airframe recordings |
| FGAddon `Aircraft/ASK21` | GPL-2.0-or-later (`README.md`) | |
| c172p-team/c172p (GitHub) | GPL-2.0 (repository `LICENSE`) | Sounds by Gilberto Agostinho and the c172p team |
| fgdata (`Sounds/`, GitLab `flightgear/fgdata`) | GPL-2.0 (`LICENSE.md`) | FlightGear base package |
| FGMEMBERS/AvroVulcanB2 (GitHub, FGUK) | GPL-2.0 (repository `COPYING`; no "or later" statement) | Sounds credited to "Algernon" in `AvroVulcanb2-set.xml`; the recordings' original source is not documented in the package (see below) |

**Compatibility note:** the A320-family files are GPL-2.0 *only* (and the
Spitfire IX package is stated as GPLv2). Shipping them is fine for a project
licensed GPL-2.0 or GPL-2.0-or-later; if Skyward were to be released under
GPL-3.0-only, the `cfm_*`, `v25_*`, `heavy_*`, `gear_heavy`,
`flap_motor_heavy`, `wind_airliner` (A320-family) and `merlin_*`
(Spitfire IX) samples would have to be replaced or dropped (the engines then
fall back to the procedural voices automatically: remove the set in
`src/audio/soundsets.rs`). Excluded on purpose: the E-jet package's
`cabin-chime.wav` (CC-BY-3.0) and anything without a clear licence.

Fetched 2026-11-12: FGAddon SVN trunk r21799, c172p `master` 63ba85273b7f,
fgdata `next` 94c38602.

## Files (`samples/*.ogg`)

Built by `tools/build_samples.py` (mono mix-down, trim, polyphase resample,
correlation-matched seamless loops, Ogg Vorbis q4; `samples/index.tsv` holds
rate / length / loop flag / build attenuation). 95 files, ~4.1 MB.

| Sample(s) | Source file(s) | Package |
|---|---|---|
| `c172_idle`, `c172_run`, `c172_crank`, `c172_start`, `c172_cut` | `Sounds/engine-idle.wav`, `engine.wav`, `engine-crank.wav`, `engine-start.wav`, `engine-cut.wav` | c172p |
| `c172_stall` | `Sounds/stall2.wav` | c172p |
| `tyre_roll`, `tyre_gravel`, `flap_motor_light` | `Sounds/tires-rolling-asphalt.wav`, `tires-rolling-gravel.wav`, `flaps-motor.wav` | c172p |
| `crash_c172`, `crash_heavy`, `crash_light`, `crash_scrape` | `Sounds/crash.wav`, `damage-heavy.wav`, `damage.wav`, `scratch.wav` | c172p |
| `merlin_1`, `merlin_2`, `merlin_3` | `Sounds/merlin_rpm1.wav`, `spita.wav`, `merlin_rpm4.wav` | spitfireIX |
| `merlin_start`, `merlin_stop` | `Sounds/merlin_starter.wav`, `merlin_shutdown.wav` | spitfireIX |
| `f16_x_idle_f`, `f16_x_idle_a`, `f16_x_mil_f`, `f16_x_mil_a`, `f16_x_ab` | `Sounds/GEED/exterior_ground_idle_loop_mono.wav`, `exterior_ground_idle_aft_loop_mono.wav`, `mil_exterior.wav`, `full_mil_aft_mono.wav`, `afterburner_exterior.wav` | f16 (GEED) |
| `f16_x_abx`, `f16_x_roar` | `Sounds/exhaust_loop.wav`, `Sounds/engine/roar-medium.wav` | f16 |
| `f16_x_whine` | `Sounds/FGUK/f16-whine.wav` | f16 (FGUK) |
| `f16_i_idle_lo/hi`, `f16_i_88_lo/hi`, `f16_i_100_lo/hi`, `f16_i_ab` | `Sounds/Timi/F-16_In_Idle_Low/High`, `_N2_88_Low/High`, `_N100_Low/High`, `_Afterburner` (`-left` + `-right` summed) | f16 (Timi) |
| `fighter_touch`, `gear_fighter` | `Sounds/touchdown.wav`, `gear-transit.wav` | f16 |
| `crash_f16`, `crash_explode`, `crash_water` | `Sounds/Effects/aircraft-crash.wav`, `aircraft-explode.wav`, `aircraft-water-crash.wav` | f16 |
| `atr_out`, `atr_int`, `atr_rev` | `Sounds/out.wav`, `intern.wav`, `reverse.wav` | DHC-8 (A. Spiekermann) |
| `atr_prop`, `atr_start`, `atr_stop` | `Sounds/turboprop1.wav`, `turbine_start.wav`, `turbine_shutdown.wav` | fgdata |
| `vul_x_turb`, `vul_x_jet`, `vul_x_blast`, `vul_x_howl`, `vul_i_eng` | `Sounds/vulcan-engine-exterior.wav`, `jet.wav`, `vulcan-jetblast.wav` (first 14 s), `vulcan-howel.wav`, `vulcan-engine-inside.wav` | AvroVulcanB2 (FGUK) |
| `vul_start`, `vul_stop` | `Sounds/vulcan-start.wav` (first 24 s), `vulcan-shutdown.wav` (first 16 s) | AvroVulcanB2 (FGUK) |
| `cfm_x_*` (idle, idle_r, idle_c, idle_d, spool, lowspool, spool_c, spool_cr, rumble, rumble_d, buzz, buzz_d, rev, start, stop) | `Sounds/SASA/CFM56B/external/cfm-idle`, `-idleRear`, `-idleClose`, `-idleDist`, `-spool`, `-lowspool`, `-spoolClose`, `-spoolCloseRear`, `-rumble`, `-rumbleDist`, `-buzz`, `-buzzDist`, `-reverse`, `-startup`, `-shutdown` | A320-family (SASA) |
| `cfm_c_*` (idle, idle2, spool, hum, comb, comb2, buzz, rev, start, stop) | `Sounds/SASA/CFM56B/cockpit/cfm-idle`, `-idle2`, `-lowspool`, `-hum`, `-comb`, `-comb-full`, `-buzz`, `-reverse`; `.../start/A320_cockpit_startup`, `A320_cockpit_shutdown` | A320-family (SASA) |
| `v25_x_*`, `v25_c_*` | `Sounds/SASA/V2500/external/v2500-idle`, `-idleRear`, `-spool`, `-lowspool`, `-buzz`, `-buzzDist`, `-startup`; `.../cockpit/v2500-idle`, `-lowspool`, `-comb`, `-buzz`, `-reverse`; `.../start/v2500-start2` | A320-family (SASA) |
| `heavy_roll`, `heavy_roll_int`, `heavy_touch`, `gear_heavy`, `flap_motor_heavy`, `wind_airliner` | `Sounds/SASA/Roll.wav`, `Cockpit_Roll.wav`, `Touch-Main.wav`, `A320_gear.wav`, `A320_flapmotor.wav`, `wind_cockpit.wav` | A320-family (SASA) |
| `glider_wind`, `glider_brake`, `glider_roll` | `Sounds/whoosh-loop.wav`, `airbrakesL.wav`, `ground_roll3.wav` | ASK21 |
| `wind_light`, `thunder_1..3` | `Sounds/wind1.wav`, `thunder1.wav`, `thunder2.wav`, `thunder3.wav` | fgdata |

How they are played (which FlightGear `*-sound.xml` tables were translated,
reference RPMs, cross-fades) is documented in `src/audio/soundsets.rs`.

## Cockpit voice callouts

| Files | What | Source | Licence |
|---|---|---|---|
| `voice/*.wav` (26 clips, 16 kHz mono 16-bit, ~850 kB) | Radio-altimeter / GPWS callouts: "twenty five hundred", "one thousand", "five hundred" … "ten", "retard", "minimums", "hundred above", "approaching minimums", "stall", "pull up", "sink rate", "too low gear", "too low flaps", "terrain", "bank angle", "vee one", "overspeed", "engine failure" | Generated offline for this project with [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M) (ONNX build, voice `bm_fable`) running locally, then trimmed, normalised and resampled to 16 kHz. Script: see below. | Kokoro-82M model weights are Apache-2.0; the generated audio is original output made for Skyward and is released with the project (treat as CC0). |

The voice is heard through a simulated cockpit loudspeaker (band-pass,
presence peak, light drive) in `src/audio/voice.rs`.

## ATC radio voices (`atc/`, Track R)

| Files | What | Source | Licence |
|---|---|---|---|
| `atc/<voice>.ogg` + `atc/<voice>.idx` (6 voices, 12 kHz mono Ogg Vorbis q1, ≈ 2.8 MB) | The ATC / ATIS / pilot phrase vocabulary of `atc/vocab.tsv`: digits, the phonetic alphabet, units, airport and unit names, stock phrases ("cleared for take-off", "line up and wait", "taxi to holding point", "report established" …), packed one stream per voice with an index of clip offsets | Generated offline for this project with [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M) (ONNX build) running locally — voices `am_liam` (the pilot), `bf_emma` (ATIS / delivery), `am_onyx` (ground), `bm_fable` (tower), `af_river` (approach / director / radar), `af_aoede` (second ATIS / units) — then time-compressed (WSOLA, pitch kept), trimmed, loudness-matched and resampled. Script: `tools/gen_atc.py`. | Kokoro-82M model weights are Apache-2.0; the generated audio is original output made for Skyward and is released with the project (treat as CC0). |

Transmissions are assembled from these clips at run time (`src/atc/phrase.rs`)
and played through an AM airband chain — squelch break and tail, carrier
hiss by signal strength, 300–3,000 Hz voice band, compression and drive —
in `src/audio/radio.rs`.

## Regenerating

* Samples: fetch the source files listed above (only the `Sounds/` folders)
  into one directory with the layout `c172/…`, `svn/<Aircraft>/…`, `fg/…`,
  then `python3 tools/build_samples.py <dir> samples` (needs numpy, scipy,
  soundfile, ffmpeg with libvorbis).
* Callouts: `tools/gen_callouts.py` (Kokoro → 24 kHz WAVs) and
  `tools/to16k.py` (→ 16 kHz). With a local Kokoro ONNX setup (`engine.py`
  exposing `Engine().synthesise(text, voice, speed, lang)`), synthesise each
  phrase at speed 1.1–1.4, trim leading/trailing silence (threshold 1 % of
  peak, keep 10 ms before / 50 ms after), normalise to -1 dBFS, apply 5 ms
  fades and write 24 kHz PCM-16; then resample to 16 kHz with a gentle
  roll-off above 6.5 kHz. File names must match `src/audio/voice.rs` (e.g.
  `2500.wav`, `app_minimums.wav`, `too_low_gear.wav`).
* ATC voices: `python tools/gen_atc.py ../atc [cache dir]` (run from
  `assets/audio/tools`, Kokoro ONNX as above, ffmpeg with libvorbis). Edit
  `atc/vocab.tsv` (key, text to speak, voices P/C/A, speed factor) and re-run:
  rendered clips are cached, so only new phrases are synthesised. The test
  `atc::tests::voice_packs_match_the_vocabulary` checks the packs match.

## Avro Vulcan B.2 samples (`vul_*`, Track H)

From github.com/FGMEMBERS/AvroVulcanB2 (`master`, fetched 2026-11-13),
licensed as a whole under GPL-2.0 (repository `COPYING`). The set file
credits the sounds to "Algernon" (FGUK); the package does not say where
the recordings were made, so their provenance rests on the package licence
alone — the same standard as the other FGAddon / FGMEMBERS assets, but
flagged here in case a stricter audit is wanted (the Vulcan then falls back
to the CFM56 set by removing `SoundSet::Olympus` in `src/audio/capture.rs`).
Built by `tools/build_vulcan.py` (numpy + ffmpeg; same processing as
`build_samples.py`, whose MANIFEST also lists them). The C-130 uses the
DHC-8 (PW100) turboprop set and the C-5 the CFM56-5B set as the nearest
recorded engines; their own packages' sounds were not imported.

The H2a jets (Track H2a) reuse recorded sets, no new samples: the 777-300ER
the CFM56-5B set (with the GE90's 22-blade fan tone), the CRJ700 and the
Citation X the CF34-class set of the E-Jet, and Concorde the Vulcan's
Olympus set above (the Olympus 593 is the 301's descendant; reheat and the
supersonic boom are synthesised).

## Rotorcraft (Track Q)

The EC130 B4 and the FPV quad are fully procedural (`src/audio/rotorcraft.rs`:
main-rotor blade passage and blade slap, Arriel turbine and exhaust, main
gearbox whine, the Fenestron's unevenly spaced blades; four motor / prop
voices with their beating, ESC whine and prop wash). No recordings were
imported for them.
