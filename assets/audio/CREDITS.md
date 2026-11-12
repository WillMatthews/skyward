# Audio assets

Almost everything Skyward plays is synthesised at run time (`src/audio/`):
engines, propellers, jets, wind, rain, thunder, birds, ground roll, tyres,
gear / flap motors, horns, chimes, the vario, crash and sonic boom. There are
no recorded samples of those.

The only recorded assets are the cockpit voice callouts:

| Files | What | Source | Licence |
|---|---|---|---|
| `voice/*.wav` (26 clips, 16 kHz mono 16-bit, ~850 kB) | Radio-altimeter / GPWS callouts: "twenty five hundred", "one thousand", "five hundred" … "ten", "retard", "minimums", "hundred above", "approaching minimums", "stall", "pull up", "sink rate", "too low gear", "too low flaps", "terrain", "bank angle", "vee one", "overspeed", "engine failure" | Generated offline for this project with [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M) (ONNX build, voice `bm_fable`) running locally, then trimmed, normalised and resampled to 16 kHz. Script: see below. | Kokoro-82M model weights are Apache-2.0; the generated audio is original output made for Skyward and is released with the project (treat as CC0). |

The voice is heard through a simulated cockpit loudspeaker (band-pass,
presence peak, light drive) in `src/audio/voice.rs`.

## Regenerating the callouts

Scripts: `tools/gen_callouts.py` (Kokoro → 24 kHz WAVs) and `tools/to16k.py` (→ 16 kHz).

With a local Kokoro ONNX setup (`engine.py` exposing
`Engine().synthesise(text, voice, speed, lang)`), run a script that, for each
phrase, synthesises at speed 1.1–1.4, trims leading/trailing silence
(threshold 1 % of peak, keep 10 ms before / 50 ms after), normalises to
-1 dBFS, applies 5 ms fades and writes 24 kHz PCM-16; then resample to
16 kHz with a gentle roll-off above 6.5 kHz. File names must match
`src/audio/voice.rs` (e.g. `2500.wav`, `app_minimums.wav`, `too_low_gear.wav`).
