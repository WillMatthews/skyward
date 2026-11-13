"""Pre-render Skyward's mission cabin PA lines and the windshear warning with
local Kokoro-82M (Apache-2.0), as small Ogg Vorbis files (16 kHz mono).

    python gen_pa.py <assets/audio/pa>

The captain is `am_onyx`, the cabin crew `bf_emma`, the windshear warning the
GPWS voice (`bm_fable`, as gen_callouts.py). Ids must match
`Clip::pa` / `Clip::Windshear` in src/audio/params.rs.
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import soundfile as sf

sys.path.insert(0, os.path.expanduser("~/coding/experiments/kokoro-tts"))
from engine import SAMPLE_RATE, Engine  # noqa: E402

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
CAPTAIN = ("am_onyx", "a", 1.0)
CREW = ("bf_emma", "b", 1.05)
SHOUT = ("bf_emma", "b", 1.18)
GPWS = ("bm_fable", "b", 1.15)
LINES = [
    ("crew_stations", CAPTAIN, "Cabin crew, at stations."),
    ("brace", CAPTAIN, "This is the captain. Brace for impact."),
    ("heads_down", SHOUT, "Brace! Brace! Heads down! Stay down!"),
    ("masks", CREW, "Put your own oxygen mask on first. Pull the mask towards you, place it over your nose and mouth, and breathe normally."),
    ("evacuate", SHOUT, "Evacuate! Evacuate! Release your seatbelts and get out!"),
    ("remain_seated", CAPTAIN, "This is the captain. Remain seated. Remain seated."),
    ("divert", CAPTAIN, "Ladies and gentlemen, this is the captain. We have a technical problem, and we are diverting to the nearest airport. Please remain seated with your seatbelts fastened."),
    ("lifejackets", CREW, "Put on your life jacket now. Do not inflate it until you leave the aircraft."),
    ("purser", CAPTAIN, "Purser to the flight deck, please. Purser to the flight deck."),
    ("emergency_descent", SHOUT, "Emergency descent! Emergency descent! Masks on, fasten your seatbelts!"),
    ("windshear", GPWS, "Windshear. Windshear. Windshear."),
]
e = Engine()
for fid, (voice, lang, spd), text in LINES:
    s, _ = e.synthesise(text, voice, spd, lang)
    s = np.asarray(s, dtype=np.float32)
    thr = 0.01 * np.max(np.abs(s))
    idx = np.where(np.abs(s) > thr)[0]
    a, b = max(idx[0] - 240, 0), min(idx[-1] + 2400, len(s))
    s = s[a:b]
    s = s / max(np.max(np.abs(s)), 1e-6) * 0.89
    n = 120
    s[:n] *= np.linspace(0, 1, n)
    s[-n:] *= np.linspace(1, 0, n)
    with tempfile.TemporaryDirectory() as td:
        wav = os.path.join(td, "x.wav")
        sf.write(wav, s, SAMPLE_RATE, subtype="PCM_16")
        ogg = os.path.join(OUT, f"{fid}.ogg")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", wav, "-ac", "1", "-ar", "16000", "-c:a", "libvorbis", "-q:a", "1", "-map_metadata", "-1", ogg], check=True)
    print(fid, f"{len(s) / SAMPLE_RATE:.2f}s", os.path.getsize(ogg), "bytes")
