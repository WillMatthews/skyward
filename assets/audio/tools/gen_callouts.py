"""Pre-render Skyward cockpit callouts with local Kokoro-82M (Apache-2.0)."""
import sys, os, numpy as np, soundfile as sf
sys.path.insert(0, os.path.expanduser("~/coding/experiments/kokoro-tts"))
from engine import Engine, SAMPLE_RATE
out = sys.argv[1]
os.makedirs(out, exist_ok=True)
VOICE = "bm_fable"
# (file id, text, speed)
PHRASES = [
    ("2500", "Twenty five hundred.", 1.15),
    ("1000", "One thousand.", 1.15),
    ("500", "Five hundred.", 1.2),
    ("400", "Four hundred.", 1.25),
    ("300", "Three hundred.", 1.25),
    ("200", "Two hundred.", 1.25),
    ("100", "One hundred.", 1.3),
    ("50", "Fifty.", 1.35),
    ("40", "Forty.", 1.4),
    ("30", "Thirty.", 1.4),
    ("20", "Twenty.", 1.4),
    ("10", "Ten.", 1.4),
    ("retard", "Retard.", 1.2),
    ("minimums", "Minimums.", 1.15),
    ("app_minimums", "Approaching minimums.", 1.15),
    ("hundred_above", "Hundred above.", 1.2),
    ("stall", "Stall.", 1.1),
    ("pull_up", "Pull up.", 1.1),
    ("sink_rate", "Sink rate.", 1.15),
    ("too_low_gear", "Too low. Gear.", 1.15),
    ("too_low_flaps", "Too low. Flaps.", 1.15),
    ("terrain", "Terrain.", 1.15),
    ("bank_angle", "Bank angle.", 1.15),
    ("v1", "Vee one.", 1.15),
    ("overspeed", "Overspeed.", 1.15),
    ("engine_fail", "Engine failure.", 1.15),
]
e = Engine()
for fid, text, spd in PHRASES:
    s, _ = e.synthesise(text, VOICE, spd, "b")
    s = np.asarray(s, dtype=np.float32)
    # trim leading/trailing silence
    thr = 0.01 * np.max(np.abs(s))
    idx = np.where(np.abs(s) > thr)[0]
    a, b = max(idx[0] - 240, 0), min(idx[-1] + 1200, len(s))
    s = s[a:b]
    s = s / max(np.max(np.abs(s)), 1e-6) * 0.89
    # 5 ms fades
    n = 120
    s[:n] *= np.linspace(0, 1, n); s[-n:] *= np.linspace(1, 0, n)
    sf.write(os.path.join(out, f"{fid}.wav"), s, SAMPLE_RATE, subtype="PCM_16")
    print(fid, f"{len(s)/SAMPLE_RATE:.2f}s")
