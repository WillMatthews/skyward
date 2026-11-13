"""Pre-render Skyward's ATC radio vocabulary with local Kokoro-82M (Apache-2.0).

    python gen_atc.py <assets/audio/atc> [cache dir]

Reads `vocab.tsv` (key, TTS text, voices, speed factor) from the output
directory and renders every phrase for every voice that needs it: the pilot
(P), the controllers (C) and the ATIS readers (A). Each voice becomes one
packed Ogg Vorbis stream (`<voice>.ogg`, 12 kHz mono — the radio is
band-limited to ~3 kHz anyway) plus an index (`<voice>.idx`: key, first
sample, length) that `src/audio/radio.rs` slices transmissions from.

Clips are trimmed to the voiced part (a few ms of lead-in, a short tail),
loudness-matched on their voiced frames (so concatenated words sit at one
level) and faded at both ends. Rendered clips are cached by voice / text /
speed in the cache directory, so editing the vocabulary only renders what is
new. Uses the Kokoro ONNX engine in ~/coding/experiments/kokoro-tts.
"""
import hashlib
import os
import subprocess
import sys

import numpy as np
import soundfile as sf

sys.path.insert(0, os.path.expanduser("~/coding/experiments/kokoro-tts"))
from engine import SAMPLE_RATE, Engine  # noqa: E402

OUT = sys.argv[1]
CACHE = sys.argv[2] if len(sys.argv) > 2 else os.path.join(OUT, ".cache")
os.makedirs(CACHE, exist_ok=True)
RATE = 12_000

# Kokoro voice, espeak language, base speed, which vocabulary tags it speaks.
VOICES = {
    "am_liam": ("a", 1.08, "P"),  # the pilot (read-backs, requests)
    "bf_emma": ("b", 1.04, "CA"),  # ATIS / delivery
    "am_onyx": ("a", 1.10, "C"),  # ground
    "bm_fable": ("b", 1.10, "C"),  # tower
    "af_river": ("a", 1.10, "C"),  # approach / director / radar
    "af_aoede": ("a", 1.06, "CA"),  # control / departure, second ATIS
}


def vocab():
    rows = []
    with open(os.path.join(OUT, "vocab.tsv")) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            k, t, tags, sp = line.rstrip("\n").split("\t")
            rows.append((k, t, tags, float(sp)))
    return rows


def trim(s):
    """The spoken part: 5 ms frames from the first above -44 dB to the last
    above -33 dB of the clip's loudest frame (Kokoro leaves long decays on
    isolated words; spoken in a string they would be cut short)."""
    f = SAMPLE_RATE // 200
    n = len(s) // f
    en = np.array([np.sqrt(np.mean(s[i * f:(i + 1) * f] ** 2)) for i in range(n)])
    # Onsets keep their (quiet) fricatives / aspiration: a lower threshold
    # at the start than at the end.
    start = np.where(en > en.max() * 10 ** (-44 / 20))[0]
    end = np.where(en > en.max() * 10 ** (-33 / 20))[0]
    a = max(start[0] * f - SAMPLE_RATE * 10 // 1000, 0)
    b = min((end[-1] + 1) * f + SAMPLE_RATE * 20 // 1000, len(s))
    s = s[a:b].copy()
    # Loudness: RMS of the voiced frames → -19 dBFS, peaks kept below -1 dBFS.
    f = SAMPLE_RATE // 100
    m = len(s) // f
    en = np.array([np.sqrt(np.mean(s[i * f:(i + 1) * f] ** 2)) for i in range(max(m, 1))])
    voiced = en[en > en.max() * 0.25]
    rms = float(np.sqrt(np.mean(voiced ** 2))) if len(voiced) else 1e-3
    s *= 10 ** (-19 / 20) / max(rms, 1e-5)
    pk = np.max(np.abs(s))
    if pk > 0.89:
        s *= 0.89 / pk
    k = SAMPLE_RATE * 4 // 1000
    s[:k] *= np.linspace(0, 1, k)
    k = SAMPLE_RATE * 12 // 1000
    s[-k:] *= np.linspace(1, 0, k)
    return s


def wsola(x, rate):
    """Time compression by `rate` (> 1 = faster) keeping the pitch (WSOLA:
    25 ms Hann frames, ±6 ms search for the best waveform match)."""
    if rate <= 1.001:
        return x
    sr = SAMPLE_RATE
    n = int(0.025 * sr)
    hs = n // 2
    ha = hs * rate
    tol = int(0.006 * sr)
    win = np.hanning(n).astype(np.float32)
    frames = int((len(x) - n - tol) / ha)
    if frames < 2:
        return x
    out = np.zeros(frames * hs + n, dtype=np.float32)
    norm = np.zeros_like(out)
    prev = 0
    for k in range(frames):
        a = int(k * ha)
        if k == 0:
            pos = 0
        else:
            nat = x[prev + hs:prev + hs + n]
            lo, hi = max(a - tol, 0), min(a + tol, len(x) - n)
            best, pos = -1e9, a
            for c in range(lo, hi + 1, 2):
                v = float(np.dot(nat, x[c:c + n]))
                if v > best:
                    best, pos = v, c
        out[k * hs:k * hs + n] += x[pos:pos + n] * win
        norm[k * hs:k * hs + n] += win
        prev = pos
    return out / np.maximum(norm, 1e-3)


def halfband(x):
    """24 kHz → 12 kHz: windowed-sinc low-pass at 5.4 kHz, then decimate."""
    n = 63
    t = np.arange(n) - (n - 1) / 2
    fc = 5400 / SAMPLE_RATE
    h = 2 * fc * np.sinc(2 * fc * t) * np.blackman(n)
    h /= h.sum()
    y = np.convolve(x, h, mode="same")
    return y[::2].astype(np.float32)


def main():
    rows = vocab()
    eng = Engine()
    total = 0
    for voice, (lang, speed, tags) in VOICES.items():
        clips = []
        for k, text, need, sp in rows:
            if not set(need) & set(tags):
                continue
            spd = round(min(speed * sp, 1.6), 3)
            h = hashlib.sha1(f"{voice}|{lang}|{text}|{spd}".encode()).hexdigest()[:16]
            cp = os.path.join(CACHE, f"{voice}_{h}.npy")
            if os.path.exists(cp):
                s = np.load(cp)
            else:
                s, _ = eng.synthesise(text, voice, spd, lang)
                s = np.asarray(s, dtype=np.float32)
                np.save(cp, s)
            # Isolated words come out slow: compress numbers / letters most
            # (ATC reads them briskly), phrases a little.
            words = len(text.split())
            rate = 1.35 if sp >= 1.3 else 1.28 if sp >= 1.2 else 1.12 if words <= 2 else 1.08
            clips.append((k, halfband(trim(wsola(s, rate)))))
        gap = np.zeros(RATE // 25, dtype=np.float32)
        parts, idx, pos = [], [], 0
        for k, c in clips:
            idx.append((k, pos, len(c)))
            parts += [c, gap]
            pos += len(c) + len(gap)
        pcm = np.concatenate(parts)
        wav = os.path.join(CACHE, f"{voice}.wav")
        sf.write(wav, pcm, RATE, subtype="PCM_16")
        ogg = os.path.join(OUT, f"{voice}.ogg")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", wav, "-c:a", "libvorbis", "-q:a", "1", "-map_metadata", "-1", ogg], check=True)
        with open(os.path.join(OUT, f"{voice}.idx"), "w") as f:
            f.write(f"{RATE}\t{len(pcm)}\n")
            for k, a, n in idx:
                f.write(f"{k}\t{a}\t{n}\n")
        size = os.path.getsize(ogg)
        total += size
        print(f"{voice}: {len(clips)} clips, {len(pcm) / RATE:.0f} s, {size / 1e6:.2f} MB")
    print(f"total {total / 1e6:.2f} MB")


main()
