import sys, glob, os, numpy as np, soundfile as sf
src, dst = sys.argv[1], sys.argv[2]
os.makedirs(dst, exist_ok=True)
tot = 0
for f in sorted(glob.glob(src + "/*.wav")):
    s, sr = sf.read(f, dtype="float32")
    n = len(s); m = int(round(n * 16000 / sr))
    S = np.fft.rfft(s)
    keep = m // 2 + 1
    S2 = S[:keep].copy()
    # gentle rolloff above 6.5 kHz
    fr = np.arange(keep) * sr / n
    S2 *= np.clip((7800 - fr) / 1300, 0, 1)
    y = np.fft.irfft(S2, m) * (m / n)
    y = np.clip(y, -1, 1)
    sf.write(os.path.join(dst, os.path.basename(f)), y, 16000, subtype="PCM_16")
    tot += os.path.getsize(os.path.join(dst, os.path.basename(f)))
print("total bytes", tot)
