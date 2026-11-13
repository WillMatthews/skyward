#!/usr/bin/env python3
"""Add the radial-engine samples (FGAddon Douglas-Dc3 recordings, GPL-2.0;
Track H2b: the DC-3's P&W R-1830 Twin Wasps, also used for the DHC-3 Otter's
R-1340 Wasp) to assets/audio/samples, with the same processing as
build_vulcan.py (mono, DC removal, resample, seamless loop cross-fade,
-1.5 dB headroom, Vorbis q4, index.tsv row).

    python3 -I build_radial.py <Douglas-Dc3-root> <out-dir>

Provenance: ../CREDITS.md.
"""
import os, subprocess, sys
import numpy as np

L, O = True, False
# name, source (relative to the Douglas-Dc3 root), loop?, rate, start s, dur s, gain dB
RADIAL = [
    ("r18_idle", "Sounds/Ralenti.wav", L, 22050, 0.0, None, 0),
    ("r18_slow", "Sounds/Croisiere_ext_2.wav", L, 22050, 0.0, None, 0),
    ("r18_cruise", "Sounds/Croisiere_ext_1.wav", L, 22050, 0.0, None, 0),
    ("r18_i_hi", "Sounds/Croisiere_int_1.wav", L, 22050, 0.0, None, 0),
    ("r18_i_lo", "Sounds/Croisiere_int_2.wav", L, 22050, 0.0, None, 0),
    ("r18_start", "Sounds/Demarrage.wav", O, 22050, 0.0, None, 0),
    ("r18_stop", "Sounds/Arret_moteur_ext1.wav", O, 22050, 0.0, None, 0),
    ("r18_stop_int", "Sounds/Arret_moteur_int1.wav", O, 22050, 0.0, None, 0),
]


def load(path, rate, start, dur):
    cmd = ["ffmpeg", "-loglevel", "error", "-i", path, "-ac", "1", "-ar", str(rate), "-f", "f32le", "-"]
    x = np.frombuffer(subprocess.run(cmd, check=True, capture_output=True).stdout, dtype=np.float32).astype(np.float64)
    a = int(start * rate)
    b = len(x) if dur is None else min(len(x), a + int(dur * rate))
    return x[a:b]


def dominant_period(x, sr):
    n = min(len(x), sr * 2)
    seg = x[:n] * np.hanning(n)
    sp = np.abs(np.fft.rfft(seg))
    fr = np.fft.rfftfreq(n, 1 / sr)
    sel = (fr > 20) & (fr < 400)
    return int(sr / fr[sel][np.argmax(sp[sel])])


def make_loop(x, sr):
    n = len(x)
    xf = int(min(0.25 * sr, 0.2 * n))
    per = dominant_period(x, sr)
    head = x[:xf]
    best, best_e = -2.0, n - xf
    for e in range(max(xf, n - xf - 2 * per), n - xf + 1):
        t = x[e:e + xf]
        c = float(np.dot(t, head) / (np.linalg.norm(t) * np.linalg.norm(head) + 1e-12))
        if c > best:
            best, best_e = c, e
    e = best_e
    tail = x[e:e + xf]
    u = (np.arange(xf) + 0.5) / xf
    k = np.clip((best - 0.3) / 0.5, 0.0, 1.0)
    fin = (1 - k) * np.sin(0.5 * np.pi * u) + k * u
    fout = (1 - k) * np.cos(0.5 * np.pi * u) + k * (1 - u)
    y = x[:e].copy()
    y[:xf] = head * fin + tail * fout
    return y, best


def main():
    root, out = sys.argv[1], sys.argv[2]
    idx = os.path.join(out, "index.tsv")
    rows = [r for r in open(idx).read().splitlines() if r and r.split("\t")[0] not in {v[0] for v in RADIAL}]
    for name, src, loop, rate, start, dur, gdb in RADIAL:
        x = load(os.path.join(root, src), rate, start, dur)
        x = x - np.mean(x)
        corr = None
        if loop:
            x, corr = make_loop(x, rate)
        else:
            nf = int(0.03 * rate)
            x[-nf:] *= np.linspace(1, 0, nf)
            ni = int(0.002 * rate)
            x[:ni] *= np.linspace(0, 1, ni)
        pk = np.max(np.abs(x)) + 1e-12
        g = 10 ** ((gdb - 1.5) / 20)
        if pk * g > 0.89:
            g = 0.89 / pk
        x = (x * g).astype(np.float32)
        dst = os.path.join(out, name + ".ogg")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "f32le", "-ar", str(rate), "-ac", "1", "-i", "-",
                        "-c:a", "libvorbis", "-q:a", "4", "-map_metadata", "-1", dst], input=x.tobytes(), check=True)
        rows.append("\t".join(map(str, (name, rate, len(x), int(loop), f"{20*np.log10(g)+1.5:.2f}"))))
        print(f"{name:12s} {rate:6d} Hz {len(x)/rate:6.2f} s {'loop' if loop else 'once'} {os.path.getsize(dst)/1024:7.1f} kB"
              + (f"  seam corr {corr:+.2f}" if loop else "") + f"  gain {20*np.log10(g):+.1f} dB")
    open(idx, "w").write("\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
