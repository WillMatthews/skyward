#!/usr/bin/env python3
"""Build the embedded engine / effect samples (assets/audio/samples/*.ogg)
from the FlightGear source recordings listed in MANIFEST.

    python3 build_samples.py <source-root> <out-dir>

<source-root> holds the fetched FlightGear files at the paths given below
(`c172/...` = github.com/c172p-team/c172p, `svn/<Aircraft>/...` = FGAddon
trunk/Aircraft, `fg/...` = fgdata, `ejet/...` = tdammers/E-jet-family-YV,
`fgm/<repo>/...` = github.com/FGMEMBERS/<repo>).
Provenance and licences: ../CREDITS.md.

Per file: mix to mono, trim, resample (polyphase, scipy) to a rate chosen so
that the highest pitch the engine plays it at stays below Nyquist at a
44.1 kHz output (the sampler then needs no run-time anti-aliasing), and for
loops bake a seamless loop: the end point is searched (±1 dominant period)
for the best waveform match with the start, and the tail is cross-faded into
the head (equal-power for uncorrelated material, equal-gain when the match
is good, blended in between). One-shots get a short fade-out. Everything is
scaled by -1.5 dB (Vorbis overshoot headroom: lewton decodes to i16) and
encoded with libvorbis (ffmpeg) at q4. Writes `index.tsv` (name, rate,
frames, loop, extra attenuation dB beyond the common -1.5 dB, which the player
undoes) next to the .ogg files; the Rust side checks decoded lengths
against it.
"""
import os, subprocess, sys, tempfile
import numpy as np
import soundfile as sf
from scipy.signal import resample_poly
from math import gcd

# name, sources (summed to mono), loop?, rate, start s, dur s (None = all), gain dB
L, O = True, False
MANIFEST = [
    # ---- Cessna 172P (Lycoming O-320 / IO-360, 2-blade prop)
    ("c172_idle", ["c172/Sounds/engine-idle.wav"], L, 30000, 0.0, 12.0, 0),
    ("c172_run", ["c172/Sounds/engine.wav"], L, 34000, 0.0, None, 0),
    ("c172_crank", ["c172/Sounds/engine-crank.wav"], L, 32000, 0.0, None, 0),
    ("c172_start", ["c172/Sounds/engine-start.wav"], O, 32000, 0.0, None, 0),
    ("c172_cut", ["c172/Sounds/engine-cut.wav"], O, 24000, 0.0, None, 0),
    ("c172_stall", ["c172/Sounds/stall2.wav"], L, 32000, 0.0, None, 0),
    ("tyre_roll", ["c172/Sounds/tires-rolling-asphalt.wav"], L, 24000, 0.0, None, 0),
    ("tyre_gravel", ["c172/Sounds/tires-rolling-gravel.wav"], L, 24000, 0.0, None, 0),
    ("flap_motor_light", ["c172/Sounds/flaps-motor.wav"], L, 32000, 0.5, 5.5, 0),
    # ---- Spitfire IX (Rolls-Royce Merlin 66 V12)
    ("merlin_1", ["svn/spitfireIX/Sounds/merlin_rpm1.wav"], L, 28000, 0.0, None, 0),
    ("merlin_2", ["svn/spitfireIX/Sounds/spita.wav"], L, 11025, 0.0, None, 0),
    ("merlin_3", ["svn/spitfireIX/Sounds/merlin_rpm4.wav"], L, 11025, 0.0, None, 0),
    ("merlin_start", ["svn/spitfireIX/Sounds/merlin_starter.wav"], O, 22050, 0.0, None, 0),
    ("merlin_stop", ["svn/spitfireIX/Sounds/merlin_shutdown.wav"], O, 22050, 0.0, None, 0),
    # ---- F-16 (GE F110 / P&W F100, afterburning)
    ("f16_x_idle_f", ["svn/f16/Sounds/GEED/exterior_ground_idle_loop_mono.wav"], L, 24000, 0.0, 12.0, 0),
    ("f16_x_idle_a", ["svn/f16/Sounds/GEED/exterior_ground_idle_aft_loop_mono.wav"], L, 26000, 0.0, None, 0),
    ("f16_x_mil_f", ["svn/f16/Sounds/GEED/mil_exterior.wav"], L, 38000, 0.0, None, 0),
    ("f16_x_mil_a", ["svn/f16/Sounds/GEED/full_mil_aft_mono.wav"], L, 32000, 0.0, None, 0),
    ("f16_x_ab", ["svn/f16/Sounds/GEED/afterburner_exterior.wav"], L, 32000, 0.0, None, 0),
    ("f16_x_abx", ["svn/f16/Sounds/exhaust_loop.wav"], L, 16000, 0.0, None, 0),
    ("f16_x_whine", ["svn/f16/Sounds/FGUK/f16-whine.wav"], L, 24000, 0.0, None, 0),
    ("f16_x_roar", ["svn/f16/Sounds/engine/roar-medium.wav"], L, 28000, 0.0, None, 0),
    ("f16_i_idle_lo", ["svn/f16/Sounds/Timi/F-16_In_Idle_Low-left.wav", "svn/f16/Sounds/Timi/F-16_In_Idle_Low-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_idle_hi", ["svn/f16/Sounds/Timi/F-16_In_Idle_High-left.wav", "svn/f16/Sounds/Timi/F-16_In_Idle_High-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_88_lo", ["svn/f16/Sounds/Timi/F-16_In_N2_88_Low-left.wav", "svn/f16/Sounds/Timi/F-16_In_N2_88_Low-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_88_hi", ["svn/f16/Sounds/Timi/F-16_In_N2_88_High-left.wav", "svn/f16/Sounds/Timi/F-16_In_N2_88_High-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_100_lo", ["svn/f16/Sounds/Timi/F-16_In_N100_Low-left.wav", "svn/f16/Sounds/Timi/F-16_In_N100_Low-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_100_hi", ["svn/f16/Sounds/Timi/F-16_In_N100_High-left.wav", "svn/f16/Sounds/Timi/F-16_In_N100_High-right.wav"], L, 16000, 0.0, 10.0, 0),
    ("f16_i_ab", ["svn/f16/Sounds/Timi/F-16_In_Afterburner-left.wav", "svn/f16/Sounds/Timi/F-16_In_Afterburner-right.wav"], L, 16000, 0.0, 10.0, 0),
    # ---- ATR 72 (P&W PW127): DHC-8 (PW120-series) recordings + fgdata generics
    ("atr_out", ["svn/DHC-8/Sounds/out.wav"], L, 32000, 0.0, None, 0),
    ("atr_int", ["svn/DHC-8/Sounds/intern.wav"], L, 22050, 0.0, None, 0),
    ("atr_rev", ["svn/DHC-8/Sounds/reverse.wav"], L, 24000, 0.0, None, 0),
    ("atr_prop", ["fg/Sounds/turboprop1.wav"], L, 11025, 0.0, None, 0),
    ("atr_start", ["fg/Sounds/turbine_start.wav"], O, 11025, 0.0, None, 0),
    ("atr_stop", ["fg/Sounds/turbine_shutdown.wav"], O, 11025, 0.0, 11.0, 0),
    # ---- Avro Vulcan B.2 Olympus 301 (FGUK, FGMEMBERS/AvroVulcanB2) — also build_vulcan.py
    ("vul_x_turb", ["fgm/AvroVulcanB2/Sounds/vulcan-engine-exterior.wav"], L, 28000, 0.0, None, 0),
    ("vul_x_jet", ["fgm/AvroVulcanB2/Sounds/jet.wav"], L, 11025, 0.0, None, 0),
    ("vul_x_blast", ["fgm/AvroVulcanB2/Sounds/vulcan-jetblast.wav"], L, 20000, 0.0, 14.0, 0),
    ("vul_x_howl", ["fgm/AvroVulcanB2/Sounds/vulcan-howel.wav"], L, 24000, 0.0, None, 0),
    ("vul_i_eng", ["fgm/AvroVulcanB2/Sounds/vulcan-engine-inside.wav"], L, 20000, 0.0, None, 0),
    ("vul_start", ["fgm/AvroVulcanB2/Sounds/vulcan-start.wav"], O, 16000, 0.0, 24.0, 0),
    ("vul_stop", ["fgm/AvroVulcanB2/Sounds/vulcan-shutdown.wav"], O, 16000, 0.0, 16.0, 0),
    # ---- A320 CFM56-5B (SASA recordings from the A320-family)
    ("cfm_x_idle", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-idle.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_idle_r", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-idleRear.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_idle_c", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-idleClose.wav"], L, 26000, 0.0, None, 0),
    ("cfm_x_idle_d", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-idleDist.wav"], L, 22050, 2.0, 12.0, 0),
    ("cfm_x_spool", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-spool.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_lowspool", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-lowspool.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_spool_c", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-spoolClose.wav"], L, 22050, 2.0, 12.0, 0),
    ("cfm_x_spool_cr", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-spoolCloseRear.wav"], L, 22050, 2.0, 12.0, 0),
    ("cfm_x_rumble", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-rumble.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_rumble_d", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-rumbleDist.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_buzz", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-buzz.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_buzz_d", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-buzzDist.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_rev", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-reverse.wav"], L, 22050, 0.0, None, 0),
    ("cfm_x_start", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-startup.wav"], O, 22050, 0.0, 16.0, 0),
    ("cfm_x_stop", ["svn/A320-family/Sounds/SASA/CFM56B/external/cfm-shutdown.wav"], O, 22050, 0.0, 18.0, 0),
    ("cfm_c_idle", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-idle.wav"], L, 26000, 0.0, 10.0, 0),
    ("cfm_c_idle2", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-idle2.wav"], L, 28000, 0.0, 10.0, 0),
    ("cfm_c_spool", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-lowspool.wav"], L, 22050, 0.0, 10.0, 0),
    ("cfm_c_hum", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-hum.wav"], L, 8000, 0.0, 10.0, 0),
    ("cfm_c_comb", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-comb.wav"], L, 22050, 0.0, 10.0, 0),
    ("cfm_c_comb2", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-comb-full.wav"], L, 32000, 0.0, 10.0, 0),
    ("cfm_c_buzz", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-buzz.wav"], L, 32000, 0.0, 10.0, 0),
    ("cfm_c_rev", ["svn/A320-family/Sounds/SASA/CFM56B/cockpit/cfm-reverse.wav"], L, 24000, 0.0, 10.0, 0),
    ("cfm_c_start", ["svn/A320-family/Sounds/SASA/CFM56B/start/A320_cockpit_startup.wav"], O, 22050, 0.0, 16.0, 0),
    ("cfm_c_stop", ["svn/A320-family/Sounds/SASA/CFM56B/start/A320_cockpit_shutdown.wav"], O, 22050, 0.0, 16.0, 0),
    # ---- IAE V2500 (SASA, A320-family) — used for the E190
    ("v25_x_idle", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-idle.wav"], L, 22050, 4.0, 12.0, 0),
    ("v25_x_idle_r", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-idleRear.wav"], L, 22050, 0.0, None, 0),
    ("v25_x_spool", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-spool.wav"], L, 22050, 0.0, None, 0),
    ("v25_x_lowspool", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-lowspool.wav"], L, 26000, 0.0, None, 0),
    ("v25_x_buzz", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-buzz.wav"], L, 36000, 0.0, None, 0),
    ("v25_x_buzz_d", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-buzzDist.wav"], L, 36000, 0.0, None, 0),
    ("v25_x_start", ["svn/A320-family/Sounds/SASA/V2500/external/v2500-startup.wav"], O, 22050, 0.0, 16.0, 0),
    ("v25_c_idle", ["svn/A320-family/Sounds/SASA/V2500/cockpit/v2500-idle.wav"], L, 22050, 0.0, 10.0, 0),
    ("v25_c_lowspool", ["svn/A320-family/Sounds/SASA/V2500/cockpit/v2500-lowspool.wav"], L, 26000, 0.0, 10.0, 0),
    ("v25_c_comb", ["svn/A320-family/Sounds/SASA/V2500/cockpit/v2500-comb.wav"], L, 36000, 0.0, 10.0, 0),
    ("v25_c_buzz", ["svn/A320-family/Sounds/SASA/V2500/cockpit/v2500-buzz.wav"], L, 36000, 0.0, 10.0, 0),
    ("v25_c_rev", ["svn/A320-family/Sounds/SASA/V2500/cockpit/v2500-reverse.wav"], L, 24000, 0.0, 10.0, 0),
    ("v25_c_start", ["svn/A320-family/Sounds/SASA/V2500/start/v2500-start2.wav"], O, 22050, 0.0, 16.0, 0),
    # ---- airframe / ground / environment
    ("heavy_roll", ["svn/A320-family/Sounds/SASA/Roll.wav"], L, 22050, 0.0, None, 0),
    ("heavy_roll_int", ["svn/A320-family/Sounds/SASA/Cockpit_Roll.wav"], L, 16000, 0.0, None, 0),
    ("heavy_touch", ["svn/A320-family/Sounds/SASA/Touch-Main.wav"], O, 24000, 0.0, None, 0),
    ("fighter_touch", ["svn/f16/Sounds/touchdown.wav"], O, 22050, 0.0, None, 0),
    ("gear_heavy", ["svn/A320-family/Sounds/SASA/A320_gear.wav"], O, 22050, 0.0, None, 0),
    ("gear_fighter", ["svn/f16/Sounds/gear-transit.wav"], L, 24000, 0.0, None, 0),
    ("flap_motor_heavy", ["svn/A320-family/Sounds/SASA/A320_flapmotor.wav"], L, 16000, 0.0, None, 0),
    ("wind_airliner", ["svn/A320-family/Sounds/SASA/wind_cockpit.wav"], L, 22050, 0.0, None, 0),
    ("wind_light", ["fg/Sounds/wind1.wav"], L, 24000, 0.0, None, 0),
    ("glider_wind", ["svn/ASK21/Sounds/whoosh-loop.wav"], L, 32000, 0.0, None, 0),
    ("glider_brake", ["svn/ASK21/Sounds/airbrakesL.wav"], L, 22050, 0.0, None, 0),
    ("glider_roll", ["svn/ASK21/Sounds/ground_roll3.wav"], L, 22050, 0.0, None, 0),
    ("thunder_1", ["fg/Sounds/thunder1.wav"], O, 16000, 0.0, None, 0),
    ("thunder_2", ["fg/Sounds/thunder2.wav"], O, 16000, 0.0, None, 0),
    ("thunder_3", ["fg/Sounds/thunder3.wav"], O, 16000, 0.0, None, 0),
    # ---- impacts
    ("crash_c172", ["c172/Sounds/crash.wav"], O, 32000, 0.0, None, 0),
    ("crash_heavy", ["c172/Sounds/damage-heavy.wav"], O, 32000, 0.0, None, 0),
    ("crash_light", ["c172/Sounds/damage.wav"], O, 32000, 0.0, None, 0),
    ("crash_scrape", ["c172/Sounds/scratch.wav"], L, 24000, 0.0, None, 0),
    ("crash_f16", ["svn/f16/Sounds/Effects/aircraft-crash.wav"], O, 32000, 0.0, None, 0),
    ("crash_explode", ["svn/f16/Sounds/Effects/aircraft-explode.wav"], O, 22050, 0.0, None, 0),
    ("crash_water", ["svn/f16/Sounds/Effects/aircraft-water-crash.wav"], O, 32000, 0.0, None, 0),
]


def load(root, paths, start, dur):
    acc, sr0 = None, None
    for p in paths:
        x, sr = sf.read(os.path.join(root, p), always_2d=True, dtype="float64")
        x = x.mean(axis=1)
        if sr0 is None:
            sr0 = sr
        assert sr == sr0, p
        acc = x if acc is None else acc[: min(len(acc), len(x))] + x[: min(len(acc), len(x))]
    a = int(start * sr0)
    b = len(acc) if dur is None else min(len(acc), a + int(dur * sr0))
    return acc[a:b], sr0


def resample(x, sr, rate):
    if rate >= sr:
        return x, sr
    g = gcd(int(rate), int(sr))
    return resample_poly(x, int(rate) // g, int(sr) // g, window=("kaiser", 9.0)), int(rate)


def dominant_period(x, sr):
    """Strongest periodicity 20..400 Hz (samples), for the loop-point search."""
    n = min(len(x), sr * 2)
    seg = x[:n] * np.hanning(n)
    sp = np.abs(np.fft.rfft(seg))
    fr = np.fft.rfftfreq(n, 1 / sr)
    sel = (fr > 20) & (fr < 400)
    f = fr[sel][np.argmax(sp[sel])]
    return int(sr / f)


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
    # Equal-power (sin/cos) for uncorrelated material, equal-gain (linear)
    # when head and tail match well; blend in between.
    k = np.clip((best - 0.3) / 0.5, 0.0, 1.0)
    fin = (1 - k) * np.sin(0.5 * np.pi * u) + k * u
    fout = (1 - k) * np.cos(0.5 * np.pi * u) + k * (1 - u)
    y = x[:e].copy()
    y[:xf] = head * fin + tail * fout
    return y, best


def main():
    root, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    rows = []
    total = 0
    for name, srcs, loop, rate, start, dur, gdb in MANIFEST:
        x, sr = load(root, srcs, start, dur)
        x = x - np.mean(x)
        x, sr = resample(x, sr, rate)
        corr = None
        if loop:
            x, corr = make_loop(x, sr)
        else:
            nf = int(0.03 * sr)
            x[-nf:] *= np.linspace(1, 0, nf)
            ni = int(0.002 * sr)
            x[:ni] *= np.linspace(0, 1, ni)
        pk = np.max(np.abs(x)) + 1e-12
        g = 10 ** ((gdb - 1.5) / 20)
        if pk * g > 0.89:
            g = 0.89 / pk
        x = (x * g).astype(np.float32)
        with tempfile.TemporaryDirectory() as td:
            wav = os.path.join(td, "x.wav")
            sf.write(wav, x, sr, subtype="FLOAT")
            dst = os.path.join(out, name + ".ogg")
            subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", wav, "-c:a", "libvorbis", "-q:a", "4", "-map_metadata", "-1", dst], check=True)
        sz = os.path.getsize(dst)
        total += sz
        rows.append((name, sr, len(x), int(loop), f"{20*np.log10(g)+1.5:.2f}"))
        print(f"{name:18s} {sr:6d} Hz {len(x)/sr:6.2f} s {'loop' if loop else 'once'} {sz/1024:7.1f} kB" + (f"  seam corr {corr:+.2f}" if loop else "") + f"  gain {20*np.log10(g):+.1f} dB")
    with open(os.path.join(out, "index.tsv"), "w") as f:
        for r in rows:
            f.write("\t".join(map(str, r)) + "\n")
    print(f"total {total/1024/1024:.2f} MB in {len(rows)} files")


if __name__ == "__main__":
    main()
