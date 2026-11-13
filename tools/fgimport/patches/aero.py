#!/usr/bin/env python3
"""Skyward calibration patches for imported JSBSim aerodynamics.

usage: python3 -I aero.py <id> assets/aircraft/<id>/fdm.xml

The FGMEMBERS C-5 and Vulcan FDMs (and the FGAddon CRJ700jsb) are Aeromatic first cuts (a generic
transport lift curve with a sharp stall at 13 deg). Skyward replaces a few
coefficients with figures calibrated to published performance (see
assets/aircraft/<id>/CREDITS.md and the flight.rs tests). Idempotent: each
patch rewrites the named function's table or constant.
"""
import re, sys

def table(rows):
    return "\n" + "\n".join("              " + "  ".join(f"{v:8.4f}" for v in r) for r in rows) + "\n            "

def table2d(cols, rows):
    """2-D tableData: a header line of column breakpoints, then rows (row key, values...)."""
    head = "              " + " " * 10 + "  ".join(f"{v:8.4f}" for v in cols)
    return "\n" + head + "\n" + "\n".join("              " + "  ".join(f"{v:8.4f}" for v in r) for r in rows) + "\n            "

# --- Concorde: ogival slender delta -------------------------------------------
# Lift: Polhamus leading-edge vortex theory for the low-speed curve,
# CL = Kp sin a cos^2 a + Kv sin^2 a cos a (Kp 2.4, Kv 3.4: CL 0.72 at the
# 13.5 deg approach alpha, i.e. Vref 162 kt at 111 t with 10 deg of pitch),
# vortex burst rounding it off past 25 deg (CLmax ~1.3 near 30 deg, no stall
# break); scaled with Mach (Prandtl-Glauert rise to the transonic peak, then
# the supersonic 4/sqrt(M^2-1)-like fall: CL-alpha ~2.2/rad at M2).
import math
_CM = [0.0, 0.6, 0.9, 1.0, 1.2, 1.6, 2.0, 2.2]
_CK = [1.0, 1.06, 1.16, 1.12, 0.95, 0.80, 0.70, 0.66]
_CA = [-0.35, -0.20, -0.10, 0.0, 0.05, 0.10, 0.15, 0.20, 0.26, 0.31, 0.35, 0.44, 0.52, 0.61, 0.79, 1.05, 1.57]

def _cl_low(a):
    d = math.degrees(abs(a))
    pol = lambda r: 2.4 * math.sin(r) * math.cos(r) ** 2 + 3.4 * math.sin(r) ** 2 * math.cos(r)
    if d <= 20.0:
        v = pol(abs(a))
    else:
        pts = [(20.0, pol(math.radians(20.0))), (25.0, 1.25), (30.0, 1.30), (35.0, 1.24), (45.0, 1.05), (60.0, 0.72), (90.0, 0.0)]
        v = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            if d <= x1:
                v = y0 + (y1 - y0) * (d - x0) / (x1 - x0)
                break
    return v if a >= 0 else -0.9 * v

def _concorde_cl():
    return table2d(_CM, [[a] + [_cl_low(a) * k for k in _CK] for a in _CA])

def _concorde_cd():
    """Zero-lift drag (friction + volume) plus drag due to lift: attached flow
    with leading-edge suction at cruise alpha (K CL^2, K 0.2 subsonic), the
    suction lost to the vortex at high alpha (CL tan a: approach L/D ~4);
    supersonic: no suction, partly recovered by the conical camber."""
    cd0 = [0.0085, 0.0085, 0.0088, 0.0090, 0.0086, 0.0078, 0.0070, 0.0069]
    rows = []
    for a in _CA:
        r = [a]
        for m, k, c0 in zip(_CM, _CK, cd0):
            cl = _cl_low(a) * k
            ta = math.tan(min(abs(a), 1.4))
            if m < 1.0:
                w = min(1.0, max(0.0, (math.degrees(abs(a)) - 4.0) / 8.0)) * 0.85
                cdi = 0.2 * cl * cl + w * max(0.0, abs(cl) * ta - 0.2 * cl * cl)
            else:
                cdi = 0.62 * abs(cl) * ta
            flat = 1.3 * math.sin(min(abs(a), 1.5707)) ** 2 if abs(a) > 0.6 else 0.0
            r.append(c0 + cdi + flat)
        rows.append(r)
    return table2d(_CM, rows)

PATCHES = {
    "crj700": {
        # Aeromatic first cut: lift rose linearly to CL 2.34 at 21 deg clean.
        # CRJ700 wing: CLmax ~1.35 at ~13 deg body alpha, CL0 0.25.
        "aero/force/Lift_alpha": ("table", [(-0.20, -0.85), (0.0, 0.25), (0.10, 0.78), (0.18, 1.18), (0.22, 1.33), (0.245, 1.36), (0.28, 1.26), (0.35, 1.05), (0.60, 0.85)]),
        # Double-slotted flaps 45 deg + slats: +0.92 at full flap.
        "aero/force/Lift_flap": ("value", 0.0205),
        "aero/force/Drag_flap": ("value", 0.0016),
        # Induced drag for AR 7.6 (e ~0.8, winglets).
        "aero/force/Drag_induced": ("value", 0.050),
        # Drag divergence ~M0.80 (Mmo 0.825; the cut had Mcrit 0.53).
        "aero/force/Drag_mach": ("table", [(0.0, 0.0), (0.76, 0.0), (0.80, 0.002), (0.83, 0.007), (0.86, 0.018), (0.90, 0.035), (1.0, 0.05), (1.8, 0.03)]),
        # Trim drag: the CRJ trims with its stabilizer.
        "aero/force/Drag_elevator": ("value", 0.008),
        # T-tail pitch damping and weathercock / rudder power for VMCA ~110 kt.
        # Static margin ~18 % MAC (the cut had ~11 %) and more damping: the cut
        # porpoised at high-altitude cruise.
        "aero/moment/Pitch_alpha": ("value", -1.0),
        # Elevator power from the T-tail (17.6 m2 at ~14 m: Cm_de ~2.5/rad at
        # low speed, enough to rotate at Vr), fading transonic.
        "aero/moment/Pitch_elevator": ("table", [(0.0, -2.4), (0.5, -2.2), (0.85, -1.5), (2.0, -0.6)]),
        "aero/moment/Pitch_damp": ("value", -30.0),
        "aero/moment/Pitch_alphadot": ("value", -10.0),
    },
    "concorde": {
        "aero/coefficient/CLalpha": ("raw", _concorde_cl),
        "aero/coefficient/CD0": ("raw", _concorde_cd),
        # Drag due to lift is in the CD0 table above.
        "aero/coefficient/CDi": ("value", 0.0),
        # Transonic wave drag, peaking at M1.1 and falling off supersonic.
        "aero/coefficient/CDmach": ("table", [(0.0, 0.0), (0.85, 0.0), (0.92, 0.0015), (0.98, 0.0060), (1.05, 0.0120), (1.12, 0.0140), (1.3, 0.0110), (1.7, 0.0065), (2.0, 0.0040), (2.2, 0.0038)]),
        # The flap lever drops the nose / visor: a small drag increment.
        "aero/coefficient/CDflap": ("value", 0.0040),
        # The reference point already follows the centre of pressure aft with
        # Mach (aero_ref_pt_shift_x); the cut's -0.7 on top of it needed ~11
        # deg of up-elevon at Mach 2. Residual stability only: static margin
        # ~8 % subsonic, ~10 % at Mach 2 with the trim fuel aft.
        "aero/coefficient/Cmalpha": ("value", -0.15),
    },
    "vulcan": {
        # Delta wing: low lift-curve slope, lift keeps rising to ~28 deg with
        # no sharp break, then a gentle decline (vortex lift, no stall drop).
        "aero/coefficient/CLalpha": ("table", [
            (-0.35, -0.95), (-0.17, -0.48), (0.00, 0.04), (0.087, 0.30), (0.175, 0.58), (0.262, 0.84),
            (0.349, 1.04), (0.436, 1.17), (0.489, 1.21), (0.541, 1.20), (0.611, 1.14), (0.785, 0.98),
            (1.047, 0.70), (1.571, 0.0)]),
        # Profile drag plus the high-alpha (lost leading-edge suction) rise:
        # the delta's huge drag at approach / high AoA.
        "aero/coefficient/CD0": ("table", [
            (-1.57, 1.50), (-0.26, 0.060), (-0.09, 0.016), (0.00, 0.0125), (0.09, 0.016), (0.175, 0.030),
            (0.262, 0.075), (0.349, 0.150), (0.436, 0.250), (0.524, 0.360), (0.785, 0.75), (1.57, 1.50)]),
        # Induced drag factor for AR 3.1 (e ~ 0.85).
        "aero/coefficient/CDi": ("value", 0.095),
        # Drag divergence ~M0.88 (Mmo 0.93, max ~M0.96 in a dive).
        "aero/coefficient/CDmach": ("table", [(0.0, 0.0), (0.84, 0.0), (0.88, 0.003), (0.92, 0.010), (0.96, 0.024), (1.05, 0.045), (1.8, 0.03)]),
        # Airbrakes (the flap lever drives them): big drag, small lift loss.
        "aero/coefficient/CDsb": ("value", 0.045),
        "aero/coefficient/dCLsb": ("value", -0.04),
        "aero/coefficient/CDde": ("value", 0.015),
    },
    "galaxy": {
        # Body-alpha lift curve without the FDM's +3 deg incidence offset
        # (CL0 0.4 had it landing flat / nose-first): CLmax 1.40 at ~16 deg.
        "aero/coefficient/CLalpha": ("table", [(-0.20, -0.85), (0.0, 0.18), (0.10, 0.66), (0.20, 1.12), (0.26, 1.36), (0.29, 1.40), (0.35, 1.25), (0.60, 0.80)]),
        # Fowler flaps + slats: CLmax ~2.6 at 40 deg (the FDM gave 3.2).
        "aero/coefficient/dCLflap": ("value", 0.030),
        # Supercritical-ish drag rise past M0.79 (Mmo 0.825).
        # Trim drag: the FDM charges 0.04 per unit elevator; the C-5 trims with
        # its stabilizer, so only a small elevator-drag term remains.
        "aero/force/Drag_elevator": ("value", 0.008),
        # Pitch damping of the big T-tail (the Aeromatic -17 / -6 let the
        # 280 t airframe porpoise under a pilot's corrections).
        "aero/moment/Pitch_damp": ("value", -32.0),
        "aero/moment/Pitch_alphadot": ("value", -12.0),
        # Rudder power for VMCA ~ 125 kt with an outboard engine out.
        "aero/moment/Yaw_rudder": ("value", -0.14),
        # High wing (despite the anhedral): positive spiral stability.
        "aero/moment/Roll_beta": ("value", -0.2),
        "aero/force/Drag_mach": ("table", [(0.0, 0.0), (0.78, 0.0), (0.81, 0.002), (0.84, 0.008), (0.88, 0.022), (0.95, 0.045), (1.10, 0.06), (1.80, 0.03)]),
    },
}

def main():
    ident, path = sys.argv[1], sys.argv[2]
    s = open(path, encoding="utf-8").read()
    for name, (kind, v) in PATCHES.get(ident, {}).items():
        m = re.search(r'<function name="%s">(.*?)</function>' % re.escape(name), s, re.S)
        if not m:
            sys.exit(f"{ident}: function {name} not found")
        body = m.group(1)
        if kind == "table":
            nb, n = re.subn(r"<tableData>.*?</tableData>", "<tableData>" + table(v) + "</tableData>", body, count=1, flags=re.S)
        elif kind == "raw":
            nb, n = re.subn(r"<tableData>.*?</tableData>", lambda _m: "<tableData>" + v() + "</tableData>", body, count=1, flags=re.S)
        else:
            vals = list(re.finditer(r"<value>\s*[-0-9.eE+]+\s*</value>", body))
            if not vals:
                sys.exit(f"{ident}: {name} has no constant")
            last = vals[-1]
            nb, n = body[: last.start()] + f"<value>{v}</value>" + body[last.end():], 1
        if n != 1:
            sys.exit(f"{ident}: could not patch {name}")
        s = s[: m.start(1)] + nb + s[m.end(1):]
    if "Skyward calibration" not in s:
        s = s.replace("-->", "-->\n<!-- Skyward calibration patches applied: tools/fgimport/patches/aero.py -->", 1)
    open(path, "w", encoding="utf-8").write(s)
    print(f"{ident}: patched {len(PATCHES.get(ident, {}))} functions")

main()
