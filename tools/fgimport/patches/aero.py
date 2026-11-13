#!/usr/bin/env python3
"""Skyward calibration patches for imported JSBSim aerodynamics.

usage: python3 -I aero.py <id> assets/aircraft/<id>/fdm.xml

The FGMEMBERS C-5 and Vulcan FDMs are Aeromatic first cuts (a generic
transport lift curve with a sharp stall at 13 deg). Skyward replaces a few
coefficients with figures calibrated to published performance (see
assets/aircraft/<id>/CREDITS.md and the flight.rs tests). Idempotent: each
patch rewrites the named function's table or constant.
"""
import re, sys

def table(rows):
    return "\n" + "\n".join("              " + "  ".join(f"{v:8.4f}" for v in r) for r in rows) + "\n            "

PATCHES = {
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
