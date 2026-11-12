#!/usr/bin/env python3
"""Neutral (unbranded) Skyward liveries derived from GPL FlightGear textures.

usage: python3 -I make_liveries.py <source-root>   (source-root = dir holding <aircraft>/ trees)
Writes <source-root>/<aircraft>/Skyward/*.png, referenced by the import configs.
Derived works of GPL-2.0 textures: same licence (see assets/aircraft/*/CREDITS.md).
"""
import colorsys, os, sys
from PIL import Image

root = sys.argv[1]

def out(ac, name):
    d = os.path.join(root, ac, "Skyward"); os.makedirs(d, exist_ok=True)
    return os.path.join(d, name)

def a320():
    """Factory-primer A320 texture → plain white fuselage / tail, light-grey wings."""
    src = Image.open(os.path.join(root, "a320/Models/Fuselage/res/Fuse-Main.png")).convert("RGBA")
    W, H = src.size
    px = src.load()
    for y in range(H):
        for x in range(W):
            r, g, b, a = px[x, y]
            h, s, v = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
            hd = h * 360
            primer = s > 0.10 and ((15 <= hd <= 110) or (185 <= hd <= 235)) and v > 0.25
            fy, fx = y / H, x / W
            tail = fx > 0.75 and 0.42 < fy < 0.66
            # The primer fin's dark diagonal masking bands: paint them over too.
            if tail and v < 0.25:
                px[x, y] = (242, 243, 245, a)
                continue
            if not primer:
                continue
            fus = fy < 0.44 or tail
            base = (0.95, 0.955, 0.96) if fus else (0.78, 0.80, 0.83)
            ref = 0.62 if fus else 0.55
            k = max(0.78, min(1.04, v / ref)) if fus else max(0.75, min(1.06, v / ref))
            px[x, y] = tuple(int(min(255, c * k * 255)) for c in base) + (a,)
    # Erase the primer airframe's registration (dark lettering on the aft fuselage).
    for (x0, y0, x1, y1) in [(0.645, 0.110, 0.735, 0.128), (0.645, 0.312, 0.735, 0.330)]:
        for y in range(int(y0 * H), int(y1 * H)):
            for x in range(int(x0 * W), int(x1 * W)):
                r, g, b, a = px[x, y]
                if max(r, g, b) < 140:
                    px[x, y] = (241, 242, 244, a)
    src.save(out("a320", "fuselage-white.png"))

def atr72():
    """Paintkit (white base + panel details + shading) + an original slate cheatline."""
    pk = os.path.join(root, "atr72/Models/paintkit.xcf")
    import subprocess, tempfile
    layers = []
    for i in range(3):
        t = tempfile.mktemp(suffix=".png")
        subprocess.run(["convert", f"{pk}[{i}]", t], check=True)
        layers.append(Image.open(t).convert("RGBA")); os.remove(t)
    img = layers[0]
    for l in layers[1:]:
        img = Image.alpha_composite(img, l)
    exk = Image.open(os.path.join(root, "atr72/Models/EXK.png")).convert("RGB").resize(img.size)
    e = exk.load(); p = img.load(); W, H = img.size
    # Cheatline: long horizontal runs of the original dark-blue band only (no logos / titles).
    for y in range(H):
        run = []
        for x in range(W + 1):
            blue = x < W and (lambda c: c[2] > 90 and c[0] < 80 and c[1] < 90)(e[x, y])
            if blue:
                run.append(x)
            else:
                if len(run) > W // 6:
                    for xx in run:
                        r, g, b, a = p[xx, y]
                        k = (r + g + b) / (3 * 255)
                        p[xx, y] = (int(52 * k), int(78 * k), int(104 * k), 255)
                run = []
    img.convert("RGB").save(out("atr72", "neutral.png"))

which = sys.argv[2:] or ["a320", "atr72"]
for w in which:
    globals()[w]()
    print("livery", w, "ok")
