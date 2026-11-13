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

# --- v3 heavies: national insignia, serials and unit markings painted out ----------

def _box(a, r):
    """Box blur (mean over a (2r+1)² window, edge-clamped) of an HxW[xC] array."""
    import numpy as np
    pad = [(r + 1, r)] * 2 + [(0, 0)] * (a.ndim - 2)
    c = np.pad(a, pad, mode="edge").cumsum(0).cumsum(1)
    n = 2 * r + 1
    return (c[n:, n:] - c[:-n, n:] - c[n:, :-n] + c[:-n, :-n]) / (n * n)

def scrub(img, boxes, dev=18.0, sat=0.16, grow=2, flat=False):
    """Paint out markings inside `boxes` (x0, y0, x1, y1): pixels that stand out
    from the local panel colour (luminance deviation or saturation) are replaced
    by a normalised-convolution fill from the surrounding unmarked pixels.
    `flat`: snap the fill to the region's dominant flat colours (cartoon-flat
    camouflage textures keep crisp pattern edges)."""
    import numpy as np
    a = np.asarray(img.convert("RGB")).astype(np.float64)
    H, W, _ = a.shape
    for (x0, y0, x1, y1) in boxes:
        pad = 48
        X0, Y0, X1, Y1 = max(0, x0 - pad), max(0, y0 - pad), min(W, x1 + pad), min(H, y1 + pad)
        r = a[Y0:Y1, X0:X1]
        lum = r @ [0.299, 0.587, 0.114]
        mx, mn = r.max(2), r.min(2)
        s = (mx - mn) / np.maximum(mx, 1.0)
        inside = np.zeros(lum.shape, bool)
        inside[y0 - Y0:y1 - Y0, x0 - X0:x1 - X0] = True
        # Local panel reference: median-ish (mean of the box border ring).
        ring = inside ^ np.pad(inside[4:-4, 4:-4], 4)
        ref_l = np.median(lum[~inside]) if flat else _box(np.where(inside, np.nan_to_num(lum), lum), 24)
        ref_s = np.median(s[~inside])
        mark = inside & ((np.abs(lum - ref_l) > dev) | (s > max(sat, ref_s + 0.08)))
        if flat:
            # Flat textures: everything in the box that is not one of the
            # surrounding colours is a marking.
            q = (r // 6).astype(np.int64)
            key = (q[..., 0] * 64 + q[..., 1]) * 64 + q[..., 2]
            vals, cnt = np.unique(key[~inside], return_counts=True)
            order = np.argsort(-cnt)[:3]
            keep = vals[order][cnt[order] > 0.08 * cnt.sum()]
            mark = inside & ~np.isin(key, keep)
        m = mark.astype(np.float64)
        for _ in range(grow):
            m = (_box(m, 1) > 0).astype(np.float64)
        known = 1.0 - m
        fill = r.copy()
        done = np.zeros(lum.shape, bool)
        for rad in (2, 4, 8, 16, 32, 64):
            num = _box(r * known[..., None], rad)
            den = _box(known, rad)
            ok = (m > 0) & ~done & (den > 0.35)
            fill[ok] = num[ok] / den[ok][:, None]
            done |= ok
        rest = (m > 0) & ~done
        if rest.any():
            fill[rest] = (r * known[..., None]).sum((0, 1)) / max(known.sum(), 1.0)
        if flat:
            cols = []
            for k in keep:
                sel = (key == k) & ~inside
                cols.append(r[sel].mean(0))
            cols = np.array(cols)
            d = ((fill[..., None, :] - cols[None, None]) ** 2).sum(-1)
            snapped = cols[d.argmin(-1)]
            fill = np.where((m > 0)[..., None], snapped, fill)
        r[m > 0] = fill[m > 0]
        a[Y0:Y1, X0:X1] = r
    return Image.fromarray(a.clip(0, 255).astype("uint8"))

def c130():
    """Default C-130 texture: French Air Force roundels, unit codes and fin serial removed."""
    im = Image.open(os.path.join(root, "c130/Models/texture.png"))
    im = scrub(im, [
        (1390, 72, 1485, 168), (1155, 105, 1280, 150),   # top-left fuselage side: roundel, code
        (555, 405, 650, 505), (770, 450, 890, 490),      # right side
        (275, 760, 370, 856), (1247, 757, 1342, 852),    # wing roundels
        (1130, 785, 1250, 825), (365, 1045, 485, 1085),  # wing codes
        (995, 1245, 1060, 1290), (1265, 1245, 1330, 1290),  # fin serial / code
    ])
    im.save(out("c130", "texture-neutral.png"))

def c5():
    """C-5M grey scheme with the national insignia, unit titles, fin flash and serials removed."""
    im = Image.open(os.path.join(root, "c5/Models/Liveries/436th.png")).convert("RGB")
    im = scrub(im, [
        (560, 68, 665, 160), (528, 98, 600, 118),         # fin (left view): flag, AMC, band, serials
        (836, 166, 972, 254),                             # fin (right view)
        (60, 425, 120, 482), (725, 425, 785, 482),        # wing-underside insignia
        (65, 586, 125, 642), (722, 584, 782, 640),        # wing-top insignia
        (365, 612, 482, 668),                             # template star
        (296, 726, 360, 766), (752, 735, 935, 786),       # fuselage right: insignia, titles, badges
        (95, 910, 285, 962), (668, 908, 722, 940),        # fuselage left
        (30, 790, 174, 833),                              # command title plate
    ], dev=10.0)
    im.save(out("c5", "c5-neutral.png"))

def vulcan():
    """XM607 camouflage scheme with roundels, fin flashes, serials and badges removed."""
    im = Image.open(os.path.join(root, "AvroVulcanB2/Models/liveries/XM607.png")).convert("RGB")
    im = scrub(im, [
        (115, 210, 200, 305), (312, 124, 352, 164),       # fin (left view): serial + flash, badge
        (484, 420, 560, 520), (640, 548, 684, 584),       # fin (right view)
        (700, 1105, 835, 1237), (700, 1750, 835, 1883),   # wing roundels
        (1388, 1852, 1478, 1908), (1696, 1852, 1788, 1908),  # fuselage roundels
    ], flat=True)
    im.save(out("AvroVulcanB2", "vulcan-neutral.png"))

which = sys.argv[2:] or ["a320", "atr72"]
for w in which:
    globals()[w]()
    print("livery", w, "ok")
