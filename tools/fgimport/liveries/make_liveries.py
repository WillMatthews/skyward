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
    # Propeller sheet: the spinning disc / blur (lower-right quadrant, a 70 %
    # transparent material) gets an alpha channel so the disc stays a faint
    # shimmer: as see-through glass at full coverage it caught the sky's
    # Fresnel reflection edge-on and read as a white band across the wing.
    pr = Image.open(os.path.join(root, "c130/Models/Propellers/propeller.png")).convert("RGBA")
    W, H = pr.size
    al = Image.new("L", (W, H), 255)
    al.paste(70, (int(0.555 * W), int(0.555 * H), W, H))
    pr.putalpha(al)
    pr.save(out("c130", "propeller-disc.png"))

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

def _uv_up(ac_path, texname, size):
    """Per texel of `texname` in an AC3D model: 0 = unused, else 1 + 254·(n_up + 1)/2
    from the up component of the face normal using it (AC3D: y up). Smaller UV
    triangles win where islands overlap; whole-texture placeholder UVs are skipped."""
    import numpy as np
    from PIL import ImageDraw
    lines = open(ac_path, errors="replace").read().split("\n")
    tris, tex, verts, i = [], None, None, 0
    while i < len(lines):
        t = lines[i].split()
        if t and t[0] == "OBJECT":
            tex, verts = None, None
        elif t and t[0] == "texture":
            tex = t[1].strip('"')
        elif t and t[0] == "numvert":
            n = int(t[1])
            verts = np.array([[float(v) for v in lines[i + 1 + j].split()[:3]] for j in range(n)])
            i += n
        elif t and t[0] == "refs":
            n = int(t[1])
            refs = [lines[i + 1 + j].split() for j in range(n)]
            i += n
            if tex and texname in tex and n >= 3 and verts is not None:
                p = verts[[int(r[0]) for r in refs]]
                uv = [(float(r[1]), float(r[2])) for r in refs]
                for a in range(1, n - 1):
                    q = p[[0, a, a + 1]]
                    nrm = np.cross(q[1] - q[0], q[2] - q[0])
                    ln = np.linalg.norm(nrm)
                    tuv = [uv[0], uv[a], uv[a + 1]]
                    area = abs((tuv[1][0] - tuv[0][0]) * (tuv[2][1] - tuv[0][1]) - (tuv[2][0] - tuv[0][0]) * (tuv[1][1] - tuv[0][1])) / 2
                    if ln > 1e-12 and area < 0.2:
                        tris.append((area, tuv, nrm[1] / ln))
        i += 1
    img = Image.new("L", (size, size), 0)
    d = ImageDraw.Draw(img)
    for _, tuv, up in sorted(tris, key=lambda t: -t[0]):
        d.polygon([(u * size, (1 - v) * size) for u, v in tuv], fill=int(1 + 254 * (up + 1) / 2))
    return np.asarray(img)

def vulcan():
    """XM607 with roundels, fin flashes, serials and badges removed, repainted in the
    1970s-80s RAF strike scheme: Dark Green / Dark Sea Grey disruptive camouflage on
    the upper surfaces and sides (the source's light-grey camo patches read as white
    blotches in daylight), Light Aircraft Grey undersides, a light weathering mottle."""
    import numpy as np
    im = Image.open(os.path.join(root, "AvroVulcanB2/Models/liveries/XM607.png")).convert("RGB")
    im = scrub(im, [
        (115, 210, 200, 305), (312, 124, 352, 164),       # fin (left view): serial + flash, badge
        (484, 420, 560, 520), (640, 548, 684, 584),       # fin (right view)
        (700, 1105, 835, 1237), (700, 1750, 835, 1883),   # wing roundels
        (1388, 1852, 1478, 1908), (1696, 1852, 1788, 1908),  # fuselage roundels
    ], flat=True)
    a = np.asarray(im).astype(np.float64)
    H, W, _ = a.shape
    up = _uv_up(os.path.join(root, "AvroVulcanB2/Models/vulcanb2.ac"), "XM597", W).astype(np.float64)
    used = up > 0
    n_up = (up - 1) / 127.0 - 1.0
    # Grow the islands' classification a few texels over their seams (mip / filter bleed).
    def grow(v, m, r):
        num, den = _box(v * m, r), _box(m.astype(np.float64), r)
        return np.where(m, v, np.where(den > 0, num / np.maximum(den, 1e-9), 0.0)), den > 0
    n_up, used = grow(n_up, used, 3)
    light = (np.abs(a - [156, 160, 168]).max(2) < 18)
    green = (np.abs(a - [76, 72, 48]).max(2) < 18)
    upper = used & (n_up > -0.35)
    dsg = np.array([90, 97, 101], np.float64)   # BS 381C 638 Dark Sea Grey
    lag = np.array([166, 171, 170], np.float64) # BS 381C 627 Light Aircraft Grey
    a[light & upper] = dsg
    a[light & ~upper] = lag
    # Weathering: low-frequency mottle (±4 %) + faint fine grain on the paint.
    rng = np.random.default_rng(607)
    mott = _box(_box(rng.standard_normal((H, W)), 24), 24)
    mott /= max(np.abs(mott).max(), 1e-9)
    grain = _box(rng.standard_normal((H, W)), 1)
    k = 1.0 + 0.04 * mott + 0.012 * grain / max(np.abs(grain).max(), 1e-9)
    paint = light | green
    a[paint] *= k[paint][:, None]
    Image.fromarray(a.clip(0, 255).astype("uint8")).save(out("AvroVulcanB2", "vulcan-neutral.png"))

def ec130():
    """EC130 B4 FlightGear demo scheme with the type lettering and web address
    removed (flat paint: each box is filled with the paint colour around it)."""
    from PIL import ImageDraw
    im = Image.open(os.path.join(root, "ec130/Models/default.png")).convert("RGB")
    d = ImageDraw.Draw(im)
    for (x0, y0, x1, y1), probe in [
        ((704, 100, 1116, 179), (1150, 140)),   # www.FlightGear.org (upper side)
        ((760, 179, 1116, 194), (1150, 140)),   # (its descenders, clear of the stripe tip)
        ((712, 622, 1116, 700), (1150, 660)),   # (lower side)
        ((702, 640, 712, 700), (1150, 660)),
        ((385, 250, 620, 310), (650, 280)),     # "EC 130" in the blue band
        ((385, 495, 620, 560), (650, 525)),
    ]:
        d.rectangle((x0, y0, x1, y1), fill=im.getpixel(probe))
    im.save(out("ec130", "default-neutral.png"))

def concorde():
    """Concorde (FGAddon, G-BOAC scheme): airline titles, speedmarques, tail
    flag, registration and fleet letters painted out — a plain white
    Concorde. The paint swatches and wheel / intake details are kept."""
    from PIL import ImageDraw
    im = Image.open(os.path.join(root, "concorde/Models/concorde.png")).convert("RGB")
    W, H = im.size
    k = W / 1024.0
    box = lambda x0, y0, x1, y1: tuple(int(v * k) for v in (x0, y0, x1, y1))
    d = ImageDraw.Draw(im)
    # Registration, fleet letters and the small flag / serial plate: flat white.
    for b in [(55, 0, 225, 50), (290, 5, 340, 40), (412, 5, 515, 45)]:
        d.rectangle(box(*b), fill=(255, 255, 255))
    # Tail flag (the fin's own island, white base): flat white.
    d.rectangle(box(15, 430, 350, 610), fill=(255, 255, 255))
    # Titles and speedmarques (with their grey drop shadows) on the fuselage
    # sides: repainted white, except the door outlines (light grey, kept
    # where the paint is not saturated) and the black door sills.
    import numpy as np
    a = np.asarray(im).astype(np.float64)
    mx, mn = a.max(2), a.min(2)
    sat = (mx - mn) / np.maximum(mx, 1.0)
    doors = np.zeros(mx.shape, bool)
    for (x0, y0, x1, y1) in [box(462, 102, 513, 188), box(575, 270, 626, 384)]:
        doors[y0:y1, x0:x1] = True
    paint = np.zeros(mx.shape, bool)
    for (x0, y0, x1, y1) in [box(130, 118, 415, 166), box(612, 316, 902, 366), box(348, 88, 608, 180), box(412, 286, 672, 376)]:
        paint[y0:y1, x0:x1] = True
    sill = mx < 60
    marked = paint & ~sill & np.where(doors, sat > 0.12, (mx < 254) | (sat > 0.02))
    a[marked] = 255.0
    # Door interiors: the speedmarque shadow crossed them; clear them and
    # redraw the outlines (the first door's top edge was under the band).
    for (x0, y0, x1, y1) in [box(468, 108, 506, 180), box(581, 277, 620, 377)]:
        a[y0:y1, x0:x1] = 255.0
    im = Image.fromarray(a.astype("uint8"))
    d = ImageDraw.Draw(im)
    for b in [(464, 104, 510, 184), (577, 273, 623, 381)]:
        d.rounded_rectangle(box(*b), radius=int(6 * k), outline=(168, 168, 170), width=max(2, int(3 * k)))
    im.save(out("concorde", "concorde-neutral.png"))

def dc3():
    """DC-3 default texture (a 1950s airline scheme): the airline titles and the
    emblems on the nose and fin are painted out (scanline fill between the
    paint either side, which carries the cheatline straight through), and the
    blue cheatline / fin sweep is repainted in an original oxblood with a gold
    pin stripe, so no operator's livery remains."""
    import numpy as np
    a = np.asarray(Image.open(os.path.join(root, "dc3/Models/texture.png")).convert("RGB")).astype(np.float64)
    for (x0, y0, x1, y1), rows in [
        ((636, 296, 1018, 337), True), ((1036, 659, 1418, 700), True),   # fuselage titles
        ((404, 350, 506, 450), True), ((1536, 710, 1638, 810), True),    # nose emblems
        ((1750, 162, 1804, 216), False), ((236, 522, 290, 580), False),  # fin emblems
    ]:
        if rows:
            l, r = a[y0:y1, x0 - 3][:, None, :], a[y0:y1, x1 + 2][:, None, :]
            t = np.linspace(0.0, 1.0, x1 - x0)[None, :, None]
        else:
            l, r = a[y0 - 3, x0:x1][None, :, :], a[y1 + 2, x0:x1][None, :, :]
            t = np.linspace(0.0, 1.0, y1 - y0)[:, None, None]
        f = l * (1.0 - t) + r * t
        # Snap to the paint on either side (white or cheatline): a clean edge
        # running straight across the box instead of a blended smear.
        lum = lambda c: c @ [0.299, 0.587, 0.114]
        fl, ll, rl = lum(f), lum(l * np.ones_like(f)), lum(r * np.ones_like(f))
        bright = np.where((ll > rl)[..., None], l * np.ones_like(f), r * np.ones_like(f))
        darkc = np.where((ll > rl)[..., None], r * np.ones_like(f), l * np.ones_like(f))
        mid = 0.5 * (ll + rl)
        snap = np.where((np.abs(ll - rl) > 40.0)[..., None], np.where((fl > mid)[..., None], bright, darkc), f)
        a[y0:y1, x0:x1] = snap
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    mx, mn = a.max(2), a.min(2)
    sat = (mx - mn) / np.maximum(mx, 1.0)
    blue = (b > r + 25) & (b > g + 10) & (sat > 0.25)
    lum = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    dark = blue & (lum < 0.42)
    light = blue & ~dark
    ox = np.array([112.0, 24.0, 32.0])     # oxblood cheatline
    gold = np.array([206.0, 172.0, 104.0])  # pin stripe
    a = np.where(dark[..., None], np.clip(ox * np.clip(lum / 0.22, 0.4, 1.8)[..., None], 0, 255), a)
    a = np.where(light[..., None], np.clip(gold * np.clip(lum / 0.6, 0.6, 1.3)[..., None], 0, 255), a)
    Image.fromarray(a.astype("uint8")).save(out("dc3", "texture-neutral.png"))

which = sys.argv[2:] or ["a320", "atr72"]
for w in which:
    globals()[w]()
    print("livery", w, "ok")
