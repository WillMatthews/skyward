#!/usr/bin/env python3
"""Building facade / roof textures (src/buildmat.rs): download the ambientCG
materials listed below (CC0 1.0, https://docs.ambientcg.com/license/) and
pack them for the renderer into assets/buildings/:

  NN_name.jpg    512 x 512 albedo (sRGB), ambient occlusion baked in
  NN_name_n.jpg  256 x 256: R, G = OpenGL tangent-space normal x, y; B = roughness

Non-square sources are resampled square (the shader scales by the physical
size in buildmat.rs / scene.wgsl BT_SIZE). Resampling wraps at the edges,
so the textures stay tileable.

  python3 tools/buildtex/pack.py [download dir]     (run from the repository root)

Needs Pillow + numpy. The download dir (default target/buildtex-src) keeps
the 1K-JPG packages between runs.
"""
import io
import os
import sys
import urllib.request
import zipfile

import numpy as np
from PIL import Image

# (layer name, ambientCG id). Keep in step with buildmat::LAYERS.
LAYERS = [
    ("brick_red", "Bricks059"),
    ("brick_orange", "Bricks092"),
    ("brick_yellow", "Bricks105"),
    ("brick_brown", "Bricks097"),
    ("brick_white", "Bricks060"),
    ("stone", "Bricks084"),
    ("concrete", "Concrete046"),
    ("render", "Plaster001"),
    ("timber", "WoodSiding009"),
    ("metal", "CorrugatedSteel005"),
    ("clay_tile", "RoofingTiles014A"),
    ("clay_pantile", "RoofingTiles012A"),
    ("slate", "RoofingTiles003"),
    ("dark_tile", "RoofingTiles013A"),
    ("gravel", "Gravel022"),
    ("membrane", "Asphalt033"),
]
UA = {"User-Agent": "skyward-asset-fetch"}
OUT = "assets/buildings"


def fetch(src, aid):
    d = os.path.join(src, aid)
    if not os.path.isdir(d):
        os.makedirs(d)
        z = urllib.request.urlopen(urllib.request.Request(f"https://ambientcg.com/get?file={aid}_1K-JPG.zip", headers=UA), timeout=120).read()
        with zipfile.ZipFile(io.BytesIO(z)) as zf:
            for n in zf.namelist():
                if n.endswith(".jpg"):
                    with open(os.path.join(d, os.path.basename(n)), "wb") as f:
                        f.write(zf.read(n))
    return lambda kind: next((os.path.join(d, f) for f in os.listdir(d) if f.endswith(f"_{kind}.jpg")), None)


def wrap_resize(a, size):
    """Resample a tileable float image (h, w, c) to size x size, wrapping at the edges."""
    h, w = a.shape[:2]
    m = max(h, w) // 8
    big = np.pad(a, ((m, m), (m, m), (0, 0)), mode="wrap")
    s = size / w, size / h
    mw, mh = round(m * s[0]), round(m * s[1])
    chans = []
    for c in range(a.shape[2]):
        im = Image.fromarray(big[:, :, c].astype(np.float32), mode="F")
        chans.append(np.asarray(im.resize((size + 2 * mw, size + 2 * mh), Image.LANCZOS)))
    r = np.stack(chans, axis=2)
    return r[mh:mh + size, mw:mw + size]


def load(path, mode):
    return np.asarray(Image.open(path).convert(mode), dtype=np.float32) / 255.0


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "target/buildtex-src"
    os.makedirs(src, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    for i, (name, aid) in enumerate(LAYERS):
        get = fetch(src, aid)
        col = load(get("Color"), "RGB")
        ao = get("AmbientOcclusion")
        if ao:
            # Crevice darkening (mortar joints, tile overlaps), softened.
            col = col * (0.35 + 0.65 * load(ao, "L"))[:, :, None]
        alb = np.clip(wrap_resize(col, 512), 0, 1)
        Image.fromarray((alb * 255 + 0.5).astype(np.uint8)).save(f"{OUT}/{i:02d}_{name}.jpg", quality=88, optimize=True)
        nrm = load(get("NormalGL"), "RGB")
        rough = load(get("Roughness"), "L")
        packed = np.concatenate([nrm[:, :, :2], rough[:, :, None]], axis=2)
        n = np.clip(wrap_resize(packed, 256), 0, 1)
        Image.fromarray((n * 255 + 0.5).astype(np.uint8)).save(f"{OUT}/{i:02d}_{name}_n.jpg", quality=92, optimize=True)
        print(f"{i:02d} {name:13s} {aid}")


if __name__ == "__main__":
    main()
