import os, struct, sys
from PIL import Image
root = sys.argv[1]
n = 0

def sgi_fallback(path):
    """Minimal SGI reader for what PIL refuses (e.g. 2-channel luminance+alpha, RLE)."""
    d = open(path, "rb").read()
    magic, rle, bpc, dim, w, h, ch = struct.unpack(">hbbHHHH", d[:12])
    if magic != 474 or bpc != 1:
        raise ValueError("unsupported SGI")
    planes = []
    if rle:
        starts = struct.unpack(">%dI" % (h * ch), d[512:512 + 4 * h * ch])
        for c in range(ch):
            rows = []
            for y in range(h):
                p, row = starts[c * h + y], bytearray()
                while True:
                    b = d[p]; p += 1
                    cnt = b & 0x7F
                    if cnt == 0:
                        break
                    if b & 0x80:
                        row += d[p:p + cnt]; p += cnt
                    else:
                        row += bytes([d[p]]) * cnt; p += 1
                rows.append(bytes(row[:w]).ljust(w, b"\0"))
            planes.append(b"".join(reversed(rows)))
    else:
        for c in range(ch):
            plane = d[512 + c * w * h:512 + (c + 1) * w * h]
            planes.append(b"".join(plane[y * w:(y + 1) * w] for y in reversed(range(h))))
    mode = {1: "L", 2: "LA", 3: "RGB", 4: "RGBA"}[ch]
    bands = [Image.frombytes("L", (w, h), p) for p in planes]
    return Image.merge(mode, bands)

for dp, dn, fn in os.walk(root):
    for f in fn:
        if f.lower().endswith((".rgb", ".rgba", ".sgi", ".bw", ".dds")):
            src = os.path.join(dp, f); dst = src + ".png"
            if os.path.exists(dst): continue
            try:
                try:
                    im = Image.open(src); im.load()
                except Exception:
                    im = sgi_fallback(src)
                if im.mode not in ("RGB", "RGBA", "L", "LA"): im = im.convert("RGBA")
                im.save(dst); n += 1
            except Exception as e:
                print("FAIL", src, e)
print("converted", n)
