import os, sys
from PIL import Image
root = sys.argv[1]
n = 0
for dp, dn, fn in os.walk(root):
    for f in fn:
        if f.lower().endswith((".rgb", ".rgba", ".sgi", ".bw", ".dds")):
            src = os.path.join(dp, f); dst = src + ".png"
            if os.path.exists(dst): continue
            try:
                im = Image.open(src); im.load()
                if im.mode not in ("RGB", "RGBA", "L", "LA"): im = im.convert("RGBA")
                im.save(dst); n += 1
            except Exception as e:
                print("FAIL", src, e)
print("converted", n)
