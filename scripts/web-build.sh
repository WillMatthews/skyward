#!/usr/bin/env bash
# Build the browser version (wasm32 + WebGPU) into dist/ (or $OUT):
#
#   dist/index.html        page shell: WebGPU check, download progress, click to start
#   dist/credits.html      licence and attributions
#   dist/pkg/              wasm-bindgen output (skyward.js + skyward_bg.wasm)
#   dist/assets/           aircraft models + textures (downloaded when chosen),
#                          manifest.json listing them; per-aircraft CREDITS.md
#   dist/LICENSE, NOTICE   the GPL and third-party notices
#
# Needs: rustup target add wasm32-unknown-unknown; wasm-bindgen-cli matching
# Cargo.lock's wasm-bindgen (cargo install wasm-bindgen-cli --version X);
# wasm-opt (binaryen) optional — used when on PATH. Python 3 for the manifest.
#
#   scripts/web-build.sh            optimised build (Cargo profile "web")
#   PROFILE=dev scripts/web-build.sh  faster, unoptimised-ish build
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${OUT:-dist}
PROFILE=${PROFILE:-web}
TARGET_DIR=${CARGO_TARGET_DIR:-target}

want=$(grep -A1 '^name = "wasm-bindgen"$' Cargo.lock | sed -n 's/^version = "\(.*\)"/\1/p')
have=$(wasm-bindgen --version 2>/dev/null | awk '{print $2}' || true)
if [[ "$have" != "$want" ]]; then
    echo "wasm-bindgen-cli $want needed (have: ${have:-none}): cargo install wasm-bindgen-cli --version $want --locked" >&2
    exit 1
fi

echo "== cargo build ($PROFILE, wasm32-unknown-unknown)"
cargo build --locked --profile "$PROFILE" --target wasm32-unknown-unknown -p skyward
dir=$PROFILE
[[ $PROFILE == dev ]] && dir=debug
wasm="$TARGET_DIR/wasm32-unknown-unknown/$dir/skyward.wasm"

rm -rf "$OUT"
mkdir -p "$OUT/pkg" "$OUT/assets"
echo "== wasm-bindgen"
wasm-bindgen --target web --no-typescript --out-dir "$OUT/pkg" --out-name skyward "$wasm"
if command -v wasm-opt >/dev/null && [[ $PROFILE != dev ]]; then
    echo "== wasm-opt"
    wasm-opt -O2 --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext --enable-mutable-globals \
        --enable-reference-types --enable-multivalue \
        "$OUT/pkg/skyward_bg.wasm" -o "$OUT/pkg/skyward_bg.opt.wasm"
    mv "$OUT/pkg/skyward_bg.opt.wasm" "$OUT/pkg/skyward_bg.wasm"
else
    echo "(wasm-opt not found: skipping)"
fi

echo "== page + assets"
cp web/index.html web/credits.html web/worker.js LICENSE NOTICE "$OUT/"
python3 - "$OUT" <<'EOF'
import json, os, shutil, sys
out = sys.argv[1]
files = {}
for ac in sorted(os.listdir("assets/aircraft")):
    src = os.path.join("assets/aircraft", ac)
    if not os.path.isdir(src):
        continue
    dst = os.path.join(out, "assets/aircraft", ac)
    os.makedirs(dst, exist_ok=True)
    for f in sorted(os.listdir(src)):
        p = os.path.join(src, f)
        # Runtime files: the model and its textures (the JSBSim data is
        # compiled in). CREDITS.md ships too, outside the manifest.
        if f == "model.skac" or (f.startswith("t") and f.endswith((".jpg", ".png"))):
            shutil.copy(p, dst)
            files[f"aircraft/{ac}/{f}"] = os.path.getsize(p)
        elif f == "CREDITS.md":
            shutil.copy(p, dst)
for extra in ["audio/CREDITS.md", "fonts/README.md"]:
    d = os.path.join(out, "assets", os.path.dirname(extra))
    os.makedirs(d, exist_ok=True)
    shutil.copy(os.path.join("assets", extra), d)
for f in os.listdir("assets/fonts"):
    if f.endswith("-OFL.txt"):
        shutil.copy(os.path.join("assets/fonts", f), os.path.join(out, "assets/fonts"))
# A compact airport list, if a native run has made one (cache/; the page
# falls back to OurAirports' CSVs otherwise).
extras = []
# (Compact airport list and ATC frequencies, src/atc/freqs.rs; the page falls
# back to OurAirports' CSVs, which GitHub Pages serves with CORS.)
for name in ["ourairports.tsv.gz", "frequencies.tsv.gz"]:
    if os.path.isfile(os.path.join("cache", name)):
        os.makedirs(os.path.join(out, "data"), exist_ok=True)
        shutil.copy(os.path.join("cache", name), os.path.join(out, "data"))
        extras.append("data/" + name)
json.dump({"files": files, "extras": extras}, open(os.path.join(out, "assets/manifest.json"), "w"), indent=0)
# The page shows download progress against the uncompressed module size.
size = os.path.getsize(os.path.join(out, "pkg/skyward_bg.wasm"))
idx = os.path.join(out, "index.html")
html = open(idx).read().replace("__WASM_SIZE__", str(size))
open(idx, "w").write(html)
print(f"wasm {size / 1e6:.1f} MB; {len(files)} aircraft files, {sum(files.values()) / 1e6:.1f} MB")
EOF
touch "$OUT/.nojekyll"
du -sh "$OUT"
