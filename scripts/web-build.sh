#!/usr/bin/env bash
# Build the browser version (wasm32 + WebGPU) into dist/ (or $OUT):
#
#   dist/index.html        page shell: WebGPU check, loading screen, click to start
#   dist/worker.<hash>.js  tile-builder worker (the same module)
#   dist/credits.html      licence and attributions
#   dist/pkg/              wasm-bindgen output: skyward.<hash>.js + skyward_bg.<hash>.wasm
#   dist/assets/           aircraft models + textures (downloaded when chosen),
#                          sound samples and ATC voices (downloaded when needed),
#                          manifest.json listing them; per-aircraft CREDITS.md
#   dist/data/             compact airport / frequency / navaid lists (web-data.mjs)
#   dist/thumbs/           aircraft-card thumbnails (web/thumbs/, pre-rendered)
#   dist/LICENSE, NOTICE   the GPL and third-party notices
#
# The module, its JS glue and the worker are named by content hash: GitHub
# Pages caches every file for 10 minutes, so a deploy must never pair a new
# index.html with an old module (or the reverse).
#
# Needs: rustup target add wasm32-unknown-unknown; wasm-bindgen-cli matching
# Cargo.lock's wasm-bindgen (cargo install wasm-bindgen-cli --version X);
# wasm-opt (binaryen) optional — used when on PATH; Python 3; Node 18+ for
# the compact data lists (else they are copied from cache/ if a native run
# made them, else the page falls back to OurAirports' CSVs).
#
#   scripts/web-build.sh                optimised build (Cargo profile "web")
#   PROFILE=dev scripts/web-build.sh    faster, unoptimised-ish build
#   WEB_DATA=cache scripts/web-build.sh use cache/*.tsv.gz, no downloads
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${OUT:-dist}
PROFILE=${PROFILE:-web}
TARGET_DIR=${CARGO_TARGET_DIR:-target}
WEB_DATA=${WEB_DATA:-download}
# -O2 / -O3 / -Os / -Oz come out within 1 % of each other in size (12.3–12.5
# MB, all 5.4 MB gzipped; measured 2026-11): -O3, for speed.
WASM_OPT_LEVEL=${WASM_OPT_LEVEL:--O3}

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
    echo "== wasm-opt $WASM_OPT_LEVEL"
    wasm-opt "$WASM_OPT_LEVEL" --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext --enable-mutable-globals \
        --enable-reference-types --enable-multivalue \
        "$OUT/pkg/skyward_bg.wasm" -o "$OUT/pkg/skyward_bg.opt.wasm"
    mv "$OUT/pkg/skyward_bg.opt.wasm" "$OUT/pkg/skyward_bg.wasm"
else
    echo "(wasm-opt not found: skipping)"
fi

echo "== page + assets"
cp web/index.html web/credits.html web/worker.js LICENSE NOTICE "$OUT/"
# Compact data lists: built from OurAirports with this very module (Node),
# else whatever a native run left in cache/.
if [[ $WEB_DATA == download ]] && command -v node >/dev/null; then
    echo "== data lists (OurAirports → data/*.tsv.gz)"
    node scripts/web-data.mjs "$OUT" >/dev/null || echo "(web-data failed: the page falls back to the CSVs)"
fi
python3 - "$OUT" <<'EOF'
import gzip, hashlib, json, os, re, shutil, sys
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
# Sound samples and ATC voices: downloaded when an aircraft / the radio
# needs them (audio/samples.rs, audio/radio.rs), not compiled in.
audio = 0
for d in ["audio/samples", "audio/atc"]:
    os.makedirs(os.path.join(out, "assets", d), exist_ok=True)
    for f in sorted(os.listdir(os.path.join("assets", d))):
        if f.endswith(".ogg"):
            shutil.copy(os.path.join("assets", d, f), os.path.join(out, "assets", d))
            audio += os.path.getsize(os.path.join("assets", d, f))
for extra in ["audio/CREDITS.md", "fonts/README.md"]:
    d = os.path.join(out, "assets", os.path.dirname(extra))
    os.makedirs(d, exist_ok=True)
    shutil.copy(os.path.join("assets", extra), d)
for f in os.listdir("assets/fonts"):
    if f.endswith("-OFL.txt"):
        shutil.copy(os.path.join("assets/fonts", f), os.path.join(out, "assets/fonts"))
# Compact airport / ATC frequency / navaid lists: from web-data.mjs, else
# from a native run's cache/ (the page falls back to OurAirports' CSVs,
# which GitHub Pages serves with CORS).
extras = []
for name in ["ourairports.tsv.gz", "frequencies.tsv.gz", "navaids.tsv.gz"]:
    dst = os.path.join(out, "data", name)
    if not os.path.isfile(dst) and os.path.isfile(os.path.join("cache", name)):
        os.makedirs(os.path.join(out, "data"), exist_ok=True)
        shutil.copy(os.path.join("cache", name), dst)
    if os.path.isfile(dst):
        extras.append("data/" + name)
# Aircraft-card thumbnails, pre-rendered natively (an aircraft without one
# shows a plain card):
#   SKYWARD_THUMBS_OUT=web/thumbs skyward --shot /tmp/a.png --menu-screen aircraft --no-live
thumbs = []
if os.path.isdir("web/thumbs"):
    os.makedirs(os.path.join(out, "thumbs"), exist_ok=True)
    for f in sorted(os.listdir("web/thumbs")):
        if f.endswith(".jpg"):
            shutil.copy(os.path.join("web/thumbs", f), os.path.join(out, "thumbs"))
            thumbs.append(f[:-4])
json.dump({"files": files, "extras": extras, "thumbs": thumbs}, open(os.path.join(out, "assets/manifest.json"), "w"), indent=0)

# Content-hashed names: module → glue (which names the module) → worker
# (which imports the glue) → index.html (which names all three).
def h(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:12]
def sub(path, pairs):
    s = open(path).read()
    for a, b in pairs:
        s = s.replace(a, b)
    open(path, "w").write(s)
pkg = os.path.join(out, "pkg")
wasm = f"skyward_bg.{h(os.path.join(pkg, 'skyward_bg.wasm'))}.wasm"
os.rename(os.path.join(pkg, "skyward_bg.wasm"), os.path.join(pkg, wasm))
sub(os.path.join(pkg, "skyward.js"), [("skyward_bg.wasm", wasm)])
glue = f"skyward.{h(os.path.join(pkg, 'skyward.js'))}.js"
os.rename(os.path.join(pkg, "skyward.js"), os.path.join(pkg, glue))
sub(os.path.join(out, "worker.js"), [("__GLUE__", glue)])
worker = f"worker.{h(os.path.join(out, 'worker.js'))}.js"
os.rename(os.path.join(out, "worker.js"), os.path.join(out, worker))
size = os.path.getsize(os.path.join(pkg, wasm))
# The page shows download progress against the uncompressed module size.
sub(os.path.join(out, "index.html"), [("__WASM_SIZE__", str(size)), ("__WASM__", wasm), ("__GLUE__", glue), ("__WORKER__", worker)])
left = [p for p in ["index.html", worker] if re.search(r"__[A-Z_]+__", open(os.path.join(out, p)).read())]
if left:
    sys.exit(f"unfilled placeholders in {left}")

# First-load report: what a new visitor downloads before the first frame
# (GitHub Pages gzips on the fly; sizes estimated at gzip -6).
def gz(path):
    return len(gzip.compress(open(path, "rb").read(), 6))
first = [os.path.join(out, p) for p in ["index.html", worker, "assets/manifest.json"]] + [os.path.join(pkg, glue), os.path.join(pkg, wasm)]
first += [os.path.join(out, e) for e in extras]
raw = sum(os.path.getsize(p) for p in first)
wire = sum(gz(p) for p in first)
print(f"wasm {size / 1e6:.1f} MB ({gz(os.path.join(pkg, wasm)) / 1e6:.1f} MB gzipped); first load {raw / 1e6:.1f} MB, {wire / 1e6:.1f} MB over the wire")
print(f"on demand: {len(files)} aircraft files {sum(files.values()) / 1e6:.1f} MB, audio {audio / 1e6:.1f} MB, {len(thumbs)} thumbnails; data {extras}")
EOF
touch "$OUT/.nojekyll"
du -sh "$OUT"
