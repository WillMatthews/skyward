#!/usr/bin/env bash
# Control-surface contact sheets: every aircraft, headless, in eleven poses
# (neutral, elevator up / down, roll left / right, rudder left / right, flaps,
# spoilers, gear up, gear down), seen from behind and above so the tail and
# wings show. One <id>.jpg per aircraft plus index.html.
#
#   scripts/surface-sheet.sh [out-dir]        # default target/surface-sheet
#   SHEET_VIEW="160,32,0.85,-0.12" scripts/surface-sheet.sh   # other camera
#
# Each pose is one run of the aircraft-card renderer (--menu-screen aircraft
# with SKYWARD_THUMB_VIEW / SKYWARD_SURFACES / SKYWARD_THUMBS_OUT, see
# src/main.rs), about two minutes each: ~25 min in all. Needs a GPU (no
# window opens) and python3 with Pillow for the sheets. The automated
# counterpart is `cargo test --release control_surface_checks -- --nocapture`.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=$(realpath -m "${1:-target/surface-sheet}")
VIEW=${SHEET_VIEW:-160,32,0.85,-0.12}
mkdir -p "$OUT/poses"
cargo build --release --quiet
BIN=$(cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')/release/skyward

# label|SKYWARD_SURFACES (Skyward signs: elevator + = trailing edge up,
# aileron + = right aileron up (roll right), rudder + = trailing edge right;
# gear 0 = down, 1 = up)
POSES=(
  "neutral|gear=0"
  "elevator up|elevator=1,gear=0"
  "elevator down|elevator=-1,gear=0"
  "roll left|aileron=-1,gear=0"
  "roll right|aileron=1,gear=0"
  "rudder left|rudder=-1,gear=0"
  "rudder right|rudder=1,gear=0"
  "flaps full|flaps=1,gear=0"
  "spoilers|spoilers=1,gear=0"
  "gear up|gear=1"
  "gear down|gear=0,steer=1"
)
for p in "${POSES[@]}"; do
  label=${p%%|*}; surf=${p#*|}
  dir="$OUT/poses/${label// /_}"
  mkdir -p "$dir"
  echo "pose: $label ($surf)"
  SKYWARD_NO_TRAFFIC=1 SKYWARD_THUMB_VIEW="$VIEW" SKYWARD_SURFACES="$surf" SKYWARD_THUMBS_OUT="$dir" \
    "$BIN" --shot "$dir/menu.png" --menu-screen aircraft --no-live --width 1280 --height 720 > "$dir/run.log" 2>&1 \
    || { echo "  render failed, see $dir/run.log"; exit 1; }
done

python3 -I - "$OUT" "${POSES[@]%%|*}" <<'PY'
import os, sys, html
from PIL import Image, ImageDraw, ImageFont
out, labels = sys.argv[1], sys.argv[2:]
try:
    font = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)
except OSError:
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
first = os.path.join(out, "poses", labels[0].replace(" ", "_"))
ids = sorted(f[:-4] for f in os.listdir(first) if f.endswith(".jpg"))
W, H, COLS = 480, 300, 4
rows = (len(labels) + COLS - 1) // COLS
for i in ids:
    sheet = Image.new("RGB", (COLS * W, rows * H), "black")
    d = ImageDraw.Draw(sheet)
    for k, lab in enumerate(labels):
        p = os.path.join(out, "poses", lab.replace(" ", "_"), i + ".jpg")
        x, y = (k % COLS) * W, (k // COLS) * H
        if os.path.exists(p):
            sheet.paste(Image.open(p).convert("RGB").resize((W, H)), (x, y))
        d.rectangle([x + 4, y + 4, x + 12 + d.textlength(lab, font=font), y + 30], fill="black")
        d.text((x + 8, y + 6), lab, fill=(255, 220, 80), font=font)
    sheet.save(os.path.join(out, i + ".jpg"), quality=85)
with open(os.path.join(out, "index.html"), "w") as f:
    f.write("<!doctype html><meta charset=utf-8><title>Control-surface sheets</title>"
            "<style>body{background:#111;color:#eee;font:15px sans-serif;margin:16px}img{max-width:100%}h2{margin:24px 0 6px}</style>"
            "<h1>Control-surface sheets</h1><p>" + " · ".join(f"<a href='#{i}' style='color:#fd5'>{i}</a>" for i in ids) + "</p>")
    for i in ids:
        f.write(f"<h2 id='{html.escape(i)}'>{html.escape(i)}</h2><a href='{i}.jpg'><img src='{i}.jpg' loading=lazy></a>")
print(f"{len(ids)} sheets in {out}")
PY
