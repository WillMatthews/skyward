#!/usr/bin/env bash
# Golden-image and audio regression (local: needs a GPU and the tile cache).
#
#   scripts/regress.sh                 # render every scene, compare with tests/goldens/
#   scripts/regress.sh chase_day       # only these scenes
#   scripts/regress.sh --update [..]   # accept the current output as the new goldens
#
# Scenes live in tests/goldens/scenes.txt. Output, diff heat-maps and a summary
# go to target/regress/. Exit status 1 if any scene regressed.
# Run from a quiet machine: the renders use the GPU, ~15 s per image scene.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
GOLD=tests/goldens
OUT=${REGRESS_OUT:-target/regress}
UPDATE=0
ONLY=()
for a in "$@"; do
    case $a in
        --update) UPDATE=1 ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) ONLY+=("$a") ;;
    esac
done
mkdir -p "$OUT"

echo "building skyward (release) and the comparison tool..."
cargo build --release --quiet
cargo build --release --quiet --manifest-path tools/regress/Cargo.toml
TARGET=$(cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')
SKYWARD=$TARGET/release/skyward
REGRESS_TARGET=$(cd tools/regress && cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')
REGRESS=$REGRESS_TARGET/release/regress

fail=0
summary=()
trim() { local s=$1; s=${s#"${s%%[![:space:]]*}"}; echo "${s%"${s##*[![:space:]]}"}"; }
while IFS='|' read -r name kind envs args opts; do
    name=$(trim "$name")
    [[ -z $name || $name == \#* ]] && continue
    if ((${#ONLY[@]})) && [[ ! " ${ONLY[*]} " == *" $name "* ]]; then continue; fi
    kind=$(trim "$kind"); envs=$(trim "$envs"); args=$(trim "$args"); opts=$(trim "${opts:-}")
    case $kind in
        image) ext=png; mode=(--shot "$OUT/$name.png" --width 960 --height 540 --no-live) ;;
        audio) ext=wav; mode=(--audio-render "$OUT/$name.wav" --no-live) ;;
        *) echo "$name: unknown kind '$kind'"; fail=1; continue ;;
    esac
    printf '%-16s ' "$name"
    # shellcheck disable=SC2086
    if ! eval "env $envs \"$SKYWARD\" \"\${mode[@]}\" $args" >"$OUT/$name.log" 2>&1; then
        echo "FAILED to run (see $OUT/$name.log)"; fail=1; summary+=("$name: run failed"); continue
    fi
    if ((UPDATE)); then
        if [[ $kind == audio ]]; then
            "$REGRESS" pack "$OUT/$name.$ext" "$GOLD/$name.$ext"
        else
            cp "$OUT/$name.$ext" "$GOLD/$name.$ext"
        fi
        echo "updated $GOLD/$name.$ext"
        continue
    fi
    if [[ ! -f $GOLD/$name.$ext ]]; then
        echo "no golden yet (run with --update)"; fail=1; summary+=("$name: missing golden"); continue
    fi
    extra=()
    [[ $kind == image ]] && extra=(--diff "$OUT/$name.diff.png")
    # shellcheck disable=SC2086
    if res=$("$REGRESS" "$kind" "$GOLD/$name.$ext" "$OUT/$name.$ext" "${extra[@]}" $opts); then
        echo "$res"
    else
        echo "$res"; fail=1; summary+=("$name: $res")
    fi
done < "$GOLD/scenes.txt"

if ((fail)); then
    echo; echo "REGRESSIONS:"; printf '  %s\n' "${summary[@]}"
    echo "Look at $OUT/*.png / *.diff.png; if the change is intended: scripts/regress.sh --update <scene>"
    exit 1
fi
((UPDATE)) || echo "all scenes within tolerance"
