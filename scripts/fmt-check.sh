#!/usr/bin/env bash
# rustfmt --check the files listed in scripts/fmt-paths.txt (see there), and
# the standalone tools that are formatted. `--fix` formats them instead.
set -euo pipefail
cd "$(dirname "$0")/.."
mode=--check
[[ ${1:-} == --fix ]] && mode=
mapfile -t files < <(grep -v '^\s*\(#\|$\)' scripts/fmt-paths.txt)
# shellcheck disable=SC2086
rustfmt --edition 2024 $mode "${files[@]}"
(cd tools/regress && cargo fmt -- $mode)
echo "rustfmt: ${#files[@]} paths + tools/regress ${mode:+clean}"
