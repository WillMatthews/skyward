#!/usr/bin/env bash
# Fetch Environment Agency LIDAR DTM squares for the hero areas into
# cache/lidar/ ahead of time (src/dem/lidar.rs reads them; it can fetch on
# demand too, but the service rate-limits, so a first flight over a whole area
# would mostly fall back to the global DEMs).
#
#   scripts/lidar-fetch.sh [area ...]     (default: all areas below)
#   scripts/lidar-fetch.sh 52,18          (one 10 km square: E 520–530 km, N 180–190 km)
#
# Areas are ranges of 10 km National Grid squares (sx0 sx1 sy0 sy1, inclusive):
# square (sx, sy) covers eastings sx·10 km .. +10 km, northings sy·10 km .. +10 km.
# One request every GAP seconds; squares already cached are skipped; ≈ 1.5–3.5 MB
# each. Data: © Environment Agency copyright and/or database right 2022. All
# rights reserved. Open Government Licence v3.0.
set -euo pipefail
cd "$(dirname "$0")/.."

GAP=${GAP:-6}
WCS="https://environment.data.gov.uk/geoservices/datasets/13787b9a-26a4-4775-8523-806d13af58fc/wcs"
COVERAGE="13787b9a-26a4-4775-8523-806d13af58fc__Lidar_Composite_Elevation_DTM_1m"
# Keep in step with lidar::url (src/dem/lidar.rs).
url() {
	local x0=$(($1 * 10000)) y0=$(($2 * 10000))
	echo "$WCS?service=WCS&version=2.0.1&request=GetCoverage&CoverageId=$COVERAGE&format=image/tiff&subset=E($x0.0,$((x0 + 10000)).0)&subset=N($y0.0,$((y0 + 10000)).0)&scaleSize=i(1000),j(1000)&geotiff:compression=Deflate"
}

declare -A AREAS=(
	[london]="50 56 15 19"    # Greater London, EGLC, the Thames to Dartford
	[estuary]="57 61 17 19"   # Thames estuary: Gravesend to Sheppey / Southend
	[chilterns]="44 50 19 22" # the Chiltern escarpment, Aylesbury vale, High Wycombe
	[downs]="43 55 10 12"     # the South Downs, Winchester to Eastbourne
)

dir=cache/lidar
mkdir -p "$dir"
squares=()
for a in "${@:-london estuary chilterns downs}"; do
	for name in $a; do
		if [[ $name =~ ^([0-9]+),([0-9]+)$ ]]; then
			squares+=("${BASH_REMATCH[1]} ${BASH_REMATCH[2]}")
			continue
		fi
		r=${AREAS[$name]:-}
		[[ -n $r ]] || { echo "unknown area $name (${!AREAS[*]})" >&2; exit 2; }
		read -r x0 x1 y0 y1 <<<"$r"
		for ((sy = y0; sy <= y1; sy++)); do
			for ((sx = x0; sx <= x1; sx++)); do
				squares+=("$sx $sy")
			done
		done
	done
done

# The service stalls now and then (a request hangs; the next one answers):
# squares that failed get a second pass at the end.
got=0 skipped=0 failed=0
fetch() { # sx sy → 0 if cached / fetched
	local sx=$1 sy=$2 out="$dir/${1}_${2}.tif" tmp code
	if [[ -s $out ]]; then
		skipped=$((skipped + 1))
		return 0
	fi
	tmp="$out.tmp$$"
	code=$(curl -sS -m 30 -A "skyward-flightsim/0.2" -o "$tmp" -w '%{http_code} %{content_type}' "$(url "$sx" "$sy")" 2>/dev/null) || code="stalled"
	sleep "$GAP"
	if [[ $code == "200 image/tiff"* ]] && head -c 2 "$tmp" | grep -q '^II\|^MM'; then
		mv "$tmp" "$out"
		got=$((got + 1))
		echo "$sx,$sy: $(stat -c %s "$out") bytes"
		return 0
	fi
	rm -f "$tmp"
	echo "$sx,$sy: failed ($code)" >&2
	if [[ $code == 429* ]]; then
		echo "rate-limited: waiting 5 minutes" >&2
		sleep 300
	fi
	return 1
}
retry=()
for s in "${squares[@]}"; do
	# shellcheck disable=SC2086
	fetch $s || retry+=("$s")
done
for s in "${retry[@]}"; do
	# shellcheck disable=SC2086
	fetch $s || failed=$((failed + 1))
done
echo "lidar: $got fetched, $skipped cached, $failed failed (into $dir/; run again for those)"
