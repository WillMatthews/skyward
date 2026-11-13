#!/usr/bin/env bash
# Multiplayer smoke test on loopback, headless (nothing on screen, no sound):
# a server on 127.0.0.1 and two clients, "Alpha" and "Bravo", that start
# together (--mp-wait 2) in formation; Alpha's capture shows Bravo's aircraft
# (real model, name tag, on the map), Bravo's chat line and the synced time /
# weather.
#
#   scripts/mp-loopback.sh [OUT_DIR] [ALPHA_AIRCRAFT] [BRAVO_AIRCRAFT] [extra Alpha args...]
#
# SKYWARD_MP_UI (see main.rs) opens the player list / quick-chat wheel /
# chat / spectator view for Alpha's capture. Release binaries are used from
# the cargo target directory.
set -euo pipefail
cd "$(dirname "$0")/.."
out=${1:-target/mp-loopback}
alpha_ac=${2:-kestrel}
bravo_ac=${3:-kestrel}
shift $(( $# > 3 ? 3 : $# ))
mkdir -p "$out"
cargo build --release --quiet -p skyward -p skyward-server
target=$(cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')
bin="$target/release"
port=${MP_PORT:-47850}
metar="EGLC 131420Z 24008KT 9999 FEW040 16/09 Q1018"

timeout -s TERM 240 "$bin/skyward-server" --bind 127.0.0.1 --port "$port" --name "Loopback test" \
    --motd "Smoke test server" --airport EGLC --region London --time 14:30 --weather "$metar" \
    --no-console > "$out/server.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null || true' EXIT
sleep 0.5
if ! kill -0 $server 2>/dev/null; then
    cat "$out/server.log" >&2
    exit 1
fi

# Bravo: 70 m ahead (east), 12 m left, 10 m up; says hello 4 s in; flies on after Alpha's shot.
timeout 230 "$bin/skyward" --shot "$out/bravo.png" --aircraft "$bravo_ac" \
    --start-at air:51.505108:0.061010:510:90 --connect "127.0.0.1:$port" --name Bravo \
    --mp-wait 2 --seconds 30 --ap --mp-chat "4:Hello from Bravo! Join up on my wing." \
    --no-live --mute > "$out/bravo.log" 2>&1 &
bravo=$!

timeout 220 "$bin/skyward" --shot "$out/alpha.png" --aircraft "$alpha_ac" \
    --start-at air:51.505:0.06:500:90 --connect "127.0.0.1:$port" --name Alpha \
    --mp-wait 2 --seconds 12 --ap --no-live --mute --cam chase "$@" > "$out/alpha.log" 2>&1
status=$?
wait $bravo || true
kill $server 2>/dev/null || true
wait $server 2>/dev/null || true
grep -h "multiplayer:" "$out/alpha.log" "$out/bravo.log" || true
echo "server log:"
cat "$out/server.log"
echo "wrote $out/alpha.png, $out/bravo.png"
exit $status
