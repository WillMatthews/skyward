#!/bin/sh
# Import the FlightGear AI ship models (fgdata Models/Maritime/Civilian) into
# assets/ships/<id>/ (src/ships/models.rs). Usage, from the repository root:
#
#   tools/fgimport/ships/import.sh <dir holding the fgdata Civilian files>
#
# The files come from https://gitlab.com/flightgear/fgdata (branch `next`),
# Models/Maritime/Civilian/: *.ac, *.xml and their *.png textures. Most hulls
# are the Naval Postgraduate School SAVAGE models (BSD-3, see readme.savage,
# copied to assets/ships/LICENSE-SAVAGE.txt); Tanker and SailBoatUnderPower11m
# are fgdata's own (GPL-2.0+). See assets/ships/CREDITS.md.
set -eu
src=$1
cargo build --release --manifest-path tools/fgimport/Cargo.toml
for cfg in tools/fgimport/ships/*.json; do
    id=$(basename "$cfg" .json)
    ./tools/fgimport/target/release/fgimport "$cfg" "$src" "assets/ships/$id"
    # Ships have no flight model: drop the empty report.
    rm -f "assets/ships/$id/fdm.json"
done
