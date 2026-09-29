#!/usr/bin/env bash
# Rebuild the jezero_rim world: Perseverance's climb up Jezero's western crater
# rim (sols 1244-1252, 64 m of climb in ~260 m). Same pipeline and usage as
# build_jezero_delta.sh; ~22 MB downloaded.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p terrain/data
[ -f terrain/data/M20_waypoints.json ] || curl -fsSL -o terrain/data/M20_waypoints.json \
    https://mars.nasa.gov/mmgis-maps/M20/Layers/json/M20_waypoints.json

# Centre: the 256 m window along the traverse with the most climb (see PLAN.md).
terrain/fetch_dtm.sh 4581886 1095950 257 terrain/data/rim_257.tif

python3 terrain/make_world.py terrain/data/rim_257.tif \
    --name jezero_rim --waypoint-sols 1244 1246 1247 1250 1251 1252
