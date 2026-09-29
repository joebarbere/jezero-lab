#!/usr/bin/env bash
# Rebuild the jezero_delta world from source data. Run inside the jazzy-harmonic
# image (it has GDAL), from the repo root:
#
#   podman run --rm -v "$PWD:/repo:z" -w /repo jezero-lab:jazzy-harmonic \
#       terrain/build_jezero_delta.sh
#
# Downloads ~22 MB of the USGS HiRISE DTM (a 257 m window, not the 1.8 GB
# mosaic) and NASA's Perseverance waypoint file, then writes the heightmap
# model, world, and waypoint yaml into ros_ws/src/osr_gz.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p terrain/data

# Perseverance's waypoints (lat/lon/elevation per drive), from NASA's MMGIS map.
curl -fsSL -o terrain/data/M20_waypoints.json \
    https://mars.nasa.gov/mmgis-maps/M20/Layers/json/M20_waypoints.json

# The delta front, sols 437-708: centre chosen as the 256 m window along the
# traverse with the most climb that still stays on the delta (not the rim).
terrain/fetch_dtm.sh 4588296 1094113 257 terrain/data/delta_front_257.tif

python3 terrain/make_world.py terrain/data/delta_front_257.tif \
    --name jezero_delta --waypoint-sols 437 441 448 455 461
