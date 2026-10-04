#!/usr/bin/env bash
# More terrain: five training sites and two held-out sites along Perseverance's
# traverse (256 m HiRISE patches, rocks everywhere at k=0.05), for training a
# policy that has to generalise beyond one patch. Writes jezero_<site> and
# jezero_<site>_rocks_k05_full. Run in the container.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p terrain/data
[ -f terrain/data/M20_waypoints.json ] || curl -fsSL -o terrain/data/M20_waypoints.json \
    https://mars.nasa.gov/mmgis-maps/M20/Layers/json/M20_waypoints.json
site() {  # name cx cy sols...
    local name=$1 cx=$2 cy=$3; shift 3
    [ -f "terrain/data/${name}_257.tif" ] || terrain/fetch_dtm.sh "$cx" "$cy" 257 "terrain/data/${name}_257.tif"
    python3 terrain/make_world.py "terrain/data/${name}_257.tif" --name "jezero_$name" --waypoint-sols "$@"
    python3 terrain/add_rocks.py "jezero_$name" --k 0.05 --everywhere
}
# training sites
site floor   4590928 1093271 13 15 16 32 48 65         # crater floor near the landing site
site seitah  4590324 1092620 200 201 202 203 210        # Séítah edge
site upper   4585945 1095573 770 784 816 818 836        # upper delta
site margin  4583118 1095948 1116 1117 1118 1120 1122 1134   # margin unit
site rimwest 4579466 1094307 1431 1435 1451 1468 1470   # crater rim, west
# held-out sites (never trained on)
site heldout_upper  4585510 1095533 868 870 871 872 884
site heldout_margin 4582204 1096392 1175 1176 1177 1182 1204
