#!/usr/bin/env bash
# Crop a square patch out of the USGS Mars 2020 HiRISE DTM mosaic (1 m/px,
# Jezero crater) without downloading the whole 1.8 GB file: GDAL reads only the
# rows it needs over HTTP range requests (/vsicurl/).
#
#   terrain/fetch_dtm.sh <centre_x> <centre_y> <size_px> <out.tif>
#
# Coordinates are in the DTM's projection: equirectangular on the Mars 2000
# sphere (R = 3396190 m), i.e. x = R*lon_rad, y = R*lat_rad.
#
# Source: Fergason et al. 2020, Mars 2020 Terrain Relative Navigation HiRISE DTM
# Mosaic, USGS Astrogeology, doi:10.5066/P9REJ9JN.
set -euo pipefail
url="/vsicurl/https://planetarymaps.usgs.gov/mosaic/mars2020_trn/HiRISE/JEZ_hirise_soc_006_DTM_MOLAtopography_DeltaGeoid_1m_Eqc_latTs0_lon0_blend40.tif"
cx="$1"; cy="$2"; n="$3"; out="$4"
# Pixel edges are on whole metres, so snap the window to them.
half=$(( n / 2 ))
ulx=$(( ${cx%.*} - half )); uly=$(( ${cy%.*} + half + 1 ))
gdal_translate -q -projwin "$ulx" "$uly" $(( ulx + n )) $(( uly - n )) "$url" "$out"
gdalinfo -stats "$out" | grep -E 'Size is|Origin|STATISTICS_(MIN|MAX|MEAN)'
