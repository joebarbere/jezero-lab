#!/usr/bin/env python3
"""Turn a DTM crop into a Gazebo Harmonic world: 16-bit heightmap PNG, a model
wrapping it, and a world with Mars gravity and waypoints from Perseverance's
real traverse.

    python3 terrain/make_world.py terrain/data/delta_front_257.tif \
        --name jezero_delta --waypoint-sols 437 441 448 455 461

Writes ros_ws/src/osr_gz/models/<name>/ and ros_ws/src/osr_gz/worlds/<name>.sdf,
plus <name>.yaml next to the world with spawn pose and waypoints in world
coordinates.

World frame: the heightmap is centred on the origin, x east, y north, z up,
and z = 0 at the lowest point of the patch. Waypoint positions come from NASA's
M20 waypoint file (terrain/data/M20_waypoints.json), converted from lat/lon to
the DTM's equirectangular projection (x = R*lon, y = R*lat, R = 3396190 m).
"""
import argparse
import json
import math
import os

import numpy as np
from osgeo import gdal

gdal.UseExceptions()

MARS_R = 3396190.0
MARS_G = 3.721
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'ros_ws', 'src', 'osr_gz')


def load_dtm(path):
    ds = gdal.Open(path)
    z = ds.ReadAsArray().astype(np.float64)
    nodata = ds.GetRasterBand(1).GetNoDataValue()
    if nodata is not None and np.any(z == nodata):
        raise SystemExit(f'{path}: contains nodata cells; crop elsewhere')
    n_rows, n_cols = z.shape
    if n_rows != n_cols or (n_cols - 1) & (n_cols - 2):
        raise SystemExit(f'{path}: need a square 2^n+1 grid, got {n_cols}x{n_rows}')
    return z, ds.GetGeoTransform()


def waypoints(sols, gt, n):
    """First NASA waypoint of each requested sol, as (sol, col, row)."""
    path = os.path.join(REPO, 'terrain', 'data', 'M20_waypoints.json')
    first = {}
    for ft in json.load(open(path))['features']:
        p = ft['properties']
        if p['sol'] not in sols:
            continue
        key = (p.get('site') or 0, p.get('drive') or 0)
        if p['sol'] in first and first[p['sol']][0] <= key:
            continue
        col = (MARS_R * math.radians(p['lon']) - gt[0]) / gt[1]
        row = (MARS_R * math.radians(p['lat']) - gt[3]) / gt[5]
        if not (0 <= col < n and 0 <= row < n):
            raise SystemExit(f"sol {p['sol']} waypoint is outside the patch")
        first[p['sol']] = (key, col, row)
    missing = set(sols) - set(first)
    if missing:
        raise SystemExit(f'no waypoints for sols {sorted(missing)}')
    return [(s, first[s][1], first[s][2]) for s in sols]


def bilinear(z, col, row):
    c0, r0 = int(col), int(row)
    fc, fr = col - c0, row - r0
    c1, r1 = min(c0 + 1, z.shape[1] - 1), min(r0 + 1, z.shape[0] - 1)
    return ((1 - fr) * ((1 - fc) * z[r0, c0] + fc * z[r0, c1])
            + fr * ((1 - fc) * z[r1, c0] + fc * z[r1, c1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dtm')
    ap.add_argument('--name', required=True)
    ap.add_argument('--waypoint-sols', type=int, nargs='+', required=True)
    args = ap.parse_args()

    z, gt = load_dtm(args.dtm)
    n = z.shape[0]
    pixel = gt[1]
    extent = (n - 1) * pixel            # metres between first and last sample
    zmin, zmax = z.min(), z.max()
    relief = zmax - zmin

    # 16-bit heightmap: 0 = lowest point, 65535 = highest (~1 mm steps for 65 m).
    h16 = np.round((z - zmin) / relief * 65535).astype(np.uint16)
    model_dir = os.path.join(PKG, 'models', args.name)
    os.makedirs(os.path.join(model_dir, 'materials', 'textures'), exist_ok=True)
    png = os.path.join(model_dir, 'materials', 'textures', 'heightmap.png')
    mem = gdal.GetDriverByName('MEM').Create('', n, n, 1, gdal.GDT_UInt16)
    mem.GetRasterBand(1).WriteArray(h16)
    gdal.GetDriverByName('PNG').CreateCopy(png, mem)
    # Textures: regolith-coloured noise and a flat normal map. Gazebo's stock
    # terrain textures aren't installed with the Harmonic packages.
    rng = np.random.default_rng(0)
    noise = rng.normal(0, 1, (256, 256))
    rgb = np.clip(np.stack([184, 118, 78])[None, None, :] + noise[..., None] * 14, 0, 255)
    flat = np.broadcast_to(np.array([128, 128, 255]), (16, 16, 3))
    for fname, img in (('diffuse.png', rgb), ('normal.png', flat)):
        mem = gdal.GetDriverByName('MEM').Create('', img.shape[1], img.shape[0], 3, gdal.GDT_Byte)
        for i in range(3):
            mem.GetRasterBand(i + 1).WriteArray(np.ascontiguousarray(img[:, :, i]).astype(np.uint8))
        gdal.GetDriverByName('PNG').CreateCopy(
            os.path.join(model_dir, 'materials', 'textures', fname), mem)
    for f in os.listdir(os.path.join(model_dir, 'materials', 'textures')):
        if f.endswith('.aux.xml'):
            os.remove(os.path.join(model_dir, 'materials', 'textures', f))

    def world_xy(col, row):
        # Image column -> east, image row -> south. Heightmap centred on origin.
        return (col * pixel - extent / 2, extent / 2 - row * pixel)

    wps = []
    for sol, col, row in waypoints(args.waypoint_sols, gt, n):
        x, y = world_xy(col, row)
        wps.append({'sol': sol, 'x': round(x, 2), 'y': round(y, 2),
                    'z': round(bilinear(z, col, row) - zmin, 2),
                    'dtm_col': round(col, 1), 'dtm_row': round(row, 1)})

    size = f'{extent} {extent} {relief:.3f}'
    heightmap = f"""<heightmap>
            <uri>model://{args.name}/materials/textures/heightmap.png</uri>
            <size>{size}</size>
            <pos>0 0 0</pos>"""
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{args.name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    {extent:.0f} m x {extent:.0f} m of Jezero crater, {relief:.1f} m relief, from the USGS
    Mars 2020 TRN HiRISE DTM mosaic (1 m/px, doi:10.5066/P9REJ9JN).
    Generated by terrain/make_world.py from {os.path.basename(args.dtm)}.
  </description>
</model>
""")
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<!-- Generated by terrain/make_world.py; do not edit by hand. -->
<sdf version="1.9">
  <model name="{args.name}">
    <static>true</static>
    <link name="terrain">
      <collision name="collision">
        <geometry>
          {heightmap}
          </heightmap>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          {heightmap}
            <texture>
              <diffuse>model://{args.name}/materials/textures/diffuse.png</diffuse>
              <normal>model://{args.name}/materials/textures/normal.png</normal>
              <size>4</size>
            </texture>
          </heightmap>
        </geometry>
        <material>
          <ambient>0.60 0.38 0.25 1</ambient>
          <diffuse>0.76 0.50 0.33 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")

    markers = '\n'.join(f"""    <model name="waypoint_sol{w['sol']}">
      <static>true</static>
      <pose>{w['x']} {w['y']} {w['z'] + 1.0} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder><radius>0.05</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>0.1 0.6 1 1</ambient><diffuse>0.1 0.6 1 1</diffuse></material>
        </visual>
      </link>
    </model>""" for w in wps)

    world = os.path.join(PKG, 'worlds', f'{args.name}.sdf')
    with open(world, 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<!-- Generated by terrain/make_world.py; do not edit by hand.
     {extent:.0f} m square of Jezero crater, Mars gravity, waypoints (blue posts,
     visual only) at Perseverance's positions on sols {' '.join(str(w['sol']) for w in wps)}. -->
<sdf version="1.9">
  <world name="{args.name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <gravity>0 0 -{MARS_G}</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.5 0.4 0.35 1</ambient>
      <background>0.85 0.65 0.50 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.85 0.8 1</diffuse>
      <direction>-0.5 0.3 -0.8</direction>
    </light>

    <include><uri>model://{args.name}</uri></include>

{markers}
  </world>
</sdf>
""")

    meta = os.path.join(PKG, 'worlds', f'{args.name}.yaml')
    start, goal = wps[0], wps[-1]
    yaw = math.atan2(wps[1]['y'] - start['y'], wps[1]['x'] - start['x'])
    with open(meta, 'w') as f:
        f.write(f"""# Generated by terrain/make_world.py. World-frame metres, z = 0 at the lowest point.
source: {os.path.basename(args.dtm)}
extent_m: {extent}
relief_m: {relief:.3f}
dtm_elevation_min_m: {zmin:.3f}   # geoid-referenced, for converting back
gravity: {MARS_G}
spawn: {{x: {start['x']}, y: {start['y']}, z: {start['z']}, yaw: {yaw:.3f}}}   # sol {start['sol']}, facing the next waypoint
waypoints:
""")
        for w in wps:
            f.write(f"  - {{sol: {w['sol']}, x: {w['x']}, y: {w['y']}, z: {w['z']}}}\n")

    print(f'{args.name}: {n}x{n} px, {extent:.0f} m, relief {relief:.1f} m')
    for w in wps:
        print(f"  sol {w['sol']}: ({w['x']:7.2f}, {w['y']:7.2f}, {w['z']:5.2f})")
    print(f'wrote {world}, {meta}, {model_dir}/')


if __name__ == '__main__':
    main()
