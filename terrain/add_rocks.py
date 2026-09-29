#!/usr/bin/env python3
"""Scatter rocks along a world's waypoint route, sized by the Golombek-Rapp Mars
rock-abundance model, and write a new world.

    python3 terrain/add_rocks.py jezero_delta --k 0.05 --seed 1

writes ros_ws/src/osr_gz/worlds/jezero_delta_rocks_k05.{sdf,yaml}.

Golombek & Rapp (1997): the cumulative fractional area covered by rocks of
diameter >= D is F(D) = k exp(-q D), q = 1.79 + 0.152/k, where k is the total
rock abundance (Mars landing sites: roughly 0.03-0.2). The number of rocks per
m^2 with diameter in [D, D + dD] is then k q exp(-q D) / (pi D^2 / 4) dD.

A 1 m DTM can't resolve rocks, so these are synthetic, but sized and spaced like
Mars. Rocks are spheres centred at ground level (so they stand D/2 tall), placed
within `--corridor` metres of the route. Nothing is placed within `--clear`
metres of a waypoint, so spawns and goals stay reachable.

Rocks are baked into the terrain, not added as models: the heightmap is
upsampled to --resolution px (2049 = 12.5 cm) and each rock's dome
sqrt(r^2 - d^2) is added on top. Heightfield collision only tests cells near the
rover, so thousands of rocks cost almost nothing, whereas 2,544 rock models ran
at 0.4x real time (one merged model with 2,544 spheres: 1.6x) vs 12.6x without.
The trade-off: a rock is a dome sampled at 12.5 cm, not a sphere with vertical
sides. The rock list still goes into the yaml, for spawn clearance.
"""
import argparse
import math
import os
import re

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'ros_ws', 'src', 'osr_gz')


def expected_counts(k, area, d_min, d_max, bins=400):
    q = 1.79 + 0.152 / k
    d = np.linspace(d_min, d_max, bins + 1)
    mid = (d[:-1] + d[1:]) / 2
    n = k * q * np.exp(-q * mid) / (math.pi * mid ** 2 / 4) * np.diff(d) * area
    return mid, n


def dist_to_polyline(x, y, pts):
    best = np.full(np.shape(x), np.inf)
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        vx, vy = bx - ax, by - ay
        t = np.clip(((x - ax) * vx + (y - ay) * vy) / (vx * vx + vy * vy), 0, 1)
        best = np.minimum(best, np.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('--k', type=float, default=0.05, help='rock abundance (fraction of area)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--d-min', type=float, default=0.2, help='m; smaller rocks go under a wheel')
    ap.add_argument('--d-max', type=float, default=1.5)
    ap.add_argument('--corridor', type=float, default=20.0, help='m either side of the route')
    ap.add_argument('--clear', type=float, default=2.0, help='m kept clear around waypoints')
    ap.add_argument('--resolution', type=int, default=2049, help='heightmap px per side (2^n+1)')
    args = ap.parse_args()

    base = os.path.join(PKG, 'worlds', args.world)
    meta = yaml.safe_load(open(base + '.yaml'))
    sdf = open(base + '.sdf').read()
    wps = [(w['x'], w['y']) for w in meta['waypoints']]
    half = meta['extent_m'] / 2 - 2.0

    # Heightmap, for placing rocks on the ground.
    from osgeo import gdal
    gdal.UseExceptions()
    png = os.path.join(PKG, 'models', args.world, 'materials', 'textures', 'heightmap.png')
    hz = gdal.Open(png).ReadAsArray().astype(float) / 65535 * meta['relief_m']
    n = hz.shape[0]
    pix = meta['extent_m'] / (n - 1)

    def ground(x, y):
        c = np.clip((x + meta['extent_m'] / 2) / pix, 0, n - 1.001)
        r = np.clip((meta['extent_m'] / 2 - y) / pix, 0, n - 1.001)
        c0, r0 = c.astype(int), r.astype(int)
        fc, fr = c - c0, r - r0
        return ((1 - fr) * ((1 - fc) * hz[r0, c0] + fc * hz[r0, c0 + 1])
                + fr * ((1 - fc) * hz[r0 + 1, c0] + fc * hz[r0 + 1, c0 + 1]))

    # Area of the corridor, by Monte Carlo over the map.
    rng = np.random.default_rng(args.seed)
    probe = rng.uniform(-half, half, (200_000, 2))
    in_corr = dist_to_polyline(probe[:, 0], probe[:, 1], wps) <= args.corridor
    area = in_corr.mean() * (2 * half) ** 2

    mids, counts = expected_counts(args.k, area, args.d_min, args.d_max)
    rocks = []
    for d, lam in zip(mids, counts):
        for _ in range(rng.poisson(lam)):
            for _ in range(50):
                x, y = rng.uniform(-half, half, 2)
                if dist_to_polyline(np.array(x), np.array(y), wps) > args.corridor:
                    continue
                if min(math.hypot(x - wx, y - wy) for wx, wy in wps) < args.clear + d / 2:
                    continue
                rocks.append((float(x), float(y), float(ground(np.array(x), np.array(y))), float(d)))
                break
    rocks.sort(key=lambda r: -r[3])

    name = f"{args.world}_rocks_k{round(args.k * 100):02d}"
    ext = meta['extent_m']
    res = args.resolution
    if (res - 1) & (res - 2):
        raise SystemExit('--resolution must be 2^n+1')
    fine_pix = ext / (res - 1)
    coords = np.arange(res) * fine_pix - ext / 2           # world x for columns
    X, Y = np.meshgrid(coords, -coords)                     # row 0 = north (+y)
    z = ground(X.ravel(), Y.ravel()).reshape(res, res)      # bilinear upsample
    for x, y, zg, d in rocks:
        r = d / 2
        c0 = max(int((x - r + ext / 2) / fine_pix), 0)
        c1 = min(int((x + r + ext / 2) / fine_pix) + 2, res)
        r0 = max(int((ext / 2 - (y + r)) / fine_pix), 0)
        r1 = min(int((ext / 2 - (y - r)) / fine_pix) + 2, res)
        sx, sy = X[r0:r1, c0:c1], Y[r0:r1, c0:c1]
        cap = np.sqrt(np.clip(r * r - (sx - x) ** 2 - (sy - y) ** 2, 0, None))
        z[r0:r1, c0:c1] = np.maximum(z[r0:r1, c0:c1], ground(sx.ravel(), sy.ravel()).reshape(sx.shape) + cap)
    zmin, relief = z.min(), z.max() - z.min()

    base_model = os.path.join(PKG, 'models', args.world)
    model_dir = os.path.join(PKG, 'models', name)
    tex = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(tex, exist_ok=True)
    h16 = np.round((z - zmin) / relief * 65535).astype(np.uint16)
    mem = gdal.GetDriverByName('MEM').Create('', res, res, 1, gdal.GDT_UInt16)
    mem.GetRasterBand(1).WriteArray(h16)
    gdal.GetDriverByName('PNG').CreateCopy(os.path.join(tex, 'heightmap.png'), mem)
    import shutil
    for f in ('diffuse.png', 'normal.png'):
        shutil.copy(os.path.join(base_model, 'materials', 'textures', f), tex)
    for f in os.listdir(tex):
        if f.endswith('.aux.xml'):
            os.remove(os.path.join(tex, f))
    for f in ('model.sdf', 'model.config'):
        text = open(os.path.join(base_model, f)).read()
        text = text.replace(f'model://{args.world}/', f'model://{name}/')
        text = text.replace(f'<name>{args.world}</name>', f'<name>{name}</name>')
        text = text.replace(f'<model name="{args.world}">', f'<model name="{name}">')
        text = re.sub(r'<size>[^<]*</size>', f'<size>{ext} {ext} {relief:.3f}</size>', text, count=2)
        open(os.path.join(model_dir, f), 'w').write(text)

    # z = 0 moved down by any rock below the old minimum (zmin <= 0 here); shift
    # waypoint heights to match.
    dz = float(-zmin)
    out_sdf = sdf.replace(f'<world name="{args.world}">', f'<world name="{name}">', 1)
    out_sdf = out_sdf.replace(f'<uri>model://{args.world}</uri>', f'<uri>model://{name}</uri>', 1)
    out_sdf = re.sub(r'(<model name="waypoint_sol\d+">\s*<static>true</static>\s*<pose>\S+ \S+ )(\S+)',
                     lambda m: m.group(1) + f'{float(m.group(2)) + dz:.2f}', out_sdf)
    with open(os.path.join(PKG, 'worlds', name + '.sdf'), 'w') as f:
        f.write(out_sdf)

    meta = dict(meta)
    meta['source'] = f"{meta['source']} + add_rocks.py k={args.k} seed={args.seed}"
    meta['relief_m'] = round(float(relief), 3)
    meta['spawn'] = {**meta['spawn'], 'z': round(meta['spawn']['z'] + dz, 3)}
    meta['waypoints'] = [{**w, 'z': round(w['z'] + dz, 3)} for w in meta['waypoints']]
    meta['rocks'] = [dict(x=round(x, 3), y=round(y, 3), d=round(d, 3)) for x, y, _, d in rocks]
    with open(os.path.join(PKG, 'worlds', name + '.yaml'), 'w') as f:
        f.write(f'# Generated by terrain/add_rocks.py from {args.world}. {len(rocks)} rocks, '
                f'baked into a {res}x{res} heightmap.\n')
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None, width=120)

    ds = np.array([r[3] for r in rocks])
    cover = sum(math.pi * d * d / 4 for d in ds) / area
    print(f'{name}: {len(rocks)} rocks in {area:.0f} m^2 of corridor; '
          f'diameter median {np.median(ds):.2f} m, max {ds.max():.2f} m; '
          f'>=0.3 m: {int(np.sum(ds >= 0.3))}, >=0.5 m: {int(np.sum(ds >= 0.5))}; '
          f'area covered {cover * 100:.1f}% (model F(0.2 m) = {args.k * math.exp(-(1.79 + 0.152 / args.k) * args.d_min) * 100:.1f}%)')


if __name__ == '__main__':
    main()
