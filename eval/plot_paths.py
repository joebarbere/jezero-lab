"""Plot policy and baseline paths from eval/evaluate.py --out over the world's
terrain and rocks.

    python3 -m eval.plot_paths runs/eval_ppo_rocks_k05.json --world jezero_delta_rocks_k05

Writes runs/plots/<json name>_<variant>.png (one panel per held-out spawn,
zoomed to that episode) and <json name>_overview.png (every path on the map).
"""
import argparse
import json
import math
import os

import matplotlib
import numpy as np
import yaml
from osgeo import gdal

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

gdal.UseExceptions()
OSR_GZ = os.environ.get('OSR_GZ_SHARE', '/osr_ws/install/osr_gz/share/osr_gz')
STYLE = {'baseline': dict(color='#1f77b4', label='baseline'),
         'policy': dict(color='#ff7f0e', label='policy')}
END = {'goal': ('o', 'reached goal'), 'tipped': ('X', 'tipped'),
       'budget': ('s', 'out of time'), 'out_of_bounds': ('^', 'left map')}


def load_world(world):
    meta = yaml.safe_load(open(os.path.join(OSR_GZ, 'worlds', world + '.yaml')))
    png = os.path.join(OSR_GZ, 'models', world, 'materials', 'textures', 'heightmap.png')
    z = gdal.Open(png).ReadAsArray().astype(np.float64) / 65535 * meta['relief_m']
    return meta, z


def hillshade(z, pixel, az=315, alt=45):
    gy, gx = np.gradient(z, pixel)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    a, e = math.radians(az), math.radians(alt)
    return np.clip(math.sin(e) * np.cos(slope) + math.cos(e) * np.sin(slope) * np.cos(a - aspect), 0, 1)


def draw_background(ax, shade, extent):
    half = extent / 2
    ax.imshow(shade, cmap='gray', extent=(-half, half, -half, half), origin='upper', vmin=0, vmax=1)


def draw_episode(ax, row):
    st = STYLE[row['controller']]
    p = np.array(row['path'])
    ax.plot(p[:, 0], p[:, 1], '-', color=st['color'], lw=1.6, alpha=0.9)
    marker, _ = END.get(row['event'], ('.', row['event']))
    ax.plot(p[-1, 0], p[-1, 1], marker, color=st['color'], ms=8, mec='k', mew=0.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('results')
    ap.add_argument('--world', required=True)
    ap.add_argument('--out-dir', default='runs/plots')
    args = ap.parse_args()

    rows = json.load(open(args.results))
    meta, z = load_world(args.world)
    extent = meta['extent_m']
    step = max(1, z.shape[0] // 1024)                 # ~25 cm for display
    shade = hillshade(z[::step, ::step], extent / (z.shape[0] - 1) * step)
    base = os.path.join(args.out_dir, os.path.splitext(os.path.basename(args.results))[0])
    os.makedirs(args.out_dir, exist_ok=True)

    legend = [plt.Line2D([], [], color=s['color'], lw=2, label=s['label']) for s in STYLE.values()]
    legend += [plt.Line2D([], [], ls='', marker=m, color='w', mec='k', label=l) for m, l in END.values()]
    legend += [plt.Line2D([], [], ls='', marker='*', color='#2ca02c', mec='k', ms=11, label='goal'),
               plt.Line2D([], [], ls='', marker='D', color='w', mec='k', label='start')]

    for variant in sorted({r['variant'] for r in rows}):
        eps = sorted({r['seed'] for r in rows if r['variant'] == variant})
        cols = 4
        fig, axes = plt.subplots(math.ceil(len(eps) / cols), cols, figsize=(4 * cols, 4 * math.ceil(len(eps) / cols)))
        for ax, seed in zip(np.ravel(axes), eps):
            pair = [r for r in rows if r['variant'] == variant and r['seed'] == seed]
            draw_background(ax, shade, extent)
            pts = np.vstack([np.array(r['path']) for r in pair] + [np.array([pair[0]['goal']])])
            lo, hi = pts.min(0) - 6, pts.max(0) + 6
            span = max(hi - lo)
            mid = (lo + hi) / 2
            ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
            ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
            for r in pair:
                draw_episode(ax, r)
            ax.plot(*pair[0]['path'][0], 'D', color='w', mec='k', ms=7)
            ax.plot(*pair[0]['goal'], '*', color='#2ca02c', mec='k', ms=14)
            outcome = ', '.join(f"{r['controller'][:4]}: {r['event']}" for r in sorted(pair, key=lambda r: r['controller']))
            ax.set_title(f"seed {seed - 1_000_000}: {outcome}", fontsize=9)
            ax.tick_params(labelsize=7)
        for ax in np.ravel(axes)[len(eps):]:
            ax.axis('off')
        fig.legend(handles=legend, loc='lower center', ncol=len(legend), fontsize=9, frameon=False)
        fig.suptitle(f"{args.world}, variant '{variant}': baseline vs policy on identical spawns "
                     f"(x/y in m, rocks baked into the shaded terrain)", fontsize=11)
        fig.tight_layout(rect=(0, 0.04, 1, 0.97))
        fig.savefig(f'{base}_{variant}.png', dpi=110)
        plt.close(fig)
        print(f'wrote {base}_{variant}.png')

    fig, ax = plt.subplots(figsize=(10, 10))
    draw_background(ax, shade, extent)
    wps = meta['waypoints']
    ax.plot([w['x'] for w in wps], [w['y'] for w in wps], '--', color='w', lw=1, alpha=0.7)
    for w in wps:
        ax.annotate(f"sol {w['sol']}", (w['x'], w['y']), color='w', fontsize=9,
                    xytext=(4, 4), textcoords='offset points')
    for r in rows:
        draw_episode(ax, r)
    ax.legend(handles=legend[:len(STYLE) + len(END)], loc='lower left', fontsize=9)
    ax.set_title(f"{args.world}: all evaluation paths (dashed: Perseverance's route)")
    fig.tight_layout()
    fig.savefig(f'{base}_overview.png', dpi=110)
    print(f'wrote {base}_overview.png')


if __name__ == '__main__':
    main()
