"""Where do training episodes end? Plots episode end points from a run's
episodes.csv (written by train/train.py) over the world's shaded terrain, one
panel per outcome, so failures that cluster (a scarp, a rock) show up.

    python3 -m eval.heatmap runs/ppo_rocks2 [--from-steps 1000000]

Writes runs/plots/<name>_heatmap.png. --from-steps keeps only episodes that
finished after that many training steps (the trained policy, not early
exploration).
"""
import argparse
import csv
import json
import os

import matplotlib
import numpy as np

from eval.plot_paths import draw_background, hillshade, load_world

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

PANELS = (('tipped', 'tipped over', 'Reds'), ('budget', 'ran out of time', 'Oranges'),
          ('goal', 'reached the goal', 'Greens'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--from-steps', type=int, default=0)
    ap.add_argument('--bin', type=float, default=4.0, help='histogram bin size, m')
    ap.add_argument('--out-dir', default='runs/plots')
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(args.run, 'config.json')))
    world = cfg.get('env_kwargs', {}).get('world', 'jezero_delta')
    rows = [r for r in csv.DictReader(open(os.path.join(args.run, 'episodes.csv')))
            if int(r['timesteps']) >= args.from_steps]
    meta, z = load_world(world)
    extent = meta['extent_m']
    half = extent / 2
    step = max(1, z.shape[0] // 1024)
    shade = hillshade(z[::step, ::step], extent / (z.shape[0] - 1) * step)
    bins = np.arange(-half, half + args.bin, args.bin)

    fig, axes = plt.subplots(1, len(PANELS), figsize=(6.5 * len(PANELS), 6.5))
    for ax, (event, label, cmap) in zip(axes, PANELS):
        draw_background(ax, shade, extent)
        pts = np.array([(float(r['end_x']), float(r['end_y'])) for r in rows if r['event'] == event])
        if len(pts):
            h, _, _ = np.histogram2d(pts[:, 0], pts[:, 1], bins=[bins, bins])
            h = np.ma.masked_equal(h.T, 0)
            ax.pcolormesh(bins, bins, h, cmap=cmap, alpha=0.75, shading='flat')
        wps = meta['waypoints']
        ax.plot([w['x'] for w in wps], [w['y'] for w in wps], '--', color='w', lw=1, alpha=0.7)
        ax.set_title(f'{label}: {len(pts)} of {len(rows)} episodes', fontsize=11)
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
    name = os.path.basename(os.path.normpath(args.run))
    fig.suptitle(f'{name}: where training episodes ended (from step {args.from_steps:,}; '
                 f'{args.bin:g} m bins; dashed: Perseverance route)', fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, f'{name}_heatmap.png')
    fig.savefig(out, dpi=100)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
