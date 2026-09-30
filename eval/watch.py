"""Evaluate a run's checkpoints on held-out spawns and log the results to
TensorBoard next to its training curves, with the baseline as a reference line.

    # alongside training (spare cores), until the run writes final.zip:
    jezero_env/run.sh python3 -m eval.watch runs/ppo_rocks2 --follow
    # backfill an existing run, every 500k steps:
    jezero_env/run.sh python3 -m eval.watch runs/ppo_rocks --every 500000

World and env options come from the run's config.json. Writes TensorBoard
events to runs/<name>/eval/ (shows as the run "<name>/eval"), per checkpoint
JSON to runs/<name>/eval/<steps>.json, and the baseline's results once to
runs/<name>/eval/baseline.json. Same seeds as eval/evaluate.py, so numbers are
comparable with its paired comparisons.

This answers what the final evaluation can't: whether held-out performance
peaked earlier in training (and which checkpoint to keep).
"""
import argparse
import glob
import json
import math
import os
import re
import statistics
import time
from multiprocessing import get_context

from eval.evaluate import SEED0, _episode, _init

STEP_RE = re.compile(r'ppo_(\d+)_steps\.zip$')


def summarize(rows):
    goals = [r for r in rows if r['event'] == 'goal']
    stats = [r.get('stats', {}) for r in rows]
    return {
        'success': len(goals) / len(rows),
        'tipped': sum(r['event'] == 'tipped' for r in rows) / len(rows),
        'budget': sum(r['event'] == 'budget' for r in rows) / len(rows),
        'time_to_goal_s': statistics.median(r['sim_time'] for r in goals) if goals else math.nan,
        'progress_frac': statistics.mean(s.get('progress_frac', 0.0) for s in stats),
        'stuck_s': statistics.mean(s.get('stuck_s', 0.0) for s in stats),
        'max_tilt_deg': statistics.mean(s.get('max_tilt_deg', 0.0) for s in stats),
    }


def checkpoints(run):
    found = []
    for path in glob.glob(os.path.join(run, 'checkpoints', 'ppo_*_steps.zip')):
        m = STEP_RE.search(path)
        if m:
            found.append((int(m.group(1)), path))
    return sorted(found)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', help='runs/<name>')
    ap.add_argument('--variants', nargs='+', default=['hard', 'unseen'])
    ap.add_argument('--episodes', type=int, default=8, help='per variant, per checkpoint')
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--every', type=int, default=0,
                    help='only checkpoints at least this many steps apart (0: all)')
    ap.add_argument('--follow', action='store_true',
                    help='keep watching for new checkpoints until final.zip appears')
    args = ap.parse_args()

    from torch.utils.tensorboard import SummaryWriter
    cfg = json.load(open(os.path.join(args.run, 'config.json')))
    world = cfg.get('env_kwargs', {}).get('world', cfg.get('world', 'jezero_delta'))
    rock_patch = cfg.get('env_kwargs', {}).get('rock_patch', False)
    out = os.path.join(args.run, 'eval')
    os.makedirs(out, exist_ok=True)
    writer = SummaryWriter(out)
    tasks = [(v, SEED0 + i) for v in args.variants for i in range(args.episodes)]

    ctx = get_context('spawn')
    with ctx.Pool(args.workers, initializer=_init, initargs=(None, world, rock_patch)) as pool:
        base_path = os.path.join(out, 'baseline.json')
        if os.path.exists(base_path):
            baseline = json.load(open(base_path))
        else:
            baseline = pool.map(_episode, [(v, s, 'baseline') for v, s in tasks], chunksize=1)
            json.dump(baseline, open(base_path, 'w'), indent=1)
        base = {v: summarize([r for r in baseline if r['variant'] == v]) for v in args.variants}
        print(f"{args.run} on {world}: baseline " + ', '.join(
            f"{v} {base[v]['success']:.0%}" for v in args.variants), flush=True)

        done, last = set(), -math.inf
        for f in glob.glob(os.path.join(out, '*.json')):
            name = os.path.basename(f)[:-5]
            if name.isdigit():
                done.add(int(name))
        while True:
            todo = []
            for steps, path in checkpoints(args.run):
                if steps in done:
                    last = max(last, steps)
                    continue
                if args.every and steps - last < args.every:
                    continue
                todo.append((steps, path))
                last = steps
            for steps, path in todo:
                t = time.time()
                rows = pool.map(_episode, [(v, s, 'policy', path) for v, s in tasks], chunksize=1)
                json.dump(rows, open(os.path.join(out, f'{steps}.json'), 'w'), indent=1)
                parts = []
                for v in args.variants:
                    m = summarize([r for r in rows if r['variant'] == v])
                    for k, val in m.items():
                        if not math.isnan(val):
                            writer.add_scalar(f'eval_{v}/{k}', val, steps)
                        if not math.isnan(base[v][k]):
                            writer.add_scalar(f'eval_{v}/baseline_{k}', base[v][k], steps)
                    parts.append(f"{v} {m['success']:.0%} (tipped {m['tipped']:.0%}, "
                                 f"stuck {m['stuck_s']:.0f} s)")
                writer.flush()
                done.add(steps)
                print(f'{steps:>9} steps: ' + '; '.join(parts) + f'  [{time.time() - t:.0f} s]',
                      flush=True)
            if not args.follow or os.path.exists(os.path.join(args.run, 'final.zip')):
                if args.follow and not todo:
                    break
                if not args.follow:
                    break
            time.sleep(60)
    writer.close()


if __name__ == '__main__':
    main()
