"""Compare arms of an experiment (several seeds each) on held-out spawns, on
one evaluation world, with the baseline on the same spawns.

    jezero_env/run.sh python3 -m eval.compare --world jezero_delta_rocks_k05_full \
        --arm control runs/bisect_a_oldsim runs/seed_old_s1 runs/seed_old_s2 \
        --arm exp1_map runs/exp1_map_s0 runs/exp1_map_s1 runs/exp1_map_s2 \
        --checkpoint 400000 --episodes 16 --out runs/compare_exp1.json

Each run's checkpoint (checkpoints/ppo_<N>_steps.zip, or final.zip with
--checkpoint final) is evaluated with the observation options from its own
config.json. Seeds for spawns are evaluate.py's held-out ones, identical for
every policy and the baseline. Reports per seed and per arm (mean, min-max).

Several --checkpoint values evaluate each run at each (results keyed
'RUN@CKPT'); a run given as RUN@CKPT uses that checkpoint whatever --checkpoint
says. --seed0 picks another range of spawns, e.g. a validation set kept apart
from the test spawns (SEED0) for choosing checkpoints.
"""
import argparse
import json
import os
import statistics
from multiprocessing import get_context

from eval.evaluate import SEED0, _episode, _init, obs_options


def ckpt_path(run, checkpoint):
    if checkpoint == 'final':
        return os.path.join(run, 'final.zip')
    return os.path.join(run, 'checkpoints', f'ppo_{checkpoint}_steps.zip')


def rate(rows, event):
    return sum(r['event'] == event for r in rows) / len(rows) if rows else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--arm', nargs='+', action='append', required=True,
                    metavar=('NAME', 'RUN'), help='arm name then its run dirs')
    ap.add_argument('--checkpoint', nargs='+', default=['400000'],
                    help="step count(s), or 'final'")
    ap.add_argument('--seed0', type=int, default=SEED0,
                    help=f'first spawn seed (default {SEED0}, the test spawns)')
    ap.add_argument('--variants', nargs='+', default=['hard', 'unseen'])
    ap.add_argument('--episodes', type=int, default=16, help='per variant')
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--out')
    args = ap.parse_args()

    tasks = [(v, args.seed0 + i) for v in args.variants for i in range(args.episodes)]
    groups = {}                                # obs options -> list of (arm, key, path)
    for arm, *runs in args.arm:
        for spec in runs:
            run, _, fixed = spec.partition('@')
            cks = [fixed] if fixed else args.checkpoint
            for ck in cks:
                path = ckpt_path(run, ck)
                if not os.path.exists(path):
                    raise SystemExit(f'missing {path}')
                name = f'{run}@{ck}' if fixed or len(cks) > 1 else run
                key = tuple(sorted(obs_options(path).items()))
                groups.setdefault(key, []).append((arm, name, path))

    ctx = get_context('spawn')
    results = {'baseline': None, 'runs': {}}
    for key, members in groups.items():
        with ctx.Pool(args.workers, initializer=_init, initargs=(None, args.world, dict(key))) as pool:
            if results['baseline'] is None:
                results['baseline'] = pool.map(_episode, [(v, s, 'baseline') for v, s in tasks], chunksize=1)
            for arm, run, path in members:
                rows = pool.map(_episode, [(v, s, 'policy', path) for v, s in tasks], chunksize=1)
                results['runs'][run] = {'arm': arm, 'rows': rows}
                print(f'evaluated {run} ({arm})', flush=True)

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(results, f, indent=1)

    print(f"\n{args.world}, checkpoint {' '.join(args.checkpoint)}, {args.episodes} spawns per variant "
          f"from seed {args.seed0}")
    base = results['baseline']
    print(f"{'':28s}" + ''.join(f'{v + " goal":>14s}{v + " tip":>12s}' for v in args.variants))
    print(f"{'baseline':28s}" + ''.join(
        f"{rate([r for r in base if r['variant'] == v], 'goal'):>14.0%}"
        f"{rate([r for r in base if r['variant'] == v], 'tipped'):>12.0%}" for v in args.variants))
    arms = {}
    for run, res in results['runs'].items():
        arms.setdefault(res['arm'], []).append((run, res['rows']))
    for arm, runs in arms.items():
        for run, rows in runs:
            print(f"  {os.path.basename(run)[-26:]:26s}" + ''.join(
                f"{rate([r for r in rows if r['variant'] == v], 'goal'):>14.0%}"
                f"{rate([r for r in rows if r['variant'] == v], 'tipped'):>12.0%}" for v in args.variants))
        line = f"{arm + ' (mean, min-max)':28s}"
        for v in args.variants:
            g = [rate([r for r in rows if r['variant'] == v], 'goal') for _, rows in runs]
            line += f"{statistics.mean(g):>6.0%} {min(g):.0%}-{max(g):.0%}".rjust(14) + ' ' * 12
        print(line)


if __name__ == '__main__':
    main()
