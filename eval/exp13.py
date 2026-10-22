"""Experiment 13 (PLAN.md): learned policy vs the rock-avoiding rule, both under
camera-like sensing (JezeroEnv sensing='camera').

    jezero_env/run.sh python3 -m eval.exp13 val  --out runs/exp13_val.json
    jezero_env/run.sh python3 -m eval.exp13 test --val runs/exp13_val.json --out runs/exp13_test.json

val: on the D32 validation spawns (seed 2,000,000+, hard, 12 per training site),
camera sensing: the rock-blind baseline, the baseline grid, and the final
checkpoint of each of the six camera-trained seeds. Chooses the baseline
setting (goal rate, then fewer tip-overs, then smaller h_rock, then 'free'
before 'block') and the policy seed (goal rate, then fewer tip-overs, then
lower seed), and applies the gate (the chosen setting must beat the rock-blind
baseline).
test: 200 new spawns per test world per variant (seed 5,000,000+), both chosen
controllers under camera sensing; then, secondary, the perfect-sensing policy
(D32) and baseline (experiment 11's setting) on the hard ones.
"""
import argparse
import itertools
import json
import math
from multiprocessing import get_context

from eval.evaluate import _episode, _init
from eval.exp11 import SITES, TEST_WORLDS, mcnemar

CAMERA = {'rock_patch': True, 'sensing': 'camera'}
PERFECT = {'rock_patch': True}
GRID = [dict(h_rock=h, length=1.2, k_block=k, unknown=u)
        for h, k, u in itertools.product((0.08, 0.12, 0.16), (1.0, 3.0), ('free', 'block'))]
SEEDS = range(6)
RUN = 'runs/exp13_cam_s{}'
VAL_SEED0, VAL_EPISODES = 2_000_000, 12
TEST_SEED0, TEST_N = 5_000_000, 200
PERFECT_POLICY = 'runs/exp8_sites_full_s0/checkpoints/ppo_1200000_steps.zip'
PERFECT_AVOID = dict(h_rock=0.08, length=1.2, k_block=3.0)      # experiment 11's choice


def _run(world, obs, tasks, workers):
    with get_context('spawn').Pool(workers, initializer=_init, initargs=(None, world, obs)) as pool:
        return pool.map(_episode, tasks, chunksize=1)


def key(p):
    return f"h{p['h_rock']}_k{p['k_block']}_{p['unknown']}"


def rate(rows, event='goal'):
    return sum(r['event'] == event for r in rows) / len(rows)


def val(args):
    out = {'grid': GRID, 'rows': {}}
    for world in SITES:
        seeds = [VAL_SEED0 + i for i in range(VAL_EPISODES)]
        tasks = [('hard', s, 'baseline') for s in seeds]
        names = ['baseline'] * len(seeds)
        for p in GRID:
            tasks += [('hard', s, 'avoid', p) for s in seeds]
            names += [key(p)] * len(seeds)
        for sd in SEEDS:
            tasks += [('hard', s, 'policy', f'{RUN.format(sd)}/final.zip') for s in seeds]
            names += [f'policy_s{sd}'] * len(seeds)
        for name, r in zip(names, _run(world, CAMERA, tasks, args.workers)):
            r['world'] = world
            out['rows'].setdefault(name, []).append(r)
        print(f'validated {world}', flush=True)
    rows = out['rows']
    order = {'free': 0, 'block': 1}
    best = max(GRID, key=lambda p: (rate(rows[key(p)]), -rate(rows[key(p)], 'tipped'),
                                    -p['h_rock'], -order[p['unknown']]))
    seed = max(SEEDS, key=lambda sd: (rate(rows[f'policy_s{sd}']),
                                      -rate(rows[f'policy_s{sd}'], 'tipped'), -sd))
    out['chosen_avoid'], out['chosen_seed'] = best, seed
    out['gate'] = dict(avoid_goal=rate(rows[key(best)]), blind_goal=rate(rows['baseline']),
                       passed=rate(rows[key(best)]) > rate(rows['baseline']))
    print(f"{'controller':26s} {'goal':>6s} {'tipped':>7s}   (n={len(rows['baseline'])} each)")
    for name in ['baseline'] + sorted((k for k in rows if k != 'baseline'), key=lambda k: -rate(rows[k])):
        print(f'{name:26s} {rate(rows[name]):6.0%} {rate(rows[name], "tipped"):7.0%}')
    print(f"chosen baseline {key(best)}; chosen policy seed {seed}; gate "
          f"{'PASSED' if out['gate']['passed'] else 'FAILED'} ({out['gate']['avoid_goal']:.0%} vs "
          f"rock-blind {out['gate']['blind_goal']:.0%})")
    return out


def paired(pairs, a, b):
    """Gap (a - b) in goal rate with the frozen Wald interval and exact McNemar."""
    n = len(pairs)
    pa = sum(p[a] == 'goal' and p[b] != 'goal' for p in pairs)
    pb = sum(p[b] == 'goal' and p[a] != 'goal' for p in pairs)
    diff = (pa - pb) / n
    half = 1.96 * math.sqrt(pa + pb - (pa - pb) ** 2 / n) / n
    return dict(n=n, **{a: sum(p[a] == 'goal' for p in pairs) / n,
                        b: sum(p[b] == 'goal' for p in pairs) / n},
                a_only=pa, b_only=pb, diff=diff, ci=[diff - half, diff + half], p=mcnemar(pa, pb))


def test(args):
    v = json.load(open(args.val))
    if not v['gate']['passed']:
        raise SystemExit('validation gate failed: no test run (PLAN.md, experiment 13)')
    avoid, policy = v['chosen_avoid'], f"{RUN.format(v['chosen_seed'])}/final.zip"
    out = {'avoid': avoid, 'policy': policy, 'sets': {}}
    seeds = [TEST_SEED0 + i for i in range(args.episodes)]
    for world in TEST_WORLDS:
        for variant in ('hard', 'unseen'):
            tasks = ([(variant, s, 'avoid', avoid) for s in seeds]
                     + [(variant, s, 'policy', policy) for s in seeds])
            if variant == 'hard':                     # secondary: perfect sensing, same spawns
                cam = _run(world, CAMERA, tasks, args.workers)
                perf = _run(world, PERFECT, [(variant, s, 'avoid', PERFECT_AVOID) for s in seeds]
                            + [(variant, s, 'policy', PERFECT_POLICY) for s in seeds], args.workers)
            else:
                cam, perf = _run(world, CAMERA, tasks, args.workers), None
            n = len(seeds)
            out['sets'][f'{world}:{variant}'] = [
                dict(seed=s, avoid=cam[i]['event'], policy=cam[n + i]['event'],
                     avoid_perfect=perf[i]['event'] if perf else None,
                     policy_perfect=perf[n + i]['event'] if perf else None,
                     avoid_row=cam[i], policy_row=cam[n + i]) for i, s in enumerate(seeds)]
            print(f'tested {world} {variant}', flush=True)
    allp = [p for ps in out['sets'].values() for p in ps]
    out['primary'] = paired(allp, 'policy', 'avoid')
    hard = [p for k, ps in out['sets'].items() if k.endswith(':hard') for p in ps]
    out['secondary'] = dict(policy_cost=paired(hard, 'policy_perfect', 'policy'),
                            avoid_cost=paired(hard, 'avoid_perfect', 'avoid'))
    print(f"{'set':48s} {'n':>4s} {'avoid':>6s} {'policy':>7s}  tips avoid/policy")
    for name, ps in out['sets'].items():
        print(f"{name:48s} {len(ps):4d} {sum(p['avoid'] == 'goal' for p in ps) / len(ps):6.0%} "
              f"{sum(p['policy'] == 'goal' for p in ps) / len(ps):7.0%}  "
              f"{sum(p['avoid'] == 'tipped' for p in ps)}/{sum(p['policy'] == 'tipped' for p in ps)}")
    m = out['primary']
    print(f"PRIMARY pooled n={m['n']}: policy {m['policy']:.1%}, avoid {m['avoid']:.1%}; "
          f"policy-only {m['a_only']} / avoid-only {m['b_only']}; gap {m['diff']:+.1%} "
          f"(95% {m['ci'][0]:+.1%} to {m['ci'][1]:+.1%}), McNemar p = {m['p']:.3g}")
    for name, m in out['secondary'].items():
        a, b = [k for k in m if k.endswith('perfect')][0], name.split('_')[0]
        print(f"secondary {name} (hard, n={m['n']}): perfect {m[a]:.1%} -> camera {m[b]:.1%} "
              f"(gap {m['diff']:+.1%}, 95% {m['ci'][0]:+.1%} to {m['ci'][1]:+.1%})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=('val', 'test'))
    ap.add_argument('--val')
    ap.add_argument('--episodes', type=int, default=TEST_N, help='test: spawns per world and variant')
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--out')
    args = ap.parse_args()
    out = val(args) if args.mode == 'val' else test(args)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
