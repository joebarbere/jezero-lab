"""Experiment 11 (PLAN.md): the rock-avoiding baseline, tuned on validation only.

    jezero_env/run.sh python3 -m eval.exp11 val  --out runs/exp11_val.json
    jezero_env/run.sh python3 -m eval.exp11 test --val runs/exp11_val.json --out runs/exp11_test.json

val: the frozen 12-setting grid, plus the old baseline, on the D32 validation
spawns (seed 2,000,000+, hard, 12 per training site). Picks the setting by
goal rate, then fewer tip-overs, then smaller h_rock, and applies the sanity
gate (it must beat the old baseline on the same spawns).
test: the chosen setting, once, on the D32 test spawns of the three test
worlds, paired with the committed policy and old-baseline rows from
runs/test_exp9_*.json.
"""
import argparse
import itertools
import json
import math
from multiprocessing import get_context

from eval.evaluate import _episode, _init

SITES = ('jezero_delta_rocks_k05_full', 'jezero_floor_rocks_k05_full',
         'jezero_seitah_rocks_k05_full', 'jezero_upper_rocks_k05_full',
         'jezero_margin_rocks_k05_full', 'jezero_rimwest_rocks_k05_full')
TEST_WORLDS = ('jezero_delta_rocks_k05_full', 'jezero_heldout_upper_rocks_k05_full',
               'jezero_heldout_margin_rocks_k05_full')
GRID = [dict(h_rock=h, length=L, k_block=k)
        for h, L, k in itertools.product((0.08, 0.12, 0.16), (1.2, 1.6), (1.0, 3.0))]
VAL_SEED0, VAL_EPISODES = 2_000_000, 12
POLICY = 'runs/exp8_sites_full_s0@1200000'
OBS = {'rock_patch': True}            # the chosen policy's observation options
DELTA_FRESH = 1_000_016               # delta: only the 32 spawns never used for a decision


def _run(world, tasks, workers):
    with get_context('spawn').Pool(workers, initializer=_init, initargs=(None, world, OBS)) as pool:
        return pool.map(_episode, tasks, chunksize=1)


def _rate(rows, event='goal'):
    return sum(r['event'] == event for r in rows) / len(rows)


def key(p):
    return f"h{p['h_rock']}_L{p['length']}_k{p['k_block']}"


def val(args):
    out = {'grid': GRID, 'rows': {}}
    for world in args.worlds:
        tasks = [('hard', VAL_SEED0 + i, 'baseline') for i in range(args.episodes)]
        tasks += [('hard', VAL_SEED0 + i, 'avoid', p) for p in GRID for i in range(args.episodes)]
        rows = _run(world, tasks, args.workers)
        for r, t in zip(rows, tasks):
            r['world'] = world
            out['rows'].setdefault('baseline' if t[2] == 'baseline' else key(t[3]), []).append(r)
        print(f'validated {world}', flush=True)
    table = sorted(((_rate(rs), -_rate(rs, 'tipped'), -GRID[[key(p) for p in GRID].index(k)]['h_rock'], k)
                    for k, rs in out['rows'].items() if k != 'baseline'), reverse=True)
    base = out['rows']['baseline']
    print(f"{'setting':22s} {'goal':>6s} {'tipped':>7s}   (n={len(base)} per setting)")
    print(f"{'old baseline':22s} {_rate(base):6.0%} {_rate(base, 'tipped'):7.0%}")
    for g, t, _, k in table:
        print(f'{k:22s} {g:6.0%} {-t:7.0%}')
    best = table[0][3]
    out['chosen'] = GRID[[key(p) for p in GRID].index(best)]
    out['gate'] = dict(chosen_goal=_rate(out['rows'][best]), old_goal=_rate(base),
                       passed=_rate(out['rows'][best]) > _rate(base))
    print(f"chosen {best}: gate {'PASSED' if out['gate']['passed'] else 'FAILED'} "
          f"({out['gate']['chosen_goal']:.0%} vs old baseline {out['gate']['old_goal']:.0%})")
    return out


def mcnemar(a_only, b_only):
    """Exact two-sided McNemar p from the discordant counts."""
    n, k = a_only + b_only, min(a_only, b_only)
    if n == 0:
        return 1.0
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def test(args):
    v = json.load(open(args.val))
    if not v['gate']['passed']:
        raise SystemExit('validation gate failed: no test run (PLAN.md, experiment 11)')
    params = v['chosen']
    out = {'chosen': params, 'sets': {}}
    for world in TEST_WORLDS:
        ref = json.load(open(f'runs/test_exp9_{world}.json'))
        pol = {(r['variant'], r['seed']): r for r in ref['runs'][POLICY]['rows']}
        old = {(r['variant'], r['seed']): r for r in ref['baseline']}
        seed_min = DELTA_FRESH if world.startswith('jezero_delta') else 0
        spawns = sorted(k for k in pol if k[1] >= seed_min)
        rows = _run(world, [(vr, s, 'avoid', params) for vr, s in spawns], args.workers)
        for variant in ('hard', 'unseen'):
            idx = [i for i, k in enumerate(spawns) if k[0] == variant]
            out['sets'][f'{world}:{variant}'] = [
                dict(variant=variant, seed=spawns[i][1], avoid=rows[i]['event'],
                     policy=pol[spawns[i]]['event'], old=old[spawns[i]]['event'],
                     avoid_row=rows[i]) for i in idx]
        print(f'tested {world}', flush=True)
    print(f"{'set':48s} {'n':>3s} {'old':>5s} {'avoid':>6s} {'policy':>7s}  tips old/avoid/policy")
    pooled = []
    for name, rs in out['sets'].items():
        pooled += rs
        g = {c: sum(r[c] == 'goal' for r in rs) / len(rs) for c in ('old', 'avoid', 'policy')}
        t = {c: sum(r[c] == 'tipped' for r in rs) for c in ('old', 'avoid', 'policy')}
        print(f"{name:48s} {len(rs):3d} {g['old']:5.0%} {g['avoid']:6.0%} {g['policy']:7.0%}  "
              f"{t['old']}/{t['avoid']}/{t['policy']}")
    p_only = sum(r['policy'] == 'goal' and r['avoid'] != 'goal' for r in pooled)
    a_only = sum(r['avoid'] == 'goal' and r['policy'] != 'goal' for r in pooled)
    out['pooled'] = dict(n=len(pooled),
                         policy=sum(r['policy'] == 'goal' for r in pooled) / len(pooled),
                         avoid=sum(r['avoid'] == 'goal' for r in pooled) / len(pooled),
                         old=sum(r['old'] == 'goal' for r in pooled) / len(pooled),
                         policy_only=p_only, avoid_only=a_only, p=mcnemar(p_only, a_only))
    m = out['pooled']
    print(f"pooled n={m['n']}: policy {m['policy']:.0%}, avoid {m['avoid']:.0%}, old {m['old']:.0%}; "
          f"discordant policy-only {p_only} / avoid-only {a_only}, McNemar p = {m['p']:.3g}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=('val', 'test', 'smoke'))
    ap.add_argument('--val', help='test: the val output')
    ap.add_argument('--worlds', nargs='+', default=list(SITES))
    ap.add_argument('--episodes', type=int, default=VAL_EPISODES)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--out')
    args = ap.parse_args()
    if args.mode == 'smoke':        # spawns outside both validation and test (seed 3,000,000+)
        p = GRID[len(GRID) // 2]
        rows = _run(args.worlds[0], [('hard', 3_000_000 + i, 'avoid', p) for i in range(args.episodes)]
                    + [('hard', 3_000_000 + i, 'baseline') for i in range(args.episodes)], args.workers)
        for r in rows:
            print(r['controller'], r['event'], f"{r['sim_time']:.0f}s", f"{r['final_dist']:.1f}m")
        return
    out = val(args) if args.mode == 'val' else test(args)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
