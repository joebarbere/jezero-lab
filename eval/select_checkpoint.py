"""Choose checkpoints on validation spawns (eval.compare --seed0 ... outputs).

    python3 -m eval.select_checkpoint runs/val_*.json --out runs/selection.json

Pools every validation file (one per world) per 'RUN@CKPT', ranks by goal
rate, then fewer tip-overs, then later checkpoint. Prints the table and writes
the best checkpoint per run plus the single best overall. Test spawns are
never looked at here: that is the point.
"""
import argparse
import json
from collections import defaultdict


def order(ck):
    return float('inf') if ck == 'final' else int(ck)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--out')
    args = ap.parse_args()
    stats = defaultdict(lambda: [0, 0, 0])          # run@ck -> goals, tips, episodes
    for path in args.files:
        for key, res in json.load(open(path))['runs'].items():
            for r in res['rows']:
                s = stats[key]
                s[0] += r['event'] == 'goal'; s[1] += r['event'] == 'tipped'; s[2] += 1
    runs = defaultdict(dict)
    for key, (g, t, n) in stats.items():
        run, _, ck = key.partition('@')
        runs[run][ck] = (g / n, t / n, n)
    rank = lambda item: (item[1][0], -item[1][1], order(item[0]))
    best = {}
    cks = sorted({ck for r in runs.values() for ck in r}, key=order)
    print(f"{'validation goal rate':28s}" + ''.join(f'{ck:>9s}' for ck in cks) + '   chosen')
    for run in sorted(runs):
        ck, (g, t, n) = max(runs[run].items(), key=rank)
        best[run] = {'checkpoint': ck, 'goal': g, 'tipped': t, 'episodes': n}
        print(f'{run.split("/")[-1]:28s}' + ''.join(
            f"{runs[run][c][0]:>9.0%}" if c in runs[run] else ' ' * 9 for c in cks) + f'   {ck} ({g:.0%})')
    overall = max(((run, ck, v) for run in runs for ck, v in runs[run].items()),
                  key=lambda x: (x[2][0], -x[2][1], order(x[1])))
    print(f'best overall: {overall[0]}@{overall[1]} ({overall[2][0]:.0%} goal, {overall[2][1]:.0%} tipped)')
    if args.out:
        with open(args.out, 'w') as f:
            json.dump({'per_run': best, 'overall': f'{overall[0]}@{overall[1]}'}, f, indent=1)


if __name__ == '__main__':
    main()
