#!/usr/bin/env python3
"""Print TensorBoard scalars from a run as a compact table.
    python3 tools/tb_summary.py runs/<name> [tag ...]"""
import glob
import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

run = sys.argv[1]
tags = sys.argv[2:] or ['rollout/ep_rew_mean', 'rollout/ep_len_mean', 'outcome/goal_rate',
                        'outcome/tipped_rate', 'outcome/budget_rate', 'outcome/out_of_bounds_rate']
ea = EventAccumulator(sorted(glob.glob(f'{run}/ppo_*'))[-1], size_guidance={'scalars': 0})
ea.Reload()
avail = ea.Tags()['scalars']
series = {t: {e.step: e.value for e in ea.Scalars(t)} for t in tags if t in avail}
steps = sorted(set().union(*[set(v) for v in series.values()])) if series else []
pick = steps[:: max(1, len(steps) // 12)] + steps[-1:]
short = [t.split('/')[-1][:14] for t in series]
print(f"{'step':>9} " + ' '.join(f'{s:>14}' for s in short))
last = {}
for st in sorted(set(pick)):
    row = []
    for t, v in series.items():
        # nearest logged value at or before this step
        ks = [k for k in v if k <= st]
        if ks:
            last[t] = v[max(ks)]
        row.append(f'{last.get(t, float("nan")):>14.3f}')
    print(f'{st:>9} ' + ' '.join(row))
