"""Compare a trained policy with the hand-written baseline on identical spawns.

    jezero_env/run.sh python3 -m eval.evaluate runs/ppo_v1/final.zip --episodes 30 --workers 4
    jezero_env/run.sh python3 -m eval.evaluate - --world jezero_delta_rocks_k05   # baseline only

The same (variant, seed) gives the same spawn for both controllers: the spawn
comes from the env's seeded RNG and both controllers are deterministic.
Evaluation seeds start at 1,000,000, far from the training seeds (0-999 per env).

Variants:
  plain   segments 0-2 from the waypoint, facing the goal (baseline's home turf)
  hard    segments 0-2, random heading, 10 m spawn jitter, held-out seeds
  unseen  segment 3 (never trained on), random heading, 10 m jitter
"""
import argparse
import json
import math
import os
import pickle
import statistics
from multiprocessing import get_context

import numpy as np

VARIANTS = {
    'plain': dict(segments=(0, 1, 2), random_heading=False, spawn_jitter=0.0),
    'hard': dict(segments=(0, 1, 2), random_heading=True, spawn_jitter=10.0),
    'unseen': dict(segments=(3,), random_heading=True, spawn_jitter=10.0),
}
SEED0 = 1_000_000

_env = None
_policy = None


def _init(model_path, world, rock_patch):
    global _env, _policy
    import torch
    torch.set_num_threads(1)
    from jezero_env.env import JezeroEnv
    _env = JezeroEnv(world=world, rock_patch=rock_patch)
    if model_path:
        from stable_baselines3 import PPO
        model = PPO.load(model_path, device='cpu')
        stats = _vecnormalize_path(model_path)
        with open(stats, 'rb') as f:
            norm = pickle.load(f)
        norm.training = False

        def policy(obs):
            o = norm.normalize_obs(obs[None, :])
            return model.predict(o, deterministic=True)[0][0]
        _policy = policy


def _vecnormalize_path(model_path):
    d, name = os.path.split(model_path)
    if name == 'final.zip':
        return os.path.join(d, 'vecnormalize.pkl')
    return os.path.join(d, name.replace('ppo_', 'ppo_vecnormalize_').replace('.zip', '.pkl'))


def _episode(task):
    variant, seed, controller = task
    from jezero_env.baseline import policy as baseline_policy
    for k, v in VARIANTS[variant].items():
        setattr(_env, k, v)
    obs, info = _env.reset(seed=seed)
    act = baseline_policy if controller == 'baseline' else _policy
    total = 0.0
    while True:
        obs, r, term, trunc, step_info = _env.step(act(obs))
        total += r
        if term or trunc:
            break
    return dict(variant=variant, seed=seed, controller=controller, segment=info['segment'],
                start_dist=info['distance'], event=step_info['event'], sim_time=step_info['sim_time'],
                final_dist=step_info['distance'], ret=total)


def summarize(rows):
    out = {}
    for variant in VARIANTS:
        for ctrl in ('baseline', 'policy'):
            rs = [r for r in rows if r['variant'] == variant and r['controller'] == ctrl]
            if not rs:
                continue
            goals = [r for r in rs if r['event'] == 'goal']
            out[(variant, ctrl)] = dict(
                n=len(rs), success=len(goals) / len(rs),
                tipped=sum(r['event'] == 'tipped' for r in rs),
                out=sum(r['event'] == 'out_of_bounds' for r in rs),
                time_to_goal=statistics.median(r['sim_time'] for r in goals) if goals else math.nan,
                speed=statistics.median(r['start_dist'] / r['sim_time'] for r in goals) if goals else math.nan,
                ret=statistics.mean(r['ret'] for r in rs))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model', help="policy .zip (final.zip or a checkpoint), or '-' for baseline only")
    ap.add_argument('--world', default='jezero_delta')
    ap.add_argument('--rock-patch', action='store_true', help='env option the policy was trained with')
    ap.add_argument('--episodes', type=int, default=30, help='per variant')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--variants', nargs='+', default=list(VARIANTS))
    ap.add_argument('--out', help='write per-episode rows as JSON here')
    args = ap.parse_args()

    ctx = get_context('spawn')
    tasks = [(v, SEED0 + i) for v in args.variants for i in range(args.episodes)]
    with ctx.Pool(args.workers, initializer=_init, initargs=(None, args.world, args.rock_patch)) as pool:
        rows = pool.map(_episode, [(v, s, 'baseline') for v, s in tasks], chunksize=1)
    if args.model != '-':
        with ctx.Pool(args.workers, initializer=_init,
                      initargs=(args.model, args.world, args.rock_patch)) as pool:
            rows += pool.map(_episode, [(v, s, 'policy') for v, s in tasks], chunksize=1)

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(rows, f, indent=1)
    print(f'{args.model} on {args.world}: {args.episodes} episodes per variant, identical spawns for both')
    print(f"{'variant':8s} {'controller':9s} {'success':>8s} {'tipped':>6s} {'out':>4s} "
          f"{'t_goal(s)':>9s} {'m/s':>6s} {'return':>8s}")
    for (variant, ctrl), m in summarize(rows).items():
        print(f"{variant:8s} {ctrl:9s} {m['success']:8.0%} {m['tipped']:6d} {m['out']:4d} "
              f"{m['time_to_goal']:9.0f} {m['speed']:6.3f} {m['ret']:8.1f}")


if __name__ == '__main__':
    main()
