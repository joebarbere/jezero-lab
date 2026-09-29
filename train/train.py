"""Train a PPO driving policy on JezeroEnv with parallel simulators.

    jezero_env/run.sh python3 -m train.train --name ppo_v1 --steps 3000000 \
        --envs 6 --segments 0 1 2 --random-heading --spawn-jitter 10
    jezero_env/run.sh python3 -m train.train --name ppo_rocks --world jezero_delta_rocks_k05 \
        --rock-patch --goal-mode route --segments 0 1 2 --random-heading --spawn-jitter 10

Writes runs/<name>/: TensorBoard logs, checkpoints (model + VecNormalize
stats, which the policy needs at inference), and final.zip / vecnormalize.pkl.
Each simulator runs in its own process with its own GZ_PARTITION, so they don't
answer each other's gz-transport services.

    tensorboard --logdir runs    (or: podman run ... tensorboard --logdir runs --bind_all)
"""
import argparse
import functools
import json
import os
import time
from collections import Counter

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

RUNS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'runs')


def make_env(rank, seed, env_kwargs):
    # Imported in the child: each subprocess builds its own simulator.
    from jezero_env.env import JezeroEnv
    env = JezeroEnv(**env_kwargs)
    env.reset(seed=seed + rank)
    return env


class OutcomeLogger(BaseCallback):
    """Log how episodes end (goal / tipped / budget / out_of_bounds) and the
    success rate over the last `window` episodes. Replaces the 2020 ELK dashboard."""

    def __init__(self, window=100):
        super().__init__()
        self.window = window
        self.recent = []
        self.totals = Counter()

    def _on_step(self):
        for info in self.locals['infos']:
            ep = info.get('episode')
            if ep is None:
                continue
            event = info.get('event') or 'unknown'
            self.recent = (self.recent + [event])[-self.window:]
            self.totals[event] += 1
        if self.recent and self.n_calls % 50 == 0:
            counts = Counter(self.recent)
            for event in ('goal', 'tipped', 'budget', 'out_of_bounds'):
                self.logger.record(f'outcome/{event}_rate', counts[event] / len(self.recent))
            self.logger.record('outcome/episodes', sum(self.totals.values()))
        return True


class NormalizedCheckpoint(CheckpointCallback):
    """CheckpointCallback that also saves the VecNormalize statistics."""

    def _on_step(self):
        if self.n_calls % self.save_freq == 0:
            self.model.get_vec_normalize_env().save(
                os.path.join(self.save_path, f'{self.name_prefix}_vecnormalize_{self.num_timesteps}_steps.pkl'))
        return super()._on_step()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--steps', type=int, default=3_000_000)
    ap.add_argument('--envs', type=int, default=6)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--segments', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--random-heading', action='store_true')
    ap.add_argument('--spawn-jitter', type=float, default=0.0)
    ap.add_argument('--world', default='jezero_delta')
    ap.add_argument('--goal-mode', default='segments', choices=['segments', 'route'])
    ap.add_argument('--rock-patch', action='store_true',
                    help='observe the fine 0.4 m terrain patch (sees baked-in rocks)')
    ap.add_argument('--torch-threads', type=int, default=2)
    ap.add_argument('--resume', help='path to a checkpoint .zip to continue from')
    args = ap.parse_args()

    torch.set_num_threads(args.torch_threads)
    out = os.path.join(RUNS, args.name)
    os.makedirs(out, exist_ok=True)
    env_kwargs = dict(world=args.world, segments=tuple(args.segments),
                      random_heading=args.random_heading, spawn_jitter=args.spawn_jitter,
                      goal_mode=args.goal_mode, rock_patch=args.rock_patch)
    with open(os.path.join(out, 'config.json'), 'w') as f:
        json.dump({**vars(args), 'env_kwargs': env_kwargs}, f, indent=2)

    venv = SubprocVecEnv([functools.partial(make_env, i, args.seed * 1000, env_kwargs)
                          for i in range(args.envs)], start_method='spawn')
    venv = VecMonitor(venv)
    if args.resume:
        # checkpoints/ppo_<N>_steps.zip pairs with checkpoints/ppo_vecnormalize_<N>_steps.pkl
        stats = args.resume.replace('ppo_', 'ppo_vecnormalize_').replace('.zip', '.pkl')
        venv = VecNormalize.load(stats, venv)
        model = PPO.load(args.resume, env=venv, tensorboard_log=out)
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)
        model = PPO(
            'MlpPolicy', venv,
            n_steps=512, batch_size=512, n_epochs=10,
            gamma=0.99, gae_lambda=0.95, learning_rate=3e-4, clip_range=0.2, ent_coef=0.0,
            policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128])),
            tensorboard_log=out, seed=args.seed, verbose=0,
        )

    callbacks = CallbackList([
        OutcomeLogger(),
        NormalizedCheckpoint(save_freq=max(1, 100_000 // args.envs), save_path=os.path.join(out, 'checkpoints'),
                             name_prefix='ppo'),
    ])
    t = time.time()
    model.learn(total_timesteps=args.steps, callback=callbacks, tb_log_name='ppo',
                reset_num_timesteps=not args.resume, progress_bar=False)
    model.save(os.path.join(out, 'final'))
    venv.save(os.path.join(out, 'vecnormalize.pkl'))
    print(f'done: {args.steps} steps in {(time.time() - t) / 60:.1f} min -> {out}')
    venv.close()


if __name__ == '__main__':
    main()
