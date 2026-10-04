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
import csv
import functools
import json
import os
import re
import time

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

STEP_RE = re.compile(r'ppo_(\d+)_steps\.zip$')
RUNS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'runs')


def make_env(rank, seed, env_kwargs):
    # Imported in the child: each subprocess builds its own simulator.
    from jezero_env.env import JezeroEnv
    worlds = env_kwargs['world']
    if isinstance(worlds, list):   # several worlds: worker i drives worlds[i % n]
        env_kwargs = dict(env_kwargs, world=worlds[rank % len(worlds)])
    env = JezeroEnv(**env_kwargs)
    env.reset(seed=seed + rank)
    return env


def git_revision(repo=os.path.dirname(RUNS)):
    """Commit of the repo, read from .git directly (the image has no git)."""
    try:
        head = open(os.path.join(repo, '.git', 'HEAD')).read().strip()
        if not head.startswith('ref: '):
            return head
        ref = head[5:]
        path = os.path.join(repo, '.git', ref)
        if os.path.exists(path):
            return open(path).read().strip()
        for line in open(os.path.join(repo, '.git', 'packed-refs')):
            if line.strip().endswith(ref):
                return line.split()[0]
    except OSError:
        pass
    return 'unknown'


class EpisodeLogger(BaseCallback):
    """Per-episode diagnostics from JezeroEnv (info['episode_stats']):

    - TensorBoard, averaged over the last `window` episodes: outcome rates,
      episode/* (progress, stuck time, tilt, path efficiency, steering churn...)
      and reward/* (each reward component's per-episode sum, so a change to one
      term can be measured on its own). Replaces the 2020 ELK dashboard.
    - runs/<name>/episodes.csv: every episode, with where it ended, for failure
      heatmaps (eval/heatmap.py).
    - The run's git revision and config as TensorBoard text.
    """

    OUTCOMES = ('goal', 'tipped', 'budget', 'out_of_bounds', 'stuck')
    REWARD_PARTS = ('progress', 'time', 'goal', 'tipped', 'out_of_bounds', 'stuck')

    def __init__(self, out_dir, config, window=100):
        super().__init__()
        self.window = window
        self.recent = []
        self.episodes = 0
        self.csv_path = os.path.join(out_dir, 'episodes.csv')
        self.config = config
        self._csv = None

    def _on_training_start(self):
        for fmt in self.logger.output_formats:
            if isinstance(fmt, TensorBoardOutputFormat):
                fmt.writer.add_text('run/git_revision', self.config['git_revision'], 0)
                fmt.writer.add_text('run/config', '```\n' + json.dumps(self.config, indent=2) + '\n```', 0)
                fmt.writer.flush()
        new = not os.path.exists(self.csv_path)
        self._csv = open(self.csv_path, 'a', newline='')
        self._writer = csv.writer(self._csv)
        if new:
            self._writer.writerow(['timesteps', *CSV_FIELDS])

    def _on_step(self):
        for info in self.locals['infos']:
            st = info.get('episode_stats')
            if st is None:
                continue
            self.episodes += 1
            self.recent = (self.recent + [st])[-self.window:]
            self._writer.writerow([self.num_timesteps, *(st.get(f, 0.0) for f in CSV_FIELDS)])
        if self.recent and self.n_calls % 50 == 0:
            rs = self.recent
            n = len(rs)
            for ev in self.OUTCOMES:
                self.logger.record(f'outcome/{ev}_rate', sum(r['event'] == ev for r in rs) / n)
            self.logger.record('outcome/episodes', self.episodes)
            for part in self.REWARD_PARTS:
                self.logger.record(f'reward/{part}', sum(r.get('reward_' + part, 0.0) for r in rs) / n)
            for key in ('progress_frac', 'final_dist_m', 'min_dist_m', 'stuck_s', 'max_tilt_deg',
                        'path_m', 'steer_change_per_s', 'sim_time_s'):
                self.logger.record(f'episode/{key}', sum(r[key] for r in rs) / n)
            eff = [r['path_efficiency'] for r in rs if r['event'] == 'goal']
            if eff:
                self.logger.record('episode/path_efficiency_goal', sum(eff) / len(eff))
            self._csv.flush()
        return True

    def _on_training_end(self):
        if self._csv:
            self._csv.close()


CSV_FIELDS = ('event', 'segment', 'start_dist_m', 'final_dist_m', 'progress_frac', 'min_dist_m',
              'steps', 'sim_time_s', 'stuck_s', 'max_tilt_deg', 'path_m', 'steer_change_per_s',
              'end_x', 'end_y', 'reward_progress', 'reward_time', 'reward_goal', 'reward_tipped',
              'reward_out_of_bounds', 'reward_stuck')


def policy_kwargs(kind, env_kwargs):
    kwargs = dict(net_arch=dict(pi=[128, 128], vf=[128, 128]))
    if kind == 'cnn':
        from jezero_env.env import obs_layout
        from jezero_env.policies import TerrainCNN
        layout = obs_layout(**{k: env_kwargs.get(k, False)
                               for k in ('rock_patch', 'clock_obs', 'proprio', 'lookahead')})
        kwargs.update(features_extractor_class=TerrainCNN,
                      features_extractor_kwargs=dict(layout=layout))
    return kwargs


def lr_schedule(kind, lr, steps, start_steps=0):
    """SB3 learning-rate schedule. 'linear' decays from `lr` to 0 over this
    run's `steps`. SB3 passes progress_remaining over the whole run including
    a resumed checkpoint's `start_steps`, so rescale to start at `lr` here."""
    if kind == 'constant':
        return lr
    p0 = steps / (steps + start_steps)        # progress_remaining when this run starts

    def schedule(progress_remaining):
        return lr * min(1.0, progress_remaining / p0)
    return schedule


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
    ap.add_argument('--world', nargs='+', default=['jezero_delta'],
                    help='one or more worlds; with several, worker i drives world i %% n '
                         '(use --envs as a multiple of the number of worlds)')
    ap.add_argument('--goal-mode', default='segments', choices=['segments', 'route', 'map'])
    ap.add_argument('--goal-bonus', default='decay', choices=['decay', 'constant'],
                    help='decay: bonus shrinks with the budget used; constant: full bonus')
    ap.add_argument('--proprio', action='store_true',
                    help='observe proprioception (wheel speeds, slip, rates, joint angles, no-progress time)')
    ap.add_argument('--lookahead', action='store_true',
                    help='observe terrain profiles out to 12 m along rays around the goal bearing')
    ap.add_argument('--stuck-limit', type=float, default=None, metavar='SECONDS',
                    help='training-only: end with a penalty after this long without progress')
    ap.add_argument('--clock-obs', action='store_true',
                    help='observe the fraction of the step budget remaining (all runs from c0_control)')
    ap.add_argument('--rock-patch', action='store_true',
                    help='observe the fine 0.4 m terrain patch (sees baked-in rocks)')
    ap.add_argument('--torch-threads', type=int, default=2)
    ap.add_argument('--resume', help='path to a checkpoint .zip to continue from')
    ap.add_argument('--policy', default='mlp', choices=['mlp', 'cnn'],
                    help='cnn: read terrain grids through small CNNs (jezero_env.policies.TerrainCNN)')
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--lr-schedule', default='constant', choices=['constant', 'linear'],
                    help='linear: decay to 0 over this run (from the resume point, if resuming)')
    args = ap.parse_args()

    torch.set_num_threads(args.torch_threads)
    out = os.path.join(RUNS, args.name)
    os.makedirs(out, exist_ok=True)
    world = args.world[0] if len(args.world) == 1 else list(args.world)
    if isinstance(world, list) and args.envs % len(world):
        raise SystemExit(f'--envs {args.envs} is not a multiple of {len(world)} worlds')
    env_kwargs = dict(world=world, segments=tuple(args.segments),
                      random_heading=args.random_heading, spawn_jitter=args.spawn_jitter,
                      goal_mode=args.goal_mode, rock_patch=args.rock_patch,
                      clock_obs=args.clock_obs, goal_bonus=args.goal_bonus,
                      proprio=args.proprio, lookahead=args.lookahead,
                      stuck_limit_s=args.stuck_limit)
    config = {**vars(args), 'env_kwargs': env_kwargs, 'git_revision': git_revision()}
    with open(os.path.join(out, 'config.json'), 'w') as f:
        json.dump(config, f, indent=2)

    venv = SubprocVecEnv([functools.partial(make_env, i, args.seed * 1000, env_kwargs)
                          for i in range(args.envs)], start_method='spawn')
    venv = VecMonitor(venv)
    if args.resume:
        # checkpoints/ppo_<N>_steps.zip pairs with checkpoints/ppo_vecnormalize_<N>_steps.pkl
        stats = args.resume.replace('ppo_', 'ppo_vecnormalize_').replace('.zip', '.pkl')
        venv = VecNormalize.load(stats, venv)
        start = int(STEP_RE.search(args.resume).group(1))
        model = PPO.load(args.resume, env=venv, tensorboard_log=out, custom_objects={
            'learning_rate': lr_schedule(args.lr_schedule, args.lr, args.steps, start)})
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)
        model = PPO(
            'MlpPolicy', venv,
            n_steps=512, batch_size=512, n_epochs=10,
            gamma=0.99, gae_lambda=0.95, learning_rate=lr_schedule(args.lr_schedule, args.lr, args.steps),
            clip_range=0.2, ent_coef=0.0,
            policy_kwargs=policy_kwargs(args.policy, env_kwargs),
            tensorboard_log=out, seed=args.seed, verbose=0,
        )

    callbacks = CallbackList([
        EpisodeLogger(out, config),
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
