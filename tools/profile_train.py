#!/usr/bin/env python3
"""Where does PPO training time go: collecting rollouts (the simulators) or
updating the network? Decides whether a GPU (e.g. ROCm on the RX 7600) could help.

    jezero_env/run.sh python3 -m tools.profile_train [iterations] [envs] [torch_threads]
"""
import functools
import sys
import time

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

from train.train import make_env


class Timer(BaseCallback):
    def __init__(self):
        super().__init__()
        self.rollout, self.update, self._t = [], [], None

    def _on_rollout_start(self):
        now = time.time()
        if self._t is not None:
            self.update.append(now - self._t)       # time since last rollout ended
        self._t = now

    def _on_rollout_end(self):
        now = time.time()
        self.rollout.append(now - self._t)
        self._t = now

    def _on_step(self):
        return True


def main():
    iters = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    envs = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    threads = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    torch.set_num_threads(threads)
    kwargs = dict(world='jezero_delta_rocks_k05', segments=(0, 1, 2), random_heading=True,
                  spawn_jitter=10.0, goal_mode='route', rock_patch=True)
    venv = VecNormalize(VecMonitor(SubprocVecEnv(
        [functools.partial(make_env, i, 0, kwargs) for i in range(envs)], start_method='spawn')))
    model = PPO('MlpPolicy', venv, n_steps=512, batch_size=512, n_epochs=10,
                policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128])), verbose=0)
    timer = Timer()
    model.learn(total_timesteps=512 * envs * (iters + 1), callback=timer)
    r, u = timer.rollout[1:], timer.update[1:]              # skip warm-up iteration
    rs, us = sum(r) / len(r), sum(u) / len(u)
    steps = 512 * envs
    print(f'{envs} envs, torch {threads} threads: rollout {rs:.2f} s ({steps / rs:.0f} steps/s), '
          f'update {us:.2f} s -> update is {us / (rs + us) * 100:.0f}% of wall time; '
          f'overall {steps / (rs + us):.0f} steps/s; ceiling if the update took zero time: '
          f'{(rs + us) / rs:.2f}x')
    venv.close()


if __name__ == '__main__':
    main()
