"""Feature extractors for SB3 policies on JezeroEnv observations.

TerrainCNN reads the terrain grids in the observation (coarse patch, fine patch,
look-ahead) as 2-D grids through small CNNs instead of feeding them to the MLP
as loose numbers, and the other inputs (goal, tilt, velocities, previous
action, clock, proprioception) through a small linear layer; the results are
concatenated. Selected with `train --policy cnn`.
"""
import gymnasium as gym
import torch
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch import nn

GRIDS = ('terrain', 'fine', 'lookahead')


def _grid_net(shape, channels=(16, 32), out=64):
    h, w = shape
    layers, c_in = [], 1
    for c in channels:
        # 3x3 convs, no padding: a 7x7 grid -> 5x5 -> 3x3; lookahead 5x8 -> 3x6 -> 1x4
        layers += [nn.Conv2d(c_in, c, kernel_size=3), nn.ReLU()]
        c_in, h, w = c, h - 2, w - 2
    if h < 1 or w < 1:
        raise ValueError(f'grid {shape} too small for {len(channels)} 3x3 convs')
    return nn.Sequential(*layers, nn.Flatten(), nn.Linear(c_in * h * w, out), nn.ReLU())


class TerrainCNN(BaseFeaturesExtractor):
    """layout: jezero_env.env.obs_layout(...) for the env's observation options."""

    def __init__(self, observation_space: gym.spaces.Box, layout, grid_features=64,
                 vector_features=64):
        self.layout = [(name, int(start), tuple(shape)) for name, start, shape in layout]
        total = sum(int(torch.tensor(shape).prod()) for _, _, shape in self.layout)
        if total != observation_space.shape[0]:
            raise ValueError(f'layout covers {total} values, observation has '
                             f'{observation_space.shape[0]}')
        self.vector_idx = [i for name, start, shape in self.layout if name not in GRIDS
                           for i in range(start, start + int(torch.tensor(shape).prod()))]
        grids = [(n, st, sh) for n, st, sh in self.layout if n in GRIDS]
        super().__init__(observation_space,
                         features_dim=vector_features + grid_features * len(grids))
        self.grids = grids
        self.grid_nets = nn.ModuleDict({n: _grid_net(sh, out=grid_features) for n, _, sh in grids})
        self.vector_net = nn.Sequential(nn.Linear(len(self.vector_idx), vector_features), nn.ReLU())
        self.register_buffer('_vidx', torch.tensor(self.vector_idx, dtype=torch.long))

    def forward(self, obs):
        feats = [self.vector_net(obs.index_select(1, self._vidx))]
        for name, start, shape in self.grids:
            n = shape[0] * shape[1]
            grid = obs[:, start:start + n].reshape(-1, 1, *shape)
            feats.append(self.grid_nets[name](grid))
        return torch.cat(feats, dim=1)
