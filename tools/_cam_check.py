"""Experiment 13 sensor checks (not a test of results)."""
import math, time
import numpy as np
from jezero_env.env import JezeroEnv, obs_layout, CameraMap

def main():
    env = JezeroEnv(world='jezero_delta_rocks_k05_full', rock_patch=True, sensing='camera',
                    random_heading=True, spawn_jitter=10.0, segments=(0, 1, 2))
    lay = {n: (st, sh) for n, st, sh in obs_layout(rock_patch=True, sensing='camera')}
    print('obs size', env.observation_space.shape, 'layout end', sum(int(np.prod(sh)) for st, sh in lay.values()))
    obs, _ = env.reset(seed=3_000_100)
    st, sh = lay['terrain_known']; tk = obs[st:st+49].reshape(7, 7)
    st, sh = lay['fine_known']; fk = obs[st:st+81].reshape(9, 9)
    print('terrain_known (rows=left -3..3, cols=fwd -3..3):'); print(tk.astype(int))
    print('fine_known:'); print(fk.astype(int))
    # Occlusion: behind a tall wall nothing is seen.
    cam = env.cam; cam.reset()
    hm = env.heightmap; z0 = hm.z.copy()
    x, y, yaw = 0.0, 0.0, 0.0
    r, c = cam._cells(np.array([1.5]), np.array([0.0]))
    hm.z[r[0]-8:r[0]+9, c[0]-1:c[0]+2] += 2.0       # a 2 m wall 1.5 m ahead, 2 m wide
    cam.update(x, y, yaw, np.random.default_rng(0))
    _, k_front = cam.sample(np.array([1.0]), np.array([0.0]))
    _, k_behind = cam.sample(np.array([2.5]), np.array([0.0]))
    _, k_back = cam.sample(np.array([-1.0]), np.array([0.0]))
    _, k_side = cam.sample(np.array([1.0]), np.array([2.0]))   # 63 deg off axis: outside 87 deg FOV
    print('seen in front of wall', k_front[0], '| behind wall', k_behind[0], '| behind rover', k_back[0], '| 63 deg left', k_side[0])
    hm.z[:] = z0
    t = time.time()
    for _ in range(200):
        cam.update(x, y, yaw, np.random.default_rng(0))
    print(f'camera update {1000 * (time.time() - t) / 200:.2f} ms')
    env2 = JezeroEnv(world='jezero_delta_rocks_k05_full', rock_patch=True)
    for e, name in ((env2, 'perfect'), (env, 'camera')):
        e.reset(seed=3_000_101); t = time.time()
        for _ in range(100):
            e.step(np.array([0.6, 0.2]))
        print(f'{name}: {1000 * (time.time() - t) / 100:.1f} ms per env step')

if __name__ == '__main__':
    main()
