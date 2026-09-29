"""Hand-written go-to-goal controller: turn toward the goal, drive at full
speed, slow down when badly misaligned. The number to beat in Phase 4.

    jezero_env/run.sh python3 -m jezero_env.baseline [segments...]"""
import math
import sys
import time

import numpy as np

from jezero_env.env import JezeroEnv


def policy(obs):
    s, c = obs[3], obs[4]                       # heading error sin/cos
    err = math.atan2(s, c)
    turn = float(np.clip(2.0 * err, -1, 1))
    speed = float(np.clip(math.cos(err), 0.2, 1.0))
    return np.array([speed, turn], dtype=np.float32)


def run(env, seed):
    obs, info = env.reset(seed=seed)
    total, t = 0.0, time.time()
    while True:
        obs, r, term, trunc, step_info = env.step(policy(obs))
        total += r
        if term or trunc:
            break
    return info, step_info, total, env.steps, time.time() - t


def main():
    segments = tuple(int(a) for a in sys.argv[1:]) or (0, 1, 2, 3)
    for seg in segments:
        env = JezeroEnv(segments=(seg,))
        info, end, total, steps, wall = run(env, seed=seg)
        wp = env.waypoints
        print(f"segment {seg} (sol {wp[seg]['sol']} -> {wp[seg + 1]['sol']}, "
              f"{info['distance']:.1f} m): {end['event']:>13s} after {steps}/{info['budget']} steps "
              f"({end['sim_time']:.0f} sim-s, {wall:.1f} s wall), final dist {end['distance']:.2f} m, "
              f"return {total:.1f}")


if __name__ == '__main__':
    main()
