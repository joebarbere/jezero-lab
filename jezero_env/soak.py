"""Soak test: a random agent for N episodes, reporting memory and timing so a
leak or a hang shows up. Phase 3's done-criterion is 1,000 clean episodes.

    jezero_env/run.sh python3 -m jezero_env.soak [episodes] [max_episode_seconds]"""
import resource
import sys
import time
from collections import Counter

from jezero_env.env import JezeroEnv


def rss_mb():
    with open('/proc/self/status') as f:
        for line in f:
            if line.startswith('VmRSS:'):
                return int(line.split()[1]) / 1024
    return float('nan')


def main():
    episodes = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    cap = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    env = JezeroEnv(segments=(0, 1, 2, 3), random_heading=True, max_episode_seconds=cap)
    events, steps_total, t_start = Counter(), 0, time.time()
    worst_reset = 0.0
    respawns, settle = 0, []
    rss0 = rss_mb()
    print(f'start: RSS {rss0:.0f} MB', flush=True)
    for ep in range(1, episodes + 1):
        t = time.time()
        _, info = env.reset(seed=ep)
        worst_reset = max(worst_reset, time.time() - t)
        respawns += info['spawn_attempts'] - 1
        settle.append(info['settle_time'])
        while True:
            _, _, term, trunc, info = env.step(env.action_space.sample())
            steps_total += 1
            if term or trunc:
                events[info['event']] += 1
                break
        if ep % 100 == 0:
            el = time.time() - t_start
            print(f'ep {ep:5d}: RSS {rss_mb():6.0f} MB (+{rss_mb() - rss0:5.0f}), '
                  f'{steps_total / el:5.0f} env steps/s, worst reset {worst_reset * 1000:4.0f} ms, '
                  f'{dict(events)}', flush=True)
            worst_reset = 0.0
    settle.sort()
    print(f'done: {episodes} episodes, {steps_total} steps in {time.time() - t_start:.0f} s; '
          f'peak RSS {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024:.0f} MB; '
          f'{respawns} respawns; settle median {settle[len(settle) // 2]:.1f} s, '
          f'max {settle[-1]:.1f} s; events {dict(events)}')


if __name__ == '__main__':
    main()
