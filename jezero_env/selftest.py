"""Spike checks for JezeroSim: speed parity with the ROS stack, throughput,
and whether reset is clean and deterministic.  jezero_env/run.sh python3 -m jezero_env.selftest"""
import math
import time

from jezero_env.sim import JezeroSim


def drive(sim, seconds, **cmd):
    sim.command(**cmd)
    return sim.step(int(round(seconds / sim.step_size)))


def main():
    t = time.time()
    sim = JezeroSim('empty', step_size=0.005)
    print(f'startup {time.time() - t:.1f} s; spawned at z={sim.state().z:.3f}')

    drive(sim, 1.0)                            # settle
    s0 = drive(sim, 2.0, linear_x=0.3)          # accelerate
    w = time.time()
    s1 = drive(sim, 5.0, linear_x=0.3)
    wall = time.time() - w
    speed = math.hypot(s1.x - s0.x, s1.y - s0.y) / (s1.sim_time - s0.sim_time)
    print(f'forward: {speed:.3f} m/s for 0.3 commanded; RTF {5.0 / wall:.1f}')

    drive(sim, 1.0)
    y0 = sim.state().yaw
    yaw, prev = 0.0, y0
    for _ in range(40):                         # 4 s in 0.1 s chunks to unwrap
        s = drive(sim, 0.1, angular_y=0.5)
        yaw += math.atan2(math.sin(s.yaw - prev), math.cos(s.yaw - prev))
        prev = s.yaw
    print(f'rotate in place: {math.degrees(yaw):+.1f} deg / 4 s')

    # Reset must be clean: after *different* histories, the same episode must
    # play out the same. (The 2020 challenge env's reset left joint state behind.)
    histories = {
        'drove forward': dict(linear_x=0.3),
        'spinning': dict(angular_y=0.5),
        'reversing + turning': dict(linear_x=-0.2, angular_z=0.4),
    }
    ends = []
    for label, cmd in histories.items():
        drive(sim, 3.0, **cmd)                  # leave the rover mid-manoeuvre
        w = time.time()
        r = sim.reset()
        reset_s = time.time() - w
        drive(sim, 3.0, linear_x=0.3, angular_z=0.3)
        e = drive(sim, 3.0, linear_x=-0.2)
        ends.append((e.x - r.x, e.y - r.y, e.yaw - r.yaw))
        print(f'after {label:20s}: reset in {reset_s * 1000:4.0f} ms to ({r.x:+.4f}, {r.y:+.4f}, '
              f'{r.z:+.4f}, yaw {r.yaw:+.4f}); episode moved ({ends[-1][0]:+.4f}, '
              f'{ends[-1][1]:+.4f}, yaw {ends[-1][2]:+.4f})')
    spread = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in ends for b in ends)
    yaw_spread = max(abs(a[2] - b[2]) for a in ends for b in ends)
    print(f'reset determinism: episode spread {spread * 1000:.3f} mm, '
          f'{math.degrees(yaw_spread):.4f} deg')

if __name__ == '__main__':
    main()
