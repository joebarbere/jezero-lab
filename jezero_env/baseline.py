"""Hand-written go-to-goal controller: turn toward the goal, drive at full
speed, slow down when badly misaligned. The number to beat in Phase 4.
make_avoid_policy: experiment 11's rock-avoiding baseline (PLAN.md).

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


# -- Experiment 11: a rock-avoiding baseline (PLAN.md) -----------------------
# Uses only what the policy sees: the fine patch (always the last
# FINE_PATCH**2 values when rock_patch=True) and the goal bearing.

ARC_RADII = (0.45, 0.8, 1.5, 3.0)       # m, each both ways, plus straight
HALF_WIDTH = 0.25                       # m: rover half-width, checked either side of the arc
ARC_STEP = 0.1                          # m between checked points along an arc
ESCAPE_CLEAR = 0.35                     # m: no arc clear this far -> reverse
STALL_STEPS = 50                        # 10 s at 5 Hz without PROGRESS_M of progress -> escape
PROGRESS_M = 0.25                       # m of goal-distance improvement that counts
REVERSE_STEPS, SWERVE_STEPS = 10, 15    # escape: reverse 2 s, then 3 s on the other side


def _fine_grid():
    from jezero_env.env import FINE_PATCH, FINE_SPACING
    g = (np.arange(FINE_PATCH) - (FINE_PATCH - 1) / 2) * FINE_SPACING
    fwd, left = np.meshgrid(g, g)        # as JezeroEnv: grid[i, j] is (fwd=g[j], left=g[i])
    return FINE_PATCH, FINE_SPACING, fwd.ravel(), left.ravel()


UNKNOWN_BLOCK = 10.0                    # m: an unseen cell under unknown='block'


def rock_height(fine, h_rock=0.12, refits=2, known=None):
    """Height above a plane fitted to the fine patch, as a (left, fwd) grid.
    Robust: refit without the points standing more than h_rock above the last
    fit, so a large rock can't tilt the plane toward itself and hide.
    known (experiment 13): fit only the cells the camera has seen."""
    n, _, fwd, left = _fine_grid()
    a = np.column_stack([np.ones_like(fwd), fwd, left])
    keep = np.ones(fine.size, dtype=bool) if known is None else known.copy()
    if keep.sum() < 3:
        return np.zeros((n, n))
    for _ in range(refits + 1):
        coef, *_ = np.linalg.lstsq(a[keep], fine[keep], rcond=None)
        resid = fine - a @ coef
        new = resid <= h_rock
        if known is not None:
            new &= known
        if new.sum() < 3 or (new == keep).all():
            break
        keep = new
    return resid.reshape(n, n)


def _sample(grid, fwd, left):
    """Bilinear sample of a (left, fwd) fine grid at rover-frame points."""
    n, sp, _, _ = _fine_grid()
    j = np.clip(np.asarray(fwd) / sp + (n - 1) / 2, 0, n - 1)
    i = np.clip(np.asarray(left) / sp + (n - 1) / 2, 0, n - 1)
    i0, j0 = np.minimum(i.astype(int), n - 2), np.minimum(j.astype(int), n - 2)
    di, dj = i - i0, j - j0
    return ((1 - di) * (1 - dj) * grid[i0, j0] + (1 - di) * dj * grid[i0, j0 + 1]
            + di * (1 - dj) * grid[i0 + 1, j0] + di * dj * grid[i0 + 1, j0 + 1])


def arcs():
    """(curvature, length cap factor) for the candidate arcs: + is left."""
    return [0.0] + [k / r for r in ARC_RADII for k in (1, -1)]


def score_arc(rock, kappa, err, h_rock, length):
    """(clear distance, arc length, heading error at the arc's end) for one arc."""
    arc_len = length if kappa == 0 else min(length, abs(1 / kappa) * math.pi / 2)
    s = np.arange(ARC_STEP, arc_len + 1e-9, ARC_STEP)
    th = kappa * s
    if kappa == 0:
        x, y = s, np.zeros_like(s)
    else:
        x, y = np.sin(th) / kappa, (1 - np.cos(th)) / kappa
    clear = arc_len
    for off in (-HALF_WIDTH, 0.0, HALF_WIDTH):
        px, py = x - off * np.sin(th), y + off * np.cos(th)
        hit = np.nonzero(_sample(rock, px, py) > h_rock)[0]
        if hit.size:
            clear = min(clear, float(s[hit[0]]) - ARC_STEP)
    end_err = math.atan2(math.sin(err - kappa * arc_len), math.cos(err - kappa * arc_len))
    return max(clear, 0.0), arc_len, end_err


def make_avoid_policy(h_rock=0.12, length=1.6, k_block=3.0, unknown=None):
    """A fresh controller per episode (eval.evaluate builds one each time): its
    only state is the stall escape's progress tracker.
    unknown (experiment 13, sensing='camera'): 'free' or 'block' for cells the
    camera hasn't seen; the fine_known mask sits just before the fine patch."""
    if unknown not in (None, 'free', 'block'):
        raise ValueError(f'unknown: {unknown!r}')
    from jezero_env.env import FINE_PATCH, GOAL_RANGE, MAX_SPEED, MAX_TURN
    st = dict(best=math.inf, since=0, escape=0, side=1.0, last_kappa=0.0)

    def act(speed, kappa):
        # radius = linear_x / angular_z, so a curvature kappa needs turn = speed*MAX_SPEED*kappa/MAX_TURN
        turn = float(np.clip(abs(speed) * MAX_SPEED * kappa / MAX_TURN, -1, 1))
        return np.array([speed, turn], dtype=np.float32)

    def policy(obs):
        err = math.atan2(obs[3], obs[4])
        dist = float(obs[2]) * GOAL_RANGE
        nf = FINE_PATCH * FINE_PATCH
        fine = np.asarray(obs[-nf:], dtype=np.float64)
        if unknown is None:
            rock = rock_height(fine, h_rock)
        else:
            known = np.asarray(obs[-2 * nf:-nf]) > 0.5
            rock = rock_height(fine, h_rock, known=known)
            rock[~known.reshape(rock.shape)] = 0.0 if unknown == 'free' else UNKNOWN_BLOCK
        scored = []
        for kappa in arcs():
            clear, arc_len, end_err = score_arc(rock, kappa, err, h_rock, length)
            cost = abs(end_err) / math.pi + k_block * (1 - clear / arc_len)
            scored.append((cost, kappa, clear, arc_len))
        # Stall escape: no progress for STALL_STEPS -> reverse, then the clearest arc
        # turning the other way from the last one, then resume.
        if dist < st['best'] - PROGRESS_M:
            st['best'], st['since'] = dist, 0
        else:
            st['since'] += 1
        if st['escape'] == 0 and st['since'] >= STALL_STEPS:
            st['escape'], st['since'] = REVERSE_STEPS + SWERVE_STEPS, 0
            st['side'] = -1.0 if st['last_kappa'] > 0 else 1.0
        if st['escape'] > 0:
            st['escape'] -= 1
            if st['escape'] >= SWERVE_STEPS:
                return act(-0.5, 0.0)
            side = [c for c in scored if c[1] * st['side'] > 0]
            _, kappa, _, _ = max(side, key=lambda c: (c[2] / c[3], -abs(c[1])))
            return act(0.6, kappa)
        if max(c[2] for c in scored) < ESCAPE_CLEAR:
            return act(-0.5, 0.0)
        _, kappa, clear, arc_len = min(scored, key=lambda c: c[0])
        st['last_kappa'] = kappa
        speed = float(np.clip(math.cos(err), 0.2, 1.0)) * (1.0 if clear >= arc_len else 0.5)
        return act(speed, kappa)
    return policy


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
