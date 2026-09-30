"""Gymnasium environment: drive the OSR between waypoints on Perseverance's
route across the Jezero delta.

    from jezero_env.env import JezeroEnv
    env = JezeroEnv()                       # segment sol 437 -> 441
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Action (Box, [-1, 1]^2): [speed, turn], scaled to linear_x = speed * MAX_SPEED and
angular_z = turn * MAX_TURN, then through the rover's own kinematics (turning
radius clipped to 0.45-6.4 m, as on the hardware).

Observation (Box, float32, see OBS_LAYOUT): goal position in the rover frame,
heading error, tilt, body-frame velocities, previous action, and a patch of
terrain heights around the rover relative to its own height, sampled from the
same heightmap the physics collides with.

Reward, ported from the 2020 AWS-JPL challenge reward and extended:
  + PROGRESS_GAIN * (metres closer to the goal this step)
  + GOAL_BONUS * (fraction of the step budget left)  on reaching the goal
    (at_destination(): a bounding box around the goal, as in 2020)
  - TIME_COST every step
  - TIP_PENALTY and terminate if |roll| or |pitch| exceeds MAX_TILT
  - OUT_PENALTY and terminate if the rover leaves the map
  Truncated when the step budget (the 2020 "power supply") runs out.
"""
from __future__ import annotations

import math
import os

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from osgeo import gdal

from jezero_env.sim import OSR_GZ, SIM_WHEEL_RADIUS, JezeroSim

gdal.UseExceptions()

MAX_SPEED = 0.3        # m/s at action speed = 1
MAX_TURN = 0.6         # rad/s at action turn = 1
CONTROL_PERIOD = 0.2   # s of sim time per env step (5 Hz)
GOAL_PADDING = 1.0     # m: half-width of the at_destination() box
MAX_TILT = math.radians(35)
BUDGET_FACTOR = 2.5    # step budget = straight-line time at full speed x this
SPAWN_ATTEMPTS = 5
MIN_GOAL_DIST = 5.0    # m: jittered spawns stay at least this far from the goal
ROUTE_GOAL_DIST = (10.0, 60.0)   # m: start-goal distance range for goal_mode='route' and 'map'
# goal_mode='map': a region training never enters (x0, x1, y0, y1), around the
# unseen segment 3 (sol 455 -> 461): its goal and most of the route to it.
DEFAULT_HOLDOUT = (-100.0, -35.0, -18.0, 45.0)
MAP_MARGIN = 8.0                 # m: map-mode starts/goals stay this far inside the edge
GOAL_ROCK_CLEARANCE = 1.5        # m: random goals stay this far from a rock's edge
# 1.5 m leaves only 15% of route points eligible on k=0.05; with 2,000 tries and
# the waypoint fallback below that's enough (the ppo_rocks crash was 200 tries
# and no fallback). 1.0 m (tried in c0_control) put goals among rocks and made
# the task much harder: timeouts stalled a median 6.4 m short of the goal.

PROGRESS_GAIN = 1.0
GOAL_BONUS = 100.0
TIME_COST = 0.01
TIP_PENALTY = 50.0
STUCK_SPEED = 0.02     # m/s: below this a step counts toward stuck_s (diagnostics only)
STUCK_PENALTY = 50.0   # stuck_limit_s: same as tipping over. Must exceed the time cost a
                       # stall would otherwise accrue (<= ~40), or getting stuck on purpose
                       # would be a cheap way out of an episode.
OUT_PENALTY = 50.0

PATCH = 7              # terrain samples per side
PATCH_SPACING = 1.0    # m between samples
FINE_PATCH = 9         # rock_patch=True: finer patch that includes rocks
FINE_SPACING = 0.4     # m (±1.6 m around the rover)
ROCK_SPAWN_CLEARANCE = 0.8   # m between a spawn and a rock's edge
GOAL_RANGE = 50.0      # m: goal distance is scaled by this (then clipped)

OBS_LAYOUT = (
    ('goal_x', 1), ('goal_y', 1), ('goal_dist', 1),       # rover frame, / GOAL_RANGE
    ('heading_err_sin', 1), ('heading_err_cos', 1),
    ('roll', 1), ('pitch', 1),                              # rad
    ('v_forward', 1), ('v_left', 1), ('yaw_rate', 1),       # m/s, rad/s
    ('prev_action', 2),
    ('terrain', PATCH * PATCH),                              # m, relative to rover z
)
OBS_SIZE = sum(n for _, n in OBS_LAYOUT)
FINE_LAYOUT = (('rocks', FINE_PATCH * FINE_PATCH),)   # appended when rock_patch=True
# proprio=True, appended after the clock (if any) and before the fine patch.
# Hardware on a real OSR in brackets (see PLAN.md, sensor policy).
PROPRIO_LAYOUT = (
    ('wheel_speed', 6),       # m/s surface speed per wheel [RoboClaw encoders: present]
    ('slip', 1),              # mean wheel speed - body forward speed [encoders + IMU]
    ('roll_pitch_rate', 2),   # rad/s, body frame [IMU gyro]
    ('corner_angle', 4),      # rad, actual steering [feedback servos / corner encoders]
    ('bogie_angle', 2),       # rad [bogie pivot encoders]
    ('no_progress', 1),       # s since the goal distance last improved by PROGRESS_STEP, /30 [computed]
)
PROPRIO_SIZE = sum(n for _, n in PROPRIO_LAYOUT)
PROGRESS_STEP = 0.25   # m
# lookahead=True: terrain height profiles along rays fanned around the goal
# bearing, relative to the rover's height, appended after proprio (if any) and
# before the fine patch. [Hardware: depth camera + mapping; idealised here: the
# true terrain, all around, aimed at the goal rather than the camera's facing.]
LOOKAHEAD_ANGLES = (-40.0, -20.0, 0.0, 20.0, 40.0)    # deg from the goal bearing
LOOKAHEAD_RANGES = tuple(1.5 * i for i in range(1, 9))  # 1.5 .. 12 m
LOOKAHEAD_SIZE = len(LOOKAHEAD_ANGLES) * len(LOOKAHEAD_RANGES)
NO_PROGRESS_SCALE = 30.0  # s


class Heightmap:
    """The world's heightmap, in world coordinates, for observations."""

    def __init__(self, world: str, meta: dict):
        # Rock worlds reuse their base world's terrain model.
        model = meta.get('heightmap_model', world)
        png = os.path.join(OSR_GZ, 'models', model, 'materials', 'textures', 'heightmap.png')
        self.z = gdal.Open(png).ReadAsArray().astype(np.float64) / 65535.0 * meta['relief_m']
        self.n = self.z.shape[0]
        self.extent = float(meta['extent_m'])
        self.pixel = self.extent / (self.n - 1)

    def inside(self, x, y, margin=0.0):
        half = self.extent / 2 - margin
        return abs(x) <= half and abs(y) <= half

    def sample(self, x, y):
        """Bilinear height at world (x, y); arrays OK. Clamped at the edges."""
        col = np.clip((np.asarray(x) + self.extent / 2) / self.pixel, 0, self.n - 1.001)
        row = np.clip((self.extent / 2 - np.asarray(y)) / self.pixel, 0, self.n - 1.001)
        c0, r0 = col.astype(int), row.astype(int)
        fc, fr = col - c0, row - r0
        z = self.z
        return ((1 - fr) * ((1 - fc) * z[r0, c0] + fc * z[r0, c0 + 1])
                + fr * ((1 - fc) * z[r0 + 1, c0] + fc * z[r0 + 1, c0 + 1]))


class Rocks:
    """The world's rocks (from its yaml), for keeping spawns off them. The rocks
    themselves are baked into the heightmap (terrain/add_rocks.py)."""

    CELL = 4.0

    def __init__(self, rocks):
        self.xy = np.array([[r['x'], r['y']] for r in rocks], dtype=np.float64).reshape(-1, 2)
        self.r = np.array([r['d'] / 2 for r in rocks], dtype=np.float64)
        self.grid = {}
        for i, (x, y) in enumerate(self.xy):
            self.grid.setdefault((int(x // self.CELL), int(y // self.CELL)), []).append(i)

    def near(self, x, y, radius):
        cx, cy, k = int(x // self.CELL), int(y // self.CELL), int(radius // self.CELL) + 1
        idx = [i for gx in range(cx - k, cx + k + 1) for gy in range(cy - k, cy + k + 1)
               for i in self.grid.get((gx, gy), ())]
        return np.array(idx, dtype=int)

    def clearance(self, x, y):
        """Distance from (x, y) to the nearest rock's edge (inf if none near)."""
        idx = self.near(x, y, 8.0)
        if not idx.size:
            return math.inf
        return float(np.min(np.hypot(self.xy[idx, 0] - x, self.xy[idx, 1] - y) - self.r[idx]))


class JezeroEnv(gym.Env):
    """segments: indices into the world's waypoint list; segment i drives from
    waypoint i to waypoint i+1. One is picked per episode.
    random_heading: spawn facing a random direction instead of the goal.
    spawn_jitter: start up to this many metres from the segment's start waypoint
        (uniform over a disc), at least MIN_GOAL_DIST from the goal.
    goal_mode: 'segments' drives from waypoint i to i+1 of a chosen segment.
        'route' picks a random start and goal anywhere along the chosen
        segments' route, ROUTE_GOAL_DIST apart, either direction: many
        start/goal pairs instead of a few, so the policy has to generalise
        (a segments-trained policy reached 42% on an unseen segment).
        'map' picks them anywhere on the map, ROUTE_GOAL_DIST apart, never in
        or across the `holdout` box, so that region stays unseen for evaluation.
    holdout: (x0, x1, y0, y1) for goal_mode='map'; default DEFAULT_HOLDOUT.
    goal_bonus: 'decay' (default): GOAL_BONUS x the fraction of the budget left,
        so the reward depends on time the policy can't see. 'constant': the full
        GOAL_BONUS whenever the goal is reached, so the reward doesn't depend on
        a hidden clock (the per-step time cost still favours finishing sooner).
        Chosen over observing the clock, which hurt training (see PLAN.md).
    clock_obs: append the fraction of the step budget remaining. The goal bonus
        and truncation both depend on it, so without it the reward isn't a
        function of the observation. On for every run from c0_control on; off
        (the default) reproduces the observation older checkpoints were trained on.
    proprio: append proprioception (PROPRIO_LAYOUT): wheel speeds, slip,
        roll/pitch rates, steering and bogie angles, time since last progress.
    lookahead: append terrain height profiles out to 12 m along rays around the
        goal bearing (LOOKAHEAD_*): scarps and rocks before the rover reaches them.
    stuck_limit_s: training-only shaping. End the episode with -STUCK_PENALTY
        when the goal distance hasn't improved by PROGRESS_STEP for this long.
        Evaluation leaves it off (eval builds envs from observation options only).
    rock_patch: append a 9x9 patch at 0.4 m of terrain heights (relative to the
        rover). On rock worlds the heightmap includes the rocks, so this is how
        the policy sees rocks the 1 m patch misses.
    max_episode_seconds: cap on episode sim time (in addition to the budget).
    """

    metadata = {'render_modes': []}

    def __init__(self, world: str = 'jezero_delta', segments=(0,), random_heading: bool = False,
                 spawn_jitter: float = 0.0, goal_mode: str = 'segments', rock_patch: bool = False,
                 clock_obs: bool = False, goal_bonus: str = 'decay', holdout=None,
                 proprio: bool = False, lookahead: bool = False, stuck_limit_s=None,
                 max_episode_seconds: float | None = None, step_size: float = 0.005,
                 camera: bool = False):
        super().__init__()
        self.sim = JezeroSim(world, step_size=step_size, camera=camera, report_joints=proprio)
        self.proprio = proprio
        self.lookahead = lookahead
        self.stuck_limit_s = stuck_limit_s
        self.waypoints = self.sim.meta.get('waypoints')
        if not self.waypoints or len(self.waypoints) < 2:
            raise ValueError(f'world {world!r} has no waypoint list in its .yaml')
        self.segments = tuple(segments)
        self.random_heading = random_heading
        self.spawn_jitter = spawn_jitter
        if goal_mode not in ('segments', 'route', 'map'):
            raise ValueError(f'goal_mode: {goal_mode!r}')
        self.goal_mode = goal_mode
        self.holdout = tuple(holdout) if holdout is not None else DEFAULT_HOLDOUT
        self.max_episode_seconds = max_episode_seconds
        self.heightmap = Heightmap(world, self.sim.meta)
        self.rocks = Rocks(self.sim.meta.get('rocks', []))
        self.rock_patch = rock_patch
        self.clock_obs = clock_obs
        if goal_bonus not in ('decay', 'constant'):
            raise ValueError(f'goal_bonus: {goal_bonus!r}')
        self.goal_bonus = goal_bonus
        self.physics_steps = int(round(CONTROL_PERIOD / step_size))

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        size = (OBS_SIZE + (1 if clock_obs else 0) + (PROPRIO_SIZE if proprio else 0)
                + (LOOKAHEAD_SIZE if lookahead else 0)
                + (FINE_PATCH * FINE_PATCH if rock_patch else 0))
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(size,), dtype=np.float32)

        g = np.arange(PATCH) - (PATCH - 1) / 2
        self._patch_fwd, self._patch_left = [a.ravel() * PATCH_SPACING for a in np.meshgrid(g, g)]
        g = np.arange(FINE_PATCH) - (FINE_PATCH - 1) / 2
        self._fine_fwd, self._fine_left = [a.ravel() * FINE_SPACING for a in np.meshgrid(g, g)]

    # -- helpers ------------------------------------------------------------

    def _route_point(self, u):
        """Point at fraction u along the route through the chosen segments."""
        pts = [self.waypoints[i] for i in self.segments] + [self.waypoints[self.segments[-1] + 1]]
        legs = [math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(pts, pts[1:])]
        d = u * sum(legs)
        for a, b, L in zip(pts, pts[1:], legs):
            if d <= L:
                t = d / L
                return {'x': a['x'] + t * (b['x'] - a['x']), 'y': a['y'] + t * (b['y'] - a['y'])}
            d -= L
        return {'x': pts[-1]['x'], 'y': pts[-1]['y']}

    def _route_pair(self):
        """Random start/goal along the route. Falls back to a segment's own
        waypoints (always rock-free) rather than failing: an exception here
        kills a SubprocVecEnv worker and hangs training."""
        lo, hi = ROUTE_GOAL_DIST
        for _ in range(2000):
            a = self._route_point(self.np_random.uniform())
            b = self._route_point(self.np_random.uniform())
            if (lo <= math.hypot(b['x'] - a['x'], b['y'] - a['y']) <= hi
                    and self.rocks.clearance(b['x'], b['y']) >= GOAL_ROCK_CLEARANCE
                    and self.rocks.clearance(a['x'], a['y']) >= ROCK_SPAWN_CLEARANCE):
                return a, b
        seg = int(self.np_random.choice(self.segments))
        return self.waypoints[seg], self.waypoints[seg + 1]

    def in_holdout(self, x, y):
        x0, x1, y0, y1 = self.holdout
        return x0 <= x <= x1 and y0 <= y <= y1

    def _map_pair(self):
        """Random start and goal anywhere on the map, ROUTE_GOAL_DIST apart,
        clear of rocks, with neither end in the holdout box and the straight
        line between them not crossing it. Falls back to a route pair."""
        lo, hi = ROUTE_GOAL_DIST
        half = self.heightmap.extent / 2 - MAP_MARGIN
        for _ in range(5000):
            ax, ay, bx, by = self.np_random.uniform(-half, half, 4)
            d = math.hypot(bx - ax, by - ay)
            if not lo <= d <= hi:
                continue
            if any(self.in_holdout(ax + t * (bx - ax), ay + t * (by - ay))
                   for t in np.linspace(0.0, 1.0, int(d) + 2)):
                continue
            if (self.rocks.clearance(bx, by) >= GOAL_ROCK_CLEARANCE
                    and self.rocks.clearance(ax, ay) >= ROCK_SPAWN_CLEARANCE):
                return {'x': float(ax), 'y': float(ay)}, {'x': float(bx), 'y': float(by)}
        return self._route_pair()

    def _spawn_xy(self, start, goal):
        if self.spawn_jitter <= 0:
            return start['x'], start['y']
        for _ in range(100):
            r = self.spawn_jitter * math.sqrt(self.np_random.uniform())
            th = self.np_random.uniform(-math.pi, math.pi)
            x, y = start['x'] + r * math.cos(th), start['y'] + r * math.sin(th)
            if (self.heightmap.inside(x, y, margin=5.0)
                    and math.hypot(goal['x'] - x, goal['y'] - y) >= MIN_GOAL_DIST
                    and self.rocks.clearance(x, y) >= ROCK_SPAWN_CLEARANCE
                    and not (self.goal_mode == 'map' and self.in_holdout(x, y))):
                return x, y
        return start['x'], start['y']

    def _episode_stats(self, ep, event, dist, s):
        """Summary of a finished episode, for logging (see train/train.py)."""
        start = ep['start_dist_m']
        stats = {k: v for k, v in ep.items() if not k.startswith('_')}
        stats.update(
            event=event, steps=self.steps, sim_time_s=self.steps * CONTROL_PERIOD,
            final_dist_m=dist,
            progress_frac=(start - dist) / start if start > 0 else 0.0,
            # straight-line distance / distance driven, for episodes that got there
            path_efficiency=(start / ep['path_m']) if event == 'goal' and ep['path_m'] > 0 else float('nan'),
            steer_change_per_s=ep['steer_change'] / max(self.steps * CONTROL_PERIOD, 1e-9),
            end_x=s.x, end_y=s.y, segment=self.segment)
        return stats

    def _at_destination(self, s):
        return abs(s.x - self.goal[0]) <= GOAL_PADDING and abs(s.y - self.goal[1]) <= GOAL_PADDING

    def _goal_dist(self, s):
        return math.hypot(self.goal[0] - s.x, self.goal[1] - s.y)

    def _obs(self, s):
        c, si = math.cos(s.yaw), math.sin(s.yaw)
        dx, dy = self.goal[0] - s.x, self.goal[1] - s.y
        gx, gy = c * dx + si * dy, -si * dx + c * dy          # goal in rover frame
        dist = math.hypot(dx, dy)
        err = math.atan2(gy, gx)
        v_fwd, v_left = c * s.vx + si * s.vy, -si * s.vx + c * s.vy
        # Terrain patch in the rover's heading frame, relative to the rover's height.
        px = s.x + c * self._patch_fwd - si * self._patch_left
        py = s.y + si * self._patch_fwd + c * self._patch_left
        ground = self.heightmap.sample(s.x, s.y)
        terrain = self.heightmap.sample(px, py) - ground
        parts = []
        if self.clock_obs:
            parts.append([1.0 - self.steps / self.budget])
        if self.proprio:
            wheel = [w * SIM_WHEEL_RADIUS for w in s.wheel_vel]
            roll_rate = c * s.wx + si * s.wy          # world -> heading frame (small tilt)
            pitch_rate = -si * s.wx + c * s.wy
            parts.append(wheel + [sum(wheel) / len(wheel) - v_fwd, roll_rate, pitch_rate]
                         + list(s.corner_pos) + list(s.bogie_pos)
                         + [min(self._since_progress, NO_PROGRESS_SCALE) / NO_PROGRESS_SCALE])
        if self.lookahead:
            bearing = math.atan2(dy, dx)
            th = bearing + np.radians(np.array(LOOKAHEAD_ANGLES))[:, None]
            r = np.array(LOOKAHEAD_RANGES)[None, :]
            lx, ly = s.x + r * np.cos(th), s.y + r * np.sin(th)
            parts.append((self.heightmap.sample(lx.ravel(), ly.ravel()) - ground).tolist())
        if self.rock_patch:
            fx = s.x + c * self._fine_fwd - si * self._fine_left
            fy = s.y + si * self._fine_fwd + c * self._fine_left
            # Rock worlds bake rocks into the (12.5 cm) heightmap, so this sees them.
            parts.append(self.heightmap.sample(fx, fy) - ground)
        obs = np.concatenate([
            [gx / GOAL_RANGE, gy / GOAL_RANGE, min(dist / GOAL_RANGE, 3.0)],
            [math.sin(err), math.cos(err)],
            [s.roll, s.pitch],
            [v_fwd, v_left, s.wz],
            self._prev_action,
            terrain,
            *parts,
        ]).astype(np.float32)
        return obs

    # -- gym API ------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        seg = int(self.np_random.choice(self.segments))
        if self.goal_mode == 'route':
            start, goal = self._route_pair()
        elif self.goal_mode == 'map':
            start, goal = self._map_pair()
            seg = -1
        else:
            start, goal = self.waypoints[seg], self.waypoints[seg + 1]
        self.segment = seg
        self.goal = (goal['x'], goal['y'])
        # Spawn: the start waypoint, optionally jittered and/or facing a random
        # way. A rare spawn (across a steep spot) never settles; resample rather
        # than start an episode that is already tipped.
        for attempt in range(SPAWN_ATTEMPTS):
            x, y = self._spawn_xy(start, goal)
            if self.random_heading or attempt > 0:
                yaw = float(self.np_random.uniform(-math.pi, math.pi))
            else:
                yaw = math.atan2(goal['y'] - y, goal['x'] - x)
            z = float(self.heightmap.sample(x, y))
            s = self.sim.reset(spawn=(x, y, z, yaw))
            if abs(s.roll) < MAX_TILT and abs(s.pitch) < MAX_TILT:
                break
        else:
            raise RuntimeError(f'segment {seg}: no stable spawn in {SPAWN_ATTEMPTS} tries')
        self.spawn_attempts = attempt + 1

        straight = math.hypot(goal['x'] - s.x, goal['y'] - s.y)
        self.budget = int(math.ceil(straight / MAX_SPEED * BUDGET_FACTOR / CONTROL_PERIOD))
        if self.max_episode_seconds is not None:
            self.budget = min(self.budget, int(self.max_episode_seconds / CONTROL_PERIOD))
        self.steps = 0
        self.t0 = s.sim_time
        self._ep = {'stuck_s': 0.0, 'path_m': 0.0, 'max_tilt_deg': 0.0, 'min_dist_m': math.inf,
                    'steer_change': 0.0, '_x': s.x, '_y': s.y, '_turn': 0.0,
                    'start_dist_m': math.hypot(self.goal[0] - s.x, self.goal[1] - s.y)}
        self._prev_action = np.zeros(2)
        self._prev_dist = self._goal_dist(s)
        self._best_dist = self._prev_dist
        self._since_progress = 0.0
        return self._obs(s), {'segment': seg, 'goal': self.goal, 'budget': self.budget,
                              'distance': self._prev_dist, 'spawn_attempts': self.spawn_attempts,
                              'settle_time': self.sim.settle_time}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self.sim.command(linear_x=float(a[0]) * MAX_SPEED, angular_z=float(a[1]) * MAX_TURN)
        s = self.sim.step(self.physics_steps)
        self.steps += 1
        self._prev_action = a

        dist = self._goal_dist(s)
        # Reward as named components (summed below), so each term's share can be
        # logged per episode and a change to one term measured on its own.
        parts = {'progress': PROGRESS_GAIN * (self._prev_dist - dist), 'time': -TIME_COST}
        if dist < self._best_dist - PROGRESS_STEP:
            self._best_dist, self._since_progress = dist, 0.0
        else:
            self._since_progress += CONTROL_PERIOD
        self._prev_dist = dist
        terminated, event = False, None
        if self._at_destination(s):
            parts['goal'] = (GOAL_BONUS if self.goal_bonus == 'constant'
                             else GOAL_BONUS * (1.0 - self.steps / self.budget))
            terminated, event = True, 'goal'
        elif abs(s.roll) > MAX_TILT or abs(s.pitch) > MAX_TILT:
            parts['tipped'] = -TIP_PENALTY
            terminated, event = True, 'tipped'
        elif not self.heightmap.inside(s.x, s.y, margin=1.0):
            parts['out_of_bounds'] = -OUT_PENALTY
            terminated, event = True, 'out_of_bounds'
        elif self.stuck_limit_s is not None and self._since_progress >= self.stuck_limit_s:
            parts['stuck'] = -STUCK_PENALTY
            terminated, event = True, 'stuck'
        truncated = not terminated and self.steps >= self.budget
        if truncated:
            event = 'budget'
        reward = sum(parts.values())

        # Episode diagnostics.
        ep = self._ep
        for k, v in parts.items():
            ep['reward_' + k] = ep.get('reward_' + k, 0.0) + v
        speed = math.hypot(s.vx, s.vy)
        ep['stuck_s'] += CONTROL_PERIOD if speed < STUCK_SPEED else 0.0
        ep['path_m'] += math.hypot(s.x - ep['_x'], s.y - ep['_y'])
        ep['_x'], ep['_y'] = s.x, s.y
        ep['max_tilt_deg'] = max(ep['max_tilt_deg'], math.degrees(max(abs(s.roll), abs(s.pitch))))
        ep['min_dist_m'] = min(ep['min_dist_m'], dist)
        ep['steer_change'] += abs(a[1] - ep['_turn'])
        ep['_turn'] = a[1]

        info = {'distance': dist, 'event': event, 'sim_time': self.steps * CONTROL_PERIOD,   # exact; absolute sim time differs across resets
                'x': s.x, 'y': s.y}
        if terminated or truncated:
            info['episode_stats'] = self._episode_stats(ep, event, dist, s)
        return self._obs(s), float(reward), terminated, truncated, info
