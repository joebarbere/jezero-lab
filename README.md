# jezero-lab

Local reinforcement-learning lab for the NASA-JPL Open Source Rover v4, driving a
simulated patch of Jezero Crater. Runs entirely on one machine in Podman: no cloud,
no NVIDIA GPU required.

See [PLAN.md](PLAN.md) for the phases and every measured result, and
[docs/decision_log.md](docs/decision_log.md) for every training decision with the
numbers behind it and an explainer for RL newcomers. All five phases
are done: the OSR runs on ROS 2 Jazzy + Gazebo Harmonic, on 256 m of real Jezero
delta terrain from HiRISE, with waypoints on Perseverance's actual route and
Mars-realistic boulder fields; a Gymnasium environment steps it deterministically
in-process at ~15× real time; PPO policies are trained and evaluated against a
hand-written baseline, with path plots and chase-camera video.

**Where it stands (2026-10):** the learned policy **beats the hand-written baseline on
every test set**. It uses a small CNN over the terrain grids, trains on six sites along
Perseverance's route, and is chosen from six seeds on a separate validation set. On
held-out starts it reaches the goal 88% of the time on the delta among boulders (baseline
50%) and 88–98% on the hard starts at two sites it never trained on (baseline 71–73%),
tipping over in 0–4% of episodes vs up to 41% for the baseline. Seeds still vary a lot
(56–84%), which is why the seed is chosen on validation. The path there, including the
ideas that didn't work, is in [docs/decision_log.md](docs/decision_log.md).

Short version with results, a pipeline diagram and next steps: [docs/writeup.md](docs/writeup.md).

![Baseline (left) tips over on the rocks after 31 s; the CNN policy (right) reaches the goal](docs/media/cnn_hard26_frame.png)

## Setup

```bash
git clone --recurse-submodules https://github.com/joebarbere/jezero-lab
cd jezero-lab
containers/build.sh jazzy-harmonic     # main image
containers/build.sh humble-classic     # optional: Phase 0 reference (Gazebo Classic)
```

Each image is ~4–5 GB.

## Drive the rover

```bash
containers/run.sh jazzy-harmonic
# inside the container:
ros2 launch osr_gz sim.launch.py world:=jezero_delta   # or no world: flat ground
```

On `jezero_delta` the rover spawns at Perseverance's sol 437 position facing the
sol 441 waypoint; the blue posts mark sols 437, 441, 448, 455, and 461.

In a second terminal:

```bash
podman exec -it jezero-lab bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Rotate-in-place uses `Twist.angular.y`, which `teleop_twist_keyboard` can't send:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {y: 0.5}}"
```

### Launch options

| Option | Default | |
|---|---|---|
| `sim` | from `ROS_DISTRO` | `harmonic` (Jazzy) or `classic` (Humble) |
| `world` | empty | `jezero_delta` (Harmonic only), a name in `osr_gz/worlds`, or a path |
| `collision` | `primitive` | `primitive`: cylinder wheels + box body. `mesh`: upstream's full STLs |
| `wheel_mu` | `0.7` | wheel–ground friction coefficient |
| `gui` | `true` | `false` for headless |
| `realtime` | `true` | `false`: step as fast as the CPU allows |
| `step` | `0.005` | physics step for `realtime:=false`; RTF scales ~linearly with it |

For training-style throughput:
`ros2 launch osr_gz sim.launch.py world:=jezero_delta gui:=false realtime:=false`.

### Benchmark

```bash
containers/bench.sh jazzy-harmonic osr_gz sim.launch.py gui:=false realtime:=false
# == jazzy-harmonic: ...
# rtf=14.61 speed=0.298m/s(cmd 0.3) rotate: dyaw=-112.1deg/4s drift=7.3cm
```

Starts a headless container, waits for the controllers, then measures real-time
factor, forward speed, and rotate-in-place against sim time
([tools/bench.py](tools/bench.py)). [tools/drive_test.py](tools/drive_test.py) drives
straight on whatever world is running and logs position and tilt.

### Talk to the rover (ROSA)

[rosa/](rosa/README.md) connects NASA-JPL's [ROSA](https://github.com/nasa-jpl/rosa)
LLM agent, with Claude, to the simulated rover: "drive to the nearest waypoint",
"which controllers are running?". `ANTHROPIC_API_KEY=... rosa/run.sh --gui`.

## Gym environment

```python
from jezero_env.env import JezeroEnv
env = JezeroEnv(segments=(0, 1, 2, 3), random_heading=False)
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

Drives the OSR between waypoints on Perseverance's route. Action `[speed, turn]` in
[−1, 1]; observation of 61 floats (goal in rover frame, tilt, velocities, previous
action, 7×7 terrain patch). Reward and termination are documented in
[jezero_env/env.py](jezero_env/env.py).

No ROS in the loop: the gz-sim server runs inside the Python process and steps
exactly as many physics steps as each action asks for
([jezero_env/sim.py](jezero_env/sim.py)). It needs the system Gazebo libraries, so
run it through `run.sh` inside the Jazzy image:

```bash
podman run --rm -v "$PWD:/repo:z" -w /repo jezero-lab:jazzy-harmonic \
    jezero_env/run.sh python3 -m jezero_env.baseline      # hand-written controller
# also: jezero_env.selftest (ROS parity, reset determinism), jezero_env.soak
```

## Training and evaluation

```bash
R="podman run --rm -v $PWD:/repo:z -w /repo jezero-lab:jazzy-harmonic jezero_env/run.sh"
# Best configuration (train several seeds and choose one on validation spawns:
# eval.compare --seed0 2000000 + eval.select_checkpoint): terrain CNN on six sites (terrain/build_sites.sh builds the
# extra five; one worker each), goals anywhere on the map, rocks everywhere,
# 400k steps at a constant learning rate, then 1.1M more with it decaying to 0.
SITES=$(printf 'jezero_%s_rocks_k05_full ' delta floor seitah upper margin rimwest)
W="--world $SITES --rock-patch --goal-mode map --policy cnn \
   --segments 0 1 2 --random-heading --spawn-jitter 10 --envs 6 --n-steps 1024 --batch-size 1024"
$R python3 -m train.train --name cnn $W --steps 400000
$R python3 -m train.train --name cnn_full $W --steps 1100000 --lr-schedule linear \
    --resume runs/cnn/checkpoints/ppo_400000_steps.zip
$R python3 -m eval.compare --world jezero_delta_rocks_k05_full --arm cnn runs/cnn_full \
    --episodes 48 --workers 12 --out runs/compare_cnn.json
WORLD=jezero_delta_rocks_k05_full eval/make_demo.sh runs/cnn_full/final.zip \
    runs/compare_cnn.json hard:26 unseen:21   # needs a display
```

`train.train` writes TensorBoard logs and checkpoints to `runs/<name>/`
(`python3 tools/tb_summary.py runs/<name>` prints the curves).

### Observability

- **TensorBoard** (`runs/` holds every run; never delete one, name a new one):
  ```bash
  podman run -d --name jz-tensorboard -p 127.0.0.1:6006:6006 -v "$PWD/runs:/runs:ro,z" \
      jezero-lab:jazzy-harmonic tensorboard --logdir /runs --bind_all --reload_interval 30
  ```
  Training logs `outcome/*` (goal / tipped / budget / out-of-bounds rates),
  `reward/*` (each reward component's per-episode sum, so a change to one term is
  measurable on its own) and `episode/*` (progress fraction, closest approach,
  stuck time, peak tilt, path efficiency, steering churn), over the last 100
  episodes; plus the run's git revision and config as text.
- **Held-out evaluation during training**: `eval.watch runs/<name> --follow`
  evaluates each checkpoint on the held-out *hard* and *unseen* spawns in spare
  processes and logs them as the run `<name>/eval`, with the baseline's score on
  the same spawns as a reference line. Without `--follow` it backfills an existing
  run (`--every 400000`).
- **Where episodes end**: every episode goes to `runs/<name>/episodes.csv`;
  `eval.heatmap runs/<name> --from-steps 1000000` plots end points by outcome over
  the terrain. `eval.evaluate`
runs the policy and the baseline on identical held-out spawns; `eval.plot_paths`
draws both paths per spawn; `eval/make_demo.sh` records side-by-side chase-camera
videos (rendered on the host GPU, so it runs the containers with the display).

![Unseen segment: baseline (blue) vs policy (orange) from identical spawns](docs/media/paths_unseen.png)

## Terrain

`terrain/build_jezero_delta.sh` rebuilds the `jezero_delta` world from source: a
257 m window of the USGS Mars 2020 HiRISE DTM mosaic (1 m/px, ~22 MB read over
HTTP, not the 1.8 GB file) and NASA's Perseverance waypoint file. The generated
heightmap, world, and waypoint yaml are committed, so this is only needed to change
the area. `terrain/build_jezero_rim.sh` does the same for Perseverance's crater-rim
climb, and `terrain/add_rocks.py` adds a Golombek–Rapp boulder field to a world,
baked into a 12.5 cm heightmap (`jezero_delta_rocks_k03/k05/k08`).

```bash
podman run --rm -v "$PWD:/repo:z" -w /repo jezero-lab:jazzy-harmonic \
    terrain/build_jezero_delta.sh
```

## Layout

- `ros_ws/src/osr-rover-code`: submodule of
  [nasa-jpl/osr-rover-code](https://github.com/nasa-jpl/osr-rover-code), pinned,
  never edited. Provides `osr_control` (rover node + kinematics shared with the real
  hardware), `osr_interfaces`, `osr_bringup`, and upstream's Classic-only `osr_gazebo`.
- `ros_ws/src/osr_gz`: the simulation package: URDF with primitive/mesh collision,
  Classic and Harmonic plugins, launch file, worlds, and the generated
  `jezero_delta` terrain model. Derived from upstream `osr_gazebo`; meshes are a
  symlink into the submodule.
- `containers/`: Containerfiles for both images, `build.sh`, `run.sh`, `bench.sh`.
- `jezero_env/`: Gymnasium env, in-process simulator wrapper, baseline controller,
  self-test, soak test.
- `train/`: PPO training. `eval/`: evaluation, path plots, video recording.
- `docs/media/`: plots and stills used in this README.
- `terrain/`: DTM crop, heightmap/world generator, rebuild script.
- `tools/`: benchmark, drive test, TensorBoard summary, training profiler.

`run.sh` passes the host `DISPLAY`, the X11 socket, the XWayland auth cookie, and
`/dev/dri`, so Gazebo renders on the host GPU through Mesa (verified on an AMD
RX 7600: `GFX1102`, hardware OpenGL 4.6).

## License

Apache-2.0, matching upstream. `osr_gz` is derived from `osr-rover-code`
(Copyright 2018 California Institute of Technology); see [NOTICE](NOTICE).
